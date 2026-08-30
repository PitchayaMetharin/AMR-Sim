#!/usr/bin/env python3
"""Prepare the native Gate 6 attachments before the first physics step.

The factory world deliberately starts its ``DetachableJoint`` systems attached
when ``factory_attachment`` is enabled.  This node is the single startup
component that is allowed to turn that mode into a usable test fixture: it
queues a detach for every registered product, advances Gazebo one bounded
step at a time, restores the registered shelf poses while the world is
paused, and performs one bounded live sensor-validation window before
releasing the world for the test graph.

All deadlines in this file are wall-clock deadlines.  The short post-release
check is bounded by Gazebo simulation time as well as a wall-clock timeout so
that a stalled clock cannot be mistaken for a stable fixture.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from typing import Callable, Dict, Iterable, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
import yaml


PRODUCT_IDS = (101, 102, 103)
STOW = {
    "arm_joint_1": 0.0,
    "arm_joint_2": -1.5708,
    "arm_joint_3": 1.5708,
    "arm_joint_4": 0.0,
    "arm_joint_5": 0.0,
    "arm_joint_6": 0.0,
}
STOW_TOLERANCE_RAD = 0.01
PRODUCT_POSITION_TOLERANCE_M = 0.005
PRODUCT_YAW_TOLERANCE_RAD = 0.01
ROBOT_POSITION_TOLERANCE_M = 0.005
ROBOT_YAW_TOLERANCE_RAD = 0.02
PRODUCT_DRIFT_TOLERANCE_M = 0.005
OBSERVATION_SIM_SECONDS = 0.5
# Allow the registered products to absorb the first paused-to-running contact
# impulse before enforcing the strict shelf-pose observation window.  This is
# simulation-time bounded; it is not a wall-clock sleep or a tolerance change.
PRODUCT_SETTLE_SIM_SECONDS = 0.25
OBSERVATION_WALL_TIMEOUT_S = 15.0
STEP_CLOCK_TIMEOUT_S = 3.0
STEP_CLOCK_EPSILON_S = 1e-9
POST_RELEASE_CONFIRM_SIM_SECONDS = 0.25
POST_RELEASE_WALL_TIMEOUT_S = 5.0
MIN_POST_RELEASE_JOINT_STAMPS = 2


class BootstrapError(RuntimeError):
    """A fail-closed fixture preparation error."""


@dataclass(frozen=True)
class Product:
    product_id: int
    model: str
    mass_kg: float
    pose: Tuple[float, float, float, float]


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise BootstrapError(f"duplicate YAML key: {key}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping)


def _finite_values(values: Iterable[object], label: str) -> Tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise BootstrapError(f"{label} contains a non-numeric value") from error
    if not result or not all(math.isfinite(value) for value in result):
        raise BootstrapError(f"{label} contains a non-finite value")
    return result


def _angle_error(actual: float, expected: float) -> float:
    return abs(math.remainder(actual - expected, 2.0 * math.pi))


def _yaw_from_quaternion(quaternion) -> float:
    values = _finite_values(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        "pose quaternion",
    )
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise BootstrapError("pose quaternion is invalid")
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _pose_error(actual: PoseStamped, expected: Tuple[float, float, float, float]) -> Tuple[float, float]:
    position = _finite_values(
        (actual.pose.position.x, actual.pose.position.y, actual.pose.position.z),
        "observed pose position",
    )
    position_error = math.sqrt(sum(
        (position[index] - expected[index]) ** 2 for index in range(3)
    ))
    yaw_error = _angle_error(_yaw_from_quaternion(actual.pose.orientation), expected[3])
    if not math.isfinite(position_error) or not math.isfinite(yaw_error):
        raise BootstrapError("observed pose error is non-finite")
    return position_error, yaw_error


def _load_products(factory_directory: Path) -> Dict[int, Product]:
    products_path = factory_directory / "config" / "products.yaml"
    sdf_path = factory_directory / "worlds" / "factory.sdf"
    try:
        with products_path.open(encoding="utf-8") as stream:
            registry = yaml.load(stream, Loader=_UniqueKeyLoader)
        root = ET.parse(sdf_path).getroot()
    except (OSError, ET.ParseError, yaml.YAMLError) as error:
        raise BootstrapError("factory products registry or SDF is unavailable") from error
    if not isinstance(registry, dict) or not isinstance(registry.get("products"), dict):
        raise BootstrapError("products registry must contain a mapping")
    world = root.find("world")
    if world is None or world.attrib.get("name") != "factory_world":
        raise BootstrapError("factory SDF world is not factory_world")
    sdf_models = {}
    for model in world.findall("model"):
        name = model.attrib.get("name", "")
        pose = model.find("pose")
        if name and pose is not None and pose.text:
            values = _finite_values(pose.text.split(), f"factory SDF pose for {name}")
            if len(values) != 6:
                raise BootstrapError(f"factory SDF pose for {name} must contain six values")
            sdf_models[name] = (values[0], values[1], values[2], values[5])

    result: Dict[int, Product] = {}
    for model_name, raw_entry in registry["products"].items():
        if not isinstance(model_name, str) or not model_name:
            raise BootstrapError("product model name is invalid")
        if not isinstance(raw_entry, dict):
            raise BootstrapError(f"registry entry for {model_name} is invalid")
        try:
            product_id = int(raw_entry["tag_id"])
            mass_kg = float(raw_entry["mass"])
        except (KeyError, TypeError, ValueError) as error:
            raise BootstrapError(f"registry entry for {model_name} is incomplete") from error
        if product_id not in PRODUCT_IDS:
            raise BootstrapError(f"unsupported product ID {product_id}")
        if product_id in result:
            raise BootstrapError(f"duplicate product ID {product_id}")
        if not math.isfinite(mass_kg) or mass_kg < 0.0:
            raise BootstrapError(f"product {product_id} mass is invalid")
        if model_name not in sdf_models:
            raise BootstrapError(f"factory SDF has no product model {model_name}")
        result[product_id] = Product(product_id, model_name, mass_kg, sdf_models[model_name])
    if set(result) != set(PRODUCT_IDS):
        raise BootstrapError("registry must contain exactly products 101, 102, and 103")
    if len({item.model for item in result.values()}) != len(result):
        raise BootstrapError("product model names must be unique")
    return result


class AttachmentBootstrap(Node):
    """Run the paused-world native attachment bootstrap and expose its proof."""

    def __init__(self) -> None:
        super().__init__("gate6_attachment_bootstrap")
        # Launch may auto-declare this parameter from its override. Keep the
        # standalone default without redeclaring an existing parameter.
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("startup_timeout_sec", 20.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        try:
            self._startup_timeout = float(self.get_parameter("startup_timeout_sec").value)
            initial_x = float(self.get_parameter("initial_x").value)
            initial_y = float(self.get_parameter("initial_y").value)
            initial_yaw = float(self.get_parameter("initial_yaw").value)
        except (TypeError, ValueError) as error:
            raise BootstrapError("startup_timeout_sec is invalid") from error
        if not math.isfinite(self._startup_timeout) or self._startup_timeout <= 0.0:
            raise BootstrapError("startup_timeout_sec must be finite and positive")
        if not all(math.isfinite(value) for value in (initial_x, initial_y, initial_yaw)):
            raise BootstrapError("initial robot pose parameters must be finite")
        self._expected_robot = (initial_x, initial_y, initial_yaw)

        self._products: Dict[int, Product] = {}
        self._attachment_states: Dict[int, str] = {product_id: "" for product_id in PRODUCT_IDS}
        self._attachment_received: Dict[int, float] = {product_id: 0.0 for product_id in PRODUCT_IDS}
        self._product_poses: Dict[int, PoseStamped] = {}
        self._product_received: Dict[int, float] = {product_id: 0.0 for product_id in PRODUCT_IDS}
        self._robot_pose: Optional[PoseStamped] = None
        self._robot_received = 0.0
        self._joint_states: Optional[JointState] = None
        self._joint_received = 0.0
        self._latest_joint_stamp_ns: Optional[int] = None
        self._release_tracking = False
        self._release_baseline_joint_stamp_ns: Optional[int] = None
        self._release_joint_stamp_count = 0
        self._sim_time: Optional[float] = None
        self._baseline_joints: Optional[Dict[str, float]] = None
        self._state = "STARTING"
        self._ready = False
        self._robot_inserted = False
        self._failure_detail = ""
        self._detach_publishers = {}

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(
            String, "/amr/simulation/attachment_bootstrap/status", status_qos)
        self._verify_srv = self.create_service(
            Trigger, "/amr/simulation/attachment_bootstrap/verify", self._verify_callback)
        self._inserted_srv = self.create_service(
            Trigger, "/amr/simulation/attachment_bootstrap/robot_inserted",
            self._robot_inserted_callback)
        self._control_client = self.create_client(
            ControlWorld, "/world/factory_world/control")
        self._set_pose_client = self.create_client(
            SetEntityPose, "/world/factory_world/set_pose")
        self._sensor_qos = sensor_qos
        self._state_qos = state_qos
        self._status_timer = self.create_timer(0.25, self._publish_status)
        self._publish_status()

    def _register_subscriptions(self) -> None:
        self._subscriptions.extend([
            self.create_subscription(
                PoseStamped, "/amr/simulation/ground_truth/pose",
                self._robot_pose_callback, self._sensor_qos),
            self.create_subscription(
                # The native Gazebo JointStatePublisher remains available
                # while controller spawners are intentionally held back.
                JointState, "/amr/simulation/base/joint_states",
                self._joint_state_callback, self._sensor_qos),
            self.create_subscription(
                Clock, "/clock", self._clock_callback, self._sensor_qos),
        ])
        for product_id, product in self._products.items():
            self._subscriptions.append(self.create_subscription(
                PoseStamped, f"/model/{product.model}/pose",
                lambda message, pid=product_id: self._product_pose_callback(pid, message),
                self._sensor_qos,
            ))
            self._subscriptions.append(self.create_subscription(
                String,
                f"/amr/simulation/internal/attachment/product_{product_id}/state",
                lambda message, pid=product_id: self._attachment_callback(pid, message),
                self._state_qos,
            ))
            self._detach_publishers[product_id] = self.create_publisher(
                Empty,
                f"/amr/simulation/internal/attachment/product_{product_id}/detach",
                self._state_qos,
            )

    def _clock_callback(self, message: Clock) -> None:
        self._sim_time = float(message.clock.sec) + float(message.clock.nanosec) * 1e-9

    def _robot_pose_callback(self, message: PoseStamped) -> None:
        self._robot_pose = message
        self._robot_received = time.monotonic()

    def _joint_state_callback(self, message: JointState) -> None:
        self._joint_states = message
        self._joint_received = time.monotonic()
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec)
        if stamp_ns <= 0 or (
            self._latest_joint_stamp_ns is not None and
            stamp_ns <= self._latest_joint_stamp_ns
        ):
            return
        self._latest_joint_stamp_ns = stamp_ns
        if (
            self._release_tracking and
            (
                self._release_baseline_joint_stamp_ns is None or
                stamp_ns > self._release_baseline_joint_stamp_ns
            )
        ):
            self._release_joint_stamp_count += 1

    def _product_pose_callback(self, product_id: int, message: PoseStamped) -> None:
        self._product_poses[product_id] = message
        self._product_received[product_id] = time.monotonic()

    def _attachment_callback(self, product_id: int, message: String) -> None:
        self._attachment_states[product_id] = message.data
        self._attachment_received[product_id] = time.monotonic()

    def _summary(self) -> str:
        states = ",".join(
            f"product_{product_id}={self._attachment_states[product_id] or 'UNKNOWN'}"
            for product_id in PRODUCT_IDS)
        return f"{self._state} {states}"

    def _publish_status(self) -> None:
        message = String()
        message.data = self._summary()
        if self._failure_detail:
            message.data = f"{message.data}; failure={self._failure_detail}"
        self._status_pub.publish(message)

    def _verify_callback(self, request: Trigger.Request, response: Trigger.Response):
        del request
        detached = all(self._attachment_states[product_id] == "detached"
                       for product_id in PRODUCT_IDS)
        response.success = self._ready and detached
        response.message = (
            "attachment bootstrap READY and all products detached"
            if response.success else
            f"attachment bootstrap not ready: {self._summary()}"
        )
        return response

    def _robot_inserted_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        del request
        if self._state != "PAUSED":
            response.success = False
            response.message = (
                "robot insertion signal rejected until the bootstrap has paused Gazebo"
            )
            return response
        self._robot_inserted = True
        response.success = True
        response.message = "robot insertion signal accepted"
        return response

    def _spin_until(self, predicate: Callable[[], bool], timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.005, deadline - time.monotonic())))
        raise BootstrapError(f"{label} timed out")

    def _fresh(self, timestamp: float, max_age: float = 0.5) -> bool:
        return timestamp > 0.0 and time.monotonic() - timestamp <= max_age

    def _wait_future(self, future, timeout: float, label: str):
        self._spin_until(lambda: future.done(), timeout, label)
        result = future.result()
        if result is None:
            raise BootstrapError(f"{label} returned no response")
        return result

    def _call_control(self, *, pause: bool, step: bool = False) -> None:
        request = ControlWorld.Request()
        request.world_control.pause = pause
        request.world_control.step = step
        request.world_control.multi_step = 1 if step else 0
        response = self._wait_future(
            self._control_client.call_async(request), 3.0,
            "Gazebo world control request")
        if not response.success:
            raise BootstrapError("Gazebo rejected world control request")

    def _step_once(self) -> None:
        # Every bootstrap physics advance is intentionally one paused step.
        # Do not issue another control request until Gazebo has published a
        # newer clock value.  The native service reply acknowledges transport,
        # not that the requested step has already been consumed by physics.
        if self._sim_time is None:
            self._spin_until(
                lambda: self._sim_time is not None,
                STEP_CLOCK_TIMEOUT_S,
                "simulation clock before paused step",
            )
        baseline = self._sim_time
        assert baseline is not None
        self._call_control(pause=True, step=True)
        deadline = time.monotonic() + STEP_CLOCK_TIMEOUT_S
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self._sim_time is not None and
                self._sim_time > baseline + STEP_CLOCK_EPSILON_S
            ):
                return
        raise BootstrapError(
            "paused physics step did not advance the simulation clock "
            f"(baseline={baseline!r}, clock={self._sim_time!r})")

    def _set_world_paused(self, paused: bool) -> None:
        self._call_control(pause=paused, step=False)

    def _set_product_pose(self, product: Product) -> None:
        request = SetEntityPose.Request()
        request.entity.name = product.model
        request.entity.type = Entity.MODEL
        request.pose.position.x = product.pose[0]
        request.pose.position.y = product.pose[1]
        request.pose.position.z = product.pose[2]
        request.pose.orientation.z = math.sin(product.pose[3] * 0.5)
        request.pose.orientation.w = math.cos(product.pose[3] * 0.5)
        response = self._wait_future(
            self._set_pose_client.call_async(request), 3.0,
            f"set pose for {product.model}")
        if not response.success:
            raise BootstrapError(f"Gazebo rejected registered pose for {product.model}")

    def _wait_for_inputs(self) -> None:
        # Gazebo loads the full sensor world before advertising the proxied
        # control endpoints.  Use the configured bounded startup deadline
        # instead of a shorter discovery race that can fault a healthy host.
        if not self._control_client.wait_for_service(timeout_sec=self._startup_timeout):
            raise BootstrapError("Gazebo world-control service is unavailable")
        if not self._set_pose_client.wait_for_service(timeout_sec=self._startup_timeout):
            raise BootstrapError("Gazebo set-pose service is unavailable")
        self._spin_until(
            lambda: all(
                # Native attachment state is an event-driven transition topic,
                # not a periodic heartbeat.  The detach loop has already
                # observed the transition; retaining that observation is the
                # valid proof even after its message ages past the sensor QoS
                # freshness window.
                self._attachment_states[product_id] in {"attached", "detached"}
                for product_id in PRODUCT_IDS),
            self._startup_timeout, "native attachment states")

    def _capture_baseline(self) -> None:
        if self._robot_pose is None or self._joint_states is None:
            raise BootstrapError("startup baseline inputs are unavailable")
        positions = dict(zip(self._joint_states.name, self._joint_states.position))
        if any(not math.isfinite(float(value)) for value in positions.values()):
            raise BootstrapError("startup arm joint state contains a non-finite value")
        if any(joint not in positions for joint in STOW):
            raise BootstrapError("startup arm joint state omitted an expected stow joint")
        self._baseline_joints = positions

    def _check_robot_and_stow(self) -> None:
        if self._baseline_joints is None:
            raise BootstrapError("startup baseline was not captured")
        if self._robot_pose is None or not self._fresh(self._robot_received):
            raise BootstrapError("AMR pose became stale")
        position_error, yaw_error = _pose_error(
            self._robot_pose,
            (
                self._expected_robot[0],
                self._expected_robot[1],
                self._robot_pose.pose.position.z,
                self._expected_robot[2],
            ),
        )
        if position_error > ROBOT_POSITION_TOLERANCE_M or yaw_error > ROBOT_YAW_TOLERANCE_RAD:
            raise BootstrapError(
                f"AMR moved during attachment bootstrap: {position_error:.6f} m / "
                f"{yaw_error:.6f} rad")
        if self._joint_states is None or not self._fresh(self._joint_received):
            raise BootstrapError("arm joint state became stale")
        positions = dict(zip(self._joint_states.name, self._joint_states.position))
        for joint, target in STOW.items():
            value = positions.get(joint)
            if value is None or not math.isfinite(float(value)):
                raise BootstrapError(f"arm joint {joint} is unavailable or non-finite")
            if abs(float(value) - target) > STOW_TOLERANCE_RAD:
                raise BootstrapError(
                    f"arm left empty stow during attachment bootstrap: {joint}={value:.6f}")

    def _check_products(self, drift_reference: Optional[Dict[int, Tuple[float, float, float]]] = None) -> None:
        for product_id, product in self._products.items():
            pose = self._product_poses.get(product_id)
            if pose is None or not self._fresh(self._product_received[product_id]):
                raise BootstrapError(f"product {product_id} pose became stale")
            position_error, yaw_error = _pose_error(pose, product.pose)
            if position_error > PRODUCT_POSITION_TOLERANCE_M or yaw_error > PRODUCT_YAW_TOLERANCE_RAD:
                raise BootstrapError(
                    f"product {product_id} pose tolerance failed: "
                    f"{position_error:.6f} m / {yaw_error:.6f} rad")
            if drift_reference is not None:
                current = (
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                reference = drift_reference[product_id]
                drift = math.sqrt(sum((current[index] - reference[index]) ** 2 for index in range(3)))
                if not math.isfinite(drift) or drift > PRODUCT_DRIFT_TOLERANCE_M:
                    raise BootstrapError(
                        f"product {product_id} drifted during startup observation: {drift:.6f} m")

    def _confirm_world_release(self) -> None:
        """Prove the final unpause left Gazebo advancing before READY."""
        baseline_sim = self._sim_time
        if baseline_sim is None:
            raise BootstrapError("simulation clock was unavailable before final release")
        self._release_baseline_joint_stamp_ns = self._latest_joint_stamp_ns
        self._release_joint_stamp_count = 0
        self._release_tracking = True
        deadline = time.monotonic() + POST_RELEASE_WALL_TIMEOUT_S
        try:
            # The earlier bounded observation includes a paused step request.
            # Send this final unpause only after that request has completed so
            # an out-of-order transport response cannot leave the world paused.
            self._set_world_paused(False)
            self.get_logger().info(
                "Final startup release requested; confirming advancing clock and joint states")
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                sim_advanced = (
                    self._sim_time is not None and
                    self._sim_time - baseline_sim >= POST_RELEASE_CONFIRM_SIM_SECONDS
                )
                joints_advanced = (
                    self._release_joint_stamp_count >= MIN_POST_RELEASE_JOINT_STAMPS
                )
                if sim_advanced and joints_advanced:
                    self.get_logger().info(
                        "Final startup release confirmed: clock and raw joint states are advancing")
                    return
            raise BootstrapError(
                "final startup release did not produce advancing clock and "
                f"{MIN_POST_RELEASE_JOINT_STAMPS} newer joint-state stamps "
                f"(clock={self._sim_time!r}, joint_stamps={self._release_joint_stamp_count})")
        finally:
            self._release_tracking = False
            self._release_baseline_joint_stamp_ns = None

    def _queue_detach_and_step(self) -> None:
        deadline = time.monotonic() + self._startup_timeout
        next_request = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                # Queue every detach before asking Gazebo for the first step.
                for product_id in PRODUCT_IDS:
                    self._detach_publishers[product_id].publish(Empty())
                next_request = now + 0.2
            if all(self._attachment_states[product_id] == "detached"
                   for product_id in PRODUCT_IDS):
                return
            self._step_once()
            rclpy.spin_once(self, timeout_sec=0.02)
        raise BootstrapError("native products did not all detach during bootstrap")

    def _observe_release(self) -> None:
        release_sim: Optional[float] = None
        start_sim: Optional[float] = None
        deadline = time.monotonic() + OBSERVATION_WALL_TIMEOUT_S
        drift_reference = {
            product_id: (
                product.pose[0], product.pose[1], product.pose[2])
            for product_id, product in self._products.items()
        }
        self._set_world_paused(False)
        self.get_logger().info("Bounded startup release requested")
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._sim_time is None:
                    continue
                if (
                    self._robot_pose is None or
                    not self._fresh(self._robot_received) or
                    self._joint_states is None or
                    not self._fresh(self._joint_received) or
                    any(
                        product_id not in self._product_poses or
                        not self._fresh(self._product_received[product_id])
                        for product_id in PRODUCT_IDS)
                ):
                    continue
                if self._baseline_joints is None:
                    self._capture_baseline()
                self._check_robot_and_stow()
                if release_sim is None:
                    release_sim = self._sim_time
                    self.get_logger().info(
                        "Startup release inputs complete; settling product contacts "
                        f"for {PRODUCT_SETTLE_SIM_SECONDS:.2f} simulated seconds")
                # During the bounded contact-settle interval keep validating
                # finite product samples, but defer the strict registered-pose
                # and drift gates until the initial contact impulse has ended.
                if self._sim_time - release_sim < PRODUCT_SETTLE_SIM_SECONDS:
                    for product_id, product_pose in self._product_poses.items():
                        _pose_error(product_pose, self._products[product_id].pose)
                    continue
                if start_sim is None:
                    start_sim = self._sim_time
                    self.get_logger().info(
                        "Product contact settle complete; starting strict "
                        f"{OBSERVATION_SIM_SECONDS:.2f} simulated-second observation")
                self._check_products(drift_reference)
                if self._sim_time - start_sim >= OBSERVATION_SIM_SECONDS:
                    self._confirm_world_release()
                    return
            raise BootstrapError(
                "0.5 simulated-second startup observation timed out "
                f"(clock={self._sim_time!r}, robot={self._robot_pose is not None}, "
                f"joint={self._joint_states is not None}, "
                f"products={tuple(product_id in self._product_poses for product_id in PRODUCT_IDS)})")
        except Exception:
            # A failed observation must return the world to the paused state
            # before the fault is latched.  If pausing itself fails, preserve
            # the original failure and expose it through the latched status.
            try:
                self._set_world_paused(True)
            except Exception as pause_error:
                raise BootstrapError(f"{pause_error}; original observation failed")
            raise

    def run(self) -> None:
        self._products = _load_products(Path(get_package_share_directory("amr_factory")))
        self._register_subscriptions()
        if not self._control_client.wait_for_service(timeout_sec=3.0):
            raise BootstrapError("Gazebo world-control service is unavailable")
        if not self._set_pose_client.wait_for_service(timeout_sec=3.0):
            raise BootstrapError("Gazebo set-pose service is unavailable")
        # Native attachment launch starts the already-warm server before this
        # node.  Pause it before insertion so the robot is created in a
        # no-update window; the launch handshake signals insertion only after
        # the create action has completed.  Validate live sensor inputs only
        # after registered product poses are restored.
        self._set_world_paused(True)
        self._state = "PAUSED"
        self._publish_status()
        self._spin_until(
            lambda: self._robot_inserted,
            self._startup_timeout,
            "robot insertion signal",
        )
        self._queue_detach_and_step()
        self._wait_for_inputs()
        # The world is already paused while detaching; retain the explicit
        # pause request so startup does not depend on Gazebo's initial state.
        self._set_world_paused(True)
        for product in self._products.values():
            self._set_product_pose(product)
        # Let Gazebo commit the paused pose updates before the release request.
        # This remains a single bounded physics step owned by the bootstrap.
        self._step_once()
        self._observe_release()
        self._ready = True
        self._state = "READY"
        self._publish_status()

    def fail_closed(self, detail: str) -> None:
        self._ready = False
        self._state = "FAULT"
        self._failure_detail = str(detail)
        try:
            self._set_world_paused(True)
        except Exception as pause_error:
            self._failure_detail = f"{self._failure_detail}; pause failed: {pause_error}"
        self._publish_status()


def main() -> int:
    rclpy.init()
    node: Optional[AttachmentBootstrap] = None
    try:
        node = AttachmentBootstrap()
        try:
            node.run()
        except Exception as error:  # noqa: BLE001 - fault must be latched
            node.fail_closed(str(error))
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            # SIGINT is the normal bounded-runtime shutdown path.
            return 0
        return 0
    except Exception as error:  # constructor/configuration failure
        print(f"GATE6 ATTACHMENT BOOTSTRAP FAULT: {error}", flush=True)
        return 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (RuntimeError, ValueError):
                # rclpy may already have removed a waitable while handling
                # SIGINT; do not turn an orderly shutdown into a Gate fault.
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
