#!/usr/bin/env python3
"""Prepare one product from the current AMR pose, then run its Gate 6 stage.

This runner is intentionally limited to products 102 and 103.  The accepted
product-101 path remains the existing gate6_mass_stage launch.  The runner
does not reset the AMR; it only resets the selected product while the factory
world is paused, navigates to that product's dock, performs at most one
bounded relocalization when physical and localized poses disagree, and
then starts the existing mass-stage executable. A relocalized retry first
retreats to the registered approach pose before making one fresh exact dock
approach, so the collision-checked controller does not turn the full
rectangular footprint beside the pedestal.
"""

from __future__ import annotations

import fcntl
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import yaml

from amr_interfaces.msg import BaseStatus, ManipulatorStatus


PRODUCT_IDS = (101, 102, 103)
RESET_PRODUCT_IDS = (102, 103)
STOW = {
    "arm_joint_1": 0.0,
    "arm_joint_2": -1.5708,
    "arm_joint_3": 1.5708,
    "arm_joint_4": 0.0,
    "arm_joint_5": 0.0,
    "arm_joint_6": 0.0,
}
STOW_TOLERANCE_RAD = 0.01
RESET_POSITION_TOLERANCE_M = 0.005
RESET_YAW_TOLERANCE_RAD = 0.01
RESET_STABILITY_WINDOW_S = 0.5
RESET_STABILITY_TOLERANCE_M = 0.005
ROBOT_RESET_POSITION_TOLERANCE_M = 0.005
ROBOT_RESET_YAW_TOLERANCE_RAD = 0.02
DOCK_POSITION_TOLERANCE_M = 0.03
DOCK_YAW_TOLERANCE_RAD = 0.15
PRODUCT_RELATIVE_TOLERANCE_M = 0.04
DOCK_CORRECTION_MAX_POSITION_M = 0.15
DOCK_CORRECTION_MAX_YAW_RAD = 0.15
NAVIGATION_FEEDBACK_MAX_AGE_S = 1.0
RELOCALIZATION_CONVERGENCE_POSITION_M = 0.03
RELOCALIZATION_CONVERGENCE_YAW_RAD = 0.10
# AMCL emits on filter updates; a precise leg can finish several seconds after
# its last update while the base is still stopped and its terminal TF is fresh.
RELOCALIZATION_TERMINAL_AMCL_MAX_AGE_S = 6.0


class PreparationError(RuntimeError):
    """A fail-closed preparation failure."""


@dataclass(frozen=True)
class ProductMetadata:
    product_id: int
    model: str
    mass_kg: float
    reset_pose: Tuple[float, float, float, float]
    approach: Tuple[float, float, float]
    dock: Tuple[float, float, float]
    egress: Optional[Tuple[float, float, float]]


def _finite_values(values: Iterable[float], label: str) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result or not all(math.isfinite(value) for value in result):
        raise PreparationError(f"{label} contains a non-finite value")
    return result


def _pose3(mapping: Dict[str, object], label: str) -> Tuple[float, float, float]:
    try:
        return _finite_values((mapping["x"], mapping["y"], mapping["yaw"]), label)  # type: ignore[index]
    except (KeyError, TypeError, ValueError) as error:
        raise PreparationError(f"{label} is invalid") from error


def _sdf_product_pose(sdf_path: Path, model: str) -> Tuple[float, float, float, float]:
    root = ET.parse(sdf_path).getroot()
    element = root.find(f"./world/model[@name='{model}']/pose")
    if element is None or not element.text:
        raise PreparationError(f"factory SDF has no pose for {model}")
    try:
        values = _finite_values(element.text.split(), f"factory SDF pose for {model}")
    except ValueError as error:
        raise PreparationError(f"factory SDF pose for {model} is invalid") from error
    if len(values) != 6:
        raise PreparationError(f"factory SDF pose for {model} must contain six values")
    return values[0], values[1], values[2], values[5]


def load_product_metadata() -> Dict[int, ProductMetadata]:
    factory = Path(get_package_share_directory("amr_factory"))
    try:
        with (factory / "config" / "products.yaml").open(encoding="utf-8") as stream:
            products_registry = yaml.safe_load(stream)
        with (factory / "config" / "stations.yaml").open(encoding="utf-8") as stream:
            stations_registry = yaml.safe_load(stream)
    except OSError as error:
        raise PreparationError("factory product or station registry is unavailable") from error

    products = products_registry.get("products", {})
    stations = stations_registry.get("stations", {})
    sdf_path = factory / "worlds" / "factory.sdf"
    metadata: Dict[int, ProductMetadata] = {}
    for model, entry in products.items():
        try:
            product_id = int(entry["tag_id"])
            station_name = entry["pickup_station"]
            station = stations[station_name]
            mass_kg = float(entry["mass"])
        except (KeyError, TypeError, ValueError) as error:
            raise PreparationError(f"invalid registry entry for {model}") from error
        if product_id not in PRODUCT_IDS or not math.isfinite(mass_kg) or mass_kg < 0.0:
            raise PreparationError(f"invalid product metadata for {model}")
        metadata[product_id] = ProductMetadata(
            product_id=product_id,
            model=str(model),
            mass_kg=mass_kg,
            reset_pose=_sdf_product_pose(sdf_path, str(model)),
            approach=_pose3(station["approach"], f"{station_name}.approach"),
            dock=_pose3(station["dock"], f"{station_name}.dock"),
            egress=(None if station.get("egress") is None else
                    _pose3(station["egress"], f"{station_name}.egress")),
        )
    if set(metadata) != set(PRODUCT_IDS):
        raise PreparationError("factory registry does not contain products 101, 102, and 103")
    return metadata


def _yaw(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def _angle_error(actual: float, target: float) -> float:
    return abs(math.remainder(actual - target, 2.0 * math.pi))


def _pose_error(actual: PoseStamped, target: Tuple[float, float, float]) -> Tuple[float, float]:
    return (
        math.hypot(actual.pose.position.x - target[0], actual.pose.position.y - target[1]),
        _angle_error(_yaw(actual), target[2]),
    )


def _pose_tuple(pose: PoseStamped) -> Tuple[float, float, float]:
    return pose.pose.position.x, pose.pose.position.y, _yaw(pose)


def _amcl_pose_tuple(pose: PoseWithCovarianceStamped) -> Tuple[float, float, float]:
    orientation = pose.pose.pose.orientation
    return (
        pose.pose.pose.position.x,
        pose.pose.pose.position.y,
        math.atan2(
            2.0 * orientation.w * orientation.z,
            1.0 - 2.0 * orientation.z * orientation.z),
    )


def _xy_distance(first: Tuple[float, float, float], second: Tuple[float, float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _dock_bias(
    physical: Tuple[float, float, float],
    localized: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    physical_values = _finite_values(physical, "physical dock pose")
    localized_values = _finite_values(localized, "localized dock pose")
    bias = (
        physical_values[0] - localized_values[0],
        physical_values[1] - localized_values[1],
        math.remainder(physical_values[2] - localized_values[2], 2.0 * math.pi),
    )
    if not all(math.isfinite(value) for value in bias):
        raise PreparationError("dock localization correction was non-finite")
    if math.hypot(bias[0], bias[1]) > DOCK_CORRECTION_MAX_POSITION_M:
        raise PreparationError(
            "dock localization correction exceeded the bounded position limit")
    if abs(bias[2]) > DOCK_CORRECTION_MAX_YAW_RAD:
        raise PreparationError(
            "dock localization correction exceeded the bounded yaw limit")
    return bias


class ProductPreparation(Node):
    """Reset and pre-position one product while preserving AMR pose history."""

    def __init__(self) -> None:
        super().__init__("gate6_product_test_preparation")
        # Launch may auto-declare this parameter from its override. Keep the
        # standalone default without redeclaring an existing parameter.
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("product_id", 102)
        self.product_id = int(self.get_parameter("product_id").value)
        if self.product_id not in RESET_PRODUCT_IDS:
            raise PreparationError("the persistent-position runner accepts only product IDs 102 and 103")
        self.metadata = load_product_metadata()
        self.selected = self.metadata[self.product_id]

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        diagnostic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        authority_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=0.1),
        )

        self._base_status: Optional[BaseStatus] = None
        self._base_status_at = 0.0
        self._odometry: Optional[Odometry] = None
        self._odometry_at = 0.0
        self._joint_states: Optional[JointState] = None
        self._joint_states_at = 0.0
        self._robot_pose: Optional[PoseStamped] = None
        self._robot_pose_at = 0.0
        self._amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self._amcl_pose_at = 0.0
        self._amcl_pose_generation = 0
        self._product_poses: Dict[int, PoseStamped] = {}
        self._product_pose_at: Dict[int, float] = {}
        self._manipulator_status: Optional[ManipulatorStatus] = None
        self._manipulator_status_at = 0.0

        self.create_subscription(
            BaseStatus, "/amr/base/status", self._base_status_callback, diagnostic_qos)
        self.create_subscription(
            Odometry, "/amr/base/odometry_raw", self._odometry_callback, sensor_qos)
        self.create_subscription(
            JointState, "/amr/base/joint_states", self._joint_states_callback, sensor_qos)
        self.create_subscription(
            PoseStamped, "/amr/simulation/ground_truth/pose", self._robot_pose_callback, sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amr/amcl_pose", self._amcl_pose_callback, amcl_qos)
        self.create_subscription(
            ManipulatorStatus, "/amr/manipulation/status",
            self._manipulator_status_callback, authority_qos)

        for product_id, metadata in self.metadata.items():
            self.create_subscription(
                PoseStamped,
                f"/model/{metadata.model}/pose",
                lambda message, pid=product_id: self._product_pose_callback(pid, message),
                sensor_qos,
            )

        self._status_publisher = self.create_publisher(
            ManipulatorStatus, "/amr/manipulation/status", authority_qos)
        self._control_client = self.create_client(
            ControlWorld, "/world/factory_world/control")
        self._set_pose_client = self.create_client(
            SetEntityPose, "/world/factory_world/set_pose")
        self._bootstrap_client = self.create_client(
            Trigger, "/amr/simulation/attachment_bootstrap/verify")
        self._set_initial_pose_client = self.create_client(
            SetInitialPose, "/amr/set_initial_pose")
        self._normal_navigation = ActionClient(
            self, NavigateToPose, "/amr/mission/navigate_to_pose")
        self._precise_navigation = ActionClient(
            self, NavigateToPose, "/amr/mission/navigate_to_pose_precise")
        self._retreat_navigation = ActionClient(
            self, NavigateToPose, "/amr/mission/navigate_to_pose_retreat")
        self._boot_id = int(time.monotonic_ns() & 0xFFFFFFFF) or 1
        self._sequence = 0
        self._state = ManipulatorStatus.STARTING
        self._base_motion_allowed = False
        self._product_attached = False
        self._detail = "Gate 6 product preparation is starting"
        self._status_publishing_enabled = False
        self._status_timer = self.create_timer(0.05, self._publish_status)

    def _base_status_callback(self, message: BaseStatus) -> None:
        self._base_status = message
        self._base_status_at = time.monotonic()

    def _odometry_callback(self, message: Odometry) -> None:
        self._odometry = message
        self._odometry_at = time.monotonic()

    def _joint_states_callback(self, message: JointState) -> None:
        self._joint_states = message
        self._joint_states_at = time.monotonic()

    def _robot_pose_callback(self, message: PoseStamped) -> None:
        self._robot_pose = message
        self._robot_pose_at = time.monotonic()

    def _amcl_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = message
        self._amcl_pose_at = time.monotonic()
        self._amcl_pose_generation += 1

    def _product_pose_callback(self, product_id: int, message: PoseStamped) -> None:
        self._product_poses[product_id] = message
        self._product_pose_at[product_id] = time.monotonic()

    def _manipulator_status_callback(self, message: ManipulatorStatus) -> None:
        self._manipulator_status = message
        self._manipulator_status_at = time.monotonic()

    def _publish_status(self) -> None:
        if not self._status_publishing_enabled:
            return
        message = ManipulatorStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.source_boot_id = self._boot_id
        self._sequence += 1
        message.sequence = self._sequence
        message.valid = self._state != ManipulatorStatus.FAULT
        message.state = self._state
        message.base_motion_allowed = self._base_motion_allowed
        message.product_attached = self._product_attached
        message.product_id = str(self.product_id) if self._product_attached else ""
        message.detail = self._detail
        self._status_publisher.publish(message)

    def _set_status(self, state: int, base_motion_allowed: bool, attached: bool, detail: str) -> None:
        self._state = state
        self._base_motion_allowed = base_motion_allowed
        self._product_attached = attached
        self._detail = detail
        self._status_publishing_enabled = True
        self._publish_status()

    def _spin_until(self, predicate: Callable[[], bool], timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self)
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if predicate():
                    return
                # Keep one callback generator alive and use a constant slice.
                # Recreating the generator with a changing timeout can starve
                # lower-rate subscriptions behind continuously-ready callbacks.
                executor.spin_once(timeout_sec=0.05)
        finally:
            executor.remove_node(self)
            executor.shutdown(timeout_sec=0.0)
        raise PreparationError(f"{label} timed out")

    def _fresh(self, timestamp: float, max_age: float = 0.2) -> bool:
        return timestamp > 0.0 and time.monotonic() - timestamp <= max_age

    def _wait_for_inputs(self) -> None:
        self._spin_until(
            lambda: self._base_status is not None and self._fresh(self._base_status_at),
            8.0, "fresh base status")
        self._spin_until(
            lambda: self._odometry is not None and self._fresh(self._odometry_at),
            8.0, "fresh base odometry")
        self._spin_until(
            lambda: self._joint_states is not None and self._fresh(self._joint_states_at),
            8.0, "fresh arm joint state")
        self._spin_until(
            lambda: self._robot_pose is not None and self._fresh(self._robot_pose_at),
            8.0, "fresh AMR ground-truth pose")
        self._spin_until(
            lambda: all(
                product_id in self._product_poses and
                self._fresh(self._product_pose_at[product_id])
                for product_id in PRODUCT_IDS
            ),
            8.0, "fresh product poses")
        # Attachment state is an event-driven transition topic with volatile
        # QoS, not a periodic heartbeat.  The durable bootstrap verify service
        # below is the authoritative all-detached snapshot.

    def _base_is_stationary(self) -> bool:
        if self._base_status is None or self._odometry is None:
            return False
        if not self._fresh(self._base_status_at) or not self._fresh(self._odometry_at):
            return False
        status = self._base_status
        twist = self._odometry.twist.twist
        return (
            status.valid and
            status.source_boot_id != 0 and
            status.sequence != 0 and
            status.state == BaseStatus.READY and
            status.reason == BaseStatus.REASON_READY and
            abs(twist.linear.x) <= 0.01 and
            abs(twist.linear.y) <= 0.01 and
            abs(twist.angular.z) <= 0.01
        )

    def _wait_stationary(self) -> None:
        stationary_since = [0.0]

        def stationary() -> bool:
            now = time.monotonic()
            if self._base_is_stationary():
                if stationary_since[0] == 0.0:
                    stationary_since[0] = now
                return now - stationary_since[0] >= 0.5
            stationary_since[0] = 0.0
            return False

        self._spin_until(stationary, 8.0, "500 ms stationary base evidence")

    def _arm_is_stowed(self) -> bool:
        if self._joint_states is None or not self._fresh(self._joint_states_at):
            return False
        positions = dict(zip(self._joint_states.name, self._joint_states.position))
        return all(
            joint in positions and abs(positions[joint] - target) <= STOW_TOLERANCE_RAD
            for joint, target in STOW.items()
        )

    def _check_preconditions(self) -> None:
        self._wait_for_inputs()
        self._wait_stationary()
        active_nodes = {name for name, _ in self.get_node_names_and_namespaces()}
        if "gate6_mass_stage" in active_nodes:
            raise PreparationError("reset refused: an existing Gate 6 mass stage is active")
        if not self._arm_is_stowed():
            raise PreparationError("reset refused: arm is not at the empty stow pose")
        if self._manipulator_status is not None and self._fresh(self._manipulator_status_at, 0.5):
            status = self._manipulator_status
            if status.product_attached or status.state in (
                ManipulatorStatus.MOVING,
                ManipulatorStatus.DEPLOYED,
                ManipulatorStatus.FAULT,
            ):
                raise PreparationError("reset refused: active manipulator status is not empty and stowed")
        assert self._robot_pose is not None
        robot = _pose_tuple(self._robot_pose)
        if not _pose_error(self._robot_pose, self.selected.dock)[0] <= DOCK_POSITION_TOLERANCE_M:
            if _xy_distance(robot, self.selected.reset_pose[:3]) < 0.80:
                raise PreparationError(
                    "reset refused: AMR is too close to the product reset pose; move it away first")

    def _wait_future(self, future: object, timeout: float, label: str):
        self._spin_until(lambda: bool(getattr(future, "done")()), timeout, label)
        result = getattr(future, "result")()
        if result is None:
            raise PreparationError(f"{label} returned no response")
        return result

    def _set_world_paused(self, paused: bool) -> None:
        if not self._control_client.wait_for_service(timeout_sec=3.0):
            raise PreparationError("Gazebo world control service is unavailable")
        request = ControlWorld.Request()
        request.world_control.pause = paused
        response = self._wait_future(
            self._control_client.call_async(request), 3.0,
            "Gazebo world pause" if paused else "Gazebo world unpause")
        if not response.success:
            raise PreparationError(
                "Gazebo world pause failed" if paused else "Gazebo world unpause failed")

    def _set_selected_product_pose(self) -> None:
        if not self._set_pose_client.wait_for_service(timeout_sec=3.0):
            raise PreparationError("Gazebo set-pose service is unavailable")
        request = SetEntityPose.Request()
        request.entity.name = self.selected.model
        request.entity.type = Entity.MODEL
        x, y, z, yaw = self.selected.reset_pose
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.w = math.cos(yaw * 0.5)
        request.pose.orientation.z = math.sin(yaw * 0.5)
        response = self._wait_future(
            self._set_pose_client.call_async(request), 3.0,
            f"reset pose for product {self.product_id}")
        if not response.success:
            raise PreparationError(f"Gazebo rejected reset pose for product {self.product_id}")

    def _verify_attachment_bootstrap(self) -> None:
        if not self._bootstrap_client.wait_for_service(timeout_sec=5.0):
            raise PreparationError(
                "attachment bootstrap verify service is unavailable; refusing product preparation")
        response = self._wait_future(
            self._bootstrap_client.call_async(Trigger.Request()), 5.0,
            "attachment bootstrap verification")
        if not response.success:
            raise PreparationError(
                f"attachment bootstrap verification failed: {response.message}")

    def _wait_selected_pose_stable(self) -> None:
        start = time.monotonic()
        first: Optional[PoseStamped] = None

        def stable() -> bool:
            nonlocal first
            pose = self._product_poses.get(self.product_id)
            if pose is None or not self._fresh(self._product_pose_at[self.product_id]):
                first = None
                return False
            position_error = math.sqrt(
                (pose.pose.position.x - self.selected.reset_pose[0]) ** 2 +
                (pose.pose.position.y - self.selected.reset_pose[1]) ** 2 +
                (pose.pose.position.z - self.selected.reset_pose[2]) ** 2)
            yaw_error = _angle_error(_yaw(pose), self.selected.reset_pose[3])
            if position_error > RESET_POSITION_TOLERANCE_M or yaw_error > RESET_YAW_TOLERANCE_RAD:
                first = None
                return False
            if first is None:
                first = pose
                return False
            first_position = first.pose.position
            drift = math.sqrt(
                (pose.pose.position.x - first_position.x) ** 2 +
                (pose.pose.position.y - first_position.y) ** 2 +
                (pose.pose.position.z - first_position.z) ** 2)
            return time.monotonic() - start >= RESET_STABILITY_WINDOW_S and \
                drift <= RESET_STABILITY_TOLERANCE_M

        self._spin_until(stable, 5.0, f"stable reset pose for product {self.product_id}")

    def _reset_selected_product(self) -> None:
        assert self._robot_pose is not None
        before_robot = _pose_tuple(self._robot_pose)
        before_other_products = {
            product_id: (
                self._product_poses[product_id].pose.position.x,
                self._product_poses[product_id].pose.position.y,
                self._product_poses[product_id].pose.position.z,
            )
            for product_id in PRODUCT_IDS if product_id != self.product_id
        }
        paused = False
        try:
            self._set_world_paused(True)
            paused = True
            self._set_selected_product_pose()
        finally:
            if paused:
                self._set_world_paused(False)
        self._wait_selected_pose_stable()
        assert self._robot_pose is not None
        after_robot = _pose_tuple(self._robot_pose)
        if _xy_distance(before_robot, after_robot) > ROBOT_RESET_POSITION_TOLERANCE_M or \
                _angle_error(before_robot[2], after_robot[2]) > ROBOT_RESET_YAW_TOLERANCE_RAD:
            raise PreparationError("AMR pose changed during product reset")
        for product_id, before in before_other_products.items():
            pose = self._product_poses[product_id]
            after = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            if math.dist(before, after) > 0.01:
                raise PreparationError(f"unselected product {product_id} moved during reset")

    def _retreat_from_other_pickup_dock(self) -> None:
        """Clear a different pickup dock before crossing the factory aisle."""
        assert self._robot_pose is not None
        for product_id, metadata in self.metadata.items():
            if product_id == self.product_id or metadata.egress is None:
                continue
            position_error, yaw_error = _pose_error(self._robot_pose, metadata.dock)
            if position_error <= DOCK_POSITION_TOLERANCE_M and \
                    yaw_error <= DOCK_YAW_TOLERANCE_RAD:
                self.get_logger().info(
                    f"AMR is at pickup dock for product {product_id}; "
                    f"retreating to its registered egress before product {self.product_id}")
                self._navigate(metadata.egress, precise=True, retreat=True)
                self._wait_stationary()
                return

    def _navigate(
        self, target: Tuple[float, float, float], precise: bool, retreat: bool = False
    ) -> Tuple[float, float, float]:
        if retreat and not precise:
            raise PreparationError("retreat navigation requires the precise controller")
        if retreat:
            client = self._retreat_navigation
            endpoint = "/amr/mission/navigate_to_pose_retreat"
        else:
            client = self._precise_navigation if precise else self._normal_navigation
            endpoint = "/amr/mission/navigate_to_pose_precise" if precise else "/amr/mission/navigate_to_pose"
        if not client.wait_for_server(timeout_sec=5.0):
            raise PreparationError(f"navigation action {endpoint} is unavailable")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.orientation.z = math.sin(target[2] * 0.5)
        goal.pose.pose.orientation.w = math.cos(target[2] * 0.5)
        latest_localized: Optional[Tuple[float, float, float]] = None
        latest_localized_at = 0.0

        def feedback_callback(feedback_message: object) -> None:
            nonlocal latest_localized, latest_localized_at
            feedback = getattr(feedback_message, "feedback", None)
            pose = getattr(feedback, "current_pose", None)
            if pose is None or getattr(pose.header, "frame_id", "") != "map":
                return
            try:
                values = _pose_tuple(pose)
                values = _finite_values(values, "localized navigation feedback")
            except (AttributeError, PreparationError, TypeError, ValueError):
                return
            latest_localized = values  # type: ignore[assignment]
            latest_localized_at = time.monotonic()

        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        goal_handle = self._wait_future(future, 5.0, f"{endpoint} goal acceptance")
        if not goal_handle.accepted:
            raise PreparationError(f"navigation goal to {endpoint} was rejected")
        try:
            result = self._wait_future(
                goal_handle.get_result_async(), 180.0,
                f"navigation result from {endpoint}")
        except PreparationError:
            cancel_future = goal_handle.cancel_goal_async()
            try:
                self._wait_future(cancel_future, 3.0, f"cancel {endpoint} goal")
            except PreparationError:
                self.get_logger().error(f"failed to cancel timed-out {endpoint} goal")
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise PreparationError(
                f"navigation endpoint {endpoint} to "
                f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) failed")
        if latest_localized is None or not self._fresh(
                latest_localized_at, NAVIGATION_FEEDBACK_MAX_AGE_S):
            raise PreparationError(
                f"fresh localized terminal pose from {endpoint} was unavailable")
        self.get_logger().info(
            f"{endpoint} localized terminal pose: "
            f"x={latest_localized[0]:.4f} y={latest_localized[1]:.4f} "
            f"yaw={latest_localized[2]:.4f}")
        return latest_localized

    def _relocalize_at_reference(
        self,
        physical_reference: Tuple[float, float, float],
        localized_reference: Tuple[float, float, float],
        terminal_amcl_generation: int,
        label: str,
    ) -> None:
        """Reseed AMCL once from stationary physical evidence, then verify convergence."""
        bias = _dock_bias(physical_reference, localized_reference)
        if not self._set_initial_pose_client.wait_for_service(timeout_sec=5.0):
            raise PreparationError("AMCL set-initial-pose service is unavailable")
        # AMCL publishes on filter updates, not as a heartbeat.  Use the
        # sample received during the recovery leg, then require that the
        # base is still stationary and the sample remains within a bounded
        # event-evidence age before seeding from physical ground truth.
        if self._amcl_pose is None or self._amcl_pose_generation < terminal_amcl_generation:
            raise PreparationError("AMCL terminal pose from recovery navigation is unavailable")
        if not self._fresh(self._amcl_pose_at, RELOCALIZATION_TERMINAL_AMCL_MAX_AGE_S):
            raise PreparationError("AMCL terminal pose from recovery navigation is too old")
        if not self._base_is_stationary():
            raise PreparationError(f"base is not stationary for {label.lower()} relocalization")
        assert self._amcl_pose is not None
        if self._amcl_pose.header.frame_id != "map":
            raise PreparationError("AMCL pose before relocalization is not in map frame")
        try:
            previous_amcl = _finite_values(
                _amcl_pose_tuple(self._amcl_pose), "AMCL pose before relocalization")
        except (AttributeError, PreparationError, TypeError, ValueError) as error:
            raise PreparationError("AMCL pose before relocalization is invalid") from error
        previous_amcl_at = self._amcl_pose_at

        request = SetInitialPose.Request()
        request.pose.header.frame_id = "map"
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.pose.position.x = physical_reference[0]
        request.pose.pose.pose.position.y = physical_reference[1]
        request.pose.pose.pose.orientation.w = math.cos(physical_reference[2] * 0.5)
        request.pose.pose.pose.orientation.z = math.sin(physical_reference[2] * 0.5)
        request.pose.pose.covariance[0] = 0.01 ** 2
        request.pose.pose.covariance[7] = 0.01 ** 2
        request.pose.pose.covariance[35] = 0.05 ** 2
        baseline_generation = self._amcl_pose_generation
        request_started_at = time.monotonic()
        self._wait_future(
            self._set_initial_pose_client.call_async(request), 5.0,
            "AMCL set-initial-pose request")
        self.get_logger().info(
            f"{label} relocalization requested from stationary physical pose: "
            f"physical=({physical_reference[0]:.4f}, {physical_reference[1]:.4f}, {physical_reference[2]:.4f}) "
            f"localized=({localized_reference[0]:.4f}, {localized_reference[1]:.4f}, {localized_reference[2]:.4f}) "
            f"bias=({bias[0]:.4f}, {bias[1]:.4f}, {bias[2]:.4f}) "
            f"previous_amcl=({previous_amcl[0]:.4f}, {previous_amcl[1]:.4f}, "
            f"{previous_amcl[2]:.4f}) previous_sample_age={request_started_at - previous_amcl_at:.3f}s")

        def converged() -> bool:
            pose = self._amcl_pose
            if pose is None or self._amcl_pose_generation <= baseline_generation:
                return False
            if not self._fresh(self._amcl_pose_at) or pose.header.frame_id != "map":
                return False
            try:
                current = _finite_values(_amcl_pose_tuple(pose), "AMCL pose after dock relocalization")
            except (AttributeError, TypeError, ValueError, PreparationError):
                return False
            return (
                _xy_distance(current, physical_reference) <= RELOCALIZATION_CONVERGENCE_POSITION_M and
                _angle_error(current[2], physical_reference[2]) <= RELOCALIZATION_CONVERGENCE_YAW_RAD)

        self._spin_until(
            converged, 8.0,
            f"new converged AMCL pose after {label.lower()} relocalization")
        assert self._amcl_pose is not None
        current = _finite_values(_amcl_pose_tuple(self._amcl_pose), "AMCL pose after relocalization")
        self.get_logger().info(
            f"{label} relocalization confirmed with new AMCL sample: "
            f"x={current[0]:.4f} y={current[1]:.4f} yaw={current[2]:.4f}")

    def _verify_dock_and_product_geometry(self) -> None:
        assert self._robot_pose is not None
        position_error, yaw_error = _pose_error(self._robot_pose, self.selected.dock)
        if position_error > DOCK_POSITION_TOLERANCE_M or yaw_error > DOCK_YAW_TOLERANCE_RAD:
            raise PreparationError(
                f"pickup dock tolerance failed: position={position_error:.4f} yaw={yaw_error:.4f}")
        product_pose = self._product_poses.get(self.product_id)
        if product_pose is None:
            raise PreparationError("selected product pose is unavailable at pickup dock")
        robot_x, robot_y, robot_yaw = _pose_tuple(self._robot_pose)
        dx = product_pose.pose.position.x - robot_x
        dy = product_pose.pose.position.y - robot_y
        relative = (
            math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy,
            -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy,
        )
        expected_dx = self.selected.reset_pose[0] - self.selected.dock[0]
        expected_dy = self.selected.reset_pose[1] - self.selected.dock[1]
        expected_relative = (
            math.cos(self.selected.dock[2]) * expected_dx +
            math.sin(self.selected.dock[2]) * expected_dy,
            -math.sin(self.selected.dock[2]) * expected_dx +
            math.cos(self.selected.dock[2]) * expected_dy,
        )
        if math.hypot(relative[0] - expected_relative[0], relative[1] - expected_relative[1]) > PRODUCT_RELATIVE_TOLERANCE_M:
            raise PreparationError("selected product is not in the expected pickup geometry")

    def prepare(self) -> None:
        # Evaluate existing manipulation authority before this node publishes
        # its own preparation status, so an active/deployed supervisor cannot
        # be hidden by the runner's status message.
        self._check_preconditions()
        # The native bootstrap owns startup detachment.  Verify it before any
        # reset, gripper, or arm command; this runner never republishes blind
        # detach requests and remains limited to products 102 and 103.
        self._verify_attachment_bootstrap()
        self._set_status(
            ManipulatorStatus.STARTING, False, False,
            f"Preparing product {self.product_id} from the current AMR pose")
        self._reset_selected_product()
        self._set_status(
            ManipulatorStatus.STOWED_EMPTY, True, False,
            f"Product {self.product_id} reset; navigating to pickup dock")
        assert self._robot_pose is not None
        self._retreat_from_other_pickup_dock()
        dock_position_error, dock_yaw_error = _pose_error(self._robot_pose, self.selected.dock)
        if dock_position_error > DOCK_POSITION_TOLERANCE_M or dock_yaw_error > DOCK_YAW_TOLERANCE_RAD:
            approach_position_error, approach_yaw_error = _pose_error(self._robot_pose, self.selected.approach)
            if approach_position_error > 0.07 or approach_yaw_error > 0.15:
                self._navigate(self.selected.approach, precise=False)
            amcl_generation_before_dock = self._amcl_pose_generation
            localized_dock = self._navigate(self.selected.dock, precise=True)
            terminal_amcl_generation = self._amcl_pose_generation
            if terminal_amcl_generation <= amcl_generation_before_dock:
                raise PreparationError("no AMCL sample was received during dock navigation")
            self._wait_stationary()
            self._spin_until(
                lambda: self._robot_pose is not None and self._fresh(self._robot_pose_at),
                2.0, "fresh physical dock pose")
            assert self._robot_pose is not None
            physical_dock = _pose_tuple(self._robot_pose)
            physical_position_error, physical_yaw_error = _pose_error(
                self._robot_pose, self.selected.dock)
            if physical_position_error > DOCK_POSITION_TOLERANCE_M or \
                    physical_yaw_error > DOCK_YAW_TOLERANCE_RAD:
                initial_bias = _dock_bias(physical_dock, localized_dock)
                self.get_logger().info(
                    "Dock localization discrepancy bounded before recovery: "
                    f"bias=({initial_bias[0]:.4f}, {initial_bias[1]:.4f}, "
                    f"{initial_bias[2]:.4f})")
                # Let the localization transform propagate while the base
                # remains stopped before planning the registered recovery.
                self._wait_stationary()
                # Recover clearance using the registered, collision-checked
                # approach leg before the exact dock leg. PlacementFollowPath
                # permits bounded reverse motion; no ad-hoc waypoint or
                # tolerance is added.
                # The retreat endpoint checks only registered XY clearance;
                # it keeps the reverse leg from forcing a yaw correction beside
                # the pedestal.  Once clear, the normal endpoint aligns yaw
                # with the exact registered approach pose before re-docking.
                amcl_generation_before_recovery = self._amcl_pose_generation
                self._navigate(self.selected.approach, precise=True, retreat=True)
                localized_approach = self._navigate(self.selected.approach, precise=False)
                terminal_amcl_generation = self._amcl_pose_generation
                if terminal_amcl_generation <= amcl_generation_before_recovery:
                    raise PreparationError(
                        "no AMCL sample was received during approach recovery")
                self._wait_stationary()
                self._spin_until(
                    lambda: self._robot_pose is not None and self._fresh(self._robot_pose_at),
                    2.0, "fresh physical approach pose")
                assert self._robot_pose is not None
                physical_approach = _pose_tuple(self._robot_pose)
                self._relocalize_at_reference(
                    physical_approach, localized_approach,
                    terminal_amcl_generation, "Approach")
                self._wait_stationary()
                if self.selected.egress is None:
                    raise PreparationError("selected pickup station has no registered egress")
                self._navigate(self.selected.egress, precise=True)
                self._wait_stationary()
                self._navigate(self.selected.dock, precise=True)
        self._verify_dock_and_product_geometry()
        self._set_status(
            ManipulatorStatus.STOWED_EMPTY, True, False,
            f"Product {self.product_id} prepared at pickup dock")

    def fail_closed(self, detail: str) -> None:
        self.get_logger().error(detail)
        self._set_status(ManipulatorStatus.FAULT, False, False, detail)
        end = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def _acquire_lock() -> object:
    domain = os.environ.get("ROS_DOMAIN_ID", "unset")
    path = Path(f"/tmp/amr_gate6_product_test_{domain}.lock")
    handle = path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        handle.close()
        raise PreparationError(
            f"another Gate 6 product test is already running for ROS_DOMAIN_ID={domain}") from error
    return handle


def main() -> int:
    lock = None
    node: Optional[ProductPreparation] = None
    child: Optional[subprocess.Popen] = None
    try:
        lock = _acquire_lock()
        rclpy.init()
        node = ProductPreparation()
        node.prepare()
        product_id = node.product_id
        node.get_logger().info(
            f"GATE6 PRODUCT PREP PASS product_id={product_id}; starting existing mass stage")
        node.destroy_node()
        node = None
        rclpy.shutdown()

        child = subprocess.Popen(
            [
                "ros2", "launch", "amr_manipulation", "gate6_mass_stage.launch.py",
                f"product_id:={product_id}",
            ],
            start_new_session=True,
        )
        return child.wait()
    except PreparationError as error:
        if node is not None and rclpy.ok():
            node.fail_closed(str(error))
        else:
            print(f"GATE6 PRODUCT PREP FAIL: {error}", flush=True)
        return 2
    except KeyboardInterrupt:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait()
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if lock is not None:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
            lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
