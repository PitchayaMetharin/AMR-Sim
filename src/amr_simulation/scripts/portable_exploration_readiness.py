#!/usr/bin/env python3
"""Local staged readiness gate for the portable exploration graph.

The gate is a one-shot process: it remains alive without an elapsed-time
timeout while required processes are alive, reports the current unmet stage,
and exits zero only after the final evidence recheck.  A process loss,
explicit base/manipulator fault, shutdown, or user stop exits non-zero so the
launch supervisor shuts down the graph instead of releasing Explorer.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from amr_interfaces.msg import BaseStatus, ManipulatorStatus
from builtin_interfaces.msg import Time as RosTime
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


STAGES = (
    "adapters_authority",
    "slam_map",
    "planner_smoother",
    "controller",
    "mission_final",
)


def stage_prerequisites(stage: str) -> Tuple[str, ...]:
    """Return the target stage and every prerequisite stage before it."""

    try:
        index = STAGES.index(stage)
    except ValueError as error:
        raise ValueError(f"unknown portable readiness stage: {stage}") from error
    return STAGES[:index + 1]


MAX_UNMET_LOG_INTERVAL_S = 60.0
# This is deliberately not a startup timeout.  The gate waits while its
# required processes live and only terminates on a proof or an explicit fault.
TERMINAL_TIMEOUT_S = None

RECEIPT_MAX_AGE_S = 1.0
STATUS_MAX_AGE_S = 0.2
TF_MAX_AGE_S = 1.0
SLAM_TF_FUTURE_TOLERANCE_S = 1.0
COSTMAP_MAX_AGE_S = 2.0

ADAPTER_NODES = (
    "base_adapter_node",
    "front_lidar_adapter_node",
    "rear_lidar_adapter_node",
    "imu_adapter_node",
    "product_camera_adapter_node",
    "wheel_odometry_node",
    "front_lidar_perception_node",
    "rear_lidar_perception_node",
    "command_arbitration_node",
)
SLAM_NODE = "slam_toolbox"
PLANNER_NODES = ("planner_server", "smoother_server")
CONTROLLER_NODES = ("controller_server",)
MISSION_NODE = "mission_supervisor_node"
ALL_LIFECYCLE_NODES = ADAPTER_NODES + PLANNER_NODES + CONTROLLER_NODES + (MISSION_NODE, "health_supervisor_node")
REQUIRED_PROCESS_NODES = ALL_LIFECYCLE_NODES + (
    SLAM_NODE,
    "lifecycle_manager_planning",
    "lifecycle_manager_controller",
    "portable_stow_authority",
)


def fresh_receipt(received_at: float, now: float, max_age: float) -> bool:
    """Use steady-clock receipt age and reject both stale and future samples."""

    try:
        age = float(now) - float(received_at)
        return math.isfinite(age) and 0.0 <= age <= float(max_age)
    except (TypeError, ValueError):
        return False


def _occupancy_data_valid(data: Iterable[int], *, require_known: bool) -> bool:
    """Validate the bounded integer payload of an OccupancyGrid."""

    try:
        values = list(data)
    except TypeError:
        return False
    if not values:
        return False
    if not all(type(value) is int and -1 <= value <= 100 for value in values):
        return False
    return not require_known or any(value != -1 for value in values)


def map_is_valid(data: Iterable[int]) -> bool:
    """Accept a non-empty bounded map containing at least one known cell."""

    return _occupancy_data_valid(data, require_known=True)


def valid_occupancy_grid(message: OccupancyGrid, *, require_known: bool) -> bool:
    """Validate OccupancyGrid geometry and payload before recording receipt."""

    try:
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)
        origin = message.info.origin
        origin_values = (
            float(origin.position.x),
            float(origin.position.y),
            float(origin.position.z),
            float(origin.orientation.x),
            float(origin.orientation.y),
            float(origin.orientation.z),
            float(origin.orientation.w),
        )
        data = list(message.data)
    except (AttributeError, TypeError, ValueError):
        return False
    if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0.0:
        return False
    if not all(math.isfinite(value) for value in origin_values):
        return False
    if len(data) != width * height:
        return False
    return _occupancy_data_valid(data, require_known=require_known)


def _state_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _map_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _stamp_seconds(stamp: RosTime) -> float:
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return math.nan


def _stamp_not_future(stamp: RosTime, now: RosTime, tolerance: float = 0.0) -> bool:
    value = _stamp_seconds(stamp)
    current = _stamp_seconds(now)
    if not math.isfinite(value) or value < 0.0:
        return False
    # Sim time may still be at zero during startup.  A zero-stamped startup
    # sample is admissible, but a positive sample cannot be accepted while the
    # clock is still zero because it would be future-dated by construction.
    if current <= 0.0:
        return value == 0.0
    return value > 0.0 and value <= current + tolerance


def _finite_transform(transform) -> bool:
    try:
        values = (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return False
        quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
        return quaternion_norm > 0.0
    except (AttributeError, TypeError, ValueError):
        return False


class PortableExplorationReadiness(Node):
    """Observe only local ROS graph/evidence and release Explorer once."""

    def __init__(self) -> None:
        super().__init__("portable_exploration_readiness")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        if not self.has_parameter("stage"):
            self.declare_parameter("stage", STAGES[-1])
        configured_stage = str(self.get_parameter("stage").value)
        if configured_stage not in STAGES:
            raise ValueError(f"unknown portable readiness stage: {configured_stage}")

        self._started_at = time.monotonic()
        self._last_unmet_log_at = 0.0
        self._last_unmet: Tuple[str, ...] = ()
        self._ready = False
        self._fault_reason = ""
        self._target_stage = configured_stage
        self._stage = configured_stage
        self._unmet: Tuple[str, ...] = ("waiting for local graph",)
        self._seen_processes = set()
        self._lifecycle_state: Dict[str, int] = {}
        self._lifecycle_pending: Dict[str, object] = {}

        self._front_scan_at = 0.0
        self._map_at = 0.0
        self._map: Optional[OccupancyGrid] = None
        self._global_costmap_at = 0.0
        self._global_costmap: Optional[OccupancyGrid] = None
        self._local_costmap_at = 0.0
        self._local_costmap: Optional[OccupancyGrid] = None
        self._base_status: Optional[BaseStatus] = None
        self._base_at = 0.0
        self._base_boot = 0
        self._base_sequence = 0
        self._base_valid = False
        self._manipulator_status: Optional[ManipulatorStatus] = None
        self._manipulator_at = 0.0
        self._manipulator_boot = 0
        self._manipulator_sequence = 0
        self._manipulator_valid = False
        self._tf_edges: Dict[Tuple[str, str], Tuple[object, float]] = {}

        self._readiness_pub = self.create_publisher(
            String, "/amr/exploration/readiness", _state_qos())
        self._front_scan_sub = self.create_subscription(
            LaserScan,
            "/amr/sensors/front_lidar/scan",
            self._front_scan_callback,
            qos_profile_sensor_data,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, _map_qos())
        self._global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/amr/global_costmap/costmap",
            self._global_costmap_callback,
            _map_qos(),
        )
        self._local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/amr/local_costmap/costmap",
            self._local_costmap_callback,
            _state_qos(),
        )
        self._base_sub = self.create_subscription(
            BaseStatus, "/amr/base/status", self._base_callback, _state_qos())
        self._manipulator_sub = self.create_subscription(
            ManipulatorStatus,
            "/amr/manipulation/status",
            self._manipulator_callback,
            _state_qos(),
        )
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._tf_sub = self.create_subscription(TFMessage, "/tf", self._tf_callback, tf_qos)
        self._tf_static_sub = self.create_subscription(TFMessage, "/tf_static", self._tf_callback, _map_qos())
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._mission_client = ActionClient(
            self, NavigateToPose, "/amr/mission/navigate_to_pose")
        self._lifecycle_clients = {
            name: self.create_client(GetState, f"/amr/{name}/get_state")
            for name in ALL_LIFECYCLE_NODES
        }
        self._timer = self.create_timer(0.2, self._tick)
        self._tick()

    # Evidence callbacks intentionally record steady receipt times.  Header
    # stamps are checked only for future data and never used as an age clock.
    def _front_scan_callback(self, message: LaserScan) -> None:
        now_ros = self.get_clock().now().to_msg()
        if not _stamp_not_future(message.header.stamp, now_ros):
            return
        if not message.ranges or not all(math.isfinite(value) or math.isinf(value) for value in message.ranges):
            return
        self._front_scan_at = time.monotonic()

    def _map_callback(self, message: OccupancyGrid) -> None:
        now_ros = self.get_clock().now().to_msg()
        if not _stamp_not_future(message.header.stamp, now_ros):
            return
        if not valid_occupancy_grid(message, require_known=True):
            return
        self._map = message
        self._map_at = time.monotonic()

    def _global_costmap_callback(self, message: OccupancyGrid) -> None:
        if not _stamp_not_future(message.header.stamp, self.get_clock().now().to_msg()):
            return
        if not valid_occupancy_grid(message, require_known=False):
            return
        self._global_costmap = message
        self._global_costmap_at = time.monotonic()

    def _local_costmap_callback(self, message: OccupancyGrid) -> None:
        if not _stamp_not_future(message.header.stamp, self.get_clock().now().to_msg()):
            return
        if not valid_occupancy_grid(message, require_known=False):
            return
        self._local_costmap = message
        self._local_costmap_at = time.monotonic()

    @staticmethod
    def _advance_identity(message, last_boot: int, last_sequence: int) -> Tuple[bool, int, int]:
        try:
            boot = int(message.source_boot_id)
            sequence = int(message.sequence)
        except (AttributeError, TypeError, ValueError):
            return False, last_boot, last_sequence
        if boot <= 0 or sequence <= 0:
            return False, last_boot, last_sequence
        if boot == last_boot and sequence <= last_sequence:
            return False, last_boot, last_sequence
        return True, boot, sequence

    def _base_callback(self, message: BaseStatus) -> None:
        if getattr(message, "state", None) == BaseStatus.FAULT:
            self._fault_reason = "explicit BaseStatus FAULT"
        if not _stamp_not_future(message.header.stamp, self.get_clock().now().to_msg()):
            self._base_valid = False
            return
        accepted, boot, sequence = self._advance_identity(
            message, self._base_boot, self._base_sequence)
        if not accepted:
            self._base_valid = False
            return
        self._base_boot, self._base_sequence = boot, sequence
        self._base_status = message
        self._base_at = time.monotonic()
        self._base_valid = bool(
            message.valid
            and message.state == BaseStatus.READY
            and message.reason == BaseStatus.REASON_READY
        )

    def _manipulator_callback(self, message: ManipulatorStatus) -> None:
        if getattr(message, "state", None) == ManipulatorStatus.FAULT:
            self._fault_reason = "explicit portable manipulator FAULT"
        if not _stamp_not_future(message.header.stamp, self.get_clock().now().to_msg()):
            self._manipulator_valid = False
            return
        accepted, boot, sequence = self._advance_identity(
            message, self._manipulator_boot, self._manipulator_sequence)
        if not accepted:
            self._manipulator_valid = False
            return
        self._manipulator_boot, self._manipulator_sequence = boot, sequence
        self._manipulator_status = message
        self._manipulator_at = time.monotonic()
        self._manipulator_valid = bool(
            message.valid
            and message.state == ManipulatorStatus.STOWED_EMPTY
            and message.base_motion_allowed
            and not message.product_attached
            and message.product_id == ""
        )

    def _tf_callback(self, message: TFMessage) -> None:
        received_at = time.monotonic()
        now_ros = self.get_clock().now().to_msg()
        for transform in message.transforms:
            parent = transform.header.frame_id.strip().lstrip("/")
            child = transform.child_frame_id.strip().lstrip("/")
            if not parent or not child:
                continue
            future_tolerance = (
                SLAM_TF_FUTURE_TOLERANCE_S
                if (parent, child) == ("map", "odom") else 0.0
            )
            if not _stamp_not_future(transform.header.stamp, now_ros, future_tolerance):
                continue
            if not _finite_transform(transform.transform):
                continue
            self._tf_edges[(parent, child)] = (
                transform,
                received_at,
            )

    def _graph_nodes(self) -> set[Tuple[str, str]]:
        try:
            return set(self.get_node_names_and_namespaces())
        except Exception:
            return set()

    def _refresh_lifecycle_states(self, graph: set[Tuple[str, str]]) -> None:
        for name in REQUIRED_PROCESS_NODES:
            if (name, "/amr") in graph:
                self._seen_processes.add(name)
            elif name in self._seen_processes:
                self._fault_reason = f"required portable process exited: {name}"
        for name, client in self._lifecycle_clients.items():
            key = (f"{name}", "/amr")
            if key in graph:
                self._seen_processes.add(name)
                if client.service_is_ready() and name not in self._lifecycle_pending:
                    try:
                        future = client.call_async(GetState.Request())
                        self._lifecycle_pending[name] = future
                        future.add_done_callback(
                            lambda result, node_name=name: self._lifecycle_done(node_name, result))
                    except Exception as error:
                        self._fault_reason = f"lifecycle state query failed for {name}: {error}"
            elif name in self._seen_processes:
                self._fault_reason = f"required lifecycle process exited: {name}"

    def _lifecycle_done(self, name: str, future) -> None:
        self._lifecycle_pending.pop(name, None)
        try:
            response = future.result()
            self._lifecycle_state[name] = int(response.current_state.id)
        except Exception as error:
            self._lifecycle_state[name] = 0
            self._unmet = (f"lifecycle query unavailable: {name}",)
            self.get_logger().warning(f"lifecycle query unavailable for {name}: {error}")

    def _active(self, name: str, graph: set[Tuple[str, str]]) -> bool:
        return (name, "/amr") in graph and self._lifecycle_state.get(name) == State.PRIMARY_STATE_ACTIVE

    def _process_present(self, name: str, graph: set[Tuple[str, str]]) -> bool:
        return (name, "/amr") in graph

    def _fresh_base_ready(self, now: float) -> bool:
        return bool(
            self._base_status is not None
            and self._base_valid
            and fresh_receipt(self._base_at, now, STATUS_MAX_AGE_S)
        )

    def _fresh_stow_empty(self, now: float) -> bool:
        return bool(
            self._manipulator_status is not None
            and self._manipulator_valid
            and fresh_receipt(self._manipulator_at, now, STATUS_MAX_AGE_S)
        )

    def _fresh_tf_edge(self, parent: str, child: str, now: float) -> bool:
        edge = self._tf_edges.get((parent, child))
        if edge is None or not fresh_receipt(edge[1], now, TF_MAX_AGE_S):
            return False
        transform = edge[0]
        try:
            future_tolerance = (
                SLAM_TF_FUTURE_TOLERANCE_S
                if (parent, child) == ("map", "odom") else 0.0
            )
            return (
                _finite_transform(transform.transform)
                and _stamp_not_future(
                    transform.header.stamp,
                    self.get_clock().now().to_msg(),
                    future_tolerance,
                )
            )
        except (AttributeError, TypeError, ValueError):
            return False

    def _fresh_composed_map_base(self, now: float) -> bool:
        if not self._fresh_tf_edge("map", "base_footprint", now):
            # Prefer the locally observed composed edge.  If a TF implementation
            # publishes only its two source edges, verify the same composition
            # through the buffer before accepting the stage.
            try:
                if not (
                    self._fresh_tf_edge("map", "odom", now)
                    and self._fresh_tf_edge("odom", "base_footprint", now)
                ):
                    return False
                transform = self._tf_buffer.lookup_transform(
                    "map", "base_footprint", RclpyTime())
                if not _finite_transform(transform.transform):
                    return False
                if not _stamp_not_future(
                    transform.header.stamp,
                    self.get_clock().now().to_msg(),
                    SLAM_TF_FUTURE_TOLERANCE_S,
                ):
                    return False
            except Exception:
                return False
        return True

    def _evaluate(self, graph: set[Tuple[str, str]], now: float) -> Tuple[str, Tuple[str, ...]]:
        target_stage = getattr(self, "_target_stage", STAGES[-1])
        try:
            required_stages = stage_prerequisites(target_stage)
        except ValueError:
            return STAGES[0], (f"unknown readiness target stage: {target_stage}",)
        target_index = len(required_stages) - 1
        unmet: List[str] = []
        for name in ADAPTER_NODES:
            if not self._active(name, graph):
                unmet.append(f"active lifecycle node: {name}")
        if not fresh_receipt(self._front_scan_at, now, RECEIPT_MAX_AGE_S):
            unmet.append("fresh front scan")
        if not self._fresh_tf_edge("odom", "base_footprint", now):
            unmet.append("fresh odom->base_footprint TF")
        if not self._fresh_base_ready(now):
            unmet.append("fresh BaseStatus READY")
        if not self._fresh_stow_empty(now):
            unmet.append("fresh portable STOWED_EMPTY")
        if unmet:
            return STAGES[0], tuple(unmet)
        if target_index == 0:
            return STAGES[0], ()

        if not self._process_present(SLAM_NODE, graph):
            unmet.append("active slam_toolbox")
        if (
            self._map is None
            or not valid_occupancy_grid(self._map, require_known=True)
            or not fresh_receipt(self._map_at, now, RECEIPT_MAX_AGE_S)
        ):
            unmet.append("fresh valid non-all-unknown map")
        if not self._fresh_tf_edge("map", "odom", now):
            unmet.append("fresh map->odom TF")
        if not self._fresh_composed_map_base(now):
            unmet.append("fresh composed map->base_footprint TF")
        if unmet:
            return STAGES[1], tuple(unmet)
        if target_index == 1:
            return STAGES[1], ()

        for name in PLANNER_NODES:
            if not self._active(name, graph):
                unmet.append(f"active planner lifecycle node: {name}")
        if (
            self._global_costmap is None
            or not valid_occupancy_grid(self._global_costmap, require_known=False)
            or not fresh_receipt(self._global_costmap_at, now, COSTMAP_MAX_AGE_S)
        ):
            unmet.append("fresh global costmap")
        if unmet:
            return STAGES[2], tuple(unmet)
        if target_index == 2:
            return STAGES[2], ()

        for name in CONTROLLER_NODES:
            if not self._active(name, graph):
                unmet.append(f"active controller lifecycle node: {name}")
        if (
            self._local_costmap is None
            or not valid_occupancy_grid(self._local_costmap, require_known=False)
            or not fresh_receipt(self._local_costmap_at, now, COSTMAP_MAX_AGE_S)
        ):
            unmet.append("fresh local costmap")
        if unmet:
            return STAGES[3], tuple(unmet)
        if target_index == 3:
            return STAGES[3], ()

        if not self._active(MISSION_NODE, graph):
            unmet.append("active mission_supervisor_node")
        if not self._mission_client.server_is_ready():
            unmet.append("mission NavigateToPose action server")
        # Final recheck: all lifecycle nodes, evidence, both costmaps, TF,
        # base, stow, and scan must still be fresh at the release boundary.
        for name in ALL_LIFECYCLE_NODES:
            if not self._active(name, graph):
                unmet.append(f"final active lifecycle node: {name}")
        if not fresh_receipt(self._front_scan_at, now, RECEIPT_MAX_AGE_S):
            unmet.append("final fresh front scan")
        if not self._fresh_base_ready(now):
            unmet.append("final fresh BaseStatus READY")
        if not self._fresh_stow_empty(now):
            unmet.append("final fresh portable STOWED_EMPTY")
        if (
            self._map is None
            or not valid_occupancy_grid(self._map, require_known=True)
            or not fresh_receipt(self._map_at, now, RECEIPT_MAX_AGE_S)
        ):
            unmet.append("final fresh map")
        if (
            self._global_costmap is None
            or not valid_occupancy_grid(self._global_costmap, require_known=False)
            or not fresh_receipt(self._global_costmap_at, now, COSTMAP_MAX_AGE_S)
        ):
            unmet.append("final fresh global costmap")
        if (
            self._local_costmap is None
            or not valid_occupancy_grid(self._local_costmap, require_known=False)
            or not fresh_receipt(self._local_costmap_at, now, COSTMAP_MAX_AGE_S)
        ):
            unmet.append("final fresh local costmap")
        if not self._fresh_tf_edge("odom", "base_footprint", now):
            unmet.append("final fresh odom->base_footprint TF")
        if not self._fresh_tf_edge("map", "odom", now):
            unmet.append("final fresh map->odom TF")
        if not self._fresh_composed_map_base(now):
            unmet.append("final composed map->base_footprint TF")
        return STAGES[4], tuple(unmet)

    def _publish_report(self) -> None:
        state = "FAULT" if self._fault_reason else ("READY" if not self._unmet else "UNMET")
        report = {
            "state": state,
            "stage": self._stage,
            "ready": state == "READY",
            "unmet": list(self._unmet),
            "fault": self._fault_reason,
        }
        message = String()
        message.data = json.dumps(report, sort_keys=True)
        self._readiness_pub.publish(message)

    def _tick(self) -> None:
        if self._fault_reason:
            self._unmet = ()
            self._publish_report()
            if rclpy.ok():
                rclpy.shutdown()
            return
        graph = self._graph_nodes()
        self._refresh_lifecycle_states(graph)
        if self._fault_reason:
            self._unmet = ()
            self._publish_report()
            if rclpy.ok():
                rclpy.shutdown()
            return
        stage, unmet = self._evaluate(graph, time.monotonic())
        self._stage, self._unmet = stage, unmet
        now = time.monotonic()
        if not unmet:
            self._ready = True
            self._publish_report()
            self.get_logger().info("PORTABLE_EXPLORATION_READY")
            if rclpy.ok():
                rclpy.shutdown()
            return
        if unmet != self._last_unmet or now - self._last_unmet_log_at >= MAX_UNMET_LOG_INTERVAL_S:
            self.get_logger().warning(
                f"portable readiness unmet stage={stage} conditions={'; '.join(unmet)}")
            self._last_unmet = unmet
            self._last_unmet_log_at = now
        self._publish_report()


def main() -> int:
    rclpy.init()
    node = PortableExplorationReadiness()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        if not node._ready and not node._fault_reason:
            node._fault_reason = "readiness shutdown or user stop"
    finally:
        ready = node._ready
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
