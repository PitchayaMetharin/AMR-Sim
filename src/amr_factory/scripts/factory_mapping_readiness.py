#!/usr/bin/env python3
"""Bounded, observable startup gates for the factory mapping graph.

Each invocation observes one prerequisite boundary and exits with status zero
only after that boundary is proven from live ROS state.  Receipt ages use the
steady clock so simulated time cannot make stale sensor, map, or TF evidence
look current.  A timeout, malformed message, unavailable lifecycle service,
or unavailable action server returns a non-zero status and leaves the launch
layer responsible for shutting down the graph.
"""

from __future__ import annotations

import math
import time
from typing import Dict, Iterable, Optional, Tuple

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


ADAPTER_NODES = (
    "base_adapter_node",
    "front_lidar_adapter_node",
    "rear_lidar_adapter_node",
    "imu_adapter_node",
    "product_camera_adapter_node",
)
PLANNER_NODES = ("planner_server", "smoother_server")
CONTROLLER_NODES = ("controller_server",)
MISSION_NODES = ("mission_supervisor_node",)

# These are the existing consumer freshness windows: the frontier explorer
# treats maps as stale after 3 s and its TF lookup window is 1 s.
SCAN_MAX_AGE_S = 1.0
MAP_MAX_AGE_S = 3.0
TF_MAX_AGE_S = 1.0
SERVICE_CALL_MAX_AGE_S = 0.5


def fresh_receipt(received_at: Optional[float], max_age_s: float,
                  now: Optional[float] = None) -> bool:
    """Return true only for a non-future receipt inside a bounded age window."""
    if received_at is None:
        return False
    current = time.monotonic() if now is None else now
    age = current - received_at
    return 0.0 <= age <= max_age_s


def valid_factory_map(message: OccupancyGrid) -> bool:
    """Validate the minimum map evidence needed to release navigation."""
    if message is None or message.header.frame_id != "map":
        return False
    info = message.info
    try:
        width = int(info.width)
        height = int(info.height)
        resolution = float(info.resolution)
        origin_values = (
            float(info.origin.position.x),
            float(info.origin.position.y),
            float(info.origin.position.z),
            float(info.origin.orientation.x),
            float(info.origin.orientation.y),
            float(info.origin.orientation.z),
            float(info.origin.orientation.w),
        )
    except (AttributeError, TypeError, ValueError):
        return False
    if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0.0:
        return False
    if not all(math.isfinite(value) for value in origin_values):
        return False
    if len(message.data) != width * height:
        return False
    # At least one known free cell proves the map is more than an all-unknown
    # startup sample.  OccupancyGrid uses 0 for free and -1 for unknown.
    return any(value == 0 for value in message.data)


def _frame_name(value: str) -> str:
    return value.strip().lstrip("/")


class MappingReadiness(Node):
    """Observe one bounded launch prerequisite."""

    def __init__(self) -> None:
        super().__init__("factory_mapping_readiness")
        self.declare_parameter("stage", "adapters")
        self.declare_parameter("timeout_sec", 60.0)
        self._stage = str(self.get_parameter("stage").value).strip().lower()
        self._timeout_s = float(self.get_parameter("timeout_sec").value)
        if self._stage not in {"adapters", "map", "planner", "controller", "mission"}:
            raise ValueError(f"unknown mapping readiness stage: {self._stage}")
        if not math.isfinite(self._timeout_s) or self._timeout_s <= 0.0:
            raise ValueError("timeout_sec must be a finite positive value")

        self._last_scan_at: Optional[float] = None
        self._last_map_at: Optional[float] = None
        self._map_valid = False
        self._tf_receipt_at: Dict[Tuple[str, str], float] = {}

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            LaserScan, "/amr/sensors/front_lidar/scan", self._scan_callback, scan_qos)
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        self.create_subscription(TFMessage, "/tf", self._tf_callback, 100)

        state_nodes = self._state_nodes_for_stage(self._stage)
        self._state_clients = {
            node_name: self.create_client(
                GetState, f"/amr/{node_name}/get_state")
            for node_name in state_nodes
        }
        self._mission_action_client = None
        if self._stage == "mission":
            self._mission_action_client = ActionClient(
                self, NavigateToPose, "/amr/mission/navigate_to_pose")

    @staticmethod
    def _state_nodes_for_stage(stage: str) -> Iterable[str]:
        if stage == "adapters":
            return ADAPTER_NODES
        if stage == "planner":
            return PLANNER_NODES
        if stage == "controller":
            return CONTROLLER_NODES
        if stage == "mission":
            return MISSION_NODES
        return ()

    def _scan_callback(self, _message: LaserScan) -> None:
        self._last_scan_at = time.monotonic()

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._map_valid = valid_factory_map(message)
        self._last_map_at = time.monotonic() if self._map_valid else None

    def _tf_callback(self, message: TFMessage) -> None:
        received_at = time.monotonic()
        for transform in message.transforms:
            parent = _frame_name(transform.header.frame_id)
            child = _frame_name(transform.child_frame_id)
            if parent and child:
                self._tf_receipt_at[(parent, child)] = received_at

    def _fresh_tf(self, parent: str, child: str,
                  now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else now
        direct = self._tf_receipt_at.get((parent, child))
        if fresh_receipt(direct, TF_MAX_AGE_S, current):
            return True
        # The graph publishes map->odom from SLAM and odom->base_footprint from
        # the EKF.  Treat their fresh composition as the required map->base
        # evidence while still checking each received edge independently.
        if (parent, child) == ("map", "base_footprint"):
            return (
                self._fresh_tf("map", "odom", current) and
                self._fresh_tf("odom", "base_footprint", current))
        return False

    def _active_state(self, node_name: str, deadline: float) -> bool:
        client = self._state_clients[node_name]
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            if not client.wait_for_service(
                    timeout_sec=min(SERVICE_CALL_MAX_AGE_S, remaining)):
                return False
            future = client.call_async(GetState.Request())
            call_deadline = min(
                deadline, time.monotonic() + SERVICE_CALL_MAX_AGE_S)
            while rclpy.ok() and not future.done() and time.monotonic() < call_deadline:
                rclpy.spin_once(
                    self, timeout_sec=min(0.05, max(0.01, call_deadline - time.monotonic())))
            if not future.done():
                future.cancel()
                return False
            try:
                response = future.result()
            except Exception:
                return False
            return bool(
                response is not None and
                response.current_state.id == State.PRIMARY_STATE_ACTIVE)
        return False

    def _all_active(self, deadline: float) -> bool:
        return all(
            self._active_state(node_name, deadline)
            for node_name in self._state_clients)

    def _stage_ready(self, deadline: float) -> bool:
        if self._stage == "adapters":
            if not self._all_active(deadline):
                return False
            # Lifecycle service calls spin the node and may deliver newer scan
            # and TF callbacks.  Take the steady-clock snapshot after those
            # callbacks so fresh evidence is never treated as future-dated.
            now = time.monotonic()
            return (
                fresh_receipt(self._last_scan_at, SCAN_MAX_AGE_S, now) and
                self._fresh_tf("odom", "base_footprint", now))
        now = time.monotonic()
        if self._stage == "map":
            return (
                self._map_valid and
                fresh_receipt(self._last_map_at, MAP_MAX_AGE_S, now) and
                self._fresh_tf("map", "odom", now) and
                self._fresh_tf("map", "base_footprint", now))
        if self._stage == "planner":
            return self._all_active(deadline)
        if self._stage == "controller":
            return self._all_active(deadline)
        if self._stage == "mission":
            return (
                self._all_active(deadline) and
                self._mission_action_client is not None and
                self._mission_action_client.wait_for_server(timeout_sec=0.1))
        return False

    def wait_until_ready(self) -> bool:
        deadline = time.monotonic() + self._timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._stage_ready(deadline):
                self.get_logger().info(
                    f"factory mapping readiness passed: {self._stage}")
                return True
        self.get_logger().error(
            f"factory mapping readiness timed out: {self._stage}")
        return False


def main() -> int:
    rclpy.init()
    node: Optional[MappingReadiness] = None
    try:
        node = MappingReadiness()
        return 0 if node.wait_until_ready() else 1
    except (RuntimeError, ValueError, TypeError) as error:
        print(f"FACTORY MAPPING READINESS FAULT: {error}", flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
