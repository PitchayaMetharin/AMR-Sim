#!/usr/bin/env python3
"""Fail-closed frontier exploration through the AMR mission action boundary."""

import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from amr_interfaces.msg import BaseStatus, ManipulatorStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_algorithm import available_candidates, frontier_clusters


class FrontierExplorer(Node):
    """Select frontiers and delegate all motion to the existing mission node."""

    def __init__(self):
        super().__init__("frontier_explorer")
        self.declare_parameter("map_timeout_sec", 3.0)
        self.declare_parameter("tf_timeout_sec", 1.0)
        self.declare_parameter("no_frontier_updates", 3)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("max_goal_failures", 3)
        self.declare_parameter("authority_timeout_sec", 1.0)
        self.declare_parameter("startup_grace_sec", 15.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)
        self.declare_parameter("min_goal_distance_m", 0.3)
        self.map_timeout = float(self.get_parameter("map_timeout_sec").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.no_frontier_limit = int(self.get_parameter("no_frontier_updates").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.max_goal_failures = int(self.get_parameter("max_goal_failures").value)
        self.authority_timeout = float(self.get_parameter("authority_timeout_sec").value)
        self.startup_grace = float(self.get_parameter("startup_grace_sec").value)
        self.cancel_timeout = float(self.get_parameter("cancel_timeout_sec").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance_m").value)
        if any(value <= 0 for value in (
                self.map_timeout, self.tf_timeout, self.goal_timeout,
                self.authority_timeout, self.startup_grace, self.cancel_timeout,
                self.min_goal_distance)):
            raise ValueError("frontier explorer timeout parameters must be positive")
        if self.no_frontier_limit < 1 or self.max_goal_failures < 1:
            raise ValueError("frontier explorer count parameters must be positive")

        self.action_callback_group = ReentrantCallbackGroup()
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        self.base_sub = self.create_subscription(BaseStatus, "/amr/base/status", self._base_callback, 10)
        self.manipulator_sub = self.create_subscription(
            ManipulatorStatus, "/amr/manipulation/status", self._manipulator_callback, 1)
        self.stop_service = self.create_service(
            Trigger, "/amr/exploration/stop", self._stop_callback,
            callback_group=self.action_callback_group)
        self.action_client = ActionClient(
            self, NavigateToPose, "/amr/mission/navigate_to_pose",
            callback_group=self.action_callback_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self._tick)

        self.started_at = time.monotonic()
        self.last_map_at = None
        self.latest_map = None
        self.map_version = 0
        self.processed_map_version = -1
        self.last_base_status_at = None
        self.last_manipulator_status_at = None
        self.base_status = None
        self.manipulator_status = None
        self.active_goal = None
        self.goal_started_at = None
        self.cancel_requested = False
        self.cancel_started_at = None
        self.cancel_reason = ""
        self.cancel_event = threading.Event()
        self.blacklist = set()
        self.goal_failures = 0
        self.no_frontier_updates_seen = 0
        self.state = "WAITING_MAP"
        self.active_candidate = None
        self.fault_requested = False
        self.cancel_reason = ""

    def _map_callback(self, message):
        if message.header.frame_id != "map":
            self._fault("map frame is not map")
            return
        self.latest_map = message
        self.last_map_at = time.monotonic()
        self.map_version += 1

    def _base_callback(self, message):
        self.base_status = message
        self.last_base_status_at = time.monotonic()

    def _manipulator_callback(self, message):
        self.manipulator_status = message
        self.last_manipulator_status_at = time.monotonic()

    def _authority_ready(self):
        now = time.monotonic()
        base_ready = self.base_status is not None and self.base_status.valid and self.base_status.state == BaseStatus.READY
        manip_ready = self.manipulator_status is not None and self.manipulator_status.valid and self.manipulator_status.base_motion_allowed and self.manipulator_status.state in (
            ManipulatorStatus.STOWED_EMPTY, ManipulatorStatus.STOWED_LOADED)
        base_fresh = (
            self.last_base_status_at is not None
            and now - self.last_base_status_at <= self.authority_timeout)
        manipulator_fresh = (
            self.last_manipulator_status_at is not None
            and now - self.last_manipulator_status_at <= self.authority_timeout)
        return base_ready and manip_ready and base_fresh and manipulator_fresh

    def _tick(self):
        if self.state in ("FAULT", "COMPLETE", "STOPPED"):
            return
        now = time.monotonic()
        if self.latest_map is None:
            if now - self.started_at > max(self.map_timeout, self.startup_grace):
                self._fault("map did not arrive before timeout")
            return
        if now - self.last_map_at > self.map_timeout:
            self._fault("map is stale")
            return
        if self.state == "CANCELLING":
            if self.cancel_started_at is not None and now - self.cancel_started_at > self.cancel_timeout:
                self._fault("navigation cancellation was not confirmed before timeout")
            return
        if self.active_goal is not None:
            if self.cancel_requested and self.cancel_started_at is not None and now - self.cancel_started_at > self.cancel_timeout:
                self._fault("navigation cancellation was not confirmed before timeout")
            elif now - self.goal_started_at > self.goal_timeout and not self.cancel_requested:
                self._request_cancel("navigation goal timed out")
            return
        if self.processed_map_version == self.map_version:
            return
        self.processed_map_version = self.map_version
        self._select_frontier()

    def _select_frontier(self):
        grid = self.latest_map
        if not any(value == 0 for value in grid.data):
            # An all-unknown initial map is not evidence that exploration is
            # complete; the robot has not yet established a mapped free cell.
            self.no_frontier_updates_seen = 0
            self.state = "WAITING_MAP"
            return
        clusters = frontier_clusters(grid.info.width, grid.info.height, grid.data)
        candidates = available_candidates(clusters, self.blacklist)
        if not candidates:
            self.no_frontier_updates_seen += 1
            if self.no_frontier_updates_seen >= self.no_frontier_limit:
                self.state = "COMPLETE"
                self.get_logger().info("exploration complete: no reachable frontier remains")
            return
        self.no_frontier_updates_seen = 0
        if not self._authority_ready():
            if time.monotonic() - self.started_at > self.startup_grace:
                self._fault("fresh base/manipulator motion authority is unavailable")
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            if time.monotonic() - self.started_at > self.startup_grace:
                self._fault("map to base_footprint TF is unavailable")
            return
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        candidates = [
            candidate for candidate in candidates
            if ((grid.info.origin.position.x + (candidate[0] + 0.5) * grid.info.resolution - robot_x) ** 2
                + (grid.info.origin.position.y + (candidate[1] + 0.5) * grid.info.resolution - robot_y) ** 2)
            >= self.min_goal_distance ** 2
        ]
        if not candidates:
            self.no_frontier_updates_seen += 1
            if self.no_frontier_updates_seen >= self.no_frontier_limit:
                self.state = "COMPLETE"
                self.get_logger().info("exploration complete: remaining frontiers are within the robot footprint")
            return
        if not self.action_client.wait_for_server(timeout_sec=0.2):
            if time.monotonic() - self.started_at > self.startup_grace:
                self._fault("mission navigation action server is unavailable")
            return
        gx, gy = candidates[0]
        self.active_candidate = (gx, gy)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = grid.info.origin.position.x + (gx + 0.5) * grid.info.resolution
        pose.pose.position.y = grid.info.origin.position.y + (gy + 0.5) * grid.info.resolution
        pose.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.state = "NAVIGATING"
        self.goal_started_at = time.monotonic()
        self.cancel_requested = False
        self.cancel_started_at = None
        self.cancel_reason = ""
        self.cancel_event.clear()
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - middleware boundary
            self._fault(f"navigation goal dispatch failed: {exc}")
            return
        if not goal_handle or not goal_handle.accepted:
            if self.cancel_requested and self.cancel_reason == "operator requested exploration stop":
                self.active_goal = None
                self.cancel_event.set()
                self.state = "STOPPED"
                return
            if self.state in ("FAULT", "STOPPED"):
                return
            self._record_goal_failure("navigation goal was rejected")
            return
        self.active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result)
        if self.state in ("FAULT", "STOPPED") or self.cancel_requested:
            self.cancel_requested = True
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - middleware boundary
                self._fault(f"navigation cancellation request failed: {exc}")

    def _goal_result(self, future):
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:  # pragma: no cover - middleware boundary
            self._fault(f"navigation result was unavailable: {exc}")
            return
        was_cancel = self.cancel_requested
        self.active_goal = None
        self.goal_started_at = None
        self.cancel_event.set()
        self.cancel_requested = False
        if was_cancel:
            if status == GoalStatus.STATUS_CANCELED:
                self.state = "FAULT" if self.fault_requested or self.cancel_reason != "operator requested exploration stop" else "STOPPED"
            else:
                self._fault("navigation cancellation did not reach terminal canceled state")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state = "SCANNING"
            self.goal_failures = 0
            self.active_candidate = None
            return
        self._record_goal_failure(f"navigation goal ended with status {status}")

    def _record_goal_failure(self, detail):
        self.active_goal = None
        self.goal_started_at = None
        self.goal_failures += 1
        if self.goal_failures >= self.max_goal_failures:
            self._fault(detail + "; maximum navigation failures exceeded")
            return
        self.get_logger().warning(detail)
        # The failed goal is already removed from the active slot.  Blacklisting
        # the selected candidate requires the next map update to be associated
        # with the goal; a conservative failure marks the current first
        # candidate by storing its grid cell when it is dispatched.
        if hasattr(self, "active_candidate") and self.active_candidate is not None:
            self.blacklist.add(self.active_candidate)
        self.active_candidate = None
        self.state = "SCANNING"

    def _request_cancel(self, reason):
        if self.active_goal is None:
            self._fault(reason + "; no active goal to cancel")
            return
        self.cancel_requested = True
        self.state = "CANCELLING"
        self.cancel_reason = reason
        self.cancel_started_at = time.monotonic()
        self.get_logger().error(reason)
        try:
            self.active_goal.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - middleware boundary
            self._fault(f"navigation cancellation request failed: {exc}")

    def _stop_callback(self, _request, response):
        if self.state in ("COMPLETE", "STOPPED"):
            response.success = True
            response.message = "exploration already stopped"
            return response
        if self.state == "FAULT":
            response.success = False
            response.message = "exploration is faulted; restart the explorer"
            return response
        if self.active_goal is None:
            if self.state == "NAVIGATING":
                self.cancel_requested = True
                self.cancel_reason = "operator requested exploration stop"
                self.cancel_started_at = time.monotonic()
                self.state = "CANCELLING"
                if not self.cancel_event.wait(timeout=self.cancel_timeout):
                    self._fault("operator stop cancellation was not confirmed before timeout")
                    response.success = False
                    response.message = "cancellation was not confirmed; explorer faulted closed"
                    return response
                response.success = self.state == "STOPPED"
                response.message = "exploration stopped" if response.success else "explorer faulted during cancellation"
                return response
            self.state = "STOPPED"
            response.success = True
            response.message = "exploration stopped with no active navigation goal"
            return response
        self._request_cancel("operator requested exploration stop")
        if not self.cancel_event.wait(timeout=self.cancel_timeout):
            self._fault("operator stop cancellation was not confirmed before timeout")
            response.success = False
            response.message = "cancellation was not confirmed; explorer faulted closed"
            return response
        response.success = self.state == "STOPPED"
        response.message = "exploration stopped" if response.success else "explorer faulted during cancellation"
        return response

    def _fault(self, detail):
        if self.state == "FAULT":
            return
        self.fault_requested = True
        self.state = "FAULT"
        self.get_logger().error(f"exploration FAULT: {detail}")
        if self.active_goal is not None and not self.cancel_requested:
            self.cancel_requested = True
            self.cancel_reason = "fault"
            try:
                self.active_goal.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - middleware boundary
                self.get_logger().error(f"fault cancellation request failed: {exc}")


def main():
    rclpy.init()
    node = FrontierExplorer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
