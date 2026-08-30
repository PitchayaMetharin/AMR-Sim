#!/usr/bin/env python3

"""Exercise the live factory-localization acceptance conditions."""

import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class Gate4Acceptance(Node):
    def __init__(self) -> None:
        super().__init__("gate4_acceptance")
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.scan = None
        self.map_grid = None
        self.costmap = None
        self.create_subscription(
            LaserScan,
            "/amr/sensors/front_lidar/scan",
            self._receive_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._receive_map, transient_qos
        )
        self.create_subscription(
            OccupancyGrid,
            "/amr/global_costmap/costmap",
            self._receive_costmap,
            transient_qos,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_client = ActionClient(
            self, ComputePathToPose, "/amr/compute_path_to_pose"
        )

    def _receive_scan(self, message: LaserScan) -> None:
        self.scan = message

    def _receive_map(self, message: OccupancyGrid) -> None:
        self.map_grid = message

    def _receive_costmap(self, message: OccupancyGrid) -> None:
        self.costmap = message

    def wait_for_inputs(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.map_grid is not None and self.costmap is not None:
                return
        missing = [
            name
            for name, value in (
                ("front LiDAR", self.scan),
                ("map", self.map_grid),
                ("global costmap", self.costmap),
            )
            if value is None
        ]
        raise RuntimeError("timed out waiting for " + ", ".join(missing))

    def verify_localization(self) -> None:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time()
                )
                break
            except TransformException:
                continue
        else:
            raise RuntimeError("map -> base_footprint transform is unavailable")

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        if math.hypot(x + 4.5, y) > 0.10:
            raise RuntimeError(
                f"localized pose ({x:.3f}, {y:.3f}) differs from home (-4.5, 0.0)"
            )

        node_names = [full_name for full_name, _ in self.get_node_names_and_namespaces()]
        if any("slam" in name.lower() for name in node_names):
            raise RuntimeError("a SLAM node is running in factory localization mode")
        print(f"PASS localization: map -> base_footprint = ({x:.3f}, {y:.3f})")

    @staticmethod
    def _grid_value(grid: OccupancyGrid, x: float, y: float) -> int:
        column = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
            raise RuntimeError(f"point ({x}, {y}) lies outside occupancy grid")
        return grid.data[row * grid.info.width + column]

    def verify_station_obstacles(self) -> None:
        assert self.scan is not None
        assert self.map_grid is not None
        assert self.costmap is not None

        forward_returns = []
        for index, value in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + index * self.scan.angle_increment
            if abs(angle) <= math.radians(3.0) and math.isfinite(value):
                forward_returns.append(value)
        if not any(6.5 <= value <= 8.5 for value in forward_returns):
            raise RuntimeError(
                "front LiDAR did not observe the pickup_b shelf in the expected range"
            )

        for station_y in (3.0, 0.0, -3.0):
            map_value = self._grid_value(self.map_grid, 5.35, station_y)
            cost_value = self._grid_value(self.costmap, 5.35, station_y)
            if map_value < 90 or cost_value < 90:
                raise RuntimeError(
                    "shelf obstacle missing at "
                    f"(5.35, {station_y:.1f}): map={map_value}, costmap={cost_value}"
                )
        print("PASS obstacles: pickup shelves appear in LiDAR, map, and global costmap")

    def verify_paths(self, registry: dict) -> None:
        if not self.path_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Nav2 compute-path action is unavailable")

        targets = []
        for station_name, station in registry["stations"].items():
            targets.append((f"{station_name}.approach", station["approach"]))
            if station["dock"] is not None:
                targets.append((f"{station_name}.dock", station["dock"]))

        for target_name, target in targets:
            assert self.costmap is not None
            target_cost = self._grid_value(
                self.costmap, float(target["x"]), float(target["y"])
            )
            print(f"CHECK path {target_name}: goal cost={target_cost}")
            goal = ComputePathToPose.Goal()
            goal.use_start = True
            goal.planner_id = "GridBased"
            goal.start = self._pose(-4.5, 0.0, 0.0)
            goal.goal = self._pose(target["x"], target["y"], target["yaw"])

            send_future = self.path_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError(f"planner rejected {target_name}")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError(f"planner timed out for {target_name}")
            if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(
                    f"planner failed for {target_name} with status {wrapped_result.status}"
                )
            pose_count = len(wrapped_result.result.path.poses)
            if pose_count == 0:
                raise RuntimeError(f"planner returned an empty path for {target_name}")
            print(f"PASS path {target_name}: {pose_count} poses")

    @staticmethod
    def _pose(x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose


def main() -> int:
    registry_path = (
        Path(get_package_share_directory("amr_factory")) / "config" / "stations.yaml"
    )
    with registry_path.open(encoding="utf-8") as stream:
        registry = yaml.safe_load(stream)

    rclpy.init()
    node = Gate4Acceptance()
    try:
        node.wait_for_inputs()
        node.verify_localization()
        node.verify_station_obstacles()
        node.verify_paths(registry)
        print("PASS Gate 4 factory and localization acceptance")
        return 0
    except Exception as error:  # Keep this executable fail-closed for gate use.
        print(f"FAIL Gate 4 factory and localization acceptance: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
