#!/usr/bin/env python3
"""Live nominal-simulation acceptance check for Phase 7 localization."""

import math
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


POSITION_LIMIT = 0.03
YAW_LIMIT = 0.04
STATIC_LIMIT = 0.01


def yaw(message):
    orientation = pose(message).orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z +
               orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def angle_delta(end, start):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def relative_pose(end, start):
    dx = pose(end).position.x - pose(start).position.x
    dy = pose(end).position.y - pose(start).position.y
    start_yaw = yaw(start)
    return (
        math.cos(start_yaw) * dx + math.sin(start_yaw) * dy,
        -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy,
        angle_delta(yaw(end), start_yaw),
    )


def pose(message):
    value = message.pose
    return value.pose if hasattr(value, "pose") else value


class AcceptanceNode(Node):
    def __init__(self):
        super().__init__("localization_acceptance")
        self.truth = None
        self.estimate = None
        self.create_subscription(
            PoseStamped, "/amr/simulation/ground_truth/pose",
            self._truth_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/amr/localization/odometry",
            self._estimate_callback, qos_profile_sensor_data)
        self.command = self.create_publisher(
            TwistStamped, "/amr/simulation/base/cmd_vel", 1)

    def _truth_callback(self, message):
        self.truth = message

    def _estimate_callback(self, message):
        self.estimate = message

    def wait_for_state(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.truth is not None and self.estimate is not None:
                return
        raise RuntimeError("Timed out waiting for truth and EKF odometry")

    def hold_command(self, linear, angular, duration):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.twist.linear.x = linear
            message.twist.angular.z = angular
            self.command.publish(message)
            rclpy.spin_once(self, timeout_sec=0.02)

    def snapshot(self):
        return self.truth, self.estimate


def compare_motion(label, truth_end, truth_start, estimate_end,
                   estimate_start):
    truth_motion = relative_pose(truth_end, truth_start)
    estimate_motion = relative_pose(estimate_end, estimate_start)
    position_error = math.hypot(
        estimate_motion[0] - truth_motion[0],
        estimate_motion[1] - truth_motion[1],
    )
    yaw_error = abs(angle_delta(estimate_motion[2], truth_motion[2]))
    print(
        f"{label}: truth=({truth_motion[0]:.4f}, "
        f"{truth_motion[1]:.4f}, {truth_motion[2]:.4f}) "
        f"estimate=({estimate_motion[0]:.4f}, "
        f"{estimate_motion[1]:.4f}, {estimate_motion[2]:.4f}) "
        f"position_error={position_error:.4f} yaw_error={yaw_error:.4f}"
    )
    if position_error > POSITION_LIMIT or yaw_error > YAW_LIMIT:
        raise AssertionError(f"{label} exceeded localization error bounds")


def main():
    rclpy.init()
    node = AcceptanceNode()
    try:
        node.wait_for_state()
        static_start = node.snapshot()
        node.hold_command(0.0, 0.0, 1.0)
        static_end = node.snapshot()
        static_motion = relative_pose(static_end[1], static_start[1])
        static_drift = math.hypot(static_motion[0], static_motion[1])
        print(f"static: position_drift={static_drift:.4f} "
              f"yaw_drift={abs(static_motion[2]):.4f}")
        if (static_drift > STATIC_LIMIT or
                abs(static_motion[2]) > STATIC_LIMIT):
            raise AssertionError("Static estimate exceeded drift bound")

        straight_start = node.snapshot()
        node.hold_command(0.25, 0.0, 2.0)
        node.hold_command(0.0, 0.0, 1.0)
        straight_end = node.snapshot()
        compare_motion(
            "straight", straight_end[0], straight_start[0],
            straight_end[1], straight_start[1])

        turn_start = node.snapshot()
        node.hold_command(0.0, 0.30, 2.0)
        node.hold_command(0.0, 0.0, 1.0)
        turn_end = node.snapshot()
        compare_motion(
            "turn", turn_end[0], turn_start[0],
            turn_end[1], turn_start[1])
        print("PASS: nominal Phase 7 localization acceptance")
    finally:
        node.hold_command(0.0, 0.0, 0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
