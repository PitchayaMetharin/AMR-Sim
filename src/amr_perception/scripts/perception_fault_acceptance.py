#!/usr/bin/env python3
"""Live fail-closed acceptance for an isolated Phase 8 pipeline."""
import time

from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


INPUT_TOPIC = "/amr/phase8_acceptance/input"
OUTPUT_TOPIC = "/amr/phase8_acceptance/output"


class AcceptanceNode(Node):
    def __init__(self):
        super().__init__(
            "perception_fault_acceptance",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.outputs = []
        self.publisher = self.create_publisher(
            PointCloud2, INPUT_TOPIC, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, OUTPUT_TOPIC, self.outputs.append,
            qos_profile_sensor_data)

    def wait_for_clock(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds:
                return
        raise RuntimeError("Timed out waiting for simulated clock")

    def wait_for_peers(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.publisher.get_subscription_count() == 1 and
                    self.count_publishers(OUTPUT_TOPIC) == 1):
                return
        raise RuntimeError("Timed out waiting for isolated pipeline peers")

    def cloud(self, stamp, malformed=False):
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = "front_lidar_link"
        message.width = 1
        message.height = 1
        message.point_step = 12
        message.row_step = 12
        message.fields = [
            PointField(
                name=name,
                offset=index * 4,
                datatype=PointField.FLOAT32,
                count=1,
            )
            for index, name in enumerate(("x", "y", "z"))
        ]
        if not malformed:
            message.data = bytes(12)
        return message

    def publish_and_wait(self, message, timeout=0.35):
        before = len(self.outputs)
        self.publisher.publish(message)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return len(self.outputs) - before


def minus_seconds(stamp, seconds):
    nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec - seconds * 1_000_000_000
    return Time(sec=nanoseconds // 1_000_000_000,
                nanosec=nanoseconds % 1_000_000_000)


def main():
    rclpy.init()
    node = AcceptanceNode()
    try:
        node.wait_for_clock()
        node.wait_for_peers()
        now = node.get_clock().now().to_msg()
        if node.publish_and_wait(node.cloud(now, malformed=True)):
            raise AssertionError("Malformed PointCloud2 was republished")
        stale = minus_seconds(now, 1)
        if node.publish_and_wait(node.cloud(stale)):
            raise AssertionError("Stale PointCloud2 was republished")
        valid = node.get_clock().now().to_msg()
        fresh_outputs = node.publish_and_wait(node.cloud(valid))
        if fresh_outputs != 1:
            raise AssertionError(
                "Fresh PointCloud2 was not republished exactly once")
        backward = minus_seconds(valid, 1)
        if node.publish_and_wait(node.cloud(backward)):
            raise AssertionError("Backward-time PointCloud2 was republished")
        print("PASS: malformed, stale, and backward-time clouds were inhibited")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
