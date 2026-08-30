#!/usr/bin/env python3
"""Release native robot insertion only after Gate 6 pauses Gazebo."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


STATUS_TOPIC = "/amr/simulation/attachment_bootstrap/status"


class BootstrapPauseGate(Node):
    """Exit successfully on PAUSED and hold on a latched bootstrap FAULT."""

    def __init__(self) -> None:
        super().__init__("gate6_bootstrap_pause_gate")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._result: int | None = None
        self.create_subscription(String, STATUS_TOPIC, self._status_callback, qos)

    def _status_callback(self, message: String) -> None:
        status = message.data.strip()
        if status.startswith("PAUSED ") or status == "PAUSED":
            self.get_logger().info(
                "Gate 6 bootstrap PAUSED; releasing robot insertion")
            self._result = 0
        elif status.startswith("FAULT ") or status == "FAULT":
            # OnProcessExit handlers run for both success and failure. Keep
            # this process alive on FAULT so robot insertion cannot start.
            self.get_logger().error(
                f"Gate 6 bootstrap FAULT; refusing robot insertion: {status}")


def main() -> int:
    rclpy.init()
    node = BootstrapPauseGate()
    try:
        while rclpy.ok() and node._result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 1 if node._result is None else node._result
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            node.destroy_node()
        except (RuntimeError, ValueError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
