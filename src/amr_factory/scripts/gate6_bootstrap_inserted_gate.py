#!/usr/bin/env python3
"""Signal the Gate 6 bootstrap after the robot create action exits."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


SERVICE = "/amr/simulation/attachment_bootstrap/robot_inserted"


class BootstrapInsertedGate(Node):
    """Call the bootstrap insertion handshake with a bounded wall deadline."""

    def __init__(self) -> None:
        super().__init__("gate6_bootstrap_inserted_gate")
        self._client = self.create_client(Trigger, SERVICE)


def main() -> int:
    rclpy.init()
    node = BootstrapInsertedGate()
    try:
        if not node._client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error("Gate 6 bootstrap insertion service unavailable")
            return 1
        future = node._client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            node.get_logger().error("Gate 6 bootstrap insertion signal timed out")
            return 1
        response = future.result()
        if response is None or not response.success:
            node.get_logger().error(
                "Gate 6 bootstrap rejected insertion signal: "
                + (response.message if response is not None else "no response"))
            return 1
        node.get_logger().info("Gate 6 bootstrap insertion signal accepted")
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
