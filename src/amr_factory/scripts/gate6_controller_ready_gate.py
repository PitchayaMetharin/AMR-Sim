#!/usr/bin/env python3
"""Release the deferred factory graph after Gate 6 controllers are active.

Native attachment startup is intentionally serialized: Gazebo must finish the
controller-manager service work before Nav2, perception, and mission nodes are
allowed to add their startup load.  This gate has a bounded discovery window
and remains alive on failure so an ``OnProcessExit`` handler cannot release the
deferred graph after a controller fault.
"""

from __future__ import annotations

import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


REQUIRED_CONTROLLERS = {
    "joint_state_broadcaster",
    "arm_controller",
    "gripper_controller",
    "gripper_right_controller",
}
CONTROLLER_SERVICE_TIMEOUT_SEC = 5.0


class ControllerReadyGate(Node):
    """Observe the controller-manager state with a wall-clock deadline."""

    def __init__(self) -> None:
        super().__init__("gate6_controller_ready_gate")
        self.declare_parameter("startup_timeout_sec", 60.0)
        self._timeout = float(self.get_parameter("startup_timeout_sec").value)
        if self._timeout <= 0.0:
            raise ValueError("startup_timeout_sec must be positive")
        self._client = self.create_client(
            ListControllers, "/controller_manager/list_controllers")

    def _controllers_active(self, response: ListControllers.Response) -> bool:
        active = {
            controller.name for controller in response.controller
            if controller.state == "active"
        }
        return REQUIRED_CONTROLLERS.issubset(active)

    def wait_until_ready(self) -> bool:
        deadline = time.monotonic() + self._timeout
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            if not self._client.wait_for_service(timeout_sec=min(0.5, remaining)):
                continue
            future = self._client.call_async(ListControllers.Request())
            call_deadline = min(
                deadline, time.monotonic() + CONTROLLER_SERVICE_TIMEOUT_SEC)
            while rclpy.ok() and not future.done() and time.monotonic() < call_deadline:
                rclpy.spin_once(
                    self, timeout_sec=min(0.1, max(0.01, call_deadline - time.monotonic())))
            if future.done():
                response = future.result()
                if response is not None and self._controllers_active(response):
                    self.get_logger().info(
                        "Gate 6 controllers active; releasing deferred factory graph")
                    return True
            else:
                future.cancel()
        self.get_logger().error(
            "Gate 6 controller readiness timed out; refusing deferred factory graph")
        return False


def main() -> int:
    rclpy.init()
    try:
        node = ControllerReadyGate()
        try:
            if node.wait_until_ready():
                return 0
            # OnProcessExit handlers run for both success and failure.  Hold
            # the process alive on failure so deferred startup stays blocked.
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
            return 1
        finally:
            node.destroy_node()
    except (RuntimeError, ValueError) as error:
        print(f"GATE6 CONTROLLER READY GATE FAULT: {error}", flush=True)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
