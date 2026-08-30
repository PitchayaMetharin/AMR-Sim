#!/usr/bin/env python3
"""Simulation-only keyboard teleoperation through normal command arbitration."""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PrototypeTeleop(Node):
    def __init__(self):
        super().__init__("prototype_teleop")
        self.command_pub = self.create_publisher(Twist, "/amr/mpc/cmd_vel", 10)
        self.linear = 0.0
        self.angular = 0.0
        self.command_deadline = 0.0
        self.create_timer(0.05, self.publish)

    def publish(self):
        command = Twist()
        if time.monotonic() < self.command_deadline:
            command.linear.x = self.linear
            command.angular.z = self.angular
        self.command_pub.publish(command)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        self.command_deadline = 0.0
        self.publish()

    def set_command(self, linear, angular):
        self.linear = linear
        self.angular = angular
        self.command_deadline = time.monotonic() + 0.25


def main():
    rclpy.init()
    node = PrototypeTeleop()
    old_settings = termios.tcgetattr(sys.stdin.fileno())
    try:
        print("Hold W/S/A/D to move; X or Space stops; Q quits")
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()
            if key == "w":
                node.set_command(1.00, 0.0)
            elif key == "s":
                node.set_command(-1.00, 0.0)
            elif key == "a":
                node.set_command(0.0, 0.80)
            elif key == "d":
                node.set_command(0.0, -0.80)
            elif key in ("x", " "):
                node.stop()
            elif key == "q":
                break
    finally:
        if rclpy.ok():
            node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
