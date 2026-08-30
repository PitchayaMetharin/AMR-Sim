#!/usr/bin/env python3
"""Small, fail-closed command-line client for the factory supervisor."""

import argparse
import sys

import rclpy
from amr_interfaces.action import TransportProduct
from amr_interfaces.msg import FactoryStatus
from amr_interfaces.srv import SetOperationMode
from rclpy.action import ActionClient


ACTION_NAME = "/amr/factory/transport_product"
MODE_NAME = "/amr/factory/set_operation_mode"
STATUS_NAME = "/amr/factory/status"


def _spin_until(node, future, timeout):
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if future.done():
            return future.result()
    return None


def _mode(args):
    node = rclpy.create_node("factory_cli_mode")
    client = node.create_client(SetOperationMode, MODE_NAME)
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            print("operation-mode service unavailable", file=sys.stderr)
            return 2
        request = SetOperationMode.Request()
        request.mode = (SetOperationMode.Request.AUTONOMOUS
                        if args.mode == "autonomous"
                        else SetOperationMode.Request.MANUAL)
        response = _spin_until(node, client.call_async(request), 3.0)
        if response is None or not response.accepted:
            print(response.message if response else "mode request timed out", file=sys.stderr)
            return 2
        print(response.message)
        return 0
    finally:
        node.destroy_node()


def _transport(args):
    node = rclpy.create_node("factory_cli_transport")
    client = ActionClient(node, TransportProduct, ACTION_NAME)
    try:
        if not client.wait_for_server(timeout_sec=3.0):
            print("transport action unavailable", file=sys.stderr)
            return 2
        goal = TransportProduct.Goal()
        goal.pickup_station_id = args.pickup
        goal.destination_station_id = args.destination
        accepted = _spin_until(node, client.send_goal_async(goal), 3.0)
        if accepted is None or not accepted:
            print("transport goal rejected or timed out", file=sys.stderr)
            return 2
        result = _spin_until(node, accepted.get_result_async(), args.timeout)
        if result is None or result.result is None:
            print("transport result timed out", file=sys.stderr)
            return 2
        print(result.result.message)
        return 0 if result.result.delivered else 1
    finally:
        node.destroy_node()


def _status(_args):
    node = rclpy.create_node("factory_cli_status")
    received = []

    def callback(message):
        received.append(message)

    node.create_subscription(FactoryStatus, STATUS_NAME, callback, 10)
    try:
        deadline = node.get_clock().now().nanoseconds + 3_000_000_000
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if received:
                message = received[-1]
                print(
                    f"mode={message.mode} phase={message.phase} active={message.active} "
                    f"queue={message.queue_depth} product={message.product_id or '-'} "
                    f"attached={message.product_attached} detail={message.detail}"
                )
                return 0
        print("factory status unavailable", file=sys.stderr)
        return 2
    finally:
        node.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="factory_cli")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="show registered stations and products")
    mode = sub.add_parser("mode")
    mode.add_argument("mode", choices=("manual", "autonomous"))
    for name in ("send", "enqueue"):
        command = sub.add_parser(name)
        command.add_argument("pickup", choices=("pickup_a", "pickup_b", "pickup_c"))
        command.add_argument("destination", choices=("dispatch",))
        command.add_argument("--timeout", type=float, default=120.0)
    sub.add_parser("status")
    args = parser.parse_args(argv)

    if args.command == "list":
        print("stations: pickup_a pickup_b pickup_c dispatch")
        print("products: 101 102 103")
        return 0
    rclpy.init()
    try:
        if args.command == "mode":
            return _mode(args)
        if args.command in ("send", "enqueue"):
            return _transport(args)
        return _status(args)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
