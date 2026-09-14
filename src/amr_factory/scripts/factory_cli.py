#!/usr/bin/env python3
"""Fail-closed CLI for one-shot and autonomous factory control."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from amr_interfaces.action import NavigateStation, RunSequence, TransportProduct
from amr_interfaces.msg import FactoryStatus
from amr_interfaces.srv import SetOperationMode
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from factory_registry import RegistryError, load_registry


TRANSPORT_ACTION = "/amr/factory/transport_product"
SEQUENCE_ACTION = "/amr/factory/run_sequence"
HOME_ACTION = "/amr/factory/navigate_station"
MODE_NAME = "/amr/factory/set_operation_mode"
STOP_NAME = "/amr/factory/stop_sequence"
CANCEL_NAME = "/amr/factory/cancel_sequence"
STATUS_NAME = "/amr/factory/status"

# Backward-compatible name used by existing integrations.
ACTION_NAME = TRANSPORT_ACTION


def _spin_until(node, future, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
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
                        if args.mode == "autonomous" else SetOperationMode.Request.MANUAL)
        response = _spin_until(node, client.call_async(request), 3.0)
        if response is None or not response.accepted:
            print(response.message if response else "mode request timed out", file=sys.stderr)
            return 2
        print(response.message)
        return 0
    finally:
        node.destroy_node()


def _transport(args):
    """Send one compatible transport request to the factory supervisor."""
    node = rclpy.create_node("factory_cli_transport")
    client = ActionClient(node, TransportProduct, TRANSPORT_ACTION)
    try:
        if not client.wait_for_server(timeout_sec=3.0):
            print("transport action unavailable", file=sys.stderr)
            return 2
        goal = TransportProduct.Goal()
        goal.pickup_station_id = args.pickup
        goal.destination_station_id = args.destination
        accepted = _spin_until(node, client.send_goal_async(goal), 3.0)
        if accepted is None or not accepted.accepted:
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


def _loop(args):
    node = rclpy.create_node("factory_cli_sequence")
    client = ActionClient(node, RunSequence, SEQUENCE_ACTION)
    try:
        if not client.wait_for_server(timeout_sec=3.0):
            print("sequence action unavailable", file=sys.stderr)
            return 2
        goal = RunSequence.Goal()
        goal.pickup_station_ids = list(args.stations)
        goal.cycle_count = 0 if args.forever else args.cycles
        goal.final_station_id = "" if args.finish == "stay" else args.finish
        accepted = _spin_until(node, client.send_goal_async(goal), 3.0)
        if accepted is None or not accepted.accepted:
            print("sequence goal rejected or timed out", file=sys.stderr)
            return 2
        result = _spin_until(node, accepted.get_result_async(), args.timeout)
        if result is None or result.result is None:
            print("sequence result timed out", file=sys.stderr)
            return 2
        print(result.result.message)
        return 0 if result.result.outcome in (
            RunSequence.Result.SUCCESS, RunSequence.Result.STOPPED) else 1
    finally:
        node.destroy_node()


def _trigger(args):
    node = rclpy.create_node("factory_cli_sequence_control")
    name = STOP_NAME if args.command == "stop" else CANCEL_NAME
    client = node.create_client(Trigger, name)
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"{args.command} service unavailable", file=sys.stderr)
            return 2
        response = _spin_until(node, client.call_async(Trigger.Request()), 3.0)
        if response is None or not response.success:
            print(response.message if response else f"{args.command} request timed out",
                  file=sys.stderr)
            return 2
        print(response.message)
        return 0
    finally:
        node.destroy_node()


def _home(args):
    node = rclpy.create_node("factory_cli_home")
    client = ActionClient(node, NavigateStation, HOME_ACTION)
    try:
        if not client.wait_for_server(timeout_sec=3.0):
            print("navigate-station action unavailable", file=sys.stderr)
            return 2
        goal = NavigateStation.Goal()
        goal.station_id = args.station
        accepted = _spin_until(node, client.send_goal_async(goal), 3.0)
        if accepted is None or not accepted.accepted:
            print("home goal rejected or timed out", file=sys.stderr)
            return 2
        result = _spin_until(node, accepted.get_result_async(), args.timeout)
        if result is None or result.result is None:
            print("home result timed out", file=sys.stderr)
            return 2
        print(result.result.message)
        return 0 if result.result.outcome == NavigateStation.Result.SUCCESS else 1
    finally:
        node.destroy_node()


def _status(_args):
    node = rclpy.create_node("factory_cli_status")
    received = []

    def callback(message):
        received.append(message)

    node.create_subscription(FactoryStatus, STATUS_NAME, callback, 10)
    try:
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if received:
                message = received[-1]
                print(
                    f"mode={message.mode} phase={message.phase} active={message.active} "
                    f"queue={message.queue_depth} sequence_active={message.sequence_active} "
                    f"cycle={message.current_cycle} index={message.sequence_index} "
                    f"completed_jobs={message.completed_jobs} "
                    f"completed_cycles={message.completed_cycles} "
                    f"product={message.product_id or '-'} attached={message.product_attached} "
                    f"fault_latched={message.fault_latched} detail={message.detail}"
                )
                return 0
        print("factory status unavailable", file=sys.stderr)
        return 2
    finally:
        node.destroy_node()


def _list(_args):
    try:
        registry = load_registry()
        pickup = " ".join(sorted(
            station_id for station_id, station in registry.stations.items()
            if station.role == "pickup"))
        products = " ".join(
            f"{product.product_id}{'' if product.autonomous_enabled else '(disabled)'}"
            for product in sorted(registry.products.values(), key=lambda item: item.tag_id))
        print(f"stations: {pickup} dispatch home")
        print(f"products: {products}")
        return 0
    except RegistryError as error:
        print(f"factory registry unavailable: {error}", file=sys.stderr)
        return 2


def main(argv=None):
    parser = argparse.ArgumentParser(prog="factory_cli")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="show registered stations and products")
    mode = sub.add_parser("mode")
    mode.add_argument("mode", choices=("manual", "autonomous"))
    for name in ("send", "enqueue"):
        command = sub.add_parser(name)
        command.add_argument("pickup", choices=("pickup_a", "pickup_b"))
        command.add_argument("destination", choices=("dispatch",))
        command.add_argument("--timeout", type=float, default=240.0)
    loop = sub.add_parser("loop", help="run an ordered finite or continuous sequence")
    loop.add_argument("stations", nargs="+", choices=("pickup_a", "pickup_b"))
    count = loop.add_mutually_exclusive_group(required=True)
    count.add_argument("--cycles", type=int, choices=range(1, 10001))
    count.add_argument("--forever", action="store_true")
    loop.add_argument("--finish", choices=("stay", "home"), default="stay")
    loop.add_argument("--timeout", type=float, default=3600.0)
    sub.add_parser("stop", help="finish the active delivery and stop the sequence")
    sub.add_parser("cancel", help="cancel the active cycle and clear the sequence")
    home = sub.add_parser("go", help="navigate an idle AMR to a registered station")
    home.add_argument("station", choices=("home",))
    home.add_argument("--timeout", type=float, default=180.0)
    sub.add_parser("status")
    args = parser.parse_args(argv)

    if args.command == "list":
        return _list(args)
    rclpy.init()
    try:
        if args.command == "mode":
            return _mode(args)
        if args.command in ("send", "enqueue"):
            return _transport(args)
        if args.command == "loop":
            return _loop(args)
        if args.command in ("stop", "cancel"):
            return _trigger(args)
        if args.command == "go":
            return _home(args)
        return _status(args)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
