#!/usr/bin/env python3

import argparse
import os
import pty
import queue
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Verify that the Gazebo plant watchdog disables motion after a process loss."
    )
    parser.add_argument("--target-pid", required=True, type=int)
    parser.add_argument("--expected-command-fragment", required=True)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--timeout-seconds", type=float, default=0.35)
    return parser.parse_args()


def validate_target(pid, expected_fragment):
    if pid <= 1:
        raise RuntimeError(f"refusing unsafe target PID {pid}")

    cmdline_path = f"/proc/{pid}/cmdline"
    with open(cmdline_path, "rb") as cmdline_file:
        command = cmdline_file.read().replace(b"\0", b" ").decode(errors="replace")

    if expected_fragment not in command:
        raise RuntimeError(
            f"PID {pid} command does not contain {expected_fragment!r}: {command!r}"
        )
    return command


def command_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=0.1),
        lifespan=Duration(seconds=0.2),
    )


def read_lines(stream, line_queue):
    try:
        for line in iter(stream.readline, ""):
            line_queue.put(line.strip())
    except OSError:
        pass


def start_monitor(topic, line_queue):
    master_fd, slave_fd = pty.openpty()
    monitor = subprocess.Popen(
        ["gz", "topic", "-e", "-t", topic],
        stdout=slave_fd,
        stderr=slave_fd,
        close_fds=True,
    )
    os.close(slave_fd)
    stream = os.fdopen(master_fd, "r", encoding="utf-8", errors="replace")
    reader = threading.Thread(
        target=read_lines, args=(stream, line_queue), daemon=True
    )
    reader.start()
    return monitor, stream


def publish_once(node, publisher, message):
    message.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.02)


def wait_for_native_command(node, publisher, line_queue, message, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        publish_once(node, publisher, message)
        try:
            if line_queue.get_nowait():
                return
        except queue.Empty:
            pass
    raise RuntimeError(f"did not observe a native Gazebo command within {timeout:.2f} s")


def wait_for_disable_log(node, publisher, message, server_log, timeout):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        publish_once(node, publisher, message)
        line = server_log.readline()
        while line:
            if "Command watchdog enable=false" in line:
                return time.monotonic()
            line = server_log.readline()

    raise RuntimeError(
        f"plant watchdog did not log enable=false within {timeout:.2f} s"
    )


def latest_server_log():
    candidates = list((Path.home() / ".gz" / "sim" / "log").glob(
        "*/server_console.log"
    ))
    if not candidates:
        raise RuntimeError("no Gazebo server log found")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def main():
    args = parse_args()
    target_command = validate_target(args.target_pid, args.expected_command_fragment)

    rclpy.init()
    node = rclpy.create_node("plant_watchdog_process_loss")
    publisher = node.create_publisher(TwistStamped, args.input_topic, command_qos())
    message = TwistStamped()
    message.header.frame_id = "base_footprint"
    message.twist.linear.x = 0.1
    command_lines = queue.Queue()
    command_monitor, command_stream = start_monitor(
        "/model/amr/cmd_vel", command_lines
    )
    server_log_path = latest_server_log()
    server_log = server_log_path.open("r", encoding="utf-8", errors="replace")
    server_log.seek(0, os.SEEK_END)

    try:
        wait_for_native_command(node, publisher, command_lines, message, 5.0)
        # Gazebo Transport discovery is asynchronous. Keep native commands flowing
        # long enough for the independently-started enable subscriber to connect
        # before creating the process-loss transition.
        ready_until = time.monotonic() + 1.0
        while time.monotonic() < ready_until:
            publish_once(node, publisher, message)
        signal_time = time.monotonic()
        os.kill(args.target_pid, signal.SIGTERM)
        disabled_time = wait_for_disable_log(
            node,
            publisher,
            message,
            server_log,
            args.timeout_seconds,
        )
        elapsed = disabled_time - signal_time
        print(
            "PASS "
            f"target_pid={args.target_pid} "
            f"target={target_command!r} "
            f"server_log={str(server_log_path)!r} "
            f"disable_latency={elapsed:.3f}s "
            f"limit={args.timeout_seconds:.3f}s"
        )
        return 0
    finally:
        command_monitor.terminate()
        try:
            command_monitor.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            command_monitor.kill()
            command_monitor.wait(timeout=1.0)
        command_stream.close()
        server_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
