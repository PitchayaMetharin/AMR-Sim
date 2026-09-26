#!/usr/bin/env python3
"""Persist the terminal Explorer proof for one bounded AWS run."""

import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node


TERMINAL_STATES = frozenset(("COMPLETE", "INCOMPLETE", "FAULT"))
EXPLORER_STATUS_NAME = "amr_exploration/frontier_explorer"
SAFE_REACHABLE_COMPLETION_POLICY = "SAFE_REACHABLE_AREA_V1"
TERMINAL_BOOLEAN_FIELDS = (
    "active",
    "pending",
    "cancel_owned_motion",
    "motion_stopped",
    "fault_latched",
)
TERMINAL_COUNT_FIELDS = (
    "run_generation",
    "motion_generation",
    "map_version",
    "goal_failures",
    "no_frontier_updates_seen",
    "reached_goal_count",
    "raw_frontier_count",
    "blocked_frontier_count",
    "blocked_safety_count",
    "blocked_route_count",
    "unresolved_frontier_count",
    "blocked_count",
    "retry_exhausted_count",
)
TERMINAL_MISSION_FIELDS = (
    "mission_goal_uuid",
    "mission_stage",
    "mission_outcome",
    "mission_reason",
    "mission_fault_class",
)
MISSION_OUTCOME_BY_STATE = {
    "COMPLETE": "SUCCEEDED",
    "INCOMPLETE": "ABORTED",
    "FAULT": "FAULT",
}


def _values(status):
    values = {}
    for item in status.values:
        if item.key in values:
            return None
        values[item.key] = item.value
    return values


def _bool_value(values, key):
    value = values.get(key)
    if value is None:
        return None
    normalized = str(value).strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def _count_value(values, key):
    value = values.get(key)
    if value is None:
        return None
    try:
        parsed = int(str(value).strip())
    except (TypeError, ValueError):
        return None
    if parsed < 0 or str(value).strip() != str(parsed):
        return None
    return parsed


def terminal_evidence(status, *, received_wall_time=None):
    """Convert one Explorer DiagnosticStatus into a fail-closed verdict."""
    if status is None or status.name != EXPLORER_STATUS_NAME:
        return None
    values = _values(status)
    if values is None:
        return None
    state = str(values.get("state", ""))
    if state not in TERMINAL_STATES:
        return None
    booleans = {
        key: _bool_value(values, key) for key in TERMINAL_BOOLEAN_FIELDS}
    if any(value is None for value in booleans.values()):
        return None
    if "cancel_target" not in values:
        return None
    counts = {
        key: _count_value(values, key) for key in TERMINAL_COUNT_FIELDS}
    if any(value is None for value in counts.values()) or counts["run_generation"] <= 0:
        return None
    if values.get("completion_policy") != SAFE_REACHABLE_COMPLETION_POLICY:
        return None
    if (counts["blocked_frontier_count"]
            != counts["blocked_safety_count"] + counts["blocked_route_count"]):
        return None
    if (counts["blocked_frontier_count"]
            + counts["unresolved_frontier_count"]
            > counts["raw_frontier_count"]):
        return None
    if (state == "COMPLETE"
            and (counts["blocked_frontier_count"]
                 + counts["unresolved_frontier_count"]
                 != counts["raw_frontier_count"]
                 or counts["unresolved_frontier_count"] != 0)):
        return None
    if "mission_goal_uuid" not in values:
        return None
    if any(key not in values or not str(values[key]).strip()
           for key in TERMINAL_MISSION_FIELDS[1:]):
        return None
    if values["mission_stage"] != "TERMINAL":
        return None
    if values["mission_outcome"] != MISSION_OUTCOME_BY_STATE[state]:
        return None
    reason = str(values.get("reason", "")).strip()
    mission_reason = str(values["mission_reason"]).strip()
    if not reason or not mission_reason:
        return None
    if state != "FAULT" and booleans["fault_latched"]:
        return None
    incomplete_acknowledgement = str(
        values.get("incomplete_acknowledgement", "")).strip()
    incomplete_explanation = str(
        values.get("incomplete_explanation", reason)).strip()
    if state == "INCOMPLETE" and not (
            incomplete_acknowledgement == "ACCEPT_INCOMPLETE"
            or reason.startswith("exploration incomplete:")):
        return None
    if state == "INCOMPLETE" and not incomplete_explanation:
        return None
    verdict = {
        "state": state,
        **booleans,
        "cancel_target": str(values.get("cancel_target", "")).strip(),
        "reason": reason or str(status.message),
        "completion_policy": values["completion_policy"],
        **counts,
        **{key: str(values[key]).strip() for key in TERMINAL_MISSION_FIELDS},
        "incomplete_acknowledgement": incomplete_acknowledgement,
        "incomplete_explanation": incomplete_explanation,
        "received_wall_time": (
            time.time() if received_wall_time is None else received_wall_time),
    }
    return verdict


def write_json_atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".%s.%s.tmp" % (path.name, os.getpid()))
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class ExplorationMonitor(Node):
    def __init__(self, verdict_file, topic, timeout_sec):
        super().__init__("aws_exploration_monitor")
        self.verdict_file = Path(verdict_file)
        self.timeout_sec = float(timeout_sec)
        self.started_at = time.monotonic()
        self.finished = False
        self.failed = False
        self.last_verdict = None
        self.subscription = self.create_subscription(
            DiagnosticArray, topic, self._status_callback, 10)

    def _status_callback(self, message):
        for status in message.status:
            verdict = terminal_evidence(status)
            if verdict is None:
                continue
            try:
                write_json_atomic(self.verdict_file, verdict)
            except (OSError, TypeError, ValueError) as exc:
                self.get_logger().error("could not persist Explorer verdict: %s" % exc)
                self.failed = True
                self.finished = True
                return
            self.last_verdict = verdict
            self.finished = True
            return

    def check_timeout(self):
        if self.timeout_sec <= 0.0 or self.finished:
            return
        if time.monotonic() - self.started_at < self.timeout_sec:
            return
        verdict = {
            "state": "FAULT",
            "active": True,
            "pending": True,
            "cancel_owned_motion": True,
            "motion_stopped": False,
            "fault_latched": True,
            "cancel_target": "monitor_timeout",
            "reason": "Explorer terminal proof was not received before monitor timeout",
            "run_generation": "",
            "motion_generation": "",
            "map_version": 0,
            "goal_failures": 0,
            "no_frontier_updates_seen": 0,
            "reached_goal_count": 0,
            "unresolved_frontier_count": 0,
            "blocked_count": 0,
            "retry_exhausted_count": 0,
            "mission_goal_uuid": "",
            "mission_stage": "UNKNOWN",
            "mission_outcome": "FAULT",
            "mission_reason": "Explorer terminal proof timeout",
            "mission_fault_class": "NAVIGATION_FAULT",
            "incomplete_acknowledgement": "",
            "incomplete_explanation": "",
            "received_wall_time": time.time(),
        }
        try:
            write_json_atomic(self.verdict_file, verdict)
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error("could not persist monitor timeout: %s" % exc)
            self.failed = True
        self.last_verdict = verdict
        self.finished = True


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verdict-file", required=True, type=Path)
    parser.add_argument("--topic", default="/amr/exploration/status")
    parser.add_argument("--timeout-sec", type=float, default=0.0)
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    if args.timeout_sec < 0.0:
        raise SystemExit("--timeout-sec must be non-negative")
    rclpy.init(args=None)
    node = ExplorationMonitor(args.verdict_file, args.topic, args.timeout_sec)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_timeout()
    except KeyboardInterrupt:
        node.failed = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
