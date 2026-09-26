#!/usr/bin/env python3
"""Persist Explorer diagnostics and opt-in simulation evidence as JSONL."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts, Entity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from portable_exploration_readiness import (  # noqa: E402
    RECEIPT_MAX_AGE_S,
    fresh_receipt,
)


CONTACTS_TOPIC = "/amr/simulation/diagnostics/contacts"
COVERAGE_TOPIC = "/amr/simulation/diagnostics/contact_coverage"
GROUND_TRUTH_TOPIC = "/amr/simulation/ground_truth/pose"
FRONT_SCAN_TOPIC = "/amr/sensors/front_lidar/scan"
REAR_SCAN_TOPIC = "/amr/sensors/rear_lidar/scan"
CLOCK_TOPIC = "/clock"
EXPLORER_STATUS_NAME = "amr_exploration/frontier_explorer"

REQUIRED_SIMULATION_STREAMS = (
    CONTACTS_TOPIC,
    COVERAGE_TOPIC,
    GROUND_TRUTH_TOPIC,
    FRONT_SCAN_TOPIC,
    REAR_SCAN_TOPIC,
)

AMR_MODEL = "amr"
AMR_SUPPORT_LINKS = frozenset({
    "left_wheel",
    "right_wheel",
    "front_left_caster_wheel",
    "front_right_caster_wheel",
    "rear_left_caster_wheel",
    "rear_right_caster_wheel",
})
GROUND_MODEL = "aws_robomaker_warehouse_GroundB_01_001"
GROUND_PATHS = frozenset({
    (GROUND_MODEL, "ground_link", "collision"),
    (GROUND_MODEL, "GroundB", "ground_link", "collision"),
    (
        GROUND_MODEL,
        "aws_robomaker_warehouse_GroundB_01",
        "ground_link",
        "collision",
    ),
})

NORMAL_SUPPORT = "NORMAL_SUPPORT"
OBSTACLE_CONTACT = "OBSTACLE_CONTACT"
SELF_CONTACT = "SELF_CONTACT"
UNEXPECTED_GROUND_CONTACT = "UNEXPECTED_GROUND_CONTACT"
UNKNOWN_CONTACT = "UNKNOWN_CONTACT"


EVIDENCE_TOPICS = (
    "/clock",
    "/tf",
    "/tf_static",
    "/map",
    "/amr/global_costmap/costmap_raw",
    "/amr/global_costmap/costmap",
    "/amr/global_costmap/published_footprint",
    "/amr/local_costmap/costmap_raw",
    "/amr/local_costmap/costmap",
    "/amr/local_costmap/published_footprint",
    "/amr/exploration/status",
    "/amr/plan",
    "/amr/plan_smoothed",
    "/amr/received_global_plan",
    "/amr/sensors/front_lidar/scan",
    "/amr/sensors/rear_lidar/scan",
    "/amr/perception/front_lidar/points",
    "/amr/perception/rear_lidar/points",
    "/amr/simulation/sensors/front_lidar/scan",
    "/amr/simulation/sensors/rear_lidar/scan",
    "/amr/simulation/sensors/front_lidar/points",
    "/amr/simulation/sensors/rear_lidar/points",
    "/amr/mpc/cmd_vel",
    "/amr/control/cmd_vel",
    "/amr/simulation/base/cmd_vel",
    "/amr/simulation/diagnostics/delivered_cmd_vel",
    "/amr/base/joint_states",
    "/amr/simulation/base/joint_states",
    "/amr/simulation/base/odometry",
    "/amr/localization/wheel_odometry",
    "/amr/localization/odometry",
    "/amr/simulation/ground_truth/pose",
    "/amr/simulation/diagnostics/contacts",
    "/amr/simulation/diagnostics/contact_coverage",
    "/amr/base/status",
    "/amr/manipulation/status",
    "/amr/mission/status",
    "/amr/mission/navigate_to_pose/_action/goal",
    "/amr/mission/navigate_to_pose/_action/feedback",
    "/amr/mission/navigate_to_pose/_action/result",
    "/amr/mission/navigate_to_pose/_action/status",
    "/amr/compute_path_to_pose/_action/status",
    "/amr/smooth_path/_action/status",
    "/amr/follow_path/_action/status",
    "/amr/lookahead_point",
    "/amr/lookahead_collision_arc",
)


def evidence_topics():
    """Return the ordered, duplicate-free diagnostics bag contract."""
    topics = tuple(topic.strip() for topic in EVIDENCE_TOPICS)
    if any(not topic.startswith("/") for topic in topics):
        raise ValueError("evidence topics must be absolute ROS names")
    if len(topics) != len(set(topics)):
        raise ValueError("evidence topics must be unique")
    return topics


def _time_ns(stamp):
    try:
        sec = stamp.sec
        nanosec = stamp.nanosec
    except AttributeError:
        return None
    if type(sec) is not int or type(nanosec) is not int:
        return None
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


def _stamp_ns(message):
    try:
        return _time_ns(message.header.stamp)
    except AttributeError:
        return None


def _status_level(value):
    """Normalize Humble's byte-valued uint8 diagnostic level."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        if len(value) != 1:
            raise ValueError("diagnostic status level must contain one byte")
        return int(value[0])
    return int(value)


def _record(message):
    """Retain the original Explorer-only JSONL format."""
    statuses = []
    for status in message.status:
        statuses.append({
            "name": status.name,
            "level": _status_level(status.level),
            "message": status.message,
            "values": {item.key: item.value for item in status.values},
        })
    return {
        "received_wall_time": time.time(),
        "ros_stamp_ns": _stamp_ns(message),
        "statuses": statuses,
    }


def _json_value(value):
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "+Infinity" if value > 0.0 else "-Infinity"
        return value
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, (bytes, bytearray, memoryview)):
        return list(bytes(value))
    fields = getattr(type(value), "get_fields_and_field_types", None)
    if callable(fields):
        return {
            name: _json_value(getattr(value, name))
            for name in fields()
        }
    if isinstance(value, (list, tuple)) or hasattr(value, "__iter__"):
        return [_json_value(item) for item in value]
    raise TypeError("unsupported ROS evidence value: %s" % type(value).__name__)


def raw_message_record(
    topic,
    message,
    *,
    received_wall_time=None,
    received_monotonic=None,
):
    """Return a complete JSON-safe copy of one received ROS message."""
    return {
        "record_type": "raw_message",
        "topic": str(topic),
        "message_type": "%s.%s" % (
            type(message).__module__, type(message).__name__),
        "received_wall_time": (
            time.time() if received_wall_time is None else float(received_wall_time)),
        "received_monotonic": (
            time.monotonic()
            if received_monotonic is None else float(received_monotonic)),
        "message": _json_value(message),
    }


def _finite_vector(vector):
    try:
        return all(math.isfinite(float(value)) for value in (
            vector.x, vector.y, vector.z))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


def _finite_wrench(wrench):
    try:
        return all(_finite_vector(value) for value in (
            wrench.body_1_wrench.force,
            wrench.body_1_wrench.torque,
            wrench.body_2_wrench.force,
            wrench.body_2_wrench.torque,
        ))
    except AttributeError:
        return False


def _entity_type(value):
    if isinstance(value, (bytes, bytearray, memoryview)):
        if len(value) != 1:
            return None
        return int(value[0])
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _collision_identity(entity):
    try:
        identifier = entity.id
        name = entity.name
        entity_type = _entity_type(entity.type)
    except AttributeError:
        return None
    if type(identifier) is not int or identifier <= 0:
        return None
    if entity_type != int(Entity.COLLISION):
        return None
    if not isinstance(name, str) or not name or name != name.strip():
        return None
    parts = tuple(name.split("::"))
    if len(parts) < 3 or any(not part or part != part.strip() for part in parts):
        return None
    return {"id": identifier, "name": name, "parts": parts}


def _contact_fields_valid(contact):
    try:
        positions = list(contact.positions)
        normals = list(contact.normals)
        depths = list(contact.depths)
        wrenches = list(contact.wrenches)
    except (AttributeError, TypeError):
        return False
    count = len(positions)
    if count <= 0 or not (
        len(normals) == count == len(depths) == len(wrenches)
    ):
        return False
    try:
        depths_finite = all(math.isfinite(float(value)) for value in depths)
    except (TypeError, ValueError, OverflowError):
        return False
    return (
        depths_finite
        and all(_finite_vector(value) for value in positions)
        and all(_finite_vector(value) for value in normals)
        and all(_finite_wrench(value) for value in wrenches)
    )


def _amr_link(identity):
    parts = identity["parts"]
    if len(parts) != 3 or parts[0] != AMR_MODEL:
        return None
    return parts[1]


def _is_ground(identity):
    return identity["parts"] in GROUND_PATHS


def classify_contact(contact):
    """Classify one complete contact without accepting ambiguous identities."""
    if not _contact_fields_valid(contact):
        return UNKNOWN_CONTACT
    first = _collision_identity(contact.collision1)
    second = _collision_identity(contact.collision2)
    if first is None or second is None:
        return UNKNOWN_CONTACT
    if first["id"] == second["id"] or first["name"] == second["name"]:
        return UNKNOWN_CONTACT

    first_amr = _amr_link(first)
    second_amr = _amr_link(second)
    first_has_amr_scope = first["parts"][0] == AMR_MODEL
    second_has_amr_scope = second["parts"][0] == AMR_MODEL
    if first_has_amr_scope and first_amr is None:
        return UNKNOWN_CONTACT
    if second_has_amr_scope and second_amr is None:
        return UNKNOWN_CONTACT

    if first_amr is not None and second_amr is not None:
        return SELF_CONTACT
    if first_amr is not None and _is_ground(second):
        return (
            NORMAL_SUPPORT
            if first_amr in AMR_SUPPORT_LINKS
            else UNEXPECTED_GROUND_CONTACT)
    if second_amr is not None and _is_ground(first):
        return (
            NORMAL_SUPPORT
            if second_amr in AMR_SUPPORT_LINKS
            else UNEXPECTED_GROUND_CONTACT)
    if (first_amr is None) == (second_amr is None):
        return UNKNOWN_CONTACT
    return OBSTACLE_CONTACT


def parse_coverage(message, previous_command_receipt_count=None):
    """Validate and return the plugin's complete coverage heartbeat."""
    try:
        payload = json.loads(message.data)
    except (AttributeError, TypeError, json.JSONDecodeError) as error:
        raise ValueError("coverage payload is not valid JSON") from error
    if not isinstance(payload, dict):
        raise ValueError("coverage payload must be an object")
    names = (
        "simulation_time_ns",
        "monitored_collisions",
        "covered_collisions",
        "contact_count",
        "command_receipt_count",
    )
    if any(name not in payload for name in names):
        raise ValueError("coverage payload is missing required counts")
    if any(type(payload[name]) is not int or payload[name] < 0 for name in names):
        raise ValueError("coverage stamps and counts must be nonnegative integers")
    monitored = payload["monitored_collisions"]
    if monitored <= 0 or payload["covered_collisions"] != monitored:
        raise ValueError("coverage must cover every monitored collision")
    previous = previous_command_receipt_count
    if previous is not None:
        if type(previous) is not int or previous < 0:
            raise ValueError("previous command receipt count is invalid")
        if payload["command_receipt_count"] < previous:
            raise ValueError("coverage command receipt count decreased")
    return payload


def valid_ground_truth(message):
    try:
        position = message.pose.position
        orientation = message.pose.orientation
        position_values = (position.x, position.y, position.z)
        quaternion = (
            orientation.x, orientation.y, orientation.z, orientation.w)
        values = tuple(float(value) for value in position_values + quaternion)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    if not all(math.isfinite(value) for value in values):
        return False
    norm = math.sqrt(sum(value * value for value in values[3:]))
    return math.isclose(norm, 1.0, rel_tol=1e-6, abs_tol=1e-6)


def valid_laser_scan(message):
    try:
        angle_min = float(message.angle_min)
        angle_max = float(message.angle_max)
        angle_increment = float(message.angle_increment)
        time_increment = float(message.time_increment)
        scan_time = float(message.scan_time)
        range_min = float(message.range_min)
        range_max = float(message.range_max)
        ranges = [float(value) for value in message.ranges]
        intensities = [float(value) for value in message.intensities]
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    geometry = (
        angle_min, angle_max, angle_increment, time_increment,
        scan_time, range_min, range_max)
    if not all(math.isfinite(value) for value in geometry):
        return False
    if (
        not ranges
        or angle_increment <= 0.0
        or angle_max < angle_min
        or time_increment < 0.0
        or scan_time < 0.0
        or range_min < 0.0
        or range_max <= range_min
    ):
        return False
    expected_angle_max = angle_min + angle_increment * (len(ranges) - 1)
    if not math.isclose(
        expected_angle_max, angle_max, rel_tol=1e-5, abs_tol=1e-5
    ):
        return False
    for value in ranges:
        if math.isnan(value) or (math.isinf(value) and value < 0.0):
            return False
        if math.isfinite(value) and not range_min <= value <= range_max:
            return False
    if intensities and len(intensities) != len(ranges):
        return False
    return all(math.isfinite(value) for value in intensities)


class _StreamState:
    def __init__(self):
        self.stamp_ns = None
        self.received_at = None
        self.highest_stamp_ns = None
        self.pending_stamp_ns = None
        self.pending_received_at = None
        self.pending_window_started_at = None


class SimulationEvidenceState:
    """Track advancing simulation stamps using steady-clock receipt ages."""

    def __init__(self):
        self.clock = _StreamState()
        self.streams = {
            topic: _StreamState() for topic in REQUIRED_SIMULATION_STREAMS}

    @staticmethod
    def _valid_stamp(stamp_ns):
        return type(stamp_ns) is int and stamp_ns >= 0

    def observe_clock(self, stamp_ns, *, now):
        if not self._valid_stamp(stamp_ns) or not math.isfinite(float(now)):
            return "invalid"
        highest = self.clock.highest_stamp_ns
        if highest is not None:
            if stamp_ns < highest:
                return "reversed"
            if stamp_ns == highest:
                return "duplicate"
        self.clock.highest_stamp_ns = stamp_ns
        self.clock.stamp_ns = stamp_ns
        self.clock.received_at = float(now)
        for stream in self.streams.values():
            pending_stamp = stream.pending_stamp_ns
            if pending_stamp is None or pending_stamp > stamp_ns:
                continue
            pending_at = stream.pending_received_at
            pending_window_started_at = stream.pending_window_started_at
            pending_is_fresh = (
                fresh_receipt(
                    pending_window_started_at, float(now), RECEIPT_MAX_AGE_S)
                and fresh_receipt(
                    pending_at, float(now), RECEIPT_MAX_AGE_S)
            )
            if pending_is_fresh:
                stream.stamp_ns = pending_stamp
                stream.received_at = pending_at
            stream.pending_stamp_ns = None
            stream.pending_received_at = None
            if pending_is_fresh:
                stream.pending_window_started_at = None
        return "advanced"

    def observe_stream(self, topic, stamp_ns, *, now):
        if topic not in self.streams:
            raise ValueError("unknown required simulation stream: %s" % topic)
        if not self._valid_stamp(stamp_ns) or not math.isfinite(float(now)):
            return "invalid"
        stream = self.streams[topic]
        highest = stream.highest_stamp_ns
        if highest is not None:
            if stamp_ns < highest:
                return "reversed"
            if stamp_ns == highest:
                return "duplicate"
        stream.highest_stamp_ns = stamp_ns
        if self.clock.stamp_ns is None or stamp_ns > self.clock.stamp_ns:
            if stream.pending_window_started_at is None:
                stream.pending_window_started_at = float(now)
            stream.pending_stamp_ns = stamp_ns
            stream.pending_received_at = float(now)
            return "future"
        stream.stamp_ns = stamp_ns
        stream.received_at = float(now)
        stream.pending_stamp_ns = None
        stream.pending_received_at = None
        stream.pending_window_started_at = None
        return "advanced"

    @staticmethod
    def _issue(topic, stream, now, *, allow_pending):
        if fresh_receipt(stream.received_at, now, RECEIPT_MAX_AGE_S):
            return None
        if (
            allow_pending
            and fresh_receipt(
                stream.pending_window_started_at, now, RECEIPT_MAX_AGE_S)
            and fresh_receipt(
                stream.pending_received_at, now, RECEIPT_MAX_AGE_S)
        ):
            return None
        return "%s %s" % (
            topic, "missing" if stream.received_at is None else "stale")

    def issues(self, *, now, allow_pending=False):
        issues = []
        clock_issue = self._issue(
            CLOCK_TOPIC, self.clock, now, allow_pending=False)
        if clock_issue is not None:
            issues.append(clock_issue)
        for topic, stream in self.streams.items():
            issue = self._issue(
                topic, stream, now, allow_pending=allow_pending)
            if issue is not None:
                issues.append(issue)
        return tuple(issues)


class EvidenceStore:
    """Append JSONL evidence and durably preserve the first fault."""

    def __init__(self, output_file):
        self.output_file = Path(output_file)
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.first_fault_path = self.output_file.with_name(
            "%s_first_fault.json" % self.output_file.stem)
        self.stream = self.output_file.open("a", encoding="utf-8")
        self._fault_written = False
        self._closed = False

    def append(self, record):
        if self._closed:
            raise OSError("evidence store is closed")
        json.dump(record, self.stream, sort_keys=True, allow_nan=False)
        self.stream.write("\n")
        self.stream.flush()

    def write_first_fault(self, record):
        if self._fault_written:
            return
        self.stream.flush()
        os.fsync(self.stream.fileno())
        with self.first_fault_path.open("x", encoding="utf-8") as stream:
            json.dump(record, stream, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        directory_fd = os.open(self.output_file.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        self._fault_written = True

    def close(self):
        if self._closed:
            return
        self.stream.flush()
        self.stream.close()
        self._closed = True


def _frontier_motion_phase(message):
    matching = [
        status for status in message.status
        if status.name == EXPLORER_STATUS_NAME
    ]
    if not matching:
        return False
    if len(matching) != 1:
        raise ValueError("Explorer status identity is ambiguous")
    values = {}
    for item in matching[0].values:
        if item.key in values:
            raise ValueError("Explorer status contains duplicate keys")
        values[item.key] = item.value
    if any(key not in values for key in ("active", "pending", "state")):
        raise ValueError("Explorer status is missing motion fields")
    booleans = {}
    for key in ("active", "pending"):
        value = values[key].strip().lower()
        if value not in ("true", "false"):
            raise ValueError("Explorer status has malformed %s" % key)
        booleans[key] = value == "true"
    state = values["state"].strip().upper()
    if not state:
        raise ValueError("Explorer status state is blank")
    return booleans["active"] or booleans["pending"] or state == "NAVIGATING"


class ExplorationDiagnostics(Node):
    def __init__(self, output_file, topic, *, simulation_evidence=False):
        super().__init__("aws_exploration_diagnostics")
        self.output_file = Path(output_file)
        self.store = EvidenceStore(self.output_file)
        self.stream = self.store.stream
        self.failed = False
        self.simulation_evidence = bool(simulation_evidence)
        self.evidence_state = SimulationEvidenceState()
        self.command_receipt_count = None
        self.support_contact_seen = False
        self._support_contact_stamps = set()
        self.motion_phase_admitted = False
        self._last_missing = None
        self.subscription = self.create_subscription(
            DiagnosticArray, topic, self._status_callback, 10)
        self.simulation_subscriptions = []
        if self.simulation_evidence:
            self.simulation_subscriptions.extend((
                self.create_subscription(
                    Clock, CLOCK_TOPIC, self._clock_callback,
                    qos_profile_sensor_data),
                self.create_subscription(
                    Contacts, CONTACTS_TOPIC, self._contacts_callback,
                    qos_profile_sensor_data),
                self.create_subscription(
                    String, COVERAGE_TOPIC, self._coverage_callback, 10),
                self.create_subscription(
                    PoseStamped, GROUND_TRUTH_TOPIC,
                    self._ground_truth_callback, qos_profile_sensor_data),
                self.create_subscription(
                    LaserScan, FRONT_SCAN_TOPIC,
                    lambda message: self._scan_callback(FRONT_SCAN_TOPIC, message),
                    qos_profile_sensor_data),
                self.create_subscription(
                    LaserScan, REAR_SCAN_TOPIC,
                    lambda message: self._scan_callback(REAR_SCAN_TOPIC, message),
                    qos_profile_sensor_data),
            ))

    def _append(self, record):
        try:
            self.store.append(record)
            return True
        except (OSError, TypeError, ValueError) as error:
            self.failed = True
            self.get_logger().error(
                "evidence loss: could not persist diagnostics: %s" % error)
            return False

    def _persist_raw(self, topic, message, now):
        try:
            record = raw_message_record(
                topic,
                message,
                received_wall_time=time.time(),
                received_monotonic=now,
            )
        except (TypeError, ValueError, OverflowError) as error:
            self._latch_fault(
                "evidence loss: could not serialize raw %s: %s" % (topic, error),
                topic=topic)
            return False
        return self._append(record)

    def _latch_fault(self, reason, *, topic=None, details=None):
        if self.failed:
            return
        self.failed = True
        record = {
            "record_type": "first_fault",
            "classification": "EVIDENCE_FAULT",
            "reason": str(reason),
            "topic": topic,
            "details": details,
            "received_wall_time": time.time(),
            "received_monotonic": time.monotonic(),
            "clock_ns": self.evidence_state.clock.stamp_ns,
            "motion_phase_admitted": self.motion_phase_admitted,
        }
        persistence_error = None
        try:
            self.store.append(record)
            self.store.write_first_fault(record)
        except (OSError, TypeError, ValueError) as error:
            persistence_error = error
        self.get_logger().error("simulation evidence observer fault: %s" % reason)
        if persistence_error is not None:
            self.get_logger().error(
                "evidence loss: could not persist first-fault sidecar: %s" %
                persistence_error)

    def _observe_required(self, topic, stamp_ns, now):
        result = self.evidence_state.observe_stream(topic, stamp_ns, now=now)
        if result in ("invalid", "reversed") and self.motion_phase_admitted:
            self._latch_fault(
                "evidence loss: %s stamp is %s" % (topic, result), topic=topic)
        return result

    def _status_callback(self, message):
        if not self.simulation_evidence:
            try:
                self.store.append(_record(message))
            except (OSError, TypeError, ValueError) as error:
                self.get_logger().error(
                    "could not persist Explorer diagnostics: %s" % error)
                self.failed = True
            return

        now = time.monotonic()
        if not self._persist_raw("/amr/exploration/status", message, now):
            return
        try:
            motion = _frontier_motion_phase(message)
        except (AttributeError, TypeError, ValueError) as error:
            self._latch_fault(
                "evidence loss: malformed Explorer status: %s" % error,
                topic="/amr/exploration/status")
            return
        if not motion or self.motion_phase_admitted:
            return
        issues = list(self.evidence_state.issues(now=now))
        if not self.support_contact_seen:
            issues.append("recognized support contact missing")
        if issues:
            self._latch_fault(
                "evidence loss at first motion phase: %s" % ", ".join(issues),
                topic="/amr/exploration/status",
                details={"issues": issues})
            return
        self.motion_phase_admitted = True
        self._append({
            "record_type": "motion_phase_admitted",
            "received_wall_time": time.time(),
            "received_monotonic": now,
            "clock_ns": self.evidence_state.clock.stamp_ns,
        })

    def _clock_callback(self, message):
        now = time.monotonic()
        if not self._persist_raw(CLOCK_TOPIC, message, now):
            return
        stamp_ns = _time_ns(message.clock)
        result = self.evidence_state.observe_clock(stamp_ns, now=now)
        if result == "advanced":
            contact_stamp = self.evidence_state.streams[CONTACTS_TOPIC].stamp_ns
            if contact_stamp in self._support_contact_stamps:
                self.support_contact_seen = True
            self._support_contact_stamps = {
                stamp for stamp in self._support_contact_stamps
                if stamp > stamp_ns
            }
        elif result in ("invalid", "reversed") and self.motion_phase_admitted:
            self._latch_fault(
                "evidence loss: /clock stamp is %s" % result,
                topic=CLOCK_TOPIC)

    def _contacts_callback(self, message):
        now = time.monotonic()
        if not self._persist_raw(CONTACTS_TOPIC, message, now):
            return
        stamp_ns = _stamp_ns(message)
        if stamp_ns is None:
            self._latch_fault(
                "contact evidence loss: Contacts has an invalid simulation stamp",
                topic=CONTACTS_TOPIC)
            return

        normal_support = False
        classifications = []
        for index, contact in enumerate(message.contacts):
            classification = classify_contact(contact)
            classifications.append({
                "index": index,
                "classification": classification,
                "collision1": _json_value(contact.collision1),
                "collision2": _json_value(contact.collision2),
            })
            if classification == NORMAL_SUPPORT:
                normal_support = True
                continue
            self._append({
                "record_type": "contact_classification",
                "topic": CONTACTS_TOPIC,
                "ros_stamp_ns": stamp_ns,
                "contacts": classifications,
            })
            self._latch_fault(
                "contact fault: %s at contact index %d" % (classification, index),
                topic=CONTACTS_TOPIC,
                details={"index": index, "classification": classification})
            return
        if classifications:
            self._append({
                "record_type": "contact_classification",
                "topic": CONTACTS_TOPIC,
                "ros_stamp_ns": stamp_ns,
                "contacts": classifications,
            })

        result = self._observe_required(CONTACTS_TOPIC, stamp_ns, now)
        if normal_support and result not in ("invalid", "reversed"):
            clock_ns = self.evidence_state.clock.stamp_ns
            if clock_ns is not None and stamp_ns <= clock_ns:
                self.support_contact_seen = True
            else:
                self._support_contact_stamps.add(stamp_ns)

    def _coverage_callback(self, message):
        now = time.monotonic()
        if not self._persist_raw(COVERAGE_TOPIC, message, now):
            return
        try:
            coverage = parse_coverage(message, self.command_receipt_count)
        except ValueError as error:
            self._latch_fault(
                "evidence loss: malformed contact coverage: %s" % error,
                topic=COVERAGE_TOPIC)
            return
        self.command_receipt_count = coverage["command_receipt_count"]
        self._observe_required(
            COVERAGE_TOPIC, coverage["simulation_time_ns"], now)

    def _ground_truth_callback(self, message):
        now = time.monotonic()
        if not self._persist_raw(GROUND_TRUTH_TOPIC, message, now):
            return
        stamp_ns = _stamp_ns(message)
        if stamp_ns is None or not valid_ground_truth(message):
            if self.motion_phase_admitted:
                self._latch_fault(
                    "evidence loss: invalid ground-truth pose",
                    topic=GROUND_TRUTH_TOPIC)
            return
        self._observe_required(GROUND_TRUTH_TOPIC, stamp_ns, now)

    def _scan_callback(self, topic, message):
        now = time.monotonic()
        if not self._persist_raw(topic, message, now):
            return
        stamp_ns = _stamp_ns(message)
        if stamp_ns is None or not valid_laser_scan(message):
            if self.motion_phase_admitted:
                self._latch_fault(
                    "evidence loss: invalid LaserScan on %s" % topic,
                    topic=topic)
            return
        self._observe_required(topic, stamp_ns, now)

    def check_liveness(self, now=None):
        if not self.simulation_evidence or self.failed:
            return
        current = time.monotonic() if now is None else float(now)
        issues = list(self.evidence_state.issues(
            now=current, allow_pending=self.motion_phase_admitted))
        if self.motion_phase_admitted:
            if issues:
                self._latch_fault(
                    "evidence loss: %s" % ", ".join(issues),
                    details={"issues": issues})
            return
        if not self.support_contact_seen:
            issues.append("recognized support contact missing")
        missing = tuple(issues)
        if missing and missing != self._last_missing:
            self.get_logger().warning(
                "simulation evidence missing before navigation: %s" %
                ", ".join(missing))
        self._last_missing = missing

    def close(self):
        try:
            self.store.close()
        except OSError:
            self.failed = True


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--topic", default="/amr/exploration/status")
    parser.add_argument("--simulation-evidence", action="store_true", default=False)
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    if args.output is None and args.run_dir is None:
        raise SystemExit("one of --run-dir or --output is required")
    output = args.output or args.run_dir / "evidence" / "exploration_diagnostics.jsonl"
    rclpy.init(args=None)
    node = ExplorationDiagnostics(
        output, args.topic, simulation_evidence=args.simulation_evidence)
    try:
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_liveness()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
