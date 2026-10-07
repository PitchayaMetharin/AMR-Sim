#!/usr/bin/env python3
"""Analyze one recorded Gate 6 bag without inferring a terminal pass.

The analyzer is intentionally independent of the stage process.  It derives
the selected product and slot from the factory registry, identifies one
product-specific mass-stage interval from its ``source_boot_id`` and explicit
terminal status, and checks only samples inside that interval.  A pass is
written as one stable machine-readable line; diagnostics for a failed bag go
to stderr and the output file contains the corresponding FAIL line.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import re
import sys
from types import SimpleNamespace
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


PRODUCT_IDS = (101, 102, 103)
SLOT_BY_PRODUCT = {101: "dispatch_1", 102: "dispatch_2", 103: "dispatch_3"}
# The recorder can connect after the one-shot STARTING status has already been
# published.  The mass-stage source_boot_id is unique to this process and is a
# durable stage boundary for every status sample that was captured.
STAGE_START_MARKER = "Gate 6 mass stage is starting"
BOOTSTRAP_TOPIC = "/amr/simulation/attachment_bootstrap/status"
NORMAL_NAV_STATUS_TOPIC = "/amr/mission/navigate_to_pose/_action/status"
PRECISE_NAV_STATUS_TOPIC = "/amr/mission/navigate_to_pose_precise/_action/status"
CONTROL_TOPIC = "/amr/control/cmd_vel"
SIMULATION_TOPIC = "/amr/simulation/base/cmd_vel"
COMMAND_FORWARDING_MAX_AGE_SECONDS = 0.25
OWNERSHIP_LOG_PREFIX = "AMR_CYCLE_OWNERSHIP_V1 "
OWNERSHIP_LOGGER = "amr.manipulation_supervisor_node"
OWNERSHIP_MAX_RECORD_BYTES = 4096
INTERNAL_STATUS_TOPIC = "/amr/manipulation/internal/status"
EXECUTE_CYCLE_STATUS_TOPIC = "/amr/manipulation/execute_product_cycle/_action/status"
EXECUTE_CYCLE_FEEDBACK_TOPIC = "/amr/manipulation/execute_product_cycle/_action/feedback"
OWNERSHIP_STALE_DETAIL = "internal manipulation status is stale or inconsistent"

# This is the recorder contract for a strict Gate 6 run.  The precise-action
# status topic is deliberately omitted: old and current Nav2 graphs may emit
# zero messages there, while normal navigation must be observed.
REQUIRED_TOPIC_SUFFIXES = (
    "/clock",
    "/tf",
    "/tf_static",
    "/amr/base/joint_states",
    "/amr/base/odometry_raw",
    "/amr/base/status",
    "/amr/simulation/base/joint_states",
    "/amr/simulation/base/cmd_vel",
    "/amr/simulation/ground_truth/pose",
    "/amr/control/cmd_vel",
    "/amr/mission/navigate_to_pose/_action/status",
    "/amr/follow_path/_action/status",
    "/arm_controller/follow_joint_trajectory/_action/status",
    "/gripper_controller/gripper_cmd/_action/status",
    "/amr/manipulation/status",
    "/amr/simulation/contacts/left_finger",
    "/amr/simulation/contacts/right_finger",
    "/amr/simulation/sensors/rear_lidar/scan",
    "/amr/sensors/rear_lidar/scan",
    BOOTSTRAP_TOPIC,
)


class AnalysisError(RuntimeError):
    """A bag did not prove the complete Gate 6 contract."""


def _finite(values: Iterable[object], label: str) -> Tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise AnalysisError(f"{label} is non-numeric") from error
    if not result or not all(math.isfinite(value) for value in result):
        raise AnalysisError(f"{label} is non-finite")
    return result


def _load_product_registry(product_id: int) -> Tuple[str, float, Tuple[float, float, float]]:
    if product_id not in PRODUCT_IDS:
        raise AnalysisError(f"product_id must be one of {PRODUCT_IDS}")
    try:
        factory = Path(get_package_share_directory("amr_factory"))
        with (factory / "config" / "products.yaml").open(encoding="utf-8") as stream:
            registry = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise AnalysisError("factory product registry is unavailable") from error
    products = registry.get("products") if isinstance(registry, dict) else None
    slots = registry.get("dispatch_slots") if isinstance(registry, dict) else None
    if not isinstance(products, dict) or not isinstance(slots, list):
        raise AnalysisError("factory product registry is incomplete")
    selected_model = None
    selected_mass = None
    seen_ids = set()
    for model, raw_entry in products.items():
        if not isinstance(raw_entry, dict):
            raise AnalysisError(f"registry entry for {model} is invalid")
        try:
            current_id = int(raw_entry["tag_id"])
            mass = float(raw_entry["mass"])
        except (KeyError, TypeError, ValueError) as error:
            raise AnalysisError(f"registry entry for {model} is incomplete") from error
        if current_id in seen_ids:
            raise AnalysisError(f"duplicate product ID {current_id}")
        seen_ids.add(current_id)
        if current_id not in PRODUCT_IDS or not math.isfinite(mass) or mass < 0.0:
            raise AnalysisError(f"unsupported or invalid product entry {model}")
        if current_id == product_id:
            selected_model, selected_mass = str(model), mass
    if seen_ids != set(PRODUCT_IDS) or selected_model is None or selected_mass is None:
        raise AnalysisError("registry does not contain exactly products 101, 102, and 103")
    slot_id = SLOT_BY_PRODUCT[product_id]
    slot = next((entry for entry in slots if isinstance(entry, dict) and entry.get("id") == slot_id), None)
    if slot is None:
        raise AnalysisError(f"dispatch slot {slot_id} is missing")
    try:
        slot_position = _finite((slot["x"], slot["y"], slot["z"]), f"dispatch slot {slot_id}")
    except (KeyError, TypeError) as error:
        raise AnalysisError(f"dispatch slot {slot_id} is invalid") from error
    if len(slot_position) != 3:
        raise AnalysisError(f"dispatch slot {slot_id} is incomplete")
    return selected_model, selected_mass, slot_position


def _storage_id(bag: Path) -> str:
    metadata = bag / "metadata.yaml"
    try:
        data = yaml.safe_load(metadata.read_text(encoding="utf-8"))
        return str(data["rosbag2_bagfile_information"]["storage_identifier"])
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return "sqlite3"


def _open_reader(bag: Path):
    if not bag.exists():
        raise AnalysisError(f"bag path does not exist: {bag}")
    storage_ids = [_storage_id(bag), "sqlite3", "mcap"]
    tried = set()
    last_error = None
    for storage_id in storage_ids:
        if storage_id in tried:
            continue
        tried.add(storage_id)
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
            return reader
        except Exception as error:  # noqa: BLE001 - try the declared fallback storage
            last_error = error
    raise AnalysisError(f"could not open rosbag: {last_error}")


def _message_stamp(message, fallback: float) -> float:
    header = getattr(message, "header", None)
    if header is None:
        return fallback
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if math.isfinite(value) else fallback


def _yaw_from_pose(pose) -> float:
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _command_values(message) -> Tuple[float, float]:
    twist = getattr(message, "twist", message)
    return float(twist.linear.x), float(twist.angular.z)


def _contact_has_model(message, model: str) -> bool:
    return any(
        model in contact.collision1.name or model in contact.collision2.name
        for contact in message.contacts
    )


def _interval_samples(samples: Sequence[Tuple[float, object]], start: float, end: float):
    return [sample for sample in samples if start <= sample[0] <= end]


def select_stage_status_stream(
        status_samples: Sequence[Tuple[float, object]], product_id: int
) -> Optional[Tuple[int, List[Tuple[float, object]]]]:
    """Select exactly one product-specific mass-stage interval.

    The shared outer supervisor can reuse one ``source_boot_id`` across
    multiple product cycles.  Each candidate therefore starts at an explicit
    mass-stage marker and ends at the first valid empty-stowed status after
    that marker.  Only a candidate containing the requested retained-loaded
    status is eligible; missing boundaries or multiple eligible candidates
    fail closed.
    """
    streams: Dict[int, List[Tuple[float, object]]] = defaultdict(list)
    for sample in status_samples:
        try:
            timestamp, message = sample
            source_boot_id = int(message.source_boot_id)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        if source_boot_id <= 0:
            continue
        streams[source_boot_id].append((timestamp, message))

    if not streams:
        return None
    if len(streams) == 1:
        stage_boot_id, stage_statuses = next(iter(streams.items()))
    else:
        marker_streams = [
            (boot_id, statuses) for boot_id, statuses in streams.items()
            if any(getattr(message, "detail", None) == STAGE_START_MARKER
                   for _, message in statuses)
        ]
        if len(marker_streams) != 1:
            return None
        stage_boot_id, stage_statuses = marker_streams[0]

    ordered_statuses = sorted(stage_statuses, key=lambda item: item[0])

    def is_empty_stowed(message) -> bool:
        try:
            return (bool(message.valid) and int(message.state) == 1 and
                    not bool(message.product_attached))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    def is_loaded_product(message) -> bool:
        try:
            return (int(message.state) == 2 and bool(message.product_attached) and
                    message.product_id == str(product_id))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    marker_indices = []
    index = 0
    while index < len(ordered_statuses):
        if getattr(ordered_statuses[index][1], "detail", None) != STAGE_START_MARKER:
            index += 1
            continue
        marker_indices.append(index)
        index += 1
        while (index < len(ordered_statuses) and
               getattr(ordered_statuses[index][1], "detail", None) == STAGE_START_MARKER):
            index += 1
    if not marker_indices:
        return None

    candidates = []
    for marker_index in marker_indices:
        terminal_index = next(
            (index for index in range(marker_index + 1, len(ordered_statuses))
             if is_empty_stowed(ordered_statuses[index][1])),
            None,
        )
        if terminal_index is None:
            return None
        candidate = ordered_statuses[marker_index:terminal_index + 1]
        if any(is_loaded_product(message) for _, message in candidate):
            candidates.append(candidate)

    if len(candidates) != 1:
        return None
    return stage_boot_id, candidates[0]


_OWNERSHIP_BASE_FIELDS = {
    "schema", "version", "event", "record_index", "execution_uuid", "product_id",
    "public_source_boot_id", "captured_monotonic_s", "public", "child",
    "child_received_monotonic_s", "child_age_s", "owned_child_boot_id",
    "owned_child_sequence", "child_consistent", "authority_open", "stage_started",
    "stage_loaded_proof", "terminal_empty_proof", "fault_latched", "cancel_requested",
    "observed_history_violations",
}
_OWNERSHIP_CLOSE_FIELDS = {
    "active_owner_match", "goal_reserved", "public_sequence_high_water", "current_state",
    "current_detail", "current_base_motion_allowed", "current_product_attached",
    "current_product_id", "child_exit_code", "child_alive", "callback_child_present",
    "child_reference_consistent", "result_product_id", "result_outcome",
    "result_delivered", "first_boundaries", "captured_record_count",
}
_OWNERSHIP_EVENTS = {"START", "LOADED", "EMPTY", "CLOSE"}
_OWNERSHIP_UUID = re.compile(r"^[0-9a-f]{32}$")


def _reject_duplicate_json_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise AnalysisError("ownership record has a duplicate JSON key")
        result[key] = value
    return result


def _reject_json_constant(value):
    raise AnalysisError(f"ownership record contains non-finite JSON value {value}")


def _finite_json_number(value, label: str, allow_none: bool = False) -> bool:
    if value is None and allow_none:
        return True
    if type(value) not in (int, float) or not math.isfinite(float(value)):
        raise AnalysisError(f"ownership {label} is not finite numeric data")
    return True


def _validate_wire_status(value, label: str) -> None:
    if not isinstance(value, dict) or set(value) != {
            "header", "source_boot_id", "sequence", "valid", "state",
            "base_motion_allowed", "product_attached", "product_id", "detail"}:
        raise AnalysisError(f"ownership {label} status schema is invalid")
    header = value["header"]
    if not isinstance(header, dict) or set(header) != {"stamp", "frame_id"}:
        raise AnalysisError(f"ownership {label} header schema is invalid")
    stamp = header["stamp"]
    if not isinstance(stamp, dict) or set(stamp) != {"sec", "nanosec"}:
        raise AnalysisError(f"ownership {label} stamp schema is invalid")
    if (type(stamp["sec"]) is not int or type(stamp["nanosec"]) is not int or
            stamp["sec"] < 0 or not 0 <= stamp["nanosec"] < 1_000_000_000 or
            type(header["frame_id"]) is not str):
        raise AnalysisError(f"ownership {label} stamp/frame is invalid")
    for name in ("source_boot_id", "sequence", "state"):
        if type(value[name]) is not int:
            raise AnalysisError(f"ownership {label}.{name} is not an integer")
    if (value["source_boot_id"] <= 0 or value["sequence"] <= 0 or
            value["source_boot_id"] > 0xFFFFFFFFFFFFFFFF or
            value["sequence"] > 0xFFFFFFFFFFFFFFFF):
        raise AnalysisError(f"ownership {label} identity is not positive")
    for name in ("valid", "base_motion_allowed", "product_attached"):
        if type(value[name]) is not bool:
            raise AnalysisError(f"ownership {label}.{name} is not boolean")
    for name in ("product_id", "detail"):
        if type(value[name]) is not str:
            raise AnalysisError(f"ownership {label}.{name} is not text")


def _validate_ownership_record(record: object) -> dict:
    if not isinstance(record, dict):
        raise AnalysisError("ownership record root is not an object")
    event = record.get("event")
    if type(event) is not str or event not in _OWNERSHIP_EVENTS:
        raise AnalysisError("ownership record event is unknown")
    expected_fields = _OWNERSHIP_BASE_FIELDS | (
        _OWNERSHIP_CLOSE_FIELDS if event == "CLOSE" else set())
    if set(record) != expected_fields:
        raise AnalysisError("ownership record fields do not match the versioned schema")
    if record["schema"] != "AMR_CYCLE_OWNERSHIP_V1" or type(record["version"]) is not int or record["version"] != 1:
        raise AnalysisError("ownership record schema/version is unsupported")
    if (type(record["record_index"]) is not int or
            not 1 <= record["record_index"] <= 64):
        raise AnalysisError("ownership record index is outside its bound")
    if type(record["execution_uuid"]) is not str or not _OWNERSHIP_UUID.fullmatch(record["execution_uuid"]):
        raise AnalysisError("ownership execution UUID is not a full 16-byte UUID")
    if type(record["product_id"]) is not str or not record["product_id"]:
        raise AnalysisError("ownership product identity is invalid")
    if (type(record["public_source_boot_id"]) is not int or
            record["public_source_boot_id"] <= 0 or
            type(record["owned_child_boot_id"]) is not int or
            record["owned_child_boot_id"] < 0):
        raise AnalysisError("ownership boot identity is invalid")
    _finite_json_number(record["captured_monotonic_s"], "capture time")
    _finite_json_number(record["child_received_monotonic_s"], "child receipt time", True)
    _finite_json_number(record["child_age_s"], "child age", True)
    if record["owned_child_sequence"] is not None and type(record["owned_child_sequence"]) is not int:
        raise AnalysisError("ownership child sequence is not an integer")
    if event == "CLOSE" and (
            type(record["owned_child_sequence"]) is not int or
            record["owned_child_sequence"] <= 0):
        raise AnalysisError("ownership CLOSE child sequence is invalid")
    for name in ("child_consistent", "authority_open", "stage_started",
                 "stage_loaded_proof", "terminal_empty_proof", "fault_latched",
                 "cancel_requested"):
        if type(record[name]) is not bool:
            raise AnalysisError(f"ownership {name} is not boolean")
    history = record["observed_history_violations"]
    if (not isinstance(history, list) or
            any(type(item) is not str for item in history) or
            len(history) != len(set(history))):
        raise AnalysisError("ownership history violations are malformed")
    if event == "CLOSE":
        if record["public"] is not None:
            raise AnalysisError("ownership CLOSE unexpectedly carries a public boundary")
        if record["child"] is not None:
            _validate_wire_status(record["child"], "close child")
        for name in ("active_owner_match", "goal_reserved", "current_base_motion_allowed",
                     "current_product_attached", "child_alive", "callback_child_present",
                     "child_reference_consistent", "result_delivered"):
            if type(record[name]) is not bool:
                raise AnalysisError(f"ownership CLOSE {name} is not boolean")
        for name in ("public_sequence_high_water", "current_state", "captured_record_count"):
            if type(record[name]) is not int:
                raise AnalysisError(f"ownership CLOSE {name} is not an integer")
        for name in ("current_detail", "current_product_id"):
            if type(record[name]) is not str:
                raise AnalysisError(f"ownership CLOSE {name} is not text")
        for name in ("child_exit_code", "result_outcome"):
            if record[name] is not None and type(record[name]) is not int:
                raise AnalysisError(f"ownership CLOSE {name} is not an integer or null")
        if record["result_product_id"] is not None and type(record["result_product_id"]) is not str:
            raise AnalysisError("ownership CLOSE result product is invalid")
        boundaries = record["first_boundaries"]
        if not isinstance(boundaries, dict) or set(boundaries) != {"START", "LOADED", "EMPTY"}:
            raise AnalysisError("ownership CLOSE boundary references are malformed")
        for reference in boundaries.values():
            if reference is not None and (
                    not isinstance(reference, dict) or set(reference) != {
                        "record_index", "source_boot_id", "sequence"} or
                    any(type(reference[key]) is not int for key in reference) or
                    any(reference[key] <= 0 for key in reference)):
                raise AnalysisError("ownership CLOSE boundary reference is invalid")
    else:
        if record["public"] is None or record["child"] is None:
            raise AnalysisError("ownership boundary is missing its public or child snapshot")
        _validate_wire_status(record["public"], "public")
        _validate_wire_status(record["child"], "child")
        if record["owned_child_boot_id"] <= 0 or record["owned_child_sequence"] is None:
            raise AnalysisError("ownership boundary lacks owned child identity")
        _finite_json_number(record["child_received_monotonic_s"], "boundary child receipt")
        _finite_json_number(record["child_age_s"], "boundary child age")
    return record


def _parse_ownership_records(rosout_records: Sequence[Tuple[int, str, str]]):
    parsed = []
    for receipt_ns, logger_name, message_text in rosout_records:
        if type(message_text) is not str or not message_text.startswith(OWNERSHIP_LOG_PREFIX):
            continue
        if type(receipt_ns) is not int:
            raise AnalysisError("ownership rosout receipt is not an integer nanosecond value")
        if len(message_text.encode("utf-8")) > OWNERSHIP_MAX_RECORD_BYTES:
            raise AnalysisError("ownership rosout record exceeds 4096 UTF-8 bytes")
        if logger_name != OWNERSHIP_LOGGER:
            raise AnalysisError("ownership record came from an unexpected logger")
        try:
            record = json.loads(
                message_text[len(OWNERSHIP_LOG_PREFIX):],
                object_pairs_hook=_reject_duplicate_json_keys,
                parse_constant=_reject_json_constant)
        except (json.JSONDecodeError, UnicodeError, TypeError) as error:
            raise AnalysisError("ownership rosout JSON is malformed") from error
        parsed.append((receipt_ns, _validate_ownership_record(record)))
    if not parsed:
        raise AnalysisError("ownership rosout records are missing")
    seen = set()
    for _, record in parsed:
        identity = (record["execution_uuid"], record["record_index"])
        if identity in seen:
            raise AnalysisError("ownership record index is duplicated")
        seen.add(identity)
    return parsed


def _raw_status_fingerprint(message) -> dict:
    stamp = message.header.stamp
    return {
        "header": {
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "frame_id": str(message.header.frame_id),
        },
        "source_boot_id": int(message.source_boot_id),
        "sequence": int(message.sequence),
        "valid": bool(message.valid),
        "state": int(message.state),
        "base_motion_allowed": bool(message.base_motion_allowed),
        "product_attached": bool(message.product_attached),
        "product_id": str(message.product_id),
        "detail": str(message.detail),
    }


def _message_consistent(message, product_id: int) -> bool:
    try:
        if (type(message.state) is not int or type(message.valid) is not bool or
                type(message.base_motion_allowed) is not bool or
                type(message.product_attached) is not bool or
                type(message.product_id) is not str or type(message.detail) is not str):
            return False
        state = int(message.state)
        attached = bool(message.product_attached)
        return all((
            bool(message.valid), state in (0, 1, 2, 3, 4, 5), state != 5,
            (not attached or str(message.product_id) == str(product_id)),
            (attached or str(message.product_id) == ""),
            (state != 1 or not attached),
            (state != 2 or attached),
            (state not in (0, 3, 4, 5) or not bool(message.base_motion_allowed)),
        ))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


def _ownership_boundary_matches(record: dict, message, product_id: int,
                                expected_event: str, child_boot_id: int) -> bool:
    public = record["public"]
    child = record["child"]
    try:
        if public != _raw_status_fingerprint(message):
            return False
        captured = record["captured_monotonic_s"]
        received = record["child_received_monotonic_s"]
        age = record["child_age_s"]
        if (record["event"] != expected_event or
                record["product_id"] != str(product_id) or
                record["owned_child_boot_id"] != child_boot_id or
                record["child"]["source_boot_id"] != child_boot_id or
                record["owned_child_sequence"] != child["sequence"] or
                not record["child_consistent"] or not record["authority_open"] or
                not record["stage_started"] or record["fault_latched"] or
                record["cancel_requested"] or record["observed_history_violations"] or
                not all(math.isfinite(value) for value in (captured, received, age)) or
                received < 0.0 or captured < received or
                age != captured - received or age < 0.0 or age > 0.2):
            return False
        if not _message_consistent(message, product_id) or not _message_consistent(
                SimpleNamespace(**{
                    "valid": child["valid"], "state": child["state"],
                    "base_motion_allowed": child["base_motion_allowed"],
                    "product_attached": child["product_attached"],
                    "product_id": child["product_id"],
                    "detail": child["detail"]}), product_id):
            return False
        expected_state = {"START": 0, "LOADED": 2, "EMPTY": 1}[expected_event]
        if (public["state"] != expected_state or child["state"] != expected_state or
                not public["valid"] or not child["valid"]):
            return False
        if expected_event == "START":
            return all((
                public["detail"] == STAGE_START_MARKER,
                child["detail"] == STAGE_START_MARKER,
                not public["base_motion_allowed"], not child["base_motion_allowed"],
                not public["product_attached"], not child["product_attached"],
                public["product_id"] == "", child["product_id"] == "",
                not record["stage_loaded_proof"], not record["terminal_empty_proof"],
            ))
        if expected_event == "LOADED":
            return all((
                public["base_motion_allowed"], public["product_attached"],
                public["product_id"] == str(product_id),
                child["base_motion_allowed"], child["product_attached"],
                child["product_id"] == str(product_id),
                record["stage_loaded_proof"], not record["terminal_empty_proof"],
            ))
        return all((
            public["base_motion_allowed"], not public["product_attached"],
            public["product_id"] == "", child["base_motion_allowed"],
            not child["product_attached"], child["product_id"] == "",
            record["stage_loaded_proof"], record["terminal_empty_proof"],
        ))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def _recognizable_stage_claim(message) -> bool:
    try:
        state = int(message.state)
        detail = str(message.detail)
        return any((
            detail == STAGE_START_MARKER,
            state == 3 and detail ==
            "Arm command inhibited pending fresh READY and 500 ms stationary evidence",
            state == 2 and detail.startswith("Gate 6 "),
            state == 2 and bool(message.product_attached),
        ))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


def _check_internal_status_span(internal_samples, start_ns: int, end_ns: int,
                                product_id: int, owned_boot_id: int) -> None:
    if any(type(receipt) is not int or receipt < 0 for receipt, _ in internal_samples):
        raise AnalysisError("raw internal status receipt is not integer nanoseconds")
    span = [(receipt, message) for receipt, message in internal_samples
            if start_ns <= receipt <= end_ns]
    if not span:
        raise AnalysisError("raw internal status is absent inside the conservative fence")
    last_sequence = {}
    saw_owned = False
    previous_receipt = None
    for receipt, message in span:
        if previous_receipt is not None and receipt < previous_receipt:
            raise AnalysisError("raw internal status receipt time rolled back")
        previous_receipt = receipt
        try:
            boot_id = message.source_boot_id
            sequence = message.sequence
            state = message.state
            if any(type(value) is not int for value in (boot_id, sequence, state)):
                raise TypeError("identity fields are not integers")
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise AnalysisError("raw internal status is malformed") from error
        if boot_id <= 0 or sequence <= 0:
            raise AnalysisError("raw internal status identity is invalid")
        if sequence <= last_sequence.get(boot_id, 0):
            raise AnalysisError("raw internal status sequence is not increasing")
        last_sequence[boot_id] = sequence
        if not _message_consistent(message, product_id) or state == 5:
            raise AnalysisError("raw internal status contains invalid/FAULT evidence")
        if boot_id == owned_boot_id:
            saw_owned = True
        elif _recognizable_stage_claim(message):
            raise AnalysisError("foreign raw internal stage claim was recorded")
    if not saw_owned:
        raise AnalysisError("owned raw internal status is absent inside the conservative fence")


def _action_identity(status_entry):
    try:
        raw_uuid = status_entry.goal_info.goal_id.uuid
        if any(type(value) is not int or not 0 <= value <= 255 for value in raw_uuid):
            raise AnalysisError("ExecuteProductCycle status UUID byte is invalid")
        uuid_bytes = bytes(raw_uuid)
        stamp = status_entry.goal_info.stamp
        if (type(stamp.sec) is not int or type(stamp.nanosec) is not int or
                type(status_entry.status) is not int):
            raise AnalysisError("ExecuteProductCycle status fields are not integers")
        stamp_pair = (stamp.sec, stamp.nanosec)
        status = status_entry.status
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise AnalysisError("ExecuteProductCycle status entry is malformed") from error
    if len(uuid_bytes) != 16 or stamp_pair[0] < 0 or not 0 <= stamp_pair[1] < 1_000_000_000:
        raise AnalysisError("ExecuteProductCycle status UUID/stamp is invalid")
    return uuid_bytes.hex(), stamp_pair, status


def _check_action_evidence(status_samples, feedback_samples, start_ns: int, empty_ns: int,
                           close_receipt_ns: int, product_id: int,
                           execution_uuid: str):
    status_events = []
    previous_action_receipt = None
    for receipt, message in status_samples:
        if type(receipt) is not int or receipt < 0:
            raise AnalysisError("ExecuteProductCycle status receipt is not integer nanoseconds")
        if previous_action_receipt is not None and receipt < previous_action_receipt:
            raise AnalysisError("ExecuteProductCycle status receipt time rolled back")
        previous_action_receipt = receipt
        for entry in message.status_list:
            uuid_hex, stamp, status = _action_identity(entry)
            status_events.append((receipt, uuid_hex, stamp, status))
    target = [(receipt, stamp, status) for receipt, uuid_hex, stamp, status in status_events
              if uuid_hex == execution_uuid]
    if not target:
        raise AnalysisError("matching ExecuteProductCycle status identity is absent")
    if len({stamp for _, stamp, _ in target}) != 1:
        raise AnalysisError("matching ExecuteProductCycle GoalInfo stamp changed")
    if any(status in (3, 5, 6) for _, _, status in target):
        raise AnalysisError("matching ExecuteProductCycle status records cancellation/abort")
    if any(status not in (1, 2, 4) for _, _, status in target):
        raise AnalysisError("matching ExecuteProductCycle status phase is unknown")
    ordered_target = sorted(target, key=lambda item: item[0])
    rank = {1: 0, 2: 1, 4: 2}
    if any(rank[right[2]] < rank[left[2]]
           for left, right in zip(ordered_target, ordered_target[1:])):
        raise AnalysisError("matching ExecuteProductCycle status chronology regressed")
    accepted = [receipt for receipt, _, status in target if status == 1]
    executing = [receipt for receipt, _, status in target if status == 2]
    succeeded = [receipt for receipt, _, status in target if status == 4]
    if (not accepted or not executing or min(accepted) >= start_ns or
            min(executing) >= start_ns or max(accepted) >= min(executing)):
        raise AnalysisError("accepted/executing action identity is not proven before START")
    if not succeeded:
        raise AnalysisError("successful action terminal is not recorded after EMPTY")
    success_ns = min(succeeded)
    if success_ns <= empty_ns:
        raise AnalysisError("earliest successful action terminal is not after EMPTY")
    upper_ns = max(close_receipt_ns, success_ns)
    if any(uuid_hex != execution_uuid and status in (1, 2, 3) and
           start_ns <= receipt <= upper_ns
           for receipt, uuid_hex, _, status in status_events):
        raise AnalysisError("another ExecuteProductCycle execution was active in the conservative fence")

    expected_names = {1: "PREPARING", 2: "EXECUTING", 3: "CANCELING"}
    feedback_rows = []
    previous_feedback_receipt = None
    for receipt, message in feedback_samples:
        if type(receipt) is not int or receipt < 0:
            raise AnalysisError("ExecuteProductCycle feedback receipt is not integer nanoseconds")
        if previous_feedback_receipt is not None and receipt < previous_feedback_receipt:
            raise AnalysisError("ExecuteProductCycle feedback receipt time rolled back")
        previous_feedback_receipt = receipt
        try:
            raw_uuid = message.goal_id.uuid
            if any(type(value) is not int or not 0 <= value <= 255 for value in raw_uuid):
                raise ValueError("feedback UUID byte is invalid")
            uuid_bytes = bytes(raw_uuid)
            feedback = message.feedback
            if (type(feedback.phase) is not int or type(feedback.phase_name) is not str or
                    type(feedback.product_id) is not str or
                    type(feedback.product_attached) is not bool):
                raise TypeError("feedback fields have invalid types")
            phase = feedback.phase
            phase_name = feedback.phase_name
            feedback_product = feedback.product_id
            attached = feedback.product_attached
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise AnalysisError("ExecuteProductCycle feedback is malformed") from error
        if len(uuid_bytes) != 16:
            raise AnalysisError("ExecuteProductCycle feedback UUID is invalid")
        if uuid_bytes.hex() != execution_uuid:
            if (start_ns <= receipt <= upper_ns and phase in (1, 2, 3) and
                    expected_names.get(phase) == phase_name):
                raise AnalysisError(
                    "foreign ExecuteProductCycle feedback is active in the conservative fence")
            continue
        feedback_rows.append((receipt, phase, phase_name, feedback_product, attached))
    if not feedback_rows:
        raise AnalysisError("matching ExecuteProductCycle feedback is absent")
    if any(product != str(product_id) or phase == 3 or phase_name == "CANCELING"
           for _, phase, phase_name, product, _ in feedback_rows):
        raise AnalysisError("matching ExecuteProductCycle feedback contradicts product/cancel state")
    if any(expected_names.get(phase) != name for _, phase, name, _, _ in feedback_rows):
        raise AnalysisError("matching ExecuteProductCycle feedback phase/name conflicts")
    ordered_feedback = sorted(feedback_rows, key=lambda item: item[0])
    if (any(left[1] > right[1] for left, right in
            zip(ordered_feedback, ordered_feedback[1:])) or
            any(receipt > success_ns for receipt, _, _, _, _ in ordered_feedback)):
        raise AnalysisError("matching ExecuteProductCycle feedback chronology conflicts")
    if not any(phase == 2 and start_ns <= receipt <= success_ns
               for receipt, phase, _, _, _ in feedback_rows):
        raise AnalysisError("matching ExecuteProductCycle EXECUTING feedback is absent")
    return success_ns, upper_ns


def select_corroborated_stage_status_stream(
        status_samples: Sequence[Tuple[int, object]],
        internal_samples: Sequence[Tuple[int, object]],
        action_status_samples: Sequence[Tuple[int, object]],
        action_feedback_samples: Sequence[Tuple[int, object]],
        rosout_records: Sequence[Tuple[int, str, str]],
        product_id: int) -> Tuple[int, List[Tuple[float, object]]]:
    """Recover one fail-closed B interval using a sealed observer epoch."""
    if type(product_id) is not int or product_id <= 0:
        raise AnalysisError("requested ownership product identity is invalid")
    by_boot = defaultdict(list)
    for receipt_ns, message in status_samples:
        if type(receipt_ns) is not int or receipt_ns < 0:
            raise AnalysisError("public status receipt is not integer nanoseconds")
        try:
            boot_id = message.source_boot_id
            detail = message.detail
            if type(boot_id) is not int or type(detail) is not str:
                raise TypeError("boot/detail fields have invalid types")
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise AnalysisError("public status is malformed") from error
        if detail == STAGE_START_MARKER and boot_id <= 0:
            raise AnalysisError("public START marker has invalid boot identity")
        if boot_id > 0:
            by_boot[boot_id].append((receipt_ns, message))

    marked_boots = {
        boot_id for boot_id, rows in by_boot.items()
        if any(getattr(message, "detail", None) == STAGE_START_MARKER
               for _, message in rows)
    }
    if len(marked_boots) != 1:
        raise AnalysisError(
            "public START markers do not identify exactly one positive source boot")

    candidates = []
    for public_boot, rows in by_boot.items():
        markers = []
        for index, (_, message) in enumerate(rows):
            if str(getattr(message, "detail", "")) == STAGE_START_MARKER:
                markers.append(index)
        blocks = []
        for index in markers:
            if not blocks or index != blocks[-1][-1] + 1:
                blocks.append([index])
            else:
                blocks[-1].append(index)
        for block in blocks:
            empty_index = None
            first_target_load = None
            for index in range(block[-1] + 1, len(rows)):
                _, message = rows[index]
                try:
                    state = message.state
                    if type(state) is not int:
                        raise TypeError("public candidate state is not an integer")
                    attached = bool(message.product_attached)
                    attached_product = str(message.product_id)
                    valid = bool(message.valid)
                except (AttributeError, TypeError, ValueError, OverflowError) as error:
                    raise AnalysisError("public candidate status is malformed") from error
                if (first_target_load is None and state == 2 and attached and
                        attached_product == str(product_id)):
                    first_target_load = index
                if state == 1 and valid and not attached:
                    empty_index = index
                    break
            if empty_index is None:
                raise AnalysisError(
                    "public START marker block lacks a first valid detached EMPTY terminal")
            if first_target_load is not None and first_target_load < empty_index:
                candidates.append({
                    "boot": public_boot, "block": block,
                    "loaded": first_target_load, "empty": empty_index,
                })

    if len(candidates) < 2:
        raise AnalysisError("overlapping requested-product START blocks are not present")
    groups = defaultdict(list)
    for candidate in candidates:
        groups[(candidate["boot"], candidate["loaded"], candidate["empty"])].append(candidate)
    if len(groups) != 1:
        raise AnalysisError("requested-product marker candidates have distinct boundaries")
    (public_boot, loaded_index, empty_index), selected_candidates = next(iter(groups.items()))
    if len(selected_candidates) < 2:
        raise AnalysisError("shared requested-product START blocks are incomplete")
    rows = by_boot[public_boot]
    selected_candidates.sort(key=lambda item: item["block"][0])
    first_index = selected_candidates[0]["block"][0]
    marker_indices = [index for index, (_, message) in enumerate(rows)
                      if str(getattr(message, "detail", "")) == STAGE_START_MARKER]
    if any(loaded_index <= index <= empty_index for index in marker_indices):
        raise AnalysisError("START marker appeared at or after LOADED")

    for left, right in zip(selected_candidates, selected_candidates[1:]):
        for _, message in rows[left["block"][-1] + 1:right["block"][0]]:
            try:
                intermarker_state = message.state
            except AttributeError as error:
                raise AnalysisError("inter-marker public row is malformed") from error
            if not all((
                    _message_consistent(message, product_id),
                    intermarker_state == 0,
                    not bool(message.base_motion_allowed),
                    not bool(message.product_attached), str(message.product_id) == "",
                    str(message.detail) == OWNERSHIP_STALE_DETAIL,
            )):
                raise AnalysisError("inter-marker public row is not a safe stale diagnostic")

    start_receipt, start_message = rows[first_index]
    loaded_receipt, loaded_message = rows[loaded_index]
    empty_receipt, empty_message = rows[empty_index]
    span_rows = rows[first_index:empty_index + 1]
    previous_receipt = None
    previous_sequence = None
    for receipt_ns, message in span_rows:
        try:
            sequence = message.sequence
            state = message.state
            detail = message.detail
            if (type(sequence) is not int or type(state) is not int or
                    type(detail) is not str):
                raise TypeError("public status fields have invalid types")
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise AnalysisError("public status sequence/detail is malformed") from error
        if sequence <= 0 or (previous_sequence is not None and sequence != previous_sequence + 1):
            raise AnalysisError("public status sequence is not positive and contiguous")
        if previous_receipt is not None and receipt_ns < previous_receipt:
            raise AnalysisError("public status receipt time rolled back")
        previous_sequence, previous_receipt = sequence, receipt_ns
        if not _message_consistent(message, product_id):
            raise AnalysisError("public status span contains invalid/fault/wrong-product evidence")
        if detail == STAGE_START_MARKER and not all((
                state == 0, bool(message.valid), not bool(message.base_motion_allowed),
                not bool(message.product_attached), str(message.product_id) == "")):
            raise AnalysisError("public START marker fields contradict their boundary")
        if state == 1 and bool(message.product_attached):
            raise AnalysisError("public EMPTY row retains attachment")
    if not (start_receipt <= loaded_receipt <= empty_receipt):
        raise AnalysisError("public marker candidate receipt bounds are reversed")

    parsed_records = _parse_ownership_records(rosout_records)
    grouped = defaultdict(list)
    for record_receipt, record in parsed_records:
        if record_receipt < 0:
            raise AnalysisError("ownership rosout receipt is negative")
        grouped[record["execution_uuid"]].append((record_receipt, record))
    matching_groups = []
    for entries in grouped.values():
        if any(
                record["event"] == "START" and
                record["product_id"] == str(product_id) and
                record["public_source_boot_id"] == public_boot and
                record["public"] is not None and
                record["public"]["sequence"] == int(start_message.sequence)
                for _, record in entries):
            matching_groups.append(entries)
    if len(matching_groups) != 1:
        raise AnalysisError("matching ownership execution is missing or ambiguous")
    records = sorted(matching_groups[0], key=lambda item: item[1]["record_index"])
    chain = [record for _, record in records]
    if ([record["record_index"] for record in chain] != [1, 2, 3, 4] or
            [record["event"] for record in chain] != ["START", "LOADED", "EMPTY", "CLOSE"]):
        raise AnalysisError("ownership record chain is incomplete or out of order")
    start_record, loaded_record, empty_record, close_record = chain
    close_receipt = records[-1][0]
    execution_uuid = start_record["execution_uuid"]
    child_boot = start_record["owned_child_boot_id"]
    if any(record["execution_uuid"] != execution_uuid or
           record["product_id"] != str(product_id) or
           record["public_source_boot_id"] != public_boot or
           record["owned_child_boot_id"] != child_boot
           for record in chain):
        raise AnalysisError("ownership execution/product/boot identity changed")
    if not all((
            _ownership_boundary_matches(start_record, start_message, product_id, "START", child_boot),
            _ownership_boundary_matches(loaded_record, loaded_message, product_id, "LOADED", child_boot),
            _ownership_boundary_matches(empty_record, empty_message, product_id, "EMPTY", child_boot))):
        raise AnalysisError("ownership boundary fingerprint or qualification failed")
    boundary_captures = [record["captured_monotonic_s"] for record in chain]
    if any(right <= left for left, right in zip(boundary_captures, boundary_captures[1:])):
        raise AnalysisError("ownership capture-monotonic chronology is not strictly increasing")
    child_sequences = [record["owned_child_sequence"] for record in chain[:3]]
    public_sequences = [record["public"]["sequence"] for record in chain[:3]]
    if (any(type(value) is not int or value <= 0 for value in child_sequences) or
            not child_sequences[0] < child_sequences[1] < child_sequences[2] or
            not public_sequences[0] < public_sequences[1] < public_sequences[2]):
        raise AnalysisError("ownership boundary sequence order is invalid")
    refs = close_record["first_boundaries"]
    for event, record in zip(("START", "LOADED", "EMPTY"), chain[:3]):
        expected_ref = {
            "record_index": record["record_index"],
            "source_boot_id": public_boot,
            "sequence": record["public"]["sequence"],
        }
        if refs[event] != expected_ref:
            raise AnalysisError("ownership CLOSE does not reference the exact first boundary")
    high_water = close_record["public_sequence_high_water"]
    if (close_record["captured_record_count"] != 4 or
            type(high_water) is not int or high_water < public_sequences[-1] or
            high_water > 0xFFFFFFFFFFFFFFFF or
            close_record["active_owner_match"] is not True or
            close_record["goal_reserved"] is not True or
            close_record["authority_open"] is not False or
            close_record["child_alive"] is not False or
            close_record["callback_child_present"] is not True or
            close_record["child_reference_consistent"] is not True or
            close_record["child_consistent"] is not True or
            close_record["current_product_attached"] is not False or
            close_record["current_product_id"] != "" or
            close_record["current_state"] != 1 or
            close_record["current_base_motion_allowed"] is not True or
            close_record["fault_latched"] is not False or
            close_record["cancel_requested"] is not False or
            close_record["observed_history_violations"] or
            close_record["stage_started"] is not True or
            close_record["stage_loaded_proof"] is not True or
            close_record["terminal_empty_proof"] is not True or
            close_record["child_exit_code"] != 0 or
            close_record["result_product_id"] != str(product_id) or
            close_record["result_outcome"] != 0 or
            close_record["result_delivered"] is not True):
        raise AnalysisError("ownership CLOSE does not prove safe successful cleanup")
    close_child = close_record["child"]
    if close_child is None:
        raise AnalysisError("ownership CLOSE lacks the owned child snapshot")
    if (close_child["source_boot_id"] != child_boot or
            close_child["sequence"] != close_record["owned_child_sequence"] or
            close_record["owned_child_sequence"] < child_sequences[-1] or
            not all((close_child["valid"], close_child["state"] == 1,
                     close_child["base_motion_allowed"],
                     not close_child["product_attached"], close_child["product_id"] == "")) or
            not _message_consistent(SimpleNamespace(**{
                "valid": close_child["valid"], "state": close_child["state"],
                "base_motion_allowed": close_child["base_motion_allowed"],
                "product_attached": close_child["product_attached"],
                "product_id": close_child["product_id"],
                "detail": close_child["detail"]}), product_id)):
        raise AnalysisError("ownership CLOSE child is not the accepted detached EMPTY state")
    close_received = close_record["child_received_monotonic_s"]
    close_age = close_record["child_age_s"]
    close_capture = close_record["captured_monotonic_s"]
    if (close_received is None or close_age is None or
            close_received < 0.0 or close_capture < close_received or
            close_age != close_capture - close_received):
        raise AnalysisError("ownership CLOSE child receipt/age is inconsistent")

    if start_receipt < 0 or close_receipt < start_receipt:
        raise AnalysisError("conservative ownership recorder fence is missing or reversed")
    success_receipt, upper_receipt = _check_action_evidence(
        action_status_samples, action_feedback_samples, start_receipt, empty_receipt,
        close_receipt, product_id, execution_uuid)
    if upper_receipt < start_receipt:
        raise AnalysisError("conservative ownership recorder fence is reversed")

    # Close's allocated sequence high-water must be complete, even for a row
    # whose recorder receipt arrived after the conservative fence.
    hwm_rows = []
    for receipt_ns, message in by_boot[public_boot]:
        try:
            sequence = message.sequence
            if type(sequence) is not int:
                raise TypeError("public high-water sequence is not an integer")
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise AnalysisError("public high-water status sequence is malformed") from error
        if int(start_message.sequence) <= sequence <= high_water:
            hwm_rows.append((receipt_ns, sequence, message))
    hwm_by_sequence = defaultdict(list)
    start_sequence = int(start_message.sequence)
    if high_water - start_sequence + 1 > len(by_boot[public_boot]):
        raise AnalysisError("public status rows cannot cover CLOSE high-water")
    previous_receipt = None
    previous_sequence = None
    for receipt_ns, sequence, message in hwm_rows:
        if previous_receipt is not None and receipt_ns < previous_receipt:
            raise AnalysisError("public high-water receipt time rolled back")
        if previous_sequence is not None and sequence <= previous_sequence:
            raise AnalysisError("public high-water sequence is not increasing")
        previous_receipt, previous_sequence = receipt_ns, sequence
        hwm_by_sequence[sequence].append((receipt_ns, message))
        if not _message_consistent(message, product_id):
            raise AnalysisError("public high-water rows contain invalid/fault/wrong-product evidence")
        if sequence > public_sequences[2] and (
                _recognizable_stage_claim(message) or int(message.state) != 1 or
                bool(message.product_attached)):
            raise AnalysisError("public high-water rows contain post-EMPTY stage activity")
    expected_sequences = range(start_sequence, high_water + 1)
    if any(len(hwm_by_sequence.get(sequence, ())) != 1 for sequence in expected_sequences):
        raise AnalysisError("public status rows do not uniquely cover CLOSE high-water")

    # Audit all public rows conservatively through CLOSE/first SUCCESS, plus
    # the full allocated HWM range above. The two time bases stay separate.
    previous_by_boot = {}
    for receipt_ns, message in status_samples:
        if not start_receipt <= receipt_ns <= upper_receipt:
            continue
        try:
            boot_id = int(message.source_boot_id)
            sequence = message.sequence
            if type(sequence) is not int:
                raise TypeError("public sequence is not an integer")
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise AnalysisError("public status in the conservative fence is malformed") from error
        if boot_id <= 0 or sequence <= 0:
            raise AnalysisError("public status in the conservative fence has invalid identity")
        prior = previous_by_boot.get(boot_id)
        if prior is not None and (receipt_ns < prior[0] or sequence <= prior[1]):
            raise AnalysisError("public status identity/receipt rolled back in the conservative fence")
        previous_by_boot[boot_id] = (receipt_ns, sequence)
        if not _message_consistent(message, product_id):
            raise AnalysisError("public status in the conservative fence is invalid or contradictory")
        if boot_id != public_boot and _recognizable_stage_claim(message):
            raise AnalysisError("foreign public stage claim is inside the conservative fence")
        if boot_id == public_boot and sequence > public_sequences[2] and (
                _recognizable_stage_claim(message) or int(message.state) != 1 or
                bool(message.product_attached)):
            raise AnalysisError("later public activity is inside the conservative fence")

    for record_receipt, record in parsed_records:
        if (start_receipt <= record_receipt <= upper_receipt and
                record["execution_uuid"] != execution_uuid):
            raise AnalysisError("another ownership epoch is inside the conservative fence")
    _check_internal_status_span(
        internal_samples, start_receipt, upper_receipt, product_id, child_boot)
    return public_boot, [
        (receipt_ns * 1e-9, message) for receipt_ns, message in span_rows
    ]


def command_trace_matches(
        control: Sequence[Tuple[float, float, float]],
        simulation: Sequence[Tuple[float, float, float]],
        max_age: float = COMMAND_FORWARDING_MAX_AGE_SECONDS) -> bool:
    """Prove each forwarded command came from a recent arbitration sample.

    The base adapter republishes its cached arbitration command on an
    independent 50 ms timer.  A simulator-facing sample can therefore trail
    the newest arbitration sample by one timer period.  Requiring an exact
    match to any control sample at or before the output, within the existing
    250 ms freshness bound, preserves the command-ownership check without
    rejecting that deterministic forwarding latency.
    """
    if not control or not simulation or max_age < 0.0:
        return False
    ordered_control = sorted(control)
    ordered_simulation = sorted(simulation)
    control_index = 0
    for timestamp, linear, angular in ordered_simulation:
        while (control_index + 1 < len(ordered_control) and
               ordered_control[control_index + 1][0] <= timestamp):
            control_index += 1
        reference_index = control_index
        matched = False
        while reference_index >= 0:
            reference = ordered_control[reference_index]
            age = timestamp - reference[0]
            if age > max_age:
                break
            if age >= 0.0 and math.isclose(
                    reference[1], linear, rel_tol=0.0, abs_tol=1e-6) and math.isclose(
                    reference[2], angular, rel_tol=0.0, abs_tol=1e-6):
                matched = True
                break
            reference_index -= 1
        if not matched:
            return False
    return True


def analyze(bag: Path, product_id: int) -> List[str]:
    model, mass_kg, slot = _load_product_registry(product_id)
    selected_state_topic = f"/amr/simulation/internal/attachment/product_{product_id}/state"
    selected_pose_topic = f"/model/{model}/pose"

    reader = _open_reader(bag)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    counts: Dict[str, int] = defaultdict(int)
    status_samples = []
    raw_status_samples = []
    raw_internal_status_samples = []
    execute_cycle_status_samples = []
    execute_cycle_feedback_samples = []
    bootstrap_samples = []
    attachment_samples = []
    pose_samples = []
    robot_pose_samples = []
    contacts = {"left": [], "right": []}
    commands = {CONTROL_TOPIC: [], SIMULATION_TOPIC: []}
    normal_nav_status_samples = []
    rosout_markers = []
    rosout_record_samples = []
    decode_failures = []
    optional_decode_failures = []
    optional_topics = {
        INTERNAL_STATUS_TOPIC, EXECUTE_CYCLE_STATUS_TOPIC, EXECUTE_CYCLE_FEEDBACK_TOPIC,
    }
    selected_topics = {
        "/amr/manipulation/status", BOOTSTRAP_TOPIC, selected_state_topic,
        selected_pose_topic, "/amr/simulation/ground_truth/pose",
        "/amr/simulation/contacts/left_finger", "/amr/simulation/contacts/right_finger",
        CONTROL_TOPIC, SIMULATION_TOPIC, NORMAL_NAV_STATUS_TOPIC, "/rosout",
        INTERNAL_STATUS_TOPIC, EXECUTE_CYCLE_STATUS_TOPIC, EXECUTE_CYCLE_FEEDBACK_TOPIC,
    }

    while reader.has_next():
        topic, payload, bag_timestamp = reader.read_next()
        counts[topic] += 1
        if topic not in selected_topics:
            continue
        message_type = topic_types.get(topic)
        if message_type is None:
            target = optional_decode_failures if topic in optional_topics else decode_failures
            target.append(f"{topic}: unknown type")
            continue
        try:
            message = deserialize_message(payload, get_message(message_type))
        except Exception as error:  # noqa: BLE001 - a corrupt evidence stream fails closed
            target = optional_decode_failures if topic in optional_topics else decode_failures
            target.append(f"{topic}: {error}")
            continue
        timestamp = float(bag_timestamp) * 1e-9
        receipt_ns = bag_timestamp if type(bag_timestamp) is int else None
        if topic == "/amr/manipulation/status":
            status_samples.append((timestamp, message))
            raw_status_samples.append((receipt_ns, message))
        elif topic == INTERNAL_STATUS_TOPIC:
            raw_internal_status_samples.append((receipt_ns, message))
        elif topic == EXECUTE_CYCLE_STATUS_TOPIC:
            execute_cycle_status_samples.append((receipt_ns, message))
        elif topic == EXECUTE_CYCLE_FEEDBACK_TOPIC:
            execute_cycle_feedback_samples.append((receipt_ns, message))
        elif topic == BOOTSTRAP_TOPIC:
            bootstrap_samples.append((timestamp, message.data))
        elif topic == selected_state_topic:
            attachment_samples.append((timestamp, message.data))
        elif topic == selected_pose_topic:
            pose_samples.append((timestamp, message))
        elif topic == "/amr/simulation/ground_truth/pose":
            robot_pose_samples.append((timestamp, message))
        elif topic == "/amr/simulation/contacts/left_finger":
            contacts["left"].append((timestamp, _contact_has_model(message, model)))
        elif topic == "/amr/simulation/contacts/right_finger":
            contacts["right"].append((timestamp, _contact_has_model(message, model)))
        elif topic in commands:
            try:
                commands[topic].append((timestamp, *_command_values(message)))
            except (AttributeError, TypeError, ValueError):
                decode_failures.append(f"{topic}: malformed command")
        elif topic == NORMAL_NAV_STATUS_TOPIC:
            normal_nav_status_samples.append((timestamp, message))
        elif topic == "/rosout":
            rosout_markers.append(str(getattr(message, "msg", "")))
            rosout_record_samples.append((
                receipt_ns, str(getattr(message, "name", "")),
                str(getattr(message, "msg", ""))))

    failures: List[str] = []
    if decode_failures:
        failures.append("message decode failure")
    missing = [topic for topic in REQUIRED_TOPIC_SUFFIXES if counts[topic] == 0]
    if counts[selected_state_topic] == 0:
        missing.append(selected_state_topic)
    if counts[selected_pose_topic] == 0:
        missing.append(selected_pose_topic)
    if missing:
        failures.append("missing required topics: " + ", ".join(sorted(set(missing))))

    selected_stage = select_stage_status_stream(status_samples, product_id)
    corroboration_error = None
    if selected_stage is None:
        try:
            if optional_decode_failures:
                raise AnalysisError("optional ownership/action evidence failed to decode")
            selected_stage = select_corroborated_stage_status_stream(
                raw_status_samples, raw_internal_status_samples,
                execute_cycle_status_samples, execute_cycle_feedback_samples,
                rosout_record_samples, product_id)
        except AnalysisError as error:
            corroboration_error = str(error)
    if selected_stage is None:
        failures.append("mass-stage source_boot_id evidence is missing or ambiguous")
        if corroboration_error:
            failures.append("ownership corroboration rejected: " + corroboration_error)
        stage_boot_id = 0
        stage_start = 0.0
        stage_end = 0.0
        stage_statuses = []
    else:
        stage_boot_id, stage_statuses = selected_stage
        stage_start = stage_statuses[0][0]
        stage_end = stage_statuses[-1][0]
        scoped_statuses = [
            (timestamp, message) for timestamp, message in stage_statuses
            if stage_start <= timestamp <= stage_end
        ]
        if any(int(message.sequence) == 0 for _, message in scoped_statuses):
            failures.append("mass-stage status sequence is invalid")
        if any(not bool(message.valid) and int(message.state) != 5 for _, message in scoped_statuses):
            failures.append("mass-stage status became invalid before terminal state")

    normal_nav_active = any(
        stage_start <= timestamp <= stage_end and any(
            int(item.status) in {2, 3, 4, 5, 6} for item in message.status_list
        ) for timestamp, message in normal_nav_status_samples
    )
    if not any(data.startswith("READY") and timestamp <= stage_start
               for timestamp, data in bootstrap_samples):
        failures.append("READY attachment bootstrap status was not recorded before the stage")
    if not normal_nav_active:
        failures.append("normal navigation action status did not show an active goal")

    scoped_attachments = [
        state for timestamp, state in attachment_samples
        if stage_start <= timestamp <= stage_end
    ]
    # Bootstrap READY is the authoritative initial detached proof.  The stage
    # itself must then prove the native attach and detach transitions.
    expected_states = ("attached", "detached")
    state_index = 0
    for state in scoped_attachments:
        if state == expected_states[state_index]:
            state_index += 1
            if state_index == len(expected_states):
                break
    if state_index != len(expected_states):
        failures.append("selected product did not prove detached -> attached -> detached in stage")

    scoped_statuses = [
        (timestamp, message) for timestamp, message in status_samples
        if stage_boot_id and int(message.source_boot_id) == stage_boot_id and
        stage_start <= timestamp <= stage_end
    ]
    if not any(int(message.state) == 2 and bool(message.product_attached) and
               message.product_id == str(product_id) for _, message in scoped_statuses):
        failures.append("retained loaded status was not recorded")
    final_empty = [
        message for _, message in scoped_statuses
        if int(message.state) == 1 and bool(message.valid) and not bool(message.product_attached)
    ]
    if not final_empty:
        failures.append("valid empty-stowed final status was not recorded")

    for side in ("left", "right"):
        if not any(is_contact for timestamp, is_contact in contacts[side]
                   if stage_start <= timestamp <= stage_end):
            failures.append(f"bilateral {side} product contact was not recorded")

    scoped_poses = [
        (timestamp, message) for timestamp, message in pose_samples
        if stage_start <= timestamp <= stage_end
    ]
    if not scoped_poses:
        failures.append("selected product pose was not recorded in stage")
    else:
        final_pose = scoped_poses[-1][1].pose.position
        try:
            slot_error = math.sqrt(sum((float(value) - float(target)) ** 2 for value, target in (
                (final_pose.x, slot[0]), (final_pose.y, slot[1]), (final_pose.z, slot[2]))))
        except (TypeError, ValueError):
            slot_error = math.inf
        if not math.isfinite(slot_error) or slot_error > 0.030:
            failures.append(f"final slot error exceeded 0.030 m: {slot_error:.6f}")

    control = sorted(
        row for row in commands[CONTROL_TOPIC]
        if stage_start <= row[0] <= stage_end
    )
    simulation = sorted(
        row for row in commands[SIMULATION_TOPIC]
        if stage_start <= row[0] <= stage_end
    )
    simulation_nonzero = [row for row in simulation if abs(row[1]) > 1e-12 or abs(row[2]) > 1e-12]
    if not control or not simulation_nonzero:
        failures.append("base command evidence was empty")
    elif not command_trace_matches(control, simulation_nonzero):
        failures.append("simulation base command did not match command arbitration")

    # Check both command authority and measured base pose while the stage says
    # base motion is forbidden.  Nav/status samples are all bag-time ordered.
    robot_samples = sorted(
        (timestamp, pose) for timestamp, pose in robot_pose_samples
        if stage_start <= timestamp <= stage_end
    )
    previous_pose = None
    previous_forbidden = False
    for timestamp, pose in robot_samples:
        active = [message for status_time, message in scoped_statuses if status_time <= timestamp]
        if not active:
            continue
        status = active[-1]
        forbidden = not bool(status.base_motion_allowed)
        if forbidden and previous_forbidden and previous_pose is not None:
            dx = pose.pose.position.x - previous_pose.pose.position.x
            dy = pose.pose.position.y - previous_pose.pose.position.y
            dz = pose.pose.position.z - previous_pose.pose.position.z
            displacement = math.sqrt(dx * dx + dy * dy + dz * dz)
            if not math.isfinite(displacement) or displacement > 1e-4:
                failures.append("base moved while motion was forbidden")
                break
        previous_pose = pose
        previous_forbidden = forbidden
    for timestamp, linear, angular in control:
        active = [message for status_time, message in scoped_statuses if status_time <= timestamp]
        if active and not bool(active[-1].base_motion_allowed) and \
                (abs(linear) > 1e-12 or abs(angular) > 1e-12):
            failures.append("nonzero base command was present while motion was forbidden")
            break

    # The stage emits these markers for planning-scene and exact lower-path
    # proof.  If rosout is recorded, require both markers; otherwise report the
    # evidence as an external log check instead of silently treating silence as
    # a pass.
    if rosout_markers:
        if not any("planning-scene attached object proof" in marker and "PASS" in marker
                   for marker in rosout_markers):
            failures.append("planning-scene attached-object proof marker is missing")
        if not any("placement lower trajectory postconditions" in marker and "PASS" in marker
                   for marker in rosout_markers):
            failures.append("placement lower trajectory proof marker is missing")

    diagnostics = [
        f"model={model}", f"mass_kg={mass_kg:.3f}", f"stage_source_boot_id={stage_boot_id}",
        f"normal_navigation_active={normal_nav_active}",
        f"precise_navigation_messages={counts[PRECISE_NAV_STATUS_TOPIC]}",
        f"control_commands={len(control)}", f"simulation_commands={len(simulation)}",
    ]
    return failures + ["diagnostic " + item for item in diagnostics]


def _write_result(path: Path, product_id: int, passed: bool) -> None:
    line = f"GATE6_BAG_ANALYSIS={'PASS' if passed else 'FAIL'} product_id={product_id}\n"
    path.write_text(line, encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 directory")
    parser.add_argument("--product-id", required=True, type=int, choices=PRODUCT_IDS)
    parser.add_argument("--output", required=True, type=Path, help="analysis output text file")
    args = parser.parse_args(argv)
    try:
        diagnostics = analyze(args.bag, args.product_id)
        failures = [item for item in diagnostics if not item.startswith("diagnostic ")]
    except AnalysisError as error:
        failures = [str(error)]
        diagnostics = failures
    passed = not failures
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        _write_result(args.output, args.product_id, passed)
    except OSError as error:
        print(f"GATE6_BAG_ANALYSIS=FAIL product_id={args.product_id}", file=sys.stderr)
        print(f"could not write analysis output: {error}", file=sys.stderr)
        return 2
    if passed:
        print(f"GATE6_BAG_ANALYSIS=PASS product_id={args.product_id}")
        for item in diagnostics:
            print(item, file=sys.stderr)
        return 0
    print(f"GATE6_BAG_ANALYSIS=FAIL product_id={args.product_id}", file=sys.stderr)
    for item in diagnostics:
        print(item, file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
