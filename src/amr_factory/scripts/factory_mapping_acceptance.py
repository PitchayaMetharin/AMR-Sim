#!/usr/bin/env python3
"""Observation-only factory mapping acceptance and promotion receipts.

The live observer subscribes to state and introspects the graph.  It never
starts exploration, publishes a command, launches a process, or calls a
motion service.  The evaluator is intentionally ROS-free so the safety and
freshness gates can be exercised with deterministic fake snapshots.
"""

from __future__ import annotations

import argparse
from collections import Counter
import math
import os
from numbers import Real
from pathlib import Path
import sys
import time
from typing import Any, Callable, Iterable, Mapping, Optional

import yaml

import factory_mapping_artifacts as artifacts


RUNTIME_ACCEPTANCE_NAME = "runtime_acceptance.yaml"
QUALITY_REVIEW_NAME = "quality_review.yaml"
MAPPING_ACCEPTANCE_NAME = "mapping_acceptance.yaml"
PROMOTION_RECEIPT_NAME = "promotion_eligibility.yaml"
ACCEPTANCE_SCHEMA_VERSION = 1
QUALITY_REVIEW_SCHEMA_VERSION = 2
ACCEPTANCE_TIMEOUT_SECONDS = 60.0
MAP_MAX_AGE_SECONDS = 3.0
TF_MAX_AGE_SECONDS = 1.0
STATUS_MAX_AGE_SECONDS = 1.0
TF_OWNERSHIP_TOPIC = "/amr/factory/tf_ownership"
TF_OWNERSHIP_EDGES = ("map->odom", "odom->base_footprint")
TF_OWNERSHIP_SOURCE = "/amr/tf_ownership_observer"

_ACCEPTANCE_RECEIPT_BASE_FIELDS = frozenset({
    "schema", "kind", "state", "created_unix", "session_path",
    "candidate_name", "candidate_path", "candidate_sha256",
    "artifact_bundle_sha256", "conditions", "artifact_manifest",
    "runtime_acceptance", "quality_review",
})
_ACCEPTANCE_CONDITIONS = {
    "artifact_manifest": "VALIDATED",
    "runtime_acceptance": "PASS",
    "quality_review": "ACCEPTED",
}
_ACCEPTANCE_ENTRY_FIELDS = {
    "artifact_manifest": frozenset({"path", "sha256"}),
    "runtime_acceptance": frozenset({"path", "sha256", "mode"}),
    "quality_review": frozenset({"path", "sha256", "reviewer"}),
}

COMMON_REQUIRED_NODES = frozenset({
    "/amr/slam_toolbox",
    "/amr/ekf_filter_node",
    "/amr/command_arbitration_node",
    "/amr/base_adapter_node",
    "/amr/front_lidar_adapter_node",
    "/amr/rear_lidar_adapter_node",
    "/amr/imu_adapter_node",
    "/amr/product_camera_adapter_node",
    "/amr/manipulation_supervisor_node",
})
MANUAL_FORBIDDEN_NODES = frozenset({
    "/amr/planner_server",
    "/amr/smoother_server",
    "/amr/controller_server",
    "/amr/mission_supervisor_node",
    "/amr/frontier_explorer",
})
AUTONOMOUS_REQUIRED_NODES = frozenset({
    "/amr/planner_server",
    "/amr/smoother_server",
    "/amr/controller_server",
    "/amr/mission_supervisor_node",
    "/amr/frontier_explorer",
})
AUTONOMOUS_TERMINAL_STATES = frozenset({"STOPPED", "COMPLETE", "INCOMPLETE"})
FORBIDDEN_NODES = frozenset({"/amr/amcl", "/amr/map_server"})

SLAM_OWNERS = frozenset({
    "/amr/slam_toolbox",
    "amr/slam_toolbox",
    "/slam_toolbox/async_slam_toolbox_node",
    "slam_toolbox/async_slam_toolbox_node",
})
EKF_OWNERS = frozenset({
    "/amr/ekf_filter_node",
    "amr/ekf_filter_node",
    "/robot_localization/ekf_filter_node",
    "robot_localization/ekf_filter_node",
})
ARBITRATION_OWNERS = frozenset({
    "/amr/command_arbitration_node",
    "amr/command_arbitration_node",
    "/amr_control/command_arbitration_node",
    "amr_control/command_arbitration_node",
})
CONTROLLER_OWNERS = frozenset({
    "/amr/controller_server",
    "amr/controller_server",
    "/nav2_controller/controller_server",
    "nav2_controller/controller_server",
})
TELEOP_OWNERS = frozenset({
    "/amr/prototype_teleop",
    "amr/prototype_teleop",
    "prototype_teleop",
})


class AcceptanceError(ValueError):
    """Raised when an acceptance proof is absent, stale, or inconsistent."""


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise AcceptanceError(f"{label} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise AcceptanceError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise AcceptanceError(f"{label} must be finite")
    return result


def _finite_real(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise AcceptanceError(f"{label} must be a numeric real")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise AcceptanceError(f"{label} must be a numeric real") from exc
    if not math.isfinite(result):
        raise AcceptanceError(f"{label} must be finite")
    return result


def _fresh(receipt_at: Any, max_age: float, now: float) -> bool:
    if isinstance(receipt_at, bool) or isinstance(now, bool):
        return False
    try:
        receipt = float(receipt_at)
        current = float(now)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(receipt) or not math.isfinite(current):
        return False
    age = current - receipt
    return 0.0 <= age <= max_age


def _full_node_name(name: Any, namespace: Any = "") -> str:
    text = str(name or "").strip()
    namespace_text = str(namespace or "").strip().rstrip("/")
    if text.startswith("/"):
        return "/" + "/".join(part for part in text.split("/") if part)
    if namespace_text:
        return "/" + "/".join(
            part for part in (namespace_text.strip("/") + "/" + text).split("/")
            if part)
    return "/" + text if text else ""


def _normalize_node_names(raw: Any) -> list[str]:
    names = []
    if raw is None or isinstance(raw, (str, bytes)):
        return names
    try:
        values = iter(raw)
    except TypeError:
        return names
    for value in values:
        if isinstance(value, (tuple, list)) and len(value) == 2:
            name = _full_node_name(value[0], value[1])
        else:
            name = _full_node_name(value)
        if name:
            names.append(name)
    return names


def _endpoint_name(endpoint: Any) -> str:
    if isinstance(endpoint, str):
        return _full_node_name(endpoint)
    if isinstance(endpoint, Mapping):
        return _full_node_name(endpoint.get("node_name"), endpoint.get("node_namespace"))
    return _full_node_name(
        getattr(endpoint, "node_name", ""),
        getattr(endpoint, "node_namespace", ""),
    )


def _publishers(snapshot: Mapping[str, Any], topic: str) -> list[str]:
    raw = snapshot.get("topic_publishers", {})
    if not isinstance(raw, Mapping):
        return []
    values = raw.get(topic, ())
    if not isinstance(values, (list, tuple)):
        return []
    return [_endpoint_name(value) for value in values]


def _owner_matches(owner: str, allowed: Iterable[str]) -> bool:
    if not isinstance(owner, str):
        return False
    normalized = owner.strip()
    return normalized in allowed


def _value(source: Any, key: str, default: Any = None) -> Any:
    if isinstance(source, Mapping):
        return source.get(key, default)
    return getattr(source, key, default)


def _map_fields(message: Any) -> Optional[dict[str, Any]]:
    if message is None:
        return None
    header = _value(message, "header", {})
    info = _value(message, "info", message)
    frame_id = _value(header, "frame_id", _value(message, "frame_id", ""))
    width_value = _value(info, "width")
    height_value = _value(info, "height")
    resolution_value = _value(info, "resolution")
    data = _value(message, "data")
    if (not isinstance(frame_id, str) or width_value is None or
            height_value is None or resolution_value is None or data is None):
        return None
    try:
        if isinstance(width_value, bool) or isinstance(height_value, bool):
            return None
        width = int(width_value)
        height = int(height_value)
        resolution = float(resolution_value)
        values = list(data)
    except (TypeError, ValueError):
        return None
    if (isinstance(width_value, float) and not width_value.is_integer()) or \
            (isinstance(height_value, float) and not height_value.is_integer()):
        return None
    if (width <= 0 or height <= 0 or not math.isfinite(resolution) or
            not math.isclose(resolution, artifacts.MAP_RESOLUTION,
                             rel_tol=0.0, abs_tol=1e-9) or
            len(values) != width * height):
        return None
    normalized_values = []
    for value in values:
        if (not isinstance(value, (int, float)) or isinstance(value, bool)):
            return None
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(numeric) or not numeric.is_integer():
            return None
        integer = int(numeric)
        if integer != -1 and not 0 <= integer <= 100:
            return None
        normalized_values.append(integer)
    origin = _value(info, "origin")
    position = _value(origin, "position")
    orientation = _value(origin, "orientation")
    if origin is None or position is None or orientation is None:
        return None
    origin_values = [
        _value(position, "x"), _value(position, "y"),
        _value(position, "z"), _value(orientation, "x"),
        _value(orientation, "y"), _value(orientation, "z"),
        _value(orientation, "w"),
    ]
    if any(value is None or isinstance(value, bool) for value in origin_values):
        return None
    try:
        origin_values = [float(value) for value in origin_values]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in origin_values):
        return None
    normalized_origin = {
        "position": {
            "x": origin_values[0], "y": origin_values[1],
            "z": origin_values[2],
        },
        "orientation": {
            "x": origin_values[3], "y": origin_values[4],
            "z": origin_values[5], "w": origin_values[6],
        },
    }
    return {
        "frame_id": frame_id,
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": normalized_origin,
        "data": normalized_values,
        "known": sum(value != -1 for value in normalized_values),
        "free": sum(value == 0 for value in normalized_values),
        "occupied": sum(1 <= value <= 100 for value in normalized_values),
        "unknown": sum(value == -1 for value in normalized_values),
    }


def _diagnostic_values(status: Any) -> dict[str, Any]:
    if isinstance(status, Mapping):
        return dict(status)
    values = {}
    for item in _value(status, "values", ()) or ():
        key = _value(item, "key")
        if key:
            values[str(key)] = _value(item, "value")
    return values


def _bool_value(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes"}:
            return True
        if lowered in {"false", "0", "no"}:
            return False
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    return None


def _map_counts(fields: Optional[Mapping[str, Any]]) -> dict[str, int]:
    if not fields:
        return {"width": 0, "height": 0, "known": 0, "free": 0,
                "occupied": 0, "unknown": 0}
    return {
        key: int(fields[key])
        for key in ("width", "height", "known", "free", "occupied", "unknown")
    }


def _preflight_pass(value: Any) -> bool:
    if isinstance(value, Mapping):
        value = value.get("verdict")
    return type(value) is str and value == "PASS"


def _common_node_gate(node_names: Iterable[str]) -> tuple[bool, dict[str, Any]]:
    names = list(node_names)
    nodes = set(names)
    counts = Counter(names)
    missing = sorted(COMMON_REQUIRED_NODES - nodes)
    duplicates = sorted(
        node for node in COMMON_REQUIRED_NODES if counts[node] > 1)
    observed = {
        "nodes": names,
        "missing": missing,
        "duplicates": duplicates,
    }
    return not missing and not duplicates, observed


def _production_localization_gate(
        node_names: Iterable[str]) -> tuple[bool, dict[str, Any]]:
    names = list(node_names)
    forbidden = sorted(FORBIDDEN_NODES & set(names))
    return not forbidden, {"nodes": names, "forbidden": forbidden}


def _persisted_node_names(observed: Any, label: str) -> list[str]:
    if not isinstance(observed, Mapping):
        raise AcceptanceError(f"runtime acceptance {label} evidence is invalid")
    names = observed.get("nodes")
    if (not isinstance(names, list) or
            any(not isinstance(name, str) or
                name != _full_node_name(name) or not name
                for name in names)):
        raise AcceptanceError(f"runtime acceptance {label} node evidence is invalid")
    if _normalize_node_names(names) != names:
        raise AcceptanceError(f"runtime acceptance {label} node evidence is invalid")
    return list(names)


def _publisher_names(raw: Any) -> Optional[list[str]]:
    if not isinstance(raw, (list, tuple)):
        return None
    return [_endpoint_name(value) for value in raw]


def _persisted_publisher_names(value: Any, label: str) -> list[str]:
    if (not isinstance(value, list) or
            any(not isinstance(owner, str) or
                owner != _full_node_name(owner) or not owner
                for owner in value)):
        raise AcceptanceError(
            f"runtime acceptance {label} publisher evidence is invalid")
    return list(value)


def _map_control_ownership_gate(
        map_publishers: Any, control_publishers: Any) -> bool:
    return (
        isinstance(map_publishers, list) and
        isinstance(control_publishers, list) and
        len(map_publishers) == 1 and
        _owner_matches(map_publishers[0], SLAM_OWNERS) and
        len(control_publishers) == 1 and
        _owner_matches(control_publishers[0], ARBITRATION_OWNERS))


def _tf_ownership_gate(tf_owners: Any) -> bool:
    if not isinstance(tf_owners, Mapping):
        return False
    map_odom_owners = tf_owners.get("map->odom")
    odom_base_owners = tf_owners.get("odom->base_footprint")
    return (
        isinstance(map_odom_owners, list) and
        isinstance(odom_base_owners, list) and
        len(map_odom_owners) == 1 and
        _owner_matches(map_odom_owners[0], SLAM_OWNERS) and
        len(odom_base_owners) == 1 and
        _owner_matches(odom_base_owners[0], EKF_OWNERS))


def _tf_ownership_source_gate(publishers: Any) -> bool:
    return (
        isinstance(publishers, list) and
        len(publishers) == 1 and
        _owner_matches(publishers[0], {TF_OWNERSHIP_SOURCE}))


def _parse_tf_ownership_message(raw: Any) -> Optional[dict[str, list[str]]]:
    """Parse the C++ observer's per-message publisher evidence strictly."""
    if not isinstance(raw, str):
        return None
    try:
        payload = yaml.safe_load(raw)
    except yaml.YAMLError:
        return None
    if not isinstance(payload, Mapping):
        return None
    if (type(payload.get("schema")) is not int or
            payload.get("schema") != 1 or
            payload.get("kind") != "factory_tf_ownership"):
        return None
    try:
        observed = float(payload.get("observed_monotonic"))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(observed):
        return None
    edges = payload.get("edges")
    if not isinstance(edges, Mapping) or set(edges) != set(TF_OWNERSHIP_EDGES):
        return None
    parsed: dict[str, list[str]] = {}
    for edge in TF_OWNERSHIP_EDGES:
        evidence = edges.get(edge)
        if not isinstance(evidence, Mapping):
            return None
        owners = evidence.get("owners")
        if not isinstance(owners, list):
            return None
        if any(not isinstance(owner, str) or not owner.strip() for owner in owners):
            return None
        if len(set(owners)) != len(owners):
            return None
        parsed[edge] = list(owners)
    return parsed


def _tf_ownership_observed(raw: Any) -> Any:
    if not isinstance(raw, Mapping):
        return "missing or malformed tf_publishers"
    observed = {}
    for edge in ("map->odom", "odom->base_footprint"):
        owners = _publisher_names(raw.get(edge))
        observed[edge] = owners if owners is not None else raw.get(edge)
    return observed


def _persisted_tf_ownership(observed: Any) -> dict[str, list[str]]:
    if (not isinstance(observed, Mapping) or
            set(observed) != {"map->odom", "odom->base_footprint"}):
        raise AcceptanceError("runtime acceptance TF ownership evidence is invalid")
    return {
        "map->odom": _persisted_publisher_names(
            observed.get("map->odom"), "TF map->odom"),
        "odom->base_footprint": _persisted_publisher_names(
            observed.get("odom->base_footprint"),
            "TF odom->base_footprint"),
    }


def _map_gate(
        map_fields: Optional[Mapping[str, Any]],
        received_at: Any,
        now: float) -> bool:
    return (
        map_fields is not None and map_fields.get("frame_id") == "map" and
        map_fields.get("free", 0) > 0 and
        _fresh(received_at, MAP_MAX_AGE_SECONDS, now))


def _persisted_map_evidence(observed: Any) -> tuple[Any, dict[str, Any]]:
    if (not isinstance(observed, Mapping) or
            "received_at" not in observed or
            observed.get("age_limit") != MAP_MAX_AGE_SECONDS or
            not isinstance(observed.get("map"), Mapping)):
        raise AcceptanceError("runtime acceptance map evidence is invalid")
    raw_map = observed["map"]
    map_fields = _map_fields(raw_map)
    required_fields = {
        "frame_id", "width", "height", "resolution", "origin", "data",
        "known", "free", "occupied", "unknown",
    }
    if (map_fields is None or set(raw_map) < required_fields or
            any(raw_map.get(key) != map_fields[key] for key in required_fields)):
        raise AcceptanceError("runtime acceptance map evidence is invalid")
    return observed["received_at"], map_fields


def _localization_tf_gate(observed: Any, now: float) -> bool:
    return (
        isinstance(observed, Mapping) and
        _fresh(observed.get("map_to_odom"), TF_MAX_AGE_SECONDS, now) and
        _fresh(observed.get("odom_to_base_footprint"), TF_MAX_AGE_SECONDS, now))


def _manipulator_observed(status: Any, received_at: Any) -> dict[str, Any]:
    return {
        "valid": _value(status, "valid"),
        "state": _value(status, "state"),
        "product_attached": _value(status, "product_attached"),
        "base_motion_allowed": _value(status, "base_motion_allowed"),
        "received_at": received_at,
    }


def _manipulator_gate(observed: Any, now: float) -> bool:
    if not isinstance(observed, Mapping):
        return False
    status_valid = _bool_value(observed.get("valid")) is True
    state = observed.get("state")
    state_ok = type(state) is int and state == 1
    attached = _bool_value(observed.get("product_attached"))
    motion_allowed = _bool_value(observed.get("base_motion_allowed"))
    return (
        status_valid and state_ok and attached is False and
        motion_allowed is True and
        _fresh(observed.get("received_at"), STATUS_MAX_AGE_SECONDS, now))


def _preflight_gate(observed: Any) -> bool:
    return (
        isinstance(observed, Mapping) and
        _preflight_pass(observed.get("host")) and
        _preflight_pass(observed.get("runtime")))


def _positive_integral(value: Any) -> Optional[int]:
    if isinstance(value, bool):
        return None
    try:
        if isinstance(value, float):
            if not math.isfinite(value) or not value.is_integer():
                return None
        result = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if result > 0 else None


def _exploration_gate(
        exploration: Any,
        received_at: Any,
        outcome: Any,
        now: float) -> bool:
    if not isinstance(exploration, Mapping):
        return False
    state = exploration.get("state")
    run_generation = _positive_integral(exploration.get("run_generation", 0))
    return (
        bool(exploration) and isinstance(state, str) and
        state in AUTONOMOUS_TERMINAL_STATES and
        isinstance(outcome, str) and outcome == state and
        outcome in AUTONOMOUS_TERMINAL_STATES and
        _bool_value(exploration.get("fault_latched")) is False and
        _bool_value(exploration.get("active")) is False and
        _bool_value(exploration.get("pending")) is False and
        run_generation is not None and exploration.get("cancel_target") == "" and
        _fresh(received_at, STATUS_MAX_AGE_SECONDS, now))


def _manual_mode_gate(node_names: Iterable[str], mpc_publishers: Any) -> bool:
    manual_missing = MANUAL_FORBIDDEN_NODES & set(node_names)
    return (
        not manual_missing and isinstance(mpc_publishers, list) and
        (not mpc_publishers or (
            len(mpc_publishers) == 1 and
            _owner_matches(mpc_publishers[0], TELEOP_OWNERS))))


def _autonomous_mode_gate(
        node_names: Iterable[str],
        mpc_publishers: Any,
        exploration: Any,
        exploration_received_at: Any,
        outcome: Any,
        now: float) -> bool:
    missing_autonomous = AUTONOMOUS_REQUIRED_NODES - set(node_names)
    prototype_present = (
        isinstance(mpc_publishers, list) and
        any(_owner_matches(owner, TELEOP_OWNERS) for owner in mpc_publishers))
    controller_only = (
        isinstance(mpc_publishers, list) and
        len(mpc_publishers) == 1 and
        _owner_matches(mpc_publishers[0], CONTROLLER_OWNERS))
    return (
        not missing_autonomous and not prototype_present and controller_only and
        _exploration_gate(
            exploration, exploration_received_at, outcome, now))


def _check(
        checks: dict[str, Any],
        name: str,
        passed: bool,
        observed: Any,
        expected: Any) -> None:
    checks[name] = {
        "passed": bool(passed),
        "observed": observed,
        "expected": expected,
    }


def evaluate_snapshot(
        snapshot: Mapping[str, Any],
        mode: str,
        *,
        now: Optional[float] = None) -> dict[str, Any]:
    """Evaluate one graph snapshot without starting or commanding anything."""
    mode = str(mode).strip().lower()
    current = _finite(
        snapshot.get("now", time.monotonic() if now is None else now), "observer clock")
    node_names = _normalize_node_names(snapshot.get("nodes", ()))
    common_pass, common_observed = _common_node_gate(node_names)
    localization_pass, localization_observed = _production_localization_gate(
        node_names)
    checks: dict[str, Any] = {}
    _check(
        checks, "mode", mode in {"manual", "autonomous"}, mode,
        "manual or autonomous")
    _check(
        checks, "common_nodes", common_pass, common_observed,
        {"required": sorted(COMMON_REQUIRED_NODES), "duplicates": []})
    _check(
        checks, "production_localization_absent", localization_pass,
        localization_observed, sorted(FORBIDDEN_NODES))

    map_fields = _map_fields(snapshot.get("map"))
    map_received_at = snapshot.get("map_received_at")
    map_pass = _map_gate(map_fields, map_received_at, current)
    _check(
        checks, "fresh_valid_map", map_pass,
        {"received_at": map_received_at, "age_limit": MAP_MAX_AGE_SECONDS,
         "map": map_fields},
        {"frame_id": "map", "fresh": True, "known_free_cell": True,
         "resolution": artifacts.MAP_RESOLUTION})

    tf_receipts = snapshot.get("tf_received_at", {})
    if not isinstance(tf_receipts, Mapping):
        tf_receipts = {}
    map_odom_at = tf_receipts.get(("map", "odom"), tf_receipts.get("map->odom"))
    odom_base_at = tf_receipts.get(
        ("odom", "base_footprint"), tf_receipts.get("odom->base_footprint"))
    tf_observed = {
        "map_to_odom": map_odom_at,
        "odom_to_base_footprint": odom_base_at,
    }
    tf_pass = _localization_tf_gate(tf_observed, current)
    _check(
        checks, "fresh_localization_tf", tf_pass,
        tf_observed,
        {"both_edges_fresh_within_seconds": TF_MAX_AGE_SECONDS})

    map_publishers = _publishers(snapshot, "/map")
    control_publishers = _publishers(snapshot, "/amr/control/cmd_vel")
    mpc_publishers = _publishers(snapshot, "/amr/mpc/cmd_vel")
    ownership_pass = _map_control_ownership_gate(
        map_publishers, control_publishers)
    _check(
        checks, "map_and_control_ownership", ownership_pass,
        {"/map": map_publishers, "/amr/control/cmd_vel": control_publishers},
        {"/map": sorted(SLAM_OWNERS),
         "/amr/control/cmd_vel": sorted(ARBITRATION_OWNERS)})

    tf_ownership_observed = _tf_ownership_observed(
        snapshot.get("tf_publishers"))
    tf_ownership_pass = _tf_ownership_gate(tf_ownership_observed)
    _check(
        checks, "tf_ownership", tf_ownership_pass, tf_ownership_observed,
        {"map->odom": sorted(SLAM_OWNERS),
         "odom->base_footprint": sorted(EKF_OWNERS)})
    tf_ownership_source = _publishers(snapshot, TF_OWNERSHIP_TOPIC)
    _check(
        checks, "tf_ownership_source",
        _tf_ownership_source_gate(tf_ownership_source),
        {TF_OWNERSHIP_TOPIC: tf_ownership_source},
        {TF_OWNERSHIP_TOPIC: [TF_OWNERSHIP_SOURCE]})

    status = snapshot.get("manipulator_status")
    status_received_at = snapshot.get("manipulator_status_received_at")
    manipulator_observed = _manipulator_observed(status, status_received_at)
    manipulator_pass = _manipulator_gate(manipulator_observed, current)
    _check(
        checks, "manipulator_authority", manipulator_pass,
        manipulator_observed,
        {"valid": True, "state": 1, "product_attached": False,
         "base_motion_allowed": True, "fresh_within_seconds": STATUS_MAX_AGE_SECONDS})

    preflight_observed = {
        "host": snapshot.get("host_preflight"),
        "runtime": snapshot.get("runtime_preflight"),
    }
    common_preflight = _preflight_gate(preflight_observed)
    _check(
        checks, "run_preflight_reports", common_preflight,
        preflight_observed,
        {"host": "PASS", "runtime": "PASS"})

    mode_pass = False
    exploration_outcome = None
    if mode == "manual":
        manual_missing = sorted(MANUAL_FORBIDDEN_NODES & set(node_names))
        mode_pass = _manual_mode_gate(node_names, mpc_publishers)
        _check(
            checks, "manual_mode", mode_pass,
            {"nodes": node_names,
             "forbidden_nodes_present": manual_missing,
             "/amr/mpc/cmd_vel": mpc_publishers},
            {"forbidden_nodes_present": [],
             "/amr/mpc/cmd_vel": "no publisher or prototype_teleop only"})
    elif mode == "autonomous":
        missing_autonomous = sorted(AUTONOMOUS_REQUIRED_NODES - set(node_names))
        exploration = _diagnostic_values(snapshot.get("exploration_status"))
        exploration_received_at = snapshot.get(
            "exploration_status_received_at")
        exploration_state = exploration.get("state")
        exploration_outcome = exploration_state
        mode_pass = _autonomous_mode_gate(
            node_names, mpc_publishers, exploration,
            exploration_received_at, exploration_outcome, current)
        _check(
            checks, "autonomous_mode", mode_pass,
            {"nodes": node_names,
             "missing_nodes": missing_autonomous,
             "/amr/mpc/cmd_vel": mpc_publishers,
             "exploration": exploration,
             "exploration_received_at": exploration_received_at},
            {"required_nodes": sorted(AUTONOMOUS_REQUIRED_NODES),
             "prototype_teleop": "absent",
             "/amr/mpc/cmd_vel": sorted(CONTROLLER_OWNERS),
             "exploration": {"state": sorted(AUTONOMOUS_TERMINAL_STATES),
                             "fault_latched": False, "active": False,
                             "pending": False, "cancel_target": "",
                             "run_generation": "> 0",
                             "fresh_within_seconds": STATUS_MAX_AGE_SECONDS}})
    else:
        _check(checks, "mode_requirements", False, mode, "manual or autonomous")

    failures = sorted(name for name, result in checks.items()
                      if not result["passed"])
    result = {
        "schema": ACCEPTANCE_SCHEMA_VERSION,
        "kind": "factory_mapping_runtime_acceptance",
        "mode": mode,
        "checked_monotonic": current,
        "verdict": "PASS" if not failures else "FAIL",
        "timed_out": False,
        "failures": failures,
        "checks": checks,
        "map_counts": _map_counts(map_fields),
    }
    if mode == "autonomous":
        result["exploration_outcome"] = exploration_outcome
    return result


def observe_until_pass(
        adapter: Any,
        mode: str,
        *,
        timeout_seconds: float = ACCEPTANCE_TIMEOUT_SECONDS,
        monotonic: Callable[[], float] = time.monotonic,
        poll_seconds: float = 0.1) -> dict[str, Any]:
    """Observe until all gates pass or the bounded steady-clock deadline expires."""
    timeout = _finite(timeout_seconds, "acceptance timeout")
    poll = _finite(poll_seconds, "acceptance poll")
    if timeout <= 0.0 or timeout > ACCEPTANCE_TIMEOUT_SECONDS:
        raise AcceptanceError(
            "acceptance timeout must be > 0 and <= "
            f"{ACCEPTANCE_TIMEOUT_SECONDS} seconds")
    if poll <= 0.0:
        raise AcceptanceError("acceptance poll must be positive")
    start = monotonic()
    deadline = start + timeout
    latest = None

    def bounded_timeout(result: dict[str, Any], elapsed: float) -> dict[str, Any]:
        result["timed_out"] = True
        result["elapsed_seconds"] = elapsed
        result["deadline_seconds"] = timeout
        if "bounded_timeout" not in result["failures"]:
            result["failures"].append("bounded_timeout")
        result["verdict"] = "FAIL"
        result["checks"]["bounded_timeout"] = {
            "passed": False,
            "observed": elapsed,
            "expected": f"<= {timeout} seconds",
        }
        return result

    while monotonic() < deadline:
        latest = evaluate_snapshot(adapter.snapshot(), mode, now=monotonic())
        completed_at = monotonic()
        if latest["verdict"] == "PASS":
            latest["elapsed_seconds"] = completed_at - start
            latest["deadline_seconds"] = timeout
            if completed_at > deadline:
                return bounded_timeout(latest, latest["elapsed_seconds"])
            return latest
        remaining = deadline - completed_at
        spin_once = getattr(adapter, "spin_once", None)
        if callable(spin_once):
            spin_once(min(poll, max(0.0, remaining)))
        else:
            time.sleep(min(poll, max(0.0, remaining)))
    snapshot = adapter.snapshot()
    latest = evaluate_snapshot(snapshot, mode, now=monotonic())
    completed_at = monotonic()
    return bounded_timeout(latest, completed_at - start)


def _safe_session_file(session_dir: Path, value: str | os.PathLike[str], label: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = session_dir / path
    path = path.absolute()
    artifacts._reject_symlink_components(path, stop=session_dir)
    if path.is_symlink():
        raise AcceptanceError(f"{label} must not be a symlink")
    resolved = path.resolve(strict=False)
    if resolved != session_dir and session_dir not in resolved.parents:
        raise AcceptanceError(f"{label} must remain inside the session directory")
    if not resolved.is_file() or resolved.is_symlink():
        raise AcceptanceError(f"{label} must be a regular file")
    return resolved


def _read_yaml_file(path: Path, label: str) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise AcceptanceError(f"{label} is not valid YAML: {exc}") from exc
    if not isinstance(value, dict):
        raise AcceptanceError(f"{label} must contain a YAML mapping")
    return value


def _require_acceptance_schema(
        document: Mapping[str, Any], label: str) -> None:
    if (type(document.get("schema")) is not int or
            document["schema"] != ACCEPTANCE_SCHEMA_VERSION):
        raise AcceptanceError(f"{label} schema is unsupported")


def _require_quality_review_schema(
        document: Mapping[str, Any], label: str) -> None:
    if (type(document.get("schema")) is not int or
            document["schema"] != QUALITY_REVIEW_SCHEMA_VERSION):
        raise AcceptanceError(f"{label} schema is unsupported")


def _acceptance_receipt_fields(mode: str) -> set[str]:
    fields = set(_ACCEPTANCE_RECEIPT_BASE_FIELDS)
    if mode == "autonomous":
        fields.add("exploration_outcome")
    return fields


def _validate_acceptance_receipt_shape(
        receipt: Mapping[str, Any], mode: str, label: str) -> None:
    if (not isinstance(receipt, dict) or
            set(receipt) != _acceptance_receipt_fields(mode)):
        raise AcceptanceError(f"{label} top-level envelope is invalid")
    if (not isinstance(receipt.get("conditions"), dict) or
            receipt["conditions"] != _ACCEPTANCE_CONDITIONS):
        raise AcceptanceError(f"{label} conditions are invalid")
    for name, expected_fields in _ACCEPTANCE_ENTRY_FIELDS.items():
        entry = receipt.get(name)
        if (not isinstance(entry, dict) or
                set(entry) != expected_fields):
            raise AcceptanceError(f"{label} {name} entry is invalid")


def _manifest_paths(
        session_dir: Path,
        candidate_name: str,
        canonical_dirs: Iterable[Path],
        expected_state: Optional[str] = None) -> tuple[Path, dict[str, Any]]:
    manifest = artifacts.verify_manifest(
        session_dir, candidate_name, canonical_dirs=canonical_dirs,
        expected_state=expected_state)
    return session_dir / artifacts.MANIFEST_NAME, manifest


def _default_canonical_dirs() -> set[Path]:
    paths = set()
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.add(Path(get_package_share_directory("amr_factory"), "maps").resolve())
    except (ImportError, KeyError, RuntimeError):
        pass
    source_maps = Path(__file__).resolve().parents[1] / "maps"
    if source_maps.is_dir():
        paths.add(source_maps.resolve())
    return paths


def _report_base(
        manifest: Mapping[str, Any],
        report: Mapping[str, Any],
        *,
        mode: str) -> dict[str, Any]:
    if not isinstance(report, Mapping):
        raise AcceptanceError("runtime acceptance must contain a mapping")
    if not isinstance(mode, str) or mode not in {"manual", "autonomous"}:
        raise AcceptanceError("runtime acceptance mode is invalid")
    _require_acceptance_schema(report, "runtime acceptance")
    expected_fields = {
        "schema", "kind", "mode", "checked_monotonic", "verdict",
        "timed_out", "failures", "checks", "map_counts", "candidate_name",
        "candidate_path", "candidate_sha256", "artifact_bundle_sha256",
        "session_path", "created_unix", "elapsed_seconds", "deadline_seconds",
    }
    if mode == "autonomous":
        expected_fields.add("exploration_outcome")
    if report.get("kind") != "factory_mapping_runtime_acceptance":
        raise AcceptanceError("runtime acceptance kind is invalid")
    if report.get("mode") != mode:
        raise AcceptanceError("runtime acceptance mode does not match request")
    if report.get("verdict") != "PASS":
        raise AcceptanceError("runtime acceptance did not pass")
    candidate_name = report.get("candidate_name")
    if (not isinstance(candidate_name, str) or
            candidate_name != manifest.get("candidate_name")):
        raise AcceptanceError("runtime acceptance candidate name does not match")
    session_path = report.get("session_path")
    if (not isinstance(session_path, str) or
            session_path != manifest.get("session_path")):
        raise AcceptanceError("runtime acceptance session path does not match")
    created_unix = _finite_real(
        report.get("created_unix"), "runtime acceptance created_unix")
    if created_unix <= 0.0:
        raise AcceptanceError("runtime acceptance created_unix must be positive")
    elapsed_seconds = _finite_real(
        report.get("elapsed_seconds"), "runtime acceptance elapsed_seconds")
    if elapsed_seconds < 0.0:
        raise AcceptanceError(
            "runtime acceptance elapsed_seconds must be non-negative")
    deadline_seconds = _finite_real(
        report.get("deadline_seconds"), "runtime acceptance deadline_seconds")
    if not (0.0 < deadline_seconds <= ACCEPTANCE_TIMEOUT_SECONDS):
        raise AcceptanceError(
            "runtime acceptance deadline_seconds is out of range")
    if elapsed_seconds > deadline_seconds:
        raise AcceptanceError(
            "runtime acceptance elapsed_seconds exceeds deadline_seconds")
    if report.get("timed_out") is not False:
        raise AcceptanceError("runtime acceptance timed_out proof is invalid")
    if report.get("failures") != [] or not isinstance(report.get("failures"), list):
        raise AcceptanceError("runtime acceptance failures proof is invalid")
    checked_monotonic = _finite_real(
        report.get("checked_monotonic"), "checked_monotonic")
    if not isinstance(report.get("checks"), dict):
        raise AcceptanceError("runtime acceptance does not preserve a passing proof")
    required_checks = {
        "mode", "common_nodes", "production_localization_absent",
        "fresh_valid_map", "fresh_localization_tf", "map_and_control_ownership",
        "tf_ownership", "tf_ownership_source", "manipulator_authority",
        "run_preflight_reports",
        "manual_mode" if mode == "manual" else "autonomous_mode",
    }
    if set(report["checks"]) != required_checks or not all(
            isinstance(value, dict) and value.get("passed") is True
            for value in report["checks"].values()):
        raise AcceptanceError("runtime acceptance checks are incomplete or failed")

    mode_observed = report["checks"]["mode"].get("observed")
    if mode_observed != mode:
        raise AcceptanceError("runtime acceptance mode evidence is inconsistent")

    common_observed = report["checks"]["common_nodes"].get("observed")
    node_names = _persisted_node_names(common_observed, "common")
    common_pass, expected_common = _common_node_gate(node_names)
    if (not common_pass or
            common_observed.get("missing") != expected_common["missing"] or
            common_observed.get("duplicates") != expected_common["duplicates"]):
        raise AcceptanceError(
            "runtime acceptance common node evidence contradicts the gate")

    localization_observed = (
        report["checks"]["production_localization_absent"].get("observed"))
    localization_nodes = _persisted_node_names(
        localization_observed, "localization")
    localization_pass, expected_localization = _production_localization_gate(
        localization_nodes)
    if (localization_nodes != node_names or not localization_pass or
            localization_observed.get("forbidden") !=
            expected_localization["forbidden"]):
        raise AcceptanceError(
            "runtime acceptance localization evidence contradicts the gate")

    map_observed = report["checks"]["fresh_valid_map"].get("observed")
    map_received_at, map_fields = _persisted_map_evidence(map_observed)
    if not _map_gate(map_fields, map_received_at, checked_monotonic):
        raise AcceptanceError("runtime acceptance map evidence is stale or invalid")
    if (not isinstance(report.get("map_counts"), dict) or
            report["map_counts"] != _map_counts(map_fields)):
        raise AcceptanceError("runtime acceptance map counts are invalid")

    tf_observed = report["checks"]["fresh_localization_tf"].get("observed")
    if (not isinstance(tf_observed, Mapping) or
            set(tf_observed) != {"map_to_odom", "odom_to_base_footprint"} or
            not _localization_tf_gate(tf_observed, checked_monotonic)):
        raise AcceptanceError("runtime acceptance TF receipts are stale or invalid")

    ownership_observed = (
        report["checks"]["map_and_control_ownership"].get("observed"))
    if (not isinstance(ownership_observed, Mapping) or
            set(ownership_observed) != {"/map", "/amr/control/cmd_vel"}):
        raise AcceptanceError("runtime acceptance map/control evidence is invalid")
    map_publishers = _persisted_publisher_names(
        ownership_observed.get("/map"), "/map")
    control_publishers = _persisted_publisher_names(
        ownership_observed.get("/amr/control/cmd_vel"),
        "/amr/control/cmd_vel")
    if not _map_control_ownership_gate(map_publishers, control_publishers):
        raise AcceptanceError(
            "runtime acceptance map/control ownership contradicts the gate")

    tf_ownership = _persisted_tf_ownership(
        report["checks"]["tf_ownership"].get("observed"))
    if not _tf_ownership_gate(tf_ownership):
        raise AcceptanceError("runtime acceptance TF ownership contradicts the gate")
    tf_ownership_source_observed = report["checks"]["tf_ownership_source"].get(
        "observed")
    if (not isinstance(tf_ownership_source_observed, Mapping) or
            set(tf_ownership_source_observed) != {TF_OWNERSHIP_TOPIC}):
        raise AcceptanceError("runtime acceptance TF evidence source is invalid")
    tf_ownership_source = _persisted_publisher_names(
        tf_ownership_source_observed.get(TF_OWNERSHIP_TOPIC),
        "TF ownership evidence")
    if not _tf_ownership_source_gate(tf_ownership_source):
        raise AcceptanceError(
            "runtime acceptance TF evidence source contradicts the gate")

    manipulator_observed = report["checks"]["manipulator_authority"].get(
        "observed")
    if (not isinstance(manipulator_observed, Mapping) or
            set(manipulator_observed) != {
                "valid", "state", "product_attached", "base_motion_allowed",
                "received_at"} or
            not _manipulator_gate(manipulator_observed, checked_monotonic)):
        raise AcceptanceError(
            "runtime acceptance manipulator evidence contradicts the gate")

    preflight_observed = report["checks"]["run_preflight_reports"].get(
        "observed")
    if (not isinstance(preflight_observed, Mapping) or
            set(preflight_observed) != {"host", "runtime"} or
            not _preflight_gate(preflight_observed)):
        raise AcceptanceError(
            "runtime acceptance preflight evidence contradicts the gate")

    if mode == "autonomous":
        exploration_outcome = report.get("exploration_outcome")
        if (not isinstance(exploration_outcome, str) or
                exploration_outcome not in AUTONOMOUS_TERMINAL_STATES):
            raise AcceptanceError("runtime acceptance exploration outcome is invalid")
        autonomous_observed = report["checks"]["autonomous_mode"].get("observed")
        observed_exploration = (
            autonomous_observed.get("exploration")
            if isinstance(autonomous_observed, Mapping) else None)
        if (not isinstance(observed_exploration, Mapping) or
                observed_exploration.get("state") != exploration_outcome):
            raise AcceptanceError(
                "runtime acceptance exploration outcome does not match observed state")
        autonomous_observed = report["checks"]["autonomous_mode"].get(
            "observed")
        if (not isinstance(autonomous_observed, Mapping) or
                set(autonomous_observed) != {
                    "nodes", "missing_nodes", "/amr/mpc/cmd_vel",
                    "exploration", "exploration_received_at"}):
            raise AcceptanceError(
                "runtime acceptance autonomous evidence is invalid")
        autonomous_nodes = _persisted_node_names(
            autonomous_observed, "autonomous")
        autonomous_publishers = _persisted_publisher_names(
            autonomous_observed.get("/amr/mpc/cmd_vel"),
            "autonomous MPC")
        expected_missing = sorted(
            AUTONOMOUS_REQUIRED_NODES - set(autonomous_nodes))
        if (autonomous_nodes != node_names or
                autonomous_observed.get("missing_nodes") != expected_missing or
                not _autonomous_mode_gate(
                    autonomous_nodes, autonomous_publishers,
                    observed_exploration,
                    autonomous_observed.get("exploration_received_at"),
                    exploration_outcome, checked_monotonic)):
            raise AcceptanceError(
                "runtime acceptance autonomous evidence contradicts the gate")
    else:
        manual_observed = report["checks"]["manual_mode"].get("observed")
        if (not isinstance(manual_observed, Mapping) or
                set(manual_observed) != {
                    "nodes", "forbidden_nodes_present", "/amr/mpc/cmd_vel"}):
            raise AcceptanceError("runtime acceptance manual evidence is invalid")
        manual_nodes = _persisted_node_names(manual_observed, "manual")
        manual_publishers = _persisted_publisher_names(
            manual_observed.get("/amr/mpc/cmd_vel"), "manual MPC")
        expected_forbidden = sorted(
            MANUAL_FORBIDDEN_NODES & set(manual_nodes))
        if (manual_nodes != node_names or
                manual_observed.get("forbidden_nodes_present") !=
                expected_forbidden or
                not _manual_mode_gate(manual_nodes, manual_publishers)):
            raise AcceptanceError(
                "runtime acceptance manual evidence contradicts the gate")

    if set(report) != expected_fields:
        raise AcceptanceError("runtime acceptance top-level envelope is invalid")
    if report.get("candidate_sha256") != manifest.get("candidate_sha256"):
        raise AcceptanceError("runtime acceptance candidate hash does not match")
    if report.get("artifact_bundle_sha256") != manifest.get("artifact_bundle_sha256"):
        raise AcceptanceError("runtime acceptance artifact bundle hash does not match")
    if report.get("candidate_path") != manifest.get("candidate_path"):
        raise AcceptanceError("runtime acceptance candidate path does not match")
    return dict(report)


def _quality_review(
        path: Path,
        manifest: Mapping[str, Any],
        *,
        runtime_acceptance_sha256: str,
        exploration_outcome: Optional[str]) -> dict[str, Any]:
    review = _read_yaml_file(path, "quality review")
    _require_quality_review_schema(review, "quality review")
    decision = str(review.get("decision", "")).strip().upper()
    if decision != "ACCEPTED":
        raise AcceptanceError("quality review decision is not ACCEPTED")
    if "accepted" in review and review["accepted"] is not True:
        raise AcceptanceError("quality review accepted field contradicts decision")
    reviewer = review.get("reviewer")
    if not isinstance(reviewer, str) or not reviewer.strip():
        raise AcceptanceError("quality review reviewer is missing")
    if review.get("candidate_sha256") != manifest.get("candidate_sha256"):
        raise AcceptanceError("quality review candidate hash does not match")
    if (review.get("artifact_bundle_sha256") !=
            manifest.get("artifact_bundle_sha256")):
        raise AcceptanceError("quality review artifact bundle hash does not match")
    if review.get("candidate_path") != manifest.get("candidate_path"):
        raise AcceptanceError("quality review candidate path does not match")
    if review.get("runtime_acceptance_sha256") != runtime_acceptance_sha256:
        raise AcceptanceError("quality review runtime acceptance hash does not match")
    if exploration_outcome == "INCOMPLETE":
        if review.get("incomplete_acknowledgement") != "ACCEPT_INCOMPLETE":
            raise AcceptanceError(
                "quality review incomplete acknowledgement is invalid")
        explanation = review.get("incomplete_explanation")
        if not isinstance(explanation, str) or not explanation.strip():
            raise AcceptanceError("quality review incomplete explanation is missing")
    return review


def _runtime_report_path(session_dir: Path) -> Path:
    return _safe_session_file(
        session_dir, session_dir / RUNTIME_ACCEPTANCE_NAME, "runtime acceptance")


def _quality_review_path(session_dir: Path, value: Optional[str]) -> Path:
    return _safe_session_file(
        session_dir, value or QUALITY_REVIEW_NAME, "quality review")


def run_runtime_acceptance(
        session_dir: str | os.PathLike[str],
        candidate_name: str,
        mode: str,
        adapter: Any,
        *,
        timeout_seconds: float = ACCEPTANCE_TIMEOUT_SECONDS,
        canonical_dirs: Iterable[Path] = (),
        monotonic: Callable[[], float] = time.monotonic,
        poll_seconds: float = 0.1) -> dict[str, Any]:
    canonical_dirs = tuple(canonical_dirs)
    session = artifacts.safe_session_dir(session_dir, canonical_dirs, create=False)
    manifest = artifacts.verify_manifest(
        session, candidate_name, canonical_dirs=canonical_dirs,
        expected_state="VALIDATED")
    result = observe_until_pass(
        adapter, mode, timeout_seconds=timeout_seconds,
        monotonic=monotonic, poll_seconds=poll_seconds)
    report = dict(result)
    report.update({
        "candidate_name": manifest["candidate_name"],
        "candidate_path": manifest["candidate_path"],
        "candidate_sha256": manifest["candidate_sha256"],
        "artifact_bundle_sha256": manifest["artifact_bundle_sha256"],
        "session_path": str(session),
        "created_unix": time.time(),
    })
    artifacts.atomic_write_yaml(session / RUNTIME_ACCEPTANCE_NAME, report)
    return report


def accept_candidate(
        session_dir: str | os.PathLike[str],
        candidate_name: str,
        *,
        quality_review: Optional[str] = None,
        canonical_dirs: Iterable[Path] = ()) -> dict[str, Any]:
    canonical_dirs = tuple(canonical_dirs)
    session = artifacts.safe_session_dir(session_dir, canonical_dirs, create=False)
    manifest_path, manifest = _manifest_paths(session, candidate_name, canonical_dirs)
    if manifest["state"] != "VALIDATED":
        raise AcceptanceError("validate the saved artifact manifest before accept")
    runtime_path = _runtime_report_path(session)
    runtime_document = _read_yaml_file(runtime_path, "runtime acceptance")
    runtime = _report_base(
        manifest, runtime_document,
        mode=runtime_document.get("mode"))
    runtime_sha256 = artifacts.sha256_file(runtime_path)
    review_path = _quality_review_path(session, quality_review)
    review = _quality_review(
        review_path, manifest,
        runtime_acceptance_sha256=runtime_sha256,
        exploration_outcome=runtime.get("exploration_outcome"))
    receipt = {
        "schema": ACCEPTANCE_SCHEMA_VERSION,
        "kind": "factory_mapping_acceptance",
        "state": "PROMOTION_ELIGIBLE",
        "created_unix": time.time(),
        "session_path": str(session),
        "candidate_name": manifest["candidate_name"],
        "candidate_path": manifest["candidate_path"],
        "candidate_sha256": manifest["candidate_sha256"],
        "artifact_bundle_sha256": manifest["artifact_bundle_sha256"],
        "conditions": {
            "artifact_manifest": "VALIDATED",
            "runtime_acceptance": "PASS",
            "quality_review": "ACCEPTED",
        },
        "artifact_manifest": {
            "path": str(manifest_path),
            "sha256": artifacts.sha256_file(manifest_path),
        },
        "runtime_acceptance": {
            "path": str(runtime_path),
            "sha256": runtime_sha256,
            "mode": runtime["mode"],
        },
        "quality_review": {
            "path": str(review_path),
            "sha256": artifacts.sha256_file(review_path),
            "reviewer": review["reviewer"],
        },
    }
    if runtime["mode"] == "autonomous":
        receipt["exploration_outcome"] = runtime["exploration_outcome"]
    _validate_acceptance_receipt_shape(
        receipt, runtime["mode"], "constructed mapping acceptance")
    artifacts.atomic_write_yaml(session / MAPPING_ACCEPTANCE_NAME, receipt)
    return receipt


def _verify_acceptance_receipt(
        session: Path,
        candidate_name: str,
        *,
        canonical_dirs: Iterable[Path]) -> tuple[dict[str, Any], dict[str, Any]]:
    manifest_path, manifest = _manifest_paths(
        session, candidate_name, canonical_dirs,
        expected_state="VALIDATED")
    acceptance_path = _safe_session_file(
        session, session / MAPPING_ACCEPTANCE_NAME, "mapping acceptance")
    receipt = _read_yaml_file(acceptance_path, "mapping acceptance")
    _require_acceptance_schema(receipt, "mapping acceptance")
    if (receipt.get("kind") != "factory_mapping_acceptance" or
            receipt.get("state") != "PROMOTION_ELIGIBLE"):
        raise AcceptanceError("mapping acceptance is not promotion eligible")
    created_unix = _finite_real(
        receipt.get("created_unix"), "mapping acceptance created_unix")
    if created_unix <= 0.0:
        raise AcceptanceError(
            "mapping acceptance created_unix must be positive")
    if receipt.get("session_path") != str(session):
        raise AcceptanceError("mapping acceptance session path does not match")
    for key in (
            "candidate_name", "candidate_path", "candidate_sha256",
            "artifact_bundle_sha256"):
        if receipt.get(key) != manifest.get(key):
            raise AcceptanceError(f"mapping acceptance {key} does not match manifest")
    runtime_entry = receipt.get("runtime_acceptance")
    if not isinstance(runtime_entry, dict):
        raise AcceptanceError("mapping acceptance evidence references are incomplete")
    if runtime_entry.get("path") != str(session / RUNTIME_ACCEPTANCE_NAME):
        raise AcceptanceError("mapping acceptance runtime path changed after accept")
    runtime_path = _safe_session_file(
        session, runtime_entry.get("path", ""), "runtime acceptance")
    runtime_document = _read_yaml_file(runtime_path, "runtime acceptance")
    runtime = _report_base(
        manifest, runtime_document,
        mode=runtime_document.get("mode"))
    runtime_sha256 = artifacts.sha256_file(runtime_path)
    _validate_acceptance_receipt_shape(receipt, runtime["mode"], "mapping acceptance")
    runtime_entry = receipt["runtime_acceptance"]
    review_entry = receipt["quality_review"]
    if runtime_entry["mode"] != runtime["mode"]:
        raise AcceptanceError("mapping acceptance runtime mode changed after accept")
    if (runtime["mode"] == "autonomous" and
            receipt.get("exploration_outcome") != runtime["exploration_outcome"]):
        raise AcceptanceError(
            "mapping acceptance exploration outcome does not match runtime acceptance")
    if runtime_entry["sha256"] != runtime_sha256:
        raise AcceptanceError("runtime acceptance hash changed after accept")
    artifact_entry = receipt["artifact_manifest"]
    if artifact_entry["path"] != str(manifest_path):
        raise AcceptanceError("artifact manifest path changed after accept")
    manifest_sha256 = artifacts.sha256_file(manifest_path)
    if artifact_entry["sha256"] != manifest_sha256:
        raise AcceptanceError("artifact manifest changed after accept")
    if not isinstance(review_entry["path"], str):
        raise AcceptanceError("mapping acceptance quality review path is invalid")
    review_path = _safe_session_file(
        session, review_entry["path"], "quality review")
    if review_entry["path"] != str(review_path):
        raise AcceptanceError("mapping acceptance quality review path changed after accept")
    review = _quality_review(
        review_path, manifest,
        runtime_acceptance_sha256=runtime_sha256,
        exploration_outcome=runtime.get("exploration_outcome"))
    if review_entry["reviewer"] != review["reviewer"]:
        raise AcceptanceError("mapping acceptance quality reviewer changed after accept")
    if review_entry["sha256"] != artifacts.sha256_file(review_path):
        raise AcceptanceError("quality review hash changed after accept")
    return receipt, manifest


def _canonical_snapshot(canonical_dirs: Iterable[Path]) -> dict[str, str]:
    snapshot = {}
    for directory in canonical_dirs:
        root = Path(directory).resolve()
        for name in ("factory.yaml", "factory.pgm", "factory.png", "factory.bmp"):
            path = root / name
            if path.is_file() and not path.is_symlink():
                snapshot[str(path)] = artifacts.sha256_file(path)
    return dict(sorted(snapshot.items()))


def promote_candidate(
        session_dir: str | os.PathLike[str],
        candidate_name: str,
        *,
        canonical_dirs: Iterable[Path] = ()) -> dict[str, Any]:
    canonical_dirs = tuple(canonical_dirs)
    session = artifacts.safe_session_dir(session_dir, canonical_dirs, create=False)
    receipt, manifest = _verify_acceptance_receipt(
        session, candidate_name, canonical_dirs=canonical_dirs)
    canonical_snapshot = _canonical_snapshot(canonical_dirs)
    promotion = {
        "schema": ACCEPTANCE_SCHEMA_VERSION,
        "kind": "factory_mapping_promotion_eligibility",
        "state": "PROMOTION_ELIGIBLE",
        "created_unix": time.time(),
        "session_path": str(session),
        "candidate_name": manifest["candidate_name"],
        "candidate_path": manifest["candidate_path"],
        "candidate_sha256": manifest["candidate_sha256"],
        "artifact_bundle_sha256": manifest["artifact_bundle_sha256"],
        "acceptance_path": str(session / MAPPING_ACCEPTANCE_NAME),
        "canonical_map_unchanged": True,
        "canonical_map_sha256": canonical_snapshot,
        "note": (
            "Eligibility receipt only. No canonical map was copied, replaced, "
            "renamed, or modified. Canonical replacement requires separate authorization."
        ),
    }
    # `receipt` is intentionally read and rechecked above; writing this one
    # receipt is the only mutation permitted by the promote operation.
    del receipt
    artifacts.atomic_write_yaml(session / PROMOTION_RECEIPT_NAME, promotion)
    return promotion


class RosGraphObserver:
    """Live ROS observer seam; it subscribes only to evidence topics."""

    def __init__(self, node: Any, host_report: Path, runtime_report: Path) -> None:
        from amr_interfaces.msg import ManipulatorStatus
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self.node = node
        self.host_report = Path(host_report)
        self.runtime_report = Path(runtime_report)
        self.map = None
        self.map_received_at = None
        self.tf_received_at: dict[tuple[str, str], float] = {}
        self.tf_publishers: Optional[dict[str, list[str]]] = None
        self.tf_ownership_received_at = None
        self.manipulator_status = None
        self.manipulator_status_received_at = None
        self.exploration_status = None
        self.exploration_status_received_at = None
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ownership_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._subscriptions = [
            node.create_subscription(
                OccupancyGrid, "/map", self._map_callback, map_qos),
            node.create_subscription(
                TFMessage, "/tf", self._tf_callback, 100),
            node.create_subscription(
                String, TF_OWNERSHIP_TOPIC, self._tf_ownership_callback, ownership_qos),
            node.create_subscription(
                ManipulatorStatus, "/amr/manipulation/status",
                self._manipulator_callback, 1),
            node.create_subscription(
                DiagnosticArray, "/amr/exploration/status",
                self._exploration_callback, 10),
        ]

    def _map_callback(self, message: Any) -> None:
        self.map = message
        self.map_received_at = time.monotonic()

    def _tf_callback(self, message: Any) -> None:
        received_at = time.monotonic()
        for transform in getattr(message, "transforms", ()):
            parent = str(getattr(transform.header, "frame_id", "")).strip().lstrip("/")
            child = str(getattr(transform, "child_frame_id", "")).strip().lstrip("/")
            if parent and child:
                self.tf_received_at[(parent, child)] = received_at

    def _tf_ownership_callback(self, message: Any) -> None:
        self.tf_publishers = _parse_tf_ownership_message(
            getattr(message, "data", None))
        self.tf_ownership_received_at = time.monotonic()

    def _manipulator_callback(self, message: Any) -> None:
        self.manipulator_status = message
        self.manipulator_status_received_at = time.monotonic()

    def _exploration_callback(self, message: Any) -> None:
        for status in getattr(message, "status", ()):
            if "frontier_explorer" in str(getattr(status, "name", "")):
                values = _diagnostic_values(status)
                self.exploration_status = values
                self.exploration_status_received_at = time.monotonic()
                return

    def _node_names(self) -> list[str]:
        return [
            _full_node_name(name, namespace)
            for name, namespace in self.node.get_node_names_and_namespaces()
        ]

    def _topic_publishers(self, topic: str) -> list[str]:
        return [
            _full_node_name(info.node_name, info.node_namespace)
            for info in self.node.get_publishers_info_by_topic(topic)
        ]

    def _preflight(self, path: Path) -> dict[str, Any]:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            return {"path": str(path), "verdict": "MISSING", "error": str(exc)}
        verdict = next((line.split("=", 1)[1].strip()
                        for line in lines if line.startswith("verdict=")), "MISSING")
        return {"path": str(path), "verdict": verdict}

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        tf_publishers = self.tf_publishers
        if not _fresh(self.tf_ownership_received_at, TF_MAX_AGE_SECONDS, now):
            tf_publishers = None
        return {
            "now": now,
            "nodes": self._node_names(),
            "topic_publishers": {
                topic: self._topic_publishers(topic)
                for topic in (
                    "/map", "/amr/control/cmd_vel", "/amr/mpc/cmd_vel",
                    TF_OWNERSHIP_TOPIC)
            },
            "map": self.map,
            "map_received_at": self.map_received_at,
            "tf_received_at": dict(self.tf_received_at),
            "tf_publishers": tf_publishers,
            "manipulator_status": self.manipulator_status,
            "manipulator_status_received_at": self.manipulator_status_received_at,
            "exploration_status": self.exploration_status,
            "exploration_status_received_at": self.exploration_status_received_at,
            "host_preflight": self._preflight(self.host_report),
            "runtime_preflight": self._preflight(self.runtime_report),
        }

    def spin_once(self, timeout_sec: float) -> None:
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)


def _live_runtime(args) -> int:
    import rclpy

    session = artifacts.safe_session_dir(
        args.session_dir, _default_canonical_dirs(), create=False)
    evidence_dir = (
        Path(args.evidence_dir) if args.evidence_dir else session / "evidence")
    host_report = (
        Path(args.host_report) if args.host_report
        else evidence_dir / "host_preflight.txt")
    runtime_report = (
        Path(args.runtime_report) if args.runtime_report
        else evidence_dir / "runtime_preflight.txt")
    host_report = _safe_session_file(session, host_report, "host preflight report")
    runtime_report = _safe_session_file(
        session, runtime_report, "runtime preflight report")
    rclpy.init()
    node = rclpy.create_node("factory_mapping_acceptance")
    observer = RosGraphObserver(node, host_report, runtime_report)
    try:
        report = run_runtime_acceptance(
            session, args.name, args.mode, observer,
            timeout_seconds=args.timeout_sec,
            canonical_dirs=_default_canonical_dirs())
        print(yaml.safe_dump(report, sort_keys=False), end="")
        return 0 if report["verdict"] == "PASS" else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    sub = subparsers.add_parser("runtime")
    sub.add_argument("--session-dir", required=True)
    sub.add_argument("--name", default="factory_candidate")
    sub.add_argument("--mode", choices=("manual", "autonomous"), required=True)
    sub.add_argument("--evidence-dir")
    sub.add_argument("--host-report")
    sub.add_argument("--runtime-report")
    sub.add_argument("--timeout-sec", type=float, default=ACCEPTANCE_TIMEOUT_SECONDS)
    sub.set_defaults(handler=_live_runtime)

    sub = subparsers.add_parser("accept")
    sub.add_argument("--session-dir", required=True)
    sub.add_argument("--name", default="factory_candidate")
    sub.add_argument("--quality-review", default=None)
    sub.set_defaults(handler=lambda args: _accept_command(args))

    sub = subparsers.add_parser("promote")
    sub.add_argument("--session-dir", required=True)
    sub.add_argument("--name", default="factory_candidate")
    sub.set_defaults(handler=lambda args: _promote_command(args))

    args = parser.parse_args(argv)
    try:
        return int(args.handler(args))
    except (OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"factory mapping acceptance: FAIL: {exc}", file=sys.stderr)
        return 1


def _accept_command(args) -> int:
    receipt = accept_candidate(
        args.session_dir, args.name, quality_review=args.quality_review,
        canonical_dirs=_default_canonical_dirs())
    print(yaml.safe_dump(receipt, sort_keys=False), end="")
    return 0


def _promote_command(args) -> int:
    receipt = promote_candidate(
        args.session_dir, args.name, canonical_dirs=_default_canonical_dirs())
    print(yaml.safe_dump(receipt, sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
