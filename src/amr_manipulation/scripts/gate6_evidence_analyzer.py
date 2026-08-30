#!/usr/bin/env python3
"""Analyze one recorded Gate 6 bag without inferring a terminal pass.

The analyzer is intentionally independent of the stage process.  It derives
the selected product and slot from the factory registry, identifies the mass
stage by its ``source_boot_id``, and checks only samples inside that stage's
recorded interval.  A pass is written as one stable machine-readable line;
diagnostics for a failed bag go to stderr and the output file contains the
corresponding FAIL line.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import math
from pathlib import Path
import sys
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


def analyze(bag: Path, product_id: int) -> List[str]:
    model, mass_kg, slot = _load_product_registry(product_id)
    selected_state_topic = f"/amr/simulation/internal/attachment/product_{product_id}/state"
    selected_pose_topic = f"/model/{model}/pose"

    reader = _open_reader(bag)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    counts: Dict[str, int] = defaultdict(int)
    status_samples = []
    bootstrap_samples = []
    attachment_samples = []
    pose_samples = []
    robot_pose_samples = []
    contacts = {"left": [], "right": []}
    commands = {CONTROL_TOPIC: [], SIMULATION_TOPIC: []}
    normal_nav_active = False
    rosout_markers = []
    decode_failures = []
    selected_topics = {
        "/amr/manipulation/status", BOOTSTRAP_TOPIC, selected_state_topic,
        selected_pose_topic, "/amr/simulation/ground_truth/pose",
        "/amr/simulation/contacts/left_finger", "/amr/simulation/contacts/right_finger",
        CONTROL_TOPIC, SIMULATION_TOPIC, NORMAL_NAV_STATUS_TOPIC, "/rosout",
    }

    while reader.has_next():
        topic, payload, bag_timestamp = reader.read_next()
        counts[topic] += 1
        if topic not in selected_topics:
            continue
        message_type = topic_types.get(topic)
        if message_type is None:
            decode_failures.append(f"{topic}: unknown type")
            continue
        try:
            message = deserialize_message(payload, get_message(message_type))
        except Exception as error:  # noqa: BLE001 - a corrupt evidence stream fails closed
            decode_failures.append(f"{topic}: {error}")
            continue
        timestamp = float(bag_timestamp) * 1e-9
        if topic == "/amr/manipulation/status":
            status_samples.append((timestamp, message))
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
            normal_nav_active = normal_nav_active or any(
                int(item.status) in {2, 3, 4, 5, 6} for item in message.status_list
            )
        elif topic == "/rosout":
            rosout_markers.append(str(getattr(message, "msg", "")))

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

    stage_statuses = [
        (timestamp, message) for timestamp, message in status_samples
        if int(message.source_boot_id) != 0
    ]
    if not stage_statuses:
        failures.append("mass-stage source_boot_id evidence is missing")
        stage_boot_id = 0
        stage_start = 0.0
        stage_end = 0.0
    else:
        stage_boot_id = int(stage_statuses[0][1].source_boot_id)
        stage_start = stage_statuses[0][0]
        stage_end = max(
            (timestamp for timestamp, message in status_samples
             if int(message.source_boot_id) == stage_boot_id),
            default=stage_start,
        )
        scoped_statuses = [
            (timestamp, message) for timestamp, message in status_samples
            if int(message.source_boot_id) == stage_boot_id and stage_start <= timestamp <= stage_end
        ]
        if any(int(message.sequence) == 0 for _, message in scoped_statuses):
            failures.append("mass-stage status sequence is invalid")
        if any(not bool(message.valid) and int(message.state) != 5 for _, message in scoped_statuses):
            failures.append("mass-stage status became invalid before terminal state")

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

    control = sorted(commands[CONTROL_TOPIC])
    simulation = sorted(commands[SIMULATION_TOPIC])
    simulation_nonzero = [row for row in simulation if abs(row[1]) > 1e-12 or abs(row[2]) > 1e-12]
    if not control or not simulation_nonzero:
        failures.append("base command evidence was empty")
    else:
        control_index = 0
        for timestamp, linear, angular in simulation_nonzero:
            # The bridge may deliver a command after its arbitration sample;
            # compare against the latest command at or before the output, not
            # a future sample from the next ramp point.
            while control_index + 1 < len(control) and control[control_index + 1][0] <= timestamp:
                control_index += 1
            reference = control[control_index]
            if timestamp - reference[0] > 0.25 or not (
                math.isclose(reference[1], linear, rel_tol=0.0, abs_tol=1e-6) and
                math.isclose(reference[2], angular, rel_tol=0.0, abs_tol=1e-6)
            ):
                failures.append("simulation base command did not match command arbitration")
                break

    # Check both command authority and measured base pose while the stage says
    # base motion is forbidden.  Nav/status samples are all bag-time ordered.
    robot_samples = sorted(robot_pose_samples)
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
