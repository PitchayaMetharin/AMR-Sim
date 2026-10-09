import importlib.util
import json
import math
from pathlib import Path

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Vector3
from ros_gz_interfaces.msg import Contact, Contacts, Entity, JointWrench
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


SCRIPT = Path(__file__).parents[1] / "scripts" / "aws_exploration_diagnostics.py"
LAUNCHER = Path(__file__).parents[1] / "scripts" / "aws_exploration_diagnostics_run.py"
RUNNER = Path(__file__).parents[1] / "scripts" / "aws_exploration_runner.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_evidence_topics_are_normalized_unique_and_cover_failed_boundary():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics")
    topics = diagnostics.evidence_topics()

    assert len(topics) == len(set(topics))
    assert all(topic.startswith("/") and topic == topic.strip() for topic in topics)
    required = {
        "/amr/base/joint_states",
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
        "/amr/simulation/base/odometry",
        "/amr/simulation/base/joint_states",
        "/amr/localization/wheel_odometry",
        "/amr/localization/odometry",
        "/amr/simulation/ground_truth/pose",
        "/amr/simulation/diagnostics/contacts",
        "/amr/simulation/diagnostics/contact_coverage",
        "/amr/manipulation/status",
        "/amr/base/status",
        "/amr/mission/status",
        "/amr/compute_path_to_pose/_action/status",
        "/amr/smooth_path/_action/status",
        "/amr/follow_path/_action/status",
        "/amr/lookahead_point",
        "/amr/lookahead_collision_arc",
    }
    assert required <= set(topics)


def test_maintained_launcher_uses_runner_gui_rviz_diagnostics_and_valid_domain(tmp_path):
    launcher = _load(LAUNCHER, "aws_exploration_diagnostics_run")
    run_dir = (tmp_path / "aws_diagnostics_01").resolve()
    config = launcher.build_run_config(run_dir, 232)

    normalized = config.normalized()
    launch = normalized.commands["launch"]
    recorder = normalized.commands["recorder"]
    assert "headless:=false" in launch
    assert "rviz:=true" in launch
    assert "simulation_diagnostics:=true" in launch
    assert "auto_start_exploration:=true" in launch
    assert "--include-hidden-topics" in recorder
    assert "--include-unpublished-topics" in recorder
    assert "--simulation-evidence" in normalized.commands["diagnostics"]
    assert normalized.map_saver is not None
    assert "nav2_map_server" in normalized.map_saver
    assert "map_saver_cli" in normalized.map_saver
    assert "save_map_timeout:=10.0" in normalized.map_saver
    runner = _load(RUNNER, "aws_exploration_runner_bound")
    assert 10.0 < runner.DEFAULT_MAP_SAVE_TIMEOUT_SEC
    assert str(run_dir / "evidence" / "aws_map") in normalized.map_saver
    assert normalized.map_output_prefix == run_dir / "evidence" / "aws_map"
    assert set(_load(SCRIPT, "diagnostics_topics").evidence_topics()) <= set(recorder)
    assert normalized.ros_domain_id == 232


def test_simulation_evidence_flag_is_explicit_and_defaults_off():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_parser")
    assert not diagnostics._parser().parse_args(["--output", "evidence.jsonl"]).simulation_evidence
    assert diagnostics._parser().parse_args([
        "--output", "evidence.jsonl", "--simulation-evidence",
    ]).simulation_evidence


def test_plugin_source_and_build_contract_are_observation_only():
    root = Path(__file__).parents[1]
    source = (root / "src" / "exploration_evidence_system.cpp").read_text()
    cmake = (root / "CMakeLists.txt").read_text()

    assert "amr-exploration-evidence-system" in cmake
    assert "src/exploration_evidence_system.cpp" in cmake
    assert "ISystemConfigure" in source
    assert "ISystemPreUpdate" in source
    assert "ISystemPostUpdate" in source
    assert "ContactSensorData" in source
    assert "kContactsTopic" in source
    assert "kDeliveredCommandTopic" in source
    assert "kCoverageTopic" in source
    assert "Subscribe" in source and "kCommandTopic" in source
    for forbidden in (
        "LinearVelocityCmd",
        "AngularVelocityCmd",
        "JointVelocityCmd",
        "ExternalWorldWrenchCmd",
        "WorldPoseCmd",
        "ApplyLinkWrench",
    ):
        assert forbidden not in source


def _entity(name, identifier):
    entity = Entity()
    entity.name = name
    entity.id = identifier
    entity.type = Entity.COLLISION
    return entity


def _contact(first_name, second_name, *, first_id=11, second_id=22):
    contact = Contact()
    contact.collision1 = _entity(first_name, first_id)
    contact.collision2 = _entity(second_name, second_id)
    contact.positions = [Vector3(x=1.0, y=2.0, z=3.0)]
    contact.normals = [Vector3(x=0.0, y=0.0, z=1.0)]
    contact.depths = [0.01]
    wrench = JointWrench()
    wrench.body_1_name.data = first_name
    wrench.body_1_id.data = first_id
    wrench.body_2_name.data = second_name
    wrench.body_2_id.data = second_id
    wrench.body_1_wrench.force.x = 4.0
    wrench.body_2_wrench.force.x = -4.0
    contact.wrenches = [wrench]
    return contact


def test_contact_classification_is_exact_and_fail_closed():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_contacts")
    ground = (
        "aws_robomaker_warehouse_GroundB_01_001::"
        "ground_link::collision"
    )
    nested_ground = (
        "aws_robomaker_warehouse_GroundB_01_001::GroundB::"
        "ground_link::collision"
    )
    left_wheel = "amr::left_wheel::collision"

    assert diagnostics.classify_contact(_contact(left_wheel, ground)) == "NORMAL_SUPPORT"
    assert diagnostics.classify_contact(_contact(nested_ground, left_wheel)) == "NORMAL_SUPPORT"
    assert diagnostics.classify_contact(_contact(
        left_wheel, "aws_robomaker_warehouse_ShelfD_01_001::shelf::collision"
    )) == "OBSTACLE_CONTACT"
    assert diagnostics.classify_contact(_contact(
        "amr::left_wheel::collision", "amr::right_wheel::collision"
    )) == "SELF_CONTACT"
    assert diagnostics.classify_contact(_contact(
        "amr::base_link::lower_chassis_collision", ground
    )) == "UNEXPECTED_GROUND_CONTACT"

    blank = _contact(left_wheel, ground)
    blank.collision2.name = ""
    assert diagnostics.classify_contact(blank) == "UNKNOWN_CONTACT"
    ambiguous = _contact(left_wheel, ground)
    ambiguous.collision2.id = ambiguous.collision1.id
    assert diagnostics.classify_contact(ambiguous) == "UNKNOWN_CONTACT"
    nonfinite = _contact(left_wheel, ground)
    nonfinite.positions[0].x = math.nan
    assert diagnostics.classify_contact(nonfinite) == "UNKNOWN_CONTACT"


def test_raw_contact_record_preserves_all_inspectable_fields():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_raw")
    message = Contacts()
    message.header.stamp.sec = 7
    message.header.stamp.nanosec = 8
    message.contacts = [_contact(
        "amr::right_wheel::collision",
        "aws_robomaker_warehouse_GroundB_01_001::ground_link::collision",
    )]

    record = diagnostics.raw_message_record(
        "/amr/simulation/diagnostics/contacts", message,
        received_wall_time=12.0, received_monotonic=34.0,
    )

    raw = record["message"]
    saved = raw["contacts"][0]
    assert saved["collision1"] == {
        "id": 11,
        "name": "amr::right_wheel::collision",
        "type": Entity.COLLISION,
    }
    assert saved["collision2"]["id"] == 22
    assert saved["positions"] == [{"x": 1.0, "y": 2.0, "z": 3.0}]
    assert saved["normals"] == [{"x": 0.0, "y": 0.0, "z": 1.0}]
    assert saved["depths"] == [0.01]
    assert saved["wrenches"][0]["body_1_name"] == {"data": saved["collision1"]["name"]}
    assert saved["wrenches"][0]["body_1_wrench"]["force"]["x"] == 4.0
    json.dumps(record, allow_nan=False)


def test_coverage_requires_complete_counts_and_non_decreasing_receipts():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_coverage")
    valid = String(data=json.dumps({
        "simulation_time_ns": 123,
        "monitored_collisions": 8,
        "covered_collisions": 8,
        "contact_count": 6,
        "command_receipt_count": 10,
    }))
    parsed = diagnostics.parse_coverage(valid, previous_command_receipt_count=9)
    assert parsed["simulation_time_ns"] == 123

    for payload in (
        {"simulation_time_ns": 123, "monitored_collisions": 0,
         "covered_collisions": 0, "contact_count": 0,
         "command_receipt_count": 10},
        {"simulation_time_ns": 123, "monitored_collisions": 8,
         "covered_collisions": 7, "contact_count": 0,
         "command_receipt_count": 10},
        {"simulation_time_ns": 123, "monitored_collisions": 8,
         "covered_collisions": 8, "contact_count": -1,
         "command_receipt_count": 10},
        {"simulation_time_ns": 123, "monitored_collisions": 8,
         "covered_collisions": 8, "contact_count": 0,
         "command_receipt_count": 8},
        {"simulation_time_ns": True, "monitored_collisions": 8,
         "covered_collisions": 8, "contact_count": 0,
         "command_receipt_count": 10},
    ):
        with pytest.raises(ValueError):
            diagnostics.parse_coverage(
                String(data=json.dumps(payload)), previous_command_receipt_count=9)


def test_pose_and_laser_validation_reject_bad_geometry_and_nan():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_geometry")
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    assert diagnostics.valid_ground_truth(pose)
    pose.pose.position.x = math.inf
    assert not diagnostics.valid_ground_truth(pose)

    scan = LaserScan()
    scan.angle_min = -1.0
    scan.angle_max = 1.0
    scan.angle_increment = 1.0
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0, math.inf, 2.0]
    assert diagnostics.valid_laser_scan(scan)
    scan.ranges[1] = math.nan
    assert not diagnostics.valid_laser_scan(scan)


def test_stream_liveness_requires_advancing_stamps_and_waits_for_clock():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_liveness")
    state = diagnostics.SimulationEvidenceState()

    assert state.observe_clock(100, now=10.0) == "advanced"
    for topic in diagnostics.REQUIRED_SIMULATION_STREAMS:
        assert state.observe_stream(topic, 100, now=10.0) == "advanced"
    assert state.issues(now=10.5) == ()

    assert state.observe_clock(100, now=10.75) == "duplicate"
    assert "/clock stale" in state.issues(now=11.01)
    assert state.observe_clock(99, now=11.02) == "reversed"

    state = diagnostics.SimulationEvidenceState()
    assert state.observe_clock(100, now=20.0) == "advanced"
    topic = diagnostics.REQUIRED_SIMULATION_STREAMS[0]
    assert state.observe_stream(topic, 110, now=20.1) == "future"
    assert state.streams[topic].received_at is None
    assert state.streams[topic].pending_window_started_at == 20.1
    assert state.observe_clock(110, now=20.5) == "advanced"
    assert state.streams[topic].stamp_ns == 110
    assert state.streams[topic].received_at == 20.1
    assert state.streams[topic].pending_stamp_ns is None
    assert state.streams[topic].pending_received_at is None
    assert state.streams[topic].pending_window_started_at is None


def test_sustained_future_samples_cannot_renew_pending_grace():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_future_window")
    state = diagnostics.SimulationEvidenceState()
    state.observe_clock(0, now=0.0)
    for topic in diagnostics.REQUIRED_SIMULATION_STREAMS:
        state.observe_stream(topic, 0, now=0.0)

    contacts = diagnostics.CONTACTS_TOPIC
    state.observe_clock(100, now=0.1)
    for topic in diagnostics.REQUIRED_SIMULATION_STREAMS:
        stamp = 1_000_000_000_100 if topic == contacts else 100
        state.observe_stream(topic, stamp, now=0.1)
    state.observe_clock(1_200, now=1.2)
    for topic in diagnostics.REQUIRED_SIMULATION_STREAMS:
        stamp = 1_000_000_001_200 if topic == contacts else 1_200
        state.observe_stream(topic, stamp, now=1.2)

    contact_state = state.streams[contacts]
    assert contact_state.pending_window_started_at == 0.1
    assert contact_state.pending_received_at == 1.2
    assert f"{contacts} stale" in state.issues(now=1.2, allow_pending=True)


def test_expired_future_window_cannot_reopen_after_clock_catch():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_expired_window")
    state = diagnostics.SimulationEvidenceState()
    contacts = diagnostics.CONTACTS_TOPIC
    state.observe_clock(100, now=0.0)
    state.observe_stream(contacts, 100, now=0.0)

    assert state.observe_stream(contacts, 200, now=0.1) == "future"
    assert state.observe_stream(contacts, 200, now=0.5) == "duplicate"
    assert state.observe_stream(contacts, 150, now=0.6) == "reversed"
    assert state.streams[contacts].pending_received_at == 0.1
    assert state.streams[contacts].pending_window_started_at == 0.1

    assert state.observe_clock(200, now=1.2) == "advanced"
    assert state.streams[contacts].stamp_ns == 100
    assert state.streams[contacts].pending_stamp_ns is None
    assert state.streams[contacts].pending_received_at is None
    assert state.streams[contacts].pending_window_started_at == 0.1

    assert state.observe_stream(contacts, 300, now=1.21) == "future"
    assert state.streams[contacts].pending_window_started_at == 0.1
    assert f"{contacts} stale" in state.issues(now=1.21, allow_pending=True)

    assert state.observe_clock(400, now=1.22) == "advanced"
    assert state.streams[contacts].stamp_ns == 100
    assert state.streams[contacts].pending_stamp_ns is None
    assert state.streams[contacts].pending_received_at is None
    assert state.streams[contacts].pending_window_started_at == 0.1
    assert state.observe_stream(contacts, 400, now=1.23) == "advanced"
    assert state.streams[contacts].pending_window_started_at is None


def test_motion_phase_detection_uses_active_pending_or_navigating():
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_motion")

    def status(*, active="false", pending="false", state="WAITING_READY"):
        message = DiagnosticArray()
        item = DiagnosticStatus(name="amr_exploration/frontier_explorer")
        item.values = [
            KeyValue(key="active", value=active),
            KeyValue(key="pending", value=pending),
            KeyValue(key="state", value=state),
        ]
        message.status = [item]
        return message

    assert not diagnostics._frontier_motion_phase(status())
    assert diagnostics._frontier_motion_phase(status(active="true"))
    assert diagnostics._frontier_motion_phase(status(pending="true"))
    assert diagnostics._frontier_motion_phase(status(state="NAVIGATING"))


def test_first_fault_sidecar_is_structured_and_durable_after_raw_record(tmp_path):
    diagnostics = _load(SCRIPT, "aws_exploration_diagnostics_store")
    output = tmp_path / "evidence" / "exploration_diagnostics.jsonl"
    store = diagnostics.EvidenceStore(output)
    raw = {"record_type": "raw_message", "topic": "/clock", "message": {"clock": {"sec": 1}}}
    fault = {
        "record_type": "first_fault",
        "classification": "EVIDENCE_FAULT",
        "reason": "evidence loss: /clock stale",
    }
    store.append(raw)
    store.write_first_fault(fault)
    store.close()

    assert json.loads(output.read_text().splitlines()[0]) == raw
    saved_fault = json.loads(store.first_fault_path.read_text())
    assert saved_fault == fault
