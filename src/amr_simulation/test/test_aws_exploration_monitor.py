import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


SCRIPT = Path(__file__).parents[1] / "scripts" / "aws_exploration_monitor.py"
DIAGNOSTICS_SCRIPT = Path(__file__).parents[1] / "scripts" / "aws_exploration_diagnostics.py"


def _load():
    spec = importlib.util.spec_from_file_location("aws_exploration_monitor", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_diagnostics():
    spec = importlib.util.spec_from_file_location(
        "aws_exploration_diagnostics", DIAGNOSTICS_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _status(
        state="COMPLETE", *, include_motion_proof=True,
        reason="finished", include_terminal_fields=True):
    status = DiagnosticStatus()
    status.name = "amr_exploration/frontier_explorer"
    status.message = reason
    values = {
        "state": state,
        "active": "false",
        "pending": "false",
        "motion_stopped": "true",
        "fault_latched": "true" if state == "FAULT" else "false",
        "cancel_target": "",
        "reason": reason,
    }
    if include_terminal_fields:
        values.update({
            "run_generation": "1",
            "motion_generation": "1",
            "map_version": "3",
            "goal_failures": "0",
            "no_frontier_updates_seen": "0",
            "reached_goal_count": "1",
            "completion_policy": "SAFE_REACHABLE_AREA_V1",
            "raw_frontier_count": "1" if state == "INCOMPLETE" else "0",
            "blocked_frontier_count": "0",
            "blocked_safety_count": "0",
            "blocked_route_count": "0",
            "unresolved_frontier_count": "1" if state == "INCOMPLETE" else "0",
            "blocked_count": "0",
            "retry_exhausted_count": "0",
            "mission_goal_uuid": "",
            "mission_stage": "TERMINAL",
            "mission_outcome": {
                "COMPLETE": "SUCCEEDED",
                "INCOMPLETE": "ABORTED",
                "FAULT": "FAULT",
                }.get(state, ""),
            "mission_reason": reason,
            "mission_fault_class": "NONE" if state != "FAULT" else "NAVIGATION_FAULT",
        })
    if include_motion_proof:
        values["cancel_owned_motion"] = "false"
    for key, value in values.items():
        item = KeyValue()
        item.key = key
        item.value = value
        status.values.append(item)
    return status


def test_terminal_evidence_requires_explicit_motion_proof():
    module = _load()
    assert module.terminal_evidence(_status("SCANNING")) is None
    assert module.terminal_evidence(
        _status("COMPLETE", include_motion_proof=False)) is None
    assert module.terminal_evidence(
        _status("COMPLETE", include_terminal_fields=False)) is None
    missing_target = _status("COMPLETE")
    missing_target.values = [
        item for item in missing_target.values if item.key != "cancel_target"]
    assert module.terminal_evidence(missing_target) is None
    verdict = module.terminal_evidence(_status("COMPLETE"), received_wall_time=4.5)
    assert verdict["state"] == "COMPLETE"
    assert verdict["active"] is False
    assert verdict["pending"] is False
    assert verdict["cancel_owned_motion"] is False
    assert verdict["motion_stopped"] is True
    assert verdict["reached_goal_count"] == 1
    assert verdict["completion_policy"] == "SAFE_REACHABLE_AREA_V1"
    assert verdict["mission_outcome"] == "SUCCEEDED"
    assert verdict["received_wall_time"] == 4.5


def test_incomplete_requires_an_explicit_safe_explanation():
    module = _load()
    assert module.terminal_evidence(_status("INCOMPLETE")) is None
    verdict = module.terminal_evidence(
        _status(
            "INCOMPLETE",
            reason="exploration incomplete: no safe costmap-valid frontier remains"))
    assert verdict["state"] == "INCOMPLETE"
    assert verdict["incomplete_explanation"].startswith("exploration incomplete:")


def test_complete_accepts_classified_blocked_frontiers():
    module = _load()
    status = _status("COMPLETE")
    for item in status.values:
        if item.key == "raw_frontier_count":
            item.value = "3"
        elif item.key == "blocked_frontier_count":
            item.value = "3"
        elif item.key == "blocked_safety_count":
            item.value = "2"
        elif item.key == "blocked_route_count":
            item.value = "1"
    verdict = module.terminal_evidence(status)
    assert verdict["raw_frontier_count"] == 3
    assert verdict["blocked_frontier_count"] == 3
    assert verdict["blocked_safety_count"] == 2
    assert verdict["blocked_route_count"] == 1


def test_complete_rejects_unclassified_raw_frontier():
    module = _load()
    status = _status("COMPLETE")
    for item in status.values:
        if item.key == "raw_frontier_count":
            item.value = "1"
    assert module.terminal_evidence(status) is None


def test_atomic_verdict_write_replaces_one_file(tmp_path):
    module = _load()
    path = tmp_path / "verdict.json"
    module.write_json_atomic(path, {"state": "COMPLETE", "active": False})
    assert json.loads(path.read_text(encoding="utf-8")) == {
        "state": "COMPLETE", "active": False}
    assert list(tmp_path.glob("*.tmp")) == []


def test_diagnostics_normalizes_humble_byte_status_level():
    module = _load_diagnostics()
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=3, nanosec=4)),
        status=[SimpleNamespace(
            name="amr_exploration/frontier_explorer",
            level=b"\x02",
            message="observed",
            values=[],
        )],
    )
    record = module._record(message)
    assert record["ros_stamp_ns"] == 3_000_000_004
    assert record["statuses"][0]["level"] == 2
