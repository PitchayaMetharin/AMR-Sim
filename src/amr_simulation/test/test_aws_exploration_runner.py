from __future__ import annotations

import json
from pathlib import Path
import signal
import sys
import time

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "aws_exploration_runner.py"


def _load():
    import importlib.util

    spec = importlib.util.spec_from_file_location("aws_exploration_runner", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _sleep_command(seconds: float = 10.0):
    return (sys.executable, "-c", f"import time; time.sleep({seconds})")


def _terminal_evidence(state="COMPLETE"):
    reason = (
        "exploration incomplete: no safe costmap-valid frontier remains"
        if state == "INCOMPLETE" else "finished")
    return {
        "state": state,
        "active": False,
        "pending": False,
        "cancel_owned_motion": False,
        "motion_stopped": True,
        "fault_latched": state == "FAULT",
        "cancel_target": "",
        "reason": reason,
        "run_generation": 1,
        "motion_generation": 1,
        "map_version": 3,
        "goal_failures": 0,
        "no_frontier_updates_seen": 0,
        "reached_goal_count": 1,
        "completion_policy": "SAFE_REACHABLE_AREA_V1",
        "raw_frontier_count": 1 if state == "INCOMPLETE" else 0,
        "blocked_frontier_count": 0,
        "blocked_safety_count": 0,
        "blocked_route_count": 0,
        "unresolved_frontier_count": 1 if state == "INCOMPLETE" else 0,
        "blocked_count": 1 if state == "INCOMPLETE" else 0,
        "retry_exhausted_count": 1 if state == "INCOMPLETE" else 0,
        "mission_goal_uuid": "",
        "mission_stage": "TERMINAL",
        "mission_outcome": {
            "COMPLETE": "SUCCEEDED",
            "INCOMPLETE": "ABORTED",
            "FAULT": "FAULT",
        }[state],
        "mission_reason": reason,
        "mission_fault_class": "NONE" if state != "FAULT" else "NAVIGATION_FAULT",
        "incomplete_acknowledgement": "",
        "incomplete_explanation": reason,
    }


def _terminal_command(
        path: Path, state: str = "COMPLETE", *, stubborn: bool = False,
        exit_after_write: bool = False):
    evidence = _terminal_evidence(state)
    code = (
        "import json, pathlib, signal, time; time.sleep(0.2); "
        f"pathlib.Path({str(path)!r}).write_text(json.dumps({evidence!r})); "
    )
    if stubborn:
        code += "signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); "
    if not exit_after_write:
        code += "time.sleep(30)"
    return (sys.executable, "-c", code)


def _map_saver_command(prefix: Path, *, write_image=True):
    image = Path(f"{prefix}.pgm")
    yaml = Path(f"{prefix}.yaml")
    code = (
        "from pathlib import Path; "
        f"p=Path({str(prefix)!r}); p.parent.mkdir(parents=True, exist_ok=True); "
        f"Path({str(yaml)!r}).write_text('image: {image.name}\\nresolution: 0.05\\n' "
        "'origin: [0.0, 0.0, 0.0]\\nnegate: 0\\n' "
        "'occupied_thresh: 0.65\\nfree_thresh: 0.196\\n'); "
    )
    if write_image:
        code += f"Path({str(image)!r}).write_bytes(b'P5\\n1 1\\n255\\n\\x00'); "
    code += "raise SystemExit(0)"
    return (sys.executable, "-c", code)


def _config(module, root: Path, *, state="COMPLETE", stubborn=False, **overrides):
    run_dir = root / overrides.pop("run_name", "run")
    verdict = run_dir / "verdict.json"
    values = dict(
        run_dir=run_dir,
        run_id=run_dir.name,
        ros_domain_id=10,
        recorder=_sleep_command(),
        monitor=_terminal_command(verdict, state, stubborn=stubborn),
        diagnostics=_sleep_command(),
        launch=_sleep_command(),
        cwd=Path.cwd(),
        verdict_file=verdict,
        graceful_shutdown_sec=0.15,
        escalation_sec=0.15,
        poll_interval_sec=0.01,
    )
    values.update(overrides)
    return module.RunConfig(**values)


def test_terminal_classification_requires_safe_motion_proof(tmp_path):
    module = _load()
    safe = module.classify_terminal_evidence(_terminal_evidence("COMPLETE"))
    unsafe_evidence = _terminal_evidence("COMPLETE")
    unsafe_evidence["motion_stopped"] = False
    unsafe = module.classify_terminal_evidence(unsafe_evidence)
    missing = module.classify_terminal_evidence({"state": "COMPLETE"})

    assert safe["classification"] == "COMPLETE"
    assert safe["pass"]
    assert unsafe["classification"] == "FAULT"
    assert not unsafe["pass"]
    assert missing["classification"] == "FAULT"


def test_terminal_classification_accepts_blocked_frontier_completion():
    module = _load()
    evidence = _terminal_evidence("COMPLETE")
    evidence.update({
        "raw_frontier_count": 4,
        "blocked_frontier_count": 4,
        "blocked_safety_count": 3,
        "blocked_route_count": 1,
    })
    result = module.classify_terminal_evidence(evidence)

    assert result["classification"] == "COMPLETE"
    assert result["pass"] is True


def test_terminal_classification_rejects_unclassified_frontier_completion():
    module = _load()
    evidence = _terminal_evidence("COMPLETE")
    evidence["raw_frontier_count"] = 1
    result = module.classify_terminal_evidence(evidence)

    assert result["classification"] == "FAULT"
    assert not result["pass"]


def test_explained_incomplete_is_an_operational_pass():
    module = _load()
    result = module.classify_terminal_evidence(_terminal_evidence("INCOMPLETE"))

    assert result["classification"] == "INCOMPLETE"
    assert result["pass"] is True


def test_incomplete_without_explanation_fails_closed():
    module = _load()
    evidence = _terminal_evidence("INCOMPLETE")
    evidence["reason"] = "stopped"
    evidence["mission_reason"] = "stopped"
    evidence["incomplete_explanation"] = ""
    result = module.classify_terminal_evidence(evidence)

    assert result["classification"] == "FAULT"
    assert not result["pass"]


@pytest.mark.parametrize(
    ("state", "expected", "passed"),
    (("COMPLETE", "COMPLETE", True), ("INCOMPLETE", "INCOMPLETE", True), ("FAULT", "FAULT", False)),
)
def test_runner_records_terminal_classification(tmp_path, state, expected, passed):
    module = _load()
    result = module.run_aws_exploration(
        _config(module, tmp_path, state=state, run_name=f"terminal_{state.lower()}")
    )

    assert result["classification"] == expected
    assert result["pass"] is passed
    assert result["cleanup"]["owned_processes_exited"]
    assert (Path(result["processes"]["monitor"]["log"])).is_file()


def test_atomic_terminal_evidence_wins_if_monitor_exits_immediately(tmp_path):
    module = _load()
    run_dir = tmp_path / "immediate_terminal"
    verdict = run_dir / "verdict.json"
    result = module.run_aws_exploration(
        _config(
            module,
            tmp_path,
            run_name="immediate_terminal",
            monitor=_terminal_command(verdict, exit_after_write=True),
        )
    )

    assert result["classification"] == "COMPLETE"
    assert result["pass"] is True
    assert result["first_failure"] is None


def test_recorder_failure_cannot_be_mistaken_for_terminal_acceptance(tmp_path):
    module = _load()
    recorder = (sys.executable, "-c", "import time; time.sleep(0.05); raise SystemExit(7)")
    result = module.run_aws_exploration(
        _config(
            module,
            tmp_path,
            run_name="recorder_failure",
            recorder=recorder,
        )
    )

    assert result["classification"] == "RECORDER_FAILURE"
    assert not result["pass"]
    assert result["first_failure"]["component"] == "recorder"
    assert result["cleanup"]["owned_processes_exited"]


def test_monitor_exit_is_observer_failure(tmp_path):
    module = _load()
    monitor = (sys.executable, "-c", "raise SystemExit(4)")
    result = module.run_aws_exploration(
        _config(module, tmp_path, run_name="monitor_failure", monitor=monitor)
    )

    assert result["classification"] == "OBSERVER_FAILURE"
    assert result["first_failure"]["component"] == "monitor"
    assert not result["pass"]
    assert result["cleanup"]["owned_processes_exited"]


def test_launch_exit_is_distinct_from_observer_and_recorder_failure(tmp_path):
    module = _load()
    launch = (sys.executable, "-c", "raise SystemExit(3)")
    result = module.run_aws_exploration(
        _config(module, tmp_path, run_name="launch_failure", launch=launch)
    )

    assert result["classification"] == "LAUNCH_EXIT"
    assert result["first_failure"]["component"] == "launch"
    assert not result["pass"]


def test_abort_log_is_a_first_fault(tmp_path):
    module = _load()
    launch = (
        sys.executable,
        "-c",
        "import time; print('Smoothing failed: path rejected', flush=True); time.sleep(10)",
    )
    result = module.run_aws_exploration(
        _config(module, tmp_path, run_name="abort_log", launch=launch)
    )

    assert result["classification"] == "FAULT"
    assert result["first_failure"]["component"] == "launch"
    assert "contradicted structured terminal" in result["first_failure"]["reason"]


def test_safe_terminal_saves_and_verifies_map_before_cleanup(tmp_path):
    module = _load()
    run_dir = tmp_path / "map_save"
    prefix = run_dir / "evidence" / "aws_map"
    result = module.run_aws_exploration(
        _config(
            module,
            tmp_path,
            run_name="map_save",
            map_saver=_map_saver_command(prefix),
            map_output_prefix=prefix,
        )
    )

    assert result["pass"] is True
    assert result["map_save"]["attempted"] is True
    assert result["map_save"]["verified"] is True
    assert Path(result["map_save"]["verification"]["yaml"]).is_file()
    assert Path(result["map_save"]["verification"]["image"]).is_file()


def test_map_save_verification_failure_is_not_acceptance(tmp_path):
    module = _load()
    run_dir = tmp_path / "map_save_missing_image"
    prefix = run_dir / "evidence" / "aws_map"
    result = module.run_aws_exploration(
        _config(
            module,
            tmp_path,
            run_name="map_save_missing_image",
            map_saver=_map_saver_command(prefix, write_image=False),
            map_output_prefix=prefix,
        )
    )

    assert result["classification"] == "FAULT"
    assert result["first_failure"]["component"] == "map_saver"
    assert result["map_save"]["verified"] is False


def test_surviving_descendant_is_cleaned_from_owned_session(tmp_path):
    module = _load()
    child_code = "import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"
    launch_code = (
        "import subprocess, sys, time; "
        f"subprocess.Popen([sys.executable, '-c', {child_code!r}]); "
        "time.sleep(30)"
    )
    launch = (sys.executable, "-c", launch_code)
    result = module.run_aws_exploration(
        _config(module, tmp_path, run_name="descendant", launch=launch)
    )

    assert result["cleanup"]["owned_processes_exited"]
    assert result["cleanup"]["survivors"] == []
    assert result["cleanup"]["escalated"]
    assert result["classification"] == "FAULT"


def test_cleanup_escalates_and_fails_closed(tmp_path):
    module = _load()
    result = module.run_aws_exploration(
        _config(module, tmp_path, run_name="cleanup_escalation", stubborn=True)
    )

    assert result["cleanup"]["escalated"]
    assert result["cleanup"]["owned_processes_exited"]
    assert result["classification"] == "FAULT"
    assert not result["pass"]


def test_run_identity_and_domain_validation_reject_reuse(tmp_path):
    module = _load()
    with pytest.raises(module.RunnerError):
        module.validate_ros_domain_id(-1)
    with pytest.raises(module.RunnerError):
        module.validate_ros_domain_id(256)
    with pytest.raises(module.RunnerError):
        module.validate_run_identity(Path("relative/run"))
    with pytest.raises(module.RunnerError):
        module.validate_run_identity(tmp_path / "bad name")

    existing = tmp_path / "existing"
    existing.mkdir()
    with pytest.raises(module.RunnerError):
        module.validate_run_identity(existing)


def test_live_bag_inspection_is_rejected_until_all_owned_processes_exit(tmp_path):
    module = _load()
    gate = module.BagInspectionGate()
    with pytest.raises(module.LiveBagInspectionError):
        gate.inspect(
            (sys.executable, "-c", "raise SystemExit(0)"),
            cwd=tmp_path,
            environment={"ROS_DOMAIN_ID": "10"},
            output_path=tmp_path / "inspection.log",
        )

    gate.mark_stopped(recorder_exited=True, owned_processes_exited=True)
    completed = gate.inspect(
        (sys.executable, "-c", "raise SystemExit(0)"),
        cwd=tmp_path,
        environment={"ROS_DOMAIN_ID": "10"},
        output_path=tmp_path / "inspection.log",
    )
    assert completed.returncode == 0
    assert (tmp_path / "inspection.log").is_file()
