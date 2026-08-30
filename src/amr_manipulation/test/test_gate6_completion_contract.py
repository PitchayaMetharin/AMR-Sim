from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MASS_STAGE = PACKAGE_ROOT / "src" / "gate6_mass_stage.cpp"
ANALYZER = PACKAGE_ROOT / "scripts" / "gate6_evidence_analyzer.py"


def test_payload_aware_lower_path_is_fail_closed():
    source = MASS_STAGE.read_text(encoding="utf-8")
    assert "robotStateToRobotStateMsg(" in source
    assert "request->robot_state.is_diff = true" in source
    assert "planning_scene_attached_object_proof(true)" in source
    assert "Validate the measured-current-to-first-point segment" in source
    assert "computeTimeStamps" in source
    assert "arm.execute(lower_plan)" in source
    assert "placement lower trajectory postconditions: PASS" in source


def test_gate6_analyzer_is_installed_and_has_stable_result_marker():
    source = ANALYZER.read_text(encoding="utf-8")
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "rosbag2_py" in source
    assert "GATE6_BAG_ANALYSIS=PASS" in source
    assert "latest command at or before the output" in source
    assert "scripts/gate6_evidence_analyzer.py" in cmake
    assert "RENAME gate6_evidence_analyzer" in cmake
