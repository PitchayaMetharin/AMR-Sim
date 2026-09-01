import importlib.util
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MASS_STAGE = PACKAGE_ROOT / "src" / "gate6_mass_stage.cpp"
ANALYZER = PACKAGE_ROOT / "scripts" / "gate6_evidence_analyzer.py"
ANALYZER_SPEC = importlib.util.spec_from_file_location("gate6_evidence_analyzer", ANALYZER)
ANALYZER_MODULE = importlib.util.module_from_spec(ANALYZER_SPEC)
ANALYZER_SPEC.loader.exec_module(ANALYZER_MODULE)


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
    assert "command_trace_matches" in source
    assert "COMMAND_FORWARDING_MAX_AGE_SECONDS = 0.25" in source
    assert "scripts/gate6_evidence_analyzer.py" in cmake
    assert "RENAME gate6_evidence_analyzer" in cmake


def test_stage_selector_chooses_mass_stage_after_product_preparation():
    statuses = [
        (1.0, SimpleNamespace(
            source_boot_id=11, detail="Gate 6 product preparation is starting")),
        (1.5, SimpleNamespace(
            source_boot_id=11, detail="Product 102 prepared at pickup dock")),
        (2.0, SimpleNamespace(
            source_boot_id=22, detail=ANALYZER_MODULE.STAGE_START_MARKER)),
        (2.5, SimpleNamespace(source_boot_id=22, detail="stage complete")),
    ]

    selected = ANALYZER_MODULE.select_stage_status_stream(statuses)

    assert selected is not None
    assert selected[0] == 22
    assert [timestamp for timestamp, _ in selected[1]] == [2.0, 2.5]


def test_stage_selector_keeps_single_source_product_101_compatible():
    selected = ANALYZER_MODULE.select_stage_status_stream([
        (3.0, SimpleNamespace(source_boot_id=101, detail="stage status")),
    ])

    assert selected is not None
    assert selected[0] == 101


def test_stage_selector_fails_closed_for_empty_or_ambiguous_sources():
    assert ANALYZER_MODULE.select_stage_status_stream([]) is None
    assert ANALYZER_MODULE.select_stage_status_stream([
        (1.0, SimpleNamespace(source_boot_id=11, detail="preparation")),
        (2.0, SimpleNamespace(source_boot_id=22, detail="stage status")),
    ]) is None
    assert ANALYZER_MODULE.select_stage_status_stream([
        (1.0, SimpleNamespace(source_boot_id=11, detail=ANALYZER_MODULE.STAGE_START_MARKER)),
        (2.0, SimpleNamespace(source_boot_id=22, detail=ANALYZER_MODULE.STAGE_START_MARKER)),
    ]) is None


def test_command_trace_accepts_one_adapter_tick_of_forwarding_latency():
    control = [
        (1.00, 0.00, 0.00),
        (1.05, 0.10, 0.00),
        (1.10, 0.20, 0.00),
    ]
    simulation = [
        (1.05, 0.00, 0.00),
        (1.10, 0.10, 0.00),
    ]
    assert ANALYZER_MODULE.command_trace_matches(control, simulation)


def test_command_trace_rejects_unowned_or_stale_forwarding():
    control = [(1.00, 0.10, 0.00)]
    assert not ANALYZER_MODULE.command_trace_matches(
        control, [(1.10, 0.20, 0.00)])
    assert not ANALYZER_MODULE.command_trace_matches(
        control, [(1.26, 0.10, 0.00)])
