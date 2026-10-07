import gzip
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MASS_STAGE = PACKAGE_ROOT / "src" / "gate6_mass_stage.cpp"
ANALYZER = PACKAGE_ROOT / "scripts" / "gate6_evidence_analyzer.py"
ANALYZER_SPEC = importlib.util.spec_from_file_location("gate6_evidence_analyzer", ANALYZER)
ANALYZER_MODULE = importlib.util.module_from_spec(ANALYZER_SPEC)
ANALYZER_SPEC.loader.exec_module(ANALYZER_MODULE)


def _status(source_boot_id, detail, state=0, valid=True, product_attached=False,
            product_id="", base_motion_allowed=False):
    return SimpleNamespace(
        source_boot_id=source_boot_id,
        detail=detail,
        state=state,
        valid=valid,
        base_motion_allowed=base_motion_allowed,
        product_attached=product_attached,
        product_id=str(product_id),
    )


def _wire_status(message):
    return ANALYZER_MODULE._raw_status_fingerprint(message)


def _ownership_fixture():
    product_id = 102
    public_boot = 44
    child_boot = 77
    execution_uuid = "0123456789abcdef0123456789abcdef"

    def status(boot, sequence, detail, state, attached=False, product="", base=False):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=100, nanosec=sequence), frame_id="map"),
            source_boot_id=boot, sequence=sequence, valid=True, state=state,
            base_motion_allowed=base, product_attached=attached,
            product_id=product, detail=detail)

    public = [
        (100_000_000, status(public_boot, 1, ANALYZER_MODULE.STAGE_START_MARKER, 0)),
        (200_000_000, status(public_boot, 2, "Product 101 loaded", 2, True, "101", True)),
        (300_000_000, status(public_boot, 3, "Product 101 empty", 1, False, "", True)),
        (1_000_000_000, status(public_boot, 10, ANALYZER_MODULE.STAGE_START_MARKER, 0)),
        (1_100_000_000, status(public_boot, 11, ANALYZER_MODULE.OWNERSHIP_STALE_DETAIL, 0)),
        (1_200_000_000, status(public_boot, 12, ANALYZER_MODULE.STAGE_START_MARKER, 0)),
        (2_000_000_000, status(public_boot, 13, "loaded", 2, True, "102", True)),
        (3_000_000_000, status(public_boot, 14, "empty", 1, False, "", True)),
    ]
    public.extend(
        (3_100_000_000 + sequence * 10_000_000,
         status(public_boot, sequence, "cycle complete", 1, False, "", True))
        for sequence in range(15, 21))
    child_statuses = [
        status(child_boot, 1, ANALYZER_MODULE.STAGE_START_MARKER, 0),
        status(child_boot, 2, "loaded", 2, True, "102", True),
        status(child_boot, 3, "empty", 1, False, "", True),
    ]
    records = []
    for index, (event, public_row, child_row) in enumerate(zip(
            ("START", "LOADED", "EMPTY"),
            (public[3][1], public[6][1], public[7][1]), child_statuses), start=1):
        records.append({
            "schema": "AMR_CYCLE_OWNERSHIP_V1", "version": 1, "event": event,
            "record_index": index, "execution_uuid": execution_uuid,
            "product_id": "102", "public_source_boot_id": public_boot,
            "captured_monotonic_s": 10.01 * index,
            "public": _wire_status(public_row), "child": _wire_status(child_row),
            "child_received_monotonic_s": 10.0 * index,
            "child_age_s": 10.01 * index - 10.0 * index,
            "owned_child_boot_id": child_boot,
            "owned_child_sequence": child_row.sequence, "child_consistent": True,
            "authority_open": True, "stage_started": True,
            "stage_loaded_proof": index >= 2, "terminal_empty_proof": index == 3,
            "fault_latched": False, "cancel_requested": False,
            "observed_history_violations": [],
        })
    close = {
        "schema": "AMR_CYCLE_OWNERSHIP_V1", "version": 1, "event": "CLOSE",
        "record_index": 4, "execution_uuid": execution_uuid, "product_id": "102",
        "public_source_boot_id": public_boot, "captured_monotonic_s": 41.0,
        "public": None, "child": _wire_status(child_statuses[-1]),
        "child_received_monotonic_s": 30.0, "child_age_s": 11.0,
        "owned_child_boot_id": child_boot, "owned_child_sequence": 3,
        "child_consistent": True, "authority_open": False, "stage_started": True,
        "stage_loaded_proof": True, "terminal_empty_proof": True,
        "fault_latched": False, "cancel_requested": False,
        "observed_history_violations": [], "active_owner_match": True,
        "goal_reserved": True, "public_sequence_high_water": 20,
        "current_state": 1, "current_detail": "cycle complete",
        "current_base_motion_allowed": True, "current_product_attached": False,
        "current_product_id": "", "child_exit_code": 0, "child_alive": False,
        "callback_child_present": True, "child_reference_consistent": True,
        "result_product_id": "102", "result_outcome": 0, "result_delivered": True,
        "first_boundaries": {
            event: {"record_index": index, "source_boot_id": public_boot,
                    "sequence": sequence}
            for event, index, sequence in (("START", 1, 10), ("LOADED", 2, 13),
                                           ("EMPTY", 3, 14))
        },
        "captured_record_count": 4,
    }
    records.append(close)
    rosout = [
        (4_000_000_000 + index, ANALYZER_MODULE.OWNERSHIP_LOGGER,
         ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX + json.dumps(record, separators=(",", ":")))
        for index, record in enumerate(records)
    ]
    internal = [
        (1_010_000_000, child_statuses[0]),
        (2_010_000_000, child_statuses[1]),
        (2_990_000_000, child_statuses[2]),
    ]

    def action_entry(status_code):
        goal_info = SimpleNamespace(
            goal_id=SimpleNamespace(uuid=list(bytes.fromhex(execution_uuid))),
            stamp=SimpleNamespace(sec=5, nanosec=7))
        return SimpleNamespace(goal_info=goal_info, status=status_code)

    action_status = [
        (900_000_000, SimpleNamespace(status_list=[action_entry(1)])),
        (950_000_000, SimpleNamespace(status_list=[action_entry(2)])),
        (3_100_000_000, SimpleNamespace(status_list=[action_entry(4)])),
    ]
    feedback = SimpleNamespace(
        goal_id=SimpleNamespace(uuid=list(bytes.fromhex(execution_uuid))),
        feedback=SimpleNamespace(phase=2, phase_name="EXECUTING", product_id="102",
                                 product_attached=False))
    feedback_rows = [(1_500_000_000, feedback)]
    return public, internal, action_status, feedback_rows, rosout


def test_payload_aware_lower_path_is_fail_closed():
    source = MASS_STAGE.read_text(encoding="utf-8")
    assert "robotStateToRobotStateMsg(" in source
    assert "request->robot_state.is_diff = true" in source
    assert "planning_scene_attached_object_proof(true)" in source
    assert "Validate the measured-current-to-first-point segment" in source
    assert "computeTimeStamps" in source
    assert "arm.execute(lower_plan)" in source
    assert "placement lower trajectory postconditions: PASS" in source


def test_delivery_completion_requires_empty_dispatch_clearance_and_slot_proof():
    source = MASS_STAGE.read_text(encoding="utf-8")
    tail = source[source.index('throw std::runtime_error("empty stow tolerance was not achieved")'):]
    assert tail.index('native_attachment_state_is("detached")') < tail.index(
        "navigate_empty_dispatch_clearance(120s)")
    assert tail.index("navigate_empty_dispatch_clearance(120s)") < tail.index(
        "fresh detached dispatch-slot proof failed after clearance") < tail.index("passed = true;")


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
        (1.0, _status(11, "Gate 6 product preparation is starting")),
        (1.5, _status(11, "Product 102 prepared at pickup dock")),
        (2.0, _status(22, ANALYZER_MODULE.STAGE_START_MARKER)),
        (2.5, _status(22, "stage complete", state=2, product_attached=True, product_id=102)),
        (3.0, _status(22, "stage complete", state=1)),
    ]

    selected = ANALYZER_MODULE.select_stage_status_stream(statuses, 102)

    assert selected is not None
    assert selected[0] == 22
    assert [timestamp for timestamp, _ in selected[1]] == [2.0, 2.5, 3.0]


def test_stage_selector_keeps_single_source_product_101_compatible():
    selected = ANALYZER_MODULE.select_stage_status_stream([
        (3.0, _status(101, ANALYZER_MODULE.STAGE_START_MARKER)),
        (3.5, _status(101, "stage loaded", state=2, product_attached=True, product_id=101)),
        (4.0, _status(101, "stage terminal", state=1)),
    ], 101)

    assert selected is not None
    assert selected[0] == 101
    assert [timestamp for timestamp, _ in selected[1]] == [3.0, 3.5, 4.0]


def test_stage_selector_starts_at_mass_stage_marker_in_shared_source():
    selected = ANALYZER_MODULE.select_stage_status_stream([
        (1.0, _status(11, "preparation")),
        (2.0, _status(11, ANALYZER_MODULE.STAGE_START_MARKER)),
        (2.5, _status(11, "stage loaded", state=2, product_attached=True, product_id=101)),
        (3.0, _status(11, "stage terminal", state=1)),
    ], 101)

    assert selected is not None
    assert [timestamp for timestamp, _ in selected[1]] == [2.0, 2.5, 3.0]


def test_stage_selector_uses_first_product_terminal_boundary_with_shared_source():
    source_boot_id = 1566243568
    statuses = [
        (1.0, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (1.05, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (1.10, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (1.5, _status(source_boot_id, "Product 101 loaded", state=2,
                      product_attached=True, product_id=101)),
        (2.0, _status(source_boot_id, "Product 101 empty stow", state=1)),
        (3.0, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (3.05, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (3.10, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (3.5, _status(source_boot_id, "Product 102 loaded", state=2,
                      product_attached=True, product_id=102)),
        (4.0, _status(source_boot_id, "Product 102 empty stow", state=1)),
    ]

    selected = ANALYZER_MODULE.select_stage_status_stream(statuses, 101)

    assert selected is not None
    assert selected[0] == source_boot_id
    assert selected[1][0][0] == 1.0
    assert selected[1][-1][0] == 2.0

    selected = ANALYZER_MODULE.select_stage_status_stream(statuses, 102)

    assert selected is not None
    assert selected[0] == source_boot_id
    assert selected[1][0][0] == 3.0
    assert selected[1][-1][0] == 4.0


def test_stage_selector_fails_closed_when_product_candidate_has_no_terminal():
    assert ANALYZER_MODULE.select_stage_status_stream([
        (1.0, _status(11, ANALYZER_MODULE.STAGE_START_MARKER)),
        (1.5, _status(11, "Product 101 loaded", state=2,
                      product_attached=True, product_id=101)),
    ], 101) is None


def test_stage_selector_fails_closed_for_ambiguous_product_candidates():
    source_boot_id = 11
    assert ANALYZER_MODULE.select_stage_status_stream([
        (1.0, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (1.5, _status(source_boot_id, "Product 101 loaded", state=2,
                      product_attached=True, product_id=101)),
        (2.0, _status(source_boot_id, "Product 101 empty stow", state=1)),
        (3.0, _status(source_boot_id, ANALYZER_MODULE.STAGE_START_MARKER)),
        (3.5, _status(source_boot_id, "Product 101 loaded", state=2,
                      product_attached=True, product_id=101)),
        (4.0, _status(source_boot_id, "Product 101 empty stow", state=1)),
    ], 101) is None


def test_stage_selector_fails_closed_for_empty_or_ambiguous_sources():
    assert ANALYZER_MODULE.select_stage_status_stream([], 101) is None
    assert ANALYZER_MODULE.select_stage_status_stream([
        (1.0, _status(11, "preparation")),
        (2.0, _status(22, "stage status")),
    ], 101) is None
    assert ANALYZER_MODULE.select_stage_status_stream([
        (1.0, _status(11, ANALYZER_MODULE.STAGE_START_MARKER)),
        (2.0, _status(22, ANALYZER_MODULE.STAGE_START_MARKER)),
    ], 101) is None


def test_corroborated_selector_keeps_earliest_start_and_intermarker_stale_row():
    public, internal, action_status, feedback, rosout = _ownership_fixture()

    selected = ANALYZER_MODULE.select_corroborated_stage_status_stream(
        public, internal, action_status, feedback, rosout, 102)

    assert selected[0] == 44
    assert [message.sequence for _, message in selected[1]] == [10, 11, 12, 13, 14]
    assert selected[1][0][1].detail == ANALYZER_MODULE.STAGE_START_MARKER
    assert selected[1][-1][1].state == 1


@pytest.mark.parametrize("counterexample", [
    "other_marked_boot", "unterminated_marker", "foreign_active_feedback",
])
def test_corroborated_selector_rejects_global_marker_and_feedback_counterexamples(
        counterexample):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    expected_error = {
        "other_marked_boot": "public START markers do not identify exactly one positive source boot",
        "unterminated_marker": "public START marker block lacks a first valid detached EMPTY terminal",
        "foreign_active_feedback": "foreign ExecuteProductCycle feedback is active in the conservative fence",
    }[counterexample]

    if counterexample == "other_marked_boot":
        for _, message in public[:3]:
            message.source_boot_id = 45
    elif counterexample == "unterminated_marker":
        source = public[-1][1]
        marker = SimpleNamespace(**vars(source))
        marker.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=100, nanosec=21), frame_id=source.header.frame_id)
        marker.sequence = 21
        marker.detail = ANALYZER_MODULE.STAGE_START_MARKER
        marker.state = 0
        marker.base_motion_allowed = False
        marker.product_attached = False
        marker.product_id = ""
        public.append((5_000_000_000, marker))
    else:
        foreign = SimpleNamespace(
            goal_id=SimpleNamespace(
                uuid=list(bytes.fromhex("fedcba9876543210fedcba9876543210"))),
            feedback=SimpleNamespace(
                phase=2, phase_name="EXECUTING", product_id="101",
                product_attached=False))
        feedback.append((2_500_000_000, foreign))

    with pytest.raises(ANALYZER_MODULE.AnalysisError, match=expected_error):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_rejects_missing_close_and_raw_action_identity():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="chain is incomplete"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout[:-1], 102)

    public, internal, action_status, feedback, rosout = _ownership_fixture()
    action_status.pop()
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="successful action terminal"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_orders_proofs_by_capture_not_rosout_arrival():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    arrival_receipts = [4_000_000_003, 4_000_000_000,
                        4_000_000_001, 4_000_000_002]
    rosout[:] = [(arrival_receipts[index], logger, text)
                 for index, (_, logger, text) in enumerate(rosout)]

    selected = ANALYZER_MODULE.select_corroborated_stage_status_stream(
        public, internal, action_status, feedback, rosout, 102)

    assert [message.sequence for _, message in selected[1]] == [10, 11, 12, 13, 14]


@pytest.mark.parametrize("missing_sequence", [10, 13, 14])
def test_corroborated_selector_rejects_missing_each_public_boundary(missing_sequence):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    public[:] = [(receipt, message) for receipt, message in public
                 if message.sequence != missing_sequence]

    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


@pytest.mark.parametrize(("fault", "expected"), [
    ("receipt", "public status receipt time rolled back"),
    ("sequence", "public status sequence is not positive and contiguous"),
])
def test_corroborated_selector_rejects_public_receipt_and_sequence_rollback(
        fault, expected):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    if fault == "receipt":
        public[6] = (1_150_000_000, public[6][1])
    else:
        public[6][1].sequence = 11

    with pytest.raises(ANALYZER_MODULE.AnalysisError, match=expected):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_rejects_corrupt_arbitrary_intermarker_public_row():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    public[4][1].detail = "corrupt inter-marker status"

    with pytest.raises(ANALYZER_MODULE.AnalysisError,
                       match="inter-marker public row is not a safe stale diagnostic"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


@pytest.mark.parametrize("missing_event", ["START", "LOADED", "EMPTY", "CLOSE"])
def test_corroborated_selector_rejects_each_missing_ownership_boundary(missing_event):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    rosout[:] = [row for row in rosout
                 if json.loads(row[2][len(ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX):])[
                     "event"] != missing_event]

    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


@pytest.mark.parametrize("missing_item", ["accepted", "executing", "success", "feedback"])
def test_corroborated_selector_requires_action_status_and_feedback(missing_item):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    if missing_item == "feedback":
        feedback.clear()
    else:
        status_code = {"accepted": 1, "executing": 2, "success": 4}[missing_item]
        action_status[:] = [
            (receipt, message) for receipt, message in action_status
            if all(entry.status != status_code for entry in message.status_list)
        ]

    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


@pytest.mark.parametrize("mutation", [
    "duplicate_index", "loaded_uuid", "loaded_product", "loaded_boot",
    "start_fingerprint", "close_reference",
])
def test_corroborated_selector_rejects_conflicting_chain_identity_and_fingerprint(mutation):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    if mutation == "duplicate_index":
        rosout.append(rosout[0])
    else:
        event_index = 1 if mutation.startswith("loaded_") else 0 if mutation == \
            "start_fingerprint" else 3
        receipt, logger, text = rosout[event_index]
        record = json.loads(text[len(ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX):])
        if mutation == "loaded_uuid":
            record["execution_uuid"] = "fedcba9876543210fedcba9876543210"
        elif mutation == "loaded_product":
            record["product_id"] = "101"
        elif mutation == "loaded_boot":
            record["public_source_boot_id"] = 45
        elif mutation == "start_fingerprint":
            record["public"]["sequence"] = 9
        else:
            record["first_boundaries"]["START"]["sequence"] = 9
        rosout[event_index] = (
            receipt, logger, ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX +
            json.dumps(record, separators=(",", ":")))

    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


@pytest.mark.parametrize("mutation", [
    "owner", "reservation", "authority", "exit", "alive", "callback",
    "reference", "result", "child_state", "child_sequence", "child_attachment",
    "child_product", "child_age",
])
def test_corroborated_selector_rejects_unsafe_close_and_stale_boundary(mutation):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    event_index = 1 if mutation == "child_age" else 3
    receipt, logger, text = rosout[event_index]
    record = json.loads(text[len(ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX):])
    close_fields = {
        "owner": ("active_owner_match", False),
        "reservation": ("goal_reserved", False),
        "authority": ("authority_open", True),
        "exit": ("child_exit_code", 1),
        "alive": ("child_alive", True),
        "callback": ("callback_child_present", False),
        "reference": ("child_reference_consistent", False),
        "result": ("result_outcome", 1),
    }
    if mutation in close_fields:
        key, value = close_fields[mutation]
        record[key] = value
    elif mutation == "child_state":
        record["child"]["state"] = 2
    elif mutation == "child_sequence":
        record["owned_child_sequence"] = 2
        record["child"]["sequence"] = 2
    elif mutation == "child_attachment":
        record["child"]["product_attached"] = True
        record["child"]["product_id"] = "102"
    elif mutation == "child_product":
        record["child"]["product_id"] = "102"
    else:
        record["child_age_s"] += 0.01
    rosout[event_index] = (
        receipt, logger, ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX +
        json.dumps(record, separators=(",", ":")))

    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


@pytest.mark.parametrize("raw_fault", ["sequence_rollback", "foreign_stage_claim"])
def test_corroborated_selector_audits_raw_internal_history_through_close(raw_fault):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    if raw_fault == "sequence_rollback":
        message = _status(77, "repeated child status", state=1, base_motion_allowed=True)
        message.sequence = 2
        internal.insert(2, (2_500_000_000, message))
    else:
        message = _status(88, ANALYZER_MODULE.STAGE_START_MARKER, state=0)
        message.sequence = 1
        internal.insert(2, (2_500_000_000, message))

    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_audits_foreign_public_stage_claim_inside_fence():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    foreign = _status(88, "loaded", state=2, product_attached=True,
                      product_id="102", base_motion_allowed=True)
    foreign.sequence = 1
    public.insert(8, (3_200_000_000, foreign))

    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="foreign public stage claim"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_attached_preparation_without_stage_claim_shape_is_not_recognized():
    preparation = _status(
        88, "Product 102 held during preparation", state=3,
        product_attached=True, product_id="102")

    assert not ANALYZER_MODULE._recognizable_stage_claim(preparation)


def test_corroborated_selector_rejects_duplicate_json_keys_and_foreign_logger():
    _, _, _, _, rosout = _ownership_fixture()
    duplicate = (5_000_000_000, ANALYZER_MODULE.OWNERSHIP_LOGGER,
                 ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX +
                 '{"schema":"AMR_CYCLE_OWNERSHIP_V1","schema":"duplicate"}')
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="duplicate JSON key"):
        ANALYZER_MODULE._parse_ownership_records([duplicate])
    wrong_logger = (5_000_000_000, "other.logger", rosout[0][2])
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="unexpected logger"):
        ANALYZER_MODULE._parse_ownership_records([wrong_logger])


@pytest.mark.parametrize("mutation", ["malformed_json", "schema", "types", "uuid", "nonfinite"])
def test_ownership_record_parser_rejects_malformed_schema_and_values(mutation):
    _, _, _, _, rosout = _ownership_fixture()
    receipt, logger, text = rosout[0]
    if mutation == "malformed_json":
        bad_text = ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX + "{"
    else:
        record = json.loads(text[len(ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX):])
        if mutation == "schema":
            record["schema"] = "AMR_CYCLE_OWNERSHIP_V2"
        elif mutation == "types":
            record["record_index"] = "1"
        elif mutation == "uuid":
            record["execution_uuid"] = "not-a-uuid"
        else:
            record["captured_monotonic_s"] = float("nan")
        bad_text = ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX + json.dumps(record)

    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE._parse_ownership_records([(receipt, logger, bad_text)])


def test_corroborated_selector_rejects_missing_or_duplicate_public_high_water():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    public[:] = [row for row in public if row[1].sequence != 18]
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="high-water"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)

    public, internal, action_status, feedback, rosout = _ownership_fixture()
    duplicate = public[-1][1]
    public.append((5_000_000_000, duplicate))
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="high-water"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_audits_late_received_hwm_fault():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    late = public[-1][1]
    late.state = 5
    late.valid = False
    public[-1] = (5_000_000_000, late)

    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="high-water.*invalid"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_rejects_after_loaded_marker_and_second_target_job():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    public.insert(7, (2_500_000_000, public[6][1].__class__(
        **{**vars(public[6][1]), "sequence": 14,
           "detail": ANALYZER_MODULE.STAGE_START_MARKER,
           "state": 0, "product_attached": False, "product_id": "",
           "base_motion_allowed": False})))
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="START marker appeared"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)

    public, internal, action_status, feedback, rosout = _ownership_fixture()
    public.extend([
        (3_400_000_000, _status(44, ANALYZER_MODULE.STAGE_START_MARKER,
                                state=0, base_motion_allowed=False)),
        (3_500_000_000, _status(44, "Product 102 loaded", state=2,
                                valid=True, product_attached=True, product_id="102",
                                base_motion_allowed=True)),
        (3_600_000_000, _status(44, "Product 102 empty", state=1,
                                base_motion_allowed=True)),
    ])
    for sequence, (_, message) in enumerate(public[-3:], start=21):
        message.sequence = sequence
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="distinct boundaries"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_audits_post_empty_raw_fault_to_close():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    fault = _status(77, "child fault", state=5, valid=False)
    fault.sequence = 4
    fault.base_motion_allowed = False
    internal.append((3_500_000_000, fault))

    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="internal status contains invalid"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_requires_detached_close_child_and_monotonic_chain():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    close = json.loads(rosout[-1][2].split(" ", 1)[1])
    close["child"]["base_motion_allowed"] = False
    rosout[-1] = (rosout[-1][0], rosout[-1][1],
                  ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX +
                  json.dumps(close, separators=(",", ":")))
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="CLOSE child"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)

    public, internal, action_status, feedback, rosout = _ownership_fixture()
    loaded = json.loads(rosout[1][2].split(" ", 1)[1])
    loaded["captured_monotonic_s"] = 9.0
    rosout[1] = (rosout[1][0], rosout[1][1],
                 ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX +
                 json.dumps(loaded, separators=(",", ":")))
    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_corroborated_selector_uses_earliest_success_and_expands_close_fence():
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    early = action_status[-1][1].status_list[0]
    early = SimpleNamespace(
        goal_info=early.goal_info, status=4)
    action_status.insert(2, (2_500_000_000, SimpleNamespace(status_list=[early])))
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="earliest successful"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)

    public, internal, action_status, feedback, rosout = _ownership_fixture()
    other_uuid = [32] + [0] * 15
    other = SimpleNamespace(
        goal_info=SimpleNamespace(
            goal_id=SimpleNamespace(uuid=other_uuid),
            stamp=SimpleNamespace(sec=5, nanosec=9)), status=1)
    success = action_status[-1][1]
    action_status[-1] = (4_500_000_000, SimpleNamespace(status_list=[other]))
    action_status.append((5_000_000_000, success))
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="another ExecuteProductCycle"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            public, internal, action_status, feedback, rosout, 102)


def test_legacy_old42_overlap_stays_rejected_without_positive_proof():
    boot = 42
    raw_rows = [
        (1_000_000_000, _status(boot, ANALYZER_MODULE.STAGE_START_MARKER)),
        (1_100_000_000, _status(boot, "Product 101 loaded", 2, True, True, "101")),
        (1_200_000_000, _status(boot, "Product 101 empty", 1)),
        (5_454_000_000, _status(boot, ANALYZER_MODULE.STAGE_START_MARKER)),
        (5_455_000_000, _status(boot, ANALYZER_MODULE.OWNERSHIP_STALE_DETAIL)),
        (5_474_000_000, _status(boot, ANALYZER_MODULE.STAGE_START_MARKER)),
        (5_500_000_000, _status(boot, "Product 102 loaded", 2, True, True, "102")),
        (5_600_000_000, _status(boot, "Product 102 empty", 1)),
    ]
    statuses = []
    for sequence, (receipt, message) in enumerate(raw_rows, start=1):
        message.sequence = sequence
        statuses.append((receipt, message))
    assert ANALYZER_MODULE.select_stage_status_stream(statuses, 102) is None
    with pytest.raises(ANALYZER_MODULE.AnalysisError, match="ownership rosout records are missing"):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            statuses, [], [], [], [], 102)


def test_retained_old42_e2_public_status_stays_rejected_without_ownership_proof():
    retained_path = (PACKAGE_ROOT.parents[1] / "phase14_evidence" /
                     "stability_20261003" / "B_RECOVERY_SNAPSHOT_20261004" /
                     "public_status.jsonl.gz")
    statuses = []
    with gzip.open(retained_path, "rt", encoding="utf-8") as retained:
        for line in retained:
            row = json.loads(line)
            header = row["header"]
            message = SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(**header["stamp"]),
                    frame_id=header.get("frame_id", "")),
                **{name: row[name] for name in (
                    "source_boot_id", "sequence", "valid", "state",
                    "base_motion_allowed", "product_attached", "product_id",
                    "detail")})
            statuses.append((row["bag_timestamp_ns"], message))

    assert ANALYZER_MODULE.select_stage_status_stream(statuses, 102) is None
    with pytest.raises(ANALYZER_MODULE.AnalysisError):
        ANALYZER_MODULE.select_corroborated_stage_status_stream(
            statuses, [], [], [], [], 102)


@pytest.mark.parametrize("contradictory_refresh", [False, True])
def test_corroborated_selector_keeps_first_loaded_and_audits_refreshes(
        contradictory_refresh):
    public, internal, action_status, feedback, rosout = _ownership_fixture()

    def refreshed_status(source, sequence, product_id=None):
        message = SimpleNamespace(**vars(source))
        message.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=100, nanosec=sequence),
            frame_id=source.header.frame_id)
        message.sequence = sequence
        if product_id is not None:
            message.product_id = product_id
        return message

    refreshes = [
        (2_200_000_000, refreshed_status(
            public[6][1], 14, "101" if contradictory_refresh else None)),
        (2_500_000_000, refreshed_status(public[6][1], 15)),
    ]
    public[7] = (public[7][0], refreshed_status(public[7][1], 16))
    for index in range(8, len(public)):
        receipt, message = public[index]
        public[index] = (receipt, refreshed_status(message, index + 9))
    public[7:7] = refreshes

    empty_receipt, empty_logger, empty_text = rosout[2]
    empty_record = json.loads(empty_text[len(ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX):])
    empty_record["public"] = _wire_status(public[9][1])
    rosout[2] = (
        empty_receipt, empty_logger,
        ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX +
        json.dumps(empty_record, separators=(",", ":")))

    close_receipt, close_logger, close_text = rosout[3]
    close_record = json.loads(close_text[len(ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX):])
    close_record["public_sequence_high_water"] = 22
    close_record["first_boundaries"]["EMPTY"]["sequence"] = 16
    rosout[3] = (
        close_receipt, close_logger,
        ANALYZER_MODULE.OWNERSHIP_LOG_PREFIX +
        json.dumps(close_record, separators=(",", ":")))

    if contradictory_refresh:
        with pytest.raises(ANALYZER_MODULE.AnalysisError,
                           match="public status span contains invalid/fault/wrong-product"):
            ANALYZER_MODULE.select_corroborated_stage_status_stream(
                public, internal, action_status, feedback, rosout, 102)
        return

    public_boot, selected = ANALYZER_MODULE.select_corroborated_stage_status_stream(
        public, internal, action_status, feedback, rosout, 102)
    assert public_boot == 44
    assert [message.sequence for _, message in selected] == [10, 11, 12, 13, 14, 15, 16]


def test_analyze_collects_raw_topics_and_runs_the_real_corroborated_fallback(monkeypatch):
    public, internal, action_status, feedback, rosout = _ownership_fixture()
    rows = []
    messages = {}

    def add(topic, timestamp_ns, message):
        payload = f"{len(messages):08d}".encode("ascii")
        messages[payload] = message
        rows.append((topic, payload, timestamp_ns))

    for receipt, message in public:
        add("/amr/manipulation/status", receipt, message)
    for receipt, message in internal:
        add(ANALYZER_MODULE.INTERNAL_STATUS_TOPIC, receipt, message)
    for receipt, message in action_status:
        add(ANALYZER_MODULE.EXECUTE_CYCLE_STATUS_TOPIC, receipt, message)
    for receipt, message in feedback:
        add(ANALYZER_MODULE.EXECUTE_CYCLE_FEEDBACK_TOPIC, receipt, message)
    for receipt, name, text in rosout:
        add("/rosout", receipt, SimpleNamespace(name=name, msg=text))

    add(ANALYZER_MODULE.BOOTSTRAP_TOPIC, 500_000_000,
        SimpleNamespace(data="READY detached"))
    state_topic = "/amr/simulation/internal/attachment/product_102/state"
    pose_topic = "/model/test_product_102/pose"
    add(state_topic, 2_500_000_000, SimpleNamespace(data="attached"))
    add(state_topic, 2_900_000_000, SimpleNamespace(data="detached"))
    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    add(pose_topic, 3_000_000_000, SimpleNamespace(pose=pose))
    add("/amr/simulation/ground_truth/pose", 1_500_000_000,
        SimpleNamespace(pose=pose))
    contact = SimpleNamespace(
        collision1=SimpleNamespace(name="test_product_102"),
        collision2=SimpleNamespace(name="finger"))
    add("/amr/simulation/contacts/left_finger", 2_500_000_000,
        SimpleNamespace(contacts=[contact]))
    add("/amr/simulation/contacts/right_finger", 2_500_000_000,
        SimpleNamespace(contacts=[contact]))
    nav_status = SimpleNamespace(status_list=[SimpleNamespace(status=2)])
    add(ANALYZER_MODULE.NORMAL_NAV_STATUS_TOPIC, 2_000_000_000, nav_status)
    twist = SimpleNamespace(
        linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))
    add(ANALYZER_MODULE.CONTROL_TOPIC, 2_500_000_000, twist)
    add(ANALYZER_MODULE.SIMULATION_TOPIC, 2_500_000_000, twist)

    selected = {
        "/amr/manipulation/status", ANALYZER_MODULE.BOOTSTRAP_TOPIC,
        state_topic, pose_topic, "/amr/simulation/ground_truth/pose",
        "/amr/simulation/contacts/left_finger",
        "/amr/simulation/contacts/right_finger", ANALYZER_MODULE.CONTROL_TOPIC,
        ANALYZER_MODULE.SIMULATION_TOPIC, ANALYZER_MODULE.NORMAL_NAV_STATUS_TOPIC,
        "/rosout", ANALYZER_MODULE.INTERNAL_STATUS_TOPIC,
        ANALYZER_MODULE.EXECUTE_CYCLE_STATUS_TOPIC,
        ANALYZER_MODULE.EXECUTE_CYCLE_FEEDBACK_TOPIC,
    }
    for topic in ANALYZER_MODULE.REQUIRED_TOPIC_SUFFIXES:
        if topic not in selected and not any(row[0] == topic for row in rows):
            add(topic, 0, object())
    rows.sort(key=lambda row: row[2])

    class FakeReader:
        def __init__(self, entries):
            self.entries = entries
            self.index = 0

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=topic, type="fake/msg/Message")
                    for topic in sorted({row[0] for row in self.entries})]

        def has_next(self):
            return self.index < len(self.entries)

        def read_next(self):
            row = self.entries[self.index]
            self.index += 1
            return row

    monkeypatch.setattr(ANALYZER_MODULE, "_load_product_registry",
                        lambda _product: ("test_product_102", 1.0, (0.0, 0.0, 0.0)))
    monkeypatch.setattr(ANALYZER_MODULE, "_open_reader",
                        lambda _bag: FakeReader(rows))
    monkeypatch.setattr(ANALYZER_MODULE, "get_message", lambda name: name)
    monkeypatch.setattr(ANALYZER_MODULE, "deserialize_message",
                        lambda payload, _type: messages[payload])

    report = ANALYZER_MODULE.analyze(Path("fake-bag"), 102)

    assert "diagnostic stage_source_boot_id=44" in report
    assert not any("ownership corroboration rejected:" in line for line in report)
    assert any("base command evidence was empty" in line for line in report)


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
