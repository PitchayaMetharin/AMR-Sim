from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import pytest  # noqa: E402
import yaml  # noqa: E402

import factory_mapping_acceptance as acceptance  # noqa: E402
import factory_mapping_artifacts as artifacts  # noqa: E402


_MISSING = object()


def _map_message():
    return {
        "header": {"frame_id": "map"},
        "info": {
            "width": 2,
            "height": 2,
            "resolution": 0.05,
            "origin": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
        "data": [-1, 0, 100, -1],
    }


def _snapshot(mode="manual", now=10.0):
    nodes = sorted(acceptance.COMMON_REQUIRED_NODES)
    mpc = []
    exploration = None
    if mode == "autonomous":
        nodes.extend(sorted(acceptance.AUTONOMOUS_REQUIRED_NODES))
        mpc = ["/amr/controller_server"]
        exploration = {
            "state": "STOPPED",
            "fault_latched": False,
            "active": False,
            "pending": False,
            "cancel_target": "",
            "run_generation": 2,
        }
    return {
        "now": now,
        "nodes": nodes,
        "topic_publishers": {
            "/map": ["/amr/slam_toolbox"],
            "/amr/control/cmd_vel": ["/amr/command_arbitration_node"],
            "/amr/mpc/cmd_vel": mpc,
            acceptance.TF_OWNERSHIP_TOPIC: [acceptance.TF_OWNERSHIP_SOURCE],
        },
        "map": _map_message(),
        "map_received_at": now - 0.2,
        "tf_received_at": {
            ("map", "odom"): now - 0.2,
            ("odom", "base_footprint"): now - 0.2,
        },
        "tf_publishers": {
            "map->odom": ["/amr/slam_toolbox"],
            "odom->base_footprint": ["/amr/ekf_filter_node"],
        },
        "manipulator_status": {
            "valid": True,
            "state": 1,
            "product_attached": False,
            "base_motion_allowed": True,
        },
        "manipulator_status_received_at": now - 0.2,
        "exploration_status": exploration,
        "exploration_status_received_at": (
            now - 0.2 if exploration is not None else None),
        "host_preflight": {"verdict": "PASS"},
        "runtime_preflight": {"verdict": "PASS"},
    }


def test_manual_snapshot_passes_all_observation_gates():
    result = acceptance.evaluate_snapshot(_snapshot("manual"), "manual")
    assert result["verdict"] == "PASS"
    assert result["failures"] == []
    assert result["map_counts"] == {
        "width": 2, "height": 2, "known": 2, "free": 1,
        "occupied": 1, "unknown": 2,
    }


@pytest.mark.parametrize("state,expected", [
    (1, True),
    (True, False),
    (1.0, False),
    ("1", False),
    ("STOWED_EMPTY", False),
    ("stowed_empty", False),
])
def test_manipulator_gate_requires_canonical_integer_state(state, expected):
    snapshot = _snapshot()
    snapshot["manipulator_status"]["state"] = state

    result = acceptance.evaluate_snapshot(snapshot, "manual")

    assert result["checks"]["manipulator_authority"]["passed"] is expected
    assert result["verdict"] == ("PASS" if expected else "FAIL")


@pytest.mark.parametrize("field", ["host_preflight", "runtime_preflight"])
@pytest.mark.parametrize("value", [
    {"verdict": True},
    {"passed": True},
    {"verdict": " PASS"},
    {"verdict": "pass"},
    {},
    {"verdict": 1},
])
def test_evaluate_rejects_noncanonical_preflight_verdict(field, value):
    snapshot = _snapshot()
    snapshot[field] = value

    result = acceptance.evaluate_snapshot(snapshot, "manual")

    assert result["verdict"] == "FAIL"
    assert "run_preflight_reports" in result["failures"]
    assert result["checks"]["run_preflight_reports"]["passed"] is False


@pytest.mark.parametrize("mutation", [
    lambda snapshot: snapshot.pop("tf_publishers"),
    lambda snapshot: snapshot.update(tf_publishers={}),
    lambda snapshot: snapshot.update(tf_publishers={
        "map->odom": ["/amr/slam_toolbox"]}),
    lambda snapshot: snapshot.update(tf_publishers={
        "map->odom": "/amr/slam_toolbox",
        "odom->base_footprint": ["/amr/ekf_filter_node"]}),
])
def test_missing_or_malformed_tf_ownership_fails_closed(mutation):
    snapshot = _snapshot()
    mutation(snapshot)
    result = acceptance.evaluate_snapshot(snapshot, "manual")
    assert result["verdict"] == "FAIL"
    assert "tf_ownership" in result["failures"]
    assert result["checks"]["tf_ownership"]["passed"] is False


def test_tf_ownership_evidence_parses_exact_edges_and_empty_edges_fail_closed():
    payload = {
        "schema": 1,
        "kind": "factory_tf_ownership",
        "observed_monotonic": 10.0,
        "edges": {
            "map->odom": {"owners": ["/amr/slam_toolbox"]},
            "odom->base_footprint": {"owners": []},
        },
    }
    parsed = acceptance._parse_tf_ownership_message(yaml.safe_dump(payload))
    assert parsed == {
        "map->odom": ["/amr/slam_toolbox"],
        "odom->base_footprint": [],
    }
    snapshot = _snapshot()
    snapshot["tf_publishers"] = parsed
    result = acceptance.evaluate_snapshot(snapshot, "manual")
    assert result["checks"]["tf_ownership"]["passed"] is False


@pytest.mark.parametrize("mutation", [
    lambda payload: payload.pop("edges"),
    lambda payload: payload.update(kind="wrong"),
    lambda payload: payload.update(schema=2),
    lambda payload: payload["edges"].update({"unexpected": {"owners": []}}),
    lambda payload: payload["edges"]["map->odom"].update(owners="/amr/slam_toolbox"),
    lambda payload: payload["edges"]["map->odom"].update(
        owners=["/amr/slam_toolbox", "/amr/slam_toolbox"]),
])
def test_tf_ownership_message_parser_rejects_malformed_evidence(mutation):
    payload = {
        "schema": 1,
        "kind": "factory_tf_ownership",
        "observed_monotonic": 10.0,
        "edges": {
            "map->odom": {"owners": ["/amr/slam_toolbox"]},
            "odom->base_footprint": {"owners": ["/amr/ekf_filter_node"]},
        },
    }
    mutation(payload)
    assert acceptance._parse_tf_ownership_message(yaml.safe_dump(payload)) is None


@pytest.mark.parametrize("state", ["STOPPED", "COMPLETE", "INCOMPLETE"])
def test_autonomous_snapshot_accepts_terminal_exploration_states(state):
    snapshot = _snapshot("autonomous")
    snapshot["exploration_status"]["state"] = state
    result = acceptance.evaluate_snapshot(snapshot, "autonomous")
    assert result["verdict"] == "PASS"
    assert result["exploration_outcome"] == state

    broken = _snapshot("autonomous")
    broken["exploration_status"]["active"] = True
    assert acceptance.evaluate_snapshot(broken, "autonomous")["verdict"] == "FAIL"


@pytest.mark.parametrize("state", [
    "WAITING_READY", "SCANNING", "GOAL_PENDING", "NAVIGATING", "CANCELLING",
    "FAULT", "UNKNOWN", None,
])
def test_autonomous_snapshot_rejects_nonterminal_or_missing_exploration_state(state):
    snapshot = _snapshot("autonomous")
    if state is None:
        snapshot["exploration_status"].pop("state")
    else:
        snapshot["exploration_status"]["state"] = state
    result = acceptance.evaluate_snapshot(snapshot, "autonomous")
    assert result["verdict"] == "FAIL"
    assert "autonomous_mode" in result["failures"]


@pytest.mark.parametrize("field,value", [
    ("active", "missing"), ("active", True), ("active", "malformed"),
    ("pending", "missing"), ("pending", True), ("pending", "malformed"),
    ("fault_latched", "missing"), ("fault_latched", True),
    ("fault_latched", "malformed"),
])
def test_autonomous_snapshot_requires_false_boolean_exploration_fields(field, value):
    snapshot = _snapshot("autonomous")
    if value == "missing":
        snapshot["exploration_status"].pop(field)
    else:
        snapshot["exploration_status"][field] = (
            "not-a-boolean" if value == "malformed" else value)
    result = acceptance.evaluate_snapshot(snapshot, "autonomous")
    assert result["verdict"] == "FAIL"
    assert "autonomous_mode" in result["failures"]


@pytest.mark.parametrize("target", ["missing", "/amr/mission/navigate_to_pose"])
def test_autonomous_snapshot_requires_no_outstanding_cancellation_target(target):
    snapshot = _snapshot("autonomous")
    if target == "missing":
        snapshot["exploration_status"].pop("cancel_target")
    else:
        snapshot["exploration_status"]["cancel_target"] = target
    result = acceptance.evaluate_snapshot(snapshot, "autonomous")
    assert result["verdict"] == "FAIL"
    assert "autonomous_mode" in result["failures"]


def test_autonomous_snapshot_requires_fresh_exploration_status():
    snapshot = _snapshot("autonomous")
    snapshot["exploration_status_received_at"] = (
        snapshot["now"] - acceptance.STATUS_MAX_AGE_SECONDS - 0.1)
    result = acceptance.evaluate_snapshot(snapshot, "autonomous")
    assert result["verdict"] == "FAIL"
    assert "autonomous_mode" in result["failures"]


@pytest.mark.parametrize("generation", [0, -1, None, 2.5])
def test_autonomous_snapshot_requires_positive_integer_generation(generation):
    snapshot = _snapshot("autonomous")
    if generation is None:
        snapshot["exploration_status"].pop("run_generation")
    else:
        snapshot["exploration_status"]["run_generation"] = generation
    result = acceptance.evaluate_snapshot(snapshot, "autonomous")
    assert result["verdict"] == "FAIL"
    assert "autonomous_mode" in result["failures"]


@pytest.mark.parametrize("mutation", [
    lambda s: s["map"]["header"].update(frame_id="odom"),
    lambda s: s.update(map_received_at=6.0),
    lambda s: s["tf_received_at"].update({("map", "odom"): 8.0}),
    lambda s: s["manipulator_status"].update(base_motion_allowed=False),
    lambda s: s["topic_publishers"].update(
        {"/amr/control/cmd_vel": ["/amr/command_arbitration_node", "/amr/intruder"]}),
    lambda s: s["topic_publishers"].update(
        {acceptance.TF_OWNERSHIP_TOPIC: ["/amr/intruder"]}),
    lambda s: s["topic_publishers"].update(
        {acceptance.TF_OWNERSHIP_TOPIC: [acceptance.TF_OWNERSHIP_SOURCE, "/amr/intruder"]}),
    lambda s: s["nodes"].append("/amr/amcl"),
])
def test_freshness_ownership_and_forbidden_node_gates_fail_closed(mutation):
    snapshot = _snapshot()
    mutation(snapshot)
    result = acceptance.evaluate_snapshot(snapshot, "manual")
    assert result["verdict"] == "FAIL"
    assert result["failures"]


def test_manual_mode_rejects_controller_and_unowned_mpc_publisher():
    snapshot = _snapshot("manual")
    snapshot["nodes"].append("/amr/controller_server")
    snapshot["topic_publishers"]["/amr/mpc/cmd_vel"] = ["/amr/intruder"]
    result = acceptance.evaluate_snapshot(snapshot, "manual")
    assert result["checks"]["manual_mode"]["passed"] is False


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class FakeAdapter:
    def __init__(self, snapshot_factory, clock):
        self.snapshot_factory = snapshot_factory
        self.clock = clock

    def snapshot(self):
        return self.snapshot_factory(self.clock.now)

    def spin_once(self, timeout_sec):
        self.clock.now += timeout_sec


def test_observer_timeout_is_bounded_and_preserves_failure_observations():
    clock = FakeClock()

    def missing_snapshot(now):
        snapshot = _snapshot(now=now)
        snapshot["map_received_at"] = now - 100.0
        return snapshot

    result = acceptance.observe_until_pass(
        FakeAdapter(missing_snapshot, clock), "manual",
        timeout_seconds=1.0, monotonic=clock.monotonic, poll_seconds=0.25)
    assert result["verdict"] == "FAIL"
    assert result["timed_out"] is True
    assert "fresh_valid_map" in result["failures"]
    assert "bounded_timeout" in result["failures"]
    assert clock.now >= 1.0


def test_observer_rejects_post_evaluation_deadline_crossing(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    clock = FakeClock()
    monotonic_values = iter((0.0, 0.0, 0.5, 1.5))

    def monotonic():
        return next(monotonic_values)

    report = acceptance.run_runtime_acceptance(
        session, "candidate", "manual",
        FakeAdapter(lambda _now: _snapshot("manual", now=0.5), clock),
        timeout_seconds=1.0, monotonic=monotonic, poll_seconds=0.1)

    assert report["verdict"] == "FAIL"
    assert report["timed_out"] is True
    assert report["elapsed_seconds"] == 1.5
    assert report["deadline_seconds"] == 1.0
    assert report["failures"] == ["bounded_timeout"]
    assert report["checks"]["mode"]["passed"] is True
    assert report["checks"]["bounded_timeout"] == {
        "passed": False,
        "observed": 1.5,
        "expected": "<= 1.0 seconds",
    }
    assert report["map_counts"] == {
        "width": 2, "height": 2, "known": 2, "free": 1,
        "occupied": 1, "unknown": 2,
    }

    _write_quality_review(session, manifest)
    with pytest.raises(ValueError, match="runtime acceptance did not pass"):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


def test_observer_rejects_timeout_above_documented_bound():
    clock = FakeClock()
    with pytest.raises(ValueError, match="acceptance timeout"):
        acceptance.observe_until_pass(
            FakeAdapter(lambda now: _snapshot(now=now), clock), "manual",
            timeout_seconds=acceptance.ACCEPTANCE_TIMEOUT_SECONDS + 1.0,
            monotonic=clock.monotonic)
    assert clock.now == 0.0


def _write_validated_session(tmp_path):
    session = tmp_path / "session"
    session.mkdir()
    image = session / "candidate.pgm"
    image.write_bytes(b"P5\n1 1\n255\n0")
    (session / "candidate.posegraph").write_text("graph", encoding="utf-8")
    (session / "candidate.data").write_bytes(b"serialized graph data")
    (session / "candidate.yaml").write_text(yaml.safe_dump({
        "image": image.name,
        "resolution": 0.05,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }), encoding="utf-8")
    manifest = artifacts.build_manifest(
        session, session / "candidate", [0.0, 0.0, 0.0])
    manifest["state"] = "VALIDATED"
    manifest["validated_unix"] = time.time()
    artifacts.atomic_write_yaml(session / artifacts.MANIFEST_NAME, manifest)
    return session, manifest


def _dynamic_valid_snapshot(now, mode="manual"):
    result = _snapshot(mode, now)
    result["map_received_at"] = now - 0.1
    result["tf_received_at"] = {
        ("map", "odom"): now - 0.1,
        ("odom", "base_footprint"): now - 0.1,
    }
    result["manipulator_status_received_at"] = now - 0.1
    return result


def test_runtime_report_is_bound_to_validated_candidate(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    clock = FakeClock()
    report = acceptance.run_runtime_acceptance(
        session, "candidate", "manual", FakeAdapter(_dynamic_valid_snapshot, clock),
        timeout_seconds=2.0, monotonic=clock.monotonic, poll_seconds=0.1)
    assert report["verdict"] == "PASS"
    assert report["candidate_sha256"] == manifest["candidate_sha256"]
    assert (session / acceptance.RUNTIME_ACCEPTANCE_NAME).is_file()


@pytest.mark.parametrize("state", ["STOPPED", "COMPLETE", "INCOMPLETE"])
def test_autonomous_runtime_report_preserves_exploration_outcome(tmp_path, state):
    session, manifest = _write_validated_session(tmp_path)
    clock = FakeClock()

    def dynamic_snapshot(now):
        snapshot = _dynamic_valid_snapshot(now, "autonomous")
        snapshot["exploration_status"]["state"] = state
        return snapshot

    report = acceptance.run_runtime_acceptance(
        session, "candidate", "autonomous", FakeAdapter(dynamic_snapshot, clock),
        timeout_seconds=2.0, monotonic=clock.monotonic, poll_seconds=0.1)
    assert report["verdict"] == "PASS"
    assert report["exploration_outcome"] == state
    saved = yaml.safe_load(
        (session / acceptance.RUNTIME_ACCEPTANCE_NAME).read_text(encoding="utf-8"))
    assert saved["exploration_outcome"] == state


def test_missing_tf_ownership_cannot_be_accepted(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    clock = FakeClock()

    def missing_tf(now):
        snapshot = _dynamic_valid_snapshot(now)
        snapshot.pop("tf_publishers")
        return snapshot

    report = acceptance.run_runtime_acceptance(
        session, "candidate", "manual", FakeAdapter(missing_tf, clock),
        timeout_seconds=0.25, monotonic=clock.monotonic, poll_seconds=0.1)
    assert report["verdict"] == "FAIL"
    assert "tf_ownership" in report["failures"]
    _write_quality_review(session, manifest)
    with pytest.raises(ValueError, match="runtime acceptance did not pass"):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


def _write_quality_review(
        session, manifest, sha=None, path=None, *, runtime_sha256=None,
        incomplete_acknowledgement=None, incomplete_explanation=None):
    review = {
        "schema": acceptance.QUALITY_REVIEW_SCHEMA_VERSION,
        "decision": "ACCEPTED",
        "accepted": True,
        "reviewer": "commissioning-reviewer",
        "candidate_path": manifest["candidate_path"] if path is None else path,
        "candidate_sha256": manifest["candidate_sha256"] if sha is None else sha,
        "artifact_bundle_sha256": manifest["artifact_bundle_sha256"],
        "runtime_acceptance_sha256": (
            artifacts.sha256_file(session / acceptance.RUNTIME_ACCEPTANCE_NAME)
            if runtime_sha256 is None else runtime_sha256),
        "reviewed_unix": time.time(),
    }
    if incomplete_acknowledgement is not None:
        review["incomplete_acknowledgement"] = incomplete_acknowledgement
    if incomplete_explanation is not None:
        review["incomplete_explanation"] = incomplete_explanation
    review_path = session / acceptance.QUALITY_REVIEW_NAME
    review_path.write_text(yaml.safe_dump(review), encoding="utf-8")
    return review_path


def _write_passing_runtime_report(session, manifest, mode="manual", state=None):
    clock = FakeClock()

    def snapshot_factory(now):
        snapshot = _dynamic_valid_snapshot(now, mode)
        if state is not None:
            snapshot["exploration_status"]["state"] = state
        return snapshot

    return acceptance.run_runtime_acceptance(
        session, manifest["candidate_name"], mode,
        FakeAdapter(snapshot_factory, clock),
        timeout_seconds=2.0, monotonic=clock.monotonic, poll_seconds=0.1)


def _mutate_saved_runtime(session, mutate):
    runtime_path = session / acceptance.RUNTIME_ACCEPTANCE_NAME
    runtime = yaml.safe_load(runtime_path.read_text(encoding="utf-8"))
    mutate(runtime)
    artifacts.atomic_write_yaml(runtime_path, runtime)
    return runtime


def _refresh_review_runtime_hash(session):
    review_path = session / acceptance.QUALITY_REVIEW_NAME
    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    review["runtime_acceptance_sha256"] = artifacts.sha256_file(
        session / acceptance.RUNTIME_ACCEPTANCE_NAME)
    artifacts.atomic_write_yaml(review_path, review)
    return review_path


def _refresh_acceptance_evidence_hashes(session):
    acceptance_path = session / acceptance.MAPPING_ACCEPTANCE_NAME
    receipt = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    receipt["runtime_acceptance"]["sha256"] = artifacts.sha256_file(
        session / acceptance.RUNTIME_ACCEPTANCE_NAME)
    review_path = session / acceptance.QUALITY_REVIEW_NAME
    receipt["quality_review"]["sha256"] = artifacts.sha256_file(review_path)
    artifacts.atomic_write_yaml(acceptance_path, receipt)


def _refresh_acceptance_evidence_hashes_for_current_bundle(session):
    acceptance_path = session / acceptance.MAPPING_ACCEPTANCE_NAME
    receipt = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    manifest_path = session / artifacts.MANIFEST_NAME
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    receipt["artifact_bundle_sha256"] = manifest["artifact_bundle_sha256"]
    receipt["artifact_manifest"]["sha256"] = artifacts.sha256_file(manifest_path)
    receipt["runtime_acceptance"]["sha256"] = artifacts.sha256_file(
        session / acceptance.RUNTIME_ACCEPTANCE_NAME)
    review_path = session / acceptance.QUALITY_REVIEW_NAME
    receipt["quality_review"]["sha256"] = artifacts.sha256_file(review_path)
    artifacts.atomic_write_yaml(acceptance_path, receipt)


def _mutate_saved_acceptance(session, mutate):
    acceptance_path = session / acceptance.MAPPING_ACCEPTANCE_NAME
    receipt = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    mutate(receipt)
    artifacts.atomic_write_yaml(acceptance_path, receipt)
    return receipt


def _resave_validated_session(session, *, changed_artifact):
    yaml_path = session / "candidate.yaml"
    yaml_text = yaml_path.read_text(encoding="utf-8")
    artifacts.discard_verified(session, "candidate")

    (session / "candidate.pgm").write_bytes(
        b"P5\n1 1\n255\n1" if changed_artifact == "image"
        else b"P5\n1 1\n255\n0")
    (session / "candidate.posegraph").write_text("graph", encoding="utf-8")
    (session / "candidate.data").write_bytes(
        b"changed serialized graph data"
        if changed_artifact == "dataset" else b"serialized graph data")
    yaml_path.write_text(yaml_text, encoding="utf-8")

    manifest = artifacts.build_manifest(
        session, session / "candidate", [0.0, 0.0, 0.0])
    manifest["state"] = "VALIDATED"
    manifest["validated_unix"] = time.time()
    artifacts.atomic_write_yaml(session / artifacts.MANIFEST_NAME, manifest)
    return manifest


def _assert_acceptance_receipt_mutation_rejected(
        tmp_path, mutate, *, mode="manual", state=None):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest, mode=mode, state=state)
    _write_quality_review(session, manifest)
    acceptance.accept_candidate(session, "candidate")
    _mutate_saved_acceptance(session, mutate)

    with pytest.raises(ValueError):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


def _set_exploration_field(runtime, field, value):
    observed = runtime["checks"]["autonomous_mode"]["observed"]
    target = observed if field == "exploration_received_at" else observed[
        "exploration"]
    target[field] = value


def _assert_runtime_mutation_rejected(
        tmp_path, operation, field, value, *, mode="manual", state=None):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest, mode=mode, state=state)
    _write_quality_review(session, manifest)
    if operation == "promote":
        acceptance.accept_candidate(session, "candidate")

    def mutate(runtime):
        if field == "__mutation__":
            value(runtime)
        elif value is _MISSING:
            runtime.pop(field)
        else:
            runtime[field] = value(runtime) if callable(value) else value

    _mutate_saved_runtime(session, mutate)
    _refresh_review_runtime_hash(session)
    if operation == "promote":
        _refresh_acceptance_evidence_hashes(session)

    with pytest.raises(ValueError):
        if operation == "accept":
            acceptance.accept_candidate(session, "candidate")
        else:
            acceptance.promote_candidate(session, "candidate")


@pytest.mark.parametrize("operation", ["accept", "promote"])
@pytest.mark.parametrize("field", ["host", "runtime"])
def test_accept_and_promote_reject_boolean_preflight_verdict(
        tmp_path, operation, field):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)
    if operation == "promote":
        acceptance.accept_candidate(session, "candidate")

    _mutate_saved_runtime(
        session,
        lambda runtime: runtime["checks"]["run_preflight_reports"][
            "observed"].update({field: {"verdict": True}}))
    _refresh_review_runtime_hash(session)
    if operation == "promote":
        _refresh_acceptance_evidence_hashes(session)

    with pytest.raises(ValueError, match="preflight"):
        if operation == "accept":
            acceptance.accept_candidate(session, "candidate")
        else:
            acceptance.promote_candidate(session, "candidate")

    if operation == "accept":
        assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()
    else:
        assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


@pytest.mark.parametrize("mode,state", [("manual", None), ("autonomous", "STOPPED")])
def test_producer_shaped_manual_and_autonomous_reports_are_accepted_and_promoted(
        tmp_path, mode, state):
    session, manifest = _write_validated_session(tmp_path)
    report = _write_passing_runtime_report(
        session, manifest, mode=mode, state=state)
    expected_fields = {
        "schema", "kind", "mode", "checked_monotonic", "verdict",
        "timed_out", "failures", "checks", "map_counts", "candidate_name",
        "candidate_path", "candidate_sha256", "artifact_bundle_sha256",
        "session_path", "created_unix", "elapsed_seconds", "deadline_seconds",
    }
    if mode == "autonomous":
        expected_fields.add("exploration_outcome")
    assert set(report) == expected_fields
    acceptance._report_base(manifest, report, mode=mode)
    _write_quality_review(session, manifest)

    accepted = acceptance.accept_candidate(session, "candidate")
    promoted = acceptance.promote_candidate(session, "candidate")
    expected_acceptance_fields = {
        "schema", "kind", "state", "created_unix", "session_path",
        "candidate_name", "candidate_path", "candidate_sha256",
        "artifact_bundle_sha256", "conditions", "artifact_manifest",
        "runtime_acceptance", "quality_review",
    }
    if mode == "autonomous":
        expected_acceptance_fields.add("exploration_outcome")
    assert set(accepted) == expected_acceptance_fields
    assert accepted["conditions"] == {
        "artifact_manifest": "VALIDATED",
        "runtime_acceptance": "PASS",
        "quality_review": "ACCEPTED",
    }
    assert set(accepted["artifact_manifest"]) == {"path", "sha256"}
    assert accepted["artifact_manifest"] == {
        "path": str(session / artifacts.MANIFEST_NAME),
        "sha256": artifacts.sha256_file(session / artifacts.MANIFEST_NAME),
    }
    assert set(accepted["runtime_acceptance"]) == {
        "path", "sha256", "mode",
    }
    assert accepted["runtime_acceptance"] == {
        "path": str(session / acceptance.RUNTIME_ACCEPTANCE_NAME),
        "sha256": artifacts.sha256_file(
            session / acceptance.RUNTIME_ACCEPTANCE_NAME),
        "mode": mode,
    }
    assert set(accepted["quality_review"]) == {"path", "sha256", "reviewer"}
    assert accepted["quality_review"] == {
        "path": str(session / acceptance.QUALITY_REVIEW_NAME),
        "sha256": artifacts.sha256_file(session / acceptance.QUALITY_REVIEW_NAME),
        "reviewer": "commissioning-reviewer",
    }
    assert accepted["state"] == "PROMOTION_ELIGIBLE"
    assert promoted["state"] == "PROMOTION_ELIGIBLE"


@pytest.mark.parametrize("operation", ["accept", "promote"])
@pytest.mark.parametrize(
    "field,value",
    [
        ("candidate_name", _MISSING),
        ("candidate_name", 42),
        ("candidate_name", "other-candidate"),
        ("session_path", _MISSING),
        ("session_path", 42),
        ("session_path", "/other/session"),
        ("created_unix", _MISSING),
        ("created_unix", "not-a-number"),
        ("elapsed_seconds", _MISSING),
        ("elapsed_seconds", "not-a-number"),
        ("deadline_seconds", _MISSING),
        ("deadline_seconds", "not-a-number"),
        ("checked_monotonic", _MISSING),
        ("checked_monotonic", "0"),
        ("checked_monotonic", True),
        ("checked_monotonic", float("nan")),
        ("checked_monotonic", float("inf")),
        ("schema", True),
        ("schema", 1.0),
    ],
)
def test_accept_and_promote_reject_invalid_runtime_producer_fields(
        tmp_path, operation, field, value):
    _assert_runtime_mutation_rejected(tmp_path, operation, field, value)


@pytest.mark.parametrize("operation", ["accept", "promote"])
@pytest.mark.parametrize("checked_monotonic", [0, 0.25])
def test_accept_and_promote_allow_real_checked_monotonic(
        tmp_path, operation, checked_monotonic):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)
    if operation == "promote":
        acceptance.accept_candidate(session, "candidate")

    _mutate_saved_runtime(
        session,
        lambda runtime: runtime.update(checked_monotonic=checked_monotonic))
    _refresh_review_runtime_hash(session)
    if operation == "promote":
        _refresh_acceptance_evidence_hashes(session)

    if operation == "accept":
        acceptance.accept_candidate(session, "candidate")
    else:
        acceptance.promote_candidate(session, "candidate")


@pytest.mark.parametrize("operation", ["accept", "promote"])
@pytest.mark.parametrize(
    "field,value",
    [
        ("created_unix", True),
        ("created_unix", "1"),
        ("created_unix", float("nan")),
        ("created_unix", float("inf")),
        ("created_unix", 0.0),
        ("created_unix", -1.0),
        ("elapsed_seconds", True),
        ("elapsed_seconds", "0"),
        ("elapsed_seconds", float("nan")),
        ("elapsed_seconds", float("inf")),
        ("elapsed_seconds", -1.0),
        ("deadline_seconds", True),
        ("deadline_seconds", "2"),
        ("deadline_seconds", float("nan")),
        ("deadline_seconds", float("inf")),
        ("deadline_seconds", 0.0),
        ("deadline_seconds", acceptance.ACCEPTANCE_TIMEOUT_SECONDS + 0.1),
        ("elapsed_seconds", lambda runtime: runtime["deadline_seconds"] + 1.0),
    ],
)
def test_accept_and_promote_reject_invalid_runtime_timing(
        tmp_path, operation, field, value):
    _assert_runtime_mutation_rejected(tmp_path, operation, field, value)


@pytest.mark.parametrize("operation", ["accept", "promote"])
@pytest.mark.parametrize("schema", [1, True, 1.0])
def test_accept_and_promote_reject_unsupported_quality_review_schema(
        tmp_path, operation, schema):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    review_path = _write_quality_review(session, manifest)
    if operation == "promote":
        acceptance.accept_candidate(session, "candidate")
    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    review["schema"] = schema
    assert review["artifact_bundle_sha256"] == manifest["artifact_bundle_sha256"]
    artifacts.atomic_write_yaml(review_path, review)
    if operation == "promote":
        _refresh_acceptance_evidence_hashes(session)

    with pytest.raises(ValueError, match="quality review schema"):
        if operation == "accept":
            acceptance.accept_candidate(session, "candidate")
        else:
            acceptance.promote_candidate(session, "candidate")

    receipt_path = (
        session / acceptance.MAPPING_ACCEPTANCE_NAME
        if operation == "accept"
        else session / acceptance.PROMOTION_RECEIPT_NAME)
    assert not receipt_path.exists()


@pytest.mark.parametrize("operation", ["accept", "promote"])
@pytest.mark.parametrize(
    "bundle_hash", [_MISSING, False, "not-a-sha256", "0" * 64])
def test_accept_and_promote_require_exact_quality_review_bundle_hash(
        tmp_path, operation, bundle_hash):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    review_path = _write_quality_review(session, manifest)
    if operation == "promote":
        acceptance.accept_candidate(session, "candidate")

    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    if bundle_hash is _MISSING:
        review.pop("artifact_bundle_sha256")
    else:
        review["artifact_bundle_sha256"] = bundle_hash
    artifacts.atomic_write_yaml(review_path, review)
    if operation == "promote":
        _refresh_acceptance_evidence_hashes(session)

    with pytest.raises(ValueError, match="artifact bundle hash"):
        if operation == "accept":
            acceptance.accept_candidate(session, "candidate")
        else:
            acceptance.promote_candidate(session, "candidate")

    receipt_path = (
        session / acceptance.MAPPING_ACCEPTANCE_NAME
        if operation == "accept"
        else session / acceptance.PROMOTION_RECEIPT_NAME)
    assert not receipt_path.exists()


@pytest.mark.parametrize("changed_artifact", ["image", "dataset"])
def test_accept_rejects_stale_review_after_discard_resave_and_fresh_runtime(
        tmp_path, changed_artifact):
    session, old_manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, old_manifest)
    _write_quality_review(session, old_manifest)

    new_manifest = _resave_validated_session(
        session, changed_artifact=changed_artifact)
    assert new_manifest["candidate_sha256"] == old_manifest["candidate_sha256"]
    assert (new_manifest["artifact_bundle_sha256"] !=
            old_manifest["artifact_bundle_sha256"])
    _write_passing_runtime_report(session, new_manifest)
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError, match="artifact bundle hash"):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


@pytest.mark.parametrize("changed_artifact", ["image", "dataset"])
def test_promote_rejects_stale_review_after_discard_resave_and_refreshed_receipt(
        tmp_path, changed_artifact):
    session, old_manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, old_manifest)
    _write_quality_review(session, old_manifest)
    acceptance.accept_candidate(session, "candidate")

    new_manifest = _resave_validated_session(
        session, changed_artifact=changed_artifact)
    assert new_manifest["candidate_sha256"] == old_manifest["candidate_sha256"]
    assert (new_manifest["artifact_bundle_sha256"] !=
            old_manifest["artifact_bundle_sha256"])
    _write_passing_runtime_report(session, new_manifest)
    _refresh_review_runtime_hash(session)
    _refresh_acceptance_evidence_hashes_for_current_bundle(session)

    with pytest.raises(ValueError, match="artifact bundle hash"):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


def test_promote_rejects_mutated_review_after_review_hash_refresh(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    review_path = _write_quality_review(session, manifest)
    acceptance.accept_candidate(session, "candidate")

    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    review["artifact_bundle_sha256"] = "0" * 64
    artifacts.atomic_write_yaml(review_path, review)
    _refresh_acceptance_evidence_hashes(session)

    with pytest.raises(ValueError, match="artifact bundle hash"):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


def test_promote_rejects_boolean_mapping_acceptance_schema(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)
    acceptance.accept_candidate(session, "candidate")

    acceptance_path = session / acceptance.MAPPING_ACCEPTANCE_NAME
    receipt = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    receipt["schema"] = True
    artifacts.atomic_write_yaml(acceptance_path, receipt)

    with pytest.raises(ValueError, match="mapping acceptance schema"):
        acceptance.promote_candidate(session, "candidate")


@pytest.mark.parametrize(
    "mode,state,mutation",
    [
        ("manual", None, lambda runtime: runtime.update(
            exploration_outcome="STOPPED")),
        ("autonomous", "STOPPED", lambda runtime: runtime.pop(
            "exploration_outcome")),
    ],
)
def test_accept_and_promote_require_mode_specific_runtime_envelope(
        tmp_path, mode, state, mutation):
    for operation in ("accept", "promote"):
        operation_dir = tmp_path / operation
        operation_dir.mkdir()
        _assert_runtime_mutation_rejected(
            operation_dir, operation, "__mutation__", mutation,
            mode=mode, state=state)


@pytest.mark.parametrize("mutation", [
    lambda runtime: runtime.update(mode="MANUAL"),
    lambda runtime: runtime.pop("checked_monotonic"),
    lambda runtime: runtime.update(checked_monotonic=float("nan")),
    lambda runtime: runtime.update(timed_out=True),
    lambda runtime: runtime.pop("timed_out"),
    lambda runtime: runtime.update(failures=["forged_failure"]),
    lambda runtime: runtime.pop("failures"),
])
def test_accept_revalidates_runtime_report_envelope(tmp_path, mutation):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)
    _mutate_saved_runtime(session, mutation)
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


@pytest.mark.parametrize("mutation", [
    lambda map_fields: map_fields.pop("origin"),
    lambda map_fields: map_fields.update(resolution=0.1),
    lambda map_fields: map_fields.update(data=[0]),
    lambda map_fields: map_fields.update(data=[-1, 0, 101, -1]),
    lambda map_fields: map_fields["origin"]["position"].update(x="nan"),
    lambda map_fields: map_fields.update(free=0),
])
def test_accept_revalidates_persisted_map_geometry_and_data(tmp_path, mutation):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)

    def mutate(runtime):
        mutation(runtime["checks"]["fresh_valid_map"]["observed"]["map"])

    mutated = _mutate_saved_runtime(session, mutate)
    assert all(check["passed"] is True
               for check in mutated["checks"].values())
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


@pytest.mark.parametrize("mutation", [
    lambda runtime: runtime["checks"]["common_nodes"]["observed"][
        "nodes"].pop(),
    lambda runtime: runtime["checks"]["common_nodes"]["observed"][
        "nodes"].append("/amr/slam_toolbox"),
    lambda runtime: runtime["checks"]["production_localization_absent"][
        "observed"]["forbidden"].append("/amr/amcl"),
    lambda runtime: runtime["checks"]["fresh_valid_map"]["observed"][
        "map"].update(frame_id="odom"),
    lambda runtime: runtime["checks"]["fresh_localization_tf"]["observed"].update(
        map_to_odom=runtime["checked_monotonic"] -
        acceptance.TF_MAX_AGE_SECONDS - 0.1),
    lambda runtime: runtime["map_counts"].update(free=0),
    lambda runtime: runtime["checks"]["map_and_control_ownership"][
        "observed"].update({"/map": ["/amr/intruder"]}),
    lambda runtime: runtime["checks"]["tf_ownership"]["observed"][
        "map->odom"].__setitem__(0, "/amr/intruder"),
    lambda runtime: runtime["checks"]["manipulator_authority"][
        "observed"].update(product_attached=True),
    lambda runtime: runtime["checks"]["run_preflight_reports"][
        "observed"].update(host={"verdict": "FAIL"}),
])
def test_accept_revalidates_persisted_gate_observations(tmp_path, mutation):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)
    mutated = _mutate_saved_runtime(session, mutation)
    assert all(check["passed"] is True
               for check in mutated["checks"].values())
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


@pytest.mark.parametrize("mutation", [
    lambda runtime: runtime["checks"]["manual_mode"]["observed"][
        "/amr/mpc/cmd_vel"].append("/amr/intruder"),
])
def test_accept_revalidates_persisted_manual_mode_ownership(tmp_path, mutation):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest, mode="manual")
    _write_quality_review(session, manifest)
    mutated = _mutate_saved_runtime(session, mutation)
    assert mutated["checks"]["manual_mode"]["passed"] is True
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


@pytest.mark.parametrize("mutation", [
    lambda runtime: runtime["checks"]["autonomous_mode"]["observed"][
        "nodes"].remove("/amr/frontier_explorer"),
    lambda runtime: runtime["checks"]["autonomous_mode"]["observed"].update(
        {"/amr/mpc/cmd_vel": ["/amr/prototype_teleop"]}),
])
def test_accept_revalidates_persisted_autonomous_graph_and_mpc(
        tmp_path, mutation):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="STOPPED")
    _write_quality_review(session, manifest)
    mutated = _mutate_saved_runtime(session, mutation)
    assert mutated["checks"]["autonomous_mode"]["passed"] is True
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


def test_accept_revalidates_persisted_mode_evidence(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest, mode="manual")
    _write_quality_review(session, manifest)
    mutated = _mutate_saved_runtime(
        session,
        lambda runtime: runtime["checks"]["mode"].update(observed="autonomous"))
    assert mutated["checks"]["mode"]["passed"] is True
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


@pytest.mark.parametrize("mutation", ["missing", "inconsistent", "invalid"])
def test_report_base_rejects_missing_or_inconsistent_autonomous_outcome(
        tmp_path, mutation):
    session, manifest = _write_validated_session(tmp_path)
    runtime = _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="STOPPED")
    if mutation == "missing":
        runtime.pop("exploration_outcome")
    elif mutation == "inconsistent":
        runtime["exploration_outcome"] = "COMPLETE"
    else:
        runtime["exploration_outcome"] = "NOT_A_TERMINAL_STATE"

    with pytest.raises(ValueError, match="exploration outcome"):
        acceptance._report_base(manifest, runtime, mode="autonomous")


@pytest.mark.parametrize("field,value", [
    ("active", True),
    ("pending", True),
    ("fault_latched", True),
    ("cancel_target", "/amr/mission/navigate_to_pose"),
    ("run_generation", 0),
    ("exploration_received_at",
     lambda runtime: runtime["checked_monotonic"] -
     acceptance.STATUS_MAX_AGE_SECONDS - 0.1),
])
def test_accept_revalidates_each_persisted_exploration_field(
        tmp_path, field, value):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="STOPPED")
    _write_quality_review(session, manifest)

    def mutate(runtime):
        _set_exploration_field(
            runtime, field, value(runtime) if callable(value) else value)

    mutated = _mutate_saved_runtime(session, mutate)
    assert all(check["passed"] is True
               for check in mutated["checks"].values())
    _refresh_review_runtime_hash(session)

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


@pytest.mark.parametrize("field,value", [
    ("active", True),
    ("pending", True),
    ("fault_latched", True),
    ("cancel_target", "/amr/mission/navigate_to_pose"),
    ("run_generation", 0),
    ("exploration_received_at",
     lambda runtime: runtime["checked_monotonic"] -
     acceptance.STATUS_MAX_AGE_SECONDS - 0.1),
])
def test_promote_revalidates_each_persisted_exploration_field(
        tmp_path, field, value):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="STOPPED")
    _write_quality_review(session, manifest)
    acceptance.accept_candidate(session, "candidate")

    def mutate(runtime):
        _set_exploration_field(
            runtime, field, value(runtime) if callable(value) else value)

    mutated = _mutate_saved_runtime(session, mutate)
    assert all(check["passed"] is True
               for check in mutated["checks"].values())
    _refresh_review_runtime_hash(session)
    _refresh_acceptance_evidence_hashes(session)

    with pytest.raises(ValueError):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


@pytest.mark.parametrize("state", ["STOPPED", "COMPLETE"])
def test_accept_autonomous_terminal_outcome_with_matching_runtime_hash(
        tmp_path, state):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state=state)
    review_path = _write_quality_review(session, manifest)
    receipt = acceptance.accept_candidate(session, "candidate")

    assert receipt["state"] == "PROMOTION_ELIGIBLE"
    assert receipt["exploration_outcome"] == state
    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    assert review["runtime_acceptance_sha256"] == artifacts.sha256_file(
        session / acceptance.RUNTIME_ACCEPTANCE_NAME)


@pytest.mark.parametrize("field,value", [
    ("incomplete_acknowledgement", "missing"),
    ("incomplete_acknowledgement", "ACCEPT"),
    ("incomplete_explanation", "missing"),
    ("incomplete_explanation", "   "),
    ("incomplete_explanation", 42),
    ("runtime_acceptance_sha256", "missing"),
    ("runtime_acceptance_sha256", "0" * 64),
])
def test_accept_incomplete_requires_acknowledgement_explanation_and_runtime_hash(
        tmp_path, field, value):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="INCOMPLETE")
    review_path = _write_quality_review(
        session, manifest,
        incomplete_acknowledgement="ACCEPT_INCOMPLETE",
        incomplete_explanation="No safe reachable frontier remained.")
    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    if value == "missing":
        review.pop(field)
    else:
        review[field] = value
    review_path.write_text(yaml.safe_dump(review), encoding="utf-8")

    with pytest.raises(ValueError):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


def test_accept_incomplete_preserves_outcome_with_exact_review_proof(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="INCOMPLETE")
    _write_quality_review(
        session, manifest,
        incomplete_acknowledgement="ACCEPT_INCOMPLETE",
        incomplete_explanation="Remaining frontiers were not safely reachable.")

    receipt = acceptance.accept_candidate(session, "candidate")
    saved = yaml.safe_load(
        (session / acceptance.MAPPING_ACCEPTANCE_NAME).read_text(encoding="utf-8"))
    assert receipt["exploration_outcome"] == "INCOMPLETE"
    assert saved["exploration_outcome"] == "INCOMPLETE"


@pytest.mark.parametrize("mutation", ["missing", "mismatched"])
def test_accept_manual_requires_matching_runtime_acceptance_hash(tmp_path, mutation):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    review_path = _write_quality_review(session, manifest)
    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    if mutation == "missing":
        review.pop("runtime_acceptance_sha256")
    else:
        review["runtime_acceptance_sha256"] = "0" * 64
    review_path.write_text(yaml.safe_dump(review), encoding="utf-8")

    with pytest.raises(ValueError, match="runtime acceptance hash"):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


def test_accept_requires_matching_artifact_runtime_and_quality_hashes(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    runtime = _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)
    receipt = acceptance.accept_candidate(session, "candidate")
    assert receipt["state"] == "PROMOTION_ELIGIBLE"

    forged = dict(runtime)
    forged["candidate_sha256"] = "0" * 64
    (session / acceptance.RUNTIME_ACCEPTANCE_NAME).write_text(
        yaml.safe_dump(forged), encoding="utf-8")
    with pytest.raises(ValueError, match="candidate hash"):
        acceptance.accept_candidate(session, "candidate")


@pytest.mark.parametrize(
    ("decision", "accepted"),
    [("REJECTED", True), ("ACCEPTED", False)],
)
def test_quality_review_decision_and_accepted_field_must_agree(
        tmp_path, decision, accepted):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    review_path = _write_quality_review(session, manifest)
    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    review.update({"decision": decision, "accepted": accepted})
    review_path.write_text(yaml.safe_dump(review), encoding="utf-8")

    with pytest.raises(ValueError, match="quality review"):
        acceptance.accept_candidate(session, "candidate")
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


def test_accept_rejects_nested_symlink_quality_review_path(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    review_dir = session / "review-real"
    review_dir.mkdir()
    review_path = review_dir / "quality_review.yaml"
    review_path.write_text(yaml.safe_dump({
        "schema": 1,
        "decision": "ACCEPTED",
        "accepted": True,
        "reviewer": "commissioning-reviewer",
        "candidate_path": manifest["candidate_path"],
        "candidate_sha256": manifest["candidate_sha256"],
        "runtime_acceptance_sha256": artifacts.sha256_file(
            session / acceptance.RUNTIME_ACCEPTANCE_NAME),
        "reviewed_unix": time.time(),
    }), encoding="utf-8")
    review_link = session / "review-link"
    review_link.symlink_to(review_dir, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        acceptance.accept_candidate(
            session, "candidate",
            quality_review=str(review_link / "quality_review.yaml"))
    assert not (session / acceptance.MAPPING_ACCEPTANCE_NAME).exists()


@pytest.mark.parametrize("mutation", [
    lambda receipt: receipt.pop("session_path"),
    lambda receipt: receipt.update(unexpected="forged"),
    lambda receipt: receipt.update(kind="other_kind"),
    lambda receipt: receipt.update(state="SAVED"),
    lambda receipt: receipt.update(candidate_name="other-candidate"),
    lambda receipt: receipt.update(candidate_path="/other/candidate.yaml"),
    lambda receipt: receipt.update(candidate_sha256="0" * 64),
    lambda receipt: receipt.update(artifact_bundle_sha256="0" * 64),
])
def test_promotion_revalidates_mapping_receipt_envelope_fields(
        tmp_path, mutation):
    _assert_acceptance_receipt_mutation_rejected(tmp_path, mutation)


@pytest.mark.parametrize("field,value", [
    ("schema", _MISSING),
    ("schema", True),
    ("schema", 1.0),
    ("schema", 2),
    ("created_unix", _MISSING),
    ("created_unix", True),
    ("created_unix", "1"),
    ("created_unix", float("nan")),
    ("created_unix", float("inf")),
    ("created_unix", 0.0),
    ("created_unix", -1.0),
])
def test_promotion_revalidates_mapping_receipt_schema_and_timestamp(
        tmp_path, field, value):
    def mutate(receipt):
        if value is _MISSING:
            receipt.pop(field)
        else:
            receipt[field] = value

    _assert_acceptance_receipt_mutation_rejected(tmp_path, mutate)


@pytest.mark.parametrize("mutation", [
    lambda receipt: receipt["conditions"].pop("artifact_manifest"),
    lambda receipt: receipt["conditions"].update(runtime_acceptance="FAIL"),
    lambda receipt: receipt["conditions"].update(extra="forged"),
    lambda receipt: receipt.update(conditions=["VALIDATED", "PASS", "ACCEPTED"]),
])
def test_promotion_revalidates_mapping_receipt_conditions(tmp_path, mutation):
    _assert_acceptance_receipt_mutation_rejected(tmp_path, mutation)


@pytest.mark.parametrize("mutation", [
    lambda receipt: receipt["artifact_manifest"].pop("sha256"),
    lambda receipt: receipt["artifact_manifest"].update(extra="forged"),
    lambda receipt: receipt["artifact_manifest"].update(
        path="/other/mapping_manifest.yaml"),
    lambda receipt: receipt["artifact_manifest"].update(sha256="0" * 64),
    lambda receipt: receipt["runtime_acceptance"].pop("mode"),
    lambda receipt: receipt["runtime_acceptance"].update(extra="forged"),
    lambda receipt: receipt["runtime_acceptance"].update(
        path="/other/runtime_acceptance.yaml"),
    lambda receipt: receipt["runtime_acceptance"].update(sha256="0" * 64),
    lambda receipt: receipt["runtime_acceptance"].update(mode="autonomous"),
    lambda receipt: receipt["quality_review"].pop("sha256"),
    lambda receipt: receipt["quality_review"].update(extra="forged"),
    lambda receipt: receipt["quality_review"].update(
        path="/other/quality_review.yaml"),
    lambda receipt: receipt["quality_review"].update(sha256="0" * 64),
    lambda receipt: receipt["quality_review"].update(
        reviewer="other-reviewer"),
    lambda receipt: receipt["quality_review"].update(reviewer=42),
])
def test_promotion_revalidates_mapping_receipt_evidence_entries(
        tmp_path, mutation):
    _assert_acceptance_receipt_mutation_rejected(tmp_path, mutation)


def test_promotion_revalidates_actual_quality_reviewer_type(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    review_path = _write_quality_review(session, manifest)
    acceptance.accept_candidate(session, "candidate")

    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    review["reviewer"] = 42
    artifacts.atomic_write_yaml(review_path, review)
    _mutate_saved_acceptance(
        session,
        lambda receipt: receipt["quality_review"].update(
            sha256=artifacts.sha256_file(review_path), reviewer=42))

    with pytest.raises(ValueError, match="quality review reviewer"):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


def test_promotion_rejects_manual_receipt_exploration_outcome(tmp_path):
    _assert_acceptance_receipt_mutation_rejected(
        tmp_path, lambda receipt: receipt.update(exploration_outcome="STOPPED"))


@pytest.mark.parametrize("outcome", [_MISSING, "RUNNING"])
def test_promotion_requires_valid_autonomous_receipt_outcome(tmp_path, outcome):
    def mutate(receipt):
        if outcome is _MISSING:
            receipt.pop("exploration_outcome")
        else:
            receipt["exploration_outcome"] = outcome

    _assert_acceptance_receipt_mutation_rejected(
        tmp_path, mutate, mode="autonomous", state="STOPPED")


def test_promotion_requires_validated_manifest_state(tmp_path):
    session, _manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, _manifest)
    _write_quality_review(session, _manifest)
    acceptance.accept_candidate(session, "candidate")

    manifest_path = session / artifacts.MANIFEST_NAME
    saved_manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    saved_manifest["state"] = "SAVED"
    artifacts.atomic_write_yaml(manifest_path, saved_manifest)
    acceptance_path = session / acceptance.MAPPING_ACCEPTANCE_NAME
    receipt = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    receipt["artifact_manifest"]["sha256"] = artifacts.sha256_file(manifest_path)
    artifacts.atomic_write_yaml(acceptance_path, receipt)

    with pytest.raises(ValueError, match="VALIDATED"):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()
    assert artifacts.verify_manifest(session, "candidate")["state"] == "SAVED"


def test_promotion_revalidates_incomplete_review_requirements(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="INCOMPLETE")
    review_path = _write_quality_review(
        session, manifest,
        incomplete_acknowledgement="ACCEPT_INCOMPLETE",
        incomplete_explanation="No safe reachable frontier remained.")
    acceptance.accept_candidate(session, "candidate")

    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    review.pop("incomplete_acknowledgement")
    review_path.write_text(yaml.safe_dump(review), encoding="utf-8")

    with pytest.raises(ValueError, match="incomplete acknowledgement"):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


def test_promotion_rejects_changed_mapping_exploration_outcome(tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(
        session, manifest, mode="autonomous", state="STOPPED")
    _write_quality_review(session, manifest)
    acceptance.accept_candidate(session, "candidate")

    acceptance_path = session / acceptance.MAPPING_ACCEPTANCE_NAME
    receipt = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    receipt["exploration_outcome"] = "COMPLETE"
    artifacts.atomic_write_yaml(acceptance_path, receipt)

    with pytest.raises(ValueError, match="exploration outcome"):
        acceptance.promote_candidate(session, "candidate")
    assert not (session / acceptance.PROMOTION_RECEIPT_NAME).exists()


def test_promotion_writes_only_eligibility_receipt_and_canonical_hashes_stay_same(
        tmp_path):
    session, manifest = _write_validated_session(tmp_path)
    _write_passing_runtime_report(session, manifest)
    _write_quality_review(session, manifest)
    acceptance.accept_candidate(session, "candidate")

    canonical = tmp_path / "canonical"
    canonical.mkdir()
    canonical_yaml = canonical / "factory.yaml"
    canonical_image = canonical / "factory.pgm"
    canonical_yaml.write_text("image: factory.pgm\n", encoding="utf-8")
    canonical_image.write_bytes(b"canonical")
    before = {path: artifacts.sha256_file(path)
              for path in (canonical_yaml, canonical_image)}
    receipt = acceptance.promote_candidate(
        session, "candidate", canonical_dirs={canonical})
    after = {path: artifacts.sha256_file(path)
             for path in (canonical_yaml, canonical_image)}
    assert receipt["state"] == "PROMOTION_ELIGIBLE"
    assert receipt["canonical_map_unchanged"] is True
    assert before == after
    assert (session / acceptance.PROMOTION_RECEIPT_NAME).is_file()
