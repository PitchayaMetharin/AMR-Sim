from pathlib import Path
from types import ModuleType, SimpleNamespace
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import pytest  # noqa: E402
import yaml  # noqa: E402

import factory_mapping_artifacts as artifacts  # noqa: E402
import factory_mapping_cli  # noqa: E402
from factory_mapping_cli import (  # noqa: E402
    _candidate_prefix, _validate_artifacts, transform_origin)


@pytest.fixture
def canonical_dir(tmp_path, monkeypatch):
    canonical = tmp_path / "canonical"
    canonical.mkdir()
    monkeypatch.setattr(factory_mapping_cli, "_canonical_maps_dir", lambda: canonical)
    return canonical


def _write_candidate(
        session, name="candidate", image_suffix=".pgm", image_value=None,
        **overrides):
    session.mkdir(parents=True, exist_ok=True)
    image = session / f"{name}{image_suffix}"
    image.write_bytes(b"P5\n1 1\n255\n0")
    graph = session / f"{name}.posegraph"
    graph.write_text("graph", encoding="utf-8")
    graph_data = session / f"{name}.data"
    graph_data.write_text("data", encoding="utf-8")
    document = {
        "image": image.name if image_value is None else image_value,
        "resolution": 0.05,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }
    document.update(overrides)
    yaml_path = session / f"{name}.yaml"
    yaml_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return yaml_path, image, graph, graph_data


def _write_manifest(session, name="candidate", canonical=()):
    manifest = artifacts.build_manifest(
        session, session / name, [0.0, 0.0, 0.0], canonical_dirs=canonical)
    artifacts.atomic_write_yaml(session / artifacts.MANIFEST_NAME, manifest)
    return manifest


def test_transform_origin_applies_surveyed_datum():
    result = transform_origin([1.0, 0.0, 0.0], [10.0, 20.0, 1.5707963267948966])
    assert abs(result[0] - 10.0) < 1e-9
    assert abs(result[1] - 21.0) < 1e-9
    assert abs(result[2] - 1.5707963267948966) < 1e-9


def test_transform_origin_wraps_yaw():
    result = transform_origin([0.0, 0.0, 3.5], [0.0, 0.0, 3.5])
    assert -3.141592653589793 <= result[2] <= 3.141592653589793


def test_transform_origin_rejects_nonfinite_datum():
    with pytest.raises(ValueError, match="finite"):
        transform_origin([0.0, 0.0, 0.0], [float("nan"), 0.0, 0.0])


def test_candidate_prefix_rejects_canonical_map_directory(canonical_dir):
    with pytest.raises(ValueError, match="canonical"):
        _candidate_prefix(str(canonical_dir), "candidate")


def _install_service_fakes(
        monkeypatch, calls, *, map_saver_result=0, use_sim_time=False):
    rclpy = ModuleType("rclpy")
    slam_toolbox = ModuleType("slam_toolbox")
    slam_srv = ModuleType("slam_toolbox.srv")

    class FakeFuture:
        def __init__(self, result):
            self._result = SimpleNamespace(result=result)

        def done(self):
            return True

        def result(self):
            return self._result

    class FakeSerializePoseGraph:
        class Request:
            def __init__(self, filename=""):
                self.filename = filename

        class Response:
            RESULT_SUCCESS = 1

    class FakeClient:
        def wait_for_service(self, timeout_sec):
            assert timeout_sec == 5.0
            return True

        def call_async(self, request):
            calls.append(("serialize_map", request.filename))
            Path(request.filename + ".posegraph").write_text(
                "graph", encoding="utf-8")
            Path(request.filename + ".data").write_text(
                "data", encoding="utf-8")
            return FakeFuture(FakeSerializePoseGraph.Response.RESULT_SUCCESS)

    class FakeNode:
        def create_client(self, service_type, service_name):
            assert service_type is FakeSerializePoseGraph
            assert service_name == "/amr/slam_toolbox/serialize_map"
            return FakeClient()

        def destroy_node(self):
            pass

    class FakeParameter:
        def __init__(self, name, value_type, value):
            assert name == "use_sim_time"
            assert value_type is rclpy.Parameter.Type.BOOL
            assert value is use_sim_time

    rclpy.Parameter = SimpleNamespace(Type=SimpleNamespace(BOOL=object()))
    rclpy.parameter = SimpleNamespace(Parameter=FakeParameter)
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None
    rclpy.create_node = lambda *args, **kwargs: FakeNode()
    rclpy.spin_until_future_complete = lambda node, future, timeout_sec: None
    slam_srv.SerializePoseGraph = FakeSerializePoseGraph
    slam_toolbox.srv = slam_srv
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "slam_toolbox", slam_toolbox)
    monkeypatch.setitem(sys.modules, "slam_toolbox.srv", slam_srv)

    def fake_run(command, *, check, timeout):
        calls.append(("map_saver", command, check, timeout))
        if isinstance(map_saver_result, BaseException):
            raise map_saver_result
        return SimpleNamespace(returncode=map_saver_result)

    monkeypatch.setattr(factory_mapping_cli.subprocess, "run", fake_run)


def test_service_call_uses_explicit_map_topic_and_qos_before_bare_prefix_serializer(
        tmp_path, monkeypatch):
    calls = []
    _install_service_fakes(monkeypatch, calls, use_sim_time=False)

    session = tmp_path / "session"
    session.mkdir()
    prefix = session / "candidate"
    factory_mapping_cli._service_call(session, prefix, use_sim_time=False)

    assert calls == [
        ("map_saver", [
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-f", str(prefix), "-t", "/map",
            "--ros-args", "-p", "map_subscribe_transient_local:=true",
        ], False, factory_mapping_cli.MAP_SAVER_TIMEOUT_SEC),
        ("serialize_map", str(prefix)),
    ]
    assert (session / "candidate.posegraph").read_text(encoding="utf-8") == "graph"
    assert (session / "candidate.data").read_text(encoding="utf-8") == "data"


@pytest.mark.parametrize("failure", ["nonzero", "timeout"])
def test_map_saver_failure_stops_before_serialization_and_manifest(
        tmp_path, canonical_dir, monkeypatch, failure):
    calls = []
    outcome = (
        23 if failure == "nonzero" else
        subprocess.TimeoutExpired("ros2", factory_mapping_cli.MAP_SAVER_TIMEOUT_SEC)
    )
    _install_service_fakes(
        monkeypatch, calls, map_saver_result=outcome, use_sim_time=True)

    session = tmp_path / "session"
    with pytest.raises(RuntimeError, match=(
            "exit code 23" if failure == "nonzero" else "timed out")):
        factory_mapping_cli.save(SimpleNamespace(
            session_dir=str(session), name="candidate", datum_x=0.0,
            datum_y=0.0, datum_yaw=0.0, use_sim_time=True))

    assert [call[0] for call in calls] == ["map_saver"]
    assert not (session / artifacts.MANIFEST_NAME).exists()


@pytest.mark.parametrize(
    "suffix", [".yaml", ".pgm", ".png", ".bmp", ".posegraph", ".data"])
def test_candidate_preflight_rejects_reserved_outputs_before_service_call(
        tmp_path, canonical_dir, monkeypatch, suffix):
    session = tmp_path / "session"
    session.mkdir()
    (session / f"candidate{suffix}").write_text("existing", encoding="utf-8")
    calls = []
    monkeypatch.setattr(
        factory_mapping_cli, "_service_call", lambda *args: calls.append(args))

    with pytest.raises(ValueError, match="output already exists"):
        factory_mapping_cli.save(SimpleNamespace(
            session_dir=str(session), name="candidate", datum_x=0.0,
            datum_y=0.0, datum_yaw=0.0, use_sim_time=True))
    assert calls == []


def test_candidate_preflight_rejects_symlink_candidate_outputs_before_service_call(
        tmp_path, canonical_dir, monkeypatch):
    session = tmp_path / "session"
    session.mkdir()
    calls = []
    monkeypatch.setattr(
        factory_mapping_cli, "_service_call", lambda *args: calls.append(args))
    for suffix in (".yaml", ".pgm", ".png", ".bmp", ".posegraph", ".data"):
        path = session / f"candidate{suffix}"
        target = tmp_path / f"target{suffix}"
        target.write_text("target", encoding="utf-8")
        path.symlink_to(target)

        with pytest.raises(ValueError, match="output already exists"):
            factory_mapping_cli.save(SimpleNamespace(
                session_dir=str(session), name="candidate", datum_x=0.0,
                datum_y=0.0, datum_yaw=0.0, use_sim_time=True))
        path.unlink()

    assert calls == []


def test_candidate_preflight_rejects_dangling_manifest_before_service_call(
        tmp_path, canonical_dir, monkeypatch):
    session = tmp_path / "session"
    session.mkdir()
    (session / artifacts.MANIFEST_NAME).symlink_to(
        session / "missing-manifest.yaml")
    calls = []
    monkeypatch.setattr(
        factory_mapping_cli, "_service_call", lambda *args: calls.append(args))

    with pytest.raises(ValueError, match="manifest already exists"):
        factory_mapping_cli.save(SimpleNamespace(
            session_dir=str(session), name="candidate", datum_x=0.0,
            datum_y=0.0, datum_yaw=0.0, use_sim_time=True))
    assert calls == []


@pytest.mark.parametrize("name", [
    "mapping_manifest", "runtime_acceptance", "quality_review",
    "mapping_acceptance", "promotion_eligibility", "mapping_manifest.yaml",
])
def test_candidate_preflight_rejects_reserved_names_before_service_call(
        tmp_path, canonical_dir, monkeypatch, name):
    session = tmp_path / "session"
    session.mkdir()
    calls = []
    monkeypatch.setattr(
        factory_mapping_cli, "_service_call", lambda *args: calls.append(args))

    with pytest.raises(ValueError, match="reserved session output"):
        factory_mapping_cli.save(SimpleNamespace(
            session_dir=str(session), name=name, datum_x=0.0,
            datum_y=0.0, datum_yaw=0.0, use_sim_time=True))
    assert calls == []


@pytest.mark.parametrize("name", [
    "mapping_manifest", "runtime_acceptance", "quality_review",
    "mapping_acceptance", "promotion_eligibility", "mapping_manifest.yaml",
])
def test_build_manifest_rejects_reserved_candidate_names(
        tmp_path, canonical_dir, name):
    session = tmp_path / "session"
    _write_candidate(session, name=name)

    with pytest.raises(ValueError, match="reserved session output"):
        artifacts.build_manifest(
            session, session / name, [0.0, 0.0, 0.0],
            canonical_dirs={canonical_dir})


@pytest.mark.parametrize("name", [
    "mapping_manifest", "runtime_acceptance", "quality_review",
    "mapping_acceptance", "promotion_eligibility", "mapping_manifest.yaml",
])
def test_verify_manifest_rejects_reserved_candidate_names(
        tmp_path, canonical_dir, name):
    session = tmp_path / "session"
    _write_candidate(session)
    manifest = _write_manifest(session, canonical={canonical_dir})
    manifest["candidate_name"] = name
    artifacts.atomic_write_yaml(session / artifacts.MANIFEST_NAME, manifest)

    with pytest.raises(ValueError, match="reserved session output"):
        artifacts.verify_manifest(session, canonical_dirs={canonical_dir})


def test_validate_artifacts_records_complete_provenance(tmp_path, canonical_dir):
    session = tmp_path / "session"
    yaml_path, image, graph, graph_data = _write_candidate(session)
    actual_yaml, manifest = _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])
    assert actual_yaml == yaml_path
    assert manifest["state"] == "SAVED"
    assert manifest["candidate_path"] == str(yaml_path.resolve())
    assert manifest["artifacts"]["map_image"]["size"] == image.stat().st_size
    assert manifest["artifacts"]["pose_graph"]["sha256"] == artifacts.sha256_file(graph)
    assert manifest["artifacts"]["pose_graph_data"]["path"] == str(graph_data.resolve())
    assert manifest["artifacts"]["pose_graph_data"]["sha256"] == artifacts.sha256_file(
        graph_data)
    assert set(manifest["artifacts"]) == {
        "map_yaml", "map_image", "pose_graph", "pose_graph_data"}
    assert manifest["map"]["mode"] == "trinary"


@pytest.mark.parametrize("suffix", [".pgm", ".png", ".bmp"])
@pytest.mark.parametrize("spelling", ["relative", "dot", "absolute"])
def test_validate_accepts_exact_candidate_image_with_safe_spellings(
        tmp_path, canonical_dir, suffix, spelling):
    session = tmp_path / "session"
    image_name = f"candidate{suffix}"
    image_value = {
        "relative": image_name,
        "dot": f"./{image_name}",
        "absolute": str((session / image_name).resolve()),
    }[spelling]
    _yaml_path, image, _graph, _graph_data = _write_candidate(
        session, image_suffix=suffix, image_value=image_value)

    manifest = _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])[1]
    assert manifest["artifacts"]["map_image"]["path"] == str(image.resolve())


@pytest.mark.parametrize("image_name", ["runtime.log", "other.pgm", "candidate.tif"])
def test_validate_rejects_non_candidate_image_identity(
        tmp_path, canonical_dir, image_name):
    session = tmp_path / "session"
    yaml_path, _image, _graph, _graph_data = _write_candidate(session)
    unrelated = session / image_name
    unrelated.write_bytes(b"unrelated")
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    document["image"] = image_name
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError, match="candidate prefix"):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])


def test_bundle_hash_changes_when_either_pose_graph_artifact_changes(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    _yaml_path, _image, graph, graph_data = _write_candidate(session)
    original = _validate_artifacts(
        session / "candidate", [0.0, 0.0, 0.0])[1]

    graph.write_text("changed graph", encoding="utf-8")
    graph_changed = _validate_artifacts(
        session / "candidate", [0.0, 0.0, 0.0])[1]
    assert graph_changed["artifact_bundle_sha256"] != original["artifact_bundle_sha256"]
    assert graph_changed["artifacts"]["pose_graph"]["sha256"] != \
        original["artifacts"]["pose_graph"]["sha256"]

    graph.write_text("graph", encoding="utf-8")
    graph_data.write_text("changed data", encoding="utf-8")
    data_changed = _validate_artifacts(
        session / "candidate", [0.0, 0.0, 0.0])[1]
    assert data_changed["artifact_bundle_sha256"] != original["artifact_bundle_sha256"]
    assert data_changed["artifacts"]["pose_graph_data"]["sha256"] != \
        original["artifacts"]["pose_graph_data"]["sha256"]


@pytest.mark.parametrize(
    ("overrides", "match"),
    [
        ({"resolution": float("nan")}, "finite"),
        ({"resolution": 0.04}, "exactly 0.05"),
        ({"free_thresh": float("inf")}, "finite"),
        ({"free_thresh": 0.8, "occupied_thresh": 0.7}, "thresholds"),
        ({"mode": "unknown"}, "mode"),
        ({"negate": 2}, "negate"),
        ({"origin": [0.0, 0.0, float("inf")]}, "finite"),
    ],
)
def test_validate_rejects_invalid_map_server_fields(
        tmp_path, canonical_dir, overrides, match):
    session = tmp_path / "session"
    _write_candidate(session, **overrides)
    with pytest.raises(ValueError, match=match):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])


@pytest.mark.parametrize("missing", [
    "image", "resolution", "origin", "negate", "occupied_thresh",
    "free_thresh", "mode",
])
def test_validate_rejects_missing_map_server_field(tmp_path, canonical_dir, missing):
    session = tmp_path / "session"
    yaml_path, _image, _graph, _graph_data = _write_candidate(session)
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    del document[missing]
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="missing required"):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])


@pytest.mark.parametrize(
    ("suffix", "message"),
    [(".posegraph", "pose graph"), (".data", "pose graph data")],
)
@pytest.mark.parametrize("condition", ["missing", "empty"])
def test_validate_rejects_missing_or_empty_pose_graph_artifacts(
        tmp_path, canonical_dir, suffix, message, condition):
    session = tmp_path / "session"
    _yaml_path, _image, _graph, _graph_data = _write_candidate(session)
    path = session / f"candidate{suffix}"
    if condition == "missing":
        path.unlink()
    else:
        path.write_bytes(b"")
    with pytest.raises(ValueError, match=message):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])


def test_validate_rejects_empty_image(tmp_path, canonical_dir):
    session = tmp_path / "session"
    _yaml_path, image, _graph, _graph_data = _write_candidate(session)
    image.write_bytes(b"")
    with pytest.raises(ValueError, match="map image.*non-empty"):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])


@pytest.mark.parametrize("suffix", [".posegraph", ".data"])
def test_verify_rejects_corrupt_pose_graph_artifact(
        tmp_path, canonical_dir, suffix):
    session = tmp_path / "session"
    _write_candidate(session)
    _write_manifest(session, canonical={canonical_dir})
    (session / f"candidate{suffix}").write_text("corrupt", encoding="utf-8")
    with pytest.raises(ValueError, match="hash does not match"):
        artifacts.verify_manifest(session, "candidate", canonical_dirs={canonical_dir})


@pytest.mark.parametrize("mutation", ["schema2", "single_file", "incomplete"])
def test_verify_rejects_legacy_single_file_and_incomplete_manifests(
        tmp_path, canonical_dir, mutation):
    session = tmp_path / "session"
    _write_candidate(session)
    _write_manifest(session, canonical={canonical_dir})
    manifest_path = session / artifacts.MANIFEST_NAME
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if mutation == "schema2":
        manifest["schema"] = 2
        manifest["schema_version"] = 2
    elif mutation == "single_file":
        del manifest["artifacts"]["pose_graph_data"]
    else:
        del manifest["artifacts"]["pose_graph_data"]["size"]
    artifacts.atomic_write_yaml(manifest_path, manifest)

    with pytest.raises(ValueError):
        artifacts.verify_manifest(session, "candidate", canonical_dirs={canonical_dir})


def test_validate_rejects_escaped_or_symlink_image(tmp_path, canonical_dir):
    session = tmp_path / "session"
    yaml_path, image, _graph, _graph_data = _write_candidate(session)
    outside = tmp_path / "outside.pgm"
    outside.write_bytes(b"outside")
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    document["image"] = "../outside.pgm"
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="path traversal"):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])

    document["image"] = "nested/../candidate.pgm"
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="path traversal"):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])

    document["image"] = image.name
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")
    image.unlink()
    image.symlink_to(outside)
    with pytest.raises(ValueError, match="symlink"):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])


def test_validate_rejects_external_alias_absolute_image_path(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    yaml_path, _image, _graph, _graph_data = _write_candidate(session)
    session_alias = tmp_path / "session-alias"
    session_alias.symlink_to(session, target_is_directory=True)
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    document["image"] = str(session_alias / "candidate.pgm")
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError, match="symlink"):
        _validate_artifacts(session / "candidate", [0.0, 0.0, 0.0])


def test_build_manifest_rejects_nested_symlink_candidate_path(tmp_path, canonical_dir):
    session = tmp_path / "session"
    real = session / "real"
    _write_candidate(real)
    link = session / "link"
    link.symlink_to(real, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        artifacts.build_manifest(
            session, link / "candidate", [0.0, 0.0, 0.0],
            canonical_dirs={canonical_dir})


def test_build_manifest_rejects_external_alias_candidate_prefix(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    session_alias = tmp_path / "session-alias"
    session_alias.symlink_to(session, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        artifacts.build_manifest(
            session_alias, session_alias / "candidate", [0.0, 0.0, 0.0],
            canonical_dirs={canonical_dir})


def test_build_manifest_rejects_external_alias_session_dir(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    session_alias = tmp_path / "session-alias"
    session_alias.symlink_to(session, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        artifacts.build_manifest(
            session_alias, session / "candidate", [0.0, 0.0, 0.0],
            canonical_dirs={canonical_dir})


def test_build_manifest_rejects_session_dir_traversal(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    traversed_session = session / "missing" / ".."

    with pytest.raises(ValueError, match="session directory.*path traversal"):
        artifacts.build_manifest(
            traversed_session, session / "candidate", [0.0, 0.0, 0.0],
            canonical_dirs={canonical_dir})


def test_build_manifest_rejects_candidate_prefix_traversal(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    traversed_prefix = session / "missing" / ".." / "candidate"

    with pytest.raises(ValueError, match="candidate prefix.*path traversal"):
        artifacts.build_manifest(
            session, traversed_prefix, [0.0, 0.0, 0.0],
            canonical_dirs={canonical_dir})


def test_validate_map_document_rejects_external_alias_candidate_prefix(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    yaml_path, _image, _graph, _graph_data = _write_candidate(session)
    session_alias = tmp_path / "session-alias"
    session_alias.symlink_to(session, target_is_directory=True)
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))

    with pytest.raises(ValueError, match="symlink"):
        artifacts.validate_map_document(
            document, yaml_path, session, canonical_dirs={canonical_dir},
            candidate_prefix=session_alias / "candidate")


def test_verify_manifest_rejects_recomputed_external_alias_image(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    yaml_path, _image, _graph, _graph_data = _write_candidate(session)
    manifest = _write_manifest(session, canonical={canonical_dir})
    session_alias = tmp_path / "session-alias"
    session_alias.symlink_to(session, target_is_directory=True)
    alias_image = str(session_alias / "candidate.pgm")

    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    document["image"] = alias_image
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")
    manifest["map"]["image"] = alias_image
    manifest["map"]["image_relative_path"] = "candidate.pgm"
    manifest["artifacts"]["map_yaml"]["size"] = yaml_path.stat().st_size
    manifest["artifacts"]["map_yaml"]["sha256"] = artifacts.sha256_file(yaml_path)
    manifest["candidate_sha256"] = manifest["artifacts"]["map_yaml"]["sha256"]
    manifest["artifact_bundle_sha256"] = artifacts._bundle_sha256(
        manifest["artifacts"])
    artifacts.atomic_write_yaml(session / artifacts.MANIFEST_NAME, manifest)

    with pytest.raises(ValueError, match="symlink"):
        artifacts.verify_manifest(
            session, "candidate", canonical_dirs={canonical_dir})


def test_validate_requires_saved_manifest_and_does_not_manufacture_one(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    with pytest.raises(ValueError, match="mapping manifest"):
        artifacts.verify_manifest(session, "candidate", canonical_dirs={canonical_dir})
    assert not (session / artifacts.MANIFEST_NAME).exists()


def test_validate_transitions_only_a_verified_manifest(tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    manifest = _write_manifest(session, canonical={canonical_dir})
    assert artifacts.verify_manifest(
        session, "candidate", canonical_dirs={canonical_dir})["state"] == "SAVED"
    factory_mapping_cli.validate(SimpleNamespace(
        session_dir=str(session), name="candidate"))
    assert artifacts.verify_manifest(
        session, "candidate", canonical_dirs={canonical_dir},
        expected_state="VALIDATED")["candidate_sha256"] == manifest["candidate_sha256"]


def test_stale_manifest_and_hash_tampering_fail_closed(tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    _write_manifest(session, canonical={canonical_dir})
    (session / "candidate.pgm").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="hash does not match"):
        artifacts.verify_manifest(session, "candidate", canonical_dirs={canonical_dir})


def test_failed_save_leaves_no_valid_manifest(tmp_path, canonical_dir, monkeypatch):
    session = tmp_path / "session"

    def fail_service(_session, _prefix, _use_sim_time):
        raise RuntimeError("simulated save failure")

    monkeypatch.setattr(factory_mapping_cli, "_service_call", fail_service)
    with pytest.raises(RuntimeError, match="simulated"):
        factory_mapping_cli.save(SimpleNamespace(
            session_dir=str(session), name="candidate", datum_x=0.0,
            datum_y=0.0, datum_yaw=0.0, use_sim_time=True))
    assert not (session / artifacts.MANIFEST_NAME).exists()


def test_forged_manifest_cannot_authorize_discard(tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    _write_manifest(session, canonical={canonical_dir})
    unrelated = session / "unrelated.evidence"
    unrelated.write_text("keep", encoding="utf-8")
    manifest_path = session / artifacts.MANIFEST_NAME
    forged = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    forged["artifacts"]["map_image"]["relative_path"] = unrelated.name
    forged["artifacts"]["map_image"]["path"] = str(unrelated.resolve())
    forged["artifacts"]["map_image"]["size"] = unrelated.stat().st_size
    forged["artifacts"]["map_image"]["sha256"] = artifacts.sha256_file(unrelated)
    manifest_path.write_text(yaml.safe_dump(forged), encoding="utf-8")
    with pytest.raises(ValueError, match="does not match map YAML"):
        artifacts.discard_verified(
            session, "candidate", canonical_dirs={canonical_dir})
    assert unrelated.exists()
    assert manifest_path.exists()


def test_recomputed_manifest_cannot_redirect_discard_to_runtime_log(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    yaml_path, _image, _graph, _graph_data = _write_candidate(session)
    _write_manifest(session, canonical={canonical_dir})
    unrelated = session / "runtime.log"
    unrelated.write_text("preserve", encoding="utf-8")

    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    document["image"] = unrelated.name
    yaml_path.write_text(yaml.safe_dump(document), encoding="utf-8")

    manifest_path = session / artifacts.MANIFEST_NAME
    forged = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    forged_map = dict(forged["map"])
    forged_map["image"] = unrelated.name
    forged_map["image_relative_path"] = unrelated.name
    forged["map"] = forged_map
    forged["artifacts"]["map_image"] = {
        "path": str(unrelated.resolve()),
        "relative_path": unrelated.name,
        "size": unrelated.stat().st_size,
        "sha256": artifacts.sha256_file(unrelated),
    }
    forged["candidate_sha256"] = artifacts.sha256_file(yaml_path)
    forged["artifact_bundle_sha256"] = artifacts._bundle_sha256(
        forged["artifacts"])
    artifacts.atomic_write_yaml(manifest_path, forged)

    with pytest.raises(ValueError, match="candidate prefix"):
        artifacts.discard_verified(
            session, "candidate", canonical_dirs={canonical_dir})
    assert unrelated.read_text(encoding="utf-8") == "preserve"
    assert manifest_path.exists()


def test_discard_removes_only_verified_candidate_and_preserves_evidence(
        tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    _write_manifest(session, canonical={canonical_dir})
    unrelated = session / "runtime.log"
    unrelated.write_text("preserve", encoding="utf-8")
    factory_mapping_cli.discard(SimpleNamespace(
        session_dir=str(session), name="candidate", confirm=True))
    assert session.is_dir()
    assert unrelated.read_text(encoding="utf-8") == "preserve"
    assert not (session / artifacts.MANIFEST_NAME).exists()
    assert not (session / "candidate.yaml").exists()
    assert not (session / "candidate.pgm").exists()
    assert not (session / "candidate.posegraph").exists()
    assert not (session / "candidate.data").exists()


def test_discard_requires_explicit_confirmation(tmp_path, canonical_dir):
    session = tmp_path / "session"
    _write_candidate(session)
    _write_manifest(session, canonical={canonical_dir})
    with pytest.raises(ValueError, match="--confirm"):
        factory_mapping_cli.discard(SimpleNamespace(
            session_dir=str(session), name="candidate", confirm=False))
    assert (session / artifacts.MANIFEST_NAME).exists()
