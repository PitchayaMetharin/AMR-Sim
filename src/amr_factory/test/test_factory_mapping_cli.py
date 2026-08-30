from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import pytest
import yaml

import factory_mapping_cli
from factory_mapping_cli import _candidate_prefix, _validate_artifacts, transform_origin


def test_transform_origin_applies_surveyed_datum():
    result = transform_origin([1.0, 0.0, 0.0], [10.0, 20.0, 1.5707963267948966])
    assert abs(result[0] - 10.0) < 1e-9
    assert abs(result[1] - 21.0) < 1e-9
    assert abs(result[2] - 1.5707963267948966) < 1e-9


def test_transform_origin_wraps_yaw():
    result = transform_origin([0.0, 0.0, 3.5], [0.0, 0.0, 3.5])
    assert -3.141592653589793 <= result[2] <= 3.141592653589793


def test_candidate_prefix_rejects_canonical_map_directory(tmp_path, monkeypatch):
    canonical = tmp_path / "canonical"
    canonical.mkdir()
    monkeypatch.setattr(factory_mapping_cli, "_canonical_maps_dir", lambda: canonical)
    with pytest.raises(ValueError, match="canonical"):
        _candidate_prefix(str(canonical), "candidate")


def test_validate_artifacts_requires_map_image_and_pose_graph(tmp_path, monkeypatch):
    canonical = tmp_path / "canonical"
    canonical.mkdir()
    monkeypatch.setattr(factory_mapping_cli, "_canonical_maps_dir", lambda: canonical)
    session = tmp_path / "session"
    session.mkdir()
    prefix = session / "candidate"
    image = session / "candidate.pgm"
    image.write_bytes(b"P5\n1 1\n255\n0")
    Path(str(prefix) + ".posegraph").write_text("graph", encoding="utf-8")
    Path(str(prefix) + ".yaml").write_text(yaml.safe_dump({
        "image": image.name,
        "resolution": 0.05,
        "origin": [0.0, 0.0, 0.0],
    }), encoding="utf-8")
    yaml_path, manifest = _validate_artifacts(prefix, [0.0, 0.0, 0.0])
    assert yaml_path == Path(str(prefix) + ".yaml")
    assert manifest["validation"] == "passed"
