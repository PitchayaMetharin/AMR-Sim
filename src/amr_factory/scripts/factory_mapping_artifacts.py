#!/usr/bin/env python3
"""Provenance-safe factory mapping artifact and manifest operations.

This module deliberately has no ROS dependency.  The mapping CLI uses it after
SLAM has written a candidate, and the acceptance harness uses it to bind every
runtime and human review decision to the exact saved candidate.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any, Iterable, Mapping, Optional, Sequence

import yaml


MANIFEST_NAME = "mapping_manifest.yaml"
SCHEMA_VERSION = 3
MANIFEST_KIND = "factory_mapping_manifest"
RESERVED_SESSION_OUTPUTS = frozenset({
    MANIFEST_NAME,
    "runtime_acceptance.yaml",
    "quality_review.yaml",
    "mapping_acceptance.yaml",
    "promotion_eligibility.yaml",
})
CANDIDATE_SUFFIXES = (".yaml", ".pgm", ".png", ".bmp", ".posegraph", ".data")
MAP_IMAGE_SUFFIXES = (".pgm", ".png", ".bmp")
REQUIRED_MAP_FIELDS = (
    "image",
    "resolution",
    "origin",
    "negate",
    "occupied_thresh",
    "free_thresh",
    "mode",
)
VALID_MAP_MODES = frozenset({"trinary", "scale", "raw"})
MAP_RESOLUTION = 0.05


class ArtifactError(ValueError):
    """Raised when a mapping artifact or manifest fails closed validation."""


def _finite(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ArtifactError(f"{label} must be numeric and finite") from exc
    if not math.isfinite(number):
        raise ArtifactError(f"{label} must be finite")
    return number


def validate_datum(datum: Sequence[Any]) -> list[float]:
    if isinstance(datum, (str, bytes)) or len(datum) != 3:
        raise ArtifactError("datum must contain finite x, y, yaw")
    return [_finite(value, "datum") for value in datum]


def transform_origin(origin: Sequence[Any], datum: Sequence[Any]) -> list[float]:
    """Transform a map YAML origin from SLAM coordinates into the datum frame."""
    if isinstance(origin, (str, bytes)) or len(origin) != 3:
        raise ArtifactError("origin must contain x, y, yaw")
    values = [_finite(value, "origin") for value in origin]
    dx, dy, dyaw = validate_datum(datum)
    x, y, yaw = values
    cosine, sine = math.cos(dyaw), math.sin(dyaw)
    return [
        dx + cosine * x - sine * y,
        dy + sine * x + cosine * y,
        math.atan2(math.sin(dyaw + yaw), math.cos(dyaw + yaw)),
    ]


def _absolute_path(value: str | os.PathLike[str]) -> Path:
    return Path(os.path.abspath(os.fspath(Path(value).expanduser())))


def _reject_path_traversal(path: Path, label: str) -> None:
    if ".." in path.parts:
        raise ArtifactError(f"{label} must not use path traversal")


def _path_prefixes(path: Path) -> Iterable[Path]:
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        yield current


def _reject_symlink_components(path: Path, stop: Optional[Path] = None) -> None:
    """Reject symlinks in an explicit path, including an existing ancestor."""
    absolute = Path(path).absolute()
    stop = None if stop is None else Path(stop).absolute()
    components = tuple(_path_prefixes(absolute))
    start = 0
    if stop is not None:
        for index, component in enumerate(components):
            if component == stop:
                start = index + 1
                break
    for component in components[start:]:
        if component.is_symlink():
            raise ArtifactError(f"symlinks are not permitted in artifact paths: {component}")
    if absolute.is_symlink():
        raise ArtifactError(f"symlink artifact is not permitted: {absolute}")


def _resolved(value: Path) -> Path:
    try:
        return value.resolve(strict=False)
    except OSError as exc:
        raise ArtifactError(f"cannot resolve path: {value}") from exc


def _inside(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def _reject_canonical(path: Path, canonical_dirs: Iterable[Path]) -> None:
    resolved = _resolved(path)
    for canonical in canonical_dirs:
        canonical_path = _resolved(Path(canonical))
        if _inside(resolved, canonical_path):
            raise ArtifactError("artifact path must not target canonical factory maps")


def _reject_reserved_candidate_prefix(prefix: Path, session_dir: Path) -> None:
    reserved_paths = {session_dir / value for value in RESERVED_SESSION_OUTPUTS}
    if prefix in reserved_paths or any(
            Path(str(prefix) + suffix) in reserved_paths
            for suffix in CANDIDATE_SUFFIXES):
        raise ArtifactError("map name aliases a reserved session output")


def safe_session_dir(
        value: str | os.PathLike[str],
        canonical_dirs: Iterable[Path] = (),
        *,
        create: bool = False) -> Path:
    """Resolve an explicit run directory without following user symlinks."""
    canonical_dirs = tuple(canonical_dirs)
    if value is None or not str(value).strip():
        raise ArtifactError("session directory must be non-empty")
    _reject_path_traversal(Path(value).expanduser(), "session directory")
    raw = _absolute_path(value)
    _reject_symlink_components(raw)
    path = _resolved(raw)
    if path == Path("/"):
        raise ArtifactError("session directory must not be the filesystem root")
    _reject_canonical(path, canonical_dirs)
    if path.exists() and not path.is_dir():
        raise ArtifactError("session directory must be a directory")
    if create:
        path.mkdir(parents=True, exist_ok=True)
    elif not path.is_dir():
        raise ArtifactError("session directory does not exist")
    _reject_symlink_components(path)
    return path


def candidate_prefix(
        session_dir: str | os.PathLike[str],
        name: str,
        canonical_dirs: Iterable[Path] = (),
        *,
        allow_existing: bool = False,
        create_session: bool = False) -> tuple[Path, Path]:
    """Return a directly-contained candidate prefix with no path traversal."""
    canonical_dirs = tuple(canonical_dirs)
    if not isinstance(name, str) or not name.strip():
        raise ArtifactError("map name must be a non-empty simple file prefix")
    name_path = Path(name)
    if name_path.name != name or name in {".", ".."}:
        raise ArtifactError("map name must be a simple file prefix")
    session = safe_session_dir(
        session_dir, canonical_dirs, create=create_session)
    raw_prefix = session / name
    _reject_symlink_components(raw_prefix, stop=session)
    prefix = _resolved(raw_prefix)
    if prefix.parent != session:
        raise ArtifactError("map output must remain directly inside session_dir")
    _reject_symlink_components(prefix, stop=session)
    _reject_canonical(prefix, canonical_dirs)
    marker = session / MANIFEST_NAME
    if not allow_existing and (marker.exists() or marker.is_symlink()):
        raise ArtifactError("mapping manifest already exists; use a new session")
    if not allow_existing:
        _reject_reserved_candidate_prefix(prefix, session)
        for suffix in CANDIDATE_SUFFIXES:
            candidate = Path(str(prefix) + suffix)
            if candidate.exists() or candidate.is_symlink():
                raise ArtifactError(
                    "map output already exists; use a new session or name")
    return session, prefix


def _contained_artifact(
        value: str | os.PathLike[str],
        session_dir: Path,
        canonical_dirs: Iterable[Path],
        label: str,
        *,
        require_regular: bool = True,
        require_nonempty: bool = True) -> Path:
    raw = Path(value)
    _reject_path_traversal(raw, label)
    candidate = raw if raw.is_absolute() else session_dir / raw
    _reject_symlink_components(candidate, stop=session_dir)
    path = _resolved(candidate)
    _reject_symlink_components(path, stop=session_dir)
    if not _inside(path, session_dir):
        raise ArtifactError(f"{label} must remain inside the session directory")
    _reject_canonical(path, canonical_dirs)
    if require_regular and (not path.is_file() or path.is_symlink()):
        raise ArtifactError(f"{label} must be a regular file")
    if require_nonempty and path.stat().st_size <= 0:
        raise ArtifactError(f"{label} must be non-empty")
    return path


def _read_yaml(path: Path, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ArtifactError(f"{label} is not valid YAML: {exc}") from exc
    if not isinstance(value, dict):
        raise ArtifactError(f"{label} must contain a YAML mapping")
    return value


def _normalize_negate(value: Any) -> int:
    if isinstance(value, bool):
        return int(value)
    number = _finite(value, "map negate")
    if number not in (0.0, 1.0):
        raise ArtifactError("map negate must be 0 or 1")
    return int(number)


def validate_map_document(
        document: Mapping[str, Any],
        yaml_path: Path,
        session_dir: Path,
        canonical_dirs: Iterable[Path] = (),
        *,
        candidate_prefix: Path) -> dict[str, Any]:
    """Validate map-server YAML and return normalized metadata."""
    missing = [field for field in REQUIRED_MAP_FIELDS if field not in document]
    if missing:
        raise ArtifactError(
            "map YAML is missing required map-server fields: " + ", ".join(missing))
    image_value = document.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise ArtifactError("map image must be a non-empty path")
    resolution = _finite(document["resolution"], "map resolution")
    if not math.isclose(resolution, MAP_RESOLUTION, rel_tol=0.0, abs_tol=1e-9):
        raise ArtifactError("map resolution must remain exactly 0.05 m")
    origin = document["origin"]
    if isinstance(origin, (str, bytes)) or len(origin) != 3:
        raise ArtifactError("map YAML origin must contain x, y, yaw")
    normalized_origin = [_finite(value, "map origin") for value in origin]
    free_thresh = _finite(document["free_thresh"], "map free_thresh")
    occupied_thresh = _finite(
        document["occupied_thresh"], "map occupied_thresh")
    if not (0.0 <= free_thresh < occupied_thresh <= 1.0):
        raise ArtifactError(
            "map thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1")
    mode = document["mode"]
    if not isinstance(mode, str) or mode.strip().lower() not in VALID_MAP_MODES:
        raise ArtifactError("map mode must be trinary, scale, or raw")
    raw_candidate_prefix = Path(candidate_prefix)
    _reject_path_traversal(raw_candidate_prefix, "candidate prefix")
    _reject_symlink_components(raw_candidate_prefix, stop=session_dir)
    candidate_prefix = _resolved(raw_candidate_prefix)
    if candidate_prefix.parent != session_dir:
        raise ArtifactError("candidate prefix must be directly inside session_dir")
    image_path = _contained_artifact(
        image_value, session_dir, canonical_dirs, "map image")
    expected_images = {
        _resolved(Path(str(candidate_prefix) + suffix))
        for suffix in MAP_IMAGE_SUFFIXES
    }
    if image_path not in expected_images:
        raise ArtifactError(
            "map image must match candidate prefix with .pgm, .png, or .bmp")
    return {
        "image": image_value,
        "image_relative_path": str(image_path.relative_to(session_dir)),
        "resolution": resolution,
        "origin": normalized_origin,
        "negate": _normalize_negate(document["negate"]),
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
        "mode": mode.strip().lower(),
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise ArtifactError(f"cannot hash artifact: {path}") from exc
    return digest.hexdigest()


def _artifact_record(path: Path, session_dir: Path) -> dict[str, Any]:
    return {
        "path": str(path),
        "relative_path": str(path.relative_to(session_dir)),
        "size": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def _bundle_sha256(artifacts: Mapping[str, Mapping[str, Any]]) -> str:
    material = [
        f"{key}:{artifacts[key]['relative_path']}:{artifacts[key]['size']}"
        f":{artifacts[key]['sha256']}"
        for key in sorted(artifacts)
    ]
    return hashlib.sha256("\n".join(material).encode("utf-8")).hexdigest()


def build_manifest(
        session_dir: Path,
        prefix: Path,
        datum: Sequence[Any],
        *,
        metadata: Optional[Mapping[str, Any]] = None,
        state: str = "SAVED",
        canonical_dirs: Iterable[Path] = ()) -> dict[str, Any]:
    """Build a manifest only from a fully validated candidate on disk."""
    canonical_dirs = tuple(canonical_dirs)
    if state not in {"SAVED", "VALIDATED"}:
        raise ArtifactError(f"unsupported artifact state: {state}")
    raw_session_dir = Path(session_dir).expanduser()
    _reject_path_traversal(raw_session_dir, "session directory")
    raw_session_dir = _absolute_path(raw_session_dir)
    _reject_symlink_components(raw_session_dir)
    session_dir = _resolved(raw_session_dir)
    raw_prefix = Path(prefix)
    _reject_path_traversal(raw_prefix, "candidate prefix")
    _reject_symlink_components(raw_prefix, stop=session_dir)
    prefix = _resolved(raw_prefix)
    if prefix.parent != session_dir:
        raise ArtifactError("candidate prefix must be directly inside session_dir")
    _reject_canonical(prefix, canonical_dirs)
    _reject_reserved_candidate_prefix(prefix, session_dir)
    datum_values = validate_datum(datum)
    yaml_path = _contained_artifact(
        Path(str(prefix) + ".yaml"), session_dir, canonical_dirs, "map YAML")
    graph_path = _contained_artifact(
        Path(str(prefix) + ".posegraph"), session_dir, canonical_dirs, "pose graph")
    graph_data_path = _contained_artifact(
        Path(str(prefix) + ".data"), session_dir, canonical_dirs,
        "pose graph data")
    document = _read_yaml(yaml_path, "map YAML")
    map_metadata = validate_map_document(
        document, yaml_path, session_dir, canonical_dirs,
        candidate_prefix=prefix)
    image_path = _contained_artifact(
        session_dir / map_metadata["image_relative_path"],
        session_dir, canonical_dirs, "map image")
    transformed_origin = list(map_metadata["origin"])
    artifacts = {
        "map_yaml": _artifact_record(yaml_path, session_dir),
        "map_image": _artifact_record(image_path, session_dir),
        "pose_graph": _artifact_record(graph_path, session_dir),
        "pose_graph_data": _artifact_record(graph_data_path, session_dir),
    }
    manifest: dict[str, Any] = {
        "schema": SCHEMA_VERSION,
        "schema_version": SCHEMA_VERSION,
        "kind": MANIFEST_KIND,
        "state": state,
        "validation": "passed",
        "created_unix": time.time(),
        "session_path": str(session_dir),
        "candidate_name": prefix.name,
        "candidate_prefix": str(prefix),
        "candidate_path": str(yaml_path),
        "candidate_sha256": artifacts["map_yaml"]["sha256"],
        "artifact_bundle_sha256": _bundle_sha256(artifacts),
        "surveyed_datum": datum_values,
        "transformed_origin": transformed_origin,
        "map": map_metadata,
        "artifacts": artifacts,
        "metadata": dict(metadata or {}),
    }
    return manifest


def _manifest_artifact_path(
        manifest_entry: Mapping[str, Any],
        session_dir: Path,
        canonical_dirs: Iterable[Path],
        label: str) -> Path:
    if not isinstance(manifest_entry, Mapping):
        raise ArtifactError(f"manifest {label} entry is invalid")
    relative = manifest_entry.get("relative_path")
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ArtifactError(f"manifest {label} relative_path is invalid")
    _reject_path_traversal(Path(relative), f"manifest {label} relative_path")
    path = _contained_artifact(
        relative, session_dir, canonical_dirs, f"manifest {label}")
    recorded = manifest_entry.get("path")
    if recorded != str(path):
        raise ArtifactError(f"manifest {label} path does not match session identity")
    return path


def verify_manifest(
        session_dir: str | os.PathLike[str] | Path,
        candidate_name: Optional[str] = None,
        *,
        canonical_dirs: Iterable[Path] = (),
        expected_state: Optional[str] = None) -> dict[str, Any]:
    """Read and verify the saved manifest; never manufacture a replacement."""
    canonical_dirs = tuple(canonical_dirs)
    session = safe_session_dir(session_dir, canonical_dirs, create=False)
    marker = session / MANIFEST_NAME
    marker = _contained_artifact(
        marker, session, canonical_dirs, "mapping manifest")
    manifest = _read_yaml(marker, "mapping manifest")
    if (manifest.get("schema") != SCHEMA_VERSION or
            manifest.get("schema_version") != SCHEMA_VERSION):
        raise ArtifactError("mapping manifest schema is unsupported")
    if manifest.get("kind") != MANIFEST_KIND:
        raise ArtifactError("mapping manifest kind is invalid")
    state = manifest.get("state")
    if state not in {"SAVED", "VALIDATED"}:
        raise ArtifactError("mapping manifest state is invalid")
    if expected_state is not None and state != expected_state:
        raise ArtifactError(f"mapping manifest must be in state {expected_state}")
    if manifest.get("validation") != "passed":
        raise ArtifactError("mapping manifest is not a passed validation")
    if manifest.get("session_path") != str(session):
        raise ArtifactError("mapping manifest session identity does not match")
    name = manifest.get("candidate_name")
    if not isinstance(name, str) or Path(name).name != name or name in {".", ".."}:
        raise ArtifactError("mapping manifest candidate identity is invalid")
    if candidate_name is not None and name != candidate_name:
        raise ArtifactError("mapping manifest candidate name does not match request")
    prefix = _resolved(session / name)
    _reject_reserved_candidate_prefix(prefix, session)
    if manifest.get("candidate_prefix") != str(prefix):
        raise ArtifactError("mapping manifest candidate prefix does not match")
    expected_candidate_path = _resolved(Path(str(prefix) + ".yaml"))
    if manifest.get("candidate_path") != str(expected_candidate_path):
        raise ArtifactError("mapping manifest candidate path does not match")

    raw_artifacts = manifest.get("artifacts")
    if not isinstance(raw_artifacts, dict) or set(raw_artifacts) != {
            "map_yaml", "map_image", "pose_graph", "pose_graph_data"}:
        raise ArtifactError("mapping manifest artifact set is invalid")
    paths = {
        key: _manifest_artifact_path(raw_artifacts[key], session, canonical_dirs, key)
        for key in raw_artifacts
    }
    if paths["map_yaml"] != expected_candidate_path:
        raise ArtifactError("mapping manifest map YAML is not the requested candidate")
    expected_graph = _resolved(Path(str(prefix) + ".posegraph"))
    if paths["pose_graph"] != expected_graph:
        raise ArtifactError("mapping manifest pose graph is not the requested candidate")
    expected_graph_data = _resolved(Path(str(prefix) + ".data"))
    if paths["pose_graph_data"] != expected_graph_data:
        raise ArtifactError(
            "mapping manifest pose graph data is not the requested candidate")

    document = _read_yaml(paths["map_yaml"], "map YAML")
    map_metadata = validate_map_document(
        document, paths["map_yaml"], session, canonical_dirs,
        candidate_prefix=prefix)
    expected_image = _resolved(session / map_metadata["image_relative_path"])
    if paths["map_image"] != expected_image:
        raise ArtifactError("mapping manifest image does not match map YAML")
    if manifest.get("map") != map_metadata:
        raise ArtifactError("mapping manifest map metadata is stale or forged")
    if manifest.get("transformed_origin") != map_metadata["origin"]:
        raise ArtifactError("mapping manifest transformed origin is stale")
    validate_datum(manifest.get("surveyed_datum", ()))

    for key, path in paths.items():
        expected = raw_artifacts[key]
        actual_size = path.stat().st_size
        actual_hash = sha256_file(path)
        if expected.get("sha256") != actual_hash:
            raise ArtifactError(f"mapping manifest {key} hash does not match")
        if expected.get("size") != actual_size:
            raise ArtifactError(f"mapping manifest {key} size does not match")
    if manifest.get("candidate_sha256") != raw_artifacts["map_yaml"]["sha256"]:
        raise ArtifactError("mapping manifest candidate hash does not match")
    if manifest.get("artifact_bundle_sha256") != _bundle_sha256(raw_artifacts):
        raise ArtifactError("mapping manifest artifact bundle hash does not match")
    return dict(manifest)


def atomic_write_yaml(path: Path, value: Mapping[str, Any]) -> None:
    """Write YAML through a same-directory fsync + replace transaction."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ArtifactError(f"refusing to replace symlink: {path}")
    payload = yaml.safe_dump(dict(value), sort_keys=False)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=path.parent,
                prefix=f".{path.name}.", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    """Write a human-readable JSON receipt with the same atomic discipline."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ArtifactError(f"refusing to replace symlink: {path}")
    payload = json.dumps(value, indent=2, sort_keys=True) + "\n"
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=path.parent,
                prefix=f".{path.name}.", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def discard_verified(
        session_dir: str | os.PathLike[str] | Path,
        candidate_name: Optional[str] = None,
        *,
        canonical_dirs: Iterable[Path] = ()) -> dict[str, Any]:
    """Delete only verified candidate files and the manifest, never the session."""
    canonical_dirs = tuple(canonical_dirs)
    manifest = verify_manifest(
        session_dir, candidate_name, canonical_dirs=canonical_dirs)
    session = _resolved(Path(session_dir))
    paths = [
        Path(manifest["artifacts"][key]["path"])
        for key in ("map_image", "pose_graph", "pose_graph_data", "map_yaml")
    ]
    marker = session / MANIFEST_NAME
    for path in paths:
        if path.parent != session and session not in path.parents:
            raise ArtifactError("refusing to discard an artifact outside the session")
        if not path.is_file() or path.is_symlink():
            raise ArtifactError("verified discard artifact is no longer a regular file")
    for path in paths + [marker]:
        path.unlink()
    return manifest


def manifest_json(manifest: Mapping[str, Any]) -> str:
    return json.dumps(manifest, indent=2, sort_keys=True)
