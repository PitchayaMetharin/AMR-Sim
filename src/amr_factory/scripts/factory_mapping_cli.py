#!/usr/bin/env python3
"""Save, validate, and explicitly discard factory SLAM session artifacts.

The generated ``mapping_manifest.yaml`` is the only validation authority, and
all paths are rejected if they target the canonical factory maps.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import yaml

import factory_mapping_artifacts as artifacts


MAP_SAVER_TIMEOUT_SEC = 15.0


def _canonical_maps_dir() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("amr_factory"), "maps").resolve()


def _canonical_maps_dirs() -> set[Path]:
    """Return installed and source-tree canonical map directories."""
    paths = {_canonical_maps_dir()}
    source_maps = Path(__file__).resolve().parents[1] / "maps"
    if source_maps.is_dir():
        paths.add(source_maps.resolve())
    return paths


def _safe_session_dir(value):
    return artifacts.safe_session_dir(value, _canonical_maps_dirs(), create=False)


def transform_origin(origin, datum):
    return artifacts.transform_origin(origin, datum)


def _candidate_prefix(session_dir, name, allow_existing=False):
    return artifacts.candidate_prefix(
        session_dir,
        name,
        _canonical_maps_dirs(),
        allow_existing=allow_existing,
        create_session=not allow_existing,
    )


def _validate_artifacts(prefix, datum):
    prefix = Path(prefix)
    session_dir = prefix.parent
    manifest = artifacts.build_manifest(
        session_dir,
        prefix,
        datum,
        canonical_dirs=_canonical_maps_dirs(),
    )
    return Path(str(prefix) + ".yaml"), manifest


def _save_occupancy_map(prefix):
    command = [
        "ros2", "run", "nav2_map_server", "map_saver_cli",
        "-f", str(prefix),
        "-t", "/map",
        "--ros-args", "-p", "map_subscribe_transient_local:=true",
    ]
    try:
        result = subprocess.run(
            command, check=False, timeout=MAP_SAVER_TIMEOUT_SEC)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("Nav2 map_saver_cli timed out") from exc
    if result.returncode != 0:
        raise RuntimeError(
            "Nav2 map_saver_cli failed with exit code "
            f"{result.returncode}")


def _service_call(session_dir, prefix, use_sim_time=True):
    import rclpy
    from slam_toolbox.srv import SerializePoseGraph

    rclpy.init()
    node = rclpy.create_node("factory_mapping_cli", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, use_sim_time)
    ])
    graph_client = node.create_client(
        SerializePoseGraph, "/amr/slam_toolbox/serialize_map")
    try:
        if not graph_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("SLAM serialize_map service is unavailable")
        _save_occupancy_map(prefix)
        graph_request = SerializePoseGraph.Request(filename=str(prefix))
        graph_future = graph_client.call_async(graph_request)
        rclpy.spin_until_future_complete(node, graph_future, timeout_sec=15.0)
        if (not graph_future.done() or graph_future.result() is None or
                graph_future.result().result != SerializePoseGraph.Response.RESULT_SUCCESS):
            raise RuntimeError("SLAM pose-graph serialization failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _load_map_yaml(path: Path) -> dict:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise artifacts.ArtifactError(f"map YAML is not valid: {exc}") from exc
    if not isinstance(value, dict):
        raise artifacts.ArtifactError("map YAML must contain a mapping")
    return value


def save(args):
    datum = artifacts.validate_datum((args.datum_x, args.datum_y, args.datum_yaw))
    session_dir, prefix = _candidate_prefix(args.session_dir, args.name)
    _service_call(session_dir, prefix, args.use_sim_time)
    yaml_path = Path(str(prefix) + ".yaml")
    document = _load_map_yaml(yaml_path)
    if "origin" not in document:
        raise artifacts.ArtifactError("map YAML origin is missing")
    document["origin"] = artifacts.transform_origin(document["origin"], datum)
    # The manifest is committed only after the complete candidate and pose graph
    # have passed validation. A failed service or artifact check leaves no
    # valid manifest that a later discard can trust.
    artifacts.atomic_write_yaml(yaml_path, document)
    manifest = artifacts.build_manifest(
        session_dir,
        prefix,
        datum,
        metadata={"use_sim_time": bool(args.use_sim_time)},
        state="SAVED",
        canonical_dirs=_canonical_maps_dirs(),
    )
    artifacts.atomic_write_yaml(session_dir / artifacts.MANIFEST_NAME, manifest)
    print(artifacts.manifest_json(manifest))


def validate(args):
    session_dir = artifacts.safe_session_dir(
        args.session_dir, _canonical_maps_dirs(), create=False)
    manifest = artifacts.verify_manifest(
        session_dir,
        args.name,
        canonical_dirs=_canonical_maps_dirs(),
    )
    if manifest["state"] == "SAVED":
        manifest["state"] = "VALIDATED"
        manifest["validated_unix"] = time.time()
        artifacts.atomic_write_yaml(session_dir / artifacts.MANIFEST_NAME, manifest)
    print(artifacts.manifest_json(manifest))


def discard(args):
    if not args.confirm:
        raise artifacts.ArtifactError(
            "pass --confirm to discard this exact mapping session")
    manifest = artifacts.discard_verified(
        args.session_dir,
        args.name,
        canonical_dirs=_canonical_maps_dirs(),
    )
    print(
        f"discarded verified candidate {manifest['candidate_name']} from "
        f"mapping session: {manifest['session_path']}"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    sub = subparsers.add_parser("save")
    sub.add_argument("--session-dir", required=True)
    sub.add_argument("--name", default="factory_candidate")
    sub.add_argument("--datum-x", type=float, default=-4.5)
    sub.add_argument("--datum-y", type=float, default=0.0)
    sub.add_argument("--datum-yaw", type=float, default=0.0)
    sub.add_argument("--use-sim-time", action=argparse.BooleanOptionalAction, default=True)
    sub.set_defaults(handler=save)

    sub = subparsers.add_parser("validate")
    sub.add_argument("--session-dir", required=True)
    sub.add_argument("--name", default="factory_candidate")
    sub.set_defaults(handler=validate)

    sub = subparsers.add_parser("discard")
    sub.add_argument("--session-dir", required=True)
    sub.add_argument("--name", default=None)
    sub.add_argument("--confirm", action="store_true")
    sub.set_defaults(handler=discard)

    args = parser.parse_args(argv)
    try:
        args.handler(args)
    except (OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"factory mapping: FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
