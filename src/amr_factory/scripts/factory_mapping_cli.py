#!/usr/bin/env python3
"""Save, validate, and explicitly discard factory SLAM session artifacts."""

import argparse
import json
import math
from pathlib import Path
import shutil
import sys
import time

import yaml


def _canonical_maps_dir():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("amr_factory"), "maps").resolve()


def _canonical_maps_dirs():
    """Return installed and source-tree canonical map directories."""
    paths = {_canonical_maps_dir()}
    source_maps = Path(__file__).resolve().parents[1] / "maps"
    if source_maps.is_dir():
        paths.add(source_maps.resolve())
    return paths


def _safe_session_dir(value):
    if not value or not value.strip():
        raise ValueError("session directory must be non-empty")
    raw_path = Path(value).expanduser()
    if raw_path.is_symlink():
        raise ValueError("session directory must not be a symlink")
    path = raw_path.resolve()
    if path == Path("/"):
        raise ValueError("session directory must not be the filesystem root")
    if any(path == canonical or canonical in path.parents
           for canonical in _canonical_maps_dirs()):
        raise ValueError("session directory must not be inside canonical factory maps")
    return path


def transform_origin(origin, datum):
    """Transform a map YAML origin from SLAM coordinates into the datum frame."""
    if len(origin) != 3 or len(datum) != 3:
        raise ValueError("origin and datum must contain x, y, yaw")
    x, y, yaw = (float(value) for value in origin)
    dx, dy, dyaw = (float(value) for value in datum)
    c, s = math.cos(dyaw), math.sin(dyaw)
    return [dx + c * x - s * y, dy + s * x + c * y,
            math.atan2(math.sin(dyaw + yaw), math.cos(dyaw + yaw))]


def _candidate_prefix(session_dir, name, allow_existing=False):
    if not name or Path(name).name != name or name in {".", ".."}:
        raise ValueError("map name must be a simple file prefix")
    session_dir = _safe_session_dir(session_dir)
    session_dir.mkdir(parents=True, exist_ok=True)
    prefix = (session_dir / name).resolve()
    if prefix.parent != session_dir:
        raise ValueError("map output must remain directly inside session_dir")
    if any(prefix.parent == canonical or canonical in prefix.parents
           for canonical in _canonical_maps_dirs()):
        raise ValueError("map output must not target canonical factory maps")
    if not allow_existing:
        for suffix in (".yaml", ".pgm", ".png", ".bmp", ".posegraph"):
            if Path(str(prefix) + suffix).exists():
                raise ValueError("map output already exists; use a new session or name")
    return session_dir, prefix


def _service_call(session_dir, prefix, use_sim_time=True):
    import rclpy
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("factory_mapping_cli", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, use_sim_time)
    ])
    save_client = node.create_client(SaveMap, "/amr/slam_toolbox/save_map")
    graph_client = node.create_client(SerializePoseGraph, "/amr/slam_toolbox/serialize_pose_graph")
    try:
        if not save_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("SLAM save_map service is unavailable")
        if not graph_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("SLAM serialize_pose_graph service is unavailable")
        save_request = SaveMap.Request()
        save_request.name = String(data=str(prefix))
        save_future = save_client.call_async(save_request)
        rclpy.spin_until_future_complete(node, save_future, timeout_sec=15.0)
        if not save_future.done() or save_future.result() is None or save_future.result().result != SaveMap.Response.RESULT_SUCCESS:
            raise RuntimeError("SLAM occupancy map save failed")
        graph_request = SerializePoseGraph.Request(filename=str(prefix) + ".posegraph")
        graph_future = graph_client.call_async(graph_request)
        rclpy.spin_until_future_complete(node, graph_future, timeout_sec=15.0)
        if not graph_future.done() or graph_future.result() is None or graph_future.result().result != SerializePoseGraph.Response.RESULT_SUCCESS:
            raise RuntimeError("SLAM pose-graph serialization failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _validate_artifacts(prefix, datum):
    yaml_path = Path(str(prefix) + ".yaml")
    graph_path = Path(str(prefix) + ".posegraph")
    if not yaml_path.is_file() or not graph_path.is_file():
        raise ValueError("map YAML and pose graph must both exist")
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or float(document.get("resolution", 0.0)) <= 0.0:
        raise ValueError("map YAML has no positive resolution")
    image_path = Path(document.get("image", ""))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    if not image_path.is_file():
        raise ValueError("map image referenced by YAML does not exist")
    if len(document.get("origin", [])) != 3:
        raise ValueError("map YAML origin must contain x, y, yaw")
    if abs(float(document["resolution"]) - 0.05) > 1e-9:
        raise ValueError("map resolution must remain 0.05 m")
    manifest = {
        "schema": 1,
        "created_unix": time.time(),
        "map_yaml": str(yaml_path),
        "map_image": str(image_path),
        "pose_graph": str(graph_path),
        "resolution": float(document["resolution"]),
        "origin": [float(value) for value in document["origin"]],
        "surveyed_datum": [float(value) for value in datum],
        "validation": "passed",
    }
    return yaml_path, manifest


def save(args):
    session_dir, prefix = _candidate_prefix(args.session_dir, args.name)
    _service_call(session_dir, prefix, args.use_sim_time)
    yaml_path = Path(str(prefix) + ".yaml")
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    document["origin"] = transform_origin(document["origin"],
                                            (args.datum_x, args.datum_y, args.datum_yaw))
    yaml_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    _, manifest = _validate_artifacts(prefix, (args.datum_x, args.datum_y, args.datum_yaw))
    manifest["map_yaml"] = str(yaml_path)
    (session_dir / "mapping_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    print(json.dumps(manifest, indent=2))


def validate(args):
    _session_dir, prefix = _candidate_prefix(args.session_dir, args.name, allow_existing=True)
    _, manifest = _validate_artifacts(prefix, (args.datum_x, args.datum_y, args.datum_yaw))
    print(json.dumps(manifest, indent=2))


def discard(args):
    session_dir = _safe_session_dir(args.session_dir)
    marker = session_dir / "mapping_manifest.yaml"
    if not marker.is_file():
        raise ValueError("refusing to discard a directory without mapping_manifest.yaml")
    if not args.confirm:
        raise ValueError("pass --confirm to discard this exact mapping session")
    shutil.rmtree(session_dir)
    print(f"discarded mapping session: {session_dir}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("save", "validate"):
        sub = subparsers.add_parser(command)
        sub.add_argument("--session-dir", required=True)
        sub.add_argument("--name", default="factory_candidate")
        sub.add_argument("--datum-x", type=float, default=-4.5)
        sub.add_argument("--datum-y", type=float, default=0.0)
        sub.add_argument("--datum-yaw", type=float, default=0.0)
        sub.add_argument("--use-sim-time", action=argparse.BooleanOptionalAction, default=True)
        sub.set_defaults(handler=save if command == "save" else validate)
    sub = subparsers.add_parser("discard")
    sub.add_argument("--session-dir", required=True)
    sub.add_argument("--confirm", action="store_true")
    sub.set_defaults(handler=discard)
    args = parser.parse_args(argv)
    try:
        args.handler(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"factory mapping: FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
