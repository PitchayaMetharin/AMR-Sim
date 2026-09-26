#!/usr/bin/env python3
"""Extract a deterministic planner-replay snapshot from a finalized ROS bag.

The extractor deliberately accepts only finalized bags.  It selects one
recorded global plan and the latest map/costmap evidence available before that
plan, then persists the exact rectangular footprint, start, goal, and paths
needed by the offline planner harness.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


NAVIGATION_FOOTPRINT = (
    (0.61, 0.41),
    (0.61, -0.41),
    (-0.61, -0.41),
    (-0.61, 0.41),
)


def _yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _stamp(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec)


def _path_pose(pose: Any) -> dict[str, float]:
    return {
        "x": float(pose.pose.position.x),
        "y": float(pose.pose.position.y),
        "yaw": float(_yaw(pose.pose.orientation)),
    }


def _path_record(message: Any, recorded_ns: int) -> dict[str, Any]:
    return {
        "recorded_ns": recorded_ns,
        "frame_id": message.header.frame_id,
        "poses": [_path_pose(pose) for pose in message.poses],
    }


def _open_reader(bag_dir: Path) -> tuple[rosbag2_py.SequentialReader, dict[str, str]]:
    metadata = bag_dir / "metadata.yaml"
    if not metadata.is_file():
        raise ValueError("bag is not finalized: metadata.yaml is missing")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    return reader, topic_types


def _geometry_record(message: Any, recorded_ns: int, is_costmap: bool) -> dict[str, Any]:
    if is_costmap:
        metadata = message.metadata
        width = int(metadata.size_x)
        height = int(metadata.size_y)
        resolution = float(metadata.resolution)
        origin = metadata.origin
        update_time = {
            "sec": int(metadata.update_time.sec),
            "nanosec": int(metadata.update_time.nanosec),
        }
    else:
        metadata = message.info
        width = int(metadata.width)
        height = int(metadata.height)
        resolution = float(metadata.resolution)
        origin = metadata.origin
        update_time = None
    data = [int(value) for value in message.data]
    if len(data) != width * height:
        raise ValueError(
            "{} data length {} does not match {}x{}".format(
                "costmap" if is_costmap else "map", len(data), width, height))
    return {
        "recorded_ns": recorded_ns,
        "frame_id": message.header.frame_id,
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": {
            "x": float(origin.position.x),
            "y": float(origin.position.y),
            "yaw": float(_yaw(origin.orientation)),
        },
        "update_time": update_time,
        "data": data,
    }


def extract(bag_dir: Path, plan_index: int) -> dict[str, Any]:
    reader, topic_types = _open_reader(bag_dir)
    required = {
        "/map",
        "/amr/global_costmap/costmap_raw",
        "/amr/plan",
        "/amr/plan_smoothed",
        "/amr/global_costmap/published_footprint",
    }
    missing = sorted(topic for topic in required if topic not in topic_types)
    if missing:
        raise ValueError("bag is missing required topics: " + ", ".join(missing))

    message_types = {
        topic: get_message(topic_types[topic])
        for topic in required
    }
    maps: list[tuple[int, dict[str, Any]]] = []
    costmaps: list[tuple[int, dict[str, Any]]] = []
    footprints: list[tuple[int, list[list[float]]]] = []
    plans: list[tuple[int, dict[str, Any]]] = []
    smoothed: list[tuple[int, dict[str, Any]]] = []

    while reader.has_next():
        topic, serialized, recorded_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == "/map":
            maps.append((recorded_ns, _geometry_record(message, recorded_ns, False)))
        elif topic == "/amr/global_costmap/costmap_raw":
            costmaps.append(
                (recorded_ns, _geometry_record(message, recorded_ns, True)))
        elif topic == "/amr/plan":
            plans.append((recorded_ns, _path_record(message, recorded_ns)))
        elif topic == "/amr/plan_smoothed":
            smoothed.append((recorded_ns, _path_record(message, recorded_ns)))
        else:
            if message.polygon.points:
                footprints.append(
                    (
                        recorded_ns,
                        [
                            [float(point.x), float(point.y)]
                            for point in message.polygon.points
                        ],
                    ))

    if not plans:
        raise ValueError("bag contains no /amr/plan messages")
    if plan_index < 0:
        plan_index += len(plans)
    if not 0 <= plan_index < len(plans):
        raise ValueError(
            "plan index {} is outside 0..{}".format(plan_index, len(plans) - 1))
    plan_ns, plan = plans[plan_index]
    prior_maps = [item for item in maps if item[0] <= plan_ns]
    prior_costmaps = [item for item in costmaps if item[0] <= plan_ns]
    prior_footprints = [item for item in footprints if item[0] <= plan_ns]
    if not prior_maps or not prior_costmaps or not prior_footprints:
        raise ValueError(
            "selected plan has no preceding map, costmap, or footprint evidence")
    map_ns, map_record = prior_maps[-1]
    costmap_ns, costmap_record = prior_costmaps[-1]
    footprint_ns, observed_footprint = prior_footprints[-1]
    if map_record["frame_id"] != costmap_record["frame_id"]:
        raise ValueError("map and costmap frames differ")
    if (
        map_record["width"],
        map_record["height"],
        map_record["resolution"],
        map_record["origin"],
    ) != (
        costmap_record["width"],
        costmap_record["height"],
        costmap_record["resolution"],
        costmap_record["origin"],
    ):
        raise ValueError("map and costmap geometry differ")
    if len(plan["poses"]) < 2:
        raise ValueError("selected plan must contain a start and goal pose")
    matching_smoothed = min(
        smoothed,
        key=lambda item: abs(item[0] - plan_ns),
        default=None,
    )
    if matching_smoothed is None:
        raise ValueError("bag contains no /amr/plan_smoothed message")

    return {
        "schema": 1,
        "source": {
            "bag_dir": str(bag_dir),
            "plan_topic": "/amr/plan",
            "plan_index": plan_index,
            "plan_count": len(plans),
        },
        "map": map_record,
        "costmap": costmap_record,
        "footprint": [list(point) for point in NAVIGATION_FOOTPRINT],
        "observed_footprint": {
            "recorded_ns": footprint_ns,
            "points": observed_footprint,
        },
        "start": plan["poses"][0],
        "goal": plan["poses"][-1],
        "plan": plan,
        "smoothed_plan": matching_smoothed[1],
        "selection": {
            "plan_recorded_ns": plan_ns,
            "map_recorded_ns": map_ns,
            "costmap_recorded_ns": costmap_ns,
            "smoothed_plan_recorded_ns": matching_smoothed[0],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plan-index", type=int, default=-1)
    args = parser.parse_args()
    snapshot = extract(args.bag_dir, args.plan_index)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(snapshot, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        "extracted plan index {} with {} poses to {}".format(
            snapshot["source"]["plan_index"],
            len(snapshot["plan"]["poses"]),
            args.output,
        ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
