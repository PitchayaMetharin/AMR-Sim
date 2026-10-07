#!/usr/bin/env python3
"""Build the native45_clearance_scene_v1 scene for production_precision_replay.

Reads only retained JSON/JSONL/JSONL.gz evidence plus current source/config
files; it never opens a bag.  Geometry here is constructed, not observed: the
captured poses supply scenes and measured biases, while every maneuver start,
bias pairing and repair miss is an explicit, labelled assumption.  Runtime
properties (observer settling, fresh post-heading bias, terminal behaviour)
are recorded as not_exercised.
"""

import argparse
import ast
import bisect
import gzip
import hashlib
import json
import math
import os
import re
import sys
import tempfile
from pathlib import Path

import yaml

SCHEMA = "native45_clearance_scene_v1"
POLYGON_TOLERANCE_M = 1e-6  # numerical consistency of serialized polygons only
PLANAR_TOLERANCE = 1e-6
SMOOTHER_BUDGET_S = 1
REPAIR_MISS_M = 0.010001
BIAS_SENSITIVITY_NOTE = (
    "geometric sensitivity: localized start from a clear-start capture paired with a "
    "pre-dock bias; not a future post-heading freshness observation")

# Packet-pinned identities; a mismatch means the retained inputs changed.
PINNED_SHA256 = {
    "clear_scene_snapshot.json":
        "accdcc93238d2d2803b5a853e72d2b00fc9af2140380b394d11406f203d95158",
    "dock_scene_snapshot.json":
        "f9db23eda45e159f22447e386f29da1c9364f9309222d812a7083101a6d14135",
    "summary.json":
        "c1b2dcf008aba331d849ee7f2bff550a76c05113a1411506df8a7502983e6802",
    "tf_edge_rows.jsonl":
        "a131920bc1159b1595bb2901271508a37ce7ca2bec7cf258af165012273ae35f",
    "pose_rows.jsonl":
        "e581504e97e696e30cd1f4b8b717ffcbe5e801b4df968d9d1eb7837b24e1c294",
    "planner.yaml":
        "8ec55573ce6fd9bcdb0bb03268244493435951746a956457008cff9b9b695577",
    "typed_stream.jsonl.gz":
        "f81dc4619b4d8a0d5e66b93fe4d35faaaecdd18f4a3dfff23cb12b71b3c80c66",
}
MISSING_AMCL = "/amr/amcl_pose boundary after geometry window"
GT_TOPIC = "/amr/simulation/ground_truth/pose"
EDGES = (("map", "odom"), ("odom", "base_footprint"))

# Source constants this adapter mirrors; (file key, regex, packet value).
SOURCE_CONSTANTS = {
    "kClearPositionTolerance": ("gate6", 0.01),
    "kRegisteredApproachAdmission": ("gate6", 0.155),
    "kClearAreaDisplacement": ("gate6", 0.15),
    "kHeadingTolerance": ("gate6", 0.15),
    "kFinalHeadingGoalMargin": ("gate6", 0.03),
    "kDesiredProduct102SlotBaseX": ("helper", 0.748),
    "kDesiredProduct102SlotBaseY": ("helper", 0.1),
}


class AdapterError(Exception):
    pass


def require(condition, message):
    if not condition:
        raise AdapterError(message)


def wrap(angle):
    return math.remainder(angle, 2.0 * math.pi)


def finite(*values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in values)


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError) as error:
        raise AdapterError(f"cannot read JSON {path}: {error}")


def read_jsonl(path, gz=False):
    opener = gzip.open if gz else open
    try:
        with opener(path, "rt", encoding="utf-8") as handle:
            for number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                try:
                    yield json.loads(line)
                except ValueError as error:
                    raise AdapterError(f"{path}:{number} malformed JSONL: {error}")
    except OSError as error:
        raise AdapterError(f"cannot read {path}: {error}")


# ---------------------------------------------------------------------------
# Planar poses (x, y, yaw)

def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def pose_dict(p):
    return {"x": p[0], "y": p[1], "yaw": p[2]}


def stamp_ns(stamp, label):
    require(isinstance(stamp, dict), f"{label}: stamp is not an object")
    sec, nsec = stamp.get("sec"), stamp.get("nanosec")
    require(type(sec) is int and type(nsec) is int, f"{label}: stamp fields are not integers")
    require(sec >= 0 and 0 <= nsec < 10**9, f"{label}: invalid stamp {sec}.{nsec}")
    value = sec * 10**9 + nsec
    require(value > 0, f"{label}: zero stamp")
    return value


def planar_pose(translation, rotation, label):
    require(isinstance(translation, dict) and isinstance(rotation, dict),
            f"{label}: malformed transform")
    try:
        x, y, z = (translation[k] for k in "xyz")
        qx, qy, qz, qw = (rotation[k] for k in "xyzw")
    except KeyError as error:
        raise AdapterError(f"{label}: missing component {error}")
    require(finite(x, y, z, qx, qy, qz, qw), f"{label}: nonfinite pose")
    require(abs(z) <= PLANAR_TOLERANCE, f"{label}: nonplanar translation")
    require(abs(qx) <= PLANAR_TOLERANCE and abs(qy) <= PLANAR_TOLERANCE,
            f"{label}: nonplanar quaternion")
    norm = qx * qx + qy * qy + qz * qz + qw * qw
    require(abs(norm - 1.0) <= PLANAR_TOLERANCE, f"{label}: quaternion is not unit")
    return (x, y, math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz))


class Series:
    """Observed transform/pose samples; interpolation never extrapolates."""

    def __init__(self, label):
        self.label = label
        self.rows = []

    def add(self, stamp, pose, ident):
        self.rows.append((stamp, pose, ident))

    def finalize(self):
        self.rows.sort(key=lambda row: row[0])
        self.stamps = [row[0] for row in self.rows]
        for first, second in zip(self.stamps, self.stamps[1:]):
            require(first != second, f"{self.label}: duplicate stamp {first}")

    def at(self, stamp):
        index = bisect.bisect_left(self.stamps, stamp)
        if index < len(self.stamps) and self.stamps[index] == stamp:
            row = self.rows[index]
            return row[1], {"exact": True, "predecessor": row[2], "successor": row[2],
                            "stamp_ns": stamp}
        require(0 < index < len(self.stamps),
                f"{self.label}: stamp {stamp} is not bracketed by observed samples")
        pred, succ = self.rows[index - 1], self.rows[index]
        fraction = (stamp - pred[0]) / (succ[0] - pred[0])
        pose = (pred[1][0] + fraction * (succ[1][0] - pred[1][0]),
                pred[1][1] + fraction * (succ[1][1] - pred[1][1]),
                wrap(pred[1][2] + fraction * wrap(succ[1][2] - pred[1][2])))
        return pose, {"exact": False, "predecessor": pred[2], "successor": succ[2],
                      "stamp_ns": stamp, "fraction": fraction,
                      "predecessor_gap_ns": stamp - pred[0],
                      "successor_gap_ns": succ[0] - stamp}


class Observations:
    def __init__(self):
        self.edges = {edge: Series("/tf %s->%s" % edge) for edge in EDGES}
        self.ground_truth = Series(GT_TOPIC)

    def finalize(self):
        for series in self.edges.values():
            series.finalize()
        self.ground_truth.finalize()


def add_tf_transform(obs, transform, ident, label):
    require(isinstance(transform, dict), f"{label}: transform is not an object")
    header = transform.get("header", {})
    edge = (header.get("frame_id"), transform.get("child_frame_id"))
    if edge not in obs.edges:
        return False
    body = transform.get("transform", {})
    pose = planar_pose(body.get("translation"), body.get("rotation"), label)
    obs.edges[edge].add(stamp_ns(header.get("stamp"), label), pose, ident)
    return True


def add_ground_truth(obs, fields, ident, label):
    header = fields.get("header", {})
    require(header.get("frame_id") == "factory_world", f"{label}: wrong ground-truth frame")
    body = fields.get("pose", {})
    pose = planar_pose(body.get("position"), body.get("orientation"), label)
    obs.ground_truth.add(stamp_ns(header.get("stamp"), label), pose, ident)


def load_clear_observations(tf_rows, pose_rows):
    obs = Observations()
    for row in read_jsonl(tf_rows):
        label = f"tf_edge_rows ordinal {row.get('source_ordinal')}"
        require(row.get("topic") == "/tf", f"{label}: wrong topic")
        ident = {"source_ordinal": row.get("source_ordinal"),
                 "cdr_payload_sha256": row.get("cdr_payload_sha256"),
                 "stamp_ns": row.get("acquisition_stamp_ns")}
        require(add_tf_transform(obs, row.get("fields"), ident, label),
                f"{label}: unexpected frame pair")
        require(ident["stamp_ns"] == obs.edges[
            (row["fields"]["header"]["frame_id"], row["fields"]["child_frame_id"])
        ].rows[-1][0], f"{label}: acquisition stamp disagrees with header")
    for row in read_jsonl(pose_rows):
        if row.get("topic") != GT_TOPIC:
            continue
        label = f"pose_rows ordinal {row.get('source_ordinal')}"
        ident = {"source_ordinal": row.get("source_ordinal"),
                 "cdr_payload_sha256": row.get("cdr_payload_sha256"),
                 "stamp_ns": row.get("acquisition_stamp_ns")}
        add_ground_truth(obs, row.get("fields", {}), ident, label)
        require(ident["stamp_ns"] == obs.ground_truth.rows[-1][0],
                f"{label}: acquisition stamp disagrees with header")
    obs.finalize()
    return obs


def load_stream_observations(path, lo_ns, hi_ns, margin_ns=500_000_000):
    obs = Observations()
    lo, hi = lo_ns - margin_ns, hi_ns + margin_ns
    for row in read_jsonl(path, gz=True):
        topic = row.get("topic")
        if topic == "/tf":
            for index, transform in enumerate(row.get("fields", {}).get("transforms", [])):
                header = transform.get("header", {})
                if (header.get("frame_id"), transform.get("child_frame_id")) not in obs.edges:
                    continue
                stamp = stamp_ns(header.get("stamp"), "typed_stream /tf")
                if lo <= stamp <= hi:
                    add_tf_transform(obs, transform, {
                        "traversal_ordinal": row.get("traversal_ordinal"),
                        "transform_index": index,
                        "cdr_payload_sha256": row.get("cdr_payload_sha256"),
                        "stamp_ns": stamp}, f"typed_stream /tf ordinal {row.get('traversal_ordinal')}")
        elif topic == GT_TOPIC:
            fields = row.get("fields", {})
            stamp = stamp_ns(fields.get("header", {}).get("stamp"), "typed_stream ground truth")
            if lo <= stamp <= hi:
                add_ground_truth(obs, fields, {
                    "traversal_ordinal": row.get("traversal_ordinal"),
                    "cdr_payload_sha256": row.get("cdr_payload_sha256"),
                    "stamp_ns": stamp}, f"typed_stream ground truth ordinal {row.get('traversal_ordinal')}")
    obs.finalize()
    return obs


# ---------------------------------------------------------------------------
# Config, registries, constants

def parse_source_constants(texts):
    values = {}
    for name, (key, expected) in SOURCE_CONSTANTS.items():
        match = re.search(r"\b%s\s*=\s*([0-9.]+)\s*;" % re.escape(name), texts[key])
        require(match, f"source constant {name} not found")
        values[name] = float(match.group(1))
        require(values[name] == expected,
                f"source constant {name}={values[name]} differs from packet value {expected}")
    return values


def read_smoother_budget(text):
    matches = re.findall(r"goal\.max_smoothing_duration\.sec\s*=\s*(\d+)\s*;", text)
    require(len(matches) == 1, "production smoother budget assignment not unique")
    require(int(matches[0]) == SMOOTHER_BUDGET_S, "production smoother budget is not 1 second")
    return int(matches[0])


def pad_footprint(points, padding):
    padded = []
    for x, y in points:
        padded.append([x + math.copysign(padding, x) if x != 0.0 else x,
                       y + math.copysign(padding, y) if y != 0.0 else y])
    return padded


def load_config(planner_path):
    raw = yaml.safe_load(Path(planner_path).read_text())
    try:
        precision = raw["/amr/planner_server"]["ros__parameters"]["PrecisionGridBased"]
        costmap = raw["/amr/global_costmap/global_costmap"]["ros__parameters"]
        smoother = raw["/amr/smoother_server"]["ros__parameters"]["simple_smoother"]
    except (KeyError, TypeError) as error:
        raise AdapterError(f"planner config is missing section {error}")
    require(precision.get("plugin") == "amr_navigation/PrecisionNavfnPlanner",
            "PrecisionGridBased plugin changed")
    require(smoother.get("plugin") == "nav2_smoother::SimpleSmoother", "smoother plugin changed")
    footprint_text = costmap.get("footprint")
    require(isinstance(footprint_text, str), "global costmap footprint is not a string")
    footprint = ast.literal_eval(footprint_text)
    require(isinstance(footprint, list) and len(footprint) >= 3 and
            all(len(p) == 2 and finite(*p) for p in footprint), "footprint is malformed")
    padding = costmap.get("footprint_padding")
    require(finite(padding) and padding >= 0.0, "footprint padding is malformed")
    smoother_params = {k: smoother[k] for k in
                       ("tolerance", "max_its", "w_data", "w_smooth", "do_refinement")}
    return {
        "precision_grid_based": {k: precision[k] for k in ("plugin", "use_astar", "allow_unknown",
                                                           "tolerance")},
        "footprint_string": footprint_text,
        "footprint": [list(map(float, p)) for p in footprint],
        "footprint_padding": float(padding),
        "padded_footprint": pad_footprint([list(map(float, p)) for p in footprint], float(padding)),
        "simple_smoother": smoother_params,
        "global_costmap_resolution": costmap.get("resolution"),
    }


def stance_from_registry(dock, slot, constants):
    """Mirror final_placement_stance() for the centered Product 102 slot."""
    require(slot[1] - dock[1] == 0.0, "selected slot is not the centered Product 102 slot")
    base_x = constants["kDesiredProduct102SlotBaseX"]
    base_y = constants["kDesiredProduct102SlotBaseY"]
    yaw = dock[2]
    map_x = math.cos(yaw) * base_x - math.sin(yaw) * base_y
    map_y = math.sin(yaw) * base_x + math.cos(yaw) * base_y
    return (slot[0] - map_x, slot[1] - map_y, yaw)


def clear_point_from(stance, approach):
    axis = (math.cos(stance[2]), math.sin(stance[2]))
    projection = (approach[0] - stance[0]) * axis[0] + (approach[1] - stance[1]) * axis[1]
    return (stance[0] + projection * axis[0], stance[1] + projection * axis[1])


def load_registry(stations_path, products_path, constants):
    stations = yaml.safe_load(Path(stations_path).read_text())
    products = yaml.safe_load(Path(products_path).read_text())
    try:
        dispatch = stations["stations"]["dispatch"]
        approach = (dispatch["approach"]["x"], dispatch["approach"]["y"], dispatch["approach"]["yaw"])
        dock = (dispatch["dock"]["x"], dispatch["dock"]["y"], dispatch["dock"]["yaw"])
        product = products["products"]["product_b"]
        slot_id = product["dispatch_slot"]
        slot = next(s for s in products["dispatch_slots"] if s["id"] == slot_id)
        slot_xyz = (slot["x"], slot["y"], slot["z"])
        product_id = product["tag_id"]
    except (KeyError, TypeError, StopIteration) as error:
        raise AdapterError(f"registry is missing entry {error}")
    require(finite(*approach, *dock, *slot_xyz), "registry geometry is nonfinite")
    require(product_id == 102, "Product B tag id is not 102")
    stance = stance_from_registry(dock, slot_xyz, constants)
    return {"product_id": product_id, "dispatch_approach": list(approach),
            "dispatch_dock": list(dock), "slot_id": slot_id, "selected_slot": list(slot_xyz),
            "physical_stance": list(stance), "clear_point": list(clear_point_from(stance, approach))}


# ---------------------------------------------------------------------------
# Snapshot loading and geometry validation

def snapshot_stamp(item):
    header = item.get("header") or item.get("fields", {}).get("header")
    require(isinstance(header, dict), "geometry item has no header")
    return stamp_ns(header.get("stamp"), "geometry stamp"), header.get("frame_id")


def load_grid(item, frame, ident):
    stamp, header_frame = snapshot_stamp(item)
    require(header_frame == frame and item.get("frame_id") == frame,
            f"{ident}: wrong frame (want {frame})")
    width, height, resolution = item.get("width"), item.get("height"), item.get("resolution")
    require(type(width) is int and type(height) is int and width > 0 and height > 0,
            f"{ident}: malformed grid size")
    require(finite(resolution) and resolution > 0.0, f"{ident}: malformed resolution")
    meta = item.get("metadata", {})
    require(meta.get("size_x") == width and meta.get("size_y") == height and
            meta.get("resolution") == resolution, f"{ident}: metadata disagrees with grid")
    origin = item.get("origin", {})
    require(finite(origin.get("x"), origin.get("y"), origin.get("yaw")) and
            abs(origin["yaw"]) <= 1e-9, f"{ident}: rotated or malformed origin")
    position = meta.get("origin", {}).get("position", {})
    require(position.get("x") == origin["x"] and position.get("y") == origin["y"],
            f"{ident}: origin disagrees with metadata")
    data = item.get("data")
    require(isinstance(data, list) and len(data) == width * height,
            f"{ident}: raw cost length does not match geometry")
    require(all(type(v) is int and 0 <= v <= 255 for v in data),
            f"{ident}: raw cost outside 0..255")
    raw = bytes(data)
    histogram = {}
    for value in raw:
        histogram[value] = histogram.get(value, 0) + 1
    return {"id": ident, "frame_id": frame, "stamp_ns": stamp, "width": width, "height": height,
            "resolution": resolution, "origin_x": origin["x"], "origin_y": origin["y"],
            "data": list(data), "data_sha256": sha256_bytes(raw),
            "cost_histogram": {str(k): v for k, v in sorted(histogram.items())},
            "source_topic": item.get("topic"), "map_to_odom": None}


def load_polygon(item, frame, ident):
    stamp, header_frame = snapshot_stamp(item)
    require(header_frame == frame, f"{ident}: wrong frame (want {frame})")
    points = item.get("fields", {}).get("polygon", {}).get("points")
    require(isinstance(points, list) and len(points) == 4, f"{ident}: footprint is not a quadrilateral")
    vertices = []
    for point in points:
        require(finite(point.get("x"), point.get("y"), point.get("z")) and
                abs(point["z"]) <= PLANAR_TOLERANCE, f"{ident}: malformed footprint vertex")
        vertices.append((point["x"], point["y"]))
    return stamp, vertices


def check_polygon(vertices, base_in_frame, padded):
    """Residual of observed vertices against the padded body polygon."""
    inv = inverse(base_in_frame)
    local = [compose(inv, (x, y, 0.0))[:2] for x, y in vertices]
    unused = list(range(len(padded)))
    worst = 0.0
    for point in local:
        best = min(unused, key=lambda i: math.hypot(point[0] - padded[i][0], point[1] - padded[i][1]))
        worst = max(worst, math.hypot(point[0] - padded[best][0], point[1] - padded[best][1]))
        unused.remove(best)
    return worst


def build_scene_geometry(name, snapshot, obs, config, anchor):
    require(snapshot.get("schema") == 1, f"{name}: unsupported snapshot schema")
    require(snapshot.get("source", {}).get("anchor_name") == anchor, f"{name}: wrong anchor")
    require(snapshot.get("missing_or_invalid") == [] and snapshot.get("replay_inputs_complete") is True,
            f"{name}: snapshot reports missing inputs")
    require(snapshot.get("footprint") == config["footprint"] and
            snapshot.get("footprint_padding_m") == config["footprint_padding"],
            f"{name}: snapshot footprint/padding differs from config")
    grids = {"global": load_grid(snapshot["costmap"], "map", f"{name}_global"),
             "local": load_grid(snapshot["local_costmap"], "odom", f"{name}_local")}
    polygons = {"global": ("map",) + load_polygon(snapshot["observed_footprint"], "map", f"{name}_global_footprint"),
                "local": ("odom",) + load_polygon(snapshot["local_observed_footprint"], "odom", f"{name}_local_footprint")}
    captures = []
    sources = [("global_costmap", "global", grids["global"]["stamp_ns"], "map"),
               ("local_costmap", "local", grids["local"]["stamp_ns"], "odom"),
               ("global_footprint", "global", polygons["global"][1], "map"),
               ("local_footprint", "local", polygons["local"][1], "odom")]
    for label, which, stamp, frame in sources:
        map_to_odom, odom_br = obs.edges[EDGES[0]].at(stamp)
        odom_to_base, base_br = obs.edges[EDGES[1]].at(stamp)
        localized = compose(map_to_odom, odom_to_base)
        physical, gt_br = obs.ground_truth.at(stamp)
        bias = (physical[0] - localized[0], physical[1] - localized[1], wrap(physical[2] - localized[2]))
        require(finite(*bias), f"{name}/{label}: nonfinite bias")
        capture = {"id": f"{name}:{label}", "scene": name, "geometry": label, "frame_id": frame,
                   "stamp_ns": stamp, "localized": pose_dict(localized), "physical": pose_dict(physical),
                   "bias": pose_dict(bias), "map_to_odom": pose_dict(map_to_odom),
                   "odom_to_base": pose_dict(odom_to_base),
                   "brackets": {"map_to_odom": odom_br, "odom_to_base_footprint": base_br,
                                "ground_truth": gt_br},
                   "stationary_proof": "not_proven"}
        pred, succ = gt_br["predecessor"]["stamp_ns"], gt_br["successor"]["stamp_ns"]
        if succ > pred:
            p0, _ = obs.ground_truth.at(pred)
            p1, _ = obs.ground_truth.at(succ)
            capture["physical_speed_mps_estimate"] = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / ((succ - pred) * 1e-9)
        captures.append(capture)
    # Local grid transform comes from its own stamped brackets.
    local_capture = next(c for c in captures if c["geometry"] == "local_costmap")
    grids["local"]["map_to_odom"] = dict(local_capture["map_to_odom"], stamp_ns=grids["local"]["stamp_ns"],
                                         bracket=local_capture["brackets"]["map_to_odom"])
    grids["global"]["map_to_odom"] = None
    footprint_checks = []
    for which, (frame, stamp, vertices) in polygons.items():
        capture = next(c for c in captures if c["geometry"] == f"{which}_footprint")
        base = capture["localized"] if frame == "map" else capture["odom_to_base"]
        residual = check_polygon(vertices, (base["x"], base["y"], base["yaw"]), config["padded_footprint"])
        require(residual <= POLYGON_TOLERANCE_M,
                f"{name}/{which}_footprint: stamped reconstruction residual {residual:.3e} m "
                f"exceeds {POLYGON_TOLERANCE_M:.0e} m")
        footprint_checks.append({"id": f"{name}:{which}_footprint", "frame_id": frame, "stamp_ns": stamp,
                                 "max_vertex_residual_m": residual,
                                 "tolerance_m": POLYGON_TOLERANCE_M,
                                 "padded_half_extents_m": [config["padded_footprint"][0][0],
                                                           config["padded_footprint"][0][1]]})
    return {"name": name, "grids": grids, "captures": captures, "footprint_checks": footprint_checks,
            "snapshot_start": snapshot.get("start"), "snapshot_goal": snapshot.get("goal")}


# ---------------------------------------------------------------------------
# Constructed maneuver cases (mirror gate6_mass_stage.cpp centered dock)

def offset_pose(localized, bias):
    return (localized[0] + bias[0], localized[1] + bias[1], wrap(localized[2] + bias[2]))


def select_tangent(forward, physical_yaw):
    """Nearest forward/reverse tangent; forward on equality (gate6_mass_stage.cpp)."""
    reverse = wrap(forward + math.pi)
    return forward if abs(wrap(forward - physical_yaw)) <= abs(wrap(reverse - physical_yaw)) else reverse


def require_continuous(stages, label):
    """Each stage starts where the previous goal ended (XY and wrapped yaw); no invented motion."""
    for previous, following in zip(stages, stages[1:]):
        goal, start = previous["goal"], following["start"]
        require(math.hypot(goal["x"] - start["x"], goal["y"] - start["y"]) <= 1e-12 and
                abs(wrap(goal["yaw"] - start["yaw"])) <= 1e-12,
                f"{label}: {previous['name']} goal is not the {following['name']} start")
    return stages


def clear_maneuver(loc, bias, registry, constants, reference, suffix):
    """tangent -> translation -> arrival heading from localized `loc`.

    `reference` is the ORIGINAL physical clear reference (the parent captured start); every
    stage is checked against it.  Returns (stages, final localized pose, reason).
    """
    stance = registry["physical_stance"]
    clear = registry["clear_point"]
    tol = constants["kClearPositionTolerance"]
    area = constants["kClearAreaDisplacement"]
    stages = []
    phys = offset_pose(loc, bias)

    def guard(label):
        if math.hypot(phys[0] - reference[0], phys[1] - reference[1]) > area:
            return f"clear_reference_displacement_exceeded:{label}"
        return None

    if math.hypot(clear[0] - phys[0], clear[1] - phys[1]) > tol:
        tangent = select_tangent(math.atan2(clear[1] - phys[1], clear[0] - phys[0]), phys[2])
        goal = (loc[0], loc[1], wrap(tangent - bias[2]))
        stages.append({"name": "tangent_heading" + suffix, "phase": "clear", "start": pose_dict(loc),
                       "goal": pose_dict(goal),
                       "provenance": "constructed: nearest forward/reverse tangent at localized XY"})
        loc = goal
        phys = offset_pose(loc, bias)
        reason = guard("tangent_heading")
        if reason:
            return None, None, reason
        target = (clear[0] - bias[0], clear[1] - bias[1], wrap(tangent - bias[2]))
        commanded = math.hypot(target[0] - loc[0], target[1] - loc[1])
        physical_command = math.hypot(clear[0] - phys[0], clear[1] - phys[1])
        if commanded > area or physical_command > area:
            return None, None, "clear_translation_bound_exceeded"
        stages.append({"name": "clear_translation" + suffix, "phase": "clear", "start": pose_dict(loc),
                       "goal": pose_dict(target),
                       "provenance": "constructed: translation to bias-corrected clear point"})
        loc = target
        phys = offset_pose(loc, bias)
        reason = guard("clear_translation")
        if reason:
            return None, None, reason
    arrival = (loc[0], loc[1], wrap(stance[2] - bias[2]))
    stages.append({"name": "arrival_heading" + suffix, "phase": "clear", "start": pose_dict(loc),
                   "goal": pose_dict(arrival),
                   "provenance": "constructed: mandatory arrival heading at localized clear XY"})
    loc = arrival
    phys = offset_pose(loc, bias)
    reason = guard("arrival_heading")
    if reason:
        return None, None, reason
    if math.hypot(phys[0] - clear[0], phys[1] - clear[1]) > tol:
        return None, None, "arrival_outside_clear_tolerance"
    return stages, loc, None


def dock_stages(loc, bias, registry, constants):
    stance = registry["physical_stance"]
    dock = (stance[0] - bias[0], stance[1] - bias[1], wrap(stance[2] - bias[2]))
    margin = constants["kFinalHeadingGoalMargin"]
    final = (dock[0], dock[1], wrap(stance[2] - bias[2] - margin))
    return [{"name": "dock", "phase": "dock", "start": pose_dict(loc), "goal": pose_dict(dock),
             "provenance": "constructed: precision dock to bias-corrected immutable stance"},
            {"name": "final_heading_margin", "phase": "dock", "start": pose_dict(dock),
             "goal": pose_dict(final),
             "provenance": "constructed: final heading target stance_yaw-bias_yaw-kFinalHeadingGoalMargin "
                           "(occurs after centered docking; no executed-action claim)"}]


def build_sequence(start_loc, bias, registry, constants):
    """Nominal captured-start sequence: (stages or None, reason, {}). Poses are map-frame (x, y, yaw)."""
    approach = registry["dispatch_approach"]
    start_phys = offset_pose(start_loc, bias)
    reference = (start_phys[0], start_phys[1])
    if math.hypot(reference[0] - approach[0], reference[1] - approach[1]) > constants["kRegisteredApproachAdmission"]:
        return None, "registered_approach_distance_exceeded", {}
    stages, loc, reason = clear_maneuver(start_loc, bias, registry, constants, reference, "")
    if reason:
        return None, ("nominal_" + reason if reason == "arrival_outside_clear_tolerance" else reason), {}
    stages += dock_stages(loc, bias, registry, constants)
    return require_continuous(stages, "nominal sequence"), None, {}


def build_repair_sequence(parent_start_loc, bias, registry, constants, miss):
    """Standalone synthetic repair-start sequence: (stages or None, reason, trace).

    The start is the clear point plus the physical `miss` at the stance yaw; its localized pose
    subtracts the selected captured bias.  The parent's captured-start ORIGINAL clear reference is
    kept for admission and the displacement bound (never reset to the repair start).  Exactly one
    repair tangent -> translation -> arrival heading, then dock and final heading.  The travel
    that produced the miss is not constructed.
    """
    approach = registry["dispatch_approach"]
    clear = registry["clear_point"]
    stance = registry["physical_stance"]
    parent_phys = offset_pose(parent_start_loc, bias)
    reference = (parent_phys[0], parent_phys[1])
    start_phys = (clear[0] + miss[0], clear[1] + miss[1], wrap(stance[2]))
    start_loc = (start_phys[0] - bias[0], start_phys[1] - bias[1], wrap(start_phys[2] - bias[2]))
    trace = {"original_reference": {"x": reference[0], "y": reference[1]},
             "start_localized": pose_dict(start_loc), "start_physical": pose_dict(start_phys)}
    if math.hypot(reference[0] - approach[0], reference[1] - approach[1]) > constants["kRegisteredApproachAdmission"]:
        return None, "registered_approach_distance_exceeded", trace
    if math.hypot(start_phys[0] - reference[0], start_phys[1] - reference[1]) > constants["kClearAreaDisplacement"]:
        return None, "clear_reference_displacement_exceeded:repair_start", trace
    if math.hypot(start_phys[0] - clear[0], start_phys[1] - clear[1]) <= constants["kClearPositionTolerance"]:
        return None, "repair_start_within_clear_tolerance", trace
    stages, loc, reason = clear_maneuver(start_loc, bias, registry, constants, reference, "_repair")
    if reason:
        return None, ("repair_did_not_converge" if reason == "arrival_outside_clear_tolerance" else reason), trace
    stages += dock_stages(loc, bias, registry, constants)
    return require_continuous(stages, "repair sequence"), None, trace


def build_cases(clear, pre_dock, registry, constants):
    cases = []
    bias_sources = [("own", None)] + [("pre_dock:" + c["geometry"], c) for c in pre_dock["captures"]]
    for capture in clear["captures"]:
        start = (capture["localized"]["x"], capture["localized"]["y"], capture["localized"]["yaw"])
        own_bias = (capture["bias"]["x"], capture["bias"]["y"], capture["bias"]["yaw"])
        for label, source in bias_sources:
            if source is None:
                bias, kind = own_bias, "nominal"
                provenance = "start localized pose and measured bias from the same clear-start capture"
            else:
                b = source["bias"]
                bias, kind = (b["x"], b["y"], b["yaw"]), "bias_sensitivity"
                provenance = BIAS_SENSITIVITY_NOTE + f" ({source['id']})"
            base_id = f"{capture['id']}|bias={label}"
            variants = [("", None)] + [(f"|repair_miss={name}", miss) for name, miss in (
                ("+x", (REPAIR_MISS_M, 0.0)), ("-x", (-REPAIR_MISS_M, 0.0)),
                ("+y", (0.0, REPAIR_MISS_M)), ("-y", (0.0, -REPAIR_MISS_M)))]
            parent_phys = offset_pose(start, bias)
            for suffix, miss in variants:
                if miss is None:
                    stages, reason, trace = build_sequence(start, bias, registry, constants)
                else:
                    stages, reason, trace = build_repair_sequence(start, bias, registry, constants, miss)
                case = {"id": base_id + suffix,
                        "kind": kind if miss is None else "synthetic_repair",
                        "synthetic": miss is not None,
                        "start_capture": capture["id"],
                        "bias_source": capture["id"] if source is None else source["id"],
                        "start_localized": trace.get("start_localized", pose_dict(start)),
                        "bias": pose_dict(bias),
                        "start_physical": trace.get("start_physical", pose_dict(parent_phys)),
                        "parent_original_reference": {
                            "x": parent_phys[0], "y": parent_phys[1],
                            "provenance": "original parent captured-start clear reference (captured localized "
                                          "start + selected bias); never reset to a repair start"},
                        "synthetic_physical_miss_m": None if miss is None else {"x": miss[0], "y": miss[1]},
                        "provenance": provenance if miss is None else
                        provenance + "; standalone synthetic repair start (clear point + physical miss at "
                        "stance yaw; localized pose subtracts the selected observed bias), not a captured "
                        "observation; no connecting motion or post-heading observation is invented",
                        "admitted": stages is not None, "admission_reason": reason or "admitted",
                        "stages": stages or []}
                if miss is not None:
                    case["start_kind"] = "synthetic_repair_start"
                    case["travel_producing_miss"] = "not_exercised"
                cases.append(case)
    return cases


# ---------------------------------------------------------------------------

def build_scene(baseline_dir, tf_dir, planner_config, stations, products, source_root=None,
                pinned=PINNED_SHA256):
    baseline_dir, tf_dir = Path(baseline_dir), Path(tf_dir)
    source_root = Path(source_root) if source_root else Path(__file__).resolve().parents[3]
    inputs = {}

    def identify(key, path, required=False):
        digest = sha256_file(path)
        require(not required or key in pinned, f"{key}: no pinned sha256 supplied")
        if key in pinned:
            require(digest == pinned[key], f"{key}: sha256 {digest} differs from pinned {pinned[key]}")
        inputs[key] = {"path": str(path), "sha256": digest, "bytes": os.path.getsize(path)}
        return path

    clear_path = identify("clear_scene_snapshot.json", baseline_dir / "clear_scene_snapshot.json")
    dock_path = identify("dock_scene_snapshot.json", baseline_dir / "dock_scene_snapshot.json")
    stream_path = identify("typed_stream.jsonl.gz", baseline_dir / "typed_stream.jsonl.gz", required=True)
    summary_path = identify("summary.json", tf_dir / "summary.json")
    tf_path = identify("tf_edge_rows.jsonl", tf_dir / "tf_edge_rows.jsonl")
    pose_path = identify("pose_rows.jsonl", tf_dir / "pose_rows.jsonl")
    identify("planner.yaml", Path(planner_config))
    for key, path in (("stations.yaml", stations), ("products.yaml", products)):
        identify(key, Path(path))
    summary = read_json(summary_path)
    status_text = (tf_dir / "run.status").read_text().strip()
    require(summary.get("hashes", {}).get("tf_edge_rows_jsonl_sha256") == inputs["tf_edge_rows.jsonl"]["sha256"] and
            summary.get("hashes", {}).get("pose_rows_jsonl_sha256") == inputs["pose_rows.jsonl"]["sha256"],
            "summary hashes disagree with retained JSONL files")
    require(summary.get("missing_required_brackets_or_boundaries") == [MISSING_AMCL] and status_text == "2",
            "extraction status is not the documented exit 2 / missing AMCL boundary")
    extraction = {"original_exit_status": int(status_text), "original_status": summary.get("status"),
                  "missing_optional_amcl_annotation": [MISSING_AMCL],
                  "disposition": "optional AMCL cross-check absent; original failed extraction retained unchanged",
                  "summary_sha256": inputs["summary.json"]["sha256"], "root": str(tf_dir)}

    texts = {"gate6": (source_root / "amr_manipulation/src/gate6_mass_stage.cpp").read_text(),
             "helper": (source_root / "amr_interfaces/include/amr_interfaces/final_placement_stance.hpp").read_text()}
    mission_text = (source_root / "amr_mission/src/mission_supervisor_node.cpp").read_text()
    constants = parse_source_constants(texts)
    budget = read_smoother_budget(mission_text)
    for key, rel in (("gate6_mass_stage.cpp", "amr_manipulation/src/gate6_mass_stage.cpp"),
                     ("final_placement_stance.hpp", "amr_interfaces/include/amr_interfaces/final_placement_stance.hpp"),
                     ("mission_supervisor_node.cpp", "amr_mission/src/mission_supervisor_node.cpp")):
        inputs[key] = {"path": str(source_root / rel), "sha256": sha256_file(source_root / rel)}
    config = load_config(planner_config)
    registry = load_registry(stations, products, constants)

    clear_snapshot, dock_snapshot = read_json(clear_path), read_json(dock_path)
    clear_obs = load_clear_observations(tf_path, pose_path)
    clear_stamps = []
    for item in ("costmap", "local_costmap"):
        clear_stamps.append(stamp_ns(clear_snapshot[item]["header"]["stamp"], item))
    for item in ("observed_footprint", "local_observed_footprint"):
        clear_stamps.append(stamp_ns(clear_snapshot[item]["fields"]["header"]["stamp"], item))
    require(sorted(clear_stamps) == sorted(summary.get("geometry_stamps_ns", [])),
            "snapshot geometry stamps disagree with the TF extraction geometry stamps")
    clear = build_scene_geometry("clear", clear_snapshot, clear_obs, config, "clear_start")
    dock_stamps = []
    for item in ("costmap", "local_costmap"):
        dock_stamps.append(stamp_ns(dock_snapshot[item]["header"]["stamp"], item))
    for item in ("observed_footprint", "local_observed_footprint"):
        dock_stamps.append(stamp_ns(dock_snapshot[item]["fields"]["header"]["stamp"], item))
    pre_obs = load_stream_observations(stream_path, min(dock_stamps), max(dock_stamps))
    pre_dock = build_scene_geometry("pre_dock", dock_snapshot, pre_obs, config, "pre_dock")

    # The snapshots' explicit targets must agree with the registry derivation.
    for name, a, b in (("clear snapshot goal vs clear point", clear["snapshot_goal"], registry["clear_point"]),
                       ("clear snapshot start vs stance", clear["snapshot_start"], registry["physical_stance"][:2]),
                       ("dock snapshot start vs clear point", pre_dock["snapshot_start"], registry["clear_point"]),
                       ("dock snapshot goal vs stance", pre_dock["snapshot_goal"], registry["physical_stance"][:2])):
        require(abs(a["x"] - b[0]) <= 1e-9 and abs(a["y"] - b[1]) <= 1e-9, f"registry disagrees: {name}")

    cases = build_cases(clear, pre_dock, registry, constants)
    admitted = [c for c in cases if c["admitted"]]
    scene = {
        "schema": SCHEMA,
        "identity": {"inputs": inputs, "adapter_sha256": sha256_file(__file__)},
        "extraction": extraction,
        "config": dict(config, smoother_budget_s=budget,
                       smoother_budget_source="mission_supervisor_node.cpp goal.max_smoothing_duration.sec"),
        "source_constants": constants,
        "registry": registry,
        "scenes": {"clear": {k: clear[k] for k in ("grids", "captures", "footprint_checks")},
                   "pre_dock": {k: pre_dock[k] for k in ("grids", "captures", "footprint_checks")}},
        "planning_grid_by_phase": {"clear": "clear.global", "dock": "pre_dock.global"},
        "cases": cases,
        "runtime_properties": {name: "not_exercised" for name in (
            "observer_settling", "fresh_post_heading_bias", "stationary_start",
            "action_execution", "controller_tracking", "terminal_state_guarantees")},
        "limitations": [
            "historical planner/config binary equivalence is not claimed",
            "captures were moving; bias is an explicit geometric scenario assumption",
            "arrival at each stage goal is assumed exact; synthetic misses are not observations",
        ],
    }
    report = {"schema": SCHEMA + "_report", "inputs": inputs, "extraction": extraction,
              "footprint_checks": clear["footprint_checks"] + pre_dock["footprint_checks"],
              "captures": {s["name"]: [{k: c[k] for k in ("id", "stamp_ns", "localized", "physical", "bias")}
                                       for c in s["captures"]] for s in (clear, pre_dock)},
              "cases_total": len(cases), "cases_admitted": len(admitted),
              "repair_candidates": sum(1 for c in cases if c["synthetic"]),
              "repair_candidates_admitted": sum(1 for c in admitted if c["synthetic"]),
              "cases_excluded": [{"id": c["id"], "reason": c["admission_reason"]} for c in cases
                                 if not c["admitted"]],
              "registry": registry}
    return scene, report


# ---------------------------------------------------------------------------
# Self-test: synthetic fixtures in the retained formats (no retained data read).

def _quat(yaw):
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def _stamp(ns):
    return {"sec": ns // 10**9, "nanosec": ns % 10**9}


def _tf_row(parent, child, ns, pose, ordinal):
    return {"topic": "/tf", "source_ordinal": ordinal, "acquisition_stamp_ns": ns,
            "cdr_payload_sha256": "00", "fields": {
                "header": {"frame_id": parent, "stamp": _stamp(ns)}, "child_frame_id": child,
                "transform": {"translation": {"x": pose[0], "y": pose[1], "z": 0.0},
                              "rotation": _quat(pose[2])}}}


def _gt_fields(ns, pose):
    return {"header": {"frame_id": "factory_world", "stamp": _stamp(ns)},
            "pose": {"position": {"x": pose[0], "y": pose[1], "z": 0.0}, "orientation": _quat(pose[2])}}


def _write_jsonl(path, rows, gz=False):
    opener = gzip.open if gz else open
    with opener(path, "wt") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def _fixture_config(root):
    planner = root / "planner.yaml"
    planner.write_text("""
/amr/planner_server:
  ros__parameters:
    PrecisionGridBased: {plugin: amr_navigation/PrecisionNavfnPlanner, use_astar: true, allow_unknown: false, tolerance: 0.01}
/amr/global_costmap/global_costmap:
  ros__parameters:
    resolution: 0.05
    footprint: "[[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]]"
    footprint_padding: 0.01
/amr/smoother_server:
  ros__parameters:
    simple_smoother: {plugin: "nav2_smoother::SimpleSmoother", tolerance: 1.0e-10, max_its: 1000, w_data: 0.2, w_smooth: 0.0, do_refinement: true}
""")
    stations = root / "stations.yaml"
    stations.write_text("""
stations:
  dispatch:
    approach: {x: -2.5, y: 0.0, yaw: 3.141592653589793}
    dock: {x: -3.4, y: 0.0, yaw: 3.141592653589793}
""")
    products = root / "products.yaml"
    products.write_text("""
products:
  product_b: {tag_id: 102, dispatch_slot: dispatch_2}
dispatch_slots:
  - {id: dispatch_2, x: -4.10, y: 0.00, z: 0.075}
""")
    src = root / "src"
    for rel, text in {
        "amr_manipulation/src/gate6_mass_stage.cpp":
            "constexpr double kClearPositionTolerance = 0.01;\nconstexpr double kRegisteredApproachAdmission = 0.155;\n"
            "constexpr double kClearAreaDisplacement = 0.15;\nconstexpr double kHeadingTolerance = 0.15;\n"
            "constexpr double kFinalHeadingGoalMargin = 0.03;\n",
        "amr_interfaces/include/amr_interfaces/final_placement_stance.hpp":
            "constexpr double kDesiredProduct102SlotBaseX = 0.748000000;\n"
            "constexpr double kDesiredProduct102SlotBaseY = 0.100000000;\n",
        "amr_mission/src/mission_supervisor_node.cpp": "goal.max_smoothing_duration.sec = 1;\n"}.items():
        path = src / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    return planner, stations, products, src


def _fixture_scene(root, dock_start_bias=(0.0, 0.0, 0.0)):
    """Two synthetic scenes; the base moves linearly so interpolation is checkable."""
    root.mkdir(parents=True)
    planner, stations, products, src = _fixture_config(root)
    baseline, tfdir = root / "baseline", root / "tf"
    baseline.mkdir()
    tfdir.mkdir()
    padded = [(0.61, 0.41), (0.61, -0.41), (-0.61, -0.41), (-0.61, 0.41)]

    def scene(anchor, t0, base0, velocity, map_odom, bias):
        stamps = [t0 + 10_000_000, t0 + 20_000_000, t0 + 30_000_000, t0 + 40_000_000]
        odom_base = lambda t: (base0[0] + velocity * (t - t0) * 1e-9, base0[1], base0[2])
        snap = {"schema": 1, "source": {"anchor_name": anchor}, "missing_or_invalid": [],
                "replay_inputs_complete": True, "footprint": [[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]],
                "footprint_padding_m": 0.01}

        def grid(frame, stamp):
            return {"header": {"frame_id": frame, "stamp": _stamp(stamp)}, "frame_id": frame,
                    "width": 3, "height": 2, "resolution": 0.5,
                    "metadata": {"size_x": 3, "size_y": 2, "resolution": 0.5,
                                 "origin": {"position": {"x": -1.0, "y": -1.0}}},
                    "origin": {"x": -1.0, "y": -1.0, "yaw": 0.0}, "data": [0, 1, 100, 252, 253, 255],
                    "topic": "/x"}

        def poly(frame, stamp, base):
            pts = []
            for corner in padded:
                p = compose(base, (corner[0], corner[1], 0.0))
                pts.append({"x": p[0], "y": p[1], "z": 0.0})
            return {"fields": {"header": {"frame_id": frame, "stamp": _stamp(stamp)},
                               "polygon": {"points": pts}}}

        def base_in(frame, t):
            ob = odom_base(t)
            return compose(map_odom, ob) if frame == "map" else ob

        snap["costmap"] = grid("map", stamps[0])
        snap["local_costmap"] = grid("odom", stamps[1])
        snap["observed_footprint"] = poly("map", stamps[2], base_in("map", stamps[2]))
        snap["local_observed_footprint"] = poly("odom", stamps[3], base_in("odom", stamps[3]))
        snap["start"] = {"x": 0.0, "y": 0.0}
        snap["goal"] = {"x": 0.0, "y": 0.0}
        tf, gt = [], []
        for k in range(-2, 8):
            t = t0 + k * 10_000_000
            tf.append(("odom", "base_footprint", t, odom_base(t)))
            gt.append((t, offset_pose(compose(map_odom, odom_base(t)), bias)))
        tf.append(("map", "odom", t0, map_odom))
        tf.append(("map", "odom", t0 + 100_000_000, map_odom))
        return snap, tf, gt, stamps

    registry_stance = (-3.352, 0.1, math.pi)
    clear_point = (-2.5, 0.1)
    bias_c = (0.002, 0.03, -0.007)
    snap_c, tf_c, gt_c, st_c = scene("clear_start", 401_000_000_000, (-2.4, 0.0, 3.0), 0.5,
                                     (0.0, 0.0, 0.0), bias_c)
    snap_d, tf_d, gt_d, st_d = scene("pre_dock", 410_000_000_000, (-2.45, 0.02, 3.1), 0.0,
                                     (0.0, 0.0, 0.0), dock_start_bias)
    snap_c["goal"], snap_c["start"] = {"x": clear_point[0], "y": clear_point[1]}, {"x": registry_stance[0], "y": registry_stance[1]}
    snap_d["goal"], snap_d["start"] = {"x": registry_stance[0], "y": registry_stance[1]}, {"x": clear_point[0], "y": clear_point[1]}
    (baseline / "clear_scene_snapshot.json").write_text(json.dumps(snap_c))
    (baseline / "dock_scene_snapshot.json").write_text(json.dumps(snap_d))
    rows = [_tf_row(p, c, t, pose, i) for i, (p, c, t, pose) in enumerate(tf_c)]
    _write_jsonl(tfdir / "tf_edge_rows.jsonl", rows)
    _write_jsonl(tfdir / "pose_rows.jsonl", [
        {"topic": GT_TOPIC, "source_ordinal": i, "acquisition_stamp_ns": t, "cdr_payload_sha256": "00",
         "fields": _gt_fields(t, p)} for i, (t, p) in enumerate(gt_c)])
    stream = [{"topic": "/tf", "traversal_ordinal": i, "cdr_payload_sha256": "00",
               "fields": {"transforms": [_tf_row(p, c, t, pose, i)["fields"]]}}
              for i, (p, c, t, pose) in enumerate(tf_d)]
    stream += [{"topic": GT_TOPIC, "traversal_ordinal": 1000 + i, "cdr_payload_sha256": "00",
                "fields": _gt_fields(t, p)} for i, (t, p) in enumerate(gt_d)]
    _write_jsonl(baseline / "typed_stream.jsonl.gz", stream, gz=True)
    summary = {"status": "blocked_missing_required_bracket_or_pose_boundary",
               "missing_required_brackets_or_boundaries": [MISSING_AMCL],
               "geometry_stamps_ns": st_c,
               "hashes": {"tf_edge_rows_jsonl_sha256": sha256_file(tfdir / "tf_edge_rows.jsonl"),
                          "pose_rows_jsonl_sha256": sha256_file(tfdir / "pose_rows.jsonl")}}
    (tfdir / "summary.json").write_text(json.dumps(summary))
    (tfdir / "run.status").write_text("2\n")
    pins = {k: sha256_file(p) for k, p in (
        ("clear_scene_snapshot.json", baseline / "clear_scene_snapshot.json"),
        ("dock_scene_snapshot.json", baseline / "dock_scene_snapshot.json"),
        ("summary.json", tfdir / "summary.json"), ("tf_edge_rows.jsonl", tfdir / "tf_edge_rows.jsonl"),
        ("pose_rows.jsonl", tfdir / "pose_rows.jsonl"), ("planner.yaml", planner),
        ("typed_stream.jsonl.gz", baseline / "typed_stream.jsonl.gz"))}
    return dict(baseline_dir=baseline, tf_dir=tfdir, planner_config=planner, stations=stations,
                products=products, source_root=src, pinned=pins), bias_c


def _expect_error(function, fragment, label):
    try:
        function()
    except AdapterError as error:
        assert fragment in str(error), f"{label}: wrong error: {error}"
        return
    raise AssertionError(f"{label}: malformed input was accepted")


def self_test():
    results = []
    with tempfile.TemporaryDirectory(prefix="native45_adapter_selftest_") as tmp:
        root = Path(tmp)
        kwargs, bias_c = _fixture_scene(root / "ok")
        scene, report = build_scene(**kwargs)
        assert scene["schema"] == SCHEMA and all(v == "not_exercised" for v in scene["runtime_properties"].values())
        assert scene["extraction"]["original_exit_status"] == 2
        assert scene["extraction"]["missing_optional_amcl_annotation"] == [MISSING_AMCL]
        results.append("schema+extraction annotation")

        # Centered stance / clear point from source-pinned constants.
        reg = scene["registry"]
        assert max(abs(a - b) for a, b in zip(reg["physical_stance"], (-3.352, 0.1, math.pi))) <= 1e-9
        assert max(abs(a - b) for a, b in zip(reg["clear_point"], (-2.5, 0.1))) <= 1e-9
        results.append("registry stance and clear point")

        # TF interpolation with no extrapolation; moving base at 0.5 m/s.
        capture = scene["scenes"]["clear"]["captures"][0]
        assert abs(capture["localized"]["x"] - (-2.4 + 0.5 * 0.01)) <= 1e-9
        assert capture["brackets"]["odom_to_base_footprint"]["exact"] is True
        assert abs(capture["bias"]["x"] - bias_c[0]) <= 1e-9 and abs(capture["bias"]["yaw"] - bias_c[2]) <= 1e-9
        obs = Observations()
        obs.edges[EDGES[1]].add(100, (0.0, 0.0, math.pi - 0.1), {"k": 0})
        obs.edges[EDGES[1]].add(200, (1.0, 0.0, -math.pi + 0.1), {"k": 1})
        obs.finalize()
        mid, info = obs.edges[EDGES[1]].at(150)
        assert abs(mid[0] - 0.5) < 1e-12 and abs(abs(mid[2]) - math.pi) < 1e-12 and info["exact"] is False
        _expect_error(lambda: obs.edges[EDGES[1]].at(99), "not bracketed", "before first")
        _expect_error(lambda: obs.edges[EDGES[1]].at(201), "not bracketed", "after last")
        results.append("TF interpolation/no extrapolation")

        # Footprint reconstruction recorded; mismatch rejected.
        assert all(c["max_vertex_residual_m"] <= POLYGON_TOLERANCE_M for c in report["footprint_checks"])
        assert check_polygon([(0.6, 0.4), (0.6, -0.4), (-0.6, -0.4), (-0.6, 0.4)], (0, 0, 0),
                             [(0.61, 0.41), (0.61, -0.41), (-0.61, -0.41), (-0.61, 0.41)]) > POLYGON_TOLERANCE_M
        results.append("padded polygon discrepancy")

        # Construction contracts.
        cases = {c["id"]: c for c in scene["cases"]}
        own = cases["clear:global_costmap|bias=own"]
        assert own["admitted"] and [s["name"] for s in own["stages"]] == [
            "tangent_heading", "clear_translation", "arrival_heading", "dock", "final_heading_margin"]
        stage = {s["name"]: s for s in own["stages"]}
        b = own["bias"]
        assert abs(stage["dock"]["goal"]["x"] - (-3.352 - b["x"])) <= 1e-9
        assert abs(wrap(stage["final_heading_margin"]["goal"]["yaw"] -
                        (stage["dock"]["goal"]["yaw"] - 0.03))) <= 1e-9
        assert abs(stage["clear_translation"]["goal"]["y"] - (0.1 - b["y"])) <= 1e-9
        assert sum(1 for c in scene["cases"] if c["kind"] == "bias_sensitivity") == 16
        assert sum(1 for c in scene["cases"] if c["kind"] == "nominal") == 4
        assert select_tangent(math.pi, math.pi / 2) == math.pi  # equality -> forward
        assert select_tangent(math.pi, 0.1) == 0.0  # reverse is nearer
        results.append("stage construction")

        # Standalone synthetic repair-start cases.
        constants = scene["source_constants"]
        repairs = [c for c in scene["cases"] if c["kind"] == "synthetic_repair"]
        assert len(repairs) == 80 and len(scene["cases"]) == 100
        assert len({(c["start_capture"], c["bias_source"]) for c in repairs}) == 20
        assert report["repair_candidates"] == 80
        assert report["repair_candidates_admitted"] == sum(1 for c in repairs if c["admitted"]) > 0
        assert {e["id"] for e in report["cases_excluded"]} == {c["id"] for c in scene["cases"]
                                                              if not c["admitted"]}
        assert all(e["reason"] != "admitted" for e in report["cases_excluded"])
        clear_point, stance = reg["clear_point"], reg["physical_stance"]
        for case in scene["cases"]:
            if not case["admitted"]:
                continue
            stages = case["stages"]
            for previous, following in zip(stages, stages[1:]):  # continuity of every admitted sequence
                assert math.hypot(previous["goal"]["x"] - following["start"]["x"],
                                  previous["goal"]["y"] - following["start"]["y"]) <= 1e-12, case["id"]
                assert abs(wrap(previous["goal"]["yaw"] - following["start"]["yaw"])) <= 1e-12, case["id"]
            bias = case["bias"]
            ref = case["parent_original_reference"]
            for stage in stages:
                if stage["phase"] != "clear":
                    continue
                for end in ("start", "goal"):
                    px, py = stage[end]["x"] + bias["x"], stage[end]["y"] + bias["y"]
                    assert math.hypot(px - ref["x"], py - ref["y"]) <= constants["kClearAreaDisplacement"] + 1e-9, \
                        (case["id"], stage["name"])
        repair = cases["clear:global_costmap|bias=own|repair_miss=+x"]
        assert repair["synthetic"] and repair["admitted"] and "synthetic" in repair["provenance"]
        assert [s["name"] for s in repair["stages"]] == [
            "tangent_heading_repair", "clear_translation_repair", "arrival_heading_repair",
            "dock", "final_heading_margin"]
        parent = scene["scenes"]["clear"]["captures"][0]
        assert parent["id"] == repair["start_capture"] and repair["start_kind"] == "synthetic_repair_start"
        assert repair["travel_producing_miss"] == "not_exercised"
        assert repair["bias"] == parent["bias"]  # selected observed bias
        assert abs(repair["start_physical"]["x"] - (clear_point[0] + REPAIR_MISS_M)) <= 1e-9
        assert abs(repair["start_physical"]["y"] - clear_point[1]) <= 1e-9
        assert abs(wrap(repair["start_physical"]["yaw"] - stance[2])) <= 1e-9  # stance heading
        assert abs(repair["start_localized"]["x"] - (repair["start_physical"]["x"] - parent["bias"]["x"])) <= 1e-9
        assert abs(wrap(repair["start_localized"]["yaw"] -
                        (repair["start_physical"]["yaw"] - parent["bias"]["yaw"]))) <= 1e-9
        assert repair["stages"][0]["start"] == repair["start_localized"]
        # Parent original reference is the captured start, not the repair start.
        assert abs(repair["parent_original_reference"]["x"] -
                   (parent["localized"]["x"] + parent["bias"]["x"])) <= 1e-9
        assert abs(repair["parent_original_reference"]["x"] - repair["start_physical"]["x"]) > 1e-3
        results.append("synthetic repair-start cases, continuity, original reference")

        # Registered admission and original-reference exclusions.
        zero = (0.0, 0.0, 0.0)
        far, reason, _ = build_sequence((-2.0, 0.0, 0.0), zero, reg, constants)
        assert far is None and reason == "registered_approach_distance_exceeded"
        far, reason, _ = build_repair_sequence((-2.0, 0.0, 0.0), zero, reg, constants, (REPAIR_MISS_M, 0.0))
        assert far is None and reason == "registered_approach_distance_exceeded"
        far, reason, _ = build_repair_sequence((-2.4, 0.0, 0.0), zero, reg, constants, (0.2, 0.0))
        assert far is None and reason == "clear_translation_bound_exceeded"
        # Safe relative to its own start, but beyond the parent's original reference (0.1552 m > 0.15 m).
        miss = (REPAIR_MISS_M, 0.0)
        near, reason, _ = build_repair_sequence((clear_point[0], clear_point[1], 0.0), zero, reg, constants, miss)
        assert near is not None and reason is None
        beyond = (clear_point[0], clear_point[1] - 0.1549, 0.0)  # within 0.155 m of the registered approach
        far, reason, trace = build_repair_sequence(beyond, zero, reg, constants, miss)
        assert far is None and reason == "clear_reference_displacement_exceeded:repair_start", reason
        assert abs(trace["original_reference"]["y"] - beyond[1]) <= 1e-12
        results.append("admission and original-reference exclusions")

        # Typed stream is mandatorily pinned: a changed retained stream is rejected before acceptance.
        assert PINNED_SHA256["typed_stream.jsonl.gz"] == \
            "f81dc4619b4d8a0d5e66b93fe4d35faaaecdd18f4a3dfff23cb12b71b3c80c66"
        assert kwargs["pinned"]["typed_stream.jsonl.gz"] == scene["identity"]["inputs"]["typed_stream.jsonl.gz"]["sha256"]
        kw_stream, _ = _fixture_scene(root / "stream_tamper")
        rows = list(read_jsonl(kw_stream["baseline_dir"] / "typed_stream.jsonl.gz", gz=True))
        rows[-1]["fields"]["pose"]["position"]["x"] += 0.25
        _write_jsonl(kw_stream["baseline_dir"] / "typed_stream.jsonl.gz", rows, gz=True)
        assert sha256_file(kw_stream["baseline_dir"] / "typed_stream.jsonl.gz") != \
            kw_stream["pinned"]["typed_stream.jsonl.gz"]
        _expect_error(lambda: build_scene(**kw_stream), "typed_stream.jsonl.gz: sha256", "stream_tamper")
        kw_stream, _ = _fixture_scene(root / "stream_unpinned")
        del kw_stream["pinned"]["typed_stream.jsonl.gz"]
        _expect_error(lambda: build_scene(**kw_stream), "no pinned sha256", "stream_unpinned")
        results.append("typed stream pin and tamper rejection")

        # Malformed-input rejections.
        def mutated(name, mutate, fragment):
            kw, _ = _fixture_scene(root / name)
            mutate(kw)
            _expect_error(lambda: build_scene(**kw), fragment, name)

        def edit_snapshot(file, fn):
            def run(kw):
                path = kw["baseline_dir"] / file
                data = json.loads(path.read_text())
                fn(data)
                path.write_text(json.dumps(data))
                kw["pinned"] = dict(kw["pinned"], **{file: sha256_file(path)})
            return run

        mutated("bad_hash", lambda kw: kw["pinned"].update({"planner.yaml": "0" * 64}), "differs from pinned")
        mutated("bad_frame", edit_snapshot("clear_scene_snapshot.json",
                lambda d: d["local_costmap"].update(frame_id="map")), "wrong frame")
        mutated("bad_stamp", edit_snapshot("clear_scene_snapshot.json",
                lambda d: d["costmap"]["header"]["stamp"].update(sec=0, nanosec=0)), "zero stamp")
        mutated("bad_grid_length", edit_snapshot("clear_scene_snapshot.json",
                lambda d: d["costmap"]["data"].pop()), "raw cost length")
        mutated("bad_cost", edit_snapshot("clear_scene_snapshot.json",
                lambda d: d["costmap"]["data"].__setitem__(0, 256)), "outside 0..255")
        mutated("bad_footprint", edit_snapshot("clear_scene_snapshot.json",
                lambda d: d["observed_footprint"]["fields"]["polygon"]["points"][0].update(x=9.0)), "residual")
        mutated("nonfinite_pose", edit_snapshot("dock_scene_snapshot.json",
                lambda d: d["observed_footprint"]["fields"]["polygon"]["points"][0].update(x=float("nan"))),
                "malformed footprint vertex")

        def break_tf(kw):
            rows = [json.loads(l) for l in open(kw["tf_dir"] / "tf_edge_rows.jsonl")]
            rows = [r for r in rows if not (r["fields"]["child_frame_id"] == "odom" and
                                            r["acquisition_stamp_ns"] > 401_000_000_000)]
            _write_jsonl(kw["tf_dir"] / "tf_edge_rows.jsonl", rows)
            summary = json.loads((kw["tf_dir"] / "summary.json").read_text())
            summary["hashes"]["tf_edge_rows_jsonl_sha256"] = sha256_file(kw["tf_dir"] / "tf_edge_rows.jsonl")
            (kw["tf_dir"] / "summary.json").write_text(json.dumps(summary))
            kw["pinned"] = dict(kw["pinned"], **{"tf_edge_rows.jsonl": sha256_file(kw["tf_dir"] / "tf_edge_rows.jsonl"),
                                                 "summary.json": sha256_file(kw["tf_dir"] / "summary.json")})
        mutated("unbracketed_tf", break_tf, "not bracketed")

        def bad_quaternion(kw):
            rows = [json.loads(l) for l in open(kw["tf_dir"] / "tf_edge_rows.jsonl")]
            rows[0]["fields"]["transform"]["rotation"]["w"] = 2.0
            _write_jsonl(kw["tf_dir"] / "tf_edge_rows.jsonl", rows)
            summary = json.loads((kw["tf_dir"] / "summary.json").read_text())
            summary["hashes"]["tf_edge_rows_jsonl_sha256"] = sha256_file(kw["tf_dir"] / "tf_edge_rows.jsonl")
            (kw["tf_dir"] / "summary.json").write_text(json.dumps(summary))
            kw["pinned"] = dict(kw["pinned"], **{"tf_edge_rows.jsonl": sha256_file(kw["tf_dir"] / "tf_edge_rows.jsonl"),
                                                 "summary.json": sha256_file(kw["tf_dir"] / "summary.json")})
        mutated("bad_quaternion", bad_quaternion, "quaternion is not unit")

        def changed_constant(kw):
            path = kw["source_root"] / "amr_manipulation/src/gate6_mass_stage.cpp"
            path.write_text(path.read_text().replace("kFinalHeadingGoalMargin = 0.03", "kFinalHeadingGoalMargin = 0.05"))
        mutated("changed_margin", changed_constant, "differs from packet value")

        def changed_budget(kw):
            path = kw["source_root"] / "amr_mission/src/mission_supervisor_node.cpp"
            path.write_text("goal.max_smoothing_duration.sec = 2;\n")
        mutated("changed_budget", changed_budget, "not 1 second")
        results.append("malformed-input rejections (hash, frame, stamp, grid, cost, footprint, nonfinite, "
                       "unbracketed TF, quaternion, changed constants)")
    for line in results:
        print("PASS", line)
    print("native45_scene_adapter self-test PASS")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    for name in ("baseline-dir", "tf-extract-dir", "planner-config", "stations-config",
                 "products-config", "output", "report"):
        parser.add_argument("--" + name)
    args = parser.parse_args(argv)
    try:
        if args.self_test:
            return self_test()
        missing = [n for n in ("baseline_dir", "tf_extract_dir", "planner_config", "stations_config",
                               "products_config", "output", "report") if getattr(args, n) is None]
        if missing:
            parser.error("missing required options: " + ", ".join(missing))
        scene, report = build_scene(args.baseline_dir, args.tf_extract_dir, args.planner_config,
                                    args.stations_config, args.products_config)
        for path, payload in ((args.output, scene), (args.report, report)):
            with open(path, "x", encoding="utf-8") as handle:  # never overwrite evidence
                json.dump(payload, handle, indent=1, sort_keys=True)
                handle.write("\n")
        print(f"scene written: cases={report['cases_total']} admitted={report['cases_admitted']} "
              f"excluded={len(report['cases_excluded'])}")
        return 0
    except AdapterError as error:
        print(f"native45_scene_adapter FAILED: {error}", file=sys.stderr)
        return 2
    except AssertionError as error:
        print(f"native45_scene_adapter self-test FAILED: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
