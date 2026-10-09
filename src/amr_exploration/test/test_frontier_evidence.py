"""Evidence for the run-12 investigation (root-authored with Codex Sol review;
implementer must not edit).

Problem A (false COMPLETE from an unsafe start) is proven deterministically;
its attribution to run 12 is not.  Problem B (why the robot ends up touching a
lethal cell) is NOT reproduced; these tests (1) make the honest-INCOMPLETE
reason precise, (2) require observation-only evidence records so the next
occurrence can be classified, and (3) pin possibility fixtures for B1/B2.
"""
import array
import hashlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR.parent / "scripts"))
sys.path.insert(0, str(TEST_DIR))

import frontier_algorithm as fa  # noqa: E402
import frontier_explorer as fe  # noqa: E402
import planning_fixtures as pf  # noqa: E402
import test_frontier_approach as approach  # noqa: E402
import test_frontier_goal_clearance as clearance  # noqa: E402
import test_frontier_lifecycle as lifecycle  # noqa: E402

CIRCUMSCRIBED = max(math.hypot(x, y) for x, y in fa.NAVIGATION_FOOTPRINT)


def _records(node, kind):
    return [r for r in node.evidence_records if r.get("kind") == kind]


def _map_sha(grid):
    return hashlib.sha256(array.array("b", grid.data).tobytes()).hexdigest()


def _costmap_sha(costmap):
    return hashlib.sha256(bytes(costmap.data)).hexdigest()


def _run_until_terminal_or_dispatch(node, pose):
    for _ in range(node.no_frontier_limit):
        clearance._plan(node, pose)
        if node.action_client.send_calls or node.state != "SCANNING":
            break


# ------------------------------------------------- (1) honest INCOMPLETE text

def test_unsafe_start_reason_says_completion_cannot_be_certified_with_cells():
    node, pose = clearance._node_with_fixture("hospital_12_unsafe_start")
    _run_until_terminal_or_dispatch(node, pose)
    assert node.state == "INCOMPLETE"
    assert node.reason == (
        "exploration incomplete: completion cannot be certified; "
        "robot pose is not collision-free")
    terminal = _records(node, "terminal")
    assert terminal, "terminal decision must leave an evidence record"
    record = terminal[-1]
    assert record["state"] == "INCOMPLETE"
    assert record["start_check"] == "collision"
    # The single overlapping lethal cell found offline (root + Sol):
    # costmap cell (294, 311), cost 254, SLAM map value 100.
    cells = {(c["x"], c["y"]): c for c in record["overlapping_cells"]}
    assert (294, 311) in cells
    assert cells[(294, 311)]["cost"] == 254
    assert cells[(294, 311)]["map_value"] == 100
    assert record["pose"]["x"] == pytest.approx(pose[0])
    assert record["pose"]["y"] == pytest.approx(pose[1])


def test_inadmissible_start_without_lethal_overlap_has_its_own_reason():
    """Start centre on cost 253 (inscribed), footprint touching no >=254 cell:
    the route proof cannot start, but this is not a footprint collision."""
    grid, costmap = approach._scene(raytrace_cleared=True)
    robot_cell = (10, 20)
    data = list(costmap.data)
    data[robot_cell[1] * approach.W + robot_cell[0]] = 253
    costmap.data = data
    node = lifecycle._node(autostart=True)
    node.latest_map = grid
    node.latest_costmap = costmap
    node.map_version += 1
    node.costmap_version += 1
    frozen = node._monotonic()
    node._monotonic = lambda: frozen
    pose = ((robot_cell[0] + 0.5) * approach.RES, (robot_cell[1] + 0.5) * approach.RES, 0.0)
    _run_until_terminal_or_dispatch(node, pose)
    assert node.action_client.send_calls == []
    assert node.state == "INCOMPLETE"
    assert node.reason == (
        "exploration incomplete: completion cannot be certified; "
        "route start is inadmissible")
    record = _records(node, "terminal")[-1]
    assert record["start_check"] == "inadmissible"
    assert record["overlapping_cells"] == []


# ---------------------------------------------------- (2) evidence records

def test_selector_reports_the_gate_actually_applied_per_candidate():
    kwargs = dict(robot_world=approach.ROBOT, min_goal_distance=0.3,
                  footprint=fa.NAVIGATION_FOOTPRINT, robot_yaw=0.0,
                  require_path_clear=False, return_diagnostics=True)
    grid, costmap = approach._scene(raytrace_cleared=True)
    clusters = fa.frontier_clusters(approach.W, approach.H, grid.data)
    candidates, _diag, tiers = fa.costmap_frontier_candidates(
        clusters, grid, costmap, (), approach_radius=0.0, return_tiers=True, **kwargs)
    assert candidates and all(tiers[c] == "frontier_cell" for c in candidates)
    grid, costmap = approach._scene()
    clusters = fa.frontier_clusters(approach.W, approach.H, grid.data)
    candidates, _diag, tiers = fa.costmap_frontier_candidates(
        clusters, grid, costmap, (), approach_radius=1.5, return_tiers=True, **kwargs)
    assert candidates and all(tiers[c] == "approach" for c in candidates)
    # Default return shape is unchanged.
    assert len(fa.costmap_frontier_candidates(
        clusters, grid, costmap, (), approach_radius=1.5, **kwargs)) == 2


def test_dispatch_evidence_identifies_goal_gate_snapshots_and_pose():
    node, _pose = clearance._node_with_fixture("hospital_12_unsafe_start")
    pose = (0.0, 0.0, 0.0)
    _run_until_terminal_or_dispatch(node, pose)
    assert node.action_client.send_calls
    record = _records(node, "dispatch")[-1]
    sent = node.action_client.send_calls[-1].pose.pose.position
    assert record["goal_world"] == pytest.approx([sent.x, sent.y], abs=1e-9)
    assert record["gate"] in ("frontier_cell", "approach")
    expected_radius = None if record["gate"] == "frontier_cell" else fe.GOAL_CLEARANCE_RADIUS_M
    assert record["gate_radius"] == (pytest.approx(expected_radius) if expected_radius else None)
    assert record["map"]["sha256"] == _map_sha(node.latest_map)
    assert record["costmap"]["sha256"] == _costmap_sha(node.latest_costmap)
    for key in ("width", "height", "resolution", "origin_x", "origin_y", "frame_id"):
        assert key in record["map"] and key in record["costmap"]
    assert record["costmap"]["width"] == node.latest_costmap.metadata.size_x
    assert record["pose"]["x"] == pytest.approx(pose[0])
    assert record["pose"]["yaw"] == pytest.approx(pose[2])
    assert record["run_generation"] == node.run_generation
    assert record["footprint"] == [list(p) for p in fa.NAVIGATION_FOOTPRINT]


def test_goal_reached_evidence_links_to_the_dispatch():
    node, _pose = clearance._node_with_fixture("hospital_12_unsafe_start")
    _run_until_terminal_or_dispatch(node, (0.0, 0.0, 0.0))
    dispatch = _records(node, "dispatch")[-1]
    handle = lifecycle._accept(node)
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    reached = _records(node, "goal_reached")
    assert reached, "goal success must leave an evidence record"
    assert reached[-1]["goal_world"] == dispatch["goal_world"]
    assert reached[-1]["gate"] == dispatch["gate"]
    assert reached[-1]["run_generation"] == dispatch["run_generation"]
    assert "pose" in reached[-1]          # may be None if TF is unavailable


def test_evidence_records_are_bounded():
    node, _pose = clearance._node_with_fixture("hospital_12_unsafe_start")
    for index in range(fe.EVIDENCE_RECORD_LIMIT + 50):
        node._record_evidence({"kind": "probe", "index": index})
    assert len(node.evidence_records) == fe.EVIDENCE_RECORD_LIMIT
    assert node.evidence_records[-1]["index"] == fe.EVIDENCE_RECORD_LIMIT + 49


# ------------------------------- (3) B1 / B2 possibility (not what happened)

def _empty_costmap(width=80, height=80, res=0.05):
    return lifecycle._costmap_message(
        data=[0] * (width * height), width=width, height=height, resolution=res,
        origin_x=0.0, origin_y=0.0)


def test_possibility_B1_arrival_offset_defeats_a_zero_margin_gate():
    """A goal passing the circumscribed disk can collide after a translated
    stop inside the 0.07 m xy tolerance; the 0.855 m gate rejects that goal."""
    costmap = _empty_costmap()
    geometry = fa.costmap_geometry(costmap)
    goal = (2.0, 2.0)
    corner = math.atan2(0.41, 0.61)
    # Lethal cell whose nearest point is 0.76 m from the goal along the
    # yaw-0 front-left corner direction.
    target = (goal[0] + 0.785 * math.cos(corner), goal[1] + 0.785 * math.sin(corner))
    cell = (int(target[0] / 0.05), int(target[1] / 0.05))
    data = list(costmap.data)
    data[cell[1] * 80 + cell[0]] = 254
    costmap.data = data
    fp = fa._strict_footprint(fa.NAVIGATION_FOOTPRINT)
    gate_nominal = fa._turn_clearance_checker(geometry, costmap.data, fp)
    gate_margin = fa._turn_clearance_checker(geometry, costmap.data, fp, radius=CIRCUMSCRIBED + 0.12)
    assert gate_nominal(goal)
    assert fa._footprint_costmap_clear(geometry, costmap.data, goal, fp, yaw=0.0)
    stop = (goal[0] + 0.069 * math.cos(corner), goal[1] + 0.069 * math.sin(corner))
    assert not fa._footprint_costmap_clear(geometry, costmap.data, stop, fp, yaw=0.0)
    assert not gate_margin(goal)


def test_possibility_B2_tier1_goal_collides_within_yaw_tolerance():
    """A frontier-cell goal admitted by the selector with a yaw-0 footprint
    clear can collide at a yaw inside the 0.15 rad goal tolerance."""
    grid, costmap = approach._scene(raytrace_cleared=True)
    clusters = fa.frontier_clusters(approach.W, approach.H, grid.data)
    kwargs = dict(robot_world=approach.ROBOT, min_goal_distance=0.3,
                  footprint=fa.NAVIGATION_FOOTPRINT, robot_yaw=0.0,
                  require_path_clear=False, return_diagnostics=True,
                  approach_radius=0.0)
    candidates = fa.costmap_frontier_candidates(clusters, grid, costmap, (), **kwargs)[0]
    geometry = fa.costmap_geometry(costmap)
    fp = fa._strict_footprint(fa.NAVIGATION_FOOTPRINT)
    found = None
    for cell in candidates:
        world = approach._world(cell)
        for dx in range(-9, 10):
            for dy in range(-9, 10):
                probe = (cell[0] + dx, cell[1] + dy)
                if not (0 <= probe[0] < approach.UNKNOWN_FROM_X and 0 <= probe[1] < approach.H):
                    continue
                data = list(costmap.data)
                data[probe[1] * approach.W + probe[0]] = 254
                if (fa._footprint_costmap_clear(geometry, data, world, fp, yaw=0.0)
                        and not fa._footprint_costmap_clear(geometry, data, world, fp, yaw=0.149)):
                    trial = SimpleNamespace(**vars(costmap))
                    trial.data = data
                    admitted = fa.costmap_frontier_candidates(clusters, grid, trial, (), **kwargs)[0]
                    if cell in admitted:
                        found = (cell, probe)
                        break
            if found:
                break
        if found:
            break
    assert found, "a tier-1 goal admitted at yaw 0 but colliding at 0.149 rad must exist"
