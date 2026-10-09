"""Acceptance tests for frontier approach goals.

Root-authored before implementation; the implementer must not edit this file.

A frontier cell borders unknown space, so the robot footprint centred on it
overlaps unknown (cost 255) cells and fails the lethal-footprint gate unless
the costmap happened to raytrace-clear that area.  The selector may instead
pick an "approach" goal: a known-free map cell within ``approach_radius`` of
the frontier whose footprint is clear of cost >= 254 (lethal and unknown) and
which passes the unchanged route proof.  ``approach_radius`` defaults to 0,
which must preserve the historical selector exactly.

Approach goals must also be leavable: the footprint's circumscribed disk at
the goal lies inside the costmap and touches no cost >= 254 cell, so the
robot can turn in place there (hospital run 05 parked against a wall and
every departure path was rejected by the smoother).  Consecutive confirmed
blockages without a reached goal must end the run INCOMPLETE instead of
looping forever.
"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

TEST_DIR = Path(__file__).resolve().parent
ROOT = TEST_DIR.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(TEST_DIR))

import frontier_explorer as frontier_explorer_module  # noqa: E402
import frontier_algorithm as fa  # noqa: E402
from frontier_algorithm import (  # noqa: E402
    NAVIGATION_FOOTPRINT, costmap_frontier_candidates, frontier_clusters)
from frontier_algorithm import patrol_candidates as patrol_candidates_fn  # noqa: E402
import test_frontier_lifecycle as lifecycle  # noqa: E402

RES = 0.1
W, H = 60, 40              # 6.0 m x 4.0 m
UNKNOWN_FROM_X = 50        # columns >= 50 are unknown
ROBOT = (1.05, 2.05)
RADIUS = 1.5


def _scene(walls=(), slot_rows=None, wall_columns=None, raytrace_cleared=False):
    """Known-free room with unknown space on the right.

    ``walls``: cells that are occupied (map 100, costmap 254).
    ``wall_columns`` + ``slot_rows``: occupied band of columns except the
    slot rows.
    ``raytrace_cleared``: costmap marks unknown cells 0 instead of 255.
    """
    grid = [0] * (W * H)
    cost = [0] * (W * H)
    for y in range(H):
        for x in range(UNKNOWN_FROM_X, W):
            grid[y * W + x] = -1
            cost[y * W + x] = 0 if raytrace_cleared else 255
    occupied = set(walls)
    if wall_columns is not None:
        for x in wall_columns:
            for y in range(H):
                if slot_rows is None or y not in slot_rows:
                    occupied.add((x, y))
    for x, y in occupied:
        grid[y * W + x] = 100
        cost[y * W + x] = 254
    return (
        lifecycle._map_message(data=grid, width=W, height=H, resolution=RES),
        lifecycle._costmap_message(
            data=cost, width=W, height=H, resolution=RES,
            origin_x=0.0, origin_y=0.0))


def _select(grid, costmap, **kwargs):
    clusters = frontier_clusters(W, H, grid.data)
    assert clusters
    result = costmap_frontier_candidates(
        clusters, grid, costmap, (), robot_world=ROBOT,
        min_goal_distance=0.3, footprint=NAVIGATION_FOOTPRINT,
        robot_yaw=0.0, require_path_clear=True, return_diagnostics=True,
        **kwargs)
    assert result is not None
    candidates, diagnostics = result
    return clusters, candidates, diagnostics


def _world(cell):
    return ((cell[0] + 0.5) * RES, (cell[1] + 0.5) * RES)


def _frontier_distance(cell, clusters):
    world = _world(cell)
    return min(
        math.dist(world, _world(frontier))
        for _rep, cells in clusters for frontier in cells)


def _footprint_clear(costmap, cell):
    return fa._footprint_costmap_clear(
        fa.costmap_geometry(costmap), costmap.data, _world(cell),
        NAVIGATION_FOOTPRINT)


CIRCUMSCRIBED = max(math.hypot(x, y) for x, y in NAVIGATION_FOOTPRINT)


def _disk_clear(costmap, cell, width=W, height=H):
    """Circumscribed footprint disk inside the costmap and clear of >= 254."""
    cx, cy = _world(cell)
    r = CIRCUMSCRIBED
    if (cx - r < -1e-9 or cy - r < -1e-9
            or cx + r > width * RES + 1e-9 or cy + r > height * RES + 1e-9):
        return False
    for y in range(max(0, int((cy - r) / RES) - 1),
                   min(height, int((cy + r) / RES) + 2)):
        for x in range(max(0, int((cx - r) / RES) - 1),
                       min(width, int((cx + r) / RES) + 2)):
            nx = min(max(cx, x * RES), (x + 1) * RES)
            ny = min(max(cy, y * RES), (y + 1) * RES)
            if (math.hypot(nx - cx, ny - cy) < r - 1e-9
                    and costmap.data[y * width + x] >= 254):
                return False
    return True


def _turn_clear(costmap, cell):
    return all(
        fa._footprint_costmap_clear(
            fa.costmap_geometry(costmap), costmap.data, _world(cell),
            NAVIGATION_FOOTPRINT, yaw=k * math.pi / 16.0)
        for k in range(32))


def _assert_safe_approach(grid, costmap, clusters, cell):
    assert grid.data[cell[1] * W + cell[0]] == 0, "goal must be known free"
    assert costmap.data[cell[1] * W + cell[0]] < 253
    assert _footprint_clear(costmap, cell), "footprint overlaps cost >= 254"
    assert _turn_clear(costmap, cell), "robot cannot turn in place at goal"
    assert _frontier_distance(cell, clusters) <= RADIUS + 1.0e-9
    assert math.dist(_world(cell), ROBOT) >= 0.3


# ------------------------------------------------------------- algorithm

def test_default_selector_still_blocks_frontiers_bordering_unknown():
    """Baseline (documents the hospital/AWS failure; must stay unchanged)."""
    _clusters, candidates, diagnostics = _select(*_scene())
    assert candidates == []
    assert [d["classification"] for d in diagnostics] == ["BLOCKED_SAFETY"]


def test_approach_radius_zero_is_identical_to_the_default():
    grid, costmap = _scene()
    default = _select(grid, costmap)
    explicit = _select(grid, costmap, approach_radius=0.0)
    assert default[1:] == explicit[1:]


def test_approach_goal_is_known_free_footprint_clear_and_near_the_frontier():
    grid, costmap = _scene()
    clusters, candidates, diagnostics = _select(
        grid, costmap, approach_radius=RADIUS)
    assert candidates, "an approach goal must be found"
    assert [d["classification"] for d in diagnostics] == ["REACHABLE"]
    assert diagnostics[0]["safe_endpoint_count"] >= 1
    assert diagnostics[0]["reachable_endpoint_count"] >= 1
    for cell in candidates:
        _assert_safe_approach(grid, costmap, clusters, cell)


def test_approach_goal_is_as_close_to_the_frontier_as_safely_possible():
    grid, costmap = _scene()
    clusters, candidates, _diagnostics = _select(
        grid, costmap, approach_radius=RADIUS)
    best = min(
        _frontier_distance((x, y), clusters)
        for y in range(H) for x in range(UNKNOWN_FROM_X)
        if grid.data[y * W + x] == 0
        and math.dist(_world((x, y)), ROBOT) >= 0.3
        and _disk_clear(costmap, (x, y)))
    assert best <= RADIUS
    # Implementations may sample approach cells on a stride of up to two
    # cells; the chosen goal must still be within that of the optimum.
    assert _frontier_distance(candidates[0], clusters) <= best + 2 * RES + 1e-9


def test_approach_is_deterministic():
    grid, costmap = _scene()
    first = _select(grid, costmap, approach_radius=RADIUS)
    second = _select(grid, costmap, approach_radius=RADIUS)
    assert first[1:] == second[1:]


def test_safe_frontier_cells_keep_priority_over_approach_cells():
    grid, costmap = _scene(raytrace_cleared=True)
    default = _select(grid, costmap)
    with_approach = _select(grid, costmap, approach_radius=RADIUS)
    assert default[1], "raytrace-cleared frontier cells are directly safe"
    assert with_approach[1][0] == default[1][0]


def test_narrow_slot_frontier_stays_blocked_on_safety():
    """A 0.6 m slot is narrower than the 0.82 m footprint; walls at x 35..49."""
    grid, costmap = _scene(
        wall_columns=range(35, UNKNOWN_FROM_X), slot_rows=range(18, 24))
    clusters, candidates, diagnostics = _select(
        grid, costmap, approach_radius=RADIUS)
    assert candidates == []
    assert diagnostics and all(
        d["classification"] == "BLOCKED_SAFETY" for d in diagnostics)


def test_approach_goal_behind_a_lethal_wall_is_blocked_on_route():
    """Wall at x=20 (full height) separates the robot from the frontier."""
    grid, costmap = _scene(walls={(20, y) for y in range(H)})
    _clusters, candidates, diagnostics = _select(
        grid, costmap, approach_radius=RADIUS)
    assert candidates == []
    assert [d["classification"] for d in diagnostics] == ["BLOCKED_ROUTE"]


def test_avoid_worlds_exclude_nearby_endpoints():
    grid, costmap = _scene()
    clusters, first, _ = _select(grid, costmap, approach_radius=RADIUS)
    avoided = _world(first[0])
    _, second, diagnostics = _select(
        grid, costmap, approach_radius=RADIUS,
        avoid_worlds=(avoided,), avoid_radius=0.5)
    assert second, "other approach goals remain outside the avoid radius"
    for cell in second:
        assert math.dist(_world(cell), avoided) >= 0.5
        _assert_safe_approach(grid, costmap, clusters, cell)
    assert [d["classification"] for d in diagnostics] == ["REACHABLE"]


def test_avoid_worlds_also_exclude_frontier_cell_endpoints():
    grid, costmap = _scene(raytrace_cleared=True)
    _, first, _ = _select(grid, costmap)
    avoided = _world(first[0])
    _, second, _ = _select(
        grid, costmap, avoid_worlds=(avoided,), avoid_radius=0.5)
    assert second
    assert all(math.dist(_world(cell), avoided) >= 0.5 for cell in second)


def test_avoiding_everything_blocks_on_safety():
    grid, costmap = _scene()
    _, candidates, diagnostics = _select(
        grid, costmap, approach_radius=RADIUS,
        avoid_worlds=((3.0, 2.0),), avoid_radius=100.0)
    assert candidates == []
    assert [d["classification"] for d in diagnostics] == ["BLOCKED_SAFETY"]


@pytest.mark.parametrize("kwargs", [
    {"approach_radius": -0.1},
    {"approach_radius": float("nan")},
    {"approach_radius": float("inf")},
    {"approach_radius": True},
    {"avoid_radius": -1.0},
    {"avoid_radius": float("nan")},
    {"avoid_worlds": ((float("nan"), 0.0),)},
    {"avoid_worlds": ((1.0,),)},
    {"avoid_worlds": None},
])
def test_invalid_approach_arguments_fail_closed(kwargs):
    grid, costmap = _scene()
    clusters = frontier_clusters(W, H, grid.data)
    assert costmap_frontier_candidates(
        clusters, grid, costmap, (), robot_world=ROBOT,
        min_goal_distance=0.3, footprint=NAVIGATION_FOOTPRINT,
        robot_yaw=0.0, require_path_clear=True, return_diagnostics=True,
        **kwargs) is None


def test_approach_goal_keeps_turn_in_place_clearance_from_walls():
    """Lethal band along the bottom 1 m: the closest-to-frontier cells hug it."""
    walls = {(x, y) for x in range(UNKNOWN_FROM_X) for y in range(10)}
    grid, costmap = _scene(walls=walls)
    clusters, candidates, diagnostics = _select(
        grid, costmap, approach_radius=RADIUS)
    assert candidates
    assert [d["classification"] for d in diagnostics] == ["REACHABLE"]
    for cell in candidates:
        _assert_safe_approach(grid, costmap, clusters, cell)
        assert _disk_clear(costmap, cell)


def test_approach_rejects_a_pocket_too_narrow_to_turn_in():
    """A 1.0 m wide corridor fits the 0.82 m footprint but not a 1.47 m turn."""
    grid, costmap = _scene(
        wall_columns=range(30, UNKNOWN_FROM_X), slot_rows=range(15, 25))
    _clusters, candidates, diagnostics = _select(
        grid, costmap, approach_radius=RADIUS)
    assert candidates == []
    assert diagnostics and all(
        d["classification"] == "BLOCKED_SAFETY" for d in diagnostics)


# -------------------------------------------------------------- explorer

def _node():
    node = lifecycle._node(autostart=True)
    grid, costmap = _scene()
    node.latest_map = grid
    node.latest_costmap = costmap
    node.map_version += 1
    node.costmap_version += 1
    return node, grid, costmap


def _plan(node):
    now = node._monotonic()
    node.last_map_at = now
    node.last_costmap_at = now
    node.last_base_status_at = now
    node.last_manipulator_status_at = now
    node.state = "PLANNING"
    node.map_version += 1
    node._select_frontier(
        node.run_generation, lifecycle._transform(x=ROBOT[0], y=ROBOT[1]))


def _goal_cell(node, index=-1):
    position = node.action_client.send_calls[index].pose.pose.position
    cell = (math.floor(position.x / RES), math.floor(position.y / RES))
    assert (position.x, position.y) == pytest.approx(_world(cell))
    return cell


def _succeed(node):
    handle = lifecycle._accept(node)
    handle.result_future.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))


def test_explorer_constants_are_positive_and_bounded():
    radius = frontier_explorer_module.FRONTIER_APPROACH_RADIUS_M
    avoid = frontier_explorer_module.FRONTIER_APPROACH_AVOID_RADIUS_M
    assert 1.0 <= radius <= 2.0
    assert 0.2 <= avoid <= radius


def test_explorer_dispatches_a_safe_approach_goal_instead_of_completing():
    node, grid, costmap = _node()
    clusters = frontier_clusters(W, H, grid.data)
    for _ in range(node.no_frontier_limit):
        _plan(node)
        if node.action_client.send_calls:
            break
    assert node.state == "GOAL_PENDING"
    assert node.fault_latched is False
    cell = _goal_cell(node)
    assert grid.data[cell[1] * W + cell[0]] == 0
    assert _footprint_clear(costmap, cell)
    assert _turn_clear(costmap, cell)
    assert _frontier_distance(cell, clusters) <= (
        frontier_explorer_module.FRONTIER_APPROACH_RADIUS_M + 1e-9)


def test_explorer_does_not_revisit_reached_goals_and_terminates():
    """A sensor-blind frontier never resolves; the explorer must not loop."""
    node, _grid, _costmap = _node()
    avoid = frontier_explorer_module.FRONTIER_APPROACH_AVOID_RADIUS_M
    reached = []
    for _ in range(400):
        if node.state in ("COMPLETE", "INCOMPLETE", "FAULT"):
            break
        before = len(node.action_client.send_calls)
        _plan(node)
        if len(node.action_client.send_calls) > before:
            cell = _goal_cell(node)
            for old in reached:
                assert math.dist(_world(cell), _world(old)) >= avoid - 1e-9
            reached.append(cell)
            _succeed(node)
    assert node.state == "COMPLETE", node.reason
    assert node.fault_latched is False
    assert 1 <= len(reached) <= 60


def test_stop_and_restart_clear_reached_goal_history():
    node, _grid, _costmap = _node()
    for _ in range(node.no_frontier_limit):
        _plan(node)
        if node.action_client.send_calls:
            break
    first = _goal_cell(node)
    _succeed(node)
    response = node._stop_callback(None, Trigger.Response())
    assert response.success is True
    assert node.state == "STOPPED"
    response = lifecycle._start(node)
    assert response.success is True
    node.action_client.send_calls.clear()
    for _ in range(node.no_frontier_limit):
        _plan(node)
        if node.action_client.send_calls:
            break
    assert _goal_cell(node) == first


def _blockage_cycle(node, goal_world):
    node.state = "PLANNING"
    now = node._monotonic()
    node.last_map_at = now
    node.last_costmap_at = now
    node.last_base_status_at = now
    node.last_manipulator_status_at = now
    before = len(node.action_client.send_calls)
    node._reserve_and_send(
        object(), (14, 10), node.run_generation,
        map_snapshot=(node.latest_map, node.map_version, node.last_map_at),
        costmap_snapshot=(
            node.latest_costmap, node.costmap_version, node.last_costmap_at),
        transform=lifecycle._transform(), goal_world=goal_world)
    assert len(node.action_client.send_calls) == before + 1, node.reason
    return lifecycle._accept(node)


def _confirmed_blockage(node, goal_world):
    handle = _blockage_cycle(node, goal_world)
    node._mission_status_callback(
        lifecycle._mission_status(node._expected_goal_uuid))
    handle.result_future.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_ABORTED))


def _reached(node, goal_world):
    handle = _blockage_cycle(node, goal_world)
    handle.result_future.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))


def test_consecutive_blockages_without_progress_end_incomplete():
    """Hospital runs 03/05: the goal shifted a few cm each cycle, so per-goal
    blockage history never exhausted and the explorer looped indefinitely."""
    limit = frontier_explorer_module.MAX_CONSECUTIVE_BLOCKAGES
    assert 3 <= limit <= 20
    node = lifecycle._selection_fixture(lifecycle._node(autostart=True))
    for i in range(limit):
        assert node.state not in ("COMPLETE", "INCOMPLETE", "FAULT")
        _confirmed_blockage(node, (14.5 + 0.1 * i, 10.5))
        if i < limit - 1:
            assert node.state == "RECOVERY_WAIT", (i, node.state, node.reason)
    assert node.state == "INCOMPLETE", node.reason
    assert "repeated obstacle blockage" in node.reason
    assert node.fault_latched is False
    assert node._has_motion_locked() is False


def test_a_reached_goal_resets_the_consecutive_blockage_count():
    limit = frontier_explorer_module.MAX_CONSECUTIVE_BLOCKAGES
    node = lifecycle._selection_fixture(lifecycle._node(autostart=True))
    for i in range(limit - 1):
        _confirmed_blockage(node, (14.5 + 0.1 * i, 10.5))
    _reached(node, (13.5, 9.5))
    for i in range(limit - 1):
        _confirmed_blockage(node, (14.5 + 0.1 * i, 11.5))
        assert node.state == "RECOVERY_WAIT", (i, node.state, node.reason)
    assert node.fault_latched is False


def test_restart_resets_the_consecutive_blockage_count():
    limit = frontier_explorer_module.MAX_CONSECUTIVE_BLOCKAGES
    node = lifecycle._selection_fixture(lifecycle._node(autostart=True))
    for i in range(limit - 1):
        _confirmed_blockage(node, (14.5 + 0.1 * i, 10.5))
    response = node._stop_callback(None, Trigger.Response())
    assert response.success is True
    response = lifecycle._start(node)
    assert response.success is True
    for i in range(limit - 1):
        _confirmed_blockage(node, (14.5 + 0.1 * i, 11.5))
        assert node.state == "RECOVERY_WAIT", (i, node.state, node.reason)


# ------------------------------------------- turn-gate side symmetry (run 10)

def _single_lethal_scene(lethal, width=60, height=60):
    grid = lifecycle._map_message(
        data=[0] * (width * height), width=width, height=height, resolution=RES)
    cost = [0] * (width * height)
    cost[lethal[1] * width + lethal[0]] = 254
    costmap = lifecycle._costmap_message(
        data=cost, width=width, height=height, resolution=RES,
        origin_x=0.0, origin_y=0.0)
    return grid, costmap


def test_patrol_turn_gate_checks_every_side_of_the_disk():
    """Hospital run 10: a lethal cell 0.729 m from the goal centre on the
    goal's -x/+y side was missed, the robot parked with a corner on it and the
    smoother rejected every departure path from the start pose."""
    grid, costmap = _single_lethal_scene((30, 30))
    # Start pose keeps the whole footprint inside the costmap (the route
    # proof fails closed otherwise).
    result = patrol_candidates_fn(
        grid, costmap, (0.75, 0.75), footprint=NAVIGATION_FOOTPRINT,
        min_goal_distance=0.0, sample_spacing=0.1)
    assert result
    wrong = [cell for cell in result
             if not _disk_clear(costmap, cell, width=60, height=60)]
    assert wrong == []


@pytest.mark.parametrize("lethal", [
    (44, 20), (40, 20), (42, 13), (42, 27), (38, 16), (38, 24), (46, 14)])
def test_approach_turn_gate_checks_every_side_of_the_disk(lethal):
    grid, costmap = _scene(walls={lethal})
    clusters, candidates, _diagnostics = _select(
        grid, costmap, approach_radius=RADIUS)
    for cell in candidates:
        assert _disk_clear(costmap, cell), (lethal, cell)
    # The gate itself, over every candidate cell the selector could rank.
    geometry = fa.costmap_geometry(costmap)
    gate = fa._turn_clearance_checker(geometry, costmap.data, NAVIGATION_FOOTPRINT)
    for y in range(H):
        for x in range(UNKNOWN_FROM_X):
            assert gate(_world((x, y))) == _disk_clear(costmap, (x, y)), (lethal, x, y)


def test_turn_gate_matches_the_exact_disk_on_the_run10_geometry():
    """Cell offset (-13, +8) at 0.05 m resolution, 0.729 m from the centre."""
    width = height = 60
    res = 0.05
    centre = (30, 30)
    cost = [0] * (width * height)
    cost[(centre[1] + 8) * width + centre[0] - 13] = 254
    costmap = lifecycle._costmap_message(
        data=cost, width=width, height=height, resolution=res,
        origin_x=0.0, origin_y=0.0)
    gate = fa._turn_clearance_checker(
        fa.costmap_geometry(costmap), costmap.data, NAVIGATION_FOOTPRINT)
    world = ((centre[0] + 0.48) * res, (centre[1] + 0.32) * res)
    radius = CIRCUMSCRIBED
    lx, ly = world[0] / res, world[1] / res
    cx, cy = centre[0] - 13, centre[1] + 8
    nx, ny = min(max(lx, cx), cx + 1), min(max(ly, cy), cy + 1)
    assert math.hypot(nx - lx, ny - ly) * res < radius
    assert gate(world) is False
