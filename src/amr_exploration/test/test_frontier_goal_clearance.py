"""Run-12 fixes: honest completion with an unsafe start pose, and one goal
clearance rule with an arrival margin (root-authored; implementer must not edit).

Run 12 (.ros_logs/hospital_explore_20261008_12): after goal 3 the robot's own
footprint overlapped a 254 cell, the route proof returned nothing, 1037 safe
endpoints were reported BLOCKED_ROUTE and the explorer declared COMPLETE on a
partly explored map.  Diagnosis confirmed independently (Codex Sol, high).
"""
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
import test_frontier_lifecycle as lifecycle  # noqa: E402

CIRCUMSCRIBED = max(math.hypot(x, y) for x, y in fa.NAVIGATION_FOOTPRINT)


def _disk_clear_r(costmap, cell, radius, width, height, res):
    cx, cy = (cell[0] + 0.5) * res, (cell[1] + 0.5) * res
    if (cx - radius < -1e-9 or cy - radius < -1e-9
            or cx + radius > width * res + 1e-9 or cy + radius > height * res + 1e-9):
        return False
    for y in range(max(0, int((cy - radius) / res) - 1), min(height, int((cy + radius) / res) + 2)):
        for x in range(max(0, int((cx - radius) / res) - 1), min(width, int((cx + radius) / res) + 2)):
            nx = min(max(cx, x * res), (x + 1) * res)
            ny = min(max(cy, y * res), (y + 1) * res)
            if math.hypot(nx - cx, ny - cy) < radius - 1e-9 and costmap.data[y * width + x] >= 254:
                return False
    return True


# ------------------------------------------------ explorer constants / wiring

def test_goal_clearance_radius_includes_an_arrival_margin():
    margin = fe.GOAL_ARRIVAL_MARGIN_M
    assert 0.07 <= margin <= 0.2      # >= controller xy_goal_tolerance (0.07 m)
    assert fe.GOAL_CLEARANCE_RADIUS_M == pytest.approx(CIRCUMSCRIBED + margin)


def test_default_selector_and_patrol_are_unchanged_without_the_new_argument():
    grid, costmap = approach._scene(raytrace_cleared=True)
    clusters = fa.frontier_clusters(approach.W, approach.H, grid.data)
    kwargs = dict(robot_world=approach.ROBOT, min_goal_distance=0.3,
                  footprint=fa.NAVIGATION_FOOTPRINT, robot_yaw=0.0,
                  require_path_clear=True, return_diagnostics=True,
                  approach_radius=1.5)
    assert fa.costmap_frontier_candidates(clusters, grid, costmap, (), **kwargs) == \
        fa.costmap_frontier_candidates(clusters, grid, costmap, (),
                                       goal_clearance_radius=None, **kwargs)


# --------------------------------------------------- uniform clearance gate

def _walled_scene():
    """Unknown band on the right (frontier cells not directly safe, so every
    goal is an approach goal) with a lethal wall row 0.80 m below mid-height."""
    walls = {(x, 12) for x in range(30, approach.UNKNOWN_FROM_X)}
    return approach._scene(walls=walls)


def _select(grid, costmap, radius, approach_radius=1.5):
    clusters = fa.frontier_clusters(approach.W, approach.H, grid.data)
    return fa.costmap_frontier_candidates(
        clusters, grid, costmap, (), robot_world=approach.ROBOT,
        min_goal_distance=0.3, footprint=fa.NAVIGATION_FOOTPRINT, robot_yaw=0.0,
        require_path_clear=False, return_diagnostics=True,
        approach_radius=approach_radius, goal_clearance_radius=radius)


def test_every_approach_goal_keeps_the_clearance_disk():
    grid, costmap = _walled_scene()
    radius = CIRCUMSCRIBED + 0.12
    candidates, _diagnostics = _select(grid, costmap, radius)
    assert candidates, "open approach space must still yield goals"
    for cell in candidates:
        assert _disk_clear_r(costmap, cell, radius, approach.W, approach.H,
                             approach.RES), cell


def test_approach_goal_inside_the_margin_is_rejected_but_was_admitted_before():
    """Obstacle between the circumscribed radius and radius+margin of the
    approach goal the circumscribed-only gate picks."""
    grid, costmap = approach._scene()
    picked = _select(grid, costmap, None)[0][0]
    lethal = (picked[0] - 8, picked[1])     # 0.80 m straight left of centre
    data = list(costmap.data)
    data[lethal[1] * approach.W + lethal[0]] = 254
    costmap.data = data
    assert picked in _select(grid, costmap, None)[0]
    assert picked not in _select(grid, costmap, CIRCUMSCRIBED + 0.12)[0]


def test_frontier_cell_goals_are_not_affected_by_the_clearance_radius():
    """Decision B (user, 2026-10-08): frontier-cell goals keep their existing
    yaw-0 footprint check and route arrival-turn proof; the margin applies to
    approach and patrol goals only."""
    grid, costmap = approach._scene(raytrace_cleared=True)
    assert _select(grid, costmap, None, approach_radius=0.0) == \
        _select(grid, costmap, CIRCUMSCRIBED + 0.12, approach_radius=0.0)


def test_patrol_uses_the_clearance_radius():
    grid, costmap = approach._single_lethal_scene((30, 30))
    radius = CIRCUMSCRIBED + 0.12
    result = fa.patrol_candidates(
        grid, costmap, (0.95, 0.95), footprint=fa.NAVIGATION_FOOTPRINT,
        min_goal_distance=0.0, sample_spacing=0.1, goal_clearance_radius=radius)
    assert result
    assert all(_disk_clear_r(costmap, c, radius, 60, 60, approach.RES) for c in result)


@pytest.mark.parametrize("value", [-0.1, 0.0, float("nan"), float("inf"), True, "0.8"])
def test_invalid_goal_clearance_radius_fails_closed(value):
    grid, costmap = approach._scene()
    clusters = fa.frontier_clusters(approach.W, approach.H, grid.data)
    assert fa.costmap_frontier_candidates(
        clusters, grid, costmap, (), robot_world=approach.ROBOT,
        min_goal_distance=0.3, footprint=fa.NAVIGATION_FOOTPRINT, robot_yaw=0.0,
        require_path_clear=True, return_diagnostics=True,
        approach_radius=1.5, goal_clearance_radius=value) is None


# ------------------------------------------- honest completion (unsafe start)

def _node_with_fixture(name):
    grid, costmap, pose = pf.hospital(name)
    node = lifecycle._node(autostart=True)
    node.latest_map = lifecycle._map_message(
        data=grid.data, width=grid.info.width, height=grid.info.height,
        resolution=grid.info.resolution, origin_x=grid.info.origin.position.x,
        origin_y=grid.info.origin.position.y)
    node.latest_costmap = lifecycle._costmap_message(
        data=costmap.data, width=costmap.metadata.size_x,
        height=costmap.metadata.size_y, resolution=costmap.metadata.resolution,
        origin_x=costmap.metadata.origin.position.x,
        origin_y=costmap.metadata.origin.position.y)
    node.map_version += 1
    node.costmap_version += 1
    # Hospital-size selection takes real time; freeze the explorer's steady
    # clock so authority/map freshness does not expire inside one decision.
    frozen = node._monotonic()
    node._monotonic = lambda: frozen
    return node, pose


def _transform(x, y, yaw):
    transform = lifecycle._transform(x=x, y=y)
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def _plan(node, pose):
    now = node._monotonic()
    node.last_map_at = now
    node.last_costmap_at = now
    node.last_base_status_at = now
    node.last_manipulator_status_at = now
    node.state = "PLANNING"
    node.map_version += 1
    node._select_frontier(
        node.run_generation,
        _transform(pose[0], pose[1], pose[2]))


def test_unsafe_start_with_safe_frontiers_ends_incomplete_not_complete():
    node, pose = _node_with_fixture("hospital_12_unsafe_start")
    for _ in range(node.no_frontier_limit):
        _plan(node, pose)
        if node.state != "SCANNING":
            break
    assert node.action_client.send_calls == []
    assert node.state == "INCOMPLETE", (node.state, node.reason)
    assert "robot pose is not collision-free" in node.reason
    assert node.fault_latched is False
    assert node._motion_owned is False


def test_valid_start_in_the_same_map_still_dispatches():
    """Positive control: the spawn pose in the same map is clear."""
    node, _pose = _node_with_fixture("hospital_12_unsafe_start")
    for _ in range(node.no_frontier_limit):
        _plan(node, (0.0, 0.0, 0.0))
        if node.action_client.send_calls:
            break
    assert node.state == "GOAL_PENDING", (node.state, node.reason)


def test_dispatched_goal_world_is_reported_in_status():
    node, _pose = _node_with_fixture("hospital_12_unsafe_start")
    for _ in range(node.no_frontier_limit):
        _plan(node, (0.0, 0.0, 0.0))
        if node.action_client.send_calls:
            break
    position = node.action_client.send_calls[-1].pose.pose.position
    with node._lock:
        node._publish_status_locked()
    values = lifecycle._diagnostic_values(node)
    gx, gy = (float(v) for v in values["goal_world"].split(","))
    assert (gx, gy) == pytest.approx((position.x, position.y), abs=1e-3)
