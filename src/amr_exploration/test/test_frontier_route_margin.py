"""Acceptance tests for the exploration route-footprint margin.

Root-authored before implementation; the implementer must not edit this file.

AWS runs 02/05 (and the passing run 07) drove the robot into aisles roughly
one footprint wide.  The explorer's route proof used the bare navigation
footprint, so a route that cleared lethal cells by a few centimetres was
admitted, and the rotating 1.22 x 0.82 m body then clipped a wall (obstacle
contact in run 05).  Replaying every recorded plan with the footprint grown by
a margin separated blockages/contacts from good plans at 0.08 m
(`.ros_logs/claude_resume_20261009/replay_scripts/margin_sweep.py`).

Contract:
* ``NAVIGATION_FOOTPRINT`` (the Nav2 footprint plus padding) is unchanged.
* The explorer module exposes ``ROUTE_FOOTPRINT_MARGIN_M == 0.08`` and
  ``EXPLORATION_ROUTE_FOOTPRINT``: every vertex of ``NAVIGATION_FOOTPRINT``
  moved outward by exactly the margin on each axis.
* Frontier route search and patrol search use ``EXPLORATION_ROUTE_FOOTPRINT``.
  The start-pose proofs keep ``NAVIGATION_FOOTPRINT`` (observation of where the
  robot actually is must not change).
* A door that fits the bare footprint but not the margin footprint is no longer
  an admissible route; a door that fits both stays admissible.  (The node
  adds its own gates, so the narrow-door effect is pinned at the selector
  level and the call sites are pinned by an AST check.)
"""
import ast
import math
from pathlib import Path
import sys

import pytest

TEST_DIR = Path(__file__).resolve().parent
ROOT = TEST_DIR.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(TEST_DIR))

import frontier_explorer as frontier_explorer_module  # noqa: E402
from frontier_algorithm import (  # noqa: E402
    NAVIGATION_FOOTPRINT, costmap_frontier_candidates, frontier_clusters)
import test_frontier_approach as approach  # noqa: E402
import test_frontier_lifecycle as lifecycle  # noqa: E402

W, H, RES, ROBOT = approach.W, approach.H, approach.RES, approach.ROBOT
WALL_COLUMNS = range(25, 35)      # a 1.0 m thick wall between robot and frontier
DOOR_ROW_START = 15


def _door(width_cells):
    """Wall with one door, centred on row 19.5 (y = 1.95 m) for width 9."""
    return approach._scene(
        wall_columns=WALL_COLUMNS,
        slot_rows=range(DOOR_ROW_START, DOOR_ROW_START + width_cells))


def _footprint_with_margin(margin):
    return tuple(
        (x + math.copysign(margin, x), y + math.copysign(margin, y))
        for x, y in NAVIGATION_FOOTPRINT)


def _admitted(grid, costmap, footprint):
    clusters = frontier_clusters(W, H, grid.data)
    assert clusters
    result = costmap_frontier_candidates(
        clusters, grid, costmap, (), robot_world=ROBOT,
        min_goal_distance=0.3, footprint=footprint, robot_yaw=0.0,
        require_path_clear=True, return_diagnostics=True,
        approach_radius=frontier_explorer_module.FRONTIER_APPROACH_RADIUS_M)
    assert result is not None
    return bool(result[0])


def _node_dispatches(grid, costmap):
    node = lifecycle._node(autostart=True)
    node.latest_map = grid
    node.latest_costmap = costmap
    node.map_version += 1
    node.costmap_version += 1
    for _ in range(node.no_frontier_limit):
        approach._plan(node)
        if node.action_client.send_calls:
            break
    return bool(node.action_client.send_calls)


def test_navigation_footprint_is_unchanged():
    assert NAVIGATION_FOOTPRINT == (
        (0.61, 0.41), (0.61, -0.41), (-0.61, -0.41), (-0.61, 0.41))


def test_route_margin_constants_are_exact():
    assert frontier_explorer_module.ROUTE_FOOTPRINT_MARGIN_M == pytest.approx(0.08)
    expected = _footprint_with_margin(0.08)
    actual = frontier_explorer_module.EXPLORATION_ROUTE_FOOTPRINT
    assert len(actual) == len(expected)
    for got, want in zip(actual, expected):
        assert got == pytest.approx(want)


def test_control_door_fits_the_bare_footprint_but_not_the_margin_footprint():
    # 0.9 m door: bare footprint is 0.82 m wide, margin footprint is 0.98 m.
    grid, costmap = _door(9)
    assert _admitted(grid, costmap, NAVIGATION_FOOTPRINT)
    assert not _admitted(
        grid, costmap, frontier_explorer_module.EXPLORATION_ROUTE_FOOTPRINT)


def test_wide_door_stays_admissible_with_the_margin_footprint():
    grid, costmap = _door(16)           # 1.6 m door
    assert _admitted(grid, costmap, NAVIGATION_FOOTPRINT)
    assert _admitted(
        grid, costmap, frontier_explorer_module.EXPLORATION_ROUTE_FOOTPRINT)


def test_explorer_still_dispatches_through_a_wide_door():
    # Regression guard only (passes before and after): the node adds its own
    # gates, so it already refuses doors below ~1.6 m in this scene; the
    # margin must not take away a comfortably wide door.
    grid, costmap = _door(16)
    assert _node_dispatches(grid, costmap) is True


def _calls(tree, name):
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, "id", getattr(node.func, "attr", None)) == name):
            yield node


def _footprint_argument(call):
    for keyword in call.keywords:
        if keyword.arg == "footprint":
            return keyword.value.id
    return [arg.id for arg in call.args if isinstance(arg, ast.Name)]


def test_search_calls_use_the_margin_footprint_and_start_proofs_do_not():
    tree = ast.parse((ROOT / "scripts" / "frontier_explorer.py").read_text())
    searches = (list(_calls(tree, "costmap_frontier_candidates"))
                + list(_calls(tree, "patrol_candidates")))
    assert len(searches) == 2
    for call in searches:
        assert _footprint_argument(call) == "EXPLORATION_ROUTE_FOOTPRINT"
    proofs = list(_calls(tree, "_route_start_proof"))
    assert len(proofs) == 2
    for call in proofs:
        names = _footprint_argument(call)
        assert "NAVIGATION_FOOTPRINT" in names
        assert "EXPLORATION_ROUTE_FOOTPRINT" not in names
