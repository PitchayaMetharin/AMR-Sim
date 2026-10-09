"""Planning speed-up acceptance tests (root-authored; implementer must not edit).

Goldens in fixtures/planning_goldens_post_gatefix.json were produced by the
selector *after* the turn-gate fix and *before* any optimisation.  The
optimisation must keep the exhaustive route proof and approach endpoints
byte-identical and keep completion (no reachable frontier) identical.

Goal order: "nearest reachable first" was tried (2026-10-08) and REVERTED by
user decision after hospital run 13: it made the selector fast but the robot
crept in ~0.5 m steps inside one 3.5 x 8.5 m patch and never moved to a new
area (350 m^2 vs 751 m^2 in run 10).  The selector keeps the original priority
(largest frontier cluster first).  Honest measured cost of that order with the
exact speed-ups: hospital_04 ~1.05 s, hospital_10 ~3.8 s (was 3.7 s / 14.0 s);
bounds below are measured x1.5.
"""
import json
import math
from pathlib import Path
import sys
import time

import pytest

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR.parent / "scripts"))
sys.path.insert(0, str(TEST_DIR))

import frontier_algorithm as fa  # noqa: E402
import planning_fixtures as pf  # noqa: E402
import test_frontier_approach as approach  # noqa: E402

GOLDENS = json.loads(
    (TEST_DIR / "fixtures" / "planning_goldens_post_gatefix.json").read_text())
CASES = [f"synthetic_{seed}" for seed in range(8)] + ["hospital_04", "hospital_10"]


def _case(name):
    if name.startswith("synthetic_"):
        return pf.synthetic(int(name.split("_")[1]))
    return pf.hospital(name)


def _jsonable(value):
    return json.loads(json.dumps(value))


def _select(grid, costmap, pose, require_path_clear):
    clusters = fa.frontier_clusters(grid.info.width, grid.info.height, grid.data)
    return clusters, fa.costmap_frontier_candidates(
        clusters, grid, costmap, (), robot_world=pose[:2], min_goal_distance=0.3,
        footprint=fa.NAVIGATION_FOOTPRINT, robot_yaw=pose[2],
        require_path_clear=require_path_clear, return_diagnostics=True,
        approach_radius=1.5)


@pytest.mark.parametrize("name", CASES)
def test_exhaustive_route_proof_is_unchanged(name):
    grid, costmap, pose = _case(name)
    geometry = fa.costmap_geometry(costmap)
    width, height = geometry[0], geometry[1]
    stride = GOLDENS[name]["targets_stride"]
    targets = [(x, y) for y in range(0, height, stride)
               for x in range(0, width, stride)
               if costmap.data[y * width + x] < 253]
    reachable = fa._reachable_costmap_cells(
        geometry, costmap.data, pose[:2], pose[2], targets,
        fa._strict_footprint(fa.NAVIGATION_FOOTPRINT), stop_after_first=False)
    assert sorted(map(list, reachable)) == GOLDENS[name]["reachable"]


@pytest.mark.parametrize("name", CASES)
def test_approach_and_safety_classification_is_unchanged(name):
    grid, costmap, pose = _case(name)
    _clusters, result = _select(grid, costmap, pose, require_path_clear=False)
    assert _jsonable(list(result)) == GOLDENS[name]["no_path"]


@pytest.mark.parametrize("name", CASES)
def test_selector_choice_is_safe_reachable_or_completion_is_unchanged(name):
    grid, costmap, pose = _case(name)
    _clusters, (candidates, diagnostics) = _select(
        grid, costmap, pose, require_path_clear=True)
    golden_candidates, golden_diagnostics = GOLDENS[name]["selector"]
    if not golden_candidates:
        # Exhaustive search: completion evidence must be identical.
        assert candidates == []
        assert _jsonable(diagnostics) == golden_diagnostics
        return
    assert candidates
    cell = candidates[0]          # the dispatched goal
    geometry = fa.costmap_geometry(costmap)
    world = fa.frontier_cell_world(grid, cell)
    target = fa._costmap_cell(geometry, world)
    assert grid.data[cell[1] * grid.info.width + cell[0]] == 0
    assert costmap.data[target[1] * geometry[0] + target[0]] < 253
    assert fa._footprint_costmap_clear(
        geometry, costmap.data, world, fa.NAVIGATION_FOOTPRINT)
    assert target in fa._reachable_costmap_cells(
        geometry, costmap.data, pose[:2], pose[2], [target],
        fa._strict_footprint(fa.NAVIGATION_FOOTPRINT), stop_after_first=False)
    assert [d["classification"] for d in diagnostics].count("REACHABLE") >= 1


def test_selector_keeps_largest_frontier_priority():
    """Large unknown band far right (big cluster), small unknown pocket near
    the robot (small cluster): both reachable; the large cluster wins."""
    grid, costmap = approach._scene()
    width = approach.W
    data, cost = list(grid.data), list(costmap.data)
    for x in range(14, 18):
        for y in range(30, 34):
            data[y * width + x] = -1
            cost[y * width + x] = 255
    grid.data, costmap.data = data, cost
    clusters = fa.frontier_clusters(width, approach.H, grid.data)
    assert len(clusters) == 2
    assert len(clusters[0][1]) > len(clusters[1][1])   # far band is cluster 0
    candidates, _ = fa.costmap_frontier_candidates(
        clusters, grid, costmap, (), robot_world=approach.ROBOT,
        min_goal_distance=0.3, footprint=fa.NAVIGATION_FOOTPRINT, robot_yaw=0.0,
        require_path_clear=True, return_diagnostics=True,
        approach_radius=approach.RADIUS)
    assert candidates
    chosen = approach._world(candidates[0])
    pocket = (1.6, 3.2)
    band = (5.0, chosen[1])
    assert math.dist(chosen, band) < math.dist(chosen, pocket), chosen


@pytest.mark.parametrize("name, bound_s", [("hospital_04", 1.6), ("hospital_10", 5.7)])
def test_selector_is_fast_on_hospital_maps(name, bound_s):
    grid, costmap, pose = _case(name)
    start = time.monotonic()
    _select(grid, costmap, pose, require_path_clear=True)
    elapsed = time.monotonic() - start
    assert elapsed <= bound_s, f"{name}: {elapsed:.2f} s > {bound_s} s"
