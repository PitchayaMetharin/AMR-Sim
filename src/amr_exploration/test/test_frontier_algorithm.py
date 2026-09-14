import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import frontier_algorithm as frontier_algorithm_module  # noqa: E402
from frontier_algorithm import (  # noqa: E402
    NAVIGATION_FOOTPRINT, available_candidates, costmap_frontier_candidates,
    frontier_cell_world, frontier_clusters, frontier_world_cell,
    occupancy_grid_geometry)


def _grid(width, height, resolution=1.0, origin_x=0.0, origin_y=0.0,
          yaw=0.0):
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0)))),
        data=[0] * (width * height))


def _costmap(width, height, data, resolution=1.0, origin_x=0.0,
             origin_y=0.0, yaw=0.0, frame_id="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        metadata=SimpleNamespace(
            size_x=width,
            size_y=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0)))),
        data=data)


def test_frontier_clusters_are_free_and_deterministic():
    # One free cell beside unknown, with an occupied cell excluded.
    grid = [100, -1, -1, 0, 0, 100, 0, 0, 0]
    clusters = frontier_clusters(3, 3, grid)
    assert clusters
    candidate, cells = clusters[0]
    assert candidate in cells
    assert all(grid[y * 3 + x] == 0 for x, y in cells)
    assert candidate == (0, 1)


def test_blacklist_removes_failed_frontier_candidate():
    clusters = [((1, 2), ((1, 2),)), ((4, 2), ((4, 2),))]
    assert available_candidates(clusters, {(1, 2)}) == [(4, 2)]


def test_invalid_grid_has_no_frontiers():
    assert frontier_clusters(2, 2, [0]) is None


@pytest.mark.parametrize("value", [
    True, False, 0.0, 1.0, "0", None, math.nan, math.inf, -2, 101])
def test_frontier_clusters_rejects_malformed_or_out_of_domain_occupancy(value):
    assert frontier_clusters(2, 1, [0, value]) is None


@pytest.mark.parametrize("width,height", [
    (True, 1), (1, False), (1.0, 1), (1, 1.0), (0, 1), (1, -1),
    (math.inf, 1)])
def test_frontier_clusters_rejects_non_integral_or_non_positive_dimensions(
        width, height):
    assert frontier_clusters(width, height, [0]) is None


def test_occupancy_grid_geometry_requires_complete_planar_map_evidence():
    grid = _grid(2, 1)
    grid.header = SimpleNamespace(frame_id="map")

    assert occupancy_grid_geometry(grid) is not None

    grid.data = [0, 0.5]
    assert occupancy_grid_geometry(grid) is None

    grid.data = [0, -1]
    grid.info.origin.orientation.x = 0.1
    assert occupancy_grid_geometry(grid) is None

    grid.info.origin.orientation.x = 0.0
    grid.info.origin.orientation.w = 0.0
    assert occupancy_grid_geometry(grid) is None


def test_frontier_world_cell_applies_translated_origin():
    grid = _grid(3, 2, origin_x=10.0, origin_y=20.0)

    assert frontier_world_cell(grid, (11.5, 20.5)) == (1, 0)


def test_frontier_world_cell_applies_resolution_and_planar_yaw():
    grid = _grid(
        3, 2, resolution=2.0, origin_x=10.0, origin_y=20.0,
        yaw=math.pi / 2.0)

    world = frontier_cell_world(grid, (1, 0))
    assert world == pytest.approx((9.0, 23.0))
    assert frontier_world_cell(grid, world) == (1, 0)


def test_frontier_world_cell_preserves_out_of_bounds_world_failure_until_geometry_contains_it():
    failed_world = (1.5, 0.5)

    assert frontier_world_cell(_grid(1, 1), failed_world) is None
    assert frontier_world_cell(
        _grid(2, 1, origin_x=1.0), failed_world) == (0, 0)


def test_frontier_world_cell_and_selector_use_exact_cells_without_radius():
    grid = _grid(3, 2)
    failed_world = frontier_cell_world(grid, (0, 0))
    clusters = [((0, 0), ((0, 0), (1, 0), (2, 0)))]

    assert frontier_world_cell(grid, failed_world) == (0, 0)
    assert costmap_frontier_candidates(
        clusters, grid, _costmap(3, 2, [0] * 6), blacklist={(0, 0)}) == [(1, 0)]


def test_blocked_representative_selects_deterministic_same_cluster_endpoint():
    width, height = 180, 170
    data = [253] * (width * height)
    data[159 * width + 166] = 0
    data[160 * width + 168] = 1
    clusters = [((172, 162), ((172, 162), (166, 159), (168, 160)))]

    candidates = costmap_frontier_candidates(
        clusters, _grid(width, height), _costmap(width, height, data))

    assert candidates == [(166, 159)]


def test_minimum_distance_uses_farther_same_cluster_endpoint():
    clusters = [((0, 0), ((0, 0), (1, 0)))]

    candidates = costmap_frontier_candidates(
        clusters, _grid(2, 1), _costmap(2, 1, [0, 0]),
        robot_world=(0.5, 0.5), min_goal_distance=0.3)

    assert candidates == [(1, 0)]


@pytest.mark.parametrize("cost", [253, 254, 255])
def test_endpoint_costs_at_or_above_collision_threshold_are_rejected(cost):
    clusters = [((1, 1), ((1, 1),))]
    data = [0] * 9
    data[4] = cost

    assert costmap_frontier_candidates(
        clusters, _grid(3, 3), _costmap(3, 3, data)) == []


def test_footprint_intersection_rejects_free_center_and_selects_clear_sibling():
    width = height = 50
    data = [0] * (width * height)
    data[20 * width + 25] = 253
    grid = _grid(width, height, resolution=0.1)
    clusters = [((20, 20), ((20, 20), (15, 20)))]

    candidates = costmap_frontier_candidates(
        clusters, grid, _costmap(
            width, height, data, resolution=0.1),
        footprint=NAVIGATION_FOOTPRINT)

    assert candidates == [(15, 20)]


def test_footprint_gate_respects_translated_rotated_costmap_geometry():
    width = height = 4
    lethal_cell = (1, 1)
    data = [0] * (width * height)
    data[lethal_cell[1] * width + lethal_cell[0]] = 253
    grid = _grid(3, 3)
    costmap = _costmap(
        width, height, data, origin_x=2.2, origin_y=0.0,
        yaw=math.pi / 2.0)
    clusters = [((1, 1), ((1, 1),))]

    # The endpoint center maps to clear cell (1, 0), while the rotated and
    # translated footprint overlaps lethal cell (1, 1).
    assert costmap_frontier_candidates(
        clusters, grid, costmap, footprint=NAVIGATION_FOOTPRINT) == []

    data[lethal_cell[1] * width + lethal_cell[0]] = 0
    assert costmap_frontier_candidates(
        clusters, grid, costmap, footprint=NAVIGATION_FOOTPRINT) == [(1, 1)]


@pytest.mark.parametrize("cost,expected", [
    (252, [(20, 20)]),
    (253, []),
    (254, []),
    (255, []),
])
def test_footprint_gate_rejects_lethal_overlap_costs_but_allows_252(
        cost, expected):
    width = height = 50
    data = [0] * (width * height)
    data[20 * width + 25] = cost
    grid = _grid(width, height, resolution=0.1)
    clusters = [((20, 20), ((20, 20),))]

    assert costmap_frontier_candidates(
        clusters, grid,
        _costmap(width, height, data, resolution=0.1),
        footprint=NAVIGATION_FOOTPRINT) == expected


def test_navigation_footprint_matches_padded_nav2_footprint():
    planner_path = (
        Path(__file__).resolve().parents[1].parent
        / "amr_navigation" / "config" / "planner.yaml")
    planner = yaml.safe_load(planner_path.read_text())
    configured = yaml.safe_load(
        planner["/amr/global_costmap/global_costmap"]["ros__parameters"][
            "footprint"])
    observed_padding = 0.01
    expected = tuple(
        (point[0] + (observed_padding if point[0] >= 0.0 else -observed_padding),
         point[1] + (observed_padding if point[1] >= 0.0 else -observed_padding))
        for point in configured)

    assert len(NAVIGATION_FOOTPRINT) == len(expected)
    assert all(
        actual == pytest.approx(configured_point)
        for actual, configured_point in zip(NAVIGATION_FOOTPRINT, expected))


def test_footprint_fallback_ranks_centers_and_stops_at_first_clear(monkeypatch):
    width, height = 8, 2
    data = [0] * (width * height)
    data[0 * width + 1] = 2
    data[0 * width + 2] = 1
    data[0 * width + 3] = 0
    clusters = [
        ((1, 0), ((1, 0), (2, 0), (3, 0))),
        ((5, 0), ((5, 0),)),
    ]
    checks = []

    def record_footprint_check(_geometry, _data, world, _footprint):
        checks.append(world[0])
        return (math.isclose(world[0], 2.5)
                or math.isclose(world[0], 5.5))

    monkeypatch.setattr(
        frontier_algorithm_module,
        "_footprint_costmap_clear",
        record_footprint_check)

    candidates = costmap_frontier_candidates(
        clusters, _grid(width, height), _costmap(width, height, data),
        footprint=NAVIGATION_FOOTPRINT)

    # The center rank visits x=3 before x=2 by cost.  The first clear
    # fallback is returned, and the second cluster remains in sequence.
    assert candidates == [(2, 0), (5, 0)]
    # The representative x=1 is checked once despite also being a fallback;
    # x=3 is rejected, x=2 is clear, and x=5 is the next representative.
    assert checks == pytest.approx([1.5, 3.5, 2.5, 5.5])


@pytest.mark.parametrize(
    "candidate,lethal_cell", [
        ((169, 160), (170, 160)), ((167, 162), (168, 162))])
def test_recorded_collision_candidates_are_rejected_by_footprint_gate(
        candidate, lethal_cell):
    width, height = 180, 170
    data = [0] * (width * height)
    data[lethal_cell[1] * width + lethal_cell[0]] = 253

    assert costmap_frontier_candidates(
        [(candidate, (candidate,))],
        _grid(width, height),
        _costmap(width, height, data),
        footprint=NAVIGATION_FOOTPRINT) == []


def test_footprint_contact_and_costmap_boundary_are_inadmissible():
    width = height = 50
    data = [0] * (width * height)
    data[20 * width + 25] = 253
    grid = _grid(width, height, resolution=0.1)

    assert costmap_frontier_candidates(
        [((20, 20), ((20, 20),))], grid,
        _costmap(width, height, data, resolution=0.1),
        footprint=NAVIGATION_FOOTPRINT) == []

    assert costmap_frontier_candidates(
        [((1, 1), ((1, 1),))], _grid(3, 3, resolution=0.1),
        _costmap(3, 3, [0] * 9, resolution=0.1),
        footprint=NAVIGATION_FOOTPRINT) == []


@pytest.mark.parametrize("footprint", [
    ((0.0, 0.0), (1.0, 0.0)),
    ((0.0, 0.0), (1.0, 0.0), (math.nan, 1.0)),
    ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),
])
def test_malformed_or_degenerate_footprint_fails_closed(footprint):
    assert costmap_frontier_candidates(
        [((20, 20), ((20, 20),))],
        _grid(50, 50, resolution=0.1),
        _costmap(50, 50, [0] * 2500, resolution=0.1),
        footprint=footprint) is None


def test_out_of_bounds_endpoint_is_rejected():
    clusters = [((2, 2), ((2, 2),))]

    assert costmap_frontier_candidates(
        clusters, _grid(3, 3), _costmap(2, 2, [0] * 4)) == []


def test_map_and_costmap_geometry_are_independent_and_support_planar_yaw():
    # Map cell centers are (13, 23) and (11, 23).  With the costmap's
    # translated, finer, 90-degree-rotated geometry they map to (6, 4) and
    # (6, 8), respectively.  The (1, 1) cost is deliberately unrelated.
    map_grid = _grid(2, 2, resolution=2.0, origin_x=10.0, origin_y=20.0)
    clusters = [((1, 1), ((1, 1), (0, 1)))]
    data = [253] * 100
    data[1 * 10 + 1] = 0
    data[8 * 10 + 6] = 0

    candidates = costmap_frontier_candidates(
        clusters, map_grid,
        _costmap(10, 10, data, resolution=0.5,
                 origin_x=15.0, origin_y=20.0, yaw=math.pi / 2.0))

    assert candidates == [(0, 1)]


@pytest.mark.parametrize("case", ["wrong_frame", "bad_dimensions", "bad_geometry"])
def test_malformed_costmap_geometry_fails_closed(case):
    clusters = [((0, 0), ((0, 0),))]
    costmap = _costmap(1, 1, [0])
    if case == "wrong_frame":
        costmap.header.frame_id = "odom"
    elif case == "bad_dimensions":
        costmap.metadata.size_x = 2
    else:
        costmap.metadata.origin.orientation.w = 0.0

    assert costmap_frontier_candidates(
        clusters, _grid(1, 1), costmap) is None
