"""Acceptance tests for opt-in continuous exploration (patrol) mode.

Root-authored before implementation; the implementer must not edit this file.
Default behaviour (continuous_exploration false) must stay unchanged.
"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from std_srvs.srv import Trigger

TEST_DIR = Path(__file__).resolve().parent
ROOT = TEST_DIR.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(TEST_DIR))

import frontier_explorer as frontier_explorer_module  # noqa: E402
from frontier_explorer import FrontierExplorer  # noqa: E402
from frontier_algorithm import (  # noqa: E402
    NAVIGATION_FOOTPRINT, frontier_world_cell, patrol_candidates)
import test_frontier_lifecycle as lifecycle  # noqa: E402
import frontier_algorithm as frontier_algorithm_module  # noqa: E402

NO_PATROL_REASON = (
    "continuous exploration: no reachable patrol goal; waiting for a map update")


# ---------------------------------------------------------------- algorithm

def _grid(width, height, data=None, resolution=1.0):
    return lifecycle._map_message(
        data=[0] * (width * height) if data is None else data,
        width=width, height=height, resolution=resolution)


def _costmap(width, height, data=None, resolution=1.0):
    return lifecycle._costmap_message(
        data=[0] * (width * height) if data is None else data,
        width=width, height=height, resolution=resolution,
        origin_x=0.0, origin_y=0.0)


def test_patrol_prefers_the_farthest_reachable_free_cell_deterministically():
    grid, costmap = _grid(10, 10), _costmap(10, 10)
    first = patrol_candidates(grid, costmap, (0.5, 0.5))
    second = patrol_candidates(grid, costmap, (0.5, 0.5))
    assert first == second
    assert first[0] == (9, 9)
    assert set(first) <= {(x, y) for x in range(10) for y in range(10)}


def test_patrol_returns_only_known_free_map_cells():
    data = [0] * 100
    data[5 * 10 + 5] = -1
    data[7 * 10 + 2] = 100
    data[9 * 10 + 9] = 50
    result = patrol_candidates(_grid(10, 10, data), _costmap(10, 10), (0.5, 0.5))
    assert result
    for excluded in ((5, 5), (2, 7), (9, 9)):
        assert excluded not in result


@pytest.mark.parametrize("cost, admitted", [(252, True), (253, False), (254, False)])
def test_patrol_endpoint_cost_uses_the_collision_threshold(cost, admitted):
    costmap_data = [0] * 100
    costmap_data[9 * 10 + 9] = cost
    result = patrol_candidates(_grid(10, 10), _costmap(10, 10, costmap_data), (0.5, 0.5))
    assert ((9, 9) in result) is admitted


def test_patrol_excludes_free_cells_behind_a_lethal_wall():
    costmap_data = [0] * 100
    for y in range(10):
        costmap_data[y * 10 + 5] = 254
    result = patrol_candidates(_grid(10, 10), _costmap(10, 10, costmap_data), (0.5, 0.5))
    assert result
    assert all(x < 5 for x, _y in result)


def test_patrol_respects_minimum_goal_distance():
    result = patrol_candidates(
        _grid(10, 10), _costmap(10, 10), (0.5, 0.5), min_goal_distance=3.0)
    assert result
    for x, y in result:
        assert math.hypot(x + 0.5 - 0.5, y + 0.5 - 0.5) >= 3.0


def test_patrol_avoids_worlds_within_the_avoid_radius():
    result = patrol_candidates(
        _grid(10, 10), _costmap(10, 10), (0.5, 0.5),
        avoid_worlds=((9.5, 9.5),), avoid_radius=1.5)
    assert result
    for x, y in result:
        assert math.hypot(x + 0.5 - 9.5, y + 0.5 - 9.5) > 1.5
    assert result[0] != (9, 9)


def test_patrol_visited_worlds_steer_toward_unvisited_space():
    result = patrol_candidates(
        _grid(10, 10), _costmap(10, 10), (0.5, 0.5),
        visited_worlds=((9.5, 9.5),))
    # (9, 0) and (0, 9) both score 9.0 m; ties break by (y, x) ascending.
    assert result[0] == (9, 0)


def test_patrol_samples_cells_on_the_requested_spacing():
    result = patrol_candidates(
        _grid(20, 20, resolution=0.5), _costmap(20, 20, resolution=0.5),
        (0.25, 0.25), sample_spacing=1.0)
    assert result
    assert all(x % 2 == 0 and y % 2 == 0 for x, y in result)


def test_patrol_footprint_excludes_cells_whose_footprint_overlaps_lethal_cost():
    width = height = 40
    costmap_data = [0] * (width * height)
    costmap_data[20 * width + 30] = 254
    result = patrol_candidates(
        _grid(width, height, resolution=0.1),
        _costmap(width, height, costmap_data, resolution=0.1),
        (1.0, 1.0), footprint=NAVIGATION_FOOTPRINT, sample_spacing=0.1)
    assert result
    for x, y in result:
        wx, wy = (x + 0.5) * 0.1, (y + 0.5) * 0.1
        # A lethal cell centre inside the axis-aligned footprint box is a hit.
        assert not (abs(wx - 3.05) <= 0.61 and abs(wy - 2.05) <= 0.41)
        # The footprint must also stay inside the costmap.
        assert 0.61 <= wx <= 4.0 - 0.61 and 0.41 <= wy <= 4.0 - 0.41


def _corridor_pocket(resolution=0.1):
    """8 m x 4 m: open room for x < 4 m, 1 m wide dead-end corridor for x >= 4 m."""
    width, height = 80, 40
    costmap_data = [0] * (width * height)
    for y in range(height):
        for x in range(40, width):
            if not 15 <= y <= 24:
                costmap_data[y * width + x] = 254
    return (_grid(width, height, resolution=resolution),
            _costmap(width, height, costmap_data, resolution=resolution))


def test_patrol_rejects_cells_where_the_robot_cannot_turn_in_place():
    """Regression: AWS patrol run 20261008_01 parked the robot in a pocket it could
    enter forward but never turn around in, so every later patrol search was empty."""
    grid, costmap = _corridor_pocket()
    result = patrol_candidates(
        grid, costmap, (1.55, 2.05), footprint=NAVIGATION_FOOTPRINT,
        min_goal_distance=1.5, sample_spacing=0.5)
    assert result
    for x, y in result:
        wx, wy = (x + 0.5) * 0.1, (y + 0.5) * 0.1
        assert wx < 4.0, (x, y)
        for k in range(32):
            assert frontier_algorithm_module._footprint_costmap_clear(
                frontier_algorithm_module.costmap_geometry(costmap), costmap.data,
                (wx, wy), NAVIGATION_FOOTPRINT, yaw=k * math.pi / 16.0), (x, y, k)


def test_patrol_turn_in_place_gate_keeps_open_space_candidates():
    grid, costmap = _corridor_pocket()
    result = patrol_candidates(
        grid, costmap, (1.55, 2.05), footprint=NAVIGATION_FOOTPRINT,
        min_goal_distance=0.0, sample_spacing=0.5)
    # The room centre has >= 0.74 m of clearance in every direction.
    assert (20, 20) in result


@pytest.mark.parametrize("kwargs", [
    {"sample_spacing": 0.0},
    {"sample_spacing": -1.0},
    {"sample_spacing": math.nan},
    {"robot_world": (math.nan, 0.5)},
    {"min_goal_distance": -1.0},
    {"avoid_radius": -1.0},
])
def test_patrol_rejects_invalid_inputs(kwargs):
    arguments = {"robot_world": (0.5, 0.5)}
    arguments.update(kwargs)
    robot_world = arguments.pop("robot_world")
    assert patrol_candidates(
        _grid(10, 10), _costmap(10, 10), robot_world, **arguments) is None


def test_patrol_rejects_malformed_costmap_geometry():
    assert patrol_candidates(_grid(10, 10), _costmap(10, 10, [0] * 99), (0.5, 0.5)) is None


def test_patrol_robot_on_lethal_cost_has_no_candidates():
    result = patrol_candidates(
        _grid(10, 10), _costmap(10, 10, [254] * 100), (0.5, 0.5))
    assert result == []


# ----------------------------------------------------------------- explorer

def _open_floor_node(continuous, width=12, height=12, unknown=()):
    node = lifecycle._node(autostart=True)
    node.continuous_exploration = continuous
    _set_floor(node, width, height, unknown)
    return node


def _set_floor(node, width=12, height=12, unknown=(), costmap_data=None):
    data = [0] * (width * height)
    for x, y in unknown:
        data[y * width + x] = -1
    node.latest_map = lifecycle._map_message(
        data=data, width=width, height=height, resolution=1.0)
    node.map_version += 1
    node.latest_costmap = lifecycle._costmap_message(
        data=[0] * (width * height) if costmap_data is None else costmap_data,
        width=width, height=height, resolution=1.0, origin_x=0.0, origin_y=0.0)
    node.costmap_version += 1


def _plan(node, x=1.5, y=1.5):
    now = node._monotonic()
    node.last_map_at = now
    node.last_costmap_at = now
    node.last_base_status_at = now
    node.last_manipulator_status_at = now
    node.state = "PLANNING"
    node.map_version += 1
    node._select_frontier(node.run_generation, lifecycle._transform(x=x, y=y))


def _goal_world(node, index=-1):
    position = node.action_client.send_calls[index].pose.pose.position
    return (position.x, position.y)


def _status(node):
    with node._lock:
        node._publish_status_locked()
    return lifecycle._diagnostic_values(node)


def _plan_until_dispatch(node, x=1.5, y=1.5):
    for _ in range(node.no_frontier_limit):
        _plan(node, x, y)
        if node.action_client.send_calls:
            break
    assert len(node.action_client.send_calls) == 1
    return _goal_world(node)


def _succeed(node):
    handle = lifecycle._accept(node)
    handle.result_future.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert node.state == "SCANNING"


def _is_free_cell_centre(node, world):
    cell = frontier_world_cell(node.latest_map, world)
    assert cell is not None
    assert world == pytest.approx((cell[0] + 0.5, cell[1] + 0.5))
    return node.latest_map.data[cell[1] * node.latest_map.info.width + cell[0]] == 0


def test_default_mode_still_completes_when_frontiers_are_exhausted():
    node = _open_floor_node(continuous=False)
    for _ in range(node.no_frontier_limit):
        _plan(node)
    assert node.state == "COMPLETE"
    assert node.reason == "exploration complete: no costmap-valid frontier remains"
    assert node.action_client.send_calls == []
    assert _status(node)["continuous_exploration"] == "false"


def test_continuous_mode_patrols_instead_of_completing():
    node = _open_floor_node(continuous=True)
    for _ in range(node.no_frontier_limit - 1):
        _plan(node)
        assert node.state == "SCANNING"
        assert node.reason == "no costmap-valid frontier in this map update"
        assert node.action_client.send_calls == []
    _plan(node)

    assert node.state == "GOAL_PENDING"
    assert len(node.action_client.send_calls) == 1
    goal = _goal_world(node)
    assert math.dist(goal, (1.5, 1.5)) >= frontier_explorer_module.PATROL_MIN_GOAL_DISTANCE_M
    assert _is_free_cell_centre(node, goal)
    assert node.fault_latched is False
    values = _status(node)
    assert values["continuous_exploration"] == "true"
    assert values["patrol_active"] == "true"
    assert values["state"] == "GOAL_PENDING"


def test_patrol_continues_after_success_without_a_new_exhaustion_wait():
    node = _open_floor_node(continuous=True)
    first = _plan_until_dispatch(node)
    _succeed(node)
    assert node.reached_goal_count == 1

    _plan(node, *first)

    assert len(node.action_client.send_calls) == 2
    second = _goal_world(node)
    assert math.dist(second, first) >= frontier_explorer_module.PATROL_MIN_GOAL_DISTANCE_M
    assert _is_free_cell_centre(node, second)
    assert node.state == "GOAL_PENDING"


@pytest.mark.parametrize("kind", ["failed", "blocked"])
def test_patrol_avoids_failed_and_blocked_destinations(kind):
    reference = _open_floor_node(continuous=True)
    avoided = _plan_until_dispatch(reference)

    node = _open_floor_node(continuous=True)
    if kind == "failed":
        node.failed_goal_worlds = {avoided}
    else:
        node._blocked_destinations = {
            node._world_identity(avoided): {
                "world": avoided,
                "fingerprints": set(),
                "attempts": frontier_explorer_module.MAX_BLOCKAGE_ATTEMPTS,
            }}
    goal = _plan_until_dispatch(node)
    assert math.dist(goal, avoided) > frontier_explorer_module.PATROL_AVOID_RADIUS_M


def test_no_reachable_patrol_goal_waits_without_terminating_and_stop_still_works():
    node = _open_floor_node(continuous=True)
    _set_floor(node, costmap_data=[254] * 144)
    for _ in range(node.no_frontier_limit + 3):
        _plan(node)
    assert node.state == "SCANNING"
    assert node.reason == NO_PATROL_REASON
    assert node.action_client.send_calls == []
    assert node.fault_latched is False

    response = node._stop_callback(None, Trigger.Response())
    assert response.success is True
    assert node.state == "STOPPED"


def test_a_new_reachable_frontier_takes_over_from_patrol():
    node = _open_floor_node(continuous=True)
    first = _plan_until_dispatch(node)
    _succeed(node)

    unknown = (6, 9)
    _set_floor(node, unknown=(unknown,))
    _plan(node, *first)

    assert len(node.action_client.send_calls) == 2
    goal_cell = frontier_world_cell(node.latest_map, _goal_world(node))
    assert max(abs(goal_cell[0] - unknown[0]), abs(goal_cell[1] - unknown[1])) == 1
    assert _status(node)["patrol_active"] == "false"


def test_unresolved_frontier_classification_patrols_in_continuous_mode(monkeypatch):
    def unresolved_selector(*_args, **_kwargs):
        return ([], [{
            "classification": "UNRESOLVED",
            "safe_endpoint_count": 0,
            "reachable_endpoint_count": 0,
        }])

    monkeypatch.setattr(
        frontier_explorer_module, "costmap_frontier_candidates", unresolved_selector)
    node = _open_floor_node(continuous=True, unknown=((6, 9),))
    _plan_until_dispatch(node)
    assert node.state == "GOAL_PENDING"
    assert node.state != "INCOMPLETE"


def test_stop_and_restart_clear_patrol_state():
    node = _open_floor_node(continuous=True)
    _plan_until_dispatch(node)
    _succeed(node)
    assert _status(node)["patrol_active"] == "true"

    response = node._stop_callback(None, Trigger.Response())
    assert response.success is True
    assert node.state == "STOPPED"

    response = lifecycle._start(node)
    assert response.success is True
    assert _status(node)["patrol_active"] == "false"
    assert node.continuous_exploration is True


@pytest.mark.parametrize("value", [1, "true", None])
def test_continuous_exploration_parameter_must_be_boolean(monkeypatch, value):
    entity_calls = []

    def fake_node_init(node, *_args, **_kwargs):
        node._test_parameters = {}

    def fake_declare_parameter(node, parameter_name, default):
        node._test_parameters[parameter_name] = (
            value if parameter_name == "continuous_exploration" else default)

    def fake_get_parameter(node, parameter_name):
        return SimpleNamespace(value=node._test_parameters[parameter_name])

    def record_entity(*_args, **_kwargs):
        entity_calls.append(True)

    monkeypatch.setattr(Node, "__init__", fake_node_init)
    monkeypatch.setattr(Node, "declare_parameter", fake_declare_parameter)
    monkeypatch.setattr(Node, "get_parameter", fake_get_parameter)
    for method in ("create_subscription", "create_publisher", "create_service",
                   "create_timer"):
        monkeypatch.setattr(Node, method, record_entity)

    with pytest.raises(ValueError):
        FrontierExplorer()
    assert entity_calls == []


def test_continuous_exploration_defaults_off_in_source_and_config():
    source = (ROOT / "scripts" / "frontier_explorer.py").read_text()
    assert 'declare_parameter("continuous_exploration", False)' in source
    config = (ROOT / "config" / "frontier_explorer.yaml").read_text()
    assert "continuous_exploration: false" in config
