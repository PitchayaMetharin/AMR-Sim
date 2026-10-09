"""Goal heading = travel direction (root-authored; implementer must not edit).

Run 13: every goal was sent with yaw 0, so the robot turned to face +x at each
goal and turned back for the next one.  The goal yaw is the straight-line
heading from the robot to the goal when the footprint is clear there at that
heading; otherwise the previously proven yaw 0 is kept.
"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR.parent / "scripts"))
sys.path.insert(0, str(TEST_DIR))

import frontier_algorithm as fa  # noqa: E402
import frontier_explorer as fe_module  # noqa: E402
import test_frontier_evidence as evidence  # noqa: E402
import test_frontier_goal_clearance as clearance  # noqa: E402
import test_frontier_lifecycle as lifecycle  # noqa: E402


def _costmap(width=80, height=80, res=0.05, lethal=()):
    data = [0] * (width * height)
    for x, y in lethal:
        data[y * width + x] = 254
    return lifecycle._costmap_message(
        data=data, width=width, height=height, resolution=res,
        origin_x=0.0, origin_y=0.0)


def test_heading_is_the_travel_direction_in_open_space():
    costmap = _costmap()
    yaw = fa.goal_heading(costmap, (1.0, 1.0), (3.0, 2.0))
    assert yaw == pytest.approx(math.atan2(1.0, 2.0))


def test_heading_falls_back_to_zero_when_the_travel_heading_collides():
    """Lethal cell touched by the footprint rotated to 90 deg but not at 0."""
    goal = (2.0, 2.0)
    # Footprint at yaw 90 deg spans y +/- 0.61; at yaw 0 it spans y +/- 0.41.
    lethal = [(int(2.0 / 0.05), int((2.0 + 0.55) / 0.05))]
    costmap = _costmap(lethal=lethal)
    geometry = fa.costmap_geometry(costmap)
    fp = fa._strict_footprint(fa.NAVIGATION_FOOTPRINT)
    assert fa._footprint_costmap_clear(geometry, costmap.data, goal, fp, yaw=0.0)
    assert not fa._footprint_costmap_clear(
        geometry, costmap.data, goal, fp, yaw=math.pi / 2)
    assert fa.goal_heading(costmap, (2.0, 0.5), goal) == 0.0


@pytest.mark.parametrize("robot, goal", [((1.0, 1.0), (1.0, 1.0)), ((float("nan"), 0.0), (1.0, 1.0))])
def test_heading_degenerate_inputs_fall_back_to_zero(robot, goal):
    assert fa.goal_heading(_costmap(), robot, goal) == 0.0


def test_dispatched_goal_orientation_uses_the_heading_and_is_recorded():
    node, _pose = clearance._node_with_fixture("hospital_12_unsafe_start")
    evidence._run_until_terminal_or_dispatch(node, (0.0, 0.0, 0.0))
    assert node.action_client.send_calls
    sent = node.action_client.send_calls[-1].pose.pose
    yaw = math.atan2(2.0 * sent.orientation.w * sent.orientation.z,
                     1.0 - 2.0 * sent.orientation.z * sent.orientation.z)
    expected = fa.goal_heading(
        node.latest_costmap, (0.0, 0.0), (sent.position.x, sent.position.y))
    assert yaw == pytest.approx(expected, abs=1e-9)
    record = evidence._records(node, "dispatch")[-1]
    assert record["goal_yaw"] == pytest.approx(expected, abs=1e-9)


# ----------------------------------------- review follow-up (Opus high review)

def test_refreshed_pose_heading_uses_the_refreshed_position(monkeypatch):
    """The refresh branch stores fresh_pose as ((x, y), yaw); the heading must
    be computed from (x, y), not silently fall back to 0 (review finding 1)."""
    node, ros_clock = lifecycle._real_route_node()
    old = lifecycle._transform(stamp_ns=ros_clock.nanoseconds - 500_000_000, x=3.5, y=3.5)
    fresh = lifecycle._transform(stamp_ns=ros_clock.nanoseconds + 900_000_000, x=3.6, y=3.4)
    node.tf_buffer = SimpleNamespace(lookup_transform=lambda *_a, **_k: fresh)
    original = fe_module.costmap_frontier_candidates

    def delayed(*args, **kwargs):
        ros_clock.nanoseconds += 1_100_000_000
        return original(*args, **kwargs)

    monkeypatch.setattr(fe_module, "costmap_frontier_candidates", delayed)
    calls = []
    real_heading = fe_module.goal_heading

    def spy(costmap, robot_world, goal_world, **kwargs):
        calls.append(tuple(robot_world))
        return real_heading(costmap, robot_world, goal_world, **kwargs)

    monkeypatch.setattr(fe_module, "goal_heading", spy)
    node._select_frontier(node.run_generation, old)
    assert node.action_client.send_calls, node.reason
    assert calls and calls[-1] == pytest.approx((3.6, 3.4))


def test_heading_helper_rejects_malformed_robot_world_without_masking_bugs():
    with pytest.raises(TypeError):
        fa.goal_heading(_costmap(), ((1.0, 1.0), 0.3), (3.0, 3.0))


def test_frontier_cell_goal_keeps_zero_unless_it_can_turn_in_place():
    """Footprint clear at 0 and at h, but the circumscribed disk is not clear:
    with require_turn_clear the proven yaw 0 is kept (liveness, review Q3)."""
    goal = (2.0, 2.0)
    robot = (0.5, 2.5)                      # h is about 161.6 deg
    lethal = [(int((2.0 + 0.55) / 0.05), int((2.0 + 0.55) / 0.05))]   # ~0.78 m diagonal
    costmap = _costmap(lethal=lethal)
    geometry = fa.costmap_geometry(costmap)
    fp = fa._strict_footprint(fa.NAVIGATION_FOOTPRINT)
    h = math.atan2(robot[1] - goal[1], robot[0] - goal[0]) * 0 + math.atan2(goal[1] - robot[1], goal[0] - robot[0])
    assert fa._footprint_costmap_clear(geometry, costmap.data, goal, fp, yaw=0.0)
    assert fa._footprint_costmap_clear(geometry, costmap.data, goal, fp, yaw=h)
    assert not fa._turn_clearance_checker(geometry, costmap.data, fp)(goal)
    assert fa.goal_heading(costmap, robot, goal) == pytest.approx(h)
    assert fa.goal_heading(costmap, robot, goal, require_turn_clear=True) == 0.0


def test_dispatch_heading_matches_the_gate_rule():
    node, _pose = clearance._node_with_fixture("hospital_12_unsafe_start")
    evidence._run_until_terminal_or_dispatch(node, (0.0, 0.0, 0.0))
    record = evidence._records(node, "dispatch")[-1]
    expected = fa.goal_heading(
        node.latest_costmap, (0.0, 0.0), tuple(record["goal_world"]),
        require_turn_clear=(record["gate"] == "frontier_cell"))
    assert record["goal_yaw"] == pytest.approx(expected, abs=1e-9)
