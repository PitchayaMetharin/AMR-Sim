"""Bounded stop-and-resume on localization loss (root-authored acceptance tests,
2026-10-09; implementer must not edit).

Hospital runs 07/14: slam_toolbox froze the map->odom stamp during map rebuilds,
the controller's TF lookup failed, FollowPath aborted and the explorer
FAULT-latched.  Approved design (Opus-high design study, user approved):
- the mission supervisor reports fault_class LOCALIZATION_UNAVAILABLE with the
  exact reason "path following lost map localization", outcome FAULT,
  blockage_confirmed false;
- the explorer enters RECOVERY_WAIT in localization mode (no blockage record, no
  goal failure, no failed goal world), requires TWO consecutive samples with the
  robot stationary, STRICTLY INCREASING map->base_footprint stamps and
  localization headroom (newest map->odom stamp minus newest odom->base stamp)
  >= LOCALIZATION_HEADROOM_MIN_S, then resumes through SCANNING;
- per-episode deadline LOCALIZATION_RECOVERY_DEADLINE_S (5.0 s) and a budget of
  LOCALIZATION_RECOVERY_LIMIT (3) consecutive recoveries (reset by a reached
  goal and by start); exhaustion -> FAULT with mission fault class
  LOCALIZATION_UNAVAILABLE and an explicit reason.
"""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR.parent / "scripts"))
sys.path.insert(0, str(TEST_DIR))

import frontier_explorer as fe  # noqa: E402
import test_frontier_lifecycle as lifecycle  # noqa: E402

LOC_REASON = "path following lost map localization"


class LocTF:
    """Fake TF buffer answering by frame pair with independent stamps."""

    def __init__(self, map_odom_s, odom_base_s, x=14.0, y=10.0):
        self.map_odom_s = map_odom_s
        self.odom_base_s = odom_base_s
        self.x, self.y = x, y

    def set(self, map_odom_s, odom_base_s, x=None, y=None):
        self.map_odom_s, self.odom_base_s = map_odom_s, odom_base_s
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y

    def lookup_transform(self, target, source, *_args, **_kwargs):
        if (target, source) == ("map", "odom"):
            return lifecycle._transform(stamp_ns=int(self.map_odom_s * 1e9), x=0.0, y=0.0)
        if (target, source) == ("odom", "base_footprint"):
            return lifecycle._transform(stamp_ns=int(self.odom_base_s * 1e9), x=self.x, y=self.y)
        stamp = min(self.map_odom_s, self.odom_base_s)
        return lifecycle._transform(stamp_ns=int(stamp * 1e9), x=self.x, y=self.y)


def _fresh(node):
    now = node._monotonic()
    node.last_map_at = now
    node.last_costmap_at = now
    node.last_base_status_at = now
    node.last_manipulator_status_at = now


def _loc_node():
    node = lifecycle._selection_fixture(lifecycle._node(autostart=True))
    mono = lifecycle.FakeMonotonic(lifecycle.time.monotonic())
    node._monotonic = mono
    node.tf_buffer = LocTF(10.4, 9.5)          # healthy: +0.9 s headroom (fake ROS now = 10.0 s)
    _fresh(node)
    return node, mono


def _dispatch(node, goal_world=(14.5, 10.5)):
    before = len(node.action_client.send_calls)
    lifecycle._dispatch_pending(node, candidate=(14, 10), goal_world=goal_world) \
        if before == 0 else _dispatch_more(node, goal_world)
    return lifecycle._accept(node)


def _dispatch_more(node, goal_world):
    node.state = "PLANNING"
    _fresh(node)
    node._reserve_and_send(
        object(), (14, 10), node.run_generation,
        map_snapshot=(node.latest_map, node.map_version, node.last_map_at),
        costmap_snapshot=(node.latest_costmap, node.costmap_version, node.last_costmap_at),
        transform=lifecycle._transform(), goal_world=goal_world)


def _localization_abort(node, handle, reason=LOC_REASON, blockage=False,
                        fault_class="LOCALIZATION_UNAVAILABLE"):
    node._mission_status_callback(lifecycle._mission_status(
        node._expected_goal_uuid, outcome="FAULT", reason=reason,
        blockage=blockage, fault_class=fault_class))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))


def _tick(node, mono, seconds=0.2):
    mono.advance(seconds)
    _fresh(node)
    node._tick()


def _status(node):
    with node._lock:
        node._publish_status_locked()
    return lifecycle._diagnostic_values(node)


def test_constants_are_bounded():
    assert fe.LOCALIZATION_RECOVERY_DEADLINE_S == pytest.approx(5.0)
    assert fe.LOCALIZATION_RECOVERY_LIMIT == 3
    assert fe.LOCALIZATION_HEADROOM_MIN_S == pytest.approx(0.5)
    assert "LOCALIZATION_UNAVAILABLE" in fe.MISSION_FAULT_CLASSES


def test_localization_abort_enters_recovery_without_blockage_or_failure():
    node, _mono = _loc_node()
    handle = _dispatch(node)
    _localization_abort(node, handle)
    assert node.state == "RECOVERY_WAIT", node.reason
    assert node.fault_latched is False
    assert node._motion_owned is False
    assert not getattr(node, "_blocked_destinations", {})
    assert node.goal_failures == 0
    assert getattr(node, "_consecutive_blockages", 0) == 0
    assert not node.failed_goal_worlds
    values = _status(node)
    assert values["mission_fault_class"] == "LOCALIZATION_UNAVAILABLE"
    assert values["localization_recoveries"] == "1"


def test_resume_requires_headroom_and_advancing_stamps_then_reselects():
    node, mono = _loc_node()
    handle = _dispatch(node)
    sends = len(node.action_client.send_calls)
    node.tf_buffer.set(9.4, 9.5)               # still stalled: negative headroom
    _localization_abort(node, handle)
    for _ in range(4):
        _tick(node, mono)
        assert node.state == "RECOVERY_WAIT"
    # Healthy headroom but FROZEN stamps (stall artefact): never stationary proof.
    node.tf_buffer.set(10.4, 9.5)
    for _ in range(3):
        _tick(node, mono)
        assert node.state == "RECOVERY_WAIT"
    assert len(node.action_client.send_calls) == sends
    # Healthy and advancing: two samples, then SCANNING, then one new dispatch.
    node.tf_buffer.set(10.5, 9.6)
    _tick(node, mono)
    node.tf_buffer.set(10.6, 9.7)
    _tick(node, mono)
    assert node.state == "SCANNING", node.reason
    assert len(node.action_client.send_calls) == sends
    _tick(node, mono)
    assert len(node.action_client.send_calls) == sends + 1


def test_motion_between_samples_restarts_the_proof():
    node, mono = _loc_node()
    handle = _dispatch(node)
    _localization_abort(node, handle)
    node.tf_buffer.set(10.5, 9.6)
    _tick(node, mono)
    node.tf_buffer.set(10.6, 9.7, x=14.5)      # moved 0.5 m
    _tick(node, mono)
    assert node.state == "RECOVERY_WAIT"
    node.tf_buffer.set(10.7, 9.8)
    _tick(node, mono)
    assert node.state == "SCANNING"


def test_deadline_ends_in_localization_fault():
    node, mono = _loc_node()
    handle = _dispatch(node)
    node.tf_buffer.set(9.4, 9.5)
    _localization_abort(node, handle)
    sends = len(node.action_client.send_calls)
    for _ in range(30):                          # 6 s of unhealthy ticks
        if node.state == "FAULT":
            break
        _tick(node, mono)
    assert node.state == "FAULT"
    assert node.fault_latched is True
    assert "localization did not recover within 5.0 s" in node.reason
    assert _status(node)["mission_fault_class"] == "LOCALIZATION_UNAVAILABLE"
    assert len(node.action_client.send_calls) == sends


def _recover(node, mono):
    step = getattr(node, "_test_tf_step", 0) + 2
    node._test_tf_step = step
    base = 9.5 + 0.01 * step                     # stays inside the freshness window
    node.tf_buffer.set(base + 0.9, base)
    _tick(node, mono)
    node.tf_buffer.set(base + 0.91, base + 0.01)
    _tick(node, mono)
    assert node.state == "SCANNING", node.reason


def test_fourth_consecutive_loss_without_progress_is_terminal():
    node, mono = _loc_node()
    for episode in range(fe.LOCALIZATION_RECOVERY_LIMIT):
        handle = _dispatch(node)
        _localization_abort(node, handle)
        assert node.state == "RECOVERY_WAIT", (episode, node.reason)
        _recover(node, mono)
    handle = _dispatch(node)
    _localization_abort(node, handle)
    assert node.state == "FAULT", node.reason
    assert "consecutive localization recoveries exhausted" in node.reason
    assert node.fault_latched is True


def test_reached_goal_resets_the_localization_budget():
    node, mono = _loc_node()
    for _ in range(fe.LOCALIZATION_RECOVERY_LIMIT):
        handle = _dispatch(node)
        _localization_abort(node, handle)
        _recover(node, mono)
    handle = _dispatch(node)
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    for _ in range(fe.LOCALIZATION_RECOVERY_LIMIT):
        handle = _dispatch(node)
        _localization_abort(node, handle)
        assert node.state == "RECOVERY_WAIT", node.reason
        _recover(node, mono)


@pytest.mark.parametrize("kwargs", [
    {"reason": "path following failed"},
    {"blockage": True},
    {"fault_class": "CONTROLLER_ABORT", "reason": "path following failed"},
])
def test_non_matching_localization_evidence_stays_fail_closed(kwargs):
    node, _mono = _loc_node()
    handle = _dispatch(node)
    _localization_abort(node, handle, **kwargs)
    assert node.state == "FAULT"
    assert node.fault_latched is True


def test_operator_stop_during_localization_recovery_stops():
    node, _mono = _loc_node()
    handle = _dispatch(node)
    _localization_abort(node, handle)
    assert node.state == "RECOVERY_WAIT"
    from std_srvs.srv import Trigger
    response = node._stop_callback(None, Trigger.Response())
    assert response.success is True
    assert node.state == "STOPPED"
