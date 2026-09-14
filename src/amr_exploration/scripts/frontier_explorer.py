#!/usr/bin/env python3
"""Fail-closed frontier exploration through the AMR mission action boundary."""

import math
from numbers import Integral, Real
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from amr_interfaces.msg import BaseStatus, ManipulatorStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_algorithm import (
    NAVIGATION_FOOTPRINT, costmap_frontier_candidates, frontier_cell_world,
    frontier_clusters, costmap_geometry, frontier_world_cell,
    occupancy_grid_geometry)


_UNSET = object()


def _parameter_bool(value, name):
    if type(value) is not bool:
        raise ValueError("frontier explorer parameter %s must be bool" % name)
    return value


def _parameter_positive_real(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(
            "frontier explorer parameter %s must be a positive finite real" % name)
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(
            "frontier explorer parameter %s must be a positive finite real" % name)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(
            "frontier explorer parameter %s must be a positive finite real" % name)
    return value


def _parameter_positive_integral(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(
            "frontier explorer parameter %s must be a positive integer" % name)
    value = int(value)
    if value <= 0:
        raise ValueError(
            "frontier explorer parameter %s must be a positive integer" % name)
    return value


class _CancellationRecord:
    """Immutable cancellation identity with a single recorded outcome."""

    __slots__ = ("token", "deadline", "event", "outcome")

    def __init__(self, token, deadline):
        self.token = token
        self.deadline = deadline
        self.event = threading.Event()
        self.outcome = None

    def __setattr__(self, name, value):
        if name in ("token", "deadline", "event") and hasattr(self, name):
            raise AttributeError("cancellation identity is immutable")
        object.__setattr__(self, name, value)


class FrontierExplorer(Node):
    """Select frontiers and delegate all motion to the existing mission node."""

    _TERMINAL_STATES = frozenset(("STOPPED", "COMPLETE", "INCOMPLETE", "FAULT"))

    def __init__(self, *, context=None):
        super().__init__("frontier_explorer", context=context)
        self.declare_parameter("autostart", True)
        self.declare_parameter("map_timeout_sec", 3.0)
        self.declare_parameter("tf_timeout_sec", 1.0)
        self.declare_parameter("no_frontier_updates", 3)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("max_goal_failures", 3)
        self.declare_parameter("authority_timeout_sec", 1.0)
        self.declare_parameter("startup_grace_sec", 15.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)
        self.declare_parameter("min_goal_distance_m", 0.3)
        self.autostart = _parameter_bool(
            self.get_parameter("autostart").value, "autostart")
        self.map_timeout = _parameter_positive_real(
            self.get_parameter("map_timeout_sec").value, "map_timeout_sec")
        self.tf_timeout = _parameter_positive_real(
            self.get_parameter("tf_timeout_sec").value, "tf_timeout_sec")
        self.no_frontier_limit = _parameter_positive_integral(
            self.get_parameter("no_frontier_updates").value, "no_frontier_updates")
        self.goal_timeout = _parameter_positive_real(
            self.get_parameter("goal_timeout_sec").value, "goal_timeout_sec")
        self.max_goal_failures = _parameter_positive_integral(
            self.get_parameter("max_goal_failures").value, "max_goal_failures")
        self.authority_timeout = _parameter_positive_real(
            self.get_parameter("authority_timeout_sec").value, "authority_timeout_sec")
        self.startup_grace = _parameter_positive_real(
            self.get_parameter("startup_grace_sec").value, "startup_grace_sec")
        self.cancel_timeout = _parameter_positive_real(
            self.get_parameter("cancel_timeout_sec").value, "cancel_timeout_sec")
        self.min_goal_distance = _parameter_positive_real(
            self.get_parameter("min_goal_distance_m").value, "min_goal_distance_m")

        self._lock = threading.RLock()
        self.action_callback_group = ReentrantCallbackGroup()
        self.lifecycle_callback_group = MutuallyExclusiveCallbackGroup()
        self.planning_callback_group = MutuallyExclusiveCallbackGroup()
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_sub = self.create_subscription(
            Costmap, "/amr/global_costmap/costmap_raw", self._costmap_callback,
            costmap_qos)
        self.base_sub = self.create_subscription(BaseStatus, "/amr/base/status", self._base_callback, 10)
        self.manipulator_sub = self.create_subscription(
            ManipulatorStatus, "/amr/manipulation/status", self._manipulator_callback, 1)
        self.status_pub = self.create_publisher(DiagnosticArray, "/amr/exploration/status", 10)
        self.start_service = self.create_service(
            Trigger, "/amr/exploration/start", self._start_callback,
            callback_group=self.lifecycle_callback_group)
        self.stop_service = self.create_service(
            Trigger, "/amr/exploration/stop", self._stop_callback,
            callback_group=self.lifecycle_callback_group)
        self.action_client = ActionClient(
            self, NavigateToPose, "/amr/mission/navigate_to_pose",
            callback_group=self.action_callback_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Receipt times use the steady clock.  ROS time is reserved for TF
        # header age and goal timestamps.
        self.started_at = self._monotonic()
        self.readiness_wait_started_at = (
            self.started_at if self.autostart else None)
        self.last_map_at = None
        self.latest_map = None
        self.map_version = 0
        self.processed_map_version = -1
        self.last_costmap_at = None
        self.latest_costmap = None
        self.costmap_version = 0
        self.last_base_status_at = None
        self.last_manipulator_status_at = None
        self.base_status = None
        self.manipulator_status = None

        # A motion token remains owned from reservation through result and
        # cancellation acknowledgement.  This prevents late action futures
        # from reopening admission or dispatching another goal.
        self.run_generation = 1 if self.autostart else 0
        self.motion_generation = 0
        self._motion_token = None
        self._motion_owned = False
        self._pending = False
        self._pending_token = None
        self._pending_future = None
        self._pending_candidate = None
        self._attempted_goal_world = None
        self.active_goal = None
        self._active_token = None
        self._result_status = None
        self._result_future = None
        self.goal_started_at = None
        self._motion_deadline = None
        self.cancel_requested = False
        self.cancel_started_at = None
        self.cancel_reason = ""
        self._cancel_target = ""
        self._cancel_invoked = False
        self._cancel_future = None
        self._cancel_ack = False
        self._cancel_record = None
        self.cancel_event = threading.Event()
        self.blacklist = set()
        self.failed_goal_worlds = set()
        self.goal_failures = 0
        self.no_frontier_updates_seen = 0
        self.active_candidate = None
        self.fault_requested = False
        self.fault_latched = False
        self.reason = "autostart waiting for readiness" if self.autostart else "autostart disabled"
        self.state = "WAITING_READY" if self.autostart else "STOPPED"

        self.timer = self.create_timer(
            0.2, self._tick, callback_group=self.planning_callback_group)
        with self._lock:
            self._publish_status_locked()

    # ------------------------------------------------------------------
    # Status and lifecycle helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _monotonic():
        return time.monotonic()

    def _now_ros_ns(self):
        try:
            return int(self.get_clock().now().nanoseconds)
        except (AttributeError, TypeError, ValueError):
            return 0

    def _now_ros_msg(self):
        try:
            return self.get_clock().now().to_msg()
        except AttributeError:
            return None

    @staticmethod
    def _format_candidate(candidate):
        if candidate is None:
            return ""
        return "%s,%s" % (candidate[0], candidate[1])

    @staticmethod
    def _validated_world_point(world):
        try:
            world_x, world_y = world
            world_point = (float(world_x), float(world_y))
            if not all(math.isfinite(value) for value in world_point):
                return None
            return world_point
        except (TypeError, ValueError, OverflowError):
            return None

    @staticmethod
    def _goal_pose_world(goal):
        try:
            position = goal.pose.pose.position
            return FrontierExplorer._validated_world_point(
                (position.x, position.y))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    def _status_message_locked(self):
        message = DiagnosticArray()
        now = self._now_ros_msg()
        if now is not None:
            message.header.stamp = now
        diagnostic = DiagnosticStatus()
        diagnostic.name = "amr_exploration/frontier_explorer"
        diagnostic.level = (
            DiagnosticStatus.ERROR if self.state == "FAULT" else DiagnosticStatus.OK)
        diagnostic.message = self.reason or self.state
        values = {
            "state": self.state,
            "reason": self.reason,
            "run_generation": self.run_generation,
            "motion_generation": self.motion_generation,
            "map_version": self.map_version,
            "pending": self._pending,
            "active": self.active_goal is not None,
            "cancel_target": self._cancel_target,
            "cancel_ack": self._cancel_ack,
            "goal_failures": self.goal_failures,
            "no_frontier_updates_seen": self.no_frontier_updates_seen,
            "candidate": self._format_candidate(self.active_candidate),
            "fault_latched": self.fault_latched,
        }
        for key, value in values.items():
            item = KeyValue()
            item.key = str(key)
            item.value = str(value).lower() if isinstance(value, bool) else str(value)
            diagnostic.values.append(item)
        message.status.append(diagnostic)
        return message

    def _publish_status_locked(self):
        publisher = getattr(self, "status_pub", None)
        if publisher is not None:
            publisher.publish(self._status_message_locked())

    def _set_state_locked(self, state, reason, force=False):
        changed = self.state != state or self.reason != reason
        self.state = state
        self.reason = reason
        if changed or force:
            self._publish_status_locked()

    def _decision_matches_locked(
            self, run_generation=None, motion_token=_UNSET,
            expected_state=None, expected_deadline=_UNSET,
            expected_started_at=_UNSET, expected_cancel_record=_UNSET,
            expected_cancel_deadline=_UNSET):
        if run_generation is not None and self.run_generation != run_generation:
            return False
        if motion_token is not _UNSET and self._motion_token != motion_token:
            return False
        if expected_state is not None and self.state != expected_state:
            return False
        if expected_deadline is not _UNSET and self._motion_deadline != expected_deadline:
            return False
        if expected_started_at is not _UNSET and self.started_at != expected_started_at:
            return False
        if (expected_cancel_record is not _UNSET
                and self._cancel_record is not expected_cancel_record):
            return False
        if expected_cancel_deadline is not _UNSET:
            if (self._cancel_record is None
                    or self._cancel_record.deadline != expected_cancel_deadline):
                return False
        return True

    def _applicable_cancel_record_locked(self):
        """Return the current run's cancellation record, if still applicable."""
        record = self._cancel_record
        if record is None or record.token[0] != self.run_generation:
            return None
        if self._motion_owned:
            if not self.cancel_requested or self._motion_token != record.token:
                return None
        elif record.outcome is None:
            return None
        elif record.outcome[0]:
            if self.fault_latched or self.state not in ("STOPPED", "COMPLETE"):
                return None
        elif not self.fault_latched and self.state != "FAULT":
            return None
        return record

    def _detach_cancel_record_locked(self):
        """Detach an old record without disturbing callers waiting on it."""
        self._cancel_record = None
        self.cancel_event = threading.Event()
        self.cancel_started_at = None

    def _new_cancel_record_locked(self, token, started_at=None):
        if started_at is None:
            started_at = self._monotonic()
        record = _CancellationRecord(token, started_at + self.cancel_timeout)
        self._cancel_record = record
        self.cancel_event = record.event
        self.cancel_started_at = started_at
        return record

    @staticmethod
    def _set_cancel_outcome_locked(record, success, message):
        if record is None:
            return bool(success), message
        if record.outcome is None:
            record.outcome = (bool(success), message)
        return record.outcome

    def _signal_cancel_record_locked(self, record):
        if record is None:
            self.cancel_event.set()
            return
        if self._cancel_record is record:
            self.cancel_event = record.event
        record.event.set()

    def _complete_cancellation_locked(
            self, record, terminal_state, reason, success, message):
        outcome = record.outcome if record is not None else None
        if outcome is None:
            was_fault_latched = self.fault_latched
            outcome = self._set_cancel_outcome_locked(record, success, message)
            if record is None or self._motion_token == record.token:
                self._clear_motion_locked()
            self._clear_readiness_episode_locked()
            self.state = terminal_state
            if terminal_state == "FAULT":
                self.fault_requested = True
                self.fault_latched = True
            if terminal_state != "FAULT" or not was_fault_latched:
                self.reason = reason
            self._publish_status_locked()
            self._signal_cancel_record_locked(record)
            return outcome

        # A late proof may release the old motion token, but it cannot alter
        # the terminal state or the already-recorded cancellation outcome.
        if record is not None and self._motion_token == record.token:
            self._clear_motion_locked()
            self._publish_status_locked()
        self._signal_cancel_record_locked(record)
        return outcome

    def _commit_cancel_timeout_locked(
            self, record, expected_run_generation, expected_motion_token,
            expected_state, expected_deadline, reason, now=None):
        """Commit one cancellation timeout without touching a newer identity."""
        if record is None:
            return None
        if record.outcome is not None:
            return record.outcome
        if now is None:
            now = self._monotonic()
        if now < record.deadline:
            return None

        if (record.token != expected_motion_token
                or record.token[0] != expected_run_generation
                or record.deadline != expected_deadline
                or not self._motion_owned
                or not self.cancel_requested
                or not self._decision_matches_locked(
                    run_generation=expected_run_generation,
                    motion_token=expected_motion_token,
                    expected_state=expected_state,
                    expected_cancel_record=record,
                    expected_cancel_deadline=expected_deadline)):
            return None

        outcome = self._set_cancel_outcome_locked(
            record, False, "cancellation was not confirmed; explorer faulted closed")
        was_fault_latched = self.fault_latched
        self.fault_requested = True
        self.fault_latched = True
        if self.state != "FAULT":
            self.state = "FAULT"
        if not was_fault_latched:
            self.reason = reason
        self._publish_status_locked()
        self._signal_cancel_record_locked(record)
        return outcome

    def _terminal_locked(self, state, reason):
        """Commit terminal state, publish it, then release stop waiters."""
        self._clear_readiness_episode_locked()
        self.state = state
        self.reason = reason
        if state == "FAULT":
            self.fault_requested = True
            self.fault_latched = True
        # The status publication deliberately precedes the event signal.  A
        # stop caller must never observe completion without its terminal proof.
        self._publish_status_locked()
        self.cancel_event.set()

    def _clear_motion_locked(self):
        self._motion_token = None
        self._motion_owned = False
        self._pending = False
        self._pending_token = None
        self._pending_future = None
        self._pending_candidate = None
        self._attempted_goal_world = None
        self.active_goal = None
        self._active_token = None
        self._result_status = None
        self._result_future = None
        self.goal_started_at = None
        self._motion_deadline = None
        self.cancel_requested = False
        self.cancel_started_at = None
        self.cancel_reason = ""
        self._cancel_target = ""
        self._cancel_invoked = False
        self._cancel_future = None
        self._cancel_ack = False
        self.active_candidate = None

    def _clear_readiness_episode_locked(self):
        self.readiness_wait_started_at = None

    def _readiness_failure(
            self, run_generation, expected_state, expected_started_at, detail):
        """Apply the grace period to one continuous no-motion miss."""
        with self._lock:
            if not self._decision_matches_locked(
                    run_generation=run_generation,
                    expected_state=expected_state,
                    expected_started_at=expected_started_at):
                return
            now = self._monotonic()
            if self.readiness_wait_started_at is None:
                self.readiness_wait_started_at = now
            episode_started_at = self.readiness_wait_started_at
            if now - episode_started_at <= self.startup_grace:
                self.processed_map_version = -1
                self._set_state_locked("WAITING_READY", detail)
                return
        self._fault(
            "exploration readiness did not become valid: " + detail,
            expected_run_generation=run_generation,
            expected_state=expected_state,
            expected_started_at=expected_started_at)

    def _token_current_locked(self, run_generation, motion_generation):
        return (
            self._motion_owned
            and self._motion_token == (run_generation, motion_generation)
            and self.run_generation == run_generation
        )

    def _has_motion_locked(self):
        return self._motion_owned or self._pending or self.active_goal is not None

    def _start_callback(self, _request, response):
        with self._lock:
            if self.fault_latched or self.state == "FAULT":
                response.success = False
                response.message = "exploration is faulted; restart the explorer"
                return response
            if self.state not in ("STOPPED", "COMPLETE", "INCOMPLETE") or self._has_motion_locked():
                response.success = False
                response.message = "exploration is already active or cancelling"
                return response
            self.run_generation += 1
            self.started_at = self._monotonic()
            self.processed_map_version = -1
            self.blacklist.clear()
            self.failed_goal_worlds.clear()
            self.goal_failures = 0
            self.no_frontier_updates_seen = 0
            self.fault_requested = False
            self.readiness_wait_started_at = self.started_at
            # Do not clear an earlier cancellation record's event.  A stop
            # caller may still be returning its immutable recorded outcome.
            self._detach_cancel_record_locked()
            self._set_state_locked("WAITING_READY", "start accepted; waiting for readiness", force=True)
            response.success = True
            response.message = "exploration started"
            return response

    # ------------------------------------------------------------------
    # Receipt callbacks and readiness gates
    # ------------------------------------------------------------------

    def _map_callback(self, message):
        with self._lock:
            self.latest_map = message
            self.last_map_at = self._monotonic()
            self.map_version += 1
            if message.header.frame_id != "map":
                return

    def _costmap_callback(self, message):
        # Invalid costmaps are retained as the latest receipt so readiness
        # can reject them without creating an active-motion cancellation path.
        with self._lock:
            self.latest_costmap = message
            self.last_costmap_at = self._monotonic()
            self.costmap_version += 1

    def _base_callback(self, message):
        with self._lock:
            self.base_status = message
            self.last_base_status_at = self._monotonic()

    def _manipulator_callback(self, message):
        with self._lock:
            self.manipulator_status = message
            self.last_manipulator_status_at = self._monotonic()

    def _authority_ready_locked(self, now=None):
        if now is None:
            now = self._monotonic()
        base_status = self.base_status
        manipulator_status = self.manipulator_status
        base_received = self.last_base_status_at
        manipulator_received = self.last_manipulator_status_at
        base_ready = (
            base_status is not None
            and bool(getattr(base_status, "valid", False))
            and getattr(base_status, "state", None) == BaseStatus.READY)
        manip_ready = (
            manipulator_status is not None
            and bool(getattr(manipulator_status, "valid", False))
            and bool(getattr(manipulator_status, "base_motion_allowed", False))
            and getattr(manipulator_status, "state", None) in (
                ManipulatorStatus.STOWED_EMPTY, ManipulatorStatus.STOWED_LOADED))
        base_fresh = (
            base_received is not None
            and 0.0 <= now - base_received <= self.authority_timeout)
        manipulator_fresh = (
            manipulator_received is not None
            and 0.0 <= now - manipulator_received <= self.authority_timeout)
        return base_ready and manip_ready and base_fresh and manipulator_fresh

    def _authority_ready(self):
        with self._lock:
            return self._authority_ready_locked()

    @staticmethod
    def _valid_transform(transform, now_ros_ns, timeout_sec):
        """Return true only for a finite, timestamped, ROS-time-fresh TF."""
        try:
            stamp = transform.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if stamp_ns <= 0 or now_ros_ns < stamp_ns:
                return False
            if now_ros_ns - stamp_ns > int(timeout_sec * 1_000_000_000):
                return False
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            numbers = (
                translation.x, translation.y, translation.z,
                rotation.x, rotation.y, rotation.z, rotation.w)
            if not all(math.isfinite(float(value)) for value in numbers):
                return False
            quaternion_norm = math.sqrt(
                float(rotation.x) ** 2 + float(rotation.y) ** 2
                + float(rotation.z) ** 2 + float(rotation.w) ** 2)
            return quaternion_norm > 0.0
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    def _lookup_fresh_transform(self):
        try:
            query_time = self.get_clock().now()
            if query_time.nanoseconds <= 0:
                return None
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", query_time,
                timeout=Duration(seconds=self.tf_timeout))
        except (AttributeError, TypeError, ValueError, OverflowError,
                TransformException, RuntimeError):
            return None
        if not self._valid_transform(transform, self._now_ros_ns(), self.tf_timeout):
            return None
        return transform

    def _map_fresh(self):
        return self._map_readiness()[0]

    def _map_readiness(self):
        now = self._monotonic()
        with self._lock:
            grid = self.latest_map
            received_at = self.last_map_at
        if (grid is None or received_at is None
                or not 0.0 <= now - received_at <= self.map_timeout):
            return False, "fresh valid map is unavailable", None
        if occupancy_grid_geometry(grid) is None:
            return False, "fresh valid map is unavailable", None
        return True, "fresh valid map is available", grid

    def _costmap_readiness(self):
        now = self._monotonic()
        with self._lock:
            costmap = self.latest_costmap
            received_at = self.last_costmap_at
        if (costmap is None or received_at is None
                or not 0.0 <= now - received_at <= self.map_timeout):
            return False, "fresh valid global costmap is unavailable", None
        if costmap_geometry(costmap) is None:
            return False, "fresh valid global costmap is unavailable", None
        return True, "fresh valid global costmap is available", costmap

    def _readiness(
            self, require_action, require_costmap=True, require_transform=True):
        map_ready, map_detail, _grid = self._map_readiness()
        if not map_ready:
            return False, map_detail, None
        if require_costmap:
            costmap_ready, detail, _costmap = self._costmap_readiness()
            if not costmap_ready:
                return False, detail, None
        if not self._authority_ready():
            return False, "fresh base/manipulator motion authority is unavailable", None
        transform = None
        if require_transform:
            transform = self._lookup_fresh_transform()
            if transform is None:
                return False, "timestamped fresh map to base_footprint TF is unavailable", None
        if require_action:
            try:
                available = self.action_client.wait_for_server(timeout_sec=0.2)
            except Exception:  # pragma: no cover - middleware boundary
                available = False
            if not available:
                return False, "mission navigation action server is unavailable", None
        return True, "readiness gates passed", transform

    # ------------------------------------------------------------------
    # Lifecycle tick and planning
    # ------------------------------------------------------------------

    def _tick(self):
        with self._lock:
            state = self.state
            run_generation = self.run_generation
            motion_token = self._motion_token
            motion_deadline = self._motion_deadline
            cancel_requested = self.cancel_requested
            cancel_record = self._cancel_record
            cancel_deadline = (
                cancel_record.deadline if cancel_record is not None else None)
            started_at = self.started_at
            self._publish_status_locked()
        now = self._monotonic()

        if state == "FAULT":
            if (cancel_requested and cancel_record is not None
                    and cancel_deadline is not None and now >= cancel_deadline):
                with self._lock:
                    self._commit_cancel_timeout_locked(
                        cancel_record, run_generation, motion_token, state,
                        cancel_deadline,
                        "fault cancellation was not confirmed before timeout",
                        now=self._monotonic())
            return
        if state in ("STOPPED", "COMPLETE", "INCOMPLETE"):
            return
        if state == "CANCELLING":
            if (cancel_requested and cancel_record is not None
                    and cancel_deadline is not None and now >= cancel_deadline):
                with self._lock:
                    self._commit_cancel_timeout_locked(
                        cancel_record, run_generation, motion_token, state,
                        cancel_deadline,
                        "navigation cancellation was not confirmed before timeout",
                        now=self._monotonic())
            return
        if state in ("GOAL_PENDING", "NAVIGATING"):
            if motion_deadline is not None and now > motion_deadline and not cancel_requested:
                self._begin_cancel(
                    "navigation goal timed out",
                    expected_run_generation=run_generation,
                    expected_motion_token=motion_token,
                    expected_state=state,
                    expected_deadline=motion_deadline)
                return
            ready, detail, _transform = self._readiness(
                require_action=False, require_costmap=False)
            if not ready:
                self._begin_cancel(
                    "exploration readiness lost: " + detail,
                    expected_run_generation=run_generation,
                    expected_motion_token=motion_token,
                    expected_state=state,
                    expected_deadline=motion_deadline)
            return
        if state == "WAITING_READY":
            ready, detail, _transform = self._readiness(
                require_action=True, require_transform=True)
            if ready:
                with self._lock:
                    if self._decision_matches_locked(
                            run_generation=run_generation,
                            expected_state=state,
                            expected_started_at=started_at):
                        self._clear_readiness_episode_locked()
                        self._set_state_locked("SCANNING", "readiness gates passed")
                return
            self._readiness_failure(
                run_generation, state, started_at, detail)
            return
        if state != "SCANNING":
            return

        ready, detail, _transform = self._readiness(
            require_action=False, require_costmap=True,
            require_transform=False)
        if not ready:
            self._readiness_failure(
                run_generation, state, started_at, detail)
            return
        with self._lock:
            if not self._decision_matches_locked(
                    run_generation=run_generation,
                    expected_state=state,
                    expected_started_at=started_at):
                return
            if self.processed_map_version == self.map_version:
                return
            self.processed_map_version = self.map_version
            self._set_state_locked("PLANNING", "planning a frontier candidate")
        self._select_frontier(run_generation)

    def _finish_without_goal(
            self, run_generation, reason, complete=False, incomplete=False):
        with self._lock:
            if self.run_generation != run_generation or self.state != "PLANNING":
                return
            if incomplete:
                self._terminal_locked("INCOMPLETE", reason)
            elif complete:
                self._terminal_locked("COMPLETE", reason)
            else:
                self._set_state_locked("SCANNING", reason)

    def _planning_gate_failure(self, run_generation, started_at, detail):
        self._readiness_failure(
            run_generation, "PLANNING", started_at, detail)

    def _select_frontier(self, run_generation=None, transform=None):
        """Plan the current map and carry one post-cluster TF sample."""
        with self._lock:
            if run_generation is None:
                run_generation = self.run_generation
            if self.state != "PLANNING":
                return
            started_at = self.started_at
            failed_goal_worlds = tuple(self.failed_goal_worlds)
        map_ready, map_detail, _grid = self._map_readiness()
        if not map_ready:
            self._planning_gate_failure(run_generation, started_at, map_detail)
            return
        with self._lock:
            if not self._decision_matches_locked(
                    run_generation=run_generation,
                    expected_state="PLANNING",
                    expected_started_at=started_at):
                return
            grid = self.latest_map
            map_snapshot = (grid, self.map_version, self.last_map_at)
        if occupancy_grid_geometry(grid) is None:
            self._planning_gate_failure(
                run_generation, started_at, "fresh valid map is unavailable")
            return
        costmap_ready, costmap_detail, costmap = self._costmap_readiness()
        if not costmap_ready:
            self._planning_gate_failure(run_generation, started_at, costmap_detail)
            return
        with self._lock:
            costmap_snapshot = (
                costmap, self.costmap_version, self.last_costmap_at)
        try:
            if not any(value == 0 for value in grid.data):
                # An all-unknown initial map is not evidence that exploration
                # is complete; the robot has not yet established a mapped free cell.
                with self._lock:
                    if self._decision_matches_locked(
                            run_generation=run_generation,
                            expected_state="PLANNING",
                            expected_started_at=started_at):
                        self.no_frontier_updates_seen = 0
                self._finish_without_goal(run_generation, "waiting for a mapped free cell")
                return
        except Exception as exc:  # pragma: no cover - algorithm boundary
            self._fault(
                "frontier planning failed: " + str(exc),
                expected_run_generation=run_generation,
                expected_state="PLANNING",
                expected_started_at=started_at)
            return
        try:
            failed_goal_cells = set()
            for failed_world in failed_goal_worlds:
                failed_cell = frontier_world_cell(grid, failed_world)
                if failed_cell is not None:
                    failed_goal_cells.add(failed_cell)
            clusters = frontier_clusters(grid.info.width, grid.info.height, grid.data)
        except Exception as exc:  # pragma: no cover - algorithm boundary
            self._fault(
                "frontier planning failed: " + str(exc),
                expected_run_generation=run_generation,
                expected_state="PLANNING",
                expected_started_at=started_at)
            return

        if clusters is None:
            self._planning_gate_failure(
                run_generation, started_at, "fresh valid map is unavailable")
            return
        if not self._authority_ready():
            self._planning_gate_failure(
                run_generation, started_at,
                "fresh base/manipulator motion authority is unavailable")
            return

        if transform is None:
            transform = self._lookup_fresh_transform()
        if not self._valid_transform(
                transform, self._now_ros_ns(), self.tf_timeout):
            self._planning_gate_failure(
                run_generation, started_at,
                "timestamped fresh map to base_footprint TF is unavailable")
            return

        robot_world = (
            transform.transform.translation.x,
            transform.transform.translation.y)
        try:
            candidates = costmap_frontier_candidates(
                clusters, grid, costmap, failed_goal_cells,
                robot_world=robot_world,
                min_goal_distance=self.min_goal_distance,
                footprint=NAVIGATION_FOOTPRINT)
        except Exception as exc:  # pragma: no cover - algorithm boundary
            self._fault(
                "frontier planning failed: " + str(exc),
                expected_run_generation=run_generation,
                expected_state="PLANNING",
                expected_started_at=started_at)
            return

        if candidates is None:
            self._planning_gate_failure(run_generation, started_at, costmap_detail)
            return

        if not candidates:
            current_map_ready, current_map_detail, current_grid = (
                self._map_readiness())
            if not current_map_ready:
                self._planning_gate_failure(
                    run_generation, started_at, current_map_detail)
                return
            with self._lock:
                if (current_grid is not self.latest_map
                        or self.map_version != map_snapshot[1]):
                    self._discard_stale_plan_locked(
                        run_generation, started_at,
                        "frontier plan discarded because map evidence changed")
                    return
            with self._lock:
                if not self._decision_matches_locked(
                        run_generation=run_generation,
                        expected_state="PLANNING",
                        expected_started_at=started_at):
                    return
                self.no_frontier_updates_seen += 1
                limit_reached = self.no_frontier_updates_seen >= self.no_frontier_limit
                incomplete = limit_reached and bool(clusters)
            if incomplete:
                detail = "exploration incomplete: no safe costmap-valid frontier remains"
            elif limit_reached:
                detail = "exploration complete: no costmap-valid frontier remains"
            else:
                detail = "no costmap-valid frontier in this map update"
            self._finish_without_goal(
                run_generation, detail,
                complete=limit_reached and not incomplete,
                incomplete=incomplete)
            return

        try:
            if not self.action_client.wait_for_server(timeout_sec=0.2):
                self._planning_gate_failure(
                    run_generation, started_at,
                    "mission navigation action server is unavailable")
                return
        except Exception as exc:  # pragma: no cover - middleware boundary
            self._fault(
                "mission action readiness check failed: " + str(exc),
                expected_run_generation=run_generation,
                expected_state="PLANNING",
                expected_started_at=started_at)
            return

        costmap_ready, costmap_detail, _costmap = self._costmap_readiness()
        if not costmap_ready:
            self._planning_gate_failure(run_generation, started_at, costmap_detail)
            return

        gx, gy = candidates[0]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        stamp = self._now_ros_msg()
        if stamp is not None:
            pose.header.stamp = stamp
        goal_world = frontier_cell_world(grid, (gx, gy))
        if goal_world is None:
            self._fault(
                "frontier map geometry is invalid",
                expected_run_generation=run_generation,
                expected_state="PLANNING",
                expected_started_at=started_at)
            return
        pose.pose.position.x = goal_world[0]
        pose.pose.position.y = goal_world[1]
        pose.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._reserve_and_send(
            goal, (gx, gy), run_generation, expected_started_at=started_at,
            map_snapshot=map_snapshot, costmap_snapshot=costmap_snapshot,
            transform=transform, goal_world=goal_world)

    def _reservation_evidence_matches_locked(
            self, map_snapshot, costmap_snapshot, transform):
        try:
            expected_map, expected_map_version, expected_map_received_at = map_snapshot
            expected_costmap, expected_costmap_version, expected_costmap_received_at = (
                costmap_snapshot)
        except (TypeError, ValueError):
            return False, "frontier evidence snapshot is incomplete"

        if occupancy_grid_geometry(self.latest_map) is None:
            return False, "frontier plan discarded because fresh valid map is unavailable"
        now = self._monotonic()
        if (self.last_map_at is None
                or not 0.0 <= now - self.last_map_at <= self.map_timeout):
            return False, "frontier plan discarded because map evidence is stale"
        if (self.latest_map is not expected_map
                or self.map_version != expected_map_version):
            return False, "frontier plan discarded because map evidence changed"
        if self.last_map_at != expected_map_received_at:
            return False, "frontier plan discarded because map evidence changed"
        if (self.latest_costmap is not expected_costmap
                or self.costmap_version != expected_costmap_version):
            return False, "frontier plan discarded because costmap evidence changed"
        if (self.last_costmap_at != expected_costmap_received_at
                or expected_costmap_received_at is None
                or not 0.0 <= now - expected_costmap_received_at <= self.map_timeout
                or costmap_geometry(self.latest_costmap) is None):
            return False, "frontier plan discarded because costmap evidence is stale"
        if not self._authority_ready_locked(now):
            return False, "frontier plan discarded because motion authority changed"
        if not self._valid_transform(
                transform, self._now_ros_ns(), self.tf_timeout):
            return False, "frontier plan discarded because carried TF is stale"
        return True, ""

    def _discard_stale_plan_locked(self, run_generation, started_at, reason):
        if not self._decision_matches_locked(
                run_generation=run_generation, expected_state="PLANNING",
                expected_started_at=started_at):
            return
        self.processed_map_version = -1
        self._set_state_locked("SCANNING", reason)

    def _reserve_and_send(
            self, goal, candidate, run_generation, expected_started_at=_UNSET,
            map_snapshot=None, costmap_snapshot=None, transform=None,
            goal_world=None):
        with self._lock:
            if (not self._decision_matches_locked(
                    run_generation=run_generation,
                    expected_state="PLANNING",
                    expected_started_at=expected_started_at)
                    or self._has_motion_locked() or self.fault_latched):
                return
            evidence_ok, detail = self._reservation_evidence_matches_locked(
                map_snapshot, costmap_snapshot, transform)
            if not evidence_ok:
                if detail in (
                        "frontier plan discarded because fresh valid map is unavailable",
                        "frontier plan discarded because map evidence is stale"):
                    planning_started_at = (
                        self.started_at if expected_started_at is _UNSET
                        else expected_started_at)
                    self._planning_gate_failure(
                        run_generation, planning_started_at,
                        "fresh valid map is unavailable")
                else:
                    self._discard_stale_plan_locked(
                        run_generation, expected_started_at, detail)
                return
            self._clear_readiness_episode_locked()
            attempted_goal_world = self._goal_pose_world(goal)
            if attempted_goal_world is None:
                attempted_goal_world = self._validated_world_point(goal_world)
            if attempted_goal_world is None:
                attempted_goal_world = frontier_cell_world(
                    self.latest_map, candidate)
            self.no_frontier_updates_seen = 0
            self.motion_generation += 1
            token = (run_generation, self.motion_generation)
            self._motion_token = token
            self._motion_owned = True
            self._pending = True
            self._pending_token = token
            self._pending_candidate = candidate
            self.active_candidate = candidate
            self.active_goal = None
            self._active_token = None
            self._result_status = None
            self._result_future = None
            self.goal_started_at = self._monotonic()
            self._motion_deadline = self.goal_started_at + self.goal_timeout
            self.cancel_requested = False
            self.cancel_started_at = None
            self.cancel_reason = ""
            self._cancel_target = ""
            self._cancel_invoked = False
            self._cancel_future = None
            self._cancel_ack = False
            self._attempted_goal_world = attempted_goal_world
            # Every cancellation owns its own event; never clear a prior
            # record that a stop caller may still be observing.
            self._detach_cancel_record_locked()
            self._set_state_locked("GOAL_PENDING", "navigation goal pending acceptance", force=True)
            motion_deadline = self._motion_deadline

        try:
            future = self.action_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done, run=run_generation, motion=token[1]:
                self._goal_response(done, run, motion))
        except Exception as exc:  # pragma: no cover - middleware boundary
            self._fault(
                "navigation goal dispatch failed: " + str(exc),
                expected_run_generation=run_generation,
                expected_motion_token=token,
                expected_state="GOAL_PENDING",
                expected_deadline=motion_deadline,
                expected_started_at=expected_started_at)
            return
        with self._lock:
            if self._token_current_locked(*token) and self._pending:
                self._pending_future = future

    # ------------------------------------------------------------------
    # Action ownership and cancellation
    # ------------------------------------------------------------------

    @staticmethod
    def _future_token(explorer, run_generation, motion_generation):
        if run_generation is not None and motion_generation is not None:
            return run_generation, motion_generation
        with explorer._lock:
            token = explorer._motion_token
        return token if token is not None else (None, None)

    def _goal_response(self, future, run_generation=None, motion_generation=None):
        run_generation, motion_generation = self._future_token(
            self, run_generation, motion_generation)
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - middleware boundary
            with self._lock:
                current = self._token_current_locked(run_generation, motion_generation)
                expected_state = self.state
                expected_deadline = self._motion_deadline
                expected_started_at = self.started_at
            if current:
                self._fault(
                    "navigation goal dispatch failed: " + str(exc),
                    expected_run_generation=run_generation,
                    expected_motion_token=(run_generation, motion_generation),
                    expected_state=expected_state,
                    expected_deadline=expected_deadline,
                    expected_started_at=expected_started_at)
            return

        stale_accepted = False
        cancel_handle = None
        with self._lock:
            current = self._token_current_locked(run_generation, motion_generation)
            if not current:
                stale_accepted = bool(goal_handle and getattr(goal_handle, "accepted", False))
            elif not goal_handle or not getattr(goal_handle, "accepted", False):
                attempted_goal_world = self._attempted_goal_world
                self._pending = False
                self._pending_future = None
                was_cancel = self.cancel_requested
                reason = self.cancel_reason
                cancel_record = self._cancel_record
                if self.fault_latched:
                    self._complete_cancellation_locked(
                        cancel_record, "FAULT",
                        "navigation goal was rejected after fault", False,
                        "explorer faulted during cancellation")
                elif was_cancel:
                    if reason == "operator requested exploration stop":
                        self._complete_cancellation_locked(
                            cancel_record, "STOPPED",
                            "operator stop completed before goal acceptance", True,
                            "exploration stopped")
                    else:
                        self._complete_cancellation_locked(
                            cancel_record, "FAULT",
                            "navigation goal was rejected during cancellation", False,
                            "explorer faulted during cancellation")
                else:
                    self._record_goal_failure_locked(
                        "navigation goal was rejected",
                        attempted_goal_world=attempted_goal_world)
            else:
                self._pending = False
                self._pending_future = None
                self.active_goal = goal_handle
                self._active_token = (run_generation, motion_generation)
                if self.cancel_requested or self.fault_latched:
                    self._cancel_target = "accepted"
                    if not self.fault_latched:
                        self._set_state_locked("CANCELLING", self.cancel_reason, force=True)
                    cancel_handle = goal_handle
                else:
                    self._set_state_locked("NAVIGATING", "navigation goal accepted", force=True)

        if stale_accepted:
            # A response from an older run must never alter this run.  Best
            # effort cancellation only removes the stale action handle.
            try:
                goal_handle.cancel_goal_async()
            except Exception:  # pragma: no cover - middleware boundary
                pass
            return
        if not current or not goal_handle or not getattr(goal_handle, "accepted", False):
            return
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done, run=run_generation, motion=motion_generation:
                self._goal_result(done, run, motion))
        except Exception as exc:  # pragma: no cover - middleware boundary
            with self._lock:
                current = self._token_current_locked(run_generation, motion_generation)
                expected_state = self.state
                expected_deadline = self._motion_deadline
                expected_started_at = self.started_at
            if current:
                self._fault(
                    "navigation result subscription failed: " + str(exc),
                    expected_run_generation=run_generation,
                    expected_motion_token=(run_generation, motion_generation),
                    expected_state=expected_state,
                    expected_deadline=expected_deadline,
                    expected_started_at=expected_started_at)
            return
        with self._lock:
            if self._token_current_locked(run_generation, motion_generation):
                self._result_future = result_future
        if cancel_handle is not None:
            self._issue_cancel((run_generation, motion_generation), cancel_handle)

    def _goal_result(self, future, run_generation=None, motion_generation=None):
        run_generation, motion_generation = self._future_token(
            self, run_generation, motion_generation)
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:  # pragma: no cover - middleware boundary
            with self._lock:
                current = self._token_current_locked(run_generation, motion_generation)
                expected_state = self.state
                expected_deadline = self._motion_deadline
                expected_started_at = self.started_at
            if current:
                self._fault(
                    "navigation result was unavailable: " + str(exc),
                    expected_run_generation=run_generation,
                    expected_motion_token=(run_generation, motion_generation),
                    expected_state=expected_state,
                    expected_deadline=expected_deadline,
                    expected_started_at=expected_started_at)
            return

        with self._lock:
            if not self._token_current_locked(run_generation, motion_generation):
                return
            self._result_status = status
            if self.cancel_requested:
                self._apply_cancel_proof_locked()
                return
            candidate = self.active_candidate
            attempted_goal_world = self._attempted_goal_world
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._clear_motion_locked()
                self._clear_readiness_episode_locked()
                self.goal_failures = 0
                self._set_state_locked("SCANNING", "navigation goal succeeded")
            else:
                self._record_goal_failure_locked(
                    "navigation goal ended with status %s" % status,
                    candidate=candidate,
                    attempted_goal_world=attempted_goal_world)

    def _record_goal_failure_locked(
            self, detail, candidate=None, attempted_goal_world=_UNSET):
        if candidate is None:
            candidate = self.active_candidate
        if attempted_goal_world is _UNSET:
            attempted_goal_world = self._attempted_goal_world
        attempted_goal_world = self._validated_world_point(attempted_goal_world)
        self._clear_motion_locked()
        self.goal_failures += 1
        if attempted_goal_world is not None:
            self.failed_goal_worlds.add(attempted_goal_world)
        if self.goal_failures >= self.max_goal_failures:
            self._terminal_locked("FAULT", detail + "; maximum navigation failures exceeded")
            return
        try:
            self.get_logger().warning(detail)
        except AttributeError:
            pass
        self._set_state_locked("SCANNING", detail)

    def _record_goal_failure(self, detail):
        with self._lock:
            self._record_goal_failure_locked(detail)

    def _apply_cancel_proof_locked(self):
        """Apply cancellation proof without releasing an unproven motion."""
        if not self.cancel_requested or self._cancel_record is None:
            return
        status = self._result_status
        if status is None:
            return
        if status == GoalStatus.STATUS_CANCELED:
            if self._cancel_ack:
                self._finish_cancel_locked()
            else:
                # CANCELED is the expected result, but the positive cancel
                # acknowledgement is still required before ownership release.
                self._publish_status_locked()
            return
        if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
            if self._cancel_ack:
                self._complete_cancellation_locked(
                    self._cancel_record, "FAULT",
                    "navigation cancellation did not reach terminal canceled state",
                    False, "explorer faulted during cancellation")
            else:
                self._terminal_fault_with_motion_locked(
                    "navigation cancellation did not reach terminal canceled state")
            return
        # UNKNOWN, ACCEPTED, EXECUTING, and CANCELING are not terminal proof.
        # Fail closed, but retain the motion until a later proof completes the
        # identity or the original cancellation deadline is committed.
        self._terminal_fault_with_motion_locked(
            "navigation cancellation result was not terminal")

    def _begin_cancel(
            self, reason, expected_run_generation=None,
            expected_motion_token=_UNSET, expected_state=None,
            expected_deadline=_UNSET, expected_started_at=_UNSET):
        handle = None
        token = None
        missing_motion = False
        with self._lock:
            if not self._decision_matches_locked(
                    run_generation=expected_run_generation,
                    motion_token=expected_motion_token,
                    expected_state=expected_state,
                    expected_deadline=expected_deadline,
                    expected_started_at=expected_started_at):
                return
            token = self._motion_token
            if not self._motion_owned or token is None:
                missing_motion = True
            elif not self.cancel_requested:
                self.cancel_requested = True
                self._new_cancel_record_locked(token)
                self.cancel_reason = reason
                self._cancel_target = "accepted" if self.active_goal is not None else "pending"
                self._cancel_invoked = False
                self._cancel_future = None
                self._cancel_ack = False
                if not self.fault_latched:
                    self._set_state_locked("CANCELLING", reason, force=True)
                else:
                    self._publish_status_locked()
            elif self._cancel_record is None:
                self._new_cancel_record_locked(token, self.cancel_started_at)
            if self._motion_owned and self.active_goal is not None and not self._cancel_invoked:
                handle = self.active_goal
        if missing_motion:
            self._fault(reason + "; no pending or active goal to cancel")
        elif handle is not None:
            self._issue_cancel(token, handle)

    def _request_cancel(self, reason):
        self._begin_cancel(reason)

    def _issue_cancel(self, token, handle):
        with self._lock:
            if (not self._token_current_locked(*token)
                    or not self.cancel_requested
                    or self.active_goal is not handle
                    or self._cancel_invoked):
                return
            self._cancel_invoked = True
            cancel_record = self._cancel_record
            cancel_deadline = (
                cancel_record.deadline if cancel_record is not None else _UNSET)
        try:
            future = handle.cancel_goal_async()
            future.add_done_callback(
                lambda done, run=token[0], motion=token[1]:
                self._cancel_response(done, run, motion))
        except Exception as exc:  # pragma: no cover - middleware boundary
            self._terminal_fault(
                "navigation cancellation request failed: " + str(exc),
                expected_run_generation=token[0],
                expected_motion_token=token,
                expected_state="CANCELLING",
                expected_cancel_record=cancel_record,
                expected_cancel_deadline=cancel_deadline)
            return
        with self._lock:
            if self._token_current_locked(*token):
                self._cancel_future = future

    def _cancel_response(self, future, run_generation, motion_generation):
        try:
            result = future.result()
            accepted = bool(getattr(result, "goals_canceling", ()))
            if hasattr(result, "accepted"):
                accepted = accepted or bool(result.accepted)
        except Exception as exc:  # pragma: no cover - middleware boundary
            accepted = False
            error = str(exc)
        else:
            error = ""
        with self._lock:
            if not self._token_current_locked(run_generation, motion_generation):
                return
            if not accepted:
                self._terminal_fault_with_motion_locked(
                    "navigation cancellation was rejected" + (": " + error if error else ""))
                return
            self._cancel_ack = True
            self._publish_status_locked()
            self._apply_cancel_proof_locked()

    def _finish_cancel_locked(self):
        if self._result_status != GoalStatus.STATUS_CANCELED or not self._cancel_ack:
            return
        cancel_record = self._cancel_record
        operator_stop = self.cancel_reason == "operator requested exploration stop"
        faulted = self.fault_latched or not operator_stop
        if faulted:
            self._complete_cancellation_locked(
                cancel_record, "FAULT",
                "navigation cancellation completed after a fault", False,
                "explorer faulted during cancellation")
        else:
            self._complete_cancellation_locked(
                cancel_record, "STOPPED", "operator stop completed", True,
                "exploration stopped")

    def _terminal_fault_with_motion_locked(self, reason):
        cancel_record = self._cancel_record if self.cancel_requested else None
        was_fault_latched = self.fault_latched
        already_faulted = self.state == "FAULT" and self.fault_latched
        if cancel_record is not None:
            self._set_cancel_outcome_locked(
                cancel_record, False, "explorer faulted during cancellation")
        self.fault_requested = True
        self.fault_latched = True
        if not already_faulted:
            self.state = "FAULT"
            if not was_fault_latched:
                self.reason = reason
        elif cancel_record is None or cancel_record.outcome is None:
            self.reason = reason
        self._publish_status_locked()
        self._signal_cancel_record_locked(cancel_record)

    def _terminal_fault(
            self, reason, expected_run_generation=None,
            expected_motion_token=_UNSET, expected_state=None,
            expected_deadline=_UNSET, expected_started_at=_UNSET,
            expected_cancel_record=_UNSET, expected_cancel_deadline=_UNSET):
        with self._lock:
            if not self._decision_matches_locked(
                    run_generation=expected_run_generation,
                    motion_token=expected_motion_token,
                    expected_state=expected_state,
                    expected_deadline=expected_deadline,
                    expected_started_at=expected_started_at,
                    expected_cancel_record=expected_cancel_record,
                    expected_cancel_deadline=expected_cancel_deadline):
                return
            self._terminal_fault_with_motion_locked(reason)

    # ------------------------------------------------------------------
    # Stop and fail-closed fault paths
    # ------------------------------------------------------------------

    def _stop_callback(self, _request, response):
        handle = None
        token = None
        cancel_record = None
        with self._lock:
            cancel_record = self._applicable_cancel_record_locked()
            if cancel_record is not None and cancel_record.outcome is not None:
                response.success, response.message = cancel_record.outcome
                return response
            if cancel_record is None and (self.state == "FAULT" or self.fault_latched):
                response.success = False
                response.message = "exploration is faulted; restart the explorer"
                return response
            if cancel_record is None and self.state in (
                    "STOPPED", "COMPLETE", "INCOMPLETE") and not self._has_motion_locked():
                if self.state == "COMPLETE":
                    self._terminal_locked("STOPPED", "operator stop acknowledged after completion")
                response.success = True
                response.message = (
                    "exploration remains incomplete; no active navigation goal"
                    if self.state == "INCOMPLETE"
                    else "exploration already stopped")
                return response
            if cancel_record is None and not self._has_motion_locked():
                self._terminal_locked("STOPPED", "operator stop completed before dispatch")
                response.success = True
                response.message = "exploration stopped with no active navigation goal"
                return response
            if cancel_record is None:
                token = self._motion_token
                if not self.cancel_requested:
                    self.cancel_requested = True
                    cancel_record = self._new_cancel_record_locked(
                        token)
                    self.cancel_reason = "operator requested exploration stop"
                    self._cancel_target = "accepted" if self.active_goal is not None else "pending"
                    self._cancel_invoked = False
                    self._cancel_future = None
                    self._cancel_ack = False
                    self._set_state_locked("CANCELLING", self.cancel_reason, force=True)
                else:
                    cancel_record = self._cancel_record
                    if cancel_record is None:
                        cancel_record = self._new_cancel_record_locked(
                            token, self.cancel_started_at)
            else:
                token = cancel_record.token
            if self.active_goal is not None and not self._cancel_invoked:
                handle = self.active_goal
        if handle is not None:
            self._issue_cancel(token, handle)

        while True:
            with self._lock:
                outcome = cancel_record.outcome
                if outcome is not None:
                    response.success, response.message = outcome
                    return response
                if self._applicable_cancel_record_locked() is not cancel_record:
                    # The record was detached by a newer run or motion.  The
                    # old caller must not mutate that newer identity.
                    response.success = False
                    response.message = "cancellation outcome was unavailable"
                    return response
                now = self._monotonic()
                remaining = cancel_record.deadline - now
                if remaining <= 0.0:
                    timeout_reason = (
                        "fault cancellation was not confirmed before timeout"
                        if self.fault_latched or self.state == "FAULT"
                        else "operator stop cancellation was not confirmed before timeout")
                    outcome = self._commit_cancel_timeout_locked(
                        cancel_record, cancel_record.token[0], cancel_record.token,
                        self.state, cancel_record.deadline, timeout_reason, now=now)
                    if outcome is not None:
                        response.success, response.message = outcome
                        return response
                    # A failed authority proof is intentionally not converted
                    # into a synthetic event or outcome.
                    response.success = False
                    response.message = "cancellation outcome was unavailable"
                    return response
            # Wait outside the lock.  An early wakeup is only a recheck; the
            # original record deadline is never extended or replaced.
            cancel_record.event.wait(timeout=remaining)

    def _fault(
            self, detail, expected_run_generation=None,
            expected_motion_token=_UNSET, expected_state=None,
            expected_deadline=_UNSET, expected_started_at=_UNSET,
            expected_cancel_record=_UNSET, expected_cancel_deadline=_UNSET):
        handle = None
        token = None
        with self._lock:
            if not self._decision_matches_locked(
                    run_generation=expected_run_generation,
                    motion_token=expected_motion_token,
                    expected_state=expected_state,
                    expected_deadline=expected_deadline,
                    expected_started_at=expected_started_at,
                    expected_cancel_record=expected_cancel_record,
                    expected_cancel_deadline=expected_cancel_deadline):
                return
            if self.fault_latched and self.state == "FAULT":
                if self._cancel_record is None or self._cancel_record.outcome is None:
                    self.reason = detail
                    self._publish_status_locked()
                return
            if not self._motion_owned:
                self.fault_requested = True
                self.fault_latched = True
                self._terminal_locked("FAULT", detail)
                return
            self.fault_requested = True
            self.fault_latched = True
            self.reason = detail
            self.cancel_reason = "fault"
            if not self.cancel_requested:
                self.cancel_requested = True
                self._new_cancel_record_locked(self._motion_token)
                self._cancel_target = "accepted" if self.active_goal is not None else "pending"
                self._cancel_invoked = False
                self._cancel_future = None
                self._cancel_ack = False
            elif self._cancel_record is None:
                self._new_cancel_record_locked(self._motion_token, self.cancel_started_at)
            self.state = "CANCELLING"
            token = self._motion_token
            if self.active_goal is not None and not self._cancel_invoked:
                handle = self.active_goal
            self._publish_status_locked()
        if handle is not None:
            self._issue_cancel(token, handle)


def main():
    rclpy.init()
    node = FrontierExplorer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
