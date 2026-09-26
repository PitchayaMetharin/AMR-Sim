#!/usr/bin/env python3
"""Fail-closed frontier exploration through the AMR mission action boundary."""

from copy import deepcopy
from contextlib import contextmanager
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
from rclpy.executors import (
    ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_algorithm import (
    NAVIGATION_FOOTPRINT, _goal_distance_is_valid, _route_start_proof,
    costmap_frontier_candidates, frontier_cell_world, frontier_clusters,
    costmap_geometry, frontier_world_cell, occupancy_grid_geometry)


_UNSET = object()
SLAM_MAP_ODOM_FUTURE_TOLERANCE_SEC = 1.0
MISSION_STATUS_DIAGNOSTIC = "amr_mission/mission_supervisor"
SAFE_REACHABLE_COMPLETION_POLICY = "SAFE_REACHABLE_AREA_V1"
FRONTIER_BLOCKED_CLASSES = frozenset(("BLOCKED_SAFETY", "BLOCKED_ROUTE"))
MAX_BLOCKAGE_ATTEMPTS = 3
RECOVERY_YAW_TOLERANCE_RAD = math.radians(1.0)
MISSION_FAULT_CLASSES = frozenset((
    "NONE", "OBSTACLE_BLOCKAGE", "PLANNER_ABORT", "SMOOTHER_ABORT",
    "CONTROLLER_ABORT", "CANCELLATION", "NAVIGATION_FAULT"))
MISSION_STAGES = frozenset((
    "PLANNING", "SMOOTHING", "FOLLOWING", "CANCELING", "TERMINAL"))
MISSION_OUTCOMES = frozenset((
    "PENDING", "SUCCEEDED", "CANCELED", "ABORTED", "FAULT"))


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
        self.declare_parameter("runtime_diagnostics", False)
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
        self.runtime_diagnostics = _parameter_bool(
            self.get_parameter("runtime_diagnostics").value, "runtime_diagnostics")

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
        self.mission_status_sub = self.create_subscription(
            DiagnosticArray, "/amr/mission/status",
            self._mission_status_callback, 10)
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
        # Route selection is intentionally CPU-bound Python work.  Keep TF
        # receipt independent of that callback so the final reservation proof
        # does not validate a sample that the local buffer received seconds
        # earlier even though upstream TF is current.
        self._tf_listener_node = Node(
            "frontier_explorer_tf_listener_%x" % id(self),
            context=self.context,
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.tf_listener = TransformListener(self.tf_buffer, self._tf_listener_node)
        self._tf_listener_executor = SingleThreadedExecutor(context=self.context)
        self._tf_listener_executor.add_node(self._tf_listener_node)
        self._tf_listener_thread = threading.Thread(
            target=self._spin_tf_listener,
            name="frontier-explorer-tf-listener",
            daemon=True,
        )
        self._tf_listener_thread.start()

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
        self._runtime_trace_sequence = 0
        self._runtime_trace_events = []

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
        self._expected_goal_uuid = None
        self._mission_status_by_uuid = {}
        self._mission_status_invalid_at = None
        self._blocked_destinations = {}
        self._retry_exhausted_destinations = set()
        self._recovery_goal_world = None
        self._recovery_stationary_sample = None
        self._recovery_stationary_samples = 0
        self.reached_goal_count = 0
        self.raw_frontier_count = 0
        self.blocked_frontier_count = 0
        self.blocked_safety_count = 0
        self.blocked_route_count = 0
        self.unresolved_frontier_count = 0
        self._mission_blockage_confirmed = False
        self._mission_fault_class = "NONE"
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

    def _spin_tf_listener(self):
        try:
            self._tf_listener_executor.spin()
        except ExternalShutdownException:
            pass

    def _stop_tf_listener(self):
        listener = getattr(self, "tf_listener", None)
        listener_node = getattr(self, "_tf_listener_node", None)
        executor = getattr(self, "_tf_listener_executor", None)
        thread = getattr(self, "_tf_listener_thread", None)
        if listener is not None:
            try:
                listener.unregister()
            except (AttributeError, RuntimeError):
                pass
        if executor is not None:
            if listener_node is not None:
                try:
                    executor.remove_node(listener_node)
                except (AttributeError, RuntimeError):
                    pass
            try:
                executor.shutdown(timeout_sec=1.0)
            except (AttributeError, RuntimeError):
                pass
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        if listener_node is not None:
            try:
                listener_node.destroy_node()
            except (AttributeError, RuntimeError):
                pass

    def destroy_node(self):
        self._stop_tf_listener()
        return super().destroy_node()

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
    def _receipt_age(now, received_at):
        if received_at is None:
            return None
        try:
            return float(now - received_at)
        except (TypeError, ValueError, OverflowError):
            return None

    def _trace_evidence_snapshot(self, wall_now):
        return {
            "map_version": getattr(self, "map_version", None),
            "costmap_version": getattr(self, "costmap_version", None),
            "base_receipt_age_sec": self._receipt_age(
                wall_now, getattr(self, "last_base_status_at", None)),
            "manipulator_receipt_age_sec": self._receipt_age(
                wall_now, getattr(self, "last_manipulator_status_at", None)),
        }

    @staticmethod
    def _trace_value(value):
        if isinstance(value, bool):
            return str(value).lower()
        return str(value)

    def _publish_runtime_trace(self, event):
        """Publish one opt-in, machine-readable timing/evidence event."""
        publisher = getattr(self, "status_pub", None)
        if publisher is None:
            return
        message = DiagnosticArray()
        now = self._now_ros_msg()
        if now is not None:
            message.header.stamp = now
        diagnostic = DiagnosticStatus()
        diagnostic.name = "amr_exploration/runtime_trace"
        diagnostic.level = (
            DiagnosticStatus.OK
            if event.get("outcome") in ("ok", "available", "accepted", "reserved")
            else DiagnosticStatus.WARN)
        diagnostic.message = "%s: %s" % (
            event.get("phase", "unknown"), event.get("outcome", "unknown"))
        for key, value in sorted(event.items()):
            item = KeyValue()
            item.key = str(key)
            item.value = self._trace_value(value)
            diagnostic.values.append(item)
        message.status.append(diagnostic)
        publisher.publish(message)

    @contextmanager
    def _trace_span(self, phase, generation=None, **initial):
        """Record an opt-in phase span without changing the control path."""
        if not getattr(self, "runtime_diagnostics", False):
            yield None
            return

        wall_before = self._monotonic()
        ros_before = self._now_ros_ns()
        event = {
            "phase": phase,
            "generation": (
                getattr(self, "run_generation", None)
                if generation is None else generation),
            "outcome": "ok",
        }
        event.update(initial)
        for key, value in self._trace_evidence_snapshot(wall_before).items():
            event["%s_before" % key] = value
        event["wall_before_sec"] = wall_before
        event["ros_before_ns"] = ros_before
        try:
            yield event
        except Exception as exc:
            event["outcome"] = "exception"
            event["error"] = str(exc)
            raise
        finally:
            wall_after = self._monotonic()
            ros_after = self._now_ros_ns()
            for key, value in self._trace_evidence_snapshot(wall_after).items():
                event["%s_after" % key] = value
            event["wall_after_sec"] = wall_after
            event["wall_duration_sec"] = wall_after - wall_before
            event["ros_after_ns"] = ros_after
            event["ros_duration_ns"] = ros_after - ros_before
            self._runtime_trace_sequence = (
                getattr(self, "_runtime_trace_sequence", 0) + 1)
            event["sequence"] = self._runtime_trace_sequence
            self._runtime_trace_events = list(
                getattr(self, "_runtime_trace_events", ())) + [dict(event)]
            # Trace publication is observational.  A diagnostics transport
            # failure must not change admission, cancellation, or fault state.
            try:
                self._publish_runtime_trace(event)
            except Exception:  # pragma: no cover - middleware boundary
                pass

    @staticmethod
    def _header_stamp_ns(message):
        """Return a header stamp, or distinguish unavailable from malformed."""
        try:
            header = message.header
        except AttributeError:
            return None
        if header is None:
            return _UNSET
        try:
            stamp = header.stamp
        except AttributeError:
            return None
        if stamp is None:
            return _UNSET
        try:
            sec_value = stamp.sec
            nanosec_value = stamp.nanosec
        except AttributeError:
            return None
        if isinstance(sec_value, bool) or isinstance(nanosec_value, bool):
            return _UNSET
        try:
            sec_float = float(sec_value)
            nanosec_float = float(nanosec_value)
        except (TypeError, ValueError, OverflowError):
            return _UNSET
        if (not math.isfinite(sec_float)
                or not math.isfinite(nanosec_float)
                or not sec_float.is_integer()
                or not nanosec_float.is_integer()):
            return _UNSET
        sec = int(sec_float)
        nanosec = int(nanosec_float)
        if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
            return _UNSET
        return sec * 1_000_000_000 + nanosec

    def _message_is_fresh(self, message, received_at, max_age_sec, now=None):
        """Check ROS header age, falling back to the steady receipt age."""
        if message is None or received_at is None:
            return False
        if now is None:
            now = self._monotonic()
        receipt_fresh = 0.0 <= now - received_at <= max_age_sec
        stamp_ns = self._header_stamp_ns(message)
        if stamp_ns is _UNSET:
            return False
        ros_now_ns = self._now_ros_ns()
        if stamp_ns is None or stamp_ns == 0 or ros_now_ns <= 0:
            return receipt_fresh
        age_ns = ros_now_ns - stamp_ns
        return 0 <= age_ns <= int(max_age_sec * 1_000_000_000)

    def _authority_message_is_fresh(
            self, message, received_at, max_age_sec, now=None):
        """Check the live receipt TTL for an authority status stream.

        BaseStatus and ManipulatorStatus headers describe the observation that
        produced the status.  They are not the Explorer's liveness clock: in
        the portable runtime the publisher and Explorer can observe the same
        ROS clock on adjacent executor turns, making a freshly received
        status header briefly appear future-dated here.  Authority therefore
        requires a recent steady-clock receipt, while the semantic status
        gates remain enforced by ``_authority_ready_locked``.
        """
        if message is None or received_at is None:
            return False
        if now is None:
            now = self._monotonic()
        return 0.0 <= now - received_at <= max_age_sec

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
    def _world_identity(world):
        """Use a finite, stable world-coordinate bucket for blockage history."""
        world = FrontierExplorer._validated_world_point(world)
        if world is None:
            return None
        # Five centimetres is below the footprint scale and keeps equivalent
        # cell-center goals together while allowing genuinely different goals.
        return tuple(int(math.floor(value / 0.05 + 0.5)) for value in world)

    @staticmethod
    def _route_evidence_fingerprint(grid, costmap):
        """Fingerprint only map/costmap geometry and content.

        Receipt counters and all message timestamps are deliberately omitted;
        the fingerprint changes only when route evidence itself changes.
        """
        try:
            map_geometry = occupancy_grid_geometry(grid)
            costmap_geometry_value = costmap_geometry(costmap)
            if map_geometry is None or costmap_geometry_value is None:
                return None
            return (
                (grid.header.frame_id, tuple(map_geometry), tuple(grid.data)),
                (costmap.header.frame_id, tuple(costmap_geometry_value),
                 tuple(costmap.data)),
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    def _blocked_worlds_for_fingerprint_locked(self, fingerprint):
        blocked = []
        for record in getattr(self, "_blocked_destinations", {}).values():
            if (record.get("attempts", 0) >= MAX_BLOCKAGE_ATTEMPTS
                    or fingerprint in record.get("fingerprints", set())):
                world = self._validated_world_point(record.get("world"))
                if world is not None:
                    blocked.append(world)
        return tuple(blocked)

    def _record_blockage_locked(self, world, fingerprint):
        world = self._validated_world_point(world)
        identity = self._world_identity(world)
        if identity is None:
            return
        destinations = getattr(self, "_blocked_destinations", None)
        if destinations is None:
            destinations = {}
            self._blocked_destinations = destinations
        record = destinations.setdefault(
            identity, {"world": world, "fingerprints": set(), "attempts": 0})
        fingerprints = record.setdefault("fingerprints", set())
        if fingerprint not in fingerprints:
            if record.get("attempts", 0) >= MAX_BLOCKAGE_ATTEMPTS:
                return
            fingerprints.add(fingerprint)
            record["attempts"] = int(record.get("attempts", 0)) + 1
        if record.get("attempts", 0) >= MAX_BLOCKAGE_ATTEMPTS:
            getattr(self, "_retry_exhausted_destinations", set()).add(identity)

    @staticmethod
    def _goal_pose_world(goal):
        try:
            position = goal.pose.pose.position
            return FrontierExplorer._validated_world_point(
                (position.x, position.y))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    @staticmethod
    def _transform_world_yaw(transform):
        try:
            translation = transform.transform.translation
            world = FrontierExplorer._validated_world_point(
                (translation.x, translation.y))
            if world is None:
                return None
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                rotation.w * rotation.w + rotation.x * rotation.x
                - rotation.y * rotation.y - rotation.z * rotation.z)
            if not math.isfinite(yaw):
                return None
            return world, yaw
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    @staticmethod
    def _canonical_goal_uuid(value):
        """Return the action UUID as the lowercase 32-character hex form."""
        try:
            raw = getattr(value, "uuid", value)
            if isinstance(raw, str):
                text = raw.replace("-", "").strip().lower()
                if (len(text) != 32
                        or any(character not in "0123456789abcdef" for character in text)):
                    return None
                return text
            data = bytes(raw)
            if len(data) != 16:
                return None
            return data.hex()
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    @staticmethod
    def _diagnostic_fields(diagnostic):
        fields = {}
        try:
            for item in diagnostic.values:
                key = str(item.key)
                value = str(item.value)
                if not key or key in fields:
                    return None
                fields[key] = value
        except (AttributeError, TypeError, ValueError):
            return None
        return fields

    def _mission_status_callback(self, message):
        """Retain only structured mission evidence; it never authorizes motion."""
        received_at = self._monotonic()
        try:
            diagnostics = tuple(message.status)
        except (AttributeError, TypeError):
            with self._lock:
                self._mission_status_invalid_at = received_at
            return
        matching = [
            diagnostic for diagnostic in diagnostics
            if getattr(diagnostic, "name", None) == MISSION_STATUS_DIAGNOSTIC]
        with self._lock:
            if not matching:
                return
            diagnostic = matching[-1]
            fields = self._diagnostic_fields(diagnostic)
            goal_uuid = (
                self._canonical_goal_uuid(fields.get("goal_uuid"))
                if fields is not None else None)
            required = (
                "goal_uuid", "stage", "outcome", "reason",
                "blockage_confirmed", "fault_class")
            valid = (
                fields is not None
                and all(key in fields for key in required)
                and goal_uuid is not None
                and fields.get("stage") in MISSION_STAGES
                and fields.get("outcome") in MISSION_OUTCOMES
                and fields.get("fault_class") in MISSION_FAULT_CLASSES
                and bool(fields.get("reason"))
                and fields.get("blockage_confirmed") in ("true", "false"))
            record = {
                "received_at": received_at,
                "valid": bool(valid),
                "goal_uuid": goal_uuid,
                "stage": fields.get("stage", "") if fields is not None else "",
                "outcome": fields.get("outcome", "") if fields is not None else "",
                "reason": fields.get("reason", "") if fields is not None else "",
                "blockage_confirmed": (
                    fields.get("blockage_confirmed") == "true"
                    if fields is not None else False),
                "fault_class": fields.get("fault_class", "") if fields is not None else "",
            }
            if goal_uuid is None:
                self._mission_status_invalid_at = received_at
            else:
                self._mission_status_by_uuid[goal_uuid] = record

    def _mission_status_max_age(self):
        return max(
            1.0,
            float(getattr(self, "map_timeout", 1.0)),
            float(getattr(self, "tf_timeout", 1.0)),
            float(getattr(self, "authority_timeout", 1.0)))

    def _matching_mission_terminal_locked(self):
        expected_uuid = getattr(self, "_expected_goal_uuid", None)
        if expected_uuid is None:
            return None, "accepted navigation goal UUID is unavailable"
        record = getattr(self, "_mission_status_by_uuid", {}).get(expected_uuid)
        if record is None:
            return None, "matching mission terminal status is missing"
        now = self._monotonic()
        received_at = record.get("received_at")
        if (received_at is None
                or now < received_at
                or now - received_at > self._mission_status_max_age()):
            return None, "matching mission terminal status is stale"
        started_at = getattr(self, "goal_started_at", None)
        if started_at is not None and received_at < started_at:
            return None, "matching mission terminal status predates the action"
        if not record.get("valid", False):
            return None, "matching mission terminal status is malformed"
        if record.get("stage") != "TERMINAL":
            return None, "matching mission status is not terminal"
        return record, ""

    @staticmethod
    def _is_recoverable_planner_abort(status, mission_record):
        """Recognize only a completed no-path result as a safe deferral."""
        return (
            status == GoalStatus.STATUS_ABORTED
            and mission_record is not None
            and mission_record.get("outcome") == "FAULT"
            and mission_record.get("fault_class") == "PLANNER_ABORT"
            and mission_record.get("reason") == "global planning failed"
            and mission_record.get("blockage_confirmed") is False)

    @staticmethod
    def _mission_stage_for_state(state):
        if state in FrontierExplorer._TERMINAL_STATES:
            return "TERMINAL"
        if state in ("GOAL_PENDING", "WAITING_READY", "SCANNING", "PLANNING"):
            return "PLANNING"
        if state == "RECOVERY_WAIT":
            return "RECOVERY_WAIT"
        if state == "CANCELLING":
            return "CANCELING"
        if state == "NAVIGATING":
            return "FOLLOWING"
        return str(state)

    def _mission_outcome_for_state(self):
        if self.state == "COMPLETE":
            return "SUCCEEDED"
        if self.state == "STOPPED":
            return "CANCELED"
        if self.state == "INCOMPLETE":
            return "ABORTED"
        if self.state == "FAULT":
            return "FAULT"
        return "PENDING"

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
            "cancel_owned_motion": self.cancel_requested and self._motion_owned,
            "cancel_target": self._cancel_target,
            "cancel_ack": self._cancel_ack,
            "goal_failures": self.goal_failures,
            "no_frontier_updates_seen": self.no_frontier_updates_seen,
            "candidate": self._format_candidate(self.active_candidate),
            "fault_latched": self.fault_latched,
            "reached_goal_count": getattr(self, "reached_goal_count", 0),
            "completion_policy": SAFE_REACHABLE_COMPLETION_POLICY,
            "raw_frontier_count": getattr(self, "raw_frontier_count", 0),
            "blocked_frontier_count": getattr(
                self, "blocked_frontier_count", 0),
            "blocked_safety_count": getattr(self, "blocked_safety_count", 0),
            "blocked_route_count": getattr(self, "blocked_route_count", 0),
            "unresolved_frontier_count": getattr(self, "unresolved_frontier_count", 0),
            "blocked_count": len(getattr(self, "_blocked_destinations", {})),
            "retry_exhausted_count": len(
                getattr(self, "_retry_exhausted_destinations", set())),
            "motion_stopped": not self._has_motion_locked(),
            "mission_goal_uuid": getattr(self, "_expected_goal_uuid", None) or "",
            "mission_stage": self._mission_stage_for_state(self.state),
            "mission_outcome": self._mission_outcome_for_state(),
            "mission_reason": self.reason,
            "blockage_confirmed": getattr(
                self, "_mission_blockage_confirmed", False),
            "mission_fault_class": getattr(
                self, "_mission_fault_class", "NONE"),
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
            # Status is observational.  A diagnostics transport failure must
            # never alter motion ownership, cancellation, or fault outcomes.
            try:
                publisher.publish(self._status_message_locked())
            except Exception:  # pragma: no cover - middleware boundary
                pass

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
                self._mission_fault_class = "CANCELLATION"
            elif terminal_state == "STOPPED":
                self._mission_fault_class = "CANCELLATION"
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
        if state == "FAULT":
            self._reset_frontier_classification_for_fault_locked()
        elif state == "INCOMPLETE" and self.raw_frontier_count == 0:
            self.raw_frontier_count = (
                self._current_unresolved_frontier_count_locked())
            self.unresolved_frontier_count = self.raw_frontier_count
        self.state = state
        self.reason = reason
        if state == "FAULT":
            self.fault_requested = True
            self.fault_latched = True
            if getattr(self, "_mission_fault_class", "NONE") == "NONE":
                self._mission_fault_class = "NAVIGATION_FAULT"
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
            getattr(self, "_blocked_destinations", {}).clear()
            getattr(self, "_retry_exhausted_destinations", set()).clear()
            self._recovery_goal_world = None
            self._recovery_stationary_sample = None
            self._recovery_stationary_samples = 0
            self.reached_goal_count = 0
            self.raw_frontier_count = 0
            self.blocked_frontier_count = 0
            self.blocked_safety_count = 0
            self.blocked_route_count = 0
            self.unresolved_frontier_count = 0
            self._expected_goal_uuid = None
            self._mission_status_by_uuid = {}
            self._mission_status_invalid_at = None
            self._mission_blockage_confirmed = False
            self._mission_fault_class = "NONE"
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
        base_fresh = self._authority_message_is_fresh(
            base_status, base_received, self.authority_timeout, now=now)
        manipulator_fresh = self._authority_message_is_fresh(
            manipulator_status, manipulator_received,
            self.authority_timeout, now=now)
        return base_ready and manip_ready and base_fresh and manipulator_fresh

    def _authority_ready(self):
        with self._lock:
            return self._authority_ready_locked()

    @staticmethod
    def _valid_transform(
            transform, now_ros_ns, timeout_sec, *, future_tolerance_sec=0.0):
        """Return true only for a finite, timestamped, ROS-time-fresh TF."""
        try:
            stamp = transform.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            future_tolerance_ns = int(future_tolerance_sec * 1_000_000_000)
            if stamp_ns <= 0 or future_tolerance_ns < 0:
                return False
            if stamp_ns > now_ros_ns:
                if stamp_ns - now_ros_ns > future_tolerance_ns:
                    return False
            elif now_ros_ns - stamp_ns > int(timeout_sec * 1_000_000_000):
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

    def _valid_lookup_transform(self, transform):
        return self._valid_transform(
            transform, self._now_ros_ns(), self.tf_timeout,
            future_tolerance_sec=SLAM_MAP_ODOM_FUTURE_TOLERANCE_SEC)

    def _lookup_fresh_transform(self):
        with self._trace_span("lookup") as trace:
            try:
                now = self.get_clock().now()
                if now.nanoseconds <= 0:
                    if trace is not None:
                        trace["outcome"] = "zero_ros_time"
                    return None
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout))
            except (AttributeError, TypeError, ValueError, OverflowError,
                    TransformException, RuntimeError) as exc:
                if trace is not None:
                    trace["outcome"] = "lookup_error"
                    trace["lookup_error"] = str(exc)
                return None
            if trace is not None:
                stamp_ns = self._header_stamp_ns(transform)
                trace["tf_stamp_ns"] = (
                    stamp_ns if isinstance(stamp_ns, int) else None)
                trace["tf_validation_age_sec"] = (
                    (self._now_ros_ns() - stamp_ns) / 1_000_000_000.0
                    if isinstance(stamp_ns, int) else None)
            if not self._valid_lookup_transform(transform):
                if trace is not None:
                    trace["outcome"] = "stale_or_invalid"
                return None
            if trace is not None:
                trace["outcome"] = "accepted"
            return transform

    def _wait_for_action_server(
            self, timeout_sec, *, generation=None, raise_on_error=False):
        with self._trace_span("action_wait", generation=generation) as trace:
            if trace is not None:
                trace["timeout_sec"] = timeout_sec
            try:
                available = self.action_client.wait_for_server(
                    timeout_sec=timeout_sec)
            except Exception as exc:  # pragma: no cover - middleware boundary
                if trace is not None:
                    trace["outcome"] = "wait_error"
                    trace["error"] = str(exc)
                if raise_on_error:
                    raise
                return False
            if trace is not None:
                trace["outcome"] = "available" if available else "unavailable"
            return available

    def _map_fresh(self):
        return self._map_readiness()[0]

    def _map_readiness(self):
        now = self._monotonic()
        with self._lock:
            grid = self.latest_map
            received_at = self.last_map_at
        if not self._message_is_fresh(
                grid, received_at, self.map_timeout, now=now):
            return False, "fresh valid map is unavailable", None
        if occupancy_grid_geometry(grid) is None:
            return False, "fresh valid map is unavailable", None
        return True, "fresh valid map is available", grid

    def _costmap_readiness(self):
        now = self._monotonic()
        with self._lock:
            costmap = self.latest_costmap
            received_at = self.last_costmap_at
        if not self._message_is_fresh(
                costmap, received_at, self.map_timeout, now=now):
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
            available = self._wait_for_action_server(
                0.2, generation=getattr(self, "run_generation", None))
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
        if state == "RECOVERY_WAIT":
            # Recovery is deliberately a non-dispatching state.  The next
            # planning tick is admitted only after this helper records two
            # separate stationary, fresh TF observations.
            self._recovery_wait_tick(run_generation, started_at)
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

    def _apply_frontier_diagnostics_locked(self, diagnostics):
        diagnostics = tuple(diagnostics or ())
        blocked_safety_count = 0
        blocked_route_count = 0
        unresolved_count = 0
        for diagnostic in diagnostics:
            classification = (
                diagnostic.get("classification")
                if isinstance(diagnostic, dict) else None)
            if classification == "BLOCKED_SAFETY":
                blocked_safety_count += 1
            elif classification == "BLOCKED_ROUTE":
                blocked_route_count += 1
            elif classification != "REACHABLE":
                unresolved_count += 1
        self.raw_frontier_count = len(diagnostics)
        self.blocked_safety_count = blocked_safety_count
        self.blocked_route_count = blocked_route_count
        self.blocked_frontier_count = (
            blocked_safety_count + blocked_route_count)
        self.unresolved_frontier_count = unresolved_count

    @staticmethod
    def _frontier_diagnostics_are_blocked(diagnostics, has_frontiers):
        if not has_frontiers:
            return True
        if not isinstance(diagnostics, (list, tuple)) or not diagnostics:
            return False
        return all(
            isinstance(item, dict)
            and item.get("classification") in FRONTIER_BLOCKED_CLASSES
            for item in diagnostics)

    def _finish_without_goal(
            self, run_generation, reason, complete=False, incomplete=False,
            frontier_diagnostics=None):
        with self._lock:
            if self.run_generation != run_generation or self.state != "PLANNING":
                return
            if frontier_diagnostics is not None:
                self._apply_frontier_diagnostics_locked(frontier_diagnostics)
            if complete:
                if frontier_diagnostics is None:
                    self.raw_frontier_count = 0
                    self.blocked_frontier_count = 0
                    self.blocked_safety_count = 0
                    self.blocked_route_count = 0
                    self.unresolved_frontier_count = 0
            elif incomplete:
                if frontier_diagnostics is None:
                    self.raw_frontier_count = (
                        self._current_unresolved_frontier_count_locked())
                    self.unresolved_frontier_count = self.raw_frontier_count
            if incomplete:
                self._terminal_locked("INCOMPLETE", reason)
            elif complete:
                self._terminal_locked("COMPLETE", reason)
            else:
                self._set_state_locked("SCANNING", reason)

    def _current_unresolved_frontier_count_locked(self):
        try:
            if occupancy_grid_geometry(self.latest_map) is None:
                return 0
            clusters = frontier_clusters(
                self.latest_map.info.width,
                self.latest_map.info.height,
                self.latest_map.data)
            return len(clusters) if clusters is not None else 0
        except (AttributeError, TypeError, ValueError, OverflowError):
            return 0

    def _reset_frontier_classification_for_fault_locked(self):
        raw_frontier_count = self._current_unresolved_frontier_count_locked()
        self.raw_frontier_count = raw_frontier_count
        self.blocked_frontier_count = 0
        self.blocked_safety_count = 0
        self.blocked_route_count = 0
        self.unresolved_frontier_count = raw_frontier_count

    def _planning_gate_failure(self, run_generation, started_at, detail):
        self._readiness_failure(
            run_generation, "PLANNING", started_at, detail)

    def _recovery_wait_tick(self, run_generation, started_at):
        """Wait for fresh readiness and two independent stationary TF samples."""
        ready, detail, transform = self._readiness(
            require_action=True, require_costmap=True, require_transform=True)
        with self._lock:
            if not self._decision_matches_locked(
                    run_generation=run_generation,
                    expected_state="RECOVERY_WAIT",
                    expected_started_at=started_at):
                return
            if not ready or transform is None:
                self._recovery_stationary_sample = None
                self._recovery_stationary_samples = 0
                self._set_state_locked(
                    "RECOVERY_WAIT", "recovery readiness unavailable: " + detail)
                return
            pose = self._transform_world_yaw(transform)
            current_costmap = self.latest_costmap
            geometry = costmap_geometry(current_costmap)
            if pose is None or geometry is None:
                self._recovery_stationary_sample = None
                self._recovery_stationary_samples = 0
                self._set_state_locked(
                    "RECOVERY_WAIT", "recovery stationary proof evidence is unavailable")
                return
            # Store (x, y, yaw) from this timer tick.
            current_sample = (pose[0][0], pose[0][1], pose[1])
            previous = self._recovery_stationary_sample
            if previous is None:
                self._recovery_stationary_sample = current_sample
                self._recovery_stationary_samples = 1
                self._set_state_locked(
                    "RECOVERY_WAIT",
                    "recovery waiting for stationary TF proof (1/2)")
                return
            translation_delta = math.hypot(
                current_sample[0] - previous[0],
                current_sample[1] - previous[1])
            yaw_delta = math.atan2(
                math.sin(current_sample[2] - previous[2]),
                math.cos(current_sample[2] - previous[2]))
            stationary = (
                translation_delta <= 0.5 * float(geometry[2])
                and abs(yaw_delta) <= RECOVERY_YAW_TOLERANCE_RAD)
            if not stationary:
                self._recovery_stationary_sample = current_sample
                self._recovery_stationary_samples = 1
                self._set_state_locked(
                    "RECOVERY_WAIT",
                    "recovery waiting for stationary TF proof (motion observed)")
                return
            self._recovery_stationary_sample = current_sample
            self._recovery_stationary_samples = 2
            self.processed_map_version = -1
            self._recovery_stationary_sample = None
            self._recovery_stationary_samples = 0
            self._set_state_locked(
                "SCANNING", "recovery readiness and stationary TF proof passed")

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
            map_snapshot = (deepcopy(grid), self.map_version, self.last_map_at)
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
                deepcopy(costmap), self.costmap_version, self.last_costmap_at)
            route_fingerprint = self._route_evidence_fingerprint(
                map_snapshot[0], costmap_snapshot[0])
            failed_goal_worlds = tuple(
                list(failed_goal_worlds)
                + list(self._blocked_worlds_for_fingerprint_locked(
                    route_fingerprint)))
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
        with self._trace_span("clustering", generation=run_generation) as trace:
            try:
                failed_goal_cells = set()
                for failed_world in failed_goal_worlds:
                    failed_cell = frontier_world_cell(grid, failed_world)
                    if failed_cell is not None:
                        failed_goal_cells.add(failed_cell)
                clusters = frontier_clusters(
                    grid.info.width, grid.info.height, grid.data)
                if trace is not None:
                    trace["cluster_count"] = (
                        len(clusters) if clusters is not None else None)
                    trace["outcome"] = "ok" if clusters is not None else "invalid"
            except Exception as exc:  # pragma: no cover - algorithm boundary
                if trace is not None:
                    trace["outcome"] = "algorithm_error"
                    trace["error"] = str(exc)
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

        transform_from_lookup = transform is None
        if transform_from_lookup:
            transform = self._lookup_fresh_transform()
        valid_transform = (
            self._valid_lookup_transform(transform)
            if transform_from_lookup else self._valid_transform(
                transform, self._now_ros_ns(), self.tf_timeout))
        if not valid_transform:
            self._planning_gate_failure(
                run_generation, started_at,
                "timestamped fresh map to base_footprint TF is unavailable")
            return

        transform_pose = self._transform_world_yaw(transform)
        if transform_pose is None:
            self._planning_gate_failure(
                run_generation, started_at,
                "timestamped fresh map to base_footprint TF is unavailable")
            return
        robot_world, robot_yaw = transform_pose
        with self._trace_span("route_search", generation=run_generation) as trace:
            try:
                selection_result = costmap_frontier_candidates(
                    clusters, grid, costmap, failed_goal_cells,
                    robot_world=robot_world,
                    min_goal_distance=self.min_goal_distance,
                    footprint=NAVIGATION_FOOTPRINT,
                    robot_yaw=robot_yaw,
                    require_path_clear=True, return_diagnostics=True)
                if (isinstance(selection_result, tuple)
                        and len(selection_result) == 2):
                    candidates, frontier_diagnostics = selection_result
                else:
                    # Preserve integration seams that still provide the
                    # historical list-only selector contract.
                    candidates = selection_result
                    frontier_diagnostics = None
                if trace is not None:
                    trace["candidate_count"] = (
                        len(candidates) if candidates is not None else None)
                    trace["outcome"] = "ok" if candidates is not None else "invalid"
            except Exception as exc:  # pragma: no cover - algorithm boundary
                if trace is not None:
                    trace["outcome"] = "algorithm_error"
                    trace["error"] = str(exc)
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
            current_map_ready, current_map_detail, _ = (
                self._map_readiness())
            if not current_map_ready:
                self._planning_gate_failure(
                    run_generation, started_at, current_map_detail)
                return
            with self._lock:
                evidence_ok, evidence_detail = (
                    self._content_and_freshness_matches_locked(
                        map_snapshot, costmap_snapshot))
                if not evidence_ok:
                    if evidence_detail in (
                            "frontier plan discarded because fresh valid map is unavailable",
                            "frontier plan discarded because map evidence is stale"):
                        self._planning_gate_failure(
                            run_generation, started_at,
                            "fresh valid map is unavailable")
                    else:
                        self._discard_stale_plan_locked(
                            run_generation, started_at, evidence_detail)
                    return
                self._apply_frontier_diagnostics_locked(frontier_diagnostics)
            with self._lock:
                if not self._decision_matches_locked(
                        run_generation=run_generation,
                        expected_state="PLANNING",
                        expected_started_at=started_at):
                    return
                self.no_frontier_updates_seen += 1
                limit_reached = self.no_frontier_updates_seen >= self.no_frontier_limit
                blocked_completion = (
                    limit_reached
                    and self._frontier_diagnostics_are_blocked(
                        frontier_diagnostics, bool(clusters)))
                incomplete = (
                    limit_reached and bool(clusters)
                    and not blocked_completion)
            if incomplete:
                detail = "exploration incomplete: frontier classification is unresolved"
            elif limit_reached and bool(clusters):
                detail = (
                    "exploration complete: reachable area exhausted; "
                    "blocked frontiers remain")
            elif limit_reached:
                detail = "exploration complete: no costmap-valid frontier remains"
            else:
                detail = "no costmap-valid frontier in this map update"
            self._finish_without_goal(
                run_generation, detail,
                complete=limit_reached and not incomplete,
                incomplete=incomplete,
                frontier_diagnostics=frontier_diagnostics)
            return

        gx, gy = candidates[0]
        goal_world = frontier_cell_world(grid, (gx, gy))
        if goal_world is None:
            self._fault(
                "frontier map geometry is invalid",
                expected_run_generation=run_generation,
                expected_state="PLANNING",
                expected_started_at=started_at)
            return

        fresh_transform = None
        valid_transform = (
            self._valid_lookup_transform(transform)
            if transform_from_lookup else self._valid_transform(
                transform, self._now_ros_ns(), self.tf_timeout))
        if not valid_transform:
            # The TF lookup is intentionally outside _lock.  It may wait for
            # middleware data while receipt callbacks continue to run.
            with self._trace_span("refresh", generation=run_generation) as trace:
                fresh_transform = self._lookup_fresh_transform()
                if fresh_transform is None:
                    if trace is not None:
                        trace["outcome"] = "unavailable"
                        trace["evidence_reason"] = (
                            "frontier plan discarded because refreshed TF is unavailable")
                    with self._lock:
                        self._discard_stale_plan_locked(
                            run_generation, started_at,
                            "frontier plan discarded because refreshed TF is unavailable")
                    return
                if trace is not None:
                    stamp_ns = self._header_stamp_ns(fresh_transform)
                    trace["tf_stamp_ns"] = (
                        stamp_ns if isinstance(stamp_ns, int) else None)
                    trace["tf_validation_age_sec"] = (
                        (self._now_ros_ns() - stamp_ns) / 1_000_000_000.0
                        if isinstance(stamp_ns, int) else None)
                fresh_pose = self._transform_world_yaw(fresh_transform)
                costmap_snapshot_geometry = costmap_geometry(costmap_snapshot[0])
                if (fresh_pose is None
                        or costmap_snapshot_geometry is None
                        or not _route_start_proof(
                            costmap_snapshot_geometry,
                            costmap_snapshot[0].data,
                            NAVIGATION_FOOTPRINT,
                            robot_world, robot_yaw,
                            fresh_pose[0], fresh_pose[1])
                        or not _goal_distance_is_valid(
                            fresh_pose[0], goal_world, self.min_goal_distance)):
                    if trace is not None:
                        trace["outcome"] = "unsafe"
                        trace["evidence_reason"] = (
                            "frontier plan discarded because refreshed route "
                            "start is unsafe")
                    with self._lock:
                        self._discard_stale_plan_locked(
                            run_generation, started_at,
                            "frontier plan discarded because refreshed route "
                            "start is unsafe")
                    return
                if trace is not None:
                    trace["outcome"] = "accepted"

        try:
            if not self._wait_for_action_server(
                    0.2, generation=run_generation, raise_on_error=True):
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

        pose = PoseStamped()
        pose.header.frame_id = "map"
        stamp = self._now_ros_msg()
        if stamp is not None:
            pose.header.stamp = stamp
        pose.pose.position.x = goal_world[0]
        pose.pose.position.y = goal_world[1]
        pose.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._reserve_and_send(
            goal, (gx, gy), run_generation, expected_started_at=started_at,
            map_snapshot=map_snapshot, costmap_snapshot=costmap_snapshot,
            transform=transform, fresh_transform=fresh_transform,
            goal_world=goal_world, transform_from_lookup=transform_from_lookup,
            fresh_transform_from_lookup=fresh_transform is not None)

    @staticmethod
    def _map_content_equal(planned_map, latest_map):
        try:
            return (
                planned_map.header.frame_id == latest_map.header.frame_id
                and planned_map.info == latest_map.info
                and list(planned_map.data) == list(latest_map.data))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _costmap_content_equal(planned_costmap, latest_costmap):
        try:
            planned_metadata = planned_costmap.metadata
            latest_metadata = latest_costmap.metadata
            field_getter = "get_fields_and_field_types"
            planned_get_fields = getattr(planned_metadata, field_getter, None)
            latest_get_fields = getattr(latest_metadata, field_getter, None)
            if callable(planned_get_fields) and callable(latest_get_fields):
                planned_fields = set(planned_get_fields().keys())
                latest_fields = set(latest_get_fields().keys())
            else:
                planned_fields = set(vars(planned_metadata))
                latest_fields = set(vars(latest_metadata))
            if planned_fields != latest_fields:
                return False
            return (
                planned_costmap.header.frame_id == latest_costmap.header.frame_id
                and all(
                    field == "update_time"
                    or getattr(planned_metadata, field) == getattr(latest_metadata, field)
                    for field in planned_fields)
                and list(planned_costmap.data) == list(latest_costmap.data))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    def _content_and_freshness_matches_locked(
            self, map_snapshot, costmap_snapshot):
        with self._trace_span("evidence_check") as trace:
            result = self._content_and_freshness_matches_impl_locked(
                map_snapshot, costmap_snapshot)
            if trace is not None:
                trace["outcome"] = "accepted" if result[0] else "rejected"
                trace["evidence_reason"] = result[1]
            return result

    def _content_and_freshness_matches_impl_locked(
            self, map_snapshot, costmap_snapshot):
        try:
            expected_map, _expected_map_version, _expected_map_received_at = (
                map_snapshot)
            expected_costmap, _expected_costmap_version, _expected_costmap_received_at = (
                costmap_snapshot)
        except (TypeError, ValueError):
            return False, "frontier evidence snapshot is incomplete"

        if occupancy_grid_geometry(self.latest_map) is None:
            return False, "frontier plan discarded because fresh valid map is unavailable"
        now = self._monotonic()
        if not self._message_is_fresh(
                self.latest_map, self.last_map_at, self.map_timeout, now=now):
            return False, "frontier plan discarded because map evidence is stale"
        if not self._map_content_equal(expected_map, self.latest_map):
            return False, "frontier plan discarded because map evidence changed"
        if not self._costmap_content_equal(expected_costmap, self.latest_costmap):
            return False, "frontier plan discarded because costmap evidence changed"
        if (not self._message_is_fresh(
                self.latest_costmap, self.last_costmap_at,
                self.map_timeout, now=now)
                or costmap_geometry(self.latest_costmap) is None):
            return False, "frontier plan discarded because costmap evidence is stale"
        return True, ""

    def _reservation_evidence_matches_locked(
            self, map_snapshot, costmap_snapshot, transform,
            fresh_transform=None, goal_world=None, transform_from_lookup=False,
            fresh_transform_from_lookup=False):
        evidence_ok, detail = self._content_and_freshness_matches_locked(
            map_snapshot, costmap_snapshot)
        if not evidence_ok:
            return False, detail
        now = self._monotonic()
        if not self._authority_ready_locked(now):
            return False, "frontier plan discarded because motion authority changed"
        if fresh_transform is None:
            valid_transform = (
                self._valid_lookup_transform(transform)
                if transform_from_lookup else self._valid_transform(
                    transform, self._now_ros_ns(), self.tf_timeout))
            if not valid_transform:
                return False, "frontier plan discarded because carried TF is stale"
            return True, ""
        valid_fresh_transform = (
            self._valid_lookup_transform(fresh_transform)
            if fresh_transform_from_lookup else self._valid_transform(
                fresh_transform, self._now_ros_ns(), self.tf_timeout))
        if not valid_fresh_transform:
            return False, "frontier plan discarded because refreshed TF is stale"
        expected_costmap = costmap_snapshot[0]
        costmap_snapshot_geometry = costmap_geometry(expected_costmap)
        old_pose = self._transform_world_yaw(transform)
        fresh_pose = self._transform_world_yaw(fresh_transform)
        if (old_pose is None or fresh_pose is None
                or costmap_snapshot_geometry is None
                or not _route_start_proof(
                    costmap_snapshot_geometry, expected_costmap.data,
                    NAVIGATION_FOOTPRINT,
                    old_pose[0], old_pose[1],
                    fresh_pose[0], fresh_pose[1])):
            return (
                False,
                "frontier plan discarded because refreshed route start is unsafe")
        if not _goal_distance_is_valid(
                fresh_pose[0], goal_world, self.min_goal_distance):
            return False, "frontier plan discarded because refreshed goal is too close"
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
            fresh_transform=None, goal_world=None, transform_from_lookup=False,
            fresh_transform_from_lookup=False):
        with self._trace_span("reservation", generation=run_generation) as trace:
            result = self._reserve_and_send_impl(
                goal, candidate, run_generation,
                expected_started_at=expected_started_at,
                map_snapshot=map_snapshot, costmap_snapshot=costmap_snapshot,
                transform=transform, fresh_transform=fresh_transform,
                goal_world=goal_world,
                transform_from_lookup=transform_from_lookup,
                fresh_transform_from_lookup=fresh_transform_from_lookup)
            if trace is not None:
                with self._lock:
                    trace["state_after"] = self.state
                    trace["reason_after"] = self.reason
                    trace["motion_owned_after"] = self._motion_owned
                    trace["outcome"] = (
                        "reserved" if self.state == "GOAL_PENDING"
                        else self.state.lower())
                    if self.state == "SCANNING":
                        trace["evidence_reason"] = self.reason
            return result

    def _reserve_and_send_impl(
            self, goal, candidate, run_generation, expected_started_at=_UNSET,
            map_snapshot=None, costmap_snapshot=None, transform=None,
            fresh_transform=None, goal_world=None, transform_from_lookup=False,
            fresh_transform_from_lookup=False):
        with self._lock:
            if (not self._decision_matches_locked(
                    run_generation=run_generation,
                    expected_state="PLANNING",
                    expected_started_at=expected_started_at)
                    or self._has_motion_locked() or self.fault_latched):
                return
            evidence_ok, detail = self._reservation_evidence_matches_locked(
                map_snapshot, costmap_snapshot, transform,
                fresh_transform=fresh_transform, goal_world=goal_world,
                transform_from_lookup=transform_from_lookup,
                fresh_transform_from_lookup=fresh_transform_from_lookup)
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
            self._expected_goal_uuid = None
            self._mission_status_by_uuid = {}
            self._mission_status_invalid_at = None
            self._mission_blockage_confirmed = False
            self._mission_fault_class = "NONE"
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
        uuid_unavailable = False
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
                self._expected_goal_uuid = self._canonical_goal_uuid(
                    getattr(goal_handle, "goal_id", None))
                uuid_unavailable = self._expected_goal_uuid is None
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
        if uuid_unavailable:
            self._fault(
                "accepted navigation goal UUID is unavailable",
                expected_run_generation=run_generation,
                expected_motion_token=(run_generation, motion_generation),
                expected_state=self.state,
                expected_deadline=self._motion_deadline,
                expected_started_at=self.started_at)
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
                self.reached_goal_count = getattr(self, "reached_goal_count", 0) + 1
                self._mission_blockage_confirmed = False
                self._mission_fault_class = "NONE"
                self._set_state_locked("SCANNING", "navigation goal succeeded")
            else:
                mission_record, evidence_detail = (
                    self._matching_mission_terminal_locked())
                confirmed_blockage = (
                    status != GoalStatus.STATUS_CANCELED
                    and mission_record is not None
                    and mission_record.get("outcome") == "FAULT"
                    and mission_record.get("fault_class") == "OBSTACLE_BLOCKAGE"
                    and mission_record.get("blockage_confirmed") is True)
                if confirmed_blockage:
                    fingerprint = self._route_evidence_fingerprint(
                        self.latest_map, self.latest_costmap)
                    self._record_blockage_locked(
                        attempted_goal_world, fingerprint)
                    self._clear_motion_locked()
                    self._clear_readiness_episode_locked()
                    self._recovery_goal_world = attempted_goal_world
                    self._recovery_stationary_sample = None
                    self._recovery_stationary_samples = 0
                    self._mission_blockage_confirmed = True
                    self._mission_fault_class = "OBSTACLE_BLOCKAGE"
                    self.processed_map_version = -1
                    self._set_state_locked(
                        "RECOVERY_WAIT",
                        "confirmed obstacle blockage; waiting for recovery evidence")
                elif self._is_recoverable_planner_abort(status, mission_record):
                    fingerprint = self._route_evidence_fingerprint(
                        self.latest_map, self.latest_costmap)
                    if (attempted_goal_world is not None
                            and fingerprint is not None):
                        # A planner-result no-path is safe to defer because the
                        # mission supervisor has already proved that no
                        # downstream motion obligation remains.  Keep the
                        # planner fault class visible, but do not turn a
                        # single unreachable frontier into a run fault.
                        self._record_blockage_locked(
                            attempted_goal_world, fingerprint)
                        self._clear_motion_locked()
                        self._clear_readiness_episode_locked()
                        self._recovery_goal_world = attempted_goal_world
                        self._recovery_stationary_sample = None
                        self._recovery_stationary_samples = 0
                        self.goal_failures += 1
                        self._mission_blockage_confirmed = False
                        self._mission_fault_class = "PLANNER_ABORT"
                        self.processed_map_version = -1
                        self._set_state_locked(
                            "RECOVERY_WAIT",
                            "global planning failed; deferring frontier under current route evidence")
                    else:
                        # Missing route evidence cannot authorize deferral.
                        detail = (
                            "navigation goal ended with status %s; %s"
                            % (status, evidence_detail))
                        self._record_goal_failure_locked(
                            detail,
                            candidate=candidate,
                            attempted_goal_world=attempted_goal_world,
                            fault_class="PLANNER_ABORT")
                else:
                    fault_class = "NAVIGATION_FAULT"
                    if mission_record is not None:
                        candidate_fault_class = mission_record.get("fault_class")
                        if candidate_fault_class:
                            fault_class = candidate_fault_class
                    detail = (
                        "navigation goal ended with status %s; %s"
                        % (status, evidence_detail))
                    self._record_goal_failure_locked(
                        detail,
                        candidate=candidate,
                        attempted_goal_world=attempted_goal_world,
                        fault_class=fault_class)

    def _record_goal_failure_locked(
            self, detail, candidate=None, attempted_goal_world=_UNSET,
            fault_class="NAVIGATION_FAULT"):
        if candidate is None:
            candidate = self.active_candidate
        if attempted_goal_world is _UNSET:
            attempted_goal_world = self._attempted_goal_world
        attempted_goal_world = self._validated_world_point(attempted_goal_world)
        self._clear_motion_locked()
        self.goal_failures += 1
        if attempted_goal_world is not None:
            self.failed_goal_worlds.add(attempted_goal_world)
        self._mission_blockage_confirmed = False
        self._mission_fault_class = fault_class or "NAVIGATION_FAULT"
        self._terminal_locked("FAULT", detail)

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
        if getattr(self, "_mission_fault_class", "NONE") == "NONE":
            self._mission_fault_class = "CANCELLATION"
        self._reset_frontier_classification_for_fault_locked()
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
