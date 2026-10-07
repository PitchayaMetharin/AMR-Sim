#!/usr/bin/env python3
"""Bounded autonomous adapter for the accepted Gate 6 product cycle.

The adapter owns the public manipulation status and ExecuteProductCycle action.
The Gate 6 runner is started as a private child with an internal status topic;
the station/product mapping is supplied by the launch layer so this package
does not depend on the factory package at runtime.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from typing import Dict, Optional, Set

import rclpy
from amr_interfaces.action import ExecuteProductCycle
from amr_interfaces.msg import ManipulatorStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


STOW = {
    "arm_joint_1": 0.0,
    "arm_joint_2": -1.5708,
    "arm_joint_3": 1.5708,
    "arm_joint_4": 0.0,
    "arm_joint_5": 0.0,
    "arm_joint_6": 0.0,
}
STOW_TOLERANCE_RAD = 0.01
JOINT_STATUS_MAX_AGE_S = 0.2
INTERNAL_STATUS_MAX_AGE_S = 0.2
BOOTSTRAP_PROOF_MAX_AGE_S = 0.75
OWNERSHIP_SCHEMA = "AMR_CYCLE_OWNERSHIP_V1"
OWNERSHIP_LOG_PREFIX = OWNERSHIP_SCHEMA + " "
OWNERSHIP_MAX_RECORDS = 64
OWNERSHIP_MAX_RECORD_BYTES = 4096
OWNERSHIP_HISTORY_REASONS = frozenset({
    "owned_status_unreadable", "invalid_initial_stage_claim",
    "wrong_product_stage_claim", "contradictory_stage_ownership",
    "foreign_stage_claim", "owned_sequence_rollback", "owned_status_invalid",
    "accepted_child_snapshot_unavailable", "public_boundary_snapshot_unavailable",
    "first_start_boundary_unqualified", "first_loaded_boundary_unqualified",
    "first_empty_boundary_unqualified",
    "record_capacity_exhausted", "observation_unavailable",
})


class MappingError(ValueError):
    """An absent or malformed launch-supplied registry mapping."""


class CycleSupervisor(Node):
    """Run exactly one validated Gate 6 cycle at a time."""

    def __init__(self) -> None:
        super().__init__("manipulation_supervisor_node")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("runner", "gate6_product_test")
        self.declare_parameter("allow_product_cycles", True)
        self.declare_parameter(
            "internal_status_topic", "/amr/manipulation/internal/status")
        self.declare_parameter(
            "internal_cancel_service", "/amr/manipulation/internal/cancel_cycle_motion")
        # Launch must provide these validated mappings.  Empty defaults keep
        # the adapter unavailable rather than inventing a second registry.
        self.declare_parameter(
            "product_station_map", Parameter.Type.STRING_ARRAY)
        self.declare_parameter(
            "autonomous_product_ids", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter(
            "dispatch_station_ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter(
            "bootstrap_service", "/amr/simulation/attachment_bootstrap/verify")

        self._lock = threading.RLock()
        self._active_goal = None
        self._goal_reserved = False
        self._cancel_requested = threading.Event()
        self._child: Optional[subprocess.Popen] = None
        self._child_attached = False
        self._child_product_id = ""
        self._child_state = ManipulatorStatus.STARTING
        self._child_valid = False
        self._child_consistent = False
        self._child_terminal_empty_proof = False
        self._child_preparation_boot_id = 0
        self._child_mass_stage_boot_id = 0
        self._child_stage_started = False
        self._child_stage_loaded_proof = False
        self._child_status_authority_open = False
        self._child_detail = "waiting for a cycle"
        self._child_boot_id = 0
        self._child_sequence = 0
        self._child_status_at = 0.0
        self._ownership_epoch = None
        self._state = ManipulatorStatus.STARTING
        self._base_motion_allowed = False
        self._product_attached = False
        self._product_id = ""
        self._detail = "waiting for validated autonomous station mapping"
        self._fault_latched = False
        self._allow_product_cycles = bool(
            self.get_parameter("allow_product_cycles").value)
        self._joint_states: Optional[JointState] = None
        self._joint_states_at = 0.0
        self._bootstrap_future = None
        self._bootstrap_detached = False
        self._bootstrap_proof_at = 0.0

        try:
            self._product_by_station, self._autonomous_product_ids, \
                self._dispatch_station_ids = self._parse_mapping_parameters()
            self._mapping_ready = True
        except MappingError as error:
            self._product_by_station = {}
            self._autonomous_product_ids = set()
            self._dispatch_station_ids = set()
            self._mapping_ready = False
            self.get_logger().error(
                f"autonomous station mapping unavailable: {error}")

        self._status_group = ReentrantCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._authority_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=0.1),
        )
        self._joint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(
            ManipulatorStatus, "/amr/manipulation/status", self._authority_qos)
        self._internal_sub = self.create_subscription(
            ManipulatorStatus,
            str(self.get_parameter("internal_status_topic").value),
            self._internal_status_callback,
            self._authority_qos,
            callback_group=self._status_group,
        )
        self._joint_sub = self.create_subscription(
            JointState,
            "/amr/base/joint_states",
            self._joint_state_callback,
            self._joint_qos,
            callback_group=self._status_group,
        )
        self._bootstrap_client = self.create_client(
            Trigger,
            str(self.get_parameter("bootstrap_service").value),
            callback_group=self._service_group,
        )
        self._cancel_client = self.create_client(
            Trigger,
            str(self.get_parameter("internal_cancel_service").value),
            callback_group=self._service_group,
        )
        self._action_server = ActionServer(
            self,
            ExecuteProductCycle,
            "/amr/manipulation/execute_product_cycle",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )
        self._boot_id = int(time.monotonic_ns() & 0xFFFFFFFF) or 1
        self._sequence = 0
        self._status_timer = self.create_timer(
            0.05, self._publish_status, callback_group=self._status_group)
        self._publish_status()

    def _parse_mapping_parameters(self):
        raw_mapping = self.get_parameter_or("product_station_map").value
        raw_ids = self.get_parameter_or("autonomous_product_ids").value
        raw_dispatch = self.get_parameter_or("dispatch_station_ids").value
        if not isinstance(raw_mapping, (list, tuple)) or not raw_mapping:
            raise MappingError("product_station_map must be a non-empty string array")
        if not isinstance(raw_ids, (list, tuple)) or not raw_ids:
            raise MappingError("autonomous_product_ids must be a non-empty integer array")
        if not isinstance(raw_dispatch, (list, tuple)) or not raw_dispatch:
            raise MappingError("dispatch_station_ids must be a non-empty string array")

        enabled_ids: Set[int] = set()
        for value in raw_ids:
            if isinstance(value, bool):
                raise MappingError("autonomous product IDs must be integers")
            try:
                product_id = int(value)
            except (TypeError, ValueError) as error:
                raise MappingError("autonomous product IDs must be integers") from error
            if product_id == 103 or product_id <= 0:
                raise MappingError("Product103 is disabled for autonomous work")
            if product_id in enabled_ids:
                raise MappingError(f"duplicate autonomous product ID: {product_id}")
            enabled_ids.add(product_id)

        product_by_station: Dict[str, str] = {}
        product_ids: Set[int] = set()
        for raw_entry in raw_mapping:
            if not isinstance(raw_entry, str) or raw_entry.count("=") != 1:
                raise MappingError("product_station_map entries must be station=product")
            station_id, raw_product_id = (part.strip() for part in raw_entry.split("=", 1))
            if not station_id or not raw_product_id.isdigit():
                raise MappingError("product_station_map contains an invalid entry")
            product_id = int(raw_product_id)
            if station_id in product_by_station or product_id in product_ids:
                raise MappingError("product station and product IDs must be unique")
            if product_id == 103 or product_id not in enabled_ids:
                raise MappingError("mapping references a disabled or unregistered product")
            product_by_station[station_id] = str(product_id)
            product_ids.add(product_id)
        if product_ids != enabled_ids:
            raise MappingError("product mapping and autonomous product IDs do not match")

        dispatch_station_ids: Set[str] = set()
        for value in raw_dispatch:
            if not isinstance(value, str) or not value.strip():
                raise MappingError("dispatch station IDs must be non-empty strings")
            station_id = value.strip()
            if station_id in dispatch_station_ids:
                raise MappingError(f"duplicate dispatch station ID: {station_id}")
            dispatch_station_ids.add(station_id)
        return product_by_station, enabled_ids, dispatch_station_ids

    def _goal_callback(self, goal: ExecuteProductCycle.Goal) -> GoalResponse:
        # Mapping mode uses this adapter as the sole manipulation-status and
        # control authority while explicitly disabling product-cycle motion.
        # Keep this as the first admission gate so a disabled adapter never
        # performs bootstrap, joint-state, or registry work for a goal.
        if not self._allow_product_cycles:
            return GoalResponse.REJECT
        if not self._mapping_ready or not goal:
            return GoalResponse.REJECT
        product_id = self._product_by_station.get(goal.pickup_station_id)
        if product_id is None or goal.destination_station_id not in self._dispatch_station_ids:
            return GoalResponse.REJECT
        if not self._fresh_independent_empty_stow():
            return GoalResponse.REJECT
        with self._lock:
            if (self._fault_latched or self._goal_reserved or self._active_goal is not None or
                    self._child is not None or self._product_attached or
                    self._state != ManipulatorStatus.STOWED_EMPTY or
                    not self._base_motion_allowed):
                return GoalResponse.REJECT
            # Reserve before the action server schedules execute_callback so a
            # second goal cannot pass admission during that scheduling window.
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        with self._lock:
            if goal_handle is None or (
                    goal_handle != self._active_goal and
                    not (self._goal_reserved and self._active_goal is None)):
                return CancelResponse.REJECT
            self._cancel_requested.set()
        return CancelResponse.ACCEPT

    def _joint_state_callback(self, message: JointState) -> None:
        with self._lock:
            self._joint_states = message
            self._joint_states_at = time.monotonic()

    def _bootstrap_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # service loss keeps the adapter unavailable
            self.get_logger().error(
                f"attachment bootstrap proof failed: {error}")
            response = None
        with self._lock:
            if future is not self._bootstrap_future:
                return
            self._bootstrap_future = None
            if response is not None and response.success:
                self._bootstrap_detached = True
                self._bootstrap_proof_at = time.monotonic()
            else:
                self._bootstrap_detached = False
                self._bootstrap_proof_at = 0.0

    def _request_bootstrap_proof(self) -> None:
        with self._lock:
            if self._bootstrap_future is not None or not self._bootstrap_client.service_is_ready():
                return
            future = self._bootstrap_client.call_async(Trigger.Request())
            self._bootstrap_future = future
            future.add_done_callback(self._bootstrap_done)

    def _joint_stowed_locked(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        if self._joint_states is None or now - self._joint_states_at > JOINT_STATUS_MAX_AGE_S:
            return False
        names = self._joint_states.name
        values = self._joint_states.position
        if len(names) != len(values) or len(set(names)) != len(names):
            return False
        positions = dict(zip(names, values))
        for joint, target in STOW.items():
            value = positions.get(joint)
            if value is None or not math.isfinite(value) or abs(value - target) > STOW_TOLERANCE_RAD:
                return False
        return True

    def _independent_empty_stow_proven_locked(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self._bootstrap_detached and
            now - self._bootstrap_proof_at <= BOOTSTRAP_PROOF_MAX_AGE_S and
            self._joint_stowed_locked(now)
        )

    def _idle_motion_allowed_locked(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            not self._fault_latched and
            self._active_goal is None and
            not self._goal_reserved and
            self._child is None and
            self._state == ManipulatorStatus.STOWED_EMPTY and
            not self._product_attached and
            not self._child_attached and
            self._independent_empty_stow_proven_locked(now)
        )

    def _fresh_independent_empty_stow(self) -> bool:
        self._request_bootstrap_proof()
        with self._lock:
            return self._independent_empty_stow_proven_locked()

    def _wait_fresh_independent_empty_stow(self, timeout: float = 3.0) -> bool:
        with self._lock:
            self._bootstrap_detached = False
            self._bootstrap_proof_at = 0.0
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._fresh_independent_empty_stow():
                return True
            time.sleep(0.02)
        return False

    @staticmethod
    def _is_mass_stage_progress(message: ManipulatorStatus, consistent: bool) -> bool:
        if not consistent:
            return False
        if (message.state == ManipulatorStatus.MOVING and
                message.detail ==
                "Arm command inhibited pending fresh READY and 500 ms stationary evidence"):
            return True
        return (message.state == ManipulatorStatus.STOWED_LOADED and
                message.detail.startswith("Gate 6 "))

    @staticmethod
    def _status_evidence_snapshot(message):
        """Copy one status message into immutable primitive evidence fields."""
        try:
            stamp = message.header.stamp
            return {
                "header": {
                    "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
                    "frame_id": str(message.header.frame_id),
                },
                "source_boot_id": int(message.source_boot_id),
                "sequence": int(message.sequence),
                "valid": bool(message.valid),
                "state": int(message.state),
                "base_motion_allowed": bool(message.base_motion_allowed),
                "product_attached": bool(message.product_attached),
                "product_id": str(message.product_id),
                "detail": str(message.detail),
            }
        except Exception:  # evidence failure must never change action behavior
            return None

    @staticmethod
    def _new_ownership_epoch(goal_handle, product_id: str, public_boot_id: int):
        """Create an execution-local observer from the accepted action UUID."""
        try:
            goal_uuid = bytes(int(value) for value in goal_handle.goal_id.uuid)
            if len(goal_uuid) != 16 or not any(goal_uuid):
                return None
            if type(product_id) is not str or not product_id:
                return None
            return {
                "execution_uuid": goal_uuid.hex(),
                "product_id": product_id,
                "public_source_boot_id": int(public_boot_id),
                "record_count": 0,
                "history": set(),
                "latest_child": None,
                "latest_child_received_monotonic_s": None,
                "first_start_attempted": False,
                "first_loaded_attempted": False,
                "first_empty_attempted": False,
                "first_start": None,
                "first_loaded": None,
                "first_empty": None,
                "initial_claim_seen": False,
                "sealed": False,
            }
        except Exception:  # missing goal identity disables only augmentation
            return None

    def _mark_ownership_observation_unavailable_locked(self) -> None:
        """Fail closed for corroboration without crossing into action behavior."""
        try:
            epoch = getattr(self, "_ownership_epoch", None)
            if epoch is not None and not epoch.get("sealed"):
                epoch["history"].add("observation_unavailable")
        except Exception:
            return

    def _observe_safely_locked(self, observer, *args):
        """Run only an added observer hook; its failures cannot affect the cycle."""
        try:
            return observer(*args)
        except Exception:
            self._mark_ownership_observation_unavailable_locked()
            return None

    def _observe_child_before_filters_locked(self, message) -> None:
        """Latch bounded evidence guards before legacy callback early returns."""
        epoch = getattr(self, "_ownership_epoch", None)
        if epoch is None or epoch.get("sealed") or self._active_goal is None:
            return
        try:
            boot_id = int(message.source_boot_id)
            sequence = int(message.sequence)
            state = int(message.state)
            valid = bool(message.valid)
            attached = bool(message.product_attached)
            product_id = str(message.product_id)
            detail = str(message.detail)
        except Exception:
            epoch["history"].add("owned_status_unreadable")
            return

        owned_boot_id = int(getattr(self, "_child_mass_stage_boot_id", 0))
        stage_started = bool(getattr(self, "_child_stage_started", False))
        stage_claim = any((
            detail == "Gate 6 mass stage is starting",
            state == ManipulatorStatus.MOVING and detail ==
            "Arm command inhibited pending fresh READY and 500 ms stationary evidence",
            state == ManipulatorStatus.STOWED_LOADED and detail.startswith("Gate 6 "),
            state == ManipulatorStatus.STOWED_LOADED and attached,
        ))
        if not stage_claim and not (stage_started and boot_id == owned_boot_id):
            return

        identity_valid = boot_id > 0 and sequence > 0
        expected_product = epoch["product_id"]
        known_state = state in (
            ManipulatorStatus.STARTING,
            ManipulatorStatus.STOWED_EMPTY,
            ManipulatorStatus.STOWED_LOADED,
            ManipulatorStatus.MOVING,
            ManipulatorStatus.DEPLOYED,
            ManipulatorStatus.FAULT,
        )
        product_consistent = product_id == (expected_product if attached else "")
        state_consistent = all((
            known_state,
            valid == (state != ManipulatorStatus.FAULT),
            product_consistent,
            state != ManipulatorStatus.STOWED_EMPTY or not attached,
            state != ManipulatorStatus.STOWED_LOADED or attached,
            state not in (
                ManipulatorStatus.STARTING,
                ManipulatorStatus.MOVING,
                ManipulatorStatus.DEPLOYED,
                ManipulatorStatus.FAULT,
            ) or not bool(message.base_motion_allowed),
        ))
        if stage_claim:
            if not identity_valid or not state_consistent or state == ManipulatorStatus.FAULT:
                epoch["history"].add("invalid_initial_stage_claim")
            if not product_consistent:
                epoch["history"].add("wrong_product_stage_claim")
            if not epoch["initial_claim_seen"]:
                epoch["initial_claim_seen"] = True
            elif (not stage_started and
                  boot_id != int(epoch.get("initial_claim_boot_id", boot_id))):
                epoch["history"].add("contradictory_stage_ownership")
            if epoch.get("initial_claim_boot_id") is None and identity_valid:
                epoch["initial_claim_boot_id"] = boot_id

        if not identity_valid:
            if stage_started:
                epoch["history"].add("owned_status_unreadable")
            return
        if stage_started and boot_id != owned_boot_id:
            if stage_claim:
                epoch["history"].add("foreign_stage_claim")
                epoch["history"].add("contradictory_stage_ownership")
        if ((stage_started and boot_id == owned_boot_id) or
                (stage_claim and boot_id == int(getattr(self, "_child_boot_id", 0)))):
            if (boot_id == getattr(self, "_child_boot_id", 0) and
                    sequence <= getattr(self, "_child_sequence", 0)):
                epoch["history"].add("owned_sequence_rollback")
            if not state_consistent or state == ManipulatorStatus.FAULT:
                epoch["history"].add("owned_status_invalid")

    def _observe_accepted_child_locked(self, message, received_monotonic_s: float) -> None:
        epoch = getattr(self, "_ownership_epoch", None)
        if epoch is None or epoch.get("sealed") or self._active_goal is None:
            return
        try:
            snapshot = self._status_evidence_snapshot(message)
            if snapshot is None:
                epoch["history"].add("accepted_child_snapshot_unavailable")
                return
            epoch["latest_child"] = snapshot
            epoch["latest_child_received_monotonic_s"] = float(received_monotonic_s)
        except Exception:
            epoch["history"].add("observation_unavailable")

    def _ownership_record_locked(self, epoch, event: str, now: float,
                                 public_status=None, extra=None):
        if epoch is None or epoch.get("sealed"):
            return None
        if epoch["record_count"] >= OWNERSHIP_MAX_RECORDS:
            epoch["history"].add("record_capacity_exhausted")
            return None
        epoch["record_count"] += 1
        received = epoch["latest_child_received_monotonic_s"]
        child_age = None if received is None else float(now) - received
        record = {
            "schema": OWNERSHIP_SCHEMA,
            "version": 1,
            "event": event,
            "record_index": epoch["record_count"],
            "execution_uuid": epoch["execution_uuid"],
            "product_id": epoch["product_id"],
            "public_source_boot_id": epoch["public_source_boot_id"],
            "captured_monotonic_s": float(now),
            "public": public_status,
            "child": epoch["latest_child"],
            "child_received_monotonic_s": received,
            "child_age_s": child_age,
            "owned_child_boot_id": int(getattr(self, "_child_mass_stage_boot_id", 0)),
            "owned_child_sequence": (
                int(epoch["latest_child"]["sequence"])
                if epoch["latest_child"] is not None else None),
            "child_consistent": bool(getattr(self, "_child_consistent", False)),
            "authority_open": bool(getattr(self, "_child_status_authority_open", False)),
            "stage_started": bool(getattr(self, "_child_stage_started", False)),
            "stage_loaded_proof": bool(getattr(self, "_child_stage_loaded_proof", False)),
            "terminal_empty_proof": bool(getattr(self, "_child_terminal_empty_proof", False)),
            "fault_latched": bool(getattr(self, "_fault_latched", False)),
            "cancel_requested": bool(getattr(self, "_cancel_requested", threading.Event()).is_set()),
            "observed_history_violations": sorted(epoch["history"]),
        }
        if extra:
            record.update(extra)
        return record

    def _capture_boundary_locked(self, message, now: float):
        epoch = getattr(self, "_ownership_epoch", None)
        if (epoch is None or epoch.get("sealed") or self._active_goal is None or
                not self._child_status_authority_open):
            return None
        event = None
        if message.detail == "Gate 6 mass stage is starting":
            if epoch["first_start_attempted"]:
                return None
            epoch["first_start_attempted"] = True
            event = "START"
        elif (message.state == ManipulatorStatus.STOWED_LOADED and
              bool(getattr(self, "_child_stage_started", False)) and
              int(getattr(self, "_child_boot_id", 0)) ==
              int(getattr(self, "_child_mass_stage_boot_id", -1))):
            if epoch["first_loaded_attempted"]:
                return None
            epoch["first_loaded_attempted"] = True
            event = "LOADED"
        elif (message.state == ManipulatorStatus.STOWED_EMPTY and
              bool(getattr(self, "_child_terminal_empty_proof", False))):
            if epoch["first_empty_attempted"]:
                return None
            event = "EMPTY"
        else:
            return None

        public = self._status_evidence_snapshot(message)
        if public is None:
            epoch["history"].add("public_boundary_snapshot_unavailable")
            return None
        child = epoch["latest_child"]
        received = epoch["latest_child_received_monotonic_s"]
        age = None if received is None else now - received
        fresh_owned_child = all((
            child is not None,
            received is not None,
            age is not None and math.isfinite(age) and 0.0 <= age <= INTERNAL_STATUS_MAX_AGE_S,
            bool(getattr(self, "_child_consistent", False)),
            bool(getattr(self, "_child_valid", False)),
            bool(getattr(self, "_child_stage_started", False)),
            int(getattr(self, "_child_mass_stage_boot_id", 0)) > 0,
            int(getattr(self, "_child_boot_id", 0)) == int(getattr(self, "_child_mass_stage_boot_id", -1)),
            child is not None and child["source_boot_id"] == int(getattr(self, "_child_mass_stage_boot_id", -1)),
            int(message.source_boot_id) == epoch["public_source_boot_id"],
            not bool(getattr(self, "_fault_latched", False)),
            not bool(getattr(self, "_cancel_requested", threading.Event()).is_set()),
            not epoch["history"],
        ))
        if event == "START":
            qualifies = all((
                fresh_owned_child,
                public["valid"], public["state"] == ManipulatorStatus.STARTING,
                not public["base_motion_allowed"], not public["product_attached"],
                public["product_id"] == "", public["detail"] == "Gate 6 mass stage is starting",
                child is not None and child["valid"] and child["state"] == ManipulatorStatus.STARTING,
                child is not None and not child["base_motion_allowed"] and
                not child["product_attached"] and child["product_id"] == "" and
                child["detail"] == "Gate 6 mass stage is starting",
                not bool(getattr(self, "_child_stage_loaded_proof", False)),
                not bool(getattr(self, "_child_terminal_empty_proof", False)),
            ))
        elif event == "LOADED":
            qualifies = all((
                epoch["first_start"] is not None,
                fresh_owned_child,
                public["valid"], public["state"] == ManipulatorStatus.STOWED_LOADED,
                public["base_motion_allowed"], public["product_attached"],
                public["product_id"] == epoch["product_id"],
                child is not None and child["valid"] and child["state"] == ManipulatorStatus.STOWED_LOADED,
                child is not None and child["base_motion_allowed"] and child["product_attached"] and
                child["product_id"] == epoch["product_id"],
                bool(getattr(self, "_child_stage_loaded_proof", False)),
                not bool(getattr(self, "_child_terminal_empty_proof", False)),
            ))
        else:  # first qualifying owned EMPTY boundary
            qualifies = all((
                epoch["first_start"] is not None,
                epoch["first_loaded"] is not None,
                fresh_owned_child,
                public["valid"], public["state"] == ManipulatorStatus.STOWED_EMPTY,
                public["base_motion_allowed"], not public["product_attached"],
                public["product_id"] == "",
                child is not None and child["valid"] and child["state"] == ManipulatorStatus.STOWED_EMPTY,
                child is not None and child["base_motion_allowed"] and
                not child["product_attached"] and child["product_id"] == "",
                bool(getattr(self, "_child_stage_loaded_proof", False)),
                bool(getattr(self, "_child_terminal_empty_proof", False)),
            ))
        if not qualifies:
            if event == "START":
                epoch["history"].add("first_start_boundary_unqualified")
            elif event == "LOADED":
                epoch["history"].add("first_loaded_boundary_unqualified")
            return None
        record = self._ownership_record_locked(epoch, event, now, public_status=public)
        if record is None:
            return None
        identity = {
            "record_index": record["record_index"],
            "source_boot_id": public["source_boot_id"],
            "sequence": public["sequence"],
        }
        if event == "START":
            epoch["first_start"] = identity
        elif event == "LOADED":
            epoch["first_loaded"] = identity
        else:
            epoch["first_empty_attempted"] = True
            epoch["first_empty"] = identity
        return record

    def _capture_close_locked(self, epoch, goal_handle, result, child,
                              child_alive: bool, child_return_code, now: float):
        if epoch is None or epoch.get("sealed"):
            return None
        child_reference_consistent = (
            self._child is child if child_alive else self._child is None)
        result_product = getattr(result, "product_id", None)
        result_outcome = getattr(result, "outcome", None)
        result_delivered = getattr(result, "delivered", None)
        extra = {
            "active_owner_match": self._active_goal is goal_handle,
            "goal_reserved": bool(self._goal_reserved),
            "public_sequence_high_water": int(self._sequence),
            "current_state": int(self._state),
            "current_detail": str(self._detail),
            "current_base_motion_allowed": bool(self._base_motion_allowed),
            "current_product_attached": bool(self._product_attached),
            "current_product_id": str(self._product_id),
            "child_exit_code": child_return_code if type(child_return_code) is int else None,
            "child_alive": bool(child_alive),
            "callback_child_present": child is not None,
            "child_reference_consistent": bool(child_reference_consistent),
            "result_product_id": result_product if type(result_product) is str else None,
            "result_outcome": int(result_outcome) if type(result_outcome) is int else None,
            "result_delivered": result_delivered if type(result_delivered) is bool else None,
            "first_boundaries": {
                "START": epoch["first_start"],
                "LOADED": epoch["first_loaded"],
                "EMPTY": epoch["first_empty"],
            },
            "captured_record_count": epoch["record_count"] + 1,
        }
        record = self._ownership_record_locked(epoch, "CLOSE", now, extra=extra)
        epoch["sealed"] = True
        return record

    def _emit_ownership_record(self, record) -> None:
        """Emit a frozen, bounded observation without affecting action behavior."""
        try:
            encoded = json.dumps(
                record, ensure_ascii=True, allow_nan=False,
                sort_keys=True, separators=(",", ":"))
            text = OWNERSHIP_LOG_PREFIX + encoded
            if len(text.encode("utf-8")) > OWNERSHIP_MAX_RECORD_BYTES:
                return
            self.get_logger().info(text)
        except Exception:  # logging is observational and must remain best effort
            return

    def _emit_ownership_record_safely(self, record) -> None:
        try:
            self._emit_ownership_record(record)
        except Exception:  # defensive boundary around the diagnostic channel
            return

    def _internal_status_callback(self, message: ManipulatorStatus) -> None:
        now = time.monotonic()
        with self._lock:
            if self._active_goal is None:
                return
            self._observe_safely_locked(self._observe_child_before_filters_locked, message)
            if not self._child_status_authority_open or self._fault_latched:
                return
            if message.source_boot_id == 0 or message.sequence == 0:
                return
            # Once the exact mass-stage start marker establishes ownership,
            # delayed preparation (or another stage) status cannot replace the
            # live stage state or its terminal proof.
            if (self._child_stage_started and
                    message.source_boot_id != self._child_mass_stage_boot_id):
                return
            if (message.source_boot_id == self._child_boot_id and
                    message.sequence <= self._child_sequence):
                return
            expected_product = self._product_id
            known_state = message.state in (
                ManipulatorStatus.STARTING,
                ManipulatorStatus.STOWED_EMPTY,
                ManipulatorStatus.STOWED_LOADED,
                ManipulatorStatus.MOVING,
                ManipulatorStatus.DEPLOYED,
                ManipulatorStatus.FAULT,
            )
            valid_state = message.valid == (message.state != ManipulatorStatus.FAULT)
            product_consistent = (
                message.product_id == expected_product if message.product_attached else
                message.product_id == "")
            attachment_state_consistent = (
                message.state != ManipulatorStatus.STOWED_EMPTY or not message.product_attached)
            loaded_state_consistent = (
                message.state != ManipulatorStatus.STOWED_LOADED or message.product_attached)
            base_state_consistent = (
                message.state not in (
                    ManipulatorStatus.STARTING,
                    ManipulatorStatus.MOVING,
                    ManipulatorStatus.DEPLOYED,
                    ManipulatorStatus.FAULT,
                ) or not message.base_motion_allowed)
            consistent = all((
                known_state, valid_state, product_consistent,
                attachment_state_consistent, loaded_state_consistent,
                base_state_consistent))
            stage_start = all((
                consistent,
                message.valid,
                message.state == ManipulatorStatus.STARTING,
                not message.product_attached,
                message.product_id == "",
                not message.base_motion_allowed,
                message.detail == "Gate 6 mass stage is starting",
            ))
            if not self._child_stage_started:
                if stage_start:
                    self._child_stage_started = True
                    self._child_mass_stage_boot_id = message.source_boot_id
                    self._child_stage_loaded_proof = False
                else:
                    if self._child_preparation_boot_id == 0:
                        self._child_preparation_boot_id = message.source_boot_id
                    elif message.source_boot_id != self._child_preparation_boot_id:
                        if not self._is_mass_stage_progress(message, consistent):
                            return
                        self._child_stage_started = True
                        self._child_mass_stage_boot_id = message.source_boot_id
                        self._child_stage_loaded_proof = False

            self._observe_safely_locked(
                self._observe_accepted_child_locked, message, now)

            self._child_boot_id = message.source_boot_id
            self._child_sequence = message.sequence
            self._child_attached = bool(message.product_attached)
            self._child_product_id = message.product_id
            self._child_state = message.state
            self._child_valid = bool(message.valid)
            self._child_consistent = consistent
            self._child_detail = message.detail
            self._child_status_at = now
            if (self._child_stage_started and
                    message.source_boot_id == self._child_mass_stage_boot_id and
                    consistent and message.valid and
                    message.state == ManipulatorStatus.STOWED_LOADED and
                    message.product_attached and
                    message.product_id == expected_product):
                self._child_stage_loaded_proof = True
            # Preparation STOWED_EMPTY is a live status only.  Terminal proof
            # requires the same owned stage boot after a consistent loaded
            # proof; any later accepted status recomputes the proof.
            self._child_terminal_empty_proof = bool(
                self._child_stage_started and
                self._child_stage_loaded_proof and
                message.source_boot_id == self._child_mass_stage_boot_id and
                self._child_consistent and message.valid and
                message.state == ManipulatorStatus.STOWED_EMPTY and
                not message.product_attached and
                message.product_id == "" and message.base_motion_allowed)
            self._product_attached = self._child_attached
            if self._child_attached:
                self._product_id = message.product_id or self._product_id
            self._detail = message.detail
            if not self._child_consistent:
                self._state = ManipulatorStatus.FAULT
                self._base_motion_allowed = False
                self._detail = "inconsistent internal manipulation status"
            else:
                self._state = message.state
                self._base_motion_allowed = bool(message.base_motion_allowed)

    def _publish_status(self) -> None:
        self._request_bootstrap_proof()
        message = ManipulatorStatus()
        ownership_record = None
        with self._lock:
            now = time.monotonic()
            if self._fault_latched:
                self._state = ManipulatorStatus.FAULT
                self._base_motion_allowed = False
                self._child_status_authority_open = False
            elif (self._active_goal is None and not self._goal_reserved and
                  self._child is None and not self._product_attached and
                  not self._child_attached and
                  self._state == ManipulatorStatus.STARTING and
                  self._independent_empty_stow_proven_locked(now)):
                self._state = ManipulatorStatus.STOWED_EMPTY
                self._base_motion_allowed = True
                self._detail = "startup detached and empty-stow proof confirmed"
            if self._active_goal is None:
                effective_base = self._idle_motion_allowed_locked(now)
                self._base_motion_allowed = effective_base
            else:
                effective_base = (self._base_motion_allowed and
                                  self._child_status_authority_open)
                if (self._child_status_at <= 0.0 or
                        now - self._child_status_at > INTERNAL_STATUS_MAX_AGE_S or
                        not self._child_consistent):
                    effective_base = False
                    if self._state != ManipulatorStatus.FAULT:
                        self._detail = "internal manipulation status is stale or inconsistent"
                if self._state not in (
                        ManipulatorStatus.STOWED_EMPTY,
                        ManipulatorStatus.STOWED_LOADED):
                    effective_base = False
            if self._fault_latched:
                self._state = ManipulatorStatus.FAULT
                self._base_motion_allowed = False
                effective_base = False
            message.header.stamp = self.get_clock().now().to_msg()
            message.source_boot_id = self._boot_id
            self._sequence += 1
            message.sequence = self._sequence
            message.valid = self._state != ManipulatorStatus.FAULT and not self._fault_latched
            message.state = self._state
            message.base_motion_allowed = effective_base and message.valid
            message.product_attached = self._product_attached
            message.product_id = (
                self._product_id if self._product_attached or self._state == ManipulatorStatus.FAULT else "")
            message.detail = self._detail
            ownership_record = self._observe_safely_locked(
                self._capture_boundary_locked, message, now)
        self._status_pub.publish(message)
        if ownership_record is not None:
            self._emit_ownership_record_safely(ownership_record)

    def _publish_feedback(self, goal_handle, phase: int, phase_name: str) -> None:
        if not goal_handle.is_active:
            return
        with self._lock:
            product_id = self._product_id
            attached = self._product_attached
        feedback = ExecuteProductCycle.Feedback()
        feedback.phase = phase
        feedback.phase_name = phase_name
        feedback.product_id = product_id
        feedback.product_attached = attached
        goal_handle.publish_feedback(feedback)

    def _request_child_cancel(self) -> bool:
        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("child cancellation service unavailable")
            return False
        future = self._cancel_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                response = future.result()
                return bool(response and response.success)
            time.sleep(0.02)
        self.get_logger().error("child cancellation service timed out")
        return False

    def _terminate_child_fallback(self, child: subprocess.Popen) -> bool:
        if child.poll() is not None:
            return True
        try:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=3.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            self.get_logger().error("child did not stop after SIGINT cleanup fallback")
        return child.poll() is not None

    def _start_child(self, product_id: str, pickup_station: str) -> subprocess.Popen:
        runner = str(self.get_parameter("runner").value)
        internal_status = str(self.get_parameter("internal_status_topic").value)
        internal_cancel = str(self.get_parameter("internal_cancel_service").value)
        command = [
            "ros2", "run", "amr_manipulation", runner,
            "--ros-args",
            "-p", f"product_id:={product_id}",
            "-p", "autonomous_mode:=true",
            "-p", f"pickup_station_id:={pickup_station}",
            "-p", f"status_topic:={internal_status}",
            "-p", f"cancel_service:={internal_cancel}",
        ]
        return subprocess.Popen(command, start_new_session=True)

    def _child_safe_empty_stow(self) -> bool:
        with self._lock:
            # A successful child can be followed by a short ros2-launch
            # wrapper teardown interval.  The bounded proof is recorded only
            # from an owned mass-stage start -> loaded -> empty sequence and
            # is invalidated by any later accepted stage status, so preparation
            # status cannot turn a successful wrapper into delivery proof.
            return (
                self._child_status_at > 0.0 and
                self._child_stage_started and
                self._child_stage_loaded_proof and
                self._child_terminal_empty_proof)

    def _make_result(self, product_id: str, outcome: int, message: str,
                     delivered: bool = False) -> ExecuteProductCycle.Result:
        result = ExecuteProductCycle.Result()
        result.delivered = delivered
        result.outcome = outcome
        result.product_id = product_id
        result.message = message
        return result

    def _set_fault(self, product_id: str, detail: str, attached: Optional[bool] = None) -> None:
        with self._lock:
            self._fault_latched = True
            self._child_status_authority_open = False
            self._state = ManipulatorStatus.FAULT
            self._base_motion_allowed = False
            self._product_attached = self._product_attached if attached is None else attached
            self._product_id = product_id or self._product_id
            self._detail = detail

    def _set_idle_from_proof(self, detail: str) -> bool:
        with self._lock:
            if self._fault_latched:
                return False
            if not self._independent_empty_stow_proven_locked():
                self._set_fault(
                    "", f"{detail}; current independent empty-stow proof refused")
                return False
            if self._product_attached or self._child_attached:
                self._set_fault("", f"{detail}; product attachment retained")
                return False
            self._state = ManipulatorStatus.STOWED_EMPTY
            self._base_motion_allowed = True
            self._product_attached = False
            self._product_id = ""
            self._detail = detail
            self._child_status_authority_open = False
            return True

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        product_id = self._product_by_station.get(goal.pickup_station_id, "")
        child: Optional[subprocess.Popen] = None
        result = None
        child_return_code = None
        ownership_epoch = None
        cancel_sent = False
        cooperative_cancel_proven = False
        cancel_sent_at = 0.0
        try:
            if not self._mapping_ready or not product_id or \
                    goal.destination_station_id not in self._dispatch_station_ids:
                result = self._make_result(
                    product_id, ExecuteProductCycle.Result.INVALID_REQUEST,
                    "validated autonomous station mapping rejected the goal")
                with self._lock:
                    self._goal_reserved = False
                goal_handle.abort()
                return result
            with self._lock:
                self._active_goal = goal_handle
                self._child_status_authority_open = True
                self._product_id = product_id
                self._state = ManipulatorStatus.STARTING
                self._base_motion_allowed = False
                self._product_attached = False
                self._detail = "cycle accepted; waiting for Gate 6 child"
                self._child_boot_id = 0
                self._child_sequence = 0
                self._child_status_at = 0.0
                self._child_valid = False
                self._child_consistent = False
                self._child_terminal_empty_proof = False
                self._child_preparation_boot_id = 0
                self._child_mass_stage_boot_id = 0
                self._child_stage_started = False
                self._child_stage_loaded_proof = False
                self._child_attached = False
                self._child_product_id = ""
                try:
                    ownership_epoch = self._new_ownership_epoch(
                        goal_handle, product_id, self._boot_id)
                except Exception:
                    ownership_epoch = None
                self._ownership_epoch = ownership_epoch
            self._publish_feedback(
                goal_handle, ExecuteProductCycle.Feedback.PREPARING, "PREPARING")
            if self._cancel_requested.is_set():
                self._publish_feedback(
                    goal_handle, ExecuteProductCycle.Feedback.CANCELING, "CANCELING")
                independent_proof = self._wait_fresh_independent_empty_stow()
                with self._lock:
                    attached = self._product_attached or self._child_attached
                if independent_proof and not attached and self._set_idle_from_proof(
                        "cycle canceled before Gate 6 child start"):
                    result = self._make_result(
                        product_id, ExecuteProductCycle.Result.CANCELED,
                        "cycle canceled before Gate 6 child start")
                else:
                    detail = "canceled before Gate 6 child start without fresh empty-stow proof"
                    self._set_fault(product_id, detail, attached=attached)
                    result = self._make_result(
                        product_id, ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT, detail)
                if goal_handle.is_active:
                    goal_handle.canceled()
                return result
            child = self._start_child(product_id, goal.pickup_station_id)
            with self._lock:
                self._child = child
            while rclpy.ok() and child.poll() is None:
                if self._cancel_requested.is_set():
                    self._publish_feedback(
                        goal_handle, ExecuteProductCycle.Feedback.CANCELING, "CANCELING")
                    with self._lock:
                        self._state = ManipulatorStatus.MOVING
                        self._base_motion_allowed = False
                        self._detail = "cooperative cancellation requested"
                    if not cancel_sent:
                        cancel_sent = True
                        cancel_sent_at = time.monotonic()
                        cooperative_cancel_proven = self._request_child_cancel()
                        if not cooperative_cancel_proven:
                            self._terminate_child_fallback(child)
                    elif (time.monotonic() - cancel_sent_at > 5.0 and
                          cooperative_cancel_proven):
                        # An acknowledged request is not proof of termination;
                        # use SIGINT only as the bounded cleanup fallback and
                        # retain a fault because cooperative completion failed.
                        cooperative_cancel_proven = False
                        self._terminate_child_fallback(child)
                else:
                    self._publish_feedback(
                        goal_handle, ExecuteProductCycle.Feedback.EXECUTING, "EXECUTING")
                time.sleep(0.05)
            return_code = child.wait()
            child_return_code = return_code
            canceled = self._cancel_requested.is_set() or return_code == 130
            independent_proof = self._wait_fresh_independent_empty_stow()
            child_empty_proof = self._child_safe_empty_stow()

            if canceled:
                if (not cooperative_cancel_proven or not independent_proof or
                        self._product_attached or self._child_attached):
                    detail = "canceled without cooperative termination and fresh empty-stow proof"
                    self._set_fault(product_id, detail)
                    result = self._make_result(
                        product_id, ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT, detail)
                else:
                    detail = "cycle canceled with fresh detached empty-stow proof"
                    if self._set_idle_from_proof(detail):
                        result = self._make_result(
                            product_id, ExecuteProductCycle.Result.CANCELED,
                            "cycle canceled before product retention")
                    else:
                        result = self._make_result(
                            product_id, ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT,
                            detail)
                if goal_handle.is_active:
                    goal_handle.canceled()
                return result

            if return_code == 0:
                if not independent_proof or not child_empty_proof:
                    detail = "child succeeded without fresh empty-stow and detached proof"
                    self._set_fault(product_id, detail)
                    result = self._make_result(
                        product_id, ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT, detail)
                    if goal_handle.is_active:
                        goal_handle.abort()
                    return result
                detail = "product cycle delivered and empty-stowed"
                if not self._set_idle_from_proof(detail):
                    result = self._make_result(
                        product_id, ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT,
                        detail)
                    if goal_handle.is_active:
                        goal_handle.abort()
                    return result
                result = self._make_result(
                    product_id, ExecuteProductCycle.Result.SUCCESS,
                    detail, delivered=True)
                if goal_handle.is_active:
                    goal_handle.succeed()
                return result

            with self._lock:
                attached = self._product_attached or self._child_attached
            if return_code == 2:
                outcome = ExecuteProductCycle.Result.PREPARATION_FAILED
            elif return_code == 1:
                outcome = ExecuteProductCycle.Result.EXECUTION_FAILED
            elif return_code == 127:
                outcome = ExecuteProductCycle.Result.DEPENDENCY_UNAVAILABLE
            else:
                outcome = (ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT
                           if attached else ExecuteProductCycle.Result.EXECUTION_FAILED)
            detail = f"Gate 6 child exited with status {return_code}"
            idle_refused = False
            if independent_proof and not attached:
                idle_refused = not self._set_idle_from_proof(detail)
            else:
                self._set_fault(product_id, detail, attached=attached)
            if idle_refused:
                outcome = ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT
            result = self._make_result(product_id, outcome, detail)
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        except FileNotFoundError as error:
            detail = f"Gate 6 runner dependency is unavailable: {error}"
            self._set_fault(product_id, detail)
            result = self._make_result(
                product_id, ExecuteProductCycle.Result.DEPENDENCY_UNAVAILABLE, detail)
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        except Exception as error:  # fail closed at the public action boundary
            self.get_logger().error(f"cycle adapter failed: {error}")
            self._set_fault(product_id, str(error))
            result = self._make_result(
                product_id, ExecuteProductCycle.Result.EXECUTION_FAILED, str(error))
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        finally:
            child_alive = child is not None and child.poll() is None
            if child_alive and child is not None:
                self._terminate_child_fallback(child)
                child_alive = child.poll() is None
            close_record = None
            with self._lock:
                self._child_status_authority_open = False
                if child_alive:
                    # Keep ownership visible until the process has really
                    # exited; admission remains blocked by _child != None.
                    self._child = child
                    self._fault_latched = True
                    self._state = ManipulatorStatus.FAULT
                    self._base_motion_allowed = False
                    self._detail = "Gate 6 child remains alive after cleanup fallback"
                elif self._child is child:
                    self._child = None
                try:
                    close_record = self._capture_close_locked(
                        ownership_epoch, goal_handle, result, child, child_alive,
                        child_return_code, time.monotonic())
                except Exception:
                    self._mark_ownership_observation_unavailable_locked()
                if getattr(self, "_ownership_epoch", None) is ownership_epoch:
                    self._ownership_epoch = None
                if not child_alive:
                    self._goal_reserved = False
                    if self._active_goal is goal_handle:
                        self._active_goal = None
                    self._cancel_requested.clear()
                    if self._state != ManipulatorStatus.FAULT:
                        self._product_id = ""
            if close_record is not None:
                self._emit_ownership_record_safely(close_record)


def main() -> None:
    rclpy.init()
    node = CycleSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
