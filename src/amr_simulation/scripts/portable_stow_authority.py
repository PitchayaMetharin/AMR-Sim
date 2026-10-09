#!/usr/bin/env python3
"""Portable empty-stow authority for the arm-equipped simulation.

This process is deliberately independent of the factory manipulation
supervisor.  It owns the portable runtime's sole public manipulation status,
performs one bounded arm trajectory, and opens base-motion permission only
after fresh, independently received base and joint evidence proves the empty
stow.
"""

from __future__ import annotations

import math
import inspect
import time
import uuid
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID
from amr_interfaces.msg import BaseStatus, ManipulatorStatus


STOW_POSITION = {
    "arm_joint_1": 0.0,
    "arm_joint_2": -1.5708,
    "arm_joint_3": 1.5708,
    "arm_joint_4": 0.0,
    "arm_joint_5": 0.0,
    "arm_joint_6": 0.0,
}
STOW_TOLERANCE_RAD = 0.01
BASE_STATUS_MAX_AGE_S = 0.2
JOINT_STATUS_MAX_AGE_S = 2.0
GOAL_RESPONSE_TIMEOUT_S = 3.0
RESULT_QUERY_TIMEOUT_S = 5.0
MAX_GOAL_SEND_ATTEMPTS = 3
NON_TERMINAL_RESULT_STATUSES = frozenset(
    (
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    )
)


def authority_qos() -> QoSProfile:
    """The public status authority QoS: reliable, volatile, depth one, 100 ms."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=0.1),
    )


def state_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def accept_base_sequence(message: BaseStatus, last_boot: int, last_sequence: int) -> bool:
    """Return whether a status starts a new valid boot or advances its boot."""

    try:
        boot = int(message.source_boot_id)
        sequence = int(message.sequence)
    except (AttributeError, TypeError, ValueError):
        return False
    if boot <= 0 or sequence <= 0:
        return False
    return boot != int(last_boot) or sequence > int(last_sequence)


def valid_fresh_base(
    message: BaseStatus,
    received_at: float,
    now: float,
    max_age: float = BASE_STATUS_MAX_AGE_S,
) -> bool:
    """Validate semantic READY evidence and its steady-clock receipt age."""

    try:
        age = float(now) - float(received_at)
        return bool(
            message.valid
            and message.state == BaseStatus.READY
            and message.reason == BaseStatus.REASON_READY
            and int(message.source_boot_id) > 0
            and int(message.sequence) > 0
            and 0.0 <= age <= float(max_age)
        )
    except (AttributeError, TypeError, ValueError):
        return False


def valid_fresh_joints(message: JointState) -> bool:
    """Validate a complete finite arm sample against the inclusive tolerance."""

    try:
        names = list(message.name)
        positions = list(message.position)
    except (AttributeError, TypeError):
        return False
    if len(names) != len(positions) or len(set(names)) != len(names):
        return False
    measured = {}
    for name, value in zip(names, positions):
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(numeric):
            return False
        measured[name] = numeric
    for name, target in STOW_POSITION.items():
        if name not in measured or abs(measured[name] - target) > STOW_TOLERANCE_RAD + 1e-12:
            return False
    return True


class PortableStowAuthority(Node):
    """Publish one fail-closed portable empty-stow authority."""

    def __init__(self) -> None:
        super().__init__("portable_stow_authority")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self._boot_id = int(time.monotonic_ns() & 0xFFFFFFFF) or 1
        self._sequence = 0
        self._state = ManipulatorStatus.STARTING
        self._detail = "waiting for one arm stow trajectory"
        self._fault_latched = False
        self._trajectory_sent = False
        self._goal_send_attempts = 0
        self._superseded_goal_futures = []
        self._trajectory_succeeded = False
        self._trajectory_sent_at = 0.0
        self._joint_after_trajectory = False
        self._joint_valid = False
        self._joint_states: Optional[JointState] = None
        self._joint_received_at = 0.0
        self._base_valid = False
        self._base_status: Optional[BaseStatus] = None
        self._base_received_at = 0.0
        self._last_base_boot = 0
        self._last_base_sequence = 0
        self._goal_handle = None
        self._goal_uuid: Optional[UUID] = None
        self._goal_response_future = None
        self._goal_response_sent_at = 0.0
        self._result_query_future = None
        self._result_query_sent_at = 0.0
        self._result_query_is_recovery = False
        self._result_recovery_next_at = 0.0

        self._status_pub = self.create_publisher(
            ManipulatorStatus,
            "/amr/manipulation/status",
            authority_qos(),
        )
        self._joint_sub = self.create_subscription(
            JointState,
            "/amr/base/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self._base_sub = self.create_subscription(
            BaseStatus,
            "/amr/base/status",
            self._base_status_callback,
            state_qos(),
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._result_client = self.create_client(
            FollowJointTrajectory.Impl.GetResultService,
            "/arm_controller/follow_joint_trajectory/_action/get_result",
        )
        self._status_timer = self.create_timer(0.05, self._tick)
        self._tick()

    def _revoke_proof(self, detail: str) -> None:
        if self._fault_latched:
            return
        self._state = ManipulatorStatus.STARTING
        self._detail = detail

    def _latch_fault(self, detail: str) -> None:
        if self._fault_latched:
            return
        self._fault_latched = True
        self._state = ManipulatorStatus.FAULT
        self._detail = detail
        self.get_logger().error(f"portable stow authority FAULT: {detail}")

    def _joint_state_callback(self, message: JointState) -> None:
        received_at = time.monotonic()
        valid = valid_fresh_joints(message)
        self._joint_states = message
        self._joint_received_at = received_at
        self._joint_valid = valid
        self._joint_after_trajectory = bool(
            valid and self._trajectory_sent and received_at > self._trajectory_sent_at
        )
        if not valid:
            self._revoke_proof("joint evidence malformed, incomplete, non-finite, or out of tolerance")
        elif self._state == ManipulatorStatus.STOWED_EMPTY and not self._joint_after_trajectory:
            self._revoke_proof("joint evidence is not newly measured after the stow trajectory")

    def _base_status_callback(self, message: BaseStatus) -> None:
        received_at = time.monotonic()
        if not accept_base_sequence(message, self._last_base_boot, self._last_base_sequence):
            self._base_valid = False
            self._revoke_proof("base READY evidence replayed or malformed")
            return
        self._last_base_boot = int(message.source_boot_id)
        self._last_base_sequence = int(message.sequence)
        self._base_status = message
        self._base_received_at = received_at
        self._base_valid = valid_fresh_base(message, received_at, received_at)
        if not self._base_valid:
            self._revoke_proof("base READY evidence invalid")

    def _make_goal(self) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(STOW_POSITION)
        point = JointTrajectoryPoint()
        point.positions = list(STOW_POSITION.values())
        point.time_from_start = Duration(seconds=2.0).to_msg()
        goal.trajectory.points = [point]
        return goal

    def _send_goal_with_uuid(self, goal: FollowJointTrajectory.Goal, goal_uuid: UUID):
        """Send the one production goal with an explicit, recoverable UUID."""

        submit_goal = self._trajectory_client.send_goal_async
        try:
            supports_goal_uuid = "goal_uuid" in inspect.signature(submit_goal).parameters
        except (TypeError, ValueError):
            supports_goal_uuid = False
        if supports_goal_uuid:
            return submit_goal(goal, goal_uuid=goal_uuid)
        # Keep lightweight contract-test clients usable without weakening the
        # real rclpy ActionClient path above.
        if isinstance(self._trajectory_client, ActionClient):
            raise RuntimeError("arm action client cannot bind an explicit goal UUID")
        return submit_goal(goal)

    def _send_trajectory_once(self) -> None:
        if self._trajectory_sent or self._fault_latched:
            return
        try:
            if not self._trajectory_client.server_is_ready():
                return
        except Exception as error:
            self._latch_fault(f"arm action server readiness exception: {error}")
            return
        # Set this before sending so an exception or rejected goal can never
        # cause a retry in a later timer tick.
        self._trajectory_sent = True
        self._goal_send_attempts = getattr(self, "_goal_send_attempts", 0) + 1
        self._trajectory_sent_at = time.monotonic()
        try:
            self._goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            future = self._send_goal_with_uuid(self._make_goal(), self._goal_uuid)
            self._goal_response_future = future
            self._goal_response_sent_at = self._trajectory_sent_at
            future.add_done_callback(self._goal_response_callback)
        except Exception as error:
            self._latch_fault(f"arm trajectory send exception: {error}")

    def _recover_unacknowledged_goal(self, now: float) -> None:
        """Recover one lost goal acknowledgement through the same UUID."""

        if self._fault_latched or self._trajectory_succeeded:
            return
        response_future = getattr(self, "_goal_response_future", None)
        if response_future is None:
            return
        done = getattr(response_future, "done", None)
        if callable(done) and done():
            return
        elapsed = now - self._goal_response_sent_at
        if elapsed < GOAL_RESPONSE_TIMEOUT_S:
            return
        if now < getattr(self, "_result_recovery_next_at", 0.0):
            return
        if getattr(self, "_result_query_future", None) is not None:
            return
        if self._goal_uuid is None:
            self._latch_fault("arm trajectory acknowledgement has no recoverable UUID")
            return
        result_client = getattr(self, "_result_client", None)
        if result_client is None:
            if elapsed >= GOAL_RESPONSE_TIMEOUT_S + RESULT_QUERY_TIMEOUT_S:
                self._latch_fault("arm trajectory result recovery client unavailable")
            else:
                self._detail = "arm trajectory acknowledgement pending; awaiting result recovery"
            return
        try:
            if not result_client.service_is_ready():
                if elapsed >= GOAL_RESPONSE_TIMEOUT_S + RESULT_QUERY_TIMEOUT_S:
                    self._latch_fault("arm trajectory result recovery service unavailable")
                else:
                    self._detail = "arm trajectory acknowledgement pending; result service unavailable"
                return
            request = FollowJointTrajectory.Impl.GetResultService.Request()
            request.goal_id = self._goal_uuid
            result_future = result_client.call_async(request)
            self._result_query_future = result_future
            self._result_query_sent_at = now
            self._result_query_is_recovery = True
            result_future.add_done_callback(self._result_callback)
        except Exception as error:
            self._latch_fault(f"arm trajectory result recovery exception: {error}")

    def _check_result_query_timeout(self, now: float) -> None:
        result_future = getattr(self, "_result_query_future", None)
        if (
            result_future is None
            or not getattr(self, "_result_query_is_recovery", False)
            or self._fault_latched
            or self._trajectory_succeeded
        ):
            return
        done = getattr(result_future, "done", None)
        if callable(done) and done():
            return
        if now - self._result_query_sent_at >= RESULT_QUERY_TIMEOUT_S:
            self._latch_fault("arm trajectory result recovery response timeout")

    def _goal_response_callback(self, future) -> None:
        if self._fault_latched or self._trajectory_succeeded:
            return
        # A late response for a superseded attempt must not become current.
        if any(future is old for old in getattr(self, "_superseded_goal_futures", ())):
            return
        if getattr(self, "_result_query_future", None) is not None:
            return
        self._goal_response_future = None
        try:
            goal_handle = future.result()
        except Exception as error:
            self._latch_fault(f"arm trajectory goal exception: {error}")
            return
        if goal_handle is None or not bool(getattr(goal_handle, "accepted", False)):
            self._latch_fault("arm trajectory goal REJECTED")
            return
        self._goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._result_query_future = result_future
            self._result_query_sent_at = time.monotonic()
            self._result_query_is_recovery = False
            result_future.add_done_callback(self._result_callback)
        except Exception as error:
            self._latch_fault(f"arm trajectory result exception: {error}")

    def _result_callback(self, future) -> None:
        if self._fault_latched or self._trajectory_succeeded:
            return
        current_future = getattr(self, "_result_query_future", None)
        if current_future is not None and current_future is not future:
            return
        is_recovery = bool(getattr(self, "_result_query_is_recovery", False))
        if hasattr(self, "_result_query_future"):
            self._result_query_future = None
            self._result_query_is_recovery = False
        try:
            wrapped = future.result()
            if not hasattr(wrapped, "status"):
                self._latch_fault("arm trajectory result omitted status")
                return
            status = int(wrapped.status)
            result = getattr(wrapped, "result", wrapped)
            if not hasattr(result, "error_code"):
                self._latch_fault("arm trajectory result omitted error_code")
                return
            error_code = int(result.error_code)
        except Exception as error:
            self._latch_fault(f"arm trajectory result exception: {error}")
            return
        if is_recovery and status == GoalStatus.STATUS_UNKNOWN:
            # The server never created this goal (goal response was lost in
            # discovery), so the arm cannot have moved. Allow a bounded resend.
            attempts = getattr(self, "_goal_send_attempts", 1)
            if attempts >= MAX_GOAL_SEND_ATTEMPTS:
                self._latch_fault(
                    "arm trajectory goal was never acknowledged after "
                    f"{attempts} attempts"
                )
                return
            self.get_logger().warning(
                f"arm trajectory goal attempt {attempts} was never created by the "
                "server; resending with a new goal UUID"
            )
            old_future = getattr(self, "_goal_response_future", None)
            if old_future is not None:
                self._superseded_goal_futures = list(
                    getattr(self, "_superseded_goal_futures", [])
                ) + [old_future]
            self._goal_response_future = None
            self._goal_uuid = None
            self._goal_handle = None
            self._result_recovery_next_at = 0.0
            self._trajectory_sent = False
            self._detail = "arm trajectory goal not created; resending"
            return
        if status in NON_TERMINAL_RESULT_STATUSES:
            self._detail = "arm trajectory still executing; awaiting terminal result"
            if is_recovery:
                self._result_recovery_next_at = (
                    time.monotonic() + RESULT_QUERY_TIMEOUT_S
                )
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            outcome = {
                GoalStatus.STATUS_CANCELED: "CANCELED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
            }.get(status, "FAILED")
            self._latch_fault(f"arm trajectory {outcome}")
            return
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._latch_fault(f"arm trajectory returned non-success error_code={error_code}")
            return
        self._trajectory_succeeded = True
        self._detail = "arm trajectory succeeded; awaiting fresh empty-stow proof"

    def _proof_is_fresh(self, now: float) -> bool:
        if self._base_status is None or not self._base_valid:
            return False
        if not valid_fresh_base(
            self._base_status,
            self._base_received_at,
            now,
            BASE_STATUS_MAX_AGE_S,
        ):
            return False
        if (
            self._joint_states is None
            or not self._joint_valid
            or not self._joint_after_trajectory
            or self._joint_received_at > now
            or now - self._joint_received_at > JOINT_STATUS_MAX_AGE_S
        ):
            return False
        return True

    def _refresh_state(self, now: float) -> None:
        if self._fault_latched:
            self._state = ManipulatorStatus.FAULT
            return
        if self._trajectory_succeeded and self._proof_is_fresh(now):
            self._state = ManipulatorStatus.STOWED_EMPTY
            self._detail = "fresh READY base and measured empty stow confirmed"
            return
        if self._state == ManipulatorStatus.STOWED_EMPTY:
            self._revoke_proof("fresh empty-stow proof lost")
        elif not self._trajectory_succeeded:
            self._state = ManipulatorStatus.STARTING

    def _publish_status(self) -> None:
        message = ManipulatorStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.source_boot_id = self._boot_id
        self._sequence += 1
        message.sequence = self._sequence
        message.state = self._state
        message.valid = bool(self._state == ManipulatorStatus.STOWED_EMPTY)
        message.base_motion_allowed = bool(message.valid and not self._fault_latched)
        product_attached = False
        message.product_attached = product_attached
        message.product_id = ""
        message.detail = self._detail
        self._status_pub.publish(message)

    def _tick(self) -> None:
        if self._fault_latched:
            self._publish_status()
            return
        self._send_trajectory_once()
        now = time.monotonic()
        self._recover_unacknowledged_goal(now)
        self._check_result_query_timeout(now)
        self._refresh_state(now)
        self._publish_status()


def main() -> None:
    rclpy.init()
    node = PortableStowAuthority()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


# Compatibility aliases keep the public process easy to exercise from launch
# and contract tests without introducing another authority implementation.
StowAuthority = PortableStowAuthority
PortableStowAuthorityNode = PortableStowAuthority
