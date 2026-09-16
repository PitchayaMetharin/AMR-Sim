"""Contract tests for the portable one-shot stow authority."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from action_msgs.msg import GoalStatus
from amr_interfaces.msg import BaseStatus, ManipulatorStatus
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "portable_stow_authority.py"


def _load():
    spec = importlib.util.spec_from_file_location("portable_stow", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _base(boot=11, sequence=1, valid=True, state=BaseStatus.READY, reason=BaseStatus.REASON_READY):
    return BaseStatus(
        header=Header(stamp=Time()),
        source_boot_id=boot,
        sequence=sequence,
        valid=valid,
        state=state,
        reason=reason,
    )


def _joints(values=None):
    module = _load()
    values = values or list(module.STOW_POSITION.values())
    return JointState(name=list(module.STOW_POSITION), position=values)


def test_stow_constants_and_qos_contract_are_explicit():
    module = _load()
    assert list(module.STOW_POSITION) == [
        "arm_joint_1", "arm_joint_2", "arm_joint_3",
        "arm_joint_4", "arm_joint_5", "arm_joint_6",
    ]
    assert list(module.STOW_POSITION.values()) == [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0]
    assert module.STOW_TOLERANCE_RAD == pytest.approx(0.01)
    authority = module.authority_qos()
    assert authority.depth == 1
    assert authority.reliability.value == 1  # RELIABLE
    assert authority.durability.value == 2  # VOLATILE
    assert authority.deadline.nanoseconds == 100000000


def test_fresh_base_and_complete_joint_proof_accepts_inclusive_tolerance():
    module = _load()
    now = 10.0
    assert module.valid_fresh_base(_base(), received_at=now, now=now)
    assert module.valid_fresh_joints(_joints([0.01, -1.5608, 1.5808, 0.01, -0.01, 0.01]))
    assert module.valid_fresh_joints(_joints([0.011, -1.5708, 1.5708, 0.0, 0.0, 0.0])) is False


@pytest.mark.parametrize(
    "message",
    [
        _base(valid=False),
        _base(state=BaseStatus.UNKNOWN),
        _base(reason=BaseStatus.REASON_UNAVAILABLE),
        _base(boot=0),
        _base(sequence=0),
    ],
)
def test_invalid_base_proof_is_denied(message):
    module = _load()
    assert not module.valid_fresh_base(message, received_at=10.0, now=10.0)
    assert not module.valid_fresh_base(message, received_at=9.0, now=10.0)


@pytest.mark.parametrize(
    "message",
    [
        JointState(name=["arm_joint_1"], position=[]),
        JointState(name=["arm_joint_1", "arm_joint_1"], position=[0.0, 0.0]),
        JointState(name=["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6"], position=[0.0, float("nan"), 1.5708, 0.0, 0.0, 0.0]),
    ],
)
def test_malformed_joint_proof_is_denied(message):
    assert not _load().valid_fresh_joints(message)


def test_source_contract_latches_fault_and_never_retries_after_one_action():
    text = SCRIPT.read_text()
    assert "portable_stow_authority" in text
    assert "FollowJointTrajectory" in text
    assert text.count("send_goal_async") == 1
    assert "_trajectory_sent" in text
    assert "_fault_latched" in text
    assert "STOWED_LOADED" not in text
    assert "product_attached = False" in text
    for marker in ("REJECTED", "ABORTED", "CANCELED", "error_code", "exception"):
        assert marker.lower() in text.lower()


def test_source_contract_revokes_proof_and_requires_strict_status_identity():
    module = _load()
    assert module.valid_fresh_base(_base(boot=5, sequence=3), 1.0, 1.0)
    assert not module.accept_base_sequence(_base(boot=5, sequence=3), 5, 3)
    assert module.accept_base_sequence(_base(boot=5, sequence=4), 5, 3)
    assert module.accept_base_sequence(_base(boot=6, sequence=1), 5, 4)
    text = SCRIPT.read_text()
    assert "STARTING" in text
    assert "base_motion_allowed" in text
    assert "now -" in text


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Future:
    def __init__(self, value):
        self.value = value
        self.callbacks = []

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


def _bare_authority():
    module = _load()
    authority = module.PortableStowAuthority.__new__(module.PortableStowAuthority)
    authority._state = ManipulatorStatus.STARTING
    authority._detail = "starting"
    authority._fault_latched = False
    authority._trajectory_sent = False
    authority._trajectory_succeeded = False
    authority._trajectory_sent_at = 0.0
    authority._joint_after_trajectory = False
    authority._joint_valid = False
    authority._joint_states = None
    authority._joint_received_at = 0.0
    authority._base_valid = False
    authority._base_status = None
    authority._base_received_at = 0.0
    authority._last_base_boot = 0
    authority._last_base_sequence = 0
    authority._goal_handle = None
    authority._logger = _Logger()
    authority._status_pub = _Publisher()
    authority._boot_id = 77
    authority._sequence = 0
    authority._clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    authority.get_clock = lambda: authority._clock
    return authority


def test_one_send_is_rejected_or_failed_without_retry_and_success_needs_proof():
    module = _load()
    authority = _bare_authority()

    class _Client:
        def __init__(self):
            self.calls = 0

        def server_is_ready(self):
            return True

        def send_goal_async(self, _goal):
            self.calls += 1
            return _Future(SimpleNamespace(accepted=False))

    authority._trajectory_client = _Client()
    with patch.object(module.time, "monotonic", return_value=10.0):
        authority._send_trajectory_once()
        authority._send_trajectory_once()
    assert authority._trajectory_client.calls == 1
    assert authority._trajectory_sent
    authority._goal_response_callback(_Future(SimpleNamespace(accepted=False)))
    assert authority._fault_latched
    assert authority._state == ManipulatorStatus.FAULT

    authority = _bare_authority()
    authority._trajectory_sent = True
    authority._trajectory_succeeded = True
    authority._trajectory_sent_at = 9.0
    with patch.object(module.time, "monotonic", return_value=10.0):
        authority._base_status_callback(_base(boot=1, sequence=1))
        authority._joint_state_callback(_joints())
    authority._refresh_state(10.1)
    assert authority._state == ManipulatorStatus.STOWED_EMPTY
    assert authority._status_pub.messages == []
    authority._publish_status()
    assert authority._status_pub.messages[-1].valid
    assert authority._status_pub.messages[-1].base_motion_allowed


@pytest.mark.parametrize(
    "result",
    [
        SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=SimpleNamespace(error_code=0)),
        SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=SimpleNamespace(error_code=0)),
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace(error_code=1)),
    ],
)
def test_cancel_abort_and_non_success_result_latch_fault(result):
    authority = _bare_authority()
    authority._result_callback(_Future(result))
    assert authority._fault_latched
    assert authority._state == ManipulatorStatus.FAULT


def test_stale_drift_and_replayed_proof_revoke_to_starting_and_recover():
    module = _load()
    authority = _bare_authority()
    authority._trajectory_sent = True
    authority._trajectory_succeeded = True
    authority._trajectory_sent_at = 9.0
    with patch.object(module.time, "monotonic", return_value=10.0):
        authority._base_status_callback(_base(boot=1, sequence=1))
        authority._joint_state_callback(_joints())
    authority._refresh_state(10.0)
    assert authority._state == ManipulatorStatus.STOWED_EMPTY

    with patch.object(module.time, "monotonic", return_value=10.1):
        authority._joint_state_callback(_joints([0.02, -1.5708, 1.5708, 0.0, 0.0, 0.0]))
    assert authority._state == ManipulatorStatus.STARTING
    assert not authority._joint_valid

    with patch.object(module.time, "monotonic", return_value=10.2):
        authority._base_status_callback(_base(boot=1, sequence=1))
    assert authority._state == ManipulatorStatus.STARTING
    assert not authority._base_valid
    assert not authority._proof_is_fresh(11.0)

    with patch.object(module.time, "monotonic", return_value=10.3):
        authority._base_status_callback(_base(boot=1, sequence=2))
        authority._joint_state_callback(_joints())
    authority._refresh_state(10.3)
    assert authority._state == ManipulatorStatus.STOWED_EMPTY


def test_status_sequence_is_strictly_increasing_on_one_boot():
    authority = _bare_authority()
    authority._publish_status()
    authority._publish_status()
    messages = authority._status_pub.messages
    assert messages[0].source_boot_id == messages[1].source_boot_id != 0
    assert messages[1].sequence > messages[0].sequence > 0
