import importlib.util
import inspect
import math
import subprocess
import sys
import threading
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from amr_interfaces.action import ExecuteProductCycle
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action.server import ServerGoalHandle
from rclpy.exceptions import (InvalidParameterTypeException,
                               ParameterUninitializedException)
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ADAPTER_PATH = PACKAGE_ROOT / "scripts" / "cycle_manipulation_supervisor.py"
ADAPTER_SPEC = importlib.util.spec_from_file_location(
    "cycle_manipulation_supervisor_contract", ADAPTER_PATH)
ADAPTER = importlib.util.module_from_spec(ADAPTER_SPEC)
sys.modules[ADAPTER_SPEC.name] = ADAPTER
ADAPTER_SPEC.loader.exec_module(ADAPTER)


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


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time())


class _Client:
    def service_is_ready(self):
        return False


class _ActionServer:
    def __init__(self, *args, **kwargs):
        self.execute_callback = kwargs["execute_callback"]


def _fake_node_init(overrides, predeclared_use_sim_time):
    def fake_init(node, node_name, **kwargs):
        node._parameters = {}
        node._publishers = []
        node._subscriptions = []
        node._clients = []
        node._timers = []
        node._parameters_callbacks = []
        node._parameter_overrides = dict(overrides)
        node._descriptors = {}
        node._allow_undeclared_parameters = False
        node._clock = _Clock()
        node._parameter_event_publisher = _Publisher()
        node._logger = _Logger()
        if predeclared_use_sim_time is not None:
            node._parameters["use_sim_time"] = Parameter(
                "use_sim_time", value=predeclared_use_sim_time)
            node._descriptors["use_sim_time"] = ParameterDescriptor(
                type=Parameter.Type.BOOL.value)

    return fake_init


def _construct_adapter(overrides, predeclared_use_sim_time=None):
    graph_patches = {
        "create_publisher": lambda *args, **kwargs: _Publisher(),
        "create_subscription": lambda *args, **kwargs: object(),
        "create_client": lambda *args, **kwargs: _Client(),
        "create_timer": lambda *args, **kwargs: object(),
        "get_namespace": lambda self: "/",
        "get_name": lambda self: "manipulation_supervisor_node",
    }
    with patch.object(
            Node, "__init__",
            _fake_node_init(overrides, predeclared_use_sim_time)), \
            patch.object(Node, "create_publisher", graph_patches["create_publisher"]), \
            patch.object(Node, "create_subscription", graph_patches["create_subscription"]), \
            patch.object(Node, "create_client", graph_patches["create_client"]), \
            patch.object(Node, "create_timer", graph_patches["create_timer"]), \
            patch.object(Node, "get_namespace", graph_patches["get_namespace"]), \
            patch.object(Node, "get_name", graph_patches["get_name"]), \
            patch.object(ADAPTER, "ActionServer", _ActionServer):
        return ADAPTER.CycleSupervisor()


def _valid_overrides(use_sim_time):
    return {
        "use_sim_time": Parameter("use_sim_time", value=use_sim_time),
        "product_station_map": Parameter(
            "product_station_map", value=["pickup_a=101", "pickup_b=102"]),
        "autonomous_product_ids": Parameter(
            "autonomous_product_ids", value=[101, 102]),
        "dispatch_station_ids": Parameter(
            "dispatch_station_ids", value=["dispatch"]),
    }


def _admissible_goal_adapter(allow_product_cycles=True):
    overrides = _valid_overrides(False)
    overrides["allow_product_cycles"] = Parameter(
        "allow_product_cycles", value=allow_product_cycles)
    adapter = _construct_adapter(overrides, predeclared_use_sim_time=False)
    now = ADAPTER.time.monotonic()
    adapter._bootstrap_detached = True
    adapter._bootstrap_proof_at = now
    adapter._joint_states = JointState(
        name=list(ADAPTER.STOW), position=list(ADAPTER.STOW.values()))
    adapter._joint_states_at = now
    adapter._state = ADAPTER.ManipulatorStatus.STOWED_EMPTY
    adapter._base_motion_allowed = True
    adapter._product_attached = False
    adapter._fault_latched = False
    return adapter


def _valid_cycle_goal():
    goal = ExecuteProductCycle.Goal()
    goal.pickup_station_id = "pickup_a"
    goal.destination_station_id = "dispatch"
    return goal


def test_humble_api_signatures_are_used_by_the_contract():
    for name in ("succeed", "abort", "canceled"):
        inspect.signature(getattr(ServerGoalHandle, name)).bind(object())

    assert inspect.signature(Node.declare_parameter).parameters["descriptor"]
    assert inspect.signature(Node.get_parameter_or).parameters["alternative_value"]


@pytest.mark.parametrize("use_sim_time", [False, True])
def test_existing_clock_setting_survives_and_arrays_use_humble_types(use_sim_time):
    adapter = _construct_adapter(
        _valid_overrides(use_sim_time), predeclared_use_sim_time=use_sim_time)

    assert adapter.get_parameter("use_sim_time").value is use_sim_time
    assert adapter._mapping_ready
    assert adapter._product_by_station == {"pickup_a": "101", "pickup_b": "102"}
    assert adapter._autonomous_product_ids == {101, 102}
    assert adapter._dispatch_station_ids == {"dispatch"}
    assert adapter._descriptors["product_station_map"].type == \
        Parameter.Type.STRING_ARRAY.value
    assert adapter._descriptors["autonomous_product_ids"].type == \
        Parameter.Type.INTEGER_ARRAY.value
    assert adapter._descriptors["dispatch_station_ids"].type == \
        Parameter.Type.STRING_ARRAY.value


def test_allow_product_cycles_defaults_true():
    adapter = _construct_adapter(_valid_overrides(False), predeclared_use_sim_time=False)

    assert adapter._allow_product_cycles is True


def test_disabled_product_cycles_reject_before_bootstrap_or_joint_proof():
    adapter = _admissible_goal_adapter(allow_product_cycles=False)
    with patch.object(adapter, "_fresh_independent_empty_stow") as proof:
        assert adapter._goal_callback(_valid_cycle_goal()) == ADAPTER.GoalResponse.REJECT
    proof.assert_not_called()


def test_enabled_product_cycles_admit_valid_goal_after_existing_proof_gate():
    adapter = _admissible_goal_adapter(allow_product_cycles=True)

    assert adapter._goal_callback(_valid_cycle_goal()) == ADAPTER.GoalResponse.ACCEPT
    assert adapter._goal_reserved


def test_omitted_static_arrays_use_unavailable_mapping_path():
    adapter = ADAPTER.CycleSupervisor.__new__(ADAPTER.CycleSupervisor)
    adapter._parameters = {
        name: Parameter(name, value=None)
        for name in (
            "product_station_map",
            "autonomous_product_ids",
            "dispatch_station_ids",
        )
    }
    adapter._descriptors = {
        "product_station_map": ParameterDescriptor(
            type=Parameter.Type.STRING_ARRAY.value),
        "autonomous_product_ids": ParameterDescriptor(
            type=Parameter.Type.INTEGER_ARRAY.value),
        "dispatch_station_ids": ParameterDescriptor(
            type=Parameter.Type.STRING_ARRAY.value),
    }
    adapter._allow_undeclared_parameters = False
    adapter.get_parameter = Node.get_parameter.__get__(adapter)
    adapter.get_parameter_or = Node.get_parameter_or.__get__(adapter)

    with pytest.raises(ADAPTER.MappingError, match="product_station_map"):
        adapter._parse_mapping_parameters()

    with pytest.raises(ParameterUninitializedException):
        Node.get_parameter(adapter, "product_station_map")


def test_unavailable_mapping_log_is_formatted_before_error():
    adapter = _construct_adapter({
        "use_sim_time": Parameter("use_sim_time", value=False),
    })

    assert adapter.get_logger().errors == [
        "autonomous station mapping unavailable: "
        "product_station_map must be a non-empty string array"]


@pytest.mark.parametrize(
    ("parameter_name", "wrong_value"),
    [
        ("product_station_map", [101]),
        ("autonomous_product_ids", ["101"]),
        ("dispatch_station_ids", [101]),
    ],
)
def test_wrong_array_override_is_rejected_by_humble_descriptor(
        parameter_name, wrong_value):
    overrides = _valid_overrides(False)
    overrides[parameter_name] = Parameter(parameter_name, value=wrong_value)

    with pytest.raises(InvalidParameterTypeException):
        _construct_adapter(overrides)


@pytest.mark.parametrize(
    ("mapping", "product_ids", "dispatch"),
    [
        (["pickup_a"], [101], ["dispatch"]),
        (["pickup_a=103"], [103], ["dispatch"]),
        (["pickup_a=101"], [101], [""]),
    ],
)
def test_malformed_or_disabled_mapping_is_rejected(
        mapping, product_ids, dispatch):
    adapter = ADAPTER.CycleSupervisor.__new__(ADAPTER.CycleSupervisor)
    values = {
        "product_station_map": mapping,
        "autonomous_product_ids": product_ids,
        "dispatch_station_ids": dispatch,
    }
    adapter.get_parameter_or = lambda name: Parameter(name, value=values[name])

    with pytest.raises(ADAPTER.MappingError):
        adapter._parse_mapping_parameters()


def test_cancel_during_goal_reservation_is_accepted_before_execute_callback():
    adapter = ADAPTER.CycleSupervisor.__new__(ADAPTER.CycleSupervisor)
    adapter._lock = threading.RLock()
    adapter._active_goal = None
    adapter._goal_reserved = True
    adapter._cancel_requested = threading.Event()

    response = adapter._cancel_callback(object())

    assert response == ADAPTER.CancelResponse.ACCEPT
    assert adapter._cancel_requested.is_set()


class _GoalHandle:
    def __init__(self):
        self.request = SimpleNamespace(
            pickup_station_id="pickup_a", destination_station_id="dispatch")
        self.is_active = True
        self.feedback = []
        self.terminal_calls = []

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.terminal_calls.append("succeed")

    def abort(self):
        self.terminal_calls.append("abort")

    def canceled(self):
        self.terminal_calls.append("canceled")


class _Child:
    pid = 4242

    def __init__(self, return_code):
        self.return_code = return_code

    def poll(self):
        return self.return_code

    def wait(self):
        return self.return_code


def _execution_adapter(return_code):
    adapter = ADAPTER.CycleSupervisor.__new__(ADAPTER.CycleSupervisor)
    adapter._lock = threading.RLock()
    now = ADAPTER.time.monotonic()
    adapter._request_bootstrap_proof = lambda: None
    adapter._bootstrap_detached = True
    adapter._bootstrap_proof_at = now
    adapter._joint_states = JointState(
        name=list(ADAPTER.STOW), position=list(ADAPTER.STOW.values()))
    adapter._joint_states_at = now
    adapter._mapping_ready = True
    adapter._product_by_station = {"pickup_a": "101"}
    adapter._dispatch_station_ids = {"dispatch"}
    adapter._active_goal = None
    adapter._goal_reserved = True
    adapter._cancel_requested = threading.Event()
    adapter._child = None
    adapter._child_attached = False
    adapter._child_product_id = ""
    adapter._child_state = ADAPTER.ManipulatorStatus.STARTING
    adapter._child_valid = False
    adapter._child_consistent = False
    adapter._child_terminal_empty_proof = False
    adapter._child_detail = ""
    adapter._child_boot_id = 0
    adapter._child_sequence = 0
    adapter._child_status_at = 0.0
    adapter._child_status_authority_open = False
    adapter._state = ADAPTER.ManipulatorStatus.STOWED_EMPTY
    adapter._base_motion_allowed = True
    adapter._product_attached = False
    adapter._product_id = ""
    adapter._detail = ""
    adapter._fault_latched = False
    adapter.get_logger = lambda: _Logger()
    child = _Child(return_code)
    adapter._start_child = lambda product_id, pickup_station: child
    adapter._wait_fresh_independent_empty_stow = lambda: True
    adapter._child_safe_empty_stow = lambda: True
    return adapter


def _status_adapter():
    adapter = ADAPTER.CycleSupervisor.__new__(ADAPTER.CycleSupervisor)
    adapter._lock = threading.RLock()
    adapter._request_bootstrap_proof = lambda: None
    adapter._active_goal = None
    adapter._goal_reserved = False
    adapter._child = None
    adapter._child_attached = False
    adapter._child_product_id = ""
    adapter._child_state = ADAPTER.ManipulatorStatus.STARTING
    adapter._child_valid = False
    adapter._child_consistent = False
    adapter._child_terminal_empty_proof = False
    adapter._child_detail = ""
    adapter._child_boot_id = 0
    adapter._child_sequence = 0
    adapter._child_status_at = 0.0
    adapter._child_status_authority_open = False
    adapter._state = ADAPTER.ManipulatorStatus.STOWED_EMPTY
    adapter._base_motion_allowed = True
    adapter._product_attached = False
    adapter._product_id = ""
    adapter._detail = "idle"
    adapter._fault_latched = False
    adapter._bootstrap_detached = True
    adapter._bootstrap_proof_at = 10.0
    adapter._joint_states = JointState(
        name=list(ADAPTER.STOW), position=list(ADAPTER.STOW.values()))
    adapter._joint_states_at = 10.0
    adapter._boot_id = 1
    adapter._sequence = 0
    adapter._status_pub = _Publisher()
    adapter.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    return adapter


def _child_status(*, state, valid=True, base_motion_allowed=True,
                  product_attached=False, product_id="", source_boot_id=7,
                  sequence=1, detail="child"):
    message = ADAPTER.ManipulatorStatus()
    message.source_boot_id = source_boot_id
    message.sequence = sequence
    message.valid = valid
    message.state = state
    message.base_motion_allowed = base_motion_allowed
    message.product_attached = product_attached
    message.product_id = product_id
    message.detail = detail
    return message


@pytest.mark.parametrize(
    ("names", "positions", "expected"),
    [
        (list(ADAPTER.STOW) + ["extra"], list(ADAPTER.STOW.values()), False),
        (list(ADAPTER.STOW) + ["arm_joint_1"],
         list(ADAPTER.STOW.values()) + [0.0], False),
        (list(ADAPTER.STOW)[:-1], list(ADAPTER.STOW.values())[:-1], False),
        (list(ADAPTER.STOW), [0.0, -1.5708, math.nan, 0.0, 0.0, 0.0], False),
        (list(ADAPTER.STOW) + ["extra"],
         list(ADAPTER.STOW.values()) + [0.4], True),
    ],
)
def test_joint_proof_rejects_malformed_evidence_and_accepts_extra_unique(
        names, positions, expected):
    adapter = _status_adapter()
    adapter._joint_states = JointState(name=names, position=positions)

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        assert adapter._joint_stowed_locked() is expected


def test_idle_permission_expires_and_refreshes_with_independent_proof():
    adapter = _status_adapter()

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        adapter._publish_status()
    assert adapter._status_pub.messages[-1].base_motion_allowed

    expired_at = 10.0 + ADAPTER.BOOTSTRAP_PROOF_MAX_AGE_S + 0.01
    with patch.object(ADAPTER.time, "monotonic", return_value=expired_at):
        adapter._publish_status()
    assert not adapter._status_pub.messages[-1].base_motion_allowed

    adapter._bootstrap_proof_at = 12.0
    adapter._joint_states_at = 12.0
    with patch.object(ADAPTER.time, "monotonic", return_value=12.0):
        adapter._publish_status()
    assert adapter._status_pub.messages[-1].base_motion_allowed


@pytest.mark.parametrize(
    ("attribute", "value"),
    [
        ("_goal_reserved", True),
        ("_child", object()),
        ("_state", ADAPTER.ManipulatorStatus.MOVING),
        ("_product_attached", True),
        ("_child_attached", True),
    ],
)
def test_idle_permission_requires_unreserved_empty_unattached_state(
        attribute, value):
    adapter = _status_adapter()
    setattr(adapter, attribute, value)

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        adapter._publish_status()

    assert not adapter._status_pub.messages[-1].base_motion_allowed


def test_fault_latch_always_publishes_fault_and_retains_attachment():
    adapter = _status_adapter()
    adapter._fault_latched = True
    adapter._state = ADAPTER.ManipulatorStatus.FAULT
    adapter._base_motion_allowed = False
    adapter._product_attached = True
    adapter._product_id = "101"
    adapter._detail = "retained product"
    adapter._internal_status_callback(_child_status(
        state=ADAPTER.ManipulatorStatus.STOWED_EMPTY,
        base_motion_allowed=True))

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        adapter._publish_status()
    message = adapter._status_pub.messages[-1]
    assert message.state == ADAPTER.ManipulatorStatus.FAULT
    assert not message.valid
    assert not message.base_motion_allowed
    assert message.product_attached
    assert message.product_id == "101"


def test_unowned_child_status_cannot_grant_idle_motion():
    adapter = _status_adapter()
    adapter._bootstrap_detached = False
    adapter._base_motion_allowed = False
    adapter._internal_status_callback(_child_status(
        state=ADAPTER.ManipulatorStatus.STOWED_EMPTY,
        base_motion_allowed=True))

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        adapter._publish_status()
    assert not adapter._status_pub.messages[-1].base_motion_allowed


def test_active_owned_fresh_loaded_child_status_remains_positive_control():
    adapter = _status_adapter()
    adapter._active_goal = object()
    adapter._child_status_authority_open = True
    adapter._product_id = "101"
    adapter._state = ADAPTER.ManipulatorStatus.STARTING
    adapter._base_motion_allowed = False
    adapter._internal_status_callback(_child_status(
        state=ADAPTER.ManipulatorStatus.STOWED_LOADED,
        product_attached=True, product_id="101", base_motion_allowed=True))

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        adapter._publish_status()
    message = adapter._status_pub.messages[-1]
    assert message.state == ADAPTER.ManipulatorStatus.STOWED_LOADED
    assert message.valid
    assert message.base_motion_allowed
    assert message.product_attached


def test_terminal_empty_proof_survives_wrapper_teardown_but_later_status_clears_it():
    adapter = _status_adapter()
    adapter._active_goal = object()
    adapter._child_status_authority_open = True
    adapter._product_id = "101"

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        adapter._internal_status_callback(_child_status(
            state=ADAPTER.ManipulatorStatus.STOWED_EMPTY,
            base_motion_allowed=True, product_attached=False,
            product_id="", sequence=1))
        assert adapter._child_terminal_empty_proof
        assert adapter._child_safe_empty_stow()

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0 +
                     ADAPTER.INTERNAL_STATUS_MAX_AGE_S + 0.01):
        assert adapter._child_safe_empty_stow()
        adapter._internal_status_callback(_child_status(
            state=ADAPTER.ManipulatorStatus.FAULT,
            valid=False, base_motion_allowed=False,
            product_attached=False, product_id="", sequence=2))
        assert not adapter._child_terminal_empty_proof
        assert not adapter._child_safe_empty_stow()


def test_closed_terminal_authority_denies_motion_after_idle_proof_expires():
    adapter = _status_adapter()
    adapter._active_goal = object()
    adapter._child_status_authority_open = False
    adapter._state = ADAPTER.ManipulatorStatus.STOWED_LOADED
    adapter._base_motion_allowed = True
    adapter._product_attached = True
    adapter._product_id = "101"
    adapter._child_attached = True
    adapter._child_state = ADAPTER.ManipulatorStatus.STOWED_LOADED
    adapter._child_valid = True
    adapter._child_consistent = True
    adapter._child_status_at = 10.0
    adapter._bootstrap_detached = False
    adapter._bootstrap_proof_at = 0.0
    adapter._joint_states_at = 0.0

    with patch.object(ADAPTER.time, "monotonic", return_value=10.0):
        adapter._publish_status()

    assert not adapter._status_pub.messages[-1].base_motion_allowed


def test_late_child_status_cannot_reacquire_authority_after_success():
    adapter = _execution_adapter(0)
    goal_handle = _GoalHandle()
    result = adapter._execute_callback(goal_handle)
    assert result.outcome == ExecuteProductCycle.Result.SUCCESS

    adapter._internal_status_callback(_child_status(
        state=ADAPTER.ManipulatorStatus.STOWED_LOADED,
        product_attached=True, product_id="101", base_motion_allowed=True,
        sequence=2))
    assert adapter._state == ADAPTER.ManipulatorStatus.STOWED_EMPTY
    assert not adapter._product_attached
    assert adapter._product_id == ""


def test_latched_idle_helper_refuses_and_retains_fault_evidence():
    adapter = _status_adapter()
    adapter._fault_latched = True
    adapter._state = ADAPTER.ManipulatorStatus.FAULT
    adapter._base_motion_allowed = False
    adapter._product_attached = True
    adapter._product_id = "101"
    adapter._child_attached = True

    assert adapter._set_idle_from_proof("must remain faulted") is False
    assert adapter._fault_latched
    assert adapter._state == ADAPTER.ManipulatorStatus.FAULT
    assert not adapter._base_motion_allowed
    assert adapter._product_attached
    assert adapter._product_id == "101"


@pytest.mark.parametrize(
    ("return_code", "terminal"),
    [(0, "abort"), (1, "abort")],
)
def test_terminal_idle_refusal_returns_retained_fault(
        return_code, terminal):
    adapter = _execution_adapter(return_code)
    adapter._set_idle_from_proof = lambda detail: False
    goal_handle = _GoalHandle()

    result = adapter._execute_callback(goal_handle)

    assert result.outcome == ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT
    assert not result.delivered
    assert goal_handle.terminal_calls == [terminal]


class _CancelChild:
    pid = 5252

    def __init__(self):
        self.poll_count = 0

    def poll(self):
        self.poll_count += 1
        return None if self.poll_count == 1 else 130

    def wait(self):
        return 130


def test_canceled_terminal_idle_refusal_returns_retained_fault():
    adapter = _execution_adapter(130)
    child = _CancelChild()

    def start_child(product_id, pickup_station):
        adapter._cancel_requested.set()
        return child

    adapter._start_child = start_child
    adapter._request_child_cancel = lambda: True
    adapter._set_idle_from_proof = lambda detail: False
    goal_handle = _GoalHandle()

    with patch.object(ADAPTER.rclpy, "ok", return_value=True), \
            patch.object(ADAPTER.time, "sleep"):
        result = adapter._execute_callback(goal_handle)

    assert result.outcome == ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT
    assert not result.delivered
    assert goal_handle.terminal_calls == ["canceled"]


@pytest.mark.parametrize(
    ("return_code", "terminal", "outcome"),
    [
        (0, "succeed", ExecuteProductCycle.Result.SUCCESS),
        (1, "abort", ExecuteProductCycle.Result.EXECUTION_FAILED),
        (130, "canceled", ExecuteProductCycle.Result.RETAINED_PRODUCT_FAULT),
    ],
)
def test_terminal_calls_use_humble_no_argument_api_and_return_typed_result(
        return_code, terminal, outcome):
    adapter = _execution_adapter(return_code)
    goal_handle = _GoalHandle()

    result = adapter._execute_callback(goal_handle)

    assert isinstance(result, ExecuteProductCycle.Result)
    assert result.outcome == outcome
    assert goal_handle.terminal_calls == [terminal]


def test_cancel_arriving_before_execute_callback_is_preserved():
    adapter = _execution_adapter(0)
    adapter._cancel_requested.set()
    goal_handle = _GoalHandle()

    result = adapter._execute_callback(goal_handle)

    assert result.outcome == ExecuteProductCycle.Result.CANCELED
    assert not result.delivered
    assert goal_handle.terminal_calls == ["canceled"]


class _FailingFuture:
    def result(self):
        raise RuntimeError("bootstrap unavailable")


def test_three_failure_logs_pass_one_formatted_message_argument():
    adapter = ADAPTER.CycleSupervisor.__new__(ADAPTER.CycleSupervisor)
    adapter._lock = threading.RLock()
    adapter._bootstrap_future = future = _FailingFuture()
    bootstrap_logger = _Logger()
    adapter.get_logger = lambda: bootstrap_logger
    adapter._bootstrap_done(future)
    assert bootstrap_logger.errors == [
        "attachment bootstrap proof failed: bootstrap unavailable"]

    cancel_logger = _Logger()
    adapter._cancel_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: False)
    adapter.get_logger = lambda: cancel_logger
    assert adapter._request_child_cancel() is False
    assert cancel_logger.errors == ["child cancellation service unavailable"]

    cleanup_logger = _Logger()
    adapter.get_logger = lambda: cleanup_logger
    child = SimpleNamespace(
        pid=123,
        poll=lambda: None,
        wait=lambda timeout: (_ for _ in ()).throw(
            subprocess.TimeoutExpired("child", timeout)),
    )
    with patch.object(ADAPTER.os, "killpg"):
        assert adapter._terminate_child_fallback(child) is False
    assert cleanup_logger.errors == [
        "child did not stop after SIGINT cleanup fallback"]


def test_general_cycle_failure_log_is_formatted_before_error():
    adapter = _execution_adapter(0)
    adapter._start_child = lambda product_id, pickup_station: (_ for _ in ()).throw(
        RuntimeError("runner failed"))
    logger = _Logger()
    adapter.get_logger = lambda: logger
    result = adapter._execute_callback(_GoalHandle())

    assert isinstance(result, ExecuteProductCycle.Result)
    assert result.outcome == ExecuteProductCycle.Result.EXECUTION_FAILED
    assert logger.errors == ["cycle adapter failed: runner failed"]


class _MainNode:
    def __init__(self):
        self.destroy_calls = 0

    def destroy_node(self):
        self.destroy_calls += 1


class _MainContext:
    def __init__(self):
        self.valid = True
        self.try_shutdown_calls = 0


class _MainExecutor:
    def __init__(self, spin_exception=None, context=None):
        self.spin_exception = spin_exception
        self.context = context
        self.added_nodes = []
        self.shutdown_calls = 0

    def add_node(self, node):
        self.added_nodes.append(node)

    def spin(self):
        if self.context is not None:
            self.context.valid = False
        if self.spin_exception is not None:
            raise self.spin_exception

    def shutdown(self):
        self.shutdown_calls += 1


@contextmanager
def _main_seams(spin_exception=None, context_already_invalid=False):
    node = _MainNode()
    context = _MainContext()
    executor = _MainExecutor(
        spin_exception=spin_exception,
        context=context if context_already_invalid else None)

    def try_shutdown():
        context.try_shutdown_calls += 1
        if context.valid:
            context.valid = False

    with patch.object(ADAPTER.rclpy, "init"), \
            patch.object(ADAPTER.rclpy, "try_shutdown", side_effect=try_shutdown), \
            patch.object(ADAPTER, "CycleSupervisor", return_value=node), \
            patch.object(ADAPTER, "MultiThreadedExecutor", return_value=executor):
        yield node, executor, context


def test_main_normal_spin_returns_and_cleans_up_once():
    with _main_seams() as (node, executor, context):
        ADAPTER.main()

    assert executor.shutdown_calls == 1
    assert node.destroy_calls == 1
    assert context.try_shutdown_calls == 1


@pytest.mark.parametrize(
    "spin_exception",
    [KeyboardInterrupt(), ADAPTER.ExternalShutdownException()],
)
def test_main_signal_shutdown_exceptions_are_orderly(spin_exception):
    with _main_seams(spin_exception) as (node, executor, context):
        ADAPTER.main()

    assert executor.shutdown_calls == 1
    assert node.destroy_calls == 1
    assert context.try_shutdown_calls == 1


def test_main_cleanup_is_idempotent_when_context_is_already_invalid():
    with _main_seams(KeyboardInterrupt(), context_already_invalid=True) as (
            node, executor, context):
        ADAPTER.main()

    assert not context.valid
    assert executor.shutdown_calls == 1
    assert node.destroy_calls == 1
    assert context.try_shutdown_calls == 1


def test_main_unexpected_spin_exception_remains_visible():
    with _main_seams(RuntimeError("spin failed"), context_already_invalid=True) as (
            node, executor, context):
        with pytest.raises(RuntimeError, match="spin failed"):
            ADAPTER.main()

    assert executor.shutdown_calls == 1
    assert node.destroy_calls == 1
    assert context.try_shutdown_calls == 1
