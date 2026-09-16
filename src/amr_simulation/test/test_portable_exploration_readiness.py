"""Contract tests for staged, local portable readiness."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from amr_interfaces.msg import BaseStatus, ManipulatorStatus
from builtin_interfaces.msg import Time
from std_msgs.msg import Header


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "portable_exploration_readiness.py"


def _load():
    spec = importlib.util.spec_from_file_location("portable_readiness", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_readiness_stages_and_no_terminal_timeout_are_explicit():
    module = _load()
    assert module.STAGES == (
        "adapters_authority", "slam_map", "planner_smoother",
        "controller", "mission_final",
    )
    assert module.MAX_UNMET_LOG_INTERVAL_S <= 60.0
    assert module.TERMINAL_TIMEOUT_S is None
    assert module.fresh_receipt(9.9, now=10.0, max_age=0.2)
    assert not module.fresh_receipt(9.7, now=10.0, max_age=0.2)
    assert not module.fresh_receipt(10.1, now=10.0, max_age=0.2)


def test_stage_prerequisites_are_current_and_prior_only():
    module = _load()
    assert module.stage_prerequisites("adapters_authority") == ("adapters_authority",)
    assert module.stage_prerequisites("slam_map") == ("adapters_authority", "slam_map")
    assert module.stage_prerequisites("planner_smoother") == (
        "adapters_authority", "slam_map", "planner_smoother",
    )
    assert module.stage_prerequisites("controller") == (
        "adapters_authority", "slam_map", "planner_smoother", "controller",
    )
    assert module.stage_prerequisites("mission_final") == module.STAGES


def test_readiness_has_structured_ready_unmet_fault_and_explicit_process_gates():
    text = SCRIPT.read_text()
    for marker in ("ready", "unmet", "fault", "process", "shutdown"):
        assert marker in text.lower()
    for name in (
        "base_adapter_node", "front_lidar_adapter_node", "rear_lidar_adapter_node",
        "imu_adapter_node", "product_camera_adapter_node", "wheel_odometry_node",
        "front_lidar_perception_node", "rear_lidar_perception_node",
        "command_arbitration_node", "slam_toolbox", "planner_server",
        "smoother_server", "controller_server", "mission_supervisor_node",
    ):
        assert name in text
    for topic in (
        "/amr/sensors/front_lidar/scan", "/map",
        "/amr/global_costmap/costmap", "/amr/local_costmap/costmap",
        "/amr/base/status", "/amr/manipulation/status",
    ):
        assert topic in text
    assert "NavigateToPose" in text
    assert "map->odom" in text or "lookup_transform" in text
    assert "all_unknown" in text or "unknown" in text.lower()
    assert 'declare_parameter("stage"' in text


def test_readiness_fails_closed_on_faults_and_final_rechecks():
    module = _load()
    assert module.map_is_valid([0, -1, 100, 50])
    assert not module.map_is_valid([-1, -1, -1])
    assert not module.map_is_valid([])
    text = SCRIPT.read_text()
    assert "STOWED_EMPTY" in text
    assert "final" in text.lower()
    assert "global_costmap" in text
    assert "local_costmap" in text


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _StrictLogger:
    def __init__(self):
        self.warning_messages = []

    def warning(self, message, **kwargs):
        self.warning_messages.append((message, kwargs))


def _ros_clock(stamp):
    return SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: stamp),
    )


def _tf_message(parent, child, stamp, *, finite=True):
    value = 0.0 if finite else float("nan")
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=parent, stamp=stamp),
        child_frame_id=child,
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=value, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _tf_readiness(module, now_stamp):
    readiness = module.PortableExplorationReadiness.__new__(
        module.PortableExplorationReadiness)
    readiness._tf_edges = {}
    readiness.get_clock = lambda: _ros_clock(now_stamp)
    return readiness


def _composed_readiness(module, now_stamp, *, output, map_odom=None,
                        odom_base=None, map_odom_receipt=100.0,
                        odom_base_receipt=100.0, lookup_error=None):
    readiness = _tf_readiness(module, now_stamp)
    readiness._tf_edges = {}
    if map_odom is not None:
        readiness._tf_edges["map", "odom"] = (map_odom, map_odom_receipt)
    if odom_base is not None:
        readiness._tf_edges["odom", "base_footprint"] = (
            odom_base, odom_base_receipt)
    if lookup_error is not None:
        def lookup_transform(*_args):
            raise lookup_error
    else:
        lookup_transform = lambda *_args: output
    readiness._tf_buffer = SimpleNamespace(lookup_transform=lookup_transform)
    return readiness


def test_slam_map_odom_future_tolerance_is_exactly_one_second():
    module = _load()
    assert module.SLAM_TF_FUTURE_TOLERANCE_S == 1.0
    now_stamp = Time(sec=10, nanosec=0)

    accepted = _tf_readiness(module, now_stamp)
    with patch.object(module.time, "monotonic", return_value=100.0):
        accepted._tf_callback(SimpleNamespace(transforms=[
            _tf_message("map", "odom", Time(sec=11, nanosec=0)),
        ]))
    assert ("map", "odom") in accepted._tf_edges
    assert accepted._fresh_tf_edge("map", "odom", now=100.5)

    rejected = _tf_readiness(module, now_stamp)
    with patch.object(module.time, "monotonic", return_value=100.0):
        rejected._tf_callback(SimpleNamespace(transforms=[
            _tf_message("map", "odom", Time(sec=11, nanosec=1)),
        ]))
    assert ("map", "odom") not in rejected._tf_edges


def test_odom_base_positive_future_stamp_remains_rejected():
    module = _load()
    readiness = _tf_readiness(module, Time(sec=10, nanosec=0))
    with patch.object(module.time, "monotonic", return_value=100.0):
        readiness._tf_callback(SimpleNamespace(transforms=[
            _tf_message("odom", "base_footprint", Time(sec=10, nanosec=1)),
        ]))
    assert ("odom", "base_footprint") not in readiness._tf_edges


def test_composed_map_base_allows_one_second_buffer_future_stamp_only_with_valid_sources():
    module = _load()
    now_stamp = Time(sec=10, nanosec=0)
    map_odom = _tf_message("map", "odom", now_stamp)
    odom_base = _tf_message("odom", "base_footprint", now_stamp)
    output = _tf_message("map", "base_footprint", Time(sec=11, nanosec=0))

    readiness = _composed_readiness(
        module, now_stamp, output=output, map_odom=map_odom,
        odom_base=odom_base)
    with patch.object(module.time, "monotonic", return_value=100.5):
        assert readiness._fresh_composed_map_base(now=100.5)

    too_future = _composed_readiness(
        module, now_stamp,
        output=_tf_message("map", "base_footprint", Time(sec=11, nanosec=1)),
        map_odom=map_odom, odom_base=odom_base)
    assert not too_future._fresh_composed_map_base(now=100.5)

    missing_source = _composed_readiness(
        module, now_stamp, output=output, map_odom=map_odom)
    assert not missing_source._fresh_composed_map_base(now=100.5)

    stale_source = _composed_readiness(
        module, now_stamp, output=output, map_odom=map_odom,
        odom_base=odom_base, odom_base_receipt=98.9)
    assert not stale_source._fresh_composed_map_base(now=100.5)

    invalid_source = _composed_readiness(
        module, now_stamp, output=output, map_odom=map_odom,
        odom_base=_tf_message("odom", "base_footprint", now_stamp, finite=False))
    assert not invalid_source._fresh_composed_map_base(now=100.5)

    invalid_output = _composed_readiness(
        module, now_stamp,
        output=_tf_message("map", "base_footprint", Time(sec=11), finite=False),
        map_odom=map_odom, odom_base=odom_base)
    assert not invalid_output._fresh_composed_map_base(now=100.5)

    lookup_failed = _composed_readiness(
        module, now_stamp, output=output, map_odom=map_odom,
        odom_base=odom_base, lookup_error=RuntimeError("lookup failed"))
    assert not lookup_failed._fresh_composed_map_base(now=100.5)


def test_unmet_tick_uses_one_preformatted_warning_message():
    module = _load()
    readiness = module.PortableExplorationReadiness.__new__(module.PortableExplorationReadiness)
    logger = _StrictLogger()
    readiness._fault_reason = ""
    readiness._graph_nodes = lambda: set()
    readiness._refresh_lifecycle_states = lambda _graph: None
    readiness._evaluate = lambda _graph, _now: (
        "adapters_authority", ("fresh front scan",))
    readiness._last_unmet = ()
    readiness._last_unmet_log_at = 0.0
    readiness._publish_report = lambda: None
    readiness.get_logger = lambda: logger

    readiness._tick()

    assert logger.warning_messages == [(
        "portable readiness unmet stage=adapters_authority conditions=fresh front scan",
        {},
    )]


def _bare_readiness(active):
    module = _load()
    readiness = module.PortableExplorationReadiness.__new__(module.PortableExplorationReadiness)
    readiness._lifecycle_state = {name: 3 for name in active}
    readiness._active = lambda name, _graph: name in active
    readiness._process_present = lambda name, graph: name in graph
    readiness._fresh_tf_edge = lambda _parent, _child, _now: True
    readiness._fresh_composed_map_base = lambda _now: True
    readiness._mission_client = SimpleNamespace(server_is_ready=lambda: True)
    readiness._map = SimpleNamespace(
        data=[0, -1],
        info=SimpleNamespace(width=1, height=2, resolution=1.0, origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    readiness._map_at = 10.0
    readiness._global_costmap = SimpleNamespace(
        data=[0],
        info=SimpleNamespace(width=1, height=1, resolution=1.0, origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    readiness._global_costmap_at = 10.0
    readiness._local_costmap = SimpleNamespace(
        data=[0],
        info=SimpleNamespace(width=1, height=1, resolution=1.0, origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    readiness._local_costmap_at = 10.0
    readiness._front_scan_at = 10.0
    readiness._base_at = 10.0
    readiness._base_valid = True
    readiness._base_status = SimpleNamespace()
    readiness._manipulator_at = 10.0
    readiness._manipulator_valid = True
    readiness._manipulator_status = SimpleNamespace()
    return readiness, module


def test_readiness_releases_only_after_causal_stage_progression_and_final_recheck():
    module = _load()
    now = 10.0
    readiness, _ = _bare_readiness(set())
    stage, unmet = readiness._evaluate(set(), now)
    assert stage == module.STAGES[0]
    assert unmet

    active = set(module.ADAPTER_NODES)
    readiness, _ = _bare_readiness(active)
    graph = set(active)
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == module.STAGES[1]
    assert "active slam_toolbox" in unmet

    graph.add(module.SLAM_NODE)
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == module.STAGES[2]
    assert unmet

    active.update(module.PLANNER_NODES)
    graph.update(module.PLANNER_NODES)
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == module.STAGES[3]
    assert "controller_server" in " ".join(unmet)

    active.update(module.CONTROLLER_NODES)
    readiness._local_costmap_at = 0.0
    stage, unmet = readiness._evaluate(graph | set(module.CONTROLLER_NODES), now)
    assert stage == module.STAGES[3]
    assert "fresh local costmap" in unmet
    readiness._local_costmap_at = now
    active.update(module.ALL_LIFECYCLE_NODES)
    graph.update(module.CONTROLLER_NODES)
    graph.add(module.MISSION_NODE)
    graph.add("health_supervisor_node")
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == module.STAGES[4]
    assert unmet == ()


def test_stage_target_does_not_require_future_evidence_and_rechecks_prior_stages():
    module = _load()
    now = 10.0
    readiness, _ = _bare_readiness(set(module.ADAPTER_NODES))
    graph = set(module.ADAPTER_NODES)

    readiness._target_stage = "adapters_authority"
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == "adapters_authority"
    assert unmet == ()

    readiness._target_stage = "slam_map"
    graph.add(module.SLAM_NODE)
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == "slam_map"
    assert unmet == ()

    readiness._target_stage = "planner_smoother"
    readiness._active = lambda name, _graph: name in {
        *module.ADAPTER_NODES, *module.PLANNER_NODES,
    }
    graph.update(module.PLANNER_NODES)
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == "planner_smoother"
    assert unmet == ()

    readiness._front_scan_at = 0.0
    stage, unmet = readiness._evaluate(graph, now)
    assert stage == "adapters_authority"
    assert "fresh front scan" in unmet


def test_readiness_status_is_structured_and_explicit_fault_stops_release():
    module = _load()
    readiness, _ = _bare_readiness(set())
    readiness._stage = module.STAGES[0]
    readiness._unmet = ("fresh front scan",)
    readiness._fault_reason = ""
    readiness._readiness_pub = _Publisher()
    readiness._publish_report()
    assert '"state": "UNMET"' in readiness._readiness_pub.messages[-1].data
    readiness._fault_reason = "explicit BaseStatus FAULT"
    readiness._publish_report()
    assert '"state": "FAULT"' in readiness._readiness_pub.messages[-1].data

    readiness, module = _bare_readiness(set())
    readiness._base_boot = 0
    readiness._base_sequence = 0
    readiness._clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    readiness.get_clock = lambda: readiness._clock
    readiness._base_callback(BaseStatus(
        header=Header(stamp=Time()),
        source_boot_id=1,
        sequence=1,
        valid=False,
        state=BaseStatus.FAULT,
        reason=BaseStatus.REASON_FAULT,
    ))
    assert readiness._fault_reason == "explicit BaseStatus FAULT"


def test_required_process_disappearance_is_an_explicit_fault():
    module = _load()
    readiness = module.PortableExplorationReadiness.__new__(module.PortableExplorationReadiness)
    readiness._seen_processes = {module.SLAM_NODE}
    readiness._fault_reason = ""
    readiness._lifecycle_clients = {}
    readiness._refresh_lifecycle_states(set())
    assert readiness._fault_reason == "required portable process exited: slam_toolbox"
