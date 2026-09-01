import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "factory_runtime_preflight.py"
SPEC = importlib.util.spec_from_file_location("factory_runtime_preflight", SCRIPT)
PREFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREFLIGHT)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class FakeGraphNode:
    def __init__(self, snapshots):
        self.snapshots = snapshots
        self.index = -1

    def advance(self):
        if self.index + 1 < len(self.snapshots):
            self.index += 1

    def get_node_names_and_namespaces(self):
        snapshot = self.snapshots[max(self.index, 0)]
        return [
            (full_name.rsplit("/", 1)[1], full_name.rsplit("/", 1)[0] or "/")
            for full_name in snapshot
        ]


def observe_graph(snapshots, timeout=4.0, stability=1.0, poll=0.5):
    clock = FakeClock()
    node = FakeGraphNode(snapshots)

    def spin_once(spun_node, timeout_sec):
        assert spun_node is node
        clock.now += timeout_sec
        node.advance()

    return PREFLIGHT.observe_required_graph(
        node,
        spin_once,
        monotonic=clock.monotonic,
        discovery_timeout=timeout,
        stability_seconds=stability,
        poll_seconds=poll,
    )


class FakeFuture:
    def __init__(self, clock, response, ready_after):
        self.clock = clock
        self.response = response
        self.ready_at = (
            None if ready_after is None else clock.monotonic() + ready_after
        )

    def done(self):
        return self.ready_at is not None and self.clock.monotonic() >= self.ready_at

    def result(self):
        return self.response


class FakeLifecycleClient:
    def __init__(self, clock, outcomes, service_ready=True):
        self.clock = clock
        self.outcomes = list(outcomes)
        self.service_ready = service_ready
        self.calls = 0
        self.removed = []

    def service_is_ready(self):
        return self.service_ready

    def call_async(self, _request):
        outcome = self.outcomes[min(self.calls, len(self.outcomes) - 1)]
        self.calls += 1
        if outcome == "timeout":
            return FakeFuture(self.clock, None, None)
        state_id, state_label, ready_after = outcome
        response = SimpleNamespace(
            current_state=SimpleNamespace(id=state_id, label=state_label)
        )
        return FakeFuture(self.clock, response, ready_after)

    def remove_pending_request(self, future):
        self.removed.append(future)


class FakeMoveItClient:
    def __init__(self, clock, response, ready_after=0.0, service_ready=True):
        self.clock = clock
        self.response = response
        self.ready_at = (
            None if ready_after is None else clock.monotonic() + ready_after
        )
        self.service_ready = service_ready
        self.calls = 0
        self.removed = []

    def service_is_ready(self):
        return self.service_ready

    def call_async(self, _request):
        self.calls += 1
        ready_after = (
            None if self.ready_at is None
            else self.ready_at - self.clock.monotonic()
        )
        return FakeFuture(self.clock, self.response, ready_after)

    def remove_pending_request(self, future):
        self.removed.append(future)


def observe_lifecycle(clients, timeout=2.0, stability=0.3,
                      response_timeout=0.2, poll=0.1):
    clock = next(iter(clients.values())).clock

    def spin_once(_node, timeout_sec):
        clock.now += timeout_sec

    return PREFLIGHT.observe_required_lifecycle(
        object(),
        clients,
        spin_once,
        request_factory=object,
        monotonic=clock.monotonic,
        discovery_timeout=timeout,
        stability_seconds=stability,
        response_timeout=response_timeout,
        poll_seconds=poll,
        required_nodes=tuple(clients),
    )


def observe_moveit(client, discovery_timeout=2.0, response_timeout=0.2,
                   poll=0.1):
    clock = client.clock

    def spin_once(_node, timeout_sec):
        clock.now += timeout_sec

    return PREFLIGHT.observe_moveit_planner_service(
        object(),
        client,
        spin_once,
        request_factory=object,
        monotonic=clock.monotonic,
        discovery_timeout=discovery_timeout,
        response_timeout=response_timeout,
        poll_seconds=poll,
    )


def stats_text(factors, real_step=1.0, sim_step=1.0):
    def stamp(seconds):
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        if nanoseconds == 1_000_000_000:
            whole_seconds += 1
            nanoseconds = 0
        return (
            f"  sec: {whole_seconds}\n"
            f"  nsec: {nanoseconds}\n"
        )

    blocks = []
    for index, factor in enumerate(factors):
        blocks.append(
            "sim_time {\n"
            f"{stamp(index * sim_step)}"
            "}\n"
            "real_time {\n"
            f"{stamp(index * real_step)}"
            "}\n"
            f"real_time_factor: {factor}\n"
        )
    return "\n".join(blocks)


def test_accessible_render_devices_only_accepts_render_nodes_with_access(tmp_path):
    (tmp_path / "card1").touch()
    (tmp_path / "renderD128").touch()
    assert PREFLIGHT.accessible_render_devices(str(tmp_path)) == [
        str(tmp_path / "renderD128")
    ]


def test_forced_software_environment_detects_both_supported_overrides():
    assert PREFLIGHT.forced_software_environment({
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "GALLIUM_DRIVER": "iris",
    }) == {"LIBGL_ALWAYS_SOFTWARE": "1"}
    assert PREFLIGHT.forced_software_environment({
        "LIBGL_ALWAYS_SOFTWARE": "0",
        "GALLIUM_DRIVER": "llvmpipe",
    }) == {"GALLIUM_DRIVER": "llvmpipe"}


def test_process_scan_excludes_only_the_preflight_parent_chain(tmp_path):
    def process(pid, parent_pid, command):
        directory = tmp_path / str(pid)
        directory.mkdir()
        (directory / "stat").write_text(
            f"{pid} (test) S {parent_pid} 0 0 0 0\n", encoding="utf-8")
        (directory / "cmdline").write_bytes(command.encode("utf-8") + b"\0")

    # The preflight's parent shell contains the planned launch command, but is
    # not a running simulator.  A separate Gazebo process must still be found.
    process(1, 0, "init")
    process(20, 1, "/bin/bash -c ros2 launch amr_factory factory")
    process(30, 20, "python3 factory_runtime_preflight.py host")
    process(40, 1, "gz sim -r factory.sdf")

    assert PREFLIGHT.calling_process_ancestry(tmp_path, start_pid=30) == {1, 20, 30}
    assert PREFLIGHT.matching_processes(
        tmp_path, ignored_pids=PREFLIGHT.calling_process_ancestry(tmp_path, 30)
    ) == [(40, "gz sim -r factory.sdf")]


def test_d205_like_stats_pass_both_performance_gates():
    summary = PREFLIGHT.summarize_stats(
        stats_text([1.0, 0.9, 1.1, 1.0, 1.0, 1.1, 0.9, 1.0, 1.0, 1.0],
                   real_step=1.0, sim_step=0.99)
    )
    assert summary["samples"] == 10
    assert summary["median"] >= 0.90
    assert summary["aggregate_sim_real"] >= 0.90


def test_degraded_stats_fail_the_aggregate_gate():
    summary = PREFLIGHT.summarize_stats(
        stats_text([0.2] * 10, real_step=1.0, sim_step=0.2)
    )
    assert summary["median"] < 0.90
    assert summary["aggregate_sim_real"] < 0.90


def test_stats_with_too_few_valid_samples_fail_closed():
    with pytest.raises(RuntimeError, match="valid /stats samples"):
        PREFLIGHT.summarize_stats(stats_text([1.0] * 9))


def test_stats_without_positive_time_span_fail_closed():
    with pytest.raises(RuntimeError, match="span is not positive"):
        PREFLIGHT.summarize_stats(stats_text([1.0] * 10, real_step=0.0, sim_step=0.0))


def test_persistent_graph_observer_waits_for_complete_stable_graph():
    required = list(PREFLIGHT.REQUIRED_GRAPH_NODES)
    result = observe_graph([
        required[:6],
        required[:12],
        required,
        required,
        required,
    ])

    assert result["passed"] is True
    assert result["missing"] == ()
    assert result["duplicates"] == ()
    assert result["stable_seconds"] >= 1.0
    assert len(result["transitions"]) == 3


def test_graph_observer_resets_stability_when_node_disappears():
    required = list(PREFLIGHT.REQUIRED_GRAPH_NODES)
    incomplete = required[:-1]
    result = observe_graph([
        required,
        incomplete,
        required,
        incomplete,
        required,
        incomplete,
    ], timeout=3.0)

    assert result["passed"] is False
    assert result["missing"] == ("/move_group",)
    assert result["stable_seconds"] == 0.0


def test_graph_observer_fails_when_required_node_never_appears():
    required = list(PREFLIGHT.REQUIRED_GRAPH_NODES)
    result = observe_graph([required[:-1]], timeout=2.0)

    assert result["passed"] is False
    assert result["missing"] == ("/move_group",)


def test_graph_observer_rejects_duplicate_required_node_names():
    required = list(PREFLIGHT.REQUIRED_GRAPH_NODES)
    result = observe_graph([required + ["/amr/amcl"]], timeout=2.0)

    assert result["passed"] is False
    assert result["missing"] == ()
    assert result["duplicates"] == ("/amr/amcl",)


def test_main_routes_graph_mode_to_graph_preflight(monkeypatch, tmp_path):
    called = []

    def graph_preflight(evidence_dir):
        called.append(evidence_dir)
        return 17

    monkeypatch.setattr(PREFLIGHT, "graph_preflight", graph_preflight)
    assert PREFLIGHT.main([
        "graph", "--evidence-dir", str(tmp_path)
    ]) == 17
    assert called == [tmp_path]


def test_lifecycle_observer_recovers_from_one_lost_response_and_stabilizes():
    clock = FakeClock()
    clients = {
        "/a": FakeLifecycleClient(
            clock, ["timeout", (3, "active", 0.0)]
        ),
        "/b": FakeLifecycleClient(clock, [(3, "active", 0.0)]),
    }

    result = observe_lifecycle(clients)

    assert result["passed"] is True
    assert result["stable_seconds"] >= 0.3
    assert result["final_states"]["/a"]["status"] == "ok"
    assert result["final_states"]["/b"]["state_label"] == "active"
    assert len(clients["/a"].removed) == 1
    assert any(
        transition["states"]["/a"]["status"] == "response_timeout"
        for transition in result["transitions"]
    )


def test_lifecycle_observer_requires_exact_active_id_and_label():
    clock = FakeClock()
    clients = {
        "/a": FakeLifecycleClient(clock, [(2, "inactive", 0.0)]),
    }

    result = observe_lifecycle(clients, timeout=0.5)

    assert result["passed"] is False
    assert result["nonactive"] == ("/a",)
    assert result["stable_seconds"] == 0.0


def test_lifecycle_observer_fails_closed_when_service_is_unavailable():
    clock = FakeClock()
    clients = {
        "/a": FakeLifecycleClient(
            clock, [(3, "active", 0.0)], service_ready=False
        ),
    }

    result = observe_lifecycle(clients, timeout=0.5)

    assert result["passed"] is False
    assert result["unavailable_services"] == ("/a",)


def test_main_routes_lifecycle_mode_to_lifecycle_preflight(monkeypatch, tmp_path):
    called = []

    def lifecycle_preflight(evidence_dir):
        called.append(evidence_dir)
        return 23

    monkeypatch.setattr(PREFLIGHT, "lifecycle_preflight", lifecycle_preflight)
    assert PREFLIGHT.main([
        "lifecycle", "--evidence-dir", str(tmp_path)
    ]) == 23
    assert called == [tmp_path]


def test_persistent_moveit_service_observer_accepts_ompl_response():
    clock = FakeClock()
    client = FakeMoveItClient(
        clock,
        SimpleNamespace(planner_interfaces=[
            SimpleNamespace(pipeline_id="ompl"),
            SimpleNamespace(pipeline_id="pilz_industrial_motion_planner"),
        ]),
    )

    result = observe_moveit(client)

    assert result["status"] == "ok"
    assert result["pipeline_ids"] == (
        "ompl", "pilz_industrial_motion_planner"
    )
    assert result["discovery_latency"] == 0.0
    assert result["response_latency"] == 0.0
    assert client.calls == 1


def test_moveit_service_response_timeout_removes_pending_request():
    clock = FakeClock()
    client = FakeMoveItClient(clock, response=None, ready_after=None)

    result = observe_moveit(client, response_timeout=0.2)

    assert result["status"] == "response_timeout"
    assert len(client.removed) == 1
    assert client.calls == 1


def test_moveit_service_observer_rejects_missing_ompl_pipeline():
    clock = FakeClock()
    client = FakeMoveItClient(
        clock,
        SimpleNamespace(planner_interfaces=[SimpleNamespace(pipeline_id="pilz")]),
    )

    result = observe_moveit(client)

    assert result["status"] == "missing_required_pipeline"
    assert result["pipeline_ids"] == ("pilz",)


def test_main_routes_moveit_mode_to_moveit_preflight(monkeypatch, tmp_path):
    called = []

    def moveit_preflight(evidence_dir):
        called.append(evidence_dir)
        return 29

    monkeypatch.setattr(PREFLIGHT, "moveit_preflight", moveit_preflight)
    assert PREFLIGHT.main([
        "moveit", "--evidence-dir", str(tmp_path)
    ]) == 29
    assert called == [tmp_path]


def test_factory_declares_direct_lifecycle_messages_runtime_dependency():
    package_xml = (ROOT / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>lifecycle_msgs</exec_depend>" in package_xml


def test_factory_declares_direct_moveit_messages_runtime_dependency():
    package_xml = (ROOT / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>moveit_msgs</exec_depend>" in package_xml
