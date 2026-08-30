import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "factory_runtime_preflight.py"
SPEC = importlib.util.spec_from_file_location("factory_runtime_preflight", SCRIPT)
PREFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREFLIGHT)


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
