#!/usr/bin/env python3
"""Run the controlled GUI AWS diagnostics reproduction with durable commands."""

import argparse
import json
from pathlib import Path
import sys


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from aws_exploration_diagnostics import evidence_topics  # noqa: E402
from aws_exploration_runner import (  # noqa: E402
    RunConfig,
    RunnerError,
    run_aws_exploration,
)


def build_run_config(
    run_dir: Path,
    ros_domain_id: int,
    *,
    workspace: Path | None = None,
    max_runtime_sec: float | None = None,
) -> RunConfig:
    """Build the one maintained command set for the missing Run08 evidence."""
    run_dir = Path(run_dir)
    workspace = Path.cwd() if workspace is None else Path(workspace)
    bag_dir = run_dir / "evidence" / "exploration"
    map_output_prefix = run_dir / "evidence" / "aws_map"
    verdict_file = run_dir / "verdict.json"

    recorder = (
        "ros2",
        "bag",
        "record",
        "--storage",
        "sqlite3",
        "--include-hidden-topics",
        "--include-unpublished-topics",
        "--max-cache-size",
        "104857600",
        "-o",
        str(bag_dir),
        *evidence_topics(),
    )
    monitor = (
        sys.executable,
        str(SCRIPT_DIRECTORY / "aws_exploration_monitor.py"),
        "--verdict-file",
        str(verdict_file),
    )
    diagnostics = (
        sys.executable,
        str(SCRIPT_DIRECTORY / "aws_exploration_diagnostics.py"),
        "--run-dir",
        str(run_dir),
        "--simulation-evidence",
    )
    map_saver = (
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        str(map_output_prefix),
        "-t",
        "/map",
        "--ros-args",
        "-p",
        "map_subscribe_transient_local:=true",
        "-p",
        "use_sim_time:=true",
    )
    launch = (
        "ros2",
        "launch",
        "amr_simulation",
        "aws_warehouse_exploration.launch.py",
        "headless:=false",
        "software_rendering:=auto",
        "rviz:=true",
        "auto_start_exploration:=true",
        "simulation_diagnostics:=true",
        "launch-prefix-filter:=frontier_explorer.py",
        "launch-prefix:=bash -c 'exec \"$@\" --ros-args -p runtime_diagnostics:=true' --",
    )
    return RunConfig(
        run_dir=run_dir,
        ros_domain_id=ros_domain_id,
        recorder=recorder,
        monitor=monitor,
        diagnostics=diagnostics,
        launch=launch,
        cwd=workspace,
        verdict_file=verdict_file,
        bag_dir=bag_dir,
        bag_inspector=("ros2", "bag", "info", str(bag_dir)),
        map_saver=map_saver,
        map_output_prefix=map_output_prefix,
        max_runtime_sec=max_runtime_sec,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--ros-domain-id", required=True, type=int)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--max-runtime-sec", type=float)
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    config = build_run_config(
        args.run_dir,
        args.ros_domain_id,
        workspace=args.workspace,
        max_runtime_sec=args.max_runtime_sec,
    )
    try:
        result = run_aws_exploration(config)
    except RunnerError as error:
        print(f"AWS diagnostics run rejected: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get("pass") else 1


if __name__ == "__main__":
    raise SystemExit(main())
