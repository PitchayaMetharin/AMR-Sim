#!/usr/bin/env python3
"""Run the standalone Smac planner replay backends as one gated test.

The backends intentionally return non-zero when a planner or its smoothed
candidate path fails the replay safety checks.  This runner preserves those
reports and stops at the first mandatory failure by default.  Use
``--continue-after-failure`` only when collecting independent diagnostics;
that mode does not change the final pass/fail result.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True, type=Path)
    parser.add_argument("--backend-2d", required=True, type=Path)
    parser.add_argument("--backend-lattice", required=True, type=Path)
    parser.add_argument("--lattice", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--ros-domain-id", type=int, default=232)
    parser.add_argument("--run07-regression", action="store_true",
                        help="expect the recorded unsafe 2D control and test lattice rejection cases")
    parser.add_argument(
        "--continue-after-failure",
        action="store_true",
        help="run later independent cases after a failed mandatory case",
    )
    return parser.parse_args()


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def run_case(
    name: str,
    command: list[str],
    output_path: Path,
    environment: dict[str, str],
) -> dict[str, Any]:
    try:
        output_path.unlink()
    except FileNotFoundError:
        pass
    completed = subprocess.run(
        command,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    result = read_json(output_path) if output_path.exists() else None
    logical_success = isinstance(result, dict) and result.get("success") is True
    return {
        "name": name,
        "command": command,
        "output": str(output_path),
        "returncode": completed.returncode,
        "crashed": completed.returncode < 0 or completed.returncode >= 128,
        "logical_success": logical_success,
        "success": completed.returncode == 0 and logical_success,
        "report": result,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def main() -> int:
    args = parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        raise SystemExit("--ros-domain-id must be in the inclusive range 0..232")

    report_dir = args.report.parent
    report_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.ros_domain_id)

    cases: list[dict[str, Any]] = []
    case_specs = [
        (
            "smac_2d",
            args.backend_2d,
            ["--planner", "2d"],
        ),
        (
            "smac_lattice",
            args.backend_lattice,
            [
                "--planner",
                "lattice",
                "--lattice",
                str(args.lattice),
            ],
        ),
    ]
    if args.run07_regression:
        for mutation in ("block", "unknown"):
            for endpoint in ("start", "goal"):
                case_specs.append((
                    f"{mutation}_{endpoint}", args.backend_lattice,
                    ["--planner", "lattice", "--lattice", str(args.lattice),
                     f"--{mutation}-endpoint", endpoint, "--allow-unknown", "false"],
                ))

    stopped_at: str | None = None
    for name, backend, planner_args in case_specs:
        output_path = report_dir / f"{args.report.stem}.{name}.json"
        command = [
            str(backend),
            "--snapshot",
            str(args.snapshot),
            *planner_args,
            "--run-smoother",
            "--output",
            str(output_path),
        ]
        case = run_case(name, command, output_path, environment)
        if args.run07_regression:
            payload = case["report"] or {}
            if name == "smac_2d":
                control = payload.get("smac_2d", {})
                case["expected"] = "2D generates a path rejected for lethal footprint overlap"
                case["success"] = (
                    case["returncode"] == 2 and control.get("planner_success") is True
                    and control.get("collision_free") is False
                    and control.get("collision", {}).get("worst_cost") == 254
                )
            elif name != "smac_lattice":
                candidate = payload.get("smac_lattice", {})
                case["expected"] = "planner rejects blocked or forbidden-unknown endpoint"
                case["success"] = (
                    case["returncode"] == 2 and candidate.get("success") is False
                    and candidate.get("planner_success", False) is False
                    and ("Starting point in lethal space" in candidate.get("error", "")
                         or candidate.get("error") == "SmacPlannerLattice createPath returned false")
                )
            else:
                control = payload.get("simple_smoother", {})
                case["success"] = case["success"] and (
                    control.get("smoother_completed") is True
                    and control.get("collision_free") is False
                    and control.get("collision", {}).get("worst_cost") == 254
                )
        cases.append(case)
        if not case["success"] and not args.continue_after_failure:
            stopped_at = name
            break

    overall_success = len(cases) == len(case_specs) and all(
        case["success"] for case in cases
    )
    report = {
        "schema": 1,
        "snapshot": str(args.snapshot),
        "lattice": str(args.lattice),
        "ros_domain_id": args.ros_domain_id,
        "stop_on_failure": not args.continue_after_failure,
        "run07_regression": args.run07_regression,
        "stopped_at": stopped_at,
        "cases": cases,
        "success": overall_success,
    }
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if overall_success else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f"planner replay runner failed: {error}", file=sys.stderr)
        sys.exit(1)
