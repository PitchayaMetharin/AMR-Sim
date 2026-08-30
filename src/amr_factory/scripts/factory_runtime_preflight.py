#!/usr/bin/env python3

"""Fail-closed renderer and Gazebo real-time-factor evidence checks."""

import argparse
import grp
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys


MIN_SAMPLES = 10
MIN_MEDIAN_RTF = 0.90
MIN_AGGREGATE_RTF = 0.90
STATS_CAPTURE_SECONDS = 12.0
PROCESS_MARKERS = (
    "gz sim",
    "gzserver",
    "move_group",
    "rviz2",
    "gate6_mass_stage",
    "ros2 bag record",
    "rosbag2",
)


def accessible_render_devices(dri_path="/dev/dri"):
    """Return readable/writable DRM render nodes."""
    if not os.path.isdir(dri_path):
        return []
    try:
        names = os.listdir(dri_path)
    except OSError:
        return []
    return [
        os.path.join(dri_path, name)
        for name in sorted(names)
        if name.startswith("renderD")
        and os.access(os.path.join(dri_path, name), os.R_OK | os.W_OK)
    ]


def forced_software_environment(environment=None):
    """Return relevant environment entries that force software OpenGL."""
    environment = os.environ if environment is None else environment
    forced = {}
    always_software = environment.get("LIBGL_ALWAYS_SOFTWARE", "").strip().lower()
    gallium_driver = environment.get("GALLIUM_DRIVER", "").strip().lower()
    if always_software in {"1", "true", "yes", "on"}:
        forced["LIBGL_ALWAYS_SOFTWARE"] = environment.get("LIBGL_ALWAYS_SOFTWARE")
    if gallium_driver == "llvmpipe":
        forced["GALLIUM_DRIVER"] = environment.get("GALLIUM_DRIVER")
    return forced


def process_command(pid, proc_root=Path("/proc")):
    try:
        command = (proc_root / str(pid) / "cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, OSError):
        return ""
    return command.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()


def calling_process_ancestry(proc_root=Path("/proc"), start_pid=None):
    """Return this preflight process and its parents, bounded at PID 1."""
    current_pid = os.getpid() if start_pid is None else start_pid
    ancestors = set()
    while current_pid > 0 and current_pid not in ancestors:
        ancestors.add(current_pid)
        try:
            fields = (proc_root / str(current_pid) / "stat").read_text(
                encoding="utf-8").split()
            current_pid = int(fields[3])
        except (FileNotFoundError, PermissionError, OSError, IndexError, ValueError):
            break
    return ancestors


def matching_processes(proc_root=Path("/proc"), ignored_pids=None):
    """Return known simulation processes as (pid, command) pairs."""
    ignored_pids = (
        calling_process_ancestry(proc_root)
        if ignored_pids is None else set(ignored_pids)
    )
    processes = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        if int(entry.name) in ignored_pids:
            continue
        command = process_command(entry.name, proc_root)
        if command and any(marker in command for marker in PROCESS_MARKERS):
            processes.append((int(entry.name), command))
    return sorted(processes)


def gazebo_processes(processes=None):
    processes = matching_processes() if processes is None else processes
    return [(pid, command) for pid, command in processes
            if "gz sim" in command or "gzserver" in command]


def process_render_devices(pid):
    devices = []
    try:
        descriptors = Path(f"/proc/{pid}/fd").iterdir()
        for descriptor in descriptors:
            try:
                target = os.readlink(descriptor)
            except (FileNotFoundError, PermissionError, OSError):
                continue
            if target.startswith("/dev/dri/") and target not in devices:
                devices.append(target)
    except (FileNotFoundError, PermissionError, OSError):
        pass
    return sorted(devices)


def group_names():
    names = []
    for group_id in os.getgroups():
        try:
            names.append(grp.getgrgid(group_id).gr_name)
        except KeyError:
            names.append(str(group_id))
    return sorted(set(names))


def write_report(evidence_dir, filename, lines):
    evidence_dir.mkdir(parents=True, exist_ok=True)
    path = evidence_dir / filename
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def host_preflight(evidence_dir):
    devices = accessible_render_devices()
    forced = forced_software_environment()
    processes = matching_processes()
    errors = []
    if not devices:
        errors.append("no readable/writable /dev/dri/renderD* device")
    if forced:
        errors.append("environment forces software OpenGL")
    if processes:
        errors.append("known simulation processes are already running")

    lines = [
        "mode=host",
        f"uid={os.getuid()}",
        f"groups={','.join(group_names()) or '<none>'}",
        f"render_devices={','.join(devices) or '<none>'}",
        "forced_software=" + (
            ",".join(f"{key}={value}" for key, value in forced.items())
            if forced else "<none>"
        ),
        "known_processes=" + (
            "; ".join(f"pid={pid} {command}" for pid, command in processes)
            if processes else "<none>"
        ),
    ]
    if errors:
        lines.append("verdict=FAIL")
        lines.extend(f"error={error}" for error in errors)
    else:
        lines.append("verdict=PASS")
    path = write_report(evidence_dir, "host_preflight.txt", lines)
    for line in lines:
        print(line)
    print(f"evidence={path}")
    return 1 if errors else 0


def _parse_time(block, field):
    match = re.search(rf"{field}\s*\{{(.*?)\}}", block, re.DOTALL)
    if not match:
        return None
    seconds = re.search(r"\bsec:\s*(-?\d+)", match.group(1))
    nanoseconds = re.search(r"\bnsec:\s*(\d+)", match.group(1))
    if not seconds or not nanoseconds:
        return None
    return int(seconds.group(1)) + int(nanoseconds.group(1)) / 1_000_000_000.0


def parse_stats(text):
    """Parse protobuf text-format /stats messages emitted by gz topic."""
    samples = []
    for block in re.split(r"\n\s*\n", text):
        sim_time = _parse_time(block, "sim_time")
        real_time = _parse_time(block, "real_time")
        factor = re.search(r"\breal_time_factor:\s*([-+0-9.eE]+)", block)
        if sim_time is None or real_time is None or factor is None:
            continue
        samples.append({
            "sim_time": sim_time,
            "real_time": real_time,
            "real_time_factor": float(factor.group(1)),
        })
    return samples


def summarize_stats(text):
    samples = parse_stats(text)
    if len(samples) < MIN_SAMPLES:
        raise RuntimeError(
            f"only {len(samples)} valid /stats samples; need at least {MIN_SAMPLES}"
        )
    sim_span = samples[-1]["sim_time"] - samples[0]["sim_time"]
    real_span = samples[-1]["real_time"] - samples[0]["real_time"]
    if sim_span <= 0.0 or real_span <= 0.0:
        raise RuntimeError("/stats simulated or wall-time span is not positive")
    factors = [sample["real_time_factor"] for sample in samples]
    return {
        "samples": len(samples),
        "real_span": real_span,
        "sim_span": sim_span,
        "aggregate_sim_real": sim_span / real_span,
        "min": min(factors),
        "median": statistics.median(factors),
        "mean": statistics.fmean(factors),
        "max": max(factors),
    }


def format_summary(summary):
    return [
        f"capture_seconds={STATS_CAPTURE_SECONDS:.1f}",
        f"samples={summary['samples']}",
        f"real_span={summary['real_span']:.9f}",
        f"sim_span={summary['sim_span']:.9f}",
        f"aggregate_sim_real={summary['aggregate_sim_real']:.15f}",
        f"min={summary['min']:.15f}",
        f"median={summary['median']:.15f}",
        f"mean={summary['mean']:.15f}",
        f"max={summary['max']:.15f}",
    ]


def capture_stats():
    try:
        return subprocess.run(
            [
                "gz", "topic", "-e", "-t", "/stats",
                "-d", str(STATS_CAPTURE_SECONDS),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=STATS_CAPTURE_SECONDS + 10.0,
            env=os.environ.copy(),
        )
    except FileNotFoundError as error:
        raise RuntimeError("gz topic executable is unavailable") from error
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("gz topic /stats capture exceeded its bounded timeout") from error


def runtime_preflight(evidence_dir):
    devices = accessible_render_devices()
    forced = forced_software_environment()
    processes = matching_processes()
    gazebos = gazebo_processes(processes)
    gpu_processes = [
        (pid, command, process_render_devices(pid))
        for pid, command in gazebos
        if process_render_devices(pid)
    ]
    errors = []
    if not devices:
        errors.append("no readable/writable /dev/dri/renderD* device")
    if forced:
        errors.append("environment forces software OpenGL")
    if not gazebos:
        errors.append("no Gazebo process found")
    if not gpu_processes:
        errors.append("no Gazebo process has an open /dev/dri device")

    lines = [
        "mode=runtime",
        f"render_devices={','.join(devices) or '<none>'}",
        "forced_software=" + (
            ",".join(f"{key}={value}" for key, value in forced.items())
            if forced else "<none>"
        ),
        "gazebo_processes=" + (
            "; ".join(
                f"pid={pid} gpu_devices={','.join(process_render_devices(pid)) or '<none>'}"
                for pid, _ in gazebos
            ) or "<none>"
        ),
    ]
    if errors:
        lines.append("verdict=FAIL")
        lines.extend(f"error={error}" for error in errors)
        path = write_report(evidence_dir, "runtime_preflight.txt", lines)
        for line in lines:
            print(line)
        print(f"evidence={path}")
        return 1

    result = capture_stats()
    stats_path = write_report(evidence_dir, "stats_raw.txt", [result.stdout.rstrip()])
    stderr_path = write_report(
        evidence_dir, "stats_stderr.txt", [result.stderr.rstrip()]
    )
    try:
        summary = summarize_stats(result.stdout)
    except RuntimeError as error:
        errors.append(str(error))
    else:
        lines.extend(format_summary(summary))
        if summary["median"] < MIN_MEDIAN_RTF:
            errors.append(
                f"median RTF {summary['median']:.6f} is below {MIN_MEDIAN_RTF:.2f}"
            )
        if summary["aggregate_sim_real"] < MIN_AGGREGATE_RTF:
            errors.append(
                "aggregate RTF "
                f"{summary['aggregate_sim_real']:.6f} is below {MIN_AGGREGATE_RTF:.2f}"
            )
    lines.extend([f"stats_raw={stats_path}", f"stats_stderr={stderr_path}"])
    lines.append("gz_topic_exit_code=" + str(result.returncode))
    if errors or result.returncode != 0:
        if result.returncode != 0:
            errors.append(f"gz topic exited with code {result.returncode}")
        lines.append("verdict=FAIL")
        lines.extend(f"error={error}" for error in errors)
    else:
        lines.append("verdict=PASS")
    path = write_report(evidence_dir, "runtime_preflight.txt", lines)
    for line in lines:
        print(line)
    print(f"evidence={path}")
    return 1 if errors else 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="factory_runtime_preflight.py")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    for mode in ("host", "runtime"):
        command = subparsers.add_parser(mode)
        command.add_argument("--evidence-dir", type=Path, required=True)
    arguments = parser.parse_args(argv)
    if arguments.mode == "host":
        return host_preflight(arguments.evidence_dir)
    return runtime_preflight(arguments.evidence_dir)


if __name__ == "__main__":
    sys.exit(main())
