#!/usr/bin/env python3

"""Fail-closed renderer, graph, lifecycle, and runtime evidence checks."""

import argparse
from collections import Counter
import grp
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys
import time


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
REQUIRED_GRAPH_NODES = (
    "/amr/amcl",
    "/amr/base_adapter_node",
    "/amr/command_arbitration_node",
    "/amr/controller_server",
    "/amr/front_lidar_adapter_node",
    "/amr/front_lidar_perception_node",
    "/amr/global_costmap/global_costmap",
    "/amr/imu_adapter_node",
    "/amr/local_costmap/local_costmap",
    "/amr/map_server",
    "/amr/mission_supervisor_node",
    "/amr/planner_server",
    "/amr/product_camera_adapter_node",
    "/amr/rear_lidar_adapter_node",
    "/amr/rear_lidar_perception_node",
    "/amr/smoother_server",
    "/amr/wheel_odometry_node",
    "/move_group",
)
GRAPH_DISCOVERY_TIMEOUT_SECONDS = 30.0
GRAPH_STABILITY_SECONDS = 2.0
GRAPH_POLL_SECONDS = 0.1
REQUIRED_LIFECYCLE_NODES = (
    "/amr/amcl",
    "/amr/base_adapter_node",
    "/amr/command_arbitration_node",
    "/amr/controller_server",
    "/amr/front_lidar_adapter_node",
    "/amr/front_lidar_perception_node",
    "/amr/global_costmap/global_costmap",
    "/amr/imu_adapter_node",
    "/amr/local_costmap/local_costmap",
    "/amr/map_server",
    "/amr/mission_supervisor_node",
    "/amr/planner_server",
    "/amr/product_camera_adapter_node",
    "/amr/rear_lidar_adapter_node",
    "/amr/rear_lidar_perception_node",
    "/amr/smoother_server",
    "/amr/wheel_odometry_node",
)
LIFECYCLE_DISCOVERY_TIMEOUT_SECONDS = 30.0
LIFECYCLE_STABILITY_SECONDS = 2.0
LIFECYCLE_RESPONSE_TIMEOUT_SECONDS = 1.0
LIFECYCLE_POLL_SECONDS = 0.1
LIFECYCLE_ACTIVE_STATE_ID = 3
LIFECYCLE_ACTIVE_STATE_LABEL = "active"
MOVEIT_SERVICE_NAME = "/query_planner_interface"
MOVEIT_DISCOVERY_TIMEOUT_SECONDS = 30.0
MOVEIT_RESPONSE_TIMEOUT_SECONDS = 1.0
MOVEIT_POLL_SECONDS = 0.1
MOVEIT_REQUIRED_PIPELINE_ID = "ompl"


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


def canonical_node_name(name, namespace):
    namespace = namespace.rstrip("/")
    return f"{namespace}/{name}" if namespace else f"/{name}"


def graph_snapshot(node):
    names = [
        canonical_node_name(name, namespace)
        for name, namespace in node.get_node_names_and_namespaces()
    ]
    counts = Counter(names)
    observed = tuple(sorted(counts))
    missing = tuple(sorted(set(REQUIRED_GRAPH_NODES) - set(observed)))
    duplicates = tuple(
        name for name in REQUIRED_GRAPH_NODES if counts.get(name, 0) > 1
    )
    return observed, missing, duplicates


def observe_required_graph(
        node, spin_once, monotonic=time.monotonic,
        discovery_timeout=GRAPH_DISCOVERY_TIMEOUT_SECONDS,
        stability_seconds=GRAPH_STABILITY_SECONDS,
        poll_seconds=GRAPH_POLL_SECONDS):
    """Observe one persistent ROS graph until it is complete and stable."""
    start = monotonic()
    complete_since = None
    samples = 0
    transitions = []
    last_state = None
    observed = ()
    missing = REQUIRED_GRAPH_NODES
    duplicates = ()

    while True:
        elapsed = monotonic() - start
        remaining = discovery_timeout - elapsed
        if remaining <= 0.0:
            break
        spin_once(node, timeout_sec=min(poll_seconds, remaining))
        samples += 1
        now = monotonic()
        observed, missing, duplicates = graph_snapshot(node)
        state = (missing, duplicates)
        if state != last_state:
            transitions.append({
                "elapsed": now - start,
                "observed_count": len(observed),
                "missing": missing,
                "duplicates": duplicates,
            })
            last_state = state

        if not missing and not duplicates:
            if complete_since is None:
                complete_since = now
            if now - complete_since >= stability_seconds:
                return {
                    "passed": True,
                    "elapsed": now - start,
                    "samples": samples,
                    "stable_seconds": now - complete_since,
                    "observed": observed,
                    "missing": missing,
                    "duplicates": duplicates,
                    "transitions": transitions,
                }
        else:
            complete_since = None

    now = monotonic()
    return {
        "passed": False,
        "elapsed": now - start,
        "samples": samples,
        "stable_seconds": (
            now - complete_since if complete_since is not None else 0.0
        ),
        "observed": observed,
        "missing": missing,
        "duplicates": duplicates,
        "transitions": transitions,
    }


def graph_preflight(evidence_dir):
    """Require one persistent observer to see the complete stable runtime graph."""
    import rclpy
    from rclpy.utilities import get_rmw_implementation_identifier

    lines = [
        "mode=graph",
        "observer=persistent_rclpy",
        f"ros_domain_id={os.environ.get('ROS_DOMAIN_ID', '<unset>')}",
        f"ros_localhost_only={os.environ.get('ROS_LOCALHOST_ONLY', '<unset>')}",
        "fastdds_builtin_transports="
        f"{os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', '<unset>')}",
        f"rmw_environment={os.environ.get('RMW_IMPLEMENTATION', '<default>')}",
        f"discovery_timeout_seconds={GRAPH_DISCOVERY_TIMEOUT_SECONDS:.1f}",
        f"stability_seconds={GRAPH_STABILITY_SECONDS:.1f}",
        "required_nodes=" + " ".join(REQUIRED_GRAPH_NODES),
    ]
    node = None
    initialized = False
    try:
        rclpy.init(args=[])
        initialized = True
        lines.append(f"rmw_actual={get_rmw_implementation_identifier()}")
        node = rclpy.create_node("amr_factory_graph_readiness")
        result = observe_required_graph(node, rclpy.spin_once)
    except Exception as error:  # Fail closed and retain the cause in evidence.
        lines.extend(["verdict=FAIL", f"error={type(error).__name__}: {error}"])
        result = None
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()

    if result is not None:
        for index, transition in enumerate(result["transitions"], start=1):
            lines.append(
                f"observation={index} elapsed={transition['elapsed']:.3f} "
                f"observed_count={transition['observed_count']} "
                "missing=" + (
                    " ".join(transition["missing"])
                    if transition["missing"] else "<none>"
                ) + " duplicates=" + (
                    " ".join(transition["duplicates"])
                    if transition["duplicates"] else "<none>"
                )
            )
        lines.extend([
            f"samples={result['samples']}",
            f"elapsed_seconds={result['elapsed']:.3f}",
            f"stable_seconds={result['stable_seconds']:.3f}",
            "final_observed_nodes=" + " ".join(result["observed"]),
            "final_missing=" + (
                " ".join(result["missing"]) if result["missing"] else "<none>"
            ),
            "final_duplicates=" + (
                " ".join(result["duplicates"])
                if result["duplicates"] else "<none>"
            ),
            "verdict=" + ("PASS" if result["passed"] else "FAIL"),
        ])
        if not result["passed"]:
            lines.append(
                "error=required graph did not remain complete and unique for "
                f"{GRAPH_STABILITY_SECONDS:.1f} seconds"
            )

    path = write_report(evidence_dir, "graph_readiness.txt", lines)
    for line in lines:
        print(line)
    print(f"evidence={path}")
    return 0 if result is not None and result["passed"] else 1


def query_lifecycle_state(
        node, client, spin_once, request_factory, monotonic,
        response_timeout=LIFECYCLE_RESPONSE_TIMEOUT_SECONDS,
        poll_seconds=LIFECYCLE_POLL_SECONDS):
    """Issue one bounded lifecycle query through an existing persistent client."""
    start = monotonic()
    try:
        if not client.service_is_ready():
            return {
                "status": "service_unavailable",
                "latency": monotonic() - start,
            }
        future = client.call_async(request_factory())
    except Exception as error:
        return {
            "status": "query_error",
            "detail": f"{type(error).__name__}: {error}",
            "latency": monotonic() - start,
        }

    while not future.done():
        remaining = response_timeout - (monotonic() - start)
        if remaining <= 0.0:
            try:
                client.remove_pending_request(future)
            except Exception:
                pass
            return {
                "status": "response_timeout",
                "latency": monotonic() - start,
            }
        spin_once(node, timeout_sec=min(poll_seconds, remaining))

    try:
        response = future.result()
        state = response.current_state
        state_id = int(state.id)
        state_label = str(state.label)
    except Exception as error:
        return {
            "status": "query_error",
            "detail": f"{type(error).__name__}: {error}",
            "latency": monotonic() - start,
        }
    return {
        "status": "ok",
        "state_id": state_id,
        "state_label": state_label,
        "latency": monotonic() - start,
    }


def lifecycle_state_groups(states, required_nodes):
    unavailable = tuple(
        name for name in required_nodes
        if states.get(name, {}).get("status") == "service_unavailable"
    )
    response_timeouts = tuple(
        name for name in required_nodes
        if states.get(name, {}).get("status") == "response_timeout"
    )
    query_errors = tuple(
        name for name in required_nodes
        if states.get(name, {}).get("status") in {"query_error", "not_queried"}
    )
    nonactive = tuple(
        name for name in required_nodes
        if states.get(name, {}).get("status") == "ok"
        and (
            states[name].get("state_id") != LIFECYCLE_ACTIVE_STATE_ID
            or states[name].get("state_label") != LIFECYCLE_ACTIVE_STATE_LABEL
        )
    )
    return unavailable, response_timeouts, query_errors, nonactive


def lifecycle_observation_result(
        passed, start, now, samples, active_since, states, transitions,
        required_nodes):
    unavailable, response_timeouts, query_errors, nonactive = (
        lifecycle_state_groups(states, required_nodes)
    )
    return {
        "passed": passed,
        "elapsed": now - start,
        "samples": samples,
        "stable_seconds": (
            now - active_since if active_since is not None else 0.0
        ),
        "final_states": states,
        "unavailable_services": unavailable,
        "response_timeouts": response_timeouts,
        "query_errors": query_errors,
        "nonactive": nonactive,
        "transitions": transitions,
    }


def observe_required_lifecycle(
        node, clients, spin_once, request_factory, monotonic=time.monotonic,
        discovery_timeout=LIFECYCLE_DISCOVERY_TIMEOUT_SECONDS,
        stability_seconds=LIFECYCLE_STABILITY_SECONDS,
        response_timeout=LIFECYCLE_RESPONSE_TIMEOUT_SECONDS,
        poll_seconds=LIFECYCLE_POLL_SECONDS,
        required_nodes=REQUIRED_LIFECYCLE_NODES):
    """Observe lifecycle state with one participant until all nodes are stable."""
    start = monotonic()
    active_since = None
    samples = 0
    transitions = []
    last_signature = None
    states = {
        name: {"status": "not_queried", "latency": 0.0}
        for name in required_nodes
    }

    while monotonic() - start < discovery_timeout:
        current = {}
        for name in required_nodes:
            remaining = discovery_timeout - (monotonic() - start)
            if remaining <= 0.0:
                current[name] = {"status": "not_queried", "latency": 0.0}
                continue
            current[name] = query_lifecycle_state(
                node,
                clients[name],
                spin_once,
                request_factory,
                monotonic,
                response_timeout=min(response_timeout, remaining),
                poll_seconds=poll_seconds,
            )
        samples += 1
        states = current
        now = monotonic()
        signature = tuple(
            (
                name,
                states[name].get("status"),
                states[name].get("state_id"),
                states[name].get("state_label"),
                states[name].get("detail"),
            )
            for name in required_nodes
        )
        if signature != last_signature:
            transitions.append({
                "elapsed": now - start,
                "states": {name: dict(states[name]) for name in required_nodes},
            })
            last_signature = signature

        unavailable, response_timeouts, query_errors, nonactive = (
            lifecycle_state_groups(states, required_nodes)
        )
        all_active = not (
            unavailable or response_timeouts or query_errors or nonactive
        )
        if all_active:
            if active_since is None:
                active_since = now
            if now - active_since >= stability_seconds:
                return lifecycle_observation_result(
                    True, start, now, samples, active_since, states,
                    transitions, required_nodes
                )
        else:
            active_since = None

        remaining = discovery_timeout - (monotonic() - start)
        if remaining > 0.0:
            spin_once(node, timeout_sec=min(poll_seconds, remaining))

    now = monotonic()
    return lifecycle_observation_result(
        False, start, now, samples, active_since, states, transitions,
        required_nodes
    )


def format_lifecycle_states(states):
    formatted = []
    for name, state in states.items():
        entry = f"{name}:{state['status']}"
        if state["status"] == "ok":
            entry += f":{state['state_id']}:{state['state_label']}"
        if state.get("detail"):
            entry += f":{state['detail']}"
        entry += f":latency={state['latency']:.6f}"
        formatted.append(entry)
    return " ".join(formatted)


def lifecycle_preflight(evidence_dir):
    """Require all runtime lifecycle nodes to remain exactly active."""
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rclpy.utilities import get_rmw_implementation_identifier

    lines = [
        "mode=lifecycle",
        "observer=persistent_rclpy",
        f"ros_domain_id={os.environ.get('ROS_DOMAIN_ID', '<unset>')}",
        f"ros_localhost_only={os.environ.get('ROS_LOCALHOST_ONLY', '<unset>')}",
        "fastdds_builtin_transports="
        f"{os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', '<unset>')}",
        f"rmw_environment={os.environ.get('RMW_IMPLEMENTATION', '<default>')}",
        f"discovery_timeout_seconds={LIFECYCLE_DISCOVERY_TIMEOUT_SECONDS:.1f}",
        f"response_timeout_seconds={LIFECYCLE_RESPONSE_TIMEOUT_SECONDS:.1f}",
        f"stability_seconds={LIFECYCLE_STABILITY_SECONDS:.1f}",
        "required_nodes=" + " ".join(REQUIRED_LIFECYCLE_NODES),
    ]
    node = None
    initialized = False
    result = None
    try:
        rclpy.init(args=[])
        initialized = True
        lines.append(f"rmw_actual={get_rmw_implementation_identifier()}")
        node = rclpy.create_node("amr_factory_lifecycle_readiness")
        clients = {
            name: node.create_client(GetState, f"{name}/get_state")
            for name in REQUIRED_LIFECYCLE_NODES
        }
        result = observe_required_lifecycle(
            node, clients, rclpy.spin_once, GetState.Request
        )
    except Exception as error:  # Fail closed and retain the cause in evidence.
        lines.extend(["verdict=FAIL", f"error={type(error).__name__}: {error}"])
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()

    if result is not None:
        for index, transition in enumerate(result["transitions"], start=1):
            lines.append(
                f"observation={index} elapsed={transition['elapsed']:.3f} "
                f"states={format_lifecycle_states(transition['states'])}"
            )
        lines.extend([
            f"samples={result['samples']}",
            f"elapsed_seconds={result['elapsed']:.3f}",
            f"stable_seconds={result['stable_seconds']:.3f}",
            "final_states=" + format_lifecycle_states(result["final_states"]),
            "final_unavailable_services=" + (
                " ".join(result["unavailable_services"])
                if result["unavailable_services"] else "<none>"
            ),
            "final_response_timeouts=" + (
                " ".join(result["response_timeouts"])
                if result["response_timeouts"] else "<none>"
            ),
            "final_query_errors=" + (
                " ".join(result["query_errors"])
                if result["query_errors"] else "<none>"
            ),
            "final_nonactive=" + (
                " ".join(result["nonactive"])
                if result["nonactive"] else "<none>"
            ),
            "verdict=" + ("PASS" if result["passed"] else "FAIL"),
        ])
        if not result["passed"]:
            lines.append(
                "error=required lifecycle nodes did not remain exactly active "
                f"for {LIFECYCLE_STABILITY_SECONDS:.1f} seconds"
            )

    path = write_report(evidence_dir, "lifecycle_readiness.txt", lines)
    for line in lines:
        print(line)
    print(f"evidence={path}")
    return 0 if result is not None and result["passed"] else 1


def query_moveit_planner_service(
        node, client, spin_once, request_factory, monotonic,
        response_timeout=MOVEIT_RESPONSE_TIMEOUT_SECONDS,
        poll_seconds=MOVEIT_POLL_SECONDS):
    """Issue one bounded planner-interface query on a persistent client."""
    start = monotonic()
    try:
        future = client.call_async(request_factory())
    except Exception as error:
        return {
            "status": "query_error",
            "detail": f"{type(error).__name__}: {error}",
            "pipeline_ids": (),
            "response_latency": monotonic() - start,
        }

    while not future.done():
        remaining = response_timeout - (monotonic() - start)
        if remaining <= 0.0:
            try:
                client.remove_pending_request(future)
            except Exception:
                pass
            return {
                "status": "response_timeout",
                "detail": (
                    "planner-interface response did not arrive within "
                    f"{response_timeout:.1f} seconds"
                ),
                "pipeline_ids": (),
                "response_latency": monotonic() - start,
            }
        spin_once(node, timeout_sec=min(poll_seconds, remaining))

    try:
        response = future.result()
        planner_interfaces = response.planner_interfaces
        pipeline_ids = []
        for interface in planner_interfaces:
            pipeline_id = getattr(interface, "pipeline_id")
            if not isinstance(pipeline_id, str):
                raise ValueError("planner interface pipeline_id is not a string")
            pipeline_ids.append(pipeline_id)
        pipeline_ids = tuple(pipeline_ids)
    except Exception as error:
        return {
            "status": "query_error",
            "detail": f"{type(error).__name__}: {error}",
            "pipeline_ids": (),
            "response_latency": monotonic() - start,
        }

    response_latency = monotonic() - start
    if MOVEIT_REQUIRED_PIPELINE_ID not in pipeline_ids:
        return {
            "status": "missing_required_pipeline",
            "detail": (
                f"required pipeline_id={MOVEIT_REQUIRED_PIPELINE_ID!r} "
                "was not returned"
            ),
            "pipeline_ids": pipeline_ids,
            "response_latency": response_latency,
        }
    return {
        "status": "ok",
        "pipeline_ids": pipeline_ids,
        "response_latency": response_latency,
    }


def observe_moveit_planner_service(
        node, client, spin_once, request_factory, monotonic=time.monotonic,
        discovery_timeout=MOVEIT_DISCOVERY_TIMEOUT_SECONDS,
        response_timeout=MOVEIT_RESPONSE_TIMEOUT_SECONDS,
        poll_seconds=MOVEIT_POLL_SECONDS):
    """Wait for one planner service, then issue one bounded query."""
    discovery_start = monotonic()
    while True:
        try:
            if client.service_is_ready():
                break
        except Exception as error:
            return {
                "status": "query_error",
                "detail": f"{type(error).__name__}: {error}",
                "pipeline_ids": (),
                "discovery_latency": monotonic() - discovery_start,
                "response_latency": 0.0,
            }

        remaining = discovery_timeout - (monotonic() - discovery_start)
        if remaining <= 0.0:
            return {
                "status": "service_unavailable",
                "detail": (
                    "planner-interface service was not ready within "
                    f"{discovery_timeout:.1f} seconds"
                ),
                "pipeline_ids": (),
                "discovery_latency": monotonic() - discovery_start,
                "response_latency": 0.0,
            }
        spin_once(node, timeout_sec=min(poll_seconds, remaining))

    discovery_latency = monotonic() - discovery_start
    result = query_moveit_planner_service(
        node,
        client,
        spin_once,
        request_factory,
        monotonic,
        response_timeout=response_timeout,
        poll_seconds=poll_seconds,
    )
    result["discovery_latency"] = discovery_latency
    return result


def moveit_preflight(evidence_dir):
    """Require a persistent planner-service query to return the OMPL pipeline."""
    lines = [
        "mode=moveit",
        "observer=persistent_rclpy",
        f"service={MOVEIT_SERVICE_NAME}",
        "service_type=moveit_msgs/srv/QueryPlannerInterfaces",
        f"required_pipeline_id={MOVEIT_REQUIRED_PIPELINE_ID}",
        f"ros_domain_id={os.environ.get('ROS_DOMAIN_ID', '<unset>')}",
        f"ros_localhost_only={os.environ.get('ROS_LOCALHOST_ONLY', '<unset>')}",
        "fastdds_builtin_transports="
        f"{os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', '<unset>')}",
        f"rmw_environment={os.environ.get('RMW_IMPLEMENTATION', '<default>')}",
        f"discovery_timeout_seconds={MOVEIT_DISCOVERY_TIMEOUT_SECONDS:.1f}",
        f"response_timeout_seconds={MOVEIT_RESPONSE_TIMEOUT_SECONDS:.1f}",
        f"poll_seconds={MOVEIT_POLL_SECONDS:.1f}",
    ]
    rclpy = None
    node = None
    initialized = False
    result = None
    try:
        import rclpy
        from moveit_msgs.srv import QueryPlannerInterfaces
        from rclpy.utilities import get_rmw_implementation_identifier

        rclpy.init(args=[])
        initialized = True
        lines.append(f"rmw_actual={get_rmw_implementation_identifier()}")
        node = rclpy.create_node("amr_factory_moveit_readiness")
        client = node.create_client(QueryPlannerInterfaces, MOVEIT_SERVICE_NAME)
        result = observe_moveit_planner_service(
            node,
            client,
            rclpy.spin_once,
            QueryPlannerInterfaces.Request,
        )
    except Exception as error:  # Fail closed and retain the cause in evidence.
        lines.extend(["verdict=FAIL", f"error={type(error).__name__}: {error}"])
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()

    if result is not None:
        lines.extend([
            f"status={result['status']}",
            f"discovery_latency_seconds={result['discovery_latency']:.9f}",
            f"response_latency_seconds={result['response_latency']:.9f}",
            "planner_pipeline_ids=" + (
                " ".join(result["pipeline_ids"])
                if result["pipeline_ids"] else "<none>"
            ),
        ])
        passed = result["status"] == "ok"
        lines.append("verdict=" + ("PASS" if passed else "FAIL"))
        if not passed:
            lines.append("error=" + result.get("detail", result["status"]))

    path = write_report(evidence_dir, "moveit_readiness.txt", lines)
    for line in lines:
        print(line)
    print(f"evidence={path}")
    return 0 if result is not None and result["status"] == "ok" else 1


def main(argv=None):
    parser = argparse.ArgumentParser(prog="factory_runtime_preflight.py")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    for mode in ("host", "runtime", "graph", "lifecycle", "moveit"):
        command = subparsers.add_parser(mode)
        command.add_argument("--evidence-dir", type=Path, required=True)
    arguments = parser.parse_args(argv)
    if arguments.mode == "host":
        return host_preflight(arguments.evidence_dir)
    if arguments.mode == "runtime":
        return runtime_preflight(arguments.evidence_dir)
    if arguments.mode == "graph":
        return graph_preflight(arguments.evidence_dir)
    if arguments.mode == "lifecycle":
        return lifecycle_preflight(arguments.evidence_dir)
    return moveit_preflight(arguments.evidence_dir)


if __name__ == "__main__":
    sys.exit(main())
