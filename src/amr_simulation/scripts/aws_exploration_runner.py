#!/usr/bin/env python3
"""Run portable AWS exploration with owned processes and durable evidence.

The runner deliberately keeps recorder, observer, diagnostics, and launch in
one private POSIX session.  It does not open a rosbag while that session is
alive.  The command protocol is intentionally small so it can be exercised
without a ROS installation in unit tests:

* the observer writes a JSON terminal record to ``verdict.json`` (or the
  configured evidence path);
* ``state`` is one of ``COMPLETE``, ``INCOMPLETE``, or ``FAULT``;
* a pass requires explicit false/empty ``active``, ``pending``,
  ``cancel_owned_motion``, and ``cancel_target`` fields.

The parent process owns the unique run directory.  A forked session leader
owns every run process, records process identities, and performs bounded
signal escalation.  The implementation uses only the Python standard library
so the evidence wrapper remains usable before the ROS graph is available.
"""

import argparse
import dataclasses
import enum
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


ROS_DOMAIN_MIN = 0
ROS_DOMAIN_MAX = 232
RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
COMMAND_NAMES = ("recorder", "monitor", "diagnostics", "launch")
TERMINAL_STATES = frozenset(("COMPLETE", "INCOMPLETE", "FAULT"))
SAFE_REACHABLE_COMPLETION_POLICY = "SAFE_REACHABLE_AREA_V1"
TERMINAL_COUNT_FIELDS = (
    "run_generation",
    "motion_generation",
    "map_version",
    "goal_failures",
    "no_frontier_updates_seen",
    "reached_goal_count",
    "raw_frontier_count",
    "blocked_frontier_count",
    "blocked_safety_count",
    "blocked_route_count",
    "unresolved_frontier_count",
    "blocked_count",
    "retry_exhausted_count",
)
MISSION_OUTCOME_BY_STATE = {
    "COMPLETE": "SUCCEEDED",
    "INCOMPLETE": "ABORTED",
    "FAULT": "FAULT",
}

DEFAULT_GRACEFUL_SHUTDOWN_SEC = 10.0
DEFAULT_ESCALATION_SEC = 5.0
DEFAULT_POLL_INTERVAL_SEC = 0.05
DEFAULT_MAP_SAVE_TIMEOUT_SEC = 15.0

ABORT_LOG_PATTERNS = (
    re.compile(r"smoothing\s+(?:failed|aborted|abort)", re.IGNORECASE),
    re.compile(r"(?:failed|aborted|abort)\s+(?:to\s+)?smooth(?:e|ing)", re.IGNORECASE),
    re.compile(r"path\s+following\s+(?:failed|aborted|abort)", re.IGNORECASE),
    re.compile(r"path\s+follower[^\n]*(?:failed|aborted|abort)", re.IGNORECASE),
    re.compile(r"failed\s+to\s+follow\s+(?:the\s+)?path", re.IGNORECASE),
    re.compile(r"failed\s+to\s+make\s+progress", re.IGNORECASE),
    re.compile(r"no\s+valid\s+trajectory", re.IGNORECASE),
)


class RunClassification(str, enum.Enum):
    COMPLETE = "COMPLETE"
    INCOMPLETE = "INCOMPLETE"
    FAULT = "FAULT"
    RECORDER_FAILURE = "RECORDER_FAILURE"
    OBSERVER_FAILURE = "OBSERVER_FAILURE"
    INTERRUPTION = "INTERRUPTION"
    LAUNCH_EXIT = "LAUNCH_EXIT"


class RunnerError(RuntimeError):
    """A run cannot be started or its evidence contract is invalid."""


class LiveBagInspectionError(RunnerError):
    """Raised when a bag inspection is attempted before recorder shutdown."""


def validate_ros_domain_id(value: Any) -> int:
    """Validate the workspace's inclusive ROS domain range."""

    if isinstance(value, bool):
        raise RunnerError("ROS_DOMAIN_ID must be an integer in the range 0..232")
    try:
        domain = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise RunnerError("ROS_DOMAIN_ID must be an integer in the range 0..232") from error
    if str(value).strip() != str(domain):
        raise RunnerError("ROS_DOMAIN_ID must be a canonical integer in the range 0..232")
    if not ROS_DOMAIN_MIN <= domain <= ROS_DOMAIN_MAX:
        raise RunnerError("ROS_DOMAIN_ID must be in the range 0..232")
    return domain


def validate_run_identity(run_dir: Path | str, run_id: Optional[str] = None) -> str:
    """Validate a new absolute run directory without touching the filesystem."""

    path = Path(run_dir)
    if not path.is_absolute():
        raise RunnerError("run directory must be absolute")
    if path.name in ("", ".", "..") or not RUN_ID_RE.fullmatch(path.name):
        raise RunnerError("run directory basename is not a valid run identity")
    identity = path.name if run_id is None else str(run_id)
    if not RUN_ID_RE.fullmatch(identity):
        raise RunnerError("run identity is not a valid run id")
    if identity != path.name:
        raise RunnerError("run identity must match the run directory basename")
    if path.exists():
        raise RunnerError(f"run directory already exists: {path}")
    if path.parent.exists() and not path.parent.is_dir():
        raise RunnerError(f"run directory parent is not a directory: {path.parent}")
    return identity


def _as_command(value: Sequence[str] | str, name: str) -> Tuple[str, ...]:
    if isinstance(value, str):
        command = tuple(shlex.split(value))
    else:
        command = tuple(str(item) for item in value)
    if not command or any(not item for item in command):
        raise RunnerError(f"{name} command must not be empty")
    return command


def _path_inside(root: Path, candidate: Path, description: str) -> Path:
    root_resolved = root.resolve()
    candidate_resolved = candidate.resolve()
    try:
        candidate_resolved.relative_to(root_resolved)
    except ValueError as error:
        raise RunnerError(f"{description} must be inside the run directory") from error
    return candidate_resolved


@dataclasses.dataclass(frozen=True)
class RunConfig:
    """Immutable input for one owned runtime attempt."""

    run_dir: Path
    ros_domain_id: int
    recorder: Sequence[str] | str
    monitor: Sequence[str] | str
    diagnostics: Sequence[str] | str
    launch: Sequence[str] | str
    run_id: Optional[str] = None
    cwd: Optional[Path] = None
    verdict_file: Path | str = Path("verdict.json")
    bag_dir: Optional[Path | str] = None
    bag_inspector: Optional[Sequence[str] | str] = None
    map_saver: Optional[Sequence[str] | str] = None
    map_output_prefix: Optional[Path | str] = None
    map_save_timeout_sec: float = DEFAULT_MAP_SAVE_TIMEOUT_SEC
    graceful_shutdown_sec: float = DEFAULT_GRACEFUL_SHUTDOWN_SEC
    escalation_sec: float = DEFAULT_ESCALATION_SEC
    poll_interval_sec: float = DEFAULT_POLL_INTERVAL_SEC
    max_runtime_sec: Optional[float] = None
    environment: Optional[Mapping[str, str]] = None

    def normalized(self) -> "NormalizedRunConfig":
        run_dir = Path(self.run_dir)
        run_id = validate_run_identity(run_dir, self.run_id)
        domain = validate_ros_domain_id(self.ros_domain_id)
        if self.cwd is None:
            cwd = Path.cwd()
        else:
            cwd = Path(self.cwd)
        if not cwd.is_absolute() or not cwd.is_dir():
            raise RunnerError("runner cwd must be an existing absolute directory")
        for field_name in (
                "graceful_shutdown_sec", "escalation_sec", "poll_interval_sec",
                "map_save_timeout_sec"):
            value = float(getattr(self, field_name))
            if not value > 0.0:
                raise RunnerError(f"{field_name} must be positive")
        max_runtime = None
        if self.max_runtime_sec is not None:
            max_runtime = float(self.max_runtime_sec)
            if not max_runtime > 0.0:
                raise RunnerError("max_runtime_sec must be positive")

        verdict = Path(self.verdict_file)
        if not verdict.is_absolute():
            verdict = run_dir / verdict
        verdict = _path_inside(run_dir, verdict, "verdict file")

        bag_dir = None
        if self.bag_dir is not None:
            bag_dir = Path(self.bag_dir)
            if not bag_dir.is_absolute():
                bag_dir = run_dir / bag_dir
            bag_dir = _path_inside(run_dir, bag_dir, "bag directory")

        environment = dict(os.environ)
        environment.update(self.environment or {})
        if "ROS_DOMAIN_ID" in environment:
            validate_ros_domain_id(environment["ROS_DOMAIN_ID"])
        environment["ROS_DOMAIN_ID"] = str(domain)
        environment["AMR_RUN_ID"] = run_id
        expected_partition = f"amr_{run_id}"
        configured_partition = environment.get("GZ_PARTITION", expected_partition)
        if configured_partition != expected_partition:
            raise RunnerError("GZ_PARTITION must be derived from the unique run identity")
        environment["GZ_PARTITION"] = expected_partition
        environment["ROS_LOG_DIR"] = str(run_dir / "ros_logs")

        bag_inspector = None
        if self.bag_inspector is not None:
            bag_inspector = _as_command(self.bag_inspector, "bag inspector")

        map_saver = None
        map_output_prefix = None
        if self.map_saver is not None:
            map_saver = _as_command(self.map_saver, "map saver")
            map_output_prefix = (
                Path(self.map_output_prefix)
                if self.map_output_prefix is not None
                else run_dir / "evidence" / "aws_map")
            if not map_output_prefix.is_absolute():
                map_output_prefix = run_dir / map_output_prefix
            map_output_prefix = _path_inside(
                run_dir, map_output_prefix, "map output prefix")

        commands = {
            name: _as_command(getattr(self, name), name)
            for name in COMMAND_NAMES
        }
        return NormalizedRunConfig(
            run_dir=run_dir,
            run_id=run_id,
            ros_domain_id=domain,
            cwd=cwd,
            verdict_file=verdict,
            bag_dir=bag_dir,
            bag_inspector=bag_inspector,
            map_saver=map_saver,
            map_output_prefix=map_output_prefix,
            map_save_timeout_sec=float(self.map_save_timeout_sec),
            commands=commands,
            environment=environment,
            graceful_shutdown_sec=float(self.graceful_shutdown_sec),
            escalation_sec=float(self.escalation_sec),
            poll_interval_sec=float(self.poll_interval_sec),
            max_runtime_sec=max_runtime,
        )


@dataclasses.dataclass(frozen=True)
class NormalizedRunConfig:
    run_dir: Path
    run_id: str
    ros_domain_id: int
    cwd: Path
    verdict_file: Path
    bag_dir: Optional[Path]
    bag_inspector: Optional[Tuple[str, ...]]
    map_saver: Optional[Tuple[str, ...]]
    map_output_prefix: Optional[Path]
    map_save_timeout_sec: float
    commands: Mapping[str, Tuple[str, ...]]
    environment: Mapping[str, str]
    graceful_shutdown_sec: float
    escalation_sec: float
    poll_interval_sec: float
    max_runtime_sec: Optional[float]


@dataclasses.dataclass
class ChildState:
    name: str
    argv: Tuple[str, ...]
    process: subprocess.Popen[bytes]
    identity: Dict[str, Any]
    log_path: Path
    reader: Optional[threading.Thread] = None
    returncode: Optional[int] = None
    exit_identity: Optional[Dict[str, Any]] = None
    exited_at: Optional[float] = None

    def as_json(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "argv": list(self.argv),
            "pid": self.identity.get("pid"),
            "pgid": self.identity.get("pgid"),
            "sid": self.identity.get("sid"),
            "start_ticks": self.identity.get("start_ticks"),
            "started_at": self.identity.get("observed_at"),
            "returncode": self.returncode,
            "exited_at": self.exited_at,
            "exit_identity": self.exit_identity,
            "log": str(self.log_path),
        }


class ArtifactStore:
    """Durable JSON/JSONL evidence writer for one run."""

    def __init__(self, run_dir: Path):
        self.run_dir = run_dir
        self.events_path = run_dir / "events.jsonl"
        self.processes_path = run_dir / "processes.json"
        self._lock = threading.Lock()

    def event(self, event: str, **fields: Any) -> None:
        record = {
            "wall_time": time.time(),
            "monotonic": time.monotonic(),
            "event": event,
            **fields,
        }
        with self._lock:
            with self.events_path.open("a", encoding="utf-8") as stream:
                json.dump(record, stream, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())

    def json(self, path: Path, value: Any) -> None:
        temporary = path.with_name(f".{path.name}.tmp")
        with self._lock:
            with temporary.open("w", encoding="utf-8") as stream:
                json.dump(value, stream, indent=2, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)


class BagInspectionGate:
    """Guard all post-run bag commands against a live SQLite file."""

    def __init__(self) -> None:
        self._stopped = False
        self._recorder_exited = False
        self._owned_processes_exited = False

    @property
    def allowed(self) -> bool:
        return self._stopped and self._recorder_exited and self._owned_processes_exited

    def mark_stopped(self, *, recorder_exited: bool, owned_processes_exited: bool) -> None:
        self._stopped = True
        self._recorder_exited = bool(recorder_exited)
        self._owned_processes_exited = bool(owned_processes_exited)

    def inspect(
        self,
        command: Sequence[str],
        *,
        cwd: Path,
        environment: Mapping[str, str],
        output_path: Path,
    ) -> subprocess.CompletedProcess[bytes]:
        if not self.allowed:
            raise LiveBagInspectionError(
                "bag inspection is forbidden until the recorder and all owned processes exit")
        with output_path.open("wb") as output:
            return subprocess.run(
                tuple(command),
                cwd=str(cwd),
                env=dict(environment),
                stdin=subprocess.DEVNULL,
                stdout=output,
                stderr=subprocess.STDOUT,
                check=False,
            )


def _proc_stat(pid: int) -> Optional[Dict[str, int]]:
    """Read the Linux process identity fields needed for ownership checks."""

    try:
        text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, OSError):
        return None
    close = text.rfind(")")
    if close < 0:
        return None
    fields = text[close + 2 :].split()
    try:
        return {
            "ppid": int(fields[1]),
            "pgid": int(fields[2]),
            "sid": int(fields[3]),
            "start_ticks": int(fields[19]),
        }
    except (IndexError, ValueError):
        return None


def _process_identity(pid: int) -> Dict[str, Any]:
    info = _proc_stat(pid) or {}
    return {
        "pid": pid,
        "pgid": info.get("pgid"),
        "sid": info.get("sid"),
        "start_ticks": info.get("start_ticks"),
        "observed_at": time.time(),
    }


def _session_processes(session_id: int, *, exclude: Iterable[int] = ()) -> List[Dict[str, int]]:
    excluded = set(exclude)
    members: List[Dict[str, int]] = []
    proc_root = Path("/proc")
    try:
        entries = tuple(proc_root.iterdir())
    except OSError:
        return members
    for entry in entries:
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in excluded:
            continue
        info = _proc_stat(pid)
        if info is not None and info["sid"] == session_id:
            members.append({"pid": pid, **info})
    return sorted(members, key=lambda item: item["pid"], reverse=True)


def _truth_value(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if value in (0, 1):
            return bool(value)
        return None
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "yes", "1"):
            return True
        if normalized in ("false", "no", "0"):
            return False
    return None


def terminal_motion_proof(evidence: Mapping[str, Any]) -> Tuple[bool, str]:
    """Prove that terminal evidence owns no active or cancellation motion."""

    missing = [
        key for key in (
            "active", "pending", "cancel_owned_motion", "motion_stopped",
            "fault_latched", "cancel_target")
        if key not in evidence
    ]
    if missing:
        return False, f"terminal evidence omitted: {', '.join(missing)}"
    for key in ("active", "pending", "cancel_owned_motion", "motion_stopped"):
        value = _truth_value(evidence[key])
        if value is None:
            return False, f"terminal evidence has malformed {key}"
        if key == "motion_stopped" and not value:
            return False, "terminal evidence does not prove motion is stopped"
        if key != "motion_stopped" and value:
            return False, f"terminal evidence still owns {key} motion"
    fault_latched = _truth_value(evidence["fault_latched"])
    if fault_latched is None:
        return False, "terminal evidence has malformed fault_latched"
    state = str(evidence.get("state", evidence.get("terminal_state", ""))).upper()
    if state in ("COMPLETE", "INCOMPLETE") and fault_latched:
        return False, "non-fault terminal evidence has a latched fault"
    target = evidence["cancel_target"]
    if target not in (None, "", [], (), {}):
        return False, "terminal evidence has a non-empty cancel_target"
    return True, "motion is stopped with no active, pending, or cancel-owned motion"


def _count_value(value: Any) -> Optional[int]:
    if isinstance(value, bool):
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if parsed < 0:
        return None
    if isinstance(value, str) and value.strip() != str(parsed):
        return None
    return parsed


def terminal_structure_proof(
    evidence: Mapping[str, Any], state: str) -> Tuple[bool, str]:
    """Prove that terminal counts and mission status agree with Explorer state."""

    missing = [
        key for key in (
            *TERMINAL_COUNT_FIELDS,
            "completion_policy",
            "mission_goal_uuid", "mission_stage", "mission_outcome",
            "mission_reason", "mission_fault_class", "reason")
        if key not in evidence
    ]
    if missing:
        return False, f"terminal evidence omitted: {', '.join(missing)}"
    counts = {key: _count_value(evidence[key]) for key in TERMINAL_COUNT_FIELDS}
    malformed = [key for key, value in counts.items() if value is None]
    if malformed:
        return False, f"terminal evidence has malformed counts: {', '.join(malformed)}"
    if counts["run_generation"] <= 0:
        return False, "terminal evidence has no positive run_generation"
    if (str(evidence["completion_policy"]).strip()
            != SAFE_REACHABLE_COMPLETION_POLICY):
        return False, "terminal evidence has an unsupported completion policy"
    if (counts["blocked_frontier_count"]
            != counts["blocked_safety_count"] + counts["blocked_route_count"]):
        return False, "terminal frontier block counts are inconsistent"
    if (counts["blocked_frontier_count"]
            + counts["unresolved_frontier_count"]
            > counts["raw_frontier_count"]):
        return False, "terminal frontier counts exceed raw frontier count"
    if (state == "COMPLETE"
            and (counts["blocked_frontier_count"]
                 + counts["unresolved_frontier_count"]
                 != counts["raw_frontier_count"]
                 or counts["unresolved_frontier_count"] != 0)):
        return False, "COMPLETE evidence does not classify every raw frontier"
    if str(evidence["mission_stage"]).strip() != "TERMINAL":
        return False, "terminal evidence mission_stage is not TERMINAL"
    expected_outcome = MISSION_OUTCOME_BY_STATE[state]
    if str(evidence["mission_outcome"]).strip() != expected_outcome:
        return (
            False,
            "terminal evidence mission_outcome does not match state")
    reason = str(evidence["reason"]).strip()
    mission_reason = str(evidence["mission_reason"]).strip()
    if not reason or not mission_reason:
        return False, "terminal evidence has a blank terminal explanation"
    if not str(evidence["mission_fault_class"]).strip():
        return False, "terminal evidence has a blank mission_fault_class"
    if state == "INCOMPLETE":
        acknowledgement = str(
            evidence.get("incomplete_acknowledgement", "")).strip()
        explanation = str(
            evidence.get("incomplete_explanation", reason)).strip()
        if acknowledgement != "ACCEPT_INCOMPLETE" and not reason.startswith(
                "exploration incomplete:"):
            return (
                False,
                "INCOMPLETE terminal evidence lacks an explicit explanation")
        if not explanation:
            return False, "INCOMPLETE terminal evidence has a blank explanation"
    return True, "terminal counts and mission status agree with Explorer state"


def classify_terminal_evidence(evidence: Mapping[str, Any]) -> Dict[str, Any]:
    """Return a fail-closed classification for an observer terminal record."""

    raw_state = evidence.get("state", evidence.get("terminal_state"))
    state = str(raw_state).strip().upper() if raw_state is not None else ""
    if state not in TERMINAL_STATES:
        return {
            "classification": RunClassification.OBSERVER_FAILURE.value,
            "pass": False,
            "reason": "terminal evidence has an unknown state",
        }
    safe, proof = terminal_motion_proof(evidence)
    if not safe:
        return {
            "classification": RunClassification.FAULT.value,
            "pass": False,
            "reason": proof,
        }
    structured, structure_proof = terminal_structure_proof(evidence, state)
    if not structured:
        return {
            "classification": RunClassification.FAULT.value,
            "pass": False,
            "reason": structure_proof,
        }
    if state == RunClassification.FAULT.value:
        return {
            "classification": RunClassification.FAULT.value,
            "pass": False,
            "reason": "structured Explorer terminal state is FAULT",
        }
    return {
        "classification": state,
        "pass": True,
        "reason": f"{proof}; {structure_proof}",
    }


def verify_saved_map(output_prefix: Path | str, run_dir: Path | str) -> Dict[str, Any]:
    """Verify a Nav2 map-saver YAML and its referenced image stay run-local."""

    run_root = Path(run_dir).resolve()
    prefix = Path(output_prefix).resolve()
    try:
        prefix.relative_to(run_root)
    except ValueError as error:
        raise RunnerError("saved map prefix must be inside the run directory") from error

    yaml_path = Path(f"{prefix}.yaml")
    if not yaml_path.is_file() or yaml_path.stat().st_size <= 0:
        raise RunnerError("saved map YAML is missing or empty")
    try:
        yaml_text = yaml_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise RunnerError(f"saved map YAML cannot be read: {error}") from error

    fields: Dict[str, str] = {}
    for raw_line in yaml_text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        if key in {"image", "resolution", "origin", "negate",
                   "occupied_thresh", "free_thresh"}:
            fields[key] = value.strip().strip("'\"")
    required = ("image", "resolution", "origin", "negate",
                "occupied_thresh", "free_thresh")
    missing = [key for key in required if not fields.get(key)]
    if missing:
        raise RunnerError(
            "saved map YAML omitted required fields: " + ", ".join(missing))
    try:
        float(fields["resolution"])
        float(fields["occupied_thresh"])
        float(fields["free_thresh"])
    except ValueError as error:
        raise RunnerError("saved map YAML has malformed numeric fields") from error

    image_value = fields["image"]
    image_path = Path(image_value)
    image_path = image_path if image_path.is_absolute() else yaml_path.parent / image_path
    image_path = image_path.resolve()
    try:
        image_path.relative_to(run_root)
    except ValueError as error:
        raise RunnerError("saved map image must be inside the run directory") from error
    if not image_path.is_file() or image_path.stat().st_size <= 0:
        raise RunnerError("saved map image is missing or empty")
    return {
        "yaml": str(yaml_path),
        "image": str(image_path),
        "yaml_size": yaml_path.stat().st_size,
        "image_size": image_path.stat().st_size,
    }


class _SessionRunner:
    def __init__(self, config: NormalizedRunConfig):
        self.config = config
        self.store = ArtifactStore(config.run_dir)
        self.session_id = os.getsid(0)
        self.session_pgid = os.getpgrp()
        self.children: Dict[str, ChildState] = {}
        self.first_failure: Optional[Dict[str, Any]] = None
        self.terminal: Optional[Dict[str, Any]] = None
        self.terminal_decision: Optional[Dict[str, Any]] = None
        self.terminal_evaluated = False
        self.deferred_abort_logs: List[Dict[str, Any]] = []
        self.map_save: Optional[Dict[str, Any]] = None
        self.shutdown_initiated = False
        self.shutdown_reason = ""
        self.cleanup_escalated = False
        self.cleanup_survivors: List[Dict[str, int]] = []
        self.stop_requested = False
        self.pending_signal: Optional[str] = None
        self.started_monotonic = time.monotonic()
        self._lock = threading.Lock()
        self.bag_gate = BagInspectionGate()

    def _write_processes(self) -> None:
        self.store.json(
            self.store.processes_path,
            {
                "session_id": self.session_id,
                "session_pgid": self.session_pgid,
                "children": {
                    name: child.as_json() for name, child in self.children.items()
                },
            },
        )

    def record_failure(self, classification: RunClassification, *, component: str, reason: str, **extra: Any) -> None:
        failure = {
            "classification": classification.value,
            "component": component,
            "reason": reason,
            "wall_time": time.time(),
            **extra,
        }
        with self._lock:
            if self.first_failure is not None:
                return
            self.first_failure = failure
        self.store.event("first_failure", **failure)

    def _signal_handler(self, signum: int, _frame: Any) -> None:
        if not self.stop_requested and not self.shutdown_initiated:
            self.stop_requested = True
            self.pending_signal = signal.Signals(signum).name

    def _start_child(self, name: str) -> None:
        argv = self.config.commands[name]
        log_path = self.config.run_dir / f"{name}.log"
        try:
            process = subprocess.Popen(
                argv,
                cwd=str(self.config.cwd),
                env=dict(self.config.environment),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                close_fds=True,
                start_new_session=False,
                bufsize=0,
            )
        except (OSError, ValueError) as error:
            classification = (
                RunClassification.RECORDER_FAILURE
                if name == "recorder"
                else RunClassification.OBSERVER_FAILURE
                if name in ("monitor", "diagnostics")
                else RunClassification.LAUNCH_EXIT
            )
            self.record_failure(
                classification,
                component=name,
                reason=f"could not start command: {error}",
            )
            return
        child = ChildState(
            name=name,
            argv=argv,
            process=process,
            identity=_process_identity(process.pid),
            log_path=log_path,
        )
        self.children[name] = child
        self.store.event(
            "process_started",
            name=name,
            pid=child.identity.get("pid"),
            pgid=child.identity.get("pgid"),
            sid=child.identity.get("sid"),
            start_ticks=child.identity.get("start_ticks"),
            argv=list(argv),
        )
        child.reader = threading.Thread(
            target=self._read_output,
            args=(child,),
            name=f"runner-{name}-output",
            daemon=True,
        )
        child.reader.start()

    def _read_output(self, child: ChildState) -> None:
        stream = child.process.stdout
        if stream is None:
            return
        with child.log_path.open("wb") as output:
            for raw_line in iter(stream.readline, b""):
                output.write(raw_line)
                output.flush()
                line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
                for pattern in ABORT_LOG_PATTERNS:
                    if pattern.search(line):
                        match = {
                            "component": child.name,
                            "line": line,
                            "log": str(child.log_path),
                            "wall_time": time.time(),
                        }
                        with self._lock:
                            self.deferred_abort_logs.append(match)
                        self.store.event("abort_log_observed", **match)
                        break

    def _record_exit(self, child: ChildState) -> None:
        if child.returncode is not None:
            return
        returncode = child.process.poll()
        if returncode is None:
            return
        child.returncode = int(returncode)
        child.exited_at = time.time()
        current = _proc_stat(child.process.pid)
        child.exit_identity = {
            "pid": child.process.pid,
            "start_ticks": child.identity.get("start_ticks"),
            "pid_reused": bool(
                current is not None
                and child.identity.get("start_ticks") is not None
                and current.get("start_ticks") != child.identity.get("start_ticks")
            ),
            "returncode": int(returncode),
        }
        self.store.event(
            "process_exited",
            name=child.name,
            pid=child.process.pid,
            pgid=child.identity.get("pgid"),
            sid=child.identity.get("sid"),
            start_ticks=child.identity.get("start_ticks"),
            returncode=int(returncode),
            exit_identity=child.exit_identity,
            expected=self.shutdown_initiated,
        )
        if self.shutdown_initiated or self.terminal is not None:
            return
        classification = (
            RunClassification.RECORDER_FAILURE
            if child.name == "recorder"
            else RunClassification.OBSERVER_FAILURE
            if child.name in ("monitor", "diagnostics")
            else RunClassification.LAUNCH_EXIT
        )
        self.record_failure(
            classification,
            component=child.name,
            reason="required process exited before terminal evidence",
            returncode=int(returncode),
        )

    def _poll_exits(self) -> None:
        changed = False
        for child in tuple(self.children.values()):
            before = child.returncode
            self._record_exit(child)
            changed = changed or before != child.returncode
        if changed:
            self._write_processes()

    def _read_terminal(self) -> None:
        if self.terminal is not None or not self.config.verdict_file.exists():
            return
        try:
            with self.config.verdict_file.open("r", encoding="utf-8") as stream:
                candidate = json.load(stream)
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            return
        if not isinstance(candidate, dict):
            return
        self.terminal = candidate
        self.terminal_decision = classify_terminal_evidence(candidate)
        self.store.event(
            "terminal_evidence",
            evidence=candidate,
            decision=self.terminal_decision,
        )

    def _apply_terminal_evidence(self) -> None:
        """Classify terminal state and deferred logs only after structured proof."""

        if self.terminal is None or self.terminal_decision is None:
            return
        with self._lock:
            abort_logs = list(self.deferred_abort_logs)
        self.store.event(
            "deferred_abort_logs_evaluated",
            count=len(abort_logs),
            terminal_decision=self.terminal_decision,
        )
        if self.terminal_decision.get("classification") == RunClassification.FAULT.value:
            self.record_failure(
                RunClassification.FAULT,
                component="explorer_terminal",
                reason=str(self.terminal_decision.get("reason", "terminal proof failed")),
                terminal=self.terminal,
                abort_logs=abort_logs,
            )
        elif abort_logs:
            self.record_failure(
                RunClassification.FAULT,
                component=abort_logs[0]["component"],
                reason="abort log contradicted structured terminal evidence",
                terminal=self.terminal,
                abort_logs=abort_logs,
            )

    def _save_map_before_shutdown(self) -> None:
        """Save and verify the run-local map while the graph is still alive."""

        record: Dict[str, Any] = {
            "configured": self.config.map_saver is not None,
            "attempted": False,
            "output_prefix": (
                str(self.config.map_output_prefix)
                if self.config.map_output_prefix is not None else None),
        }
        if self.config.map_saver is None:
            record["reason"] = "map saver is not configured"
            self.map_save = record
            self.store.event("map_save_skipped", **record)
            return
        if self.config.map_output_prefix is None:
            record["reason"] = "map saver output prefix is not configured"
            self.map_save = record
            self.record_failure(
                RunClassification.FAULT,
                component="map_saver",
                reason=record["reason"],
            )
            self.store.event("map_save_failed", **record)
            return

        log_path = self.config.run_dir / "map-saver.log"
        record.update({
            "attempted": True,
            "command": list(self.config.map_saver),
            "log": str(log_path),
        })
        process = None
        output = None
        try:
            output = log_path.open("wb")
            process = subprocess.Popen(
                self.config.map_saver,
                cwd=str(self.config.cwd),
                env=dict(self.config.environment),
                stdin=subprocess.DEVNULL,
                stdout=output,
                stderr=subprocess.STDOUT,
                close_fds=True,
                start_new_session=False,
            )
            record["identity"] = _process_identity(process.pid)
            timed_out = False
            try:
                returncode = int(process.wait(timeout=self.config.map_save_timeout_sec))
            except subprocess.TimeoutExpired:
                timed_out = True
                process.kill()
                returncode = int(process.wait())
            record["returncode"] = returncode
            record["timed_out"] = timed_out
            current = _proc_stat(process.pid)
            record["exit_identity"] = {
                "pid": process.pid,
                "start_ticks": record["identity"].get("start_ticks"),
                "pid_reused": bool(
                    current is not None
                    and record["identity"].get("start_ticks") is not None
                    and current.get("start_ticks") != record["identity"].get("start_ticks")
                ),
                "returncode": returncode,
            }
        except (OSError, ValueError) as error:
            record["reason"] = f"could not run map saver: {error}"
            self.map_save = record
            self.record_failure(
                RunClassification.FAULT,
                component="map_saver",
                reason=record["reason"],
                map_save=record,
            )
            self.store.event("map_save_failed", **record)
            return
        finally:
            if output is not None:
                output.close()

        if record["returncode"] != 0:
            record["reason"] = "map saver exited unsuccessfully"
            self.map_save = record
            self.record_failure(
                RunClassification.FAULT,
                component="map_saver",
                reason=record["reason"],
                map_save=record,
            )
            self.store.event("map_save_failed", **record)
            return
        try:
            record["verification"] = verify_saved_map(
                self.config.map_output_prefix, self.config.run_dir)
            record["verified"] = True
        except (OSError, RunnerError) as error:
            record["verified"] = False
            record["reason"] = str(error)
            self.record_failure(
                RunClassification.FAULT,
                component="map_saver",
                reason="saved map verification failed",
                map_save=record,
            )
            self.map_save = record
            self.store.event("map_save_failed", **record)
            return
        self.map_save = record
        self.store.event("map_save_verified", **record)

    def _session_members(self) -> List[Dict[str, int]]:
        return _session_processes(self.session_id, exclude=(os.getpid(),))

    def _signal_session_members(self, signum: signal.Signals) -> None:
        members = self._session_members()
        self.store.event(
            "cleanup_signal",
            signal=signum.name,
            members=members,
        )
        for member in members:
            try:
                os.kill(member["pid"], signum)
            except (ProcessLookupError, PermissionError):
                continue

    def _wait_for_session_empty(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._poll_exits()
            if not self._session_members():
                return True
            time.sleep(min(self.config.poll_interval_sec, 0.05))
        self._poll_exits()
        return not self._session_members()

    def _cleanup(self) -> None:
        if self.shutdown_initiated:
            return
        self.shutdown_initiated = True
        self.store.event("shutdown_requested", reason=self.shutdown_reason)
        self._signal_session_members(signal.SIGINT)
        if self._wait_for_session_empty(self.config.graceful_shutdown_sec):
            return
        self.cleanup_escalated = True
        self.store.event("cleanup_escalation", signal="SIGTERM")
        self._signal_session_members(signal.SIGTERM)
        if self._wait_for_session_empty(self.config.escalation_sec):
            return
        self.store.event("cleanup_escalation", signal="SIGKILL")
        self._signal_session_members(signal.SIGKILL)
        if not self._wait_for_session_empty(self.config.escalation_sec):
            self.cleanup_survivors = self._session_members()
            self.record_failure(
                RunClassification.FAULT,
                component="cleanup",
                reason="owned process session survived SIGKILL escalation",
                survivors=self.cleanup_survivors,
            )

    def _inspect_bag_after_shutdown(self) -> Optional[Dict[str, Any]]:
        recorder = self.children.get("recorder")
        recorder_exited = recorder is not None and recorder.returncode is not None
        owned_exited = not self._session_members()
        self.bag_gate.mark_stopped(
            recorder_exited=recorder_exited,
            owned_processes_exited=owned_exited,
        )
        inspection: Dict[str, Any] = {
            "allowed": self.bag_gate.allowed,
            "bag_dir": str(self.config.bag_dir) if self.config.bag_dir else None,
        }
        if not self.bag_gate.allowed:
            self.record_failure(
                RunClassification.FAULT,
                component="runner",
                reason="owned process session did not finalize before bag checks",
            )
            inspection["reason"] = "recorder or owned process still active"
            return inspection
        if self.config.bag_dir is not None:
            metadata = self.config.bag_dir / "metadata.yaml"
            databases = tuple(self.config.bag_dir.glob("*.db3"))
            try:
                inspection["metadata_exists"] = metadata.is_file() and metadata.stat().st_size > 0
            except OSError:
                inspection["metadata_exists"] = False
            inspection["database_count"] = len(databases)
            if not inspection["metadata_exists"] or not databases:
                self.record_failure(
                    RunClassification.RECORDER_FAILURE,
                    component="recorder",
                    reason="finalized bag is missing metadata or database",
                )
        if self.config.bag_inspector is not None:
            command = tuple(
                item.replace("{bag_dir}", str(self.config.bag_dir or self.config.run_dir))
                for item in self.config.bag_inspector
            )
            output_path = self.config.run_dir / "bag-inspection.log"
            try:
                completed = self.bag_gate.inspect(
                    command,
                    cwd=self.config.cwd,
                    environment=self.config.environment,
                    output_path=output_path,
                )
            except LiveBagInspectionError as error:
                self.record_failure(
                    RunClassification.FAULT,
                    component="runner",
                    reason=str(error),
                )
                inspection["returncode"] = None
            else:
                inspection["returncode"] = int(completed.returncode)
                if completed.returncode != 0:
                    self.record_failure(
                        RunClassification.RECORDER_FAILURE,
                        component="bag_inspector",
                        reason="post-run bag inspection failed",
                        returncode=int(completed.returncode),
                    )
        self.store.event("post_run_bag_checks", **inspection)
        return inspection

    def _join_output_readers(self) -> None:
        for child in self.children.values():
            if child.reader is not None:
                child.reader.join(timeout=self.config.escalation_sec)

    def _classification(self) -> Tuple[str, bool, str]:
        if self.first_failure is not None:
            failure = self.first_failure
            return (
                str(failure["classification"]),
                False,
                str(failure["reason"]),
            )
        if self.terminal_decision is not None:
            decision = self.terminal_decision
            accepted = bool(decision.get("pass"))
            if accepted and self.cleanup_escalated:
                return (
                    RunClassification.FAULT.value,
                    False,
                    "cleanup required signal escalation",
                )
            return (
                str(decision["classification"]),
                accepted,
                str(decision["reason"]),
            )
        return (
            RunClassification.LAUNCH_EXIT.value,
            False,
            "run ended without terminal evidence",
        )

    def _result(self, bag_inspection: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        classification, passed, reason = self._classification()
        result = {
            "run_id": self.config.run_id,
            "ros_domain_id": self.config.ros_domain_id,
            "classification": classification,
            "pass": bool(passed),
            "reason": reason,
            "first_failure": self.first_failure,
            "terminal": self.terminal,
            "session": {
                "sid": self.session_id,
                "pgid": self.session_pgid,
            },
            "cleanup": {
                "shutdown_reason": self.shutdown_reason,
                "escalated": self.cleanup_escalated,
                "owned_processes_exited": not self._session_members(),
                "survivors": self.cleanup_survivors,
            },
            "map_save": self.map_save,
            "bag_inspection": bag_inspection,
            "processes": {
                name: child.as_json() for name, child in self.children.items()
            },
        }
        return result

    def run(self) -> Dict[str, Any]:
        self.store.event(
            "session_started",
            sid=self.session_id,
            pgid=self.session_pgid,
            run_id=self.config.run_id,
            ros_domain_id=self.config.ros_domain_id,
        )
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self.config.run_dir.joinpath("ros_logs").mkdir(parents=True, exist_ok=True)

        for name in COMMAND_NAMES:
            self._start_child(name)
            if self.first_failure is not None:
                break

        self._write_processes()
        while True:
            self._read_terminal()
            self._poll_exits()
            if self.stop_requested and not self.shutdown_initiated:
                self.record_failure(
                    RunClassification.INTERRUPTION,
                    component="runner",
                    reason="received stop signal",
                    signal=self.pending_signal,
                )
                self.shutdown_reason = "interruption"
                self._cleanup()
                break
            if self.terminal is not None and not self.shutdown_initiated:
                if not self.terminal_evaluated:
                    self._apply_terminal_evidence()
                    self.terminal_evaluated = True
                    if (
                        self.first_failure is None
                        and self.terminal_decision is not None
                        and self.terminal_decision.get("pass")
                    ):
                        self._save_map_before_shutdown()
                self.shutdown_reason = (
                    "first failure" if self.first_failure is not None
                    else "terminal evidence")
                self._cleanup()
                break
            if self.first_failure is not None and not self.shutdown_initiated:
                self.shutdown_reason = "first failure"
                self._cleanup()
                break
            if all(child.returncode is not None for child in self.children.values()):
                if self.first_failure is None:
                    self.record_failure(
                        RunClassification.LAUNCH_EXIT,
                        component="runner",
                        reason="all required processes exited without terminal evidence",
                    )
                self.shutdown_reason = "all processes exited"
                self._cleanup()
                break
            if (
                self.config.max_runtime_sec is not None
                and time.monotonic() - self.started_monotonic >= self.config.max_runtime_sec
            ):
                self.record_failure(
                    RunClassification.INTERRUPTION,
                    component="runner",
                    reason="maximum runtime elapsed",
                )
                self.shutdown_reason = "maximum runtime"
                self._cleanup()
                break
            time.sleep(self.config.poll_interval_sec)

        self._poll_exits()
        self._join_output_readers()
        self._apply_terminal_evidence()
        bag_inspection = self._inspect_bag_after_shutdown()
        self._poll_exits()
        self._write_processes()
        result = self._result(bag_inspection)
        self.store.json(self.config.run_dir / "result.json", result)
        self.store.event(
            "run_finished",
            classification=result["classification"],
            passed=result["pass"],
            reason=result["reason"],
        )
        return result


def _create_run_directory(config: RunConfig) -> NormalizedRunConfig:
    normalized = config.normalized()
    normalized.run_dir.parent.mkdir(parents=True, exist_ok=True)
    try:
        normalized.run_dir.mkdir()
    except FileExistsError as error:
        raise RunnerError(f"run directory already exists: {normalized.run_dir}") from error
    normalized.run_dir.joinpath("ros_logs").mkdir()
    normalized.run_dir.joinpath("evidence").mkdir()
    initial = {
        "run_id": normalized.run_id,
        "ros_domain_id": normalized.ros_domain_id,
        "commands": {name: list(command) for name, command in normalized.commands.items()},
        "map_saver": list(normalized.map_saver) if normalized.map_saver else None,
        "map_output_prefix": (
            str(normalized.map_output_prefix)
            if normalized.map_output_prefix is not None else None),
        "cwd": str(normalized.cwd),
        "verdict_file": str(normalized.verdict_file),
        "created_at": time.time(),
    }
    ArtifactStore(normalized.run_dir).json(normalized.run_dir / "run.json", initial)
    return normalized


def run_aws_exploration(config: RunConfig) -> Dict[str, Any]:
    """Run one unique attempt and return its durable result record."""

    normalized = _create_run_directory(config)
    pid = os.fork()
    if pid == 0:
        try:
            os.setsid()
            result = _SessionRunner(normalized).run()
            os._exit(0 if result["pass"] else 1)
        except BaseException as error:  # pragma: no cover - emergency artifact path
            failure = {
                "run_id": normalized.run_id,
                "ros_domain_id": normalized.ros_domain_id,
                "classification": RunClassification.FAULT.value,
                "pass": False,
                "reason": f"runner crashed: {error}",
            }
            try:
                ArtifactStore(normalized.run_dir).json(normalized.run_dir / "result.json", failure)
            finally:
                os._exit(1)

    interrupted = False

    def forward_signal(signum: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        try:
            os.kill(pid, signum)
        except ProcessLookupError:
            pass

    previous_int = signal.signal(signal.SIGINT, forward_signal)
    previous_term = signal.signal(signal.SIGTERM, forward_signal)
    try:
        while True:
            try:
                waited, _status = os.waitpid(pid, 0)
            except InterruptedError:
                continue
            if waited == pid:
                break
    finally:
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    result_path = normalized.run_dir / "result.json"
    if not result_path.is_file():
        result = {
            "run_id": normalized.run_id,
            "ros_domain_id": normalized.ros_domain_id,
            "classification": RunClassification.INTERRUPTION.value if interrupted else RunClassification.FAULT.value,
            "pass": False,
            "reason": "session runner exited without a result record",
        }
        ArtifactStore(normalized.run_dir).json(result_path, result)
    with result_path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--run-id")
    parser.add_argument("--ros-domain-id", required=True, type=int)
    parser.add_argument("--recorder-cmd", required=True, help="recorder argv as one shell-style string")
    parser.add_argument("--monitor-cmd", required=True, help="observer argv as one shell-style string")
    parser.add_argument("--diagnostics-cmd", required=True, help="diagnostic argv as one shell-style string")
    parser.add_argument("--launch-cmd", required=True, help="simulation launch argv as one shell-style string")
    parser.add_argument("--cwd", type=Path, default=Path.cwd())
    parser.add_argument("--verdict-file", type=Path, default=Path("verdict.json"))
    parser.add_argument("--bag-dir", type=Path)
    parser.add_argument("--bag-inspector-cmd", help="post-run bag inspection argv as one shell-style string")
    parser.add_argument("--map-saver-cmd", help="pre-shutdown map saver argv as one shell-style string")
    parser.add_argument("--map-output-prefix", type=Path)
    parser.add_argument("--map-save-timeout-sec", type=float, default=DEFAULT_MAP_SAVE_TIMEOUT_SEC)
    parser.add_argument("--graceful-shutdown-sec", type=float, default=DEFAULT_GRACEFUL_SHUTDOWN_SEC)
    parser.add_argument("--escalation-sec", type=float, default=DEFAULT_ESCALATION_SEC)
    parser.add_argument("--poll-interval-sec", type=float, default=DEFAULT_POLL_INTERVAL_SEC)
    parser.add_argument("--max-runtime-sec", type=float)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    config = RunConfig(
        run_dir=args.run_dir,
        run_id=args.run_id,
        ros_domain_id=args.ros_domain_id,
        recorder=args.recorder_cmd,
        monitor=args.monitor_cmd,
        diagnostics=args.diagnostics_cmd,
        launch=args.launch_cmd,
        cwd=args.cwd,
        verdict_file=args.verdict_file,
        bag_dir=args.bag_dir,
        bag_inspector=args.bag_inspector_cmd,
        map_saver=args.map_saver_cmd,
        map_output_prefix=args.map_output_prefix,
        map_save_timeout_sec=args.map_save_timeout_sec,
        graceful_shutdown_sec=args.graceful_shutdown_sec,
        escalation_sec=args.escalation_sec,
        poll_interval_sec=args.poll_interval_sec,
        max_runtime_sec=args.max_runtime_sec,
    )
    try:
        result = run_aws_exploration(config)
    except RunnerError as error:
        print(f"aws exploration runner rejected run: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get("pass") else 1


if __name__ == "__main__":
    raise SystemExit(main())
