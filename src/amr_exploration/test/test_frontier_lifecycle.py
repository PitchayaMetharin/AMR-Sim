from pathlib import Path
import math
import os
import sys
import threading
import time
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from amr_interfaces.msg import BaseStatus, ManipulatorStatus
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from tf2_ros import (
    StaticTransformBroadcaster, TransformBroadcaster, TransformException)
from rclpy.duration import Duration


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import frontier_explorer as frontier_explorer_module  # noqa: E402
from frontier_explorer import (  # noqa: E402
    FrontierExplorer, _CancellationRecord)
from frontier_algorithm import frontier_cell_world  # noqa: E402


class FakeFuture:
    def __init__(self, result=None):
        self._result = result
        self._done = result is not None
        self._callbacks = []

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:
            callback(self)

    def set_result(self, result):
        self._result = result
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def set_exception(self, exception):
        self._result = exception
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def result(self):
        if isinstance(self._result, BaseException):
            raise self._result
        if not self._done:
            raise RuntimeError("future is pending")
        return self._result


class FakePublisher:
    def __init__(self, event=None):
        self.messages = []
        self.event = event
        self.event_was_set = []

    def publish(self, message):
        self.messages.append(message)
        if self.event is not None:
            self.event_was_set.append(self.event.is_set())


class FakeActionClient:
    def __init__(self):
        self.send_calls = []
        self.pending = []
        self.server_available = True

    def wait_for_server(self, timeout_sec):
        return self.server_available

    def send_goal_async(self, goal):
        self.send_calls.append(goal)
        future = FakeFuture()
        self.pending.append(future)
        return future


class SequencedTransformBuffer:
    def __init__(self, first_transform=None):
        self.first_transform = first_transform
        self.lookup_calls = 0

    def lookup_transform(self, *_args, **_kwargs):
        self.lookup_calls += 1
        if self.lookup_calls == 1 and self.first_transform is not None:
            return self.first_transform
        raise RuntimeError("independent TF lookup must not be attempted")


class QueryTimeTransformBuffer:
    def __init__(self, clock=None, missing=False, events=None):
        self.clock = clock
        self.missing = missing
        self.calls = []
        self.events = events

    def lookup_transform(self, target, source, query_time, **kwargs):
        self.calls.append((target, source, query_time, kwargs))
        if self.events is not None:
            self.events.append("lookup")
        if self.missing:
            raise RuntimeError("TF intentionally unavailable")
        query_ns = int(getattr(query_time, "nanoseconds", 0))
        if query_ns <= 0 and self.clock is not None:
            query_ns = self.clock.nanoseconds
        return _transform(stamp_ns=query_ns - 100_000_000)


class FakeGoalHandle:
    def __init__(self):
        self.accepted = True
        self.cancel_calls = 0
        self.cancel_future = FakeFuture()
        self.result_future = FakeFuture()
        self.cancel_called = threading.Event()
        self.cancel_exception = None

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        if self.cancel_exception is not None:
            raise self.cancel_exception
        self.cancel_calls += 1
        self.cancel_called.set()
        return self.cancel_future


class FakeLogger:
    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class FakeClock:
    def __init__(self, nanoseconds=10_000_000_000):
        self.nanoseconds = nanoseconds

    def now(self):
        stamp = RosTime(sec=self.nanoseconds // 1_000_000_000,
                        nanosec=self.nanoseconds % 1_000_000_000)
        return SimpleNamespace(
            nanoseconds=self.nanoseconds,
            to_msg=lambda: stamp)


class FakeMonotonic:
    def __init__(self, value=100.0):
        self.value = float(value)

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += float(seconds)


def _transform(stamp_ns=9_500_000_000, x=10.0, y=10.0):
    transform = TransformStamped()
    transform.header.stamp.sec = stamp_ns // 1_000_000_000
    transform.header.stamp.nanosec = stamp_ns % 1_000_000_000
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.w = 1.0
    return transform


def _map_message(data=None, width=2, height=1, resolution=1.0,
                 origin_x=0.0, origin_y=0.0, yaw=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        data=(
            [0, -1]
            if data is None and width == 2 and height == 1
            else ([0] * (width * height) if data is None else data)),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0)))))


def _costmap_message(data=None, width=2, height=1, resolution=2.0,
                     origin_x=-1.0, origin_y=-1.0, yaw=0.0,
                     frame_id="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        data=[0] * (width * height) if data is None else data,
        metadata=SimpleNamespace(
            size_x=width,
            size_y=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0)))))


def _set_header_stamp(header, stamp_ns):
    header.stamp.sec = stamp_ns // 1_000_000_000
    header.stamp.nanosec = stamp_ns % 1_000_000_000


def _production_sized_frontier_fixture():
    """Build a production-dimension fixture with recorded-like densities."""
    width = 232
    height = 193
    resolution = 0.05
    origin_x = -1.3106583660854871
    origin_y = -4.834236125860132
    cell_count = width * height

    # The runtime snapshot was 232x193 at 5 cm resolution with 7,886 free
    # map cells.  Isolating those cells keeps the fixture deterministic while
    # retaining the production message size and a long real selector pass.
    free_cells = []
    for cell_y in range(1, height, 2):
        for cell_x in range(1, width, 2):
            if len(free_cells) == 7886:
                break
            free_cells.append((cell_x, cell_y))
        if len(free_cells) == 7886:
            break

    map_data = [-1] * cell_count
    for cell_x, cell_y in free_cells:
        map_data[cell_y * width + cell_x] = 0
    occupied_count = 0
    for cell_y in range(0, height, 2):
        for cell_x in range(0, width, 2):
            index = cell_y * width + cell_x
            if map_data[index] == -1:
                map_data[index] = 100
                occupied_count += 1
                if occupied_count == 158:
                    break
        if occupied_count == 158:
            break

    map_message = OccupancyGrid()
    map_message.header.frame_id = "map"
    map_message.info.width = width
    map_message.info.height = height
    map_message.info.resolution = resolution
    map_message.info.origin.position.x = origin_x
    map_message.info.origin.position.y = origin_y
    map_message.info.origin.orientation.w = 1.0
    map_message.data = map_data

    # Keep all known-free endpoint cells lethal, as in the failed runtime
    # candidate evidence, while matching the recorded high-cost density.
    costmap_data = [0] * cell_count
    lethal_count = 0
    for cell_x, cell_y in free_cells:
        costmap_data[cell_y * width + cell_x] = 253
        lethal_count += 1
    for index in range(cell_count):
        if lethal_count == 11565:
            break
        if costmap_data[index] == 0:
            costmap_data[index] = 253
            lethal_count += 1
    for value, count in ((254, 893), (255, 2136)):
        assigned = 0
        for index in range(cell_count):
            if costmap_data[index] == 0:
                costmap_data[index] = value
                assigned += 1
                if assigned == count:
                    break

    costmap_message = Costmap()
    costmap_message.header.frame_id = "map"
    costmap_message.metadata.size_x = width
    costmap_message.metadata.size_y = height
    costmap_message.metadata.resolution = resolution
    costmap_message.metadata.origin.position.x = origin_x
    costmap_message.metadata.origin.position.y = origin_y
    costmap_message.metadata.origin.orientation.w = 1.0
    costmap_message.data = costmap_data
    return map_message, costmap_message


def _run_live_tf_planning_diagnostic(executor_workers):
    """Run the same real-selector TF probe with a selected worker count."""
    ros_context = Context()
    domain_id = (os.getpid() % 60) + 120 + executor_workers
    explorer = None
    publisher_node = None
    executor = None
    spin_thread = None
    publisher_thread = None
    lookup_timer = None
    stop_publishers = threading.Event()
    inputs_started = threading.Event()
    planning_started = threading.Event()
    planning_finished = threading.Event()
    progress_wakeup = threading.Event()
    planning_lock = threading.Lock()
    raw_lock = threading.Lock()
    observations_lock = threading.Lock()
    planning_times = {
        "calls": 0,
        "started_at": None,
        "finished_at": None,
        "carried_transform": None,
    }
    raw_state = {
        "sequence": 0,
        "clock_ns": None,
        "map_odom_stamp_ns": None,
        "odom_base_stamp_ns": None,
    }
    observations = []
    publisher_errors = []
    spin_errors = []

    original_frontier_clusters = frontier_explorer_module.frontier_clusters
    original_costmap_candidates = (
        frontier_explorer_module.costmap_frontier_candidates)

    def instrumented_frontier_clusters(*args, **kwargs):
        with planning_lock:
            planning_times["calls"] += 1
            if planning_times["started_at"] is None:
                planning_times["started_at"] = time.monotonic()
        planning_started.set()
        return original_frontier_clusters(*args, **kwargs)

    def instrumented_costmap_candidates(*args, **kwargs):
        try:
            return original_costmap_candidates(*args, **kwargs)
        finally:
            with planning_lock:
                if planning_times["finished_at"] is None:
                    planning_times["finished_at"] = time.monotonic()
            planning_finished.set()

    def spin_executor():
        try:
            executor.spin()
        except Exception as exc:  # pragma: no cover - cleanup diagnostic
            spin_errors.append(exc)

    def wait_until(predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return
            remaining = deadline - time.monotonic()
            progress_wakeup.wait(min(0.02, max(0.0, remaining)))
            progress_wakeup.clear()
        if predicate():
            return
        pytest.fail("timed out waiting for %s" % description)

    def lookup_probe():
        sample_started_at = time.monotonic()
        local_ros_ns = None
        transform = None
        lookup_path = "return"
        exception_type = ""
        exception_message = ""
        try:
            local_ros_ns = int(explorer.get_clock().now().nanoseconds)
            try:
                transform = explorer.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception as exc:  # pragma: no cover - diagnostic path
                lookup_path = "exception"
                exception_type = type(exc).__name__
                exception_message = str(exc)[:160]
        except Exception as exc:  # pragma: no cover - diagnostic path
            lookup_path = "clock_exception"
            exception_type = type(exc).__name__
            exception_message = str(exc)[:160]
        sample_finished_at = time.monotonic()

        result_stamp_ns = None
        if transform is not None:
            stamp = transform.header.stamp
            result_stamp_ns = (
                int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))
        valid_transform = (
            transform is not None
            and local_ros_ns is not None
            and FrontierExplorer._valid_transform(
                transform, local_ros_ns, explorer.tf_timeout))

        with raw_lock:
            raw = dict(raw_state)
        raw_common_stamp_ns = None
        raw_tf_current = False
        if (raw["clock_ns"] is not None
                and raw["map_odom_stamp_ns"] is not None
                and raw["odom_base_stamp_ns"] is not None):
            raw_common_stamp_ns = min(
                raw["map_odom_stamp_ns"], raw["odom_base_stamp_ns"])
            raw_ages = (
                raw["clock_ns"] - raw["map_odom_stamp_ns"],
                raw["clock_ns"] - raw["odom_base_stamp_ns"])
            raw_tf_current = all(0 <= age <= 200_000_000 for age in raw_ages)

        with observations_lock:
            observations.append({
                "sample_started_at": sample_started_at,
                "sample_finished_at": sample_finished_at,
                "local_ros_ns": local_ros_ns,
                "result_stamp_ns": result_stamp_ns,
                "lookup_path": lookup_path,
                "exception_type": exception_type,
                "exception_message": exception_message,
                "lookup_duration_sec": (
                    sample_finished_at - sample_started_at),
                "valid_transform": valid_transform,
                "local_age_sec": (
                    (local_ros_ns - result_stamp_ns) / 1_000_000_000.0
                    if local_ros_ns is not None
                    and result_stamp_ns is not None else None),
                "raw_sequence": raw["sequence"],
                "raw_clock_ns": raw["clock_ns"],
                "raw_common_stamp_ns": raw_common_stamp_ns,
                "raw_tf_current": raw_tf_current,
            })
        progress_wakeup.set()

    try:
        frontier_explorer_module.frontier_clusters = (
            instrumented_frontier_clusters)
        frontier_explorer_module.costmap_frontier_candidates = (
            instrumented_costmap_candidates)
        rclpy.init(context=ros_context, domain_id=domain_id)
        explorer = FrontierExplorer(context=ros_context)
        original_reserve_and_send = explorer._reserve_and_send

        def instrumented_reserve_and_send(*args, **kwargs):
            with planning_lock:
                planning_times["carried_transform"] = kwargs.get("transform")
            return original_reserve_and_send(*args, **kwargs)

        explorer._reserve_and_send = instrumented_reserve_and_send
        parameter_result = explorer.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        assert parameter_result[0].successful
        assert isinstance(
            explorer.planning_callback_group, MutuallyExclusiveCallbackGroup)

        publisher_node = Node(
            "frontier_tf_publisher_%s_%s" % (os.getpid(), executor_workers),
            context=ros_context)
        clock_qos = QoSProfile(depth=1)
        clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        clock_qos.durability = DurabilityPolicy.VOLATILE
        clock_publisher = publisher_node.create_publisher(
            Clock, "/clock", clock_qos)
        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_qos.durability = DurabilityPolicy.VOLATILE
        tf_broadcaster = TransformBroadcaster(publisher_node, qos=tf_qos)
        static_qos = QoSProfile(depth=1)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        static_broadcaster = StaticTransformBroadcaster(
            publisher_node, qos=static_qos)
        base_qos = QoSProfile(depth=10)
        base_qos.reliability = ReliabilityPolicy.RELIABLE
        base_qos.durability = DurabilityPolicy.VOLATILE
        manipulator_qos = QoSProfile(depth=1)
        manipulator_qos.reliability = ReliabilityPolicy.RELIABLE
        manipulator_qos.durability = DurabilityPolicy.VOLATILE
        base_publisher = publisher_node.create_publisher(
            BaseStatus, "/amr/base/status", base_qos)
        manipulator_publisher = publisher_node.create_publisher(
            ManipulatorStatus, "/amr/manipulation/status", manipulator_qos)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_publisher = publisher_node.create_publisher(
            OccupancyGrid, "/map", map_qos)
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        costmap_publisher = publisher_node.create_publisher(
            Costmap, "/amr/global_costmap/costmap_raw", costmap_qos)

        map_message, costmap_message = _production_sized_frontier_fixture()
        assert (map_message.info.width, map_message.info.height) == (232, 193)
        assert (costmap_message.metadata.size_x,
                costmap_message.metadata.size_y) == (232, 193)
        static_transform = TransformStamped()
        static_transform.header.frame_id = "base_footprint"
        static_transform.child_frame_id = "base_link"
        static_transform.transform.rotation.w = 1.0
        static_broadcaster.sendTransform(static_transform)

        def publish_inputs():
            sequence = 0
            try:
                while not stop_publishers.is_set():
                    clock_ns = 100_000_000_000 + sequence * 20_000_000
                    map_odom_stamp_ns = clock_ns - 80_000_000
                    odom_base_stamp_ns = clock_ns - 13_000_000
                    clock_message = Clock()
                    clock_message.clock.sec = clock_ns // 1_000_000_000
                    clock_message.clock.nanosec = (
                        clock_ns % 1_000_000_000)
                    map_to_odom = TransformStamped()
                    map_to_odom.header.frame_id = "map"
                    map_to_odom.child_frame_id = "odom"
                    _set_header_stamp(
                        map_to_odom.header, map_odom_stamp_ns)
                    map_to_odom.transform.rotation.w = 1.0
                    odom_to_base = TransformStamped()
                    odom_to_base.header.frame_id = "odom"
                    odom_to_base.child_frame_id = "base_footprint"
                    _set_header_stamp(
                        odom_to_base.header, odom_base_stamp_ns)
                    odom_to_base.transform.rotation.w = 1.0
                    base_message = BaseStatus()
                    base_message.valid = True
                    base_message.state = BaseStatus.READY
                    base_message.sequence = sequence
                    manipulator_message = ManipulatorStatus()
                    manipulator_message.valid = True
                    manipulator_message.state = (
                        ManipulatorStatus.STOWED_EMPTY)
                    manipulator_message.base_motion_allowed = True
                    manipulator_message.sequence = sequence

                    clock_publisher.publish(clock_message)
                    tf_broadcaster.sendTransform(
                        [map_to_odom, odom_to_base])
                    if sequence < 10:
                        _set_header_stamp(map_message.header, clock_ns)
                        _set_header_stamp(costmap_message.header, clock_ns)
                        map_publisher.publish(map_message)
                        costmap_publisher.publish(costmap_message)
                    base_publisher.publish(base_message)
                    manipulator_publisher.publish(manipulator_message)
                    with raw_lock:
                        raw_state["sequence"] = sequence + 1
                        raw_state["clock_ns"] = clock_ns
                        raw_state["map_odom_stamp_ns"] = map_odom_stamp_ns
                        raw_state["odom_base_stamp_ns"] = odom_base_stamp_ns
                    inputs_started.set()
                    progress_wakeup.set()
                    sequence += 1
                    stop_publishers.wait(0.005)
            except Exception as exc:  # pragma: no cover - cleanup diagnostic
                publisher_errors.append(exc)

        executor = MultiThreadedExecutor(
            num_threads=executor_workers, context=ros_context)
        executor.add_node(explorer)
        executor.add_node(publisher_node)
        lookup_timer = explorer.create_timer(
            0.01, lookup_probe,
            callback_group=MutuallyExclusiveCallbackGroup())
        spin_thread = threading.Thread(
            target=spin_executor, name="frontier-diagnostic-executor", daemon=True)
        spin_thread.start()
        publisher_thread = threading.Thread(
            target=publish_inputs, name="frontier-diagnostic-publisher", daemon=True)
        publisher_thread.start()

        def initial_evidence_ready():
            with observations_lock:
                local_tf_ready = any(
                    observation["lookup_path"] == "return"
                    and observation["valid_transform"]
                    and observation["local_ros_ns"] > 0
                    for observation in observations)
            return (
                inputs_started.is_set()
                and explorer._map_readiness()[0]
                and explorer._costmap_readiness()[0]
                and explorer._authority_ready()
                and explorer.get_clock().now().nanoseconds > 0
                and local_tf_ready)

        wait_until(
            initial_evidence_ready, 8.0,
            "initial map, costmap, authority, clock, and TF evidence")
        with raw_lock:
            baseline_sequence = raw_state["sequence"]
        with explorer._lock:
            explorer.state = "SCANNING"
            explorer.reason = "diagnostic planning run"
            explorer.started_at = time.monotonic()
            explorer.processed_map_version = -1

        wait_until(
            planning_finished.is_set, 12.0,
            "real _tick to _select_frontier planning pass")
        wait_until(
            lambda: explorer.state != "PLANNING", 3.0,
            "real selector to leave PLANNING")
        with planning_lock:
            planning_call_count = planning_times["calls"]
            planning_started_at = planning_times["started_at"]
            planning_finished_at = planning_times["finished_at"]
        assert planning_call_count == 1
        assert planning_started_at is not None
        assert planning_finished_at is not None
        planning_duration = planning_finished_at - planning_started_at
        assert planning_duration < explorer.tf_timeout
        with planning_lock:
            carried_transform = planning_times["carried_transform"]
        assert carried_transform is not None
        planning_finish_ros_ns = explorer.get_clock().now().nanoseconds
        carried_valid_at_finish = FrontierExplorer._valid_transform(
            carried_transform, planning_finish_ros_ns, explorer.tf_timeout)
        assert carried_valid_at_finish

        with raw_lock:
            final_raw = dict(raw_state)
        assert final_raw["sequence"] > baseline_sequence
        assert (
            final_raw["clock_ns"] - final_raw["map_odom_stamp_ns"]
            == 80_000_000)
        assert (
            final_raw["clock_ns"] - final_raw["odom_base_stamp_ns"]
            == 13_000_000)
        with observations_lock:
            captured = list(observations)
        during_planning = [
            observation for observation in captured
            if planning_started_at <= observation["sample_started_at"]
            <= planning_finished_at]
        assert during_planning, "no Explorer-local lookup was captured during planning"
        assert all(
            math.isfinite(observation["lookup_duration_sec"])
            and observation["lookup_duration_sec"] >= 0.0
            for observation in during_planning)
        assert all(
            observation["raw_tf_current"] for observation in during_planning)
        if executor_workers == 3:
            assert any(
                observation["lookup_path"] == "return"
                and observation["valid_transform"]
                for observation in during_planning)

        future_samples = [
            observation for observation in during_planning
            if (observation["result_stamp_ns"] is not None
                and observation["local_ros_ns"] is not None
                and observation["result_stamp_ns"]
                > observation["local_ros_ns"])]
        stale_or_exception_samples = [
            observation for observation in during_planning
            if observation["lookup_path"] != "return"
            or not observation["valid_transform"]]
        if future_samples and stale_or_exception_samples:
            interpretation = (
                "future stamp observed; stale/exception samples also occurred")
        elif future_samples:
            interpretation = (
                "future stamp observed: lookup API race candidate")
        elif any(observation["lookup_path"] != "return"
                 for observation in stale_or_exception_samples):
            interpretation = (
                "local lookup exception while planning: executor capacity "
                "candidate")
        elif stale_or_exception_samples:
            interpretation = "returned local TF sample failed freshness"
        else:
            interpretation = "all local TF samples passed freshness"
        path_counts = {}
        for observation in during_planning:
            path = observation["lookup_path"]
            path_counts[path] = path_counts.get(path, 0) + 1
        local_ages = [
            observation["local_age_sec"] for observation in during_planning
            if observation["local_age_sec"] is not None]
        print(
            "[TF-DIAG] workers=%s planning=%.3fs samples=%s paths=%s "
            "future=%s stale_or_exception=%s local_age_max=%.3fs "
            "carried_valid_at_finish=%s interpretation=%s" % (
                executor_workers, planning_duration, len(during_planning),
                path_counts, len(future_samples), len(stale_or_exception_samples),
                max(local_ages) if local_ages else float("nan"),
                carried_valid_at_finish,
                interpretation))
    finally:
        stop_publishers.set()
        if lookup_timer is not None:
            lookup_timer.cancel()
        if explorer is not None:
            explorer.timer.cancel()
        if publisher_thread is not None:
            publisher_thread.join(3.0)
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if spin_thread is not None:
            spin_thread.join(3.0)
        if explorer is not None:
            explorer.destroy_node()
        if publisher_node is not None:
            publisher_node.destroy_node()
        if ros_context.ok():
            rclpy.shutdown(context=ros_context)
        frontier_explorer_module.frontier_clusters = original_frontier_clusters
        frontier_explorer_module.costmap_frontier_candidates = (
            original_costmap_candidates)
        assert not publisher_errors, (
            "publisher thread failed: %s" % publisher_errors)
        assert not spin_errors, "executor thread failed: %s" % spin_errors


def _node(autostart=False):
    node = FrontierExplorer.__new__(FrontierExplorer)
    node._lock = threading.RLock()
    node.status_pub = FakePublisher()
    node.get_clock = lambda: FakeClock()
    node.get_logger = lambda: FakeLogger()
    node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args, **kwargs: _transform())
    node.action_client = FakeActionClient()
    node.map_timeout = 3.0
    node.tf_timeout = 1.0
    node.no_frontier_limit = 3
    node.goal_timeout = 120.0
    node.max_goal_failures = 3
    node.authority_timeout = 1.0
    node.startup_grace = 15.0
    node.cancel_timeout = 5.0
    node.min_goal_distance = 0.3
    node.autostart = autostart
    node._monotonic = time.monotonic
    node.started_at = time.monotonic()
    node.readiness_wait_started_at = node.started_at if autostart else None
    node.last_map_at = time.monotonic()
    node.latest_map = _map_message()
    node.map_version = 1
    node.processed_map_version = -1
    node.last_costmap_at = time.monotonic()
    node.latest_costmap = _costmap_message()
    node.costmap_version = 1
    node.last_base_status_at = time.monotonic()
    node.last_manipulator_status_at = time.monotonic()
    node.base_status = SimpleNamespace(valid=True, state=BaseStatus.READY)
    node.manipulator_status = SimpleNamespace(
        valid=True,
        base_motion_allowed=True,
        state=ManipulatorStatus.STOWED_EMPTY)
    node.run_generation = 1 if autostart else 0
    node.motion_generation = 0
    node._motion_token = None
    node._motion_owned = False
    node._pending = False
    node._pending_token = None
    node._pending_future = None
    node._pending_candidate = None
    node._attempted_goal_world = None
    node.active_goal = None
    node._active_token = None
    node._result_status = None
    node._result_future = None
    node.goal_started_at = None
    node._motion_deadline = None
    node.cancel_requested = False
    node.cancel_started_at = None
    node.cancel_reason = ""
    node._cancel_target = ""
    node._cancel_invoked = False
    node._cancel_future = None
    node._cancel_ack = False
    node._cancel_record = None
    node.cancel_event = threading.Event()
    node.blacklist = set()
    node.failed_goal_worlds = set()
    node.goal_failures = 0
    node.no_frontier_updates_seen = 0
    node.active_candidate = None
    node.fault_requested = False
    node.fault_latched = False
    node.reason = "autostart waiting for readiness" if autostart else "autostart disabled"
    node.state = "WAITING_READY" if autostart else "STOPPED"
    return node


def _start(node):
    response = Trigger.Response()
    return node._start_callback(None, response)


def _dispatch_pending(node, candidate=(1, 0), goal_world=(1.5, 0.5)):
    node.state = "PLANNING"
    now = node._monotonic()
    node.last_map_at = now
    node.last_costmap_at = now
    node.last_base_status_at = now
    node.last_manipulator_status_at = now
    node._reserve_and_send(
        object(), candidate, node.run_generation,
        map_snapshot=(node.latest_map, node.map_version, node.last_map_at),
        costmap_snapshot=(
            node.latest_costmap, node.costmap_version, node.last_costmap_at),
        transform=_transform(), goal_world=goal_world)
    assert node.state == "GOAL_PENDING"
    assert len(node.action_client.send_calls) == 1


def _accept(node):
    handle = FakeGoalHandle()
    node.action_client.pending[-1].set_result(handle)
    assert node.active_goal is handle
    return handle


def _diagnostic_values(node):
    return {item.key: item.value for item in node.status_pub.messages[-1].status[0].values}


@pytest.mark.parametrize("name,value", [
    ("autostart", 1),
    ("autostart", "true"),
    ("map_timeout_sec", math.nan),
    ("map_timeout_sec", math.inf),
    ("tf_timeout_sec", -math.inf),
    ("goal_timeout_sec", 0.0),
    ("authority_timeout_sec", True),
    ("startup_grace_sec", "15.0"),
    ("cancel_timeout_sec", -1.0),
    ("min_goal_distance_m", None),
    ("no_frontier_updates", 1.0),
    ("no_frontier_updates", False),
    ("max_goal_failures", math.inf),
])
def test_invalid_parameters_are_rejected_before_ros_entities(monkeypatch, name, value):
    defaults = {
        "autostart": True,
        "map_timeout_sec": 3.0,
        "tf_timeout_sec": 1.0,
        "no_frontier_updates": 3,
        "goal_timeout_sec": 120.0,
        "max_goal_failures": 3,
        "authority_timeout_sec": 1.0,
        "startup_grace_sec": 15.0,
        "cancel_timeout_sec": 5.0,
        "min_goal_distance_m": 0.3,
    }
    overrides = dict(defaults)
    overrides[name] = value
    entity_calls = []

    def fake_node_init(node, *_args, **_kwargs):
        node._test_parameters = {}

    def fake_declare_parameter(node, parameter_name, default):
        node._test_parameters[parameter_name] = overrides.get(
            parameter_name, default)

    def fake_get_parameter(node, parameter_name):
        return SimpleNamespace(value=node._test_parameters[parameter_name])

    def record_entity(*_args, **_kwargs):
        entity_calls.append(True)

    monkeypatch.setattr(Node, "__init__", fake_node_init)
    monkeypatch.setattr(Node, "declare_parameter", fake_declare_parameter)
    monkeypatch.setattr(Node, "get_parameter", fake_get_parameter)
    for method in ("create_subscription", "create_publisher", "create_service",
                   "create_timer"):
        monkeypatch.setattr(Node, method, record_entity)

    with pytest.raises(ValueError):
        FrontierExplorer()
    assert entity_calls == []


def _active_cancel(node, reason="operator requested exploration stop"):
    _dispatch_pending(node)
    handle = _accept(node)
    node._begin_cancel(reason)
    assert node._cancel_record is not None
    assert node.state == "CANCELLING"
    return handle, node._cancel_record


def _stop_thread(node):
    completed = threading.Event()
    result = {}

    def run():
        response = Trigger.Response()
        result["response"] = node._stop_callback(None, response)
        completed.set()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, completed, result


def test_autostart_and_start_boundary_reset_state_and_publish_status():
    node = _node(autostart=False)
    assert node.state == "STOPPED"
    response = _start(node)
    assert response.success is True
    assert node.state == "WAITING_READY"
    assert node.run_generation == 1
    assert _diagnostic_values(node)["state"] == "WAITING_READY"
    response = _start(node)
    assert response.success is False


def test_readiness_and_goal_lifecycle_has_one_pending_request():
    node = _node(autostart=True)
    node._tick()
    assert node.state == "SCANNING"
    node._tick()
    assert node.state == "GOAL_PENDING"
    node.latest_map = _map_message()
    node.map_version += 1
    node._tick()
    assert len(node.action_client.send_calls) == 1
    handle = _accept(node)
    assert node.state == "NAVIGATING"
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert node.state == "SCANNING"


def test_initial_persistent_invalid_readiness_faults_after_startup_grace():
    clock = FakeMonotonic(100.0)
    node = _node(autostart=True)
    node._monotonic = clock
    node.started_at = clock()
    node.readiness_wait_started_at = clock()
    node.last_map_at = clock()
    node.last_costmap_at = clock()
    node.last_base_status_at = clock()
    node.last_manipulator_status_at = clock()
    node.tf_buffer = SequencedTransformBuffer()

    clock.advance(node.startup_grace + 0.01)
    node._tick()

    assert node.state == "FAULT"
    assert node.fault_latched is True
    assert node._motion_owned is False
    assert node.readiness_wait_started_at is None
    assert node.action_client.send_calls == []


def test_post_navigation_tf_miss_uses_fresh_episode_and_recovers_before_fault():
    clock = FakeMonotonic(100.0)
    ros_clock = FakeClock()
    node = _node(autostart=True)
    node._monotonic = clock
    node.started_at = clock()
    node.readiness_wait_started_at = clock()
    node.get_clock = lambda: ros_clock
    transform_buffer = QueryTimeTransformBuffer(ros_clock)
    node.tf_buffer = transform_buffer

    def refresh_non_tf_evidence():
        now = clock()
        node.last_map_at = now
        node.last_costmap_at = now
        node.last_base_status_at = now
        node.last_manipulator_status_at = now

    refresh_non_tf_evidence()
    node._tick()
    assert node.state == "SCANNING"
    assert node.readiness_wait_started_at is None

    node._tick()
    handle = _accept(node)
    handle.result_future.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert node.state == "SCANNING"
    assert node.readiness_wait_started_at is None

    # The run is now older than the initial 15-second startup grace, but this
    # is the first invalid observation after a successful navigation.
    clock.advance(node.startup_grace + 1.0)
    refresh_non_tf_evidence()
    node.map_version += 1
    transform_buffer.missing = True
    sends_before_miss = len(node.action_client.send_calls)
    node._tick()

    assert node.state == "WAITING_READY"
    assert node.fault_latched is False
    assert node.readiness_wait_started_at == pytest.approx(clock())
    assert len(node.action_client.send_calls) == sends_before_miss

    # Recovery within this new episode returns to scanning and clears its
    # episode deadline without dispatching a goal.
    transform_buffer.missing = False
    refresh_non_tf_evidence()
    node._tick()
    assert node.state == "SCANNING"
    assert node.fault_latched is False
    assert node.readiness_wait_started_at is None
    assert len(node.action_client.send_calls) == sends_before_miss

    # A subsequent continuous miss gets a fresh 15-second budget and then
    # faults closed when that episode remains invalid.
    node.map_version += 1
    refresh_non_tf_evidence()
    transform_buffer.missing = True
    node._tick()
    assert node.state == "WAITING_READY"
    episode_started_at = node.readiness_wait_started_at
    assert episode_started_at == pytest.approx(clock())

    clock.advance(node.startup_grace + 0.01)
    refresh_non_tf_evidence()
    node._tick()
    assert node.state == "FAULT"
    assert node.fault_latched is True
    assert node.readiness_wait_started_at is None
    assert len(node.action_client.send_calls) == sends_before_miss


def test_production_selection_looks_up_tf_after_frontier_clustering(monkeypatch):
    node = _node(autostart=True)
    ros_clock = FakeClock()
    events = []
    node.get_clock = lambda: ros_clock
    node.tf_buffer = QueryTimeTransformBuffer(ros_clock, events=events)
    original_clusters = frontier_explorer_module.frontier_clusters

    def delayed_clusters(*args, **kwargs):
        events.append("clusters")
        ros_clock.nanoseconds += 2_000_000_000
        return original_clusters(*args, **kwargs)

    monkeypatch.setattr(
        frontier_explorer_module, "frontier_clusters", delayed_clusters)
    node.state = "SCANNING"

    node._tick()

    assert events == ["clusters", "lookup"]
    assert len(node.tf_buffer.calls) == 1
    assert len(node.action_client.send_calls) == 1
    assert node.state == "GOAL_PENDING"
    assert node.fault_latched is False


def test_selection_dispatches_farther_same_cluster_frontier_when_representative_is_too_close():
    node = _node(autostart=True)
    node.latest_map = _map_message(
        data=[0, 0, -1, -1, -1, -1, -1, -1, -1, -1], width=5, height=2)
    node.map_version += 1
    node.latest_costmap = _costmap_message(data=[0] * 10, width=5, height=2)
    node.costmap_version += 1
    node.state = "PLANNING"

    node._select_frontier(node.run_generation, _transform(x=0.5, y=0.5))

    assert len(node.action_client.send_calls) == 1
    assert node.action_client.send_calls[0].pose.pose.position.x == pytest.approx(1.5)
    assert node._pending_candidate == (1, 0)
    assert node.no_frontier_updates_seen == 0


def test_failed_goal_world_is_captured_at_reservation_and_reprojected_after_pending_map_change():
    node = _node(autostart=True)
    attempted_world = (1.5, 0.5)

    _dispatch_pending(node, candidate=(1, 0), goal_world=attempted_world)
    assert node._attempted_goal_world == attempted_world

    node._map_callback(_map_message(data=[0, -1], origin_x=1.0))
    node.action_client.pending[-1].set_result(SimpleNamespace(accepted=False))

    assert node.failed_goal_worlds == {attempted_world}
    assert node._attempted_goal_world is None

    node._costmap_callback(
        _costmap_message(data=[0, 0], origin_x=1.0))
    node.state = "PLANNING"
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert len(node.action_client.send_calls) == 1
    assert node.failed_goal_worlds == {attempted_world}


def test_failed_world_reprojection_uses_translated_resolution_and_planar_yaw():
    node = _node(autostart=True)
    grid = _map_message(
        data=[0, 0, -1, 0, 0, -1], width=3, height=2,
        resolution=2.0, origin_x=10.0, origin_y=20.0,
        yaw=math.pi / 2.0)
    costmap = _costmap_message(
        data=[0] * 6, width=3, height=2, resolution=2.0,
        origin_x=10.0, origin_y=20.0, yaw=math.pi / 2.0)
    node.latest_map = grid
    node.map_version += 1
    node.last_map_at = node._monotonic()
    node.latest_costmap = costmap
    node.costmap_version += 1
    node.last_costmap_at = node._monotonic()
    failed_world = frontier_cell_world(grid, (1, 0))
    node.failed_goal_worlds = {failed_world}
    node.state = "PLANNING"

    assert failed_world == pytest.approx((9.0, 23.0))
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert len(node.action_client.send_calls) == 1
    assert node._pending_candidate == (1, 1)
    goal = node.action_client.send_calls[0]
    assert goal.pose.pose.position.x == pytest.approx(7.0)
    assert goal.pose.pose.position.y == pytest.approx(23.0)
    assert node.failed_goal_worlds == {failed_world}


def test_out_of_bounds_failed_world_is_retained_until_later_geometry_contains_it():
    node = _node(autostart=True)
    attempted_world = (1.5, 0.5)
    _dispatch_pending(node, candidate=(1, 0), goal_world=attempted_world)
    node.action_client.pending[-1].set_result(SimpleNamespace(accepted=False))
    assert node.failed_goal_worlds == {attempted_world}

    node._map_callback(_map_message(data=[0], width=1, height=1))
    node.state = "PLANNING"
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))
    assert len(node.action_client.send_calls) == 1
    assert node.failed_goal_worlds == {attempted_world}

    node._map_callback(_map_message(data=[0, -1], origin_x=1.0))
    node._costmap_callback(
        _costmap_message(data=[0, 0], origin_x=1.0))
    node.state = "PLANNING"
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert len(node.action_client.send_calls) == 1
    assert node.failed_goal_worlds == {attempted_world}


def test_accepted_start_clears_run_scoped_failed_goal_worlds():
    node = _node(autostart=True)
    node.state = "STOPPED"
    node.failed_goal_worlds = {(1.5, 0.5)}

    response = _start(node)

    assert response.success is True
    assert node.failed_goal_worlds == set()


def test_failed_goal_worlds_survive_success_cleanup_skipped_cluster_and_map_update():
    node = _node(autostart=True)
    retained_world = (1.5, 0.5)
    node.failed_goal_worlds = {retained_world}
    _dispatch_pending(node, candidate=(1, 0), goal_world=retained_world)
    handle = _accept(node)
    handle.result_future.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert node.failed_goal_worlds == {retained_world}

    with node._lock:
        node._clear_motion_locked()
    assert node.failed_goal_worlds == {retained_world}

    node._map_callback(_map_message(data=[0, -1], origin_x=1.0))
    node._costmap_callback(
        _costmap_message(data=[0, 0], origin_x=1.0))
    node.state = "PLANNING"
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert len(node.action_client.send_calls) == 1
    assert node.failed_goal_worlds == {retained_world}


@pytest.mark.parametrize("change", ["map", "costmap", "authority"])
def test_reservation_discards_plan_when_evidence_changes_during_action_wait(change):
    node = _node(autostart=True)
    node.state = "PLANNING"
    node.no_frontier_updates_seen = 2
    old_wait = node.action_client.wait_for_server

    def change_evidence(timeout_sec):
        if change == "map":
            node.latest_map = _map_message(data=[0, 0])
            node.map_version += 1
        elif change == "costmap":
            node.latest_costmap = _costmap_message(data=[253, 253])
            node.last_costmap_at = node._monotonic()
            node.costmap_version += 1
        else:
            node.base_status.valid = False
        return old_wait(timeout_sec)

    node.action_client.wait_for_server = change_evidence
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.action_client.send_calls == []
    assert node.state == "SCANNING"
    assert node.processed_map_version == -1
    assert node._motion_owned is False
    assert node.no_frontier_updates_seen == 2
    assert node.reason.startswith("frontier plan discarded because")


def test_evidence_change_during_blocked_selection_prevents_dispatch(monkeypatch):
    node = _node(autostart=True)
    node.state = "PLANNING"
    selection_started = threading.Event()
    release_selection = threading.Event()
    selection_errors = []

    def blocked_candidates(*_args, **_kwargs):
        selection_started.set()
        release_selection.wait(2.0)
        return [(1, 0)]

    monkeypatch.setattr(
        frontier_explorer_module,
        "costmap_frontier_candidates",
        blocked_candidates)

    def select():
        try:
            node._select_frontier(
                node.run_generation, _transform(x=0.0, y=0.0))
        except BaseException as exc:  # pragma: no cover - test failure detail
            selection_errors.append(exc)

    selection_thread = threading.Thread(target=select)
    selection_thread.start()
    assert selection_started.wait(1.0)

    node._map_callback(_map_message(data=[0, 0]))
    release_selection.set()
    selection_thread.join(2.0)

    assert not selection_thread.is_alive()
    assert selection_errors == []
    assert node.action_client.send_calls == []
    assert node.state == "SCANNING"
    assert node.reason == "frontier plan discarded because map evidence changed"
    assert node._motion_owned is False
    assert node.goal_failures == 0


@pytest.mark.parametrize("stale", ["map", "costmap", "authority"])
def test_reservation_discards_plan_when_receipt_expires_during_action_wait(stale):
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    node.started_at = clock()
    node.readiness_wait_started_at = node.started_at
    node.no_frontier_updates_seen = 2
    node.last_map_at = clock()
    node.last_costmap_at = clock()
    node.last_base_status_at = clock()
    node.last_manipulator_status_at = clock()
    old_wait = node.action_client.wait_for_server

    def expire_evidence(timeout_sec):
        if stale != "costmap":
            clock.advance(node.map_timeout + 0.01)
            if stale != "map":
                node.last_map_at = clock()
            if stale != "authority":
                node.last_base_status_at = clock()
                node.last_manipulator_status_at = clock()
            node.last_costmap_at = clock()
        return old_wait(timeout_sec)

    node.action_client.wait_for_server = expire_evidence
    if stale == "costmap":
        original_readiness = node._costmap_readiness
        readiness_calls = [0]

        def expire_after_readiness():
            readiness_calls[0] += 1
            result = original_readiness()
            if readiness_calls[0] == 2:
                node.last_costmap_at = clock() - node.map_timeout - 0.01
            return result

        node._costmap_readiness = expire_after_readiness
    node.state = "PLANNING"
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.action_client.send_calls == []
    assert node.state == ("WAITING_READY" if stale == "map" else "SCANNING")
    assert node.processed_map_version == -1
    assert node._motion_owned is False
    assert node.no_frontier_updates_seen == 2
    if stale == "map":
        assert node.reason == "fresh valid map is unavailable"
    else:
        assert "frontier plan discarded" in node.reason


def test_reservation_discards_plan_when_carried_tf_expires_during_action_wait():
    node = _node(autostart=True)
    node.state = "PLANNING"
    ros_clock = FakeClock()
    node.get_clock = lambda: ros_clock
    transform = _transform(stamp_ns=ros_clock.nanoseconds - 500_000_000)
    old_wait = node.action_client.wait_for_server

    def expire_tf(timeout_sec):
        ros_clock.nanoseconds += 1_100_000_000
        return old_wait(timeout_sec)

    node.action_client.wait_for_server = expire_tf
    node._select_frontier(node.run_generation, transform)

    assert node.action_client.send_calls == []
    assert node.state == "SCANNING"
    assert node.processed_map_version == -1
    assert node._motion_owned is False
    assert node.reason == "frontier plan discarded because carried TF is stale"


def test_fully_blocked_frontier_is_skipped_without_motion_or_goal_failure():
    node = _node(autostart=True)
    node.latest_costmap = _costmap_message(data=[253, 253])
    node.state = "PLANNING"

    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.state == "SCANNING"
    assert node.reason == "no costmap-valid frontier in this map update"
    assert node.no_frontier_updates_seen == 1
    assert node.goal_failures == 0
    assert node.blacklist == set()
    assert node._motion_owned is False
    assert node._motion_token is None
    assert node.action_client.send_calls == []

    for _ in range(node.no_frontier_limit - 1):
        node.state = "PLANNING"
        node.map_version += 1
        node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))
    assert node.state == "INCOMPLETE"
    assert node.reason == "exploration incomplete: no safe costmap-valid frontier remains"
    assert node.goal_failures == 0
    assert node.blacklist == set()


def test_footprint_blocked_frontier_reaches_incomplete_without_dispatch():
    node = _node(autostart=True)
    width = height = 20
    data = [0] * (width * height)
    data[5 * width + 6] = 253
    node.latest_costmap = _costmap_message(
        data=data, width=width, height=height, resolution=1.0,
        origin_x=-5.0, origin_y=-5.0)
    node.costmap_version += 1
    node.state = "PLANNING"

    for index in range(node.no_frontier_limit):
        node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))
        assert node.action_client.send_calls == []
        assert node.goal_failures == 0
        assert node.blacklist == set()
        if index + 1 < node.no_frontier_limit:
            node.state = "PLANNING"
            node.map_version += 1

    assert node.state == "INCOMPLETE"
    assert node.reason == "exploration incomplete: no safe costmap-valid frontier remains"


def test_true_raw_frontier_exhaustion_reaches_complete():
    node = _node(autostart=True)
    node.latest_map = _map_message(data=[0, 0, 0, 0], width=2, height=2)
    node.map_version += 1
    node.state = "PLANNING"

    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))
    for _ in range(node.no_frontier_limit - 1):
        node.state = "PLANNING"
        node.map_version += 1
        node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.state == "COMPLETE"
    assert node.reason == "exploration complete: no costmap-valid frontier remains"
    assert node.no_frontier_updates_seen == node.no_frontier_limit
    assert node.goal_failures == 0
    assert node.action_client.send_calls == []


def test_dispatchable_choice_resets_exhaustion_only_after_reservation():
    node = _node(autostart=True)
    node.latest_costmap = _costmap_message(data=[253, 253])
    node.state = "PLANNING"
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))
    assert node.no_frontier_updates_seen == 1

    node.latest_costmap = _costmap_message(data=[0, 0])
    node.costmap_version += 1
    node.last_costmap_at = node._monotonic()
    node.map_version += 1
    node.state = "PLANNING"
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert len(node.action_client.send_calls) == 1
    assert node.no_frontier_updates_seen == 0


def test_incomplete_stop_preserves_safe_stop_and_restart_clears_it():
    node = _node(autostart=True)
    node.latest_costmap = _costmap_message(data=[253, 253])
    for index in range(node.no_frontier_limit):
        node.state = "PLANNING"
        node.map_version += 1
        node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))
        if index + 1 < node.no_frontier_limit:
            assert node.state == "SCANNING"
    assert node.state == "INCOMPLETE"

    response = node._stop_callback(None, Trigger.Response())
    assert response.success is True
    assert node.state == "INCOMPLETE"
    assert node.reason == "exploration incomplete: no safe costmap-valid frontier remains"

    response = _start(node)
    assert response.success is True
    assert node.state == "WAITING_READY"
    assert node.no_frontier_updates_seen == 0
    assert node.run_generation == 2


@pytest.mark.parametrize("case", [
    "missing", "stale", "wrong_frame", "bad_dimensions", "bad_geometry"])
def test_invalid_costmap_blocks_planning_before_dispatch(case):
    node = _node(autostart=True)
    if case == "missing":
        node.latest_costmap = None
        node.last_costmap_at = None
    elif case == "stale":
        node.last_costmap_at = node._monotonic() - node.map_timeout - 0.1
    elif case == "wrong_frame":
        node.latest_costmap = _costmap_message(frame_id="odom")
    elif case == "bad_dimensions":
        node.latest_costmap = _costmap_message(data=[0], width=2)
    else:
        node.latest_costmap = _costmap_message()
        node.latest_costmap.metadata.origin.orientation.w = 0.0
    node.state = "PLANNING"

    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.state == "WAITING_READY"
    assert node.reason == "fresh valid global costmap is unavailable"
    assert node.processed_map_version == -1
    assert node.goal_failures == 0
    assert node.blacklist == set()
    assert node._motion_owned is False
    assert node.action_client.send_calls == []


def test_invalid_costmap_after_startup_grace_faults_without_motion_reservation():
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    node.started_at = clock()
    node.readiness_wait_started_at = node.started_at
    node.last_map_at = clock()
    node.last_costmap_at = clock()
    node.latest_costmap.metadata.size_x = 3
    node.state = "PLANNING"
    clock.advance(node.startup_grace + 0.01)
    node.last_map_at = clock()

    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.state == "FAULT"
    assert node.fault_latched is True
    assert "fresh valid global costmap is unavailable" in node.reason
    assert node._motion_owned is False
    assert node.action_client.send_calls == []


def test_invalid_map_during_startup_grace_remains_readiness_gated():
    node = _node(autostart=True)
    node.latest_map = _map_message(data=[0, 0.5])
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    node.failed_goal_worlds = {(1.5, 0.5)}

    node._tick()

    assert node.state == "WAITING_READY"
    assert node.no_frontier_updates_seen == 2
    assert node.goal_failures == 0
    assert node.blacklist == {(7, 7)}
    assert node.failed_goal_worlds == {(1.5, 0.5)}
    assert node._motion_token is None
    assert node._motion_owned is False
    assert node.active_goal is None
    assert node.action_client.send_calls == []


def test_invalid_map_after_startup_grace_faults_without_motion_reservation():
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    node.started_at = clock()
    node.readiness_wait_started_at = node.started_at
    node.last_map_at = clock()
    node.latest_map = _map_message(data=[0, 0.5])
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    node.failed_goal_worlds = {(1.5, 0.5)}
    clock.advance(node.startup_grace + 0.01)

    node._tick()

    assert node.state == "FAULT"
    assert node.fault_latched is True
    assert "fresh valid map is unavailable" in node.reason
    assert node.no_frontier_updates_seen == 2
    assert node.goal_failures == 0
    assert node.blacklist == {(7, 7)}
    assert node.failed_goal_worlds == {(1.5, 0.5)}
    assert node._motion_token is None
    assert node._motion_owned is False
    assert node.active_goal is None
    assert node.action_client.send_calls == []


def test_invalid_map_in_planning_uses_gate_without_counting_exhaustion():
    node = _node(autostart=True)
    node.state = "PLANNING"
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    node.failed_goal_worlds = {(1.5, 0.5)}
    node.latest_map = _map_message(data=[0, 0.5])

    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.state == "WAITING_READY"
    assert node.reason == "fresh valid map is unavailable"
    assert node.processed_map_version == -1
    assert node.no_frontier_updates_seen == 2
    assert node.goal_failures == 0
    assert node.blacklist == {(7, 7)}
    assert node.failed_goal_worlds == {(1.5, 0.5)}
    assert node._motion_token is None
    assert node._motion_owned is False
    assert node.active_goal is None
    assert node.action_client.send_calls == []


def test_invalid_current_map_is_rejected_before_reservation():
    node = _node(autostart=True)
    node.state = "PLANNING"
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    old_wait = node.action_client.wait_for_server

    def invalidate_map(timeout_sec):
        node.latest_map.data[1] = 0.5
        return old_wait(timeout_sec)

    node.action_client.wait_for_server = invalidate_map
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.state == "WAITING_READY"
    assert "fresh valid map is unavailable" in node.reason
    assert node.processed_map_version == -1
    assert node.no_frontier_updates_seen == 2
    assert node.goal_failures == 0
    assert node.blacklist == {(7, 7)}
    assert node._motion_token is None
    assert node._motion_owned is False
    assert node.active_goal is None
    assert node.action_client.send_calls == []


def test_invalid_map_during_navigation_keeps_cancellation_ownership_gate():
    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    node.latest_map.data[1] = 0.5

    node._tick()

    assert node.state == "CANCELLING"
    assert handle.cancel_calls == 1
    assert node._motion_owned is True

    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "FAULT"
    assert node._motion_owned is False


def test_wrong_frame_map_receipt_during_startup_uses_readiness_gate():
    node = _node(autostart=True)
    wrong_frame = _map_message()
    wrong_frame.header.frame_id = "odom"
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    node.failed_goal_worlds = {(1.5, 0.5)}

    node._map_callback(wrong_frame)

    assert node.latest_map is wrong_frame
    assert node.map_version == 2
    assert node.state == "WAITING_READY"
    assert node.fault_latched is False
    assert node._motion_token is None
    assert node._motion_owned is False
    assert node.no_frontier_updates_seen == 2
    assert node.blacklist == {(7, 7)}
    assert node.failed_goal_worlds == {(1.5, 0.5)}
    assert node.action_client.send_calls == []

    node._tick()

    assert node.state == "WAITING_READY"
    assert node.fault_latched is False
    assert node.no_frontier_updates_seen == 2
    assert node.action_client.send_calls == []


def test_wrong_frame_map_receipt_during_planning_uses_planning_gate():
    node = _node(autostart=True)
    node.state = "PLANNING"
    wrong_frame = _map_message()
    wrong_frame.header.frame_id = "odom"
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    node.failed_goal_worlds = {(1.5, 0.5)}

    node._map_callback(wrong_frame)
    assert node.state == "PLANNING"
    assert node.fault_latched is False

    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    assert node.state == "WAITING_READY"
    assert node.reason == "fresh valid map is unavailable"
    assert node.fault_latched is False
    assert node.processed_map_version == -1
    assert node._motion_token is None
    assert node._motion_owned is False
    assert node.no_frontier_updates_seen == 2
    assert node.blacklist == {(7, 7)}
    assert node.failed_goal_worlds == {(1.5, 0.5)}
    assert node.action_client.send_calls == []


def test_wrong_frame_map_receipt_during_motion_uses_readiness_loss_cancellation():
    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    wrong_frame = _map_message()
    wrong_frame.header.frame_id = "odom"
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    node.failed_goal_worlds = {(1.5, 0.5)}

    node._map_callback(wrong_frame)

    assert node.state == "NAVIGATING"
    assert node.fault_latched is False
    assert handle.cancel_calls == 0

    node._tick()

    assert node.state == "CANCELLING"
    assert node.fault_latched is False
    assert handle.cancel_calls == 1
    assert node._motion_owned is True
    assert node.no_frontier_updates_seen == 2
    assert node.blacklist == {(7, 7)}
    assert node.failed_goal_worlds == {(1.5, 0.5)}
    assert node.action_client.send_calls != []

    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "FAULT"
    assert node._motion_owned is False


@pytest.mark.parametrize("map_failure", ("invalid", "stale"))
@pytest.mark.parametrize("after_startup_grace", (False, True))
def test_reservation_map_failure_uses_readiness_gate_without_side_effects(
        map_failure, after_startup_grace):
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    node.started_at = clock()
    node.readiness_wait_started_at = node.started_at
    node.last_map_at = clock()
    node.last_costmap_at = clock()
    node.last_base_status_at = clock()
    node.last_manipulator_status_at = clock()
    node.state = "PLANNING"
    node.no_frontier_updates_seen = 2
    node.blacklist = {(7, 7)}
    node.failed_goal_worlds = {(1.5, 0.5)}
    old_wait = node.action_client.wait_for_server

    def fail_map_during_wait(timeout_sec):
        if after_startup_grace:
            clock.advance(node.startup_grace + 0.01)
            node.last_costmap_at = clock()
            node.last_base_status_at = clock()
            node.last_manipulator_status_at = clock()
        if map_failure == "invalid":
            node.latest_map.data[1] = 0.5
            if after_startup_grace:
                node.last_map_at = clock()
        else:
            node.last_map_at = clock() - node.map_timeout - 0.01
        return old_wait(timeout_sec)

    node.action_client.wait_for_server = fail_map_during_wait
    node._select_frontier(node.run_generation, _transform(x=0.0, y=0.0))

    expected_state = "FAULT" if after_startup_grace else "WAITING_READY"
    assert node.state == expected_state
    assert node.fault_latched is after_startup_grace
    assert "fresh valid map is unavailable" in node.reason
    assert node.processed_map_version == -1
    assert node.no_frontier_updates_seen == 2
    assert node.goal_failures == 0
    assert node.blacklist == {(7, 7)}
    assert node.failed_goal_worlds == {(1.5, 0.5)}
    assert node._motion_token is None
    assert node._motion_owned is False
    assert node.active_goal is None
    assert node.action_client.send_calls == []


def test_costmap_loss_during_navigation_does_not_change_cancellation_behavior():
    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    node.latest_costmap = None
    node.last_costmap_at = None

    node._tick()

    assert node.state == "NAVIGATING"
    assert handle.cancel_calls == 0
    assert node._motion_owned is True

    node._begin_cancel("operator requested exploration stop")
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "STOPPED"
    assert node._motion_owned is False


@pytest.mark.parametrize("case", ("stale", "future", "invalid_quaternion"))
def test_invalid_carried_tf_is_rejected_without_dispatch(case):
    if case == "stale":
        transform = _transform(stamp_ns=8_000_000_000)
    elif case == "future":
        transform = _transform(stamp_ns=10_500_000_000)
    else:
        transform = _transform()
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 0.0

    node = _node(autostart=True)
    node.tf_buffer = SequencedTransformBuffer()
    node.state = "PLANNING"

    node._select_frontier(node.run_generation, transform)

    assert node.tf_buffer.lookup_calls == 0
    assert node.action_client.send_calls == []
    assert node.state == "WAITING_READY"
    assert node.reason == "timestamped fresh map to base_footprint TF is unavailable"
    assert node.processed_map_version == -1
    assert node.fault_latched is False


def test_pending_deadline_and_late_acceptance_cancel_once():
    node = _node(autostart=True)
    _dispatch_pending(node)
    node._motion_deadline = time.monotonic() - 0.01
    node._tick()
    assert node.state == "CANCELLING"
    handle = _accept(node)
    assert handle.cancel_calls == 1
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert node.state == "CANCELLING"
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "FAULT"


def test_faulted_pending_request_cancels_late_acceptance_after_both_proofs():
    node = _node(autostart=True)
    _dispatch_pending(node)
    node._fault("fault while goal request is pending")
    assert node.state == "CANCELLING"
    assert node.cancel_requested is True
    assert node._cancel_target == "pending"
    assert node.cancel_event.is_set() is False

    handle = _accept(node)
    assert node.state == "CANCELLING"
    assert handle.cancel_calls == 1
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "CANCELLING"
    assert node.cancel_event.is_set() is False
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert node.state == "FAULT"
    assert node.cancel_event.is_set() is True
    assert handle.cancel_calls == 1


@pytest.mark.parametrize("proof_order", [("result", "ack"), ("ack", "result")])
def test_faulted_active_goal_waits_for_cancel_ack_and_result(proof_order):
    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    node._fault("fault while navigating")
    assert node.state == "CANCELLING"
    assert node.cancel_event.is_set() is False
    assert handle.cancel_calls == 1
    for proof in proof_order:
        if proof == "result":
            handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        else:
            handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
        if proof == proof_order[0]:
            assert node.state == "CANCELLING"
            assert node.cancel_event.is_set() is False
    assert node.state == "FAULT"
    assert node.cancel_event.is_set() is True


def test_late_cancellation_proof_does_not_replace_original_fault_reason():
    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    node._fault("original authority fault")
    assert node.reason == "original authority fault"
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    assert node.state == "FAULT"
    assert node.reason == "original authority fault"
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert node.state == "FAULT"
    assert node.reason == "original authority fault"


def test_faulted_pending_rejection_commits_terminal_fault_proof():
    node = _node(autostart=True)
    _dispatch_pending(node)
    node._fault("fault while goal request is pending")
    rejected = SimpleNamespace(accepted=False)
    node.action_client.pending[-1].set_result(rejected)
    assert node.state == "FAULT"
    assert node.cancel_event.is_set() is True
    assert node._motion_owned is False
    assert node._pending is False


def test_operator_stop_waits_for_cancel_ack_and_canceled_result():
    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    node._begin_cancel("operator requested exploration stop")
    assert handle.cancel_calls == 1
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert node.state == "CANCELLING"
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "STOPPED"
    assert node.cancel_event.is_set()


def test_active_authority_loss_enters_cancelling_and_timeout_faults():
    node = _node(autostart=True)
    clock = FakeMonotonic()
    node._monotonic = clock
    _dispatch_pending(node)
    handle = _accept(node)
    node.base_status.valid = False
    node._tick()
    assert node.state == "CANCELLING"
    assert handle.cancel_calls == 1
    clock.advance(node.cancel_timeout + 0.01)
    node._tick()
    assert node.state == "FAULT"


@pytest.mark.parametrize("stamp_ns, expected", [
    (9_500_000_000, True),
    (0, False),
    (10_500_000_000, False),
    (8_000_000_000, False),
])
def test_tf_requires_nonzero_finite_ros_time_fresh_stamp(stamp_ns, expected):
    assert FrontierExplorer._valid_transform(
        _transform(stamp_ns), 10_000_000_000, 1.0) is expected


def test_lookup_uses_exact_nonzero_node_ros_time():
    class ExactNodeClock:
        def __init__(self, nanoseconds):
            self.nanoseconds = nanoseconds
            self.times = []

        def now(self):
            value = rclpy.time.Time(nanoseconds=self.nanoseconds)
            self.times.append(value)
            return value

    node = _node(autostart=True)
    clock = ExactNodeClock(10_000_000_000)
    buffer = QueryTimeTransformBuffer()
    node.get_clock = lambda: clock
    node.tf_buffer = buffer

    assert node._lookup_fresh_transform() is not None
    assert len(buffer.calls) == 1
    query_time = buffer.calls[0][2]
    assert query_time is clock.times[0]
    assert query_time.nanoseconds == 10_000_000_000
    assert query_time.nanoseconds > 0


def test_zero_node_ros_time_fails_closed_before_tf_lookup():
    class ZeroNodeClock:
        def now(self):
            return rclpy.time.Time(nanoseconds=0)

    node = _node(autostart=True)
    buffer = QueryTimeTransformBuffer()
    clock = ZeroNodeClock()
    node.get_clock = lambda: clock
    node.tf_buffer = buffer

    assert node._lookup_fresh_transform() is None
    assert buffer.calls == []


def test_newer_slightly_future_tf_sample_is_rejected_with_valid_past_sample():
    now_ros_ns = 10_000_000_000
    past_transform = _transform(stamp_ns=now_ros_ns - 80_000_000)
    future_transform = _transform(stamp_ns=now_ros_ns + 1_000_000)

    assert FrontierExplorer._valid_transform(
        past_transform, now_ros_ns, 1.0) is True
    assert FrontierExplorer._valid_transform(
        future_transform, now_ros_ns, 1.0) is False

    class FutureTransformBuffer:
        def __init__(self):
            self.calls = []

        def lookup_transform(self, target, source, query_time, **kwargs):
            self.calls.append((target, source, query_time, kwargs))
            return future_transform

    node = _node(autostart=True)
    node.get_clock = lambda: FakeClock(now_ros_ns)
    node.tf_buffer = FutureTransformBuffer()
    assert node._lookup_fresh_transform() is None
    assert len(node.tf_buffer.calls) == 1


def test_stale_generation_callback_cannot_change_new_run():
    node = _node(autostart=True)
    _dispatch_pending(node)
    old_future = node.action_client.pending[-1]
    with node._lock:
        node._clear_motion_locked()
        node.state = "STOPPED"
    assert _start(node).success is True
    old_handle = FakeGoalHandle()
    old_future.set_result(old_handle)
    assert node.state == "WAITING_READY"
    assert node.active_goal is None
    assert old_handle.cancel_calls == 1


def test_cancel_reject_and_non_canceled_result_latch_fault():
    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    node._begin_cancel("operator requested exploration stop")
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[]))
    assert node.state == "FAULT"
    assert node._motion_owned is True
    rejection_outcome = node._cancel_record.outcome
    assert _start(node).success is False
    repeated = node._stop_callback(None, Trigger.Response())
    assert (repeated.success, repeated.message) == rejection_outcome

    node = _node(autostart=True)
    _dispatch_pending(node)
    handle = _accept(node)
    node._begin_cancel("operator requested exploration stop")
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    assert node.state == "FAULT"


def test_terminal_status_is_published_before_stop_waiter_signal():
    node = _node(autostart=True)
    node.status_pub = FakePublisher(node.cancel_event)
    response = Trigger.Response()
    result = node._stop_callback(None, response)
    assert result.success is True
    assert node.state == "STOPPED"
    assert node.status_pub.event_was_set[-1] is False
    fields = _diagnostic_values(node)
    assert {"state", "reason", "run_generation", "motion_generation", "map_version",
            "pending", "active", "cancel_target", "cancel_ack", "goal_failures",
            "candidate", "fault_latched"} <= set(fields)


def test_repeated_stop_callers_share_one_record_and_return_immutable_outcome():
    node = _node(autostart=True)
    handle, record = _active_cancel(node)
    first, first_done, first_result = _stop_thread(node)
    assert handle.cancel_called.wait(1.0)
    deadline = record.deadline
    waiters = [first]
    results = [first_result]
    completed = [first_done]
    for _ in range(2):
        waiter, waiter_done, waiter_result = _stop_thread(node)
        waiters.append(waiter)
        completed.append(waiter_done)
        results.append(waiter_result)

    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    for waiter, waiter_done in zip(waiters, completed):
        assert waiter_done.wait(1.0)
        waiter.join(1.0)
    assert handle.cancel_calls == 1
    assert record.deadline == deadline
    assert record.outcome == (True, "exploration stopped")
    assert [result["response"].message for result in results] == [
        "exploration stopped", "exploration stopped", "exploration stopped"]

    repeated = node._stop_callback(None, Trigger.Response())
    assert (repeated.success, repeated.message) == record.outcome


def test_timeout_is_shared_and_repeated_fault_stop_returns_recorded_tuple():
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    handle, record = _active_cancel(node)
    waiter, completed, result = _stop_thread(node)
    assert handle.cancel_called.wait(1.0)
    clock.advance(node.cancel_timeout + 0.01)
    node._tick()
    assert completed.wait(1.0)
    waiter.join(1.0)
    assert (result["response"].success, result["response"].message) == (
        False, "cancellation was not confirmed; explorer faulted closed")
    assert node.state == "FAULT"
    assert node._motion_owned is True
    assert handle.cancel_calls == 1

    repeated = node._stop_callback(None, Trigger.Response())
    assert (repeated.success, repeated.message) == record.outcome
    assert repeated.message != "exploration is faulted; restart the explorer"


def test_fault_latched_pending_stop_joins_unfinished_matching_record():
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    _dispatch_pending(node)
    node._fault("fault while goal request is pending")
    record = node._cancel_record
    assert record is not None and record.outcome is None
    waiter, completed, result = _stop_thread(node)
    clock.advance(node.cancel_timeout + 0.01)
    node._tick()
    assert completed.wait(1.0)
    waiter.join(1.0)
    assert result["response"].success is False
    assert result["response"].message == record.outcome[1]
    assert result["response"].message != "exploration is faulted; restart the explorer"


def test_early_waiter_wakeup_only_rechecks_until_original_deadline():
    class EarlyWakeEvent:
        def __init__(self):
            self._event = threading.Event()
            self._early = True

        def wait(self, timeout=None):
            if self._early:
                self._early = False
                return False
            return self._event.wait(timeout)

        def set(self):
            self._event.set()

        def is_set(self):
            return self._event.is_set()

    class TestCancellationRecord:
        def __init__(self, source):
            self.token = source.token
            self.deadline = source.deadline
            self.event = EarlyWakeEvent()
            self.outcome = None

    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    handle, source_record = _active_cancel(node)
    record = TestCancellationRecord(source_record)
    node._cancel_record = record
    node.cancel_event = record.event
    waiter, completed, result = _stop_thread(node)
    assert handle.cancel_called.wait(1.0)
    assert not completed.wait(0.1)
    assert record.outcome is None
    assert node.state == "CANCELLING"

    clock.advance(node.cancel_timeout + 0.01)
    node._tick()
    assert completed.wait(1.0)
    waiter.join(1.0)
    assert result["response"].success is False
    assert record.outcome == (False, "cancellation was not confirmed; explorer faulted closed")


def test_timeout_commit_is_mutation_free_for_every_stale_authority_mismatch():
    cases = ("run", "motion", "state", "deadline", "record", "predeadline")
    for case in cases:
        clock = FakeMonotonic()
        node = _node(autostart=True)
        node._monotonic = clock
        _handle, record = _active_cancel(node)
        token = record.token
        status_count = len(node.status_pub.messages)
        expected_run = token[0]
        expected_token = token
        expected_state = "CANCELLING"
        expected_deadline = record.deadline
        candidate = record
        now = record.deadline + 0.01
        if case == "run":
            expected_run += 1
        elif case == "motion":
            expected_token = (token[0], token[1] + 1)
        elif case == "state":
            expected_state = "NAVIGATING"
        elif case == "deadline":
            expected_deadline += 1.0
        elif case == "record":
            candidate = _CancellationRecord(token, record.deadline)
        else:
            now = record.deadline - 0.01

        outcome = node._commit_cancel_timeout_locked(
            candidate, expected_run, expected_token, expected_state,
            expected_deadline, "stale timeout", now=now)
        assert outcome is None, case
        assert record.outcome is None, case
        assert record.event.is_set() is False, case
        assert node._motion_owned is True, case
        assert node.state == "CANCELLING", case
        assert node.fault_latched is False, case
        assert len(node.status_pub.messages) == status_count, case

    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    _handle, record = _active_cancel(node)
    outcome = node._commit_cancel_timeout_locked(
        record, record.token[0], record.token, "CANCELLING", record.deadline,
        "valid timeout", now=record.deadline + 0.01)
    assert outcome == (False, "cancellation was not confirmed; explorer faulted closed")
    assert node.state == "FAULT"
    assert record.event.is_set() is True


def test_completed_stop_wins_over_stale_timer_and_new_motion_detaches_old_record():
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    handle, record = _active_cancel(node)
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "STOPPED"
    old_reason = node.reason
    outcome = node._commit_cancel_timeout_locked(
        record, record.token[0], record.token, "CANCELLING", record.deadline,
        "stale timeout", now=record.deadline + 1.0)
    assert outcome == (True, "exploration stopped")
    assert node.state == "STOPPED"
    assert node.reason == old_reason
    assert node.fault_latched is False

    with node._lock:
        node._clear_motion_locked()
        node.state = "PLANNING"
    node._reserve_and_send(
        object(), (2, 0), node.run_generation,
        map_snapshot=(node.latest_map, node.map_version, node.last_map_at),
        costmap_snapshot=(
            node.latest_costmap, node.costmap_version, node.last_costmap_at),
        transform=_transform())
    new_token = node._motion_token
    assert new_token != record.token
    assert node._cancel_record is None
    assert record.event.is_set() is True
    stale = node._commit_cancel_timeout_locked(
        record, record.token[0], record.token, "CANCELLING", record.deadline,
        "stale timeout", now=record.deadline + 1.0)
    assert stale == record.outcome
    assert node._motion_token == new_token
    assert node.state == "GOAL_PENDING"
    assert node.fault_latched is False


def test_later_independent_fault_does_not_get_masked_by_old_successful_stop():
    node = _node(autostart=True)
    handle, record = _active_cancel(node)
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert record.outcome == (True, "exploration stopped")
    node._fault("independent fault after stop")
    assert node.state == "FAULT"
    response = node._stop_callback(None, Trigger.Response())
    assert (response.success, response.message) == (
        False, "exploration is faulted; restart the explorer")


def test_restart_detaches_record_but_old_stop_returns_its_recorded_outcome():
    node = _node(autostart=True)
    handle, record = _active_cancel(node)
    waiter, completed, result = _stop_thread(node)
    assert handle.cancel_called.wait(1.0)
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "STOPPED"
    old_run = node.run_generation
    started = _start(node)
    assert started.success is True
    assert node.run_generation == old_run + 1
    assert node.state == "WAITING_READY"
    assert node._cancel_record is None
    assert completed.wait(1.0)
    waiter.join(1.0)
    assert (result["response"].success, result["response"].message) == record.outcome
    assert node.state == "WAITING_READY"


@pytest.mark.parametrize("proof_order", [("result", "ack"), ("ack", "result")])
@pytest.mark.parametrize("status", [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED])
def test_unexpected_terminal_result_requires_ack_before_releasing_faulted_motion(
        proof_order, status):
    node = _node(autostart=True)
    handle, record = _active_cancel(node)
    for proof in proof_order:
        if proof == "result":
            handle.result_future.set_result(SimpleNamespace(status=status))
        else:
            handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
        if proof == proof_order[0] and proof == "result":
            assert node._motion_owned is True
            assert node.state == "FAULT"
    assert node.state == "FAULT"
    assert record.outcome == (False, "explorer faulted during cancellation")
    assert node._motion_owned is False


def test_canceled_result_without_ack_times_out_then_late_ack_releases_only_old_motion():
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    handle, record = _active_cancel(node)
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node.state == "CANCELLING"
    assert node._motion_owned is True
    assert record.event.is_set() is False

    clock.advance(node.cancel_timeout + 0.01)
    node._tick()
    assert node.state == "FAULT"
    assert node._motion_owned is True
    assert record.outcome is not None

    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert node.state == "FAULT"
    assert node._motion_owned is False
    assert node._cancel_record is record


@pytest.mark.parametrize("status", [
    GoalStatus.STATUS_UNKNOWN,
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
])
def test_unknown_or_nonterminal_cancel_result_faults_closed_and_retains_ownership(status):
    node = _node(autostart=True)
    handle, record = _active_cancel(node)
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.result_future.set_result(SimpleNamespace(status=status))
    assert node.state == "FAULT"
    assert node.fault_latched is True
    assert record.outcome == (False, "explorer faulted during cancellation")
    assert node._motion_owned is True


def test_cancel_proof_exception_is_fail_closed_and_retains_motion():
    node = _node(autostart=True)
    handle, record = _active_cancel(node)
    handle.cancel_future.set_exception(RuntimeError("cancel proof unavailable"))
    assert node.state == "FAULT"
    assert node._motion_owned is True
    assert record.outcome == (False, "explorer faulted during cancellation")

    node = _node(autostart=True)
    handle, record = _active_cancel(node)
    handle.cancel_exception = RuntimeError("cancel request failed")
    # Start a fresh cancellation after replacing the accepted handle's request
    # seam; the already-created cancellation record remains the authority.
    with node._lock:
        node._cancel_invoked = False
    node._issue_cancel(record.token, handle)
    assert node.state == "FAULT"
    assert node._motion_owned is True
    assert record.outcome == (False, "explorer faulted during cancellation")


def test_late_proof_publishes_released_diagnostics_before_waiter_release():
    clock = FakeMonotonic()
    node = _node(autostart=True)
    node._monotonic = clock
    handle, record = _active_cancel(node)
    node.status_pub = FakePublisher(record.event)
    clock.advance(node.cancel_timeout + 0.01)
    node._tick()
    assert node._motion_owned is True
    assert node.status_pub.event_was_set[-1] is False
    handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert node._motion_owned is True
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert node._motion_owned is False
    assert _diagnostic_values(node)["active"] == "false"


def test_real_two_thread_executor_planning_selection_does_not_starve_receipts():
    """A blocked planning selection must leave receipt callbacks runnable."""
    ros_context = Context()
    domain_id = (os.getpid() % 200) + 20
    explorer = None
    publisher_node = None
    observer_node = None
    executor = None
    spin_thread = None
    receipt_timer = None
    clock_observer_timer = None
    release_selection = threading.Event()
    selection_started = threading.Event()
    selection_finished = threading.Event()
    receipt_progress = threading.Event()
    clock_progress = threading.Event()
    base_progress = threading.Event()
    manipulator_progress = threading.Event()
    explorer_clock_progress = threading.Event()
    selector_lock = threading.Lock()
    timing_lock = threading.Lock()
    selector_active = 0
    selector_calls = 0
    selector_max_active = 0
    selection_entered_at = [None]
    selection_exited_at = [None]
    receipt_events = (clock_progress, base_progress, manipulator_progress)
    spin_errors = []

    original_base_callback = FrontierExplorer._base_callback
    original_manipulator_callback = FrontierExplorer._manipulator_callback

    def mark_receipt_progress(event):
        with timing_lock:
            entered_at = selection_entered_at[0]
        if entered_at is not None and time.monotonic() - entered_at >= 1.0:
            event.set()
            if all(progress.is_set() for progress in receipt_events):
                receipt_progress.set()

    def instrumented_base_callback(node, message):
        original_base_callback(node, message)
        mark_receipt_progress(base_progress)

    def instrumented_manipulator_callback(node, message):
        original_manipulator_callback(node, message)
        mark_receipt_progress(manipulator_progress)

    def blocked_select(_run_generation=None, _transform=None):
        nonlocal selector_active, selector_calls, selector_max_active
        with selector_lock:
            selector_active += 1
            selector_calls += 1
            selector_max_active = max(selector_max_active, selector_active)
        with timing_lock:
            selection_entered_at[0] = time.monotonic()
        selection_started.set()
        try:
            release_selection.wait(10.0)
        finally:
            with timing_lock:
                selection_exited_at[0] = time.monotonic()
            with selector_lock:
                selector_active -= 1
            selection_finished.set()

    def spin_executor():
        try:
            executor.spin()
        except Exception as exc:  # pragma: no cover - cleanup diagnostic
            spin_errors.append(exc)

    try:
        FrontierExplorer._base_callback = instrumented_base_callback
        FrontierExplorer._manipulator_callback = instrumented_manipulator_callback
        rclpy.init(context=ros_context, domain_id=domain_id)
        explorer = FrontierExplorer(context=ros_context)
        parameter_result = explorer.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        assert parameter_result[0].successful
        assert isinstance(
            explorer.planning_callback_group, MutuallyExclusiveCallbackGroup)
        assert explorer.planning_callback_group is not explorer.lifecycle_callback_group
        clock_before_ns = explorer.get_clock().now().nanoseconds
        explorer._readiness = (
            lambda require_action, require_costmap=True, require_transform=True: (
            True, "readiness test", None)
        )
        explorer._select_frontier = blocked_select
        with explorer._lock:
            explorer.state = "SCANNING"
            explorer.map_version = 1
            explorer.processed_map_version = -1

        publisher_node = Node(
            "frontier_receipt_publisher_%s" % os.getpid(),
            context=ros_context)
        clock_publisher = publisher_node.create_publisher(Clock, "/clock", 10)
        base_publisher = publisher_node.create_publisher(
            BaseStatus, "/amr/base/status", 10)
        manipulator_publisher = publisher_node.create_publisher(
            ManipulatorStatus, "/amr/manipulation/status", 10)
        clock_count = [0]
        clock_lock = threading.Lock()

        def clock_callback(_message):
            with clock_lock:
                clock_count[0] += 1
            mark_receipt_progress(clock_progress)

        publisher_node.create_subscription(Clock, "/clock", clock_callback, 10)

        clock_sequence = [0]

        def publish_receipts():
            sequence = clock_sequence[0]
            clock_ns = 100_000_000_000 + sequence * 20_000_000
            clock_message = Clock()
            clock_message.clock.sec = clock_ns // 1_000_000_000
            clock_message.clock.nanosec = clock_ns % 1_000_000_000
            base_message = BaseStatus()
            base_message.valid = True
            base_message.state = BaseStatus.READY
            base_message.sequence = sequence
            manipulator_message = ManipulatorStatus()
            manipulator_message.valid = True
            manipulator_message.state = ManipulatorStatus.STOWED_EMPTY
            manipulator_message.base_motion_allowed = True
            manipulator_message.sequence = sequence
            clock_publisher.publish(clock_message)
            base_publisher.publish(base_message)
            manipulator_publisher.publish(manipulator_message)
            clock_sequence[0] += 1

        receipt_timer = publisher_node.create_timer(0.02, publish_receipts)

        def observe_explorer_clock():
            with timing_lock:
                entered_at = selection_entered_at[0]
            if (entered_at is not None
                    and time.monotonic() - entered_at >= 1.0
                    and explorer.get_clock().now().nanoseconds > clock_before_ns):
                explorer_clock_progress.set()

        observer_node = Node(
            "frontier_clock_observer_%s" % os.getpid(),
            context=ros_context)
        clock_observer_timer = observer_node.create_timer(
            0.01, observe_explorer_clock)

        executor = MultiThreadedExecutor(num_threads=2, context=ros_context)
        executor.add_node(explorer)
        executor.add_node(publisher_node)
        executor.add_node(observer_node)
        spin_thread = threading.Thread(
            target=spin_executor, name="frontier-planning-executor", daemon=True)
        spin_thread.start()

        assert selection_started.wait(5.0)
        assert receipt_progress.wait(5.0)
        assert explorer_clock_progress.wait(5.0)
        assert not selection_finished.is_set()
        with selector_lock:
            assert selector_calls == 1
            assert selector_active == 1
            assert selector_max_active == 1
        with clock_lock:
            assert clock_count[0] > 0

        release_selection.set()
        assert selection_finished.wait(3.0)
        with timing_lock:
            blocked_duration = (
                selection_exited_at[0] - selection_entered_at[0])
        assert blocked_duration >= 1.0
        assert explorer.base_status.sequence > 0
        assert explorer.manipulator_status.sequence > 0
    finally:
        release_selection.set()
        if receipt_timer is not None:
            receipt_timer.cancel()
        if clock_observer_timer is not None:
            clock_observer_timer.cancel()
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if spin_thread is not None:
            spin_thread.join(3.0)
        if explorer is not None:
            explorer.destroy_node()
        if publisher_node is not None:
            publisher_node.destroy_node()
        if observer_node is not None:
            observer_node.destroy_node()
        if ros_context.ok():
            rclpy.shutdown(context=ros_context)
        FrontierExplorer._base_callback = original_base_callback
        FrontierExplorer._manipulator_callback = original_manipulator_callback
        assert not spin_errors, "executor thread failed: %s" % spin_errors


@pytest.mark.xfail(
    strict=False,
    reason=(
        "paused two-vs-three-worker TF probe is synthetic and non-diagnostic; "
        "it is not an Explorer correctness gate"))
@pytest.mark.parametrize("executor_workers", (2, 3))
def test_real_executor_diagnoses_tf_during_frontier_planning(executor_workers):
    _run_live_tf_planning_diagnostic(executor_workers)


@pytest.mark.xfail(
    strict=False,
    reason=(
        "paused two-worker capacity probe is synthetic and non-diagnostic; "
        "it is not an Explorer correctness gate"))
def test_real_executor_capacity_keeps_tf_buffer_fresh_during_cpu_loaded_selection():
    """Retained diagnostic probe; worker count and workload are unchanged."""
    # This is intentionally the production value for the required red run.
    executor_workers = 2
    ros_context = Context()
    domain_id = (os.getpid() % 200) + 40
    explorer = None
    publisher_node = None
    executor = None
    spin_thread = None
    publisher_thread = None
    release_selection = threading.Event()
    stop_publishers = threading.Event()
    selection_started = threading.Event()
    selection_finished = threading.Event()
    progress_wakeup = threading.Event()
    selector_lock = threading.Lock()
    timing_lock = threading.Lock()
    selector_active = 0
    selector_calls = 0
    selector_max_active = 0
    selection_entered_at = [None]
    selection_exited_at = [None]
    publisher_errors = []
    spin_errors = []

    def set_header_stamp(header, stamp_ns):
        header.stamp.sec = stamp_ns // 1_000_000_000
        header.stamp.nanosec = stamp_ns % 1_000_000_000

    def spin_executor():
        try:
            executor.spin()
        except Exception as exc:  # pragma: no cover - cleanup diagnostic
            spin_errors.append(exc)

    def blocked_select(_run_generation=None, _transform=None):
        nonlocal selector_active, selector_calls, selector_max_active
        with selector_lock:
            selector_active += 1
            selector_calls += 1
            selector_max_active = max(selector_max_active, selector_active)
        with timing_lock:
            selection_entered_at[0] = time.monotonic()
        selection_started.set()
        checksum = 0
        try:
            # Hold the planning callback group with CPU work; do not wait on
            # an event so the executor-capacity path remains exercised.
            while not release_selection.is_set():
                for value in range(4000):
                    checksum = (checksum + value * 17 + 3) & 0xFFFFFFFF
            if checksum == -1:  # pragma: no cover - keep the work observable
                raise AssertionError("unreachable planning checksum")
        finally:
            with timing_lock:
                selection_exited_at[0] = time.monotonic()
            with selector_lock:
                selector_active -= 1
            selection_finished.set()

    def wait_until(predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            remaining = deadline - time.monotonic()
            progress_wakeup.wait(min(0.02, max(0.0, remaining)))
            progress_wakeup.clear()
        if predicate():
            return True
        pytest.fail("timed out waiting for %s" % description)

    try:
        rclpy.init(context=ros_context, domain_id=domain_id)
        explorer = FrontierExplorer(context=ros_context)
        parameter_result = explorer.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        assert parameter_result[0].successful
        assert isinstance(
            explorer.planning_callback_group, MutuallyExclusiveCallbackGroup)

        publisher_node = Node(
            "frontier_tf_publisher_%s" % os.getpid(), context=ros_context)
        tf_broadcaster = TransformBroadcaster(publisher_node)
        clock_publisher = publisher_node.create_publisher(Clock, "/clock", 10)
        base_publisher = publisher_node.create_publisher(
            BaseStatus, "/amr/base/status", 10)
        manipulator_publisher = publisher_node.create_publisher(
            ManipulatorStatus, "/amr/manipulation/status", 10)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_publisher = publisher_node.create_publisher(
            OccupancyGrid, "/map", map_qos)
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        costmap_publisher = publisher_node.create_publisher(
            Costmap, "/amr/global_costmap/costmap_raw", costmap_qos)

        map_message = OccupancyGrid()
        map_message.header.frame_id = "map"
        map_message.info.width = 2
        map_message.info.height = 1
        map_message.info.resolution = 1.0
        map_message.info.origin.orientation.w = 1.0
        map_message.data = [0, -1]
        costmap_message = Costmap()
        costmap_message.header.frame_id = "map"
        costmap_message.metadata.size_x = 2
        costmap_message.metadata.size_y = 1
        costmap_message.metadata.resolution = 2.0
        costmap_message.metadata.origin.orientation.w = 1.0
        costmap_message.data = [0, 0]
        map_to_odom = TransformStamped()
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_footprint"
        odom_to_base.transform.rotation.w = 1.0

        def publish_inputs():
            sequence = 0
            try:
                while not stop_publishers.is_set():
                    # TF is deliberately much denser than the other live
                    # evidence, matching the loaded production path.
                    stamp_ns = 100_000_000_000 + sequence * 20_000_000
                    set_header_stamp(map_to_odom.header, stamp_ns)
                    set_header_stamp(odom_to_base.header, stamp_ns)
                    tf_broadcaster.sendTransform([map_to_odom, odom_to_base])
                    if sequence % 10 == 0:
                        clock_message = Clock()
                        clock_message.clock.sec = stamp_ns // 1_000_000_000
                        clock_message.clock.nanosec = stamp_ns % 1_000_000_000
                        set_header_stamp(map_message.header, stamp_ns)
                        set_header_stamp(costmap_message.header, stamp_ns)
                        base_message = BaseStatus()
                        base_message.valid = True
                        base_message.state = BaseStatus.READY
                        base_message.sequence = sequence
                        manipulator_message = ManipulatorStatus()
                        manipulator_message.valid = True
                        manipulator_message.state = ManipulatorStatus.STOWED_EMPTY
                        manipulator_message.base_motion_allowed = True
                        manipulator_message.sequence = sequence
                        clock_publisher.publish(clock_message)
                        map_publisher.publish(map_message)
                        costmap_publisher.publish(costmap_message)
                        base_publisher.publish(base_message)
                        manipulator_publisher.publish(manipulator_message)
                    sequence += 1
                    progress_wakeup.set()
                    stop_publishers.wait(0.002)
            except Exception as exc:  # pragma: no cover - cleanup diagnostic
                publisher_errors.append(exc)

        executor = MultiThreadedExecutor(
            num_threads=executor_workers, context=ros_context)
        executor.add_node(explorer)
        executor.add_node(publisher_node)
        spin_thread = threading.Thread(
            target=spin_executor, name="frontier-capacity-executor", daemon=True)
        spin_thread.start()
        publisher_thread = threading.Thread(
            target=publish_inputs, name="frontier-tf-publisher", daemon=True)
        publisher_thread.start()

        def current_common_transform():
            try:
                return explorer.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time())
            except (TransformException, RuntimeError):
                return None

        def oracle_snapshot():
            now_ns = explorer.get_clock().now().nanoseconds
            transform = current_common_transform()
            tf_stamp_ns = None
            if transform is not None:
                stamp = transform.header.stamp
                tf_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            with explorer._lock:
                base_received = explorer.last_base_status_at
                manipulator_received = explorer.last_manipulator_status_at
                authority_fresh = explorer._authority_ready_locked()
            return {
                "clock_ns": now_ns,
                "tf_stamp_ns": tf_stamp_ns,
                "tf_fresh": (
                    transform is not None
                    and FrontierExplorer._valid_transform(
                        transform, now_ns, explorer.tf_timeout)),
                "base_received": base_received,
                "manipulator_received": manipulator_received,
                "authority_fresh": authority_fresh,
            }

        def initial_evidence_ready():
            map_ready = explorer._map_readiness()[0]
            costmap_ready = explorer._costmap_readiness()[0]
            snapshot = oracle_snapshot()
            return (
                map_ready and costmap_ready and snapshot["tf_fresh"]
                and snapshot["authority_fresh"])

        wait_until(initial_evidence_ready, 5.0, "initial map, TF, and authority evidence")
        baseline = oracle_snapshot()
        assert baseline["tf_stamp_ns"] is not None
        assert baseline["base_received"] is not None
        assert baseline["manipulator_received"] is not None
        explorer._select_frontier = blocked_select
        with explorer._lock:
            explorer.state = "SCANNING"
            explorer.started_at = time.monotonic()
            explorer.processed_map_version = -1

        wait_until(selection_started.is_set, 5.0, "CPU-loaded planning selection")
        observations = []
        oracle_deadline = time.monotonic() + 3.0
        oracle_progressed = False
        selective_stale = False
        while time.monotonic() < oracle_deadline:
            snapshot = oracle_snapshot()
            observations.append(snapshot)
            with timing_lock:
                entered_at = selection_entered_at[0]
            elapsed = (
                time.monotonic() - entered_at if entered_at is not None else 0.0)
            clock_progress = snapshot["clock_ns"] > baseline["clock_ns"]
            authority_progress = (
                snapshot["base_received"] > baseline["base_received"]
                and snapshot["manipulator_received"] > baseline["manipulator_received"]
                and snapshot["authority_fresh"])
            tf_progress = (
                snapshot["tf_stamp_ns"] is not None
                and snapshot["tf_stamp_ns"] > baseline["tf_stamp_ns"])
            selective_stale = selective_stale or (
                elapsed >= 1.0 and clock_progress and authority_progress
                and not snapshot["tf_fresh"] and snapshot["tf_stamp_ns"] is not None)
            oracle_progressed = (
                elapsed >= 1.0 and clock_progress and authority_progress
                and tf_progress and snapshot["tf_fresh"])
            if oracle_progressed:
                break
            remaining = oracle_deadline - time.monotonic()
            progress_wakeup.wait(min(0.02, max(0.0, remaining)))
            progress_wakeup.clear()
        if not oracle_progressed:
            pytest.fail(
                "executor capacity=%s did not reach the live TF oracle; "
                "selective_stale=%s observations=%s" % (
                    executor_workers, selective_stale, observations[-5:]))

        assert not selection_finished.is_set()
        with selector_lock:
            assert selector_calls == 1
            assert selector_active == 1
            assert selector_max_active == 1
        release_selection.set()
        assert selection_finished.wait(3.0)
        with timing_lock:
            blocked_duration = (
                selection_exited_at[0] - selection_entered_at[0])
        assert blocked_duration >= 1.0
    finally:
        release_selection.set()
        stop_publishers.set()
        if publisher_thread is not None:
            publisher_thread.join(3.0)
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if spin_thread is not None:
            spin_thread.join(3.0)
        if explorer is not None:
            explorer.destroy_node()
        if publisher_node is not None:
            publisher_node.destroy_node()
        if ros_context.ok():
            rclpy.shutdown(context=ros_context)
        assert not publisher_errors, "publisher thread failed: %s" % publisher_errors
        assert not spin_errors, "executor thread failed: %s" % spin_errors


def test_real_two_thread_executor_cancellation_does_not_starve_action_callbacks():
    """Exercise the production callback groups and action cancellation boundary."""
    ros_context = Context()
    domain_id = (os.getpid() % 200) + 20
    explorer = None
    server_node = None
    client_nodes = []
    action_server = None
    executor = None
    spin_thread = None
    stop_futures = []
    hold_future = Future()
    goal_started = threading.Event()
    execute_started = threading.Event()
    cancel_received = threading.Event()
    stop_started = threading.Event()
    cancel_response_seen = threading.Event()
    stop_responses_done = threading.Event()
    spin_errors = []
    state_lock = threading.Lock()
    stop_active = 0
    stop_max_active = 0
    stop_thread_id = None
    cancel_calls = 0
    cancel_during_stop = False
    cancel_thread_id = None

    def wait_for_event(event, description, timeout=5.0):
        if event.wait(timeout):
            return
        state = "unknown"
        if explorer is not None:
            with explorer._lock:
                state = (
                    "state=%s run=%s motion=%s owned=%s cancel_requested=%s "
                    "cancel_ack=%s" % (
                        explorer.state, explorer.run_generation,
                        explorer._motion_token, explorer._motion_owned,
                        explorer.cancel_requested, explorer._cancel_ack))
        with state_lock:
            details = (
                "stop_active=%s stop_max=%s cancel_calls=%s "
                "cancel_during_stop=%s" % (
                    stop_active, stop_max_active, cancel_calls,
                    cancel_during_stop))
        pytest.fail("timed out waiting for %s (%s; %s)" % (
            description, state, details))

    async def execute_callback(goal_handle):
        execute_started.set()
        await hold_future
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()

    def goal_callback(_goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(_cancel_request):
        nonlocal cancel_calls, cancel_during_stop, cancel_thread_id
        with state_lock:
            cancel_calls += 1
            cancel_during_stop = stop_active > 0
            cancel_thread_id = threading.get_ident()
        cancel_received.set()
        return CancelResponse.ACCEPT

    original_stop_callback = FrontierExplorer._stop_callback
    original_goal_response = FrontierExplorer._goal_response
    original_cancel_response = FrontierExplorer._cancel_response

    def instrumented_stop_callback(node, request, response):
        nonlocal stop_active, stop_max_active, stop_thread_id
        with state_lock:
            stop_active += 1
            stop_max_active = max(stop_max_active, stop_active)
            stop_thread_id = threading.get_ident()
        stop_started.set()
        try:
            return original_stop_callback(node, request, response)
        finally:
            with state_lock:
                stop_active -= 1

    def instrumented_goal_response(node, future, run_generation=None, motion_generation=None):
        result = original_goal_response(
            node, future, run_generation, motion_generation)
        with node._lock:
            active = node.active_goal is not None
        if active:
            goal_started.set()
        return result

    def instrumented_cancel_response(node, future, run_generation, motion_generation):
        result = original_cancel_response(
            node, future, run_generation, motion_generation)
        with node._lock:
            acknowledged = node._cancel_ack
        if acknowledged:
            cancel_response_seen.set()
        return result

    def mark_stop_future_done(_future):
        if all(future.done() for future in stop_futures):
            stop_responses_done.set()

    try:
        rclpy.init(context=ros_context, domain_id=domain_id)
        server_node = Node(
            "frontier_lifecycle_action_server_%s" % os.getpid(),
            context=ros_context)
        action_group = ReentrantCallbackGroup()
        assert isinstance(action_group, ReentrantCallbackGroup)
        action_server = ActionServer(
            server_node,
            NavigateToPose,
            "/amr/mission/navigate_to_pose",
            execute_callback,
            callback_group=action_group,
            goal_callback=goal_callback,
            cancel_callback=cancel_callback)

        FrontierExplorer._stop_callback = instrumented_stop_callback
        FrontierExplorer._goal_response = instrumented_goal_response
        FrontierExplorer._cancel_response = instrumented_cancel_response
        explorer = FrontierExplorer(context=ros_context)
        # The real timer would require unrelated map, TF, and authority setup.
        # The private reserve seam below provides only the accepted navigation
        # goal needed for this cancellation-boundary test.
        explorer.timer.cancel()
        assert isinstance(explorer.lifecycle_callback_group, MutuallyExclusiveCallbackGroup)
        assert isinstance(explorer.action_callback_group, ReentrantCallbackGroup)

        for index in range(3):
            client_nodes.append(Node(
                "frontier_lifecycle_client_%s_%s" % (os.getpid(), index),
                context=ros_context))
        clients = [
            node.create_client(Trigger, "/amr/exploration/stop")
            for node in client_nodes]

        executor = MultiThreadedExecutor(num_threads=2, context=ros_context)
        executor.add_node(explorer)
        executor.add_node(server_node)
        for node in client_nodes:
            executor.add_node(node)

        def spin_executor():
            try:
                executor.spin()
            except Exception as exc:  # pragma: no cover - cleanup diagnostic
                spin_errors.append(exc)

        spin_thread = threading.Thread(
            target=spin_executor, name="frontier-lifecycle-executor", daemon=True)
        spin_thread.start()

        assert explorer.action_client.wait_for_server(timeout_sec=5.0)
        for client in clients:
            assert client.wait_for_service(timeout_sec=5.0)

        with explorer._lock:
            explorer.state = "PLANNING"
            explorer.reason = "integration test reserving an action goal"
            run_generation = explorer.run_generation
            explorer.latest_map = _map_message()
            explorer.last_map_at = time.monotonic()
            explorer.map_version = 1
            explorer.latest_costmap = _costmap_message()
            explorer.last_costmap_at = time.monotonic()
            explorer.costmap_version = 1
            explorer.last_base_status_at = time.monotonic()
            explorer.last_manipulator_status_at = time.monotonic()
            explorer.base_status = SimpleNamespace(
                valid=True, state=BaseStatus.READY)
            explorer.manipulator_status = SimpleNamespace(
                valid=True,
                base_motion_allowed=True,
                state=ManipulatorStatus.STOWED_EMPTY)
            transform = _transform(stamp_ns=explorer._now_ros_ns())
            explorer._reserve_and_send(
                NavigateToPose.Goal(), (0, 0), run_generation,
                expected_started_at=explorer.started_at,
                map_snapshot=(
                    explorer.latest_map, explorer.map_version,
                    explorer.last_map_at),
                costmap_snapshot=(
                    explorer.latest_costmap, explorer.costmap_version,
                    explorer.last_costmap_at),
                transform=transform)

        wait_for_event(goal_started, "accepted active navigation goal")
        wait_for_event(execute_started, "active action execute callback")
        with explorer._lock:
            assert explorer.active_goal is not None
            assert explorer.state == "NAVIGATING"

        stop_futures = [client.call_async(Trigger.Request()) for client in clients]
        for future in stop_futures:
            future.add_done_callback(mark_stop_future_done)
        wait_for_event(stop_started, "first real stop callback")
        wait_for_event(cancel_received, "real action cancellation callback")
        wait_for_event(cancel_response_seen, "real action cancellation response")

        assert not any(future.done() for future in stop_futures)
        with state_lock:
            assert cancel_calls == 1
            assert cancel_during_stop is True
            assert stop_max_active == 1
            assert stop_thread_id != cancel_thread_id

        # The action execute task is suspended on a real rclpy Future, so the
        # second executor worker can service cancellation while the lifecycle
        # callback waits. Releasing it produces the CANCELED action result.
        hold_future.set_result(None)
        wait_for_event(stop_responses_done, "three real stop responses")
        responses = [(future.result().success, future.result().message)
                     for future in stop_futures]
        assert responses == [responses[0]] * 3
        assert responses[0] == (True, "exploration stopped")
        with explorer._lock:
            assert explorer.state == "STOPPED"
            assert explorer._motion_owned is False
    finally:
        if not hold_future.done():
            hold_future.set_result(None)
        if stop_futures:
            stop_responses_done.wait(3.0)
            for future in stop_futures:
                if not future.done():
                    future.cancel()
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if spin_thread is not None:
            spin_thread.join(3.0)
        if action_server is not None:
            action_server.destroy()
        if explorer is not None:
            explorer.destroy_node()
        if server_node is not None:
            server_node.destroy_node()
        for node in client_nodes:
            node.destroy_node()
        if ros_context.ok():
            rclpy.shutdown(context=ros_context)
        FrontierExplorer._stop_callback = original_stop_callback
        FrontierExplorer._goal_response = original_goal_response
        FrontierExplorer._cancel_response = original_cancel_response
        assert not spin_errors, "executor thread failed: %s" % spin_errors
