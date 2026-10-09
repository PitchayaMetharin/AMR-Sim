#!/usr/bin/env python3
"""Run-owned hospital acceptance: observable fault injection and normal terminal proof."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


_STOP_REQUESTED = False

def request_termination(_signum, _frame):
    global _STOP_REQUESTED
    _STOP_REQUESTED = True


WS = Path(os.environ['AMR_WS']).resolve()
sys.path.insert(0, str(WS / 'src/amr_simulation/scripts'))
from aws_exploration_diagnostics import evidence_topics
from aws_exploration_monitor import terminal_evidence


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def session_members(sid):
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(stat[3]) == sid and stat[0] != 'Z':
                members.append((int(entry.name), (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode()))
        except (OSError, ValueError):
            pass
    return members


class Observer(Node):
    def __init__(self, run_dir):
        super().__init__('hospital_sol_acceptance')
        self.events = (run_dir / 'acceptance_events.jsonl').open('w')
        self.metrics = (run_dir / 'acceptance_metrics.jsonl').open('w')
        self.explorer = {}
        self.mission = {}
        self.explorer_received = None
        self.mission_received = None
        self.tf = {}
        self.odom = None
        self.delivered = None
        self.sim_ns = 0
        self.free_area = 0.0
        self.map_received = None
        self.history = []
        self.missions = []
        self.samples = []
        self.last_metric = 0.0
        self.last_print = 0.0
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subs = [
            self.create_subscription(DiagnosticArray, '/amr/exploration/status', self.explorer_cb, 50),
            self.create_subscription(DiagnosticArray, '/amr/mission/status', self.mission_cb, 50),
            self.create_subscription(TFMessage, '/tf', self.tf_cb, qos),
            self.create_subscription(Odometry, '/amr/localization/odometry', self.odom_cb, qos),
            self.create_subscription(TwistStamped, '/amr/simulation/diagnostics/delivered_cmd_vel', self.delivered_cb, qos),
            self.create_subscription(Clock, '/clock', self.clock_cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
            self.create_subscription(OccupancyGrid, '/map', self.map_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]

    def record(self, kind, **values):
        row = dict(kind=kind, wall=time.time(), mono=time.monotonic(), sim_ns=self.sim_ns, **values)
        self.events.write(json.dumps(row) + '\n')
        self.events.flush()
        return row

    def explorer_cb(self, msg):
        for item in msg.status:
            if item.name != 'amr_exploration/frontier_explorer':
                continue
            values = {kv.key: kv.value for kv in item.values}
            keys = ('state', 'reason', 'motion_generation', 'mission_fault_class', 'localization_recoveries', 'reached_goal_count')
            if any(values.get(k) != self.explorer.get(k) for k in keys):
                row = self.record('explorer', values=values)
                self.history.append(row)
            self.explorer = values
            self.explorer_received = time.monotonic()

    def mission_cb(self, msg):
        for item in msg.status:
            if item.name != 'amr_mission/mission_supervisor':
                continue
            values = {kv.key: kv.value for kv in item.values}
            if values != self.mission:
                row = self.record('mission', values=values)
                self.missions.append(row)
            self.mission = values
            self.mission_received = time.monotonic()

    def tf_cb(self, msg):
        for tr in msg.transforms:
            pair = (tr.header.frame_id, tr.child_frame_id)
            if pair in (('map', 'odom'), ('odom', 'base_footprint')):
                self.tf[pair] = (ns(tr.header.stamp), time.monotonic())

    def odom_cb(self, msg):
        self.odom = (time.monotonic(), ns(msg.header.stamp), msg.twist.twist.linear.x,
                     msg.twist.twist.angular.z, msg.pose.pose.position.x, msg.pose.pose.position.y)

    def delivered_cb(self, msg):
        self.delivered = (time.monotonic(), msg.twist.linear.x, msg.twist.angular.z, ns(msg.header.stamp))

    def clock_cb(self, msg):
        self.sim_ns = ns(msg.clock)

    def map_cb(self, msg):
        self.free_area = sum(v == 0 for v in msg.data) * msg.info.resolution ** 2
        self.map_received = time.monotonic()

    def headroom(self):
        mo = self.tf.get(('map', 'odom'))
        ob = self.tf.get(('odom', 'base_footprint'))
        return (mo[0] - ob[0]) / 1e9 if mo and ob else None

    def stopped(self):
        now = time.monotonic()
        return bool(self.odom and self.delivered and now - self.odom[0] < 0.5
                    and now - self.delivered[0] < 0.5 and abs(self.odom[2]) <= 0.02
                    and abs(self.odom[3]) <= 0.02 and abs(self.delivered[1]) <= 0.01
                    and abs(self.delivered[2]) <= 0.01
                    and 0 <= (self.sim_ns - self.delivered[3]) / 1e9 <= 0.5)

    def driving(self):
        h = self.headroom()
        now = time.monotonic()
        return bool(self.explorer.get('state') == 'NAVIGATING'
                    and self.mission.get('stage') == 'FOLLOWING'
                    and self.explorer_received and now - self.explorer_received < 0.5
                    and self.mission.get('goal_uuid') == self.explorer.get('mission_goal_uuid')
                    and self.mission.get('outcome') == 'PENDING'
                    and all(now - value[1] < 0.5 for value in self.tf.values())
                    and self.odom and now - self.odom[0] < 0.5
                    and abs(self.odom[2]) > 0.04 and h is not None and h >= 0.5)

    def step(self):
        if _STOP_REQUESTED:
            raise RuntimeError('hospital acceptance interrupted')
        rclpy.spin_once(self, timeout_sec=0.02)
        now = time.monotonic()
        if now - self.last_metric >= 0.1:
            self.last_metric = now
            row = dict(mono=now, sim_ns=self.sim_ns, headroom=self.headroom(),
                       tf={f'{a}->{b}': v for (a, b), v in self.tf.items()},
                       odom=self.odom, delivered=self.delivered, stopped=self.stopped(),
                       state=self.explorer.get('state'), fault_class=self.explorer.get('mission_fault_class'))
            self.metrics.write(json.dumps(row) + '\n')
            self.samples.append(row)
        if now - self.last_print >= 30:
            self.last_print = now
            print(json.dumps(dict(state=self.explorer.get('state'), reason=self.explorer.get('reason'),
                                 reached=self.explorer.get('reached_goal_count'), free_m2=round(self.free_area, 1),
                                 headroom=self.headroom())), flush=True)

    def wait(self, predicate, timeout, label, launch, allow_terminal=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.step()
            if predicate():
                return
            if launch.poll() is not None:
                raise RuntimeError(f'{label}: launch exited {launch.returncode}')
            if self.explorer.get('state') == 'FAULT':
                raise RuntimeError(f'{label}: unexpected FAULT: {self.explorer}')
            if not allow_terminal and self.explorer.get('state') in ('COMPLETE', 'INCOMPLETE'):
                raise RuntimeError(f'{label}: terminal before required gate: {self.explorer}')
        raise RuntimeError(f'{label}: timeout {timeout}s, status {self.explorer}')

    def close(self):
        self.events.close()
        self.metrics.close()
        self.destroy_node()


def inject(node, launch, seconds):
    node.wait(node.driving, 300, 'mid-drive injection admission', launch)
    owned = [(pid, cmd) for pid, cmd in session_members(launch.pid)
             if 'async_slam_toolbox_node' in cmd or 'sync_slam_toolbox_node' in cmd]
    if len(owned) != 1:
        raise RuntimeError(f'expected one owned SLAM process: {owned}')
    pid, command = owned[0]
    before = dict(node.explorer)
    begin = node.record('SIGSTOP', pid=pid, command=command, seconds=seconds,
                        tf={f'{a}->{b}': v for (a, b), v in node.tf.items()}, values=before,
                        mission_uuid=node.mission.get('goal_uuid'))
    os.kill(pid, signal.SIGSTOP)
    try:
        while time.monotonic() - begin['mono'] < seconds:
            node.step()
    finally:
        os.kill(pid, signal.SIGCONT)
        end = node.record('SIGCONT', pid=pid, duration=time.monotonic() - begin['mono'])
        begin['end_mono'] = end['mono']
    return begin, before


def localization_episodes(node, begin):
    return [r for r in node.history if r['mono'] >= begin['mono']
            and r['values'].get('state') == 'RECOVERY_WAIT'
            and r['values'].get('mission_fault_class') == 'LOCALIZATION_UNAVAILABLE']


def stop_samples(node, begin):
    return [r for r in node.samples if r['mono'] >= begin['mono']
            and r['state'] == 'RECOVERY_WAIT' and r['fault_class'] == 'LOCALIZATION_UNAVAILABLE' and r['stopped']]


def consumer_boundary(node, begin):
    injected = [r for r in node.samples if begin['mono'] <= r['mono'] <= begin['end_mono']
                and r['headroom'] is not None and r['headroom'] < 0]
    frozen_negative = [(a, b) for a, b in zip(injected, injected[1:])
                       if a['tf']['map->odom'][0] == b['tf']['map->odom'][0]
                       and a['tf']['odom->base_footprint'][0] < b['tf']['odom->base_footprint'][0]]
    correlated = [r for r in node.missions if r['mono'] >= begin['mono']
                  and r['values'].get('goal_uuid') == begin['mission_uuid']
                  and r['values'].get('stage') == 'TERMINAL'
                  and r['values'].get('fault_class') == 'LOCALIZATION_UNAVAILABLE'
                  and r['values'].get('reason') == 'path following lost map localization'
                  and r['values'].get('blockage_confirmed') == 'false']
    if not frozen_negative or not correlated:
        raise RuntimeError('injected loss lacks frozen negative headroom or correlated mission terminal proof')
    node.record('consumer_boundary_proven', samples=frozen_negative[0], mission=correlated[0])


def fault_checks(node, launch):
    start, before = inject(node, launch, 1.6)
    node.wait(lambda: bool(localization_episodes(node, start)), 3, 'short loss reaches recovery', launch)
    episodes = localization_episodes(node, start)
    consumer_boundary(node, start)
    for episode in episodes:
        if any(episode['values'].get(k) != before.get(k) for k in ('goal_failures', 'blocked_count')):
            raise RuntimeError('localization recovery counted a goal failure or blockage')
    node.wait(lambda: bool(stop_samples(node, start)), 3, 'short recovery physical stop', launch)
    node.record('short_stop_proven', sample=stop_samples(node, start)[0])
    node.wait(lambda: int(node.explorer.get('reached_goal_count', '0')) > int(before.get('reached_goal_count', '0')),
              600, 'reached goal after short recovery', launch)
    if int(node.explorer.get('motion_generation', '0')) <= int(before.get('motion_generation', '0')):
        raise RuntimeError('short recovery did not dispatch a new goal')
    node.record('short_recovery_pass', values=node.explorer, episode=episodes[0])
    start, before = inject(node, launch, 7.0)
    consumer_boundary(node, start)
    episodes = localization_episodes(node, start)
    if not episodes:
        raise RuntimeError('long loss never entered localization recovery (non-diagnostic)')
    faults = [r for r in node.history if r['mono'] >= start['mono'] and r['values'].get('state') == 'FAULT']
    if not faults:
        raise RuntimeError('long loss did not end in FAULT')
    terminal = faults[0]
    values = terminal['values']
    if values.get('mission_fault_class') != 'LOCALIZATION_UNAVAILABLE' or 'localization did not recover within 5.0 s' not in values.get('reason', ''):
        raise RuntimeError(f'long loss wrong terminal: {values}')
    elapsed = terminal['mono'] - episodes[0]['mono']
    if not 4.8 <= elapsed <= 5.7:
        raise RuntimeError(f'localization deadline elapsed {elapsed:.3f}s')
    if any(values.get(k) != v for k, v in (('active', 'false'), ('pending', 'false'), ('cancel_owned_motion', 'false'), ('cancel_target', ''))):
        raise RuntimeError(f'long loss motion ownership persists: {values}')
    until = time.monotonic() + 2.0
    while time.monotonic() < until:
        node.step()
        if node.explorer.get('motion_generation') != before.get('motion_generation'):
            raise RuntimeError('new dispatch after long localization loss')
    if not node.stopped():
        raise RuntimeError('base did not remain stopped after terminal FAULT and SIGCONT')
    node.record('long_deadline_pass', elapsed=elapsed, terminal=values, odom=node.odom, delivered=node.delivered)
    return dict(short_recovery=True, long_deadline=True, long_deadline_elapsed=elapsed)


def normal_checks(node, launch, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        node.step()
        if launch.poll() is not None:
            raise RuntimeError(f'launch exited {launch.returncode}')
        state = node.explorer.get('state')
        if state == 'FAULT':
            raise RuntimeError(f'normal exploration FAULT: {node.explorer}')
        if state in ('COMPLETE', 'INCOMPLETE'):
            evidence = terminal_evidence(type('Status', (), dict(
                name='amr_exploration/frontier_explorer', message=node.explorer.get('reason', ''),
                values=[type('KV', (), dict(key=k, value=v)) for k, v in node.explorer.items()]))())
            if evidence is None:
                raise RuntimeError(f'normal terminal evidence invalid: {node.explorer}')
            if int(node.explorer.get('reached_goal_count', '0')) == 0:
                raise RuntimeError('normal run reached no goals')
            node.wait(node.stopped, 2, 'normal terminal physical stop', launch, allow_terminal=True)
            return dict(terminal=evidence, free_area_m2=node.free_area)
    raise RuntimeError(f'normal exploration timeout {timeout}s; status {node.explorer}')


def capture_slam_parameters(node, launch, run_dir):
    expected = dict(minimum_time_interval=0.5, minimum_travel_distance=0.3,
                    minimum_travel_heading=0.3, map_update_interval=1.0,
                    transform_timeout=1.0, scan_queue_size=10, resolution=0.05,
                    use_sim_time=True, scan_topic='/amr/sensors/merged_lidar/scan')
    client = node.create_client(GetParameters, '/amr/slam_toolbox/get_parameters')
    end = time.monotonic() + 90
    try:
        while time.monotonic() < end and not client.service_is_ready():
            node.step()
            if launch.poll() is not None:
                raise RuntimeError('SLAM parameter capture: launch exited')
        if not client.service_is_ready():
            raise RuntimeError('SLAM parameter service unavailable')
        request = GetParameters.Request()
        request.names = list(expected)
        future = client.call_async(request)
        while time.monotonic() < end and not future.done():
            node.step()
            if launch.poll() is not None:
                raise RuntimeError('SLAM parameter capture: launch exited')
        if not future.done():
            raise RuntimeError('SLAM parameter request timed out')
        response = future.result()
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
        actual = {name: getattr(value, fields[value.type]) if value.type in fields else None
                  for name, value in zip(request.names, response.values)}
        receipt = dict(actual=actual, expected=expected, match=actual == expected,
                       wall=time.time(), sim_ns=node.sim_ns)
        (run_dir / 'effective_slam_parameters.json').write_text(json.dumps(receipt, indent=2))
        if actual != expected:
            raise RuntimeError('effective SLAM parameters differ from reviewed overlay/invariants')
    finally:
        node.destroy_client(client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--mode', choices=('fault-injection', 'normal'), default='fault-injection')
    parser.add_argument('--timeout', type=float, default=1800)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    if WS not in run_dir.parents or run_dir.exists():
        raise RuntimeError('run directory must be a new workspace descendant')
    run_dir.mkdir(parents=True)
    for child in ('ros_home', 'tmp', 'ros', 'recorder_ros'):
        (run_dir / child).mkdir()
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='231', ROS_HOME=str(run_dir / 'ros_home'), ROS_LOG_DIR=str(run_dir / 'ros'),
               TMPDIR=str(run_dir / 'tmp'), GZ_PARTITION='amr_' + run_dir.name, AMR_RUN_ID=run_dir.name,
               PYTHONDONTWRITEBYTECODE='1')
    topics = (*evidence_topics(), '/rosout', '/amr/unsmoothed_plan', '/amr/sensors/merged_lidar/scan')
    record_cmd = ['ros2', 'bag', 'record', '--include-hidden-topics', '--include-unpublished-topics',
                  '--storage', 'sqlite3', '--compression-mode', 'message', '--compression-format', 'zstd',
                  '--compression-threads', '1', '--compression-queue-size', '0', '--max-bag-size', '25000000',
                  '--max-cache-size', '104857600', '--qos-profile-overrides-path',
                  str(WS / '.ros_logs/hospital_tools/qos_overrides.yaml'), '-o', str(run_dir / 'bag'), *topics]
    launch_cmd = ['bash', str(WS / '.ros_logs/hospital_tools/run_hospital.sh'), str(run_dir), 'simulation_diagnostics:=true']
    (run_dir / 'commands.json').write_text(json.dumps(dict(launch=launch_cmd, recorder=record_cmd,
        mode=args.mode, environment={k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_HOME', 'ROS_LOG_DIR', 'TMPDIR', 'GZ_PARTITION', 'AMR_RUN_ID')}), indent=2))
    launch_log = (run_dir / 'launch.log').open('w')
    recorder_log = (run_dir / 'recorder.log').open('w')
    record_env = dict(env, ROS_LOG_DIR=str(run_dir / 'recorder_ros'))
    previous_term = signal.signal(signal.SIGTERM, request_termination)
    recorder = None
    launch = None
    node = None
    result = dict(pass_=False, mode=args.mode, run_dir=str(run_dir))
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_HOME', 'ROS_LOG_DIR', 'TMPDIR', 'PYTHONDONTWRITEBYTECODE')})
    try:
        recorder = subprocess.Popen(record_cmd, cwd=WS, env=record_env, stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
        launch = subprocess.Popen(launch_cmd, cwd=WS, env=env, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        (run_dir / 'launch.pgid').write_text(str(launch.pid) + '\n')
        rclpy.init()
        node = Observer(run_dir)
        capture_slam_parameters(node, launch, run_dir)
        result.update(fault_checks(node, launch) if args.mode == 'fault-injection' else normal_checks(node, launch, args.timeout))
        result['pass_'] = True
    except Exception as exc:
        result['error'] = str(exc)
        print('ACCEPTANCE FAIL:', exc, flush=True)
    finally:
        if launch is not None:
            for pid, command in session_members(launch.pid):
                if 'slam_toolbox_node' in command:
                    try:
                        os.kill(pid, signal.SIGCONT)
                    except ProcessLookupError:
                        pass
            stop_log = (run_dir / 'stop.log').open('w')
            stop = subprocess.Popen([sys.executable, str(WS / '.ros_logs/hospital_tools/stop_group.py'), str(run_dir), '45'],
                                    cwd=WS, env=env, stdout=stop_log, stderr=subprocess.STDOUT)
            while stop.poll() is None:
                # Reap our launch child immediately; an unreaped zombie otherwise
                # looks like a survivor to the pre-existing stop_group helper.
                launch.poll()
                time.sleep(0.1)
            stop_log.close()
            result['shutdown_exit'] = stop.returncode
            try:
                result['launch_exit'] = launch.wait(timeout=10)
            except subprocess.TimeoutExpired:
                result['launch_exit'] = 'timeout'
                result['pass_'] = False
            if result['shutdown_exit'] != 0 or result['launch_exit'] != 0:
                result['pass_'] = False
        if recorder is not None:
            try:
                if recorder.poll() is None:
                    os.killpg(recorder.pid, signal.SIGINT)
                try:
                    result['recorder_exit'] = recorder.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(recorder.pid, signal.SIGTERM)
                    result['recorder_exit'] = recorder.wait(timeout=10)
                    result['pass_'] = False
            except Exception as exc:
                result['recorder_cleanup_error'] = str(exc)
                result['pass_'] = False
            if result.get('recorder_exit') != 0:
                result['pass_'] = False
        if node:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
        launch_log.close()
        recorder_log.close()
        if _STOP_REQUESTED:
            result['pass_'] = False
            result['error'] = 'hospital acceptance interrupted'
        (run_dir / 'result.json').write_text(json.dumps(result, indent=2))
        signal.signal(signal.SIGTERM, previous_term)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['pass_'] else 1


if __name__ == '__main__':
    sys.exit(main())
