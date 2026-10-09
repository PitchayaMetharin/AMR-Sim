#!/usr/bin/env python3
"""Session adapter: unchanged AWS acceptance with bounded compressed recording."""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

WS = Path(os.environ['AMR_WS']).resolve()
sys.path.insert(0, str(WS / 'src/amr_simulation/scripts'))
from aws_exploration_diagnostics_run import build_run_config
from aws_exploration_runner import run_aws_exploration
sys.path.insert(0, str(WS / 'phase14_evidence/factory_runtime_tools'))
import storage_budget


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--worker', action='store_true')
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    if WS not in run_dir.parents or run_dir.exists():
        raise RuntimeError('unique workspace run directory required')
    usage = storage_budget.usage()
    if any(v['logs'] >= 900_000_000 or v['evidence'] >= 11_500_000_000 for v in usage.values()):
        raise RuntimeError(f'insufficient authorized storage reserve: {usage}')
    scratch = run_dir.with_name(run_dir.name + '_control')
    if not args.worker:
        scratch.mkdir()
        (scratch / 'tmp').mkdir()
        (scratch / 'ros_home').mkdir()
        environment = dict(os.environ, TMPDIR=str(scratch / 'tmp'), ROS_HOME=str(scratch / 'ros_home'))
        command = [sys.executable, __file__, '--run-dir', str(run_dir), '--worker']
        (scratch / 'command.json').write_text(json.dumps(dict(command=command, evidence_cap=12_000_000_000,
            logs_cap=1_000_000_000, environment={k: environment[k] for k in ('TMPDIR', 'ROS_HOME')}), indent=2))
        worker = subprocess.Popen(command, cwd=WS, env=environment, start_new_session=True)
        checks = (scratch / 'storage_checks.jsonl').open('w')
        next_check = 0.0
        interrupted = False
        requested_signal = []
        previous = {}
        def request_stop(signum, _frame):
            requested_signal.append(signum)
        def forward_once(signum, reason):
            nonlocal interrupted
            if interrupted:
                return
            interrupted = True
            checks.write(json.dumps(dict(mono=time.monotonic(), wall=time.time(),
                action=reason, signal=signum, worker=worker.pid)) + '\n')
            checks.flush()
            try:
                os.kill(worker.pid, signum)
            except ProcessLookupError:
                pass
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, request_stop)
        try:
            while worker.poll() is None:
                now = time.monotonic()
                if requested_signal:
                    forward_once(requested_signal[0], 'forward outer interruption to owned runner')
                if now >= next_check:
                    usage = storage_budget.usage()
                    row = dict(mono=now, wall=time.time(), usage=usage)
                    near_cap = any(v['logs'] >= 950_000_000 or v['evidence'] >= 11_500_000_000 for v in usage.values())
                    checks.write(json.dumps(row) + '\n')
                    checks.flush()
                    if near_cap:
                        forward_once(signal.SIGTERM, 'storage reserve reached; stop owned runner')
                    next_check = now + (2.0 if near_cap else 10.0)
                time.sleep(0.2)
        finally:
            if worker.poll() is None:
                forward_once(signal.SIGTERM, 'outer cleanup; stop owned runner')
                while worker.poll() is None:
                    time.sleep(0.2)
            checks.write(json.dumps(dict(mono=time.monotonic(), action='worker reaped', exit=worker.returncode)) + '\n')
            checks.close()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
        return worker.returncode if not interrupted else 1
    os.environ.pop('GZ_PARTITION', None)
    config = build_run_config(run_dir, 231, workspace=WS, max_runtime_sec=900)
    recorder = list(config.recorder)
    recorder[3:3] = ['--compression-mode', 'message', '--compression-format', 'zstd',
                     '--compression-threads', '1', '--compression-queue-size', '0',
                     '--max-bag-size', '25000000', '--qos-profile-overrides-path',
                     str(WS / '.ros_logs/hospital_tools/qos_overrides.yaml')]
    recorder.append('/amr/sensors/merged_lidar/scan')
    config = replace(config, recorder=tuple(recorder), environment={
        'ROS_HOME': str(scratch / 'ros_home'), 'TMPDIR': str(scratch / 'tmp'),
        'PYTHONDONTWRITEBYTECODE': '1'})
    result = run_aws_exploration(config)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result.get('pass') else 1


if __name__ == '__main__':
    sys.exit(main())
