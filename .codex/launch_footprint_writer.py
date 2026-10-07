import json
import os
from pathlib import Path
import signal
import subprocess
import threading

root = Path.cwd()
e = Path((root / '.codex/current_footprint_evidence.txt').read_text().strip())
packet = root / '.codex/NATIVE45_FOOTPRINT_CORRECTION_20261007.md'
assert packet.is_file(), 'Missing release packet'
prompt = f'''Exact Claude Sonnet5.5/medium is sole source writer. Read AGENTS.md,
SESSION_HANDOFF.md and {packet}. Latest user resumes ONLY finish footprint first.
Implement only that packet in src/amr_navigation/test/replay/production_precision_replay.cpp.
Do NOT fix incomplete smoothing or planner-map mutations; they remain pending.
Before editing git status, read exact current source and compare E/before hash.
Separate ±0.02m plugin/scene consistency allowance from unchanged stance/scene
shape/cost gates. Conservative collision geometry must enclose expected and actual
plugin footprints from both phase fixtures. Report deltas and focused checks.
Writer executes NO Python, compilation/build, self-test, CTest, replay or simulation.
Bash only read-only status/source/header/hash/diff inspection; use Edit/Write source.
No shell source mutations, dependencies, outside-workspace modifications, credential
access, protected artifact/handoff/ledger/config changes, commit/push or deletions.
Any material ambiguity/denial/hash mismatch HOLD immediately. Do not retry checks.
Preserve unrelated dirty work and all evidence. Save scoped before/after diff and
implementation_report.md in E; report actual commands, hashes and UNVERIFIED status,
then HOLD. Root checks and independent review follow; no model substitution.
Evidence root E: {e}
Before image: {e / 'before/production_precision_replay.cpp'}
'''
(e / 'prompt.txt').write_text(prompt)
argv = ['/home/pete/.local/bin/claude', '--safe-mode', '--model', 'claude-sonnet-5-5',
        '--effort', 'medium', '--no-session-persistence', '--tools', 'Read,Edit,Write,Bash',
        '--allowedTools', 'Read,Edit,Write,Bash', '--permission-mode', 'dontAsk',
        '--strict-mcp-config', '--mcp-config', '{"mcpServers":{}}', '--setting-sources', '',
        '--output-format', 'stream-json', '--verbose', '--print', prompt]
(e / 'command.json').write_text(json.dumps({'argv': argv[:-1] + ['<prompt.txt>'],
                                          'model': 'claude-sonnet-5-5', 'effort': 'medium'}, indent=2))
creds = json.loads((Path('/home/pete/.claude/.credentials.json')).read_text())['claudeAiOauth']
env = os.environ.copy()
env.update(CLAUDE_CODE_OAUTH_TOKEN=creds['accessToken'], CLAUDE_CONFIG_DIR=str(e / 'config'),
           XDG_CACHE_HOME=str(e / 'cache'), TMPDIR=str(e / 'tmp'), ROS_DOMAIN_ID='232')
with (e / 'worker.stderr.log').open('w') as err:
    proc = subprocess.Popen(argv, cwd=root, env=env, stdout=subprocess.PIPE,
                            stderr=err, text=True, start_new_session=True)
    (e / 'process.json').write_text(json.dumps({'pid': proc.pid, 'pgid': proc.pid, 'root': str(e)}))
    (e / 'run.status').write_text('running\n')
    def stop():
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    timer = threading.Timer(1800, stop)
    timer.start()
    try:
        with (e / 'worker_stream.jsonl').open('w') as out:
            size = 0
            for line in proc.stdout:
                for key in ['accessToken', 'refreshToken']:
                    value = creds.get(key)
                    if value:
                        line = line.replace(value, '<redacted>')
                size += len(line.encode())
                if size > 20000000:
                    stop()
                    break
                out.write(line)
                out.flush()
        rc = proc.wait()
    finally:
        timer.cancel()
    (e / 'run.status').write_text(str(rc) + '\n')
    print('Footprint writer exit:', rc)
