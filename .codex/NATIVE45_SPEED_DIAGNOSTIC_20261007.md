# Native45 speed diagnosis — bounded startup-only packet, awaiting root release

User resumed the speed investigation. Exact `gpt-6.1-sol`/high plans; exact
`gpt-6-luna`/max writes diagnostic tooling only; a separate non-author Sol/high
reviews. Root executes all checks/runtime after writer HOLD and review. No model
substitution. This packet releases no production fix, full46 run or Native45 retry.

## Objective, evidence and hypothesis

Collect the missing contemporaneous resource evidence during one unchanged
startup/runtime-preflight sequence, distinguishing owned simulator CPU work from
scheduler contention. Cause remains UNKNOWN. Native45 stopped before any product,
bag or home operation; preserve its failure and accepted code/clearance results.

Retained45: `phase14_evidence/factory_full_validation_20261003_45/evidence/`:
runtime preflight12s/1785samples, medianRTF0.542964161104,
sim-span5.946666072s/real-span11.988736581s=0.496021080438.
Required default median and aggregate floors remain0.90. Native44 same gate passed
median1.000012150149/aggregate0.994488293329. Host/startup clock-costmap passed;
DRM devices were open, no forced software renderer; owned teardown/lifecycle
integrity passed. Device access does not prove GPU utilization or renderer speed.
No CPU/scheduler/GPU telemetry was captured then. Current host readings are
availability checks, not retrospective cause evidence.

H1 UNTESTED: runnable owned simulator threads lose CPU time to contention or quota;
prediction: lowRTF overlaps increasing per-thread runnable wait, host CPU pressure
and/or cgroup throttling. Absence of those counters while a hot owned thread
executes nearly continuously weakens H1.
H2 UNTESTED: owned simulator/render workload consumes available execution time;
prediction: lowRTF overlaps sustained hot-thread CPU execution with little runnable
wait/global pressure. Low owned CPU together with high waiting weakens H2.
GPU engine deltas, frequencies/throttle flags and memory/IO pressure can narrow
alternatives. Missing GPU fields, a busy GPU or correlation alone do not identify
a causal subsystem. Do not force a diagnosis when evidence is insufficient.

## Exact scope and immutable inputs

Set `AMR_WS=/home/pete/amr_ws` once; derive all paths. Exclusive diagnostic identity:

`native45_speed_diagnostic_20261007T101602Z`

`E="$AMR_WS/phase14_evidence/stability_20261003/native45_speed_diagnostic_20261007T101602Z"`

This directory was absent at planning. Root rechecks absence/non-symlink before
authorizing its initial creation. Writer creates ONLY:

- `E/tooling/speed_diagnostic.py` (import-safe driver and pure contract mode);
- `E/implementation_report.md` and bounded source-diff/hash/command receipts.

Root-owned future `E/run` must remain absent through writer work and offline checks.
Writer does not execute Python, checks, subprocesses, ROS, signals or simulation.
No production/source/test/config, historical evidence, handoff, ledger or protected
`AMR_CODEX_HANDOFF.md` edits. Preserve unrelated dirty work, all-model210/Luna109
and historical authorship/failures. No dependency install, privilege, system
configuration, outside-workspace write, decoder, profiler framework or GUI setting.

Read actual source before implementing:

- original34 `phase14_evidence/factory_full_validation_20261003_34/owned_run_source.py`,
  SHA256 `b5ba2ceba2c15d8de1264134b063e8a4599edc10a49d097a5bfc730f672e9652`;
- lifecycle launcher `phase14_evidence/stability_20261003/startup_transition_probe/diagnostic_launch.py`,
  `63386c54e3ff56cc5113b13e9112ad192dc9e5334ea82e78d03970495f97a444`;
- preflight source/install script,
  `98bce036347073fdfc4425bea23976ab0f38c932234f1427d84882ebec7aca2b`;
- source/install `amr_factory/launch/factory_autonomous.launch.py`,
  `048bf9a627b704e06da00315c6b29c6726604411ce300baf69dc43c7cbc4e2af`;
- source/install `amr_factory/launch/factory_localization.launch.py`,
  `2ed17bc1de92c7ebc1e5b93f7cd5ce6e899ffb8a368fba6ac85aa863e2c4dcd2`;
- accepted45 wrapper/source_preview,
  `b6207c6e5f1a555b91584812a3049d92b2b41fc7c5088d6c8e7226c5e305110b` /
  `2e3d4d0a073c6b75d3416e320bebafc61de213fe94bcb073fca4fc6b9114ac88`;
- source/install factory world,
  `3f06d8a93c61a305e1321d8577b9951ac7237f4b7935cb1602a4aac590207582`;
- source/install amr URDF,
  `68007e20408effe5e14ece8f0cb516196c2ab75da3dd689bd89751c30c85d709`;
- source/install phase14 URDF,
  `5e27f005787297b7f0fc3d1f61012c152eaec27413681d0bfe166c9a71bd621c`.

Preserve/recheck the65 accepted source/input pins in prep45 `input_pins.json`
(`inputs_after`) and readiness `supplemental_ab_input_pins.json`
(`inputs_before_launch`), under the existing readiness evidence root:
`phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/native45_readiness_20261007T094501Z`.
Its `native45_launch.command.json` is the exact runtime environment authority.

## Driver sequence and ownership

Standard-library imports and definitions only at import. Put ROS imports, directory
creation, live reads, threads, processes and signals behind explicit `--run`.
`--check-contract` is mutually exclusive with `--run`, reads pinned inputs and
checks pure command/parser/math contracts; it never creates E/run or uses ROS,
live telemetry, threads or subprocesses. Derive workspace through the known E
location/explicit environment, avoiding an unbounded upward search.

Runtime steps, once, in order:

1. Validate pins, E/run absence including symlink, exact domain232 and owned env.
   Fresh canonical `storage_budget.check(reserve=132008192)` must pass BEFORE
   creating E/run. Create only declared directories within E/run. Set
   `AMR_RUN_ID` to the diagnostic identity, `GZ_PARTITION='amr_'+identity`,
   `AMR_LIFECYCLE_AUDIT=E/run/lifecycle_audit.jsonl`,
   `ROS_LOG_DIR=E/run`, `TMPDIR=E/run/tmp`, matching the original owned wrapper's
   runtime reassignment. Preserve all other accepted45 environment values.
2. Run the unchanged installed host command with a60s owner timeout:
   `ros2 run amr_factory factory_runtime_preflight.py host --evidence-dir E/run/evidence`.
   Require exit0, no forced software, no known simulation process, render device.
3. Start the1s telemetry sampler BEFORE starting the factory. Obtain a timestamped
   baseline sample, then start exactly the original factory command, in its own
   process session:
   `python3 -B <startup_transition_probe/diagnostic_launch.py>
   headless:=false software_rendering:=false require_hardware_rendering:=true
   factory_attachment:=true control_mode:=autonomous
   initial_x:=-4.5 initial_y:=-1.5 initial_yaw:=0.0`.
4. Preserve original34 lines215–239 observer behavior: advancing positive /clock
   AND nonempty positive-width `/amr/global_costmap/costmap_raw`, identical QoS,
  60s deadline and factory-liveness check,0.2s spin. Destroy observer/shutdown
   rclpy in finally even on failure. Record monotonic start/end of this gate.
5. Invoke exactly once, without profile override or modified capture duration:
   `ros2 run amr_factory factory_runtime_preflight.py runtime --evidence-dir E/run/evidence`.
   The installed tool captures12s and enforces BOTH0.90 floors. Use the original
   gate's60s owner timeout; do not confuse its internal22s capture timeout with
   the owner bound. Record monotonic immediately before launch and after exit,
   and retain raw stats, stderr, command and exit. Keep telemetry active until
   return, flush the final sample, then proceed directly to cleanup on PASS or FAIL.
6. Finally always stop/join the sampler, terminate/reap the owned runtime gate
   if still alive, and perform the original owned factory teardown protocol:
   SIGINT to launched process, wait15s; owned group SIGTERM/wait5s then
   SIGKILL/wait5s if necessary. Sweep only proven own survivors with the original
   SIGINT→SIGTERM→SIGKILL,5s bounded waits. Validate lifecycle integrity using
   original34 `validate_audit`/`record_audit_integrity` semantics after launcher
   exit, including start/end, identities/rear perception and980000byte cap.
   Record factory/gate exit codes, survivors, signals and audit outcome.

Copy only the required small helpers/observer/cleanup semantics from original34;
do not import/execute its top-level body or run the full45 wrapper. Own-process
proof requires current UID, exact AMR_RUN_ID environment token and PID starttime;
revalidate before individual signals. Retain session/PGID for launched children;
never signal a reused PID or an unrelated group. Parse `/proc/PID/stat` by the
closing comm parenthesis, not whitespace field positions across a spaced comm.
No MoveIt stage, recorder, factory/product CLI, final observer, home request or
mission action. Startup's original bootstrap behavior remains unchanged.

## Bounded telemetry, availability and interpretation

One sampler, monotonic deadlines1s apart; no catch-up burst. At each sample record
`monotonic_ns`, `time_ns`, actual collection duration and current stage. These
timestamps bracket host→startup→runtime→cleanup; do not claim acquisition-time
alignment inside `gz topic` beyond the retained stage bounds/monotonic sequence.
Sampling begins before factory launch, includes the whole runtime invocation,
and ends at first gate failure/return; it never delays a failed mandatory stop.

Collect a small record from readable files:

- Host `/proc/stat` CPU counters/runqueue, `/proc/loadavg`, selected `/proc/meminfo`
  (MemAvailable/SwapFree), `/proc/pressure/{cpu,memory,io}` cumulative totals.
  Record CPU count, affinity and SC_CLK_TCK once. Derive deltas with actual elapsed
  time; distinguish iowait/steal from executing CPU. Do not add guest counters
  twice. No fixed new pressure/saturation acceptance threshold.
- Discover current-user processes bearing the exact diagnostic AMR_RUN_ID;
  retain only PID/starttime/PPID/comm (bounded64chars), roles and whitelisted
  numeric stat/status/schedstat fields. Never store full environments/cmdlines or
  secrets. Cap at64 owned processes; exceeding scope is diagnostic failure.
  Record utime/stime, RSS/thread count/context-switch counters and scheduler
  cumulative execution/wait. Different starttime or regressed counter invalidates
  that interval; process disappearance is a recorded exit, not zero usage.
- For the owned Gazebo server/GUI only, read per-thread stat/schedstat (cap256
  threads each) so a saturated single thread is not hidden by16-core averages.
  Keep all deltas in memory but emit aggregate execution/wait and top8 CPU plus
  top8 wait threads per process, deduplicated by TID/starttime. Do not sum leader
  process CPU with its thread CPU a second time. The limit must be explicit;
  do not silently truncate and claim complete attribution.
- Resolve each owned Gazebo `/proc/PID/cgroup` against actual cgroup mount/root
  metadata; read applicable `cpu.max`, `cpu.stat`, pressure and memory.events
  where available. Record exact resolved path/unavailable reason. Current host
  cgroup `cpu.max` absence and nr_throttled0 do not describe the future process.
  No cgroup writes or quota changes.
- Read owned Gazebo DRM fd symlinks/fdinfo only. Actual i915 availability proved
  `drm-client-id`, `drm-pdev`, cumulative `drm-engine-{render,copy,video,video-enhance}`
  in ns. Deduplicate shared descriptors/clients by(pdev,client-id,engine), including
  duplicates across processes; do not sum the same client's counter repeatedly.
  Compute client-engine deltas/elapsed interval; apply reported engine capacity
  only when present. Record resets/unavailable, never infer GPU busy from openFD.
- Contextual read-only Intel paths: `/sys/class/drm/card1/gt_cur_freq_mhz`,
  `/sys/class/drm/card1/gt/gt0/{rps_cur_freq_mhz,rps_act_freq_mhz,rc6_residency_ms}`
  and readable `throttle_reason_*`. No generic gpu_busy_percent was available.
  Frequency/RC6/throttle data are context, not proof of which work causes slowdown.

Optional GPU/cgroup/task counters may be unavailable; name that limitation and
leave corresponding diagnosis UNKNOWN. Essential host/process CPU data absent,
sampler exception, cap violation or identity loss invalidates the diagnostic and
stops runtime through main's finally. Keep optional read exceptions isolated;
thread failure must be delivered to the driver, never silently ignored.

Output: `E/run/telemetry.jsonl`≤2MB,≤130samples; record bound16KB/sample. If
collection overhead exceeds100ms in any sample, mark attribution compromised
and stop, rather than silently perturbing the next sample schedule. This is a
diagnostic overhead guard, not a changed product acceptance gate. No arbitrary
runtime warm-up/wait/retry. Host/startup/runtime owner limits plus bounded cleanup
give a finite run; a sampler deadline/cap cannot extend any stage.

All E/run artifacts, INCLUDING launcher-created gazebo_runtime caches/logs, must
stay below10000000 logical AND allocated bytes. Retained45 Gazebo cache was~3.1MB.
Check sizes at1s cadence and all controlled writes; use per-file bounds without
applying a process-wide file-size limit that changes Gazebo behavior. At cap,
stop observation/runtime and retain only reserved bounded failure/cleanup receipts.
Reserve128KB inside the10MB limit for final result/teardown; cap normal observation
growth at9872000bytes. Never truncate/delete old evidence or continue after cap.
Global logs/evidence each remain<1000000000 logical AND allocated. Fresh canonical
132008192reserve before launch; canonical checks before/after major phases and
existing50MB runtime headroom checks at2s cadence; no storage reclamation.

## Writer HOLD; root checks and runtime release

Writer runs status/read/hash/diff commands only, writes scoped tooling/report,
then HOLD. Report actual changes, immutable pins, complete new-file diff, assumptions,
unverified checks and no-execution status. Unknown mechanism permits this diagnostic
tooling, never a guessed production patch.

Root sequential checks, fresh captured statuses, first unexpected failure stops:

1. Exact scope/diff/whitespace/pins and E/run absent.
2. In-memory `compile()` of driver bytes (`python3 -B`), then import under a
   temporary patched subprocess/thread/ROS entry guard that rejects execution.
   Verify import creates no E/run and standard-library-only import behavior.
3. `python3 -B "$E/tooling/speed_diagnostic.py" --check-contract` exits0.
   Compare its command manifest with the actual immutable original34 host/factory/
   runtime argv, changing only evidence path/diagnostic identity. Require12s/default
  0.90 preflight pins,60s startup,1s sampler, bounds/cleanup, no later native gates.
   Pure parser/math cases must exercise spaced comm+starttime reuse, CPU seconds
   versus cores, scheduler-ns deltas, counter regression, and duplicate DRM clients
   (e.g. twoFDs of one client produce one100ms engine delta, not200ms).
   These are written diagnostic contracts; no live process/test simulation.
4. Separate non-author Sol/high reviews complete tooling, contracts and exact
   environment/ownership/finally/cleanup/overhead boundaries. Root records release.

Root launches via clean subprocess environment copied from retained45
`native45_launch.command.json`, same USER/PATH/LANG/GUI/Xauthority/userbus and
workspace-owned prep43/owned_runtime_home HOME/ROS_HOME/XDG paths. Source Humble
then workspace without nounset, domain232/-B/no-bytecode/core0. Root retains
argv/env/pins/handles and actual statuses in E-owned evidence; no host/home writes.
Root adds explicit `AMR_WS` and `E` environment values for the quoted launcher
below. The only changed entry point is this driver, then its declared run-owned
overrides. Keep one script with the required small original34 helpers and observer;
do not introduce a general telemetry or process-management framework.

```bash
# Root resolves E and AMR_WS, copies exact retained45 clean env, checks reserve/pins.
bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash && source "$AMR_WS/install/setup.bash" && ulimit -c 0 && exec python3 -B "$E/tooling/speed_diagnostic.py" --run'
```

Driver must separately report `runtime_gate_passed`, `diagnostic_evidence_valid`,
`teardown_passed` and `lifecycle_integrity_passed`; never turn an RTF failure into
PASS because telemetry was captured. Exit1 on any mandatory gate failure even
with valid diagnostic evidence; exit0 requires all declared gates and teardown
passed. Diagnostic exit0 still provides no A/B/home or full runtime acceptance.
Root retains first failed gate and analyzes telemetry once; no second capture here.

Stop on pin/scope/model mismatch, hostile path/symlink, storage/cap failure,
sampler essential-data failure, startup/factory loss, first mandatory host/runtime
failure or user stop. Cleanup/final receipts still run. Report≤60lines: commands,
environment/status/handles, gate measurements, ownership/audit/teardown, telemetry
coverage/overhead, observed CPU execution versus wait/pressure, GPU limitations,
hypotheses supported/falsified/UNKNOWN and safe resume. No threshold change or
production fix until the slowdown mechanism is established.
