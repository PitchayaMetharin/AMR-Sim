# Native45 diagnostic tooling correction — awaiting root release

Exact `gpt-6.1-sol`/high diagnosis/packet; exact `gpt-6-luna`/max writer;
separate non-author Sol/high full-source review. Root alone executes checks/runtime.
Writer edits/reports/HOLD. This corrects rejected diagnostic tooling, not the
UNKNOWN simulator slowdown. No runtime has executed this harness.

## Objective and evidence

Make the existing single startup-only diagnostic fulfill the released
`.codex/NATIVE45_SPEED_DIAGNOSTIC_20261007.md` contract. Preserve its identity,
environment, argv, original34 startup/lifecycle/owned teardown and unchanged
default12s preflight/BOTH0.90 floors. Root and independent full-source review
returned SPEC FAIL / QUALITY FAIL. HIGH confidence in deterministic harness
mechanisms; simulator cause remains UNKNOWN.

Set `AMR_WS=/home/pete/amr_ws` once; derive paths:

```bash
E="$AMR_WS/phase14_evidence/stability_20261003/native45_speed_diagnostic_20261007T101602Z"
C="$E/correction_20261007"
S="$E/tooling/speed_diagnostic.py"
```

Rejected source SHA256:
`89ca884094546a1dbef9a77fadfa46a3b8d8be4af0a0bce64dea62f57113d743`.
Source/control-flow evidence on that exact artifact:

| Boundary | Observed mechanism |
|---|---|
| run state938 → host1378 | timestamps exist in result, absent in state; first host timestamp raises KeyError before Popen |
| identity462–471 | raw-byte substring accepts wrong variable name/value suffix; cleanup shares predicate |
| cleanup1190–1244,1514–1588 | wait/finish call normal-cap guards; survivor scan calls essential CPU telemetry; those failures abort signal/reap/sweep |
| role473–490 | executable/comm whitelist rejects installed Ruby-hosted `gz sim`; Gazebo-specific collection is skipped |
| DRM174–195,754–767,1320 | capacity key matches broad engine regex first; flattened records are fed again to a raw-record deduper, losing counters |
| runtime851–867,1495 | parser first demands PASS, so failed report/measurements are discarded before retained gate status |
| sampling1323–1355,1615 | timing ends before sysfs/serialization/budget/write; validity requires merely two samples; no forced final runtime sample |
| process499–506/thread637–643/DRM757 | ordinary disappearance raises essential-data failure; optional fd-directory failure escapes; invalid thread deltas are omitted |
| terminal1652–1687 | prior failure is absent from success conjunction; receipt/postwrite failure can leave on-disk success contradicting exit1 |
| budgets69,989–1029,1630 | per-file cap constant unused; final canonical check absent; repeated dictionaries have no realistic16KB proof |

Installed consumer proof (read-only, no Gazebo execution): factory_localization
lines42–43/169/174 launches separate `gz sim -s` and `gz sim -g`. `/usr/bin/gz`
is Ruby, loads `/usr/share/gz/sim8.yaml` → `/usr/lib/ruby/gz/cmdsim8.rb`, sets
process title `gz <argv>`, and calls Cmd.execute. cmdsim8.rb loads
`libgz-sim8-gz.so.8.15.0`; lines593–614 directly call Importer.runServer/runGui
inside Ruby, not an exec of gzserver/gzclient. Combined mode539/561 sets titles
`gz sim server/gui`. Preserve actual command/title forms; tokenized argv alone
is insufficient after setproctitle replaces it. Pins:
gz `863c95b52d3d60a23b8e7f542d5ca71f4d783ce617989742eb0d63d85e3578cc`;
cmdsim8.rb `7c8d12f4bdfb016a5b7bd1c2b8600e497e23267f1a64cd480d7e2024b0b597ba`;
sim8.yaml `a6b7d809febb79ef22f8322e6709b39a168904a0d3bfb26374fc30df55c1b448`.

Retained45 factory.log has38 starts and9 clean exits before the last start:
29 launch processes remain, plus launcher/gate descendants. This proves compact
serialization needs a real size check; it does not claim a measured cap breach.

## Allowed edits and preservation

Only `$S` source may change. Writer may create exclusive `$C` bounded receipts:
`before/speed_diagnostic.py`, `before.sha256`, `scoped.diff`, `after.sha256`,
`implementation_report.md`, `command_receipts.md`. Root later owns `$C/checks/`.
Root rechecks C absence/non-symlink and E/run absence/non-symlink before release.
Before edits writer checks status/current source/hash, saves exact failed before
image, verifies its hash, then patches from current source. Do not overwrite E's
original implementation_report/diff/checksums/command receipts or original packet.

Existing unrelated dirty work is broad; this author changed only this packet.
Canonical reserve1MB passed before packet write; logs/evidence allocated
654848000/826138624. Runtime still requires fresh132008192 reserve. Preserve
all65 prep/supplemental inputs, protected artifact/ledgers and counters210/109.
No production/test/config/handoff/ledger edits, new simulation identity, native46,
build/replay/extraction, install, system changes, GUI/physics change or outside write.

## Required bounded changes

1. Use one initialized timestamp mapping in result and the live path. Remove the
   stale state accesses or deliberately alias the same mapping. Preserve exact
   host/startup/runtime commands and deadlines; no artificial warmup/retry.
2. Exact ownership is `OWNED_ENV_TOKEN in raw_environ.split(b'\0')`, with UID
   and stable starttime proof. Reject prefix/suffix/different-key matches and
   reused PID. Use this same predicate for discovery and every signal boundary.
3. Cleanup must not depend on CPU/schedstat/DRM/normal-cap/receipt success.
   Separate minimal survivor enumeration using only UID/exact token/stat starttime
   and liveness. Revalidate individual identity; group signals require existing
   exclusive ownership proof. Ordinary exits are normal, uncertain identity is
   never signaled and is reported. Signal/wait/reap phases always continue despite
   observation/receipt errors, accumulating bounded errors in memory. Remove cap
   checks from cleanup waits; defer receipts until signal/reap completes. Preserve
   INT15s→TERM5s→KILL5s factory and owned survivor sweep5s per phase, lifecycle
   integrity and post-sweep proof. Failed enumeration cannot count as survivors[].
4. Recognize actual owned Ruby-hosted server/GUI commands and rewritten titles,
   legacy `ign gazebo` counterparts, and existing native gzserver/gzclient forms.
   Bound transient cmdline reads; retain only role/identity, never full cmdline/env.
   A plain Ruby process or `gz topic` is not Gazebo. Track stable server/GUI
   identities and assert both have usable per-thread CPU coverage throughout the
   runtime interval; missing roles must invalidate evidence, not silently pass.
5. Keep raw fdinfo records until one cross-process dedupe; `_read_drm` returns
   raw records plus explicit availability, collector normalizes once. Parse
   capacity before broad engine regex. Same(pdev,client,engine) duplicates count
   once, max cumulative value, independent clients separate; retain resets.
6. Parse and retain complete runtime report/status before deciding gate acceptance.
   Failed12s/default/0.90 measurement reports remain diagnostic evidence, with
   runtime_gate_passed=false and exit1. Report malformed/missing data separately.
   Never reinterpret installed FAIL as PASS or continue beyond it.
7. Measure full observer overhead through sysfs reads, serialization, budgeting,
   append/flush and bookkeeping, using monotonic timestamps. Store completed total
   duration in bounded sampler/result records (a next-sample timing entry plus final
   completion record is acceptable); enforce existing100ms on that duration.
   On normal runtime return, stop/join without race and collect one serialized
   final sample immediately at the terminal boundary before teardown, including
   FAIL; no sleep/second capture. Unexpected sampler failure skips further unsafe
   collection and invalidates coverage. Record baseline before factory, runtime
   start/end, all sample start/end and final sample marker. Evidence validity
   requires runtime invocation/report, prelaunch baseline, whole invocation bracket,
   no unexplained missed1s slots (allow measured≤100ms collection/poll jitter),
   final sample after runtime end, stable server+GUI and usable thread deltas,
   all guards and no sampler/identity/coverage failure. Two lines alone never pass.
8. Treat ENOENT/ESRCH disappearance between discovery/reads as explicit exit/churn,
   rechecking identity to distinguish a still-live owned essential-data failure.
   Tasks disappearing are recorded, not assigned zero usage; PID/TID reuse or
   regressed counters invalidate intervals. Thread summaries expose invalid/new/
   exited counts, aggregate only valid matched identities; topCPU/topwait union
   emitted once with rankings/flags, not two duplicated dictionaries. Isolate all
   optional DRM/cgroup/sysfs access errors and report unavailable reason. Missing
   optional GPU fields cannot fail otherwise complete CPU evidence. Missing live
   essential CPU data remains fail-closed.
9. One sticky terminal failure determines exit1 and success=false, including late
   cleanup/pin/storage/receipt errors; validity may remain true for a completed
   measured RTF FAIL, but success may never be recomputed true. Validate final
   state/pins/storage before publication, bounded reserve-aware final receipts,
   and final canonical check after all run artifacts. Any late failure must leave
   a consistent failed terminal receipt and exit1; no stale result success=true.
   A receipt write problem cannot interrupt teardown. Keep the128KB reserved final
   region genuinely usable without applying NORMAL_RUN_BYTES to final writes;
   total logical+allocated remains≤10MB. Failure to publish is explicit terminal
   failure with bounded stderr/owner status, not a claimed successful receipt.
10. Enforce existing per-file limits on actual controlled writes and1s scans,
    including child-produced files, in addition to aggregate checks. Keep normal
    headroom/reserved distinction correct, MAX_NORMAL_FILE4MB, telemetry2MB,
    sample16384bytes,130samples,1s/100ms guards and global1GB caps unchanged.
    Compact repeated process/thread metadata using one explicit schema/catalog
    and numeric rows, retaining all64 possible owned identities/counters and the
    two Gazebo processes' deduped top8CPU+top8wait. Share cgroup data by resolved
    path. A bounded catalog inside E/run is permitted, with declared per-file cap,
    aggregate accounting and completion coverage. No silent truncation or larger
    caps. Avoid new generic frameworks; simplify only defective duplicated paths.

## Prediction and root-only offline validation

Prediction: actual consumers reject ownership impostors; actual Ruby Gazebo roles
are covered; one client produces one engine delta; churn/optional absence is
recorded; injected CPU/cap/receipt failures still reach all teardown phases and
terminal FAIL; subfloor measurement survives in failed result; full sample size
and interval/overhead contracts pass without changing launch or thresholds.

Extend existing pure `--check-contract` with bounded fixtures through shared live
consumer seams (narrow injected reads/clock/process doubles, no parallel framework).
Use small shared seams for stage timestamps, cleanup phases and terminal
publication, with narrow standard-library doubles. Do not duplicate the runtime
algorithm, invoke a fake full run, or virtualize the whole filesystem or ROS.
No fixture may launch, signal, read live /proc, start threads, create E/run or use
ROS. Root must compare independent fixture expectations, not test-only predicates.
Provide optional `--baseline-source "$C/before/speed_diagnostic.py"` in contract
mode only: execute the same consumer assertions against import-safe preserved
source where interfaces exist; missing corrected consumer/coverage is failure,
not SKIP/PASS. Root retains group-specific old failures. The four required baseline
failures below must use existing old consumers with narrowly injected reads and
the raw retained original45 runtime report, not missing new symbols or a new CLI.
At least exact-token,
Ruby-role, DRM collection and failed-report retention must fail old for the stated
mechanism and pass corrected, not simply fail due to a new absent CLI/function.

Required fixture groups: host-prefix reaches intended mocked Popen without
KeyError; exact env positive/negative and reused starttime; Ruby actual server/gui
titles versus ruby/topic negatives; raw `_read_drm`→collector path duplicated FDs
acrossprocesses+capacity2+reset; failed original45 report retained; scheduler/task
delta/new/exit/regression and top union; complete/incomplete runtime coverage and
full100ms overhead including delayed optional read/write;64 realistic process
rows+two16-unique-top-thread Gazebo summaries+cgroup/GPU/host serialize≤16384
using the actual serializer; cap/CPU/sampler/receipt faults exercise real finally,
signals/reap/postaudit via safe doubles, wrong identity receives no signal, sticky
terminal failure survives otherwise passing stages. Serialization fixture must
include long actual-style names, realistic large counters and declared catalog;
report bytes rather than asserting only process count.

Root commands after writer HOLD, under original owned check environment/domain232
with `PYTHONDONTWRITEBYTECODE=1`, C-owned outputs and no E/run:

```bash
test ! -e "$E/run" && test ! -L "$E/run"
sha256sum "$C/before/speed_diagnostic.py" "$S"
git diff --no-index --check "$C/before/speed_diagnostic.py" "$S"
python3 -B -c 'import os,pathlib; p=pathlib.Path(os.environ["S"]); compile(p.read_bytes(),str(p),"exec")'
python3 -B "$S" --check-contract --baseline-source "$C/before/speed_diagnostic.py"
python3 -B "$S" --check-contract
test ! -e "$E/run" && test ! -L "$E/run"
```

Root exports AMR_WS/E/C/S explicitly. Expected baseline exit1 with asserted
mechanism failures; corrected exit0. No-index diff exit1 means differences,
not automatically a whitespace failure; inspect output. Root additionally repeats
the original import side-effect guard and verifies pins/scope/manifest equivalence.
No CMake/build/production-test run. Separate non-author Sol/high must give full
source SPEC/QUALITY PASS, not merely review fixture output, before root live release.

Live command/environment/storage/startup/runtime/cleanup remain exactly the
original diagnostic packet. Root runs its single existing identity invocation
only after offline discrimination/review PASS; one strict12s capture, first
mandatory failure→owned cleanup. No planner/writer checks or runtime authorized.

Stop on changed failed-source hash, preexisting C/run, pin/model/scope ambiguity,
counterexample or unresolved cleanup/storage contract. Writer reports changed
files, complete scoped diff, pins, no-execution status and remaining uncertainties
in≤60lines, then HOLD. Preserve all failed artifacts; no ledger/counter changes.
