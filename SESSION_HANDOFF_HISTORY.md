# AMR Session Handoff

## Active authorized plan — 2026-09-14 — Portable exploration and canonical AWS warehouse

The user has authorized implementation of a new world-agnostic simulation
entry point while preserving the existing factory product-acceptance launches.
The work must follow the repository workflow one packet at a time:

`Sol/high diagnosis -> Luna/max implementation -> Sol/high independent review`

No Astra delegation is authorized.  `AMR_CODEX_HANDOFF.md` must remain
untouched.  Preserve the untracked `phase14_evidence/` directory and exclude it
from commits.  The final requested commit is
`feat: add portable simulation exploration and canonical AWS world`; pushing to
`origin/main` remains approval-gated by the repository rules.

### Packet 1 — portable exploration core

- Add `amr_simulation/launch/portable_exploration.launch.py` without changing
  `amr_simulation.launch.py`.
- Accept a required absolute trusted local SDF 1.9 world plus finite spawn pose,
  optional local model resource paths, `headless`, `rviz`, and
  `auto_start_exploration` arguments.  Parse the one named world and derive all
  Gazebo bridge namespaces from it.
- Fail closed before launch for malformed/multiple worlds, missing required
  Gazebo systems, unsafe/non-finite physics, a pre-existing `amr` model,
  invalid pose, unresolved `model://` assets, `file://` resources, direct
  remote worlds, or Fuel model URLs without an explicit positive revision.
- Spawn `phase14_mobile_manipulator.urdf.xacro` empty with
  `factory_attachment=false`; start controller spawners only after successful
  insertion and shut down on any required-process failure.
- Add a portable-runtime-only simulation stow authority.  It sends exactly one
  standard arm trajectory `[0, -1.5708, 1.5708, 0, 0, 0]`, uses the existing
  0.01-rad tolerance plus fresh joints/base proof and authority QoS, remains
  `STARTING` with motion denied until proven, admits only `STOWED_EMPTY`,
  immediately revokes stale/drifted proof, and latches `FAULT` on rejection or
  explicit failure without retry.
- Require stowed manipulation in command arbitration and causally stage
  adapters/authority, SLAM/map, planner, controller, mission, then Explorer.
  Readiness has no elapsed-time cap while required processes live, reports an
  unmet condition at least every 60 seconds, and fails immediately on process
  exit, explicit fault, shutdown, or user stop.  Explorer release requires
  fresh map/costmap/TF/base/stow/lifecycle/action evidence and passes
  `auto_start_exploration` through its existing `autostart` parameter.
- Add package dependencies, portable-runtime ownership documentation, and
  failing-first contract/behavior tests for validation, dynamic bridge names,
  stow terminal/freshness/sequence behavior, staged release, process failure,
  autostart override, and the absence of AMCL/map-server.

### Packet 2 — canonical AWS preset

- Package the proven AWS warehouse as `aws_warehouse_world`, with Bucket at
  Fuel revision 3 and ShelfF, WallB, ShelfE, ShelfD, GroundB, Lamp,
  ClutteringA/C/D, TrashCanC, and PalletJackB at revision 4.
- Add `aws_warehouse_exploration.launch.py` as a thin include of the portable
  launch at spawn `(0, 0, 0.12, 0)` and package an RViz view for the SLAM map,
  both costmaps, robot model, TF, pose, and both LiDAR streams.
- Record exact Fuel ownership, revisions, URLs, cache/network requirements,
  and attribution without extending the local MIT-0 asset claim.  Remove all
  repository/runtime dependencies on `/tmp/amr-aws-exploration-preview.*`.
  SLAM Toolbox remains the sole `map -> odom` owner; AMCL stays absent.

### Packet 3 — commands, saving, and closeout

- Update repository simulation documentation and external AWS/factory quick
  commands.  Clarify that a "map" input is a Gazebo SDF environment and every
  launch creates a fresh in-memory SLAM occupancy map.
- Preserve explicit manual saving through the existing validated mapping CLI
  after `COMPLETE` or safe `INCOMPLETE`, using the chosen spawn datum.  Do not
  auto-save, auto-accept, promote, or replace a canonical map.
- Update this handoff only after validation.  Run Python compilation, focused
  pytest, selected package builds/tests, verbose `colcon test-result`,
  `git diff --check`, and installed-package `--show-args` for both launches.
- With runtime authorization, run one fresh generic simple-world launch and one
  fresh AWS+RViz launch.  Acceptance requires automatic `run_generation=1`
  without a start service call, fresh safety authority, growing SLAM map, both
  costmaps, unique SLAM `map -> odom` ownership, at least one successful
  navigation goal, and non-faulted `COMPLETE` or safe `INCOMPLETE`.  Save and
  validate one run-specific AWS candidate without promotion.

### Preserved behavior and stop conditions

The complete arm-equipped AMR remains the portable default with no payload.
Factory acceptance paths, fail-closed gates, ownership boundaries, public
interfaces, thresholds, documented hardware values, cancellation semantics,
and canonical maps remain unchanged unless this plan explicitly says otherwise.
Any failed implementation returns to Sol/high diagnosis before another source
change; timing/runtime contradictions and mandatory-gate failures stop the
packet under the existing two-attempt/three-hypothesis limits.

## Runtime validation checkpoint — 2026-09-15 (in progress)

The first generic attempt, `portable_simple_20260915_02` (ROS domain 230),
failed before acceptance because the required
`portable_exploration_readiness.py` crashed on a Humble
`RcutilsLogger.warning` printf-style `TypeError`.  The launch shut down
fail-closed.  Evidence is preserved under
`.ros_logs/portable_simple_20260915_02`.

Sol/high diagnosed the failure, Luna/max implemented the approved two-file
fix, and Sol/high independently reviewed it as `PASS`.  Focused readiness
validation passed all 9 tests; build, source/install identity, and diff checks
also passed.  Command-only retries found that an empty `resource_paths:=`
argument was rejected and ROS domain 233 was invalid; no simulation started
for either retry.

The current generic run, `portable_simple_20260915_03` (ROS domain 231,
partition `amr_portable_simple_20260915_03`), is still in progress at this
checkpoint with `headless=true`, `rviz=false`, `autostart=true`, and the
optional `resource_paths` argument omitted.  Robot insertion, controllers,
stow authority, and SLAM have started.  Fresh map and single-owner
`TF`/authority evidence has been observed.  Readiness is waiting on fresh
`map->odom` and composed `map->base_footprint` TF.  There is no terminal
verdict, no AWS launch, and no Packet 3B closeout yet.  State-observer evidence
is under `.ros_logs/portable_simple_20260915_03/evidence/`.

`AMR_CODEX_HANDOFF.md` and `phase14_evidence/` remain protected.

## Runtime validation stop checkpoint — 2026-09-15

The generic run `portable_simple_20260915_03` (ROS domain 231,
partition `amr_portable_simple_20260915_03`) was stopped through its owned
launch, observer, and TF-observer handles after repeated readiness reports
left it at `slam_map` with only `fresh map->odom TF` and `fresh composed
map->base_footprint TF` unmet.  No acceptance verdict was claimed.  Evidence
is preserved under `.ros_logs/portable_simple_20260915_03/`.

Sol/high diagnosed a source/configuration compatibility defect: Humble
SLAM Toolbox intentionally future-dates `map->odom` by the configured
`transform_timeout: 1.0`, while portable readiness rejected all future TF
stamps with zero tolerance.  Direct TF and ownership evidence showed fresh
`/amr/slam_toolbox` and `/amr/ekf_filter_node` publishers, so this was not
missing publication or QoS loss.  The approved next packet is limited to the
readiness script and its focused tests, permitting at most the exact 1.0-second
SLAM `map->odom` offset while preserving zero future tolerance elsewhere.

The subsequent Luna/max implementation turn reached its usage limit before
editing; no TF-tolerance source or test change was made from that attempt.
The previously accepted logger correction remains installed and reviewed.
AWS has not been launched, no candidate has been saved, and Packet 3B is not
complete.  `AMR_CODEX_HANDOFF.md` remains untouched and `phase14_evidence/`
remains preserved.

## Runtime observer stop checkpoint — 2026-09-15

The fresh generic run `portable_simple_20260915_04` (ROS domain 231) passed
the corrected readiness stages through Explorer startup and produced fresh
SLAM map, global/local costmap, base, stow, and TF-owner evidence.  The
run-specific observer then failed at its own parsing boundary because Humble
`DiagnosticStatus.level` is byte-valued (`b'\\x00'`) while the observer cast it
with `int()`.  Its exception escaped into a second finalization path, which
also reported a closed-JSONL write error.  No terminal acceptance verdict was
claimed; all evidence is preserved under
`.ros_logs/portable_simple_20260915_04/`.

Sol/high diagnosed the exact observer-only repair: normalize byte-valued
levels, catch unexpected spin exceptions before one finalization, and make a
final JSONL write failure fail closed without a second write.  Two Luna/max
implementation attempts were made after the usage reset but stalled without
editing.  Under the repository two-attempt stop rule, autonomous observer
patching is paused pending explicit user direction.  The source TF-tolerance
fix remains independently reviewed `PASS`.  AWS has not been launched, no
candidate has been saved, and Packet 3B remains incomplete.

`AMR_CODEX_HANDOFF.md` remains untouched and the untracked `phase14_evidence/`
directory remains preserved.

## Runtime observer correction checkpoint — 2026-09-15

The user explicitly authorized a fresh Luna/max observer-fix attempt after the
two-attempt stop.  The run-specific observer artifact was corrected in place
only at `.ros_logs/portable_simple_20260915_01/evidence/` (mode `0775`).
Diagnostic levels now accept Humble's one-byte representation and reject
malformed values fail-closed.  Unexpected spin exceptions are recorded even
when terminal admission had already set `finish_requested`; finalization is
single-path, and final JSONL failures persist a `FAIL` verdict without a
closed-stream retry.

The corrected artifact was independently reviewed by Sol/high as `PASS` with
SHA256
`46daad3bcab151864d6defbbe341376eaa46be78db62c587db720a625c60dc3b`.
Focused self-check, bytecode compilation, and `git diff --check` passed.  No
runtime was launched from this correction yet; the next safe step is a fresh
generic simple-world runtime run using the reviewed observer.  AWS and Packet
3B remain pending, and `AMR_CODEX_HANDOFF.md` plus `phase14_evidence/` remain
protected.

## Generic runtime checkpoint — 2026-09-15 (run 05 stopped for diagnosis)

The fresh generic simple-world run `portable_simple_20260915_05` (ROS domain
231, `headless=true`, `rviz=false`, automatic exploration) reached the
behavioral acceptance evidence with the corrected observer: Explorer
`run_generation=1`, two successful navigation goals, SLAM known cells growing
from 4,921 to 9,032, fresh nonempty global/local costmaps, advancing READY
base status, fresh `STOWED_EMPTY` authority owned by
`/amr/portable_stow_authority`, and exact single-owner
`map->odom`/`odom->base_footprint` TF evidence.  Explorer published a safe
non-faulted `INCOMPLETE` terminal state with no pending goal, empty
`cancel_target`, and zero goal failures.

The run-specific observer then failed to finalize its own verdict: its last
JSONL snapshot contained the terminal state and no failures, but no
`verdict.yaml` was written and the observer remained alive in
`futex_wait_queue` for over three minutes.  The owned launch and probes were
stopped with the exact launch handle; no acceptance verdict is claimed.
Evidence is preserved under `.ros_logs/portable_simple_20260915_05/`.

Sol/high diagnosis is pending for this observer-only terminal-finalization
hang.  No source edits were made from the runtime symptom.  AWS and Packet 3B
remain pending; `AMR_CODEX_HANDOFF.md` remains untouched and
`phase14_evidence/` remains preserved.

## Generic runtime acceptance checkpoint — 2026-09-15 (run 06 PASS)

The corrected observer was rerun against the simple SDF as
`portable_simple_20260915_06` (ROS domain 231, `headless=true`, `rviz=false`,
automatic exploration).  It produced exactly one `PASS` verdict and exited
cleanly after `finish_reason: terminal evidence evaluated`; no
`/amr/exploration/start` call was issued.  The accepted terminal was safe
non-faulted `INCOMPLETE` with `run_generation=1`, `active=false`,
`pending=false`, empty `cancel_target`, and zero goal failures.

Runtime evidence includes two successful NavigateToPose goals, strict SLAM
known-cell growth from 4,921 to 9,063, fresh valid nonempty global and local
costmaps, advancing READY base status, fresh empty stow authority owned by
`/amr/portable_stow_authority`, exactly one `/map` publisher owned by
`/amr/slam_toolbox`, and exact TF ownership
`map->odom=/amr/slam_toolbox` plus
`odom->base_footprint=/amr/ekf_filter_node`.  No AMCL or map-server node was
observed.  Evidence is preserved under
`.ros_logs/portable_simple_20260915_06/` and all runtime processes are
cleaned up.

The generic runtime gate is now accepted.  The next safe step is the fresh
canonical AWS warehouse launch with RViz and the AWS-specific lidar checks;
Packet 3B remains open until that run and manual candidate save/validation
complete.

## AWS runtime startup checkpoint — 2026-09-15 (run 01 stopped for diagnosis)

The first canonical AWS launch `aws_warehouse_20260915_01` (ROS domain 232,
`headless=false`, `rviz=true`) failed closed before robot insertion.  The
canonical-host Fuel bundle was cold: Gazebo downloaded all 12 distinct pinned
model revisions, and the world/create services became available about 100
seconds after Gazebo start.  The concurrent `ros_gz_sim create` request timed
out at about five seconds but returned code 0; the launch consequently started
the joint-state spawner, which timed out after 30 seconds because
`/controller_manager/list_controllers` did not yet exist.  The required
spawner failure shut down the launch.  Network access succeeded and all exact
Fuel model directories are now cached; GUI/RViz was not causal.

Sol/high diagnosed both the cold-cache timing and a real portable-launch gate
defect: create-process return code 0 is not positive insertion proof, so the
spawners are not causally gated on world readiness/control-plugin evidence.
The observer correctly recorded a fail-closed verdict; no AWS acceptance is
claimed.  The next implementation packet is limited to an evidence-based
world/insertion gate (no arbitrary sleep or widened timeout), followed by a
fresh warm-cache AWS runtime.  `AMR_CODEX_HANDOFF.md` remains untouched and
`phase14_evidence/` remains preserved.

## User stop checkpoint — 2026-09-15

The user instructed this session to stop after updating the handoff.  All
runtime processes and probes are stopped.  Generic run 06 remains the accepted
simple-world runtime (`PASS`, safe `INCOMPLETE`); AWS run 01 remains a
fail-closed cold-cache startup failure with all exact Fuel model revisions now
cached.  Sol/high's diagnosis and the proposed causal insertion-gate packet
are recorded above.  Luna/max was interrupted before making any source edits,
so the portable launch still needs that reviewed gate before another AWS run.

No AWS candidate was saved, Packet 3B is incomplete, `AMR_CODEX_HANDOFF.md`
was not modified, and the untracked `phase14_evidence/` directory remains
preserved.  Safe resume point: Sol/high review of the causal insertion-gate
implementation, then a warm-cache canonical AWS+RViz runtime and manual
run-specific save/validate only after a PASS terminal verdict.

## Latest authoritative state — 2026-09-14 — Phase 15 TF fix and autonomous mapping runtime complete

The Phase 15 Explorer TF/readiness correction is implemented, independently
reviewed by Sol/high, and validated in a fresh autonomous mapping runtime.
There are no live simulation, Explorer, SLAM, RViz, or diagnostic-probe
processes. `AMR_CODEX_HANDOFF.md` was not modified. No canonical map was
replaced or promoted.

### Source and offline validation

- `src/amr_exploration/scripts/frontier_explorer.py` now queries
  `map->base_footprint` at the node's exact current ROS time, rejects zero,
  future, stale, and non-finite samples, clusters frontiers before taking the
  carried TF sample, and rechecks that sample at the final reservation gate.
  A stale sample is discarded without sending a goal.
- Readiness has a fresh 15-second budget for each continuous no-motion
  episode. Initial startup still fails closed after startup grace; readiness
  loss during active navigation still cancels immediately. The executor
  worker count was not changed.
- `src/amr_slam/config/mapper.yaml` uses `transform_timeout: 1.0` for SLAM
  scan/TF synchronization. The mapping-specific readiness path remains
  distinct from the generic navigation graph preflight.
- Focused lifecycle/SLAM tests passed with `152 passed, 2 xfailed, 1 xpassed`;
  the selected package build passed for `amr_slam`, `amr_exploration`,
  `amr_factory`, and `amr_bringup`; all nine selected CTest groups passed; and
  `git diff --check` passed.

### Runtime evidence — `phase15_tf_fix_20260914_04`

Evidence is preserved under
`.ros_logs/phase15_tf_fix_20260914_04/factory_mapping/`.

- Host preflight and hardware-rendered Gazebo/RViz startup passed. Runtime
  preflight passed with median RTF `0.9496737443`, aggregate RTF
  `0.8276216795`, and `/dev/dri/renderD128`.
- RViz loaded the SLAM map view and robot/TF displays. Known lidar QoS
  warnings and one GLSL-link warning were logged; this is visualization
  evidence, not a claim of perfect sensor-panel presentation.
- Exactly one authorized start call was made. The original failure point was
  passed: two navigation goals succeeded, a changed-map plan was safely
  discarded back to `SCANNING`, and no goal failures or Explorer fault were
  recorded.
- The run terminated safely as `INCOMPLETE` with reason
  `exploration incomplete: no safe costmap-valid frontier remains`; it had no
  pending/active goal, `cancel_ack=false`, `goal_failures=0`, and
  `fault_latched=false`. This is an accepted safe terminal outcome, not a
  claim that every cell was explored.
- `factory_candidate` was saved at 233x194 cells and validated. The artifact
  bundle SHA256 is
  `cf002ff045d923022b9351ec0e1c90a76c3417141ba78fc8e04dae8c30d3f0fd`.
  Runtime mapping acceptance exited 0 with `verdict: PASS`,
  `timed_out: false`, and no failures. The report records fresh map/TF,
  ownership, manipulator authority, host/runtime preflight, and the terminal
  Explorer state.
- Key evidence files are `evidence/runtime_preflight.txt`,
  `evidence/exploration_terminal_status.yaml`,
  `evidence/runtime_acceptance.yaml`, `mapping_manifest.yaml`, and
  `evidence/factory_candidate_preview.png`.

After acceptance, Ctrl-C produced the known Explorer shutdown traceback
(`KeyboardInterrupt` followed by `rcl_shutdown already called`) and a launch
exit code of 1; all runtime processes were nevertheless cleaned up. Do not
rerun or patch from that cleanup symptom without a new Sol/high diagnosis.
Human map-quality review and any later canonical-map promotion remain outside
this runtime proof.

## Current authoritative state — 2026-09-14 — Phase 15 runtime stop checkpoint

The user instructed this session to stop after updating the session handoff.
There are no live simulation, Explorer, SLAM, or diagnostic-probe processes.
The protected `AMR_CODEX_HANDOFF.md` was not modified during the stopped
runtime continuation; the user later explicitly authorized its synchronization
to this 2026-09-14 state.

The accepted source/offline work remains unchanged. The Phase 15 mapping RTF
profile now uses inclusive floors of `0.80` for both median and aggregate RTF
(`phase15_mapping`); the default profile remains `0.90`. The reviewed mapping
packets also use the live `/amr/slam_toolbox/serialize_map` endpoint and a
direct `/map` Nav2 map-saver invocation with transient-local subscription QoS,
and fail closed before graph serialization or manifest creation on map-save
failure. Focused mapping CLI/demo-contract validation reported 100 passed, the
`amr_factory` build passed, and the source/offline changes were independently
reviewed by Sol/high. No canonical map was changed.

## Phase 15 diagnostic runtime — `phase15_tf_diag_20260914_01`

This fresh, no-recorder runtime used the fixed SLAM overlay, GUI hardware
rendering, `factory_attachment:=true`, autonomous mode, initial pose
`(-4.5, 0, 0)`, and ROS domain 232. Host preflight, Gate 6, adapter/map/
planner/controller/mission readiness, 30-second stabilization telemetry, and
the `phase15_mapping` runtime gate all passed. Runtime evidence recorded
aggregate RTF `0.9990677485`, median `1.0000051000`, 3,596 RTF samples, and
stats probe exit code 0.

After the single authorized exploration start call, the real Explorer reached
navigation and then discarded two plans for carried-TF staleness before
latching `FAULT` at ROS time `173.183316015` with
`timestamped fresh map to base_footprint TF is unavailable`. At that fault,
raw `/tf` still contained `map->odom` at age `0.080 s` and
`odom->base_footprint` at age `0.013 s`; both edges continued publishing.
Independent `tf2_echo map base_footprint` returned successfully around the
fault. This ruled out a missing/future/stale upstream TF graph and supported a
local Explorer lookup/callback-capacity problem, but did not by itself prove
executor starvation versus a latest-sample timing race.

Evidence is preserved under
`.ros_logs/phase15_tf_diag_20260914_01/factory_mapping/evidence/`, including
the raw clock/TF/static-TF/status streams, external lookup output, runtime
preflight, and wall-time start/fault observations. No map save, pose-graph
serialization, artifact validation, promotion, or canonical-map operation was
attempted after the fault. The runtime therefore is diagnostic evidence, not
mapping acceptance.

## Executor hypothesis and paused test-only diagnostic

Sol/high first proposed a narrowly scoped production executor change from two
to three workers, but Luna's required baseline regression passed without
reproducing the selective local-TF failure. Sol/high then withdrew production
authorization: the existing test replaced `_select_frontier()` wholesale,
used synthetic publisher-controlled timing, and did not assert the failure.

The next Sol/high packet was test-only and limited to
`src/amr_exploration/test/test_frontier_lifecycle.py`. It was to exercise the
real `_tick()` to `_select_frontier()` path with a production-sized map/
costmap, real simulated clock semantics, production-compatible publishers,
Explorer-local lookup instrumentation, identical two-/three-worker cases,
and a separate strict future-transform invariant. A second Luna diagnostic
attempt added a fixture/helper, parameterized two-/three-worker case, and
future-TF invariant to that already-untracked test file, then was stopped at
the user's request. Its focused command was interrupted before a reliable
result was produced. Treat that test-only diff as unverified; do not infer a
production fix or acceptance from it.

When work resumes, Sol/high must review the test-only diff/result and
re-diagnose if it does not genuinely distinguish local stale/exception from a
future latest sample. No production Explorer change, runtime retry, artifact
save, promotion, canonical-map replacement, threshold weakening, or scope
expansion is authorized by this checkpoint. Preserve the inclusive
`phase15_mapping` RTF floors of `0.80`, strict freshness/future rejection,
ownership and cancellation proofs, fail-closed behavior, Product 103/Gate 7
exclusion, and all unrelated dirty work.

## Current authoritative state — 2026-09-13 — Phase 15 source/offline closeout

Phase 15 packets P0 through P8, the factory CLI packet, and the cycle-adapter
source/offline packets are complete and accepted at the
source/offline boundary. P0's isolated Humble evidence established that the
installed Python subscription API exposes per-message source and receipt
timestamps but no publisher GID, so individual TF-edge ownership remains
fail-closed; aggregate `/tf` publisher lists are not a substitute. P1, P2,
P3, and P4 passed their focused implementation checks and independent Sol/high
reviews. P5, P6, P7, and P8 have also passed focused validation and fresh
independent Sol/high review.

P1 retains mission cancellation ownership through downstream terminal-result
proof. Accepted planner, smoother, and controller handles remain owned until
their result callbacks; pending acceptance remains an obligation across a
cancel race; repeated cancellation is idempotent; unknown-goal cancellation
errors do not count as terminal proof; and deactivation monotonically
strengthens a prior cancel to `ABORTED`. Public mission results and the next
mission remain blocked until those obligations are resolved.

P2 revalidates map/costmap identity and freshness, motion authority, and the
carried TF sample immediately before motion reservation, with zero-send
discard behavior on stale or changed evidence. P3 applies minimum-distance
eligibility before representative/fallback selection, resets the exhaustion
counter only after the final reservation gate, and reports persistent raw
frontiers with no admissible costmap endpoint as `INCOMPLETE` rather than
`COMPLETE`. A true raw-frontier exhaustion remains `COMPLETE`; skipped
clusters do not consume a motion token, blacklist entry, or navigation-failure
count. `INCOMPLETE` is safely stoppable without erasing that condition and is
restartable through the existing start boundary.

P4 captures the dispatched frontier endpoint in map-frame world coordinates at
reservation, retains failed endpoints through motion cleanup, and reprojects
those coordinates through the current map origin, resolution, and planar yaw
on later plans. Out-of-bounds projections remain retained, only an accepted
explicit start clears the run-scoped failures, and exclusion is exact-cell
only with no radius. The representative fallback remains deterministic and
does not suppress safe siblings in the same cluster.

P5 sends the SLAM Toolbox serialization request a bare candidate prefix and
requires the installed two-file pose-graph output, `<prefix>.posegraph` plus
`<prefix>.data`. Manifest schema 3 records and hashes both files, rejects
legacy single-file/incomplete or tampered bundles, reserves output aliases
before service calls, and limits verified discard to the manifest-enumerated
candidate files while preserving unrelated evidence. Its final independent
Sol/high review passed.

P6 requires terminal autonomous state, explicit false `active`, `pending`,
and `fault_latched`, fresh status, positive generation, and no cancellation
target. It preserves `INCOMPLETE` distinctly and requires explicit operator
acknowledgement and explanation. Persisted acceptance reports are strictly
validated, including exact producer types, deadlines, state `uint8` value
`1`, and preflight verdict string `PASS`; its final independent Sol/high
review passed.

P7 requires quality-review schema 2 and an exact
`artifact_bundle_sha256` matching the currently verified manifest at both
`accept` and `promote`. Schema-1/unbound reviews, changed image or pose-graph
dataset, discard/resave, and refreshed-evidence receipt mutations fail closed;
its final independent Sol/high review passed. P8 validates finite explorer
parameters, map geometry, and integral occupancy values before readiness,
extraction, or completion accounting; its final independent Sol/high review
passed.

Fresh final verification after the analyzer correction:

* Combined scoped pytest across `src/amr_exploration/test
  src/amr_factory/test src/amr_manipulation/test`: 653 passed;
* Focused builds/tests, Python compilation, `git diff --check`, and independent
  Sol/high reviews: passed.

Fresh direct runtime `phase15_commission_20260912_02` passed the approved
Product 101 (1 kg) and Product 102 (3 kg) direct terminal/post-status and Gate 6
analyzer proof. Product 103 and Gate 7 remain excluded.

Final mapping evidence is not accepted. `phase15_mapping_diag_20260913_04`
stopped on a genuine RegulatedPurePursuit (RPP) collision/robot-footprint
obstacle condition. `phase15_mapping_accept_20260913_01`, from the documented
open pose `(-4.5, 0, 0)`, passed host, rendering, RTF, and staged mapping graph
gates, then failed closed in `CANCELLING`/`FAULT`. The bag shows
`slam_toolbox` owns `map->odom` and continuously delivered it approximately
0.87-0.89 s future versus `/clock`, while `odom->base_footprint` stayed fresh at
approximately 0.003 s. Sol/high diagnosed the explorer fail-closed behavior as
correct; no safe repository-side source/config/threshold/retry change is
justified. The upstream SLAM timestamp source requires separate resolution.

No source/offline packet remains. The worktree contains broad pre-existing
modified and untracked work; it was preserved. `AMR_CODEX_HANDOFF.md` was not
modified during this continuation, and canonical maps remain unchanged.

## Current completion boundary and next authorized action

The source/offline scope and approved Product 101/102 runtime scope are
complete, but Phase 15 mapping acceptance is not. No passing mapping runtime
report, candidate artifact save/validation, human map-quality decision,
promotion eligibility, canonical-map replacement, per-edge TF ownership proof,
or hardware/functional-safety acceptance is claimed. Canonical maps remain
unchanged; `AMR_CODEX_HANDOFF.md` remains unchanged. Product 103 and Gate 7
remain excluded.

The next dependency is separate upstream SLAM timestamp correction/diagnosis,
followed by the prescribed mapping acceptance. This is not a blind repository
patch or rerun.

## Historical authoritative state — 2026-09-10 — Phase 15 runtime diagnostic checkpoint

At that checkpoint, Phase 15 Slices 1 and 2 and Packet C remained complete and accepted at the
source/offline boundary. The authorized simulation work then exposed and
diagnosed two separate runtime issues in the autonomous frontier path. The
latest diagnostic run is complete and the user instructed the session to stop
after the focused implementation test and update this handoff. Phase 15 is
not closed: source review/build follow-up, autonomous runtime proof after the
latest source change, map-quality review, promotion, and canonical-map
replacement remain unverified.

Packet C source/offline closeout evidence remains:

* focused Packet C pytest: 52 passed;
* `amr_factory` colcon tests: 9/9 passed;
* `colcon test-result`: 457 tests, 0 errors, 0 failures, 5 skipped;
* Python compilation, `ament_flake8`, the `amr_factory` build, and
  `git diff --check`: passed.

The latest frontier-feasibility packet is implemented by Luna/max in the
five approved exploration files. The scoped pytest run reported 124 passed;
the `amr_exploration` build passed; its registered suites passed 3/3 with 66
tests; and `git diff --check` passed. `ament_flake8` currently reports nine
findings: seven E501 findings, the existing E402 finding, and E127 at
`test_frontier_lifecycle.py:168`. No runtime was run after that packet.

Astra/high completed the requested read-only review of the current Phase 15
source and the surrounding acceptance/artifact paths. It made no edits and
ran no runtime. The review found the current endpoint-cost correction
plausible but not sufficient for Phase 15 acceptance, and produced the
small-packet plan recorded below. The independent Sol/high implementation
verdict and all post-review implementation work remain pending.

`AMR_CODEX_HANDOFF.md` was updated only after the user's explicit authorization
to include it in the documentation packet. No canonical map was modified or
replaced. Existing safety thresholds, ownership boundaries, public interfaces,
fail-closed behavior, and hardware values remain the authority for the next
session.

## Documentation synchronization — 2026-09-10

The user authorized a docs-only synchronization of the current project state,
including `AMR_CODEX_HANDOFF.md`. The packet updates the current RPP/controller
identity, 17-package inventory, Product 101/102 runtime boundary, canonical
autonomous launch, Phase 15 frontier/artifact/acceptance limits, and historical
status labels. It preserves historical records, does not change source or
runtime behavior, and does not replace the canonical map.
The docs-only verification `git diff --check` passed; the existing dirty source
and evidence paths remain preserved and no runtime or build was run for this
packet.

## Documentation consistency audit — 2026-09-10

A read-only audit covered the repository's Markdown/RST/text documentation and
cross-checked the material PLC/mimic, controller, launch, phase-status, and
acceptance claims against the current source. The audit findings below are
retained as historical evidence; the current-state corrections are recorded in
the documentation synchronization section above and the updated files.

### PLC/mimic distinction

* No tracked project docs, source package, or source path contains an active
  `PLC`, `plc_mimic`, or `mimic_state` subsystem. The former simulated-
  permission subsystem is removed. The `log/**/amr_plc_mimic` and related
  generated build/log artifacts are stale July 2026 output, not active source;
  do not delete them as part of this documentation packet without explicit
  scope.
* The active robot description still contains a generic passive mimic at
  `src/amr_description/urdf/phase14_mobile_manipulator.urdf.xacro:63`:
  `gripper_right_finger_joint` has `<mimic joint="gripper_finger_joint" ...>`.
  Both fingers are independently exposed through `ros2_control`, but
  `src/amr_description/test/test_description.py:721-743` only rejects mimic
  parameters inside `ros2_control`; it does not assert absence of the URDF
  `<mimic>` element. If passive coupling is no longer intended, this is a
  separate source/test packet and must not be silently changed during doc
  cleanup.
* The retained mimic diagnosis in
  `docs/PHASE_14_GATE6_RUNTIME_DEBUG_REPORT.md:110` and
  `PROJECT_STATUS.md:132` is historical evidence. The upstream
  `third_party/gz_ros2_control/CHANGELOG.rst` mention is third-party history,
  not project behavior. Current command arbitration/motion-gate language is
  still a safety boundary and is unrelated to the retired PLC subsystem.

### Other material stale documentation

* The active controller is Regulated Pure Pursuit, but
  `docs/PHASE_1_SYSTEM_ARCHITECTURE.md:3`,
  `docs/PHASE_10_NAVIGATION.md:4`, `CHANGELOG.md:27-29`, and
  `TODO.md:22-27` still present MPPI/MPC and the fourteen-package count as
  current. Phase 0 inventory/performance claims are dated snapshots and need
  an explicit historical label if retained. Keep the compatibility names
  `amr_mpc_controller` and `/amr/mpc/cmd_vel` unless a separate interface
  migration is approved.
* Phase 14 procedures in
  `docs/PHASE_14_FACTORY_MOBILE_MANIPULATION.md:543-576` still require
  1/3/5 kg FIFO completion and use `factory_demo.launch.py`. The current
  accepted scope is Product 101/102; Product 103/5 kg and Gate 7 are out of
  scope. The canonical autonomous entry point is
  `src/amr_factory/launch/factory_autonomous.launch.py` with explicit
  `factory_attachment:=true`; `factory_demo.launch.py` is legacy/optional.
  `docs/ROS2_BEGINNER_PROJECT_GUIDE.md:301-368` is also actionable but stale:
  it omits the explicit attachment requirement and still describes the 5 kg
  path.
* Phase 15 mapping instructions in
  `docs/PHASE_15_FACTORY_SLAM_COMMISSIONING.md:74-117` and the corresponding
  `docs/SIMULATION_COMMANDS.md:227-275` are behind the saved P3/P5/P6/P7
  plan. They need the `INCOMPLETE` safe-stop and explicit operator decision,
  the two-file pose-graph bundle (`.posegraph` plus `.data`), and bundle- and
  runtime-report-bound quality-review hashes. The installed serializer issue
  is recorded in the plan: passing `prefix.posegraph` produces
  `prefix.posegraph.posegraph` and `prefix.posegraph.data`, while the old
  verifier expects the wrong single file.
* `PROJECT_STATUS.md:81,115` still labels retained failed-boundary records
  as “Current” despite the authoritative closeout saying later sections are
  historical. `docs/PHASE_14_GATE6_RUNTIME_DEBUG_REPORT.md:62` has the same
  misleading “Current” label. `docs/ROBOT_PARAMETER_REGISTER.md:19` still
  describes only 1 kg runtime evidence, and `src/README.md:10-25` omits the
  current `amr_exploration`, `amr_factory`, and `amr_manipulation` packages
  (the current source inventory is 17 packages).

The relevant documentation packet has now been applied: historical records were
relabelled or cross-linked, current procedures/status claims were corrected,
and `AMR_CODEX_HANDOFF.md` was included under explicit user authorization.
Non-goals remain deleting evidence, renaming compatibility interfaces,
removing the live URDF mimic, or changing safety/ownership behavior. The final
packet check is `git diff --check` plus a worktree-status review; no runtime is
implied.

## Documentation recheck — 2026-09-10

A fresh read-only cross-check found no remaining current-state documentation
corrections. The source inventory is 17 packages; `src/README.md` lists 17
package rows and the beginner-guide build block names all 17. The active RPP
configuration, Product 101/102 registry enablement, canonical autonomous launch
arguments, mapping entry points, artifact bundle gap, and terminal-acceptance
limits agree with the current source. Relative Markdown/RST links resolve, no
active PLC/mimic subsystem is documented, and `git diff --check` passes. No
source, runtime, dependency, canonical-map, commit, or push action was taken.

## Phase 15 runtime and frontier-feasibility checkpoint — 2026-09-09

The authorized runtime evidence is preserved under:

* `.ros_logs/phase15_visibility_fix_20260909_11/` — after the LiDAR visual
  self-exclusion fix, front/rear self-return points inside the base footprint
  were zero, the start-neighborhood raw costmap was clear, and direct planner
  probes from the start/open cells succeeded. That run still exposed the
  separate explorer TF lookup race.
* `.ros_logs/phase15_tf_carry_20260909_12/` — the Sol-reviewed TF carry-forward
  packet allowed the explorer to pass readiness and dispatch a goal. One goal
  succeeded; three later goals ended with Nav2 status 6, and the explorer
  correctly latched `FAULT` at `goal_failures=3`. Host preflight passed before
  and after shutdown.
* `.ros_logs/phase15_candidate_diag_20260909_13/` — diagnostic-only runtime
  capture with hardware rendering, host preflight PASS before/after, a 37 MiB
  bag containing `/map` (198), global raw costmap (135), global footprint
  (395), TF (14,157), exploration status (949), and command arbitration
  (1,505) messages, plus per-status candidate snapshots.

Sol/high diagnosed the latest failure as a Phase 15 source defect, not a
Phase 14 Nav2 configuration fault. The frontier algorithm selected a raw
`/map` free cell adjacent to unknown space and the explorer dispatched that
representative without proving that it was admissible in the current global
costmap. In `_13`, all four failed goals had `/map` value `0` but global raw
costmap value `253`; the three successful goals had costmap value `0` with
all-zero 7x7 neighborhoods. The first failed representative's same cluster
contained a cost-0 fallback cell `(166,159)`; the later failed clusters had no
cost-below-253 endpoint. Nav2 therefore rejected the invalid endpoints and
the explorer's existing three-failure gate faulted closed as designed.

The approved Luna/max implementation packet is limited to:

* `src/amr_exploration/scripts/frontier_algorithm.py`;
* `src/amr_exploration/scripts/frontier_explorer.py`;
* `src/amr_exploration/test/test_frontier_algorithm.py`;
* `src/amr_exploration/test/test_frontier_contract.py`;
* `src/amr_exploration/test/test_frontier_lifecycle.py`.

It adds a fresh `/amr/global_costmap/costmap_raw` admission view, validates
map-frame/fresh/structural geometry, maps `/map` cells through world
coordinates with planar yaw handling, replaces an obstructed representative
only within its own cluster using deterministic cost/distance/order, and
skips fully obstructed clusters without a motion token, blacklist entry, or
navigation-failure increment. Navigation ownership, cancellation, TF
carry-forward, thresholds, and the `/amr/mission/navigate_to_pose` boundary
are unchanged.

Current stop point: the user ran out of usage and requested that this plan be
saved. Do not start another runtime, source test, or implementation until the
user resumes the phase. When resumed, use the saved Astra/high findings and
packet order below; Sol/high must diagnose or independently review each
implementation packet, and Luna/max may implement only one fully specified
packet at a time. Complete the pending source review/build checks before one
fresh authorized runtime. If any admitted costmap-valid endpoint still
deterministically aborts, stop and return to Sol/high before another source
change. Runtime acceptance still requires the documented non-faulted inactive
explorer proof, artifact/runtime/quality gates, and separate human map-quality
and promotion decisions.

## Saved Astra/high review and remediation plan — 2026-09-10

This section preserves the read-only Astra/high review requested by the user.
The review is complete, but no finding has been implemented or runtime-tested
from it. Phase 15 remains open. The review surface included the latest
exploration patch and lifecycle, the mission supervisor cancellation boundary,
the SLAM save/artifact/acceptance path, and their tests and consumers.

### Review conclusions

The Phase 15 runtime failure remains a Phase 15 frontier-selection defect,
not a Phase 14 Nav2 configuration fault: raw `/map` frontier representatives
were dispatched without proving admission in the current global costmap. The
latest patch addresses endpoint cost admission, and offline replay retained
the successful endpoints, replaced `(172,162)` with cost-zero fallback
`(166,159)`, and rejected the other failed clusters. That does not prove
collision-free paths, autonomous completion, or acceptance readiness.

The following findings are prioritized. “Proven” means reproduced offline or
established from the current source; live occurrence is called out separately.

1. **High — mission cancellation can report terminal completion before
   downstream termination.** `mission_supervisor_node.cpp` changes the
   pending state to `CANCELING` and clears downstream handles. A later
   `request_cancel` or deactivation path can therefore complete the public
   action without a downstream result. TF-failure feedback can request cancel
   while cancellation is already underway. The falsifier is a held downstream
   result with repeated cancellation/deactivation: public completion and new
   mission admission must remain blocked until downstream terminal proof.

2. **High — dispatch can use expired map, authority, and TF evidence.** The
   explorer checks authority and carried TF before action-server waiting, and
   the final gate currently checks only costmap readiness. Offline boundary
   probes dispatched with evidence older than the allowed freshness window.
   The falsifier is to cross each freshness boundary during the existing wait
   and observe zero reservation and zero send.

3. **High — the SLAM serialization filename and artifact set do not match the
   installed implementation.** The CLI supplies `prefix.posegraph`, while
   the installed serializer appends `.posegraph` and `.data`, producing
   `prefix.posegraph.posegraph` and `prefix.posegraph.data`. The artifact
   verifier instead expects `prefix.posegraph` and omits the dataset required
   for deserialization. Tests currently manufacture only one `graph` file.

4. **High — autonomous acceptance treats pending or still-running exploration
   as finished.** `factory_mapping_acceptance.py` checks `active=false`, a
   positive generation, freshness, and no reported fault, but does not require
   an explicit terminal state or `pending=false`. Offline snapshots in
   `WAITING_READY`, `SCANNING`, `GOAL_PENDING`, and pending `CANCELLING`
   incorrectly passed.

5. **Medium — exploration completion has counter, distance-selection, and
   terminal-policy defects.** The exhaustion counter is reset before the
   near-candidate branch increments it, so consecutive updates can remain at
   `1`. Distance filtering happens after one representative is selected per
   cluster, so a near representative can hide a distant admissible cell. The
   current three-blocked-update path reaches `COMPLETE` while raw frontiers
   remain. The user selected the required policy: remaining frontiers with no
   safe reachable goal must stop as `INCOMPLETE`/safely stopped and require an
   explicit operator decision before map acceptance; they must not be called
   successful completion.

6. **Medium — blacklist identity changes when map geometry changes.** Failed
   grid indices are retained across map updates and then interpreted in the
   new geometry. An origin/resolution/yaw change can retry the failed world
   location or suppress an unrelated one, causing repeated failures or false
   exhaustion.

7. **Medium — human approval is bound to YAML rather than the reviewed map
   image/dataset bundle.** The quality seam hashes the YAML and path only. A
   discard/resave workflow can preserve those bytes while changing image
   contents and leave the old human review valid. Bundle verification protects
   artifact mutation but does not invalidate a stale human decision.

8. **Medium — bounded input validation is incomplete.** Explorer timeout
   parameters accept nonfinite values; invalid occupancy values can reach
   frontier exhaustion; and reserved candidate names can alias the candidate
   YAML/manifest or receipt outputs, causing later writes to collide or replace
   the wrong artifact.

There is also a confirmed commissioning blocker: the live
`RosGraphObserver.snapshot` does not supply `tf_publishers`, while snapshot
evaluation requires it. This is correctly fail-closed, but the shipped live
observer cannot pass TF ownership until an evidence-backed ownership collector
is established. Do not substitute the list of `/tf` publishers for ownership
of individual edges.

No authentication vulnerability was established under the trusted-local-writer
boundary. Editable receipts are not signatures, and symlink/hardlink races
requiring manipulation by that same trusted writer are not a separate
authentication threat here.

### Small implementation packets

Each packet requires a fresh `git status --short`, rereading its target files,
the smallest coherent edit, a complete diff review, focused tests, and a
Sol/high independent review before the next packet. Luna/max is restricted to
one approved packet at a time. No packet may weaken a safety gate, threshold,
ownership boundary, public interface, collision gate, or fail-closed path.

| Packet | Allowed files | Required behavior and validation |
|---|---|---|
| **P0 — TF-owner evidence** | Read-only first; exact collector files only after evidence | Verify whether the installed rclpy/RMW exposes publisher identity for each received TF message and whether it can be matched unambiguously to graph endpoint identity. Produce support/limitations and a bounded design. If identity cannot be established, retain failure; do not implement a large collector or bypass ownership. |
| **P1 — retain mission cancellation ownership** | `src/amr_mission/src/mission_supervisor_node.cpp`; `src/amr_mission/test/test_mission_supervisor_behavior.cpp` | Retain downstream acceptance/terminal obligations through cancellation; repeated cancellation joins the same obligation; deactivation may strengthen the outcome to abort but cannot erase ownership. Cancel each accepted handle once. Test repeated TF feedback, cancel→deactivate, pending late acceptance/rejection, held results, and result races. Public result and next mission remain blocked until proof. Build/test `amr_mission`. |
| **P2 — revalidate dispatch evidence** | `src/amr_exploration/scripts/frontier_explorer.py`; `src/amr_exploration/test/test_frontier_lifecycle.py` | Immediately before reservation, recheck map/costmap receipt age, authority, and carried TF after waits/computation. Ensure the selected endpoint belongs to the checked map/costmap snapshot; discard the plan if identity changes. Preserve one carried TF lookup, run/state guards, and zero-send behavior on stale evidence. Test each freshness boundary during the existing wait. |
| **P3 — correct selection and incomplete stopping** | `src/amr_exploration/scripts/frontier_algorithm.py`; `src/amr_exploration/scripts/frontier_explorer.py`; exploration algorithm/lifecycle tests; `docs/PHASE_15_FACTORY_SLAM_COMMISSIONING.md` | Apply minimum-distance eligibility while choosing representatives/fallbacks within a cluster. Reset exhaustion only when a dispatchable choice exists. After the existing three-update limit, raw exhaustion may be `COMPLETE`; remaining frontiers with no admissible choice must become `INCOMPLETE`/safely stopped with reason beginning `exploration incomplete:`. No token, blacklist entry, or failure increment for skipped clusters. Preserve the existing three-navigation-failure `FAULT` gate. Test distant fallback, persistent/mixed blocked updates, true exhaustion, stop, and restart. Correct the isolated E127 in the lifecycle test as part of this packet only if still in scope. |
| **P4 — preserve blacklist world identity** | `src/amr_exploration/scripts/frontier_algorithm.py`; `src/amr_exploration/scripts/frontier_explorer.py`; exploration algorithm/lifecycle tests | Capture attempted endpoint world coordinates at reservation. Project failed positions into current grid geometry on later maps; never reinterpret the original index. Test translated origin, resolution/yaw changes, out-of-bounds failures, and geometry changes while a goal is pending. Preserve failures until explicit run restart; add no exclusion radius. |
| **P5 — repair serialized artifact contract** | `src/amr_factory/scripts/factory_mapping_cli.py`; `src/amr_factory/scripts/factory_mapping_artifacts.py`; factory mapping CLI/acceptance tests; `docs/PHASE_15_FACTORY_SLAM_COMMISSIONING.md` | Send the bare serialization prefix. Require, hash, and verify both `.posegraph` and `.data`; update the manifest schema and explicitly reject incomplete older bundles. Model the installed writer’s two outputs in tests. Cover missing/corrupt dataset and verified discard. Reject reserved candidate names and output-path aliases before service calls. |
| **P6 — enforce terminal acceptance and explicit incomplete approval** | `src/amr_factory/scripts/factory_mapping_acceptance.py`; its tests; `docs/PHASE_15_FACTORY_SLAM_COMMISSIONING.md` | Require terminal state, explicit false `active`, `pending`, and `fault_latched`, fresh status, positive generation, and no outstanding cancellation target. Preserve `INCOMPLETE` distinctly in runtime reporting. Accepting that outcome requires a specific operator acknowledgement, explanation, and matching runtime-report hash in the quality review; missing acknowledgement fails. Test every pending/transitional state and incomplete reports with/without matching acknowledgement. |
| **P7 — bind human review to the bundle** | `src/amr_factory/scripts/factory_mapping_acceptance.py`; its tests; `docs/PHASE_15_FACTORY_SLAM_COMMISSIONING.md` | Require the existing `artifact_bundle_sha256` in quality review and verify it at accept and promote. Update the review schema explicitly; do not reinterpret old approvals silently. Test unchanged YAML with changed image/dataset, discard/resave, and receipt mutation. |
| **P8 — validate explorer inputs** | `src/amr_exploration/scripts/frontier_algorithm.py`; `src/amr_exploration/scripts/frontier_explorer.py`; exploration algorithm/lifecycle tests | Reject all nonfinite scalar parameters before subscriptions/timers. Validate map structure, planar geometry, and integral occupancy domain `{-1, 0…100}` before extraction or completion accounting. Invalid evidence cannot count toward exhaustion. Test NaN/infinity and malformed-but-serializable occupancy data while retaining bounded readiness/fault behavior. |

Dependencies and order: establish P0’s TF-owner evidence first; P2 precedes
P3; P3 precedes P6; P5 precedes P7. P1 is independent but must be resolved
before trusting cancellation-based acceptance. P4 and P8 can be implemented
independently. For every packet the sequence is Sol/high diagnosis or review →
one Luna/max implementation → focused validation → Sol/high independent review.

### Resume and stop conditions

On resume, review this section against the current worktree and source before
choosing the first packet. Do not assume the packet is still applicable if
source changes invalidate its evidence. Run focused source checks before any
runtime. Runtime requires fresh authorization and must stop at the first
mandatory gate failure; a runtime failure returns to Sol/high diagnosis before
another edit. No runtime proof is implied by build or unit success.

The current known validation state is: 124 scoped pytest tests passed;
`amr_exploration` build passed; registered exploration tests passed 3/3 (66
tests); `git diff --check` passed; `ament_flake8` has nine findings listed
above; no runtime was run after the latest frontier packet; and no Astra/high
or Sol/high review authorized acceptance. Packet C remains source/offline
accepted, while runtime, map quality, promotion, and canonical-map replacement
remain unverified.

The user-selected completion rule is authoritative for future packets: if
unexplored frontiers remain but none has a safe reachable goal, stop safely as
`INCOMPLETE` and require an explicit operator decision before accepting the
map. Do not reuse `COMPLETE` for that condition.

`AMR_CODEX_HANDOFF.md` has been updated only as part of the explicitly
authorized documentation packet. No canonical map has been modified or
replaced.

## Current authoritative state — 2026-09-08 — Phase 14 runtime acceptance complete; closeout

The active feature is the approved autonomous factory cycle for Product 101
(1 kg) and Product 102 (3 kg): the operator can select the pickup station, the
AMR can begin from any valid localized pose, and the system can run 1 kg then
3 kg repeatedly like a factory. The approved behavior includes finite and
continuous modes, graceful stop after the current cycle, immediate cancel, and
optional return home. Product 103 remains disabled.

The source and offline-integration phase is complete for this approved scope.
The bounded A5/A6 factory terminal-proof correction is accepted at the
source/offline-contract level by Sol/high review. The autonomous launch,
registry-derived mapping, action/interface ownership, cycle adapter, and
downstream late-goal acceptance seams are implemented and covered by source
contracts, real Humble builds, and registered package tests.

Fresh direct-host runtime acceptance is complete for the approved Product
101/102 scope. Product 101 completed a normal native-attachment cycle in
`_11`; Product 102 completed a normal native-attachment cycle in `_13`;
`_14` exercised navigation cancellation and retained-product manipulation
cancellation; and `_16` exercised graceful stop with one active delivery
completing while the queued delivery did not start. All mandatory host,
runtime, graph, lifecycle, MoveIt, registry, bootstrap, status, and ownership
gates passed for the accepted runs. Product 103 and Gate 7 remain out of
scope. No natural late-response callback was produced during these nominal
runs; its fail-closed ownership is covered by the source/runtime contract
tests.

The durable compact evidence is the run-level gate output, analyzer result,
CLI/status window, and cancellation/stop summaries under
`.ros_logs/amr_autonomous_factory_20260908_{11,13,14,16}/evidence/`.
Full-topic bags are not required for closeout and are not retained in the ROS
log area.

The approved design and scope are recorded in
`docs/PHASE_14_FACTORY_MOBILE_MANIPULATION.md` under
“Autonomous 1 kg / 3 kg factory cycle — approved 2026-09-04.”

## Current worktree

For the current state, use this worktree inventory and the closeout section at
the end of this handoff. The older sections below are retained as historical
diagnosis and audit evidence.

Modified paths:

- `AGENTS.md`
- `SESSION_HANDOFF.md`
- `docs/PHASE_14_FACTORY_MOBILE_MANIPULATION.md`
- `src/amr_bringup/config/interface_ownership.yaml`
- `src/amr_bringup/test/test_workspace_contract.py`
- `src/amr_factory/CMakeLists.txt`
- `src/amr_factory/config/products.yaml`
- `src/amr_factory/package.xml`
- `src/amr_factory/scripts/factory_cli.py`
- `src/amr_factory/src/factory_supervisor_node.cpp`
- `src/amr_factory/test/test_factory_demo_contract.py`
- `src/amr_interfaces/CMakeLists.txt`
- `src/amr_interfaces/msg/FactoryStatus.msg`
- `src/amr_manipulation/CMakeLists.txt`
- `src/amr_manipulation/scripts/gate6_evidence_analyzer.py`
- `src/amr_manipulation/launch/gate6_mass_stage.launch.py`
- `src/amr_manipulation/scripts/gate6_product_test.py`
- `src/amr_manipulation/src/gate6_mass_stage.cpp`
- `src/amr_manipulation/test/test_gate6_completion_contract.py`
- `src/amr_manipulation/test/test_moveit_config.py`
- `src/amr_manipulation/test/test_product_test_contract.py`

Untracked paths:

- `src/amr_factory/launch/factory_autonomous.launch.py`
- `src/amr_factory/scripts/factory_registry.py`
- `src/amr_factory/test/test_factory_autonomous_contract.py`
- `src/amr_interfaces/action/ExecuteProductCycle.action`
- `src/amr_interfaces/action/NavigateStation.action`
- `src/amr_interfaces/action/RunSequence.action`
- `src/amr_manipulation/scripts/cycle_manipulation_supervisor.py`
- `src/amr_manipulation/test/test_cycle_adapter.py`

Preserve all of this work. `AMR_CODEX_HANDOFF.md` was separately updated only
in its documentation sections under explicit user authorization.

## Source/integration implementation present — runtime accepted for Product 101/102

The live diff contains:

- the three new action contracts and additional `FactoryStatus` fields;
- Product 101/102 autonomous configuration, with Product 103 disabled;
- a shared factory registry parser;
- factory CLI and supervisor changes for station selection and sequencing;
- a cycle-level manipulation adapter;
- the accepted bounded A5/A6 factory terminal/cancellation correction;
- the autonomous factory launch, registry-derived Product 101/102 mapping, and
  Product 103 exclusion;
- complete action/interface ownership and bringup contract coverage;
- the cycle-level manipulation adapter and generalized Gate 6 runner;
- callback-based acceptance ownership for factory execute/home, mass-stage
  navigation/egress/gripper, and Python preparation navigation; and
- build/install declarations and registered tests for the new surface.

The source contracts, real Humble builds, and package tests provide offline
evidence for callback ownership, status freshness, bootstrap/joint-state proof,
accepted-goal cancellation, and MoveIt stop plumbing. They do not prove live
ROS graph behavior, Gazebo/MoveIt execution, or physical terminal outcomes.

The approved runtime/integration acceptance is complete for Product 101 and
Product 102. The remaining phase boundaries are deliberate: Product 103 and
Gate 7 are not enabled, and no physical-robot or hardware-safety claim is
made. The nominal runtime did not naturally generate a late acceptance
callback; the bounded source contract test remains the evidence for that rare
path.

## Debugging and implementation-attempt ledger

Attempt 1 exposed deterministic blockers including a duplicate `nav2_msgs`
manifest declaration plus unresolved callback, status, startup-proof,
cancellation, registry-parsing, and factory state-machine concerns. Sol/high
issued a stop/correction packet.

Attempt 2 addressed many source-level markers, but the focused build stopped
before compiling factory C++ because `src/amr_factory/package.xml` declared
both `<depend>std_srvs</depend>` and `<exec_depend>std_srvs</exec_depend>`.
With package format 3, the generic dependency already includes the execution
dependency, so the declarations overlap and the manifest parser rejects them.

The user then authorized a duplicate-only correction. Luna/max removed only
the redundant `<exec_depend>std_srvs</exec_depend>` and retained the generic
dependency. The direct manifest parser passed, `colcon build
--packages-select amr_factory --symlink-install` passed, `git diff --check`
passed, and Sol/high independently passed that one-line correction.

This confirms that the duplicate declaration was the cause of that specific
`amr_factory` build failure. It does not show that it was the only defect in
the broader feature. The two-attempt limit for the broader implementation was
reached and reported. Broader autonomous source patching is paused until a
fresh bounded diagnosis is reviewed and the user authorizes another
implementation packet.

## Earlier validation evidence and limits — before 2026-09-07 review

Passed evidence available from the partial work:

- `amr_interfaces` built successfully earlier;
- Python compilation passed for `factory_cli.py`, `factory_registry.py`,
  `cycle_manipulation_supervisor.py`, and `gate6_product_test.py`;
- 34 pre-existing tests passed before the planned new tests existed; and
- after the duplicate-only fix, the manifest parser, focused `amr_factory`
  build, and `git diff --check` passed.

Important limits:

- the 34 existing tests do not cover the new autonomous behavior;
- the full `amr_manipulation` build/install has not been freshly completed
  against the current worktree;
- the planned autonomous contract and cycle-adapter tests do not exist; and
- no autonomous simulation or hardware runtime has run.

The strongest unverified areas are the factory state machine, loop/stop/home
semantics, arbitrary-localized-start behavior, selected-station routing,
registry validation, cross-package adapter integration, QoS and status
freshness, bootstrap proof, and cancellation propagation through terminal
verification. Do not claim these are fixed based only on code markers.

## Continuation evidence — 2026-09-08 — before independent review

The bounded A5/A6 correction attempt 2 changed only the approved factory
supervisor and autonomous contract-test paths. It now:

- latches the factory fault when a delivered cycle lacks fresh empty-stow proof,
  even when no product is currently marked held;
- latches standalone home interlock and home-cancellation-confirmation
  failures before clearing their transient flags; and
- keeps the final home interlock check and `async_send_goal` in one mutex-held
  dispatch section.

The new contract probes were red before the source correction (3 failed, 4
passed) and green afterward (7 passed). Fresh validation then produced:

- `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_factory/test/test_factory_autonomous_contract.py` — 7 passed;
- `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_factory/test` — 70 passed;
- `source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select amr_factory --symlink-install --executor sequential --parallel-workers 2` — passed;
- `ctest --test-dir build/amr_factory -R '^factory_autonomous_contract_test$' --output-on-failure` — passed; and
- `git diff --check` — passed.

This is source and offline contract evidence only. No runtime, simulation,
hardware, commit, push, dependency installation, or external change was
performed. The implementation session found no contradiction and requested a
separate Astra/medium review. That review subsequently withheld acceptance as
recorded below. Usage telemetry was not observed during implementation; the
independent-review session subsequently located the primary 300-minute telemetry.

## Exact next action

1. Obtain explicit runtime authorization before launching Gazebo/MoveIt or
   running autonomous product cycles. No runtime authorization or runtime
   execution is recorded by this handoff.
2. Source `/opt/ros/humble/setup.bash` and `install/setup.bash`, set the
   documented `GZ_VERSION=harmonic`, perform the required empty-domain/readiness
   checks, and launch only the approved autonomous graph. The graph must use
   `factory_autonomous.launch.py` with `control_mode=autonomous`, the registry
   must derive Product 101/102 mappings, and Product 103 must remain excluded.
3. Verify live graph/ownership/QoS/status freshness and the fail-closed gates
   before attempting a cycle. Exercise Product 101, then Product 102, including
   navigation, pickup/place, attachment/detachment, empty-stow proof, graceful
   stop, immediate cancel during navigation and manipulation, fault retention,
   and no next-job/home movement after cancellation.
4. Specifically observe downstream behavior for acceptance timeout/late goal
   responses. The source callbacks issue cancellation for late accepted handles,
   but source tests cannot prove terminal `CANCELED` after the caller returns.
   Preserve logs and stop at the first failed mandatory gate.
5. Do not patch immediately after a runtime failure. Return the runtime evidence
   to Sol/high diagnosis/review first; preserve fail-closed behavior, thresholds,
   public interfaces, ownership boundaries, and the protected handoff.

## Historical independent A5/A6 review — 2026-09-08 — acceptance withheld

User requested continued engineering work and an agent check of Luna against
Astra's plan. A separately selected `gpt-6-astra`/medium reviewer confirmed all
three attempt-1 misses are corrected, but found:

- `sequence_loop`'s early `sequence_cancel_requested_` exit skips fresh
  empty-stow proof. Missing proof returns ordinary CANCELED without establishing
  a fault latch. A prior latch survives but is masked by the terminal outcome.
- The home-dispatch probe calls `try_lock()` on a nonrecursive `std::mutex`
  already owned by the calling thread. This is undefined behavior and cannot
  establish the lock-scope requirement.

Fresh factory pytest: 70 passed; registered autonomous CTest: 1/1 passed;
focused factory build and `git diff --check`: passed. These checks do not
invalidate the review findings. An independent compiled production-seam probe
passed two safe-cancel controls and failed six injected unsafe/prior-fault
cases across EXECUTING/CANCELING terminal states. No home goal was dispatched.
The runnable probe and evidence are at
`/tmp/amr-a5a6-review-20260908-8zz7VJ/`. Runtime incidence was not measured.

Classification: SOURCE plus TEST/EVIDENCE HARNESS, HIGH confidence. The
diagnosis remains supported, but implementation/test coverage is incomplete.
No third source patch was made. The Phase 14 plan now contains the concrete
escalation proposal; another implementation requires the user's decision under
the two-attempt rule. No Astra/high escalation was needed to resolve these
findings. Launch/ownership integration remains pending and was only inspected.

Session telemetry was observable: `rate_limits.primary.window_minutes == 300`,
latest observed usage 32% at the review evidence checkpoint. This work stopped
at the mandatory implementation-attempt boundary before exhausting allowance.
Production source/tests and the protected handoff were unchanged by this review;
only this handoff and the Phase 14 plan were updated in the repository.

## Packet A3/A4 — status authority and fault retention — implemented; review passed

Astra/medium’s targeted replay against the post-A1 adapter still reproduced
R2/R3/R4: stale idle proof publishes motion permission, malformed/duplicate
joint arrays establish proof, and unowned/late child status can overwrite a
latched fault. `_set_idle_from_proof` can also clear fault and retained-product
evidence. The user approved the packet. Luna completed implementation attempt
1, then a bounded correction attempt 2 after Astra/high found a terminal-window
gap. The corrected delta passed Astra/high re-review; no runtime was run.

Luna/max may modify only the adapter and its existing offline test:

- `src/amr_manipulation/scripts/cycle_manipulation_supervisor.py`
- `src/amr_manipulation/test/test_cycle_adapter.py`

The bounded correction must reject mismatched/duplicate joint names; require
fresh independent bootstrap and joint proof on every idle publication; deny
motion whenever a fault is latched; gate child status on active-goal authority;
make terminal idle proof return success/failure without clearing a latch; and
make all three terminal callers fail closed on refusal. Preserve thresholds,
public interfaces, geometry, A1 behavior, and active child freshness checks.
Do not change timers, bootstrap request lifetime, child handoff authentication,
cancellation handshake, factory/registry/launch code, or runtime behavior.

Required focused evidence is a red pre-packet replay followed by green tests
for expiry/refresh, malformed and duplicate joints, unowned/late status,
fault/attachment retention, terminal refusal, and an active loaded positive
control. Run the existing focused pytest and `git diff --check` only; then
Astra/high independently reviewed this motion-permission change. Stop on
contradictory state-model evidence, a need to touch another path, or the
90%-used five-hour allowance. Snapshots/logs are under
`/tmp/amr-a3a4-pre-ThWSze/` and `/tmp/amr-a3a4-attempt1-5yoFti/`.

## Packet A5/A6 — factory terminal and cancellation semantics — attempt 1 failed review

Astra/medium’s source trace confirms HIGH-confidence factory defects: failed
jobs can still reach optional home, missing empty-stow proof can reach home
when no product is marked held, and typed `INTERLOCK_FAILED`/retained-product
failures can be masked by cancellation flags. `navigate_home` also lacks a
final fresh-proof/fault check immediately before dispatch. The Trigger service
does not establish the public action’s ROS `CANCELING` state; Humble transitions
that state only after the cancel callback returns.

Allowed files are only `src/amr_factory/src/factory_supervisor_node.cpp`, new
`src/amr_factory/test/test_factory_autonomous_contract.py`, and its one CMake
test registration. The bounded packet would prioritize typed failures, require
fresh safe empty-stow proof before ordinary cancel/home, prohibit home after a
failed job, recheck home interlocks immediately before send, keep fault latches
monotonic, and call `canceled()` only when `goal->is_canceling()`; otherwise it
would abort with a typed CANCELED result. Critical interlock/retention failures
always abort with their failure outcome.

Required offline cases cover R5/R6/R7, proof expiry before home, ordinary versus
Trigger cancellation, actual CANCELING versus EXECUTING action states, and
fault-latch preservation. Attempt 1 added only the allowed C++/test/CMake paths;
its saved replay had 4/5 red before implementation and 5/5 focused tests after.
Astra/medium then found the three misses above. At the previous handoff,
correction attempt 2 had started but stopped at 94% five-hour usage before any
correction edit or test; it resumed on 2026-09-08, with fresh evidence recorded
above. Snapshots/logs: `/tmp/amr-a5a6-pre-vTNTrK/` and
`/tmp/amr-a5a6-attempt1-W8d2tO/`. No runtime, launch, registry, downstream
cancellation handshake, or late-goal ownership changes are included.

## Fresh review evidence — 2026-09-07

- `amr_interfaces`, `amr_factory`, and `amr_manipulation` built successfully.
- Focused pytest over factory/manipulation/bringup: 96 passed, 1 failed at
  `test_moveit_config.py:149` (old literal gripper-wait assertion).
- Both manipulation gtest suites passed; five Python AST checks, three manifest
  parser checks, and `git diff --check` passed.
- Offline checks reproduce Humble startup/type/terminal API incompatibilities,
  stale idle motion permission, fault overwrite by a late child status,
  malformed joint proof acceptance, home after failure/missing stow proof, and
  cancellation-failure masking. The Trigger cancellation path also lacks the
  public action's required CANCELING transition.
- Missing autonomous launch, new behavior tests, and ownership entries remain.
  Late goal acceptance, child handoff, and downstream stop proof need further
  bounded work. Passing source checks are not runtime proof.

Raw logs, offline probes, and source hashes are in
`/tmp/amr-autonomous-review-20260907-ZSfLdb/`; detailed findings, commands,
limitations, and the first concrete packet are saved in the Phase 14 plan.
At the initial read-only review, only `AGENTS.md`, this handoff, and the Phase 14 document were edited. No
production source, repository tests, runtime, commit, push, or dependencies
were changed. The temporary probe had one corrected stub-compilation error,
preserved in its first log; this was not an implementation attempt.

## Historical model workflow revision — superseded by current `AGENTS.md`

The user approved saving a token-conscious workflow in `AGENTS.md`, this
handoff, and the active Phase 14 plan on 2026-09-07. That earlier
Astra/medium-first workflow and its packet history are retained below as an
audit record, but are not the active assignment.

The current `AGENTS.md` rule is authoritative: Sol/high is the default and
only model role for analysis, planning, diagnosis, and independent review;
Luna/max is used for one approved, fully specified implementation packet;
Astra is not selected or delegated unless the user explicitly directs it.
Use one writer, concise handoffs, focused checks, and existing evidence where
still valid. Do not claim a model switch unless one was actually selected.

Every packet must identify exact changes and allowed files, causal evidence,
state-transition decisions, invariants, behavioral tests, expected results,
and stop conditions. Follow `.codex/DEBUG_PLAYBOOK.md` using the updated roles
in `AGENTS.md`. Changing models does not reset attempt limits.

## Usage guard — requested 2026-09-07

Stop work when the five-hour allowance has 5% remaining (95% used), or earlier
at a required approval/safety boundary. Read current session `token_count`
events' `rate_limits.primary` and confirm `window_minutes == 300`; do not use
the weekly percentage or token estimates as a substitute. Check between work
steps and before delegation, and propagate this guard to agents. If current
usage cannot be observed, report that limitation rather than promising the cap.

## Historical latest A5/A6 continuation — 2026-09-08 — Astra/high required

The user authorized continued correction after the documented medium-review
stop condition. The work remained limited to
`src/amr_factory/src/factory_supervisor_node.cpp` and
`src/amr_factory/test/test_factory_autonomous_contract.py`; unrelated dirty
paths and `AMR_CODEX_HANDOFF.md` were preserved.

The implementation history is:

- A5/A6 implementation attempt 1 failed review for typed-failure masking,
  missing empty-stow fault retention, and an unlocked home dispatch check.
- Correction attempt 2 addressed those three targets. Astra/medium then found
  an unchecked early cancellation proof and an invalid `std::mutex` lock-test
  oracle; those were corrected in the shared source/test paths.
- A subsequent Astra/medium review found three more issues: newer attachment
  evidence could be consumed after an unlocked proof, final-`COMPLETE`
  cancellation could bypass proof, and cancellation remained admissible after
  result selection. A correction added current-proof revalidation,
  final-cancellation proof, and `sequence_terminal_decided_` admission closure.
- A fresh Astra/medium review then found three remaining races in that
  intermediate correction: proof could become stale after a selected cancel,
  confirmed home cancellation skipped post-cancel proof, and the final-lock
  race incorrectly faulted a safe cancellation. The latest local correction
  carries proof state into terminal commit, re-proves confirmed home
  cancellation, and retries the terminal commit when cancellation appears at
  the boundary.

Fresh local evidence for the latest correction:

- focused autonomous contract test: 8 passed;
- full `src/amr_factory/test`: 71 passed;
- `amr_factory` build: passed;
- all 7 registered `amr_factory` CTests: passed; and
- `git diff --check`: passed.

These are source/offline checks only. The latest independent Astra/high review
was started but errored at the account usage limit before producing a verdict.
The last observed primary telemetry before that review was 89% of the
300-minute window. Do not claim the latest correction is safe or accepted.

Required next action on resume: use Astra/high for an independent diagnosis and
review of the complete cancellation, proof freshness, attachment retention,
terminal-action, and optional-home state transitions before any further source
edit. Do not apply another patch from a medium-only review. Runtime,
simulation, hardware, downstream cancellation integration, and final launch
acceptance remain unverified.

## Fast DDS correction and pending readiness proof

The earlier Fast DDS service-response correction is committed and pushed at
`abf3dbc`. Preserve these invariants:

- the non-default publisher profile named `service` keeps
  `reliability/max_blocking_time/sec = 1`;
- factory launch uses the absolute installed profile through
  `FASTRTPS_DEFAULT_PROFILES_FILE` before starting children;
- `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` remains in place; and
- do not substitute `FASTDDS_DEFAULT_PROFILES_FILE`, enable
  `RMW_FASTRTPS_USE_QOS_FROM_XML`, or override reliability kind, publication
  mode, history/memory, data sharing, or default profiles.

Static validation passed: XML syntax, 28 focused factory/MPC tests,
`amr_factory` and `amr_mpc_controller` builds, installed/source checksum,
installed Fast DDS probe, and diff check.

Runtime `_02` in domain 210 was only a partial readiness proof: startup showed
no prior Fast DDS response warning, median RTF was `0.9984307662`, aggregate
RTF was `0.8100800247`, and shutdown was clean, but the run stopped before
MoveIt and final readiness gates. The user temporarily accepted an RTF floor
of 0.80 without changing the checked-in 0.90 threshold. `_01` and `_03` must
not be reused. If separately authorized, the suggested next readiness-only
identity is `fastdds_service_match_20260904_04`, domain 213, after an empty
domain check. Do not mix that proof with autonomous-product runtime.

Product 102 remains accepted from
`.ros_logs/gate6_product102_geometry_20260902_03/`. Product 103 and Gate 7
remain pending. Do not rerun Product 101 or 102 merely for progression.

## Protected invariants

Preserve fail-closed behavior, lifecycle barriers, command ownership, public
interfaces, collision and placement gates, safety thresholds, and documented
hardware values. Never weaken a test or gate to obtain a pass. No runtime,
commit, push, dependency installation, system change, or external change was
authorized or performed during this handoff update.

## Historical continuation checkpoint — 2026-09-08 — review interrupted

The user requested “fix it” and then interrupted the resumed Astra/high review.
No production or test source was changed during that continuation. Before the
review, the current worktree was recorded with the same modified and untracked
paths listed above; `AMR_CODEX_HANDOFF.md` was not modified. Baseline copies of
the two approved factory paths and a short experiment ledger are preserved at
`/tmp/amr-fix-baseline-o6iZjs/`.

Fresh focused evidence before interruption:

- `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_factory/test/test_factory_autonomous_contract.py` — 9 passed in 2.95s.
- The latest observed `rate_limits.primary` telemetry was 90% used with a
  300-minute window at `2026-09-08T03:30:43Z`. The 95% usage stop condition
  remains active; no further delegation or patching should occur after it.

The required Astra/high review was assigned to agent
`01a07f0d-cd2a-7682-8dd2-7cf7e6a1c1dd` for read-only analysis of cancellation,
proof freshness, attachment retention, terminal action state, and optional
home transitions. It did not return a verdict before the user interruption.
Therefore the current correction remains unaccepted. Do not claim that the
passing offline tests establish safety.

On resume, first determine whether the Astra/high review produced a result.
If it did, use only its concrete diagnosis as the implementation packet for
one Luna/max attempt, within the approved two factory paths and the 95% usage
guard. If it did not, stop and report that the required independent review is
incomplete; do not improvise another source patch. Runtime, simulation,
hardware, downstream cancellation integration, launch integration, and final
autonomous acceptance remain unverified.

## Historical final A5/A6 checkpoint — 2026-09-08 — superseded by closeout below

The bounded terminal-proof correction is accepted for source/offline evidence.
Only these two paths were changed for the correction:

- `src/amr_factory/src/factory_supervisor_node.cpp`
- `src/amr_factory/test/test_factory_autonomous_contract.py`

The final sequence commit now revalidates current safe empty-stow proof under
the terminal mutex for all non-interlock outcomes. Unsafe retained, attached,
stale, missing, or faulted state latches the fault and returns
`INTERLOCK_FAILED`; committed delivery counters are preserved. Standalone home
proves every non-typed result before clearing its goal, while safe generic
navigation failure remains `NAVIGATION_FAILED`. Typed failure priority,
cancel ownership, `CANCELING` versus `EXECUTING` terminal behavior, freshness,
timeouts, public interfaces, and Product 103 exclusion are preserved.

Evidence:

- red-before correction: 10 passed, 2 failed on the new terminal-window cases;
- focused autonomous contract tests: 12 passed;
- full `src/amr_factory/test`: 75 passed;
- `amr_factory` build: passed;
- registered factory CTests: 7/7 passed;
- `git diff --check`: passed;
- independent Sol/high review: PASS, source/offline only.

Final correction hashes were `917b0b70b52ac824b053716724cc3fc6a20ca95a186972d6db40ac865e9cd039`
for the supervisor and `77afc7e1c56f8a14d44e50d5f68e5d5624926f2d877e9042fb8be0edfaf76a6e`
for the contract test. No runtime, simulation, hardware, commit, push,
dependency installation, or external change was performed.

At that historical checkpoint, the next process was a fresh read-only diagnosis
of downstream late-goal acceptance, cancellation ownership, and the
preparation-to-mass-stage handoff. The launch/ownership integration and source
gates were completed afterward; current runtime instructions are in the
authoritative closeout below. Runtime acceptance still requires separate
authorization and must stop at the first failed gate with evidence preserved.

The user-reported 4% allowance checkpoint was active at that historical point.
The current active role and usage rules are those in `AGENTS.md`; the source
and integration closeout below is the latest state.

## Source/integration and runtime closeout — 2026-09-08 — Product 101/102 accepted

This is the latest implementation state and supersedes earlier historical
wording that described the autonomous launch, ownership coverage, or source
gates as missing.

### Completed source and integration work

- The A5/A6 factory terminal-proof correction is accepted for source/offline
  evidence. Sequence and home terminal decisions preserve typed interlock and
  retained-product failures, require fresh empty-stow proof, preserve
  `CANCELING` versus `EXECUTING` action semantics, and keep the home final proof
  and goal dispatch in one mutex-held section.
- `factory_autonomous.launch.py` starts the factory localization/world graph,
  cycle manipulation adapter, and factory supervisor in autonomous mode. It
  derives Product 101/102 station mappings from the registries, rejects invalid
  mappings, and excludes Product 103. The autonomous graph does not start the
  legacy standalone manipulation executable.
- `ExecuteProductCycle`, `RunSequence`, and `NavigateStation` are present with
  the associated CLI, registry, status, interface-ownership, build, and bringup
  contract coverage. The cycle adapter owns the downstream manipulation action,
  cancellation service, and canonical internal status path.
- Factory execute-cycle and home navigation, mass-stage navigation and dock
  egress, bilateral gripper commands, and Python preparation navigation now
  retain acceptance ownership. If a goal response arrives after the acceptance
  window, the retained callback issues cancellation instead of abandoning the
  handle. Accepted in-window cancellation still requires cancellation
  acknowledgement and terminal `CANCELED` proof; unresolved acceptance fails
  closed.
- Gate 6 preparation and mass-stage paths preserve the existing motion,
  attachment, freshness, geometry, and safety thresholds. No Product 103 or
  unrelated phase behavior was enabled.

### Fresh validation evidence

- `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q src/amr_bringup/test
  src/amr_factory/test src/amr_manipulation/test` — **151 passed**.
- `colcon build --packages-select amr_bringup amr_manipulation amr_factory
  --event-handlers console_direct+` — **3 packages finished**.
- `colcon test --packages-select amr_bringup amr_manipulation amr_factory
  --event-handlers console_direct+` — **100% passed**: bringup **1/1**,
  manipulation **7/7**, factory **7/7**.
- Focused autonomous factory contract test — **12 passed**, including the
  late home acceptance-cancellation probe.
- `git diff --check` — passed.
- `AMR_CODEX_HANDOFF.md` remains untouched. No commit, push, dependency
  installation, system change, or external change was performed.

### Runtime acceptance evidence

- `_11`: Product 101 normal native-attachment cycle passed the readiness,
  ownership, stage, attachment/detachment, terminal empty-stow, and analyzer
  gates.
- `_13`: Product 102 normal native-attachment cycle passed the same gates;
  `GATE6_BAG_ANALYSIS=PASS product_id=102` is recorded after the corrected
  stage boundary.
- `_14`: navigation cancellation ended in fresh detached empty stow without
  home motion; manipulation cancellation with Product 102 retained failed
  closed in `FAULT` with no next-job or home motion. The all-topic recorder
  filled the disk before metadata finalization; its SQLite integrity check
  passed and compact observer/CLI summaries are retained.
- `_16`: graceful stop completed the active Product 101 delivery, left the
  queued Product 102 delivery unstarted, reported `graceful_stop=true`, and
  issued no home motion. The targeted 231,680-message bag finalized cleanly.

The nominal runs did not naturally produce a late action-response callback.
The source-level accepted-goal cancellation and terminal-proof contracts,
including the late-response branch, are covered by the registered focused
tests; no acceptance threshold or fail-closed rule was weakened.

Product 103/5 kg and Gate 7 remain outside the approved autonomous Product
101/102 scope. Do not enable or test them as part of this runtime gate without
separate authorization.

### Phase boundary

Phase 14 autonomous Product 101/102 runtime acceptance is complete. Product
103/5 kg and Gate 7 remain outside the approved scope and must not be enabled
from this handoff. Any future late-response investigation or additional
runtime repetition requires a new explicit phase decision and a bounded
retention plan for recorder data.

## Packet B — frontier cancellation atomicity and generation safety — planned, not implemented — 2026-09-09

The next bounded correction is Packet B for
`src/amr_exploration/scripts/frontier_explorer.py` and
`src/amr_exploration/test/test_frontier_lifecycle.py`. The current diagnosis is
that delayed timeout, readiness, and cancellation decisions observe state under
the lock, do work outside it, and then commit without proving that the same run
and motion still exist. Deterministic probes reproduced stale decisions that
could turn a completed stop into `FAULT`, fault a completed navigation, or let
an old stop callback report against a newer run.

The intended correction is to capture and revalidate the run generation, motion
token, and expected state around every delayed decision; make timer and
stop-service timeout commits use one lock-guarded helper; retain one completion
record per cancellation (motion identity, original deadline, event, and
terminal outcome); and return that recorded outcome from every repeated stop
caller. Start/stop services will use a dedicated mutually exclusive callback
group while action callbacks remain independently executable, so blocking stop
waits cannot starve cancellation proof callbacks. Completed outcomes must win
over stale timeout observations, a latched `FAULT` must remain terminal, and
late proof may release ownership only without restoring dispatch permission or
rewriting the recorded outcome.

Required deterministic coverage includes both timeout-boundary races,
readiness after navigation completion, obsolete decisions after a new run or
motion, restart before an old stop callback returns, repeated-stop sharing of a
single deadline/request/outcome, missing acknowledgement or terminal proof,
cancellation rejection and unexpected terminal results, late proof after a
timeout, diagnostics-before-waiter release, and an isolated ROS
executor/action-server check that repeated stops do not starve result or
cancellation callbacks. Existing pending-acceptance and cancellation-proof
ordering tests remain required.

Packet B was intentionally paused at the user's request. No Packet B
production or test edit had been made in that earlier continuation, and no
runtime, commit, push, dependency installation, system change, or external
change was performed there. The dirty worktree listed above remains user-owned;
preserve all unrelated changes and leave `AMR_CODEX_HANDOFF.md` untouched. The
resume and acceptance are recorded below.

The preceding text records the read-only plan carried forward from the paused
session. The previous implementation-attempt history remains an audit record;
the resumed work stayed within the bounded correction and the two approved
paths. After Packet B acceptance, the next planned work is Phase 15 Packet C:
artifact handling, the commissioning harness, runtime acceptance, and
conditional map promotion.

### Astra/high diagnosis and implementation packet — recorded 2026-09-09

At the user's explicit direction, Astra/high re-ran the Packet B diagnosis
against the current dirty worktree. The classification is SOURCE plus
TEST/EVIDENCE HARNESS, with high confidence for the deterministic reproductions
and an explicitly unverified live-executor starvation boundary. The partial
implementation has useful generation guards and callback groups, but four
coherence defects remain: a generic fault-latched stop response can bypass an
unfinished matching cancellation record; timeout commits can write an outcome
after their captured authority is stale; an early waiter return can fabricate
an outcome before the original deadline; and accepted-cancellation ownership
release is not governed by one proof-complete rule. These defects explain the
reproduced stale ownership, late-proof diagnostics, and repeated-stop boundary
failures.

The approved implementation packet is limited to
`src/amr_exploration/scripts/frontier_explorer.py` and
`src/amr_exploration/test/test_frontier_lifecycle.py`:

* resolve a matching cancellation record before generic stop responses, keep
  its stored tuple immutable, and detach old records when a new run or motion
  is admitted without clearing their waiter/event state;
* make timeout commit entirely conditional on current run, motion token,
  record identity, state, deadline, and elapsed-time proofs, with no outcome,
  event, ownership, or diagnostic mutation on a mismatch;
* replace stop's fallback outcome with a predicate-driven wait loop that waits
  only to the original deadline and treats early wakeups as rechecks;
* centralize accepted-cancellation release so positive acknowledgement plus an
  explicit terminal result is required, while rejection, missing proof,
  unknown/nonterminal statuses, and unexpected terminal results fail closed and
  retain the required motion ownership;
* preserve publication-before-waiter-release ordering and fault/reason
  latching, and add fake-clock race/proof-matrix regressions, including the
  existing pending-acceptance and late-proof cases. A bounded isolated ROS
  executor/action-server check is required if it can be added within these two
  files; otherwise its evidence remains explicitly unverified.

The non-goals are changing public services/actions, callback-group policy,
hardware values, fail-closed safety behavior, unrelated dirty paths, or
starting runtime hardware. Required evidence is focused lifecycle/ownership
pytest, Python compilation, the `amr_exploration` build and registered tests,
`colcon test-result --verbose`, `git diff --check`, and an independent Sol/high
review. Packet B remains unaccepted until those checks and review pass.

### Packet B implementation and acceptance — 2026-09-09

At the user's direction, Luna/max implemented the Astra/high correction in
`src/amr_exploration/scripts/frontier_explorer.py` and
`src/amr_exploration/test/test_frontier_lifecycle.py`. The implementation makes
cancellation-record identity immutable, prevents stale timeout mutation,
joins matching faulted stop records, replaces early-wakeup fallback outcomes
with deadline-predicate waiting, centralizes ACK/result release proofs, and
preserves publication-before-waiter release. It also adds a backwards-
compatible keyword-only ROS context seam and a real isolated two-thread ROS
executor/action-server regression for the previously unverified callback-group
boundary.

Fresh evidence for the final integrated state:

* focused lifecycle, exploration-contract, and workspace-contract pytest:
  44 passed, including 37 lifecycle cases;
* the real two-thread executor regression: 3 independent direct runs passed,
  plus Sol/high's same-process repeat/restoration checks;
* Python compilation and `git diff --check`: passed;
* `colcon build --packages-select amr_exploration --symlink-install`: passed;
* `colcon test --packages-select amr_exploration`: 3/3 registered tests passed;
* `colcon test-result --verbose`: 408 tests, 0 errors, 0 failures, 5 skipped;
* independent Sol/high review: pass, no material findings.

No hardware, external runtime graph, commit, push, dependency installation, or
system change was performed. The package-level runtime/hardware acceptance
remains outside this source packet. Packet B is accepted. `AMR_CODEX_HANDOFF.md`
was not modified; unrelated dirty paths remain preserved.

## Phase 15 Packet C — Sol/high diagnosis and implementation handoff — 2026-09-09

Phase 15 Slices 1 and 2 satisfy their current offline launch and ownership
contracts, but Sol/high found a bounded Packet C in the artifact and acceptance
layer. The current mapping CLI can report success for non-finite or incomplete
artifacts, has no provenance/hash binding, writes final artifacts before a
manifest transaction is complete, and lacks the observation-only commissioning
acceptance and promotion-eligibility state machine. Documentation also uses
`factory_attachment=false` even though the mapping launch requires `true`, and
does not document the autonomous exploration start boundary.

The approved source/test packet is limited to:

* `src/amr_factory/scripts/factory_mapping_artifacts.py` — new;
* `src/amr_factory/scripts/factory_mapping_cli.py`;
* `src/amr_factory/scripts/factory_mapping_acceptance.py` — new;
* `src/amr_factory/test/test_factory_mapping_cli.py`;
* `src/amr_factory/test/test_factory_mapping_acceptance.py` — new;
* `src/amr_factory/test/test_factory_demo_contract.py`;
* `src/amr_factory/CMakeLists.txt`;
* `src/amr_factory/package.xml` only for direct harness dependencies;
* `docs/PHASE_15_FACTORY_SLAM_COMMISSIONING.md` and
  `docs/SIMULATION_COMMANDS.md`.

Required behavior is provenance-safe run-specific artifacts: finite datum and
map metadata, resolution `0.05`, complete map YAML fields and thresholds,
nonempty regular image/pose-graph files, in-session non-symlink paths, canonical
map rejection, resolved paths/sizes/SHA-256/datum/schema in a manifest, atomic
manifest-last save, manifest-backed validation, and discard limited to
manifest-enumerated artifacts while preserving unrelated evidence. Failed saves
must not leave a valid manifest.

The new acceptance harness must observe an already-running graph only. It must
not launch processes, start exploration, publish velocity, or command motion.
Within the bounded deadline it must check SLAM Toolbox without AMCL/map-server,
fresh map and TF, EKF and command-arbitration ownership, adapters and fresh
stowed/detached manipulator authority, run-specific preflight evidence, and
manual/autonomous node and `/amr/mpc/cmd_vel` ownership expectations. It must
write `runtime_acceptance.yaml` atomically and preserve both pass and failure
observations without inventing a quantitative map-quality threshold.

Acceptance and promotion eligibility must require the explicit chain
`SAVED -> VALIDATED + RUNTIME_PASS + QUALITY_REVIEW_ACCEPTED ->
PROMOTION_ELIGIBLE`. A quality-review YAML binds the reviewer decision to the
candidate hash. `accept` verifies the artifact manifest, runtime report,
quality review, and exact candidate path. `promote` may write only a receipt
pointing to that run-specific candidate; it must never copy, replace, rename
over, or modify `src/amr_factory/maps/factory.*` or the installed canonical
map. Canonical replacement remains separately authorized.

Required tests cover malformed/NaN/Inf/missing/empty/symlink/escaped/canonical
artifacts, stale manifests and hash tampering, failed-save cleanup, forged
discard manifests, unrelated evidence preservation, fake-clock graph freshness
and ownership in both control modes, failed interlock, bounded timeout, and
matching artifact/runtime/quality hashes. Focused factory/SLAM/localization/
exploration/ownership pytest, package build/tests, verbose results, and diff
checks are required. No direct-host, Gazebo, hardware, or canonical-map
replacement operation is authorized in this implementation packet.

### Packet C implementation checkpoint — accepted 2026-09-09

At the user's request, Luna/max implemented the Packet C source layer in the
approved factory, test, build, dependency, and documentation paths. The change
adds `factory_mapping_artifacts.py`, hardens
`factory_mapping_cli.py`, adds the observation-only
`factory_mapping_acceptance.py` and its tests, registers/install-integrates the
new tools and tests, and reconciles the Phase 15 commissioning documentation.
It does not modify maps, localization/launch ownership, motion/control code,
the protected handoff, or canonical map files.

Earlier verification before the review pause:

* focused cross-package suite: 78 passed;
* factory package registered suite: 122 passed, 0 errors, 0 failures, 0
  skipped;
* `colcon build --packages-select amr_factory --symlink-install`: passed;
* target Python compilation, `ament_flake8`, and `git diff --check`: passed;
* installed `factory_mapping_cli.py --help` and
  `factory_mapping_acceptance.py --help`: passed.

The earlier independent Sol/high review was dispatched after the initial
implementation but was intentionally stopped at the user's request before a
verdict was returned; that historical pause is superseded by the final review
below.

After the final Packet C correction, the Luna/max implementation was
independently reviewed by Sol/high and accepted at the source/offline boundary.
Fresh final closeout evidence is recorded in the authoritative section above:
focused Packet C pytest 52 passed; `amr_factory` colcon tests passed 9/9;
`colcon test-result` reported 457 tests, 0 errors, 0 failures, and 5 skipped;
Python compilation, `ament_flake8`, the `amr_factory` build, and
`git diff --check` passed. No direct-host commissioning, ROS/Gazebo runtime,
hardware motion, canonical-map replacement, commit, push, dependency
installation, or system change was performed. Packet C is accepted for
source/offline closeout; runtime/hardware and canonical-map replacement remain
separate and unverified.

## Paused AWS simulation and rendering checkpoint — 2026-09-19

The user explicitly paused work after observing that Gazebo was black and then
running at approximately 1 FPS. Simulation success is the highest priority;
AWS implementation and Phase 15 closeout must not be treated as complete from
the source/offline results below. The live run was stopped at the user's
request. `AMR_CODEX_HANDOFF.md` was not modified.

### Worktree and implementation state

Before this handoff update, `git status --short` showed the following expected
dirty paths. Preserve unrelated existing work, especially
`src/amr_bringup/launch/amr_system.launch.py` and `phase14_evidence/`:

* `AGENTS.md`, `docs/SIMULATION_COMMANDS.md`, and `src/README.md`;
* `src/amr_simulation/CMakeLists.txt`, `package.xml`, the AWS and portable
  launch files, and their launch tests;
* new `src/amr_simulation/src/portable_robot_inserter.cpp` and
  `src/amr_simulation/test/test_portable_robot_inserter.cpp`.

The approved startup fix is now implemented as a native Gazebo Transport
blocking create request. The inserter reads `robot_description`, waits on
`/world/<validated-world>/create/blocking`, inserts the reserved `amr` entity,
and exits nonzero on timeout, negative response, or shutdown. The controller
spawner chain remains released only by a successful inserter exit.

The rendering implementation adds run-local Gazebo/XDG cache, config, runtime,
and log directories; explicit OGRE2/OpenGL GUI arguments; `QT_X11_NO_MITSHM`;
and a public `software_rendering:=auto|true|false` argument. In `auto`, the
launch detects accessible `/dev/dri/renderD*` nodes and enables llvmpipe only
when none are available. This was a bounded implementation attempt, not a
claim that GUI rendering or AWS runtime acceptance is complete.

### Fresh evidence

Focused offline checks completed:

* `colcon build --packages-select amr_simulation --symlink-install` passed;
* native inserter gtest passed 7/7;
* portable launch contract passed 24/24 with a writable `ROS_LOG_DIR`;
* `git diff --check` passed for the changed task paths;
* AWS contract test was 7 passed, 1 failed only because the existing
  `aws_warehouse.sdf` fixture is 7950 bytes while the stale test expects 7954
  (the old hash expectation is stale too). Do not silently change that fixture
  or its expected hash during rendering work.

AWS runtime `aws_warehouse_20260919_02` proved native insertion and clean arm,
gripper, and joint-broadcaster startup. Its separate portable runtime observer
failed on manipulation publisher identity (`not portable_stow_authority`), so
that observer result is not proof that insertion failed.

The fresh rendering run was `aws_warehouse_20260919_03`, with
`ROS_DOMAIN_ID=229`, `GZ_PARTITION=amr_aws_warehouse_20260919_03`,
`headless:=false`, `software_rendering:=auto`, `rviz:=true`, and
`auto_start_exploration:=true`. Its evidence was:

* native insertion succeeded and all four controller spawners exited cleanly;
* robot-state-publisher loaded the wheel links, including `left_wheel` and
  `right_wheel`;
* the Gazebo GUI process started and the run-local Mesa/Qt shader caches were
  populated under `.ros_logs/aws_warehouse_20260919_03/gazebo_runtime`;
* Gazebo still printed attempts to open `/home/pete/.gz/auto_default.log` and
  `/home/pete/.gz/sim/log/.../server_console.log`, despite the launch setting
  `GZ_LOG_PATH`; this logging-path issue remains unresolved;
* readiness remained blocked at `adapters_authority` waiting for active
  `base_adapter_node` in the captured run. The run was stopped with Ctrl-C;
  it did not pass the full AWS runtime gate.

The 1 FPS cause has concrete host evidence: `/dev/dri` is absent, and
`glxinfo -B` reports Mesa `llvmpipe`, `Accelerated: no`, after iris/DRM probe
errors. Therefore `software_rendering:=auto` selected software rendering.
This explains the severe GUI slowdown, but does not by itself prove the
original black viewport mechanism or justify weakening runtime gates.

### Required resume sequence

When the user grants driver access, resume with a fresh diagnosis/runtime
loop:

1. Re-run `git status --short`; preserve all unrelated dirty paths and do not
   edit `AMR_CODEX_HANDOFF.md`.
2. Ask/reuse Sol/high's pending analysis of the 1 FPS evidence before another
   source edit. The request was sent, but the agent was closed when the user
   stopped the session, so no verdict was returned.
3. Check `/dev/dri`, `glxinfo -B`, and the actual Gazebo child environment with
   the newly granted driver access. The discriminating prediction is that an
   accessible hardware renderer removes llvmpipe and materially improves GUI
   frame rate; if it does not, stop and return the new evidence to Sol/high.
4. Resolve the `/home/pete/.gz` log-path error and the GUI black/lag behavior
   with the smallest approved launch change. Do not change system configuration
   or install dependencies. Do not intensify synthetic workloads.
5. Run the AWS simulation again with a writable `ROS_LOG_DIR`, preserve the
   complete launch output, and stop at the first mandatory runtime failure.
   Simulation/runtime evidence is separate from build and aggregate test
   counts.
6. Only after the implementation attempt and a fresh simulation/system result,
   perform the independent Sol/high source review requested by the user. A
   passing runtime is not a substitute for that later review; a failed runtime
   must return to diagnosis before another source edit.

The portable observer identity failure, wheel visibility in RViz, AWS stale
fixture expectation, and Phase 15 human map review remain open and must be
re-tested or triaged separately after the rendering blocker is resolved.

## Paused frontier exploration fix and AWS runtime checkpoint — 2026-09-20

The user explicitly stopped work after requesting a handoff. The immediate
objective was full reachable-map exploration: the explorer must not stop with
unexplored map regions. `AMR_CODEX_HANDOFF.md` was not modified.

### Root cause and implementation

Run12 reproduced the exploration stall. Its paired bag is
`.ros_logs/aws_warehouse_20260919_12/frontier_evidence`. All 457 `/map`
messages had identical frame, metadata, and occupancy data (456/456
consecutive pairs unchanged; one unique data hash). All 328 raw global
costmaps were likewise identical. Planning took about 1.3–2.7 seconds while
map receipts arrived about once per second. The explorer incremented
`map_version` for every receipt and rejected a plan when the latest message
object/version/receipt time differed, even when the evidence content was
unchanged. Run12 therefore produced repeated
`frontier plan discarded because map evidence changed`, stayed at
`motion_generation=1`, and never dispatched the next frontier.

Sol/high diagnosed this with high confidence. Luna/max implemented the approved
bounded fix in only these files:

* `src/amr_exploration/scripts/frontier_explorer.py` — immutable planning
  snapshots, exact map/costmap content comparison at reservation, freshness
  checks against the latest receipts, and equivalent validation before
  no-candidate exhaustion accounting;
* `src/amr_exploration/test/test_frontier_lifecycle.py` — duplicate-publication
  dispatch and changed/invalid evidence regressions.

The fix preserves unknown/lethal endpoint checks, full-footprint collision
checks, minimum goal distance, authority/TF gates, cancellation ownership,
blacklists, thresholds, public status, and fail-closed behavior. It does not
mark incomplete exploration complete, alter launch/configuration, or weaken
costmap safety.

Offline evidence after implementation:

* focused lifecycle tests: `15 passed, 94 deselected`;
* full lifecycle suite: `106 passed, 2 xfailed, 1 xpassed`;
* `colcon build --packages-select amr_exploration --symlink-install`: passed;
* `git diff --check` for both changed files: passed.

These are source/offline results only and are not full-map acceptance.

### Runtime evidence

Run13 (`aws_warehouse_20260920_13`, ROS domain 219) did not release Explorer.
The final planner/smoother readiness process remained unmet for about five
minutes on `active lifecycle node: front_lidar_adapter_node`, while a separate
`ros2 lifecycle get /amr/front_lidar_adapter_node` returned `active [3]`.
Sol/high classified this as the intermittent pre-existing lifecycle response
problem seen in Run10, not evidence against the frontier patch. No source edit
was made for it. Logs are under `.ros_logs/aws_warehouse_20260920_13/`.

A single retry was authorized by that diagnosis. Run14 used:

```bash
source '/home/pete/sh&text/amr_cmd/amr_sim_setup.sh' aws_warehouse_20260920_14 218
ros2 launch amr_simulation aws_warehouse_exploration.launch.py headless:=false software_rendering:=auto rviz:=true auto_start_exploration:=true
```

Run14 passed all staged readiness gates and released Explorer. The paired
runtime bag is `.ros_logs/aws_warehouse_20260920_14/frontier_evidence` (114.93
seconds; 115 `/map` messages; 83 raw global costmaps; 503 exploration status
messages; 3 plans and 3 smoothed plans). The patch crossed its discriminating
boundary: the explorer reached `NAVIGATING` with `motion_generation=3`,
`active=true`, `goal_failures=0`, and candidate `46,348`; the old repeated
map-evidence discard loop was absent. This proves the admission-stall fix, not
full map exploration.

The first mandatory runtime failure was then:

```text
controller_server: Failed to make progress
mission_supervisor_node: Mission aborted: path following failed
frontier_explorer: navigation goal ended with status 6
```

The next attempts also hit smoother collision aborts near
`(-4.302157, 7.610850)` and `(-4.284661, 7.588587)`. Run14 was stopped at
that first runtime boundary as required. The map was not accepted as fully
explored. Do not treat the 3 dispatched goals or aggregate test totals as
map/simulation acceptance.

### Safe resume point

1. Re-run `git status --short`; preserve the existing dirty worktree. Never
   modify `AMR_CODEX_HANDOFF.md`.
2. Ask Sol/high to diagnose the fresh Run14 controller progress failure and
   smoother collision evidence before any source edit. The user requested
   Sol/high after a runtime failure, but explicitly stopped before that
   diagnosis was dispatched.
3. Inspect/reuse the Run14 bag and launch logs. Separate robot motion/pose
   evidence, path/smoother collision evidence, and frontier admission evidence.
4. Do not change the frontier snapshot-equivalence patch until Sol/high
   identifies a supported new root cause. Preserve fail-closed safety and do
   not make unknown/lethal cells safe or convert `INCOMPLETE` to `COMPLETE`.
5. After any approved implementation, run focused checks, then a fresh AWS
   simulation across the failed boundary. Only after implementation plus fresh
   runtime may the independent Sol/high source review occur.

Current state: all runtime processes and the Run14 bag recorder are stopped;
the frontier patch is present but full-map exploration, navigation progress,
and AWS/Phase 15 acceptance remain open.

## Provisional RCA and Luna/max implementation packet — 2026-09-20

The user asked to record the complete root-cause analysis and solution plan
before implementation. This is a provisional engineering record, not a
completed post-mortem: the fix has not yet been implemented, reviewed, or
validated in AWS runtime.

### Objective and acceptance target

Restore automatic exploration so the explorer continues through every
reachable, observable warehouse area and does not terminate with unexplored
frontiers. The acceptance target is a fresh AWS simulation that remains
non-faulted through the navigation boundary, advances costmap/TF evidence,
and reaches the existing full-reachable-map terminal gate. Unit, build, and
aggregate test totals are not substitutes for that runtime evidence.

### Observed failure

Run14 used:

```bash
source '/home/pete/sh&text/amr_cmd/amr_sim_setup.sh' aws_warehouse_20260920_14 218
ros2 launch amr_simulation aws_warehouse_exploration.launch.py headless:=false software_rendering:=auto rviz:=true auto_start_exploration:=true
```

The frontier snapshot fix removed the old duplicate-publication admission
stall: Run14 reached `NAVIGATING` at `motion_generation=3`. The run then
failed at the navigation boundary:

```text
controller_server: Failed to make progress
mission_supervisor_node: Mission aborted: path following failed
frontier_explorer: navigation goal ended with status 6
```

Two subsequent candidates were rejected by the smoother because the
rectangular robot footprint overlapped lethal cells, at approximately
`(-4.302157, 7.610850, 1.614230)` and
`(-4.284661, 7.588587, 1.872124)`. Those collision gates are valid safety
failures and must remain enabled.

The first controller failure occurred after the robot reached approximately
`(-4.54727, 6.98353)`. During the last ten seconds before the failure, the
recorded `odom -> base_footprint` position changed by less than 0.0001 m while
heading changed by about 3.8 rad. The configured installed
`nav2_controller::PoseProgressChecker` counts angular movement, so a simple
"the robot did not rotate" explanation is false.

### Confirmed root-cause mechanism

The installed ROS Humble TF2 libraries contain a lock-order deadlock in the
asynchronous transform-wait path used by Nav2 costmaps.

The involved paths are:

1. `tf2_ros::Buffer::waitForTransform()` holds
   `timer_to_request_map_mutex_` while calling
   `tf2::BufferCore::addTransformableRequest()`.
2. `addTransformableRequest()` acquires the TF2
   `transformable_requests_mutex_`.
3. When a new TF arrives, `tf2::BufferCore::testTransformableRequests()` holds
   `transformable_requests_mutex_` and invokes the registered callback.
4. The callback in `Buffer::waitForTransform()` tries to acquire
   `timer_to_request_map_mutex_`.

The resulting cycle is:

```text
waitForTransform: timer mutex -> transformable-request mutex
TF callback:      transformable-request mutex -> timer mutex
```

This was reproduced against the installed libraries
`ros-humble-tf2 0.25.23-1jammy.20260907.213559` and
`ros-humble-tf2-ros 0.25.23-1jammy.20260907.224752` with an isolated C++/GDB
harness. The debugger showed the waiter holding the timer mutex and blocked on
the request mutex, while the TF setter held the request mutex and blocked on
the timer mutex. The same harness completed when the calls were serialized.
No repository files were changed by that probe.

The operational consequence is that a private costmap TF/message-filter path
can stop consuming fresh transforms while external `/tf` continues to record
new data. This matches the preserved evidence: in Run09 the local published
footprint timestamp froze at sim time `431.53` while `/clock` advanced to
`489.19`; Run14 also showed a later frozen global-footprint timestamp. A
frozen controller-side local costmap/TF view can make the controller’s
progress checker repeatedly see the same pose and report failure even though
the external TF recording shows rotation.

The TF2 lock inversion is confirmed. Its attribution as the direct cause of
the Run14 progress failure is strongly supported by the frozen costmap
evidence but still requires a fresh runtime run with the repaired library.

### Evidence and falsified alternatives

Evidence paths:

* Run14 bag: `.ros_logs/aws_warehouse_20260920_14/frontier_evidence`;
* Run14 controller log:
  `.ros_logs/aws_warehouse_20260920_14/controller_server_177245_1789838406850.log`;
* Run14 smoother log:
  `.ros_logs/aws_warehouse_20260920_14/smoother_server_177008_1789838401226.log`;
* Run09 bag: `.ros_logs/aws_warehouse_20260919_09/nav_timing`.

The following alternatives are not the current root cause:

* `PoseProgressChecker` ignoring clockwise rotation — falsified by the
  installed binary: its `isRobotMovedEnough()` compares translation first and
  normalized angular distance second;
* global TF publisher stopping — falsified by advancing Run14 `/tf` samples
  and monotonically increasing transform timestamps;
* simulation clock jumping backward — falsified by the Run14 `/clock` record;
* centreline-only path safety — falsified by direct costmap inspection showing
  lethal cells under the full rectangular footprint at both smoother failures.

### Luna/max implementation packet

Use one bounded implementation attempt. Do not edit the frontier explorer or
navigation thresholds in this packet.

Allowed implementation surface:

* add a workspace-local `tf2` overlay under `src/vendor/geometry2/tf2/`, pinned
  to the installed upstream source version `0.25.23`;
* modify the overlay's `src/buffer_core.cpp` and indispensable TF2 regression
  test registration/files;
* document overlay activation and rollback in
  `docs/SIMULATION_COMMANDS.md`;
* do not modify `/opt/ros`, install packages, change system configuration, or
  modify `AMR_CODEX_HANDOFF.md`.

Required code change:

* remove the lock inversion at its source in
  `tf2::BufferCore::testTransformableRequests()` by collecting ready callbacks
  and invoking them only after releasing `transformable_requests_mutex_`;
* preserve request removal, callback ownership, result classification, and
  callback arguments; do not change public APIs or weaken any navigation gate.

Required focused tests:

* reproduce the two-thread lock-order interleaving and verify that
  `setTransform()` and the concurrent `waitForTransform()` both complete;
* retain the existing TF2 core tests for cache, storage, time, static cache,
  and core transform behavior;
* keep the test scheduling controlled and fail fast on a deadlock; do not add
  retries or weaken safety behavior to obtain a pass.

## Implementation checkpoint — 2026-09-20 (TF2 core backport complete)

The implementation boundary in the original packet was incorrect: it named a
`tf2_ros` overlay and `Buffer::waitForTransform()`. Rechecking the installed
source and the upstream Humble fix showed that the callback is actually invoked
under `tf2::BufferCore::testTransformableRequests()` while holding
`transformable_requests_mutex_`; the `tf2_ros` callback then acquires
`timer_to_request_map_mutex_`. The diagnosis is unchanged, but the correct
repair boundary is `tf2`, not `tf2_ros`. No `tf2_ros` source was modified.

The bounded implementation is present under `src/vendor/geometry2/tf2/`:

* `src/buffer_core.cpp` now removes ready requests and collects their callback
  data under the request mutex, then invokes callbacks after that mutex is
  released. This is the upstream Humble deadlock repair adapted to the
  installed 0.25.23 source layout.
* `CMakeLists.txt`, `package.xml`, and `test/tf2_ros_deadlock_test.cpp` add a
  deterministic two-thread regression test. The test fails fast after five
  seconds rather than hanging the test process.
* The existing benchmark registration is conditional because this host does
  not install `ament_cmake_google_benchmark`; benchmark tests still register
  whenever that extension is present. The core and deadlock tests remain
  enabled.
* `docs/SIMULATION_COMMANDS.md` records overlay activation, verification, and
  runtime-only rollback without deleting artifacts or changing `/opt/ros`.

Focused source validation passed:

```text
colcon --log-base log/tf2_deadlock_test_real build --packages-select tf2
  --symlink-install --build-base build/tf2_deadlock_test_real
  --install-base install/tf2_deadlock_test_real --allow-overriding tf2
  --cmake-args -DBUILD_TESTING=ON                         exit 0
ctest --test-dir build/tf2_deadlock_test_real/tf2 --output-on-failure
  12/12 passed, including test_tf2_ros_deadlock             exit 0
```

The same package was then installed into the normal workspace overlay with
`colcon build --packages-select tf2 --symlink-install --allow-overriding tf2
--cmake-args -DBUILD_TESTING=ON` (exit 0), followed by
`colcon test --packages-select tf2 --event-handlers console_direct+
--ctest-args --output-on-failure` (exit 0; the same 12 CTest targets passed).
`ros2 pkg prefix tf2` resolves to `/home/pete/amr_ws/install/tf2`, and `ldd`
confirmed the regression executable loads that overlay `libtf2.so` while using
the Humble underlay `libtf2_ros.so`.

The test command used `LD_LIBRARY_PATH` with the isolated overlay first, so
the passing regression exercised the patched `libtf2.so`. These are source and
library checks only; they do not prove AWS, Gazebo, controller, or full-map
acceptance. The initial `BUILD_TESTING=ON` configure failed only because the
benchmark extension was absent; the conditional registration removed that
environment-only configure blocker without weakening an available benchmark.

Fresh AWS runtime evidence has now crossed the repaired TF2 boundary in run
16 below. Full-map acceptance remains open, and any further source edit must
return to diagnosis from the new collision evidence. Independent Sol/high
source review is now eligible under the required post-runtime ordering, but is
not claimed in this checkpoint.

## AWS runtime checkpoint — 2026-09-20 (run 16)

The fresh retry used the repaired normal workspace overlay with:

```bash
source '/home/pete/sh&text/amr_cmd/amr_sim_setup.sh' aws_warehouse_20260920_16 221
export ROS_LOG_DIR=/tmp/amr_aws_warehouse_20260920_16/ros_logs
ros2 launch amr_simulation aws_warehouse_exploration.launch.py \
  headless:=true software_rendering:=false rviz:=false \
  auto_start_exploration:=true
```

The first GUI attempt in run 15 stopped at planner activation after a
`change_state` response timeout. It was stopped without a source change. The
headless retry removed that rendering contention hypothesis: robot insertion,
all staged readiness gates, planner/smoother, controller, mission supervisor,
and automatic Explorer startup all passed. The run reached two successful
controller goals and never reported `controller_server: Failed to make
progress`.

The first mandatory failure was a valid safety rejection from the existing
full-footprint smoother gate:

```text
Smoothed path leads to a collision at
(-4.359928, 8.646098, -0.941912)
(-4.396995, 8.674851, -0.892352)
(-4.371517, 8.674367, -0.942291)
Mission aborted: path smoothing failed or was incomplete
```

The Explorer then latched `FAULT` with `motion_generation=5` and three goal
failures. The smoother gate was not weakened and the map was not accepted as
fully explored. Evidence is preserved outside `.ros_logs` at
`/tmp/amr_aws_warehouse_20260920_16/`: the targeted bag is 24.4 MiB,
48.05 seconds, and 25,604 messages. Offline bag inspection found monotonic
local-footprint stamps over 46.8 seconds, global-footprint stamps over 48.0
seconds, map known-cell growth from 66,990 to 83,462, and advancing clock/TF
streams. This supports the TF2 repair prediction and isolates the remaining
runtime blocker to collision-constrained frontier navigation.

All run-16 processes were stopped cleanly after the first mandatory failure.
No source edit was made from this runtime failure. The next safe step is
read-only diagnosis of the rejected frontier/path geometry; do not alter the
footprint, lethal-cell, smoother, or frontier safety gates without a new
evidence-backed packet.

## AWS runtime checkpoint — 2026-09-20 (run 18 GUI retry)

At the user's request, a fresh GUI retry used a new run root and ROS domain:

```bash
source '/home/pete/sh&text/amr_cmd/amr_sim_setup.sh' aws_warehouse_20260920_18 223
export ROS_LOG_DIR=/tmp/amr_aws_warehouse_20260920_18/ros_logs
ros2 launch amr_simulation aws_warehouse_exploration.launch.py \
  headless:=false software_rendering:=auto rviz:=true \
  auto_start_exploration:=true
```

Gazebo and RViz both started successfully. The staged readiness gates,
planner/smoother, controller, mission supervisor, and automatic Explorer
startup all passed. Three controller goals reached success before the first
mandatory failure. The smoother then rejected the path at:

```text
Smoothed path leads to a collision at x: -4.233123, y: 7.796606, theta: 2.026463
Mission aborted: path smoothing failed or was incomplete
```

Two subsequent retries were rejected in the same local corridor
(`x=-4.307748,y=7.605045` and `x=-4.310397,y=7.602297`). This is consistent
with a deterministic collision-constrained frontier/path boundary, not the
previous startup timeout or a TF/costmap freeze. The full safety gate was not
weakened. The run-specific launch and node logs are preserved at
`/tmp/amr_aws_warehouse_20260920_18/ros_logs/`.

The separately started targeted bag is invalid for runtime evidence: its
metadata reports zero messages. Do not use that bag to claim topic freshness
or map coverage. The launch logs are valid evidence for startup, goal success,
and the smoother failure. All run-18 processes were stopped cleanly and no
source edit followed the failure.

### Validation packet and prediction

Before runtime, Luna must recheck `git status --short`, inspect the complete
overlay diff, run the focused TF2 tests, and build/test the overlay in isolated
build/install directories with testing enabled. Report exact commands,
statuses, changed files, and residual risks.

After those checks and separate runtime authorization, activate the overlay
last, verify the controller/costmap process loads the overlay library, and run
AWS auto exploration with at least `/tf`, `/clock`, local/global published
footprints, local/global costmaps, wheel odometry, ground-truth pose, command
topics, navigation action status, and exploration status recorded.

Falsifiable prediction: with the repaired library, costmap footprint
timestamps continue advancing while the robot rotates; the controller does
not report `Failed to make progress` at the prior boundary; navigation can
reach later frontier generations. The smoother must still reject genuinely
lethal full-footprint paths.

Stop and return to diagnosis if the overlay cannot be built against the
installed ABI, if a test exposes a changed TF2 contract, if the fresh runtime
still freezes costmap evidence, or if the first mandatory AWS gate fails.
Do not apply a second source patch without re-diagnosis.

### Runtime and review order

`.ros_logs` is currently approximately 2.2 GB, above the documented 1 GB
retention limit. Do not delete or overwrite evidence without explicit user
direction; runtime must wait for a safe retention decision or an approved
run-specific evidence location.

The required order remains:

```text
confirmed diagnosis -> Luna/max implementation -> focused validation
-> fresh AWS runtime across the failed boundary -> independent Sol/high review
```

A passing source suite is not simulation acceptance. A passing runtime is not
the later independent source review. Full-map acceptance remains open until
automatic exploration reaches the existing reachable-map terminal condition.

Current worktree at handoff time is intentionally dirty in the paths reported
by `git status --short` immediately before this update. All unrelated changes
must be preserved. The TF2 implementation and focused source validation are
complete; no dependency installation or external write was performed. AWS and
Gazebo runtime evidence is preserved under the run-specific `/tmp` roots
documented above. `AMR_CODEX_HANDOFF.md` remains untouched.

## AWS runtime checkpoint — route-admission implementation attempts (2026-09-20)

The user authorized implementation of the confirmed frontier failure. Sol/high
was consulted before editing and after each fresh runtime failure. The source
changes are limited to the existing dirty files
`src/amr_exploration/scripts/frontier_algorithm.py` and
`src/amr_exploration/scripts/frontier_explorer.py`; no new tests were added.

The first route-admission attempt added an eight-heading, footprint-checked
costmap search. AWS run 19 reached all startup/readiness/Nav2 gates, but
`frontier_explorer` remained at 100% CPU for 1:42, emitted no exploration
status, and dispatched no goal. The valid bounded bag is
`/tmp/amr_aws_warehouse_20260920_19/evidence/frontier_evidence` (16 MiB); the
run was stopped cleanly.

The second attempt replaced that search with one two-dimensional parent BFS,
then checked only reconstructed candidate paths with the full footprint. The
focused algorithm suite passed 49/49, a 275x414 timing probe completed in
0.215 seconds, the scoped `amr_exploration` build exited 0, and AWS run 20
reached the smoother boundary without collision or progress failures. Two
navigation goals succeeded, but exploration terminated `INCOMPLETE` with
`exploration incomplete: no safe costmap-valid frontier remains` while the
final map still contained 30,173 unknown cells (about 26.4%). The valid bag
is `/tmp/amr_aws_warehouse_20260920_20/evidence/frontier_evidence` (44.7 MiB,
59,245 messages).

Offline replay of the final run-20 map/costmap showed the immediate remaining
mechanism: endpoint-only admission retained two candidates; center-only BFS
could reach both; the single reconstructed full-footprint parent paths were
rejected at early diagonal segments. That proves candidate starvation, not
that every alternate route or frontier cell is unsafe. The map/costmap still
contains substantial unknown/lethal regions, so actual warehouse reachability
is not yet determined.

The two implementation attempts for this route-admission hypothesis are
exhausted under the debug-loop limit. Do not make a third source patch on this
hypothesis. The safe resume point is a read-only feasibility analysis of the
run-20 frozen map/costmap: enumerate alternate cells within the remaining
frontier clusters and determine whether any footprint-safe route exists. If a
safe alternate exists, create a new diagnosis/implementation packet. If none
exists, obtain user direction on warehouse access or the coverage target while
retaining the lethal-collision and failure gates. `AMR_CODEX_HANDOFF.md` was
not modified.

## AWS runtime checkpoint — portable stow acknowledgement recovery (2026-09-20)

Sol/high diagnosed Run24 as an upstream portable-stow action transport gap:
the one-shot arm goal could be accepted by `arm_controller` while the client
never received its goal acknowledgement, so the authority could not obtain
result proof and readiness remained closed. The approved implementation was
limited to `src/amr_simulation/scripts/portable_stow_authority.py`:
generate one explicit goal UUID, send exactly one trajectory, and—only after a
bounded missing acknowledgement—make at most one result-service query for that
same UUID. Only `SUCCEEDED` plus `FollowJointTrajectory.SUCCESSFUL` can grant
stow proof; missing status/error code, unknown, canceled, aborted, unavailable,
or late duplicate callbacks remain fail-closed. The normal acknowledged-goal
result wait was left unchanged.

Focused validation passed:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_simulation/test/test_portable_stow_authority.py
18 passed
colcon build --packages-select amr_simulation --symlink-install --event-handlers console_direct+
exit 0
```

The first implementation attempt was returned to Sol/high after the existing
textual one-send contract failed; the correction passed all 18 checks. No new
tests were added. Independent source review remains deferred until a fresh
runtime exercises the failed boundary.

## AWS runtime checkpoint — Run25 stow pass, frontier evidence gap (2026-09-20)

Run25 was a fresh GUI AWS run using ROS domain 230 and a separate Gazebo
partition. Evidence is preserved at
`/tmp/amr_aws_warehouse_20260920_25/`; the bounded bag is
`/tmp/amr_aws_warehouse_20260920_25/evidence/frontier_evidence` (90.1 MiB,
170.22 s, 108,717 messages). The unrelated user-owned TurtleBot3 process was
left untouched.

The repaired stow boundary passed in runtime. The arm action log records goal
receipt/acceptance at `1789890208.0576` and `Goal reached, success!` at
`1789890210.0636`. The authority published valid `STOWED_EMPTY` with
`base_motion_allowed=true`; both raw joint-state topics were recorded
(`/amr/base/joint_states`: 15,507 messages,
`/amr/simulation/base/joint_states`: 16,539). Portable readiness, planning,
controller, mission, and Explorer startup all passed. One navigation goal was
received at `1789890267.5289` and reached at `1789890277.4305` with no goal
failure or latched fault.

The run then safely terminated with:

```text
state=INCOMPLETE
reason=exploration incomplete: no safe costmap-valid frontier remains
map_version=132 motion_generation=1 goal_failures=0
no_frontier_updates_seen=3 pending=false active=false fault_latched=false
```

The recorder initially captured `/amr/global_costmap/costmap`, which is the
visualization `nav_msgs/msg/OccupancyGrid`. The Explorer actually consumes
`/amr/global_costmap/costmap_raw` as `nav2_msgs/msg/Costmap`; that raw input was
not recorded. Therefore Run25 proves the stow/startup/navigation boundary and
the incomplete terminal state, but it cannot distinguish endpoint exhaustion
from full-footprint route rejection. Sol/high classified the frontier result
as non-diagnostic and approved no exploration source packet.

Safe resume point: obtain separate user authorization for one fresh run that
records `/amr/global_costmap/costmap_raw` before Explorer starts, together with
`/map`, `/tf`, raw joint states, and `/amr/exploration/status`. If the failure
recurs, replay the three decision-time snapshots at the recorded robot pose
and count frontier cells, endpoint-safe cells, and route-admitted cells. If
the raw topic is missing again or the failure does not recur, stop without a
source patch and report the evidence gap. `AMR_CODEX_HANDOFF.md` remains
untouched.

## AWS exploration-stop implementation plan — 2026-09-20

The user asked to record the approved plan before continuing implementation.
This is a plan-only checkpoint: no source edit, simulation launch, or runtime
claim is made by this section.

### Objective and current diagnosis

Restore automatic AWS exploration so it continues through every reachable,
observable warehouse area and reaches the existing `COMPLETE` terminal gate,
without weakening Nav2 collision or Explorer fail-closed behavior.

Run25 reached the stow, readiness, planning, controller, mission, Explorer,
and first navigation-goal boundaries successfully. It then entered
`INCOMPLETE` after three map updates with no candidate, while the map still
contained unexplored cells. The recorded bag contained the visualization
`/amr/global_costmap/costmap` but not the raw
`/amr/global_costmap/costmap_raw` consumed by the Explorer, so the runtime
failure is not yet fully diagnostic.

Offline replay of the Run25 visualization shows the strongest current
hypothesis: the robot center is cost 0, but its padded footprint intersects
cost 253 (`INSCRIBED_INFLATED_OBSTACLE`) and no 254/255 cells. The current
Explorer rejects 253 in both full-footprint admission and route masks, reaches
no route, and then terminates after the existing three-update limit. Run23
confirmed that visualization 99 corresponds to raw 253, 100 to raw 254, and
-1 to raw 255 for the blocked categories, but Run25 still needs a fresh raw
costmap capture for confirmation.

### Approved implementation contract

The selected Nav2-compatible policy is:

* center, diagonal, and endpoint checks continue to reject cost `>=253`;
* full-footprint polygon checks and route-search footprint masks reject only
  `>=254`, allowing 253 inflation/padding overlap;
* 254 lethal obstacle and 255 unknown remain blocked everywhere in the
  footprint checks;
* footprint geometry, map/costmap freshness and frame validation, boundary
  rejection, navigation ownership, cancellation, failure thresholds, and
  terminal-state semantics remain unchanged.

The implementation packet is limited to the existing Explorer algorithm and
its existing focused contract cases. No Nav2 configuration, smoother gate,
mission boundary, or unrelated package is in scope. No new test file is to be
added; existing tests should be adjusted only where they encode the changed
253-versus-254 contract. Historical collision cases must remain represented by
254/255, and no safety test may be weakened solely to obtain a pass.

### Execution and validation order

1. Before editing, recheck `git status --short`, preserve all unrelated dirty
   paths, and leave `AMR_CODEX_HANDOFF.md` untouched.
2. With separate user authorization for a simulation run, record raw
   `/amr/global_costmap/costmap_raw` before Explorer startup together with
   `/map`, `/tf`, raw joint states, `/amr/exploration/status`, and navigation
   action status. Verify the raw topic contains messages. Replay the decision
   snapshot and confirm the predicted 253-only footprint blockage. If raw
   evidence is missing or contradicts the prediction, stop and return to
   diagnosis without editing source.
3. Implement one bounded source packet in
   `src/amr_exploration/scripts/frontier_algorithm.py` and update only the
   affected existing algorithm assertions. Inspect the complete diff.
4. Run focused validation with exact commands and statuses, including:
   `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider
   src/amr_exploration/test/test_frontier_algorithm.py`, the scoped
   `amr_exploration` build, and `git diff --check`. Replay the raw snapshot;
   the prediction is that a route-admitted candidate appears while center 253
   and footprint 254/255 cases remain blocked.
5. After the implementation and focused checks, and only after the user
   authorizes it, run fresh AWS auto-exploration. Acceptance requires continued
   progress through reachable shelf areas and the existing `COMPLETE` state;
   one successful goal, a passing source suite, or an aggregate test count is
   not simulation acceptance. Preserve bounded evidence and stop at the first
   mandatory runtime failure.
6. A fresh runtime failure returns to Sol/high diagnosis before any further
   source edit. After the implementation has exercised the failed runtime
   boundary, perform the deferred independent Sol/high source review. Record
   commands, exit statuses, runtime gates, changed files, and remaining risks
   in the next handoff update.

### Stop conditions and resume point

Do not patch if the raw-costmap confirmation is unavailable or non-diagnostic.
Do not bypass smoother collision failures, progress failures, stale evidence,
or the Explorer’s fail-closed terminal gates. Do not start the simulation
automatically from this handoff update. The safe resume point is raw-costmap
capture and replay, followed by the single approved implementation packet if
the prediction is confirmed. `AMR_CODEX_HANDOFF.md` remains untouched.

## AWS exploration implementation attempt — Run26 startup stop — 2026-09-20

The user authorized implementation of the recorded plan and instructed that
work stop at the first failure. The attempt stopped before any source edit
because the required raw-costmap confirmation run failed at portable startup.

### Command and evidence

The run used a fresh domain and Gazebo partition:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export GZ_VERSION=harmonic
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOCALHOST_ONLY=1
export AMR_RUN_ID=aws_warehouse_20260920_26
export GZ_PARTITION=amr_aws_warehouse_20260920_26
export ROS_DOMAIN_ID=231
export ROS_LOG_DIR=/tmp/amr_aws_warehouse_20260920_26/ros_logs
ros2 launch amr_simulation aws_warehouse_exploration.launch.py \
  headless:=true software_rendering:=false rviz:=false \
  auto_start_exploration:=true
```

The bounded recorder started before launch at
`/tmp/amr_aws_warehouse_20260920_26/evidence/raw_costmap`. The launch log is
under `/tmp/amr_aws_warehouse_20260920_26/ros_logs/`.

### First mandatory failure

Gazebo inserted the robot and the stow action reached success, but the
portable readiness authority remained closed. The wheel odometry lifecycle
transition emitted:

```text
failed to send response to /amr/wheel_odometry_node/change_state (timeout)
```

Readiness continued to report `active lifecycle node: wheel_odometry_node`
missing through simulated timestamp `1789916985.332671589`, about two minutes
after startup. `/amr/base/status` and `/tf` were present, but `/map` and
`/amr/global_costmap/costmap_raw` never became available. Therefore the raw
costmap hypothesis was not exercised and Run26 cannot validate or falsify the
planned 253/254 change.

The run and recorder were stopped cleanly after this startup failure. No
frontier source or test file was changed by this attempt. The headless launch
also reported the known `/dev/dri`/Mesa fallback and Gazebo log-directory
permission warnings; these are recorded as environment evidence, not treated
as proof of the frontier diagnosis.

### Required next action

Return this evidence to Sol/high for diagnosis of the wheel-odometry startup
timeout before making another source edit or starting another simulation.
Do not apply the frontier 253/254 packet until a fresh run reaches readiness
and records `/amr/global_costmap/costmap_raw`. Preserve the existing dirty
worktree and leave `AMR_CODEX_HANDOFF.md` untouched.

## AWS exploration evidence audit and same-session runtime — 2026-09-21

The Run35 raw-costmap evidence was not deleted. The intact SQLite database at
`/tmp/amr_aws_warehouse_20260920_35/evidence/raw_costmap/raw_costmap_0.db3`
was created before the launch, contains the ten requested topic definitions,
and contains zero message rows. It has no WAL/journal or metadata, and its
directory entry did not change after creation. The recorder was interrupted
before its clean `Recording stopped` finalization. A minimal domain-225 probe
reproduced the delivery problem across separate execution sessions: topic
discovery worked, but a subscriber and a zero-cache recorder received no
messages. A same-session probe received 10 messages and wrote valid metadata.
The cause is cross-session ROS/DDS delivery isolation combined with
non-graceful recorder shutdown, not deletion by Luna or the agent.

The corrected same-PTY AWS capture `aws_warehouse_20260921_01` used
`ROS_DOMAIN_ID=224`, started recorder, observer, and launch as children of one
execution session, and preserved the raw database at
`/tmp/amr_aws_warehouse_20260921_01/evidence/exploration/exploration_0.db3`.
Direct SQLite inspection after shutdown found 151 raw costmaps, 292 maps, 399
exploration-status messages, 4 navigation-action status messages, 19,569
`/clock` messages, and 15,369 `/tf` messages. The last map had 29.6805%
unknown cells. Its last raw costmap had 52,393 cost-0 cells, 35,571 cost-253
cells, 4,189 cost-254 cells, and 12,992 cost-255 cells. The database is
integrity-valid but lacks `metadata.yaml` because the parent shell was
interrupted before its cleanup block; the raw rows remain preserved.

The run passed startup and two navigation goals, then stalled at the first
mandatory freshness boundary: 60 distinct planning-to-scan transitions
discarded a nonempty candidate because the carried TF was stale. Those
intervals were 1.41–2.35 simulated seconds, beyond the existing 1.0-second
freshness limit. No third goal, collision/progress failure, `INCOMPLETE`, or
`COMPLETE` decision occurred; `no_frontier_updates_seen` remained zero.
Sol/xhigh classified this as a planning/freshness liveness failure. It neither
confirms nor falsifies the Run25 cost-253-only footprint hypothesis.

No frontier source or test file was changed. Stop further AWS reruns and source
edits at this boundary. The 253/254 packet remains gated on a decision-time
`INCOMPLETE` snapshot with raw costmap, map, TF, exploration status, and
navigation status. `AMR_CODEX_HANDOFF.md`, unrelated dirty work, and all raw
evidence remain untouched.

### Sol/xhigh stale-TF reconciliation — 2026-09-21

The preserved bag cannot timestamp Explorer's private TF lookup and candidate
search calls separately. It does establish the boundary: each of the 60
stale-TF discards followed a nonempty candidate search, while fresh upstream
`map->odom` and `odom->base_footprint` TF samples were present (maximum observed
ages 0.09 s and 0.03 s). The action-server wait is bounded at 0.2 s; offline
replay of the frozen map/costmap snapshot found a candidate under the current
253 policy, with heading-aware route search dominating the profiled work.

The focused baseline command

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_exploration/test/test_frontier_lifecycle.py \
  -k 'real_executor_diagnoses_tf_during_frontier_planning or reservation_discards_plan_when_carried_tf_expires_during_action_wait'
```

exited 1 because the stale-TF fixture produced no candidate and reached
`no costmap-valid frontier in this map update`; the real executor probe is
explicitly xfail. This is non-diagnostic baseline evidence, not a regression
to patch. No freshness implementation packet is ready: increasing
`tf_timeout_sec`, retimestamping an old transform, or dispatching its route
would weaken the existing freshness or current-pose route-admission contract.
The 253/254 packet remains gated, and no further AWS run or source edit is
authorized by this diagnosis.

---

## Post-archive latest stop point

# AMR Session Handoff

**Updated:** 2026-09-21
**Analysis/review:** Sol/xhigh
**Implementation:** Luna/max, one packet at a time

## Objective

Finish the portable AWS warehouse exploration work and obtain a fresh runtime
that reaches the existing `COMPLETE` gate. Preserve fail-closed behavior,
ownership boundaries, public interfaces, safety gates, and documented hardware
values. Keep `AMR_CODEX_HANDOFF.md` untouched.

## Current worktree

The repository has substantial pre-existing dirty work from the portable
exploration phase. Preserve unrelated changes, `phase14_evidence/`, and
untracked vendor/runtime files. Do not reset, normalize, stage, commit, push,
or rewrite history without explicit authorization.

Relevant implementation packets already present in the worktree include:

- Frontier footprint checks allow cost `253` overlap while center, diagonal,
  endpoint, `254`, and `255` checks remain fail-closed.
- Frontier heading resolution uses 32 masks with corrected angle wrapping.
- Explorer lookup-derived TF samples use a 1.0 s future tolerance; explicit
  and carried transforms remain strict.
- Portable stow joint freshness is 2.0 s; base authority freshness remains
  0.2 s.
- Portable goal timeout is 600 s and startup grace is 30 s.
- Controller startup uses the package-local spawner wrapper so the requested
  service timeout reaches `load_controller`.
- The latest packet changes only
  `PORTABLE_BASE_INPUT_TIMEOUT_MS` from `2000` to `3500` in
  `src/amr_simulation/launch/portable_exploration.launch.py` and its launch
  test. `PORTABLE_BASE_GATED_COMMAND_TIMEOUT_MS` remains `1500`.

## Latest packet verification

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_simulation/test/test_portable_exploration_launch.py
26 passed, 1 warning

source /opt/ros/humble/setup.bash && source install/setup.bash && \
ROS_DOMAIN_ID=180 colcon build --packages-select amr_simulation --symlink-install
passed

git diff --check -- \
  src/amr_simulation/launch/portable_exploration.launch.py \
  src/amr_simulation/test/test_portable_exploration_launch.py
passed
```

The full repository diff check has only the pre-existing unrelated trailing
space in `src/amr_bringup/launch/amr_system.launch.py`.

## Diagnosis behind the latest packet

Run59 and lean Run61 both reached readiness, accepted a navigation goal, and
then lost Explorer motion authority during simulation feedback starvation.
Raw odometry/joint receipt gaps reached about 3.36 s; the base adapter emitted
invalid status during those gaps. The lean recorder reproduced the behavior,
so recorder load was not the cause. Raising the simulation input deadline to
3.5 s is the bounded hypothesis. The 1.5 s command gate was deliberately
preserved.

## Runtime stop point

Run62 used a same-session recorder and headless AWS launch with
`ROS_DOMAIN_ID=182` at `/tmp/amr_aws_warehouse_20260921_62`. Adapter and SLAM
readiness passed; planner/controller startup was still in progress when the
user stopped the session. It was interrupted before an Explorer terminal state
was recorded, so it is neither `COMPLETE` nor runtime acceptance evidence.
The bag was not inspected after interruption and may be partial.

The host contained the active Run62 process group plus eighteen stale AWS
runtime sandbox groups from earlier attempts. Those stale groups and Run62
were terminated; the Codex session itself remains open.

## Acceptance gates for the next runtime

Run one fresh AWS exploration in a single execution session with recorder and
launch together. Stop at the first mandatory failure and return to Sol/high
diagnosis before any new edit. Require nonzero rows for raw costmap, map, TF,
exploration status, and navigation status; continued mapping through reachable
shelf areas; no authority/freshness/terminal fault; and the existing
`COMPLETE` gate. Runtime evidence is separate from unit tests and build
success.

After the runtime attempt, perform the deferred independent Sol/xhigh source
review. Do not simplify Explorer navigation ownership or remove custom route
search in this packet; that is a separate design and implementation packet.

## Safe resume point

Do not restart AWS exploration automatically. First verify no stale ROS/Gazebo
process groups remain and determine whether the Run62 recorder finalized.
Then diagnose the interrupted runtime evidence. If the latest hypothesis is
falsified or inconclusive, stop production edits and report the evidence.

---

## Historical snapshot — 2026-09-21 — Corrected AWS diagnosis and Luna/max plan

This snapshot preserves the plan that is currently maintained in
`SESSION_HANDOFF.md`. The current file is latest-only; this section keeps the
full plan available after later handoff updates.

### Objective

Finish portable AWS warehouse exploration and obtain a fresh run that reaches
the existing `COMPLETE` gate. Preserve fail-closed behavior, ownership
boundaries, public interfaces, safety gates, and documented hardware values.
Keep `AMR_CODEX_HANDOFF.md` untouched.

### Corrected diagnosis

Run64 used one recorder and one headless AWS launch with `ROS_DOMAIN_ID=182`
and `GZ_PARTITION=amr_aws_warehouse_20260921_64`. It reached two goals, then
repeatedly discarded plans because refreshed TF was stale or unavailable. A
preserved snapshot admits frontier `(59, 356)`, but Python route selection
took 3.31 s cold and 3.17 s warm; heading-aware route search dominates the
profile. Upstream recorded TF was current at the published discard times, so
the exact private lookup/receipt failure is still unproven. Do not increase a
freshness timeout or add executor threads until that boundary is measured.

The recorder exception was misattributed. Run64's recorder log ended with
`SQLite error (5): database is locked`. The log was last written about 163
seconds after topic discovery. An isolated rosbag2 probe reproduced the same
error only when a read cursor was held on the active SQLite file; closing the
reader restored writes. Never open an active bag with SQLite, `rosbag2_py`,
`ros2 bag info`, or an integrity query. Finalize the recorder before any
inspection. Run64 is not acceptance evidence because it lacks a terminal
result and finalized metadata.

Host inspection also found Run63 still alive: its wrapper was waiting while
Gazebo, Explorer, navigation, and adapter descendants remained in process
group `713820`. The old wrapper does not reliably terminate descendants.

The current `SmacPlanner2D` change is not footprint-equivalent to the custom
route check. Installed SmacPlanner2D configures `GridCollisionChecker` with
radius mode; it does not prove the rectangular footprint used by the
Explorer/smoother. The navigation package also lacks an explicit
`nav2_smac_planner` dependency. Treat the existing YAML change as unaccepted
until replay and runtime evidence prove planner/smoother compatibility.

The route search has a confirmed safety gap: first-step validation checks the
current and movement orientations but skips the swept turn between them. A
fixture with a lethal cell at an intermediate heading admitted a route. Four
existing lifecycle tests currently fail because their fixtures place the robot
outside the tiny costmap; a separate test helper can generate forbidden ROS
domains 233–239.

### Luna/max implementation plan

Run packets sequentially. Before each packet, Luna must run `git status
--short`, reread the exact targets, preserve unrelated dirty work, inspect the
complete scoped diff, and run focused validation. If evidence contradicts a
packet, Luna stops and returns to Sol/xhigh.

#### Packet 1 — Make AWS runtime ownership and evidence trustworthy

Allowed paths: a maintained AWS runner/monitor under
`src/amr_simulation/scripts/`, their existing/new tests,
`src/amr_simulation/CMakeLists.txt`, and `docs/SIMULATION_COMMANDS.md`.

The runner must validate a unique run directory and domain 0–232; start
recorder, monitor, diagnostics, and launch under one owned process session;
record child process groups and exit identities; preserve the first failure;
classify `COMPLETE`, `INCOMPLETE`, `FAULT`, recorder failure, observer failure,
interruption, and launch exit separately; detect smoothing/path-following
abort logs; and bound graceful shutdown/escalation for every descendant.
Only `COMPLETE` with no active/pending/cancel-owned motion can be a pass.
Never inspect the active SQLite bag. Finalize and verify all run-owned
processes have exited before post-run bag checks.

Tests must cover terminal classification, recorder crash, monitor exit,
surviving descendants, cleanup escalation, invalid/reused run identity, and
attempted live-bag inspection. Prediction: a stopped run leaves no owned
processes and a recorder failure cannot be mistaken for acceptance.

#### Packet 2 — Repair lifecycle regression fixtures and domain bounds

Allowed path: `src/amr_exploration/test/test_frontier_lifecycle.py`.
Repair only the four failing fixtures so robot pose, footprint, and expected
route fit valid geometry; retain negative out-of-map rejection cases; and
constrain all generated test domains to 0–232. No production behavior changes.

#### Packet 3 — Validate the initial swept turn

Allowed paths: `src/amr_exploration/scripts/frontier_algorithm.py` and its
existing test file. Pass the actual starting yaw into first-step feasibility;
sample the shortest wrapped turn to each first movement heading with the
existing exact footprint check; and apply the same proof in refreshed
route-start validation. Preserve center/diagonal/endpoint rejection at 253,
footprint rejection at 254/255, and footprint-only permission for 253.
Regression: the constrained intermediate-heading collision must be rejected,
while the clear-turn and Run03 253 cases remain admissible.

#### Packet 4 — Separate costmap content from update time

Allowed paths: `frontier_explorer.py` and its lifecycle test. Exclude only
`metadata.update_time` from content equality. Keep frame, geometry, layer,
creation identity, cell data, receipt freshness, and ROS timestamp checks.
Test identical fresh data with a newer update time plus geometry/data/freshness
changes. This is a confirmed latent defect, not the proven cause of Run64.

#### Packet 5 — Instrument the TF admission boundary

Allowed paths: `frontier_explorer.py`, its lifecycle test, and the maintained
diagnostic monitor. Add diagnostics disabled by default for generation, wall
and ROS times before/after clustering, route search, lookup, refresh, action
wait, and reservation; returned TF stamps and validation ages; lookup errors;
map/costmap versions; evidence-change reasons; authority receipt ages; and
reservation duration. Run one clean AWS attempt using Packet 1 and stop at
the first failure. Classify whether lookup returns old data, lookup throws,
the sample ages during measured work, or evidence/authority changes. No
timeout increase, executor change, or retimestamping is allowed from this
packet alone.

#### Packet 6 — Prove a planner with the required footprint behavior

First add an offline replay harness/test for preserved Run63 map, costmap,
start, and goal snapshots. Compare installed SmacPlanner2D with the
installed SmacPlannerLattice differential-drive primitive set, checking the
actual rectangular footprint and then the existing collision-checked
smoother. Include blocked start/goal, unknown space, narrow turns, reachable
shelf routes, final orientation, and controller compatibility. Do not relax
inflation, collision checks, or controller safety limits.

Only after replay passes may Luna change the navigation YAML/launch/package
dependency and contract test. If the lattice candidate fails replay or
controller compatibility, stop and return evidence; do not substitute another
planner speculatively.

### Validation and runtime acceptance

Use focused tests first, then affected-package symlink builds and scoped
`git diff --check`. Aggregate totals are not runtime proof. Before AWS, clean
only the verified old Run63 process group, confirm the TF overlay and startup
wrappers resolve from the workspace, and choose an unused domain/partition.

Final acceptance requires one uninterrupted AWS run reaching `COMPLETE`,
continued mapping through reachable shelves, no authority/freshness/terminal
fault, nonzero finalized raw-costmap/map/TF/exploration/navigation rows,
valid bag metadata/database, and no surviving run-owned processes. Stop at
the first mandatory failure, preserve evidence, and return to Sol/xhigh before
any new edit or run. Independent Sol/xhigh source review follows the runtime
attempt.

### Worktree rules and records

Preserve all unrelated dirty files, `phase14_evidence/`, vendor/runtime files,
and `SESSION_HANDOFF_HISTORY.md`. Keep `SESSION_HANDOFF.md` latest-only; put
detailed historical records in this file or a separate AWS diagnosis document.
Do not reset, stage, commit, push, rewrite history, install dependencies,
alter system configuration, or modify `AMR_CODEX_HANDOFF.md` without explicit
authorization. Explorer navigation simplification remains a separate design
packet after runtime acceptance.

---

## Policy implementation record — 2026-09-25 — Safe Reachable-Area Completion v1

The user authorized a new exploration-completion policy after the finalized
Run04 diagnosis. The policy defines `COMPLETE` as reachable-area exhaustion:
no raw frontier may remain safely reachable under the current validated map,
raw costmap, padded rectangular footprint, and existing heading-aware route
proof. It explicitly does not claim full physical accessibility or human map
quality. Three fresh, content-matched zero-candidate cycles are required;
every raw frontier must be classified `BLOCKED_SAFETY` or `BLOCKED_ROUTE`,
while any unclassified frontier remains fail-closed `INCOMPLETE`.

The policy preserves all planner, inflation, footprint, unknown-space,
smoother, controller, collision-threshold, ownership, freshness, and
fail-closed fault behavior. It was implemented in the existing frontier
selector/Explorer seam and the AWS monitor/runner evidence contract. The
selector keeps its historical list return by default and exposes optional
diagnostics for the terminal policy.

Fresh validation:

- Explorer pytest scope: `195 passed, 2 xfailed, 1 xpassed`.
- AWS monitor/runner/diagnostics pytest scope: `38 passed`; runner-only scope:
  `19 passed`.
- Python compilation of the four changed runtime scripts: exit 0.
- `colcon build --packages-select amr_exploration amr_simulation
  --symlink-install --event-handlers console_direct+`: exit 0.
- `ctest --test-dir build/amr_exploration --output-on-failure`: 3/3 passed.
- `ctest --test-dir build/amr_simulation --output-on-failure`: 11/12 passed;
  the known unrelated AWS world-size assertion remains `7950` actual versus
  `7954` expected.
- Scoped `git diff --check`: exit 0.

No AWS runtime was launched from this packet. No runtime `COMPLETE`, map
quality, or hardware acceptance is claimed. `AMR_CODEX_HANDOFF.md` remains
untouched.

A combined Explorer-plus-AWS pytest invocation was non-diagnostic: it passed
230 tests, then segfaulted in runner subprocess cleanup and left two exact
test-owned descendants. Process audit tied them to the temporary cleanup
tests; they exited before cleanup. Isolated scopes and the CTest runner case
passed, so no production change was made for that test-order/resource symptom.

## Runtime acceptance record — 2026-09-25 — Run05

The user directed continuation through the runtime gate after the policy
implementation was source-validated. A fresh AWS warehouse simulation was run
through the maintained `src/amr_simulation/scripts/aws_exploration_runner.py`
using `ROS_DOMAIN_ID=220`, unique run identity
`aws_warehouse_runner_20260925_05`, and the runner-derived unique Gazebo
partition. The authoritative artifacts are under
`.ros_logs/aws_warehouse_runner_20260925_05/`.

The runner returned `classification=COMPLETE`, `pass=true`, and
`first_failure=null`. The Explorer terminal evidence used
`completion_policy=SAFE_REACHABLE_AREA_V1` and recorded 110 raw frontiers,
110 blocked frontiers (109 `BLOCKED_SAFETY`, 1 `BLOCKED_ROUTE`), and 0
unresolved. It reported
`exploration complete: reachable area exhausted; blocked frontiers remain`,
`mission_outcome=SUCCEEDED`, `mission_stage=TERMINAL`,
`motion_stopped=true`, no active/pending/cancel-owned motion, and
`fault_latched=false`.

The finalized rosbag contains 151737 messages over 163.600086661 seconds in
one SQLite database. `ros2 bag info` confirmed nonzero map, raw costmap, TF,
exploration-status, navigation-action, sensor, odometry, and simulation
diagnostics rows. The map saver returned 0 and verified the run-local
`evidence/aws_map.yaml` and `evidence/aws_map.pgm`. Recorder, monitor, and
diagnostics returned 0; the launch return code 1 occurred during the expected
terminal shutdown sequence after the runner had received valid terminal
evidence. The runner recorded no first failure, no cleanup escalation, and no
surviving owned processes. Final diagnostics showed 24/24 monitored collision
classes covered and 76404 contact classifications, all `NORMAL_SUPPORT`.

Independent Sol/high review after the runtime found no material policy
finding. It checked the selector compatibility seam, existing safety gates,
fail-closed unresolved handling, terminal count invariants, focused tests,
package builds, and the direct Run05 result. This establishes simulation
runtime acceptance for the new completion policy; it does not establish
hardware acceptance or human map-quality completeness. The known unrelated
`amr_simulation` world-size CTest mismatch remains documented (7950 actual vs
7954 expected).

## Whole planned AWS simulation closeout — Packet 6 — 2026-09-25

The user clarified that continuation should cover the whole planned AWS
simulation scope beyond the exploration-policy fix. The remaining Packet 6
planner/footprint replay boundary was completed without changing safety
thresholds. The replay contract was integrated into
`src/amr_navigation/CMakeLists.txt`, so the normal navigation package build
now exercises it.

Fresh replay validation against the preserved Run07 snapshot and installed
Humble lattice primitives passed all six cases. The Smac 2D result is an
intentional negative control: it returns 2 because its generated path fails
the rectangular footprint gate at cost 254. The configured Smac lattice
returns 0, remains collision-free through the external SimpleSmoother, and
has worst footprint cost 253. Blocked start/goal and forbidden-unknown
start/goal cases are rejected as required.

Verification completed:

- standalone replay CMake build: exit 0;
- standalone replay contract CTest: 1/1 passed;
- integrated `amr_navigation` CTest: 2/2 passed;
- combined `colcon build --packages-select amr_navigation amr_exploration
  amr_simulation --symlink-install --event-handlers console_direct+`: exit 0;
- scoped `git diff --check`: exit 0.

The build emitted non-fatal overlay RPATH-cycle warnings for replay binaries;
the binaries and tests executed successfully. Together with Run05, this
completes the planned AWS warehouse simulation implementation and acceptance
scope. It does not establish hardware acceptance, full physical accessibility,
human map-quality completeness, or unrelated product-phase acceptance.

## AWS software-scope final validation — 2026-09-25

The stale AWS world byte-identity assertion was diagnosed before correction.
`src/amr_simulation/worlds/aws_warehouse.sdf` is clean in Git and is the same
7950-byte file in both source and install locations, with SHA-256
`41d9685cf8104a5e76ea832fa5330eadcbc4f7952c3ed2115f38a352d695fbb8`. The
test literals expected 7954 bytes and SHA-256
`1c1eae5f61e9ec486f9435bdb12c2a6210d50f2da9a001829117bae3bd5623f5`, which
did not match the tracked canonical fixture. An independent semantic check
passed the production `validate_world` path, all six plugins, ODE physics and
step size, 25 model/URI entries, reserved-model exclusion, and resource URI
hygiene.

The only source correction was to update those two stale literals in
`src/amr_simulation/test/test_aws_warehouse_exploration.py`. No world file or
production safety behavior was changed. Focused validation passed:

- AWS contract pytest: 8/8;
- simulation CTest: 12/12;
- navigation CTest: 2/2;
- exploration CTest: 3/3;
- scoped `colcon build --packages-select amr_navigation amr_exploration
  amr_simulation --symlink-install --event-handlers console_direct+`: exit 0;
- scoped `git diff --check`: exit 0.

The final serial planner replay against the preserved Run07 snapshot and
installed Humble lattice primitives passed all six cases: `smac_2d` returned
2 as the expected unsafe negative control, `smac_lattice` returned 0 with the
required collision-free smoothed result, and block/unknown start/goal cases
were rejected. A prior validation invocation ran CTest and replay concurrently
in ROS domain 0 and stopped at `smac_lattice`; that result was classified
non-diagnostic process/domain interference, not a source failure. The isolated
serial rerun passed without further edits. `AMR_CODEX_HANDOFF.md` remains
untouched, and no external systems were changed.

## Model-family policy correction — 2026-09-25

The user established the exact model assignments for this workspace: Sol/high
must use `gpt-5.6-sol` with high reasoning effort, and Luna/max must use
`gpt-5.6-luna` with max reasoning effort. GPT-6 model identifiers are
prohibited for diagnosis, planning, implementation, review, delegation, and
fallback. The exact model identifier must be verified before every switch or
delegation; if the assigned GPT-5.6 model is unavailable, work stops and the
blocker is reported.

A mistaken GPT-6 delegated attempt was stopped immediately. It produced no
accepted implementation result and does not authorize any AWS completion
claim. `AMR_CODEX_HANDOFF.md` remains untouched.

## Packet 6 raster-fixture diagnosis and stop checkpoint — 2026-09-25

GPT-5.6 Luna/max attempted the approved Packet 6 replay-coverage packet and
stopped after two failed attempts on the same swept-rotation fixture
hypothesis. Baseline navigation CTest passed 2/2, the preserved Run07 replay
passed all six cases, and the replay harness built. The focused
`planner_replay_contract` failed twice because the supposedly clear endpoint
returned lethal cost 254.

GPT-5.6 Sol/high re-diagnosed the failure with high confidence as a
`TEST/EVIDENCE HARNESS` issue: the continuous fixture geometry did not match
Nav2's 5 cm rasterization. The obstacle at `(0.425, -0.425)` maps onto the
yaw-zero endpoint footprint edge at cell `(48,31)`. An independently checked
replacement cell at `(0.675, 0.175)` is clear at both endpoints and lethal
during the interpolated rotation at approximately `0.735266366` rad.

The existing between-pose interpolation and endpoint-compatibility reporting
are present in `src/amr_navigation/test/replay/planner_replay_backend.cpp`.
No route fixtures or CTest route registration were added. The safe resume point
is one new GPT-5.6 Luna/max correction limited to
`src/amr_navigation/test/replay/planner_replay_contract_test.cpp`, followed by
focused contract/full navigation/Run07 validation and independent GPT-5.6
Sol/high review. Do not add route fixtures, change thresholds, or claim AWS
completion in that packet. The ledger records 7 mistakes; no new
implementation was started after this diagnosis. `AMR_CODEX_HANDOFF.md`
remains untouched.
