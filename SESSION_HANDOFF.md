# AMR Session Handoff

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
