# Project Status

## Current Gate 6 higher-mass boundary — `_13` retained placement — 2026-09-01

Product 102 is the current 3 kg boundary. Phase 14 Gate 6 remains
**FAIL / unresolved**; Product 103 and Gate 7 remain blocked. The latest
direct-host Product 102-only run is
`.ros_logs/gate6_product102_retry_20260901_13/`. All pre-placement
host/rendering, readiness, lifecycle, controller, MoveIt/OMPL, attachment
bootstrap, ownership, cleanup, shutdown, and post-shutdown gates passed;
Product 102 consumed one attempt and Product 103 consumed zero.

The first causal failure was retained placement-state validity:
`arm_link_2 <-> product_camera_link` at
`1788214919.396335684`, after exact attached-scene proof passed at
`1788214919.365739585`. The source rejected the first invalid sample at
`1788214919.396375389` and failed at `1788214919.439088368`. The later
`move_group` `-11` occurred during cleanup and is secondary.

The corrected offline MoveIt/FCL probe reproduced the collision using the
current URDF/SRDF and current retained KDL branch. Structured IK seeds and
bounded placement geometry probes did not find a collision-free alternative
for the exact center-slot retained-placement contract. This is a
**product/source geometry/path contract defect**, not infrastructure,
DDS/discovery, lifecycle timing, Gazebo, host, runner, or analyzer behavior.
No source or safety weakening has been applied. The unresolved boundary is
phase authority for a compliant correction preserving the camera collision
model, exact release/slot acceptance, retained-path validity, and fail-closed
behavior. No Luna/max packet is authorized until that choice is explicit.

## Current Gate 6 higher-mass boundary — `_12` pickup frame — 2026-09-01

Product 102 is the current 3 kg boundary. Phase 14 Gate 6 higher-mass
validation remains **FAIL / unresolved**; Product 103 and Gate 7 remain
blocked. The latest direct-host Product 102-only run is
`.ros_logs/gate6_product102_retry_20260901_12/`. All host/rendering,
readiness, lifecycle, controller, MoveIt/OMPL, bootstrap, ownership, cleanup,
shutdown, and post-shutdown gates passed; Product 102 consumed one attempt and
Product 103 consumed zero.

The first `_12` Product 102 failure was the mass-stage bilateral-contact gate
at `1788213217.270275868`. Right contact began at `1788213212.561138900` and
the retained bag contains 1,704 right-contact messages but zero left-contact
messages. The right finger hit the product handle first and shifted it before
the placement path. `_11` had 20 contacts on each side. This is a
product/source pickup-frame geometry defect: `_12`'s fresh base yaw and fixed
zero-lateral pickup scene left the left finger outside the handle. It is not a
DDS, lifecycle, controller, Gazebo, host, runner, or analyzer failure.

Sol/high recorded the RCA and packet at
`.ros_logs/gate6_product102_retry_20260901_12/evidence/post_run_root_cause.txt`
and `next_luna_packet.txt`. Luna/max implemented only the allowed
`gate6_mass_stage.cpp` and focused contract test; the fresh product/robot
poses are mapped into `base_footprint` and their lateral offset is used by the
pickup scene and arm targets. Luna did not plan or replan independently.
Nominal branch, safety/fail-closed gates, ownership, attachment semantics,
tolerances, and time bounds are unchanged.

Sol/high independently passed the focused build and `amr_manipulation` CTest
suite (`6/6`, `100%`), 14 focused Python contracts, and `git diff --check`.
The next authorized action is exactly one fresh clean-host direct-host Product
102-only runtime with refreshed current source/install hashes, stopping at the
first failed gate. Do not start Product 103 or Gate 7.

## Current Gate 6 higher-mass boundary — `_08`, 2026-09-01

Product 102 is the current 3 kg boundary. Phase 14 Gate 6 higher-mass
validation remains **FAIL / unresolved**; Product 103 and Gate 7 remain
blocked. The latest direct-host Product 102-only run is
`.ros_logs/gate6_product102_retry_20260901_08/`. Host/rendering, readiness,
lifecycle, controller, MoveIt/OMPL, bootstrap, ownership, cleanup, shutdown,
and post-shutdown gates all passed. Product preparation and mass-stage startup
passed; Product 102 consumed one attempt and Product 103 consumed zero.

The first `_08` product failure was the unchanged mass-stage proof at
`1788209274.045441999`: `fresh bilateral contact gripper positions were not
proven`, following the successful close action at
`1788209270.985996398`. Bag joint samples show left exactly `0.020 m` and
right fixed at `0.035 m`; left product contact count is zero while right
product-handle contact is present. The source uses a strict `>` comparison at
the exact close target, and the URDF/ros2_control model relies on a passive
mimic that DART reports it cannot constrain. This is classified as a
product/source bilateral-gripper defect, not DDS, lifecycle/startup, harness,
analyzer, host, or readiness failure.

The complete RCA and frozen Luna/max packet are in
`.ros_logs/gate6_product102_retry_20260901_08/evidence/post_run_root_cause.txt`
and `next_luna_packet.txt`. Luna/max may edit only the packet's eight listed
files, must not plan or replan independently, and must run focused checks
only. The packet preserves all bilateral contact, attachment, ownership,
fail-closed, timeout, and tolerance gates. After focused verification, the
next authorized action is one fresh clean-host direct-host Product 102-only
runtime with both gripper action-status topics recorded, stopping at the first
failed gate. Product 103 and Gate 7 remain blocked.

## Superseded Gate 6 higher-mass boundary — `_07`, 2026-09-01

Product 102 is the current 3 kg boundary. Phase 14 Gate 6 higher-mass
validation remains **FAIL / unresolved**; Product 103 and Gate 7 remain
blocked. The latest direct-host run is
`.ros_logs/gate6_product102_retry_20260901_07/`. All host/rendering,
readiness, lifecycle, controller, MoveIt/OMPL, bootstrap, ownership, cleanup,
shutdown, and post-shutdown gates passed. It consumed one Product 102 runner
attempt and zero Product 103 attempts.

The first final-gate failure was the independent physical dock proof at
`1788206985.239138529`, after the two intended final precise actions succeeded
at `1788206985.234373732` and `1788206985.238695165`. Settled ground truth was
`(2.347034, 0.002245, 0.052365)` and correctly failed the unchanged dock
position limit with `0.0537 m / 0.0524 rad`; the mass stage was not started.
Raw/wheel odometry and ground truth agree, and the command is zero after
settling. AMCL/map feedback instead ended at
`(2.390182, -0.001508, 0.063884)`, with AMCL covariance growing during the
final leg. This is not a runner, controller, command, or settling defect.

A factory-only front-lidar discriminator at that settled ground-truth pose
passed and falsified a remaining map/SDF mismatch. In the pickup forward
sector, measured beams matched the canonical map at `0.000939 m` MAE / 100%
within 20 mm; the AMCL terminal pose differed by `0.043768 m` MAE / 0% within
20 mm. The map pickup cells exactly cover the SDF pedestal. The remaining
classification is a **product/source localization-configuration defect**:
factory AMCL uses `alpha1..alpha5: 0.2` despite deterministic Gazebo DiffDrive
and matching raw/wheel odometry, allowing the estimate to run ahead. Evidence
is in `.ros_logs/gate6_localization_scan_probe_20260901_01/` and the `_07`
RCA file.

Sol/high froze one implementation packet at
`.ros_logs/gate6_product102_retry_20260901_07/evidence/next_luna_packet.txt`.
Luna/max must apply only that packet, must not plan or replan independently,
and must stop on any scope or hypothesis mismatch. The packet preserves map,
runner, lifecycle, ownership, fail-closed, tolerance, timeout, and mass-stage
behavior. After focused checks, the next action is exactly one fresh clean-host
direct-host Product 102-only runtime, stopping at the first failed gate.
Product 103 and Gate 7 remain blocked.

## Superseded Gate 6 higher-mass boundary — `_06`, 2026-09-01

The `_06` run stopped before navigation on an intermittent Gazebo ControlWorld
response-send DDS/RMW boundary after the pause side effect. Its evidence and
the passing factory-only probes remain at
`.ros_logs/gate6_product102_retry_20260901_06/` and
`.ros_logs/gate6_world_control_probe_20260901_0[1-6]/`.

## Superseded Gate 6 higher-mass boundary — `_05`, 2026-09-01

Product 102 is the current 3 kg boundary. Phase 14 Gate 6 higher-mass
validation remains **FAIL / unresolved**; Product 103 and Gate 7 remain
blocked. The latest direct-host Product 102-only run is
`.ros_logs/gate6_product102_retry_20260901_05/`. Host/rendering setup,
persistent graph/lifecycle, controller, MoveIt/OMPL, ownership, cleanup,
shutdown, and post-shutdown host gates passed. It consumed one Product 102
preparation attempt and zero Product 103 attempts.

The first causal `_05` failure was the runner's independent product-geometry
proof at `1788202887.720104694`, after final precise navigation succeeded at
`1788202887.718557119`. Ground truth ended at `(2.371799, 0.004260,
0.080093)`, inside the unchanged physical dock gate at `0.028520940 m /
0.080093 rad`. Product 102 stayed at `(3.25, 0, 0)`, while the fixed
base-frame pickup geometry measured `0.078605507 m`, above the unchanged
`0.040 m` product gate. This is a product/source route-controller contract
defect: one final precise goal coupled translation with terminal yaw, allowing
Nav2's broad dock yaw window to succeed without satisfying the fixed
top-grasp frame. A validator-only frame change was rejected because the C++
mass-stage grasp proof would reject the same error.

The revised runner sends a current-to-dock travel-bearing precise goal and
then a same-position registered-yaw precise goal on both final-dock paths.
Luna/max implemented only the runner and focused contract test, without
planning or replanning independently. Dock/product tolerances, fail-closed
proof, abort semantics, C++ mass-stage logic, map/SDF, and controller settings
were preserved. Independent verification passed 11 focused tests, package
build, 37 package tests with zero errors/failures/skips, Python compile, diff
check, and source/install hash equality. Evidence:
`.ros_logs/gate6_product102_retry_20260901_05/evidence/implementation_validation.txt`.

Next authorized action: one fresh clean-host direct-host Product 102-only
runtime with the corrected runner and map, after stale-process and rendering
preflight. Stop at the first failed gate; do not start Product 103 or Gate 7.

## Active phase

Phase 14 factory mobile manipulation remains authorized gate-by-gate. Gates 1
through 5 passed. Phase J runtime-performance acceptance and Phase K
integrated MoveIt/Product 101 validation passed on the direct Ubuntu host. The
independent 1 kg repeatability boundary is now also accepted after corrected
bag analysis. Product 102 is the current 3 kg boundary; do not start Product
103, 5 kg, or Gate 7 before it passes.

## Superseded `_13` Gate 6 boundary — retained below

The latest direct-host Product 102-only run is
`.ros_logs/gate6_product102_retry_20260831_13/`. Host/runtime setup,
persistent readiness, lifecycle, controller, MoveIt/OMPL, ownership, cleanup,
shutdown, and post-shutdown host gates all passed. The run consumed one
Product 102 preparation attempt and zero Product 103 attempts. The `_12`
terminal-abort fix worked: the final recovery precise controller goal reached
`Reached the goal!` at `1788195662.116643123`.

The first causal `_13` failure was the independent physical dock proof at
`1788195662.119917155`, which rejected ground truth at `position=0.0355 m`
and `yaw=0.1336 rad`; the mass stage was never started. This is a
Gazebo/simulation map-consistency issue. The final precise action succeeded on
the AMCL/map pose, while the independent physical pose remained outside the
unchanged `0.030 m / 0.15 rad` gate.

The frozen correlation evidence shows final-leg ground-truth displacement
`0.524498 m`, raw/simulation and wheel odometry displacement `0.524263 m`,
and a final composed mission `map→base` pose `0.027052 m` from ground truth.
The SDF pickup pedestal is `x=[3.10,3.50]`, `y=[-0.25,0.25]`; the canonical
PGM pickup blocks are approximately `x=[3.15,3.60]`, `y=[-0.25,0.30]` for all
three rows. The resulting one-cell map/world discrepancy explains the AMCL
shift as the robot approaches the pedestal. Full packet:
`.ros_logs/gate6_product102_retry_20260831_13/evidence/post_run_root_cause.txt`.

Luna/max is limited to reconciling those three PGM regions with the existing
SDF and adding focused asset-contract coverage. Luna must not plan or replan
independently. Preserve map metadata, station poses, physical tolerances,
AMCL ownership, command ownership, fail-closed behavior, and all unrelated
work. Product 103 and Gate 7 remain blocked until a corrected-map runtime
passes.

The approved implementation is complete and independently verified. The
source and installed PGM both hash to
`c9da32376a478c3f52ca7f1624e06e2d60eb772c872fd7e64bb0b26b8d2a7b01`; 57
payload bytes changed. The focused asset test passed 13/13 and
`colcon test --packages-select amr_factory` passed 63 tests with zero errors,
failures, or skips. Luna did not plan or replan independently, and no runtime
was started by the implementation pass. Evidence:
`.ros_logs/gate6_product102_retry_20260831_13/evidence/implementation_validation.txt`.

The next authorized action is one fresh clean-host direct-host Product 102-only
runtime with the corrected installed map, after a direct-host stale-process
scan. Stop at the first failed gate; Product 103 and Gate 7 remain blocked.

## Superseded `_12` boundary — retained below

The latest direct-host Product 102-only run is
`.ros_logs/gate6_product102_retry_20260831_12/`. Host/runtime setup,
persistent readiness, lifecycle, controller, MoveIt/OMPL, ownership, cleanup,
shutdown, and post-shutdown host gates all passed. The run consumed one
Product 102 preparation attempt and zero Product 103 attempts, then stopped
before the mass stage.

The first causal failure was the final precise recovery re-dock being aborted
by the Nav2 controller progress checker:
`1788194453.873780266` `controller_server: Failed to make progress`, followed
by `1788194453.874380827` `Mission aborted: path following failed`. Bag replay
shows the checker reached its 10.0-second bound with only
`0.160357 m / 0.125214 rad` mission-feedback progress and independently
`0.137067 m / 0.113642 rad` localization-odometry progress, below its
unchanged `0.20 m / 0.20 rad` requirements. The physical ground-truth pose
ended inside the unchanged final dock gate at `0.028579 m / 0.148726 rad`.

This is classified as a product/source bug in the runner's recovery branch:
the initial precise dock has bounded terminal-abort handling, but the final
precise recovery dock does not, so preparation fails before the existing
independent physical/product proof. The AMCL warning was transient and
relocalization succeeded. No DDS, lifecycle, Gazebo, rendering, analyzer, or
host-contamination failure occurred. Full evidence and the frozen Sol/high
packet are in
`.ros_logs/gate6_product102_retry_20260831_12/evidence/post_run_root_cause.txt`.

Luna/max implemented only the product runner and its focused contract test:
the bounded catch now applies to the final recovery precise dock, followed by
the existing stationary/fresh-physical boundary and unchanged independent
final/product proof. Focused build/test validation passed; its exact results
are in
`.ros_logs/gate6_product102_retry_20260831_12/evidence/implementation_validation.txt`.
No timeout, tolerance, progress-checker, route, ownership, or fail-closed
behavior changed. Luna did not plan or replan independently and ran no
integrated runtime. Product 103 and Gate 7 remain blocked.

## Phase J evidence

The preserved run is
`.ros_logs/gate6_1kg_retained_20260830_01/`. Its runtime-performance report
records `/dev/dri/renderD128`, no forced software renderer, 3,600 samples,
aggregate RTF `0.9999999293`, median RTF `1.0000144002`, and `verdict=PASS`.
No Phase J source, geometry, controller, or performance setting was changed.

## Phase K status

- The existing project-owned MoveIt launch uses the authoritative composite
  Xacro and SRDF, the `manipulator` group, OMPL, the `arm_controller` and
  `gripper_controller` adapters, and `/amr/base/joint_states`.
- The exact composite Xacro expanded and `check_urdf` passed with
  `arm_joint_1` through `arm_joint_6`, both gripper joints, `arm_base_link`,
  and `gripper_tcp`.
- A bounded MoveIt smoke started the model, OMPL pipeline, both configured
  controllers, and all required MoveGroup capabilities.
- Integrated readiness passed on the direct host: all required lifecycle nodes
  reported `active [3]`, the joint-state/arm/gripper controllers were active
  exactly once, required actions/services/topics were present, MoveGroup
  actions and the OMPL query service responded, and visible node names had no
  unexpected duplicates.

## Phase K result

The single direct-host Product 101 run used the mandated Phase J environment,
the strict headless factory graph with `factory_attachment:=true`, the
project-owned MoveIt launch, and the prescribed hidden-topic recorder. The
stage passed attachment bootstrap, bilateral gripper evidence, pickup,
attachment safety rejection, dock egress, pickup approach, split dispatch
navigation, dispatch dock, placement alignment/lower, release, and empty
stow. It produced the exact terminal line:

`GATE 6 1.0 KG COMPLETE 1 KG PASS`

The recorder finalized a 111.3 MiB, 200,534-message bag at
`.ros_logs/gate6_1kg_retained_20260830_01/product101_evidence/`. No Phase J
performance evidence was rerun or changed, and no source or configuration
file was modified.

## Validation and stop reason

`src/amr_manipulation/test/test_moveit_config.py`: 4 passed. The original
bounded MoveIt smoke log remains preserved at
`.ros_logs/gate6_1kg_retained_20260830_01/move_group_18_1788097414517.log`;
the Phase K runtime MoveIt log is
`.ros_logs/gate6_1kg_retained_20260830_01/move_group_34342_1788098551620.log`.
The initial direct-host attempt found stale duplicate Gazebo processes; only
the identified PIDs were stopped before the clean graph launch. An early
lifecycle query briefly returned `Node not found` during service discovery,
then the same check and the full lifecycle set returned `active [3]` after the
graph settled. This was an environment/startup timing boundary, not a proven
source defect.

The recorder, MoveIt, and factory processes were stopped and the final exact
process scan passed for that original Phase K run. The subsequent continuation
and current source/test changes are recorded below.

The Gate 6 pass-2 graph-readiness, controller-startup, and analyzer boundaries
were diagnosed and repaired with focused regression coverage. A fresh
readiness-gated Product 101 run then passed the stage and corrected bag
analysis. The worktree contains the preserved runtime evidence plus the
source, test, and tracking-document changes; no dependency or external asset
was installed.

No dependency or external asset was installed. No commit or push was made.
`AMR_CODEX_HANDOFF.md` remains user-owned and untouched.

## Gate 6 pass-2 readiness repair and accepted second 1 kg pass — 2026-08-31

The changing missing-node sets in
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/readiness_graph_check.txt`
were an observer-boundary failure, not evidence that the factory nodes were
restarting. ROS 2 Humble's `ros2 node list --no-daemon` path creates a fresh
direct participant with a 0.5 s default discovery wait. Six independent
short-lived observers therefore returned non-cumulative partial graphs while
the continuously running factory and MoveIt processes remained alive. An
isolated persistent observer converged to all 18 required nodes in 1.105 s and
held the complete graph stable; removing `/move_group` caused the same check to
fail.

The readiness gate now uses one persistent `rclpy` observer with a bounded 30 s
discovery window, requires all 17 factory nodes plus `/move_group`, rejects
duplicate required names, and requires a complete unique graph for 2 s. It
records the ROS domain, discovery environment, actual RMW, transitions, and
final verdict. Direct-host readiness-only runs
`gate6_graph_readiness_barrier_20260831_01` and `_02` both passed with
`rmw_fastrtps_cpp`; each observed an initially partial graph before reaching a
complete stable graph. The implementation and focused contract tests are in
`src/amr_factory/scripts/factory_runtime_preflight.py` and
`src/amr_factory/test/test_factory_runtime_preflight.py`.

The first Product 101 attempt after that fix exposed a separate startup race:
Humble's lifecycle manager uses a zero-delay wall timer for autostart while
`controller_server` is still constructing its nested local costmap. In the
failed run the manager began bringup roughly 45–78 ms after the controller
appeared, and the controller logged a `change_state` response timeout. The
minimum fix starts the controller process first and delays only its lifecycle
manager by one second in
`src/amr_mpc_controller/launch/amr_mpc_controller.launch.py`. Two subsequent
readiness-only runs passed without that timeout, with no controller parameter,
command-ownership, or motion-semantics change.

The fresh full run was
`gate6_1kg_repeat2_graphfix_20260831_03` (`ROS_DOMAIN_ID=228`). Host preflight
passed with `/dev/dri/renderD128`; runtime preflight passed with 3,600 samples,
aggregate RTF `0.9999999013`, and median RTF `1.0000073501`; persistent graph
readiness and all lifecycle/controller/action/service/topic/OMPL/ownership
checks passed before recording. The recorder started before exactly one
Product 101 stage. The stage exited 0 with the exact line
`GATE 6 1.0 KG COMPLETE 1 KG PASS`.

The finalized bag contains 191,936 messages over 90.156199656 s. The original
analyzer invocation returned FAIL because the base adapter's independent 50 ms
timer can forward the prior exact arbitration sample, while the analyzer only
compared against the newest sample at or before each output. Source tracing and
bag replay showed all 1,066 nonzero simulation outputs matched an exact
arbitration sample at or before output within 0.25 s. The analyzer now checks
that bounded historical trace, preserving rejection of unowned and stale
commands. Corrected reanalysis of this actual bag returned
`GATE6_BAG_ANALYSIS=PASS product_id=101`; focused analyzer regressions and the
full affected-package test pass are recorded below.

The second independent 1 kg pass is therefore accepted after the analyzer
tooling correction. This was the one permitted Product 101 retry and consumed
one Product 101 attempt. No further Product 101 retry, 3 kg, 5 kg, Product
102/103, or Gate 7 run is authorized by this handoff. Shutdown completed with
no remaining runtime processes, and the post-shutdown host preflight passed.
Evidence is under
`.ros_logs/gate6_1kg_repeat2_graphfix_20260831_03/`, including
`evidence/graph_readiness.txt`, `evidence/product101_bag_info.txt`,
`evidence/product101_analysis_corrected.txt`, and
`evidence/shutdown_process_scan.txt`.

The focused tests passed, and the combined
`amr_factory amr_mpc_controller amr_manipulation` build/test validation passed:
271 tests, 0 errors, 0 failures, and 5 skipped. `git diff --check` passed.

## Gate 6 post-fix direct-host readiness boundary — 2026-08-30

The source-ordering correction was exercised on a new direct-host run,
`gate6_1kg_repeat2_settlefix_20260830_02`, using strict hardware rendering,
`ROS_DOMAIN_ID=208`, and a fresh run directory. Host preflight passed with
`/dev/dri/renderD128`, no forced software renderer, and no known processes.
Runtime preflight also passed: 3,413 samples, median RTF `0.9989792429`, and
aggregate RTF `0.9480839893`.

The strict true-attachment factory launch and project-owned MoveIt launch were
started. The integrated readiness gate then failed at graph completeness: six
bounded non-daemon ROS graph queries never returned the complete required
17-node factory set plus `/move_group`. The exact missing-node attempts are
preserved in
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/readiness_graph_check.txt`.
The stop rule was applied immediately: no lifecycle/controller/action/service/
topic/ownership acceptance was declared, and no recorder or Product 101 stage
was started.

Shutdown followed the documented available order, MoveIt then factory. MoveIt
emitted the known Humble destructor exit `-11` after SIGINT; factory components
shut down, followed by a launch-side adapter-shutdown exception after Gazebo
exited. The final process scan and post-shutdown preflight found no actual
Gazebo, MoveIt, recorder, or Gate 6 process. Cleanup evidence is under
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/`.

This run produced no Product 101 result, bag, or analyzer result. The source
fix and corrected recorder procedure were not changed during the run. Gate 6
second-pass acceptance remains **FAIL / unresolved**; Phase K remains PASS.

## Gate 6 pass-2 repeatability result — 2026-08-30

The next documented Gate 6 boundary was attempted once on the direct host with
the fresh strict true-attachment run
`gate6_1kg_repeat2_20260830_01` (`ROS_DOMAIN_ID=207`). Host preflight passed
with `/dev/dri/renderD128`; runtime preflight passed with 3,600 samples,
median RTF `0.9999916001`, and aggregate RTF `0.9999999890`. The complete
factory/Nav2 readiness, bootstrap READY/Trigger, controller, ownership, graph,
and MoveGroup/OMPL checks passed.

The recorder reported `Recording...` before the single Product 101 stage. The
stage itself completed all manipulation, navigation, placement, release, and
empty-stow gates and ended with the exact line
`GATE 6 1.0 KG COMPLETE 1 KG PASS`. Its finalized bag contains 199,518
messages over 115.618 seconds. The required independent bag analyzer then
failed, so this run does not establish the second accepted 1 kg pass and Gate
6 repeatability remains open.

The analyzer failure is preserved at
`.ros_logs/gate6_1kg_repeat2_20260830_01/evidence/product101_analyzer_console.txt`
and reports missing recorder coverage/QoS for bootstrap status, rear LiDAR,
arm/gripper action status, `/amr/base/joint_states`, and `/tf_static`. The
live QoS inspection proves that the documented recorder command omitted the
first group and used incompatible/default QoS for the latter topics. The same
bag independently shows a strict forbidden-motion sample: `0.000197628 m`
of ground-truth displacement immediately after the stage published
`MOVING`/`base_motion_allowed=false`; the arbitration command at that sample
was zero, with the preceding nonzero command still in the stop transition.
This is a runtime acceptance/interlock-timing failure in addition to the
recorder-procedure defect. No source or runtime configuration was changed, no
Product 101 retry was made, and Products 102/103 and Gate 7 were not started.

The recorder coverage/QoS procedure is now corrected in
`docs/SIMULATION_COMMANDS.md`. The next action is separate diagnosis of the
status-to-zero timing boundary, followed by a fresh valid Gate 6 pass-2
attempt. Do not start 3 kg, 5 kg, or Gate 7 until two valid independent 1 kg
passes are established.

## Gate 6 pass-2 motion-boundary diagnosis and retry boundary — 2026-08-30

Retained bag replay and source tracing proved that the residual-motion failure
was an ordering defect in `MassStageNode::wait_for_motion_permission`, not a
continued nonzero command issued after the motion-forbidden state. The prior
stage reached its final navigation result at
`1788100711.329639878`; the arbitration command became zero at
`1788100711.379224539`. The stage published `MOVING` with
`base_motion_allowed=false` at `1788100711.375472784`, before the command and
plant had settled. The simulation base command remained nonzero through
`1788100711.365955353` and first recorded zero at `1788100711.416099548`.
Ground truth then moved from `1788100711.378139019` to
`1788100711.380825758` by `0.000197628 m` in `0.002686739 s`; raw and filtered
odometry also showed decelerating motion before reaching zero. This falsifies
both a stale-odometry explanation and an intentionally nonzero forbidden
command.

The minimal source correction is in
`src/amr_manipulation/src/gate6_mass_stage.cpp`: the existing fresh
`BaseStatus::READY` and odometry-twist stationary condition must hold for
500 ms before publishing the motion-forbidden status, then the existing
400 ms/500 ms post-announcement guard is retained. The duplicate early
post-detachment status publication was removed so the same feedback-first
boundary governs that transition. A source-contract regression check was added
to `src/amr_manipulation/test/test_moveit_config.py`. No analyzer threshold,
acceptance criterion, geometry, controller setting, physics, or performance
requirement was changed. The corrected recorder/QoS procedure in
`docs/SIMULATION_COMMANDS.md` was preserved unchanged.

Focused build and test validation passed: the `amr_manipulation` package build,
all six package CTest targets, 28 reported tests, and `git diff --check`.

The single authorized fresh retry was then stopped at the required host
preflight before any runtime launch. Run directory:
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_01/`. The preflight reported
`render_devices=<none>` and `verdict=FAIL` for the missing readable/writable
`/dev/dri/renderD*` device; it reported no known simulation processes. No
Gazebo, MoveIt, recorder, Product 101 stage, bag analyzer, or later payload
stage was started. Gate 6 second-pass acceptance therefore remains
**FAIL / unresolved**, and Phase K remains PASS.

## Authoritative higher-mass execution boundary — 2026-08-31

One explicitly authorized combined 3 kg/5 kg execution used
`.ros_logs/gate6_3kg_5kg_20260831_01` with `ROS_DOMAIN_ID=230`.
Its recorded timing was `t0_monotonic=24880.26`,
`hard_cutoff_monotonic=42520.26`, and `script_start_monotonic=26249.81`;
approximately `22:49.55` had elapsed at script start/leaving and
approximately `4:31:10.45` remained to the cutoff.
Final handoff timing is recorded in
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/final_timing.txt`: approximately
`34:15.36` had elapsed from T0 and `4:19:44.64` remained to the cutoff.

Host preflight failed closed before hashes, factory, MoveIt, or recorder at
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/host_preflight/host_preflight.txt`.
It recorded `render_devices=<none>`, `forced_software=<none>`,
`known_processes=<none>`, and `verdict=FAIL` because no readable/writable
`/dev/dri/renderD*` device was available. Product 102 (3 kg) and Product 103
(5 kg) attempts remain 0/not started; no stage, runner, recorder, bag,
analyzer, or `ros2 bag info` ran.

Cleanup passed the process gate. Post-shutdown preflight found no runtime
processes but still failed the render check at
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/post_shutdown/host_preflight.txt`.
The elevated rerun was rejected, and no retry or workaround is authorized.
Before launch, the authorized deterministic analyzer defect was fixed in
`src/amr_manipulation/scripts/gate6_evidence_analyzer.py` with regression
coverage in `src/amr_manipulation/test/test_gate6_completion_contract.py`:
preparation publishes a first nonzero boot stream, and the analyzer now
fail-closes/selects exactly one later stream carrying `Gate 6 mass stage is
starting`; single-stream Product 101 compatibility is preserved. The live
higher-mass attempt itself changed no source. The exact existing motion source
fix remains `src/amr_manipulation/src/gate6_mass_stage.cpp`. The focused
37-test pytest run, 274 package tests (0 errors, 0 failures, 5 skipped), build,
and `git diff --check` remain green.

Higher-mass acceptance is not claimed. The current worktree remains dirty,
and the protected `AMR_CODEX_HANDOFF.md` Git blob hash remains
`469bd6ac1e0f1d85901c0f112ba8800cbfa67507`. The next blocker is a fresh,
explicitly authorized direct-host run with a readable/writable approved render
node; Product 102/103 and Gate 7 remain unvalidated.

## Mission Supervisor startup readiness closure — 2026-08-31

The direct host now exposes readable/writable `/dev/dri/renderD128` with the
required `video` and `render` groups. The retained `_03` higher-mass harness
selected invalid Fast DDS domain 233 and failed every ROS participant before
readiness; `_04` used valid domain 230 but exposed the pre-fix Mission
Supervisor lifecycle race, with process start at `1788166909.8267248`, a
`change_state` response timeout `0.393066 s` later, and final state
`inactive [2]`. Neither run consumed a Product 102/103 attempt.

Fresh direct-host readiness-only run
`.ros_logs/mission_supervisor_readiness_20260831_02/` passed. It used valid
domain 229, strict true-attachment factory startup, hardware rendering, and no
MoveIt or product stage. The Mission Supervisor started at
`1788171944.7317126`; configure began `1.002432 s` later and activation began
another `0.047938 s` later. A persistent observer recorded
`unconfigured [1] -> inactive [2] -> active [3]` and held active for
`2.179422 s`. No `change_state` response timeout occurred. Cleanup and the
final process scan passed, and Product 102/103 attempts remain zero.

This closes the lifecycle/startup blocker without changing Mission Supervisor
C++, lifecycle criteria, safety behavior, ownership, tolerances, or product
logic. Higher-mass Gate 6 acceptance remains unclaimed; a separately reviewed
and authorized valid-domain Product 102/103 run is the next boundary. Gate 7
must not start.

## Gate 6 Product 102 preparation result — 2026-08-31

The first valid higher-mass product execution reached Product 102 after a
harness-only hidden-topic correction. `_05` proved that `ros2 topic list -t`
cannot satisfy checks for hidden `/_action/status` topics; `_06` added only
`--include-hidden-topics`, used valid domain 226, and passed domain,
direct-host, hardware-rendering, RTF, Mission Supervisor, graph, lifecycle,
controller, action, topic, OMPL, bootstrap, and ownership readiness.

Product 102 preparation then failed before the mass stage. The precise dock
goal began at `1788173309.127366377`; Nav2's controller failed progress at
`1788173332.677563518`, followed by the Mission Supervisor and runner aborts.
The finalized bag proves continuous owned commands and reproduces the
configured progress-checker decision from localization: `0.060525 m` and
`0.021949 rad` over `10.033143 s` did not meet the configured `0.20 m` or
`0.20 rad` thresholds. The placement controller had slowed to `0.01 m/s`,
issued zero yaw command, and still had approximately `-0.256 rad` terminal
yaw against a `0.15 rad` goal tolerance.

This is a product/source configuration defect in the precise-docking
route/controller/progress-checker contract. It is not a lifecycle, DDS,
Gazebo-rendering, command-ownership, or analyzer failure. Product 102 consumed
one attempt; Product 103 consumed none. Cleanup and post-shutdown host checks
passed. Gate 6 higher-mass status is **FAIL / unresolved**; no retry or later
gate is authorized until the source boundary is corrected without weakening
dock tolerances or collision safety.

## Product 102 precise-dock source correction — 2026-08-31

The preparation runner now routes only a fresh, terminal precise-dock failure
inside the existing 0.03 m position window into its already registered,
fail-closed retreat/relocalization/re-dock sequence. It does not promote the
failed action to success. Stationary-state, fresh physical pose, AMCL event,
bounded localization bias, final 0.03 m / 0.15 rad dock acceptance, collision
safety, ownership, and attachment checks remain unchanged. All other action
failures still terminate preparation.

Focused tests passed 8/8; the runner compiled; the `amr_manipulation` package
built and passed 34 tests with zero errors, failures, or skips. No Product 102
retry or other runtime was started. Gate 6 higher-mass status therefore
remains **FAIL / unresolved** until a separately authorized direct-host
Product 102 retry validates the corrected path. Product 103 and Gate 7 remain
blocked.

## Product 102 pre-runtime audit closure — 2026-08-31

The recovery exception is now abort-only: only `STATUS_ABORTED` with fresh
terminal localization can reach the bounded first-dock recovery. Cancellation
and every other non-success result remain fail-closed. A full parallel test
also exposed cross-package DDS contamination in the ROS-live `amr_control`
test, not a control product defect. The test captured the exact `0.25 m/s`
command published concurrently by `amr_base_adapter`; isolated execution
passed both egress legs. The control test now uses dedicated domain 211 without
changing production code or timeouts.

All 18 packages build. The final full parallel result is 276 tests, 0 errors,
0 failures, and 5 skipped. No integrated runtime was started. Higher-mass Gate
6 remains **FAIL / unresolved** pending an explicitly authorized Product 102
direct-host retry; Product 103 and Gate 7 remain blocked.

## Product 102 retry readiness result — 2026-08-31

The authorized Product 102-only direct-host run `_07` passed valid empty
domain 225, hardware rendering, host/runtime RTF, Mission Supervisor, MoveIt,
and persistent complete/stable graph gates. It stopped before recorder or
runner startup when the one-shot lifecycle query for the already-managed-active
base adapter timed out. Base adapter logged that its `get_state` response send
timed out and the client would not receive it; the CLI then exited 124 at its
15-second bound. This is a DDS/RMW service-response readiness issue, not a
Product 102 or lifecycle-state defect. Product 102/103 attempts remain zero.

Cleanup and post-shutdown host/process gates passed. Gate 6 higher-mass
acceptance remains **FAIL / unresolved**. The allowed 2/2 replans are
exhausted; no further retry, source change, Product 103 run, or Gate 7 work is
authorized.

## Persistent lifecycle readiness correction — 2026-08-31

The user reset the replan counter to 0/2 and authorized correction of the
`_07` readiness harness defect. The factory preflight now provides a
`lifecycle` mode using one persistent rclpy participant and persistent clients
for all 17 required lifecycle nodes. It observes until every node reports exact
state ID `3` and label `active` for two continuous seconds, or one global
30-second deadline expires. Individual responses remain bounded at one second;
timed-out requests are removed before the next observation. Missing services,
response loss, query errors, and non-active states remain fail-closed and are
written to lifecycle evidence.

This is a harness/readiness synchronization fix. It does not alter production
lifecycle timing, DDS configuration, Product 102 behavior, motion ownership,
safety, attachment semantics, tolerances, or evidence acceptance. The
documented Gate 6 acceptance sequence now uses this persistent lifecycle
preflight instead of fresh `ros2 lifecycle get` participants.

Validation passed: 17 focused tests; Python compilation; the `amr_factory`
build and all 57 package tests; all 18 workspace packages; and 281 full
workspace tests with 0 errors, 0 failures, and 5 skipped. `git diff --check`
also passed. Replans used after reset: 0/2. No integrated runtime was started,
so higher-mass Gate 6 remains **FAIL / unresolved** and Product 102/103 attempt
counters remain zero for `_07`. The next boundary is a separately authorized
clean-host Product 102-only direct-host run using the persistent lifecycle
preflight and stopping at its first failed gate. Product 103 and Gate 7 remain
blocked.
