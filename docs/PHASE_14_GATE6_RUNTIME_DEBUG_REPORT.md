# Phase 14 Gate 6 runtime debug report

This short record maps the evidence-backed D205 correction chain and the
current Product 102 boundary. It does not claim higher-mass, hardware, or
Gate 7 acceptance.

## Current Product 102 boundary — `_13` retained placement — 2026-09-01

The latest direct-host Product 102-only runtime is
`.ros_logs/gate6_product102_retry_20260901_13/`; Product 102 is the current
3 kg boundary and Product 103/Gate 7 remain blocked. All readiness and
pre-placement runtime gates passed. Product 102 consumed one attempt and
Product 103 consumed zero.

The first causal failure was the retained payload-aware state-validity check.
Exact attached-scene proof passed at `1788214919.365739585`, then MoveIt
reported `arm_link_2 <-> product_camera_link` at retained placement segment
30, sample 1 at `1788214919.396335684`. The source recorded the first invalid
sample at `1788214919.396375389` and failed at `1788214919.439088368`.
The `move_group` `-11` during later shutdown is a secondary cleanup symptom.

A corrected offline MoveIt/FCL probe using the current generated URDF/SRDF
reproduced the same collision on the current KDL branch. Structured IK seeds
and bounded radius/lateral/yaw/orientation/XY/full-path probes did not find a
collision-free retained path for the exact center-slot contract. The camera
collision body and the absence of an SRDF-disabled camera pair are intentional
model behavior. The classification is therefore a **product/source
geometry/path contract defect**, not a startup race, DDS, Gazebo, host,
runner, or evidence/analyzer issue.

No camera collision removal, allowed-collision entry, release/slot change, or
retained-path weakening is authorized from this evidence alone. The exact
unresolved boundary is phase authority for a compliant correction that keeps
the project collision model and fail-closed placement acceptance intact. No
Luna/max implementation packet has been dispatched; if authority is granted,
Sol/high must freeze the packet first and Luna/max must implement only it,
without planning or replanning independently.

## Current Product 102 boundary — `_12` pickup-frame geometry — 2026-09-01

The latest direct-host Product 102-only runtime is
`.ros_logs/gate6_product102_retry_20260901_12/`; Product 102 is the current
3 kg boundary and Product 103/Gate 7 remain blocked. Host/rendering,
readiness, lifecycle, controller, MoveIt/OMPL, bootstrap, ownership, cleanup,
shutdown, and post-shutdown gates passed.

The first Product 102 failure was the mass-stage bilateral-contact gate at
`1788213217.270275868`. The right contact stream first became non-empty at
`1788213212.561138900` and contained 1,704 messages; the left stream contained
zero messages. The same bag shows the right finger contacting the product
handle first and the product moving before the failure. The accepted `_11`
run recorded 20 contacts on each side. The causal defect was the fixed
zero-lateral pickup scene/target combined with the fresh `_12` base yaw, not
the bilateral controller or readiness graph.

Sol/high froze the RCA and implementation packet at
`.ros_logs/gate6_product102_retry_20260901_12/evidence/post_run_root_cause.txt`
and `next_luna_packet.txt`. Luna/max changed only the mass-stage source and
focused source contract: fresh finite product/robot poses are mapped to the
base frame, and the measured product lateral coordinate is used for the
pickup scene, pre-grasp, and grasp. Nominal arm branch, dimensions,
bilateral-contact and native-attachment proof, fail-closed behavior,
tolerances, time bounds, ownership, and placement gates remain unchanged.
Luna did not plan or replan independently.

Independent verification passed: 14 focused Python tests, the
`amr_manipulation` build, all 6 package CTest targets, and `git diff --check`.
The next action is one fresh clean-host direct-host Product 102-only runtime
with the current source/install hashes and a new run identity, stopping at the
first failed gate. Do not start Product 103 or Gate 7.

## Superseded Product 102 boundary — `_08` bilateral gripper — 2026-09-01

The latest authorized direct-host Product 102-only runtime is
`.ros_logs/gate6_product102_retry_20260901_08/`. Product 102 is the current
3 kg boundary; Product 103 and Gate 7 remain blocked. Host/rendering,
readiness, lifecycle, controller, MoveIt/OMPL, bootstrap, ownership, cleanup,
shutdown, and post-shutdown gates passed. Product preparation reached the
existing mass stage at `1788209258.254874156` and Product 103 was not started.

The first product failure was the mass-stage bilateral position proof at
`1788209274.045441999`, after the close action succeeded at
`1788209270.985996398` with measured left position `0.0200 m`. The bag shows
left exactly `0.020 m`, right remaining `0.035 m`, zero left product contacts,
and right product-handle contacts. The source predicate is strict `>` at the
exact close target, while the URDF/ros2_control model relies on a passive
mimic. DART logged that its selected physics engine cannot create the mimic
constraint at `1788209134.597...`. This classifies the boundary as a
product/source bilateral-gripper defect, not DDS, lifecycle/startup, harness,
analyzer, host, or readiness timing.

Sol/high's complete RCA and frozen implementation packet are retained at
`.ros_logs/gate6_product102_retry_20260901_08/evidence/post_run_root_cause.txt`
and `next_luna_packet.txt`. Luna/max is implementation/focused verification
only and must not plan or replan independently. The packet preserves the
existing bilateral contact, attachment, ownership, fail-closed, tolerance,
and timeout gates. After focused verification, perform one new clean-host
direct-host Product 102-only runtime with both gripper action-status topics
recorded, stopping at the first failed gate.

## Superseded Product 102 boundary — `_07` AMCL localization — 2026-09-01

The latest authorized direct-host Product 102-only runtime is
`.ros_logs/gate6_product102_retry_20260901_07/`. All host/rendering,
readiness, lifecycle, controller, MoveIt/OMPL, bootstrap, ownership, cleanup,
shutdown, and post-shutdown gates passed. It consumed one Product 102 runner
attempt and zero Product 103 attempts.

The handled initial precise-dock progress abort occurred at
`1788206946.378412202`; recovery completed. The first final-gate failure was
the unchanged independent physical dock proof at `1788206985.239138529`,
immediately after the two intended final precise actions succeeded at
`1788206985.234373732` and `1788206985.238695165`. Settled ground truth was
`(2.347034, 0.002245, 0.052365)`, yielding the rejected unchanged physical
position error `0.0537 m` and yaw error `0.0524 rad`. The mass stage was not
started.

The bag's raw/simulation and wheel odometry agree with ground truth, and the
command streams are zero after the final action; ground truth moves only about
0.6 mm during the settling tail. AMCL/map feedback instead ends at
`(2.390182, -0.001508, 0.063884)`. Its last update at
`1788206982.529401` has x covariance `0.008734`, while the AMCL-owned
`map->odom` correction remains the ahead-of-truth transform. This excludes
missing settling, plant motion, command ownership, controller completion, and
stale runner feedback.

The targeted factory-only front-lidar probe
`.ros_logs/gate6_localization_scan_probe_20260901_01/` passed host/readiness,
recording, static observation, and cleanup. At the observed ground-truth pose,
60 beams in the pickup forward sector `[-0.10,0.30] rad` matched the canonical
map raster ray-cast with `0.000939 m` MAE and `100%` within 20 mm. The same
measured scan compared with the AMCL terminal pose had `0.043768 m` MAE and
`0%` within 20 mm; beam 360 was `0.43843 m`, versus map predictions `0.44000 m`
at ground truth and `0.39500 m` at AMCL. The corrected map cells exactly cover
the SDF pickup pedestal. This falsifies another map/SDF correction as the
next action.

Classification: **product/source localization configuration in the noiseless
simulation**. Factory AMCL currently sets `alpha1..alpha5` to `0.2`, while the
Gazebo DiffDrive and raw/wheel streams are deterministic. The observed
covariance growth and ahead-of-truth estimate during the final ~0.9 m leg are
consistent with unmodeled motion noise; the scan itself favors ground truth.
This is not a DDS, lifecycle, world-control, map, runner, controller, or
analyzer defect. The exact source-backed implementation packet is frozen at
`.ros_logs/gate6_product102_retry_20260901_07/evidence/next_luna_packet.txt`.

Luna/max is implementation/verification only and must not plan or replan
independently. The packet permits only zero simulated motion-noise alphas and
a focused AMCL configuration contract test; all tolerances, timeout bounds,
ownership, fail-closed behavior, map assets, runner logic, and mass handling
remain unchanged. After focused verification, run exactly one fresh clean-host
direct-host Product 102-only runtime and stop at its first failed gate.

## Superseded Product 102 boundary — `_06` world-control response — 2026-09-01

The `_06` run stopped before navigation on an intermittent Gazebo ControlWorld
response-send DDS/RMW boundary after the pause side effect. Its evidence and
the passing factory-only probes remain retained under the `_06` run and
`.ros_logs/gate6_world_control_probe_20260901_0[1-6]/evidence/`.

## Superseded `_05` final-dock geometry — 2026-09-01

The latest direct-host Product 102-only runtime is
`.ros_logs/gate6_product102_retry_20260901_05/`. Readiness, lifecycle,
controller, MoveIt/OMPL, ownership, cleanup, shutdown, and post-shutdown host
gates passed. It consumed one Product 102 preparation attempt and zero
Product 103 attempts.

The first causal failure was the unchanged independent product-geometry proof
at `1788202887.720104694`, immediately after final precise navigation
succeeded at `1788202887.718557119`. Ground truth ended at
`(2.371799, 0.004260, 0.080093)`, so the independent physical dock gate
passed at `0.028520940 m / 0.080093 rad` against unchanged `0.030 m / 0.15
rad` limits. Product 102 remained stationary at `(3.25, 0, 0)`, but the fixed
base-frame pickup geometry measured `0.078605507 m`, over the unchanged
`0.040 m` product gate. The absent mass-stage log is downstream.

The bag timeline identifies a final-dock route/controller contract defect: a
single precise goal coupled translation with terminal yaw. Nav2 accepted its
unchanged yaw window while the physical final yaw was broad-dock-valid but
invalid for the fixed top-grasp frame. The readiness and DDS/lifecycle,
Gazebo, host, ownership, and analyzer evidence excludes infrastructure as the
first cause. A validator-only coordinate-frame correction was falsified by
the C++ mass-stage `base_footprint` grasp target `(0.85, 0)`; it would reject
the same attachment error later and weaken the preflight proof.

Sol/high froze the revised packet at
`.ros_logs/gate6_product102_retry_20260901_05/evidence/next_luna_packet.txt`.
Luna/max implemented only the product runner and focused contract test,
without planning or replanning independently. Both final-dock paths now use
a finite current-to-dock travel-bearing precise goal followed by the same-XY
registered-yaw precise goal. Dock/product tolerances, fail-closed proof,
abort semantics, C++ mass-stage logic, map/SDF, and controller configuration
were preserved. Independent validation passed 11 focused tests, Python
compile, the package build, 37 package tests with zero errors/failures/skips,
diff check, and source/install hash equality. Full record:
`.ros_logs/gate6_product102_retry_20260901_05/evidence/implementation_validation.txt`.

The next authorized boundary is one fresh clean-host direct-host Product
102-only runtime with a new run identity and ROS domain, after stale-process
and rendering preflight. Stop at the first failed gate. Product 103 and Gate
7 remain blocked.

| Problem | Smallest solution | Evidence |
| --- | --- | --- |
| Direct CAD mount made pregrasp `z=1.10` infeasible | Use the direct-mount FK/IK endpoint at `z=1.00` | D178 direct-mount KDL/FK; D205 pre-place IK/OMPL/lower passed |
| Mirrored wheel origins inverted drive signs | Parameterize wheel `axis_z`; instantiate left `+1`, right `-1` so both base axes are `+Y` | D183/D185 sign and transformed-axis traces; final URDF contracts passed |
| Vendored POSITION interface synthesized velocity and left cross-axis error | Apply Harmonic `JointPositionReset` in the POSITION branch; preserve VELOCITY/EFFORT paths | D175 joint5 final error trace; vendored build/test passed |
| Rear lifecycle adapter stayed inactive after a configure/activate race | Register activation before configure and serialize lifecycle startup | D184 raw rear scan healthy but adapter inactive; D205 adapted rear sample passed |
| Configure burst and readiness observer caused startup false negatives | Configure adapters in an active-state chain and use one bounded rclpy observer for lifecycle, controllers, and fresh TF | D198-D200 discovery/participant failures; D205 strict readiness passed in 16 s |
| RPP pickup final-heading chatter | Split pickup travel-bearing navigation from same-position final-heading navigation | D201 repeated westbound heading chatter; D205 pickup split passed |
| Localization bias moved the dock target | Measure fresh GT/localized planar bias and correct one normal dock target | D190-D192 bias traces; D205 corrected dock and downstream gates passed |
| Placement stance radius left too little reach margin | Derive the nominal stance radius as `0.785 - 0.07 - 0.005 = 0.710 m` and scale the existing direction | D192 release `0.796532 m`; D205 placement error `0.000894 m` |
| Radial held-product yaw collided with the base | Keep radial bearing for reach/IK but map-align product yaw from fresh robot yaw | D203 OMPL base/product collision; D205 map-aligned collision-free IK/OMPL/lower passed |
| Post-detach retreat began contact-invalid and duplicated the world object | Keep the returned `held_product`, remove duplicate `placed_product`, temporarily allow only held-product/finger pairs during the straight retreat, restore ACM, then require state validity | D204 0%/1-point retreat; D205 37 points/100%, ACM restored, validity and empty stow passed |
| Analyzer treated retired precise-action silence as a bag failure | Treat `/amr/mission/navigate_to_pose_precise/_action/status` zero messages as expected and analyze normal navigation status | D205 bag: 239.8 MiB/412,523 messages; stale analyzer alone reported `BAG_ANALYSIS=FAIL` |

## D205 boundary

Verified artifacts are under
`.ros_logs/gate6_d205_product101_loop28_20260824_01` with
`ROS_DOMAIN_ID=205` and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Factory readiness,
MoveIt readiness, ownership, recorder, performance (median RTF `0.999818`,
aggregate `0.994510`), and the exact terminal `GATE 6 1.0 KG COMPLETE 1 KG
PASS` passed. Only this one 1 kg run is accepted; Gate 7 and higher-mass or
repeatability claims remain pending.

## Phase J performance PASS and Phase K MoveIt boundary — 2026-08-30

The retained Phase J run is
`.ros_logs/gate6_1kg_retained_20260830_01/` with
`AMR_RUN_ID=gate6_1kg_retained_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_retained_20260830_01`, and `ROS_DOMAIN_ID=206`.
Its preserved runtime report records `/dev/dri/renderD128`,
`forced_software=<none>`, 3,600 samples, aggregate RTF `0.999999929313705`,
median RTF `1.000014400208803`, and `verdict=PASS`.

The current branch has no source diff; the only pre-documentation worktree
entry was the preserved `.ros_logs/` directory. The authoritative composite
Xacro expanded and passed `check_urdf`; the MoveIt contract test passed 4 tests;
and the project-owned MoveIt smoke loaded the composite model, OMPL, the
`FollowJointTrajectory` arm controller, the `GripperCommand` controller, and
the required MoveGroup capabilities. Its log is
`.ros_logs/gate6_1kg_retained_20260830_01/move_group_18_1788097414517.log`.

The first integrated Phase K readiness probe,
`ros2 lifecycle get /amr/command_arbitration_node`, returned `Node not found`
because the restricted execution environment did not contain the Phase J
factory/Nav2 graph. The MoveIt smoke was stopped cleanly; no recorder or
Product 101 stage was started and no runtime processes remain. Phase K is
therefore source-validated and partially runtime-validated, but not complete.
The direct-host continuation must keep the Phase J factory session, start
MoveIt with the same run environment, complete the lifecycle/controller/action
checks, then record and run exactly one Product 101 stage. Stop at the first
failed check.

## Phase K integrated direct-host PASS — 2026-08-30

Phase K was completed on the direct Ubuntu host with the preserved
`AMR_RUN_ID=gate6_1kg_retained_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_retained_20260830_01`, `ROS_DOMAIN_ID=206`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `ROS_LOCALHOST_ONLY=1`, and workspace-local
`ROS_LOG_DIR`. Phase J was not rerun or modified. The strict headless factory
launch used the project-owned graph with `factory_attachment:=true`; MoveIt
was started with `ros2 launch amr_manipulation move_group.launch.py`.

The initial host preflight found two stale same-partition Gazebo processes and
the graph showed duplicate controller-manager/Gazebo-control nodes. The exact
identified processes were stopped, the host preflight then passed with
`/dev/dri/renderD128` and no forced software renderer, and a clean factory
graph was launched. A first lifecycle query briefly returned `Node not found`
while DDS service discovery was settling; the command-arbitration service
became responsive on the bounded retry, and the full readiness pass returned:

- all 17 required lifecycle nodes `active [3]`;
- exactly one active `joint_state_broadcaster`, `arm_controller`, and
  `gripper_controller`;
- no unexpected duplicate visible node names;
- one server for each required mission, planning, smoothing, follow-path,
  dock-egress, arm, and gripper action;
- required attachment-bootstrap, controller-manager, and MoveIt services;
- required Product 101 topics, with `/amr/control/cmd_vel` owned by the sole
  `/amr/command_arbitration_node` publisher; and
- operational MoveGroup actions plus an OMPL response from
  `/query_planner_interface`.

The prescribed hidden-topic recorder reported `Recording...` before the stage.
The single Product 101 run passed bootstrap detachment, gripper/contact proof,
pickup, attachment safety rejection, dock egress, pickup approach, split
dispatch translation and heading, dispatch dock, four-segment placement
alignment, collision-checked pre-place/lower, release, and empty stow. The
stage log ended with the exact acceptance line:

`GATE 6 1.0 KG COMPLETE 1 KG PASS`

The finalized bag is
`.ros_logs/gate6_1kg_retained_20260830_01/product101_evidence/`, with
200,534 messages over 96.603 seconds. The stage and MoveIt logs are
`gate6_mass_stage_53626_1788099193302.log` and
`move_group_34342_1788098551620.log` in the same run directory. The final
exact-process shutdown scan passed with no Gazebo, MoveIt, rosbag, or Gate 6
processes remaining.

The initial stale-process and early lifecycle-discovery boundaries are
classified as direct-host setup/timing issues, not software defects; no source,
configuration, geometry, tolerance, controller, or performance file was
changed. The known non-blocking `imu_link` collision warning, missing MoveIt
3D octomap sensor warning, and MoveIt Humble shutdown destructor segfault
remain documented risks; the segfault occurred after the successful stage.
Products 102/103 and Gate 7 remain unvalidated.

## Gate 6 1 kg pass-2 repeatability attempt — 2026-08-30

The next ordered acceptance boundary after the preserved Phase K Product 101
pass was one fresh independent 1 kg pass. The direct-host run used
`AMR_RUN_ID=gate6_1kg_repeat2_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_repeat2_20260830_01`, `ROS_DOMAIN_ID=207`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `ROS_LOCALHOST_ONLY=1`, and
`ROS_LOG_DIR=/home/pete/amr_ws/.ros_logs/gate6_1kg_repeat2_20260830_01`.
The strict true-attachment factory graph and the project-owned MoveIt launch
were used; Phase J and Phase K were not rerun or modified.

Host preflight passed with `/dev/dri/renderD128`, no forced software renderer,
and no stale runtime processes. Runtime preflight passed with 3,600 samples
over `11.996665599 s`, median RTF `0.999991600069809`, aggregate RTF
`0.999999988996943`, and no forced software renderer. Integrated readiness
passed: the bootstrap status was `READY`, its Trigger returned success with
all products detached, all 17 required lifecycle nodes were active, each of
the three required controllers was active exactly once, required actions,
services, and topics were present, command ownership was correct, and
MoveGroup/OMPL responded.

The exact hidden-topic recorder reported `Recording...` before exactly one
Product 101 stage. The stage passed bootstrap, bilateral contact, pickup,
attachment safety rejection, dock egress, pickup approach, split dispatch
navigation, dispatch dock, four-segment placement alignment, payload-aware
pre-place/lower, release, and empty stow. It exited cleanly with the exact
line:

`GATE 6 1.0 KG COMPLETE 1 KG PASS`

The finalized bag is
`.ros_logs/gate6_1kg_repeat2_20260830_01/product101_evidence/`; `ros2 bag
info` reports 199,518 messages, 115.618215760 s, and 90.3 MiB. The stage log
is `gate6_mass_stage_78032_1788100641871.log` and the MoveIt log is
`move_group_66173_1788100231426.log` in the same run directory.

The required analyzer was then run exactly as documented and returned:

`GATE6_BAG_ANALYSIS=FAIL product_id=101`

It reported missing recorded topics:
`/amr/base/joint_states`, `/amr/sensors/rear_lidar/scan`,
`/amr/simulation/attachment_bootstrap/status`,
`/amr/simulation/sensors/rear_lidar/scan`,
`/arm_controller/follow_joint_trajectory/_action/status`,
`/gripper_controller/gripper_cmd/_action/status`, and `/tf_static`.
The exact recorder command omitted the rear-LiDAR, bootstrap, and arm/gripper
status topics. The bag recorded zero `/amr/base/joint_states` messages because
the live publisher is `BEST_EFFORT` while the recorder requested reliable QoS,
and zero `/tf_static` messages because the publisher is `TRANSIENT_LOCAL` and
the default recorder subscription did not obtain its latched sample. The live
QoS evidence is in
`.ros_logs/gate6_1kg_repeat2_20260830_01/evidence/analyzer_required_topic_qos.txt`;
the analyzer result is in
`.ros_logs/gate6_1kg_repeat2_20260830_01/evidence/product101_analyzer_console.txt`.

The analyzer also reported `base moved while motion was forbidden`. Independent
bag timestamp analysis reproduced the first violation at status time
`1788100711.375472784`, when the stage entered `MOVING` with
`base_motion_allowed=false`. The subsequent ground-truth samples moved by
`0.000197628 m` over `0.002686739 s`; the recorded arbitration command at
`1788100711.379224539` was zero, while the preceding sample at
`1788100711.329441786` was `linear=0.059288730` and
`angular=0.054635909`. This establishes a strict sampled residual-motion
boundary during the status-to-zero handoff, not a nonzero command intentionally
issued while forbidden. The corrected diagnosis is preserved at
`.ros_logs/gate6_1kg_repeat2_20260830_01/evidence/analyzer_base_motion_diagnosis_corrected.txt`.

This pass-2 attempt is **FAIL** for Gate 6 repeatability: the stage terminal
line alone cannot override the analyzer failure. The recorder coverage/QoS
problem was a proven procedure/documentation defect, and the corrected
analyzer-complete recorder recipe is now in `docs/SIMULATION_COMMANDS.md`.
The residual pose motion is a runtime acceptance/interlock-timing failure
requiring separate bounded review. No source, controller, geometry, tolerance,
performance, or launch configuration was changed. No Product 101 retry was
made, and Products 102/103 and Gate 7 remain unvalidated. The final process
scan passed with no Gazebo, MoveIt, rosbag, or Gate 6 processes remaining.

## Pass-2 status-to-zero diagnosis and bounded retry — 2026-08-30

### Proven ordering

Replay of the retained bag and direct source inspection established this
timeline around the first analyzer violation:

| Event | Bag timestamp / evidence |
| --- | --- |
| Final navigation result reached | `1788100711.329048949` controller log; stage terminal `1788100711.329639878` |
| Last nonzero arbitration sample | `1788100711.329441786`, linear `0.059288730`, angular `0.054635909` |
| Stage status became `MOVING`, motion forbidden | `1788100711.375472784`, sequence 1380 |
| First post-status ground-truth sample pair | `1788100711.378139019` → `1788100711.380825758`, displacement `0.000197628 m` over `0.002686739 s` |
| Arbitration command recorded zero | `1788100711.379224539` |
| Simulation base command first recorded zero | `1788100711.416099548` |
| Raw odometry still decelerating | `1788100711.398223877`: linear `0.059957414`, angular `0.054789120`; near zero by `1788100711.584464788` |

The stage implementation previously called `set_status(MOVING, false, ...)`
before waiting for stationary feedback. The 50 ms status publisher exposed
that state while the arbitration tick and 50 ms base-adapter forwarding path
were still draining their prior command and the simulated base was settling.
The arbitration zero sample and the decaying raw/filtered odometry falsify a
continuing forbidden command and stale odometry as the primary cause. The
analyzer’s bag-time ordering therefore correctly rejected the boundary.

### Minimal correction

`src/amr_manipulation/src/gate6_mass_stage.cpp` now waits for fresh valid
`BaseStatus::READY` plus fresh odometry with linear x/y and angular z within
the existing `0.01` limits for the existing 500 ms evidence window before it
publishes `MOVING`/`base_motion_allowed=false`. It then keeps the existing
400 ms announcement guard and a second 500 ms feedback-qualified stationary
window. The duplicate explicit post-detachment `MOVING` publication was
removed so detachment uses the same ordering. This is feedback-based, bounded,
and fail-closed: if the base never settles, the stage faults rather than
issuing arm/gripper work. The source-order contract is covered by
`src/amr_manipulation/test/test_moveit_config.py`.

The focused package build, six package CTest targets, 28 reported tests, and
`git diff --check` passed. The corrected recorder/QoS procedure was not changed
again.

### Fresh retry boundary

The one authorized retry was started with a new run identity but stopped at
the documented host preflight. Its evidence is under
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_01/`; the report records
`render_devices=<none>`, no forced software setting, no known simulation
processes, and `verdict=FAIL` because no readable/writable
`/dev/dri/renderD*` device was available. No factory, MoveIt, recorder, stage,
analyzer, or bag-info command ran. The Gate 6 second independent 1 kg boundary
is consequently still **FAIL / unresolved**; Phase K remains PASS and no
3 kg, 5 kg, Product 102/103, or Gate 7 work was started.

## Post-fix direct-host readiness attempt — 2026-08-30

The source correction was tested under a new strict true-attachment direct-host
run, `gate6_1kg_repeat2_settlefix_20260830_02`, with
`ROS_DOMAIN_ID=208`. Hardware preflight passed with `/dev/dri/renderD128`, no
forced software renderer, and no known processes. Runtime RTF preflight passed
with 3,413 samples, median `0.9989792429`, and aggregate
`0.9480839893`.

The factory and MoveIt processes started, but the integrated graph gate failed:
six bounded non-daemon `ros2 node list` attempts never returned all 17 required
factory lifecycle nodes plus `/move_group`. The exact attempts and final
observed subset are preserved in
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/readiness_graph_check.txt`.
Per the stop rule, no lifecycle/controller/action/service/topic/ownership
acceptance was declared, and the corrected recorder, Product 101 stage, bag
analyzer, and `ros2 bag info` were not run.

MoveIt was stopped first and emitted the known Humble destructor exit `-11`
after SIGINT. The factory was then stopped; its components shut down, with a
launch-side `Cannot shutdown a ROS adapter that is not running` exception after
Gazebo exited. The final documented process scan and a post-shutdown host
preflight found no actual runtime processes. This is a readiness-environment
boundary, not evidence to change the source fix, graph ownership, or any
tuning. Gate 6 second-pass acceptance remains **FAIL / unresolved** and Phase
K remains PASS.

## Graph readiness repair, controller startup repair, and accepted pass-2 — 2026-08-31

### Confirmed graph observer root cause

The six changing missing-node sets from
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/readiness_graph_check.txt`
were not evidence of six different factory failures. Direct inspection of the
installed ROS 2 Humble implementation showed that the non-daemon `ros2 node
list` path creates a fresh direct rclpy participant and uses a 0.5 s default
discovery timeout. Each short-lived query consequently returned a partial,
non-cumulative graph snapshot. Process logs and the final process scan showed
the factory and MoveIt processes remained alive until deliberate shutdown.

An isolated experiment with 18 continuously running listener nodes reproduced
the behavior: fresh 0.5 s and 2.0 s observers returned incomplete changing
snapshots, while one persistent rclpy observer reached all 18 nodes in 1.105 s
and held them complete for 2 s. Killing the exact MoveIt node caused the
persistent observer to fail. This distinguishes observer discovery delay from
an actually incomplete runtime graph.

### Readiness implementation

`src/amr_factory/scripts/factory_runtime_preflight.py` now uses one persistent
`rclpy` observer. It records `ROS_DOMAIN_ID`, localhost/transport settings, the
actual RMW implementation, graph transitions, and the final verdict. The gate
requires all 17 factory nodes plus `/move_group`, rejects duplicate required
names, waits up to 30 s for discovery, and requires a complete unique graph for
2 s. Missing or unstable nodes fail closed. Focused tests cover convergence,
stability reset, permanent absence, duplicate names, and CLI routing in
`src/amr_factory/test/test_factory_runtime_preflight.py`.

Readiness-only direct-host runs
`.ros_logs/gate6_graph_readiness_barrier_20260831_01/` and
`.ros_logs/gate6_graph_readiness_barrier_20260831_02/` both passed. Each
recorded an initially partial graph followed by a complete stable graph, with
`rmw_actual=rmw_fastrtps_cpp`; full lifecycle/action/service/topic/OMPL and
ownership checks also passed.

### Controller startup race and minimum fix

The first Product 101 harness after the graph repair stopped before recording
because `/amr/controller_server` was inactive. The failed controller log shows
the lifecycle manager began bringup roughly 45–78 ms after the controller
process appeared, while the controller was still constructing its nested local
costmap; it then logged a `change_state` response timeout. Installed Humble
`nav2_lifecycle_manager` source confirms its autostart path uses a zero-delay
wall timer.

`src/amr_mpc_controller/launch/amr_mpc_controller.launch.py` now starts
`controller_server` first and delays only
`lifecycle_manager_controller` by one second. The change is bounded to startup
ordering; controller parameters, remappings, lifecycle semantics, command
ownership, and motion behavior are unchanged. Two new direct-host
readiness-only runs passed after this fix, including active controller state
and clean shutdown.

### Accepted fresh independent 1 kg run

The only permitted Product 101 retry used
`.ros_logs/gate6_1kg_repeat2_graphfix_20260831_03/` with
`ROS_DOMAIN_ID=228`. Host preflight passed with `/dev/dri/renderD128`; runtime
preflight passed with 3,600 samples, aggregate RTF
`0.999999901305919`, and median RTF `1.000007350054780`. The persistent graph
observer reached a complete unique graph and held it stable for `2.009 s`;
all prescribed lifecycle/controller/action/service/topic/OMPL/ownership checks
passed before the recorder started.

The recorder started before exactly one Product 101 stage. The stage exited 0
with the exact line:

`GATE 6 1.0 KG COMPLETE 1 KG PASS`

`ros2 bag info` reports a valid sqlite3 bag with 191,936 messages,
90.156199656 s duration, and 115.4 MiB size. The original analyzer invocation
returned FAIL only for command forwarding. Source tracing and bag replay found
1,066 nonzero simulation outputs; all matched an exact arbitration command at
or before output within 0.25 s. The base adapter's independent 50 ms timer
explains the expected one-tick lag.

`src/amr_manipulation/scripts/gate6_evidence_analyzer.py` now validates that
historical trace rather than only the newest prior sample. It still rejects
unowned values and stale outputs. Corrected reanalysis of the actual captured
bag returned:

`GATE6_BAG_ANALYSIS=PASS product_id=101`

The analyzer regression tests cover both the accepted one-tick delay and
rejection cases. No Product 101 attempt was repeated; this run consumed the
single permitted retry. Gate 6's second independent 1 kg pass is accepted.

### Validation and boundary

The combined build/test command for `amr_factory`, `amr_mpc_controller`, and
`amr_manipulation` passed with 271 tests, 0 errors, 0 failures, and 5 skipped;
`git diff --check` passed. Evidence includes
`.ros_logs/gate6_1kg_repeat2_graphfix_20260831_03/evidence/graph_readiness.txt`,
`product101_bag_info.txt`, `product101_analysis_corrected.txt`, and
`shutdown_process_scan.txt`. Shutdown and post-shutdown host preflight passed
with no runtime processes left. No 3 kg, 5 kg, Product 102/103, or Gate 7
runtime was started; a new explicit evidence review and authorization is still
required before any such work.

## Authorized higher-mass execution boundary — 2026-08-31

One explicitly authorized combined 3 kg/5 kg execution used the exact run
`.ros_logs/gate6_3kg_5kg_20260831_01` and `ROS_DOMAIN_ID=230`. The orchestration
recorded `t0_monotonic=24880.26`, `hard_cutoff_monotonic=42520.26`, and
`script_start_monotonic=26249.81`; approximately `22:49.55` had elapsed at
script start/leaving and approximately `4:31:10.45` remained to the cutoff.
Final handoff timing is recorded in
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/final_timing.txt`: approximately
`34:15.36` had elapsed from T0 and `4:19:44.64` remained to the cutoff.

The required host preflight failed closed before source/installed hashes,
factory, MoveIt, or recorder at
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/host_preflight/host_preflight.txt`.
It recorded `render_devices=<none>`, `forced_software=<none>`,
`known_processes=<none>`, and `verdict=FAIL` because no readable/writable
`/dev/dri/renderD*` device was available. Product 102/3 kg and Product 103/5
kg attempts remained 0/not started. No stage, product runner, recorder, bag,
analyzer, or `ros2 bag info` ran.

Cleanup passed its process gate. The post-shutdown host preflight was clean of
runtime processes but still failed the render check at
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/post_shutdown/host_preflight.txt`.
The elevated rerun was rejected; no retry or workaround is authorized.
Before launch, the authorized deterministic analyzer defect was fixed in
`src/amr_manipulation/scripts/gate6_evidence_analyzer.py` with regression
coverage in `src/amr_manipulation/test/test_gate6_completion_contract.py`:
preparation publishes a first nonzero boot stream, and the analyzer now
fail-closes/selects exactly one later stream carrying `Gate 6 mass stage is
starting`; single-stream Product 101 compatibility is preserved. The live
higher-mass attempt itself changed no source. The exact existing motion source
fix remains `src/amr_manipulation/src/gate6_mass_stage.cpp`; 37 focused pytest
checks, 274 package tests (0 errors, 0 failures, 5 skipped), the build, and
`git diff --check` remain green. This attempt does not establish higher-mass
acceptance. The next blocker is a fresh explicitly authorized direct-host run
with a readable/writable approved render node.

## Product 102 retry `_11` — planning lifecycle response boundary — 2026-08-31

The next valid-domain direct-host run used `ROS_DOMAIN_ID=222`, UDPv4 Fast DDS,
and the persistent lifecycle preflight. Host setup, graph settling, cleanup,
and post-shutdown host checks passed; Product 102 and Product 103 attempts
remained zero. The run stopped at lifecycle readiness after 30 seconds.

The first causal event was in the planning lifecycle transition, not in the
preflight observer. `lifecycle_manager_planning` requested planner
configuration at `1788193620.372792034`. `planner_server` completed plugin
configuration at `1788193620.398379201`, then logged at
`1788193620.506363734`:

`failed to send response to /amr/planner_server/change_state (timeout): client will not receive response`

The manager therefore never logged `Activating planner_server`; the persistent
observer recorded `planner_server` and `global_costmap` as `ok:2:inactive` at
the final deadline. The later product-camera response timeout and trailing
`not_queried` entries were deadline consequences. The planner process stayed
alive to normal cleanup and emitted no plugin/configuration error. Runs `_07`
through `_10` completed the same planning transition successfully, so this is
classified as an intermittent lifecycle/startup race with Fast DDS/RMW
response-path loss, not a Product 102 source failure or an observer bug.

The frozen next hypothesis is a single 1.0-second construction barrier before
`lifecycle_manager_planning` starts, using the existing controller launch
pattern. Planner/smoother nodes, lifecycle acceptance, and all safety,
ownership, attachment, tolerance, and product semantics remain unchanged.
Evidence and the Luna/max implementation packet are preserved at
`.ros_logs/gate6_product102_retry_20260831_11/evidence/post_run_root_cause.txt`.

## Product 102 retry `_12` — recovery precise-dock progress boundary — 2026-08-31

The fresh direct-host retry used run
`.ros_logs/gate6_product102_retry_20260831_12/` and a new valid ROS domain.
Host preflight, runtime timing, persistent graph/lifecycle readiness,
controller readiness, actions/services/topics, MoveIt/OMPL, bootstrap,
ownership, cleanup, shutdown, and post-shutdown host gates all passed. The
recorder finalized a valid Product 102 bag. Product 102 preparation consumed
one attempt; Product 103 remained at zero attempts.

### First causal event

Preparation reached the registered pickup dock and then executed the bounded
recovery sequence after the initial precise dock's localization discrepancy.
The approach recovery and AMCL relocalization completed. The final precise
recovery plan was published at `1788194438.272287846`, and the controller
accepted it at `1788194438.273704767`. The first causal failure was:

  `1788194453.873780266` — `controller_server: Failed to make progress`

The resulting follow-path abort, mission abort, and runner endpoint error were
logged at `1788194453.874104023`, `1788194453.874380827`, and
`1788194453.875928988`. The harness's missing mass-stage log is downstream;
the mass stage never started.

### Causal mechanism and classification

Bag replay of the unchanged global `PoseProgressChecker` requirements
(`0.20 m`, `0.20 rad`, `10.0 s`) shows the final recovery precise action had
mission-feedback progress of only `0.160357 m / 0.125214 rad` at the predicted
10.050144-second failure. An independent localization-odometry replay predicts
the same failure at `1788194453.818374157`, after 10.033329 seconds, with
`0.137067 m / 0.113642 rad`. This is deterministic evidence of the controller
progress boundary, not a DDS response loss.

The same bag's independent ground truth ended at
`(2.374303, 0.012507, 0.148726)` for target `(2.4, 0, 0)`: position error
`0.028579 m` and yaw error `0.148726 rad`, both inside the unchanged final
`0.03 m / 0.15 rad` gate. Product 102 remained at `(3.25, 0, 0)`, and MPC,
control, and simulation command streams remained owned and consistent. The
AMCL future-transform warning at `1788194433.403329611` was followed by a
confirmed new sample at `1788194433.492877014`, so it is not causal.

Classification: product/source bug. The runner already handles a typed,
bounded `STATUS_ABORTED` for the initial precise dock, but the final precise
dock in the recovery branch is unhandled. The runner therefore exits before
the existing fresh physical dock and product-geometry proof can evaluate the
physical pose. No controller tuning, timeout increase, tolerance change,
route change, command-ownership change, or safety relaxation is justified.

### Frozen next action

Sol/high recorded the full packet at
`.ros_logs/gate6_product102_retry_20260831_12/evidence/post_run_root_cause.txt`.
Luna/max is limited to `gate6_product_test.py` and
`test_product_test_contract.py`: handle only an in-window
`NavigationAbortedError` on the final recovery precise dock, wait for the
existing stationary boundary and a fresh physical pose, and then execute the
unchanged independent final/product proof. Out-of-window and non-aborted
failures remain fail-closed. Luna must not plan or replan independently. The
focused checks must pass before one new targeted runtime; Product 103 and Gate
7 remain blocked.

### Recovery proof implementation and focused validation

Luna/max implemented the frozen change in only
`src/amr_manipulation/scripts/gate6_product_test.py` and
`src/amr_manipulation/test/test_product_test_contract.py`. The final recovery
precise dock now accepts only the existing typed terminal-abort path whose
localized XY is inside the existing recovery window, then waits for stationarity
and a fresh physical pose before reaching the unchanged independent dock and
product proof. All other failures remain fail-closed; no retry, timeout,
tolerance, controller, route, ownership, or safety change was made. Luna did
not plan or replan independently and ran no integrated runtime.

Focused validation passed: py_compile, 10 product contract tests, the
`amr_manipulation` build, 36 package tests, and `git diff --check`. Independent
verification also passed with `python3 -m pytest` (10 tests) and the package
test-result summary (36 tests, zero errors/failures/skips). The exact record is
`.ros_logs/gate6_product102_retry_20260831_12/evidence/implementation_validation.txt`.

The next action is one new clean-host, direct-host Product 102-only runtime
with the installed/source hash updated and a pre-run stale-process scan. Stop
at the first failed gate; do not start Product 103 or Gate 7.

## Product 102 retry `_13` — physical dock gate / map-consistency boundary — 2026-09-01

The fresh direct-host Product 102-only retry used
`.ros_logs/gate6_product102_retry_20260831_13/`. Host, runtime timing,
persistent graph/lifecycle, controller, MoveIt/OMPL, bootstrap, ownership,
cleanup, shutdown, and post-shutdown host gates passed. Product 102 consumed
one preparation attempt; Product 103 remained at zero attempts.

The `_12` implementation fix was effective: the final precise recovery action
was accepted and the controller logged `Reached the goal!` at
`1788195662.116643123`. The first causal `_13` failure was the unchanged
independent physical dock proof at `1788195662.119917155`:
`pickup dock tolerance failed: position=0.0355 yaw=0.1336`. The mass stage
never started, so the missing mass-stage log is a downstream harness verdict.

The finalized bag correlates the independent streams at the action result:

| Evidence | Pose/result |
| --- | --- |
| Ground truth | `(2.365221, 0.006927, 0.133608)` |
| Mission feedback / composed AMCL TF | `(2.390527, -0.002632, 0.138662)` |
| Composed map-vs-ground discrepancy | `0.027052 m / 0.005054 rad` |
| Settled ground-truth dock error | `0.035006 m / 0.133608 rad` |

Over the final precise leg, ground truth moved `0.524498 m`; raw Gazebo,
simulation, and wheel odometry each moved `0.524263 m`, while filtered
localization moved `0.524912 m`. The physical and wheel/raw streams therefore
agree; the AMCL-owned `map→odom` correction is what makes mission feedback
accept the goal while physical truth remains short. The product remained
stationary and command streams remained owned and consistent. No DDS,
lifecycle, controller-progress, analyzer, or host failure occurred.

The canonical map/world assets are inconsistent at the same boundary. The SDF
defines each pickup pedestal at `(3.30, y, 0)` with size `(0.40, 0.50, 0.75)`;
pickup_b therefore occupies `x=[3.10,3.50]`, `y=[-0.25,0.25]`. At the
canonical map origin and resolution, the original PGM pickup-b component
occupied columns `182..190` and image rows `94..104`, while the SDF-derived
block is columns `182..189` and image rows `95..104`; pickup_a and pickup_c
had the same extra top row and rightmost column. This is classified as a
Gazebo/simulation map-consistency issue, not a product-runner acceptance
defect. The runner's physical gate remains unchanged and fail-closed.

Sol/high froze the implementation packet at
`.ros_logs/gate6_product102_retry_20260831_13/evidence/post_run_root_cause.txt`.
The only authorized implementation is to reconcile those three PGM regions
with the existing SDF grid and add focused asset-contract coverage. Preserve
the map metadata, station registry, AMCL/TF ownership, route and retry
semantics, all tolerances, safety gates, and unrelated work. Luna/max must not
plan or replan independently. Focused map tests/build must pass before one new
clean-host direct-host Product 102 run.

### Corrected map implementation and focused validation

Luna/max implemented the frozen packet in only
`src/amr_factory/maps/factory.pgm` and
`src/amr_factory/test/test_factory_assets.py`; Luna did not plan or replan
independently and did not run integrated runtime validation. The PGM header,
dimensions, and metadata were preserved. The three former occupied blocks
were corrected from columns `182..190` / rows `34..44`, `94..104`, and
`154..164` to the SDF-derived columns `182..189` / rows `35..44`, `95..104`,
and `155..164`, changing 57 payload bytes. The final source and installed
asset hash is
`c9da32376a478c3f52ca7f1624e06e2d60eb772c872fd7e64bb0b26b8d2a7b01`.

The strengthened contract test asserts the exact SDF-derived occupied sets,
including the previously omitted top-left boundary cells and the former
rightmost edge. Luna's focused validation passed 13 asset tests, the
`amr_factory` build, and 63 package tests with zero errors/failures/skips.
Independent validation reproduced 13/13 pytest, the 63-test package result,
`git diff --check`, the source/installed hash match, and the protected
`AMR_CODEX_HANDOFF.md` hash. Full record:
`.ros_logs/gate6_product102_retry_20260831_13/evidence/implementation_validation.txt`.

The next step is one fresh direct-host Product 102-only run with a new run
identity and ROS domain, after a direct-host stale-process and rendering
preflight. Stop at the first failed gate; do not start Product 103 or Gate 7.
