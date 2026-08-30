# AMR Session Handoff

## Authority and stop condition

The user explicitly authorized Phase 14 implementation according to
`docs/PHASE_14_FACTORY_MOBILE_MANIPULATION.md`, gate-by-gate, with no progress
past a failed gate. Preserve the dirty worktree. Do not modify
`AMR_CODEX_HANDOFF.md`; do not commit or push. Request approval before installing
dependencies or downloading external assets.

Work stopped at the user's quota condition. The next elevated live-launch
request returned a usage-limit rejection with reset time August 20, 2026 at
11:16 AM. No simulation or MoveIt process is intentionally left running.

## Gate progress

- Gates 1-5: passed.
- Gate 6 empty motion: passed live at the required 0.2 velocity and acceleration
  scaling and returned to the fixed stow tolerance.
- Gate 6 1 kg stage: in progress and still failed closed.
- Gate 6 3 kg and 5 kg, Gate 7, completion documentation, and full workspace
  acceptance: not started. Do not advance to them.

## Gate 6 evidence and fixes

- Added the Gazebo Contact system and product/finger contact plumbing.
- Added fail-closed base/product reference evidence, attachment pose and fresh
  bilateral product-contact checks, exact product IDs, and positive Gazebo
  attachment confirmation.
- Patched the vendored `gz_ros2_control` node construction so controller YAML is
  applied before the hardware node starts. The active simulation position gain
  was proven as `0.5`; strict 0.05 rad path and 0.01 rad goal tolerances remain.
- Use `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for live ROS processes; Fast DDS shared
  memory port initialization became unreliable. No new middleware was installed.
- Graphical inspection and installed Gazebo examples proved that every
  `DetachableJoint` starts attached. `gate6_mass_stage` now commands and
  positively confirms all products 101/102/103 detached before any gripper or
  arm command. This removed the false loads and product dragging.
- The collision-checked Cartesian staging retreat passes live with zero
  base/product displacement.
- The pre-grasp now uses current-state-seeded exact IK for `gripper_tcp`, then
  retains an independent wrist-branch guard. Staging, seeded pre-grasp, and the
  Cartesian grasp approach all passed live with zero base/product displacement.
- The last live boundary failed before attachment because finger positions
  reached exactly `0.0275 m`, leaving the same 0.100 m gap as the handle width;
  no contact was generated. The source now commands `0.020 m` so the fingers
  should stall on the handle and generate bilateral contact. This change builds
  and all 11 `amr_manipulation` tests pass, but quota stopped the fresh live run.

No live attachment has yet been accepted in the corrected sequence. Lift and
loaded stow have therefore not been exercised. The current mass executable also
does not implement the required transport/place half of the 1 kg acceptance
stage, so do not mark the 1 kg gate or Gate 6 complete even if grasp/stow passes.

## Next actions after quota reset

Use a fresh runtime because Gazebo publishes `detached` only on a real startup
`attached -> detached` transition; a second mass-stage invocation in the same
runtime correctly fails closed rather than inferring a no-op detach.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export GZ_PARTITION=amr_phase14_gate6_contact_retry
export ROS_DOMAIN_ID=99
ros2 launch amr_factory factory_localization.launch.py \
  headless:=true initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

In a second terminal with the same environment:

```bash
ros2 launch amr_manipulation move_group.launch.py
```

After MoveIt reports ready, in a third terminal with the same environment:

```bash
ros2 launch amr_manipulation gate6_mass_stage.launch.py product_id:=101
```

At the contact boundary, verify the finger joints stall above the 0.020 m
command, both native contact topics contain `product_a`, the application sees
both contacts no older than 100 ms, the measured grasp transform is within
30 mm / 0.15 rad, and attachment is positively confirmed. Do not weaken any of
those gates. If grasp/stow passes, implement and validate 1 kg transport/place
before considering the 1 kg stage passed; then proceed to 3 kg only after the
full 1 kg acceptance gate passes.

## Latest validation and important files

- `colcon build --packages-select amr_manipulation --symlink-install`: pass.
- `colcon test --packages-select amr_manipulation`: pass.
- `colcon test-result --test-result-base build/amr_manipulation --verbose`:
  11 tests, zero errors, zero failures, zero skips.
- No full-workspace Phase 14 validation has run.

Primary current files include:

- `src/amr_manipulation/src/gate6_mass_stage.cpp`
- `src/amr_manipulation/test/test_moveit_config.py`
- `src/amr_description/config/phase14_mobile_manipulator_controllers.yaml`
- `third_party/gz_ros2_control/src/gz_ros2_control_plugin.cpp`
- `src/amr_factory/worlds/factory.sdf`

Known follow-up risks include the MoveIt Humble class-loader segfault on clean
Ctrl-C shutdown, the mass executable's incomplete live placement acceptance,
the fixed pickup-dock/costmap collision documented below, and the lack of
complete-workspace validation. Preserve all unrelated dirty-tree content.

## Phase 14 continuation — 2026-08-21

The corrected 1 kg grasp and loaded-stow boundary was rerun in a fresh runtime
(`GZ_PARTITION=amr_phase14_gate6_contact_retry`, `ROS_DOMAIN_ID=99`) and passed:
the boot detach transitions, bilateral `product_a` contacts, finger-position
proof above the `0.020 m` request, attachment confirmation, 80 mm lift, loaded
stow, and dock-reference stability all passed. The required negative
out-of-dispatch detachment check also passed without publishing a detach
request; native state remained `attached`.

The complete 1 kg run remains failed closed at the first loaded navigation
leg. Fresh retries (`ROS_DOMAIN_ID=96` and `91`) reached the same boundary and
reported `Optimizer fail to compute path` / `path following failed` while
navigating from the fixed pickup dock toward the registered pickup approach.
The product remained attached and the stage published `FAULT`.

Read-only diagnosis in an isolated baseline runtime (`ROS_DOMAIN_ID=87`) found
the local costmap's robot-center cell at the pickup dock was lethal (`253`)
because the fixed pickup pedestal is within the configured 1.20 x 0.80 m
footprint/inflation envelope. A temporary correctly-QoS'd reverse command moved
the disposable robot from approximately `(2.4, 3.0)` to `(2.009, 2.958)`; the
same registered approach goal then succeeded. This confirms a fixed dock /
costmap geometry blocker, not an interlock-QoS failure. No footprint,
tolerance, registry pose, or safety check was weakened.

`factory_localization.launch.py` now delays the localization lifecycle manager
by 5 seconds so map_server and AMCL services are ready before the first
configure request. A fresh runtime (`ROS_DOMAIN_ID=86`) automatically reached
active states for both nodes, logged AMCL pose `(2.400, 3.000, 0.000)`, and
published `map -> odom` translation `(2.400, 3.000, 0.000)`.

Validation after the continuation edits:

- `colcon build --packages-select amr_description amr_control amr_factory amr_manipulation --symlink-install`: pass.
- `colcon test --packages-select amr_description amr_control amr_factory amr_manipulation`: pass.
- `colcon test-result --verbose`: 162 tests, 0 errors, 0 failures, 5 skipped.
- `python3 -m py_compile` passed for the three changed launch files.

The 1 kg acceptance gate is **not passed**. Do not start 3 kg, 5 kg, Gate 7,
or completion documentation. The next action requires an explicitly approved
resolution of the fixed pickup-dock/costmap collision; under the Phase 14 plan,
stop and report this blocker rather than changing registry positions, the
robot footprint, tolerances, or fixed stow pose. All ROS, MoveIt, and Gazebo
processes were stopped; no commit or push was made.

## Phase 14 continuation — 2026-08-21 GUI gripper timeout fix

`gate6_mass_stage.cpp` now gives an accepted gripper goal an internal 30-second
wall-clock result limit. The server and goal-acceptance waits remain 3 seconds,
and the 0.020 m close, 0.035 m open, 60 N effort, controller settings, grasp
geometry, and bilateral evidence gates are unchanged. The helper retains the
accepted goal handle, checks the result code and pointer before reading result
fields, requires `reached_goal || stalled`, logs requested/measured position,
elapsed wall time, and result flags, and fails closed with verified cancellation
and terminal-result checks if the 30-second limit expires.

The manipulation source-contract test now asserts the 30-second result wait,
retained goal handle, cancellation call and ordering, result guards, success
flags, and post-close bilateral position/contact checks.

Automated validation:

- `colcon build --packages-select amr_description amr_control amr_factory amr_manipulation --symlink-install`: pass.
- `colcon test --packages-select amr_description amr_control amr_factory amr_manipulation`: pass.
- `colcon test-result --verbose`: 162 tests, 0 errors, 0 failures, 5 skipped.

Fresh GUI live validation used `GZ_PARTITION=amr_phase14_gate6_gui_timeout_fix`,
`ROS_DOMAIN_ID=98`, `headless:=false`, and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.
Gazebo reported `real_time_factor=0.1973`. Product 101 open completed in 1.241 s;
the close completed in 6.478 s, beyond the former 5-second client limit and
before 30 seconds, with result `SUCCEEDED`, `reached_goal=false`,
`stalled=true`, measured position `0.0269 m`, and bilateral positions
`0.0269/0.0281 m`. Fresh bilateral contact, attachment, 80 mm lift, loaded
stow, and the required out-of-dispatch detachment rejection passed; native
state remained attached. The stage then failed closed at the pre-existing
loaded navigation pickup-dock/costmap blocker. Afterward the gripper action
graph showed one active server and zero clients; this controller did not expose
an action-status topic for a stronger historical-goal query. All runtime
processes were stopped, with the known MoveIt Humble clean-shutdown segfault
recurring during Ctrl-C cleanup.

This verifies the GUI gripper timeout fix only. The 1 kg acceptance gate remains
not passed, and no later gate or placement work is authorized until the fixed
pickup-dock/costmap collision receives an explicit Phase 14 decision.

## Phase 14 continuation — bounded pickup-dock egress — 2026-08-21

The approved bounded reverse egress is implemented. Pickup stations A/B/C now
register collinear 0.50 m egress poses at `(1.90, 3.00, 0.0)`, `(1.90, 0.00,
0.0)`, and `(1.90, -3.00, 0.0)`. The command-arbitration lifecycle node remains
the sole `/amr/control/cmd_vel` publisher and now owns the internal
`/amr/control/dock_egress` `nav2_msgs/action/BackUp` server. It accepts only
fresh, semantically valid loaded-stow/READY/filtered-odometry/rear-LiDAR/TF
evidence, commands straight negative X through the existing clamps, checks the
complete footprint swept corridor plus one 0.05 m cell, ignores Nav2 samples
while reserved, and publishes zero with an exact terminal reason on every
success, cancellation, timeout, stale-evidence, obstacle, drift, deactivation,
or interlock-loss path. Gate 6 retains the accepted goal handle, waits 65 s,
verifies cancellation and terminal `CANCELED` on client timeout, retains the
attachment, and only then starts the unchanged Nav2 pickup-approach goal.

The integrated factory's 10 Hz rear lidar can be slower in wall time when
Gazebo runs at low real-time factor, so the configuration-backed freshness
deadline is 1.0 s; missing or malformed data still fails closed. An initial
headless attempt with the prior 0.2 s deadline failed safely as `rear LiDAR
evidence is stale` after 0.227 s, with measured travel 0.001 m.

Validation after the egress implementation:

- `colcon build --packages-select amr_control amr_manipulation amr_factory
  --symlink-install`: pass.
- Full `colcon test --event-handlers console_direct+`: 167 tests, 0 errors,
  0 failures, 5 skipped. The focused egress action tests covered success,
  straight reverse output, Nav2 sample clearing, malformed requests, and an
  obstructed rear scan; the contract suite passed 23 Python tests.
- A fresh headless product-101 run (`GZ_PARTITION=amr_phase14_gate6_headless_3`,
  `ROS_DOMAIN_ID=97`) logged `Dock egress SUCCEEDED: requested=0.500 m`,
  command-arbitration measured travel `0.501 m`, and elapsed wall time `8.981 s`.
  The following unchanged Nav2 goal reached the registered pickup approach;
  the later dispatch-approach leg failed with the pre-existing MPPI
  `Optimizer fail to compute path` condition. Product attachment was retained
  and the stage failed closed.
- A fresh Gazebo GUI product-101 run (`GZ_PARTITION=amr_phase14_gate6_gui_egress`,
  `ROS_DOMAIN_ID=96`) logged egress success at `0.500 m` in `6.093 s`; the
  pickup-approach goal reached, and the same later dispatch-approach leg failed
  closed. `ros2 topic info /amr/control/cmd_vel -v` showed exactly one publisher,
  `amr/command_arbitration_node`.
- The obstructed/malformed control test is the simulated egress-failure
  injection: the goal was rejected before motion and no delayed Nav2 command
  replayed. No custom action/message was added.

The 1 kg Gate 6 acceptance gate remains **not passed** because the subsequent
dispatch navigation leg still fails; this change is limited to the approved
pickup-dock egress and does not alter Nav2 footprint/inflation, grasp geometry,
attachment rules, or velocity ownership. Do not start the 3 kg or 5 kg stages,
Gate 7, or completion documentation. All ROS, MoveIt, and Gazebo processes from
the headless and GUI runs were stopped; no commit or push was made.

## Phase 14 continuation — 2026-08-21 1 kg XY-tolerance verification

The minimal approved controller change is applied: `amr_mpc_controller/config/controller.yaml`
now sets `goal_checker.xy_goal_tolerance: 0.07`; the existing controller
contract test requires exactly `0.07`. `yaw_goal_tolerance: 0.15`, progress
checking, MPPI limits, footprint/inflation, egress, manipulation, attachment,
and acceptance settings were unchanged.

Focused validation passed:

- `GZ_VERSION=harmonic colcon build --packages-select amr_mpc_controller --symlink-install`
  completed successfully.
- `colcon test --packages-select amr_mpc_controller` completed successfully;
  `colcon test-result --test-result-base build/amr_mpc_controller --verbose`
  reported 6 tests, 0 errors, 0 failures, 0 skipped.

A fresh headless Harmonic product-101 run used
`GZ_PARTITION=amr_phase14_gate6_xy_tolerance_070`, `ROS_DOMAIN_ID=94`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and a writable `ROS_LOG_DIR` under
`.ros_logs/xy_tolerance_070`. The run reached the following boundaries before
failing closed:

- Gripper open succeeded (`0.0350 m` requested, `0.0335 m` measured); close
  stalled against the product (`0.0200 m` requested, `0.0270 m` measured), with
  bilateral positions `0.0270/0.0280 m`.
- Grasp, attachment confirmation, 80 mm lift, loaded stow, dock-reference
  stability, and the required out-of-dispatch detachment rejection passed;
  native product state remained attached and no valid dispatch detachment was
  requested.
- The bounded reverse egress succeeded at the unchanged `0.10 m/s` cap:
  requested `0.500 m`, command-arbitration measured travel `0.503 m`, elapsed
  wall time `12.631 s`. The following pickup-approach Nav2 goal reached its
  goal (`controller_server` logged `Reached the goal!`).
- The next dispatch-approach goal was accepted, but the stage's unchanged
  120-second wait expired and it exited with code 1 at
  `navigation to dispatch approach failed`. `controller_server` subsequently
  logged `Failed to make progress` and aborted the path-following handle. The
  factory runtime also logged a mission-supervisor `UnknownGoalHandleError`
  during the transition.

The run did not reach the dispatch dock ground-truth check, placement,
detachment, gripper opening, retreat, empty stow, or final success publication.
The 1 kg Gate 6 acceptance gate remains **not passed**. All ROS, MoveIt, and
Gazebo processes, including stale launch groups found before the run, were
stopped; no additional navigation tuning, recovery logic, later mass stage, or
phase advancement was performed.

## Phase 14 continuation — Reliable 1 kg Gate 6 implementation and bounded acceptance run — 2026-08-21

The approved reliability plan was implemented without changing the URDF, SRDF,
gripper geometry, stow target, allowed-collision pairs, navigation footprint or
inflation, MPPI safety limits, station poses, progress thresholds, velocity
limits, attachment rules, or acceptance tolerances.

Manipulation now executes the approved `grasp -> lift_checkpoint (+0.080 m) ->
clearance_retreat (pregrasp)` sequence as one collision-checked Cartesian path,
restores the temporary pickup collision allowance before stow, verifies fresh
MoveIt `/check_state_validity` evidence for `manipulator` (including returned
contact pairs), and calls `setStartStateToCurrentState()` before loaded stow.
The manipulation contract test covers the ordered waypoints, one Cartesian
calculation/execution, allowance restoration, and validity proof.

Navigation now wraps the existing MPPI controller with
`nav2_rotation_shim_controller::RotationShimController` using the exact plan
parameters and keeps the existing MPPI namespace/limits. The package declares
the rotation-shim runtime dependency, and the controller contract test checks
the wrapper, primary controller, rotation parameters, tolerances, and safety
settings.

The mission supervisor now uses explicit `IDLE`, `PLANNER_PENDING`,
`PLANNER_ACTIVE`, `CONTROLLER_PENDING`, `CONTROLLER_ACTIVE`, and `CANCELING`
states with one public mission identity, explicit cancel/result races, stale
callback rejection, downstream handle clearing, defensive
`UnknownGoalHandleError` handling, ROS-clock feedback timing, and latest
`map -> base_footprint` feedback poses. Runtime behavior tests cover planning
cancel, following cancel, sequential missions, abort/deactivation, result
races, malformed goals, and exactly-once public completion.

Focused verification passed:

- `GZ_VERSION=harmonic colcon build --packages-select amr_description
  amr_mpc_controller amr_mission amr_manipulation amr_factory --symlink-install`.
- The matching focused `colcon test` command passed all packages.
- `colcon test-result --verbose`: **179 tests, 0 errors, 0 failures, 5 skipped**.
- Integrated controller logs confirm `RotationShimController`, its internal
  `MPPIController`, configuration, and lifecycle activation:
  `.ros_logs/reliable_kg_20260821/factory/controller_server_406003_1787292844418.log`.

One fresh product-101 run was performed exactly once after the focused gates,
with `GZ_PARTITION=amr_phase14_gate6_reliable_kg_20260821`, `ROS_DOMAIN_ID=124`,
Harmonic headless factory/MoveIt, and evidence bag
`.ros_logs/reliable_kg_20260821/product101_evidence_full` (274.0 MiB,
517065 messages, 132.820725 s). The run evidence is:

- Grasp preparation and bilateral contact passed; the continuous retreat
  produced a 100% Cartesian path, loaded stow completed, and the required
  out-of-dispatch detachment rejection passed with native attachment retained.
- Dock egress succeeded: requested `0.500 m`, measured `0.502 m`, elapsed wall
  `8.291 s`.
- Pickup approach succeeded: target `(1.500, 3.000, 0.000)`, localized
  `(1.602, 3.006, -0.029)`, XY error `0.102 m`, yaw error `0.029 rad`.
- Dispatch approach was accepted but the controller failed to make progress;
  the mission supervisor reported `Mission aborted: path following failed`
  without a process exception. Terminal evidence was code `6` (ABORTED), target
  `(-2.500, 0.000, 3.142)`, localized `(-2.481, 0.101, -3.134)`, XY error
  `0.102 m`, yaw error `0.007 rad`, distance remaining `0.000`, simulation
  navigation time `43.503 s`, ground truth `(-2.409, 0.122, -0.000)`.

The run stopped immediately at that boundary, so dispatch placement,
detachment, empty stow, and final success were not claimed. The 1 kg Gate 6
acceptance gate remains **not passed**. No retry, parameter tuning, recovery
logic, later mass/gate work, commit, or push was performed. All ROS, MoveIt,
Gazebo, stage, and rosbag processes were stopped afterward.

## Phase 14 continuation — MPPI near-goal fix acceptance evidence — 2026-08-21

The bounded MPPI fix was implemented and independently verified. The reusable
agent workflow was added to `AGENTS.md`; `GoalAngleCritic`, `PathAlignCritic`,
and `PathAngleCritic` `threshold_to_consider` are each `0.07`, matching the
unchanged `xy_goal_tolerance`. The focused contract, five-package build/test,
and test-result gates passed: **179 tests, 0 errors, 0 failures, 5 skipped**.

One fresh product-101 run was then performed exactly once with
`GZ_PARTITION=amr_codex_gate6_final_20260821_01`, `ROS_DOMAIN_ID=130`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and evidence bag
`.ros_logs/gate6_final_20260821_01/product101_evidence` (246.8 MiB,
466020 messages, 139.340439437 s). Controller logs confirmed the
`RotationShimController`, internal MPPI controller, and 0.070 critic threshold
were loaded and activated.

- Grasp preparation, bilateral contact, continuous Cartesian retreat,
  loaded stow, and out-of-dispatch detachment rejection passed.
- Dock egress succeeded: requested `0.500 m`, elapsed `9.757 s`.
- Pickup approach succeeded: target `(1.500, 3.000, 0.000)`, localized
  `(1.529, 3.046, -0.141)`, XY error `0.055 m`, yaw error `0.141 rad`.
- Dispatch approach succeeded: target `(-2.500, 0.000, 3.142)`, localized
  `(-2.441, 0.037, -2.995)`, XY error `0.070 m`, yaw error `0.147 rad`,
  distance remaining `0.000`, simulation navigation time `35.789 s`, ground
  truth `(-2.391, 0.044, -0.000)`.
- Dispatch dock succeeded: target `(-3.400, 0.000, 3.142)`, localized
  `(-3.348, 0.011, -3.066)`, XY error `0.053 m`, yaw error `0.076 rad`,
  distance remaining `0.000`, simulation navigation time `5.233 s`, ground
  truth `(-3.284, -0.018, -0.000)`.
- The first new boundary was exact seeded pre-place IK: `GATE 6 1.0 KG:
  FAIL: exact seeded pre-place IK failed` in
  `.ros_logs/gate6_final_20260821_01/stage/gate6_mass_stage_476609_1787303473591.log`.

The run stopped immediately at that boundary. Placement, authorized
detachment, empty stow, and final success were not claimed; the 1 kg Gate 6
acceptance gate remains **not passed**. No retry, parameter tuning, recovery
logic, later mass/gate work, commit, or push was performed. All runtime
processes were stopped. MoveIt emitted an exit-11 destructor fault during the
deliberate shutdown after the stage had already failed; it did not occur during
the acceptance path.

## Phase 14 continuation — Alignment sequencing follow-up and bounded failure evidence — 2026-08-21

The prior alignment defect was corrected in `src/amr_manipulation/src/gate6_mass_stage.cpp`:
command segment spacing now accounts for the unchanged 0.07 m Nav2 terminal
tolerance (`0.15 - 0.07 = 0.08 m`), translation goals use their travel bearing,
and a separate same-position goal requests the approved dispatch heading. The
contract test was updated accordingly. Focused build/test gates remained green:
179 tests, 0 errors, 0 failures, 5 skipped.

One fresh post-fix product-101 run was performed exactly once with
`GZ_PARTITION=amr_codex_gate6_loop4_20260821_01`, `ROS_DOMAIN_ID=132`, and
evidence bag
`.ros_logs/amr_codex_gate6_loop4_20260821_01/bag/product101_evidence`.
Grasp, continuous retreat, validity proof, loaded stow, negative detachment
rejection, egress, pickup approach, dispatch approach, dispatch dock, all five
bounded alignment segments, and the final heading goal passed. The first new
boundary was loaded pre-place OMPL planning:

```
GATE 6 1.0 KG: FAIL: pre-place planning failed
```

MoveIt reported a 27-state path with an invalid state at index 19 due to an
unallowed `base_link <-> held_product` collision:
`.ros_logs/amr_codex_gate6_loop4_20260821_01/moveit/move_group_558919_1787309292855.log`.
The failing source path is `arm.setStartStateToCurrentState()` followed by
`arm.plan(pre_place_plan)` in `gate6_mass_stage.cpp` (lines 1883-1888); the
new placement stance has collision-free IK/release endpoints, but the OMPL
stow-to-pre-place path enters the chassis with the attached product. No ACM,
geometry, or collision tolerance was relaxed. The stage, recorder, MoveIt, and
factory processes were stopped immediately after this boundary. Placement,
detachment, empty stow, and final success remain unclaimed.

## Phase 14 continuation — Placement calculations, final-bias correction, and product-101 closure — 2026-08-21

The placement coordinates were evaluated against the production URDF and joint
limits before being used. The analysis included FK/IK reachability at the
release and pre-place heights, the Nav2 0.07 m terminal-position tolerance,
the 0.15 m bounded alignment segment, and attached-product collision geometry.
It also exposed that the first kinematic candidates were not sufficient: the
held product could intersect `base_link` during the loaded-stow-to-pre-place
transition, and the achieved pose could differ from the commanded pose by the
Nav2 tolerance. No ACM, URDF/SRDF geometry, or safety tolerance was relaxed.

The final source correction in
`src/amr_manipulation/src/gate6_mass_stage.cpp` obtains a fresh ground-truth /
localized pose pair immediately before the final heading goal, computes the
fresh bias, rejects a non-finite or out-of-envelope bias change, and composes
the final command from the physical placement pose and that fresh bias. The
contract test in `src/amr_manipulation/test/test_moveit_config.py` asserts this
ordering. Focused and five-package verification remained green:

```
colcon test-result --verbose: 179 tests, 0 errors, 0 failures, 5 skipped
```

Intermediate runtime evidence was captured and stopped at each boundary:

- Loop 5 rejected the older placement stance because its derived total motion
  exceeded the existing 0.35 m bound.
- Loop 6 reached alignment but failed the unchanged yaw envelope by 0.0014 rad
  (`0.1514 > 0.15`).
- Loop 7 reached final heading but failed the unchanged physical XY envelope:
  `0.0742752 m > 0.07 m`; this produced the fresh-bias correction above.
- Loop 8 was an invalid launch setup (`initial_x=-4.5, initial_y=0`) and was
  stopped at the first bilateral-contact gate; it never attempted a valid
  pickup.
- Loop 9 was stopped before MoveIt/stage because the factory localization
  lifecycle stalled while configuring `map_server`; no product result was
  claimed.

The corrected product-101 acceptance run was performed once with
`GZ_PARTITION=amr_codex_gate6_loop10_20260821_01`, `ROS_DOMAIN_ID=140`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, initial pose `(2.4, 3.0, 0)`, and evidence
bag `.ros_logs/amr_codex_gate6_loop10_20260821_01/bag/product101_evidence_run`
(339.9 MiB, 714292 messages, 154.878 s). The stage log is
`.ros_logs/amr_codex_gate6_loop10_20260821_01/stage/gate6_mass_stage_637839_1787313233340.log`.
The run evidence is:

- MoveIt logged 100% Cartesian completion for the continuous retreat and all
  placement Cartesian segments; loaded planning used the conservative
  `RRTConnect` edge validation and completed successfully.
- Bilateral contact, attachment, continuous retreat, validity/stow, and the
  out-of-dispatch detachment rejection passed.
- Dock egress succeeded for `0.500 m` in `9.005 s`; decoded egress commands
  were capped at `0.1000000015 m/s`.
- Pickup approach, dispatch approach, and dispatch dock all returned code 4.
  All five placement translation segments and the final heading goal returned
  code 4. The final heading evidence was localized XY error `0.058 m` and yaw
  error `0.135 rad`.
- Decoded attachment state was `detached -> attached -> detached`; final
  manipulation state was `STOWED_EMPTY`, `product_attached=false`, with detail
  `Gate 6 1.000000 kg grasp, transport, placement, and empty stow passed`.
- The final product pose was `(-4.094762, 0.505473, 0.075000)`, `0.007576 m`
  from the slot target `(-4.10, 0.50, 0.075)`.
- The terminal stage line was:

  `GATE 6 1.0 KG COMPLETE 1 KG PASS`

All factory, MoveIt, stage, and rosbag processes were stopped afterward and a
process scan found no remaining runtime processes. The rosbag recorder warned
that hidden action topics were not recorded because `--include-hidden-topics`
was not supplied; terminal outcomes remain in the stage log and the run did
complete successfully. MoveIt emitted its known destructor exit-11 during the
deliberate post-run shutdown, after the successful terminal result and not
during the acceptance path.

## Phase 14 continuation — Requested recheck stopped at deterministic placement envelope — 2026-08-21

The requested fresh recheck used
`GZ_PARTITION=amr_codex_gate6_loop12_20260821_01`, `ROS_DOMAIN_ID=142`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and the validated initial pose `(2.4, 3.0,
0)`. Factory localization, controller-manager, RotationShim/MPPI activation,
MoveIt, and complete hidden-topic rosbag recording all became ready.

The run passed grasp, bilateral contact, continuous retreat, loaded stow,
negative detachment rejection, egress, pickup approach, dispatch approach,
dispatch dock, five bounded alignment translations, and the final heading
goal. It then failed closed before pre-place planning:

```
GATE 6 1.0 KG: FAIL: placement target was outside the deterministic IK envelope
```

The exact guard is `gate6_mass_stage.cpp:1846-1848`, using the unchanged
`kMaxPlacementReleaseRadius = 0.785` at line 1609. The final fresh ground-truth
pose in the evidence bag produced release-base coordinates
`(0.535057, -0.579631)` and radius `0.788834 m`, exceeding the guard by
`0.003834 m` (3.8 mm). This is a safety-envelope rejection, not an OMPL or
controller crash. The stage log is
`.ros_logs/amr_codex_gate6_loop12_20260821_01/stage/gate6_mass_stage_651186_1787313980084.log`;
the complete evidence bag is
`.ros_logs/amr_codex_gate6_loop12_20260821_01/bag/product101_evidence_run`
(329.6 MiB, 687334 messages, including navigation action feedback/status).

All factory, MoveIt, stage, and rosbag processes were stopped immediately
after this boundary. No coordinate, tolerance, geometry, or safety parameter
was changed and no further retry was performed.

## Manual developer handoff — latest GUI navigation oscillation — 2026-08-21

The user will continue the remaining work manually. Do not claim that the
product-101 path is reliable: loop 10 completed once, loop 12 later failed the
deterministic placement envelope, and the latest GUI rerun failed earlier at
the dispatch approach. No source or configuration file was changed during the
latest rerun. All Gazebo, ROS, MoveIt, and stage processes were stopped after
the failure, and a process scan found none remaining.

### Latest reproducible failure

The latest run used `headless:=false`, initial pose `(2.4, 3.0, 0)`,
`GZ_PARTITION=amr_codex_gate6_factory_only_20260821_01`, `ROS_DOMAIN_ID=143`,
and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Grasp, bilateral contact, attachment,
the continuous retreat, loaded stow, negative pickup detachment rejection,
the 0.500 m egress, and the pickup approach all passed. The loaded navigation
to the registered dispatch approach then visibly moved back and forth and
failed closed:

```
target=(-2.500, 0.000, 3.142)
terminal code=6 (ABORTED)
localized=(-2.400, -0.052, -2.609)
xy_error=0.113 m
yaw_error=0.533 rad
distance_remaining=0.000 m
simulation navigation time=40.402 s
GATE 6 1.0 KG: FAIL: navigation to dispatch approach failed
```

The controller accepted the goal, emitted 225 instances of `Control loop
missed its desired rate of 20.0000Hz`, then logged `Failed to make progress`
and aborted the `follow_path` handle. The stage and controller evidence is:

- `.ros_logs/amr_codex_gate6_factory_only_20260821_01/stage/gate6_mass_stage_662541_1787314561693.log`
- `.ros_logs/amr_codex_gate6_factory_only_20260821_01/factory/controller_server_657615_1787314422297.log`

This is a navigation terminal-heading/progress failure, not a grasp, MoveIt,
attachment, or placement failure. The path distance had reached zero while
the unchanged `0.07 m` XY and `0.15 rad` yaw goal conditions were still not
met. The unchanged `PoseProgressChecker` then correctly aborted after the AMR
failed to make the required `0.20 m` or `0.20 rad` progress within 10
simulation seconds. The mission supervisor only propagated the downstream
abort as `path following failed`; its action lifecycle did not crash.

### Source path responsible

The current transport code sends the registered dispatch approach as one
combined translation-and-final-heading goal:

- `src/amr_manipulation/src/gate6_mass_stage.cpp:634-708` constructs and
  monitors every `NavigateToPose` goal.
- `src/amr_manipulation/src/gate6_mass_stage.cpp:1575-1580` directly calls
  `navigate_to(product.dispatch_approach, 120s)` and then the dispatch dock.
- `src/amr_factory/config/stations.yaml:28-32` defines the unchanged dispatch
  approach `(-2.5, 0.0, pi)` and dock `(-3.4, 0.0, pi)`.
- `src/amr_mpc_controller/config/controller.yaml:3-48` owns the 20 Hz
  controller, unchanged progress/goal tolerances, RotationShim wrapper, and
  MPPI workload. Do not begin by weakening these safety/acceptance values.
- `src/amr_mission/src/mission_supervisor_node.cpp:304-376` forwards the
  planner path to `FollowPath`; lines 349-374 clear the controller handle and
  propagate a non-success result. This code is not the source of the
  oscillation in the latest run.

From the fresh pickup-approach terminal pose, the straight travel bearing to
the dispatch approach is approximately `-2.51 rad`, while the same goal also
requires the final station yaw `pi` (equivalently `-3.142 rad`). The controller
therefore consumes the diagonal path and must solve the remaining heading at
the same terminal boundary. The zero remaining path distance, large terminal
heading error, visible reversing, repeated controller-rate misses, and final
progress abort are consistent with that coupled goal being the immediate
failure mechanism.

### Smallest proposed manual fix

Do not change the station registry, footprint, inflation, velocity and
acceleration limits, `0.07 m` / `0.15 rad` goal tolerances, progress checker,
attachment gates, or cancellation semantics. Change only the dispatch-
approach sequencing in `gate6_mass_stage.cpp`:

1. After the existing pickup-approach success, obtain a fresh localized pose.
2. Build a translation target using the registered dispatch-approach X/Y and
   `atan2(target_y - current_y, target_x - current_x)` as its travel yaw.
3. Navigate to that translation target with the existing `navigate_to(...,
   120s)` helper and require the product to remain attached.
4. Obtain another fresh localized pose, then send a second, same-position goal
   using that achieved X/Y and the unchanged registered dispatch-approach yaw.
   This resets `PoseProgressChecker` for the terminal rotation and prevents
   MPPI from fighting the diagonal path heading and final station heading in
   one action.
5. Re-prove fresh attachment after both subgoals, then retain the existing
   registered dispatch-dock goal unchanged.

The already-implemented placement-alignment pattern at
`gate6_mass_stage.cpp:1693-1705` computes travel-bearing translation goals;
the pattern at lines `1721-1767` performs a separate final-heading goal. Reuse
that sequencing concept without copying its placement-specific coordinate,
bias, or displacement logic into the dispatch approach.

Update `src/amr_manipulation/test/test_moveit_config.py` to reject the old
direct `navigate_to(product.dispatch_approach, 120s)` call and assert: fresh
pose before bearing calculation, registered X/Y retained, `atan2` travel yaw,
translation navigation before heading navigation, fresh pose between them,
unchanged registered final yaw, and attachment proof after each action. This
proposal is evidence-backed but has not been implemented or runtime-verified.

If a focused headless rerun still reports controller-rate misses after the
goal split, stop and measure that timing problem separately. Do not guess a
new `controller_frequency`, MPPI `batch_size`, model horizon, velocity limit,
progress threshold, or tolerance during the same change. The latest GUI run
proves an overrun exists, but it does not isolate which MPPI workload change
would be safe.

### Expected next boundary after navigation

Even if dispatch navigation is corrected, placement is not yet proven
repeatable. Loop 12 reached the deterministic release-radius guard at
`src/amr_manipulation/src/gate6_mass_stage.cpp:1846-1848`: the achieved
release radius was `0.788834 m`, exceeding the unchanged `0.785 m` envelope by
`0.003834 m`. Do not raise or remove that guard. If it recurs, correct the
bounded placement-alignment goal/terminal-pose strategy so the fresh achieved
pose stays inside the existing envelope, then keep the exact IK, collision,
30 mm placement, attachment, and detachment gates unchanged.

### Manual verification order

After the source and contract-test edit, run and require zero failures:

```bash
GZ_VERSION=harmonic colcon build --packages-select \
  amr_description amr_mpc_controller amr_mission amr_manipulation amr_factory \
  --symlink-install

colcon test --packages-select \
  amr_description amr_mpc_controller amr_mission amr_manipulation amr_factory

colcon test-result --verbose
```

Then use a new `GZ_PARTITION`, `ROS_DOMAIN_ID`, and log directory for one
headless product-101 run with hidden action topics recorded. Inspect the
controller log for missed-rate warnings and require both dispatch-approach
subgoals, dispatch dock, bounded placement alignment, deterministic IK
preflight, collision-checked placement, valid detachment, empty stow, and the
exact terminal line `GATE 6 1.0 KG COMPLETE 1 KG PASS`. Stop at the first
failed boundary and stop every runtime process. Do not proceed to product
102/103 or later gates until product 101 completes reliably in a fresh run.

## Bounded CAD-visual corrective revision — 2026-08-24

The approved corrective scope keeps the legacy export untouched and uses the
fail-closed derived mesh step for visuals. The active Xacro uses CAD meshes for
appearance and primitive chassis/wheel/caster/sensor collisions, preserving the
public frames, topics, watchdog, DiffDrive, pose/joint-state plugins,
fail-closed ownership boundary, and `1.20 x 0.80 m` footprint. Provisional
values remain drive radius `0.1128 m`, separation `0.566 m`, base height
`0.0478 m`, caster radius/width `0.0393/0.0421 m`, and base mass `22.15 kg`
with positive box inertia. The untouched `amr_urdf_cad` source is not edited.

The composite removes `arm_pedestal_link` and mounts the six-joint articulated
KUKA directly at `base_link` `xyz="0 0 0.33"`; generic payload remains
base-only default-on and composite default-off. Wheel odometry and Gazebo use
the same radius/separation.

Focused source validation now covers the mesh derivation, description,
localization, controller, mission, factory, and manipulation contracts. No
live Gazebo, MoveIt, factory, or Gate 6 acceptance evidence is claimed here.

## Phase 14 source continuation — 2026-08-24

The current source state supersedes the earlier “primitive-only” wording above,
but not the historical runtime results. Derived CAD meshes are active as
visuals with explicit CAD colors; chassis, wheel, caster, and sensor collisions
remain conservative primitives. The baked arm, mounting plate, and centered
lower pedestal are excluded from the derived base visual. The composite has no
`arm_pedestal_link`; the articulated `KR6 R900-2` mounts directly on `base_link`
at `z=0.33`, flush with the AMR top.

The controller source now uses direct Humble Regulated Pure Pursuit with a
provisional 0.50 m/s target, curvature regulation, and approach slowdown. The
factory world source uses a 2 ms physics step, RTF 1.0, and disabled shadows.
The precise placement endpoint shares the mission supervisor's one-goal
reservation/cancel state and selects a non-stateful 5 mm XY checker; normal
navigation remains 70 mm/0.15 rad. Gate 6 dispatch is split into bearing
translation, fresh-pose registered-yaw rotation, and dock navigation, while
placement alignment/final heading use the precise endpoint only.

Gate 7 source boundaries are implemented and fail closed: manipulation action
and status, factory transport action, mode service, manual/autonomous FIFO
capacity, cancellation, held-product fault retention, factory demo launch,
CLI, and interface ownership entries. They are not runtime acceptance
evidence.

Fresh source validation after this continuation:

- focused Python contracts: 41 passed;
- focused C++/ROS build: `amr_interfaces`, `amr_mission`, `amr_manipulation`,
  `amr_factory`, and `amr_mpc_controller` passed;
- full workspace `colcon build --symlink-install`: all 17 packages succeeded;
- full workspace `colcon test-result --verbose`: 202 tests, 0 errors, 0
  failures, 5 skipped;
- `check_urdf` passed for the base and composite URDFs, and `gz sdf -p` passed
  for the corresponding SDFs with `SDF_PATH` set; and
- no live Gazebo/MoveIt/factory run or runtime acceptance has been claimed for
  the 2 ms/RPP revision.

The next authorized runtime pass must use fresh partitions and stop at the first
failed gate: performance baseline, empty motion, two consecutive 1 kg passes,
then 3 kg and 5 kg. Gate 7 orchestration runtime remains blocked until Gate 6
is repeatable. `AMR_CODEX_HANDOFF.md` remains protected and was not edited.

## Next-session handoff — Phase 14 runtime validation — 2026-08-24

Objective: validate the current 2 ms/RPP/CAD-visual factory stack live, starting
at the first unproven runtime gate.

Baseline: all 17 packages build; full `colcon test-result` reports 202 tests,
0 errors, 0 failures, and 5 skipped; base/composite `check_urdf` and factory
`gz sdf -p` pass with `SDF_PATH`; no live runtime acceptance exists for this
revision.

Current source facts: derived CAD meshes are visuals; chassis, wheel, caster,
and sensor collisions are primitives; no mounting plate or pedestal is retained;
the KUKA mounts directly on `base_link` at `z=0.33`; RPP desired speed is
`0.50 m/s` with approach/curvature regulation; factory targets RTF 1.0 with a
2 ms physics step and shadows off.

Critical Gate 7 limitation: the manipulation supervisor public action is
fail-closed because the Gate 6 executor hook is not wired. Do not claim Gate 7
completion or bypass/duplicate motion ownership.

Preserved invariants/non-goals: fail-closed command ownership and the existing
frames, topics, footprint, tolerances, stow, attachment, contact, and place
gates remain unchanged. Do not tune or weaken gates, install/download anything,
commit, or push; `AMR_CODEX_HANDOFF.md` remains protected and untouched.

Next authorized runtime order: use fresh `GZ_PARTITION`, `ROS_DOMAIN_ID`, and
`ROS_LOG_DIR` for the performance/RTF baseline, Gate 6 empty motion, two
consecutive 1 kg passes, then 3 kg and 5 kg; stop at the first failed gate.
Only after repeatable Gate 6 may Gate 7 integration/runtime proceed.

Worktree warning: the tree is heavily dirty; rerun `git status --short` and
preserve everything. Follow `AGENTS.md`: use Sol/high for analysis/evidence,
then Luna/max for approved implementation and focused verification.

## Phase 14 Gate 6 runtime evidence — 2026-08-24

The first authorized Gate 6 performance pass used `GZ_PARTITION=gate6_d151_perf_empty_20260824_01`, `ROS_DOMAIN_ID=151`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, headless startup, and initial pose `(2.4, 3.0, 0)`. Factory localization activated, controllers loaded, and MoveIt became ready. The applicable baseline remains fresh: `colcon test-result --verbose` reports 202 tests, 0 errors, 0 failures, and 5 skipped.

After warmup, the raw 65 s capture was bounded to the exact first 60 s window: 389 samples spanning `59.941606464 s`; median RTF `0.016324721300072727`, minimum `0.0082633773367679405`, maximum `1.2633751952704286`, mean `0.38295381894476921`, and aggregate sim/real advance `0.3236483161600171`. The required median RTF `>= 0.90` therefore **FAILS**. No `Control loop missed its desired rate` warnings were emitted. The stop rule was applied: empty motion was not started; command ownership/profile checks and bag capture were not performed/applicable; domain 152 plus product 101 runs, products 102/103, and Gate 7 were not started.

Evidence supports, but does not prove as the sole cause, a likely environment/rendering bottleneck: runtime logged a failed MESA DRM query, failed iris driver, missing `/dev/dri/card1`, and absent `/dev/dri`; the active world uses DART at 2 ms/OGRE2 with two `720x4` 10 Hz GPU lidars and a `640x480` 10 Hz RGB-D sensor; the RGB-D consumer received camera info but zero image messages; and the trace alternated approximately `0.01` stalls with partial recoveries. No retry, knob, or profile change is authorized from this evidence.

Artifacts: `.ros_logs/gate6_d151_perf_empty_20260824_01/evidence/rtf_60s_summary.txt`, `stats_60s.txt`, `factory/console.log`, `moveit/console.log`, and `evidence/shutdown_process_scan.txt`. The shutdown scan contained only its header and found no Gazebo, ROS, MoveIt, recorder, or Gate 6 processes. Gate 6 remains **NOT runtime accepted**; Gate 7 is explicitly pending and out of scope. This update changes no source, configuration, tuning, interface, or tests; no commit or push was performed. `AMR_CODEX_HANDOFF.md` remains protected.

## Authoritative current status — D205 Gate 6 product 101 — 2026-08-24

The authorized runtime scope was exactly one complete 1 kg Gate 6 run. D205
accepted product 101 once; no second 1 kg run, 3 kg run, 5 kg run, or Gate 7
run was authorized or started. Gate 7 remains pending.

Verified run identity and artifacts: `GZ_PARTITION=gate6_d205_product101_loop28_20260824_01`,
`ROS_DOMAIN_ID=205`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, headless startup,
initial pose `(2.4, 3.0, 0)`, and artifact path
`.ros_logs/gate6_d205_product101_loop28_20260824_01`. The supplied loop26
directory name is not present; this loop28 path is the exact current D205
directory. Strict factory readiness passed in 16 s, MoveIt readiness in 1 s,
the adapted rear-lidar sample, ownership checks, and recorder checks passed.

Performance passed with median RTF `0.99981793313616918` (approximately
`0.999818`), aggregate RTF `0.9945095342459388` (approximately `0.994510`),
and zero controller-rate misses. The stage produced the exact terminal line
`GATE 6 1.0 KG COMPLETE 1 KG PASS`. The pickup bearing/heading split,
corrected dock target, four-segment placement alignment and final heading,
map-aligned held-product collision-free IK/OMPL/lower sequence, `0.000894 m`
placement error, native `detached -> attached -> detached` state sequence,
37-point/100% retreat, ACM restoration, post-retreat state-validity check,
and empty stow all passed. The observed command caps remained `0.50 m/s` and
`0.40 rad/s`; bag analysis found no unmatched simulation profiles. The bag was
`239.8 MiB` with `412,523` messages, and shutdown completed cleanly.

The verdict's `BAG_ANALYSIS=FAIL` was a stale analyzer false report: it
required `/amr/mission/navigate_to_pose_precise/_action/status`, while the
precise action is retired by design. D205 correctly recorded zero messages on
that retired action and used normal navigation status; zero precise messages
is therefore expected, not a runtime failure. Historical failed runtime
sections above remain unchanged.

## Phase 14 continuation — independent 3 kg and 5 kg preparation implementation — 2026-08-28

The user selected separate, independently runnable 3 kg and 5 kg tests. The
accepted 1 kg command and operational path remain unchanged; do not rerun the
1 kg simulation as part of this split. The AMR must retain its current pose,
while the selected product is reset to its registered pickup-station pose at
the beginning of the test. The user also required reset refusal when the arm
is attached, deployed, moving, faulted, or not at empty stow, and requested no
subagents.

### Implemented source path

- `src/amr_manipulation/scripts/gate6_product_test.py` is the persistent
  preparation runner and accepts only product IDs 102 and 103.
- `gate6_3kg_test.launch.py` selects product 102; `gate6_5kg_test.launch.py`
  selects product 103. Each runner keeps the factory and MoveIt session alive.
- Preparation confirms fresh base/arm/product/attachment evidence, checks for
  an active Gate 6 stage, detaches the startup attachment transitions, pauses
  Gazebo, sets only the selected product pose, unpauses, verifies reset
  stability and unchanged AMR/unselected-product poses, then navigates from
  the AMR's current pose to the selected pickup dock.
- Preparation publishes `FAULT` and stops fail-closed on invalid authority,
  unsafe arm state, reset-service failure, navigation failure, or timeout;
  timed-out navigation goals are explicitly canceled. A per-ROS-domain lock
  prevents concurrent product preparation runs.
- `factory_localization.launch.py` bridges the Gazebo world control and
  set-pose services. The existing mass stage is reused for the grasp,
  transport, placement, and empty-stow sequence. Its terminal message is now
  mass-aware; the 1 kg line remains exactly `GATE 6 1.0 KG COMPLETE 1 KG PASS`.
- Focused contract coverage was added in
  `src/amr_manipulation/test/test_product_test_contract.py`; the command
  reference and beginner guide document the separate test commands and RViz
  option.

### Fresh source validation

- `PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile ...`: pass.
- `python3 -m pytest -q src/amr_manipulation/test/test_product_test_contract.py`:
  5 passed.
- `colcon build --packages-select amr_factory amr_manipulation
  --symlink-install`: pass.
- `colcon test --packages-select amr_factory amr_manipulation` followed by
  `colcon test-result --verbose`: 220 tests, 0 errors, 0 failures, 5 skipped.
- Both alias launch files parsed successfully with `--show-args`; the
  installed executable is `amr_manipulation gate6_product_test`.

### Current boundary and next action

Live 3 kg and 5 kg runtime evidence has not been collected. Gate 6 therefore
remains open: product 101 is accepted from D205, but products 102 and 103 are
not runtime accepted. Run the 3 kg alias first, stop at its first failed gate,
and run the 5 kg alias only after the 3 kg result is reviewed. Do not use the
optional factory supervisor at the same time as a manual product test. Gate 7
and completion documentation remain blocked until the independent 3 kg and
5 kg runtime checks pass. `AMR_CODEX_HANDOFF.md` remains untouched.

## Phase 14 continuation — renderer guard and runtime preflight — 2026-08-28

### Objective and diagnosis

The objective was to prevent the Gazebo black-screen/lag condition from being
mistaken for a valid Gate 6 runtime and to make performance evidence
repeatable for both headless and GUI demonstrations.

The degraded environment had no accessible `/dev/dri/renderD*`, no
`video`/`render` group membership, and `software_rendering:=auto` selected
llvmpipe. Gazebo server and GUI each consumed more than one CPU core; a fresh
10 s sample contained 660 observations with aggregate RTF `0.2251409655` and
median RTF `0.7081588`. Gate 6 then correctly failed closed when its unchanged
freshness evidence became too old. The accepted D205 hardware-rendered run
remains the reference: median RTF `0.9998179331`, aggregate RTF `0.9945095342`,
zero controller-rate misses, and exact terminal line
`GATE 6 1.0 KG COMPLETE 1 KG PASS`.

### Implemented source changes

Only the following paths were changed for this continuation; all other dirty
worktree content is user-owned and must be preserved:

- `src/amr_factory/launch/factory_localization.launch.py`
  - Added `require_hardware_rendering:=false`.
  - Strict mode rejects forced software OpenGL and missing or inaccessible
    `/dev/dri/renderD*` before Gazebo starts.
  - Existing `software_rendering:=auto|true|false` behavior remains available
    when strict mode is disabled.
- `src/amr_factory/launch/factory_demo.launch.py`
  - Forwards both rendering arguments to the localization launch.
- `src/amr_factory/scripts/factory_runtime_preflight.py`
  - Added installed `host` and `runtime` checks.
  - Host mode records DRM access, forced-renderer variables, group membership,
    and stale simulation processes without killing anything.
  - Runtime mode verifies Gazebo owns a DRM device and captures `/stats` for a
    fixed 12 s window. It requires at least 10 samples, median RTF `>= 0.90`,
    and aggregate RTF `>= 0.90`.
- `src/amr_factory/CMakeLists.txt`
  - Installs the preflight executable and its pytest contract.
- `src/amr_factory/test/test_factory_assets.py`
  - Covers the strict rendering launch contract.
- `src/amr_factory/test/test_factory_demo_contract.py`
  - Covers rendering argument passthrough.
- `src/amr_factory/test/test_factory_runtime_preflight.py`
  - Covers DRM detection, forced software detection, D205-like passing stats,
    degraded stats, insufficient samples, and non-positive time spans.
- `docs/SIMULATION_COMMANDS.md`
  - Documents strict launch arguments, preflight order, RTF gates, absolute
    `ROS_LOG_DIR`, corrected rosbag continuation syntax, and the rule to keep
    RViz and the optional factory supervisor out of performance evidence.

### Validation evidence

- `GZ_VERSION=harmonic colcon build --packages-select amr_factory
  --symlink-install`: passed.
- `colcon test --packages-select amr_factory`: passed.
- `colcon test-result --test-result-base build/amr_factory --verbose`: 27
  tests, 0 errors, 0 failures, 0 skipped.
- Focused pytest: 19 passed.
- Python compilation and `ros2 launch ... --show-args`: passed.
- Installed command is discoverable as
  `ros2 run amr_factory factory_runtime_preflight.py`.
- Current host preflight deliberately failed with
  `render_devices=<none>` and exit 1.
- Strict launch deliberately aborted before Gazebo with
  `hardware rendering required, but no readable/writable
  /dev/dri/renderD* device is available`.
- The new parser independently reproduced the D205 reference as 115 samples,
  median `0.999818`, aggregate `0.994510`.

### Preserved invariants and non-goals

Do not weaken Gate 6 freshness, timing, controller, contact, attachment,
placement, stow, command-ownership, or fail-closed behavior. Do not reduce
world sensors, alter physics, tune motion limits, add a software fallback to
strict evidence mode, run the optional factory supervisor beside manual Gate 6,
or start products 102/103 or Gate 7 from this handoff. Do not commit, push,
install dependencies, download assets, or modify `AMR_CODEX_HANDOFF.md`.

### Current boundary and next action

No live Gate 6 or GUI evidence pass was started by this continuation. No
Gazebo, MoveIt, rosbag, or Gate 6 processes were intentionally left running.
The current agent environment lacks `/dev/dri`, so it cannot prove the GUI
stage. Runtime evidence must be authorized after source review and run from a
direct host terminal with accessible DRM render nodes and matching
`GZ_PARTITION`, `ROS_DOMAIN_ID`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and
workspace-local `ROS_LOG_DIR` in every terminal.

Run the host preflight first, then strict headless localization, then the
runtime RTF check. Stop before MoveIt, recording, or the product stage on any
failed check. A GUI repeat may proceed only after the headless RTF and Gate 6
boundaries pass; keep RViz out of the measured GUI run. Preserve the fresh
preflight reports, raw stats, stage bag, logs, screenshots, and shutdown scan
under the run-specific `.ros_logs` directory.

## Phase 14 continuation — direct-host 1 kg runtime result — 2026-08-28

### Objective and runtime boundary

The user requested a fresh full 1 kg simulation after restoring the current
working source. The run used the direct host because the normal sandbox has no
visible `/dev/dri`; no source, physics, sensor, motion-limit, tolerance, or
rendering fallback changes were made for this run.

The repository was rebuilt and tested before runtime. The run used strict
headless hardware rendering with `GZ_VERSION=harmonic`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `ROS_LOCALHOST_ONLY=1`,
`ROS_DOMAIN_ID=231`, `GZ_PARTITION=amr_gate6_1kg_host_20260828_01`, and
`ROS_LOG_DIR=/home/pete/amr_ws/.ros_logs/gate6_1kg_host_20260828_01`.

### Validation and runtime evidence

- `colcon build --symlink-install --executor parallel --parallel-workers 4`:
  passed all 17 packages.
- Full `colcon test --executor parallel --parallel-workers 4`: 228 tests,
  0 errors, 0 failures, 5 skipped.
- Host hardware preflight: `PASS`; Gazebo used `/dev/dri/renderD128`.
- Runtime preflight: `PASS`; 3,586 samples over 11.994 s, median RTF
  `0.9999445031`, aggregate RTF `0.9963141486`.
- Navigation lifecycle nodes and arm/gripper controllers were active; both
  front and rear acceptance LiDAR topics published.
- MoveIt became ready and the prescribed recorder captured 262,194 messages
  in a 143.6 MiB bag over 144.394 s.

### Terminal result

The stage passed gripper setup, bilateral stall evidence, pickup, attachment
safety rejection, dock egress, pickup approach, transfer navigation, and
placement alignment. It stopped fail-closed at the placement-lower gate:

`GATE 6 1.0 KG: FAIL: Cartesian placement lower was incomplete`

MoveIt reported only 80% Cartesian completion for the final lower path (33
points). Therefore the 1 kg simulation is not complete or accepted from this
run. Do not claim Gate 6 completion and do not retry or tune from this result
without a reviewed fix plan.

### Artifacts and shutdown

Evidence is preserved under
`.ros_logs/gate6_1kg_host_20260828_01/`, including:

- `evidence/host_preflight.txt`
- `evidence/runtime_preflight.txt`
- `evidence/stats_raw.txt`
- `evidence/stats_stderr.txt`
- `product101_evidence/`
- `gate6_mass_stage_34326_1787890830548.log`
- `move_group_31981_1787890731880.log`

The recorder finalized successfully and the factory shut down. MoveIt emitted
a segmentation fault during SIGINT teardown after the stage had already
failed; this is a separate unresolved shutdown risk and was not the Gate 6
failure. No simulation processes were intentionally left running.

### Current worktree and next action

The worktree remains intentionally dirty with 99 changed/untracked entries;
these include the restored Phase 14 source and prior user artifacts. `HEAD`
remains commit `00b8cfa1d07af043df5ba830e54c55ec5e978ab0`, while the restored
latest source is present as uncommitted working-tree content. Preserve all
unrelated changes. Do not reset, clean, stage, commit, push, or modify
`AMR_CODEX_HANDOFF.md` without explicit direction.

Next action is diagnosis and a bounded fix plan for the incomplete final
Cartesian lower path, followed by focused validation and a fresh strict host
runtime pass. Preserve the fail-closed placement, attachment, ownership,
timing, and safety gates; do not weaken acceptance criteria or overload the
hardware to force completion.

## Phase 14 continuation — factory product startup and GUI diagnosis — 2026-08-28

### Objective and diagnosis

The user reported that factory products were flying and then could not start
the Gazebo GUI. The product motion was reproduced in a clean direct-host
Gazebo partition and traced to the robot xacro, not the factory map or driver:

- `factory.sdf` alone kept products 101/102/103 at their registered shelf
  poses.
- The full robot with `factory_attachment:=true` moved all three products and
  emitted duplicate fixed-joint/shape-name warnings.
- The same full robot with `factory_attachment:=false` kept all three products
  stationary.

The cause is the Gazebo `DetachableJoint` contract: configured pairs start
rigidly attached and detach only after a command. See the
[Gazebo DetachableJoint documentation](https://gazebosim.org/api/sim/9/detachablejoints.html).
The prior factory launch hardcoded `factory_attachment: "true"` even though
the products begin on shelves.

The GUI failure in the restricted agent environment was a separate host
boundary: strict rendering correctly stopped when `/dev/dri/renderD*` was not
visible. A direct-host launch found `/dev/dri/renderD128`, kept `gz sim -g`
running, and `xdpyinfo` connected successfully to `DISPLAY=:0`.

### Implemented source changes

Only these paths were changed for the product-startup fix; preserve all other
dirty worktree content:

- `src/amr_factory/launch/factory_localization.launch.py`
  - Added `factory_attachment:=false` as the ordinary factory default.
  - Resolves the launch-time xacro mapping after launch arguments are known.
  - Keeps native attachment topics and the existing explicit attachment mode.
- `src/amr_factory/launch/factory_demo.launch.py`
  - Forwards the new argument and defaults it to false.
- `src/amr_factory/test/test_factory_assets.py`
  - Covers the launch-time attachment option and default.
- `src/amr_factory/test/test_factory_demo_contract.py`
  - Covers demo passthrough and default.
- `docs/SIMULATION_COMMANDS.md`
  - Shows the normal factory and demo commands with
    `factory_attachment:=false`.

No factory world poses, map geometry, driver, motion limits, command
ownership, attachment proof, or fail-closed gate was changed. The strict
hardware-rendering defaults recorded in the preceding handoff section remain
unchanged.

### Validation evidence

- Focused pytest for factory assets, demo contract, and runtime preflight:
  `19 passed`.
- `colcon build --packages-select amr_factory --symlink-install`: passed.
- `colcon test --packages-select amr_factory --event-handlers
  console_direct+`: all 4 CTest targets passed.
- Python compilation of both factory launch files: passed.
- Direct-host strict headless factory run with the default attachment setting
  used `/dev/dri/renderD128`. Product pose samples remained at
  `product_a=(3.25, 3.0, 0.825)`, `product_b=(3.25, 0.0, 0.825)`, and
  `product_c=(3.25, -3.0, 0.825)`.
- Direct-host GUI startup kept both Gazebo server and GUI processes alive;
  `xdpyinfo -display :0` returned 0. This was a startup diagnosis, not a new
  Gate 6 acceptance run.
- All explicit test Gazebo/ROS processes were stopped afterward. No commit or
  push was made, and `AMR_CODEX_HANDOFF.md` remains untouched.

### Preserved invariants and unresolved risks

The ordinary factory and demo launches now keep shelf products detached and
are suitable for GUI inspection when run from a desktop host with accessible
DRM and X11 devices. The bridge still exposes the declared attachment topics,
but the native Gazebo attachment systems are absent in the default mode.

Gate 6 still requires an explicit `factory_attachment:=true` factory launch
because its native attach/detach proof depends on those systems. Gazebo's
initial attached behavior remains a known property of that explicit mode; the
existing Gate 6 initial-detachment sequence must be revalidated before using
it for acceptance evidence. This continuation does not claim any new Gate 6
pass.

Do not start the independent 3 kg/5 kg runs, Gate 7, or completion
documentation. The authoritative next action remains a diagnosis and bounded
fix plan for the incomplete final Cartesian placement-lower path, followed by
focused validation and a fresh strict host runtime pass. Preserve placement,
attachment, ownership, timing, and safety gates; do not weaken criteria or
modify `AMR_CODEX_HANDOFF.md`.

## Phase 14 continuation — retained placement-lower branch — 2026-08-28

The bounded Gate 6 source correction is implemented in
`src/amr_manipulation/src/gate6_mass_stage.cpp`. The release-to-pre-place IK
continuation now retains every 5 mm solution. After the OMPL pre-place motion,
the executable requires the measured endpoint to match the retained branch
within the existing 0.01 rad goal tolerance, validates every reversed branch
waypoint through `/check_state_validity` with bounds/contact diagnostics, and
time-parameterizes the exact joint trajectory at the unchanged 0.2 scaling.
The release endpoint is checked after time parameterization. Existing
placement, contact, attachment, ownership, timing, and fail-closed gates are
unchanged.

Focused validation is green:

- `colcon build --packages-select amr_manipulation --symlink-install`: passed.
- `colcon test --packages-select amr_manipulation`: 5 tests, 0 failures, 0
  errors.
- `git diff --check` for the changed source and contract test: passed.

The source contract test was updated to assert the retained-branch validation
and execution order. No runtime evidence was started. Gate 6 remains failed
closed at the prior direct-host placement-lower boundary; the next authorized
step is a fresh strict host runtime pass. Do not proceed to 3 kg, 5 kg, Gate 7,
or completion evidence until the full 1 kg acceptance gate passes.

## Phase 14 continuation — runtime host preflight blocked — 2026-08-28

The next authorized validation attempt stopped at the required host preflight
before Gazebo startup. The current agent environment reported
`render_devices=<none>` and failed closed with
`no readable/writable /dev/dri/renderD* device`. Evidence is preserved at
`.ros_logs/gate6_1kg_retry_20260828_02/evidence/host_preflight.txt`.

No simulation, MoveIt, Gate 6 mass stage, 3 kg/5 kg run, or Gate 7 process was
started. A fresh direct-host run with an accessible DRM render node remains
required before Phase 14 can advance.
