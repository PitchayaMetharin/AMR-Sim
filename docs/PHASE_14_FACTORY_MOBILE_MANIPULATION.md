# Phase 14 — Factory Mobile Manipulation Implementation Plan and Closeout

## Authority and scope

This document preserves the decision-complete Phase 14 implementation handoff
and the current closeout/operator boundary above. The implementation is now
present for the accepted Product 101/102 simulation scope; the dated plan and
diagnostic sections below remain historical engineering authority and do not
supersede the current status. This document does not itself authorize new
implementation, dependency installation, a commit, or a push. Before editing,
read `AGENTS.md`, `SESSION_HANDOFF.md`, and `PROJECT_STATUS.md`, then run
`git status --short` and preserve every existing worktree change. Never modify,
stage, discard, normalize, or commit `AMR_CODEX_HANDOFF.md` without explicit
user direction.

The result remains a one-laptop ROS 2 Humble/Gazebo Harmonic simulation. The
AMR's intended maximum payload is 300 kg, but the repository currently validates
only its lower simulation baseline; Phase 14 must not claim that a physical AMR,
mount, or manipulator is safe or rated for 300 kg. The KUKA arm may manipulate
products of at most 5.0 kg in this simulation.

## Current Phase 14 runtime closeout — 2026-09-08

The approved autonomous factory-cycle scope is complete for Product 101 (1 kg)
and Product 102 (3 kg). Fresh direct-host runs passed the documented host,
runtime, graph, lifecycle, MoveIt, registry, bootstrap, status, ownership, and
fail-closed gates. `_11` passed the normal Product 101 cycle; `_13` passed the
normal Product 102 cycle and independent analyzer; `_14` passed navigation and
retained-product cancellation behavior; and `_16` passed graceful stop after
the active Product 101 delivery while leaving the queued Product 102 delivery
unstarted.

The retained compact evidence is under
`.ros_logs/amr_autonomous_factory_20260908_{11,13,14,16}/evidence/`:
run-level gate outputs, analyzer/CLI/status windows, the cancellation and
graceful-stop summaries, and the targeted `_16` bag. Full-topic raw bags were
pruned after the evidence was recorded; `.ros_logs` is capped below 1 GB.
Product 103/5 kg, Gate 7, physical hardware, and functional-safety claims
remain outside this phase.

All later dated sections in this implementation plan are retained historical
diagnosis and acceptance planning; they do not supersede this closeout.

## Current operator boundary

The canonical autonomous entry point is
`src/amr_factory/launch/factory_autonomous.launch.py`. Use
`control_mode:=autonomous` and pass `factory_attachment:=true` explicitly for
the accepted native-attachment path. Product 101 (1 kg) and Product 102 (3 kg)
are the only autonomous-enabled registry products. Product 103 (5 kg) remains
disabled, and Gate 7, physical hardware, and functional-safety claims remain
out of scope.

`factory_demo.launch.py` is a legacy/optional integration launch. It is not the
canonical autonomous acceptance entry point and must not be used to claim a
current Product 101/102 acceptance result. Use `factory_cli.py` for the
station-selectable send, finite/continuous loop, graceful stop, cancellation,
home, and status boundaries documented in `SIMULATION_COMMANDS.md`.

### Approved Humble/Harmonic compatibility revision — 2026-08-13

The user approved preserving Gazebo Harmonic after the released Humble
`kuka_gazebo` and `gz_ros2_control` binaries were proven to require Gazebo
Fortress and conflict with the installed Harmonic ROS integration. Therefore:

- retain released `kuka_agilus_support` version 1.1.2 for the KUKA model;
- do not install or use the released `kuka_gazebo` binary;
- build project-vendored `gz_ros2_control` 0.7.20 at commit
  `dd35b604a6a60c25da4d4c3838880dea2a24108e` with
  `GZ_VERSION=harmonic`;
- retain the installed `ros_gzharmonic` bridge, simulator, and interface
  packages instead of replacing them with their Fortress binaries; and
- perform Gate 1 with a project-owned standalone KUKA launch that proves the
  same description-expansion and Gazebo trajectory behavior required by the
  original gate.

This compatibility revision changes only the simulator integration mechanism;
all robot-model, control-interface, safety, and gate acceptance requirements
below remain binding.

## Invariants

- Preserve the only base command path: Nav2 or teleop -> command arbitration ->
  base adapter -> Gazebo plant.
- No factory, perception, manipulation, MoveIt, or helper node may publish a
  base velocity command.
- Base motion is allowed only with fresh, valid proof that the arm is in an
  approved stowed pose. Missing, stale, malformed, deployed, moving, or faulted
  manipulator status must produce zero base velocity and reset the acceleration
  ramp.
- The manipulation supervisor must block base motion before sending any arm or
  gripper command. The arm may not deploy while the base is moving.
- A task failure must never report delivery. If failure occurs after attachment,
  preserve the attached-product state and block base motion.
- Automatic recovery is excluded. A manipulation fault requires operator action
  or a simulation restart.

## Fixed robot configuration

### KUKA model and source

Use KUKA `kr6_r900_2` from the released ROS 2 Humble robot-description packages.
Use upstream description geometry, visual/collision meshes, inertias, joint axes,
and joint limits; do not re-create these values. Prefix all imported arm links
and joints with `arm_`.

- Expected released package version: `1.1.2`.
- Upstream Humble reference: commit
  `3a2b8b57e3f8a07847b136ecb168b82d837b5c37`.
- Source: <https://github.com/kroshu/kuka_robot_descriptions/tree/humble>

Do not launch the upstream standalone KUKA world. Include the robot-model macro
inside the composite AMR description and use project-owned ROS 2 Control and
MoveIt configuration for the prefixed composite robot.

### Explicit top-mounted placement

Mount the arm upright **on top of the AMR chassis**, centered only in the
horizontal plane. Do not place or embed any part of the KUKA base inside the
chassis.

The AMR frame convention is x forward, y left, and z upward. Use these poses
relative to the existing `base_link`:

| Item | Value |
| --- | --- |
| Chassis top | `z = 0.33 m` in the active `base_link` visual contract |
| Retained pedestal | None; the lower duplicate pedestal/plate is excluded |
| KUKA mounting plane | `(x, y, z) = (0.0, 0.0, 0.33 m)` |
| KUKA mounting orientation | `rpy = (0.0, 0.0, 0.0)` |
| Fixed joint | `base_link -> arm_base_link` |

The first KUKA axis must point upward and its base must sit flat on the AMR top.
The active composite uses the upstream arm-base mesh underhang directly; no
second primitive pedestal or hidden mounting plate is inserted.

Before accepting the composite description, verify that:

- the arm base does not intersect the chassis or pedestal;
- the empty and loaded stow poses do not collide with the AMR, camera, or
  product;
- the stowed arm and attached product remain inside the navigation footprint in
  XY projection; and
- the product camera retains an unobstructed view of each station from its dock
  pose.

### Stow pose and load budget

Use the following empty and loaded transport stow pose. If it is not
collision-free in the composed robot, stop and report a blocker rather than
inventing another pose.

```text
arm_joint_1 =  0.0
arm_joint_2 = -1.5708
arm_joint_3 =  1.5708
arm_joint_4 =  0.0
arm_joint_5 =  0.0
arm_joint_6 =  0.0
```

- Maximum product mass: 5.0 kg.
- Required test masses: 1.0, 3.0, and 5.0 kg.
- Simulated parallel-jaw gripper mass: 0.8 kg total.
- Camera location: AMR-mounted, not wrist-mounted.
- Reject a configuration if gripper plus product exceeds the arm's 6.0 kg rated
  wrist payload.
- Do not use the optional 6.7 kg maximum-load figure.
- Omit the generic 50 kg payload box in the mobile-manipulator configuration.
- Preserve the existing generic-payload model as the default base-only
  configuration.
- The empty composite AMR, arm, pedestal, gripper, and camera must total between
  80 and 90 kg. Project-owned masses are provisional simulation parameters, not
  hardware specifications.

## Factory world and registry

Create a 12 x 10 m factory world. Port only the required shelves, pallets,
clutter, and decorative assets from AWS Small Warehouse World into local SDF 1.9
resources. Do not require Gazebo Fuel or internet access at runtime, do not add a
submodule, and do not modify `.gitmodules`.

- AWS source reference: commit
  `ee0af733315e78432408c3cd98d378ecee5f767c` on branch `ros2`.
- License: MIT-0; retain the license and an attribution/conversion note beside
  the copied assets.
- Source: <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world>

Use these map-frame poses as the station registry's single source of truth:

| Location | Approach `(x, y, yaw)` | Dock `(x, y, yaw)` | Egress `(x, y, yaw)` |
| --- | --- | --- | --- |
| Home | `(-4.5, 0.0, 0.0)` | N/A | N/A |
| `pickup_a` | `(1.5, 3.0, 0.0)` | `(2.4, 3.0, 0.0)` | `(1.9, 3.0, 0.0)` |
| `pickup_b` | `(1.5, 0.0, 0.0)` | `(2.4, 0.0, 0.0)` | `(1.9, 0.0, 0.0)` |
| `pickup_c` | `(1.5, -3.0, 0.0)` | `(2.4, -3.0, 0.0)` | `(1.9, -3.0, 0.0)` |
| `dispatch` | `(-2.5, 0.0, pi)` | `(-3.4, 0.0, pi)` | N/A |

Use AprilTag family `36h11`, station tag size 0.10 m, product tag size
0.06 m, and `max_hamming: 0`.

| Entity | Tag ID | Product mass |
| --- | ---: | ---: |
| `pickup_a` | 10 | N/A |
| `pickup_b` | 11 | N/A |
| `pickup_c` | 12 | N/A |
| `dispatch` | 20 | N/A |
| `product_a` | 101 | 1.0 kg |
| `product_b` | 102 | 3.0 kg |
| `product_c` | 103 | 5.0 kg |

Products are identical 0.30 x 0.20 x 0.15 m cuboids with a standardized top
handle. Store the fixed product-tag-to-grasp transform in the product registry;
do not infer it from an arbitrary detected bounding box. Provide three separate
dispatch slots so the autonomous A/B/C sequence can complete in one run.

Create a canonical occupancy map with resolution 0.05 m, origin
`(-6.0, -5.0, 0.0)`, and dimensions 240 x 200 cells. Factory runtime uses
`nav2_map_server` and AMCL. AMCL alone owns `map -> odom`; the EKF continues to
own `odom -> base_footprint`. Do not launch online SLAM in factory runtime, but
preserve the current SLAM launch as a separate mapping mode.

## Camera and tag perception

Add an AMR-mounted RGB-D sensor with these simulation settings:

- resolution: 640 x 480;
- rate: 10 Hz;
- horizontal field of view: approximately 60 degrees;
- range: 0.1 to 5.0 m; and
- REP-103 camera and optical frames.

Keep simulator-facing names separate from stable project topics. Add a lifecycle
camera adapter that exposes:

```text
/amr/sensors/product_camera/image_rect
/amr/sensors/product_camera/camera_info
/amr/sensors/product_camera/depth
```

Image and `CameraInfo` timestamps must match. Configure `apriltag_ros` to publish
`apriltag_msgs/msg/AprilTagDetectionArray` on
`/amr/perception/product_tags`.

A product pose is acceptable only when the expected tag ID has:

- hamming distance zero;
- receive age no greater than 250 ms;
- five observations collected within one second;
- position spread no greater than 15 mm; and
- orientation spread no greater than 0.05 rad.

## Packages and interfaces

Add `amr_manipulation` for product-pose validation, MoveIt/gripper execution,
Gazebo attachment coordination, and manipulator status. Add `amr_factory` for
the station registry, job queue, task orchestration, and terminal helper.

Extend `amr_interfaces` with the following contracts.

### `ManipulatorStatus.msg`

```text
std_msgs/Header header

uint8 STARTING=0
uint8 STOWED_EMPTY=1
uint8 STOWED_LOADED=2
uint8 MOVING=3
uint8 DEPLOYED=4
uint8 FAULT=5

uint32 source_boot_id
uint32 sequence
bool valid
uint8 state
bool base_motion_allowed
bool product_attached
string product_id
string detail
```

Publish it at 20 Hz on `/amr/manipulation/status`. Only `STOWED_EMPTY` and
`STOWED_LOADED` may set `base_motion_allowed=true`; `FAULT` always sets it false.

### `ManipulateProduct.action`

```text
uint8 PICK=1
uint8 PLACE=2

uint8 operation
string station_id
string product_id
---
uint8 SUCCESS=0
uint8 BASE_NOT_STATIONARY=1
uint8 PERCEPTION_FAILED=2
uint8 PLANNING_FAILED=3
uint8 EXECUTION_FAILED=4
uint8 GRASP_FAILED=5
uint8 ATTACHMENT_FAILED=6
uint8 INTERLOCK_FAILED=7

uint8 outcome
string message
---
uint8 phase
string phase_name
```

Serve it on `/amr/manipulation/manipulate_product`.

### `TransportProduct.action`

```text
string pickup_station_id
string destination_station_id
---
uint8 SUCCESS=0
uint8 NAVIGATION_FAILED=1
uint8 PICK_FAILED=2
uint8 PLACE_FAILED=3
uint8 CANCELED=4
uint8 INTERLOCK_FAILED=5
uint8 DEPENDENCY_UNAVAILABLE=6

bool delivered
uint8 outcome
string message
---
uint8 phase
uint32 queue_position
string current_station_id
bool product_attached
```

Serve it on `/amr/factory/transport_product`.

### `SetOperationMode.srv`

```text
uint8 MANUAL=0
uint8 AUTONOMOUS=1

uint8 mode
---
bool accepted
string message
```

Serve it on `/amr/factory/set_operation_mode`.

### `FactoryStatus.msg`

Include timestamp, sequence, mode, phase, active flag, queue depth,
pickup/destination station IDs, product ID, attachment state, last outcome, and
diagnostic detail. Publish it at 5 Hz on `/amr/factory/status`.

Record every new topic, action, service, and TF owner in
`amr_bringup/config/interface_ownership.yaml` and add matching contract tests.

## Base/arm interlock

Add these command-arbitration parameters:

```yaml
require_manipulator_stowed: false
manipulator_status_timeout_ms: 200
```

The existing launch retains `require_manipulator_stowed=false`; the factory
launch sets it to `true`. When enabled, arbitration may forward a fresh source
command only when manipulator status:

- arrived within 200 ms according to steady-clock receive time;
- has valid, nonzero boot and sequence identity;
- is monotonic within a boot;
- has `valid=true`;
- is `STOWED_EMPTY` or `STOWED_LOADED`; and
- has a semantically consistent attachment state and
  `base_motion_allowed=true`.

Before any arm or gripper command, the manipulation supervisor must:

1. publish `MOVING` to block base motion;
2. wait at least 400 ms;
3. verify base linear and angular speeds remain below 0.01 m/s and 0.01 rad/s
   for 500 ms;
4. verify fresh `BaseStatus::READY`; and
5. only then send an arm or gripper command.

### Pickup-dock egress

The pickup dock is intentionally close to the pedestal for the KR6 grasp. A
loaded robot therefore leaves the dock through the command-arbitration node's
internal `/amr/control/dock_egress` `nav2_msgs/action/BackUp` server before
normal Nav2 navigation resumes. The registered pickup egress pose is collinear
behind its dock, has the same yaw, lies between the dock and approach, and is
currently exactly 0.50 m away. The server is configuration-backed with a
0.50 m maximum distance, 0.10 m/s maximum speed, a 60 s wall-clock limit, a
1.0 s rear-LiDAR freshness deadline, the 0.05 m drift tolerances, and a 0.05 m
swept-corridor clearance cell.

The action accepts only while the lifecycle node is active and fresh,
semantically valid `STOWED_LOADED`, `READY`, filtered odometry, and rear-LiDAR
plus TF evidence are present. It rejects non-reverse, malformed, concurrent,
stale, or obstructed requests; commands only negative linear X through the
existing sole publisher; clears Nav2 samples during and immediately after the
retreat; and publishes zero on every terminal path. Any failure leaves the
product attached, publishes `FAULT`, and starts no navigation goal.

## ROS 2 Control and MoveIt

The composite description must provide project-owned `gz_ros2_control` position
interfaces for the six prefixed arm joints and gripper. Configure:

- `joint_state_broadcaster`;
- `arm_controller` using `FollowJointTrajectory`; and
- `gripper_controller` using `GripperCommand`.

Create a project-owned SRDF and MoveIt configuration:

- group `manipulator`: chain `arm_base_link -> gripper_tcp`;
- end-effector group `gripper`;
- named state `stowed` using the fixed joint values above;
- OMPL RRTConnect;
- five-second planning time;
- three planning attempts; and
- velocity and acceleration scaling 0.2.

The planning scene must include the AMR body, nearby station surface, product,
and gripper. On successful Gazebo attachment, remove the product from world
collision objects and add it as an attached collision object with gripper touch
links. Reverse this only after confirmed detachment.

## Contact-gated Gazebo attachment

Do not teleport products. Implement a Gazebo Harmonic system based on the
detachable-joint mechanism that:

- accepts only product IDs 101 through 103;
- rejects an unknown product;
- rejects attachment unless the product is within 30 mm and 0.15 rad of the
  expected grasp transform;
- rejects attachment without recent contact from both fingers;
- creates a fixed joint only after all checks pass;
- publishes confirmed attached/detached state;
- rejects detachment unless the product is within 30 mm of a configured
  dispatch placement pose; and
- never reports attachment before Gazebo confirms the joint exists.

## Task sequences

### Pick

```text
Navigate to pickup approach pose
-> verify the expected station tag
-> navigate to dock pose
-> publish MOVING and verify the base is stationary
-> acquire and validate the expected product tag
-> plan to pre-grasp
-> execute the Cartesian approach
-> close the gripper
-> require bilateral product contact
-> request and confirm attachment
-> attach the MoveIt collision object
-> lift 80 mm
-> move to the stowed-loaded pose
-> publish STOWED_LOADED
```

### Place

```text
Navigate to dispatch approach and dock poses
-> publish MOVING and verify the base is stationary
-> move to the next free pre-place pose
-> lower into the dispatch slot
-> confirm placement pose
-> request and confirm detachment
-> open the gripper
-> update the MoveIt planning scene
-> retreat
-> return to the stowed-empty pose
-> publish STOWED_EMPTY
-> report delivered
```

## Manual and autonomous behavior

Default to `MANUAL`. Accept a mode change only while the queue is empty, no goal
is active, the arm has fresh `STOWED_EMPTY` status, and no product is attached.
The current autonomous registry enables only Product 101 at `pickup_a` and
Product 102 at `pickup_b`; Product 103 remains disabled.

- Manual mode accepts one transport goal and rejects concurrent goals.
- Autonomous mode accepts at most three transport goals, executes them FIFO,
  rejects duplicate pickup/product requests, and reports queue position in
  action feedback.
- Canceling a queued goal removes only that goal.
- Canceling an active goal cancels the current navigation/manipulation action
  and starts no new work. If the robot is not safely empty and stowed, publish
  `FAULT` and keep base motion blocked.
- Failure after pickup preserves `product_attached=true`, publishes `FAULT`,
  keeps base motion blocked, and never reports `delivered=true`.

Provide a terminal helper with these commands:

```text
factory_cli list
factory_cli mode manual
factory_cli mode autonomous
factory_cli send pickup_a dispatch
factory_cli enqueue pickup_a dispatch
factory_cli status
```

## Historical factory runtime plan

Add a separate factory launch that starts the factory Gazebo world, composite
robot, existing base/sensor/control stack, static map server, AMCL, Nav2,
existing navigation mission supervisor, ROS 2 Control, MoveIt, camera adapter,
AprilTag detector, manipulation supervisor, and factory supervisor. Preserve
the existing simulation launch defaults.

Missing system dependencies require explicit approval before installation. The
expected packages are:

```text
ros-humble-kuka-agilus-support
ros-humble-kuka-gazebo
ros-humble-kuka-kr-moveit-config
ros-humble-gz-ros2-control
ros-humble-apriltag-ros
ros-humble-apriltag-msgs
ros-humble-ros-gz-interfaces
```

Under the approved Humble/Harmonic compatibility revision, install the
non-conflicting released packages from this list, omit `ros-humble-kuka-gazebo`
and the Fortress `ros-humble-gz-ros2-control` binary, and use the already
installed Harmonic `ros_gzharmonic` equivalents for ROS-Gazebo integration.

## Historical ordered implementation gates

Do not combine the phase into one unverified edit, and do not proceed past a
failed gate.

1. **Dependency smoke test:** Verify the released `kr6_r900_2` description
   expands and the standalone arm accepts a Gazebo trajectory.
2. **Composite robot:** Validate URDF, unique links/joints, mass range, top
   mounting, collision-free stow, arm/gripper controllers, and unchanged
   base-only behavior.
3. **Interlock:** Prove missing, stale, malformed, deployed, moving, and faulted
   arm status stops base output; prove fresh stowed status permits the existing
   route.
4. **Factory and localization:** Load all assets locally, start AMCL without
   SLAM, plan to every approach/dock pose, and confirm station obstacles appear
   in LiDAR and Nav2 costmaps.
5. **Perception:** Verify synchronized camera topics and correct rejection or
   acceptance of every station/product tag.
6. **Manipulation:** Validate empty motion, then 1 kg, 3 kg, and 5 kg grasp/place
   in that order. Do not advance after a failed mass stage.
7. **Orchestration:** Validate manual transport and autonomous A/B/C FIFO
   transport, cancellation, fault retention, and terminal commands.

## Historical required negative tests

Cover unknown or role-invalid stations, duplicate products, queue overflow,
mode change while busy, unavailable Nav2/MoveIt dependencies, navigation failure
before and after pickup, wrong/stale/inconsistent tags, camera timeout, arm plan
or execution failure, missing bilateral contact, attachment at a distance,
attachment timeout, detachment outside dispatch, stale manipulator status, base
commands while the arm is moving/deployed/faulted, cancellation during both
navigation and manipulation, and preservation of a held product after a
post-pick failure.

## Historical completion evidence

Run focused tests after every gate, then finish with:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
ros2 launch amr_factory factory_autonomous.launch.py \
  headless:=true control_mode:=autonomous factory_attachment:=true
```

The former 1/3/5 kg FIFO requirement is historical. The current accepted
runtime boundary is autonomous Product 101/102 operation with fresh AMCL
localization, registry-derived station/product mapping, confirmed
attachment/detachment, zero base motion whenever the arm is not stowed,
retained-product fault behavior, and placement only in dispatch slots. Product
103/5 kg is disabled and must not be started.

At phase completion, update the relevant project status, session handoff,
architecture, parameter register, beginner guide, changelog, TODO, and Phase 14
test report. Do not create future-phase artifacts. Do not commit or push without
separate user authorization.

Stop and report a blocker instead of improvising if the upstream KUKA model
cannot compose with the AMR, the fixed stow pose collides, dependency versions
differ materially, the 5 kg product violates the total wrist-load constraint,
or the requested Gazebo attachment cannot be proven contact- and pose-gated.

## Phase 14 completion execution plan — 2026-08-28

Implementation is serialized behind the current Gate 6 failure. The first
bounded source correction retains the complete 5 mm release-to-pre-place IK
branch, verifies the measured post-OMPL endpoint, validates every reversed
joint waypoint through `/check_state_validity`, and executes the exact
time-parameterized lower trajectory at the existing 0.2 scaling. No placement,
collision, contact, attachment, or fail-closed acceptance threshold is
relaxed.

After focused checks pass, a separately authorized direct-host runtime must
complete the required performance baseline, empty motion, two consecutive
current-source 1 kg runs, then 3 kg and 5 kg in order, stopping at the first
failure. Gate 7 implementation is intentionally blocked until Gate 6 passes;
it will then reuse one manipulation executor for manual and autonomous FIFO
transport, wire the existing tag validator into pickup/product/dispatch
verification, reconcile attachment from authoritative status, and preserve
held-product fault behavior and cancellation.

The final completion sequence is full workspace build/test, headless factory
demonstration, and evidence updates in this document, the runtime report, and
`SESSION_HANDOFF.md`. Phase 15 SLAM remains out of scope, and no commit or
push is authorized by this plan.

## Bounded CAD-visual corrective revision — 2026-08-24

The active ROS 2 description uses the derived CAD meshes for visuals and
primitive chassis, wheel, caster, and sensor collisions. The untouched export
is preserved. Provisional geometry is explicit: drive radius `0.1128 m`, wheel
separation `0.566 m`, `base_footprint -> base_link` height `0.0478 m`, caster
radius/width `0.0393/0.0421 m`, and base mass `22.15 kg` with positive inertia.
The lower duplicate pedestal and mounting plate are not part of the active
visual; the generic payload remains base-only default-on.

The composite mounts the articulated six-joint KUKA directly from `base_link`
at `xyz="0 0 0.33"`; SRDF adjacency is `base_link <-> arm_base_link`. The
existing frames, camera transform, watchdog, DiffDrive, sensor topics,
controllers, and fail-closed ownership remain unchanged. Gate 6/Gate 7 runtime
acceptance is still pending fresh evidence.

## Historical implementation correction — 2026-08-24

The active description supersedes the earlier primitive-only wording above:
derived CAD meshes are used for chassis, wheel, caster, and LiDAR visuals, with
explicit CAD colors; collisions and dynamics remain conservative primitives.
The retained export geometry excludes the baked arm, mounting plate, and
centered lower pedestal. The composite has no `arm_pedestal_link` and mounts
the articulated `KR6 R900-2` directly to `base_link` at
`xyz="0 0 0.33"`, flush with the AMR top surface. The navigation footprint,
public frames, camera transform, sensor interfaces, watchdogs, and fail-closed
ownership are unchanged.

The motion revision uses direct Humble Regulated Pure Pursuit at a provisional
`0.50 m/s` cruise target with built-in curvature and approach regulation. The
factory experiment uses a `0.0025 s` (`400` steps/s) DART physics step,
real-time factor `1.0`, and shadows disabled. This gives exactly four physics
steps per 100 Hz controller/contact cycle and 40 per 10 Hz lidar/camera cycle.
These are source-level changes only until fresh Gate 6 runtime evidence is
completed; historical 1 ms/MPPI evidence is not reused as proof for this
configuration.

Gate 7 source boundaries are now present: the manipulation action,
factory transport action, operation-mode service, 5 Hz status, FIFO/manual vs
autonomous capacity checks, cancellation/fault retention, and ownership
entries. The supervisors fail closed when their downstream manipulation or
navigation dependencies are unavailable. Runtime orchestration acceptance,
including manual and autonomous transport, remains unclaimed until Gate 6
passes repeatably.

## Historical runtime status — D205 product 101 — 2026-08-24

D205 completed one authorized headless 1 kg product-101 Gate 6 run with median
RTF approximately `0.999818`, aggregate RTF approximately `0.994510`, zero
controller-rate misses, and the exact terminal `GATE 6 1.0 KG COMPLETE 1 KG
PASS`. Gate 6 1 kg is accepted for this run only; no second 1 kg, 3 kg, 5 kg,
or Gate 7 run was started. Gate 7 remains pending, and repeatability/higher
mass evidence is intentionally unclaimed.

## Gate 6 completion plan — detailed implementation and acceptance sequence — 2026-08-28

This is the durable implementation plan for closing Gate 6. It is a plan only;
no runtime pass, Gate 6 completion, or Gate 7 completion is claimed by this
section. The protected `AMR_CODEX_HANDOFF.md` file must not be edited.

### Confirmed failure and constraints

- The latest strict host run passed grasp, attachment, loaded stow, egress,
  transport, alignment, and pre-place OMPL planning, then stopped because the
  final Cartesian lower path completed only 80%.
- The retained-IK correction is source-validated but has not yet been proven
  in a fresh runtime.
- Existing validity requests serialize a complete robot state with
  `is_diff=false`; this can omit the authoritative `held_product` planning
  scene object and falsely validate an unloaded arm.
- Gazebo `DetachableJoint` systems start attached. Gate 6 must use
  `factory_attachment:=true` for native attach/detach evidence, but the shelf
  products must not be dragged before the test starts.
- The historical bag analyzer is absent. A terminal pass line is insufficient
  evidence without reproducible bag and log analysis.
- Runtime evidence must still be collected on a direct host with a readable and
  writable `/dev/dri/renderD*` device; the current host has been checked with
  `/dev/dri/renderD128` and passes that access preflight.

All existing placement, collision, contact, attachment, stow, timing, load,
motion-authority, and fail-closed thresholds remain unchanged.

### Source work package

Before editing, run `git status --short`, preserve every unrelated dirty or
untracked path, and record this plan as the active work item. Use the current
handoff as phase authority. Do not reset, clean, stage, commit, push, or modify
`AMR_CODEX_HANDOFF.md`.

#### Deterministic native-attachment bootstrap

Add `src/amr_factory/scripts/gate6_attachment_bootstrap.py` and install it from
`amr_factory`. Update `factory_localization.launch.py` so the ordinary
`factory_attachment:=false` mode keeps `-r -s <world>` and starts no bootstrap.
For `factory_attachment:=true`, start the Gazebo server with `-r -s <world>`
so dynamically inserted systems can initialize, start the bootstrap before the
delayed robot insertion, and let the bootstrap pause the warm server before
the first robot update. The robot is inserted while paused; the bootstrap
queues detach commands before its first bounded physics step and is the only
component allowed to unpause the world.

When native attachment mode is enabled, hold the controller spawners until a
small READY-gate process observes the bootstrap's reliable transient-local
status. Starting the bootstrap before insertion is required: Gazebo's stock
`DetachableJoint` creates a fixed joint on its first update even when a detach
request is already pending. Inserting while paused lets the bootstrap process
that attach/detach transition before controller activation, avoiding a DART
duplicate-joint update deadlock. The bootstrap reads the native Gazebo
joint-state bridge during this paused window; starting controller activation
while paused can otherwise block Gazebo's update thread and prevent the
controlled unpause from taking effect.

On this Humble/Harmonic installation, `ros_gz_bridge` does not provide a
`ros_gz_interfaces/srv/SetEntityPose` converter even though Gazebo exposes the
native `/world/factory_world/set_pose` endpoint. Keep the bootstrap's existing
ROS service contract and add the bounded
`src/amr_factory/src/gazebo_set_pose_proxy.cpp` adapter for native pose resets.
The adapter is started only in `factory_attachment:=true`, forwards to the
native `gz.msgs.Pose`/`gz.msgs.Boolean` service, accepts model entities only,
rejects non-finite poses and out-of-range IDs, and reports failure on transport
timeout or a false Gazebo reply. Do not add the unsupported raw service to the
`parameter_bridge` argument list.

The same Humble bridge serializes ROS `ControlWorld.run_to_sim_time` even when
the request leaves it at its zero default. The bounded
`src/amr_factory/src/gazebo_control_world_proxy.cpp` therefore owns the ROS
`/world/factory_world/control` service in native attachment mode and forwards
only pause and one-step fields to Gazebo's native transport service. It rejects
reset, seed, run-to-time, and multi-step requests outside the one-step contract.

The bootstrap must:

1. Load product IDs, model names, and expected poses from `products.yaml` and
   `factory.sdf`; reject missing, duplicate, unsupported, or non-finite data.
2. Wait for world-control/set-pose services and native attachment states using
   wall-clock deadlines.  Gazebo's pose publisher does not emit unchanged
   model poses while paused, so product/AMR/joint freshness is validated in
   the bounded live window below rather than by an impossible paused-topic
   freshness requirement.
3. Queue detach commands for products 101, 102, and 103 before the first
   physics step.
4. Process only bounded single steps (`ControlWorld` with `pause=true` and
   `multi_step=1`), republishing detach commands until every native state is
   `detached`.
5. While paused, restore every product to its registered SDF pose with
   `SetEntityPose`.
6. After the paused pose reset, unpause only for the bounded startup
   observation window.  Require fresh product/AMR/joint samples, product
   pose error `<=0.005 m`, yaw error `<=0.01 rad`, AMR displacement `<=0.005
   m`, AMR yaw error `<=0.02 rad`, and all arm joints within the existing
   `0.01 rad` empty-stow tolerance before continuing.
7. Unpause, observe 0.5 simulated seconds, and require the same pose/stow
   limits plus product drift `<=0.005 m`.
8. Latch `READY` on success. On any timeout, rejected service, unexpected
   motion, or tolerance failure, pause again, latch `FAULT`, and never report
   success.

Add the internal interfaces:

- `/amr/simulation/attachment_bootstrap/verify`,
  `std_srvs/srv/Trigger`. It succeeds only when startup passed and all three
  current native states are detached.
- `/amr/simulation/attachment_bootstrap/status`, `std_msgs/msg/String`,
  reliable and transient-local, periodically reporting `STARTING`, `READY`,
  or `FAULT` with the state summary.

Add only the required existing ROS dependencies. If paused detachment still
moves the AMR or arm beyond the existing tolerances, stop and request separate
authority before considering a custom Gazebo plugin.

#### MoveIt payload-aware validity and placement path

In `src/amr_manipulation/src/gate6_mass_stage.cpp`, use one helper for every
loaded-retreat, lower-path, and post-detach validity request:

```cpp
moveit::core::robotStateToRobotStateMsg(
  state, request->robot_state, false);
request->robot_state.is_diff = true;
request->group_name = "manipulator";
```

Immediately before loaded lower validation, query `/get_planning_scene` for
`ROBOT_STATE_ATTACHED_OBJECTS` and require exactly one `held_product` attached
to `gripper_left_finger_link`, with the existing three gripper touch links.
Before post-detach validation, require `held_product` to be absent. Log every
contact pair and the first invalid sample. Add `moveit_core` directly to the
`gate6_mass_stage` target dependencies.

Replace the current diagonal release-to-pre-place continuation with a retained
L-shaped branch:

1. Keep the exact release pose and current high/outward pre-place pose.
2. Create `above_release` with release X/Y/orientation and pre-place Z.
3. Solve and retain exact seeded IK at 5 mm or smaller spacing for
   `release -> above_release`, then `above_release -> pre_place`, without a
   duplicated corner waypoint.
4. Plan OMPL to the retained pre-place joint solution.
5. Execute the retained sequence in reverse:
   `pre_place -> above_release -> release`.

Before execution, require measured pre-place error `<=0.01 rad`. Validate the
measured-current-to-first-point segment and every retained segment by
interpolating `RobotState` at the existing OMPL resolution
`0.001 * manipulator_group->getMaximumExtent()` and calling the payload-aware
validity helper for every sample. Reject non-finite distances, bounds failures,
collision failures, or more than the derived 1,000 samples per segment.

Time-parameterize the exact sequence at the existing `0.2/0.2` scaling. Before
`arm.execute()`, require the six expected joint names, unchanged point count,
six finite positions/velocities/accelerations per point, finite strictly
increasing timestamps after the first, and a release endpoint error `<=1e-9`.
Reacquire current joints immediately before execution and again require the
existing `0.01 rad` start tolerance. After execution and before detach, require
release-joint error `<=0.01 rad`, slot error `<=0.030 m`, native attachment
state `attached`, and attachment error `<=0.030 m` / `<=0.15 rad`.

Remove `request_and_confirm_initial_detachment()` and call the bootstrap
Trigger before any gripper or arm command. In `gate6_product_test.py`, verify
the same Trigger, remove blind `_detach_all()` preparation, preserve the
paused selected-product reset, AMR pose preservation, navigation, and FAULT
behavior, and keep the runner limited to products 102 and 103.

#### Reproducible evidence analyzer

Add and install `src/amr_manipulation/scripts/gate6_evidence_analyzer.py`:

```bash
ros2 run amr_manipulation gate6_evidence_analyzer \
  --bag <bag-directory> \
  --product-id <101|102|103> \
  --output <analysis.txt>
```

Derive model, mass, and dispatch slot from the existing registry. Scope data to
the selected mass-stage `source_boot_id`. Return nonzero unless the bag proves
READY bootstrap status, required topics, in-stage
`detached -> attached -> detached`, bilateral product contact, retained loaded
status, valid empty-stowed final status, slot error `<=0.030 m`, matching
`/amr/control/cmd_vel` to `/amr/simulation/base/cmd_vel`, and no base motion
while motion is forbidden. Zero messages on the retired precise-navigation
status topic are acceptable; active normal-navigation status is required.

The analyzer must emit exactly:

```text
GATE6_BAG_ANALYSIS=PASS product_id=<id>
```

Planning-scene proof and exact lower-trajectory checks remain structured stage
log markers and must be checked alongside the bag result.

### Focused validation before runtime

Add contract/unit coverage for bootstrap mode selection, all bootstrap failure
states, Trigger ordering, product-runner preparation, `is_diff=true` validity
requests, attached-object proof, L-shaped ordering and spacing, interpolated
validation, trajectory postconditions, analyzer pass/fail behavior, and
unchanged detach/retreat/stow ordering.

Run:

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/amr_factory/test src/amr_manipulation/test
colcon build --packages-select amr_factory amr_manipulation --symlink-install
source install/setup.bash
colcon test --packages-select amr_factory amr_manipulation
colcon test-result --test-result-base build/amr_factory --verbose
colcon test-result --test-result-base build/amr_manipulation --verbose
git diff --check -- src/amr_factory src/amr_manipulation \
  docs/PHASE_14_FACTORY_MOBILE_MANIPULATION.md
colcon build --symlink-install --executor parallel --parallel-workers 4
colcon test --executor parallel --parallel-workers 4
colcon test-result --verbose
```

Require zero build errors and zero test failures. Inspect the combined diff and
stop if any safety, ownership, collision, load, timing, attachment, or
fail-closed rule was weakened. Runtime requires separate direct-host
authorization after these checks.

### Direct-host acceptance order

Use a unique `GZ_PARTITION`, `ROS_DOMAIN_ID`, and `ROS_LOG_DIR` per session.
Every session must use strict headless hardware rendering, no RViz or factory
supervisor, host/runtime preflight, at least 10 RTF samples, median and
aggregate RTF `>=0.90`, active lifecycle/controllers, exactly one declared
`/amr/control/cmd_vel` publisher, MoveIt readiness, source/executable hashes,
and a recorder started before the stage. Stop at the first failed gate and
preserve logs/bags; never retry or tune during an acceptance chain.

1. **Empty motion:** fresh `factory_attachment:=false` session; require exit 0
   and `GATE 6 EMPTY MOTION: PASS`; clean shutdown and process scan.
2. **1 kg pass 1:** fresh `factory_attachment:=true` session; require bootstrap
   READY/Trigger success, analyzer success, and
   `GATE 6 1.0 KG COMPLETE 1 KG PASS`; clean shutdown.
3. **1 kg pass 2:** another fresh strict true-attachment session; require the
   same complete result.
4. **3 kg:** after the second 1 kg pass, keep factory and MoveIt alive, prepare
   product 102, and require `GATE6 PRODUCT PREP PASS product_id=102`, analyzer
   success, and `GATE 6 3.0 KG COMPLETE 3 KG PASS`.
5. **5 kg:** only after 3 kg passes, prepare product 103 and require
   `GATE6 PRODUCT PREP PASS product_id=103`, analyzer success, and
   `GATE 6 5.0 KG COMPLETE 5 KG PASS`.

The final sequential session must leave all products detached in their assigned
dispatch slots and the arm empty-stowed. Shut down stage/runner, recorder,
MoveIt, and factory in that order, then save the exact process scan.

Any failed 1 kg run resets the repeatability claim. Any shared source,
attachment, factory, MoveIt, navigation, or motion change after a pass resets
the affected current-source acceptance chain and requires fresh validation and
two 1 kg passes. Do not start 3 kg before both 1 kg passes, 5 kg before 3 kg,
or Gate 7 before all Gate 6 evidence passes.

### Completion documentation and boundaries

After all acceptance gates pass, update this document, `SESSION_HANDOFF.md`,
`docs/PHASE_14_GATE6_RUNTIME_DEBUG_REPORT.md`, `docs/SIMULATION_COMMANDS.md`,
`PROJECT_STATUS.md`, `TODO.md`, and `CHANGELOG.md` where Gate 6 status is
recorded. Correct stale commands that say the 1 kg run should not be repeated
or that use `factory_attachment:=false` for native product acceptance.

Phase 15 SLAM, Gate 7 runtime, GUI acceptance, hardware claims, dependency
installation, commit, and push remain outside this Gate 6 plan.

## Autonomous 1 kg / 3 kg factory cycle — approved 2026-09-04

This section records the approved implementation handoff for station-selectable
autonomous factory operation. Source implementation is authorized for the
scoped `amr_interfaces`, `amr_factory`, `amr_manipulation`, and required
`amr_bringup` ownership/test files only. Runtime/Gazebo/MoveIt/product
execution remains a separate, explicitly authorized validation activity; this
implementation pass must not run it. Product 103 remains disabled for
autonomous operation. `AMR_CODEX_HANDOFF.md` is protected and must not be
modified.

### Intended behavior

- Execute one complete cycle from any valid localized, collision-free AMR pose:
  navigate to the selected pickup station, pick the station's registered
  product, navigate to dispatch, place it in the registered slot, empty-stow,
  and schedule the next job only after fresh detached/empty-stow proof.
- Support one-shot `pickup_a`/`pickup_b` selection, finite ordered sequences,
  and a continuous sequence such as `pickup_a, pickup_b, pickup_a, pickup_b`.
- Support graceful stop (finish the active delivery, then optionally return
  home), immediate cooperative cancel (clear pending work and do not add home
  movement), and idle `go home`. Home is allowed only from a fresh safe
  empty-stowed state.
- Product/station/dispatch mappings come from the YAML registries. Products 101
  (1 kg) and 102 (3 kg) are autonomous-enabled; product 103 (5 kg) is not.

### Public contracts

- Add `ExecuteProductCycle.action` at
  `/amr/manipulation/execute_product_cycle` with pickup and destination station
  IDs, typed terminal outcomes, delivered/product/message result fields, and
  preparing/executing/canceling feedback including attachment state.
- Add `RunSequence.action` at `/amr/factory/run_sequence` with ordered pickup
  IDs, finite/continuous cycle count, optional final station, cycle/job result
  counts, and per-cycle phase/product/attachment feedback.
- Add `NavigateStation.action` at `/amr/factory/navigate_station`; v1 accepts
  only a registry station with the `home` role.
- Add `/amr/factory/stop_sequence`, `/amr/factory/cancel_sequence`, and the
  private `/amr/manipulation/internal/cancel_cycle_motion` Trigger service.
- Append sequence state, counters, final station, graceful-stop request, and
  fault-latched fields to `FactoryStatus`; retain one canonical manipulation
  status publisher at `/amr/manipulation/status`.

### Implementation and safety rules

- The autonomous manipulation cycle adapter owns the canonical manipulation
  status and the new cycle action. It runs the generalized Gate 6 product
  preparation/mass-stage child with internal status remapping; legacy C++ and
  autonomous supervisors must not run concurrently.
- Generalize Gate 6 preparation for products 101/102, arbitrary valid starting
  poses, selected-only product reset, registered other-dock retreat, and
  configurable internal status without changing accepted motion geometry,
  thresholds, slots, or safety gates.
- Cancellation must propagate cooperatively through preparation, Nav2,
  MoveIt, gripper, and mass-stage work. If detached/empty-stowed state cannot
  be proven after cancellation or failure, latch `FAULT`, preserve the held
  product, and block base motion; never retry or advance automatically.
- The factory supervisor uses the cycle action for one-shot transport and
  sequences, enforces one active command owner and one cycle at a time, rejects
  unsupported/disabled stations or concurrent requests, and updates terminal
  outcomes on every path.
- Add the autonomous launch and exact CLI controls for `send`, `loop`, `stop`,
  `cancel`, `go home`, and `status`, while excluding standalone Gate 6 runners
  and the legacy manipulation supervisor from autonomous launch.

### Validation boundary

Validate interfaces and source contracts first, then build/test the affected
packages and workspace. Do not claim factory runtime acceptance from source
tests. Stop patching after two failed implementation attempts for one root
cause, return evidence for Astra/medium re-diagnosis (high only if needed), and preserve this record and
the protected handoff file.

### Token-conscious execution revision — approved 2026-09-07

`SESSION_HANDOFF.md` retains authority over the paused partial implementation.
Use Astra/medium first for analysis, planning, and diagnosis; escalate to
Astra/high only when evidence or design remains unresolved at medium. The
initial complete-diff review and focused source validation below are already
complete; do not repeat them merely because the model assignment changed.
Convert supported diagnoses into bounded packets with exact
allowed changes, causal evidence, invariants, behavioral tests, expected
results, and stop conditions. Resolve state transitions, cancellation ownership
and terminal proof, status freshness, and next-job/home permission explicitly.

After the user approves a concrete packet, Luna/max implements it and runs its
focused checks. Astra/medium independently reviews every packet by default;
escalate review to Astra/high only when medium cannot resolve a material
contradiction, high-risk state transition, or contract decision. Astra
implementation is reserved for fixes whose diagnosis shows they cannot safely
be separated from difficult cross-component reasoning.
Keep one writer and an independent reviewer, and return contradictory evidence
or failed implementation to diagnosis before further source changes.

Pass concise packets rather than full conversation histories, reuse findings
until invalidated, and reserve broader builds/tests for integration milestones.
No default parallel agents or Astra/max runs. Select assigned models explicitly;
written roles do not switch the active model. Model changes do not reset the
existing attempt limits. Runtime acceptance still requires separate approval.

### Read-only autonomous review — 2026-09-07

**Verdict: FAIL for implementation acceptance; source/build evidence is partial.**
Reviewed the complete changed interface, registry, CLI, factory supervisor,
cycle adapter, Gate 6 runner/mass-stage, and build/install surface against HEAD
`abf3dbc67a9fa8fa123003e1d0185f61344b0232` and the approved September 4 behavior.
Traced the relevant existing launch, ownership, bootstrap, mission, and Humble
library contracts. No production source or repository tests were changed.

Raw evidence is in `/tmp/amr-autonomous-review-20260907-ZSfLdb/` (temporary
storage; retain it before host cleanup). `offline_review.py` executes real
adapter methods and compiles the unmodified C++ `sequence_loop` against scripted
dependency seams. `boundary_probes.py` checks installed Humble APIs and the pure
action state machine. Neither initializes ROS, creates ROS nodes, runs launch,
or starts Gazebo/MoveIt. These are deterministic source reproductions, not
timing measurements or runtime acceptance.

| ID | Finding and mechanism | Evidence / confidence |
| --- | --- | --- |
| A1 | Adapter cannot start normally: its line 56 redeclares `use_sim_time`, which Humble's Node TimeSource already declares. Its three `[]` mapping defaults infer BYTE_ARRAY, rejecting intended string/integer array overrides. | `boundary_probes.log` B1/B2; installed Node/TimeSource; HIGH, SOURCE |
| A2 | Adapter terminal calls pass a result to Python `succeed/abort/canceled`, which accept only `self`. Three logger calls also use unsupported positional formatting arguments, so error reporting can raise another TypeError. | `offline_review_v2.log` R1; adapter lines 105, 252, 503–622; HIGH, SOURCE |
| A3 | Once idle empty-stow is established, `_publish_status` no longer gates base permission on current joint/bootstrap proof. Expired proof still produces fresh status with `base_motion_allowed=true`. `_joint_stowed_locked` also accepts mismatched name/position arrays through `zip`. | R2/R4; adapter lines 273–297, 361–397; HIGH, SOURCE |
| A4 | `_internal_status_callback` can overwrite state, permission, and held-product evidence after `_set_fault` latches a fault. A later empty-stow child sample permits base motion even while `_fault_latched` remains true. | R3, with a faulted-state negative control; adapter lines 305–359, 470–477; HIGH, SOURCE |
| A5 | `sequence_loop` permits final home when a job failed but no attachment fault was latched; failed empty-stow proof with `held_product_=false` also reaches home. `navigate_home` itself does not recheck empty-stow proof before sending. | R5/R6; factory lines 776–800, 840, 934–951; HIGH, SOURCE |
| A6 | Factory cancellation handling tests `call.canceled` before the typed failure result. An unconfirmed cancellation reported as INTERLOCK_FAILED becomes ordinary CANCELED without a fault latch. The Trigger service only sets flags; the final `goal->canceled` can be called without the public goal entering CANCELING. | R7 and B3; factory lines 164–185, 635–640, 716–723, 770–773, 887; HIGH, SOURCE |
| A7 | Required autonomous launch, factory autonomous contract tests, cycle-adapter tests, and new action/service ownership entries are absent. The existing demo starts the legacy manipulation server, while the modified factory now calls ExecuteProductCycle. | B4; factory_demo.launch.py and interface_ownership.yaml inspection; HIGH, ORCHESTRATION / TEST COVERAGE |
| A8 | Existing gripper contract asserts literal `result.wait_for(30s)`; source now uses a 30-second steady-clock deadline and 50 ms polling. The old text assertion fails without testing the cancellation behavior. | `pytest.log`: test_moveit_config.py:149; HIGH, TEST/EVIDENCE HARNESS |

Additional source-traced risks remain outside the first implementation packet:

- Pending goal acceptance is not retained for late cancellation in factory
  `call_execute_cycle`/`navigate_home` or runner `_navigate`. Cancellation during
  acceptance can release the caller while the downstream request remains live.
  Existing `amr_mission` goal-response callbacks show a local pattern for
  retaining and canceling late accepted handles. Timing incidence is unmeasured.
- The preparation node destroys its cancel service before launching the mass
  stage, leaving a handoff gap. The adapter accepts any changed internal boot
  identity and has no explicit runner-to-stage ownership handshake. A fresh
  proof wait can also outlast the stopped child's 200 ms status freshness window.
- Mass-stage attachment/detachment helpers have no cancellation checks around
  their requests, and several planning/contact waits are not interruptible.
  A MoveIt stop hook and a process exit alone do not prove every downstream
  command reached a terminal state. SIGINT cleanup is not that proof.
- Python registry loading uses `yaml.safe_load` without duplicate-key rejection;
  consumers independently parse registry data. CLI station choices remain
  hard-coded, and one-shot admission does not check `autonomous_enabled` for
  Product 101/102. Preserve fixed hardware values while reconciling validation.
- Factory feedback does not subscribe to cycle feedback; only boundary feedback
  is sent. Status and action outcome consistency needs behavioral coverage.
- The original three-goal FIFO text and the newer rejection of concurrent
  commands must be reconciled explicitly in compatibility tests. Do not silently
  restore parallel admission or remove a public interface.

These are source-review findings, not a claim that every possible defect was
enumerated. Future packets require a reproducible mechanism and explicit
decisions; no broad "finish everything" packet is ready.

Fresh commands and results (run from the workspace with Humble sourced):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_factory/test src/amr_manipulation/test src/amr_bringup/test
# Exit 1: 96 passed, 1 failed (A8).
source install/setup.bash
export GZ_VERSION=harmonic
colcon build --packages-select amr_interfaces amr_factory amr_manipulation \
  --symlink-install --executor sequential --parallel-workers 2
# Exit 0: all three packages built; existing-overlay warning recorded.
ctest --test-dir build/amr_manipulation -L gtest --output-on-failure
# Exit 0: both existing gtest suites passed.
PYTHONDONTWRITEBYTECODE=1 python3 /tmp/amr-autonomous-review-20260907-ZSfLdb/offline_review.py
# Exit 0: seven undesired behaviors/API violations reproduced, healthy controls passed.
PYTHONDONTWRITEBYTECODE=1 python3 /tmp/amr-autonomous-review-20260907-ZSfLdb/boundary_probes.py
# Exit 0: constructor/type/state-machine incompatibilities and missing artifacts confirmed.
git diff --check
# Exit 0 at review time.
```

The five changed/new Python files passed AST parsing; the three package
manifests parsed successfully. `syntax_manifest.log`, `build.log`,
`ctest_gtest.log`, and `source_sha256.log` preserve results and source identities.
The first offline harness run reproduced R1–R4 but omitted one C++ stub member
(`detail_`); adding that member to the temporary harness enabled R5–R7.
`offline_review.log` preserves that harness failure. No product patch or new
implementation attempt occurred. The findings survive the passing build and
old tests because those checks do not exercise the new adapter/state machine.
Full workspace tests and runtime were not advanced after the focused failure.

### Packet A1 — Humble adapter API compatibility — approved; review passed

**Owner:** Luna/max implementation, Sol/high independent review. This is a
bounded compatibility correction for A1/A2 only, not safety/integration
acceptance. Confidence HIGH. The user approved proceeding. An initial Luna/max
usage-limit pause consumed no implementation attempt. After resumption, Luna
completed attempt 1: guarded clock declaration, typed mapping parameters,
compatible terminal/logging calls, and the registered offline test suite.
Luna reports pre-change 8 failed/1 passed and post-change 16 passed, with the
package build, registered CTest, and diff check passing. Sol/high independently
reviewed the exact dirty-source delta and issued PASS; the pre-packet replay was
red as expected. Pre-packet source snapshots: `/tmp/amr-a1-pre-XtoZw5/`.
The historical broader implementation cap remains recorded.

**Allowed files:**

- `src/amr_manipulation/scripts/cycle_manipulation_supervisor.py`
- `src/amr_manipulation/test/test_cycle_adapter.py` (new)
- `src/amr_manipulation/CMakeLists.txt` (register this test only)

**Exact changes:**

1. Guard `use_sim_time` declaration with `has_parameter`, preserving an existing
   launch override. Do not force or overwrite the selected clock setting.
2. Declare `product_station_map` and `dispatch_station_ids` using
   `rclpy.parameter.Parameter.Type.STRING_ARRAY`; declare
   `autonomous_product_ids` using `INTEGER_ARRAY`. Read these three with
   `get_parameter_or(name).value` so absent overrides yield the existing
   MappingError/unavailable path rather than ParameterUninitializedException.
   Keep the existing nonempty mapping requirements and Product 103 exclusion.
3. Call Python `goal_handle.succeed()`, `abort()`, and `canceled()` with no
   result argument. Return the existing typed result from `_execute_callback`.
   Preserve result values, messages, branch ordering, and ownership cleanup.
4. Format the three logger messages before calling `error`, using one message
   argument. Preserve their diagnostic content.
5. Add a registered offline pytest suite that exercises these behaviors using
   installed Humble method signatures and parameter validation. Mock only ROS
   graph/process infrastructure; do not mock away the incompatible signatures
   or descriptor checks. No ROS initialization, processes, or simulation.

**Behavioral tests and prediction:** Existing true/false clock overrides survive;
valid string/integer mappings pass declaration and parsing; omitted mappings
remain unavailable and malformed/wrong-type/103 mappings are rejected. Exercise
success, abort, and cancellation with signature-faithful goal handles and
assert the returned typed result. Exercise the three logging failure paths.
These tests must expose the current API defects before changes and pass after
the correction. The known A8 test remains a separately recorded failure;
do not change it under this packet or report the complete suite as passing.

**Validation:** From the workspace, source `/opt/ros/humble/setup.bash` and
`install/setup.bash`; set `GZ_VERSION=harmonic`. Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_manipulation/test/test_cycle_adapter.py
colcon build --packages-select amr_manipulation --symlink-install
ctest --test-dir build/amr_manipulation -R '^cycle_adapter_test$' --output-on-failure
git diff --check
```

Register the test as `cycle_adapter_test` inside the existing BUILD_TESTING
block. Independently inspect the packet diff against the pre-packet dirty
source, not only HEAD. Preserve the entire existing worktree and protected
handoff. Do not change status/fault logic, timers, geometry, launch behavior,
other tests, public interfaces, or any runtime state. Stop on a contradiction,
unexpected failure, or need to touch another file; return evidence for diagnosis.

After A1 review, prepare separate packets for A3/A4 status authority, A5/A6
factory terminal decisions, cancellation ownership across child boundaries,
and launch/registry/ownership/test integration. Start diagnosis and review with
Astra/medium, escalating to high only if medium cannot resolve a material
contradiction or contract decision. Do not activate these later packets based
on A1 approval.

### Packet A3/A4 — status authority and fault retention — implemented; review passed

**Owner:** Luna/max implementation, Astra/high independent review. This packet
is a bounded safety correction for the adapter only. Diagnosis was Astra/medium,
confidence HIGH, SOURCE. The user approved it. Luna completed implementation
attempt 1, then a bounded correction attempt 2 after Astra/high found a terminal
window gap. Astra/high re-review now passes the corrected delta.

**Allowed files:**

- `src/amr_manipulation/scripts/cycle_manipulation_supervisor.py`
- `src/amr_manipulation/test/test_cycle_adapter.py`

**Evidence and mechanism:** The targeted replay against the post-A1 source
still reproduces R2/R3/R4 from the read-only review. `_joint_stowed_locked`
truncates mismatched arrays with `zip` and accepts duplicate names. Idle
`_publish_status` only checks independent bootstrap/joint proof during the
initial STARTING transition, so expired proof can continue publishing
`base_motion_allowed=true`. `_internal_status_callback` accepts unowned or late
child samples and can overwrite a latched fault. `_set_idle_from_proof`
unconditionally clears latch and retention evidence. Additional deterministic
probes recorded `DUPLICATE_PROOF_ACCEPTED True`,
`UNOWNED_IDLE_CHILD_BASE_ALLOWED True`, and `IDLE_HELPER_CLEARS_LATCH True`.

**Exact changes:**

1. Reject unequal joint-name/position lengths and duplicate names before
   building the joint map. Preserve the six required joints, finite checks,
   0.01-rad tolerance, 0.2-second freshness threshold, and acceptance of extra
   unique joints.
2. Factor a side-effect-free locked independent-proof predicate and use it for
   `_fresh_independent_empty_stow`, startup transition, and every idle status
   publication. Idle permission additionally requires no reserved goal, no
   child, `STOWED_EMPTY`, and no attachment. Expired proof denies permission;
   fresh proof can restore it. A fault latch always publishes FAULT, invalid,
   and motion denied. Preserve active-cycle child freshness/state checks.
3. Add a narrowly scoped child-status authority flag. Accept child samples only
   for an active goal while authority is open and no fault is latched. Close it
   atomically in fault, successful idle transition, and final cleanup. This is
   not a runner/stage identity handshake.
4. Make `_set_idle_from_proof` return a boolean. A latched fault never clears;
   current independent proof and no canonical/child attachment are required.
   Failed checks latch FAULT with retained evidence. Update all three terminal
   callers to honor refusal and return `RETAINED_PRODUCT_FAULT` with
   `delivered=false`; success aborts rather than succeeds when idle proof is
   refused.

**Invariants and non-goals:** Preserve status constants, freshness thresholds,
hardware geometry, public interfaces, Product 103 exclusion, child-success and
cooperative-cancel proof requirements, and A1 behavior. Do not change bootstrap
request lifetime, timers, child handoff authentication, cancellation handshake,
downstream terminal proof, factory logic, registries, launch, or runtime behavior.

**Prediction and tests:** Fresh idle proof permits motion; expiry, unstowed,
mismatched, duplicate, missing, or nonfinite joint evidence denies it, while a
fresh valid pair restores it. Unowned or late samples cannot grant motion or
overwrite FAULT/attachment/product/detail. A late sample cannot reacquire
authority after a successful terminal transition. Terminal idle refusal cannot
produce safe SUCCESS/CANCELED. An active owned fresh `STOWED_LOADED` sample
remains a positive control; stale/inconsistent samples deny motion.

Extend the existing offline fixture with mocked monotonic time and run. The
initial implementation red baseline had 15 failures/20 passes; the corrected
suite has 36 passes. The correction's focused regression was red before the
one-condition source change and green after it. Snapshots and logs are under
`/tmp/amr-a3a4-pre-ThWSze/` and `/tmp/amr-a3a4-attempt1-5yoFti/`.

```bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/amr_manipulation/test/test_cycle_adapter.py
git diff --check
```

Demonstrate the new regression failures on a pre-packet snapshot before the
fix, then green post-fix. Astra/high independently verified the complete
corrected delta, including the real helper-to-cleanup terminal window, and
`git diff --check` passed. Stop on contradictory state-model evidence, any need
to touch another path, or the 95%-used five-hour allowance. Runtime remains a
separate gate.

### Packet A5/A6 — factory terminal and cancellation semantics — attempt 2 acceptance withheld

**Diagnosis:** Astra/medium, HIGH confidence, SOURCE. Reused R5/R6/R7 and
current factory tracing. Attempt 1 implemented the approved bounded scope but
was not accepted after review.
`sequence_loop` can home after failed jobs or missing empty-stow proof when no
product is marked held. `call_execute_cycle` can return `INTERLOCK_FAILED` while
callers prioritize a cancellation flag. The Trigger service does not establish
the public action’s `CANCELING` state, and Humble transitions that state only
after the cancel callback returns. `navigate_home` lacks a final interlock and
fresh-proof check immediately before dispatch.

**Allowed files:** `src/amr_factory/src/factory_supervisor_node.cpp`, new
`src/amr_factory/test/test_factory_autonomous_contract.py`, and the one CMake
registration for that test.

**Proposed bounded changes:** Preserve typed failures before generic cancellation;
require fresh safe empty-stow proof for ordinary safe cancellation/home; prohibit
home after failed jobs; recheck fault/cancel/stopping/readiness immediately before
home send; make fault latches monotonic; and call `canceled()` only when
`goal->is_canceling()`, otherwise abort with a typed CANCELED result. Critical
interlock/retention failures always abort with their failure outcome.

**Required offline evidence:** R5/R6/R7 regressions, proof expiry before home,
ordinary versus Trigger cancellation, actual CANCELING versus EXECUTING action
states, and fault-latch preservation. Attempt 1 had 4/5 red before
implementation and 5/5 focused tests after. Astra/medium found three misses:
missing-proof sequence failure does not always latch factory fault; standalone
home interlock/cancel-confirmation failure does not latch fault; and the final
home check unlocks before `async_send_goal`. Correction attempt 2 was paused at
94% five-hour usage before correction edits/tests, then resumed on 2026-09-08.
The three named misses were corrected and focused-tested; the independent
review below still withholds packet acceptance. Snapshots/logs are under
`/tmp/amr-a5a6-pre-vTNTrK/` and `/tmp/amr-a5a6-attempt1-W8d2tO/`.
Preserve station geometry, timing, ownership, held-product evidence, and all
non-goals from the handoff. No runtime, launch, registry, downstream
cancellation handshake, or late-goal ownership changes.

### A5/A6 independent review and escalation — 2026-09-08

**Verdict:** Astra/medium confirms Luna followed the three exact correction
instructions, but complete A5/A6 specification acceptance FAILS. The factory
source has an uncovered early cancellation path; the lock probe has an invalid
oracle. No third autonomous implementation attempt is authorized by this record.

**Prediction versus result:** Attempt 2 should establish monotonic fault latches
for missing delivery proof and standalone home failure, and hold the final home
check through dispatch. Source review confirms these three changes. The broader
requirement that ordinary cancellation requires fresh empty-stow proof remains
violated by `sequence_loop`'s initial/between-job cancel exit. Diagnosis supported;
implementation and evidence incomplete. No causal hypothesis was falsified by
this review, and the attempt count is not reset.

**Confirmed source mechanism (HIGH, SOURCE):** Admission checks readiness, then
starts a separate sequence worker. Status can change or exceed the existing
200 ms freshness window before worker execution or between jobs. Trigger/action
cancel sets `sequence_cancel_requested_`; the branch at source line 764 sets
`canceled=true` and exits without the proof check used after a cycle. Terminal
selection at line 842 prioritizes that flag, returning CANCELED without a new
fault latch. A pre-existing latch is retained but does not change the result.
This proves a terminal-contract failure; it does not prove physical motion or
measure the scheduling window's runtime incidence.

**Confirmed harness defect (HIGH confidence, TEST/EVIDENCE HARNESS):** Test line
366 calls `try_lock()` on the same nonrecursive mutex already held by that
thread. This is undefined behavior. The passing probe cannot prove the final
check/dispatch lock scope, although the current source visibly holds that lock.

**Fresh evidence:** Factory pytest 70 passed; registered autonomous CTest 1/1
passed; sourced Humble/overlay `colcon build --packages-select amr_factory
--symlink-install --executor sequential --parallel-workers 2` passed; diff check
passed. Independent production-body extraction with a replacement harness main
gave 2 safe controls passing and 6 unsafe/prior-latch cases failing. Both
EXECUTING and CANCELING handles were covered; all cases sent zero home goals.
The prior-latch cases are injected invariant checks. Runnable command:
`PYTHONDONTWRITEBYTECODE=1 python3 /tmp/amr-a5a6-review-20260908-8zz7VJ/early_cancel_probe.py`
(expected current exit 1). Source hashes and result details are saved beside it.

**Proposed next packet — user decision required:**

- Objective: close the remaining early-cancel terminal seam and replace the
  invalid lock oracle. Candidate write scope is only
  `src/amr_factory/src/factory_supervisor_node.cpp` and
  `src/amr_factory/test/test_factory_autonomous_contract.py`; no CMake change.
- Snapshot the cancel decision under the mutex, then perform the existing
  bounded proof wait outside that mutex. Fresh proof with no factory fault may
  produce ordinary CANCELED; failed proof or a factory fault must latch/retain
  fault, produce INTERLOCK_FAILED, and abort. Recheck fault under the mutex at
  terminal selection. Preserve held-product evidence and dispatch no next job
  or home. Reuse existing proof timing; do not add arbitrary waits or alter thresholds.
- Replace the same-thread mutex `try_lock()` oracle with defined deterministic
  lock-ownership instrumentation in the offline seam. Demonstrate that it fails
  on the saved pre-correction unlock-before-send source and passes on current
  source; avoid scheduling sleeps and same-thread nonrecursive lock attempts.
- Behavioral checks: early cancel before the first job and between jobs;
  missing/expired proof, fresh safe proof, and prior fault; EXECUTING typed-cancel
  abort versus CANCELING canceled; unsafe paths always INTERLOCK_FAILED abort;
  no added child/home dispatch; unchanged job counters and attachment evidence.
- Validate red-before/green-after focused autonomous pytest, full factory
  pytest, focused factory build, registered autonomous CTest, complete diff,
  and `git diff --check`, followed by separate Astra/medium review. Escalate to
  Astra/high only for a material unresolved transition/contract contradiction.
- Preserve public interfaces, station/hardware geometry, safety gates, ownership,
  timing values, Product 103 exclusion, and unrelated dirty work. Runtime,
  launch/registry integration, downstream cancellation, and late goal ownership
  remain outside this packet.
- Stop if source contradicts this diagnosis, another file is needed, an invariant
  changes, focused validation fails, or the usage guard is reached. A failed
  implementation returns to diagnosis. The user must explicitly resolve the
  exhausted attempt limit before this proposal becomes implementation authority.

**Current worktree:** All source/config/test changes and untracked files listed
in `SESSION_HANDOFF.md` predate this review and are preserved. This review only
updates that handoff and this plan; temporary probes are outside the worktree.
No runtime, commit, push, installation, system change, or external change occurred.
