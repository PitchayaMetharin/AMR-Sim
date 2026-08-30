# Phase 14 — Factory Mobile Manipulation Implementation Plan

## Authority and scope

This is the decision-complete implementation handoff for a future approved
Phase 14. It does not itself authorize implementation, dependency installation,
a commit, or a push. Before editing, read `AGENTS.md`, `SESSION_HANDOFF.md`, and
`PROJECT_STATUS.md`, then run `git status --short` and preserve every existing
worktree change. Never modify, stage, discard, normalize, or commit
`AMR_CODEX_HANDOFF.md`.

The result remains a one-laptop ROS 2 Humble/Gazebo Harmonic simulation. The
AMR's intended maximum payload is 300 kg, but the repository currently validates
only its lower simulation baseline; Phase 14 must not claim that a physical AMR,
mount, or manipulator is safe or rated for 300 kg. The KUKA arm may manipulate
products of at most 5.0 kg in this simulation.

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

## Factory runtime

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

## Ordered implementation gates

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

## Required negative tests

Cover unknown or role-invalid stations, duplicate products, queue overflow,
mode change while busy, unavailable Nav2/MoveIt dependencies, navigation failure
before and after pickup, wrong/stale/inconsistent tags, camera timeout, arm plan
or execution failure, missing bilateral contact, attachment at a distance,
attachment timeout, detachment outside dispatch, stale manipulator status, base
commands while the arm is moving/deployed/faulted, cancellation during both
navigation and manipulation, and preservation of a held product after a
post-pick failure.

## Completion evidence

Run focused tests after every gate, then finish with:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
ros2 launch amr_factory factory_demo.launch.py headless:=true
```

The headless runtime must demonstrate manual 1 kg transport and autonomous
1/3/5 kg FIFO transport, fresh AMCL localization, correct tag verification,
confirmed attachment/detachment, zero base motion whenever the arm is not
stowed, retained products during transport, and placement only in dispatch
slots. Confirm that no additional base-velocity publisher exists.

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

## Current implementation correction — 2026-08-24

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

## Current runtime status — D205 product 101 — 2026-08-24

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
