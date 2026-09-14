# AMR Project Handoff for Codex

## Role
You are the Lead Robotics Engineer responsible for completing this industrial AMR project. Do not teach unless asked.

## Current repository state — 2026-09-14

The repository is currently a laptop-only ROS 2 Humble/Gazebo Harmonic
simulation and contains 17 ROS 2 packages. The active navigation controller is
Nav2 Regulated Pure Pursuit (RPP); the compatibility package and topic names
remain `amr_mpc_controller` and `/amr/mpc/cmd_vel`.

Phase 14 autonomous factory-cycle runtime acceptance is complete for Product
101 (1 kg) and Product 102 (3 kg). Product 103 (5 kg), Gate 7, physical
hardware, and functional-safety claims are outside the accepted scope.

Phase 15 packets P0 through P8, the factory mapping CLI packet, and the
cycle-adapter source/offline packets are complete and accepted at the
source/offline boundary. The current reviewed mapping support includes:

- a named `phase15_mapping` runtime profile with inclusive median and aggregate
  RTF floors of `0.80`; the default profile remains `0.90`;
- direct Nav2 map saving from absolute topic `/map` with transient-local
  subscription QoS, followed only on success by the live
  `/amr/slam_toolbox/serialize_map` service; and
- a required two-file pose-graph bundle, `<prefix>.posegraph` plus
  `<prefix>.data`, with manifest, terminal-status, runtime-report,
  quality-review, and promotion gates that remain fail closed.

The original future-dated `map->odom` blocker was corrected in an isolated
upstream SLAM Toolbox 2.6.10 overlay at
`/tmp/amr_slam_toolbox_git.vlhnFQ/install/` by publishing the transform at the
scan timestamp instead of adding the transform lookup timeout. That overlay is
runtime evidence, not a durable checked-in workspace dependency.

The latest no-recorder diagnostic runtime,
`phase15_tf_diag_20260914_01`, passed host preflight, Gate 6, all staged mapping
readiness gates, 30-second stabilization, and the `phase15_mapping` RTF gate
(aggregate `0.9990677485`, median `1.0000051000`). Exploration then completed
one navigation goal, discarded two later plans because the carried TF became
stale during planning, and latched `FAULT` at ROS time `173.183316015`.
Contemporaneous raw TF showed `map->odom` age `0.080 s` and
`odom->base_footprint` age `0.013 s`; independent
`tf2_echo map base_footprint` succeeded around the fault. Global TF publication
was healthy. The remaining blocker is local to the Explorer lookup/callback
path, with executor starvation versus a latest-sample timing race still
unresolved. No production executor change is authorized.

The Sol/high-approved test-only diagnostic in the already-untracked
`src/amr_exploration/test/test_frontier_lifecycle.py` was interrupted before a
reliable result. Its current additions are unverified and must not be treated
as proof of a fix. When work resumes, Sol/high must review that test-only state
and re-diagnose before any production edit or runtime retry.

Phase 15 mapping acceptance is not complete. No passing terminal exploration
report, candidate artifact save/validation, human quality decision, promotion,
or canonical-map replacement is claimed. Evidence is preserved under
`.ros_logs/phase15_tf_diag_20260914_01/factory_mapping/evidence/`; canonical
maps remain unchanged. Product 103/5 kg, Gate 7, physical hardware, and
functional-safety acceptance remain excluded. No simulation processes are
currently running.

The user explicitly authorized this protected handoff synchronization on
2026-09-14. The failure/time-box/stop rules in `AGENTS.md` are binding for any
resume: two consecutive no-progress waits or ten minutes without new evidence
require an early blocker report and stop; an explicit user stop requires a new
explicit user instruction before work resumes.

The initial requirements below remain preserved as project intent and
historical design authority where they are not superseded by the current
phase status and parameter register. No physical-robot or functional-safety
claim is implied by simulation evidence.

## Frozen Decisions
- Jetson Orin Nano
- 2× SICK MRS1000
- Xsens MTi-8
- ZLAC8030D
- ZLTECH hub motors
- Differential drive
- SLAM Toolbox + EKF
- Nav2 + current Regulated Pure Pursuit implementation (retained MPC
  compatibility package/topic names)
- Internal motor PID
- Remove outdoorScan3.
- User designs all CAD.

# Initial Robot Mechanical Concept and Simulation Requirements

Codex must create and maintain a parameterized URDF/Xacro model for ROS 2 simulation.

The user is responsible for all mechanical CAD design. The URDF/Xacro model is intended only for simulation, TF, navigation, controller development, collision checking, and system integration. It is not a manufacturing model.

---

## Initial Robot Geometry

Robot type:
- Differential-drive industrial AMR

Initial chassis dimensions:
- Length: 1000 mm
- Width: 800 mm
- Body height: approximately 600 mm
- Base robot mass (without payload): approximately 30 kg

Coordinate convention:
- +X = Forward
- +Y = Left
- +Z = Up

All dimensions must be parameterized in Xacro.

---

## Ground Clearance

The chassis must not touch the ground.

Simulation requirement:

- Ground clearance: **80 mm**

Ground clearance is defined as the distance between the floor and the lowest rigid chassis component, excluding wheels and caster contact surfaces.

The URDF shall expose ground clearance as a configurable Xacro parameter.

---

## Wheel Configuration

The robot has:

- 2 drive wheels
- 4 passive caster wheels

Layout:

- Left drive wheel
- Right drive wheel
- Front-left caster
- Front-right caster
- Rear-left caster
- Rear-right caster

The drive wheels form the differential-drive axle.

---

## Sensors

Robot sensors:

- Front SICK MRS1000
- Rear SICK MRS1000
- Xsens MTi-8 IMU

Initial mounting:

- Front MRS1000 near the front-left corner
- Rear MRS1000 near the rear-right corner

Create fixed frames:

- front_lidar_link
- rear_lidar_link
- imu_link

Sensor positions shall remain configurable.

---

## Drive System

Drive hardware:

- ZLTECH ZLLG10ASM800 V2.0 Hub Motors
- ZLAC8030D Dual Servo Driver

Whenever specifications are required:

1. Check the project BOM first.
2. Verify the exact model using official manufacturer documentation.
3. Use verified specifications only.
4. Never guess hardware values.
5. Never silently replace component specifications.

---

## Mass and Inertia

Initial unloaded robot mass:

50 kg

Use realistic inertial properties based on simplified geometry.

Do not use zero inertia or unrealistic values.

---

## Required TF Frames

At minimum:

- base_footprint
- base_link
- left_drive_wheel_link
- right_drive_wheel_link
- front_left_caster_link
- front_right_caster_link
- rear_left_caster_link
- rear_right_caster_link
- imu_link
- front_lidar_link
- rear_lidar_link

Follow ROS REP-103.

---

## Required URDF Features

The robot description shall include:

- Visual geometry
- Collision geometry
- Inertial properties
- Differential-drive joints
- Caster wheels
- Sensor frames
- Simulator plugins
- Joint state publisher
- Parameterized dimensions
- Parameterized sensor positions
- Parameterized ground clearance
- Valid TF tree

Primitive geometry shall be used initially.

Detailed CAD meshes are not required.

---

## Required Package

Create:

amr_description

Suggested structure:

amr_description/
├── urdf/
├── launch/
├── config/
├── meshes/
├── rviz/
└── test/

---

## Historical robot-description completion criteria

The robot description phase is NOT complete until:

- Xacro expands successfully.
- URDF validates successfully.
- TF tree contains no errors.
- Robot displays correctly in RViz.
- Robot spawns successfully in simulation.
- Chassis does not intersect the ground.
- Ground clearance equals 80 mm.
- Drive wheels contact the ground correctly.
- Caster wheels contact the ground correctly.
- Differential-drive forward motion works.
- Differential-drive rotation-in-place works.
- IMU frame is correct.
- Both MRS1000 frames are correct.
- Joint-state publishing works.
- All configurable parameters are documented.
- All assumptions are documented.

---

## Engineering Rules

Do NOT build the URDF during Phase 0.

During Phase 0:

- Record robot requirements.
- Identify missing parameters.
- Produce a parameter table.
- Ask for confirmation where required.

The URDF/Xacro package shall be created during the Robot Description / Simulation phase.

## Rules
- Complete one phase at a time.
- Stop after every phase and report progress.
- Wait for approval before continuing.
- Update PROJECT_STATUS.md after every phase.
- Prefer industrial solutions.
- Use at most three temporary subagents.
- Do not redesign completed work without justification.

## Report Format
Summary
Files created
Files modified
Design decisions
Risks
Questions
Next phase
Awaiting approval

## Deliverables
ROS2 packages, configs, launch files, documentation, wiring docs, simulation, tests.
No CAD.

## Repository
Maintain:
PROJECT_STATUS.md
CHANGELOG.md
TODO.md
docs/
SESSION_HANDOFF.md

## Git Workflow

After every completed phase:

1. Update PROJECT_STATUS.md
2. Update CHANGELOG.md
3. Update TODO.md
4. Run project tests
5. Present a summary to the user
6. Wait for approval

Only after approval:

7. Create a Git commit with a descriptive message.

Never push to GitHub without explicit user confirmation.

Never force-push.

Never rewrite Git history unless explicitly instructed.

## Required Skills

When applicable, use the following project skills before implementing work.

### ROS2 Skill
Use for:
- Package creation
- Nodes
- Launch files
- TF
- Parameters
- Lifecycle nodes

### GitHub Skill
Use for:
- Commit creation
- Pull requests
- Branch management

Never push to GitHub without explicit user approval.

### Documentation Skill
Use for:
- Updating PROJECT_STATUS.md
- Updating CHANGELOG.md
- Updating TODO.md
- Creating engineering documentation

### Testing Skill
Use for:
- Running builds
- Unit tests
- Simulation validation
- Regression testing

### Skill By user demand
karpathy-guidelines
mantra-debug

### Subagent Rules

- Only create subagents when beneficial.
- Maximum three subagents.
- Destroy them after the phase completes.

## Skill Invocation Rules

Before beginning any task:

1. Use the **karpathy-guidelines** skill first to plan the implementation strategy whenever software development or engineering design is involved.
2. Determine whether another installed Skill matches the task.
3. If a matching Skill exists, use it instead of reinventing the workflow.
4. If multiple Skills apply, combine them when appropriate.
5. Follow the Skill unless it conflicts with this handoff document.
6. Use **mantra-debug** whenever debugging, investigating failures, or resolving unexpected behavior.
7. Document every Skill used in the phase report.
