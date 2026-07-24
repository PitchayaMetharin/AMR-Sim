# AMR Project Handoff for Codex

Current session-transfer summary: `SESSION_HANDOFF.md`.

## Role
You are the Lead Robotics Engineer responsible for completing this industrial AMR project. Do not teach unless asked.

## Current Project Scope — 2026-07-24

- The current project is a simulation-only academic AMR project.
- All ROS 2, SLAM, localization, navigation, sensor, PLC-authority, drive, and
  vehicle behavior runs across the approved Ubuntu and Windows laptops using
  simulated data.
- Gazebo Harmonic is the selected simulator.
- No physical hardware will be purchased, installed, wired, commissioned, or
  certified in the current project.
- Jetson Orin Nano Developer Kit, Siemens PLC/network hardware, SICK MRS1000,
  Xsens MTi-8-5A-DK, ZLAC8030D, motors, battery, and safety equipment remain
  conceptual candidates for a possible future physical implementation.
- The BOM is a conceptual engineering reference, not an active procurement
  list.
- Physical functional-safety compliance is outside scope. The simulation may
  model safety architecture and state behavior but shall make no PL/SIL or
  certification claim.

## Frozen Decisions
- Jetson Orin Nano
- Siemens S7-1500F simulated PLC through PLCSIM Advanced
- Siemens S7-1200F retained only as a conceptual future physical candidate
- 2× SICK MRS1000
- Xsens MTi-8
- ZLAC8030D
- ZLTECH hub motors
- Differential drive
- SLAM Toolbox + EKF
- Nav2 + MPC
- Internal motor PID
- Remove outdoorScan3.
- User designs all CAD.

# Robot Mechanical Concept and Simulation Requirements

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

Approximately 30 kg with a preliminary tolerance of ±5 kg.

This value includes the complete operational reference AMR and excludes
transported payload. It is a provisional simulation input, not a measured
physical mass.

Simulated payload:

- Default and initially rated simulated payload: 50 kg.
- Nominal initial total simulated moving mass: approximately 80 kg.
- Payload mass shall be a manual Xacro or launch parameter so the user can
  change it before spawning a later simulation run.
- Live payload adjustment during an active Gazebo session is not required.
- Payload center of gravity and inertia shall remain physically consistent
  with the selected payload mass and geometry.
- A 300 kg payload may be retained as an optional future simulation stress case
  and physical design target. It is not the current simulated rating.

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
- Parameterized payload mass with a 50 kg default
- Consistent payload center of gravity, collision geometry, and inertia
- Manual payload override before spawning a simulation run
- Valid TF tree

Primitive geometry shall be used initially.

Detailed CAD meshes are not required.

Final CAD is not a prerequisite for the Phase 6 simulation model. The initial
robot shall use parameterized boxes and cylinders for chassis, payload,
wheels, casters, and sensor housings. Later CAD meshes may replace visual
geometry without changing the validated TF, joint, collision, inertia, or
controller interfaces.

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

## Completion Criteria

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
- The default 50 kg payload produces approximately 80 kg total moving mass and
  valid inertial properties.
- A manual payload override before model spawn produces consistent mass,
  center-of-gravity, collision, and inertia values.
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
