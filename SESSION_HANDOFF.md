# New Session Handoff

## Required Startup Order

Before taking any project action, read these files completely and in this
order:

1. `AMR_CODEX_HANDOFF.md`
2. `PROJECT_STATUS.md`
3. `TODO.md`
4. `CHANGELOG.md`
5. `SESSION_HANDOFF.md`

Then read the Phase 0 evidence relevant to the next task:

1. `docs/PHASE_0_REQUIREMENTS.md`
2. `docs/ROBOT_PARAMETER_REGISTER.md`
3. `docs/PHASE_0_SOFTWARE_BASELINE.md`
4. `docs/PHASE_0_TEST_REPORT.md`
5. `docs/SIMULATION_RISK_REDUCTION_PLAN.md`
6. `docs/PHASE_0_SAFETY_SCOPE.md`
7. `docs/PHASE_0_BOM_REVIEW.md`

The governing files take precedence over this summary if a discrepancy is
found.

## Transfer State

- Current phase: Phase 0 — Requirements and architecture confirmation.
- Phase 0 engineering work, review revisions, environment setup, and available
  tests are complete.
- Phase 0 is awaiting explicit user approval for its local Git commit.
- No Phase 1 work has started.
- No URDF/Xacro, ROS 2 project package, simulation world, or mechanical CAD has
  been created.
- `src/` is empty.
- Do not start Phase 1 or create the Phase 6 model without explicit user
  authorization.

The user's last direction was to keep the basic primitive URDF/Xacro model in
Phase 6.

## Non-Negotiable Workflow

- Work on one phase only.
- Never skip phases.
- Do not proceed to the next phase without explicit user approval.
- Do not create mechanical CAD; the user owns mechanical design.
- Phase 0 explicitly prohibits creating the project URDF.
- After phase approval, create a local Git commit with a clear message.
- Never push to GitHub without explicit instruction.
- Keep `PROJECT_STATUS.md`, `TODO.md`, and `CHANGELOG.md` synchronized.

## Current Project Scope

- Simulation-only academic AMR project.
- Runs on the laptop using ROS 2 Humble, Ubuntu 22.04, and Gazebo Harmonic.
- C++17 minimum is the required implementation language.
- No physical hardware will be purchased, installed, commissioned, or
  certified in the current project.
- Candidate hardware remains a conceptual reference for simulated interfaces
  and sensor characteristics.

## Frozen Robot Baseline

- Differential-drive AMR.
- Nominal body envelope: 1.000 m long × 0.800 m wide × approximately 0.600 m
  high.
- Rigid-chassis ground clearance: 0.080 m.
- Unloaded mass: 30 kg nominal with ±5 kg provisional tolerance.
- Default and initially rated simulated payload: 50 kg.
- Initial total simulated moving mass: approximately 80 kg.
- Payload is a manual Xacro/launch parameter selected before model spawn.
- A 300 kg payload is only an optional future stress case and physical design
  target, not the current rating.
- Two nominal 0.127 m radius drive wheels.
- Four passive casters.
- Exact caster geometry, wheel separation, and poses remain deferred.

## Sensor Baseline

- 2 × simulated SICK MRS1104C-111011, order number 1081208.
- Front sensor near the front-left corner.
- Rear sensor near the rear-right corner.
- Simulated IMU using Xsens MTi-8 characteristics as a future hardware
  reference.
- Exact sensor poses remain configurable and deferred.
- MRS1000/Nav2 perception is operational navigation perception, not
  safety-rated personnel protection.

## Phase 6 Primitive Model Strategy

Final CAD is not required for the initial simulation model. Phase 6 will create
a parameterized `amr_description` package using:

- box or compound primitives for the chassis/body;
- cylinders for the drive wheels;
- simplified parameterized passive casters;
- a primitive payload body with 50 kg default mass and derived inertia;
- primitive MRS1000 and IMU housings with configurable frames;
- Gazebo LiDAR and IMU sensors;
- realistic nonzero inertias;
- visual and collision geometry;
- differential-drive joints and simulator integration.

Later CAD meshes may replace visual geometry without changing validated TF,
joint, collision, inertia, sensor, or controller interfaces.

## Motion Baseline

- Linear speed: 1.0 m/s design; 0.5 m/s initial simulation limit.
- Angular speed: 0.8 rad/s design; 0.4 rad/s initial simulation limit.
- Linear acceleration: 0.4 m/s².
- Normal linear deceleration: 0.5 m/s².
- Provisional controlled-command deceleration: 1.0 m/s².
- Angular acceleration/deceleration: 0.6/0.8 rad/s².
- Linear/angular jerk: 0.5 m/s³ and 1.0 rad/s³.

These are software constraints, not verified physical performance.

## Installed Environment

- Ubuntu 22.04.5 LTS, x86_64.
- ROS 2 Humble using `rmw_fastrtps_cpp`.
- GCC/G++ 11.4.0.
- CMake 3.22.1.
- `colcon-core` 0.21.0.
- Gazebo Harmonic / Gazebo Sim 8.14.0.
- Gazebo Classic 11 remains installed but is not the project baseline.
- `ros-humble-ros-gzharmonic` 0.244.12-3jammy.
- `ros-humble-ros-gzharmonic-bridge` 0.244.12-3jammy.
- `ros-humble-ros-gzharmonic-sim` 0.244.12-3jammy.
- `joint_state_publisher` 2.4.0.
- `joint_state_publisher_gui` 2.4.0.

Do not install the conflicting `ros-humble-ros-gz` or
`ros-humble-ros-gzgarden` packages.

## Completed Validation

The detailed record is `docs/PHASE_0_TEST_REPORT.md`. Current evidence includes:

- strict C++17 `ament_cmake` build and node runtime passed;
- Fast DDS delivered 6 of 6 messages;
- Gazebo headless startup, GUI/OGRE2, transport, physics, clock, statistics,
  and pause control passed;
- RViz started with OpenGL/GLSL 4.6;
- SDFormat validation and analytical inertia calculation passed;
- Gazebo-to-ROS `/clock` delivery passed;
- LaserScan, PointCloud2, and IMU bridge type mappings passed;
- joint-state publisher modules and executable interfaces loaded;
- `ros_gz_sim create` model-spawn interface loaded;
- package database audit passed;
- BOM XLSX archive integrity passed;
- parameter register IDs are unique.

Warnings:

- Gazebo GUI emits one non-fatal QML world-statistics binding-loop warning.
- `rosdep` emits a Python `pkg_resources` deprecation warning.
- Actual dual-LiDAR/IMU data, robot dynamics, SLAM, Nav2, EKF, MPC, payload
  stability, braking, and caster behavior require later project artifacts.

## Safety Boundary

- Intended future jurisdiction: Thailand.
- Conceptual references: ISO 3691-4:2023, ISO 12100:2010,
  ISO 13849-1:2023, and IEC 60204-1:2016+A1:2021.
- No PL, SIL, certification, or industrial-suitability claim is made.
- PL d, Category 3 is only a provisional future architecture target.
- Present validation authority is the project team and university supervisor.
- Simulation cannot validate structural capacity, traction, stopping distance,
  functional safety, or physical compliance.

## BOM State

- Workbook: `Industrial_AMR_BOM_with_Thailand_Suppliers_Prices.xlsx`.
- Five sheets; archive validation passes.
- LMS151 and outdoorScan3 entries were removed.
- Exactly two MRS1104C-111011 / 1081208 entries remain.
- Caster quantity is four.
- Supplier/shop/price columns in later rows remain shifted and unreliable.
- The user explicitly deferred that procurement-data correction.
- The BOM is a conceptual future reference; no procurement is planned.

## Git and Workspace State

At handoff creation, Phase 0 changes are uncommitted:

```text
 M AMR_CODEX_HANDOFF.md
 M CHANGELOG.md
 M PROJECT_STATUS.md
 M TODO.md
?? Industrial_AMR_BOM_with_Thailand_Suppliers_Prices.xlsx
?? SESSION_HANDOFF.md
?? docs/
```

Do not discard or overwrite these changes. They are the accumulated Phase 0
work. No Git push has occurred.

## Deferred Inputs

- Effective wheel radii and wheel separation.
- Drive-wheel and caster mounting poses.
- Caster radius, width, trail, load distribution, friction, and damping.
- Payload geometry, restraint, center-of-gravity range, and inertia profile.
- Exact MRS1000 and IMU poses.
- Floor friction, slope, threshold, gap, contamination, and environment limits.
- Reverse-speed design limit.
- ROS/PLC protocol, heartbeat timing, state machine, and cause/effect matrix.
- Network addressing, ROS QoS, and time synchronization.
- MPC solver/plugin and timing design.
- Acceptance routes, obstacle classes, trial count, docking method, and
  recovery-time limit.
- ROS 2 Humble migration plan before May 2027 end of support.

## Exact Next Action

1. Ask the user for explicit Phase 0 approval if it has not been given.
2. If changes are requested, remain in Phase 0, implement them, rerun affected
   validation, and resynchronize documentation.
3. If Phase 0 is explicitly approved, review the complete diff and create a
   local commit such as:

   `docs: complete phase 0 requirements baseline`

4. Stop after the commit.
5. Start Phase 1 only after separate explicit authorization.
