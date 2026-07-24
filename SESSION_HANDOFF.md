# New Session Handoff

## Required Startup Order

Before taking any project action, read these files completely and in this
order:

1. `AMR_CODEX_HANDOFF.md`
2. `PROJECT_STATUS.md`
3. `TODO.md`
4. `CHANGELOG.md`
5. `SESSION_HANDOFF.md`

Then read the active Phase 2 architecture, the approved Phase 1 architecture,
and their governing Phase 0 evidence:

1. `docs/PHASE_2_ELECTRICAL_POWER_ARCHITECTURE.md`
2. `docs/PHASE_1_SYSTEM_ARCHITECTURE.md`
3. `docs/PHASE_0_REQUIREMENTS.md`
4. `docs/ROBOT_PARAMETER_REGISTER.md`
5. `docs/PHASE_0_SOFTWARE_BASELINE.md`
6. `docs/PHASE_0_TEST_REPORT.md`
7. `docs/SIMULATION_RISK_REDUCTION_PLAN.md`
8. `docs/PHASE_0_SAFETY_SCOPE.md`
9. `docs/PHASE_0_BOM_REVIEW.md`

The governing files take precedence over this summary if a discrepancy is
found.

## Transfer State

- Current phase: Phase 2 — Electrical and power architecture.
- Phase 0 was approved and locally committed as `7db85f7`.
- Phase 1 architecture was prepared for approval, then reopened when the user
  specified TIA Portal V17 for PLC and HMI engineering.
- TIA Portal V17 is fixed on a separate Windows laptop.
- ROS 2, Gazebo, RViz, and AMR development remain on the Ubuntu laptop.
- The laptops communicate over Ethernet using OPC UA, provisionally with the
  virtual PLC as server and the ROS 2 gateway as client.
- The user selected S7-1500F as the current simulated PLC family, resolving the
  PLCSIM Advanced compatibility conflict.
- S7-1200F remains only a conceptual future physical BOM candidate.
- Phase 1 is approved and complete.
- Phase 1 was locally committed as `8be2e8b` with message
  `docs: complete phase 1 system architecture`.
- The user explicitly authorized Phase 2.
- Phase 2 architecture is approved and complete.
- The local Phase 2 closeout commit is the next action.
- No Phase 3 work has started or is authorized.
- No URDF/Xacro, ROS 2 project package, simulation world, or mechanical CAD has
  been created.
- `src/` is empty.
- Do not create Phase 3 communication artifacts or the Phase 6 model without
  explicit user authorization for the applicable phase.

The user's last directions were to retain the 48 V, 30 Ah battery baseline,
approve Phase 2, and prepare this new-session handoff. The basic
primitive-geometry URDF/Xacro model remains assigned to Phase 6.

## Non-Negotiable Workflow

- Work on one phase only.
- Never skip phases.
- Do not proceed to the next phase without explicit user approval.
- Do not create mechanical CAD; the user owns mechanical design.
- Phase 0 explicitly prohibits creating the project URDF.
- Phase 1 is architecture-only and does not authorize later-phase
  implementation.
- Phase 2 is electrical/power architecture only. It does not authorize physical
  wiring, procurement, PLC code, ROS/OPC UA contracts, or later-phase
  implementation.
- Do not treat the S7-1500F simulation as automatically equivalent to a future
  S7-1200F implementation. WinCC edition, license, firmware, and runtime remain
  explicit decisions.
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

## Phase 2 Electrical and Power Baseline

- Current runtime power is supplied by the two laptops; the AMR battery and
  onboard electrical system remain conceptual future references.
- Conceptual domains are battery/source, traction, regulated 24 V control,
  regulated 12 V compute, device-local low voltage, and charging.
- The user confirmed a 48 V, 30 Ah nominal LiFePO4 battery, giving 1.44 kWh
  nominal stored energy. Exact battery/BMS models and usable energy remain
  unknown.
- Retain 48 V, 30 Ah as the current capacity baseline unless the user later
  directs a higher-capacity selection.
- Control and compute branches are upstream of the propulsion contactors so
  supervision and diagnostics can remain available when traction is isolated.
- The provisional 24 V load is 174 W (7.25 A). The workbook's 25% allowance
  gives 217.5 W (9.0625 A), leaving 22.5 W on the 240 W DDR-240C-24 candidate.
- The provisional 12 V load is 40 W (3.333 A). The 25% allowance gives 50 W
  (4.167 A), leaving 10 W on the 60 W DDR-60L-12 candidate.
- These are architecture calculations, not final supply validation. Most BOM
  load rows still require exact-variant evidence, simultaneous-state checks,
  derating, wiring-loss, inrush, and thermal review.
- The 1,625 W traction value is only a nameplate sum and cannot size the
  battery, BMS, protection, conductors, contactors, precharge, or endurance.
- The provisional all-listed auxiliary load case requires 1.712 kWh at the
  loads over eight hours and approximately 1.881 kWh using typical converter
  efficiencies, before traction and reserve. The 30 Ah pack therefore does not
  close the eight-hour endurance target.
- The current user-selected runtime case uses 50% of combined motor nameplate
  power (800 W), 25 W driver allowance, and approximately 235 W auxiliary
  source load. Total planning load is 1,060 W and ideal nominal runtime is
  1.36 hours, approximately 1 hour 22 minutes. This is not measured or
  guaranteed physical endurance.
- ZLTECH verifies a 24–48 VDC input range for ZLAC8030D.
- Blue Sea verifies 48 VDC maximum for the 6006 disconnect.
- The battery is 48 V nominal, but its exact model and full voltage window are
  unknown. A nominal 48 V rating does not prove compatibility with devices
  rated to 48 V maximum, so battery-to-driver and battery-to-disconnect
  compatibility remain blocked.
- The conceptual BOM's provisional 80–100 A fuse and 100 ohm/100 W precharge
  values are not approved design inputs.
- Regenerative-energy handling, driver DC-link capacitance, BMS charge
  acceptance, fault current, current limits, and protection coordination are
  open.
- K1/K2 are represented as independently commanded and monitored series
  traction-isolation elements. This creates no PL/Category claim.
- Electrical states and 22 logical simulated signal meanings are frozen in
  `docs/PHASE_2_ELECTRICAL_POWER_ARCHITECTURE.md`; Phase 3 owns transport and
  Phase 12 owns detailed PLC logic.

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

## TIA Portal V17 Baseline

- TIA Portal V17 is required for PLC and HMI engineering.
- A separate Windows laptop runs TIA Portal V17, PLCSIM Advanced, and HMI
  simulation.
- The Ubuntu laptop runs ROS 2, Gazebo, RViz, and AMR development.
- The laptops use Ethernet and OPC UA for the ROS/PLC interface.
- S7-1500F is the current simulated PLC family through PLCSIM Advanced V4.0
  provisionally; exact model, firmware, and installed update remain TBD.
- S7-1200F is only a future physical candidate and would require explicit
  porting and revalidation.
- Exact TIA update, STEP 7/Safety license, WinCC edition, panel images, and HMI
  runtime/simulator remain TBD.
- HMI commands must pass through the PLC/ROS contract and may not publish
  directly to the motion path or force drive permission.

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
- Phase 1 Markdown links, architecture decision IDs, parameter IDs,
  project-record consistency, diff whitespace, code fences, and empty-`src/`
  boundary passed after the TIA/S7-1500F revision.
- Phase 2 Markdown links, decision IDs, signal IDs, parameter IDs, power-budget
  arithmetic, project-record consistency, diff whitespace, code fences, and
  empty-`src/` boundary passed.

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

Phase 0 is committed locally:

```text
7db85f7 docs: complete phase 0 requirements baseline
```

Phase 1 is closed by the local commit:

```text
8be2e8b docs: complete phase 1 system architecture
```

Phase 2 is approved. Its documentation and project-record changes are ready
for the local closeout commit. No Git push has occurred. Phase 3 has not
started.

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
- TIA Portal update, STEP 7/Safety license, PLCSIM Advanced V4.0 update, exact
  S7-1500F model/firmware, WinCC edition, and HMI runtime.
- OPC UA namespace, security, data ownership, heartbeat, timeout,
  acknowledgement, reconnect, and Ethernet addressing.
- Exact battery/BMS model, complete voltage window, current/fault limits,
  usable portion of the confirmed 1.44 kWh nominal energy, reserve, thermal
  limits, protection behavior, and interface.
- ZLAC8030D manual/revision, DC-link capacitance, current and transient limits,
  regeneration, braking, enable, protection, and fault behavior.
- Exact motor electrical, torque, speed, current, efficiency, encoder, and
  thermal data.
- Source and branch protection, disconnect, conductors, connectors, contactors,
  precharge, grounding, bonding, shielding, isolation, surge, EMC, and thermal
  design.
- Charger, charging contacts, charge profile/current, pilot/interlock,
  protocol, and docking sequence.
- Mission duty-cycle power profile, acceptance of the 30 Ah capacity against
  the eight-hour target, SOC thresholds, and shutdown reserve.

## Exact Next Action

1. Create the approved local Phase 2 closeout commit.
2. Refresh this handoff with the resulting commit ID and verify a clean
   workspace.
3. Do not start Phase 3 without separate explicit user approval.
4. Do not create ROS packages, URDF/Xacro, PLC code, HMI screens, or later-phase
   implementation artifacts under the Phase 2 authorization.
