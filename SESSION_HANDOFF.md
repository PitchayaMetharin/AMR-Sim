# New Session Handoff

## Required Startup Order

Before taking any project action, read these files completely and in this
order:

1. `AMR_CODEX_HANDOFF.md`
2. `PROJECT_STATUS.md`
3. `TODO.md`
4. `CHANGELOG.md`
5. `SESSION_HANDOFF.md`

Then read the active Phase 3 architecture, the approved Phase 2 and Phase 1
architectures, and their governing Phase 0 evidence:

1. `docs/PHASE_3_COMMUNICATION_ARCHITECTURE.md`
2. `docs/PHASE_2_ELECTRICAL_POWER_ARCHITECTURE.md`
3. `docs/PHASE_1_SYSTEM_ARCHITECTURE.md`
4. `docs/PHASE_0_REQUIREMENTS.md`
5. `docs/ROBOT_PARAMETER_REGISTER.md`
6. `docs/PHASE_0_SOFTWARE_BASELINE.md`
7. `docs/PHASE_0_TEST_REPORT.md`
8. `docs/SIMULATION_RISK_REDUCTION_PLAN.md`
9. `docs/PHASE_0_SAFETY_SCOPE.md`
10. `docs/PHASE_0_BOM_REVIEW.md`

The governing files take precedence over this summary if a discrepancy is
found.

## Transfer State

- Current phase: Phase 3 — Communication architecture.
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
- Phase 2 was locally committed as `9e64d41` with message
  `docs: complete phase 2 electrical power architecture`.
- The user explicitly authorized Phase 3.
- Phase 3 communication architecture is documented and validated locally.
- The user approved Phase 3 on 2026-07-25.
- The local Phase 3 closure commit is pending.
- Phase 4 has not started and is not authorized.
- No URDF/Xacro, ROS 2 project package, simulation world, or mechanical CAD has
  been created.
- `src/` is empty.
- Do not create Phase 4 ROS workspace artifacts or the Phase 6 model without
  explicit user authorization for the applicable phase.
- `AMR_CODEX_HANDOFF.md` has a pre-existing uncommitted user edit that removes
  approved scope language and restores older conflicting S7-1200F/50 kg
  unloaded-mass text. Preserve it untouched unless the user explicitly directs
  reconciliation. The approved Phase 1/2 records govern current Phase 3 work.

The user's last directions were to approve Phase 3 and retain user ownership of
future ladder programming. Codex shall provide the ladder-programming guide,
tag/interface mapping, state-machine and cause/effect guidance, test checklist,
and review support. Do not author the ladder program unless the user later
explicitly requests it. No weekly-usage percentage is exposed to the agent in
this workspace; stop if the interface surfaces the user's requested 5% warning.
The basic primitive-geometry URDF/Xacro model remains assigned to Phase 6.

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
- Phase 3 is communication architecture only. It does not authorize applying
  network/firewall/certificate settings, creating ROS packages, implementing
  the OPC UA gateway, or creating TIA/PLC/HMI code.
- The user owns Phase 12 ladder implementation. Codex supplies guidance,
  mappings, cause/effect design, tests, and review unless separately authorized
  to implement code.
- Do not treat the S7-1500F simulation as automatically equivalent to a future
  S7-1200F implementation. WinCC edition, license, firmware, and runtime remain
  explicit decisions.
- After phase approval, create a local Git commit with a clear message.
- Never push to GitHub without explicit instruction.
- Keep `PROJECT_STATUS.md`, `TODO.md`, and `CHANGELOG.md` synchronized.

## Phase 3 Communication Baseline

- DDS remains on the Ubuntu host; Windows receives no ROS discovery or DDS
  traffic.
- OPC UA is the sole inter-laptop application protocol.
- PLCSIM Advanced S7-1500F is the OPC UA server; one Ubuntu ROS gateway is the
  client.
- The non-applied closed-network plan is `192.168.50.0/24`: Ubuntu
  `192.168.50.10`, Windows `192.168.50.20`, no gateway, DNS, or DHCP.
- Before application, verify subnet collision, interface identity, PLCSIM
  adapter support, and Windows/Ubuntu firewall ownership.
- Fast DDS through `rmw_fastrtps_cpp` and `ROS_DOMAIN_ID=1` are retained.
  Phase 4 shall set `ROS_LOCALHOST_ONLY=1`; the current shell was observed as
  `0`.
- The planned OPC UA endpoint is `opc.tcp://192.168.50.20:4840`, subject to the
  exact server path exposed by the verified toolchain.
- Drive-enabled tests require a verified secure endpoint, preferred
  `SignAndEncrypt` with `Basic256Sha256`, and explicit certificate trust.
- Unsecured OPC UA is allowed only as a documented, closed-network,
  motion-inhibited diagnostic exception.
- Resolve Siemens namespace
  `http://www.siemens.com/simatic-s7-opcua` at every session; never hardcode a
  numeric namespace index.
- The symbolic interface root is `DB_AMR_OPCUA` with `Interface`, `RosToPlc`,
  `PlcToRos`, and `Diagnostics` groups.
- ROS request bundles use a commit-last sequence. PLC state uses a double-read
  `StateSeq` coherent-snapshot check. Reset/enable/stop requests require
  sequence-correlated acknowledgement.
- Initial simulation timing: 100 ms gateway heartbeat, 500 ms PLC heartbeat
  watchdog, 100 ms PLC state update, 300 ms ROS PLC-state freshness, and
  200 ms stamped motion-command expiry.
- Reconnect cancels pending requests, makes the ROS authority snapshot
  non-permissive, revalidates endpoint/security/namespace/schema/boot IDs, and
  returns only to ready-inhibited.
- Gazebo time stamps robot data; Ubuntu steady time governs gateway freshness;
  PLC elapsed time governs the PLC watchdog; UTC wall time is evidence only.
- Canonical ROS names, QoS profiles, field types, electrical-signal mapping,
  inhibition enumerations, fault responses, and verification tests are in
  `docs/PHASE_3_COMMUNICATION_ARCHITECTURE.md`.
- In Phase 12, the user implements the TIA ladder program and PLC/HMI project.
  Codex supplies the guide, OPC UA/tag mapping, state-transition and
  cause/effect guidance, test checklist, and review. Phase 13 applies and
  validates the network and cross-host integration.

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

Phase 2 is closed by the local commit:

```text
9e64d41 docs: complete phase 2 electrical power architecture
```

Phase 3 changes are approved and awaiting the local closure commit. A pre-existing
uncommitted user edit to `AMR_CODEX_HANDOFF.md` is outside the Phase 3 change
set and must remain untouched unless the user directs otherwise. No Git push
has occurred. Phase 4 has not started.

## Deferred Inputs

- Effective wheel radii and wheel separation.
- Drive-wheel and caster mounting poses.
- Caster radius, width, trail, load distribution, friction, and damping.
- Payload geometry, restraint, center-of-gravity range, and inertia profile.
- Exact MRS1000 and IMU poses.
- Floor friction, slope, threshold, gap, contamination, and environment limits.
- Reverse-speed design limit.
- Detailed PLC state transitions, timer/latch/reset implementation, and
  cause/effect matrix.
- Applied network-interface, firewall, certificate, and trust-store
  configuration after collision and toolchain checks.
- MPC solver/plugin and timing design.
- Acceptance routes, obstacle classes, trial count, docking method, and
  recovery-time limit.
- ROS 2 Humble migration plan before May 2027 end of support.
- TIA Portal update, STEP 7/Safety license, PLCSIM Advanced V4.0 update, exact
  S7-1500F model/firmware, WinCC edition, and HMI runtime.
- Exact installed OPC UA endpoint path, secure-policy/user-token support,
  certificate profile, and server-revised subscription intervals.
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

1. Create a local commit containing only the approved Phase 3 files; do not
   include the pre-existing `AMR_CODEX_HANDOFF.md` edit without explicit user
   direction.
2. Refresh this handoff and project status with the Phase 3 commit hash.
3. Wait for separate explicit user authorization for Phase 4.
4. Do not create ROS packages, URDF/Xacro, PLC code, HMI screens, or later-phase
   implementation artifacts under the Phase 3 authorization.
