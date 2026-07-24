# Phase 0 Requirements and Architecture Baseline

## Purpose

This document records the Phase 0 baseline for a simulation-only academic
industrial differential-drive AMR concept. It is a requirements and decision
record only. No URDF/Xacro, ROS 2 package, simulation asset, or mechanical CAD
is part of Phase 0.

The detailed geometry and configuration inputs are maintained in
[`ROBOT_PARAMETER_REGISTER.md`](ROBOT_PARAMETER_REGISTER.md).

## Requirement Status

- **Confirmed**: explicitly required and not contradicted by another project
  source.
- **Provisional**: supplied as an initial concept and must remain configurable
  or be confirmed before implementation.
- **Conflict**: contradictory values exist; implementation is blocked until the
  user selects the governing value.
- **TBD**: no value has been supplied.
- **Verification required**: a selected hardware family or model must be
  reconciled with the project BOM and official manufacturer documentation
  before its specifications are used.

## Scope and Authority

1. The user owns all mechanical CAD and manufacturing design.
2. Codex owns the ROS 2 software, configuration, integration documentation,
   primitive-geometry simulation model, and validation artifacts requested by
   the phase plan.
3. The future URDF/Xacro is a simulation, TF, navigation, controller,
   collision-checking, and integration model. It is not manufacturing CAD.
4. Work is phase-gated. Only one phase may be active, and the next phase
   requires explicit user approval.
5. Hardware values must be obtained by checking the project BOM first and then
   verifying the exact model or ordering code against official manufacturer
   documentation.
6. The current project runs entirely on a laptop using simulated robot and
   sensor data. Candidate physical hardware is retained only as a future
   architecture reference.
7. Physical procurement, assembly, hardware commissioning, certification, and
   industrial deployment are outside the current project.

## Confirmed System Requirements

### Robot and mechanics

- Robot type: industrial differential-drive AMR.
- Coordinate convention: ROS REP-103, with +X forward, +Y left, and +Z up.
- Chassis nominal plan dimensions: 1.000 m long by 0.800 m wide.
- Nominal body height: approximately 0.600 m; its reference surfaces remain
  undefined.
- Required rigid-chassis ground clearance: 0.080 m, measured from the floor to
  the lowest rigid chassis component and excluding wheel/caster contact
  surfaces.
- Running gear: two drive wheels and four passive caster wheels.
- Caster selection: four TENTE LEVINA 5370PJP100P62 swivel casters, pending
  dimensional and load verification.
- Caster locations: front-left, front-right, rear-left, and rear-right.
- Mechanical CAD and detailed meshes are excluded from Codex scope.

### Compute, control, and networking selections

- The laptop is the current simulation compute platform.
- NVIDIA Jetson Orin Nano Developer Kit 8GB is a candidate future physical
  compute platform and will not be purchased or used in the current project.
- Siemens S7-1200F PLC, SCALANCE switch, 48 V LiFePO4 system, ZLAC8030D, and
  ZLTECH motors are conceptual future physical selections.
- Phase 1 subsequently selected S7-1500F through PLCSIM Advanced as the current
  simulated PLC family for Ethernet/OPC UA integration. It is not claimed to be
  equivalent to the future S7-1200F candidate.
- Two conceptual ZLTECH ZLLG10ASM800 V2.0 10-inch hub motors.
- Manufacturer-confirmed nominal wheel diameter: 0.254 m; provisional nominal
  URDF radius: 0.127 m. This is not a measured rolling radius.
- The motor driver's internal wheel-speed PID is the selected low-level speed
  loop.

### Sensors

- Two SICK MRS1104C-111011 units, order number 1081208, define the simulated
  indoor/outdoor 3D perception characteristics for SLAM, obstacle detection,
  and Nav2 costmaps. No physical units will be purchased in the current
  project.
- Verified sensor supply/load basis: 10–30 VDC, typical 13 W, maximum 37 W per
  unit, including a 30 W maximum one-second startup phase.
- Initial qualitative mounting concept:
  - front unit near the front-left corner;
  - rear unit near the rear-right corner.
- Xsens MTi-8-5A-DK is a candidate future physical IMU. Gazebo-generated IMU
  data replaces physical measurements in the current project.
- MRS1000 firmware constraints, driver selection, IP plan, and bench validation
  remain open.

### Software and algorithms

- ROS 2 Humble on Ubuntu 22.04 LTS is confirmed.
- C++ is the required primary implementation language for production ROS 2
  nodes, hardware interfaces, control, and MPC. C++17 is the minimum standard.
- SLAM Toolbox is selected for mapping/localization workflow.
- `robot_localization` EKF is selected for state estimation.
- Nav2 is selected for global planning and costmaps.
- MPC is selected as the local motion controller; the implementation and Nav2
  integration approach are TBD.
- Gazebo Harmonic is the selected simulator for Phase 6. The installed Gazebo
  Classic 11 is not the project baseline.
- MoveIt 2 is installed but is outside the initial mobile-base scope. It shall
  be used only if a later manipulation or motion-planning requirement needs it.
- Differential-drive kinematics are required.

The inspected software environment is recorded in
[`PHASE_0_SOFTWARE_BASELINE.md`](PHASE_0_SOFTWARE_BASELINE.md).

### Mass, payload, and motion limits

- Unloaded robot mass: approximately 30 kg.
- Default and initially rated simulated payload: 50 kg.
- Nominal initial total simulated moving mass: approximately 80 kg.
- Payload mass shall be a manual Xacro or launch parameter that can be changed
  before spawning a simulation run.
- Live payload adjustment during an active Gazebo session is not required.
- The payload model shall derive or select consistent mass, center of gravity,
  collision representation, and inertia. Changing only a scalar mass without
  consistent inertial properties is not acceptable.
- A 300 kg payload remains an optional future simulation stress case and
  physical design target, not the current simulated rating.
- Design maximum linear speed: 1.0 m/s.
- Initial simulation and commissioning linear-speed limit: 0.5 m/s.
- Design maximum angular speed: 0.8 rad/s.
- Initial simulation and commissioning angular-speed limit: 0.4 rad/s.
- Maximum linear acceleration: 0.4 m/s².
- Maximum normal linear deceleration: 0.5 m/s².
- Provisional maximum commanded emergency deceleration: 1.0 m/s².
- Maximum angular acceleration: 0.6 rad/s².
- Maximum angular deceleration: 0.8 rad/s².
- Linear jerk limit: 0.5 m/s³.
- Angular jerk limit: 1.0 rad/s³.

The speed and motion limits are initial software constraints, not verified
hardware capability. Increases from commissioning limits require stability,
braking, perception, controller, traction, payload, and stopping tests.
The 1.0 m/s² value describes a commanded controlled deceleration target; it
does not define or guarantee emergency-stop behavior after the PLC removes
drive permission or motor power. Stop categories, coast-down behavior, and
safety stopping distances require separate validation.

### Kinematic calibration requirements

- Wheel radius, wheel separation, sensor poses, and related controller/EKF
  values shall remain consistently configurable.
- Initial wheel radius shall use the verified 0.127 m nominal value.
- Effective rolling radius shall be measured under normal operating load over
  multiple complete rotations, separately for left and right wheels.
- Wheel separation shall be measured between effective ground-contact
  centerlines and later calibrated using repeated rotations and an external
  heading reference.
- Final LiDAR and IMU transforms shall be measured from the approved
  `base_link` origin after mechanical mounting is complete.

### Safety

- The SICK outdoorScan3 has been removed from the architecture.
- The MRS1000 units, SLAM, and Nav2 costmaps shall not be represented as a
  certified personnel-safety system.
- Physical emergency stops, contactors, and PLC-based power shutdown shall be
  retained.
- The PLC has final drive-permission authority. ROS may request motion and
  drive enable but cannot override PLC permissives.
- Loss of the ROS/Jetson heartbeat shall cause the PLC to remove motion
  permission or transition the machine to a defined stopped state.
- Required risk reduction, safety functions, performance level/SIL targets,
  and formal validation would be determined for a future physical machine.
- The intended future jurisdiction is Thailand.
- ISO 3691-4:2023 is the primary AMR safety reference. ISO 12100:2010,
  ISO 13849-1:2023, and IEC 60204-1:2016+A1:2021 are supporting references.
- No PL or SIL compliance is claimed for this simulation-only academic
  project.
- PL d, Category 3 is a provisional conceptual target for critical future
  physical safety functions; it is not a derived or validated PLr.
- Present validation authority is the project team and university supervisor.
  Formal competent/accredited safety validation is required before physical
  deployment.

The complete boundary is recorded in
[`PHASE_0_SAFETY_SCOPE.md`](PHASE_0_SAFETY_SCOPE.md).

The staged test and risk-reduction boundary is recorded in
[`SIMULATION_RISK_REDUCTION_PLAN.md`](SIMULATION_RISK_REDUCTION_PLAN.md).

### PLC/ROS responsibility boundary

The simulated PLC authority, implemented using S7-1500F in PLCSIM Advanced,
owns:

- emergency-stop and safety-bumper supervision;
- contactor, drive-power, and hardware-permissive control;
- safe torque/power removal supported by the selected hardware;
- safety input/output feedback, reset, restart, and fault-latch logic;
- main power sequencing;
- ROS/Jetson watchdog supervision;
- prevention of motion when safety or hardware conditions are unsatisfied;
- exposure of safety and machine state to ROS.

The simulated ROS 2 system on the laptop owns the following responsibilities.
The same boundary is intended for a possible future Jetson deployment:

- LiDAR and IMU drivers, wheel odometry, and EKF;
- mapping, SLAM/localization, Nav2 planning, costmaps, and operational obstacle
  detection;
- MPC path tracking and motion requests to the driver;
- mission execution, diagnostics, logging, user interfaces, and future external
  interfaces;
- heartbeat and software-health reporting to the PLC.

### Future robot-description requirements

The parameterized `amr_description` package is assigned to Phase 6, not Phase
0. It shall use primitive geometry initially and expose configurable chassis,
running-gear, sensor-pose, and payload parameters. Payload shall default to
50 kg and be manually overridable before model spawn.

Final mechanical CAD is not a prerequisite for simulation. The initial model
shall deliberately use:

- a box or compound primitive for the 1.000 × 0.800 m chassis/body envelope;
- cylinders for the two nominal 0.127 m radius drive wheels;
- parameterized simplified passive casters at all four corners;
- a primitive payload body with 50 kg default mass and derived inertia;
- primitive sensor housings and configurable frames for both MRS1000 units and
  the IMU;
- Gazebo LiDAR and IMU sensor definitions connected through the validated
  LaserScan, PointCloud2, and IMU bridge types.

Unknown mechanical values shall remain named parameters with documented
provisional defaults. CAD meshes may replace visuals later without changing
the governing frames, joints, collision intent, or interfaces.

At minimum, its TF tree shall contain:

- `base_footprint`
- `base_link`
- `left_drive_wheel_link`
- `right_drive_wheel_link`
- `front_left_caster_link`
- `front_right_caster_link`
- `rear_left_caster_link`
- `rear_right_caster_link`
- `imu_link`
- `front_lidar_link`
- `rear_lidar_link`

It shall include visual and collision geometry, realistic nonzero inertial
properties, differential-drive joints, passive casters, sensor frames,
simulator integration, and joint-state publication.

## Resolved and Open Conflicts

### Resolved unloaded-mass conflict

The user confirmed approximately 30 kg unloaded. The prior 50 kg unloaded value
is superseded.

### Payload interpretation resolved

The default and initially rated simulated payload is 50 kg, giving
approximately 80 kg initial total moving mass with the nominal 30 kg unloaded
robot. Payload mass will be manually configurable before spawning a simulation
run. No live in-session adjustment mechanism is required.

The 300 kg value is retained only as an optional future simulation stress case
and physical design target. It is not an approved current simulation or
hardware rating. Structure, stability, braking, drive capability, caster
loading, payload restraint, and safety limits would require future validation.

### Resolved BOM versus frozen sensing architecture conflict

The initial Phase 0 BOM review found legacy navigation and safety sensors
outdoorScan3 safety scanners in the workbook. On 2026-07-24, the user confirmed
that these were the old plan. The current workbook now contains exactly two
MRS1000 entries and no legacy sensor entries. The MRS1000 entries now identify
MRS1104C-111011 / 1081208 and use official electrical data.

### Caster quantity

The robot requirement and BOM now specify four TENTE LEVINA 5370PJP100P62
casters. Exact dimensions, swivel geometry, individual load rating, and
four-caster load distribution remain verification items.

### Sensor naming versus placement

The handoff calls the sensors "front" and "rear" while the placement concept is
front-left and rear-right. The future frame names remain
`front_lidar_link`/`rear_lidar_link` as required, but the precise six-degree-of-
freedom poses and the intended forward/rearward scan orientations are TBD.

## Frozen Architecture Boundary

The following simulation targets and conceptual future physical selections are
treated as frozen unless a later engineering change is explicitly approved:

- Jetson Orin Nano
- simulated Siemens S7-1500F through PLCSIM Advanced
- Siemens S7-1200F as a conceptual future physical candidate only
- two SICK MRS1000 units
- Xsens MTi-8
- ZLAC8030D
- ZLTECH hub motors
- differential drive
- SLAM Toolbox plus EKF
- Nav2 plus MPC
- internal motor PID
- no outdoorScan3

Phase 0 does not redesign these selections. They do not imply current
procurement or physical deployment.

## Initial Operating and Mission Concept

- Single-robot prototype on a mostly flat indoor industrial or laboratory
  floor.
- No outdoor operation, stairs, or uncontrolled public-space operation.
- Navigate between predefined stations, stop at a target, and return to a home
  or charging area.
- Avoid operational obstacles through Nav2 costmaps.
- Recover to a defined stopped/recoverable state from canceled, blocked, failed,
  navigation, or communication conditions.
- Report mission state, health, and faults.
- Initial interaction is local manual goal submission through ROS 2 actions or
  services.
- Full fleet, WMS, and MES integration is deferred. Architecture may later
  expose REST, MQTT, OPC UA, or VDA 5050 only when a real integration requires
  it.

Still open: slope, threshold, floor-friction range, payload center of gravity,
environmental limits, docking method/accuracy, charging method, and
aisle/turning constraints.

## Full-Project Acceptance Framework

The project is accepted only when all applicable phase artifacts are complete
and the final system passes a traceable verification matrix. Numeric criteria
marked TBD cannot be finalized until the corresponding parameter is approved.

| Area | Acceptance gate |
|---|---|
| Requirements | Every approved requirement has a unique identifier, owner, verification method, and pass/fail record; no unresolved conflict is used as an implementation input. |
| BOM and hardware evidence | Every implemented hardware-dependent value traces to an exact BOM model/ordering code and an archived citation to official manufacturer documentation. |
| Build and deployment | The selected ROS 2 workspace builds reproducibly from a clean environment on the approved laptop/Ubuntu/ROS 2 baseline; automated tests pass. |
| Robot description | Xacro expands, URDF validation passes, TF is error-free, RViz display is correct, all required frames exist, inertias are realistic and nonzero, ground clearance is 0.080 m, and all six wheel/caster contact surfaces are correct. |
| Simulation mobility | The robot spawns reliably and demonstrates commanded forward, reverse, rotation-in-place, and stop behavior without chassis-ground intersection or unstable caster behavior, initially limited to 0.5 m/s and 0.4 rad/s. |
| Payload configuration | The payload defaults to 50 kg and can be manually overridden before model spawn. The selected load produces the expected total mass and consistent center-of-gravity, collision, and inertia properties. Optional higher-load stress cases require an explicitly approved test envelope. |
| Future physical motion | If a physical project is authorized, wheel direction, speed feedback, command scaling, saturation, watchdogs, and controlled stop behavior are verified at approved speed/acceleration limits. |
| State estimation | Time-synchronized wheel odometry and IMU inputs produce stable EKF output with documented frame conventions and covariance. Accuracy/drift thresholds are TBD. |
| Perception | Both MRS1000 data streams are time-aligned, correctly transformed, health-monitored, and fused into the approved mapping/costmap pipeline without blind regions that violate the approved operating concept. |
| Mapping/localization | The AMR can create, save, reload, and localize in the approved indoor test environment. Provisional stop/recovery threshold: 100 mm position error or 5° heading uncertainty, implemented from validated estimator diagnostics rather than unavailable ground truth. |
| Navigation and MPC | Normal goal accuracy is within ±50 mm and ±2°. Fixed-station repeatability target is ±30 mm. Mission success is at least 95% over an approved representative test set without manual intervention. |
| PLC and shutdown | PLC authority, heartbeat/watchdog behavior, fault reactions, contactor control, reset rules, and ROS/PLC loss-of-communication behavior are verified against an approved cause-and-effect matrix. |
| Safety | Academic review verifies the simulated control-authority and stopped-state behavior without a compliance claim. A future physical machine requires qualified risk assessment and safety-function validation. Standard perception is not credited as certified personnel protection. |
| Fault handling | Defined sensor, network, compute, motor-driver, encoder, power, and software faults transition to documented safe or controlled states and create diagnosable records. |
| Endurance | The simulation supports an 8-hour representative duty-cycle scenario. Physical per-charge endurance remains a future target dependent on confirmed battery capacity and measured power. |
| Availability | Prototype scheduled-test availability is at least 95%. Future production target is at least 98%, excluding charging and scheduled maintenance. |
| Documentation | Architecture, BOM evidence, wiring, interfaces, parameters, calibration, commissioning, operation, maintenance, test results, known limitations, and recovery procedures are current and internally consistent. |

## Phase 0 Exit Criteria

Phase 0 can be approved when:

1. this baseline and the parameter register have been reviewed;
2. the unloaded-mass conflict has been resolved and the 300 kg payload meaning
   is explicitly classified;
3. the Phase 0 decision set below has been answered or explicitly deferred with
   an owner and due phase;
4. hardware-dependent simulation values used in the current scope trace to
   exact models and official evidence; physical-only BOM details may remain
   explicitly deferred;
5. the user approves Phase 0.

## Phase 0 Decision Closure

- 50 kg is the default and initially rated simulated payload, giving
  approximately 80 kg initial total moving mass.
- Payload is manually configurable before model spawn. Live in-session
  adjustment is not required.
- 300 kg is retained only as an optional future stress case and physical design
  target, not the current rating.
- Gazebo Harmonic is the Phase 6 simulator.
- Jetson Orin Nano Developer Kit and MTi-8-5A-DK are candidate future hardware
  only; the current project is laptop-based simulation.
- BOM supplier/shop/price realignment is deferred.
- Thailand is the intended future jurisdiction, using the international safety
  references and academic-validation boundary defined above.
- Unloaded mass is 30 kg ±5 kg, covering the complete operational reference
  AMR but excluding transported payload.
- All Phase 0 decisions are closed or explicitly deferred. User approval is the
  only remaining Phase 0 gate.

## Deferred Engineering Inputs

These inputs are not Phase 0 approval blockers. They are assigned to later
simulation phases or to a future authorized physical project. They prevent
final mechanical, electrical, safety, or controller validation:

- payload envelope, restraint method, center-of-gravity range, and maximum
  center-of-gravity height;
- payload geometry, center-of-gravity, and inertia derivation for manual
  pre-spawn load configuration;
- body-height reference surfaces, `base_link` origin, chassis mass distribution,
  inertia, and collision-footprint details;
- design reverse-speed limit;
- exact caster diameter, width, swivel trail, mounting poses, load rating, and
  load distribution at the maximum approved gross mass;
- effective loaded wheel radii, wheel separation, and drive-wheel mounting
  poses;
- motor torque/speed/thermal curves, encoder interpretation, driver
  regeneration behavior, braking-energy path, and validated battery-voltage
  compatibility;
- battery capacity, mass, BMS limits/interface, charger, charging contacts, and
  low-energy behavior;
- maximum floor slope, threshold/gap height, floor-friction range, contamination
  conditions, aisle/turning limits, temperature, humidity, and ingress target;
- exact LiDAR/IMU poses, scan overlap/occlusion analysis, IP plan, firmware,
  driver versions, time synchronization, output rates, and covariance inputs;
- ROS-to-PLC protocol, heartbeat period, watchdog timing, complete state
  machine, and cause-and-effect matrix;
- required physical stopping distance by operating mode/load and the future
  risk assessment used to derive each safety function's PLr;
- MPC solver/plugin, loop rates, latency budgets, costmap clearances, and wheel
  slip policy;
- representative mission-test routes, obstacle classes, number of trials,
  confidence treatment for the 95% success target, and exact recovery-time
  limit;
- docking/charging alignment method and final docking tolerance;
- maintenance intervals, logging retention, cybersecurity, deployment,
  rollback, and software update requirements.

## Phase 0 Evidence Notes

- Initial repository inventory on 2026-07-24 found the four root project
  records and empty `src/`. The user subsequently added
  `Industrial_AMR_BOM_with_Thailand_Suppliers_Prices.xlsx`.
- The BOM was reviewed across all five sheets. Its detailed findings are
  recorded in [`PHASE_0_BOM_REVIEW.md`](PHASE_0_BOM_REVIEW.md).
- SICK's official MRS1104C-111011 / 1081208 datasheet was checked before using
  its electrical and sensing classification.
- ZLTECH's official ZLLG10ASM800 V2.0 product page was checked before deriving
  the 0.127 m nominal radius from its stated 10-inch wheel diameter.
- The local ROS/Gazebo environment was inspected read-only and documented.
- Official ISO and IEC catalogs were checked before recording the safety
  editions and their conceptual use.
- The governing handoff was synchronized with the confirmed simulation-only
  scope and 30 kg ±5 kg unloaded-mass baseline.
