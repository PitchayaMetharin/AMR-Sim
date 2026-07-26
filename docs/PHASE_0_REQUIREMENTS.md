# Phase 0 Requirements and Architecture Baseline

## Authority, scope, and status

This is the Phase 0 decision record for a simulation-only academic industrial
differential-drive AMR. It governs later work unless superseded by an approved
phase decision or current user instruction. It creates no CAD, URDF/Xacro,
ROS package, simulator asset, procurement, or physical deployment claim.

- The user owns mechanical CAD and manufacturing design. Codex owns requested
  ROS, primitive simulation, integration documentation, and validation work.
- Work is phase-gated; one phase is active at a time and the next needs explicit
  approval. Hardware values require BOM review and exact official evidence.
- **Confirmed** means selected; **provisional** remains configurable; **TBD**
  is unknown; **verification required** blocks use as a hardware fact.
- The parameter source of truth is
  [\`ROBOT_PARAMETER_REGISTER.md\`](ROBOT_PARAMETER_REGISTER.md).

## Frozen simulation baseline

| Area | Decision |
|---|---|
| Runtime | Ubuntu 22.04, ROS 2 Humble, C++17+, Fast DDS, Gazebo Harmonic, RViz |
| PLC/HMI simulation | Windows, TIA Portal V17, PLCSIM Advanced, WinCC; simulated S7-1500F |
| Future-only hardware | S7-1200F, Jetson Orin Nano, SCALANCE, ZLAC8030D, hub motors, physical sensors and wiring |
| Robot | Differential drive; ROS REP-103 (+X forward, +Y left, +Z up) |
| Body | Nominal 1.000 m × 0.800 m × about 0.600 m; 0.080 m rigid-body clearance |
| Running gear | Two drive wheels; four passive casters (FL, FR, RL, RR) |
| Wheel basis | ZLLG10ASM800 V2.0, 0.254 m nominal diameter; 0.127 m nominal radius only |
| Sensors | Two simulated SICK MRS1104C-111011 / 1081208: front-left and rear-right; simulated IMU follows Xsens MTi-8 characteristics |
| Mass/payload | 30 kg unloaded provisional ±5 kg; 50 kg default/rated simulated payload; ~80 kg initial total; 300 kg is a future stress case only |
| Algorithms | \`robot_localization\` EKF, SLAM Toolbox, Nav2 global planning/costmaps, one MPC local controller; ZLAC internal wheel-speed PID is conceptual low-level control |

The MRS1104C-111011 basis is 10–30 VDC, 13 W typical and 37 W maximum per
unit; its firmware, driver, IP plan, and bench validation remain open. Sensor
identities must remain separate throughout the system.

## Parameter and modeling rules

- Wheel radius, wheel separation, caster geometry, payload geometry/CG/inertia,
  and sensor poses are open parameters; do not silently invent values.
- Measure effective left/right rolling radii under normal load, wheel separation
  at contact centerlines and by repeated rotations, and final sensor transforms
  from approved \`base_link\` after physical mounting.
- A payload setting must produce consistent mass, CG, collision geometry, and
  inertia. A scalar-only live payload change is not required.
- User CAD is excluded. Phase 6 may create only a parameterized primitive
  simulation model, never manufacturing geometry.

## Initial software motion limits

| Quantity | Design maximum | Initial simulation/commissioning |
|---|---:|---:|
| Linear speed | 1.0 m/s | 0.5 m/s |
| Angular speed | 0.8 rad/s | 0.4 rad/s |
| Linear acceleration / deceleration | 0.4 / 0.5 m/s² | same |
| Commanded emergency deceleration | 1.0 m/s² provisional | same |
| Angular acceleration / deceleration | 0.6 / 0.8 rad/s² | same |
| Linear / angular jerk | 0.5 m/s³ / 1.0 rad/s³ | same |

These are software constraints, not hardware capability or an emergency-stop
claim. Increasing them requires stability, traction, braking, payload,
perception, controller, and stopping validation.

## Safety and control boundary

- This project makes no PL, SIL, certified-safety, physical stopping-distance,
  electrical, or deployment claim. Perception, SLAM, and Nav2 are not
  personnel-safety functions. outdoorScan3 is not in the architecture.
- A future physical machine retains E-stops, contactors, PLC supervision, and
  power shutdown. ISO 3691-4:2023 is the primary reference, with ISO 12100,
  ISO 13849-1, and IEC 60204-1 supporting; provisional PL d/Category 3 is not
  a derived PLr.
- The PLC has final drive-permission authority. ROS can request, never grant or
  override, enable. Loss of ROS heartbeat removes permission or reaches a
  defined stopped state. Reset/restart/fault recovery cannot restore motion or
  a goal automatically.
- The only permitted motion path is: mission/manual command → arbitration →
  motion constraints → PLC permission and timeout gate → simulated base
  interface. Missing, stale, malformed, contradictory, or invalid data inhibits
  motion; no node may bypass this path.

## Ownership and interfaces

| Owner | Responsibility |
|---|---|
| Gazebo | Simulated sensors, \`/clock\`, base dynamics |
| Wheel odometry + EKF | Local state and local TF; EKF publishes \`odom → base_link\` |
| SLAM Toolbox | Mapping/localization and \`map → odom\` |
| Nav2 | Global planning and costmaps |
| MPC | Sole local path tracking controller |
| OPC UA gateway | Only Ubuntu client and bridge to PLC state/request bundles |
| Simulated PLC | Safety/machine state, watchdog, drive permission, reset/fault latch and acknowledgement |

ROS/DDS is Ubuntu-only. OPC UA is the sole inter-laptop application protocol.
The simulation subnet is planned as \`192.168.50.0/24\` (Ubuntu \`.10\`, Windows
\`.20\`), without gateway, DNS, or DHCP. \`ROS_DOMAIN_ID=1\` and
\`ROS_LOCALHOST_ONLY=1\` are required; applying network, firewall, and
certificate settings is deferred to Phase 13.

## Communication and time invariants

- PLCSIM Advanced is the OPC UA server; exactly one Ubuntu gateway is client.
  Secure drive-enabled testing requires verified \`SignAndEncrypt\` /
  \`Basic256Sha256\` and mutual trust. Unsecured OPC UA is diagnosis-only and
  motion-inhibited.
- Resolve the Siemens namespace by URI and symbolic browse path under
  \`DB_AMR_OPCUA\`, never a numeric namespace index. Requests are commit-last
  bundles with sequence-correlated acknowledgement; PLC state uses a coherent
  double-read \`StateSeq\` snapshot.
- Initial values, subject to later measurement: gateway heartbeat 100 ms, PLC
  watchdog 500 ms, PLC state 100 ms, ROS state freshness 300 ms, and motion
  command expiry 200 ms.
- Gazebo time stamps robot data; Ubuntu steady time measures gateway freshness;
  PLC elapsed time drives its watchdog; UTC wall time is evidence correlation
  only.

## Deferred inputs and acceptance

Open inputs are effective wheel geometry/casters/payload/sensor poses; routes,
obstacles, docking, recovery and MPC behavior; PLC CPU/firmware/state machine/
timers/HMI; network/firewall/certificates/timing; and battery, BMS, drive,
motor, protection, thermal, and EMC evidence. ROS 2 Humble support ends May
2027; migration must be planned before then.

Phase 0 is complete when its requirement baseline, safety boundary, software
baseline, BOM review, parameter register, and simulation risk-reduction plan
are internally consistent. Evidence and detailed findings remain in
\`PHASE_0_BOM_REVIEW.md\`, \`PHASE_0_SAFETY_SCOPE.md\`,
\`PHASE_0_SOFTWARE_BASELINE.md\`, and \`PHASE_0_TEST_REPORT.md\`.
