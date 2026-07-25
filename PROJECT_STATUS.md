# PROJECT_STATUS.md

## Project
Industrial Differential-Drive AMR

## Current Phase
None

## Status
Phase 3 communication architecture is approved, complete, and locally
committed as `9dd6d18`. Phase 4 workspace structure and interfaces are
approved, complete, and clean-build validated with 14 passing tests. Phase 5
is not authorized.

## Current Project Scope
- Simulation-only academic AMR project.
- Runs across an Ubuntu ROS/Gazebo laptop and a Windows TIA/PLC/HMI simulation
  laptop.
- Robot, LiDAR, IMU, PLC authority, drive, safety-state, and vehicle behavior
  are simulated.
- No physical hardware will be purchased, installed, commissioned, or certified.

## Conceptual Future Hardware
- Jetson Orin Nano
- Siemens S7-1200F PLC
- 2 × SICK MRS1104C-111011 / 1081208 LiDAR
- Xsens MTi-8 IMU
- ZLAC8030D dual-axis servo driver
- ZLTECH hub motors
- User-supplied 48 V, 30 Ah LiFePO4 battery system
- Siemens SCALANCE managed Ethernet switch

## Frozen Software and Algorithms
- ROS 2 Humble on Ubuntu 22.04
- C++17 minimum
- Differential-drive kinematics
- SLAM Toolbox
- robot_localization EKF
- Nav2 global planning and costmaps
- MPC local controller
- ZLAC8030D internal wheel-speed PID
- S7-1500F virtual PLC through PLCSIM Advanced V4.0 provisional target
- OPC UA over Ethernet between the Windows PLC-simulation laptop and Ubuntu ROS
  laptop

## Safety Direction
- SICK outdoorScan3 removed.
- No safety LiDAR in the current architecture.
- Two MRS1000 units provide perception for SLAM, obstacle detection, and Nav2 costmaps.
- Keep physical E-stop, contactors, and PLC-based power shutdown.
- This architecture must not be represented as safety-certified solely through SLAM or standard LiDAR.

## Scope Exclusions
- Mechanical CAD is designed by the user.
- Codex must not create or redesign CAD unless explicitly requested.
- Physical procurement, construction, wiring, commissioning, and certification
  are outside the current project.

## Completed Work
- Initial project architecture defined.
- Controller changed from Pure Pursuit to MPC.
- Navigation LiDAR changed to 2 × SICK MRS1000.
- outdoorScan3 removed.
- Phase-gated Codex workflow defined.
- Repository structure inventoried.
- Initial robot requirements and frozen architecture recorded.
- Comprehensive robot parameter register created.
- Unloaded-mass conflict identified and isolated from implementation.
- Full-project acceptance framework drafted.
- Supplied five-sheet BOM reviewed for architecture alignment and data quality.
- BOM conflicts and supplier-field misalignment documented without modifying
  unrelated procurement entries.
- Obsolete sensing-plan BOM content replaced by exactly two MRS1000 entries;
  dependent unverified values initially marked TBD.
- C++ confirmed as the primary production implementation language.
- Exact MRS1104C-111011 / 1081208 variant and official electrical data verified.
- Nominal 0.127 m drive-wheel radius derived from the official 10-inch motor
  specification and distinguished from calibrated rolling radius.
- Four TENTE casters confirmed.
- Initial mass, payload, motion, mission, PLC/ROS boundary, endurance, and
  acceptance targets recorded.
- Local Ubuntu/ROS/Nav2/MoveIt/Gazebo/compiler environment inspected.
- Gazebo Harmonic selected for Phase 6.
- Gazebo Harmonic 8.14.0 installation verified with successful headless,
  transport/control, and GUI tests.
- Project scope frozen to laptop-based simulation only; Jetson and MTi-8
  classified as candidate future hardware.
- Default and initially rated simulated payload restored to 50 kg, with
  approximately 80 kg initial total moving mass.
- Payload will be manually configurable before model spawn; live in-session
  adjustment is not required.
- Unloaded mass closed at 30 kg ±5 kg.
- Thailand/international safety-reference scope and academic-validation
  boundary documented.
- Staged simulation risk-reduction plan documented, including current test
  limits and later payload-escalation gates.
- Phase 0 environment tests passed for C++17/ament build and runtime, Fast DDS
  delivery, Gazebo transport/control and GUI, RViz rendering, and SDFormat
  inertia calculation.
- `ros-humble-ros-gzharmonic` and joint-state publisher packages installed.
- End-to-end Gazebo-to-ROS `/clock` delivery and LaserScan, PointCloud2, and
  IMU bridge mappings verified.
- Basic parameterized primitive-shape robot model confirmed as the Phase 6
  path; final mechanical CAD is not required.
- Root-level `SESSION_HANDOFF.md` prepared with the complete transfer state,
  validation evidence, Git status, deferred inputs, and next permitted action.
- Phase 0 approved and locally committed as `7db85f7`.
- Final logical system architecture documented with explicit component,
  authority, command, estimation, perception, mission, lifecycle, failure, and
  deployment boundaries.
- Single motion-command path frozen through arbitration, motion constraints,
  simulated PLC permission, timeout handling, and the simulated base interface.
- Gazebo simulation time, TF ownership, dual-LiDAR identity, one-MPC-controller
  policy, startup/shutdown sequence, and fail-inhibited behavior defined.
- Detailed electrical, communication, package, adapter, simulation, EKF,
  perception, SLAM, Nav2, MPC, PLC, integration, and validation decisions
  assigned to their governing later phases.
- TIA Portal V17 recorded as the required PLC/HMI engineering environment.
- The Windows-only TIA toolchain conflict with the Ubuntu ROS/Gazebo host
  identified without assuming a VM, second computer, PLCSIM product, or WinCC
  edition.
- Two-laptop deployment frozen: Ubuntu for ROS 2/Gazebo/RViz/development and
  Windows for TIA Portal V17/PLC/HMI simulation.
- Ethernet with an OPC UA virtual-PLC-server/ROS-client contract selected for
  inter-laptop communication.
- S7-1500F selected as the current simulated PLC family, resolving the PLCSIM
  Advanced/OPC UA compatibility conflict.
- S7-1200F retained only as a conceptual future physical BOM candidate, with no
  equivalence or automatic portability claim.
- Phase 1 approved and locally committed as `8be2e8b`.
- Conceptual traction, 24 V control, 12 V compute, device-local, charging, and
  laptop-simulation power domains defined without creating a physical wiring
  design.
- Control-power persistence separated from traction isolation so simulated PLC
  supervision and diagnostics can remain available after propulsion is
  inhibited.
- Provisional 24 V load arithmetic reconciled at 174 W, 217.5 W with the
  workbook's 25% allowance, and 22.5 W residual headroom on the 240 W candidate
  supply.
- Provisional 12 V load arithmetic reconciled at 40 W, 50 W with the 25%
  allowance, and 10 W residual headroom on the 60 W candidate supply.
- The 1,625 W traction figure classified as a nameplate sum that cannot size
  the battery, BMS, protection, conductors, contactors, or endurance.
- User-confirmed battery rating recorded as 48 V, 30 Ah, giving 1.44 kWh
  nominal stored energy.
- Eight-hour endurance remains open: the provisional all-listed auxiliary load
  case requires at least 1.712 kWh at the loads and approximately 1.881 kWh
  using typical converter efficiencies, before traction and reserve.
- The 48 V, 30 Ah battery retained as the current capacity baseline. Using the
  user-selected 50% combined motor-nameplate assumption, 25 W driver allowance,
  and approximately 235 W auxiliary source load gives a provisional 1.36-hour
  runtime estimate (approximately 1 hour 22 minutes).
- Official input ranges verified for the DDR-240C-24 and DDR-60L-12 converter
  candidates.
- A blocking battery-voltage compatibility conflict identified: the unresolved
  full voltage window of the 48 V nominal pack cannot be approved with the
  ZLAC8030D 48 V maximum input or Blue Sea 6006 48 V maximum rating.
- Precharge, independent K1/K2 command/feedback, charging interlock,
  regenerative-energy handling, low-energy behavior, and branch-protection
  requirements defined without guessing component values.
- Logical electrical energy states, simulated electrical signals, ownership,
  default-inhibited behavior, and fault responses defined.
- Phase 2 electrical and power architecture documented in
  `docs/PHASE_2_ELECTRICAL_POWER_ARCHITECTURE.md`.
- Phase 2 approved by the user, closed, and locally committed as `9e64d41`.
- ROS 2/DDS traffic confined to the Ubuntu host and OPC UA selected as the sole
  inter-laptop application protocol.
- Closed `192.168.50.0/24` simulation-network plan defined without applying
  host configuration; Ubuntu `.10`, Windows `.20`, no gateway/DNS/DHCP.
- Fast DDS and `ROS_DOMAIN_ID=1` retained for the single-host ROS graph, with
  `ROS_LOCALHOST_ONLY=1` required in the later deployment configuration.
- Secure OPC UA endpoint, application-certificate, namespace-resolution,
  symbolic-schema, ownership, quality, and coherent-snapshot rules defined.
- Commit-last ROS-to-PLC request bundles and sequence-correlated PLC
  acknowledgements defined so pulse loss and replay cannot silently grant
  authority.
- Initial 100 ms gateway heartbeat, 500 ms PLC watchdog, 300 ms PLC-state
  freshness, and 200 ms stamped-command timeout defined as provisional
  simulation values requiring later measurement.
- Gazebo simulation time, Ubuntu steady time, PLC elapsed time, and UTC
  evidence time assigned distinct responsibilities.
- Canonical ROS namespaces, interface classes, QoS profiles, unique publishers,
  reconnect behavior, communication fault responses, and verification plan
  documented in `docs/PHASE_3_COMMUNICATION_ARCHITECTURE.md`.
- User ownership of Phase 12 ladder programming recorded. Codex will provide
  the implementation guide, OPC UA/tag mapping, state-machine and cause/effect
  guidance, test checklist, and review support rather than authoring the
  ladder program unless explicitly requested later.

## Work in Progress
- None. Phase 4 is complete.

## Next Required Action
Await explicit user authorization for Phase 5. Do not create Phase 5 driver or
gateway implementation artifacts while waiting.

## Deferred Parameters
- MRS1000 firmware and ROS 2 driver/configuration; physical sensor IP and time
  synchronization remain future-project inputs
- Effective rolling radius and wheel separation
- Caster geometry and all wheel/caster mounting poses
- Payload envelope and center-of-gravity range
- Payload geometry/inertia derivation and Phase 6 manual configuration
- Numerical poses and orientations for both MRS1000 units and the MTi-8
- Detailed operating floor, slope, threshold, and environmental limits
- MPC implementation approach
- Detailed PLC/ROS cause-and-effect, transitions, timers, latches, and state
  machine
- Applied network-interface, firewall, certificate, and OPC UA configuration
- Representative acceptance test matrix and docking accuracy/method
- TIA Portal V17 update level
- STEP 7 edition, Safety license/option, PLCSIM Advanced V4.0 update, and exact
  S7-1500F simulated model/firmware
- WinCC V17 edition, panel-image version, and HMI runtime/simulator
- Exact OPC UA security/user-token capabilities, endpoint path, revised
  subscription intervals, certificates, and trust stores for the installed
  V17/PLCSIM/CPU combination
- Exact battery/BMS model, voltage window, current limits, fault current,
  usable portion of the confirmed 1.44 kWh nominal energy, reserve, and
  interface
- ZLAC8030D revision/manual, DC-link capacitance, input/transient limits,
  current limits, regeneration, enable, and fault behavior
- Exact motor electrical, torque, speed, current, encoder, efficiency, and
  thermal data
- Final source/branch protection, disconnect, conductor, connector, contactor,
  precharge, grounding, bonding, shielding, isolation, surge, EMC, and thermal
  design
- Charger, charging contacts, charge profile/current, pilot/interlock, protocol,
  and docking sequence
- Mission duty-cycle power profile and low-energy/shutdown-reserve thresholds

## Known Risks
- Standard MRS1000-based perception and SLAM are not substitutes for certified functional-safety sensing.
- MPC integration with Nav2 requires careful interface, timing, and constraint design.
- Dual-LiDAR calibration and time synchronization will affect mapping and obstacle detection.
- Missing mechanical parameters will block accurate kinematics and MPC tuning.
- A 300 kg payload is only an optional future simulation stress case and
  physical design target. It does not establish structure, stability, braking,
  drive, caster, or safety capability.
- Shifted supplier/shop/price fields make part of the BOM procurement data and
  its subtotal unreliable.
- Gazebo Classic 11 remains installed but is not the project baseline. The
  explicit Humble/Harmonic bridge is installed; installing the conflicting
  generic `ros-humble-ros-gz` package would break this package selection.
- Actual dual-LiDAR and IMU streams remain untested until the primitive robot
  model and sensors are implemented.
- Gazebo GUI startup emits a non-fatal QML world-statistics binding-loop warning.
- ROS 2 Humble is supported only through May 2027; the project needs a
  deployment-support and later migration plan.
- Detailed environmental limits remain undefined.
- Simulation cannot demonstrate physical functional-safety performance,
  stopping distance, structural capacity, traction, or certified compliance.
- The Ubuntu laptop must later be performance-tested with Gazebo, two simulated
  LiDAR streams, SLAM, Nav2, MPC, visualization, and evidence collection active.
- Phase 3 communication timing values are initial simulation bounds and require
  Phase 13 latency/jitter/load validation; they are not safety response times.
- TIA Portal V17 runs on a separate Windows laptop. The direct Ethernet path,
  secure OPC UA capability, certificates, reconnect behavior, and cross-host
  timing remain untested on the exact installed toolchain.
- The planned `192.168.50.0/24` subnet and interface bindings must be checked
  for collision before any configuration is applied.
- A future physical S7-1200F implementation would require explicit PLC-program,
  I/O, OPC UA, timing, fail-safe behavior, and validation porting from the
  S7-1500F simulation; simulation results are not automatically transferable.
- The unresolved battery may exceed the verified 48 V maximum of the
  ZLAC8030D and Blue Sea 6006 candidates; the conceptual traction path is
  blocked from physical approval.
- The current 24 V and 12 V budgets use several unverified conceptual BOM load
  values and do not close derating, simultaneity, wiring-loss, inrush, or
  peripheral-load checks.
- Regenerative energy, available fault current, protection coordination,
  precharge energy, and physical emergency-stop behavior remain unknown.
- The confirmed 30 Ah capacity does not close the 8-hour target under the
  provisional all-listed auxiliary load case, even before traction demand;
  actual duty-cycle loads and usable battery energy remain unmeasured.

## Last Updated
2026-07-25 — Phase 4 approved after independent clean-build validation: two
packages, 14 tests, installed interfaces, and launch smoke test passed. Phase 5
is not authorized.
