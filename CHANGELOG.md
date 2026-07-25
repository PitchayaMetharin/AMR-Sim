# CHANGELOG.md

All significant project changes must be recorded here.

## Unreleased

### Added
- Phase 4 ROS 2 Humble/C++17 workspace skeleton with `amr_interfaces` and
  `amr_bringup`, plus reserved later-phase package boundaries.
- Fail-closed base, motion-gate, PLC-state, and PLC-connection interfaces;
  gateway motion-enable/reset delivery services; localhost-only DDS and QoS
  configuration; machine-readable authority ownership; compiled C++ default
  tests; and headless structural tests.
- Exact future package/executable, public-I/O, lifecycle, composition, and
  forbidden-responsibility assignments without later-phase implementation.
- Clean isolated Phase 4 validation evidence: two packages built, 14 tests
  passed, installed interfaces resolved, launch smoke test passed, and
  generated Python caches excluded from installation.
- Phase 4 workspace architecture record and compact, current-phase-only
  session-handoff rule.
- Phase-gated engineering workflow.
- Requirement to stop after every phase and await approval.
- `PROJECT_STATUS.md`, `CHANGELOG.md`, and `TODO.md` repository records.
- MPC selected as the local path-tracking controller.
- Dual SICK MRS1000 perception architecture.
- Phase 0 requirements and architecture baseline.
- Comprehensive robot parameter register covering geometry, mass, running gear,
  sensors, motion limits, control, compute, PLC, safety, networking, power,
  environment, and validation targets.
- Full-project acceptance framework and hardware evidence gate.
- Phase 0 BOM quality and architecture-alignment review.
- C++ selected as the primary production implementation language.
- Two SICK MRS1000 lines added to the BOM and subsequently resolved to the exact
  MRS1104C-111011 / 1081208 ordering code and official electrical values.
- Initial payload, motion, PLC/ROS responsibility, mission, endurance,
  availability, and navigation-accuracy requirements.
- Phase 0 software baseline with inspected workstation evidence.
- Simulation-only safety scope and official standards reference set.
- Staged simulation risk-reduction plan separating current environment checks,
  later robot-model tests, optional payload escalation, and prohibited physical
  safety claims.
- Phase 0 environment and integration test report covering host capacity,
  C++17/ament, ROS 2/Fast DDS, Gazebo transport/control/GUI, RViz rendering,
  SDFormat inertia, required packages, and remaining blockers.
- Root-level new-session handoff containing phase state, frozen requirements,
  installed environment, validation evidence, deferred inputs, and exact next
  actions.
- Phase 1 final system architecture defining logical layers, component
  responsibilities, deployment, authority, data flow, lifecycle, and
  failure-containment boundaries.
- A single non-bypassable motion path through command arbitration, motion
  constraints, simulated PLC permission, timeout handling, and the simulated
  base interface.
- Unique time and TF ownership, independent front/rear LiDAR identity,
  verification observability, and later-phase architecture ownership.
- TIA Portal V17 selected as the PLC and HMI engineering environment, with
  STEP 7/Safety, S7-PLCSIM, WinCC, firmware, licensing, and runtime details
  explicitly tracked.
- Two-laptop deployment selected: Ubuntu for ROS 2/Gazebo/RViz/development and
  Windows for TIA Portal V17, PLC simulation, and HMI simulation.
- Ethernet and OPC UA selected for the inter-laptop ROS/PLC interface, with the
  virtual PLC as server and ROS 2 gateway as provisional client.
- S7-1500F selected as the current simulated PLC family for PLCSIM Advanced and
  OPC UA integration.
- Phase 2 electrical and power architecture defining conceptual power domains,
  source and branch boundaries, traction isolation, control-power persistence,
  charging, energy states, simulated electrical signals, fault responses, and
  later-phase ownership.
- Reproducible provisional 24 V and 12 V load calculations with explicit
  evidence limits and future verification gates.
- User-confirmed 48 V, 30 Ah nominal battery rating, its 1.44 kWh nominal
  energy, and the resulting eight-hour endurance check.
- Logical battery/BMS, rail-valid, precharge, K1/K2 command/feedback, traction
  bus, driver-fault, charger, electrical-state, and inhibition-reason signals
  for later transport and PLC implementation.
- Phase 3 communication architecture separating the Ubuntu ROS 2/DDS plane
  from the Windows OPC UA authority plane.
- Closed static-address two-laptop simulation-network plan with explicit
  isolation, firewall, and no-routing requirements.
- Symbolic `DB_AMR_OPCUA` interface contract with one writer per direction,
  namespace-URI resolution, type/access/quality validation, commit-last
  request bundles, coherent PLC snapshots, and sequence acknowledgements.
- Initial ROS/PLC heartbeat, watchdog, state-freshness, command-expiry, and
  reconnect timing contract.
- Canonical ROS namespaces and QoS classes for sensors, state, authority,
  commands, diagnostics, dynamic TF, and static TF.
- Communication fault, observability, clock-domain, HMI-boundary, security,
  and later verification requirements.

### Changed
- Phase 4 approved after clean C++17/Humble build, 14 passing tests, installed
  interface verification, and launch/configuration smoke validation.
- Project-agent role changed from teaching assistant to lead robotics engineer.
- Navigation LiDAR changed from SICK LMS151 to 2 × SICK MRS1000.
- Local controller changed from Regulated Pure Pursuit to MPC.
- Mechanical CAD removed from Codex scope.
- Project status advanced to Phase 0 review, with unresolved values explicitly
  prevented from becoming implementation assumptions.
- BOM, power-budget, communication-matrix, and design-note content updated to
  remove the obsolete LMS151/outdoorScan3 sensing plan.
- LiDAR selection frozen to 2 × SICK MRS1104C-111011, order number 1081208.
- Software baseline frozen to ROS 2 Humble, Ubuntu 22.04, and C++17 minimum.
- Unloaded mass resolved to approximately 30 kg.
- Caster quantity changed from two to four TENTE LEVINA 5370PJP100P62 units.
- Provisional nominal drive-wheel radius set to 0.127 m from the verified
  manufacturer-stated 10-inch diameter.
- Initial fleet integration deferred; local single-robot mission interfaces
  retained with future integration boundaries.
- Current project scope changed to laptop-based simulation only; all listed
  physical hardware is conceptual future-reference equipment.
- Gazebo Harmonic selected as the Phase 6 simulator.
- Default and initially rated simulated payload confirmed as 50 kg, with
  approximately 80 kg initial total moving mass.
- Payload retained as a manual Xacro/launch parameter before model spawn;
  live in-session adjustment is not required.
- A 300 kg payload retained only as an optional future simulation stress case
  and physical design target, not the current rating.
- Payload configuration required to maintain consistent mass,
  center-of-gravity, collision, and inertia properties.
- Gazebo Harmonic 8.14.0 installation verified through a headless empty-world
  smoke test; the explicit Humble/Harmonic integration was subsequently
  installed and validated.
- C++17/ament build and runtime, Fast DDS 6/6 delivery, Gazebo world control,
  Gazebo GUI/OGRE2, RViz OpenGL 4.6, and analytical inertia checks passed.
- Exact `ros-humble-ros-gzharmonic` installation path dry-run completed with
  no removals or upgrades; joint-state publisher package gaps recorded.
- Installed the explicit Humble/Harmonic integration and joint-state publisher
  packages after user authorization.
- Verified end-to-end Gazebo-to-ROS `/clock` delivery plus LaserScan,
  PointCloud2, and IMU bridge mappings.
- Confirmed that Phase 6 will use parameterized primitive geometry and does not
  depend on completed mechanical CAD.
- Unloaded mass tolerance set to ±5 kg around the 30 kg nominal value.
- Thailand selected as the future jurisdiction, with ISO 3691-4:2023,
  ISO 12100:2010, ISO 13849-1:2023, and IEC 60204-1:2016+A1:2021 as conceptual
  international references.
- PL d, Category 3 recorded only as a provisional conceptual target; no
  compliance claim is made.
- Phase 0 approved and locally committed as `7db85f7`; active work advanced to
  Phase 1.
- Gazebo fixed as the simulation-time authority, wheel odometry plus IMU/EKF as
  the local-state path, and SLAM Toolbox as the global `map -> odom` authority.
- Nav2 plus a single MPC controller fixed as the navigation and local-control
  path; no automatic alternate-controller fallback is part of the baseline.
- Simulated PLC authority and fail-inhibited motion behavior refined without
  defining the detailed Phase 3/12 protocol or state machine prematurely.
- Phase 1 returned to architecture revision after the TIA Portal V17
  requirement exposed an unresolved Windows-hosting conflict with the Ubuntu
  ROS/Gazebo deployment.
- The Windows-hosting conflict was closed by the two-laptop decision.
- PLCSIM Advanced versus the frozen S7-1200F target recorded as a blocking
  compatibility conflict; no PLC family was silently substituted.
- PLCSIM Advanced compatibility conflict resolved by explicit user selection
  of S7-1500F for simulation.
- S7-1200F reclassified as a conceptual future physical BOM candidate only;
  no automatic equivalence or portability from the S7-1500F simulation is
  claimed.
- Phase 1 final system architecture approved and closed.
- Phase 1 locally committed as `8be2e8b` and the new-session handoff refreshed
  to the clean post-phase state.
- Phase 2 authorized and prepared for review; Phase 3 remains unauthorized.
- Traction, 24 V control, 12 V compute, device-local, and charging power
  domains separated, with control and compute supervision retained when
  simulated propulsion power is isolated.
- The workbook's 174 W 24 V load now has an explicit 217.5 W result after its
  25% allowance and 22.5 W residual headroom on the 240 W candidate converter.
- The workbook's 40 W 12 V load now has an explicit 50 W result after its 25%
  allowance and 10 W residual headroom on the 60 W candidate converter.
- The 1,625 W traction figure reclassified as a nameplate sum that cannot size
  the battery, BMS, fuse, conductors, contactors, precharge, or endurance.
- Battery-to-ZLAC8030D and battery-to-Blue-Sea-6006 compatibility blocked:
  the pack is confirmed as 48 V nominal but its maximum charged/transient
  voltage remains unknown, while both candidates have verified 48 V maximum
  limits.
- The provisional 80–100 A fuse and 100 ohm/100 W precharge entries prevented
  from becoming design values until battery, fault-current, driver-capacitance,
  regeneration, and timing evidence is available.
- The battery capacity changed from TBD to 30 Ah nominal. The eight-hour target
  remains open because the provisional all-listed auxiliary load case requires
  at least 1.712 kWh at the loads and approximately 1.881 kWh using typical
  converter efficiencies, before traction, reserve, aging, and distribution
  loss.
- The 48 V, 30 Ah battery retained as the current capacity baseline. A
  user-selected planning case using 50% of combined motor nameplate power,
  25 W driver allowance, and approximately 235 W auxiliary input gives a
  provisional 1.36-hour runtime estimate (approximately 1 hour 22 minutes).
- Stale single-laptop wording reconciled with the approved Ubuntu ROS/Gazebo
  plus Windows TIA/PLC/HMI two-laptop topology.
- Phase 2 electrical and power architecture approved, closed, and locally
  committed as `9e64d41`.
- Phase 3 explicitly authorized and prepared for review; Phase 4 remains
  unauthorized.
- Fast DDS with `ROS_DOMAIN_ID=1` retained for the Ubuntu-only ROS graph;
  `ROS_LOCALHOST_ONLY=1` is required in the later deployment configuration so
  DDS is not extended to the Windows PLC/HMI laptop.
- Inter-laptop addressing planned as closed `192.168.50.0/24` with Ubuntu
  `.10`, Windows `.20`, and no gateway/DNS/DHCP, pending collision and
  interface verification before application.
- Drive-enabled OPC UA integration requires a verified secure endpoint,
  preferred `SignAndEncrypt`/`Basic256Sha256`, and explicit certificate trust.
  Unsecured OPC UA is limited to motion-inhibited diagnostics on the closed
  simulation network.
- Siemens OPC UA nodes will be found by namespace URI and symbolic browse path
  rather than a fixed numeric namespace index.
- Initial gateway heartbeat set to 100 ms, PLC watchdog to 500 ms, PLC-state
  freshness at ROS to 300 ms, and stamped command expiry to 200 ms; all remain
  simulation values requiring later latency/jitter validation.
- Gazebo time retained for robot data while steady/PLC elapsed clocks govern
  communication freshness and UTC wall time governs evidence correlation.
- Phase 3 communication architecture approved and locally committed as
  `9dd6d18`.
- Phase 12 ladder-program implementation assigned to the user. Codex will
  supply the ladder guide, tag/interface mapping, state-machine and
  cause/effect guidance, test checklist, and review support unless the user
  later explicitly requests implementation.

### Removed
- SICK outdoorScan3 safety LiDAR.
- PROFIsafe field-switching requirements tied specifically to outdoorScan3.
- Teaching-mode behavior.

### Safety Note
- SLAM, Nav2 costmaps, and standard LiDAR obstacle detection must not be described as a certified personnel-safety system.

### Resolved Requirement Conflict
- The current default and initially rated simulated payload is 50 kg, giving
  approximately 80 kg total moving mass at nominal unloaded mass. The user may
  manually override the payload before later simulation runs. A 300 kg case is
  optional and unvalidated.

### Verification Note
- The current BOM still contains shifted supplier/shop/price fields in later
  rows. Correction is deferred because there is no current procurement.
