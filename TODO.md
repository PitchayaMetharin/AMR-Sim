# TODO.md

## Phase 0 — Requirements and Architecture
- [x] Read `AMR_CODEX_HANDOFF.md`.
- [x] Confirm repository structure.
- [x] Identify all missing robot parameters in the Phase 0 parameter register.
- [x] Record initial robot requirements and frozen architecture.
- [x] Identify and isolate the 30 kg versus 50 kg unloaded-mass conflict.
- [x] Inspect the supplied project BOM.
- [x] Document BOM architecture conflicts and data-quality defects.
- [x] Remove obsolete LMS151 and outdoorScan3 BOM content.
- [x] Replace the legacy navigation BOM and communication entries with two MRS1000 entries.
- [x] Record C++ as the primary production implementation language.
- [x] Review the official SICK MRS1000 family variants without selecting one by assumption.
- [x] Confirm ROS 2 Humble and Ubuntu 22.04.
- [x] Confirm C++17 minimum for the ROS 2 Humble baseline.
- [x] Inspect the local ROS/Nav2/MoveIt/Gazebo/compiler environment.
- [x] Confirm the initial indoor operating environment.
- [x] Record initial speed, acceleration, deceleration, and jerk limits.
- [x] Confirm approximately 30 kg unloaded mass.
- [x] Confirm 50 kg as the default and initially rated simulated payload.
- [x] Record approximately 80 kg initial total simulated moving mass.
- [x] Require a manual payload parameter before model spawn.
- [x] Classify 300 kg as an optional future stress case/design target.
- [x] Verify the 0.127 m nominal drive-wheel radius from official documentation.
- [x] Defer final MRS1000/IMU poses to the completed mechanical layout.
- [x] Define PLC/ROS responsibilities and control-authority boundary.
- [x] Confirm exact MRS1104C-111011 / 1081208 LiDAR ordering code.
- [x] Reconcile the BOM to four TENTE casters.
- [x] Defer BOM supplier/shop/price correction; no current procurement.
- [x] Confirm Gazebo Harmonic as the Phase 6 simulator.
- [x] Verify the installed Gazebo Harmonic version and run a headless smoke test.
- [x] Record missing ROS 2 Humble `ros_gz` integration packages.
- [x] Confirm Thailand jurisdiction and international safety-reference set.
- [x] Record provisional PL d, Category 3 conceptual target with no compliance claim.
- [x] Confirm academic project-team/university-supervisor validation boundary.
- [x] Confirm 30 kg ±5 kg provisional unloaded-mass estimate.
- [x] Confirm simulation-only laptop scope and defer all physical procurement/commissioning.
- [x] Confirm initial prototype missions and quantitative acceptance targets.
- [x] Defer full fleet/WMS/MES integration while keeping future-ready boundaries.
- [x] Define the full-project acceptance framework.
- [x] Define the staged simulation risk-reduction and payload-test plan.
- [x] Build and run a temporary strict C++17 ROS 2 package.
- [x] Verify Fast DDS discovery and message delivery.
- [x] Verify Gazebo topics, services, clock, statistics, and pause control.
- [x] Verify Gazebo GUI/OGRE2 and RViz OpenGL startup.
- [x] Verify SDFormat validation and analytical inertia calculation.
- [x] Inventory missing ROS/Gazebo and joint-state publisher packages.
- [x] Dry-run the explicit Humble/Harmonic integration installation.
- [x] Install `ros-humble-ros-gzharmonic`.
- [x] Install `joint_state_publisher` and its GUI package.
- [x] Verify end-to-end Gazebo-to-ROS `/clock` delivery.
- [x] Verify LaserScan, PointCloud2, and IMU bridge type support.
- [x] Verify joint-state publisher modules and executable interfaces.
- [x] Verify the `ros_gz_sim` model-spawn executable interface.
- [x] Confirm that the initial Phase 6 model will use parameterized primitives
      and does not require final CAD.
- [x] Create the Phase 0 environment and integration test report.
- [x] Create `SESSION_HANDOFF.md` for a clean new-session transfer.
- [x] Update `PROJECT_STATUS.md`.
- [x] Update `TODO.md`.
- [x] Update `CHANGELOG.md`.
- [x] Create Phase 0 requirements and parameter documentation.
- [x] Produce Phase 0 report.
- [x] Stop and await user approval.

## Phase 1 — Final System Architecture
- [x] Record Phase 0 approval and local commit.
- [x] Define the simulation-only system context and deployment boundary.
- [x] Define the layered logical architecture.
- [x] Assign each major component one responsibility and prohibited scope.
- [x] Define the single valid motion-command and authority path.
- [x] Freeze simulated PLC drive-permission and reset authority.
- [x] Define state-estimation and TF ownership.
- [x] Define independent front/rear LiDAR perception flow.
- [x] Define Nav2, MPC, mission-supervisor, and operator boundaries.
- [x] Define Gazebo `/clock` as the simulation time authority.
- [x] Define startup, readiness, shutdown, and restart ordering.
- [x] Define architectural failure-containment behavior.
- [x] Define verification and observability requirements.
- [x] Assign detailed implementation decisions to Phases 2–15.
- [x] Document explicit deferred decisions without inventing values.
- [x] Record TIA Portal V17 as the PLC/HMI engineering environment.
- [x] Identify the Windows/Ubuntu deployment conflict.
- [x] Select separate Ubuntu and Windows laptops as the deployment topology.
- [x] Select Ethernet and OPC UA for inter-laptop ROS/PLC communication.
- [x] Define provisional virtual-PLC-server and ROS-client OPC UA roles.
- [x] Resolve the PLCSIM Advanced versus S7-1200F compatibility conflict by
      selecting S7-1500F for the current simulation.
- [x] Keep S7-1200F only as a conceptual future physical candidate and document
      the required porting/revalidation boundary.
- [x] Defer exact STEP 7/Safety, PLCSIM Advanced, CPU firmware, WinCC, and HMI
      runtime selections without guessing.
- [x] Create `docs/PHASE_1_SYSTEM_ARCHITECTURE.md`.
- [x] Update `PROJECT_STATUS.md`.
- [x] Update `TODO.md`.
- [x] Update `CHANGELOG.md`.
- [x] Update `SESSION_HANDOFF.md`.
- [x] Rerun Phase 1 documentation and repository consistency checks after the
      TIA deployment decision.
- [x] Stop and await user approval.

## Phase 2 — Electrical and Power Architecture
- [x] Record explicit user authorization for Phase 2.
- [x] Read the Phase 1 architecture and governing Phase 0 evidence.
- [x] Extract and reconcile the workbook power budget without modifying
      procurement data.
- [x] Define the traction, 24 V control, 12 V compute, device-local, charging,
      and laptop-simulation power domains.
- [x] Define the conceptual source, branch-protection, service-isolation,
      precharge, contactor, converter, driver, and charging boundaries.
- [x] Preserve control-power supervision when simulated traction power is
      isolated.
- [x] Reproduce the provisional 174 W 24 V load and 217.5 W capacity check.
- [x] Reproduce the provisional 40 W 12 V load and 50 W capacity check.
- [x] Classify the 1,625 W traction value as an insufficient nameplate sum.
- [x] Record the user-confirmed 48 V, 30 Ah nominal battery rating and
      calculate its 1.44 kWh nominal energy.
- [x] Record that the 30 Ah pack does not close the eight-hour target under the
      provisional all-listed auxiliary load case before traction and reserve.
- [x] Retain 48 V, 30 Ah as the current battery baseline and record the
      user-selected 50% motor-power runtime case at approximately 1 hour
      22 minutes.
- [x] Verify current official input/output ratings for the ZLAC8030D,
      DDR-240C-24, DDR-60L-12, Blue Sea 6006, and EV200AAANA candidates.
- [x] Block battery/driver/disconnect compatibility until the exact pack
      voltage window is known.
- [x] Define precharge, independent K1/K2 feedback, regeneration, charging,
      low-energy, branch-protection, and fault-response requirements.
- [x] Define logical electrical states and simulated electrical signals without
      creating Phase 3 transport or Phase 12 PLC implementation details.
- [x] Create `docs/PHASE_2_ELECTRICAL_POWER_ARCHITECTURE.md`.
- [x] Update `docs/ROBOT_PARAMETER_REGISTER.md`.
- [x] Update `PROJECT_STATUS.md`.
- [x] Update `TODO.md`.
- [x] Update `CHANGELOG.md`.
- [x] Update `SESSION_HANDOFF.md`.
- [x] Run Phase 2 documentation and repository consistency checks.
- [x] Stop and await user approval.
- [x] Record Phase 2 approval and create the local Phase 2 commit.

## Phase 3 — Communication Architecture
- [x] Record explicit user authorization for Phase 3.
- [x] Read the Phase 2 and Phase 1 architectures and governing Phase 0
      evidence in the required order.
- [x] Preserve the pre-existing uncommitted `AMR_CODEX_HANDOFF.md` edit without
      incorporating or overwriting it.
- [x] Separate the Ubuntu ROS 2/DDS plane from the Windows OPC UA authority
      plane.
- [x] Define the closed, static-address, non-applied two-laptop Ethernet plan.
- [x] Define OPC UA endpoint roles, security gate, certificate trust, namespace
      resolution, symbolic schema, data ownership, and quality rules.
- [x] Assign transport representations to all Phase 2 electrical signals.
- [x] Define coherent PLC snapshots and commit-last ROS request bundles.
- [x] Define sequence-correlated acknowledgement and replay/restart behavior.
- [x] Define initial heartbeat, watchdog, freshness, command timeout, and
      reconnect values.
- [x] Separate Gazebo simulation time, Ubuntu steady time, PLC elapsed time,
      and UTC evidence time.
- [x] Define canonical ROS namespaces, topic/action classes, QoS profiles, and
      unique publisher ownership.
- [x] Define communication fault responses, diagnostics, verification tests,
      and later-phase ownership.
- [x] Record that the user owns Phase 12 ladder implementation and Codex
      supplies the guide, mappings, cause/effect guidance, tests, and review.
- [x] Create `docs/PHASE_3_COMMUNICATION_ARCHITECTURE.md`.
- [x] Update `docs/ROBOT_PARAMETER_REGISTER.md`.
- [x] Update `PROJECT_STATUS.md`.
- [x] Update `TODO.md`.
- [x] Update `CHANGELOG.md`.
- [x] Update `SESSION_HANDOFF.md`.
- [x] Run Phase 3 documentation and repository consistency checks.
- [x] Stop and await user approval.
- [x] Record Phase 3 approval and create the local Phase 3 commit.

## Deferred Inputs — Later Simulation Phases or Future Physical Project
- [ ] Define the design reverse-speed limit before controller configuration.
- [x] Define the simulated ROS/PLC transport, heartbeat period, acknowledgement,
      and reconnect architecture before PLC integration.
- [ ] Define the detailed PLC state machine, timers, latches, reset rules, and
      cause/effect matrix before PLC integration.
- [ ] Prepare the Phase 12 ladder-programming guide, OPC UA/tag mapping,
      cause/effect guidance, and PLC/HMI test checklist for the user.
- [ ] User implements the Phase 12 ladder program in TIA Portal; Codex reviews
      it when requested.
- [x] Define network addressing, ROS 2 QoS, and simulation time architecture
      before sensor integration.
- [ ] Verify the planned subnet, interfaces, firewall, certificates, and secure
      OPC UA capabilities on the exact Windows/Ubuntu toolchain.
- [ ] Define representative acceptance routes, obstacle classes, trial count,
      and recovery-time limit before validation.
- [ ] Define deployment support and migration plan before ROS 2 Humble EOL.
- [ ] Measure/calibrate effective rolling radii and wheel separation if a
      physical project is authorized and assembled.
- [ ] Define payload envelope, restraint, center-of-gravity range, and height
      for a future physical implementation.
- [ ] Define payload geometry, center-of-gravity, and inertia derivation for the
      Phase 6 manual pre-spawn payload parameter.
- [ ] Define physical stopping-distance requirements by load and operating mode
      through a future risk assessment and test program.
- [ ] Identify the exact battery/BMS and verify its complete voltage, current,
      fault-current, energy, thermal, protection, and communication envelope
      before any future physical electrical design.
- [ ] Resolve the ZLAC8030D and service-disconnect compatibility gate against
      the exact battery voltage and regenerative-transient range.
- [ ] Obtain the exact driver DC-link, current-limit, enable, protection, and
      regenerative-braking evidence before sizing precharge or protection.
- [ ] Define the mission duty-cycle power profile, usable-energy reserve, and
      low-energy/shutdown thresholds before accepting the 30 Ah capacity
      against the eight-hour endurance target.
- [ ] Complete source and branch protection coordination, conductor/connector
      sizing, contactor/feedback selection, charging design, grounding/bonding,
      EMC, surge, and thermal design only if a physical project is authorized.

## Future Phases
- [x] Phase 1 — Final system architecture
- [x] Phase 2 — Electrical and power architecture
- [ ] Phase 3 — Communication architecture
- [ ] Phase 4 — ROS 2 workspace and package structure
- [ ] Phase 5 — Hardware drivers and interfaces
- [ ] Phase 6 — Robot model and simulation
- [ ] Phase 7 — Odometry and EKF
- [ ] Phase 8 — Dual-MRS1000 perception pipeline
- [ ] Phase 9 — SLAM and localization
- [ ] Phase 10 — Nav2 global planning and costmaps
- [ ] Phase 11 — MPC controller implementation
- [ ] Phase 12 — PLC and shutdown integration
- [ ] Phase 13 — Full-system integration
- [ ] Phase 14 — Verification and validation
- [ ] Phase 15 — Final documentation and handoff

## Rules
- Do not start the next phase without user approval.
- Update this file at the end of every phase.
- Add newly discovered tasks instead of keeping them only in chat.
