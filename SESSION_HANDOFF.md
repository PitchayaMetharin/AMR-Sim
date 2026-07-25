# AMR Session Handoff

## Authority and current gate

1. The user's current instruction and phase authorization override this file.
2. This file is the complete startup authority for the active phase. Do not
   read earlier phase documents before starting unless a specific detail is
   genuinely absent here or the user requests an audit/history review.
3. At every phase handoff, write only the executable task packet for the then
   current phase. Reduce completed and future phases to compact status/roadmap
   entries; do not make prior documents a startup requirement. Keep this file
   at 300 lines or fewer.
4. Active phase: **None**.
5. Last completed phase: **Phase 4 — ROS 2 workspace and package structure**.
   It is approved, clean-build validated, and committed at repository HEAD for
   this handoff.
6. Exact next action: wait for explicit user authorization for Phase 5. Do not
   create Phase 5 artifacts while waiting.

## Protected workspace and workflow

- Before any edit, run `git status --short` and preserve unrelated work.
- `AMR_CODEX_HANDOFF.md` has a protected, pre-existing user edit. Never modify,
  normalize, stage, discard, or commit it without explicit user direction. Its
  S7-1200F/50 kg wording is legacy material and not implementation authority.
- `SESSION_HANDOFF.md` was intentionally created/edited for handoff management;
  retain it and update it at the end of every approved phase.
- No Git push, force-push, history rewrite, or external change without explicit
  user instruction. Make a descriptive local commit only after user approval of
  the completed phase.
- Work one numbered phase at a time. At phase end: update `PROJECT_STATUS.md`,
  `TODO.md`, `CHANGELOG.md`, and this file; validate; report using Summary,
  Files created, Files modified, Design decisions, Risks, Questions, Next
  phase, Awaiting approval; then wait for approval.
- The user owns all mechanical CAD. Do not create URDF/Xacro before Phase 6.
- Use verified project or official manufacturer evidence for hardware values;
  never guess or silently substitute them.
- Use `karpathy-guidelines` before software development/engineering design and
  `debug-mantra` whenever debugging a failure. Use subagents only when useful,
  with at most three temporary subagents.

## Frozen project baseline

- Simulation-only academic industrial AMR. Ubuntu 22.04 runs ROS 2 Humble,
  C++17+, Fast DDS, Gazebo Harmonic, and RViz. Windows runs TIA Portal V17,
  PLCSIM Advanced, and HMI simulation. The simulated PLC family is S7-1500F;
  S7-1200F is a future conceptual candidate requiring porting/revalidation.
- Differential drive: two hub wheels, four passive casters; nominal body
  1.000 m x 0.800 m x about 0.600 m, 0.080 m clearance, 30 kg unloaded
  (provisional +/-5 kg), 50 kg default/rated simulated payload, about 80 kg
  initial total mass, and 0.127 m nominal wheel radius. Effective radius,
  wheel separation, caster geometry, payload geometry/CG/inertia, and sensor
  poses remain open.
- Two simulated SICK MRS1104C-111011 LiDARs (1081208): front-left and
  rear-right. Simulated IMU follows Xsens MTi-8 characteristics. Front and
  rear sensor identities must remain separate.
- SLAM Toolbox owns global mapping/localization; `robot_localization` EKF owns
  fused local state; Nav2 owns global planning/costmaps; one MPC controller
  owns local path tracking; ZLAC8030D internal wheel-speed PID is conceptual
  low-level control.
- The only motion path is mission/manual command -> arbitration -> motion
  constraints -> PLC permission and timeout gate -> simulated base interface.
  No node may bypass it. Missing, stale, malformed, contradictory, or invalid
  data inhibits motion. Restart/reconnect/fault/shutdown never restores motion
  permission or a goal automatically.
- Time authority: Gazebo stamps simulated robot data; Ubuntu steady time is for
  gateway freshness; PLC elapsed time is for its watchdog; UTC wall time is only
  evidence correlation. Wheel odometry/EKF owns local TF; SLAM owns `map->odom`.
- DDS is Ubuntu-only. OPC UA is the only cross-laptop application protocol.
  `ROS_DOMAIN_ID=1` and `ROS_LOCALHOST_ONLY=1` are required. The planned closed
  subnet is 192.168.50.0/24 (Ubuntu .10, Windows .20), with no gateway, DNS, or
  DHCP; applying network/firewall/certificate settings belongs to Phase 13.
- PLCSIM Advanced is the OPC UA server and exactly one Ubuntu ROS gateway is
  client. Drive-enabled use requires verified secure SignAndEncrypt/
  Basic256Sha256 and certificate trust. Unsecured OPC UA is diagnosis-only with
  motion inhibited. Resolve the Siemens namespace by URI and symbolic browse
  path, never numeric index. Root: `DB_AMR_OPCUA`.
- ROS requests are commit-last bundles. PLC state requires coherent double-read
  `StateSeq`; requests require sequence-correlated acknowledgement. Initial
  limits: gateway heartbeat 100 ms, PLC watchdog 500 ms, PLC state 100 ms, ROS
  state freshness 300 ms, motion-command expiry 200 ms. These require later
  measurement.
- Battery baseline is conceptual 48 V, 30 Ah LiFePO4 (1.44 kWh nominal); its
  evidence is incomplete. No physical electrical/safety/certification claim is
  permitted. Perception, SLAM, and Nav2 are not personnel-safety functions.

## Current step

- Wait for explicit Phase 5 authorization.
- Phase 4 created only `amr_interfaces` and `amr_bringup`; the clean isolated
  ROS 2 Humble/C++17 build, all 14 tests, installed interface resolution,
  no-node launch smoke test, environment settings, formatting, XML, and
  whitespace checks passed.
- Phase 5 will own simulated base/sensor adapter and OPC UA gateway APIs and
  implementations. It must preserve the fail-closed interfaces, canonical
  ownership, process isolation, and no-bypass motion path already recorded in
  this handoff and `src/README.md`.
- Phase 5 must not create the Phase 6 robot model, later estimation/navigation/
  MPC behavior, or Phase 12 PLC/HMI/ladder program.

## Roadmap and deferred inputs

- Completed: Phase 0 requirements (`7db85f7`), Phase 1 architecture
  (`8be2e8b`), Phase 2 electrical/power (`9e64d41`), Phase 3 communication
  (`9dd6d18`), and Phase 4 workspace/interfaces (handoff HEAD). Next: Phase 5
  drivers/interfaces; 6 model/sim; 7 odometry/EKF; 8 perception; 9 SLAM; 10
  Nav2; 11 MPC; 12 PLC/shutdown (user writes ladder); 13 integration; 14
  validation; 15 final handoff.
- Open later inputs: effective wheel geometry/casters/payload/sensor poses;
  reverse speed/MPC/routes/obstacles/docking/recovery; PLC CPU/firmware/tool
  versions/state-machine/timers/HMI; subnet collision/adapters/firewall/
  certificate endpoint/timing; battery/BMS/drive/motor/protection/thermal/EMC
  evidence. ROS 2 Humble support ends May 2027; plan migration before then.
