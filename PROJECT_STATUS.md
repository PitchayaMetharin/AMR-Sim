# PROJECT_STATUS.md

## Project
Industrial Differential-Drive AMR

## Current Phase
Phase 0 — Requirements and architecture confirmation

## Status
Not started in Codex yet.

## Frozen Hardware
- Jetson Orin Nano
- Siemens S7-1200F PLC
- 2 × SICK MRS1000 LiDAR
- Xsens MTi-8 IMU
- ZLAC8030D dual-axis servo driver
- ZLTECH hub motors
- 48 V LiFePO4 battery system
- Siemens SCALANCE managed Ethernet switch

## Frozen Software and Algorithms
- ROS 2
- Differential-drive kinematics
- SLAM Toolbox
- robot_localization EKF
- Nav2 global planning and costmaps
- MPC local controller
- ZLAC8030D internal wheel-speed PID

## Safety Direction
- SICK outdoorScan3 removed.
- No safety LiDAR in the current architecture.
- Two MRS1000 units provide perception for SLAM, obstacle detection, and Nav2 costmaps.
- Keep physical E-stop, contactors, and PLC-based power shutdown.
- This architecture must not be represented as safety-certified solely through SLAM or standard LiDAR.

## Scope Exclusions
- Mechanical CAD is designed by the user.
- Codex must not create or redesign CAD unless explicitly requested.

## Completed Work
- Initial project architecture defined.
- Controller changed from Pure Pursuit to MPC.
- Navigation LiDAR changed to 2 × SICK MRS1000.
- outdoorScan3 removed.
- Phase-gated Codex workflow defined.

## Work in Progress
- None.

## Next Required Action
Codex must review the handoff and complete Phase 0 only.

## Open Decisions
- ROS 2 distribution and Ubuntu version
- Exact MRS1000 interface and driver package
- Robot wheel radius and wheel separation
- Maximum speed and acceleration limits
- Final robot mass and payload
- MPC implementation approach
- PLC's final responsibilities
- Simulation platform and robot model availability

## Known Risks
- Standard MRS1000-based perception and SLAM are not substitutes for certified functional-safety sensing.
- MPC integration with Nav2 requires careful interface, timing, and constraint design.
- Dual-LiDAR calibration and time synchronization will affect mapping and obstacle detection.
- Missing mechanical parameters will block accurate kinematics and MPC tuning.

## Last Updated
Initial repository template.
