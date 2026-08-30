# Changelog

## Unreleased

### Changed

- Removed the former simulated-permission subsystem, its interfaces, gate, and
  acceptance tools.
- The simulation command route is Nav2 MPPI → command arbitration → base
  adapter → Gazebo plant, protected by adapter and native plant watchdogs.
- The workspace now contains fourteen ROS 2 packages and 101 automated tests.
- Recorded the direct-host Phase J runtime-performance PASS for
  `gate6_1kg_retained_20260830_01` (median RTF `1.0000144002`, aggregate RTF
  `0.9999999293`) without changing source or performance settings.
- Verified the existing project-owned MoveIt configuration against the
  authoritative composite robot description and completed a bounded MoveIt
  server smoke; integrated factory readiness remains pending direct-host
  validation.
- Completed Phase K on the direct Ubuntu host: integrated lifecycle,
  controller, action, service, topic, command-ownership, and MoveGroup checks
  passed, and the single recorded Product 101 run ended with
  `GATE 6 1.0 KG COMPLETE 1 KG PASS`.
- Preserved the 200,534-message Product 101 evidence bag and diagnosed the
  initial stale-process/lifecycle-discovery boundaries without changing source
  or rerunning the closed Phase J performance gate.
