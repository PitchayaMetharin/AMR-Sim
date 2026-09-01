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
- Attempted the next documented independent 1 kg repeatability pass once on
  the direct host. The stage reached the exact 1 kg PASS line, but the required
  bag analyzer failed on recorder topic/QoS coverage and a sampled residual
  base displacement after `MOVING` became forbidden. The failure is preserved;
  no source/configuration change, Product 101 retry, 3 kg/5 kg run, or Gate 7
  run was performed.
- Corrected the documented Gate 6 recorder recipe to include the analyzer’s
  required topics and explicit best-effort/transient-local QoS overrides; the
  pass-2 runtime evidence remains failed pending the separate motion-boundary
  review.
- Diagnosed the retained `0.000197628 m` forbidden-motion sample as a
  status-before-settling ordering defect: `gate6_mass_stage` now requires
  feedback-qualified stationary evidence before publishing `MOVING` with
  `base_motion_allowed=false`, while retaining the existing post-announcement
  guard. Added a source-contract regression check; no acceptance threshold or
  runtime tuning was changed.
- The one fresh retry after that fix stopped at host preflight because this
  execution environment exposed no readable/writable `/dev/dri/renderD*`
  device. No runtime or Product 101 stage was started, so the second 1 kg pass
  remains unestablished.
- A subsequent direct-host retry passed hardware and RTF preflight, but the
  bounded integrated ROS graph check never produced the complete required
  factory plus MoveIt node set. The run stopped before recording or Product
  101, shut down in order, and left no runtime processes. Gate 6 second-pass
  acceptance remains open; no source or tuning change was made.
- Confirmed that the changing graph snapshots were caused by fresh short-lived
  ROS 2 Humble direct observers using the 0.5 s non-daemon discovery wait, not
  by factory or MoveIt process restarts. Replaced that readiness method with a
  persistent `rclpy` observer requiring the complete unique graph to remain
  stable for 2 s within a bounded 30 s window, with regression coverage.
- Added a one-second controller-construction barrier before the Nav2 lifecycle
  manager. This fixes the observed Humble zero-delay autostart race without
  changing controller parameters, command ownership, or motion semantics.
- Corrected the Gate 6 analyzer's base-command trace check to account for the
  base adapter's independent 50 ms forwarding timer while retaining the 250 ms
  freshness and exact-value ownership checks. Regression coverage now rejects
  unowned and stale forwarding.
- Completed the one permitted fresh independent 1 kg Product 101 attempt under
  the repaired readiness gate. Host/runtime/readiness checks passed, the stage
  ended with the exact 1 kg PASS line, and the 191,936-message bag passed
  corrected analyzer reanalysis. Gate 6's second independent 1 kg pass is
  accepted; no later mass or Gate 7 runtime was started.
- Attempted the explicitly authorized combined 3 kg/5 kg execution
  `gate6_3kg_5kg_20260831_01` (`ROS_DOMAIN_ID=230`) once. Host preflight failed
  closed with `render_devices=<none>` and no forced software renderer before
  hashes, factory, MoveIt, recorder, or either Product 102/103 stage; both
  higher-mass attempt counts remain zero. Cleanup passed and post-shutdown
  found no runtime processes, but the render preflight still failed. No
  higher-mass acceptance is claimed, and an elevated rerun/retry/workaround is
  not authorized pending a fresh explicitly authorized direct-host run with a
  readable/writable approved render node.
