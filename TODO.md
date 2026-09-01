# TODO

## Completed simulation scope

- [x] Parameterized Gazebo AMR model, sensors, localisation, perception, SLAM,
  Nav2 planning, MPPI path following, command arbitration, and base adapter.
- [x] Independent Gazebo native command watchdog.
- [x] Observation-only base health reporting.
- [x] Removed the former simulated-permission subsystem, interfaces, gate, and tools.
- [x] Built fourteen packages and passed 101 automated tests.

## Final scope

- [x] Exclude automatic recovery, hardware, procurement, fieldbus, and
  industrial deployment.

## Phase 14 current continuation

- [x] Accept and preserve the direct-host Phase J runtime-performance evidence.
- [x] Verify the project-owned MoveIt launch, SRDF, OMPL, controller adapters,
  authoritative composite Xacro, and exact joint/frame names.
- [x] Complete Phase K integrated readiness on the direct host: lifecycle nodes,
  arm/gripper controllers, and required actions.
- [x] Start the prescribed recorder and run exactly one Product 101, 1 kg Gate
  6 validation, stopping at the first failed gate; the run passed.
- [x] Review the retained Phase K evidence before later Gate 6 work; the
  second 1 kg boundary was attempted once and stopped at bag analysis.
- [x] Correct the documented Gate 6 recorder topic and QoS recipe from the
  pass-2 analyzer evidence.
- [x] Diagnose the retained status-to-zero forbidden-motion boundary and make
  the minimal feedback-first status-ordering correction without weakening
  acceptance criteria.
- [x] Run one post-fix direct-host readiness attempt; stop at incomplete graph
  discovery before recording or Product 101 and preserve the evidence.
- [x] Diagnose the incomplete short-lived ROS graph observer and replace it
  with one persistent, bounded, complete-and-stable graph observer.
- [x] Add the minimum controller/lifecycle-manager construction barrier and
  regression coverage for the Humble startup race.
- [x] Complete one fresh direct-host 1 kg run with the corrected recorder/QoS
  procedure; the stage and corrected required bag analyzer passed.
- [x] Establish a valid second independent 1 kg pass before any 3 kg, 5 kg,
  or Gate 7 validation.
- [x] Obtain explicit evidence review and authorization for one combined 3 kg/
  5 kg preflight attempt; it stopped before runtime at host rendering
  preflight and consumed zero Product 102/103 attempts.
- [x] Diagnose the higher-mass harness's invalid Fast DDS domain and the
  Mission Supervisor lifecycle/startup race without consuming a Product
  102/103 attempt.
- [x] Validate the one-second Mission Supervisor configure barrier in one
  fresh direct-host readiness-only run with hardware rendering and clean
  shutdown evidence.
- [x] Obtain a fresh evidence review and explicit authorization for one
  valid-domain higher-mass Product 102/103 run; `_05` exposed and `_06`
  corrected a hidden-topic harness defect before Product 102 execution.
- [x] Execute one valid-domain Product 102 attempt and stop at its first
  product failure; precise docking failed Nav2 progress before the mass stage,
  Product 103 remained unstarted, and cleanup passed.
- [x] Reconstruct the Product 102 bag timeline and classify the first causal
  failure as a precise-docking route/controller/progress-checker source defect,
  not DDS, lifecycle, rendering, ownership, or analyzer behavior.
- [x] Review and correct the precise-docking source contract so a terminal
  `STATUS_ABORTED` result inside the existing position window enters the
  registered, fail-closed retreat/relocalization/re-dock sequence while
  cancellation remains fail-closed; isolate the ROS-live control test from
  cross-package DDS traffic and pass the 276-test full workspace suite.
- [x] Execute the authorized direct-host Product 102-only retry with a new
  valid domain/run identity; stop before product startup when the base-adapter
  lifecycle response failed at the DDS/RMW boundary. Product 102/103 attempts
  remain zero and cleanup passed.
- [x] After the user reset replans to 0/2, replace the one-shot lifecycle CLI
  acceptance loop with one persistent, bounded exact-active observer; pass the
  17 focused tests, 57 package tests, and 281-test full workspace suite without
  changing production lifecycle or product behavior.
- [x] Reconstruct the `_11` lifecycle failure and classify the first causal
  planner-server response loss as a planning startup/DDS race; preserve the
  timestamped evidence and frozen Sol/high analyst packet.
- [x] Have Luna/max implement and focused-validate the single approved
  `amr_navigation` planning lifecycle construction barrier. Luna did not plan
  or replan independently; the fresh `_12` readiness gates passed.
- [x] Reconstruct the `_12` Product 102 preparation failure from the finalized
  bag and classify the first causal failure as the unhandled final precise
  recovery-dock progress-checker abort; preserve the timestamped evidence and
  frozen Sol/high analyst packet.
- [x] Have Luna/max implement and focused-validate the bounded terminal-abort
  proof for the final recovery precise dock in the product runner. Luna did
  not plan or replan independently; focused source/package checks passed.
- [x] Obtain separate authorization for one clean-host, direct-host Product
  102-only run using the persistent lifecycle preflight; `_13` stopped at its
  first product gate after the recovery-proof fix, with Product 103 unstarted.
- [x] Reconstruct `_13` and classify the first causal physical dock rejection
  as a canonical PGM/SDF pickup-pedestal map-consistency defect; preserve the
  bag correlation and frozen Sol/high packet.
- [x] Have Luna/max reconcile only the three pickup-pedestal PGM regions with
  the existing SDF grid and add focused asset-contract coverage. Luna did not
  plan or replan independently; focused source/package validation passed.
- [x] Reconstruct the `_05` Product 102 product-geometry failure, falsify the
  validator-frame hypothesis against the fixed C++ grasp target, and classify
  the first causal failure as a final-dock route/controller contract defect.
- [x] Have Luna/max implement and focused-validate the two-phase final-dock
  sequence: travel-bearing translation followed by registered terminal yaw.
  Luna did not plan or replan independently; physical/product gates and
  tolerances remain unchanged.
- [x] Execute the fresh `_06` clean-host direct-host Product 102-only
  validation; it stopped at the first pre-navigation world-control response
  failure, with Product 103 unstarted and cleanup passed.
- [x] Reconstruct `_06` and classify its first causal failure as a DDS/RMW
  service-response transport boundary after the world pause side effect, not
  a Product 102 logic failure.
- [x] Preserve the bounded Sol/high diagnostic packet and separate the
  `_01`-`_04` probe-harness/startup artifacts from Product evidence.
- [x] Complete one factory-only `control_mode:=manual` independent rclpy
  ControlWorld pause/unpause probe after Gate 6 bootstrap READY, retaining
  recorder load; the probe passed and its temporary bag assertion was corrected
  in analysis evidence.
- [x] Complete one runner-shaped high-load single-threaded rclpy
  ControlWorld pause/unpause probe; it passed with the Product runner's
  callback/status/action-client shape and no proxy response warning.
- [x] Execute the fresh `_07` clean-host direct-host Product 102-only
  validation after the control-boundary evidence; it reached the final dock,
  stopped at the first physical dock gate, and left Product 103 unstarted.
- [x] Reconstruct `_07` and correlate ground truth, raw/wheel odometry, AMCL
  TF, command settling, and final action success; classify the first causal
  failure as factory AMCL localization configuration, not DDS, runner,
  controller, map/SDF, or analyzer behavior.
- [x] Run one bounded factory-only front-lidar/map discriminator at the
  observed settled ground-truth pose; the canonical map matched the measured
  pickup-sector scan and falsified the AMCL terminal pose.
- [x] Have Luna/max apply only the frozen AMCL motion-noise packet and focused
  contract test. Luna did not plan or replan independently.
- [x] Run exactly one fresh clean-host direct-host Product 102-only validation
  after focused AMCL verification; `_08` passed readiness and stopped at its
  first mass-stage gripper proof failure. Product 103 remained unstarted.
- [x] Reconstruct `_08` and classify its first causal failure as the strict
  exact-target bilateral-position proof plus nondeterministic passive-mimic
  right-finger actuation; preserve the timestamped RCA and frozen packet.
- [x] Have Luna/max implement and focused-validate only the `_08` bilateral
  gripper packet. Luna did not plan or replan independently; focused source
  and package verification passed.
- [x] Reconstruct the `_12` pickup-frame bilateral-contact failure and classify
  it as a product/source pickup-frame geometry defect; preserve the timestamped
  RCA and frozen packet.
- [x] Have Luna/max implement and focused-validate only the `_12` pickup-frame
  geometry packet. Luna did not plan or replan independently; the focused
  build and package suite passed.
- [x] Run exactly one fresh clean-host direct-host Product 102-only validation
  after focused pickup-frame verification; `_13` advanced through pickup and
  stopped at the first retained-placement state-validity failure. Product 103
  remained unstarted.
- [x] Reconstruct `_13` and reproduce the first causal
  `arm_link_2 <-> product_camera_link` collision with a corrected offline
  MoveIt/FCL probe; classify it as a source geometry/path contract defect,
  not infrastructure or analyzer behavior.
- [ ] Obtain explicit phase authority for one compliant Product 102 center-slot
  placement correction that preserves collision checking, release/slot
  acceptance, retained-path validity, and fail-closed behavior. Do not rerun
  `_13` unchanged.
- [ ] Freeze one bounded Sol/high analyst packet, then have Luna/max implement
  only that packet. Luna must not plan or replan independently and must stop
  on any scope or hypothesis mismatch.
- [ ] Run one fresh clean-host Product 102-only validation after focused
  verification. Product 103 and Gate 7 remain blocked.
