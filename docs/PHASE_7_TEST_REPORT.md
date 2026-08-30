# Phase 7 Test Report

## Result

Phase 7 local odometry and state-estimation implementation passed its static,
integration, and nominal-simulation trajectory gates on 2026-07-28. These
results apply only to the provisional Gazebo model; they are not physical
calibration, localization-performance, or safety evidence.

## Implementation evidence

- `wheel_odometry_node` lifecycle-activates and derives planar odometry from
  the named left/right drive-joint positions using the Phase 6 provisional
  0.127 m radius and 0.680 m separation.
- The wheel node publishes `/amr/localization/wheel_odometry` without TF.
- `robot_localization` runs a 2D EKF at 30 Hz, fusing wheel longitudinal and
  lateral velocity, wheel yaw rate, IMU relative yaw, and IMU yaw rate.
- The EKF alone owns `/amr/localization/odometry` and
  `odom -> base_footprint`; no map or global-localization transform exists.
- Both the wheel node and EKF reset their local state on backward simulation
  time jumps.
- The nominal wheel pose/twist covariance values are simulation tuning only:
  0.0025 for planar translation and 0.0012 for yaw.

## Validation evidence

- All eight packages built with `colcon build --symlink-install`.
- The complete suite reported 45 tests, zero errors, zero failures, and zero
  skipped tests. `git diff --check` passed.
- Live launch confirmed active wheel odometry, EKF input parameters, filtered
  odometry, 30 Hz diagnostics, and `odom -> base_footprint`.
- A backward-time reset was detected and cleared the EKF TF buffer. Gazebo's
  joint-state publisher does not resume after a world or time-only reset, so
  post-reset data recovery still requires the Phase 6 model-respawn procedure.
- The reusable `localization_acceptance` check compares filtered odometry with
  Gazebo's independent physical model pose. Its limits are 0.01 m/rad static
  drift, 0.03 m trajectory position error, and 0.04 rad trajectory yaw error.
- The final nominal run reported:
  - static drift: 0.0000 m and 0.0000 rad;
  - 0.5000 m straight travel: 0.0000 m and 0.0000 rad error;
  - 0.6401 rad in-place turn: 0.0000 m and 0.0000 rad error.

## Plant correction found by the gate

The first quantitative turn test exposed that DiffDrive odometry was not an
independent ground-truth source: it reported rotation while the chassis slid.
Gazebo model pose and IMU evidence traced this to passive-caster contact
friction and the drive-wheel axis sign. Passive caster-wheel contact friction
was reduced to 0.01, the drive-wheel axis sign was corrected, and an
independent Gazebo model-pose topic was added. The acceptance test then passed
against actual chassis motion.

The private simulation command topic is used only by this acceptance fixture.
Phase 7 does not add teleoperation, arbitration, permission, or motion-gate
behavior.
