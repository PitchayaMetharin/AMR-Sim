# Simulation Risk-Reduction and Test Plan

## Purpose

This plan defines what can be tested in the current simulation-only project,
what evidence each test can provide, and what conclusions remain prohibited.
It does not create a robot model or begin a later phase.

## Payload Baseline

- Default and initially rated simulated payload: 50 kg.
- Nominal unloaded robot mass: 30 kg, with a provisional ±5 kg tolerance.
- Nominal initial total simulated moving mass: approximately 80 kg.
- Payload will be a manual Xacro or launch parameter selected before model
  spawn.
- A 300 kg payload is an optional future simulation stress case, not the
  current rating.
- Any payload selection must use consistent mass, center of gravity, collision
  geometry, and inertia.

## Testing Possible During Phase 0

The following risk-reduction work is valid now:

1. Verify operating-system, ROS 2, compiler, simulator, and package versions.
2. Start Gazebo Harmonic with a manufacturer-supplied empty world to verify the
   simulator server and physics initialization.
3. Identify missing integration packages before implementation.
4. Maintain a traceable parameter register so unknown geometry, inertia,
   contact, timing, and sensor values cannot silently become assumptions.
5. Define test stages, evidence requirements, and stop criteria before the
   robot model is created.
6. Calculate idealized motion bounds for test planning, clearly separated from
   validated stopping performance.

Completed Phase 0 evidence:

- Gazebo Harmonic 8.14.0 was detected.
- The installed `empty.sdf` loaded and initialized its 1 ms physics profile in
  a headless server smoke test.
- Gazebo transport topics/services and world pause control passed.
- Gazebo GUI/OGRE2 and RViz OpenGL 4.6 startup passed.
- A strict C++17 `ament_cmake` package built and ran successfully.
- Fast DDS delivered all 6 of 6 messages in an isolated talker/listener test.
- Gazebo's SDFormat validator and automatic-inertia calculation matched a
  known analytical box fixture.
- The explicit ROS 2 Humble/Harmonic bridge and joint-state publisher packages
  were installed.
- End-to-end Gazebo-to-ROS `/clock` delivery passed.
- LaserScan, PointCloud2, and IMU bridge type mappings passed.

Detailed evidence is recorded in
[`PHASE_0_TEST_REPORT.md`](PHASE_0_TEST_REPORT.md).

## Testing Not Yet Meaningful

The following tests require the parameterized robot model and therefore cannot
be claimed during Phase 0:

- static settling and chassis ground-clearance verification;
- drive-wheel and four-caster contact behavior;
- payload center-of-gravity and tip/stability response;
- acceleration, braking, jerk, traction, and wheel-slip behavior;
- differential-drive odometry and wheel-separation calibration;
- IMU and dual-LiDAR timing, transforms, noise, and occlusion;
- Nav2, SLAM, EKF, MPC, watchdog, and mission recovery performance;
- 50 kg versus higher-load comparative dynamics.

## Staged Test Strategy

### Environment gate

Before robot integration:

- install and version-pin the approved ROS 2 Humble/Gazebo Harmonic integration;
- verify `/clock`, service, topic, and message bridging;
- verify headless and GUI startup;
- record the physics engine, step size, real-time update rate, and solver
  configuration;
- create a reproducible environment check.

### Robot-description gate

During Phase 6:

- validate Xacro expansion and URDF-to-SDF conversion;
- reject missing, zero, negative, non-finite, or physically invalid inertias;
- verify the default 50 kg payload produces approximately 80 kg total mass;
- verify manual payload overrides produce consistent inertial properties;
- verify TF, ground clearance, collision geometry, wheel contact, and caster
  contact;
- spawn, settle, pause, reset, and respawn repeatedly without numerical
  instability.

### Low-speed dynamics gate

Start with the 50 kg payload and the existing commissioning limits:

- straight motion in both directions;
- rotation in place;
- low-speed stop and command-timeout behavior;
- caster swivel and oscillation behavior;
- wheel slip and odometry consistency;
- controller saturation and rate-limit behavior.

No higher speed or higher payload proceeds until the preceding configuration
has passed its documented tests.

### Payload escalation gate

If higher-load simulation is later requested:

- approve the payload geometry, center of gravity, inertia, and test envelope;
- increase load in documented stages rather than jumping directly to 300 kg;
- repeat static settling, low-speed motion, turning, braking, caster, slip, and
  controller tests at every stage;
- compare results against the 50 kg baseline;
- stop escalation on instability, collision penetration, loss of control,
  unacceptable slip, invalid solver behavior, or failed acceptance criteria.

A successful 300 kg simulation would demonstrate only that the selected
mathematical model remained stable under that scenario. It would not establish
real-world equipment or personnel-safety performance.

### Navigation and fault gate

Later phases shall add:

- odometry and EKF accuracy tests;
- LiDAR/IMU noise, delay, dropout, and transform-fault injection;
- blocked-path, canceled-goal, localization-loss, and communication-loss tests;
- Nav2 and MPC constraint, recovery, and mission-success testing;
- command freshness, plant-watchdog, and stopped-state tests.

## Idealized Stopping-Distance Bounds

Ignoring command latency, jerk, slope, slip, and controller dynamics, the
constant-deceleration lower bound is:

`distance = speed² / (2 × deceleration)`

| Speed | Deceleration | Ideal lower-bound distance |
|---:|---:|---:|
| 0.5 m/s | 0.5 m/s² normal | 0.25 m |
| 0.5 m/s | 1.0 m/s² provisional commanded | 0.125 m |
| 1.0 m/s | 0.5 m/s² normal | 1.00 m |
| 1.0 m/s | 1.0 m/s² provisional commanded | 0.50 m |

These values are planning bounds, not acceptance results. Jerk limits,
software and communication latency, contact friction, payload motion, slope,
and actuator saturation will increase actual stopping distance. The
1.0 m/s² value is a controlled-command target and must not be described as
guaranteed emergency-stop performance.

## Current Highest-Priority Risk Reductions

1. Preserve and version-pin the validated explicit ROS 2 Humble/Gazebo
   Harmonic integration.
2. Obtain or define wheel separation, caster geometry, wheel/caster poses, and
   payload geometry/center-of-gravity assumptions before dynamic modeling.
3. Use the 50 kg payload as the first dynamics baseline.
4. Keep the initial 0.5 m/s and 0.4 rad/s commissioning limits.
5. Require automated mass, inertia, TF, ground-contact, spawn, and motion tests
   before navigation tuning.
6. Keep every real-world equipment, safety, and 300 kg capability claim
   outside simulation evidence.
