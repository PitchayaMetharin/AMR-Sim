# Phase 1 Final System Architecture

## Scope and governing decisions

Phase 1 defines the simulation architecture, responsibilities, authority,
process boundaries, startup, and failure behavior. It does not implement ROS
packages, URDF/Xacro, PLC logic, physical networking, physical devices, or
safety certification. The Phase 0 baseline governs all unresolved hardware
values.

| Runtime boundary | Decision |
|---|---|
| Ubuntu laptop | ROS 2 Humble, Gazebo Harmonic, RViz, estimation, perception, SLAM, Nav2, MPC, motion gate, mission supervision, OPC UA gateway |
| Windows laptop | TIA Portal V17, PLCSIM Advanced simulated S7-1500F OPC UA server, WinCC HMI simulation |
| Between laptops | One closed Ethernet link; OPC UA only |
| Within Ubuntu | Fast DDS; \`ROS_DOMAIN_ID=1\`, \`ROS_LOCALHOST_ONLY=1\` |
| Physical hardware | Future conceptual reference only; no fieldbus, safety protocol, wiring, or hardware claim |

## Architecture invariants

1. One owner writes each authoritative state, TF edge, OPC UA field, and motion
   command interface.
2. The sole motion route is mission/manual intent → arbitration → constraints →
   PLC-permission/timeout gate → simulated base interface. Any bad, stale,
   absent, contradictory, or unacknowledged input inhibits motion.
3. PLC permission is necessary but never sufficient: constraints, command
   freshness, lifecycle health, and simulation health also pass.
4. Reconnect, restart, reset, lifecycle transition, shutdown, or fault clearing
   never restores drive permission, a command, or a navigation goal.
5. No safety-rated, deterministic, physical-stop, or production-network claim
   is made by the simulation.

## Logical architecture

\`\`\`text
Gazebo (/clock, sensors, base) → adapters → odometry/EKF → local TF/state
                                  ↘ perception → SLAM → map→odom
Mission/manual → arbitration → constraints → motion gate → base adapter → Gazebo
                                         ↑       ↕
                              Nav2 global plan → MPC
Ubuntu OPC UA gateway ↔ PLCSIM Advanced / simulated PLC ↔ HMI
\`\`\`

| Component | Sole responsibility |
|---|---|
| Gazebo adapters | Convert simulator-specific base/sensor I/O to project interfaces; no authority decisions |
| Wheel odometry | Wheel-derived local motion estimate; no \`map → odom\` |
| \`robot_localization\` EKF | Fused local state and \`odom → base_link\`; consumes valid odometry/IMU only |
| SLAM Toolbox | Mapping/localization and \`map → odom\` only |
| Perception | Separate front/rear LiDAR handling, obstacle products, diagnostics |
| Nav2 | Global planning and costmaps; no direct base output |
| MPC | Only local path tracking controller; emits constrained motion intent |
| Arbitration/constraints | Select legal mission/manual source and apply speed, acceleration, jerk, geometry and freshness policy |
| Motion gate | Requires valid source, constraints, PLC snapshot/permission, timeout and system health before base command |
| OPC UA gateway | Sole Ubuntu OPC UA client; validates schema, bundles requests, publishes coherent PLC state |
| Simulated PLC | Machine/safety state, watchdog, permissives, reset/fault latches, acknowledgements |
| HMI | Observes PLC authority and sends only approved operator requests through PLC logic |

## TF, data, and time contract

- \`map → odom\` has exactly one owner: SLAM Toolbox. \`odom → base_link\` has
  exactly one owner: EKF/wheel-odometry chain. Static transforms connect
  \`base_link\` to sensors; simulator plugins do not duplicate authoritative TF.
- Preserve front and rear LiDAR identities; fusion must retain provenance and
  quality. Invalid sensor data is excluded and reported, never fabricated.
- Gazebo \`/clock\` timestamps simulated robot data. Ubuntu steady time measures
  process/gateway freshness. PLC elapsed time is its watchdog source. UTC wall
  time correlates evidence only.

## Control, lifecycle, and failure containment

Startup is non-permissive: establish Gazebo clock and adapters; validate
estimation/perception; connect and schema-validate OPC UA; obtain a coherent
PLC snapshot; activate navigation/controller only when dependencies are ready;
then require a new, acknowledged operator request before motion. Shutdown and
all dependency loss revoke motion first, then stop/deactivate dependent nodes.

| Failure | Required behavior |
|---|---|
| Gazebo, \`/clock\`, base adapter, EKF, sensor, or controller invalid/stale | Inhibit motion; preserve diagnostic evidence |
| OPC UA disconnect/schema/quality/snapshot failure | Gateway publishes non-permissive state; gate inhibits; reconnect is observation-only |
| PLC watchdog, E-stop, safety input, power, or fault latch | PLC removes permission; base gate commands/holds stop according to valid state |
| Nav/MPC/goal failure | Cancel or hold motion; do not route around the gate |
| Process restart | Boot identity changes; all old requests/acks/goals invalid until a fresh sequence |

## Deployment and observability

Run simulator, adapters, estimation, perception, SLAM, Nav2/MPC, gateway, and
supervision as separately observable processes/lifecycle units on Ubuntu;
keep PLC/HMI in the Windows simulation. Record lifecycle state, command source,
constraint result, expiry, PLC snapshot sequence/quality, permission reason,
fault/reset/ack sequence, TF source, sensor provenance, and timing metrics.
Configuration is versioned, parameterized, validated at startup, and never
contains private keys or secrets.

## Deferred ownership

| Later phase | Owns |
|---|---|
| 4 | Workspace, packages, typed interfaces and launch skeleton |
| 5 | Gazebo base/sensor adapters and OPC UA gateway implementation |
| 6 | Primitive robot model and Gazebo world/plugins |
| 7–11 | Odometry/EKF, perception, SLAM, Nav2, and MPC behavior/tuning |
| 12 | User-authored PLC/HMI state machine, ladders, and shutdown behavior |
| 13–14 | Network/security configuration, end-to-end integration and validation |

Open decisions include final geometry/inertia and sensor poses, sensor drivers,
PLC CPU/firmware/address space, secure endpoint support, motion recovery and
mission semantics, MPC implementation, and all physical electrical/safety
evidence. Phase 1 is complete as an architecture record; Phase 2/3 refine the
power and communication contracts without weakening these invariants.
