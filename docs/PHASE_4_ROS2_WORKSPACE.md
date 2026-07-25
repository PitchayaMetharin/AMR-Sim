# Phase 4 — ROS 2 workspace and package structure

## Result

Phase 4 creates a buildable ROS 2 Humble/C++17 workspace with two packages:
`amr_interfaces` owns fail-closed shared contracts, and `amr_bringup` owns
environment, QoS, launch, and structural validation. Later-phase package names
and exact executables, public I/O, lifecycle policy, process isolation, and
forbidden responsibilities are reserved in `src/README.md`; they are not
created or implemented here.

## Interface rules

All structured status/authority messages carry a header, sequence, validity,
source boot ID, state, and reason. Permission-bearing messages additionally
carry raw permission. Zero values map to UNKNOWN/UNAVAILABLE and false
permission, so default construction cannot express readiness or permission.
Gateway request services confirm tracked delivery acceptance only; PLC state is
the authoritative outcome. Compiled C++ tests instantiate every message,
request, and response to verify those defaults in generated code.

## Runtime configuration

`amr_bringup` sets `ROS_DOMAIN_ID=1` and `ROS_LOCALHOST_ONLY=1`, names `/amr`
as the robot namespace, and establishes simulation-time defaults. Its launch
file starts no runtime nodes: node implementation remains with owner phases.
QoS profiles encode the approved depth, reliability, durability, deadline, and
lifespan values. `interface_ownership.yaml` records one publisher or action
server for each Phase 4 canonical boundary and preserves independent front/rear
LiDAR ownership.

## Validation

Validation was rerun from a clean isolated build/install tree on 2026-07-25:

- both ROS 2 Humble packages built successfully with C++17;
- `colcon test-result --verbose` reported 14 tests, 0 errors, 0 failures;
- three compiled C++ tests proved fail-closed generated defaults;
- the fresh overlay resolved both packages and displayed the installed
  `PlcState` interface;
- `amr_system.launch.py` completed its no-node Phase 4 smoke run;
- domain/local-only environment settings, QoS values, canonical authority
  ownership, independent LiDAR identities, package scope, and Python formatting
  passed;
- installed launch resources contained no generated Python cache files.

No hardware/simulator driver, robot model, estimator, perception, navigation,
controller, OPC UA client, or PLC/HMI logic was introduced.
