# AMR Session Handoff

## Current authoritative state — 2026-09-04

The active task is a readiness-only runtime proof of the Fast DDS
service-response reliability correction. The source/configuration change is
implemented, statically validated, and has passed read-only review. Runtime
evidence is encouraging but incomplete because the latest run was stopped
before MoveIt and the final readiness gates.

This file is intentionally limited to current state and next actions. Historical
investigation remains available in Git history and must not be appended here.

## Fast DDS diagnosis and implemented correction

The observed defect was an intermittent `rmw_fastrtps` service-response reader
discovery race during factory startup. Service replies timed out approximately
107–114 ms after controller configuration. The installed Fast DDS 2.6.12
default publisher `max_blocking_time` is 100 ms, making this a middleware
response-discovery boundary rather than a lifecycle threshold, controller,
safety, or public ROS API defect.

The approved change is limited to:

- `src/amr_factory/config/fastdds_service_profiles.xml`: one non-default
  `<publisher profile_name="service">` with
  `reliability/max_blocking_time/sec = 1`;
- `src/amr_factory/launch/factory_localization.launch.py`: sets the absolute
  installed profile through `FASTRTPS_DEFAULT_PROFILES_FILE` before Gazebo,
  bridges, the robot, and deferred child actions, while preserving
  `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`; and
- `src/amr_factory/test/test_factory_assets.py`: focused XML, installed-path,
  environment-key, ordering, and negative contract coverage.

Do not replace the working `FASTRTPS_DEFAULT_PROFILES_FILE` spelling with
`FASTDDS_DEFAULT_PROFILES_FILE`: the latter was directly proven to be ignored
by this installed library. Do not set `RMW_FASTRTPS_USE_QOS_FROM_XML` or
override reliability kind, publication mode, history/memory, data sharing, or
default profiles.

Static validation already passed:

- XML syntax validation;
- 28 focused factory/MPC contract tests;
- `amr_factory` and `amr_mpc_controller` build;
- installed/source XML checksum equality;
- an installed Fast DDS probe showing `max_blocking_time=1.0 s` with reliable
  service QoS preserved; and
- `git diff --check HEAD`.

## Fresh runtime evidence

### `_01` — previous environment blocker

`.ros_logs/fastdds_service_match_20260904_01/`, domain 209, stopped at host
preflight because `/dev/dri/renderD*` was not visible. Gazebo did not start.
Do not reuse this identity/domain pair for evidence.

### `_02` — hardware rendering available, partial readiness proof

`.ros_logs/fastdds_service_match_20260904_02/`, domain 210, used strict GUI
hardware rendering with `/dev/dri/renderD128` and the prescribed initial pose
`(2.4, 3.0, 0.0)`.

Observed results:

- host preflight passed: render node readable/writable, no forced software
  renderer, no stale runtime processes;
- Gazebo server and GUI started and used the DRM device;
- bootstrap reached READY and all four ros2_control controllers activated;
- map/AMCL, planner/smoother, and controller lifecycle startup progressed
  without the prior service-response timeout;
- no genuine `failed to send response ... client will not receive response`,
  `rmw_response.cpp:154`, or `rcl/service.c:314` warning was found in the run;
- runtime capture recorded 2,915 samples over 11.9906 s, median RTF
  `0.9984307662`, and aggregate RTF `0.8100800247`; and
- shutdown completed with no remaining runtime processes; post-shutdown host
  preflight passed.

The checked-in runtime preflight still enforces RTF `>= 0.90`, so it reported
FAIL and the run stopped before MoveIt, graph, lifecycle-stability, action, and
controller-manager readiness checks. The user subsequently authorized RTF
`>= 0.80` as acceptable for now. Under that explicit temporary criterion, the
recorded `_02` median and aggregate RTF pass. Do not change the checked-in
preflight threshold merely to reflect this session-specific acceptance.

The partial `_02` startup supports the Fast DDS diagnosis and correction but
does not complete the intermittent runtime proof. One clean partial startup
without the warning is not final acceptance.

### `_03` — setup inspection only

The setup helper was sourced with
`fastdds_service_match_20260904_03`/domain 212 only to inspect preflight CLI
arguments. No host preflight, Gazebo, MoveIt, or readiness proof ran. The run
directory now exists, so do not reuse `_03` as an evidence identity.

## Exact next action

Run one fresh readiness-only proof with a new valid identity/domain; suggested
pair: `fastdds_service_match_20260904_04` and domain 213, after proving the
domain is empty. Use the same pair in every terminal through
`/home/pete/amr_sim_setup.sh` and follow
`/home/pete/SIMULATION_COMMANDS_QUICK.md` in this order:

1. stale-process/domain check;
2. host preflight;
3. one factory launch with `headless:=false`,
   `software_rendering:=false`, `require_hardware_rendering:=true`,
   `factory_attachment:=true`, and initial pose `(2.4, 3.0, 0.0)`;
4. runtime RTF capture, evaluated against the temporarily authorized `0.80`
   median and aggregate threshold without editing the script;
5. MoveIt startup;
6. persistent graph and lifecycle checks, including the required two-second
   active stability window; and
7. action and controller-manager readiness checks.

Require every non-RTF readiness gate to pass, required actions/controllers to
be present and active, no new Fast DDS/RMW service-response warning, and a
clean shutdown/process scan. Stop at the first failed mandatory gate. Do not
patch source directly from a runtime failure; return the evidence to Sol/high
for diagnosis first.

Do not start a recorder, product runner, Product 101/102/103, or Gate 7 during
this readiness-only proof.

## Broader phase state and protected invariants

Product 102 (3 kg) remains accepted from
`.ros_logs/gate6_product102_geometry_20260902_03/`. Product 103 (5 kg) and
Gate 7 remain pending and are outside this readiness-only activity. Do not
rerun Product 101 or Product 102 merely for progression.

Preserve fail-closed behavior, lifecycle barriers, command ownership, public
interfaces, collision and placement gates, safety thresholds, and documented
hardware values unless the user explicitly authorizes a specific change.
Existing controller and Mission Supervisor startup barriers remain in place.

The worktree intentionally contains the pre-existing `.gitignore` change,
this concise `SESSION_HANDOFF.md`, and the three Fast DDS implementation paths
listed above. Runtime `.ros_logs/` evidence remains local. No commit, push,
dependency installation, system change, or external change was performed.
`AMR_CODEX_HANDOFF.md` was not modified and remains protected.
