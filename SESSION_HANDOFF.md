# AMR Session Handoff

## Current authoritative state — `_bootstrapfix_01` 1 kg regression and pause — 2026-09-01

The user requested a pause for host shutdown after the first fresh current-source
1 kg attempt. Gate 6 is **incomplete** and must not be reported as complete.
The accepted empty-motion run is
`.ros_logs/gate6_empty_motion_20260901_05/`; it passed the exact empty-motion
marker, recorder finalization, runtime/readiness gates, cleanup, process scan,
and post-shutdown host gate.

The fresh native-attachment Product 101 run is
`.ros_logs/gate6_1kg_bootstrapfix_20260901_01/`, using ROS domain 215 and the
current mass-stage source hash
`33dab99b63c81ec8e4bcaf9c8aff4ed0d057fe264b1524407aaaa8760995c5fd` with
executable hash
`5e03503dbf8e49480908d148ff9cc81b44621e96f6ce6e1250156a0315d9d2a0`.
Host/rendering, runtime RTF, graph/lifecycle/controller/action/service/topic,
MoveIt/OMPL, bootstrap, recorder, and ownership gates passed. The stage
started exactly one Product 101 attempt and reached grasp, native attachment,
transport, dispatch alignment, pre-place execution, and exact attached
planning-scene payload proof.

The first causal failure was the retained 1 kg placement validity check:
MoveIt reported `arm_link_4 <-> base_link` at `retained placement segment 52
sample 1`. The stage then failed closed with
`GATE 6 1.0 KG: FAIL: payload-aware state validity failed at retained
placement segment 52 sample 1`. The launch wrapper exit code was 0, but the
required exact `GATE 6 1.0 KG COMPLETE 1 KG PASS` marker was absent; no
analyzer pass or final-slot claim exists. Cleanup, shutdown process, and
post-shutdown host gates passed. Evidence is in
`.ros_logs/gate6_1kg_bootstrapfix_20260901_01/evidence/`, with the stage log
at `.ros_logs/gate6_1kg_bootstrapfix_20260901_01/gate6_mass_stage_45264_1788234164653.log`.

This invalidates the intended current-source guarantee that Product 101's
retained path was unchanged. Do not start Product 101 pass 2, Product 102,
Product 103, or Gate 7 until Sol/high completes a fresh read-only diagnosis,
freezes a bounded Luna/max packet if a source correction is justified, and
focused validation passes. The existing older 1 kg evidence is not current
source acceptance. The static validation immediately before this runtime
passed: 26 focused tests, Python compilation, the `amr_manipulation` build,
40 package tests with zero errors/failures/skips, and `git diff --check`.

No staging, commit, push, history rewrite, or dependency installation was
performed. `AMR_CODEX_HANDOFF.md` remains untouched. On resume, begin with
`git status --short`, read this section, and keep the fail-closed motion,
collision, exact-slot, ownership, and Gate 6 ordering rules intact.

## Current authoritative state — `_bootstrapfix_01` source validation — 2026-09-01

The user explicitly authorized continuing through Gate 6. Sol/high's bounded
diagnosis and Luna/max's implementation packet are preserved at
`.ros_logs/gate6_product102_pathfix_20260901_01/evidence/next_luna_packet.txt`.
The diagnosis was an intermittent Product 102 preparation-runner service
response boundary: discovery succeeded, the existing 5-second Trigger future
did not complete, and the bootstrap server remained READY with all products
detached. The correction is intentionally limited to
`src/amr_manipulation/scripts/gate6_product_test.py` and
`src/amr_manipulation/test/test_product_test_contract.py`.

ProductPreparation no longer shares its high-rate graph node for bootstrap
verification. Each verification creates one temporary node and one absolute
Trigger client in the existing ROS context, services it with a dedicated
SingleThreadedExecutor, keeps the existing 5-second discovery and response
bounds, removes an incomplete pending request, and always tears down the
temporary executor/node. Negative responses and every existing preparation
ordering/fail-closed boundary are unchanged. No bootstrap server, mass-stage,
Product 101, placement, collision, exact-slot, ownership, or tolerance
behavior was changed by this packet.

Fresh static validation passed: focused runner/MoveIt/completion contracts
reported 26 tests with zero failures; Python compilation passed;
`amr_manipulation` rebuilt successfully; package tests reported 40 tests, zero
errors, zero failures, and zero skips; and `git diff --check` passed. The
current source/install hashes are captured by the next runtime's evidence;
the runner source/install hash is
`50a85115af22b624322457d84fe505dfa07726f061fdd17566f3e311ff0de04a`, the
mass-stage source hash is
`33dab99b63c81ec8e4bcaf9c8aff4ed0d057fe264b1524407aaaa8760995c5fd`, and the
rebuilt mass-stage executable hash is
`5e03503dbf8e49480908d148ff9cc81b44621e96f6ce6e1250156a0315d9d2a0`.

The required live order is now fresh empty motion, two fresh native-attachment
1 kg passes, Product 102 (3 kg), then Product 103 (5 kg). Each run must use a
unique valid domain/partition/log directory, direct-host hardware rendering,
the prescribed readiness gates and recorder, and must stop at its first
failure. The existing accepted 1 kg evidence cannot be reused as current
source acceptance because the mass-stage source changed. Product 103 and Gate
7 remain blocked until the preceding current-source gates pass.
`AMR_CODEX_HANDOFF.md` remains untouched.

## Current authoritative state — `_pathfix_02` bootstrap response diagnosis — 2026-09-01

Sol/high completed the bounded read-only diagnosis for the first failure in
`.ros_logs/gate6_product102_pathfix_20260901_01/`; the evidence packet is
`.ros_logs/gate6_product102_pathfix_20260901_01/evidence/sol_bootstrap_diagnosis.txt`.
The first failure remains the Product 102 preparation runner's bounded
attachment-bootstrap Trigger response timeout at
`1788230402.881373408`, before product reset, navigation, manipulation,
attachment, or mass-stage execution.

The runner's service discovery path completed, but its future did not become
ready within the existing 5-second response bound. The independent readiness
service call immediately before the runner passed. The bootstrap status stream
continued to report `READY` with products 101, 102, and 103 detached through
the failure and afterward. The cleanup terminal output also contained the
matching Fast DDS/RMW warning `failed to send response (timeout): client will
not receive response` at `rmw_response.cpp:154` / `rcl/service.c:314` from the
bootstrap process. This classifies the consumed attempt as an intermittent
ROS 2/Fast DDS service-response transport failure; it is not evidence against
the approved placement route or the bootstrap state.

The current product-runner source/install hash and factory-launch
source/install hash match the earlier accepted Product 102 preparation runs;
the current recorder QoS content also matches. The approved change is in the
mass-stage executable, which was never started. No source change is justified
by this diagnosis. Preserve the authoritative service and fail-closed
preparation boundary; do not substitute a status-topic shortcut or relax the
response bound. Do not retry this consumed runtime, start Product 103, advance
to Gate 7, or alter Product 101. A new transport implementation packet and a
separately authorized runtime would be required before further live evidence.
This Sol packet authorizes no source or test files for editing.
`AMR_CODEX_HANDOFF.md` remains untouched.

## Current authoritative state — `_pathfix_01` Product 102 preparation boundary — 2026-09-01

Phase 14 Gate 6 remains **FAIL / unresolved** at Product 102 (3 kg). The
authorized fresh direct-host run was
`.ros_logs/gate6_product102_pathfix_20260901_01/`; it consumed one Product
102 preparation-runner attempt, started no mass stage, and Product 103
remains at zero attempts. Gates 1-5, Phase J, Phase K, host/rendering,
runtime-RTF, graph/lifecycle, controller, action/service/topic, MoveIt/OMPL,
bootstrap, ownership, cleanup, shutdown, and post-shutdown checks passed
before the product runner.

The approved correction was implemented only in
`src/amr_manipulation/src/gate6_mass_stage.cpp`, with the focused source
contract in `src/amr_manipulation/test/test_moveit_config.py`. Product 101
keeps the existing retained placement sequence; only higher-mass products use
the collision-aware lower planner and the common exact-endpoint,
sample-by-sample validity, timing, and placement gates remain unchanged.
Focused verification passed with 15 tests, the `amr_manipulation` build and
package tests passed with 40 tests and zero errors/failures/skips, and
`git diff --check` passed. No 1 kg runtime was rerun or changed.

The first runtime failure occurred before any product reset, navigation, arm,
gripper, attachment, or mass-stage action. The Product 102 runner started at
`1788230395.5282896`, then reported
`attachment bootstrap verification timed out` at
`1788230402.881373408` and exited with code `2`. The recorder had already
reported `Recording...`. Its finalized bag contains 79,458 messages over
40.975503266 s, but zero Product 102 attach/detach/state transitions, zero
finger contacts, zero navigation action status, zero arbitration commands,
and no mass-stage startup marker. The independent analyzer consequently
reported `GATE6_BAG_ANALYSIS=FAIL`; that is expected incomplete-stage
evidence, not a placement-path result.

Cleanup followed the required recorder -> MoveIt -> factory order. The known
MoveIt Humble destructor `-11` and factory launch
`Cannot shutdown a ROS adapter that is not running` occurred only during
SIGINT teardown. The post-shutdown process scan found no runtime processes and
the direct-host renderer preflight again passed with `/dev/dri/renderD128`.

This run does not validate or falsify the higher-mass collision-aware
placement route because the runner failed at bootstrap verification first.
The next bounded activity is Sol/high read-only diagnosis of this bootstrap
service-response boundary and a new explicit implementation packet if source
work is justified. Do not retry Product 102, start Product 103, advance to
Gate 7, relax any gate, or alter the accepted Product 101 path from this
evidence alone. Preserve the fail-closed motion, attachment, ownership,
collision, exact-slot, and timing invariants. `AMR_CODEX_HANDOFF.md` remains
untouched.

## Current authoritative state — `_13` retained-placement boundary — 2026-09-01

Phase 14 Gate 6 remains **FAIL / unresolved** at Product 102 (3 kg). Gates
1-5, Phase J, Phase K, and the host/readiness/lifecycle/controller/MoveIt,
bootstrap, ownership, cleanup, shutdown, and post-shutdown gates in the latest
direct-host run passed. Product 102 consumed one attempt and Product 103
consumed zero. The latest evidence is
`.ros_logs/gate6_product102_retry_20260901_13/`.
The run used the pinned harness hash
`2ce1dfc9550040540fba64b9066fe34fda2582a684221909b633dc499f1c448b`, source
hash `e6f927de3bf41c8fa510cc2caa0991826179faf1d97bf26b94970ee4ec9b71d7`,
and installed mass-stage hash
`3984178c2a99faa78d5eae49ee3f81763b3e55de6010b6019f4e514ad80dac64`.

The `_13` evidence does prove the 3 kg movement and pickup boundary in the
simulation: the AMR completed navigation to the station and dispatch area,
the bilateral pickup-contact gate passed, Product 102 reached native
attachment state `attached` (`attach=1`, `detach=0` in the retained bag), and
the loaded transport/pre-place steps completed. This is a simulation result,
not a hardware capability claim. Full delivery is still unproven because the
subsequent retained placement path failed collision validation.

The first causal failure is the retained placement-state validity check, not
the later `move_group` cleanup crash:

- exact attached planning-scene proof passed at `1788214919.365739585`;
- MoveIt reported `arm_link_2 <-> product_camera_link` at retained placement
  segment 30, sample 1 at `1788214919.396335684`;
- the source logged the first invalid sample at `1788214919.396375389` and
  failed the payload-aware state-validity gate at `1788214919.439088368`;
- `move_group` exited with `-11` during subsequent shutdown, so that is a
  secondary cleanup symptom, not the cause.

The exact current Product 102 center-slot placement target is geometrically
invalid under the project collision model. A corrected offline MoveIt/FCL
probe using the generated Phase 14 URDF/SRDF reproduced the same KDL branch
and the same camera self-collision at the retained trajectory. The probe
called `state.update()` before collision checks; its earlier dirty-state
mistake was corrected before interpreting results. Sixteen structured
shoulder/elbow seeds converged to the same branch, and bounded radius,
lateral, yaw, orientation, and XY/full-path probes found no alternative
collision-free retained path for the exact center-slot contract. The model
defines `product_camera_link` as a real collision body and SRDF intentionally
does not disable the `arm_link_2`/camera pair.

This classifies `_13` as a **product/source geometry/path contract defect**:
the current exact release/pre-place/retained-path contract cannot pass the
current camera self-collision model. It is not a DDS, lifecycle/startup,
Gazebo, host, runner, or evidence/analyzer failure. No source change is
authorized from this evidence alone: removing the camera collision,
allowing the collision pair, changing the exact slot/release contract, or
silently replacing the retained path would weaken or alter an existing
requirement. No Luna/max implementation packet has been dispatched.

The exact unresolved boundary is phase authority for a compliant correction
that preserves fail-closed collision checking and the declared placement
acceptance criteria. Until that authority exists, do not rerun `_13`
unchanged, start Product 103, or advance to Gate 7. If authority is granted,
Sol/high will freeze one bounded packet first; Luna/max must implement only
that packet, must not plan or replan independently, and must stop on any
scope or hypothesis mismatch.

## Current authoritative state — `_12` pickup-frame boundary — 2026-09-01

Phase 14 Gate 6 remains **FAIL / unresolved** at Product 102 (3 kg). Gates
1-5, Phase J, Phase K, and two independent Product 101 passes remain accepted;
Product 103 and Gate 7 remain blocked. The latest direct-host Product 102-only
runtime is `.ros_logs/gate6_product102_retry_20260901_12/`; it consumed one
Product 102 attempt and zero Product 103 attempts.

The `_12` host/rendering, source/install, readiness, lifecycle, graph,
controller, MoveIt/OMPL, bootstrap, ownership, cleanup, shutdown, and
post-shutdown host gates passed. Its first causal Product 102 failure was the
mass-stage bilateral-contact proof at
`1788213217.270275868`: the close action and bilateral position proof passed,
but right-finger contact was recorded from `1788213212.561138900` while the
left contact stream remained empty. The retained bag has 1,704 right-contact
messages and zero left-contact messages. The right finger contacted the
product handle first and the product shifted; this occurred before placement
alignment and before any analyzer or cleanup failure.

The accepted `_11` comparison recorded 20 contacts on each side. `_12` used
the same arm branch but the fresh base yaw was `+0.010902 rad`, placing the
left finger center at approximately `y=+0.07893 m` while the product handle
edge was about `y=+0.0513 m`; the right finger at `y=-0.06106 m` contacted
first. This classifies the first failure as a **product/source pickup-frame
geometry defect**, not a controller-readiness, DDS, lifecycle, Gazebo, host,
runner, or evidence/analyzer defect.

Sol/high froze the exact packet at
`.ros_logs/gate6_product102_retry_20260901_12/evidence/next_luna_packet.txt`.
Luna/max implemented only the mass-stage source and focused contract test:
after fresh reference/open evidence, the pickup product and robot poses are
validated and mapped into `base_footprint`; the measured product lateral
coordinate is used consistently for the pickup scene, pre-grasp, and grasp.
The nominal arm branch, dimensions, bilateral contact, native attachment,
fail-closed behavior, tolerances, time bounds, ownership, and placement
proofs remain unchanged. Luna did not plan or replan independently.

Sol/high independently verified the change: source hash
`e6f927de3bf41c8fa510cc2caa0991826179faf1d97bf26b94970ee4ec9b71d7`,
installed executable hash
`3984178c2a99faa78d5eae49ee3f81763b3e55de6010b6019f4e514ad80dac64`,
`colcon build --packages-select amr_manipulation --symlink-install` passed,
the package CTest suite passed `6/6` (`100%`), focused Python contracts passed
`14`, and `git diff --check` passed.

The exact unresolved boundary is one fresh clean-host, direct-host Product
102-only runtime using a new run identity and the current installed hash.
Stop at the first failed gate; do not repeat `_12` unchanged, start Product
103, or advance to Gate 7. Before launch, verify no stale ROS/Gazebo process
and refresh the harness's source/install hash pins. If the fresh run fails,
reconstruct that first failure before any further runtime attempt.

## Current authoritative state — `_08` Product 102 gripper boundary — 2026-09-01

Phase 14 Gate 6 remains **FAIL / unresolved** at Product 102 (3 kg). Gates
1-5, Phase J, Phase K, and two independent Product 101 passes remain
accepted; Product 103 and Gate 7 remain blocked. The latest authorized
direct-host Product 102-only run is
`.ros_logs/gate6_product102_retry_20260901_08/`.

The `_08` host/rendering, source/install, graph/lifecycle, controller,
MoveIt/OMPL, bootstrap, ownership, cleanup, shutdown, and post-shutdown gates
passed. Product preparation passed at `1788209258.254874156`, the mass stage
started, and the open gripper proof passed. The first product failure was the
mass-stage bilateral position proof at `1788209274.045441999`, after the close
action succeeded at `1788209270.985996398` with measured left position
`0.0200 m`. Product 103 was not started.

The retained bag and logs prove a product/source bilateral-gripper defect, not
infrastructure: left joint samples reached exactly `0.020 m` while the right
joint remained `0.035 m`; the left contact stream recorded zero product
contacts while the right stream recorded the product handle. The source proof
uses `left > threshold && right > threshold` with the exact close threshold
`0.020`. The composite model exposes the right joint as a passive mimic, and
DART logged that its physics engine does not support the mimic constraint at
`1788209134.597...`. The existing one-joint gripper action therefore does not
provide deterministic bilateral actuation in this runtime. Readiness and
action results were present, so the first failure is not DDS, lifecycle,
harness, analyzer, host, or startup timing.

Sol/high recorded the complete RCA and froze one implementation packet at
`.ros_logs/gate6_product102_retry_20260901_08/evidence/next_luna_packet.txt`.
Luna/max is implementation/focused-verification only: it must not plan or
replan independently. The packet allows only the listed gripper source,
description, factory launch/readiness, and focused contract-test files. It
changes the proof boundary to inclusive comparison and replaces unsupported
passive mimic reliance with an independently commanded right stock gripper
controller, while preserving bilateral contact, attachment, ownership,
fail-closed, tolerance, and timeout semantics. No integrated runtime is
authorized from the implementation pass.

After focused verification and diff review, run exactly one new clean-host,
direct-host Product 102-only validation with both gripper action-status topics
recorded, stopping at the first failed gate. Do not start Product 103 or Gate
7. The secondary fixed-grasp lateral-alignment risk is intentionally deferred
until deterministic bilateral actuation is validated.

## Superseded `_07` Product 102 localization boundary — 2026-09-01

Phase 14 Gate 6 remains **FAIL / unresolved** at Product 102 (3 kg). Gates
1-5, Phase J, Phase K, and two independent Product 101 passes remain accepted;
Product 103 and Gate 7 remain blocked. The latest authorized Product run is
`.ros_logs/gate6_product102_retry_20260901_07/`.

`_07` host/rendering, source/install hashes, graph/lifecycle, controller,
MoveIt/OMPL, bootstrap, ownership, cleanup, shutdown, and post-shutdown gates
passed. It consumed one Product 102 runner attempt and zero Product 103
attempts. The handled initial precise-dock progress abort occurred at
`1788206946.378412202`; recovery completed. The first final-gate failure was
the independent physical dock proof at `1788206985.239138529`, after both
final precise actions succeeded at `1788206985.234373732` and
`1788206985.238695165`. Settled ground truth was
`(2.347034, 0.002245, 0.052365)`, so the unchanged physical dock check
correctly rejected `position=0.0537 m / yaw=0.0524 rad`. The mass stage was
never started.

The bag shows the causal pose split, not stale settling or command ownership:
raw/wheel odometry and ground truth agree and remain stationary after the
final command, while the AMCL/map feedback ends at
`(2.390182, -0.001508, 0.063884)`. At the last AMCL update,
`1788206982.529401`, its covariance had grown to `x=0.008734` and its
`map->odom` transform remained the corresponding ahead-of-truth correction.
The final command stream is zero by `1788206986.390560`; ground truth changes
only about 0.6 mm afterward. This falsifies missing settling, wheel/plant
motion, controller completion, and the runner's terminal-pose freshness as
the first cause.

The targeted factory-only front-lidar probe
`.ros_logs/gate6_localization_scan_probe_20260901_01/` recorded the same
sensor at the settled ground-truth pose. In the forward pickup sector
`[-0.10, 0.30] rad`, 60 measured beams matched a raster ray-cast of the
canonical map with `0.000939 m` mean absolute error and `100%` within 20 mm;
the AMCL terminal pose had `0.043768 m` mean error and `0%` within 20 mm.
The forward beam at index 360 was measured `0.43843 m`, versus map predictions
`0.44000 m` at ground truth and `0.39500 m` at the AMCL pose. The probe itself
passed host/readiness/recording/cleanup and ran no Product action. The map
pickup cells now exactly cover the SDF pedestal `[3.10,3.50] x [-0.25,0.25]`.

Classification is a **product/source localization-configuration defect in
the noiseless simulation**, specifically the factory AMCL motion model's
`alpha1..alpha5: 0.2` in `src/amr_factory/config/amcl.yaml`. The deterministic
Gazebo DiffDrive/raw/wheel streams provide no modeled motion noise, yet AMCL
covariance expands during the final 0.9 m leg and the estimate runs ahead
while the actual scan favors ground truth. This is not a DDS, lifecycle,
world-control, map-asset, controller, runner, or evidence/analyzer failure.
Full `_07` RCA and probe correlation are under the run-scoped evidence files.

The single approved implementation packet is
`.ros_logs/gate6_product102_retry_20260901_07/evidence/next_luna_packet.txt`.
Luna/max is implementation/verification only: apply that packet, do not plan
or replan independently, and stop if its scope or hypothesis is invalid. The
packet changes only the factory AMCL motion-noise parameters and a focused
configuration contract test; it does not change map geometry, runner logic,
timeouts, tolerances, ownership, fail-closed behavior, or mass handling.
After focused verification, run exactly one fresh clean-host direct-host
Product 102-only validation and stop at the first failed gate. Do not start
Product 103 or Gate 7. Preserve unrelated work and do not modify
`AMR_CODEX_HANDOFF.md`.

## Superseded `_06` Product 102 pause-response boundary — 2026-09-01

The `_06` host/rendering, hash, graph/lifecycle, controller, MoveIt/OMPL,
bootstrap, ownership, cleanup, shutdown, and post-shutdown gates passed. Its
first causal failure was before navigation: runner STARTING at
`1788204768.373800278`, proxy `rmw_response.cpp:154` response-send timeout at
`1788204768.474395936`, then runner `Gazebo world pause timed out` at
`1788204771.402934305`. The bag proves the pause side effect, while the
factory-only and runner-shaped probe runs passed ControlWorld independently.
It remains classified as an intermittent DDS/RMW service-response boundary;
no Product source change came from that incident. Record:
`.ros_logs/gate6_product102_retry_20260901_06/evidence/post_run_root_cause.txt`.

## Superseded `_05` Product 102 final-dock boundary — 2026-09-01

Phase 14 Gate 6 higher-mass validation remains **FAIL / unresolved**. Gates
1-5, Phase J, Phase K, and two independent Product 101 passes remain accepted.
Product 103 and Gate 7 remain blocked. The latest direct-host Product 102-only
run is `.ros_logs/gate6_product102_retry_20260901_05/`; its host, rendering,
runtime, graph/lifecycle, controller, MoveIt/OMPL, bootstrap, ownership,
cleanup, shutdown, and post-shutdown host gates passed.

The `_05` run consumed one Product 102 preparation attempt and zero Product
103 attempts. The first causal failure was the runner's unchanged independent
product-geometry proof at `1788202887.720104694`, immediately after the final
precise action succeeded at `1788202887.718557119`. The physical dock proof
had already passed: settled ground truth was `(2.371799, 0.004260, 0.080093)`,
giving `0.028520940 m / 0.080093 rad` against the unchanged `0.030 m / 0.15
rad` limits. The product remained stationary at `(3.25, 0, 0)`, but the
fixed base-frame grasp geometry measured `0.078605507 m`, over the unchanged
`0.040 m` product gate. The missing mass-stage log is downstream.

The bag timeline shows the final recovery dock sent one precise goal coupling
translation and terminal yaw. Nav2 accepted the goal inside its unchanged
`0.15 rad` yaw window while the physical robot ended at a yaw that was valid
for the broad dock gate but invalid for the fixed top-grasp frame. Readiness,
DDS/lifecycle, Gazebo, host, ownership, and analyzer gates passed; the
validator-only frame correction was falsified because the C++ mass stage uses
the same fixed `(0.85, 0)` base-frame grasp target and would reject the same
`0.078605507 m` error later.

Classification: **product/source route-controller contract defect**. Sol/high
froze the revised packet at
`.ros_logs/gate6_product102_retry_20260901_05/evidence/next_luna_packet.txt`.
Luna/max implemented only the runner and its focused contract test, without
planning or replanning independently. Both final-dock paths now send a
current-to-dock travel-bearing precise goal, then the same-position
registered-yaw precise goal. Dock/product tolerances, fail-closed proof,
abort semantics, C++ mass-stage logic, map/SDF, and controller configuration
remain unchanged. Independent package verification passed 11 focused tests,
the package build, 37 package tests with zero errors/failures/skips, Python
compile, diff check, and source/install hash equality. Record:
`.ros_logs/gate6_product102_retry_20260901_05/evidence/implementation_validation.txt`.

The next authorized boundary is exactly one fresh clean-host direct-host
Product 102-only run with the corrected runner and map, after a stale-process
and rendering preflight. Stop at the first failed gate. Do not start Product
103 or Gate 7. Preserve the dirty worktree and do not modify
`AMR_CODEX_HANDOFF.md`.

## Superseded `_12` state — retained below

Phase 14 Gate 6 higher-mass validation remains **FAIL / unresolved**. Gates
1-5, Phase J, Phase K, and two independent Product 101 passes remain accepted.
Product 103 and Gate 7 remain blocked. The latest direct-host Product 102-only
run is `.ros_logs/gate6_product102_retry_20260831_12/`; all host, runtime,
graph, lifecycle, controller, MoveIt/OMPL, action/service/topic, bootstrap,
ownership, cleanup, shutdown, and post-shutdown host gates passed.

The `_12` run consumed one Product 102 preparation attempt and zero Product 103
attempts. It stopped before the mass stage because the final precise recovery
re-dock was aborted by Nav2's global progress checker:

  `1788194438.273704767` controller accepted the recovery precise goal;
  `1788194453.873780266` logged `Failed to make progress`;
  `1788194453.874380827` mission supervisor logged `Mission aborted: path following failed`.

Bag replay shows the progress checker had only `0.160357 m / 0.125214 rad`
mission-feedback progress, and independently `0.137067 m / 0.113642 rad`
localization-odometry progress, over its 10.0-second `0.20 m / 0.20 rad`
requirement. The unchanged physical final gate was not disproven: ground truth
ended at position error `0.028579 m` and yaw error `0.148726 rad`, inside the
unchanged `0.03 m / 0.15 rad` acceptance limits. Product 102 remained
stationary, and command streams were owned and consistent.

Classification is a product/source bug: the runner already applies a typed,
bounded terminal-abort recovery gate to the initial precise dock, but the
final precise dock inside the recovery branch is unhandled. It fails before
the existing independent fresh physical dock and product-geometry proof can
run. The AMCL future-transform warning at `1788194433.403329611` was followed
by successful relocalization at `1788194433.492877014` and is not causal. No
DDS, lifecycle, Gazebo, rendering, analyzer, or host-contamination failure
occurred in `_12`.

Sol/high froze the exact implementation packet at
`.ros_logs/gate6_product102_retry_20260831_12/evidence/post_run_root_cause.txt`.
Luna/max implemented only
`src/amr_manipulation/scripts/gate6_product_test.py` and
`src/amr_manipulation/test/test_product_test_contract.py`. The final recovery
precise dock now catches only an in-window `NavigationAbortedError`, waits for
the existing stationary boundary and a fresh physical pose, and then falls
through to the unchanged independent final/product proof. Out-of-window and
non-aborted failures remain fail-closed. No retry loop,
timeout/tolerance/progress-config change, direct velocity path, or route
change was made. Luna did not plan or replan independently and ran no
integrated runtime. Focused validation is recorded in
`.ros_logs/gate6_product102_retry_20260831_12/evidence/implementation_validation.txt`.

The next authorized boundary is one fresh clean-host direct-host Product
102-only attempt with a new run identity, after a direct-host stale-process
scan. It must stop at the first failed gate. Do not repeat `_12` unchanged,
start Product 103, or advance to Gate 7. Preserve the dirty worktree and do
not modify `AMR_CODEX_HANDOFF.md`.

Sections below are retained investigation history and are superseded where
they conflict with this current state.

## Authority and stop condition

The user explicitly authorized Phase 14 implementation according to
`docs/PHASE_14_FACTORY_MOBILE_MANIPULATION.md`, gate-by-gate, with no progress
past a failed gate. Preserve the dirty worktree. Do not modify
`AMR_CODEX_HANDOFF.md`; do not commit or push. Request approval before installing
dependencies or downloading external assets.

Work stopped at the user's quota condition. The next elevated live-launch
request returned a usage-limit rejection with reset time August 20, 2026 at
11:16 AM. No simulation or MoveIt process is intentionally left running.

## Gate progress

- Gates 1-5: passed.
- Gate 6 empty motion: passed live at the required 0.2 velocity and acceleration
  scaling and returned to the fixed stow tolerance.
- Phase J: passed and closed; preserved evidence remains authoritative.
- Phase K integrated readiness and the single Product 101 acceptance run:
  passed and closed under the retained `gate6_1kg_retained_20260830_01` run.
- Gate 6 1 kg repeatability pass 2: attempted once; the stage reached its
  terminal pass line, but the required independent bag analyzer failed, so
  repeatability is not accepted.
- Gate 6 3 kg and 5 kg, Products 102/103, Gate 7, completion documentation,
  and full workspace acceptance: not started. Do not advance to them.

## Gate 6 evidence and fixes

- Added the Gazebo Contact system and product/finger contact plumbing.
- Added fail-closed base/product reference evidence, attachment pose and fresh
  bilateral product-contact checks, exact product IDs, and positive Gazebo
  attachment confirmation.
- Patched the vendored `gz_ros2_control` node construction so controller YAML is
  applied before the hardware node starts. The active simulation position gain
  was proven as `0.5`; strict 0.05 rad path and 0.01 rad goal tolerances remain.
- Use `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for live ROS processes; Fast DDS shared
  memory port initialization became unreliable. No new middleware was installed.
- Graphical inspection and installed Gazebo examples proved that every
  `DetachableJoint` starts attached. `gate6_mass_stage` now commands and
  positively confirms all products 101/102/103 detached before any gripper or
  arm command. This removed the false loads and product dragging.
- The collision-checked Cartesian staging retreat passes live with zero
  base/product displacement.
- The pre-grasp now uses current-state-seeded exact IK for `gripper_tcp`, then
  retains an independent wrist-branch guard. Staging, seeded pre-grasp, and the
  Cartesian grasp approach all passed live with zero base/product displacement.
- The last live boundary failed before attachment because finger positions
  reached exactly `0.0275 m`, leaving the same 0.100 m gap as the handle width;
  no contact was generated. The source now commands `0.020 m` so the fingers
  should stall on the handle and generate bilateral contact. This change builds
  and all 11 `amr_manipulation` tests pass, but quota stopped the fresh live run.

No live attachment has yet been accepted in the corrected sequence. Lift and
loaded stow have therefore not been exercised. The current mass executable also
does not implement the required transport/place half of the 1 kg acceptance
stage, so do not mark the 1 kg gate or Gate 6 complete even if grasp/stow passes.

## Next actions after quota reset

Use a fresh runtime because Gazebo publishes `detached` only on a real startup
`attached -> detached` transition; a second mass-stage invocation in the same
runtime correctly fails closed rather than inferring a no-op detach.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export GZ_PARTITION=amr_phase14_gate6_contact_retry
export ROS_DOMAIN_ID=99
ros2 launch amr_factory factory_localization.launch.py \
  headless:=true initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

In a second terminal with the same environment:

```bash
ros2 launch amr_manipulation move_group.launch.py
```

After MoveIt reports ready, in a third terminal with the same environment:

```bash
ros2 launch amr_manipulation gate6_mass_stage.launch.py product_id:=101
```

At the contact boundary, verify the finger joints stall above the 0.020 m
command, both native contact topics contain `product_a`, the application sees
both contacts no older than 100 ms, the measured grasp transform is within
30 mm / 0.15 rad, and attachment is positively confirmed. Do not weaken any of
those gates. If grasp/stow passes, implement and validate 1 kg transport/place
before considering the 1 kg stage passed; then proceed to 3 kg only after the
full 1 kg acceptance gate passes.

## Latest validation and important files

- `colcon build --packages-select amr_manipulation --symlink-install`: pass.
- `colcon test --packages-select amr_manipulation`: pass.
- `colcon test-result --test-result-base build/amr_manipulation --verbose`:
  11 tests, zero errors, zero failures, zero skips.
- No full-workspace Phase 14 validation has run.

Primary current files include:

- `src/amr_manipulation/src/gate6_mass_stage.cpp`
- `src/amr_manipulation/test/test_moveit_config.py`
- `src/amr_description/config/phase14_mobile_manipulator_controllers.yaml`
- `third_party/gz_ros2_control/src/gz_ros2_control_plugin.cpp`
- `src/amr_factory/worlds/factory.sdf`

Known follow-up risks include the MoveIt Humble class-loader segfault on clean
Ctrl-C shutdown, the mass executable's incomplete live placement acceptance,
the fixed pickup-dock/costmap collision documented below, and the lack of
complete-workspace validation. Preserve all unrelated dirty-tree content.

## Phase 14 continuation — 2026-08-21

The corrected 1 kg grasp and loaded-stow boundary was rerun in a fresh runtime
(`GZ_PARTITION=amr_phase14_gate6_contact_retry`, `ROS_DOMAIN_ID=99`) and passed:
the boot detach transitions, bilateral `product_a` contacts, finger-position
proof above the `0.020 m` request, attachment confirmation, 80 mm lift, loaded
stow, and dock-reference stability all passed. The required negative
out-of-dispatch detachment check also passed without publishing a detach
request; native state remained `attached`.

The complete 1 kg run remains failed closed at the first loaded navigation
leg. Fresh retries (`ROS_DOMAIN_ID=96` and `91`) reached the same boundary and
reported `Optimizer fail to compute path` / `path following failed` while
navigating from the fixed pickup dock toward the registered pickup approach.
The product remained attached and the stage published `FAULT`.

Read-only diagnosis in an isolated baseline runtime (`ROS_DOMAIN_ID=87`) found
the local costmap's robot-center cell at the pickup dock was lethal (`253`)
because the fixed pickup pedestal is within the configured 1.20 x 0.80 m
footprint/inflation envelope. A temporary correctly-QoS'd reverse command moved
the disposable robot from approximately `(2.4, 3.0)` to `(2.009, 2.958)`; the
same registered approach goal then succeeded. This confirms a fixed dock /
costmap geometry blocker, not an interlock-QoS failure. No footprint,
tolerance, registry pose, or safety check was weakened.

`factory_localization.launch.py` now delays the localization lifecycle manager
by 5 seconds so map_server and AMCL services are ready before the first
configure request. A fresh runtime (`ROS_DOMAIN_ID=86`) automatically reached
active states for both nodes, logged AMCL pose `(2.400, 3.000, 0.000)`, and
published `map -> odom` translation `(2.400, 3.000, 0.000)`.

Validation after the continuation edits:

- `colcon build --packages-select amr_description amr_control amr_factory amr_manipulation --symlink-install`: pass.
- `colcon test --packages-select amr_description amr_control amr_factory amr_manipulation`: pass.
- `colcon test-result --verbose`: 162 tests, 0 errors, 0 failures, 5 skipped.
- `python3 -m py_compile` passed for the three changed launch files.

The 1 kg acceptance gate is **not passed**. Do not start 3 kg, 5 kg, Gate 7,
or completion documentation. The next action requires an explicitly approved
resolution of the fixed pickup-dock/costmap collision; under the Phase 14 plan,
stop and report this blocker rather than changing registry positions, the
robot footprint, tolerances, or fixed stow pose. All ROS, MoveIt, and Gazebo
processes were stopped; no commit or push was made.

## Phase 14 continuation — 2026-08-21 GUI gripper timeout fix

`gate6_mass_stage.cpp` now gives an accepted gripper goal an internal 30-second
wall-clock result limit. The server and goal-acceptance waits remain 3 seconds,
and the 0.020 m close, 0.035 m open, 60 N effort, controller settings, grasp
geometry, and bilateral evidence gates are unchanged. The helper retains the
accepted goal handle, checks the result code and pointer before reading result
fields, requires `reached_goal || stalled`, logs requested/measured position,
elapsed wall time, and result flags, and fails closed with verified cancellation
and terminal-result checks if the 30-second limit expires.

The manipulation source-contract test now asserts the 30-second result wait,
retained goal handle, cancellation call and ordering, result guards, success
flags, and post-close bilateral position/contact checks.

Automated validation:

- `colcon build --packages-select amr_description amr_control amr_factory amr_manipulation --symlink-install`: pass.
- `colcon test --packages-select amr_description amr_control amr_factory amr_manipulation`: pass.
- `colcon test-result --verbose`: 162 tests, 0 errors, 0 failures, 5 skipped.

Fresh GUI live validation used `GZ_PARTITION=amr_phase14_gate6_gui_timeout_fix`,
`ROS_DOMAIN_ID=98`, `headless:=false`, and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.
Gazebo reported `real_time_factor=0.1973`. Product 101 open completed in 1.241 s;
the close completed in 6.478 s, beyond the former 5-second client limit and
before 30 seconds, with result `SUCCEEDED`, `reached_goal=false`,
`stalled=true`, measured position `0.0269 m`, and bilateral positions
`0.0269/0.0281 m`. Fresh bilateral contact, attachment, 80 mm lift, loaded
stow, and the required out-of-dispatch detachment rejection passed; native
state remained attached. The stage then failed closed at the pre-existing
loaded navigation pickup-dock/costmap blocker. Afterward the gripper action
graph showed one active server and zero clients; this controller did not expose
an action-status topic for a stronger historical-goal query. All runtime
processes were stopped, with the known MoveIt Humble clean-shutdown segfault
recurring during Ctrl-C cleanup.

This verifies the GUI gripper timeout fix only. The 1 kg acceptance gate remains
not passed, and no later gate or placement work is authorized until the fixed
pickup-dock/costmap collision receives an explicit Phase 14 decision.

## Phase 14 continuation — bounded pickup-dock egress — 2026-08-21

The approved bounded reverse egress is implemented. Pickup stations A/B/C now
register collinear 0.50 m egress poses at `(1.90, 3.00, 0.0)`, `(1.90, 0.00,
0.0)`, and `(1.90, -3.00, 0.0)`. The command-arbitration lifecycle node remains
the sole `/amr/control/cmd_vel` publisher and now owns the internal
`/amr/control/dock_egress` `nav2_msgs/action/BackUp` server. It accepts only
fresh, semantically valid loaded-stow/READY/filtered-odometry/rear-LiDAR/TF
evidence, commands straight negative X through the existing clamps, checks the
complete footprint swept corridor plus one 0.05 m cell, ignores Nav2 samples
while reserved, and publishes zero with an exact terminal reason on every
success, cancellation, timeout, stale-evidence, obstacle, drift, deactivation,
or interlock-loss path. Gate 6 retains the accepted goal handle, waits 65 s,
verifies cancellation and terminal `CANCELED` on client timeout, retains the
attachment, and only then starts the unchanged Nav2 pickup-approach goal.

The integrated factory's 10 Hz rear lidar can be slower in wall time when
Gazebo runs at low real-time factor, so the configuration-backed freshness
deadline is 1.0 s; missing or malformed data still fails closed. An initial
headless attempt with the prior 0.2 s deadline failed safely as `rear LiDAR
evidence is stale` after 0.227 s, with measured travel 0.001 m.

Validation after the egress implementation:

- `colcon build --packages-select amr_control amr_manipulation amr_factory
  --symlink-install`: pass.
- Full `colcon test --event-handlers console_direct+`: 167 tests, 0 errors,
  0 failures, 5 skipped. The focused egress action tests covered success,
  straight reverse output, Nav2 sample clearing, malformed requests, and an
  obstructed rear scan; the contract suite passed 23 Python tests.
- A fresh headless product-101 run (`GZ_PARTITION=amr_phase14_gate6_headless_3`,
  `ROS_DOMAIN_ID=97`) logged `Dock egress SUCCEEDED: requested=0.500 m`,
  command-arbitration measured travel `0.501 m`, and elapsed wall time `8.981 s`.
  The following unchanged Nav2 goal reached the registered pickup approach;
  the later dispatch-approach leg failed with the pre-existing MPPI
  `Optimizer fail to compute path` condition. Product attachment was retained
  and the stage failed closed.
- A fresh Gazebo GUI product-101 run (`GZ_PARTITION=amr_phase14_gate6_gui_egress`,
  `ROS_DOMAIN_ID=96`) logged egress success at `0.500 m` in `6.093 s`; the
  pickup-approach goal reached, and the same later dispatch-approach leg failed
  closed. `ros2 topic info /amr/control/cmd_vel -v` showed exactly one publisher,
  `amr/command_arbitration_node`.
- The obstructed/malformed control test is the simulated egress-failure
  injection: the goal was rejected before motion and no delayed Nav2 command
  replayed. No custom action/message was added.

The 1 kg Gate 6 acceptance gate remains **not passed** because the subsequent
dispatch navigation leg still fails; this change is limited to the approved
pickup-dock egress and does not alter Nav2 footprint/inflation, grasp geometry,
attachment rules, or velocity ownership. Do not start the 3 kg or 5 kg stages,
Gate 7, or completion documentation. All ROS, MoveIt, and Gazebo processes from
the headless and GUI runs were stopped; no commit or push was made.

## Phase 14 continuation — 2026-08-21 1 kg XY-tolerance verification

The minimal approved controller change is applied: `amr_mpc_controller/config/controller.yaml`
now sets `goal_checker.xy_goal_tolerance: 0.07`; the existing controller
contract test requires exactly `0.07`. `yaw_goal_tolerance: 0.15`, progress
checking, MPPI limits, footprint/inflation, egress, manipulation, attachment,
and acceptance settings were unchanged.

Focused validation passed:

- `GZ_VERSION=harmonic colcon build --packages-select amr_mpc_controller --symlink-install`
  completed successfully.
- `colcon test --packages-select amr_mpc_controller` completed successfully;
  `colcon test-result --test-result-base build/amr_mpc_controller --verbose`
  reported 6 tests, 0 errors, 0 failures, 0 skipped.

A fresh headless Harmonic product-101 run used
`GZ_PARTITION=amr_phase14_gate6_xy_tolerance_070`, `ROS_DOMAIN_ID=94`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and a writable `ROS_LOG_DIR` under
`.ros_logs/xy_tolerance_070`. The run reached the following boundaries before
failing closed:

- Gripper open succeeded (`0.0350 m` requested, `0.0335 m` measured); close
  stalled against the product (`0.0200 m` requested, `0.0270 m` measured), with
  bilateral positions `0.0270/0.0280 m`.
- Grasp, attachment confirmation, 80 mm lift, loaded stow, dock-reference
  stability, and the required out-of-dispatch detachment rejection passed;
  native product state remained attached and no valid dispatch detachment was
  requested.
- The bounded reverse egress succeeded at the unchanged `0.10 m/s` cap:
  requested `0.500 m`, command-arbitration measured travel `0.503 m`, elapsed
  wall time `12.631 s`. The following pickup-approach Nav2 goal reached its
  goal (`controller_server` logged `Reached the goal!`).
- The next dispatch-approach goal was accepted, but the stage's unchanged
  120-second wait expired and it exited with code 1 at
  `navigation to dispatch approach failed`. `controller_server` subsequently
  logged `Failed to make progress` and aborted the path-following handle. The
  factory runtime also logged a mission-supervisor `UnknownGoalHandleError`
  during the transition.

The run did not reach the dispatch dock ground-truth check, placement,
detachment, gripper opening, retreat, empty stow, or final success publication.
The 1 kg Gate 6 acceptance gate remains **not passed**. All ROS, MoveIt, and
Gazebo processes, including stale launch groups found before the run, were
stopped; no additional navigation tuning, recovery logic, later mass stage, or
phase advancement was performed.

## Phase 14 continuation — Reliable 1 kg Gate 6 implementation and bounded acceptance run — 2026-08-21

The approved reliability plan was implemented without changing the URDF, SRDF,
gripper geometry, stow target, allowed-collision pairs, navigation footprint or
inflation, MPPI safety limits, station poses, progress thresholds, velocity
limits, attachment rules, or acceptance tolerances.

Manipulation now executes the approved `grasp -> lift_checkpoint (+0.080 m) ->
clearance_retreat (pregrasp)` sequence as one collision-checked Cartesian path,
restores the temporary pickup collision allowance before stow, verifies fresh
MoveIt `/check_state_validity` evidence for `manipulator` (including returned
contact pairs), and calls `setStartStateToCurrentState()` before loaded stow.
The manipulation contract test covers the ordered waypoints, one Cartesian
calculation/execution, allowance restoration, and validity proof.

Navigation now wraps the existing MPPI controller with
`nav2_rotation_shim_controller::RotationShimController` using the exact plan
parameters and keeps the existing MPPI namespace/limits. The package declares
the rotation-shim runtime dependency, and the controller contract test checks
the wrapper, primary controller, rotation parameters, tolerances, and safety
settings.

The mission supervisor now uses explicit `IDLE`, `PLANNER_PENDING`,
`PLANNER_ACTIVE`, `CONTROLLER_PENDING`, `CONTROLLER_ACTIVE`, and `CANCELING`
states with one public mission identity, explicit cancel/result races, stale
callback rejection, downstream handle clearing, defensive
`UnknownGoalHandleError` handling, ROS-clock feedback timing, and latest
`map -> base_footprint` feedback poses. Runtime behavior tests cover planning
cancel, following cancel, sequential missions, abort/deactivation, result
races, malformed goals, and exactly-once public completion.

Focused verification passed:

- `GZ_VERSION=harmonic colcon build --packages-select amr_description
  amr_mpc_controller amr_mission amr_manipulation amr_factory --symlink-install`.
- The matching focused `colcon test` command passed all packages.
- `colcon test-result --verbose`: **179 tests, 0 errors, 0 failures, 5 skipped**.
- Integrated controller logs confirm `RotationShimController`, its internal
  `MPPIController`, configuration, and lifecycle activation:
  `.ros_logs/reliable_kg_20260821/factory/controller_server_406003_1787292844418.log`.

One fresh product-101 run was performed exactly once after the focused gates,
with `GZ_PARTITION=amr_phase14_gate6_reliable_kg_20260821`, `ROS_DOMAIN_ID=124`,
Harmonic headless factory/MoveIt, and evidence bag
`.ros_logs/reliable_kg_20260821/product101_evidence_full` (274.0 MiB,
517065 messages, 132.820725 s). The run evidence is:

- Grasp preparation and bilateral contact passed; the continuous retreat
  produced a 100% Cartesian path, loaded stow completed, and the required
  out-of-dispatch detachment rejection passed with native attachment retained.
- Dock egress succeeded: requested `0.500 m`, measured `0.502 m`, elapsed wall
  `8.291 s`.
- Pickup approach succeeded: target `(1.500, 3.000, 0.000)`, localized
  `(1.602, 3.006, -0.029)`, XY error `0.102 m`, yaw error `0.029 rad`.
- Dispatch approach was accepted but the controller failed to make progress;
  the mission supervisor reported `Mission aborted: path following failed`
  without a process exception. Terminal evidence was code `6` (ABORTED), target
  `(-2.500, 0.000, 3.142)`, localized `(-2.481, 0.101, -3.134)`, XY error
  `0.102 m`, yaw error `0.007 rad`, distance remaining `0.000`, simulation
  navigation time `43.503 s`, ground truth `(-2.409, 0.122, -0.000)`.

The run stopped immediately at that boundary, so dispatch placement,
detachment, empty stow, and final success were not claimed. The 1 kg Gate 6
acceptance gate remains **not passed**. No retry, parameter tuning, recovery
logic, later mass/gate work, commit, or push was performed. All ROS, MoveIt,
Gazebo, stage, and rosbag processes were stopped afterward.

## Phase 14 continuation — MPPI near-goal fix acceptance evidence — 2026-08-21

The bounded MPPI fix was implemented and independently verified. The reusable
agent workflow was added to `AGENTS.md`; `GoalAngleCritic`, `PathAlignCritic`,
and `PathAngleCritic` `threshold_to_consider` are each `0.07`, matching the
unchanged `xy_goal_tolerance`. The focused contract, five-package build/test,
and test-result gates passed: **179 tests, 0 errors, 0 failures, 5 skipped**.

One fresh product-101 run was then performed exactly once with
`GZ_PARTITION=amr_codex_gate6_final_20260821_01`, `ROS_DOMAIN_ID=130`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and evidence bag
`.ros_logs/gate6_final_20260821_01/product101_evidence` (246.8 MiB,
466020 messages, 139.340439437 s). Controller logs confirmed the
`RotationShimController`, internal MPPI controller, and 0.070 critic threshold
were loaded and activated.

- Grasp preparation, bilateral contact, continuous Cartesian retreat,
  loaded stow, and out-of-dispatch detachment rejection passed.
- Dock egress succeeded: requested `0.500 m`, elapsed `9.757 s`.
- Pickup approach succeeded: target `(1.500, 3.000, 0.000)`, localized
  `(1.529, 3.046, -0.141)`, XY error `0.055 m`, yaw error `0.141 rad`.
- Dispatch approach succeeded: target `(-2.500, 0.000, 3.142)`, localized
  `(-2.441, 0.037, -2.995)`, XY error `0.070 m`, yaw error `0.147 rad`,
  distance remaining `0.000`, simulation navigation time `35.789 s`, ground
  truth `(-2.391, 0.044, -0.000)`.
- Dispatch dock succeeded: target `(-3.400, 0.000, 3.142)`, localized
  `(-3.348, 0.011, -3.066)`, XY error `0.053 m`, yaw error `0.076 rad`,
  distance remaining `0.000`, simulation navigation time `5.233 s`, ground
  truth `(-3.284, -0.018, -0.000)`.
- The first new boundary was exact seeded pre-place IK: `GATE 6 1.0 KG:
  FAIL: exact seeded pre-place IK failed` in
  `.ros_logs/gate6_final_20260821_01/stage/gate6_mass_stage_476609_1787303473591.log`.

The run stopped immediately at that boundary. Placement, authorized
detachment, empty stow, and final success were not claimed; the 1 kg Gate 6
acceptance gate remains **not passed**. No retry, parameter tuning, recovery
logic, later mass/gate work, commit, or push was performed. All runtime
processes were stopped. MoveIt emitted an exit-11 destructor fault during the
deliberate shutdown after the stage had already failed; it did not occur during
the acceptance path.

## Phase 14 continuation — Alignment sequencing follow-up and bounded failure evidence — 2026-08-21

The prior alignment defect was corrected in `src/amr_manipulation/src/gate6_mass_stage.cpp`:
command segment spacing now accounts for the unchanged 0.07 m Nav2 terminal
tolerance (`0.15 - 0.07 = 0.08 m`), translation goals use their travel bearing,
and a separate same-position goal requests the approved dispatch heading. The
contract test was updated accordingly. Focused build/test gates remained green:
179 tests, 0 errors, 0 failures, 5 skipped.

One fresh post-fix product-101 run was performed exactly once with
`GZ_PARTITION=amr_codex_gate6_loop4_20260821_01`, `ROS_DOMAIN_ID=132`, and
evidence bag
`.ros_logs/amr_codex_gate6_loop4_20260821_01/bag/product101_evidence`.
Grasp, continuous retreat, validity proof, loaded stow, negative detachment
rejection, egress, pickup approach, dispatch approach, dispatch dock, all five
bounded alignment segments, and the final heading goal passed. The first new
boundary was loaded pre-place OMPL planning:

```
GATE 6 1.0 KG: FAIL: pre-place planning failed
```

MoveIt reported a 27-state path with an invalid state at index 19 due to an
unallowed `base_link <-> held_product` collision:
`.ros_logs/amr_codex_gate6_loop4_20260821_01/moveit/move_group_558919_1787309292855.log`.
The failing source path is `arm.setStartStateToCurrentState()` followed by
`arm.plan(pre_place_plan)` in `gate6_mass_stage.cpp` (lines 1883-1888); the
new placement stance has collision-free IK/release endpoints, but the OMPL
stow-to-pre-place path enters the chassis with the attached product. No ACM,
geometry, or collision tolerance was relaxed. The stage, recorder, MoveIt, and
factory processes were stopped immediately after this boundary. Placement,
detachment, empty stow, and final success remain unclaimed.

## Phase 14 continuation — Placement calculations, final-bias correction, and product-101 closure — 2026-08-21

The placement coordinates were evaluated against the production URDF and joint
limits before being used. The analysis included FK/IK reachability at the
release and pre-place heights, the Nav2 0.07 m terminal-position tolerance,
the 0.15 m bounded alignment segment, and attached-product collision geometry.
It also exposed that the first kinematic candidates were not sufficient: the
held product could intersect `base_link` during the loaded-stow-to-pre-place
transition, and the achieved pose could differ from the commanded pose by the
Nav2 tolerance. No ACM, URDF/SRDF geometry, or safety tolerance was relaxed.

The final source correction in
`src/amr_manipulation/src/gate6_mass_stage.cpp` obtains a fresh ground-truth /
localized pose pair immediately before the final heading goal, computes the
fresh bias, rejects a non-finite or out-of-envelope bias change, and composes
the final command from the physical placement pose and that fresh bias. The
contract test in `src/amr_manipulation/test/test_moveit_config.py` asserts this
ordering. Focused and five-package verification remained green:

```
colcon test-result --verbose: 179 tests, 0 errors, 0 failures, 5 skipped
```

Intermediate runtime evidence was captured and stopped at each boundary:

- Loop 5 rejected the older placement stance because its derived total motion
  exceeded the existing 0.35 m bound.
- Loop 6 reached alignment but failed the unchanged yaw envelope by 0.0014 rad
  (`0.1514 > 0.15`).
- Loop 7 reached final heading but failed the unchanged physical XY envelope:
  `0.0742752 m > 0.07 m`; this produced the fresh-bias correction above.
- Loop 8 was an invalid launch setup (`initial_x=-4.5, initial_y=0`) and was
  stopped at the first bilateral-contact gate; it never attempted a valid
  pickup.
- Loop 9 was stopped before MoveIt/stage because the factory localization
  lifecycle stalled while configuring `map_server`; no product result was
  claimed.

The corrected product-101 acceptance run was performed once with
`GZ_PARTITION=amr_codex_gate6_loop10_20260821_01`, `ROS_DOMAIN_ID=140`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, initial pose `(2.4, 3.0, 0)`, and evidence
bag `.ros_logs/amr_codex_gate6_loop10_20260821_01/bag/product101_evidence_run`
(339.9 MiB, 714292 messages, 154.878 s). The stage log is
`.ros_logs/amr_codex_gate6_loop10_20260821_01/stage/gate6_mass_stage_637839_1787313233340.log`.
The run evidence is:

- MoveIt logged 100% Cartesian completion for the continuous retreat and all
  placement Cartesian segments; loaded planning used the conservative
  `RRTConnect` edge validation and completed successfully.
- Bilateral contact, attachment, continuous retreat, validity/stow, and the
  out-of-dispatch detachment rejection passed.
- Dock egress succeeded for `0.500 m` in `9.005 s`; decoded egress commands
  were capped at `0.1000000015 m/s`.
- Pickup approach, dispatch approach, and dispatch dock all returned code 4.
  All five placement translation segments and the final heading goal returned
  code 4. The final heading evidence was localized XY error `0.058 m` and yaw
  error `0.135 rad`.
- Decoded attachment state was `detached -> attached -> detached`; final
  manipulation state was `STOWED_EMPTY`, `product_attached=false`, with detail
  `Gate 6 1.000000 kg grasp, transport, placement, and empty stow passed`.
- The final product pose was `(-4.094762, 0.505473, 0.075000)`, `0.007576 m`
  from the slot target `(-4.10, 0.50, 0.075)`.
- The terminal stage line was:

  `GATE 6 1.0 KG COMPLETE 1 KG PASS`

All factory, MoveIt, stage, and rosbag processes were stopped afterward and a
process scan found no remaining runtime processes. The rosbag recorder warned
that hidden action topics were not recorded because `--include-hidden-topics`
was not supplied; terminal outcomes remain in the stage log and the run did
complete successfully. MoveIt emitted its known destructor exit-11 during the
deliberate post-run shutdown, after the successful terminal result and not
during the acceptance path.

## Phase 14 continuation — Requested recheck stopped at deterministic placement envelope — 2026-08-21

The requested fresh recheck used
`GZ_PARTITION=amr_codex_gate6_loop12_20260821_01`, `ROS_DOMAIN_ID=142`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and the validated initial pose `(2.4, 3.0,
0)`. Factory localization, controller-manager, RotationShim/MPPI activation,
MoveIt, and complete hidden-topic rosbag recording all became ready.

The run passed grasp, bilateral contact, continuous retreat, loaded stow,
negative detachment rejection, egress, pickup approach, dispatch approach,
dispatch dock, five bounded alignment translations, and the final heading
goal. It then failed closed before pre-place planning:

```
GATE 6 1.0 KG: FAIL: placement target was outside the deterministic IK envelope
```

The exact guard is `gate6_mass_stage.cpp:1846-1848`, using the unchanged
`kMaxPlacementReleaseRadius = 0.785` at line 1609. The final fresh ground-truth
pose in the evidence bag produced release-base coordinates
`(0.535057, -0.579631)` and radius `0.788834 m`, exceeding the guard by
`0.003834 m` (3.8 mm). This is a safety-envelope rejection, not an OMPL or
controller crash. The stage log is
`.ros_logs/amr_codex_gate6_loop12_20260821_01/stage/gate6_mass_stage_651186_1787313980084.log`;
the complete evidence bag is
`.ros_logs/amr_codex_gate6_loop12_20260821_01/bag/product101_evidence_run`
(329.6 MiB, 687334 messages, including navigation action feedback/status).

All factory, MoveIt, stage, and rosbag processes were stopped immediately
after this boundary. No coordinate, tolerance, geometry, or safety parameter
was changed and no further retry was performed.

## Manual developer handoff — latest GUI navigation oscillation — 2026-08-21

The user will continue the remaining work manually. Do not claim that the
product-101 path is reliable: loop 10 completed once, loop 12 later failed the
deterministic placement envelope, and the latest GUI rerun failed earlier at
the dispatch approach. No source or configuration file was changed during the
latest rerun. All Gazebo, ROS, MoveIt, and stage processes were stopped after
the failure, and a process scan found none remaining.

### Latest reproducible failure

The latest run used `headless:=false`, initial pose `(2.4, 3.0, 0)`,
`GZ_PARTITION=amr_codex_gate6_factory_only_20260821_01`, `ROS_DOMAIN_ID=143`,
and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Grasp, bilateral contact, attachment,
the continuous retreat, loaded stow, negative pickup detachment rejection,
the 0.500 m egress, and the pickup approach all passed. The loaded navigation
to the registered dispatch approach then visibly moved back and forth and
failed closed:

```
target=(-2.500, 0.000, 3.142)
terminal code=6 (ABORTED)
localized=(-2.400, -0.052, -2.609)
xy_error=0.113 m
yaw_error=0.533 rad
distance_remaining=0.000 m
simulation navigation time=40.402 s
GATE 6 1.0 KG: FAIL: navigation to dispatch approach failed
```

The controller accepted the goal, emitted 225 instances of `Control loop
missed its desired rate of 20.0000Hz`, then logged `Failed to make progress`
and aborted the `follow_path` handle. The stage and controller evidence is:

- `.ros_logs/amr_codex_gate6_factory_only_20260821_01/stage/gate6_mass_stage_662541_1787314561693.log`
- `.ros_logs/amr_codex_gate6_factory_only_20260821_01/factory/controller_server_657615_1787314422297.log`

This is a navigation terminal-heading/progress failure, not a grasp, MoveIt,
attachment, or placement failure. The path distance had reached zero while
the unchanged `0.07 m` XY and `0.15 rad` yaw goal conditions were still not
met. The unchanged `PoseProgressChecker` then correctly aborted after the AMR
failed to make the required `0.20 m` or `0.20 rad` progress within 10
simulation seconds. The mission supervisor only propagated the downstream
abort as `path following failed`; its action lifecycle did not crash.

### Source path responsible

The current transport code sends the registered dispatch approach as one
combined translation-and-final-heading goal:

- `src/amr_manipulation/src/gate6_mass_stage.cpp:634-708` constructs and
  monitors every `NavigateToPose` goal.
- `src/amr_manipulation/src/gate6_mass_stage.cpp:1575-1580` directly calls
  `navigate_to(product.dispatch_approach, 120s)` and then the dispatch dock.
- `src/amr_factory/config/stations.yaml:28-32` defines the unchanged dispatch
  approach `(-2.5, 0.0, pi)` and dock `(-3.4, 0.0, pi)`.
- `src/amr_mpc_controller/config/controller.yaml:3-48` owns the 20 Hz
  controller, unchanged progress/goal tolerances, RotationShim wrapper, and
  MPPI workload. Do not begin by weakening these safety/acceptance values.
- `src/amr_mission/src/mission_supervisor_node.cpp:304-376` forwards the
  planner path to `FollowPath`; lines 349-374 clear the controller handle and
  propagate a non-success result. This code is not the source of the
  oscillation in the latest run.

From the fresh pickup-approach terminal pose, the straight travel bearing to
the dispatch approach is approximately `-2.51 rad`, while the same goal also
requires the final station yaw `pi` (equivalently `-3.142 rad`). The controller
therefore consumes the diagonal path and must solve the remaining heading at
the same terminal boundary. The zero remaining path distance, large terminal
heading error, visible reversing, repeated controller-rate misses, and final
progress abort are consistent with that coupled goal being the immediate
failure mechanism.

### Smallest proposed manual fix

Do not change the station registry, footprint, inflation, velocity and
acceleration limits, `0.07 m` / `0.15 rad` goal tolerances, progress checker,
attachment gates, or cancellation semantics. Change only the dispatch-
approach sequencing in `gate6_mass_stage.cpp`:

1. After the existing pickup-approach success, obtain a fresh localized pose.
2. Build a translation target using the registered dispatch-approach X/Y and
   `atan2(target_y - current_y, target_x - current_x)` as its travel yaw.
3. Navigate to that translation target with the existing `navigate_to(...,
   120s)` helper and require the product to remain attached.
4. Obtain another fresh localized pose, then send a second, same-position goal
   using that achieved X/Y and the unchanged registered dispatch-approach yaw.
   This resets `PoseProgressChecker` for the terminal rotation and prevents
   MPPI from fighting the diagonal path heading and final station heading in
   one action.
5. Re-prove fresh attachment after both subgoals, then retain the existing
   registered dispatch-dock goal unchanged.

The already-implemented placement-alignment pattern at
`gate6_mass_stage.cpp:1693-1705` computes travel-bearing translation goals;
the pattern at lines `1721-1767` performs a separate final-heading goal. Reuse
that sequencing concept without copying its placement-specific coordinate,
bias, or displacement logic into the dispatch approach.

Update `src/amr_manipulation/test/test_moveit_config.py` to reject the old
direct `navigate_to(product.dispatch_approach, 120s)` call and assert: fresh
pose before bearing calculation, registered X/Y retained, `atan2` travel yaw,
translation navigation before heading navigation, fresh pose between them,
unchanged registered final yaw, and attachment proof after each action. This
proposal is evidence-backed but has not been implemented or runtime-verified.

If a focused headless rerun still reports controller-rate misses after the
goal split, stop and measure that timing problem separately. Do not guess a
new `controller_frequency`, MPPI `batch_size`, model horizon, velocity limit,
progress threshold, or tolerance during the same change. The latest GUI run
proves an overrun exists, but it does not isolate which MPPI workload change
would be safe.

### Expected next boundary after navigation

Even if dispatch navigation is corrected, placement is not yet proven
repeatable. Loop 12 reached the deterministic release-radius guard at
`src/amr_manipulation/src/gate6_mass_stage.cpp:1846-1848`: the achieved
release radius was `0.788834 m`, exceeding the unchanged `0.785 m` envelope by
`0.003834 m`. Do not raise or remove that guard. If it recurs, correct the
bounded placement-alignment goal/terminal-pose strategy so the fresh achieved
pose stays inside the existing envelope, then keep the exact IK, collision,
30 mm placement, attachment, and detachment gates unchanged.

### Manual verification order

After the source and contract-test edit, run and require zero failures:

```bash
GZ_VERSION=harmonic colcon build --packages-select \
  amr_description amr_mpc_controller amr_mission amr_manipulation amr_factory \
  --symlink-install

colcon test --packages-select \
  amr_description amr_mpc_controller amr_mission amr_manipulation amr_factory

colcon test-result --verbose
```

Then use a new `GZ_PARTITION`, `ROS_DOMAIN_ID`, and log directory for one
headless product-101 run with hidden action topics recorded. Inspect the
controller log for missed-rate warnings and require both dispatch-approach
subgoals, dispatch dock, bounded placement alignment, deterministic IK
preflight, collision-checked placement, valid detachment, empty stow, and the
exact terminal line `GATE 6 1.0 KG COMPLETE 1 KG PASS`. Stop at the first
failed boundary and stop every runtime process. Do not proceed to product
102/103 or later gates until product 101 completes reliably in a fresh run.

## Bounded CAD-visual corrective revision — 2026-08-24

The approved corrective scope keeps the legacy export untouched and uses the
fail-closed derived mesh step for visuals. The active Xacro uses CAD meshes for
appearance and primitive chassis/wheel/caster/sensor collisions, preserving the
public frames, topics, watchdog, DiffDrive, pose/joint-state plugins,
fail-closed ownership boundary, and `1.20 x 0.80 m` footprint. Provisional
values remain drive radius `0.1128 m`, separation `0.566 m`, base height
`0.0478 m`, caster radius/width `0.0393/0.0421 m`, and base mass `22.15 kg`
with positive box inertia. The untouched `amr_urdf_cad` source is not edited.

The composite removes `arm_pedestal_link` and mounts the six-joint articulated
KUKA directly at `base_link` `xyz="0 0 0.33"`; generic payload remains
base-only default-on and composite default-off. Wheel odometry and Gazebo use
the same radius/separation.

Focused source validation now covers the mesh derivation, description,
localization, controller, mission, factory, and manipulation contracts. No
live Gazebo, MoveIt, factory, or Gate 6 acceptance evidence is claimed here.

## Phase 14 source continuation — 2026-08-24

The current source state supersedes the earlier “primitive-only” wording above,
but not the historical runtime results. Derived CAD meshes are active as
visuals with explicit CAD colors; chassis, wheel, caster, and sensor collisions
remain conservative primitives. The baked arm, mounting plate, and centered
lower pedestal are excluded from the derived base visual. The composite has no
`arm_pedestal_link`; the articulated `KR6 R900-2` mounts directly on `base_link`
at `z=0.33`, flush with the AMR top.

The controller source now uses direct Humble Regulated Pure Pursuit with a
provisional 0.50 m/s target, curvature regulation, and approach slowdown. The
factory world source uses a 2 ms physics step, RTF 1.0, and disabled shadows.
The precise placement endpoint shares the mission supervisor's one-goal
reservation/cancel state and selects a non-stateful 5 mm XY checker; normal
navigation remains 70 mm/0.15 rad. Gate 6 dispatch is split into bearing
translation, fresh-pose registered-yaw rotation, and dock navigation, while
placement alignment/final heading use the precise endpoint only.

Gate 7 source boundaries are implemented and fail closed: manipulation action
and status, factory transport action, mode service, manual/autonomous FIFO
capacity, cancellation, held-product fault retention, factory demo launch,
CLI, and interface ownership entries. They are not runtime acceptance
evidence.

Fresh source validation after this continuation:

- focused Python contracts: 41 passed;
- focused C++/ROS build: `amr_interfaces`, `amr_mission`, `amr_manipulation`,
  `amr_factory`, and `amr_mpc_controller` passed;
- full workspace `colcon build --symlink-install`: all 17 packages succeeded;
- full workspace `colcon test-result --verbose`: 202 tests, 0 errors, 0
  failures, 5 skipped;
- `check_urdf` passed for the base and composite URDFs, and `gz sdf -p` passed
  for the corresponding SDFs with `SDF_PATH` set; and
- no live Gazebo/MoveIt/factory run or runtime acceptance has been claimed for
  the 2 ms/RPP revision.

The next authorized runtime pass must use fresh partitions and stop at the first
failed gate: performance baseline, empty motion, two consecutive 1 kg passes,
then 3 kg and 5 kg. Gate 7 orchestration runtime remains blocked until Gate 6
is repeatable. `AMR_CODEX_HANDOFF.md` remains protected and was not edited.

## Next-session handoff — Phase 14 runtime validation — 2026-08-24

Objective: validate the current 2 ms/RPP/CAD-visual factory stack live, starting
at the first unproven runtime gate.

Baseline: all 17 packages build; full `colcon test-result` reports 202 tests,
0 errors, 0 failures, and 5 skipped; base/composite `check_urdf` and factory
`gz sdf -p` pass with `SDF_PATH`; no live runtime acceptance exists for this
revision.

Current source facts: derived CAD meshes are visuals; chassis, wheel, caster,
and sensor collisions are primitives; no mounting plate or pedestal is retained;
the KUKA mounts directly on `base_link` at `z=0.33`; RPP desired speed is
`0.50 m/s` with approach/curvature regulation; factory targets RTF 1.0 with a
2 ms physics step and shadows off.

Critical Gate 7 limitation: the manipulation supervisor public action is
fail-closed because the Gate 6 executor hook is not wired. Do not claim Gate 7
completion or bypass/duplicate motion ownership.

Preserved invariants/non-goals: fail-closed command ownership and the existing
frames, topics, footprint, tolerances, stow, attachment, contact, and place
gates remain unchanged. Do not tune or weaken gates, install/download anything,
commit, or push; `AMR_CODEX_HANDOFF.md` remains protected and untouched.

Next authorized runtime order: use fresh `GZ_PARTITION`, `ROS_DOMAIN_ID`, and
`ROS_LOG_DIR` for the performance/RTF baseline, Gate 6 empty motion, two
consecutive 1 kg passes, then 3 kg and 5 kg; stop at the first failed gate.
Only after repeatable Gate 6 may Gate 7 integration/runtime proceed.

Worktree warning: the tree is heavily dirty; rerun `git status --short` and
preserve everything. Follow `AGENTS.md`: use Sol/high for analysis/evidence,
then Luna/max for approved implementation and focused verification.

## Phase 14 Gate 6 runtime evidence — 2026-08-24

The first authorized Gate 6 performance pass used `GZ_PARTITION=gate6_d151_perf_empty_20260824_01`, `ROS_DOMAIN_ID=151`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, headless startup, and initial pose `(2.4, 3.0, 0)`. Factory localization activated, controllers loaded, and MoveIt became ready. The applicable baseline remains fresh: `colcon test-result --verbose` reports 202 tests, 0 errors, 0 failures, and 5 skipped.

After warmup, the raw 65 s capture was bounded to the exact first 60 s window: 389 samples spanning `59.941606464 s`; median RTF `0.016324721300072727`, minimum `0.0082633773367679405`, maximum `1.2633751952704286`, mean `0.38295381894476921`, and aggregate sim/real advance `0.3236483161600171`. The required median RTF `>= 0.90` therefore **FAILS**. No `Control loop missed its desired rate` warnings were emitted. The stop rule was applied: empty motion was not started; command ownership/profile checks and bag capture were not performed/applicable; domain 152 plus product 101 runs, products 102/103, and Gate 7 were not started.

Evidence supports, but does not prove as the sole cause, a likely environment/rendering bottleneck: runtime logged a failed MESA DRM query, failed iris driver, missing `/dev/dri/card1`, and absent `/dev/dri`; the active world uses DART at 2 ms/OGRE2 with two `720x4` 10 Hz GPU lidars and a `640x480` 10 Hz RGB-D sensor; the RGB-D consumer received camera info but zero image messages; and the trace alternated approximately `0.01` stalls with partial recoveries. No retry, knob, or profile change is authorized from this evidence.

Artifacts: `.ros_logs/gate6_d151_perf_empty_20260824_01/evidence/rtf_60s_summary.txt`, `stats_60s.txt`, `factory/console.log`, `moveit/console.log`, and `evidence/shutdown_process_scan.txt`. The shutdown scan contained only its header and found no Gazebo, ROS, MoveIt, recorder, or Gate 6 processes. Gate 6 remains **NOT runtime accepted**; Gate 7 is explicitly pending and out of scope. This update changes no source, configuration, tuning, interface, or tests; no commit or push was performed. `AMR_CODEX_HANDOFF.md` remains protected.

## Authoritative current status — D205 Gate 6 product 101 — 2026-08-24

The authorized runtime scope was exactly one complete 1 kg Gate 6 run. D205
accepted product 101 once; no second 1 kg run, 3 kg run, 5 kg run, or Gate 7
run was authorized or started. Gate 7 remains pending.

Verified run identity and artifacts: `GZ_PARTITION=gate6_d205_product101_loop28_20260824_01`,
`ROS_DOMAIN_ID=205`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, headless startup,
initial pose `(2.4, 3.0, 0)`, and artifact path
`.ros_logs/gate6_d205_product101_loop28_20260824_01`. The supplied loop26
directory name is not present; this loop28 path is the exact current D205
directory. Strict factory readiness passed in 16 s, MoveIt readiness in 1 s,
the adapted rear-lidar sample, ownership checks, and recorder checks passed.

Performance passed with median RTF `0.99981793313616918` (approximately
`0.999818`), aggregate RTF `0.9945095342459388` (approximately `0.994510`),
and zero controller-rate misses. The stage produced the exact terminal line
`GATE 6 1.0 KG COMPLETE 1 KG PASS`. The pickup bearing/heading split,
corrected dock target, four-segment placement alignment and final heading,
map-aligned held-product collision-free IK/OMPL/lower sequence, `0.000894 m`
placement error, native `detached -> attached -> detached` state sequence,
37-point/100% retreat, ACM restoration, post-retreat state-validity check,
and empty stow all passed. The observed command caps remained `0.50 m/s` and
`0.40 rad/s`; bag analysis found no unmatched simulation profiles. The bag was
`239.8 MiB` with `412,523` messages, and shutdown completed cleanly.

The verdict's `BAG_ANALYSIS=FAIL` was a stale analyzer false report: it
required `/amr/mission/navigate_to_pose_precise/_action/status`, while the
precise action is retired by design. D205 correctly recorded zero messages on
that retired action and used normal navigation status; zero precise messages
is therefore expected, not a runtime failure. Historical failed runtime
sections above remain unchanged.

## Phase 14 continuation — independent 3 kg and 5 kg preparation implementation — 2026-08-28

The user selected separate, independently runnable 3 kg and 5 kg tests. The
accepted 1 kg command and operational path remain unchanged; do not rerun the
1 kg simulation as part of this split. The AMR must retain its current pose,
while the selected product is reset to its registered pickup-station pose at
the beginning of the test. The user also required reset refusal when the arm
is attached, deployed, moving, faulted, or not at empty stow, and requested no
subagents.

### Implemented source path

- `src/amr_manipulation/scripts/gate6_product_test.py` is the persistent
  preparation runner and accepts only product IDs 102 and 103.
- `gate6_3kg_test.launch.py` selects product 102; `gate6_5kg_test.launch.py`
  selects product 103. Each runner keeps the factory and MoveIt session alive.
- Preparation confirms fresh base/arm/product/attachment evidence, checks for
  an active Gate 6 stage, detaches the startup attachment transitions, pauses
  Gazebo, sets only the selected product pose, unpauses, verifies reset
  stability and unchanged AMR/unselected-product poses, then navigates from
  the AMR's current pose to the selected pickup dock.
- Preparation publishes `FAULT` and stops fail-closed on invalid authority,
  unsafe arm state, reset-service failure, navigation failure, or timeout;
  timed-out navigation goals are explicitly canceled. A per-ROS-domain lock
  prevents concurrent product preparation runs.
- `factory_localization.launch.py` bridges the Gazebo world control and
  set-pose services. The existing mass stage is reused for the grasp,
  transport, placement, and empty-stow sequence. Its terminal message is now
  mass-aware; the 1 kg line remains exactly `GATE 6 1.0 KG COMPLETE 1 KG PASS`.
- Focused contract coverage was added in
  `src/amr_manipulation/test/test_product_test_contract.py`; the command
  reference and beginner guide document the separate test commands and RViz
  option.

### Fresh source validation

- `PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile ...`: pass.
- `python3 -m pytest -q src/amr_manipulation/test/test_product_test_contract.py`:
  5 passed.
- `colcon build --packages-select amr_factory amr_manipulation
  --symlink-install`: pass.
- `colcon test --packages-select amr_factory amr_manipulation` followed by
  `colcon test-result --verbose`: 220 tests, 0 errors, 0 failures, 5 skipped.
- Both alias launch files parsed successfully with `--show-args`; the
  installed executable is `amr_manipulation gate6_product_test`.

### Current boundary and next action

Live 3 kg and 5 kg runtime evidence has not been collected. Gate 6 therefore
remains open: product 101 is accepted from D205, but products 102 and 103 are
not runtime accepted. Run the 3 kg alias first, stop at its first failed gate,
and run the 5 kg alias only after the 3 kg result is reviewed. Do not use the
optional factory supervisor at the same time as a manual product test. Gate 7
and completion documentation remain blocked until the independent 3 kg and
5 kg runtime checks pass. `AMR_CODEX_HANDOFF.md` remains untouched.

## Phase 14 continuation — renderer guard and runtime preflight — 2026-08-28

### Objective and diagnosis

The objective was to prevent the Gazebo black-screen/lag condition from being
mistaken for a valid Gate 6 runtime and to make performance evidence
repeatable for both headless and GUI demonstrations.

The degraded environment had no accessible `/dev/dri/renderD*`, no
`video`/`render` group membership, and `software_rendering:=auto` selected
llvmpipe. Gazebo server and GUI each consumed more than one CPU core; a fresh
10 s sample contained 660 observations with aggregate RTF `0.2251409655` and
median RTF `0.7081588`. Gate 6 then correctly failed closed when its unchanged
freshness evidence became too old. The accepted D205 hardware-rendered run
remains the reference: median RTF `0.9998179331`, aggregate RTF `0.9945095342`,
zero controller-rate misses, and exact terminal line
`GATE 6 1.0 KG COMPLETE 1 KG PASS`.

### Implemented source changes

Only the following paths were changed for this continuation; all other dirty
worktree content is user-owned and must be preserved:

- `src/amr_factory/launch/factory_localization.launch.py`
  - Added `require_hardware_rendering:=false`.
  - Strict mode rejects forced software OpenGL and missing or inaccessible
    `/dev/dri/renderD*` before Gazebo starts.
  - Existing `software_rendering:=auto|true|false` behavior remains available
    when strict mode is disabled.
- `src/amr_factory/launch/factory_demo.launch.py`
  - Forwards both rendering arguments to the localization launch.
- `src/amr_factory/scripts/factory_runtime_preflight.py`
  - Added installed `host` and `runtime` checks.
  - Host mode records DRM access, forced-renderer variables, group membership,
    and stale simulation processes without killing anything.
  - Runtime mode verifies Gazebo owns a DRM device and captures `/stats` for a
    fixed 12 s window. It requires at least 10 samples, median RTF `>= 0.90`,
    and aggregate RTF `>= 0.90`.
- `src/amr_factory/CMakeLists.txt`
  - Installs the preflight executable and its pytest contract.
- `src/amr_factory/test/test_factory_assets.py`
  - Covers the strict rendering launch contract.
- `src/amr_factory/test/test_factory_demo_contract.py`
  - Covers rendering argument passthrough.
- `src/amr_factory/test/test_factory_runtime_preflight.py`
  - Covers DRM detection, forced software detection, D205-like passing stats,
    degraded stats, insufficient samples, and non-positive time spans.
- `docs/SIMULATION_COMMANDS.md`
  - Documents strict launch arguments, preflight order, RTF gates, absolute
    `ROS_LOG_DIR`, corrected rosbag continuation syntax, and the rule to keep
    RViz and the optional factory supervisor out of performance evidence.

### Validation evidence

- `GZ_VERSION=harmonic colcon build --packages-select amr_factory
  --symlink-install`: passed.
- `colcon test --packages-select amr_factory`: passed.
- `colcon test-result --test-result-base build/amr_factory --verbose`: 27
  tests, 0 errors, 0 failures, 0 skipped.
- Focused pytest: 19 passed.
- Python compilation and `ros2 launch ... --show-args`: passed.
- Installed command is discoverable as
  `ros2 run amr_factory factory_runtime_preflight.py`.
- Current host preflight deliberately failed with
  `render_devices=<none>` and exit 1.
- Strict launch deliberately aborted before Gazebo with
  `hardware rendering required, but no readable/writable
  /dev/dri/renderD* device is available`.
- The new parser independently reproduced the D205 reference as 115 samples,
  median `0.999818`, aggregate `0.994510`.

### Preserved invariants and non-goals

Do not weaken Gate 6 freshness, timing, controller, contact, attachment,
placement, stow, command-ownership, or fail-closed behavior. Do not reduce
world sensors, alter physics, tune motion limits, add a software fallback to
strict evidence mode, run the optional factory supervisor beside manual Gate 6,
or start products 102/103 or Gate 7 from this handoff. Do not commit, push,
install dependencies, download assets, or modify `AMR_CODEX_HANDOFF.md`.

### Current boundary and next action

No live Gate 6 or GUI evidence pass was started by this continuation. No
Gazebo, MoveIt, rosbag, or Gate 6 processes were intentionally left running.
The current agent environment lacks `/dev/dri`, so it cannot prove the GUI
stage. Runtime evidence must be authorized after source review and run from a
direct host terminal with accessible DRM render nodes and matching
`GZ_PARTITION`, `ROS_DOMAIN_ID`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and
workspace-local `ROS_LOG_DIR` in every terminal.

Run the host preflight first, then strict headless localization, then the
runtime RTF check. Stop before MoveIt, recording, or the product stage on any
failed check. A GUI repeat may proceed only after the headless RTF and Gate 6
boundaries pass; keep RViz out of the measured GUI run. Preserve the fresh
preflight reports, raw stats, stage bag, logs, screenshots, and shutdown scan
under the run-specific `.ros_logs` directory.

## Phase 14 continuation — direct-host 1 kg runtime result — 2026-08-28

### Objective and runtime boundary

The user requested a fresh full 1 kg simulation after restoring the current
working source. The run used the direct host because the normal sandbox has no
visible `/dev/dri`; no source, physics, sensor, motion-limit, tolerance, or
rendering fallback changes were made for this run.

The repository was rebuilt and tested before runtime. The run used strict
headless hardware rendering with `GZ_VERSION=harmonic`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `ROS_LOCALHOST_ONLY=1`,
`ROS_DOMAIN_ID=231`, `GZ_PARTITION=amr_gate6_1kg_host_20260828_01`, and
`ROS_LOG_DIR=/home/pete/amr_ws/.ros_logs/gate6_1kg_host_20260828_01`.

### Validation and runtime evidence

- `colcon build --symlink-install --executor parallel --parallel-workers 4`:
  passed all 17 packages.
- Full `colcon test --executor parallel --parallel-workers 4`: 228 tests,
  0 errors, 0 failures, 5 skipped.
- Host hardware preflight: `PASS`; Gazebo used `/dev/dri/renderD128`.
- Runtime preflight: `PASS`; 3,586 samples over 11.994 s, median RTF
  `0.9999445031`, aggregate RTF `0.9963141486`.
- Navigation lifecycle nodes and arm/gripper controllers were active; both
  front and rear acceptance LiDAR topics published.
- MoveIt became ready and the prescribed recorder captured 262,194 messages
  in a 143.6 MiB bag over 144.394 s.

### Terminal result

The stage passed gripper setup, bilateral stall evidence, pickup, attachment
safety rejection, dock egress, pickup approach, transfer navigation, and
placement alignment. It stopped fail-closed at the placement-lower gate:

`GATE 6 1.0 KG: FAIL: Cartesian placement lower was incomplete`

MoveIt reported only 80% Cartesian completion for the final lower path (33
points). Therefore the 1 kg simulation is not complete or accepted from this
run. Do not claim Gate 6 completion and do not retry or tune from this result
without a reviewed fix plan.

### Artifacts and shutdown

Evidence is preserved under
`.ros_logs/gate6_1kg_host_20260828_01/`, including:

- `evidence/host_preflight.txt`
- `evidence/runtime_preflight.txt`
- `evidence/stats_raw.txt`
- `evidence/stats_stderr.txt`
- `product101_evidence/`
- `gate6_mass_stage_34326_1787890830548.log`
- `move_group_31981_1787890731880.log`

The recorder finalized successfully and the factory shut down. MoveIt emitted
a segmentation fault during SIGINT teardown after the stage had already
failed; this is a separate unresolved shutdown risk and was not the Gate 6
failure. No simulation processes were intentionally left running.

### Current worktree and next action

The worktree remains intentionally dirty with 99 changed/untracked entries;
these include the restored Phase 14 source and prior user artifacts. `HEAD`
remains commit `00b8cfa1d07af043df5ba830e54c55ec5e978ab0`, while the restored
latest source is present as uncommitted working-tree content. Preserve all
unrelated changes. Do not reset, clean, stage, commit, push, or modify
`AMR_CODEX_HANDOFF.md` without explicit direction.

Next action is diagnosis and a bounded fix plan for the incomplete final
Cartesian lower path, followed by focused validation and a fresh strict host
runtime pass. Preserve the fail-closed placement, attachment, ownership,
timing, and safety gates; do not weaken acceptance criteria or overload the
hardware to force completion.

## Phase 14 continuation — factory product startup and GUI diagnosis — 2026-08-28

### Objective and diagnosis

The user reported that factory products were flying and then could not start
the Gazebo GUI. The product motion was reproduced in a clean direct-host
Gazebo partition and traced to the robot xacro, not the factory map or driver:

- `factory.sdf` alone kept products 101/102/103 at their registered shelf
  poses.
- The full robot with `factory_attachment:=true` moved all three products and
  emitted duplicate fixed-joint/shape-name warnings.
- The same full robot with `factory_attachment:=false` kept all three products
  stationary.

The cause is the Gazebo `DetachableJoint` contract: configured pairs start
rigidly attached and detach only after a command. See the
[Gazebo DetachableJoint documentation](https://gazebosim.org/api/sim/9/detachablejoints.html).
The prior factory launch hardcoded `factory_attachment: "true"` even though
the products begin on shelves.

The GUI failure in the restricted agent environment was a separate host
boundary: strict rendering correctly stopped when `/dev/dri/renderD*` was not
visible. A direct-host launch found `/dev/dri/renderD128`, kept `gz sim -g`
running, and `xdpyinfo` connected successfully to `DISPLAY=:0`.

### Implemented source changes

Only these paths were changed for the product-startup fix; preserve all other
dirty worktree content:

- `src/amr_factory/launch/factory_localization.launch.py`
  - Added `factory_attachment:=false` as the ordinary factory default.
  - Resolves the launch-time xacro mapping after launch arguments are known.
  - Keeps native attachment topics and the existing explicit attachment mode.
- `src/amr_factory/launch/factory_demo.launch.py`
  - Forwards the new argument and defaults it to false.
- `src/amr_factory/test/test_factory_assets.py`
  - Covers the launch-time attachment option and default.
- `src/amr_factory/test/test_factory_demo_contract.py`
  - Covers demo passthrough and default.
- `docs/SIMULATION_COMMANDS.md`
  - Shows the normal factory and demo commands with
    `factory_attachment:=false`.

No factory world poses, map geometry, driver, motion limits, command
ownership, attachment proof, or fail-closed gate was changed. The strict
hardware-rendering defaults recorded in the preceding handoff section remain
unchanged.

### Validation evidence

- Focused pytest for factory assets, demo contract, and runtime preflight:
  `19 passed`.
- `colcon build --packages-select amr_factory --symlink-install`: passed.
- `colcon test --packages-select amr_factory --event-handlers
  console_direct+`: all 4 CTest targets passed.
- Python compilation of both factory launch files: passed.
- Direct-host strict headless factory run with the default attachment setting
  used `/dev/dri/renderD128`. Product pose samples remained at
  `product_a=(3.25, 3.0, 0.825)`, `product_b=(3.25, 0.0, 0.825)`, and
  `product_c=(3.25, -3.0, 0.825)`.
- Direct-host GUI startup kept both Gazebo server and GUI processes alive;
  `xdpyinfo -display :0` returned 0. This was a startup diagnosis, not a new
  Gate 6 acceptance run.
- All explicit test Gazebo/ROS processes were stopped afterward. No commit or
  push was made, and `AMR_CODEX_HANDOFF.md` remains untouched.

### Preserved invariants and unresolved risks

The ordinary factory and demo launches now keep shelf products detached and
are suitable for GUI inspection when run from a desktop host with accessible
DRM and X11 devices. The bridge still exposes the declared attachment topics,
but the native Gazebo attachment systems are absent in the default mode.

Gate 6 still requires an explicit `factory_attachment:=true` factory launch
because its native attach/detach proof depends on those systems. Gazebo's
initial attached behavior remains a known property of that explicit mode; the
existing Gate 6 initial-detachment sequence must be revalidated before using
it for acceptance evidence. This continuation does not claim any new Gate 6
pass.

Do not start the independent 3 kg/5 kg runs, Gate 7, or completion
documentation. The authoritative next action remains a diagnosis and bounded
fix plan for the incomplete final Cartesian placement-lower path, followed by
focused validation and a fresh strict host runtime pass. Preserve placement,
attachment, ownership, timing, and safety gates; do not weaken criteria or
modify `AMR_CODEX_HANDOFF.md`.

## Phase 14 continuation — retained placement-lower branch — 2026-08-28

The bounded Gate 6 source correction is implemented in
`src/amr_manipulation/src/gate6_mass_stage.cpp`. The release-to-pre-place IK
continuation now retains every 5 mm solution. After the OMPL pre-place motion,
the executable requires the measured endpoint to match the retained branch
within the existing 0.01 rad goal tolerance, validates every reversed branch
waypoint through `/check_state_validity` with bounds/contact diagnostics, and
time-parameterizes the exact joint trajectory at the unchanged 0.2 scaling.
The release endpoint is checked after time parameterization. Existing
placement, contact, attachment, ownership, timing, and fail-closed gates are
unchanged.

Focused validation is green:

- `colcon build --packages-select amr_manipulation --symlink-install`: passed.
- `colcon test --packages-select amr_manipulation`: 5 tests, 0 failures, 0
  errors.
- `git diff --check` for the changed source and contract test: passed.

The source contract test was updated to assert the retained-branch validation
and execution order. No runtime evidence was started. Gate 6 remains failed
closed at the prior direct-host placement-lower boundary; the next authorized
step is a fresh strict host runtime pass. Do not proceed to 3 kg, 5 kg, Gate 7,
or completion evidence until the full 1 kg acceptance gate passes.

## Phase 14 continuation — runtime host preflight blocked — 2026-08-28

The next authorized validation attempt stopped at the required host preflight
before Gazebo startup. The current agent environment reported
`render_devices=<none>` and failed closed with
`no readable/writable /dev/dri/renderD* device`. Evidence is preserved at
`.ros_logs/gate6_1kg_retry_20260828_02/evidence/host_preflight.txt`.

No simulation, MoveIt, Gate 6 mass stage, 3 kg/5 kg run, or Gate 7 process was
started. A fresh direct-host run with an accessible DRM render node remains
required before Phase 14 can advance.

## Phase 14 continuation — Phase J PASS and Phase K MoveIt boundary — 2026-08-30

The manually verified direct-host Phase J runtime-performance result is
preserved under `.ros_logs/gate6_1kg_retained_20260830_01/` with
`AMR_RUN_ID=gate6_1kg_retained_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_retained_20260830_01`, and `ROS_DOMAIN_ID=206`.
The runtime report records `/dev/dri/renderD128`, no forced software renderer,
3,600 samples, aggregate RTF `0.999999929313705`, median RTF
`1.000014400208803`, and `verdict=PASS`. Treat Phase J as complete; do not
redo or tune it.

Phase K source inspection confirmed that the existing project-owned MoveIt
launch uses the authoritative `phase14_mobile_manipulator.urdf.xacro`,
`phase14_mobile_manipulator.srdf`, `manipulator` group, OMPL pipeline, and the
actual `arm_controller`/`gripper_controller` names, with joint states remapped
to `/amr/base/joint_states`. The composite Xacro passed `check_urdf`, and
`test_moveit_config.py` passed 4 tests. A bounded MoveIt smoke loaded the
composite model, OMPL, both controller adapters, and MoveGroup capabilities;
its log is
`.ros_logs/gate6_1kg_retained_20260830_01/move_group_18_1788097414517.log`.

The first integrated readiness check returned `Node not found` for
`/amr/command_arbitration_node`, because this restricted environment cannot
see or run the direct-host Phase J factory/Nav2 graph. No source/configuration
or validated Phase J setting was changed. The smoke process was stopped and a
process scan found no Gazebo, MoveIt, recorder, or Gate 6 processes. No
Product 101 run was started.

Next action: on the direct Ubuntu host, keep the exact Phase J factory runtime
and environment alive, start `ros2 launch amr_manipulation move_group.launch.py`,
complete the Phase K lifecycle/controller/action checks, start the prescribed
hidden-topic recorder, and run exactly one `product_id:=101` stage. Stop at the
first failed check and preserve all evidence. Do not start 3 kg, 5 kg, or Gate 7.

## Phase 14 continuation — Phase K integrated readiness and Product 101 PASS — 2026-08-30

Phase J remains closed and its performance evidence was not rerun or changed.
Phase K was completed on the direct Ubuntu host using the exact preserved
environment (`AMR_RUN_ID=gate6_1kg_retained_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_retained_20260830_01`, `ROS_DOMAIN_ID=206`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `ROS_LOCALHOST_ONLY=1`, and the existing
workspace-local `ROS_LOG_DIR`). The strict headless factory/Nav2 graph was
started with `factory_attachment:=true`, followed by the project-owned
`move_group.launch.py`.

The first direct-host preflight found two stale same-partition Gazebo
processes, producing duplicate controller-manager/Gazebo-control graph
entries. Only the identified stale PIDs were stopped. The corrected host
preflight passed with `/dev/dri/renderD128`, no forced software renderer, and
no known simulation processes. A first lifecycle CLI query briefly returned
`Node not found` during service discovery; after the graph settled, the same
probe and the complete lifecycle set returned `active [3]`. This was a
host-startup/DDS timing boundary, not a proven source defect, and no source or
configuration change was made.

Phase K readiness then passed: all 17 lifecycle nodes were active; the
joint-state, arm, and gripper controllers were active exactly once; visible
node names had no unexpected duplicates; all required navigation, egress, arm,
and gripper actions had one server; required services and Product 101 topics
were present; MoveGroup actions and `/query_planner_interface` responded; and
`/amr/control/cmd_vel` had exactly one publisher owned by
`/amr/command_arbitration_node`.

The prescribed hidden-topic recorder reported `Recording...` before exactly
one Product 101 stage. The run passed bootstrap detachment, gripper/contact
proof, pickup, attachment safety rejection, dock egress, pickup approach,
split dispatch navigation, dispatch dock, placement alignment, collision-
checked lower/release, and empty stow. The stage log ended with:

`GATE 6 1.0 KG COMPLETE 1 KG PASS`

The finalized evidence bag is
`.ros_logs/gate6_1kg_retained_20260830_01/product101_evidence/` (200,534
messages, 96.603 seconds). Phase K evidence is under
`.ros_logs/gate6_1kg_retained_20260830_01/phase_k/evidence/`; the runtime
stage and MoveIt logs are `gate6_mass_stage_53626_1788099193302.log` and
`move_group_34342_1788098551620.log`. The final exact-process scan passed with
no Gazebo, MoveIt, rosbag, or Gate 6 processes remaining. MoveIt emitted its
known Humble shutdown destructor segfault after the successful stage; it did
not affect the acceptance path.

Current boundary: Phase K PASS and the single Product 101 run passed, while
products 102/103 and Gate 7 remain unvalidated. Review the complete evidence
before any later work. Do not rerun or tune Phase J, do not start 3 kg, 5 kg,
or Gate 7 from this handoff, and do not modify `AMR_CODEX_HANDOFF.md`.

## Authoritative current status — Gate 6 1 kg pass-2 analyzer boundary — 2026-08-30

Phase J and Phase K remain closed and were not rerun or modified. The next
ordered Phase 14 acceptance boundary was one fresh independent 1 kg pass. It
was attempted once on the direct Ubuntu host with
`AMR_RUN_ID=gate6_1kg_repeat2_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_repeat2_20260830_01`, `ROS_DOMAIN_ID=207`, strict
hardware rendering, and `factory_attachment:=true`.

Host/runtime preflight passed with `/dev/dri/renderD128`, 3,600 RTF samples,
median `0.9999916001`, aggregate `0.9999999890`, and no forced software
renderer. Bootstrap READY/Trigger, all 17 lifecycle nodes, controllers,
actions/services/topics, command ownership, and MoveGroup/OMPL readiness all
passed. The recorder reported `Recording...` before the only Product 101
stage attempt.

The stage passed every product-101 manipulation, navigation, placement,
release, and empty-stow gate and ended with the exact
`GATE 6 1.0 KG COMPLETE 1 KG PASS` line. The finalized bag contains 199,518
messages over 115.618 s under
`.ros_logs/gate6_1kg_repeat2_20260830_01/product101_evidence/`.

The required independent analyzer failed with
`GATE6_BAG_ANALYSIS=FAIL product_id=101`. The exact failure output is in
`.ros_logs/gate6_1kg_repeat2_20260830_01/evidence/product101_analyzer_console.txt`.
The documented recorder omitted bootstrap/rear-LiDAR/arm/gripper status
topics and used incompatible/default QoS for best-effort
`/amr/base/joint_states` and transient-local `/tf_static`; live QoS proof is
in `evidence/analyzer_required_topic_qos.txt`. Separately, timestamp analysis
found `0.000197628 m` of ground-truth displacement immediately after the
stage published `MOVING`/`base_motion_allowed=false`, with the arbitration
command already zero during the stop transition. This is a strict runtime
acceptance/interlock-timing failure, not a basis for weakening the analyzer or
acceptance limits.

The pass-2 attempt is **FAIL** for Gate 6 repeatability. The documented
recorder recipe is corrected in `docs/SIMULATION_COMMANDS.md` to include the
analyzer-required topics and QoS overrides. No source or runtime configuration
was changed, no Product 101 retry was made, and Products 102/103 and Gate 7
were not started. Runtime cleanup passed with no known Gazebo, MoveIt, rosbag,
or Gate 6 processes remaining. The next authorized action is separate bounded
diagnosis of the status-to-zero timing boundary, followed by a fresh valid
pass-2 attempt. Do not advance to 3 kg, 5 kg, Gate 7, or modify
`AMR_CODEX_HANDOFF.md`.

Supporting pass-2 evidence is retained under
`.ros_logs/gate6_1kg_repeat2_20260830_01/`, including host/runtime preflight,
readiness, controller, action-server, MoveGroup, recorder-startup, bag-info,
QoS, analyzer-console, corrected base-motion diagnosis, and final shutdown
process-scan captures. The worktree's pre-existing documentation changes and
the untracked `.ros_logs/` evidence are preserved; no source or configuration
files were changed.

## Authoritative current status — Gate 6 status-to-zero diagnosis and retry boundary — 2026-08-30

Phase K remains PASS. The current execution boundary remains the second
independent 1 kg Gate 6 pass, which is **FAIL / unresolved**. No 3 kg, 5 kg,
Product 102, Product 103, or Gate 7 work was started.

### Diagnosis

The retained pass-2 bag was replayed through the required analyzer and traced
against the stage, arbitration, base-adapter, and odometry paths. The first
forbidden-motion sample occurred at status bag time
`1788100711.375472784`, when sequence 1380 entered `MOVING` with
`base_motion_allowed=false`. The final navigation result had completed at
`1788100711.329048949` in the controller log and
`1788100711.329639878` in the stage log. The last nonzero arbitration sample
was `1788100711.329441786` with linear `0.059288730` and angular
`0.054635909`; arbitration recorded zero at `1788100711.379224539`.

The base command forwarded to simulation remained nonzero through
`1788100711.365955353` and first recorded zero at
`1788100711.416099548`. Ground truth moved from
`1788100711.378139019` to `1788100711.380825758` by exactly
`0.000197628 m` in `0.002686739 s`. Raw odometry was still nonzero at
`1788100711.398223877` and decayed to approximately zero by
`1788100711.584464788`; filtered odometry showed the same settling behavior.
This proves a real simulated base-settling window after the status transition,
not stale odometry or an intentionally continuing forbidden command.

The source cause was the ordering in
`MassStageNode::wait_for_motion_permission`: it published the motion-forbidden
`MOVING` status first and only then waited for fresh READY/odometry stationary
feedback. The 50 ms status publisher could therefore expose the forbidden
state before the arbitration/base-adapter pipeline and plant had settled.

### Source correction and validation

`src/amr_manipulation/src/gate6_mass_stage.cpp` now uses the existing
feedback-qualified condition—fresh valid `BaseStatus::READY`, fresh odometry,
and linear x/y/angular z within the existing `0.01` limits—for 500 ms before
publishing `MOVING`/`base_motion_allowed=false`. It retains the existing
400 ms post-announcement guard and a second 500 ms stationary window. The
duplicate explicit post-detachment `MOVING` publication was removed so that
transition follows the same rule. If feedback never settles, the bounded wait
fails closed and the stage faults; no arm/gripper work is issued. The source
ordering is covered by the new contract check in
`src/amr_manipulation/test/test_moveit_config.py`.

No analyzer threshold, Gate 6 criterion, robot geometry, controller tuning,
physics, tolerance, performance setting, or recorder procedure was weakened
or changed. The corrected recorder/QoS recipe remains in
`docs/SIMULATION_COMMANDS.md`.

Validation passed:

- `amr_manipulation` package build;
- all six package CTest targets;
- `colcon test-result`: 28 tests, zero errors/failures/skips; and
- `git diff --check`.

### Fresh retry attempt

The one authorized fresh retry used
`AMR_RUN_ID=gate6_1kg_repeat2_settlefix_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_repeat2_settlefix_20260830_01`,
`ROS_DOMAIN_ID=208`, strict hardware-rendering settings, and a new evidence
directory at
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_01/`. It stopped immediately
at the documented host preflight because this execution environment reported
`render_devices=<none>` and no readable/writable `/dev/dri/renderD*` device.
The preflight recorded no known simulation processes. No factory, MoveIt,
recorder, Product 101 stage, bag analyzer, or `ros2 bag info` command ran, so
there is no fresh Product 101 or analyzer result to claim. The final process
inspection found no actual Gazebo/ROS runtime process; only the inspection
shell/matcher appeared in the command text.

The next permitted action is to run the full documented clean readiness
procedure on a direct host exposing `/dev/dri/renderD128` (or another approved
readable/writable hardware render node), then perform exactly one fresh
Product 101 attempt with the corrected recorder and required analyzer. Do not
advance to any later mass or Gate 7.

## Authoritative current status — post-fix direct-host readiness stop — 2026-08-30

Phase K remains PASS. Gate 6 second independent 1 kg acceptance remains
**FAIL / unresolved**. Products 102/103 and Gate 7 remain prohibited.

The existing feedback-first status-ordering source fix was not changed during
this run. A new direct-host session used
`AMR_RUN_ID=gate6_1kg_repeat2_settlefix_20260830_02`,
`GZ_PARTITION=amr_gate6_1kg_repeat2_settlefix_20260830_02`,
`ROS_DOMAIN_ID=208`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`,
`ROS_LOCALHOST_ONLY=1`, strict hardware rendering, and
`ROS_LOG_DIR=/home/pete/amr_ws/.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02`.

### Fresh preflight evidence

Host preflight passed with `/dev/dri/renderD128`, no forced software renderer,
and `known_processes=<none>`. Runtime preflight passed with 3,413 valid
`/stats` samples over `11.996123048 s`, simulated span `11.373332196 s`,
median RTF `0.998979242907521`, aggregate RTF `0.948083989343221`, and
`gz_topic_exit_code=0`. The fresh reports are
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/host_preflight.txt`
and `runtime_preflight.txt`.

The strict factory launch used `factory_attachment:=true`; the project-owned
MoveIt launch reached its normal `You can start planning now!` message. The
first integrated graph gate then failed. Six bounded non-daemon ROS graph
queries never returned the complete required 17-node factory set plus
`/move_group`; the missing-node sequence and final observed subset are in
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/readiness_graph_check.txt`.
Because integrated readiness failed, no lifecycle/controller/action/service/
topic/ownership acceptance was declared and the run stopped before the
recorder.

### Stage and analyzer boundary

No `product101_evidence` directory was created in this run. The corrected
hidden-topic recorder was not started, the exact Product 101 launch was not
started, and neither `gate6_evidence_analyzer` nor `ros2 bag info` was run.
Therefore this run has no fresh Product 101 or analyzer result and consumed no
Product 101 attempt. The prior retained run remains the one whose stage passed
but analyzer failed; this run does not alter that evidence.

### Shutdown and current next action

With stage and recorder absent, shutdown was performed in the remaining
documented order: MoveIt, then factory. MoveIt emitted the known Humble
destructor exit `-11` after SIGINT. Factory components shut down; launch then
reported `Cannot shutdown a ROS adapter that is not running` after Gazebo
exited. The final documented process scan showed only its inspection shell and
matcher, and a post-shutdown host preflight reported no known processes. The
captured cleanup summary is
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/shutdown_process_scan_final.txt`.

No source, runtime configuration, recorder procedure, threshold, tolerance,
controller setting, or phase boundary was changed by this attempt. The
appropriate next action is to review and resolve the direct-host ROS graph
discovery/readiness boundary, then use a new run identity for another complete
readiness-gated 1 kg attempt. Do not start any later mass or Gate 7.

## Repository handoff note — 2026-08-30

No staging, commit, remote push, force-push, or history rewrite was performed.
The current worktree intentionally retains the modified status, changelog,
TODO, runtime-debug, simulation-command, source, and regression-test files,
plus the untracked `.ros_logs/` evidence directory. The large recorded evidence
bags remain local and are not part of any commit. `AMR_CODEX_HANDOFF.md` remains
untouched and must not be staged or committed without explicit user direction.

## Readiness evidence review — 2026-08-31

The preserved
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/readiness_graph_check.txt`
was reviewed and supplied to the user verbatim. It is an 11-line summarized
result, not a raw transcript: it records the 18 expected names (17 factory
names plus `/move_group`), six `missing=` sets, one `final_observed_nodes=`
set, `readiness_graph_gate=FAIL`, and the stop rule. It does not contain the
raw node-list output for each attempt, query timestamps, stderr, or the exact
shell/script invocation.

The six missing sets vary materially between attempts. `/move_group` is missing
in attempts 1–3 but is present in the later observed graph, while different
factory nodes appear and disappear across attempts. This is consistent with an
unstable/incomplete graph snapshot or ROS CLI/RMW discovery boundary; the file
does not by itself prove that a particular factory process was continuously
absent or that a query timed out.

The run environment recorded in the authoritative status remains
`ROS_DOMAIN_ID=208`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and
`ROS_LOCALHOST_ONLY=1`. `RMW_IMPLEMENTATION`, the timeout value, and the exact
`--no-daemon` command form were not preserved in the readiness artifact, so
they remain unverified from this evidence alone. No source, runtime, recorder,
threshold, tolerance, controller setting, or phase boundary was changed by
this review. The next action remains bounded diagnosis of the direct-host ROS
graph discovery/readiness boundary before another complete readiness-gated 1 kg
attempt; do not start later mass or Gate 7 work.

## Phase 14 current continuation — graph readiness repair and Gate 6 pass-2 completion — 2026-08-31

The direct-host graph failure is resolved at the observer boundary. The six
changing missing-node sets in
`.ros_logs/gate6_1kg_repeat2_settlefix_20260830_02/evidence/readiness_graph_check.txt`
were produced by repeated fresh Humble `ros2 node list --no-daemon` observers.
The installed Humble direct implementation uses a 0.5 s default discovery
wait; each invocation creates a new participant and its graph snapshot is not
cumulative. The factory and MoveIt processes stayed alive through the failed
run. A bounded isolated experiment reproduced incomplete fresh snapshots and
showed one persistent observer converging to all 18 required nodes in 1.105 s;
removing `/move_group` made the persistent check fail.

The implemented readiness method is a single persistent `rclpy` observer in
`src/amr_factory/scripts/factory_runtime_preflight.py`. It requires all 17
factory nodes plus `/move_group`, rejects duplicate required names, records the
ROS/RMW environment and discovery transitions, and requires a complete unique
graph for 2 s inside a 30 s bound. This remains fail-closed. Direct-host
readiness-only runs
`.ros_logs/gate6_graph_readiness_barrier_20260831_01/` and `_02/` both passed
the graph and full integrated readiness checks.

A separate Humble startup race was then exposed and fixed. The lifecycle
manager's zero-delay autostart could issue `change_state` while
`controller_server` was still constructing its nested local costmap. The
minimum change in
`src/amr_mpc_controller/launch/amr_mpc_controller.launch.py` starts the
controller first and delays only its lifecycle manager by one second. Two
readiness-only direct-host runs passed after this change without the observed
controller response timeout. No controller parameters, ownership boundary,
motion semantics, geometry, mass, tolerance, or performance requirement
changed.

The one permitted Product 101 retry used the new identity
`gate6_1kg_repeat2_graphfix_20260831_03` (`ROS_DOMAIN_ID=228`). Host preflight
passed with `/dev/dri/renderD128`; runtime preflight passed with 3,600 samples,
aggregate RTF `0.9999999013`, and median RTF `1.0000073501`. Persistent graph
readiness and all prescribed lifecycle/controller/action/service/topic/OMPL/
ownership checks passed before recording. The recorder started before exactly
one Product 101 stage. The stage exited 0 with
`GATE 6 1.0 KG COMPLETE 1 KG PASS`; the finalized bag contains 191,936
messages over 90.156199656 s. This consumed the one Product 101 attempt.

The original analyzer returned FAIL on the captured bag because the base
adapter publishes its cached arbitration command on an independent 50 ms timer
and can therefore trail the newest arbitration sample by one tick. Bag replay
showed every nonzero simulation command matched an exact arbitration sample at
or before output within the existing 250 ms bound. The minimum analyzer fix in
`src/amr_manipulation/scripts/gate6_evidence_analyzer.py` now checks that
historical trace; focused regressions cover one-tick forwarding, unowned
values, and stale output. Corrected reanalysis of the actual bag returned
`GATE6_BAG_ANALYSIS=PASS product_id=101`.

Validation: the combined `amr_factory amr_mpc_controller amr_manipulation`
build passed; package tests passed with 271 tests, 0 errors, 0 failures, and 5
skips; and `git diff --check` passed. Evidence is under
`.ros_logs/gate6_1kg_repeat2_graphfix_20260831_03/evidence/`, including
`graph_readiness.txt`, `product101_bag_info.txt`,
`product101_analysis_corrected.txt`, and `shutdown_process_scan.txt`.
Shutdown completed and post-shutdown host preflight found no runtime
processes. Gate 6 now has two valid independent 1 kg passes. Do not start a
further Product 101 attempt or any 3 kg, 5 kg, Product 102/103, or Gate 7 run
without a new explicit evidence review and authorization.

## Authoritative next-session handoff — 2026-08-31

The Phase 14 Gate 6 second independent 1 kg boundary is complete and accepted
after corrected analysis of the one permitted Product 101 retry. The prior
graph-readiness FAIL, controller-startup race, and analyzer FAIL are historical
results; the repaired behavior and final evidence are authoritative in the
section immediately above.

Preserved invariants: the existing feedback-first `gate6_mass_stage` settle
fix was not weakened and passed in the actual stage run; Gate 6 mass,
tolerance, performance, forbidden-motion, geometry, and controller semantics
were not relaxed; no Product 101 attempt was repeated; and no 3 kg, 5 kg,
Product 102/103, or Gate 7 runtime was started.

Current worktree state is intentionally dirty from the implementation,
regression coverage, synchronized documentation, and untracked `.ros_logs/`
evidence. No staging, commit, push, history rewrite, or dependency install was
performed. `AMR_CODEX_HANDOFF.md` remains untouched and protected.

If work resumes, begin with `git status --short` and read this handoff's latest
section. Do not launch another Product 101 run. The next permitted activity is
a separately authorized evidence review deciding whether to advance to later
mass or Gate 7 validation.

## Authoritative higher-mass execution boundary — 2026-08-31

An explicitly authorized combined higher-mass execution was attempted once
under the exact run identity
`.ros_logs/gate6_3kg_5kg_20260831_01` with `ROS_DOMAIN_ID=230`.
The orchestration recorded `t0_monotonic=24880.26`,
`hard_cutoff_monotonic=42520.26`, and `script_start_monotonic=26249.81`.
At script start/leaving, approximately `22:49.55` had elapsed since T0 and
approximately `4:31:10.45` remained before the hard cutoff.
Final handoff timing is recorded in
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/final_timing.txt`: approximately
`34:15.36` had elapsed from T0 and `4:19:44.64` remained to the cutoff.

The required host preflight failed closed before source/installed hashes, the
factory launch, MoveIt, or the recorder. Its evidence is
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/host_preflight/host_preflight.txt`;
it records `render_devices=<none>`, `forced_software=<none>`,
`known_processes=<none>`, and `verdict=FAIL` for the missing readable/writable
`/dev/dri/renderD*` device. Product 102 (3 kg) and Product 103 (5 kg) attempts
remain `0` and were not started. No stage, product runner, recorder, bag,
analyzer, or `ros2 bag info` command ran.

Cleanup passed its process gate; the captured scan is
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/shutdown_process_scan.txt`.
The post-shutdown host preflight found no runtime processes but still failed
the render check at
`.ros_logs/gate6_3kg_5kg_20260831_01/evidence/post_shutdown/host_preflight.txt`.
An elevated rerun was rejected. No retry, software-rendering workaround, or
other bypass is authorized.

Before launch, the authorized deterministic analyzer defect was fixed in
`src/amr_manipulation/scripts/gate6_evidence_analyzer.py` with regression
coverage in `src/amr_manipulation/test/test_gate6_completion_contract.py`:
preparation publishes a first nonzero boot stream, and the analyzer now
fail-closes/selects exactly one later stream carrying `Gate 6 mass stage is
starting`; single-stream Product 101 compatibility is preserved. The live
higher-mass attempt itself changed no source. The exact existing motion source
fix remains `src/amr_manipulation/src/gate6_mass_stage.cpp`; validation remains
green with 37 focused pytest checks, 274 package tests with 0 errors,
0 failures, and 5 skipped, a passing build, and passing `git diff --check`.
The worktree remains dirty with the prior source, test, launch, documentation,
and `.ros_logs/` changes; `AMR_CODEX_HANDOFF.md` remains untouched and its
protected Git blob hash is `469bd6ac1e0f1d85901c0f112ba8800cbfa67507`.

Higher-mass acceptance is not claimed. The blocker is a fresh explicitly
authorized direct-host run with a readable/writable approved render node;
until that is available, Product 102/103 and Gate 7 remain unvalidated.

## Current session handoff — higher-mass harness repair — 2026-08-31

The first launch of
`/tmp/amr_gate6_higher_mass_validation_20260831_02.sh` was invalid harness
evidence and did not reach Gate 6. With `set -u` active while sourcing
`/opt/ros/humble/setup.bash`, unset `AMENT_TRACE_SETUP_FILES` terminated the
shell before the intended failure ledger ran. During the resulting cleanup,
five `rg ... -E` calls were also invalid because ripgrep interprets `-E` as
`--encoding`; the cleanup filter had no search pattern and its error was
misclassified as a no-match. The incomplete overlay also made the cleanup
preflight report `Package 'amr_factory' not found`.

Preserved evidence is
`.ros_logs/gate6_3kg_5kg_20260831_02/`. It records both higher-mass attempt
counters as `0`, all runner/recorder/MoveIt/factory groups as not started, and
no host/hash/readiness/recorder/bag/analyzer/product evidence. This is not a
Product 102/103 failure and does not consume a product attempt.

Harness-only repairs are complete. `_02.sh` now disables nounset only while
sourcing ROS/workspace setup, preserves the primary setup failure, skips ROS
cleanup commands when the environment is incomplete, uses explicit `rg -e`,
and fails closed on scan errors. Because `_02` evidence already exists and
run identities cannot be reused, the fresh candidate is
`/tmp/amr_gate6_higher_mass_validation_20260831_03.sh` with
`RUN_ID=gate6_3kg_5kg_20260831_03`, `ROS_DOMAIN_ID=233`, and a new evidence
directory. No source, configuration, acceptance threshold, runtime evidence,
or `AMR_CODEX_HANDOFF.md` was changed; the protected handoff blob remains
`469bd6ac1e0f1d85901c0f112ba8800cbfa67507`.

Fresh validation after the repair passed:

- clean-environment ROS setup sourcing;
- 37 focused pytest checks;
- three-package Harmonic build;
- `colcon test` and `colcon test-result --verbose`: 274 tests, 0 errors,
  0 failures, 5 skipped;
- `git diff --check` and `bash -n` for both corrected harnesses.

Current worktree remains intentionally dirty; no runtime processes are left.
The next session must begin with `git status --short`, preserve this handoff,
and launch exactly once from the working direct-host shell:

```bash
cd /home/pete/amr_ws
/tmp/amr_gate6_higher_mass_validation_20260831_03.sh
```

The host must first prove `render`/`video` membership, a readable/writable
`/dev/dri/renderD*`, no forced software renderer, and no stale runtime
processes. Codex may monitor `_03` evidence read-only. Stop at the first
failed gate; do not retry, rerun Product 101, start Gate 7, or update the Gate
6 verdict/documentation until a valid Product 102/103 runtime result exists.

## Current session handoff — Mission Supervisor startup barrier validation — 2026-08-31

The authorized objective was to address the Mission Supervisor lifecycle/DDS
startup response race without starting Product 102/103 or creating the `_05`
higher-mass harness. The original `_04` evidence remains authoritative:
`/amr/mission_supervisor_node/change_state` timed out while the Mission
Supervisor remained `inactive [2]`; no higher-mass product attempt was
consumed.

### Implemented scope

Only `src/amr_mission/launch/amr_mission.launch.py` was changed for the
runtime fix. The initial `TRANSITION_CONFIGURE` event is now behind one
one-shot `TimerAction(period=1.0)` wall-clock barrier, matching the existing
controller startup-barrier design. Launch-only markers were added for
`MISSION_SUPERVISOR_CONFIGURE_TRANSITION_START` and
`MISSION_SUPERVISOR_ACTIVATE_TRANSITION_START` so a future readiness run can
capture the transition timeline. Mission Supervisor C++, lifecycle criteria,
product criteria, tolerances, controller behavior, and unrelated launch
behavior were not changed.

A fresh readiness-only validator was created at
`/tmp/amr_mission_readiness_validation_20260831_01.sh` with identity
`mission_supervisor_readiness_20260831_01`, `ROS_DOMAIN_ID=229`, one factory/
Gazebo-only startup attempt, no MoveIt, recorder, bag, analyzer, or products,
a 90-second startup bound, and a 45-second cleanup watchdog. It was executed
exactly once. No `_05` harness was created or executed.

### Verification and runtime result

Static checks passed: `bash -n`, launch AST/static contract, and
`git diff --check`. `amr_mission` built successfully. Its three package tests
passed; the existing aggregate result was 274 tests, 0 errors, 0 failures,
and 5 skipped.

The readiness validation result is **HARNESS BLOCKED**. It stopped before the
transition observer or factory launch at host preflight because the execution
environment exposed no readable/writable render device. Evidence:

- `.ros_logs/mission_supervisor_readiness_20260831_01/evidence/host_preflight/host_preflight.txt`
  records `render_devices=<none>` and
  `error=no readable/writable /dev/dri/renderD* device`;
- `evidence/failure_reason.txt` records `host preflight failed or timed out`;
- `evidence/readiness_verdict.txt` records `HARNESS BLOCKED`;
- `evidence/product102_attempts.txt` and `product103_attempts.txt` both record
  `attempts=0`;
- `evidence/cleanup_result.txt` records zero cleanup failures; and
- `evidence/shutdown_process_scan.txt` records
  `no_matching_runtime_processes`.

No Mission Supervisor process spawn, configure transition, activation
transition, factory, MoveIt, recorder, bag, analyzer, or product evidence was
created by this validation.

### Unresolved risks and next boundary

The validator's `domain_validation.txt` correctly records selected domain 229,
valid range `0..232`, no process conflict, and an empty direct graph probe.
However, `run_environment.txt` records inherited `ROS_DOMAIN_ID=1` because it
was written before the validator exported the selected domain. This is a
validator bookkeeping defect discovered by the single blocked run and was not
fixed or rerun under the current authorization.

The 1-second Mission Supervisor barrier has not yet been runtime-validated.
The next activity requires separate authorization, a direct host exposing a
readable/writable `/dev/dri/renderD*`, correction of the validator's domain
evidence ordering, and one fresh readiness-only identity. Do not retry this
run, tune the barrier, start Product 102/103, create `_05`, or modify
`AMR_CODEX_HANDOFF.md` without explicit authorization.

The worktree remains intentionally dirty from pre-existing project changes,
the intentional Mission Supervisor launch edit, and untracked runtime
evidence. No staging, commit, push, reset, checkout, or history rewrite was
performed. `AMR_CODEX_HANDOFF.md` remains untouched.

## Current authoritative handoff — Mission Supervisor barrier PASS — 2026-08-31

The Mission Supervisor startup boundary is now runtime-validated on the direct
host. No Product 102/103 stage, MoveIt process, recorder, bag, or analyzer was
started; both higher-mass attempt counters remain `0`.

The retained `_03` higher-mass run was invalid harness evidence: it selected
`ROS_DOMAIN_ID=233`, outside Fast DDS's recorded valid `0..232` range. At
`1788165852.316` the first ROS processes reported `Calculated port number is
too high`; the later mission-startup wait was only a downstream symptom. The
retained `_04` run used valid domain 230 and passed host, runtime, and graph
readiness, but the pre-fix Mission Supervisor process started at
`1788166909.8267248`, logged a `change_state` response timeout at
`1788166910.219790760` only `0.393066 s` later, and remained `inactive [2]`.
These are respectively a harness configuration bug and a lifecycle/startup
race, not evidence of a Mission Supervisor product-logic failure.

Fresh readiness-only run
`.ros_logs/mission_supervisor_readiness_20260831_02/` used valid domain 229,
strict hardware rendering, the strict true-attachment factory graph, and one
persistent lifecycle observer. Host preflight passed with
`/dev/dri/renderD128`, required `video`/`render` membership, no forced software
renderer, and no stale runtime processes. Source and installed Mission
Supervisor launch hashes matched at
`9d9361a35707e9243ac597bd7dec9e94e1ba9c32dc0ef121fb5637a0cbeb2c52`.

The new runtime timeline proves the one-second barrier:

- process start: `1788171944.7317126`;
- configure marker: `1788171945.7341447` (`+1.002432 s`);
- activate marker: `1788171945.7820826` (`+0.047938 s` after configure);
- persistent observer: `unconfigured [1] -> inactive [2] -> active [3]`;
- active stability: `2.179422 s`; and
- no Mission Supervisor `change_state` response timeout.

The validator's prior domain-bookkeeping defect is corrected in the retained
run copy: `run_environment.txt` records domain 229 after export, and
`domain_validation.txt` records valid range `0..232`, an empty pre-launch
graph, and `verdict=PASS`. Cleanup used SIGINT, recorded zero cleanup failures,
and the final process scan reports `no_matching_runtime_processes`.

Current status: Gates 1-5, Phase J, Phase K, and two independent 1 kg Gate 6
passes remain accepted. The Mission Supervisor readiness blocker is closed.
Gate 6 higher-mass acceptance is still unclaimed because Products 102 and 103
have not run. Do not rerun Product 101 or advance to Gate 7. The next action is
a separate evidence review and explicit authorization for one fresh
higher-mass harness/run using a valid domain and the now-validated Mission
Supervisor launch; stop at its first failed gate. `AMR_CODEX_HANDOFF.md`
remains untouched.

## Current authoritative handoff — Product 102 preparation FAIL — 2026-08-31

The authorized higher-mass boundary was exercised without rerunning Product
101 or starting Gate 7. Run `_05` used valid domain 227 and passed direct-host,
hardware-rendering, runtime-RTF, Mission Supervisor, graph, lifecycle,
controller, and action readiness, but its readiness command omitted
`--include-hidden-topics` while asserting hidden action-status topics. It
failed 0.066 s after writing the filtered topic list. This is a deterministic
test/harness bug; Product 102/103 attempts remained zero and cleanup passed.

Fresh run `.ros_logs/gate6_3kg_5kg_20260831_06/` changed only the run identity,
valid domain to 226, and that one CLI flag. Domain 226 was empty; host and
runtime preflight passed with `/dev/dri/renderD128` and aggregate RTF
`0.9997039311`; Mission Supervisor markers and complete integrated readiness
passed. Product 102 then consumed its single attempt. Its recorder was ready
before the preparation runner; Product 103 remained unstarted.

The first causal Product 102 failure is a product/source configuration defect,
not DDS, lifecycle, rendering, command ownership, or evidence analysis. The
precise dock controller started at `1788173309.127366377`; Nav2 logged
`Failed to make progress` at `1788173332.677563518`, the Mission Supervisor
aborted `0.000423 s` later, and the runner failed `0.002917 s` after the
controller error. Bag replay of `/amr/localization/odometry` predicts the same
configured `PoseProgressChecker` failure at `1788173332.405500412`, only
`0.272063 s` before the actual line: over `10.033143 s` the precise leg moved
`0.060525 m` and rotated `0.021949 rad`, below the configured `0.20 m` /
`0.20 rad` progress thresholds. `PlacementFollowPath` had decelerated to its
configured `0.01 m/s` minimum with zero angular command while terminal yaw
remained about `-0.256 rad`, outside the `0.15 rad` goal tolerance.

All command layers remained continuously nonzero and consistent until abort;
the physical path advanced `0.906384 m`, final physical target error was
`0.033627 m`, and product B did not move. The harness's final
`product102 mass-stage log is missing` text is a downstream reporting symptom;
the mass stage never started because preparation had already failed. Cleanup,
the final process scan, and post-shutdown host preflight passed.

Authoritative diagnostics are
`evidence/post_run_root_cause.txt`,
`evidence/post_run_product102_bag_timeline.txt`, and the retained reader beside
them. Product 102 attempts are now `1`; Product 103 attempts remain `0`.
Gate 6 higher-mass acceptance is **FAIL / unresolved**. No product source or
acceptance criterion was changed. The next justified activity is a separate
source-design review of the precise-docking route/controller/progress-checker
contract that addresses both slow-progress rejection and terminal yaw without
weakening final dock tolerances or collision safety. Do not merely increase a
timeout, retry Product 102, start Product 103, or start Gate 7 without fresh
authority. `AMR_CODEX_HANDOFF.md` remains untouched.

## Current authoritative handoff — Product 102 dock-abort recovery correction — 2026-08-31

The source-design boundary is corrected but not yet runtime validated. The
retained `_06` terminal localized pose `(2.3994, -0.0047, -0.2643)` was inside
the existing 0.03 m dock-position window when Nav2 aborted for lack of
progress. `gate6_product_test.py` now preserves fresh terminal localization in
a typed navigation failure and catches it only at the first precise-dock leg.
That failure is never accepted as success: only a pose inside the unchanged
0.03 m position window may proceed, and it must still pass the existing AMCL
generation, stationary-base, fresh physical-pose, and bounded localization
bias checks before the registered retreat, approach alignment, relocalization,
egress, and precise re-dock sequence runs. Stale, non-finite, distant, or
unbounded evidence remains fail-closed. The final 0.03 m / 0.15 rad dock
tolerances, controllers, progress checker, collision-safe route, ownership,
attachment semantics, and analyzer criteria are unchanged.

Validation passed: focused contract tests `8 passed`; Python compilation and
scoped `git diff --check` passed; `colcon build --packages-select
amr_manipulation --symlink-install` passed; and the package test result is 34
tests, 0 errors, 0 failures, 0 skipped. No runtime process was started and no
Product 102 retry was consumed. Sol/high replans used: 0 of 2.

Gate 6 higher-mass acceptance remains **FAIL / unresolved** pending fresh
runtime evidence. The next justified action is an explicit review and
authorization for one direct-host Product 102 retry using the corrected `_06`
procedure with a new valid domain/run identity, clean-host preflight, and a
stop at the first failed gate. Product 103 and Gate 7 remain blocked.
`AMR_CODEX_HANDOFF.md` remains untouched.

## Current authoritative handoff — pre-runtime safety and test-isolation closure — 2026-08-31

A pre-runtime audit found and corrected one safety defect in the unvalidated
dock-abort patch: its typed exception originally covered every non-success
action result. It is now raised only for `GoalStatus.STATUS_ABORTED` with fresh
terminal localization. Canceled, unknown, rejected, timed-out, stale, and
distant results remain ordinary fail-closed preparation failures. The single
recovery catch remains limited to the first precise-dock leg, and no motion,
geometry, controller, lifecycle, ownership, attachment, or analyzer criterion
changed.

The first full parallel test exposed a separate test-harness defect in
`amr_control`: its ROS-live behavior test shared the default DDS domain with
concurrent packages. It aborted its first egress at 0.264 s with
`BASE_READY_EVIDENCE_STALE_OR_INVALID` and captured three foreign positive
`0.25 m/s` `/amr/control/cmd_vel` samples. The concurrent base-adapter test
publishes that exact value on the same topic. The failing egress test passed
alone on domain 225, proving the control behavior itself was not defective.
CTest now runs `control_configuration_test` on dedicated valid domain 211,
with a contract test preserving that isolation; production control code and
timeouts are unchanged.

Fresh validation: Product runner contract 8/8 passed; `amr_control` contract
6/6 and package results 13 tests with zero failures passed; all 18 packages
built; and the post-fix full parallel workspace result is 276 tests, 0 errors,
0 failures, 5 skipped. Scoped diff checks passed. Sol/high used the allowed two
replans; no third plan was needed. No Gazebo, MoveIt, recorder, product stage,
or integrated runtime was started.

The source/static boundary is ready for evidence review, but Gate 6
higher-mass acceptance remains **FAIL / unresolved** until runtime proves the
new recovery path. The next action still requires explicit authorization for
one clean-host, direct-host Product 102 retry with a new valid domain and run
identity, stopping at the first failed gate. Product 103 and Gate 7 remain
blocked. `AMR_CODEX_HANDOFF.md` remains untouched.

## Current authoritative handoff — Product 102 retry stopped at DDS readiness — 2026-08-31

The authorized one-Product-102 direct-host retry used
`.ros_logs/gate6_product102_retry_20260831_07/`, valid empty domain 225, and a
bounded `_06`-derived harness that structurally prohibited Product 103. Source
and installed hashes matched. `/dev/dri/renderD128`, host preflight, hardware
runtime preflight, aggregate RTF `0.999999829119212`, Mission Supervisor
markers, MoveIt startup, and persistent complete/stable graph readiness all
passed. Product 102 and Product 103 attempt counters both remain zero; no
recorder, product runner, mass stage, or analyzer started.

The first failed gate was `ros2 lifecycle get /amr/base_adapter_node`, exit
124. The factory-localization lifecycle manager had reported all managed nodes
active at `1788176915.789597060`; the persistent observer later included the
base adapter with no missing/duplicate required node, and the immediately
preceding AMCL query returned `active [3]`. The base-adapter query began at
approximately `1788176949.110266840` from evidence-file mtime. At
`1788176951.431460259`, 2.321 s later, base adapter logged that sending its
`get_state` response timed out and the client would not receive it. The CLI
reached its 15-second bound at approximately `1788176964.274022366`.

This is a DDS/RMW service-response readiness failure exposed by a one-shot
harness query. It is not evidence of an inactive lifecycle node, Product 102
logic, the dock-abort correction, Gazebo, rendering, command ownership, or the
analyzer. Cleanup began at `1788176965.905113947`; cleanup failure count was
zero, shutdown process and post-shutdown host gates passed, and no runtime
process remains. Shutdown-only wheel-odometry `-11` and bootstrap exit `1`
lines occurred after SIGINT and were not causal.

Authoritative diagnosis is `evidence/post_run_root_cause.txt`. Gate 6
higher-mass acceptance remains **FAIL / unresolved**. The user-mandated replan
ceiling is exhausted at 2/2, so stop: do not retry, tune, modify source, start
Product 103, or start Gate 7 without new user direction.
`AMR_CODEX_HANDOFF.md` remains untouched.
