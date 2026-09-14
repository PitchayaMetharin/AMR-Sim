# Phase 15 — Factory SLAM commissioning

## Authority and status

As of 2026-09-12, Phase 15 packets P0 through P8 are implemented and
independently accepted at the source/offline boundary. This is a bounded
follow-on to the simulation-only online SLAM work in Phase 9; it does not claim
autonomous mapping runtime acceptance, hardware or functional-safety
acceptance, human map-quality acceptance, promotion, or canonical-map
replacement. The current Phase 14 Product 101/102 runtime closeout remains a
separate boundary.

P0's isolated Humble evidence established that the installed Python
subscription API exposes per-message source and receipt timestamps but no
publisher GID. Individual TF-edge ownership therefore remains fail-closed;
aggregate `/tf` publisher lists are not a substitute.

## Current checkpoint — 2026-09-12 — P0-P8 source/offline closeout

The accepted source/offline behavior is:

- P1 retains mission cancellation ownership through downstream terminal-result
  proof, carries pending acceptance through cancel races, makes repeated
  cancellation idempotent, and blocks public results and the next mission
  until the obligation is resolved.
- P2 revalidates map/costmap identity and freshness, motion authority, and the
  carried TF sample immediately before reservation; stale or changed evidence
  produces zero-send discard behavior.
- P3 applies minimum-distance eligibility before representative/fallback
  selection, resets exhaustion only after the final reservation gate, and
  reports persistent raw frontiers with no admissible endpoint as `INCOMPLETE`.
  True raw-frontier exhaustion remains `COMPLETE`; skipped clusters consume no
  motion token, blacklist entry, or navigation-failure count.
- P4 captures failed endpoints in map-frame world coordinates at reservation,
  retains them through cleanup, and reprojects them through later map geometry.
  Exclusion is exact-cell only, and only an accepted explicit start clears the
  run-scoped failures.
- P5 sends a bare candidate prefix, requires `<prefix>.posegraph` plus
  `<prefix>.data`, and uses manifest schema 3 to record and hash both files.
  Legacy, incomplete, aliased, missing, or tampered bundles fail closed, and
  verified discard is limited to manifest-enumerated candidate files.
- P6 requires a terminal autonomous state, fresh status, explicit false
  `active`, `pending`, and `fault_latched`, a positive run generation, and an
  empty `cancel_target`. `INCOMPLETE` remains distinct and requires explicit
  acknowledgement and explanation; persisted reports are strictly revalidated.
- P7 requires quality-review schema 2 and an exact
  `artifact_bundle_sha256` matching the verified manifest at both `accept` and
  `promote`; changed or unbound evidence fails closed.
- P8 validates finite explorer parameters, map geometry, and integral
  occupancy values before readiness, extraction, or completion accounting.
- Explorer TF readiness queries `map -> base_footprint` at the node's exact
  current ROS time, rejects zero, future, stale, nonfinite, or invalid-
  quaternion samples, and carries the validated sample into reservation. In
  autonomous mapping, frontier clustering runs before the production lookup,
  so an expensive cluster pass cannot age out the sample that is later used
  for the robot pose; the reservation gate revalidates that carried sample
  and sends no goal when it has expired.
- No-motion readiness uses a continuous episode budget. Autostart and an
  accepted start begin the initial 15-second episode; full readiness clears
  it. A later `WAITING_READY`/`SCANNING` miss starts a fresh 15-second episode
  and remains motionless with no send while it is within grace. Recovery clears
  the episode; a continuously invalid episode faults closed after grace.
  `GOAL_PENDING` and `NAVIGATING` readiness loss still cancel immediately and
  do not use this grace episode.

Fresh parent validation passed 650 tests across
`src/amr_exploration/test`, `src/amr_factory/test`, and
`src/amr_manipulation/test`; `git diff --check` also passed. The two focused
post-diagnostic source packets were independently reviewed by Sol/high:

- The factory CLI packet sets the default transport/enqueue timeout to 240 s,
  passed 24 focused tests, built `amr_factory`, and received review `PASS`.
- The cycle-adapter packet catches `KeyboardInterrupt` and
  `ExternalShutdownException`, performs executor/node cleanup, calls
  `rclpy.try_shutdown()`, passed 47 focused tests, built `amr_manipulation`,
  passed an isolated Humble probe with exit 0, and received review `PASS`.

No runtime was run after either fix. The source/offline boundary is therefore
complete, while the runtime and human acceptance boundaries below remain open.

## Historical checkpoint — 2026-09-11 — P1/P2/P3/P4/P5/P6 repair (superseded)

The P1 cancellation-ownership, P2 dispatch-evidence, and P3 frontier-selection
packets are implemented and verified at the source/offline level in their
approved files. P1 retains accepted downstream action handles until terminal
result proof, carries pending acceptance through cancellation races, makes
repeated cancellation idempotent, and makes deactivation monotonically
strengthen a prior cancellation to `ABORTED`. Public mission completion remains
blocked until downstream terminal proof.

Fresh P1 verification recorded a passing `amr_mission` build, 4 focused P1
cancellation probes passing across 5 repetitions, the full 12-case mission
behavior suite passing, and all 3 registered mission suites passing.
`colcon test-result` then reported 495 tests with 0 errors, 0 failures, and 5
skips; `git diff --check` passed. The independent Sol/high review returned
PASS with no blocker, high-, or medium-severity finding; its one low test
publication-order finding was corrected and revalidated. The P2/P3 exploration evidence remains 75
focused tests passed, Python compilation passed, the `amr_exploration` build
passed, and all 3 registered exploration suites passed. `ament_flake8` remains
at the known baseline of seven E501 findings and the existing test E402; the
P3-isolated lifecycle E127 was corrected. P0 isolated Humble probing found per-message
source/receipt timestamps but no publisher GID in the installed Python
subscription API; live TF edge ownership therefore remains fail-closed, with
no aggregate `/tf` publisher-list bypass. No runtime was run after these
packets.

P3 applies minimum-distance eligibility before representative/fallback choice,
resets exhaustion only after the final dispatch reservation gate, and reports
raw frontiers with no safe costmap-valid endpoint as terminal `INCOMPLETE`
rather than `COMPLETE`. It does not prove collision-free paths, reachable
goals, or autonomous completion. This historical checkpoint still pointed to
the ordered remediation packets; the current 2026-09-12 checkpoint records
their completion and independent review. Runtime authorization, map-quality
review, and promotion remain separate decisions.

P4 is implemented and accepted at the source/offline boundary. Failed
navigation endpoints are captured as map-frame world coordinates at
reservation and reprojected through each later map's origin, resolution, and
planar yaw; out-of-bounds failures remain retained until an accepted explicit
start. Exact-cell exclusion has no radius, and a blacklisted representative
does not discard safe siblings from its cluster. Fresh P4 evidence is 82
focused algorithm/lifecycle tests passed, 84 tests across
`src/amr_exploration/test`, successful Python compilation, a successful
`amr_exploration` build, and 3/3 registered exploration suites passed. The
package result summary is 87 tests with 0 errors, 0 failures, and 0 skipped;
`git diff --check` is clean. The independent Sol/high P4 review returned PASS
with no blocker, high-, or medium-severity finding.

P6 tightens autonomous acceptance at the source/offline boundary. Only the
terminal outcomes `STOPPED`, `COMPLETE`, and `INCOMPLETE` are eligible, with
fresh status, `active: false`, `pending: false`, `fault_latched: false`, a
positive run generation, and an empty `cancel_target` required. `FAULT`, all
transitional, unknown, missing, stale, and malformed states fail closed. The
autonomous runtime report preserves a top-level `exploration_outcome`, which
acceptance cross-checks against the observed exploration state. `INCOMPLETE`
remains distinct and requires explicit operator acknowledgement and a
nonblank explanation; accepted reviews also bind to the SHA-256 of the actual
runtime report. The final repair makes `accept` and `promote` strictly validate
the report envelope and independently rederive every acceptance gate from
normalized persisted observations, including map geometry/data, node graphs,
TF receipts/ownership, authority, preflight, and manual/autonomous MPC
ownership. Freshness during revalidation uses the report's
`checked_monotonic` observation instant and the existing thresholds. The
focused factory mapping acceptance pytest passes 108 tests. At this historical
checkpoint, the source/test result was still awaiting parent validation and
independent review; that status is superseded by the current P0-P8
source/offline acceptance. P7 bundle binding was tracked separately and is
accepted in the current closeout.

## Historical source/offline completion record — 2026-09-09 (pre-final P6 repair; superseded)

Fresh verification for the earlier Phase 15 source/offline boundary; this
record predates the final P6 semantic-revalidation repair and is not an
independent P6 acceptance:

- focused Packet C pytest: 52 passed;
- `amr_factory` colcon tests: 9/9 passed;
- `colcon test-result`: 457 tests, 0 errors, 0 failures, 5 skipped;
- Python compilation, `ament_flake8`, the `amr_factory` build, and
  `git diff --check`: passed.

No Phase 15 runtime/hardware execution, human production acceptance, map-quality
acceptance, promotion, or canonical-map replacement was performed. Live TF
ownership remains fail-closed until the edge publisher identity is honestly
observable; separate runtime authorization is required.

## Objective

Provide a factory-world commissioning entry point that uses the existing robot,
sensor adapters, local EKF, rendering controls, and attachment option while
keeping production localization unchanged:

- `factory_localization.launch.py` remains the production static-map + AMCL
  entry point, with its existing defaults and `nav2_amcl` ownership of
  `map -> odom`.
- `factory_mapping.launch.py` is the separate online-mapping entry point.
  SLAM Toolbox is the sole `map -> odom` authority; it does not launch
  `nav2_map_server` or AMCL.
- The local EKF remains the sole `odom -> base_footprint` authority.
- Command arbitration remains the sole `/amr/control/cmd_vel` publisher.

### Mapping-specific readiness

Online mapping has a deliberately different graph from the generic factory
product graph: `nav2_map_server`, AMCL, and `move_group` are absent by design.
Do not use the generic graph preflight for mapping; its required-node contract
would fail on those intentional omissions. Use the mapping-specific runtime
preflight profile and the observation-only `factory_mapping_acceptance.py`
checks instead. Those checks require the SLAM Toolbox `map -> odom` owner, the
EKF `odom -> base_footprint` path, fresh map and exact-current-time Explorer TF
readiness, mapping-mode authority, and the mode-appropriate Nav2/exploration
chain. Human map-quality review and promotion remain separate, out of scope
decisions for this source/offline packet.

## Slice 1 — factory mapping entry point

1. Add the separate factory mapping launch. Forward the existing `headless`,
   rendering, attachment, and initial-pose arguments to the shared factory
   launch, and run SLAM Toolbox with the established front adapted LaserScan
   and mapper configuration.
2. Keep manual and autonomous mapping as mutually exclusive control modes.
   Manual mode omits the Nav2 controller and mission launch, so the existing
   `amr_control/prototype_teleop.py` is the only expected
  `/amr/mpc/cmd_vel` compatibility publisher. Autonomous mode starts the
  existing Nav2 mission chain and the frontier explorer.
3. Add an explicit `map_yaml` argument to the production factory localization
   launch, defaulting to the canonical `maps/factory.yaml`, and pass it to
   `map_server`. AMCL behavior and production defaults remain unchanged.
4. Declare direct runtime dependencies and add focused source contracts for
   mode exclusivity, forwarded arguments, defaults, and mapping/factory TF
   ownership.
5. Document the manual mapping and candidate-map workflow without claiming
   runtime evidence.

## Slice 2 — autonomous exploration and map artifacts

- `amr_exploration/frontier_explorer.py` selects free/unknown frontiers from a
  live map and sends one goal at a time through
  `/amr/mission/navigate_to_pose`. It never publishes velocity directly.
- The production planning path checks non-TF readiness, enters planning, runs
  the expensive frontier clustering pass, then obtains one exact-current-time
  `map -> base_footprint` sample and carries it through candidate selection,
  action-server waiting, and the final zero-send reservation gate.
- Autonomous mapping launches the existing Nav2 planner/controller/mission
  chain; manual mapping omits that chain so teleoperation remains exclusive.
- Stale map/TF, missing motion authority, unavailable actions, repeated goal
  failures, and unconfirmed cancellation fault closed. The stop service is
  `/amr/exploration/stop`.
- `factory_mapping_cli.py save` stores the occupancy map, the serialized
  pose-graph bundle, the surveyed-datum manifest, and the transformed
  candidate YAML under the explicit session directory. P5 is implemented at
  the source/offline boundary: the serializer receives the bare prefix and
  the manifest schema 3 bundle requires exact, regular, non-empty
  `<prefix>.posegraph` and `<prefix>.data` artifacts, with per-file
  provenance and a bundle hash. Legacy single-file or incomplete manifests,
  reserved output aliases, and tampered or missing artifacts fail closed;
  verified discard removes only the manifest-enumerated candidate files and
preserves unrelated evidence. Fresh P5 evidence is 49 mapping CLI tests
passed, 72 mapping CLI/acceptance tests passed, and 155 tests across the
factory test directory passed; all four P5 Python files compiled
successfully, the `amr_factory` build passed, and all 9 registered factory
suites passed. The package result summary is 162 tests with 0 errors, 0
failures, and 0 skipped; code/document diff checks are clean. The final
independent Sol/high P5 review returned `PASS`. No Phase 15 runtime, hardware,
or canonical-map acceptance is claimed.

## Packet C — offline acceptance and promotion eligibility

The mapping artifact lifecycle is fail-closed:

```text
SAVED -> VALIDATED
                  + RUNTIME_PASS
                  + QUALITY_REVIEW_ACCEPTED
                  -> PROMOTION_ELIGIBLE
```

`factory_mapping_acceptance.py runtime` is observation-only. It subscribes to
map, TF, manipulator, and exploration status evidence and inspects the graph;
it does not launch processes, start exploration, publish velocity, or command
motion. It uses a bounded 60-second steady-clock window and the established
map (3 s), TF (1 s), and authority-status (1 s) freshness windows. It writes
the run-specific `runtime_acceptance.yaml` atomically, including all passing
and failing observations, normalized map frame/geometry/data, map dimensions,
and known/free/occupied/unknown counts. No quantitative map-quality threshold
is invented.

The observer requires SLAM Toolbox as the sole `map -> odom` owner, no AMCL or
map server, fresh valid `/map`, fresh `map -> odom` and `odom ->
base_footprint`, the existing EKF, command arbitration, sensor adapters, and
manipulation authority, plus PASS host and runtime-preflight reports. Manual
mode excludes planner, smoother, controller, mission, and frontier explorer;
autonomous mode requires that chain, excludes prototype teleoperation, and
requires the P6 terminal exploration proof: `STOPPED`, `COMPLETE`, or
`INCOMPLETE`, with explicit false `active`, `pending`, and `fault_latched`, a
positive `run_generation`, an exact empty `cancel_target`, and fresh status.
The report exposes and validates the corresponding top-level
`exploration_outcome`; transitional `WAITING_READY`, `SCANNING`,
`GOAL_PENDING`, `NAVIGATING`, and `CANCELLING` snapshots cannot be considered
complete.

`accept` and `promote` treat the persisted runtime report as untrusted
evidence. They require the exact passing envelope, then rederive every gate
from its normalized observations: mode and graph membership, forbidden
localization, map geometry/data/counts and freshness, both TF receipts and
edge owners, map/control owners, manipulator authority, preflight reports,
and mode-specific graph/MPC ownership. Revalidation uses the report's
`checked_monotonic` as the observation instant rather than the current clock.
The runtime hash and all candidate, manifest, and conditional review proofs
remain bound before either receipt is written.

Before `accept`, the target review contract is an explicit run-specific
`quality_review.yaml` with review schema 2. It records the reviewer,
`decision: ACCEPTED`, the exact candidate path, the candidate SHA-256, the
exact `artifact_bundle_sha256` from the verified manifest, and the matching
`runtime_acceptance_sha256` for the actual runtime report. Schema-1 or otherwise
unbound reviews are rejected. Discard/resave or any change to the artifact
bundle requires a newly bundle-bound review. If exploration stops with
remaining frontiers but no safe reachable goal, the required terminal outcome
is `INCOMPLETE`/safely stopped, with exact
`incomplete_acknowledgement: ACCEPT_INCOMPLETE` and a nonblank
`incomplete_explanation` before acceptance; it must not be treated as
`COMPLETE`.

`promote` re-verifies all three proofs and writes only
`promotion_eligibility.yaml`, pointing to the exact run-specific candidate.
It never copies, replaces, renames over, or modifies
`src/amr_factory/maps/factory.*` or an installed canonical map. Replacing the
canonical map remains a separately authorized operation.

## Diagnostic runtime record — `phase15_commission_20260912_01`

This pre-fix diagnostic run is evidence for diagnosis only, not Phase 15
mapping acceptance or post-fix runtime acceptance. Host, RTF, graph, lifecycle,
and MoveIt preflights passed. Product 101's downstream cycle succeeded in
approximately 134.86 s, but the old 120 s CLI deadline returned exit 2 before
the outer transport result was delivered. Product 102 was not attempted. The
compact recorder omitted the outer transport action-status topic, and the
processes were stopped after the run.

## Deferred runtime evidence

For a Phase 15 simulation mapping run, use the named runtime-preflight profile
below before collecting acceptance evidence:

```bash
ros2 run amr_factory factory_runtime_preflight.py runtime \
  --profile phase15_mapping \
  --evidence-dir "$mapping_session/evidence"
```

The `phase15_mapping` profile requires median RTF `>= 0.80` and aggregate
simulated-time/real-time `>= 0.80`. Hardware rendering, Gazebo/device
presence, sample-count, positive-span, stats-exit, graph, lifecycle, TF,
authority, collision, exploration, and promotion gates are unchanged. The
default/shared runtime profile remains median and aggregate RTF `>= 0.90`.

No fresh runtime has been run after the factory CLI timeout and cycle-adapter
shutdown fixes. Fresh explicit authorization is required before launching
Gazebo/MoveIt or running autonomous product cycles. Post-fix direct terminal
proof for Product 101 and Product 102, autonomous mapping runtime acceptance,
per-edge TF publisher ownership, human map-quality acceptance, a passing
runtime report, promotion eligibility, canonical-map replacement, and hardware
or functional-safety acceptance remain unverified. Product 103 and Gate 7
remain excluded. These boundaries must not replace the production AMCL path or
overwrite the canonical map automatically.

The mapping launch also preserves the factory control interlock
`require_manipulator_stowed=true`. Accepted Phase 14 Product 101/102 runtime
evidence included stowed proof but does not itself prove a new Phase 15 mapping
run. The interlock therefore remains fail-closed unless the current mapping run
provides valid stowed authority; this source implementation does not invent or
bypass it.

## Acceptance boundaries

The implemented source is complete only when focused tests, launch-file
compilation, and package builds pass; production still selects the canonical
map and AMCL by default; mapping has one explicit SLAM Toolbox map-to-odom
owner; autonomous exploration has no direct velocity publisher; and the
offline acceptance gates preserve artifact provenance. Direct-host and
hardware runtime acceptance remain unverified. Any candidate map must be
written to a run-specific path; the canonical factory map is not an output
target.
