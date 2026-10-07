# Native45 captured-scene clearance implementation packet

Status: SUSPENDED after first adapter self-test failure and Sonnet stop-rule
deviation. Partial adapter remains preserved. No writer may resume this packet
until Sol/high diagnosis and a new bounded correction release; check execution
will be owned by orchestrator after writer HOLD. See SESSION_HANDOFF for current state.

## Objective, diagnosis and authority

Sol/high established valid acquisition-stamped TF and padded footprint geometry
for Native44 clear-start and pre-dock scenes using retained JSON/JSONL and an
already-decoded typed stream. No further bag extraction is needed.
Current intended production planner/configuration is replay authority; historical
binary equivalence is UNKNOWN and must remain unclaimed.
Latest user "continue" is interpreted as choosing the previously recommended
complete-TF route; orchestrator stated this interpretation. Preserve missing
optional AMCL annotation and original extraction exit2/status.
No source gate, safety threshold or runtime acceptance is waived.

Implementer: exact claude-sonnet-5-5 / medium through installed Claude CLI.
Probe sonnet_probe_20261006T172557Z returned exit0, canonicalModel
claude-sonnet-5-5, requested --effort medium. No fallback model is permitted.
Sol/high analyzes/reviews; sole production/test-source writer is Sonnet.

## Allowed files and worktree

Only modify:
- src/amr_navigation/test/replay/CMakeLists.txt
- src/amr_navigation/test/replay/planner_replay_backend.cpp, reusable bounded support only
- new src/amr_navigation/test/replay/native45_scene_adapter.py
- new src/amr_navigation/test/replay/production_precision_replay.cpp

Evidence-only files may be created under the assigned exclusive evidence root E.
Do not edit SESSION_HANDOFF, AGENTS, protected AMR_CODEX_HANDOFF, ledgers, production
planner/config/mission/manipulation source, package manifests or unrelated files.
Worktree has numerous pre-existing changes: preserve all of them. Run git status
and reread exact targets before editing; keep before/after scoped diff.
Target baseline hashes:
CMakeLists.txt ab8ca5559c9f649fd7cc165c5daa14a4f5e9a365b2ba05e25ef56734aeb21ccb
planner_replay_backend.cpp 0a3c6410e93a97c11f4ed102e75f35257e44ef418d3a62b51229adbacff09d6b
planner.yaml source/install 8ec55573ce6fd9bcdb0bb03268244493435951746a956457008cff9b9b695577

## Required behavior

Use installed PrecisionNavfnPlanner instance configure/activate/createPlan through
pluginlib, current installed planner.yaml parameters, unchanged SimpleSmoother
parameters and 1-second production budget (mission_supervisor_node.cpp:653).
Use Costmap2DROS fixture seam from exact_goal_lattice_contract_test, replacing grid
with captured raw costs and footprint. Retain plugin/source/build/install identities.
Do not substitute helper-only paths or generic Navfn fallback success as clearance.
Record direct-precision vs fallback routing using the existing installed helper
where supported; always sweep the returned real plugin path.

Existing backend CLI retains its 2-second default. If reusable support needs a
budget parameter, preserve default=2 and explicitly pass 1 from the dedicated
production replay. Preserve existing accepted tests and raw cost loading.

New adapter CLI:
  --baseline-dir PATH --tf-extract-dir PATH --planner-config PATH
  --stations-config PATH --products-config PATH --output PATH --report PATH
and --self-test.

Read only retained JSON/JSONL and typed_stream.jsonl.gz, never bag files.
Produce native45_clearance_scene_v1 schema with hashes, stamped row identities,
extraction missing optional AMCL annotation, current config params/hashes,
unchanged global/local raw cost grids and origins, acquisition stamps and each
local map-to-odom transform, padded footprint and discrepancy checks, clear/pre-dock
physical/localized captures and measured biases, constructed-case endpoints,
and runtime properties explicitly not_exercised.
Reject invalid stamps, unbracketed transforms, nonfinite poses, invalid quaternions,
wrong frames, malformed grids/raw costs, changed hashes and footprint discrepancy.
Use 1e-6 m tolerance for polygon numerical consistency only.
Preserve 0..255 costs; reject unknown/lethal/outside-map collision coverage.

Derive registries/current centered stance from current shared helper/source.
Present registry values: approach (-2.5,0,pi), dock (-3.4,0,pi), centered slot
(-4.1,0,0.075), physical stance (-3.345,0.1,pi), clear point (-2.5,0.1).
No fixture shift or geometry correction is authorized. Adapter may implement the
centered formula from source-pinned constants, but the C++ executable must call
shared final_placement_stance() with supplied registry inputs and independently
verify agreement <=1e-9. For this source-only test use include directory
${CMAKE_CURRENT_SOURCE_DIR}/../../../amr_interfaces/include on the new target;
no package manifest/dependency change is authorized.

For each of 4 clear-start captures produce complete nominal sequence using its
measured bias. Repeat each using all 4 pre-dock observed bias samples as explicit
geometric sensitivity cases, not future post-heading freshness observations.
Follow gate6_mass_stage.cpp:1710 onward: when clear distance >0.01 choose nearest
forward/reverse tangent (forward on equality); tangent heading at localized XY;
translation to bias-corrected clear point; mandatory arrival heading at localized
clear XY; precision dock path to bias-corrected immutable stance. Include the dock
path's actual terminal heading sweep. Additionally cover the constructed subsequent
placement final-heading action at stance, targeting stance_yaw-bias_yaw-0.03 rad.
That source-backed margin is kFinalHeadingGoalMargin at gate6_mass_stage.cpp:3312;
command target/action are at 3646/3651. It occurs AFTER centered docking, not inside
that routine. Keep geometric construction explicit; no executed-action claim.

Include synthetic one-repair cases with physical misses 0.010001 m along ±X/±Y,
only when they satisfy unchanged 0.15 m original reference and registered admission
bounds. Use actual repair formulas and report excluded cases with admission reason.
No synthetic case is a captured observation. Start pose/case provenance is explicit.
Moving captures do not demonstrate observer settling; runtime acceptance remains
separate. Capture bias stays an explicit geometric scenario assumption.

For every stage evaluate planner and 1-second smoothed paths, and explicit heading
sweeps against all four captured grids (clear global/local, pre-dock global/local).
Transform paths into each grid's frame using its own valid stamped transform.
Whole polygon includes boundary plus interior cells. Dense motion interpolation
spacing <=0.5*resolution for translation plus corner travel. Reject empty paths,
invalid frame/pose, unknown/lethal/intersecting cells and outside-map coverage;
retain production center rejection and inscribed-cost semantics. No clipping.
Report first failing case/stage/map/pose/cell and stop at that mandatory failure.

## Falsifiable prediction and focused checks

Stamped reconstruction agrees with effective ±0.61/±0.41 m padded footprint within
1e-6 m. Every stage either produces explicit full-polygon clearance evidence or
fails at a named stage/pose/cell. Plugin load/build alone is not clearance.

New executable production_precision_replay; new CTest native45_clearance_contract
runs production_precision_replay --self-test. Self-tests exercise real strict seam:
empty path, lethal/unknown interior with clear center/perimeter, intermediate turn
collision with clear endpoints, outside-map polygon, clear positive, malformed
frame/grid/pose, padded shape and 1-second smoother selection. For empty/interior
cases demonstrate permissive old checker accepts known-broken baseline and new
strict checker rejects. Do not weaken old tests. Adapter --self-test covers schema,
TF interpolation/no extrapolation, malformed input and construction contracts.

## Exact validation sequence

E is assigned by orchestrator before release; all outputs/cache/tmp/logs within E.
Check workspace storage_budget.check(reserve=30000000) before/after major steps;
logical and allocated logs/evidence each must stay below 1,000,000,000 bytes.
No deletion or evidence disposal is authorized in this packet.

export AMR_WS=/home/pete/amr_ws
cd "$AMR_WS"
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=232 PYTHONDONTWRITEBYTECODE=1 CMAKE_BUILD_PARALLEL_LEVEL=2
export ROS_LOG_DIR="$E/ros_logs" TMPDIR="$E/tmp" XDG_CACHE_HOME="$E/cache"

git status --short
git diff --check -- src/amr_navigation/test/replay
python3 -B src/amr_navigation/test/replay/native45_scene_adapter.py --self-test
colcon --log-base "$E/build_logs" build --packages-select amr_navigation --executor sequential --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/amr_navigation --output-on-failure -R '^native45_clearance_contract$'

After source checks and INDEPENDENT COMPLETED-PACKET REVIEW, orchestrator may
run the following captured-scene runtime commands; implementer must HOLD and
NOT run these before review. Verify source/build/install plugin/config parity first:
python3 -B src/amr_navigation/test/replay/native45_scene_adapter.py \
 --baseline-dir "$AMR_WS/phase14_evidence/stability_20261003/b_turn_placement_20261006/native44_baseline" \
 --tf-extract-dir "$AMR_WS/phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/extract_tf_20261006T170734Z" \
 --planner-config "$AMR_WS/install/amr_navigation/share/amr_navigation/config/planner.yaml" \
 --stations-config "$AMR_WS/src/amr_factory/config/stations.yaml" \
 --products-config "$AMR_WS/src/amr_factory/config/products.yaml" \
 --output "$E/scene.json" --report "$E/adapter_report.json"

build/amr_navigation/test/replay/production_precision_replay --scene "$E/scene.json" --output "$E/clearance_report.json"

Do not rerun accepted unrelated source suites. Run existing affected navigation
replay contracts once if shared support changes, as required by reviewer after
scope inspection. No fresh native simulation or readiness gates in this packet.

## Stop conditions and report

Stop at first mandatory failure (model mismatch, stale target/config/hash, schema
ambiguity, missing bracket, failing self-test/build, storage or first clearance).
Return to Sol diagnosis before another source edit. Do not repeatedly patch until
green. Routine typo/format corrections within the packet remain allowed.
Report changed files, scoped before/after diff, exact commands/environment/status,
hashes/evidence, first failed gate or all exercised cases and unproved runtime
properties. After adapter self-test, build, new CTest and scoped diff, HOLD for independent
completed-packet review. No captured-scene replay or native simulation by writer. Any Sonnet deviation or
established implementation mistake must be reported with evidence; faithfully
executing a flawed plan is not automatically a Sonnet mistake.
