# Native45 planner-map correction — bounded packet, RELEASED

Planner: `gpt-6.1-sol` / high. Sole source writer: exact `gpt-6-luna` / max.
Independent reviewer: a separate non-author `gpt-6.1-sol` / high. Codex models
only; no model substitution, Claude worker or Astra. The user authorized this
bounded correction. Root reported the separate smoother review SPEC/QUALITY PASS
and recorded acceptance in SESSION_HANDOFF. Root reviewed the complete packet and
released this bounded source-only slice to Luna/max. Root owns all execution checks.

## Objective and confirmed diagnosis

Before every replay planning call, the selected phase's live planning costmap must
contain its original captured bytes. Determine direct precision versus Navfn
fallback from that restored map before invoking the real plugin. A plugin mutation
must neither contaminate later calls nor retrospectively change routing evidence.
This corrects replay evidence; it does not change the production planner's policy.

Classification: SOURCE / replay evidence harness. Confidence: high for the two
mechanisms, conditional on the current pins below. A focused behavioral baseline
has not yet been run for this slice; root owns that discrimination check.

Current source/control-flow evidence, reread on 2026-10-07:

- `production_precision_replay.cpp:1020–1034`: each `PlanningFixture` owns one
  live `Costmap2DROS`, one plugin and an independent `ReplayData` copy.
- `1102–1147`: `make_fixture` selects the captured grid and populates its live
  costmap once. `1169–1177` copies the original geometry and costs into
  `fixture.data`, independently of the live map.
- `1320–1328`, `1437–1443`: two fixtures are reused across stages/cases;
  `run_stage` selects dock versus clear, then directly invokes `createPlan`.
- `1457–1461`: routing is computed after the plugin call, by inspecting that
  same potentially mutated live costmap.
- `precision_navfn_planner.cpp:159–175`: the actual production plugin first
  evaluates `make_precision_segment` on its map and actual padded footprint,
  then calls `NavfnPlanner::createPlan` when the segment is empty.
- `precision_navfn_planner.cpp:59–67`: direct precision rejects a center cost
  >=253, while preserving the existing footprint boundary threshold.
- Installed `/opt/ros/humble/lib/libnav2_navfn_planner.so`: read-only disassembly
  shows `makePlan` calling `clearRobotCell` at address `0x1293b`, before passing
  live costs into NavFn. `clearRobotCell` zeros the unsigned-char cost argument
  at `0xb5c8` and jumps to `Costmap2D::setCost` at `0xb5ca`. This establishes
  the mutation in the installed collaborator without assuming remembered code.
- `Scene::grids` owns separate byte vectors (`GridView::costs`, line199);
  `ReplayContext::scene` is const. `sweep_path_against_grids:1337` checks those
  four captured grids, not the mutable plugin costmap. Preserve that boundary.

Failure mechanism: a fallback can clear a captured start cell, so subsequent
planning uses a different input. The post-call direct helper can then become
nonempty and report `direct_precision` even though the actual plugin selected
fallback from the original blocked center. Strict clearance may still reject;
its separate fail-closed behavior does not make the routing/input error valid.

Falsifiable prediction: with a single captured center cell253, otherwise free
planning space and clear footprint perimeter, the real plugin takes fallback and
clears its own start cell. The routing remains `navfn_fallback`; the next call
again sees captured253 at plugin entry. Removing restoration makes the second
entry see0; moving classification after the call reports direct precision.

## Scope, pins, worktree and ownership

Define `AMR_WS=/home/pete/amr_ws` once per shell/session, then derive paths.
The only allowed source edit is:

`src/amr_navigation/test/replay/production_precision_replay.cpp`

Current SHA256:
`43717c758ccbfc27d52a35da1549209d34dd2829d9e3053190bbdce5c3025897`.
Supporting files are read-only and must remain pinned:

- backend `60635c2e46e6ac5af7564018cecb85bf4afeb83129c0e1d4cf681933ea30bb56`;
- replay CMake `5d58aadb0e799105a154dc63f04221a3d4faa2b4f06a09b32d8c6a1bb1836ed6`;
- adapter `92ee621ef44ab0ab6932d66932f1ea506a34fdcb94eda84f2261e69f5df8d587`;
- production precision planner
  `bda846ef26b143e79af4c59be23a863ab4e33ce856c085e257dcd20e902e8710`;
- installed Navfn library
  `0f3565a32c6973cc4e72b2e0817440f9aaa03e472f6c9d669f1a4d4bae1b835f`.

Evidence root assigned by root, relative to `AMR_WS`:

`phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/codex_planner_map_20261007T090839Z`

Resolve `E="$AMR_WS/<relative root above>"`. Root has retained
`E/before/production_precision_replay.cpp` and `E/preparation.json`. Writer may
write its bounded diff/report artifacts in E, but no executable script, test,
build, replay or simulation outputs. Source-only edit/report/HOLD; root executes.

The source is untracked; backend and replay CMake already have unrelated intended
modifications. The workspace has many additional modified/untracked files and
pre-existing deletions. Preserve all existing work. Do not use an ordinary
`git diff` of the untracked source as the complete scoped delta; compare against
the retained exact before copy. Do not edit `SESSION_HANDOFF.md`, either ledger,
`AGENTS.md`, adapter, CMake, config, backend, planner, mission, manipulation or
protected `AMR_CODEX_HANDOFF.md`. No stage/commit/push, deletion, installation,
outside-workspace write, global change, refactor, bag decode or simulator.
Preserve historical failures, authorship and all-model210/Luna109 counters;
no new ledger entry is authorized.

## Smallest coherent implementation

1. Recheck `git status --short` and all pins; reread the actual target blocks.
   A pin change or overlapping writer is HOLD, not a reason to overwrite work.
2. Introduce one small result aggregate `PlannedStage`, with members
   `nav_msgs::msg::Path path` and `std::string routing`. Immediately before
   `run_stage`, add exactly one shared call seam named `plan_fixture_stage`,
   returning `PlannedStage`, taking a `PlanningFixture &` and the already
   stamped start/goal `PoseStamped` values. Keep this helper immediately adjacent
   to `run_stage`; root's evidence-only baseline transformation replaces it.
3. In that helper, read the original geometry/bytes from `fixture.data` through
   a const reference. Confirm the existing live map has matching width, height,
   resolution, origins and byte count before copying. Fail closed on mismatch;
   do not resize/reconfigure it or replace the map pointer retained by the plugin.
   Under the map mutex, overwrite EVERY live byte from the original costs,
   then call `amr_navigation::make_precision_segment` on that restored map and
   `fixture.padded_footprint`. Save the existing routing string according to
   segment emptiness. Release this pre-call mutex scope before invoking the
   actual plugin, which has its own locking. There is no activated background
   costmap updater or layer in this fixture; do not add one.
4. Invoke `fixture.plugin->createPlan(start, goal)` exactly once per helper call
   and return its actual path plus the saved pre-call routing. Preserve plugin
   exceptions. Do not substitute the helper segment for the actual plugin path.
   Live plugin mutations may remain after the call; the next call restores from
   the original captured bytes. Never copy live costs back into `fixture.data`
   or any `Scene::grids` byte vector.
5. Make `run_stage` invoke this seam inside its current plan exception boundary;
   preserve the current plan-failure handling and use its path in
   `poses_from_path`. Remove the old post-plugin classification block. Send the
   saved routing to the unchanged smoother/strict consumer. The helper is the
   sole replay `createPlan` call path. No unrelated helper redesign is allowed.
6. Add the focused tests below in this same source and one exclusive CLI mode
   `--self-test-planner-map`, using the existing `SelfTestResult` pattern. Also
   call the same groups from the full `--self-test`. Update file CLI comments,
   both usage strings, exclusivity counting and dispatch. The focused mode does
   not call `replay_scene`, the smoother or the full100-case fixture replay.
   No CMake or CTest registration change is needed.

Preserve: accepted±0.02m corresponding-coordinate footprint consistency allowance,
0.01m production padding, nominal/actual/conservative strict geometry, stance and
center/perimeter thresholds, all-four-grid full interior/boundary and heading
sweeps, malformed/empty-path handling, the mandatory explicit completion gate,
production1s smoother duration and backend2s default, phase selection, schema,
roster/exclusion policy, report provenance and `not_exercised` runtime claims.
This packet does not establish captured-scene clearance, readiness or simulation.

## Focused behavioral checks to author, not execute

Tests invoke `plan_fixture_stage`, the seam used by `run_stage`. Use actual
`make_fixture`/pluginlib-loaded PrecisionNavfnPlanner instances. A test-only
`GlobalPlanner` decorator may replace `fixture.plugin`: it records all live map
bytes at `createPlan` entry, then delegates to the actual already configured and
activated plugin. Its required lifecycle methods forward to that delegate so
fixture shutdown still owns proper cleanup. It introduces no alternate production
planning branch or helper-only fake. Retain the delegate lifetime. The observed
entry bytes, not a test-side restore, prove consumer-boundary delivery.

Build the test Scene from `validate_scene(make_fixture_scene())`, then modify only
the synthetic test scene BEFORE constructing its planning fixtures. Use explicit
cell-center coordinates in free interior space, such as start(-1.025,-0.025,0),
goal(-0.525,-0.025,0); confirm world-to-map succeeds instead of assuming indices.
Mark only the start center253 in the selected captured global grids. Leave the
perimeter/goal clear. Use harmless distinct17/23 marker costs away from the route
for clear/dock phase identity. Never change a captured production scene file.

Required separately named groups:

- `planner_map_classifies_before_real_navfn_mutation`: show the captured start
  is253, direct helper on an independent captured map is empty, and the decorated
  real plugin receives253. Call the shared seam; require nonempty real plugin
  output, live start0 afterwards, and nonempty post-call direct helper as the
  mutation witness. Require returned routing `navfn_fallback`, despite that
  post-call helper being nonempty. This isolates ordering from restoration.
- `planner_map_restores_every_phase_call`: use separate clear and dock fixtures,
  with distinct captured bytes. Invoke clear→dock→clear→dock through the same
  seam. Require each full recorded entry byte vector equals that fixture's
  original captured costs, including center253 on successive calls. After each
  first call prove real start clearing; additionally corrupt a live off-route
  cell before its next call and require it restored too. Compare complete byte
  vectors, not just the start or a counter. This isolates full restoration and
  cross-phase selection; it must reject baseline map reuse even when an earlier
  ordering assertion fails in a separate group.
- `planner_map_keeps_captured_collision_data_immutable`: after actual fallback
  clearing, require fixture original costs and all four Scene grid byte vectors
  equal their pre-call copies. `strict_check_path` against the captured selected
  global grid at the blocked start must still reject with
  `center_cost_inscribed_or_worse`, observed cost253 and the correct cell. Do
  not feed the mutable live map into strict collision validation.
- `planner_map_clear_positive_uses_real_direct_plugin`: both phase fixtures on
  an all-free captured Scene return nonempty real plugin output with
  `direct_precision`; recorded entry bytes equal captured bytes and plugin path
  frame/poses validate through `poses_from_path`. This catches an always-fallback
  report or a seam that never invokes the actual plugin.

Do not hide exceptions as a pass. If the installed real plugin does not produce
the expected mutation witness on the stated fixture, report HOLD rather than
invent another planner policy or silently substitute a synthetic mutator. The
source diagnosis is strong; these fixture conditions remain root-verified.

## Writer exit/report

Writer executes read/status/hash/diff commands only, edits the one source using
the edit tool, saves the full before/after delta and `E/implementation_report.md`,
then HOLD. No Python execution, compiler/build, self-test, CTest, replay, simulation
or process signalling. Report exact paths, before/after/support hashes, actual
changes, commands and statuses, deviations, material assumptions and unverified
conditions. Root immediately reports any established writer mistake with evidence;
no counter or ledger edit is implied. A faithful implementation of a failed plan
is not automatically a writer mistake.

## Root validation — exact ordered gates

Before growth, check canonical storage with `PYTHONDONTWRITEBYTECODE=1` and
`PYTHONPATH="$AMR_WS/phase14_evidence/factory_runtime_tools"`:

```bash
python3 -B -c 'import json, storage_budget; print(json.dumps(storage_budget.check(reserve=30000000), indent=2))'
```

Both logical and allocated logs/evidence limits remain1,000,000,000bytes; do not
dispose of old failures. Planning-only reserve1M passed on this preparation:
logical logs576366716/evidence789761660, allocated logs634597376/evidence809168896.
Root's prior30M receipt is retained in `E/preparation.json`; recheck before build.
Use E-owned ros_logs/tmp/cache, capture command/environment/source+binary hashes,
fresh stdout/stderr/status files, and stop on the first unexpected mandatory failure.

```bash
cd "$AMR_WS"
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=232 PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$E/ros_logs" TMPDIR="$E/tmp" XDG_CACHE_HOME="$E/cache"
export CMAKE_BUILD_PARALLEL_LEVEL=2
ulimit -c 0
git diff --check -- src/amr_navigation/test/replay/production_precision_replay.cpp
diff -u "$E/before/production_precision_replay.cpp" src/amr_navigation/test/replay/production_precision_replay.cpp
colcon --log-base "$E/build_logs" build --packages-select amr_navigation --executor sequential --cmake-args -DBUILD_TESTING=ON
"$AMR_WS/build/amr_navigation/test/replay/production_precision_replay" --self-test-planner-map
```

`diff -u` exit1 means expected source differences; inspect the complete delta.
Because the source is untracked, additionally whitespace-check that retained
delta rather than treating empty `git diff --check` output as proof. No build
before writer HOLD. Build must exit0; focused mode must exit0 with all four named
groups present, zero failures, real installed plugin entry/mutation proofs.

Then run ONE expected-red baseline discrimination, retaining the corrected
worktree and built target untouched. Root creates an evidence-only copy of the
corrected source under `E/baseline/`; replace ONLY the `plan_fixture_stage`
definition with the equivalent old behavior: invoke `fixture.plugin->createPlan`
first, omit ALL restoration, compute `make_precision_segment` on the post-call
live map, return `{actual_path, post_call_routing}`. Keep the new tests/decorator,
fixture setup and every other source byte identical. Save its exact diff and
hashes. This is the old real shared call sequence, not a unrelated test predicate.

Compile/link the one evidence translation unit with the generated target flags:
parse `build/amr_navigation/test/replay/CMakeFiles/production_precision_replay.dir/flags.make`
(`CXX_DEFINES`, `CXX_INCLUDES`, `CXX_FLAGS`) with `shlex.split`, add only
`-I "$AMR_WS/src/amr_navigation/test/replay"` for the copied source's backend
include, and compile with `/usr/bin/c++ -c <evidence_source> -o <evidence_object>`.
Parse the existing target's `link.txt` with `shlex.split`; replace only its target
object and `-o` output with the evidence object/binary, and run with cwd
`$AMR_WS/build/amr_navigation/test/replay` so relative library paths retain their
meaning. Use a root-generated evidence script, no shell eval or production source
swap. Retain compiler/link command argv and statuses. Existing target object and
binary total about12MB, so one copy is modest within the30M reserve; check actual
growth, logs and capped storage before/after.

Exact baseline invocation after successful evidence compilation/link:

```bash
"$E/baseline/production_precision_replay" --self-test-planner-map
```

Expected exit1 with BOTH named ordering and restoration groups failing for their
targeted assertions; immutable-grid and all-free positive groups should still
pass. A compile error, missing mode/group, load error or unrelated exception is
not baseline proof. Exit0 means the check is non-diagnostic: HOLD, no acceptance.
Expected-red baseline is an explicit diagnostic exception to first-failure stop;
any unexpected result stops. Reverify the corrected source/binary pins afterwards.
Do not run another corrected focused test absent new evidence/source changes.

Once discrimination passes, run the required existing replay contracts once:

```bash
ctest --test-dir "$AMR_WS/build/amr_navigation" --output-on-failure -R '^native45_clearance_contract$'
ctest --test-dir "$AMR_WS/build/amr_navigation" --output-on-failure -R '^(planner_replay_contract|precision_planner_contract|exact_goal_lattice_contract)$'
```

Capture individual gate statuses; first actual contract failure returns to
Sol/high diagnosis before any edit. Then separate non-author Sol/high completed
packet review is required, including preservation of footprint/smoother gates,
every real shared call, tests, baseline diff and supporting pins. Author claims
are not independent review. Root retains acceptance decision and safe resume.

Full captured replay remains deferred until all code checks/review are accepted,
current source/build/install plugin/config parity, actual loaded-library identity
(including earlier tf2 search-path warning implications) and frozen scene hash
are verified. Its later mandatory command, with a fresh output, is:

```bash
"$AMR_WS/build/amr_navigation/test/replay/production_precision_replay" --scene "$AMR_WS/phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/sonnet_adapter_correction_20261006T174631Z/scene.json" --output "$E/clearance_report.json"
```

Frozen scene SHA256:
`d8fdd7c954b63fc55d7d768509412b2f69aa6583052f3b6e9a4a0ac73f4d0128`.
First mandatory replay failure stops. No Native45 launch is released here;
readiness, identity preparation and strict A/B/home runtime gates still follow
the authoritative SESSION_HANDOFF dependency order.

## Stop and safe resume

Root release absent; wrong/unavailable assigned worker; changed pin;
overlapping writer; lost authoritative process; insufficient storage; scope or
contract ambiguity; missing mutation witness; failed build/focused/contract/
review/parity/replay gate; non-diagnostic baseline; contradictory evidence; or
user stop means HOLD for the affected slice. Return to Sol/high diagnosis before
another production edit. Preserve exact failed commands, environment, statuses,
named gates, map entry bytes, routing/mutation evidence, files and uncertainty.
Safe resume is the first unpassed dependency, never a reset of old evidence or
assumed whole-system success.
