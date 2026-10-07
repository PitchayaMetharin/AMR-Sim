# Native45 footprint correction only — bounded release

Latest user direction supersedes the preceding proposed three-defect correction:
finish the footprint first. Sol/medium orchestrates; Sol/high diagnoses/reviews;
exact Sonnet5.5/medium remains sole source writer. This packet releases only the
footprint correction. Writer edits and reports, then HOLD; root owns checks.

## Objective, authority and diagnosis

Allow the production padded footprint to differ from the nominal scene polygon by
at most +/-0.02 m per corresponding X/Y coordinate. This is the smaller of the
allowances explicitly offered by the user and selected transparently by root.
It supersedes the earlier 1e-6 m plugin/scene numerical comparison requirement.
It does NOT authorize smaller collision geometry, changed padding, changed costs,
or changes to stance agreement or nominal scene-shape validation.

Root's first mandatory CTest failed: native45_clearance_contract exit8, eight
groups passed and two plugin-replay groups failed before planning, both category
fixture_setup / plugin padded footprint disagrees with the scene padded footprint.
Root build had passed exit0. Retain this failure without overwriting it.

Cause is confirmed: production_precision_replay.cpp:982-989 compares the actual
Costmap2DROS padded polygon to decimal-double scene coordinates using
kPaddingTolerance=1e-9 at line42. Installed nav2_costmap_2d/array_parser.hpp:47
returns vector<vector<float>>. Installed makeFootprintFromString calls parseVVF
(libnav2_costmap_2d_core.so disassembly address0xcd236), then promotes floats to
doubles (0xcd6eb/0xcd6f9). Thus 0.6 parses as0.6000000238418579 and0.4 as
0.4000000059604645 before0.01 padding. The getter IS the padded footprint:
installed costmap_2d_ros.hpp:269-271 returns padded_footprint_. Do not substitute
the unpadded getter or change production parsing. Confidence: high.

Sonnet's implementation defect was the overstrict plugin/scene comparison.
The two other confirmed source defects remain explicitly pending: incomplete
smoothing currently continues (new source1245-1248, contrary to production mission
715-716/740-742); planner fixture costs are populated once975-979 and may retain
Navfn start-cell mutation across stages, with routing classified after mutation
1202/1218-1220. Do not fix either in this packet and do not claim whole replay PASS.

## Inputs, pins and worktree

Define AMR_WS=/home/pete/amr_ws once and derive paths from it.
Existing writer evidence E0:
phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/sonnet_cpp_20261006T180036Z
Evidence: contract.stdout.log, contract.stderr.log, contract.status; build.status;
implementation_report.md and scoped_before_after.diff; git_status_before.txt and
git_status_root_checks.txt. The report's assertion that no Python was executed is
incorrect: the already-reported empty Python heredoc deviation is retained in
empty_python_command_deviation.json. No additional mistake is inferred from it.

Source pins verified read-only before this packet:
- production_precision_replay.cpp:5cfa0390a3d6324950c566c2acd47a77c78353c4e3a57a654830c64aa7a97bcc
- replay/CMakeLists.txt:5d58aadb0e799105a154dc63f04221a3d4faa2b4f06a09b32d8c6a1bb1836ed6
- planner_replay_backend.cpp:60635c2e46e6ac5af7564018cecb85bf4afeb83129c0e1d4cf681933ea30bb56
- adapter:92ee621ef44ab0ab6932d66932f1ea506a34fdcb94eda84f2261e69f5df8d587
- accepted scene:d8fdd7c954b63fc55d7d768509412b2f69aa6583052f3b6e9a4a0ac73f4d0128

The worktree contains numerous unrelated modified/deleted/untracked files.
The replay CMake/backend are modified and new replay source is untracked. Preserve
all existing state. Do not modify protected AMR_CODEX_HANDOFF.md or any ledger;
counters remain210/109. Fresh assigned evidence root E1 is
phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/sonnet_footprint_20261006T182650Z
Root saved before/production_precision_replay.cpp there. Do not reuse E0
reports/status files.

## Only allowed source

src/amr_navigation/test/replay/production_precision_replay.cpp

Reread this file and run git status --short before editing. No CMake, backend,
adapter, production/config/manifest/mission/manipulation or other source edit.
Writer may retain before-image, complete scoped diff, hashes and implementation
report in E1. Do not stage, commit, install dependencies, delete evidence, decode
bags, run Python, build/test/replay/simulation, or write outside workspace.

## Required minimal control-flow change

1. Keep kPaddingTolerance=1e-9 for existing nominal scene shape and expected
   footprint-plus-padding checks, and keep kStanceAgreementTolerance=1e-9.
   Introduce a separately named plugin/scene consistency allowance0.02 m.
   In make_fixture compare each corresponding actual padded X/Y to scene nominal
   padded X/Y: abs(delta)<=0.02, inclusive. Reject nonfinite values, wrong vertex
   count/order or invalid shape; preserve current four-corner rectangle contract.
   Error proof names phase/vertex/coordinate, actual, expected, signed delta and
   allowance in metres. The allowance cannot alter the nominal scene source.

2. Preserve actual plugin footprint and its original0.01 padding. Derive a
   conservative body-frame collision polygon enclosing the expected polygon AND
   the verified actual padded polygons of BOTH clear and dock fixtures. Current
   geometry is rectangular: the smallest axis-aligned bounding rectangle of all
   vertices is a simple bounded solution; retain the same clockwise vertex order
   (+maxX,+maxY),(+maxX,minY),(minX,minY),(minX,+maxY). Do not shrink any bound.
   No need for generic geometry dependency/refactor. Reject degenerate/nonfinite
   result. The allowance accepts consistency only; do not subtract it from shape
   or grant collision slack.

3. ReplayContext owns this verified conservative polygon after BOTH fixtures are
   configured and before stage processing. All strict raw/smoothed path and start/
   terminal heading sweeps use it, including circumradius interpolation bounds.
   All FOUR original captured grids remain checked, in their existing correct
   frames. Immutable strict costs, cost>=253 center rejection, lethal/unknown
   interior/boundary rejection, outside-map rejection and half-cell motion bound
   remain unchanged. Production plugin and smoother retain actual fixture inputs;
   only the independent strict collision-check polygon becomes conservative.

4. Report nominal padded polygon, both actual phase polygons, each phase's signed
   vertex deltas/max absolute coordinate delta, consistency allowance0.02 m and
   final conservative polygon. Keep existing schema and provenance; additive keys
   only. Do not report exact footprint equality when allowance was used. Keep
   runtime properties not_exercised, report size bound and first-failure behavior.

5. Add a narrowly scoped CLI --self-test-footprint, reusing existing fixture,
   comparison/envelope and strict-check seams. It runs ONLY the footprint-focused
   groups below, not replay_scene, smoother checks or the100-case replay fixture.
   Existing --self-test retains all existing groups and includes these new groups.
   No CMake test registration change is needed; root invokes the executable CLI
   directly. Distinguish this limited PASS from full native45_clearance_contract.

## Required discriminating source tests (writer does not execute)

- Actual production Costmap2DROS parsing/configuration with nominal0.6/0.4 and
  padding0.01 is accepted in both phase fixtures; show nonzero float-rounding delta
  and conservative envelope enclosing actual plus nominal polygons.
- A matching ordered rectangle with an actual bound enlarged by0.019 m passes
  comparison; an exactly0.02 m coordinate difference passes (use0 and0.02 for a
  deterministic boundary assertion, without a comparison epsilon); a0.0201 m
  difference fails with useful proof. Do not merely assert constants.
- Clear and dock actual polygons with different enlarged bounds produce ONE
  envelope covering both and nominal. Validate every input vertex lies inside it.
- Place a lethal cell covered by the enlarged envelope but clear of the nominal
  polygon (select cell/grid geometry to discriminate0.019 m expansion). Nominal
  strict checker passes; real conservative strict seam fails with exact cell/cost.
  This proves allowance cannot hide collisions. Also retain unknown/outside-map
  rejection through the same unchanged strict checker.
- Material malformed/nonfinite polygon/count/order still rejects. Existing nominal
  scene-shape1e-9 and stance1e-9 validations remain diagnostic and unchanged.

Prediction: production float-rounded fixture setup no longer falsely fails;
verified differences<=0.02 are admitted; larger differences fail; newly enclosed
lethal geometry fails rather than benefiting from the consistency allowance.

## Writer HOLD, root validation and stop rules

Writer inspects complete scoped diff, reports exact changes/commands/statuses,
before/after hashes and untouched file checks, then HOLD. Writer does not execute
any check, not even the focused CLI. Routine edit typos may be corrected in scope;
material ambiguity, denied operation or input pin mismatch requires HOLD/report.

Root retains fresh command/environment/status/stdout/stderr and storage evidence
inside E1. Reuse existing owned E0 environment directories only if root verifies
ownership; prefer E1/{ros_logs,tmp,cache}. Use ROS_DOMAIN_ID232, reserve30 MB,
existing1 GB logical/allocated evidence/log caps, no disposal without authorization,
and CMAKE_BUILD_PARALLEL_LEVEL2. Sequential mandatory gates; STOP first failure:

```bash
export AMR_WS=/home/pete/amr_ws
cd "$AMR_WS"
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=232
# Root resolves E1 to the fresh assigned absolute evidence path first.
export ROS_LOG_DIR="$E1/ros_logs"
export TMPDIR="$E1/tmp"
export XDG_CACHE_HOME="$E1/cache"
export CMAKE_BUILD_PARALLEL_LEVEL=2
git diff --check -- src/amr_navigation/test/replay/production_precision_replay.cpp
colcon --log-base "$E1/build_logs" build --packages-select amr_navigation --executor sequential --cmake-args -DBUILD_TESTING=ON
"$AMR_WS/build/amr_navigation/test/replay/production_precision_replay" --self-test-footprint
```

Root creates the owned environment directories and checks storage before commands;
also inspects the before/after diff because the new file is untracked and git diff
alone does not include it. Check immutable CMake/backend/adapter/scene pins above.
After focused PASS, independent Sol/high review verifies completed source plus root
evidence. Do NOT run full contract, captured replay or simulation in this packet;
the two explicitly pending production-fidelity defects still block those gates.
Do not reset original failure history or mark overall clearance accepted.

If build, focused CLI or independent review fails, report exact evidence and return
to diagnosis before another production edit. Report accepted footprint scope only,
remaining smoother/map defects, unexecuted runtime properties and safe resume point.
