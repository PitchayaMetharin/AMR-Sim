# Native45 smoother-completion correction — bounded release

User explicitly authorizes fixing incomplete smoothing now. Sol/medium orchestrates,
Sol/high diagnoses/reviews, exact Sonnet5.5/medium is the sole source writer.
Writer edits/reports then HOLD; root runs mandatory checks. This slice does not
release planner-map correction, full replay or simulation.

## Diagnosis and contract

Current production mission_supervisor_node.cpp:653 requests1 s smoothing;
714-716 requires SUCCEEDED, a present result, was_completed=true;740-742 aborts
non-success/missing/incomplete/empty results before controller acceptance.
Current production_precision_replay.cpp:1397-1402 calls the real configured
SimpleSmoother with1 s and already catches exceptions. But1412-1415 merely counts
false/missing completion and continues strict sweeps, routing records and later
stage/case acceptance.1523-1535 shows returningfalse from run_stage stops that
acceptance.1469 incorrectly says completion is reported, not gated.

Confidence high: the actual current source establishes the missing transition
gate. This is the previously reported Sonnet implementation defect. No new runtime
failure is inferred; captured replay has not executed.

Important helper reality: planner_replay_backend.cpp:668-669 calls synchronous
SimpleSmoother::smooth;686 stores its bool as smoother_completed.702-703 sets
JSON success to a composite of completed, old perimeter collision, and endpoint
tolerance compatibility. There is NO action result code in this helper. Do NOT
equate that composite success key to production action SUCCEEDED and do NOT add
an endpoint-alias gate. In this synchronous replay, successful invocation means
no exception; completion additionally must be present, boolean, and true. All
existing strict collision/empty-path gates remain authoritative after completion.
No backend edit or invented action status is needed.

## Pins, evidence, scope and state

Define AMR_WS=/home/pete/amr_ws once; derive all workspace paths from it.
Fresh E assigned by root:
phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/sonnet_smoother_20261006T185243Z
Root saved the source before-image there. Verified input SHA256:
- production_precision_replay.cpp:91fba11fdd82eec0c571bec8c9a38dfeb977bcbd3e425258c353ad616383924c
- planner_replay_backend.cpp:60635c2e46e6ac5af7564018cecb85bf4afeb83129c0e1d4cf681933ea30bb56
- replay/CMakeLists.txt:5d58aadb0e799105a154dc63f04221a3d4faa2b4f06a09b32d8c6a1bb1836ed6
- adapter:92ee621ef44ab0ab6932d66932f1ea506a34fdcb94eda84f2261e69f5df8d587
- mission source (read-only authority):03ed5a016c8489ea8071c342c1fc9f7812755b1e38de21a4c15e2e30bf152c33

Accepted footprint evidence remains under
phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/sonnet_footprint_20261006T182650Z/assertion_repair/
Root build exit0 and footprint focused CLI exit0, sixgroups passed; independent
SPEC/QUALITY PASS. Retain the original earlier5PASS1FAIL assertion-format failure,
its repair and all prior failure history. No ledger/counter changes;210/109 remain.
Worktree has numerous unrelated modified/deleted/untracked files; new replay source
is untracked and backend/CMake already modified. Preserve all of them.

Allowed source ONLY: src/amr_navigation/test/replay/production_precision_replay.cpp.
Backend, CMake, adapter, configs, production mission/navigation/manipulation,
manifests and all other sources remain untouched. No protected AMR_CODEX_HANDOFF,
handoff/ledger edits, commit/stage/push, dependency install, deletion or bag decode.
Writer may save before/after diff, hashes and report only inside assigned E.

## Required smallest coherent change

1. Keep the real1 s smoother call and exception failure unchanged. At its result
   consumer boundary, before any strict sweep or successful-stage record, require
   result to be an object with smoother_completed present, boolean and true.
   Null/missing/nonboolean/false completion fails closed; use useful distinct
   reasons, with category smoother and case/stage/provenance from failure_base.
   Record observed completion value/presence and actual1 s budget in failure proof.
   Missing whole result object likewise rejects. Do not use JSON success as an
   action code. Maintain the existing budget and nonempty path checks, allowing
   clearer order so incomplete completion cannot become a successful stage.
   A nontrue result may count as smoother_incomplete, but must then returnfalse.

2. Extract only necessary post-smoother stage processing into a small seam used
   by BOTH actual run_stage and focused tests: completion/budget/path validation,
   existing raw/smoothed strict sweeps, routing count and successful record.
   Inputs include actual result/raw/smoothed path and existing stage context;
   production real planner+smoother calls stay in run_stage. Do not add a fake
   planner, substitute production smoother or redesign the full replay driver.
   Tests must exercise this consumer seam, not merely a bool predicate. Failed
   consumption returnsfalse with failure proof, leaves no successful record or
   routing/stage/case acceptance and performs zero strict sweeps. Existing outer
   replay stop1523-1535 remains unchanged and independently inspectable.

3. Correct the report limitation to say completion is mandatory and wall-clock
   completion can vary with host load. Preserve runtime not_exercised claims,
   footprint allowance0.02 plus existing numeric boundary noise, conservative
   BOTH-phase envelope,1e-9 nominal/stance checks, all4 frames, raw immutable
   strict costs,>=253 center rule, lethal/unknown/outside-map rules and heading
   sweeps/interpolation bounds. No planner-map restoration/routing-order change.

4. Add --self-test-smoother-completion, mutually exclusive with existing CLI modes;
   run only the focused groups below and print explicitly limited scope. Retain
   --self-test and --self-test-footprint behavior; full self-test also includes
   new groups. No CMake/CTest registration change. No replay_scene invocation in
   focused mode; no100-case fixture replay. Backend default2 s remains untouched.

## Required discriminating focused tests, written but not run by writer

- Consumer receives nonempty finite paths on clear grids and result with
  smoother_completed=false. Assertfalse return, useful incomplete reason/proof,
  zero strict samples/cells, no successful record/routing and no stage/case
  acceptance. Collision/path geometry alone would previously have passed.
- Repeat with missing completion, null/nonobject whole result, and completion
  null/string/number; all failclosed without throw-to-acceptance or continuation.
  Keep numeric/structural assertions; use existing formatting when checking proof
  numbers, avoiding the previously demonstrated brittle decimal substring error.
- Real configured SimpleSmoother on a bounded clear finite multi-pose path with
  explicit1 s returns explicit boolean completiontrue and nonempty output; feed
  its ACTUAL result/output into the same consumer seam and assert success record,
  exercised strict sweeps and no failure. Existing helper composite success is
  not substituted for completion. Preserve exact mission duration and olddefault.
- Positive synthetic completiontrue uses the same seam; false or missing helper
  composite success alone is NOT treated as missing production action status.
  These fixtures still meet existing strict path/collision gates. Conversely,
  completiontrue must not bypass empty-path, wrongbudget or strictcollision gates.

Falsifiable prediction: nontrue/missing/malformed completion stops immediately
before sweeps and successful acceptance; completed valid output proceeds through
all existing strict checks; real smoother still receives1 s and old backend2 s
default is unchanged. No claim of action-server execution or historical parity.

## Writer HOLD and root checks

Before editing run git status --short, reread target/current consumer and pins,
confirm scope. After editing inspect full scoped before/after diff (untracked
source is absent from ordinary git diff), report exact changes, commands/statuses,
hashes, pending concerns and HOLD. Writer runs NO Python, compiler/build, tests,
CLI/self-test, replay or simulation. Material ambiguity/pin mismatch/denial→HOLD.
Routine source slips may be corrected in-place within this packet.

Root uses fresh E command/environment/status/stdout/stderr/storage evidence,
owned E/{ros_logs,tmp,cache}, reserve30 MB,1 GB logical+allocated caps and no
disposal authorization. ROS_DOMAIN_ID232, CMAKE_BUILD_PARALLEL_LEVEL2. Root
creates owned dirs and checks storage first; then sequential gates, STOP firstfail:

```bash
export AMR_WS=/home/pete/amr_ws
cd "$AMR_WS"
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=232
# Root resolves E to the fresh assigned absolute evidence path first.
export ROS_LOG_DIR="$E/ros_logs"
export TMPDIR="$E/tmp"
export XDG_CACHE_HOME="$E/cache"
export CMAKE_BUILD_PARALLEL_LEVEL=2
git diff --check -- src/amr_navigation/test/replay/production_precision_replay.cpp
colcon --log-base "$E/build_logs" build --packages-select amr_navigation --executor sequential --cmake-args -DBUILD_TESTING=ON
"$AMR_WS/build/amr_navigation/test/replay/production_precision_replay" --self-test-smoother-completion
```

Root also inspects full scoped diff and verifies backend/CMake/adapter pins unchanged.
Do not rerun accepted footprint checks absent a material relevant source change;
independent reviewer checks footprint preservation from exact diff. After focused
PASS, independent Sol/high reviews actual source and fresh retained evidence.
Build/test/review failure returns to diagnosis before another production edit.
No fullcontract, captured replay or native simulation: planner-map mutation and
postmutation routing remain pending and must not be silently accepted. Final
report distinguishes this completion slice from overall clearance/runtime gates.
