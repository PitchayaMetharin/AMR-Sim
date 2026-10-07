# Repair only the failed malformed-current-TF boundary case

User: "fix the failed one first". Scope is a test-harness repair, not production
behavior or new coverage. Sol/high supervises; verified gpt-6-luna/max is sole
writer. Current metadata received verbatim: thread01a1109f-dd94-7c81-bc3f-5915a733a058,
turn01a11100-c770-7863-abf8-a26f5801687c, modelgpt-6-luna, reasoning_effortmax.

## Diagnosis and evidence

HIGH confidence TEST/EVIDENCE HARNESS defect. C repair1 built exit0 but all seven
ran6pass/1fail, CTest exit8/17.13s. In mode5 TF2 converts raw scaled quaternion
through Matrix3x3 s=2/length2, so lookup returns norm1 and actual consumer admits
it. Sol's packet assumed raw scale survives; M207 accounts the plan defect, not
a Luna mistake. Previous zero residual/coherent TF-reset fixes pass and remain.
Retained failing evidence: packet_b/boundary_c_resume3/ctest_boundary.stdout and
boundary_filter.gtest.xml under the existing B evidence root.
Root directly checked production1342–1376, native BufferCore178–198,
Transform48–51 and Matrix3x3 normalization155–169. Installed BufferCore three-arg
lookup declaration140–143 is virtual (`const override`); four-arg Buffer override
is a different overload. Suppress virtual dispatch with qualified native calls.

## Allowed change and contract

Only src/amr_manipulation/test/test_gate6_pickup_retreat_behavior.cpp may change.
Production pin2e9b1c37c783a10e01ab152de2d7ad6b39db67fb1c511c4081b3cc0021894650;
test before repair e9c8f1bf9a84c5845b4257293bf053e3bf1dfc0538276f2de895421dd26fa334.
Recheck git status and reread target include block, fixture defect setter/producer
and failed case. Preserve all unrelated dirty work and prior retained evidence.

1. Add a small test-only derived Buffer class in tf2_ros namespace with a unique
   descriptive name, native inherited constructors and native overloads exposed.
   Override the THREE-argument lookupTransform(string,string,tf2::TimePoint) const.
   First call qualified tf2_ros::Buffer::lookupTransform with unchanged arguments.
   An atomic per-instance boolean defaults false. Only when enabled for map to
   base_footprint at TimePointZero, scale returned quaternion components by1.0001.
   Preserve stamp/position/exception and every other native behavior. No observer,
   action, clock, validity or navigation mocking. Do not use a base static_cast
   call: virtual lookup would dispatch recursively into the override.
2. Preinclude all existing production CPP dependencies normally in the test TU
   before substitution, ensuring no third-party declaration is macro rewritten.
   Immediately around the existing production CPP include, scoped `Buffer`
   identifier substitution changes the concrete member to the test-derived type;
   undefine immediately afterward. Keep existing private-public/main scaffolding.
   Native listener still receives the derived native BufferCore instance.
   No exported header or production file changes. If dependencies/private access
   make this incompatible, stop and report before inventing another seam.
3. In set_tf_defect_and_reseed, while existing pose mutex is held, atomically
   enable return corruption iff defect==5 before clear/reseed. Remove raw scale
   injection for mode5 from producer: insert normal valid native TF, then corrupt
   lookup return only. Every other defect keeps its original native path.
4. In existing failed case, mode5 must explicitly prove native qualified lookup
   norm1 and intercepted result norm1.0001; keep existing norm tolerance1e-9.
   Do not allow a caught insertion/lookup exception to count as mode5 proof.
   Then actual latest_current_tf_pose must reject and actual centered navigation
   helper must return false with zero dispatch/clear/precision goals. Modes6/7
   may retain their original native insertion/absence semantics. Existing fresh
   physical/base/odom/attachment prerequisites remain. End via coherent setter0,
   disabling corruption; check normal fresh TF restored within the same case.

Keep exactly the same seven cases and all original gates/assertions/thresholds,
geometry, default fixtures, public interfaces, CTest TIMEOUT90 and runtime budgets.
No new test cases, production refactor, CMake/Python/config edits, broad suite,
simulation, replay, decoding, handoff update or cleanup. Packet attempt1 and
repair1 remain failed evidence; this is implementation attempt3/repair2.
Current all-model208/Luna107 after deferred Sol entries and source-search error;
preserve all counters. Failed new implementation returns to Sol before any edit.

## Prediction and exact validation

Default-off native behavior remains identical. For mode5 native transform is
valid but consumer receives norm1.0001 (norm-squared defect above1e-6), rejects
before any navigation. Same seven-case filter then passes7/7, no added case.

Exclusive evidence root:
/home/pete/amr_ws/phase14_evidence/stability_20261003/b_turn_placement_20261006/packet_b/boundary_c_resume4.
Require absent destination before creating it; preserve resume2/resume3.
Owned env root:
/home/pete/amr_ws/phase14_evidence/stability_20261003/G1_OWNERSHIP_20261005/env.
Use env HOME=<env>/home ROS_HOME=<env>/ros TMPDIR=<env>/tmp
XDG_CACHE_HOME=<env>/cache XDG_CONFIG_HOME=<env>/home/.config
XDG_DATA_HOME=<env>/home/.local/share ROS_LOG_DIR=/home/pete/amr_ws/.ros_logs
ROS_DOMAIN_ID=232 PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2; core0. In workspace cwd source
/opt/ros/humble/setup.bash then /home/pete/amr_ws/install/setup.bash without nounset.
No global files/settings/dependencies. Inspect directories with stat, not file
discovery (an empty owned directory is valid). Record exact commands/statuses.

Before retained growth, under that environment:
PYTHONPATH=phase14_evidence/factory_runtime_tools python3 -B -c 'import storage_budget; print(storage_budget.check(reserve=30000000))'

Save before test copy/pins and inspect complete repair delta. Sequential checks:
1. colcon --log-base <resume4>/build_logs build --packages-select amr_manipulation
   --symlink-install --executor sequential --event-handlers console_direct+
   --cmake-args -DBUILD_TESTING=ON
   Expected exit0. Capture stdout/stderr/status plainly, no nested receipt scripts.
2. GTEST_FILTER='Gate6CenteredDockBehavior.BoundaryCurrentTfDefectsRejectBeforeNavigation'
   GTEST_FAIL_FAST=1 ctest --test-dir build/amr_manipulation --output-on-failure
   -R '^gate6_pickup_retreat_behavior_test$'
   Expected exit0/1 named case, actual malformed norm proof and zero goals.
   Copy actual runner XML to exclusive named-result file before next command.
3. GTEST_FILTER='Gate6CenteredDockBehavior.Boundary*' GTEST_FAIL_FAST=1
   ctest --test-dir build/amr_manipulation --output-on-failure
   -R '^gate6_pickup_retreat_behavior_test$'
   Expected exit0/exact7 cases,0fail/disabled/skipped; retain stdout/stderr/status/XML.
4. git diff --check -- src/amr_manipulation/test/test_gate6_pickup_retreat_behavior.cpp
   Expected exit0; confirm production/header/CMake unchanged against initial pins.
Then HOLD; no broader checks, new cases, simulation or replacement work.

Stop at first failed check, inconclusive/contradictory result, scope/API ambiguity,
model mismatch/unavailability, lost handle, storage failure or user stop. Do not
patch again or enlarge injection to force pass. Report changed files and complete
delta versus this packet, exact commands/exits, evidence, actual named-case/count
results, handles, blockers and uncertainty. Progress at least every two minutes.
Sol independently checks completed diff and actual source/logs before acceptance.

## Source-edit continuation after rejected patch context

Root directly compared current file to resume4 before snapshot. Only step1/2
preincludes/derived Buffer/scoped macro have applied; their semantics match the
packet. Second patch failed atomically because EXPECT_TRUE context did not match
existing ASSERT_TRUE prerequisites. This is a patch-construction error, not new
TF-contract evidence. No build/check is active. M209 recorded in both ledgers;
all209/Luna108, same implementation attempt3 retained, context rejection retained.
After verifying current model/effort, reread exact setter/producer/case blocks and
apply only remaining step3/4 against actual ASSERT_TRUE source. Preserve those
prerequisite assertions. Keep current correct seam, original checks/stops/env and
exclusive resume4. No diagnosis change, new case or extra check is authorized.

## Check continuation after named-test cache typo

Build exit0/36.7s; named command exit0/.21s. Root directly reads current XML:
exact BoundaryCurrentTfDefectsRejectBeforeNavigation,1completed/0fail/disabled/errors.
XDG_CACHE_HOME had an extra incorrect G1_OWNERSHIP_202605 component; all other
reported env/domain values correct. Wrong path remains inside workspace and Root
confirmed it absent. This is M210 tooling/environment deviation, all210/Luna109,
not a source/functional failure. No additional source edit or named rerun needed.
Preserve named XML before overwrite; derive ALL env paths from ONE correctly
assigned owned env-root variable, including XDG_CACHE_HOME=<env-root>/cache.
Continue original seven-case filter, diff/pins and receipts, then HOLD. The fresh
seven run covers the repaired named case under the fully correct environment.

## Completed repair2 — verified scope, not simulation acceptance

Build handle19347 exit0/36.7s. Named case XML1completed/0fail/disabled/errors,
CTest exit0/.21s with cache-path deviation M210 disclosed above. Correct-env
seven-filter handle40502 exit0/15.40s; retained boundary_filter.gtest.xml has
exact original seven names,7completed/0fail/disabled/errors, including repaired
TF case. Worker reports scoped git diff --check exit0, no live process.
Root directly reads actual statuses/stdout/XML, full delta versus before snapshot
and production/header/CMake/protected-handoff pins. Delta matches this packet;
native lookup delegate is qualified, corruption defaults off and is mode5-only,
mode5 native/intercepted norms asserted without catch, coherent cleanup restores
valid TF. Safety/thresholds and original seven cases unchanged.
Final behavior-test SHA256:
82fe4897a45ae8ed50cc4d85b476a964eccf63eb93233f40e6c801ae241ad2f9.
Production2e9b1c37..., stancef1820271..., CMake3a3f7585..., protected AMR handoff
032bf299... unchanged. All210/Luna109, prior attempts/evidence retained.
Only this test-harness repair is complete. Broader current82/Python/integration,
separate nonauthor milestone review, clearance and fresh Native45 remain pending.
No new tests, broad rerun, simulation, replay, decode or SESSION_HANDOFF update.
Workers hold; current user scope is the failed-test repair first.
