# Centered-B boundary coverage C — bounded release

Root gpt-6.1-sol/high verified from current session turn_context. Release only
after the worker's actual gpt-6-luna/max metadata is verified. Current user resumed
2026-10-06. One test writer; scene worker has no source or test write permission.

## Objective, evidence and prediction

Finish the geometric and entry-evidence subset of the remaining B coverage.
HIGH confidence coverage gap: current behavior file has nine centered/admission
cases; scalar admission covers 0/.009/.010/>.010, but actual dock helper has no
residual injection cases. Current TF fixture supports defects2–7 without tests.
Existing post-arrival bias test passes; post-tangent construction remains untested.
Native44 and the genuine pre-heading target baseline retain causal evidence;
new helper coverage is not itself a claimed failing pre-fix regression.

Prediction: fresh post-tangent bias changes the actual clear target; helper
accepts physical residuals through .010m and rejects greater residual before
any short correction; malformed/stale entry evidence produces no navigation;
.151m commanded clear translation, original-reference drift>.15m, and arrival
heading error>.15rad fail without a later stage or repair.

## Allowed files and exact fixture decisions

Only src/amr_manipulation/test/test_gate6_pickup_retreat_behavior.cpp may change.
No production, CMake, Python contracts, parameters, source gates or public APIs.
Read AGENTS.md, DEBUG_PLAYBOOK.md, B_TURN_PLACEMENT_PLAN_20261006.md,
B_TURN_PLACEMENT_PACKET_B_20261006.md and B_REMAINING_COVERAGE_PACKET_20261006.md.
Preserve all existing dirty changes and A/CA1 fixture defaults.

Exclusive artifacts: existing packet_b parent, NEW boundary_c_resume1 directory.
Capture a before snapshot and pins, then inspect the complete delta against it.
Do not overwrite older commands/artifacts. Effective counters all188/Luna96;
existing implementation-attempt/hypothesis counts stay unchanged.

Required cases use names beginning Boundary in Gate6CenteredDockBehavior:

1. BoundaryPostTangentBiasChangesClearTarget: start physical(-2.5,0,pi/2),
   localized(-2.56,-.033,pi/2). At FIRST normal heading acceptance retain physical
   pose and change localized XY to(-2.528,-.009). Thus independently specified
   physical-minus-localized bias is(.028,.009); clear target is(-2.528,.091),
   dock target is immutable stance minus(.028,.009). Subsequent heading hooks
   simulate normal motion. Assert actual targets, dispatch2/clear1/precision1,
   physical dock proof, and no unrelated endpoint. Keep post-arrival test intact.
2. BoundaryPhysicalDockResiduals: inject actual post-dock Y residuals0,.009,
   .010,.0100001m through precision acceptance hook after normal simulated goal.
   Fresh physical/current TF evidence must remain coherent. First three succeed,
   last fails. Exercise actual helper and existing admission seam for each case;
   existing Python main-loop regression connects seam to skip. Exactly one long
   precision goal per call and no extra clear translation or short correction.
   Reset starting evidence/cancel state between calls only if required by the
   documented helper behavior. Do not bypass a failed safety state to continue.
   For this residual case only, a valid centered fixture with yaw0 and exactly
   zero immutable stance Y is authorized to represent physical .010 exactly.
   Preserve registered-scene fixture defaults in every other case and production.
   Assert CENTER selection, exact derived zero Y and measured requested residual.
   Represent equality carefully; report actual measured residual and avoid
   inventing an epsilon in production or weakening the threshold assertion.
3. BoundaryCurrentTfDefectsRejectBeforeNavigation: existing defect2–7 means
   .4s stale, .1s future, zero stamp, small nonunit quaternion, nonfinite position,
   nonfinite quaternion. Clear prior TF using existing coherent reseed method;
   verify fresh valid physical/base/attachment prerequisites for each case and
   fail with zero navigation. tf2 refusing an invalid transform is valid evidence
   if absence and the intended input defect are both observed.
4. BoundaryInvalidPhysicalEvidenceRejectsBeforeNavigation: nonfinite XYZ,
   invalid quaternion, stale receipt201ms and future receipt. Freeze only physical
   refresh before direct injection under pose_update_mutex then evidence_mutex;
   keep READY/odom/attachment/currentTF fresh. No timing sleeps needed. Assert
   defect and prerequisites, helper false, zero navigation. Existing frame test
   remains intact. Any fixture flags added default to the existing behavior.
5. BoundaryCommandedClearTranslationBound: physical start(-2.5,-.051,pi/2)
   with coherent existing bias is within .155m registered admission, but required
   .151m clear translation must not be sent after tangent proof. Expect helper
   false, tangent dispatch1, clear0, precision0; inspect physical and command
   geometry independently.
6. BoundaryOriginalReferenceDriftCancelsExactGoal: start physical(-2.5,0,pi/2),
   complete clear travel to(-2.5,.1); HOLD arrival heading and shift physical X
   by+.113m from initial X while retaining current Y. Total original-reference
   drift hypot(.113,.1)>.15m even though last-stage drift is only .113m.
   Keep fresh valid physical evidence. Require helper false, exact accepted
   arrival UUID cancellation AND terminal CANCELED, no dock or second repair.
   Use observables and bounded async cleanup; no success-versus-cancel race.
7. BoundaryArrivalHeadingMissCannotRepair: normal heading hook produces fresh
   physical clear-point proof but yaw error .151rad after successful heading.
   Expect false, no clear repair or dock; preserve valid unit orientation.

For async cases, ensure cleanup sets cancellation/releases barriers and joins
futures on assertion failure. Do not leave hooks capturing expired stack state;
clear hooks before local state is destroyed. Preserve coherent pose locks and
avoid recursively calling locking helpers from already locked callbacks.

## Checks, environment, stop conditions

Before edits: git status --short, reread targets, immutable pins/before snapshot.
Use one bound existing G1 environment root from B packet; validate its owned
home/ros/tmp/cache and home/.config/.local/share, ROS_LOG_DIR workspace/.ros_logs.
Domain232; PYTHONNOUSERSITE=1/PYTHONDONTWRITEBYTECODE=1/-B/core0. Source Humble
then overlay without nounset; MAKEFLAGS=-j2/CMAKE_BUILD_PARALLEL_LEVEL=2.
Canonical storage_budget.check(reserve=30000000) from workspace before growth;
no disposal. The concurrent scene task carries combined reserve150M.

Sequential commands with unique logs/statuses:
colcon --log-base <C>/build_logs build --packages-select amr_manipulation
  --symlink-install --executor sequential --event-handlers console_direct+
  --cmake-args -DBUILD_TESTING=ON
GTEST_FILTER='Gate6CenteredDockBehavior.Boundary*' GTEST_FAIL_FAST=1 ctest
  --test-dir build/amr_manipulation --output-on-failure
  -R '^gate6_pickup_retreat_behavior_test$'
GTEST_FILTER='Gate6CenteredDockBehavior.*:CenteredBAlignmentAdmission.*'
  GTEST_FAIL_FAST=1 ctest --test-dir build/amr_manipulation --output-on-failure
  -R '^gate6_pickup_retreat_behavior_test$'
GTEST_FAIL_FAST=1 ctest --test-dir build/amr_manipulation --output-on-failure
  -R '^gate6_pickup_retreat_behavior_test$'
python3 -B -m pytest -q --tb=short
  src/amr_manipulation/test/test_moveit_config.py
  src/amr_manipulation/test/test_product_test_contract.py
  -o cache_dir=<C>/pytest_cache
git diff --check -- src/amr_manipulation/test/test_gate6_pickup_retreat_behavior.cpp

For FULL CTest unset GTEST_FILTER/shard/repeat variables. Keep approved90s
aggregate registration unchanged; new duration risk is explicit, not permission
to raise it. Preserve runner inventory/XML and compare exact enabled case names.
If full run times out, retain partial result and stop for timing diagnosis; never
claim filtered passes as full-suite acceptance or silently raise the limit.

STOP immediately at first pin/storage/model mismatch, failed build/test,
contradiction, ambiguous requirement or packet deviation. Return exact evidence
to Root before another source edit or retry. No simulation/decode/install/global
changes. Shared ROS-time and delayed acceptance/terminal-success cases remain
pending for a separately resolved packet; do not improvise them in C.

Report progress<=2min; changed files/full before-to-after delta and pins; exact
commands/env/exit statuses/handles; requirement→test→actual result matrix; case
inventory/XML, durations, defects/prerequisites; remaining gaps and uncertainty.
HOLD for Root completed-packet review; author checks are not independent review.
