# Centered B placement packet — RELEASED 2026-10-06

Coordinator: exact gpt-6.1-sol/high, independently verified session metadata.
Writer: sole verified gpt-6-luna/max, same worker as Packet A. Begin this packet
only on Root release after reporting Packet A; no concurrent production writer.
Root released after A baseline genuinely failed and candidate scoped build and
exact two-test CTest passed. Same worker metadata reverified gpt-6-luna/max.
Read AGENTS.md, DEBUG_PLAYBOOK.md and B_TURN_PLACEMENT_PLAN_20261006.md.

## Diagnosis, evidence and prediction

HIGH confidence stale-target mechanism: existing clear helper preserves target
across tangent heading (mass-stage1398–1409). Dock target2991–2994 is corrected
before aligned helper1297–1305 can issue another heading. Native44 factory.log
1012–1020 records that intervening heading followed by original precision
target;1039–1040 records localized arrival<=.01 but physical stance residual
.0481m. Final short correction failed at1061–1062. Upstream curvature cutoff
31.62mm cannot remove recorded remaining lateral error under10mm gate.
Full runtime convergence prediction remains unverified.

Prediction: every precision target reflects stopped physical/current-TF bias
AFTER the preceding heading; direct long precision approach reaches immutable
stance physically<=.01m or fails without issuing a short centered correction.
One physical-clear position miss may repair once under the original clear
reference/shared budget; heading or safety failures cannot authorize repair.

## Allowed changes and protected state

- src/amr_manipulation/src/gate6_mass_stage.cpp
- src/amr_manipulation/test/test_gate6_pickup_retreat_behavior.cpp
- src/amr_manipulation/test/test_moveit_config.py
- src/amr_manipulation/test/test_product_test_contract.py only if affected
  existing contract must change to express the authorized behavior.

No other production/test/config edits. Exclusive evidence/diagnostic programs:
phase14_evidence/stability_20261003/b_turn_placement_20261006/packet_b.
Root owns policy, handoff and ledgers. Preserve unrelated work and
AMR_CODEX_HANDOFF.md (SHA032bf29994fae1610585a62a0a23d26a23eda6753e78a3fb1513d16bdab5f114).
Pre-packet production SHA f20dfee90387934f23eea59a541d306c39bc411ca95cfc034fb714d5f369af73;
behavior test SHA383ffa4547795815f4af0dde14df6709246564eef0272fed7ecef95a0d772485.
Recheck status, reread files and retain scoped before snapshots/current pins.

## Required implementation

Keep one small internal centered-B choreography helper, directly exercised by
the existing real action harness. Reuse existing clients/action endpoints and
validation primitives. Extract CA1 current-TF validation into a shared private
member; preserve legacy CA1 behavior and its tests when sharing that validator.
Do not create a generic navigation framework or alter public ROS interfaces.

The specialized helper takes immutable physical stance and existing timeout,
and returns stopped physical/current-TF dock evidence for common downstream
placement proofs. It rejects product!=102 or non-centered branch misuse.

1. Capture fresh physical/current TF at entry, validate finite position and
   unit quaternion before any bias or yaw computation. Physical receipt<=200ms
   (preserve existing physical freshness contract); reject wrong physical
   frame if inconsistent with recorded factory_world contract. TF must have
   positive canonical stamp, finite position, valid unit quaternion and age
   0..0.30 simulation seconds. No feedback-pose substitute.
2. Save original physical clear-area position once and simulation start/previous
   clock once. Derive clear point as the projection of registered approach
   onto the line through immutable stance along its physical unit heading.
   Current scene stance(-3.345,.100,pi), clear(-2.500,.100). Margin .03 is used
   only in the later existing final-heading stage, never this line.
3. Preserve .155m registered-approach admission and .15m clear-area displacement
   limits. Clear-area movement, including any repair and in-place drift, uses
   the SAME original physical reference. Validate commanded clear translation
   <=.15m as well. Keep finite geometry checks before actions.
4. For needed clear translation, choose closest forward/reverse physical tangent
   from fresh physical pose to clear point. Send normal owned heading at current
   TF XY, yaw=tangent minus current physical/localized yaw bias. Observe stopping
   and capture AGAIN before constructing corrected clear translation target.
   Use existing clear-approach action directly, retaining its position checker
   and reverse-capable controller. If already physically within .01m of clear
   point, do not manufacture a tangent from zero displacement or issue a
   needless translation. Heading stages still prove arrival heading below.
5. After clear translation, observe stopping/capture, send normal owned arrival
   heading at CURRENT TF XY with yaw=stance physical yaw minus FRESH yaw bias.
   Observe stopping/capture again before any dock target construction.
6. Prove physical clear XY<=.01m and physical heading error<=.15rad. Safety,
   freshness, attachment, action result, clock or heading failure is fatal.
   Only a physical XY miss with all other proofs valid may take one clear-area
   repair. Recompute tangent and all targets from fresh stopped evidence,
   repeat steps4–6 under SAME reference/budget. Second miss fails closed.
7. Only after successful proof construct dock XYZ/yaw target from immutable
   stance minus this latest physical/localized bias; call existing dispatch
   precision client directly. Never use aligned-precision helper here.
8. Observe stopping/capture after dock and prove physical stance XY<=.01m.
   Report residual and fail if larger. Return the fresh physical/localized
   evidence to main. Keep existing physical dock admission/attachment checks.

The .15/.155 envelopes apply through final pre-dock capture, not long dock
traversal(~.845m). Preserve existing travel safety gates during dock. One120s
simulation budget covers clear stages, optional repair, dock and post-dock
observation/capture. Separate existing final-heading stage keeps its own
existing budget. Clock rollback or exhausted sequence budget fails admission.

## Action, cancellation and observation contracts

The new stopping observer must only observe. Require READY base valid/nonzero
boot/sequence/reason READY and base+odom receipts<=200ms; measured linear X/Y and
angular Z each<=.01 continuously500ms. Bound each observation by8wall seconds
and remaining shared simulation budget; cancellation/attachment/physical-bound
loss or rollback fails immediately. No wait_for_motion_permission, set_status,
manipulator publication, AMCL reseed or authority change.

For only the specialized sequence, an optional internal action guard defaulting
to empty may be added to navigate_to_with_client. Existing generic callers
remain unchanged. It checks shared monotonic simulation budget, cancellation,
attachment and fresh valid physical evidence; clear stages also check original
clear displacement bounds. Evaluate outside evidence_mutex_ (guard acquires it):
entry, after server wait/immediately before send, acceptance waits, active result
loop before/after waits, and before terminal success admission. A passed per-goal
feedback duration alone does not prove the full shared budget. Preserve existing
feedback invalidity/staleness/navigation rollback checks and goal timeouts.

Before acceptance failure: mark pending goal abandoned, preserve late-response
callback's cancellation ownership, and prevent later actions. Once accepted:
use existing exact UUID cancellation and terminal CANCELED path; never cancel
all goals. If guard fails when result is already terminal, fail admission and
consume/log that terminal without claiming an additional cancel is required.
No next stage may start after unsafe or unproven terminal. Do not call guard
from persistent late callback if it captures helper locals with shorter lifetime.

## Main integration and final heading decision

Limit new route to final_placement_stance.product102_center_slot. This branch
replaces old clear calculation/aligned dock call; A/non-centered code preserves
its old flow. Feed returned stopped dock evidence into downstream bias and
previous_alignment_pose variables. Finite geometry/attachment/cancellation and
existing displacement checks precede admission. Centered residual0/.009/.010m
skips positive segment rejection AND entire short precision translation loop;
>.010m rejects before any short correction. Generic positive distance/segments
and loop remain unchanged. Test actual invoked branch, not a copied formula.

Preserve existing separate final-heading formula, .03margin, physical .07m/.15rad
final placement envelope, .15m final-heading displacement, .35m total alignment,
post-permission reference-drift and arm/attachment/collision/release proofs.
The new .01m physical stance proof is DOCK admission as explicitly supplied in
the user's plan; no new .01m final-heading acceptance gate is authorized here.
Measure/report any later final-heading drift. Centered final-heading bias may
use fresh current-TF/physical evidence instead of feedback cache, preserving
existing target formula/thresholds. Do not add a short translation to recover a
failure. Do not change IK seeds, wrist yaw, waypoint heights, controller knobs,
registry, collision policy or documented hardware values.

## Baseline discrimination and checks

Establish meaningful failing baseline for pre-heading target reuse before source
change: extend existing action harness, change physical/localized bias at a normal
heading acceptance, call the actual existing aligned-precision path and assert
the required post-heading precision target. Capture exact mismatch. Native44
is independent failing evidence for caller/short-correction and zero-displacement
requirements. New helper tests cannot be claimed as a pre-fix failure merely
because its symbol is missing. Preserve baseline discriminator as evidence;
candidate tests must exercise actual new helper and actual main admission seam.
No assertions may be weakened for green.

Cover post-tangent and post-arrival bias changes, both tangent choices, initially
clear point, physical residual0/.009/exact.010/just>.010, no short correction,
one repair success/second miss, heading failure no repair; current TF missing,
stale/future/zero/bad quaternion/nonfinite; stale/invalid physical; stale READY or
odom and measured-axis threshold/drift; no authority side effects; active and
preacceptance cancellation/late UUID ownership; attachment loss; rollback and
shared budget exhaustion at stops/actions/terminal, including long dock after
valid clear proof. Existing A/non-centered/CA1 cases preserve behavior.

Retained packet outputs inside packet_b; env ROS_DOMAIN_ID=232,
PYTHONDONTWRITEBYTECODE=1,core0,owned HOME/ROS_HOME/TMP/cache/config/data/logs.
Reuse ONE validated existing G1 environment root:
/home/pete/amr_ws/phase14_evidence/stability_20261003/G1_OWNERSHIP_20261005/env.
Derive home/ros/tmp/cache from it, validate directories before subprocesses;
ROS_LOG_DIR=/home/pete/amr_ws/.ros_logs. Do not repeat manually typed date literals
for every variable or create unverified environment paths. Read-only git/hash
commands need only valid domain232 and must not rerun valid snapshots over typos.
Canonical storage_budget.check with>=30000000 reserve before growth; no disposal
or decode without separate packet. Source Humble+overlay without nounset.
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2, sequential scoped command:

```bash
colcon --log-base <packet_b>/build_logs build --packages-select amr_manipulation --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/amr_manipulation --output-on-failure -R '^gate6_pickup_retreat_behavior_test$'
python3 -B -m pytest -q src/amr_manipulation/test/test_moveit_config.py src/amr_manipulation/test/test_product_test_contract.py -o cache_dir=<packet_b>/pytest_cache
```

Run selected baseline before source edit and selected new behavior before full
focused suite. Root assigns ownership/completion integration suite once after
completed batch; no duplicate run, simulations or installs in this packet.

## Stops, counters and report

Every harness/build failure, contradiction, ambiguous contract, invalid baseline
discriminator or unapproved change returns to Root diagnosis before another
source edit. Numeric engineering caps removed; historical counters remain,
this stale-target candidate begins at0, CA1 historical candidate1 unchanged.
Report progress<=2min, exact commands/statuses, authoritative handles, changed
files, complete scoped delta against before snapshot, baseline/candidate evidence
paths and pins, observed results, blockers and uncertainty. After completed
checks HOLD for Root completed-batch review/separate milestone/clearance/native45.
Do not claim a source check proves runtime convergence.
