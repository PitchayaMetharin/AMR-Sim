# Product B behavior CTest timeout diagnosis

Root: metadata-verified gpt-6.1-sol/high. User released pending checks with
`continue working` and requested Luna/max test agents. This is a coordination
record; no implementation, timeout, assertion or safety gate is changed.

## Observed versus expected

The current-source build exited 0; centered/admission filter passed 9/9 in
17.887 wall seconds. Main-loop Python regression passed 1/1 and the two affected
Python contract files passed 63/63. All source/test/protected pins were retained.
Expected complete registered behavior coverage; actual full CTest exited 8 at
60.01 seconds: 61 cases passed, case 62 was interrupted, 13 were not started.
Thus 52 of the previously pending 66 passed; 14 remain unverified.

Evidence: `phase14_evidence/stability_20261003/b_turn_placement_20261006/packet_b/`
`resume_behavior_20261006_agent1/{case_inventory.txt,filter_centered9.log,full_behavior_ctest.log}`
and `resume_python_20261006_agent2/` command/status/log records.

## Failure mechanism and hypotheses

Classification: TEST/EVIDENCE HARNESS / ORCHESTRATION; aggregate-budget mechanism
SUPPORTED, not yet complete. Source CMakeLists.txt:97-99 supplies no explicit
TIMEOUT; installed Humble ament_add_test.cmake:75-76 defaults to 60 seconds.
The suite deliberately exercises 5-second no-feedback/stale-feedback/server
waits. Last two completed private-feedback cases took 10.241 and 10.239 seconds;
the interrupted missing-private-server case enters wait_for_action_server(5s).
The parent CTest killed this process while tests were still making progress.

Ranked alternatives: aggregate budget exhausted; interrupted case stalls;
unexpected cumulative setup/contention. Do not attach a debugger to a terminated
process or repeat the whole failing run merely to reproduce adequate evidence.

Discriminating check: same built source and registered CTest, same unchanged
60-second timeout, filter exactly the 14 unfinished cases. A missing-server case
stalls or asserts => aggregate-only diagnosis contradicted; stop and re-diagnose.
All 14 pass within their fresh registered budget => all 75 have individual-case
coverage across runs, while the unfiltered full CTest remains FAILED.

## Released check-only continuation

Same verified Luna/max worker, no edits, rebuild, retries, simulation or decode.
Owned G1 environment/domain 232, core0, -B/no-user-site, Humble then overlay;
fresh canonical storage_budget.check reserve30M before exclusive evidence growth.
Exact filter:
`Products/Gate6DispatchBehavior.MissingPrivateRouteDoesNotFallbackToPublic/*:CurrentTf/Gate6ClearApproachTfBehavior.*:Stages/Gate6ClearApproachStageBehavior.*`
Run this via GTEST_FILTER and GTEST_FAIL_FAST=1 environment with the original
`ctest --test-dir build/amr_manipulation --output-on-failure -R '^gate6_pickup_retreat_behavior_test$'`.
Capture raw test runner output/XML before another CTest overwrites it; preserve
the existing full timeout log. Report exact commands, statuses, all case names,
durations, source pins and authoritative handles. Stop first unexpected failure,
inventory mismatch, conflict or pin change; no source patch authorized.

Preserve handoff counters M180/Luna90 and all historical attempts. Faithful
execution of the full-test packet is not a Luna mistake. No counter reset.
The current all-model ledger header says 97 and ends in a truncated M095 entry,
contradicting the handoff; Luna's ledger retains its count90 and latest entries.
Preserve these existing files pending evidence-based reconciliation; do not
silently lower the effective handoff counters or reconstruct missing history.

## Completed disproof and approved implementation

The 14-case continuation exited 0 in 12.51 wall seconds; XML proves exactly14
cases, zero failures/errors, and no overlap with the earlier61. All75 inventory
entries now have passing individual-case evidence. Missing-server cases took
5.039/5.028 seconds. Unique-case durations total69.835 seconds before overhead:
61 completed cases57.509 seconds plus14 remainder cases12.326 seconds. This
supports aggregate-budget exhaustion and falsifies a hang in the interrupted case.
The original unfiltered CTest gate remains failed, not accepted by this union.

User explicitly approved the prepared90-second aggregate CTest timeout proposal
with `yea, approved`. RELEASE same verified Luna/max for exactly
`src/amr_manipulation/CMakeLists.txt`'s behavior ament_add_gtest TIMEOUT90 argument,
as in `resume_behavior_20261006_agent1/ctest_timeout90_proposal2.diff`.
No production/action/observation/clock budgets, tests, thresholds or assertions
change. Source CMake before SHA256:
`c41aa8b99f63b173e564eaa058e916087c0d462516481b4070b31090a1630ada`.
Recheck status/pins, retain before snapshot and canonical storage reserve30M;
use owned G1 environment/domain232/core0/-B and Humble then overlay. Apply only
the approved patch, inspect complete scoped diff and git diff --check, then the
canonical sequential jobs2 package build with explicit --cmake-force-configure.
Require generated CTest registration TIMEOUT90. Reconcile unfiltered inventory75;
unset GTEST_FILTER/GTEST_SHARD_INDEX/GTEST_TOTAL_SHARDS/GTEST_REPEAT before running
the original unfiltered registered CTest command once with GTEST_FAIL_FAST=1.
Expect75/75 pass below90 seconds with clean XML and original runtime safety tests.
Retain raw command/env/status/XML/logs in exclusive timeout90_approved_run1.
Stop first failed build/config/test or contradictory evidence; return to Root
before any further edit. No simulation/decode/integration suite released here.
Report exact diff, pins, commands/statuses, actual75-case results, durations,
authoritative handles and uncertainty. Root checks the completed packet; this is
not independent milestone review or full A/B/home simulation acceptance.

## Apply command correction

Approved apply stopped before reading the patch: worker typed20260606 in place
of20261006 in its evidence path. Exit128/no such file; original CMake/source pins
unchanged. This contradicts the packet's exact path, not the approved CMake change.
M181/Luna91 is recorded in both ledgers; preserve all earlier counters.
Root verified run1 contains CMakeLists.before and the copied validated proposal.
Use one bound runRoot (correct20261006) and `git apply "$runRoot/ctest_timeout90_proposal2.diff"`
after checking that copy SHA25633e783ce2e93653bd965014bb300732fca5341d77d057c0858a1cb1084542d19.
Capture the failed literal command/status in an exclusive run1 receipt, recheck
original hash/status/owned-process/storage preflight, then continue the same
approved90-second build/full75 packet. No production change was attempted by
the failed command; no user reapproval or counter reset is needed.

## Registration verifier correction

Approved patch applied and forced-configure build exited0. Actual CMake SHA
3a3f7585c2640b2f4a7f8094505ba2e136d9f19ed75804f5a55bd3d372ffdbf3;
production/test/protected pins unchanged. Worker then stopped on a malformed
property regex: a word boundary after the closing quote rejects the correct
TIMEOUT90 token followed by whitespace. Root's direct fresh scoped registration
read at build/amr_manipulation/CTestTestfile.cmake:12 proves the behavior test
has TIMEOUT90. M182/Luna92 recorded; no source defect or rebuild is indicated.
Retain the exact failed command/assertion/status in an exclusive receipt; use the
direct scoped property proof without that faulty regex. Re-establish owned env,
status/pins/storage/process preflight, then inventory and original unfiltered75
CTest once under existing approval. Same failure stops/recording obligations;
no additional source edit, patch application, rebuild or budget change.

## Completed timeout packet review

Full unfiltered CTest exited0 in70.06s (outer elapsed70.077s); XML reports75
completed cases, zero failures/errors/disabled/skipped, exact inventory equality,
case-time69.882s. Evidence retained in timeout90_approved_run1/full75_*.
An unnecessary final source locator expected a multiline registration on one
line and exited1; M185/Luna95 recorded. Root then completed the scoped review:
only approved CMake TIMEOUT90 diff; CMake SHA3a3f7585c2640b2f4a7f8094505ba2e136d9f19ed75804f5a55bd3d372ffdbf3;
production/test/protected pins exact, scoped git diff--check0. This packet passes
the current75-case gate and closes aggregate timeout diagnosis. Prior failed
60-second run remains retained. No rerun is indicated. Independent milestone
review, planned additional centered boundary coverage, scene recovery and full
native A/B/home acceptance remain pending; none are implied by this unit pass.
