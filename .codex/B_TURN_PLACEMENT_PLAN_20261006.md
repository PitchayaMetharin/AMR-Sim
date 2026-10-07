# Fix B pickup turning and placement alignment

Recorded from the user's supplied plan; implementation authorized on 2026-10-06.
Latest user override: remove numeric engineering retry limits. Historical
attempt/hypothesis counters remain evidence; AGENTS.md and DEBUG_PLAYBOOK.md
retain diagnosis, freshness, ownership, safety, runtime and review requirements.

## Objective and causal evidence

Complete Product A, Product B and return-home under existing acceptance gates.
Measure path-turn settling and physical B placement convergence in a fresh
native45 run; performance tuning remains a later task.

Pickup: HeadingLatchedRPP enters only above 90 degrees, while upstream starts
turning above the unchanged 0.785 rad threshold. Recorded moderate-turn input
enters near -0.816 rad and resumes forward motion near -0.783 rad with measured
angular speed -0.323 rad/s. Completing only the initial 180-degree turn leaves
this intermediate handoff unresolved.

Placement: native44 captured physical/localized XY bias before a heading
maneuver; observed bias then changed approximately 32 mm X and 24 mm Y. The
precision target retained the earlier correction. Localized arrival passed,
but physical stance residual was 48 mm. Its short correction turned sharply
and stalled at about 17.4 mm lateral error. Recorded carrot distance was
31.48 mm when angular command became zero. Upstream Nav2 1.1.20 sets curvature
zero when squared carrot distance is at most 0.001 m^2 (31.62 mm), preventing
that short correction from removing the residual under the 10 mm XY gate.
Source: https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp#L298-L329

Preserve native44 as failed evidence. Its result, factory.log and recorder
remain under phase14_evidence/factory_full_validation_20261003_44. Planning
checks under stability_20261003/plan44_input_check_20261005 and
plan44_placement_point_check_20261005 retain deletion receipts. The placement
check emitted four useful samples but exited on EOF handling; it is causal
input evidence, not a completed stability report. No production files changed
during the previous planning phase.

## Packet A: complete intermediate turns

Allowed production: src/amr_mpc_controller/src/heading_latched_rpp.cpp;
existing test: src/amr_mpc_controller/test/test_heading_latched_rpp.cpp;
header comment only if needed: include/amr_mpc_controller/heading_latched_rpp.hpp.

Replace the hard-coded 90-degree entry boundary with inherited
rotate_to_heading_min_angle_, preserving upstream's strict greater-than entry.
Keep all parameters unchanged. Preserve these existing phases:

1. STOPPING issues a complete zero command; waits for measured linear speed
   at most 0.01 m/s.
2. ROTATING holds linear command zero until carrot bearing is within active
   goal yaw tolerance.
3. SETTLING commands zero until measured linear speed is at most 0.01 m/s and
   angular speed at most 0.03 rad/s. Bearing drift outside yaw tolerance
   returns to ROTATING. Tracking resumes only after this proof.

Preserve terminal-heading delegation, plan/lifecycle resets, lock order,
PlanRestore, finite-input checks, TF failure handling and collision checking.
Update the old 1.0 rad/90-degree delegation expectation: these now latch.
Ordinary tracking at or below the upstream threshold still delegates.
Prediction: the recorded moderate-turn sequence cannot translate while
measured angular velocity remains -0.323 rad/s.

## Packet B: centered Product 102 placement choreography

Allowed production: src/amr_manipulation/src/gate6_mass_stage.cpp; existing
behavior and affected product/placement contract tests only. Product A and
non-centered-slot paths retain their behavior.

Use an internal helper directly exercised by behavior tests:

1. Reuse immutable final_placement_stance.physical.
2. Derive the clear point from the line through that stance along its physical
   heading; current-scene point approximately (-2.500, 0.100) m.
3. Use the existing clear-approach endpoint and owned heading/translation
   stages, choosing the closest forward/reverse tangent for lateral travel.
4. Observe measured stopping, then capture physical pose and current
   map -> base_footprint TF.
5. Send an owned normal heading goal at current localized XY: stance physical
   arrival heading corrected by freshly measured yaw bias.
6. Observe stopping again.
7. Capture physical/current-TF again; only then compute dock XY/yaw correction.
8. Call existing dispatch precision endpoint directly. Passing the target
   through navigate_to_aligned_precision would allow another heading after
   target construction and recreate the stale-target boundary.

Keep 0.03 rad goal margin for the separate final-heading stage. The margin
must no longer tilt the clear-approach line. Preserve curvature steering on
the long precision approach; add no ballistic-ray prerequisite or tighter
heading tolerance.

Extract/reuse CA1 TF validation: canonical positive stamp, age 0..0.30
simulation seconds, finite pose, valid unit quaternion. Post-heading capture
must not use feedback's five-second receipt allowance. Preserve feedback
monitoring while actions execute. The stopping observer only observes: no
wait_for_motion_permission call, manipulator-state publication or authority
change. Require fresh READY/base and odometry receipts within 200 ms and
measured linear X/Y and angular Z each at most 0.01 continuously for 500 ms.
Observer bound: eight wall seconds and remaining sequence simulation budget.

After arrival heading/settling require physical clear-point XY at most 0.01 m
and physical heading error at most 0.15 rad. If only position proof fails,
allow one bounded clear-area repair followed by arrival heading, stopping and
fresh capture. A further position miss fails; heading or safety failures
never authorize another repair. Share one 120-second simulation budget and
the original physical clear-area reference across both attempts. Preserve
clock rollback, cancellation ownership, attachment proof, 0.15 m clear-area
displacement bound and 0.155 m registered-approach envelope.

Coordinator scope clarification from actual geometry: the 0.15 m bound and
registered-approach envelope cover clear-area heading/translation/repair and
the final pre-dock capture. The long dock leg is approximately 0.845 m and
keeps its existing travel safety/arrival gates. The shared sequence budget
includes docking; do not apply the clear-area radius to that dock traversal.
Stage targets and budget checks must not silently reuse a pre-heading bias.

After dock travel observe stopping and capture physical/current TF. Physical
stance XY at most 0.01 m skips all centered-B placement translation, including
exact zero displacement, and continues separate final-heading/placement
proofs. Above 0.01 m report residual and fail closed before a short correction.
Place this branch before positive-distance/positive-segment rejection while
preserving finite geometry, attachment, cancellation and displacement checks.

Preserve endpoints, registry geometry, arm motions, controller profiles,
collision policy, thresholds and motion caps.

## Verification and native45 acceptance

Before production changes, establish meaningful failing baseline regressions
for moderate turns, post-heading target correction and centered-B skip/reject.
Cover both turn directions, threshold boundary, settling drift, plan/lifecycle
reset, ordinary/terminal delegation, invalid input/TF and collision rejection.
Exercise heading-induced bias change, stale/future TF, stale physical evidence,
cancellation, attachment loss, clock rollback, budget expiry, one repair and
second-miss rejection. Test centered-B residuals 0, 0.009 m and just above
0.01 m without short corrections; cover equality at 0.01 m as a boundary.
Keep existing A/non-centered behavior covered.

Scoped builds: amr_mpc_controller then amr_manipulation, sequential execution,
two compiler jobs, workspace-owned environment/log/temp/cache/home paths,
valid ROS_DOMAIN_ID (use 232). Verify source/build/install parity before run.

Required focused commands:

```bash
ctest --test-dir build/amr_mpc_controller --output-on-failure -R '^(heading_latched_rpp_test|final_position_rpp_test)$'
ctest --test-dir build/amr_manipulation --output-on-failure -R '^gate6_pickup_retreat_behavior_test$'
```

Run affected product contracts and existing ownership/completion suites once
at integration. Reuse the existing planner replay seam with captured scene
geometry and full footprint sweeps for new clear-area turns, translation and
arrival corridor. Generic fixture or Navfn fallback success is not clearance
proof. Tests/build success is not full-run acceptance.

Prepare a fresh native45 identity with the existing strict runner. Require
startup/admission, Product A, B pickup/placement/terminal proofs, return-home,
both strict acceptance analyzers, recorder finalization, lifecycle integrity
and owned teardown. Stop the run at its first failed mandatory gate, preserve
evidence and return to Sol diagnosis before another edit or simulation.

Every simulation stability report is at most 60 lines: available per-phase
XY/yaw error, overshoot, command/measured reversals, settling/convergence,
roll/pitch, missing data, units and simulation versus wall time. Distinguish
intentional maneuver changes from oscillation; compare against same-scenario
baseline. Faster operation remains a later tuning goal.

## Roles, storage, accounting and progress

Verify actual metadata exact gpt-6-luna/max before implementation release.
Luna owns source, tests and diagnostic programs; exact gpt-6.1-sol/high owns
planning, diagnosis, supervision and completed-batch review. One production
writer; a separate non-author milestone reviewer. Author verification is not
independent review. Runtime-timeout/startup review timing exception remains.
Luna reports changed files, complete scoped diff, commands/statuses, evidence,
blockers and remaining uncertainty; ambiguity returns to Sol before edits.

All task artifacts stay inside /home/pete/amr_ws. Preserve unrelated work and
AMR_CODEX_HANDOFF.md. Preserve retained counters, attribution rules, storage
caps/reserves and disposal receipts. Numeric engineering retry caps are removed
by the latest user instruction; faithful execution of a failed plan is not
automatically a Luna mistake. Update SESSION_HANDOFF.md after successful tasks.

Coordinator metadata independently read: session 01a10d3b-4908-7d60-acc0-9877f3c516b4,
latest turn_context 2026-10-05T18:06:13.600Z, gpt-6.1-sol/high. Candidate writer
metadata independently read: session 01a10d3f-6045-7ba3-9488-b7d643b3310d,
turn_context 2026-10-05T18:07:01.189Z, gpt-6-luna/max. Neither nickname nor spawn
response was used as identity proof.
