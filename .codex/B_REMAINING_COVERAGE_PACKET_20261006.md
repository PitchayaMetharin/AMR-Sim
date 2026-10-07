# Remaining centered-B behavior coverage

Prepared by verified gpt-6.1-sol/high; RELEASE only after correction3 nine-case
filter passes. Sole writer: same metadata-verified gpt-6-luna/max. No redelegation.

Objective: exercise candidate2's unverified boundary contracts through the real
centered-dock/action harness. This is a test-coverage packet, not a production fix.
Canonical requirements: B_TURN_PLACEMENT_PACKET_B_20261006.md and its plan.
Existing stale-target baseline/native44 failure remain evidence; do not rerun them.

Allowed edits: src/amr_manipulation/test/test_gate6_pickup_retreat_behavior.cpp;
affected test_moveit_config.py/test_product_test_contract.py only when an existing
structural contract expresses superseded centered routing. No production/config
changes. Evidence only in packet_b. Preserve correction3, all unrelated work,
protected handoff, source candidate2, counters and thresholds.

Required tests and fixture decisions:
- Post-tangent bias change must change the actual clear translation target;
  preserve post-arrival target test. Assert independent physical-minus-localized
  arithmetic, endpoint/action counts, returned physical dock proof.
- Actual dock residuals0/.009/.010/just>.010m: success through threshold, reject
  larger residual, no short correction. Cover actual helper plus the admission
  seam called by main; structural contract must connect that seam to main's skip.
- Entry currentTF defects: stale .4s, future .1s, zero stamp, small nonunit
  quaternion, nonfinite position/quaternion. Clear old good TF before injection;
  require fresh valid physical/attachment prerequisites and no navigation.
  Physical cases: nonfinite position, invalid quaternion, stale/future receipt.
- Original-reference clear displacement and commanded translation bounds remain
  .15m; .151m achieved drift rejects/cancels accepted UUID and blocks later goals.
  A .151m clear translation can pass .155m registered admission but must not be
  sent. Arrival heading outside .15rad fails without a repair/dock.
- Shared ROS-time budget/rollback at initial stop, intermediate clear stage,
  acceptance, active long dock and post-dock observation. Use existing installed
  rcl ROS-time override API and coherent reseeds, not long wall sleeps. A valid
  clear proof cannot reset the original120s budget; 1ms feedback time cannot
  override it. Assert no subsequent goal; exact accepted UUID canceled when active.
- Observer requires fresh valid READY/nonzero boot+sequence/reason READY and fresh
  odometry; X/Y/angularZ each<=.01 continuously500ms. Test axes independently,
  invalid/stale evidence and interruption of settling. Use async helper plus
  observable harness events/timestamps: first action cannot precede500ms of
  restored valid stationary evidence. Keep500ms/200ms/8s unchanged. Existing
  locked authority/sequence equality proof remains; physical drift fails promptly.
- Delayed acceptance: add a bounded request-callback barrier/hook to the action
  harness and a retained optional Reentrant callback group enabled ONLY for these
  centered delayed tests (installed create_server.hpp has group parameter).
  Heartbeat must continue while acceptance is withheld, so stale physical evidence
  cannot masquerade as the intended cancellation/budget boundary. Defaults for
  existing A/CA1 tests unchanged. Observe requested UUID, cause cancellation or
  shared-budget failure before response, require helper failure/no later action,
  release response, require exact late UUID cancellation and terminal CANCELED.
  Barrier cleanup must release on assertion failure; never leave callback blocked.
  Active long-dock guard tests use HOLD behavior to avoid success-vs-cancel races.
  A terminal-success budget case must distinguish an already completed result from
  active cancellation; if the seam cannot establish that deterministically, report
  the ambiguity before editing rather than asserting an invented contract.

No arbitrary stress/retries, weaker assertions, gate changes or sleeps to force
failure. Fixture changes must preserve coherent pose/update locking and avoid
nested-lock deadlocks. Every newly observed failed check stops edits and returns
exact evidence to Root for diagnosis before another patch.

Before edits: git status, reread targets, retain unique before snapshot/source pins.
Canonical storage_budget.check reserve>=30M before growth; existing validated G1
env root/domain232/core0/-B; Humble then overlay without nounset; compiler jobs2.
Commands sequential: scoped colcon build as canonical B packet (unique log root),
new centered-boundary filter first, then existing complete centered+admission filter,
then full CTest gate6_pickup_retreat_behavior_test, then affected Python contracts.
Run git diff --check and inspect full scoped delta. Stop at first failure, retain
commands/statuses/authoritative handles/logs. No simulation/decoder/integration suite.
Report progress<=2min and a complete requirement->test->actual result matrix,
changed files, actual changes vs packet, before/after pins, remaining uncertainty.
HOLD for Root completed A+B batch review, then separate nonauthor milestone review.
