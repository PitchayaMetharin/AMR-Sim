# Luna implementation ledger

This ledger records reviewed implementation mistakes under the current
Astra/high -> GPT-5.6/Luna/max policy.

Mistake count: 25

Each reviewed mistake records date/time, topic, observed failure, why it is a
mistake, likely cause, detection evidence, and the exact failure text when
available.

## Attempt 1

- Date/time: 2026-09-24, during the first diagnostics-package broad-test
  boundary; the exact command start time was not captured.
- Topic: optional `simulation_diagnostics` launch configuration compatibility.
- Mistake: The new diagnostics launch path reads `LaunchConfiguration("simulation_diagnostics")`
  unconditionally inside `_runtime_actions`, so direct callers/tests that do not
  construct the new launch argument raise `SubstitutionFailure`.
- Why it is a mistake: The packet required an opt-in launch argument while
  preserving existing launch/test seams; the default path must remain valid for
  callers that invoke the runtime expansion directly.
- Likely cause: Luna assumed every `_runtime_actions` call was preceded by
  `generate_launch_description()` declaring the new argument and did not check
  the existing direct-call tests before the broad build.
- Detection: `colcon test --packages-select amr_simulation` broad run; focused
  Python contract tests still passed.
- Exact failure: `SubstitutionFailure: launch configuration
  'simulation_diagnostics' does not exist` at
  `src/amr_simulation/launch/portable_exploration.launch.py:752` in the direct
  `_runtime_actions` test context.

## Attempt 2

- Date/time: 2026-09-24T20:33:28+07:00 (independent review boundary).
- Topic: end-to-end contact/coverage evidence collection and classification.
- Mistake: The diagnostics collector still subscribes only to the Explorer
  `DiagnosticArray`; it does not consume raw contacts or coverage heartbeats,
  classify exact wheel/caster ground support versus chassis/obstacle/unknown
  contact, or fail on missing diagnostic streams.
- Why it is a mistake: The user plan requires simulation evidence that can
  distinguish obstacle contact from normal ground contact and expose evidence
  loss before the controlled reproduction is treated as diagnostic. Topic
  declarations and a Gazebo publisher alone do not provide that proof.
- Likely cause: Luna implemented the topic/source contract and focused tests
  without tracing the collector’s end-to-end runtime ownership; the first
  diagnostics handoff also under-specified the collector’s contact/liveness
  stop semantics. This cause is an inference, not a model-capability claim.
- Detection: Astra/high independent review of the integrated diff; runtime was
  correctly blocked before launch.
- Exact review finding: collector has no contact subscription/classifier,
  coverage/liveness validation, or first-contact/evidence-loss stop path; tests
  do not exercise ground/obstacle classification or plugin lifecycle discovery.

## Attempt 3

- Date/time: 2026-09-25T01:18:29+07:00 (implementation checkpoint).
- Topic: scoped patch application.
- Mistake: Luna's first whole-file patch for the correction packet was rejected
  before changing production files because it combined delete/add operations on
  the same path; the implementation then had to be reapplied as sequential
  patches.
- Why it is a mistake: The approved packet required a bounded, reviewable
  change. A rejected patch wastes an implementation attempt's time and delays
  the required validation, even though no source corruption occurred.
- Likely cause: Patch composition error while replacing a large script in one
  operation; this is an implementation-process inference, not a runtime cause.
- Detection: Luna/max progress report; the apply operation was rejected before
  production modification, and the subsequent sequential patches completed.
- Exact failure: `apply_patch` rejected the combined delete/add whole-file
  patch (no additional tool error text was reported).

## Attempt 4

- Date/time: 2026-09-25T01:32:28+07:00 (Astra/high controlled-clock review).
- Topic: future-dated simulation-evidence liveness.
- Mistake: After motion admission, each far-future Contacts sample replaced
  `pending_received_at`, and `check_liveness(allow_pending=True)` treated that
  renewed pending receipt as healthy indefinitely even though no contact sample
  had been validated against `/clock`.
- Why it is a mistake: The packet permits a bounded wait for future samples to
  meet `/clock`, while preserving the existing one-second receipt bound. It
  must not allow an unbounded future-message flood to authorize continued
  motion or hide stale validated evidence.
- Likely cause: The implementation used pending-arrival freshness as a health
  signal after admission and refreshed that timestamp on every future sample;
  the focused test covered only one future sample followed by clock catch-up.
- Detection: Astra/high controlled-clock probe; at simulated seconds 1 through
  10, `/clock` and every other stream advanced while Contacts stayed 1000 s
  ahead, yet `issues(now=t, allow_pending=True)` remained empty.
- Exact failure: `issues(now=t, allow_pending=True)==()` for every t=1..10
  while the last clock-validated Contacts receipt remained at 0.0.

## Attempt 5

- Date/time: 2026-09-25, immediately before the Packet 6 implementation.
- Topic: mandatory model-family assignment.
- Mistake: The delegation selected `gpt-6-sol` for Sol/high diagnosis and
  `gpt-6-luna` for Luna/max implementation even though this workspace policy
  requires `gpt-5.6-sol` with high reasoning and `gpt-5.6-luna` with max
  reasoning. GPT-6 is explicitly prohibited for all workspace work.
- Why it is a mistake: Model-family selection is a hard workflow and policy
  invariant. Using the wrong family invalidates the delegated diagnosis and
  implementation path regardless of technical output.
- Likely cause: The delegation tool's available-model list was followed
  instead of the repository's exact model-family policy being verified before
  each delegation.
- Detection: User review identified the mismatch immediately. The active
  Luna agent was shut down before an accepted implementation result; no AWS
  completion claim is authorized from that attempt.
- Exact failure: `model: gpt-6-sol` and `model: gpt-6-luna` were selected for
  roles that require `gpt-5.6-sol` and `gpt-5.6-luna`.

## Attempt 6

- Date/time: 2026-09-25T21:16:34+07:00 (Packet 6 focused contract validation).
- Topic: swept-rotation regression fixture geometry.
- Mistake: The new endpoint-clear swept-rotation test placed its lethal cell at
  world coordinates `(0.425, 0.425)`, but Nav2's rasterized footprint checker
  classified that cell as colliding at one or both endpoint orientations.
- Why it is a mistake: The approved regression requires both endpoint poses to
  be clear and only the interpolated rotation to become lethal. The fixture
  violated its own independent precondition, so it could not validly test the
  strengthened between-pose check.
- Likely cause: The fixture was selected from continuous polygon geometry
  without first accounting for the costmap cell rasterization used by
  `FootprintCollisionChecker::footprintCostAtPose`.
- Detection: Focused `planner_replay_contract` CTest after the first Packet 6
  harness slice; the endpoint assertion failed before the swept-path assertion.
- Exact failure: `swept-rotation endpoint unexpectedly collides`.

## Attempt 7

- Date/time: 2026-09-25T21:17:51+07:00 (Packet 6 focused contract validation).
- Topic: swept-rotation regression fixture correction.
- Mistake: The bounded second fixture attempt moved the lethal cell to the
  opposite diagonal `(0.425, -0.425)`, but Nav2's rasterized footprint checker
  still classified an endpoint as colliding.
- Why it is a mistake: The correction still failed the mandatory endpoint-clear
  precondition, so the contract test remained unable to isolate swept rotation
  from endpoint collision.
- Likely cause: The second coordinate choice continued to rely on continuous
  rectangle geometry without an independent characterization of Nav2's cell
  rasterization and boundary handling.
- Detection: The second focused `planner_replay_contract` run after rebuilding
  the contract test; the endpoint assertion failed before the swept-path
  assertion.
- Exact failure: `swept-rotation endpoint unexpectedly collides`.

## Attempt 8

- Date/time: 2026-09-28T23:41:03+07:00 (Factory Docking Planner Regression packet).
- Topic: scoped patch application.
- Mistake: Luna's first combined patch for the five approved files was
  rejected during context verification before changing any file; the packet
  then had to be reapplied as smaller sequential patches.
- Why it is a mistake: The implementation process requires a bounded,
  reviewable patch. A rejected patch wastes an implementation iteration and
  requires an explicit ledger entry even when no source corruption occurs.
- Likely cause: The combined patch used a surrounding context block that did
  not exactly match the mission supervisor's current line layout.
- Detection: `apply_patch` returned a verification failure, and a follow-up
  status/diff check confirmed that no approved file had changed from that
  attempt.
- Exact failure: `apply_patch verification failed: Failed to find expected
  lines in /home/pete/amr_ws/src/amr_mission/src/mission_supervisor_node.cpp`.

## Attempt 9

- Date/time: 2026-09-29T (focused cycle-adapter validation; exact wall-clock
  time was not captured).
- Topic: wrapper-teardown regression fixture sequence.
- Mistake: The updated later-fault case reused status sequence 2 after the
  terminal empty status had already advanced the same boot to sequence 3.
- Why it is a mistake: The test intended to prove that a later same-boot fault
  clears terminal proof, but the stale sequence was correctly rejected by the
  production monotonic-sequence guard, so the test did not exercise that
  behavior.
- Likely cause: The full start -> loaded -> terminal sequence was added to an
  existing teardown test without updating the final fault sequence number.
- Detection: Focused cycle-adapter/MoveIt test run after the packet source
  patch; the teardown test failed while all other tests passed.
- Exact failure: `assert not adapter._child_terminal_empty_proof` failed because
  the sequence-2 fault was ignored after terminal sequence 3.

## Attempt 10

- Date/time: 2026-09-29T00:22:20+07:00 (handoff inspection).
- Topic: mass-stage launch event-handler ordering.
- Mistake: The launch factory returned the mass-stage `Node` before its
  `RegisterEventHandler(OnProcessExit(...))`, so an immediate child exit could
  occur before the nonzero-exit propagation handler was registered.
- Why it is a mistake: The approved packet requires the exit handler to be
  registered before starting the stage node, closing the loader-exit race while
  preserving exact nested failure propagation.
- Likely cause: The first implementation appended the handler after the node
  in the returned action list and did not include an action-order regression
  assertion.
- Detection: Handoff inspection after the packet's initial validation; the
  returned action order was `[stage, RegisterEventHandler(...)]`.
- Exact failure: `gate6_mass_stage.launch.py:_make_node returned the stage
  action before RegisterEventHandler(OnProcessExit(...))`.

## Attempt 11

- Date/time: 2026-09-29T00:23: (focused ordering-regression validation; exact
  wall-clock seconds were not captured).
- Topic: launch ordering regression assertion seam.
- Mistake: The new test assumed `OnProcessExit` exposes a public
  `target_action` attribute and failed before asserting the ordering contract.
- Why it is a mistake: The focused regression must validate the returned action
  ordering without depending on an unavailable private implementation detail of
  the launch event handler.
- Likely cause: The assertion was written against an intuitive public name
  without inspecting the installed Launch API.
- Detection: Focused cycle-adapter/MoveIt test run after the ordering fix; 64
  tests passed and the new ordering test raised `AttributeError`.
- Exact failure: `AttributeError: 'OnProcessExit' object has no attribute
  'target_action'` at `test_moveit_config.py:42`.

## Attempt 12

- Date/time: 2026-09-29 (endpoint-mismatch packet; exact wall-clock seconds
  were not captured).
- Topic: scoped patch application.
- Mistake: Luna's first endpoint-mismatch implementation patch targeted the
  same C++ file in two separate patch operations, so the patch tool rejected
  it before changing any file.
- Why it is a mistake: The implementation process requires a bounded,
  reviewable patch application; a rejected patch is an avoidable iteration
  failure even when the worktree remains unchanged.
- Likely cause: The patch was assembled with separate `Update File` entries
  for the constructor/navigation changes and the call-site/member changes,
  despite the patch tool requiring one operation per target file.
- Detection: `apply_patch` returned a verification error and a follow-up
  `rg` check confirmed that the C++ source still had no retreat endpoint,
  helper, or client member.
- Exact failure: `apply_patch verification failed: invalid patch: multiple
  operations target /home/pete/amr_ws/src/amr_manipulation/src/gate6_mass_stage.cpp`.

## Attempt 13

- Date/time: 2026-09-29 (endpoint-mismatch packet; exact wall-clock seconds
  were not captured).
- Topic: client-parameterized navigation cancellation ownership.
- Mistake: The first helper refactor changed readiness, late acceptance,
  submission, and result retrieval to the selected client but left all three
  active-goal cancellation calls passing the normal member client.
- Why it is a mistake: The approved packet requires the selected retreat or
  normal client to own every cancellation path, including requested,
  timeout, and stale-feedback cancellation; the initial patch could route a
  retreat goal's cancellation to the wrong action server.
- Likely cause: The patch replaced the member client in the obvious direct
  operations but did not include the three existing `cancel_navigation_goal`
  arguments in the replacement set.
- Detection: Focused `test_moveit_config.py` after the first source patch;
  the new endpoint contract failed its exact selected-client cancellation
  count assertion while the other 11 tests passed.
- Exact failure: `AssertionError: assert helper.count("cancel_navigation_goal(navigation_client, goal_handle, result)") == 3; assert 0 == 3`.

## Attempt 14

- Date/time: 2026-09-29 (fresh-AMCL admission packet; exact wall-clock
  seconds were not captured).
- Topic: scoped reversal of the retreat client and post-grasp call site.
- Mistake: Luna's combined follow-up patch for the direct fresh-AMCL call and
  retreat-client member removal was rejected before changing either hunk.
- Why it is a mistake: The implementation process requires bounded,
  reviewable patch application; an avoidable rejected patch interrupts the
  approved source transition even when no partial source change occurs.
- Likely cause: The member-context hunk did not match the current file context
  exactly after the preceding helper reversal, so the patch tool rejected the
  complete multi-hunk operation.
- Detection: `apply_patch` returned a verification failure; subsequent source
  inspection showed the retreat call and member still present, after which
  each was applied separately.
- Exact failure: `apply_patch verification failed: Failed to find expected
  lines in /home/pete/amr_ws/src/amr_manipulation/src/gate6_mass_stage.cpp`.

## Attempt 15

- Date/time: 2026-09-29 (fresh-AMCL admission packet; exact wall-clock
  seconds were not captured).
- Topic: replacing the prior retreat endpoint test contract.
- Mistake: The revised test body retained the old helper-specific assertions
  after correctly changing its endpoint and post-grasp proof assertions, so
  it still required the removed `navigate_to_with_client` helper.
- Why it is a mistake: The test must describe the revised normal-client
  navigation path and direct AMCL admission, not a deleted implementation
  detail; leaving the stale assertion made the focused validation fail after
  the production change was complete.
- Likely cause: The function body was patched in two hunks and the trailing
  helper-ownership block from the previous packet was not removed in the same
  edit.
- Detection: Focused `test_moveit_config.py` after the fresh-AMCL source/test
  patch; 11 tests passed and the revised test raised `ValueError` while
  searching for the removed helper.
- Exact failure: `ValueError: substring not found` at
  `helper_start = source.index("bool navigate_to_with_client(")`.

## Attempt 16

- Date/time: 2026-09-30 (focused pickup-retreat validation; exact wall-clock
  seconds were not captured).
- Topic: client-parameterized navigation cancellation test contract.
- Mistake: The focused test was updated to require the new shared navigation
  helper but retained the old callback-capture assertion `[this, pending]`;
  the implementation correctly captured the selected client as well.
- Why it is a mistake: The test contract must reflect the approved shared
  normal/retreat cancellation ownership, otherwise it rejects the intended
  implementation before validating the actual behavior.
- Likely cause: The source helper refactor and the test assertions were
  updated in separate hunks, and the callback-capture assertion was not
  updated with the helper signature.
- Detection: Focused `test_moveit_config.py` after the scoped source patch;
  10 tests passed and the new retreat contract failed at the stale callback
  capture assertion.
- Exact failure: `AssertionError: assert '[this, pending]' in navigation`.

## Attempt 17

- Date/time: 2026-09-30T18:03:36+07:00 (Sol/high independent review).
- Topic: required behavioral coverage for the factory pickup-retreat fix.
- Mistake: The implementation packet replaced the second bounded reverse with
  the registered retreat action, but omitted the required behavioral test
  executable and CMake registration. The remaining Python assertions inspect
  source text and do not exercise the real MassStageNode action clients.
- Why it is a mistake: The approved packet explicitly required controlled
  NavigateToPose servers, exact normal versus retreat endpoint routing,
  cancellation UUID ownership, terminal-result failures, feedback/admission
  fail-closed behavior, and a mutation that fails when clients are swapped.
  The implementation therefore lacked evidence that its runtime seam matches
  the intended endpoint ownership.
- Likely cause: Luna completed the production packet and updated the
  source-contract assertions but did not deliver the separately required
  behavioral test target.
- Detection: Sol/high review found
  src/amr_manipulation/test/test_gate6_pickup_retreat_behavior.cpp absent
  and no replacement behavioral target in
  src/amr_manipulation/CMakeLists.txt; an in-memory normal/retreat client
  swap still passed all three affected Python contract tests.
- Exact evidence: test_gate6_pickup_retreat_behavior.cpp did not exist;
  CMakeLists.txt registered no corresponding executable; the source-only
  tests passed after swapping the normal and retreat routes in memory.

## Attempt 18

- Date/time: 2026-09-30T18:11:26+07:00 (first focused behavioral test run).
- Topic: controlled action-server cancellation harness.
- Mistake: The initial test harness kept its hold loop inside the ROS action
  server accepted callback. Because that callback used the node's mutually
  exclusive callback group, the cancel service could not run while the goal
  was held.
- Why it is a mistake: The required cancellation test must exercise the
  selected client's accepted-goal cancellation and terminal CANCELED proof;
  blocking the server callback group makes the harness unable to provide that
  behavior and can leave the test process stuck during teardown.
- Likely cause: The harness modeled a long-running action directly in the
  accepted callback without accounting for ROS 2 callback-group scheduling.
- Detection: First focused run of
  gate6_pickup_retreat_behavior_test; cancellation response timed out,
  retreat_server canceled_count remained 0 instead of 1, and the interrupted
  process terminated with runtime_error:
  Asked to publish result for goal that does not exist.
- Exact failure: Navigation cancellation response timed out; expected
  retreat_server_->canceled_count() == 1U but observed 0.

## Attempt 19

- Date/time: 2026-09-30T18:13:41+07:00 (second focused behavioral test run).
- Topic: controlled NavigateToPose result and feedback harness.
- Mistake: The initial harness attempted to call goal->canceled from the
  EXECUTING state for a synthetic canceled-result case, and emitted only one
  feedback sample before a successful retreat result.
- Why it is a mistake: ROS 2 permits CANCELED only after an accepted cancel
  transition, and the route test must receive valid feedback before terminal
  success. The harness therefore both aborted the test process and produced a
  false negative for endpoint routing.
- Likely cause: The harness modeled terminal result codes without preserving
  the action state machine and assumed one immediate feedback publication was
  sufficient.
- Detection: Second focused run; route navigation returned false with no
  feedback, and the process terminated with RCLError:
  goal_handle attempted invalid transition from state EXECUTING with event
  CANCELED.
- Exact failure: Gate6PickupRetreatBehavior.NormalAndRegisteredRetreat...
  expected true but got false; RCLError from goal_handle.c:95.

## Attempt 20

- Date/time: 2026-09-30T18:16:04+07:00 (third focused behavioral test run).
- Topic: diagnostic malformed-feedback coverage.
- Mistake: The malformed-feedback harness published only one sample before
  entering its hold loop, and the passing test did not assert that the node
  received and classified that sample as invalid.
- Why it is a mistake: A test named for malformed feedback must distinguish
  invalid feedback from the separate no-feedback watchdog; otherwise it can
  pass while exercising the wrong failure branch.
- Likely cause: The harness was optimized for the stale-feedback timing case
  and reused its one-shot publication for malformed feedback without adding
  receipt-state assertions.
- Detection: The third focused run passed, but its log reported
  no navigation feedback within 5 wall seconds for the malformed case.
- Exact evidence: MalformedFeedbackDeniesRetreat returned OK only through the
  no-feedback cancellation path; the node's received/invalid flags were not
  asserted.

## Attempt 21

- Date/time: 2026-09-30T18:33:17+07:00 (final correction packet focused pytest).
- Topic: production admission-helper placement versus the existing source-order
  contract.
- Mistake: The final correction moved the complete pickup-station admission
  proof into a helper defined after `main`. This preserved the runtime call
  order but moved the textual downstream dispatch sequence before the helper
  definition, so the existing focused source-contract test could not find the
  expected bearing calculation after the proof markers.
- Why it is a mistake: The implementation must preserve both the runtime safety
  order and the repository's existing focused validation contract. The packet's
  mandatory pytest therefore failed before the package build and behavioral
  validation gates.
- Likely cause: The helper was placed after `main` to keep the legacy source
  markers after the retreat call, without checking the complete set of
  downstream source-order assertions.
- Detection: Focused command
  `env ROS_DOMAIN_ID=223 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p
  no:cacheprovider src/amr_manipulation/test/test_moveit_config.py`.
- Exact failure: `ValueError: substring not found` at
  `src/amr_manipulation/test/test_moveit_config.py:302` while searching for
  `dispatch_translation_heading = std::atan2` after
  `dispatch_translation_start`.
- Additional environment blocker observed in the same command: the launch
  test could not create `/home/pete/.ros/log/...` because that path is
  read-only (`OSError: [Errno 30] Read-only file system`).

## Attempt 22

- Date/time: 2026-09-30T18:59:24+07:00 (final correction packet focused pytest).
- Topic: caller guard/throw contract assertions.
- Mistake: The correction applied `_assert_guarded_throw_block` to the existing
  pickup caller slice, which starts at the retreat call expression rather than
  the preceding `if` guard. The new assertion therefore searched for a guard
  that was not present in the slice.
- Why it is a mistake: The mandatory focused pytest gate failed on the newly
  added test contract before the requested mutation checks and remaining
  validation gates could run.
- Likely cause: The assertion was added without accounting for the existing
  `pickup_retreat` anchor used to construct `pickup_caller`.
- Detection: Required pytest command exited 1 with 21 passed and 2 failed in
  the two affected source-contract tests.
- Exact failure: `ValueError: substring not found` in
  `_assert_guarded_throw_block` while searching for
  `if (!node->navigate_to_registered_retreat(product.pickup_station, 120s))`
  in a slice beginning with `node->navigate_to_registered_retreat(...)`.

## Attempt 23

- Date/time: 2026-09-30T21:57:34+07:00 (fresh Product 101 runtime validation).
- Topic: mass-stage status ownership handoff.
- Mistake: The implementation required the one-shot `Gate 6 mass stage is
  starting` status sample to establish the new child boot. If that sample was
  not delivered to the supervisor, the first valid MOVING or loaded-stow
  sample from the new mass-stage boot was treated as an unrelated preparation
  publisher and discarded.
- Why it is a mistake: The runtime handoff must remain live when a valid child
  status publisher starts between status samples. The source change preserved
  the late-boot rejection rule but did not provide a safe recovery path for a
  missed first marker, causing a valid loaded-stow state to disappear from the
  public authority status and blocking dock egress.
- Likely cause: The marker was published once with volatile QoS and the
  implementation assumed subscriber delivery rather than reasoning about the
  preparation-node teardown and mass-stage publisher transition.
- Detection: Run
  `factory_pickup_validation_20260930_05` recorded preparation boot
  `898293414`, mass-stage boot `420139072`, a single start marker, subsequent
  MOVING/loaded messages, and a public status that remained stale/inconsistent.
  The command-arbitration gate then rejected dock egress.
- Exact failure: `Dock egress goal rejected: fresh loaded-stow, READY base,
  filtered odometry, and idle Nav2 evidence are required`; the public status
  remained `state=STOWED_EMPTY product_attached=false detail="internal
  manipulation status is stale or inconsistent"` while internal status
  reported `state=STOWED_LOADED product_attached=true product_id=101`.

## Attempt 24

- Date/time: 2026-09-30T22:20:31+07:00 (fresh Product 101 runtime validation).
- Topic: dispatch placement alignment segment heading.
- Mistake: The implementation held the registered dispatch heading during
  translation segments but left the segment waypoints anchored to the initial
  dock pose. When an earlier navigation action returned with the robot behind
  its nominal waypoint, the next fixed waypoint was more than the allowed
  0.15 m from the actually achieved pose.
- Why it is a mistake: Every bounded alignment command must be bounded from
  the fresh achieved pose, not only from the planned interpolation. The source
  correctly rejected the over-length physical segment, but the implementation
  allowed the command that caused it to be issued.
- Likely cause: The heading correction was changed without closing the
  independent stale/incomplete waypoint-progress boundary exposed by run06.
- Detection: Run
  `factory_pickup_validation_20260930_07` passed all host/runtime/graph/
  lifecycle/MoveIt/mode gates, then logged alignment waypoints through
  `(-3.600, -0.039, 3.142)` and failed after the achieved pose moved from the
  prior fresh pose by `0.229 m`.
- Exact failure: `GATE 6 1.0 KG: FAIL: achieved placement alignment segment
  exceeded 0.15 m`.

## Attempt 25

- Date/time: 2026-09-30T22:31:00+07:00 (focused validation after the closed-loop alignment patch).
- Topic: closed-loop placement alignment source contract.
- Mistake: The implementation changed the segment heading from an assignment to
  a braced initializer but left the focused source test asserting the removed
  assignment spelling.
- Why it is a mistake: The validation contract must describe the actual safety
  invariant and syntax of the bounded closed-loop target, otherwise the focused
  gate fails before runtime evidence can be collected.
- Likely cause: The source contract was updated for the new loop structure but
  not for the initializer form used in the final patch.
- Detection: The focused command ran 23 tests and failed one assertion in
  `test_moveit_launch_sets_factory_model_and_publishes_descriptions`; the C++
  build continued independently.
- Exact failure: `AssertionError: assert 'segment_target[2] = dispatch_yaw'
  in mass_source`.
