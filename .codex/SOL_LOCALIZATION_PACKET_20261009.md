# Bounded localization stop-and-resume implementation

Current user authorizes an autonomous engineering loop. Role: gpt-6.1-sol/medium sole production writer; gpt-6.1-sol/high diagnosis/orchestration and independent review. No other writer.

## Objective / diagnosis
Hospital runs 07/14: the controller aborts after map->odom stamp stalls. Supervisor currently converts every non-collision FollowPath failure to CONTROLLER_ABORT; explorer accepts no LOCALIZATION_UNAVAILABLE class and fault-latches. HIGH confidence in this failure path, strongly supported but unproven SLAM rebuild/CPU contention origin. Do not change SLAM settings.

## Evidence / baseline
SESSION_HANDOFF.md latest hospital section; .codex/TF_STALL_DIAG_20261009.md; .ros_logs/tf_replay_20261009/STATUS.md (E0b2 aborted, E1-E3 never run).
Fresh .ros_logs/sol_localization_20261009/baseline_exploration.log: 8 failures, 3 guard passes.
Fresh baseline_mission_behavior.log: stale localization expected class/reason fails; 3 healthy/collision/missing-evidence guards pass. Build exit 0.

## Allowed files / required behavior
- src/amr_mission/src/mission_supervisor_node.cpp: add FaultClass::LOCALIZATION_UNAVAILABLE and status name. On non-success FollowPath terminal result, preserve collision precedence. If newest map->odom stamp is older than newest odom->base_footprint stamp, classify LOCALIZATION_UNAVAILABLE with exact reason "path following lost map localization", outcome FAULT, blockage_confirmed false. Use existing buffer latest edge evidence; missing/invalid TF keeps generic fail-closed controller abort. Preserve cancellation/lifecycle precedence and terminal ownership proof.
- src/amr_exploration/scripts/frontier_explorer.py: accept the new class. Only UUID-correlated, fresh, valid TERMINAL FAULT record with exact reason above, blockage false, and aborted action authorizes localization RECOVERY_WAIT. Clear proven motion ownership, do not count blockage/failure or blacklist goal. Per-episode monotonic deadline 5.0 s; budget 3 consecutive episodes, reset by reached goal/start. Fourth -> FAULT class LOCALIZATION_UNAVAILABLE. Deadline -> FAULT with reason including "localization did not recover within 5.0 s". Expose localization_recoveries status. Require all existing readiness plus two independent stationary TF samples with strictly advancing map->base_footprint stamps and newest map->odom minus newest odom->base stamp >=0.5 s. Validate finite, positive stamps; no missing/stale evidence authorizes resume. Check deadline before/after evidence acquisition so blocking lookup cannot bypass expiry. Existing obstacle/planner recovery behavior unchanged. Honor run generation/operator stop/terminal token semantics. Constants exact: LOCALIZATION_RECOVERY_DEADLINE_S=5.0, LOCALIZATION_RECOVERY_LIMIT=3, LOCALIZATION_HEADROOM_MIN_S=0.5.
- src/amr_exploration/CMakeLists.txt: register existing test/test_frontier_localization_recovery.py for colcon.

## Invariants / non-goals
Fail-closed, existing collision precedence, thresholds, footprint/inflation, freshness, command ownership, cancellation and terminal guarantees. Do not change existing tests, manifests, SLAM config, Nav2 tolerances, hardware, unrelated source, protected AMR_CODEX_HANDOFF.md, or user dirty work. No dependencies/system/outside-workspace changes, no stage/commit/push. No simulation while writing.

## Falsifiable prediction / checks
Known-red pinned tests become green without weakening other classes. Short runtime SLAM stall reaches localization consumer, stops motion, recovers with advancing fresh TF then dispatches/reaches a new goal. Prolonged stall reaches expected safe deadline FAULT; no new motion. Tests at .codex/SOL_LOCALIZATION_PINS_20261009.sha256 must remain byte-exact.
Root validation commands (AMR_WS already defined, source /opt/ros/humble/setup.bash then $AMR_WS/install/setup.bash, ROS_DOMAIN_ID=232):
colcon build --packages-select amr_mission amr_exploration
colcon test --packages-select amr_mission amr_exploration
colcon test-result --test-result-base "$AMR_WS/build/amr_mission" --verbose
colcon test-result --test-result-base "$AMR_WS/build/amr_exploration" --verbose
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_exploration/test/test_frontier_localization_recovery.py

## Writer process / report
Before edits git status --short, read target files and preserve existing dirty work. Smallest coherent patch, inspect scoped diff. Do not run builds/tests/sims (root owns them). Run git diff --check on allowed scope and sha256sum -c pins. Report exact edits, checks/exit, assumptions, blockers then HOLD. A failed check or contradiction returns to diagnosis; do not improvise.
