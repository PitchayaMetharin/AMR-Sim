# Hospital exploration — debug ledger (2026-10-08)

Every run is a breadcrumb.  All runs use domain 231, spawn (-0.05, 7.55), live SLAM, run dirs `.ros_logs/hospital_explore_20261008_NN`.

| Run | Code state | Outcome | What it ruled in or out |
|---|---|---|---|
| 01 | base explorer | COMPLETE after 0 goals; 122 frontiers blocked | Spawn (-0.5, -22) had 0.40 m wall clearance, so the start was inside inflation. Bad spawn choice. |
| 02 | base | COMPLETE after 2 goals; 90 frontiers all BLOCKED_SAFETY | Ruled in: frontier-cell footprint overlaps unknown (cost 255). Offline replay (run 04) showed treating 255 as free unblocks 25 clusters. |
| 03 | base | 6 goals, then a 57x abort loop "collision boundary" near (1.16, 17.05) | Ruled in: elevator-bay hole in the north wall (front LiDAR inf -21..14 deg). World sealed afterwards. |
| 04 | sealed world | COMPLETE after 1 goal (68/69 BLOCKED_SAFETY) | Same as 02; capture.pkl replay reproduces it exactly. |
| 05 | + approach goals | 2 goals, then a 58-cycle smoother abort loop | Robot parked about 5 cm from a wall, so it could not turn. Ruled in: goals not leavable. |
| 06 | + turn gate + blockage limit | startup stall (controller change_state timeout) | Stale ros2 daemon or KILLed nodes. Not the explorer. |
| 07 | same as 06 | 15 goals, ~700 m², then FAULT: controller TF extrapolation (map->odom 60 ms late) | One-off TF race; open item, not reproduced since. |
| 08 | same | 4 goals, then INCOMPLETE (10 consecutive blockages) | Robot in OPEN space, start footprint clear; plan/abort reason not recorded. Different from 05/10/12. |
| 09 | same (GUI) | blockage loop | Recorded paths: unsmoothed planner path footprint overlaps ONLY cost 255 near approach goals, so Smac skips the full footprint check near unknown. Fix: inflate_around_unknown overlay. |
| 10 | + overlay (GUI) | 9 goals, 751 m², then an abort loop | Smoother: "collision at <robot pose>". Parked footprint touches a 254 cell at 0.729 m, inside the 0.735 m disk; turn-gate off-by-one (-x/-y ring unchecked). Fixed and tested. |
| 11 | tooling error | launcher `set -u` broke ROS setup | n/a |
| 12 | + gate fix + speed rewrite (GUI) | COMPLETE after 3 goals, 163 frontiers (83 safety / 80 route), no aborts | Capture 242 s later (map version 295 vs 258 at completion): robot footprint overlaps one 254 cell at robot-frame (-0.62, -0.43); nearest distance from the overlapping cell to the robot centre 0.7216 m (0.704 m was the nearest ANY >=254 cell, a different cell). Route proof fails closed on unsafe start, so 1037 safe endpoints become BLOCKED_ROUTE and the run COMPLETEs. Goal 3 world/tier NOT logged. SLAM optimisation warnings + map height 503->619 during goal 3. |

Facts:
- xy_goal_tolerance 0.07 m with stateful SimpleGoalChecker; yaw_goal_tolerance 0.15.
- Planner endpoint tolerance 0.05.
- Explorer goal yaw = 0 (w=1).
- NAVIGATION_FOOTPRINT +/-0.61 x +/-0.41, padded nav footprint 0.6x0.4 + 0.01.
- Circumscribed radius 0.735.

## Problem A — false COMPLETE when the robot pose is in collision
- Reproducible: deterministic. test_frontier_goal_clearance::test_unsafe_start_with_safe_frontiers_ends_incomplete_not_complete on the pre-fix code; ~6 s.
- Mechanism proven offline: clearing the single 254 cell yields candidates.
- NOT proven: that the same unsafe-start state existed at the run-12 completion decision (map version 258).

## Problem B — the robot ends up with its footprint touching a lethal cell after reaching a goal
- Not reliably reproduced. Seen in 05 (approach goal, pre-gate), 10 (gate bug, explained), 12 (unexplained).
- Ranked hypotheses for run 12:
  - B1 arrival offset: the goal passed the 0.735 disk, but the robot stopped up to 7 cm away and/or at a different yaw, so a corner touches.
  - B2 tier-1 frontier-cell goal: no disk gate; the yaw-0 footprint was clear, but the arrival yaw differs.
  - B3 new obstacle cell observed after arrival (SLAM map update; the 503->619 resize).
  - B4 localisation shift: a map->odom correction moved the estimated pose toward the wall without physical motion.
  - B5 costmap re-alignment on static-layer resize shifted cell indices/values.
- Changes made so far (unproven for B): goal_clearance_radius = 0.855 for approach/patrol goals (decision B: not for frontier cells); goal_world + goal-kind dispatch logging.
