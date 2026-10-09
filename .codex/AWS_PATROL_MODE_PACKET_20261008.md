# Implementation packet: opt-in continuous exploration (patrol) mode

## Objective
User request (2026-10-08): "I want the robot to not stop and keep exploring until user manually
command it." User chose PATROL: never finish on its own; explore reachable frontiers first; when
none are reachable, keep driving to free, reachable map spots (a different one each time); stop only
on `/amr/exploration/stop` or Ctrl+C. Must be OPT-IN (`continuous_exploration`, default false);
default behaviour and the AWS acceptance runner stay exactly as they are.

## Diagnosis / context (confidence high)
AWS run `.ros_logs/aws_warehouse_runner_20261008_06` reached COMPLETE after ~2 min: LiDARs mapped
almost the whole warehouse from near the start; all remaining frontiers (178) are BLOCKED_SAFETY
shelf aisles. Completion happens in `FrontierExplorer._select_frontier` (scripts/frontier_explorer.py,
the `if not candidates:` branch ~L1700-1745): after `no_frontier_limit` planning cycles without a
candidate it calls `_finish_without_goal(..., complete=/incomplete=)` -> `_terminal_locked`.

## Worktree state
Many unrelated uncommitted changes exist (earlier session). Run `git status --short` first. Do not
touch, revert, or reformat anything outside the allowed files. Do not commit.

## PINNED acceptance tests (root-authored; DO NOT EDIT)
Verify before and after your work: `sha256sum -c /tmp/claude-1000/-home-pete-amr-ws/f1023d1b-3c65-4de9-9480-4b85ec01d7be/scratchpad/patrol_pins.sha256`
(run from `/home/pete/amr_ws`). Pinned: `src/amr_exploration/test/test_frontier_patrol.py`,
`test_frontier_algorithm.py`, `test_frontier_contract.py`, `src/amr_exploration/CMakeLists.txt`,
`src/amr_simulation/test/test_aws_warehouse_exploration.py`,
`src/amr_simulation/test/test_portable_exploration_launch.py`.
Read `test_frontier_patrol.py` fully; it is the behavioural spec. If you believe a pinned test is
wrong or contradicts this packet, STOP and report it with evidence. Do not work around it.

## Allowed files
1. `src/amr_exploration/scripts/frontier_algorithm.py` — add `patrol_candidates`.
2. `src/amr_exploration/scripts/frontier_explorer.py` — parameter, constants, patrol logic, status keys.
3. `src/amr_exploration/config/frontier_explorer.yaml` — add `continuous_exploration: false`.
4. `src/amr_exploration/test/test_frontier_lifecycle.py` — ONLY add default values for any NEW node
   attributes inside the `_node()` fixture (e.g. `node.continuous_exploration = False`). No other edit.
5. `src/amr_simulation/launch/portable_exploration.launch.py` — launch argument + explorer parameter.
6. `src/amr_simulation/launch/aws_warehouse_exploration.launch.py` — declare + forward.
7. `docs/SIMULATION_COMMANDS.md` — short usage subsection.

## Required changes

### A. `frontier_algorithm.patrol_candidates`
Signature (exact):
`patrol_candidates(grid, costmap, robot_world, robot_yaw=0.0, footprint=None, min_goal_distance=0.0, sample_spacing=1.0, avoid_worlds=(), avoid_radius=0.5, visited_worlds=())`
- Returns a list of map cells `(x, y)` best-first; `[]` when nothing qualifies; `None` on invalid
  input: invalid map geometry or data length (`_grid_geometry(..., allow_identity=True)`), invalid
  costmap geometry or data length (`costmap_geometry`), non-finite robot pose or yaw, invalid
  footprint when given (`_strict_footprint`), `sample_spacing` non-finite or <= 0,
  `min_goal_distance` or `avoid_radius` non-finite or < 0. Follow the module's existing
  validation/try-except style (catch AttributeError/TypeError/ValueError/OverflowError -> None).
- Sampling: `stride = max(1, round(sample_spacing / map_resolution))`; consider only cells with
  `x % stride == 0 and y % stride == 0`.
- A sampled cell qualifies only if ALL hold: map value exactly 0; `_endpoint_cost` is not None and
  < 253; when footprint is given, `_footprint_costmap_clear(costmap_geometry, data, cell_centre,
  footprint)` (yaw 0) is True; Euclidean distance from robot >= `min_goal_distance`; distance to
  every avoid world > `avoid_radius` (excluded when <=); and the cell is reachable via
  `_reachable_costmap_cells(costmap_geometry, data, robot_world, robot_yaw, target_cells,
  footprint, stop_after_first=False)` — the same heading-aware route proof frontier selection uses.
- Order: score = min Euclidean distance from the cell centre to `robot_world` and to every
  `visited_worlds` entry; sort by score descending, ties by `(y, x)` ascending. Deterministic.

### B. Explorer (`frontier_explorer.py`)
- Module constants: `PATROL_SAMPLE_SPACING_M = 1.0`, `PATROL_MIN_GOAL_DISTANCE_M = 1.5`,
  `PATROL_AVOID_RADIUS_M = 0.5`, `PATROL_HISTORY_LENGTH = 4`. Import `patrol_candidates` into the
  module namespace from `frontier_algorithm` (tests monkeypatch module-level names).
- `self.declare_parameter("continuous_exploration", False)` (exact text), validated with
  `_parameter_bool` alongside the other parameters BEFORE any ROS entity is created; store as
  `self.continuous_exploration`.
- New state: `patrol_active` flag (bool) and a bounded history of dispatched patrol goal worlds
  (last `PATROL_HISTORY_LENGTH`). Status message gains keys `"continuous_exploration"` and
  `"patrol_active"` (bools; the existing serializer lowercases them).
- Trigger: in the no-candidate branch, keep everything that happens today (evidence re-check,
  frontier diagnostics application, `no_frontier_updates_seen` counting). Then, ONLY when
  `self.continuous_exploration` is True and (`limit_reached` or `patrol_active`), do NOT produce
  COMPLETE/INCOMPLETE; attempt a patrol goal instead. This applies to all three would-be terminal
  outcomes (no frontier remains, blocked frontiers remain, unresolved classification). When
  continuous mode is off, behaviour must be byte-for-byte the same as today.
- Patrol inputs: the same map snapshot and costmap snapshot used for this plan; `robot_world`,
  `robot_yaw` from the same transform used for frontier selection; `footprint=NAVIGATION_FOOTPRINT`;
  `min_goal_distance=max(self.min_goal_distance, PATROL_MIN_GOAL_DISTANCE_M)`;
  `sample_spacing=PATROL_SAMPLE_SPACING_M`; `avoid_worlds` = the same `failed_goal_worlds` tuple
  already built in `_select_frontier` (run failed goal worlds + blocked worlds for the current route
  fingerprint); `avoid_radius=PATROL_AVOID_RADIUS_M`; `visited_worlds` = patrol history.
- Result `None` -> fail closed exactly like a `None` frontier selection result
  (`_planning_gate_failure(run_generation, started_at, costmap_detail)`). Result `[]` ->
  `_finish_without_goal(run_generation, "continuous exploration: no reachable patrol goal; waiting for a map update")`
  (non-terminal SCANNING; exact reason text). Non-empty -> dispatch the first cell through EXACTLY
  the existing frontier dispatch path (goal_world = cell centre via `frontier_cell_world`, TF
  refresh / `_route_start_proof`, action-server readiness, costmap readiness, `_reserve_and_send`).
  Do not duplicate that dispatch code — restructure minimally so the patrol cell flows into the
  existing `gx, gy = candidates[0]` path.
- On a successful patrol reservation/dispatch: `patrol_active = True`, append goal world to history.
  On any frontier goal dispatch: `patrol_active = False`. Once `patrol_active` is True, a
  no-frontier planning cycle patrols immediately (no new `no_frontier_limit` wait).
- `_start_callback` resets `patrol_active = False` and clears the history. Stop needs no special case.
- Patrol goals are ordinary motion: success increments `reached_goal_count`; mission blockage uses
  the existing recovery/blocked-destination memory; other failures use `goal_failures` and the
  existing FAULT limit. No special-casing, no new retry loops.
- In continuous mode exhaustion never yields COMPLETE/INCOMPLETE. FAULT/STOPPED unchanged. The
  "waiting for a mapped free cell" path is unchanged.

### C. Launch + config
- portable_exploration.launch.py: `DeclareLaunchArgument("continuous_exploration", default_value="false", choices=["true", "false"]),`
  (exact text, next to `auto_start_exploration`), and add to the explorer `parameters` list exactly
  `{"continuous_exploration": ParameterValue(LaunchConfiguration("continuous_exploration"), value_type=bool)},`
- aws_warehouse_exploration.launch.py: declare `continuous_exploration` (default "false", choices
  ["true","false"]) as the LAST DeclareLaunchArgument (after `simulation_diagnostics`) and forward
  `"continuous_exploration": LaunchConfiguration("continuous_exploration")` in the include arguments.
  No other launch file needs changes (portable default is false).
- frontier_explorer.yaml: `continuous_exploration: false`.

### D. Docs
`docs/SIMULATION_COMMANDS.md`: short subsection after "Canonical AWS warehouse exploration" —
"Continuous exploration (patrol) mode": the launch command with `continuous_exploration:=true`
(plus headless/software_rendering/rviz/auto_start like the canonical command), stop with
`ros2 service call /amr/exploration/stop std_srvs/srv/Trigger` or Ctrl+C, and a note that this mode
never reaches COMPLETE, so it is not used with the bounded evidence runner and is not AWS
acceptance evidence. Any example `ROS_DOMAIN_ID` must be within 0-232.

## Invariants (do not change)
Default-mode behaviour; costmap thresholds (253/254), footprint, inflation, `MAX_BLOCKAGE_ATTEMPTS`,
`max_goal_failures`, timeouts, freshness gates; goals only via the mission NavigateToPose action;
no cmd_vel/Twist; all fail-closed paths; evidence runner/monitor scripts untouched.

## Non-goals
No runner changes, no new services, no tuning, no changes to other packages, no simulations.

## Falsifiable prediction
Before: 13 explorer tests in `test_frontier_patrol.py` fail (19 algorithm tests error at import
until `patrol_candidates` exists), plus the 2 launch tests fail. After: all pass, and every
pre-existing test in amr_exploration and amr_simulation still passes (previous totals:
amr_exploration 205 with 2 skipped; amr_simulation 157).

## Validation (run from your own shell; report exact exit statuses)
```bash
export AMR_WS=/home/pete/amr_ws
cd "$AMR_WS"
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=232
sha256sum -c /tmp/claude-1000/-home-pete-amr-ws/f1023d1b-3c65-4de9-9480-4b85ec01d7be/scratchpad/patrol_pins.sha256
colcon build --packages-select amr_exploration amr_simulation
colcon test --packages-select amr_exploration amr_simulation
colcon test-result --test-result-base "$AMR_WS/build/amr_exploration" --verbose
colcon test-result --test-result-base "$AMR_WS/build/amr_simulation" --verbose
```
Before that, iterate quickly with
`PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_exploration/test/test_frontier_patrol.py`.

## Stop conditions
A pinned test appears wrong; a pre-existing test fails and the cause is not obviously your change;
scope needs a file outside the allowed list; hash check fails. Stop and report instead of improvising.

## Report
Changed files; summary of actual changes; commands with exit statuses; test totals; the pin check
output; any deviation from this packet; remaining risks (e.g. patrol CPU cost on the AWS map).
