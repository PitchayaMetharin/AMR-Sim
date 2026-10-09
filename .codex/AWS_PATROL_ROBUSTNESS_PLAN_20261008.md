# Plan: make patrol exploration robust on any map (stopping spots, planning speed, driving clearance)

## Context
Opt-in patrol mode (`continuous_exploration:=true`) was added today and run in the AWS GUI sim
(`.ros_logs/aws_warehouse_patrol_20261008_01`). Three problems showed up, and the user wants the
fixes to hold on any map they download later, not just AWS:

1. **Stuck robot.** The first patrol goal parked it in a corner pocket at (-2.98, 9.65), 0.7 m below
   a wall. It can't turn around there: the padded footprint's corner sweeps 0.735 m. The only exit
   is reversing, which the route check doesn't model, so every later patrol search came back empty.
   Root cause: patrol spots had to be reachable but not leavable.
2. **Slow "calculating a path".** There's a 2–5 s pause before each goal. Most of it is the
   explorer's pure-Python heading-aware search, about 0.9M states on a 278×415 costmap.
   - When no frontier is reachable, `costmap_frontier_candidates(stop_after_first=True)` searches
     the whole reachable space.
   - `patrol_candidates(stop_after_first=False)` then repeats that full search from scratch.
   - The search loop also never stops early, even once every target has been resolved
     (`frontier_algorithm.py` ~L784).
3. **Driving too close to shelves.** Paths come from Nav2 (Smac lattice / Navfn) on the global
   costmap, which has `inflation_radius: 0.75` and `cost_scaling_factor: 3.0`.
   - Cost drops to 0 at 0.75 m from an obstacle, so planned paths may pass with only ~0.2–0.3 m
     between the robot's edge and a shelf.
   - These values live in the shared `src/amr_navigation/config/planner.yaml`. The factory runs
     were validated with it, and its hash is pinned by the Native45 replay.
   - The user chose to change clearance for **all exploration maps only**. The factory keeps its
     tuning.

The user will shut down the running sim themselves.

## Process
Same as the patrol work. Root writes the acceptance tests first, checks they fail on the current
code, and pins their SHA256. Sonnet 5.5 at medium effort implements from a bounded packet and may
not edit pinned files. Root reviews the diff, re-runs tests, and validates in the sim. Nothing is
committed. `planner.yaml`, `controller.yaml` and the factory launches are not touched.

## A. Patrol spots must allow a full turn in place
- **File:** `src/amr_exploration/scripts/frontier_algorithm.py` (`patrol_candidates`).
- **Change:** a spot qualifies only if the robot can rotate fully in place there.
  - Define a disk at the cell centre whose radius is the circumscribed radius of the footprint passed in.
  - The disk must lie inside the costmap, and no costmap cell of cost ≥ 254 may intersect it.
  - Check this with a cell-offset list computed once per call, not 32 footprint checks per sample.
  - This is footprint- and costmap-based, so it works on any map.
- **Tests:** already written in `src/amr_exploration/test/test_frontier_patrol.py`.
  - `test_patrol_rejects_cells_where_the_robot_cannot_turn_in_place` is the regression for the
    AWS pocket, using a room plus a 1 m dead-end corridor. It is red on the current code.
  - `test_patrol_turn_in_place_gate_keeps_open_space_candidates` checks the gate doesn't
    over-reject.
- **Re-pin:** that test file's hash must be recorded again.

## B. Halve the planning pause (any map)
File: `src/amr_exploration/scripts/frontier_algorithm.py` (`_reachable_costmap_cells`).
1. **Early exit.** In the heading-aware loop, stop as soon as every target is resolved
   (`remaining` is empty), for both `stop_after_first` modes. Results are identical: priority
   order is still applied over the resolved set.
2. **One exhaustive search per planning cycle.**
   - Keep a single-entry module cache, `_REACHABILITY_CACHE`, of the last search that ran to
     completion (queue exhausted).
   - Key: costmap geometry, `bytes(data)`, footprint, start world and start yaw.
   - Value: the arrival-heading bitmask for each visited costmap cell, stored compactly as one int
     per cell, never a set of state tuples.
   - On a key hit, resolve any target set by lookup: a target is reachable if it has some arrival
     heading for which `goal_turn_clear` passes.
   - Effect: when the frontier search exhausts the space, the patrol search right after it costs
     almost nothing.
- **Root-written pinned tests** (new tests in `test_frontier_algorithm.py` or the patrol test
  file), red first where they can be:
  - **Cache hit:** costmap where no frontier is reachable. Run `costmap_frontier_candidates(...,
    require_path_clear=True)`, then `patrol_candidates` on the same costmap and pose. With
    `_first_step_headings` wrapped by a monkeypatched counter, it is called once. The patrol
    result equals a cold run after setting `frontier_algorithm._REACHABILITY_CACHE = None`.
  - **Cache miss:** a changed costmap byte, pose or yaw gives a miss (counter increments).
  - **Equivalence:** `_reachable_costmap_cells` returns identical sets with a cold and a warm
    cache, and with and without early exit, on existing fixtures plus the corridor pocket.
- **Expected effect:** about 1.5–2 s instead of 3–5 s on the AWS map. The pause still grows with
  the reachable area of a downloaded map. A faster search (numpy or C++) remains a possible
  follow-up and is not in this plan.

## C. Wider driving clearance for every exploration map (factory untouched)
- **New overlay** `src/amr_simulation/config/exploration_navigation_overlay.yaml`, keyed
  `/amr/global_costmap/global_costmap:`:
  - `inflation_layer.inflation_radius: 1.0` (from 0.75)
  - `inflation_layer.cost_scaling_factor: 2.0` (from 3.0)
  - Under Nav2's inflation formula, a path 1.0 m from an obstacle then still costs about 79
    instead of 0. The planner (`cost_penalty: 2.0`) then prefers aisle centres.
  - Lethal and inscribed costs (≥ 253) are unchanged, so the explorer's route checks, the
    smoother's collision check and passable space are the same as now.
  - Local costmap and RPP are untouched: RPP's `inflation_cost_scaling_factor` must match the
    local costmap's scaling factor, so changing only the global costmap is safe.
  - Values are a starting point. The user judges in RViz and they can be tuned in this one file.
- **`src/amr_navigation/launch/amr_navigation.launch.py`:** add an optional argument
  `params_overlay` (default `""`).
  - When it's non-empty, append the file after `planner.yaml` in `planner_server`'s parameters.
    The global costmap runs in that process.
  - Use an OpaqueFunction or a condition, following the existing `amr_mpc_controller`
    `enable_final_position_profiles` precedent.
  - A dict can't reach the nested costmap node, so this has to be a YAML file.
  - The factory, mapping and standalone launches pass nothing, so they behave exactly as today.
- **`src/amr_simulation/launch/portable_exploration.launch.py`:**
  `_package_launch_include("amr_navigation", arguments={"params_overlay": <share>/config/exploration_navigation_overlay.yaml})`.
  The AWS presets and any downloaded world inherit it. Add the config directory to
  `amr_simulation`'s install rules if needed.
- **Root-written pinned tests:**
  - The navigation launch declares `params_overlay` with default `""`.
  - With an empty value, `planner_server`'s parameters equal today's.
  - With a value, the overlay comes after `planner.yaml`.
  - The portable launch passes the overlay path.
  - The overlay YAML is keyed to the global costmap node and has `inflation_radius ≥ 0.75` and
    `0 < cost_scaling_factor ≤ 3.0`.
  - Existing `test_navigation_contract.py` and the Native45 `planner.yaml` hash must stay green
    and unmodified.

## Files to change (implementer)
- `src/amr_exploration/scripts/frontier_algorithm.py` (A, B)
- `src/amr_navigation/launch/amr_navigation.launch.py` (C)
- `src/amr_simulation/launch/portable_exploration.launch.py` (C)
- `src/amr_simulation/config/exploration_navigation_overlay.yaml` (new, C)
- `src/amr_simulation/CMakeLists.txt` (install the config only if not already installed)
- `docs/SIMULATION_COMMANDS.md` (one note on the exploration clearance overlay)
- Root-only: the pinned test files and their CMake registration.

## Verification
1. **Source:** pin hashes OK. `colcon build` and `colcon test` for `amr_exploration`,
   `amr_navigation`, `amr_simulation` and `amr_mpc_controller`, all green with no regressions.
   Previous totals were 238 (2 skipped), navigation n/a, 158 and n/a.
2. **Timing on real data:** replay the patrol and frontier searches on the saved AWS map, as in
   today's probe. Report before and after timings.
3. **Patrol GUI sim:** launch `aws_warehouse_exploration.launch.py` with
   `continuous_exploration:=true` and GUI. Watch `/amr/exploration/status` and
   `/amr/mission/status`. Pass criteria:
   - no FAULT
   - no "no reachable patrol goal" streak longer than 30 s
   - at least 10 patrol goals reached
   - the PLANNING → GOAL_PENDING pause logged for each goal, with a median of 2 s or less
   - the user confirms visibly larger clearance in RViz
   - the user stops it with `/amr/exploration/stop`
4. **Default-mode regression:** one bounded AWS acceptance run through
   `aws_exploration_diagnostics_run.py`, because the overlay also applies there. It must reach
   COMPLETE with no faults, save and verify the map, and shut down cleanly. Compress its bag
   afterwards to stay within the storage budget.
5. **Record:** update `SESSION_HANDOFF.md` with the changes, evidence paths and residuals:
   - the pause still scales with map area
   - frontier goals could in principle hit the same pocket issue; the turn-in-place gate applies
     to patrol only
   - the open shutdown segfault in `product_camera_adapter_node` seen in run 06
