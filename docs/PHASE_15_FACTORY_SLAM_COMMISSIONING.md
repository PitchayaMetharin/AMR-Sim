# Phase 15 — Factory SLAM commissioning

## Authority and status

The factory SLAM commissioning plan is approved as a bounded follow-on to the
simulation-only online SLAM work in Phase 9. Slices 1 and 2 are implemented in
source. Phase 14 remains deferred and is not advanced by this plan. Runtime
commissioning evidence is a separate activity and is not claimed here.

## Objective

Provide a factory-world commissioning entry point that uses the existing robot,
sensor adapters, local EKF, rendering controls, and attachment option while
keeping production localization unchanged:

- `factory_localization.launch.py` remains the production static-map + AMCL
  entry point, with its existing defaults and `nav2_amcl` ownership of
  `map -> odom`.
- `factory_mapping.launch.py` is the separate online-mapping entry point.
  SLAM Toolbox is the sole `map -> odom` authority; it does not launch
  `nav2_map_server` or AMCL.
- The local EKF remains the sole `odom -> base_footprint` authority.
- Command arbitration remains the sole `/amr/control/cmd_vel` publisher.

## Slice 1 — factory mapping entry point

1. Add the separate factory mapping launch. Forward the existing `headless`,
   rendering, attachment, and initial-pose arguments to the shared factory
   launch, and run SLAM Toolbox with the established front adapted LaserScan
   and mapper configuration.
2. Keep manual and autonomous mapping as mutually exclusive control modes.
   Manual mode omits the Nav2 controller and mission launch, so the existing
   `amr_control/prototype_teleop.py` is the only expected
   `/amr/mpc/cmd_vel` publisher. Autonomous mode starts the existing Nav2
   mission chain and the frontier explorer.
3. Add an explicit `map_yaml` argument to the production factory localization
   launch, defaulting to the canonical `maps/factory.yaml`, and pass it to
   `map_server`. AMCL behavior and production defaults remain unchanged.
4. Declare direct runtime dependencies and add focused source contracts for
   mode exclusivity, forwarded arguments, defaults, and mapping/factory TF
   ownership.
5. Document the manual mapping and candidate-map workflow without claiming
   runtime evidence.

## Slice 2 — autonomous exploration and map artifacts

- `amr_exploration/frontier_explorer.py` selects free/unknown frontiers from a
  live map and sends one goal at a time through
  `/amr/mission/navigate_to_pose`. It never publishes velocity directly.
- Autonomous mapping launches the existing Nav2 planner/controller/mission
  chain; manual mapping omits that chain so teleoperation remains exclusive.
- Stale map/TF, missing motion authority, unavailable actions, repeated goal
  failures, and unconfirmed cancellation fault closed. The stop service is
  `/amr/exploration/stop`.
- `factory_mapping_cli.py save` stores the occupancy map, serialized pose
  graph, surveyed-datum manifest, and transformed candidate YAML under the
  explicit session directory. `validate` checks the artifacts; `discard`
  requires an explicit confirmation and a generated manifest.

## Deferred runtime evidence

Direct-host commissioning, map-quality review, and candidate-map production
acceptance remain a separate runtime validation pass. They must not replace
the production AMCL path or overwrite the canonical map automatically.

The mapping launch also preserves the factory control interlock
`require_manipulator_stowed=true`. The current manipulation supervisor starts
without a valid stowed authority until the deferred Phase 14 work supplies
that proof, so mapping motion is intentionally fail-closed in the meantime;
this source implementation does not invent or bypass it.

## Acceptance boundaries

The implemented source is complete only when focused tests, launch-file
compilation, and package builds pass; production still selects the canonical
map and AMCL by default; mapping has one explicit SLAM Toolbox map-to-odom
owner; and autonomous exploration has no direct velocity publisher. Runtime
acceptance remains unverified. Any candidate map must be written to a
run-specific path; the canonical factory map is not an output target.
