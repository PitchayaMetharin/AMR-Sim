# AMR simulation command reference

This is the copy/paste reference for running the simulation from a clean
terminal. It applies to the laptop-only ROS 2 Humble and Gazebo Harmonic
workspace. It does not make physical-robot, hardware, or functional-safety
claims.

Current Phase 14 status: Product 101 (1 kg) and Product 102 (3 kg) passed the
approved autonomous factory-cycle scope. Normal-cycle evidence is retained in
`.ros_logs/amr_autonomous_factory_20260908_11/` and
`.ros_logs/amr_autonomous_factory_20260908_13/`; cancellation evidence is in
`_14/`, and graceful-stop evidence is in `_16/`. Product 103 (5 kg) and Gate 7
are out of scope and remain disabled. Do not rerun an accepted product merely
for progression. The `.ros_logs` retention set is intentionally capped below
1 GB; use a targeted recorder and retain only the gate/analyzer/status evidence
needed for the run.

As of 2026-09-12, Phase 15 packets P0 through P8 are implemented and
independently accepted at the source/offline boundary. No fresh canonical
factory acceptance runtime was run after the factory CLI timeout and
cycle-adapter shutdown fixes. The installed Python API still exposes no
publisher GID, so per-edge TF publisher ownership remains fail-closed;
aggregate `/tf` publisher lists are not a substitute.
The recorded Phase 15 mapping runtime is accepted at the safe terminal
boundary: its non-faulted outcome was `INCOMPLETE` and its bounded runtime
report passed. Human map-quality approval, promotion, canonical-map
replacement, and hardware or functional-safety acceptance remain unverified
and must not be inferred from the commands below.

On 2026-09-14, a separate AWS warehouse visualization/mapping smoke run reached
the safe terminal state `INCOMPLETE` with final map version 899, no goal
failures, and no latched fault. That run used a temporary visualization launch
and kept the map in memory; that former launch flow is historical, superseded,
and non-current. The packaged AWS launch below is the supported entry point. This
runtime result is not human map-quality approval, canonical factory-map
acceptance, or promotion. The persistent save/validate flow below is the
supported way to retain a run-specific candidate.

For mapping, `world` means a trusted absolute local SDF 1.9 Gazebo environment;
`/map` means a fresh in-memory SLAM occupancy map created for each launch. A
saved map is not loaded automatically and no saved map is loaded or
overwritten by these commands. `resource_paths` is a colon-separated list of
absolute local model roots, or the empty string. The portable validator rejects
direct remote worlds, `file://` resources, malformed or unversioned Fuel URLs,
and unresolved local model references.

The current navigation chain is:

```text
NavFn planner -> collision-checked SimpleSmoother -> Regulated Pure Pursuit
-> /amr/mpc/cmd_vel -> command_arbitration_node -> base_adapter_node -> Gazebo
```

## Before every run

Use a fresh `AMR_RUN_ID`, `GZ_PARTITION`, `ROS_DOMAIN_ID`, and log directory.
Repeat the same setup in every terminal belonging to that run. Source the
workspace environment before exporting the run-specific domain because
`amr_ros_env.sh` sets a default domain.

Keep the complete `.ros_logs` directory below 1 GB. Do not use
`ros2 bag record -a` for routine work: high-rate clock, sensor, and controller
topics can create multi-gigabyte bags in minutes. Record only the explicit
topics required by the gate being exercised, and preserve the gate/analyzer/
status text outputs as the primary evidence.

Paste this in each terminal, changing the run ID and domain only for a new
run:

```bash
cd /home/pete/amr_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export GZ_VERSION=harmonic
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOCALHOST_ONLY=1
export AMR_RUN_ID=manual_01
export GZ_PARTITION=amr_${AMR_RUN_ID}
export ROS_DOMAIN_ID=230
export ROS_LOG_DIR="$PWD/.ros_logs/$AMR_RUN_ID"
mkdir -p "$ROS_LOG_DIR"
```

Do not source `amr_system.launch.py` as the full simulation launch. It is a
small bringup contract launch and deliberately sets its own ROS domain.

## Build and test

Run this after source changes, or whenever `install/` does not contain the
current packages:

```bash
cd /home/pete/amr_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --symlink-install
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export ROS_DOMAIN_ID=230
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
colcon test
colcon test-result --verbose
```

Do not start a product run if the relevant test result reports an error or
failure.

## Standalone simulation

This starts the base, sensors, localization, SLAM, Nav2 planning/smoothing,
RPP, arbitration, and mission stack without the factory world or MoveIt.
The legacy `amr_simulation.launch.py` entry point below remains a manual smoke
path; world-parameterized autonomous exploration uses the portable launch in
the next section.

### Terminal 1 — Gazebo and ROS graph

Paste the common setup, then choose one launch:

```bash
# GUI mode:
ros2 launch amr_simulation amr_simulation.launch.py headless:=false

# Headless mode:
# ros2 launch amr_simulation amr_simulation.launch.py headless:=true
```

### Terminal 2 — keyboard teleoperation

Paste the common setup, then run:

```bash
ros2 run amr_control prototype_teleop.py
```

Keys are `W`, `S`, `A`, and `D` to move, `X` or Space to stop, and `Q` to
quit. Teleoperation still enters through `/amr/mpc/cmd_vel`; it does not
bypass arbitration or the base adapter.

### Terminal 3 — optional RViz

Paste the common setup, then run:

```bash
rviz2 -d install/amr_simulation/share/amr_simulation/rviz/sensors.rviz \
  --ros-args -p use_sim_time:=true
```

If a LiDAR or point-cloud display is blank, set its Reliability Policy to
`Best Effort`.

## Portable exploration with a local world

Use this world-agnostic entry point for a trusted absolute local SDF 1.9 world.
The example uses the registered simple world and starts the complete portable
exploration graph with the empty arm, safety authority, SLAM, navigation, and
frontier explorer:

```bash
ros2 launch amr_simulation portable_exploration.launch.py \
  world:=/home/pete/amr_ws/src/amr_simulation/worlds/amr_world.sdf \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.12 initial_yaw:=0.0 \
  resource_paths:='' headless:=false rviz:=true \
  auto_start_exploration:=true
```

With `auto_start_exploration:=true` (the default), readiness releases the
explorer and no `/amr/exploration/start` call is needed. Only an intentional
`auto_start_exploration:=false` launch uses that service after readiness. Stop
or cancel exploration only through its cancellation boundary and wait for a
non-faulted terminal state before saving.

## Canonical AWS warehouse exploration

The packaged AWS preset uses the derived `aws_warehouse.sdf` and the same
portable graph. It declares only the `headless`, `rviz`, and
`auto_start_exploration` toggles; it adds no runtime nodes of its own and
includes the portable launch once:

```bash
ros2 launch amr_simulation aws_warehouse_exploration.launch.py \
  headless:=false rviz:=true auto_start_exploration:=true
```

The SDF contains exact, pinned OpenRobotics Fuel model URLs. Model bundles are
not vendored: first use requires DNS/TLS/network access to the canonical Fuel
host unless the exact host, owner, model, and revision are already cached; a
cache with every exact revision may then be reused offline. See the complete
[AWS Fuel attribution and provenance record](../src/amr_simulation/assets/AWS_WAREHOUSE_FUEL_ATTRIBUTION.md)
and the official [industrial-warehouse SDF source](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/industrial-warehouse/4/files/industrial-warehouse.sdf).

The packaged RViz view uses fixed frame `map` and displays `/map`, the global
and local costmaps, `/robot_description`, TF, `/amr/pose`, and front/rear
LaserScan streams; point clouds are optional. `/map` and both costmaps use
Reliable/Transient Local depth 1, costmap updates use Reliable/Volatile,
`/amr/pose` uses Reliable/Volatile, and both LaserScan streams use
Best Effort/Volatile. Mapping uses SLAM Toolbox and intentionally does not run
AMCL.

The AWS run autostarts exploration; do not issue a separate start call for the
command above. After the recorded run reaches a non-faulted terminal
`COMPLETE` or safe `INCOMPLETE` state with no active or pending goal and no
latched fault, save and validate a run-specific candidate explicitly:

```bash
aws_session="$ROS_LOG_DIR/aws_mapping"
mkdir -p "$aws_session"
ros2 run amr_factory factory_mapping_cli.py save \
  --session-dir "$aws_session" --name aws_candidate \
  --datum-x 0.0 --datum-y 0.0 --datum-yaw 0.0
ros2 run amr_factory factory_mapping_cli.py validate \
  --session-dir "$aws_session" --name aws_candidate
```

The datum must match the AWS preset spawn `(0.0, 0.0, 0.12, 0.0)`; the
candidate remains run-specific and outside the canonical factory map. Saving
is opt-in: there is no automatic persistence. Do not run `accept` or `promote`,
copy a candidate into the canonical map directory, replace a canonical map, or
infer human quality approval from this runtime result. Only an intentional
`auto_start_exploration:=false` launch uses the start service, after readiness.

## Factory and Gate 6 simulation

The factory launch uses the registered static factory map and AMCL. It starts
the localization, perception, Nav2 planner/smoother, RPP controller,
arbitration, and mission nodes, but not MoveIt.

For the approved native-attachment autonomous product path, use
`factory_autonomous.launch.py` with `factory_attachment:=true`; the completed
acceptance runs used that mode. The mapping commissioning commands below also
use `factory_attachment:=true` so the stowed-authority evidence is explicit.

The canonical autonomous factory launch is:

```bash
ros2 launch amr_factory factory_autonomous.launch.py \
  factory_attachment:=true headless:=true software_rendering:=false \
  require_hardware_rendering:=true control_mode:=autonomous \
  initial_x:=-4.5 initial_y:=0.0 initial_yaw:=0.0
```

The lower-level `factory_localization.launch.py` remains useful for manual
static-map inspection and is included by the autonomous launch; it is not the
standalone entry point for an accepted autonomous factory cycle.

### Terminal 1 — canonical autonomous factory world

Paste the common setup, run the host renderer preflight, then choose GUI or
headless mode. Factory launch defaults now require hardware rendering so a
timing-sensitive run cannot silently fall back to llvmpipe. The arguments are
shown explicitly below for clarity:

```bash
ros2 run amr_factory factory_runtime_preflight.py host \
  --evidence-dir "$ROS_LOG_DIR/evidence"

# GUI mode:
ros2 launch amr_factory factory_autonomous.launch.py \
  headless:=false software_rendering:=false \
  require_hardware_rendering:=true factory_attachment:=true \
  control_mode:=autonomous initial_x:=-4.5 initial_y:=0.0 initial_yaw:=0.0

# Headless mode:
# ros2 launch amr_factory factory_autonomous.launch.py \
#   headless:=true software_rendering:=false \
#   require_hardware_rendering:=true factory_attachment:=true \
#   control_mode:=autonomous initial_x:=-4.5 initial_y:=0.0 initial_yaw:=0.0
```

If the host preflight fails, stop and fix the host/device access problem. Do
not substitute software rendering for timing-sensitive evidence.

`factory_autonomous.launch.py` is the canonical autonomous entry point.
`factory_attachment:=true` is mandatory for accepted Product 101/102 cycles
and mapping commissioning because it starts the native attachment bootstrap and
detachable-joint interfaces used by the fail-closed attachment proof. A
`factory_attachment:=false` launch is limited to explicitly non-product
visualization and is not an acceptance path. `factory_demo.launch.py` remains a
legacy/optional launch and is not a current acceptance entry point.

### Factory online mapping (manual or autonomous commissioning)

Use a run-specific session directory. This entry point starts SLAM Toolbox as
the sole `map -> odom` publisher and does not start the static map server or
AMCL. Manual mode omits Nav2 controller/mission so the existing teleop process
is the only command source. Autonomous mode starts the existing Nav2 mission
chain and the fail-closed frontier explorer.

```bash
mapping_session="$ROS_LOG_DIR/factory_mapping"
mkdir -p "$mapping_session"
ros2 launch amr_factory factory_mapping.launch.py \
  control_mode:=manual session_dir:="$mapping_session" \
  headless:=false software_rendering:=false \
  require_hardware_rendering:=true factory_attachment:=true \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

For autonomous frontier exploration, use the same launch with
`control_mode:=autonomous`; do not run keyboard teleoperation at the same time.
Start the mapping launch and leave it active in one terminal:

```bash
ros2 launch amr_factory factory_mapping.launch.py \
  control_mode:=autonomous session_dir:="$mapping_session" \
  headless:=false software_rendering:=false \
  require_hardware_rendering:=true factory_attachment:=true \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

The mapping Explorer uses mapping-specific readiness. Its TF lookup requests
`map -> base_footprint` at the node's exact current ROS time and rejects zero,
future, stale, nonfinite, or invalid-quaternion samples. Production planning
checks non-TF readiness first, clusters frontiers, then obtains one carried TF
sample for robot-pose and candidate computation; the final reservation gate
revalidates that same sample and sends no goal if it expires while waiting.
Initial autostart/start readiness has a 15-second grace episode. After a full
readiness success, a later no-motion `WAITING_READY`/`SCANNING` miss receives a
fresh continuous 15-second episode: recovery returns to `SCANNING` and clears
it, while a persistent miss faults closed. Readiness loss during
`GOAL_PENDING` or `NAVIGATING` still cancels immediately rather than using
this grace period.

After the mapping graph passes its readiness gates, use a separate terminal to
start exploration through its explicit service before collecting the map:

```bash
ros2 service call /amr/exploration/start std_srvs/srv/Trigger {}
```

Both modes preserve the factory `require_manipulator_stowed=true` interlock.
Accepted Phase 14 Product 101/102 runtime evidence included stowed proof, but
it does not prove current stowed authority for a Phase 15 mapping run. The
current mapping run must provide that proof; otherwise command arbitration
remains fail-closed and these commands will not move the robot. Do not weaken
that gate to force a mapping run.

Stop autonomous exploration only through its cancellation boundary:

```bash
ros2 service call /amr/exploration/stop std_srvs/srv/Trigger {}
```

Wait for the final exploration status to be non-faulted and inactive before
saving the candidate. The current P6 acceptance gate requires a terminal
`STOPPED`, `COMPLETE`, or `INCOMPLETE` state, fresh status, explicit
`active: false`, `pending: false`, and `fault_latched: false`, a positive
`run_generation`, and an exact empty `cancel_target`. Do not treat
`WAITING_READY`, `SCANNING`, `GOAL_PENDING`, `NAVIGATING`, or `CANCELLING` as
completion. If frontiers remain without a safe reachable goal, the required
outcome is `INCOMPLETE`/safely stopped with explicit
`incomplete_acknowledgement: ACCEPT_INCOMPLETE` and a nonblank
`incomplete_explanation`. Do not call `start` again after the stop boundary
unless you are intentionally beginning a new run-specific session.

#### Fresh mapping versus a saved candidate

Every invocation of `factory_mapping.launch.py` starts SLAM Toolbox with a new
in-memory map. The `session_dir` stores run evidence and explicitly saved
artifacts; it is not loaded automatically on the next launch. To demonstrate
mapping from scratch, use a new `AMR_RUN_ID`/`ROS_LOG_DIR` and a new
`mapping_session`, then launch without a `map_yaml` argument. Do not delete the
canonical map to reset a demonstration.

Once a valid stowed authority is available, manual mode drives the robot
through the factory with the existing teleop boundary. Do not start teleop in
autonomous mode:

```bash
ros2 run amr_control prototype_teleop.py
```

After reviewing the online map, the commissioning CLI saves a candidate outside
the canonical map directory, including the surveyed home datum and a two-file
pose-graph bundle: `<prefix>.posegraph` and `<prefix>.data`. The serializer
receives a bare prefix and appends those suffixes. Manifest schema 3 requires
both exact, regular, non-empty files, records and hashes both files, and rejects
legacy, incomplete, aliased, missing, or tampered bundles. Verified discard is
limited to manifest-enumerated candidate files. The CLI never overwrites
`src/amr_factory/maps/factory.yaml`:

```bash
ros2 run amr_factory factory_mapping_cli.py save \
  --session-dir "$mapping_session" --name factory_candidate \
  --datum-x 2.4 --datum-y 3.0 --datum-yaw 0.0
ros2 run amr_factory factory_mapping_cli.py validate \
  --session-dir "$mapping_session" --name factory_candidate
```

Collect run-specific, read-only evidence before acceptance. The preflight
commands only observe the existing graph and write their reports under the
session; they do not replace the bounded mapping acceptance observer:

```bash
ros2 run amr_factory factory_runtime_preflight.py host \
  --evidence-dir "$mapping_session/evidence"
ros2 run amr_factory factory_runtime_preflight.py runtime \
  --profile phase15_mapping \
  --evidence-dir "$mapping_session/evidence"
ros2 run amr_factory factory_mapping_acceptance.py runtime \
  --session-dir "$mapping_session" --name factory_candidate \
  --mode autonomous --evidence-dir "$mapping_session/evidence"
```

Do not run the generic factory graph preflight as the mapping readiness gate.
Online mapping intentionally omits AMCL, `nav2_map_server`, and `move_group`,
so that generic product graph contract is expected to fail. Use the named
`phase15_mapping` runtime profile and the mapping acceptance observer above;
they validate the SLAM Toolbox/EKF TF path, fresh map, mapping authority, and
the mode-appropriate exploration chain. Human map-quality review and
promotion are separate decisions and are not established by these commands.

After the runtime evidence is collected, inspect the map image and the complete
pose-graph bundle, then create a run-specific
`$mapping_session/quality_review.yaml` containing the exact candidate path,
`candidate_sha256`, quality-review schema 2,
the exact `artifact_bundle_sha256` matching the verified manifest, and the
exact `runtime_acceptance_sha256` for the actual runtime report, a named
reviewer, and `decision: ACCEPTED`. If
exploration ends with remaining frontiers but no safe reachable goal, acceptance
also requires the explicit `INCOMPLETE` acknowledgement and nonblank
explanation described above. `accept` and `promote` revalidate these bound
proofs from persisted observations; do not create a promotion claim from a
transitional snapshot. Verify the three-proof state before creating the
promotion-eligibility receipt:

```bash
ros2 run amr_factory factory_mapping_acceptance.py accept \
  --session-dir "$mapping_session" --name factory_candidate \
  --quality-review "$mapping_session/quality_review.yaml"
ros2 run amr_factory factory_mapping_acceptance.py promote \
  --session-dir "$mapping_session" --name factory_candidate
```

`promote` writes only the run-specific eligibility receipt. It never copies,
replaces, renames over, or modifies `src/amr_factory/maps/factory.*` or the
installed canonical map; canonical replacement requires separate
authorization.

Review the candidate before production and pass its exact YAML path:

```bash
ros2 launch amr_factory factory_localization.launch.py \
  map_yaml:="$mapping_session/factory_candidate.yaml" \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

Saving is opt-in: if `save` is not run, no map candidate is persisted for later
localization. To remove only this generated candidate for another fresh demo,
use the explicit, manifest-guarded command; the canonical map is never a valid
discard target:

```bash
ros2 run amr_factory factory_mapping_cli.py discard \
  --session-dir "$mapping_session" --name factory_candidate --confirm
```

After Gazebo is running and before starting MoveIt, run the bounded RTF gate
in another terminal with the same run environment:

```bash
ros2 run amr_factory factory_runtime_preflight.py runtime \
  --evidence-dir "$ROS_LOG_DIR/evidence"
```

This captures 12 seconds of `/stats` and requires at least 10 valid samples,
median RTF `>= 0.90`, aggregate simulated-time/real-time `>= 0.90`, and a
Gazebo process with an open `/dev/dri/*` device. Keep the raw stats and report
under the run directory. If this check fails, stop the run before MoveIt,
rosbag, or Gate 6.

For GUI evidence, also confirm that the Gazebo window remains visibly
non-black after startup and after at least 60 seconds. Do not run RViz during
the RTF gate or product stage; it may be opened afterward for read-only
inspection.

### Terminal 2 — MoveIt

Paste the common setup, wait for the factory graph to start, then run:

```bash
ros2 launch amr_manipulation move_group.launch.py
```

Wait until MoveIt reports that the planning group is ready.

### Terminal 3 — readiness and graph inspection

Paste the common setup, then run these read-only checks:

```bash
ros2 run amr_factory factory_runtime_preflight.py graph \
  --evidence-dir "$ROS_LOG_DIR/evidence"

ros2 run amr_factory factory_runtime_preflight.py lifecycle \
  --evidence-dir "$ROS_LOG_DIR/evidence/lifecycle_preflight"

ros2 action list
ros2 control list_controllers -c /controller_manager
```

The lifecycle preflight should report `verdict=PASS`; the arm and gripper
controllers should report `active`; and the action list should include the
mission, planning, smoothing, and follow-path endpoints.

The graph preflight creates one persistent ROS 2 observer, requires the 17
factory nodes plus `/move_group`, rejects duplicate required names, and
requires the complete graph to remain stable for two seconds. Do not replace
it with repeated short-lived `ros2 node list --no-daemon` calls: those create
fresh discovery participants and can report incomplete snapshots. If graph
preflight fails, stop before starting the recorder or Product 101.

The lifecycle preflight uses one persistent ROS 2 participant and one client
per required lifecycle node. It requires all 17 nodes to report exact lifecycle
state ID `3` and label `active` continuously for two seconds within one bounded
30-second readiness window. Individual responses are bounded at one second and
a lost response is removed before the next observation. Do not replace this
acceptance check with short-lived `ros2 lifecycle get` processes; fresh DDS
participants can discover the service but lose the response during startup. If
lifecycle preflight fails, stop before starting the recorder or product stage.

The RPP controller launch (retained in the `amr_mpc_controller` compatibility
package) starts `controller_server` before its lifecycle manager and applies a
bounded one-second construction barrier. This protects Humble's zero-delay
lifecycle-manager autostart from racing the controller's nested local-costmap
construction; it does not change controller parameters or command ownership.

### Optional — empty-arm motion check

Run this only after MoveIt is ready and before the product stage:

```bash
ros2 launch amr_manipulation gate6_empty_motion.launch.py
```

### Optional — mapping RViz view

RViz2 can inspect the live map, robot, scans, and TF while mapping is running.
Paste the common setup in another terminal, then:

```bash
rviz2 -d install/amr_simulation/share/amr_simulation/rviz/sensors.rviz \
  --ros-args -p use_sim_time:=true
```

The supplied view already includes `TF`, `RobotModel`, `/map`, both LiDAR
scans, and both point clouds. For the mapping run, add two `Map` displays and
one `PoseWithCovariance` display when you want the navigation overlays:

```text
global costmap: /amr/global_costmap/costmap
local costmap:  /amr/local_costmap/costmap
SLAM pose:      /amr/pose
```

Use Reliable/Transient Local QoS for both costmap displays and
Reliable/Volatile QoS for `/amr/pose`. Mapping mode intentionally does not run
AMCL; `/amr/pose` is the SLAM Toolbox pose. `/amr/amcl_pose` is available only
when the static-map factory localization launch is being used.

### Terminal 4 — record evidence

Paste the common setup in this terminal before starting the recorder. Shell
variables are not shared between terminals, so `ROS_LOG_DIR` must be set here
as well as in the factory terminal. Confirm that it is non-empty and points
inside the workspace:

```bash
printf 'ROS_LOG_DIR=%s\n' "${ROS_LOG_DIR:-<unset>}"
if [ -n "${ROS_LOG_DIR:-}" ]; then
  mkdir -p "$ROS_LOG_DIR"
fi
```

It should print a path such as
`/home/pete/amr_ws/.ros_logs/manual_01`, not an empty value. The factory's
`GZ_PARTITION` and `ROS_DOMAIN_ID` must match this terminal's values.

Start recording before the product stage:

```bash
recorder_qos="$ROS_LOG_DIR/gate6_recorder_qos.yaml"
cat > "$recorder_qos" <<'YAML'
/amr/base/joint_states:
  depth: 5
  reliability: best_effort
  durability: volatile
/tf_static:
  depth: 1
  reliability: reliable
  durability: transient_local
/amr/simulation/attachment_bootstrap/status:
  depth: 1
  reliability: reliable
  durability: transient_local
/amr/simulation/sensors/rear_lidar/scan:
  depth: 5
  reliability: reliable
  durability: volatile
/amr/sensors/rear_lidar/scan:
  depth: 5
  reliability: best_effort
  durability: volatile
/arm_controller/follow_joint_trajectory/_action/status:
  depth: 1
  reliability: reliable
  durability: transient_local
/gripper_controller/gripper_cmd/_action/status:
  depth: 1
  reliability: reliable
  durability: transient_local
/gripper_right_controller/gripper_cmd/_action/status:
  depth: 1
  reliability: reliable
  durability: transient_local
YAML

ros2 bag record --include-hidden-topics --include-unpublished-topics \
  --qos-profile-overrides-path "$recorder_qos" \
  -o "$ROS_LOG_DIR/product_evidence" \
  /clock /tf /tf_static \
  /amr/simulation/attachment_bootstrap/status \
  /amr/amcl_pose /amr/localization/odometry /amr/localization/wheel_odometry \
  /amr/base/odometry_raw /amr/simulation/base/odometry \
  /amr/simulation/ground_truth/pose \
  /amr/plan /amr/plan_smoothed /amr/received_global_plan \
  /amr/lookahead_point \
  /amr/mission/navigate_to_pose/_action/goal \
  /amr/mission/navigate_to_pose/_action/feedback \
  /amr/mission/navigate_to_pose/_action/result \
  /amr/mission/navigate_to_pose/_action/status \
  /amr/compute_path_to_pose/_action/goal \
  /amr/compute_path_to_pose/_action/feedback \
  /amr/compute_path_to_pose/_action/result \
  /amr/compute_path_to_pose/_action/status \
  /amr/smooth_path/_action/goal \
  /amr/smooth_path/_action/feedback \
  /amr/smooth_path/_action/result \
  /amr/smooth_path/_action/status \
  /amr/follow_path/_action/goal \
  /amr/follow_path/_action/feedback \
  /amr/follow_path/_action/result \
  /amr/follow_path/_action/status \
  /arm_controller/follow_joint_trajectory/_action/status \
  /gripper_controller/gripper_cmd/_action/status \
  /gripper_right_controller/gripper_cmd/_action/status \
  /amr/control/dock_egress/_action/goal \
  /amr/control/dock_egress/_action/feedback \
  /amr/control/dock_egress/_action/result \
  /amr/control/dock_egress/_action/status \
  /amr/mpc/cmd_vel /amr/control/cmd_vel /amr/simulation/base/cmd_vel \
  /amr/base/joint_states /amr/simulation/base/joint_states \
  /amr/base/status /amr/manipulation/status \
  /amr/simulation/contacts/left_finger \
  /amr/simulation/contacts/right_finger \
  /amr/simulation/sensors/rear_lidar/scan \
  /amr/sensors/rear_lidar/scan \
  /amr/simulation/internal/attachment/product_101/attach \
  /amr/simulation/internal/attachment/product_101/detach \
  /amr/simulation/internal/attachment/product_101/state \
  /amr/simulation/internal/attachment/product_102/attach \
  /amr/simulation/internal/attachment/product_102/detach \
  /amr/simulation/internal/attachment/product_102/state \
  /amr/simulation/internal/attachment/product_103/attach \
  /amr/simulation/internal/attachment/product_103/detach \
  /amr/simulation/internal/attachment/product_103/state \
  /model/product_a/pose /model/product_b/pose /model/product_c/pose
```

Every continuation backslash (`\`) must be the final character on its line;
do not use doubled backslashes or add extra text after it. Recording all three
registered products keeps one strict recorder command valid for any selected
Gate 6 product; unused product topics may have zero messages.

Wait for `Recording...` before starting the stage. Stop the recorder with
`Ctrl-C` only after the stage has finished so the remaining messages are
written.

The recorder above is the historical full Gate 6 analyzer contract. It is not
the default because it includes high-rate sensors and can exceed 1 GB quickly.
For routine cancellation, graceful-stop, or status/ownership evidence, use
this bounded recorder instead (it deliberately omits `/clock`, lidar, plans,
and other high-rate streams):

```bash
ros2 bag record --include-hidden-topics \
  --qos-profile-overrides-path "$recorder_qos" \
  -o "$ROS_LOG_DIR/compact_evidence" \
  /amr/factory/status /amr/manipulation/status /amr/manipulation/internal/status \
  /amr/factory/run_sequence/_action/status \
  /amr/factory/run_sequence/_action/feedback \
  /amr/manipulation/execute_product_cycle/_action/status \
  /amr/manipulation/execute_product_cycle/_action/feedback \
  /amr/simulation/attachment_bootstrap/status \
  /amr/simulation/internal/attachment/product_101/state \
  /amr/simulation/internal/attachment/product_102/state \
  /amr/amcl_pose /amr/simulation/ground_truth/pose \
  /amr/control/cmd_vel /amr/mpc/cmd_vel /amr/simulation/base/cmd_vel \
  /amr/base/status /amr/localization/odometry /amr/base/odometry_raw \
  /amr/base/joint_states /joint_states \
  /amr/mission/navigate_to_pose/_action/status \
  /amr/mission/navigate_to_pose/_action/feedback \
  /amr/mission/navigate_to_pose_precise/_action/status \
  /amr/mission/navigate_to_pose_precise/_action/feedback \
  /amr/control/dock_egress/_action/status \
  /amr/control/dock_egress/_action/feedback \
  /arm_controller/follow_joint_trajectory/_action/status \
  /gripper_controller/gripper_cmd/_action/status \
  /gripper_right_controller/gripper_cmd/_action/status
```

This compact recorder is sufficient for the accepted stop/cancellation
evidence but is intentionally not a substitute for the full analyzer contract
when a future phase explicitly authorizes a new normal Gate 6 acceptance run.

### Terminal 5 — autonomous factory control

The accepted autonomous boundary is Product 101 (1 kg) and Product 102 (3 kg)
through the registry-backed factory CLI. Product 103 (5 kg) is disabled and
must not be started:

```bash
ros2 run amr_factory factory_cli.py list
ros2 run amr_factory factory_cli.py mode autonomous
ros2 run amr_factory factory_cli.py send pickup_a dispatch --timeout 240
ros2 run amr_factory factory_cli.py send pickup_b dispatch --timeout 240
ros2 run amr_factory factory_cli.py loop pickup_a pickup_b --cycles 1 --finish stay
ros2 run amr_factory factory_cli.py status
```

Use `factory_cli.py stop` to finish the active delivery and stop a sequence,
`factory_cli.py cancel` for immediate cooperative cancellation, and
`factory_cli.py go home` only while idle with fresh safe empty-stowed proof.
Stop at the first failed gate and preserve the run evidence. Do not rerun an
accepted product merely for progression.

The former standalone `gate6_mass_stage` and higher-mass aliases remain
historical diagnostic entry points. They are not the current autonomous
acceptance path; the Product 103/5 kg command must remain disabled.

## Factory supervisor and CLI details

The canonical autonomous launch adds the factory and manipulation supervisors
to the factory graph. MoveIt remains a separately started process:

```bash
ros2 launch amr_factory factory_autonomous.launch.py \
  headless:=false software_rendering:=false \
  require_hardware_rendering:=true \
  factory_attachment:=true control_mode:=autonomous \
  initial_x:=-4.5 initial_y:=0.0 initial_yaw:=0.0
```

`factory_demo.launch.py` is legacy/optional and is not a current acceptance
entry point.

With the factory graph running, the CLI commands are:

```bash
ros2 run amr_factory factory_cli.py list
ros2 run amr_factory factory_cli.py status
ros2 run amr_factory factory_cli.py mode manual
ros2 run amr_factory factory_cli.py mode autonomous
ros2 run amr_factory factory_cli.py send pickup_a dispatch --timeout 240
```

Use the CLI only for the boundaries it exposes. Its high-level transport
path is separate from the manually verified `gate6_mass_stage` acceptance.

## Read-only inspection

Use the common setup in an additional terminal:

```bash
ros2 node list
ros2 action list
ros2 topic list -t
ros2 topic info --verbose /amr/mpc/cmd_vel
ros2 topic info --verbose /amr/control/cmd_vel
ros2 topic echo /amr/control/cmd_vel
ros2 topic echo --qos-reliability best_effort \
  /amr/simulation/ground_truth/pose
ros2 topic echo /amr/manipulation/status
ros2 lifecycle get /amr/command_arbitration_node
ros2 lifecycle get /amr/controller_server
ros2 run tf2_ros tf2_echo map base_footprint
```

For a recorded run:

```bash
ros2 bag info "$ROS_LOG_DIR/product101_evidence"
ros2 bag play "$ROS_LOG_DIR/product101_evidence" --clock
```

The Gate 6 analyzer checks each nonzero simulation command against an exact
arbitration command at or before that output, within the existing 250 ms
freshness bound. The base adapter forwards its cached arbitration command on
an independent 50 ms timer, so a one-tick delay is valid evidence of the same
owned command. An unowned value or stale output still fails closed.

Only play a bag in a separate, read-only ROS domain. Do not replay command
topics into a live plant.

## Shutdown and cleanup

Use `Ctrl-C` in this order:

1. Gate 6 stage or teleoperation.
2. Rosbag recorder; wait for `Recording stopped`.
3. MoveIt.
4. Factory or standalone Gazebo launch.

Then inspect for leftover processes:

```bash
ps -eo pid,ppid,pgid,stat,etime,cmd | rg -i \
  '(gz sim|gzserver|ruby|component_container|controller_manager|rosbag|move_group|gate6)'
```

If a process from this run remains, stop it from the terminal that launched it
or use its exact PID after a read-only check. Avoid broad `pkill` commands.
Preserve the run directory for evidence and use a new run identity next time.

## Common mistakes

- A terminal has a different `ROS_DOMAIN_ID`, `GZ_PARTITION`, or
  `ROS_LOCALHOST_ONLY` value.
- `amr_ros_env.sh` was sourced after the run-specific domain was exported.
- MoveIt was started before the factory graph or its joint-state stream was
  ready.
- The recorder was started after the stage, so action transitions are missing.
- GUI rendering lowers real-time factor. Use headless mode for timing-sensitive
  verification and GUI mode for visual inspection.
- A stale `/amr/control/cmd_vel` publisher is mistaken for the active source.
  Check `ros2 topic info --verbose` and keep the arbitration boundary intact.
