# AMR simulation command reference

This is the copy/paste reference for running the simulation from a clean
terminal. It applies to the laptop-only ROS 2 Humble and Gazebo Harmonic
workspace. It does not make physical-robot, hardware, or functional-safety
claims.

Current Gate 6 status: the 1 kg run is accepted. The independent 3 kg and 5 kg
preparation commands are implemented and source-validated, but their live
runtime evidence is still pending, so Gate 6 remains open.

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

## Factory and Gate 6 simulation

The factory launch uses the registered static factory map and AMCL. It starts
the localization, perception, Nav2 planner/smoother, RPP controller,
arbitration, and mission nodes, but not MoveIt.

### Terminal 1 — factory world

Paste the common setup, run the host renderer preflight, then choose GUI or
headless mode. Factory launch defaults now require hardware rendering so a
timing-sensitive run cannot silently fall back to llvmpipe. The arguments are
shown explicitly below for clarity:

```bash
ros2 run amr_factory factory_runtime_preflight.py host \
  --evidence-dir "$ROS_LOG_DIR/evidence"

# GUI mode:
ros2 launch amr_factory factory_localization.launch.py \
  headless:=false software_rendering:=false \
  require_hardware_rendering:=true \
  factory_attachment:=false \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0

# Headless mode:
# ros2 launch amr_factory factory_localization.launch.py \
#   headless:=true software_rendering:=false \
#   require_hardware_rendering:=true \
#   factory_attachment:=false \
#   initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

If the host preflight fails, stop and fix the host/device access problem. Do
not substitute software rendering for timing-sensitive evidence.

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
  require_hardware_rendering:=true factory_attachment:=false \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

For autonomous frontier exploration, use the same launch with
`control_mode:=autonomous`; do not run keyboard teleoperation at the same time:

```bash
ros2 launch amr_factory factory_mapping.launch.py \
  control_mode:=autonomous session_dir:="$mapping_session" \
  headless:=false software_rendering:=false \
  require_hardware_rendering:=true factory_attachment:=false \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

Both modes preserve the factory `require_manipulator_stowed=true` interlock.
Until Phase 14 provides a valid stowed authority, command arbitration remains
fail-closed and these commands will not move the robot; do not weaken that
gate to force a mapping run.

Stop autonomous exploration only through its cancellation boundary:

```bash
ros2 service call /amr/exploration/stop std_srvs/srv/Trigger {}
```

Once a valid stowed authority is available, manual mode drives the robot
through the factory with the existing teleop boundary. Do not start teleop in
autonomous mode:

```bash
ros2 run amr_control prototype_teleop.py
```

After reviewing the online map, the commissioning CLI saves a candidate outside
the canonical map directory, including the pose graph, surveyed home datum,
and writes a manifest. It never overwrites `src/amr_factory/maps/factory.yaml`:

```bash
ros2 run amr_factory factory_mapping_cli.py save \
  --session-dir "$mapping_session" --name factory_candidate \
  --datum-x 2.4 --datum-y 3.0 --datum-yaw 0.0
ros2 run amr_factory factory_mapping_cli.py validate \
  --session-dir "$mapping_session" --name factory_candidate
```

Review the candidate before production and pass its exact YAML path:

```bash
ros2 launch amr_factory factory_localization.launch.py \
  map_yaml:="$mapping_session/factory_candidate.yaml" \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

To remove one generated session, use the explicit, manifest-guarded command;
the canonical map is never a valid discard target:

```bash
ros2 run amr_factory factory_mapping_cli.py discard \
  --session-dir "$mapping_session" --confirm
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
ros2 lifecycle get /amr/command_arbitration_node
ros2 lifecycle get /amr/controller_server
ros2 lifecycle get /amr/planner_server
ros2 lifecycle get /amr/smoother_server
ros2 action list
ros2 control list_controllers -c /controller_manager
```

The lifecycle nodes should report `active`; the arm and gripper controllers
should report `active`; and the action list should include the mission,
planning, smoothing, and follow-path endpoints.

### Optional — empty-arm motion check

Run this only after MoveIt is ready and before the product stage:

```bash
ros2 launch amr_manipulation gate6_empty_motion.launch.py
```

### Optional — factory RViz view

Factory simulation has no project-specific RViz layout yet, but RViz2 can be
run against the same graph. Paste the common setup in another terminal, then:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

Add the `TF`, `RobotModel`, `Map`, `LaserScan`, and `Path` displays as needed.

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
ros2 bag record --include-hidden-topics --include-unpublished-topics \
  -o "$ROS_LOG_DIR/product101_evidence" \
  /clock /tf /tf_static \
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
  /amr/control/dock_egress/_action/goal \
  /amr/control/dock_egress/_action/feedback \
  /amr/control/dock_egress/_action/result \
  /amr/control/dock_egress/_action/status \
  /amr/mpc/cmd_vel /amr/control/cmd_vel /amr/simulation/base/cmd_vel \
  /amr/base/joint_states /amr/simulation/base/joint_states \
  /amr/base/status /amr/manipulation/status \
  /amr/simulation/contacts/left_finger \
  /amr/simulation/contacts/right_finger \
  /amr/simulation/internal/attachment/product_101/attach \
  /amr/simulation/internal/attachment/product_101/detach \
  /amr/simulation/internal/attachment/product_101/state \
  /model/product_a/pose
```

Every continuation backslash (`\`) must be the final character on its line;
do not use doubled backslashes or add extra text after it. The attachment
state topic is `/amr/simulation/internal/attachment/product_101/state`, and
the product pose topic is `/model/product_a/pose`.

Wait for `Recording...` before starting the stage. Stop the recorder with
`Ctrl-C` only after the stage has finished so the remaining messages are
written.

### Terminal 5 — product stage

Paste the common setup, then choose exactly one test. The accepted 1 kg path is
unchanged and should not be rerun for the 3 kg or 5 kg tests:

```bash
# Existing accepted 1 kg path — unchanged:
ros2 launch amr_manipulation gate6_mass_stage.launch.py product_id:=101
```

For the independent 3 kg or 5 kg test, use one of these aliases instead. Keep
the factory and MoveIt terminals running; the runner resets only the selected
product to its registered pickup station, leaves the AMR at its current pose,
then navigates to that product's dock before starting the existing mass stage:

```bash
# Product B, 3 kg (tag 102):
ros2 launch amr_manipulation gate6_3kg_test.launch.py

# Product C, 5 kg (tag 103):
# ros2 launch amr_manipulation gate6_5kg_test.launch.py
```

The runner refuses to reset if the arm is attached, deployed, moving, faulted,
or not at empty stow, and refuses to run beside an active Gate 6 stage. Run
only one product test at a time. Stop at the first failed gate and retain the
run directory; do not start the 5 kg test after a failed 3 kg test without
reviewing the failure.

For 3 kg or 5 kg evidence, add the matching product topics to the recorder:

```bash
# 3 kg:
/amr/simulation/internal/attachment/product_102/attach
/amr/simulation/internal/attachment/product_102/detach
/amr/simulation/internal/attachment/product_102/state
/model/product_b/pose

# 5 kg:
# /amr/simulation/internal/attachment/product_103/attach
# /amr/simulation/internal/attachment/product_103/detach
# /amr/simulation/internal/attachment/product_103/state
# /model/product_c/pose
```

The reset service is exposed by `factory_localization.launch.py`; do not use
the optional `factory_demo.launch.py` supervisor at the same time as a manual
product test.

## Factory supervisor and CLI (optional)

The demo launch adds the factory and manipulation supervisors to the factory
graph. It does not replace the manual Gate 6 stage and does not start MoveIt:

```bash
ros2 launch amr_factory factory_demo.launch.py \
  headless:=false software_rendering:=false \
  require_hardware_rendering:=true \
  factory_attachment:=false \
  initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

With the factory graph running, the CLI commands are:

```bash
ros2 run amr_factory factory_cli.py list
ros2 run amr_factory factory_cli.py status
ros2 run amr_factory factory_cli.py mode manual
ros2 run amr_factory factory_cli.py mode autonomous
ros2 run amr_factory factory_cli.py send pickup_a dispatch --timeout 120
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
