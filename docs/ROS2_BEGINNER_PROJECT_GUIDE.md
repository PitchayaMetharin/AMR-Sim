# Beginner’s guide to this AMR ROS 2 project

This repository is a laptop-only simulation of an autonomous mobile robot (AMR). It uses ROS 2 Humble, C++17, Gazebo Harmonic, Nav2, robot_localization, and SLAM Toolbox. It is not a physical robot controller, fieldbus integration, or functional-safety system.

Its central rule is **fail closed**: a component can request movement, but several later components must independently accept it before the simulated plant moves. Missing, stale, malformed, or inconsistent data becomes zero velocity.

For copy/paste runtime procedures, use [SIMULATION_COMMANDS.md](SIMULATION_COMMANDS.md). This guide explains the architecture and the reason for each command.

## ROS 2 in plain language

| Term | Meaning here |
| --- | --- |
| Workspace | This repository. src/ holds packages; colcon creates build/, install/, and log/. |
| Package | A self-contained ROS unit: metadata, code/config, and tests. |
| Node | A running program communicating with other programs. |
| Topic | Named stream of typed messages; a publisher writes, subscribers receive. |
| Service | Quick request/response operation between ROS nodes. |
| Action | Long-running cancellable goal; used for missions. |
| Lifecycle node | A node deliberately moved unconfigured -> inactive -> active; outputs publish only while active. |
| TF | Coordinate-frame relationships. This project uses map -> odom -> base_footprint. |
| QoS | Topic delivery rules. Commands/authority are reliable and short-lived; simulated sensors are best effort. |

Frames:

```text
map  -- SLAM’s global map
 |
 `-- odom  -- local pose estimate (smooth but can drift)
      |
      `-- base_footprint  -- robot’s ground-plane center
           |
           `-- sensor frames
```

The EKF alone publishes odom -> base_footprint. SLAM Toolbox alone publishes map -> odom. One owner per TF edge avoids conflicting poses.

## The motion path

This is the most important architecture:

```text
Mission goal (optional)
  -> Nav2 planner
  -> collision-checked Nav2 path smoother
  -> Nav2 Regulated Pure Pursuit controller: /amr/mpc/cmd_vel
  -> command_arbitration_node: validate, limit, ramp
  -> /amr/control/cmd_vel
  -> base_adapter_node: validate again and bridge to Gazebo
  -> /amr/simulation/base/cmd_vel
  -> Gazebo native 200 ms watchdog -> simulated drive plant
```

The demonstration teleop tool publishes to /amr/mpc/cmd_vel, entering through the same arbitration boundary. It is not a production operator interface.

The source command expires after 200 ms in arbitration, and the base adapter expires its received command after 200 ms. The Gazebo native watchdog independently disables the plant after 200 ms without a command—even if the ROS base adapter or bridge process fails.

## The sensing and navigation path

```text
Gazebo sensors and joints
  -> base adapter + sensor adapters
  -> wheel joints, IMU, raw LiDAR/point clouds
  -> wheel odometry + EKF
  -> local pose and odom -> base_footprint TF
  -> front/rear perception pipelines
  -> validated point clouds
  -> SLAM map and map -> odom TF
  -> Nav2 costmaps, planner, path smoother, RPP controller
  -> motion path above
```

The front LaserScan feeds SLAM. Both front/rear point clouds feed Nav2 obstacle layers. Perception contributes navigation data but has no command or personnel-safety authority.

## Every package and node in src/

### amr_interfaces — contracts, no runtime node

This package defines the custom message vocabulary: BaseStatus and the
observation-only HealthStatus.

Status messages use boot ID, sequence, validity, and timestamp so consumers can reject replayed, duplicate, malformed, and backward-time evidence.

### amr_bringup — startup ownership

amr_system.launch.py starts and activates health_supervisor_node. interface_ownership.yaml records the intended publisher for named topics. qos_profiles.yaml declares QoS intent, implemented by the C++ helper in amr_interfaces/include/amr_interfaces/qos_profiles.hpp.

### amr_description — robot body, no node

urdf/amr.urdf.xacro describes the simulated links, joints, sensors, and Gazebo plugins. The external robot_state_publisher reads it and publishes robot-frame transforms.

### amr_simulation — Gazebo plant and native watchdog

There is no custom rclcpp node here. amr_simulation.launch.py starts Gazebo, robot_state_publisher, robot spawning, ROS/Gazebo bridging, and each subsystem launch.

command_watchdog_system.cpp is a Gazebo plugin. It enables the plant when native Gazebo command data arrives and disables it after 200 ms without data. It is independent of ROS nodes.

### amr_base_adapter — final ROS boundary before Gazebo

**Node:** base_adapter_node

Input: /amr/control/cmd_vel plus raw Gazebo odometry/joint state.

Output: /amr/simulation/base/cmd_vel, /amr/base/odometry_raw, /amr/base/joint_states, /amr/base/status.

It accepts only finite planar TwistStamped commands in base_footprint; sideways, vertical, roll, and pitch values must be zero. Every 50 ms it forwards a fresh valid command or publishes zero. It passes raw plant state through stable AMR topics and reports whether state is fresh. It does not arbitrate commands.

### amr_sensor_adapters — stable sensor boundaries

**Nodes:** front_lidar_adapter_node, rear_lidar_adapter_node, imu_adapter_node.

These lifecycle nodes copy Gazebo bridge output to stable names:

- Each LiDAR adapter copies scan and point-cloud data from /amr/simulation/sensors/... to /amr/sensors/....
- The IMU adapter copies to /amr/sensors/imu/data_raw.

They do not estimate pose, apply perception policy, or command the robot.

### amr_localization — wheel odometry plus fused local state

**Project node:** wheel_odometry_node.  
**External configured node:** robot_localization/ekf_node, named /amr/ekf_filter_node.

The wheel node receives /amr/base/joint_states. For successive wheel angles:

```text
wheel distance = wheel radius × angle change
forward distance = (left distance + right distance) / 2
yaw change = (right distance - left distance) / wheel separation
```

It integrates this into /amr/localization/wheel_odometry. First data establishes a baseline; repeated/backward timestamps never create false motion.

The 30 Hz EKF fuses wheel velocity/yaw-rate with IMU yaw/yaw-rate, outputs /amr/localization/odometry, and solely owns odom -> base_footprint.

### amr_perception — validate then forward LiDAR clouds

**Nodes:** front_lidar_perception_node and rear_lidar_perception_node.

Both compile from lidar_pipeline_node.cpp with a different sensor ID. They validate PointCloud2 schema and discard invalid, future, stale (over 0.5 s), or non-monotonic data. Valid clouds keep their sensor frame and publish under /amr/perception/.../points.

This is navigation input quality control, not obstacle avoidance or a safety function.

### amr_slam — online mapping, configuration only

**External configured node:** slam_toolbox/async_slam_toolbox_node, named /amr/slam_toolbox.

It consumes the front adapted LaserScan and local TF, publishes /map, and owns map -> odom. Its map resolution is 5 cm. It has no velocity or navigation authority.

The factory launch uses the registered static map and AMCL instead of online
SLAM; standalone simulation is the normal path for experimenting with SLAM.

### amr_navigation — global planning and path smoothing, configuration only

**External configured nodes:** Nav2 planner_server, smoother_server, and lifecycle manager.

Navfn A* planning uses a global costmap built from the map, front/rear clouds,
and an inflation layer (clearance buffer). The collision-checked
SimpleSmoother then regularizes the path before it reaches the controller.
These nodes create paths, never velocity output.

### amr_mpc_controller — local path following, configuration only

**External configured nodes:** Nav2 controller_server and lifecycle manager.

The active controller is Nav2 Regulated Pure Pursuit (RPP). It follows the
smoothed path, regulates speed for curvature/cost/approach, performs collision
checking, and publishes a velocity request to /amr/mpc/cmd_vel. The request
uses encoder-derived wheel odometry for acceleration feedback while the EKF
continues to own the localization TF. It then enters the project-owned
arbitration boundary. The package name is retained for compatibility with the
Phase 11 interface.

Its rolling local costmap is 5 m × 5 m and uses both perception clouds. RPP
requests up to its configured 0.50 m/s target and 0.40 rad/s heading speed;
the project-owned arbitration boundary independently clamps these limits and
applies acceleration ramps.

### amr_control — key custom motion-protection code

**Node:** command_arbitration_node.

command_arbitration_node receives /amr/mpc/cmd_vel and publishes /amr/control/cmd_vel.

- It rejects non-finite/non-planar velocities.
- A source expires after 200 ms.
- At 20 Hz, output is capped at 0.50 m/s and 0.40 rad/s.
- Acceleration is ramped to 0.50 m/s² and 0.40 rad/s².
- Stale/invalid input produces zero and resets the ramp.

### amr_mission — narrow cancellable mission action

**Node:** mission_supervisor_node

It offers /amr/mission/navigate_to_pose (NavigateToPose). One mission can be active. It rejects goals outside map, invalid planar poses/quaternions, custom behavior trees, inactive state, and concurrent missions.

After acceptance, it calls Nav2 ComputePathToPose, sends the nonempty path to
SmoothPath with collision checking, and then sends the completed path to
FollowPath. Feedback/results are forwarded to the caller. Cancellation is
forwarded downstream; unavailable/rejecting/failing planner, smoother, or
controller aborts rather than guessing. This node never publishes velocity.

### amr_health — observation only

**Node:** health_supervisor_node

It observes base status and emits /amr/health/status at 10 Hz. It validates freshness (300 ms), boot/sequence identity, monotonic sequence/time, recognized state/reason pairs, and fault status.

Fresh, valid, ready base evidence makes HEALTHY. Missing/stale, invalid, backward-time, or not-ready evidence gives DEGRADED; a fresh base fault gives FAULT. It cannot command motion, change lifecycle state, or recover automatically.

## Recommended beginner reading order

1. src/amr_interfaces/msg/ — data contracts and named state/reason values.
2. src/amr_control/src/command_arbitration_node.cpp — parameters, limits, acceleration ramps.
3. src/amr_base_adapter/src/base_adapter_node.cpp — final ROS boundary.
4. src/amr_localization/src/wheel_odometry_node.cpp and include/amr_localization/diff_drive.hpp — movement math.
5. src/amr_mission/src/mission_supervisor_node.cpp — ROS actions and async callbacks.
6. src/amr_health/src/health_supervisor_node.cpp — defensive diagnostics.

For every lifecycle node, read constructor (parameters), on_configure() (connections), on_activate() (timers/publishers), callbacks, tick(), then on_deactivate().

## Build, run, inspect

These instructions are for the one-laptop ROS 2 Humble/Gazebo Harmonic
simulation. They do not establish physical-robot or functional-safety claims.
Use a new `GZ_PARTITION`, `ROS_DOMAIN_ID`, and log directory for each run so a
stale ROS/Gazebo graph cannot be mistaken for the current run. Repeat the same
setup values in every terminal.

The complete command sequence is maintained in
[SIMULATION_COMMANDS.md](SIMULATION_COMMANDS.md). The shortened sections
below explain which launch owns each part of the graph.

### 1. Build and run the focused checks

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --packages-select \
  amr_interfaces amr_description amr_simulation amr_localization \
  amr_perception amr_slam amr_navigation amr_mpc_controller amr_control \
  amr_mission amr_health amr_base_adapter amr_sensor_adapters amr_factory \
  amr_manipulation amr_bringup \
  --symlink-install
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
colcon test --packages-select \
  amr_description amr_mpc_controller amr_mission amr_manipulation amr_factory
colcon test-result --verbose
```

Do not start a product run if the focused tests report an error or failure.

### 2. Standalone simulation and teleoperation

Use this path for the base, sensors, localization, SLAM, Nav2, controller, and
command-arbitration stack without the factory or MoveIt. In terminal 1, choose
an unused run identity and start Gazebo:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export GZ_VERSION=harmonic
export RUN_ID=standalone_01
export GZ_PARTITION=amr_$RUN_ID
export ROS_DOMAIN_ID=124
export ROS_LOG_DIR="$PWD/.ros_logs/$RUN_ID"
mkdir -p "$ROS_LOG_DIR"
ros2 launch amr_simulation amr_simulation.launch.py headless:=true
```

Use `headless:=false` (or omit the argument) when Gazebo's GUI is required. In
terminal 2, after repeating the same setup and environment values:

```bash
ros2 run amr_control prototype_teleop.py
```

Use `W`, `S`, `A`, and `D` to move, `X` or Space to stop, and `Q` to quit.

### 3. Factory and Gate 6 product run

The factory launch starts the Harmonic factory world, AMCL, localization,
perception, Nav2 planning and collision-checked smoothing, RPP, command
arbitration, and the mission supervisor. It does not start MoveIt. Start
terminal 1 with the same setup pattern, using a fresh run identity and an
initial pose near pickup station A:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export GZ_VERSION=harmonic
export RUN_ID=gate6_product101_01
export GZ_PARTITION=amr_$RUN_ID
export ROS_DOMAIN_ID=125
export ROS_LOG_DIR="$PWD/.ros_logs/$RUN_ID"
mkdir -p "$ROS_LOG_DIR"
ros2 launch amr_factory factory_localization.launch.py \
  headless:=true initial_x:=2.4 initial_y:=3.0 initial_yaw:=0.0
```

Wait for the lifecycle managers to report active planner, smoother, and
controller nodes. Confirm that `/amr/smooth_path` is available before starting
the MoveIt server and mission commands.
In terminal 2, repeat the same setup and start the project-owned MoveIt server:

```bash
ros2 launch amr_manipulation move_group.launch.py
```

The accepted 1 kg Gate 6 path is the existing `product_id:=101` mass-stage
launch. The independent 3 kg and 5 kg tests use aliases that reset only the
selected product to its registered pickup station, preserve the AMR's current
pose, navigate to the selected pickup dock, and then start the same mass-stage
implementation. They refuse to reset if the arm is attached, deployed,
moving, faulted, or not at empty stow.

For a product-101 evidence run, terminal 3 can record the required clock, TF,
localization, ground truth, plans, action feedback/status, odometry, contacts,
joint states, and all three velocity-command paths:

```bash
ros2 bag record --include-hidden-topics \
  -o "$ROS_LOG_DIR/product101_evidence" \
  /clock /tf /tf_static \
  /amr/amcl_pose /amr/localization/odometry /amr/localization/wheel_odometry \
  /amr/simulation/ground_truth/pose /amr/simulation/base/odometry \
  /amr/base/odometry_raw \
  /amr/plan /amr/plan_smoothed /amr/received_global_plan \
  /amr/mission/navigate_to_pose/_action/feedback \
  /amr/mission/navigate_to_pose/_action/status \
  /amr/compute_path_to_pose/_action/feedback \
  /amr/compute_path_to_pose/_action/status \
  /amr/smooth_path/_action/feedback /amr/smooth_path/_action/status \
  /amr/follow_path/_action/feedback \
  /amr/follow_path/_action/status \
  /amr/base/joint_states /amr/simulation/base/joint_states \
  /amr/simulation/contacts/left_finger /amr/simulation/contacts/right_finger \
  /amr/simulation/base/cmd_vel /amr/control/cmd_vel /amr/mpc/cmd_vel \
  /model/product_a/pose
```

In terminal 4, after repeating the same setup, run the selected mass stage. The
1 kg acceptance command remains unchanged and should not be rerun as part of
the independent 3 kg or 5 kg tests:

```bash
ros2 launch amr_manipulation gate6_mass_stage.launch.py product_id:=101
```

Choose one independent test when the factory and MoveIt terminals are already
running:

```bash
# 3 kg, product B/tag 102:
ros2 launch amr_manipulation gate6_3kg_test.launch.py

# 5 kg, product C/tag 103:
# ros2 launch amr_manipulation gate6_5kg_test.launch.py
```

The stage is fail-closed. Stop at the first failed boundary and retain its
logs/bag; do not retry the complete product run or tune another parameter
without a new decision. Run only one product test at a time, and review a
failed 3 kg run before starting the 5 kg test.

### 4. Read-only inspection and shutdown

Useful checks while the graph is running:

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /amr/control/cmd_vel
ros2 topic echo /amr/manipulation/status
ros2 topic echo /amr/simulation/ground_truth/pose
ros2 topic info --verbose /amr/control/cmd_vel
ros2 lifecycle get /amr/controller_server
ros2 lifecycle get /amr/command_arbitration_node
ros2 run tf2_ros tf2_echo map base_footprint
```

After a recorded run, inspect the bag without starting another runtime:

```bash
ros2 bag info "$ROS_LOG_DIR/product101_evidence"
```

Stop processes in dependency order with `Ctrl-C`: stop the stage first, then
the bag recorder and wait for it to finish writing, then MoveIt, then the
factory/Gazebo launch. Preserve the run directory for evidence. Confirm that no
ROS, Gazebo, MoveIt, stage, or rosbag process remains before starting another
run.

Run the complete installed suite when a broader regression check is needed:

```bash
colcon test
colcon test-result --verbose
```

## Scope and evidence

Claims apply only to the one-laptop simulation. Do not treat simulated timeouts, speeds, kinematics, or covariance as physical-machine specifications. Hardware, external fieldbus, procurement, industrial deployment, and automatic recovery are excluded.

For the design and validation history, read docs/PHASE_0_SOFTWARE_BASELINE.md, docs/PHASE_4_ROS2_WORKSPACE.md, docs/PHASE_8_PERCEPTION.md, docs/PHASE_9_SLAM.md, docs/PHASE_10_NAVIGATION.md, docs/PHASE_11_CONTROL.md, docs/PHASE_13_HEALTH.md, and docs/CORRECTIVE_ACTION_TEST_REPORT.md.

When changing code, begin with the message/topic owner and phase contract. In ROS 2, many integration failures come from a topic name, frame, QoS policy, lifecycle state, or timeout rather than a C++ compile error.
