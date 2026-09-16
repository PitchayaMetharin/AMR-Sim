# Beginner’s guide to this AMR ROS 2 project

This repository is a laptop-only simulation of an autonomous mobile robot (AMR). It uses ROS 2 Humble, C++17, Gazebo Harmonic, Nav2, robot_localization, and SLAM Toolbox. The current navigation controller is Nav2 Regulated Pure Pursuit (RPP); the compatibility package/topic names remain `amr_mpc_controller` and `/amr/mpc/cmd_vel`. It is not a physical robot controller, fieldbus integration, or functional-safety system.

The current accepted factory scope is Product 101 (1 kg) and Product 102 (3 kg)
through the autonomous factory-cycle entry point. Product 103 (5 kg), Gate 7,
hardware, and functional-safety acceptance are outside scope.

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
| World | A Gazebo SDF environment. For portable exploration it is a trusted absolute local SDF 1.9 file, not a saved SLAM map. |
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

The legacy `amr_simulation.launch.py` entry point uses the registered simple
world and is useful for manual smoke tests. The
`portable_exploration.launch.py` entry point accepts one trusted absolute local
SDF 1.9 world, validates it before starting processes, derives the
world-qualified Gazebo bridge names, and starts the staged autonomous
exploration graph. It spawns the arm empty with
`factory_attachment=false`, runs a one-shot portable empty-stow authority, and
keeps motion denied until fresh stow, base, map, TF, costmap, lifecycle, and
action evidence is present. A failed validation or required process shuts the
run down fail-closed.

`aws_warehouse_exploration.launch.py` is a thin preset around that portable
launch. Its world contains pinned OpenRobotics Fuel references; model bundles
are not vendored. See [SIMULATION_COMMANDS.md](SIMULATION_COMMANDS.md) and the
[Fuel attribution record](../src/amr_simulation/assets/AWS_WAREHOUSE_FUEL_ATTRIBUTION.md)
for cache, network, and ownership details.

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
SLAM. Standalone simulation and portable exploration use online SLAM instead;
each launch creates a fresh in-memory `/map`, and no saved map is loaded or
overwritten automatically. Portable exploration intentionally has no AMCL or
`nav2_map_server`.

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

### amr_exploration — bounded frontier exploration

**Node:** `frontier_explorer.py`

This Phase 15 node selects frontier goals from `/map` and sends them through
the existing `/amr/mission/navigate_to_pose` action. It never publishes base
velocity. It requires fresh map, costmap, TF, and command-authority evidence;
fully obstructed clusters are skipped without consuming a motion token. A
fault, confirmed cancellation, or unsafe evidence stops the run fail-closed.
The current portable runtime has one accepted non-faulted safe terminal
outcome, `INCOMPLETE`; this does not establish human map-quality approval.
The portable launch autostarts it after staged readiness by default; set
`auto_start_exploration:=false` only when you intentionally want to call its
start service after readiness. Human map-quality review and canonical-map
promotion remain separate decisions.

The portable stow authority is the sole `/amr/manipulation/status` publisher
in that runtime. Do not combine it with the factory or Gate 6 manipulation
status owner in the same ROS graph.

### amr_manipulation — cycle and Gate 6 manipulation

The cycle adapter owns the canonical manipulation status and the
`/amr/manipulation/execute_product_cycle` action. It coordinates the registered
product preparation, navigation handoff, MoveIt/gripper work, attachment and
detachment proof, and empty-stow proof. Missing or stale proof blocks base
motion and preserves a held-product fault rather than attempting recovery.

### amr_factory — factory orchestration and mapping tools

The factory supervisor owns station-selectable transport and sequence actions,
the stop/cancel/home/status boundaries, and registry-derived Product 101/102
mapping. `factory_autonomous.launch.py` is the canonical autonomous launch;
`factory_demo.launch.py` is legacy/optional. The mapping CLI and acceptance
tools write only run-specific artifacts and never replace the canonical map.

## Beginner learning path: read the project in this order

Do not begin by opening every file in `src/`. This is a working ROS 2 system,
so the easiest way to understand it is to follow one piece of information from
its definition, through its producer, to its consumers. Use the lessons below
in order. For each lesson, read the named files, run the small check, and do
not move on until you can explain the checkpoint in your own words.

### Lesson 0 — learn the vocabulary and the safety story

Read these first:

1. `docs/ROS2_BEGINNER_PROJECT_GUIDE.md` — this guide and its two diagrams.
2. `src/README.md` — the package ownership table and scope boundaries.
3. `docs/SIMULATION_COMMANDS.md` — the supported launch commands and what is
   accepted versus still unverified.

Before reading C++, learn these words: node, topic, publisher, subscriber,
service, action, lifecycle node, parameter, message, QoS, Gazebo world, URDF,
SDF, and TF. In this project, remember the difference between these two
things:

- `world` is the Gazebo environment, such as
  `src/amr_simulation/worlds/amr_world.sdf`.
- `/map` is the occupancy grid built by SLAM during a run. Portable launches
  create it in memory; they do not automatically load a saved map.

Checkpoint: explain why a node can publish a velocity request without the
robot moving. The answer should include command arbitration, the base adapter,
and the Gazebo watchdog.

### Lesson 1 — start with the contracts, not the implementations

Read the interface definitions in this order:

1. `src/amr_interfaces/msg/BaseStatus.msg` — what the base reports about
   readiness, freshness, and faults.
2. `src/amr_interfaces/msg/HealthStatus.msg` — observation-only health output.
3. `src/amr_interfaces/msg/ManipulatorStatus.msg` — empty-stow and held-product
   authority states.
4. `src/amr_interfaces/msg/FactoryStatus.msg` — factory orchestration state.
5. `src/amr_interfaces/srv/SetOperationMode.srv` — a request/response service.
6. `src/amr_interfaces/action/ExecuteProductCycle.action`, then
   `ManipulateProduct.action`, `NavigateStation.action`, `RunSequence.action`,
   and `TransportProduct.action` — long-running goals, feedback, results, and
   cancellation.
7. `src/amr_interfaces/include/amr_interfaces/qos_profiles.hpp` — why command
   and authority topics use different delivery rules from simulated sensors.

Then read the tests that describe the contract:

- `src/amr_interfaces/test/test_interface_schema.py`
- `src/amr_interfaces/test/test_fail_closed_defaults.cpp`
- `src/amr_interfaces/test/test_qos_profiles.cpp`

Checkpoint: for any status message, identify its timestamp, sequence or boot
identity, validity, and fault fields. Explain why a consumer must reject stale,
replayed, malformed, or backward-time evidence instead of guessing.

### Lesson 2 — understand the robot before the nodes

Read the robot model from outside in:

1. `src/amr_description/urdf/amr.urdf.xacro` — the mobile base, frames,
   wheels, IMU, and LiDAR links.
2. `src/amr_description/urdf/phase14_mobile_manipulator.urdf.xacro` — the
   arm-equipped robot used by the portable and factory paths.
3. `src/amr_description/config/phase14_mobile_manipulator_controllers.yaml` —
   the simulated joint controllers.
4. `src/amr_description/config/phase14_mobile_manipulator.srdf` — MoveIt
   groups and planning relationships.
5. `src/amr_simulation/worlds/amr_world.sdf` — a simple Gazebo environment.
6. `src/amr_simulation/src/command_watchdog_system.cpp` and
   `src/amr_simulation/include/amr_simulation/command_watchdog.hpp` — the
   native Gazebo stop condition.

Do not spend time on mesh geometry yet. The files under
`src/amr_description/meshes/` describe appearance and collision shapes; they
are useful later, but they do not explain the ROS data flow.

Checkpoint: draw the TF chain `map -> odom -> base_footprint -> sensor frame`
and point to the file or node responsible for each part. Also explain what
happens if the base receives no command for 200 ms.

### Lesson 3 — learn how the graph is assembled

Read launch files only after you know the contracts and robot model:

1. `src/amr_bringup/config/interface_ownership.yaml` — intended topic
   publishers and ownership boundaries.
2. `src/amr_bringup/config/qos_profiles.yaml` — QoS intent.
3. `src/amr_bringup/config/runtime_defaults.yaml` — shared runtime defaults.
4. `src/amr_bringup/launch/amr_system.launch.py` — startup of the health
   supervisor and bring-up conventions.
5. `src/amr_simulation/launch/amr_simulation.launch.py` — the fixed, legacy
   smoke-test graph.
6. `src/amr_simulation/launch/portable_exploration.launch.py` — the validated,
   world-agnostic graph. Read its validation helpers first, then its staged
   process graph.
7. `src/amr_simulation/launch/aws_warehouse_exploration.launch.py` — the thin
   AWS preset that includes the portable launch once.

When reading a launch file, ask four questions: what arguments are public,
which process starts first, what event releases the next process, and what
event shuts the graph down? The portable launch is intentionally more
defensive: it validates the SDF, spawn pose, local resources, and required
Gazebo systems before expansion; it also shuts down when a required process
exits.

Checkpoint: state the difference between the legacy fixed launch, the portable
local-world launch, and the AWS preset. Also explain why the AWS preset is not
a second independent implementation of the runtime graph.

### Lesson 4 — trace a velocity request through the safety gates

Read these in runtime order, from the last ROS decision to the simulated
plant:

1. `src/amr_control/config/control.yaml` — limits, timeouts, and ramp values.
2. `src/amr_control/src/command_arbitration_node.cpp` — rejects malformed
   commands, expires stale input, clamps speed, ramps acceleration, and
   publishes zero when authority is not proven.
3. `src/amr_control/launch/amr_control.launch.py` — how the node is started.
4. `src/amr_base_adapter/src/base_adapter_node.cpp` — the final ROS boundary
   before Gazebo; it validates again and forwards only fresh planar commands.
5. `src/amr_base_adapter/test/test_base_adapter_contract.py` and
   `src/amr_base_adapter/test/test_base_adapter_node.cpp` — examples of the
   boundary tests.
6. Re-read `src/amr_simulation/src/command_watchdog_system.cpp` — the
   independent native watchdog.

Do not start with the Nav2 controller. First understand why a bad command is
stopped even if it came from a trusted-looking node. The project uses several
independent gates because a single publisher or process must not have total
motion authority.

Checkpoint: follow `/amr/mpc/cmd_vel` to `/amr/control/cmd_vel` and then to the
Gazebo command topic. List at least three conditions that cause zero output.

### Lesson 5 — learn the robot's measurements and local motion estimate

Read the sensor path in this order:

1. `src/amr_sensor_adapters/src/lidar_adapter_node.cpp` and
   `imu_adapter_node.cpp` — stable ROS names over Gazebo bridge topics.
2. `src/amr_base_adapter/src/base_adapter_node.cpp` — joint states and raw
   odometry entering the ROS graph.
3. `src/amr_localization/include/amr_localization/diff_drive.hpp` — the small,
   testable differential-drive equations.
4. `src/amr_localization/src/wheel_odometry_node.cpp` — timestamps, wheel
   deltas, and wheel odometry publication.
5. `src/amr_localization/config/ekf.yaml` — robot_localization configuration.
6. `src/amr_localization/launch/amr_localization.launch.py` — how wheel
   odometry and the EKF are composed.
7. `src/amr_localization/test/test_diff_drive.cpp`,
   `test_wheel_odometry_configuration.cpp`, and
   `test_localization_contract.py` — the expected edge cases.

Checkpoint: explain why the first wheel sample establishes a baseline instead
of moving the robot, why backward timestamps are rejected, and why the EKF
alone owns `odom -> base_footprint`.

### Lesson 6 — understand perception validation before SLAM

Read:

1. `src/amr_perception/include/amr_perception/point_cloud_validation.hpp` —
   the validation rules.
2. `src/amr_perception/src/lidar_pipeline_node.cpp` — the lifecycle adapter
   that validates and forwards front/rear point clouds.
3. `src/amr_perception/launch/amr_perception.launch.py` — the two configured
   sensor instances.
4. `src/amr_perception/test/test_point_cloud_validation.cpp`,
   `test_lidar_pipeline_configuration.cpp`, and
   `test_perception_contract.py` — stale, future, malformed, and
   non-monotonic data cases.

Checkpoint: explain why perception can improve navigation without owning
velocity or personnel safety, and name the conditions that make a cloud
unusable.

### Lesson 7 — learn SLAM and TF ownership

Read:

1. `src/amr_slam/config/mapper.yaml` — SLAM Toolbox parameters.
2. `src/amr_slam/launch/amr_slam.launch.py` — the configured external SLAM
   node.
3. `src/amr_slam/test/test_slam_contract.py` — the map and ownership
   expectations.
4. `src/amr_localization/config/ekf.yaml` again — compare the EKF's TF edge
   with SLAM's TF edge.

SLAM Toolbox owns `map -> odom`; the EKF owns `odom -> base_footprint`. Do not
solve a TF problem by adding a second publisher. In portable exploration,
SLAM creates a new `/map` during every launch. In the factory launch, AMCL and
the registered static map replace the online SLAM path.

Checkpoint: explain why two publishers for the same TF edge are dangerous and
why a saved map is not automatically part of a portable exploration launch.

### Lesson 8 — learn planning and path following separately

Read planning first:

1. `src/amr_navigation/config/planner.yaml` — global planner, smoother, and
   costmap settings.
2. `src/amr_navigation/launch/amr_navigation.launch.py` — planner, smoother,
   costmaps, and lifecycle manager.
3. `src/amr_navigation/test/test_navigation_contract.py` — configuration and
   ownership checks.

Then read local control:

4. `src/amr_mpc_controller/config/controller.yaml` — the active Regulated
   Pure Pursuit controller settings.
5. `src/amr_mpc_controller/launch/amr_mpc_controller.launch.py` — the Nav2
   controller and lifecycle manager.
6. `src/amr_mpc_controller/test/test_mpc_controller_contract.py` — the
   compatibility names and safety assumptions.

Remember the separation: planners create paths, the smoother checks and
regularizes paths, RPP follows a path and requests velocity, and
`command_arbitration_node` remains the project-owned motion boundary.

Checkpoint: describe the order `map -> global costmap -> planner -> smoother
-> controller -> velocity request`, and identify which component is allowed to
publish the velocity request.

### Lesson 9 — learn one ROS action end to end

Read:

1. `src/amr_mission/include/amr_mission/goal_validation.hpp` — input
   validation without ROS transport details.
2. `src/amr_mission/src/mission_supervisor_node.cpp` — action server,
   asynchronous Nav2 calls, feedback, cancellation, and result handling.
3. `src/amr_mission/launch/amr_mission.launch.py` — parameters and startup.
4. `src/amr_mission/test/test_goal_validation.cpp`,
   `test_mission_supervisor_behavior.cpp`, and `test_mission_contract.py` —
   invalid goals, cancellation, and downstream failure behavior.

The mission node does not publish velocity. It validates a goal, asks Nav2 for
a path, asks the smoother to check it, asks the controller to follow it, and
forwards the result. If a downstream action is unavailable, rejected, or
fails, the mission must abort rather than inventing success.

Checkpoint: explain the difference between a topic command and an action goal,
and trace what happens when the operator cancels a mission.

### Lesson 10 — learn health and fail-closed observation

Read:

1. `src/amr_health/src/health_supervisor_node.cpp` — freshness, boot ID,
   sequence, recognized state/reason, and fault checks.
2. `src/amr_health/launch/amr_health.launch.py` — startup and parameters.
3. `src/amr_health/test/test_health_configuration.cpp` and
   `test_health_contract.py` — the defensive cases.

Health is an observer. It can report `HEALTHY`, `DEGRADED`, or `FAULT`, but it
cannot command motion, change lifecycle state, or recover a failed process.

Checkpoint: explain why “no message received” is not the same as “healthy,”
and why health must not secretly become another command authority.

### Lesson 11 — learn portable exploration last among the core runtime paths

Now read the exploration behavior:

1. `src/amr_exploration/config/frontier_explorer.yaml` — bounded exploration
   parameters.
2. `src/amr_exploration/scripts/frontier_algorithm.py` — pure frontier
   selection logic; start here because it is easier to test.
3. `src/amr_exploration/scripts/frontier_explorer.py` — lifecycle, readiness,
   mission goals, cancellation, and terminal states.
4. `src/amr_exploration/launch/frontier_explorer.launch.py` — normal launch
   wiring.
5. `src/amr_exploration/test/test_frontier_algorithm.py`,
   `test_frontier_contract.py`, and `test_frontier_lifecycle.py` — behavior
   and lifecycle expectations.
6. `src/amr_simulation/scripts/portable_stow_authority.py` — one-shot arm
   stow proof and the portable `/amr/manipulation/status` owner.
7. `src/amr_simulation/scripts/portable_exploration_readiness.py` — staged
   readiness and the evidence required before Explorer is released.
8. `src/amr_simulation/test/test_portable_stow_authority.py` and
   `test_portable_exploration_readiness.py` — terminal, freshness, and
   fail-closed behavior.
9. Re-read `src/amr_simulation/launch/portable_exploration.launch.py` and
   `test_portable_exploration_launch.py` now that its child processes are
   familiar.

The causal order is adapters and authority, SLAM/map, planner and smoother,
controller, mission, then Explorer. Readiness is not a timer that eventually
declares success: it waits for live evidence, reports unmet conditions, and
fails immediately on process exit, explicit fault, shutdown, or user stop.

Checkpoint: explain why Explorer is released last and why a fresh empty-stow
proof is required before it can cause navigation.

### Lesson 12 — learn manipulation and factory orchestration after navigation

The factory path combines everything above with products, stations, MoveIt,
attachments, and cancellation. Read it last:

1. `src/amr_factory/config/products.yaml` and `stations.yaml` — data-driven
   product and station definitions.
2. `src/amr_factory/scripts/factory_registry.py` — how those definitions are
   loaded and validated.
3. `src/amr_factory/launch/factory_autonomous.launch.py` — the accepted factory
   graph and its AMCL/static-map path.
4. `src/amr_factory/src/factory_supervisor_node.cpp` — factory actions,
   stop/cancel/home, and state ownership.
5. `src/amr_manipulation/include/amr_manipulation/attachment_gate.hpp` and
   `src/amr_manipulation/src/attachment_gate.cpp` — attachment proof rules.
6. `src/amr_manipulation/scripts/cycle_manipulation_supervisor.py` and
   `src/amr_manipulation/src/manipulation_supervisor_node.cpp` — product-cycle
   coordination.
7. `src/amr_manipulation/launch/move_group.launch.py` and the MoveIt YAML files
   — planning configuration, after you understand the arm URDF/SRDF.
8. `src/amr_factory/scripts/factory_cli.py` — the operator boundary.
9. The matching tests under `src/amr_factory/test/` and
   `src/amr_manipulation/test/` before attempting a product run.

Do not begin with the historical Gate 6 mass-stage launch files or the mesh
assets. They are specialized evidence and geometry, not the foundation of the
ROS graph. Product 101 and Product 102 are the accepted factory scope;
Product 103, hardware, and functional-safety claims remain outside scope.

### How to read any C++ or Python node

Use the same pass every time:

1. Find the executable entry point and node name.
2. Read parameter declarations and defaults.
3. List publishers, subscribers, services, and actions with their exact names
   and message types.
4. Find the timer or callback that changes state.
5. Mark every freshness, timeout, validity, lifecycle, and fault check.
6. Find the success path, cancellation path, process-failure path, and
   shutdown path.
7. Read the matching test and identify which invariant it protects.

For a lifecycle node, inspect the constructor, `on_configure()`,
`on_activate()`, callbacks and timers, `on_deactivate()`, and `on_cleanup()`
in that order. Keep a one-page notebook with four columns: input, validation,
output, and failure action. That notebook will make the ownership boundaries
and fail-closed behavior visible much faster than reading files alphabetically.

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
  amr_mission amr_health amr_exploration amr_base_adapter \
  amr_sensor_adapters amr_factory \
  amr_manipulation amr_bringup \
  --symlink-install
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
colcon test --packages-select \
  amr_description amr_mpc_controller amr_mission amr_exploration \
  amr_manipulation amr_factory
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

### 3. Portable world-based exploration

Use the portable launch for autonomous frontier exploration in a trusted local
Gazebo SDF environment. It creates a fresh in-memory SLAM map, so `world` is
the environment—not a previously saved map. The validator requires an
absolute local SDF 1.9 path and rejects unsafe or unresolved resources before
Gazebo starts. Omit `resource_paths` when the world uses the workspace's
registered assets; otherwise provide a colon-separated list of existing
absolute local model directories.

In terminal 1, use a fresh run identity:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export GZ_VERSION=harmonic
export RUN_ID=portable_simple_01
export GZ_PARTITION=amr_$RUN_ID
export ROS_DOMAIN_ID=126
export ROS_LOG_DIR="$PWD/.ros_logs/$RUN_ID"
mkdir -p "$ROS_LOG_DIR"
ros2 launch amr_simulation portable_exploration.launch.py \
  world:="$PWD/src/amr_simulation/worlds/amr_world.sdf" \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.12 initial_yaw:=0.0 \
  headless:=true rviz:=false auto_start_exploration:=true
```

The launch stages adapters and the portable empty-stow authority before SLAM,
planning, control, mission, and Explorer. With the default
`auto_start_exploration:=true`, no start-service call is needed. To start
intentionally after readiness, set it to `false` and then call:

```bash
ros2 service call /amr/exploration/start std_srvs/srv/Trigger {}
```

Do not run factory or Gate 6 manipulation processes in the same ROS graph:
portable exploration owns the sole `/amr/manipulation/status` publisher for
that run. Stop exploration through its cancellation boundary and wait for a
non-faulted terminal `COMPLETE` or safe `INCOMPLETE` state before saving a
run-specific candidate. The full save/validate procedure is in
[SIMULATION_COMMANDS.md](SIMULATION_COMMANDS.md).

For the packaged AWS warehouse preset, use the canonical pinned Fuel world:

```bash
ros2 launch amr_simulation aws_warehouse_exploration.launch.py \
  headless:=false rviz:=true auto_start_exploration:=true
```

The first AWS run needs network access to download uncached model revisions;
the exact ownership, revisions, attribution, and cache rules are recorded in
[`AWS_WAREHOUSE_FUEL_ATTRIBUTION.md`](../src/amr_simulation/assets/AWS_WAREHOUSE_FUEL_ATTRIBUTION.md).

### 4. Approved autonomous factory cycle

The canonical autonomous launch starts the Harmonic factory world, AMCL,
localization, perception, Nav2 planning and collision-checked smoothing, RPP,
command arbitration, the mission supervisor, the cycle adapter, and the
factory supervisor. MoveIt remains a separately started process. Start
terminal 1 with a fresh run identity and explicit native-attachment mode:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export GZ_VERSION=harmonic
export RUN_ID=amr_autonomous_factory_01
export GZ_PARTITION=amr_$RUN_ID
export ROS_DOMAIN_ID=125
export ROS_LOG_DIR="$PWD/.ros_logs/$RUN_ID"
mkdir -p "$ROS_LOG_DIR"
ros2 launch amr_factory factory_autonomous.launch.py \
  headless:=true control_mode:=autonomous factory_attachment:=true \
  require_hardware_rendering:=true initial_x:=-4.5 initial_y:=0.0 initial_yaw:=0.0
```

Wait for the lifecycle managers to report active planner, smoother, and
controller nodes. Confirm that `/amr/smooth_path` is available before starting
the MoveIt server and mission commands.
In terminal 2, repeat the same setup and start the project-owned MoveIt server:

```bash
ros2 launch amr_manipulation move_group.launch.py
```

Product 101 and Product 102 are the only accepted autonomous products. Use the
registry-backed CLI after MoveIt and readiness checks pass:

```bash
ros2 run amr_factory factory_cli.py list
ros2 run amr_factory factory_cli.py mode autonomous
ros2 run amr_factory factory_cli.py send pickup_a dispatch --timeout 240
ros2 run amr_factory factory_cli.py send pickup_b dispatch --timeout 240
ros2 run amr_factory factory_cli.py loop pickup_a pickup_b --cycles 1 --finish stay
ros2 run amr_factory factory_cli.py status
```

Use `factory_cli.py stop` for graceful stop, `cancel` for immediate
cooperative cancellation, and `go home` only from an idle safe empty-stowed
state. Product 103/5 kg must remain disabled and must not be started.

For a current autonomous evidence run, use the bounded recorder and gate
commands in `SIMULATION_COMMANDS.md`; do not reuse the historical 1/3/5 kg
mass-stage procedure from older records.

The legacy/manual Gate 6 mass-stage launch remains a historical diagnostic
entry point, not the current autonomous acceptance path. It is not required for
Product 101/102 factory-cycle acceptance.

For a legacy product-101 evidence run only, terminal 3 can record the required
clock, TF, localization, ground truth, plans, action feedback/status, odometry,
contacts, joint states, and all three velocity-command paths:

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

### 5. Read-only inspection and shutdown

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
