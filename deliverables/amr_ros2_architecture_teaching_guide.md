# AMR ROS 2 Architecture Teaching Guide

This is the standalone teaching and presenter guide for
amr_ros2_architecture_presentation.pptx. It is intended for a presenter, a
new engineer, or another agent that needs to understand the current ROS 2
architecture and explain it accurately.

**Guide date:** 2026-10-04  
**Workspace:** /home/pete/amr_ws  
**Scope:** ROS 2 Humble and Gazebo Harmonic laptop simulation  
**Hardware claim:** none  
**Functional-safety claim:** none

The current source tree and configuration define implementation behavior. The
latest dated SESSION_HANDOFF.md defines runtime status. Older reports and
older presentation files are historical unless the current source and latest
handoff confirm them.

## Read this status warning first

The latest handoff says that functional recovery is **stopped after Native42
because the mandatory Product 102 unchanged analyzer failed**. The A cycle, B
cycle, home, status, and final factory proof commands returned exit 0, but the
strict Product 102 analyzer returned exit 1. The recorded evidence is missing
bilateral right contact/product-pose evidence and empty base-command evidence;
the cause remains unknown. No post-failure diagnosis, source edit, tuning,
retry, or new runtime was started.

This means:

- source checks and unit tests are separate from runtime acceptance;
- the functional milestone is not currently complete;
- stability tuning remains later work;
- this guide does not authorize resuming the stopped simulation;
- Product 103, Gate 7, physical hardware, and functional-safety claims remain
  outside scope.

The presentation was created before the latest Native42 stop. Its slide 19
contains an older Native41 status sentence. Use this guide and the latest
SESSION_HANDOFF.md for current status.

## The one-minute explanation

This is a simulated autonomous mobile robot with a KUKA arm and gripper.
Gazebo is the plant: it produces clock, wheel, IMU, LiDAR, camera, and contact
data and moves the simulated robot when an accepted command reaches it.

ROS 2 adapters give those raw signals stable project-owned topics. Wheel
odometry and IMU measurements are combined by the robot_localization EKF to
produce a local estimate and the odom -> base_footprint transform. Mapping
mode uses SLAM Toolbox to create /map and publish map -> odom. Factory mode
uses a saved map and AMCL for the global transform.

Nav2 uses the map, local pose, and obstacle costmaps to compute a path, smooth
it, and follow it with Regulated Pure Pursuit. RPP requests a velocity; it
does not directly drive the plant. Command arbitration checks freshness,
validity, limits, acceleration, and authority. The base adapter and Gazebo
watchdog provide later command boundaries.

MoveIt plans arm and gripper trajectories. Mission, manipulation, and factory
supervisors coordinate long-running actions, cancellation, attachment proof,
dispatch, and home. Each active mode has one owner for every important TF edge
and command boundary.

~~~text
Gazebo sensors/joints
        |
        v
stable adapters
        |
        +--> wheel odometry + IMU --> EKF --> odom -> base_footprint
        +--> front LaserScan --> SLAM Toolbox --> /map + map -> odom
        +--> front/rear clouds --> validation --> Nav2 costmap obstacles

goal --> Nav2 planner --> smoother --> RPP
                                  |
                                  v
                          /amr/mpc/cmd_vel
                                  |
                                  v
                      command_arbitration_node
                                  |
                                  v
                          /amr/control/cmd_vel
                                  |
                                  v
                          base_adapter_node
                                  |
                                  v
                            Gazebo plant

MoveIt --> arm/gripper joint trajectories
~~~

## How to teach each slide

For each slide: state the purpose, trace the inputs and outputs, name the
owner of the key decision, explain the fail-closed behavior, then ask:

> What is the input, who owns the decision, what is the output, and what
> happens if the evidence becomes stale?

## Slide 1 — How the simulated mobile manipulator works

**Purpose:** Introduce a complete ROS 2 software architecture.

**Explain:** The system is a ROS 2 Humble and Gazebo Harmonic simulation of an
autonomous mobile robot with a KUKA KR6 R900-2 arm, gripper, two LiDARs, IMU,
and product camera. The presentation follows sense, estimate, map/localize,
plan, follow, arbitrate, move, manipulate, and prove state.

**Emphasize:** It is a simulation and source-contract explanation. It makes no
physical-robot, fieldbus, industrial-deployment, or functional-safety claim.

**Source basis:** AGENTS.md, SESSION_HANDOFF.md, src/README.md, and the
composite robot Xacro.

## Slide 2 — Scope and operating modes

**Purpose:** Explain why the graph has several modes and why ownership changes.

The primary product set is the 17 packages under src/: amr_interfaces,
amr_bringup, amr_base_adapter, amr_sensor_adapters, amr_description,
amr_simulation, amr_localization, amr_perception, amr_slam, amr_navigation,
amr_mission, amr_mpc_controller, amr_control, amr_health, amr_exploration,
amr_factory, and amr_manipulation. colcon may discover additional legacy,
evidence, and vendored packages.

| Mode | Owner of map -> odom |
| --- | --- |
| Mapping/portable exploration | SLAM Toolbox |
| Factory localization | AMCL |
| Local motion in either mode | EKF owns odom -> base_footprint |

Portable exploration has a one-shot stow authority for
/amr/manipulation/status. Do not run it together with the factory/Gate 6
manipulation-status owner. Do not run SLAM and AMCL as simultaneous global TF
owners.

**Key message:** Modes are ownership contracts, not just launch options.

## Slide 3 — End-to-end ROS 2 workflow

**Purpose:** Show the full causal flow before discussing subsystems.

Read the upper path as Gazebo -> adapters -> estimation/perception -> map and
costmaps -> planner -> smoother -> RPP -> arbitration -> base adapter -> plant.
Read the lower branch as mission/factory actions and MoveIt arm actions.

The diagram is simplified: these streams run in parallel in the actual graph.
Nav2 creates a velocity request, but project-owned arbitration and the base
boundary decide whether it becomes plant motion.

**Source basis:** interface_ownership.yaml, planner.yaml,
controller.yaml, and command_arbitration_node.cpp.

## Slide 4 — AMR capabilities and operating loop

**Purpose:** Explain the robot as a coordinated operating loop.

1. Sense with LiDAR, IMU, camera, and simulated plant state.
2. Estimate local motion.
3. Build or use a map and localize.
4. Plan and follow a base path.
5. Move the arm and gripper when a product action requires it.
6. Verify freshness, collision, cancellation, attachment, detachment, and
   terminal state.
7. Stop or fail closed when required proof is missing.

The documented product scope is Product 101 at 1 kg and Product 102 at 3 kg.
Product 103 at 5 kg, Gate 7, hardware, and functional-safety acceptance are
outside scope. In-scope does not mean currently accepted: the latest handoff
still records an unresolved strict Product 102 evidence failure.

**Key message:** Reaching a pose is only one part of a successful product
cycle.

## Slide 5 — Robot model: URDF and Xacro

**Purpose:** Connect the robot model to TF, Gazebo, sensors, MoveIt, and
collision geometry.

amr.urdf.xacro defines base_footprint, base_link, drive wheels, passive
casters, IMU, front/rear LiDAR links, Gazebo sensors, differential drive, and
the command watchdog.

phase14_mobile_manipulator.urdf.xacro includes the base and adds the KUKA
macro, arm mount, two-finger gripper, gripper_tcp, product camera, optional
loaded product, optional contact/attachment systems, and gz_ros2_control
interfaces.

The SRDF defines MoveIt groups for manipulator and gripper, the product
end-effector relationship, and the stowed arm state.

The description provides fixed relationships such as
base_footprint -> base_link -> front_lidar_link. Runtime nodes provide
map -> odom and odom -> base_footprint.

**Key message:** URDF/Xacro is the shared mechanical and frame contract.

## Slide 6 — Sensors and ROS interfaces

| Signal | Topic | Type | Consumer |
| --- | --- | --- | --- |
| Front scan | /amr/sensors/front_lidar/scan | LaserScan | SLAM Toolbox |
| Rear scan | /amr/sensors/rear_lidar/scan | LaserScan | Egress/observation evidence |
| Front cloud | /amr/perception/front_lidar/points | PointCloud2 | Nav2 costmaps |
| Rear cloud | /amr/perception/rear_lidar/points | PointCloud2 | Nav2 costmaps |
| IMU | /amr/sensors/imu/data_raw | Imu | EKF |
| Raw odometry | /amr/base/odometry_raw | Odometry | Base/manipulation evidence |
| Joint states | /amr/base/joint_states | JointState | Wheel odometry and MoveIt |
| Fused odometry | /amr/localization/odometry | Odometry | Navigation/local state |
| Product camera | /amr/sensors/product_camera/* | Image/CameraInfo | Product perception |

Adapters separate simulator-facing topics from stable project-facing names and
preserve source frames. The point-cloud pipeline rejects malformed, future,
stale, or non-monotonic samples.

Sensor data uses Best Effort/Volatile intent. Commands and authority use
Reliable delivery with short deadlines or lifespans. A blank RViz display may
be a QoS mismatch rather than a missing sensor.

**Key message:** Adapters stabilize interfaces; they do not estimate pose,
choose paths, or command the robot.

## Slide 7 — Sensor fusion: how the EKF helps

**Purpose:** Explain fusion and its boundary.

Wheel odometry uses:

~~~text
left distance  = wheel radius * change in left wheel angle
right distance = wheel radius * change in right wheel angle
forward change = (left distance + right distance) / 2
yaw change     = (right distance - left distance) / wheel separation
~~~

The EKF combines wheel-odometry velocity information with IMU yaw and yaw
rate. It is planar, runs at 30 Hz, has a 0.2 s sensor timeout, resets on time
jumps, publishes /amr/localization/odometry, and owns
odom -> base_footprint.

The EKF improves local motion estimation. It does not build /map and does not
decide global map position. SLAM Toolbox or AMCL supplies map -> odom.

**Key message:** Sensor fusion is not LiDAR concatenation and is not mapping.

## Slide 8 — Localization and TF ownership

**Purpose:** Explain localization as an ownership problem.

In mapping/exploration, SLAM Toolbox consumes the front scan and local TF,
builds a fresh in-memory /map, and publishes map -> odom. In factory mode,
map_server and AMCL use a saved map and AMCL publishes map -> odom. The EKF
publishes odom -> base_footprint in both modes. robot_state_publisher
publishes the fixed sensor and arm branches.

Two publishers for one TF edge can produce inconsistent poses and jumps. The
correct fix is one active owner, not a second publisher.

**Key message:** Localization is a chain of owned transforms, not one node.

## Slide 9 — Two LiDARs and SLAM Toolbox

**Purpose:** Answer exactly how both LiDARs are used without interference.

The current implementation **does not concatenate front and rear LaserScan
messages into one scan for SLAM Toolbox**.

~~~text
front scan  -> front adapter -> /amr/sensors/front_lidar/scan -> SLAM Toolbox
front cloud -> front adapter -> front perception -> Nav2 obstacle layer
rear scan   -> rear adapter  -> /amr/sensors/rear_lidar/scan -> egress evidence
rear cloud  -> rear adapter  -> rear perception  -> Nav2 obstacle layer
~~~

SLAM consumes the front LaserScan only. Both validated PointCloud2 outputs feed
the global and local Nav2 obstacle layers as separate observation sources.

Interference is prevented by distinct topics, distinct frames
(front_lidar_link and rear_lidar_link), one matching adapter per sensor,
per-cloud validation, and no motion or TF authority in perception. The
costmap creates one navigation obstacle representation from two separately
owned streams; this is not one merged SLAM scan.

**Key message:** Say “both LiDARs contribute to the costmap” rather than
claiming that current code concatenates them for SLAM.

## Slide 10 — Nav2: planning and path following

**Purpose:** Explain Nav2's role and its boundary.

The mission supervisor coordinates:

1. ComputePathToPose.
2. collision-checked SmoothPath.
3. FollowPath.
4. Feedback and terminal result.

Current configured components:

| Function | Current implementation |
| --- | --- |
| Normal global planner | nav2_smac_planner/SmacPlannerLattice |
| Exact-goal planner | amr_navigation/ExactGoalLattice |
| Precision planner | amr_navigation/PrecisionNavfnPlanner |
| Smoother | nav2_smoother::SimpleSmoother |
| Controller | RPP plugins in controller_server |

Unknown space is rejected by the current planner configuration. Precision
plugins exist because endpoint and full-footprint behavior matter during
docking and placement.

**Key message:** Nav2 plans and follows paths; it does not bypass arbitration
or own base transport.

## Slide 11 — RPP: regulated local control

**Purpose:** Explain RPP and where it runs.

Regulated Pure Pursuit selects a lookahead point, converts path geometry to
curvature, and reduces speed for curvature, cost, approach distance, or
collision time. It produces a velocity request.

Normal navigation uses project-owned amr_mpc_controller::HeadingLatchedRPP:

- desired velocity 0.50 m/s;
- velocity-scaled lookahead;
- lookahead range 0.30 to 0.90 m;
- lookahead time 1.50 s;
- rotate-to-heading velocity 0.64 rad/s;
- maximum angular acceleration 0.40 rad/s²;
- collision and regulated speed scaling enabled.

Precision placement uses native
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController at
0.10 m/s, with rotate-to-heading disabled and reversing allowed.

The package name amr_mpc_controller is a compatibility name. The active
FollowPath plugin is RPP-based, not an MPC solver. The controller uses
encoder-derived wheel odometry for its configured odometry input while the
EKF remains the TF owner.

**Key message:** RPP turns a path into a bounded request; arbitration still
decides acceptance.

## Slide 12 — Global and local costmaps

**Global costmap**

- frame map; robot frame base_footprint;
- resolution 0.05 m;
- static layer reads /map;
- front and rear validated clouds in the obstacle layer;
- update 2 Hz, publish 1 Hz;
- padded rectangular footprint;
- inflation radius 0.75 m.

**Local costmap**

- frame odom; robot frame base_footprint;
- rolling window 5 m by 5 m;
- resolution 0.05 m;
- front and rear validated clouds;
- update 5 Hz, publish 2 Hz;
- inflation radius 0.75 m.

The footprint corners are x plus/minus 0.60 m and y plus/minus 0.40 m with
0.01 m padding. Inflation creates a clearance cost field around obstacles.
The current value is 0.75 m. A 0.55 m value belongs to an older configuration
snapshot and must be labeled historical.

**Key message:** Costmaps provide collision and clearance information; they do
not own the base command.

## Slide 13 — Command path and safety gates

~~~text
RPP or demo teleop
      |
      v
/amr/mpc/cmd_vel (Twist)
      |
      v
command_arbitration_node
  finite/planar check
  freshness and authority checks
  speed limits and acceleration ramp
      |
      v
/amr/control/cmd_vel (TwistStamped)
      |
      v
base_adapter_node
  command and base-state freshness
      |
      v
/amr/simulation/base/cmd_vel
      |
      v
Gazebo watchdog and differential drive
~~~

Current command-arbitration limits are 0.50 m/s linear, 0.64 rad/s angular,
0.50 m/s² linear acceleration, 0.40 rad/s² angular acceleration, and a
200 ms source timeout. The base adapter uses a 200 ms gated-command timeout
and 300 ms input-state timeout by default. The Gazebo watchdog independently
stops the plant after its 200 ms command freshness budget.

Stale, malformed, non-planar, or unauthorized input becomes zero motion or a
rejected action. The normal configuration has require_manipulator_stowed set
to false; mode-specific supervisors and egress actions still use status,
odometry, scan, and attachment evidence where their contracts require it.

**Key message:** A velocity request is not accepted plant motion.

## Slide 14 — Operator commands and control functions

Use ROS_DOMAIN_ID only in the inclusive range 0 through 232. Use a fresh
domain and run identity for each independent graph.

### Build and test

~~~bash
cd /home/pete/amr_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --symlink-install
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
colcon test
colcon test-result --verbose
~~~

### Standalone simulation and teleop

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
export GZ_VERSION=harmonic
export ROS_DOMAIN_ID=230
ros2 launch amr_simulation amr_simulation.launch.py headless:=false
~~~

In another sourced terminal:

~~~bash
ros2 run amr_control prototype_teleop.py
~~~

W/S/A/D move, X or Space stops, and Q quits. Teleop enters through the same
arbitration boundary.

### Factory

~~~bash
ros2 launch amr_factory factory_autonomous.launch.py \
  control_mode:=autonomous factory_attachment:=true
ros2 launch amr_manipulation move_group.launch.py
ros2 run amr_factory factory_cli.py list
ros2 run amr_factory factory_cli.py mode autonomous
ros2 run amr_factory factory_cli.py send pickup_a dispatch --timeout 240
ros2 run amr_factory factory_cli.py send pickup_b dispatch --timeout 240
ros2 run amr_factory factory_cli.py loop pickup_a pickup_b --cycles 1 --finish stay
ros2 run amr_factory factory_cli.py status
ros2 run amr_factory factory_cli.py stop
ros2 run amr_factory factory_cli.py cancel
ros2 run amr_factory factory_cli.py go home
~~~

Start MoveIt as a separate process. stop requests a graceful stop; cancel
requests cooperative cancellation and clears the sequence; go home is guarded
by factory idle, safety, and empty-stowed preconditions. These commands do not
imply hardware readiness or current acceptance.

## Slide 15 — MoveIt and manipulation

MoveIt 2 runs move_group with the composite URDF, SRDF, kinematics, OMPL
planning, controller execution configuration, and gz_ros2_control interfaces.
The SRDF defines the arm chain arm_base_link to gripper_tcp, gripper group,
product end effector, and stowed pose.

MoveIt plans arm and gripper trajectories. It does not publish mobile-base
velocity. Nav2 and command arbitration retain that authority.

The manipulation supervisor coordinates product actions, status, gripper
actions, attachment/detachment, contact evidence, cancellation, empty-stow
proof, and held-product fault behavior.

**Key message:** Nav2 plans base motion; MoveIt plans arm motion.

## Slide 16 — Factory mission cycle

~~~text
autonomous mode
      |
      v
pickup navigation -> gripper action -> attachment proof
      |
      v
transport with mission/Nav2/RPP
      |
      v
dispatch -> detachment proof -> terminal result
      |
      v
home only after idle, safe, and stowed proof
~~~

The factory supervisor owns run_sequence, transport_product,
navigate_station, stop, cancel, and factory status. The manipulation
supervisor owns execute_product_cycle and manipulation status. The mission
supervisor owns navigate_to_pose. These boundaries preserve action identity,
cancellation, and terminal proof.

**Key message:** A successful pose does not by itself prove a successful
product cycle.

## Slide 17 — Third-party packages and project plugins

External foundations include ROS 2 Humble, Gazebo Harmonic, ros_gz_bridge,
Nav2, robot_localization, SLAM Toolbox, MoveIt 2, tf2/tf2_ros, AprilTag ROS,
gz_ros2_control, KUKA Agilus support, standard ROS interfaces, and pluginlib.

Project-owned behavior includes the HeadingLatchedRPP and final-position RPP
plugins, ExactGoalLattice, PrecisionNavfnPlanner, sensor adapters, cloud
validation, command arbitration, base adapter, Gazebo watchdog, mission,
health, exploration, manipulation, and factory supervisors, and the
amr_interfaces contracts.

The workspace contains a vendored tf2 package. Runtime use of that overlay
must be established by the actual build/source environment and evidence.

**Key message:** External packages provide foundations; project code defines
the local interfaces, policies, and ownership boundaries.

## Slide 18 — Project difficulties and reliability risks

Teach these six difficulties:

1. **Time and freshness:** simulated-time data can be stale, future-dated,
   replayed, or from another boot.
2. **Single ownership:** duplicate TF, command, status, or attachment
   publishers create ambiguous behavior.
3. **Geometry:** the full padded footprint matters, not only the robot center.
4. **Lifecycle/startup:** readiness is evidence from dependencies, not merely a
   timer delay.
5. **Cancellation and proof:** stop, retreat, attachment, home, and terminal
   state cross node boundaries.
6. **Runtime evidence:** build and unit tests cannot replace a fresh runtime
   analyzer and clean teardown.

**Key message:** This is a distributed, timed, stateful system. The safe
design preserves ownership and fails closed when evidence is absent.

## Slide 19 — Current status and presentation guardrails

Use this current wording:

> The source architecture and focused checks are developed, but the latest
> Native42 runtime was stopped after the mandatory Product 102 unchanged
> analyzer failed. The failure showed missing bilateral right contact/product
> pose and empty base-command evidence; the cause remains unknown. Runtime
> acceptance is incomplete, and no post-failure repair or retry has been
> performed.

Current configuration facts:

- GridBased uses nav2_smac_planner/SmacPlannerLattice.
- Precision planners are ExactGoalLattice and PrecisionNavfnPlanner.
- Normal following uses HeadingLatchedRPP.
- Placement uses native Nav2 RPP.
- Inflation radius is 0.75 m.
- SLAM Toolbox consumes the front LaserScan only.
- Front/rear PointCloud2 streams feed costmaps separately.
- The EKF owns odom -> base_footprint.
- SLAM or AMCL owns map -> odom by mode.

If an older document says NavFn is the normal GridBased planner or gives
inflation as 0.55 m, label it as an older snapshot.

# Core reference for agents

## Ownership table

| Area | Owner | Output |
| --- | --- | --- |
| Shared contracts | amr_interfaces | Messages, services, actions, QoS helpers |
| Simulation plant | Gazebo Harmonic | Clock, sensors, joints, plant motion |
| Base boundary | amr_base_adapter | Simulation command, raw odom, joint states, status |
| Sensor boundary | amr_sensor_adapters | Stable scan, cloud, IMU topics |
| Local wheel state | amr_localization | Wheel odometry |
| Local fused state | robot_localization EKF | Fused odometry, odom -> base_footprint |
| Cloud quality | amr_perception | Validated front/rear clouds |
| Mapping | SLAM Toolbox | /map, map -> odom |
| Factory localization | AMCL/map server | map -> odom |
| Global planning | Nav2 planner | Path |
| Smoothing | Nav2 smoother | Smoothed path |
| Local following | Nav2 controller/RPP | /amr/mpc/cmd_vel |
| Motion gate | amr_control | /amr/control/cmd_vel |
| Mobile mission | amr_mission | Navigation actions/results |
| Arm planning | MoveIt | Joint trajectories |
| Manipulation | amr_manipulation | Product-cycle and status evidence |
| Factory | amr_factory | Sequence, transport, home, status |

## TF ownership

| TF edge | Mapping/exploration | Factory |
| --- | --- | --- |
| map -> odom | SLAM Toolbox | AMCL |
| odom -> base_footprint | EKF | EKF |
| Base/sensor/arm branches | Robot description/static publisher | Robot description/static publisher |

## Useful read-only inspection

~~~bash
ros2 node list
ros2 topic list -t
ros2 topic info --verbose /amr/control/cmd_vel
ros2 topic echo /amr/control/cmd_vel
ros2 topic echo /amr/manipulation/status
ros2 lifecycle get /amr/controller_server
ros2 lifecycle get /amr/command_arbitration_node
ros2 run tf2_ros tf2_echo map base_footprint
~~~

Do not call a runtime accepted because a topic appears in ros2 topic list.
Check the publisher, message type, QoS, timestamps, lifecycle state, gate
result, and teardown evidence.

## Recommended agent reading order

1. This guide's one-minute explanation.
2. src/amr_bringup/config/interface_ownership.yaml.
3. src/amr_interfaces message, service, and action definitions.
4. The base and composite Xacro files.
5. src/amr_localization/config/ekf.yaml.
6. src/amr_slam/config/mapper.yaml.
7. src/amr_navigation/config/planner.yaml.
8. src/amr_mpc_controller/config/controller.yaml.
9. src/amr_control/config/control.yaml.
10. Mission, manipulation, factory, and CLI source.
11. The matching tests.
12. The latest SESSION_HANDOFF.md.

For a longer code-reading course use docs/ROS2_BEGINNER_PROJECT_GUIDE.md. For
copy/paste runtime procedures use docs/SIMULATION_COMMANDS.md. Both contain
dated summaries, so the latest handoff remains the status authority.

## Questions for an agent or learner

1. Which node owns odom -> base_footprint? The robot_localization EKF.
2. Who owns map -> odom in mapping? SLAM Toolbox.
3. Who owns map -> odom in factory mode? AMCL.
4. Does SLAM consume both LiDAR scans? No, it consumes the front scan.
5. Where do both LiDARs contribute? Separate validated PointCloud2 costmap
   observations.
6. What frames do global and local costmaps use? map and odom.
7. What does RPP publish? A velocity request on /amr/mpc/cmd_vel.
8. Why does that request not directly move Gazebo? Arbitration, the base
   adapter, and the watchdog are later boundaries.
9. What invalidates a cloud? Bad layout, missing frame/stamp/data, future
   stamp, excessive age, or non-monotonic stamp.
10. Why is MoveIt separate? It owns arm/gripper planning; Nav2 owns base
    navigation.
11. Who controls a factory sequence? The factory supervisor action boundary.
12. What proves acceptance? Fresh runtime evidence satisfying the required
    analyzer, terminal, ownership, freshness, and teardown gates.

# Source index

| Topic | File |
| --- | --- |
| Package scope | src/README.md |
| Topic and TF ownership | src/amr_bringup/config/interface_ownership.yaml |
| QoS intent | src/amr_bringup/config/qos_profiles.yaml |
| Robot base | src/amr_description/urdf/amr.urdf.xacro |
| Mobile manipulator | src/amr_description/urdf/phase14_mobile_manipulator.urdf.xacro |
| MoveIt model | src/amr_description/config/phase14_mobile_manipulator.srdf |
| EKF | src/amr_localization/config/ekf.yaml |
| SLAM | src/amr_slam/config/mapper.yaml |
| Nav2 and costmaps | src/amr_navigation/config/planner.yaml |
| RPP | src/amr_mpc_controller/config/controller.yaml |
| Command limits | src/amr_control/config/control.yaml |
| Mission | src/amr_mission/src/mission_supervisor_node.cpp |
| MoveIt launch | src/amr_manipulation/launch/move_group.launch.py |
| Factory CLI | src/amr_factory/scripts/factory_cli.py |
| Runtime commands | docs/SIMULATION_COMMANDS.md |
| Beginner lessons | docs/ROS2_BEGINNER_PROJECT_GUIDE.md |
| Latest status | SESSION_HANDOFF.md |

## Final presentation rule

If a detail is not backed by current configuration or latest runtime evidence,
label it historical, illustrative, or pending. This prevents older planner
names, old costmap values, incomplete test results, and unverified runtime
claims from being presented as current facts.
