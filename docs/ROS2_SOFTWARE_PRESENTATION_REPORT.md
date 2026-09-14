# ROS 2 Software System Report

**Project:** AMR factory software stack  
**Audience:** presentation preparation  
**Scope:** ROS 2 software, interfaces, nodes, communication, execution flow, TF, theory, calculations, and verification state  
**Source audit date:** 2026-09-10  
**Hardware scope:** excluded. Hardware and mechanical design are left to the hardware team; this report mentions a hardware-related value only when the ROS 2 software contract depends on it.

## 1. Presentation thesis

The project uses ROS 2 as a typed, distributed software graph. Sensor data enters through stable adapter topics, perception and localization convert it into navigation-ready state, Nav2 plans and follows paths, and one command-arbitration node decides whether any velocity may reach the base. Factory actions coordinate navigation and manipulation through explicit interfaces. Lifecycle state, timestamps, sequence numbers, QoS, TF ownership, and fail-closed gates make stale or ambiguous data produce a stopped or unavailable state.

The shortest presentation story is:

1. ROS 2 provides the communication model: nodes, topics, services, actions, parameters, launch, and lifecycle.
2. The AMR graph separates sensing, state estimation, planning, control, mission, factory orchestration, and safety authority.
3. Typed interfaces make ownership visible and testable.
4. TF connects the global map to the robot body and sensors, with one owner per dynamic edge in each runtime mode.
5. The main engineering idea is gated motion: a planner may request motion, but only fresh and valid authority can publish the command that reaches the simulated base.
6. The repository has strong source and contract evidence, while Phase 15 mapping runtime proof and candidate-map promotion remain open.

## 2. Evidence and current-state rules

This report is a source-backed model of the ROS 2 graph. It was assembled from package manifests, CMake files, launch files, source code, interface definitions, YAML configuration, ownership contracts, and the current project handoff. It does not claim that every node is running now. A source-level contract, a successful build, and a runtime acceptance run are different kinds of evidence.

The current software scope is:

- ROS 2 Humble conventions, C++17 and Python 3 nodes.
- Gazebo Harmonic through `ros_gz_bridge` for simulation.
- `robot_localization` EKF for fused odometry.
- SLAM Toolbox for mapping and AMCL plus a saved map for production localization.
- Nav2 planner, smoother, and Regulated Pure Pursuit controller.
- Product 101 and Product 102 enabled for autonomous factory work.
- Product 103 present in the registry but disabled for autonomous work.
- No claim of hardware acceptance, functional-safety certification, or Phase 15 production map promotion.

Primary source contracts:

- [package ownership overview](../src/README.md)
- [interface ownership registry](../src/amr_bringup/config/interface_ownership.yaml)
- [QoS profile definitions](../src/amr_interfaces/include/amr_interfaces/qos_profiles.hpp)
- [ROS 2 system launch](../src/amr_bringup/launch/amr_system.launch.py)
- [simulation launch](../src/amr_simulation/launch/amr_simulation.launch.py)
- [factory autonomous launch](../src/amr_factory/launch/factory_autonomous.launch.py)
- [factory mapping launch](../src/amr_factory/launch/factory_mapping.launch.py)

## 3. ROS 2 theory used by this project

### 3.1 The ROS 2 computational graph

ROS 2 processes are represented as nodes. Nodes communicate through named, typed interfaces:

| ROS 2 concept | Communication pattern | AMR example | Why it matters |
|---|---|---|---|
| Node | A process or component that owns publishers, subscriptions, clients, servers, timers, parameters, or lifecycle state | `amr_control/command_arbitration_node` | Gives one software responsibility a name and an ownership boundary |
| Topic | Asynchronous publish/subscribe stream | `/amr/localization/odometry` | Suits continuous sensor, state, and command data |
| Service | Short request/response exchange | `/amr/factory/set_operation_mode` | Suits a bounded command such as changing mode or requesting stop |
| Action | Long-running goal with feedback, result, and cancellation | `/amr/factory/run_sequence` | Suits navigation, manipulation, transport, and multi-cycle jobs |
| Message | Compile-time typed data structure | `amr_interfaces/msg/FactoryStatus` | Defines the data contract independently of a node implementation |
| Parameter | Configuration owned by a node | RPP lookahead, timeout, map path | Makes runtime behavior configurable while keeping defaults explicit |
| Launch description | Declarative graph construction and startup sequencing | `factory_autonomous.launch.py` | Starts processes, remaps topics, sets parameters, and orders readiness gates |
| Lifecycle node | Managed state machine: unconfigured, inactive, active, finalized/error | base adapter, sensor adapters, localization, perception, Nav2 servers | Prevents consumers from treating partially initialized nodes as ready |

ROS 2 normally discovers endpoints through DDS. The topic name and message type must match, and the publisher/subscriber QoS must be compatible. Services and actions also use DDS-backed request/response channels, but the ROS 2 API presents a higher-level contract.

### 3.2 Namespaces and topic identity

The project uses `/amr` as the main namespace. The leading slash in the ownership registry makes the public interface names unambiguous. A node name and an executable name may differ: for example, the factory autonomous launch runs the Python cycle adapter with executable `cycle_manipulation_supervisor.py` but names the node `manipulation_supervisor_node`.

The interface registry is intended to answer three questions for every public endpoint:

1. What exact name and ROS type does the endpoint use?
2. Which node is allowed to publish or serve it?
3. Which runtime mode owns a mode-dependent resource such as `map -> odom`?

### 3.3 Topics, services, and actions in this system

Topics carry the high-rate or continuously refreshed data needed by downstream nodes. The base status, health status, manipulator status, factory status, sensor streams, odometry, costmaps, and velocity commands are topics.

Services perform short operations. The factory mode service changes manual/autonomous mode, Trigger services stop or cancel a sequence, and exploration services start or stop frontier exploration.

Actions hold state over time. A navigation action can provide feedback while a path is planned and followed, accept cancellation, and return a structured result. Factory transport and sequence actions compose several navigation and manipulation actions without exposing direct velocity control to the caller.

### 3.4 Lifecycle theory in the AMR graph

The managed nodes use the lifecycle sequence:

```text
unconfigured -> configuring -> inactive -> activating -> active
                                         \-> error/finalized
```

The active state means the node has created and validated its ROS interfaces and is allowed to process or publish its normal data. Deactivation clears or zeros command state where appropriate. The factory launch releases dependent parts of the graph only after the required upstream readiness process exits successfully or a lifecycle transition completes.

### 3.5 Time theory

The simulation graph uses `use_sim_time=true`, so message headers, TF timestamps, map timestamps, and action goal timestamps use the simulated ROS clock. Freshness decisions in several safety paths use a steady/monotonic clock for elapsed time. This separation avoids treating a simulation pause or backward time jump as a valid freshness interval.

The software checks both kinds of time:

- ROS time validates message stamps and TF age.
- Steady time measures how long ago a callback actually received a message.
- Backward time, invalid stamps, future stamps, stale receipts, or non-monotonic source sequences can force an unavailable or fault state.

## 4. System structure

### 4.1 Package layers

There are 17 ROS 2 packages under `src`. Their software responsibilities are:

| Layer | Packages | Responsibility |
|---|---|---|
| Contracts | `amr_interfaces` | Project messages, actions, service, and shared QoS helpers |
| Launch and policy | `amr_bringup` | Namespace, environment, interface ownership, QoS policy names, runtime defaults |
| Robot model | `amr_description` | URDF/Xacro, TF frame topology, Gazebo sensor definitions, ros2_control model inputs |
| Simulation boundary | `amr_simulation` | Gazebo launch and native command watchdog plugin |
| Input adapters | `amr_sensor_adapters`, `amr_base_adapter` | Stable ROS topics and the final simulation base command boundary |
| State estimation | `amr_localization` | Wheel odometry and EKF configuration |
| Perception | `amr_perception` | PointCloud2 validation and front/rear cloud forwarding |
| Mapping/localization | `amr_slam` plus external SLAM Toolbox, AMCL, and map server | `map -> odom` authority by runtime mode |
| Navigation | `amr_navigation`, `amr_mpc_controller`, `amr_mission` | Nav2 planning/smoothing, RPP path following, mission action boundary |
| Command authority | `amr_control` | Single project-owned velocity output, rate/acceleration limits, interlocks, docking egress |
| Observability | `amr_health` | Freshness and semantic health summary; no motion authority |
| Exploration | `amr_exploration` | Frontier selection and bounded delegation to the mission action |
| Factory orchestration | `amr_factory` | Product/station registry, transport and sequence actions, mapping tools, simulation proxies |
| Manipulation | `amr_manipulation` | Product-cycle adapter, attachment/tag gates, MoveIt/Gate 6 paths |

The repository also contains `src/amr_machine_controller/src/machine_controller_node.cpp`. It has no package manifest or CMake entry and references legacy `Machine*` interfaces, so it is an orphaned legacy source file, not an active ROS 2 package in the current graph.

### 4.2 High-level dataflow

```text
Gazebo / external sensors
        |
        v
stable sensor and base adapters
        |
        +--> validated lidar clouds --> Nav2 costmaps and SLAM Toolbox
        +--> camera pair ------------> AprilTag detector --> product tags
        +--> IMU + wheel joints ----> wheel odometry --> robot_localization EKF
        +--> raw odometry ----------> base status / diagnostic evidence
                                      |
                                      v
                           fused odometry + odom -> base_footprint TF

map source: SLAM Toolbox in mapping, or map server + AMCL in factory mode
                                      |
                                      v
                         map -> odom TF + OccupancyGrid map
                                      |
                                      v
                         Nav2 planner -> smoother -> RPP controller
                                      |
                                      v
                         /amr/mpc/cmd_vel (Twist)
                                      |
                                      v
                    command arbitration and manipulation interlock
                                      |
                                      v
                    /amr/control/cmd_vel (TwistStamped)
                                      |
                                      v
                  base adapter -> simulation bridge -> Gazebo base
```

The name `amr_mpc_controller` is a compatibility package/topic name. The current controller configuration uses Nav2 Regulated Pure Pursuit; the current source does not depend on an MPPI or MPC controller package.

### 4.3 Runtime entry points

| Entry point | Software graph |
|---|---|
| `amr_simulation.launch.py` | General Gazebo simulation, robot state publisher, bridge, base/sensor adapters, localization, perception, SLAM, Nav2, RPP, control, and mission |
| `amr_system.launch.py` | ROS-only system environment and health launch contract; sets `ROS_DOMAIN_ID=1` and `ROS_LOCALHOST_ONLY=1` |
| `factory_localization.launch.py` | Factory world and bridge. Production mode uses map server + AMCL; mapping mode omits AMCL/map server and lets the mapping entry point own SLAM |
| `factory_mapping.launch.py` | Run-specific online mapping. Manual mode keeps motion under teleoperation; autonomous mode releases planner, controller, mission, and frontier exploration through readiness gates |
| `factory_autonomous.launch.py` | Canonical autonomous factory entry point, static-map mode, Product 101/102 registry-derived cycle mapping, factory supervisor, and cycle manipulation supervisor |
| `factory_demo.launch.py` | Legacy/optional demonstration path; not the canonical autonomous acceptance entry point |
| `amr_manipulation/move_group.launch.py` | Separate MoveIt `move_group` process for manipulation paths and tests |

## 5. Node and executable structure

The following table describes production/runtime code. Test files are verification code and are summarized in Section 7. Launch and configuration files are included because they create the ROS graph even when they do not define a node class.

### 5.1 Core runtime nodes

| Package | Code | Node/executable role |
|---|---|---|
| `amr_base_adapter` | `src/base_adapter_node.cpp` | Managed final adapter before the simulated base. Revalidates planar finite commands, forwards raw odometry and joint states, publishes zero when the command or input evidence is stale, and publishes `BaseStatus`. It is not the velocity arbiter. |
| `amr_sensor_adapters` | `src/lidar_adapter_node.cpp` | One generic lifecycle relay compiled as `front_lidar_adapter_node` and `rear_lidar_adapter_node`. Converts simulation scan/point topics into stable `/amr/sensors/...` topics without filtering or authority decisions. |
| `amr_sensor_adapters` | `src/imu_adapter_node.cpp` | Lifecycle relay from simulated IMU to `/amr/sensors/imu/data_raw`. |
| `amr_sensor_adapters` | `src/product_camera_adapter_node.cpp` | Lifecycle camera relay. Matches image and camera-info timestamps, enforces the configured 640 x 480 stream, sets the optical frame, and publishes RGB image, camera info, and depth. |
| `amr_control` | `src/command_arbitration_node.cpp` | Single project-owned velocity publisher. Validates source commands and status evidence, enforces velocity and acceleration limits, applies manipulator interlocks, and owns `/amr/control/dock_egress`. |
| `amr_control` | `scripts/prototype_teleop.py` | Simulation-only keyboard publisher to the upstream `/amr/mpc/cmd_vel` test input. It does not bypass arbitration. |
| `amr_localization` | `src/wheel_odometry_node.cpp` | Managed differential-drive odometry from named wheel joint positions. Handles first sample, duplicate/backward timestamps, finite values, integration, velocity, covariance, and `/amr/localization/wheel_odometry`. |
| `amr_perception` | `src/lidar_pipeline_node.cpp` | One lifecycle implementation compiled for front and rear clouds. Rejects malformed, stale, future, and non-monotonic PointCloud2 data; forwards accepted clouds unchanged. |
| `amr_health` | `src/health_supervisor_node.cpp` | Observes `BaseStatus`, validates freshness, boot ID, sequence, time, and state/reason semantics, and publishes `HealthStatus`. It has no command or recovery authority. |
| `amr_mission` | `src/mission_supervisor_node.cpp` | Action boundary for normal, precise, and retreat navigation. Validates planar goals, coordinates planner -> smoother -> controller actions, forwards cancellation, and publishes navigation feedback/results. It never publishes velocity directly. |
| `amr_factory` | `src/factory_supervisor_node.cpp` | Loads and validates product/station registries; exposes transport, sequence, and station actions plus mode/stop/cancel services; coordinates cycle and navigation clients; publishes `FactoryStatus`. |
| `amr_manipulation` | `scripts/cycle_manipulation_supervisor.py` | Current autonomous cycle adapter. Owns `ExecuteProductCycle`, tracks product/station mappings, forwards internal motion cancellation, and exposes canonical manipulator status for the factory graph. |
| `amr_manipulation` | `src/manipulation_supervisor_node.cpp` | Existing action/status manipulation supervisor used by the older manipulation path and compatibility launch. It owns the public `ManipulateProduct` action in the interface registry. |
| `amr_exploration` | `scripts/frontier_explorer.py` | Fail-closed mapping node. Observes map, costmap, TF, base/manipulator status, selects bounded frontier goals, delegates them to the mission action, owns cancellation, blacklists failed goals, and publishes diagnostics. |
| external Nav2 | `nav2_planner`, `nav2_smoother`, `nav2_controller` | Planner, path smoother, and RPP controller. The current configuration uses NavFn, Simple Smoother, and Regulated Pure Pursuit. |
| external estimation | `robot_localization/ekf_filter_node` | Fuses wheel odometry and IMU into `/amr/localization/odometry` and owns dynamic `odom -> base_footprint`. |
| external mapping/localization | `slam_toolbox/async_slam_toolbox_node`, `nav2_map_server/map_server`, `nav2_amcl/amcl` | SLAM Toolbox owns `map -> odom` in mapping. AMCL owns it in factory localization. Map server publishes the saved occupancy map in factory mode. |
| external perception | `apriltag_ros/product_tag_detector` | Consumes the adapted camera pair and publishes AprilTag detections. |
| external manipulation | MoveIt `move_group` | Provides the MoveIt planning/execution process used by the separate manipulation launch and Gate 6 paths. |

### 5.2 Simulation-side software

| Code | Role |
|---|---|
| `amr_simulation/src/command_watchdog_system.cpp` | Gazebo System plugin. It reads the native base command, uses simulation time, and publishes a native enable state. A missing command or backward simulation time disables the plant-side command path. |
| `amr_simulation/include/command_watchdog.hpp` | Header-only watchdog state machine used by the plugin and its tests. |
| `amr_factory/src/gazebo_set_pose_proxy.cpp` | Restricted ROS service proxy to Gazebo `SetEntityPose` for controlled evidence/setup operations. It is not an autonomy command path. |
| `amr_factory/src/gazebo_control_world_proxy.cpp` | Restricted ROS service proxy to Gazebo world-control operations used by bounded attachment setup. |
| `amr_factory/tools/generate_apriltag_textures.cpp` | Offline AprilTag texture generator. It is a build/tool utility, not a runtime ROS node. |

### 5.3 Pure validation libraries

| Code | Role |
|---|---|
| `amr_localization/include/amr_localization/diff_drive.hpp` | Differential-drive midpoint integration equations used by wheel odometry. |
| `amr_mission/include/amr_mission/goal_validation.hpp` | Validates finite planar navigation goals and unit planar quaternions. |
| `amr_perception/include/amr_perception/point_cloud_validation.hpp` | Validates PointCloud2 frame, stamp, dimensions, row layout, and non-overlapping finite-size X/Y/Z fields. |
| `amr_manipulation/include/amr_manipulation/attachment_gate.hpp` and `src/attachment_gate.cpp` | Pure attach/detach evidence gate and attached-product state. Checks product ID, pose/orientation, fresh left/right contact, and dispatch position error. |
| `amr_manipulation/include/amr_manipulation/tag_observation_validator.hpp` and `src/tag_observation_validator.cpp` | Pure AprilTag stability validator. Requires five recent observations for the expected configured ID, zero hamming correction, bounded collection window, position spread, and orientation spread. |

## 6. Public ROS 2 interfaces

### 6.1 Project messages

The definitions are in [amr_interfaces/msg](../src/amr_interfaces/msg).

| Message | Main fields | Meaning |
|---|---|---|
| `BaseStatus` | `header`, `sequence`, `valid`, `source_boot_id`, `state`, `reason` | Base readiness evidence. States are `UNKNOWN`, `UNAVAILABLE`, `READY`, `FAULT`. Reasons distinguish missing/stale odometry, missing/stale joint states, and fault. |
| `HealthStatus` | `header`, `sequence`, `valid`, `source_boot_id`, `state`, `reason`, `base_ready` | Health summary derived from base evidence. States are `UNKNOWN`, `DEGRADED`, `HEALTHY`, `FAULT`; reasons include stale/invalid evidence, time reversal, source not ready, and source fault. |
| `ManipulatorStatus` | `source_boot_id`, `sequence`, `valid`, `state`, `base_motion_allowed`, `product_attached`, `product_id`, `detail` | Manipulator state and motion permission. States include `STARTING`, `STOWED_EMPTY`, `STOWED_LOADED`, `MOVING`, `DEPLOYED`, and `FAULT`. This message uses boot ID and sequence rather than a standard header. |
| `FactoryStatus` | `header`, `sequence`, `mode`, `phase`, `active`, queue and product fields, outcome, cycle counters, stop/fault flags | Factory orchestration state, active job/sequence, product identity, delivery counters, final station, graceful stop, and fault latch. |

Status messages are not merely display text. The consumers use the validity, boot ID, sequence, state, reason, and freshness fields to decide whether a source can be trusted.

### 6.2 Project actions

The definitions are in [amr_interfaces/action](../src/amr_interfaces/action).

| Action | Goal | Result | Feedback/owner |
|---|---|---|---|
| `ManipulateProduct` | Operation `PICK` or `PLACE`, station ID, product ID | Success or base-stationary, perception, planning, execution, grasp, attachment, or interlock failure | Phase and phase name; older manipulation supervisor |
| `TransportProduct` | Pickup station and destination station | Delivered flag plus navigation, pick, place, cancellation, interlock, or dependency result | Queue position, current station, attachment; factory supervisor |
| `ExecuteProductCycle` | Pickup station and destination station | Delivered flag plus invalid request, dependency, preparation, execution, cancellation, interlock, or retained-product result | Preparation/execution/cancel phase, product ID, attachment; cycle adapter |
| `RunSequence` | Array of pickup stations, cycle count, final station | Completed jobs/cycles plus success, stopped, canceled, job/home/interlock/invalid/dependency result | Current cycle, sequence index, station, product, phase, attachment; factory supervisor |
| `NavigateStation` | Station ID | Success or navigation, cancellation, interlock, invalid, dependency result | Current station and phase; factory supervisor |

The external navigation actions are `nav2_msgs/action/NavigateToPose`, exposed by the mission supervisor at `/amr/mission/navigate_to_pose`, `/amr/mission/navigate_to_pose_precise`, and `/amr/mission/navigate_to_pose_retreat`. The ownership registry currently lists the normal and precise endpoints but omits the retreat endpoint; source and tests show that the retreat server exists. This is a documentation/contract completeness gap to resolve before presenting the registry as exhaustive.

### 6.3 Project and standard services

| Service | Type | Role |
|---|---|---|
| `/amr/factory/set_operation_mode` | `amr_interfaces/srv/SetOperationMode` | Request `MANUAL` or `AUTONOMOUS`; response states whether the transition was accepted |
| `/amr/factory/stop_sequence` | `std_srvs/srv/Trigger` | Graceful stop after the active delivery; does not add home motion |
| `/amr/factory/cancel_sequence` | `std_srvs/srv/Trigger` | Cancellation path for the active sequence; cancels current manipulation motion and keeps terminal ownership explicit |
| `/amr/manipulation/internal/cancel_cycle_motion` | `std_srvs/srv/Trigger` | Internal cycle-adapter cancellation boundary |
| `/amr/exploration/start` and `/amr/exploration/stop` | `std_srvs/srv/Trigger` | Start or stop the frontier state machine |
| `/amr/control/dock_egress` | `nav2_msgs/action/BackUp` | Action, not a service: bounded reverse egress from a pickup dock under arbitration-owned safety checks |

### 6.4 Standard ROS message types in the graph

| Data | Type |
|---|---|
| Velocity input | `geometry_msgs/msg/Twist` |
| Gated base velocity | `geometry_msgs/msg/TwistStamped` |
| Raw/fused/wheel odometry | `nav_msgs/msg/Odometry` |
| Lidar scan | `sensor_msgs/msg/LaserScan` |
| Lidar cloud | `sensor_msgs/msg/PointCloud2` |
| IMU | `sensor_msgs/msg/Imu` |
| Joint state | `sensor_msgs/msg/JointState` |
| Camera image/depth | `sensor_msgs/msg/Image` |
| Camera calibration | `sensor_msgs/msg/CameraInfo` |
| Map | `nav_msgs/msg/OccupancyGrid` |
| Planned/smoothed path | `nav_msgs/msg/Path` |
| Health/exploration diagnostics | `diagnostic_msgs/msg/DiagnosticArray` |
| TF transport | `tf2_msgs/msg/TFMessage` on `/tf` and `/tf_static` |
| Mode and setup triggers | `std_srvs/srv/Trigger`, `std_msgs/msg/Empty`, `std_msgs/msg/String` where used by simulation setup |
| Product detections | `apriltag_msgs/msg/AprilTagDetectionArray` |
| MoveIt/control interfaces | `moveit_msgs`, `control_msgs`, and `controller_manager` interfaces from external packages |
| Gazebo bridge interfaces | `ros_gz_interfaces` and native Gazebo message types behind `ros_gz_bridge` |

## 7. Communication and ownership

### 7.1 Sensor and perception path

```text
Gazebo sensor topic
    -> ros_gz_bridge
    -> stable adapter topic under /amr/sensors
    -> optional validation/forwarding under /amr/perception
    -> Nav2 costmaps and/or SLAM/AprilTag consumer
```

The adapters preserve a stable project-facing contract even if a simulator or external device changes its native topic name. The adapters are relays, not filters. The lidar perception nodes perform the data-shape and age checks for PointCloud2.

The product camera adapter keeps image and camera-info messages paired by exact timestamp, requires the configured 640 x 480 stream, and sets the optical-frame identity used by the AprilTag detector. The tag detector publishes `/amr/perception/product_tags`.

### 7.2 Localization path

```text
/amr/base/joint_states
    -> wheel_odometry_node
    -> /amr/localization/wheel_odometry

/amr/sensors/imu/data_raw
    + wheel odometry
    -> robot_localization/ekf_filter_node
    -> /amr/localization/odometry
    -> dynamic odom -> base_footprint TF
```

Wheel odometry does not publish TF. The EKF is the sole owner of `odom -> base_footprint` in the current design. This prevents two estimators from publishing the same dynamic edge.

### 7.3 Navigation and command path

```text
mission action goal
    -> Nav2 planner
    -> Nav2 smoother
    -> Nav2 RPP controller
    -> /amr/mpc/cmd_vel (Twist)
    -> command_arbitration_node
    -> /amr/control/cmd_vel (TwistStamped)
    -> base_adapter_node
    -> /amr/simulation/base/cmd_vel
    -> ros_gz_bridge
```

The mission supervisor owns the project-facing navigation action. It coordinates Nav2 action clients but never writes velocity. The controller requests a command, the arbitration node applies the project limits and interlock policy, and the base adapter performs a final finite/planar validation before forwarding it.

The normal upstream command owner is `nav2_controller/controller_server`. The simulation-only teleop script also publishes the same upstream type/topic for a controlled test path. Both paths still pass through command arbitration and the base adapter.

### 7.4 Factory orchestration path

```text
factory_cli or another ROS 2 client
    -> set_operation_mode / stop / cancel services
    -> TransportProduct / RunSequence / NavigateStation actions
    -> factory_supervisor_node
    -> NavigateToPose client and ExecuteProductCycle client
    -> mission supervisor and cycle manipulation supervisor
    -> status topics and structured action feedback/results
```

The factory supervisor loads the product and station registries at startup. It rejects invalid mappings, duplicate identifiers, invalid station roles, invalid poses, missing dispatch slots, and an enabled Product 103. The autonomous launch derives its product/station mapping from the same registry, so the runtime route is configuration-driven rather than duplicated in multiple nodes.

### 7.5 Mapping and exploration path

```text
lidar -> stable sensor topic -> validated cloud -> SLAM Toolbox -> /map + map -> odom
                                               |
                                               v
                                global costmap and frontier explorer
                                               |
                                  NavigateToPose action goals
                                               |
                                    mission supervisor -> Nav2
```

The frontier algorithm finds unknown cells adjacent to known free space, clusters them, chooses representatives, rechecks candidates against costmap geometry, and avoids a blacklist of failed endpoints. The frontier node delegates all motion to the existing mission action, so exploration does not create a second velocity authority.

Mapping has explicit manual and autonomous modes. Manual mapping does not start the autonomous planning/mission/exploration chain. Autonomous mapping releases adapters, SLAM, map readiness, planner/smoother, controller, mission, and exploration through ordered readiness gates.

## 8. QoS design

The canonical helper definitions are in [qos_profiles.hpp](../src/amr_interfaces/include/amr_interfaces/qos_profiles.hpp), with named profiles mirrored in [qos_profiles.yaml](../src/amr_bringup/config/qos_profiles.yaml).

| Profile | Reliability | Durability | Depth | Deadline/lifespan | Use |
|---|---|---|---:|---|---|
| Sensor | Best effort | Volatile | 5 | None | Lidar, IMU, camera, raw sensor streams |
| State | Reliable | Volatile | 5 | None | Odometry and state messages |
| Authority | Reliable | Volatile | 1 | 100 ms deadline | Base/manipulator authority and readiness evidence |
| Command | Reliable | Volatile | 1 | 100 ms deadline, 200 ms lifespan | Gated velocity commands |
| Nav2 command input | Reliable | Volatile | 1 | None | Controller output entering arbitration |
| Diagnostic | Reliable | Volatile | 20 | None | Health and exploration diagnostics |
| Clock | Best effort | Volatile | 1 | None | Simulation clock |
| TF | Reliable | Volatile | 100 | None | Dynamic transforms |
| Static TF | Reliable | Transient local | 1 | None | Latched static transforms |

The command lifespan adds a DDS-level expiry to the application-level timeout. The application still validates freshness and authority explicitly; QoS does not replace the safety state machine.

## 9. TF tree and frame theory

### 9.1 Intended tree

```text
map
└── odom                         dynamic map localization edge
    └── base_footprint           dynamic EKF odometry edge
        └── base_link            static robot-state-publisher edge
            ├── left_wheel
            ├── right_wheel
            ├── front_left_caster and front_right_caster
            ├── rear_left_caster and rear_right_caster
            ├── imu_link
            ├── front_lidar_link
            ├── rear_lidar_link
            └── product_camera_link
                └── product_camera_optical_frame

base_link
└── arm_base_link -> arm_link_1 -> ... -> arm_tool0
                                  └── gripper_base_link -> gripper_tcp
```

The composite mobile-manipulator description adds the arm, gripper, TCP, camera, and optional loaded-product frames. The base-only description keeps the navigation frames and sensor frames used by the base graph.

### 9.2 Dynamic edge ownership

| TF edge | Mapping mode | Factory localization mode |
|---|---|---|
| `map -> odom` | SLAM Toolbox | AMCL |
| `odom -> base_footprint` | `robot_localization/ekf_filter_node` | `robot_localization/ekf_filter_node` |
| `base_footprint -> base_link` and sensor/joint tree | `robot_state_publisher` from URDF | `robot_state_publisher` from URDF |

The `map -> odom` authority must switch with the runtime mode. Running SLAM and AMCL as simultaneous authorities for the same edge would make the TF graph ambiguous and can destabilize navigation.

### 9.3 Why the frames matter

- Lidar data arrives in `front_lidar_link` or `rear_lidar_link`, then TF transforms it into the costmap or SLAM frame.
- Odometry is expressed as `odom` with child `base_footprint`.
- Navigation goals are expressed in `map`.
- The mission supervisor transforms the robot pose using `map -> base_footprint` before making navigation decisions.
- The command arbiter uses base-frame geometry for planar velocity validation and docking egress clearance.

The current Phase 15 evidence limits matter here: an aggregate `/tf` message proves less than an independently verified per-edge ownership and freshness record. The current commissioning packet still treats live per-edge TF verification and production map acceptance as pending.

## 10. Main algorithms and calculations

The values below come from the current source/configuration. Calculations are explanatory presentation material, not new runtime limits.

### 10.1 Differential-drive kinematics

The wheel odometry uses wheel radius `r = 0.1128 m` and wheel separation `L = 0.566 m`.

For wheel angular displacements `Delta phi_L` and `Delta phi_R`:

```text
Delta s     = r / 2 * (Delta phi_L + Delta phi_R)
Delta theta = r / L * (Delta phi_R - Delta phi_L)
```

The midpoint integrator then uses:

```text
theta_mid = theta + Delta theta / 2
Delta x    = Delta s * cos(theta_mid)
Delta y    = Delta s * sin(theta_mid)
```

Velocity form:

```text
v     = r / 2 * (omega_L + omega_R)
omega = r / L * (omega_R - omega_L)
```

Examples:

- Equal one-radian wheel motion: `Delta s = 0.1128 m`, `Delta theta = 0`. The robot translates without rotating.
- Opposite one-radian wheel motion: `Delta s = 0`, `Delta theta = 0.1128 / 0.566 * 2 = 0.3986 rad`, approximately `22.84 degrees`. The robot rotates in place.
- At `v = 0.5 m/s` and `omega = 0`, each wheel needs approximately `v/r = 4.43 rad/s`.
- At pure rotation `omega = 0.4 rad/s`, each wheel speed magnitude is `omega L / (2r) = 1.00 rad/s`, with opposite signs.
- At simultaneous `v = 0.5 m/s`, `omega = 0.4 rad/s`, the ideal wheel angular rates are approximately `omega_L = 3.43 rad/s` and `omega_R = 5.44 rad/s`.
- The instantaneous turning radius at `v = 0.5 m/s`, `omega = 0.4 rad/s` is `R = v/omega = 1.25 m`.

Source: [diff_drive.hpp](../src/amr_localization/include/amr_localization/diff_drive.hpp) and [wheel_odometry_node.cpp](../src/amr_localization/src/wheel_odometry_node.cpp).

### 10.2 Command limits and slew-rate calculation

The arbitration node publishes at `20 Hz`, so the nominal tick is:

```text
Delta t = 1 / 20 = 0.05 s
```

The configured limits are:

```text
linear speed limit  = 0.5 m/s
angular speed limit  = 0.4 rad/s
linear acceleration  = 0.5 m/s^2
angular acceleration = 0.4 rad/s^2
```

The per-tick maximum changes are therefore:

```text
Delta v_max     = 0.5 * 0.05 = 0.025 m/s per tick
Delta omega_max = 0.4 * 0.05 = 0.020 rad/s per tick
```

The implementation uses the pattern:

```text
next = current + clamp(target - current,
                       -limit * Delta t,
                       +limit * Delta t)
```

The exact elapsed time is measured by the node, so the values above are nominal rather than a timing guarantee.

### 10.3 Freshness and timing calculations

Important configured windows:

| Contract | Value | Equivalent interpretation |
|---|---:|---|
| Arbitration input timeout | 200 ms | Four nominal 20 Hz arbitration ticks |
| Manipulator status freshness | 200 ms | A stale interlock evidence stream stops normal motion |
| Command QoS lifespan | 200 ms | DDS expires a command after this age |
| Authority deadline | 100 ms | Ten-Hz heartbeat expectation |
| Base odometry/joint evidence freshness | 300 ms | Three nominal 10 Hz health cycles |
| Health publication period | 100 ms | 10 Hz status output |
| EKF output frequency | 30 Hz | Approx. 33.3 ms nominal period |
| Lidar sensor rate | 10 Hz | Approx. 100 ms nominal period |
| Point cloud pipeline max age | 500 ms | Five nominal lidar periods |
| SLAM TF publish period | 50 ms | 20 Hz dynamic map transform target |
| SLAM map update period | 1 s | Map updates are slower than TF updates |
| Frontier map timeout | 3 s | Three nominal 1 Hz map freshness intervals |
| Frontier TF timeout | 1 s | TF must remain available for goal selection |
| Frontier goal timeout | 120 s | Bound for one exploration goal |
| Frontier cancel timeout | 5 s | Bound for cancellation acknowledgement |

A stale source is not repaired by publishing the last command again. The arbiter transitions to zero output or rejects the request, and the base adapter also forwards zero when its command evidence expires.

The 200 ms timeout plus a 50 ms output tick should be presented as layered timing constraints, not as an end-to-end latency guarantee. End-to-end latency also includes DDS delivery, executor scheduling, callback time, and the base adapter timer.

### 10.4 Costmap and footprint geometry

The software footprint is a rectangle with vertices at `x = +/-0.6 m` and `y = +/-0.4 m`:

```text
footprint width  = 0.6 - (-0.6) = 1.2 m
footprint depth  = 0.4 - (-0.4) = 0.8 m
```

The local costmap is `5 m x 5 m` at `0.05 m/cell`:

```text
cells per side = 5 / 0.05 = 100
total cells    = 100 * 100 = 10,000
```

The global and local configurations use an inflation radius of `0.55 m`:

```text
inflation radius in cells = 0.55 / 0.05 = 11 cells
```

Obstacle observations are configured to a maximum range of `10 m` with `12 m` ray tracing. The same front/rear perception streams feed costmap obstacle layers.

The checked factory map image is `240 x 200` pixels at `0.05 m/cell`, so its nominal physical extent is:

```text
width  = 240 * 0.05 = 12.0 m
height = 200 * 0.05 = 10.0 m
origin = (-6.0 m, -5.0 m, 0.0 rad)
```

Sources: [planner.yaml](../src/amr_navigation/config/planner.yaml), [controller.yaml](../src/amr_mpc_controller/config/controller.yaml), [factory.yaml](../src/amr_factory/maps/factory.yaml), and [factory.pgm](../src/amr_factory/maps/factory.pgm).

### 10.5 Regulated Pure Pursuit lookahead

The normal RPP profile has desired linear speed `0.5 m/s`, minimum lookahead `0.3 m`, maximum lookahead `0.9 m`, and lookahead time `1.5 s`. A velocity-scaled lookahead estimate at nominal speed is:

```text
lookahead = velocity * lookahead_time
          = 0.5 * 1.5
          = 0.75 m
```

That value lies inside the configured `[0.3, 0.9] m` range. The controller also regulates speed for approach and collision constraints. The placement profile uses `0.1 m/s`, `0.01 m` approach distance, and allows reversing; the normal profile does not allow reversing.

### 10.6 Planar pose and quaternion validation

For a planar navigation goal:

```text
position.z = 0
quaternion.x = 0
quaternion.y = 0
quaternion.z^2 + quaternion.w^2 ~= 1
```

The goal validator uses a squared-norm tolerance of `1e-6`. For a yaw angle `theta`, the planar quaternion is:

```text
q_z = sin(theta / 2)
q_w = cos(theta / 2)
```

This rejects malformed 3D orientation input before it enters the navigation action chain.

### 10.7 PointCloud2 layout validation

The perception validator accepts only FLOAT32 or FLOAT64 X/Y/Z fields, with exactly one field for each coordinate, `count = 1`, valid offsets, and no overlapping byte ranges. It also requires:

```text
row_step >= width * point_step
data.size == row_step * height
```

This catches a malformed byte layout before the cloud reaches costmap or SLAM consumers. The check is about message memory layout and metadata; it does not claim that every point is geometrically useful.

### 10.8 Attachment and tag-evidence calculations

The software attachment gate uses these values:

- position error at most `0.030 m`;
- orientation error at most `0.15 rad`;
- left and right contact evidence no older than `0.100 s`;
- dispatch position error at most `0.030 m`.

The tag validator requires at least five observations for the expected configured ID, all in the `tag36h11` family, zero hamming correction, each no older than `0.250 s`, collected within `1.0 s`, with position spread no greater than `0.015 m` and orientation spread no greater than `0.05 rad`.

These thresholds are software acceptance gates. They are not a substitute for the separate hardware and functional-safety review.

## 11. Fail-closed control and safety boundaries

The motion boundary can be explained as a chain of proof obligations:

```text
planner request
    -> valid planar command
    -> fresh command source
    -> fresh valid manipulator permission when required
    -> fresh base readiness and localization evidence
    -> velocity/acceleration limits
    -> final base-adapter finite/planar validation
    -> simulation watchdog still enabled
    -> base command reaches Gazebo
```

Important invariants:

- `command_arbitration_node` is the sole project-owned publisher of `/amr/control/cmd_vel`.
- The base adapter does not trust the arbiter blindly; it revalidates frame and planar finite constraints.
- A stale or invalid source results in zero or no admitted motion.
- The factory interlock requires valid, fresh manipulator status before normal motion in the autonomous factory path.
- A loaded product must remain represented consistently by `product_attached`, `product_id`, manipulator state, and factory state.
- The native Gazebo watchdog provides an independent simulation-side command expiry and backward-time disable path.
- Health reporting is observational. It cannot silently enable motion or recover a fault.
- Exploration owns a motion token through goal result and cancellation acknowledgement, so a late action future cannot reopen motion admission or dispatch a second goal.
- Stop and cancellation paths preserve terminal ownership and do not append an unrequested home movement.
- Product 103 remains disabled for autonomous operation.

The design separates three concerns that are often confused in presentations:

1. Planning decides where the robot should go.
2. Arbitration decides whether a velocity request is currently admissible.
3. The base adapter and simulation watchdog enforce the final command boundary.

## 12. Configuration and code inventory by package

This section is a file-oriented map for explaining what each current package contributes. It covers non-test source, launch, interface, and configuration files under `src`.

### `amr_interfaces`

- `CMakeLists.txt`: invokes ROS 2 interface generation and installs the shared QoS header.
- `msg/BaseStatus.msg`, `HealthStatus.msg`, `ManipulatorStatus.msg`, `FactoryStatus.msg`: status contracts.
- `action/ManipulateProduct.action`, `TransportProduct.action`, `ExecuteProductCycle.action`, `RunSequence.action`, `NavigateStation.action`: long-running project workflows.
- `srv/SetOperationMode.srv`: manual/autonomous mode request.
- `include/amr_interfaces/qos_profiles.hpp`: canonical C++ QoS builders.

### `amr_bringup`

- `config/interface_ownership.yaml`: public topic/action/service ownership and mode-dependent TF ownership.
- `config/qos_profiles.yaml`: named YAML QoS expectations for clock, sensor, state, authority, command, diagnostics, and TF.
- `config/runtime_defaults.yaml`: shared runtime defaults and software contract values.
- `launch/amr_system.launch.py`: ROS-only bringup environment and health entry point.

### `amr_description`

- `urdf/amr.urdf.xacro`: base frame tree, wheel/caster links, sensor frames, Gazebo sensor topics, base simulation interfaces, and base ros2_control inputs.
- `urdf/phase14_mobile_manipulator.urdf.xacro`: composite mobile-manipulator frames, camera, gripper, optional native attachment topics, and ros2_control interfaces.
- `urdf/phase14_kr6_r900_2.urdf.xacro`: arm composition for the Phase 14 path.
- `config/phase14_arm_controllers.yaml`, `phase14_mobile_manipulator_controllers.yaml`: controller-manager configuration.
- `scripts/derive_cad_meshes.py`: offline mesh/geometry preparation utility.
- `launch/phase14_kuka_smoke.launch.py`, `phase14_composite_smoke.launch.py`: smoke-test launch paths.

### `amr_simulation`

- `launch/amr_simulation.launch.py`: starts Gazebo Harmonic, robot state publisher, `ros_gz_bridge`, and the baseline ROS 2 stack.
- `include/command_watchdog.hpp`: watchdog state transitions.
- `src/command_watchdog_system.cpp`: Gazebo plugin implementing native command expiry and backward-time disable.

### `amr_sensor_adapters`

- `src/lidar_adapter_node.cpp`: front/rear generic lidar relay.
- `src/imu_adapter_node.cpp`: IMU relay.
- `src/product_camera_adapter_node.cpp`: paired RGB/depth/camera-info relay with timestamp and resolution checks.

### `amr_base_adapter`

- `src/base_adapter_node.cpp`: managed command, raw odometry, joint-state, and base-status adapter at the base boundary.

### `amr_localization`

- `include/amr_localization/diff_drive.hpp`: differential-drive equations.
- `src/wheel_odometry_node.cpp`: joint-position integration into wheel odometry.
- `config/ekf.yaml`: external robot_localization EKF inputs, output, timing, two-dimensional mode, and TF publication.
- `launch/amr_localization.launch.py`: lifecycle wheel odometry plus EKF startup/remaps.
- `scripts/localization_acceptance.py`: measurement-based localization acceptance utility.

### `amr_perception`

- `include/amr_perception/point_cloud_validation.hpp`: PointCloud2 layout and metadata validation.
- `src/lidar_pipeline_node.cpp`: front/rear validated cloud forwarder.
- `launch/amr_perception.launch.py`: starts the front and rear lifecycle pipeline instances.
- `scripts/perception_fault_acceptance.py`: fault-injection/acceptance observer for malformed or stale perception data.

### `amr_slam`

- `config/mapper.yaml`: external SLAM Toolbox asynchronous mapper configuration, scan input, map update, and TF settings.
- `launch/amr_slam.launch.py`: launches the external SLAM Toolbox node; the package contains configuration/launch rather than a project-owned mapper implementation.

### `amr_navigation`

- `config/planner.yaml`: NavFn planner, global costmap, local costmap-related shared geometry, obstacle layers, inflation, and Simple Smoother settings.
- `launch/amr_navigation.launch.py`: starts planner, smoother, and Nav2 lifecycle manager.

### `amr_mpc_controller`

- `config/controller.yaml`: Nav2 controller server, normal/placement goal checkers, RPP profiles, local costmap, obstacle layers, and velocity/acceleration behavior.
- `launch/amr_mpc_controller.launch.py`: starts the configured Nav2 controller and lifecycle manager.

### `amr_control`

- `src/command_arbitration_node.cpp`: authoritative velocity gate, limits, interlocks, docking egress action, and output command.
- `scripts/prototype_teleop.py`: keyboard test publisher to the upstream command input.
- `config/control.yaml`: command/arbitration parameters.
- `launch/amr_control.launch.py`: lifecycle command arbiter startup and interlock launch argument.

### `amr_mission`

- `include/amr_mission/goal_validation.hpp`: planar finite pose and quaternion validation.
- `src/mission_supervisor_node.cpp`: public navigation action servers and planner/smoother/controller orchestration.
- `launch/amr_mission.launch.py`: managed mission node startup.

### `amr_health`

- `src/health_supervisor_node.cpp`: base evidence validation and health status publication.
- `launch/amr_health.launch.py`: managed health node startup.

### `amr_exploration`

- `scripts/frontier_algorithm.py`: frontier cell detection, clustering, representative selection, costmap feasibility, and blacklist filtering.
- `scripts/frontier_explorer.py`: bounded exploration state machine, readiness checks, mission action delegation, cancellation, and diagnostics.
- `config/frontier_explorer.yaml`: autostart, freshness, failure, goal, and cancellation bounds.
- `launch/frontier_explorer.launch.py`: frontier node startup.

### `amr_factory`

- `src/factory_supervisor_node.cpp`: registry-backed factory action/service orchestration and status.
- `src/gazebo_set_pose_proxy.cpp`: restricted pose-setting service proxy.
- `src/gazebo_control_world_proxy.cpp`: restricted world-control service proxy.
- `tools/generate_apriltag_textures.cpp`: offline tag texture generator.
- `config/products.yaml`: product IDs, masses/dimensions used by software registry, pickup stations, dispatch slots, and autonomous enable flags.
- `config/stations.yaml`: station roles and map-frame approach/dock/egress poses.
- `config/amcl.yaml`: AMCL, map server, and factory localization lifecycle parameters.
- `config/apriltag.yaml`: tag family, station/product IDs, sizes, and detector QoS.
- `maps/factory.yaml` and `maps/factory.pgm`: saved map metadata and image.
- `launch/factory_localization.launch.py`: shared factory simulation/bridge/adapter/localization/control graph with production AMCL or mapping shell.
- `launch/factory_mapping.launch.py`: run-specific mapping session and readiness release graph.
- `launch/factory_autonomous.launch.py`: canonical autonomous static-map factory graph.
- `launch/factory_demo.launch.py`: legacy/optional demonstration composition.
- `scripts/factory_registry.py`: Python registry loading and validation helpers.
- `scripts/factory_cli.py`: operator CLI for listing, mode, enqueue/send, stop, cancel, status, and home requests.
- `scripts/factory_mapping_cli.py`: save, serialize, validate, and discard mapping candidates.
- `scripts/factory_mapping_artifacts.py`: run-specific artifact paths, manifests, hashes, and bundle validation.
- `scripts/factory_mapping_readiness.py`: observation-only mapping readiness gate.
- `scripts/factory_mapping_acceptance.py`: observation-only mapping evidence, human review, and promotion gate logic.
- `scripts/factory_runtime_preflight.py`: host/rendering/device/runtime preflight checks.
- `scripts/gate4_acceptance.py`: Gate 4 acceptance observer.
- `scripts/gate5_camera_acceptance.py`: camera/tag acceptance observer.
- `scripts/gate6_attachment_bootstrap.py`: simulation attachment/bootstrap coordinator.
- `scripts/gate6_bootstrap_inserted_gate.py`, `gate6_bootstrap_pause_gate.py`, `gate6_bootstrap_ready_gate.py`, `gate6_controller_ready_gate.py`: bounded startup/readiness gates for the attachment-enabled simulation path.

### `amr_manipulation`

- `include/amr_manipulation/attachment_gate.hpp` and `src/attachment_gate.cpp`: attachment/detachment evidence gate.
- `include/amr_manipulation/tag_observation_validator.hpp` and `src/tag_observation_validator.cpp`: AprilTag observation stability gate.
- `src/manipulation_supervisor_node.cpp`: public manipulation action/status supervisor used by the compatibility path.
- `src/gate6_empty_motion.cpp`: empty-motion Gate 6 test state machine.
- `src/gate6_mass_stage.cpp`: older, detailed Gate 6 mass/manipulation state machine and acceptance path.
- `scripts/cycle_manipulation_supervisor.py`: current autonomous cycle adapter.
- `scripts/gate6_product_test.py`: product-cycle/Gate 6 execution and evidence runner.
- `scripts/gate6_evidence_analyzer.py`: offline rosbag/evidence analyzer.
- `launch/move_group.launch.py`: MoveIt process launch.
- `launch/gate6_empty_motion.launch.py`, `gate6_mass_stage.launch.py`, `gate6_3kg_test.launch.py`, `gate6_5kg_test.launch.py`: separate Gate 6 test compositions.
- `config/joint_limits.yaml`, `kinematics.yaml`, `moveit_controllers.yaml`, `ompl_planning.yaml`: MoveIt planning and controller configuration.

## 13. What to show on presentation slides

A focused ROS 2 software deck can use the following structure:

| Slide | Topic | Visual/content |
|---:|---|---|
| 1 | ROS 2 software architecture | One-layered graph: sensing -> estimation -> navigation -> arbitration -> base |
| 2 | ROS 2 communication model | Node, topic, service, action, parameter, and lifecycle definitions with one AMR example each |
| 3 | Package ownership | 17-package layer map; distinguish project-owned nodes from external Nav2/SLAM/AMCL/MoveIt nodes |
| 4 | Sensor and perception pipeline | Bridge -> adapters -> validated clouds/camera -> costmaps, SLAM, and AprilTag detector |
| 5 | Localization | Wheel odometry + IMU -> EKF -> fused odometry and `odom -> base_footprint` |
| 6 | Navigation control chain | Mission action -> planner -> smoother -> RPP -> upstream Twist -> arbitration -> stamped command |
| 7 | Interfaces and types | Selected custom messages/actions/services plus key standard ROS types |
| 8 | QoS and timing | Sensor best effort, state reliable, command lifespan, authority deadline, freshness windows |
| 9 | TF tree | `map -> odom -> base_footprint -> base_link -> sensors`, with mapping/factory `map -> odom` owner switch |
| 10 | Differential-drive theory | Equations, `r=0.1128 m`, `L=0.566 m`, one worked translation/rotation example |
| 11 | Safety and fail-closed behavior | Freshness, sequence, interlock, arbitration, base adapter, and watchdog gates |
| 12 | Factory orchestration | CLI/service/action -> factory supervisor -> navigation/cycle adapter -> status/results |
| 13 | Mapping/exploration | SLAM Toolbox, frontier algorithm, bounded mission delegation, manual/autonomous separation |
| 14 | Current status and limits | Implemented source/contract evidence, Product 101/102 scope, and open Phase 15 runtime/map-promotion items |

For a shorter deck, combine slides 2 and 7, combine slides 4 and 5, and combine slides 12 and 13.

## 14. Current status, caveats, and open items

### Implemented in the current source tree

- Typed project interfaces and a central ownership registry exist.
- Stable adapter boundaries exist for base, lidar, IMU, and product camera data.
- Wheel odometry, EKF configuration, Nav2 planning, RPP configuration, mission action orchestration, command arbitration, health observation, factory orchestration, and frontier exploration are represented in source.
- Factory autonomous launch derives enabled product routes from the registry and currently accepts Product 101 and Product 102 while keeping Product 103 disabled.
- Mapping has a separate session directory and explicit manual/autonomous launch modes.
- Mapping artifacts use run-specific paths and manifests rather than automatically replacing the canonical map.

### Current evidence boundary

- Source and contract tests have been used in the project workflow, and the latest documentation recheck verified package/README/build-name consistency, canonical launch references, relative documentation links, and `git diff --check`.
- The current Phase 15 packet is source/offline evidence. It does not prove a fresh runtime mapping run, live per-edge TF ownership, map-quality acceptance, or canonical map promotion.
- Product 101/102 autonomous software scope is documented. Hardware execution and functional-safety acceptance remain outside this report.

### Known items to keep visible in the presentation notes

1. The ownership registry omits the source-defined `/amr/mission/navigate_to_pose_retreat` action. This should be reconciled before calling the registry complete.
2. The `amr_mpc_controller` package name is historical/compatibility naming; the active controller is Nav2 RPP.
3. `amr_machine_controller/src/machine_controller_node.cpp` is legacy orphan code, not an active package.
4. `factory_demo.launch.py` and older Gate 6 nodes remain in the repository for compatibility or evidence, but the canonical autonomous factory path uses `factory_autonomous.launch.py` and the cycle adapter.
5. The current mapping serializer/verifier contract expects a single graph artifact path while the installed serializer may emit a bare-prefix pair (`<prefix>.posegraph` and `<prefix>.data`). This remains a Phase 15 artifact gap.
6. The mapping acceptance observer checks fresh non-faulted readiness evidence but does not yet constitute terminal exploration acceptance or quantitative map-quality approval.

## 15. Source index for slide preparation

The most useful files to open while building the deck are:

- [package ownership table](../src/README.md)
- [interface ownership registry](../src/amr_bringup/config/interface_ownership.yaml)
- [shared QoS code](../src/amr_interfaces/include/amr_interfaces/qos_profiles.hpp)
- [custom message definitions](../src/amr_interfaces/msg)
- [custom action definitions](../src/amr_interfaces/action)
- [base adapter](../src/amr_base_adapter/src/base_adapter_node.cpp)
- [command arbitration](../src/amr_control/src/command_arbitration_node.cpp)
- [wheel odometry](../src/amr_localization/src/wheel_odometry_node.cpp)
- [EKF configuration](../src/amr_localization/config/ekf.yaml)
- [mission supervisor](../src/amr_mission/src/mission_supervisor_node.cpp)
- [Nav2 planner and costmap configuration](../src/amr_navigation/config/planner.yaml)
- [RPP controller configuration](../src/amr_mpc_controller/config/controller.yaml)
- [TF/model source](../src/amr_description/urdf/amr.urdf.xacro)
- [composite model source](../src/amr_description/urdf/phase14_mobile_manipulator.urdf.xacro)
- [frontier node](../src/amr_exploration/scripts/frontier_explorer.py)
- [factory supervisor](../src/amr_factory/src/factory_supervisor_node.cpp)
- [canonical factory launch](../src/amr_factory/launch/factory_autonomous.launch.py)
- [mapping launch](../src/amr_factory/launch/factory_mapping.launch.py)
- [current cycle adapter](../src/amr_manipulation/scripts/cycle_manipulation_supervisor.py)
- [current Phase 15 commissioning boundary](PHASE_15_FACTORY_SLAM_COMMISSIONING.md)
- [current project handoff](../SESSION_HANDOFF.md)

This report is the software-only source of truth for preparing the ROS 2 presentation until the project source or handoff changes.
