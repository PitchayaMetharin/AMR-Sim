# ROS 2 Software Architecture

## 10-minute speaker notes

This script follows `ROS2_Software_Architecture_10min_v2.pptx` and the Canva presentation. It is written for a software-only presentation. Hardware, mechanical design and functional-safety acceptance are outside the scope.

### Timing plan

| Slide | Topic | Time |
|---:|---|---:|
| 1 | Scope and system story | 0:30 |
| 2 | Package structure | 0:55 |
| 3 | Interfaces and communication | 1:05 |
| 4 | Gazebo, bridge, RViz and adapters | 0:55 |
| 5 | Localization and TF | 1:10 |
| 6 | Nav2 pipeline | 1:05 |
| 7 | MoveIt and factory orchestration | 1:00 |
| 8 | Command authority and fail-closed motion | 1:10 |
| 9 | Theory and calculations | 1:20 |
| 10 | Current state and close | 0:50 |
| **Total** |  | **10:00** |

## Slide 1 — ROS 2 Software Architecture

### Purpose

Introduce the software boundary and give the audience one mental model for the rest of the talk.

### What to say

“This presentation covers the ROS 2 software stack only. The project is an autonomous mobile manipulator, but the important software idea is that it is a distributed, typed ROS 2 graph. Different parts of the graph handle sensing, state estimation, planning, command authority and motion execution.

The flow at the bottom is the story for the whole presentation. Sensor data becomes estimated state. Planning components turn goals into requests. The authority layer checks whether a request is allowed to reach the base. Finally, the simulator or base endpoint receives the accepted command.

Nav2 and MoveIt can request work, but neither one independently owns safe base motion. The project-owned command path is the point where software policy and freshness checks are enforced.”

### Point at

- The highlighted **Authority** box: it is the control boundary between planning and motion.
- The words **Software scope only**: hardware details are intentionally excluded.

### Transition

“To understand that flow, the next slide maps responsibilities to the packages in the workspace.”

## Slide 2 — System structure and package ownership

### Purpose

Show how the source tree is divided and distinguish project-owned packages from external ROS 2 stacks.

### What to say

“The workspace contains 17 ROS 2 packages, grouped here by responsibility rather than by build order.

At the top, `amr_interfaces` and `amr_bringup` define the public contracts and startup policy. `amr_description` and `amr_simulation` define the robot model and the simulator boundary. The adapter and state packages convert sensor and base data into stable project topics and maintain the estimated robot state.

The mapping and navigation group contains SLAM, Nav2 configuration, the controller package and the mission supervisor. The final group owns command arbitration, health reporting, exploration, factory orchestration and manipulation.

The list on the right contains external components that the project configures or consumes: Nav2, `robot_localization`, SLAM Toolbox or AMCL, MoveIt, AprilTag ROS and `ros_gz_bridge`. These are important parts of the running system, but the project still needs explicit ownership around their interfaces.

One source directory, `amr_machine_controller`, is shown as legacy. It is not an active package because it has no package manifest or CMake entry. That distinction prevents us from describing unused code as part of the deployed graph.”

### Point at

- Read the five project groups from top to bottom.
- Point to the orange legacy box when explaining active versus inactive source.

### Transition

“The ownership groups become meaningful through the interfaces that connect their nodes.”

## Slide 3 — ROS 2 interfaces and communication

### Purpose

Explain the four communication concepts the audience needs: topics, services, actions and lifecycle nodes.

### What to say

“ROS 2 communication is built around named, typed endpoints.

The first card is a topic. A topic carries a continuous stream, such as `/amr/localization/odometry` with type `nav_msgs/Odometry`. Multiple consumers can subscribe without directly calling the publisher.

The second card is a service. `/amr/factory/set_operation_mode` uses a project service type for a bounded request and response. This is appropriate for a short state change.

The third card is an action. `/amr/factory/run_sequence` uses `RunSequence`, which supports a goal, feedback, a final result and cancellation. That makes it appropriate for work that can take time and may need to be stopped.

The last card is the lifecycle model. Adapters, estimation nodes and Nav2 components use controlled startup states such as configure, inactive and active. This prevents downstream nodes from consuming data before an upstream component is ready.

DDS performs discovery, but discovery alone is not enough. The endpoint name, message type, QoS profile and declared owner must agree. QoS controls properties such as reliability, durability, queue depth, deadline and lifespan. The project status messages add freshness information such as boot IDs and sequence values so that consumers can reject stale observations.”

### Point at

- Read one concrete topic, one service and one action rather than all endpoint names.
- Point to “DDS discovery + compatible QoS” as the connection between nodes.
- Point to the status-message line to explain that state is more than a Boolean.

### Transition

“The next slide follows those interfaces through the simulation boundary and into the project’s stable `/amr` namespace.”

## Slide 4 — Gazebo, ros_gz_bridge, RViz2 and adapters

### Purpose

Explain the simulation data path and the role of visualization tools.

### What to say

“Gazebo Harmonic is the simulated world. It produces simulator-native sensor, joint, odometry and base endpoints. `ros_gz_bridge` converts those messages between Gazebo Transport and ROS 2.

The project does not want every downstream node to depend on simulator-specific names. Lifecycle adapters translate the bridge output into stable project interfaces under `/amr/sensors` and `/amr/base`. Localization, perception, SLAM, Nav2 and AprilTag processing consume those stable interfaces.

The product camera adapter also checks that an image and its `CameraInfo` share the expected timestamp relationship and resolution. This matters because perception results are only useful if the calibration and image correspond to the same observation.

RViz2 is a visualization and inspection tool. It subscribes to maps, TF, robot models, scans, clouds and paths. It does not own accepted velocity commands. Factory acceptance can run headless, so RViz is useful for diagnosis but is not part of the motion-authority chain.”

### Point at

- Trace the main arrow from Gazebo to the bridge, then to adapters and consumers.
- Point to the RViz box and say “visualization only.”

### Transition

“Once the adapters provide consistent sensor data, the localization stack estimates motion and publishes the TF relationships used by planning.”

## Slide 5 — Localization and TF ownership

### Purpose

Explain how wheel and IMU data become odometry, then show the expected TF tree and its ownership.

### What to say

“The localization path starts with wheel joint states. `wheel_odometry_node` converts wheel motion into an odometry estimate. The EKF from `robot_localization` fuses that wheel information with IMU data at 30 Hz in two-dimensional mode.

The EKF publishes `/amr/localization/odometry` and owns the dynamic transform from `odom` to `base_footprint`. That is the local, continuously updated motion estimate.

The TF tree below separates global localization from local motion. In mapping mode, SLAM Toolbox provides `map` to `odom`. In factory mode, AMCL provides that same global-to-local relationship. The EKF owns `odom` to `base_footprint`. `robot_state_publisher` supplies the fixed robot model from `base_footprint` through `base_link` and down to the lidars, IMU, camera and arm.

The key design rule is one owner per transform edge. If two components publish the same edge, the graph may look connected while the pose is inconsistent or changes unexpectedly.”

### Point at

- Point to the EKF box and then the green odometry output.
- Trace the TF row from `map` to sensors.
- Emphasize the labels “SLAM Toolbox or AMCL,” “EKF” and “robot_state_publisher.”

### Transition

“Nav2 consumes this map, pose and perception data to turn a mission goal into a velocity request.”

## Slide 6 — Nav2 navigation pipeline

### Purpose

Show how a goal becomes a path and then an upstream velocity request, while clarifying the controller naming.

### What to say

“The mission supervisor first validates that the requested goal is planar and sends the navigation work into Nav2.

NavFn computes a global path using the map and global costmap. The Simple Smoother refines that path. The Regulated Pure Pursuit controller, or RPP, follows the path and produces the upstream request on `/amr/mpc/cmd_vel` as `geometry_msgs/Twist`.

The controller profile shown here is 0.5 meters per second desired speed, a 0.3 to 0.9 meter lookahead range, 1.5 seconds of lookahead time and a 1.2 by 0.8 meter software footprint. The front and rear PointCloud2 streams contribute obstacle information to the costmap and controller path.

There is a naming detail worth calling out. The package is named `amr_mpc_controller` for compatibility, but the active algorithm in the current configuration is Regulated Pure Pursuit. This slide ends at a request topic. It does not mean Nav2 has permission to move the base.”

### Point at

- Trace mission supervisor → NavFn → smoother → RPP → `/amr/mpc/cmd_vel`.
- Point to the orange note about compatibility naming.

### Transition

“Navigation is one part of the application. The factory layer combines station navigation with manipulation into a longer-running product cycle.”

## Slide 7 — MoveIt and factory orchestration

### Purpose

Explain the application-level sequence and how MoveIt fits without owning the whole mission.

### What to say

“The operator or CLI sends a request to the factory supervisor. The factory supervisor owns the product and station registries, queues work and publishes factory status.

For a product cycle, the factory coordinates two major boundaries. The mission and Nav2 path move the mobile base to a station. The cycle supervisor then coordinates the manipulation stage through MoveIt’s `move_group`, which plans and executes arm trajectories.

The public factory interfaces include product transport, sequence execution, station navigation, operation-mode changes and stop or cancel behavior. The manipulation stage still has evidence gates. These include tag identity, stable observations, contact freshness, pose tolerance and attachment state. The gates prevent the application from declaring success from a single ambiguous sensor observation.

The current autonomous product scope is Product 101 and Product 102. Product 103 remains disabled, so it should not be presented as an accepted autonomous capability.”

### Point at

- Trace Operator/CLI → Factory supervisor → Mission/Nav2 and Cycle supervisor → MoveIt.
- Point to the green and orange product boxes.

### Transition

“All of these planners and application actions still need a final decision point before a velocity can reach the base.”

## Slide 8 — Command authority and fail-closed motion

### Purpose

Make the safety-relevant software boundary clear: planning requests are checked before execution.

### What to say

“This is the most important control boundary in the software architecture.

Nav2 publishes an upstream `Twist` request. Command arbitration checks command freshness, planar validity, manipulator permission, base readiness and configured velocity limits. If the evidence is admissible, it publishes the project-owned accepted command as `/amr/control/cmd_vel` with `TwistStamped`.

The base adapter validates the command again and outputs zero when the command is stale or invalid. The bridge and Gazebo side also have a watchdog that expires a command if fresh input stops arriving. These checks are intentionally layered so that one stale publisher cannot keep the base moving indefinitely.

The current timing and limit values are 20 Hz arbitration output, 200 milliseconds for command timeout and lifespan, 0.5 meters per second linear speed, 0.4 radians per second angular speed and a nominal 0.025 meters per second command change per 20 Hz tick.

The final rule is fail-closed: missing, stale, invalid or contradictory evidence stops or rejects motion. Health reporting can describe a problem, but it cannot enable motion.”

### Point at

- Trace the five boxes from Nav2 request to the Gazebo watchdog.
- Point to “Sole project-owned velocity publisher.”
- Point to the orange sentence at the bottom and pause there.

### Transition

“The numerical limits are easier to understand when connected to the underlying robot geometry and timing calculations.”

## Slide 9 — Theory and project calculations

### Purpose

Connect the formulas to the implementation and show how current configuration values produce concrete software behavior.

### What to say

“The left side shows the differential-drive equations used by wheel odometry. Linear velocity is the wheel radius divided by two, multiplied by the sum of left and right angular wheel velocities. Angular velocity is the radius divided by the track width, multiplied by the difference between right and left wheel velocities.

The configured wheel radius is 0.1128 meters and the track width is 0.566 meters. If both wheels move by plus one radian, the robot translates by one wheel radius, 0.1128 meters, with zero heading change. If the wheels move in opposite directions by plus or minus one radian, translation cancels and the heading change is approximately 0.3986 radians.

On the right, a five-meter local costmap at 0.05-meter resolution gives 5 divided by 0.05, or 100 cells per side. The 0.55-meter inflation radius gives 0.55 divided by 0.05, or 11 cells. RPP’s 0.5 meters per second speed and 1.5 seconds of lookahead time give a nominal lookahead distance of 0.75 meters.

Finally, at 20 Hz one tick is 0.05 seconds. With a 0.5 meters-per-second-squared acceleration limit, the nominal velocity change is 0.5 multiplied by 0.05, or 0.025 meters per second per tick.

These are not new thresholds invented for the presentation. They explain how the current source parameters affect odometry, costmap geometry, controller behavior and command arbitration.”

### Point at

- Read the two equations slowly.
- Point to the two wheel-motion examples.
- On the right, say each division or multiplication as it appears.

### Transition

“The final slide separates what the source implements from what still needs fresh runtime evidence.”

## Slide 10 — Current software state

### Purpose

Close accurately without overstating project readiness.

### What to say

“The left column summarizes software that is implemented in the source tree: typed interfaces and ownership, adapters and localization, the TF model, the Nav2 RPP pipeline, command arbitration and factory cycles for Products 101 and 102.

The right column lists claims that still require fresh evidence. Phase 15 still needs a new mapping runtime proof, per-edge TF ownership and freshness evidence, quantitative map-quality acceptance and an explicit candidate-map promotion step. Hardware and functional-safety acceptance are also outside this software presentation and remain pending.

There is one contract gap to keep visible. The retreat action `/amr/mission/navigate_to_pose_retreat` exists in source but is absent from the interface ownership registry. Until that is reconciled, the public contract is incomplete.

The main takeaway is simple: the project connects typed interfaces to explicit ownership, and it gates motion with freshness and interlock checks. That is the software architecture.”

### Point at

- Contrast the green “implemented in source” column with the orange “pending evidence” column.
- Point to the contract-gap box.
- End on the blue sentence at the bottom.

## Delivery reminders

- Keep the flow conversational. Do not read every package name or every standard message type.
- Use the arrows as the narrative spine: data enters through adapters, becomes state, feeds planning, passes authority and reaches motion.
- Say “request” for Nav2 and MoveIt outputs. Reserve “accepted command” for `/amr/control/cmd_vel`.
- Do not call `amr_mpc_controller` an MPC algorithm. The active controller is Regulated Pure Pursuit.
- Do not claim that RViz owns motion, that Product 103 is enabled or that mapping has been runtime-accepted.
- If time is short, compress Slides 2 and 4 first. Keep Slides 5, 8 and 9 because they connect the architecture, safety boundary and calculations.

## Source basis

- `docs/ROS2_SOFTWARE_PRESENTATION_REPORT.md`
- `docs/ROS2_10_MIN_PRESENTATION_BRIEF.md`
- `docs/ROS2_10_MIN_PRESENTATION_BY_SOFTWARE.md`
- `src/amr_bringup/config/interface_ownership.yaml`
- `src/amr_localization/src/wheel_odometry_node.cpp`
- `src/amr_localization/include/amr_localization/diff_drive.hpp`
- `src/amr_localization/config/ekf.yaml`
- `src/amr_navigation/config/planner.yaml`
- `src/amr_mpc_controller/config/controller.yaml`
- `src/amr_control/src/command_arbitration_node.cpp`
- `src/amr_base_adapter/src/base_adapter_node.cpp`
- `src/amr_factory/src/factory_supervisor_node.cpp`
- `src/amr_manipulation/scripts/cycle_manipulation_supervisor.py`
- `src/amr_simulation/src/command_watchdog_system.cpp`
