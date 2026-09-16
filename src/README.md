# AMR ROS 2 workspace

This workspace uses ROS 2 Humble and C++17. It contains shared interfaces,
simulated adapters, a parameterized
robot description, a Gazebo plant, and local wheel/IMU state estimation. It
makes no external-control, physical-actuator, or functional-safety claim.

The current workspace contains 17 ROS 2 packages. Phase 14 autonomous runtime
acceptance is complete for Product 101 (1 kg) and Product 102 (3 kg). Phase 15
mapping has one accepted non-faulted safe `INCOMPLETE` runtime outcome. Human
map-quality approval, promotion, and canonical-map replacement remain
unverified. Product 103 (5 kg), Gate 7, hardware, and functional-safety claims
are outside scope.

## Package and executable ownership

| Package | Phase | Planned executables/components | Public boundary | Lifecycle | Forbidden responsibility |
| --- | --- | --- | --- | --- | --- |
| `amr_interfaces` | 4 | None | Shared messages/services | N/A | Runtime behavior |
| `amr_bringup` | 4 | `amr_system.launch.py` | Environment, QoS, ownership configuration | N/A | Runtime nodes and host networking |
| `amr_base_adapter` | 5 | `base_adapter_node` | Gated command in; raw odometry/joints/base status out | Managed, separate process | Arbitration or machine authority |
| `amr_sensor_adapters` | 5 | `front_lidar_adapter_node`, `rear_lidar_adapter_node`, `imu_adapter_node` | Independent raw sensor topics out | Managed, separate processes | Estimation or perception policy |
| `amr_description` | 6 | None | Robot description and static TF inputs | N/A | Mechanical CAD |
| `amr_simulation` | 6 | Gazebo/bridge launch | Clock, joints, sensors, simulated plant | Launch-managed, separate processes | Navigation authority |
| `amr_localization` | 7 | `wheel_odometry_node`, configured EKF | Raw joints/IMU in; local odometry/TF out | Wheel node managed; EKF launch-managed, separate processes | Global mapping |
| `amr_perception` | 8 | Independent front/rear pipelines | Raw LiDAR in; navigation perception out | Managed, separate processes | Personnel-safety claims |
| `amr_slam` | 9 | Configured SLAM Toolbox | Perception/local state in; map and `map->odom` out | Managed, separate process | Local-state TF authority |
| `amr_navigation` | 10 | Configured Nav2 servers | Map/perception/pose in; path and behavior requests out | Nav2 lifecycle, separate processes | Base transport authority |
| `amr_mission` | 10 | `mission_supervisor_node` | Mission action boundary | Managed, separate process | Direct motion or goal replay |
| `amr_mpc_controller` | 11 | Nav2 Regulated Pure Pursuit controller configuration | Local path/state in; velocity request out | Nav2 controller lifecycle | Bypassing arbitration/gate |
| `amr_control` | 11 | `command_arbitration_node` | Motion source in; constrained stamped command out | Managed, separate process | Base transport authority |
| `amr_health` | 13 | `health_supervisor_node` | Base diagnostics and freshness evidence | Managed, separate process | Motion or recovery authority |
| `amr_exploration` | 15 | `frontier_explorer.py` | Frontier start/stop services; mission-goal requests | Separate process | Direct velocity or map promotion |
| `amr_factory` | 14/15 | Factory supervisor, CLI, mapping and acceptance tools | Factory actions/status and run-specific mapping artifacts | Separate processes/launch-managed | Direct base velocity or canonical-map replacement |
| `amr_manipulation` | 14 | Cycle adapter, Gate 6 runner, MoveIt launch | Cycle action, manipulation status, attachment proof | Separate processes/launch-managed | Base command ownership or independent status authority |

Packages through Phase 15 are present in the current source tree. The accepted
runtime boundaries are Product 101/102 factory cycles, the existing simulation
stack, and the recorded safe `INCOMPLETE` mapping outcome. Phase 15 human
quality review, promotion, and canonical-map replacement remain separately
gated; automatic recovery, Product 103, Gate 7, and hardware are excluded from
scope.

Portable exploration uses `amr_simulation/portable_stow_authority` as the sole
`/amr/manipulation/status` publisher. That publisher is a portable-runtime
exception and cannot coexist with the factory or Gate 6 manipulation-status
owner in the same ROS graph.

For mapping, `world` is a trusted absolute local SDF 1.9 Gazebo environment;
`/map` is a fresh in-memory SLAM occupancy map created on every launch. No
saved map is loaded or overwritten. `resource_paths` is a colon-separated list
of absolute local model roots. The portable validator rejects direct remote
worlds, `file://` resources, malformed or unversioned Fuel URLs, and unresolved
local model references. AWS model bundles remain remote; use the pinned
OpenRobotics Fuel URLs and the [full simulation command reference](../docs/SIMULATION_COMMANDS.md)
for the network/cache and attribution rules.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## Run the simulation

Build the workspace first, then start the full Gazebo simulation in one
terminal. This legacy launch remains a manual smoke path:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
ros2 launch amr_simulation amr_simulation.launch.py
```

To drive the simulated robot through the normal command-arbitration path, open
a second terminal and run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
ros2 run amr_control prototype_teleop.py
```

Use `W`, `S`, `A`, and `D` to move, `X` or Space to stop, and `Q` to quit.

To view the live SLAM map in RViz, use a third terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/amr_bringup/share/amr_bringup/env/amr_ros_env.sh
rviz2 -d install/amr_simulation/share/amr_simulation/rviz/sensors.rviz \
  --ros-args -p use_sim_time:=true
```

The LiDAR and point-cloud publishers use Best Effort QoS. If their RViz
displays are blank, set each corresponding display's Reliability Policy to
Best Effort. The SLAM map remains available on `/map`.

For world-agnostic autonomous exploration, use the packaged portable launch
with the trusted simple-world SDF:

```bash
ros2 launch amr_simulation portable_exploration.launch.py \
  world:=/home/pete/amr_ws/src/amr_simulation/worlds/amr_world.sdf \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.12 initial_yaw:=0.0 \
  resource_paths:='' headless:=false rviz:=true \
  auto_start_exploration:=true
```

For the canonical AWS warehouse preset:

```bash
ros2 launch amr_simulation aws_warehouse_exploration.launch.py \
  headless:=false rviz:=true auto_start_exploration:=true
```

Both commands autostart exploration after readiness; no separate start-service
call is needed. The AWS source, exact Fuel ownership/revisions, and the local
MIT-0 ownership boundary are recorded in
[`AWS_WAREHOUSE_FUEL_ATTRIBUTION.md`](amr_simulation/assets/AWS_WAREHOUSE_FUEL_ATTRIBUTION.md).

## Autonomous factory cycle

For the accepted Product 101/102 autonomous boundary, use the registry-derived
factory launch with explicit native attachment enabled:

```bash
ros2 launch amr_factory factory_autonomous.launch.py \
  control_mode:=autonomous factory_attachment:=true
```

Start MoveIt separately, then use `ros2 run amr_factory factory_cli.py` for
station selection, sequences, stop/cancel, home, and status. Product 103,
Gate 7, physical hardware, and functional-safety acceptance are outside scope.
`factory_demo.launch.py` remains legacy/optional. Phase 15 mapping uses the
separate `factory_mapping.launch.py` entry point; its recorded runtime boundary
is a safe `INCOMPLETE` outcome, while human map-quality approval, promotion,
and canonical-map replacement remain unverified.
