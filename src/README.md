# AMR ROS 2 workspace

This workspace uses ROS 2 Humble and C++17. It contains shared interfaces,
simulated adapters, a parameterized
robot description, a Gazebo plant, and local wheel/IMU state estimation. It
makes no external-control, physical-actuator, or functional-safety claim.

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

Packages through Phase 11, the Phase 13 health package, and the Phase 14
factory/manipulation source boundaries are implemented. Gate 6/Gate 7 runtime
acceptance remains separately gated; automatic recovery is excluded from scope.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## Run the simulation

Build the workspace first, then start the full Gazebo simulation in one
terminal:

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
