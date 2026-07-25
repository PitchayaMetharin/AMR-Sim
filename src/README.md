# AMR ROS 2 workspace

This workspace uses ROS 2 Humble and C++17. Phase 4 creates only shared
interfaces and bringup/configuration ownership; no driver, simulation, control,
or PLC behavior belongs here yet.

## Package and executable ownership

| Package | Phase | Planned executables/components | Public boundary | Lifecycle | Forbidden responsibility |
| --- | --- | --- | --- | --- | --- |
| `amr_interfaces` | 4 | None | Shared messages/services | N/A | Runtime behavior |
| `amr_bringup` | 4 | `amr_system.launch.py` | Environment, QoS, ownership configuration | N/A | Runtime nodes and host networking |
| `amr_base_adapter` | 5 | `base_adapter_node` | Gated command in; raw odometry/joints/base status out | Managed, separate process | Arbitration or PLC authority |
| `amr_sensor_adapters` | 5 | `front_lidar_adapter_node`, `rear_lidar_adapter_node`, `imu_adapter_node` | Independent raw sensor topics out | Managed, separate processes | Estimation or perception policy |
| `amr_plc_gateway` | 5 | `plc_gateway_node` | OPC UA requests/state and gateway services | Managed, separate process | PLC/HMI program or permission grant |
| `amr_description` | 6 | None | Robot description and static TF inputs | N/A | Mechanical CAD |
| `amr_simulation` | 6 | Gazebo/bridge launch | Clock, joints, sensors, simulated plant | Launch-managed, separate processes | Navigation or permission |
| `amr_localization` | 7 | `wheel_odometry_node`, configured EKF | Raw odometry/IMU in; local odometry/TF out | Managed, separate processes | Global mapping |
| `amr_perception` | 8 | Independent front/rear pipelines | Raw LiDAR in; navigation perception out | Managed, separate processes | Personnel-safety claims |
| `amr_slam` | 9 | Configured SLAM Toolbox | Perception/local state in; map and `map->odom` out | Managed, separate process | Local-state TF authority |
| `amr_navigation` | 10 | Configured Nav2 servers | Map/perception/pose in; path and behavior requests out | Nav2 lifecycle, separate processes | Permission authority |
| `amr_mission` | 10 | `mission_supervisor_node` | Mission action boundary | Managed, separate process | Direct motion or goal replay |
| `amr_mpc_controller` | 11 | Nav2 MPC controller plugin | Local path/state in; velocity request out | Nav2 controller lifecycle | Bypassing arbitration/gate |
| `amr_control` | 11 | `command_arbitration_node`, `motion_gate_node` | Motion sources/PLC state in; sole request/gated outputs | Managed, separate processes | PLC permission grant |
| `amr_health` | 13 | `health_supervisor_node` | Diagnostics and lifecycle/freshness evidence | Managed, separate process | PLC authority override |

The Phase 4 packages are the only packages created now. Future package names
are reserved assignments, not implementation authorization.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```
