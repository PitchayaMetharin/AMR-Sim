# Phase 0 Software Baseline

## Approved baseline

| Item | Baseline | Status |
|---|---|---|
| Operating system | Ubuntu 22.04 LTS | Confirmed |
| ROS | ROS 2 Humble | Confirmed |
| Production language | C++17 minimum | Confirmed by the ROS 2 Humble platform requirement |
| Navigation | Nav2 | Confirmed |
| State estimation | `robot_localization` EKF | Confirmed |
| Mapping/localization | SLAM Toolbox | Confirmed |
| Local controller | MPC | Confirmed; implementation approach TBD |
| Manipulation | MoveIt 2 only if a later manipulator or motion-planning use case requires it | Installed but outside the initial mobile-base scope |
| Simulator | Gazebo Harmonic 8.14.0 | Confirmed Phase 6 baseline; installed and server smoke-tested |
| ROS/Gazebo integration | `ros-humble-ros-gzharmonic` 0.244.12-3jammy | Installed; `/clock` and sensor-type bridges tested |

ROS 2 Humble officially targets Ubuntu 22.04 on both amd64 and arm64 and
requires C++17:

- [ROS 2 Humble supported platforms and language requirements](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)

## Workstation evidence

Inspected on 2026-07-24:

| Item | Observed value |
|---|---|
| Host architecture | x86_64 development workstation |
| Ubuntu | 22.04.5 LTS |
| ROS environment | ROS 2 Humble |
| RMW | `rmw_fastrtps_cpp` |
| ROS desktop package | `ros-humble-desktop` 0.10.0 |
| Nav2 | `ros-humble-navigation2` and `nav2-bringup` 1.1.20 |
| MoveIt 2 | `ros-humble-moveit` 2.5.9 |
| Installed simulators | Gazebo Harmonic 8.14.0 and Gazebo Classic 11.10.2 |
| Gazebo Harmonic metapackage | `gz-harmonic` 1.0.0-1~jammy |
| Gazebo simulation library | `libgz-sim8` 8.14.0-1~jammy |
| ROS/Gazebo bridge | Explicit Harmonic bridge 0.244.12-3jammy installed |
| Joint-state publisher | Publisher and GUI 2.4.0 installed and load-tested |
| Compiler | GCC/G++ 11.4.0 |
| CMake | 3.22.1 |

The laptop workstation is the permanent and only execution target.

## Simulator lifecycle decision

Gazebo Harmonic and Gazebo Classic 11 are both installed. Gazebo Classic is no
longer listed among currently supported Gazebo releases and is not the project
baseline. ROS 2 Humble's official modern pairing is Gazebo Fortress; Gazebo
Harmonic can be used with Humble through non-default packages but is marked
“use with caution.” Harmonic is supported through September 2028.

Sources:

- [Current ROS/Gazebo compatibility guidance](https://gazebosim.org/docs/jetty/ros_installation/)
- [Gazebo release lifecycle](https://gazebosim.org/docs/harmonic/releases/)

Gazebo Harmonic is selected for Phase 6 because it has an LTS lifecycle through
September 2028. ROS 2 Humble/Harmonic is a supported possible combination but
not Humble's default pairing. The explicit `ros-humble-ros-gzharmonic`
packages are installed; the generic conflicting `ros-humble-ros-gz` package is
not installed.

On 2026-07-24, `gz sim --version` reported 8.14.0. A headless server loaded the
installed `empty.sdf`, initialized its 1 ms physics profile, and remained
healthy until intentionally terminated after eight seconds. That initial smoke
test did not exercise a project world, robot model, GUI, sensor, or ROS bridge.

Subsequent Phase 0 environment testing verified Gazebo transport/control,
GUI/OGRE2 startup, RViz OpenGL 4.6 startup, C++17/ament compilation, ROS 2 node
runtime, Fast DDS delivery, SDFormat inertia calculation, Gazebo-to-ROS
`/clock`, and LaserScan/PointCloud2/IMU bridge type support. No project robot
model or generated sensor stream was tested. See
[`PHASE_0_TEST_REPORT.md`](PHASE_0_TEST_REPORT.md).
