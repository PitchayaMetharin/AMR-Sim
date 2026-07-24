# Phase 0 Environment and Integration Test Report

## Scope

This report records tests that can reduce future integration risk before a
project robot model or ROS 2 workspace exists. It does not validate AMR
dynamics, navigation, sensors, payload stability, or physical safety.

Test date: 2026-07-24

## Result Summary

| ID | Test | Result | Evidence |
|---|---|---|---|
| P0-T001 | Host resource inventory | Pass | 16 logical CPUs, 31 GiB RAM with 24 GiB available, 132 GiB workspace storage available |
| P0-T002 | ROS 2 environment and health | Pass with observations | ROS 2 Humble, Fast DDS, valid network interfaces; no active graph during the report |
| P0-T003 | Required ROS package inventory | Pass | Core description, control, localization, SLAM, Nav2, MPPI, MoveIt, Harmonic bridge, and joint-state publisher packages present |
| P0-T004 | C++17 `ament_cmake` build | Pass | Temporary package compiled with GCC 11.4 using `-Wall -Wextra -Wpedantic -Werror` |
| P0-T005 | C++ ROS 2 node runtime | Pass | Temporary `rclcpp` node emitted `AMR_PHASE0_CPP17_SMOKE_PASS` |
| P0-T006 | Fast DDS discovery and delivery | Pass | Isolated C++ talker/listener delivered all 6 of 6 published messages |
| P0-T007 | `rosdep` dependency resolution | Pass with warning | All temporary-package dependencies satisfied; Python `pkg_resources` deprecation warning observed |
| P0-T008 | Gazebo Harmonic headless startup | Pass | Gazebo Sim 8.14.0 loaded `empty.sdf` and initialized a 1 ms physics profile |
| P0-T009 | Gazebo transport topics/services | Pass | Clock, statistics, scene, state, pose, resource, spawn, remove, physics, and control endpoints present |
| P0-T010 | Gazebo world-control request | Pass | Pause request returned `data: true`; statistics reported `paused: true` |
| P0-T011 | Gazebo GUI and renderer startup | Pass with warning | GUI 8.14.0 loaded OGRE2 and standard plugins; one non-fatal QML binding-loop warning |
| P0-T012 | RViz 2 rendering startup | Pass | RViz reported OpenGL 4.6 / GLSL 4.6; stereo rendering unsupported informational message |
| P0-T013 | SDFormat validation | Pass | Installed empty world and temporary inertia fixture validated successfully |
| P0-T014 | Automatic inertia calculation | Pass | Known box fixture returned expected 100 kg mass and analytical inertia values |
| P0-T015 | Harmonic integration installation dry run | Pass with required actions | Exact Humble/Harmonic and joint-state package set resolves with 22 new packages, zero upgrades, and zero removals |
| P0-T016 | Phase 0 documentation/parameter integrity | Pass | Unique parameter IDs, no open Phase 0 checklist item, valid BOM archive, empty project `src/` |
| P0-T017 | Required package installation | Pass | 22 packages installed; requested metapackages configured; package audit clean |
| P0-T018 | Gazebo-to-ROS simulation clock | Pass | Harmonic bridge delivered a live Gazebo timestamp on ROS 2 `/clock` |
| P0-T019 | Sensor bridge type support | Pass | LaserScan, PointCloud2, and IMU Gazebo-to-ROS mappings initialized |
| P0-T020 | Joint-state tools | Pass | Publisher and GUI Python modules imported and both executables displayed their interfaces |
| P0-T021 | Gazebo model-spawn interface | Pass | `ros_gz_sim create` loaded and exposed file, parameter, topic, string, name, pose, and world arguments |

## Host Baseline

| Item | Observed |
|---|---|
| CPU | Intel Core i7-1260P |
| Logical CPUs | 16 |
| Memory | 31 GiB total; 24 GiB available during test |
| Swap | 2 GiB total; unused during test |
| Workspace storage | 174 GiB filesystem; 132 GiB available |
| Graphics | Intel Alder Lake-P integrated graphics |
| Display session | X11 on display `:0` |
| Compiler | GCC/G++ 11.4.0 |
| CMake | 3.22.1 |
| `colcon-core` | 0.21.0 |
| ROS | ROS 2 Humble |
| RMW | `rmw_fastrtps_cpp` |

These resources are adequate for initial development and empty-world
simulation. Performance with two simulated multi-layer LiDARs, SLAM, Nav2,
MPC, RViz, logging, and a detailed physics model must be measured later.

## ROS 2 and C++ Evidence

A temporary `ament_cmake` package was created under `/tmp`, outside the project
workspace. It:

- required C++17;
- used `rclcpp`;
- enabled `-Wall`, `-Wextra`, `-Wpedantic`, and `-Werror`;
- configured, compiled, linked, and installed successfully;
- ran successfully and initialized Fast DDS.

An isolated ROS domain was used for the C++ talker/listener test. Six messages
were published and all six were received before the test processes were
intentionally terminated.

`rosdep check` reported that all dependencies for the temporary package were
satisfied. Its `pkg_resources` deprecation warning should be monitored but does
not block the current toolchain.

## Required Package Inventory

Present:

- `rclcpp`
- `ament_cmake`
- `xacro`
- `urdf`
- `robot_state_publisher`
- `controller_manager`
- `diff_drive_controller`
- `robot_localization`
- `slam_toolbox`
- `nav2_bringup`
- `nav2_mppi_controller`
- `moveit_core`
- `check_urdf`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `ros_gz_sim`
- `ros_gz_bridge`

The previously missing Harmonic and joint-state packages were installed after
the user explicitly authorized the system change.

`ros2 doctor` listed newer upstream patch versions for several ROS packages,
while APT reported no locally available ROS upgrades and the sampled installed
packages matched their configured APT candidates. This is not an inconsistent
partial upgrade, but package versions shall still be pinned in the future
reproducible environment record.

## Gazebo and Rendering Evidence

Gazebo Harmonic 8.14.0 successfully:

- loaded its installed empty world;
- initialized physics at a 1 ms step;
- published root and world clock/statistics topics;
- exposed world state, scene, pose, spawn, remove, physics, collision, and
  control services;
- accepted a pause request and reported the paused state;
- started the GUI;
- loaded the OGRE2 renderer and normal GUI plugins.

The GUI emitted one QML `RowLayout` binding-loop warning from the world
statistics plugin. It did not prevent startup or world control and is recorded
as a watch item.

RViz 2 started successfully and reported OpenGL 4.6 / GLSL 4.6. The
informational `Stereo is NOT SUPPORTED` message is not a blocker for the AMR
workflow.

## SDFormat and Inertia Evidence

A temporary SDFormat 1.11 box fixture used:

- dimensions: 1.0 × 0.5 × 0.2 m;
- density: 1000 kg/m³;
- expected mass: 100 kg.

Gazebo validated the file and calculated:

- mass: 100 kg;
- center of mass: (0, 0, 0);
- `Ixx`: 2.41666667 kg·m²;
- `Iyy`: 8.66666667 kg·m²;
- `Izz`: 10.4166667 kg·m².

These match the analytical solid-box inertia equations. The tooling needed to
check later payload and chassis inertias is therefore operational.

## Installation and Verification

The correct explicit integration candidate for the selected pairing is:

`ros-humble-ros-gzharmonic` version `0.244.12-3jammy`

Before installation, the package-manager simulation proposed:

- 22 new packages;
- 0 upgrades;
- 0 removals;
- 0 held packages affected.

Important conflict rule:

- `ros-humble-ros-gzharmonic` conflicts with `ros-humble-ros-gz` and
  `ros-humble-ros-gzgarden`.
- The generic/default `ros-humble-ros-gz` package must not be installed in
  parallel with the explicit Harmonic package.

The authorized installation subsequently added those 22 packages, downloaded
approximately 5 MB, and used approximately 42 MB of disk. It performed no
upgrade or removal. Installed primary package versions:

- `ros-humble-ros-gzharmonic`: `0.244.12-3jammy`;
- `ros-humble-ros-gzharmonic-bridge`: `0.244.12-3jammy`;
- `ros-humble-ros-gzharmonic-sim`: `0.244.12-3jammy`;
- `ros-humble-joint-state-publisher`:
  `2.4.0-1jammy.20260605.143758`;
- `ros-humble-joint-state-publisher-gui`:
  `2.4.0-1jammy.20260605.152847`.

`dpkg --audit` reported no incomplete package state. The conflicting
`ros-humble-ros-gz` and `ros-humble-ros-gzgarden` packages remain absent.

End-to-end bridge validation started Gazebo Harmonic and the installed
`parameter_bridge`. ROS 2 received:

```text
clock:
  sec: 4
  nanosec: 365000000
```

The bridge also accepted the future sensor mappings:

- `gz.msgs.LaserScan` to `sensor_msgs/msg/LaserScan`;
- `gz.msgs.PointCloudPacked` to `sensor_msgs/msg/PointCloud2`;
- `gz.msgs.IMU` to `sensor_msgs/msg/Imu`.

## Findings Requiring Action

### Before Phase 6 integration

1. Preserve the explicit Harmonic package selection; do not install conflicting
   generic or Garden bridge metapackages.
2. Convert the validated `/clock` and sensor-type checks into reproducible
   project tests when the workspace exists.
3. Validate actual image, point-cloud, laser-scan, IMU, TF, and spawn data once
   the primitive robot model exists.

### During Phase 6

1. Add automated Xacro, URDF, SDF, TF, mass, and inertia checks.
2. Verify the 50 kg default payload produces approximately 80 kg total moving
   mass.
3. Build the first robot from parameterized primitive shapes; final CAD is not
   a prerequisite.
4. Measure real-time factor and CPU/memory use with both simulated MRS1000
   streams, IMU, RViz, and logging active.
5. Treat the Gazebo QML warning as non-blocking unless it causes GUI
   instability.

## Conclusions Not Supported by These Tests

These passes do not demonstrate:

- correct robot geometry or TF;
- wheel/caster contact stability;
- payload stability or braking;
- sensor fidelity;
- SLAM, localization, navigation, or MPC performance;
- physical structural, electrical, thermal, traction, or safety capability.
