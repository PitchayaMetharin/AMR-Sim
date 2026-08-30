# Changelog

## Unreleased

### Changed

- Removed the former simulated-permission subsystem, its interfaces, gate, and
  acceptance tools.
- The simulation command route is Nav2 MPPI → command arbitration → base
  adapter → Gazebo plant, protected by adapter and native plant watchdogs.
- The workspace now contains fourteen ROS 2 packages and 101 automated tests.
