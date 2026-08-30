# Phase 3 Communication Architecture

All communication is local ROS 2 and Gazebo transport on one laptop. The base
adapter bridges `/amr/simulation/base/cmd_vel` to Gazebo, and the native plant
watchdog validates command liveness independently. Physical networking,
fieldbus, and external control systems are out of scope.
