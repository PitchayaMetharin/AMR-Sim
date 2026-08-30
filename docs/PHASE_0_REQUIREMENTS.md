# Phase 0 Requirements

The project is a one-laptop Ubuntu ROS 2 Humble/C++17/Gazebo simulation of a
differential-drive AMR. The only command route is mission or teleop input →
command arbitration → base adapter → simulated plant. Stale or malformed
commands produce zero velocity.

Physical construction, procurement, hardware integration, fieldbus work, and
industrial deployment are excluded.
