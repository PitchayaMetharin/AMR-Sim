# Phase 1 System Architecture

The active simulation route is Nav2 MPPI → command arbitration → base adapter
→ Gazebo plant. Arbitration and the base adapter use steady-clock freshness;
the native Gazebo watchdog independently disables the plant after 200 ms
without command traffic. The architecture is limited to laptop simulation.
