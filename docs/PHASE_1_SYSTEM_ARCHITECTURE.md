# Phase 1 System Architecture

The active simulation route is Nav2 Regulated Pure Pursuit (RPP) → command
arbitration → base adapter → Gazebo plant. The compatibility package and topic
names retain `amr_mpc_controller` and `/amr/mpc/cmd_vel`; they do not indicate
an MPPI/MPC controller implementation. Arbitration and the base adapter use
steady-clock freshness;
the native Gazebo watchdog independently disables the plant after 200 ms
without command traffic. The architecture is limited to laptop simulation.
