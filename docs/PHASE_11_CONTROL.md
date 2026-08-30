# Phase 11 Control

The active motion route is mission → planning → Nav2 Regulated Pure Pursuit →
command arbitration → base adapter → Gazebo plant. RPP targets a provisional
0.50 m/s cruise and provides curvature and goal-approach regulation; arbitration
still owns planar validation, speed/acceleration limits, source freshness, and
the sole `/amr/control/cmd_vel` output. The base adapter and native Gazebo
watchdog independently stop stale command traffic.
