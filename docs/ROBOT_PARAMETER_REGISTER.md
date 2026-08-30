# Robot Parameter Register

| ID | Area | Value | Status |
| --- | --- | --- | --- |
| SW-001 | Runtime | Ubuntu 22.04, ROS 2 Humble, C++17, Gazebo simulation | Active |
| SYS-001 | Command source timeout | 200 ms steady-time limit | Active simulation only |
| SYS-002 | Base-adapter timeout | 200 ms steady-time limit | Active simulation only |
| SYS-003 | Native plant watchdog | 200 ms simulation-time limit | Active simulation only |
| URDF-001 | Drive wheel radius | 0.1128 m | Provisional URDF/simulation only |
| URDF-002 | Drive wheel separation | 0.566 m | Provisional URDF/simulation only |
| URDF-003 | Base footprint to base link | 0.0478 m | Provisional URDF/simulation only |
| URDF-004 | Passive caster cylinder | radius 0.0393 m, width 0.0421 m | Provisional URDF/simulation only |
| URDF-005 | Base mass/inertia | 22.15 kg, positive box inertia | Provisional URDF/simulation only |
| URDF-006 | Navigation footprint | 1.20 x 0.80 m | URDF contract; not a hardware claim |
| URDF-007 | CAD geometry status | Legacy export untouched; derived CAD visuals with primitive collisions | Simulation contract; not a mechanical claim |
| CTRL-001 | Navigation controller | Direct Humble Regulated Pure Pursuit; desired linear speed 0.50 m/s | Provisional simulation tuning; runtime verification pending |
| CTRL-002 | Normal Nav2 goal checker | 0.07 m XY, 0.15 rad yaw | Preserved public contract |
| CTRL-003 | Placement goal checker | 0.005 m XY, 0.15 rad yaw, non-stateful | Private placement boundary; runtime verification pending |
| SIM-001 | Factory physics | Exact `0.0033333333333333335 s` step (`1/300 s`, 300 steps/s; 3 steps per 100 Hz controller/contact cycle and 30 per 10 Hz lidar/camera cycle), real-time factor target 1.0, shadows disabled; D205 observed median/aggregate RTF `0.999818`/`0.994510` | Current 1 kg runtime evidence; repeatability, 3 kg/5 kg, and Gate 7 remain pending |
| FACT-001 | Factory orchestration | Manual default; autonomous FIFO capacity 3; status at 5 Hz | Source boundary; Gate 7 runtime acceptance pending |
