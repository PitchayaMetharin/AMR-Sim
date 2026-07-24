# Robot Parameter Register

## Use

This register is the source-of-truth checklist for values that will later drive
architecture, electrical design, kinematics, URDF/Xacro, simulation,
estimation, perception, navigation, MPC, PLC integration, and validation.

Values are not implementation-authorized while marked **Conflict**, **TBD**, or
**Verification required**. SI units shall be used in software and Xacro even
when source drawings use millimetres.

## Geometry and Frames

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| GEO-001 | Robot type | Differential drive | Confirmed | All phases |
| GEO-002 | Forward axis | +X | Confirmed | TF/URDF |
| GEO-003 | Left axis | +Y | Confirmed | TF/URDF |
| GEO-004 | Up axis | +Z | Confirmed | TF/URDF |
| GEO-005 | Chassis/body nominal length | 1.000 m | Provisional | CAD interface/URDF/collision |
| GEO-006 | Chassis/body nominal width | 0.800 m | Provisional | CAD interface/URDF/collision |
| GEO-007 | Body height | approximately 0.600 m | Provisional | URDF/collision |
| GEO-008 | Height reference surfaces | TBD | TBD | URDF/collision |
| GEO-009 | Rigid-chassis ground clearance | 0.080 m | Confirmed | URDF/simulation |
| GEO-010 | Lowest rigid component and footprint | TBD | TBD | URDF/collision |
| GEO-011 | Overall operating envelope, including sensors | TBD | TBD | Navigation/safety |
| GEO-012 | Chassis primitive shape and corner radii | TBD | TBD | URDF/collision |
| GEO-013 | `base_footprint` ground projection definition | TBD | TBD | TF/navigation |
| GEO-014 | `base_link` origin position | TBD | TBD | TF/URDF |
| GEO-015 | Nominal floor plane/contact datum | TBD | TBD | URDF/simulation |
| GEO-016 | Maximum slope/ramp angle | TBD | TBD | Mechanics/navigation |
| GEO-017 | Maximum threshold/gap/step | TBD | TBD | Mechanics/navigation |
| GEO-018 | Minimum turning-space/aisle constraints | TBD | TBD | Navigation/acceptance |

## Mass, Payload, and Inertia

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| MAS-001 | Unloaded robot mass | 30 kg nominal | Confirmed provisional simulation value | Sizing/URDF/MPC |
| MAS-002 | Unloaded mass tolerance | ±5 kg; acceptable estimate 25–35 kg | Confirmed provisional range | URDF/validation |
| MAS-003 | Chassis-only mass allocation | TBD | TBD | URDF |
| MAS-004 | Component mass allocation | TBD | TBD | URDF |
| MAS-005 | Unloaded center of mass x/y/z | TBD | TBD | Stability/URDF/MPC |
| MAS-006 | Unloaded inertia tensor | To be derived from approved mass distribution | TBD | URDF/simulation |
| MAS-007 | Default and initially rated simulated payload | 50 kg | Confirmed current baseline | URDF/Gazebo/MPC/acceptance |
| MAS-008 | Optional future payload stress case/design target | 300 kg | Not the current rating; test envelope and physical capability unverified | Future simulation/sizing |
| MAS-009 | Payload envelope | TBD | TBD | Collision/stability |
| MAS-010 | Payload center-of-gravity range | TBD | TBD | Stability/MPC |
| MAS-011 | Payload mounting/restraint interface | User mechanical scope; interface TBD | TBD | Integration/safety |
| MAS-012 | Initial total simulated moving mass | approximately 80 kg at nominal unloaded mass | Confirmed initial baseline | Simulation/MPC/acceptance |
| MAS-013 | Payload adjustment method | Manual Xacro/launch override before model spawn; no live in-session adjustment required | Confirmed requirement; Phase 6 implementation | Gazebo/operations |
| MAS-014 | Payload mass-property consistency | Mass, center of gravity, collision geometry, and inertia must change together | Confirmed modeling constraint; profiles TBD | URDF/Gazebo |

## Drive Wheels and Motors

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| DRV-001 | Drive-wheel quantity | 2 | Confirmed | URDF/control |
| DRV-002 | Selected motor family/model text | ZLTECH ZLLG10ASM800 V2.0 | Confirmed selection | BOM/driver/URDF |
| DRV-003 | Exact motor ordering code and revision | ZLLG10ASM800 V2.0 | Manufacturer page verified; delivered hardware revision still inspect | All hardware work |
| DRV-004 | Nominal unloaded wheel outer radius | 0.127 m from manufacturer-stated 10-inch diameter | Verified nominal; not measured rolling radius | URDF/kinematics |
| DRV-005 | Effective rolling radius under approved load | TBD | TBD | Odometry/control |
| DRV-006 | Wheel/tire width | TBD | TBD | URDF/contact |
| DRV-007 | Tire material/tread and friction basis | TBD | TBD | Simulation/traction |
| DRV-008 | Wheel center-to-center separation | TBD | TBD | Kinematics/odometry |
| DRV-009 | Left wheel center pose x/y/z | TBD | TBD | URDF/TF |
| DRV-010 | Right wheel center pose x/y/z | TBD | TBD | URDF/TF |
| DRV-011 | Axle/joint axis convention | TBD | TBD | URDF/control |
| DRV-012 | Motor/wheel assembly mass and inertia | TBD | Verification required | URDF |
| DRV-013 | Direct-drive/reduction ratio | TBD | Verification required | Driver/control |
| DRV-014 | Encoder type and resolution at wheel | TBD | Verification required | Odometry/driver |
| DRV-015 | Encoder polarity/index behavior | TBD | TBD | Commissioning |
| DRV-016 | Motor rated voltage/current/power | TBD | Verification required | Electrical design |
| DRV-017 | Motor continuous/peak torque | TBD | Verification required | Sizing/MPC |
| DRV-018 | Motor rated/maximum speed | TBD | Verification required | Sizing/control |
| DRV-019 | Motor thermal limits/sensing | TBD | Verification required | Protection |
| DRV-020 | Left/right command and feedback polarity | TBD | TBD | Commissioning |

## Casters

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| CST-001 | Caster quantity | 4 | Confirmed | URDF/mechanics |
| CST-002 | Caster type/model/ordering code | 4 × TENTE LEVINA 5370PJP100P62 | Confirmed selection; dimensions/load verification required | BOM/URDF |
| CST-003 | Swivel, fixed, ball, or simplified contact model | Passive caster; mechanism TBD | TBD | URDF/simulation |
| CST-004 | Wheel radius | TBD | TBD | URDF/contact |
| CST-005 | Wheel width | TBD | TBD | URDF/contact |
| CST-006 | Swivel trail/offset | TBD | TBD | URDF/dynamics |
| CST-007 | Swivel-axis and wheel-axis geometry | TBD | TBD | URDF |
| CST-008 | Front-left mounting pose x/y/z | TBD | TBD | URDF |
| CST-009 | Front-right mounting pose x/y/z | TBD | TBD | URDF |
| CST-010 | Rear-left mounting pose x/y/z | TBD | TBD | URDF |
| CST-011 | Rear-right mounting pose x/y/z | TBD | TBD | URDF |
| CST-012 | Assembly masses/inertias | TBD | TBD | URDF |
| CST-013 | Load rating and load distribution | TBD | Verification required | Mechanical validation |
| CST-014 | Contact friction/slip parameters | TBD | TBD | Simulation |
| CST-015 | Joint damping/friction limits | TBD | TBD | Simulation |

## LiDAR and IMU

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| SEN-001 | LiDAR quantity/family | 2 × simulated SICK MRS1000 characteristics | Confirmed simulation selection; no current purchase | BOM/perception |
| SEN-002 | Each LiDAR ordering code/hardware revision | MRS1104C-111011 / 1081208 | Official datasheet verified; delivered revision TBD | Driver/perception |
| SEN-003 | Each LiDAR firmware version | TBD | TBD | Driver/validation |
| SEN-004 | LiDAR communication interface/protocol | Ethernet/CoLa; final ROS 2 driver configuration TBD | Official family integration verified; bench test required | Network/driver |
| SEN-005 | Supported ROS 2 driver/package/version | TBD | TBD | Software architecture |
| SEN-006 | Front LiDAR parent frame | `base_link` expected; confirm | TBD | TF |
| SEN-007 | Front LiDAR x/y/z | Near front-left; numeric pose TBD | Provisional | TF/URDF |
| SEN-008 | Front LiDAR roll/pitch/yaw | TBD | TBD | TF/perception |
| SEN-009 | Rear LiDAR parent frame | `base_link` expected; confirm | TBD | TF |
| SEN-010 | Rear LiDAR x/y/z | Near rear-right; numeric pose TBD | Provisional | TF/URDF |
| SEN-011 | Rear LiDAR roll/pitch/yaw | TBD | TBD | TF/perception |
| SEN-012 | LiDAR scan-layer selection and usable FOV | Four layers over 275° horizontal and 7.5° vertical; used layers/config TBD | Official datasheet verified | Perception/costmaps |
| SEN-013 | LiDAR scan frequency/data rate | 50 Hz / four layers at 12.5 Hz; network rate TBD | Official datasheet verified | Compute/network |
| SEN-014 | LiDAR range/intensity/filter settings | TBD | Verification required | Perception |
| SEN-015 | LiDAR occlusion/exclusion zones | TBD | TBD | Mounting/perception |
| SEN-016 | LiDAR IP addresses/subnet/VLAN | TBD | TBD | Network |
| SEN-017 | LiDAR time-synchronization method | TBD | TBD | Perception/SLAM |
| SEN-018 | LiDAR calibration method and tolerances | TBD | TBD | Perception/acceptance |
| SEN-019 | IMU quantity/family | Simulated IMU; Xsens MTi-8 candidate reference | Confirmed simulation scope | BOM/estimation |
| SEN-020 | IMU ordering code/hardware revision | MTi-8-5A-DK candidate future hardware | Not used in current project | Driver/estimation |
| SEN-021 | IMU firmware version | TBD | TBD | Driver/validation |
| SEN-022 | IMU interface/protocol and connector | TBD | Verification required | Electrical/driver |
| SEN-023 | Supported ROS 2 driver/package/version | TBD | TBD | Software architecture |
| SEN-024 | IMU parent frame | `base_link` expected; confirm | TBD | TF |
| SEN-025 | IMU x/y/z | TBD | TBD | TF/URDF |
| SEN-026 | IMU roll/pitch/yaw and axis alignment | TBD | TBD | TF/EKF |
| SEN-027 | IMU output rate and enabled messages | TBD | Verification required | EKF/compute |
| SEN-028 | IMU time-synchronization method | TBD | TBD | EKF |
| SEN-029 | IMU covariance/noise parameters | TBD | Verification/characterization required | EKF |
| SEN-030 | Magnetic aiding policy/calibration | TBD | TBD | EKF/environment |

## Motion and Control Limits

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| MOT-001 | Maximum forward linear speed | Design 1.0 m/s; initial commissioning 0.5 m/s | Provisional software limits; hardware validation required | Sizing/Nav2/MPC |
| MOT-002 | Maximum reverse linear speed | 0.5 m/s initial; final reverse design limit TBD | Provisional | Sizing/Nav2/MPC |
| MOT-003 | Maximum angular speed | Design 0.8 rad/s; initial commissioning 0.4 rad/s | Provisional software limits; hardware validation required | Sizing/Nav2/MPC |
| MOT-004 | Maximum linear/angular acceleration | 0.4 m/s² linear; 0.6 rad/s² angular | Provisional software limits | Sizing/Nav2/MPC |
| MOT-005 | Normal linear/angular deceleration | 0.5 m/s² linear; 0.8 rad/s² angular | Provisional software limits | Control/acceptance |
| MOT-006 | Maximum commanded emergency deceleration | 1.0 m/s² linear | Provisional controlled-command target; not guaranteed E-stop deceleration | Safety/sizing |
| MOT-007 | Linear/angular jerk limits | 0.5 m/s³ linear; 1.0 rad/s³ angular | Provisional software limits | MPC/stability |
| MOT-008 | Minimum controllable wheel/body speed | TBD | TBD | Driver/MPC |
| MOT-009 | Command timeout/watchdog period | TBD | TBD | Driver/PLC |
| MOT-010 | Wheel-speed command/update rate | TBD | TBD | Driver/control |
| MOT-011 | Controller/odometry/EKF update rates | TBD | TBD | Software architecture |
| MOT-012 | Goal position/yaw tolerances | ±0.050 m and ±2° | Initial acceptance target | Nav2/acceptance |
| MOT-013 | Path cross-track/yaw error limits | TBD | TBD | MPC/acceptance |
| MOT-014 | Required obstacle/structure clearance | TBD | TBD | Costmaps/safety |
| MOT-015 | Wheel slip allowance/detection policy | TBD | TBD | EKF/control |

## Motor Driver and Low-Level Interface

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| CTL-001 | Driver model text | ZLAC8030D | Confirmed selection; verification required | BOM/control |
| CTL-002 | Exact driver ordering code/revision/firmware | ZLAC8030D in BOM; revision/firmware TBD | Verification required | Driver implementation |
| CTL-003 | Driver supply range and current limits | TBD | Verification required | Electrical design |
| CTL-004 | Command interface and physical layer | TBD | Verification required | Communications |
| CTL-005 | Selected operating mode | Internal speed PID required; exact mode TBD | TBD | Driver implementation |
| CTL-006 | Command/feedback units and scaling | TBD | Verification required | Driver implementation |
| CTL-007 | Register/object map revision | TBD | Verification required | Driver implementation |
| CTL-008 | Acceleration/deceleration handling split | TBD | TBD | Driver/MPC |
| CTL-009 | Internal PID gains and tuning procedure | TBD | TBD | Commissioning |
| CTL-010 | Enable, brake, alarm-reset behavior | TBD | Verification required | PLC/driver |
| CTL-011 | Fault/status set and ROS diagnostics mapping | TBD | Verification required | Diagnostics |
| CTL-012 | Communication-loss reaction | TBD | TBD | Safety/PLC |
| CTL-013 | Motor temperature/overcurrent protection settings | TBD | TBD | Commissioning |

## Compute, Software, and Simulation

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| SW-001 | ROS generation | ROS 2 | Confirmed | All software |
| SW-002 | ROS 2 distribution | Humble | Confirmed; observed installed | All software |
| SW-003 | Ubuntu version | 22.04 LTS | Confirmed; 22.04.5 observed | All software |
| SW-004 | JetPack/L4T version | Not applicable to current laptop simulation | Deferred to future physical project | Deployment |
| SW-005 | Jetson Orin Nano module/carrier/storage | Developer Kit 8GB candidate future hardware | Not used in current project | BOM/deployment |
| SW-006 | DDS/RMW implementation and QoS policy | Fast DDS observed; production RMW/QoS policy TBD | Partially confirmed | Communications |
| SW-007 | Simulation platform/version | Gazebo Harmonic 8.14.0 | Confirmed; headless, GUI, transport, and ROS clock bridge tested | Phase 6 |
| SW-008 | `ros2_control` use and controller set | TBD | TBD | Simulation/hardware |
| SW-009 | Simulation physics engine and step/update rate | TBD | TBD | Phase 6 |
| SW-010 | SLAM Toolbox version/configuration mode | Selected package; details TBD | TBD | SLAM |
| SW-011 | `robot_localization` version/state vector | Selected package; details TBD | TBD | EKF |
| SW-012 | Nav2 version/global planner | Nav2 selected; details TBD | TBD | Navigation |
| SW-013 | MPC implementation/plugin/solver | TBD; MoveIt not required for initial mobile base | TBD | Phase 11 |
| SW-014 | Real-time/timing requirements | TBD | TBD | Control architecture |
| SW-015 | Logging, bagging, retention, and clock source | TBD | TBD | Validation/support |
| SW-016 | Software update/rollback mechanism | TBD | TBD | Deployment |
| SW-017 | Primary production implementation language | C++17 minimum | Confirmed for ROS 2 Humble; GCC 11.4 observed | All software |
| SW-018 | Current execution target | Laptop-based simulation only | Confirmed; no physical hardware | All phases |
| SW-019 | ROS 2 Humble/Gazebo Harmonic integration | `ros-humble-ros-gzharmonic` 0.244.12-3jammy | Installed; `/clock` bridge and sensor message type mappings pass; do not mix with conflicting `ros-humble-ros-gz` packages | Phase 6 |
| SW-020 | Joint-state publication tools | `joint_state_publisher` and GUI 2.4.0 | Installed; modules and executable interfaces load | Phase 6 |
| SW-021 | Rendering baseline | Intel integrated graphics; RViz OpenGL/GLSL 4.6 and Gazebo OGRE2 startup pass | Confirmed for empty-world baseline; full-load performance TBD | Simulation/visualization |

## PLC, Safety, and Network

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| SYS-001 | PLC family | Siemens S7-1200F | Confirmed selection | BOM/PLC |
| SYS-002 | PLC safety CPU/I/O ordering codes and firmware | CPU 6ES7214-1AF40-0XB0; F-DI 6ES7226-6BA32-0XB0; F-DQ 6ES7226-6DA32-0XB0 in BOM; firmware TBD | Verification required | Safety/electrical |
| SYS-003 | PLC safety and standard responsibilities | E-stop/bumper, contactors/drive power, permissives, safety I/O feedback, reset/restart, power sequencing, safety fault latching, ROS watchdog | Confirmed boundary; detailed cause/effect TBD | Architecture |
| SYS-004 | ROS-to-PLC protocol and data contract | TBD | TBD | Communications |
| SYS-005 | Control authority/state machine | ROS requests motion; PLC owns drive permission and power enable | Confirmed boundary; detailed state machine TBD | Architecture |
| SYS-006 | Heartbeat/watchdog timing | TBD | TBD | PLC/ROS |
| SYS-007 | E-stop zones, devices, reset, and restart behavior | TBD | TBD | Safety |
| SYS-008 | Contactor/brake/drive-enable cause-and-effect | TBD | TBD | Safety/electrical |
| SYS-009 | Required PL/SIL and safety standards | ISO 3691-4:2023 primary; ISO 12100:2010, ISO 13849-1:2023, IEC 60204-1:2016+A1:2021 supporting; provisional PL d Category 3 concept | No compliance claim; physical PLr requires future risk assessment | Safety |
| SYS-010 | Safety validation authority | Academic project team and university supervisor | Simulation review only; formal assessor required for physical deployment | Safety/acceptance |
| SYS-011 | Managed switch model/port plan | SCALANCE XC216 / 6GK5216-0BA00-2AC2 in BOM; port plan TBD | Verification required | BOM/network |
| SYS-012 | Network topology, IP plan, VLANs, QoS | TBD | TBD | Communications |
| SYS-013 | System time source/synchronization topology | TBD | TBD | EKF/SLAM/logging |
| SYS-014 | Cybersecurity/access-control requirements | TBD | TBD | Deployment |
| SYS-015 | External host/fleet/WMS/MES interface | Not required for prototype; keep future-ready for REST/MQTT/OPC UA/VDA 5050 only if later required | Confirmed initial scope | Architecture |

## Power and Operational Environment

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| PWR-001 | Traction battery nominal voltage/chemistry | 48 V LiFePO4 system | Confirmed concept; verification required | BOM/electrical |
| PWR-002 | Battery/BMS/charger ordering codes | User-supplied 48 V LiFePO4; capacity, mass, manufacturer, BMS, and charger TBD | TBD | Electrical |
| PWR-003 | Capacity, continuous/peak current, SOC limits | TBD | Verification required | Electrical/endurance |
| PWR-004 | Charging method/interface/duty | TBD | TBD | Operations/electrical |
| PWR-005 | Required mission duration/duty cycle | At least 8 h representative mixed operation per charge | Initial target; capacity/power validation required | Sizing/acceptance |
| PWR-006 | Low-energy behavior and shutdown reserve | TBD | TBD | PLC/operations |
| ENV-001 | Indoor/covered-outdoor/outdoor classification | Indoor industrial/laboratory prototype; no outdoor or uncontrolled public operation | Confirmed initial scope | All design |
| ENV-002 | Temperature and humidity range | TBD | TBD | BOM/acceptance |
| ENV-003 | Dust/water/ingress requirement | TBD | TBD | BOM/acceptance |
| ENV-004 | Floor material, friction, flatness, and contamination | Mostly flat indoor floor; friction/contamination envelope TBD | Partial | Simulation/control |
| ENV-005 | Ambient-light/weather exposure | TBD | TBD | Perception |
| ENV-006 | Electromagnetic environment/EMC requirements | TBD | TBD | Electrical/validation |
| ENV-007 | Regulatory country/jurisdiction | Thailand; international standards used for conceptual architecture | Confirmed future jurisdiction | Safety/compliance |

## Application and Validation Targets

| ID | Parameter | Value | Status | Required by |
|---|---|---:|---|---|
| ACC-001 | Mission/use-case definition | Predefined stations, goal stop, return home/charge area, blocked/canceled/failed recovery, state/fault reporting | Confirmed prototype scope | Architecture |
| ACC-002 | Route/scenario test matrix | TBD | TBD | Validation |
| ACC-003 | Localization accuracy and relocalization time | ±0.050 m normal goal; provisional stop/recovery at 0.100 m or 5° uncertainty; relocalization time TBD | Initial targets; estimator implementation method TBD | SLAM/acceptance |
| ACC-004 | Mapping accuracy/coverage criteria | TBD | TBD | SLAM/acceptance |
| ACC-005 | Navigation success/throughput targets | ≥95% mission success over representative test set without manual intervention | Initial target; test set TBD | Navigation/acceptance |
| ACC-006 | Docking/positioning accuracy, if applicable | Fixed-station repeatability ±0.030 m; final docking accuracy/method TBD | Initial target | Navigation |
| ACC-007 | Availability/reliability target | Prototype ≥95%; future production ≥98% excluding charging/scheduled maintenance | Initial/future targets | Architecture/acceptance |
| ACC-008 | Endurance-test duration | 8 h representative mixed operation per charge | Initial target | Validation |
| ACC-009 | Fault-recovery time and operator intervention limits | Restart/recovery without complete reconfiguration; safety faults require defined reset | Partial; numeric time TBD | Diagnostics/acceptance |
| ACC-010 | Data retention/traceability requirement | TBD | TBD | Validation/operations |
| ACC-011 | Noise requirement | TBD | TBD | Compliance |
| ACC-012 | Maintenance/service intervals and diagnostics access | TBD | TBD | Operations |

## Hardware Evidence Gate

The supplied BOM contains a mix of exact, provisional, stale, and internally
misaligned entries; see
[`PHASE_0_BOM_REVIEW.md`](PHASE_0_BOM_REVIEW.md). Before a hardware
specification is used for implementation, the following evidence is mandatory:

1. exact manufacturer;
2. full model and ordering code;
3. hardware and firmware revision where applicable;
4. quantity;
5. approved substitutions, if any;
6. official manufacturer document title, revision/date, and source;
7. the specific value and document location used by the design.

This gate applies at minimum to the motors, motor driver, casters, both LiDARs,
IMU, Jetson module/carrier, PLC and I/O, network switch, battery, BMS, charger,
contactors, protection devices, emergency-stop devices, and power supplies.
