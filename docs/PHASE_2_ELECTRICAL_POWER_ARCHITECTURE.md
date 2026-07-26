# Phase 2 Electrical and Power Architecture

## Scope and evidence boundary

This is a conceptual future-physical power architecture and a simulation signal
contract. The current AMR runs entirely on two laptops; it does not authorize
procurement, conductor/fuse/terminal sizing, wiring, construction, charging,
or a safety/certification claim. Exact hardware evidence must precede any such
work.

| Status | Basis |
|---|---|
| Confirmed concept | 48 V, 30 Ah LiFePO4 (1.44 kWh nominal); simulated 50 kg payload; two MRS1104C-111011 LiDARs |
| Verified exact electrical datum | Each MRS1104C-111011 / 1081208: 10–30 VDC, 13 W typical, 37 W maximum |
| Candidate only | Battery/BMS, charger, 24 V and 12 V converters, contactors, precharge, protection, ZLAC8030D, motors, PLC/F-I/O, HMI, switch and Jetson |
| Blocking condition | Battery operating range, BMS limits, driver input/ripple/regeneration limits and all component ratings are not reconciled |

## Power architecture

\`\`\`text
Battery/BMS → service disconnect → source protection
  ├─ protected 24 V converter → PLC/control, sensors, network, HMI, coils
  ├─ protected 12 V converter → compute/peripherals
  ├─ protected traction branch → precharge + K1/K2 feedback → drive → motors
  └─ charging contacts/interlock ↔ matched charger ↔ battery/BMS
\`\`\`

- 24 V and 12 V branches are upstream of traction isolation so diagnostics and
  PLC state can persist after traction is removed. Every converter and branch
  needs coordinated future protection.
- K1/K2 are independently commanded and monitored series traction-isolation
  devices. Drive enable is secondary and is not credited as certified STO.
- Charging connection/status inhibits motion. Charger and BMS define the
  electrochemical envelope. The exact disconnect, isolation, feedback,
  precharge, regeneration and charging topology is TBD.
- No component rating, cable/connector, enclosure, thermal, bonding, EMC,
  creepage, clearance, fuse value, or construction detail is implied.

## Provisional auxiliary budgets

| Rail | Load basis | Calculated load | Candidate capacity | With 25% allowance | Residual after allowance |
|---|---|---:|---:|---:|---:|
| 24 V | BOM maximum/nameplate lines, including 2×37 W LiDAR | 174 W / 7.25 A | 240 W | 217.5 W / 9.06 A | 22.5 W / 0.94 A |
| 12 V | Future Jetson allowance | 40 W / 3.33 A | 60 W | 50 W / 4.17 A | 10 W / 0.83 A |

The 24 V budget includes provisional PLC/F-I/O, switch, HMI, coils, warning
devices, Wi-Fi, dock sensor and fan inputs; only the LiDAR figures pass the
exact-model evidence gate. The 12 V line does not establish simultaneous
Jetson, storage, USB, IMU, CAN, or thermal demand. At 91% typical converter
efficiency, the two loads imply about 235 W source input before distribution
loss—an estimate only. Validate worst-case states, derating, inrush, wiring
loss, variants, and measured consumption before closure.

## Traction, energy, and compatibility gates

The BOM's 1,625 W traction sum (two provisional 800 W motors plus 25 W driver)
cannot size the battery, BMS, fuse, contactors, conductors, precharge circuit,
runtime, or thermal solution. It omits duty cycle, acceleration, slope,
rolling resistance, payload, efficiencies, current limits, regeneration,
voltage sag, reserve, aging, and temperature.

Future mission sizing uses:

\`required usable energy = mission duration × average source power\`

where average source power includes traction duty, converters, accessories,
distribution loss, and reserve. The 8-hour target is therefore unverified.
Before any hardware-dependent implementation, verify the full battery voltage
range and BMS continuous/peak charge/discharge limits; driver input range,
current limits, regeneration handling and thermal derating; motor/load duty;
charger compatibility; and coordinated protection/isolation ratings.

## Conceptual fault and state rules

| Condition | Required simulated/architectural response |
|---|---|
| E-stop, safety inconsistency, BMS critical fault, contactor feedback fault, traction fault | Remove drive permission; request/hold traction isolation; latch until approved reset policy |
| Low SOC, voltage invalid, BMS warning, converter rail fault | Inhibit or derate according to later PLC policy; publish reason/quality |
| Charging connected, precharge incomplete, stale plant state, invalid data | Inhibit motion |
| ROS heartbeat loss | PLC watchdog removes permission or reaches defined stopped state |
| Restart/reconnect/fault clear | Never restores motion or a goal automatically |

The simulation exposes coherent logical signals, not physical I/O proof:
\`BatteryPresent\`, \`PackVoltageV\`, \`StateOfChargePct\`, \`BmsReady\`,
\`BmsWarning\`, \`BmsCriticalFault\`, \`Control24VValid\`,
\`Compute12VValid\`, \`TractionBusValid\`, \`ChargeConnected\`,
\`EstopChannelA/B\`, \`SafetyBumperChannelA/B\`, \`K1/K2Command\`,
\`K1/K2Feedback\`, \`PrechargeCommand\`, \`PrechargeFeedback\`, and
\`DriveFault\`. Invalid, non-finite, inconsistent, missing, or stale critical
values are non-permissive.

## Implementation and verification ownership

Phase 5 provides typed simulated plant inputs and gateway validation; it does
not model physical circuits. Phase 6 may model battery state/energy only as
explicitly labeled simulation assumptions. Phase 12 is user-authored PLC/HMI
logic. Phase 13/14 own approved network/security integration and validation.

Before a physical project, complete supplier-row repair and exact part suffixes,
official datasheets and compatibility review, hazard/risk assessment, fault
injection, converter/thermal/energy measurements, protection coordination,
precharge/regeneration testing, charging interlocks, EMC/bonding design, and
independent safety validation. Phase 2 is complete as a bounded conceptual
architecture only.
