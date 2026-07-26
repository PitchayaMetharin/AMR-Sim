# Phase 3 Communication Architecture

## Scope and governing rules

Phase 3 defines the simulation communication contract. It does not configure
either laptop, create ROS/PLC/gateway code, establish deterministic control,
or claim a safety-rated protocol. Future physical fieldbus, safety protocol,
and production network remain out of scope.

| Plane | Contract |
|---|---|
| Ubuntu ROS plane | ROS 2 Humble, Gazebo, RViz, estimation, perception, navigation, MPC, supervision, motion gate, and one OPC UA gateway; Fast DDS only |
| Windows authority plane | TIA Portal V17, PLCSIM Advanced simulated S7-1500F OPC UA server, and WinCC HMI simulation |
| Inter-laptop plane | OPC UA only; Windows receives no DDS traffic |
| Planned link | Closed \`192.168.50.0/24\`: Ubuntu \`.10\`, Windows \`.20\`; no gateway, DNS, DHCP, forwarding, or sharing |
| ROS environment | \`ROS_DOMAIN_ID=1\`, \`ROS_LOCALHOST_ONLY=1\` |

1. Each authoritative field/topic has one writer. Consumers do not rewrite
   observed state.
2. Missing, stale, malformed, contradictory, unacknowledged, or invalid
   critical data is non-permissive; a healthy transport is not machine
   permission.
3. Requests are level plus sequence/commit, never a pulse alone. Reconnect
   restores observation/transport only—not permission, reset, command, or goal.
4. Application heartbeat/freshness rules are required; session timeout, DDS
   discovery, TCP keepalive, and certificates are not substitutes.

## Endpoint and security contract

| Property | Requirement |
|---|---|
| Server/client | PLCSIM Advanced virtual S7-1500F / exactly one Ubuntu gateway |
| Endpoint | \`opc.tcp://192.168.50.20:4840\`, unless verified server path differs |
| Gateway URI | \`urn:amr:ros2:opcua-gateway\` |
| Namespace | Resolve \`http://www.siemens.com/simatic-s7-opcua\` by URI every session; never hard-code its index |
| Root | Symbolically browse \`DB_AMR_OPCUA\` |
| Drive-enabled security | Verified \`SignAndEncrypt\`, \`Basic256Sha256\`, and mutual certificate allowlist |
| Unsecured endpoint | Documented diagnosis only on closed link, with motion inhibited |

Private keys stay outside Git. Reject untrusted, expired, invalid,
host/address-mismatched, downgraded, wrong-server, missing-node, wrong-type,
wrong-access, or incompatible-schema sessions. Network/firewall/certificate
application is deferred to Phase 13; Windows TCP 4840 must ultimately accept
only Ubuntu \`.10\` on the simulation interface.

## Address-space and data ownership

Required symbolic tree:

\`\`\`text
DB_AMR_OPCUA
├── Interface (InterfaceVersion UInt32 = 0x00010000; RequiredClientMajor UInt16 = 1)
├── RosToPlc
├── PlcToRos
└── Diagnostics
\`\`\`

The gateway verifies existence, scalar rank, type, owner/write access, and
major interface compatibility before subscribing or publishing a ready state.
A major mismatch blocks; a minor change is acceptable only with compatible
required semantics.

| Direction | Sole writer | Required content |
|---|---|---|
| \`RosToPlc\` | Gateway | \`ClientBootId\`, \`HeartbeatSeq\`, \`RosReady\`, \`SimClockRunning\`, \`MotionEnableRequest\`, \`ControlledStopRequest\`, \`ResetRequest\`, \`RequestSeq\`, \`InputValidMaskLow/High\`, \`CommitSeq\`; simulated plant/safety inputs from Phase 2 |
| \`PlcToRos\` | PLC | \`StateSeq\`, machine/permission/fault/power/safety state, acknowledgements/result, watchdog and diagnostic state |
| \`Diagnostics\` | Declared owner only | Interface, timing, quality and evidence fields; no authority bypass |

Gateway writes a request bundle in dependency order and \`CommitSeq\` last. PLC
accepts only a new commit with valid inputs, matching boot handshake, and
Phase 12 eligibility. \`HeartbeatSeq\` proves only update-loop liveness. Each
new request has a new sequence; acknowledgement must correlate to it. Wrap is
valid. HMI reads authoritative state and issues only approved requests through
the PLC—never writes \`RosToPlc\` or bypasses PLC logic.

## PLC-state snapshot and timing

Gateway reads \`StateSeq\`, reads every required field, then reads \`StateSeq\`
again. It publishes a coherent state only when both sequence values match and
all values/types/quality are valid; otherwise it retries within a bounded
window, then publishes non-permissive invalid/stale state. It never combines
individual variables from unrelated scans.

| Contract | Initial value | Clock authority |
|---|---:|---|
| Gateway heartbeat | 100 ms | Ubuntu steady clock |
| PLC watchdog | 500 ms | PLC elapsed time |
| PLC state publication | 100 ms | PLC elapsed time |
| ROS PLC-state freshness | 300 ms | Ubuntu steady clock |
| Motion-command expiry | 200 ms | Ubuntu steady clock |

Gazebo \`/clock\` timestamps simulated robot data. UTC wall time is evidence
correlation only. Values need measurement and revision in Phase 13/14.

On connect: establish secure endpoint; resolve namespace; validate server
identity/schema; create subscriptions; obtain a coherent state; publish
non-permissive gateway state; then wait for fresh PLC eligibility and a new
acknowledged request before motion. On disconnect, schema failure, state
incoherence, stale state, or timeout: revoke gateway readiness and inhibit
motion. No cache can be used as permission.

## ROS contract

Canonical interfaces use namespaced, typed topics/actions for:

| Class | Contract |
|---|---|
| Robot state | wheel odometry, IMU, EKF state, TF, separate front/rear LiDAR and derived perception products |
| Commands | manual/mission intent into arbitration; only gated base command reaches simulator |
| Safety/machine | coherent PLC state, permission, fault/reason, request/ack and gateway health |
| Navigation | goal/action, global path, controller status; Nav2 never directly commands base |
| Diagnostics | lifecycle, timing, command source/expiry, schema/quality, watchdog and sensor provenance |

Use reliable QoS with appropriate bounded queues for authority/configuration,
reliable transient-local only for stable latched state where consumers need the
last valid value, and sensor-data QoS for high-rate LiDAR/IMU. Do not use
transient-local command velocity or permission as an automatic restoration
mechanism. Exact message definitions and package names are owned by Phase 4/5.

## Fault behavior, observability, and deferred work

| Event | Mandatory response |
|---|---|
| OPC UA/security/schema/quality/snapshot failure | Non-permissive gateway state and motion inhibit |
| PLC watchdog/safety/power/fault state | PLC permission false; gate prevents base command |
| ROS command expired or source conflict | Arbitration/constraint failure and zero/non-permissive output |
| Gateway, PLC, ROS, or simulator restart | New boot/sequence; no restored command, goal, reset, or permission |

Record endpoint/security selection (without secrets), schema version and browse
paths, state/request/ack sequences, gateway and PLC timestamps, freshness,
quality, permission reason, command source/expiry, lifecycle, and fault
transitions. Packet capture is diagnosis-only and cannot include credentials or
private keys.

Phase 4 owns interfaces, Phase 5 gateway implementation, Phase 12 PLC/HMI
state machine, and Phase 13/14 actual network/security configuration,
compatibility evidence, fault injection, timing measurement, and end-to-end
validation. Open gates are TIA/PLCSIM/CPU versions and policies, certificate
lifecycle/user identity, final namespace/schema, firewall/adapter/subnet
collision, timing measurements, and all physical deployment requirements.
