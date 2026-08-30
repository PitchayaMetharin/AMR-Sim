# Phase 14 Gate 6 runtime debug report

This short record maps the evidence-backed D205 correction chain. It does not
claim higher-mass, repeatability, hardware, or Gate 7 acceptance.

| Problem | Smallest solution | Evidence |
| --- | --- | --- |
| Direct CAD mount made pregrasp `z=1.10` infeasible | Use the direct-mount FK/IK endpoint at `z=1.00` | D178 direct-mount KDL/FK; D205 pre-place IK/OMPL/lower passed |
| Mirrored wheel origins inverted drive signs | Parameterize wheel `axis_z`; instantiate left `+1`, right `-1` so both base axes are `+Y` | D183/D185 sign and transformed-axis traces; final URDF contracts passed |
| Vendored POSITION interface synthesized velocity and left cross-axis error | Apply Harmonic `JointPositionReset` in the POSITION branch; preserve VELOCITY/EFFORT paths | D175 joint5 final error trace; vendored build/test passed |
| Rear lifecycle adapter stayed inactive after a configure/activate race | Register activation before configure and serialize lifecycle startup | D184 raw rear scan healthy but adapter inactive; D205 adapted rear sample passed |
| Configure burst and readiness observer caused startup false negatives | Configure adapters in an active-state chain and use one bounded rclpy observer for lifecycle, controllers, and fresh TF | D198-D200 discovery/participant failures; D205 strict readiness passed in 16 s |
| RPP pickup final-heading chatter | Split pickup travel-bearing navigation from same-position final-heading navigation | D201 repeated westbound heading chatter; D205 pickup split passed |
| Localization bias moved the dock target | Measure fresh GT/localized planar bias and correct one normal dock target | D190-D192 bias traces; D205 corrected dock and downstream gates passed |
| Placement stance radius left too little reach margin | Derive the nominal stance radius as `0.785 - 0.07 - 0.005 = 0.710 m` and scale the existing direction | D192 release `0.796532 m`; D205 placement error `0.000894 m` |
| Radial held-product yaw collided with the base | Keep radial bearing for reach/IK but map-align product yaw from fresh robot yaw | D203 OMPL base/product collision; D205 map-aligned collision-free IK/OMPL/lower passed |
| Post-detach retreat began contact-invalid and duplicated the world object | Keep the returned `held_product`, remove duplicate `placed_product`, temporarily allow only held-product/finger pairs during the straight retreat, restore ACM, then require state validity | D204 0%/1-point retreat; D205 37 points/100%, ACM restored, validity and empty stow passed |
| Analyzer treated retired precise-action silence as a bag failure | Treat `/amr/mission/navigate_to_pose_precise/_action/status` zero messages as expected and analyze normal navigation status | D205 bag: 239.8 MiB/412,523 messages; stale analyzer alone reported `BAG_ANALYSIS=FAIL` |

## D205 boundary

Verified artifacts are under
`.ros_logs/gate6_d205_product101_loop28_20260824_01` with
`ROS_DOMAIN_ID=205` and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Factory readiness,
MoveIt readiness, ownership, recorder, performance (median RTF `0.999818`,
aggregate `0.994510`), and the exact terminal `GATE 6 1.0 KG COMPLETE 1 KG
PASS` passed. Only this one 1 kg run is accepted; Gate 7 and higher-mass or
repeatability claims remain pending.
