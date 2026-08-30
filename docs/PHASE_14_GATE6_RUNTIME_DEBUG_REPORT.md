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

## Phase J performance PASS and Phase K MoveIt boundary — 2026-08-30

The retained Phase J run is
`.ros_logs/gate6_1kg_retained_20260830_01/` with
`AMR_RUN_ID=gate6_1kg_retained_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_retained_20260830_01`, and `ROS_DOMAIN_ID=206`.
Its preserved runtime report records `/dev/dri/renderD128`,
`forced_software=<none>`, 3,600 samples, aggregate RTF `0.999999929313705`,
median RTF `1.000014400208803`, and `verdict=PASS`.

The current branch has no source diff; the only pre-documentation worktree
entry was the preserved `.ros_logs/` directory. The authoritative composite
Xacro expanded and passed `check_urdf`; the MoveIt contract test passed 4 tests;
and the project-owned MoveIt smoke loaded the composite model, OMPL, the
`FollowJointTrajectory` arm controller, the `GripperCommand` controller, and
the required MoveGroup capabilities. Its log is
`.ros_logs/gate6_1kg_retained_20260830_01/move_group_18_1788097414517.log`.

The first integrated Phase K readiness probe,
`ros2 lifecycle get /amr/command_arbitration_node`, returned `Node not found`
because the restricted execution environment did not contain the Phase J
factory/Nav2 graph. The MoveIt smoke was stopped cleanly; no recorder or
Product 101 stage was started and no runtime processes remain. Phase K is
therefore source-validated and partially runtime-validated, but not complete.
The direct-host continuation must keep the Phase J factory session, start
MoveIt with the same run environment, complete the lifecycle/controller/action
checks, then record and run exactly one Product 101 stage. Stop at the first
failed check.

## Phase K integrated direct-host PASS — 2026-08-30

Phase K was completed on the direct Ubuntu host with the preserved
`AMR_RUN_ID=gate6_1kg_retained_20260830_01`,
`GZ_PARTITION=amr_gate6_1kg_retained_20260830_01`, `ROS_DOMAIN_ID=206`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `ROS_LOCALHOST_ONLY=1`, and workspace-local
`ROS_LOG_DIR`. Phase J was not rerun or modified. The strict headless factory
launch used the project-owned graph with `factory_attachment:=true`; MoveIt
was started with `ros2 launch amr_manipulation move_group.launch.py`.

The initial host preflight found two stale same-partition Gazebo processes and
the graph showed duplicate controller-manager/Gazebo-control nodes. The exact
identified processes were stopped, the host preflight then passed with
`/dev/dri/renderD128` and no forced software renderer, and a clean factory
graph was launched. A first lifecycle query briefly returned `Node not found`
while DDS service discovery was settling; the command-arbitration service
became responsive on the bounded retry, and the full readiness pass returned:

- all 17 required lifecycle nodes `active [3]`;
- exactly one active `joint_state_broadcaster`, `arm_controller`, and
  `gripper_controller`;
- no unexpected duplicate visible node names;
- one server for each required mission, planning, smoothing, follow-path,
  dock-egress, arm, and gripper action;
- required attachment-bootstrap, controller-manager, and MoveIt services;
- required Product 101 topics, with `/amr/control/cmd_vel` owned by the sole
  `/amr/command_arbitration_node` publisher; and
- operational MoveGroup actions plus an OMPL response from
  `/query_planner_interface`.

The prescribed hidden-topic recorder reported `Recording...` before the stage.
The single Product 101 run passed bootstrap detachment, gripper/contact proof,
pickup, attachment safety rejection, dock egress, pickup approach, split
dispatch translation and heading, dispatch dock, four-segment placement
alignment, collision-checked pre-place/lower, release, and empty stow. The
stage log ended with the exact acceptance line:

`GATE 6 1.0 KG COMPLETE 1 KG PASS`

The finalized bag is
`.ros_logs/gate6_1kg_retained_20260830_01/product101_evidence/`, with
200,534 messages over 96.603 seconds. The stage and MoveIt logs are
`gate6_mass_stage_53626_1788099193302.log` and
`move_group_34342_1788098551620.log` in the same run directory. The final
exact-process shutdown scan passed with no Gazebo, MoveIt, rosbag, or Gate 6
processes remaining.

The initial stale-process and early lifecycle-discovery boundaries are
classified as direct-host setup/timing issues, not software defects; no source,
configuration, geometry, tolerance, controller, or performance file was
changed. The known non-blocking `imu_link` collision warning, missing MoveIt
3D octomap sensor warning, and MoveIt Humble shutdown destructor segfault
remain documented risks; the segfault occurred after the successful stage.
Products 102/103 and Gate 7 remain unvalidated.
