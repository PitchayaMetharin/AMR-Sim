# Project Status

## Active phase

Phase 14 factory mobile manipulation remains authorized gate-by-gate. Gates 1
through 5 passed. Phase J runtime-performance acceptance and Phase K
integrated MoveIt/Product 101 validation passed on the direct Ubuntu host.
Do not start 3 kg, 5 kg, or Gate 7 without the required evidence review and
authorization.

## Phase J evidence

The preserved run is
`.ros_logs/gate6_1kg_retained_20260830_01/`. Its runtime-performance report
records `/dev/dri/renderD128`, no forced software renderer, 3,600 samples,
aggregate RTF `0.9999999293`, median RTF `1.0000144002`, and `verdict=PASS`.
No Phase J source, geometry, controller, or performance setting was changed.

## Phase K status

- The existing project-owned MoveIt launch uses the authoritative composite
  Xacro and SRDF, the `manipulator` group, OMPL, the `arm_controller` and
  `gripper_controller` adapters, and `/amr/base/joint_states`.
- The exact composite Xacro expanded and `check_urdf` passed with
  `arm_joint_1` through `arm_joint_6`, both gripper joints, `arm_base_link`,
  and `gripper_tcp`.
- A bounded MoveIt smoke started the model, OMPL pipeline, both configured
  controllers, and all required MoveGroup capabilities.
- Integrated readiness passed on the direct host: all required lifecycle nodes
  reported `active [3]`, the joint-state/arm/gripper controllers were active
  exactly once, required actions/services/topics were present, MoveGroup
  actions and the OMPL query service responded, and visible node names had no
  unexpected duplicates.

## Phase K result

The single direct-host Product 101 run used the mandated Phase J environment,
the strict headless factory graph with `factory_attachment:=true`, the
project-owned MoveIt launch, and the prescribed hidden-topic recorder. The
stage passed attachment bootstrap, bilateral gripper evidence, pickup,
attachment safety rejection, dock egress, pickup approach, split dispatch
navigation, dispatch dock, placement alignment/lower, release, and empty
stow. It produced the exact terminal line:

`GATE 6 1.0 KG COMPLETE 1 KG PASS`

The recorder finalized a 111.3 MiB, 200,534-message bag at
`.ros_logs/gate6_1kg_retained_20260830_01/product101_evidence/`. No Phase J
performance evidence was rerun or changed, and no source or configuration
file was modified.

## Validation and stop reason

`src/amr_manipulation/test/test_moveit_config.py`: 4 passed. The original
bounded MoveIt smoke log remains preserved at
`.ros_logs/gate6_1kg_retained_20260830_01/move_group_18_1788097414517.log`;
the Phase K runtime MoveIt log is
`.ros_logs/gate6_1kg_retained_20260830_01/move_group_34342_1788098551620.log`.
The initial direct-host attempt found stale duplicate Gazebo processes; only
the identified PIDs were stopped before the clean graph launch. An early
lifecycle query briefly returned `Node not found` during service discovery,
then the same check and the full lifecycle set returned `active [3]` after the
graph settled. This was an environment/startup timing boundary, not a proven
source defect.

The recorder, MoveIt, and factory processes were stopped and the final exact
process scan passed. The worktree contains the preserved untracked `.ros_logs/`
evidence plus these tracking-document changes; no source code was modified.

The next action is to review the complete Phase K evidence before any
independent 3 kg, 5 kg, or Gate 7 work.

No dependency or external asset was installed. No commit or push was made.
`AMR_CODEX_HANDOFF.md` remains user-owned and untouched.
