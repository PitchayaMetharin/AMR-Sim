# Phase 14 CAD/URDF remediation report

## Scope and decision

`amr_urdf_cad` remains the untouched ROS 1 source export. The active ROS 2
description uses the derived CAD meshes for visual appearance only and
conservative primitives for collision and dynamics. It is simulation-ready,
not a mechanically authoritative export.

## Export problems and bounded responses

| Export problem | Effect | Applied response |
| --- | --- | --- |
| `base_link.STL` contains a static `KR6 R900 sixx` arm and mounting shells | The arm is the wrong model and cannot articulate | Remove the high arm shells from the derived visual and mount the existing articulated `KR6 R900-2` directly to `base_link` |
| Base mass/inertia includes the arm and assembly | Exported mass is contaminated and cannot be used as base data | Use clearly provisional `22.15 kg` positive box inertia |
| Missing arm, camera, IMU, payload, and `base_footprint` contracts | Software cannot rely on stable frames | Keep the project-owned frames and sensor interfaces in the active Xacro |
| No explicit drive-wheel radius | Odometry and simulation can drift apart | Pin both to radius `0.1128 m` and separation `0.566 m` |
| Four casters are fixed monoliths | No passive swivel or wheel motion exists | Use continuous swivel and rolling joints at the exported mounts; radius/width `0.0393/0.0421 m` |
| ROS 1 package, Gazebo launch, and missing ROS 2 plugins/controllers | Export cannot be used directly by this ROS 2 stack | Keep it as source-only and use the project-owned ROS 2 package/interfaces |
| STL collision geometry is monolithic and unvalidated | Mesh contact would be brittle and expensive | Keep CAD meshes as visuals; use a chassis box plus wheel, caster, and sensor primitive collisions; retain no pedestal collision |

The base visual derivation is fail-closed in
`src/amr_description/scripts/derive_cad_meshes.py`: it checks SHA-256
`b0f27db25987905634d2bff27f52c68fd1fe90f1ce1d0977ec57de63eb44d015`, exactly
128,670 triangles and 80 connected components, retains the 54 chassis
components selected by the export topology, and rejects changed mounting-plate,
centered-pedestal, or high-arm shells. The retained visual has 96,178 triangles;
caster files are split into body and wheel shells. No source STL is edited.

## Active description

- Chassis and sensor visuals use the installed derived CAD meshes, with explicit
  CAD colors; collisions remain primitive.
- The composite has no `arm_pedestal_link` or duplicate base. The KUKA mount is
  `arm_mount_joint: xyz="0 0 0.33"`, flush with the AMR top surface.
- The navigation footprint remains `1.20 x 0.80 m`; the existing watchdog,
  DiffDrive, topics, limits, LiDAR/IMU/camera frames, gripper, and stow pose
  remain unchanged.
- Base-only payload arguments remain available; the mobile-manipulator
  composite omits the generic payload.

## Validation and limitations

Source checks cover mesh derivation, unique visual names, primitive collisions,
positive inertias, one articulated six-joint KUKA, exact mount/camera transforms,
preserved frames and topics, and matching odometry/Gazebo geometry. Focused
Humble builds/tests for description, control, factory, mission, MPC/RPP, and
manipulation pass in the current worktree.

The provisional values do not establish real mass properties, traction,
structural strength, or factory safety. A future clean export must separate the
authoritative `KR6 R900-2`, provide isolated mass/inertia, explicit wheel and
caster axes, modular visual/collision assets, ROS 2 metadata, and project-owned
plugins/controllers before replacing this workaround.

## Runtime lessons — D205 — 2026-08-24

The D205 run separates two description/URDF-caused issues from the simulator,
launch, and runtime issues that followed:

| Category | Lesson and evidence |
| --- | --- |
| URDF/description | The direct CAD mount was about 0.10 m lower than the earlier staging assumption, so the infeasible pregrasp `z=1.10` was corrected to `z=1.00`. Mirrored wheel origins also require local left `axis_z=+1` and right `axis_z=-1` so both transformed drive axes are base `+Y`; no steering, geometry, or tolerance workaround was used. |
| Not URDF | The vendored Gazebo Harmonic POSITION-interface reset behavior, rear lifecycle activation race, launch configure ordering/startup observer, RPP pickup heading chatter, localization dock bias, MoveIt placement reach margin, held-product radial-yaw/base collision, and post-detach contact/duplicate-object scene were simulator, launch, or runtime mechanisms. They were corrected at their owning source/launch boundaries without changing the CAD export. |

Final base/composite Xacro and `check_urdf` validation remained passing, and no
source STL was edited. These runtime lessons do not turn provisional simulation
geometry or one 1 kg run into a hardware or higher-mass safety claim.
