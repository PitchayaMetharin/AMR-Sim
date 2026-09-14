# Phase 10 Navigation

Nav2 provides global planning and costmaps from SLAM, local state, and
perception. The mission supervisor owns the public action boundary; the Nav2
Regulated Pure Pursuit (RPP) velocity output enters command arbitration before
it reaches the base adapter. The compatibility topic remains
`/amr/mpc/cmd_vel` and the package remains `amr_mpc_controller`.
