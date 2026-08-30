# Phase 4 ROS 2 Workspace

Shared interfaces define BaseStatus and HealthStatus. Ownership configuration
assigns the command topic to `amr_control/command_arbitration_node` and plant
transport to `amr_base_adapter/base_adapter_node`. Defaults fail closed.
