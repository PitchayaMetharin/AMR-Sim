# Phase 13 Health

`amr_health/health_supervisor_node` observes base-adapter status and publishes
`/amr/health/status`. Missing, stale, invalid, malformed, duplicate, regressed,
or backward-time evidence is non-healthy. The node has no motion, lifecycle,
or recovery authority.
