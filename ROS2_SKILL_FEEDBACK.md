# ROS 2 Skill Feedback

Collect evidence-backed flaws and improvement ideas for the installed `ros2`
skill so the user can improve it later. This is a feedback backlog, not authority
to edit `/home/pete/.codex/skills/ros2` or change robot behavior.

## Recording rules

* Distinguish a skill defect from a repository defect, test-fixture mistake or
  missing task-specific fact. Do not blame the skill without a source reference.
* Record the exact skill file/section, expected guidance, observed consequence,
  reproduction/evidence, proposed minimal improvement and validation needed.
* Label findings `candidate`, `confirmed`, `rejected` or `resolved`; preserve
  uncertainty. Reuse a finding rather than duplicating it across agents.
* Agents report findings to the coordinator, who appends them here serially.
* Skill changes require a later user-approved task; no automatic skill repair.

## Findings

### Current-task usage decision — 2026-10-04

User permits excluding confusing/flawed guidance and delegates that choice.
Coordinator sets aside the installed ros2 skill workflow/templates for current
functional A/B/home recovery, using workspace rules and installed Humble APIs
directly. This is a task-local usage preference, not an installed-skill edit,
global disablement or finding that the whole skill is flawed.

CT1 GUI35 scope error is not attributed to the skill: SKILL.md router explicitly
says to validate facts against the actual workspace. The observer/fixture and
ROOT verification assumed LifecycleNode, while actual installed controller launch
uses Node; M034/M035 record that implementation/verification mistake. No read
skill source instructs treating every ROS lifecycle process as a LifecycleNode
launch action. Confirmed source references would be needed to claim causality.

No confirmed skill flaws recorded yet.

### ROS2-001 — Collision-fixture specificity (candidate, 2026-10-02)

Skill references: `systems/navigation/MODULE.md` (blocked/unreachable goals and
footprint contracts), `diagnostics/nav2-failure.md` (costmap footprint inspection).

Observation: a new full rectangular-footprint RPP test placed one lethal cell at
the robot center and expected a collision exception. The controller instead
reduced speed through cost regulation. The current source diagnosis is that the
installed footprint checker samples polygon edges, not filled polygon interiors.
The direct primitive discriminating check now passed. Existing small-circle
fixtures cannot be copied blindly to this larger padded rectangle.

Evidence: `SESSION_HANDOFF.md` usage-limit/profile-build checkpoint;
`src/amr_mpc_controller/test/test_final_position_rpp.cpp`, collision assertion in
`ReverseAndTurnAndCollisionRemainInherited`; installed Nav2 1.1.20 checker.
This is presently a test-fixture issue, not a confirmed skill defect.
Confirmed fixture evidence: collision_primitive_20261002_luna_xhigh_01/
collision_primitive_probe_fresh_sample_report.md; same map/pose and one lethal
cell, center cost0/no-throw versus actual footprint vertex cost254 and the exact
inherited collision error from both public/private controllers. One accepted
physical observation stayed within unchanged .2s ROS/steady freshness.

Possible improvement: advise authors to derive negative fixture cells from the
actual transformed padded footprint and assert the installed collision primitive
detects the obstruction before testing controller rejection. Distinguish footprint
collision from center-cell cost regulation; never enlarge obstacles merely to
force an expected result.

Remaining validation: confirm the proposed guidance is missing or misleading in
the relevant skill before elevating status; a fixture mistake alone does not
establish a skill defect. Primitive/controller checks passed, not simulation.
