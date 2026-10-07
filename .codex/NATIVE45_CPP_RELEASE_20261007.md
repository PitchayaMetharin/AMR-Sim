# Native45 remaining C++ replay slice — RELEASED

Exact Sonnet5.5/medium is sole source writer. Original implementation packet's
technical requirements remain binding, but its execution release is suspended;
THIS packet releases only remaining C++/CMake work. No further plan redesign.
Writer edits/diff/reports then HOLD; orchestrator runs all mandatory checks.

## Verified inputs and scope

Adapter source corrected and independently accepted SPEC/QUALITY PASS by Sol/high.
SHA25692ee621ef44ab0ab6932d66932f1ea506a34fdcb94eda84f2261e69f5df8d587.
Root self-test exit0; actual captured-input adapter exit0 produced100 admitted
cases (4 nominal,16 bias_sensitivity,80 synthetic_repair) and no exclusions.
Current intended production configuration governs replay; historical equivalence
and runtime settling/fresh bias/terminal execution remain unclaimed.

Read actual scene/schema summary under the preceding adapter evidence root:
phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/sonnet_adapter_correction_20261006T174631Z/
scene.json SHA256d8fdd7c954b63fc55d7d768509412b2f69aa6583052f3b6e9a4a0ac73f4d0128
scene_schema_summary.json is a compact actual-data schema/config/example receipt.
Do not modify these files or adapter source. E is the new assigned writer evidence
root; only reports/diffs may be written there.

Allowed sources ONLY:
- src/amr_navigation/test/replay/CMakeLists.txt
- src/amr_navigation/test/replay/planner_replay_backend.cpp (necessary reusable support)
- new src/amr_navigation/test/replay/production_precision_replay.cpp

Target hashes unchanged from original release:
CMake ab8ca5559c9f649fd7cc165c5daa14a4f5e9a365b2ba05e25ef56734aeb21ccb
backend0a3c6410e93a97c11f4ed102e75f35257e44ef418d3a62b51229adbacff09d6b.
Numerous unrelated worktree changes exist; preserve every one. Read current source,
AGENTS, SESSION_HANDOFF and original technical packet before editing; git status.
No adapter/config/production planner/mission/manipulation/manifest/ledger/handoff
edit; protected AMR_CODEX_HANDOFF immutable. No install/dependencies/push/commit,
unrelated cleanup, bag decode, deletion or files outside workspace.

## Exact schema contract (use actual summary)

schema=native45_clearance_scene_v1.
scenes.clear.grids.global/local; scenes.pre_dock.grids.global/local.
Grid width,height,resolution,origin_x,origin_y,data,frame_id,stamp_ns,data_sha256.
Local map_to_odom is map←odom; INVERT it for map-path→local-grid coordinates.
planning_grid_by_phase selects clear.global for phase clear, pre_dock.global for dock.
Every raw/smoothed path must also pass all FOUR captured grids, no clipping.
Case kind strings are nominal,bias_sensitivity,synthetic_repair. Require expected
4/16/80 roster, unique IDs, continuous stages and no excluded mandatory nominal/bias
case for overall PASS. Repair exclusions require exact admission reason.
Stage keys name,phase,start,goal,provenance; pose keys x,y,yaw. Explicit provenance
and runtime not_exercised survive report. Subsequent final_heading_margin is AFTER
centered docking, source-backed command margin0.03 rad; not a sixth action inside
centered routine and not an executed action claim.

Read config.precision_grid_based/simple_smoother/footprint_string/footprint_padding/
padded_footprint/smoother_budget_s; registry input arrays from actual scene.
Independently call shared final_placement_stance() and require stance<=1e-9 agreement.
Source-only include path for new target:
${CMAKE_CURRENT_SOURCE_DIR}/../../../amr_interfaces/include.
No amr_interfaces package manifest change/dependency needed.

## Required implementation

Add production_precision_replay --scene PATH --output PATH and --self-test.
Use installed PrecisionNavfnPlanner via pluginlib configure/activate/createPlan,
with captured phase-selected raw global costs and current real parameters.
Reuse existing Costmap2DROS configuration seam in exact_goal_lattice_contract_test.
Verify plugin padded footprint agrees with scene; preserve0.01 padding/current shape.
Record direct precision vs Navfn fallback using existing helper if needed; generic
fallback success never means clearance. Call actual plugin, not helper-only path.
Run current configured SimpleSmoother for ONE second, production mission budget.
Existing backend CLI/smoother helper default remains TWO seconds; any shared support
parameter must keep its old default2 and dedicated replay explicitly passes1.
Expose actual smoothed pose vector if needed for strict checks; do not substitute
endpoint compatibility or old perimeter-only check for complete polygon proof.

Reject empty/nonfinite/malformed path/frame/grid/pose. Preserve production center
rejection (cost>=253), perimeter semantics (253 allowed away from center), and full
interior unknown/lethal rejection. Sweep complete polygon boundary AND interior
cells. Interpolation bound<=0.5*grid resolution for translation plus corner rotation
travel. Explicit start and terminal heading sweeps are mandatory even if plugin
path does not emit intermediate heading motion. Check raw and smoothed paths
against all4 grids with their own frames/stamped transforms. Outside map is failure,
not clipping or skipping. First failed case/stage/map/pose/cell stops full replay.
Success reports count all required exercised cases/stages and unproved runtime
properties. Bound logs/report/path output under30 MB reserve; no per-cell success
noise. Runtime reports must retain useful first-failure proof and units.

CMake: executable production_precision_replay linked precision_navfn_planner plus
needed existing Nav2/core/pluginlib/navfn/smoother/replay dependencies. Preserve
existing targets/CLI and expectations. Register native45_clearance_contract running
production_precision_replay --self-test. Existing include-main-rename seam is allowed.

Self-test SOURCE (writer does NOT execute) must cover:
empty path, lethal/unknown interior with clear center/perimeter, intermediate-turn
collision with clear endpoints, outside-map footprint, clear positive, malformed
frame/grid/pose, expected padded shape and explicit1 s smoother selection. Demonstrate
old permissive checker accepts known-broken empty/interior baseline while strict
checker rejects; do not weaken old test. Test meaningful public seams, not just
constants or plugin-load smoke. Build/actual scene clearance will be run by root.

## Writer HOLD and root checks

Writer may inspect source/status/hash/diff; edit3 allowed source files; save complete
before/after scoped diff and E/implementation_report.md, then HOLD. NO Python
execution, adapter invocation, compile/build, self-test, CTest, replay or simulation.
Any material ambiguity/denied operation/hash mismatch→HOLD; do not invent behavior.
Report exact changes/hashes/commands/statuses and unverified conditions.

After HOLD root sets ROS_DOMAIN_ID232, sources Humble+workspace, workspace-owned
ROS_LOG_DIR/TMPDIR/XDG_CACHE_HOME, CMAKE_BUILD_PARALLEL_LEVEL2 and reserve30 MB.
Run sequentially, capture actual status, STOP first failure:
1. scoped diff inspection and git diff --check;
2. colcon --log-base "$E/build_logs" build --packages-select amr_navigation --executor sequential --cmake-args -DBUILD_TESTING=ON;
3. ctest --test-dir build/amr_navigation --output-on-failure -R '^native45_clearance_contract$';
4. affected existing replay contracts once if shared support changed, narrowed by review.
Then independent completed-packet review, source/build/install plugin/config parity,
scene input hash verification, and captured replay:
build/amr_navigation/test/replay/production_precision_replay --scene PREVIOUS_E/scene.json --output "$E/clearance_report.json".
Root resolves PREVIOUS_E exact path before execution. No full replay before review.
No native simulation until this clearance gate PASS and retained readiness/identity
gates PASS. Failed build/test/review/replay returns Sol/high diagnosis before edits.
