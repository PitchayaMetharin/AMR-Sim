# AMR Session Handoff

Latest resume: 2026-10-07. Workspace root is `/home/pete/amr_ws`; define
`AMR_WS=/home/pete/amr_ws` once and derive paths from it.

## Current status and authorization

Goal: functional Product A, Product B and return-home in a fresh simulation.
User's model policy: Codex models only; Sol/high for planning, diagnosis and
independent review; Luna/max for bounded source writing. Source fixes, independent
reviews, replay contracts and captured clearance PASS. Controller/manipulation
readiness, host/GUI and identity preparation/review PASS.

**Native45 stopped at its first mandatory runtime gate, before Product A.** Host
preflight and startup clock/costmap passed. The 12.0-second runtime sample had
1,785 samples over 11.988736581 seconds of real time and 5.946666072 seconds of
simulated time: aggregate RTF 0.496021080438484 and median RTF
0.542964161103998, both below the required 0.90. The minimum, mean and maximum
were 0.112201787394848, 0.533520782575226 and 0.907785257929051.
Hardware rendering was selected (`/dev/dri/renderD128`); Gazebo processes had GPU
card 1 open and software rendering was not forced. Native44's comparable gate
passed at median RTF 1.000012150149042 and aggregate 0.994488293329035.

The wrapper exited 1 after 81.061 seconds; host and factory exited 0, the runtime
gate exited 1, no owned processes survived, and the lifecycle audit passed
integrity (166 records, 49,594 bytes); all 65 source/input pins were unchanged.
No Product A/B/home actions, bag or analyzer gates ran. Exact launch argv and
environment (`ROS_DOMAIN_ID=232`, GUI enabled), wrapper result and stop receipt
are recorded under
`phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/native45_readiness_20261007T094501Z`:
`native45_launch.command.json`, `native45_launch.result.json` and
`runtime_stop_receipt.json`. Evidence is under
`phase14_evidence/factory_full_validation_20261003_45/`, including
`evidence/runtime_preflight.txt`, raw statistics, `runtime.log`, `factory.log` and
`result.json`; the root stop receipt and unchanged input pins are retained in the
readiness evidence. Do not reuse the run path or repeat the run without new
discriminating evidence.

**Native45 slowdown explained by host power state (2026-10-07, Claude session).**
Native44/45 compared: identical launch argv, process/bridge set, owned run source
(run ID only) and installed Gazebo-path files (install is symlinked to `src`; no
`install/*/lib` change between runs). Native45 was slow for the whole run, not only
the sample (Gazebo sim/real at sample start 0.53 vs Native44 0.87; per-second RTF
0.38-0.62, no recovery). Host: ThinkPad, Alder Lake iGPU (i915). PPD
`/var/lib/power-profiles-daemon/state.ini` was rewritten to `power-saver` on
2026-10-06 14:01 (between runs); firmware `platform_profile` was `low-power`
during Native45 (same boot, 2026-10-07 15:32). Native44's profile is not recorded.

Discriminating run (user set `performance`, AC plugged in; same boot, same launch,
stop after runtime gate): median RTF 1.000032101033653, aggregate
0.995498044901091 (3,583 samples), unchanged preflight PASS; host/runtime exit 0;
survivors none. Acceptance check written and hash-pinned before implementation
(`acceptance_check.py`, SHA256
`029d89f46c30ef04eb1835841903283bbd134545bebad567277956ae7910df6d`), rerun by root:
`verdict=SUPPORTED`, exit 0. Telemetry (0.5 s): Gazebo main thread ~45% of one core,
run delay ~1-2% of wall, CPU PSI avg10 3-8, memory/IO 0, GPU ~47% RC6 idle.
Remaining confound: profile and AC both differed from Native45 (Native45 AC state
unknown). Either way the operating condition is AC + `performance`.
Evidence: `phase14_evidence/native45_speed_profile_performance_20261007/` and
`phase14_evidence/stability_20261003/native45_speed_profile_20261007/` (runner
`bda1efc6...`, sampler `ae94da7f...`, launch `663367dd...`; ~6.05 MB total).
Process note: per explicit user instruction this slice used Claude (Opus 5.5 root,
Sonnet 5.5/medium writer/runner), not the Codex assignment below; root review only,
no independent review.

The custom speed-diagnostic harness under
`phase14_evidence/stability_20261003/native45_speed_diagnostic_20261007T101602Z/`
(rejected first version SHA256
`89ca884094546a1dbef9a77fadfa46a3b8d8be4af0a0bce64dea62f57113d743`; partial
correction `7460fee7ae97866430560ee9d7dc16d1434200363d148c9e336f1404000e85dd`,
incomplete and unverified) is superseded and retired; preserve it as evidence and
do not execute or resume it.

Safe resume: before any full run, confirm AC online and `performance`
(`cat /sys/class/power_supply/AC/online`, `powerprofilesctl get`). Next candidate:
fresh full Product A/B/home identity run (Native46) under those conditions,
pending user authorization. Optional: one AC + `power-saver` startup run to separate
the AC confound; optional host-preflight check for AC/profile. Do not relax the RTF
gate or thresholds.

| Item | Current state | Next required proof |
|---|---|---|
| Footprint/smoother/planner-map source corrections | Accepted; reviews/contracts PASS | Preserve behavior/pins |
| Captured clearance | PASS100cases/500stages | Preserve evidence |
| Readiness and identity preparation | All PASS, independent reviews | Preserve pins/gates |
| Native45 runtime | RTF gate FAIL before Product A; clean teardown | Explained: host power state |
| Speed diagnostic (AC + performance) | Runtime gate PASS, checker SUPPORTED | Keep host condition for future runs |
| Product A/B/home and strict closure | Not exercised in45 | Native46 under AC + performance |

Current user authorization covers this handoff update; Native46 and any source or
preflight change need explicit user authorization.

**Native46/47 and Product B speed work (2026-10-07, Claude session, user-directed loop).**
Native46 (AC + performance): all startup gates PASS, Product A PASS, Product B exit 2
(240 s transport timeout during final retreat; transport otherwise completed). Bag
analysis: (1) outbound A->B weave: arbiter angular slew 0.40 rad/s^2 under the
0.64 rad/s cap lagged controller turn commands ~1 s (ctrl vs /amr/control/cmd_vel),
causing +-25-30 deg overshoot and HeadingLatched stop-rotate cycles; (2) short
GridBased (Smac lattice, 0.5 m turning radius) goals with a 5 cm lateral offset
became 2.7-2.9 m loops (Native46 dock B twice, Native47 dock A); (3) one repeated
dock-B approach (~45 s), cause not yet diagnosed.
Native47 tried a 2 m/s envelope at user request: unsafe near docks (loop path kept
remaining length > approach slowdown, 2 m/s bursts), reverted byte-exact to the
Native46 config (pins verified). Native47 then stopped at the retained-evidence
storage gate (evidence ~932 MB allocated); teardown clean.
Current uncommitted fixes (tests written first, red then green):
- `amr_mission` mission_supervisor_node.cpp: GridBased goals 0.07 m < d < 1.0 m use
  PrecisionGridBased (ExactGoalLattice dispatch unchanged). amr_mission 32/32 PASS.
- Arbiter `max_angular_acceleration` and Gazebo DiffDrive angular acceleration
  0.40 -> 1.0 rad/s^2 (controller rotate ramp stays 0.40). amr_control 14/14,
  amr_mpc_controller 71/71 PASS; amr_description 37/37 via direct pytest (62.8 s;
  colcon ctest 60 s default timeout expires - pre-existing borderline).
Runtime validation (Native48) is BLOCKED on evidence storage: removing old run bags
was refused by the session safety classifier; the user must reclaim storage.

**Native50 FULL PASS (2026-10-07):** all 17 gates exit 0 incl. product101, product102,
home, final_factory_proof and both unchanged analyzers; success True, survivors [].
Changes (uncommitted, tests first): mission short GridBased goals (<1.0 m) -> PrecisionGridBased;
arbiter/DiffDrive angular accel 1.0; kDesiredProduct102SlotBaseX 0.755 -> 0.748 (release IK
self-collision-free band 0.7430-0.7635 m; offline product102_arm_branch_test); Product102
upright grasp seed with upright-only IK + wrist path constraints, legacy flipped fallback.
Captured clearance replay cannot be regenerated for the new stance (capture taken at -3.345);
see native45_clearance/claude_stance748_20261007/NOTE.md. Native48 lifecycle DDS response race
(one-off, Native49 identical rerun passed). Remaining: wrist null-space spin (j4/j6 +-2.5 rad)
during the Cartesian loaded lift after the upright grasp (j5~0 singularity) - next fix.

## Assignments and process requirements

- Sol/medium orchestrates.
- `gpt-6.1-sol`/high diagnoses and plans; a separate non-author Sol/high reviews.
  `/root/smoother_review` completed both bounded reviews. The former reviewer's
  usage-limit failure remains historical evidence, not an outstanding review.
  Do not silently substitute models or claim author checks as independent review.
- Codex models only. `gpt-6-luna`/max is the source writer under Sol/high's bounded
  packets. Separate `gpt-6.1-sol`/high performs independent review. No Claude CLI
  workers. Never use Astra. Preserve historical authorship and evidence.
- Report newly observed writer mistakes immediately with concrete evidence.
  A failed mandatory gate stops further implementation/runtime and returns to
  diagnosis. All source writers edit/report/HOLD; root executes checks.
- Preserve unrelated dirty work, protected `AMR_CODEX_HANDOFF.md`, ledgers and
  all-model210/Luna109 counters. No ledger entries were authorized this session.
  No stage/commit/push, dependency install, system changes or outside-workspace
  modifications. Do not recreate `SESSION_HANDOFF_HISTORY.md`.
- ROS domain is232; every command must satisfy0..232. Keep logs/cache/tmp/evidence
  inside the workspace. Logs and evidence each have a1,000,000,000-byte cap in
  both logical and allocated size; follow existing receipt rules at the cap.
  Rechecked storage: `{"logical_bytes": {"logs": 576365699, "evidence": 789629624}, "allocated_bytes": {"logs": 634585088, "evidence": 809017344}}`.

## Accepted smoother verification; planner independent review

Read `.codex/NATIVE45_SMOOTHER_COMPLETION_CORRECTION_20261007.md` before resuming
this slice. Evidence root, relative to AMR_WS:

`phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/sonnet_smoother_20261006T185243Z`

Sonnet edited only `src/amr_navigation/test/replay/production_precision_replay.cpp`.
`run_stage` uses `consume_smoothed_stage`: false, missing, nonboolean or nonobject
completion rejects before strict sweeps and successful records. The real configured
smoother still receives1s; the old backend default remains2s. Helper JSON `success`
is composite endpoint/collision compatibility, not a production action status.
Production mission requires `SUCCEEDED`, a present result and `was_completed`;
the synchronous replay proves normal invocation plus explicit completion instead
of inventing an action result. The stale reported-not-gated limitation was corrected.

Smoother acceptance source pin, before planner-map correction:
`43717c758ccbfc27d52a35da1549209d34dd2829d9e3053190bbdce5c3025897`.
Unchanged supporting pins:

- backend: `60635c2e46e6ac5af7564018cecb85bf4afeb83129c0e1d4cf681933ea30bb56`
- replay CMake: `5d58aadb0e799105a154dc63f04221a3d4faa2b4f06a09b32d8c6a1bb1836ed6`
- adapter: `92ee621ef44ab0ab6932d66932f1ea506a34fdcb94eda84f2261e69f5df8d587`

Writer HOLD exit0; PID/PGID285318/rootexec78597 ended. Root complete scoped diff
inspection and whitespace check PASS. Root build PASS exit0,13.0s; retained
`build.stdout.log`, `build.stderr.log`, `build.status`. Rootexec34333 ended.
Focused `--self-test-smoother-completion` ran once on resume: four groups PASS,
zero failed, exit0,0.717s, ROS_DOMAIN_ID232. Fresh command, environment, binary hash,
stdout/stderr, status and canonical storage checks are retained under this evidence
root at `resume_verification_20261007T085619Z/`. All four source pins matched.
Separate non-author `/root/smoother_review` (`gpt-6.1-sol`/high) completed independent
SPEC/QUALITY PASS on exact source and fresh retained evidence. No material finding;
outer stop behavior and footprint/strict-grid preservation were inspected. At that
source-review checkpoint, full captured replay was still unverified; it subsequently
passed as documented below. Production action-server execution and runtime
acceptance remain unverified.

Writer process deviation: a Bash `sed -i` edited two usage strings despite the
Edit/Write-only instruction. It stayed in the allowed source file, was disclosed
in the report and immediately reported to the user. Evidence:
`shell_source_edit_deviation.json`, `implementation_report.md`, scoped diff.

Current slice: Sol/high prepared `.codex/NATIVE45_PLANNER_MAP_CORRECTION_20261007.md`;
root released it and `/root/planner_writer` (`gpt-6-luna`/max) completed source-only
edit/report/HOLD. Root executed all source gates below. Fresh evidence root relative
to AMR_WS: `phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/codex_planner_map_20261007T090839Z`.
Before-image and source/support pins are retained there. Separate non-author
`/root/smoother_review` (`gpt-6.1-sol`/high) completed planner SPEC/QUALITY PASS,
with no material findings. No build/test/replay is active. Luna/max identity
preparation completed and passed root verification and independent review; it is
now idle/HOLD. Native45 runtime ended exit1 at the mandatory RTF gate, with no
survivors.
`/root/planner_plan` completed parity/loader assessment and the readiness plan.
Root captured replay exit0,14.319s:100 admittedcases/500stages PASS, first_failure
null, all direct_precision, no incomplete smoother. Strict samples159552;
cells72314609. Evidence: captured_replay.* and clearance_report.json in planner
evidence root. Runtime properties remain not_exercised. Root readiness completed:7object freshness checks/24byte comparisons PASS; six
required named CTests each exit0, no82/centered reruns. Fresh132008192reserve,
strict global hostpreflight/renderdevice/no forcedsoftware/no processes and GUI PASS.
Readiness evidence: `phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/native45_readiness_20261007T094501Z`.
Luna/max identity prep completed/HOLD:6files<1MB,4exactwrapperdeltas,normalized
byte/ASTparity/restoration/compilePASS,50inputsunchanged. Root identityverification
and separate Sol/high SPEC/QUALITY PASS; supplemental15mission/interfacepins retained.
Source unchanged; launched Native45 stopped at mandatory RTFgate as recorded above.
Root helper ancestry mistake stalled before checks; ownedPID interrupted, evidence
helper_correction.json retained, corrected helper then all gates PASS.

```bash
# Set AMR_WS once as above; resolve E to the smoother evidence root.
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=232
export ROS_LOG_DIR="$E/ros_logs" TMPDIR="$E/tmp" XDG_CACHE_HOME="$E/cache"
export PYTHONDONTWRITEBYTECODE=1
ulimit -c 0
"$AMR_WS/build/amr_navigation/test/replay/production_precision_replay" --self-test-smoother-completion
```

Check storage/owned directories first and use fresh result filenames. The source
build already passed; repeat it only after source changes or new evidence justifies
it. Do not rerun accepted footprint checks absent a relevant source change.
The planner-map correction and captured replay are accepted. Do not repeat the
captured replay or launch another simulation while the Native45 speed blocker is
unresolved, unless new evidence gives the run a discriminating purpose.

## Implemented planner-map reuse/routing correction

The new shared `plan_fixture_stage` restores every captured byte from fixture.data
under the live map mutex, classifies routing before plugin execution, releases the
lock and invokes the real plugin once. run_stage uses its actual path and saved
routing. Strict collision grids remain separate immutable copies.
Current source SHA256: `23bf32e0ff719f0ba0a7d6a8336532d554b4b26f6ec9340bbd8a12066b167c8f`.
Supporting backend/CMake/adapter and protected artifact/ledger pins are unchanged.
Root scoped diff/whitespace inspection PASS; build exit0,19.375s; focused four
groups PASS/zero failures/exit0,1.419s. Evidence-only old-sequence baseline compiled
and linked exit0, then exit1 as expected: ordering and restoration assertions FAIL,
immutable and clear-positive groups PASS. Corrected source/binary pins unchanged.
Full native45_clearance_contract CTest PASS (5.42s); all three existing replay
contracts PASS. Fresh command/env/stdout/stderr/status/hash/storage records are
retained in the current planner evidence root; baseline failures remain retained.
Separate independent SPEC/QUALITY review PASS on the exact final source/diffs and
fresh evidence, no material findings. This accepts source behavior/contracts;
captured clearance passed as recorded above; runtime remains unproved.
Writer reported one incorrect read-only ROS header lookup (exit2), corrected
without source impact; recorded in implementation_report.md. Counters unchanged.
Diagnosis/release/review remain Sol/high responsibilities; Luna/max is the writer.

Current config/manifest source-install byte parity and precision plugin build-install
byte parity PASS. Frozen scene hash and its planner/smoother/footprint/resolution
values match current installed config. Actual loader initialization paths/hashes are
retained in `parity_loader_evidence.json`; eager-bind probe in
`loader_bind_now_evidence.json` resolves exactly one workspace libtf2 provider with
no symbol lookup/load errors. Intentional --help usage exit1 is identity evidence,
not a failed contract. Sol/high assessment PASS: intentional overlay provider and identical headers/symbol
coverage; installed tf2 differs from build only by verified RPATH removal. Captured
replay then passed with eager binding and all configured plugin identities retained.

## Accepted footprint correction and numeric interpretation

Footprint consistency allowance is±0.02m (20mm) per corresponding X/Y coordinate.
This compares expected scene and actual plugin outlines; it is NOT padding or
collision slack. Production padding remains0.01m (10mm). Strict collision geometry
conservatively encloses the nominal polygon plus both phase plugin polygons.
Stance and nominal scene-shape checks remain unchanged. Account for floating-point
rounding and judge physical simulation geometry using meaningful millimetre-scale
requirements; exact decimal text is not a physical correctness requirement.

Nominal double0.61 prints as0.60999999999999999 with17-digit formatting;
that is valid rounding. The original overstrict plugin comparison and subsequent
fragile text assertion were corrected. No robot geometry defect was established.
Final footprint acceptance at source91fba11f...924c: root build exit0,13.2s;
`--self-test-footprint`6groups PASS/0failed/exit0; Sol/high SPEC/QUALITY PASS.
Smoother changes now produce the current source hash above; footprint preservation
was confirmed in smoother independent review; whole captured replay subsequently passed; runtime remains unproved.

Evidence relative to the clearance root:
`sonnet_footprint_20261006T182650Z/assertion_repair/` contains build/test statuses,
`footprint.stdout.log` and `orchestrator_verification.json`. Verified again here.
Original footprint focused5PASS1FAIL remains in its parent; original C++ contract
8PASS2FAIL/exit8 remains in `sonnet_cpp_20261006T180036Z/`. Preserve all failures.

## Captured scene and adapter (accepted inputs, not clearance proof)

Clearance root relative to AMR_WS:
`phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/`.
Read `.codex/NATIVE45_CLEARANCE_DIAGNOSIS_20261007.md` for retained TF/polygon
reconstruction and `.codex/NATIVE45_CPP_RELEASE_20261007.md` for replay requirements.
Original broad implementation packet execution is suspended; newer bounded packets
control releases. Existing C++ target and full captured clearance are now accepted.

Complete-TF route is selected. Original extractor
`extract_tf_20261006T170734Z` recovered all8clear-window TF brackets but exited2
because an optional after-window AMCL sample was absent. Keep that exact annotation,
original exit status and evidence. The orchestrator packet had incorrectly made
that optional sample mandatory. No additional extraction is needed. Existing typed
stream also provides contemporaneous pre-dock TF; reconstruction residuals<1.4e-7m.
Current intended production config governs replay; historical binary equivalence
is unknown. Retained samples are moving and do not prove stationary start, settling,
fresh post-heading bias, controller tracking or terminal execution.

Adapter corrected and accepted: self-test PASS, independent SPEC/QUALITY PASS,
real captured adapter exit0:100 admitted cases (4nominal,16bias,80synthetic repair),
no exclusions. Scene at `sonnet_adapter_correction_20261006T174631Z/scene.json`:
SHA256 `d8fdd7c954b63fc55d7d768509412b2f69aa6583052f3b6e9a4a0ac73f4d0128`.
Adapter source pin appears above. Runtime properties remain `not_exercised`.
Initial writer fixture-directory failure and forbidden retry, correction launch
permission error, and empty Python heredoc deviation remain in their original
worker streams/reports; do not reset evidence or mistake counters.

## Native45 validation status

The previously planned source, replay, readiness and identity gates all passed:
captured clearance was 100 cases/500 stages with `first_failure=null`; readiness
passed seven freshness objects/24 byte comparisons and all six required CTests;
host/GUI preflight and identity preparation/review also passed. Their evidence
paths and exact pins are recorded above. Native45 was then launched once and
stopped at the first mandatory runtime gate: RTF failed before Product A. That
failure supersedes the old ordered launch plan; the remaining Product A/B/home,
strict closure and stability extraction have not been exercised.

Native44 had Product A success, Product B failure and no return-home proof. Do not
repeat its full bag decode or accepted Product B82-case/centered behavior tests while
pins remain unchanged. The0.58m diagnosis was disproved (centered source
Y=+0.100); no geometry correction is authorized.

## Product B source milestone (accepted, source-level only)

Completed on 2026-10-06 on top of Repair 2 (`.codex/B_BOUNDARY_C_REPAIR2_20261006.md`):

- Remaining-coverage packet (`.codex/B_REMAINING_COVERAGE_PACKET_20261006.md`):
  new centered tests for budget/rollback, cumulative budget, delayed acceptance
  and observer settling. They sit under `AMR_GATE6_CENTERED_COVERAGE_TESTS` in
  `test_gate6_pickup_retreat_behavior.cpp` and run in the new CTest target
  `gate6_centered_coverage_behavior_test` (TIMEOUT 90, ENV GTEST_FILTER, same
  source). The original `gate6_pickup_retreat_behavior_test` is unchanged at 82
  cases, TIMEOUT 90.
- Fixture fix: the optional reentrant callback group is now a fixture member. As
  a `SetUp` local it expired, because Humble holds custom groups weakly, so the
  servers never received goals.
- Batch review (gpt-6.1-sol) first REJECTED the batch with three findings. Fixes:
  - production `gate6_mass_stage.cpp` now runs a final `check_guard(false)`
    before dock success (`final_admission_guard_failed`);
  - the harness has a TF post-lookup hook and a terminal-succeeded record;
  - the post-dock test is deterministic (no wall wait);
  - a new cumulative-budget test (100→150→200→225 s) detects a per-stage reset.
- Validation outside the sandbox, evidence in
  `phase14_evidence/stability_20261003/b_turn_placement_20261006/packet_b/review_fix_20261006/`:
  - coverage target: 19 passed + 1 intentional skip, 30.5 s;
  - original target: 82/82, 85.1 s of 90 s, so headroom is tight;
  - Python contracts (moveit_config, product_test_contract,
    gate6_completion_contract, cycle_adapter): 208 passed;
  - scoped `git diff --check`: 0;
  - discrimination: the post-dock test FAILS on the pre-fix production file
    (`baseline_c1.log`) and passes after the fix (`restored_c1.log`).
- Re-review: ACCEPT-WITH-NOTES (gpt-6.1-sol). Milestone review: ACCEPT-WITH-NOTES
  (gpt-6-sol). Both are source-level only; they do not cover clearance or
  runtime acceptance.
- Current pins:
  - `gate6_mass_stage.cpp` `5bc9d3a37da497a038789d2f258069efbb040af185732f0624146e0bc04adf9a`
  - `test_gate6_pickup_retreat_behavior.cpp` `22af73271b05e9a25f5e220f2378df2b5ef2612024b4b10f4a3708b6ab2f8224`
  - `amr_manipulation/CMakeLists.txt` `7fed3c0e61d99755d231ef79e67d56e94341dbe24d697b088b577f4ac61b16df`

## Handoff recheck evidence

The previous accumulated handoff was preserved as a historical snapshot at
`sonnet_smoother_20261006T185243Z/session_handoff_before_recheck.md` under the
clearance evidence root. Its old active/resume instructions are superseded by
this document. That previous recheck only read source/status/evidence and updated this handoff;
no source edits, tests, replay, simulation, protected artifact or ledger changes.
