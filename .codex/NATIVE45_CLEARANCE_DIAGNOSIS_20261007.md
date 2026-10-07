# Native45 clearance diagnosis — 2026-10-07

Read-only diagnosis: gpt-6.1-sol/high. Orchestrator: Sol/medium.
No Sonnet implementation has started; no Sonnet mistake is established.

## Established mechanism

Retained extraction `phase14_evidence/stability_20261003/b_turn_placement_20261006/native45_clearance/extract_tf_20261006T170734Z/summary.json`
reports all eight geometry TF brackets and physical-pose boundaries recovered.
The sole missing item is `/amr/amcl_pose boundary after geometry window`.
`extract_tf.py:230–278` requires before/after samples for both physical and
AMCL topics. SESSION_HANDOFF records AMCL as optional in the original spec but
requires a user choice before accepting the retained extraction. That choice
remains pending; recommendation is to use the complete TF extraction.
Do not rewrite the retained failed status or waive it silently.

## Replay gaps supported by current source

- `planner_replay_backend.cpp:657` uses a 2-second smoother budget; production
  `mission_supervisor_node.cpp:653` uses 1 second.
- `loadSnapshot` already preserves raw `snapshot.costmap` costs. Keep this.
- The loader does not apply footprint padding; production planner config says
  0.01 m. Recover the effective historical shape from published polygons and
  stamped TF before assuming how padding was represented.
- `checkWorldPath` uses the Nav2 boundary checker; required full polygon proof
  additionally needs interior coverage, dense motion sweeps, and explicit
  rejection of empty paths, unknown/lethal cells and outside-map coverage.
- Pre-dock geometry stamps (~409.98–410.37 s) lie outside the recovered clear-start
  TF window (~401.60–402.00 s). Do not apply clear-start transforms to pre-dock
  local geometry. The written plan does not authorize a narrower static-only
  clearance claim.
- Retained logs establish PrecisionGridBased/SimpleSmoother plugin loading;
  inspected historical evidence does not yet establish footprint/padding or
  smoother parameter values. Current source alone is insufficient historical proof.

## Tentative bounded implementation scope — NOT released

`src/amr_navigation/test/replay/CMakeLists.txt`,
`planner_replay_backend.cpp`, new `native45_scene_adapter.py`, and new
`production_precision_replay.cpp`.
Use installed production planner and configured 1-second smoother. Derive actual
centered-B tangent turn, clear translation, arrival turn and dock corridor from
current `gate6_mass_stage.cpp:1520` onward. Preserve production behavior, raw
costs, thresholds, ownership, accepted source checks and protected artifacts.

Prediction: stamped reconstruction matches observed effective footprints within
an evidence-grounded tolerance; each complete padded polygon sweep produces
explicit collision-free evidence or fails at a named stage/pose/cell.

## Before releasing Sonnet

1. Obtain user's pending AMCL boundary choice.
2. Compute stamped reconstruction, physical observations and polygon discrepancy.
3. Resolve required pre-dock captured-scene coverage using valid contemporaneous TF.
4. Effective captured footprint is now resolved (see below); resolve historical
   smoother settings and planner/config identity.
5. Define exact stage cases, schema, CLI, validation targets and commands.
6. Verify exact Sonnet 5.5/medium execution metadata and availability; no substitution.

Read-only retained JSON/JSONL reconstruction may proceed while choice is pending.
No production edits, decoding, simulation or accepted-test repetition is authorized
by this tentative packet. Any ambiguity returns to Sol/high before implementation.
Keep all task outputs in an exclusive workspace evidence directory and check both
logical/allocated 1,000,000,000-byte caps before reserving space. Preserve receipts.
Report each established Sonnet mistake to the user with evidence and needed
correction; faithful execution of a flawed plan is not itself a Sonnet mistake.

## Completed retained-data reconstruction

Sol/high loaded retained JSON/JSONL only, bracketed geometry acquisition stamps,
interpolated planar translation and shortest wrapped yaw, composed
map→odom→base_footprint, and inverse-transformed published polygon vertices.
At 401.733293160 s map→base is (-2.4073143746, -0.0137157049, 3.0109036132).
Vertices match body half-extents ±0.61/±0.41 m within 9.3331e-8 m; the local
footprint at 401.996626467 s matches within 1.2223e-7 m. Across clear/pre-dock
polygons, edge lengths match 0.82/1.22 m within 1.9525e-7 m.
Effective captured padded dimensions are resolved. Proposed 1e-6 m numerical
consistency tolerance exceeds measured serialization residuals; this is not a
changed safety or arrival threshold and has not yet been released in a packet.
Physical-minus-localized bias at the global footprint is
(+0.00180331 m, +0.03065246 m, -0.00701060 rad).

Ground-truth X moves 28.34 mm across the 0.38 s window: no stationary-start proof.
No Native44 observation proves new choreography's post-heading bias. A constructed
stage replay must state its scope without inventing those future observations.
Pre-dock stored TF (map→odom 410.603292293 s; odom→base 410.499958950 s) postdates
geometry. Rigid dimensions are resolved, contemporaneous pre-dock pose is not.
Next permissible source is the retained typed stream; samples.json.gz receipt-time
coordinates do not themselves establish acquisition-stamped TF. Do not decode bags
without the pending authorization decision and a bounded evidence plan.
Historical planner/config identity remains UNKNOWN in source_pins.json/execution.json.

Current routing: gate6_mass_stage.cpp:1710 onward uses optional tangent heading
and translation when distance exceeds 0.01 m, mandatory arrival heading, at most
one repair, precision docking and post-dock proof. Same-position headings select
PrecisionGridBased through mission_supervisor_node.cpp:477 (≤0.07 m branch).

Input identities, under phase14_evidence/stability_20261003/b_turn_placement_20261006:

- native44_baseline/clear_scene_snapshot.json:
  accdcc93238d2d2803b5a853e72d2b00fc9af2140380b394d11406f203d95158
- native44_baseline/dock_scene_snapshot.json:
  f9db23eda45e159f22447e386f29da1c9364f9309222d812a7083101a6d14135
- native45_clearance/extract_tf_20261006T170734Z/summary.json:
  c1b2dcf008aba331d849ee7f2bff550a76c05113a1411506df8a7502983e6802
- same root, tf_edge_rows.jsonl:
  a131920bc1159b1595bb2901271508a37ce7ca2bec7cf258af165012273ae35f
- same root, pose_rows.jsonl:
  e581504e97e696e30cd1f4b8b717ffcbe5e801b4df968d9d1eb7837b24e1c294

Orchestrator independently rehashed clear/dock snapshots and TF/pose JSONL;
all four agree. Summary identity above is analyst-reported.
One analyst schema inspection treated a list as a dictionary and raised
AttributeError; it produced no files or acceptance claims. This is not a Sonnet
mistake. No Sonnet has executed. Production sources are unchanged.

## Subsequent resolution and release

The user's latest "continue", following the recommended complete-TF route, was
interpreted as selecting that route and stated to the user. Original extractor
exit2/status and missing optional AMCL annotation remain immutable.
Current authority/resume state lives in SESSION_HANDOFF.md; pending-selection
statements above describe the earlier diagnosis, not current policy.

The retained typed_stream.jsonl.gz provides all pre-dock acquisition-stamped
map→odom→base brackets and exact physical observations. Compressed SHA256:
f81dc4619b4d8a0d5e66b93fe4d35faaaecdd18f4a3dfff23cb12b71b3c80c66.
One bounded pass read 78,718 rows / 76,641,803 uncompressed bytes; no bag decode.
Global footprint discrepancy is 1.372e-7 m at 410.199958980 s;
local is 9.941e-8 m at 410.366625630 s. Both satisfy numerical 1e-6 m check.
This resolves pre-dock stamped geometry. No parameter topics exist in the stream.
Current intended installed production config governs the constructed geometric
replay; historical binary/config equivalence remains unclaimed. Moving captures
and absent future bias observations remain runtime limitations, not a reason to
invent offline runtime acceptance.

Sol/high reviewed bounded stage cases and shared stance verification. Subsequent
placement final heading has source-backed 0.03 rad margin (gate6_mass_stage.cpp:
3312 and 3646/3651); it is a constructed action AFTER centered docking. Numerical
stance check uses shared C++ final_placement_stance(), with agreement <=1e-9.

Released packet: .codex/NATIVE45_CLEARANCE_IMPLEMENTATION_20261007.md.
Exact Sonnet5.5/medium access was probed successfully; live CLI init confirms
claude-sonnet-5-5. Worker must hold after focused source checks for independent
review before captured-scene replay. No native simulation is included in packet.
