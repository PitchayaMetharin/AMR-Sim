# Native45 adapter correction — bounded release

Status: RELEASED for adapter-only edits to exact Sonnet5.5/medium.
This packet supersedes earlier check allocation: writer MUST NOT run self-test,
invoke adapter, build, CTest, or replay. Edit, inspect complete diff, report, HOLD.
Orchestrator owns all checks after HOLD; first nonzero mandatory check freezes edits
and returns to Sol/high. No new C++/CMake implementation in this correction slice.

## Objective and high-confidence diagnosis

Preserve partial adapter and its valid missing-fixture-mkdir correction. First
self-test failed because fixture parent was absent before planner.yaml write.
Sonnet repaired and reran rather than mandatory HOLD. Retain both traces:
sonnet_implementation_20261006T173312Z/selftest_failure_and_retry.json.
Source pins/identity requirements also omitted stream validation. Repair sequence
continuity was insufficiently explicit in Sol's initial packet; this clarified
construction resolves that specification gap, not automatically a Sonnet mistake.
User was notified of failure, stop deviation, stream omission and attribution correction.

## Allowed file and current state

ONLY src/amr_navigation/test/replay/native45_scene_adapter.py may be edited.
Exclusive evidence root E is assigned in environment. Evidence/report only within E.
Read AGENTS, SESSION_HANDOFF and current exact source before editing; git status.
Do not change CMake/backend/new C++ source, configs, handoff/ledgers, protected
AMR_CODEX_HANDOFF, manifests, unrelated changes or files outside workspace.
No dependency installation, deletes, commits, pushes, build/runtime checks.
Current adapter SHA256 must equal:
9b1449866b0f9f7c9100b74b5f02cb8e68eed257526365e2e1e92849b54861ba.
Initial worker before-images prove replay CMake/backend unchanged since release.
Historical counters210/109 and all evidence remain unchanged; reporting does not
permit ledger modifications.

## Required changes

1. PINNED_SHA256 must include typed_stream.jsonl.gz:
f81dc4619b4d8a0d5e66b93fe4d35faaaecdd18f4a3dfff23cb12b71b3c80c66.
Route stream identity through existing identify()/mandatory pin comparison, not
record-only hash (existing lines662–664). Synthetic fixture pin map must include
its stream hash. Add public build_scene() self-test that changes stream while
retaining original pin and proves rejection. No CLI pin bypass for real inputs.

2. Preserve 4 nominal +16 observed-bias constructed sequences. Replace current
repair representation (lines565–577) with standalone synthetic repair-start cases.
For each ±X/±Y physical miss0.010001 m from clear point, start physical XY at
clear+miss and yaw at stance yaw; localized pose subtracts selected captured bias.
Retain the parent captured-start ORIGINAL clear reference, never reset it to repair
start. Enforce unchanged registered approach admission and0.15 m original-reference
bound for repair start. There are80 representative repair candidates across20
parent cases; report excluded cases with exact production admission reasons.
Construct exactly one repair tangent→translation→arrival heading, then dock and
subsequent final-heading action (source margin0.03 rad). Preserve repair stage
suffixes and explicitly label original parent reference, synthetic start, selected
observed bias and travel-producing-miss not_exercised. Do not invent a connecting
motion or captured post-heading observation. Every admitted sequence is continuous:
each previous goal equals next start in XY and wrapped yaw.

3. Self-test SOURCE (writer does not execute it) must verify stream tamper rejection,
synthetic stream pin,4 nominal16 bias80 repair candidate counts, admitted endpoint
continuity, repair start provenance/stance heading/original parent reference,
exactly one repair cycle, and exclusion of a case safe relative to itself but beyond
parent original reference. Existing schema/TF/padding/malformed-input tests remain.
Do not weaken thresholds or remove meaningful negative tests.

## Invariants, prediction, non-goals

Preserve extraction exit2/missing optional AMCL annotation, current raw costs,
no extrapolation, numerical consistency1e-6 m (not arrival threshold), shared stance
math, source-backed margin0.03 rad, registered geometry/bounds and runtime
not_exercised fields. No fixture shift, production behavior change, historical
binary equivalence, simulated motion, source acceptance or clearance claim.
Prediction: modified retained stream fails public build_scene before acceptance;
admitted repair sequences start at declared miss, remain continuous and reject any
original-reference violation. A passing self-test alone is not clearance.

## Writer steps and report

Read exact source/status, confirm allowed file/hash, make smallest coherent edits,
inspect complete before/after adapter diff, save report E/implementation_report.md.
No self-test/adapter/build/test command. Any material ambiguity, input mismatch or
denied operation ends writer turn with HOLD report. No silent workaround.
Report changed file/delta, source hashes, inspected commands/status, evidence,
unverified checks and potential mistakes. Then HOLD.

## Orchestrator validation after HOLD

Set valid ROS_DOMAIN_ID232 and workspace-owned TMPDIR/cache/log paths; reserve30 MB
under logical/allocated1,000,000,000-byte caps. Verify scope/hash and full diff.
Run exactly:
python3 -B src/amr_navigation/test/replay/native45_scene_adapter.py --self-test
Retain true exit status directly, no trailing echo or masked pipeline.
Nonzero→freeze editing and return Sol/high diagnosis. Pass→independent adapter
review before separate remaining-C++/CMake release. No captured replay until full
implementation source checks and independent completed-packet review.
