# G1 future-run ownership evidence packet

Coordinator release: exact gpt-6-luna/max sole production writer; Sol/high plans,
analyzes and reviews. No redelegation. This packet is related local AMR work
authorized by the user. Keep generated files inside /home/pete/amr_ws.

## Objective and diagnosed failure

Native42 physically completed A/B/home but strict B selection returns None at
gate6_evidence_analyzer.py:278 because two interrupted START marker blocks are
both eligible and share the same LOADED and first EMPTY boundary. Exact real
replay: public8797 rows, B marker indices5454/5474, terminal7395; A marker977 is
not B-eligible. Public source boot1446432738. E3 membership-report correction is
separate and does not change the production selector.

E2 actual B UUID e6780058a3977980ec137dbbb018da00 has executing/success evidence
and feedback102, but the recorded internal mass-stage stream begins at seq37
after both public starts. Temporal overlap cannot prove the missing ownership.
Confidence HIGH in selector mechanism; historical ownership gap remains UNKNOWN.
Native42 remains FAIL. Do not infer or retrofit missing ownership.

Evidence: B_RECOVERY_SNAPSHOT_20261004 finalized provenance/status/action exports;
B_STAGE_SELECTOR_REPLAY_20261005 actual unchanged-selector replay; Singer's
read-only contract and root source inspection. Installed Humble ServerGoalHandle
goal_id returns GoalInfo UUID without acquiring the goal state mutex; is_active
and status have locking implications. Do not add those getters under our lock.

Prediction: with explicit fresh ownership records, an interrupted START cycle
can select its earliest public START through first EMPTY while preserving every
public row and existing physical gate. Missing/contradictory ownership still
fails. Legacy selector behavior and old native42 rejection remain unchanged.

## Allowed paths and current worktree

Only these production/test files:

- src/amr_manipulation/scripts/cycle_manipulation_supervisor.py
- src/amr_manipulation/scripts/gate6_evidence_analyzer.py
- src/amr_manipulation/test/test_cycle_adapter.py
- src/amr_manipulation/test/test_gate6_completion_contract.py

These four were clean at release. Many other existing files are dirty; preserve
all of them. Recheck git status and read all four before edits. Evidence, command
receipts, original scoped source copies, owned HOME/tmp/cache/ROS logs and report
may be generated only in phase14_evidence/stability_20261003/G1_OWNERSHIP_20261005/.
Do not modify ledgers/handoff/AGENTS, protected AMR_CODEX_HANDOFF.md, other source,
installed files or global settings. Root owns coordination records. No simulation,
ROS node/probe, bag decode, planner replay, dependency install or push in this
packet. One production writer. Existing hypothesis limits do not reset.

## Preserved behavior

Preserve public messages/interfaces, status validity/detail/base permission,
stage markers and their forwarding handshake, ownership/admission/cancellation
and cleanup behavior, all existing thresholds and physical acceptance gates.
INTERNAL_STATUS_MAX_AGE_S stays0.2. Closure does not add a200ms terminal age gate:
existing owned terminal proof intentionally survives wrapper teardown.
No controller/path/placement edits or tuning. No legacy selector relaxation.

## Producer: explicit observation records

Use the existing /rosout channel and a versioned canonical JSON prefix
`AMR_CYCLE_OWNERSHIP_V1 `. No new topics or interface fields. Records must be
immutable primitives; never keep mutable ROS messages or reread ownership while
emitting. Use the expected runtime-pinned node/logger from factory_autonomous:
/amr/manipulation_supervisor_node. Confirm logger naming from installed APIs;
stop rather than guess a materially different contract. Consumer requires the
expected logger and exact fingerprint/public counterpart under the existing ROS
evidence trust model. No claim of DDS authentication or publication acknowledgement.

Create an observational per-execution epoch under the existing initialization
lock. UUID comes from goal_handle.goal_id.uuid; execution product comes from the
validated local product_id, not detached public product_id. Capture accepted raw
child status as immutable primitives, including header and every status field,
under the existing callback lock after its existing ownership/sequence filters.
Retain each accepted snapshot unchanged when later child data replaces it.

Track a bounded set of OBSERVED evidence violations under existing locks without
changing acceptance/rejection behavior: owned invalid/inconsistent/FAULT or wrong
product; owned sequence rollback/nonincreasing sequence; a foreign mass-stage
start/retention claim after ownership; contradictory stage ownership. Ignore benign
preparation as preparation, never promote it to stage proof. Mark relevant events
before their existing early returns. Reset observer only at new execution; seal
it at closure before ownership reset. Do not claim unobserved input history.

At _publish_status, construct finalized public fields as before. Under that SAME
lock, snapshot proof when the public row is the FIRST START marker of this
execution, first owned LOADED row, or first qualifying owned EMPTY row. Later
separated START blocks are certified by the same uninterrupted observer epoch
and sealed CLOSE, not additional per-block log records. A later row cannot
replace a missed earliest boundary. Four records suffice: START/LOADED/EMPTY/CLOSE.

Boundary proof fields must include:

- schema/version, event(kind START/LOADED/EMPTY), contiguous per-execution record
  index, execution UUID/product, public source boot and exact full public message
  (header stamp/frame, boot/sequence/valid/state/base/attachment/product/detail);
- complete accepted child snapshot, owned mass-stage boot and accepted sequence;
- captured monotonic time, original child receipt monotonic time and age,
  consistency, authority-open, stage-start/loaded/terminal flags, fault/cancel
  flags and observed-history violations.

Qualifying boundary requires same owned stage, valid consistent fresh child with
0<=age<=existing0.2 limit, no cancellation/fault/history contradiction. START is
valid STARTING/base forbidden/detached/empty product/exact marker; LOADED is
valid owned retained requested product; EMPTY is valid detached empty-stowed with
owned loaded-to-terminal proof. Keep physical/public state untouched.

Publish the public message outside the lock as before. Only after its publish
returns emit the frozen boundary payload outside the lock. Public receipt remains
mandatory; arrival order/proximity of log and public row is not identity proof.
Preserve exception behavior of public publication. Diagnostic serialization/logging
failure must not change returned action behavior or safety/state.

Bound this observation channel to64 captured records per execution and4096UTF-8
bytes per record (262144B maximum). These are evidence limits, not motion knobs.
Capture/reserve indices under the same epoch lock. Exhaustion/failed emission
causes missing/unsafe evidence, never silent success or safety changes. Keep an
epoch reference with delayed emitters so they cannot mutate a subsequent job.

Initialize actual returned result and observed child return code to unknown for
closure bookkeeping only. Preserve all existing returns. In finally preserve
existing fallback/child-alive/fault/authority handling. Under its existing lock,
AFTER closing child authority/handling child references but BEFORE reservation,
active owner, cancel flag or product clear, capture CLOSE for this callback's
goal_handle. Perform existing clearing in that SAME lock. Emit frozen CLOSE
afterward. No new safety permissions or cancellation fix.

CLOSE includes version/index/UUID/validated product/public boot; active-owner
identity match and reservation state; capture time/public-sequence allocation
high-water(marked not acknowledgement); latched cancel/fault and observed-history
flags; current state/detail/attachment; owned child boot/latest accepted raw
snapshot/sequence and started/loaded/terminal flags; child-exit/return-code and
reference consistency; actual returned result product/outcome/delivered; references
to exact first START/LOADED/EMPTY public identities and total captured record count.
Every START block is linked by the same epoch/index chain and public sequence
range, without an unbounded identity list. Actual terminal status is corroborated
from the recorded action stream; do not introduce a goal-mutex getter under our lock.

Qualifying close requires same active owner, child gone/no foreign retained child,
closed authority, no attachment, no accepted cancel or fault/history violation,
retained owned terminal proof and actual SUCCESS/delivered=true/product match.
Report original ages honestly, without refreshing proof or imposing0.2 at closure.
The cancellation claim ends when this lock clears its active owner. A delayed old
cancel during a subsequent reservation is an existing issue: its latched flag
must reject that NEXT epoch's augmentation; this packet does not repair it.

## Analyzer: narrow corroborated selection

Keep select_stage_status_stream(samples, product_id) unchanged. Preserve all seven
legacy fixtures and behavior, including incomplete old fixture fields. Only if
legacy selection returns None may a separate helper attempt corroboration.
Add optional collection of complete raw internal statuses, ExecuteProductCycle
status/feedback and timestamped structured rosout alongside current rosout strings.
Do not unconditionally add these to legacy REQUIRED_TOPIC_SUFFIXES. They are
mandatory for the corroborated path. Keep every message/receipt/no dedup.

Strictly parse expected-prefix records: exact schema, numeric/bool/string types,
valid full UUIDs, finite times, duplicate JSON key rejection, known version/event,
record bounds and expected logger. Reject malformed/duplicate/conflicting records
when augmentation is needed. No synthetic defaults, float-derived nanoseconds,
copied selector return or source monkeypatching. Preserve integer recorder ns for
proof/action/raw evidence; legacy receipt conversion remains unchanged.

Eligibility is ONLY multiple overlapping pre-LOADED START-block candidates in
one uniquely marked public boot that share the SAME first requested LOADED row
and SAME first valid EMPTY row. Missing terminal, two distinct completed jobs,
multiple marked boots or markers after LOADED remain failures. Within the union,
public sequences are positive/contiguous with no receipt/sequence rollback. Every
row between contributing START blocks is valid STARTING, base forbidden, detached,
empty product, detail exactly marker or existing stale/inconsistent diagnostic.
Keep earliest START, all public/stale rows and first terminal; no row dedup/filter.

Require exact proof/public fingerprints for the FIRST START row, first
LOADED, first EMPTY, and one qualifying CLOSE. UUID/product/public boot/ownedchild
boot match across all records; positive increasing child boundary sequences,
original freshness/consistency/ownership flags valid. Complete unique contiguous
record-index chain through CLOSE, correct first-boundary references, high-water
enclosing all required public sequences. Never use transient Python IDs as proof.

Require recorded ExecuteProductCycle executing and later successful terminal
for this UUID, with stable raw GoalInfo stamp, sufficient accepted/executing
identity evidence before START and success after EMPTY; full UUID-bound feedback
has requested product/EXECUTING and no CANCELING or contradictory product/ownership.
Reject canceled/aborted/other active execution contradictions. GoalStatusArray has
ONLY status_list, no message header/array timestamp; GoalInfo.stamp is acceptance
time and not recorder receipt. Do not invent product fields in status entries.

Check every recorded public/internal/action/proof negative guard in the proposed
span: invalid/FAULT, rollback, foreign stage/restart/retention, wrong product,
cancel/unsafe result, missing or conflicting boundary/closure. Observational
history witness strengthens these recorded guards; do not infer missing internal
records are clean. New explicit producer proofs bridge recorder gaps positively.

Return normal public boot/status slice to existing analyze gates. Every validity,
terminal/attachment/contact/command/pose/physical gate remains unchanged; failure
still uses the existing missing-or-ambiguous result. A successful legacy selection
does not suddenly require the new protocol. Old42 has no proof and must still fail.

## Verification and stop conditions

Before source change record actual baseline: E3 real B None/cardinality2 and old
helper tests. Add meaningful positive/negative behavior tests in the two allowed
test files; do not weaken existing tests. Required cases:

- old seven selector fixtures unchanged; proven START/stale/START retains earliest
  boundary/all stale rows/first EMPTY; missing each individual record/action/feedback;
- malformed/duplicate keys/version/types/UUIDs, conflicting/duplicate fingerprints,
  changed owner/child/product/boot, sequence/time rollback/noncontiguity;
- marker after LOADED, invalid/FAULT/arbitrary interruption, two completed jobs,
  missing terminal, history violation, stale boundary, live/foreign child at CLOSE;
- immutable snapshot through child refresh/goal reset/delayed log; publish failure
  emits no valid proof; log failure/cap exhaustion fails augmentation without
  altering public status/action/cleanup; reordered publication is not borrowed;
- late accepted cancellation after worker's cached canceled decision is captured
  before close reset and fails; next-reservation delayed cancellation remains
  latched for next epoch; unsafe result/owner mismatch/terminal loss fail;
- ordinary successful and canceled executions preserve existing results, permissions,
  handshake/detail semantics and cleanup. No new goal-status getter lock inversion.

Run focused checks under owned HOME/tmp/cache/ROS paths, PYTHONDONTWRITEBYTECODE=1,
ulimit -c0, ROS_DOMAIN_ID232, with canonical storage check/reserve before growth:
Source Humble/workspace setup WITHOUT nounset: do not use set -u before setup
because its AMENT_TRACE_SETUP_FILES check can fail before validation starts.

`python3 -B -m pytest -q src/amr_manipulation/test/test_cycle_adapter.py src/amr_manipulation/test/test_gate6_completion_contract.py`

Add syntax checks without writing __pycache__, inspect COMPLETE scoped diff,
report exact commands/exits/test scope/source hashes and bounded<=60line report.
No aggregate test total is simulation acceptance. Build/install parity and fresh
receipted43 readiness are later packets after completed source review.

G1-PROOF1 is new root-cause implementation attempt1/2. On any failed/contradicted
test, unsafe API/scope requirement, source mismatch or inconclusive discriminant,
STOP edits and return evidence to Sol. Do not fix/retry without re-diagnosis.
Any materially required path/contract change needs coordinator decision, not
silent scope expansion. Do not reset older exhausted hypotheses/counters.
Progress at least every2min; report changed files/results/risks before HOLD.
