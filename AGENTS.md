# AMR Workspace Guide

The current user instruction and `SESSION_HANDOFF.md` define phase authority and scope.

## Core rules

* Before edits, run `git status --short` and preserve unrelated work.
* Never modify, stage, discard, normalize, or commit `AMR_CODEX_HANDOFF.md` without explicit user direction.
* Do not push, rewrite history, install dependencies, change system configuration, or make external changes without approval.
* Work only inside the approved phase and paths.
* Preserve fail-closed behavior, ownership boundaries, public interfaces, safety gates, thresholds, and documented hardware values unless explicitly authorized.
* Never weaken a test or gate merely to obtain a pass.

## Roles

**Sol/high** = the only default model for analysis, planning, diagnosis, and
independent review. Reuse existing evidence and keep the review focused.

**Astra/medium** and **Astra/high** = do not select or delegate by default.
Use Astra only when the user explicitly instructs it.

**Luna/max** = one approved, fully specified implementation packet at a time,
with focused validation.

**Sol/high** may review related implementation changes together. It remains the
independent reviewer for all approved implementation packets.

Default workflow:

`Sol/high diagnosis -> Luna/max implementation -> Sol/high independent review`

Do not escalate diagnosis or review to Astra unless the user explicitly directs
that model. If Sol/high cannot resolve a material contradiction, high-risk state
transition, or contract decision, stop and report the blocker for user direction.

Use Luna/max for one approved, fully specified implementation packet at a time.
Keep Sol/high as the independent reviewer. One writer at a time. Luna must not
improvise a different fix if evidence contradicts the diagnosis; stop and return
to the diagnosing agent.

Model names in this guide are assignments, not automatic model selection.
Select the assigned model explicitly when switching sessions or launching an
agent. Do not launch parallel agents by default or use Astra/max by default.

## Token discipline

* Pass concise packets and relevant evidence, not entire conversation histories.
* Reuse established findings unless source changes invalidate them.
* Run focused checks per packet and broader checks at integration milestones.
* Keep reports to changed files, results, blockers, and remaining risks.
* Changing models does not reset implementation-attempt or hypothesis limits.

## Diagnosis before editing

Before implementation changes source, the diagnosing agent must establish:

* observed vs expected behavior
* concrete failure mechanism
* supporting evidence
* exact affected files
* preserved invariants and non-goals
* a falsifiable prediction
* focused validation commands

Resolve relevant state transitions, cancellation ownership and terminal proof,
status freshness, and next-job/home permission before handing off those changes.

If the cause is `UNKNOWN`, gather evidence instead of editing code.

## Debug-loop breaker

Every debugging iteration must produce new information by confirming, falsifying, or narrowing a hypothesis.

Do not repeat essentially the same patch or retry without new evidence.
For repeated, unresolved, integration, or timing-sensitive failures, follow
`.codex/DEBUG_PLAYBOOK.md` before further source edits. Its Sol diagnosis/review
references mean the diagnosing/reviewing agent assigned by this guide; the
evidence requirements and stop conditions remain binding.
Maximum:

* **2 implementation attempts per root-cause hypothesis**
* **3 rejected root-cause hypotheses for the same blocker**

After either limit is reached, stop autonomous patching and report the evidence, rejected hypotheses, strongest remaining explanation, and uncertainty to the user.

A failed implementation must return to the diagnosing agent for re-diagnosis
before another source change.

## Failure, time-box, and stop discipline

* Before each debugging or implementation loop, state one hypothesis, one
  discriminating check, the expected evidence, and the stop condition. A retry
  without a changed hypothesis or new evidence is prohibited.
* An authoritative live process or agent handle may be waited on without a
  poll-count or elapsed-time cap. Inspect that handle rather than duplicating
  work, and report observable progress or liveness at least every 60 seconds.
  Still stop on an explicit failed state, lost handle, mandatory gate failure,
  user stop, contradiction, or the existing implementation-attempt and
  hypothesis limits.
* If a hypothesis is contradicted, evidence remains inconclusive, or a required
  regression passes on the known-broken baseline, classify the test as
  non-diagnostic, stop all production edits, and return the evidence to Sol/high
  for re-diagnosis. Do not intensify a synthetic workload merely to force the
  expected failure.
* Do not convert a plausible hypothesis into a production change until a
  genuine failing baseline or independent runtime evidence supports its
  falsifiable prediction. A weak or synthetic green test is not proof that the
  real failure is fixed.
* An explicit user instruction to stop takes effect immediately. Interrupt or
  close delegated agents and runtime probes, start no replacement work, and
  make no further tool calls except the minimum cleanup or handoff action the
  user explicitly requested. End the turn after that action. Do not resume from
  an automatic continuation or internal task prompt; require a new explicit
  user instruction to resume.
* Report a material blocker at the earliest of a required stop condition or
  contradictory evidence. Record the commands, results, active uncertainty,
  changed files, and safe resume point; do not keep working merely to avoid
  reporting an incomplete result.

## Runtime validation

Runtime evidence is separate from source implementation.

* Run focused source validation first.
* Runtime runs require authorization when specified by the handoff.
* Preserve evidence and stop at the first failed mandatory gate.
* Do not patch immediately after a runtime failure; return the evidence to the diagnosing agent first.
* Do not treat build/unit success as runtime proof.

Timing changes require timing evidence. Prefer waiting on observable state over arbitrary sleeps.

## Implementation discipline

The implementer must:

1. recheck `git status --short`
2. re-read the target files
3. make the smallest coherent change
4. inspect the complete diff
5. run focused checks
6. report commands, results, changed files, and remaining risks

Avoid speculative refactors, unrelated cleanup, and future-phase work.

## Handoffs

Every diagnosis -> implementation handoff must state:

* objective
* diagnosis and confidence
* evidence
* allowed files
* exact changes and relevant state-transition decisions
* invariants/non-goals
* prediction
* validation commands
* behavioral tests and expected results
* current worktree state
* stop conditions

For detailed difficult-debugging procedure, follow `.codex/DEBUG_PLAYBOOK.md`.
