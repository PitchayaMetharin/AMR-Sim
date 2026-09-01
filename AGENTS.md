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

**Sol/high** = read-only analysis, diagnosis, debugging, evidence review, planning.

**Luna/max** = approved implementation and focused validation.

Default workflow:

`Sol diagnosis -> Luna implementation -> Sol review`

One writer at a time. Luna must not improvise a different fix if evidence contradicts Sol's diagnosis; stop and return to Sol.

## Diagnosis before editing

Before Luna changes source, Sol must establish:

* observed vs expected behavior
* concrete failure mechanism
* supporting evidence
* exact affected files
* preserved invariants and non-goals
* a falsifiable prediction
* focused validation commands

If the cause is `UNKNOWN`, gather evidence instead of editing code.

## Debug-loop breaker

Every debugging iteration must produce new information by confirming, falsifying, or narrowing a hypothesis.

Do not repeat essentially the same patch or retry without new evidence.
For repeated, unresolved, integration, or timing-sensitive failures, follow DEBUG_PLAYBOOK.md before further source edits.
Maximum:

* **2 implementation attempts per root-cause hypothesis**
* **3 rejected root-cause hypotheses for the same blocker**

After either limit is reached, stop autonomous patching and report the evidence, rejected hypotheses, strongest remaining explanation, and uncertainty to the user.

A failed implementation must return to Sol for re-diagnosis before another source change.

## Runtime validation

Runtime evidence is separate from source implementation.

* Run focused source validation first.
* Runtime runs require authorization when specified by the handoff.
* Preserve evidence and stop at the first failed mandatory gate.
* Do not patch immediately after a runtime failure; return the evidence to Sol first.
* Do not treat build/unit success as runtime proof.

Timing changes require timing evidence. Prefer waiting on observable state over arbitrary sleeps.

## Implementation discipline

Luna must:

1. recheck `git status --short`
2. re-read the target files
3. make the smallest coherent change
4. inspect the complete diff
5. run focused checks
6. report commands, results, changed files, and remaining risks

Avoid speculative refactors, unrelated cleanup, and future-phase work.

## Handoffs

Every Sol -> Luna handoff should state:

* objective
* diagnosis and confidence
* evidence
* allowed files
* invariants/non-goals
* prediction
* validation commands
* current worktree state
* stop conditions

For detailed difficult-debugging procedure, follow `DEBUG_PLAYBOOK.md`.
For repeated, unresolved, integration, or timing-sensitive failures, follow DEBUG_PLAYBOOK.md before further source edits.
