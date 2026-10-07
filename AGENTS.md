# AMR Workspace Agent Guide

> Diagnose from evidence before editing, make the smallest justified change,
> and do not claim success until the required behavior has been directly verified.

Durable engineering rules only. Current phase, active blocker, temporary
authorization, agent/model assignments, counters, and resume state belong in
`SESSION_HANDOFF.md`. Detailed debugging procedure belongs in `.codex/DEBUG_PLAYBOOK.md`.

## 1. Authority

  Follow instructions in this order. Within the same authority level, newer explicit instructions supersede conflicting older ones:

1. Current explicit user instruction
2. `SESSION_HANDOFF.md`
3. `AGENTS.md`
4. Task plans and project documentation
5. Historical records and ledgers (evidence, not active policy)

Work only inside the approved task, phase, and paths.

## 2. Protect the workspace

Workspace root: `$AMR_WS`, defined once in section 4. Derive workspace paths from
it; do not repeatedly transcribe absolute workspace paths across commands, scripts,
or packets. Keep task-created logs, caches, temporary files, and evidence inside it
unless explicitly instructed otherwise.

Before editing, run `git status --short`. Preserve unrelated user work.

Never modify, stage, discard, normalize, or commit `AMR_CODEX_HANDOFF.md`
without explicit user direction.

Do not, without approval: push; rewrite Git history; discard unrelated changes;
install dependencies; change system/global configuration; modify resources outside
the approved workspace; perform unrelated refactors or cleanup.

## 3. Invariants and ROS 2 domain

Preserve unless explicitly changed: fail-closed behavior, safety gates,
ownership boundaries, public interfaces, validated thresholds, hardware values,
collision behavior, freshness requirements, terminal-state guarantees.

Never weaken a test, gate, threshold, or acceptance rule merely to obtain a pass.

Every command, test, launch, and runtime instruction must satisfy
`0 <= ROS_DOMAIN_ID <= 232`. Never use `233`–`255`.

## 4. Baseline validation commands

Task-specific handoffs may define narrower or additional validation commands.

```bash
export AMR_WS=/home/pete/amr_ws
cd "$AMR_WS"
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=232


colcon build --packages-select <pkg>
colcon test --packages-select <pkg>
colcon test-result --test-result-base "$AMR_WS/build/<pkg>" --verbose
```

## 5. Diagnose before editing

Do not edit production source from a plausible guess alone. Before implementation, establish:

- observed and expected behavior
- concrete failure mechanism and supporting evidence
- affected files and the relevant control flow/state transition
- preserved invariants and non-goals
- a falsifiable prediction
- focused validation

If the cause is still `UNKNOWN`, gather evidence instead of editing.
Inspect actual current source, not only summaries or earlier assumptions.
Reuse verified findings while the relevant source and contracts remain unchanged.

## 6. Debugging discipline

Every iteration has one hypothesis, one discriminating check, expected evidence,
and a stop condition. Each iteration must confirm, falsify, or narrow the hypothesis.

- Do not repeat essentially the same patch, command, or simulation without new evidence.
- A failed implementation returns to diagnosis before another production edit.
- A test that passes on the known-broken baseline is non-diagnostic.
  Do not weaken the requirement to make it pass.
- For repeated, timing-sensitive, concurrency, or integration failures, follow
  `.codex/DEBUG_PLAYBOOK.md`.

## 7. Implementation discipline

The implementer must:

1. run `git status --short`
2. reread the target files
3. confirm scope
4. make the smallest coherent change
5. inspect the complete scoped diff
6. run checks proportionate to the change (section 4 for source changes; for config, constants, or docs, diff inspection plus a `grep` for stale values is enough unless the handoff requires a build)
7. report the result

Construct patches from the exact current source; if patch context is stale, reread the file.
Correct routine typos or formatting slips within the existing packet or file in
place; do not open a new packet or restart the cycle for them.
Avoid speculative refactors, unrelated cleanup, future-phase work, and silent scope expansion.
Do not run builds, tests, or simulations for a change fully checkable by diff or grep unless the user or handoff asks.

## 8. Runtime validation

Source validation and runtime validation are different evidence. A successful build
or unit test does not prove simulation or system behavior; a successful runtime run
does not prove unexercised paths.

```text
diagnose -> bounded implementation -> focused source checks
         -> review -> runtime validation -> accept, or back to diagnosis
```

After a runtime failure, return to diagnosis before another source edit.
Record the exact command, relevant environment, exit status, exercised acceptance
gates, and retained evidence. Stop at the first failed mandatory gate unless the
active diagnostic plan explicitly requires additional safe observation.

## 9. Timing and stateful behavior

Prefer observable state over arbitrary sleeps. Record relevant timestamps, state
transitions, ownership, cancellation, terminal proof, and freshness. Do not infer
causality only because two events occurred close together. For injected defects,
prove the bad value reaches the intended consumer boundary before judging the gate.

## 10. Tests and review

Add tests only for written requirements, demonstrated regressions, demonstrated
coverage gaps, or otherwise-unverified contracts. Do not add tests to raise the count.
Test totals are scope information, not proof of simulation, integration, hardware,
or timing acceptance.

When the current milestone requires independent review, a non-author performs it.
Do not claim author verification as independent review.

## 11. Agent coordination

Current model and agent assignments live in `SESSION_HANDOFF.md`.

- One production-code writer at a time; parallel workers get disjoint scope.
- Parallel agents may do independent inspection, validation, monitoring, or review
  when scopes do not conflict.
- Do not silently substitute an unavailable assigned worker.
- Do not reset evidence or debugging history when changing workers/models.
- Do not run duplicate simulations without a diagnostic reason.

## 12. Stop conditions

Stop the affected work when: a mandatory gate fails; scope is ambiguous; a
material contract cannot be resolved; an authoritative process/worker is lost;
or the user says stop (takes effect immediately).

Stop production editing and return to diagnosis when evidence contradicts the
hypothesis or remains insufficient to justify the next edit. Further diagnostic
checks are allowed.

Report the observed failure, relevant evidence, changed files, remaining
uncertainty, and safe resume point. Do not keep editing to avoid reporting a blocker.

## 13. Handoffs and reports

**Diagnosis → implementation handoff** must include: objective; diagnosis and
confidence; supporting evidence; allowed files; required changes; invariants;
non-goals; falsifiable prediction; exact validation commands; expected behavioral
evidence; current worktree state; stop conditions; reporting requirements.
If a material ambiguity remains, the implementer reports it rather than inventing behavior.

**Implementation report:** changed files, actual changes, commands executed, exit
statuses, observed results, evidence paths, blockers, remaining risk. Prefer concise
evidence and bounded packets over full histories or repeated findings.
Do not repeat unchanged completion reports unless the user asks or new evidence changes the result.

## 14. File responsibilities

| File | Responsibility |
|---|---|
| `AGENTS.md` | Durable engineering rules |
| `SESSION_HANDOFF.md` | Current phase, blocker, authorization, assignments, counters, runtime state, resume point |
| `.codex/DEBUG_PLAYBOOK.md` | Detailed difficult-debugging procedure |
| `MODEL_IMPLEMENTATION_LEDGER.md` | Historical established implementation mistakes |
| `LUNA_IMPLEMENTATION_LEDGER.md` | Luna-specific historical mistakes |
| `ROS2_SKILL_FEEDBACK.md` | Evidence-backed ROS 2 skill issues |
| `AMR_CODEX_HANDOFF.md` | Protected artifact; modify only by explicit user request |

Keep one authoritative location for each type of information.
Do not preserve superseded policy in this file.
