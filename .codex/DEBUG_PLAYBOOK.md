# Debug Playbook

Use this playbook when a failure is unresolved, repeated, integration-related, timing-sensitive, or has already survived one implementation attempt.

Normal straightforward fixes do not need this entire procedure.

## 0. Current Luna/max implementation and Sol/high supervision

AGENTS.md's current 2026-10-04 role assignment is authoritative: exact
`gpt-6-luna` / `max` alone implements; exact `gpt-6.1-sol` / `high` thinks,
plans, analyzes, orchestrates, debugs and supervises. Sol never writes
implementation code without manual explicit user authorization. Sol supplies
the detailed packet; Luna may implement assigned test/probe code, run checks
and monitor owned processes, then report actual changes, commands and evidence.
Batch Sol's check after completed implementation and before simulation; worker
self-checks, immediate failure stops and the runtime-specific review exception
remain binding. Verify actual session model/effort before release; no model
substitution or global settings change is authorized. Failed work returns to
Sol's diagnosis; worker changes reset no counters or scope. Use AGENTS.md's
Implementation discipline to attribute mistakes: plan deviations can count
against Luna; faithful implementation of a failed plan does not. Account for
established mistakes in MODEL_IMPLEMENTATION_LEDGER.md and also record actual
Luna mistakes in LUNA_IMPLEMENTATION_LEDGER.md; preserve historical entries.
All diagnosis, evidence, stop and counter requirements below remain binding.

Historical 2026-10-02 task-fit model choices below are superseded by the
current role assignment; shared evidence, scope and stop rules remain binding:

* Small mechanical lower-risk work: prefer `gpt-5.6-luna` / `xhigh`; `/ max`
  only when extra effort is justified by the task.
* Core ROS2, lifecycle/concurrency/safety-sensitive work, hard verification or
  demonstrated Luna mistakes: select `gpt-6.1-sol` / `medium` directly with
  recorded task-fit evidence. No mandatory Luna trial or serial model ladder.
* `gpt-6.1-sol` / `high` is the orchestrator and diagnoses a contradiction or
  error in Sol/medium analysis. Record the failed prediction/evidence first;
  routine orchestration is not a mandate for high-effort diagnosis of every task.
* No Astra or another model generation/family without user direction. Verify
  exact model/effort; if unavailable, report rather than substitute.
* Scoped delegation is authorized for the current stability task, one production
  writer at a time. Do not change global settings or expand phase authority.
* Role references elsewhere in this playbook follow the current assignment:
  Sol is the analyzer/supervisor; Luna is the sole implementation worker.
  Historical permission for a Sol implementation worker is superseded.
* Retain all evidence requirements, attempt/hypothesis counters and independent
  review. A model change resets no counters, and author verification is never
  independent review. Availability errors alone are not implementation mistakes.

## 1. Classify the failure

Before changing source, classify the blocker as:

* `SOURCE`
* `CONFIGURATION`
* `ORCHESTRATION`
* `ENVIRONMENT`
* `TEST/EVIDENCE HARNESS`
* `EXPECTED GATE FAILURE`
* `UNKNOWN`

If classification is `UNKNOWN`, gather evidence. Do not edit source yet.

Do not fix environment or orchestration failures by changing product behavior unless evidence demonstrates that the product is responsible.

## 2. Establish the failure

Record:

* exact failing command or gate
* observed result
* expected result
* relevant timestamps or measurements
* relevant logs/evidence paths
* current `git status --short`

Prefer preserved evidence over memory or summaries.

Do not rerun an expensive integration test merely to reproduce already adequate evidence.

## 3. Build hypotheses

The diagnosing agent selected by the current elevation rule owns diagnosis.

For each plausible cause, classify it as:

* `SUPPORTED`
* `WEAK`
* `UNTESTED`
* `FALSIFIED`

Prefer tests that distinguish competing hypotheses before modifying code.

Do not revive a falsified hypothesis unless new evidence contradicts the earlier falsification.

## 4. Require a causal mechanism

Before implementation source edits, the assigned diagnosing agent must explain:

`observed state -> code/runtime mechanism -> failure`

The diagnosis should identify the relevant files, functions, nodes, callbacks, states, topics, actions, services, or timing relationship.

Correlation alone is not enough.

## 5. Make a prediction

Every proposed fix must include a falsifiable prediction.

Example:

> If the failure is caused by the forbidden-motion state being published before base velocity settles, then waiting for existing stationary odometry before that publication should move the state transition after measured velocity reaches the stationary threshold while leaving the threshold itself unchanged.

The prediction must describe what evidence should change if the diagnosis is correct.

## 6. Evidence-backed implementation packet

The assigned diagnosing agent provides the implementer:

### Objective

Exact blocker being fixed.

### Diagnosis

Root cause and confidence: `HIGH`, `MEDIUM`, or `LOW`.

### Evidence

Relevant commands, values, logs, timestamps, and paths.

### Allowed files

Exact files Luna may modify.

### Invariants

Behavior/interfaces that must remain unchanged.

### Non-goals

Adjacent changes explicitly excluded.

### Prediction

Expected observable result.

### Focused validation

Commands that test the proposed mechanism.

### Stop conditions

Conditions requiring Luna to stop and return to Sol.

Luna should not implement a `LOW` confidence diagnosis unless explicitly authorized.

## 7. Luna implementation rules

Before editing:

1. run `git status --short`
2. inspect relevant target files
3. identify pre-existing dirty changes
4. confirm allowed paths

Then:

* make the smallest coherent change
* avoid unrelated cleanup/refactoring
* preserve public interfaces and acceptance criteria
* add focused regression coverage when practical
* inspect the complete diff
* run focused validation first

Apply AGENTS.md's Implementation discipline before assigning blame. Compare
the scoped diff, commands and reports with the approved packet; failed tests
or runs alone do not establish a Luna mistake. Faithful execution of a failed
plan returns to Sol's diagnosis without incrementing Luna's count. For an
established Luna deviation, update both `MODEL_IMPLEMENTATION_LEDGER.md` and
`LUNA_IMPLEMENTATION_LEDGER.md` before closing or handing off, preserving the
packet requirement, observed deviation, exact failure, impact and evidence.

Keep each new ledger entry at most six lines total, combining metadata where
needed while retaining exact failure, cause, evidence and safe resume point.
Do not rewrite historical entries merely to impose the new format.

Do not alter thresholds, safety gates, tests, analyzers, or expected values just to produce a pass.

## 8. Contradicting evidence

If Luna discovers evidence that contradicts Sol's causal model:

STOP.

Do not improvise another source fix.

Return to Sol with:

* what was expected
* what actually happened
* new evidence
* changed files
* current worktree state

Sol must re-diagnose before another implementation attempt.

## 9. Debug-loop breaker

Every debugging iteration must increase information.

A useful iteration must do at least one of:

* confirm a causal relationship
* falsify a hypothesis
* isolate the fault further
* produce a discriminating measurement
* fix the failure

The following do not count as progress by themselves:

* rerunning unchanged tests
* adding arbitrary sleeps
* increasing timeouts without evidence
* widening tolerances
* trying another plausible patch without new evidence
* restarting repeatedly hoping for a different outcome

### Retry policy — user override 2026-10-06

There is no numeric cap on implementation attempts under one hypothesis or
rejected hypotheses for a blocker. Preserve historical counters and evidence.
Count-based exhausted allowances and historical PENDING exception notes do not
require another approval. Every failure still returns to Sol for diagnosis
before another source edit, and every retry must produce new information.
Use the same hypothesis identity when the mechanism is substantially unchanged.
Runtime/action budgets, bounded repair counts, safety gates and failure stops
remain binding; this override changes engineering retry policy only.

## 10. Failed-attempt review

After a failed implementation, Sol must compare:

### Prediction

What the previous diagnosis predicted.

### Result

What actually changed.

### Interpretation

Choose one:

* diagnosis supported, implementation incorrect
* diagnosis partially supported
* diagnosis falsified
* evidence insufficient

### New information

What was learned that was not known before.

No further patch is allowed until this comparison exists.

## 11. Timing-related fixes

Before adding or increasing:

* sleeps
* settle durations
* startup delays
* retries
* watchdogs
* freshness windows
* debounce periods

provide timing evidence showing why the change is required.

Prefer waiting on an existing observable state, acknowledgement, or measured condition over arbitrary elapsed time.

Do not stack another delay on top of a failed timing fix without re-diagnosis.

## 12. Runtime evidence

Runtime validation is separate from source validation.

Before runtime:

* focused source validation passes
* required authorization exists
* expected process state is known
* stale processes are checked when relevant
* evidence destination is declared

During runtime:

* execute only the authorized gate/run
* preserve raw evidence
* stop at the first failed mandatory boundary
* do not edit source while the run is active

After failure:

1. preserve evidence
2. perform documented shutdown/cleanup
3. return evidence to Sol
4. perform fresh read-only diagnosis
5. only then consider another implementation

A successful build or unit test is not runtime proof.

## 13. Retry discipline

A retry without any source, configuration, or environment change is allowed only when:

* the behavior is known to be nondeterministic, or
* the retry tests a stated hypothesis.

Before retrying, state what possible outcome will distinguish competing explanations.

If no outcome is discriminating, gather better evidence instead.

## 14. Test integrity

Treat tests and evidence analyzers as part of the specification.

If implementation and a test disagree, determine which behavior matches the documented requirement.

Do not assume the implementation is wrong.

Do not assume the test is wrong.

Do not modify a test simply because it blocks progress.

## 15. Escalation packet

When a material blocker cannot be resolved from available evidence, stop editing and report:

### Original failure

Exact blocker.

### Evidence

Most important measurements/logs.

### Attempts

Changes already tried and their outcomes.

### Falsified hypotheses

Root causes now ruled out.

### Strongest remaining explanation

Current best theory and confidence.

### Remaining uncertainty

What is still unknown.

### Worktree

Current `git status --short` and modified files.

### Required next decision

What evidence, permission, environment access, or architectural decision is needed from the user.

Do not continue autonomous patching after escalation.
