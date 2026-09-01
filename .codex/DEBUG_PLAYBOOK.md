# Debug Playbook

Use this playbook when a failure is unresolved, repeated, integration-related, timing-sensitive, or has already survived one implementation attempt.

Normal straightforward fixes do not need this entire procedure.

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

Sol/high owns diagnosis.

For each plausible cause, classify it as:

* `SUPPORTED`
* `WEAK`
* `UNTESTED`
* `FALSIFIED`

Prefer tests that distinguish competing hypotheses before modifying code.

Do not revive a falsified hypothesis unless new evidence contradicts the earlier falsification.

## 4. Require a causal mechanism

Before Luna/max edits source, Sol must explain:

`observed state -> code/runtime mechanism -> failure`

The diagnosis should identify the relevant files, functions, nodes, callbacks, states, topics, actions, services, or timing relationship.

Correlation alone is not enough.

## 5. Make a prediction

Every proposed fix must include a falsifiable prediction.

Example:

> If the failure is caused by the forbidden-motion state being published before base velocity settles, then waiting for existing stationary odometry before that publication should move the state transition after measured velocity reaches the stationary threshold while leaving the threshold itself unchanged.

The prediction must describe what evidence should change if the diagnosis is correct.

## 6. Sol -> Luna implementation packet

Sol provides Luna:

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

### Limits

Maximum **2 implementation attempts under the same root-cause hypothesis**.

After two failures, Sol must revise or falsify the causal model before another source edit.

Maximum **3 rejected root-cause hypotheses for the same blocker**.

After three rejected hypotheses, stop autonomous debugging and escalate to the user.

Do not evade these limits by renaming substantially identical hypotheses.

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

When the debug-loop limit is reached, stop editing and report:

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
