# AMR Workspace Guide

Work deliberately and keep process proportional to risk. The current user
instruction and `SESSION_HANDOFF.md` phase authority govern this workspace.

## Non-negotiable rules

- Before edits, run `git status --short`; preserve unrelated work.
- Never modify, stage, discard, normalize, or commit `AMR_CODEX_HANDOFF.md`
  without explicit user direction.
- Do not push, force-push, rewrite history, or make external changes without
  explicit approval.
- Work only in an explicitly approved phase. Do not create future-phase
  artifacts while waiting for authorization.
- Preserve the fail-closed motion path and declared ownership boundaries. Do
  not invent, substitute, or silently promote hardware values or safety claims.

## Efficient operating defaults

- Read the current handoff and only files directly relevant to the task; do not
  reload prior phases unless a needed detail is absent.
- Make the smallest coherent change. Avoid speculative abstractions, duplicate
  documentation, and unrelated cleanup.
- Use concise progress updates and final reports. State only decisions,
  changes, validation, risks, and blockers relevant to the task.
- Match validation to risk: run focused checks for focused edits and broader
  builds/tests only when interfaces, behavior, or integration can be affected.
- Use subagents only for independent, bounded work that materially reduces
  elapsed time; do not delegate simple inspection or routine edits.

## Implementation and phase completion

- Preserve public interfaces, fail-closed behavior, and time/authority rules
  unless the approved phase explicitly changes them.
- Record behavior-affecting assumptions in the relevant artifact; do not add
  commentary that repeats established project context.
- At an approved phase end, follow the handoff's required documentation,
  validation, approval, and commit sequence. This file intentionally does not
  duplicate those detailed phase instructions.

## Reusable diagnosis-to-implementation workflow

- Always use **Sol/high** for read-only planning, debugging, analysis,
  diagnosis, and evidence, and **Luna/max** for approved implementation and
  focused verification. A separate runtime-evidence pass is optional and only
  runs when explicitly authorized after source validation.
- Sol/high must trace the reported behavior to a concrete mechanism, record
  the exact affected paths and invariants, and hand off a concise analyst
  packet containing evidence, non-goals, validation commands, and blockers.
- Luna/max starts only from that packet plus an approved plan, rechecks
  `git status --short`, makes the smallest scoped edit, inspects the combined
  diff, and runs focused checks before any broader or runtime validation.
- Treat this as a serialized shared workspace: one writer at a time, no
  concurrent edits to the same file or interface, and no reset, checkout,
  normalization, staging, commit, push, or deletion of another agent's work.
  Every handoff must identify the current worktree state; the receiving role
  must re-read the relevant files and status before writing.
- Escalate and stop when phase authority, target ownership, existing dirty
  state, safety boundaries, or required test baselines are ambiguous; when a
  change would exceed the approved paths; or when validation needs new
  privileges, dependencies, downloads, live-system access, or a workaround.
  Do not silently substitute tuning, weaken a gate, or reclassify a failure.
- A context handoff must state the objective, diagnosis, exact allowed files,
  preserved invariants and non-goals, commands and results, and unresolved
  risks. Runtime evidence must remain a separately bounded activity and must
  stop at the first failed gate.
- Do not create a custom plugin, call an external API, install a dependency,
  or download an asset by default. Use repository-local tools and existing
  interfaces unless the user explicitly approves the expansion.
