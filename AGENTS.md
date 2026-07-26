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
