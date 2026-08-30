# Project Status

## Active phase

Phase 14 factory mobile manipulation is explicitly authorized and is being
implemented gate-by-gate. Gates 1 through 5 passed. Gate 6 is in progress; no
later gate is authorized to run until Gate 6 passes.

## Gate 6 status

- Composite and empty arm motion passed live.
- The 1 kg stage now positively detaches all three Gazebo startup joints before
  motion, completes the collision-checked staging retreat, reaches the exact
  seeded-IK pre-grasp, and completes the Cartesian grasp approach with zero
  measured base or product displacement.
- The last completed live run failed closed before attachment because a
  `0.0275 m` finger target only touched the `0.100 m` handle tangentially and
  produced no bilateral contact evidence.
- The close target is now `0.020 m`, retaining product-specific bilateral
  contact freshness, pose-tolerance, and Gazebo attachment-confirmation gates.
  The package builds and its 11 tests pass, but this final contact change has
  not been exercised live.
- The current executable covers grasp and loaded stow only. The required 1 kg
  grasp/place stage is not complete until transport/place behavior and its
  acceptance evidence are implemented and pass.

## Validation and stop reason

The latest focused `amr_manipulation` result is 11 tests, zero errors, zero
failures, zero skips. No complete-workspace Phase 14 validation has run yet.
Work stopped when Codex reported the usage limit while starting the fresh live
contact retry; the reported reset is August 20, 2026 at 11:16 AM.

No dependency or external asset was installed. No commit or push was made.
`AMR_CODEX_HANDOFF.md` remains user-owned and was not modified by this Phase 14
work.
