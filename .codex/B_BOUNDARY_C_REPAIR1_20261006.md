# C attempt1: two test-setup corrections, same seven cases

Sol/high directly inspected actual source and retained ctest_boundary.stdout.
Source and header remain pinned2e9b1c37c783a10e01ab152de2d7ad6b39db67fb1c511c4081b3cc0021894650
and f1820271028be1ee83e21dc804b017e595abbb858a85b7a03fb333da5848a385.
Test before repair7a6689e46c04156b03bbab7c5a2ac36a9a62a450f002353385ec482c0120dde9.
Build passed; actual seven-case run5pass/2fail, exit8,15.38s. Original C scope
and checks are unchanged; no new cases, production/config/header/CMake edits.

## Diagnosis, exact changes and prediction

1. Residual-zero setup uses nonzero(.060,.033) bias. simulate_goal_locked computes
   physical=(localized target+bias); subtract/add roundoff leaves X8.88178e-16
   from stance and breaks demanded exact measured zero. The helper/admission
   outcomes at.009/.010/.0100001 already match spec; keep those exact assertions.
   In BoundaryPhysicalDockResiduals ONLY, use coherent zero bias: physical and
   localized starting poses both(-2.5,0,0). Keep its existing allowed yaw0/zeroY
   geometry and all other fixture defaults. Update dock-target expectations to
   stanceX/stanceY and the long-goal reference to the new localized(-2.5,0).
   Keep normal simulated goal then requested physical-Y offset, exact measured
   residual equality, exact admission/goal-count proofs and all four residuals.
   Prediction: target/bias reconstruction is exact copying with bias0, physical
   X matches the same stance double, so hypot(0,requestedY)==requestedY.
   Do not add epsilon or relax equality/thresholds. Numeric setup attribution is
   unestablished; retain failed attempt independently of blame.
2. TF loop changes the flag to0 without clearing a previously future-stamped
   buffer entry. Root read the existing method: it holds pose mutex, sets mode,
   clears buffer and updates evidence coherently. Replace direct flag reset by
   set_tf_defect_and_reseed(0) AFTER set_poses_and_reseed in each iteration,
   before fresh baseline assertions. Then inject each defect with its same
   existing setter. Prediction: fresh baseline is valid for all six iterations,
   each actual defect rejects current TF and causes zero navigation. Keep all
   original stamp/norm/nonfinite and prerequisite assertions. M205 accounts
   explicit missing reset; all205/Luna107, no counters reset.

## Release, validation and stops

Exact verified gpt-6-luna/max is sole writer of the existing behavior test.
Recheck git status/source pins and reread the two target blocks before edits.
Inspect complete delta against attempt1; only the two changes above are allowed.
Retain failed resume2 receipts/XML untouched; new evidence exclusively
packet_b/boundary_c_resume3 under the same workspace B evidence root.
Use original C G1 owned environment, domain232/jobs2/core0/no-bytecode.
Fresh canonical command before growth:
`PYTHONPATH=phase14_evidence/factory_runtime_tools python3 -B -c 'import storage_budget; print(storage_budget.check(reserve=30000000))'`
Execute original C commands unchanged: scoped manipulation build; seven-case
Boundary filter; centered+admission filter; unfiltered full CTest with inherited
filter/shard/repeat unset and TIMEOUT90 unchanged; affected Python checks; scoped
diff check. Preserve actual named-case/count logs/XML before overwrite.
Stop first actual failed check, source contradiction, scope ambiguity, model
mismatch/unavailability, lost handle or user stop; return to Sol before editing.
Expected all seven then broader checks pass. No simulation, replay or decoder.
Report actual diff, commands/exits, live handles, evidence and uncertainty;
progress<=2min. C attempt1 remains failed; repair implementation attempt2.
Root completed-batch review follows checks, before any fresh simulation.
