Status: spike measured 2026-11-13 on branch `spike/soft-singular-step`, built on `wip/closing-rate-restore`. Not merged, not reviewed, default off. No decision taken; the owner reviews this.

# Soft springs at the singular step only: spike results (2026-11-13)

The first soft-spring spike (`spike/soft-springs`, `2026-11-12-soft-spring-spike.md` on that branch) put the
springs on every iteration and failed: the huge loose motion fed the corner rules. The owner asked
(2026-11-13) what happens if the springs run only when the matrix is singular. This is that spike. Release
build; every number was measured on this branch.

## 1. Summary

- **The arch gets the same answer about 4.5 times faster.** A1.4: 0.9 s against 4.2 s with the dense step.
  Every corner state, verdict and released body is the same, pass by pass.
- **All 18 scenario files give byte-identical CLI output.**
- **The shed and the warehouse are unchanged**, as expected. Their undecided components are small and never
  reach this step at the default size limit.
- **"Same answers" does not fully hold.** The forces read in iterations that end as a Mechanism differ from
  the dense path's, and with the step forced on for components of every size 7 of 479 suite tests fail.
  Those differences sit in small components that would keep the dense step at the default size limit.

## 2. What was built

- `solver.singular_soft_spring_relative` (default 0, off). When above 0, a component over
  `dense_dof_limit` whose sparse solve fails takes the soft step where it took the dense factorisation.
  Full-rank iterations are untouched. Restore, power check and `FillMechanism` are unchanged and take the
  same two inputs as before: the load projected on the free motions, and the supported part of the solution.
- **The method**, one sparse factorisation of the stiffness plus a small shift, then back-solves only:
  - the free-motion projection from eight back-solves and a small eigenvalue problem on their span (the
    plain "apply the shifted inverse k times" form measured worse, see §3);
  - the supported part of the solution from one solve and ten refinements.
- **Three bands on the projection's size relative to the load:**
  - above 1e-6: use the soft result;
  - at or below 2e-13: the motion is not driven by the load; build the set of free motions the same sparse
    way and carry on as the dense path does;
  - in between: fall back to the dense step.
- Recommended value from the measurements: 1e-11.
- Spike-only switches: `LOADPATH_SOFT_SINGULAR`, `LOADPATH_SOFT_ALL_SIZES` (force the step for every size),
  `LOADPATH_SOFT_DIAG` (log soft against dense per iteration), `LOADPATH_END_LOG`, and tuning variables.
- Default suite with the setting off: 479 passed, 0 failed.

## 3. Accuracy against the dense step (arch, 17 driven singular iterations)

Relative difference of the soft projection from the dense one:

| Method | Shift | Worst | Typical |
|---|---|---|---|
| shifted inverse applied 60 times | 1e-6 | 0.25 | |
| same | 1e-8 | 1.1e-6 | 2.9e-7 |
| same | 1e-12 | 5e-3 | |
| eight back-solves plus the eigenvalue step (as built) | 1e-11 | 6.7e-8 | 2.8e-11 |

- The supported part of the solution matches its exact value to 2e-10 with ten refinements (7e-2 with none).
- **A weak motion cannot be matched at any setting.** When the dense projection is 1e-13 to 7e-12 of the
  load, the soft one is off by 7% to 200%. That is the reason for the fallback band. The arch has one such
  iteration: pass 3, iteration 5, the near-zero motion already flagged in the closing-rate note.

## 4. The arch

| Mode | Runs | Median |
|---|---|---|
| dense step | 4.16, 4.44, 4.08, 4.03, 4.32 s | 4.16 s |
| soft step (as built) | 0.86, 0.83, 0.88, 0.90, 0.88 s | 0.88 s |
| soft step, load-free motions sent to dense | 1.29 to 1.53 s | 1.41 s |

An independent re-run by the orchestrator read 4.05 to 4.38 s and 0.91 to 0.98 s, with identical output.

- Of the 21 singular iterations, 17 take the soft step, 3 are load-free and take the sparse free-motion
  set, and 1 (the weak motion) falls back to dense.
- **Where the 0.88 s goes:** all the soft solves together 0.09 s; the three free-motion sets 0.02 s; the one
  dense fallback about 0.27 s; the rest is the full-rank sparse iterations and the cascade (an arch of the
  same size that never goes singular takes 0.16 s for the whole process).
- **What limits it now:** the single dense fallback is about 30% of the run.
- **Same:** per-pass endings (Overstressed with 1, 3 and 8 iterations, then Mechanism in 16), the 133
  released voussoirs, both piers standing, every corner state, cluster and failure id, and the motions to
  8.5e-13.
- **Different:** the forces recorded in pass 4's singular iterations and the final readings, by up to 200%
  with sign flips; 491 axis labels; failure ratios by up to 1.6%.

## 5. Why the forces differ

A singular system has many valid force solutions. The dense step's `SolveInRange` returns one that depends
on its pivot order: it puts the unbalanced part of the load on whichever unknowns were pivoted last. The soft
step returns the one that balances the load with its free-motion part removed. Neither is "the" answer; they
are two choices. One test (`Statics.Solver.RestoredCornerTangentDoesNotInflateThePivots`) asserts the dense
choice. An attempt to reproduce the dense choice from an unpivoted sparse factorisation was unusable.

## 6. The suite with the step forced on for every size

472 passed, 7 failed. These are all components under 600 unknowns, which keep the dense step at the default
limit, so none of this happens with the setting simply on.

| Test | What differs |
|---|---|
| `Statics.Ungrounded.AnOverflowingFloatingPairIsInvalidInputAsOnTheFullPath` | at 1e304 stiffness the pair is no longer read as invalid input |
| `Statics.Factor.TheBlockedFactorReadsItsPins`, `...AcrossPanels`, `Statics.Sparse.AgreesWithTheDensePath` | a few draws count a borderline free motion differently (a pivot threshold against an eigenvalue threshold) |
| `Statics.Solver.RestoredCornerTangentDoesNotInflateThePivots` | the force choice of §5: bed forces up to 23% off the weight |
| `Statics.Anchor.OrdunaHollowPileReadsNoFalseMechanismBelowTheBracketAndNoStandingAboveIt` | a false Mechanism at lambda 0.427: the two projections differ at about 1e-8 to 1e-9, where the restore rule's 1e-9 tolerance sits |
| `Statics.Lime.ARestoreStepReclosureNeitherCountsNorResets` | runs to 50 iterations where it took 13; passes with the sparse free-motion set off |

Solver endings across the suite, dense against soft: Converged 33,491 and 33,459; Mechanism 13,360 and
13,363; `IterationCap` 777 and 784; `PowerCheckFailed` 11 and 10; InvalidInput 985 and 964.

## 7. The shed and the warehouse

All four Measure rows end the same under the dense step, the soft step, and the soft step forced for every
size. At the default size limit the soft step never runs on them: the undecided components are 90 unknowns
(shed) and 198 (warehouse), both `IterationCap`. Times are within noise, except that forcing the soft step
on the shed's small component makes its warm cascade slower (27 ms against 11 ms): a 90-unknown dense
factorisation is cheaper than the soft step.

## 8. Open

- **Which force solution is right in a Mechanism iteration** (§5). The owner has not ruled.
- **The weak-motion iteration** costs the one remaining dense factorisation on the arch. It is the same
  iteration the findings review's free-motion floor would remove.
- **The 1e-9 restore tolerance** sits on the precision floor between the two projections.
- Not done: a per-iteration profile build; the Measure times are single runs; the random start of the
  sparse free-motion set is seeded but not checked for robustness.
- It inherits everything open on `wip/closing-rate-restore` (that branch's note, §9).
