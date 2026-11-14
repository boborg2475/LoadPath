Status: spike measured 2026-11-13 on branch `spike/soft-singular-step`, built on `wip/closing-rate-restore`. Not merged, not reviewed, default off. No decision taken; the owner reviews this.

# Soft springs at the singular step only: spike results (2026-11-13)

The first soft-spring spike (`spike/soft-springs`, `2026-11-12-soft-spring-spike.md` on that branch) put the
springs on every iteration and failed: the huge loose motion fed the corner rules.

A one-page summary of this whole note is at https://claude.ai/artifact/QGcCfE5GaEwBfTANqFxwVf (private to the owner).
 The owner asked
(2026-11-13) what happens if the springs run only when the matrix is singular. This is that spike. Release
build; every number was measured on this branch.

## 1. Summary

- **The arch gets the same answer about 4.5 times faster.** A1.4: 0.9 s against 4.2 s with the dense step.
  Every corner state, verdict and released body is the same, pass by pass.
- **All 18 scenario files give byte-identical CLI output.**
- **The shed and the warehouse are unchanged**, as expected. Their undecided components are small and never
  reach this step at the default size limit.
- **"Same answers" does not fully hold.** The forces read in iterations that end as a Mechanism differ from
  the dense path's. With the step forced on for components of every size 7 of 479 suite tests fail, and with
  it on at the default size limit 2 fail (corrected in §9; this note first said none).

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
limit. Two of the seven also fail with the setting simply on at the default limit: see §9.

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
- **The weak-motion iteration** cost the one remaining dense factorisation on the arch. The free-motion floor
  removes it: see §9.
- **The 1e-9 restore tolerance** sits on the precision floor between the two projections.
- Not done: a per-iteration profile build; the Measure times are single runs; the random start of the
  sparse free-motion set is seeded but not checked for robustness.
- It inherits everything open on `wip/closing-rate-restore` (that branch's note, §9).

## 9. The free-motion floor (added 2026-11-13)

The owner asked for the free-motion floor from the findings review, then the arch with the floor and the soft
step together.

### 9.1 What the floor is

- A free motion the load drives only weakly is treated as not driven at all. It goes to the load-free branch
  and the restore rule never sees it.
- **It is an existing setting with a larger value.** The dense path already tests the load's projection on
  the free motions against `solver.neutral_load_tolerance_relative` (1e-12) times the load. The floor is that
  number raised. On the soft path it is the lower band of §2 (2e-13).
- **What the number means:** the fraction of the load that must be unbalanced before the solver treats a
  motion as real. A floor of 1e-9 reads a structure within one billionth of its limit as not moving.
- The findings review words it as a ratio of free motion to elastic deflection. With a soft spring that ratio
  is this same fraction divided by the spring value, so it is one measure, not two.
- Spike-only switch `LOADPATH_NEUTRAL_FLOOR` (commit `98d661e`) sets the floor on both paths so the suite can
  be run under it. Unset, nothing changes: 479 passed, 0 failed.

### 9.2 The arch

| Path | Floor | Singular steps | Runs |
|---|---|---|---|
| dense | 1e-12 (none) | 21 dense | 3.6 to 4.1 s |
| dense | 1e-11, 1e-9, 1e-6 | 21 dense | 3.6 to 4.1 s |
| soft step | none | 17 soft, 3 load-free, 1 dense fallback | 0.72, 0.72, 0.73, 0.74, 0.76 s |
| soft step | 1e-11 | 18 soft, 4 load-free, 0 dense | 0.59 to 0.62 s |
| soft step | 1e-9 | same | 0.58 to 0.62 s |
| soft step | 1e-6 | same | 0.58 to 0.66 s |

- **The answer is the same in all eight runs:** passes end Overstressed, Overstressed, Overstressed,
  Mechanism in 1, 3, 8 and 16 iterations; 133 voussoirs released, both piers stand. The CLI output is
  byte-identical across all eight.
- **The lopsided step is gone.** Without the floor, pass 3 iteration 5 re-closes `v.65/v.66` alone. With it
  that iteration changes the four springing joints as mirror pairs, the next changes `v.59/v.60` and
  `v.72/v.73`, and the pass ends in the same state (1,480 corners closed, 16 open, 24 slipping) with the
  same 30 haunch cracks.
- **Soft and dense agree step for step under the floor.** At 1e-9 every iteration's corner states, every
  ending and the released set are identical between the two paths.
- **The dense fallback is gone on the arch**, which is the 0.74 s to 0.60 s.
- Any floor from 1e-11 to 1e-6 works on the arch because nothing sits between its weak motion (4.8e-12 of
  the load) and its weakest real one (1.4e-4).

### 9.3 The suite under the floor

Default tier, 479 tests, measured by an agent; the 1e-9 dense row was re-run by the orchestrator.

| Floor | Soft step | Passed | Failed |
|---|---|---|---|
| none | off | 479 | 0 |
| 1e-11 | off | 478 | 1 |
| 1e-9 | off | 478 | 1 |
| 1e-6 | off | 470 | 9 |
| none | on, default size limit | 477 | 2 |
| 1e-9 | on, default size limit | 476 | 3 |
| none | on, every size | 472 | 7 |
| 1e-9 | on, every size | 473 | 6 |
| 1e-6 | on, every size | 466 | 13 |

- **1e-11 and 1e-9 fail the same one test.** `Statics.Factor.TheBlockedFactorReadsItsPinsAcrossPanels`:
  three dry walls (draws 13, 34 and 51, friction 0.40 to 0.49, pushed at or past sliding) go from Mechanism
  to Undecided. Not a false stand. §9.5 shows the three Mechanisms were proven on noise.
- **No test at 1e-11 or 1e-9 turns a collapse into a stand.**
- **1e-6 is unusable.** Collapses just past their limit now stand: the free wall overturning, the incline
  tipping, the Housner block, the coursed free wall, the harmonic stack. The incline slide and a dry-bed slip
  go Undecided. This is the meaning in §9.1 showing: those tests load a block within 1e-6 of its limit.
- **The floor fixes one soft-step failure:** the Orduna false Mechanism at lambda 0.427. The other six of §6
  stay. It adds none beyond the `AcrossPanels` one.
- **Correction to §1 and §6.** With the soft step on at the default size limit and no floor, 2 tests fail:
  `Statics.Ungrounded.AnOverflowingFloatingPairIsInvalidInputAsOnTheFullPath` and
  `Statics.Sparse.AgreesWithTheDensePath` (3 trades against a budget of 1). The suite does hold components
  over the limit.
- The ending census is not quoted: six processes append to one log and totals drift by about 1,500 between
  identical runs.

### 9.4 Does the floor remove the dense fallback?

Singular steps with the soft step forced on for every size:

| Floor | Soft | Load-free | Dense fallback |
|---|---|---|---|
| none | 39,461 | 330 | 596 |
| 1e-9 | 39,388 | 497 | 503 |
| 1e-6 | 39,566 | 758 | 438 |

- **No. At 1e-9 it removes about one fallback in six.**
- **What the 503 at 1e-9 are:** 410 are motions classed as load-free where the sparse free-motion set came
  back empty; 63 lie between the floor and the 1e-6 line where the soft result is trusted; 30 are failed
  factorisations.
- **The 410 are the limit, and the floor does not touch them.** The count is 410 with no floor and at 1e-6
  too. At the default size limit the suite shows 231 fallbacks, every one an empty free-motion set, so this
  reaches large components.
- An earlier count of "about 450" fallbacks was low; 596 is the measured figure.

### 9.5 The three dry walls (2026-11-14)

The owner asked to look at them. Traced per iteration with the spike-only `LOADPATH_TRACE_LOG` and the case
`Measure.Spike.FloorLostWallDraws` (commit `88665e8`).

- **The floor did not break three good collapses. It removed three false proofs.** Without the floor each
  wall's Mechanism is declared on a motion the load drives at about 1e-12 of itself:

| Draw | Wall | Declared at iteration | Drive at the declaration | With the floor |
|---|---|---|---|---|
| 13 | 4 x 4, mu 0.405, lambda (0.407, 0.376) | 18 | 2.6e-11 | `IterationCap` at 50 |
| 34 | 3 x 3, mu 0.404, lambda (0.255, 0.452) | 46 | 2.0e-12 | `IterationCap` at 50 |
| 51 | 5 x 4, mu 0.488, lambda (0.418, 0.250) | 15 | 2.2e-12 | `IterationCap` at 50 |

- **The declared motions are not what the loads do.** Draw 13: two top-course bricks tip about their long
  edge, which needs a sideways push of 1.58 of weight against the 0.376 applied. Draw 34: an end half-brick
  tips against the push and a top brick lifts straight up. Draw 51: one half-brick twists and moves against
  the load.
- **The power check cannot reject them.** On a dry joint the dissipation is 0 and the load's power on the
  motion is the squared size of the projection, which is positive for any non-zero projection however small
  (read from the code, not measured separately).
- **Is collapse the right answer?** Draws 13 and 34: yes. The push is 0.55 and 0.52 of weight against
  friction of 0.405 and 0.404, and the admissibility oracle says they cannot stand. They should slide. Draw
  51: not established. The push is 0.13% inside the friction cone and the oracle says Ambiguous.
- **So two were right by accident and one is unknown.** Before the closing-rate rule all three were pinned
  Undecided (`solver_rework`); the pin regeneration on `wip/closing-rate-restore` made them Mechanism.
- **Why they are undecided underneath.** In every run, with and without the floor, the strongly driven
  candidates (0.2 to 0.7 of the load) are each turned down by the restore step re-closing corners. No corner
  pattern repeats, so the run wanders to the cap. Why restore turns them down was not determined.

How common this is, from the trace over the whole default tier (one process, 479 passed):

| Drive at the Mechanism declaration | Declarations |
|---|---|
| 1e-12 to 3e-11 | 14 |
| 3e-11 to 1e-9 | 0 |
| 1e-9 to 1e-6 | 28 |
| 1e-6 to 1e-3 | 79 |
| above 1e-3 | 12,166 |

- **14 of 12,287 declarations sit on a near-zero drive, with a clean gap above them.** They are five
  distinct solves: the three walls (each solved twice) and two not identified (78 and 174 unknowns, four
  times each) that fail no test under the floor.
- The 28 between 1e-9 and 1e-6 include the anchor tests that load a block just past its limit; a floor at
  1e-6 loses those (§9.3).
- Counted over every driven singular iteration, not only declarations, there is no gap (§9.6).

### 9.6 Open

- Why the restore step turns down the strongly driven slide on these dry walls. This is what leaves them
  undecided, with or without the floor.
- The power check accepts any dry-joint motion. The floor covers that hole at the singular step; nothing
  else does.
- The two unidentified solves declared on a near-zero drive.
- Why the sparse free-motion set comes back empty 410 times. That, not the floor, decides how often a large
  component pays for a dense factorisation.
- The floor's value. 1e-9 is 200 times above the arch's weak motion and 140,000 times below its weakest
  real one; 1e-11 is only twice above. The suite has motions in every decade from 1e-13 to 1e-7, so no value
  separates cleanly there.
- Not built: the floor as a changed default with tests. It was only tried through overrides and the switch.
