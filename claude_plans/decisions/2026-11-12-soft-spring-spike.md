Status: spike measured 2026-11-12 on branch `spike/soft-springs`. Not merged, not reviewed, default off. No decision taken; the owner reviews this.

# Soft-spring spike: what it does on the arch, the shed and the warehouse (2026-11-12)

The owner asked (2026-11-12) for a spike of the soft-spring idea in
`2026-11-12-solver-rework-findings-review.md`, run on the arch, the shed and the warehouse, with pictures.
Every number here was measured on this branch, Release, wall times as medians of 5 from a copied exe.

## 1. Summary

- **The arch folds correctly and faster.** A1.4 passes at 1e-6: 0.34 s, against 0.93 s for today's rule
  (wrong answer) and 5.6 s for the closing-rate rule (right answer).
- **It does not fix the shed or the warehouse.** Both still end `IterationCap` and are force-released. On
  the shed the iteration diverges.
- **It is not a general replacement as built.** With it forced on across the suite, 64 of 475 tests fail,
  `IterationCap` endings go from 452 to 8,829 and proven Mechanisms halve.
- **The spring value is a narrow window.** 1e-4 holds the arch up, 1e-8 never settles, only 1e-6 works.

## 2. What was built

- `solver.soft_spring_relative` (default 0, off). When above 0, each free body's six unknowns get a soft
  stiffness equal to that value times the unknown's own diagonal stiffness with every corner closed.
- With it on the solve runs `SolveSoftProblem`: one factorisation per active-set iteration (sparse above
  `dense_dof_limit`, dense LDLT below), the ordinary corner update, no rank detection, no null basis, no
  restore step. `IterationCap` stays Undecided. The ungrounded "falls rigidly" shortcut is untouched.
- **How a mechanism is recognised.** The first measure tried, soft-spring force over mean body load, did
  not separate the two cases: standing passes read up to 1e-1 and mechanism passes from 1e-2. The measure
  kept: on convergence, solve once more with the springs ten times softer and take each body's ratio of
  displacement (all six unknowns). A held body reads about 1, a loose one about 10.
- `solver.soft_spring_mechanism_ratio` (default 3.1623, the square root of 10) is the cut. On the arch's
  mechanism pass the 98 pier bricks read 1.0028 to 1.0044 and the 133 voussoirs 9.44 to 10.32; in the
  standing passes all 231 bodies read 1.0025 to 1.053.
- That test costs one extra factorisation for every converged component.
- Released clusters carry every body's own motion (`ReleasedCluster::BodyMotions`, viewer
  `"motion_kind":"per_body"`), ported from `spike/closing-rate-restore` `c4a0295`.
- Spike-only switches: `LOADPATH_SOFT_SPRING`, `LOADPATH_SOFT_RATIO`, `LOADPATH_END_LOG`,
  `LOADPATH_SOFT_HIST`, `LOADPATH_SOFT_DUMP`, and two `Measure.Spike.SoftSpring.*` cases that write viewer
  JSON for the shed and the warehouse with a post removed.
- Default suite with everything off: 475 passed, 0 failed.

## 3. The arch (A1.4, `core-a1-brick-ring-folds`)

| Setting | What happens | Released | Time |
|---|---|---|---|
| off | pass 3 ends `RestoreFailedClosed` after 7 iterations | all 231, forced (wrong) | 0.93 s |
| 1e-4 | stands in pass 1: the springs carry the arch | nothing (wrong) | 0.11 s |
| 1e-6 | passes 1 to 3 Overstressed (1, 3, 9 iterations), pass 4 Mechanism in 10 | 133 voussoirs, piers stand (right) | 0.34 s |
| 1e-8 | pass 4 ends `IterationCap`: no corner flips after iteration 9 but the held slip forces never settle | all 231, forced (wrong) | 0.69 s |

The motion at 1e-6 is a five-segment fold, mirror-symmetric to the four decimals printed:

- Segments `v.0` to `v.22`, `v.23` to `v.65`, `v.66` alone, `v.67` to `v.109`, `v.110` to `v.132`.
- Hinges at both springings, the haunches `v.22/v.23` and `v.109/v.110`, and either side of the keystone,
  which drops straight down.
- The haunch hinges are three voussoirs lower than under the closing-rate rule (`v.25/v.26`).
- It is slightly smeared: rotation varies about 5% within a segment (a rigid fold is 0%), and the one body
  beside each haunch hinge reads 0.64 against about 0.90 for its segment.

0.34 s is 23 sparse factorisations plus the mechanism test. The findings review's estimate was under 100 ms.

## 4. The shed

- **Nothing removed:** stands at every setting. About 1.7 times slower with springs on (one elastic solve
  38 ms off, 65 ms on) because of the extra mechanism-test solve.
- **One post removed:** the walls stand; the 15-body roof (5 posts, 4 beams, 6 planks) is undecided and
  force-released at every setting.

| Setting | The 15-body roof | Warm cascade |
|---|---|---|
| off | `RestoreFailedClosed`, 18 iterations | 2.7 ms |
| 1e-4 | `IterationCap`, two states alternating at utilisation 1.1 and 1.3 | not timed |
| 1e-6 | `IterationCap`, diverges: utilisation about 15 at iteration 15, 1e67 at iteration 50 | 7.4 ms |
| 1e-8 | `IterationCap`, diverges to 1e105 | not timed |

The two `Parked.E2E.Shed.*` cases read the same with springs on as off: one passes, one fails.

## 5. The warehouse

- **Nothing removed:** stands at every setting. About 1.8 times slower with springs on (one elastic solve
  476 ms off, 849 ms on).
- **One post removed:** the walls stand; the 33-body roof (9 posts, 8 beams, 16 planks) ends `IterationCap`
  and is force-released at every setting. Warm cascade 39 ms off, 43 to 45 ms on.
- At 1e-6 the corner flipping stops after iteration 3 but the held forces alternate (utilisation 1.01 and
  1.03). At 1e-8 it is identical to off. The findings review did not expect soft springs to fix this.

## 6. The other side: single blocks and the suite

- **Both incline anchors fail with springs on.** The slide threshold moves 1.4e-6 rad (tolerance 1e-6) and
  the slide is no longer a pure translation: rotation is the largest part. The tip threshold moves 3.0e-6
  rad and the motion is not about the downhill edge. The springs supply a small missing moment, so a
  slide-plus-tip motion becomes admissible.
- **Suite with springs forced on at 1e-6:** 411 passed, 64 failed. By area: Statics.Anchor 16,
  Statics.Solver 15, Statics.Lime 6, Cascade 5, Statics.Authority, Factor and Sparse 3 each, Core.Keystone
  2, the rest 1 or 2 each. None was looked into or fixed.

| Solver ending | Off | On |
|---|---|---|
| Converged | 33,403 | 33,670 |
| Mechanism | 10,788 | 4,823 |
| `RestoreFailedClosed` | 2,872 | 0 |
| `PowerCheckFailed` | 8 | 0 |
| `IterationCap` | 452 | 8,829 |
| InvalidInput | 985 | 1,109 |

## 7. Why it misbehaves

- **The loose part's motion feeds back into the corner rules.** A corner that the huge mechanism motion
  pushes shut reads an enormous trial compression (joint stiffness times that motion). That becomes an
  enormous friction capacity and held force, which drives a larger motion. On the arch it settles; on the
  shed it blows up. This is the main defect, and it comes from using a regularised mechanism's
  displacements as if they were elastic ones.
- **The spring value depends on the structure.** Too stiff and the springs hold up what should fall; too
  soft and solve noise, which grows as one over the value, stops the held forces settling.
- **The mechanism test doubles the cost of a standing structure.**

## 8. Not done

- No fix was tried for the divergence. A cap on the trial normal force used for friction capacity is one
  place to start; borrowing soft springs for the singular step only, as the findings review first described,
  is another.
- The suite census ran only at 1e-6. The arch motion was dumped only at 1e-6.
- The pictures for the shed and warehouse use the E2E scene's joint profiles; the timings use the
  catalogue rows.
- The `Parked.E2E.Shed` pictures were not regenerated through `RecordVisual`.

## 9. Where the outputs are

- The page with all six runs (arch, shed, warehouse, each off and on): the "Soft-spring spike" link at the
  top of https://claude.ai/artifact/VgUd282GyArdjwHKB9KurC.
- Raw outputs (viewer JSON, timing and census CSVs, override files, scripts) are in the session scratch
  folder `soft-spring-spike`, not in the repo.
