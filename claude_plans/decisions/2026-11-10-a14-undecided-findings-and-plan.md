Status: the findings (§1 to §5) stand. The plan (§6) is superseded by the owner's 2026-11-10 decision: a closing-rate check replaces the history check in the restore rule; the back-off in §6 is kept only as a fallback and was never built. The closing-rate rule was built 2026-11-12 (`2026-11-12-closing-rate-restore-rule.md`), and it removed `RestoreFailedClosed`, so §1 to §5 describe the solver as it was.

# A1.4's Undecided: why it happens and a plan to remove it (2026-11-10)

The owner asked (2026-11-10) why core case A1.4 (`data/scenarios/core-a1-brick-ring-folds.json`, the ring at
R = 2.75 m) ends Undecided, and for a plan. Every number below comes from the pass recorder
(`loadpath_cli scenario <file> --viz <out> --record-passes`) on `solver_rework` at `7dd0464`, Release.

## 1. What happens

| Pass | Solver end | Iterations | What the cascade does |
|---|---|---|---|
| 1 | Converged, Overstressed | 1 | cracks the two inner impost contacts |
| 2 | Converged, Overstressed | 3 | cracks 14 adjacent crown joints (v.59/v.60 to v.72/v.73, all inside the 0.5 band) and the two outer impost contacts |
| 3 | RestoreFailedClosed, Undecided | 7 | forced collapse: all 231 bodies released, piers included |

The Undecided is raised at joint v.66/v.67, the crown joint to the right of the keystone v.66.

## 2. Why pass 3 ends Undecided

Corners 0 and 3 of a ring joint are the extrados edge, 1 and 2 the intrados edge.

1. **Iteration 1.** With tension gone from the 14 cracked joints, the first solve reads tension at every one of
   their intrados corners (167 to 207 N against 535 to 565 N of compression at the extrados). The update rule
   opens every corner in tension in the same step, so all 14 joints become hinges at once.
2. **Iterations 2 to 5.** Fourteen hinges in a row make the crown a slack chain of 13 voussoirs: a mechanism
   with many freedoms, where one or two hinges were needed. The solver takes its motion from the load projected
   on the free motions. That motion closes some of the corners just opened, so the restore step re-closes them:
   6 joints, then 2, 2 and 2. In iteration 5 it re-closes v.65/v.66 and leaves its mirror v.66/v.67 open, so the
   state is no longer symmetric.
3. **Iteration 6.** The lopsided state is full rank. Its solve reads 5 to 11 N of tension at six left-hand
   intrados corners (against about 380 N of compression) and opens them again.
4. **Iteration 7.** The state is a mechanism again. Its motion closes the intrados of v.66/v.67, which was open
   in the last solved state. `RestoreInadmissibleReleases` refuses to re-close a corner that the last
   equilibrium opened (the cycle guard), and returns Undecided.

So the cause is the sequence, not the physics of the final state: opening a whole run of adjacent no-tension
joints in one step overshoots into a mechanism the restore heuristic cannot walk back from.

## 3. What was ruled out, and what it depends on

Same RestoreFailedClosed at v.66/v.67 with each of these overrides, so it is not noise, the cap or the solve path:

| Override | Result |
|---|---|
| `admissibility_tolerance_relative` 1e-6 and 1e-12 | unchanged |
| `max_active_set_iterations` 200 | unchanged |
| `dense_dof_limit` 10 (sparse path) and 100000 (dense path) | unchanged |

It depends on how many adjacent crown joints are cracked when the solve starts:

| `crack_band_fraction` | Crown joints cracked together | Outcome |
|---|---|---|
| 0.5 (default), 0.3, 0.2 | 14 | Undecided, forced, 231 released |
| 0.1 | 10 | Undecided, forced, 231 released |
| 0.05, 0.03 | 6 | Undecided, forced, 231 released |
| 0.02, 0.01, 0.001 | 4 | crown cracks, then the haunches (v.22 to v.25 and mirrors), then **Mechanism**: 133 voussoirs fall, the piers stand, not forced. The file's expectation is met. |

The right answer therefore exists and the solver reaches it when it is not handed a long run of cracked joints.

## 4. Why narrowing the band is not the fix

- The threshold is a joint count (4 passes, 6 fails), not a band value. A ring cut into more voussoirs puts more
  joints inside any band and fails again.
- All 18 scenario files were run at 0.5 and at 0.01. Sixteen give the same result. A1.4 turns green. **A2.5
  turns red**: at 0.01 the pier bases crack before the course's end joints do, and each pier topples with its
  half of the course (117 released against the expected 55, none forced). A2.5's expectation depends on the band.
  That is a separate finding for the owner, tied to the band ruling (cases D2, D3, H1, H2).

## 5. Not examined here

The shed losing a post (RestoreFailedClosed, 15 bodies), the 3.13 wall (restore on loose end bricks) and the
warehouse frame (IterationCap, one corner flipping stuck/slipping at utilisation 1.017) are the other known
Undecided cases. The first two share A1.4's ending; whether they share its cause is not verified. The warehouse
one has a different signature. Step 3 below classifies them.

## 6. Plan (not started)

**Step 0, pin it in tests.** A small fixture that ends RestoreFailedClosed today for the same reason: a short
mortared ring or beam with a run of six or more adjacent joints cracked before the solve, expected to converge
with one or two hinges. Plus A1.4 itself, which leaves `deferred` when green at the default band.

**Step 1, back off instead of overshooting.** In the active-set loop, when an opening step is followed by a
singular state whose motion is inadmissible, return to the last solved state and open only the worst half of
that step's candidates by tension (equal tensions stay together, which keeps symmetry); halve again down to the
single worst joint if needed. The step that fits is already there: the loop keeps the last full-rank state. The
fast path is untouched when no mechanism follows an opening step. Expected on A1.4: pass 3 converges with one or
two crown hinges, reads the haunches Overstressed, the next pass cracks them, then Mechanism.

This is a hypothesis. It has not been prototyped. The evidence is indirect: the solver converges on the same
ring when only four crown joints are cracked.

Risks to check in the build: extra solves against the 50-iteration cap; cost on the warehouse and shed rows
(SCALE.md) where thousands of corners open in the first step; determinism of the tie rule.

**Step 2, review.** review-expert on the test quality and the change; mutants on the back-off rule.

**Step 3, census.** Run the recorder over the shed, 3.13 and the parked red cases before and after step 1 and
sort every Undecided by signature (run of opened joints, loose end brick, stick/slip flip). This says how much
step 1 removes and what the next fix must target.

**Fallback if step 1 does not cover the census:** replace the flip-all update with a principal-pivoting contact
solve (one change per step, as the test oracle's hinge sequence does). It is the textbook method and a larger
change.

## 7. Two rulings for the owner, independent of the fix

1. **What a forced release takes.** When the restore fails the solver already holds a candidate motion. Releasing
   only the bodies that motion moves, instead of the whole component, would have left A1.4's piers standing even
   while the Undecided remained. The 2026-11-08 ruling says the whole component; this would narrow it.
2. **The band and A2.5.** A2.5's outcome changes between band 0.5 and 0.01 (§4). Decide with D2/D3 which
   history is the expected one.
