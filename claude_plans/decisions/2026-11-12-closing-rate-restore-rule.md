Status: decided by the owner 2026-11-12, not yet built. The measurements are from a spike that is not merged.

# The closing-rate restore rule: spike measurements and the owner's decision (2026-11-12)

The owner decided 2026-11-10 that a closing-rate check replaces the history check in
`RestoreInadmissibleReleases`. This note records what a spike measured on 2026-11-11 and what the owner
decided from it on 2026-11-12.

## 1. The decision

- **Build the closing-rate rule on `solver_rework` with threshold 0.1**, as a setting
  (`solver.restore_closing_rate_threshold`).
- **No stopping rule.** Count what the rule leaves undecided at the re-run (findings review item 3).
- **The bridge false collapse (§5) is not fixed now.** The owner defers it.
- **The released-cluster picture shows each body's own motion** (§6). Approved, to be built as its own change.

## 2. The rule

The restore step runs when the stiffness is singular and gravity drives a free motion. For each open corner:

- `Rate = -GapM / MotionScaleM`, where `GapM` is the normal relative movement of the corner under the motion
  and `MotionScaleM` is the largest relative movement of any joint corner in the component. It runs 0 to 1.
- `Rate > threshold`: the corner is re-closed and the solve repeats.
- `Rate <= threshold`: the corner stays open.
- No open corner above the threshold and positive gravity power: Mechanism.

The old rule re-closed any corner past `admissibility_tolerance_relative` (1e-9), and returned Undecided
(`RestoreFailedClosed`) when that corner was open in the last full-rank solve. Under the new rule that ending
is not produced.

A re-closed corner on a slip-releasing joint re-enters Slipping with its friction capacity taken from the last
full-rank normal force. In every case measured that force was tension, so the capacity is zero (§5).

## 3. Where the spike lives

Local branch `spike/closing-rate-restore` (`3bc921a`, `c4a0295`), worktree `LoadPath-spike-restore`, based on
`9945095`. It is instrumented (environment variables, a CSV log, extra recorder fields) and is never merged or
pushed. The rule is rebuilt on `solver_rework` with tests.

## 4. A1.4 (`core-a1-brick-ring-folds`) per threshold

Release, median of 5 whole-process runs, no recording, about 0.09 s of start-up included. "Slipping" and
"stuck" are how a re-closed corner comes back; the build uses slipping.

| Threshold | Re-entry | Time | Singular iterations | Result |
|---|---|---|---|---|
| old rule | | 1.1 s | 5 | wrong: RestoreFailedClosed in pass 3, forced release takes the piers |
| 1e-9, 1e-6 | slipping | 12.4, 13.6 s | 57 | wrong: IterationCap in pass 4, forced release takes the piers |
| 1e-9, 1e-6 | stuck | 12.1, 12.2 s | 47 | correct |
| 1e-3 | slipping | 14.8 s | 57 | correct |
| 1e-3 | stuck | 12.6 s | 47 | correct |
| 0.01 | either | 7.0, 6.6 s | 26 | correct |
| 0.1 | either | 5.6 s | 21 | correct |
| 0.5 | either | 6.3, 6.2 s | 24 | correct |
| 0.95 | either | 5.9 s | 23 | wrong: only 97 voussoirs fall |

Correct means pass 4 ends Mechanism, the 133 voussoirs are released and both piers stand.

- **Cost.** Time divided by singular iterations is about 0.27 s at every threshold (by division, not
  profiled). The arch has 1,386 unknowns, above `dense_dof_limit`, so every singular iteration is a dense
  factorisation; full-rank iterations use the sparse path and cost almost nothing.
- **Where the 5.6 s at 0.1 goes** (estimated from the iteration counts, not timed per pass): passes 1, 2 and
  5 have no singular iterations and cost almost nothing; pass 3 has 6 (about 1.6 s); pass 4, the one that
  decides the fold, has 15 (about 4 s).
- **Why pass 4 is long.** The restore step re-closes one joint pair on each side per iteration as it walks
  along the haunch, and each of those iterations pays for the dense factorisation.
- **The old rule's 1.1 s is not a fair baseline.** It stops in pass 3 and never runs pass 4. Any rule that
  reaches the right answer pays for pass 4.
- **Neither cost has been worked on**: the price of one singular iteration, or how many the restore step
  needs. Per-iteration timing (findings review item 6) would replace these estimates with measurements.
- **Pass 3 ends the same at every threshold from 1e-9 to 0.95**: same corner states, same 30 haunch cracks.
  Only the iteration count differs (18, 15, 8, 10, 10 for 1e-3 and below, 0.01, 0.1, 0.5, 0.95).
- **At 0.1 the threshold decides twice.** Pass 3 iteration 5 leaves `v.72/v.73` open at 0.077; pass 4
  iteration 10 leaves `v.65/v.66` and `v.66/v.67` open at 0.008. Every corner it re-closes is at 0.108 or
  above, so 0.1 sits in a gap between 0.077 and 0.108.
- **Pass 3 iteration 5 decides on a weak motion.** Its motion scale is 3.6e-9 against 0.057 to 0.159 in
  iterations 2 to 4, just above `neutral_load_tolerance_relative`, and it is the one restore step that is not
  mirror-symmetric.
- **The motion is the textbook fold.** Five rigid segments (`v.0` to `v.25`, `v.26` to `v.65`, `v.66`, `v.67`
  to `v.106`, `v.107` to `v.132`), hinges at both springings, both haunches (`v.25/v.26`, `v.106/v.107`) and
  both sides of the keystone, mirror-symmetric to 2e-11, gravity power positive. The unmodified solver at
  `crack_band_fraction` 0.02 finds the same fold with the haunch hinges one voussoir lower.

## 5. What the rule costs elsewhere

Measured on the spike at 0.1, slipping re-entry, default suite only.

- **Scenario files.** A1.4 passes; the other 17 are byte-identical. At 0.5 `colonnade-middle-post` collapses
  falsely.
- **`RestoreFailedClosed` endings in the suite: 2,872 to 0.** About 2,500 become Mechanism. Whether those are
  correct collapses is not checked.
- **`IterationCap` endings: 452 to 781.**
- **Five suite tests turn red.**
  - `Statics.Solver.RestoredCornerHoldsFrictionOnABridge`: a false Mechanism at lambda (2.5, 0.75). A re-closed
    corner comes back with zero friction capacity. This is a wrong answer the rule causes.
  - `Statics.Factor.TheBlockedFactorReadsItsPins` and `...AcrossPanels`: of 6,000 seeded draws, 1,088 trade
    Undecided for Mechanism and 29 differ otherwise (unconstrained-body counts; one Undecided becomes Stands).
  - `Statics.Sparse.HiddenPivotsAreCaughtByTheEstimate`: draw 715, pinned Undecided, reads Mechanism.
  - `Statics.Lime.TheLateralGridStandsInItsOwnField`: one grid cell moves from Undecided to Stands (12 to 11,
    128 to 129).
- **Two red tests turn green**: the deferred A1.4 scenario and
  `Parked.Statics.Combined.TwoBrickStackPastFrictionCollapses`.
- **No clean gap in the rate across fixtures.** Corners the old rule blocked sit at 0.94 (arch keystone), 0.18
  (two-brick stack), 0.007 to 0.79 (bridge), 0.002 to 1.0 (oracle property test), 1.3e-9 to 7.2e-4 (Orduna
  pile).
- **Not measured:** the 3.13 wall, the Slow tier, the Measure rows other than the four in §7, thresholds
  between 0.1 and 0.5.

## 6. The released-cluster picture

`ReleasedCluster::Motion` keeps one body's mechanism motion and the viewer applies it to the whole cluster, so
a fold is drawn as a rigid tip about one corner. The spike records every body's own motion and the viewer
moves each body by it. The owner approved that change on 2026-11-12.

## 7. The shed and the warehouse (measured 2026-11-12 on the spike build)

`Measure.Scale.Shed`, `ShedPostRemovedWarm`, `Warehouse` and `WarehousePostRemovedWarm`, one per process,
Release, at the old rule and at 1e-9, 1e-3, 0.01, 0.1, 0.5 and 0.95. Times are medians of 3 to 5 runs taken
while another build was running, so differences under about 20% are noise.

- **The rule fixes neither structure's undecided component.**
  - Shed, one post removed: a 15-body component. Old rule: `RestoreFailedClosed` after 18 iterations. Rule at
    1e-9 to 0.5: `IterationCap` at 50. Still undecided, so the forced release takes the same component. At
    0.95 it ends Mechanism after 29 iterations, but 0.95 is wrong for the arch (§4).
  - Warehouse, one post removed: a 33-body component ends `IterationCap` at 50 under every setting. The
    restore step never runs anywhere in the warehouse, so the rule cannot change it.
- **Standing, nothing removed:** both stand under every setting.
- **What was not read:** the measurement prints the end state and the solver endings, not the released bodies.
  That the release is unchanged is inferred from the undecided component being the same size.

| Row | Old rule | 1e-3 | 0.01 | 0.1 | 0.5 |
|---|---|---|---|---|---|
| Shed, one elastic solve | 50 ms | 39 ms | 66 ms | 52 ms | 48 ms |
| Shed, post removed, warm cascade | 5.5 ms | 9.5 ms | 11.8 ms | 11.4 ms | 10.7 ms |
| Warehouse, one elastic solve | 510 ms | 496 ms | 507 ms | 510 ms | 485 ms |
| Warehouse, post removed, cold cascade | 580 ms | 549 ms | 530 ms | 550 ms | 530 ms |
| Warehouse, post removed, warm cascade | 55 ms | 49 ms | 49 ms | 51 ms | 60 ms |

The one real change is the shed's warm cascade, about 6 ms to about 11 ms: its undecided component now runs
to the 50-iteration cap where the old rule stopped at 18. The warehouse rows do identical work under every
setting.
