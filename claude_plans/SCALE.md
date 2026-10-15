# Scale measurements

The slice-6a table (API.md "Slice 6a"): what the engine costs today on structures ten and a
hundred times the fixtures, measured before any scale design. Every row is a `Measure.Scale.*`
case in `tests/ScaleMeasurementTest.cpp`, Release build, run one case per process by the
orchestrator. Times are the case's own `steady_clock` readings, not wall clock.

Machine: the owner's Windows 10 workstation (the one that runs the suite; MSVC 2022, `/O2`,
Eigen 3.4 dense). Dates are the run dates.

## Rows, 2026-10-11 (HEAD 8d05593)

| Structure | Pieces | Joints | Components | Elastic | Certificate | Judge | Cascade: one post removed | Cascade: base course of wall 0 removed |
|---|---|---|---|---|---|---|---|---|
| Wall 10 × 8 (ec6 bonded rows; the harness's walls are bonded) | 86 | 227 | 1 | 50 ms, Stands | 878 ms, Stands | 104 ms, Stands | n/a | 2,652 ms, **0 passes, Undecided** before the rigid-fall fix; **60 ms, 1 pass, Stands** after it (HEAD of the 6a round) |
| Shed (12 × 8 plan, 10 courses, 2 bays, door + window) | 420 | 1,090 | 5 | 282 ms, Stands | 6,877 ms, Stands | 527 ms, Stands | 579 ms, 0 passes, Stands | 1,927 ms, 1 pass, Stands |
| Half warehouse (30 × 15 plan, 20 courses, 3 bays, two openings) | 1,822 | 5,138 | 5 | 59.3 s, Stands | 594 s, Stands | 90.8 s, Stands | 89.6 s, 0 passes, Stands | 255 s, 1 pass, Stands |
| Half warehouse, stage 1a blocked factor (2026-10-12, HEAD 1bceee9) | 1,822 | 5,138 | 5 | 5.9 s, Stands | 572 s, Stands | 11.8 s, Stands | 12.3 s, 0 passes, Stands | 19.0 s, 1 pass, Stands |
| Wall 40 × 32 (ec6 bonded, 1,296 bricks, 7,776 DoF, one component) | 1,297 | 3,476 | 1 | **killed after 25 min inside the first elastic solve**, peak working set 1,071 MB | not reached | not reached | n/a | not reached |
| Warehouse (40 × 20 plan, 30 courses, 4 bays) | 3,695 | | 5 | not run at 6a: its largest wall is 1,215 bricks (7,290 DoF), the row above says the dense solve does not finish | | | | |

Pieces count the ground. The shed's five components are its four walls and the timber frame,
joined only through the ground (the corners are not bonded and the frame does not bear on the
walls). Judge is the combined authority: every component solved alone by the elastic authority,
Mechanism and Undecided refereed by the certificate.

## What the first rows say

- **The certificate dominates at rest.** On the shed it is 13× the judge and 24× the elastic
  solve, and the judge calls it only for the components the elastic field cannot decide. A
  cascade pass that releases pays for it on every component it touches.
- **A floating wall was cut into bricks, or left Undecided, not released whole.** Removing the
  10 × 8 wall's base course leaves 76 bricks with no path to the ground. On the dry wall the
  certificate read Collapses at t* ≈ 1e-12 but its selection picked a motion with relative
  movement at every joint, so the cascade severed every joint and released one cluster per
  brick; on the bonded wall (the row above) the selection did not converge and the pass read
  Undecided. Both the same in Debug and Release (the review rebuilt the pre-fix commit in both).
  Ruled and fixed in the 6a round (DESIGN §10): an ungrounded component carrying no host load
  falls rigid, lists no hinge, and releases as one cluster with nothing severed; the row is
  re-measured in the same cell. A loaded or hanging ungrounded component still takes the
  general selection (`Deferred.Slice6.*`).
- **The dense factorisation is the wall, and it is a slow one.** From the shed to the half
  warehouse the largest wall grows from 125 to 615 bricks (750 to 3,690 DoF) and the elastic
  solve from 282 ms to 59 s. `Factorise` in RbsmStandingAuthority.cpp is an unblocked rank-1
  update over the full square matrix; the review transcribed it and timed it against Eigen on
  the same machine: 750 DoF 123 ms vs 12 ms (LLT) / 15 ms (LDLT); 1,860 DoF 2.5 s vs 0.18 /
  0.25 s; 3,660 DoF 20.4 s vs 1.23 / 3.9 s; 7,776 DoF not run vs 11.5 / 36 s. So the first
  stage of 6b is the factorisation itself (a blocked dense factor is 10–17× faster at the same
  size), and sparsity comes after it. A single judge of the half warehouse takes a minute and a
  half against a 200 ms budget for three times as many pieces.
- **The judge costs twice the elastic solve, and the certificate costs nothing at rest.** The
  combined authority solves the whole structure once and then every component again
  (CombinedAuthority.cpp; the shed 292 ms + 280 ms → 572 ms, the half warehouse 44 s + 43 s →
  82 s in the review's instrumented run), and consults the certificate only on Mechanism or
  Undecided components: 0 of 5 on the shed and the half warehouse. The certificate-alone column
  (594 s on the half warehouse) is what a release pass costs when the field cannot decide.

## Stage 1a of 6b: the blocked factor (2026-10-12)

The blocked factor (`StandingOptions::FactorKind::Blocked` while the reference lived, the only
factor since the 6b close) is a blocked right-looking pivoted LDLᵀ in the style
of LAPACK `dpstrf` behind the same rank test (DESIGN §10; Eigen's own `LDLT` was tried and
does not reveal rank). Elastic authority alone, at rest, Release:

| Structure | DoF | Reference (rank-1) | Blocked |
|---|---|---|---|
| ec6 wall 16 × 20 (328 bricks) | 1,968 | 3.0–3.4 s, Stands after 1 | 0.44–0.50 s, Stands after 1 |
| ec6 wall 32 × 40 (1,296 bricks) | 7,776 | not finished in 25 min (6a) | 24.1 s, Stands after 1 |

The cheap rows with the blocked factor (same machine, same day): the 10 × 8 wall judges in
19 ms (was 72), elastic 11 ms (was 50), the base-course cascade 15 ms; the shed judges in
136 ms (was 614), elastic 78 ms (was 282), its cascades 199 and 216 ms (were 579 and 1,927).
The certificate column is unchanged (851 ms and 7.3 s): its interior point has its own dense
factor.

The half warehouse row re-run at HEAD 1bceee9 (the `Measure.Scale.HalfWarehouse` test from a
copy of the Release exe): elastic 5.9 s (was 59), judge 11.8 s (was 91), the post cascade
12.3 s with no pass, the course cascade 19.0 s with one pass; the certificate column 572 s
(was 594), unchanged within noise because its interior point has its own dense factor. The
whole row took 633 s, almost all of it the certificate call the row makes for the record.

The 32 × 40 wall at 24 s and the half warehouse's elastic solve at 5.9 s are the n³ constant
brought down, not the n³ removed: stage 1b's sparse factor is what makes a 1,215-brick
warehouse wall cheap, and the judge still solving the whole structure and then every
component again is why the judge is twice the elastic solve.

## Stage 1b-i of 6b: the sparse path (2026-10-12, HEAD 0d13a29)

Components above `DenseDofLimit` (600 DoF) take `SimplicialLDLT` under AMD, accepted on an
inverse-iteration estimate of the smallest eigenvalue (DESIGN §10). Rows re-run from a copy of
the Release exe built at 0d13a29, one process each, with the stage-1b-i review and the
stage-1b-ii round building on the same machine (so the small numbers carry noise):

| Structure | Pieces | Joints | Components | Elastic | Certificate | Judge | Cascade: one post removed | Cascade: base course of wall 0 removed |
|---|---|---|---|---|---|---|---|---|
| Wall 10 × 8 | 86 | 227 | 1 | 10 ms, Stands | 918 ms, Stands | 23 ms, Stands | n/a | 15 ms, 1 pass, Stands |
| Shed | 420 | 1,090 | 5 | 39 ms, Stands | 7.7 s, Stands | 87 ms, Stands | 140 ms, 0 passes, Stands | 177 ms, 1 pass, Stands |
| Wall 40 × 32 (1,296 bricks, 7,776 DoF) | 1,297 | 3,784 | 1 | **161 ms, Stands** (killed at 25 min in 6a, 24.1 s dense at 1a) | **754 s, Stands** | 377 ms, Stands | n/a | 50.7 s, 1 pass, Stands |
| Half warehouse | 1,822 | 5,138 | 5 | **170 ms, Stands** (59 s in 6a, 5.9 s at 1a) | 589 s, Stands | **457 ms, Stands** (91 s, 11.8 s) | 568 ms, 0 passes, Stands | 5.1 s, 1 pass, Stands |
| Warehouse (40 × 20 plan, 30 courses, 4 bays; first run) | 3,695 | 10,678 | 5 | **531 ms, Stands** | 1,566 s, Stands | **1.56 s, Stands** | 1.70 s, 0 passes, Stands | 40.6 s, 1 pass, Stands |

What the rows say: the elastic solve of the largest wall is now 0.16 s and its judge 0.38 s;
the warehouse (3,695 pieces, its first run ever) solves elastically in 0.53 s and judges in
1.56 s, the half warehouse in 0.17 s and 0.46 s, so the elastic side is inside the design
budget of a fifth of a second for the half warehouse and within a factor of three of it for
the whole; the certificate is the whole of the remaining cost (1,566 s on the warehouse,
589 s on the half warehouse, both at rest, where the row calls it only for the record) (754 s on the wall at rest, called by the row for the record, and
by the cascade for every Mechanism or Undecided component). The base-course cascade on this
wall is 50.7 s for one pass, and that is not the certificate (the floating remainder carries
no host load, so the rigid-fall rule decides it without an interior point): the floating
1,215-brick remainder is rank-deficient, the sparse factor hands it to the dense blocked
factor at about 24 s, and the 1b-i judge still solved it twice (whole structure, then the
component); the warehouse's base-course cascade is the same 40.6 s for the same reason, and
the half warehouse's 5.1 s is its 595-body wall going dense once or twice. 1b-ii removes the second solve; the first is a dense factorisation spent to
discover that an ungrounded component is a mechanism, which its connectivity already says. Stage 1b-ii (one solve
per judge, no copies) measured on the shed from a second copy: judge 40 ms against an elastic
solve of 39–54 ms, since no component is consulted. Stage 1b-iii puts the certificate's
normal equations on a sparse factor above the same limit.

## Stage 1b-iii of 6b: the certificate's sparse normal equations (2026-10-12, branch 0c52f6f)

Components above `DenseRowLimit` (600 rows) factor `A D Aᵀ` with `SimplicialLLT` under AMD,
the load-factor coupling assembled on the demand's support (DESIGN §10). Rows from a copy of
the branch's Release exe, one process each, with the stage-2 dev round building alongside:

| Structure | Certificate (dense, 1b-i row) | Certificate (sparse) | Interior-point iterations | Elastic | Judge | Cascade: base course |
|---|---|---|---|---|---|---|
| Shed | 7.7 s | 7.1 s (6.65 s alone) | 35 and 35 | 48 ms | 40 ms | 93 ms |
| Half warehouse | 589 s | **152 s** | | 194 ms | 179 ms | 2.7 s |
| Wall 40 × 32 | 754 s | **114 s** | | 207 ms | 225 ms | 23.6 s |

What the rows say: the factor was a quarter to a sixth of the certificate's cost at scale,
not the whole of it. The shed's components (684 and 714 rows) barely cross the limit and gain
nothing; the half warehouse's 580- and 595-body walls (3,480 and 3,570 rows) and the 1,296-brick wall (7,776
rows) gain 4–7×, which leaves 114–152 s per structure at rest. What remains per
interior-point iteration is unprofiled: the per-component dense `Eigen::LLT` in
`SelectionSystem`, the cap-site loops, the dense `Demand`/`AddSiteBlock` work on the dense
path's data structures, and the iteration count itself. The next step is a profile, not a
guess. The base-course cascade on the large wall halves (50.7 s to 23.6 s) because the 1b-ii
judge no longer solves the rank-deficient floating remainder twice; the one dense solve it
still spends is the item logged in CURRENT_STATE.

## Profile of the certificate at rest (2026-10-12, commit 0c52f6f, Release, measured alone)

Wall-clock accumulators per phase in a throwaway build (not committed):

| | Shed | Half warehouse | Wall 32 × 40 |
|---|---|---|---|
| Total | 7.13 s | 157.9 s | 108.2 s |
| Largest component: rows / joints / cap sites | 714 / 315 / 1,260 | 3,570 / 1,699 / 6,796 | 7,776 / 3,784 / 15,136 |
| Inscribed / circumscribed iterations per component | 13/13, 22/18, 34/35, 13/13, 9/9 | 41/43, 14/15, **66/200 (stalled)**, 14/15, 9/9 | 28/27 |
| Assembly (triplets, coupling, `setFromTriplets`, `analyzePattern`) | 0.30 s (4 %) | 11.0 s (7 %) | 6.7 s (6 %) |
| Factor (`factorize` with retries; dense below 600 rows) | 0.50 s (7 %) | 46.0 s (29 %) | **66.6 s (62 %)** |
| Cap sites (18 × 18 `Mww` build and LLT per site, plus the `SolveNormal` cap loops) | **4.71 s (66 %)** | **73.6 s (47 %)** | 24.5 s (23 %) |
| Triangular solves | 0.06 s | 2.3 s | 1.8 s |
| `SelectionSystem` | 0 (never runs at rest) | 0 | 0 |
| Everything else (`Apply`/`ApplyTranspose`, vectors, residuals) | 1.56 s (21 %) | 25.0 s (16 %) | 8.6 s (8 %) |

What it says: the factor's remaining cost on the wall is the coupling block on the demand's
support, dense at 1,296 × 1,296 (1.68M of the 1.99M non-zeros; the site part alone factors in
5.0 s against 81.6 s); every corner is a cap site at about 18 µs per site per factorisation,
arithmetic not allocation; one half-warehouse circumscribed solve converged to 1.05e-9 against
1e-9 at iteration 62, diverged to 6.75e-6 and ran to the 200 cap with 139 regularisation
retries (59 s for nothing, the verdict Stands anyway on the inscribed factor); iteration
counts do not grow with size. Stage 3 (DESIGN §10) takes these in order: the structured
cap-site factor (exact), the stall rule, the coupling out of the factor with refinement, and
only then a verdict-only option that skips the circumscribed solve.

## Rows at the merged HEAD b72c262 (2026-10-12): stages 1b-i, 1b-ii, 1b-iii and 2 together

From a copy of the Release exe, one process each, with two reviews and a test round building
alongside (small numbers carry noise). The cascade rows are COLD: the harness builds the
structure, removes the post or the course and runs the cascade with no previous verdict, so
the event region (stage 2) helps only between passes, not on the first judge.

| Structure | Elastic | Certificate | Judge | Cascade: one post removed | Cascade: base course removed |
|---|---|---|---|---|---|
| Shed | 45 ms | 6.9 s | 32 ms | 82 ms, 0 passes | 51 ms, 1 pass |
| Half warehouse | 173 ms | 155 s | 192 ms | 453 ms, 0 passes | 3.1 s, 1 pass |
| Warehouse | 509 ms | **254 s** (1,566 s at 1b-i) | **485 ms** (1.56 s at 1b-i, never finished before 1a) | 698 ms, 0 passes | 20.9 s, 1 pass |

Against 6a's first rows: the half warehouse's judge 90.8 s to 0.19 s, its post cascade 89.6 s
to 0.45 s, its course cascade 255 s to 3.1 s; the warehouse now has rows at all. The post
cascade's 0.70 s on the warehouse is one cold judge (0.49 s) plus the removal's bookkeeping;
the stage-2 target of 200 ms is for the WARM case (a previous verdict in hand, only the frame
dirty), which the harness does not yet measure: a `Measure.Scale.*Warm` row that judges once,
removes the post and runs the cascade with that verdict as Previous is owed. The course
cascades are the one dense solve of the rank-deficient floating remainder (CURRENT_STATE).

## Warm rows (2026-10-12, HEAD 2ccc878): a previous verdict in hand

`Measure.Scale.ShedPostRemovedWarm` and `WarehousePostRemovedWarm` judge once, remove the
same post as the cold row, and cascade with that verdict as Previous (stage 2):

| Structure | First judge | Warm cascade, one post removed | Reused | Cold cascade (row above) |
|---|---|---|---|---|
| Shed | 36 ms | 59 ms, 0 passes | 4 of 5 | 93 ms |
| Warehouse | 488 ms | **254 ms**, 0 passes | 4 of 5 | 698 ms |

The stage-2 target (200 ms when only the frame is touched) is missed by 54 ms on a loaded
machine; the review measured the same case at 317 ms with 225 ms of it the frame's own
certificate (one Solve on the frame's 33 bodies is 52 ms, everything else about 40 ms). The
event region has done its part; the remainder is stage 3's.

The 1b-ii/1b-iii review traced the half warehouse's 595-body component on both paths: its
circumscribed solve reaches a gap of 1.04e-9 against 1e-9 at iteration 62, every factor
after that needs the regularised retry, the primal residual jumps to 6.7e-6 and the solve
runs to the 200 cap, identically on the dense path (537 s for that component dense, 145 s
sparse); about 137 stalled iterations at 0.55 s. The stall predates 1b-iii and is stage 3b.
The review also measured the sparse assembly bit-identical to the dense matrix on 1,268
matrices, coupling included, and proposed for stage 3c a bordered system
`[M_sites d; dᵀ −1/Coupling]` eliminated last under `SimplicialLDLT`, which keeps the
coupling out of the factor without needing the site part to be invertible.

## Rows after stages 3a and 3b (2026-10-12, HEAD 722c41b)

From a copy of the Release exe, one process each, with two reviews and a test round building
alongside:

| Structure | Elastic | Certificate | Judge | Cascade: post | Cascade: base course |
|---|---|---|---|---|---|
| Shed | 57 ms | 7.1 s | 56 ms | 127 ms | 80 ms |
| Wall 40 × 32 | 276 ms | 118 s | 224 ms | n/a | 27.1 s |
| Half warehouse | 210 ms | **89 s** (152 s at 1b-iii, 589 s at 1b-i) | 186 ms | 316 ms | 3.0 s |
| Warehouse | 797 ms | 253 s | 625 ms | 828 ms | 21.0 s |

The stall rule took the half warehouse's certificate from 152 s to 89 s (its 595-body wall
from 200 iterations to 72); the shed and the warehouse show no certificate gain on this
loaded machine (the structured cap-site factor's 0.72 on the shed is inside the noise of a
run alongside two builds), and the 32 × 40 wall is unchanged because its cost is the
coupling block in the factor, which stage 3c removes.

## Rows after stage 3c (2026-10-12, the bordered coupling, 425506a)

From a copy of the Release exe, one run each, two other rounds building alongside:

| Structure | Elastic | Certificate | Judge | Cascade: post | Cascade: base course |
|---|---|---|---|---|---|
| Wall 40 × 32 | 284 ms | **34.4 s** (118 s at 3b, 107 s dense at 1b-iii) | 218 ms | n/a | 28.8 s |
| Half warehouse | 201 ms | **58.6 s** (89 s at 3b, 152 s at 1b-iii, 589 s at 1b-i) | 169 ms | 256 ms | 2.62 s |

The coupling block was 62 % of the wall's certificate time; with it out of the factor the wall
gains 3.4× and the half warehouse 1.5× (its five components are smaller, so the coupling block
was a smaller share). The 310-body wall of the Slow ratio test gains only 13 %: at that size the
coupling block is 96k of about 400k non-zeros and the factor is not the dominant cost.

## Rows after stages 3a–3c with the fix rounds (2026-10-12, HEAD 0214d3e)

From a copy of the Release exe, one process, a review and a test round building alongside:

| Structure | Elastic | Certificate | Judge | Cascade: post (cold) | Cascade: base course | Warm post cascade (4 of 5 reused) |
|---|---|---|---|---|---|---|
| Shed | 38 ms | 8.0 s | 47 ms | 118 ms | 84 ms | 50 ms |
| Warehouse | 648 ms | **103 s** (253 s at 3b, 1,566 s at 1b-i) | 519 ms | 660 ms | 29.5 s | **175 ms** |

The warehouse's warm post-removal cascade is under the 200 ms target (254 ms at stage 2), so
stage 3d (a verdict-only option) is dropped; the frame's own certificate is what remains of
the 175 ms. The shed's certificate is unchanged (its components are below `DenseRowLimit`, so
3c does not apply, and 3a's gain is inside the noise of a loaded run).

## Rows at the 6b close (2026-10-12, branch slice-6b-close, the referees removed)

From a copy of the Release exe, one run each, nothing else running:

| Structure | Elastic | Certificate | Judge | Cascade: post (cold) | Cascade: base course | Warm post cascade (4 of 5 reused) |
|---|---|---|---|---|---|---|
| Warehouse | 473 ms | **75 s** (103 s loaded, 253 s at 3b, 1,566 s at 1b-i) | 509 ms | 619 ms | 18.9 s | **140 ms** |

Slice 6b against its 6a opening: the half warehouse judged in 91 s and the warehouse was not
attempted; the warehouse now judges in 0.51 s and re-judges after a post removal in 140 ms.
The certificate, never on the judge's path when the elastic field decides, went from 26 min
to 75 s on the warehouse. The Slow tier holds 9 tests (the two ratio tests retired with the
referees); the 20 × 15 wall's dense-against-sparse ratio reads 0.37.

## Carried from 6b, item 1: the ungrounded shortcut (2026-10-12, branch carried-6b)

An ungrounded component reads Mechanism before any assembly (DESIGN §10 "carried from 6b,
item 1 opening"). From a copy of the Release exe, one run each, same day before and after:

| Structure | Cascade: base course, before | after | Floating remainder solve |
|---|---|---|---|
| Wall 40 × 32 | 22.1 s | **38 ms** before the finiteness gate; **114 ms** at the close (the gate assembles and scales the sparse stiffness once, about 65 ms on the 1,256-brick remainder) | 1,256 bricks: 20.5 s → 4.5 ms before the gate, 70 ms at the close |
| Half warehouse | 2.2 s | **129 ms** | |

Everything else in those rows is unchanged (wall judge 168 ms, certificate 27 s; half
warehouse judge 142 ms, certificate 44 s). The base-course cascade was a dense factorisation
of a singular stiffness spent to learn what connectivity already said.

## Where the base-course cascade spends its time (2026-10-12, at 685a5be)

A throwaway copy of master with timers around the three phases of each cascade pass
(`Judge`, `Release`, `CrackBand`), Release, one run each, cold (no previous result):

| Structure | Bodies after | Readings | Judge | Release | CrackBand | Cascade total |
|---|---|---|---|---|---|---|
| Wall 40 × 32 | 1 | 3,625 | 77 ms | 32 ms | 0 | 110 ms |
| Half warehouse | 1,242 | 5,032 | 137 ms | 25 ms | 0 | 163 ms |
| Warehouse | 2,480 | 10,519 | 390 ms | 119 ms | 0 | 512 ms |
| Warehouse, post removed, cold | 3,694 | 10,675 | 594 ms | 0 | 0 | 596 ms |

The second pass of each costs under 1 ms. The judge is the floating remainder's finiteness
assembly (about 65 ms per 1,250 bricks) plus the grounded components; the release is the
quadratic scans in `Cascade.cpp` and `Structure::RemoveBody`. Items 2 and 3 carried from 6b
take one each (DESIGN §10, their openings).

After items 2 and 3 (2026-10-12, Release, a copied exe, one run each): the cascade's scans
indexed, the judge consulting the certificate once per judge, the shortcut's finiteness check
a fold.

| Structure | Judge, cold | Cascade: base course | Cascade: post removed, cold |
|---|---|---|---|
| Wall 40 × 32 | 197 ms | **16 ms** (was 110) | |
| Half warehouse | 158 ms | **125 ms** (was 163) | 216 ms, one run; the reviewer read 266 to 274 ms at HEAD against 237 to 269 ms before, so this cell is noise, not a change |
| Warehouse | 553 ms | **314 ms** (was 512) | 632 ms |
| 2,000 floating pairs | | **15 ms** (was 575) | |
| 8,000 floating pairs | | **67 ms** (was 8,833) | |
| 2,000 Overstressed bricks, one crack pass | | **under 2 ms** (was 16) | |
| 8,000 Overstressed bricks, one crack pass | | **6 ms** (was 266) | |

The judge on structures of five components is unchanged (its cost is the elastic solve and the
certificate of the grounded components); what fell is everything proportional to the component
count times the structure, and the floating remainder's finiteness check (65 ms to under 10).
The warm judge on the 8,000-pair field after one removal: 85 ms to 19 ms (`FindClean` indexed).

After the fix round (0e7fa19): the wall 15 ms, the half warehouse 108 ms, the warehouse 325 to
334 ms (noise around the 305 to 331 ms the review read), the crack band bucketed by component.

The certificate's cap-site loops solving into per-site scratch (2026-10-12, Release, copied
exes from before and after, alternated, best of three `DecideWith` calls per process; every bound
and iteration count bit-identical in hexfloat on the shed, the half warehouse and the 10 × 8 wall):

| Structure | Certificate before | After | Ratio |
|---|---|---|---|
| Shed | 5,174 ms | 4,370 ms | 0.84 |
| Half warehouse | 47.2 s | 44.0 s | 0.93 |
| Wall 10 × 8 | 619 ms | 512 ms | noise |

Run-to-run noise is about ±0.5 s on the shed. The absolute times are below the 7.1 s and 73.6 s
the profile table records for the shed and half warehouse; that table predates 3c.

## Rows and the elastic profile (2026-10-15, HEAD 5998770, Release, copied exe, one row per process)

| Structure | Pieces | Joints | Components | Elastic | Certificate | Judge | Cascade: base course | Cascade: post removed |
|---|---|---|---|---|---|---|---|---|
| Wall 10 × 8 | 86 | 227 | 1 | 13 ms | 880 ms | 10 ms | 0.9 ms, 1 pass | |
| Shed | 420 | 1,090 | 5 | 35 ms | 5.3 s | 33 ms | 27 ms, 1 pass | 68 ms cold, 45 ms warm |
| Wall 40 × 32 | 1,297 | 3,784 | 1 | 168 ms | 30.7 s | 192 ms | 16 ms, 1 pass | |
| Half warehouse | 1,822 | 5,138 | 5 | 177 ms | 46.5 s | 184 ms | 130 ms, 1 pass | 229 ms cold |
| Warehouse | 3,695 | 10,678 | 5 | 419 ms | 77.5 s | 549 ms | 294 ms, 1 pass | 577 ms cold, 138 ms warm |

Every verdict Stands. "Warm" is the post-removed cascade with the first judge as `Previous` (4 of
5 components reused).

Where the warehouse's cold elastic solve spends its time (profiled separately, 442 to 530 ms per
run with the instrumentation): factorization about 185 ms (40%, `SimplicialLDLT` at about
1.9 G multiply-adds/s); assembly about 175 ms (building the triplet list, both triangles into an
unreserved vector, about 100 ms; `setFromTriplets` about 74 ms); `analyzePattern` about 41 ms;
the eigenvalue check about 32 ms. Every component converges in one active-set pass at rest, the
solve is single-threaded, nothing is solved twice, and `Components()` costs 0.4 ms. The judge is
the elastic solve plus about 1.5 ms.

The warm re-judge after the post removal is about 150 ms, and most of it is one component: the
198-DoF frame flips one corner Stuck ↔ Slipping to the 50-pass cap (36 to 46 ms), reads
Undecided, and the certificate takes 96 to 111 ms to stand it. The candidate speed-ups and this
chatter are listed, parked, in IMMEDIATE_UPDATES.md section 4.
