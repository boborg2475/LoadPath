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

`StandingOptions::FactorKind::Blocked` is a blocked right-looking pivoted LDLᵀ in the style
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
the half warehouse's 5.1 s is its 600-brick wall going dense once or twice. 1b-ii removes the second solve; the first is a dense factorisation spent to
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
nothing; the half warehouse's 600-brick wall (3,600 rows) and the 1,296-brick wall (7,776
rows) gain 4–7×, which leaves 114–152 s per structure at rest. What remains per
interior-point iteration is unprofiled: the per-component dense `Eigen::LLT` in
`SelectionSystem`, the cap-site loops, the dense `Demand`/`AddSiteBlock` work on the dense
path's data structures, and the iteration count itself. The next step is a profile, not a
guess. The base-course cascade on the large wall halves (50.7 s to 23.6 s) because the 1b-ii
judge no longer solves the rank-deficient floating remainder twice; the one dense solve it
still spends is the item logged in CURRENT_STATE.
