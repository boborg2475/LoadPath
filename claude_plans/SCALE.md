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
