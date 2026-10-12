# Scale measurements

The slice-6a table (API.md "Slice 6a"): what the engine costs today on structures ten and a
hundred times the fixtures, measured before any scale design. Every row is a `Slow.Scale.*`
case in `tests/ScaleMeasurementTest.cpp`, Release build, run one case per process by the
orchestrator. Times are the case's own `steady_clock` readings, not wall clock.

Machine: the owner's Windows 10 workstation (the one that runs the suite; MSVC 2022, `/O2`,
Eigen 3.4 dense). Dates are the run dates.

## Rows, 2026-10-11 (HEAD 8d05593)

| Structure | Pieces | Joints | Components | Elastic | Certificate | Judge | Cascade: one post removed | Cascade: base course of wall 0 removed |
|---|---|---|---|---|---|---|---|---|
| Wall 10 × 8 (dry) | 86 | 227 | 1 | 50 ms, Stands | 878 ms, Stands | 104 ms, Stands | n/a | 2,652 ms, **0 passes, Undecided** before the rigid-fall fix; **60 ms, 1 pass, Stands** after it (HEAD of the 6a round) |
| Shed (12 × 8 plan, 10 courses, 2 bays, door + window) | 420 | 1,090 | 5 | 282 ms, Stands | 6,877 ms, Stands | 527 ms, Stands | 579 ms, 0 passes, Stands | 1,927 ms, 1 pass, Stands |
| Half warehouse (30 × 15 plan, 20 courses, 3 bays, two openings) | 1,822 | 5,138 | 5 | 59.3 s, Stands | 594 s, Stands | 90.8 s, Stands | 89.6 s, 0 passes, Stands | 255 s, 1 pass, Stands |
| Wall 40 × 32 (dry, 1,296 bricks, 7,776 DoF, one component) | 1,297 | 3,476 | 1 | **killed after 25 min inside the first elastic solve**, peak working set 1,071 MB | not reached | not reached | n/a | not reached |
| Warehouse (40 × 20 plan, 30 courses, 4 bays) | 3,695 | | 5 | not run at 6a: its largest wall is 1,215 bricks (7,290 DoF), the row above says the dense solve does not finish | | | | |

Pieces count the ground. The shed's five components are its four walls and the timber frame,
joined only through the ground (the corners are not bonded and the frame does not bear on the
walls). Judge is the combined authority: every component solved alone by the elastic authority,
Mechanism and Undecided refereed by the certificate.

## What the first rows say

- **The certificate dominates at rest.** On the shed it is 13× the judge and 24× the elastic
  solve, and the judge calls it only for the components the elastic field cannot decide. A
  cascade pass that releases pays for it on every component it touches.
- **A floating wall was cut into bricks, not released whole.** Removing the 10 × 8 wall's base
  course leaves 76 bricks with no path to the ground. The certificate read Collapses at
  t* ≈ 1e-12 but its selection picked a motion with relative movement at every joint, so the
  cascade severed every joint and released one cluster per brick (Debug), or the selection
  failed to converge and the pass read Undecided (the Release row above). Ruled and fixed in
  the 6a round (DESIGN §10 "slice 6a first finding"): an ungrounded component with a net force
  falls rigid, lists no hinge, and releases as one cluster with nothing severed; the row is
  re-measured below.
- **The dense solve is the wall.** From the shed to the half warehouse the pieces grow 4.3×,
  the largest wall from 125 to 615 bricks (750 to 3,690 DoF), and the elastic solve grows
  210× (282 ms to 59 s), the judge 172×, the certificate 86×. That is the per-component dense
  LDLT's n³ (a factor 4.9 in DoF gives 120 in flops) plus the combined authority copying the
  structure once per component. A single judge of the half warehouse takes a minute and a half;
  the design budget is 200 ms for three times as many pieces. 6b starts from this row.
- **The certificate is called even when the elastic field decides.** The judge (91 s) costs
  more than the elastic solve (59 s) by about a third on a structure where every component
  Stands elastically; the difference is the per-component copies and the certificate on the
  components the elastic field hands over. The certificate alone, 594 s, says what a release
  pass costs when the field cannot decide.
