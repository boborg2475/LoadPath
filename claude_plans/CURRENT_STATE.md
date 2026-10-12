# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-12)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11 and Eigen
3.4.0, the compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced.
Slices 1 (the model), 1b (JSON structure file, `RunCli`, `loadpath_cli`), 2a (the elastic
standing authority), 2c (dry corners at zero load), 4a (the limit-analysis collapse certificate), 4b (the combined authority,
the shear cap, the kinematic anchors), 3 (the cascade) and 5 (the catalogue, the geometry
layer, the structural fuzz) are committed and reviewed, every green-on-arrival row proven by a
recorded mutation (commit messages and ANCHORS). The engine of DESIGN §8 slices 1 to 3 exists
end to end: `IStandingAuthority` (RBSM corner springs, per-component Schur-pivoted LDLT, free
active set, Stands/Overstressed certified, Mechanism a verdict, Undecided fail-closed),
`ICollapseCertificate` (inscribed/circumscribed LP, homogeneous self-dual interior point,
mechanism selection, shear-cap facet rows), `IStructuralAuthority` (every component solved
alone by the elastic authority, Mechanism and Undecided refereed by the certificate,
disagreements counted, the certificate's list the only release set) and `ICascade` (judge,
release certified clusters severing their boundary, crack the top band of over-capacity bonded
joints, repeat; pass stamps and a pass counter on the structure, carried by the file; cracked
joints read dry through `EffectiveProfile`; per-axis utilisations in every reading). Above it,
the producers of slice 5: `data/catalogue.json` (EN 1996-1-1 bed and filled perpend, Gooch and
UPC measured beds, C24 bearing, EC5 screw, nail and bolt, every row cited and labelled, code
flexural tension stored as direct f_x / 3, fastener rows with EC5 slip moduli) with a loader
that converts once and refuses strictly; `FaceContact` and `BuildRunningBondWall` (a 10 × 8
wall is 85 bricks, 152 beds, 75 perpends); and the seeded cascade fuzz over its five invariants.
Anchors pinned from the literature: inclines, Housner, walls, Itasca slope regimes, harmonic
stacks, Paterson–Zwick, Couplet–Heyman at Milankovitch's 0.107478 with hinges at 0°, ±54°,
±54.484°, ±90° and its 21°/22° friction switch, the Orduña pile at λ 0.5005, the 3DEC wedge at
the certificate's associative 31.07°, the cascade on the arch below least thickness, the dry
wall at t / H within 1 % (10 × 8 and 4 × 5), the bonded 4 × 5 wall at its monolith factor
`(t/H)(1 + f_t A_base / W) = 2.971` to 1 %, two screws at twice their published shear and
withdrawal whatever the joint area. Default suite 306 cases in Debug and 322 in Release (the
arch, pile, arch-cascade, fuzz, small-wall and multi-panel factor anchors run only in Release
under `LOADPATH_SLOW_IN_DEBUG`; six `Slow.*` cases run in Release at every slice close, and the
cheap `Measure.Scale.*` rows after them); the `Deferred.*` tier holds the physical wedge pin
(slice 7) and the two hanging-component cases (slice 6), red by design. The design brief
([DESIGN.md](DESIGN.md)) has every §9 question decided and §10 carries the slice-2, 3, 4a, 4b,
5, 2c and 6 rulings.

## Next

1. Slice 2c, **dry perpends settle**: **done** after a cold review and two follow-ups
   (2026-10-11). `DryNoiseFloorRelative` 1e-12 of the component's force scale: a tensionless
   corner opens only below −floor, a dry corner is open inside ±floor and re-closes above
   2 × floor, a closed tensionless corner inside the band reports N 0, a non-finite force scale
   reads Undecided. Every dry wall at rest, 1 × 3 to 10 × 8, settles in two iterations; the
   lime-mortar (cohesion, no tension) walls Stand at rest. Open items, none host-unsafe:
   - **laterally loaded dry walls still read Undecided, and the floor has nothing to do with
     it**: on a 5 × 5 grid of sizes × λx 0–0.3, λy 0–0.15, 147 points stay Undecided (1 × 5,
     2 × 5, 3 × 4, 3 × 5, 5 × 4, 5 × 5 at every λ ≥ 0.05; 10 × 8 and 14 × 10 at λ 0.2) with
     perpend corners carrying real load (3e-7 to 5e-4 of the scale) and slipping; floors from
     1e-14 to 1e-6, slip slack, and a 2000-iteration cap change nothing; the NNLS oracle says
     CanStand for all of them and no point reads a false Stands. This is a stick/slip
     non-convergence of the free active set, the next solver item (2d) after slice 6 opens;
   - the restore step (`RestoreInadmissibleReleases`) re-closes a floor-opened corner on a
     motion gap with no floor check: a corner Open in the full-rank state returns Undecided
     (the 3 × 1 wall at λ (0.8, 0.8) went Mechanism → Undecided), otherwise the corner re-enters
     Slipping and is reopened in a cycle (a light body on a heavy one at mass ratio 1e-12 to
     3e-12 reads Undecided/50 where it read Stands/1; at 1e-13 it reads Stands with the light
     body's support corners open, floating). Fails closed; untested;
   - one regression independent of the floor value: the 4 × 2 wall at λ (0.30, 0.15) went from
     Stands/14 to Undecided/50, and still reads Undecided/50 after the hysteresis (dev
     measurement, 2026-10-11): it belongs with the lateral non-convergence above;
   - an unloaded component (zero gravity, no loads) has floor 0 and `<=` opens every dry corner,
     reading Stands with all corners open; `<=` → `<` survives the suite, so the boundary is
     unpinned;
   - in the overflow case the floor is +inf and the reading clamp then zeroes every tensile
     reading on tensionless corners (a 2e154 uplift reads Undecided with bed utilisation 0), so
     "readings filled from the overflowed solve" is not true for tension; fails closed; skip the
     clamp when the floor is not finite;
   - the floor's size is pinned from above by the end-push scaling test (floor 1e-9 reddens it)
     and from below by nothing: a smaller floor only brings the at-rest chatter back at larger
     sizes (noise reaches 1.5e-16 of the scale on the 14 × 10 wall, so 1e-12 keeps about four
     orders of margin that shrink roughly linearly with size).
   **Slice 6a, the scale measurement: done** (2026-10-12, after a cold review and two
   follow-ups).
   `BuildWarehouse` (four walls on one ground, posts, beams, planks; openings by removal),
   `WallSpec` origin / orientation / shared ground, the `Measure.Scale.*` harness, the table in
   [SCALE.md](SCALE.md): the shed (420 pieces) judges in 0.5 s, the half warehouse (1,822) in
   91 s, the 1,296-brick wall never finishes its first dense solve (killed at 25 min, 1.07 GB).
   First finding fixed in the round: an ungrounded component with a net force falls rigid, so
   a floating wall releases as one cluster (it was cut into bricks, or Undecided on the bonded wall).
   Left from the round: a grounded mechanism that severs a dry joint can still record
   `Tension inf` as the reason (the cascade copies the reading); an ungrounded component with
   zero net force (a pure couple) still goes through the general selection and can list
   hinges; the rigid-fall motion is a pure translation even when the net load misses the
   centroid; the general selection near t* ≈ 0 does not converge on a bonded floating or hanging wall
   (the bonded 10 × 8 with its base course removed read Undecided with 0 passes in both builds
   before the rigid fall; a 3 × 8 ec6 wall hanging from a dry ceiling still does) and splits a
   dry pair wedged under a ceiling into singletons: the least-joint-movement selection ruled in
   DESIGN §10 is still owed (`Deferred.Slice6.*`); there was no Debug/Release difference (the
   review rebuilt the pre-fix commit in both); the warehouse generator removes opening bricks
   after building them (O(interfaces) per brick); the shed's frame does not bear on its walls
   and the corners are not bonded (five components by construction). From the review:
   the rigid fall is now gravity-only (a loaded floating pair read one cluster where the upper
   brick lifts or slides off alone; fixed red-first), a rounding residue in the net force
   flipped a squeezed floating pair to Collapses (fixed by the gravity-only rule; the relative floor the dev added in that branch was dead
   by construction, the net magnitude there being exactly 0 or 1, and was removed), and a
   floating component carrying any non-zero `BodyLoad` takes the general selection and can be
   cut (0.01 N on the bonded 4 × 4: 2 clusters, 8 failures) until the least-joint-movement
   selection exists, a `BodyLoad` with a NaN force reads as no load on the rigid-fall test
   (`Length > 0` is false for NaN; the structure validation upstream is what should refuse it,
   check it does), the fall motion and the origin's base
   beds had no tests (added), the `ExpectedLayout` oracle copies the post/beam/plank formulas
   from the generator (brick positions and the areas are independent), `Beds`/`Perpends` keep
   removed ids after openings (the header says so now), the spec is not validated (`Bays ≤ 0`
   gives a NaN span), `OriginM.Z` and `GroundZM` overlap, the test helper undercounts openings
   at a wall end, cascade determinism is unchecked, and the never-finishing rows moved from the
   `Slow.` tier to `Measure.*`. Now **6b** (API.md "6b", DESIGN §10).
   **Stage 1a, the factorisation: green, in review (2026-10-12).** `FactorKind::Blocked`, a
   blocked right-looking pivoted LDLᵀ (dpstrf style) behind the same rank test; the rank-1
   factor stays as `FactorKind::Reference` for one slice as the referee and is then removed
   (log that removal when 6b closes). Eigen's own dense LDLT was tried and rejected (it does
   not reveal rank). The referee property compares 6,000 seeded draws under the §10 rulings:
   certified states exact, Mechanism/Undecided may trade on ≤ 0.1 % (measured 2), readings
   and flags compared only on certified states; the 10×-looser-tolerance mutation is caught
   only by the trade budget (32 against 6), so the budget must not be loosened. Timings in
   SCALE.md (the 32 × 40 wall 24 s, was unfinished; the shed judges in 136 ms, was 614).
   The cold review (2026-10-12) confirmed the factor against the reference on every matrix the
   Release suite factorises (40,231 factorisations, 0 rank disagreements) and found the
   property never ran the panel code (the seeded draws are at most 30 DoF, the panel is 64
   columns): fixed the same day by `Statics.Factor.BlockedAgreesWithTheReferenceAcrossPanels` (two
   8 × 10 walls, Mechanism and Stands, no trade allowed, plus 120 seeded dry walls, 116 of
   them above one panel; Slow in Debug) and a mechanism-motion comparison within 1e-6 when
   both kinds read Mechanism, so the tolerance regression now has two detectors (32 trades
   against 6, and 18 bodies moving differently); the skipped-trailing-update and
   panel-limited-pivot mutants redden it (ANCHORS rows). Still open from the round: the rank landing exactly on
   a panel boundary (64, 128) is unpinned (`LargestPivot` resets per panel; a fixture of
   reference rank 64 would pin it); the two incidental guards of panel faults outside the
   property are `ArchAnchorsTest.cpp` (Orduna hollow pile, catches a skipped trailing update)
   and `StackAnchorsTest.cpp` (diamond Stands, catches a panel-limited pivot search), so
   neither may be loosened without knowing it guards the factor; the trade count (2 of 6,000)
   is deterministic across Release runs (Eigen single-threaded here); a Mechanism draw's
   readings differ by up to 0.79 relative between the two factors (printed, not asserted);
   the budget bound of 1.0 s on the 16 × 20 wall is a timing assertion on this machine.
   **Stage 1b-i, the sparse path: green, in review (2026-10-12, HEAD 0d13a29).**
   `StandingOptions::DenseDofLimit` (600; the largest default-suite component is 510 DoF);
   above it a sparse `SimplicialLDLT` under AMD, accepted only when the factor succeeds, every
   pivot is finite and above tolerance, and five inverse iterations put the Rayleigh quotient
   above `PivotToleranceRelative × max |D|`; anything else is redone through the blocked dense
   factor (DESIGN §10: the unpivoted pivot test alone missed five rank-17 bridges in 1,000).
   The 32 × 40 wall solves in 0.19 s (was 24.8 s dense). Open from the round: the
   eigenvalue-estimate mutation is caught only by the bridges at 5 trades against a budget of
   1 (thin); the agreement property's 24 walls yield only 4 certified draws (most dry and
   lime walls under λ up to 0.5 read Mechanism or Undecided), so the wall half does little
   work on readings; one bridge (draw 443) still trades Mechanism/Undecided between the paths
   by rounding drift on a 30-iteration path, inside budget; the five iterations and margin 1
   are the smallest pair measured, nothing separated the nine pairs swept.
   The cold review (2026-10-12) found four blocking items, all fixed the same day: the two
   stage-1a factor timing tests ran the sparse path unknowingly (now forced dense: blocked
   0.56 s, reference 3.3 s on the 16 × 20 wall); the agreement property cost 12.7 s with its
   wall half idle (now 12 ec6 walls under λ and 12 dry walls at rest, 24 of 24 certified, about
   2.5 s with the 1,000 stacks); the limit boundary and the estimate were unpinned
   (`Statics.Sparse.TheLimitBoundaryIsInclusive`, an 8 × 12 wall of exactly 600 DoF, and
   `HiddenPivotsAreCaughtByTheEstimate`, bridges 423, 556, 708, 715 with no trade allowed;
   all three estimate mutants now redden it); docs behind the amendment. Still open: the pivot test is pinned by no mutant once the estimate exists (the
   estimate alone passes the suite; a start vector orthogonal to a soft mode with visible
   pivots is the case only the pivot test catches); the estimate's constructed adversary (a
   soft mode orthogonal to the ±1 start with a second mode within 10³ of it accepts at λ_min
   1e-3 of the bar; worst consequence an equilibrium residual of 1e-3 of the load, never a
   false Stands since the unpivoted LDLᵀ is backward stable; two start vectors or an
   eigen-residual check would close it for five extra solves); the symbolic analysis is redone
   every active-set iteration though the pattern never changes (`analyzePattern` once per
   `SolveProblem`); the zero-norm guard in the estimate is unreachable once the pivots pass;
   `K * V` in the estimate relies on both triangles being stored (a lower-only assembly would
   need `selfadjointView`). Mutant builds under %TEMP% are not recompiled by MSBuild (MSB8029):
   build mutants outside it.
   **Stage 1b-ii, per-component results and no copies: green (2026-10-12), awaiting the
   stage 1b review.** One `Solve` per judge, the certificate consulted through a component
   filter on the original structure, `Structure::Components()` cached. Open: the cache is
   mutable state behind a const method (one `Structure` queried from two threads races; no
   thread-safety promise exists anywhere in the library, log it when one is made).
   Found by the 1b-i rows: the base-course cascade on the 32 × 40 wall costs 50.7 s because
   the floating 1,215-brick remainder is rank-deficient, so the sparse factor hands it to the
   dense blocked factor (about 24 s per solve). An ungrounded component is a mechanism by
   connectivity before any factorisation; the elastic authority could read it (and its
   rigid-body null space) without factorising, as the certificate's rigid-fall rule already
   does. Not in 1b; log for stage 2 or the certificate's next item.
   **Stage 2, the event region: green (0fc14a5), reviewed, fix round in progress.** Done
   since the review: options are part of cleanliness (`AuthorityResult::Options`, exact
   comparison, e92c4df); the direct contract test of `Solve`'s filter; the warm Measure rows.
   The cold review (2026-10-12) sent it back on two defects, both with red tests on branch
   `stage-2-fix` (989b5a0) and dev in progress: a verdict from a diverged copy of a structure
   was matched and reused (ruled: a lineage token minted at construction and on every copy,
   recorded in the verdict, DESIGN §10 second amendment), and invalid standing options with
   nothing dirty returned the copied verdict (ruled: `Judge` validates options before
   matching). The same branch pins the first-pass Previous of `Run`, the cap re-judge, a
   moved load point, a load on a dead body with nothing dirty, a ground contact added to a
   loose brick and `LastTouched` of an unknown id, strengthens the fuzz guard (reused
   components exact, Mechanism and Disagreements compared, a cascade-parity variant: 32 draws,
   251 components, 68 reused) and removes the red-phase scaffolding. Decisions from the dev
   round ratified in the amendments (live-neighbour stamps, free-end rule for ground readings,
   exact load tuples, fail-closed on an uncopyable reading). Measured (SCALE.md): the
   warehouse judges in 0.49 s; its warm post cascade (4 of 5 components reused) is 254 ms
   against the 200 ms target, the remainder the frame's own certificate (about 225 ms),
   which is stage 3's.
   **Stage 1b-iii, the certificate's sparse normal equations: green and merged (b72c262);
   the cold review (2026-10-12) found no correctness defect** (assembly bit-identical to the
   dense matrix on 1,268 matrices) and blocked on docs and the shed ceiling, both fixed; it
   asked for a filter test naming two components (kills a "stop after the first chosen
   component" mutant), the cache test extended to AddBody/AddProfile/MarkPass, and a timing
   subject of about 310 bodies instead of the shed (ratio 0.45–0.59 against the shed's 0.8). Uncaught mutants recorded: the retry's LargestDiagonal without the coupling
   (a 1e-12 shift), the NaN-only positivity scan. `StandingComponent::ReadingSlots` is read by
   nothing in the library. The three asked-for tests landed in f872105 (the sparse timing
   subject is now a 310-body wall, ratio 0.42–0.47 against a 0.67 threshold).
   **Next: stage 3, the certificate at scale (DESIGN §10 "stage 3 opening", SCALE.md
   profile): 3a the structured cap-site factor (exact), 3b the stall rule, 3c the coupling out
   of the factor with refinement, 3d a verdict-only option last; panels contingent; the
   `Measure.Scale.*` rows re-run after each stage into SCALE.md.** Then the 6b close: remove
   `FactorKind::Reference`, the overview artifact, the Slow tier and cheap Measure rows.
 **Slice 8,
   the Unreal adapter, is on hold by owner directive (2026-10-11): do not start it.** The
   remaining slice-2 anchors (item 2) can run alongside slice 6.
   Slice 3 and 4b items still open (none host-unsafe; the unsafe one, the wedge band, is under
   Deferred):
   - the overflow guard (`HighestPass() == UINT32_MAX` → InvalidInput) runs only at entry; a run
     starting at `UINT32_MAX − 1` that needs two counted passes computes pass 0 on the second,
     where Crack/Sever/MarkPass refuse and a release removes bodies with no stamp or record;
     practically unreachable, the fix is the same guard at the top of the loop; the
     InvalidInput verdict at the limit also reports `FirstPass 0, LastPass UINT32_MAX`
     (cosmetic, computed before the guard);
   - the cap re-judge returning InvalidInput leaves `PassCapReached` with empty readings and
     components (fails closed, undocumented);
   - the file reader's own `crack_pass ≤ sever_pass` check is now masked by `Sever` refusing
     the same (defence in depth, no test fails without it);
   - a severed joint's `FailedInterface` axis and ratio are the elastic reading's, which on a
     mechanism means little (the arch crown hinge reads `Compression 2.4e-5`); derive the axis
     from the certificate's hinge motion (opening = Tension, slip = Shear) or pin the current
     behaviour; a mutant that always writes `None` survives the suite;
   - the arch's two 0.484° slivers are released as single-voussoir clusters; whether a hinge
     needs a minimum rotation is still open, and this is the measurement against it;
   - the mechanism selection ignores finite positive shear caps (harmless at `t* = 0`; for
     `0 < t* < 1e-9` it reads Undecided, which fails closed);
   - the cascade's scans
     are quadratic (`IndexOf`/`Contains`, union-find without path compression, the release
     scanning every interface per cluster, `FindReading` O(I) per sever, the crack band
     O(K·I·|component|)): the RBSM's per-component states are exposed since 1b-ii; the
     cascade's own scans are still to be indexed;
   - the per-component certificate InvalidInput branch in CombinedAuthority.cpp is
     unreachable today: fail closed, untested;
   - a cap-0 corner on the inscribed side still uses the inner tension offset min(ft, c/μ):
     conservative, not unsafe;
   - the reverse hinge property only checks that a cluster of listed bodies touches a hinge;
   - the pile anchor runs on a reconstructed bond (bed joints only); verify against the paper's
     figure when it can be read;
   - B-ARC-09, the voussoir-count sweep, is not pinned (expect convergence to 0.107478 from at
     or below);
   - the 4a items still open: the selection gate is `FeasibilityTolerance`; `t_out` is the
     circumscribed converged primal, not the dual objective; `Iterations` excludes the
     selection solve's count; HiGHS stays slice 7's referee question.
   The remaining slice-2 anchors (item 2) can run alongside slice 2c.
2. Slice 2b, the remaining slice-2 anchors (BENCHMARKS groups 2 and 3): B-WAL-03 (wall slides at
   λ = μ), B-STK-04 (constant-step stack), B-WDG-02, B-ROK-02, F-CHC-01, F-LIB-01/02, F-EMI-01,
   the 5-diamond (hits the 50-pass cap today, logged below), and the arch and pile rows that
   API.md "Anchors slice 2 must pin" lists but BENCHMARKS marks slice 4 (B-ARC-01/06 Couplet–
   Heyman, B-ORD-01 Orduña): attempt them with the elastic no-tension field in 2b and move the
   ones it cannot reach to slice 4 with the reason recorded, as the wedge was. Rows with inputs
   still to retrieve: B-COR-01 corner geometry, F-PSY-01 friction angle, F-FOR-01 and F-STR-01
   pier geometry (Portioli 2012), B-DER-01 wall geometry, B-SW-01 exact limit load (graphical
   only), F-RON-01 panel thickness, B-FTL-01 layout. B-BUR-01 needs a soil-pressure driver;
   F-PYR-01 needs a zero-extent contact; group 9 runs only in the host adapter's suite.
3. `Scripts/Test.ps1` excludes `Slow.*`, `Deferred.*` and `Measure.*` from the default run (2026-10-11; the
   arch and pile anchors carry `LOADPATH_SLOW_IN_DEBUG`, Slow. in Debug only). Still open: any
   caller `-tc` or `-tce` is passed straight through and drops all three exclusions.

## Deferred

- B-WDG-01, the 3DEC tetrahedral wedge: the certificate pins its own associative switch at
  31.07 deg (`Certificate.Anchor.ItascaWedgeSlidesBelowItsAssociativeCriticalFrictionAngle`,
  G = 64, reads 31.080); the physical value is 33.36 deg, and between 31.1 and 33.4 deg the
  combined authority reads Stands (elastic Undecided, so no disagreement is counted and the
  host gets no signal) where the physical wedge slides. `Deferred.Slice7.*` holds the physical
  pin for the non-associative referee. The elastic test stays `doctest::skip()`: the kn = kt
  corner-spring field releases every corner near 40 deg or finds a squeezing set near 31 deg.
- Statics.Authority.InsertionOrderChangesNoReading compares with a 1e-12 relative tolerance, not
  `==` (orchestrator 2026-10-11): unknowns are ordered by body slot, so a different insertion
  order changes the assembly and pivot order and the last bit. Bit-identity is pinned only for
  repeated solves of the same structure.
- Slice 2 behaviour still with no test after the characterisation round (2026-10-11,
  `tests/StandingSolverTest.cpp`): the 1e-9 stick hysteresis (margins 0 and 1e-3 both survive;
  no fixture leaves a slipping corner's trial shear within the margin of capacity, and on a single
  block the slip and re-stick conditions exclude each other); the determinate side of the
  indeterminate-gap rule (a mutant that keeps every open corner open in any neutral solve
  survives); the `D` mapping on the neutral motions.
- Undecided probe results: the two causes found on 2026-10-11 (open corners re-closed stuck,
  status-only cycle detection) are fixed. A single brick at μ 0.6 and μ 5 over λx 0..4 ×
  λy 0..2 (step 0.125, 561 solves each) went from 30 + 8 Undecided to 0, and the Stands counts
  (22 and 351) match the rigid-block answer exactly; the stack and bridge grid counts after the
  narrowed contract are in "Next" 1. The cost of narrowing (third review): Undecided on
  provable collapses went from 23 to 119 on the μ 0.6 grid and 4 to 361 on the μ 5 grid, and
  619 of 3,000 realistic random stacks and bridges (μ 0.4 to 1.0) read Undecided where
  collapse is provable; all fail closed and belong to slice 4.
- On the same μ 0.6 grid, 25 biaxial points just past the sliding threshold (for example
  λ = (0.625, 0.5)) read Mechanism with a twist about z and a translation off the load line,
  before and after the review-round fix. All four corners are slipping there; the twist
  (Rz about 7% of the corner displacement) comes from non-parallel held directions inherited
  from the elastic trial, and the collapse mode is a pure translation. The verdict and the body
  list are right, the shape is not the collapse mode, and no test asserts it. Slice 4's
  collapse certificate owns mechanism shapes; API.md's "a sliding block reads a pure
  translation" holds for uniaxial or symmetric loading. Readings on such a mechanism component
  can show slipping corners up to about 5% over μN because the held force came from an
  earlier pass's N; slice 3 reads standing components only.
- The held-force magnitude loop still converges at roughly μ per pass where friction at one
  joint sets the normal force at another: the brick pushed into a wall takes 22 passes at
  μ = 0.3, 37 to 38 at μ = 0.5 and hits the 50-pass cap at μ = 0.7, λ = 0.8, matching
  log(1e-12) / log(μ). The `d capacity / d N` term that would remove it is non-symmetric, so it
  cannot go into the LDLT. Fail-closed Undecided; no test drives it yet. The single-brick slow
  convergence logged before (μ = 5, λ = (0.5, 1.0) and 25 more grid points, 5 at μ = 0.6) was
  the held-direction lag and is gone with the transverse tangent: λ = (0.5, 1.0) stands in 5
  passes and no single-brick probe point reaches the cap (31 at HEAD).
- Slice 2's `IsGapIndeterminate` recomputes the motion scale per open corner per null vector
  (O(corners² · nullity)); fine at tier-1 sizes, hoist it before slice 6.
- Eigen's no-exceptions `EIGEN_TRY`/`EIGEN_CATCH` makes MSVC emit C4702 from the backend, which
  `/external:W0` does not cover; `src/EigenInclude.h` wraps the include in a pragma. Any new
  Eigen include goes through it.
- A second compiler (clang-cl ships with this Visual Studio) as a CI-style check for
  portability; not set up.
- The test script's run is single-configuration; a `-Both` switch for Debug and Release would
  catch optimiser-dependent float differences early.
- Slice 1 behaviour left untested by test-expert, so unimplemented or unpinned:
  - `AddProfile` still accepts zero and negative stiffness; slice 2 refuses them at `Solve` as
    InvalidInput when the profile is on a live interface (pinned by the slice-2 tests).
  - A mass of `-0.0` is accepted (`!(m >= 0)` lets it through); no test says either way.
  - `Dot`, `Cross`, `Length` and `IsFinite` are exercised only through `AddInterface`.
  - Two normal guards are unreachable defence in depth: the NaN branch of
    `!(LargestComponent > 0)` (the finiteness check runs first) and the stored-length check
    `|len - 1| <= 1e-9` after scaled normalisation (a scaled vector's length is in [1, sqrt 3], so
    the division lands within ulps of one). Neither can be driven red by a test.
- Structure file, open rulings from the 1b review (2026-10-11; the format now carries state,
  stamps and an optional interface name, slice 3): the reader rejects an unknown state string,
  a non-integer pass and a stamp/state mismatch (a ten-row table pins them).
  A non-object array element (`"bodies": [3]`) is refused with a misleading `missing name`
  message rather than "not an object" (fails closed, unpinned). Any duplicate in a caller-built
  name vector, generated or explicit, writes a file the reader refuses. A UTF-8 BOM is refused
  as "not a JSON object" (RFC 8259 permits ignoring it; Notepad writes one). Invalid raw UTF-8
  and raw-encoded surrogates pass through. Error messages embed decoded names raw, so a name
  with a newline makes the CLI's Err span lines. `cli/Main.cpp` opens the path through the ANSI
  code page. The writer iterates `ProfileId{Index, 1}` and so relies on profiles never being
  removed.
- `RemoveBody`, `InterfacesOf` and the live counts scan every slot (O(n)); fine for slice 1,
  but a per-body adjacency list is needed before a solver calls `InterfacesOf` in a loop
  (slice 6, 15,042 joints).
- Doctest runs with `DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS`, so `REQUIRE` does not
  abort the test case: a test that dereferences or indexes after a failed `REQUIRE` keeps going
  and can crash the runner instead of reporting. Sweep the suite for that pattern (for example
  `CliTest.cpp`'s `REQUIRE_FALSE(Loaded.Ok())` and the `REQUIRE`s in `MakeHandBuilt`) and add an
  early return where the next line depends on the check (flagged by test-expert, slice 1b).
- Slice 1b items left open after the second ratification round:
  - Underflow reads as the nearest value only because MSVC's `from_chars` stores it alongside
    `result_out_of_range`. The standard says the value is left unmodified; there the reader's NaN
    preset turns underflow into "number out of range" (fail closed, but against the ratified
    rule). Nothing on MSVC can drive the preset red; check it when a second compiler is set up.
  - The writer's generated names (`b<Index>`, `p<Index>`) can collide with a real name in the
    same vector (`BodyNames = {"b1"}` with two bodies writes `b1` twice, and the reload refuses
    the duplicate). No test and no ruling.
  - Loaded `InterfaceNames` holds one empty string per interface as ratified; the count is
    pinned, the empty content is not.
- Slice 2a internals left unpinned after four reviews (2026-10-11; a mutant survives the default
  suite for each): the restored corner's held magnitude (zero survives; only 2 of 4,488 probe
  points restore a corner with a positive full-rank normal) and held direction (flipping
  changes only a mechanism shape at bridge μ 0.6 (0, 0.75)); transverse stiffness kt instead
  of 0 on re-entry; a cohesive corner re-entering slipping instead of stuck (reachable at 278
  points of a cohesive bridge grid, flips 6 from Undecided to Mechanism); the Undecided-readings
  source (always reading the current pass survives; bridge μ 0.6 at (0, 0.875) has residual
  0.010 W at HEAD and 1.50 W under the mutant); the no-full-rank-yet restoration branch
  (unreachable: any component with a stuck interface is full rank on pass 1).
- Slice 2a readings caveats (informative only; slice 3 reads standing components, API.md):
  mechanism readings can show tension on a slipping dry corner that re-entered with zero held
  force (0.08 m stack, μ 0.6, λx 1.5: N = −0.43 N on the bed, −2.10 N on the joint), and both
  interfaces then read +inf on the tension axis, so the `bNonFinite` term in `IsOverstressed`
  is reachable there; a component that is InvalidInput through its assembled stiffness returns
  InvalidInput for the whole solve with `InvalidInterface` unset.
- `Scripts/Test.ps1`: any caller `-tce` (for example a future `-tce="Slow.*"`) silently includes
  the `Deferred.` tests; the "skipped" trigger also counts `doctest::skip` cases.
- `tests/StaticsOracleTest.cpp` draws bridge half-spans from 0.10 m, below the 0.1075 m that
  keeps the two legs apart, so about a quarter of bridge draws interpenetrate by up to 15 mm;
  both sides see the same model so the property is unaffected, but start the range at 0.1075
  or record why not. The oracle now lives in tests/StaticAdmissibilityOracle.h; remaining spec from the fourth review:
  make the two bridge `Statics.Combined.ThreeBrickBridge*` cases CHECK `Oracle == CanStand` as a
  precondition (`MakeBridge(5.0, 0.12, 0.0)` at (2.5, 0.875), (2.375, 0.875), (2.25, 0.875)
  reads CanStand with inner residual about 1e-16 at μ 5 and 3.5).
- Slice 5 items left open after its review (2026-10-11), none host-unsafe:
  - the cascade fuzz never cracks a wall joint: over the 40 Release draws every crack is in a
    weakened stack, the 8 wall draws release only a whole-wall overturn and loose bricks, and
    the termination invariant (`Passes ≤ live interfaces at entry`) cannot fail on a wall with
    ≥ 16 live interfaces against the 16-pass loop cap; the fix is a weakened-wall family
    (ec6 rows × 10^U[−3,−1], in-plane lateral along x) with coverage guards for one crack in a
    wall draw and one wall cluster smaller than its component (the review's T5);
  - invariant (i) reads the release list through the combined authority rather than the raw
    certificate: equivalent by construction, the same code path rather than an independent
    oracle;
  - `BoxContacts.cpp` keeps an unreachable branch (two contact axes → nullopt): on any axis
    with |gap| ≤ tol the overlap on that axis is ≤ tol, so it can never serve as U or V for
    another axis;
  - data caveats the rows disclose: the lime-mortar row carries the M10 GP compression of
    8.93 MPa; the fasteners bear on the C24 row's 3.75 MPa and 8.2 GPa/m while their examples
    are in GL30c; the nail row's withdrawal 0.42 kN is the 3.1 mm nail's datum; the timber
    row's stiffness divides by Ex 3.2's 45 mm joist width because the example gives no other
    bearing dimension; EN 1995-1-1 Table 7.1's nail expression and the §7.1(3) ×2 were matched
    through secondary sources (Swedish Wood Vol. 2, the COFORD handbook), the standard itself
    unopened; the timber bearing friction 0.3 is the Wood Handbook range's low end matched
    through a search snippet, the PDF unopened;
  - `FastenerProfile` has no guard for `Count ≤ 0` or `AreaM2 ≤ 0` (a zero area gives inf or
    NaN strengths, which the Structure refuses, so it fails closed one step later);
  - a withdrawal (axial) stiffness differs from the bearing stiffness in a real fastened joint;
    the profile carries one normal stiffness, the bearing member's, both ways;
  - a mean fastener row waits for a cited mean (Blass & Colling's ratio is characteristic).
  - after the review round: `FaceContact` still accepts a negative half extent (only non-finite
    input is refused); the loader refuses unknown keys inside rows but ignores unknown keys at
    the document level; `BuildRunningBondWall` keeps going after a refused body or interface
    with `bComplete` false and no phantom interface ids, but `Courses` still lists a refused
    brick's id and `Ground` a refused ground's; the old `SmallBondedWallCollapseLambda = 8.5`
    anchor is slack now that λ_c is 2.97 (the tight anchor beside it is the bound).
  - from the follow-up review: `FastenerProfile`'s `Count` is per interface (one shear plane),
    so a multi-plane fastener with withdrawal is over-counted on withdrawal (documented in the
    header; no row today has both); the screw's A1:2008 withdrawal uses l_ef 44 mm, below the
    6 d = 48 mm minimum penetration the clause is recalled to require (noted in the row; verify
    against the standard).
