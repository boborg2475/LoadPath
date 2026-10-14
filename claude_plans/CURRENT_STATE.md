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
withdrawal whatever the joint area. Default suite 391 cases in Debug and 435 in Release after
slice 2b and the cohesive-wedge check (325 and 347 at the 6b close) (the arch, pile, arch-cascade, fuzz, small-wall and multi-panel factor anchors
run only in Release under `LOADPATH_SLOW_IN_DEBUG`; twenty `Slow.*` cases run in Release at
every slice close, and the
cheap `Measure.Scale.*` rows after them); the `Deferred.*` tier holds the physical wedge pin
(it belonged to slice 7, which the owner dropped on 2026-10-12; the pin stays as the record of
the non-associative answer) and the two hanging-component cases (slice 6), red by design. The design brief
([DESIGN.md](DESIGN.md)) has every §9 question decided and §10 carries the slice-2, 3, 4a, 4b,
5, 2c, 6a and 6b rulings.

## Next

1. Slice 2c, **dry perpends settle**: **done** after a cold review and two follow-ups
   (2026-10-11). `DryNoiseFloorRelative` 1e-12 of the component's force scale: a tensionless
   corner opens only below −floor, a dry corner is open inside ±floor and re-closes above
   2 × floor, a closed tensionless corner inside the band reports N 0, a non-finite force scale
   reads Undecided. Every dry wall at rest, 1 × 3 to 10 × 8, settles in two iterations; the
   lime-mortar (cohesion, no tension) walls Stand at rest. Open items, none host-unsafe:
   - **slice 2d, done 2026-10-12 (c4a372a, closed at 1b1e655 after a cold review; DESIGN §10
     "slice 2d opening", "dev notes", "cold-review rulings")**: the laterally loaded dry walls
     read Undecided because a re-closing corner carried its
     built-up slip as elastic shear and was reopened; a slip-releasing corner that re-closes
     above μN now enters Slipping at the cap, and the 160-point grid all Stands (mean 8.1
     iterations, worst 30) with zero oracle refutations; the pins were regenerated under the
     pins ruling. Left from the round: the grid test runs the oracle on all 160 Stands and
     is split: the convergence half with the own-field check runs in the default tier
     (2.2 s Release, `LOADPATH_SLOW_IN_DEBUG`), the NNLS on a 24-point subset in `Slow.` (61 s);
     the own-field property re-derives every pinned Stands from its readings in 1 s; the mutant "apply the rule to cohesive corners too" is caught by the lime-wall
     scope test (e4366ec); the regeneration's oracle audit's one CannotStand among 1,420 old
     Stands is the 8 × 10 ec6 wall under λ (0, 0.5), a bonded wall the compression-only NNLS
     oracle cannot model (the field is sound; the oracle gains a scope guard in the fix round);
     the 50-iteration cap is close near the stability limit, not at size (random dry walls at
     μ 0.99 and λ (0.38, 0.28) took 46; over the cap fails closed to Undecided); under the scope mutant the lime fuzz also
     reddened on a bonded ec6 stack of 5 (seed 300281570), which means a corner of a joint with
     effective cohesion re-closed with trial shear above μN: check whether that joint was
     Cracked (effective tension zero, cohesion zero, so legitimately slip-releasing) or intact; the restore-step Undecideds below are now more
     visible (145 of the 196 Mechanism-to-Undecided pin trades); **the same cycle hits
     cohesive tensionless (lime-mortar) perpends**: the 4 × 5 lime wall at λx 0.8, the 5 × 5 at
     0.8 and the 10 × 8 at (0.2, 0.1), (0.5, 0.1), (0.8, 0) and (0.8, 0.1) read Undecided at
     50, and all Stand in 5 to 9 iterations if the dry rule is applied to them, which the
     scope ruling forbids (`Statics.Lime.AReclosingCohesiveCornerStaysOutOfSlipping`); a
     re-closing cohesive corner got its own ruling in slice 2e;
   - **slice 2e, done 2026-10-12 after a cold review and a follow-up review (red step d089f5a, rule
     8db9af3, fix red step 9da89f8; DESIGN §10 "slice 2e opening", "dev notes", "cold-review
     rulings", "fix-round dev notes")**: a cohesive
     tensionless corner that re-closes for the second or later time in one solve resets its
     tangential reference to the current offset (carried as a held pre-force kt·Ref in the
     load); the seven lime points Stand in 8 to 16 iterations, the 140-point lime grid goes
     from 52 Undecided to 5. The review found the bit-identity claim overclaimed (12 converged
     lime points move, accepted and pinned) and the guard observable and harmful (402 of 404
     verdict changes for the worse), so the fix round drops the guard, skips the pre-force when
     the reference is zero (signed zeros), pins the moved points, the two walls the guard kept
     from standing and the weak-perpend Overstressed, adds a solve-twice test and the grid's
     Stand/Mechanism counts, and updates API.md. Left from the review: the five 10 × 8 lime
     exceptions at (1, 0.15) and λx 1.2, and the one-course three-brick wall with c 1 kPa
     joints under a 30 N upward load at the middle brick's centroid (and the 1 × 5 alike) at
     λx 0.3, exit Undecided through
     `RestoreInadmissibleReleases` because the closing corner was Open in the full-rank state
     and the step has no trial shear to decide slip; investigated and left as it is (DESIGN
     §10 "the restore step's refusal on cohesive corners"): the certificate certifies all six
     as Stands and the combined authority already returns Stands, while every candidate rule
     traded fast refusals for 50-iteration solves or certificate-refuted Mechanisms;
   - **the cohesive-wedge Mechanism, done 2026-10-13 after a cold review (DESIGN §10 "the
     cohesive-wedge Mechanism (opening)", "dev notes", "cold-review rulings")**: an elastic Mechanism whose null-space motion slides
     open cohesive corners against at least as much cohesive dissipation as the load supplies
     reads Undecided (the certificate then answers, as for any Mechanism); no host verdict
     changes, the hunt's refuted Mechanisms go from 856 to 195 per 100,000 (the remaining ones are
     the non-associative gap); `>` to `>=` at P = Dc is unpinned (a ±200-ulp scan never lands on equality); a refused
     component reports the last full-rank readings, so its maximum utilisation can read +inf
     where the Mechanism path read a finite value (358 of 1,579 in the hunt, 168 of them on
     Collapses components whose `FailedInterface` then carries the inf), next to the existing
     "`Tension inf` as the reason" item;
     also logged from the probe: 5 of 458 dry random-wall Mechanisms
     carry a Stuck corner moving up to 2.4e-5 of the motion scale;
     readings at the iteration cap on a corner reset in the last pass are out of equilibrium
     (the `HeldN` pattern, untested); restore-step re-closures neither count nor reset by
     ruling, pinned on two random-hunt walls (`ARestoreStepReclosureNeitherCountsNorResets`); the
     follow-up review's seeded hunt (100,000 lime walls with held loads, 1.3 s per 1,000 draws)
     belongs in `Slow.` as an own-field property, the only generator that reaches the restore
     path with a reference; the zero-reference skip in
     `AssembleLoad` survives its mutant (no reading moves); readings at formerly cycling
     points are path-dependent between equally admissible rules (accepted);
   - the restore step (`RestoreInadmissibleReleases`) re-closes a floor-opened corner on a
     motion gap with no floor check: a corner Open in the full-rank state returns Undecided
     (the 3 × 1 wall at λ (0.8, 0.8) went Mechanism → Undecided), otherwise the corner re-enters
     Slipping and is reopened in a cycle (a light body on a heavy one at mass ratio 1e-12 to
     3e-12 reads Undecided/50 where it read Stands/1; at 1e-13 it reads Stands with the light
     body's support corners open, floating). Fails closed; untested;
   - an unloaded component (zero gravity, no loads) has floor 0 and `<=` opens every dry corner,
     reading Stands with all corners open; `<=` → `<` survives the suite, so the boundary is
     unpinned;
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
   **Slice 6b, scale: done (2026-10-12; DESIGN §10 from "stage 1a opening" to "slice 6b close
   opening", API.md "6b", SCALE.md).** Nine stages, each cold-reviewed, with the three
   referee options removed at the close (branch `slice-6b-close`; its cold review blocked on
   the generated pins fixing Undecided iteration counts, fixed under the pins ruling; the
   follow-up review passed with nothing blocking; merged). What
   changed: a blocked right-looking pivoted LDLᵀ (dpstrf style) behind the same rank test;
   a sparse `SimplicialLDLT` above `DenseDofLimit` 600 accepted by pivots plus a five-step
   inverse-iteration estimate, redone dense otherwise; one `Solve` per judge with
   per-component results and no copies; the event region (revisions, `LastTouched`, a lineage
   token, options as part of cleanliness, clean components copied from the previous verdict);
   sparse certificate normal equations above `DenseRowLimit` 600 with the structured cap-site
   factor, a stall rule (window 20, factor 0.5) and the load-factor coupling as a border
   eliminated last. Measured at the close (SCALE.md): the warehouse (3,695 pieces) judges in
   0.51 s (was 91 s for the half warehouse at 6a), its full certificate 75 s (was 1,566 s at
   1b-i), its warm post-removal cascade 140 ms against the 200 ms target; the 32 × 40 wall's
   elastic solve 0.19 s (was unfinished) and certificate 34 s; the half warehouse's
   certificate 59 s (was 594 s). Stage 3d (a verdict-only option) was dropped as unneeded.
   Carried from the stage reviews, none host-unsafe:
   - elastic factor: a rank landing exactly on a panel boundary is pinned at 64 and 128
     (`Statics.Factor.ARankOnThePanelBoundaryReadsItsNullSpace`, 2026-10-12: tipping cubes on a
     bonded slab, nullity checked by `FullPivLU` on the closed-corner Jacobian); resetting
     `LargestPivot` per panel reddens the rank-64 row only; skipping the trailing update's
     `Diagonal.tail(Rest)` refresh survives the whole suite (the in-panel running update already
     holds the Schur diagonal, so the refresh changes rounding only; removing it would be a
     bit-level change to every factor and needs the pins regenerated, not worth it alone);
     `ArchAnchorsTest` (Orduña pile) and `StackAnchorsTest`
     (diamond) incidentally guard panel faults, so neither may be loosened blindly; with the
     reference gone, Mechanism-draw readings and motions are not pinned (they differed by up to
     0.79 relative between the factors); the pins (`tests/FactorPins.h`, regenerated only by
     `Measure.Factor.GeneratePins` with a reason in ANCHORS) catch the pivot tolerance × 10
     only through the trade budget (30 against 6) and miss `>=` for `>` in the pivot test (no
     pivot sits on the boundary); Mechanism iteration counts are not pinned though none of the
     3,319 moved under three equivalent changes (they could be pinned exactly); an InvalidInput
     pin's unconstrained count is written but never checked (no pin is in that state); a
     null-space check of mechanism motions (‖K·v‖ small) needs the assembled stiffness, which
     only a production seam could expose (the public corner flags allow a weaker necessary
     check; flipping the motion sign reddens 9 Release tests meanwhile); the 1.0 s budget on
     the 16 × 20 wall is a timing assertion on this machine;
   - from the ungrounded-shortcut reviews (pre-existing, not from the item): the sparse and
     dense first-iteration finiteness checks sum in different orders, so for components above
     `DenseDofLimit` they disagree in a stiffness band of 3.6e304 to 1.45e305 Pa/m; a body
     weight that overflows to infinity (a 1e308 kg mass) is never validated and read Mechanism
     with a NaN normal; a gravity of 1e-300 counts as non-zero and takes the shortcut; the
     certificate's rigid fall now also fires on a floating component whose net load's sum of
     squares would overflow (components from about 1e155; it was skipped before and the LP ran),
     untested; an association-order mutant in the shortcut's finiteness fold, `d_i·(K·d_j)`
     for `(d_i·K)·d_j`, survives the agreement sweep and the review's exact bisection (the
     two orders agreed to the last bit on every fixture tried); the fold test could adopt that
     bisection with a twin-joint pair and a six-brick slab (the review's `ProbeFoldTest`);
   - sparse elastic path: the pivot test is pinned by no mutant once the estimate exists; the
     estimate's constructed adversary (a soft mode orthogonal to the ±1 start with a second
     mode within 10³) accepts at λ_min 1e-3 of the bar (worst consequence a 1e-3 equilibrium
     residual, never a false Stands; two start vectors or an eigen-residual check would close
     it); the symbolic analysis is redone every active-set iteration (`analyzePattern` once per
     `SolveProblem`); `K * V` relies on both triangles being stored;
   - `Structure::Components()` is mutable state behind a const method (no thread-safety
     promise exists anywhere; log it when one is made); `StandingComponent::ReadingSlots` is
     read by nothing in the library;
   - certificate: uncaught mutants recorded in ANCHORS: the retry's `LargestDiagonal` without
     the coupling term (dense and bordered), the bordered regularisation's size (× 100 green),
     the NaN-only part of the positivity scan, either bordered pivot guard alone (they back
     each other up; the suite reaches the retry ten times); the structured `SolveNormal` cap loops solve into per-site scratch on the `Normal`
     (2026-10-12; `mutable`, reused across solves, no answer depends on it), bit-identical,
     shed certificate 5.17 to 4.37 s and half warehouse 47.2 to 44.0 s on a loaded machine (the
     earlier 1.70 s / 1.05 s figures were the loop's own share, not the certificate); `Y` and the
     sparse path's bordered solve still allocate per call; there is no regularised retry at the cap-site
     level, so one site's rounding failure reads the component Undecided (rare); the stall
     rule is a safety stop that no large fixture in the suite reaches since 3c (its pins are
     the ec6 1e-15 wall and the shear-capped brick);
   - tests: `Certificate.Sparse.AgreesWithTheDensePath` is now the only
     two-path comparison for the certificate (13 s in Release; load factors 1e-9, iteration
     counts equal, since the close: a border coupling off by 0.01 % reddens it, by 1.2× over
     the 1e-12 absolute floor; near-zero load factors differ by 7e-22 at base, so a floor near
     1e-18 would widen that margin a million-fold with no false-red risk); the Slow tier is 9
     tests, the two ratio tests having retired with the referees; the shed's certificate
     (8 s, below `DenseRowLimit`) gained nothing from 3c.
   Then the overview artifact (Version 7) and this file's snapshot are the last close items.
 **Slice 8,
   the Unreal adapter, is on hold by owner directive (2026-10-11): do not start it.** The
   remaining slice-2 anchors (item 2) can run alongside slice 6.
   Slice 3 and 4b items still open (none host-unsafe; the unsafe one, the wedge band, is under
   Deferred):
   - the InvalidInput verdict at the pass-counter limit reports `FirstPass 0, LastPass
     UINT32_MAX` (cosmetic, computed before the entry guard); a run that exhausts the counter
     mid-run ends at the cap (2026-10-13, `Cascade.ARunThatExhaustsThePassCounterStopsAtTheCap`);
   - the cap re-judge returning InvalidInput leaves `PassCapReached` with empty readings and
     components (fails closed, undocumented);
   - the file reader's own `crack_pass ≤ sever_pass` check is now masked by `Sever` refusing
     the same (defence in depth, no test fails without it);
   - a severed joint's axis now comes from the mechanism (2026-10-13, DESIGN §10 "a severed
     joint's axis" and the two reviews' rulings; closed after the follow-up review, whose order
     and threshold pins were added); its `Ratio` is still the elastic reading's utilisation, which
     on a mechanism means little; the 1e-3 threshold is written out in Cascade.cpp, not shared with
     the certificate's `UnconstrainedComponent`; no test drives a closing (Compression) mechanism
     through the real certificate end to end (the follow-up review found 10 such joints in seeded
     cascade draws with weakened crushing, 4 in `Cascade.Fuzz.SeededStructuresKeepTheCascadeInvariants`
     and 6 in `Cascade.EventRegion.ReusedComponentsEqualAFreshJudge`, none asserting the axis); a
     joint with `CompressiveStrengthPa == 0` gets no friction group in the certificate, so no
     dilatancy, but `MechanismAxis` still subtracts μ (degenerate: it carries nothing); the
     separator test's `HorizontalN < LpLimitN` check compares two constants;
   - the arch's two 0.484° slivers are released as single-voussoir clusters; whether a hinge
     needs a minimum rotation is still open, and this is the measurement against it;
   - the mechanism selection ignores finite positive shear caps (harmless at `t* = 0`; for
     `0 < t* < 1e-9` it reads Undecided, which fails closed);
   - from the cascade-scan reviews (pre-existing, logged): the pass loop's load filter
     (`Contains(ReleasedThisRun, Load.Body)`) and `MotionOf` are linear searches, O(loads ×
     released) and O(clusters × mechanism) per pass, unmeasured; `FindClean` and the report
     index call `front()` on a previous report's bodies, undefined for a forged empty list; a
     forged body in two Overstressed components is owned by the first in the crack band (the
     old loops let the second count it too); the indexed tables match a forged
     stale-generation handle by slot; the capped-bounds test's power at caps 1.2 and 3.0 rests
     on one report (seed 0x6b1b2 + 14); the fuzz test could assert after each pass that
     `InterfacesOf(b)` equals `LiveInterfaces` filtered by endpoint;
   - the shipped certificate never returns a per-component InvalidInput entry, so that arm of
     `Consult` is reached only through the no-entries branch (pinned by
     `Authority.Combined.ACertificateWithNoEntriesReadsInvalidInput`); `std::min(Cap, NaN)`
     reads the cap, so a NaN component bound would read as standing at the cap (the
     aggregate's existing expression; for the NaN audit);
   - a cap-0 corner on the inscribed side still uses the inner tension offset min(ft, c/μ):
     conservative, not unsafe;
   - the reverse hinge property only checks that a cluster of listed bodies touches a hinge;
   - the pile anchor runs on a reconstructed bond (bed joints only); verify against the paper's
     figure when it can be read;
   - B-ARC-09, the voussoir-count sweep, is not pinned (expect convergence to 0.107478 from at
     or below);
   - the 4a items still open: the selection gate is `FeasibilityTolerance`; `t_out` is the
     circumscribed converged primal, not the dual objective; `Iterations` excludes the
     selection solve's count; HiGHS was slice 7's referee question, and slice 7 is dropped by owner
     directive (2026-10-12), so no independent LP referee is coming: the NNLS oracle, the pins and
     the dense-against-sparse comparison are the certificate's referees.
   The remaining slice-2 anchors (item 2) can run alongside slice 2c.
2. Slice 2b, the remaining slice-2 anchors (BENCHMARKS groups 2 and 3). Group A is pinned
   (2026-10-12, `tests/Slice2bAnchorsTest.cpp`, all green on arrival, two-sided at 1 %): B-WAL-03,
   B-STK-04, F-EMI-01, F-CHC-01 (except "a braced parapet stands": the fixtures make only bed
   contacts, so a vertical bracing contact needs a fixture first), F-LIB-01/02 (the upper joints
   bonded by fixture choice, the base dry), B-ROK-02 (the static threshold b/h only; the measured
   α is dynamic). On the two sliding anchors the combined authority reads Undecided between μ and
   μ/cos(π/16) (the certificate's G = 16 polygon band; documented fail-closed, but the host gets no
   slide signal there). Group B is pinned too (2026-10-12, `tests/Slice2bWedgeArchAnchorsTest.cpp`, green on arrival):
   the frictionless 5-diamond falls and the 4-diamond stands (the paper's vertical-force model,
   μ 0); B-WDG-02's no-push slip at μ 0.20, self-locking at 0.25 and the raise force 0.935 W
   two-sided; the hold force 0.0436 W only as a fail-closed band (the elastic solve hits the
   50-pass cap from 0.92 to 1.15 × hold, and the certificate reads Undecided just over the
   raise force); the Couplet–Heyman arch in the elastic field (least thickness bisected to
   [0.107478, 0.107479], the friction switch to [21.5947°, 21.5957°], Undecided below, never a
   false Mechanism). **Moved to slice 4:** B-ORD-01's two-sided bracket in the elastic field (it
   Stands to λ 0.3874 and reads Undecided from 0.3875, below the published lower bound 0.427;
   pinned one-sided). The 5-diamond at μ 0.5 stands (DESIGN §10 "the 5-diamond with friction and the
   false-Stands class": a probe DEM and the exact LP agree on the switch at μ 0.422); it is pinned
   (`Statics.Anchor.FiveDiamondStandsWithFrictionAboveTheCriticalValue`), and an elastic Mechanism
   under a certificate Stands now reads Undecided (2026-10-13, done after a cold review; the
   bridge at μ 5 and a rebuilt random dry wall pin it). Left from the review: 71 of the rule's 80
   verdict changes in the random families are on cohesive structures that no referee can judge
   (the probe DEM has no cohesion), and the DEM's fixed contact pairs cannot settle cases that
   arrest at millimetres (the bridge's siblings), so releasing the elastic mechanism stays
   unsupported until a referee with geometry-following contacts exists; the seeded
   composition-table property is done (`Statics.Combined.EveryConsultedComponentFollowsTheCompositionTable`,
   which also catches a Collapses with an empty release set), but no generated structure reaches a
   certificate Undecided: `Statics.Combined.ElasticMechanismWithCertificateUndecidedReleasesNothing`
   pins (Mechanism, Undecided) and `ElasticUndecidedWithCertificateUndecidedReleasesNothing`
   (Undecided, Undecided); a
   Stands carries no geometric-margin signal (a knife-edge stack both authorities stand falls in
   the DEM). **Known unsafe gap, logged 2026-10-13:**
   where the elastic authority reads Undecided and the certificate Stands, the structure can
   still fall under Coulomb friction (36 of 152 such dry random walls and 4 of 135 dry random
   stacks fall in the probe DEM); no cheap discriminator was found. A load-stepped elastic path was
   probed and not adopted (DESIGN §10 2026-10-13 "the load-stepped elastic path"): a proportional
   ramp equals the one-shot solve; gravity first, then the lateral loads, catches all 25 robust
   staged DEM falls among the dry gap walls and loses 15 of 122 stands, but does nothing on stacks,
   costs about 77 ms per gap component, changes only the label, and breaks sparse/dense
   determinism; it waits on a referee with geometry-following contacts, a load-history ruling,
   cohesive scope and determinism. Accepting an admissible elastic iterate as Stands (51 gap cases
   had one, all stand) is still a candidate. Candidate, not ruled: releasing the elastic mechanism where the certificate stands
   (needs a non-associative test-tier referee first). Inputs retrieved
   2026-10-13 (BENCHMARKS "Inputs retrieved 2026-10-13"): B-FTL-01, F-FOR-01 (thickness
   inferred) and B-SW-01 (±0.2 kN band) are buildable; F-PSY-01 has its friction (37°) but no drum
   heights; B-DER-01 has three of its six walls; F-RON-01 supports only a simplified bearing model
   and its source's provenance needs checking; F-STR-01 and B-COR-01 were not found (substitutes
   listed there). All except F-PSY-01 are slice-4 rows. B-BUR-01 needs a soil-pressure driver;
   F-PYR-01 needs a zero-extent contact; group 9 runs only in the host adapter's suite.
3. `Scripts/Test.ps1` excludes `Slow.*`, `Deferred.*` and `Measure.*` from the default run (2026-10-11; the
   arch and pile anchors carry `LOADPATH_SLOW_IN_DEBUG`, Slow. in Debug only). A caller `-tc` drops the exclusions (it names what to run); a caller
   `-tce` alone adds to them (2026-10-13).

## Deferred

- The certificate's interior-optimum and stall defects are fixed (2026-10-13, DESIGN §10 opening
  and amendment; the retry shift escalates 1e-16 to 1e-12 of the full diagonal with 20 refinement
  passes after a shifted factor, and the stall window restarts when the complementarity halves);
  B-SW-01 is pinned in the paper's set-up. The cold review's three send-backs are done (DESIGN §10
  "cold-review rulings"): `Slow.Certificate.Solver.TheForaboschiWallStandsInsideItsAssociativeLimit`
  separates the two halves of the retry fix (80 kN / 42 kN and 140 kN / 80 kN Stand; a straight
  1e-12 shift or one refinement pass reads Undecided), the severed axis has scripted tests, reads
  Compression where the mechanism closes the joint and subtracts no friction lift on a zero-cap
  joint, and the Minho test is `Slow.`. Left from it: no fixture reaches the escalation's 1e-12 rung, which
  on coupling-dominated components would be 1e-8 to 1e-6 of the site diagonal; the selection LP's single 1e-12 retry failed rocking walls past their limit, fixed (DESIGN §10
  "the selection LP's retry"; `Certificate.Selection.ARockingWallPastItsLimitNamesItsMechanism`);
  the selection corrector takes 3 refinement passes (DESIGN §10 "the modified Cholesky
  (cold-review rulings: reverted)"); left: the selection has no stall rule. The certificate's
  conditioning near an interior optimum is fixed (2026-10-13, DESIGN §10 "the certificate's
  conditioning near an interior optimum"): a rigorous dual bound at every cone-solve exit, a capped
  re-solve on an inscribed stall, `t_out = min(cap, U)` on a circumscribed stall, a scaled start
  for bodies under 1e-6 of the load scale, `CertificateOptions::bStallRecovery`; the 46-block and
  137-block walls, the Foraboschi crushing anchor at 23,900 N and the light-block table at mass
  ratios 1e11 and 1e12 now decide; its cold review's two bound holes (the facet box under μ > 2,
  an overflowed column dropped) are fixed and guarded through `src/CertificateProbe.h` (DESIGN §10,
  the conditioning's cold-review rulings). Left from it: the γ rounding allowance is unguarded
  (dropping it moves U by about 1e-17); the boxes' own rounding is not allowed for (a computed
  |f · d| errs by up to γ₃|f||d|, the whole value at small μ; about 1e-13 relative in U; fix:
  inflate each box by (1 + γ) plus γ₃ B |d| per group); which duals give the mechanism after a
  capped re-solve (the re-solve's, or the stalled solve's last when the re-solve reached its cap)
  is unguarded: the 46-block wall at 47,150 N takes the at-cap branch and the choices differ by
  0.227 in normalised motion, but its lists are identical, so a pin must assert the mechanism (a
  virtual-work check that its kinematic multiplier equals t_in is the principled form; nearby
  loads' mechanisms differ by 0.21, so a neighbour is no reference); a lasting oracle for the
  boxes (the LP with `ColumnBoxes` imposed keeps the optimum, through an (A, b, boxes) export and
  a small bounded simplex in test support) is not written; the seeded box draws carry no cohesion
  or tension (only the bolted block covers a cohesion group's facet term); U can exceed t by 1.2e-4 relative on shear-capped solves, and when
  it is loose by more than 1e-3 neither rung converges (the cohesive open-slide test stays
  Undecided; small `MaxIterations` costs about 3× in failed rungs); every solve now copies x and
  y at exit for the probe; the margin policy's cost (the probe saw the 1e-5 re-solve stall on 2
  of 10 fallbacks, 16 to 50 s, before 1e-3 converged); mechanism deflation was not prototyped. The dense selection takes 20 to 27 s on the full wall; the
  hinging list wanders between 17 and 33 joints on the 57-body wall; The
  Foraboschi fixture's limits with the paper's f_c 1.21 MPa are about 25.0, 40.0 and 57.1 kN at
  40, 80 and 140 kN, 4.6, 0.5 and 2.3 % above Portioli's model (23.9, 39.8 and 55.8 kN);
  `Slow.Certificate.Anchor.ForaboschiWallWithCrushingBracketsThePaperModel` pins Stands at the
  model values and Collapses about 8 % past each limit; the 80 kN Stands point has only 0.5 %
  margin.
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
