# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-11)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11 and Eigen
3.4.0, the compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced.
Slices 1 (the model), 1b (JSON structure file, `RunCli`, `loadpath_cli`), 2a (the elastic
standing authority), 4a (the limit-analysis collapse certificate), 4b (the combined authority,
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
withdrawal whatever the joint area. Default suite 253 cases in Debug and 264 in Release (the
arch, pile, arch-cascade, fuzz and small-wall anchors run only in Release under
`LOADPATH_SLOW_IN_DEBUG`; three `Slow.*` wall anchors run in Release at every slice close);
the `Deferred.Slice7` tier holds the physical wedge pin and `Deferred.Slice6` the smallest dry
wall the elastic authority cannot settle, both red by design. The design brief
([DESIGN.md](DESIGN.md)) has every §9 question decided and §10 carries the slice-2, 3, 4a, 4b
and 5 rulings.

## Next

1. Slice 2c, **dry perpends settle** (the next solver slice, before slice 6): the elastic
   authority reads Undecided on every dry running-bond wall with perpends between full bricks
   (1 × 3 and 5 × 1 up; 1 × 2, 3 × 1 and 4 × 1, whose perpends sit between half bricks,
   settle) because a dry perpend at N ≈ 1e-18 stays closed (`UpdateActiveSet` opens a corner
   only on N < 0 strictly), its slip capacity μN and its roundoff shear are the same size, and
   the Stuck ↔ Slipping test flips every iteration; the 2 × 1 dry wall reads Overstressed at
   rest for the same reason (shear utilisation = roundoff / 0 = +inf). Ruling to write at the
   slice opening: a corner whose normal force is within a force-noise floor of zero, on a joint
   with zero tension and cohesion, is Open and reads N 0, shear 0, utilisations 0; the floor is
   relative to the component's force scale (the `ForceScaleN` the held-force tolerance uses),
   one named constant; the stick/slip comparison gets the same absolute slack; a genuinely
   loaded dry corner keeps today's rule. Tests: `Deferred.Slice6.SmallestDryWallThatDoesNot
   SettleElastically` moves to the default tier green; the 2 × 1 wall Stands at rest with finite
   zero utilisations on the perpend; a sweep 1 × 3, 2 × 3, 3 × 4, 4 × 5 Stands elastically with
   the iteration counts recorded; the incline and bridge anchors unchanged (the floor is not a
   blanket); a dry perpend under a real load (a 0.2 W push on the end brick of the 1 × 3 wall;
   a uniform λ 0.2 moves the bricks together and the perpends carry nothing) still carries
   above the floor; a bonded perpend at N ≈ 0 is not opened. Mutations: floor 0 restores
   the Undecided; a floor large enough to swallow the end-push perpend (0.22 N against a force
   scale of about 84 N, so above 1e-2) makes the loaded-perpend test red. Then check
   whether the 10 × 8 dry wall (Slow, Release) settles and how many iterations it takes.
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
   - the combined authority copies the structure for EVERY component (RemoveBody scans every
     interface per call, O(C · B · I)) and solves every structure twice; the cascade's scans
     are quadratic (`IndexOf`/`Contains`, union-find without path compression, the release
     scanning every interface per cluster, `FindReading` O(I) per sever, the crack band
     O(K·I·|component|)): slice 6 exposes the RBSM's per-component states and indexes these;
   - two InvalidInput branches in CombinedAuthority.cpp are unreachable today (a sub-solve
     InvalidInput; a per-component certificate InvalidInput): fail closed, untested;
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
3. `Scripts/Test.ps1` excludes `Slow.*` and `Deferred.*` from the default run (2026-10-11; the
   arch and pile anchors carry `LOADPATH_SLOW_IN_DEBUG`, Slow. in Debug only). Still open: any
   caller `-tc` or `-tce` is passed straight through and drops both exclusions.

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
    against the standard); the zero-`bearing_mpa` refusal has no test case of its own (only a
    negative bearing is in the error table): add the case in the next test round.
- **The elastic authority never settles on a dry wall with perpends between full bricks** (slice-5
  sweep, 2026-10-11): the next solver slice, Next item 1 (slice 2c), which carries the
  mechanism, the ruling to write and the tests. Pinned red by design meanwhile:
  `Deferred.Slice6.SmallestDryWallThatDoesNotSettleElastically` (the 1 × 3 wall). The
  measured sweep is in its ANCHORS row.
