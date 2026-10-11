# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-11)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11 and Eigen
3.4.0, the compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced.
Slice 1 (the model: handles, profiles, bodies, interfaces, validation, removal), slice 1b (JSON
structure reader/writer, `RunCli`, the `loadpath_cli` binary) and slice 2a (the standing
authority) are committed and reviewed, every green-on-arrival row proven by a recorded mutation
(commit messages). Slice 2a, through 1025bd9 and four reviews: `IStandingAuthority`, the RBSM
corner-spring implementation with per-component Schur-pivoted dense LDLT solves and the free
active set, under the narrowed contract of DESIGN §10 (it certifies Stands and Overstressed,
reports Mechanism as a verdict, and fails closed to Undecided rather than guess); 18 anchor and
mechanics tests (incline tip and slide, Housner, free and coursed wall, the Itasca slope
regimes, harmonic stack, Paterson–Zwick triangle and diamond), 20 solver characterisation and
defect tests, a seeded locality property and a seeded static-admissibility oracle property.
Default suite 113 cases (6288 assertions in Debug, 10081 in Release, about 1.8 s and 0.3 s);
the `Deferred.Slice4` tier holds 3 cases red by design. The design brief
([DESIGN.md](DESIGN.md)) has every §9 question decided and §10 carries the slice-2 rulings.

## Next

1. Slice 4 pulled forward (DESIGN §10, 2026-10-11): the kinematic collapse certificate behind
   its own swappable interface (API.md contract to write first), a limit-analysis LP over the
   same interfaces (associative friction for the LP bound, HiGHS vendored as the solver, the
   referee DESIGN §8 planned for slice 7 arrives with it), used to referee every component the
   elastic authority leaves Undecided **and every Mechanism it reports** (DESIGN §10: RBSM's
   Mechanism is a verdict, not a certificate), and to report mechanism shapes. The contract
   must say which theorem governs under non-associative friction (the associative LP is an
   upper bound on the collapse load; a Mechanism the LP rejects is a false release). The
   seeded oracle property TS2 (never Stands where the outer cone oracle is infeasible, never
   Mechanism where the inner one is feasible) is its first cross-check. Acceptance: the
   `Deferred.Slice4.*` cases go green and move back under `Statics.*`; the probe grids' 318
   fail-closed Undecided points resolve; the arch (B-ARC-01/06), Orduña (B-ORD-01) and wedge
   (B-WDG-01) rows become reachable.
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
3. `Scripts/Test.ps1` excludes a `Slow.` prefix from the default run once the first slow test
   exists; today there is none (the default suite runs in about 1.8 s in Debug, 0.3 s in Release).

## Deferred

- B-WDG-01 (the 3DEC tetrahedral wedge, phi_crit 33.36 deg) is written as a skipped test
  (`Statics.Anchor.ItascaWedgeSlidesAlongTheIntersectionBelowItsCriticalFrictionAngle`,
  orchestrator 2026-10-11) and its BENCHMARKS row now says slice 4: the published threshold is a
  kinematic answer (friction along the line of intersection), and the elastic corner-spring field
  with kn = kt releases every corner near 40 deg or finds a squeezing force set that stands near
  31 deg. Un-skip it when the collapse certificate (slice 4) exists, or decide a kt << kn fixture
  is acceptable and record why.
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
  - `IsLive(InterfaceId)` means "not Removed"; slice 3 decides whether `Cracked` or `Severed`
    count as live (API.md).
- Structure file, open rulings from the 1b review (2026-10-11): the format carries no interface
  state or pass stamps, so once slice 3 produces Cracked or Severed joints load(write(x)) would
  read them Intact; slice 3 decides whether to write state and passes or omit severed joints.
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
  or record why not. Test spec from the fourth review: move the oracle into a shared test
  header so the two bridge `Deferred.Slice4` cases CHECK `Oracle == CanStand` as a
  precondition (`MakeBridge(5.0, 0.12, 0.0)` at (2.5, 0.875), (2.375, 0.875), (2.25, 0.875)
  reads CanStand with inner residual about 1e-16 at μ 5 and 3.5).
- Slice 4's contract must settle the theorem question the slice-2 tests presuppose: API.md says
  which answer is physical under non-associative friction is open, while TS1 and TS2's
  never-Mechanism direction assert the lower-bound answer and the `Deferred.Slice4` cases are
  the slice-4 acceptance; say what a future red on TS2's never-Mechanism direction means (a
  defect, or a known limit that moves under `Deferred.`).
