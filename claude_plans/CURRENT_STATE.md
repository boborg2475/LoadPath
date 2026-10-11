# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-11)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11 and Eigen
3.4.0, the compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced.
Slice 1 (the model: handles, profiles, bodies, interfaces, validation, removal) and slice 1b
(JSON structure reader/writer, `RunCli`, the `loadpath_cli` binary) are committed and reviewed:
every green-on-arrival row proven by a recorded mutation (commit messages). Slice 2a, the
standing authority, is committed through 3d39b2a (98 cases, 2948 assertions, Debug and Release):
`IStandingAuthority`, the RBSM corner-spring implementation with a Schur-pivoted dense LDLT and
the free active set, 18 anchor and mechanics tests (incline tip and slide, Housner, free and
coursed wall, the Itasca slope regimes, harmonic stack, Paterson–Zwick triangle and diamond) and
10 solver characterisation tests. **Its first review (2026-10-11) found five blocking items and
the slice is not done**: see "Next" 1. The design brief ([DESIGN.md](DESIGN.md)) has every §9
question decided and §10 carries the slice-2 rulings.

## Next

1. Close the slice-2a review (findings and the rulings that answer them are in API.md "Slice 2",
   DESIGN §10 2026-10-11 and the review-round tests `Statics.Solver.*` T1–T11):
   - a squat dry block past both thresholds reads Undecided (restore re-closes open corners
     stuck, which re-open; ruling: restored corners re-enter slipping on a dry joint);
   - status-only cycle detection turns decided answers into Undecided (ruling: remove it);
   - verdicts depend on unrelated bodies through a global, unit-mixing pivot tolerance (ruling:
     per-component solves with rotational scaling by the component's characteristic length);
   - readings on a Mechanism come from a stale or zero displacement (ruling: range-space
     solution of the last active set, per component);
   - options are not validated (NaN pivot tolerance reads Mechanism; ruling: InvalidInput);
   - unpinned declared behaviour: re-stick, the open-corner restore, the transverse tangent
     magnitude, the slip-capacity clamp in tension, `UnconstrainedBodies`' negative side, Axis
     None on a zero-force interface; weak `!=` assertions and sign-blind shape helpers.
   Then re-review.
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
   exists; today there is none (the whole suite runs in under a second).

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
  block the slip and re-stick conditions exclude each other); the `bNonFinite` term in
  `IsOverstressed`, which looks unreachable because `Ratio` already maps a non-finite ratio to
  +inf and a NaN corner force trips compression; the determinate side of the indeterminate-gap
  rule (a mutant that keeps every open corner open in any neutral solve survives).
- Undecided probe results (dev-expert and review, 2026-10-11; 8,445- and 3,366-solve grids over
  a single brick, bridges and stacks at bonded, μ 5 and μ 0.6): two causes, both under "Next" 1.
  The 30 single-brick μ 0.6 cases are the open-corner restore re-closing stuck (118 grid verdicts
  flip Undecided → Mechanism when that half of restore is removed, no decided verdict changes);
  the μ 5 single-brick and stack cases are status-only cycle detection (removing it recovers 40
  of 109 Undecided on the review grid, every one to the right verdict). The projected motion is
  also not unit-invariant (it mixes m with rad); the per-component rotational scaling in API.md
  addresses the tolerance side of that, and the restore ruling the cycling side. If the 2b
  anchors still cycle after both, the next step is a search of the admissible cone rather than
  one projected direction.
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
