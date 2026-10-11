# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-11)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11 and Eigen
3.4.0, the compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced.
Slice 1 (the model: handles, profiles, bodies, interfaces, validation, removal) and slice 1b
(JSON structure reader/writer, `RunCli`, the `loadpath_cli` binary) are committed and reviewed:
68 cases, 1826 assertions, green in Debug and Release, every green-on-arrival row proven by a
recorded mutation (commit messages). No physics exists yet; slice 2 (the standing authority,
API.md) is next. The design brief ([DESIGN.md](DESIGN.md)) has every §9 question decided.

## Next

1. `claude_plans/BENCHMARKS.md` (2026-10-11) holds the 100 known-answer rows from both
   reports, in nine adoption groups, each with its earliest slice. Tier-1 set for slice 2
   (orchestrator, autonomous mode): every group-2 and group-3 row marked slice 2, in the order
   API.md "Anchors slice 2 must pin" lists them; each row becomes a test in the slice it
   names. Rows with inputs still to retrieve: B-COR-01 corner geometry, F-PSY-01 friction
   angle, F-FOR-01 and F-STR-01 pier geometry (Portioli 2012), B-DER-01 wall geometry,
   B-SW-01 exact limit load (graphical only), F-RON-01 panel thickness, B-FTL-01 layout.
   B-BUR-01 needs a soil-pressure driver; F-PYR-01 needs a zero-extent contact; group 9 runs
   only in the host adapter's suite.
2. Slice 2, the standing authority (API.md "Slice 2"): the `IStandingAuthority` interface, the
   RBSM implementation with four corner springs and a dense Eigen LDLT, the free active set,
   mechanism reporting, and the tier-1 anchors from BENCHMARKS.md groups 2 and 3.
3. `Scripts/Test.ps1` excludes a `Slow.` prefix from the default run once the first slow test
   exists; today there is none.
3b. `claude_plans/ANCHORS.md` has no rows yet; slice 2 pins the first anchor and adds its row
   (the brick dimensions in `tests/ModelTestSupport.h` are fixture inputs, not asserted numbers).

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
- Slice 2 standing authority (`src/RbsmStandingAuthority.cpp`, 2026-10-11, awaiting review): two
  model steps beyond API.md's rulings were needed for the 4-diamond and need ratifying there.
  (1) A singular active set with load in the null space is a Mechanism only if the motion is
  admissible: an open corner the motion would close re-closes, and a slipping corner the motion
  would slide along its held friction re-sticks; then the loop continues. (2) In a neutral solve
  (singular, zero load projection) an open corner whose gap the null space moves is not
  re-closed, since its gap is indeterminate. Without (1) the diamond reads a spurious slide;
  without (2) a corner at exactly zero force cycles open/closed until Undecided.
- Slice 2 behaviour still with no test (characterisation round 2026-10-11, `tests/StandingSolverTest.cpp`,
  every other item proven by mutation): the 1e-9 stick hysteresis (margins 0 and 1e-3 both survive;
  no fixture leaves a slipping corner's trial shear within the margin of capacity, and on a single
  block the slip and re-stick conditions exclude each other); status-cycle detection (every cycling
  fixture found is driven by one of the two defects below, so pinning one would enshrine the defect;
  pin it once they are fixed); the `bNonFinite` term in `IsOverstressed`, which looks unreachable
  because `Ratio` already maps a non-finite ratio to +inf and a NaN corner force trips compression.
- Red, `Statics.Solver.BrickOverturnedByABiaxialLoadIsAMechanismAboutItsToe` (found 2026-10-11): a
  brick past its toe under λ = (3.375, 0.375) reads Undecided after 3 passes, not Mechanism.
  `Factorise` counts rank up to the first pivot under tolerance, but Eigen's LDLT can emit a zero
  pivot ahead of a non-zero one (measured D = 1.1e8, 1.1e8, 1.1e8, 289414, 0, 289414 with two stuck
  toe corners), so rank reads 4 where it is 5, `NullBasis` adds a direction that moves the stuck
  corners, the restore re-closes a corner and the loop revisits a state. The same defect makes a dry
  μ = 0.6 brick under λ = (0.5, 0.25) cycle (rank 4 where 5) and makes the two-brick stack
  neighbourhood around λ = (1.5, 0.75) alternate Mechanism and Undecided.
- Red, `Statics.Solver.BrickPushedIntoAWallIsNeverAMechanism` (found 2026-10-11): a brick pushed
  into a grounded wall (dry μ = 0.3, λ = 0.4 to 0.9) reads Mechanism, a pure Y translation. With
  every bed and wall corner slipping, Y is the only free motion and no load has a Y component, but
  the held friction forces carry about 1e-10 N of Y from rounding, which beats the 1e-12 relative
  neutral-load tolerance. At μ = 0.5, λ = 0.6 and 0.9 and μ = 0.7, λ = 0.9 the same fixture breaks
  its Y symmetry the same way and cycles to Undecided in 19 to 25 passes. The always-settled mutant
  makes this test pass, so the drift enters through the held-direction updates.
- A single dry brick under a biaxial lateral load can hit the 50-pass cap with one corner slipping
  while it plainly stands (μ = 5, λ = (0.5, 1.0) and 25 more grid points; 5 at μ = 0.6): the
  held-direction fixed point converges slowly or not at all. Fail-closed Undecided, so not red
  under the ruling, but the cap or the iteration scheme needs looking at before larger structures.
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
