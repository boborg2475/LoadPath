# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-11)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11 and Eigen
3.4.0, the compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced.
Slice 1 (the model: handles, profiles, bodies, interfaces, validation, removal) and slice 1b
(JSON structure reader/writer, `RunCli`, the `loadpath_cli` binary) are committed and reviewed:
every green-on-arrival row proven by a recorded mutation (commit messages). Slice 2a, the
standing authority, is committed through f85bc54 (107 cases, 5453 assertions, Debug and
Release): `IStandingAuthority`, the RBSM corner-spring implementation with per-component
Schur-pivoted dense LDLT solves and the free active set, 18 anchor and mechanics tests (incline
tip and slide, Housner, free and coursed wall, the Itasca slope regimes, harmonic stack,
Paterson–Zwick triangle and diamond), 19 solver characterisation and defect tests and a seeded
locality property. **Two reviews so far (2026-10-11); the second left the items in "Next" 1
open and the slice is not done.** The design brief ([DESIGN.md](DESIGN.md)) has every §9
question decided and §10 carries the slice-2 rulings.

## Next

1. Close the slice-2a review (findings and rulings are in API.md "Slice 2", DESIGN §10
   2026-10-11 and the review-round tests `Statics.Solver.*`). State after the dev round on the
   ruling "The elastic authority does not guess" (2026-10-11; default suite 112/112 in Debug
   and Release, the `Deferred.` tier 0/2 as intended):
   - implemented: a restored dry corner re-enters slipping with `μ · N⁺` from its own normal in
     the last full-rank solve, opposite the motion's tangential part, zero transverse tangent;
     a cohesive one re-enters stuck; nothing is seeded from the singular solve; a corner open
     in the last full-rank solve (or with none yet) makes the component Undecided with the last
     full-rank readings and no mechanism. `RestoredCornerTangentDoesNotInflateThePivots` is
     green (bridge beds carry 3W, motion a +x translation). The two cases the elastic field
     cannot certify are split: the default suite pins never-Mechanism on the six bridge rows
     and never-Stands on the stack past friction; `Deferred.Slice4.*` holds the strong
     assertions (Stands on all six bridge rows, Mechanism on the stack), red until slice 4,
     excluded from the default run by `Scripts/Test.ps1`.
   - not implemented, held at the TDD gate because no test bites them (mutation: identical
     suite failures): `SolveInRange` on Load − Projection and the `min(kt, …)` transverse clamp
     (neither changes any of the 3366 probe verdicts; the clamp can only bind inside the 1e-9
     stick margin now that restored corners re-enter with zero tangent), and "a corner restored
     a second time is Undecided" (changes two probe points: bridge μ 5 (2.5, 0.125)
     Undecided → Stands, bridge μ 0.6 (0.5, 0.375) Undecided → Mechanism). Each needs a red:
     a toppling-brick Mechanism whose readings are checked against the range load; a reachable
     clamp case, or a ruling that drops the clamp; a fixture that reaches a second restoration.
   - deferred to slice 4 (`Deferred.Slice4.*`): the two-brick stack μ 0.6, shift 0.03 m,
     λ (0.625, 0.875), a provable mechanism (|λ| > μ) that reads Undecided after 3 passes (bed
     corner 2 opens in pass 1, is open in the pass-2 full-rank solve, and the pass-3 motion
     closes it: the fail-closed clause); and the bridge μ 5 rows (2.5, 0.75), (2.25, 0.5),
     (2.25, 0.625), (2.125, 0.625), all Undecided by the same clause ((2.0, 0) and (2.125, 0)
     stand after 12 passes). Without the clause (2.5, 0.75) and (2.25, 0.5) read false
     Mechanisms, which the default-suite never-Mechanism assertion now guards.
   - probe (561 points each, λx 0..4 × λy 0..2 step 0.125; brick, 0.08 m stack, three-brick
     bridge; μ 0.6 then μ 5) Undecided counts, c50e7d3 → now: brick 0/0 → 0/0, stack 12/0 →
     48/102, bridge 11/19 → 65/136. No Stands ↔ Mechanism flips. Stands → Undecided: bridge
     μ 5 (2.375, 0.25), (2.375, 0.625), (2.5, 0.75). Undecided → Stands: bridge μ 5
     (2.5, 0.125). Eleven Undecided → Mechanism, every one at |λ| > μ. The 318 Mechanism →
     Undecided points are slice-4 territory.
   - the restored corner's held magnitude is unpinned (re-entering with zero held force
     survives the suite).
   - Mechanism readings can show tension on a slipping dry corner that re-entered with zero
     held force (0.08 m stack, μ 0.6, λx 1.5: N = −0.43 N on the bed, −2.10 N on the joint);
     informative, not a certificate, and unasserted.
   - a component that is InvalidInput through its assembled stiffness returns InvalidInput for
     the whole solve with `InvalidInterface` unset.
   Then re-review.
2. Slice 4 pulled forward (DESIGN §10, 2026-10-11): the kinematic collapse certificate behind
   its own swappable interface (API.md contract to write first), a limit-analysis LP over the
   same interfaces (associative friction for the LP bound, HiGHS vendored as the solver, the
   referee DESIGN §8 planned for slice 7 arrives with it), used to decide every component the
   elastic authority leaves Undecided and to report mechanism shapes. Acceptance: the two
   `Deferred.Slice4.*` cases go green and move back under `Statics.*`; the probe grids' 318
   fail-closed Undecided points resolve; the arch (B-ARC-01/06), Orduña (B-ORD-01) and wedge
   (B-WDG-01) rows become reachable.
3. Slice 2b, the remaining slice-2 anchors (BENCHMARKS groups 2 and 3): B-WAL-03 (wall slides at
   λ = μ), B-STK-04 (constant-step stack), B-WDG-02, B-ROK-02, F-CHC-01, F-LIB-01/02, F-EMI-01,
   the 5-diamond (hits the 50-pass cap today, logged below), and the arch and pile rows that
   API.md "Anchors slice 2 must pin" lists but BENCHMARKS marks slice 4 (B-ARC-01/06 Couplet–
   Heyman, B-ORD-01 Orduña): attempt them with the elastic no-tension field in 2b and move the
   ones it cannot reach to slice 4 with the reason recorded, as the wedge was. Rows with inputs
   still to retrieve: B-COR-01 corner geometry, F-PSY-01 friction angle, F-FOR-01 and F-STR-01
   pier geometry (Portioli 2012), B-DER-01 wall geometry, B-SW-01 exact limit load (graphical
   only), F-RON-01 panel thickness, B-FTL-01 layout. B-BUR-01 needs a soil-pressure driver;
   F-PYR-01 needs a zero-extent contact; group 9 runs only in the host adapter's suite.
4. `Scripts/Test.ps1` excludes a `Slow.` prefix from the default run once the first slow test
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
- Undecided probe results: the two causes found on 2026-10-11 (open corners re-closed stuck,
  status-only cycle detection) are fixed. A single brick at μ 0.6 and μ 5 over λx 0..4 ×
  λy 0..2 (step 0.125, 561 solves each) went from 30 + 8 Undecided to 0, and the Stands counts
  (22 and 351) match the rigid-block answer exactly. The bridge and stack grids (8,445 and 3,366
  solves) were not re-run; do that before 2b. If the 2b anchors still cycle, the next step is a
  search of the admissible cone rather than one projected direction.
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
