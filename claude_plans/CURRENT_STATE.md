# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-10)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11, the
compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced, one smoke test
green through `Scripts/Test.ps1`. Slice 1 model (handles, profiles, bodies, interfaces,
validation, removal) committed after two review rounds (33/33 cases, 966 assertions, Debug and
Release); the four review rulings in API.md are implemented and every coverage row is proven by
a recorded mutation (slice-1 commit message).
The design brief ([DESIGN.md](DESIGN.md)) has every §9 question decided by the owner.

## Next

1. Distil `reports/Rigid block benchmark scenarios.md` (2026-10-10, five research notes under
   `research_notes/`) into `claude_plans/BENCHMARKS.md`: one row per anchor with inputs, the
   pinned number, the assumptions tag and the source, in the report's adoption order (seven
   exact anchors first, then the friction switches, then the flow-rule set, then experiments
   as bracket tests, then bonded-joint anchors). Each row becomes a test in slices 2–5. Owner
   picks the tier-1 set before slice 2 starts.
2. Slice 1b: the structure file reader/writer and the CLI (API.md "Structure file"). The model
   itself is committed and reviewed.
3. `Scripts/Test.ps1` excludes a `Slow.` prefix from the default run once the first slow test
   exists; today there is none.
3b. `claude_plans/ANCHORS.md` has no rows yet; slice 2 pins the first anchor and adds its row
   (the brick dimensions in `tests/ModelTestSupport.h` are fixture inputs, not asserted numbers).
4. Validation cases from documented real failures and known solver failure modes: researched
   2026-10-11, report at `reports/Structural failure case studies.md` (six adoption phases);
   distil with the benchmark report into `claude_plans/BENCHMARKS.md` (item 1) and schedule
   each case as a test in the slice its phase names.

## Deferred

- Eigen (decided, DESIGN §9 Q4) is not vendored yet; slice 2 vendors it under `third_party/`.
- A second compiler (clang-cl ships with this Visual Studio) as a CI-style check for
  portability; not set up.
- The test script's run is single-configuration; a `-Both` switch for Debug and Release would
  catch optimiser-dependent float differences early.
- Slice 1 behaviour left untested by test-expert, so unimplemented or unpinned:
  - Negative stiffness is accepted (only negativity of strengths, friction and the cap is
    checked); slice 2 decides whether zero or negative stiffness is refused.
  - A mass of `-0.0` is accepted (`!(m >= 0)` lets it through); no test says either way.
  - `Dot`, `Cross`, `Length` and `IsFinite` are exercised only through `AddInterface`.
  - Two normal guards are unreachable defence in depth: the NaN branch of
    `!(LargestComponent > 0)` (the finiteness check runs first) and the stored-length check
    `|len - 1| <= 1e-9` after scaled normalisation (a scaled vector's length is in [1, sqrt 3], so
    the division lands within ulps of one). Neither can be driven red by a test.
  - `IsLive(InterfaceId)` means "not Removed"; slice 3 decides whether `Cracked` or `Severed`
    count as live (API.md).
- `RemoveBody`, `InterfacesOf` and the live counts scan every slot (O(n)); fine for slice 1,
  but a per-body adjacency list is needed before a solver calls `InterfacesOf` in a loop
  (slice 6, 15,042 joints).
