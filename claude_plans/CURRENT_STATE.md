# Current State

**Living document.** The running TODO list, not a changelog. Finished something? Remove it here.
Deferring something? Add it here with enough context to pick up cold.

## Where the project stands (2026-10-10)

Repository scaffolded: CMake + Visual Studio 2022 generator, vendored doctest 2.4.11, the
compiler contract (no exceptions, no RTTI, `/W4 /WX`, `/fp:precise`) enforced, one smoke test
green through `Scripts/Test.ps1`. No production code exists. The design brief
([DESIGN.md](DESIGN.md)) is a draft awaiting the owner's review of its §9 questions.

## Next

1. Owner reviews DESIGN.md §9 (name, units, unit types, linear algebra, API STL use, file
   format, license) and rules.
2. Slice 1, the model (DESIGN §8): bodies, interfaces, profiles, generational handles,
   fail-closed validation. First red: a two-body structure refuses a degenerate normal.
3. `Scripts/Test.ps1` excludes a `Slow.` prefix from the default run once the first slow test
   exists; today there is none.
4. `claude_plans/ANCHORS.md` is created with the first pinned number.

## Deferred

- Eigen is not vendored yet; slice 2 decides (DESIGN §9 Q4) and vendors it under `third_party/`.
- A second compiler (clang-cl ships with this Visual Studio) as a CI-style check for
  portability; not set up.
- The test script's run is single-configuration; a `-Both` switch for Debug and Release would
  catch optimiser-dependent float differences early.
