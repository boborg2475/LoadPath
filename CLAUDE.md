# LoadPath

A host-neutral C++20 static library that decides which joints in a structure of rigid pieces
fail, in what order, and which pieces move as a result. It is the successor to the structural
solver inside `C:\Users\bobby\Documents\Unreal Projects\DestructionGame`, rebuilt standalone so
it can be tested in seconds and later dropped into Unreal as a module. The host engine (Unreal,
Chaos) owns falling, collision and piece fragmentation; LoadPath owns the standing verdict and
the break decision. Pieces never fail internally here: that is Chaos's job.

[claude_plans/DESIGN.md](claude_plans/DESIGN.md) is the design brief and the host contract.
Read it before implementing anything substantial. [claude_plans/CURRENT_STATE.md](claude_plans/CURRENT_STATE.md)
is the living TODO list. [claude_plans/TRAPS.md](claude_plans/TRAPS.md) holds the footguns
carried over from the old project; most of them cost a day each to learn.

The old project is reference material, not a template. Read its `claude_plans/DESIGN.md`,
`RBSM_DESIGN.md`, `GRAPHCUT_DESIGN.md` and `TRAPS.md` when a design question comes up; do not
copy its code. Materials and connection types are data, never code paths.

## Units: SI base units, no conversions inside the library

Metres, kilograms, seconds, newtons, pascals, newton-metres. Gravity is 9.80665 m/s². Published
strengths arrive in megapascals and are converted to pascals once, by the catalogue loader, with
a test. Nothing inside the engine converts anything. The old project lost days to a 100× factor
between Unreal force units and SI; that boundary now lives in the Unreal adapter, which does not
exist yet. Name every quantity with its unit suffix (`AreaM2`, `MassKg`, `StrengthPa`).

## Comments: headers only, one or two sentences

- **`.cpp` files carry no comments.** Not on functions, not inline, not in tests. Names and
  assertion messages carry the meaning.
- **Headers may carry one or two sentences per declaration**, as `//` lines directly above it,
  and only when they say something the name does not: a unit trap, a fail-closed reason, an
  invariant. No paragraphs, no prose blocks, no TDD narrative.
- Where a pinned number comes from (a paper, a code table, a measurement) is recorded in
  [claude_plans/ANCHORS.md](claude_plans/ANCHORS.md) under the test's name, not in the test.

This is the owner's rule (2026-10-10). It exists because comment prose was the slowest part of
every change in the old project and most of it restated the code.

## Current state and the running TODO list

[claude_plans/CURRENT_STATE.md](claude_plans/CURRENT_STATE.md) describes the present and the
not-yet-done. Two mandatory rules:

- **When you finish something, remove it from CURRENT_STATE.md** as part of that work.
- **When you defer something, add it to CURRENT_STATE.md** before moving on, with enough
  context to pick it up cold.

Read it at the start of a session and update it at the end of one.

## A commit is a checkpoint, not a finish line

Committing is never the end of a task. Do not commit until the tests are run and green (a
deliberately red test from an in-flight slice is the one exception, and the commit message says
so). Commit at every verified point rather than batching. After every commit re-read
CURRENT_STATE.md and the session's todo list, and resume anything open inside the requested
scope. Stop only when the whole ask is done or you are genuinely blocked, and when you stop with
work outstanding, say exactly what is left and why.

## Development process: TDD is mandatory

1. **Red**: write a test for the behaviour and run it. Confirm it fails because the behaviour is
   missing, not because the test is broken.
2. **Green**: write the minimum production code that makes it pass.
3. **Refactor** with the test as the net, then re-run.

No production code for behaviour with no failing test. Exempt: build configuration
(`CMakeLists.txt`, presets, scripts) and data files. Anything with branching or arithmetic is
not exempt.

Test design rules are in DESIGN.md §6: assert on mechanism in unit tests and on outcome in
integration tests; displacement is never a break assertion; every generated test is seeded and
prints its failing seed; a green-on-arrival test is proven to bite by a recorded mutation.

### Building and running the tests

Tests live in `tests/`, one doctest file per feature, named `<Feature>Test.cpp`, and are listed
in `tests/CMakeLists.txt`. Test case names are dotted paths, `Area.Feature.Behaviour`, so a
doctest filter can select a feature.

```bash
powershell -NoProfile -ExecutionPolicy Bypass -File Scripts/Test.ps1
```

That configures on first run, builds Debug, and runs the whole suite. Narrow it with doctest's
own filters, which pass straight through:

```bash
powershell -NoProfile -ExecutionPolicy Bypass -File Scripts/Test.ps1 -tc="Model.*"
```

`-Config Release` builds and runs the optimised build. `Scripts/Build.ps1` builds without
running. `-Profile` on either script builds the phase-timing build instead (preset `msvc-profile`,
`LOADPATH_PROFILE=ON`, in `build/msvc-profile`), which fills the `PhaseTimings` on solver results;
the normal build reads no clock and leaves them 0.0.

**The default run is sharded**: `Test.ps1` splits it across processes (half the logical
cores, at most 8; `-Jobs N` overrides, `-Jobs 1` runs serially), each running a contiguous
range of the same filtered order, balanced by per-test durations cached in the build folder. A `-tc`
run stays serial unless `-Jobs` is given. Same tests, same totals; the Release default takes
about 20 s instead of about 60 s, then about 11 s more to rebuild the E2E picture page
(`Scripts/Visuals.ps1 -NoScreenshots`; `-NoVisuals` skips it). Never shard `Slow.*`: its budget rows assert on wall time.

**The exit code is meaningful**: non-zero when any assertion fails, or when a Release
default run finds an E2E case that recorded no picture or a refused one (a buried box), and the summary prints
to stdout. The suite is expected to run in seconds; a test that needs more than about a second
goes behind a `Slow.` prefix, which `Scripts/Test.ps1` excludes from the default run (it
prints how many it skipped). A case that is slow only in Debug puts `LOADPATH_SLOW_IN_DEBUG`
(`tests/SlowInDebug.h`) in front of its name: Debug skips it under `Slow.`, Release runs it under
its real prefix, so the Release default run is part of validating any such case. **The `Slow.*` tier
runs in Release at every slice close, by the orchestrator, before the review**: that is the
milestone that keeps an opt-in tier from rotting (TRAPS). `Measure.*` are timing rows
(SCALE.md) run one per process from a copy of the exe, never as a tier; the script excludes them
with the other two prefixes. The two cheap rows, `Measure.Scale.WallTenByEight` and
`Measure.Scale.Shed` (about 2 s together in Release), run at every slice close with the Slow
tier so the harness cannot rot.

**Each tier runs once per change** (owner, 2026-10-15). Repeating the same tier in every agent
cost 8 to 10 minutes per change:

| Who | Runs |
|---|---|
| test-expert | its own prefix (`-tc`), to see red; mutation proofs through `Scripts/Mutate.ps1` |
| dev-expert | its prefix while iterating, then the Release default run once at the end |
| orchestrator | the `Slow.*` tier and the two cheap `Measure.Scale` rows in Release, once, before the review |
| review-expert | the Release default run once, in the scratch worktree; another tier only when a result looks off |
| orchestrator | `Test.ps1 -Profile -Config Release -tc="Profile.*"` at every slice close, and the `Measure.Profile.ReadingsDigest` row in both builds (digests must match) whenever a change touches `src/PhaseClock.h` or an instrumented call; profiled-only assertions are mutation-proven with `Mutate.ps1 -Profile` |
| orchestrator | the Debug default run once, before the commit |
| orchestrator | `Scripts/Visuals.ps1` (with screenshots) once after a feature lands, then publishes `build/visuals/index.html` with `screenshots.html` to the fixed E2E page, https://claude.ai/artifact/SLuMLdeb5UNZh382sBvYgd; `Scripts/Visuals.Tests.ps1` only when `Visuals.ps1` changes. Every Release default run already rebuilds the 3D page without screenshots |

**The scratch worktree** (`Scripts/Scratch.ps1`, at `..\LoadPath-scratch`) is a persistent git
worktree with its own warm build folder. Sync it, then build and run there, for reviews,
mutants and measurements, so nothing locks or dirties this tree and builds stay incremental.
`-Rev <commit>` syncs a specific commit instead of the working tree. **Mutants go through
`Scripts/Mutate.ps1`**, which applies one find/replace in the scratch worktree, builds, runs the
named tests, prints CAUGHT or SURVIVED and restores the file byte for byte; never hand-edit a
source file in this tree to make a mutant.

**Deliberately red tests live under the `Deferred.` prefix.** A test that pins an answer the
current implementation cannot yet give (owed to a later slice, named in the test's second
segment, e.g. `Deferred.Slice4.*`) stays red there and the script excludes it from the default
run; it prints how many it skipped. Run them with `-tc="Deferred.*"`. Every one has a
CURRENT_STATE entry saying which slice turns it green and moves it back under its real prefix.

The compiler contract is enforced by the build: `/W4 /WX`, no exceptions, no RTTI,
`/fp:precise`. Code that needs any of those will not port to an Unreal module, so the build
refuses it here.

## Specialized agents

Development goes through three subagents, in this order. Never work the task directly.

| Agent | Owns | Entry condition |
|---|---|---|
| [test-expert](.claude/agents/test-expert.md) | Test design, assertion choice, confirming red | Start of every behaviour, feature or bug fix |
| [dev-expert](.claude/agents/dev-expert.md) | Implementation and refactoring | A failing test exists and has been run |
| [review-expert](.claude/agents/review-expert.md) | Test quality, TDD compliance, design alignment | Tests are green |

`dev-expert` and `test-expert` run on **Opus at low effort** (owner, 2026-10-11, a speed trial), `review-expert` on **Opus at high effort**, unless the owner says otherwise for a task; the
agent definitions carry the setting. They are subagents, not inline instructions: each starts
cold and reads `CLAUDE.md`, `DESIGN.md` and `CURRENT_STATE.md` itself. `dev-expert` enforces the
TDD gate by running the test itself; `review-expert` has no write access.

Invoke them with the Agent tool and `subagent_type`, or through the `/test-expert`,
`/dev-expert`, `/review-expert` skills, which are thin dispatchers. The definitions in
`.claude/agents/` are the single source of truth. Pass a subagent anything that exists only in
the conversation: the exact red output, values already worked out, constraints that must hold.

### The owner approves every test plan before test code is written

Owner, 2026-10-15. test-expert works in two steps:

1. **Plan.** It writes `claude_plans/test_plans/<YYYY-MM-DD>-<feature>.md` and no test code. One
   entry per case: the test name; the behaviour it pins, in a sentence; the fixture; the
   assertion and its kind (mechanism or outcome, DESIGN §6); why it is red today, or, for a
   regression net that is green on arrival, the mutant that will prove it bites; and its tier
   (default, `LOADPATH_SLOW_IN_DEBUG`, `Slow.`, `Measure.`, `Deferred.`). The file opens with a
   status line: `Status: proposed`. **Every case that involves a structure has pictures** (owner,
   2026-10-16): the structure before and the expected state after, as entries in the 3D viewer's
   data format, named on a `Visual:` line under the case; the orchestrator publishes the viewer
   page and the plan goes to the owner with its link. Only non-geometric cases (parsers, option
   checks) go without, and the entry says so. Once the scenario workbench exists (DESIGN §10,
   2026-10-16) a structural case is a scenario file, and its pictures come from the file.
2. **Tests, after approval.** The orchestrator relays the plan to the owner and waits. The owner
   approves it, edits it or rejects cases; the orchestrator records that on the status line
   (`Status: approved by the owner, <date>`, with any edits applied to the entries). Only then
   does test-expert write the tests, exactly as approved. A case it wants to add or change
   while writing goes back to the owner as an amendment to the plan, not into the code.

A review that asks for new tests produces a plan amendment the same way. The plan files stay in
the repository as the record of what was agreed.

Expect `dev-expert` to refuse when a test is not genuinely red and `review-expert` to send work
back. Act on blocking findings, log deferred ones in CURRENT_STATE.md, and do not call anything
done until review has passed.
