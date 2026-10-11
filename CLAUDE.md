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
powershell -NoProfile -File Scripts/Test.ps1
```

That configures on first run, builds Debug, and runs the whole suite. Narrow it with doctest's
own filters, which pass straight through:

```bash
powershell -NoProfile -File Scripts/Test.ps1 -tc="Model.*"
```

`-Config Release` builds and runs the optimised build. `Scripts/Build.ps1` builds without
running. **The exit code is meaningful**: non-zero when any assertion fails, and the summary
prints to stdout. The suite is expected to run in seconds; a test that needs more than about a
second goes behind a `Slow.` prefix and is excluded from the default run by the script once one
exists.

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

All three run on **Opus 5.5 at high effort** unless the owner says otherwise for a task; the
agent definitions carry the setting. They are subagents, not inline instructions: each starts
cold and reads `CLAUDE.md`, `DESIGN.md` and `CURRENT_STATE.md` itself. `dev-expert` enforces the
TDD gate by running the test itself; `review-expert` has no write access.

Invoke them with the Agent tool and `subagent_type`, or through the `/test-expert`,
`/dev-expert`, `/review-expert` skills, which are thin dispatchers. The definitions in
`.claude/agents/` are the single source of truth. Pass a subagent anything that exists only in
the conversation: the exact red output, values already worked out, constraints that must hold.

Expect `dev-expert` to refuse when a test is not genuinely red and `review-expert` to send work
back. Act on blocking findings, log deferred ones in CURRENT_STATE.md, and do not call anything
done until review has passed.
