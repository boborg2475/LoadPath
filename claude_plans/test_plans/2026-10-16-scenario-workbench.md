Status: approved 2026-10-17 (owner: "create the tests according to your plans"; the recommendations below are the answers)

# Scenario workbench: test plan (2026-10-16)

The owner's ask (DESIGN §10, "2026-10-16, owner, visuals and the scenario workbench"): build a
structure quickly in a short file, see it, and run a test against it. This plan covers the
workbench itself: the file format, the loader, automatic joints, the runner, expectations, the
viewer export, the CLI command, the generic test that runs every file, and (amendment, 2026-10-17)
the pictures every E2E test records for itself, with `Scripts/Visuals.ps1` collecting them into
the E2E page. The physics cases that
become its first files are the e2e plan's (2026-10-15); they are not repeated here.

Nothing exists yet, so every case below is red today: the header `include/loadpath/Scenario.h`
does not exist, and neither does the `scenario` CLI command or `data/scenarios/`. When the tests
are written, the smallest declarations needed to compile go in first, so each case fails on its
assertion, not on a missing symbol.

**Pictures.** Every case with a structure names an entry on a `Visual:` line. They are on the viewer page
https://claude.ai/artifact/5VPVB9mvJaB35oXsVbZs4q, published from the scratchpad folder `workbench_viz\` (the viewer page retitled "LoadPath scenario workbench plan",
`data\*.json`, `data\index.json`), made by a probe in `workbench_probe\` that builds each fixture
with the existing library (`Box`, `FaceContact`, `BuildRunningBondWall`) at the scratch build of
553611c. The page shows the structure before and the state the plan expects after: released
bodies are coloured by cluster, removed bodies are dashed ghosts. Each entry's note says what the
engine does today on the same fixture. On every fixture the probe ran, the engine agrees with the
expectation, except the deliberately wrong one.

## 1. The file format

### A complete example: `data/scenarios/colonnade-middle-post.json`

```json
{
  "title": "Colonnade losing its middle post",
  "summary": "Five dry timber posts carry four beams. The middle post goes; the two beams it carried fall, each on its own.",
  "build": [
    { "name": "ground", "size": [12, 2, 0.2], "centre": [4.4, 0, -0.1], "grounded": true },
    { "name": "post", "size": [0.2, 0.2, 2.0], "centre": [0, 0, 1.0], "material": "c24",
      "repeat": { "count": 5, "step": [2.2, 0, 0] } },
    { "name": "beam", "size": [2.199, 0.2, 0.3], "centre": [1.1, 0, 2.15], "material": "c24",
      "repeat": { "count": 4, "step": [2.2, 0, 0] } }
  ],
  "joints": [
    { "materials": ["c24", "c24"], "row": "c24_bearing_perp_to_grain" },
    { "materials": ["c24", "ground"], "row": "c24_bearing_perp_to_grain" }
  ],
  "steps": ["cascade", { "remove": ["post.2"] }, "cascade"],
  "expect": { "released": [["beam.1"], ["beam.2"]] }
}
```

That builds 10 bodies (the ground, `post.0` to `post.4`, `beam.0` to `beam.3`) and 13 joints (5
post bases, 8 beam ends; the beams are 1 mm apart end to end, so they do not touch). `state`
defaults to `Stands`, and every body not listed under `released` must still be live at the end.

A wall with an opening and a timber lintel, to show the other build item:

```json
{ "wall": "wall", "courses": 10, "bricks": 8, "origin": [0, 0, 0], "on": "ground",
  "material": "clay_brick", "bed": "dry_brick", "perpend": "dry_brick",
  "omit": ["wall.c[0-4].b[2-5]", "wall.c5.b[2-6]"] },
{ "name": "lintel", "size": [1.075, 0.1025, 0.065], "centre": [0.86, 0, 0.3575], "material": "c24" }
```

The lintel replaces course 5 over x 0.3225 to 1.3975 m and bears half a brick on each side. The
jambs are ragged, because running bond staggers them; e2e 3.13's straight jambs need cut bricks,
which a file writes as boxes.

### Fields

Top level. Unknown keys are an error everywhere, as in the structure file.

| Key | Type | Default | Notes |
|---|---|---|---|
| `title` | string | required | Shown in the CLI output and the viewer. |
| `summary` | string | `""` | One plain sentence. |
| `deferred` | string | absent | The slice or item that owes this answer. Empty string is an error. |
| `tier` | `"default"`, `"slow_in_debug"`, `"slow"` | `"default"` | Which generic test runs the file (section 4). |
| `materials` | array | `[]` | Materials local to this file: `{ "name", "density_kg_per_m3" }`. A name the catalogue already has is an error (no shadowing). |
| `profiles` | array | `[]` | Joint rows local to this file: `{ "name", "from": <catalogue joint row>, ...overrides }`. Overrides use the catalogue's keys and units (`cohesion_mpa`, `tensile_mpa`, `friction`, `shear_cap_mpa`, ...); `"shear_cap_mpa": null` removes the cap. Converted to Pa once, by the loader. |
| `build` | array | required | Boxes and walls, below. |
| `joints` | array | `[]` | Rules, below. |
| `steps` | array | `["cascade"]` | Run in file order, below. |
| `expect` | object | absent | Required for a file under `data/scenarios/`; optional for a sketch run from anywhere else. |

A **box** in `build`:

| Key | Type | Default | Notes |
|---|---|---|---|
| `name` | string | required | Letters, digits, `_`, `-`. No dots: dots are reserved for generated names. Unique across the file. |
| `size` | [x, y, z] m | required | Every component finite and > 0. |
| `centre` | [x, y, z] m | required | Finite. Boxes are axis-aligned; there is no rotation. |
| `material` | string | `"ground"` if grounded, else required | Catalogue material or a file-local one. Mass is density × volume; a grounded box has mass 0 whatever its material. |
| `grounded` | bool | `false` | |
| `repeat` | `{ "count", "step" }` | absent | `count` an integer ≥ 1, `step` [x, y, z] m finite. Makes `name.0` to `name.<count-1>`, copy k at `centre + k × step`. |

A **wall** in `build` (the existing generator, `geometry::BuildRunningBondWall`):

| Key | Type | Default | Notes |
|---|---|---|---|
| `wall` | string | required | The wall's name; its bricks are `<wall>.c<course>.b<brick>`. |
| `courses`, `bricks` | integer ≥ 1 | required | Bricks per even course; odd courses have one more piece (two half bricks at the ends). |
| `origin` | [x, y, z] m | `[0, 0, 0]` | The wall's start, base level, centre line. |
| `along` | `"x"` or `"y"` | `"x"` | |
| `brick` | [l, t, h] m | `[0.215, 0.1025, 0.065]` | |
| `bond` | `"running"` or `"stack"` | `"running"` | |
| `on` | box name | required | A grounded box whose top is at the wall's base level (error otherwise). The base course's beds join it. |
| `material` | string | required | For mass and for rules against other boxes. |
| `bed`, `perpend` | joint row or file profile | required | The wall's own joints, as the generator makes them. |
| `omit` | patterns | `[]` | Bricks never built: no body, no joint, no name an expectation may use, no ghost. A pattern matching no brick is an error. |

Brick names count from 0: course 0 is the bottom, brick 0 is at the wall's start (lowest x, or
lowest y for `along: "y"`). In running bond, odd courses run `b0` (half) to `b<bricks>` (half).

A **rule** in `joints`. Every pair of boxes touching on a face (`FaceContact`, tolerance 1e-9 m)
gets one joint, except pairs the generator already joined (two bricks of the same wall, or a base
brick and its wall's `on` box). The joint's row is the first matching rule: **every `between`
rule is tried before any `materials` rule**, and within each kind the first in file order wins.

| Key | Type | Notes |
|---|---|---|
| `between` | [pattern, pattern] | Matches a pair whose two names match the two patterns, in either order. |
| `materials` | [string, string] | Matches a pair with these two materials, in either order. Exactly one of `between` and `materials`. |
| `plane` | `"bed"` or `"side"` | Optional: only joints whose contact plane is horizontal (`bed`, normal along z) or vertical (`side`). |
| `row` | string | A catalogue joint row, a file profile, or a catalogue fastener row. |
| `count` | integer ≥ 1 | Required with a fastener row, an error with a joint row. The profile is `FastenerProfile(row, count, area)` with the joint's own contact area. |

A touching pair that no rule matches is an error naming both boxes and their materials. Boxes that
overlap in volume (more than the tolerance on all three axes) are an error naming both. Touching
along an edge or at a corner is not a joint and needs no rule.

A **step** is a bare string or an object with one key:

| Step | Effect |
|---|---|
| `"cascade"` | Runs the cascade (`MakeCascade(MakeCombinedAuthority(...))`, default `CascadeOptions`) with the current loads and gravity; its releases join the run's record. |
| `"judge"` | One judge, nothing released; its verdict is printed and recorded. |
| `{ "remove": [names or patterns] }` | Removes bodies. A name already removed or released when the step runs is a run error. |
| `{ "load": { "body", "force_n": [x, y, z], "point": [x, y, z] } }` | Adds a load for every later judge and cascade. `point` defaults to the body's centroid. Loads accumulate. |
| `{ "lateral_g": [ax, ay] }` | Sets the horizontal acceleration, as fractions of g, for every later judge and cascade: gravity becomes (ax g, ay g, −g). A later `lateral_g` replaces it. |

`expect`:

| Key | Type | Default | Notes |
|---|---|---|---|
| `state` | cascade state name | `"Stands"` | The last cascade's (or judge's) state. |
| `released` | list of patterns, or list of lists | `[]` | A flat list asserts the set of released bodies and nothing about clusters. A list of lists asserts the partition: each inner list is one cluster. |
| `either` | patterns | `[]` | Bodies whose fate is not asserted (a brick whose centroid sits exactly on a support edge). |
| `passes` | integer | not asserted | Total counted passes over every cascade step. |
| `cracked` | joint names | not asserted | Joints named `<a>/<b>`, a pattern on each side. |

Every body that is not removed, not listed in `released` and not in `either` must be live at the
end. A pattern in `expect` that matches no body, or that names a removed body, is a load error.

**Patterns.** `*` matches any run of characters, dots included; `[a-b]` matches one integer from a
to b inclusive. Everything else is literal. So `post.*` is every post, `wall.c3.*` is course 3,
`wall.c*.b0` is the first brick of every course, `wall.c[6-9].b[2-5]` is a block of bricks.

**Materials.** The catalogue gains a `materials` table (data, plus a loader change tested below):
`clay_brick` at 2000 kg/m³ (DESIGN §3) and `c24` at 420 kg/m³ (EN 338 ρ_mean), each with a
source, like every other row.

### API (a proposal, so the tests have something to call; dev-expert may refine names)

- `LoadScenario(std::string_view Json, const Catalogue&) -> ScenarioLoadResult` (value plus
  `FileError`, `Ok()`), which parses, builds the `Structure` and finds the joints. The value keeps
  each body's name, box and material by slot, the steps and the expectation.
- `RunScenario(Scenario&, const ICascade&, const IStructuralAuthority&) -> ScenarioOutcome`, and an
  overload that makes the standard engine. Injecting the cascade lets a test record what the
  runner hands it (DESIGN §7: tests run against the interface).
- `CompareExpectation(const ScenarioExpect&, const ScenarioOutcome&) -> ExpectationReport`, a pure
  function over names.
- `WriteViewerJson(const Scenario&, const ScenarioOutcome&, std::string_view Slug) -> std::string`,
  built on `WriteViewerJson(const ViewerScene&)`, the one format writer, which code-based E2E
  tests also use (see "Visuals from the tests").
- `RunCli` gains the `scenario` command. It reads and writes through a small file interface the
  caller supplies, so the CLI tests use an in-memory map and `cli/Main.cpp` uses the disk.

## 2. Test cases

Prefixes: `Scenario.Load.*`, `Scenario.Joints.*`, `Scenario.Build.*`, `Scenario.Steps.*`,
`Scenario.Expect.*`, `Scenario.Viewer.*` in `tests/ScenarioTest.cpp` (split by area if it grows);
`Cli.Scenario.*` in `tests/CliTest.cpp`; `Catalogue.Materials.*` in `tests/CatalogueTest.cpp`;
the generic tests in `tests/ScenarioFilesTest.cpp`; the visuals cases where their section says.
Fixtures are inline JSON in the test, except
the generic tests, which read `data/scenarios/`. Every case is `default` tier unless it says
otherwise, and all of them together run in well under a second in Release.

### Loader

**S1. `Scenario.Load.TheColonnadeExampleParses`** (must-have)
- Behaviour: the section 1 example loads into 10 named bodies and 13 joints, with the right boxes.
- Fixture: the example, inline.
- Assertion (mechanism): `Ok()`; names in slot order `ground, post.0..post.4, beam.0..beam.3`;
  `post.3`'s centre is `[6.6, 0, 1.0]` to 1e-12; ground grounded, mass 0; 13 live interfaces, all
  on the bearing row's profile; no `beam.k`/`beam.k+1` joint.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

**S2. `Scenario.Load.ErrorsNameTheLineAndTheProblem`** (must-have)
- Behaviour: a bad file is refused with the line of the offending element and a message naming the
  problem; nothing is half-built.
- Fixture: a table of small documents, one fault each: an unknown key; a missing `size`; a zero,
  negative or `1e999` size component; an unknown material; an unknown row in a rule; a fastener row
  without `count`; a joint row with `count`; `count` 0 or 2.5 on a rule; a rule with both
  `between` and `materials`; `repeat.count` 0, −1 or 2.5; a non-finite `repeat.step`; a name with a
  dot; a duplicate name (including one a `repeat` generates); a wall whose `on` is not grounded or
  whose top is not at the wall's base; an `omit` pattern matching nothing; a step with two keys or
  an unknown verb; a `remove` naming no body; an `expect` pattern matching nothing or naming a
  removed body; a file `material` or `profile` shadowing a catalogue name; `profiles.from` unknown;
  `deferred: ""`; an unknown `tier`.
- Assertion (mechanism): `!Ok()`, `Error.Line` equals the line of the element, and the message
  contains the row's key phrase (for example `unknown material 'oak'`).
- Red today: nothing exists.
- Visual: none: not a structure (the faults are in the text).

**S3. `Scenario.Load.OverlappingBoxesAreAnErrorNamingBothBoxes`** (must-have)
- Behaviour: two boxes that overlap in volume refuse the file, naming both; `FaceContact` alone
  would silently skip the pair.
- Fixture: bricks `a` and `b` on the ground, `b` 10 mm into `a`. A second row overlaps by exactly
  the tolerance (accepted: they touch).
- Assertion (mechanism): `!Ok()`, line of `b`, message names `a` and `b`; the tolerance row loads.
- Red today: nothing exists.
- Visual: `overlapping-boxes-refused`.

**S4. `Scenario.Load.ATouchingPairWithNoRuleIsAnErrorNamingThePair`** (must-have)
- Behaviour: no joint is ever guessed.
- Fixture: a brick on a timber block on the ground; the only rule is `materials: [c24, ground]`.
- Assertion (mechanism): `!Ok()`, line of `brick`, message names `block / brick (c24 /
  clay_brick)`.
- Red today: nothing exists.
- Visual: `missing-rule-refused`.

**S5. `Scenario.Load.RepeatMakesNumberedCopiesAlongItsStep`** (must-have)
- Behaviour: `repeat` makes `name.0` to `name.<n-1>`, copy k at `centre + k × step`, each with the
  same size, material and mass.
- Fixture: the example's posts and beams.
- Assertion (mechanism): five posts at x = 0, 2.2, 4.4, 6.6, 8.8 (to 1e-12), four beams at 1.1 +
  2.2k; a box without `repeat` keeps its plain name.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

**S6. `Scenario.Load.TheWallGivesItsBricksCourseAndBrickNames`** (must-have)
- Behaviour: a wall item builds exactly the generator's wall, and its bricks are named
  `wall.c<course>.b<brick>`.
- Fixture: a 3-course, 4-brick EC6 wall on a ground box.
- Assertion (mechanism): 13 bricks; `wall.c1.b0` and `wall.c1.b4` are half bricks; every brick's
  centroid and mass equal those of `BuildRunningBondWall` with the same spec, built directly in the
  test; 20 beds (4 on the ground) and 10 perpends, the same interface records as the direct build.
- Red today: nothing exists.
- Visual: `wall-brick-names`.

**S7. `Scenario.Load.OmittedWallBricksAreNeverBuilt`** (must-have: openings need it)
- Behaviour: `omit` leaves a hole with no body, no joint and no usable name.
- Fixture: a 6 × 6 EC6 wall with `omit: ["wall.c[0-2].b[2-3]"]`.
- Assertion (mechanism): 33 live bricks plus the ground; none of the six names resolves (an
  `expect` naming `wall.c1.b2` is a load error); no live interface touches an omitted slot.
- Red today: nothing exists.
- Visual: `wall-with-opening`.

### Joints

**J1. `Scenario.Joints.EveryFaceTouchingPairGetsOneJointWithFaceContactsRectangle`** (must-have)
- Behaviour: every face-touching pair gets exactly one joint, equal to `FaceContact` on the two
  boxes; gaps and edge contacts get none.
- Fixture: five bricks on a ground slab: `a` and `b` side by side, `c` on top across both, `d` 1 mm
  from `b`, `e` touching `d` only along a vertical edge. Rules: `plane: bed` and `plane: side`.
- Assertion (mechanism): seven interfaces (four on the ground, `a/b`, `a/c`, `b/c`), each equal
  field by field to `FaceContact` called in the test on the same boxes (normal, centre, axes, half
  extents); no `b/d`, no `d/e`.
- Red today: nothing exists.
- Visual: `contacts-mixed`.

**J2. `Scenario.Joints.APlaneRuleSeparatesBedsFromPerpends`** (must-have)
- Behaviour: `plane: bed` matches only horizontal contacts and `plane: side` only vertical ones.
- Fixture: as J1, bed rule → EC6 bed, side rule → EC6 perpend, listed side first.
- Assertion (mechanism): `a/b` has the perpend row's profile; the other six have the bed row's.
- Red today: nothing exists.
- Visual: `contacts-mixed`.

**J3. `Scenario.Joints.ABodyPairRuleBeatsAMaterialRuleWhateverTheirOrder`** (must-have)
- Behaviour: a `between` rule wins over a `materials` rule even when listed after it; within a
  kind, the first in file order wins.
- Fixture: one bay (`post.0`, `post.1`, `beam.0`). Rows: (a) `materials c24/c24 → bearing`, then
  `between [beam.0, post.1] → screw, count 3`; (b) the same two rules in the other order; (c) two
  `materials` rules for c24/c24 with different rows.
- Assertion (mechanism): in (a) and (b) only `beam.0/post.1` carries the screw profile; in (c)
  every c24 joint carries the first rule's row.
- Red today: nothing exists.
- Visual: `rule-precedence-screwed-pair` (with `post.0` removed, the screwed pair tips together;
  had the material rule won, `beam.0` would fall alone; the engine agrees today).

**J4. `Scenario.Joints.AFastenerRuleGivesFastenerProfileForTheContactArea`** (must-have)
- Behaviour: a fastener rule's profile is `FastenerProfile(row, count, area)` with the joint's own
  contact area.
- Fixture: J3 row (a). The contact is 0.0995 × 0.2 m.
- Assertion (mechanism): the profile equals, bit for bit, `FastenerProfile(ec5_screw…, 3, 4 ×
  HalfExtentU × HalfExtentV)` computed in the test from the interface record; and the area is
  0.0199 m² to 1e-12, written out as 0.0995 × 0.2.
- Red today: nothing exists.
- Visual: `rule-precedence-screwed-pair`.

**J5. `Scenario.Joints.WallBricksKeepTheGeneratorsJointsAndMeetOtherBoxesByRule`** (must-have)
- Behaviour: bricks of one wall, and a base brick and its `on` box, are never joined twice; other
  boxes meet bricks through rules.
- Fixture: a 4 × 4 EC6 wall with a C24 beam along its top; rule `materials [c24, clay_brick] →
  bearing`.
- Assertion (mechanism): 28 beds and 14 perpends with the generator's profiles, plus exactly 5
  beam joints (the top course is odd: two halves and three full) on the bearing row; no two live
  interfaces share a body pair.
- Red today: nothing exists.
- Visual: `wall-with-timber-beam`.

**J6. `Scenario.Joints.ProfilesAreSharedPerRow`** (nice-to-have)
- Behaviour: one profile per joint row and per (fastener row, count, area), so a big file does not
  register thousands of copies.
- Fixture: the example plus J3 (a).
- Assertion (mechanism): the example registers 1 profile; J3 (a) registers 2.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

### Build

**B1. `Scenario.Build.MassIsDensityTimesBoxVolume`** (must-have)
- Behaviour: a body's mass is its material's density times its box volume; grounded is 0.
- Fixture: the cantilever: a C24 post 0.2 × 0.2 × 1.0 m, a C24 beam 1.6 × 0.2 × 0.1 m, a 0.2 m
  cube of file material `light_block` at 500 kg/m³.
- Assertion (mechanism): masses equal 420 × 0.04, 420 × 0.032 and 500 × 0.008 kg, each written out
  in the test from the published densities (16.8, 13.44, 4.0); the ground is 0.
- Red today: nothing exists.
- Visual: `cantilever-light-counterweight`.

**B2. `Scenario.Build.FileLocalProfilesDeriveFromACatalogueRow`** (must-have: the dry-brick rows
of the e2e plan need it)
- Behaviour: a file profile copies its `from` row and overrides only the keys it gives, converted
  from MPa once.
- Fixture: `dry_brick` from `clay_gp_mortar_bed_ec6` with `cohesion_mpa 0`, `tensile_mpa 0`,
  `friction 0.7`, `shear_cap_mpa null`.
- Assertion (mechanism): cohesion and tension 0, μ 0.7, cap +inf, compressive strength and both
  stiffnesses equal the catalogue row's.
- Red today: nothing exists.
- Visual: none: compares profile numbers.

**B3. `Catalogue.Materials.RowsCarryTheirDensityAndRefuseBadOnes`** (must-have)
- Behaviour: the catalogue's `materials` table loads, and a bad row is refused with its line.
- Fixture: the checked-in `data/catalogue.json`; inline documents with a missing, zero, negative or
  non-finite density, a missing source, a duplicate name.
- Assertion (mechanism): `clay_brick` 2000 and `c24` 420 kg/m³ found by name; each bad document
  refused at its line.
- Red today: the loader refuses a `materials` key.
- Visual: none: not a structure.

### Steps

**T1. `Scenario.Steps.RunInFileOrder`** (must-have)
- Behaviour: steps run in file order, and each cascade sees the structure as the earlier steps
  left it.
- Fixture: the example, run with a recording `ICascade` that logs, per call, the live body names
  and the gravity vector, then delegates to the real cascade.
- Assertion (mechanism): two cascade calls; the first sees `post.2` live, the second does not; the
  step records read cascade (nothing released), remove `post.2`, cascade (`{beam.1}`, `{beam.2}`).
  The same file with the remove before the first cascade gives one fewer call with releases.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

**T2. `Scenario.Steps.LoadAndLateralStepsReachTheCascadeExactlyAsTheApiWould`** (must-have)
- Behaviour: the runner hands the cascade exactly the loads and gravity a direct API call would.
- Fixture: rows: (a) a 5 × 4 EC6 wall, `lateral_g [0, 1.02 λc]` with λc = (t/H)(1 + f_t A_base /
  W) = 1.9646 computed in the test from the row's numbers, then cascade; (b) a C24 beam on two
  five-brick EC6 piers, a load putting each bearing at 1.2 f_c,90 A (P = 2(1.2 f_c,90 A − W_beam / 2)),
  then cascade.
- Assertion (mechanism): the runner's verdict equals, field by field (state, passes, failures,
  clusters by slot), a `MakeCascade(...)->Run` on the same structure built directly with
  `FaceContact` and the same options, gravity (0, 1.02 λc g, −g) in (a) and the `BodyLoad` at the
  beam's centroid in (b).
- Red today: nothing exists. The engine today releases all 22 bricks as one slab after cracking
  the 4 base beds in (a), and `{beam}` in (b), so the files can also carry those expectations.
- Visual: `bonded-wall-lateral-step`, `beam-on-piers-load-step`.

**T3. `Scenario.Steps.RemovingABodyAlreadyGoneIsARunError`** (nice-to-have)
- Behaviour: removing a body a cascade already released fails the run, naming the body and the
  step, instead of passing silently.
- Fixture: the example with a fourth step `{ "remove": ["beam.1"] }`.
- Assertion (mechanism): the outcome carries a run error naming `beam.1` and step 4; the
  expectation report reads not met.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

**T4. `Scenario.Steps.AJudgeStepReleasesNothing`** (nice-to-have)
- Behaviour: `judge` records a verdict and changes nothing.
- Fixture: the example with `steps: [{ "remove": ["post.2"] }, "judge"]`.
- Assertion (mechanism): the recorded verdict is Collapses; every body except `post.2` is still
  live; no interface changed state.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

### Expectations

**E1. `Scenario.Expect.TheColonnadeMeetsItsExpectation`** (must-have)
- Behaviour: the example file's run meets its expectation, end to end through the real engine.
- Fixture: the example.
- Assertion (outcome): the report is met and empty.
- Red today: nothing exists. The engine today releases exactly `{beam.1}` and `{beam.2}` in one
  pass.
- Visual: `colonnade-middle-post`.

**E2. `Scenario.Expect.AMismatchNamesEveryBodyThatDiffers`** (must-have)
- Behaviour: an unmet expectation lists each body that differs, in both directions.
- Fixture: the example with `released: [["beam.1"], ["post.1"]]`.
- Assertion (mechanism): not met; the report's lines are exactly `released, not expected: beam.2`,
  `expected to fall, still standing: post.1`, and the cluster line `expected {beam.1} {post.1};
  engine {beam.1} {beam.2}`.
- Red today: nothing exists.
- Visual: `colonnade-wrong-expectation` (the picture shows the wrong expectation).

**E3. `Scenario.Expect.ClustersAreComparedAsAPartition`** (must-have)
- Behaviour: a list of lists compares the partition; a flat list compares only the set.
- Fixture: `CompareExpectation` on synthetic outcomes: engine `{a} {b}` against `[["a", "b"]]`
  (not met: cluster line), against `["a", "b"]` (met), against `[["b"], ["a"]]` (met: order free).
- Assertion (mechanism): met or not as listed; the cluster line names both partitions.
- Red today: nothing exists.
- Visual: none: compares name sets.

**E4. `Scenario.Expect.PatternsExpandOverNames`** (must-have)
- Behaviour: `*` crosses dots, `[a-b]` is an inclusive integer range, everything else is literal.
- Fixture: names of a 10 × 8 wall plus `post.0`..`post.4`, `post`, `postbox`.
- Assertion (mechanism): `post.*` gives the five posts and not `post` or `postbox`; `wall.c3.*`
  gives the 9 bricks of course 3; `wall.c*.b0` gives 10; `wall.c[6-9].b[2-5]` gives 16;
  `wall.c[9-6].*` is a load error; a pattern matching nothing is a load error.
- Red today: nothing exists.
- Visual: none: names only.

**E5. `Scenario.Expect.DefaultsAreStandsAndNothingElseFalls`** (must-have)
- Behaviour: `state` defaults to Stands and `released` to nothing, so a short file is strict.
- Fixture: synthetic outcomes against `"expect": {}`: Stands with nothing released (met);
  Undecided with nothing released (not met: `state: expected Stands, engine Undecided`); Stands with
  `x` released (not met: `released, not expected: x`).
- Assertion (mechanism): as listed.
- Red today: nothing exists.
- Visual: none: compares names and states.

**E6. `Scenario.Expect.EitherBodiesAreNotAsserted`** (must-have: e2e 3.13 needs it)
- Behaviour: a body under `either` may fall or stand.
- Fixture: synthetic outcomes, `either: ["w.c6.b0"]`, with and without `w.c6.b0` released; a
  partition expectation that also lists it is a load error.
- Assertion (mechanism): met both ways; the overlap is refused.
- Red today: nothing exists.
- Visual: none: names only.

**E7. `Scenario.Expect.PassesAndCrackedAreCheckedOnlyWhenGiven`** (nice-to-have)
- Behaviour: `passes` and `cracked` are asserted when present and ignored when absent.
- Fixture: synthetic outcomes with 2 passes and `w.c0.b1/ground` cracked.
- Assertion (mechanism): `passes: 2` met, `passes: 1` not met with a `passes` line; `cracked:
  ["w.c0.*/ground"]` met; absent keys never produce a line.
- Red today: nothing exists.
- Visual: none: names only.

**E8. `Scenario.Expect.TheDeferredFieldIsKeptAndReported`** (must-have)
- Behaviour: `deferred` is read and kept, and changes nothing about how the file runs.
- Fixture: the example plus `"deferred": "Slice6 least-joint-movement selection"`.
- Assertion (mechanism): the loaded value carries the string; the outcome and report equal the
  undeferred file's.
- Red today: nothing exists.
- Visual: none: a text field.

### Viewer export

**V1. `Scenario.Viewer.TheExportCarriesBodiesBeforeAndAfter`** (must-have)
- Behaviour: the export is the viewer's format: every body with its box, mass, material and name;
  the joints; the removed bodies as ghosts; the verdict before; the engine's after.
- Fixture: the example, run.
- Assertion (mechanism): parsed back in the test with the library's JSON reader: `slug`, `title`,
  `summary`, `units: "m"`, `gravity`; 10 bodies with `c` and `h` equal to the boxes, `q` identity,
  `grounded`, `mass_kg`, `material`, `name`; 13 interfaces with `a`, `b`, `p`, `n`;
  `removed_before_run` is `[post.2's slot]`; `before.verdict` is `Stands` (the judge of the
  structure as built, before any step); `after.state` `Stands`, `after.passes` 1, two clusters
  `[beam.1]` and `[beam.2]` by slot, each with its `motion`; no `expected` key.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

**V2. `Scenario.Viewer.TheExpectedAfterAppearsOnlyWhenItDiffers`** (must-have)
- Behaviour: when the engine and the file disagree, the export also carries the expected after, so
  the page can show both.
- Fixture: the E2 file, and the example (agrees).
- Assertion (mechanism): E2's export has `expected` with `state` and `clusters` `[beam.1]`,
  `[post.1]` by slot; the example's has none.
- Red today: nothing exists.
- Visual: `colonnade-wrong-expectation`.

**V3. `Scenario.Viewer.WallBricksExportTheirBoxes`** (must-have)
- Behaviour: generator bricks export their true boxes, half bricks included.
- Fixture: the S6 wall.
- Assertion (mechanism): `wall.c1.b0` has `h` [0.05375, 0.05125, 0.0325], `wall.c0.b0` has
  [0.1075, 0.05125, 0.0325], each centre equal to the body's centroid.
- Red today: nothing exists.
- Visual: `wall-brick-names`.

**V4. `Scenario.Viewer.OmittedBricksAreNotExported`** (must-have)
- Behaviour: an omitted brick is not a ghost; it never existed.
- Fixture: the S7 wall.
- Assertion (mechanism): 34 bodies in the export, no omitted name, `removed_before_run` empty.
- Red today: nothing exists.
- Visual: `wall-with-opening`.

**V5. `Scenario.Viewer.TheExportIsByteIdenticalRunToRun`** (nice-to-have)
- Behaviour: the export holds no clock reading, so two runs of a file write the same bytes and a
  diff of the page data means the answer changed.
- Fixture: the example, run twice from two loads.
- Assertion (mechanism): the two strings are equal; there is no `elapsed_ms` key.
- Red today: nothing exists.
- Visual: `colonnade-middle-post`.

### CLI

**C1. `Cli.Scenario.AMetScenarioPrintsTheStepsAndReturns0`** (must-have)
- Behaviour: `scenario <file>` prints what was built, each step's result and `expectation met`,
  and returns 0.
- Fixture: the example and the catalogue in the in-memory file map.
- Assertion (mechanism): exit 0; `Out` contains `built 10 bodies (1 grounded), 13 joints`, `step 3
  cascade: Stands, 1 pass, released {beam.1} {beam.2}` and `expectation met`; `Err` empty.
- Red today: the CLI takes one argument and prints usage.
- Visual: `colonnade-middle-post`.

**C2. `Cli.Scenario.AnUnmetScenarioPrintsTheMismatchAndReturns1`** (must-have)
- Behaviour: an unmet expectation returns 1 and prints the E2 report.
- Fixture: the E2 file.
- Assertion (mechanism): exit 1; `Out` contains `expectation not met` and the E2 lines.
- Red today: as C1.
- Visual: `colonnade-wrong-expectation`.

**C3. `Cli.Scenario.ABadFileReturns2WithTheFileAndLine`** (must-have)
- Behaviour: a file that does not load, or cannot be read, returns 2 and says where.
- Fixture: a file with an unknown material on line 7; a missing file; a missing catalogue.
- Assertion (mechanism): exit 2; `Err` is `x.json:7: unknown material 'oak'`, `cannot read x.json`,
  `cannot read <catalogue path>`.
- Red today: as C1.
- Visual: none: not a structure.

**C4. `Cli.Scenario.VizWritesTheViewerJson`** (must-have)
- Behaviour: `--viz <out>` writes the V1 export to that path.
- Fixture: the example; `scenario colonnade.json --viz out.json`.
- Assertion (mechanism): the map holds `out.json` equal to `WriteViewerJson` called directly; the
  exit code is unchanged by the flag.
- Red today: as C1.
- Visual: `colonnade-middle-post`.

**C5. `Cli.Scenario.UsageErrorsReturn2`** (must-have)
- Behaviour: `scenario` with no file, `--viz` with no path, or an unknown flag prints usage and
  returns 2; the existing one-argument structure command is unchanged.
- Fixture: those argument lists.
- Assertion (mechanism): exit 2 and a usage line; the existing `Cli.*` cases stay green.
- Red today: as C1 (the usage text does not mention `scenario`).
- Visual: none: not a structure.

**C6. `Cli.Scenario.TheCatalogueDefaultsToTheDataFolder`** (nice-to-have)
- Behaviour: without `--catalogue`, the CLI reads `catalogue.json` from the scenario folder's
  parent (`data/scenarios/x.json` → `data/catalogue.json`); `--catalogue <path>` overrides it.
- Fixture: the file map with the catalogue in each place.
- Assertion (mechanism): the path read is the expected one in both rows.
- Red today: as C1.
- Visual: none: not a structure.

### The generic tests

**G1. `E2E.Scenarios.EveryScenarioFileMeetsItsExpectation`** (must-have)
- Behaviour: every non-deferred `default`-tier file under `data/scenarios/` loads, runs and meets
  its expectation; a failure names the file.
- Fixture: the folder, through a compile definition `LOADPATH_SCENARIO_DIR` (as
  `LOADPATH_CATALOGUE_PATH` is today), listed in sorted name order with the non-throwing
  `std::filesystem` overloads.
- Assertion (outcome): one `SUBCASE` per file, named by the file; in each, `CHECK` the load
  (message: `<file>:<line>: <error>`), that `expect` is present, and that the report is met
  (message: the report). Zero files found is a failure (`no scenario files in <dir>`), so a wrong
  path can never pass vacuously.
- Red today: the folder and the runner do not exist. Bite proof once green: `Scripts/Mutate.ps1`
  changing the example's `beam.2` to `beam.3` makes it fail naming `colonnade-middle-post.json`.
- Tier: default. Files tagged `slow_in_debug` run in G2, `slow` in G3.
- Visual: none of its own; each file's pictures come from the file.

**G2. `LOADPATH_SLOW_IN_DEBUG E2E.Scenarios.EverySlowInDebugScenarioFileMeetsItsExpectation`**
(must-have)
- As G1 for files tagged `slow_in_debug` (the arch-sized and 10 × 8 wall cases): Release runs it
  in the default run, Debug skips it under `Slow.`. It passes with zero such files.

**G3. `Slow.Scenarios.EverySlowScenarioFileMeetsItsExpectation`** (nice-to-have)
- As G1 for files tagged `slow`, in the `Slow.*` tier the orchestrator runs at slice close. It
  passes with zero such files.

**G4. `Deferred.Scenarios.EveryDeferredScenarioFileMeetsItsExpectation`** (must-have)
- Behaviour: every deferred file, run the same way, red by design until its item lands; the
  failure message names the file and its `deferred` item.
- Tier: `Deferred.`. It passes with zero deferred files. A file turning green here is moved out of
  `deferred` with its CURRENT_STATE entry, as for any `Deferred.` test.

**G5. `E2E.Scenarios.FileNamesAreTheirSlugs`** (nice-to-have)
- Behaviour: every file name is lowercase letters, digits and dashes plus `.json`, so it is a
  usable viewer slug and doctest subcase name.
- Assertion (mechanism): every name matches; a failure names the file.

### Visuals from the tests (amendment, 2026-10-17)

Every E2E test writes its own pictures, so the E2E page always matches the real suite. Today the
page comes from a scratchpad probe that rebuilds fixtures by hand and does not follow new tests.

- **One format writer.** `WriteViewerJson(const ViewerScene&)` in the library. A `ViewerScene`
  holds the structure, each body's box (centre, half extents, rotation), name and material, the
  removed bodies, the verdict before, the engine's after, an optional expected after, and the
  slug, title and test name. The scenario overload (V1) builds a `ViewerScene` and calls it.
- **One helper**, `tests/VisualSupport.h`: `RecordVisual(TestName, ViewerScene)`. With
  `LOADPATH_VISUALS=<dir>` set it writes `<dir>/<slug>.json`; unset or empty, it returns before
  building anything and does no file I/O. Its core, `RecordVisualTo(optional dir, file interface,
  ...)`, returns an error instead of failing, so the cases below can test it; the wrapper turns an
  error into an `ADD_FAIL_CHECK` naming the test. Files go through the CLI's small file interface,
  so the tests use an in-memory map.
- **Callers record before they assert.** A test calls `RecordVisual` after the run and before its
  first `CHECK`, never after a `REQUIRE`: this build defines
  `DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS`, so a failed `REQUIRE` ends the process and a
  picture recorded after it is lost. Review checks the order.
- **Which cases are E2E.** Every case named `E2E.*`, plus every case in `tests/EndToEndTest.cpp` or
  `tests/ScenarioFilesTest.cpp` whatever its prefix, so e2e 3.13
  (`Deferred.Slice6.DryWallLosingItsLintel...`) and `Deferred.Scenarios.*` count without a naming
  rule. `Visuals.ps1` lists them with doctest's `-ltc`, once with `-tc="E2E.*"` and once with
  `-sf`, and takes the union.

Prefixes: `Visuals.Record.*` and `Visuals.Scenarios.*` in `tests/VisualSupportTest.cpp`, two more
`Scenario.Viewer.*` cases in `tests/ScenarioTest.cpp`, and `Visuals.Script.*` in
`Scripts/Visuals.Tests.ps1`, a plain PowerShell test script (no Pester) that feeds the script's
functions fixture inputs and runs in about a second. The doctest cases are default tier. The
script cases are outside `Test.ps1`: they run whenever `Visuals.ps1` changes (test-expert to see
red, dev-expert at the end, review-expert once). All are red today: none of the pieces exist.

Most of these cases check files, names and order, not a structure, and their `Visual:` line says
so. Where a structure is the fixture, it is an existing entry.

**W1. `Visuals.Record.WritesNothingWhenTheVariableIsUnset`** (must-have)
- Behaviour: with no directory the helper does no file I/O, so a normal run pays nothing and stays
  deterministic.
- Fixture: the colonnade scene; the core given no directory and an in-memory file interface that
  counts every call; the environment reader given unset, empty and `C:\v`.
- Assertion (mechanism): zero calls of any kind on the file interface and no error; the reader
  returns no directory for unset and empty, and `C:\v` for the third.
- Red today: the helper does not exist.
- Visual: none: counts file calls (the scene is `colonnade-middle-post`).

**W2. `Visuals.Record.WritesOneEntryPerCallWithExpectedOnlyWhenItDiffers`** (must-have)
- Behaviour: with a directory set, each call writes one file `<dir>/<slug>.json` holding the
  structure before, the engine's after, and the expected after only when it differs; the bytes are
  the library writer's, so there is one format.
- Fixture: the in-memory interface, directory `v`; two calls from one test: the colonnade scene
  with an expected equal to the engine's after (slug `a`), and with E2's wrong expectation
  (slug `b`).
- Assertion (mechanism): exactly two files, `v/a.json` and `v/b.json`; each equals
  `WriteViewerJson` of its scene with the test name set, byte for byte; parsed back, each has
  `test`, `slug`, `before`, `after`; `expected` is absent from `a` and present in `b`.
- Red today: as W1.
- Visual: `colonnade-middle-post`, `colonnade-wrong-expectation`.

**W3. `Visuals.Record.ASlugThatIsNotFileNameSafeIsAnError`** (must-have)
- Behaviour: a slug is lowercase letters, digits and dashes (G5's rule), so it is a safe file name
  and a viewer slug; anything else is refused before a file is written.
- Fixture: slugs empty, `Colonnade`, `a b`, `a/b`, `a\b`, `..`, `a_b`, `a.json`; and
  `arch-keystone-intact-2`.
- Assertion (mechanism): each bad slug returns an error naming the slug and the test, with no write
  on the file interface; the good one writes.
- Red today: as W1.
- Visual: none: names only.

**W4. `Visuals.Record.ASlugUsedByTwoTestsIsAnErrorNamingBoth`** (must-have)
- Behaviour: two tests cannot share a picture. The same test writing its slug again rewrites it,
  because doctest reruns a case body once per subcase.
- Fixture: the in-memory interface; `E2E.A` writes `x`, then `E2E.B` writes `x`; separately
  `E2E.A` writes `x` twice.
- Assertion (mechanism): B's call returns an error containing `x`, `E2E.A` and `E2E.B`, and
  `v/x.json` still holds A's bytes; A's repeat returns no error and leaves one file. The check
  reads the `test` field of the existing file, so it holds across processes; `Visuals.ps1` empties
  the folder first (W13), so a stale file cannot collide.
- Red today: as W1.
- Visual: none: names only.

**W5. `Visuals.Record.AFailingTestStillWritesItsPicture`** (must-have)
- Behaviour: the helper never judges the outcome. A scene whose engine after disagrees with its
  expected after, which is what a red test records, is written with both, and the helper adds no
  failure, so a red case is visible on the page.
- Fixture: the colonnade scene with E2's wrong expectation, through the core.
- Assertion (mechanism): one file written, with `expected`, and no error returned (the wrapper adds
  a failure only for an error). End to end, W12 shows a red case keeps its picture.
- Red today: as W1.
- Visual: `colonnade-wrong-expectation`.

**W6. `Visuals.Scenarios.TheGenericTestWritesOneEntryPerFile`** (must-have)
- Behaviour: G1 to G4 record one picture per scenario file, slug the file name without `.json`,
  test name the generic test's own, through the same helper, so file cases and code cases make
  identical pictures.
- Fixture: G1's body as a function over a folder read through the file interface; an in-memory
  folder holding the example and the E2 file; directory `v`.
- Assertion (mechanism): `v/colonnade-middle-post.json` and `v/colonnade-wrong-expectation.json`
  exist, each equal byte for byte to `WriteViewerJson` of that file's scenario and outcome (the
  bytes C4's `--viz` writes), each with `test` `E2E.Scenarios.EveryScenarioFileMeetsItsExpectation`;
  the unmet file is written too, with `expected`, and the function reports it unmet.
- Red today: G1 does not exist.
- Visual: `colonnade-middle-post`, `colonnade-wrong-expectation`.

**W7. `Scenario.Viewer.TheScenarioExportIsTheSceneExport`** (must-have)
- Behaviour: there is one writer: the scenario overload gives the same bytes as the scene overload
  on the same structure and outcome.
- Fixture: the example, run; a `ViewerScene` filled by hand in the test from the loaded bodies'
  boxes, names and materials and the outcome.
- Assertion (mechanism): the two strings are equal.
- Red today: neither overload exists.
- Visual: `colonnade-middle-post`.

**W8. `Scenario.Viewer.ARotatedBoxExportsItsRotation`** (must-have: the arch case needs it)
- Behaviour: a rotated box exports its rotation `q` and its own half extents, so a voussoir is
  drawn where it is.
- Fixture: a `ViewerScene` of one box, half extents [0.2, 0.1, 0.05], rotated 30° about y.
- Assertion (mechanism): `q` is (cos 15°, 0, sin 15°, 0) in the viewer's component order, to 1e-12;
  `h` is the half extents.
- Red today: as W7.
- Visual: `arch-keystone-intact` (on the E2E plan page).

**W9. Code-based E2E cases record their pictures** (must-have; a rule on other cases, not a test)
- Behaviour: every E2E case in code calls `RecordVisual` with the slugs on its e2e-plan `Visual:`
  line: 3.2 (`arch-keystone-intact`, `arch-keystone-minus-crown-pair`,
  `arch-keystone-minus-one-crown-voussoir`), 3.12 (`file-colonnade-intact`,
  `file-colonnade-minus-post2`), 3.16 (`build-order-shuffled-screwed-colonnade`, one picture of the
  first shuffle, not one per shuffle), and 3.9 and 3.10 while the shed stays in code.
- Assertion: none of its own. The W10 guard fails naming any of them that writes nothing, and a
  mistyped test name trips the same guard.
- Red today: the cases are not written yet; each is written with its call.
- Visual: the slugs above, on the E2E plan page.

**W10. `Visuals.Script.TheGuardNamesEveryE2ECaseThatWroteNoEntry`** (must-have)
- Behaviour: `Visuals.ps1` fails, naming each executed E2E case that wrote no entry, so no E2E case
  can be added without a picture. G2, G3 and G4 are exempt by name: they legitimately run zero
  files, and W6 already makes them write one entry per file.
- Fixture: function inputs: executed cases `E2E.A`, `E2E.B`, `Deferred.Slice6.C` and G4; entries
  from `E2E.A` and `Deferred.Slice6.C`; then entries from all three; then an empty case list.
- Assertion (outcome): the first fails with a message naming `E2E.B` and no other case; the second
  passes; the empty list fails with `no E2E cases found`, so a wrong filter cannot pass vacuously.
  On failure the script still writes the page, so the gap is visible, and exits 1.
- Red today: the script does not exist.
- Visual: none: names only.

**W11. `Visuals.Script.TheIndexIsOrderedByTestNameAndMarksStatus`** (must-have)
- Behaviour: `data/index.json` lists every entry ordered by test name, then slug, each marked
  `passed`, `failed` or `deferred` from its case's result.
- Fixture: entries `E2E.B`/`b1`, `E2E.A`/`a2`, `E2E.A`/`a1`, `Deferred.Slice6.C`/`c1`,
  `Deferred.Slice6.D`/`d1`; results A passed, B failed, C failed, D passed.
- Assertion (outcome): order `c1`, `d1`, `a1`, `a2`, `b1` (ordinal, so `Deferred.` sorts before
  `E2E.`); statuses `deferred`, `passed`, `passed`, `passed`, `failed`. A `Deferred.` case is
  `deferred` while red; one that passes is `passed`, and the script prints
  `Deferred.Slice6.D passes: move it out of Deferred`.
- Red today: as W10.
- Visual: none: names only.

**W12. `Visuals.Script.ARedCaseKeepsItsPictureOnThePage`** (must-have)
- Behaviour: a failing or aborted case does not stop the run or lose any picture: each case runs in
  its own process, and its exit code is its status.
- Fixture: a fake runner (a function input): `E2E.A` writes an entry and exits 0, `E2E.B` writes
  one and exits 1, `E2E.C` writes one and aborts (a failed `REQUIRE`).
- Assertion (outcome): all three entries are in the index, A `passed`, B and C `failed`; the script
  exits 0 because the guard passes (the page is a picture; the test tiers are the gate) and prints
  `1 passed, 2 failed, 0 deferred`.
- Red today: as W10.
- Visual: none: names only.

**W13. `Visuals.Script.TheDataFolderIsEmptiedBeforeTheRun`** (nice-to-have)
- Behaviour: entries from an earlier run are deleted before the cases run, so a removed test's
  picture leaves the page and cannot trip W4.
- Fixture: a temporary data folder holding `old.json` from a case no longer listed.
- Assertion (outcome): after the run function, `old.json` is gone and not in the index.
- Red today: as W10.
- Visual: none: names only.

## 3. How it runs and how it is used

**Adding an end-to-end test:**

1. Write `data/scenarios/<slug>.json`, usually by copying the nearest file.
2. Run `Scripts/Scenario.ps1 -File data/scenarios/<slug>.json`. It builds `loadpath_cli` (Release)
   if needed, runs `scenario <file> --viz`, writes the page and its data to
   `build/scenarios/` (`index.html`, `data/<slug>.json`, `data/index.json`), and prints the result
   and the page path. `-Out <dir>` writes elsewhere. (`-All` is replaced by `Visuals.ps1`, below,
   which draws every file through G1 to G4; open question 8.)
3. Look at the picture: before, the engine's after, and the expected after when they differ.
4. The owner approves the file (in a plan or directly); it is then in the suite, because G1 picks
   up every file. No C++ is written.

A sketch without `expect` can live anywhere: the CLI runs it and draws it, and G1 never sees it.

**The viewer page moves into the repository** (`viewer/index.html`), so the script can copy it
next to the data; today it lives only in the scratchpad.

**Runtime budget.** The workbench's own cases: under 0.2 s in Release and 2 s in Debug together
(tiny fixtures, one 10 × 8 wall at most). The files: the e2e plan's budget, at most 3 s serial in
Release for G1 and G2 together and no file over 1 s; a file that outgrows it moves to
`tier: slow_in_debug`, then `slow`. Loading and finding joints is O(n²) over boxes, which is
milliseconds at a few hundred boxes; a file with thousands (the warehouse) is a `slow` file or
waits for a sweep-and-prune contact finder.

**What stays in code for now.** Anything that is not axis-aligned boxes: arches with rotated
voussoirs (`tests/ArchFixtures.h`, e2e 3.2), wedges and inclined planes, the Orduña pile; and
cases whose subject is not a scenario's outcome: e2e 3.12 (the structure file round trip) and
3.16 (build-order invariance, a property over shuffles). They stay as `E2E.*` cases in
`tests/EndToEndTest.cpp`, each recording its own pictures (W9). An `arch` build item (the fixture's
geometry moved into the geometry layer, with oriented boxes in the export) is the natural next
addition. The shed cases (e2e 3.9, 3.10) can be written as files once walls along y and `omit`
exist, but the shed is easier kept in code until open question 5 is answered.

**The E2E picture page: `Scripts/Visuals.ps1`.**

1. Builds Release (`Scripts/Build.ps1 -Config Release`).
2. Empties `build/visuals/data` (W13), lists the E2E cases (every `E2E.*` case plus every case in
   the two E2E files), and runs each in its own `loadpath_tests` process with
   `LOADPATH_VISUALS=build/visuals/data`; the exit code is the case's status (W12).
3. Applies the guard (W10): any E2E case with no entry is named and the script exits 1, after
   still writing the page so the gap shows.
4. Writes `data/index.json` (W11), copies `viewer/index.html` next to it, and prints the folder and
   the counts passed, failed and deferred. `-Out <dir>` writes elsewhere.

It costs the E2E budget plus one process start per case: a few seconds in Release. Every publish
run is also the script's end-to-end smoke.

**Publishing.** After every feature lands (review passed, committed), the orchestrator runs
`Visuals.ps1` and publishes its output folder to the E2E page: one fixed artifact URL, https://claude.ai/artifact/SLuMLdeb5UNZh382sBvYgd
(first published 2026-10-17, with `screenshots.html` as a second page beside it), never changed, so the owner's link always shows the current suite. A script
cannot publish to claude.ai, so this step is the orchestrator's. The scratchpad probe
(`e2e_viz_gen\viz_probe.cpp`) retires once every code-based E2E case records its own pictures.
In a plan, before its test exists, a code case's pictures still come from a probe; a file case's
come from `Scenario.ps1 -File`.

**CLAUDE.md row**, added to "Each tier runs once per change" when these tests are written:

| Who | Runs |
|---|---|
| orchestrator | `Scripts/Visuals.ps1` once after a feature lands, then publishes `build/visuals/` to the fixed E2E page URL; the run fails if an E2E case wrote no picture. `Scripts/Visuals.Tests.ps1` runs only when `Visuals.ps1` changes |

## 4. Open questions for the owner

1. **Box position by centre only, or also by lower corner?** Stacking is easier with a `min`
   corner (a post from z = 0 is `"min": [-0.1, -0.1, 0]`). Recommendation: allow exactly one of
   `centre` and `min`; it costs one error rule and saves arithmetic in most files.
2. **Where densities live.** Recommendation: a `materials` table in `data/catalogue.json` with
   sources, plus file-local materials for one-off blocks (B3, B1). The alternative, a density on
   every box, repeats the number everywhere.
3. **Rows of one case: one file each, or `variants` in one file?** The e2e cases often have two
   rows (n = 2 and n = 3 screws). Recommendation: one file per row for now; files are short, and
   each row gets its own picture and its own failure name. Revisit if copies start drifting.
4. **Exit codes.** Recommendation: 0 met, 1 not met, 2 the file cannot be run (usage, unreadable,
   invalid). A deferred file reports honestly (1 when not met); `Scenario.ps1 -All` labels it
   deferred rather than failed. The existing structure command keeps returning 1 for a load error.
5. **A `warehouse` build item** (the existing `BuildWarehouse`, so the shed and warehouse become
   files). Recommendation: not in this feature; add it when a shed case is next needed, since the
   generator already exists and the item is small.
6. **Per-step expectations** (an `expect` on a `cascade` or `judge` step, for "intact stands, then
   after the removal ..."). Recommendation: leave out; the final `released` list already fails if
   an earlier cascade releases something unexpected, and T4's `judge` covers a checkpoint verdict.
7. **Should `Visuals.ps1` later cover non-E2E structural tests** (the cascade, standing and
   authority fixtures)? Recommendation: not now. Any test may call `RecordVisual` and its pictures
   appear on the page, but the guard covers only E2E cases. Requiring pictures from the ~400 unit
   cases would bloat the page and cost a feature of its own; widen it once the E2E page has proved
   useful.
8. **Drop `Scenario.ps1 -All`?** Recommendation: yes. `Visuals.ps1` draws every scenario file
   through G1 to G4 along with the code cases, so two page builders would only drift. Keep `-File`
   for authoring one file without rebuilding the tests. Open question 4's "`-All` labels it
   deferred" then applies to `Visuals.ps1` (W11).

## 5. Owner answers (2026-10-17)

The owner approved the plan; every recommendation in section 4 is the answer (1 exactly one of
`centre` and `min`; 2 a `materials` table plus file-local materials; 3 one file per row; 4 exit
codes 0, 1, 2; 5 no `warehouse` item yet; 6 no per-step expectations; 7 the guard covers E2E
cases only; 8 `Scenario.ps1 -All` is dropped). Tests are written to the plan; a case red because
the engine disagrees is not made to pass.
