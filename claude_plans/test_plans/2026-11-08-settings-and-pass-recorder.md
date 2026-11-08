Status: built and accepted by the owner 2026-11-08 (`bb7b083`); the review findings carried are listed under item 1 in CURRENT_STATE.md.

# Build-order item 1: settings file and pass recorder

Owner decision 2026-11-08 ([decisions/2026-11-08-forced-collapse-and-core-suite.md](../decisions/2026-11-08-forced-collapse-and-core-suite.md) §3, §5). Owner reviews this item before item 2 starts. Three work packages; A and C are independent, B follows A.

## A. Settings file (library + CLI)

**Files.** `data/settings.json`: the committed real-life defaults, never edited at run time. An
overrides file (any path; the CLI default is `data/settings.local.json`, absent is fine) holds
only the keys a user changed. Reset discards the overrides file. JSON, read by `src/JsonReader.h`.

**Shape of `data/settings.json`** (every key below is required in the defaults file; the
overrides file may hold any subset, nested the same way):

```json
{
  "solver": {
    "max_active_set_iterations": 50,
    "pivot_tolerance_relative": 1e-10,
    "dense_dof_limit": 600,
    "stick_margin_relative": 1e-9,
    "held_force_tolerance_relative": 1e-12,
    "dry_noise_floor_relative": 1e-12,
    "neutral_load_tolerance_relative": 1e-12,
    "admissibility_tolerance_relative": 1e-9,
    "unconstrained_component": 1e-6
  },
  "cascade": {
    "crack_band_fraction": 0.5,
    "max_passes": 32
  },
  "debug": {
    "record_passes": false
  }
}
```

Material and joint values stay in `data/catalogue.json`, which already is a data file; the
settings file holds the engine's tunables. (`crack_band_fraction` 0.5 is today's value, carried
over unchanged; narrowing it is build-order item 3, cases D2/D3/H1/H2.)

**API** (`include/loadpath/Settings.h`):

```cpp
struct SolverSettings { /* the nine solver keys, same names in PascalCase, same defaults */ };
struct CascadeSettings { double CrackBandFraction = 0.5; int MaxPasses = 32; };
struct DebugSettings { bool bRecordPasses = false; };
struct Settings { SolverSettings Solver; CascadeSettings Cascade; DebugSettings Debug; };
struct SettingsResult { Settings Value; FileError Error; bool Ok() const; };
// Every key present and of the right type, no unknown keys, or Error names the key and line.
SettingsResult LoadSettings(std::string_view DefaultsJson);
// Overrides may hold any subset; an unknown key or a wrong type is an error naming it. A non-finite number is an error.
SettingsResult ApplyOverrides(const Settings& Defaults, std::string_view OverridesJson);
StandingOptions ToStandingOptions(const Settings&, Vec3 Gravity);
CascadeOptions ToCascadeOptions(const Settings&, Vec3 Gravity);
```

The six solver constants that are `constexpr` in `src/RbsmStandingAuthority.cpp` today
(`StickMarginRelative`, `HeldForceToleranceRelative`, `DryNoiseFloorRelative`,
`NeutralLoadToleranceRelative`, `AdmissibilityToleranceRelative`, `UnconstrainedComponent`) become
fields of `StandingOptions` with the same defaults, read from the options at the point of use.
`StandingOptions` also gains `const DebugSettings* Debug = nullptr` (the host's debug hook;
null means no debug work). `CascadeOptions` reaches the same pointer through `Authority.Standing`.

**CLI.** `scenario` gains `--settings <defaults.json>` (default `data/settings.json`) and
`--overrides <file>` (default `data/settings.local.json`, silently absent). Exit 2 with the key
named on a bad settings file. The resolved settings drive both the authority and the cascade.

**Tests (`Settings.*`).**
- `Settings.DefaultsFileLoadsAndMatchesTheCompiledDefaults`: the shipped `data/settings.json`
  loads, and every field equals the C++ default (so the two cannot drift).
- `Settings.AMissingKeyIsAnErrorNamingIt`; `Settings.AnUnknownKeyIsAnErrorNamingIt`;
  `Settings.AWrongTypeIsAnErrorNamingIt`; `Settings.ANonFiniteNumberIsAnError` (`1e999`).
- `Settings.OverridesChangeOnlyTheKeysTheyName` (override `crack_band_fraction` to 0.1 and
  `debug.record_passes` true; everything else equals the defaults).
- `Settings.ResetIsTheDefaultsWithoutTheOverrides` (apply then reload defaults: equal).
- `Settings.ToOptionsCarriesEveryField`.
- `Statics.Settings.TheSolverReadsItsTolerancesFromTheOptions`: a solve with
  `DryNoiseFloorRelative` set to 0.5 (absurd) changes the reading of a dry two-brick stack at
  rest, proving the constant is no longer compiled in; the default-options solve is bit-identical
  to before (the existing `Statics.Authority.*` determinism tests guard that).
- `Cli.ScenarioReadsSettingsAndOverrides`: an in-memory store with an overrides file setting
  `max_passes` to 1 changes a cascade's `Passes`; a bad key exits 2 and names it.

## B. Pass recorder (library + viewer JSON + harness + CLI)

**Hook.** When `Options.Debug != nullptr && Options.Debug->bRecordPasses`, the elastic solver
fills `StandingComponent::Passes` and the cascade fills `CascadeVerdict::Passes`. Otherwise both
stay empty and no recording code runs (the check is one branch per iteration). Record
everything: every component, every cascade pass, every inner iteration.

**Data** (`include/loadpath/StandingAuthority.h`, `include/loadpath/Cascade.h`):

```cpp
enum class CornerState : std::uint8_t { Closed, Open, Slipping };
enum class SolveEnd : std::uint8_t { None, Converged, Mechanism, RestoreFailedClosed, PowerCheckFailed, IterationCap, InvalidInput };
struct JointPassRecord
{
	InterfaceId Id;
	std::array<CornerState, 4> Corners;      // after this iteration's update
	std::array<bool, 4> bFlipped;            // state differs from the previous iteration (all false on iteration 1)
	std::array<double, 4> CornerNormalN;     // compression positive
	std::array<Vec3, 4> CornerShearN;
	double NormalForceN; Vec3 ShearForceN; Vec3 MomentNm;
	double Utilisation; GoverningAxis Axis;
};
struct ElasticPassRecord { int Iteration; std::vector<JointPassRecord> Joints; };
// On StandingComponent:
std::vector<ElasticPassRecord> Passes;   // empty unless recording
SolveEnd End = SolveEnd::None;           // always filled: why the solve stopped
std::vector<InterfaceId> Blocking;       // RestoreFailedClosed only: the interfaces whose corners blocked the restore
```

```cpp
struct CascadePassRecord
{
	std::uint32_t Pass;
	std::vector<ComponentReport> Components;     // the judge of this pass, with each component's SolveEnd and inner Passes
	std::vector<InterfaceReading> Readings;
	std::vector<ReleasedCluster> Clusters;       // released in this pass
	std::vector<FailedInterface> Failures;       // cracked or severed in this pass
};
// On CascadeVerdict:
std::vector<CascadePassRecord> Passes;   // empty unless recording; includes the final judge that changed nothing
```

`ComponentReport` (StructuralAuthority.h) gains `SolveEnd End` and `std::vector<ElasticPassRecord> Passes`
carried from the elastic component. `SolveEnd` is filled whether or not recording is on: it is
cheap and the debugging playbook needs it (the forced-collapse item will reuse it as the
`ForcedReason`).

**Viewer JSON** (`WriteViewerJson`): each `after` entry gains `"pass_records": [...]` (`passes` stays the integer pass count the page already reads) when non-empty:

```json
"pass_records": [
  { "pass": 1,
    "components": [ { "bodies": [...], "verdict": "Collapses", "end": "Mechanism", "iterations": 4,
                      "inner": [ { "iteration": 1,
                                   "joints": [ { "interface": 17, "corners": ["closed","open","closed","closed"],
                                                 "flipped": [false,true,false,false],
                                                 "corner_n": [..4..], "corner_shear": [[x,y,z]..4..],
                                                 "n": 12.3, "shear": [x,y,z], "moment": [x,y,z],
                                                 "utilisation": 0.42, "axis": "Shear" } ] } ] } ],
    "readings": [ ...same shape as the joints entries, one per live interface... ],
    "clusters": [ ...same shape as after.clusters... ],
    "failures": [ ...same shape as after.failures... ] }
]
```

Interface ids are the viewer's existing `interface` numbers; the scene already lists the
joints' geometry, so the viewer can place a corner at its rectangle corner (order (+u,+v),
(−u,+v), (−u,−v), (+u,−v)).

**Harness and CLI.** `RecordVisual`/`CaseRun` in `tests/EndToEndSupport.h` and the scenario
runner in `src/Scenario.cpp` run with `bRecordPasses` true whenever a picture is being recorded
(`LOADPATH_VISUALS` set) and otherwise false, so the plain suite's cost is unchanged. The CLI
honours `debug.record_passes` from the settings/overrides and also takes `--record-passes` as a
shorthand override.

**Tests.**
- `Statics.Recorder.OffByDefaultRecordsNothing`: null `Debug` and `bRecordPasses` false both give
  empty `Passes`; `End` is still filled.
- `Statics.Recorder.EveryIterationIsRecordedWithItsFlips`: a dry two-brick stack under a lateral
  load that takes ≥ 3 iterations records exactly `ActiveSetIterations` entries, iteration 1 has
  no flips, and each later entry's `bFlipped` equals the state difference against the previous
  entry (checked by the test from the recorded states).
- `Statics.Recorder.TheEndReasonNamesWhyTheSolveStopped`: Stands → Converged; an ungrounded body
  → Mechanism; the 3.13-style loose end bat → RestoreFailedClosed with non-empty `Blocking`
  (or, if no small fixture reaches it, the iteration cap at `MaxActiveSetIterations` 1 → IterationCap).
- `Statics.Recorder.RecordingChangesNoAnswer`: the same solve with and without recording gives
  bit-identical readings and state (the recorder is observation only).
- `Cascade.Recorder.EveryPassIsRecordedWithItsReleasesAndCracks`: a bonded wall pushed past its
  limit records one entry per counted pass plus the final judge; the union of recorded clusters
  and failures equals `Verdict.Clusters` and `Verdict.Failures`.
- `Scenario.Viewer.PassesAreWrittenWhenRecordedAndOmittedWhenNot`.
- `Cli.RecordPassesWritesPassesIntoTheVizFile`.

## C. Viewer: pass-by-pass replay (`viewer/index.html`)

Reads `after.pass_records` when present. Adds, below the existing before/after panes:

- An outer timeline scrubber over cascade passes (slider plus step back/forward buttons and the
  pass number). Selecting pass k draws the after pane at the state after pass k: bodies released
  in passes ≤ k are drawn as released (in their recorded cluster colour), joints cracked in
  passes ≤ k are drawn cracked (dashed), severed ones removed; the components' verdicts of pass k
  are listed in the facts table with their `end` reason and iteration count.
- Inside a pass, an expandable inner scrubber over the elastic iterations of one component
  (choose the component from the pass's list; default the first non-Stands one). Each joint's
  four corners are drawn as small discs at the rectangle corners: closed (filled, body colour),
  open (hollow), slipping (filled, accent colour). Corners that flipped this iteration get a
  bright ring. Hovering or clicking a joint shows its N, shear, moment, utilisation and axis.
- A legend for the corner glyphs and the released/cracked colours, and the pass that caused each
  release or crack shown in the inspect panel.
- With no `passes` the page behaves exactly as today (the Release default run's pictures are
  recorded with passes on, so most will have them; the expected-by-the-test overlay has none).

Verification: `Scripts/Visuals.ps1 -NoScreenshots` renders every case; open two in the browser
(the bonded wall pushed past its limit and the colonnade losing a post) and step both scrubbers.
Screenshot checks in `Scripts/Visuals.Tests.ps1` stay as they are.

## Out of scope for item 1

Forced collapse (`Forced`, whole-component release), crushing as rubble, narrowing the band,
the core suite, the speed re-measure (item 2). `SolveEnd` is recorded here because the recorder
needs it; the forced-collapse item maps it to `ForcedReason`.
