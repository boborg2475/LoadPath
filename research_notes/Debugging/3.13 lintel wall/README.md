# 3.13 lintel wall: debugging probe (2026-10-25)

Scratch probes from the session that explained why `Deferred.Slice6.DryWallLosingItsLintelKeepsTheBricksOverThePiers` over-releases. They are not part of the build or the suite, and they link against the Release `loadpath.lib`. The method and the lessons are written up in [claude_plans/DEBUG_PLAYBOOK.md](../../../claude_plans/DEBUG_PLAYBOOK.md); this folder holds the code and the data snapshot.

| File | What it is |
|---|---|
| `probe.cpp` | Shared fixture code: the `Scene` builder, `Lintel(S, bBonded, L0, L1)` (3.13 is `Lintel(S, false, 2, 6)`), `LintelExpected`, the friction `gMu` and the `gFixed` set of brick names to ground. It ends with `#include "main.inc"`. |
| `helpers.inc` | JSON helpers and `RunCascade`, copied from the picture generator. |
| `main.inc` | The main probe. It judges five states (intact, lintel removed, solver-only pass 2, after the cascade, triangle pre-removed). It writes `data.json` with readings, both authorities' verdicts, both motions, the solver's blockers and the cascade's failures. It also prints the diagnostics: whether the expected motion is admissible under associative flow, per-brick vertical balance, thrust across a cut, a friction sweep, and the fixed-abutment runs. |
| `rbsm_main.inc` | An elastic-only probe: the corner-status trajectory per iteration and an elastic-only cascade. Build it by swapping the last include of `probe.cpp` to `rbsm_main.inc`. |
| `RbsmDiag.h`, `patch.pl` | Instrumentation for the elastic solver. `perl patch.pl < src/RbsmStandingAuthority.cpp > RbsmDiag.cpp` makes a copy whose factory is `MakeRbsmDiag()`. The copy records why each component ended (`mechanism`, `restore`, `power`, `cap`), every corner that blocked a restore, the free motion and the final corner statuses. The library is untouched. |
| `build_rbsm.bat`, `build_probe.bat` | MSVC builds. The paths point at the session scratchpad, so edit `cd /d` before reuse. Compile `RbsmDiag.cpp` with `src/` and `third_party/eigen` on the include path and `/DEIGEN_MPL2_ONLY /fp:precise`. |
| `template.html`, `inject.pl` | The heatmap page. `inject.pl` puts `data.json` into the template and writes the published page (https://claude.ai/artifact/5ymS1pfAN1eQNaZGbgMqnA) plus a `preview.html` with a full document skeleton for local viewing. |
| `data.json` | The data the published page shows, from LoadPath `abf5463`. |
