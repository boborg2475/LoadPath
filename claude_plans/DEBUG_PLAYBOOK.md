# Debugging playbook: a test whose result looks wrong

Notes toward a debugging skill or program for test cases and validations (owner, 2026-10-25). The first worked case is 3.13, the dry wall losing its lintel. Its probe code is in `research_notes/Debugging/3.13 lintel wall/`. Add each new case's lessons here; when a step has been done by hand three times, it is a candidate for the program.

## 1. The questions, in order

These came up in this order on 3.13. Each is answered by a specific experiment in §2.

1. **What exactly differs from the expectation?** List the released set against the expected set, with each failure's axis and ratio. A ratio of 0 or none means the joint was cut because a mechanism moved it, not because it was overloaded. (3.13: 34 released against 10; many severed joints read 0.)
2. **Which component made the call?** Trace the path: cascade → combined authority → elastic solver or certificate. On a certificate `Collapses`, `Release` is the certificate's moving-body list (CombinedAuthority `Consult`). The elastic lists are never released.
3. **Why did that component choose this set?** For the certificate: λ, the selection rule (most gravity work per unit L1 body motion, listing threshold 1e-3) and the flow rule (associative). For the elastic solver: why it ended (§2 g).
4. **Is the expected outcome even admissible in that component's model?** Turn the expected outcome into a motion and test it against the component's rules (§2 b). If it is inadmissible, no tie-breaking rule can ever pick it.
5. **What would the other component say on its own?** (§2 f, g.) On 3.13, the certificate over-releases (34) and the elastic solver under-releases (2, then Undecided).
6. **Is the expectation right?** Argue it from bounds that hold for the real material (§3), not from either component's answer.
7. **Does the remainder hold once the expected set is gone?** (§2 d.) This also tests whether the expected set is complete.

## 2. Experiment catalogue

| | Experiment | What it shows | 3.13 result |
|---|---|---|---|
| a | Judge each state separately: intact, after the removal (before the cascade), after the cascade, expected remainder | where the verdicts change, and the readings in each | intact Stands only through the certificate; removed Collapses at λ ≈ 4e-11 |
| b | Expected outcome as a motion, checked against the certificate's flow rule (every sliding joint opens by at least μ × slip) | whether the right answer is reachable | the triangle falling violates the rule at all 8 side joints |
| c | Friction sweep | whether dilatancy drives the result (its angle is atan μ) | μ 0.05–0.7 releases 31–34; μ ≥ 1 reads Stands at λ 160 |
| d | Pre-remove the expected set, then judge | whether the remainder stands, and whether the expected set is complete | the shoulders still collapse, so "exactly 10" is too strong |
| e | Ground chosen bodies (`gFixed`): ideal abutments, fixed piers | which part is the weak link | triangle with fixed surroundings: Stands; piers fixed, shoulders free: Collapses at μ 0.7 |
| f | Elastic-only cascade: release the elastic `UnconstrainedBodies` while it reads Mechanism | what happens without the certificate | 2 bricks, then Undecided |
| g | Instrumented copy of the elastic solver (`patch.pl`) | why it ended: `mechanism`, `restore` (with the blocking corners), `power` or `cap`; plus the corner-status trajectory | every Undecided here is `restore`, caused by spurious loose end bricks |
| h | Per-brick vertical balance from the readings: beds below, side joints (friction against μ × side compression), beds above | what holds a brick up; an upward force from a dry bed above is tension, so the field is not valid | c7x4.00 hangs 24.6 N of its 28 N from the bricks above |
| i | Horizontal thrust across a vertical cut | whether the field is arching | about 165 N across the middle; 282 N in the top-course apex joint |
| j | Heatmap page from the exported states | readings on the standing structure, click to inspect, both motion overlays, blockers | https://claude.ai/artifact/5ymS1pfAN1eQNaZGbgMqnA |

## 3. Physics bounds for judging an expectation

- **Associative collapse means real collapse.** With non-associative friction, the collapse load is no higher than the associative one (Radenkovic's first theorem). If the certificate (associative) reads Collapses with λ < 1, the real structure collapses too.
- **Associative Stands proves nothing on its own.** Dilatancy can lock a wedge that would slide in reality. On 3.13 at μ ≥ 1 it reads Stands at λ 160.
- **A zero-shear stacked field is safe for any μ.** Blocks resting wholly on standing blocks under gravity, with no friction demand, stand whatever the flow rule.
- **Fixing bodies only removes mechanisms.** A structure's collapse load is at most that of the same structure with some bodies grounded. If it collapses with the piers fixed, it collapses with them free.
- **Elastic readings on Mechanism or Undecided are not an equilibrium.** They come from the last active set, and dry joints can show tension.

## 4. Pitfalls met (tooling)

- In Git Bash a heredoc containing quotes can fail; write the file with the Write tool instead.
- Perl replacement strings: `\\n` becomes a real newline inside C string literals, `${...}` interpolates (it breaks JS template strings), and `\25` is an octal escape. Use the Edit tool, single-quoted files, or `chr()`.
- Python is not installed; use Perl `JSON::PP` for JSON checks.
- The browser pane cannot inspect `file://` pages. Serve the folder with a small Perl HTTP server on 127.0.0.1. `TaskStop` can leave the Perl process listening, so free the port with `taskkill`.
- The probes link the Release `loadpath.lib`. Rebuild it after any library change, or the probe tests old code.
- Listing thresholds differ: certificate 1e-3 of the peak motion, elastic solver 1e-6. The certificate's load-factor cap is 1000, so λ 160 is a real limit.

## 4b. Second worked case: A1.4, an Undecided from the pass recorder (2026-11-10)

Full note: `decisions/2026-11-10-a14-undecided-findings-and-plan.md`. What worked, in order:

1. Run the file with `loadpath_cli scenario <file> --viz <out> --record-passes` and print, per pass, each component's end reason, iteration count and unsettled joints. This names the pass and the joint.
2. For that pass, print per iteration the corner states and corner normal forces of the joints around the unsettled one. The trajectory showed 14 joints opening in one step, the restore closing them unevenly, and the cycle guard firing.
3. Rule out noise with `--overrides`: tolerances, the iteration cap, the dense/sparse limit. Unchanged, so the cause is structural.
4. Sweep the setting that controls the input state (`crack_band_fraction`) to find what the failure depends on. Here it was the count of adjacent cracked joints.
5. Run every scenario file at the two ends of the sweep before calling a setting a fix. It turned A2.5 red.

Copy `loadpath_cli.exe` before a sweep (it locks the build output). Perl `JSON::PP` reads the viz files.

## 4c. Third worked case: three dry walls, a Mechanism proven on noise (2026-11-14)

Full note: `decisions/2026-11-13-soft-singular-step-spike.md` §9.5 (branch `spike/soft-singular-step`).

- **Symptom:** raising the singular step's neutral threshold from 1e-12 to 1e-9 turned three pinned wall Mechanisms into `IterationCap`.
- **Look first at** how strongly the load drives the motion on the iteration that declares the Mechanism (projection over load), and at the declared motion against the load direction. On that branch `LOADPATH_TRACE_LOG` writes one line per iteration.
- **What it was:** the Mechanisms were declared on motions driven at about 1e-12 of the load, with bricks tipping and lifting against the push. The power check cannot reject them on dry joints. The tighter threshold exposed a false proof; it did not lose a real one.
- **Rules it leaves:** a changed verdict under a tighter tolerance is not a regression until the old verdict's proof has been read. Check a pin's history before defending it: these were Undecided one regeneration earlier. Ask the oracle whether the outcome is right separately from whether the proof is.

## 5. Toward a program and a skill

A `loadpath_cli debug <scenario>` command (or a Scripts wrapper) would replace the hand-written probe:

- Export per state: readings, both authorities' verdicts, both motions, the elastic end reason and blockers, and the cascade failures with ratios. This is the `data.json` schema in the 3.13 folder, and the owner's 2026-10-25 directive to record stress in every test picture would produce most of it.
- Experiment flags: `--sweep-mu`, `--fix <names>`, `--pre-remove <names|expected>`, `--elastic-only`, `--check-motion <names>` (associative admissibility of a motion), `--balance <names>`, `--thrust-cut x,zmin`.
- Write the heatmap page from the same export.
- Print a summary: who decided, the expected motion's admissibility, why the elastic solver ended, and which bounds of §3 apply.

The elastic end reason is on `StandingComponent::End` (and `ComponentReport::End`) since build-order item 1 package B (2026-11-08); the blocking-corner list went with `RestoreFailedClosed` on 2026-11-12; with `debug.record_passes` on, every inner iteration is in `StandingComponent::Passes` and every cascade pass in `CascadeVerdict::PassRecords`, written to the viewer JSON as `after.pass_records`.

The skill would be the checklist in §1, with the experiment table of §2 as its toolbox and §3 as the rules for judging an expectation.
