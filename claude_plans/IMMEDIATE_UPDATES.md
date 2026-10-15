# Immediate updates

The short list of what happens next, in order. CURRENT_STATE.md stays the full TODO list; this
file is the near-term slice of it, for the owner's goal (2026-10-15): **a fast elastic solver
that is trusted in more cases, so the collapse certificate is eventually not needed.** Remove an
item when it lands, as with CURRENT_STATE.

## 1. Workflow

### In progress (approved 2026-10-15)

- **Warm scratch copy.** `Scripts/Scratch.ps1` keeps `C:\Users\bobby\Documents\LoadPath-scratch`
  in sync (no `.git`, no `build`, timestamps kept) with its own warm build folder, so reviews,
  mutants and measurements build incrementally (about 1 min) instead of from nothing (5 to 10 min).
- **Each tier once per change.** Test and dev agents run their feature prefixes plus the Release
  default once; the Slow tier runs once, at review; Debug runs once, before the commit. A
  reviewer re-runs a tier only when something looks off. Goes into CLAUDE.md and the agent briefs.
- **Sharded default run.** `Scripts/Test.ps1 -Jobs N` splits the default run across processes
  with doctest's `--first`/`--last` over the same filtered order, balanced by cached per-test
  durations. Same tests, same assertions; no test moves tier. Target: about 60 s down to 10 to 15 s.
  Only `Slow.*` asserts on wall time, and it stays serial.
- **Mutation script.** `Scripts/Mutate.ps1` applies a single-occurrence find/replace in the
  scratch copy, builds, runs the named tests, reports caught or survived, restores the file and
  touches it so the next build recompiles it.

### Not yet approved (owner to decide)

- **Slim DESIGN.md.** It is over 2,100 lines, mostly dated §10 rulings, and every agent reads it
  cold. Move superseded §10 entries into `claude_plans/DESIGN_HISTORY.md`, keep the current
  rulings in DESIGN.md, and have briefs cite the exact entries an agent needs. Saves minutes and
  tokens per agent, and changes use three to six agents each.
- **One-pass review policy.** The first review lists every blocking item in one pass; later
  rounds only verify those fixes, and a new constructed edge case found in a later round is logged
  in CURRENT_STATE rather than blocking (unless it is unsafe for the host: a false release or a
  false Stands on a real export). The work-gate change took four rounds, each finding a new edge
  case. The oracle scope ruling (DESIGN §10, 2026-10-15) is the precedent.

## 2. Code: elastic speed

Warehouse, Release, 2026-10-15: cold elastic solve 419 ms (442 to 530 ms in the profile), judge
the same plus about 1.5 ms; warm re-judge after a removal about 140 to 150 ms. Every component
converges in one active-set pass at rest; the solve is single-threaded.

Where the cold time goes: factorization about 185 ms (40%), assembly about 175 ms (building the
triplet list about 100 ms, `setFromTriplets` about 74 ms), `analyzePattern` about 41 ms, the
eigenvalue check about 32 ms.

1. **Permanent profiling timers.** A CMake option `LOADPATH_PROFILE`, off by default, a
   `-Profile` switch on the scripts building into `build/msvc-profile`, and a phase-timing struct
   (assembly, analysis, factorization, solves, eigenvalue check, active-set passes) on the elastic
   result and the judge. Tests: the profiled build fills it, the normal build reads zero, and
   answers are bit-identical with profiling on or off. Kept in the code for debugging; never
   needed for a normal run.
2. **Lower-triangle assembly with a reserved triplet list.** Emit only the lower triangle the
   LDLᵀ reads, into a vector reserved to its final size. Measured estimate −130 ms; bit-identical.
3. **Cache the symbolic analysis.** Reuse the AMD ordering and elimination tree while the
   sparsity pattern is unchanged (active-set passes, warm re-judges). −20 to −40 ms; bit-identical.
4. **Cached CSC assembly.** Keep the compressed matrix's structure and scatter values straight
   into it instead of rebuilding from triplets. About −50 ms (estimate); bit-identical.
5. **Parallel component solves through a host hook.** The library takes a host-provided
   "parallel for" callback (Unreal's task system later, a small pool or serial in tests) and never
   creates threads itself. About −47% on the warehouse's five components. Answers must not depend
   on the worker count (a test runs 1 and N workers and compares bitwise).

Not immediate: a supernodal or blocked factor (−120 to −150 ms, but not bit-identical, so it
needs its own ruling).

## 3. Code: trust the elastic answer (fewer certificate fallbacks)

The owner's ask: resolve what the elastic solver leaves **Undecided**, either by making it reach
a decision (adaptive thresholds) or by deciding a default for what it leaves unsolved (the
"assumed standing" verdict). Measure first, then build in the order the counts justify.

1. **Fix the warm re-judge chatter on the warehouse.** After the post removal, the 198-DoF
   component flips one corner Stuck ↔ Slipping until the 50-pass cap (36 to 46 ms), reads
   Undecided, and the certificate then takes 96 to 111 ms to say Stands. It is the one known live
   chatter case and the biggest single warm cost: warm re-judge about 150 ms → about 40 ms. Find
   why that corner cycles (force size against its capacity, against `StickMarginRelative`) before
   choosing a fix.
2. **Fallback census.** Count every certificate consult by cause (Mechanism; Undecided by trigger;
   only-crushed Overstressed) with what the certificate answered, plus the flipping corners'
   force sizes relative to the force scale and to their own capacity. Run it over the seeded
   families, the 3,000 random stacks and bridges (the 619 Undecided predate slices 2d and 2e and
   this week's certificate work), the lateral wall grids, and the warehouse and shed cascades. A
   `Measure.*` row, run from a copy of the exe.
3. **Configurable noise floor.** `DryNoiseFloorRelative` (1e-12) becomes a `StandingOptions` field
   with today's value as the default: validated (positive, finite), part of the combined
   authority's `SameOptions`, and read by `ReadJoint` so the read-out matches. Document the safe
   range: 1e-9 already breaks the end-push scaling test; noise reaches about 1.5e-16 of the force
   scale on the 14 × 10 wall.
4. **Adaptive per-corner thresholds**, if the census finds chatter near the floor. Count each
   corner's flips within a solve and widen that corner's band until it commits, deterministically
   and with a capped width. Prefer the direction that keeps a corner open (widening re-close
   leans to Mechanism or Undecided, which the certificate checks), and cap any tension an adapted
   corner may carry (widening open leans to a false Stands, which nothing checks). The end-push
   scaling test guards the cap.
5. **"Assumed standing" default** for what still will not settle. When the 50-pass cap is reached
   **and** the same corners kept flipping over the last few passes, report a distinct verdict
   instead of consulting the certificate: the cascade treats it as standing, its readings are
   marked unconverged, and it is a host setting, off by default. Detect narrowly so it never
   catches slow drift, the restoration cycle, or trigger A2. Count first the chatter cases the
   certificate proves Collapses: those are the only ones where the default changes an answer.

Secondary, not immediate (the session's suggestions, not the owner's ask): a plastic crushing cap
in the active set, and a verified elastic mechanism that releases without the certificate.
