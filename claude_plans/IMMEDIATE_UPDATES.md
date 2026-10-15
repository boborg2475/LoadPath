# Immediate updates

The short list of what happens next, in order. CURRENT_STATE.md stays the full TODO list; this
file is the near-term slice of it. The owner's goal (2026-10-15) is a fast elastic solver that is
trusted in more cases, so the collapse certificate is eventually not needed, but **the workflow
comes first** (owner, 2026-10-15: more control over test cases before anything else). Order:
section 1, then 2, then 3, then the parked items in section 4. Remove an item when it lands.

## 1. Workflow

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
- **The 14 s default-tier test.** `Certificate.Sparse.AgreesWithTheDensePath` takes 14 s alone in
  Release, a quarter of the serial default run, and it is the floor of the sharded run (about 20 s
  at any shard count). Options: move it to `Slow.*`, or split it so its fixtures land in different
  shards. Left as it is until the owner decides; the sharding work changes no test.

## 2. Code: elastic speed (approved, after section 1)

Warehouse, Release, 2026-10-15: cold elastic solve 419 ms (442 to 530 ms in the profile), judge
the same plus about 1.5 ms; warm re-judge after a removal about 140 to 150 ms. The solve is
single-threaded at that measurement, and every component converges in one active-set pass at rest. The phases can be
read at any time from the profiling build (`-Profile`, `LOADPATH_PROFILE`; landed 2026-10-15).

1. **Parallel component solves, the rest** (owner's ideas file; the elastic part landed
   2026-10-15). The elastic `SolveComponents` runs through the host's `IParallelFor`: the
   warehouse's elastic solve went from 448 ms serial to 227 ms on 2 workers, 231 on 4 and 220 on
   8 (the largest component bounds it). Left: the certificate's `DecideAll` and the combined
   authority's per-component consults through the same hook, which first needs the nesting
   decision (a consult's `Body` would call the elastic solve, which calls `For`). Details in
   CURRENT_STATE.

## 3. Test control (owner, 2026-10-15), after section 2

The owner wants more control over what gets tested, and a set of end-to-end tests the project is
validated against before features are added. Both run alongside test-expert.

1. **The owner approves test cases before they are written.** test-expert first proposes a test
   plan: each case's name, the behaviour it pins, the fixture, the assertion and its kind
   (mechanism or outcome), and why it should be red today. The orchestrator relays it; no test code
   is written until the owner approves or edits it. Recorded in CLAUDE.md and the test-expert
   brief; the plan format is decided with the first use.
2. **End-to-end validation tests.** Whole-structure scenarios through the public API (build the
   structure, judge it, run the cascade, remove pieces, read who stands and what releases),
   checked against expected outcomes. A feature is validated against them before it lands. Start
   with an inventory of what already does this (the `Measure.Scale` rows, the anchors from papers
   and codes, the cascade fuzz families, the warehouse and shed), then test-expert proposes the
   missing scenarios for the owner to approve, under step 1.

## 4. Parked (owner, 2026-10-15): not before sections 1 to 3

### Elastic speed-ups

All measured on the warehouse cold solve, where factorization is about 185 ms (40%), assembly
about 175 ms (triplets about 100 ms, `setFromTriplets` about 74 ms), `analyzePattern` about 41 ms
and the eigenvalue check about 32 ms.

- **Lower-triangle assembly with a reserved triplet list**: emit only the lower triangle the LDLᵀ
  reads, into a vector reserved to its final size. Estimate −130 ms; bit-identical.
- **Cache the symbolic analysis**: reuse the AMD ordering and elimination tree while the sparsity
  pattern is unchanged. −20 to −40 ms; bit-identical.
- **Cached CSC assembly**: keep the compressed matrix's structure and scatter values straight into
  it. About −50 ms (estimate); bit-identical.
- **A supernodal or blocked factor**: −120 to −150 ms, but not bit-identical, so it needs its own
  ruling.

### Trust the elastic answer (fewer certificate fallbacks)

The owner's ask, when it resumes: resolve what the elastic solver leaves **Undecided**, either by
making it reach a decision (adaptive thresholds) or by deciding a default for what it leaves
unsolved (the "assumed standing" verdict). Measure first, then build in the order the counts
justify.

1. **The warehouse's warm re-judge chatter.** After the post removal, the 198-DoF component flips
   one corner Stuck ↔ Slipping until the 50-pass cap (36 to 46 ms), reads Undecided, and the
   certificate then takes 96 to 111 ms to say Stands: warm re-judge about 150 ms, about 40 ms
   without it. Find why that corner cycles (its force against its capacity and against
   `StickMarginRelative`) before choosing a fix.
2. **Fallback census.** Count every certificate consult by cause (Mechanism; Undecided by trigger;
   only-crushed Overstressed) with what the certificate answered, plus the flipping corners' force
   sizes relative to the force scale and to their own capacity. Over the seeded families, the 3,000
   random stacks and bridges (the 619 Undecided predate slices 2d and 2e), the lateral wall grids,
   and the warehouse and shed cascades. A `Measure.*` row, run from a copy of the exe.
3. **Configurable noise floor.** `DryNoiseFloorRelative` (1e-12) becomes a `StandingOptions` field
   with today's value as the default: validated, part of the combined authority's `SameOptions`,
   read by `ReadJoint`. Safe range: 1e-9 already breaks the end-push scaling test; noise reaches
   about 1.5e-16 of the force scale on the 14 × 10 wall.
4. **Adaptive per-corner thresholds**, if the census finds chatter near the floor: widen a flipping
   corner's band until it commits, deterministically, with a capped width. Prefer keeping a corner
   open (that leans to Mechanism or Undecided, which the certificate checks) and cap any tension an
   adapted corner may carry (that leans to a false Stands, which nothing checks).
5. **"Assumed standing" default** for what still will not settle: at the 50-pass cap **and** with
   the same corners flipping over the last few passes, a distinct verdict instead of the
   certificate. The cascade treats it as standing, readings are marked unconverged, and it is a host
   setting, off by default. Detect narrowly (never slow drift, the restoration cycle, or trigger
   A2), and first count the chatter cases the certificate proves Collapses: the only ones where the
   default changes an answer.

Secondary (the session's suggestions, not the owner's ask): a plastic crushing cap in the active
set, and a verified elastic mechanism that releases without the certificate.
