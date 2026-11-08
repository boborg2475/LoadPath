# Owner rulings 2026-11-08: forced collapse, settings, pass recorder, core suite

Mirror of the owner's doc "LoadPath: Undecided resolves to forced collapse"
(https://claude.ai/artifact/1cFkspaL7mYWHP63NsY9HG, rev 41). **These are the decisions in force
for the moment** (owner, 2026-11-08); they supersede the OPEN recommendations in
[2026-11-08-elastic-only-rules.md](2026-11-08-elastic-only-rules.md) where the two differ. An
agent works the build order at the end, one item at a time, and the owner reviews each item
before the next starts.

## 1. Undecided resolves to forced Collapses

- An elastic Undecided component resolves to `Collapses` and **the whole component** is
  released, so nothing is left hanging. Rejected: releasing only the last pass's elastic
  `UnconstrainedBodies` (can leave pieces floating). Some over-release is expected; the flag
  makes it visible.
- No new verdict; host code is unchanged.
- Per-component report gains: `Forced` (bool: Collapses came from the Undecided fallback, false
  for an elastic Mechanism), `ForcedReason` (enum: pass cap, restore rule failing closed, power
  check, work-unit cap, at least), `PassCount` (passes before giving up), `UnsettledJoints`
  (interfaces whose corner states were still changing when the solver stopped, plus blocking
  corners where known). The cascade carries them on every pass that forced a collapse; the
  structure file and viewer JSON write them.
- Viewer: forced collapses tinted distinctly from proven ones; the inspect panel shows
  `ForcedReason`, `PassCount`, unsettled joints.
- Tests: new cases pin Undecided → Collapses with `Forced` true and a reason, Mechanism →
  Collapses with `Forced` false. The existing reds that expect Stands on Undecided components
  **stay red as written**; they are the solver work queue, and each fix removes a forced case.
  Add a tally of forced collapses by reason across the fuzz and scale runs.
- Context: 619 of 3,000 realistic random stacks and bridges read Undecided where collapse is
  provable.

## 2. Crushing: the crushed material turns to rubble (decides D1)

- Crushing is its own outcome, separate from cracking and from collapse. The solver reports the
  joint as crushed and names the body whose material crushed; that body is marked for
  destruction. The host (Chaos) turns it to rubble and handles what follows; once settled it
  hands the structure back and LoadPath judges again (more bricks may crush in turn).
- Several bricks over crushing strength at once: crush the worst plus any within a small margin
  of it (stress against each joint's own limit), hand off, judge again. **Crushing and cracking
  share one margin setting: the cascade's existing top band.** Today that is
  `CascadeOptions::CrackBandFraction = 0.5` (joints with ratio ≥ R_max × (1 − 0.5) crack in the
  same pass), which is far wider than "a small margin"; case D2 is the check that the band is not
  too wide, and the value becomes a settings-file entry (§5).
- Visualisations in a separate session confirm the rule.

## 3. Pass-by-pass replay (owner request)

- Record every pass so a solve and a cascade can be stepped through.
  - Cascade passes (outer): per pass the verdict per component, bodies released, joints cracked
    or removed, the reason including the forced flag.
  - Elastic passes (inner): per active-set iteration the four corner states per joint (closed,
    open, slipping), forces (N, shear, moment), utilisation, and which corners changed state
    since the last pass.
- Viewer: a timeline scrubber over cascade passes with step forward/back; inside a pass an
  expandable inner scrubber over elastic passes; corners that flipped this pass highlighted (so
  oscillation behind an Undecided is visible); released bodies and cracked joints shown with the
  pass that caused them.
- Recording is off by default and switched on by the test harness and the CLI, so runtime cost
  is untouched. **Record everything** (every cascade pass, every inner pass, every component);
  no selective recording for now. Written through the existing viewer JSON and `RecordVisual`
  path. File size on large structures is accepted; revisit only if it becomes a problem.
- **Debug settings hook**: the library takes an optional debug settings object from the host;
  absent (default) means no debug work and no cost. Option one is the pass recorder: a live
  game run writes the same pass data as the tests. Tests and the CLI switch it on through the
  same hook (one path). Built to grow: later options add logging or pull data without changing
  the solver's interface again.

## 4. Simplified core test suite

Shrink day-to-day testing to a small core set the owner can study in depth. Each case has a
known answer (literature or common sense), covers one behaviour, gets a picture and the
pass-by-pass replay. Real catalogue materials only; geometry is what is tuned to reach an edge.
Everything else is **parked, not deleted**.

Core set (to be expanded): block on a slope that slides; stack that tips; dry wall pushed
sideways; bonded wall; arch; crushed pier; beam on posts losing one post; lintel wall (3.13).

- **A, keystone over a gap.** One keystone brick holds a span; with it in place the structure
  stands near its limit (highest utilisation just under 1), without it the span bricks fall.
  Two variants: a wedged arch with a wedge keystone, and a flat course with the keystone in the
  middle. Mortared joints. The gap is widened until the real mortar alone cannot carry the span.
- **B, overhang on a column.** A straight mortared column (2 × 2 or 4 × 4) with a ledge
  cantilevered out near the top. The ledge is one long piece running from inside the column out
  past the face, no joint in the overhang near the face, so failure is at the column face.
  B1: nothing above the ledge; its moment breaks the mortar at the face and it tips off.
  B2: the column continues above the ledge; the weight clamps the root and it stands. Same ledge
  length in both; the ledge length puts B1 just past its limit.
- **C, incline.** C1 dry brick on a slope steeper than the friction angle slides; C2 on a
  shallower slope stays; C3 a mortared column normal to the slope stands; C4 the same column past
  its limit collapses, C4a tall slender (bending, tips at the base), C4b short wide on a steep
  slope (shear, slides off at the base). Base shear is W sin θ; base moment grows with height
  squared.
- **D, propped cantilever, prop knocked out (owner: very important).** A fixed wall acts as
  ground (cannot move, nothing attached to it breaks at the connection). A mortared 2 × 2 brick
  beam cantilevers out, propped at its far end by a dry stack. Knock out the prop: only the first
  section out from the wall cracks and the beam beyond falls as one mortared piece. D1 baseline:
  only the joint next to the wall is over capacity. D2 crack-rule test: several joints over
  capacity at once, still only the worst (next to the wall) breaks, then re-solve; the next joint
  out is clearly outside the band. D3 band test: the joints next to the wall are inside the band
  so more than one cracks in the same pass. D2 and D3 pin the band from both sides, and share the
  setting with crushing (H1, H2).
- **E, timber cantilever, nailed vs bolted to a fixed wall.** Restores fastener coverage lost
  with the certificate. The timber's end butts against the wall and the fasteners go straight
  in, so they are loaded in withdrawal; with the prop gone the top fasteners pull hardest and the
  bottom edge bears. E1 nailed: the top nails fail first, unzipping downward until the beam drops
  (the replay should show the order). E2 bolted: holds. Real EC5 nail and bolt rows; same timber
  and length. **Open problem:** a four-corner-spring joint lumps all fasteners and cannot show
  rows failing in turn; hand-splitting rows is ruled out (a player would not know to). Idea: the
  library expands a fastened joint into per-row springs from the fastener layout automatically.
- **F, beam on three posts, middle post removed.** The beam stands on the two outer posts. Tests
  that the solver finds a new load path rather than giving up (the similar shed case reads
  Undecided today).
- **G, dry wall pushed sideways** (even push over the face). G1 just under the limit stands;
  G2 just over, falls and tips, not slides; G3 a short thick wall with t/H > μ, just over the
  friction limit, slides, not tips; G4 the same force along a whole course at base, middle and
  top: only the top push tips, all under μW so none slides; G5 a push on a single brick just over
  the friction from the bricks above: that brick pops out, the rest stands, and nothing above the
  hole falls (running bond bridges it). More variants to come.
- **H, crushed pier.** A brick pier carrying a heavy stone just past the bricks' crushing
  strength. H1: only the bottom brick crushes (the brick above is clearly below the band). H2: a
  heavy stone on a short pier so the lowest bricks fall inside the band and several crush in one
  pass.

Known red E2E cases on `solver_rework` noted in the doc: the shed losing one post (Undecided,
wants Stands), the overloaded timber bearing (crushing), the arch losing its keystone (no hinge
split), the beam-on-piers scenario file (crushing). The lintel wall stays deferred.

## 5. Settings file: real-life defaults, user overrides

- Every tunable constant moves into a data file: material values, the crack/crush band, pass
  caps and the like. Defaults are real life, committed, never changed at run time. The program
  loads them at start-up; a user can change any setting (weaker joints, a wider band, a
  destruction-heavy "crazy mode") and can always reset to the defaults.
- Shape: a committed read-only defaults file plus a separate overrides file holding only the
  changed values; reset discards the overrides.
- Debug settings live in the same system: the debug options, starting with the pass recorder,
  are entries whose real-life default is off; the test harness or a user switches them on; reset
  turns them off.
- Format: **JSON** (the repo already reads JSON with its own reader; the common C++ YAML
  library uses exceptions, which the build forbids). YAML considered and dropped.

## 6. Research topic: fast vs slow loads on a mortared wall

A single brick in a mortared wall pushed until the mortar around it breaks: a fast concentrated
hit should break out or shatter only that brick before the load reaches its neighbours; a slow
push spreads load through the mortar so a wider patch goes. To research: how load rate changes
the size of the break-out; whether a static solver without time or inertia can represent it;
whether impacts belong to the host (Chaos, see COLLAPSE_DYNAMICS_NOTES) with LoadPath judging
only the slow case.

## 7. Open topic: hinge splitting (D2 in the earlier doc; not decided or scheduled)

A hinge opens on one edge and pivots on the edge still in compression; an arch losing its
keystone folds at a few hinges. A cracked or dry joint has no tension strength, so it opens and
pivots rather than shearing off, and slides only if the sideways force beats friction. For dry
masonry hinging is the normal failure. The certificate found the hinges and split the falling
bricks there; the elastic-only cascade releases one connected cluster, so Chaos gets one rigid
piece and cannot fold it. Splitting released clusters at hinges is what lets Chaos fold the
structure; it affects the arch tests and core case A.

## Build order (owner decision 2026-11-08)

1. **Settings file and the pass recorder** (§5, §3), so every later change can be watched pass
   by pass.
2. **Re-measure speed** against the 200 ms target with the certificate gone.
3. Then, in an order the owner sets at each review: forced collapse (§1), crushing as rubble
   (§2), the core suite (§4), the research topic (§6), hinge splitting (§7).

Each item is reviewed by the owner before the next starts.
