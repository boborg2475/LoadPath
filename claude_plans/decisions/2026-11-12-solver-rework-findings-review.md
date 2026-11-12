Status: owner decisions from planning conversations on 2026-11-10 and 2026-11-12. Copied from the planning doc at https://claude.ai/artifact/2WxehQFac1eqLxHY4EbvhZ on 2026-11-12. Some items are built since (hysteresis); see CURRENT_STATE for what is done.

# LoadPath: solver_rework findings review

Branch `solver_rework` at 7dd0464 (2026-11-09). Owner decisions from the review of the build findings,
2026-11-10, with the speed, validation and soft-spring sections added 2026-11-12.

## Crack and crush band: two rules

The band decides which joints break in the same cascade pass as the worst one. It is a filter on top of the
over-capacity test: a joint under utilisation 1.0 never cracks or crushes. Today it is one relative setting,
`crack_band_fraction` 0.5, which pulls in joints at half the worst joint's stress. Too wide; case D2 would
break the cantilever into pieces.

Decided (owner, 2026-11-10): two rules, a joint cracks or crushes in this pass if either holds.

1. **Relative band, 0.2.** Joints over capacity and within 0.2 of the worst joint's utilisation break with
   it. Replaces 0.5. Purpose: speed, so a cluster of near-equal overloads goes in one pass instead of one per
   pass.
2. **Catastrophic level, 2.0.** Every joint at or above twice its capacity breaks in this pass, regardless of
   the band. Something severe has hit the structure; what is doubly overloaded must go. Starts deliberately
   high; tune down later if needed.

Both are settings in `data/settings.json`, shared by cracking and crushing (ruling 2026-11-08). Cases D2 and
D3 pin the relative band; a catastrophic case (owner to define) pins the 2.0 level.

To check in the code: that the band compares utilisations (demand over capacity), so the relative rule is
'within 0.2 of the worst utilisation', and that the recorder logs how many passes each cascade takes, so the
band can be tuned from data.

## Flat keystone span stands to 53 bricks

Finding (case A2): the flat soldier course stands from 19 to 53 bricks and falls only at 55 (3.575 m). The
solver treats the bonded piers as rigid abutments with full thrust and no spread, so the cracked course wedges
itself in as a flat arch. Real piers lean a little, the thrust drops, and a flat arch fails at a much shorter
span. The realistic sequence is tension cracks at the pier faces first, then the course hinges or drops in
shear; the solver finds the first step but never the second.

Decided (owner, 2026-11-10):

- Accepted for now. The A2 files stay as they are, including the 55-brick file as a pin on the engine's edge.
- Flag the unrealistic rows. Any row that stands at a span real masonry would not (the 53-brick course, and
  similar) is marked in the test and in the pictures as 'needs fixing: model edge', so it is never mistaken
  for a correct answer.
- Experiment, later: a softer ground joint under each pier (a footing stiffness row in the catalogue, no
  solver change) so the thrust falls off and the course drops at a shorter span. Not before cases A to H are
  all built, because the owner wants to see how it changes every case, not just A.
- Ruled out: dry joints at the pier faces (changes the structure into a loose beam on piers).
- Pie in the sky, far down the road: a geometry-updating solver that models real pier lean. More realistic, a
  different kind of solver, and a lot of work. Kept as a long-term goal, not a current plan.

To define: how the flag appears, for example a `model_edge` key in the scenario file that the viewer and the
pictures page show.

## Feature: segmented members (beam flex and snapping)

Owner decision 2026-11-10: a new feature. One physical beam (timber first; a stone lintel works the same way)
is split into logical segments. The joints between segments are the material itself, not fasteners or mortar:
their stiffness comes from the wood's elastic modulus and the cross-section, their strength from the wood's
bending and shear strengths. To the player it is still one beam.

What it gives:

- Flex: the chain of segments sags under load like a real beam.
- Snapping: when the bending at a segment joint exceeds the wood's strength, the beam breaks there, through
  the existing crack rule.
- Hand-off: the broken beam goes to Chaos as pieces, which it already handles.

Work, about a day or two:

1. A catalogue row for solid timber as a continuous material: elastic modulus, bending and shear strength,
   from EC5 / the C24 grade already in use. Data only.
2. A segmenting step in the geometry layer: given one member and a segment length, build the segments and the
   joints between them.
3. A segment-length setting in `data/settings.json`.
4. Tests: a simply supported beam under a mid-span load sags by the textbook amount; a heavier load snaps it
   at mid-span.

Risk: four corner springs on a cut face approximate beam bending; the joint stiffness may need calibrating
against the textbook sag.

Separate from the pie-in-the-sky pier lean: beam flex is small-displacement and the current solver handles it;
pier lean needs the geometry updated as it moves.

## Test to add: contact count must not change the answer

Owner request 2026-11-10. Splitting a joint face into more contacts (two pier bricks under one ring brick, or
a segmented member) must not make the joint stronger or weaker. The total force is the same, and the stress at
the inner edge, the middle and the outer edge of the face must read the same whether the face is one contact
or several. Guard against the failure where force is divided by the number of contacts and each one reads
lower, so finer segmenting makes a structure artificially stronger.

- Same structure built two ways: one contact across the face, and the face split into two (and four)
  contacts. Compare edge stresses and utilisation at each edge; they must agree within a small tolerance.
- Also the pass at which the face cracks: with one contact the whole face cracks at once; with several, the
  most-stressed contact cracks first. Both are acceptable, but the load at which the first crack appears must
  be the same.
- Came up on case A1.4, where the ring's end brick straddles two pier bricks and the inner contact cracks a
  pass before the outer.

## Question to answer: cost per elastic iteration

Owner question 2026-11-10. Measure how long each elastic iteration takes, and find out whether an iteration
re-solves the whole component or only the region around the corners that changed. Came up on the arch fold
(A1.4), where pass 3 took seven iterations on a 231-body, 1,386-unknown component. From the code as read: each
iteration re-assembles and re-factorises the full stiffness matrix of the component; there is no local update.
The recorder does not time iterations yet; add per-iteration timing to the pass records.

## Arch fold (A1.4): why the solver reads Undecided, and the fix

Traced 2026-11-10 with debug prints on a scratch copy of `RbsmStandingAuthority.cpp`.

What happens in pass 3 (18 cracked joints: both contacts at each foot, 14 crown joints):

1. Iteration 1, a normal solve: opens all 14 crown joints on one edge and the inner foot contacts. The crown
   is now a chain of loose bricks: 11 free motions.
2. Iterations 2 to 5 are the restore rule running four times, not ordinary settling. Each time it finds crown
   joints being pushed shut by the free motion and closes them, working from the outside in (6, then 2, 2, 2
   joints). Allowed because iteration 1 had them closed. By iteration 5 only the keystone joints are open: a
   three-hinged arch.
3. Iteration 6, a normal solve: the load has shifted onto the hinges. On the left, v.60/v.61 goes back into
   tension and reopens, and the inner foot contact goes fully open; the mirror joints on the right stay
   closed. A rounding-level asymmetry.
4. Iteration 7: singular again, 5 free motions. The restore rule wants to close v.60/v.61 (allowed, closed in
   iteration 6) and the keystone joint v.66/v.67 (blocked: open in every normal solve so far). One blocked
   corner makes the result Undecided, `RestoreFailedClosed`, blocking joint v.66/v.67.

Correction (2026-11-12, from the same debug trace and confirmed by the spike note): the left/right mismatch
starts in iteration 5, not 6. Iteration 5's free motion is numerical noise (7.6e-12 of the load, just over
`neutral_load_tolerance_relative`), and the restore rule acts on it lopsidedly: it closes v.65/v.66 and
v.72/v.73 and leaves the keystone joint v.66/v.67 open. Iteration 6 inherits that. Neither hysteresis nor the
closing-rate check addresses it; see the free-motion floor under the soft-spring section.

The rule: an open corner that the free motion pushes shut may be closed only if the last full-rank solve had
it closed. It was built to stop a false stand (a sliding brick's stray nudge re-closing a corner and arresting
the slide). On the arch it is wrong: the fold motion closing the keystone at nearly full speed is itself the
evidence that the edge bears. The rule uses history as a proxy for 'can carry compression', and history is a
poor proxy once the structure moves.

Checked: the 0.2 crack band does not fix this case. The crown joints read 1.18 to 1.41, all 14 still crack in
one pass, and it ends in the same `RestoreFailedClosed` one pass later. The band change and this fix are
independent.

Decided (owner, 2026-11-10):

- **Replace the history check with a closing-rate check.** The restore rule already measures how fast each
  open corner's gap shrinks under the free motion, relative to the largest motion in the component. Close the
  corner when that rate is above a threshold; leave it open when it is below (noise, the sliding-brick nudge).
  The threshold is a new setting in `data/settings.json`.
- **Hysteresis on corner state changes.** A closed corner opens only when its tension is clearly past the
  line, and an open corner closes only when its compression is clearly past the line, each by a small
  tolerance, so a rounding hair does not flip one side of a symmetric structure and not the other. The
  tolerance is a new setting. (The solver already has a 1e-9 stick margin for slip; this adds the same idea to
  open/close.)
- Tests: A1.4 goes green as a proven Mechanism with the piers standing; the existing sliding-brick and incline
  anchors guard the other side; a symmetric structure must produce mirrored corner states.

Still true after the fix: the forced release of a whole component takes the piers with the ring whenever a
solve does end Undecided. That release rule is a separate decision.

## Priority order

Owner decision 2026-11-10. One item at a time, re-run between items so each one's effect is known.

1. **Hysteresis on corner open/close.** Lowest risk and earliest in the chain; may reduce how often the
   restore rule is reached at all.
2. **Closing-rate check in the restore rule**, replacing the history check. The arch fold A1.4 is its test.
3. Re-run the arch, the shed and the warehouse post-removal. The shed ends in `RestoreFailedClosed` like the
   arch, and the warehouse stops on one joint flipping at 1.017, so both may clear with 1 and 2. Then check
   which parked tests follow.
4. **Crack and crush band**: relative 0.2 plus the catastrophic 2.0 level.
5. Core cases B to H.
6. Then the rest: contact-count test, segmented members, per-iteration timing, the pier footing experiment
   (after B to H), model-edge flags.

## Speed of the arch fold under the closing-rate rule

Discussed 2026-11-12. The spike's A1.4 run at threshold 0.1 takes 5.6 s (3.2 s on Linux) against the 200 ms
target, for 231 bodies and 1,386 unknowns.

Where the time goes: the arch is above `dense_dof_limit` (600), so normal iterations use the sparse path and
cost almost nothing. Only singular iterations are expensive: the sparse factoriser cannot detect rank, so the
solver falls back to a dense factorisation, about 0.27 s each. The fold needs 21 of them, 15 in pass 4 alone,
because the restore step re-closes one joint pair per side per iteration as it walks along the haunch.

Two levers: fewer singular iterations, or cheaper ones.

1. **Fewer: close all at once (try first).** In the restore step, re-close every open corner above the
   threshold in one go instead of one pair per iteration. A few lines; could cut the fold from 21 singular
   iterations to a handful.
2. **Cheaper: soft springs (MuJoCo's approach).** Add a very soft spring from every body to the world, around
   1e-6 of a joint stiffness. The matrix is never singular, so the fast sparse solve always works. A mechanism
   shows as an enormous displacement of the loose part; the shape of that displacement is the fold.
   Trade-offs: a threshold for 'enormous' (the gap is usually wide), worse numerical scaling, and the crisp
   rank count is lost. Borrows MuJoCo's trick only for the singular step; the exact statics stay elsewhere.
3. **Cheaper: sparse rank-revealing factorisation.** Skips the zeros and checks every pivot, so no dense
   fallback. Would make a singular iteration cost about what a normal one does. Libraries: SuiteSparse SPQR
   (GPL, unsuitable for the game; heavy dependency chain), Eigen's own SparseQR (already in the repo under
   MPL2 but slow and weak at rank detection), MUMPS (CeCILL-C, Fortran, heavy). None is a drop-in.
4. **Cheaper: condense the settled part.** Free motions live in the loose crown bricks; the piers are settled.
   Reducing the system to the loose region before the dense step would shrink 1,386 unknowns to a few hundred.
5. **Order of checks.** Run the dense check only when a cheap sparse pass has settled and still reads
   singular, rather than on every singular iteration.

Order decided: 1 first, then time the warehouse before choosing among 2 to 5.

Licensing: Eigen is MPL2 and the build sets `EIGEN_MPL2_ONLY`, which excludes Eigen's non-MPL2 modules at
compile time. Using Eigen unmodified in a closed-source game is fine; keep the licence notices with the
headers and credit it in the game's licence file.

## Validating the Undecided-to-Mechanism changes

Discussed 2026-11-12. Under the closing-rate rule about 2,900 suite solves that ended `RestoreFailedClosed`
now end otherwise, about 2,500 of them as Mechanism. Most are inside the seeded factor tests (6,000 random
stacks and bridges: 1,088 changed) and the grid scans. Those tests pin recorded answers, not physics, so they
went red on the change without saying whether the new answers are right.

Proposed approach (owner: 'sure, not certain how I feel about it'):

- Re-check every changed Mechanism automatically: positive gravity work along the motion, and admissibility
  (no open corner closes, no slipping corner slides against its friction).
- A small rigid-block oracle for stacks and bridges: pure statics, resultant inside the base and sideways
  force under friction. Compare verdicts on every changed case; look by hand only at disagreements.
- The one solve that moved Undecided to Stands (the lime lateral grid cell) gets a hand check regardless.
- Then re-pin the factor tests to the validated answers.

## Spike: soft springs (regularised stiffness)

Owner decision 2026-11-12: try soft springs as a spike on the arch, alongside building the closing-rate rule;
dead code is acceptable if soft springs win.

The idea: a very soft spring from every body to the world, around 1e-6 to 1e-8 of a joint stiffness (a
setting). The stiffness matrix is never singular, so every iteration uses the fast sparse path. No dense
fallback, no rank detection, no restore rule, no free-motion floor. A mechanism shows as bodies displacing
enormously (about a million times the elastic deflection); the displacement pattern is its shape. MuJoCo's
approach, borrowed for the singular step only.

Expected on the arch: the same fold, under 100 ms (the spike's 21 dense iterations at 0.27 s become sparse
ones at about 1 ms). An estimate, not a measurement.

What it should clear: `RestoreFailedClosed` entirely (arch, shed, about 2,900 suite solves); the noise-motion
asymmetry; the `dense_dof_limit`; the ungrounded 'falls rigidly' special case; the dense step that blocked
parallel component solves.

What it should not be expected to fix: corner chatter (the warehouse joint flipping at 1.017), `IterationCap`
endings, over-release and hinge splitting in the cascade.

Risks: a threshold for 'enormous' displacement (a setting); worse numerical conditioning with stiff and soft
springs in one matrix; the mechanism shape is an approximation, so the five-piece fold may come out slightly
smeared. Test on the arch and the sliding brick, then time the warehouse.

If soft springs win, the noise-motion floor is unnecessary; otherwise add it: a free motion smaller than the
elastic deflection (ratio below 1.0, a setting) is treated as no motion before the restore rule acts.
