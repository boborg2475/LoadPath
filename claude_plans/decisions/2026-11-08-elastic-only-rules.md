# Elastic-only runtime: rule decisions owed (2026-11-08)

The limit-analysis certificate was removed at `91afe97` on `solver_rework` (DESIGN §10,
2026-11-08; the code lives on branch `certificate_checkpoint`). The elastic authority now
answers alone: Stands and Overstressed pass through, a Mechanism component is Collapses and
releases the elastic solver's own `UnconstrainedBodies`, an Undecided component releases
nothing. Four rules the certificate used to supply no longer have an owner. Each section below
states the question, what the code does today, the evidence, the options and a
recommendation. **Status is OPEN until the owner rules**; record the ruling in DESIGN §10 and
change the status line here. An agent working the red list in CURRENT_STATE ("Elastic-only:
cases the solver alone gets wrong") must not change a test's expectation under an open
decision; it works the Undecided cases (decision-free) first.

---

## D1. What happens after a joint crushes?

**Status: OPEN.**

**Question.** A component whose only failure is compression over `fc · A/4` at some corner
reads elastic `Overstressed`. The crack step does nothing for compression (a crack removes
tension and cohesion and keeps compressive strength, DESIGN §10 2026-10-11), so without a
further rule the component stays Overstressed forever and nothing moves. The 2026-10-15
ruling sent such "only-crushed" components to the certificate, which decided whether the
crushed joint's loss collapses the structure. That consult is gone.

**Evidence.** Six reds: `Statics.Anchor.APierAboveItsCrushingStrengthCollapsesAndBelowItStands`,
`Statics.Combined.ACrushedComponentIsConsultedAndCollapses`,
`Statics.Cracked.CrackedBedKeepsTheCrushingBound`,
`Cascade.CrushingOverloadCracksNothingAndReleasesTheBrick`,
`E2E.Bearing.AnOverloadedTimberBearingDropsTheBeamAndSparesThePiers`, and the scenario file
`data/scenarios/beam-on-piers-load-step.json` (plus `Scenario.Steps.TheBeforeVerdictJudges…`
on the same file). The E2E case is the owner-approved one (plan 2026-10-15, answer 7): two
timber bearings at 1.2 × `f_c,90 A` crush, the beam falls, the piers (at about half capacity)
stand. BENCHMARKS F-NEG-01 is the physical precedent (towers that failed by crushing).

**Options.**

- **(a) Crushing is a release.** A crushed joint is severed (it carries nothing), the body it
  carried is re-judged, and whatever is then unsupported falls as a Mechanism. Implement as a
  crack-step action: "crushed → severed", then the next pass's elastic solve decides. Matches
  the E2E expectation (the beam falls, the piers stand) with no new solver. Risk: a joint that
  crushes under a tall pier's own weight severs the pier's base and the pier reads Mechanism
  (it falls), which is the F-NEG-01 answer but drastic when the overload is 1.01×.
- **(b) Crushing is a plastic cap.** The corner's compression is capped at `fc · A/4` in the
  active set (a fourth corner state, "crushing", analogous to slipping with a held normal
  force); the solve redistributes; the component collapses only if no field under the cap
  exists (Mechanism) and stands otherwise with the crushed corners reported. Physically the
  best answer (this is what the certificate computed) and the IMMEDIATE_UPDATES §4 "secondary"
  idea. Cost: a new corner state in the active set, with the convergence risk that brings.
- **(c) Crushing stays Overstressed.** Report it, release nothing, let the host fragment the
  brick. The pre-2026-10-15 behaviour; the E2E bearing case and F-NEG-01 stay red.

**Recommendation.** (a) now, (b) later if the census shows (a) over-releasing. (a) is a
crack-step rule with an existing test shape; (b) is solver work that belongs with the
Undecided queue.

---

## D2. Does a released set split into clusters along the motion?

**Status: OPEN.**

**Question.** The cascade groups a component's released bodies into clusters over live
interfaces (`ClustersOf`, Cascade.cpp). The certificate also returned a *hinging* list (joints
its mechanism opened or slid), and bodies joined only by a hinging joint went into different
clusters, so an arch collapsing in a four-hinge mechanism released as four rigid runs. With
the certificate gone the hinging list is gone, and every connected released set is one
cluster. The dev round removed rather than replaced it (no failing test asked for a
replacement).

**Evidence.** Six reds: `Cascade.ArchBelowLeastThicknessReleasesVoussoirRunsBetweenHinges`,
`Cascade.ArchBelowLeastThicknessOpensEveryHingeItSevers` (1 cluster and 2 failures where 7
hinges are expected), `E2E.Arch.LosingTheKeystoneLeavesOnlyTheHaunchesFrictionHolds` (two
big clusters, the wrong survivors), the two `Cascade.FloatingWall.*` tests (an uplifted upper
brick should lift off its falling partner; one cluster holds both),
`Cascade.Fuzz.SeededStructuresKeepTheCascadeInvariants` (no compression sever is seen).

**Options.**

- **(a) Split by relative motion.** The elastic Mechanism gives every released body a
  translation and rotation. Two released bodies sharing a joint stay in one cluster only if
  their relative velocity at that joint is below the listing threshold (1e-3 of the component's
  largest corner velocity, the same ratio the certificate used); otherwise the joint is a hinge
  and severs. This reproduces the old grouping from data the elastic solver already returns.
  Caveat from CURRENT_STATE: on a sliding block the elastic mechanism shape can carry a small
  twist (Rz about 7% of the corner displacement) that is not the collapse mode, so the threshold
  needs a fixture check on the three-brick bridge and the arch.
- **(b) One cluster per connected set.** Today's behaviour. The host sees fewer, larger rigid
  pieces; the arch falls as a slab. Simple and deterministic, physically wrong for arches and
  for a brick lifting off a falling partner.
- **(c) Split by the active set.** A joint whose corners are all open or all slipping in the
  final active set is a hinge. Needs no motion threshold, but a half-open rocking joint (two
  corners bearing) would weld two bodies that in fact rotate about that edge.

**Recommendation.** (a). It is the certificate's rule applied to the elastic motion, and the
six red tests already pin it.

---

## D3. Is an elastic Mechanism always a release?

**Status: OPEN.**

**Question.** The elastic solver's Mechanism was ruled "a verdict, not a certificate" on
2026-10-11 because it once read Mechanism where a standing field existed (a dry three-brick
bridge at μ 5). The certificate refereed every Mechanism; now every Mechanism releases. Two
old tests pinned the certificate overruling the elastic solver inside the friction cone's
polygon band, and two pinned fixtures the elastic solver "sees fall" that were expected to
read Undecided pending the referee.

**Evidence.** Four reds: `Statics.Anchor.FreeWallWithFrictionBelowThicknessOverHeightSlidesAtFriction`
and `Statics.Anchor.SeatedBeamSlidesAtTheMirandolaAccelerationOnNeopreneAndHoldsAtTheCodeFriction`
(inside the old 16-generator cone band the certificate read Undecided; the elastic solver
reads Mechanism and now Collapses, which is the exact-friction answer),
`Statics.Combined.ThreeBrickBridgeTheElasticSeesFallReadsUndecided` (releases {2, 3}) and
`Statics.Combined.DryTwoCourseWallTheElasticSeesFallReadsUndecided` (releases all 5 bricks).
Realistic friction (μ 0.4 to 1.0) showed no false Mechanism in 3,000 random stacks and
bridges; the μ 5 case is the only known one.

**Options.**

- **(a) Mechanism releases, always.** Today's behaviour. The two cone-band tests are
  re-expected to Collapses (they asserted an artefact of the certificate's polygon, not
  physics). The two "sees fall" tests become checks of *what* is released: owner to say whether
  {2, 3} on the bridge and all 5 on the two-course wall are the right answers; if not they join
  the Undecided queue as wrong-release cases.
- **(b) Mechanism releases only when a static check confirms it.** Re-solve the component with
  the released bodies removed; if the remainder stands and the released set has no path to
  ground, release. A cheap self-check against the one known false Mechanism. Adds a solve per
  Mechanism component.

**Recommendation.** (a), with (b) kept as the first thing to try if a false release turns up
in the census. The two cone-band tests should be re-expected under this ruling, not before.

---

## D4. 3.13: does "exactly the 10-brick triangle" stand as the expectation?

**Status: OPEN** (owed since 2026-10-25; unchanged by the removal).

**Question.** `Deferred.Slice6.DryWallLosingItsLintelKeepsTheBricksOverThePiers` asserts
that the 10 bricks over the opening fall and the 4 bricks wholly on the piers stay; the
stepped shoulder bricks are unasserted, but the basis text in `viewer/reasons.json` and the
plan say "exactly the 10-brick triangle falls".

**Evidence** (`research_notes/Debugging/3.13 lintel wall/`, DEBUG_PLAYBOOK §2): with the
triangle removed beforehand the shoulders still collapse at μ 0.7 under the associative rule,
which bounds the real answer from above, so some shoulder corbels (c9x3.00, c8x2.50 and
mirrors) must fall too. The elastic solver alone under-releases here (2 of 10, then
Undecided through the restore rule on loose end bats), so the test stays red either way until
the Undecided work lands.

**Options.** (a) Keep the assertions, reword the basis to "the triangle falls and the pier
bricks stay; the shoulder corbels are not asserted". (b) Assert the corbels fall too (needs the
list; the certificate-free engine cannot yet produce it). (c) Assert they stay (contradicts the
bound).

**Recommendation.** (a).

---

## Not decisions, for the record

- The 12 **Undecided** reds need no ruling: an Undecided where an answer is expected is a
  solver defect by the 2026-10-11 contract (fail closed). They are the work queue, starting with
  the restore rule failing on lightly loaded end bricks (seen on the 3.13 wall intact, after the
  lintel, and with the triangle pre-removed) and the warm re-judge chatter (IMMEDIATE_UPDATES §4).
- Coverage lost with the certificate (screw slip and pull-out anchors, the bolt-row check, the
  dry-wall t/H brackets, the Heyman arch anchors) is a test-plan item, not a rule: those
  anchors need elastic-authority versions, and the owner approves test plans.
