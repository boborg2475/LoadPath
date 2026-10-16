Status: proposed

# End-to-end validation suite: test plan (2026-10-15)

Before and expected-after pictures for every case below are in the E2E plan viewer page,
https://claude.ai/artifact/E4f4zBNCHNXr2u3zRZzazp (one entry per row). Each case lists its entries on its
`Visual:` line.

The owner's ask (IMMEDIATE_UPDATES §3): whole-structure scenarios through the public API (build a
structure or load it from a file, judge it, run the cascade, remove pieces, apply loads), checked
against outcomes the owner can defend: who stands, what cracks, what falls and in which clusters.
Every feature is validated against the suite before it lands, so it has to stay fast.

Every case below goes through the public API only: `Structure` (or `geometry::BuildRunningBondWall`,
`BuildWarehouse`, `FaceContact` on boxes, or `LoadStructure`), the catalogue rows from
`data/catalogue.json`, `MakeCascade(MakeCombinedAuthority(...))`, `RemoveBody`, `BodyLoad` and the
gravity vector in the options. Assertions are on **outcome** (DESIGN §6): the set of released
bodies by name, the cluster partition where it matters, the final cascade state, and that the
remainder judges Stands. Never displacement. A pass count is asserted only where the order of events
is the behaviour (crack first, then tip).

"Today" in each entry is what the engine does now. I measured it with a throwaway probe compiled
against the Release library at HEAD dfde1c9 (in the session scratchpad, not in the repository; no
test was added or changed).

## 1. What already exercises a whole structure

| Family | What it builds | What it asserts | Kind | Tier, Release time | End to end? |
|---|---|---|---|---|---|
| `Cascade.*` slice-3 rows (CascadeTest.cpp) | one to three bricks, a bonded pier, two separate collapses, the Couplet–Heyman arch below least thickness | released clusters, severed and cracked joints, stamps, pass counts, final state | outcome and mechanism | default (the two arch rows `LOADPATH_SLOW_IN_DEBUG`), about 0.4 s | Yes, but only on tiny fixtures. The closest thing today. |
| `Cascade.FloatingWall.*` | walls with the base course removed, floating pairs | a wall with no ground falls as one cluster | outcome | default (two `Slow.` versions at 10 × 8), 12 ms | Yes, for one topological rule only. |
| `Cascade.Fuzz.*` | seeded stacks and 3 to 6 course walls, 0 to 3 bricks removed, dry or EC6, tilt or lateral load | five invariants: clusters certified, stamps consistent, termination, remainder re-judged, determinism, file round trip | property | `LOADPATH_SLOW_IN_DEBUG`, 1.8 s | No. It checks the answer is consistent, not that it is right. |
| `Cascade.EventRegion.*`, `Authority.Judge.*` | small structures judged twice | a warm re-judge equals a fresh one | mechanism | default, about 7 s | No. |
| Statics and certificate anchors (ANCHORS.md: single blocks, inclines, Housner, free walls, harmonic stacks, diamonds, wedges, Couplet–Heyman arch, Orduña pile, Ferris–Tin-Loi, Liberton, Christchurch, LNEC, Mirandola; Foraboschi and Minho in `Slow.`) | whole structures from papers and codes | Stands or Collapses either side of a published threshold, load-factor brackets, hinge positions | outcome, one judge | default, `LOADPATH_SLOW_IN_DEBUG` and `Slow.`; about 7 s default, minutes in `Slow.` | Partly. Whole structures through the API, but one judge: nothing removed, no cascade. |
| `Certificate.Anchor.OnlyThe…`, `AStableBystanderIsNotListed` | two to four bricks | which bodies the certificate says move | outcome | default, under 0.1 s | Partly (judge level, tiny). |
| `Geometry.Anchor.*` | running-bond walls 4 × 5 and 10 × 8, dry and EC6 | stands at rest; overturns at t/H and at the bonded monolith factor; removing a base brick raises the beds' utilisation | outcome, one judge | `LOADPATH_SLOW_IN_DEBUG` and `Slow.`, 0.9 s | Partly. One removal case, judged rather than cascaded. |
| `Catalogue.Anchor.*` | one brick on a bed, a two-screw joint | published strength × area thresholds | outcome | default, 0.03 s | No (one joint). |
| `Statics.Dry.*`, `Statics.Lime.*` lateral grids and pins | many small walls under lateral load | stands, own-field check, oracle agreement | outcome and property | default and `Slow.` | No (verdict only). |
| `Measure.Scale.*` | 10 × 8 and 32 × 40 walls, shed, half warehouse, warehouse | Stands at rest, same verdict twice, warm post-removal cascade ends Stands; prints times | Stands only, plus timing | `Measure.` (not a tier; the two cheap rows run at slice close, about 2 s) | Partly. The only whole-building cascades, but nothing checks **what** is released. |
| `Geometry.Warehouse.*`, `Geometry.Wall.*`, `Geometry.Contact.*` | the shed and warehouse builder, walls, contacts | counts, positions, joints | mechanism (geometry) | default, 0.03 s | No physics. |
| `File.*`, `Cli.*`, `Model.StructureFile.*`, `Cascade.PassCounterSurvivesTheStructureFile` | JSON documents | parse, write, round trip; the pass counter survives a reload mid-cascade | mechanism | default | No (format only). |
| `data/` | only `catalogue.json` | | | | No structure file is checked in. |

## 2. Gaps

What an owner would expect a destruction engine to get right end to end, which nothing checks today:

- **Removing a load-bearing member brings down exactly what it carried.** No test removes a post, a
  lintel or a keystone and checks the released set. The shed and warehouse cascades after a post
  removal are timed, not checked.
- **Removing a non-structural piece changes nothing.** Only judged (one bonded base-brick case),
  never cascaded.
- **Fastened joints change what falls together.** The screw rows are pinned on one joint; no frame
  test shows a fastened corner carrying a cantilever or tipping as one piece with its post.
- **Openings and lintels.** No wall with an opening and a lintel exists in the suite.
- **An arch losing its keystone.** Only the arch below least thickness is cascaded.
- **Cantilever and counterweight.** Only harmonic stacks, judged.
- **A lateral push to overturning, followed through the cascade** (crack, then tip, then one slab).
  Pinned at one judge, and on a three-brick pier through the cascade.
- **Crushing in a multi-material assembly** (timber on brick). Pinned on one brick.
- **Confinement**: a collapse that must stop at the next support (F-RON-02, untied against tied).
- **A building-scale outcome** on the shed: which roof pieces fall when posts go.
- **A structure file loaded and run end to end**, with the same outcome as the in-memory build.
- **Build order**: nothing checks that the outcome depends on the structure, not on the order its
  bodies were added.

## 3. Proposed cases

Ordered by value. Timber is C24 at its EN 338 mean density, 420 kg/m³; bricks are 0.215 × 0.1025 ×
0.065 m at 2000 kg/m³ (DESIGN §3). "Bearing" is the `c24_bearing_perp_to_grain` row (dry,
μ 0.3, f_c 3.75 MPa); "EC6" is the `clay_gp_mortar_bed_ec6` and `..._perpend_ec6` rows; "dry
brick" is the EC6 bed with cohesion and tension set to zero and μ 0.7. Debug times are estimated at
40 × Release.

### 3.1 `E2E.Colonnade.RemovingAPostDropsOnlyTheBeamsItCarried` (must-have)

Visual: colonnade-dry-minus-post2, colonnade-dry-minus-post0

- **Behaviour:** removing one post of a dry post-and-beam colonnade releases the beams that rested
  on it, each as its own cluster, and nothing else.
- **Fixture:** five timber posts 0.2 × 0.2 × 2.0 m at 2.2 m centres on a grounded slab; four beams
  0.2 wide × 0.3 deep × 2.199 m, each bearing on half of each post top (0.0995 m), 1 mm apart end to
  end; every joint Bearing. Rows: remove post 2 (middle); remove post 0 (end).
- **Assertion (outcome):** released set and clusters by name; final state Stands; every other body
  live. Row post 2: clusters {beam1} and {beam2}, pass 1. Row post 0: {beam0}.
- **Expected, and why:** closed form. A beam left on one dry bearing has its centroid about 1.05 m
  beyond that bearing and no tension to hold it, so it tips; a beam on two bearings stands; dry
  bearings pass no moment, so the neighbouring bays are untouched (F-RON-02, untied: collapse
  confined to one bay).
- **Today:** green (exactly this answer). Mutant: `RemoveBody` leaves the removed post's interfaces
  live, so nothing falls.
- **Tier:** default, about 10 ms Release, 0.4 s Debug.

### 3.2 `E2E.Arch.LosingTheKeystoneLeavesOnlyTheHaunchesFrictionHolds` (must-have)

Visual: arch-keystone-intact, arch-keystone-minus-crown-pair, arch-keystone-minus-one-crown-voussoir

- **Behaviour:** an arch that stands loses its crown voussoirs, and exactly the part of each
  half-arch that cannot stand as a free chain falls; the result is mirror symmetric.
- **Fixture:** the Couplet–Heyman semicircle of `tests/ArchFixtures.h` at t/R = 0.15 (above the
  0.1075 least thickness), radial joints every 3° plus ±54.484°, μ 0.7 (62 voussoirs). Rows: remove
  the two voussoirs at the crown; remove one crown voussoir.
- **Assertion (outcome):** intact arch Stands with nothing released; after removal, the surviving
  voussoirs equal the oracle's set exactly, both sides; final state Stands.
- **Expected, and why:** an independent oracle in the test. A half-arch with a free top is an open
  chain, so it is statically determinate: each joint carries exactly the weight of the voussoirs
  above it, vertically. The chain from the springing up to joint k stands only if, at every joint
  below k, the vertical through the centroid of the part above crosses the joint
  (R_i sin φ ≤ x̄ ≤ R_o sin φ) and the joint does not slide (cot φ ≤ μ). The longest such chain ends
  at the 54.484° joint: one voussoir higher, cot 54.484° = 0.714 exceeds μ = 0.7. So 12 voussoirs
  stand on each side and 36 fall (37 in the one-voussoir row).
- **Today:** green (the engine releases those 36, one per side per pass: 18 passes for the two-voussoir row, 19 for one). Mutant: scale
  the friction coefficient the certificate reads by 1.05, which moves the frontier one voussoir.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 0.5 s Release (the largest case).

### 3.3 `E2E.Wall.RemovingANonStructuralBrickReleasesNothing` (must-have)

Visual: wall-ec6-minus-top-end-brick, wall-ec6-minus-mid-brick, wall-dry-minus-top-end-brick

- **Behaviour:** pulling a brick that carries nothing releases nothing, cracks nothing, and the wall
  stands.
- **Fixture:** a 10 course × 8 brick running-bond wall. Rows: EC6, top-course end brick removed;
  EC6, a mid-height brick in the middle removed; dry brick, top-course end brick removed.
- **Assertion (outcome):** `Passes == 0`, no failures, no clusters, every other brick live, state
  Stands.
- **Expected, and why:** engineering judgement stated plainly: a brick with nothing above it, or one
  whose neighbours bridge it by bond, carries no load another piece depends on.
- **Today:** green. Mutant: raise the dry-corner noise floor (`DryNoiseFloorRelative`) to 1e-3, so
  the dry row reads Undecided; for the EC6 rows, a mutant that maps an elastic Stands to Undecided.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG` (the dry wall takes about 80 ms Release); about 0.1 s Release.

### 3.4 `E2E.Colonnade.ScrewedBearingsBridgeALostPostAboveTheWithdrawalThreshold` (must-have)

Visual: colonnade-screwed-2-screws-minus-post2, colonnade-screwed-3-screws-minus-post2

- **Behaviour:** fastening the beams to the posts lets the frame bridge a lost interior post once
  the screws can hold the cantilever, and not before.
- **Fixture:** the colonnade of 3.1 with every beam-on-post joint made by `FastenerProfile` of the
  `ec5_screw_8x60_steel_to_glulam` row (withdrawal 5.24 kN) with n screws over the 0.0995 × 0.2 m
  bearing; post bases stay Bearing. Remove post 2. Rows: n = 2 and n = 3.
- **Assertion (outcome):** n = 2: the two beam-on-post joints next to the gap crack in tension on
  pass 1, then {beam1} and {beam2} are released on pass 2, everything else stands. n = 3: nothing
  released, nothing cracked, state Stands.
- **Expected, and why:** closed form, from the catalogue's EC5 withdrawal. A beam left on one post
  overturns about the bearing's inner edge with moment W·a = 543.6 N × 1.0 m against
  n · F_ax · L/2 = n × 5240 N × 0.04975 m, so it holds when n ≥ 2.08. F-RON-02 (tied: the loss is
  bridged) is the published precedent.
- **Today:** green for both rows (the n = 2 crack ratio reads 1.042 against the closed form's
  1.04). Mutants: withdrawal × 0 in `FastenerProfile` (the n = 3 row falls); withdrawal × 1.2 (the
  n = 2 row stands).
- **Tier:** default, about 40 ms Release, 1.6 s Debug.

### 3.5 `E2E.Colonnade.AScrewedEndBayTipsWithItsPost` (must-have)

Visual: two-bay-screwed-2-screws-minus-middle-post, two-bay-screwed-3-screws-minus-middle-post, two-bay-dry-minus-middle-post

- **Behaviour:** when a fastened joint holds, the pieces it joins fall together.
- **Fixture:** a two-bay colonnade (three posts, two beams) with screwed beam-on-post joints, n = 2
  and n = 3; remove the middle post.
- **Assertion (outcome):** two clusters, {post0, beam0} and {post2, beam1}, pass 1; the ground is all
  that remains. Contrast row: the same frame with Bearing joints releases {beam0} and {beam1} and
  leaves both end posts standing (confirmed by the picture run, 2026-10-16).
- **Expected, and why:** closed form. Post and beam together have their centroid about 0.68 m from
  the post's centre line, far outside its 0.1 m half-width, and the post base carries no tension,
  so the pair tips about the base edge facing the gap; with dry bearings the beam tips alone off a
  post that is then stable.
- **Today:** green. Mutant: the cascade treats every interface as hinging when grouping clusters
  (four singleton clusters).
- **Tier:** default, about 10 ms Release.

### 3.6 `E2E.Wall.BondedWallPushedPastItsLimitFallsAsOneSlab` (must-have)

Visual: bonded-wall-push-0-98, bonded-wall-push-1-02

- **Behaviour:** a mortared wall under a lateral push just past its limit cracks at the base, then
  overturns as one piece; just below the limit nothing happens.
- **Fixture:** 5 courses × 4 bricks running bond (22 bodies), EC6, lateral acceleration λ g across
  the thickness. λ_c = (t/H)(1 + f_t A_base / W) ≈ 1.965, computed in the test from the row's
  numbers. Rows: 0.98 λ_c and 1.02 λ_c.
- **Assertion (outcome):** 0.98: nothing cracked or released, Stands. 1.02: pass 1 cracks only the
  base beds, in tension; pass 2 releases all 22 bricks as one cluster; `Passes == 2`; only the
  ground remains.
- **Expected, and why:** the bonded monolith overturning formula (already pinned at one judge in
  `Geometry.Anchor.SmallBondedWallOverturnsAtTheBondedMonolithFactor`); the base governs strictly
  because every bed above carries less weight against the same bond. F-VIR-01: with modest joint
  tension, a chimney falls as a single mass.
- **Today:** green. Mutant: every interface hinging when grouping clusters (the slab splits).
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 0.16 s Release.

### 3.7 `E2E.Wall.DryWallPushedPastThicknessOverHeightFallsEntirely` (must-have)

Visual: dry-wall-push-0-98, dry-wall-push-1-05

- **Behaviour:** a dry-laid wall under a lateral push past t/H falls entirely; below it, nothing
  falls.
- **Fixture:** the wall of 3.6 in dry brick (μ 0.7 > t/H). Rows: 0.98 t/H and 1.05 t/H (t/H =
  0.315).
- **Assertion (outcome):** 0.98: nothing released, Stands. 1.05: every brick released, only the
  ground remains. The cluster count is not asserted (open question 4).
- **Expected, and why:** B-WAL-01, the free wall overturning at λ = t/H about its toe when
  μ > t/H.
- **Today:** green (released as one cluster). Mutant: shrink the contact half-extents the
  certificate reads by 10 %, which moves the threshold below 0.98 t/H.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 40 ms Release.

### 3.8 `E2E.Bearing.AnOverloadedTimberBearingDropsTheBeamAndSparesThePiers` (must-have)

Visual: timber-bearing-load-0-8, timber-bearing-load-1-2

- **Behaviour:** in a timber beam on brick piers, a heavy load crushes the weaker timber bearing,
  so the beam falls and the brickwork stands.
- **Fixture:** two single-brick piers of 5 courses (EC6), 0.785 m apart; a timber beam
  1.215 × 0.1025 × 0.15 m resting on both pier tops (bearing area 0.215 × 0.1025 m, Bearing row); a
  point load P at midspan. Rows: P set so each bearing carries 0.8 and 1.2 of f_c,90 · A.
- **Assertion (outcome):** 0.8: nothing released, Stands. 1.2: {beam} released on pass 1, all ten
  pier bricks live, Stands.
- **Expected, and why:** closed form from the catalogue: each bearing carries P/2 + W_beam/2 against
  f_c,90 · A = 3.75 MPa × 0.0220 m² = 82.6 kN (EN 1995-1-1 §6.1.5, Swedish Wood Ex 3.2), while the
  brick beds sit at about half of the EC6 f_k · A = 196.8 kN (EN 1996-1-1 eq. 3.2). That crushing
  releases what is above the crushed joint is the 2026-10-15 crushing ruling (DESIGN §10).
- **Today:** green. Mutant: skip the certificate consult on an only-crushed Overstressed component
  (the 1.2 row ends Overstressed with nothing released).
- **Tier:** default, about 20 ms Release, 0.8 s Debug.

### 3.9 `E2E.Shed.LosingOneSideOfPostsDropsTheRoofBayByBay` (must-have)

Visual: shed-minus-side-posts

- **Behaviour:** at building scale, when every post along one side goes, each bay's roof falls as
  one piece and nothing else moves.
- **Fixture:** `ShedSpec()` from tests/WarehouseTestSupport.h (12 × 8 brick walls, 10 courses, two
  openings, two bays, three planks per bay, EC6 walls, Bearing posts and beams, planks screwed with
  two screws per joint). Remove the three posts along wall 0.
- **Assertion (outcome):** exactly two clusters, {beam0, beam2, planks 0 to 2} and {beam1, beam3,
  planks 3 to 5}; the three far posts and every wall brick live; state Stands.
- **Expected, and why:** closed form. With the near posts gone, each bay's beams and screwed planks
  rest only on dry bearings atop the far posts, with their centroid well to the near side, so they
  tip about the far post tops. The two bays share no joint, and the frame does not touch the walls
  (five components by construction), so nothing else can be affected.
- **Today:** green. Mutant: withdrawal × 0 in `FastenerProfile` (the planks separate from the
  beams).
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 0.1 s Release.

### 3.10 `E2E.Shed.LosingOnePostLeavesTheScrewedRoofStanding` (must-have; judgement call)

Visual: shed-minus-middle-post, shed-minus-corner-post

- **Behaviour:** losing a single post does not bring the shed's roof down, because the screwed
  planks carry the beam ends that lost their support.
- **Fixture:** `ShedSpec()`. Rows: remove the middle post on wall 0's side; remove a corner post.
- **Assertion (outcome):** nothing released, state Stands.
- **Expected, and why:** engineering judgement, with a check: the free beam ends hang from three
  planks each, which cantilever from the far beams; the demand per plank joint is about 85 N·m
  against a capacity of about 1,050 N·m (two screws × 5.24 kN acting over half the 0.2 m joint). A
  real screwed frame would sag here but stand. This is open question 1: the owner may prefer the
  roof to drop.
- **Today:** green (this is the row `Measure.Scale.ShedPostRemovedWarm` times but never checks).
  Mutant: withdrawal × 0 in `FastenerProfile`.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 0.1 s Release.

### 3.11 `E2E.Cantilever.TheCounterweightHoldsTheBeamAndItsRemovalTipsIt` (must-have)

Visual: cantilever-no-counterweight, cantilever-16kg-counterweight, cantilever-4kg-counterweight, cantilever-16kg-counterweight-removed

- **Behaviour:** a beam overhanging its post stands only while a counterweight keeps the combined
  centroid over the post, and falls with whatever rides on it.
- **Fixture:** timber post 0.2 × 0.2 × 1.0 m; timber beam 1.6 × 0.2 × 0.1 m from x = −0.4 to 1.2 m on
  the post top (x 0 to 0.2 m); a 0.2 m cube counterweight on the beam's back end; all joints
  Bearing. Rows: no counterweight; 16 kg (2000 kg/m³); 4 kg (500 kg/m³); 16 kg then removed.
- **Assertion (outcome):** none: {beam} released, post stands. 16 kg: nothing released. 4 kg:
  {beam, counterweight} released as one cluster, post stands. 16 kg removed: {beam} released.
- **Expected, and why:** closed form, the lever rule: the beam's centroid is 0.2 m past the post's
  edge; the counterweight must be at least 5.36 kg to pull the combined centroid back over the post
  (16 kg puts it at x = 0.019 m; 4 kg leaves it at 0.239 m).
- **Today:** green. Mutant: every interface hinging when grouping clusters (the 4 kg row splits).
- **Tier:** default, about 5 ms Release.

### 3.12 `E2E.File.ALoadedColonnadeFallsLikeTheBuiltOne` (must-have)

Visual: file-colonnade-intact, file-colonnade-minus-post2

- **Behaviour:** a structure loaded from a checked-in file gives the same outcome as the same
  structure built in code, and its end state survives a save and reload.
- **Fixture:** a new data file, `data/structures/colonnade.json` (data, so TDD-exempt): the dry
  colonnade of 3.1 with named bodies and the Bearing row's numbers written in pascals. Load it,
  cascade, remove the body named `post2`, cascade, write, reload, judge.
- **Assertion (outcome):** intact Stands with nothing released; after the removal, clusters
  {beam1} and {beam2} by name, the same as 3.1's in-memory build; the reloaded end state judges
  Stands with the same live bodies.
- **Expected, and why:** as 3.1.
- **Today:** red only because the file does not exist yet; green once it does. Mutant: the reader
  reads `grounded` as false (everything falls).
- **Tier:** default, about 10 ms Release.

### 3.13 `Deferred.Slice6.DryWallLosingItsLintelKeepsTheBricksOverThePiers` (must-have; red today)

Visual: dry-lintel-intact, dry-lintel-removed-expected, dry-lintel-removed-today

- **Behaviour:** removing the lintel of a dry-laid wall drops the masonry over the opening and
  leaves the bricks that sit wholly on the piers.
- **Fixture:** a 10 × 8 dry-brick running-bond wall built brick by brick, with a door opening four
  bricks wide (x 0.43 to 1.29 m) through courses 0 to 4, and a timber lintel 1.075 × 0.1025 ×
  0.065 m replacing course 5 over x 0.3225 to 1.3975 m (half a brick of bearing each side); the
  lintel's joints are Bearing.
- **Assertion (outcome):** with the lintel, Stands. Lintel removed: the four course-6 bricks with no
  bed under them are released; every pier brick (courses 0 to 5) is live, and so are the bricks
  that sit wholly on pier bricks: the course-6 end bricks (x 0 to 0.215 m and its mirror) and the
  course-7 half bricks above them. Bricks whose centroid lands exactly on a support edge (running
  bond's corbel steps) are not asserted either way.
- **Expected, and why:** lower-bound statics: a block resting wholly on standing blocks, under
  gravity alone, with dry joints, cannot be pulled down by blocks falling beside it. And the
  unsupported bricks must fall: the four courses above the opening cannot arch across 1.075 m,
  because the thrust (about 380 N) exceeds the friction the light abutments above the piers can
  supply (about 120 N).
- **Today:** red. The engine releases all 34 bricks above course 5, end bricks included, as 26
  clusters on pass 1 with 76 severs. This is the certificate's mechanism selection listing stable
  bystanders at t* = 0, the problem the owed least-joint-movement selection fixes (DESIGN §10,
  2026-10-11; the other `Deferred.Slice6.*` cases). It stays under `Deferred.` with a CURRENT_STATE
  entry and moves to `E2E.Wall.*` when it turns green. Open question 3.
- **Tier:** `Deferred.`, about 0.5 s Release.

### 3.14 `E2E.Wall.BondedWallStandsOverItsOpeningWithoutTheLintel` (nice-to-have; judgement call)

Visual: bonded-lintel-intact, bonded-lintel-removed

- **Behaviour:** in a mortared wall, a modest opening stays bridged by the bond after its lintel is
  taken out.
- **Fixture:** the wall of 3.13 with EC6 rows.
- **Assertion (outcome):** with and without the lintel: nothing released, nothing cracked, Stands.
- **Expected, and why:** engineering judgement, with a check: the four courses above act as a
  bonded deep beam, M = wL²/8 ≈ 75 N·m against about 115 N·m of plastic capacity from the row's
  0.033 MPa perpend tension. Real practice puts a lintel there anyway; open question 8.
- **Today:** green. Mutant: `EffectiveProfile` reads every joint as cracked (no bond), so the
  masonry over the opening falls.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 30 ms Release.

### 3.15 `E2E.Wall.DryWallWithAGapInItsBaseReleasesNothing` (nice-to-have; judgement call)

Visual: dry-wall-minus-two-base-bricks, dry-two-brick-lintel-removed

- **Behaviour:** a dry wall with two adjacent base bricks knocked out (or a two-brick opening with
  its lintel removed) drops nothing.
- **Fixture:** 10 × 8 dry-brick wall. Rows: the two base bricks at x 0.645 to 1.075 m removed; the
  wall of 3.13 with a two-brick opening (x 0.645 to 1.075 m) and its lintel removed.
- **Assertion (outcome):** nothing released, no failures.
- **Expected, and why:** engineering judgement: the bricks over the gap corbel or wedge between
  their neighbours, and the certificate finds a standing field.
- **Today:** the assertion above is green, but the state is **Undecided**, not Stands: the elastic
  authority says Mechanism and the certificate says Stands, which the combined authority reports
  as a disagreement (Undecided, nothing released, fail closed). If the owner wants Stands, that
  row goes under `Deferred.` until the parked "trust the elastic answer" work (IMMEDIATE_UPDATES
  §4). Open question 2.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 0.6 s Release for both rows.

### 3.16 `E2E.Property.BuildOrderChangesNoOutcome` (nice-to-have)

Visual: build-order-shuffled-screwed-colonnade

- **Behaviour:** the outcome depends on the structure, not on the order its bodies and joints were
  added.
- **Fixture:** the cheap fixtures above (3.1, 3.4, 3.5, 3.8, 3.11), each rebuilt under 8 seeded
  shuffles of body and interface insertion order.
- **Assertion (property):** the released sets, the cluster partition and the final state, all by
  name, equal the unshuffled run's; the failing seed and fixture are printed.
- **Expected, and why:** the physics has no notion of insertion order. The oracle is the unshuffled
  run itself, which is legitimate here because the property is invariance, not correctness.
- **Today:** green. The picture run (2026-10-16) rebuilt the 12 rows of 3.1, 3.4, 3.5, 3.8 and
  3.11 under 8 seeded shuffles each: 96 of 96 matched the unshuffled outcome by name.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 0.5 s Release.

### 3.17 `E2E.Corner.InterlockingRaisesTheOverturningLoad` (nice-to-have)

Visual: corner-straight-joint-push, corner-interlocked-push

- **Behaviour:** a façade bonded into its return wall at the corner resists a push that topples the
  same façade with a straight joint at the corner.
- **Fixture:** two dry-brick walls at a right angle, 6 courses, façade 6 bricks, return 4 bricks.
  Variant A: a 10 mm gap at the corner (as the warehouse builds its corners). Variant B: the corner
  brick alternates between the two walls course by course. Lateral push outward, normal to the
  façade, at 1.1 t/H.
- **Assertion (outcome):** A: the façade is released and the return wall stands. B: nothing
  released.
- **Expected, and why:** A is B-WAL-01 (a free wall at t/H). B is F-STR-02 (Casapulla et al. 2024:
  interlocking raises the collapse multiplier); the published case gives the direction but not the
  size, so 1.1 t/H is an engineering choice the owner should accept or change.
- **Today:** green for both rows with a corner built for the pictures (straight joint: the 39
  façade bricks fall as one cluster and the return stands; interlocked: nothing falls). The test
  still needs a corner builder in its support code, since the geometry layer has none.
- **Tier:** `LOADPATH_SLOW_IN_DEBUG`, about 0.3 s Release (estimate).

## 4. How the suite runs

- **Prefix and file.** `E2E.<Structure>.<Behaviour>` in `tests/EndToEndTest.cpp`, with a small
  `tests/EndToEndSupport.h` (a box-scene builder that joins touching boxes by `FaceContact`, a
  name map, and one checker for "released set, clusters, survivors, final state, remainder
  Stands"). Red cases go under `Deferred.<slice>.*`, as CLAUDE.md requires.
- **Tier.** `E2E.*` is part of the default run, so no new step is needed: dev-expert's Release
  default run, the reviewer's Release run and the orchestrator's Debug run already cover it, which
  is exactly "validated before it lands". Cases over about 1 s in Debug carry
  `LOADPATH_SLOW_IN_DEBUG`, so the Release default run is the one that validates them.
  `-tc="E2E.*"` is a quick check while iterating. No row of the "Each tier runs once per change"
  table changes.
- **Budget.** Release, the whole prefix: at most 3 s serial (estimate 2.3 s), no case over 1 s.
  Debug, the cases not marked slow-in-Debug: at most 5 s (estimate 3 s). A case that outgrows its
  budget moves to `LOADPATH_SLOW_IN_DEBUG`, never to `Slow.`, so the suite always runs at every
  feature close.
- **Changing an expectation.** An `E2E.*` outcome changes only through an owner-approved amendment
  to this plan, never in a dev round. That is what makes the suite a validation gate rather than a
  set of pins.
- **Existing tests counted as part of the suite** (they stay where they are; they already run in
  the default tier, and E2E cases do not duplicate them):
  `Cascade.BondedPierCracksItsBaseThenTipsAsOneCluster`,
  `Cascade.StackWithOnlyTheTopPastItsEdgeReleasesTheTopAlone`,
  `Cascade.StackSlidingAtTheGroundIsReleasedAsOneCluster`,
  `Cascade.ArchBelowLeastThicknessReleasesVoussoirRunsBetweenHinges`,
  `Cascade.CrushingOverloadCracksNothingAndReleasesTheBrick`,
  `Cascade.FloatingWall.DryWallWithoutItsBaseCourseReleasesAsOneCluster`,
  `Cascade.FloatingWall.BondedWallWithoutItsBaseCourseReleasesAsOneCluster`,
  `Geometry.Anchor.RemovingABaseBrickArchesTheLoadOntoItsNeighbours`,
  `Geometry.Anchor.SmallDryWallOverturnsAtThicknessOverHeight`,
  `Geometry.Anchor.SmallBondedWallOverturnsAtTheBondedMonolithFactor`,
  `Certificate.Anchor.OnlyTheUnsupportedEndBrickFalls`,
  `Certificate.Anchor.AStableBystanderIsNotListed`, and `Cascade.Fuzz.*` as the property net.
  `Measure.Scale.ShedPostRemovedWarm` stays a timing row; 3.10 checks its outcome.
- **Sources.** Each case's pinned numbers and their sources go into ANCHORS.md under the test name
  when the tests are written, as for every anchor.

## 5. Open questions for the owner

1. **The shed losing one post (3.10).** The engine says the screwed roof bridges it and nothing
   falls, and the arithmetic backs that (about 8 % of the plank joints' moment capacity). Accept
   "stands", or should a lost post drop the bay it supported?
2. **A dry wall with a gap in its base (3.15).** Today it reads Undecided (elastic Mechanism,
   certificate Stands): nothing falls, but the state is not Stands. Pass the case on "nothing
   released" now, or require Stands and hold it under `Deferred.` until the elastic-trust work?
3. **The dry-wall lintel (3.13).** Today the engine drops 34 bricks in 26 clusters, including
   bricks sitting wholly on the piers. Keep it as a `Deferred.` red case for the owed
   least-joint-movement selection, or bring that selection forward? It is the clearest
   player-visible over-release I found. Also: the assertion names only the four course-6 bricks as
   released; the six above them (courses 7 to 9) rest only on falling bricks and must fall too.
   Assert them as well? (Recommended: yes.)
4. **A dry wall pushed over (3.7) falls as one cluster.** F-VIR-01 says a cohesionless chimney
   "crumbles course by course". The plan asserts only that every brick falls and leaves the cluster
   count to the host (Chaos fragments a released cluster on impact). Should the suite pin more
   clusters for dry work?
5. **The keystone (3.2) comes down one voussoir per side per pass** (18 passes, 36 single-body
   clusters) rather than as two hinged runs in one pass. It all happens inside one `Run`, so the
   host sees one event. The plan asserts only the final set and the symmetry. Accept?
6. **Fastener values are characteristic (3.4, 3.5, 3.9, 3.10).** The screw threshold (2 screws fail,
   3 hold) uses EC5 characteristic withdrawal, since no mean fastener row exists (B-CAT-09). With
   mean values the frame would be stronger. Accept the characteristic basis for E2E outcomes?
7. **Crushing drops the beam (3.8).** Under today's crushing ruling a crushed bearing releases what
   is above it, instead of the beam sinking or the crushing spreading (the softening law is still a
   CURRENT_STATE item). Accept this as the expected outcome for now?
8. **A mortared wall over an opening with no lintel stands (3.14).** Plausible from the bond
   arithmetic, but builders always fit a lintel. Include it as an expected outcome, or leave it out?
9. **Buildings as data.** Only 3.12 loads from a file. Should the other fixtures also become
   checked-in structure files under `data/structures/` (slower to write, closer to how levels will
   arrive), or stay built in code?
10. **Scope.** 13 must-haves and 4 nice-to-haves. Approve, cut, or add (for example a two-storey
    frame, or the full warehouse in Release only, at about 0.6 s per cascade).
