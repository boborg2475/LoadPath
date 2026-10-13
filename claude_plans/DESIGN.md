# LoadPath: design brief and host contract

**Status: DRAFT 2026-10-10, for owner review.** Decisions marked *decided* were made by the
owner in the kickoff conversation; everything else is a recommendation until ruled. Rulings go
in §10 with their date. Open items go to [CURRENT_STATE.md](CURRENT_STATE.md).

## 1. What it is

A host-neutral C++20 library that answers two questions about a structure of rigid pieces held
together by joints: **does it stand**, and if not, **what lets go, in what order, and what moves
as one body**. Every piece carries real mass. Every joint carries a directional strength envelope
and the force on it is classified against the joint's own plane: compression, shear, tension,
bending. Load redistributes as joints fail, so removing enough support brings a structure down
under its own weight rather than crumbling in place.

It is the structural solver of `DestructionGame` rebuilt standalone. That project spent three
months learning what the model has to be; this one builds that model without the engine around
it, so a test runs in milliseconds and a design mistake costs an afternoon rather than a week.

*Decided:* 3D from the first line. No 2D assembler, ever. *Decided:* pieces never fail internally
here; a beam snapping mid-span is the host's physics engine's job. *Decided:* the host is Unreal
with Chaos, eventually; the library must drop into an Unreal module without rewriting.

## 2. The host contract

The library is a pure function over its own data. The host owns every object that has a shape,
a mesh, or a physics body; the library owns a graph of masses and interfaces and never learns
what they look like.

### 2.1 What the host supplies

- **Bodies.** Mass (kg), centre of mass (m, in a frame the host chooses and keeps), an inertia
  tensor if it has one, a grounded flag. The host gets back a stable `BodyId`.
- **Interfaces.** Two body ids, a unit normal pointing from the first to the second, the contact
  rectangle (centre, two in-plane half-extents, the two in-plane axes), and a `ProfileId`. The
  normal is **the axis of separation, oriented by which body is second**, never the direction
  between centroids (TRAPS). The host gets back a stable `InterfaceId`.
- **Profiles.** The strength and stiffness numbers for a kind of joint, registered once by name.
- **Events.** Remove a body; apply an external load or impulse to a body; change a profile.
- **A call to solve**, on the host's thread of choice.

An optional geometry layer in the same repo can find the interfaces between axis-aligned boxes
for a host that has no better contact information. It is a producer above the engine, like the
test fixtures, and the engine never depends on it.

### 2.2 What the library returns

A **verdict** per solve:

- Every interface that failed in this solve, with the pass it failed in, the **governing axis**
  (crushed, slid, pulled apart, cracked in bending) and whether it cracked (bond lost, friction
  kept) or severed.
- Every body released, grouped into **rigid clusters** that move as one. The host releases each
  cluster as one Chaos body (a geometry-collection cluster that fragments on impact), which is
  the only release mechanism that survived the old project's experiments.
- Per-interface utilisation (demand over capacity, worst axis) for every live joint, for an
  overlay or a readout. A ratio, never a boolean.
- The standing or collapse **certificate** that justified the verdict, for tests and for the
  referee.

### 2.3 What the library never does

Simulate motion. Split a piece. Allocate on behalf of the host. Call back into the host. Keep
state the host cannot snapshot. Convert units (§3).

### 2.4 How it lands in Unreal

The host builds the structure at level load from its pieces, keeps a map from its objects to
`BodyId`s, and keeps every body kinematic. A destruction event becomes `RemoveBody` or
`ApplyImpulse` plus a solve, run off the game thread on an immutable snapshot; the verdict is
applied on the game thread: clusters flip to dynamic, utilisations feed the overlay. When Chaos
fragments a released or struck piece, the host removes it from the graph. The adapter that does
this is a separate module, written when the library is worth hosting; the Unreal-specific
constraints it will impose are enforced here from day one (§7).

## 3. Units

SI base units throughout, no exceptions and no internal conversion boundary.

| Quantity | Unit | Note |
|---|---|---|
| Length | m | A standard brick is 0.215 × 0.1025 × 0.065 |
| Mass | kg | |
| Force | N | Weight = mass × 9.80665 |
| Stress, strength | Pa | Published MPa × 1e6, converted once by the catalogue loader |
| Moment | N·m | |
| Stiffness | N/m, N·m/rad | |

The old project kept strengths in MPa beside forces in Unreal units and compared them through
one named constant; a missing or doubled conversion was out by exactly 100× and tuned thresholds
hid it. Here there is nothing to double. Every variable carries its unit in its name. Whether to
go further and use strong unit types is open (§9).

## 4. The model vocabulary

Material-agnostic by construction. "Brick on mortar" and "stud nailed to a plate" are the same
types with different numbers.

- **Body**: mass, centroid, grounded flag, optional inertia. An identity, not a shape.
- **Interface**: two bodies, a normal, a contact rectangle with its area and section moduli, a
  profile. Carries its own failure state: intact, cracked (pass N), severed (pass N), or gone
  with a removed body. State is irreversible; nothing heals.
- **StrengthProfile**: compressive strength, shear cohesion, tensile strength, friction
  coefficient, shear cap (Mohr-Coulomb with a truncated envelope: only compression buys friction;
  μ = 0 gives independent axes, right for a fastener), plus normal and tangential stiffness for
  the elastic field. Seven numbers. Adding a material is adding a row.
- **Structure**: the graph of the two, with generational handles that stay valid across removal
  (removal tombstones; nothing renumbers, because break stamps cannot be recomputed).
- **Load**: a force or impulse on a body, in the structure frame. Gravity is a load the structure
  applies to itself.
- **Verdict**: §2.2.

## 5. The physics authority

Carried from the old project's ratified runtime plan (`SOLVER_RUNTIME_PLAN.md`, 2026-09-23) and
its two designs, which should be read, not copied:

- **Rigid-block statics is the reference.** Joints carry load in every direction their profile
  allows; corners, arches and cantilevers may hold things up. There is no routing heuristic, no
  support tier, no "load flows downward". The old project built that stack first and then spent
  two months removing it; every one of its heuristics was a patch on the absence of equilibrium.
- **Standing certificate: an elastic rigid-body-spring field** (six degrees of freedom per body,
  corner springs per interface, no tension past the profile's limit, slip capped by friction).
  Every interface at or under capacity in a converged field is a lower-bound proof that the
  structure stands. `RBSM_DESIGN.md` has the formulation and the rulings R1–R31.
- **Collapse certificate: a graph min-cut over candidate rigid motions** (superseded 2026-10-11, §10: slice 4 ships a limit-analysis LP; the graph cut remains the slice-6 scale plan). A cut with negative
  energy is an upper-bound proof that a mechanism exists, and names the cluster that moves.
  `GRAPHCUT_DESIGN.md` has the functional, the candidate dictionary and the determinism rules
  (integer capacities, minimal source set, fixed orderings).
- **Between them, a brittle loop.** Interfaces over capacity in tension or cohesion crack, in the
  top band of ratio only, never all at once; the field re-solves; the cut re-searches. Cracking
  everything at once once ran damage to the far wall of a warehouse. Severing is the cut's job.
- **A referee in the test tier**: a rigid-block limit-analysis LP (HiGHS) that measures the true
  load factor on small fixtures and audits both certificates. It never ships and is not day one.

## 6. Testing strategy

- Unit tests isolate one force on one joint; integration tests put gravity on a structure.
- **Assert on mechanism in unit tests, on outcome in integration tests.** A severed joint is not
  collapse. **Displacement is never a break assertion**; two pieces can sever and stay in place.
- Expected values trace to published strengths, written out in the test as formulas over the
  profile's numbers, so the test fails if a constant is wrong instead of agreeing with it. The
  source of every pinned number lives in [ANCHORS.md](ANCHORS.md) under the test name.
- **Property tests against an independent oracle** for anything topological. Seeded, fully
  deterministic, printing the failing seed. An oracle that mirrors production is worthless.
- **A green-on-arrival test is proven to bite** by a recorded mutation before it is trusted.
- Degenerate inputs fail closed, and the comparison direction matters: `!(x > 0.0)` catches a
  NaN where `x <= 0.0` lets it through (TRAPS).
- The default suite runs in seconds. Anything slower is opt-in behind a prefix. An opt-in tier
  rots unless a milestone names it, so milestones name it.

## 7. Engineering constraints (enforced by the build)

- C++20, no exceptions, no RTTI, `/fp:precise`, warnings as errors. These are what an Unreal
  module imposes; code that needs them will not port.
- No global mutable state. A solve on a snapshot gives the same answer on any thread and at any
  worker count, bit for bit. Budgets are work units, never wall-clock.
- Doubles throughout. Ordered reductions, no float atomics.
- The public API is plain structs, integer handles and spans. Linear algebra stays behind the
  API (§9 decides the library).
- **Anything that may change sits behind an interface with swappable implementations** (owner,
  2026-10-10): the standing authority, the collapse search, the linear solver, the contact
  finder, the structure file reader. Tests run against the interface, so a second
  implementation inherits the suite, and two implementations can be run side by side as each
  other's referee.
- Comments: headers only, one or two sentences (CLAUDE.md).

## 8. Build order (proposed)

1. **Model.** Bodies, interfaces, profiles, handles, validation that fails closed. A CLI that
   loads a structure file and prints it. First red: a two-body structure refuses a degenerate
   normal.
2. **Statics v1.** The elastic field on small structures with a dense solve; per-interface
   utilisation with its governing axis. Anchors: one brick on the ground, a stack, a brick
   cantilevered off another, a leaning stack that must fall.
3. **Cascade.** Top-band cracking, severing, pass stamps, released bodies grouped into clusters.
   The verdict type is complete here.
4. **Collapse certificate.** The drop cut first, then tipping candidates.
5. **Catalogue and fixtures.** Brick, mortar, perpend, C24 timber, nail, screw, bolt, with
   citations; a running-bond wall generator; the structural fuzz with its oracle.
6. **Scale.** Sparse solve, event region, panels. The old warehouse is the benchmark: 5,612
   pieces, 15,042 joints, a decision inside 200 ms.
7. **Referee.** HiGHS LP in the test tier.
8. **Unreal adapter**, as its own module.

Slices 1 to 3 are the engine; 4 onward is the engine being right at scale.

## 9. Decided questions (owner, 2026-10-10)

| # | Question | Decision |
|---|---|---|
| 1 | Project name | `LoadPath`, kept |
| 2 | Metres or millimetres | Metres |
| 3 | Strong unit types or suffixed doubles | Owner: strong types only if they cost no performance. Orchestrator's call under that condition: **suffixed doubles in the core**. A wrapper is free in Release but not in Debug, where the suite runs, and Eigen needs plain scalars in its matrices anyway. A strong-typed façade can be added at the public API later without touching the solver. |
| 4 | Linear algebra | Eigen, vendored under `third_party/`, behind the solver interface |
| 5 | STL in the public API | Yes: `std::span` in, `std::vector` of plain structs out |
| 6 | Structure file format | JSON, hand-written reader, no dependency |
| 7 | License | None for now |
| 8 | Friction flow rule | Non-associative is the engine's answer; tests report the associative bound beside it where a benchmark publishes both |

## 10. Rulings

- *2026-10-10, owner:* 3D from the start. No piece-internal failure. Unreal with Chaos is the
  eventual host. Same TDD and three-agent process as the old project. Comments in headers only,
  one or two sentences. Agents run on Opus 5.5 at high effort unless told otherwise.
- *2026-10-10, owner:* every component that may change (solver above all) is an interface with
  swappable implementations.
- *2026-10-10, owner:* known-answer scenarios from published closed forms and experiments are
  the first test anchors; a fuzz oracle comes later, for topological coverage only. The
  catalogue is `reports/Rigid block benchmark scenarios.md`, distilled into BENCHMARKS.md.
- *2026-10-10, owner:* the §9 answers above. Metres, Eigen, STL in the API, JSON, no license,
  non-associative friction, the name kept, strong unit types only if free (decided against in
  the core, see §9 row 3).
- *2026-10-10, owner:* the orchestrator works the slices without stopping for approval, makes
  the remaining calls itself on its best recommendation, and records each one here with its
  reasoning. Documented real failures (collapses with a known cause) join the benchmarks as
  validation cases.
- *2026-10-10, owner:* **no test, fixture or pinned number is taken from DestructionGame's
  suite.** Every anchor comes from a web-published source with a citation, so its correctness
  is independent of anything either project wrote. The old project remains reference for
  design reasoning only.
- *2026-10-10, orchestrator:* the public API is specified in [API.md](API.md) before each slice
  is built, so the three agents build to one contract. Slice 1's contract is there now.
- *2026-10-11, orchestrator:* four model steps found necessary while making slice 2 green are
  ratified in [API.md](API.md) "Slice 2": a singular active set is a Mechanism only when the
  null-space motion is admissible against the open and slipping corners; an open corner whose
  gap is indeterminate in a neutral solve is not re-closed; a slipping corner carries the
  consistent transverse tangent of its held friction (`kt · capacity / |trial S|`), which adds
  no force at convergence but stops the held direction over-correcting; and the factorisation
  is a right-looking LDLT pivoting on the Schur diagonal, because `Eigen::LDLT` picks pivots
  before the Schur update and cannot reveal rank. The 3DEC wedge (B-WDG-01) moves to slice 4:
  its published threshold is a kinematic answer the elastic field does not reproduce.
- *2026-10-11, orchestrator, after the slice-2a review:* verdicts are **local**: the authority
  solves each connected component of free bodies on its own, with rotational unknowns scaled by
  the component's characteristic length, so a body's state never depends on bodies it has no
  interface path to. Restored open corners re-enter the active set slipping on a dry joint,
  status-only cycle detection is removed (the cap is the only early exit), readings on a
  Mechanism are the range-space solution of the last active set, and options are validated.
  All in [API.md](API.md) "Slice 2".
- *2026-10-11, orchestrator, after the third slice-2a red round:* **the elastic standing
  authority stops guessing.** Three rounds of heuristics on its mechanism path (restore
  re-closing stuck, then slipping; seeding from the singular solve, then from neighbours; a
  consistency pass; held-force re-derivation) each fixed one fixture and regressed another, and
  a non-associative frictional active set has no convergence theory to lean on. So RBSM's
  contract is narrowed to what it can certify: `Stands` and `Overstressed` from a converged
  full-rank active set, `Mechanism` only when the singular active set is admissible with no
  restoration needed, and `Undecided` (fail closed, nothing released) whenever a restored
  corner would need a value the last full-rank solve does not supply or the cap is reached.
  The kinematic collapse certificate (DESIGN §8 slice 4: limit-analysis LP with the referee
  solver, later the graph cut) is the authority for Undecided and for mechanism shapes, and is
  pulled forward ahead of the remaining slice-2 anchors because the arch, pile and wedge rows
  are kinematic answers anyway. Tests that pin a Stands the elastic field cannot yet certify
  are kept red under the `Deferred.` prefix, excluded from the default run, and listed in
  CURRENT_STATE.
- *2026-10-11, orchestrator, after the third slice-2a review:* **RBSM's Mechanism is a verdict,
  not a certificate.** An independent static-admissibility oracle (NNLS over 16-generator
  friction cones) found the elastic path reporting Mechanism on the dry three-brick bridge at
  μ 5, λ = (2.5, 0.875), where an admissible field exists; under non-associative friction the
  lower-bound theorem does not settle which answer is physical. So the slice-4 kinematic
  certificate referees **both** Undecided and Mechanism components, and a Mechanism remains a
  release for the host until then. At realistic friction (μ 0.4 to 1.0) the oracle found no
  violation in 3,000 random stacks and bridges; the cost of the narrowed contract is 619 of
  those 3,000 reading Undecided where collapse is provable, all fail closed.
- *2026-10-11, orchestrator, slice 4a:* the collapse certificate is a **limit-analysis LP**
  (API.md "Slice 4"), not the graph min-cut §5 sketched; the cut stays the slice-6 plan for
  scale. Rulings absorbed into the contract: proportional loading; inscribed inner strength
  region `|S| ≤ c + μN` for `N ≥ −min(ft, c/μ) A/4` against the circumscribed hull (a hull on
  both sides read a bonded Stands at 2.4 × capacity); the homogeneous self-dual embedding;
  mechanism selection at `t* = 0` by greatest external work per unit L1 motion, with a failed
  selection reading Undecided rather than handing the host the analytic centre; lists collect
  every collapsing component; static admissibility is the host's standing criterion and slice 4b
  referees slice 2's Mechanism verdicts against it.
- *2026-10-11, orchestrator, slice 4a closing:* **listing threshold 1e-3.** A body is part of a
  reported mechanism when the largest of its six scaled components exceeds 1e-3 of the
  component's largest, and an interface hinges only when a corner exceeds the same ratio and
  one of its bodies is listed. The earlier 1e-6 listed interior-point residue (a stable
  bystander lifting at 1.6e-6 relative, a weakly bonded bystander's joint read hinging) and let
  the two lists disagree. A genuinely slow participant hidden by 1e-3 would be a translating
  piece within a thousandth of the main body's length of its pivot; the cost is one step of
  delay, which is conservative. The elastic authority still lists at 1e-6 from a direct null
  space (roundoff-level residue); slice 4b agrees one listing definition for both authorities
  before counting disagreements.
- *2026-10-11, orchestrator, slice 4b opening:* **one release set, the certificate's.** The
  combined authority (API.md "Slice 4b") releases only what the certificate lists at 1e-3 on
  a Collapses component; the elastic authority's Mechanism lists stay informational and are
  never compared body by body, so "disagreement" means verdicts, not lists: an elastic
  Mechanism the certificate answers with Stands. That is the shared listing definition 4a
  asked for, by making one list authoritative rather than reconciling two thresholds.
  **Tension-only profiles get no cohesion floor:** a joint with `ft > 0`, `c = 0`, `μ > 0` has
  `T = 0` on the inscribed side and reads Undecided at best; the combined authority then
  reports that component Undecided, which fails closed, and a host that wants such a joint to
  certify gives it the cohesion the physical bond has. **Severity across components** is
  answered by per-component reporting: `Worst` orders InvalidInput > Undecided > Collapses >
  Overstressed > Stands, and the host reads the component list, never `Worst` alone, to decide
  what to release. **The shear cap enters the LP as facet rows** on the corner shear (apothem
  `cap · A/4 · cos(π/G)` inscribed, `cap · A/4` circumscribed), never as a bound on `Σα`.
  **The Orduña pile is pinned on a reconstructed bond** (five 0.2 × 0.4 blocks per course in
  two mirrored tilings, six courses, bed joints only): the paper's figure was not retrievable,
  so the anchor asserts only the published bracket and is marked reconstructed in ANCHORS.
- *2026-10-11, orchestrator, after the first slice-4b review:* **a component's elastic verdict
  is its own solve.** The combined authority solves each component's sub-structure with the
  elastic authority and takes that State; the whole-structure solve serves only the readings.
  Inferring per-component states from the one global state relabelled an Overstressed brick
  Undecided beside an unrelated pass-capped brick (and released it) and read NaN corner forces
  as Stands through a private copy of the overstress rule; no copy of that rule exists now.
  **A shear cap of 0 is a cap**, not "no cap": `+inf` is the only unset value, and the
  selection LP keeps only the normal row on a cap-0 corner. **The wedge band is a known
  unsafe gap:** the associative certificate switches at 31.07° where the physical wedge
  switches at 33.36°, and between them the combined authority reads Stands while the elastic
  reads Undecided, so no disagreement is counted and the host gets no signal; the physical pin
  is a `Deferred.Slice7.*` case for the non-associative referee. **Discrete arches stand at or
  below the continuous least thickness** when voussoirs carry exact sector weights and
  centroids (the joints are a subset of the continuous sections); B-ARC-09's "only raises" is a
  DDA artefact, and the anchor pins Milankovitch exactly with joints at ±54.484°.
- *2026-10-11, orchestrator, slice 3 opening:* **the cascade is a loop over the combined
  authority** (API.md "Slice 3"), one judge per pass, release before crack within a pass by
  construction (a component is either Collapses or Overstressed, never both). Rulings carried
  from the old project's R27–R30 and TRAPS: cracks land event by event in the top band of
  crack ratio (`CrackBandFraction`, default 0.5, relative edge 1e-9, NaN and +inf always
  crack, `R_max` over finite eligible ratios only); crushing has no crack action; a crack
  removes tension and cohesion and keeps compression, friction, the cap and the stiffnesses,
  through one `Structure::EffectiveProfile` both authorities read; severing is the
  certificate's job, done only at a release, on every live interface crossing the released
  cluster's boundary; clusters are the certificate's released bodies grouped over non-hinging
  interfaces, so each is a certified rigid motion and never a reachability weld. New here: a
  severed joint is **not live** (it carries nothing), `RemoveBody` keeps a severed joint's
  stamp, pass numbers continue from `HighestPass` across calls, and the per-axis utilisations
  move into `InterfaceReading` so the crack rule reads the elastic authority's own ratios
  rather than a second copy of the strength rule. The run mutates the structure it is given,
  which is the host's snapshot (DESIGN §2.4), and the verdict lists every change.
- *2026-10-11, orchestrator, after the first slice-3 review:* **the pass counter is the
  structure's, not the stamps'.** A counted pass that writes no stamp (a free body released, a
  body whose last joints a neighbour severed) still advances `HighestPass` through `MarkPass`,
  and the file carries `highest_pass`, because the writer never sees a sever whose other side
  was removed and a reloaded structure was restarting at pass 1. `Sever` refuses a pass below
  the crack stamp so the file reader and the structure agree. On `PassCapReached` the cascade
  judges once more so the readings describe the structure that remains. A crack ratio with a
  NaN on either axis is NaN and cracks (the order-dependent max was the slice-2 trap again).
- *2026-10-11, orchestrator, slice 5 opening:* **the catalogue is a JSON file, converted once
  by its loader, every row cited and labelled characteristic or mean** (API.md "Slice 5");
  a value with no source is omitted rather than invented, so the timber bearing row ships
  without friction unless a cited coefficient is found. **Fasteners are rows of per-fastener
  capacities turned into an area profile with μ = 0** (independent axes, DESIGN §4), so a
  fastened joint's capacity is count × the published value whatever contact area the host
  gives it. **The geometry layer and the wall generator are producers above the engine**
  (DESIGN §2.1) and nothing in the solvers includes them. **The structural fuzz checks the
  cascade's invariants, not its answers**: every released cluster is a subset of the
  certificate's own release list on the pre-pass structure, stamps match records, passes are
  contiguous, the remainder re-judges to the same verdicts, the file round-trips the end state.
- *2026-10-11, orchestrator, slice 5 close (after the review):* five data rulings, each a
  correction the review traced to the repo's own research notes. **`TensileStrengthPa` is
  direct tension, uniform over the joint.** A code flexural strength (EN 1996-1-1 f_xk) is a
  bending strength through Z = b t² / 6; the certificate's rigid-plastic hinge gives
  M = f_t A t / 2 = 3 f_t Z, so a flexural value enters the catalogue as f_xk / 3, which makes
  the bonded wall's overturning factor the code's exactly. Measured direct values (Gooch) enter
  as they are. **A perpend row is the perpend joint's own bond**: EN 1996-1-1 eq. 3.6 is the
  bed shear of a wall with unfilled perpends and f_xk2 a wall flexural strength through the
  units, so neither describes the joint; the filled perpend carries the bed row's bond and an
  unfilled perpend is a dry profile. **Fastener capacities are per shear plane as EN 1995-1-1
  §8.2.3(1) defines them**, so the M16 bolt is 12.9 kN per plane (the row had halved it). **A
  "mean" row needs a cited mean**: Blass & Colling's 1.074 is an EN 14358 characteristic ratio
  on yield loads with the 1.05/1.15 factors removed, so the two scaled fastener rows are gone
  until a mean is found. **A withdrawal value belongs to the screw it was computed for**: the
  8 × 60 row carries the A1:2008 §8.7.2 value with the example's own inputs (5.24 kN), not the
  2.5 mm screw's 1.74 kN. Two stiffness rulings with them: **a fastener row carries EN 1995-1-1
  Table 7.1's slip modulus K_ser per shear plane per fastener** (×2 steel-to-timber, §7.1(3)),
  and `FastenerProfile` gives the joint `Count · K_ser / Area` tangential stiffness and the
  bearing member's normal stiffness, so a fastened joint is usable by the elastic authority
  and the area cancels in stiffness as it does in capacity; and **a rigid-block masonry spring
  is the masonry modulus over a course** (E / (h_unit + h_joint) = 119 GPa/m), not over the
  mortar joint alone, because the rigid unit's share of the deformation has to live somewhere.
  Process: the `Slow.*` Release tier runs at every slice close, by the orchestrator, before the
  review (TRAPS: name the milestone that runs an opt-in tier).
- *2026-10-11, orchestrator, slice 2c opening:* **a dry corner carrying nothing is open.** The
  slice-5 sweep showed the elastic authority Undecided on every dry running-bond wall with
  perpends between full bricks (1 × 3 and 5 × 1 up): a perpend at N ≈ 1e-18 stays closed
  because a corner opens only on N < 0 strictly, its slip capacity μN and its roundoff shear
  are the same size, and the stuck/slipping test flips every iteration; the 2 × 1 wall reads
  Overstressed at rest for the same reason (roundoff shear over zero capacity, +inf). Ruling: a
  corner on a joint with zero tension and zero cohesion whose trial normal force lies within a
  **force-noise floor** of zero is Open, carries nothing and reads N 0, S 0, utilisations 0; the
  floor is one named constant relative to the component's `ForceScaleN`, of the order of the
  held-force tolerance (1e-12), so it is far below any real load (a perpend of the 1 × 3 wall under a 0.2 W push on the end
  brick carries 0.2 to 1.2 N; under a uniform λ 0.2 the three bricks move together and the
  perpends carry nothing, which the floor may open) and far above the roundoff it exists to absorb; the stuck/slipping
  comparison gets the same absolute slack so a capacity and a shear both at noise level never
  flip a corner. A genuinely loaded corner keeps the slice-2 rule unchanged, so the incline,
  bridge and wall anchors must not move. Bonded corners are untouched: cohesion carries at N 0.
- *2026-10-11, orchestrator, after the slice-2c dev round:* **the floor opens; it does not slacken
  the slip test.** As built, `DryNoiseFloorRelative = 1e-12` of the component's force scale
  opens a dry corner whose trial `|N|` is at or below it, and such a corner re-closes only when
  its full-rank trial normal rises above the floor (a zero gap no longer re-closes it; without
  that the flip moves to the re-close step). The slip-test slack the opening ruling asked for
  drove no test once the opening rule was in and was withdrawn under the TDD gate; the 1 × 3,
  2 × 3, 3 × 4, 4 × 5 and 10 × 8 dry walls all settle in two iterations at rest (laterally
  loaded dry walls can still read Undecided: the review's grid, CURRENT_STATE). Two edges recorded,
  not ruled: a NaN trial normal stays closed (`abs(N) <= floor` is false for NaN), because
  opening is the direction that hides an overflow and the overflow test reads +inf through the
  closed corner; and an overflowing component makes the force scale and the floor +inf, so
  every dry corner in it would open if any finite load remained (the review showed this fails
  OPEN from one finite input of 2e154: ruled below).
- *2026-10-11, orchestrator, after the slice-2c review:* **a non-finite force scale fails
  closed, and the floor has hysteresis.** A single load or weight component above about
  1.3e154 overflows the component's force scale through `sqrt(Dot)`, the floor becomes +inf,
  every dry corner opens and the overflowed load is classed neutral: a false Stands from one
  finite input. Ruled: a component whose force scale is not finite never Stands (Undecided or
  the overflow reading, never an open corner). A dry corner opens at |N| ≤ floor and re-closes
  only above 2 × floor, because with one threshold a corner just under it reads a larger gap
  once open, re-closes, and chatters to Undecided under a real load of 1e-12 to 1e-8 W. The
  review also measured that laterally loaded dry walls stay Undecided at any floor (147 grid
  points, perpend corners under real load slipping): that is a separate non-convergence of the
  free active set, recorded as the next solver item, not a floor question. The test round then
  showed a tensionless joint WITH cohesion chattering the same way (noise-level negative N opens
  its corners through the plain `N < 0` rule, then the wall reads Undecided): ruled, every
  tensionless corner opens only at N < −floor, and a dry corner is open inside ±floor as well
  because it carries nothing either way; a cohesive tensionless corner inside the band stays
  closed and carries its cohesion.
- *2026-10-11, orchestrator, after the slice-2c follow-up review:* two corrections to the
  round before. **A non-finite force scale is Undecided**, not a frozen active set: frozen, a
  pure in-plane overflow (2e154 along the bed plane) read Stands with shear utilisation 1e153,
  because a dry corner's slip lives in the active set and the overstress rule never flags dry
  shear. **A closed tensionless corner inside the noise band reads zero tension**: with the
  opening threshold at −floor, a corner with N in [−floor, 0) stayed closed and its noise
  tension over a zero tensile capacity read +inf, so every wall on the lime-mortar row
  (cohesion, no tension) read Overstressed at rest, the very class the slice was opened to
  remove, and one that would feed the cascade's crack band.
- *2026-10-11, orchestrator, slice 6 opening:* **measure before designing.** Every cost item
  logged so far (the combined authority's per-component copies, the cascade's quadratic scans,
  the dense per-component LDLT, the certificate's interior point) is a guess until timed on
  structures ten and a hundred times the fixtures, so slice 6 opens with 6a (API.md): a
  warehouse generator built only from the slice-5 pieces (four separate running-bond walls, a
  post-and-beam frame on the C24 bearing row, planks on fastener rows, one ground), and
  `Measure.Scale.*` rows that time the elastic authority, the certificate, the combined judge and
  one cascade pass at five sizes from the 85-brick wall to a 3,700-piece warehouse, asserting
  only what is already promised (Stands at rest, determinism, a hang ceiling). The table goes
  to `claude_plans/SCALE.md` with the machine named; 6b (sparse factorisation behind the same
  `IStandingAuthority`, an event region that re-solves only what a change touched, panels that
  keep an uncracked bonded run as one body) is contracted against those numbers, not before
  them. The generator is a producer above the engine like the wall (DESIGN §2.1): no solver
  code includes it. Slice 8, the Unreal adapter, is on hold by owner directive (2026-10-11).
- *2026-10-11, orchestrator, slice 6a first finding:* **a floating component falls rigid.** The
  measurement's course removal showed the cascade cutting a floating wall into one cluster per
  brick with every joint severed (dry joints at `Tension inf`, bonded joints at ratios of
  0.002 to 0.37), because at `t* = 0` every motion is a mechanism and the certificate's
  selection picked one with relative motion at every joint; on the 10 × 8 wall the selected
  motion did not even cover the top bricks, which left for a second pass. Ruled: the mechanism
  selection prefers the motion with the least joint movement (an L1 measure of opening and
  sliding over the joints, at unit external work), so a component with no path to the ground
  selects the rigid fall, lists no hinging joint, and the cascade releases it as one cluster
  with nothing severed. A severed joint must also never carry a reading of `inf` as its
  reason: a joint released by a rigid fall has no failure. The Release measurement read this
  same case Undecided with zero passes where Debug read Collapses: a build-dependent answer
  from the interior point at `t* ≈ 1e-12` is the second thing the dev round must explain.
- *2026-10-11, orchestrator, after the slice-6a review:* corrections to the ruling above, which
  described a selection the round did not build. **As built, the rigid fall is a check before
  any solve**: a component with no live joint to a grounded body falls rigid, Collapses at
  load factor 0/0 with no hinge. The review showed that is exact only under gravity alone
  (every body accelerates at g, no joint carries anything) and wrong with a host load (a dry
  pair in free fall with +100 N on the upper brick read one cluster where the upper brick lifts
  off alone), and that a rounding residue in the net force could flip a squeezed floating pair
  from Stands to Collapses. Ruled: **the rigid fall applies only to an ungrounded component
  carrying no `BodyLoad`**; a loaded ungrounded component, and any component hanging from or
  wedged under a grounded body through dry contacts, takes the general path, where the
  least-joint-movement selection of the ruling above is still owed (the hanging bonded wall
  reads Undecided, the dry pair under a ceiling splits): pinned under `Deferred.Slice6.*`. The
  selected fall motion is `(0, 0, −1)` per body with zero rotation, the header's normalisation.
  **There was no Debug/Release difference**: the measured Undecided was the bonded 10 × 8 wall
  and the Debug Collapses the dry one; rebuilt at the pre-fix commit, both builds agree on
  both. **The measurement's cost reading changes too**: the certificate is never called when
  the elastic field decides (0 of 5 components on the shed and the half warehouse); the judge
  costs twice the elastic solve because it solves the whole structure and then every component
  again; and the hand-rolled rank-1 `Factorise` is 10–17× slower than Eigen's blocked dense
  factorisation at the same size (20.4 s against 1.2 s at 3,660 DoF), so 6b's first stage is
  the factorisation itself, before any sparsity. Measurement rows that cannot finish leave the
  `Slow.` tier for `Measure.*`, excluded from every tier and run one per process.
- *2026-10-12, orchestrator, slice 6b opening:* **the factorisation first, sparsity second,
  the event region third, panels only if the rows still demand them** (API.md "6b"). The
  measurement says the elastic solve is the cost and the hand-rolled rank-1 factor is 10–17×
  slower than a blocked dense one at the same size, so stage 1a swaps `Factorise` for Eigen's
  pivoted dense `LDLT` behind the same rank test and keeps the old factor in the test tier as
  the referee for one slice; every anchor must read the same to 1e-9. Stage 1b puts a sparse
  `SimplicialLDLT` above a DoF limit with the dense path kept below it, so no anchor changes
  path, verifies candidate null vectors before calling a mechanism and reads Undecided when it
  cannot, and removes the combined authority's whole-structure-then-every-component double
  solve and its per-component copies through per-component results and a component filter on
  the certificate. Stage 2 re-solves only the components a change touched, with a fuzz guard
  that a reused verdict equals a fresh one. Each stage ends with the `Measure.Scale.*` rows
  appended to SCALE.md. The least-joint-movement selection owed to the hanging cases
  (`Deferred.Slice6.*`) is not in 6b; it is the certificate's next item.
- *2026-10-12, orchestrator, slice 6b stage 1a, two rulings:* **the blocked factor is our own,
  not Eigen's.** Eigen's dense `LDLT` is left-looking and picks each pivot before the Schur
  update with no cutoff, so it does not reveal rank (3,078 of 34,493 factorisations disagreed
  with the reference on rank, positive-definite cases among them); the stage-1a factor is a
  blocked right-looking pivoted LDLᵀ in the style of LAPACK `dpstrf` (64-column panels, GEMV
  within a panel, one GEMM per trailing update, the largest signed updated diagonal as the
  pivot, the same `Pivot > tol × largest` cutoff, the same `Factorisation` shape), 7× faster at
  1,968 DoF (0.45 s against 3.1 s) and 24 s at 7,776 DoF where the reference did not finish in
  25 minutes. **What "agrees with the reference" means for a factor that rounds differently.**
  The same eliminations summed in another order move readings by 1e-9 to 7e-7 relative through
  a badly conditioned K, and 2 of 6,000 seeded draws sit so close to the pivot cutoff that
  Mechanism and Undecided trade places. Ruled: `Stands` and `Overstressed`, the host-facing
  certified answers, must agree exactly; `Mechanism` and `Undecided` may trade places on at
  most 0.1 % of draws (both say "the field cannot stand this" and both are refereed by the
  certificate); active-set iterations may differ by at most 2; forces compare at 1e-7 relative
  plus 1e-9 of the structure's weight absolute, utilisations at 1e-7 relative plus 1e-9
  absolute, so a 1e-16 utilisation is zero to the test. The 10×-looser-tolerance mutation must
  still redden the property under these bounds.
- *2026-10-12, orchestrator, stage 1a, amendment:* **the reading and iteration bounds apply
  only to certified states.** A Mechanism or Undecided field never settled, so its readings
  are arbitrary (a 6,557 W force gap between two factors on one Undecided draw) and its
  iteration count is a path through a flipping active set; the API already calls those
  readings informative only. So the property compares readings (1e-7 relative plus the
  absolute floors) and iterations (within 2) only when both kinds read Stands or
  Overstressed, where the measured worst gaps are 1.7e-14 W and 3.2e-11; on Mechanism and
  Undecided draws it requires the state class and the body lists when the states match, and
  the trade budget of 0.1 %. Second amendment, same day: an Undecided pair compares nothing
  beyond its class (its open flags are the snapshot of a flipping active set: draw 5279 reads
  Undecided after 17 on one factor and after the 50 cap on the other, with 7 flags apart);
  body lists are compared when both read Mechanism, a verdict, or a certified state; open
  flags only on certified states. The 10×-looser-tolerance mutation is caught by the trade
  budget (32 trades against 6), not by the certified states, which it leaves untouched.
- *2026-10-12, orchestrator, stage 1b opening:* **stage 1b runs as three rounds, each with
  its own red step, in this order: (i) the sparse factor above `DenseDofLimit`, (ii)
  per-component results and no per-component copies, (iii) the certificate's sparse normal
  equations.** The three touch different files and the API "6b" text already separates
  them; one brief covering all three is the hour-long round the slice-pace note warns
  against. Round (i) measured against: the half warehouse's elastic solve at 5.9 s after the
  blocked factor, the 32 × 40 wall at 24 s, both n³ in a dense factor of a matrix whose rows
  carry a dozen non-zero blocks. The sparse path's agreement property reuses the stage-1a
  comparison rulings unchanged (certified states exact, Mechanism/Undecided trades budgeted,
  readings only on certified states); the limit's default of 600 DoF is chosen so that no
  default-suite anchor crosses it, and a test pins that. Round (ii) is measured against the
  judge still costing twice the elastic solve on the half warehouse (11.8 s against 5.9 s).
  Round (iii) against the certificate column, unchanged by 1a at 572 s, which is what every
  release pass pays per Undecided or Mechanism component.
- *2026-10-12, orchestrator, stage 1b-i ruling:* **the sparse factor is the full-rank fast
  path; any sign of rank deficiency hands that iteration to the dense blocked factor.**
  `Eigen::SimplicialLDLT` does no pivoting, so once a pivot of D is tiny the rows below it are
  divided by it and the factor past that point is noise; verifying null candidates taken from
  such a factor (the API "6b" text) is verifying noise. So above `DenseDofLimit` the component
  is assembled sparse and factorised with AMD ordering; if the factor reports success and
  every pivot of D is finite and above `PivotToleranceRelative × max |D|`, the solve proceeds
  from it and the readings are the dense path's to rounding; otherwise the iteration is redone
  with `FactoriseBlocked`, whose rank test and null basis stage 1a verified. A mechanism
  component above the limit therefore pays the dense factor once per rank-deficient
  iteration, which the certificate it is then handed to dwarfs; the common case, a standing
  wall, never does. The "unverifiable candidate reads Undecided" test of the brief has no
  path to reach and is dropped; the detection rule itself is pinned by the mechanism wall
  above the limit (`Statics.Sparse.AMechanismAboveTheLimitReadsTheSameAsBelowIt`) and by a
  mutation that disables the fallback.
- *2026-10-12, orchestrator, stage 1b-i amendment:* **an unpivoted pivot test is not a rank
  test; the sparse path is accepted on an estimate of the smallest eigenvalue, not on its
  pivots.** The dev round measured it: on 1,000 forced-sparse stacks and bridges, five
  18-DoF bridges whose pivoted dense factor reads rank 17 passed every unpivoted pivot of
  `SimplicialLDLT` at 1e-10 relative (the last pivot is bounded by λ_min / v_n², so a null
  vector with little weight on the last-eliminated DoF hides behind a moderate pivot), and
  the two paths then traded Mechanism and Undecided five times in 1,000 where the two dense
  factors trade zero. So the acceptance rule is: factor succeeds, every pivot finite, and the
  Rayleigh quotient after a fixed number of inverse iterations with the sparse factor from a
  deterministic start vector exceeds a margin times `PivotToleranceRelative × max |D|`; any
  failure hands the iteration to the dense blocked factor. Inverse iteration converges fastest
  exactly when the matrix is near singular, which is the case that must be caught, and a
  sound wall's estimate is far above the margin. The iteration count and the margin are set
  by measurement in the round (trades on the forced-sparse draws against the cost on the
  32 × 40 wall), not guessed, and recorded here. Also ratified from the round: a non-finite
  sparse stiffness on the first iteration reads InvalidInput without allocating the dense
  matrix to confirm it.
- *2026-10-12, orchestrator, stage 1b-i, the measured constants:* **five inverse iterations,
  margin 1.** The dev round swept 5, 10 and 20 iterations against margins 1, 10 and 100 on
  the forced-sparse draws: every pair gave 0 trades on the 24 walls and 1 on the 1,000 stacks
  and bridges (draw 443, a bridge reading Mechanism after 30 dense against Undecided after 22
  sparse; a probe running the dense factor on every sparse acceptance found zero rank misses
  after the change, six before, so the survivor is rounding drift on a long active-set path,
  inside the budget of 1 per 1,000), and the 32 × 40 wall stayed sparse at 0.17–0.20 s with
  an estimated λ_min / max |D| of 3.0e-6 against a bar of 1e-10. Nothing in the data
  separates the pairs, so the smallest is taken; the constants are file-scope in the solver,
  not options, because no caller has a reason to move them. A zero or non-finite norm during
  the iteration rejects the factor. Skipping the estimate is caught only by the bridges (5
  trades against 1), a thin margin to note for the review.
- *2026-10-12, orchestrator, stage 1b-ii opening:* **the judge solves the elastic field once;
  the certificate takes a component filter; a bad filter is InvalidInput.** `StandingResult`
  carries the per-component answer the solver already computes (bodies, state, iterations,
  reading slots), so `CombinedAuthority::Judge` builds every report from one `Solve` and
  copies the structure zero times; `ICollapseCertificate::Decide` gains a fourth argument,
  the bodies whose components it decides (empty: all), consulted only for Mechanism and
  Undecided components; `Structure::Components()` is the cached partition, invalidated by
  every mutating call. A filter naming anything but a live free body (the ground, an unknown
  or invalid id, a free body listed with the ground) reads InvalidInput: reading it as the
  whole structure or dropping the entry would widen the question asked without the caller
  knowing (the test-expert's choice, ratified). The equivalence guard
  `Authority.Judge.MatchesSolvingEachComponentAlone` (40 seeded multi-part structures, 161
  components) is green on arrival by construction and pins that the no-copy judge reads what
  the copying judge read; its mutation record is a judge that reports the whole structure's
  state for every component.
- *2026-10-12, orchestrator, stage 2 opening:* **cleanliness is derived from revisions, not
  from a dirty set the judge would have to clear.** `Structure` counts a revision per
  successful mutation and stamps the bodies a mutation touches (both bodies of an interface
  added, cracked or severed; the surviving neighbours of a removed body); a judge records the
  revision it saw. Given a previous verdict, a component is clean when a previous component
  has the same body list, no body of it was touched after that revision, and its loads are
  unchanged; clean components are copied forward with `bReused` set and their readings copied
  by interface id, dirty ones are solved through one `Solve` call carrying a component filter
  (the same fourth argument `Decide` took in 1b-ii), and no call at all is made when nothing
  is dirty. A previous verdict that cannot be matched reuses nothing and is never an error,
  so a caller can always pass what it has. The judge stays const and stateless: the only
  state is the structure's own counters and whatever verdict the caller keeps, which is what
  the cascade does between passes. The guard is equality with a fresh judge on a copy after
  every pass of the seeded fuzz. The 1b-i rows set the target: one post removed from the
  warehouse, 1.70 s today, under 200 ms when only the frame is touched.
- *2026-10-12, orchestrator, stage 1b-iii rulings from the dev round:* **the sparse normal
  matrix carries the load-factor coupling on the demand's support; regularisation is a
  retry, as on the dense path.** The API text named only the site blocks, but the dense
  `A D Aᵀ` also holds `Coupling · Demand · Demandᵀ`, dense in principle, and without it the
  site-block part alone is singular (smallest eigenvalue 1e-17 of the largest on the shed) so
  a Sherman–Morrison correction took the interior point from 35 to 115–126 iterations.
  `Demand` has one non-zero per body, so the rank-one term is assembled as triplets over
  that support (about 119² entries on a shed wall), with the dense path's per-entry
  arithmetic, and iteration parity returned (35 and 35). If a load ever fills all six rows
  per body, that block is dense and the ruling is revisited. The sparse factor is tried
  without regularisation first and retried with `RegularisationRelative × LargestDiagonal`
  only on failure, exactly as the dense `HasPositivePivots` retry; both triangles are stored.
  Measured: the shed's certificate 6.65 s sparse against 7.7–8.2 s dense, same 35 iterations;
  its components are 684 and 714 rows, barely above the limit, so the factor was not the
  shed's cost; the half-warehouse and 32 × 40 rows (589 s and 754 s dense) are the
  measurement that matters: 152 s and 114 s (SCALE.md). What remains per iteration after the factor
  (the per-component dense LLT in `SelectionSystem`, the cap-site loops, assembly) is to be
  profiled before anything else is changed.
- *2026-10-12, orchestrator, stage 2 amendment:* **options are part of cleanliness.** A
  previous verdict carries the `AuthorityOptions` it was judged under, and a previous judged
  under any other options (gravity, tolerances, limits, certificate options, every field) is
  unmatched as a whole: nothing is reused. The dev round flagged that a different gravity
  would otherwise be reused as if nothing had changed; a wrong reuse is a wrong verdict with
  `bReused` set, which no caller can be expected to check. Also ratified from the round: a
  removed body stamps only the neighbours it still shared a live interface with (a severed
  joint already split the partition); copying a ground-contact reading needs only the free
  end untouched (every Sever stamps the ground, which would otherwise block every ground
  reading on the structure); an unknown id's stamp reads as always dirty; load matching is
  exact tuple equality in order; a reading that can be neither solved nor copied, or a load on
  a dead body, throws the reuse away and judges everything.
- *2026-10-12, orchestrator, stage 3 opening (the certificate at scale), from the profile:*
  **exact algebra first, the stall rule second, a verdict-only option last.** The profile of
  commit 0c52f6f at rest: the shed's certificate (7.1 s) is 66 % cap-site work (an 18 × 18
  build and LLT per corner per iteration); the half warehouse's (158 s) is 47 % cap sites,
  29 % factor and one circumscribed solve that converged to 1.05e-9 against 1e-9 at iteration
  62, then diverged to 6.75e-6 and ran to the 200 cap with 139 regularisation retries (59 s
  for nothing); the 32 × 40 wall's (108 s) is 62 % factor, because the coupling block on the
  demand's support is a dense 1,296 × 1,296 block (1.68M of 1.99M non-zeros). Iteration
  counts do not grow with size (28 on 7,776 rows). So stage 3, in order: (3a) a structured
  cap-site factor: `Mww` is a diagonal plus rank ≤ 4 (the facet rows see only the in-plane
  part of each direction), so Woodbury replaces the 18 × 18 work with 18 × 4, exact, no answer
  changes, about 85 s of the 273 s; (3b) a stall rule: when the residuals stop improving the
  solve keeps its best iterate and stops, with the verdict rules unchanged (the component read
  Stands anyway because the inscribed factor was at least 1), about 59 s; (3c) the coupling
  out of the factor: factor the site part with a tiny regularisation and apply the rank-one
  term by Sherman–Morrison with two or three steps of iterative refinement against the true
  matrix (the plain Sherman–Morrison blow-up from 35 to 115 iterations is what the
  refinement must prevent, measured before it is kept), about 68 s on the wall; (3d) only if
  still needed, a verdict-only option that skips the circumscribed solve when the inscribed
  factor is at least 1, which changes `LoadFactorUpper`'s meaning on Stands and so stays an
  option the judge sets, never the default. Eight cone generators are rejected: 5× on cap
  sites for a 2 % to 8 % inscribed/circumscribed gap. Panels stay contingent on the stage-3
  rows.
- *2026-10-12, orchestrator, timing tests:* **a Slow-tier timing assertion compares two paths
  in the same process, or carries at least three times headroom; never a thin absolute
  ceiling.** The shed certificate's 8.0 s ceiling (6.7 s quiet) read 8.56 s, 9.9 s and 10.4 s
  on three loaded runs with no code change in the certificate. It becomes "sparse at most the
  dense time, both measured back to back" (the regression it exists to catch is the sparse
  path falling to dense speed); the 1.0 s ceiling on the 32 × 40 wall's 0.19 s and the 1.0 s
  on the 16 × 20 wall's 0.56 s stand on headroom.
- *2026-10-12, orchestrator, stage 3a rulings from the test round:* **the structured cap-site
  factor must match the dense block to a backward error of 1e-12 across the full slack range,
  so it cannot be a textbook Woodbury; the dense block stays as the referee for one slice.**
  The test round proved a plain Woodbury (`Dg⁻¹ − Dg⁻¹U(I + C UᵀDg⁻¹U)⁻¹CUᵀDg⁻¹`) fails all
  200 seeded cases at slack 1e-6 against direction weights 1e6 (backward error 3e-5, the
  dense reference's κ reaching 4e12), and a scaled QR form fails 199. So the structured form
  eliminates the group rows first (their block is diagonal), leaving
  `diag(D_facet) + Facets · K′ · Facetsᵀ` with K′ 3 × 3, and factors that by a stable
  rank-3 update (a Cholesky rank update, or a 3 × 3 core formed without inverting the
  diagonal); the tolerances stand (backward 1e-12, forward max(1e-10, 1e-14 κ), Schur 1e-10
  of ‖K3‖). Positivity: the structured factor reports the dense verdict on every finite input
  up to a condition number of about 1e13, including a zero facet slack whose facet still sees
  a direction (positive), and fails closed on a NaN slack, which Eigen's `info()` lets
  through. (Narrowed by the cold review, 2026-10-12: past κ ≈ 1e16 both factors are
  rounding-limited on exactly positive definite matrices and disagree in 4–30 % of cases,
  the structured one failing less often; both fail closed to Undecided, never a false
  Stands.) The dense block stays
  behind `CertificateOptions::CapSiteKind { Dense, Structured }` (default Structured) for one
  slice as the referee, so the Slow timing test is a same-process ratio (structured at most
  0.85 of dense on the shed; the bar moved from 0.8 at the merge, measured 0.68–0.80) rather than the 6.5 s ceiling the brief asked for, which the
  timing-test ruling forbids; both referees (`FactorKind::Reference`, `CapSiteKind::Dense`)
  go at the 6b close.
- *2026-10-12, orchestrator, stage 2 second amendment (from the cold review):* **a verdict
  belongs to one structure's lineage, and the judge validates its options before it matches
  anything.** A copied `Structure` copies the revision and the stamps, so two histories
  forked from one copy number their revisions identically, and a verdict from the fork was
  matched and reused on the original (the review's repro: crack A's bed on the copy, crack
  B's on the original, judge the original with the copy's verdict: A read Collapses where a
  fresh judge reads Stands). So `Structure` carries a lineage token minted at construction
  and by every copy construction and copy assignment (a move keeps it; mutations keep it),
  `AuthorityResult` records it, and a mismatch reuses nothing. And the standing options are
  validated in `Judge` before matching, since `Solve` is skipped when nothing is dirty and
  `PivotToleranceRelative = 0` with a clean verdict returned the copied verdict instead of
  InvalidInput; the conditions are `FindInvalidInput`'s (tolerance in (0, 1) and finite,
  MaxActiveSetIterations at least 1). The review also found the free-end rule for ground
  readings unreachable (every unsolved interface belongs to a clean component), kept as
  documented defence; the live-neighbour stamp redundant with the body-list match (kept, it
  is what the ruling says); and five behaviours unpinned (the first-pass Previous of `Run`,
  the re-judge after the cap, a moved load point, a load on a dead body with nothing dirty,
  a ground contact added to a loose brick), pinned in the fix round. Measured by the review
  on the warehouse: a judge with Previous after one post removed is 317 ms against 558 ms
  fresh (4 of 5 reused), of which the frame's own certificate is about 225 ms.
- *2026-10-12, orchestrator, stage 3a rulings from the dev round (merged 2623138):*
  **the 3 × 3 core is formed as a sum of PSD terms, the facet block is factored dense, and
  the structured path caches `Mww⁻¹Fᵀ` per site.** The core
  `K′ = Σ_J D_J (d_J − m_W)(d_J − m_W)ᵀ + Σ_W (T_W · gs_W / P_W) m_W m_Wᵀ` (m_W the D-weighted
  mean direction of group W, T_W its weight sum, P_W its pivot) is the ruling's subtraction
  form rewritten so nothing cancels; measured backward error 1.45e-16 over 200 cases at κ up
  to 4e12. Eigen's `LLT::rankUpdate` divides by the running diagonal, which is exactly zero
  for a zero facet slack, so the 16 × 16 facet block `diag(D_facet) + Facets K′ Facetsᵀ` takes
  a plain dense LLT instead: the ruling's concern was never inverting a tiny diagonal, and
  this route does not. `Mww⁻¹Fᵀ` is cached per site so `SolveNormal` does one cap solve per
  site instead of two; without the cache the structured path was slower than the dense one
  (solve time 1.9 s to 3.0 s on the shed). The guard's 12 fixtures keep their iteration
  counts; load factors move by at most 3e-13 relative (ec6 4 × 4) from the changed rounding.
  Measured on the shed: 7.09 s dense against 5.34 s structured (0.69–0.75 over four runs),
  far short of the profile's 5–10× on the cap-site share, because the structured factor
  still spends half its time copying constant geometry per iteration and allocating small
  dynamic matrices; the follow-up (dcb9086) cached the geometry per component. The 32 × 40
  wall read 106.8 s dense against 92.4 s structured.
- *2026-10-12, orchestrator, stage 2 fix round (merged):* ratified from the dev
  round: the standing option conditions live in one shared `statics::AreStandingOptionsValid`
  used by both `FindInvalidInput` and `Judge`'s first guard; a non-finite standing gravity
  needs no separate check in `Judge` because the gravities must match and the certificate's
  must be finite; interface stiffness and load checks stay with `Solve`, since a bad
  stiffness is reached only through a touched body; the lineage counter is a process-wide
  atomic starting at 1, copy assignment guarded against self-assignment, and an InvalidInput
  result carries the judged structure's lineage like any other.
- *2026-10-12, orchestrator, stage 3b rulings from the dev round:* **the stall rule stops a
  cone solve; it does not keep an iterate.** Every iterate is tested against the tolerance as
  it arrives, so an iterate that was ever within tolerance ended the solve then; the best
  iterate of a stalled solve is by construction not within tolerance, and the not-converged
  outcome carries no load factor, so a stored best iterate would never be read. The rule
  therefore tracks only the best residual: the window restarts whenever the largest relative
  residual falls below `StallImprovement` times the best at the window's start, and when
  `StallWindow` iterations pass without that the solve returns the iteration cap's outcome.
  The selection solve keeps its own loop and residuals and is untouched. Measured: the
  half warehouse's 595-body wall from 200 iterations in 127 s to 72 in 45 s, the verdict and
  lower bound unchanged; the twelve pinned fixtures and the 64 agreement cases bit-identical
  with the rule on and off. The brief's "returns its best iterate" mutation has no code to
  mutate; the ANCHORS row says so.
- *2026-10-12, orchestrator, stage 3c opening:* **the load-factor coupling leaves the sparse
  factor through a bordered system, behind a referee option for one slice.** The coupling
  `c · d dᵀ` is a dense (bodies)² block on the demand's support (84 % of the 32 × 40 wall's
  non-zeros, 62 % of its certificate time), and the site part alone is singular, so a
  Sherman–Morrison correction is ill-posed (35 to 115 iterations when tried). The bordered
  symmetric indefinite system `[M_sites + ρI, d; dᵀ, −1/c]`, factored by `SimplicialLDLT`
  with the border eliminated last, solves the original system exactly (`y = c dᵀx`), keeps
  the site block's sparsity; eliminating the border last means factoring the site block first,
  so it needs positive site pivots as Sherman–Morrison did, with the same regularised retry as
  today when they are not (corrected by the cold review; see "stage 3c rulings from the cold
  review"). Its success condition is positive site pivots and one negative border pivot. `CertificateOptions::CouplingKind { InFactor, Bordered }` defaults to Bordered;
  InFactor is the third referee removed at the 6b close. The contract is answers unchanged
  (iteration counts equal, load factors to 1e-9) and the large-component time down; the Slow
  ratio test's bar is set halfway in log terms between the first measured ratio and 1.0. The
  same round adds the owed options-reuse variants (CapSite, the stall fields, Coupling).
- *2026-10-12, orchestrator, stage 2 third amendment (from the follow-up review):* **a move
  keeps the lineage in the destination and mints a fresh one for the source.** A defaulted
  move left the moved-from structure holding the same token, and a host that rebuilds that
  object (the builder-variable-outside-the-loop pattern) re-issues the same slot ids under the
  same lineage; the review's repro judged the rebuilt source, then reused its brick on the
  destination and read Collapses where a fresh judge reads Stands. The token becomes a small
  member type that owns these rules so `Structure`'s special members can be defaulted again
  (the hand-listed copy members were a second trap). The review also found six of the eleven
  option-field comparisons and a force-only load change unpinned (each a surviving mutant),
  and the certificate-option checks pinned only without a Previous; all in the second fix
  round.
- *2026-10-12, orchestrator, stage 3a rulings from the cold review (2623138):* the algebra
  is exact (Schur complement, structured `F`, group pivots bit-identical to dense; 40,000
  harness cases, 3,111 suite calls under each kind with no state or count difference, nothing
  past 1e-9 relative) and the dense path is bit-identical to 77e2be7 on 113 of 113 comparable
  tests. The code needs no change. Sent back on tests: the group-pivot guard and the
  mean's `Total > 0` guard survive removal, the ruling's zero-slack case is not enumerated,
  the 12-fixture guard's mutation records describe the dense path, and the shed ratio test's
  fixed order D, S lands any load change on one side (fix round `stage-3a-fix`: kinds TS1–TS4,
  records TS5, order D, S, S, D with the minimum per kind). Deferred to CURRENT_STATE: the
  structured `SolveNormal` cap loop allocates a temporary per site per solve and is slower
  than dense (1.70 s against 1.05 s on the shed, about 1 s recoverable with `SolveInto` into a
  scratch); three copies of `ShedSpec` in the tests; an out-of-range `CapSiteKind` selects
  Dense unvalidated; no regularised retry at the cap-site level (pre-existing).
- *2026-10-12, orchestrator, stage 3c rulings from the dev round:* ratified. The ordering is a
  functor that runs AMD on the n × n site block and appends the border as the last
  permutation entry; the pattern is analysed once per decision (the demand's support is fixed
  per component, as the LLT path already assumes). On the retry ρ is `RegularisationRelative`
  times the largest of `M_sites_ii + c·d_i²`, not the site diagonal alone, so the regularised
  bordered system is algebraically InFactor's regularised one. A non-positive c makes −1/c
  non-finite, fails the finiteness checks and reads Undecided (fails closed; InFactor would
  handle c = 0; unreachable with interior-point D > 0). `Coupling` has no range validation,
  matching `CapSite` (both logged in CURRENT_STATE). The dev round recorded the border pivot's
  sign check and the first-n positivity check as unreachable; the cold review showed that
  wrong (see its entry). Measured: iteration counts equal and bounds
  identical on every case; the 40 × 32 wall's certificate 118 s to 34.4 s, the half
  warehouse's 89 s to 58.6 s; the Slow ratio bar 0.93 from a median 0.871 on the 310-body
  wall, thin over its worst run (0.911), which the 3c review should weigh against a larger
  fixture.
- *2026-10-12, orchestrator, stage 2 second fix round (merged):* ratified from the dev round:
  `LineageToken` is a private nested class of `Structure` (not a detail namespace); the
  default constructor mints, so `Structure() = default`; copy self-assignment keeps the
  current value rather than minting; the revision counter is untouched by a move (the
  moved-from structure keeps its count, which the rebuilt-source test relies on). The merge
  kept the fix round's eight-variant `Add` table in the options-reuse test over the 3c
  branch's three and added the `Coupling` variant to it.
- *2026-10-12, orchestrator, stage 3b rulings from the cold review (722c41b):* the rule is correct
  (it returns exactly the cap's outcome; across the default and Slow suites it fired five times,
  each on a solve that fails at the cap with the rule off; the tracked quantity is the
  convergence test itself). **The default `StallWindow` becomes 20.** Converging anchors contain
  runs of 9 iterations without halving the best residual (the harmonic stack converging at 18,
  the Couplet–Heyman arch at 20–21), one short of the window of 10, and stage 3c moved the
  iterates; a cut there is a silent false Undecided or a looser bound. The half warehouse's
  stalled solve (window start 62) still stops at 82. The factor and the off-by-one of the rule
  survive mutation, so the fix round pins both (Undecided under `StallImprovement` 1e-6 with
  window 3; the firing iteration exactly WindowStart + W), renames the never-converging test to
  what it asserts, and tightens the Slow anchor's iteration bar to the measured count.
- *2026-10-12, orchestrator, stage 3d dropped:* the 3c measurement meets the warm-cascade
  target the stage-2 opening set (warehouse post-removal cascade with four of five components
  reused: 175 ms against 200 ms; 254 ms at stage 2), and the warehouse's full certificate is
  103 s from 253 s. A verdict-only option would save only the readings' back-substitution and
  is not worth a third option on the certificate; the stage closes at 3c once its review and
  the 3b fix round land.
- *2026-10-12, orchestrator, the half warehouse anchor after 3c:* the 3b fix round found the
  595-body wall's circumscribed solve converging since stage 3c (Stands [350.208526108,
  350.423921758] in 66 iterations under windows 0, 10 and 20), so the stall rule no longer
  fires on it and the Slow anchor's "upper equals the cap" check is red at base. Ruled: the
  anchor becomes a convergence pin (lower and upper within 1e-6 relative, at most 69
  iterations), keeping its place in the Slow tier as the one large certificate the suite
  solves to convergence. The rule's own pins are the ec6 1e-15 wall (firing at exactly window
  start + W) and the shear-capped brick under a 1e-6 factor; it is a safety stop, and the
  suite now holds no large fixture that reaches it, which is the intended state. The ruling's
  "still stops at 82" is void with it.
- *2026-10-12, orchestrator, stage 3c rulings from the cold review (425506a):* the algebra
  is exact and the border is last at runtime (`permutationP().indices()(n) == n`); the code
  needs no change; the lineage fix (Part B) is clean under swap, container reallocation and
  copies of moved-from structures. **The record was wrong about the pivot guards.** The
  bordered factor factors the site block first, so the site pivots must be positive: the
  suite reaches the retry ten times (first-attempt `info()` Success with a site pivot between
  −3.6e-7 and −3.7e-3 and a positive border pivot, c between 1e10 and 2.5e14, in the Coupling
  agreement tests and `Sparse.AgreesWithTheDensePath`); in nine of the ten InFactor's first
  attempt would have succeeded. The two checks back each other up (removing both leaves the
  88 Certificate/Authority tests green at the same 1.76e-11 worst gap), which is why neither
  alone was caught; Sylvester only makes the border check redundant once the site pivots are
  positive. "Needs no invertible site part" is withdrawn. The Slow ratio test moves to
  `Ec6Wall(30, 24)` (735 bodies: InFactor 18.2 s, Bordered 11.3 s, ratio 0.62, bar 0.79), run
  I, B, B, I, because at 310 bodies the bar had a 2 % false-red margin against a 5 % detection
  margin (forcing InFactor reads 0.976, forcing Bordered 0.990). The `requires` stand-ins go.
- *2026-10-12, orchestrator, slice 6b close opening:* **the three referees go**, as the stage
  1a, 3a and 3c openings ruled: `FactorKind::Reference`, `CapSiteKind::Dense` and
  `CouplingKind::InFactor`, each with its `enum` and its field in the options (the structured
  bindings in the option comparisons shrink with them). What pins the behaviour afterwards:
  the blocked factor's agreement tests become pinned-value tests measured against the
  reference one last time (states, iteration counts, a short list of readings to 1e-10); the
  cap-site unit tests keep their in-test dense oracle and the 12-fixture table keeps the
  certificate; the bordered path keeps `Certificate.Sparse.AgreesWithTheDensePath` (sparse
  against the dense normal equations below `DenseRowLimit`, two paths that both stay) plus
  its pinned-fixture and singular-block tests; the two Slow ratio tests are retired with their
  last measured ratios in ANCHORS and the rows in SCALE.md. The independent check the
  referees gave the elastic factor passes to slice 7's HiGHS referee.
- *2026-10-12, orchestrator, slice 6b close, pins ruling (from the cold review of 4928119):*
  the review found the removal diff clean and the generated pins a real guard (seven factor
  mutants red, including panel-update faults no analytic anchor reaches), with one defect: the
  pins fixed exact iteration counts on Undecided draws, and a change of floating-point order
  alone (`PanelWidth` 32 or 128) moved one from 50 to 43 or 41. **The pins pin what the
  stage-1a ruling compared and no more**: on Stands and Overstressed the state, the exact
  iteration count, the unconstrained-body count and the readings; on Mechanism the state and
  the count of unconstrained bodies; on Undecided the state only; Mechanism and Undecided may
  trade against the pin on at most max(1, draws / 1000) draws per table. **The generator is
  committed** (`Measure.Factor.GeneratePins`, deterministic output), and regenerating the
  header is a snapshot refresh of the code under test, allowed only with a reason recorded in
  ANCHORS; the single comparison against the reference factor stands at 4e7fbb8 (worst
  certified reading gap 5.8e-14). Also from the review, into the fix round: mechanism motions
  are checked against the null space of the assembled stiffness (the reference used to compare
  them); the dense-against-sparse certificate comparison is tightened to the retired coupling
  test's strength (load factors 1e-9, iteration counts equal), since at 1e-7 and ±2 it passed
  a 10 % error in the border's coupling; the bordered regularisation's size (× 100 green) and
  the retry's `LargestDiagonal` without the coupling are recorded as uncaught.
- *2026-10-12, orchestrator, slice 6b closed:* the follow-up review of the fix round (e310d5a)
  found nothing blocking: equivalent changes of floating-point order leave the pins green,
  seven real factor mutants redden them (tolerance × 2 already trips the budget, 8 against 6),
  the generator is deterministic, a border coupling off by 0.01 % reddens the tightened
  comparison. Merged to master. Not done from the pins ruling's list: the null-space check of
  mechanism motions, which needs a seam to the assembled stiffness (CURRENT_STATE carries it).
- *2026-10-12, orchestrator, carried from 6b, item 1 opening:* **an ungrounded component is a
  mechanism by connectivity, read before any factorisation.** A component with no live
  interface to a grounded body cannot be in equilibrium under gravity; today the elastic
  authority assembles and factorises it (the sparse estimate rejects the singular stiffness,
  the dense blocked factor finds the rank defect: 20.5 s on the 1,256-brick remainder of the
  32 × 40 wall) to read what connectivity already says. After this item it reads Mechanism,
  every body unconstrained, the motion the rigid fall (every body translated (0, 0, −1) in
  the mechanism normalisation, zero rotation) whatever the host load (the rule is
  connectivity, not force; the certificate's gravity-only rigid-fall rule decides the
  release, which is the combined authority's and cascade's business), zero active-set
  iterations, and **every reading of the component zero with axis None**: nothing is
  certified about the joints of a body that falls rigid, the stage-1a ruling already compared
  readings only on certified states, and today's values there are the range solution (mostly
  infinite tension), which the cascade once copied as a sever reason. The reading list still
  covers every live interface. Grounded components are untouched, bit for bit.
- *2026-10-12, orchestrator, carried from 6b, item 1 amendment (from the cold review):* **the
  shortcut applies to an ungrounded component that carries no host load, under a non-zero
  gravity.** The review's probe showed a floating brick held up by a +m·g load reading
  Undecided (elastic Mechanism, the certificate's general path at a bracket [0, 1000]) where
  it read Stands, turning a whole structure's verdict; a held pair kept Stands but counted a
  disagreement and lost a certified reading. Connectivity alone says "cannot be in
  equilibrium" only when the only load is gravity, which is the 6a amendment's condition for
  the rigid fall, and those are the cases the measurement needed (the base-course remainders
  carry no host load). A loaded or weightless ungrounded component takes the full solve as
  before. The finiteness check the full path makes at its first iteration must still apply
  before the shortcut (a floating pair with a 1e308 stiffness read InvalidInput and briefly
  read Collapses). Pinned in the fix round: the held brick Stands with its reading, the held
  pair's disagreement count, InvalidInput on the non-finite pair, a tilted gravity's motion,
  the zero-gravity pair as before, the corner points carried, and the sever reason of a lifted
  floating pair reading None 0 where it read Tension inf.
- *2026-10-12, orchestrator, carried from 6b, item 1, correction of the amendment:* the
  amendment's last pin contradicted its own rule: a lifted floating pair carries a host load,
  so it takes the full solve and its sever reason stays what the full solve reads (Tension
  inf, the 6a note); a sever reason of None 0 arises only when an unloaded floating
  component's internal joint is severed, which the rigid fall never does, so there is nothing
  to pin and that test is withdrawn. Likewise the pushed floating pair takes the full solve
  (Mechanism after at least one iteration), not the shortcut. **A `BodyLoad` with zero force
  is no load**: the shortcut still applies, matching the cascade's existing
  `AZeroForceLoadDoesNotStopTheRigidFall`.
- *2026-10-12, orchestrator, carried from 6b, item 1, second fix round (from the follow-up
  review of 3027071):* two defects in the amendment's implementation, neither in the rule.
  **The finiteness check before the shortcut makes the same decision as the full path's
  first-iteration check:** the full path sums the unscaled stiffness blocks and scales the
  dense sum (`D*AssembleStiffness*D`), the shortcut scaled each block and then summed, and
  the two overflow at different stiffnesses, so in a band of about 3.6e304 to 1.45e305 Pa/m an
  unloaded floating pair read Mechanism (judge Collapses, cascade Stands with a cluster) where
  the same pair with a 1 N push, and master, read InvalidInput. The shortcut now assembles
  unscaled, scales afterwards, then checks. **The fall direction divides each gravity
  component by the largest magnitude** instead of multiplying by its reciprocal, which
  overflowed for a gravity of 1e-310 and read a NaN motion. Under such a gravity the elastic
  state is Mechanism by the ruling (any non-zero gravity takes the shortcut) where master's
  full solve read Stands; the judge reads Stands with one disagreement, and the review showed
  that to be a certificate defect, not the answer: the certificate's own rigid-fall rule takes
  the length of the net load by summing squares, which underflows below about 1.5e-162 N, so
  it never fires and the LP reads [1000, 1000] (master read Stands because both of its
  authorities lost the load). The test pins the judge to neither InvalidInput nor Undecided
  and the certificate underflow is logged in CURRENT_STATE. Logged
  as pre-existing, not from the item: a 1e308 kg mass whose weight overflows is never
  validated; the sparse and dense finiteness checks above `DenseDofLimit` sum in different
  orders and can disagree in the same band; a gravity of 1e-300 counts as non-zero.
- *2026-10-12, orchestrator, carried from 6b, item 2 opening (the cascade's scans):* a
  per-phase profile of the base-course cascade on master (685a5be, Release, a throwaway copy
  with timers in `BrittleCascade::Run`): the 40 × 32 wall judges its floating remainder in
  77 ms and spends 32 ms in `Release`; the half warehouse 137 ms and 25 ms; the warehouse
  390 ms and 119 ms (10,519 readings, a 1,214-body cluster). `CrackBand` costs nothing on
  these fixtures (no Overstressed component) but has the same shape. The release cost is the
  quadratic scans: `ClustersOf` finds each interface's bodies in the release list by linear
  search, `Release` tests every live interface against every cluster by linear search, and
  `Structure::RemoveBody` walks every interface slot per removed body. **Ruling: index them,
  changing no answer.** The structure keeps a per-body list of incident interface slots
  (appended by `AddInterface`), so `RemoveBody` and `InterfacesOf` cost the body's degree;
  the cascade builds, once per pass, a reading index by interface slot and a cluster-of-body
  table by body slot, assigns severs in one ascending sweep of the live interfaces, and the
  crack band tests component membership through a flag table. Cluster order, each cluster's
  `Severed` order, the `Failures` order and every verdict stay what they are, which the
  `Cascade.*` suite pins. The measurement is a ratio: a field of K floating pairs cascaded at
  K and 4K reads about 16× today and about 4× indexed (the judge is linear in K), with the bar
  halfway between in log terms (8×), in the Slow tier.
- *2026-10-12, orchestrator, carried from 6b, item 3 opening (the shortcut's finiteness fold):*
  the ungrounded shortcut assembles and scales a sparse stiffness only to test it finite:
  65 ms of the 70 ms the 1,256-brick floating remainder costs, 4.5 ms before the gate. A
  block-wise check (any non-finite block) is necessary but not sufficient: finite blocks sum
  past DBL_MAX in the 3.6e304 to 1.45e305 Pa/m band, which is the band the second fix round
  pinned. **Ruling: fold the blocks instead of assembling them, in the dense check's order.**
  `ForEachStiffnessBlock` emits, per joint and per corner, the four 6 × 6 blocks of the body
  pair; the dense check adds them entry by entry in that order. The fold keeps one 6 × 6
  accumulator per body (its diagonal block) and one per joint body pair (the off-diagonal
  blocks; two joints on the same pair add in joint order, as the dense sum does), adds in the
  same callback order, then scales each accumulated entry by its two characteristic-length
  factors and tests it finite. The per-entry addition order is the dense check's, so the
  decision is bit-identical, which the agreement sweep (the follow-up review's
  `Probe.SweepSmall`: the unloaded route reads InvalidInput exactly when the pushed route does,
  swept over 1e300 to DBL_MAX on the big pair and a ×40 wall remainder at `DenseDofLimit` 600
  and 0) now pins in place of the two absolute stiffness pins. The Slow remainder bar drops
  from 2.0 s to 30 ms, red at 70 ms today and 3× above the expected cost. (Green step: the
  fold measured 14 to 17 ms on a loaded machine and 11 ms quiet, so the bar was set at 50 ms,
  3× the loaded reading; recorded in ANCHORS.)
- *2026-10-12, orchestrator, carried from 6b, item 3 red step note:* the agreement sweep on
  685a5be agrees at all 100 points for both fixtures, three modes and both limits; the big
  pair crosses to InvalidInput between 3.18e304 and 3.85e304 Pa/m in the (k, k) and (1e9, k)
  modes and between 6.86e304 and 8.31e304 in (k, 1e9); the ×40 wall remainder between
  1.22e305 and 1.48e305, and 2.17e305 and 2.63e305. The band the second fix round named
  (3.6e304 to 1.45e305) was the disagreement band of the old scale-then-sum check on the
  big pair, not a fixture's crossing. The test runs in 0.5 s and stays in the default tier.
- *2026-10-12, orchestrator, carried from 6b, item 2 amendment (the judge is quadratic in the
  component count):* with the cascade's own scans indexed (9af35b7: `Release` and `CrackBand`
  10 ms at 8,000 floating pairs), the ratio test stayed red at 15 to 17× because
  `CombinedAuthority::Judge` costs 483 ms at 2,000 components and 6.2 s at 8,000. The opening
  said the judge was linear in K; it is not. The elastic side is (one `Solve` per judge since
  stage 2), but the combined authority consults the certificate **once per component** whose
  elastic state is neither Stands nor Overstressed, and every `Decide` call validates the
  whole input, sizes its tables to every component and walks every live interface before
  deciding the one component asked for: O(K · (I + K)) over a judge. **Ruling: one `Decide`
  per judge, with per-component results, as the elastic authority already does.**
  `CertificateResult` gains `Components`: for each decided component in partition order, its
  bodies, state, load-factor bounds, iterations, mechanism, unconstrained bodies and hinging
  interfaces; the aggregate fields keep their meaning (worst state, smallest bounds, largest
  iteration count, concatenations) so `DecideWith` and every certificate anchor read as
  before. The combined authority collects the dirty components that need the certificate,
  calls `Decide` once with their bodies as `Only`, and fills each report from its entry; a
  global InvalidInput (invalid input or an unusable `Only`) reads InvalidInput on every
  consulted report, as the per-component calls read it. The existing event-region pins (one
  `Decide` with the dirty component's bodies) hold, since they have one dirty component.
  `FindClean` compares every component against every previous report (quadratic on warm
  judges); it is indexed by front body in the same change only if the dev measures it on the
  pair field, otherwise logged. The ratio test is the measurement; a counting certificate pins
  the call count on a many-component structure.
- *2026-10-12, orchestrator, carried from 6b, items 2 and 3 cold-review rulings (of 97a2ff9):*
  the review's differential over 964 judges found every cluster, sever, failure, stamp,
  reading, verdict and `InterfacesOf` list identical to 685a5be, the fold's decision
  bit-identical to the dense check over 72 fixture combinations, and four defects. **(1) A
  `CertificateComponent` carries capped bounds**, `min(LoadFactorCap, ·)` on both, as the
  aggregate always did; the entries were carrying the raw solve values, a hair above the cap
  on ten of the differential's reports (the 1e-9 comparisons hid it). The judge-against-alone
  property compares bounds exactly and asserts every entry at or below the cap, with a cap of
  1.2 as well as the default. **(2) `Release` marks positions and hinges per component**, not
  across the pass: with the marks shared, an interface from one component's released body to
  another component's released body (impossible from the shipped authority, reachable through
  the public seam) indexed the wrong union-find array and merged bodies across components. The
  answer is the old one: such an interface is skipped and each body stays its own cluster. The
  tables are built once and cleared per component, so the cost stays linear. **(3)
  `CrackBand` buckets the readings by component in one sweep** through a component-of-body
  table (a reading belongs to the component of its free body; an interface cannot span two free
  components; order within a component is the reading order), replacing the per-component
  scan of every reading: a field of K Overstressed single bricks read 14.9× from K to 4K. A
  Slow ratio test pins it at the 8× bar. **(4) A certificate that returns no `Components`
  entries is non-conforming and reads InvalidInput on every consulted report**, the judge's
  worst InvalidInput and the cascade InvalidInput with no clusters; a component that did not
  consult it keeps its verdict. Pinned through a wrapper certificate that clears the entries.
  Logged, not fixed: the pass-loop load filter and `MotionOf` are linear searches (O(loads ×
  released) and O(clusters × mechanism) per pass); `FindClean` and the report index call
  `front()` on a previous report's bodies, undefined for a forged empty list; an
  association-order mutant in the fold, `d_i·(K·d_j)`, survives the sweep and the review's
  exact bisection (the two orders agree to the last bit on every fixture tried, so there is
  nothing to pin); the fold test could adopt the review's exact-edge bisection with a
  twin-joint pair and a six-brick slab.
- *2026-10-12, orchestrator, carried from 6b, items 2 and 3 re-review rulings (of 0e7fa19):*
  the review's differential against 685a5be is byte-identical over 964 judges, 300 fuzz
  seeds pass by pass and 36 ec6 walls (1,062 cracks, 1,202 severs). Two records corrected and
  one ruling added. **Ruling (1)'s pin is `Authority.Judge.MatchesSolvingEachComponentAlone`
  itself**, now comparing bounds exactly (NaN-aware, since unconsulted reports carry NaN
  bounds) at the default cap and at 1.2, alongside the filtered-aggregate test the fix round
  added. **A body listed in two release lists, or twice in one, severs as API.md's
  sequential rule says**: the cluster-of-body table keeps the first cluster that claims a
  body, as the old per-cluster loops did (685a5be's answer, restored by the one-line fix the
  review trialled; a scripted test pins both shapes). The orchestrator made the test and the
  line itself after confirming the red, the change being the review's own specification.
  The crack-field ratio test times best of three at each size (its small run is 1.5 ms, so one
  scheduling hiccup on the large run could cross the bar). Logged: the capped-bounds test's
  power at caps 1.2 and 3.0 rests on one report (seed 0x6b1b2 + 14); a forged body in two
  Overstressed components is owned by the first in the crack band, where 685a5be let the
  second count it too; the indexed tables match a forged stale-generation handle by slot.
- *2026-10-12, orchestrator, slice 2d opening (laterally loaded dry walls converge):* the
  investigation (a probe copy of the solver with the active set dumped per iteration, Release,
  cd25019) found the cause of the 2c leftovers. Every iterate is in equilibrium (residuals at
  roundoff); what cycles is status consistency, and only on perpend corners: the 3 × 4 wall at
  λx 0.1 alternates two base-course perpends with period 2, the 5 × 5 at λx 0.05 has period 6,
  the 4 × 2 at λ (0.30, 0.15) period 2 on one top perpend, the 10 × 8 at λx 0.2 wanders and
  settles into period 36. No slip direction ever flips and no bed corner ever changes. **The
  mechanism: a corner that re-closes (Open to Stuck) brings with it the whole tangential
  displacement it built up while open; as elastic shear that is 30× to 1,700× the corner's
  friction capacity, which wrenches the field, puts the corner (or its twins on the perpend)
  into tension, and the opening test, which runs before the slip test, opens it again, so it
  never reaches Slipping.** The remedies tried: one change per iteration (Dantzig) and an
  NNLS-style single release converge but slowly and break the two-iteration rest pins (89 of
  160 grid points Undecided at cap 50); forbidding Slipping-to-Stuck, under-relaxing the slip
  direction and a status-hash anti-cycling rule do nothing, because the cycle is open/close.
  **Ruling: a slip-releasing corner that re-closes enters Slipping when its trial shear
  exceeds μN at closure, holding μN along the trial shear,** which is what the restore step's
  `ReEnterSlipping` and the 2a ruling already do for restored corners and what
  `UpdateActiveSet`'s Open branch never did. Simultaneous updates stay; the consistency test
  that certifies a converged state is unchanged, so Stands remains a lower-bound certificate.
  Measured in the probe: the four points Stand in 9, 8, 13 and 9 iterations; the 160-point
  grid (eight sizes × λx {0, 0.05, 0.1, 0.2, 0.3} × λy {0, 0.05, 0.1, 0.15}) goes from 47
  Stands and 113 Undecided to 160 Stands, mean 8.1 iterations, every rest point still 2, the
  worst 30 (10 × 8 at (0.3, 0.15)), with zero Stands the NNLS oracle refutes; the 14 × 10 on
  the sparse path at λy 0.1 Stands in 22 to 33 iterations where it read Undecided at 50. The
  points are not ill-posed: the oracle keeps them CanStand to 7× to 14× the test load, and
  four different update rules reach the same readings to 1e-12 with identical slipping
  counts. Consequences accepted: the generated factor pins and the sparse hidden-pivot pin
  move (62 Undecided to Stands, 30 fewer iterations, 175 Undecided to Mechanism, 196
  Mechanism to Undecided of which 145 by the restore step, 7 mechanism shapes; the oracle
  refutes none of the new Stands), so they are regenerated under the pins ruling with this
  entry as the reason and an oracle audit of zero false Stands; iteration counts grow with
  wall size, so the cap of 50 is measured against the warehouse's walls under lateral load
  before it is trusted at scale (a separate ruling if it needs raising); the restore-step
  Undecideds (a 2c open item) become more visible and stay logged. Pinned: the mechanism (a
  dry corner re-closing with a tangential offset above μN/kt goes Open to Slipping holding μN
  along the offset; "close into Stuck" must redden it), the four points' iteration counts
  with margin, the grid with zero refutations, the 14 × 10 sparse case, the rest walls at two
  iterations, every existing anchor.
- *2026-10-12, orchestrator, slice 2d dev notes (corrected by the cold review):* the Open
  branch calls `ReEnterSlipping` with the trial shear and normal (which sets the transverse
  stiffness to zero, right for the restore step, where there is a motion but no trial shear)
  and then sets the transverse stiffness to kt·μN/|trial shear|, exactly what the
  Stuck-to-Slipping branch sets, so re-closing means close, then apply the slip test to the
  same trial in the same update. Without that line the stiffness stays zero and the pin
  trades read 62 / 30 / 168 / 206 / 6 against the probe's 62 / 30 / 175 / 196 / 7; no test but
  the generated pins tells the two apart, and neither changes a Stands state. (An earlier
  wording of this note said the line zeroes the stiffness; it does the opposite.) The grid test now runs the NNLS oracle on
  all 160 Stands and takes 6.5 min, so it stays in the Slow tier. The warehouse's mortared
  walls under λx 0.1 Stand or read Overstressed after one iteration, the 34-body frame
  Mechanism after three: the cap of 50 is nowhere near at scale, and the dry rule barely
  enters. The regeneration's oracle audit: all 62 new Stands CanStand; of the 1,420 draws
  Stands before and after, 26 Ambiguous and one CannotStand, pre-existing, handed to the cold
  review. The mutant applying the rule to cohesive (non-slip-releasing) corners survives the
  suite: a cohesive fixture that pins the ruling's scope follows.
- *2026-10-12, orchestrator, slice 2d cold-review rulings (of c4a372a):* the rule sits only in
  the Open branch, only on slip-releasing corners, the certifying consistency test is
  unchanged, and across about 2,600 Stands (the pins, the grid, 358 random dry walls, 347
  stacks and bridges, 281 pairs) the oracle refuted none and every dry field satisfied its own
  equilibrium and cone checks to 1e-12 of the weight. The pins diff has no Stands lost and no
  Stands taking more iterations. **The one pinned Stands the oracle read CannotStand is an
  oracle artefact**: `PanelPins[1]`, the 8 × 10 ec6 wall under λ (0, 0.5), whose bond carries
  tension and cohesion the NNLS oracle does not model (its cones are compression and friction
  only); the authority's field is in equilibrium to 3.8e-15 W with the bond carrying −0.038 W
  of tension and 0.0125 W of shear beyond friction, and the monolith estimate 0.88 sits above
  0.5. Ruled: the oracle refuses any structure with a tensile or cohesive profile, so it cannot
  be pointed at a bonded wall again. **The scope mutant** (the rule applied to cohesive corners)
  would hide a sheared bond, since a Slipping corner holds μN and its ratio against cA/4 + μN
  can never trip `bShearedBond`; the lime-wall test catches it, and an invariant that no corner
  of a cohesive joint ever reads Slipping goes into the fuzz. **The slip direction at entry is
  not pinned by the slice's tests** (a reversed held force survives everything but the pins):
  pinned through named draws whose convergence depends on it (seeded 3329 Stands in 21 where
  the reversed entry reads Undecided at 50; 3253 in 11 against 9), and an own-field certificate
  property (equilibrium of every Stands from its readings, cone and no-tension checks on dry
  joints) that implies the oracle's answer on dry fixtures in milliseconds. The grid test's
  cost is the oracle, not the solves (under a second): the convergence half moves to the
  default tier under `LOADPATH_SLOW_IN_DEBUG` with the own-field check, the NNLS stays on a
  subset in Slow. The cap of 50 is close near the stability limit, not at size: random walls at
  μ 0.99 and λ (0.38, 0.28) took 46 iterations; over the cap fails closed to Undecided and the
  certificate referees it (logged).
- *2026-10-12, orchestrator, slice 2e opening (a re-closing cohesive corner):* the lime-mortar
  walls that read Undecided under lateral load (the 4 × 5 at λx 0.8, the 5 × 5 at 0.8, the
  10 × 8 at (0.2, 0.1), (0.5, 0.1), (0.8, 0) and (0.8, 0.1); 52 of a 140-point wider grid)
  cycle for the 2d reason: a cohesive tensionless perpend corner re-closes with the tangential
  offset it built up while open, 60× to 260× its friction capacity but under 6 % of its
  cohesive capacity cA/4 + μN, the fictitious shear puts it back into tension, and it reopens,
  with periods of 2 and 4; on the 10 × 8 single bed corners join in through out-of-plane
  rocking. The dry rule is wrong here, as the scope ruling said and the probe now shows: a
  cohesive corner put into Slipping holds μN, skips the slip update, and four of the seven
  points then fail their own equilibrium by up to 2e-2 W. The solver keeps no shear reference
  (shear is kt times the total tangential displacement from the as-built position), and the
  iteration is not a load history: a tensionless cohesive joint in contact may carry up to
  cA/4 + μN at any normal force, so the offset is an artefact of the iterates, and a Stands
  certifies an admissible field, not a unique one. Rules tried: capping the re-closed shear at
  the cohesive capacity does nothing (no trial reaches it); resetting the reference at every
  re-closure stands all seven but moves 12 converged points by up to 0.28 W; **resetting it
  only from a corner's second re-closure in one solve stands all seven (8, 6, 9, 10, 9, 16, 10
  iterations), takes the wide grid from 52 Undecided to 5 (the 10 × 8 at λx ≥ 1.0, untraced),
  passes the own-field check everywhere (equilibrium to 1.7e-13 W, no tension, |S| at most
  0.098 of capacity, no cohesive corner Slipping), and leaves every solve that converges today
  bit-identical, pins included (every pinned fixture is dry or ec6, whose corners never
  open).** Ruling: a cohesive tensionless corner that re-closes for the second or later time in
  one solve re-closes Stuck with its tangential reference reset to the current offset, carried
  as a held pre-force kt·Ref in the load the way `HeldN` is; the first re-closure keeps the
  full offset; a guard keeps the full offset whenever the forgotten shear kt|Ref| would exceed
  cA/4 + μN, so the reset can never mask a sheared bond (the probe's worst forgotten shear is
  0.19 of cA/4); the certifying consistency test and the scope ruling (no cohesive corner ever
  slips) are unchanged. Accepted: readings at formerly cycling points are path-dependent (up to
  about 1 W per interface on the 10 × 8 between equally admissible rules), so the crack band
  could pick a different first joint than another valid rule would; per-corner state grows by
  a reference and a re-closure count. The physical answer: these walls stand only because of
  cohesion (λx 0.8 exceeds μ 0.667; the base bed's cohesion has about 94× margin), which the
  friction-only oracle correctly refuses to certify. There is no normal-direction twin (a
  re-closing corner's normal force has a geometric zero); the opening side drops the elastic
  shear at opening, under 4 % of cA/4, and stops cycling with the reset. Pinned: the seven points
  within iteration bars (12; 20 for the 10 × 8 at (0.8, 0)) with the own-field check; the 25
  converged grid points unchanged (readings pinned at 1e-12 on a sample, which the every-
  re-closure rule breaks); a guard fixture where the forgotten shear would exceed the
  capacity, if one can be built; the lime scope test and fuzz stay green.
- *2026-10-12, orchestrator, slice 2e dev notes:* landed as ruled. `JointState` carries a
  per-corner tangential reference `ReferenceM` and a per-solve `ReclosureCount`; both start at
  zero and `States` is built fresh in every `SolveProblem`, which is the per-solve reset. In
  `UpdateActiveSet`'s Open branch a corner of a joint with `!bReleasesSlip` that re-closes Stuck
  calls `ResetReferenceOnReclosure`: the count goes up, the first re-closure returns, and from
  the second the reference becomes the current offset −T.ShearN / kt unless the forgotten
  shear kt·|offset − Ref| fails `<=` cA/4 + μ·max(N, 0) (written so a NaN keeps the old
  reference). `AssembleLoad` adds kt·Ref as a pre-force on every Stuck corner, + on B and − on
  A, exactly as `HeldN` enters for Slipping corners; `ReadJoint` reports a non-slipping
  corner's shear as T.ShearN + kt·Ref, the elastic shear the corner carries, which is what the
  Overstressed ratio and the own-field check read. Slip-releasing corners, the slip test (which
  cohesive corners skip), the consistency test and `RestoreInadmissibleReleases` are untouched.
  Measured: the seven points Stand in 8, 6, 9, 10, 9, 16, 10 (the probe's counts exactly); the
  140-point grid 128 Stand, 5 Undecided, 7 Mechanism, no own-field failure, 2.7 s (7.2 s red);
  every pin and anchor unchanged, 379 Release / 352 Debug green, `Slow.*` 14 green, the shed
  and half-warehouse judge times unchanged within noise (31 / 147 ms against 44 / 150 ms at
  HEAD). Mutants: no reset reddens the seven and the grid at the red step's counts; reset at
  every re-closure reddens `ConvergedWallsAreUnchanged` at five of seven points (four converge
  one iteration sooner, the 5 × 5 at (0.8, 0.1) moves its readings); reset on slip-releasing
  corners too is caught by `Statics.Factor.TheBlockedFactorReadsItsPins` (four draws, 125
  trades against a budget of 6), not by a Dry test. **The guard is unobservable by verdict.**
  The red step's guard fixture (c 1 kPa on every joint, 2 × 5 at λx 0.3 to 0.7) reached its
  second re-closures with forgotten shear at 16 to 38 % of cA/4 + μN, so the guard never
  fired and the walls Stand in 5 under the rule; the orchestrator then swept 90 points of
  weak-perpend walls (lime beds, perpend c 50 to 500 Pa, 2 to 4 courses, λx 0.3 to 1.0) under
  the rule and under the rule with the guard removed. At every point where the guard fires the
  guarded solve reads Undecided at 50 (the kept offset keeps the cycle) or Overstressed, and
  the unguarded solve reads Overstressed after 4 to 9, from neighbouring perpends that carry
  comparable shear and are over the same capacity; at every point where the unguarded solve
  Stands the guarded one Stands identically. Ruling: the guard stays, as the fail-closed
  invariant the opening asked for (a reset never forgets more than the bond could carry), with
  the record that no reachable fixture isolates it by verdict, so the mutant "guard removed"
  survives the suite by design; the fixture is rewritten to pin what is observable (the
  weak-perpend 2 × 5 at λx 0.3, 0.5, 0.7 does not read Stands, today Undecided/50). The reason
  is structural: a re-closing corner's forgotten offset is comparable to what its closed
  neighbours carry, so when it exceeds the capacity the neighbours do too. Observed and
  logged, not traced: the five 10 × 8 exceptions at λx ≥ 1.0 read Undecided after 3 to 9
  iterations, not at the cap, so they are an early fail-closed exit (a floor or a factor
  refusal), not the cycle. The grid test stays in the default tier at 2.7 s.
- *2026-10-12, orchestrator, slice 2e cold-review rulings (amendment):* the cold review ran
  the claims above and two of them failed. **Bit-identity was overclaimed.** Twelve of the 82
  lime grid points that Stood at fa1a35d move under the rule (eight change their iteration
  count), because a cohesive corner re-closed twice before the old solve converged; the worst
  moves are 0.87 W per interface on the 6 × 8 at (1.0, 0.1) and 0.77 W on the 10 × 8 at
  (0.7, 0.05), with eleven and sixteen corners changing open status; all six Mechanism points
  change their readings and mechanism; every new field passes the own-field check. The probe
  had sampled, not swept. Ruling: accepted, as the same path dependence the opening accepted
  at the cycling points (a Stands certifies an admissible field, not a unique one, and the
  rule is a choice of reference, not a strength claim); the twelve are pinned at their new
  values so the next rule change has to say so. Everything else is bit-identical: the dry
  grid, the factor, sparse and field tests, the fuzz with its ec6 and lime draws, the Slow
  tier and the three Measure fixtures, 29,900 solves. Not quite: adding kt·0 in `AssembleLoad`
  and `ReadJoint` turned −0.0 shear components into +0.0 on 6,235 of the suite's 6,242
  converged solves; harmless (nothing reads a sign bit) but false under a hexfloat diff, so
  the pre-force and the reading add are skipped when the reference is exactly zero, and the
  claim becomes true. **The guard goes.** The review swept 6,960 fixtures and found the guard
  firing in 709 and changing the verdict in 404: in 400 it turns an Overstressed found in 4 to
  9 iterations into Undecided at 50; in two (a 4 × 3 with c 3 kPa beds at (0.7, 0.1), a 6 × 3
  with c 10 kPa everywhere at (0.8, 0.1)) it turns a Stands with an admissible field (residual
  under 3e-15 W, no violation) into Undecided; in one (one course of three, every joint c 1 kPa,
  30 N pushing up at the middle brick's centroid, λx 0.3; the 1 × 5 does the same) the unguarded solve exits Undecided after 9
  through the restore step where the guarded one Stands in 15; in one both Stand with
  different readings. The structural argument in the dev notes holds only for weak perpends on
  strong beds. The reason to drop it is not the count but the principle the review named: a
  Stands is certified by its readings (equilibrium plus the cone test on every closed corner),
  and by the static lower-bound theorem any admissible equilibrium field proves standing
  whatever path reached it, so the forgotten offset was never a bond force the guard could be
  protecting; the guard bounded nothing (per reset, not in total, and without the shear cap)
  and mostly replaced an answer with a non-answer. Ruling: a cohesive tensionless corner that
  re-closes for the second or later time in one solve resets its reference, unconditionally;
  the weak-perpend fixture now pins Overstressed after 5 at λx 0.3, 0.5, 0.7 (the direction of
  the change) and the two walls above pin Stands with the own-field check; the one-course
  host-load case is logged under the restore step, not pinned. With the guard gone the review's
  surviving mutants "guard on |offset|" and "guard without μN" vanish, and "clear the
  reference when the corner opens" is equivalent (an open corner carries no pre-force and the
  next re-closure re-references anyway), so it is recorded as such (corrected by the follow-up
  rulings below: not equivalent through restore re-closures). **Restore-step
  re-closures neither count nor reset**: the restore step rebuilds the full-rank state for the
  consistency check, it is not an iterate; a cohesive corner it re-closes keeps its reference.
  The mutant that makes them count survives the suite under this ruling. **The five 10 × 8
  exceptions are traced**: all five leave through `RestoreInadmissibleReleases` returning
  Undecided because the closing corner was Open in the full-rank state and the step has no
  trial shear to decide slip; every one is a cohesive corner, which never slips, so the refusal
  is unnecessary for them and re-closing them Stuck is the candidate fix (measured afterwards:
  not sufficient, and the refusal stays; see the restore-step entry below), logged as the next
  item, not folded into this slice. Also logged: readings at the iteration cap on a corner
  reset in the last pass are out of equilibrium (the advanced reference against the previous
  field, the `HeldN` pattern); the grid test pins its Stand and Mechanism counts as well as
  its Undecided; API.md gains the tangential reference, the 2d slip re-entry and this rule.
- *2026-10-12, orchestrator, slice 2e fix-round dev notes:* the guard is removed from
  `ResetReferenceOnReclosure` (from the second re-closure the reference is always −T.ShearN / kt);
  `IsExactlyZero` (exact `==`, no tolerance) gates the pre-force in `AssembleLoad` and the add in
  `ReadJoint`, so a corner whose reference is zero reads exactly what it read before the slice,
  −0.0 included. Measured: the 4 × 3 and 6 × 3 walls Stand in 6 and 10, the weak-perpend 2 × 5
  reads Overstressed after 5 at λx 0.3, 0.5, 0.7, the twelve moved points and the seven converged
  ones do not move, 383 Release / 356 Debug green, `Slow.*` 14 green, no pin moved. Mutants: the
  guard put back reddens the two walls and the Overstressed fixture; the `ReadJoint` skip removed
  reddens the signed-zero test; the count kept across solves reddens `SolvingTwiceReadsTheSameField`
  when run alone, and in the full run (where earlier tests have already polluted the shared count)
  reddens the converged pins and the grid instead, so it is caught either way; the reference
  cleared on opening leaves the suite green, as ruled. Survives: the `AssembleLoad` skip removed
  (adding a +0.0 pre-force to the load changes no reading in the suite; the skip there is for
  symmetry with the reading, not for a measured difference).
- *2026-10-12, orchestrator, slice 2e follow-up review rulings:* the follow-up review closed
  the four earlier findings in the code (every solve that resets nothing is bit-identical to
  fa1a35d, signed zeros included, over 52,336 suite, Slow and Measure solves; the 6,960-fixture
  sweep has no own-field failure and moves 400 Undecided to Overstressed and 2 Undecided to
  Stands against the guarded build) and blocked on three claims. (1) "Clear the reference on
  opening" is not equivalent: a corner that resets, opens, and is re-closed by the restore step
  keeps a non-zero reference under the rule and none under the mutant; the suite never reaches
  that path, a seeded hunt of 100,000 random lime walls with held loads reaches it in 56 solves
  and the mutant moves 14 (one Mechanism after 13 becomes Mechanism after 15, no verdict
  changes). The rule stands (the reference lives until the next reset); the ruling's word is
  corrected. (2) "Restore re-closures neither count nor reset" changes verdicts on the hunt
  (counting: three, Overstressed ↔ Undecided; counting and resetting: six, including
  Overstressed → Mechanism), never a Stands, and had no test: it is now pinned by
  `ARestoreStepReclosureNeitherCountsNorResets` (HUNT 5 Overstressed after 15, HUNT 96643
  Mechanism after 13), which kills both mutants. The ruling may be revisited by the restore-step
  investigation that follows, which has to justify moving either pin. (3) The solve-twice test
  was order-dependent; it now pins 16 iterations on both solves. Corrections to the pick-up
  context: the host load in the one-course case points up (30 N on a 28.1 N brick), and the 1 × 5
  behaves like the 1 × 3. Left for later, logged: the review's seeded hunt as a `Slow.` own-field
  property with held loads (1.3 s per 1,000 draws; the only generator reaching the restore path
  with a reference); test names that describe history (`WallsTheGuardKeptFromStandingStand`,
  `FormerlyConvergedPointsReadTheirNewFields`) are kept, since renaming pins costs more than it
  clarifies while their ANCHORS rows explain them.
- *2026-10-12, orchestrator, the restore step's refusal on cohesive corners (investigated, left
  as it is):* a high-effort probe traced the six lime points that leave through
  `RestoreInadmissibleReleases` (the five 10 × 8 grid exceptions and the one-course 1 × 3 and
  1 × 5 walls with c 1 kPa joints under 30 N of uplift at the middle brick, λx 0.3). In every one
  the closing corner is cohesive, was Open in the full-rank state, and was never re-closed by an
  iterate (reference and count zero). **No host verdict is wrong:** the certificate's inscribed
  cone is exactly the own-field cone (|S| ≤ cA/4 + μN, N ≥ 0) and certifies all six as Stands
  (t 22 to 35 on the 10 × 8, the cap on the one-course walls), and the combined authority already
  returns Stands for all six through it (0.5 s each on the 10 × 8). Hand checks agree: in-plane
  overturning at λx 2.65, out-of-plane rocking at t/H 0.158 (the λy 0.15 points sit 5 % under it),
  base-bed cohesive sliding about 10× at λx 1.2; the evidence is thin only under a strict
  no-tension reading, where an end-of-wall stepped wedge rocks freely at λx ≈ 0.96 because
  cohesion at N = 0 is what holds it, which the project's lower-bound standard and the catalogue's
  zero-precompression intercept (EC6's fvk0) accept. Candidates measured on the six, the
  140-point grid, a 984-fixture cohesive sweep (92 reaching the refusal), the suite, Slow and
  the HUNT pins: (a) re-close Stuck cycles with period 4 on the 1 × 3 (the 2e cycle again; the
  candidate CURRENT_STATE named is not sufficient) and caps the 10 × 8 at (1.2, 0); (b) count
  the re-closure, and (c) reset the reference at it, do better (the best, (c), stands the two
  one-course walls in 11, the 10 × 8 at (1.0, 0.15) and (1.2, 0.05), grid 130 / 9 / 1) but every
  candidate turns fast refusals (mean 7.1 iterations) into 50-iteration Undecideds on about a
  fifth of the sweep (mean 22) and turns two grid Undecideds into Mechanisms the certificate
  refutes. **Ruling: the refusal stays.** It is honest fail-closed, the host answer is right
  today, and every rule trades a fast non-answer for a slow one or a false label; the restore
  step's rulings (cohesive restore re-closures neither count nor reset; Open in the full-rank
  state reads Undecided) are unchanged. **The finding it exposed is logged as its own item:** on
  cohesive tensionless walls the elastic authority's Mechanism is not a mechanism under the
  project's own standard. Its Stands credits cohesion at N ≥ 0 but its Mechanism treats an open
  cohesive corner as free, so at λx 0.96 to 1.2 its label depends on the path: the 7 Mechanisms
  the lime grid pins (5 × 5 and 6 × 8 at λx 1.2) are all one 9-body stepped wedge sliding on 36
  open cohesive corners with dissipation 272× the load power, all 7 certified Stands (t 37 to
  48), and HUNT 96643's Mechanism is the same (239×, certified Stands). The combined authority
  overrules each (no host harm) and counts a Disagreement. The candidate is to make the
  restore step's admissibility test for a Mechanism ask whether the motion's power exceeds the
  cohesive dissipation on the open corners it slides (an upper-bound check the certificate
  already does), measured first. (Corrected by the next entry: the seven are two wedges, and
  the check is a necessary condition on one motion, not the certificate's upper bound.)
- *2026-10-12, orchestrator, the cohesive-wedge Mechanism (opening):* a high-effort probe
  measured every elastic Mechanism on a structure with a cohesive joint: the lime grid (7), HUNT
  96643, a rebuilt 984-fixture sweep (24), the follow-up review's 100,000-wall random hunt (3,743)
  and dry, mixed and bonded families of about 3,300 each. **Physics:** with cohesion credited as
  the intercept at N = 0, an open corner that slides costs c·A/4·|slip| under either flow rule
  (associative: the cone apex; non-associative: an open corner at N = 0 can absorb up to cA/4
  per unit slip); only the reading in which cohesion breaks at the tension cut-off (3DEC and
  softening models) makes it free. The project ruled the intercept reading (slice 2c: a cohesive
  tensionless corner carries its cohesion; 2e: up to cA/4 + μN at any normal force; slice 3:
  cohesion is lost only by the cascade's crack; slice 4a's inscribed region; the restore-step
  entry's acceptance of EC6's fvk0), so a Mechanism whose motion slides open cohesive corners
  against more dissipation than the load supplies is a false label, and the certificate's Stands
  there is right. The strict reading would rock the end wedge free at λx ≈ 0.96 but would also
  void the elastic Stands and the certificate: a model change, not this fix. **Scale, corrected:**
  the grid's seven are two wedges (the three 5 × 5 points slide 28 open corners at 387× the load
  power, the four 6 × 8 points 36 at 272×), HUNT 96643 slides 16 at 240× with t 22.7, and the
  hunt has 856 certificate-refuted Mechanisms per 100,000 random lime walls, not eight. Of the
  hunt's Mechanisms: with no cohesive dissipation (Dc = 0) 583 Collapses and none refuted; with
  Dc below the load power P, 1,378 Collapses and 195 refuted, every one sliding an open cohesive
  corner with less opening than μ times the slip, the non-associative gap the theorem ruling
  already gives the certificate (there the elastic motion may be the physical answer); with
  Dc ≥ P, 909 Collapses and 661 refuted. No closed corner moves in any of them. **Candidates:**
  (a) refuse to Undecided when P ≤ Dc; (b) re-close the sliding open cohesive corners Stuck and
  continue (three variants); (c) open cohesive corners keep their tangential stiffness; (d) hold
  them tangentially and continue; (e) (a) after searching the null space for a cohesion-free
  motion. Only (a) and (e) change no host verdict (the combined authority consults the
  certificate for Undecided as for Mechanism); (b) to (d) turn 97 to 5,918 hunt verdicts into
  elastic Overstressed and new crack rounds, sometimes where the certificate says Stands; (c)
  churns the lime pins. (a) and (e) move no dry or bonded result (by construction: an open dry
  corner's capacity is zero; and by measurement over about 125,000 fixtures), cost nothing
  measurable (hunt elastic 76 to 79 s against 74 to 78 s), and take the hunt's refuted
  Mechanisms from 856 to 195, so after them a Disagreement means the non-associative gap and
  nothing else. (e) recovers 25 hunt and 15 mixed Mechanism labels for about 40 more lines and
  an extra factorisation per refusal, and adds 3 refuted Mechanisms in the mixed family.
  **Ruling: (a).** When the restore step finds nothing to restore, the elastic authority
  compares the load power on the null-space motion (`AssembleLoad · motion`, held friction
  included) with the cohesive dissipation on the open corners of joints that do not release
  slip that the motion slides beyond the admissibility tolerance (min(c, shear cap)·A/4 times
  the tangential slip per corner); unless the power strictly exceeds it the component reads
  Undecided with the full-rank readings (a NaN fails closed). It is a necessary condition
  checked on one motion, not the certificate's upper bound: 909 refused hunt motions are
  certified Collapses, and the combined authority gets them right. Pins: the lime grid reads
  128 Stand, 0 Mechanism, 12 Undecided (the 5 known plus the 5 × 5 at λx 1.2 with λy 0.05,
  0.1, 0.15 and the 6 × 8 at λx 1.2, all λy); HUNT 96643 reads Undecided after 13; the
  two-brick stack overturned by a biaxial load (c 10 MPa, ft 0) reads Undecided from the
  elastic authority (its projected motion spins three open corners at 3.3e4× the power) and
  Collapses from the combined, and a twin at c 100 Pa keeps the elastic Mechanism label
  (Dc/P 0.33); a 5 × 5 lime-friction wall at λx 1.3 reads Mechanism at c 300 Pa (Dc/P 0.49),
  Undecided at 1 kPa (1.63; certificate Collapses) and Undecided at 10 kPa with the combined
  Stands and no disagreement. Logged, not ruled: 5 of 458 dry random-wall Mechanisms carry a
  Stuck corner moving 2e-6 to 2.4e-5 of the motion scale (null-vector residue near the pivot
  tolerance).
- *2026-10-12, orchestrator, the cohesive-wedge Mechanism (dev notes):* landed as ruled in
  `CohesiveDissipationW` and a check at the top of `SolveProblem`'s `Restoration::None` block;
  the power is `AssembleLoad · Motion`, positive by construction (the motion is the orthogonal
  projection of the load onto the null space, so the power is the projection's squared norm);
  a refusal reports the readings the restore step's Undecided reports (full-rank readings when a
  full-rank state exists, else the current ones) and fills no motion. Measured: the four red
  tests green at the ruling's readings, the green twins (the c 100 Pa stack, the c 300 Pa wall)
  still Mechanism, 401 Release / 371 Debug after merging slice 2b, `Slow.*` 14 green, no pin
  moved. Mutants: the dissipation forced to zero reddens the four; skipping the check when a
  full-rank state exists reddens the same four; slice 2e's clear-on-open still moves HUNT
  96643 (Undecided after 15 against 13); `>` to `>=` survives (no fixture sits on P = Dc; the
  only difference is a dissipation-free zero-power motion, which the neutral-load test already
  removes); dropping the slip-holding filter survives as the equivalent mutant (a dry corner's
  capacity is zero).
- *2026-10-13, orchestrator, the cohesive-wedge Mechanism (cold-review rulings):* the review
  confirmed the physics (the certificate charges at least c·A/4·|slip| at an opening sliding
  cohesive corner and much more if it opens by less than μ times the slip, so the elastic check
  is a necessary condition strictly weaker than the certificate on the same motion, and a
  refusal only ever sends the component to the certificate), the code (held friction's work on a
  slipping corner that reaches the check is never positive, cracked joints are skipped through
  their zero effective cohesion, bonded joints never open), and the ruling's central claim: over
  127,009 fixtures old against new, 0 combined verdicts change, every elastic change is
  Mechanism → Undecided on a structure with a cohesive joint, iteration counts are identical and
  unchanged fixtures' readings bit-identical; the hunt's Disagreements 856 → 195, cracked walls
  191 → 93, mixed 76 → 34, dry and bonded unchanged. Blocks, all fixed: the held-friction term
  and the shear-cap minimum had no test (`HeldFrictionOnACrackedJointCountsAgainstTheLoadPower`,
  `ACappedCohesiveJointChargesItsShearCapNotItsCohesionForAnOpenSlide`, both killing their
  mutants), API.md said nothing about the check (now in the Mechanism and Undecided bullets),
  and an ANCHORS row was stale. Also added: `ACrackedCohesiveStackOverturnsLikeADryOne` (a
  cracked joint dissipates nothing). Equivalent clauses kept and recorded: the slide tolerance
  and the open-only filter (no fixture distinguishes them; they guard against null-vector
  residue). The dev notes' reason for the surviving `>=` mutant was wrong: the difference is any
  motion with P exactly equal to Dc, not only a zero-power one; on the 5 × 5 lime wall at λx 1.3
  P is independent of c and Dc linear in it, and a ±200-ulp scan around the crossing at
  c ≈ 612.92 Pa never lands on equality, so a pin would need two inputs tuned to the last ulp
  and would break on a harmless reordering of the sums: not pinned. Logged: every refused
  component reports the last full-rank readings, which differ from the Mechanism path's; in the
  hunt 358 of 1,579 refused components go from a finite maximum utilisation to +inf (168 on
  Collapses components, where the cascade copies the axis and ratio into `FailedInterface`).
- *2026-10-13, orchestrator, the 5-diamond with friction and the false-Stands class (opening):*
  a high-effort probe answered the slice 2b question with independent methods: an explicit 3-D
  rigid-block DEM on the library's own interfaces (corner springs, Cundall–Strack Coulomb
  friction, no tension, fixed topology; controls: the 4-diamond stands, the frictionless
  5-diamond falls, a brick on an incline slides at atan μ to 0.5°, 80 of 80 certificate
  Collapses fall), the exact planar associative LP, and the Gilbert–Casapulla–Ahmed iterative LP.
  **The 5-diamond at μ 0.5 stands:** in 15 DEM variants (release, damping, kt/kn, stiffness,
  tilt, friction scatter), with the DEM's critical friction in [0.420, 0.425] against the LP's
  0.421995 and its critical tilt matching the LP to the third decimal at μ 0.5, 0.6 and 0.8; the
  wings' friction sits on bed joints loaded by gravity, so no dilatancy is needed and the
  non-associative gap is nil here. The combined Stands is right; the certificate switches between
  0.42 (Undecided) and 0.45 (Stands). The elastic authority reads Undecided at every μ above 0
  (here the restore step's refusal at iteration 12; no iterate was ever admissible). **The probe
  found the real false-Stands class instead.** (1) Elastic Mechanism, certificate Stands: the
  suite's `Statics.Combined.ThreeBrickBridgeStandsWhereAnAdmissibleFieldExists` (μ 5, λ (2.5,
  0.875) and two siblings) falls in every DEM variant, the right base brick rocking and the top
  brick following, and the elastic Mechanism tracks the DEM's boundary (λy 0.79 to 0.80 against
  0.78 to 0.80) while the certificate stands to about 0.9; in the dry random families 4 of the 5
  such cases fall (the fifth only under sudden release). The theorem ruling's premise, that a
  Mechanism under a certificate Stands is a slice-2 defect, is refuted for this class: under
  Coulomb friction the certificate's field can rely on associative dilatancy (a worked wall: ground
  friction clamping a perpend that then carries a push by friction). (2) Elastic Undecided,
  certificate Stands: 36 of 152 gap cases in the dry wall family and 4 of 135 in a dry stack family
  fall in the DEM (26 of the 36 also under staged loading); no cheap discriminator separates them
  (a friction margin, the elastic exit kind, frictionless perpends: each loses many cases that
  stand or flips the diamond). No robust DEM fall ever had an admissible elastic iterate; 51 gap
  cases had one and all 51 stand. **Ruling.** (1) Where the elastic authority reads Mechanism and
  the certificate Stands, the combined authority reports **Undecided**, not Stands: nothing
  released (the release set stays the certificate's), the disagreement still counted. This
  amends the theorem ruling for that one case: static admissibility stays the standing criterion
  where the elastic authority found no mechanism, and an elastic mechanism the certificate cannot
  refute by a kinematic bound is no longer overruled into a Stands. Measured cost: one suite
  test (the bridge, re-pinned to Undecided), the Slow tier green; in the families it catches 4 of
  the 5 DEM falls and loses one quasi-static stand (corrected by the cold-review rulings below:
  3 robust falls, 1 damping-dependent, 1 sudden-only). Host effect: Undecided and Stands both
  release nothing, so the bridge does not fall in the host either way; the label stops claiming
  it stands. (2) The Undecided-over-Stands residual is logged as a known unsafe gap, like the
  wedge band, with its measured rate. (3) Logged candidates, not ruled: releasing the elastic
  mechanism where the certificate stands (the evidence supports it on the bridge, but it breaks
  the one-release-set ruling and rests on one probe DEM; it would need the DEM, or another
  non-associative referee, as a test-tier referee first); a load-stepped elastic path (what the
  DEM does, and it tracked every boundary measured); accepting an admissible iterate as Stands
  (the lower-bound theorem allows it, and it would settle 51 gap cases that all stand). Pins:
  the bridge reads Undecided with one disagreement; the 5-diamond's certificate switch two-sided
  around μ 0.422 and its combined Stands at μ 0.5 with the DEM's evidence recorded; the probe's
  sources and data are in the session scratchpad, not the repository.
- *2026-10-13, orchestrator, the false-Stands rule (dev notes):* one line in `Consult`: on a
  certificate `Stands`, `bDisagreed` is the elastic Mechanism test and the verdict is Undecided
  when it holds, Stands otherwise; `Kinematic` and the load factors are reported as before and
  nothing is released. `StructuralAuthority.h`'s `bDisagreed` comment and API.md (the theorem
  ruling, the composition table, the Mechanism bullet, the `Disagreements` comment) say so.
  Measured: 407 Release / 377 Debug, `Cascade.*` 38 green (no cascade test assumed a Stands
  there), `Slow.*` 14 green. Mutants: Undecided also over an elastic Undecided reddens seven
  tests (the bridge's no-mechanism sibling, the 5-diamond at μ 0.5, the bridge's provable points,
  the refereed-Undecided test, the Orduña bracket, two lime tests); dropping `bDisagreed` reddens
  the bridge and the dry wall.
- *2026-10-13, orchestrator, the false-Stands rule (cold-review rulings):* the review confirmed the
  code (the line does what API.md says; every non-equivalent mutant is caught; the cascade's
  `Release` acts only on Collapses and `CrackBand` only on Overstressed, `FindClean` reuses a report
  verbatim, so the new Undecided changes only the final `CascadeState`), the diamond (an
  independent planar LP gives the 5-diamond's critical friction 0.4219954 and the 6-diamond's
  0.8093690; the DEM's controls reproduce; 80 of 80 certificate Collapses fall) and the rule's
  scope (34,300 judgements old against new: 80 changes, every one a single Mechanism-over-Stands
  component going Stands → Undecided with nothing released either way). It refuted three of the
  ruling's evidential claims, all corrected here and in API.md and ANCHORS: **the bridge** falls in
  every DEM variant only at (2.5, 0.875); its siblings at λx 2.375 and 2.25 fall only under light
  damping and stand at the catalogue's own kt/kn 0.4, arrested at about 2 mm, and the DEM's
  boundary moves from λy 0.78 to 0.84 with damping, so "the elastic Mechanism tracks the DEM
  boundary" holds only at ζ 0.05; **the dry walls**: 3 of the 5 fall robustly (the pinned 2592 in 16
  of 16 variants), one only under light damping or sudden release, one only under sudden release;
  **the scope**: 71 of the 80 changed verdicts are on cohesive structures (37 hunt, 34 mixed), which
  the probe DEM cannot judge (it reads but never applies cohesion or tension), so most of the rule's
  reach has no referee. The rule stands as a ceiling: where the elastic authority finds a mechanism
  and the certificate's Stands may rest on dilatancy, the honest answer is that the library does
  not know, which is Undecided. The review notes the DEM's fixed contact pairs carry a gap error
  of the order of displacement × rotation, comparable to the elastic penetration at the
  siblings' 2 mm arrest, so the DEM cannot settle them either way. **Consequence for the logged
  candidate "release the elastic mechanism":** argued against for now; the evidence does not
  support releasing where the DEM itself is split, and a DEM used as a test-tier referee would
  need contact pairs that follow the geometry. Also from the review: a both-Stands knife-edge
  stack (blocks resting with a centroid exactly over a support edge) falls in the DEM by
  exponential growth from 3e-6 m; the ruling's "no robust DEM fall had an admissible elastic
  iterate" covers the gap cases only, and a Stands carries no geometric-margin signal. Logged: a
  seeded composition-table property (every consulted component's verdict follows API.md's table
  for its elastic and kinematic pair, `bDisagreed` exactly on Mechanism over Stands, lists empty
  unless Collapses; hunt draws 559 and 636 hit the case); a Collapses with an empty release set is
  caught only by the new pins, not by any cascade test.
- *2026-10-13, orchestrator, the load-stepped elastic path on the unsafe gap (probe ruling):* a
  high-effort probe built a stepped path on an instrumented copy (corner statuses, held forces,
  transverse tangents and a plastic-slip reference on every dry corner carried between steps,
  backward-Euler return mapping within a step, halving on a failed step down to 1/640) and
  measured it on the 287 gap cases against a staged DEM battery (5 variants per case). Findings.
  **A proportional ramp is the one-shot solve:** dry Coulomb joints without tension are
  homogeneous in the load, so the ramp's first step is the whole problem (Undecided persists to
  λ 0.0016; on 1,202 one-shot Stands it read 12 Mechanism and 8 Undecided). Only a history
  (vertical gravity first, then the lateral and host loads in steps) carries information. **On
  dry walls that history separates falls from stands on its own path:** of 25 robust staged DEM
  falls it reads 5 Mechanism and 20 Undecided-with-a-failed-step, none Stands; of 122 robust
  staged stands it keeps 107 Stands (15 lost); step counts 5 to 40 are stable; against the
  ramp-loaded DEM classes it catches 28 of 36 falls (the 8 it stands do not fall robustly on its
  path; the DEM is itself path-dependent, 7 ramp falls are staged stands). **On stacks and any
  gravity-only loading it does nothing** (122 of 135 gap stacks fail in phase 0, the one-shot
  problem; 0 of 22 falls caught, 10 stands lost); the 5-diamond is unchanged. **Scope and cost:**
  it never stands a certificate Collapses; switched on everywhere it reddens 36 suite tests
  (path-dependent readings, the factor pins, phase-0 artefacts, cohesive Mechanisms turning
  Overstressed) and costs 17× the one-shot on dry walls (the shed 59 to 583 ms, the warehouse
  0.8 to 8.4 s); run only where the one-shot reads Undecided under a certificate Stands it costs
  about 77 ms per gap component (6× the certificate) and nothing on any Stands; it turns
  sparse-against-dense rounding into different Mechanism body lists on 6 draws. **Ruling: not
  adopted now.** Its benefit is a label (Undecided and Stands both release nothing; the
  Mechanisms it finds sit under a certificate Stands, which the false-Stands rule already turns
  into Undecided without a release), its referee is a DEM with fixed contact pairs, its load
  history (gravity first) is a modelling decision the host has not made, it reaches only dry
  walls (cohesive gap cases have no referee, and there it turns Undecided into Overstressed,
  which would change the cascade), and it breaks determinism between the sparse and dense paths.
  It stays the leading candidate for the dry-wall part of the gap, with these four
  prerequisites. Also confirmed: accepting an admissible iterate as Stands loses nothing on the
  dry gap (saves one stand), and 8 knife-edge stacks the certificate stands with zero friction
  fall robustly in the staged DEM: geometric instability, which no friction path addresses. The
  probe's sources and data are in the session scratchpad (`stepped\`), not the repository.
- *2026-10-13, orchestrator, the certificate's interior-optimum and stall defects (opening):* a
  high-effort probe of the slice-4 walls found that the Foraboschi wall's Undecided bands (t_in 0,
  t_out at the cap, non-monotone at 140 kN, Undecided at f_c 1.21 MPa with no push) are inscribed
  solve failures, not the polygon band or the contract. **Defect A:** when the inscribed optimum is
  finite and strictly inside (1, cap) (a finite crushing strength makes it so: the dry profile's
  1e9 Pa on this wall gives t* 212 to 827), the normal matrix loses positive pivots near
  convergence, the regularised retry adds 1e-12 of the largest diagonal, one refinement pass cannot
  undo it, and the primal residual floors near 1e-3 while the dual and gap converge; the stall rule
  ends the solve. Moving t* to the cap or above (cap 100, or f_c 1e12) removes it. Both the dense
  and sparse paths fail the same way. **Defect B:** past a threshold the stall rule fires while a
  t* = 0 solve still progresses (τ falling, the gap measure rising, then t drops to 0 and the solve
  converges in 5 more iterations; the Minho wall at 16.6 kN converges at 44 with no stall rule).
  **Independent check:** a 2-D associative LP with exact cones, optional crushing and its own
  simplex (certified by primal and dual objectives, sharing no code) gives the full Foraboschi wall's
  associative limits 26.28, 50.28 and 83.61 kN at 40, 80 and 140 kN; the probe's two fixes together
  make the certificate stand to within the G = 16 band (under 0.5 kN) of each and collapse past it,
  sparse and dense identical, every one of 31 sub-threshold points converged. **The Minho wall:**
  the certificate is exact for the anchor agent's fixture (massless beam, push at the beam centroid
  z 1.05 m: 15,542.1 N by hand, 15,542.13 by the 2-D LP); the paper's value 16.55 kN cannot exceed
  the associative limit of its own model, which rules that set-up out; the push at the masonry top
  (z 1.0 m) with the 500 N beam's weight, the bottom course five full blocks and the beam free to
  rotate gives 16,569.2 N by hand (stepped wedge rotating about the toe, 18 heel blocks sliding up at
  the dilatancy angle; pure separation, so non-associative equals associative), inside 16.55 ± 0.2.
  **Rulings.** (1) Fix defect A: escalate the retry shift through 1e-16, 1e-14 and 1e-12 of the
  largest site diagonal (excluding the coupling term c·d²), taking the first that gives positive
  pivots, and run up to 20 iterative-refinement passes on the predictor and corrector in an
  iteration that used a shifted factor (measured: 1e-12 with 20 passes fixes none of the points,
  1e-16 with 20 fixes all). (2) Fix defect B: the stall window also restarts when the
  complementarity (xᵀs + τκ)/(n + 1) halves. (3) Red tests on analytic values, no LP referee in the
  suite (slice 7 stays dropped): the 23-block wall (the Foraboschi bond, 1.04 m by 5 courses, no
  opening, the beam, 80 kN vertical, f_c 1e8 Pa) at the loads that read Undecided today must stand,
  its exact limit being the beam sliding on the top course, H* = μ(V + W_beam); and the Minho wall in
  the paper's set-up stands at 16.56 kN and collapses from 16.575 kN. (4) B-SW-01 is pinned on that
  set-up. (5) The stall pins in `CertificateStallTest` are re-measured under the ruling (the ec6
  1e-15 wall still stalls, at 69/70 instead of 45/46). **Not in this slice:** the Ferris & Tin-Loi
  wall at G = 64 reads Undecided within about 1e-6 relative of its threshold (the admissible set
  nearly empty, t* at the cap; neither fix cures it): a resolution limit written into API.md, fail
  closed; the selection LP failing on the Foraboschi wall at 140 kN / 85 kN after both cone solves
  converge; and the Foraboschi fixture's associative limits at 80 and 140 kN sitting 25 to 47 % above
  Portioli's associative-to-non-associative ratio, so those loads are not pinned (the fixture's rigid
  beam with a point load is suspect). The probe's sources (the 2-D LP, the harness, the patch) are in
  the session scratchpad (`certwalls\`).
- *2026-10-13, orchestrator, a severed joint's axis (ruling and amendment):* a joint severed with a
  cluster reported the elastic reading's axis, which on a mechanism means little (the arch's crown
  hinge read `Compression 2.4e-5`, an overturned brick's bed `Shear 0.4`). Ruling: the axis comes
  from the component's selected mechanism, the relative velocity of the two bodies at the joint's
  four corners (a body outside the mechanism still): `Tension` if the joint opens, else `Shear` if it
  slides above 1e-3 of the fastest corner's relative speed, else the reading's axis; `Ratio` stays
  the reading's utilisation. **Amendment (dev round):** the certificate's flow is associative, so a
  sliding corner also lifts by μ times its slip, and the literal rule read a sliding brick's bed as
  `Tension 1.05`; the opening counted is the separation less that dilatancy, Normal − μ·|tangential|
  with the profile's friction coefficient, compared with the same threshold. Pins: an overturned
  brick's bed reads Tension, a sliding brick's Shear (a guard, green before), and all seven hinges of
  the arch below least thickness read Tension. Mutants caught: the separation's sign, a zero
  threshold, Tension and Shear swapped, NaN motion outside the mechanism, and dropping the dilatancy
  term (the sliding guard reads Tension).
