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
