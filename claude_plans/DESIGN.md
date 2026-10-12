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
  held-force tolerance (1e-12), so it is far below any real load (a perpend under λ 0.2 on the
  1 × 3 wall carries newtons) and far above the roundoff it exists to absorb; the stuck/slipping
  comparison gets the same absolute slack so a capacity and a shear both at noise level never
  flip a corner. A genuinely loaded corner keeps the slice-2 rule unchanged, so the incline,
  bridge and wall anchors must not move. Bonded corners are untouched: cohesion carries at N 0.
