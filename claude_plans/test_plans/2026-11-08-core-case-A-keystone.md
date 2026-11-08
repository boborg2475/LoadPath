Status: proposed 2026-11-08, awaiting the owner's sign-off; no code built

# Core case A: keystone over a gap (test plan)

Owner's text ([decisions/2026-11-08-forced-collapse-and-core-suite.md](../decisions/2026-11-08-forced-collapse-and-core-suite.md) §4, A): one
keystone holds a span. With it in place the structure stands near its limit (highest utilisation
just under 1). Without it the span bricks fall. Two variants: a wedged arch with a wedge keystone
(A1), and a flat course with the keystone in the middle (A2). Mortared joints. The gap is widened
until the real mortar alone cannot carry the span.

Written against `solver_rework` at a896864 (forced collapse merged, `cascade.undecided_resolution`
= `collapse`). Nothing was built or run for this plan. Every "today" below is a prediction from the
code as read, not a measurement.

## 0. Inputs, and how the engine reads a mortared joint

**Catalogue rows used** (`data/catalogue.json`, nothing else):

| Row | Values used |
|---|---|
| `clay_brick` | ρ = 2000 kg/m³ |
| `clay_gp_mortar_perpend_ec6` (A2, vertical joints) and `clay_gp_mortar_bed_ec6` (A1, radial ring joints) | f_t = 0.0333 MPa (f_xk1 / 3, direct), c = 0.30 MPa, μ = 0.4, f_c = 8.93 MPa, cap 1.30 MPa, k_n = 119 GPa/m, k_t = 47.6 GPa/m (k_n / k_t = 2.5). The two rows carry the same numbers |

Brick 0.215 × 0.1025 × 0.065 m, mass 2000 × 0.215 × 0.1025 × 0.065 = 2.864875 kg, weight
P = 2.864875 × 9.80665 = 28.0948 N. Joint piers and abutments are grounded bodies (they stand in for
brick piers, the way case D's fixed wall stands in for a wall), so the case tests the span alone.

**The four-corner joint** (`src/RbsmStandingAuthority.cpp`, `MakeJoint`, `RatiosOf`). Each corner
carries a quarter of the joint area: tension capacity f_t A/4, cohesion c A/4, compression f_c A/4.
A bonded corner never opens: a mortared joint is linear elastic until the cascade cracks it. For a
joint of area A and depth h (corner lines h apart) carrying normal force N (compression positive)
and moment M about its centre:

- corner normal force = N/4 ± M/(2h);
- tension ratio = (M/(2h) − N/4) / (f_t A/4) = (2M/h − N) / (f_t A), which is positive only when
  the eccentricity e = M/N exceeds **h/2**: the four-corner joint's kern is the whole section;
- with N = 0 the joint breaks at M_cap = f_t A h / 2 (the rigid-plastic hinge of DESIGN §10, slice 5 close).

**Both variants use the same joint**, A = 0.215 × 0.1025 = 0.0220375 m², so every corner has
f_t A/4 = 183.65 N in tension and c A/4 = 1652.8 N in cohesion. Shear and compression never govern
in this case (worked below); every failure is on the **Tension** axis.

**Cascade rules that shape the expectations** (`src/Cascade.cpp`, `CrackBand`): only bonded joints
with ratio **> 1** are eligible; of those, the ones at ≥ R_max × (1 − `crack_band_fraction`) crack
in the same pass, R_max taken per Overstressed component; today the fraction is 0.5. A crack keeps
compression and friction and drops tension and cohesion. A released component falls as clusters
joined by live interfaces, and cracked joints are live, so a released component is one cluster
(no hinge split, decision §7). An Undecided component is released whole with `Forced` true (§1,
built in a896864).

## 1. Fixtures and hand statics

### A2, flat course: a soldier course between two piers

**Geometry.** n soldiers (bricks on end, 0.215 tall, 0.1025 deep into the wall, 0.065 along the
span), touching face to face between two grounded piers, every joint the EC6 perpend row. The span
is L = 0.065 n m, the course top flush with the pier tops, nothing under the course. The keystone
is the middle soldier, s.(n−1)/2. Joint j (j = 0..n) is the joint with j soldiers to its left:
joint 0 is `pier_left/s.0`, joint n is `s.(n−1)/pier_right`.

Joint depth h = 0.215 m (the soldier's height), spacing s = 0.065 m.
P s = 28.0948 × 0.065 = 1.82616 N·m; M_cap = f_t A h/2 = 33,333.3 × 0.0220375 × 0.1075 = 78.968 N·m.

**With the keystone: a fixed-ended beam of identical joints.** Bonded joints cannot open, so the
course is a beam fixed into both piers. Its axis is straight and the load is vertical, so in a
linear solve no axial force appears (N = 0 at every joint; there is no thrust while the course is
bonded). By symmetry each pier carries V = nP/2, which leaves one unknown, the end moment M_0.
Each joint's relative rotation is M_j / k_rot with the same k_rot = k_n A h²/4 at every joint,
and the two fixed ends need Σ θ_j = 0, so M_0 is the mean of the simply-supported moments over the
n + 1 joints. The simply-supported moment at joint j is M_ss(j) = (P s/2) j (n − j) (the same as a
uniform load at joint positions), and Σ_{j=0..n} j (n − j) = n (n + 1)(n − 1)/6, so

    M_j = P s [ j (n − j)/2 − n (n − 1)/12 ]           (negative = hogging, tension at the top)
    M_0 = P s n (n − 1)/12                              (at both piers, the largest for n > 3)
    ratio_0 = M_0 / M_cap = ρ g T² n (n − 1) / (6 f_t H) = 0.00192712 n (n − 1)

with T = 0.065 m (along the span) and H = 0.215 m (the joint depth); the brick's depth into the
wall cancels. Shear deformation does not enter: with V fixed by symmetry the only compatibility
condition is the rotation one, and a joint's corner shear depends only on its translation. So the
engine's reading should equal this formula to solver precision.

| n | L (m) | ratio at the piers | |
|---|---|---|---|
| 21 | 1.365 | 0.00192712 × 420 = **0.8094** | well inside |
| 23 | 1.495 | 0.00192712 × 506 = **0.9751** | the limit fixture: just under 1 |
| 25 | 1.625 | 0.00192712 × 600 = **1.1563** | the mortar breaks |

Other axes at n = 23, joint 0: corner shear V/4 = 11.5 × 28.0948/4 = 80.8 N against c A/4 = 1652.8 N
plus friction, shear ratio ≤ 0.049. Corner compression M_0/(2h) = 77.00/0.43 = 179.1 N against
f_c A/4 = 49,198 N, ratio 0.0036. **Tension governs by a factor of 20.**

**Without the keystone (n = 23, s.11 removed): two cantilevers of m = 11 soldiers.** Each half is
statically determinate. The moment at its joint j is P s (m − j)²/2, so

    ratio_j = ρ g T² (m − j)² / (f_t H) = 0.0115627 (m − j)²
    j = 0: 1.3991    j = 1: 1.1563    j = 2: 0.9366    j = 3: 0.7400 ... j = 10: 0.0116

Only joints 0 and 1 exceed 1, and joint 1 (1.1563) is above the band edge 1.3991 × 0.5 = 0.6995,
so pass 1 cracks both on each side. With joint 0 cracked the half has no tension at the pier, and
its weight acts 0.3575 m out from the pier face (5.5 soldiers × 0.065 m), so it rotates about the
bottom corners of joint 0. That is a pure rotation about one corner line, a singular stiffness, and
an elastic Mechanism. Pass 2 releases each half as one cluster, physically correct here as well:
the half falls as a rigid piece because joint 1 does not need to open.

**Why the cracked course cannot arch (n = 25, keystone in place).** Pass 1 cracks only joints 0 and
25 (joint 1 reads |12 − 50| × 0.0231255 = 0.879, not eligible). The cracked ends now act as pins at
their bottom corners, and the elastic compatibility of a beam pinned at its bottom fibre gives a
thrust

    H = (2/h) Σ M_ss(j) / (n_bonded + n_axial) = (2/0.215) × 2374.0 / (24 + 28) ≈ 425 N,

with Σ M_ss = (P s/2) n (n + 1)(n − 1)/6 = 0.91308 × 2600 = 2374.0 N·m. Each cracked end has to
carry V = 12.5 P = 351.2 N by friction alone, against μ H = 0.4 × 425 = 170 N, so the ends slip. Even
the best case for arching, a three-hinge flat arch with lever arm h, gives H = W L/(8h) = 663.6 N and
μ H = 265 N < 351 N. A flat arch with vertical skewbacks slides whenever 4h/L > μ, which is the
published flat-arch result (F-ARC-02, Casapulla 2001, BENCHMARKS.md). Here 4h/μ = 2.15 m, so every
span in the sweep (L ≤ 1.625 m) slides once its pier joints crack. Dry or cracked, the course stands
on mortar alone, so the sweep's edge is exactly where the mortar fails. **Sweep limit:** the sweep
must stay below n = 33 (L = 2.145 m), or a cracked course could arch on friction.

### A1, wedged arch: a half-brick ring of gauged brick voussoirs

**Geometry.** A semicircular ring of N_v = 71 voussoirs (odd, so v35 is the keystone at the crown),
radial depth t = 0.1025 m, barrel depth b = 0.215 m (a rowlock ring in a one-brick wall),
centreline radius R (the swept quantity), clay_brick with exact annular-sector weights and
centroids (as `tests/ArchFixtures.h` builds them), radial joints every Δ = 180°/71 = 2.5352°, every
ring joint the EC6 bed row, springings on two grounded abutments at z = 0. Joint J_k joins v(k−1)
and v(k) at −90° + k Δ from the crown (J0 = `abutment_left/v0`, J71 = `v70/abutment_right`, the
keystone's faces J35 and J36 at ∓1.268°). At R ≈ 1.45 m a voussoir weighs
2000 × 0.215 × (Δ/2) × 2Rt ≈ 2.84 kg, about one brick, with a tangential size at the centreline of
πR/71 ≈ 0.064 m. The joint area is the same 0.0220375 m² as in A2.

**Literature anchor.** A dry semicircle needs t/R ≥ 0.1075 (Milankovitch; B-ARC-01, Cocchetti,
Colasante & Rizzi 2011). This ring sits at t/R ≈ 0.070, **below** the least thickness. Dry, it
cannot stand, and once enough hinges crack it must fall. It stands only because the mortar's
tension holds the thrust line where the ring cannot.

**With the keystone: the elastic hingeless arch (force method, continuous estimate).** Cut at the
crown. By symmetry the shear there is zero, leaving the unknowns m = M_c/(qR²) and h = H/(qR), with
q = ρ g b t the ring's weight per unit arc. At a section φ from the crown,

    M/(qR²) = m − h (1 − cos φ) + F(φ),  F(φ) = φ sin φ − 1 + cos φ
    N/(qR)  = h cos φ + φ sin φ,          T/(qR) = −h sin φ + φ cos φ

The joints are springs with EI/EA = t²/4 and EI/GA = 2.5 t²/4 (the corner model and the row's
k_n/k_t). The two compatibility conditions are zero crown rotation, ∫M dφ = 0, and zero crown
spread, ∫M(1 − cos φ) dφ − (t/R)²/4 ∫N cos φ dφ + 2.5 (t/R)²/4 ∫T sin φ dφ = 0. The integrals on
[0, π/2] are π/2, π/2 − 1, 3π/4 − 2 = 0.35619, ∫F = 2 − π/2 = 0.42920, ∫F(1 − cos) = 0.25111,
∫N cos = 0.7854 h + 0.3927 and ∫T sin = −0.7854 h + 0.3927. The discrete ring's last joint sits *at*
the springing, which the midpoint sum reads as half a joint spacing beyond it, so every integral
gains f(π/2) Δ/2 (Δ/2 = 0.02212). Solving with t/R = 0.0704 gives

    h = 0.6209, m = −0.0462, M_springing/(qR²) = m − h + (π/2 − 1) = −0.0964, N_springing = (π/2) qR

so the springing's eccentricity is 0.0964/1.5708 = 0.0614 R, on the extrados side (tension at the
springing's intrados corners). The springing tension ratio is

    ratio_s = (ρ g R / f_t)(2 e_s/t − 1)(π/2) = 0.5884 R × 1.5708 × (1.1974 R − 1)

and setting it to 1 gives **R₁ ≈ 1.456 m** (clear span 2(R − t/2) ≈ 2.81 m). Without the end-cell
correction it gives 1.39 m, and without the axial and shear terms 1.33 m. Each refinement moves R₁
by 3 to 5 %, and because the ratio is the small difference (2e/t − 1), a 3 % error in R is about a
10 % error in the ratio. **So the hand value brackets the fixture, and it cannot place it at
"just under 1".** The test will place it with an exact discrete oracle (open question 3).

Elastic readings at R = 1.456 (ρ g R/f_t = 0.8567, 2R/t = 28.41; ratio = 0.8567 (|M| × 28.41 − N/qR) where positive):

| φ from crown | M/(qR²) | N/(qR) | tension ratio | side in tension |
|---|---|---|---|---|
| 0° (crown, the keystone's faces) | −0.0462 | 0.621 | **0.59** | intrados |
| 17° | −0.0300 | 0.682 | 0.15 | intrados |
| 34° to 46° | 0.009 to 0.036 | 0.85 to 1.01 | 0 to 0.01 | thrust line inside the ring |
| 57° (haunch) | +0.0501 | 1.177 | **0.21** | extrados |
| 69° to 80° | | | 0 | inside |
| 90° (springing) | −0.0964 | 1.571 | **≈ 1.0 (by construction)** | intrados |

Other axes at the springing: corner shear H/4 = 0.621 × 629.3/4 = 97.7 N (qR = 432.2 × 1.456 = 629.3 N)
against 1652.8 N plus friction, ratio 0.06; compression ≈ 0.013. **Tension governs.**

**Without the keystone (v35 removed): two curved cantilevers.** Statically determinate. For the
part between the keystone's former face (δ = Δ/2 = 0.02212 rad) and a joint at φ,

    ratio(φ) = (ρ g R/f_t) [ (2R/t) G(φ) − (φ − δ) sin φ ],  G(φ) = (φ − δ) sin φ − (cos δ − cos φ)

At R = 1.456: springing 0.8567 × (28.41 × 0.54892 − 1.5487) = **12.03** (tension at the extrados:
the half leans into the gap). 69° 10.3, 57° 8.1, 49.4° (J16) 6.49, 46.9° (J17) 5.95, 34° 3.4,
17° 0.85. Joints above about 18° from the crown exceed 1. The band edge is 12.03 × 0.5 = 6.02, so
pass 1 cracks J0..J16 and J55..J71, 17 per side; J17/J54 sit 1.1 % under the edge, so the oracle
decides them. With the springing cracked, the half's centroid (x̄ = R cos δ/(π/2 − δ) = 0.940 m from
the axis) lies 0.46 m inside the springing's intrados corner (R − t/2 = 1.405 m). The half rotates
about that corner line, a Mechanism, and pass 2 releases each half as one cluster.

**Widening the gap: three thresholds, not one.** Once the springings crack the ring is a two-hinged
arch (pins at the springing extrados) and can still stand. The edge of "the mortar alone cannot
carry it" is where the hinge sequence ends in a mechanism:

1. **R₁ ≈ 1.456 m**: the springing mortar cracks (above).
2. **Two-hinged arch** (pins at the extrados of both springings, one redundant). Statics gives
   m = h − (π/2 − 1) − (π/2)(t/2R), and compatibility ∫M cos φ dφ = 0 (bending only) gives
   h = 0.5 + 2(t/2R) and m = −0.0708 + 0.4292 (t/2R). The crown is now critical:
   ratio_c = 0.5884 (1.38139 R² − 0.92920 R − 0.1025) = 1 at **R₂ ≈ 1.53 m**, with the haunch about
   0.65 and the springing friction H/(μN) ≈ 0.90.
3. **Three-hinged arch** (crown extrados too, statically determinate): h = (π/2 − 1 + (π/2) s)/(1 + s),
   s = t/2R. At R = 1.527 the haunch (57°) reads 1.21, so it cracks at once. That gives five hinge
   lines (both springings, both crown faces, both haunches) in a ring below least thickness: a
   Mechanism. The springing friction H/(μN) is 0.96 in this state, close to sliding (compare
   B-ARC-06: the least-thickness semicircle needs μ ≥ 0.404).

So the sweep is: **R just under R₁** stands uncracked. **R₁ < R < R₂** cracks both springings in
pass 1 and stands as a two-hinged arch. **R > R₂** falls after three crack passes (springings, crown
faces, haunches) and a release in pass 4. R₂ is uncorrected and the window R₁ to R₂ is about 5 %
wide, so the oracle places all three rows (open question 3). If the discrete oracle shows a window
narrower than two grid steps, the window row is dropped and the plan says so.

**The contrast between the variants is the point of case A.** Once its mortar cracks, the flat
course has only friction at vertical faces and slides out. The wedge keystone keeps a cracked arch
standing on geometry, until enough hinges form that the ring, being below least thickness, must fold.

## 2. Cases and expectations

Each row is a scenario file under `data/scenarios/core-a-*.json`, run by
`E2E.Scenarios.EveryScenarioFileMeetsItsExpectation`, which also records each row's pictures and
pass replay. Keys marked *(new)* do not exist in the format yet (§2.3, open question 4). "before" is
the structure as the before view draws it: removals applied, no cascade (DESIGN §10, 2026-10-17).

### 2.1 A2, flat course (files expressible today except the *(new)* keys)

**A2.1 `core-a2-soldier-course-23-stands-at-its-limit`**

Visual: core-a2-soldier-course-23-stands-at-its-limit (before: 23 soldiers between grey piers,
joints coloured by tension ratio, the two pier joints at 0.975; expected after: unchanged)

- Behaviour: a mortared soldier course of 23 bricks between two piers stands, its pier joints at
  97.5 % of their tension capacity.
- Assertion (outcome plus the mechanism reading): Stands, nothing cracked or released, 0 passes;
  highest before utilisation in [0.970, 0.980] at both pier joints, axis Tension.
- Today: predicted green except the *(new)* key. Mutant that proves it bites (Scripts/Mutate.ps1):
  `J.TensileCapacityN = Profile.TensileStrengthPa * CornerAreaM2;` → `* CornerAreaM2 * 0.97`. The
  pier joints then read 1.005, crack and fall.
- Tier: default (25 bodies).

```json
{
  "title": "Core A2: a soldier course held by mortar alone",
  "summary": "23 soldiers span 1.495 m between two piers on EC6 perpends; the pier joints read 0.975 in tension and the course stands.",
  "build": [
    { "name": "pier_left",  "size": [0.3275, 0.1025, 1.215], "centre": [-0.16375, 0, 0.6075], "grounded": true },
    { "name": "pier_right", "size": [0.3275, 0.1025, 1.215], "centre": [1.65875, 0, 0.6075], "grounded": true },
    { "name": "s", "size": [0.065, 0.1025, 0.215], "centre": [0.0325, 0, 1.1075], "material": "clay_brick",
      "repeat": { "count": 23, "step": [0.065, 0, 0] } }
  ],
  "joints": [
    { "materials": ["clay_brick", "clay_brick"], "row": "clay_gp_mortar_perpend_ec6" },
    { "materials": ["clay_brick", "ground"], "row": "clay_gp_mortar_perpend_ec6" }
  ],
  "steps": ["cascade"],
  "expect": {
    "cracked": [], "passes": 0,
    "utilisation": { "before": { "max": [0.970, 0.980], "at": ["pier_left/s.0", "s.22/pier_right"], "axis": "Tension" } }
  }
}
```

**A2.2 `core-a2-soldier-course-23-minus-keystone`**

Visual: core-a2-soldier-course-23-minus-keystone (before: s.11 gone, the pier joints at 1.40 and
the next joints at 1.16 in red; expected after: s.0–s.10 and s.12–s.22 released as two clusters,
the four cracked joints dashed, the piers alone)

- Behaviour: pulling the middle soldier turns the course into two cantilevers whose root mortar
  cannot hold them, and both halves fall.
- Fixture: A2.1 with `"steps": ["cascade", { "remove": ["s.11"] }, "cascade"]`.
- Expectation:

```json
"expect": {
  "released": [["s.[0-10]"], ["s.[12-22]"]],
  "cracked": ["pier_left/s.0", "s.0/s.1", "s.21/s.22", "s.22/pier_right"],
  "passes": 2,
  "forced": false,
  "pass_record": [
    { "pass": 1, "cracked": ["pier_left/s.0", "s.0/s.1", "s.21/s.22", "s.22/pier_right"], "released": [] },
    { "pass": 2, "cracked": [], "released": [["s.[0-10]"], ["s.[12-22]"]] }
  ],
  "utilisation": { "before": { "max": [1.390, 1.410], "at": ["pier_left/s.0", "s.22/pier_right"], "axis": "Tension" } }
}
```

- Pass recorder, as it should read: pass 1 has two Overstressed components, R_max 1.3991 at the
  pier joints, joints 0 and 1 of each half cracked in Tension, one inner iteration (bonded corners
  never change state). Pass 2 has two Collapses components, end `Mechanism`, not forced. The
  cracked pier joint's top corners flip Closed → Open in the inner iterations, and the bottom pair
  stays closed (the pivot).
- Today: predicted green except the *(new)* keys; the bonded wall pushed past its limit (E2E 3.6)
  is the same crack-then-rotate sequence and is green. Mutants: band fraction read as 0.1 (joint 1
  at 1.156 falls under the edge 1.259 and the cracked set loses `s.0/s.1`); tension × 1.5 (only
  the pier joints exceed 1).
- Band dependence: `s.0/s.1` cracks in pass 1 only while the fraction is ≥ 0.174
  (1 − 1.1563/1.3991). If D2/D3 narrow the band below that, it is released uncracked and this file's
  `cracked` and `pass_record` lose it (open question 6).
- Tier: default.

**A2.3 `core-a2-soldier-course-21-stands`** (the sweep, well inside)

Visual: core-a2-soldier-course-21-stands (before: pier joints at 0.809; after unchanged)

- 21 soldiers, `pier_right` centre x = 1.365 + 0.16375 = 1.52875. Expectation:
  `{ "cracked": [], "passes": 0, "utilisation": { "before": { "max": [0.800, 0.820], "at": ["pier_left/s.0", "s.20/pier_right"], "axis": "Tension" } } }`.
  Mutant: as A2.1 with × 0.80. Tier: default.

**A2.4 `core-a2-soldier-course-25-mortar-fails`** (the sweep, past the edge)

Visual: core-a2-soldier-course-25-mortar-fails (before: pier joints at 1.156 in red, the rest
under 0.88; expected after: all 25 soldiers released as one cluster, the two pier joints dashed)

- Behaviour: one more pair of soldiers puts the pier mortar past its tension strength. Once it
  cracks, friction at the vertical pier faces cannot carry the course and it slides out whole.
- 25 soldiers, `pier_right` centre x = 1.625 + 0.16375 = 1.78875. Expectation:

```json
"expect": {
  "released": [["s.[0-24]"]],
  "cracked": ["pier_left/s.0", "s.24/pier_right"],
  "passes": 2,
  "pass_record": [ { "pass": 1, "cracked": ["pier_left/s.0", "s.24/pier_right"], "released": [] } ],
  "utilisation": { "before": { "max": [1.150, 1.160], "at": ["pier_left/s.0", "s.24/pier_right"], "axis": "Tension" } }
}
```

- `forced` is not asserted. Pass 2 is a friction-slip mechanism (μH ≈ 170 N against V = 351 N),
  and the elastic solver reads slip mechanisms as Undecided elsewhere (B-ARC-06's elastic row
  reads Undecided at 21°). Forced or proven, the released set is the same. The pass record names
  the end reason, and a forced release here is a work-queue item, not a red (§4).
- Today: predicted green on the released set. Mutant: tension × 1.2 (the pier joints read 0.964,
  nothing falls).
- Tier: default.

### 2.2 A1, wedged arch (needs the arch builder, §2.3)

The radius of each row is set by the force-method oracle (§3) on a 0.01 m grid, and the C++ oracle
test checks that the file's radius satisfies the row's rule. Estimates: R_stand ≈ 1.45 m,
R_window ≈ 1.49 m, R_fall ≈ 1.60 m.

**A1.1 `core-a1-rowlock-arch-stands-at-its-limit`** (R = R_stand: the largest grid radius whose
oracle springing ratio is < 1)

Visual: core-a1-rowlock-arch-stands-at-its-limit (before: 71 voussoirs, joints coloured by tension
ratio: springings ≈ 0.98 to 1.0, keystone faces ≈ 0.6, haunch ≈ 0.2, the rest 0; thrust line
overlay drawn from the readings; expected after: unchanged)

```json
{
  "title": "Core A1: a mortared rowlock arch at its limit",
  "summary": "A half-brick ring of 71 gauged brick voussoirs, t/R about 0.07, below the 0.1075 a dry semicircle needs; the mortar holds the thrust line at the springings and the arch stands.",
  "build": [
    { "name": "abutment_left",  "size": [0.5, 0.215, 0.5], "centre": [-1.45, 0, -0.25], "grounded": true },
    { "name": "abutment_right", "size": [0.5, 0.215, 0.5], "centre": [1.45, 0, -0.25], "grounded": true },
    { "arch": "v", "material": "clay_brick", "radius": 1.45, "thickness": 0.1025, "depth": 0.215,
      "voussoirs": 71, "springings": ["abutment_left", "abutment_right"], "joint": "clay_gp_mortar_bed_ec6" }
  ],
  "steps": ["cascade"],
  "expect": {
    "cracked": [], "passes": 0,
    "utilisation": { "before": { "max": [0.950, 1.000], "at": ["abutment_left/v0", "v70/abutment_right"], "axis": "Tension" } }
  }
}
```

(The radius shown is the estimate; the file carries the oracle's R_stand, and the abutment centres
follow it.) Assertion: Stands, nothing cracked; highest before utilisation in [0.95, 1.0) at both
springings, Tension, and equal to the oracle's value within 1e-6 relative (the C++ oracle test).
Today: red (no arch builder). Tier: `slow_in_debug`.

**A1.2 `core-a1-rowlock-arch-minus-keystone`** (R_stand, v35 removed)

Visual: core-a1-rowlock-arch-minus-keystone (before: keystone gone, every joint above about 18°
red, springings at about 12; expected after: v0–v34 and v36–v70 released as two clusters, the
oracle's cracked set dashed)

```json
"steps": ["cascade", { "remove": ["v35"] }, "cascade"],
"expect": {
  "released": [["v[0-34]"], ["v[36-70]"]],
  "cracked": ["<the oracle's set: estimate J0..J16 and J55..J71, written out as joint names>"],
  "passes": 2,
  "forced": false,
  "pass_record": [
    { "pass": 1, "cracked": ["<the same set>"], "released": [] },
    { "pass": 2, "cracked": [], "released": [["v[0-34]"], ["v[36-70]"]] }
  ],
  "utilisation": { "before": { "max": [11.0, 13.0], "at": ["abutment_left/v0", "v70/abutment_right"], "axis": "Tension" } }
}
```

The cracked list is computed by the cantilever oracle (exact: each half is statically determinate)
with the settings file's band, and written into the file at the test step. Today: red (no
builder). Mutant: band fraction 0.1 (the set shrinks to the springing region). Tier: `slow_in_debug`.

**A1.3 `core-a1-rowlock-arch-cracks-and-stands-two-hinged`** (R_window, between the oracle's R₁ and R₂)

Visual: core-a1-rowlock-arch-cracks-and-stands-two-hinged (before: springings just over 1;
expected after: both springings dashed, nothing released, the crown faces now the highest at
about 0.9)

```json
"expect": {
  "cracked": ["abutment_left/v0", "v70/abutment_right"],
  "passes": 1,
  "pass_record": [ { "pass": 1, "cracked": ["abutment_left/v0", "v70/abutment_right"], "released": [] } ],
  "utilisation": { "before": { "max": [1.000, 1.100], "at": ["abutment_left/v0", "v70/abutment_right"], "axis": "Tension" } }
}
```

State Stands. In the final judge the springings' intrados corners are open (pins at the extrados)
and the crown faces govern below 1. Today: red (no builder). Tier: `slow_in_debug`.

**A1.4 `core-a1-rowlock-arch-folds`** (R_fall ≈ 1.05 × the oracle's R₂)

Visual: core-a1-rowlock-arch-folds (before: springings over 1; expected after: all 71 voussoirs
released; dashed springings, crown faces and haunches)

```json
"expect": {
  "released": ["v[0-70]"],
  "pass_record": [ { "pass": 1, "cracked": ["abutment_left/v0", "v70/abutment_right"], "released": [] } ],
  "utilisation": { "before": { "max": [1.200, 1.500], "at": ["abutment_left/v0", "v70/abutment_right"], "axis": "Tension" } }
}
```

`released` is a flat list (union, not a partition). The prediction, recorded in the file's summary
and asserted only if the owner wants it (open question 5): pass 1 springings, pass 2 the keystone's
faces J35/J36, pass 3 the haunch joints near 57° (several, a flat peak), pass 4 release, so
`passes` 4. `forced` is not asserted: a ring below least thickness folding at five hinge lines
reads Undecided in the elastic field (ANCHORS, `Statics.Anchor.CoupletHeymanArchStandsAboveLeastThicknessAndNotBelow`:
Undecided below the limit, not Mechanism). Today: red (no builder). Tier: `slow_in_debug`.

### 2.3 Format and harness changes these cases need (built after sign-off)

- **`arch` builder** in the scenario `build` list. It makes the voussoirs (written `v0..v70` in
  this plan; `v.0..v.70` if they follow `repeat`'s naming, open question 4) as exact annular sectors,
  with radial joints of the named row, the two end joints against the named grounded bodies, and
  boxes for the viewer (the largest inscribed box, as `BuildArch` draws them today).
- **`utilisation`** *(new)*: `{ "before" | "after": { "max": [lo, hi], "at": [joint patterns], "axis": "Tension" } }`.
  It holds when the highest utilisation over live joints lies in [lo, hi), every `at` joint is within
  1e-9 relative of it, and `axis` governs at those joints. It needs the before-state readings
  block (CURRENT_STATE, "Joint stress recorded in every test picture"); `after` reads the final judge.
- **`pass_record`** *(new)*: a list of `{ pass, cracked, released }`, each compared exactly like the
  top-level keys for that cascade pass. Passes not listed are not checked.
- **`forced`** *(new)*: `false` means no released cluster has `bForced`; absent means unchecked.
- **The C++ oracle test**, `tests/CoreKeystoneTest.cpp`:
  `Core.Keystone.FlatCourseReadsTheClosedForm` (the A2 joint ratios equal
  ρ g T² n(n − 1)/(6 f_t H) and the cantilever formula within 1e-9 relative, as formulas over the
  row's numbers) and `Core.Keystone.ArchRadiiSitOnTheOracleEdges`. The second loads the A1 files and
  checks R_stand: oracle < 1 at R and ≥ 1 at R + 0.01. It checks R_window in (R₁, R₂) and
  R_fall ≥ R₂, and that the engine's springing reading equals the oracle within 1e-6. The oracle is
  the force method on the discrete ring: two redundants (crown moment and thrust), the sums over the
  actual joints with each joint's k_rot, k_ax and k_sh, sector weights at their centroids. The
  production solver is a 6-DOF stiffness assembly with an active set, so the two share no algorithm.
  The two-hinged R₂ is the same force method with the springing joints as pins at the extrados
  corner line (one redundant). Tier: default.

## 3. Per-joint and per-brick stress readings, before and after

Per-brick value = the highest utilisation of the brick's joints, so a brick's colour is its
worst joint.

**A2.1 (n = 23, before = after):** no normal force anywhere (the bonded course carries no thrust),
so all the load is bending. Tension ratio by joint, from the pier inward, symmetric:
0.975, 0.721, 0.489, 0.281, 0.096 (hogging, top corners in tension), then the moment changes sign
between joints 4 and 5: 0.066, 0.204, 0.320, 0.412, 0.482, 0.528, 0.551 (sagging, bottom corners
in tension), with the largest sag at the keystone's two faces. Shear falls linearly from 80.8 N per
corner at the piers to zero at the keystone. Brick heat map: the pier-end soldiers hot, a cool band
about a third of the way in (s.4/s.5 and s.17/s.18), warm toward the keystone.

**A2.2 before (keystone removed):** each half reads 1.399, 1.156, 0.937, 0.740, 0.567, 0.416,
0.289, 0.185, 0.104, 0.046, 0.012 from the pier out, all hogging. After: the two halves are gone
and only the piers remain, with no joints left to read. The pass-1 record holds the cracked pier
joints with tension 0 and their compression on the bottom pair.

**A2.4 (n = 25) before:** 1.156 at the piers, 0.879, 0.624, 0.393, 0.185, 0.0, ... 0.648 at the
keystone faces. In pass 2's inner iterations the pier joints' top corners open and their bottom
corners go Slipping (the slide).

**A1.1 before (R_stand):** compression everywhere. Normal force rises from H ≈ 0.62 qR ≈ 390 N at the
crown to (π/2) qR ≈ 990 N at each springing (qR ≈ 629 N). The thrust line is near the extrados at the
crown, near the intrados at the haunch and outside the extrados centreline at the springing (0.061 R
out). Tension ratio by region: keystone faces 0.59 (intrados), 0.15 at 17°, 0 from about 25° to
50°, 0.21 peak near 57° (extrados), 0 from about 65° to 85°, 0.98 to 1.0 at the springing
(intrados). Shear ratio ≤ 0.06 everywhere. Brick heat map: two hot springing voussoirs, a warm
keystone, a faint haunch band, cold in between. That is the elastic picture of the classic
crown–haunch–springing hinge pattern.

**A1.2 before (keystone removed):** tension on the extrados rising monotonically from the gap down
to the springing (0.35 at 11°, 0.85 at 17°, 3.4 at 34°, 8.1 at 57°, 12.0 at 90°). After: nothing
but the abutments; the pass-1 record shows the cracked set.

**A1.3 after:** springings cracked (intrados corners open, all compression on the extrados pair, the
pin), keystone faces now the highest at about 0.9 (two-hinged estimate 0.95 at R = 1.50), haunch
about 0.65.

## 4. Known risks, and what each expectation does

| Risk | Where | What the expectation does | Defect or known gap |
|---|---|---|---|
| **No hinge splitting** (§7): a released component is one cluster. | A1.4 (physically five pieces: two springing-to-haunch segments, two haunch-to-crown segments, the keystone); A1.2 (each half may physically leave a stub of about 8 voussoirs, 68° to 90°, where cot φ ≤ μ = 0.4 lets a cracked chain stand, by the E2E 3.2 open-chain oracle) | A1.4 asserts the released **union** only. A1.2 asserts one cluster per half, the whole half falling, which is today's behaviour; the stub is the §7 answer and becomes a `Deferred.Hinges.*` row when §7 is scheduled. A2 rows assert the partition, which is physically right (each falling piece moves rigidly with no cracked joint opening). | Known gap, §7. A wrong *union* is a defect. |
| **Forced collapse** (§1): a slip or multi-hinge mechanism may read Undecided and be force-released. | A2.4 pass 2, A1.4 pass 4 | `forced` unasserted. The released set is the same either way, and the replay shows `ForcedReason`. | Known gap (solver work queue), recorded in CURRENT_STATE when seen. A forced release on A2.2 or A1.2 (pure rotation about one corner line, provably a Mechanism) is a **defect**: those rows assert `forced: false`. |
| **Forced over-release**: the whole Undecided component goes. | A1.3: if the two-hinged final judge read Undecided instead of Stands, the whole arch would be force-released. | Asserts Stands, nothing released. | Defect: a two-hinged arch with every joint under 1 has an admissible elastic field, and Undecided there is a solver failure (the warehouse IterationCap chatter, CURRENT_STATE item 2, is the likeliest cause). |
| **Band width**: the cracked sets depend on `crack_band_fraction` = 0.5. | A2.2 (`s.0/s.1`), A1.2 (the 17 per side) | Pinned at today's setting. A1.2's list is generated from the settings file by the oracle. | When D2/D3 change the band, these files change in the same change. Not a defect. |
| **Hand estimate precision** for A1 (±3 % in R, ±10 % in ratio). | A1.1, A1.3, A1.4 | Radii come from the discrete force-method oracle, and the file's radius is checked against it. | A gap between engine and oracle above 1e-6 at the springing is a defect in one of them; the debug playbook decides which. |
| **Springing friction near its limit** in the three-hinged state (H/μN ≈ 0.96). | A1.4 | Only the outcome and pass 1 are asserted; the pass count is a prediction. | If the springing slides in pass 3 the arch still falls and passes reads 3. Not a defect. |
| **Window too narrow** (R₂ − R₁ under two grid steps). | A1.3 | The row is dropped and the plan amended. | Physics, not a defect. |

## 5. Open questions for the owner (each with a recommendation)

1. **Which mortar row?** EC6 characteristic (f_t 0.0333 MPa, μ 0.4) or Gooch mean (f_t 0.20 MPa,
   μ 1.05)? **Recommend EC6.** With Gooch's μ = 1.05 a cracked flat course arches on friction past
   4h/μ = 0.82 m, so A2 would never fall at its mortar edge (that edge moves to about 57 soldiers,
   3.7 m). EC6 is also the row every other wall in the suite uses.
2. **Flat-course orientation?** Soldiers (on end) or stretchers laid flat? **Recommend soldiers.**
   A soldier course over an opening is real practice, and its edge falls between 23 and 25 bricks,
   so the sweep has resolution. Flat stretchers break between 3 and 5 bricks, too coarse to show
   "just under 1".
3. **A1's radius from the oracle rather than by hand?** **Recommend yes.** The hand calculation
   brackets R₁ at 1.39 to 1.46 m depending on refinements, and the ratio is a small difference.
   The file carries a number, and the C++ oracle test proves it sits on the right side of each edge.
4. **Extend the scenario format** (arch builder, `utilisation`, `pass_record`, `forced`), or write A1
   as a C++ E2E case on a generalised `ArchFixtures.h`? **Recommend extending the format**, the
   preferred path (scenario file → viewer → generic test), so all eight rows run through one test
   and draw the same way. Voussoir names `v.0..v.70` to match `repeat`.
5. **Assert A1.4's pass count (4)?** **Recommend no.** Assert pass 1 and the outcome only. The count
   hangs on the springing friction (0.96 of its limit in the three-hinged state), and the replay
   shows the sequence for study either way.
6. **Pin band-dependent crack sets now?** **Recommend yes, at today's 0.5**, with the dependence
   noted in each file's summary. D2/D3 then update A2.2 and A1.2 when they narrow the band, which
   is a useful cross-check on that change.
7. **Piers and abutments as ground?** **Recommend yes** (like case D's fixed wall). Real brick piers
   bring their own base joints and make the case about two things at once.
8. **Add dry contrast rows** (the same course and ring with a dry profile, both fall at once)?
   **Recommend no for now.** The cracked rows already show the dry behaviour, and the core set is
   meant to stay small.
