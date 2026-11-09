Status: approved by the owner 2026-11-08 ("go ahead on test cases A") after the revision with their eight answers; the three questions the revision raised are decided per their recommendations (approximate oracle with a measured tolerance and a [0.80, 1.00) band on the A2 limit row; a fourth `stands` expectation key; the brick-on-edge ring with the straddled impost). Build in progress.

# Core case A: keystone over a gap (test plan)

Owner's text ([decisions/2026-11-08-forced-collapse-and-core-suite.md](../decisions/2026-11-08-forced-collapse-and-core-suite.md) §4, A): one
keystone holds a span. With it in place the structure stands near its limit (highest utilisation
just under 1). Without it the span bricks fall. Two variants: a wedged arch with a wedge keystone
(A1), and a flat course with the keystone in the middle (A2). Mortared joints. The gap is widened
until the real mortar alone cannot carry the span.

Written against `solver_rework` at fb69475 (forced collapse and crushing as rubble merged). Nothing
was built or run. Every "today" and every number below is a hand calculation from the code as read,
not a measurement.

## Decided (owner, 2026-11-08)

1. **Mortar:** the EC6 rows. Every scenario file and structural test already uses them; the Gooch
   row appears only in the catalogue-loading tests.
2. **Flat course:** soldiers (bricks on end).
3. **Arch radius:** placed by the in-test force-method oracle.
4. **Format:** scenario files, extended with the arch builder and the `utilisation` and `forced`
   expectation keys. Voussoirs are named like `repeat` (`v.0` … `v.N−1`). No C++ E2E case.
5. **Pass counts:** never asserted. There is no `pass_record` key, and no expectation names a pass.
   The recorder still records for the viewer, and the order of events appears below only as
   explanation.
6. **Crack sets:** band-dependent sets are pinned at today's `crack_band_fraction` of 0.5.
7. **Supports:** the piers and abutments are mortared EC6 clay-brick piers standing on ground, built
   in the same file. Every expectation lists them among the bodies that stand.
8. **Dry contrast rows:** none.

## 0. Inputs, and how the engine reads a mortared joint

**Catalogue rows** (`data/catalogue.json`): `clay_brick` (ρ 2000 kg/m³, declared f_b 20 MPa, which
only ranks crushing); `clay_gp_mortar_bed_ec6` and `clay_gp_mortar_perpend_ec6`. Both mortar rows
carry the same numbers: f_t 0.0333 MPa (direct), c 0.30 MPa, μ 0.4, f_c 8.93 MPa, cap 1.30 MPa,
k_n 119 GPa/m, k_t 47.6 GPa/m.
Brick 0.215 × 0.1025 × 0.065 m, 2.864875 kg, weight P = 28.0948 N. Every wall in this case is one
brick wide (0.1025 m along y) and loaded in its own plane, so out-of-plane bending never enters.

**The four-corner joint** (`src/RbsmStandingAuthority.cpp`, `MakeJoint`, `RatiosOf`). Each corner has
a quarter of the joint area. A bonded corner never opens, so a mortared structure is linear elastic
until the cascade cracks a joint. A corner's tension ratio equals its tensile strain times k_n/f_t,
which does not depend on its area. That lets a joint made of several sub-contacts be read as one
section:

    corner tension ratio = (M·d/I − N/A) / f_t,   I = Σ a_c d_c²   (a_c corner areas, d_c offsets from the stiffness centroid)

- A single 0.215 × 0.1025 joint loaded about its 0.215 side (the joint of every soldier course and
  every arch ring here) has I = A (0.1075)² = 2.5466e-4 m⁴ and A = 0.0220375 m². With N = 0 it
  breaks at **M_cap = f_t I/0.1075 = 78.97 N·m**, and every corner holds f_t A/4 = 183.65 N in
  tension.
- The cascade (`src/Cascade.cpp`, `CrackBand`) cracks only bonded joints whose ratio exceeds 1, and of
  those only the ones at ≥ R_max × (1 − 0.5), per Overstressed component. A crack keeps compression
  and friction. A released component falls as clusters joined by live joints, and cracked joints are
  live (no hinge split, §7). An Undecided component is released **whole**, piers included, with
  `Forced` true (§1). Crushing needs a compression ratio over 1 + 1e-9. Compression never exceeds
  0.02 in this case, so every row expects `"crushed": []`.

## 1. A2, flat course: a soldier course between two brick piers

### 1.1 The pier, and why it is built this way

**Pier.** Each pier is a running-bond EC6 wall from the existing `wall` builder: **3 bricks long
(0.645 m), one brick (0.1025 m) wide, 9 courses (0.585 m) high**, on a grounded slab. Even courses
are 3 full bricks; odd courses are a bat, 2 full bricks and a bat. `pier_left` occupies
x ∈ [−0.645, 0]; `pier_right` starts at x = 0.065 n.

**The course.** n soldiers (0.065 m along x, 0.215 m tall) stand side by side in the gap, mortared
to each other and to the piers' inner faces. Their bottoms are at z = 0.240 and their tops at 0.455,
so the gap underneath is 0.24 m high. The keystone is s.(n−1)/2. The two courses above the soldier
course, c7 and c8, clamp the pier bricks the course pulls on.

**The joint at the pier face is a composite.** A 0.215 m soldier face cannot line up with 0.065 m
courses. s.0 touches four pier bricks: a 0.02 m sliver of c3 (the top of the course spanning
0.195–0.26), then the full heights of c4, c5 and c6. This is how a soldier course meets a real pier.
Reading the four contacts as one section (corner lines at 0, 0.02, 0.085, 0.15 and 0.215 m above
the soldier's base; areas 5.125e-4 m² per sliver corner and 1.6656e-3 m² per full-course corner):

    stiffness centroid z̄ = 0.1075 m (the course axis), I_j = 9.9102e-5 m⁴, d_top = 0.1075 m
    junction capacity at N = 0: M_cap,j = f_t I_j / 0.1075 = 30.73 N·m (vs 78.97 for a soldier–soldier joint)

**The junction governs. The proof, with every pier and course joint checked at the junction's own
limit (M_0 = 30.73 N·m at n = 17, V = 8.5 P = 238.8 N; the rigid-pier bound M_0 ≤ 35.25 N·m is used
wherever it is larger):**

- **Course joints.** Joint j carries M_j = M_ss(j) − M_0. At j = 1, |M_j| = |14.61 − 29.4| = 14.8,
  ratio 0.19. The largest sag is at the keystone's faces, 65.74 − 29.4 = 36.3, ratio 0.46. Every
  soldier–soldier joint stays below 0.5.
- **Pier beds.** One bed spans the 3-brick pier: corners every 0.1075 m (the running-bond
  overlaps), I_b = 2.4194e-3 m⁴, A_b = 0.0661 m², d = 0.3225 m. Under the junction zone (z = 0.195)
  it carries M_b ≤ 35.25 + 0.3225 × 238.8 = 112.3 N·m, against a compression of 6 courses
  × 3 × 28.09 + 238.8 = 744.5 N. Tension ratio = (112.3 × 0.3225/2.4194e-3 − 744.5/0.0661)/33,333 =
  (14,969 − 11,261)/33,333 = **0.11**. The base bed carries more compression and reads lower.
- **The face bricks the course pulls.** The tension corners pull c6's face brick toward the gap
  with 5.04 M_0 = 155 N. Even if all of that went through the perpend behind it (f_t × 0.065 ×
  0.1025 = 222 N), the ratio would be **0.70**. Its two beds are stiffer in shear (k_t × 0.0441 m²
  against k_n × 0.0067 m²) and take about 72 %, so the perpend reads about 0.2. Bed shear
  capacity is c × 0.022 m² ≥ 6,600 N per bed, against 155 N.
- **Junction shear** spreads by area. The sliver takes 0.093 V = 22 N against c × 0.00205 m² = 615 N.

So the face junction reads 1 while every other joint reads ≤ 0.46 (course) or ≤ 0.70 (worst-case
pier perpend). **The pier bricks stay.**

### 1.2 Keystone in place: the fixing moment, and how widening breaks it

**Rigid-pier bound (exact for that model).** With rigid piers the course is a beam of identical
joints k between two softer end junctions k_e. The junction-to-joint stiffness ratio is
I_j/I = 9.9102e-5/2.5466e-4 = 1/2.5697. The axis is straight, so there is no axial force, and
symmetry fixes V = nP/2. The rotation condition Σθ = 0 gives

    M_0 = Σ_{j=1..n−1} M_ss(j) / (n − 1 + 2 × 2.5697),   M_ss(j) = (P s/2) j (n − j),   Σ = P s n (n² − 1)/12
    P s = 28.0948 × 0.065 = 1.82616 N·m

**Brick piers lower it.** The piers rotate, and the shear acts on a lever arm about the pier
centroid: Σ_beds (M_0 + 0.3225 V)/k_b, with k_b = 9.50 k, about 5.5 beds effective (4 below the
junction zone plus about half of the 3 inside it). The piers tilting toward the gap also squeeze
the course. The pier's horizontal compliance at the junction height (0.3475 m) is 0.271/k_b. Two
piers against the course's axial compliance (n + 1)/(k_n A) give a thrust H ≈ 0.8 M_b, which
lowers every tension ratio by H/(f_t A). First-order estimate:

    M_0 [n − 1 + 5.139 + 1.158] = Σ M_ss − 0.3734 V,   ratio_j = M_0/30.73 − H/734.6

| n | L (m) | rigid-pier bound | with pier rotation | with the thrust too | |
|---|---|---|---|---|---|
| 15 | 0.975 | 0.869 | 0.694 | 0.59 | well inside |
| 17 | 1.105 | 1.147 | 0.957 | 0.84 | **the limit fixture**, estimate 0.84 to 0.96 |
| 19 | 1.235 | 1.464 | 1.261 | 1.14 | the mortar fails, estimate 1.14 to 1.26 |

These are estimates. The pier rotation and the thrust are coupled, and the rigid-course view of
the pier is an approximation, so n is placed by a reduced oracle (§4, new question 1). The
rigid-pier column is a true upper bound: pier flexibility only lowers the end moment.

**Past the edge (n = 19), described, not asserted.** The junction's top contact (s.0 to c6) cracks
first. Each crack moves the neutral axis down and concentrates the moment on the contacts below:
s.0 to c5 then reads about 1.7, then c4, then the sliver. The junction unzips from the top. Once
it has fully cracked, the course hangs on friction at the pier faces. The best case for arching is
pins at the junction bottoms with a lever of 0.1075 m, which gives H ≈ (2/0.215) × 1040.9/40 ≈ 242
N. Shear V = 9.5 P = 267 N needs μH ≥ 267 N, that is H ≥ 667 N. So the course slides out whole (a
flat arch with vertical skewbacks slides whenever 4h/L > μ; F-ARC-02, Casapulla 2001). The midspan
joints peak at about 1.03 only after the ends are almost free, far outside the band set by the
junction's own ratio, so they never crack. The piers carry at most the fully fixed moment (the
0.11 above) and stand.

### 1.3 Keystone removed (n = 17)

Each half is a cantilever of m = 8 soldiers hanging on its pier. The moment at the pier face is
P s m²/2 = 58.44 N·m, whatever the pier stiffness. N = 0 because the halves no longer touch.

- **Junction:** 58.44/30.73 = **1.90** (estimate; the share among the four contacts assumes the pier
  courses move together). Only the top contact exceeds 1 at first. It then unzips as in 1.2 until
  the half rotates about the bottom of the sliver contact, and each half falls as one piece.
- **Course joints:** P s (8 − j)²/2 / 78.97 = 0.567, 0.416, 0.289, 0.185, 0.104, 0.046, 0.012
  for j = 1 to 7. None reaches 1 at any point, so none cracks.
- **Pier:** the half weighs 8 P = 224.8 N, with its centroid 0.26 m out from the face, 0.5825 m from
  the pier centroid: M = 130.9 N·m on every pier bed. Bed under the junction zone: N = 505.7 +
  224.8 = 730.5 N, ratio (130.9 × 0.3225/2.4194e-3 − 730.5/0.0661)/33,333 = (17,449 − 11,049)/33,333 =
  **0.19**. Base: 0.08. As a rigid body, the pier's restoring moment of 758.6 N × 0.3225 m =
  244.6 N·m beats 224.8 × 0.26 = 58.4 N·m. **The pier stands, and no pier joint cracks.** While the
  junction unzips, the face bricks' perpends are pulled harder (a perpend behind c4 may read about
  1.1 when only c4 and the sliver hold). That is far outside the band of the junction contact then
  failing (ratio 7.4 or more), so they do not crack.

## 2. A1, wedged arch: a one-brick ring on brick piers

### 2.1 The ring, the piers and the springing

**Ring.** A semicircle of N_v voussoirs (odd, the nearest odd integer to πR/0.065, so each
voussoir is about a brick thick at the centreline). Radial depth **t = 0.215 m** (the brick's
length), barrel **b = 0.1025 m** (the brick's width): a brick-on-edge ring in a half-brick wall.
That keeps the ring in the same one-brick-wide plane as its piers, built by the same `wall`
builder; a 0.1025 m ring would need 0.215 m-wide piers the builder cannot make. Exact
annular-sector weights and centroids, EC6 bed row on every ring joint, the keystone v.(N_v − 1)/2.
Every ring joint is the 0.215 × 0.1025 joint of §0.

**Piers.** Running-bond EC6 walls, **4 bricks long (0.86 m), 11 courses (0.715 m), one brick
wide**, on a grounded slab. The right pier spans x ∈ [R − 0.215, R + 0.645], the left one mirrors
it. The top course c10 is an even course, so it is full bricks laid from the pier's start, and it
puts a perpend exactly at x = ±R.

**Springing geometry.** v.0's lower face is the horizontal springing plane at z = 0.715. Its radial
footprint is x ∈ [−R − 0.1075, −R + 0.1075], so it **straddles two top-course bricks**:
`pier_left.c10.b3` (the impost brick, x ∈ [−R, −R + 0.215], whose inner half projects 0.1075 m into
the opening beyond the intrados) and `pier_left.c10.b2` (x ∈ [−R − 0.215, −R]). The right pier
mirrors this with `pier_right.c10.b0` (impost) and `.b1`. The springing is therefore a two-contact
composite, corner lines at −0.1075, 0 (×2) and +0.1075 m from the ring centreline, with
I_s = 1.2733e-4 m⁴, half a solid joint. At its tension corner:

    springing ratio = (N/(f_t A)) (4e/t − 1)     (a solid joint would be (2e/t − 1))

**Why straddle.** At the limit the springing's thrust passes outside the ring (that is what mortar
tension there means). A single pier brick directly under v.0 would carry that same resultant one
course lower, shifted further out by H/V × 0.065, on a bed that running bond splits into two
contacts. That bed would govern, not the springing. With the straddle the springing is the weaker
section. The bricks under it are checked in 2.3.

### 2.2 Keystone in place: the elastic hingeless arch (force method)

Cut the arch at the crown. Use m = M_c/(qR²), h = H/(qR), q = ρ g b t = 432.2 N/m. At a section φ
from the crown, M/(qR²) = m − h(1 − cos φ) + F(φ), with F = φ sin φ − 1 + cos φ, N/(qR) =
h cos φ + φ sin φ and T/(qR) = −h sin φ + φ cos φ. The conditions are zero crown rotation and zero
crown spread, with EI/EA = t²/4 and EI/GA = 2.5 t²/4. The integrals on [0, π/2] are π/2,
π/2 − 1, 3π/4 − 2, ∫F = 0.42920, ∫F(1 − cos) = 0.25111, ∫N cos = 0.7854h + 0.3927 and
∫T sin = −0.7854h + 0.3927. The discrete ring adds the springing's compliance at the end. The
composite springing has twice the compliance of a ring joint, so every integral gains 1.5 Δ f(π/2).
Iterating R with N_v = 85 (Δ = 0.03696) and t/R = 0.122:

    h = 0.5932, m = −0.0550, M_s/(qR²) = m − h + (π/2 − 1) = −0.0773, e_s = 0.0773/1.5708 R = 0.0492 R
    springing ratio = 0.5884 R × 1.5708 × (0.9159 R − 1) = 1  at  R₁ ≈ 1.76 m   (clear span 2(R − t/2) = 3.31 m)

At R = 1.762: qR = 761.6 N, H = 451.7 N, N_s = 1,196 N, M_s = 103.7 N·m. Composite check:
(844.25 × 103.7 − 45.38 × 1196)/33,333 = **1.00**. The tension is at the intrados corner, on
`pier_left.c10.b3` and `pier_right.c10.b0`. Elsewhere: keystone faces 1.0368 × (0.05497 × 16.39 −
0.5932) = **0.32** (intrados), and every other joint carries no tension (the thrust line lies inside
the ring; the haunch at 57° reads 0). Shear ratios are ≤ 0.07 and compression ≤ 0.02.

**Precision.** The refinements (axial and shear compliance, the end correction) each move R₁ by
3 to 8 %, which moves the ratio by 10 to 25 %. So R comes from the oracle (decided 3), extended to
the composite springing and the brick piers (§4).

**Widening the gap.** At t/R = 0.122 the ring is **thicker** than the 0.1075 a dry semicircle needs
(B-ARC-01). Past R₁ the springing's impost contact cracks, and the ring stands on its geometry,
with mortar it no longer needs, until R reaches t/0.1075 = 2.0 m. Beyond that, the mortar at the
crown and haunch is what keeps it up. As a hinged arch, pinned at the extrados end of each
springing, the crown reads 0.5884 (0.6586 R² − 0.9292 R − 0.215), which is 1 at R ≈ 2.55. As a
three-hinged arch with h = (π/2 − 1 + (π/2) s)/(1 + s), s = t/2R, the haunch near 57° reads 1 at
R ≈ 2.59. Then five hinge lines form in a ring below least thickness, and it folds. The cracked
springing's friction stands at H/(μN) ≈ 0.97 in these states, close to sliding (B-ARC-06). Either
way it falls. So the fall edge is **R_fall ≈ 2.6 m**, and the arch cracks yet stands across a wide
window of about 1.76 to 2.6 m.

### 2.3 The pier bricks stay (keystone in place, R = 1.762)

The springing composite's corner forces are 2 × (−91.8) at the intrados, 4 × 149.5 at the centre
line and 2 × 390.8 N at the extrados; they sum to 1,196 N.

- **The impost brick (b3, left) alone.** It is pulled up by 183.6 N at its centre line and pushed
  down by 299 N at its outer edge, and weighs 28 N. Its bed is two contacts (I = 1.2733e-4 m⁴),
  with N = 143.5 N and M = 299 × 0.1075 = 32.1 N·m. Ratio (32.1 × 0.1075/1.2733e-4 −
  143.5/0.0220)/33,333 = **0.62**, before any help from the perpend to b2.
- **b2 alone:** 1,080.6 N at 0.078 m from its inner end, plus its own 28 N, offset 0.030 m from its
  bed centroid. Its 225.8 N share of the thrust, applied 0.065 m above its bed, works the other way.
  Net M = 18.3 N·m, resultant inside the bed: **no tension**.
- **The bed under the top course** (four bricks, corners every 0.1075 m, I = 5.6028e-3 m⁴,
  A = 0.0882 m²): N = 1,308 N with e = 0.095 m inward. Tension stress 9,518 Pa against compression
  14,838 Pa: **no tension**.
- **The base:** N = 2,432 N, e = 0.070 m outward, no tension. Base shear H = 452 N against
  c × 0.088 m² = 26 kN.

Every pier joint is ≤ 0.62 while the springing reads 1.00. **The pier bricks stay.**

### 2.4 Keystone removed (R₁ row)

Each half is a determinate curved cantilever. With δ = Δ/2 = 0.01848 rad, 2R/t = 16.39 and
ρgR/f_t = 1.0368:

    joint ratio(φ) = 1.0368 [16.39 G(φ) − (φ − δ) sin φ],   G = (φ − δ) sin φ − (cos δ − cos φ)
    springing (composite): 1.0368 [32.78 × 0.5525 − 1.5523] = 17.2   (tension at the extrados: the half leans in)

The ring joints peak just above the springing at 7.8, inside the 8.6 band edge. So the springing's
**outer contact** (`v.0/pier_left.c10.b2`) cracks first. The impost contact then carries the hanging
half alone, its ratio rises far above every ring joint, and it cracks too. The half then rotates
about its inner corner and falls as one piece. **Cracked: both springing contacts each side; no
ring joint.**

**Pier:** the half (1,182 N, centroid 1.135 m from the axis) hangs 0.842 m from the pier centroid,
so M = 995 N·m. Pier-bed tension ratios are (76,364 − σ_N)/33,333: **1.85** at the bed under the
top course, 1.47 at the base. These are over 1, but far outside the band of the springing
(17.2 → edge 8.6; later passes higher), so they never crack. As a rigid body the pier's restoring
moment of 1,236 × 0.43 = 531 N·m beats 1,182 × 0.412 = 487 N·m, so it would not tip even if the
springing held. **The piers stand. No pier joint cracks at band 0.5**; any band fraction below
1 − 1.85/17.2 = 0.89 keeps it so.

## 3. Cases and expectations

Each row is a scenario file, `data/scenarios/core-a-*.json`, run by
`E2E.Scenarios.EveryScenarioFileMeetsItsExpectation`, which records each row's pictures and replay.
"before" is the structure as the before view draws it: removals applied, no cascade (DESIGN §10,
2026-10-17). Keys marked *(new)* are the format additions of §4.

### A2 rows (piers and course from §1; `pier_right` origin x = 0.065 n)

Common build:

```json
"build": [
  { "name": "ground", "size": [4.0, 0.6, 0.2], "centre": [0.5, 0, -0.1], "grounded": true },
  { "wall": "pier_left",  "courses": 9, "bricks": 3, "origin": [-0.645, 0, 0], "on": "ground", "material": "clay_brick",
    "bed": "clay_gp_mortar_bed_ec6", "perpend": "clay_gp_mortar_perpend_ec6" },
  { "wall": "pier_right", "courses": 9, "bricks": 3, "origin": [1.105, 0, 0], "on": "ground", "material": "clay_brick",
    "bed": "clay_gp_mortar_bed_ec6", "perpend": "clay_gp_mortar_perpend_ec6" },
  { "name": "s", "size": [0.065, 0.1025, 0.215], "centre": [0.0325, 0, 0.3475], "material": "clay_brick",
    "repeat": { "count": 17, "step": [0.065, 0, 0] } }
],
"joints": [ { "materials": ["clay_brick", "clay_brick"], "row": "clay_gp_mortar_perpend_ec6" } ]
```

(The values shown are for n = 17. The slab's size and centre follow n.)

**A2.1 `core-a2-soldier-course-17-at-its-limit`.** Visual: core-a2-soldier-course-17-at-its-limit
(before: the two pier junctions near 0.9, joints coloured by tension ratio; expected after:
unchanged).

```json
"steps": ["cascade"],
"expect": {
  "stands": ["pier_left.*", "pier_right.*", "s.*", "ground"], "cracked": [], "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [0.80, 1.00], "at": ["s.0/pier_left.c6.*", "s.16/pier_right.c6.*"], "axis": "Tension" } }
}
```

Mutant: `J.TensileCapacityN = Profile.TensileStrengthPa * CornerAreaM2;` → `* 0.8` (the junction
cracks and the course falls). Tier: default (69 bodies).

**A2.2 `core-a2-soldier-course-17-minus-keystone`.** Visual: …-minus-keystone (before: s.8 gone,
junctions at about 1.9, course joints at 0.57 and below; expected after: s.0–s.7 and s.9–s.16
released as two clusters, the piers standing, the eight junction contacts dashed).

```json
"steps": ["cascade", { "remove": ["s.8"] }, "cascade"],
"expect": {
  "released": [["s.[0-7]"], ["s.[9-16]"]],
  "stands": ["pier_left.*", "pier_right.*", "ground"],
  "cracked": ["s.0/pier_left.*", "s.16/pier_right.*"],
  "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [1.70, 2.10], "at": ["s.0/pier_left.c6.*", "s.16/pier_right.c6.*"], "axis": "Tension" } }
}
```

`cracked` matches all four contacts of each end soldier with its pier, and nothing else. The
release is a rotation about one corner line, a Mechanism, so `forced` is false (§5). Mutant:
tension capacity × 2 (the junction reads 0.95 and nothing falls). The band setting is not a useful
mutant here: any fraction below about 0.85 gives the same cracked set. Tier: default.

**A2.3 `core-a2-soldier-course-15-stands`** (sweep, inside). `pier_right` origin x = 0.975.
Expectation: stands `["pier_left.*", "pier_right.*", "s.*", "ground"]`, cracked [], crushed [],
forced false, utilisation before max [0.50, 0.87] at the c6 junction contacts (0.87 is the
rigid-pier bound). Visual: …-15-stands. Tier: default.

**A2.4 `core-a2-soldier-course-19-mortar-fails`** (sweep, past the edge). `pier_right` origin x =
1.235.

```json
"expect": {
  "released": [["s.[0-18]"]],
  "stands": ["pier_left.*", "pier_right.*", "ground"],
  "cracked": ["s.0/pier_left.*", "s.18/pier_right.*"],
  "crushed": [],
  "utilisation": { "before": { "max": [1.05, 1.47], "at": ["s.0/pier_left.c6.*", "s.18/pier_right.c6.*"], "axis": "Tension" } }
}
```

`forced` is not asserted. The final release is a friction slide, which the elastic field tends to
read as Undecided, and forcing it would release the piers with the course (§5). Visual:
…-19-mortar-fails (expected after: all 19 soldiers in one cluster, the piers standing). Tier:
default.

### A1 rows (ring and piers from §2; R from the oracle, estimates shown)

Common build (R = 1.76, N_v = 85; the pier origins follow R):

```json
"build": [
  { "name": "ground", "size": [5.0, 0.6, 0.2], "centre": [0, 0, -0.1], "grounded": true },
  { "wall": "pier_left",  "courses": 11, "bricks": 4, "origin": [-2.405, 0, 0], "on": "ground", "material": "clay_brick",
    "bed": "clay_gp_mortar_bed_ec6", "perpend": "clay_gp_mortar_perpend_ec6" },
  { "wall": "pier_right", "courses": 11, "bricks": 4, "origin": [1.545, 0, 0], "on": "ground", "material": "clay_brick",
    "bed": "clay_gp_mortar_bed_ec6", "perpend": "clay_gp_mortar_perpend_ec6" },
  { "arch": "v", "material": "clay_brick", "centre": [0, 0, 0.715], "radius": 1.76, "thickness": 0.215, "depth": 0.1025,
    "voussoirs": 85, "springings": ["pier_left", "pier_right"], "joint": "clay_gp_mortar_bed_ec6" }
]
```

**A1.1 `core-a1-brick-ring-at-its-limit`** (R_stand: the largest R on a 0.01 m grid whose oracle
springing ratio is < 1; estimate 1.76). Visual: …-at-its-limit (before: impost contacts at 0.98 to
1.0, keystone faces at about 0.3, the rest 0; after: unchanged).

```json
"steps": ["cascade"],
"expect": {
  "stands": ["v.*", "pier_left.*", "pier_right.*", "ground"], "cracked": [], "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [0.95, 1.00], "at": ["v.0/pier_left.c10.b3", "v.84/pier_right.c10.b0"], "axis": "Tension" } }
}
```

Tier: `slow_in_debug` (85 voussoirs plus 88 pier bricks).

**A1.2 `core-a1-brick-ring-minus-keystone`** (R_stand, v.42 removed). Visual: …-minus-keystone
(before: springings at about 17, the pier beds at about 1.9; expected after: the two halves released,
the piers standing).

```json
"steps": ["cascade", { "remove": ["v.42"] }, "cascade"],
"expect": {
  "released": [["v.[0-41]"], ["v.[43-84]"]],
  "stands": ["pier_left.*", "pier_right.*", "ground"],
  "cracked": ["v.0/pier_left.c10.b[2-3]", "v.84/pier_right.c10.b[0-1]"],
  "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [15.0, 19.5], "at": ["v.0/pier_left.c10.b2", "v.84/pier_right.c10.b1"], "axis": "Tension" } }
}
```

Tier: `slow_in_debug`.

**A1.3 `core-a1-brick-ring-cracks-and-stands`** (R = 1.95, inside the window, t/R = 0.110 above
least thickness; N_v = 95, keystone v.47). Visual: …-cracks-and-stands (before: impost contacts at
about 1.4; expected after: the two impost contacts dashed, nothing released).

```json
"expect": {
  "stands": ["v.*", "pier_left.*", "pier_right.*", "ground"],
  "cracked": ["v.0/pier_left.c10.b3", "v.94/pier_right.c10.b0"],
  "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [1.05, 1.80], "at": ["v.0/pier_left.c10.b3", "v.94/pier_right.c10.b0"], "axis": "Tension" } }
}
```

Once an impost contact cracks, the thrust (at 0.096 m outside the centreline) lies inside the
outer contact's footprint (x ∈ [R, R + 0.1075], so e = 0.042 m < 0.054 m). The outer contact
carries no tension, and its cohesion holds the shear (0.03 of capacity). The cracked arch is
two-hinged, with the crown at 0.28 and t/R above least thickness. Tier: `slow_in_debug`.

**A1.4 `core-a1-brick-ring-folds`** (R_fall ≥ 1.05 × the oracle's fall edge; estimate 2.75,
N_v = 133). Visual: …-folds (expected after: every voussoir released, the piers standing).

```json
"expect": {
  "released": ["v.*"],
  "stands": ["pier_left.*", "pier_right.*", "ground"],
  "crushed": [],
  "utilisation": { "before": { "max": [1.5, 4.0], "at": ["v.0/pier_left.c10.b3", "v.132/pier_right.c10.b0"], "axis": "Tension" } }
}
```

`released` is a flat list, a union. Physically the ring folds into five pieces (§5). `cracked` is
not asserted: the impost contacts, then the keystone faces, then a flat-topped band of haunch
joints near 57°, so the set depends on the band and on a near-tie in the haunch. `forced` is not
asserted: a ring below least thickness reads Undecided in the elastic field (ANCHORS,
`Statics.Anchor.CoupletHeymanArchStandsAboveLeastThicknessAndNotBelow`). The pier check at
R = 2.75 (V = 1,867 N, H ≈ 713 N): the base resultant sits 0.119 m outward, no tension. Tier:
`slow_in_debug`.

## 4. Format, oracle and harness changes (built after sign-off)

- **`arch` builder.** It makes `<name>.0..N−1` as exact annular sectors, centred on `centre`, with
  radial joints of the `joint` row. Each end voussoir is joined to every top-course brick of the
  named `springings` wall that its lower face overlaps, one contact per overlap rectangle. For the
  viewer it draws each voussoir as the largest box inscribed in it (as `BuildArch` does).
- **`utilisation`** *(new)*: `{ "before" | "after": { "max": [lo, hi], "at": [joint patterns], "axis" } }`.
  It holds when the highest utilisation over live joints lies in [lo, hi], an `at` joint attains it
  (within 1e-9 relative) and `axis` governs there. It needs the before-state readings block
  (CURRENT_STATE, "Joint stress recorded in every test picture").
- **`forced`** *(new)*: `false` means no released cluster has `bForced`; absent means unchecked.
- **`stands`** *(new, owner answer 7; new question 2)*: every body matching these patterns must be
  live at the end.
- **Oracle test `Core.Keystone.TheFilesSitOnTheOracleEdges`** (`tests/CoreKeystoneTest.cpp`, default
  tier). A planar force method on a reduced model: soldiers or voussoirs as rigid bodies, joints as
  corner springs with the profile's k_n, k_t and areas, each pier course as one rigid body on group
  bed joints, the composite junction and springing contact by contact, sector weights at their
  centroids. It shares no algorithm with the production solver (a 6-DOF stiffness assembly per
  brick with an active set). It checks:
  - A1: R_stand, oracle < 1 at R and ≥ 1 at R + 0.01; A1.3's R between the springing edge and the
    fall edge; A1.4's R ≥ 1.05 × the fall edge. The fall edge is the hinge sequence of §2.2, run
    in the oracle.
  - A2: n_limit, the largest odd n whose oracle junction ratio is < 0.98; n_limit − 2 inside;
    n_limit + 2 > 1.
  - The engine's governing reading agrees with the oracle within 2 % for A2 and 1 % for A1, the
    error of treating each pier course as rigid. That tolerance is an estimate the test step
    measures before it is pinned (new question 1).

## 5. Known risks, and what each expectation does

| Risk | Where | What the expectation does | Defect or known gap |
|---|---|---|---|
| **Forced release takes the piers with it.** An Undecided component is released whole, and each half or course is one component with its pier. | Every row | `forced: false` on A2.1–A2.3 and A1.1–A1.3, where the hand statics give a stand or a one-line rotation. Unasserted on A2.4 and A1.4; there `stands` still requires the piers to remain. | On a `forced: false` row, a forced release is a **defect**. On A2.4 and A1.4 the piers falling with a forced release is the §1 over-release, a known gap and a solver work-queue item; the row stays red as written until the solver proves the mechanism. |
| **No hinge split** (§7). | A1.4 physically folds into 5 pieces; A2.2, A2.4 and A1.2 fall as rigid pieces | A1.4 asserts only the union. The others assert the partition, which is physically right. | Known gap, §7. A wrong union is a defect. |
| **Crack band** of 0.5. | A2.2, A2.4 and A1.2 cracked sets; A1.2's pier beds read about 1.85 | Pinned at 0.5. Each set is the junction or springing contacts, and every other over-1 joint sits far below the band edge. | Changes only above a fraction of about 0.85 (A2) or 0.89 (A1); D2/D3 narrow the band, so the sets hold. Not a defect. |
| **Estimate precision**: pier rotation, thrust and the composite share. | A2 n, A1 R | The reduced oracle places both. Bands are wide where only the hand estimate stands. | Engine against oracle beyond tolerance: work it through DEBUG_PLAYBOOK. |
| **Springing friction near its limit**: H/μN ≈ 0.97 in the hinged states. | A1.3, A1.4 | A1.3 relies on the outer contact staying bonded, so cohesion carries its shear (0.03). | A1.3 sliding or reading Undecided is a defect: an admissible field exists. |
| **Face-brick perpends in A2.2** may read about 1.1 while the junction unzips. | A2.2 | `cracked` excludes them, and the junction contact failing at the same time reads 7.4 or more. | A pier perpend cracking would be a band defect. |

## 6. Open questions (new with the brick piers)

1. **No closed form for A2 any more, and the oracle is approximate.** The pier's rotation and
   thrust have no exact hand value, and the reduced oracle treats each pier course as rigid.
   **Recommend:** place A2's n and check both variants against the reduced oracle within a
   tolerance the test step measures first (estimated 2 % for A2, 1 % for A1), and accept the A2
   limit band [0.80, 1.00) for the fixture.
2. **A `stands` key.** Listing the pier bricks explicitly needs a fourth expectation key beyond the
   approved three. **Recommend yes.** `released` already implies what stands, but a named `stands`
   list makes the pictures and the report read the way the owner asked.
3. **The A1 ring changed shape to fit brick piers.** The ring is now one brick deep radially
   (0.215 m) in a half-brick wall, so the piers are ordinary `wall` builds. Its springing
   straddles two pier bricks on a 0.1075 m impost, so that the springing, not the pier brick under
   it, governs. Consequences: R₁ ≈ 1.76 m (85 voussoirs), the cracked ring stands across a wide
   window (A1.3), and the fold row needs R ≈ 2.75 m (133 voussoirs). **Recommend accepting.** The
   alternative, a 0.1025 m ring on 0.215 m-wide piers, needs a two-wythe pier builder and makes the
   springing lose to the brick beneath it.
