Status: plan for owner sign-off, not built. Written against `solver_rework` at 5df46c7. Nothing was built or run; every number below is a hand calculation from the code as read.

# Core case C: incline (test plan)

Owner's text ([decisions/2026-11-08-forced-collapse-and-core-suite.md](../decisions/2026-11-08-forced-collapse-and-core-suite.md) §4, C):
C1 a dry brick on a slope steeper than the friction angle slides; C2 on a shallower slope stays;
C3 a mortared column normal to the slope stands; C4 the same column past its limit collapses,
C4a tall slender (bending, tips at the base), C4b short wide on a steep slope (shear, slides off
at the base). Base shear W sin θ; base moment grows with height squared.

**Headline findings.**
- The format's `lateral_g` step does not normalise gravity. Recommend a normalised `slope_deg` step (§1).
- No dry clay-brick friction value was found. Proposed: a new dry row at μ 0.6, sourced from
  dry-stack tests on concrete units (§2).
- **C4b cannot be built as written with the EC6 row under self-weight.** Mortar shear only
  governs on a column at least 15.3/sin θ m tall, and bending fails such a column long before
  that (§3.3). The recommended replacement is open question 3.

## Decided (carried from case A, owner 2026-11-08)

1. Real catalogue rows only. A dry row that is missing gets added to the catalogue with a source, not invented in a file.
2. Mortar: the EC6 rows (`clay_gp_mortar_bed_ec6`, `clay_gp_mortar_perpend_ec6`). Gooch appears only in the catalogue tests.
3. Bodies are `clay_brick`.
4. Scenario files, run by the generic `E2E.Scenarios.*` test, with a picture per file. No C++ E2E case.
5. Pass counts are never asserted.
6. Every brick structure stands on a grounded `ground` box.
7. No dry rows inside mortared structures.
8. Geometry, not material values, places a row at its edge.
9. Every structural row gets a before and an expected-after visual description, with per-joint readings.
10. Near-limit rows carry a `utilisation` band expectation (A2.1 style).
11. Crack sets are pinned at today's `crack_band_fraction` 0.5. No set in this case depends on it.

## 0. Inputs, and the joint formulas the engine uses

**Brick.** 0.215 (x) × 0.1025 (y) × 0.065 (z) m, ρ 2000 kg/m³, W_b = 28.0948 N.
ρg/f_t = 19,613/33,333 = 0.5884 m⁻¹. f_t A/4 = 183.6 N and c A/4 = 1,652.8 N per corner of a
0.215 × 0.1025 bed.

**Joint** (`src/RbsmStandingAuthority.cpp`, `MakeJoint`, `RatiosOf`, `SlipCapacityN`). There are
four corners at the contact rectangle's vertices, each with a quarter of the area.
- Tension ratio = corner tension / (f_t A/4).
- Shear ratio = |corner shear| / min(c A/4 + μ C_corner, cap A/4), where C is that corner's compression.
- A bonded joint (c > 0, f_t > 0) never opens or slips, so a mortared stack is linear until the cascade cracks a joint.
- A dry corner slips at min(μ C, cap A/4).
- Shear splits equally over the four corners (equal k_t, no torsion here). Normal forces are linear across the rectangle.

For a joint of width a in the tilt direction, carrying N, V and moment M about its centre:

    corner normal  = N/4 ± M/(2a)
    tension ratio  = (2M/a − N) / (f_t A)                    (mortared; the f_t A a/2 hinge at N = 0)
    shear ratio    = V / (c A + μ (N − 2M/a)⁺)               (the tension-side corner governs)
    dry corner slip starts at the uphill corners:  tan θ = μ / (1 + μ h/a)

The cap (1.30 MPa) would take over only above 2.5 MPa of corner compression. Compression in this
case stays below 0.012 of capacity, so no row crushes.

**A mortared stack is statically determinate.** Each bed carries everything above it, so the
engine's readings should equal the closed form to round-off (§4).

## 1. How the slope is modelled

**Today.** `lateral_g [Lx, Ly]` sets gravity to (Lx g, Ly g, −g) (`src/Scenario.cpp:1860`). It is
not normalised, so its magnitude is g√(1+L²). With L = tan θ this is a slope of θ under gravity
g / cos θ. N = W and V = W tan θ, where a real slope gives W cos θ and W sin θ.

| Quantity | Effect of the unnormalised tilt |
|---|---|
| Dry sliding (V/μN) and rigid tipping (resultant against the base edge) | none: ratios of loads |
| Mortar tension and cohesion utilisation | × 1/cos θ: 1.035 at 15°, 1.22 at 35° |
| C3 edge | 25.17 courses instead of 25.47: n = 25 would read 0.980, not 0.947 |

The C++ anchors use the normalised form. `tests/StaticsTestSupport.h` `TiltedPlane` sets
gravity to g(sin θ, 0, −cos θ), and the incline anchors check it against `LateralLoad(tan θ)` for
the verdict only.

**Options.**
- **(a) Recommended: a normalised step**, `{ "slope_deg": [θx, θy] }`, with gravity
  g·(tan θx, tan θy, −1)/|…|. On one axis this is exactly `TiltedPlane`. It costs a parser case
  and a viewer field: the writer emits `lateral` = gravity/g, which would show sin θ labelled λ,
  so it should write `slope_deg` instead.
- **(b) `lateral_g` as it is.** Choose L = tan θ and accept the 1/cos θ factor. C3 moves to
  n = 24 (0.867) and C4a stays at n = 26 (1.100). Every mortar number in the pictures is then
  3.5 % heavier than the real slope.
- **(c) A real sloped ground body.** `FaceContact` handles axis-aligned boxes only, so this needs
  a rotated builder like `arch`. The solver depends on orientation only through the gravity
  vector, so the physics would be identical to (a). Not recommended: the cost buys only the
  picture.

**Before and after.** The before judge applies every `lateral_g` (or `slope_deg`) step
(`Scenario.h`, `ScenarioOutcome::Before`), so the slope is in force from the start. "Before" is
the brick or column on the slope, and "after" is the cascade's result. This matches case A's
model, and no flat-then-tilted picture is needed. The case is about standing on a slope, not
about the act of tilting. In the picture, the ground is drawn flat with a gravity arrow (open
question 5).

## 2. Dry brick–ground friction (C1, C2, and C4b if option (a) is taken)

**Today.** The catalogue has no dry row. The C++ helper `DryBrick()` (`tests/EndToEndSupport.h:107`)
is the EC6 bed with c = f_t = 0 and μ = 0.7, and that 0.7 has no source.

**Sources.** No published static friction value for dry fired clay brick was found.
- Lin, Totoev, Liu, Wei 2015, *Materials* 8(12) 8731, dry-stack triplet shear on solid concrete
  units (https://pmc.ncbi.nlm.nih.gov/articles/PMC5458854/, **opened**): μ 0.61 initially and 0.58
  after wear; 0.48, 0.56 and 0.62 at 0.1, 0.3 and 0.5 MPa.
- Liu, Liu, Lin, Zhao 2016, *Materials* 9(3) 166, flat mortarless light-aggregate concrete joints
  (https://pmc.ncbi.nlm.nih.gov/articles/PMC5456668, **opened**): 0.55, 0.592, 0.622 and 0.638 at
  0.017 to 0.050 MPa.
- Hossain, Totoev, Masia 2016, 16th IBMAC pp. 1635–1643, doi:10.1201/b21889-217, dry brick surfaces
  in a modified couplet test: **not opened**, numbers not seen.

**Proposed row.** `brick_dry_contact_mean`, kind bed, basis mean.
- c 0, f_t 0, μ **0.6** (the round value of both opened papers).
- No shear cap.
- Compression and stiffness are the EC6 code row's, as the Gooch and UPC rows already do.

The note must say the data are concrete units, not clay. Both papers show μ falling as the normal
stress falls. A single brick's bed stress is 1.3 kPa, below both test ranges, so the real value
may be nearer 0.5.

**Angles.** atan 0.6 = 30.96°.
- **C2 at 25°**: tan 0.466 = 0.78 μ. That is below the lowest measured value (0.48), and below
  the uphill-corner slip onset of 26.93°, so every corner sticks and the reading is a clean
  elastic number.
- **C1 at 35°**: tan 0.700 = 1.17 μ. That is above the highest measured value (0.64).

So both outcomes hold for any μ inside the measured scatter, not only for the row's 0.6. Tipping
needs tan θ > a/h = 3.31 (73°), far off.

## 3. Rows

### 3.1 C1 and C2: one dry brick on ground

Build: `ground` [1.0, 0.6, 0.2] grounded, top at z = 0. `brick` [0.215, 0.1025, 0.065] at
[0, 0, 0.0325], long axis downhill (+x). Joint rule: `{ "materials": ["clay_brick", "ground"],
"row": "brick_dry_contact_mean" }`.

Uphill and downhill corner ratios are tan θ / (μ(1 ∓ 0.3023 tan θ)), where 0.3023 = h/a.

| Row | θ | tan θ / μ | uphill corners | downhill corners | outcome |
|---|---|---|---|---|---|
| C2 `core-c2-dry-brick-stays-at-25` | 25° | 0.777 | 0.905 Shear | 0.681 Shear | Stands, all stuck |
| C1 `core-c1-dry-brick-slides-at-35` | 35° | 1.167 | slipping (1.0) | slipping (1.0) | Mechanism, pure translation +x |

```json
"steps": [{ "slope_deg": [25, 0] }, "cascade"],
"expect": { "stands": ["brick", "ground"], "cracked": [], "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [0.85, 0.95], "at": ["brick/ground"], "axis": "Shear" } } }
```

```json
"steps": [{ "slope_deg": [35, 0] }, "cascade"],
"expect": { "released": [["brick"]], "stands": ["ground"], "cracked": [], "crushed": [], "forced": false }
```

C1 carries no `utilisation` key: every corner of a sliding block reads exactly 1.

**Visuals.**
- C2: before, the brick on the flat slab with the slope arrow at 25°, uphill corners 0.905 and
  downhill corners 0.681, all Shear. Expected after: unchanged.
- C1: before, the same at 35° with all four corners marked slipping. Expected after: the brick
  released and moved downhill along +x with no rotation, the slab standing.

**Mutants.**
- C2, μ × 0.85 (0.51): the uphill corners slip and read 1.0, out of band.
- C2, μ × 0.75: it slides.
- C1, μ × 1.2 (0.72): it stands.

Tier: default.

### 3.2 C3 and C4a: a mortared column normal to the slope (bending)

Build: `ground` [1.0, 0.6, 0.2] grounded. `col`: brick [0.215, 0.1025, 0.065] at
[0, 0, 0.0325], `repeat` n with step [0, 0, 0.065]. Joint rules:
`["clay_brick", "clay_brick"]` → `clay_gp_mortar_bed_ec6`, and `["clay_brick", "ground"]` →
`clay_gp_mortar_bed_ec6`. Slope fixed at **θ = 15°** with tilt along x, so a = 0.215. Height is
the tuned geometry.

A bed with height H′ of column above it reads:

    W = ρ g a b H′,   N = W cos θ,   V = W sin θ,   M = V H′/2
    tension ratio = (ρg/f_t) H′ (H′ sin θ / a − cos θ) = 0.5884 H′ (1.2038 H′ − 0.9659)
    edge: H = 1.6553 m = 25.47 courses (unnormalised tilt: 25.17)

The base governs because the ratio rises with H′. Only beds with H′ > a cot θ = 0.802 m carry
tension; the rest are compression-only with shear ≤ 0.013. Base shear is 0.028 against cohesion,
so shear never governs.

A dry column this tall would tip at atan(a/H) = 7.5°. At 15° it stands only on the mortar's tension.

| Row | n | H (m) | W (N) | N (N) | V (N) | M (N·m) | base (normalised) | base (`lateral_g`) | bed 1 | base shear |
|---|---|---|---|---|---|---|---|---|---|---|
| — | 24 | 1.560 | 674.3 | 651.3 | 174.5 | 136.1 | 0.837 | 0.867 | 0.733 | 0.026 |
| **C3** | 25 | 1.625 | 702.4 | 678.4 | 181.8 | 147.7 | **0.947** | 0.980 | 0.837 | 0.028 |
| **C4a** | 26 | 1.690 | 730.5 | 705.6 | 189.1 | 159.8 | **1.063** | 1.100 | 0.947 | 0.029 |
| — | 27 | 1.755 | 758.6 | 732.7 | 196.3 | 172.3 | 1.184 | 1.226 | 1.063 | 0.030 |

C3 base corners: 173.9 N tension uphill and 513.1 N compression downhill.

**After the C4a base cracks.** The cracked base keeps compression and friction μ 0.4 and loses
tension and cohesion. The resultant sits e = tan θ · H/2 = 0.226 m from the base centre, past the
toe at 0.1075, so the column **tips about its downhill edge**. It does not slide: tan 15° = 0.268
< 0.4, a 33 % margin, and that margin also holds on the toe corners alone.

Nothing above the base cracks. Bed 1 reads 0.947 < 1, and the stack's internal forces do not
change when the base cracks (they are determinate). At n = 27, bed 1 would read 1.063 and crack in
the same pass, which is why C4a is n = 26. The column falls as one cluster, as a proven Mechanism.

```json
"steps": [{ "slope_deg": [15, 0] }, "cascade"],
"expect": { "stands": ["col.*", "ground"], "cracked": [], "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [0.90, 1.00], "at": ["col.0/ground"], "axis": "Tension" } } }
```

```json
"expect": { "released": [["col.[0-25]"]], "stands": ["ground"], "cracked": ["col.0/ground"], "crushed": [], "forced": false,
  "utilisation": { "before": { "max": [1.00, 1.15], "at": ["col.0/ground"], "axis": "Tension" } } }
```

Bed readings for the visuals (j = 0 is the base; T tension, S shear):

| | j0 | j1 | j2 | j3 | j4 | j5 | j6 | j8 | j10 | j12 | j13 | top |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C3 (n 25) | 0.947 T | 0.837 T | 0.733 T | 0.636 T | 0.544 T | 0.458 T | 0.378 T | 0.237 T | 0.119 T | 0.026 T | 0.013 S | 0.001 S |
| C4a (n 26) | 1.063 T | 0.947 T | 0.837 T | 0.733 T | 0.636 T | 0.544 T | 0.458 T | 0.305 T | 0.175 T | 0.069 T | 0.026 T | 0.001 S |

**Visuals.**
- C3 `core-c3-column-25-at-its-limit`: before, a 1.625 m column on the slope, the base bed at
  0.947 Tension on its uphill edge, shading down to compression-only above 0.80 m. Expected
  after: unchanged.
- C4a `core-c4a-column-26-tips`: before, the base at 1.063. Expected after: the base dashed
  (cracked), the 26 bricks released as one piece rotating about the downhill base edge, the slab
  standing.

**Mutants.**
- C3, tension capacity × 0.9: the base reads 1.052 and the column tips.
- C4a, tension × 1.1: it reads 0.966 and stands.
- Dropping the slope normalisation does not move C3 out of its band (0.980). The oracle test (§4)
  catches it.

Tier: default (26 bodies).

### 3.3 C4b: short wide on a steep slope (shear) — not reachable as written

With a mortared base, the shear ratio at the governing corner is at most V/(cA) = ρ g H sin θ / c
= 0.0654 H sin θ. It reaches 1 only at **H ≥ 15.3/sin θ m**, whatever the footprint. At that height
the bending ratio is in the hundreds unless a > H tan θ, that is, a block more than 15 m wide.

The two failure modes do not overlap. A tension crack needs tan θ > (a/H)(1 + f_t/(ρ g H cos θ)),
which is already past the rigid tipping angle. So **a mortared base that cracks always tips**, and
slides as well only if tan θ > 0.4. EC6 cohesion is 9 × the tension and self-weight shear stress
is tiny, so the slope never shears the bond. Physically: a short mortared block on a steep slope
just stays.

Three ways to keep a sliding row (open question 3):

- **(a) Recommended:** `core-c4b-mortared-block-slides-at-35`. A mortared running-bond block
  (course 0: `lo` 4 bricks; course 1: `bat_l`, `hi` × 3, `bat_r`; 0.86 × 0.1025 × 0.13 m; W =
  224.8 N) **resting dry** on the slab (`brick_dry_contact_mean`) at 35°.
  - V = 128.9 N against μN = 110.5 N, ratio 1.17, so it slides.
  - The resultant offset is 0.046 m against a half-base of 0.43, so it cannot tip (that would take 81°).
  - Internal joints: shear ≤ 0.02 and no tension crack. The block slides as one piece.
  - Expect released [["lo.*", "hi.*", "bat_*"]], stands ["ground"], cracked [], forced false.
  - Needs repeat boxes, not the `wall` builder: the wall builder makes its own base joints, and
    rules skip them (`Join`, `src/Scenario.cpp:1233`).
  - This is a dry base under a mortared block. Under ruling 7 that is a joint *to* the ground,
    not inside the structure; the owner should confirm.
- **(b)** A load-driven row: one mortared brick at 45° with a surcharge P(sin θ, 0, −cos θ) at its
  centroid. Shear governs at P + W = cA / (sin θ − μ(cos θ − 0.3023 sin θ)) = 12.97 kN, with no
  tension. After the crack it slides (tan 45° > 0.4). It is load-driven, not slope-driven: about
  1.3 t on one brick.
- **(c)** Drop C4b and record the finding in ANCHORS.

## 4. Oracle (minimal; everything here is closed form)

`tests/CoreInclineOracle.h` holds two inline functions:
- `BedTensionRatio(rho_g, f_t, a, H′, θ)`
- `DryCornerShearRatio(θ, μ, h/a, uphill)`

They use the formulas of §0, and the slope gravity comes from the same `TiltedPlane` form.

Test `Core.Incline.TheFilesSitOnTheClosedFormEdges` (default tier) checks:
- C3's n is the largest n with an oracle base ratio < 1, and C4a's is n + 1.
- C4a's bed 1 is < 1.
- C2 sits below the corner-slip onset and C1 above atan μ.
- C4a after cracking: tan θ < μ_cracked and e > a/2.

Test `Core.Incline.TheEngineMatchesTheClosedFormOnEveryJoint` (default tier) checks every bed of
C3 and C4a, and every corner of C2, against the oracle to 1e-9 relative. The stack is
determinate, so any larger error is a defect. Provenance goes in ANCHORS.md.

Format work, built after sign-off:
- The `slope_deg` step, with parser cases `Scenario.Steps.ASlopeStepTiltsGravityAtFullMagnitude`
  and a refusal at |θ| ≥ 90°.
- The viewer's slope field.
- The catalogue row and its loader case.

## 5. Risks

| Risk | Rows | What the expectation does |
|---|---|---|
| Unnormalised gravity if option (b) is kept: mortar readings × 1/cos θ | C3, C4a, C4b-(b) | The oracle pins the exact value; with (b), C3 moves to n = 24 |
| Uphill corners of a dry block slip from 26.9°, below atan μ. Between 26.9° and 31° the block stands with corners slipping, and the active set may read Undecided | C2 if moved up; the optional C2b (Q4) | C2 sits at 25°, all stuck. A forced release there is a defect |
| Pure slide read as Undecided rather than Mechanism | C1, C4b-(a) | `forced: false`. The component is the brick or block alone, so a forced release gives the same set and only the flag differs. `Statics.Anchor.InclineSlidesAtAtanFrictionAsAPureTranslation` covers it in the solver |
| Toe rotation after the crack read as Undecided (open/closed corner oscillation) | C4a | `forced: false`. The set is the column either way |
| Released-slab motion drawn as a translation rather than a topple (CURRENT_STATE, scenario workbench follow-up 5) | C4a picture | Not asserted; the picture may mislead, as for the bonded wall |
| C4a slide margin, tan 15° = 0.268 against cracked μ 0.4 | C4a | 33 % margin; a slide would be a defect |
| Unsourced μ 0.7 in `DryBrick()` | other tests | Out of scope; noted only |

## 6. Open questions

1. **Slope model.** Recommend a normalised `slope_deg` step (§1a). The alternative is to keep
   `lateral_g` and accept mortar readings 3.5 % high at 15° (C3 at n = 24).
2. **Dry row.** Recommend adding `brick_dry_contact_mean` at μ 0.6. Its sources are concrete-unit
   dry-stack tests (both opened); no clay-brick value was found. C1 and C2 hold for any μ in the
   measured 0.48–0.64.
3. **C4b.** Mortar shear cannot govern under self-weight. Recommend (a), a mortared block resting
   dry on the slab that slides at 35°. Otherwise (b), a 13 kN load-driven brick, or (c), drop C4b.
4. **Add C2b at 29°** (the dry brick stands with its uphill corners slipping; reads 1.0)?
   Recommend yes, held under `deferred` if it reads Undecided. It is the honest test of the
   active set near the friction edge.
5. **Picture.** Recommend the viewer rotates the scene by θ so the ground reads as a slope.
   Otherwise the slab stays flat with a gravity arrow.
6. **Bands.** C3 [0.90, 1.00) and C4a [1.00, 1.15), with the exact values pinned by the oracle
   test. Recommend accepting.
