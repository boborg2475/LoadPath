# Masonry arch and vault benchmarks for a rigid-block engine

Scope: published benchmarks for arches analysed as assemblies of rigid voussoirs, where the collapse condition is known exactly from limit analysis or was measured. All quantities SI. Unless a benchmark says otherwise, the assumptions are Heyman's three: no tensile strength, infinite compressive strength, no sliding (infinite friction), with hinges forming at intrados/extrados edges and joints radial.

Notation used throughout (from Cocchetti, Colasante & Rizzi 2011): `r` = radius of the arch centreline, `t` = radial thickness, `η = t/r`, `α` = half-angle of embrace (α = 90° is the full semicircular arch; the full embrace angle β = 2α), `β_hinge` = angular position of the intrados (haunch) hinge measured from the crown, `h = H/(w r)` with `w = γ t d` the weight per unit length of centreline (γ = unit weight, d = out-of-plane depth), and `ĥ = H/(γ d r²) = η h`.

Local text extractions of the PDFs cited below are in the session scratchpad (`cocchetti2011.txt`, `jota2020.txt`, `forgacs.txt`, `sarhosis_review.txt`, `nnj.txt`, `fzu_collapse.txt`, `arch04.txt`, `diva.txt`, `chalmers.txt`, `ijbe.txt`, `compdyn.txt`, `ff2024.txt`).

---

## Key question 1 — The Couplet–Heyman problem (semicircular arch under self-weight)

### Takeaway
There is no single "exact" answer; there are three well-defined answers depending on how the tangency condition and the voussoir centroid are treated, and a fourth depending on joint orientation. For radial joints and the true self-weight distribution (Milankovitch) the minimum thickness is `t/r = 0.107478` with the haunch hinge at `54.484°` from the crown; Heyman's often-quoted `0.1060` (hinge at `58.83°`) is slightly unconservative; vertical joints give `0.10946` (hinge at `54.92°`). A test should pin the Milankovitch triple (`η`, `β_hinge`, `h`) to about 4 significant figures and the Heyman triple as a named, distinct variant.

### Cited findings

**Heyman's classical solution (tangency of the resultant thrust force, weight on the centreline)**
- "for a complete semicircular arch (α = π/2), the solution of Heyman's formulas renders the following usually-referenced values of β, η, h: β_H = 58.8293° = 1.02677 rad, η_H = 0.105965, h_H = 0.621113 (ĥ_H = 0.0658164)" — [Cocchetti, Colasante & Rizzi 2011, "On the analysis of minimum thickness in circular masonry arches", Applied Mechanics Reviews (accepted manuscript)](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf)
- Heyman's ratio of extrados to intrados radius: "K_H = 1.11189", and "η_H/2 = 0.0529826" (thickness/span) — same source, Table 1.
- Couplet (1730) "is quoted by Heyman to assume, for the full semicircular arch (α = π/2), that β_cr is at 45° and to obtain that η_cr is at around 0.101" — same source.
- Independent confirmation of the Heyman numbers: "Heyman (1977) using analytical formulations found that for a semi-circular arch, β_cr is equal to 58.82° and t_min is equal to 0.105965 · R" — [Forgács, Sarhosis & Bagi 2017, Engineering Structures 140:317–336, DOI 10.1016/j.engstruct.2017.02.036 (accepted manuscript)](https://eprints.whiterose.ac.uk/id/eprint/144597/3/Manuscript_ForgacsSarhosisBagi_EngStr_revised_marked_20161218.pdf)

**"CCR" solution (true line of thrust = locus of centres of pressure, weight still on the centreline)**
- "β_cr was found at around β_CCR = 54.4963° = 0.951141 rad, with η_cr at about η_CCR = 0.107426 and h_cr near h_CCR = 0.621772 (ĥ_CCR = 0.0667947)"; "K_CCR = 1.11352" — [Cocchetti et al. 2011](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf)

**Milankovitch solution (true line of thrust and true centroid of each voussoir) — the reference answer for radial joints**
- "β_M = 54.4840° = 0.950925 rad, η_M = 0.107478, h_M = 0.620881 (ĥ_M = 0.0667311)"; "K_M = 1.11358"; "η_M/2 = 0.0537390" — [Cocchetti et al. 2011, Table 7](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf)
- Foce (2007) reports that Milankovitch obtains "β = 54°29' = 54.4833°, together with η = 0.1075" — same source.
- Makris & Alexakis, deriving the same result by stationary potential energy: "This happens when β_r = 54.484° and the corresponding minimum thickness is t/R = 0.10748" — [Makris & Alexakis 2015, "The effect of stereotomy on the shape of the thrust-line and the minimum thickness of masonry arches", Int. J. Bridge Engineering 3(3):35–42](https://www.ijbe.net/issues/volumes/item/download/104_225d6c04d4d044dd4b08acc37f969f22)
- Ochsendorf's trial-and-error procedure "converged to the correct value of the rupture angle β_r = 54.5° and minimum thickness t/R = 0.1075 initially discovered by Milankovitch" — same IJBE source.
- The CCR and Milankovitch hinge angles differ by "0.024%, since while β_CCR = 54.4963°, β_M = 54.4833°, thus both at around 54.5° (as opposed to β_H = 58.8293°, thus at around 59°)" — [Cocchetti et al. 2011](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf)

**Other published values for the same problem (useful as tolerance evidence)**
- Sinopoli, Corradi & Foce (1997) give "the critical value K_cr = 1.1136 for the geometrical parameter K = r_e/r_i", with the haunch hinge "at around 54°"; this "would lead to a critical value η_cr = 0.10749" — [Cocchetti et al. 2011](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf)
- Lucchesi et al. (1997): "a value of η_cr of nearly 2×0.0537 = 0.1074" — same source.
- Bicanic et al. (2003), DDA numerical model: "β_cr = 56.2° and η_cr = 0.10598", with the rotational mechanism appearing "when η < η_cr" — same source.
- Blasi & Foraboschi (1994): thickness/span "0.9/17 = 0.0529412 … would correspond to η_cr = 0.105882 < η_H" (slightly sub-critical) — same source.
- Boothby (1996), discretised segmental arch: hinge "of about 90°−36° = 54°" — same source.

**Joint orientation (stereotomy)**
- Vertical joints: "This happens when x_r = 0.7736R and the corresponding minimum thickness is t/R = 0.10946", equivalently "a rupture angle β_r = 54.923° and a minimum thickness value t/R = 0.10946" — [Makris & Alexakis 2015, IJBE](https://www.ijbe.net/issues/volumes/item/download/104_225d6c04d4d044dd4b08acc37f969f22)
- Same source, Table 1: radial rupture `β_r = 54.484°, t/R = 0.10748`; vertical rupture `β_r = 54.923°, t/R = 0.10946`.
- "Heyman [13] incorrectly assumed that the line of action of [the half-arch weight passes through the hinge] … slight error in the calculation of his minimum thickness, t/R = 0.106; rather than the correct value t/R = 0.1075"; and the Heyman value "remains unconservative regardless the stereotomy exercised on the arch — even if one assumes vertical joints" — same IJBE source.
- Independent restatement: "[1] confirmed the results of [4] about the dimensionless minimum thickness of semicircular arches that equals t/R = 0.1075 … utilising radial cuts and t/R = 0.1095 by implementing vertical cuts" — [Kampas et al., COMPDYN 2021 paper CD21_19608](https://files.eccomasproceedia.org/papers/compdyn-2021/CD21_19608.pdf?mtime=20210910085227)
- A search-result summary states that with a non-continuous (free) stereotomy the Milankovitch value "0.1075 can be reduced to 0.0819" — [search summary pointing at "The Influence of Milankovitch's Thrust Line Theory on the Equilibrium Analysis of Masonry Arches" (ResearchGate)](https://www.researchgate.net/publication/334588422_The_Influence_of_Milankovitch's_Thrust_Line_Theory_on_the_Equilibrium_Analysis_of_Masonry_Arches). Not verified against the full text; treat as a pointer only.

**Finite friction**
- Analytical: for the least-thickness semicircular arch with radial joints, "The maximal frictional resistance should exist at the springing line of the arch, where φ = π/2. The analytically observed minimum necessary angle of friction that prevents shear sliding is equal to 21.57°." — [Forgács, Sarhosis & Bagi 2017](https://eprints.whiterose.ac.uk/id/eprint/144597/3/Manuscript_ForgacsSarhosisBagi_EngStr_revised_marked_20161218.pdf). (tan 21.57° = 0.395.)
- DEM confirmation: "a shear sliding failure mechanism observed when friction angle was 21°. In the case that the friction angle increases to 22°, the arch is standing." — same source.
- Bound that covers all openings up to 142.5°: "any form of sliding, for α ≤ α_lm = 2.48716 = 142.504°, shall be prevented by μ ≥ μ_lm = 1.41527 (φ_lm = 54.7558°)" — [Cocchetti & Rizzi 2020, J. Optim. Theory Appl. 187:707–757, DOI 10.1007/s10957-020-01772-0 (open manuscript)](https://re.public.polimi.it/retrieve/e0c31c10-4b89-4599-e053-1705fe0aef77/Cocchetti-Rizzi-JOTA-2020(187)-%5BStatic-upper-lower-thrust-and-kinematic-work-balance-stationarity-for-least-thickness-circular-masonry-arch-optimization%5D.pdf). The same paper's numerical validation used "a high value of μ = 10 (friction angle φ = 84.3°)" to guarantee purely rotational collapse.
- Full map of states vs friction: "a value of friction coefficient that shall be larger than that (say friction angle larger than about 70°) shall prevent any purely-sliding mode to appear within the masonry arch", the far boundary being at "β = 2.774176793356034 ≈ 159° and μ = 2.690297881776622 (φ ≈ 69.6°)"; the paper classifies seven collapse types (purely rotational, purely sliding, mixed sliding-rotational with shoulder sliding, two mixed rotational-sliding with inner sliding, overturning-sliding, purely overturning) — [Cocchetti & Rizzi 2024, "Finite-friction least-thickness self-standing domains of symmetric circular masonry arches", Structures 66:106800, DOI 10.1016/j.istruc.2024.106800](https://re.public.polimi.it/retrieve/e3e524b4-a174-43e3-97f6-941a2092481d/Cocchetti-Rizzi-STRUCTURES-2024(66)-%5BFinite-friction-least-thickness-self-standing-domains-of-symmetric-circular-masonry-arches%5D.pdf)
- Explicit statement that purely rotational collapse requires infinite friction only near the limit opening: horizontal-thrust-zero states "arise just at infinite friction coefficient, μ → ∞, for 2.648388899151007 ≤ β ≤ 2.786498150651177 (say 152° ≤ β ≤ 160°)" — same 2024 source.
- Earlier dedicated paper (abstract only): Cocchetti & Rizzi, "Analytical and numerical analysis on the collapse modes of least-thickness circular masonry arches at decreasing friction", Frattura ed Integrità Strutturale 14(51):356–375 (2020), which "analytically derived and numerically identified" the friction thresholds separating purely rotational, mixed and purely sliding modes — [journal page](https://www.fracturae.com/index.php/fis/article/view/2554); also Rizzi, Rusconi & Cocchetti, "Analytical and numerical DDA analysis on the collapse mode of circular masonry arches", Engineering Structures 60:241–257 (2014) — cited in the 2024 paper's reference list.

**Finite number of voussoirs / discretisation**
- 3DEC discrete-element model of a radially jointed semicircular arch converged to the Milankovitch values with "precision … ± 1.5° and ± 0.0005 · R", the residual attributed to "the discrete element model consists of discrete blocks, while Milankovitch's derivations assumed a homogenous material where cracks can develop anywhere along the arch" and to the fact that "the equations of motion are written always on the current geometry" — [Forgács, Sarhosis & Bagi 2017](https://eprints.whiterose.ac.uk/id/eprint/144597/3/Manuscript_ForgacsSarhosisBagi_EngStr_revised_marked_20161218.pdf)
- Boothby (1996) and Bicanic et al. (2003) discretised results (54° and 56.2°/0.10598 respectively) bracket the continuous answers — [Cocchetti et al. 2011](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf)
- The experimental/analytical examples at MIT use 16 voussoirs for a 157.5° arch (see Q6), i.e. ~9.8° per voussoir — [Block, DeJong & Ochsendorf 2006, Nexus Network Journal 8(2):9–19](https://web.mit.edu/masonry/papers/block_dejong_ochs_NNJ.pdf)

### Inferences
- A rigid-block test for the semicircular arch should use the Milankovitch triple as the reference: `t/r = 0.10748`, haunch hinge at `54.48°` from the crown (i.e. `35.52°` above the springing), thrust `H = 0.06673 γ d r²` (equivalently `H = 0.6209 γ t d r`). Tolerance: the spread between the three analytical formulations is 1.4% in `t/r` and 4.3° in hinge angle; a discrete model with ~16–36 voussoirs should land within ±0.5% of `t/r` (Forgács' ±0.0005 R) and within one voussoir of the hinge angle.
- With a finite number of voussoirs the hinge is forced onto a joint, so the discrete minimum thickness is bounded below by the continuous value only when a joint sits exactly at 54.48°; otherwise a discrete arch of thickness exactly 0.10748 r can be marginally stable. Tests should either place a joint at 54.48° or assert a bracket (`0.1075 ± 0.001`) rather than an equality.
- Friction: for the least-thickness semicircular arch, sliding governs at the springing joint when μ < 0.395; any μ ≥ 0.5 reproduces the Heyman (no-sliding) answer, and μ ≥ 1.415 covers every opening up to 142.5°. Thicker arches need less friction (thrust is flatter), so 0.395 is the worst case for self-weight-only semicircular arches.
- Vertical (bed-parallel) joints raise the requirement by 1.8% (0.10946 vs 0.10748). An engine that lets the user choose joint orientation should be tested against both numbers.

### Gaps
- The exact friction coefficient at which the semicircular (α = 90°) arch switches from purely rotational to mixed collapse in the Cocchetti–Rizzi classification was not recoverable from the garbled PDF text of the 2024 paper (it is read from their Fig. 9), and the 2020 Frattura paper was not accessible. The Forgács analytical value (φ = 21.57°, μ = 0.395) is the only clean threshold found.
- The "0.0819 with non-continuous stereotomy" figure comes from a search summary and was not verified in a primary text.
- The DOI of Cocchetti, Colasante & Rizzi 2011 (Applied Mechanics Reviews) was not confirmed from the manuscript; only the accepted-manuscript URL is given.

---

## Key question 2 — Segmental (variable embrace) and pointed arches

### Takeaway
For circular arches with any half-angle of embrace, Cocchetti et al. publish the full Heyman, CCR and Milankovitch tables (hinge angle, `t/r`, `h`) at 10° steps; those are directly pinnable. For pointed arches the primary tables (Romano & Ochsendorf 2010; Nikolić 2017) were not accessible; only a qualitative result and one worked thrust comparison were recovered.

### Cited findings

**Minimum thickness vs half-angle of embrace α (self-weight, radial joints)** — all from [Cocchetti, Colasante & Rizzi 2011, Tables 2, 5 and 8](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf). Note: in the PDF text extraction the `η` column of each table is displaced one row downward from α = 90° on; the values below were re-aligned using the α = 90° and α = 140° anchor values printed separately in Tables 1, 4 and 7 and the α = 60°/70°/80° CCR values restated in [Cocchetti & Rizzi 2020](https://re.public.polimi.it/retrieve/e0c31c10-4b89-4599-e053-1705fe0aef77/Cocchetti-Rizzi-JOTA-2020(187)-%5BStatic-upper-lower-thrust-and-kinematic-work-balance-stationarity-for-least-thickness-circular-masonry-arch-optimization%5D.pdf) ("α = 60°, β = 39.461°, η = 0.0228482, ĥ = 0.0188518; α = 70°, β = 44.931°, η = 0.0412576, ĥ = 0.0315629; α = 80°, β = 49.959°, η = 0.0686352, ĥ = 0.0478511; α = 90°, β = 54.496°; α = 100°, β = 58.467°; α = 110°, β = 61.733°").

| α (half-embrace) | Heyman β_hinge | Heyman η = t/r | Heyman h | CCR β_hinge | CCR η | CCR h | Milankovitch η | Milankovitch h |
|---|---|---|---|---|---|---|---|---|
| 30° | 20.9898° | 0.00152490 | 0.954859 | 20.8132° | 0.00152531 | 0.954859 | 0.00152531 | 0.954859 |
| 40° | 27.7670° | 0.00472692 | 0.920459 | 27.3561° | 0.00473069 | 0.920461 | 0.0047307 | 0.920459 |
| 50° | 34.3743° | 0.0112735 | 0.877041 | 33.5887° | 0.0112940 | 0.877051 | 0.0112941 | 0.877040 |
| 60° | 40.7902° | 0.0227694 | 0.825056 | 39.4608° | 0.0228482 | 0.825094 | 0.0228489 | 0.825052 |
| 70° | 47.0037° | 0.0410159 | 0.764908 | 44.9308° | 0.0412576 | 0.765024 | 0.0412613 | 0.764887 |
| 80° | 53.0139° | 0.0680052 | 0.696886 | 49.9588° | 0.0686352 | 0.697181 | 0.0686505 | 0.696808 |
| 90° | 58.8293° | 0.105965 | 0.621113 | 54.4963° | 0.107426 | 0.621772 | 0.107478 | 0.620881 |
| 100° | 64.4657° | 0.157463 | 0.537491 | 58.4674° | 0.160584 | 0.538829 | 0.160736 | 0.536902 |
| 110° | 69.9447° | 0.225573 | 0.445657 | 61.7328° | 0.231885 | 0.448199 | 0.232295 | 0.444334 |
| 120° | 75.2920° | 0.314124 | 0.344942 | 64.0072° | 0.326547 | 0.349556 | 0.327607 | 0.342263 |
| 130° | 80.5361° | 0.428043 | 0.234310 | 64.6207° | 0.452593 | 0.242555 | 0.455450 | 0.229424 |
| 140° | 85.7066° | 0.573854 | 0.112302 | 61.5313° | 0.625256 | 0.127517 | 0.634867 | 0.104800 |

- Anchor values printed in the text for α = 140°: Heyman "β_H = 1.49586 rad = 85.7066°, η_H = 0.573854, h_H = 0.112302"; CCR "β_CCR = 1.07392 rad = 61.5313°, η_CCR = 0.625256, h_CCR = 0.127517"; Milankovitch "β_M = 1.02933 rad = 58.9760°, η_M = 0.634867, h_M = 0.104800" — [Cocchetti et al. 2011, Tables 1, 4, 7](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf). (The Milankovitch hinge-angle column for α > 90° could not be realigned reliably; only the 140° anchor is given.)
- The paper notes the over-complete (α > 90°) results differ most between formulations: the hinge-angle discrepancy "occurs more evidently at increasing opening angle of the arch, especially for over-complete arches" — same source, abstract.
- Limit of existence of the purely rotational solution: "Such a purely-rotational solution holds true until half-opening angle limit α = α_s = 2.648388899151005 ≈ 151.7°, where h = h_s pulls back to 0" — [Cocchetti & Rizzi 2024](https://re.public.polimi.it/retrieve/e3e524b4-a174-43e3-97f6-941a2092481d/Cocchetti-Rizzi-STRUCTURES-2024(66)-%5BFinite-friction-least-thickness-self-standing-domains-of-symmetric-circular-masonry-arches%5D.pdf)

**Pointed (Gothic) arches**
- Thrust comparison at equal `t/R = 18%`: "Their minimum and maximum thrusts, expressed as a percentage of their self-weight, are respectively 16% and 25% for the semicircular arch (fig. 4a), and 14% and 23% for the pointed arch (fig. 4b). The self-weight of the two arches is nearly identical … However, the pointed arch thrusts about 15% less than the circular arch." — [Block, DeJong & Ochsendorf 2006, NNJ 8(2)](https://web.mit.edu/masonry/papers/block_dejong_ochs_NNJ.pdf)
- Romano & Ochsendorf 2010, "The mechanics of Gothic masonry arches", Int. J. Architectural Heritage 4(1):59–82: per the search summary it "compared the collapse modes of semi-circular and pointed arches by keeping the opening angle fixed and varying how pointed the arch was" and found that "for pointed-circular arches, the number and arrangement of hinges at limit state vary subject to geometry" — [ResearchGate record](https://www.researchgate.net/publication/233461479_The_Mechanics_of_Gothic_Masonry_Arches) (full text returned HTTP 403).
- Nikolić, "Thrust line analysis and the minimum thickness of pointed masonry arches", Acta Mechanica 228 (2017), DOI 10.1007/s00707-017-1823-6, "estimated the minimum dimensionless thickness of pointed/gothic arches under self-weight, on the verge of becoming a five-hinge mechanism, via the thrust-line method" — [COMPDYN 2021 paper citing it](https://files.eccomasproceedia.org/papers/compdyn-2021/CD21_19608.pdf?mtime=20210910085227); the Springer page itself redirected to a login.
- Alexakis & Makris, "Limit equilibrium analysis of masonry arches", Archive of Applied Mechanics 85:1363–1381 (2015), DOI 10.1007/s00419-014-0963-6 computes the minimum thickness "of a circular masonry arch with mid-thickness radius, R, and embracing angle, β" — [Springer record](https://link.springer.com/article/10.1007/s00419-014-0963-6) (redirected to a login; numbers not retrieved).

### Inferences
- For segmental arches the engine can be pinned against the table above at any α; the CCR/Milankovitch columns are the "true thrust line" answers and the Heyman column is a named alternative. For α ≤ 80° the three formulations agree in `t/r` to better than 1%, so a single tolerance of ±1% is safe there; for α ≥ 100° the formulations diverge (up to 10% at 140°) and the test must say which it targets.
- For a pointed arch no pinnable minimum-thickness number was recovered; the only testable statement is relative (pointed arch of `t/R = 0.18` thrusts about 15% less than the semicircular one with the same `t/R`, min/max thrust 14%/23% vs 16%/25% of self-weight).

### Gaps
- Romano & Ochsendorf 2010 pointed-arch minimum-thickness table (vs eccentricity) and hinge counts: inaccessible (403).
- Nikolić 2017 values: inaccessible.
- Alexakis & Makris 2015 embrace-angle table (variational formulation): inaccessible; the Cocchetti tables cover the same ground for radial joints.

---

## Key question 3 — Arch under a concentrated load (four-hinge mechanism)

### Takeaway
No closed-form worked example for a bare voussoir arch under a point load was recoverable online (Heyman 1982 is not online). Two full-scale laboratory arches with explicit geometry, ¼-span loading, measured collapse load and the four-hinge sequence (Roca & Molins, UPC) are the best substitute, together with the Prestwood field test whose geometry is fully documented.

### Cited findings

**UPC laboratory arches BA1 (segmental) and BA2 (semicircular)** — [Roca & Molins, "Experiments on arch bridges", ARCH'04, CIMNE Barcelona 2004](https://caminstech.upc.edu/sites/default/files/Experiments%20on%20Arch%20Bridges.pdf)
- Geometry: "The two experimental models (Fig. 1), spanning 3.2 m, are characterized by a different rise amounting to 0.65 m in the case of the first segmental one (arch BA1) and to 1.6 m in the case of the semicircular one (arch BA2). The total width is of 1 m in both cases." "Both the arch vault and the spandrel walls are of brick masonry and have thickness of 14 cm."
- Materials: bricks "13.5×28.5×4.5 cm and bed joints 1–1.5 cm thick"; infill "compacted sand … specific weight of 18 kN/m3 (dry specific weight of 15.55 kN/m3)"; joint "Cohesion 0.36 … Initial friction angle 45° Residual friction angle 37.2°"; masonry compression strength 21.0 / 14.0 N/mm².
- Loading: "The axis of the actuator was placed at ¼ of the span (80 cm with respect to the springing). The load was applied on the structure by means of a loading beam spreading over the infill, consisting of a HEB200, 60 cm long steel profile".
- Mechanism: "The arches failed because of the generation of the expectable 4-hinge mechanism; … the hinges located at the loaded section and near the springing closest to the load appeared almost immediately. At about 80%–90% of the maximum load, the hinge at mid-span appeared … the appearance of a fourth hinge at the other springing caused the failure of the arch."
- Results (Table 3, ultimate load / horizontal thrust in kN): Experiment BA1 = 60, BA2 = 95. Limit analysis: "(1) Arch ring (+ backings in BA2) 15 / 24 [BA1], 16.1 / 14 [BA2]; (2) Arch ring + spandrel walls 46 / 58, 48 / 30; (3) Arch ring + spandrel walls + ties 60 / 74 (17), 91 / 52.3 (30)" (the bracketed numbers are the tie forces).

**Prestwood Bridge field test (brick segmental arch, ¼-span line load)**
- "This bridge has a span length equal to 6.55 m, with upper rise 1.42 m. The vault thickness is 0.22 m and comprises a single ring of bricks laid as headers; the fill depth at the crown is 0.165 m and the total bridge width is 3.800 m." Load "applied at the quarter-span of the bridge. The strip was 0.30 m wide". "The fill density is 20 kN/m3 and the masonry density is 20 kN/m3." "The first visible sign of damage occurred at Pf = 173 kN and the experimental collapse load was Pu = 228 kN." — [Drosopoulos, Stavroulakis & Massalas, "On the collapse analysis of single span masonry/stone arch bridges with fill interaction" (ARCH conference paper)](https://arch-bridges.fzu.edu.cn/__local/8/83/32/FFD47B4C2992F51ED7390BB691E_B7B56259_B3646.pdf?e=.pdf)
- Independent restatement with more detail: span 6.55 m, rise 1.428 m, ring 0.22 m, fill at crown 0.165 m, width 3.80 m including two 0.38 m spandrel walls, east springing 0.08 m lower than west; masonry density 2040 kg/m³, E = 4140 MPa, f_c = 4.5 MPa; backfill 2040 kg/m³, φ = 37°, c = 7 kPa; hinges formed in the order "(3) below loading, followed by (2), (4), and (1)"; collapse 228 kN — [IOP Engineering Research Express, "Collapse analysis of a masonry arch bridge using the applied element method", DOI 10.1088/2631-8695/ad6ad8](https://iopscience.iop.org/article/10.1088/2631-8695/ad6ad8)
- Primary report: Page, J. (1987) "Load tests to collapse on two arch bridges at Preston, Shropshire and Prestwood, Staffordshire", TRRL Research Report RR 117: Prestwood "Brick segmental arch (220 mm thick) … Load Position: 1/4-span … Collapse Load: 228 kN … Four hinged mechanism" — [TRID record](https://trid.trb.org/view/283016)

**General statements**
- "Heyman's plastic analysis is based on the formation of four hinges, causing failure of any loading position thereby allowing the critical position to be identified, usually near to the quarter span." — [search summary citing the RING/Gilbert–Melbourne method](https://www.sciencedirect.com/science/article/pii/S235271101830061X) (Stockdale et al., "Kinematic collapse load calculator: Circular arches", SoftwareX 2018 — page returned 403; the tool itself is a candidate oracle).

### Inferences
- The UPC BA1/BA2 pair is the cleanest concentrated-load benchmark with full geometry: a rigid-block engine modelling ring + fill weight only should reproduce roughly the "arch ring" limit-analysis row (15 kN for BA1, 16 kN for BA2 with backings), not the experimental 60/95 kN, which include spandrel-wall and tie action. The measured 4-hinge order (loaded section, near springing on loaded side, crown, far springing) is a mechanism-level assertion independent of load magnitude.
- Prestwood is the standard field benchmark for ring + resistant fill; a ring-only rigid-block model will under-predict 228 kN substantially (the fill is what raises the quarter-span capacity), so its value to the engine is the hinge pattern and the load position, not the load.

### Gaps
- No closed-form dimensionless collapse load `P/(γ t d r)` vs load position for a bare semicircular or segmental voussoir arch was found online; Heyman (1982) "The Masonry Arch" and Gilbert & Melbourne (1994) "Rigid-block analysis of masonry structures" (The Structural Engineer 72(21)) are offline. The SoftwareX calculator paper is open access but blocked here.

---

## Key question 4 — Full-scale and model arch-bridge tests with reported collapse loads

### Takeaway
The TRL/TRRL programme (1984–1994) and the Bolton tests give reported collapse loads with partial geometry; ring-only rigid-block models will not match these loads because fill and spandrels carry 50–90% of capacity, so they are useful as upper-bound sanity checks and for mechanism type. Small-scale dry-joint tests (Ochsendorf's spreading arch; Royles & Hendry model arches) are closer to a rigid-block idealisation.

### Cited findings

**TRL/TRRL field tests** (all loads are total line load across the bridge width)
- Programme: "Between 1984 and 1994, TRRL … carried out eight tests on masonry and broken stone arches" — [search summary, Sarhosis et al. review](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf)
- Table 1 of the review ("Failure modes of test bridges (after Page 1993 & 1995)") lists, with square span and failure load: Prestwood 6.55 m / 228 kN / "Formation of four hinges"; Bolton 6 m model 6.00 m / 1170 kN / four hinges; Shinafoot 6.16 m / 2500 kN; Torksey 4.90 m / 1080 kN; Bargower 10.36 m / 5600 kN; Preston 5.18 m; Strathmashie 9.42 m / 1325 kN; Barlae 9.87 m / 2900 kN; failure modes listed include "Formation of four hinges", "Three pinned snap through", "Crushing failure below load", "Not well defined, material falling out of existing longitudinal crack" and, for the skewed Barlae, "Three pinned snap through followed by shear failure in the spandrel" — [Sarhosis, De Santis & de Felice, "A review of experimental investigations and assessment methods for masonry arch bridges" (accepted manuscript)](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf). The PDF table's column alignment is corrupted in extraction; the per-row rise/thickness/fill assignments could not be read reliably.
- Bridgemill: "a shallow single arch bridge made of 62 sandstone blocks with a significant span of 18.29 m and an 8.30 m width; the fill in crown is only 20 cm thick"; "derived 50% of its strength from the arch barrel alone" — same source. Collapse load "3100 kN" — [Holmström 2010, Chalmers MSc thesis, Table 5.1 after Crisfield et al. 1987](https://publications.lib.chalmers.se/records/fulltext/134461.pdf); search summary also gives "361 kN/m" per unit width — [search result](https://publications.lib.chalmers.se/records/fulltext/134461.pdf). Primary: Hendry, Davies & Royles (1985) "Test on stone masonry arch at Bridgemill–Girvan", TRRL Contractor Report 7 — [ARCH'04 reference list](https://caminstech.upc.edu/sites/default/files/Experiments%20on%20Arch%20Bridges.pdf)
- Bargower: "semi-circular profile, built up of regular, cut to shape, sandstone voussoirs … span of 10 m, rise at mid span equal to 5.18 m and arch thickness 0.58 m"; "derived only 8% of its strength from the barrel" — [Sarhosis et al.](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf). Collapse load 5600 kN — [Holmström 2010](https://publications.lib.chalmers.se/records/fulltext/134461.pdf). Primary: Hendry et al. (1986) "Test on masonry arch bridge at Bargower", TRRL Contractor Report 26.
- Preston: "Elliptical sandstone arch (360 mm thick) … Load Position: 1/3-span … Collapse Load: 2,110 kN … Crushing of the arch ring beneath the load line" — [TRID record of Page 1987, RR 117](https://trid.trb.org/view/283016); Holmström quotes 2100 kN.
- Barlae: "In the full scale test the failure load was 2900 kN" (Abaqus simulation 2750 kN) — [Holmström 2010](https://publications.lib.chalmers.se/records/fulltext/134461.pdf)
- MEXE comparison (serviceability method, far below collapse): Bargower 1285 kN, Bridgemill 363 kN, Preston 597 kN vs experimental 5600 / 3100 / 2100 kN — same source, Table 5.1.
- RING (rigid-block) back-analysis vs experiment, per LimitState: Bridgemill 100%, Preston 92%, (one further bridge 90%) — same source.
- Torksey and Shinafoot primary: Page, J. (1988) "Load tests on two arch bridges at Torksey and Shinafoot", TRRL Research Report 159 — [ARCH'04 reference list](https://caminstech.upc.edu/sites/default/files/Experiments%20on%20Arch%20Bridges.pdf)

**Bolton Institute large-scale tests (Melbourne & Gilbert)**
- Melbourne & Walker (1990): "full scale model test on 6 m span brickwork arch bridge … A diffused four-hinge mechanism took place, which was facilitated by ring separation" — [Sarhosis et al.](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf) (1170 kN in Table 1).
- Melbourne & Gilbert (1992, for British Rail): "Two 3 m span arch barrels with two brick rings … the introduction of ring separation reduced the arch strength by 33%. Two larger models with 5 m span and four brick rings … the strength of the arch reduced by 71%." — same source.
- Melbourne & Gilbert (1995), The Structural Engineer 73(3): "six large scale model tests … (i) two bridges with 2 rings and 3 m span (with detached spandrel and wing walls), (ii) two bridges with 2 rings and 3 m span (with attached spandrel and wing walls), and (iii) two bridges of 4 rings and 5 m span … The failure load of the bridges with the built-in defect of ring separation were 1.5–3 times lower … the strength of the bridge models with separated spandrel walls was about 25% lower" — same source; [IStructE record](https://www.istructe.org/journal/volumes/volume-73-(published-in-1995)/issue-3/the-behaviour-of-multiring-brickwork-arch-bridges/)
- Bridges 3-1 and 5-1 (single span, used as reference cases): experimental failure loads "540" kN (3-1) and "1720" kN (5-1); FE 545 / 1722 kN; RING 442 / 1915 kN; materials: backfill unit weight 22.2 kN/m³, bricks 23.7 kN/m³, backfill friction angle 60°, crushing strength 20 N/mm² — [KTH/DiVA MSc thesis "Load carrying capacity assessment of stone masonry arch bridges", Tables 4.1 and 4.2](https://www.diva-portal.org/smash/get/diva2:1880201/FULLTEXT01.pdf). Search summary: "For Bridge 3-1, the experimental collapse load was 540 kN." Geometry of 3-1 is cited in the Sheffield follow-on tests as "3 m span, 0.75 m rise and 21.5 cm thickness (one brick)" — [Sarhosis et al.](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf)
- Multi-span: Melbourne, Gilbert & Wagstaff (1995/1997), three 3-span large-scale brickwork bridges; "the presence of the spandrel walls … led to an increase of 70% of ultimate load" — same source.

**Model-scale tests**
- Royles & Hendry (1991): "tested 24 model arches … The maximum load was 100 kN on the arch with fill and no spandrel walls, 150 kN with unrestrained spandrel walls, and, finally, 320 kN with restrained spandrel walls" — [Sarhosis et al.](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf)
- Hughes (1998) 1/6-scale centrifuge models "replicated large scale tests carried out at Bolton, UK (Gilbert 1993). The failure load and formation of hinges of the two experimental investigations showed very good agreement." — same source.
- Historical rigid-voussoir experiments: Barlow (1846) "a series of tests on model voussoir arches" demonstrating multiple admissible thrust lines; Pippard (1930s) "a series of 23 tests on concrete voussoir arches" — same source.
- Ochsendorf (2006) spreading-support model: "16 voussoirs cast as individual concrete blocks with a 50-mm radial thickness, with a mean radius R equal to 220 mm and a thickness–radius ratio (t/R) of 0.23"; "The experimental collapse took place for a displacement value of 30 mm, resulting in a span increase equal to 15.4%" — [search summary of Coccia, Di Carlo & Rinaldi 2015, Int. J. Advanced Structural Engineering, DOI 10.1007/s40091-015-0101-x](https://link.springer.com/article/10.1007/s40091-015-0101-x) (page itself redirected to login).

### Inferences
- For a rigid-block engine the usable bridge benchmarks are the ones with a published ring-only or fill-as-dead-load analysis to compare against (Prestwood, Bridgemill via RING ≈ 100%), plus the mechanism type. Matching the raw collapse load of deep arches (Bargower: 8% from the barrel) is not a meaningful target for a voussoir-only model.
- Ring separation and spandrel detachment are the two multi-ring/3D effects that change load by 25–70%; if the engine models rings as separate block layers with a frictional interface, the Bolton 33%/71% reductions are a pinnable ratio.

### Gaps
- Page (1993) "Masonry arch bridges — a state of the art review" (TRL/HMSO) itself was not accessible; the collated table is second-hand and its rise/thickness/fill columns were unreadable in extraction. Per-bridge rise and ring thickness for Shinafoot, Torksey, Strathmashie and Barlae remain unconfirmed.
- Melbourne & Gilbert 1995 full geometry of bridges 3-1/5-1 (rise, ring thickness of the 5 m span, fill depth, load position) not retrieved; only the 3 m geometry via the Sheffield description.
- No 3D-printed scale-model arch benchmark with reported collapse loads was found in this pass.

---

## Key question 5 — Arches on spreading supports

### Takeaway
Ochsendorf's 2006 result for a semicircular voussoir arch is a three-hinge mechanism (extrados hinge at the crown, two intrados hinges at the haunches) whose hinges migrate as the span grows, with collapse when a fourth/fifth hinge appears; his static procedure predicts a span increase of 16.9% for the t/R = 0.23 test arch, the experiment gave 15.4%, and a later kinematic treatment gives 16.53%.

### Cited findings
- Citation: Ochsendorf, J.A. 2006. "The Masonry Arch on Spreading Supports." The Structural Engineer, Vol. 84, 2: 29–36 — [NNJ reference list](https://web.mit.edu/masonry/papers/block_dejong_ochs_NNJ.pdf); [IStructE record](https://www.istructe.org/journal/volumes/volume-84-(published-in-2006)/issue-2/the-masonry-arch-on-spreading-supports/) (403 on fetch).
- Scope per search summary: "determine the horizontal support movements necessary to cause collapse for a rigid-block arch, using computer programs to track the changing equilibrium conditions as the supports move, with results verified experimentally"; "three hinges are formed — one at the extrados in the keystone and two at the intrados. Ochsendorf's static procedure found a span increase of 16.9%." — [search summary](https://www.researchgate.net/publication/284800254_The_masonry_arch_on_spreading_supports)
- Experiment and comparison: arch of "16 voussoirs … 50-mm radial thickness … mean radius R equal to 220 mm … t/R of 0.23"; "experimental collapse took place for a displacement value of 30 mm, resulting in a span increase equal to 15.4%. Additionally, the analytic ultimate displacement was equal to 32.24 mm, related to a span increase of 16.53%, with Ochsendorf's static procedure yielding a span increase of 16.9%"; the same study ran "t/R values of 0.15, 0.2, 0.25 and 0.3" — [search summary of Coccia, Di Carlo & Rinaldi 2015, DOI 10.1007/s40091-015-0101-x](https://link.springer.com/article/10.1007/s40091-015-0101-x)
- Mechanism description and hinge migration: "where the line touches the extremities of the structure, hinges are most likely to form … three hinges create a statically determinate structure with a unique equilibrium solution … Failure occurs when there are more than three hinges, i.e. when the thrust line touches the boundaries of the structure in more than three places"; the worked figure is "an arch on spreading supports with a t/R = 15% and a total angle of embrace of 160° (a) initially in its minimum thrust state, (b) at an intermediate state, and (c) right at collapse"; as the arch spreads "the horizontal thrust, in the system increase[s]" — [Block, DeJong & Ochsendorf 2006, NNJ 8(2):9–19](https://web.mit.edu/masonry/papers/block_dejong_ochs_NNJ.pdf)
- The hinge-migration point is also made in the search summary: "hinges can move from their initial locations as the supports spread" — [search summary](https://www.researchgate.net/publication/284800254_The_masonry_arch_on_spreading_supports)
- Cocchetti et al. note that Ochsendorf (2006) independently tabulated the minimum-thickness/hinge results "fully consistent with" their CCR solution — [Cocchetti et al. 2011](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf)

### Inferences
- A pinnable test: 16-voussoir semicircular arch, R = 220 mm, t = 50 mm (t/R = 0.227), concrete blocks, supports pulled apart symmetrically; expected collapse at span increase between 15.4% (measured) and 16.9% (Ochsendorf static), i.e. 30–33 mm of total spread for a 440 mm span; mechanism before collapse is 3 hinges (crown extrados, two intrados), with the intrados hinges moving outward/downward as spread increases; collapse when a further hinge forms.
- Because the result is quasi-static and dry-jointed, friction is not expected to govern (the intrados hinges carry near-normal thrust); the sensitivity to discretisation is that hinges jump between joints, so an engine with 16 voussoirs should reproduce the experiment better than the continuous theory.

### Gaps
- Ochsendorf's table of maximum span increase vs t/R (and vs embrace angle), and the migrating hinge angles, were not retrieved (IStructE and ResearchGate both 403). The Coccia et al. values for t/R = 0.15, 0.2, 0.25, 0.3 were likewise not retrieved.

---

## Key question 6 — Leaning / tilted arches (tilting-table collapse)

### Takeaway
The MIT equivalent-static result is that a 16-voussoir arch with embrace 157.5° and t/R = 0.15 collapses at a constant horizontal acceleration of 0.34 g (tilt ≈ 18.8°), forming a four-hinge mechanism; the experimental tilt-table campaigns (DeJong 2009; DeJong, De Lorenzis, Adams & Ochsendorf 2008) exist but their measured angles were not retrievable.

### Cited findings
- "The arch of sixteen voussoirs, inclusion angle (β) of 157.5 degrees, and thickness/radius ratio of 0.15 would collapse at a constant horizontal acceleration of 0.34*g (fig. 6a). A similar arch (t/r = 0.15, β = 120°) on buttresses with relative height and width dimensions of 1.5 and 0.5 times the vault span, respectively, would collapse at a constant horizontal acceleration of 0.13*g (fig. 6b)." Method: "Tilting the ground surface effectively applies a vertical acceleration (gravity, g) and a horizontal acceleration (λ*g, where λ = tan(α) and α is the ground surface inclination angle). The ground surface is tilted until the thrust line cannot be contained within the structure: the point where it passes through the exterior surface of the structure at four locations." — [Block, DeJong & Ochsendorf 2006, NNJ 8(2)](https://web.mit.edu/masonry/papers/block_dejong_ochs_NNJ.pdf)
- Caveats stated by the authors: the values "are conservative in that they assume an infinite duration of loading" and "unconservative because the possibilities of local crushing and sliding have not been evaluated" — same source.
- Experimental campaigns: DeJong, De Lorenzis, Adams & Ochsendorf, "Rocking stability of masonry arches in seismic regions", Earthquake Spectra 24(4):847–865 (2008), DOI 10.1193/1.2985763 — [Crossref record](https://chooser.crossref.org/?doi=10.1193%2F1.2985763); DeJong, M.J. (2009) "Seismic assessment strategies for masonry structures", PhD thesis, MIT, handle 1721.1/49538, whose abstract states the arch rocking response was characterised with "rigid body dynamics and an experimental testing program" — [MIT DSpace](https://dspace.mit.edu/handle/1721.1/49538) (PDF download blocked by a bot challenge on both DSpace and core.ac.uk).
- Later tilt-table test on semicircular URM arches: "Three semi-circular unreinforced masonry arches are tested to identify collapse mechanisms and seismic capacity" via "quasi-static rotation until collapse" — [Tarhan et al. 2025, Earthquake Engineering & Structural Dynamics, DOI 10.1002/eqe.4294](https://onlinelibrary.wiley.com/doi/10.1002/eqe.4294) (403 on fetch).
- Related variational solution for the same problem: Alexakis & Makris, "Limit equilibrium analysis and the minimum thickness of circular masonry arches to withstand lateral inertial loading", Archive of Applied Mechanics (2014), DOI 10.1007/s00419-014-0831-4, finding that "a circular arch becomes a mechanism with vertical ruptures when subjected to a horizontal ground acceleration that is slightly lower than the horizontal acceleration needed to create a mechanism with radial ruptures" — [search summary](https://link.springer.com/article/10.1007/s00419-014-0831-4)
- Pointed-arch tilt tests identified "two types of collapse mechanisms …: pure rotation and rotation/sliding combination" — [search summary of Engineering Structures 2022 paper](https://www.sciencedirect.com/science/article/abs/pii/S0141029622004540)

### Inferences
- Pinnable analytical test: semicircular-type arch, embrace 157.5°, t/R = 0.15, 16 voussoirs, no fill, radial joints; apply lateral acceleration (or tilt the base) and assert collapse at λ = 0.34 (tilt = arctan 0.34 = 18.8°) with a four-hinge non-symmetric mechanism. The 0.13 g value for the buttressed version is a second, system-level check.
- Alexakis & Makris' remark implies the result depends on joint orientation at the ~percent level; a test should fix radial joints.

### Gaps
- Measured collapse tilt angles from DeJong (2008/2009) and from Tarhan et al. (2025) were not retrievable.

---

## Key question 7 — Per-benchmark pinnable numbers, discretisation and friction sensitivity, and whether sliding governs

### Takeaway
Across the catalogue, the formulation spread is ~1–2% in minimum thickness and ~4° in hinge angle; the discretisation error of a ~16–40-voussoir model is ≤0.5% in thickness and about one joint spacing in hinge position; sliding governs only below μ ≈ 0.4 for the least-thickness semicircular arch and is irrelevant for the bridge tests, which were governed by hinging (or crushing where the fill was deep).

### Cited findings
- Formulation spread (semicircular): `t/r` = 0.105965 (Heyman) / 0.107426 (CCR) / 0.107478 (Milankovitch) / 0.10946 (vertical joints); hinge 58.83° / 54.50° / 54.48° / 54.92° — [Cocchetti et al. 2011](https://aisberg.unibg.it/retrieve/472995cf-86c8-4292-9661-8cc57ef17bd9/PaperCCRAMR11-Accepted.pdf); [Makris & Alexakis 2015](https://www.ijbe.net/issues/volumes/item/download/104_225d6c04d4d044dd4b08acc37f969f22)
- Discretisation: 3DEC DEM precision "± 1.5° and ± 0.0005 · R" against Milankovitch — [Forgács et al. 2017](https://eprints.whiterose.ac.uk/id/eprint/144597/3/Manuscript_ForgacsSarhosisBagi_EngStr_revised_marked_20161218.pdf)
- Friction: least-thickness semicircular arch slides at the springing if φ < 21.57° (μ < 0.395); DEM slides at 21°, stands at 22° — same source. For skewed arches the required friction is much higher ("32.5 degrees" for the false-skew method at 45° skew; "27.5 degrees" for helicoidal/logarithmic) and the hinge lines become zig-zag with local sliding — same source.
- Friction bound for all openings ≤ 142.5°: μ ≥ 1.41527 — [Cocchetti & Rizzi 2020](https://re.public.polimi.it/retrieve/e0c31c10-4b89-4599-e053-1705fe0aef77/Cocchetti-Rizzi-JOTA-2020(187)-%5BStatic-upper-lower-thrust-and-kinematic-work-balance-stationarity-for-least-thickness-circular-masonry-arch-optimization%5D.pdf); purely sliding modes excluded for φ > ~70° for all openings — [Cocchetti & Rizzi 2024](https://re.public.polimi.it/retrieve/e3e524b4-a174-43e3-97f6-941a2092481d/Cocchetti-Rizzi-STRUCTURES-2024(66)-%5BFinite-friction-least-thickness-self-standing-domains-of-symmetric-circular-masonry-arches%5D.pdf)
- Bridge tests: failure modes were hinging (Prestwood, Bolton model, Shinafoot, Torksey), snap-through (Bargower, Barlae) or crushing (Preston, Strathmashie); none is reported as sliding — [Sarhosis et al. Table 1](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf); [TRID](https://trid.trb.org/view/283016)
- Fill share of capacity: Bridgemill 50% from the barrel, Bargower 8% — [Sarhosis et al.](https://eprints.whiterose.ac.uk/id/eprint/146294/2/SARHOS~1.pdf)
- Tilt/lateral load: sliding "not evaluated" in the 0.34 g result — [NNJ](https://web.mit.edu/masonry/papers/block_dejong_ochs_NNJ.pdf)

### Inferences — recommended pins (SI, dimensionless where the source is)
1. **Semicircular self-weight (Milankovitch, radial joints):** `t/r = 0.10748 ± 0.0005`; haunch hinge `54.5° ± one joint`; `H = 0.0667 γ d r²`; five-hinge symmetric mechanism (crown extrados, haunch intrados ×2, springing extrados ×2). Variant pins: Heyman `0.10597`/`58.83°`; vertical joints `0.10946`/`54.92°`.
2. **Segmental arches:** the α-table in Q2 (e.g. α = 60°: `t/r = 0.02285`, hinge 39.46°, `h = 0.8251`; α = 120°: `t/r = 0.3276`, hinge 64.0°, `h = 0.3496`).
3. **Friction switch:** least-thickness semicircular arch stands for μ = tan 22° = 0.404, slides at μ = tan 21° = 0.384 (springing joint).
4. **Spreading supports:** 16 voussoirs, R = 0.220 m, t = 0.050 m → collapse at 15.4–16.9% span increase, 3-hinge pre-collapse mechanism.
5. **Lateral acceleration:** 16 voussoirs, 157.5°, t/R = 0.15 → λ_collapse = 0.34 (four hinges).
6. **Point load with fill (ring-only limit analysis):** UPC BA1 (span 3.20 m, rise 0.65 m, t = 0.14 m, width 1.0 m, ¼ span) ≈ 15 kN ring-only, 60 kN measured; BA2 (rise 1.60 m) ≈ 16 kN ring+backings, 95 kN measured; four-hinge order loaded-section → near springing → crown → far springing.
7. **Field bridge (upper bound / mechanism only):** Prestwood 228 kN at ¼ span, four hinges; Bridgemill 3100 kN; Bargower 5600 kN; Preston 2110 kN (crushing, ⅓ span); Bolton 3-1 540 kN, 5-1 1720 kN.

### Gaps
- No source gives the discretisation sensitivity of the minimum thickness as an explicit function of the number of voussoirs (only the DEM end-point precision).
- The exact α = 90° friction threshold in the Cocchetti–Rizzi classification, the Romano & Ochsendorf pointed-arch table, the Alexakis & Makris embrace-angle table, DeJong's measured tilt angles, Ochsendorf's spread-vs-t/R table, and a closed-form point-load collapse load for a bare voussoir arch all remain open (paywalled or bot-blocked in this pass).
