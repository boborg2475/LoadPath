# Timber joints and bonded masonry: known-answer benchmark cases for a rigid-block / directional-joint engine

Scope note. Everything below is for an engine where pieces are rigid bodies and a joint carries compression, shear (Coulomb friction + cohesion), tension and bending about the joint section. Members never break internally, so the cases are joint-level (one fastener, one bed joint) or assembly-level (a wall, a lintel, a frame) with exact or measured answers. SI throughout: N, Pa (MPa = 1e6 Pa), m. Where a source gives N/mm², that is identically MPa. Characteristic (5 % fractile, "k") and mean values are separated explicitly; the engine wants means.

Convention for "derived" items: a few worked numbers were computed in this session from the cited code formulas because no public page shows the full arithmetic. Each is marked **[derived]** and placed under *Inferences*; the inputs and formulas are cited, the arithmetic is mine and should be re-checked by the implementer.

---

## KQ1. Eurocode 5 Johansen (EN 1995-1-1 §8) worked capacities: nail, screw, bolt per shear plane; nail and screw withdrawal

### Takeaway
Published, fully-numbered EC5 connection examples that are freely downloadable exist in Swedish Wood *Design of timber structures, Vol. 3* (screw into glulam 4.83 kN char./shear plane; 4 mm ring nail 1.607 kN; M16 bolt 12.9 kN) and on Structural Basics (nail withdrawal 0.42 kN; screw withdrawal 1.74 kN). The published cases are steel-to-timber; a timber-to-timber nailed case had to be derived from the same clauses and lands at about 0.83 kN characteristic per shear plane for a 3.1 mm nail in C24.

### Cited Findings

**Formulas as printed in a worked example (Swedish Wood Vol. 3, Example 4.1, screw in GL30c, steel plate):**
- Inputs: "wood screws have a length of 60 mm, a diameter of 8 mm and an ultimate strength of 410 MPa", steel plate t = 8 mm, "t1 = ls - t = 52 mm", force at 45° to grain, "ρk = 390 kg/m3", "kmod = 0,65", "γM = 1,3" — [Swedish Wood, Design of timber structures Vol. 3, 2022 ed., Example 4.1, pp. 19–20](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
- "My,Rk = 0,3 fu d^2,6 ef … My,Rk = 2,74 · 10^4 Nmm" (i.e. 27.4 N·m) — [same, p. 20](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
- "fh,0,k = 0,082(1 - 0,01def) ρk … fh,0,k = 29,42 MPa"; "k90 = 1,35 + 0,015def = 1,47"; "fh,α,k = 23,82 MPa" — [same, p. 20](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
- "Failure mode c Fv,Rk,c = … = 9,91 kN; Failure mode d Fv,Rk,d = … = 4,83 kN; Failure mode e Fv,Rk,e = … = 5,26 kN; Characteristic capacity Fv,Rk = min(…) = 4,83 kN; Design capacity Fv,Rd = … = 2,42 kN" — [same, p. 20](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
  - Session check of these numbers: mode c = f_h,α,k·t1·d = 23.82 × 52 × 8 = 9 909 N; mode d = f·t1·d·[√(2 + 4M_y/(f·d·t1²)) − 1] = 9 909 × 0.4876 = 4 832 N; mode e = 2.3·√(M_y·f·d) = 2.3 × √(27 440 × 23.82 × 8) = 5 259 N. All three reproduce the printed values, so the printed example is internally consistent with EN 1995-1-1 eq. (8.10) thick-plate single shear.

**Nail, steel-to-timber (Swedish Wood Vol. 3, Example 4.2):**
- "Annular ring shanked nails of the dimension 40-4,0 mm … head diameter of 6 mm … steel plate is chosen as 5 mm … t1 = 40-5 = 35 mm", "Nail material, fu = 600 MPa", "ρk = 390 kg/m3" → "The lowest capacity of the three failure modes occurs for failure mode II, that is Fv,Rk = 1,607 kN/nail"; rope-effect contribution "only 6 percent" (limit 50 % for ring-shank nails) — [Swedish Wood Vol. 3, Example 4.2, pp. 21–23](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
- The same book prints the non-predrilled embedment formula with a number: "fh,k = 0,082 ρk d^-0,3 … fh,k = 21,1 MPa" for d = 4 mm, ρk = 390 kg/m³ (Example 4.4, column base with 60-4.0 ring nails, fu = 600 MPa) — [same, p. 28](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)

**Bolt, double shear steel-to-timber (Swedish Wood Vol. 3, Example 4.3):**
- Inputs: "thickness of the steel plate t = 8 mm", "bolt diameter d = 16 mm", "tensile strength of bolt fu = 360 MPa", "glulam GL30c, ρk = 390 kg/m3", timber member 90 mm (figure 4.5), "double shear steel-to-timber joints with t = 0,5d, failure mode j and k" → "the characteristic shear capacity is 12,9 kN/bolt"; fictitious washer "min(128; 416) = 64 mm" for the rope term; with group effect "Without the group effect the connection could have been established using 250/14,9 = 17 bolts" (design value 14.9 kN/bolt incl. rope) — [Swedish Wood Vol. 3, Example 4.3, pp. 24–25](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
  - Session check: M_y,Rk = 0.3 × 360 × 16^2.6 = 146 000 N·mm; f_h,0,k = 0.082(1 − 0.16) × 390 = 26.9 MPa; mode k = 1.15·√(2·M_y·f_h·d) = 1.15 × √(2 × 146 000 × 26.9 × 16) = 12.9 kN — reproduces the printed value; mode j = 0.5·f_h·t2·d = 0.5 × 26.9 × 90 × 16 = 19.3 kN, so mode k (one plastic hinge per shear plane) governs.

**Nail withdrawal, EN 1995-1-1 §8.3.2 (Structural Basics worked example):**
- Inputs: "d = 3.1 mm", "ln = 75 mm", "dh = 7.5 mm", ring-shank ("non-smooth") nail, OSB 20 mm ρk = 550 kg/m³ over rafter ρk = 350 kg/m³, "t2 = 55 mm", kmod = 0.9, γM = 1.3 — [Structural Basics, Tensile Capacity of Nails](https://www.structuralbasics.com/?p=6714)
- "fax.k = 20 ⋅ 10⁻⁶ (ρk/(kg/m³))² N/mm² = 2.45 N/mm²"; "fhead.k = 70 ⋅ 10⁻⁶ (ρk/(kg/m³))² N/mm² = 8.6 N/mm²"; "Fax.Rk = min(fax.k ⋅ d ⋅ t2; fhead.k ⋅ dh²) = 0.42 kN"; "Fax.Rd = kmod ⋅ Fax.Rk/γM = 0.29 kN" — [same](https://www.structuralbasics.com/?p=6714)
  - Session check: 2.45 × 3.1 × 55 = 418 N (withdrawal governs); head pull-through 8.575 × 7.5² = 482 N. In SI: f_ax,k = 2.45 MPa = 2.45e6 Pa; F_ax,Rk ≈ 418 N.

**Screw withdrawal, EN 1995-1-1 §8.7.2 (Structural Basics worked example, ETA-style formula):**
- Inputs: "d = 2.5 mm", "dh = 8.0 mm", "ln = 50 mm", OSB t1 = 20 mm ρk = 550, rafter ρk = 350, "t2 = 30 mm", α = 90°; "lef = t₂ – d = 27.5 mm" — [Structural Basics, Tensile Capacity of Screws](https://www.structuralbasics.com/tensile-capacity-of-screws-eurocode-tutorial/)
- "fax.k.1 = 3.6 ⋅ 10⁻³ ⋅ ρk^1.5/(1.5 ⋅ cos²(90°) + sin²(90°)) N/mm² = 46.4 N/mm²" (OSB) and "= 23.6 N/mm²" (rafter); "Fax.Rk = min(fax.k.1, fax.k.2) (π ⋅ d ⋅ lef)^0.8 = 1.74 kN"; "Fax.head.k = … = 1.44 kN"; "Fax.Rd = … = 1.0 kN" — [same](https://www.structuralbasics.com/tensile-capacity-of-screws-eurocode-tutorial/)
  - Caution: the 3.6e-3·ρk^1.5 expression is the pre-2008 / ETA form. The EN 1995-1-1:2004/A1:2008 §8.7.2 form is f_ax,k = 0.52·d^-0.5·l_ef^-0.1·ρk^0.8 (valid 6 mm ≤ d ≤ 12 mm, 0.6 ≤ d1/d ≤ 0.75) with F_ax,α,Rk = n_ef·f_ax,k·d·l_ef·k_d/(1.2cos²α + sin²α), k_d = min(d/8; 1). A regression-based form of this ("Fax,k = 0.52 d^0.52 t^0.9 ρ^0.8 … (Blass et al. 2006) and adopted in EN 1995-1-1 (2008) (§8.7.2)") is quoted in [J. Wood Science (2020) 10.1186/s10086-020-01926-8](https://jwoodscience.springeropen.com/articles/10.1186/s10086-020-01926-8). The two forms give different numbers; implementers must choose one and say which.

**Material inputs used by the examples:**
- C24: "The characteristic value of the compressive strength is fc,90,k = 2,5 MPa" (Example 3.2, C24 joist); C24 fm,k = 24 MPa (Example 3.1); GL30c "fc,90,k = 2,5 N/mm2", "ρm = 430 kg/m3", ρk = 390 kg/m³ — [Swedish Wood Vol. 3](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
- A rafter of ρk = 350 kg/m³ is used as the softwood density in the Structural Basics examples (this is the EN 338 C24 characteristic density) — [Structural Basics](https://www.structuralbasics.com/?p=6714)
- Johansen's original: Johansen, K.W. (1949) "Theory of Timber Connections", IABSE Publications 9, pp. 249–262, Bern — citation confirmed via [Blass & Colling, INTER 48-07-03 reference list](https://publikationen.bibliothek.kit.edu/1000049999/3752771) and [ITB journal](https://journals.itb.ac.id/index.php/jts/article/view/2664)
- Blass & Sandhaas, *Timber Engineering – Principles for Design*, KIT Scientific Publishing 2017, open access, DOI 10.5445/KSP/1000069616, CC BY-SA 4.0 — [KIT record](https://publikationen.bibliothek.kit.edu/1000069616) (not opened in this session; the Johansen chapter is the authoritative derivation to cite).

### Inferences
- **[derived] Timber-to-timber nailed joint, single shear, EN 1995-1-1 eq. (8.6) + §8.3.1.1.** Inputs chosen to be code-legal: smooth round wire nail d = 3.1 mm, L = 75 mm, fu = 600 MPa; both members C24 with ρk = 350 kg/m³; t1 = 35 mm (head side), point-side penetration t2 = 40 mm (≥ 12d = 37.2 mm, so withdrawal counts); not predrilled.
  - f_h,k = 0.082·350·3.1^-0.3 = 20.4 MPa (both members, β = 1); M_y,Rk = 0.3·600·3.1^2.6 = 3 411 N·mm.
  - Modes: (a) 2 218 N; (b) 2 535 N; (c) 1 037 N; (d) 875 N; (e) 975 N; (f) 1.15·√(2·M_y·f_h·d) = 1.15 × 657.5 = 756 N. Mode (f) governs (two plastic hinges).
  - Rope: F_ax,Rk = min(20e-6·350²·3.1·40 = 304 N; 70e-6·350²·7.5² = 482 N) = 304 N → F_ax,Rk/4 = 76 N (≤ 15 % of Johansen part for smooth nails, 0.15 × 657 = 99 N, so fully counted).
  - **F_v,Rk ≈ 756 + 76 ≈ 0.83 kN per nail per shear plane** (characteristic). The pure Johansen plastic-hinge load without EC5's 1.15 calibration factor and without rope is 0.66 kN.
  - What a test would pin: for the rigid-block engine the fastener is a point joint whose lateral strength is F_v and axial strength F_ax; a two-member lap with one nail pulled in shear should release at F_v (0.66–0.83 kN char.; mean likely 0.9–1.1 kN, see KQ2 ratios), and pulled apart axially at F_ax ≈ 0.30 kN (char., smooth nail) — the lateral/axial ratio ≈ 2.5–3 is the thing to get right.
- For the engine's *mean* values, re-run with ρ_mean instead of ρk: for C24 the mean density (EN 338 lists 420 kg/m³ — not re-verified this session) raises f_h to ≈ 24.5 MPa and the mode (f) Johansen part to ≈ 720 N; the KQ2 test/prediction ratios (≈ 1.07 characteristic ratio for dowels; ultimate ≈ 3.3 × EYM plastic load for nails) are the better guide to mean failure loads.
- The 1.05 (modes c–e) and 1.15 (mode f) multipliers in eq. (8.6) are calibration to the partial-factor system, not mechanics; Blass & Colling explicitly drop them when comparing with tests (KQ2). An engine comparing to *mean* test loads should use the bare Johansen expressions plus a mean-strength M_y.

### Gaps
- No open-access source was found that prints a complete timber-to-timber nailed or bolted EC5 example with every mode numbered; Porteous & Kermani and the STEP lectures are paywalled/print-only, and the Blass & Sandhaas PDF was not opened. The derived case above fills the slot but is session arithmetic.
- The Swedish Wood PDF's equation images did not extract, so its nail example gives only the governing value (1.607 kN); a session recomputation with EN 1995-1-1 eq. (8.10) gives ≈ 1.48 kN (f_h = 21.1 MPa, M_y = 6 617 N·mm, t1 = 35 mm), i.e. the book may use a manufacturer M_y or a different embedment form. Treat 1.607 kN as "published", not reproduced.
- EN 338 mean density and mean f_c,90 for C24 were not verified from a primary page this session.

---

## KQ2. Measured capacities of nailed, screwed and dowelled/bolted timber joints versus Johansen/EC5 predictions

### Takeaway
The largest open evaluation (Blass & Colling, 1 588 dowelled-joint tests) finds the EC5 characteristic prediction is slightly conservative: the EN 14358 characteristic ratio test/F_v,Rk is 1.074 for timber-to-timber and 1.073 for steel-to-timber, with the under-prediction growing with dowel diameter; for nails, ultimate loads run roughly 3.3 × the yield-model plastic load because of rope effect and hardening, while the yield load itself is predicted within 6–28 %.

### Cited Findings
- Blass, H.J. & Colling, F., "Load-carrying capacity of dowelled connections", INTER 48-07-03 (2015): "Altogether 1588 tests were evaluated, 1045 timber-to-timber and 543 steel-to-timber connections"; after excluding non-code geometries "561 test results with timber-to-timber and 325 with steel-to-timber connections remain" — [KIT open repository](https://publikationen.bibliothek.kit.edu/1000049999/3752771)
- "The characteristic ratio calculated according to EN 14358 is 1.074 for timber-to-timber and 1.073 for steel-to-timber connections … Ideally, the characteristic ratio would be 1.0 for both cases. The calculation model according to Eurocode 5 is hence slightly conservative." The 1.05/1.15 factors of §8.2.2/8.2.3 "are disregarded" in that comparison — [same](https://publikationen.bibliothek.kit.edu/1000049999/3752771)
- "the difference between the ultimate test load and the calculated characteristic load-carrying-capacities increases with increasing dowel diameter." — [same](https://publikationen.bibliothek.kit.edu/1000049999/3752771)
- Dowel yield moments measured (EN 409) vs EC5 eq. (8.30) with nominal fu — Table 1 rows (diameter/length, Rm,mean, My,EN409,mean, My,EC5,Rm, My,EC5,fuk, all N·m): "GH 8/200 593 52 40 24", "GH 12/200 604 174 116 69", "RB 16/240 742 494 301 146", "GH 20/300 572 759 414 261" — [same](https://publikationen.bibliothek.kit.edu/1000049999/3752771). The full-plastic formula M_y = f_y,ef·d³/6 fits the bending tests with "average ratio … 1.09, the characteristic ratio is 1.00."
- Two-hinge failure modes show "an additional slenderness effect … increasing the load-carrying capacity of these connections in the order of 25 % compared to calculated values based on the Johansen model." — [same](https://publikationen.bibliothek.kit.edu/1000049999/3752771)
- Nailed plywood-to-lumber joints (ASCE J. Mater. Civ. Eng. 5(2):226, 1993): the European Yield Model plastic load was "within 6–28%" of the 5 % offset load, "while the ultimate load was on average approximately 3.3 times the EYM plastic load" — [ASCE abstract via search](https://ascelibrary.com/doi/10.1061/%28ASCE%290899-1561%281993%295%3A2%28226%29) (page itself returned 403; wording from the search-engine abstract, treat as second-hand until the abstract is opened)
- Driven-nail embedment: the EC5 embedding strength for driven nails "over-predicted the empirical model by 15% at 350 kg/m³ density, improving … to 8% at 600 kg/m³" — reported in a search summary attributed to a KIT publication; the underlying page was not opened, so this is **unverified second-hand**.

### Inferences
- For a *mean*-value engine: take the EC5 characteristic F_v,Rk with the 1.05/1.15 factors removed as the yield (plastic-hinge) load, and expect the mean ultimate to be ≈ 1.1–1.3 × that for dowels/bolts (1.074 characteristic ratio, 25 % slenderness surplus) and considerably more for slender nails with rope effect (up to ≈ 3 × for plywood-to-lumber nails). A benchmark should therefore test the *onset of plastic slip* (yield), not the peak, unless the engine models rope effect.
- Using measured steel strength matters: EC5's M_y from nominal fu under-predicts measured dowel yield moments by a factor of 1.5–3 at 12–20 mm (Table 1 above). If an engine stores a fastener yield moment, store the full-plastic value f_y·d³/6 with real steel strength, not the EC5 0.3·fu·d^2.6 with nominal fu.

### Gaps
- No open dataset with per-specimen nailed timber-to-timber capacities (N) alongside EC5 predictions was retrievable (the Sri Lankan IESL paper on EC5 nail-equation effectiveness returned 503 repeatedly). The plywood figure is second-hand.
- Johansen's 1949 original test data were not accessed.

---

## KQ3. Bearing (compression perpendicular to grain) of a timber plate: EN 1995-1-1 §6.1.5, f_c,90,k for C24 and k_c,90, in Pa and N

### Takeaway
For C24 f_c,90,k = 2.5 MPa (2.5e6 Pa); the check is σ_c,90,d = F/A_ef ≤ k_c,90·f_c,90,d with the contact length extended 30 mm at each free end and k_c,90 = 1.5 (solid timber, discrete support ≥ 2h from the next load), 1.75 (glulam), else 1.0. A 45 × 66 mm joist end bearing therefore has a characteristic resistance of ≈ 16 kN (with the 30 mm extension) and the engine's "crush" threshold for a 102.5 × 102.5 mm brick on a C24 plate is ≈ 26–40 kN characteristic depending on k_c,90.

### Cited Findings
- "The characteristic value of the compressive strength is fc,90,k = 2,5 MPa" (C24 joist); "b = 45 mm", "lb = 66 mm"; design value with "kmod = 0,8 and γM = 1,3" — [Swedish Wood Vol. 3, Example 3.2, p. 7](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
- Check formula: "σc,90,d = Fc,90,d / Aef" and "σc,90,d ≤ kc,90 fc,90"; "The length of the contact area is increased by 30mm in accordance with 6.1.5(1) on both ends of the support, providing that the distance from another joint or member end is longer than this value." — [FIN EC help, Compression perpendicular to the grain in supports](https://finesoftware.eu/help/truss4/en/compression-perpendicular-to-the-grain-in-supports-01/)
- k_c,90: "1.5 for solid timber when distance from nearest joint exceeds 2h (h = supported member height)"; "1.75 for glued laminated timber when distance exceeds 2h and support length < 400mm"; "1.0 for all other cases" — [same](https://finesoftware.eu/help/truss4/en/compression-perpendicular-to-the-grain-in-supports-01/). (The 1.25 value for continuous support of solid timber is in the 2004 text but was not shown on this page.)
- Background: "since the test method for compression strength perpendicular to the grain in EN 1193 was changed from a partial area to a full area loading test, the strength values for softwood in EN 338 were significantly reduced" and k_c,90 compensates for partial loading — [KIT, Blass & Görlacher](https://publikationen.bibliothek.kit.edu/1000002867/685544)

### Inferences
- **[derived] Example 3.2 completed.** Joist load 3.3 kN/m² × 0.6 m = 1.98 kN/m over 4.5 m → end reaction F = 4.46 kN. Nominal contact 45 × 66 = 2 970 mm² → σ = 1.50 MPa. With the inward 30 mm extension A_ef = 45 × 96 = 4 320 mm² → σ = 1.03 MPa. f_c,90,d = 0.8 × 2.5/1.3 = 1.54 MPa; k_c,90 = 1.5 → limit 2.31 MPa; utilisation 0.45–0.65, "sufficient", agreeing with the book's conclusion.
  - Characteristic bearing resistance of that joist end: F_R,k = k_c,90·f_c,90,k·A_ef = 1.5 × 2.5e6 Pa × 4.32e-3 m² = **16.2 kN** (nominal area: 11.1 kN).
- **[derived] Brick on a C24 wall plate** (the project's bearing case): a 102.5 × 102.5 mm bearing patch, A = 1.05e-2 m². With k_c,90 = 1.0: F_k = 2.5e6 × 1.05e-2 = 26.3 kN; with k_c,90 = 1.5 and the 30 mm extensions each side along the grain (162.5 × 102.5 = 1.67e-2 m²): F_k = 1.5 × 2.5e6 × 1.67e-2 = 62 kN. The brick's own weight (≈ 2.3 kg × 9.81 = 23 N) is ~0.1 % of this, i.e. plates never crush under a few courses; a benchmark for bearing failure needs a tall stack or a point load.
- Mean f_c,90: EN 338 publishes only the characteristic 2.5 MPa; test means for spruce under full-area loading are typically ~1.3–1.6 × the characteristic. Not sourced here — flag.

### Gaps
- No primary page quoting EN 1995-1-1 §6.1.5 verbatim with all four k_c,90 values (1.0/1.25/1.5/1.75) was reached; the FIN EC page is a vendor paraphrase. Mean f_c,90 for C24 is unsourced.

---

## KQ4. Simple statics with exact joint forces (lintel on two bearings, overhanging beam tipping, three-hinged portal, propped cantilever)

### Takeaway
These are closed-form and need no literature; the exact answers are stated here so the report writer can quote them. The only one that is *not* rigid-body-determinate is the propped cantilever (it needs joint compliance), which matters for an engine that solves statics by rigid blocks.

### Cited Findings
- Lintel load geometry rule used as input (NCMA TEK 17-1D as reproduced by FORSE): "draw lines extending upward at 45 degrees from each lintel end … see if there is at least 8" between the apex of the triangle that is formed and the location of the loads on the wall. If yes, then the lintel only needs to be designed for the dead load of the wall material within the triangle"; figure labels "8 in. (203 mm) minimum", "h = effective span/2", "End bearing 4 in. (102 mm) minimum (typ)" — [FORSE Consulting, New Openings in Existing Masonry Walls (2018), citing NCMA TEK 17-1D (2011)](https://masonryadvisorycouncil.org/wp-content/uploads/2024/01/SE-Insight_New-Opening-in-Existing-Masonry.pdf)
- Beam example for self-weight: C24 45 × 220 mm joist, span 4.5 m, 0.6 m spacing, 3.3 kN/m² — [Swedish Wood Vol. 3, Example 3.1](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)

### Inferences (all [derived], exact statics)
- **Simply supported lintel, UDL.** Span between bearing centres L, load w per metre: R_A = R_B = wL/2; midspan M = wL²/8; the two bearing joints carry pure compression wL/2 each and zero shear if the bearings are frictionless. Worked: 45° triangle of 102.5 mm clay brickwork over effective span ℓ = 1.35 m (1.2 m clear + ½ × 150 mm bearing each end), unit weight γ = 1 900 kg/m³ × 9.81 = 18 640 N/m³ (assumed density): W = ℓ²·t·γ/4 = 1.8225 × 0.1025 × 18 640/4 = **871 N**, so each bearing takes **435 N**. Full rectangle to a floor 1.0 m up instead: 1.35 × 1.0 × 0.1025 × 18 640 = 2 579 N (1 290 N per bearing). The triangle-vs-rectangle ratio (≈ 1 : 3 here) is exactly what the arching test in KQ5 should reproduce.
- **Overhanging beam tipping.** Supports at x = 0 (A) and x = a (B), total length L, self weight w per metre, no hold-down at A. Tipping about B when the overhang moment exceeds the back-span moment: w(L−a)²/2 > wa²/2 ⇔ **overhang > back span** (a < L/2 with A at the end). With a tip point load P: P_tip = w·[a² − (L−a)²]/(2(L−a)). Example: 45 × 220 mm C24 at 420 kg/m³ → w = 0.045 × 0.22 × 420 × 9.81 = 40.8 N/m; L = 3 m, a = 2 m: P_tip = 40.8 × (4 − 1)/2 = **61 N**; at that instant R_A = 0 and R_B = total weight 122 N + 61 N = 183 N. The reaction at A going to zero is the mechanism to assert on (contact force, not displacement).
- **Three-hinged portal, hinges at both bases and crown.** Span L, column height h (crown at height h, flat roof beam). Vertical P at the crown: V_A = V_B = P/2; moment about the crown of one half: V_A·L/2 − H·h = 0 → **H = P·L/(4h)** (equal and opposite thrust at both bases). Example L = 4 m, h = 3 m, P = 10 kN: V = 5 kN, H = 3.33 kN each. Horizontal F applied at the left eave: V_B = F·h/L (up), V_A = −F·h/L; right-half crown moment → H_B = V_B·L/(2h) = F/2 and H_A = F/2 (both resisting). Example F = 10 kN: V = ±7.5 kN, H_A = H_B = 5 kN. Each base joint in the engine must carry exactly these (compression + shear); crown hinge carries H and V/2 but zero moment.
- **Propped cantilever (fixed at A, roller at B, UDL w, length L).** R_B = 3wL/8, R_A = 5wL/8, M_A = wL²/8 (statically indeterminate; needs the beam's EI). A rigid-block engine cannot get this without a compliant fixed joint; the determinate alternative is the two-span-equivalent or a cantilever with a cable tie. Note it as a negative benchmark (expect divergence).
- **Simply supported beam point load at midspan**: R = P/2 each, M = PL/4.

### Gaps
- None for the mechanics; the masonry unit weight (1 900 kg/m³) is an assumption, not a sourced value for a specific brick type.

---

## KQ5. Masonry arching over openings: BS 5977-1 / NCMA triangle rules and tests of walls over openings

### Takeaway
The code rule is the 45° isosceles triangle on the effective span with masonry above the apex (NCMA: ≥ 203 mm) carried by arching to the jambs; BS 5977-1:1981 formalises this as a "load triangle" plus an "interaction zone" for spans ≤ 4.5 m (single storey) / 3.6 m (2–3 storey) with no openings inside the triangle. Clean, open-access *measured* data for a brick wall over an opening exist for the composite-lintel tests of Vermeltfoort (18 walls, 2.8 m span, mean failure shear 31 kN four-point / 24.4 kN one-point) and for calcium-silicate walls with openings on a deflecting beam (Piekarczyk: lintel-end cracking at < 3 mm beam deflection; failure 354–620 kN/m²); Hendry's and Dawe & Seah's arching data were only reachable second-hand.

### Cited Findings
- BS 5977-1:1981 scope: "a method for assessing the load carried by lintels in masonry where lintels span up to 4.5 m in single storey buildings and up to 3.6 m in two or three storey buildings in normal domestic use, and where no openings are wholly or partly within the load triangle" — [BSI product page](https://knowledge.bsigroup.com/products/lintels-method-for-assessment-of-load); contents list "load triangle" and "interaction zone" — [SAI Global listing](https://shop.standards.ie/en-ie/standards/bs-5977-1-1981-259607_saig_bsi_bsi_601391)
- Manufacturer summary of the UK practice: "the masonry load is taken as the weight of masonry in a triangle shape above the lintel (where the lintel is the base of the triangle)"; load share between leaves of a cavity wall "Masonry only 1:1; Timber floor loads 3:1 (inner to outer leaf); Concrete floor loads 5:1"; deflection limit "0.003 x the clear span" (BS EN 845-2) — [Catnic, How do you calculate lintel load](https://catnic.com/blog/how-do-you-calculate-lintel-load)
- NCMA/TMS rule and numbers (45°, 203 mm above apex, h = span/2, 102 mm bearing) — [FORSE / NCMA TEK 17-1D](https://masonryadvisorycouncil.org/wp-content/uploads/2024/01/SE-Insight_New-Opening-in-Existing-Masonry.pdf)
- Historical origin of the composite (wall-beam / arching) model: "Wood described the wall/beam composite behaviour as a tied arch … The arch becomes evident when the height of the wall is at least 60% of the beam span, while Stafford Smith and Riddington found that the minimum height/span ratio of the arch is 0.70." — [City University thesis (Raja 1991) via search](https://openaccess.city.ac.uk/id/eprint/28545/1/Raja%20thesis%201991%20PDF-A.pdf)

**Measured assembly case A — composite lintels (Vermeltfoort 2009):**
- "Eighteen identical walls were loaded in plane to rupture. Nine layers of stretcher bond masonry, 562.5 mm in height, were built on prefab concrete lintels (60×100 mm²) with a span of 2800 mm." — [Vermeltfoort, 11th Canadian Masonry Symposium 2009, B8-2](https://www.canadamasonrydesigncentre.com/wp-content/uploads/11th_symposium/B8-2.pdf)
- "The mean failure shear load for the four point loading condition was Vfail = 31 kN. For the one point condition it was Vfail = 24.4 kN. On average, the ultimate load (Fult) was 15% higher than the failure load (Ffail)." Supports (rollers vs felt on brick) "had a negligible effect on the load bearing capacity." — [same](https://www.canadamasonrydesigncentre.com/wp-content/uploads/11th_symposium/B8-2.pdf)
- Materials: "Soft mud bricks, brand Rijswaard (mean compressive strength ±20 N/mm2)"; factory mortar "f'mor = 9.5 N/mm2" nominal (per-wall prism means 4.46–12.5 MPa in Table 1); "masonry with a compressive strength of f'rep = 7.1 N/mm2"; lintel concrete 55 N/mm², prestressed with two 4 mm FeP 1770 bars — [same](https://www.canadamasonrydesigncentre.com/wp-content/uploads/11th_symposium/B8-2.pdf)
- Joint shear law used by the author for the arch foot: "τ = 0.35 + 0.65σ (1)" (MPa), "assume that the contact area is 250 mm in length at the foot of the arch … a reaction force for the tested walls of 15 kN and a tensile force in the lintel of 19 kN is estimated" — [same](https://www.canadamasonrydesigncentre.com/wp-content/uploads/11th_symposium/B8-2.pdf)
- Table 1 per-wall cracking loads F_fail (kN): series 1 (rollers, 4-point) 61, 69, 65, 57, 58, 54; series 2 (felt, 4-point) 62, 60, 64, 38, 60, 61; series 3 (felt, 1-point) 42, 36, 30, 28, 28, 31; stiffness k ≈ 60 kN/mm; "V1 through V9 were precracked" (vertical joints cut to force a free arch) — [same](https://www.canadamasonrydesigncentre.com/wp-content/uploads/11th_symposium/B8-2.pdf)

**Measured assembly case B — walls with openings on a deflecting support (Piekarczyk 2020):**
- Walls "4.55 m long and 2.45 m high … group 1 calcium silicate masonry units" 250 × 180 × 220 mm, "fB = 17.7 N/mm2 … fb = 21.8 N/mm2", mortar "fm = 12.5 N/mm2", thin bed joints, unfilled head joints; RC lintels 180 × 220 mm with ≥ 125 mm bearing — [Piekarczyk, Budownictwo i Architektura 19(3) 2020, DOI 10.35784/bud-arch.2142](https://ph.pollub.pl/index.php/bia/article/view/2142)
- Cracking: "Walls with one door opening also cracked at an early stage … cracking at the ends of the lintel occurred because of the rotation of the pillars"; two-door walls "first cracks occurred at the lintel ends at a deflection not exceeding 3 mm and a load not exceeding 90 kN/m2" — [same](https://ph.pollub.pl/index.php/bia/article/view/2142)
- Failure loads p_u (kN/m² of wall section) and beam midspan deflections: A-1 286 / 6.05 mm; A-2 609 / 25.6 mm; B-1 (one door) 391 / 11.2 mm; B-2 354 / 6.59 mm; C-1 (two doors) 620 / 23.4 mm; C-2 538 / 22.0 mm; D-1 (door + window) 458 / 15.0 mm; D-2 519 / 20.0 mm; failure modes include "diagonal cracking of the narrow pillar", "crushing of the masonry under the lintel", "diagonal 'stepped' cracking" under the window — [same](https://ph.pollub.pl/index.php/bia/article/view/2142)

**Arching in confined panels (for the thrust-capacity rule):**
- Dawe & Seah citation: "Out-of-plane resistance of concrete masonry infilled panels." Canadian J. Civil Eng., 16, 854–864 (1989) — [Sepasdar & Liu 2017 reference list](https://canadamasonrydesigncentre.com/wp-content/uploads/13th_symposium/035-Sepasdar-Paper-1.pdf)
- MSJC/TMS arching equation (built on Dawe & Seah, Angel, Klingner): "q = 4.1 f'm^0.75 t² (α/l^2.5 + β/h^2.5)" with "α = (1/h)(Ec Ic h²)^0.25 < 50", "β = (1/l)(Eb Ib l²)^0.25 < 50", f'm in kPa, t, h, l in mm — [Sepasdar & Liu, 13th CMS 2017](https://canadamasonrydesigncentre.com/wp-content/uploads/13th_symposium/035-Sepasdar-Paper-1.pdf)
- Measured: control infill (half-scale 200 mm CMU, prism f'm = 9.4 MPa, aspect ratio 1.4) "Pcr (kPa) 26.8 … Pult (kPa) 66.3"; with a 17 % window opening Pult = 43.7 kPa (34 % reduction); MSJC prediction "48.3 kPa, resulting in a design-to-test ratio of 48.3/66.3=0.73" — [same](https://canadamasonrydesigncentre.com/wp-content/uploads/13th_symposium/035-Sepasdar-Paper-1.pdf)

### Inferences
- A rigid-block engine with cohesive bed joints should reproduce the Vermeltfoort configuration qualitatively and semi-quantitatively: 9 courses × 62.5 mm on a 2.8 m span; with τ = 0.35 + 0.65σ at the lintel–masonry interface the arch-foot shear check gives a reaction of order 15 kN, i.e. V ≈ 15–30 kN, bracketing the measured 24–31 kN. The dominant mechanism is horizontal-joint shear near the supports ("fifteen other walls failed in the maximum shear load area near the supports"), so the benchmark should assert the first slip at the lintel–brick bed joint, not midspan tension.
- The 45° triangle is a *design envelope*, not the true mechanics; the test for an engine is: remove the lintel (or make it very soft) under a wall whose height above the opening exceeds the span/2 + 203 mm rule, and check that only the triangle (± its boundary courses) falls while the rest arches — the exact mass that falls is the geometric triangle on the effective span (KQ4 worked number 871 N for a 1.35 m span in 102.5 mm brickwork), provided the jambs can take the thrust (Vermeltfoort's τ = 0.35 + 0.65σ at the arch foot is a usable thrust-capacity rule).
- Piekarczyk's walls show the *first* joint to open when a support deflects is at the lintel end (pillar rotation), at a few millimetres — an engine with stiff-ish cohesive joints should crack there first too; the magnitudes (90 kN/m² ≈ 16 kN/m run on a 180 mm wall) are usable only with the measured material set (fb = 21.8 MPa, thin-bed mortar fm = 12.5 MPa, unfilled perpends → f_vk uses eq. 3.6 with 0.5·f_vko).

### Gaps
- The actual geometry of BS 5977-1's interaction zone (60° line, 1.1 × span, 300 mm rules) could not be quoted from a legitimate source — only the scope sentence. Report as "BS 5977-1 defines a load triangle and an interaction zone; see the standard" rather than numbers.
- Hendry's own arching-over-opening test data (Structural Masonry) and Dawe & Seah's per-panel results were not reachable open-access; only their formula's descendants and citations.
- No open test was found of a brick wall whose *lintel was removed* with the collapsed mass measured — the closest are Vermeltfoort's pre-cracked (V-series) composite lintels.

---

## KQ6. Masonry flexural (bond) and shear-bond strengths as joint strengths: EN 1996-1-1 Tables 3.2/3.4, f_vk0, friction coefficient; measured means (Gooch, Masia, Stewart, Lam)

### Takeaway
Code characteristic values for clay units with general-purpose mortar are f_xk1 = 0.10 MPa (bed-joint tension in bending), f_xk2 = 0.20–0.40 MPa (perpendicular to bed joints), f_vk0 = 0.30 MPa (M10–M20) with f_vk = f_vk0 + 0.4σ_d ≤ 0.065 f_b; measured *means* are far higher: flexural bond ≈ 2.06 × direct tension (direct tension means 0.15–0.25 MPa → flexural ≈ 0.3–0.5 MPa), shear bond ≈ 1.34 × flexural (couplet cohesion 0.58–1.04 MPa), and the bed-joint friction coefficient is ≈ 1.0–1.1 at 0.2 MPa normal stress falling to 0.6–0.9 at 1.0 MPa (COV 0.13–0.14), versus the code's 0.4.

### Cited Findings

**EN 1996-1-1 characteristic values (Ukrainian adoption DSTU-N B EN 1996-1-1:2010, English text columns):**
- Table 3.4 f_vk0 (N/mm²): "Clay 0.30 (general purpose M10–M20), 0.30 (thin layer), 0.15 (lightweight); Calcium silicate 0.20 / 0.40 / 0.15; Aggregate concrete 0.20 / 0.30 / 0.15; AAC 0.15; Manufactured stone 0.20 / 0.40; Dimensioned natural stone 0.20 / 0.40 / 0.15" — [dnaop.com EN 1996-1-1 §3.6](https://dnaop.com/html/60058_6.html)
- Table 3.2 f_xk1 (N/mm²): "Clay: 0.10 (general purpose both fm ranges), 0.10 (thin layer), 0.15 (lightweight)"; "Calcium silicate: 0.05 (fm<5), 0.10 (fm≥5), 0.20 (thin layer)"; "Aggregate concrete: 0.05, 0.10, 0.20"; "AAC: 0.05, 0.10, 0.15, 0.10" — [same](https://dnaop.com/html/60058_6.html)
- Table 3.2 f_xk2 (N/mm²): "Clay: 0.20 (both general purpose), 0.40 (thin layer), 0.15 (lightweight)"; "Calcium silicate: 0.20, 0.20, 0.40, 0.30"; "Aggregate concrete: 0.20, 0.20, 0.40, 0.30" — [same](https://dnaop.com/html/60058_6.html). **Conflict:** a search summary of the Cyprus NA table gives clay f_xk2 = 0.20 for fm < 5 N/mm² and 0.40 for fm ≥ 5 N/mm² — [Cyprus NA to CYS EN 1996-1-1 (PDF, 403 on direct fetch)](https://moi.gov.cy/MOI/moiup/moi.nsf/All/05FE06BB537676ACC22585A100430DF8/$file/Cyprus_National_Annex_To_CYS_EN_1996-1-1_2005A1_2012_.pdf). Use 0.20 (weak mortar) / 0.40 (M5+) and flag.
- Unit-based cap: "fxk1 = 0,035 fb with filled and unfilled perpend joints"; "fxk2 = 0,035 fb, with filled perpend joints or 0,025 fb, with unfilled perpend joints" (for thin-layer/AAC cases) — [same](https://dnaop.com/html/60058_6.html)
- Shear strength clause 3.6.2(3): "f vk = f vko + 0,4 σd (3.5) but not greater than 0,065 fb, or fvlt"; unfilled perpends (3.6.2(4)): "f vk = 0,5 f vko + 0,4 σd, (3.6) but not greater than 0,045 fb, or fvlt"; shell bedding (3.7): f_vk = (g/t)·f_vko + 0.4σ_d; f_vko "should be determined from tests in accordance with EN 1052-3 or EN 1052-4" or Table 3.4 — [dnaop.com EN 1996-1-1 §3.6.2](https://dnaop.com/html/60058_5.html)
- Partial factors (UK NA vs German NA): "Shear strength 2.0 / 1.5", "Bond strength 1.5 / 1.5", "Compression strength Cat I 2.0 / 1.76" — [Watt, JRC Eurocode 6 dissemination 2009](https://eurocodes.jrc.ec.europa.eu/sites/default/files/2022-06/3_pm_Watt.pdf)

**Measured means — Gooch, Masia, Stewart & Lam 2023 (clay brick, Australian 1:1:6 mortar):**
- DOI "10.1016/j.conbuildmat.2023.131578"; "A mean ratio of flexural tensile bond strength to direct tensile bond strength of 2.06 and a COV of 31.5%"; brick unit flexural/direct "mean ratio of 1.29 with a COV of 14.7%"; "ratio of the shear bond to flexural tensile bond strengths … mean ratio of 1.34 with a COV of 28.4%" — [JCU record / abstract](https://researchonline.jcu.edu.au/87642/)

**Measured means — Gooch, Masia, Stewart & Spadari 2025 (friction and cohesion; open access CC-BY):**
- Citation: Construction and Building Materials 489 (2025) 142348, DOI 10.1016/j.conbuildmat.2025.142348; bricks "230 mm × 110 mm × 76 mm", "1:1:6 (cement: lime: sand, by volume) mortar mix (approximately 1:0.5:7.6 by mass), as per AS 3700"; shear-box couplet tests, 10 specimens per cell — [open-access PDF](https://researchonline.jcu.edu.au/87638/1/Experimental%20characterisation%20of%20the%20friction%20coefficient%20of%20mortar%20bed%20joints%20in%20clay-brick%20masonry.pdf)
- Table 1, mean residual friction coefficient μf [COV] at mean normal stress 0.22 / 0.60 / 1.00 MPa: E1 "1.10 [0.18] / 0.82 [0.07] / 0.60 [0.10]"; E2 "1.11 [0.09] / 0.96 [0.07] / 0.80 [0.06]"; E3 "1.11 / 0.95 / 0.81"; P1 "1.04 / 0.94 / 0.88"; P2 "1.16 / 0.99 / 0.91"; S1 (solid) "1.06 [0.14] / 0.91 [0.12] / 0.82 [0.10]" — [same](https://researchonline.jcu.edu.au/87638/1/Experimental%20characterisation%20of%20the%20friction%20coefficient%20of%20mortar%20bed%20joints%20in%20clay-brick%20masonry.pdf)
- Table 2 probabilistic model: "Mean 1.12·e^(−0.35·σn)" (extruded), "1.04·e^(−0.13·σn)" (pressed/solid), σn in MPa, "COV 0.13", "Distribution Normal" — [same](https://researchonline.jcu.edu.au/87638/1/Experimental%20characterisation%20of%20the%20friction%20coefficient%20of%20mortar%20bed%20joints%20in%20clay-brick%20masonry.pdf)
- EN 1052-3-style regression on the same couplets: "shear bond strengths, ranging from 0.58 MPa to 1.04 MPa … initial friction coefficients determined via linear regression analysis between 0.64 and 1.00"; and "overall mean value of µf,0 / µf = 0.91" (regression under-reads friction at low σn) — [same](https://researchonline.jcu.edu.au/87638/1/Experimental%20characterisation%20of%20the%20friction%20coefficient%20of%20mortar%20bed%20joints%20in%20clay-brick%20masonry.pdf)
- Table 3 direct tensile bond strength ft (MPa) [COV]: E1 "0.17 [0.42]", E2 "0.21 [0.22]", E3 "0.25 [0.18]", P1 "0.15 [0.59]", P2 "0.25 [0.18]", S1 "0.22 [0.20]"; tension-pre-cracked joints keep the same friction ("overall mean value of µf,cr / µf = 1.01") — [same](https://researchonline.jcu.edu.au/87638/1/Experimental%20characterisation%20of%20the%20friction%20coefficient%20of%20mortar%20bed%20joints%20in%20clay-brick%20masonry.pdf)
- Literature range quoted there: "a deterministic value of the friction coefficient … ranging in magnitude from 0.25 to 1.20"; Gonen et al. adopt "ϕ′ = 35° (µf = 0.70)" — [same](https://researchonline.jcu.edu.au/87638/1/Experimental%20characterisation%20of%20the%20friction%20coefficient%20of%20mortar%20bed%20joints%20in%20clay-brick%20masonry.pdf)

**Measured means — bond wrench on full walls (Gooch, Lam, Terry, Masia, Stewart, 15th CMS 2025):**
- Concrete brick wall (230 × 110 × 76 mm, 1:1:6 mortar, 20 courses, fully bedded): "mean flexural tensile bond strength of 0.52 and COV of 0.43" over "93" joints; face-shell-bedded 190 mm and 140 mm block walls only "0.05 MPa and 0.07 MPa, with high coefficients of variation (COVs) of 0.52 and 0.56" — [Gooch et al., 15th Canadian Masonry Symposium 2025, paper 019](https://www.canadamasonrydesigncentre.com/wp-content/uploads/15th_symposium/019-Gooch.pdf)
- AS 3700 bond-wrench reduction used: "fsp = (Msp/Zd) − (Fsp/Ad)", "Msp = 9.81 m2 (d2 − tu/2) + 9.81 m1 (d1 − tu/2)", "Fsp = 9.81 (m1 + m2 + m3)"; brick-wrench constants "tu 110 mm, d1 389 mm, d2 1300 mm, m3 3.0 kg", "m1 8.2 kg" — [same](https://www.canadamasonrydesigncentre.com/wp-content/uploads/15th_symposium/019-Gooch.pdf)
- Mortar joint thickness in those walls: brick wall mean "10.9 mm", COV 0.11 — [same](https://www.canadamasonrydesigncentre.com/wp-content/uploads/15th_symposium/019-Gooch.pdf)

**Triplet (EN 1052-3) cohesion/friction, Spanish handmade brick with lime mortar (Segura, Pelà, Saloustros, Roca):**
- Masonry "c [MPa] 0.16", "Φ [º] 33.71" (3 × 3 triplets), "fc [MPa] 6.51"; brick fb,c 17.99 MPa, fb,f 2.44 MPa; mortar fm,c 2.19 MPa, fm,f 0.66 MPa — [UPC open repository](https://upcommons.upc.edu/server/api/core/bitstreams/77631472-a1eb-4bc2-b2ad-5bfcb4bd5f02/content)

### Inferences
- Code-to-mean mapping for the engine (clay brick, GP mortar ~M4–M6): bed-joint tensile (flexural) strength mean ≈ 0.3–0.5 MPa (Gooch: 2.06 × direct tension 0.15–0.25 MPa; concrete-brick wall 0.52 MPa) against f_xk1 = 0.10 MPa characteristic — a factor of 3–5. Cohesion mean ≈ 0.4–0.7 MPa (1.34 × flexural) and up to 0.58–1.04 MPa from couplet regression, against f_vk0 = 0.30 MPa (M10–M20) or 0.20 (M2.5–M9). Friction mean ≈ 1.0 (low σn) → 0.8 (1 MPa) against the code's 0.4. Lime-mortar handmade brick (UPC) is the weak end: c = 0.16 MPa, μ = tan 33.7° = 0.67.
- Because friction drops with normal stress (dilatancy), a single Coulomb μ in the engine is an approximation; a benchmark that pins μ should state the normal stress (0.2 MPa ≈ 10 courses of 110 mm brick; use μ ≈ 1.05 there for extruded clay, ≈ 0.6–0.8 at 1 MPa).
- The EN 1052-3 regression slope (0.64–1.00) is what a code-calibrated model would use for "initial friction"; the residual sliding friction is *not* lower than that, so an engine that uses one μ for both cracking and sliding is not far wrong.
- For a 230 × 110 mm Australian brick bed (fully bedded, gross section): A = 2.53e-2 m², Z = b·d²/6 = 0.230 × 0.110²/6 = 4.64e-4 m³. **[derived]** Pure-bending capacity of one bed joint (no axial load): M = f_x·Z = 0.10e6 × 4.64e-4 = **46 N·m** (EC6 characteristic) or 0.52e6 × 4.64e-4 = **241 N·m** (Gooch concrete-brick mean) or ≈ 0.40e6 × 4.64e-4 = 186 N·m (clay mean ≈ 2.06 × 0.2 MPa). For a UK 215 × 102.5 mm brick: Z = 3.76e-4 m³ → 38 N·m at 0.10 MPa, 150 N·m at 0.40 MPa. These are the exact single-joint answers a bond-wrench benchmark pins; axial compression N adds N·t/6 to the moment capacity (the "− Fsp/Ad" term in the AS 3700 formula is exactly this correction).

### Gaps
- EN 1996-1-1 Table 3.4 columns for weaker GP mortars (M2.5–M9, M1–M2) were not displayed by the retrieved page; only the M10–M20, thin-layer and lightweight columns are quoted. (Recalled but unverified: clay 0.20 and 0.10 N/mm² respectively.)
- The UK National Annex flexural table (clay by water absorption and mortar class, values of order 0.3–0.7 / 0.9–2.0 N/mm²) could not be retrieved; it differs greatly from the CEN recommended 0.10/0.20–0.40 and should be sourced before use.
- The 2023 paper's absolute per-type means (bond wrench MPa, triplet MPa) are behind the paywall; only the ratios and the 2025 open-access tables were obtained.

---

## KQ7. Bonded-wall tests a rigid-block + cohesive-joint model can reproduce: cantilever overturning, bond-wrench stacks, diagonal compression (stepped crack)

### Takeaway
Single-joint bending (bond wrench) and cantilever-wall cracking/overturning are exact and small; the one-way URM wall tests of Derakhshan et al. give measured versus rigid-body predictions (uncracked strength predicted within 24 %; post-cracking rocking resistance 83 % of the rigid-body value); diagonal compression gives a measured panel load (≈ 115–179 kN for 1.27 m Flemish-bond walls) but its crack path only follows the joints when the bond pattern allows a stair-step — in Flemish bond the bricks broke, so it is a *negative* benchmark for a rigid-brick engine.

### Cited Findings
- Derakhshan, Griffith & Ingham (one-way spanning URM walls, airbag tests, Adelaide): "Six full-scale unreinforced masonry (URM) walls were built and tested … two-leaf (230 mm nominal thickness) or three-leaf (350 mm nominal thickness) walls had variable heights from 2000 mm to 4100 mm"; "initial uncracked wall strength predictions were within 24% of the experimental measurements"; "the maximum cracked wall resistance was, on average, 83% of the predicted value based upon the mechanics of rigid body rocking"; "analytical predictions underestimated the wall crack height by 16%" — [University of Adelaide repository item](https://digital.library.adelaide.edu.au/items/e1aeaedc-e903-4459-a590-e6fb05d1949e) (abstract text as returned by search; the companion analytical paper is J. Eng. Mech. 139(4):409–417, DOI 10.1061/(ASCE)EM.1943-7889.0000347)
- Diagonal compression (ASTM E519), five double-leaf Flemish-bond walls "1270 × 1270 × 311 mm³", 15 mm lime-mortar joints: "Pmax [kN] URM_1 178, URM_2 167, URM_3 117, URM_4 179, URM_5 115, Average 151, St. Dev. 32, CV 21.5%"; "ft,ASTM = 0.707 Pmax/A" → average 0.27 MPa (Frocht's approach 0.20 MPa, "36% lower") — [Segura et al., UPC](https://upcommons.upc.edu/server/api/core/bitstreams/77631472-a1eb-4bc2-b2ad-5bfcb4bd5f02/content)
- Crack path: "Contrarily to other bond arrangements such as header bond, where cracks follow the mortar joints and bricks are broken only rarely, the length of the bricks used herein hindered the formation of a stair-stepped crack and necessarily required the failure of bricks in tension." — [same](https://upcommons.upc.edu/server/api/core/bitstreams/77631472-a1eb-4bc2-b2ad-5bfcb4bd5f02/content)
- Joint parameters for that masonry: c = 0.16 MPa, Φ = 33.71°, fc = 6.51 MPa (see KQ6) — [same](https://upcommons.upc.edu/server/api/core/bitstreams/77631472-a1eb-4bc2-b2ad-5bfcb4bd5f02/content)
- Bond-wrench formula with the self-weight/compression correction and the measured 0.52 MPa / COV 0.43 wall mean — [Gooch et al. 2025 CMS](https://www.canadamasonrydesigncentre.com/wp-content/uploads/15th_symposium/019-Gooch.pdf)
- Spatial variability: for clay brick walls adjacent-joint correlation "ρk=1 = 0.4 observed by [1, 3]"; for concrete masonry "statistical independence may most accurately represent" it — [same](https://www.canadamasonrydesigncentre.com/wp-content/uploads/15th_symposium/019-Gooch.pdf)
- Vermeltfoort's 18 composite-lintel walls (KQ5) are also a joint-shear-governed assembly test with per-wall numbers.

### Inferences
- **[derived] Cantilever (free-standing) single-leaf wall, lateral pressure, base bed joint.** Thickness t = 0.1025 m, height H = 1.0 m, unit weight γ = 18 640 N/m³ (1 900 kg/m³ assumed), per metre length. Self-weight stress at the base γH = 18 640 Pa. Cracking when the tension-face stress reaches f_x1: M_cr = (f_x1 + γH)·t²/6. With f_x1 = 0.10 MPa (EC6 char.): M_cr = 118 640 × 0.010506/6 = 207.8 N·m/m → uniform pressure p_cr = 2M/H² = **416 Pa**; with a mean f_x1 = 0.40 MPa: M_cr = 733 N·m/m, p_cr = 1 466 Pa. With *zero* bond (pure rigid-body overturning about the toe): M = W·t/2 = γHt²/2 → p_ov = γt²/H = **196 Pa** (and the resisting moment is independent of bond). So an engine should show: cohesive wall cracks at the base joint at 0.4–1.5 kPa then falls, while an unbonded wall tips at 0.2 kPa. Both numbers scale as t² and 1/H (overturning) or 1/H² (cracking with fixed f_x). Derakhshan's 83 % figure says the real rocking resistance of a cracked wall is ~0.83 × this rigid-body value (finite-thickness hinge, crushing at the pivot).
- **Single-joint bond-wrench benchmark** (exact): clamp the top brick of a two-brick stack and apply a moment; the joint opens at M = f_x·Z − correction for the clamp weight (46–241 N·m for a 230 × 110 bed, see KQ6). Assert the opening moment, not displacement.
- **Diagonal compression:** only use with a bond pattern that permits a stair-step through joints (stretcher/running bond, single leaf). A limit-analysis estimate with Coulomb joints for a running-bond square panel: shear strength along the stepped path ≈ Σ(c + μσ)·A_bed over the bed joints crossed, with head joints contributing only if filled; the UPC Flemish-bond result (0.27 MPa "ft") is a check that a model *without* brick fracture will over-predict such panels.

### Gaps
- No open-access paper giving a cantilever (parapet-type) URM wall's measured cracking load together with its bond-wrench f_x on the same masonry was retrieved; Derakhshan's are one-way spanning (top-supported) walls, and the paper's per-wall table was not accessible (repository PDF host unresolved).
- No diagonal-compression study on running-bond brickwork with both triplet parameters and a reported stepped-crack load was retrieved this session (van der Pluijm 1992/1999, Delft, is the classic source and should be pulled next).

---

## KQ8. Characteristic vs mean: what the engine should store

### Takeaway
Everything in EN 1995/EN 1996 is a 5 % fractile with further partial factors (γ_M 1.3 timber connections; 1.5–2.0 masonry shear/bond, UK) on top; measured means sit 1.1–1.3 × above characteristic for dowelled timber joints, ~3 × for nailed-joint ultimate loads, and 3–5 × for masonry bond. A realism-first engine should store means and treat code values as lower bounds.

### Cited Findings
- Timber connection partial factor and modification used in all examples: "γM = 1,3", "kmod = 0,8" (medium-term, SC1/2) — [Swedish Wood Vol. 3](https://www.svenskttra.se/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol3-2022.pdf)
- Dowel tests: characteristic ratio test/F_v,Rk = 1.074; dowel yield-moment mean ratio 1.09 to the full-plastic formula — [Blass & Colling](https://publikationen.bibliothek.kit.edu/1000049999/3752771)
- Masonry γ_M (UK/DE): shear 2.0/1.5, bond 1.5/1.5 — [Watt, JRC](https://eurocodes.jrc.ec.europa.eu/sites/default/files/2022-06/3_pm_Watt.pdf)
- Masonry friction COV 0.13–0.14 (normal); bond-strength COV 0.2–0.6 — [Gooch et al. 2025](https://researchonline.jcu.edu.au/87638/1/Experimental%20characterisation%20of%20the%20friction%20coefficient%20of%20mortar%20bed%20joints%20in%20clay-brick%20masonry.pdf), [Gooch et al. 2025 CMS](https://www.canadamasonrydesigncentre.com/wp-content/uploads/15th_symposium/019-Gooch.pdf)

### Inferences
- Suggested engine means (clay brick, 1:1:6 GP mortar): bed-joint tension 0.3–0.5 MPa (use 0.4), cohesion 0.5–0.7 MPa (use 0.55), friction 0.9 at ~0.3 MPa normal stress (or μ = 1.12·e^(−0.35σ)), all with COV ≈ 0.3 for bond and 0.14 for friction. For a lime-mortar/handmade-brick profile: tension ≈ 0.1–0.15 MPa, c = 0.16 MPa, μ = 0.67.
- Suggested engine means (C24 softwood, 3.1 mm smooth nail): lateral yield ≈ 0.7 kN per shear plane, lateral ultimate ≈ 1.0–1.3 kN, withdrawal ≈ 0.3–0.4 kN per 40 mm penetration (smooth) and several × that for ring-shank; 8 mm screw withdrawal ≈ 6 kN per 60 mm thread (A1:2008 formula, char.); M16 bolt in 90 mm GL30c double shear ≈ 13 kN char. per bolt (mode k).

### Gaps
- Mean-to-characteristic ratios for C24 density and f_c,90 were not verified from EN 338 this session.

---

## Session tool-availability notes (for the writer)
- PDFs that WebFetch could not read were extracted locally with `pdftotext` (Git for Windows) and `pypdf` under Unreal's bundled Python; quotes above come from those extractions. Full texts are in the session scratchpad (`txt/friction.txt`, `gooch_symp.txt`, `sepasdar.txt`, `kit_nail.txt`, `b8_2.txt`, `diag_upc.txt`, `pollub.txt`, `swedishwood3.txt`).
- Not reachable: Cyprus NA PDF (403), Adelaide PDF hosts (DNS), IESL Sri Lanka paper (503), ASCE abstract pages (403), Porteous & Kermani (print), BS 5977-1 (paywalled standard).
