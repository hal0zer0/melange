# Tungsten Material Properties for Device Modeling

Reference doc for modeling tungsten-based electronic components in melange.
Covers metallic tungsten (point-contact/Schottky diodes) and tungsten compound
semiconductors (WO3, WSe2, WS2). Not a device model — provides the material
constants needed to derive SPICE `.model` parameters.

## Tungsten Metal (W)

Tungsten is a **conductor**, not a semiconductor. Relevant as an electrode material
in point-contact and Schottky barrier diodes.

| Property | Value | Notes |
|----------|-------|-------|
| Atomic number | 74 | |
| Work function (φ) | 4.5–4.6 eV | Polycrystalline; varies by crystal face |
| Work function (100) | 4.63 eV | |
| Work function (110) | 5.25 eV | |
| Work function (111) | 4.47 eV | |
| Resistivity (20°C) | 5.6 µΩ·cm | Very good conductor |
| Melting point | 3422°C | Highest of all metals |
| Thermal conductivity | 173 W/(m·K) | |
| Temp coeff of resistance | 0.0045 /°C | |

### Cat's Whisker Diodes (Historical)

The original point-contact detector: a sharpened tungsten wire pressed against a
semiconductor crystal (usually galena PbS, later silicon or germanium). The
tungsten wire forms the metal side of a metal-semiconductor (Schottky) junction.

**Schottky barrier height** at a tungsten contact:
- W–Si (n-type): φ_B ≈ 0.67 eV
- W–GaAs (n-type): φ_B ≈ 0.80 eV
- W–Ge (n-type): φ_B ≈ 0.48 eV
- W–PbS (galena): φ_B ≈ 0.4–0.5 eV (estimated, poorly characterized)

**Approximate SPICE parameters for a tungsten point-contact diode (W on n-Si):**
```
.model W_POINT_CONTACT D(IS=1e-8 N=1.05 RS=20 CJO=0.5p BV=10 IBV=1e-5)
```
- IS higher than bulk Si diode (surface states, small contact area imperfections)
- N ≈ 1.0–1.1 (Schottky diodes are closer to ideal than PN junctions)
- RS ≈ 10–50 Ω (dominated by spreading resistance of the point contact)
- CJO very small (sub-pF, tiny contact area — why they worked at microwave frequencies)
- BV low (point contacts break down easily)

These are rough estimates. Real cat's whisker diodes varied wildly unit-to-unit
depending on contact pressure, wire tip geometry, and crystal surface preparation.
This is historically accurate — operators had to "find a good spot" by adjusting
the whisker.

## Tungsten Compound Semiconductors

These are actual semiconductors with well-characterized band structures.

### Tungsten Trioxide (WO3)

| Property | Value | Notes |
|----------|-------|-------|
| Band gap | 2.6–2.8 eV | Wide-gap, n-type |
| Crystal structure | Monoclinic (room temp) | |
| Electron mobility | 10–20 cm²/(V·s) | Low — ionic/polar semiconductor |
| Dielectric constant | ~50 | High |

Applications: electrochromic devices, gas sensors. Not used in transistors.
Too wide a gap and too low mobility for conventional diode/BJT modeling.

### Tungsten Diselenide (WSe2)

| Property | Value | Notes |
|----------|-------|-------|
| Band gap (bulk) | 1.2 eV | Indirect |
| Band gap (monolayer) | 1.65 eV | Direct — 2D semiconductor |
| Electron mobility | 30–100 cm²/(V·s) | Bulk |
| Hole mobility | 250–500 cm²/(V·s) | Monolayer, p-type dominant |
| Carrier type | Ambipolar (p-type dominant) | |
| Dielectric constant | ~7.2 | |
| Intrinsic carrier conc (est) | ~1e6 cm⁻³ | At 300K, from Eg=1.2 eV |

Most promising tungsten compound for transistor-like devices. Monolayer WSe2
FETs have been demonstrated in research with on/off ratios >1e6.

**Hypothetical SPICE diode from WSe2 bulk:**
```
.model WSE2_DIODE D(IS=1e-14 N=1.3 RS=500 CJO=0.1p EG=1.2)
```
- IS very low (wide gap, low ni)
- N > 1 (recombination in depletion region dominates in these materials)
- RS high (low mobility)

### Tungsten Disulfide (WS2)

| Property | Value | Notes |
|----------|-------|-------|
| Band gap (bulk) | 1.3 eV | Indirect |
| Band gap (monolayer) | 2.1 eV | Direct — 2D semiconductor |
| Electron mobility | 50–200 cm²/(V·s) | Monolayer |
| Carrier type | n-type dominant | |
| Dielectric constant | ~6.6 | |
| Intrinsic carrier conc (est) | ~3e5 cm⁻³ | At 300K, from Eg=1.3 eV |

Similar to WSe2 but n-type. Slightly wider bulk gap.

## Comparison Table

| Material | Eg (eV) | ni (cm⁻³) | µe (cm²/V·s) | µh (cm²/V·s) | Typical IS |
|----------|---------|-----------|---------------|---------------|------------|
| Germanium | 0.67 | 2.4e13 | 3900 | 1900 | 1e-6 A |
| Silicon | 1.12 | 1.5e10 | 1400 | 450 | 1e-12 A |
| GaAs | 1.42 | 1.8e6 | 8500 | 400 | 1e-16 A |
| WSe2 (bulk) | 1.2 | ~1e6 | 30–100 | 250–500 | ~1e-14 A |
| WS2 (bulk) | 1.3 | ~3e5 | 50–200 | — | ~1e-15 A |
| WO3 | 2.6 | negligible | 10–20 | — | ~1e-30 A |
| W metal | — (conductor) | — | — | — | — |

## Implications for Melange Modeling

### What's modelable now (no melange changes needed)

1. **Tungsten point-contact diode**: Standard Shockley diode `.model` with
   appropriate IS/N/RS/CJO/BV. Use the `W_POINT_CONTACT` parameters above as
   a starting point. Melange's existing diode model handles this directly.

2. **WSe2 or WS2 Schottky diode**: Standard diode model with EG parameter
   (if temperature modeling is added) and adjusted IS/N/RS.

### What would need new work

1. **2D material FETs (WSe2/WS2 monolayer transistors)**: These are MOSFETs
   with very different subthreshold behavior, quantum capacitance effects, and
   contact resistance dominance. Would need a new device model — not just
   different SPICE parameters on the existing MOSFET Level 1.

2. **Temperature coefficients**: Melange currently fixes all devices at 27°C.
   Tungsten compounds have different temperature behavior than Si/Ge (wider gap
   = less temperature sensitivity for IS, but mobility degrades differently).

### For the Tungsten Thunder Horse circuit

The `TUNGSTEN_*` tube models in that circuit are fictional triode voicings, not
tungsten material models. They use the standard Koren triode equations with
custom MU/EX/KG1/KP/KVB parameters chosen for tonal character. No material
property connection to actual tungsten.

If someone wanted to model a tube with a tungsten cathode (which is historically
accurate — many real tubes use thoriated tungsten cathodes), the cathode material
affects emission characteristics (thermionic emission follows Richardson-Dushman):
```
J = A * T² * exp(-φ / (k*T))
```
where A ≈ 60–120 A/(cm²·K²) for thoriated tungsten (vs ~30 for oxide-coated).
This is not currently modeled in melange's Koren tube model, which lumps cathode
emission into the KG1 parameter.
