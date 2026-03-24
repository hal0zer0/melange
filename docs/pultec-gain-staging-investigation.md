# Pultec EQP-1A Gain Staging Investigation

## Problem

The melange Pultec circuit produces **+31 dB gain** at "all controls off" (pots at minimum).
The real Pultec EQP-1A is **nominally unity gain** — the passive EQ network has ~16-20 dB
insertion loss that the tube makeup amp compensates for. Our circuit is missing this insertion loss.

**Impact**: At normal DAW signal levels (speech near 0 dBFS = ±1V), the +31 dB gain overdrives
the tubes so severely that the Newton-Raphson solver fails on 97% of samples, producing noise.
At 100mV input the circuit is perfectly clean and stable (zero NR failures).

## What the Real Pultec Does

- Input: +4 dBu (1.228 Vrms) via 600Ω balanced input transformer
- Passive LC EQ network: **16-20 dB insertion loss** even at "flat" settings
- Tube makeup amplifier: ~16-20 dB gain (2× 12AX7 + 2× 12AU7)
- Output: ~unity gain (±1 dB) with global NFB from S-217-D tertiary winding
- The insertion loss is a fundamental property of the passive LC topology — signal passes
  through a resistive pad (the boost/cut pots) even when "off"

## Measured Results (codegen, 2026-03-23)

### "Flat" response (all pots at `.pot` default = minimum boost/cut):
```
20Hz: +29 dB    100Hz: +32 dB    1kHz: +31 dB    10kHz: +24 dB    20kHz: -2 dB
```
Expected: ~0 dB across the band.

### EQ controls DO work correctly (relative to flat):
- LF Boost: +31 dB shelf at 20Hz, shifts with frequency switch ✓
- LF Atten: -21 dB low cut ✓
- HF Boost: +13 dB bell at selectable frequencies ✓
- HF Cut: -14 dB rolloff ✓
- Pultec trick (simultaneous boost+cut): correct resonant shelf shape ✓

### Stability vs input level:
| Input Level | Output | NR Failures | Status |
|-------------|--------|-------------|--------|
| 10mV | 0.35V (+31 dB) | 0 | Clean |
| 100mV | 3.0V (+30 dB) | 0 | Clean |
| 250mV | 8.75V (clipped) | 0 | Clean (white noise) |
| 1.0V (0 dBFS) | clipped at 10V | 46,830/48,000 (97%) | **Noise** |

The 1V input causes tube plate voltages to swing to 1169V (from 290V DC OP).

## Circuit File

**`circuits/pultec-eq.cir`** — 58 elements, 32 nodes, 4 tube devices (2× 12AX7, 2× 12AU7)

### Passive EQ Network Topology

Signal flow through passive section:
```
eq_in ──┬── R_hfb_u ── b ──┬── R_hfb_l ──┬── eq ──┬── R_hfc_u ── f ──┬── R_hfc_l ── mid
        │              (HF boost LC)       │        │            (HF cut LC)          │
        │              C_hfb + L_hfb       │        │            C_hfc                │
        │                                  │        │                                 │
        │                                  ├─ C_lfa ┤                                 ├─ R_lfb ── gnd
        │                                  └─ R_lfc ┘                                 └─ C_lfb
        │                                  (LF atten)                                 (LF boost)
        │
        └─ to V1 tube grid (12AX7 input stage)
```

The `eq_in` node connects to both:
1. The start of the passive EQ chain
2. The first tube's grid (V1, 12AX7 input stage)

### Pot Defaults (`.pot` directives)

| Pot | Range | Default | Function | At Default |
|-----|-------|---------|----------|------------|
| R_hfb_u | 100-10k | 10k | HF Boost upper | Max R = bypass |
| R_hfb_l | 100-10k | 100 | HF Boost lower | Min R = no boost |
| R_hfb_q | 100-2.5k | 2.5k | Bandwidth | Max R |
| R_lfb | 100-10k | 100 | LF Boost | Min R = no boost |
| R_lfc | 100-100k | 100 | LF Atten | Min R = no atten |
| R_hfc_u | 100-1k | 1k | HF Cut upper | Max R = no cut |
| R_hfc_l | 100-1k | 100 | HF Cut lower | Min R = no cut |

### Key Components in Signal Path

```
R_in    in eq_in 620          ← 600Ω input impedance
R_hfb_u eq_in b 10k          ← HF boost upper pot (default 10k)
R_hfb_l b eq 10k             ← HF boost lower pot (default 100Ω)
R_hfc_u eq f 1k              ← HF cut upper pot (default 1k)
R_hfc_l f mid 1k             ← HF cut lower pot (default 100Ω)
R2      h out_tap 1k         ← LF path output tap
R3      out_tap mid 10k      ← LF path to mid node
```

### Questions for Investigation

1. **Where does the insertion loss come from in the real Pultec?**
   The passive network (pots + LC filters) between `eq_in` and `mid` should attenuate
   the signal by 16-20 dB. With pots at minimum boost/cut, is the signal path through
   the LC network providing this loss, or is the path too low-impedance?

2. **Is the "all pots minimum" state correct for "flat"?**
   In the real Pultec, the "flat" position might not be "all pots at minimum." The Pultec's
   calibration procedure involves adjusting a trim pot for unity gain. There might be specific
   pot positions that give flat response with proper insertion loss.

3. **Is the NFB (negative feedback) loop providing enough gain reduction?**
   The S-217-D transformer tertiary winding provides global NFB. The circuit has this winding
   connected. Is the NFB loop gain correct? (Currently NFB provides ~3 dB stabilization.)

4. **Does the EQ network topology match the real Pultec schematic?**
   Verify against the original Sowter DWG E-72,658-2 schematic that:
   - The signal path from input to tube grid passes through the entire passive network
   - The passive network has the correct pad/attenuator structure
   - The node connections are correct (especially around `eq_in`, `eq`, `mid`, `out_tap`)

## Reference Files

| File | Purpose |
|------|---------|
| `circuits/pultec-eq.cir` | Full SPICE netlist |
| `docs/aidocs/STATUS.md` | Validation results and device support |
| `docs/aidocs/DEVICE_MODELS.md` | Tube model equations (Koren) |

## Verified Schematic Sources

- Sowter DWG E-72,658-2 (primary schematic reference)
- Peerless/Triad winding data for HS-29 and S-217-D transformers
- Component values verified 2026-03-16 (see memory files)

## Node-by-Node Signal Trace (1kHz, 10mV input, runtime NodalSolver)

```
Node              Peak (V)  Gain (dB)   Notes
INPUT             0.010V        0.0     Reference
in                5.726V      +55.2     ← Problem: 570× voltage gain at input!
eq_in             5.423V      +54.7     Through R_in_series (620Ω) — minimal loss
b                 0.613V      +35.8     After R_hfb_u (10k) — 19 dB attenuation
eq                0.565V      +35.0     After R_hfb_l (100Ω)
f                 0.077V      +17.7     After R_hfc_u (1k) — 17 dB attenuation
mid               0.034V      +10.7     After R_hfc_l (100Ω)
out_tap           0.581V      +35.3     Through C_lfa/R_lfc path — higher than mid!
h                 0.567V      +35.1     Before R2 (1k) to out_tap
grid1            11.877V      +61.5     HS-29 transformer 1:2 step-up from out_tap
grid2            14.343V      +63.1     HS-29 secondary
plate1          196.667V      +85.9     V1 12AX7 amplification
plate2          178.050V      +85.0     V1 12AX7 other phase
out             105.558V      +80.5     Output transformer secondary
OUTPUT (DC blk)  10.000V      +60.0     Clamped at ±10V by DC block
```

### Key Observation

The "in" node has +55 dB gain from a 10mV input. This means the circuit's input impedance
at the "in" node is ~341kΩ (dominated by the HS-29 transformer primary inductance reflected
through the passive network). The Thevenin source model (R_in=600Ω, G_in=1/600) injects
current that develops a huge voltage across this high impedance.

**In the real Pultec**, the input transformer is voltage-driven from a 600Ω source. The
transformer's primary IS the 600Ω termination — the signal voltage at the primary is close
to the source voltage. Our Thevenin model doesn't account for the transformer's role as an
impedance-matching device.

### Signal Path Through Passive EQ

The passive network DOES attenuate: `eq_in`(+55dB) → `b`(+36dB) → `eq`(+35dB) → `f`(+18dB) → `mid`(+11dB). That's ~44 dB of attenuation through the series pots/LC network. But the starting point is +55 dB (not 0 dB), so the output of the passive section is still +11 dB.

The `out_tap` node (+35 dB) feeds the HS-29 transformer which steps up to `grid1` (+62 dB).
This is 27 dB higher than `out_tap` — consistent with the 1:2 transformer ratio (+6 dB)
plus tube grid impedance effects.

## Experimental Results (2026-03-23)

### Fix attempts and their effects:

| Fix | Change | Result |
|-----|--------|--------|
| 1. R_fb_b wiring | `fb_s5 0` → `fb_s5 cathode` | No change (+31 dB) — shared cathode defeats NFB |
| 2. Tertiary polarity | `L_fb_a fb_s5 fb_s3` → `fb_s3 fb_s5` | No change (+31 dB) — same reason |
| 3. Rload 600Ω | `100k` → `600` | -20 dB at 1kHz, but massive HF loss (-23 dB at 5kHz) — transformer model can't drive 600Ω |
| 1+2+3 combined | All three | +2 dB at 50Hz, -15 dB at 1kHz — HF rolloff from 600Ω load |
| 4. Split cathodes | Shared 820Ω → 2× 410Ω + differential FB | **+53 to +60 dB — POSITIVE FEEDBACK** |
| 4 + reversed tertiary | Split cathodes + swapped L_fb_a nodes | **+60 dB — still positive feedback** |

### Key findings:

1. **Fixes 1-2 are no-ops** with shared cathode (schemer's theory confirmed)
2. **600Ω load** works but our simplified transformer model (coupled inductors K=0.95) can't
   drive it — too much HF loss. Real S-217-D secondary would need proper winding R, leakage L.
3. **Split cathodes create positive feedback** regardless of tertiary polarity. The schemer's
   polarity analysis may have an error, or the melange transformer coupling model creates
   different phase relationships than assumed.
4. **The NFB loop polarity needs to be determined empirically** by testing both tertiary
   orientations with the correct cathode topology, not by tracing signal phase theoretically.

### Reverted to: original circuit (all fixes reverted)
All EQ curves still work at +31 dB baseline. Circuit is functional but with wrong absolute gain.

## What We Need

The circuit should produce **~0 dB net gain** at flat EQ settings, not +31 dB. The passive
EQ network needs to provide ~16-20 dB of insertion loss to keep the tube amp in its linear
operating range. The EQ controls (boost/cut) should modulate around that baseline.

Once the gain staging is correct, the circuit will handle normal DAW signal levels (±1V)
without NR convergence failures.
