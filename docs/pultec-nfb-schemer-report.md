# Pultec EQP-1A NFB Issue — Report for Schemer

## Summary

The NFB loop activates with a single wiring fix (`R_fb_b fb_s5 cathode 360` instead of `fb_s5 0`). But the feedback is **60 dB too strong**: gain goes from +31 dB (broken NFB) to -29 dB (overcorrected). Target is ~0 dB.

The 33 dB discrepancy between calculated loop gain (~27 dB) and measured loop gain (~60 dB) needs investigation. The S-217-D transformer parameters are the prime suspect.

## The One Fix That Activates NFB

```
BEFORE: R_fb_b fb_s5 0 360        (tertiary return to ground — NO feedback)
AFTER:  R_fb_b fb_s5 cathode 360  (tertiary return to cathode — FEEDBACK ACTIVE)
```

This is confirmed correct per Sowter schematic: "12AX7 pin 3 → 360Ω → S-217-D pin 3, 12AX7 pin 8 → 360Ω → S-217-D pin 5". Both 360Ω resistors go to the shared cathode.

No other topology changes are needed. The dot convention, HS-29 pin 8 → grid2, and Rk to ground are all confirmed correct by schemer.

## Measured Results

| Configuration | Gain (1kHz, 10mV) | Notes |
|--------------|-------------------|-------|
| Original (R_fb_b to ground) | **+31.5 dB** | NFB completely dead |
| R_fb_b to cathode | **-29.2 dB** | NFB active but way too strong |
| Feedback winding removed entirely | +31.5 dB | Confirms tertiary has zero effect when R_fb_b→ground |
| Target (real Pultec) | **~0 dB** | Unity gain device |

## The Discrepancy

### Calculated loop gain: ~27 dB
```
Tertiary turns ratio: N_tert/N_pri_half = 142/1575 = 0.0902
Voltage ratio: K × 0.0902 = 0.95 × 0.0902 = 0.086 (-21.3 dB)
Feedback fraction: Rk/(Rk + R_fb_a||R_fb_b) = 820/1000 = 0.82 (-1.7 dB)
Open-loop amp gain: ~+50 dB (12AX7 + 12AU7, two stages)
Loop gain = 50 - 21.3 - 1.7 = 27 dB
Expected closed-loop = 50 - 27 = +23 dB → with 16-20 dB passive EQ loss → +3 to +7 dB
```

### Measured loop gain: ~60 dB
```
Open-loop gain: +31.5 dB (measured without feedback)
Closed-loop gain: -29.2 dB (measured with feedback)
Loop gain = 31.5 - (-29.2) = 60.7 dB
```

### The 33 dB gap
The simulation shows 60 dB of loop gain where the calculation predicts 27 dB. Something in the transformer model is delivering ~33 dB more feedback voltage than the turns ratio predicts.

## Questions for Schemer

### Q1: Is the push-pull dot convention causing additive tertiary coupling?

Current circuit:
```
L_opt_a opt_a opt_ct 55   (dot at plate A)
L_opt_b opt_ct opt_b 55   (dot at center tap)
```

In the coupled inductor model, the tertiary voltage from each primary half is:
```
V_tert = M_a × dI_a/dt + M_b × dI_b/dt
```

With both K_fba = K_fbb = 0.95 (same sign), the push-pull differential signal may cause the contributions to ADD rather than partially cancel, depending on the current direction conventions.

**Question**: With L_opt_b's dot at opt_ct (not opt_b), does the SPICE dot convention cause the two primary halves to create canceling or additive contributions to the tertiary? If additive, the effective tertiary voltage is 2× the single-half calculation, which is +6 dB.

If we swap L_opt_b to `opt_b opt_ct 55` (dots at both plate ends), the gain changes from -29 dB to... still -29 dB (tested both ways — no change). So the dot convention doesn't seem to matter for this circuit. The push-pull cancellation issue I hypothesized earlier was wrong.

### Q2: S-217-D inductance — 220H vs 38.1H

The S-217-D primary is currently modeled at 55H per half (from 220H total at 30Hz). CineMag measures 38.1H at 1kHz.

The turns ratio (and voltage ratio) is independent of inductance. But the impedance affects loading:
- At L_tert = 0.447H (220H ref): Z_tert@1kHz = 2808Ω → shunt loading: 1000Ω out of 3808Ω = 26% loss
- At L_tert = 0.0775H (38.1H ref): Z_tert@1kHz = 487Ω → shunt loading: 1000Ω out of 1487Ω = 67% through

**Lower L_tert means less voltage at the tertiary terminals** (more of the induced EMF drops across the winding's own impedance when loaded). Could this reduce the feedback from -29 dB toward 0 dB?

**Question**: Should we scale the S-217-D to the 38.1H@1kHz reference? The HS-29 already uses 37H (from Sowter 9330 at ~1kHz).

### Q3: Tertiary coupling K — schemer confirmed bifilar, K ≥ 0.99

Schemer confirmed the tertiary is bifilar with the primary. K should be ≥0.99. Currently K_fba/K_fbb/K_sec_fb = 0.95.

Raising to K=0.99 has NO effect on the gain (tested: still -29 dB with either K=0.95 or K=0.99). The feedback strength appears insensitive to K in this range.

### Q4: What else could cause 33 dB excess loop gain?

Possibilities:
1. **The trapezoidal discretization** at 48kHz creates different transformer dynamics than continuous-time. At 48kHz, the inductor impedances are scaled by 2/T = 96000, which may distort the feedback transfer function.
2. **The augmented MNA inductor model** may handle the mutual inductance differently than a SPICE companion model, giving different effective coupling.
3. **The DC operating point** with the NFB active may put the tubes in a very different bias region (high-gain cutoff), amplifying the feedback effect.
4. **The 12AX7 open-loop gain** may be much higher than +50 dB. With mu=100 per section and 200K plate loads, the voltage gain per section is ~76 (37.6 dB). Two sections = 75 dB. The 12AU7 adds another ~15-20 dB. Total open-loop could be 90+ dB.

**If open-loop gain is 90 dB** instead of 50 dB:
```
Loop gain = 90 - 23 = 67 dB (close to measured 60 dB)
Closed-loop = 90 - 67 = +23 dB → with passive EQ loss → +3 to +7 dB ≈ near unity ✓
```

But our measurement shows +31.5 dB open-loop, not 90 dB. Unless the +31.5 dB IS the closed-loop gain even with R_fb_b to ground (residual feedback through stray paths)?

## Signal Path (for reference)

```
Input → passive EQ → HS-29 → 12AX7 push-pull → 12AU7 push-pull →
S-217-D primary (L_opt_a, L_opt_b) → S-217-D secondary (output)
                                    ↓ (magnetic coupling K=0.95)
                              S-217-D tertiary (L_fb_a, 0.447H)
                                    ↓
                    fb_s3 ← R_fb_shunt (1000Ω) → fb_s5
                      ↓                              ↓
                R_fb_a (360Ω)                   R_fb_b (360Ω)
                      ↓                              ↓
                    cathode ← Rk (820Ω) → ground
                      ↑
              12AX7 T1a + T1b (shared cathode)
```

## Circuit File

`circuits/pultec-eq.cir` — currently at original topology (+31 dB). The R_fb_b fix is confirmed correct but produces -29 dB instead of ~0 dB.

## What We Need From Schemer

1. Verify whether the S-217-D should use 38.1H@1kHz instead of 220H@30Hz
2. Check the open-loop gain of the two-stage amplifier (is +31 dB really open-loop, or is there residual feedback?)
3. Any insight into why the coupled inductor model delivers more feedback than the turns ratio calculation predicts
4. Whether an ngspice simulation of this circuit with R_fb_b to cathode shows similar behavior
