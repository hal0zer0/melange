# Klon Centaur Circuit Research

## Overview

The Klon Centaur (Bill Finnegan, 1994-2000) uses:
- **2× TL072** dual JFET op-amps (4 sections total)
- **1× MAX1044/ICL7660S** charge pump (voltage doubler/inverter)
- **2× 1N34A** germanium clipping diodes (matched pair, anti-parallel)
- **3 pots**: GAIN (100kB dual-gang), TREBLE (10kB), OUTPUT (10kB)

## Power Supply / Voltage Rails

The charge pump generates multiple rails from a 9V battery:

| Rail | Voltage | Source | Powers |
|------|---------|--------|--------|
| V+ | +9V | Battery direct | U1 (buffer + gain stage) |
| VB+ | +4.5V | R29/R30 (27k/27k) divider, C21 (47uF) bypass | Virtual ground / bias |
| V- | -9V | MAX1044 inversion | U2 negative supply |
| V2+ | +18V | MAX1044 doubler | U2 positive supply |

- U1 (TL072 #1): powered 0V to +9V (9V swing)
- U2 (TL072 #2): powered -9V to +18V (**27V swing** — the key headroom innovation)
- VB+ (4.5V) is the signal reference point for all op-amp stages

**For audio simulation**: The charge pump is DC-only. Model as ideal voltage sources.

## Signal Path — 6 Functional Sections

### 1. Input Buffer (U1A — unity-gain follower)

```
Guitar → R1 (10k, ESD) → C1 (100nF, DC block) → U1A (+)
                                                    |
                                              R2 (1M) → VB+ (bias)
                                                    |
                                              U1A out = U1A (-)  [follower]
```

- Input impedance: ~1M ohm (good for guitar pickups)
- HP filter: fc = 1/(2π·1M·100nF) = **1.6 Hz** (negligible)
- Output splits into THREE parallel paths

### 2. Gain Stage (U1B — non-inverting amp with bandpass)

```
Buffer out → C3 (100nF) → U1B (+)
                            |
                      C5 (68nF) ∥ R6 (10k)  [to GAIN1 wiper]
                            |
                      GAIN1 pot (100k, variable ground leg)
                            |
                      R3 (2k, minimum R)

U1B feedback:
    U1B out → R12 (422k) ∥ C8 (390pF) → U1B (-)
                                          |
                                    R11 (15k) ∥ C7 (82nF) → [to R3/GAIN1]
```

**Gain formula** (mid-band):
```
Gain = 1 + R12 / (R11 + R3 + R_pot)
     = 1 + 422k / (15k + 2k + R_pot)
```

| Pot Position | R_pot | Gain | dB |
|-------------|-------|------|----|
| Min (CCW) | 100k | 4.6x | 13.3 dB |
| Max (CW) | 0Ω | 25.8x | 28.2 dB |

**Frequency shaping** (bandpass):
- C7 (82nF) ∥ R11 (15k): HP at ~130 Hz
- C8 (390pF) ∥ R12 (422k): LP at ~970 Hz
- **Peak gain at ~1 kHz** — the famous "mid hump"
- At max gain, peak can reach ~40 dB (100x) at 1 kHz

### 3. Clipping Stage (D2/D3 — shunt germanium)

```
U1B out → C9 (1uF) → R13 (1k) → diode junction → to summing amp
                                        |
                                   D2 (1N34A) ↑ + D3 (1N34A) ↓  [anti-parallel]
                                        |
                                   R16 (47k) → VB+ (4.5V)
```

**CRITICAL**: Diodes are **shunt to VB+** (hard clipping), NOT in the op-amp feedback loop.

- At low-moderate gain: signal < 0.35V Vf → diodes inactive → clipping comes from **op-amp rails**
- At high gain: diodes engage, germanium gives soft knee
- Both diodes are identical 1N34A — **symmetric clipping** (no asymmetry)

**1N34A parameters**:
- IS = 2.6µA (datasheet) or ~15µA (measured on originals)
- N = 1.6 (emission coefficient)
- RS = 7Ω
- Vf ≈ 0.3-0.35V

### 4. Feedforward Network 1 (Clean LF — always on)

```
Buffer out → C3 → R7 (1.5k) → junction → R19 (15k) → summing node
                                  |
                                C16 (1uF) → VB+
```

- Low-pass filter, fc ≈ 10-106 Hz
- **Always active** — not controlled by the gain pot
- Preserves clean bass in the final mix

### 5. Feedforward Network 2 (Clean variable — gain-dependent)

```
Buffer out → R5 (5.1k) / C4 (68nF) → R8 (1.5k) → GAIN2 pot (100k)
                                                        |
                                                   → summing node
```

Additional filtering: R9 (1k) + C6 (390nF), R15 (22k) + C11 (2.2nF), R17 (27k) ∥ (R18 (12k) + C12 (27nF))

**GAIN2** (second gang of dual pot) works **inversely** to GAIN1:
- Gain up → GAIN2 attenuates clean signal
- Gain down → GAIN2 passes more clean signal
- This is the "transparent" crossfade — you always hear some clean signal at low-moderate gain

### 6. Summing Amplifier (U2A — inverting)

```
[Clipped signal] ──→ summing node → U2A (-) ← R20 (392k) ∥ C13 (820pF) ← U2A out
[FF1 clean LF]  ──→ summing node
[FF2 clean var] ──→ summing node
```

- Inverting summer combines all three paths
- R20 (392k) ∥ C13 (820pF): LPF at fc ≈ 495 Hz (more mid shaping)
- Powered by -9V/+18V rails — **27V headroom**, never clips the summed signal

### 7. Tone Control (U2B — active high-shelf)

```
U2A out → R22 (100k) → U2B (-) ← C14 (3.9nF) → TREBLE pot (10k) wiper
                                                      |
                                                R21 (1.8k) — pot top — R23 (4.7k)
                                                      |
                                              R24 (100k) feedback ← U2B out
```

- Active high-frequency shelving EQ
- Shelf frequency: fc ≈ 408 Hz
- Treble boost: up to +18 dB
- Treble cut: down to -8 dB
- Also inverting — double inversion (U2A + U2B) restores correct phase

### 8. Output Stage

```
U2B out → C15 (4.7uF, DC block) → R25 (560Ω) → OUTPUT pot (10kB log) → output jack
                                                                            |
                                                                      R28 (100k, pulldown)
```

## The "Transparent" Character — Why It Sounds Different

1. **Clean blend** (the big one): dual-ganged pot crossfades clean/clipped signal continuously
2. **Germanium Vf = 0.35V**: soft clipping knee, gradual onset
3. **Diodes often inactive**: at low-moderate gain, clipping is from op-amp rail saturation
4. **27V headroom** on summing/tone stages: post-clipping stages never compress
5. **Bandpass gain stage**: concentrates clipping in the midrange (~1 kHz), avoids muddy bass/harsh treble distortion
6. **Bass feedforward always on**: clean low-end preserved regardless of gain setting

## Melange Implementation Considerations

### What melange already supports
- Diodes (1N34A germanium with IS/N/RS params)
- Op-amps (VCCS model — but currently ideal, no rail clipping)
- Potentiometers (`.pot` with Sherman-Morrison)
- Wiper pots (`.wiper` for dual-function pots)

### What would need work
1. **Op-amp rail clipping**: Current op-amp model is ideal (infinite headroom). The Klon's gain stage clips at 0V/9V rails — this is a major part of the sound. Need op-amp model with supply rails and soft clipping near rails.
2. **Dual-ganged pot**: Two pots mechanically linked (same knob). GAIN1 and GAIN2 move together. Could model as two `.pot` directives controlled by a single parameter.
3. **Charge pump as DC sources**: Replace MAX1044 with `VCC_18V 18 0 18V` and `VCC_NEG9 neg9 0 -9V` voltage sources
4. **Multiple supply domains**: U1 on 0/9V, U2 on -9V/+18V — different headroom per op-amp section

### Simplified first approach
- Model charge pump as DC sources (done at netlist level)
- Use ideal op-amps initially (test the topology)
- Add rail clipping to op-amp model later for accuracy
- The diode clipping and clean blend should work with current melange features

## Complete BOM (Gold Edition — Consensus Values)

### Resistors
| Ref | Value | Function |
|-----|-------|----------|
| R1 | 10k | Input ESD/series |
| R2 | 1M | Input bias to VB+ |
| R3 | 2k | Gain stage minimum R |
| R5 | 5.1k | FF2 input series |
| R6 | 10k | Parallel with C5 |
| R7 | 1.5k | FF1 series |
| R8 | 1.5k | FF2 series |
| R9 | 1k | FF2 series with C6 |
| R11 | 15k | Gain feedback (with C7) |
| R12 | 422k | Gain feedback main |
| R13 | 1k | Post-clipping series |
| R15 | 22k | FF2 series with C11 |
| R16 | 47k | Diode bias to VB+ |
| R17 | 27k | FF2 parallel |
| R18 | 12k | FF2 series with C12 |
| R19 | 15k | FF1 to summing |
| R20 | 392k | Summing amp feedback |
| R21 | 1.8k | Tone control |
| R22 | 100k | Tone input R |
| R23 | 4.7k | Tone control |
| R24 | 100k | Tone feedback R |
| R25 | 560 | Output series |
| R28 | 100k | Output pulldown |
| R29 | 27k | VB+ divider |
| R30 | 27k | VB+ divider |

### Capacitors
| Ref | Value | Function |
|-----|-------|----------|
| C1 | 100nF | Input DC block |
| C3 | 100nF | Buffer-to-gain coupling |
| C4 | 68nF | FF2 coupling |
| C5 | 68nF | Gain stage (with R6) |
| C6 | 390nF | FF2 (with R9) |
| C7 | 82nF | Gain feedback HP |
| C8 | 390pF | Gain feedback LP |
| C9 | 1uF | Gain-to-diode coupling |
| C10 | 1uF | Diode-to-summing coupling |
| C11 | 2.2nF | FF2 (with R15) |
| C12 | 27nF | FF2 (with R18) |
| C13 | 820pF | Summing amp LPF |
| C14 | 3.9nF | Tone control shelf |
| C15 | 4.7uF | Output DC block |
| C16 | 1uF | FF1 (to VB+) |
| C17 | 47uF | V+ filter |
| C18 | 47uF | VB+ bypass |
| C20 | 20uF | V2+ filter |
| C21 | 47uF | VB+ bypass |

### Semiconductors
| Ref | Part | Function |
|-----|------|----------|
| U1 | TL072 | Buffer (U1A) + Gain (U1B) |
| U2 | TL072 | Summer (U2A) + Tone (U2B) |
| D2 | 1N34A | Germanium clipping |
| D3 | 1N34A | Germanium clipping |

### Potentiometers
| Control | Value | Type | Function |
|---------|-------|------|----------|
| GAIN | 100kB dual-gang | Linear | Gang A: gain, Gang B: clean blend |
| TREBLE | 10kB | Linear | Tone control |
| OUTPUT | 10kB | Linear (or log) | Volume |
