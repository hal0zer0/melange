# BA284 Schematic Analysis — Notes for Melange Agent

Source: Neve drawing EX/10,284 Rev C, 15/11/72 (CN 10,362)
PDF saved at: `docs/BA284-EX10284.pdf`

## Architecture

The BA284 is a **"Triple Amplifier"** — three independent 3-transistor gain stages on one PCB. **All 9 transistors are BC184C** (NPN). No PNP, no power transistors.

| Section | Transistors | Function | External Use |
|---------|-------------|----------|-------------|
| 1 (TR1-TR3) | BC184C ×3 | Mic preamp (= BA283 AV) | Same as preamp, with gain switch |
| 2 (TR4-TR6) | BC184C ×3 | LF/HF EQ amplifier | Drives B205 balanced bridge |
| 3 (TR7-TR9) | BC184C ×3 | Presence EQ amplifier | Drives BA211 presence bridge |

**Sections 2 and 3 have identical internal topology and component values.**

## Internal Topology (Sections 2 and 3)

**CRITICAL: R27/R17(120K) is INTER-STAGE COUPLING, not feedback.**

User traced the schematic (2026-04-02): R17(120K) connects TR4 collector to TR5 base.
It is NOT feedback from the output to TR4 (neither base nor emitter).

This makes BA284 §2/§3 a true discrete operational amplifier:
- No internal overall feedback (unlike Section 1 which has R9=2K2 from emit3→emit1)
- ALL feedback is external (from B205/BA211 boards on the mainboard)
- R15/R25(27K) provides LOCAL DC feedback: TR5 emitter → TR4 base (1 inversion = negative)
- C11/C19(47pF) Miller comp provides single dominant pole for stability
- C15(100µF) bypasses TR4 emitter → AC virtual ground → very high open-loop gain
- The Nyan-1073-EQ replacing these with NE5532 op-amps is topologically correct

```
          +24V ───────────────────────────────────────────
           │          │              │              │
          R23(100K)  R18(180K)     R30(3K6)        │
           │          │              │              │
          base7      base8         coll8 ──→ TR9 base
           │          │   │          │              │
          C19(47p)   R19  C14      TR8/TR5 CE    TR9/TR6 EF
           │        (180K)(22µ)     │              │
          coll7──GND  │   │        emit8          emit9──output
           │     │   GND GND        │              │    │
          C20   R27(120K)          R31(820)       R32   C24
         (470p)  │                   │           (3K9) (1nF)
           │    base8               emit8_j        │    │
          GND   (see above)          │   │        GND  GND
                                   C23(100µ)
        TR7/TR4 CE                   │
           │                        GND
  base ────┼── R23 (100K) ── +24V   (DC bias pull-up)
           │── R24 (39K) ── GND     (DC bias pull-down → Vbase≈6.7V)
           │── R25 (27K) ── emit8_j (LOCAL DC feedback, 1 inversion = negative)
           │── C19 (47pF) ── coll7  (Miller comp → dominant pole)
           │── C12 (470pF) ── emit area (base-emitter comp)
           │
  coll ────┼── R27 (120K) ── base8  (INTER-STAGE COUPLING, not feedback!)
           │── C20 (470pF) ── GND   (HF bypass)
           │── C11/C19 from base    (Miller comp, other end)
           │
  emit ────┼── R26 (3K3) ── GND     (DC emitter path)
           │── C15 (100µF) ── GND   (AC bypass → emitter is virtual ground)
           │── C12 (470pF) ── base  (base-emitter comp, other end)
           │── C21 (660pF) ── GND   (HF bypass)
           │── external feedback    (220K, 2K7+22N from BA211 to emitter)
```

Component placement (user-traced 2026-04-02):
- **R27/R17 (120K)**: INTER-STAGE COUPLING from TR7 collector to TR8 base.
  NOT a feedback resistor. This is the DC path for TR7 collector current.
- **R24/R14 (39K)**: INTER-STAGE DC FEEDBACK from TR8 emitter to TR7 base.
  User traced (2026-04-02): R14 goes from base4 area "all the way across the
  board" to land between R20(3.6K) and R21(820) = TR8/TR5 emitter node.
  Equivalent of R10(68K) in preamp (node_d → base4).
  With R23(100K) to ground: Vbase7 ≈ 0.72 × V_emit8.
- **C12 (470pF)**: Base-to-emitter compensation (like C3=1500p in Section 1).
- **C15 (100µF)**: TR7 emitter bypass → AC virtual ground → very high Av.
- **R18 (180K) + R19 (180K)**: Bias divider at TR8 base node (Vbase8≈12V).
  C14(22µ) decouples this node at AC.
- **C23 (100µF)**: TR8 emitter bypass (parts list CA61000).
- **C22 (22µF)**: Supply decoupling.
- **C25 (22µF)**: Output coupling cap.
- **C18 (10µF)**: Input coupling cap.

## Component Value Corrections

These were wrong in the tracking doc / earlier netlists:

| Component | Was | Actual | Impact |
|-----------|-----|--------|--------|
| **R17/R27** | 180K | **120K** | LF/HF EQ gain: -120K/27K = -4.44 (was -6.67). Presence DC feedback tighter. |
| **R21/R31** | 330Ω | **820Ω** | More emitter degeneration → lower stage 2 gain, better stability |
| **C11/C19** | 1pF | **47pF** | Significantly more Miller comp → lower GBW, more phase shift |
| **C15/C23** | 6µF | **100µF** (parts list CA61000) | Lower emitter bypass corner freq |

## Key Insight: Pin L = +24V, R25(27K) = Collector Load

C19 = 47pF is **confirmed** from the Neve parts list (CA10470).

**Pin L is the +24V supply for Section 3** (user-confirmed 2026-04-02).
Each BA284 section has +24V at its top rail: N (§1), D (§2), L (§3).
R25(27K), R28(180K), and C22(22µ) all hang from the L rail (+24V).

**R25(27K) from +24V to coll7 is the COLLECTOR LOAD** — not Miller feedback.
R28(180K) from +24V to base8 area (TR8 bias pull-up).
C22(22µ) from +24V to ground (supply bypass).

With R25(27K) collector load:
- Ic = 0.3-0.5 mA → Vcoll = 10-16V (healthy)
- No Miller feedback at base7 → normal input impedance
- Bridge (R_c=6.8K) can drive base7
- AC gain: Av1 = gm × R25 (moderate, depends on Ic)
- The dominant pole and presence peak behavior depend on the
  overall AC gain and whether there is local feedback

## Parts List Corrections (from archive.org 1073-fullpak)

| Component | Schematic reading | Parts list | Part # |
|-----------|------------------|------------|--------|
| C11, C19 | 47pF | **47pF confirmed** | CA10470 |
| C15, C23 | 64µF | **100µF** | CA61000 |
| C4, C13, C21 | 680pF | **660pF** | CA16800 |

## Changes Needed in Melange Netlists

### 1. `neve-1073-presence.cir` — DONE (updated)
- Replaced `.model AMPBA284 OA(AOL=10000)` with discrete 3-BJT circuit
- Same BA211 bridge topology, same external feedback values
- BC184C model matches preamp netlist

### 2. `neve-1073-eq.cir` — Needs Update
- **R17 must change from 180K to 120K**: Line 102 `R17 eq_out amp_inv 180K` → `120K`
- This changes LF/HF gain from -180K/27K = -6.67 to -120K/27K = -4.44 (13 dB)
- The ideal OA model (AOL=5000) may still work for LF/HF since the bridge shapes the response, but consider discrete model if gain profile doesn't match spec
- Also update comment on line 94: gain = -120K/27K = -4.44 (13 dB)

### 3. Section 2 vs Section 3 Usage
Both are wired differently by the mainboard despite identical internals:
- **Section 2 (LF/HF)**: Likely inverting — signal through R15(27K) to base summing node, R17(120K) feedback from output to base. B205 passive bridge shapes input signal.
- **Section 3 (Presence)**: Non-inverting — signal at base (through C18), external series-shunt feedback at emitter from BA211 board. LC positive feedback mechanism requires non-inverting for boost.

### 4. Open Questions for Simulation
- [ ] Does the discrete presence circuit converge at DC? (R27 now at emit7 — should be stable)
- [ ] Does the presence peak appear at 3.2 kHz with full boost?
- [ ] Is the peak depth/width reasonable? (~10 dB boost at full, per 1073 spec)
- [ ] Is the gain profile correct? LF ~29 dB, HF ~6 dB, with LC peak in transition
  - If gain is wrong: R18/R14/R19 placement may need adjustment
  - R26(3.3K) in parallel with R_gnd(27K) gives effective 2.94K ground ref
- [ ] Do all 6 frequency positions (360/700/1.6k/3.2k/4.8k/7.2k) produce peaks?
- [ ] If collector load (R18+R14 split) doesn't give right Vce for TR7, try:
  R24(39K) from Vcc to coll7 directly (single load, simpler)

### 5. Gain Switch Values (Section 1 = Mic Preamp)
From Neve documentation, BA284 Section 1 gain via external R between pins T and V:
- 18 dB: open
- 23 dB: 530Ω
- 28 dB: 120Ω
- 33 dB: 27Ω
- 36 dB: 15Ω
- 43 dB: 3.2Ω

(Only relevant if Section 1 is used separately from BA283 AV)
