# Neve 1073 — Implementation Tracking

**Started**: 2026-03-31
**Circuit**: Neve 1073 channel amplifier (mic preamp + 3-band EQ + HPF)
**Goal**: Full codegen pipeline — `melange compile` to real-time plugin

## Circuit Summary

All-NPN discrete Class A design, single +24V supply. ~14 BC184C + 1 2N3055.
Three signal transformers, three EQ inductors, passive LC EQ networks.

| Parameter | Estimate |
|-----------|----------|
| Total BJTs (signal path) | ~14 |
| Transformers | 3 (mic in, line in, output) |
| EQ inductors | 3 (HPF, presence×2) |
| Switches | ~6 (gain, LF freq, presence freq, HPF freq, EQ in/out, impedance) |
| Pots | ~3 (LF boost/cut, presence boost/cut, HF boost/cut) |
| Estimated N (nodes) | 30-50 |
| Estimated M (NL dim) | 20-28 raw, 10-16 after FA reduction |

## Key Risk: Nonlinear Dimension

14 BJTs × 2D = M=28 raw, exceeding MAX_M=16. Mitigations:
1. **Forward-active reduction** — Class A bias means most BJTs stay in FA (1D instead of 2D)
2. **Stage-by-stage build** — validate M at each stage before combining
3. **Raise MAX_M if needed** — mechanical change but increases matrix sizes
4. **Nodal full-LU fallback** — works for any M, just slower

## Stages

### Stage 1: Output Amplifier (BA283 output)
- **Status**: COMPLETE — SPICE validated
- **Components**: 3 BJTs (2× BC184C + 1× 2N3055), output transformer (LO1166)
- **Topology**: CE input (TR1) → Darlington CE pair (TR2+TR3) → LO1166 transformer
- **Why first**: Most sonically important stage, self-contained, simplest feedback
- **Actual**: N=13, M=6 (3 BJTs × 2D), augmented to N=25 with 2 inductor variables
- **Transformer**: LO1166 (Carnhill VTB1148), DC-gapped, ~70mA Class A bias through primary
  - 4× 150 ohm windings, configurable. Typically 1:1.67 step-up (200Ω:600Ω).
  - Primary inductance: **TBD** — 305mH gives LF rolloff at 874Hz; needs ~5-10H for flat 20Hz response
- **Schematic sources** (downloaded):
  - `/tmp/ba283av_300dpi-2.png` — BA283 AM & AV circuit (Neve drawing EX/10 283)
  - `/tmp/ba283av_300dpi-1.png` — parts list + layout + general description
  - `/tmp/neve_1073_300dpi-1.png` — full 1073 channel amplifier (EH10023)
- **Verified parts list** (BA283 AM, from EX/10 283):
  - R1=2.2K, R2=56K, R3=68K, R4=1.2K, R5=3.3K, R6=18K, R7=47Ω, R8=33K
  - RV1=4.7K preset, C1=10µF, C2=220pF, C3=4700pF, C4=80µF, C5=330pF, C6=C7=100µF
- **Tasks**:
  - [x] Find/verify BA283 output schematic (component values from EX/10 283)
  - [x] Source BC184C and 2N3055 SPICE model parameters (added to catalog)
  - [x] Write netlist: `circuits/unstable/neve-1073-output.cir`
  - [x] Validate DC operating point (TR3 bias ~70mA, stable with R8 from coll1)
  - [x] Frequency response — ~21 dB unbalanced gain matches 25 dB gain table entry
  - [x] **SPICE validation PASSED**: correlation 0.999999 (6 nines), RMS 0.11%, gain 3.4× matches ngspice exactly
- **Resolved issues**:
  1. **R8 connection**: R8 from coll1 (not coll_dar) — two CE inversions (TR1+Darlington) make coll_dar non-inverting from base, so R8 from coll_dar would be positive DC feedback.
  2. **Transformer L_pri**: Marinair LO1166 spec: L_pri=1.9H (XL at 20Hz, no DC bias), DCR pri=14Ω sec=44Ω, ratio 1:1.68. Gives f_3dB=17.8Hz.
  3. **R_gain/C_gain**: Must be IN SERIES (R then C to ground), not parallel. Parallel C_gain shorts emitter at AC, killing feedback. Series gives correct 21 dB unbalanced gain at 25 dB setting.
  4. **C3=4700pF**: Across R1 (fb_node↔emit1), HF bypass at ~15kHz.
  5. **C4=100µF**: Per parts list (not 80µF).
- **Remaining**:
  - HF rolloff above 800Hz in analyze — may need more settle time or investigation
  - RV1 at midpoint gives 28mA bias (not 70mA). Need to trim RV1 for correct operating point.

### Stage 2: Preamp (BA283 AV)
- **Status**: COMPLETE — SPICE validated
- **Components**: 3× BC184C (TR4 CE, TR5 CE DC-coupled, TR6 EF buffer)
- **Topology**: R11(33K)+R12(47K) series collector load, R10(68K) inter-stage feedback from TR5 emitter to TR4 base
- **Gain**: 7 positions via Pin T boost resistor (+18 to +48 dB stage gain)
- **SPICE validation**: correlation 0.999986 (5 nines), RMS 0.53%, gain 28.3 dB matches ngspice
- **Netlist**: `circuits/unstable/neve-1073-preamp.cir`
- **Validation test**: `test_neve_1073_preamp_vs_spice` in spice_validation.rs
- **Tasks**:
  - [x] Trace and verify BA283 AV schematic (user+Schemer, junction dots vs crossovers)
  - [x] Write netlist: `circuits/unstable/neve-1073-preamp.cir`
  - [x] Validate DC operating point (Ic4=0.24mA, Vc4=4.3V, Ic5=1.86mA, Ic6=2.52mA)
  - [x] Validate gain at 28 dB position (28.26 dB melange, 28.25 dB ngspice)
  - [x] SPICE validation PASSED
- **Key fixes applied**:
  - VIN/R_src removed from melange circuit (double loading with G_in stamp)
  - DK internal node expansion skipped for DK path (ill-conditioning)
  - Junction cap stamping added to validation `run_melange_solver`

### Stage 3: EQ Section (passive LC + BA284 amps)
- **Status**: IN PROGRESS — LF/HF working, Presence topology resolved, peak not yet sharp
- **Components**: 3 inductors (T1295, T1530, T1280), switched C networks, pots, 2× BA284 amp sections
- **HPF (B182)**: DONE — 3rd-order C-L-C pi-section, 18 dB/oct, 4 positions validated
- **LF/HF (B205 + BA284 §2)**: WORKING — correct LF shelf shape
  - Opposing coupling, correct pot polarity, amp isolation (R15/R17 corrected to 120K per schematic)
- **Presence (BA211 + BA284 §3)**: Topology RESOLVED, amp produces signal, peak not yet sharp
  - BA284 §3 = **CE(TR7) → EF(TR8) → EF(TR9)** = 1 inversion = INVERTING discrete op-amp
  - All 12 resistors + critical caps fully traced from EX/10,284 schematic
  - C14(22µF/25V): between coll7 and base8 (NOT base8→GND — user confirmed from screenshot)
  - C15(100µF): emit7→GND (AC bypass, confirmed as direct simple connection)
  - Feedback R_fb1(220K) to base7 (summing node), not emit7
  - **Current result**: +15 dB at 50 Hz, rolling off to -17 dB at 8 kHz. Pot creates ±2 dB effect.
  - **Expected**: ~30 dB flat gain with ±10 dB presence peak at 3.2 kHz
  - **Possible issues**: Loop gain may be insufficient (C15 at emit7 makes Z_emit≈2.5Ω, feedback through 220K can barely develop voltage). Gain rolloff too steep — may need to verify open-loop gain vs closed-loop requirements.
- **Netlist**: `circuits/unstable/neve-1073-eq.cir` (HPF + LF bridge), `circuits/unstable/neve-1073-presence.cir` (discrete amp, in progress)
- **Tasks**:
  - [x] HPF: all 4 positions validated
  - [x] B205 LF/HF bridge: opposing coupling, shelf shape verified
  - [x] BA284 topology fully traced (all resistors accounted for)
  - [x] Coupled inductor switching implemented in melange
  - [x] C14 placement resolved (coll7↔base8, not base8→GND)
  - [x] 8 codegen tests passing (`neve_1073_eq_tests.rs`)
  - [ ] Presence peak: get sharp ±10 dB peak at 3.2 kHz (currently ±2 dB)
  - [ ] Gain profile: should be ~30 dB flat (currently +15 dB at LF, rolling off)
  - [ ] Test HF shelf (RV1 pot, 22nF caps)
  - [ ] Validate EQ curves against published 1073 response

### Stage 4: Integration (preamp + EQ + output)
- **Status**: NOT STARTED
- **Expected**: N~30-50, M=12 raw (6 BJTs in two BA283 stages), likely M=6-8 after FA
- **Concerns**:
  - Global NFB path (if any — need to verify from schematic)
  - Transformer coupling between stages
  - DC operating point with all stages loaded
- **Tasks**:
  - [ ] Combine netlists into `circuits/unstable/neve-1073.cir`
  - [ ] Verify DC OP convergence with full circuit
  - [ ] Check M dimension — if >16, apply FA reduction or raise MAX_M
  - [ ] Validate gain structure end-to-end
  - [ ] Frequency response sweep with EQ flat
  - [ ] Frequency response with each EQ band active
  - [ ] Transient simulation — check for oscillation, stability
  - [ ] Route to correct solver (DK vs nodal full-LU)
  - [ ] Performance benchmark (target: >4× realtime)

### Stage 5: Plugin Build
- **Status**: NOT STARTED
- **Tasks**:
  - [ ] `melange compile circuits/unstable/neve-1073.cir --format plugin`
  - [ ] Map pots: LF gain, presence gain, HF gain
  - [ ] Map switches: LF freq, presence freq, HPF freq, gain, EQ in/out
  - [ ] Oversampling selection (2× or 4×)
  - [ ] Test in DAW
  - [ ] SPICE validation against ngspice (if feasible at full circuit level)

## Transistor Models Needed

| Device | Type | Role | SPICE Model Source |
|--------|------|------|--------------------|
| BC184C | NPN | All small-signal stages | TBD — check Philips/NXP datasheet |
| 2N3055 | NPN | Output Darlington | TBD — Motorola/ON Semi |

Key parameters to source: IS, BF, BR, VAF, CJE, CJC, RB, RC, RE, TF, TR, NF

## Transformer Models Needed

| Neve P/N | Carnhill P/N | Role | Key Specs |
|----------|-------------|------|-----------|
| 10468 | VTB9045 | Mic input | 1:2, dual primary (300/1200 ohm) |
| 31267 | VTB9046 | Line input | 1:2, 600:2400 ohm |
| LO1166 | VTB1148 | Output | 4×150 ohm, DC-gapped, ~305mH primary |

## Reference Sources

- SI14 Lab: BA283 circuit analysis — https://si14lab.blogspot.com/2019/11/circuit-analysis-of-neve-1073-preamp-ba.html
- JLM Audio: BA283 variations — http://www.jlmaudio.com/neve_ba283.htm
- Neve 1073 schematics — https://archive.org/details/neve_1073_channel_amplifer_schematic_EH10023
- GroupDIY: 1073 inductor specs — https://groupdiy.com/threads/neve-1073-inductor-questions.2368/
- RN73 BOM (rackneve.com) — component values reference
- Carnhill transformer datasheets
- Nyan-1073-EQ (GitHub) — EQ section LTSpice model (gyrator-based, useful for EQ curve verification)

## Design Notes

- **No PNP transistors** in signal path — all-NPN simplifies modeling
- **Class A throughout** — all BJTs biased in forward-active, good for FA reduction
- **EQ is passive** — inductors + caps + switches, no active EQ stages (adds nodes but not M)
- **Gain switch** is 13-position rotary selecting feedback resistors — model as `.switch`
- **Output transformer DC-gapped** — carries ~70mA Class A bias, must model in DC OP
- **Presence EQ signature**: lower 3 freqs switch both L and C (constant Q), upper 3 switch only C (rising Q with frequency) — this is a key sonic characteristic
