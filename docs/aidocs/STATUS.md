# Melange Status Reference

Quick-reference for AI agents. For math details see other aidocs. For architecture see CLAUDE.md.

## SPICE Validation Results

Tests in `crates/melange-validate/tests/spice_validation.rs`. Uses ngspice batch mode + `.OPTIONS INTERP`.

| Circuit | Correlation | RMS Error | Notes |
|---------|------------|-----------|-------|
| RC lowpass | 0.99999995 | 0.03% | Linear reference |
| Diode clipper | 0.999 | 14% | Needs caps for DK stability |
| Antiparallel diodes | 0.998 | 8.9% | |
| BJT common-emitter | 0.965 | 35% | `bjt_config()` relaxed tolerances |
| Op-amp inverting | 1.0 | ~0% | VCCS+Rout model, SNR 130dB |
| JFET common-source | 0.999 | 3.5% | ngspice BETA→IDSS (BETA*VP²) |
| MOSFET common-source | 0.99999997 | 0.03% | Skip THD (linear region) |
| Tweed preamp | 0.989 | ~18% | After junction caps (was 0.962) |
| Wurli preamp | 0.999997 | 3.2% | 6 nines correlation |

## Device Model Features (All Implemented 2026-03-18)

- **Junction capacitances**: CCG/CGP/CCP (tube), CJE/CJC (BJT), CGS/CGD (JFET/MOSFET), CJO (diode)
- **Parasitic resistances**: diode RS, BJT RB/RC/RE, JFET/MOSFET RD/RS, tube RGI
- **BJT extras**: NF/ISE/NE (emission/leakage), Gummel-Poon (VAF/VAR/IKF/IKR)
- **Diode**: BV/IBV (Zener breakdown)
- **MOSFET**: GAMMA/PHI (body effect)
- **Not implemented**: BJT self-heating (RTH/CTH), pentode support

## Codegen Device Support

| Device | NR Dim | Model | Codegen | Runtime |
|--------|--------|-------|---------|---------|
| Diode | 1D | Shockley + RS + BV | ✓ | ✓ |
| BJT | 2D | Gummel-Poon / Ebers-Moll | ✓ | ✓ |
| JFET | 2D | Shichman-Hodges | ✓ | ✓ |
| MOSFET | 2D | Level 1 SPICE | ✓ | ✓ |
| Tube | 2D | Koren + Leach | ✓ | ✓ |
| VCA | 2D | THAT 2180 exponential | ✓ | ✓ |
| Op-amp | Linear | Boyle VCCS + GBW pole | ✓ | ✓ |

M=1 direct, M=2 Cramer's, M=3..16 Gaussian elimination with partial pivoting.

## Codegen Solver Routing (Updated 2026-03-23)

| Path | When Selected | Cost | Notes |
|------|--------------|------|-------|
| DK Schur | M<10, ≤1 xfmr, K well-conditioned | O(N²+M³)/sample | 100-600× realtime |
| Nodal Schur | M≥10 or 2+ xfmr, K well-conditioned | O(N²+M³)/sample | Pultec: 0.6× RT |
| Nodal full LU | K≈0 (VCA), positive K diag, K ill-cond | O(N³)/sample | Matches runtime exactly |

K≈0 detection: max|K| < 1e-6 with M > 0.

## Validated Circuits

| Circuit | N | M | Solver | Performance | Notes |
|---------|---|---|--------|-------------|-------|
| tweed-preamp | 13 | 4 | DK | fast | 2× 12AX7, 1 pot, 1 switch |
| wurli-preamp | 11 | 3-5 | DK | fast | 2 BJTs + 1 diode, FA detection |
| wurli-power-amp | 20 | 9-16 | DK/Nodal | 0.4× / 0.04× | Class AB, 8 BJTs |
| pultec-eq | 41 | 8 | Nodal full LU | 0.6× | 4 tubes, 2 xfmrs, 7 pots, 3 switches. Freq response verified 2026-03-23. |
| ssl-bus-compressor | 28 | 2 | Nodal full LU | — | 4 op-amps, 1 VCA, 2 diodes |

## Pultec Schematic Data (Verified 2026-03-16)

Source: Sowter DWG E-72,658-2 + Peerless/Triad winding data.

- **HS-29**: 1:2 step-up, 37H, true push-pull (pin 5→grid1, pin 8→grid2), CT→43K+270pF
- **S-217-D**: 220H primary (30Hz), 71-turn tertiary (0.447H), 20pF plate cap
- **Feedback winding**: 12AX7 pin 3→360Ω→S-217-D pin 3; pin 8→360Ω→pin 5
- **Cathode**: 820Ω to GROUND (not through transformer)
- Gain budget: +25 dB amp - 23 dB EQ = +2 dB net
