# Melange Status Reference

Quick-reference for AI agents. For math details see other aidocs. For architecture see CLAUDE.md.

## SPICE Validation Results

Tests in `crates/melange-validate/tests/spice_validation.rs`. Run with
`cargo test -p melange-validate --test spice_validation -- --include-ignored --nocapture`
(requires `ngspice` on PATH). Each test calls `run_melange_codegen()`, which
generates Rust code from the netlist via the codegen pipeline, compiles it
with `rustc -O`, runs it as a subprocess, and compares the output samples
against ngspice with `.OPTIONS INTERP` for sample alignment.

**Last re-baselined: 2026-04-08** (full sweep, 19/19 passing in 4.78 s).

| Circuit | Correlation | Norm RMS | THD Err | Notes |
|---------|-------------|----------|---------|-------|
| RC lowpass | 0.99999995 | 0.030% | 0.00 dB | Linear reference, SNR 70.4 dB |
| Diode clipper | 0.99999990 | 0.071% | 0.00 dB | 1N4148 antiparallel, SNR 63.0 dB |
| Antiparallel diodes (2D) | 0.99999985 | 0.058% | 0.03 dB | Two-diode soft clipper, SNR 64.8 dB |
| BJT common-emitter | 0.99970213 | 5.63% | — | BC547 NPN, gain ~210×, SNR 23.8 dB |
| Op-amp inverting | 1.00000000 | 0.000% | 0.00 dB | Boyle macromodel, SNR 130.7 dB |
| JFET common-source | 0.99940687 | 3.47% | 4.70 dB | N-channel, ngspice BETA→IDSS |
| MOSFET common-source | 0.99999997 | 0.029% | — | N-channel Level 1, SNR 70.8 dB |
| Tube Screamer (TS808) | 0.99999064 | 0.43% | 0.04 dB | Op-amp + 1N4148 clipping, SNR 47.2 dB |
| Tube Screamer (wiper) | 0.99998416 | 0.59% | 0.04 dB | TS808 + 100K wiper at pos=0.85 |
| Wurli preamp | 0.99999998 | 0.014% | — | 2× 2N5089, M=5, gain 39.9×, SNR 69.2 dB |
| Neve 1073 output (BA283 AM) | 0.99609584 | 14.4% | — | 3 BJT + LO1166 transformer, gain 3.7×, marginal |

Additional smoke tests (no SPICE comparison):

| Test | Correlation | RMS Error |
|---|---|---|
| Diode clipper silence→signal | 0.99999933 | 5.84e-4 |
| RC lowpass chirp (100Hz→10kHz, 100ms) | 0.99994732 | 3.36e-3 |
| RC lowpass step response (500Hz square) | 0.99968801 | 2.24e-2 |

Tests excluded from this baseline: `test_klon_centaur_vs_spice` (under
investigation in a parallel session), `test_neve_1073_preamp_vs_spice`
(unstable, hung the runner). Re-include them via `--include-ignored` once
the underlying circuits stabilize.

## Device Model Features (All Implemented 2026-03-18)

- **Junction capacitances**: CCG/CGP/CCP (tube), CJE/CJC (BJT), CGS/CGD (JFET/MOSFET), CJO (diode)
- **Parasitic resistances**: diode RS, BJT RB/RC/RE, JFET/MOSFET RD/RS, tube RGI
- **BJT extras**: NF/ISE/NE (emission/leakage), Gummel-Poon (VAF/VAR/IKF/IKR), self-heating (RTH/CTH/XTI/EG/TAMB) — disabled by default (RTH=∞)
- **Diode**: BV/IBV (Zener breakdown)
- **MOSFET**: GAMMA/PHI (body effect)
- **Op-amp**: Boyle macromodel with GBW dominant pole, rail-clamping modes (`auto/none/hard/active-set/boyle-diodes`, see `--opamp-rail-mode` CLI flag), and optional slew-rate limiting via `.model OA(SR=13)` in V/μs (per-sample `|Δv_out| ≤ SR·dt` clamp, all 3 codegen paths, default `SR=∞` → zero code emitted)
- **BJT Gummel-Poon**: matches ngspice `bjtload.c` line-for-line (q2 uses `cbe/IKF + cbc/IKR` with `cbe = IS*(exp(Vbe/(NF*VT))-1)`; Ib ideal forward NOT divided by qb)
- **Pentode / beam tetrode**: Three screen-current equation families selected per-slot by a `ScreenForm` discriminator on `TubeParams`. **Rational** (Reefman Derk §4.4, `1/(1+β·Vp)`, 9 params) for true pentodes; **Exponential** (Reefman DerkE §4.5, `exp(-(β·Vp)^{3/2})`, 9 params) for beam tetrodes with critical-compensation knees; **Classical** (Norman Koren 1996 / Cohen-Hélie 2010, `arctan(Vpk/Kvb)` + Vp-independent screen, 6 params) as a fallback for tubes without Reefman fits. Optionally blended via Reefman §5 two-section Koren (variable-mu) for remote-cutoff tubes. Catalog: EL84/6BQ5, EL34/6CA7, EF86/6267 (Rational, `-P` suffix); 6L6GC/5881, 6V6GT (Exponential, `-T` suffix); KT88, 6550 (Classical, no suffix); 6K7, EF89 (variable-mu Rational/Exponential). New element prefix `P` (`P n_plate n_grid n_cathode n_screen [n_suppressor] model`) and `VP` model token. Phase 1b adds grid-off FA reduction; phase 1d adds datasheet-refit entries for 6386/6BA6/6BC8 (varimu compressor tubes).
- **Not implemented**: temperature coefficients on resistors (TC1/TC2), noise models, 6386/6BA6/6BC8 datasheet fits for varimu compressors (phase 1d deferred)
- **Known model limitations**:
  - Diode BV: exponential reverse breakdown (matches codegen template), evaluated in both codegen and DC OP solver (FIXED 2026-04-15)
  - DC OP diode Gmin: 1e-12 S minimum junction conductance added to prevent zero Jacobian entries at reverse bias (FIXED 2026-04-15)
  - DC OP op-amp AOL capped at 1000 to prevent multi-equilibrium NR instability in precision rectifier circuits (FIXED 2026-04-15)
  - DC OP failed convergence: low-rate warmup (200 Hz × 1000 samples = 5s circuit time) charges coupling caps before transient NR. Settled state cached for `reset()`. 4kbuscomp: BE fallback <1%, stable at all amplitudes. (ADDED 2026-04-16)
  - BJT GP Q1: singularity guard at `q1_denom <= 0` (physically near Early voltage limit)
  - Tube Koren: no space-charge, no transit-time effects
  - JFET/MOSFET subthreshold: hardcoded 2×VT slope (real devices: 60-120 mV/decade)
  - VCA noise_floor field exists but unused
  - Precision rectifier transient: VCCS back-substitution contamination at cap-only nodes downstream of high-AOL op-amps. Fixed via selective Gm cap on op-amps matching Rule D' (n_plus on non-zero DC rail AND diode connects output→inverting input through R-only path). 4kbuscomp `max_abs_v_prev`: 1.18B → 15V. User override via `.model OA(AOL_TRANSIENT_CAP=N)`. Klon and other working circuits unaffected (Rule D' correctly excludes them). (FIXED 2026-04-16)

## Codegen Device Support

The runtime solvers (`CircuitSolver`, `NodalSolver`, `DeviceEntry`) have been removed.
All device handling lives in the codegen pipeline. Only `LinearSolver` (M=0 linear-only)
remains in the solver crate as a fallback for purely linear circuits.

| Device | NR Dim | Model |
|--------|--------|-------|
| Diode | 1D | Shockley + RS + BV |
| BJT | 2D (Vbe→Ic, Vbc→Ib) | Gummel-Poon / Ebers-Moll |
| BJT (forward-active flagged) | 1D (Vbe→Ic, Ib = Ic/βF) | Auto-detected at DC OP |
| BJT (linearized) | 0D (removed from NR) | Small-signal `g`s stamped into G after DC OP |
| JFET | 2D | Shichman-Hodges |
| MOSFET | 2D | Level 1 SPICE |
| Tube (triode) | 2D (Vgk→Ip, Vpk→Ig) | Koren + Leach |
| Tube (pentode) | 3D (Vgk→Ip, Vpk→Ig2, Vg2k→Ig1) | Reefman Derk §4.4 / DerkE §4.5 / Classical + Leach |
| Tube (pentode, grid-off) | 2D (Vgk→Ip, Vpk→Ig2, Vg2k frozen) | Auto-detected at DC-OP when Vgk<cutoff; `--tube-grid-fa` override |
| VCA | 2D (Vsig, Vctrl) | THAT 2180 exponential |
| Op-amp | Linear (no NR dim) | Boyle VCCS + GBW pole + rail clamp |

M=1 direct, M=2 Cramer's, M=3..16 Gaussian elimination with partial pivoting.

## Codegen Solver Routing (Updated 2026-03-23)

| Path | When Selected | Cost | Notes |
|------|--------------|------|-------|
| DK Schur | M<10, ≤1 xfmr, K well-conditioned | O(N²+M³)/sample | 100-600× realtime |
| Nodal Schur | M≥10 or 2+ xfmr, K well-conditioned | O(N²+M³)/sample | Medium-complexity circuits |
| Nodal full LU | K≈0 (VCA), positive K diag, K ill-cond | O(N³)/sample | Matches runtime exactly |

K≈0 detection: max|K| < 1e-6 with M > 0.

## Circuit Library Status

Circuits have been migrated to the [melange-audio/circuits](https://github.com/melange-audio/circuits) repo.
All circuits are in `unstable/` until the user manually tests and approves promotion.

The compiler validation status of circuits known to exercise specific solver paths:

| Topology class | N | M | Solver | Performance | Notes |
|----------------|---|---|--------|-------------|-------|
| Linear RC | 2 | 0 | Linear | trivial | Smoke test |
| 2-stage BJT preamp | 11 | 3-5 | DK | fast | FA detection, 2N5089 Ebers-Moll |
| 2-stage triode preamp | 13 | 4 | DK | fast | 2× 12AX7, pot + switch |
| 4-tube passive EQ + 2 xfmrs | 41 | 8 | Nodal full LU | ~11× | Chord + cross-timestep + sparse LU |
| 8-BJT Class AB power amp | 20 | 9-16 | DK/Nodal | 0.4× / 0.04× | Parasitic R, FA detection |
| 4-opamp + diode clipper | 44 | 10 | Nodal full LU (auto) | — | ActiveSetBe auto for clean clipping; BoyleDiodes diverges at heavy clip |
| Op-amp overdrive + diodes | — | — | DK | — | TS808-class clipping |
| VCA compressor + sidechain | 21 | 3 | Nodal full LU | ~42× | Current-mode VCA, K≈0 |
| Pentode single stage | — | 2-3 | DK | fast | Grid-off reduces M=3→2 |
| Push-pull pentode amp + OT | — | — | Nodal | — | Transformer forces nodal path |
| Variable-mu pentode | — | 3 | DK | fast | M=3, no grid-off reduction |

Only circuits using standard SPICE models (D, NPN/PNP, NJF/PJF, NM/PM) can be validated
against ngspice. Circuits with melange-extended models (OA, VCA, VP, triode) use
`melange compile`/`analyze`/`simulate` for validation, not ngspice.

Promotion to `stable/` requires user sign-off after a DAW listening test.
SPICE correlation and successful compilation are necessary but not sufficient.

## Pultec Schematic Data (Verified 2026-03-16)

Source: Sowter DWG E-72,658-2 + Peerless/Triad winding data.

- **HS-29**: 1:2 step-up, 37H, true push-pull (pin 5→grid1, pin 8→grid2), CT→43K+270pF
- **S-217-D**: 220H primary (30Hz), 71-turn tertiary (0.447H), 20pF plate cap
- **Feedback winding**: 12AX7 pin 3→360Ω→S-217-D pin 3; pin 8→360Ω→pin 5
- **Cathode**: 820Ω to GROUND (not through transformer)
- Gain budget: +25 dB amp - 23 dB EQ = +2 dB net
