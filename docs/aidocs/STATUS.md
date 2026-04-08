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
- **Op-amp**: Boyle macromodel with GBW dominant pole and rail-clamping modes (`auto/none/hard/active-set/boyle-diodes`, see `--opamp-rail-mode` CLI flag)
- **Not implemented**: pentode support, temperature coefficients on resistors (TC1/TC2), noise models
- **Known model limitations**:
  - Diode BV: hard clamp reverse breakdown (no smooth Zener knee)
  - BJT GP Q1: singularity guard at `q1_denom <= 0` (physically near Early voltage limit)
  - Tube Koren: no space-charge, no transit-time effects
  - JFET/MOSFET subthreshold: hardcoded 2×VT slope (real devices: 60-120 mV/decade)
  - Op-amp: no slew-rate limiting (GBW pole only)
  - VCA noise_floor field exists but unused

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
| Tube | 2D (Vgk→Ip, Vpk→Ig) | Koren + Leach |
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

Promotion criteria:
- **`circuits/stable/`** — passes SPICE validation AND user has manually
  verified the audio output sounds right in a DAW. Promotion requires
  user sign-off, not just numerical correlation.
- **`circuits/testing/`** — compiles, runs, may pass SPICE validation, but
  not yet user-verified for audio quality. Use at your own risk.
- **`circuits/unstable/`** — known issues; works in some configurations
  only, or under active investigation. Do not ship.

### Stable circuits (user-verified)

| Circuit | N | M | Solver | Performance | Notes |
|---------|---|---|--------|-------------|-------|
| rc-lowpass | 2 | 0 | Linear | trivial | Linear-reference smoke test |
| tweed-preamp | 13 | 4 | DK | fast | 2× 12AX7, 1 pot, 1 switch |
| wurli-preamp | 11 | 3-5 | DK | fast | 2 BJTs + 1 diode, FA detection |
| pultec-eq | 41 | 8 | Nodal full LU | ~11× | 4 tubes, 2 xfmrs, 7 pots, 3 switches. Chord + cross-timestep + sparse LU. All EQ curves verified 2026-03-23. |

### In testing (compiles, not yet user-verified)

| Circuit | N | M | Solver | Performance | Notes |
|---------|---|---|--------|-------------|-------|
| wurli-power-amp | 20 | 9-16 | DK/Nodal | 0.4× / 0.04× | Class AB, 8 BJTs |
| klon-centaur | 44 | 10 | Nodal full LU (auto) | — | 4× TL072 (charge-pump rails 9V & 18V/-9V) + 2× Ge clipping diodes, 3 pots (gain dual-gang). Auto picks `ActiveSetBe` (commit `016ac6c`) for clean clipping 0.05–0.20 V; above 220 mV the trap+pin Nyquist limit cycle returns. **`BoyleDiodes`** (commits `5544c8a` + `39397d1`) works for clean clipping up to amp ≈ 0.05 V (matches ActiveSetBe within 25 mV) but **diverges at amp ≥ 0.07 V** due to bistable Newton oscillation at the catch-diode knee — see `DEBUGGING.md` "Op-amp BoyleDiodes Failure Signatures" and the agent-memory note `task_12_bistable_oscillation_finding.md`. Real fix needs Anderson acceleration / trust-region NR. |
| tube-screamer | — | — | DK | — | TS808 clipper |
| mordor-screamer | — | — | DK | — | TS-style variant |
| tube-preamp | — | — | DK | — | Generic tube preamp |
| vcr-audio-alc | 21 | 3 | Nodal full LU | ~42× | VHS audio ALC compressor (feedforward VCA + sidechain) |
| wurli-tremolo | — | — | — | — | LDR tremolo stage |

### Unstable (known issues, under investigation)

| Circuit | N | M | Solver | Performance | Notes |
|---------|---|---|--------|-------------|-------|
| ssl-bus-compressor | 28 | 2 | Nodal full LU | — | 4 op-amps, 1 VCA, 2 diodes. Audio path codegen matches runtime, full plugin not stable. |
| neve-1073-preamp | — | — | — | — | BA283 AV stage |
| neve-1073-output | — | — | — | — | BA283 AM stage |
| neve-1073-eq | — | — | — | — | EQ section in progress |
| neve-1073-presence | — | — | — | — | Presence stage |
| big-muff | — | — | DK | — | EHX classic, fuzz topology |
| fuzz-face | — | — | DK | — | Arbiter Fuzz Face Ge transistor pair |

Promotion to `stable/` requires user sign-off after a DAW listening test.
SPICE correlation and successful compilation are necessary but not sufficient.

## Pultec Schematic Data (Verified 2026-03-16)

Source: Sowter DWG E-72,658-2 + Peerless/Triad winding data.

- **HS-29**: 1:2 step-up, 37H, true push-pull (pin 5→grid1, pin 8→grid2), CT→43K+270pF
- **S-217-D**: 220H primary (30Hz), 71-turn tertiary (0.447H), 20pF plate cap
- **Feedback winding**: 12AX7 pin 3→360Ω→S-217-D pin 3; pin 8→360Ω→pin 5
- **Cathode**: 820Ω to GROUND (not through transformer)
- Gain budget: +25 dB amp - 23 dB EQ = +2 dB net
