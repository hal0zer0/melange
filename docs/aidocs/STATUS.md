# Melange Status Reference

Quick-reference for AI agents. For math details see other aidocs. For architecture see CLAUDE.md.

> **2026-07-18 accuracy campaign (commits `3e246cb`, `5159b8c`, `fde289a`, `b421358`, `8056f95` — all six review chunks complete):** a
> full-codebase accuracy review fixed, among others: op-amp VCCS polarity (was
> inverted — clipping/comparator behavior changed), Koren triode ×2 factor (triode
> stages now run at datasheet current/gm — Pultec/SeriesOfTubes gain staging
> shifted), oversampling decimator + half-band tables (2x/4x plugins gain flat HF
> passband and real (−87 dB) alias rejection), vendor-verbatim BJT/diode catalog
> cards, FET reverse quadrant/depletion NMOS, DC-OP device parity + pnjlim
> normalization, and 16 parser strictness fixes (femto suffix, node case-folding,
> gnd alias, model-type validation). All SPICE validations re-pass. Shipped-circuit
> sound WILL shift (accuracy-correct); nothing promoted without listening. See
> DEBUGGING.md "Historical Failure Signatures" (2026-07-18 rows) and the
> codebase-review-2026-07 memory for the full inventory.

## SPICE Validation Results

Tests in `crates/melange-validate/tests/spice_validation.rs`. Run with
`cargo test -p melange-validate --test spice_validation -- --include-ignored --nocapture`
(requires `ngspice` on PATH). Each test calls `run_melange_codegen()`, which
generates Rust code from the netlist via the codegen pipeline, compiles it
with `rustc -O`, runs it as a subprocess, and compares the output samples
against ngspice with `.OPTIONS INTERP` for sample alignment.

**Last re-baselined: 2026-07-18** (HEAD `b421358`, ngspice-42, single-VIN
Thevenin-PWL decks, ngspice `reltol=1e-4`). The suite is now 22 tests (18
gated on ngspice + 4 unit/harness tests). Every gated test carries a cited
measured value in its gate comment (`crates/melange-validate/tests/spice_validation.rs`);
the table below is derived from those comments. Gains, settle windows, and
per-metric gates were all tightened in the same pass.

| Circuit | Correlation | Norm RMS | Peak err | Notes |
|---------|-------------|----------|----------|-------|
| RC lowpass (1 kHz sine) | > 0.99999 | < 0.05% | — | Linear reference (strict-linear gate) |
| RC lowpass step (500 Hz square) | 0.99999924 | 0.132% | 6.9e-3 V | Onset matched via `input[0]=0` |
| RC lowpass chirp (100 Hz→10 kHz) | > 0.9999 | < 2% | — | Trapezoidal HF-warping gate (no cited point measurement) |
| Op-amp inverting (gain −10) | > 0.99999 | < 0.05% | — | VCCS model, M=0 linear |
| Diode clipper | 0.99999991 | 0.069% | 1.6e-3 V | 1N4148 antiparallel, THD err 0.00 dB |
| Antiparallel diodes (2D) | 0.99999983 | 0.059% | 1.2e-3 V | THD err 0.03 dB |
| Diode clipper silence→signal | 0.99999934 | 0.125% | 1.43e-2 V | NR startup transient |
| BJT common-emitter | 0.99964880 | 4.26% | 0.164 V | BC547, gain ratio 1.024, 3 ms settle |
| JFET common-source | 0.99940687 | 3.47% | 4.6e-6 V | THD err 4.70 dB |
| MOSFET common-source | 0.99999997 | 0.029% | 2.7e-6 V | Level 1, small-signal |
| Tube Screamer (TS808) | 0.99999032 | 0.442% | 9.9e-3 V | Op-amp + 1N4148, THD err 0.04 dB |
| Tube Screamer (wiper, pos=0.85) | 0.99838696 | 5.71% | — | Volume divider + simplified tone; THD err 0.11 dB |
| Wurli preamp | 0.99999734 | 0.235% | 1.28e-3 V | 2× 2N5089, M=5, gain ratio 1.0006, 10 ms settle |
| Neve 1073 output (BA283 AM) | 0.99999952 | 0.107% | 1.06e-4 V | 3 BJT + LO1166 xfmr, gain 6.7×, ratio 1.0000, 10 ms settle |
| Neve 1073 preamp (BA283 AV) | 1.00000000 | 0.0346% | 1.12e-4 V | 3× BC184C, gain 26.0×, ratio 0.9996, 64 ms settle |
| Pot static off-nominal | 0.99999995 | 0.0374% | 4.06e-3 V | `.pot` rebuild vs fixed-R deck, THD err 0.01 dB |
| Pot modulation (5 kHz R sweep) | 0.99991763 | 1.28% | 3.93e-2 V | vs native ngspice B-source; residual is per-sample ZOH of R(t) |

Two rows corrected the largest stale figures from the 2026-04-08 baseline:

- **Neve 1073 output** was recorded at corr 0.9961 / rms 14.4% ("marginal").
  That predated the deck double-load fix — the deck baked in a `VIN in_src` +
  `R_src in_src in` Thevenin pair that escaped both the harness VIN strip and
  the Thevenin inject (n+ was `in_src`, not `in`), leaving a second 1-ohm
  shunt at the input node on both sides. Removing it doubled the drive
  (gain 3.4× → 6.7×) at corr 0.99999952.
- **Neve 1073 preamp** was recorded at corr 0.99999 / rms 0.53%. Most of that
  rms was a harness artifact: the melange output was DC-blocked twice (once
  inside the generated code, once in the test) while the SPICE side was
  blocked once. Removing the second application dropped rms to 0.0346% at
  corr 1.00000000.

The pot static / pot modulation rows are new tests (first armed 2026-07-18).
The static off-nominal test sits at the 0.037% floor — the same floor as the
nominal-position diode tests — which confirms the 1.28% modulation residual
is R(t) zero-order-hold discretization at 5 kHz mod / 48 kHz fs, not the
`.pot` rebuild mechanism.

## Device Model Features (All Implemented 2026-03-18)

- **Junction capacitances**: CCG/CGP/CCP (tube), CJE/CJC (BJT), CGS/CGD (JFET/MOSFET), CJO (diode)
- **Parasitic resistances**: diode RS, BJT RB/RC/RE, JFET/MOSFET RD/RS, tube RGI
- **BJT extras**: NF/ISE/NE (emission/leakage), Gummel-Poon (VAF/VAR/IKF/IKR), self-heating (RTH/CTH/XTI/EG/TAMB) — disabled by default (RTH=∞)
- **Diode**: BV/IBV (Zener breakdown), self-heating (RTH/CTH/XTI/EG/TAMB) — disabled by default (RTH=∞); analytic-validated 2026-04-21 against `Tj_ss = TAMB + P·Rth` and exponential τ = RTH·CTH; ngspice parity not applicable (SPICE3f5 BJT/diode silently drop RTH)
- **MOSFET**: GAMMA/PHI (body effect)
- **Op-amp**: Boyle macromodel with GBW dominant pole, rail-clamping modes (`auto/none/hard/active-set/boyle-diodes`, see `--opamp-rail-mode` CLI flag), and optional slew-rate limiting via `.model OA(SR=13)` in V/μs (per-sample `|Δv_out| ≤ SR·dt` clamp, all 3 codegen paths, default `SR=∞` → zero code emitted)
- **BJT Gummel-Poon**: matches ngspice `bjtload.c` line-for-line (q2 uses `cbe/IKF + cbc/IKR` with `cbe = IS*(exp(Vbe/(NF*VT))-1)`; Ib ideal forward NOT divided by qb)
- **Pentode / beam tetrode**: Three screen-current equation families selected per-slot by a `ScreenForm` discriminator on `TubeParams`. **Rational** (Reefman Derk §4.4, `1/(1+β·Vp)`, 9 params) for true pentodes; **Exponential** (Reefman DerkE §4.5, `exp(-(β·Vp)^{3/2})`, 9 params) for beam tetrodes with critical-compensation knees; **Classical** (Norman Koren 1996 / Cohen-Hélie 2010, `arctan(Vpk/Kvb)` + Vp-independent screen, 6 params) as a fallback for tubes without Reefman fits. Optionally blended via Reefman §5 two-section Koren (variable-mu) for remote-cutoff tubes. Catalog: EL84/6BQ5, EL34/6CA7, EF86/6267 (Rational, `-P` suffix); 6L6GC/5881, 6V6GT (Exponential, `-T` suffix); KT88, 6550 (Classical, no suffix); 6K7, EF89 (variable-mu Rational/Exponential). New element prefix `P` (`P n_plate n_grid n_cathode n_screen [n_suppressor] model`) and `VP` model token. Phase 1b adds grid-off FA reduction; phase 1d adds datasheet-refit entries for 6386/6BA6/6BC8 (varimu compressor tubes).
- **Not implemented**: temperature coefficients on resistors (TC1/TC2), op-amp `EN_FC`/`IN_FC` 1/f corner (Phase 4 is white-band only in v1), DK-path BJT parasitic-R (rbb′) thermal noise (nodal only), diode `RS` / tube `RGI` thermal noise, tube microphonics (Phase 6), 6386/6BA6/6BC8 datasheet fits for varimu compressors (phase 1d deferred). All five noise phases (thermal, shot, junction+resistor flicker, op-amp en/in, pentode partition) ARE shipped — see "Circuit Noise" under Feature Inventory.
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

## Feature Inventory

### Core Pipeline
- MNA stamping: R, C, L, V/I sources, diodes, BJTs, JFETs, MOSFETs, tubes, op-amps, VCAs
- DK kernel with proper trapezoidal discretization; NR solver 1D / 2D / M-dimensional (M≤16)
- Codegen for diode, BJT, JFET, MOSFET, tube/triode/pentode (Gaussian elimination M=3..16)
- Per-device `.model` params (heterogeneous models supported per device)
- Parasitic cap auto-insertion (10pF junction caps) when nonlinear circuit has no caps
- Sparsity-aware emission (systematic zero-skipping in A_neg, N_v, K, S*N_i)
- Runtime sample rate: `set_sample_rate()` recomputes matrices from G+C

### DC Operating Point
- LU with partial pivoting, logarithmic junction-aware voltage limiting, source + Gmin stepping
- Internal nodes for parasitic BJTs (basePrime/colPrime/emitPrime, ngspice-style)
- Op-amp seeding + per-iteration rail clamp + AOL=1000 cap in DC G (precision rectifiers)
- Diode BV/IBV breakdown + device-level Gmin (1e-12 S) — physical reverse bias
- Low-rate DC warmup (200 Hz × 1000 samples) for failed-DC-OP circuits; settled state cached
- `DC_NL_I` constant initializes `i_nl_prev` in generated code

### Device Models
- **BJT**: Gummel-Poon (VAF/VAR/IKF/IKR, CJE/CJC, NF/ISE/NE) matching ngspice `bjtload.c` line-for-line; Ebers-Moll fallback; self-heating (RTH/CTH/TAMB); RB/RC/RE parasitic R
- **JFET/MOSFET**: 2D Shichman-Hodges / Level 1; CGS/CGD junction caps; RD/RS parasitic R; MOSFET body effect (GAMMA/PHI)
- **Diode**: Shockley + RS + CJO + BV/IBV Zener; optional self-heating (RTH/CTH/XTI/EG/TAMB) using the same quasi-static electrothermal model as BJT, with `IS(T) = IS_nom·(Tj/Tnom)^XTI·exp(EG/VT_nom·(1−Tnom/Tj))` and `N·VT(T) = (N·VT)_nom·(Tj/Tnom)`. Pipe-shouter (TS-808) uses RTH=500 CTH=2e-4 on the 1N4148 clippers; sad-bastard uses RTH=1200 CTH=1e-4 EG=0.67 on the 1N34A Ge clippers. Dead code when RTH=∞ (default).
- **Tube (triode)**: Koren + Leach grid current, early-effect lambda, CCG/CGP/CCP junction caps, RGI grid-stop
- **Tube (pentode)**: 3 screen-current equation families — Rational (Reefman §4.4), Exponential (DerkE §4.5), Classical Koren. `--tube-grid-fa {auto,on,off}` reduces 3D→2D when Vgk<cutoff
- **Op-amp**: Boyle macromodel, VCC/VEE asymmetric rails, optional `SR=` slew-rate limiting (V/μs), rail modes `auto/none/hard/active-set/active-set-be/boyle-diodes`, `AOL_TRANSIENT_CAP` override
- **VCA**: THAT 2180 / DBX 2150 current-mode exponential gain with gain-dependent THD

### Unit Variation (2026-04-21)
- `.seed <u64>`: sets master RNG seed (default 0). Shared by `.mismatch` and `.tolerance`.
- `.mismatch D IS=tol N=tol RS=tol` / `.mismatch Q IS=tol BF=tol BR=tol`: per-device parameter jitter, baked at codegen. Two diodes on the same `.model` land at distinct `DEVICE_N_IS` constants — the thing that makes antiparallel clippers and push-pull pairs audibly asymmetric. `J` / `M` / `T` parse but aren't yet IR-wired.
- `.tolerance R=0.01 C=0.02 L=0.005`: fixed-passive value jitter, applied at end of `Netlist::parse()`. Skips components under `.pot`/`.wiper`/`.switch`/`.runtime R` control so UI-driven mappings stay intact.
- Deterministic: `FNV(seed, class_tag, name) → SplitMix64 → [-1, 1]`. Same seed always produces the same unit personality. Absent directives ⇒ byte-identical output (regression-guarded).
- Full reference: [UNIT_VARIATION.md](UNIT_VARIATION.md).

### Dynamic Parameters
- `.pot R min max [default] [label]`: per-block O(N³) rebuild on change; per-sample smoother via `.smoothed.next()`; reseed-free setter — use `recompute_dc_op()` for preset-recall NR refresh (DK only; nodal falls back to NR catch-up)
- `.wiper R_cw R_ccw total [pos] [label]`: two-resistor wiper; position-0..1 UI param
- `.switch R/C/L pos0 pos1 ...`: up to 16 switches; G/C/L stamped at pos-0 baseline (not static) so initial state is self-consistent
- `.gang "Label" m1 m2 ...`: links multiple `.pot`/`.wiper` members under one parameter; `!` prefix inverts; `.runtime R` members rejected at parse time (drive multiple setters from one plugin envelope instead)
- `.runtime V as <field>`: binds existing VS to `pub <field>: f64` on `CircuitState`; host writes per sample, RHS stamp uses `VSOURCE_<NAME>_RHS_ROW`
- `.runtime R min max as <field>` (2026-04-19): audio-rate resistor modulation; emits `set_runtime_R_<field>(r)` WITHOUT the `.pot R` 20% DC-OP warm re-init (that snap clicks at envelope-follower rates); emits `RUNTIME_R_<FIELD>_MIN/_MAX/_NOMINAL` consts + `<field>()` getter; no nih-plug knob. Unblocks Latinum §5(b) envelope-linked bias

### Behavioral Sources (B) — nodal codegen shipped, oracle-tested
- `B... V={expr}` / `I={expr}` arbitrary-expression sources on the **nodal** path: exprs over node voltages, `time`, `ddt` (backward diff), `idt`, and `.param`/`.runtime` params. Oracle-validated in `behavioral_source_tests.rs` (current/voltage multiplier, tanh clipper, ddt-of-time, idt-ramp, slew-opamp compile).
- **Not yet**: branch-current references in exprs (errors loudly), and the DK path (B-source circuits route nodal or error). Full surface: [BEHAVIORAL_SOURCES.md](BEHAVIORAL_SOURCES.md).

### Circuit Noise (Phases 1–5, opt-in via `--noise {thermal|shot|full}`)
- **Thermal (Phase 1)**: Johnson-Nyquist on every fixed R + every dynamic R (`.pot`/`.wiper`/`.runtime R`/`.switch` R). Per-sample two-draw stamp `i_n[n] = w[n] + w[n-1]` with `w[n] = (scale/2)·sqrt(1/R)·g` — zeros the Nyquist bin so resistor-only nodes don't accumulate (FIXED 2026-04-24 `a5dff8c`). Shipped on both DK and nodal codegen paths via the shared `build_noise_emission()`.
- **Shot (Phase 2)**: per-junction two-draw Nyquist-anti-aliased stamp, amplitude `sqrt(Γ²)·sqrt(4·q·|I_prev|·fs)` from one-sample-lagged `state.i_nl_prev` (Γ²=1 for plain junctions). Diode 1 src, BJT 2 (1 when forward-active-reduced, **plus** a base-shot src Γ²=1/BF), JFET/MOSFET 1, Tube 1 — triode plate **space-charge smoothed** (Γ²=10·k·T₀·gm/(2·q·I_p) at the DC OP; `SHOT_GAMMA2=` override, 1.0 restores bare shot). Pentode plate → Phase 5 partition, not bare shot. VCA/op-amp skipped. Two-draw shot Nyquist fix 2026-07-19 (`a472807`); BE-primary single-draw.
- **Flicker (Phase 3 junction + Phase 3.5 resistor)**: per-junction and per-resistor 1/f via Paul Kellett 7-pole pink cascade. **fs/OS-invariant** calibration (recalibrated 2026-07-18): white input `sqrt(2·KF/K_pink)·|I|^(AF/2)` (K_pink≈6e-3 analytic; the ×0.11 tail is NOT unit-gain — K_pink sets the level) + Nyquist pair-sum `0.5·amp·(pink[n]+pink[n-1])`. Old `sqrt(4·KF·|I|^AF·fs)` was ~+30 dB hot at 96 k and fs-dependent. Junctions opt-in `.model NAME TYPE(KF=… AF=…)`, AF default 1.0; resistors opt-in per-element `R1 a b 10k KF=… AF=…`, Hooge bias-squared, AF default 2.0 (unbiased R → thermal only). KF=0 (default) → byte-identical. Shared `set_flicker_gain`. BE-primary single-draw `sqrt(0.5·KF/K_pink)`.
- **Partition (Phase 5)**: pentode plate two-draw stamp `sqrt(4·q·I_p·I_s/(I_p+I_s)·fs)·PARTITION_F` **replaces** bare plate shot (reuses `shot_gain`/`set_shot_gain`). `PARTITION_F` default 1.0 (process knob; ~0.6 for selected low-noise EF86). Triode-only / passive circuits emit zero partition codegen.
- **Op-amp en/in (Phase 4, v1 white-only)**: three Norton streams via `.model NAME OA(EN=… IN=…)` — en at in+ (`EN·noise_opamp_en_g_diag·sqrt(2·fs)`), in at in+ and in- (`IN·sqrt(2·fs)`), two-draw. Runtime `set_opamp_input_gain` (signal-independent, distinct from `shot_gain`). `EN_FC`/`IN_FC` parse and store but are **NOT wired** in v1. Op-amps without EN/IN → byte-identical.
- Runtime: `set_noise_enabled(bool)`, `set_noise_gain`, `set_thermal_gain`/`set_shot_gain`/`set_flicker_gain`/`set_opamp_input_gain`, `set_temperature_k(K)` (290 K default; only thermal scales with T), `set_seed(u64)` (0 → entropy from system clock, nonzero → deterministic). Each method emitted only when its mechanism is present. Salted per-phase streams so thermal/shot/flicker/partition/op-amp never share a prefix under one master seed.
- Calibration validated by kTC theorem (`tests/noise_psd_validation.rs`): `V²_rms = k_B·T/C` ±15% on a 10 kΩ / 100 nF RC. Nyquist regression: `thermal_noise_no_nyquist_artifact_on_resistor_only_output_node` asserts lag-1 > -0.5 AND RMS < 100 µV on a diode + series-R circuit. Full reference: [NOISE.md](NOISE.md).
- **BJT parasitic-R thermal (2026-07-18)**: `rbb′`/RC/RE thermal noise collected on the **nodal** path only (real internal-node injection); **skipped on the DK path** (no node pair for the Norton stamp) with a `log::warn!`. Route `--solver nodal` to include it. Diode `RS` / tube `RGI` are still not thermal-noise sources.
- The per-phase detail above is synced to [NOISE.md](NOISE.md) as of 2026-07-19 (fs-invariant flicker, triode space-charge smoothing, FA base shot, shot two-draw Nyquist fix `a472807`, Phase 4/5). [NOISE.md](NOISE.md) remains the authoritative reference for derivations and validation. User-facing guide: [../NOISE_GUIDE.md](../NOISE_GUIDE.md).

### Codegen Infrastructure
- **DK codegen** with augmented MNA (≤1 transformer group, M<10, K well-conditioned)
- **Nodal Schur** (medium complexity), **Nodal full LU** (K≈0 / positive K / ill-cond K or S)
- Full-LU optimizations stacked: chord method + cross-timestep Jacobian persistence + compile-time sparse LU (AMD ordering, symbolic factorization)
- Oversampling 2x/4x: self-contained polyphase half-band IIR, no runtime dependencies
- `--solver {auto|dk|nodal}`, `--backward-euler`, `--oversampling {1,2,4}`, `--opamp-rail-mode`
- **Runtime BE-latch (2026-07-28)**: nodal trapezoidal builds carry a cheap input-aware lag-1 anti-correlation detector; if the solver falls into a self-sustaining Nyquist `(-1)^n` limit cycle at a large-signal operating point (which the compile-time quiescent-OP auto-BE promotion can't see — jeffreys-tube V2 class), it latches that instance to the L-stable BE path for the rest of the stream (cleared by `reset()`, exposed via `diag_be_latch_count`). Not emitted for BE/force-trap/passive/saturating-inductor builds. Golden-audio verified zero-change across all 42 shipped circuits.
- **`.integrator {trap|be}` netlist directive (2026-07-28)**: deterministic compile-time integrator pin so a fleet regen can't silently change it. `be` ⇒ backward Euler; `trap` ⇒ trapezoidal + opt out of auto-promotion AND the runtime BE-latch net (same as `--force-trap`). Explicit CLI flags override the directive.

### CLI
- `melange compile` → Rust code or plugin project
- `melange simulate` → parse → MNA → DK/nodal → process WAV (`--input`, `--amplitude`)
- `melange analyze` → frequency response with `--pot`/`--switch` overrides
- `melange dc-op` → DC operating point
- Plugin shipability flags: `--vendor`, `--vendor-url`, `--email`, `--vst3-id`, `--clap-id`
- Plugin level params: Input Level + Output Level (±24 dB), `--no-level-params` to opt out

### Validation & Quality
- SPICE validation infrastructure (ngspice correlation)
- Parser hardening: input-size caps (10M bytes, 50k elements, 1k models, 256 name len), non-ASCII normalization
- cargo-fuzz target (parser → MNA → DkKernel → CircuitIR)
- Error types: `#[non_exhaustive]` enums, no panicking library code
- Logging via `log` crate (no `eprintln!` in library code)
- Real-time safety: no alloc/locks/syscalls in audio processing, all buffers preallocated

## Performance

- DK circuits: 100–600× realtime (Schur path)
- Nodal full LU (Pultec, N=41, M=8, 2 transformers): ~11× realtime with all stacked optimizations
- VCA compressor (N=21, M=3, nodal full LU): ~42× realtime
- 8-BJT Class AB power amp (DK M=9): 0.4× realtime (parasitic-R limited; K_eff approach planned)

## Known Limitations

- Parasitic caps (10pF) auto-inserted across junctions for purely resistive nonlinear circuits
- Tube Koren: lambda parameter models finite plate resistance; no space-charge or transit-time effects
- BJT GP: no substrate current or avalanche breakdown
- All device models fixed at room temperature (27°C); no TNOM/TC1/TC2/XTI
- `MAX_M=24` — bound on NR dimension; iterative/sparse NR for M>24 deferred. Bumped from 16 on 2026-04-19 to admit Uniquorn v2 (M=20) and leave headroom for split-band saturation designs.
- Full-LU NR + ill-conditioned A (cond(A) > ~1000): Schur preferred when K well-conditioned. No known circuit needs both pathological K and ill-conditioned A. See DEBUGGING.md "Known Full-LU NR Limitations"
- Ideal transformer decomposition (dependent sources + explicit leakage/magnetizing L): deferred, current coupled-inductor approach sufficient for Pultec at +1.8 dB

## Validated Circuits

Circuit netlists live in the [melange-audio/circuits](https://github.com/melange-audio/circuits) repo
(locally `../melange-circuits`). Circuit-specific tests use `.test.toml` sidecars. All circuits
start in `unstable/`; promotion to `stable/` requires user DAW sign-off (SPICE correlation
and compilation are necessary but not sufficient).

- **Passive tube EQ** (passive-eq1a): 4 tubes, 2 transformers, 7 pots, 3 switches, global NFB. Sowter DWG E-72,658-2. ~11× RT on nodal full LU. Flat ±1 dB 20Hz–15kHz, 21 dB differential NFB.
- **Wurlitzer 200A preamp** (wurli-preamp): N=11, M=3–5 FA, 2N5089 Ebers-Moll. SPICE-validated 6-nines, 3.2% RMS.
- **Wurlitzer 200A power amp** (wurli-power-amp): N=20, M=9–16 FA, quasi-complementary class AB. DK codegen 0.4× RT, nodal 0.04×.
- **Tweed-style 2-stage 12AX7 preamp** (twas-preamp): N=13, M=4. 50 mV → 549 mV (+20.8 dB). Zero NR divergence.
- **SSL bus compressor** (4kbuscomp): 12 op-amps, 2 VCAs, 6 diodes, 2 pots, 2 switches. DC OP basin trap FIXED 2026-04-17 (`b771512`, post-fallback refinement NR). Transient chord-NR false convergence PARTIAL FIX 2026-04-17 (`c3d3eae`, residual check on ActiveSetBe/ActiveSet) — stable at `d ≤ 2 s` all amps on the original netlist. `d = 5 s` closes only with the netlist-side `.model OA_TL074 VSAT=11 → 13.5` fix (TL07x on ±15 V swings to ±13.5 V per TI datasheet); that diff is currently uncommitted in `melange-circuits/unstable/dynamics/4kbuscomp.cir`. See DEBUGGING.md "ActiveSetBe Chord-NR False Convergence" and "Precision Rectifier DC OP Convergence".
- **VCR audio ALC compressor**: N=21, M=3, nodal full-LU ~42× RT. Key: 100Ω Rdecouple between VCA sig- and I-V converter fixes positive K diagonal.
- **Klon Centaur**: ActiveSetBe auto-route (verified amp=[0.01..0.50]). BoyleDiodes opt-in only (heavy-clip divergence at amp ≥ 0.05 unsolved — not a blocker, see DEBUGGING.md).
- **Tube Screamer** / guitar pedals: stable.
- **Pentode stages**: EL84 single stage, Tweed Deluxe (6V6GT beam tetrode), 6K7 varimu, Plexi (4×EL34 grid-off FA M=18→14). DC-OP validated, end-to-end compile-and-run verified.
- **Uniquorn v2**: 16-stage cascade (N=64, M=12, ~3× RT mono) + push-pull power (N=23, M=6, ~15× RT).

## Pending Work

- **Neve 1073**: EQ section (Stage 3), integration (Stage 4), plugin (Stage 5). Stages 1 & 2 BA283 amps SPICE-validated.
- **Oomox plugin roadmap**: `.runtime` VS, named constants, DC op accessor, warmup constant, runtime DC OP recompute
- **Performance**: DK parasitic BJTs (power amp 0.41×, K_eff approach planned); hot/cold state split; fast_powf for Koren tube model
- **Documentation**: user-facing docs, example circuits, getting-started guide
- **Multi-language codegen**: `Emitter` trait + `CircuitIR` are language-agnostic by design. Planned: C++, FAUST, Python/NumPy, MATLAB/Octave.
- **wurli-power-amp residual — FIXED 2026-08-03** (raised by melange-circuits 2026-07-25, after the auto-BE router-corroboration fix `b0dcb27` closed the timeout/explosion bug). Prior text here ("~10 dB past clipping, output still reaches 353 V") was itself stale — the raw internal-node blowup was far worse and erratic across amplitude (not monotonic with drive): amp 0.05 → 16,079 V, amp 1.00 → 27,977 V, amp 2.00 → 22,201 V internal, while amp 0.10/0.30/0.50 stayed physical (20–32 V) — convergence-path-dependent, not a clipping-level threshold. Root cause: `emit_nodal`'s per-iteration "global node voltage damping" (`nodal_emitter.rs`, both the primary NR loop and the Backward Euler fallback loop) capped the damping ratio with `.max(0.01)`, so a single NR iteration's LU solve producing a raw voltage delta many orders of magnitude beyond the intended cap (observed 3.8e7 V at a class-AB crossover device-state transition) still let a multi-kV single-iteration jump through (1% of 3.8e7 ≫ the ≤10 V ceiling). The BE-fallback's voltage-step-only convergence check then falsely accepted the resulting nonphysical fixed point (its relative tolerance scales with the already-diverged node voltage). Fixed by removing the `.max(0.01)` floor so the ratio divides uncapped, bounding every iteration's worst-case node step at exactly the intended threshold regardless of raw delta magnitude. All amplitudes now stay within 20–32 V internal; `nr_max_iter_count`/`be_fallback_count` also dropped 10–70× (bad state no longer cascades into subsequent samples). Regression: `nodal_be_fallback_alpha_floor_tests.rs::test_nodal_full_lu_node_damping_has_no_ratio_floor`.
- **BJT forward-active (FA) reduction rule re-check** (raised by melange-circuits 2026-07-25): on wurli-power-amp, 7 of 8 BJTs clear the `Vbc < -0.5 V` FA threshold (`DEBUGGING.md` — device evaluated at Vbc ≈ -20 V) yet all 8 stay full 2D in the shipped codegen. Open question whether the FA rule is still being applied as documented for this circuit, or whether something else (e.g. `--tube-grid-fa`-style override, K-conditioning skip-expansion gate) is suppressing it. Gates a downstream CPU-budget decision in openwurli. Not yet investigated.
- **BJT analogue of `--tube-grid-fa off`** (requested by melange-circuits 2026-07-25): a CLI flag letting a caller trade Gummel-Poon fidelity for CPU per-circuit on BJTs, mirroring the existing tube grid-cutoff override. Not designed or scoped yet — needs a decision on whether this is worth the new surface area before implementation.

### Deferred
- **`.switch`/`.pot`/`.runtime R` G-swap first-sample 2× artifact** (root-caused 2026-08-15, raised by melange-circuits/openfarf; user-gated fix). A conductance changed mid-run produces an output at the *swap sample* exactly 2.000× the physical value (drive-independent, deterministic), correct one sample later. Mechanism: trapezoidal puts every conductance in BOTH `A = g+(2/T)C` and `a_neg = (2/T)C−g`, so a switch of Δg adds +Δg to the forward matrix and −Δg to the history matrix; on the swap sample `v[n] = v_prev − 2·A_new⁻¹·Δg·v_prev` — Δg counted twice (`rebuild_matrices` correctly rebuilds `a_neg`; this is inherent to the trap formulation, not staleness). It is a **bug, not a contract** — a resistive divider responds instantly, so the swap sample should be physical; do NOT document it as "undefined." Fix: use the pre-switch conductance in the history term for the one transition sample (one-sample old-g lag on the switched conductance in `a_neg`) — a core-solver change touching every `.switch`/`.pot`/`.runtime R` circuit, needs full golden/SPICE validation. Repro: any `.switch` onto a resistive path, toggle mid-run, compare first-sample deflection to settled ratio. **Second artifact (same event):** the swap also excites a trapezoidal z=−1 Nyquist marginal-stability mode that rings for ~1000 samples (damped by circuit RC; parity-split-into-two-smooth-sequences signature; verified undamped in a no-cap repro, and gone under `--backward-euler`). The old-g lag does NOT kill this mode — it only shrinks the exciting impulse. **Complete fix is two parts:** (a) the one-sample old-g history lag (kills the 2×); (b) breakpoint-style force-BE for 1–2 samples after any `.switch`/`.pot`/`.runtime R` event (damps the Nyquist mode at the source — commercial-SPICE breakpoint practice; the switch-triggered analog of the existing nodal auto-BE). Regression oracle: a purely resistive node obeys `node − ratio·other ≈ 0` at all times, so the entire post-swap residual is the artifact. Workaround (openfarf): fit transients from closure+1; settle-time measurements within ~1000 samples of a swap are contaminated under trap. See memory `switch_gswap_trap_2x_first_sample_2026_08_15`.
- **G10 divider hard-switching NR overshoot** (root-caused 2026-08-14, user-deferred as non-blocking). Germanium-PNP astable divider (`melange-circuits/local-docs/repro-ic-vcvs-blowup.cir` / `repro-wav-input-blowup.cir`) overshoots to ~80 V on an 8 V rail under a full-strength switching trigger (ground truth from melange-circuits: real full chain swings inside 0–8 V, duty 79.6%). Signature: BE-fallback storm (~88 % of samples vs ~1 % when physical). Root cause: at a switching edge the astable's positive-feedback K coupling pins a junction at v/vt ≈ 300 (v_d ≈ 7.8 V), `i_dev` saturates at `IS·exp(40)` ≈ 7e10 A, the Jacobian goes catastrophically ill-conditioned, and per-sample direct Newton stalls (‖f‖ flat at 7e10, exhausts MAX_ITER=90); the BE fallback inherits the same pinned state. Decisively ruled out by experiment: warm-start predictor, **line-search** (flat region, no descent direction), pnjlim sub-threshold-skip removal, and absolute v_d clamping. **Fix = port the DC-OP continuation (gmin/source stepping, `dc_op.rs`) into the per-sample `solve_nonlinear`** — substantial, higher-risk; validate against 24 SPICE + solver tests + oomox plugin-render golden gate. NOT blocking the working full G10 chain (which converges to rail). Prior "true Newton / Anderson" guess for this class is superseded by the gmin/source-stepping direction. **Acceptance criteria when this lands (from melange-circuits 2026-08-15):** (1) the ~80 V overshoot case stays bounded/physical; (2) NEW — *converged-vs-starved pitch agreement*: on the free-running G10 astable cascade, `--max-iter 70` (~11% NR starvation, which does NOT latch and looks healthy) leaves the master oscillator **3.8% flat** (a third of a semitone) with duty drifting ~3 points vs `--max-iter 1000`, because every non-converged sample leaves a slightly-wrong state that *integrates into pitch* on an oscillator. A failure-fraction warning (see the NR-starvation warning, `fe5c12a`) fundamentally cannot catch this sub-threshold detune class — the continuation is the real fix. See memory `g10_divider_hard_switching_overshoot_2026_08_14`.
- Ideal transformer formulation (Pultec at +1.8 dB with current approach, not blocking)
- Phase 6a/6b type safety (NodeIdx newtype, field visibility)
- Phase 7 crate split (extract melange-parser, melange-codegen)
- M>24 iterative/sparse NR
- BoyleDiodes heavy-clip Anderson acceleration / BoyleDiodes→ActiveSetBe hybrid (low priority)

## Cross-Compilation (macOS from Linux)

Zig 0.13 + cargo-zigbuild + macOS SDK 13.3 + rcodesign (ad-hoc signing).
`cargo zigbuild --release --target universal2-apple-darwin` produces universal Mac binaries.
melange-cli does NOT cross-compile (ureq/dirs need CoreFoundation), but generated plugins do.
