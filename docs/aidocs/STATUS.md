# Melange Status Reference

Quick-reference for AI agents. For math details see other aidocs. For architecture see CLAUDE.md.

> **Latest release: v0.1.11 (2026-09-27)** — a same-night patch over v0.1.10, whose test suite
> did not compile (test-harness code only; shipped behaviour is byte-identical). v0.1.10 was
> the never-silently-wrong release. Nodal full-LU could
> commit an unsolved sample and then freeze on it, giving a 22 dB-wrong render that reported a
> healthy −0.50 dBFS peak; the fix cuts the timestep to 64× and honours the pinned integrator, and
> two zero-threshold counters now fail every verb rather than reporting a reassuring number. Triode
> grid current became Dempwolf & Zölzer eq. (11) — which **fails** its Philips ECC83 acceptance test
> 15/15 and ships as the less-wrong model, replacing a law that was further out in the same
> direction; see Pending Work for the specified fix. See CHANGELOG and `docs/aidocs/DEBUGGING.md`.
>
> **UNRELEASED work sits on local `main`, 8 commits ahead of `origin/main`** (a push to origin/main
> IS a release, so it is held). Headline: the **Dempwolf & Zölzer triode grid-current law**
> (`30915fb`) — `Ig = Gg·(softplus(Cg·Vgk)/Cg)^ξ`, evaluated for ALL Vgk. **This changes generated
> DSP for every circuit containing a triode**, and `VGK_ONSET`/`IG_MAX` are now REFUSED on triode
> `.model` cards (still honoured on pentodes). Costs 14–29 % throughput on triode decks. Also:
> `analyze`'s `phase_deg` sign fix, a codegen warning fix, and a full performance re-measurement.
>
> **The grid law is shipped but ungraded** — the out-of-sample ECC83 `V_o` acceptance run and the
> per-tube-type onset WARN are specified in `.claude/release-docs/FOLLOWUPS.md` and NOT started.
> It is a better-founded law that nothing has yet falsified; that is not the same as a validated one.

> **2026-07-18 accuracy campaign (commits `3e246cb`, `5159b8c`, `fde289a`, `b421358`, `8056f95` — all six review chunks complete):** a
> full-codebase accuracy review fixed, among others: op-amp VCCS polarity (was
> inverted — clipping/comparator behavior changed), Koren triode ×2 factor (triode
> stages now run at datasheet current/gm — passive-EQ/SeriesOfTubes gain staging
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
| Tube-Screamer-style overdrive (TS808) | 0.99999032 | 0.442% | 9.9e-3 V | Op-amp + 1N4148, THD err 0.04 dB |
| Tube-Screamer-style overdrive (wiper, pos=0.85) | 0.99838696 | 5.71% | — | Volume divider + simplified tone; THD err 0.11 dB |
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
- **Op-amp**: Boyle VCCS macromodel (no bandwidth pole: GBW only defaults the rails), rail-clamping modes (`auto/none/hard/active-set/boyle-diodes`, see `--opamp-rail-mode` CLI flag), and optional slew-rate limiting via `.model OA(SR=13)` in V/μs (per-sample `|Δv_out| ≤ SR·dt` clamp, all 3 codegen paths, default `SR=∞` → zero code emitted)
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
| Tube (triode) | 2D (Vgk→Ip, Vpk→Ig) | Koren plate + Dempwolf & Zölzer grid |
| Tube (pentode) | 3D (Vgk→Ip, Vpk→Ig2, Vg2k→Ig1) | Reefman Derk §4.4 / DerkE §4.5 / Classical + Leach |
| Tube (pentode, grid-off) | 2D (Vgk→Ip, Vpk→Ig2, Vg2k frozen) | Opt-in only (`--tube-grid-fa on`, warned); `auto` keeps full 3D since 2026-09-04 — the freeze is not accuracy-neutral (cathode-referenced Vg2k, +2–12% measured) |
| VCA | 2D (Vsig, Vctrl) | THAT 2180 exponential |
| Op-amp | Linear (no NR dim) | Boyle VCCS + rail clamp; no GBW pole (GBW only defaults ±13 V rails, with a notice) |

M=1 direct, M=2 Cramer's, M=3..24 Gaussian elimination with partial pivoting.

## Codegen Solver Routing (Updated 2026-03-23)

| Path | When Selected | Cost | Notes |
|------|--------------|------|-------|
| DK Schur | M<10, ≤1 xfmr, K well-conditioned, no op-amp needing active-set rail handling | O(N²+M³)/sample | *(no measured figure — see README table)* |
| Nodal Schur | M≥10 or 2+ xfmr, K well-conditioned | O(N²+M³)/sample | Medium-complexity circuits |
| Nodal full LU | saturating inductor or behavioral source (structural); K≈0 (VCA), positive K diag, K ill-cond | O(N³)/sample | Matches runtime exactly |

K≈0 detection: max|K| < 1e-6 with M > 0.

A clamped op-amp whose rail mode resolves to `active-set`/`active-set-be` (an
AC-coupled output under `auto`, or asked for explicitly) routes nodal: only
nodal implements the pin-and-resolve, and forcing DK is refused. See
`OPAMP_RAIL_MODES.md` → "Which solver runs which mode".

## Circuit Library Status

Circuits live in a separate repository. **Public since 2026-09-27: https://gitlab.com/oomox-group/melange-circuits** (43 circuits — a FILTERED set; the full catalog is the private `melange-circuits-private`, checked out locally at `../melange-circuits`). Short names resolve through the repo's `circuits-index.json`; see `docs/CIRCUIT_INDEX.md`.
All circuits are in `unstable/` until the user manually tests and approves promotion.

The compiler validation status of circuits known to exercise specific solver paths:

| Topology class | N | M | Solver | Performance | Notes |
|----------------|---|---|--------|-------------|-------|
| Linear RC | 2 | 0 | Linear | trivial | Smoke test |
| 2-stage BJT preamp | 11 | 3-5 | DK | fast | FA detection, 2N5089 Ebers-Moll |
| 2-stage triode preamp | 13 | 4 | DK | fast | 2× 12AX7, pot + switch |
| 4-tube passive EQ + 3 xfmrs | 52 | 8 | Nodal full LU | ~21× | Chord + cross-timestep + sparse LU |
| 8-BJT Class AB power amp | 20 | 9-16 | DK/Nodal | 0.4× / 0.04× | Parasitic R, FA detection |
| 4-opamp + diode clipper | 44 | 10 | Nodal full LU (auto) | — | Clean clipping verified under ActiveSetBe; auto now resolves ActiveSet + transition-BE, not re-run on this deck; BoyleDiodes diverges at heavy clip |
| Op-amp overdrive + diodes | — | — | DK | — | TS808-class clipping |
| VCA compressor + sidechain | 21 | 3 | Nodal full LU | ~42× | Current-mode VCA, K≈0 |
| Pentode single stage | — | 3 | DK | fast | Full 3D by default; grid-off M=3→2 only with `--tube-grid-fa on` |
| Push-pull pentode amp + OT | — | — | Nodal | — | Transformer forces nodal path |
| Variable-mu pentode | — | 3 | DK | fast | M=3, no grid-off reduction |

Only circuits using standard SPICE models (D, NPN/PNP, NJF/PJF, NM/PM) can be validated
against ngspice. Circuits with melange-extended models (OA, VCA, VP, triode) use
`melange compile`/`analyze`/`simulate` for validation, not ngspice.

Promotion to `stable/` requires user sign-off after a DAW listening test.
SPICE correlation and successful compilation are necessary but not sufficient.

## Passive-EQ Schematic Data

Source: Sowter DWG E-72,658-2 (amp §) + Peerless/Triad winding data.

- **HS-56**: input transformer (37H:37H), 620Ω shunt across the secondary
- **HS-29**: 1:2 step-up, 45H, true push-pull (pin 5→grid1, pin 8→grid2); CT grounded, 43K+270pF bridges grid-to-grid
- **S-217-D**: 220H primary (30Hz), 71-turn tertiary (0.447H), 20pF plate cap, .003µF+620Ω output Zobel (secondary floats)
- **Feedback winding**: 12AX7 pin 3→360Ω→S-217-D pin 3; pin 8→360Ω→pin 5
- **Cathodes**: 12AX7 820Ω between cathodes (not to ground); 12AU7 separate, 4.7kΩ+50µF each
- Gain budget: +25 dB amp − 23 dB EQ = +2 dB net

## Feature Inventory

### Core Pipeline
- MNA stamping: R, C, L, V/I sources, diodes, BJTs, JFETs, MOSFETs, tubes, op-amps, VCAs
- DK kernel with proper trapezoidal discretization; NR solver 1D / 2D / M-dimensional (M≤24)
- Codegen for diode, BJT, JFET, MOSFET, tube/triode/pentode (Gaussian elimination M=3..24)
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
- **Tube (triode)**: Koren plate + Dempwolf & Zölzer grid current, early-effect lambda, CCG/CGP/CCP junction caps, RGI grid-stop
- **Tube (pentode)**: 3 screen-current equation families — Rational (Reefman §4.4), Exponential (DerkE §4.5), Classical Koren. `--tube-grid-fa {auto,on,off}`: `on` reduces 3D→2D (warned, not accuracy-neutral); `auto` == `off` == full 3D (2026-09-04). `diag_region_exit_count` counts grid-conduction / BJT-saturation samples on every path
- **Op-amp**: Boyle VCCS macromodel (no GBW pole), VCC/VEE asymmetric rails, optional `SR=` slew-rate limiting (V/μs), rail modes `auto/none/hard/active-set/active-set-be/boyle-diodes`, `AOL_TRANSIENT_CAP` override
- **VCA**: THAT 2180 / DBX 2150 current-mode exponential gain with gain-dependent THD
- **Saturating inductor / transformer**: anhysteretic flux `Φ = L_mag·Isat·tanh(i/Isat) + L_air·i` with an air-core floor (`LAIR=`/`CORE=gapped|steel|nickel`, default 3e-4 with a notice); datasheet ratings `ISAT_DROP=`/`ISAT_BASIS=`/`L_AT_IDC=` converted exactly; two-winding shared cores saturate on the T-model's magnetizing branch. A flux device inside the nodal full-LU Newton loop at every site (main, sub-step, BE fallback, active-set pin); DK and nodal Schur refused. Refuses k ≤ 0.8 groups, 3+ windings, conflicting ISAT/floors. Full reference: [SATURATING_TRANSFORMERS.md](SATURATING_TRANSFORMERS.md).

### Unit Variation (2026-04-21)
- `.seed <u64>`: sets master RNG seed (default 0). Shared by `.mismatch` and `.tolerance`.
- `.mismatch D IS=tol N=tol RS=tol` / `.mismatch Q IS=tol BF=tol BR=tol`: per-device parameter jitter, baked at codegen. Two diodes on the same `.model` land at distinct `DEVICE_N_IS` constants — the thing that makes antiparallel clippers and push-pull pairs audibly asymmetric. **`T` / `J` / `M` are now IR-wired too (v0.1.3):** `T` (triode+pentode) jitters MU/EX/KG1/KP/KVB (+KG2 pentode), `J` IDSS/VP/LAMBDA, `M` KP/VT/LAMBDA. Byte-identical when the directive is absent; `analyze` applies it. Per-device tube mismatch is the physically-honest H2 source in a balanced push-pull stage (identical halves cancel evens exactly) — see `UNIT_VARIATION.md` and `SATURATING_TRANSFORMERS.md` §8-Q1.
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
- **Runtime BE-latch (2026-07-28)**: nodal trapezoidal builds carry a cheap input-aware lag-1 anti-correlation detector; if the solver falls into a self-sustaining Nyquist `(-1)^n` limit cycle at a large-signal operating point (which the compile-time quiescent-OP auto-BE promotion can't see — jeffreys-tube V2 class), it latches that instance to the L-stable BE path for the rest of the stream (cleared by `reset()`, exposed via `diag_be_latch_count`). Not emitted for BE/force-trap/passive builds. Emitted for saturating-inductor circuits, including M = 0 ones, since 2026-09-28. On both nodal sub-paths a latched sample skips the trapezoidal solve and runs the same backward-Euler routine a `--backward-euler` build of that sub-path runs (full-LU: main loop, sub-step, pin; Schur: M-dim Newton, pin), bit-identically from the same state. On a core with no air-core floor (`LAIR=0`) it catches the deep-saturation trapezoidal ring (inductor current 20 % over the V/R ceiling at 20× Isat; 4× oversampling does not cure it); with a floor (default 3e-4 of L0) the saturating RL at 10-20 V and the choke-loaded common-source stage at 1-30 V do not ring and it does not fire. It still fires on a step into an open saturating transformer (golden `sat-core-open/step`), and that ring is not saturation: with `ISAT=100` (core linear throughout) it latches identically, with a 600 Ω load it does not (measured 2026-09-28; the open secondary's stiff leakage-into-1 MΩ mode, see SATURATING_TRANSFORMERS.md §3.4). **Cost, because the latch is sticky:** a ring that is really a transient commits that instance to BE for the rest of the stream; measured with `LAIR=0`, H1 −1.8e-4 (saturating RL at 10 V) and output −0.28 % (choke-loaded stage at 5 V). A release policy (unlatch after N clean samples) is open only if a deck shows that cost mattering.
- **`.integrator {trap|be}` netlist directive (2026-07-28)**: deterministic compile-time integrator pin so a fleet regen can't silently change it. `be` ⇒ backward Euler; `trap` ⇒ trapezoidal + opt out of auto-promotion AND the runtime BE-latch net (same as `--force-trap`). Explicit CLI flags override the directive.
- **`.oversampling {1|2|4}` netlist directive (2026-09-05)**: a deck declares its recommended oversampling factor to control aliasing from nonlinear distortion products. It is an accuracy **minimum/recommendation, not a mandate** — rate costs CPU/latency (the plugin author's call). Resolution on compile/simulate/analyze: an explicit `--oversampling` always wins (even when lower — logs a `log::warn!`), else the deck value, else 1. **`validate` does NOT read the directive** — it takes `--oversampling {1|2|4}` explicitly (default 1) and reports what it was asked to measure; the reference is unfiltered and aligned by one best-fit constant delay, so the half-bands' phase stays in the number (see OVERSAMPLING.md § Validating an oversampled build). Stripped for ngspice via `MELANGE_ONLY_DIRECTIVES`.

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

**Re-measured 2026-09-27** on an AMD Ryzen 9 7950X pinned to one CCD (single core, noiseless, `-C target-cpu=x86-64-v3`, via `tools/perf-harness/bench.sh`); host-dependent. Figures predating 2026-08-25 were largely fabricated/stale — see `memory/perf_numbers_measured_2026_08_25.md`. Measured: nonlinear audio circuits ≈7.5–46× RT; light stages ~159× (single 12AX7); trivial linear ~2960× (7.0 ns/sample).

- Passive EQ (N=52, M=8, 3 xfmrs, nodal full LU): **~20.3×** realtime (1028 ns/sample)
- Wurlitzer preamp (2 BJT, full GP): ~45.9× · Tweed 5F1 amp: ~19.0× · Ge diode network: ~12.0× · bus comp (full, 12 op-amps + 2 VCAs): **~7.5×** · 12AX7 stage: ~158.8×
- ⚠️ The **"overdrive pedal ~60.3×"** row was DELETED 2026-09-27: no deck in any repo matches its "op-amp + 2 diodes" description, so it has been unreproducible since 2026-09-02 and shipped twice unverified. Do not reinstate it without a named deck.
- **What moved since the 2026-09-03 table**, all attributed on the same box: the three TRIODE rows lost 14–29 % to the Dempwolf & Zölzer grid-current law (`30915fb`) — 12AX7 216.7→153.2, tweed 22.3→18.3, passive EQ 24.0→20.6, each measured against the commit immediately before it. The Wurlitzer row (56→43.8) is a DECK revision of 2026-09-16, not a compiler regression: the pre-revision deck still reads 52.9× on the 0.1.5 binary that published the 56×. Everything else reproduces within −6 % to +2 % on the old binary, which is this bench's honest width on this host.
- **Full-LU exit step (2026-09-28).** A chord-accepted full-LU sample with a node residual above 1e-9 A takes one refactored Newton step (see the capless-row item under Pending Work). Measured before/after on the same 7950X, same session (`bench.sh`, x86-64-v3, best of 7 × 2M; box not idle, load ~2.7, so absolute figures read a few % below the idle table): bus compressor 2901 → 3136 ns/sample (7.18× → 6.64×, **−7.5 %**); 12AX7, tweed, passive EQ, Ge network and Wurlitzer preamp within −1.0 % to +0.5 %. **The published bus-compressor row predates this change and must be re-measured on an idle box at the next release.** Corpus full-LU decks (golden `circuit.rs`, 48 kHz, one core, best of 3, `target-cpu=native`, ad hoc): at silence every deck is within noise of before (the gate keeps the chord's reuse); on a 0.1 V 1 kHz sine moonladder +32 %, steve-1073-preamp +11 %, gravity +11 %, sad-bastard +10 %, 4kbuscomp-audiopath +6 %, wurli-power-amp +3 %, the rest within noise. The cost is paid only on samples whose accepted chord step left a node residual.
- VCA compressor (N=21, M=3, nodal full LU): ~42× realtime *(not re-measured 2026-08-25)*
- 8-BJT Class AB power amp (DK M=9): 0.4× realtime *(not re-measured; parasitic-R limited; K_eff approach planned)*

## Known Limitations

- Parasitic caps (10pF) auto-inserted across junctions for purely resistive nonlinear circuits
- Tube Koren: lambda parameter models finite plate resistance; no space-charge or transit-time effects
- BJT GP: no substrate current or avalanche breakdown
- All device models fixed at room temperature (27°C); no TNOM/TC1/TC2/XTI
- `MAX_M=24` — bound on NR dimension; iterative/sparse NR for M>24 deferred. Bumped from 16 on 2026-04-19 to admit Uniquorn v2 (M=20) and leave headroom for split-band saturation designs.
- Full-LU NR + ill-conditioned A (cond(A) > ~1000): Schur preferred when K well-conditioned. No known circuit needs both pathological K and ill-conditioned A. See DEBUGGING.md "Known Full-LU NR Limitations"
- Linear coupled inductors use the exact `[L]` coupled-inductor path; the ideal-transformer T-model (leakage + ideal couplings + one magnetizing L) is built only for saturating two-winding cores, on nodal full-LU. The coupled-inductor approach is sufficient for the passive EQ at +1.8 dB.
- Saturating transformers with 3+ windings are refused (the exact three-winding star form is specified, not built; SATURATING_TRANSFORMERS.md §8).
- **Glow/neon relaxation-oscillator (`N … NEON(…)`, Phase 0c; EXPERIMENTAL).** **SHIPPED on main in v0.1.7 (`7ecb36c`, 2026-09-10), incl. `--subsample-fire` (see Deferred/Now-Landed).** Reset model = **Option A maintaining LINE** `i=(v−V0)/RS`, intercept `V0=VM−RS·IK` DERIVED (`.model NEON(VO VM IK RS IHOLD ROFF)`) — the reservoir-cap reset floor emerges at ~89 V (measured 93→88.95 V, both routes) instead of the old fixed VD=93. Slope RS is `placeholder-pending-ZA1001` (ZA1004 form-transfer, sourced ~2.5–4.25 kΩ). **⭐ ROOT-CAUSE UPDATE 2026-09-10 (design review + control run): the Philicorda VO=128 divide-fail is the RESET FLOOR parked too high (static V_m≈89 V vs cap-dependent V_m≈82 V), a DISCHARGE-side defect — NOT RS, NOT the strike/extinction model.** Control: lowering V0 ~6.5 V with RS held at 3k opens the both-hold (B5-lock ∧ B6-÷2) window at VO=128. A 2026-09 detour chasing RS then a deionisation-state extinction ("v2") was a false premise: v2 fixes the self-extinguish CEILING (held-out Sheet C: v1 falsified, v2 confirmed) but STRUCTURALLY BREAKS the divider at every t_r (re-ignition depression → stages fire too easily), and field-dependence can't reconcile (~1.7× too weak). v2 is NOT deployed (parked patch). **Real fix (operator-scheduled) = a physics-derived V_m(C) reset model, validated vs held-out Sheet C via pre-registered ceiling predictions (never Sheet B — retired).** VO=135 remains the working card (openphilicorda 12/12 boards, 49/49 keys offline).
  - **Edge aliasing:** strike/extinguish edge quantized to the sample grid → a bare oscillator node aliases at base rate. **Anti-alias = whole-circuit oversampling** (internal blind-review ruling — NOT the cross-project design review: output BLEP REJECTED — ill-posed for a mixed multi-oscillator output, and a cosmetic output filter is forbidden by the accuracy-over-output-mapping rule). **UPDATE 2026-09-07: the sub-sample breakpoint re-solve is no longer deferred — it LANDED as `--subsample-fire` (see Deferred/Now-Landed below); at the top of a divider chain the defect is injection-lock breakage, not aliasing, and oversampling cannot reach it (12 pitches need inner ≈768 kHz = os16).** Measured (`glow_relaxation_tests.rs`): for RC-loaded dividers the reservoir cap band-limits the discharge (τ=RS·C≈30 µs → a fast ramp, not an ideal step), so base-rate aliasing is modest (ASR ≈ −35 dB on a ~115 Hz divider) and 4× OS does not worsen it; OS=4 preserves the oscillator physics exactly. Recommend OS≥4 for glow-bearing decks. A trustworthy cross-divider ASR study (a naive FFT ASR on a few-sample-period self-oscillator is artifact-dominated) + a listening pass are the gate on ever building the breakpoint fix.

## Validated Circuits

Circuit netlists live in a separate repository (public subset at https://gitlab.com/oomox-group/melange-circuits since 2026-09-27; full catalog private)
(locally `../melange-circuits`). Circuit-specific tests use `.test.toml` sidecars. All circuits
start in `unstable/`; promotion to `stable/` requires user DAW sign-off (SPICE correlation
and compilation are necessary but not sufficient).

- **Passive tube EQ** (passive-eq1a): 4 tubes, 3 transformers, 7 pots, 3 switches, global NFB. Amp § from Sowter DWG E-72,658-2. N=52, M=8; ~21× RT on nodal full LU. Flat ±1 dB 20Hz–15kHz, 21 dB differential NFB.
- **Wurlitzer 200A preamp** (wurli-preamp): N=11, M=3–5 FA, 2N5089 Ebers-Moll. SPICE-validated 6-nines, 3.2% RMS.
- **Wurlitzer 200A power amp** (wurli-power-amp): N=20, M=9–16 FA, quasi-complementary class AB. DK codegen 0.4× RT, nodal 0.04×.
- **Tweed-style 2-stage 12AX7 preamp** (twas-preamp): N=13, M=4. 50 mV → 549 mV (+20.8 dB). Zero NR divergence.
- **SSL bus compressor** (4kbuscomp): 12 op-amps, 2 VCAs, 6 diodes, 2 pots, 2 switches. DC OP basin trap FIXED 2026-04-17 (`b771512`, post-fallback refinement NR). Transient chord-NR false convergence PARTIAL FIX 2026-04-17 (`c3d3eae`, residual check on ActiveSetBe/ActiveSet) — stable at `d ≤ 2 s` all amps on the original netlist. `d = 5 s` closes only with the netlist-side `.model OA_TL074 VSAT=11 → 13.5` fix (TL07x on ±15 V swings to ±13.5 V per TI datasheet); that diff is currently uncommitted in `melange-circuits/unstable/dynamics/4kbuscomp.cir`. See DEBUGGING.md "ActiveSetBe Chord-NR False Convergence" and "Precision Rectifier DC OP Convergence".
- **VCR audio ALC compressor**: N=21, M=3, nodal full-LU ~42× RT. Key: 100Ω Rdecouple between VCA sig- and I-V converter fixes positive K diagonal.
- **4-op-amp overdrive with diode clipper**: verified bounded under ActiveSetBe at amp=[0.01..0.50]; auto now resolves ActiveSet + transition-BE, not re-run on this deck. BoyleDiodes opt-in only (heavy-clip divergence at amp ≥ 0.05 unsolved — not a blocker, see DEBUGGING.md).
- **Tube-Screamer-style overdrive** / guitar pedals: stable.
- **Pentode stages**: EL84 single stage, Tweed Deluxe (6V6GT beam tetrode), 6K7 varimu, Plexi (4×EL34; grid-off M=18→14 only under `--tube-grid-fa on` — full 3D by default routes it nodal). ngspice-validated full-3D (2026-09-04): twill-deluxe 0.063%, el84-single-stage 0.233%, noyce-6bq5 0.060%, noyce-ef86 0.060%.
- **Uniquorn v2**: 16-stage cascade + push-pull power. ⚠️ The dimensions and throughput figures
  once quoted here (N=64/M=12/~3× RT; N=23/M=6/~15× RT) are UNSUPPORTED and have been
  removed: no `uniquorn*.cir` exists in melange-circuits, so there is no deck to attribute
  them to and they cannot be re-measured. Do not reconstruct them from the stale generated
  `.rs` artifacts. Survivors of the 2026-08-25 fabricated-figure sweep, which removed them
  from README and limitations but missed this file.

## Pending Work

- **⭐ TRIODE GRID CURRENT FAILS ITS ACCEPTANCE TEST — sub-µA branch specified, not built** (opened 2026-09-27). Dempwolf & Zölzer eq. (11) as shipped in 0.1.10 reaches the 0.3 µA criterion **0.26–0.35 V too late**: −0.26..−0.35 V against the **−0.61 V** implied by the Philips ECC83 (Jan 1970) AF-amplifier block's own five columns, which agree to sd 0.067 V across a 2× range of `Vb` and 2.75× of `Rk`. Fails **15/15 cells** (3 Table 1 rows × 5 `Vb`) under *both* readings of the criterion; the verdict does not flip. It ships because the hard-zero law it replaced is 0.61 V out in the same direction — less wrong, not right. Full evidence in `CHANGELOG.md` 0.1.10 and `docs/limitations.md` → Triode.

  **This is in scope, not an inaudible tail:** 0.3 µA into a following stage's 680 kΩ grid leak is ~0.2 V of bias shift, i.e. where blocking and bias-shift distortion begin in cascaded stages.

  **ROOT CAUSE (device-physics review, 2026-09-27): eq. (11)'s tail SLOPE is wrong, not its magnitude.** For `Vgk << 0` eq. (11) tends to `Gg·Cg^-xi·exp(xi·Cg·Vgk)` — a pure exponential of slope `xi·Cg` = 13.0 /V (0.177 V/decade for RSD-1), i.e. an *effective* cathode temperature of ~890 K. The physical tail is the retarding-field law (Spangenberg 1948 eq. 4.2: Maxwellian emission, `n/n0 = exp(-Ve/kT)`), whose slope is `e/kT_k`: 7.7 /V at 1500 K ("typical oxide operating temperature", Spangenberg), ~10.6–11 /V at the 1050–1100 K receiving-tube figure. **eq. (11) is steeper than any physical thermal slope**, so a fit made in the mA region falls away too fast when extrapolated 3–4 decades. The slope excess alone accounts for ~3–14× of the 29–257×; the rest is a tube-specific offset.

  **The onset spread is itself normal** — RDH4 ch. 2 pp. 18–19 puts the cross-over of new indirectly-heated valves "usually between zero and −1.0 volt", varying between valves and drifting with life, and Philips' own ECC83 ratings are −1.3 V (1955) and −0.9 V (1960). The sheet's −0.61 V and eq. (11)'s −0.26..−0.35 V both sit inside that. But contact potential is only ~0.1 V of it (Spangenberg §4.6), so the bulk is the initial-velocity tail, which is the part above.

  ⚠️ **The measurand is NET grid current.** RDH4 pp. 19–20: a grid-current reading is electron current *minus* gas ionisation, grid primary emission and leakage. Philips' `Ig = +0.3 µA` is therefore a net meter reading, while eq. (11) models the electron term only — so **the true gap is if anything larger than the figures above, never smaller.** Gas cannot explain the law being late.

  **Shape of the fix (these are constraints, not suggestions — do not freelance around them):**
  - A **sub-µA branch alongside eq. (11)**, blended smoothly — *not* a refit of eq. (11), whose conducting-region fit is the part that works.
  - Below a transition current `I_t` (~1–10 µA, i.e. inside eq. (11)'s fitted range), use `Ig = I_t · exp((Vgk - V_t)/V_T)` with `V_T = k·T_k/e` and **`T_k` a physical constant taken from the 1050–1500 K band, not fitted**. `V_t` is where eq. (11) equals `I_t`. Match the *value* at `V_t` exactly; slope continuity is not free there, so blend over ~`V_T` with a softmax or accept a C0 join.
  - **The only per-tube parameter is an offset `dV`** (contact potential plus prefactor spread). That is the physical knob. A 28.7×–257× change in `Gg` has no physical reading; a few-tenths-of-a-volt offset does.
  - Its level may be anchored only by documented **static** device data, and **must NOT be fitted to the AF table**, which is what keeps that table an out-of-sample check. ⚠️ Philips' static −0.9 V is a **LIMIT (max), not a typical** — do not anchor a typical tail on a limit.

  **ACCEPTANCE TEST FOR THE FIX, defined in advance:** with `T_k` fixed and `dV` taken from **one** Philips column, predict the other **four** columns' onset voltages — they span 2× in `Vb` and 2.75× in `Rk`. That is out-of-sample within our own data. Evaluate it with the **plate card held fixed** (see the next item), or the two errors trade off and neither is measured.

  **Then:** implementation, then re-run the acceptance. The remaining four blocks of the sheet (47 kΩ, 100 kΩ, both phase-inverters) are out for an independent re-read and are genuinely unseen by melange — they stay that way until the run.

- **⭐ KOREN ECC83 PLATE CARD OVER-COMPRESSES 1.32×–2.26×** (opened 2026-09-27; predates the grid law and is tracked separately from it). At the Philips sheet's printed `Vo` the shipped card (`mu=100 ex=1.4 Kg1=1060 Kp=600 Kvb=300`, Koren's published 1996 set) reads THD 6.08/4.80/4.50/3.12/2.49 % against the printed 4.6/3.4/2.6/1.6/1.1 %, and runs `Ia` 2–10 % **low** (323.7/449.9/595.5/809.0/999.0 µA vs 360/480/630/850/1020). Measured at 192 kHz with a non-commensurate 997 Hz tone — a commensurate read (1 kHz at 48 kHz) folds aliases onto harmonic bins and cannot see its own aliasing.

  **⭐ STANDING RULE: every future grid-current check at these operating points MUST report the plate model's share**, because the two errors run in opposite directions and partly cancel at the output — over-compression depresses `Vo` at the criterion, so an output-voltage comparison alone *understates* the grid-side error.

- **Capless rows carry a walk of accepted Newton residuals under trapezoidal integration** (opened 2026-09-28). On a row with no capacitor, whole-system trap enforces `g(x_(n+1)) = -g(x_n) + e_n`, so every sample's accepted residual `e_n` adds, alternating in sign, until a backward-Euler sample resets it. The rows involved are the diode node of a clipper behind an op-amp's output cap, and any capless nonlinear node. No per-sample tolerance bounds the sum. On the overdrive deck in `OPAMP_RAIL_MODES.md`, with transition-BE resetting it at every pin and release, it is bounded: flat per-second maxima over 30 s, at most 1.9 µA, inside the main loop's own row tolerance (~3–5 µA). Two injection sites are measured:
  - **The pinned resolve** (both sub-paths, rail plateaus): it accepts on the node step (`1e-3·|v| + 1e-6`), so the committed pair is off KCL by about `½·g′·dv²`, ~0.05 µA per sample.
  - **The full-LU main loop in fast free swings — FIXED 2026-09-28.** A chord (reused Jacobian) step leaves a KCL residual `(J − J_chord)·Δ`, first order in the step: up to 2.2 µA per sample, invisible to the node-step test because a 0.64 mV node tolerance on a stiff diode row is ~45 µA of current. A chord-accepted iterate whose max node-row residual exceeds the row test's 1e-9 A floor now takes one refactored Newton step; full-LU matches the Schur sub-path in every measured cell. Measured and rejected: a simplified-Newton stopping rule on the chord's contraction rate (no effect: the accepted steps were already 12× inside the node tolerance), a tighter chord-refresh threshold (a tuned constant), refactoring every iteration (up to 2×), and an ungated exit step (2–3.4× at silence).
  - **Tolerance units, for the one-definition work:** node steps are tested in volts, residuals in amps. On high-conductance rows the volt tolerance is loose in current terms; a current-residual acceptance is the other principled form (a larger change, not taken).
  - **Structural item (design review, after the saturation work):** SPICE applies trapezoidal integration to charge and flux states only, so a capless row has no memory. A companion (charge-state) form would remove this walk and the `z=-1` family behind breakpoint-BE and transition-BE, but not trap ringing on the reactive states themselves. Before scheduling, enumerate that family with measured costs, then prototype on the overdrive deck: trapezoidal, no transition-BE, with the diode-node residual at the floor as the go/no-go. melange has no nonlinear junction charge today (diode `CJO` is constant; BJT `CJE`/`CJC` are linearized at the DC operating point); the saturating inductor's flux row is already in charge form.
- ~~**`melange simulate --switch "Label=pos"`**~~ **DONE** (verified 2026-09-22): the flag exists on `simulate` (`--switch <NAME=POS>`, `main.rs:4439` builds `switch_calls` from `switch_runtime_overrides`) and a `--switch "Tone=2"` run resolves and applies the position. This entry was stale.
- **Switch/pot z=−1 intrinsic to hard-switching (not just `.switch`):** beyond the `.switch` G-swap fix (Deferred), the trap z=−1 mode on capless subspaces is re-excited by ANY hard-switching edge (BJT edges in the g10 divider re-excite it every ~160 samples → openfarf's residual wanders 0.9–4.9% with no decay). Existing nodal auto-BE gates on ρ>1.002; a marginal z=−1 (ρ=1) may slip it — likely gap. Grader for this case: CLI == codegen window-for-window on the g10 chain keyed closed (not a 1e-7 target).
- **Neve 1073**: EQ section (Stage 3), integration (Stage 4), plugin (Stage 5). Stages 1 & 2 BA283 amps SPICE-validated.
- **Oomox plugin roadmap**: `.runtime` VS, named constants, DC op accessor, warmup constant, runtime DC OP recompute
- **Performance**: DK parasitic BJTs (power amp 0.41×, K_eff approach planned); hot/cold state split; fast_powf for Koren tube model
- **Documentation**: user-facing docs, example circuits, getting-started guide
- **Multi-language codegen**: `Emitter` trait + `CircuitIR` are language-agnostic by design. In progress: C++. Planned: Python/NumPy, MATLAB/Octave. **FAUST: explored, ruled out (2026-09-02)** — FAUST's generated code is not Turing-complete by design, so a data-dependent NR iteration count is inexpressible; only circuits emitting no NR loop at all would work (6 of 41 corpus circuits). Note the predicate is "no NR loop emitted", NOT `M == 0`: behavioural B-sources route nodal and get Newton regardless of M.
- **wurli-power-amp residual — FIXED 2026-08-03** (raised by melange-circuits 2026-07-25, after the auto-BE router-corroboration fix `b0dcb27` closed the timeout/explosion bug). Prior text here ("~10 dB past clipping, output still reaches 353 V") was itself stale — the raw internal-node blowup was far worse and erratic across amplitude (not monotonic with drive): amp 0.05 → 16,079 V, amp 1.00 → 27,977 V, amp 2.00 → 22,201 V internal, while amp 0.10/0.30/0.50 stayed physical (20–32 V) — convergence-path-dependent, not a clipping-level threshold. Root cause: `emit_nodal`'s per-iteration "global node voltage damping" (`nodal_emitter.rs`, both the primary NR loop and the Backward Euler fallback loop) capped the damping ratio with `.max(0.01)`, so a single NR iteration's LU solve producing a raw voltage delta many orders of magnitude beyond the intended cap (observed 3.8e7 V at a class-AB crossover device-state transition) still let a multi-kV single-iteration jump through (1% of 3.8e7 ≫ the ≤10 V ceiling). The BE-fallback's voltage-step-only convergence check then falsely accepted the resulting nonphysical fixed point (its relative tolerance scales with the already-diverged node voltage). Fixed by removing the `.max(0.01)` floor so the ratio divides uncapped, bounding every iteration's worst-case node step at exactly the intended threshold regardless of raw delta magnitude. All amplitudes now stay within 20–32 V internal; `nr_max_iter_count`/`be_fallback_count` also dropped 10–70× (bad state no longer cascades into subsequent samples). Regression: `nodal_be_fallback_alpha_floor_tests.rs::test_nodal_full_lu_node_damping_has_no_ratio_floor`.
- **BJT forward-active (FA) reduction rule re-check** (raised by melange-circuits 2026-07-25): on wurli-power-amp, 7 of 8 BJTs clear the `Vbc < -0.5 V` FA threshold (`DEBUGGING.md` — device evaluated at Vbc ≈ -20 V) yet all 8 stay full 2D in the shipped codegen. Open question whether the FA rule is still being applied as documented for this circuit, or whether something else (e.g. `--tube-grid-fa`-style override, K-conditioning skip-expansion gate) is suppressing it. Gates a downstream CPU-budget decision in openwurli. Not yet investigated.
- ~~**BJT analogue of `--tube-grid-fa off`**~~ **SHIPPED** (verified 2026-09-22): `--bjt-fa {auto,force,off}` exists on `compile`. `auto` reduces only pure Ebers-Moll BJTs (where the 1-D forward-active model is exact) and is byte-identical to prior codegen; `force` also 1-D-reduces Gummel-Poon / ISE / parasitic BJTs with a per-device warning, dropping the `qb` base-charge term (~1–2 dB under hard drive); `off` keeps every BJT full 2-D. Self-heating BJTs are never force-reduced. This entry was stale.

### Deferred
- **`.switch`/`.pot`/`.runtime R` G-swap first-sample 2× artifact** (root-caused 2026-08-15, raised by melange-circuits/openfarf; user-gated fix). A conductance changed mid-run produces an output at the *swap sample* exactly 2.000× the physical value (drive-independent, deterministic), correct one sample later. Mechanism: trapezoidal puts every conductance in BOTH `A = g+(2/T)C` and `a_neg = (2/T)C−g`, so a switch of Δg adds +Δg to the forward matrix and −Δg to the history matrix; on the swap sample `v[n] = v_prev − 2·A_new⁻¹·Δg·v_prev` — Δg counted twice (`rebuild_matrices` correctly rebuilds `a_neg`; this is inherent to the trap formulation, not staleness). It is a **bug, not a contract** — a resistive divider responds instantly, so the swap sample should be physical; do NOT document it as "undefined." Fix: use the pre-switch conductance in the history term for the one transition sample (one-sample old-g lag on the switched conductance in `a_neg`) — a core-solver change touching every `.switch`/`.pot`/`.runtime R` circuit, needs full golden/SPICE validation. Repro: any `.switch` onto a resistive path, toggle mid-run, compare first-sample deflection to settled ratio. **Second artifact (same event):** the swap also excites a trapezoidal z=−1 Nyquist marginal-stability mode that rings for ~1000 samples (damped by circuit RC; parity-split-into-two-smooth-sequences signature; verified undamped in a no-cap repro, and gone under `--backward-euler`). The old-g lag does NOT kill this mode — it only shrinks the exciting impulse. **Complete fix is two parts:** (a) the one-sample old-g history lag (kills the 2×); (b) breakpoint-style force-BE for 1–2 samples after any `.switch`/`.pot`/`.runtime R` event (damps the Nyquist mode at the source — commercial-SPICE breakpoint practice; the switch-triggered analog of the existing nodal auto-BE). **Third manifestation (reproduced 2026-08-15): PERSISTENT residual on capless switched nodes.** On a purely-resistive (algebraic, no-cap) node the z=−1 mode is *undamped*, so the swap excitation never decays — the two-node residual stays non-zero for as long as you rest in the non-default position (openfarf's g10-ref busbar: 1.44% held-pos1, RC-damped by the chain's 1µF; a no-cap minimal repro shows ±0.32 undamped). Confirmed the trap-marginal mode, not a rebuilt-matrix error: `--backward-euler` held-pos1 residual = −3.5e-13 (consistent), null Δg=0 exact. **Breakpoint-BE is load-bearing** — it fixes both the decaying ring (capped nodes) and this persistent residual (capless nodes); the old-g lag alone does not fix the capless case. Regression oracle: a purely resistive node obeys `node − ratio·other ≈ 0` at all times — grade the two-node residual of a CAPLESS node held in a non-default position (must read ~1e-7), NOT node-vs-DC (a damping cap hides the bug). Workaround (openfarf): fit transients from closure+1; measurements while resting in a non-default position on a capless node are contaminated under trap. See memory `switch_gswap_trap_2x_first_sample_2026_08_15`.
- **G10 divider hard-switching NR overshoot** (root-caused 2026-08-14, user-deferred as non-blocking). Germanium-PNP astable divider (`melange-circuits/local-docs/repro-ic-vcvs-blowup.cir` / `repro-wav-input-blowup.cir`) overshoots to ~80 V on an 8 V rail under a full-strength switching trigger (ground truth from melange-circuits: real full chain swings inside 0–8 V, duty 79.6%). Signature: BE-fallback storm (~88 % of samples vs ~1 % when physical). Root cause: at a switching edge the astable's positive-feedback K coupling pins a junction at v/vt ≈ 300 (v_d ≈ 7.8 V), `i_dev` saturates at `IS·exp(40)` ≈ 7e10 A, the Jacobian goes catastrophically ill-conditioned, and per-sample direct Newton stalls (‖f‖ flat at 7e10, exhausts MAX_ITER=90); the BE fallback inherits the same pinned state. Decisively ruled out by experiment: warm-start predictor, **line-search** (flat region, no descent direction), pnjlim sub-threshold-skip removal, and absolute v_d clamping. **Fix = port the DC-OP continuation (gmin/source stepping, `dc_op.rs`) into the per-sample `solve_nonlinear`** — substantial, higher-risk; validate against 24 SPICE + solver tests + oomox plugin-render golden gate. NOT blocking the working full G10 chain (which converges to rail). Prior "true Newton / Anderson" guess for this class is superseded by the gmin/source-stepping direction. **Acceptance criteria when this lands (from melange-circuits 2026-08-15):** (1) the ~80 V overshoot case stays bounded/physical; (2) NEW — *converged-vs-starved pitch agreement*: on the free-running G10 astable cascade, `--max-iter 70` (~11% NR starvation, which does NOT latch and looks healthy) leaves the master oscillator **3.8% flat** (a third of a semitone) with duty drifting ~3 points vs `--max-iter 1000`, because every non-converged sample leaves a slightly-wrong state that *integrates into pitch* on an oscillator. A failure-fraction warning (see the NR-starvation warning, `fe5c12a`) fundamentally cannot catch this sub-threshold detune class — the continuation is the real fix. See memory `g10_divider_hard_switching_overshoot_2026_08_14`.
- Exact three-winding saturating core (star form `k_ij = c_i·c_j`; physics ruled, not built) — SATURATING_TRANSFORMERS.md §8
- Phase 6a/6b type safety (NodeIdx newtype, field visibility)
- Phase 7 crate split (extract melange-parser, melange-codegen)
- M>24 iterative/sparse NR
- BoyleDiodes heavy-clip Anderson acceleration / BoyleDiodes→ActiveSetBe hybrid (low priority)
- **Sub-sample breakpoint re-solve for glow/oscillator edges — DEFERRAL REVERSED + STAGE A LANDED (`--subsample-fire {auto|on|off}`, 2026-09-07, branch `fix/nodal-schur-nr-convergence`).** The deferral was reversed by internal blind review (NOT the cross-project design review): at the TOP of a divider chain the defect is not aliasing (deferrable on taste) but **injection-lock breakage** — a fixed grid at inner 96 kHz divided only **8/12** Philicorda pitch classes; the other four produced the wrong note. The fix splits each firing sample at the latent crossing fraction `alpha` (`StatefulUpdate.alpha`) into variable-dt sub-steps (`rust_emitter/subsample_fire.rs`): a multi-breakpoint event loop resolving BOTH the strike AND the extinction crossing per firing device in temporal order, plus lit-phase sub-stepping at τ/2 so the lit duration is not grid-quantized. ⚠️ **τ is NOT derived** — `glow_lit_tau_min` = RS × largest terminal-node C-*diagonal*, min over lamps (`subsample_fire.rs:120-145`); a conservative LOWER BOUND on the fastest lit discharge (assumes the terminal self-cap discharges through RS alone), NOT the real loop τ, which the 47k/series caps make 1.8–15× larger (B5 ≈ 21.7µs vs heuristic 1.48µs). Failure direction is COST (finer-than-needed sub-steps, ~14/sample), not accuracy. **Result: 12/12 verdict AGREEMENT with the converged ngspice oracle** — 11 divide 1:1 (incl. E7/G#7/B7 recovered from 8/12), and C8 an AGREED 3:2 non-divide (a5=1.5000, dk 768k/1536k a third arm). C8 is a PRODUCT model-completeness gap (missing B10/B11, TRIGGERTUBE-blocked, pre-registered) — NOT a mechanism exclusion. Nodal route only (DK bakes `S=A⁻¹`, cannot vary dt per fire → refuses `on`; nodal SCHUR sub-path — full-LU inert pending follow-up). `auto`=on for glow-on-nodal is the default (8/12 is a correctness defect). RT-safe (stack scratch, no alloc/lock/syscall) and bounded per sample at 2·n_lamps + LIT_SUBSTEPS_MAX(32) + 2 segments (=44 for the 5-lamp divider; the "≤38" earlier was neither a code constant nor correct) then safe abandon. **STAGE B — DESIGN-REVIEW RE-RULING (2026-09-08; supersedes the internal-review "gate a/gate b" framing):** (1) **CPU** — Speedy reports measured WORST-CASE (all-lit, max-breakpoint) AND steady-state, in µs/board/sample, for BOTH equal-coverage configs (nodal+subsample @96k vs dk @os16), graded against openphilicorda's **1.74 µs/board/sample** bar (their number, they grade); if dk@os16 fits and nodal+subsample does not, Stage B is not on the ship path. (2) **Route guard is NOT a predicate** — `auto` can NEVER force Schur: every nodal full-LU trigger (`nodal_emitter.rs:1740-1757`) is a Schur safety/correctness condition, so the "Schur-safe-but-unused" set is empty. The guard is a route-pinning TEST asserting the shipped divider routes nodal-Schur+subsample from the provenance header, + `{mode,active,reason}` emitted UNCONDITIONALLY for every glow deck (fix the by-presence proxy at `dk_emitter.rs:448`). (3) ⭐ **THE LIVE HOLE is the DK route** — the ACTUAL shipped config (openphilicorda 48k os2 **dk**) has subsample-fire INERT (flag only set on nodal, `ir/mod.rs:3322`) and sits silently at 3/12 with no line saying so; the nodal full-LU revert this originally cited protects a case no shipped deck is in. Cover it with reason `"dk-route"`. (4) The consumer's ON-path **lock** CI test is the hard failure (measurand=lock, not a proxy); `--subsample-fire on` already hard-errors on full-LU (`nodal_emitter.rs:1817`). O(N³)-per-segment factor-reuse decided on Speedy's measured share; NR-wall-hit check. **MECHANISM RULING (cross-project design review, 2026-09-08): mechanism SOUND, `auto`=on the right default — SHIP + re-baseline every glow-on-nodal deck (attribute to strike/extinction quantisation, 8→11).** Structure forced; HALT reasoning confirmed. Corrections: τ is a conservative heuristic not a derivation (above); C8 is the 12th AGREEMENT (above); the "lit-waveform order irrelevant" claim is WRONG — extinction time IS a waveform property (crosses V0+RS·IHOLD, `helpers.rs:952-962`), and first-order BE lengthens each lit phase by ≈x/2 (x=h/τ; 2–12% on this deck, B6 worst) → trap-in-lit is the cheap 2nd-order fix (err ~x²/12). **3 OPEN DEMANDS gate Stage-B closure:** (i) LOCK MARGIN (cents/pitch, `on` vs oracle) under BE-lit vs trap-lit/halved-substep — openphilicorda's lock_margin.py; if margins move toward oracle under trap, build trap-in-lit BEFORE Stage B; (ii) oracle convergence two halvings below envelope (tmax 2.5e-7 & 1.25e-7, SMOOTH_V 0.00025; verdicts+a5..a9 unchanged — openphilicorda owns the oracle); (iii) sub-step factor sweep (τ/2·{1,½,2}) — verdict+margin invariance proves the factor isn't load-bearing. Plus: abandon_count=0 as a CI ASSERTION (an abandoned sample silently reverts to whole-sample latch); split the NR-wall-hit vs abandon counters in reporting (a sub-step wall-hit DOES abandon). SEPARATE from the switch/pot breakpoint-BE above (that is a stability fix for events ON a boundary; this is sub-sample timing for events BETWEEN samples). **✅✅ RESOLVED (2026-09-08) — SKIP trap-in-lit + τ_true; default lit factor 0.5→1.0.** Demand-1 lock-margin sweep (openphilicorda: 4 pitches incl a C#7 CONTROL, factors 2.0/1.0/0.5/0.25, both edges bisected to 5c, lower limit re-run at −800c to un-clamp) is FLAT across the 8× sub-step range on every pitch AND the control (spread ≤4c = at the bisection floor) → **lit-integration accuracy does NOT limit the STATIC lock margin; claim-2 (trap-in-lit) is retired for this deck family.** Readable at face value because the `detected−resolved` "leak" was root-caused BENIGN (dr-debuggenshmirtz: entirely `gridpoint` = extinction crossing ≤1e-3·dt from a segment end; true miss `ceiling+coincident`=0, reproduced on the real divider deck; reason-split counters added). CPU: memoization makes finer ~free (same-rate Schur builds served as bit-identical reuses); **default factor 1.0 = review-derived last tested-safe point (x=h/τ_true envelope was flat 0.017→1.1; 1.0 keeps a heuristic-exact deck at x=1.0), ~36% CPU win (11.28 vs 17.73 µs/board).** Exposed as a diagnostic knob `MELANGE_LIT_FACTOR` recorded in Build:/JSON provenance (`lit_factor`); CLI-flag promotion pending. SCOPE (design review): a divider-family finding (lock fixed by strike timing + threshold-set reset depth, not lit duration) — a future latched device coupled by pulse WIDTH needs this sweep again; NOT "lit waveform never matters". Standing caveat: static-pull margins are an UPPER BOUND on the 6c/s vibrato margin. Remaining (conditional/low-pri): worst-block ship-path CPU (nodal+ssf@1.0 vs dk) for openphilicorda's route decision [measuring]; oracle two-halvings + on-path lock CI DORMANT until the nodal arm ships (their default/compiled route is DK, feature inert there). ⚠️ The "3 OPEN DEMANDS" and "trap-in-lit is the cheap fix" language ABOVE is now historical — superseded by this measured result.

- **Knee re-solve for a railing op-amp into a saturating inductor at 1× — PARKED** (2026-09-28). At 1× with an active-set rail mode, the inductor's internal current overshoots 5-13 % where the op-amp rails into the core (oa_sat choke deck; output H1 within 0.21 %); 4× is accurate (i_L 1.3 %, H1 0.1 %), and compile prints a notice below 4×. The overshoot is born where the core crosses its knee within one sample with the full rail across it (per-sample trace: 1× peaks 6.00 mA against 5.34 at 4×, then alternates at ~−0.5/sample for ~5 samples). Measured and rejected: one backward-Euler sample at each pin-state change, either form (does not reduce it; worse H1); the existing recovery sub-step ladder triggered on the L_diff collapse ratio (catastrophic: fired on the unpinned iterate, the full-step pin re-solve discarded the sub-steps). The collapse ratio is not a sound trigger by itself: a saturating RL at 20 V collapses harder (r 0.012 vs 0.06) and does not ring; the discriminator is the voltage across the core at the knee. **Reopen when** (i) a real deck needs internal-current accuracy at 1× (e.g. an op-amp-driven output transformer where i_L feeds something audible), or (ii) ship-path CPU rules out 4× for such a deck. **Requirements for whoever reopens it:** trigger on the exact per-element stiffness (Z_k from the factored matrix), possibly ANDed with the knee collapse, not the ratio alone; trigger on the committed post-pin solution; each sub-step converged to the main loop's own definition; the pin detected and resolved inside each sub-step; the saturating residual at each sub-step. Prior art (local, not in the repo): the per-element stiffness guard branch, a transition-BE patch and a first knee sub-step patch.
- **Node Gmin moves the fixed point** (opened 2026-09-28). A regularisation must not move the solution; three do. The nodal `G` carries +1e-12 S to ground on every node diagonal (DK's does not); full-LU's Jacobian "Gmin regularization" (+1e-12 on node diagonals, in the matrix but not the RHS) is a second leak at convergence; and a third, unidentified contribution holds the nodal paths at the DC solve's leaky value (the DC operating point itself carries node Gmin). Measured on a 1 MΩ/1 MΩ follower bias divider: gate 12.0000 V on DK, 11.999987 on nodal Schur, 11.999981 on full-LU, 11.999988 in the DC OP; ngspice (no rshunt) 12.0. Paths agree to ~1e-6 at the gate and ~2e-5 at the output, independent of Newton tolerance. At 10–100 MΩ grid leaks or piezo/electret nodes this becomes an audible-scale bias error. Target: every path equal to ngspice and DK/nodal agreement to 1e-9; either apply the regularisation in residual form or compensate the RHS with Gmin·v_iter so it cancels at convergence, and delete the baked `G` term if nodal does not need it. Interim tripwire: `mosfet_body_effect_tests.rs::follower_paths_agree_within_the_node_gmin_tripwire`. Scheduled after the deep-saturation stiffness work.

## Cross-Compilation (macOS from Linux)

Zig 0.13 + cargo-zigbuild + macOS SDK 13.3 + rcodesign (ad-hoc signing).
`cargo zigbuild --release --target universal2-apple-darwin` produces universal Mac binaries.
melange-cli does NOT cross-compile (ureq/dirs need CoreFoundation), but generated plugins do.

---

# v0.1.5 SHIPPED 2026-09-03 (tag `v0.1.5` -> `d641457`)

Everything in this section shipped in v0.1.5. Not DSP-byte-identical to 0.1.4:
generated source moves for essentially every deck (F9 rewrites the nodal
`reset()` body; DK decks get the new `N_I` layout), but rendered audio moves
only on noise-enabled decks hitting a sub-step path (194 of 196 golden renders
identical). **F9 cannot move `simulate`/`analyze` output** — neither calls the
generated `reset()`. Downstream re-verify is ~2 decks by output, all nodal decks
by generated source.

v0.1.5 also closed a version-reporting gap: `main` had been advanced past the
`v0.1.4` tag without a bump, so builds from `main` reported a 0.1.4 they were
not.

# 2026-09-02 — verification-instrument repairs, and what they cost

Three of melange's verification instruments were found to have integrity defects
in one day. **None was found by the instruments themselves.** Each measured
something real and was *believed* to be measuring something else.

| instrument | defect | fixed |
|---|---|---|
| golden gate | renders were **f32** (hiding the entire `-ffp-contract` class the C++ numerics contract exists to prevent); `compare` never diffed `circuit.rs`; **zero** `diag_*` counters recorded | `847d91f`, `867fd18`, `671a575` |
| SPICE validate | built a **different circuit** than compile ships — N=44/M=16 vs N=20/M=14 on the shipped power amp | `6bc3ef1` |
| `DC_BLOCK_CUTOFF_HZ` | one tuned constant duplicated at **six** sites (plan said one, review said five) | `ddd29c1` |

A refactor could have deleted the BE latch, NaN reset and active-set resolve and
kept the golden gate green.

## Golden corpus

**46 → 35 → 38 decks.** Eleven were dropped when melange-circuits pruned their
netlists (`cf8a04c`); the maintainer confirmed all eleven were in-progress or
abandoned. They were **not** vendored back in: gating on an abandoned circuit
freezes its pathology as the specification, so a legitimate solver fix later
reads as a regression.

Three replacements chosen **by measurement**, not reputation — compiled, driven
with a hot sweep, `diag_*` counters read off the run:

| deck | substep | ls_fail | be_fallback | nr_max |
|---|---|---|---|---|
| steve-1073-preamp | 490 | 12412 | 896 | 1386 |
| wurli-power-amp | 427 | 8603 | 0 | 427 |
| gravity | 0 | 926 | 0 | 0 |
| *(dropped tungsten-thunder-horse, for scale)* | 53 | 5627 | — | — |

**38 decks now exercise more recovery ladders than 46 did.** Only
`diag_voltage_damp_count` is still thinned (7 → 5).

⚠ **What this corpus is:** a CHANGE DETECTOR — it compares melange against
melange, so deck quality is irrelevant to catching a refactor that moves output.
It is **not** an accuracy oracle. Note the asymmetry: *agreement* is robust to
deck quality (a bad circuit cannot manufacture agreement between two
implementations); *disagreement* is not. A broken deck may **detect** a defect;
it can never **be** the evidence for one.

## Coverage closed

* **MOSFET / JFET** — were implemented and SPICE-validated with **zero** circuits
  anywhere. Three decks added (`150bda6`). The MOSFET pair covers both solver
  families. Body effect is evaluated at the **live Newton iterate on every
  path** — DK and nodal Schur from `v = v_pred + S_NI·i_nl`, full-LU from the
  node iterate — with gmb = gm·dVT/dVsb in the Jacobian, and the DC operating
  point the same way. Taking Vsb from `v_pred` alone (the old DK/Schur form)
  leaves out the device's own current; in a follower that current sets the
  source, and the stage settled 1 V high, 8 % hot on H1. Pinned against
  ngspice by `mosfet_body_effect_tests.rs`.
* **Pentode ngspice validation** (`a62ddad`) — 10 library decks / 4 corpus decks
  had no oracle at all.

## Still open

* Validate applies **no forward-active reduction** (residual 0.246% on
  wurli-power-amp is a candidate).
* **Per-timestep junction-charge re-linearization** — blocks `TR`, would make
  `TF` exact. Build charge-first; see `DEVICE_MODELS.md`.
* **Multi-input** is CLI-restricted to linear (`M=0`) circuits, which is exactly
  the case superposition already covers — so the nonlinear-mixing case it exists
  for is unreachable, and no deck uses it.
* Schur NR diverges on expanded parasitic internal nodes where the same circuit
  converges unexpanded (or expanded on full-LU). **Latent** — the CLI's K-gate
  never constructs that combination; measured **0** corpus decks in that state.
