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
- **Not implemented**: temperature coefficients on resistors (TC1/TC2), op-amp `EN_FC`/`IN_FC` 1/f corner (accepted with a compile notice; Phase 4 is white-band only in v1), DK-path BJT parasitic-R (rbb′) thermal noise (nodal only), diode `RS` / tube `RGI` thermal noise, tube microphonics (Phase 6), 6386/6BA6/6BC8 datasheet fits for varimu compressors (phase 1d deferred). All five noise phases (thermal, shot, junction+resistor flicker, op-amp en/in, pentode partition) ARE shipped — see "Circuit Noise" under Feature Inventory.
- **Known model limitations**:
  - Diode BV: exponential reverse breakdown (matches codegen template), evaluated in both codegen and DC OP solver (FIXED 2026-04-15)
  - DC OP diode Gmin: 1e-12 S minimum junction conductance added to prevent zero Jacobian entries at reverse bias (FIXED 2026-04-15)
  - DC OP op-amp AOL capped at 1000 to prevent multi-equilibrium NR instability in precision rectifier circuits (FIXED 2026-04-15)
  - DC OP input ports: the DC OP solves the `mna.g` the build stamped (every input port's conductance counted once, as the transient counts it); `DcOpConfig` has no input fields. The reported KCL residual is against the circuit's own G, not the working copy with its solver aids, so it carries the node Gmin leak (~1e-12·|v| A). Witness: a divider tapped through a 1 MΩ port matches ngspice (v(in) 2.992519 V) in the baked `DC_OP`, `recompute_dc_op` and `melange dc-op` (FIXED 2026-09-29; the compile-time DC OP and `dc-op` had counted the port twice, v(in) 1.9934 V)
  - DC OP failed convergence: low-rate warmup (200 Hz × 1000 samples = 5s circuit time) charges coupling caps before transient NR. Settled state cached for `reset()`. 4kbuscomp: BE fallback <1%, stable at all amplitudes. (ADDED 2026-04-16)
  - BJT GP Q1: singularity guard at `q1_denom <= 0` (physically near Early voltage limit)
  - Tube Koren: no space-charge, no transit-time effects
  - JFET/MOSFET subthreshold: hardcoded 2×VT slope (real devices: 60-120 mV/decade)
  - VCA noise_floor field exists but unused
  - Precision rectifier transient: VCCS back-substitution contamination at cap-only nodes downstream of high-AOL op-amps. Fixed via selective Gm cap on op-amps matching Rule D' (n_plus on non-zero DC rail AND diode connects output→inverting input through R-only path). 4kbuscomp `max_abs_v_prev`: 1.18B → 15V. User override via `.model OA(AOL_TRANSIENT_CAP=N)`. Klon and other working circuits unaffected (Rule D' correctly excludes them). (FIXED 2026-04-16) **The automatic cap was removed 2026-09-29** (unneeded under the charge form and active-set pinning, and it moved the answer by 4.5 mV on a biased rectifier); `AOL_TRANSIENT_CAP` remains an author's key and routes nodal.

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
| 4-opamp + diode clipper | 44 | 10 | Nodal full LU (auto) | — | Clean clipping verified under ActiveSetBe; auto now resolves ActiveSet, not re-run on this deck; BoyleDiodes diverges at heavy clip |
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
- **Integrator: trapezoidal in the charge (companion) form** on every generated path (DK and nodal; Schur, full-LU, adaptive sub-steps, sub-sample fire): `A·x_{n+1} − N_i·i_nl(x_{n+1}) = RHS_CONST + αC·x_n + q_dot_n + b_{n+1}`, with the capacitor currents `q_dot = C·ẋ` carried as state (`pub q_dot` on trapezoidal builds) and every source (DC, input, `.inject`, `.runtime`, noise) entering once, at n+1. KCL of a committed sample is that sample's own solve residual — no `z = −1` walk on capless rows. Backward Euler (fallback, breakpoint-BE, transition-BE, BE-latch, auto-BE promotion, `--backward-euler`) has the same shape with `C/T` and no `q_dot`. `RHS_CONST` is ×1 on every row under both. Full reference: [COMPANION_MODELS.md](COMPANION_MODELS.md) "Charge (Companion) Form"
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
- **BJT**: Gummel-Poon (VAF/VAR/IKF/IKR, CJE/CJC, NF/ISE/NE) matching ngspice `bjtload.c` line-for-line; Ebers-Moll fallback; device temperature `TAMB` (SPICE `.temp`: IS/BF/BR/ISE/ISC/VT scaled from TNOM, ngspice-gated); self-heating (RTH/CTH/TAMB); RB/RC/RE parasitic R
- **JFET/MOSFET**: 2D Shichman-Hodges / Level 1; CGS/CGD junction caps; RD/RS parasitic R; MOSFET body effect (GAMMA/PHI)
- **Diode**: Shockley + RS + CJO + BV/IBV Zener; device temperature `TAMB` (SPICE `.temp`: IS and N·VT scaled from TNOM, ngspice-gated); optional self-heating (RTH/CTH/XTI/EG/TAMB) using the same quasi-static electrothermal model as BJT, with `IS(T) = IS_amb·(Tj/TAMB)^(XTI/N)·exp(EG/(N·VT_amb)·(1−TAMB/Tj))` and `N·VT(T) = (N·VT)_amb·(Tj/TAMB)`. Pipe-shouter (TS-808) uses RTH=500 CTH=2e-4 on the 1N4148 clippers; sad-bastard uses RTH=1200 CTH=1e-4 EG=0.67 on the 1N34A Ge clippers. Dead code when RTH=∞ (default).
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
- **Calibration (all phases)**: every stamp is the PHYSICAL noise current at n+1, one draw per source per sample, under both integrators (the charge form enters each source once; `A − A_neg = G`). The trapezoidal integrator's own `(1 + z⁻¹)` nulls Nyquist on the charge-carrying rows; capless rows carry no history. No lag state (`*_w_prev`) exists. The BE-fallback replay of the cached currents is exact. See [NOISE.md](NOISE.md) "Constant derivation".
- **Thermal (Phase 1)**: Johnson-Nyquist on every fixed R + every dynamic R (`.pot`/`.wiper`/`.runtime R`/`.switch` R). Per-sample `sqrt(2·k_B·T·fs)·sqrt(1/R)·N(0,1)`. Shipped on both DK and nodal codegen paths via the shared `build_noise_emission()`.
- **Shot (Phase 2)**: per-junction stamp, amplitude `sqrt(Γ²)·sqrt(q·|I_prev|·fs)` from one-sample-lagged `state.i_nl_prev` (Γ²=1 for plain junctions). Diode 1 src, BJT 2 (1 when forward-active-reduced, **plus** a base-shot src Γ²=1/BF), JFET/MOSFET 1, Tube 1 — triode plate **space-charge smoothed** (Γ²=10·k·T₀·gm/(2·q·I_p) at the DC OP; `SHOT_GAMMA2=` override, 1.0 restores bare shot). Pentode plate → Phase 5 partition, not bare shot. VCA/op-amp skipped.
- **Flicker (Phase 3 junction + Phase 3.5 resistor)**: per-junction and per-resistor 1/f via Paul Kellett 7-pole pink cascade. **fs/OS-invariant** calibration (recalibrated 2026-07-18): white input `sqrt(0.5·KF/K_pink)·|I|^(AF/2)` (K_pink≈6e-3 analytic; the ×0.11 tail is NOT unit-gain — K_pink sets the level), one draw per sample. Old `sqrt(4·KF·|I|^AF·fs)` was ~+30 dB hot at 96 k and fs-dependent. Junctions opt-in `.model NAME TYPE(KF=… AF=…)`, AF default 1.0; resistors opt-in per-element `R1 a b 10k KF=… AF=…`, Hooge bias-squared, AF default 2.0 (unbiased R → thermal only). KF=0 (default) → byte-identical. Shared `set_flicker_gain`.
- **Partition (Phase 5)**: pentode plate stamp `sqrt(q·I_p·I_s/(I_p+I_s)·fs)·PARTITION_F` **replaces** bare plate shot (reuses `shot_gain`/`set_shot_gain`). `PARTITION_F` default 1.0 (process knob; ~0.6 for selected low-noise EF86). Triode-only / passive circuits emit zero partition codegen.
- **Op-amp en/in (Phase 4, v1 white-only)**: three Norton streams via `.model NAME OA(EN=… IN=…)` — en at in+ (`EN·noise_opamp_en_g_diag·sqrt(0.5·fs)`), in at in+ and in- (`IN·sqrt(0.5·fs)`). Runtime `set_opamp_input_gain` (signal-independent, distinct from `shot_gain`). `EN_FC`/`IN_FC` are accepted with a compile notice and **not modelled** in v1. Op-amps without EN/IN → byte-identical.
- Runtime: `set_noise_enabled(bool)`, `set_noise_gain`, `set_thermal_gain`/`set_shot_gain`/`set_flicker_gain`/`set_opamp_input_gain`, `set_temperature_k(K)` (290 K default; only thermal scales with T), `set_seed(u64)` (0 → entropy from system clock, nonzero → deterministic). Each method emitted only when its mechanism is present. Salted per-phase streams so thermal/shot/flicker/partition/op-amp never share a prefix under one master seed.
- Calibration validated by kTC theorem (`tests/noise_psd_validation.rs`): `V²_rms = k_B·T/C` ±15% on a 10 kΩ / 100 nF RC, and on an RC + diode compiled both trapezoidal and BE at 96 kHz (measured 3.888e-14 V² each vs kT/C 4.004e-14 V²). Nyquist regression: `thermal_noise_no_nyquist_artifact_on_resistor_only_output_node` asserts lag-1 > -0.5 AND RMS < 100 µV on a diode + series-R circuit. Full reference: [NOISE.md](NOISE.md).
- **BJT parasitic-R thermal (2026-07-18)**: `rbb′`/RC/RE thermal noise collected on the **nodal** path only (real internal-node injection); **skipped on the DK path** (no node pair for the Norton stamp) with a `log::warn!`. Route `--solver nodal` to include it. Diode `RS` / tube `RGI` are still not thermal-noise sources.
- The per-phase detail above is synced to [NOISE.md](NOISE.md) (fs-invariant flicker, triode space-charge smoothing, FA base shot, Phase 4/5, charge-form one-draw calibration). [NOISE.md](NOISE.md) remains the authoritative reference for derivations and validation. User-facing guide: [../NOISE_GUIDE.md](../NOISE_GUIDE.md).

### Codegen Infrastructure
- **DK codegen** with augmented MNA (≤1 transformer group, M<10, K well-conditioned)
- **Nodal Schur** (medium complexity), **Nodal full LU** (K≈0 / positive K / ill-cond K or S)
- Full-LU optimizations stacked: chord method + cross-timestep Jacobian persistence + compile-time sparse LU (AMD ordering, symbolic factorization)
- Oversampling 2x/4x: self-contained polyphase half-band IIR, no runtime dependencies
- `--solver {auto|dk|nodal}`, `--backward-euler`, `--oversampling {1,2,4}`, `--opamp-rail-mode`
- **Runtime BE-latch (2026-07-28; entry re-derived 2026-09-28)**: nodal trapezoidal builds carry a cheap input-aware lag-1 detector on the mean-removed output. For one mode x = A·zⁿ its ratio is z; for a mixture it is the power-weighted mean of the components' factors. It engages at ratio ≤ −exp(−α), α = 1/(τ·fs) the estimator's forgetting rate at the internal rate: an alternating mode that outlives the window and carries the output. An alternation inside the solver's node tolerance (1e-3·|v| + 1e-6 V) is not evidence, and neither is one below −60 dB of the program that excited it (2026-09-29): the floor is the larger of the node tolerance and 1e-3 × a program reference, passband gain × input amplitude, remembered as long as the slowest Nyquist-side ring the linearised circuit carries at the running rate (held on index-2 circuits). The latch and the compile-time ring predicate share one threshold ([RING_PREDICATE.md](RING_PREDICATE.md)); before, the latch overrode the predicate at the first quiet moment after a transient. The entry threshold used to be a tuned −0.6, which latched a mastering deck permanently on a 2-sample impulse tail (its ratio now bottoms at −0.32 against −0.99). At 4× the threshold sits closer to −1 per internal sample; an open-secondary leakage ring that trap nearly damps unaided at 4× latches late or not at all, which is the criterion working. If the solver falls into a self-sustaining Nyquist `(-1)^n` limit cycle at a large-signal operating point (which the compile-time quiescent-OP auto-BE promotion can't see — jeffreys-tube V2 class), it latches that instance to the L-stable BE path for the rest of the stream (cleared by `reset()`, exposed via `diag_be_latch_count`). Not emitted for BE/force-trap/passive builds. Emitted for saturating-inductor circuits, including M = 0 ones, since 2026-09-28. On both nodal sub-paths a latched sample skips the trapezoidal solve and runs the same backward-Euler routine a `--backward-euler` build of that sub-path runs (full-LU: main loop, sub-step, pin; Schur: M-dim Newton, pin), bit-identically from the same state. On a core with no air-core floor (`LAIR=0`) it catches the deep-saturation trapezoidal ring (inductor current 20 % over the V/R ceiling at 20× Isat; 4× oversampling does not cure it); with a floor (default 3e-4 of L0) the saturating RL at 10-20 V and the choke-loaded common-source stage at 1-30 V do not ring and it does not fire. It still fires on a step into an open saturating transformer (golden `sat-core-open/step`), and that ring is not saturation: with `ISAT=100` (core linear throughout) it latches identically, with a 600 Ω load it does not (measured 2026-09-28; the open secondary's stiff leakage-into-1 MΩ mode, see SATURATING_TRANSFORMERS.md §3.4). **Cost, because the latch is sticky:** a ring that is really a transient commits that instance to BE for the rest of the stream; measured with `LAIR=0`, H1 −1.8e-4 (saturating RL at 10 V) and output −0.28 % (choke-loaded stage at 5 V). Release is the reopened item under Pending Work.
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
- **Nodal-Schur exact Newton start (2026-09-29).** The Schur Newton now starts at full-LU's point (`v_prev`) instead of extrapolating device currents. On smooth signals the old predictor was an O(h²) start that converged on the first check; the exact start is O(h) and costs about one extra Newton iteration per sample. Where the old predictor started badly (singular-`K` decks, railing op-amps) it is faster. Measured same session, 7950X, 0.3 V 440 Hz at 48 kHz, `-C target-cpu=native`, ns/sample old → new (mean Newton iterations): pipe-shouter 3344 → 1573 (5.30 → 1.49); opamp-pin-audio 4199 → 3185; opamp-pin-control 7799 → 6234; tungsten-glow 641 → 1160 (0 → 0.96); vurli 194 → 312; noyce-tape-head 189 → 299; gold-press-mastering 1071 → 1567; velvet-elvis 1069 → 1272; qapla-1a 1526 → 1645; five-watt-freddie 1627 → 1746. README rows on nodal Schur, same driver: Wurlitzer preamp 687 → 898 (+31 %), tweed 5F1 1611 → 1725 (+7 %), passive EQ 1490 → 1625 (+9 %). **Those README rows are re-measured with `bench.sh` on an idle box at release.**
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
- **4-op-amp overdrive with diode clipper**: verified bounded under ActiveSetBe at amp=[0.01..0.50]; auto now resolves ActiveSet, not re-run on this deck. BoyleDiodes opt-in only (heavy-clip divergence at amp ≥ 0.05 unsolved — not a blocker, see DEBUGGING.md).
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

- **Runtime BE-latch is sticky — REOPENED 2026-09-28.** Once latched, a stream stays on backward Euler until `reset()`, and on saturating circuits that costs harmonic accuracy for the rest of the session. Measured on a smooth witness (1 kHz sine through 1 kΩ into a 100 mH choke, ISAT 2 mA, `CORE=gapped`, no op-amp; ngspice twin converged to 1e-6), error vs ngspice:

  | | 2 V H1 | 2 V H3 | 5 V H1 | 5 V H3 |
  |---|---|---|---|---|
  | 1× trapezoidal | +0.115 % | +0.40 % | +0.095 % | +0.76 % |
  | 1× latched | −1.96 % | −15.0 % | −0.44 % | −5.65 % |
  | 4× trapezoidal | +0.007 % | +0.025 % | +0.006 % | +0.047 % |
  | 4× latched | −0.49 % | −4.1 % | −0.10 % | −1.35 % |

  Exposure is the deciding factor, not cost: golden renders are ~2 s of clean programs, and a plugin streams for hours through level jumps, transport starts, pot moves and bypass toggles, where one fire is permanent. **Step 1 (measurement, no design yet):** drive every latch-emitting saturating deck plus sat-core-open (control) with 60 s of hostile-but-realistic material (level steps −40 → 0 dBFS, transport start from silence, a pot sweep while playing, an impulse train; 1× and 4×). Report fires, time to first fire, and cause (stiff ring vs transient). **Step 1 result (2026-09-28):** 9 latch-emitting golden decks, 1× and 4×. Two fired. **sat-core-open** (control) fired at the transport stop, 35.06 s. With the latch disabled, the detector's condition then holds for 237,078 consecutive samples (~4.9 s): a genuine sustained ring from the open-secondary stiff mode. **gold-press-mastering** fired at 46.86 s, on one impulse of a 10 Hz train. The condition held for 2 samples, never again in 60 s: a false fire, which the sticky latch turned into backward Euler for the rest of the stream. The seven others never fired. So entry needs evidence of persistence too, not only release. **Constraints for any design:** release on evidence (the detector reporting quiet over a window), not a bare sample count; the first trapezoidal sample after release goes through the breakpoint path; a chatter witness (swaps per second on a deck sitting at the detector's boundary).
- **BE-latch release — specified, PARKED (2026-09-28): no transient-stiffness witness exists.** Entry was re-derived (`ddd4fbf`). Release shape (A): once per window while latched, estimate the trapezoidal factor of the ring's own mode at the CURRENT linearization, M = (G_eff + αC)⁻¹(αC − G_eff), by power iteration from the stored ring shape d; release iff that factor is above the entry threshold. **Build it with the consistent-subspace projection, not the naive iteration:** power iteration converges to the largest |z|, which is the exact z = −1 algebraic family in null(C). d carries walk components by construction, so a naive probe never releases on any deck with a capless combination (every T-model deck). Project d once onto the consistent subspace, d' = d + U·y with (Uᵀ G U)·y = −Uᵀ G·d (U a basis of null(C), from `c_work` where caps are runtime), and re-project after each iteration. The probe then sees only reactive modes, and its unit test must reproduce the analytic trap factors of the sat-core-open leakage mode (48 kHz: −0.99619 unsaturated, −0.99803 deep saturation; 192 kHz: −0.98483 / −0.99213). Measured: the choke witness (sine into 1 kΩ + 100 mH, ISAT 2 mA, `CORE=gapped`) never fires entry at 5–50 V, 100 Hz–1 kHz, 1× and 4×, because the air floor removes its ring. sat-core-open's 4× fire is the algebraic walk (the entry condition holds ~5 s; the leakage mode at 4× decays in 0.34 ms). sat-core-open's 1× fire is its linear leakage mode (−0.996 / −0.998), which (A) would never release, correctly. **Reopen trigger:** a deck whose ring comes and goes with the operating point, re-measured after the charge form removes the walks.
- **L-stable integrator (BDF2 / TR-BDF2) — reopen evidence recorded (2026-09-28), not scheduled.** A permanently stiff linear mode (sat-core-open's open-secondary leakage mode, trap factor −0.996 / −0.998 at 48 kHz) keeps the runtime latch engaged for the rest of the stream once excited, and a latched stream costs up to 15 % on H3 at 1× (table above). An integrator that damps stiff modes without BE's first-order error is the remedy, not a latch release. Re-measure the latch's firing set after the charge form: the case is "a latch that fires only on permanently stiff linear modes". Second deck (2026-09-29): noyce-transformer-triode keeps trapezoidal under the ring predicate (single-event ring −63 dB, 45× better in-band sine error than BE) but its stiff mode (z = −0.99997, τ 0.7 s) accumulates under phase-coherent even-period clicks to −46 dB, and in the default build the latch engages 9 ms after the first impulse of the 0.1 V hostile program and holds BE for the rest of the stream.
- **Capless-row walk of accepted Newton residuals — removed by the charge form** (opened 2026-09-28, closed 2026-09-29). On a row with no capacitor (and on combinations whose capacitor currents cancel, e.g. `KCL(a)+KCL(b)` across a coupling cap), the whole-system trapezoidal form enforces `g(x_(n+1)) = -g(x_n) + e_n`, so every sample's accepted residual `e_n` adds, alternating in sign, until a backward-Euler sample resets it. The generated integrator is now the charge (companion) form ([COMPANION_MODELS.md](COMPANION_MODELS.md)), under which a committed sample's KCL residual is its own solve residual. Measured: cap-coupled diode witness `KCL(a)+KCL(b)` 1.31–1.52 µA → ≤ 0.011 µA at 96 kHz, 0.43–0.52 → ≤ 0.005 µA at 192 kHz; the op-amp overdrive deck with the transition-BE sample removed sits at the floor (0.29–0.43 µA at 48/96/192 kHz; whole-system 105–2650 µA), and at 1 kHz 0.5 V its 48 kHz op-amp output is 1.1 mV RMS from a 768 kHz render (whole-system 0.49 V). Golden corpus (188 renders): 107 bit-identical (BE builds), linear decks within 1e-13, 7 changed, every change traced to the walk (Nyquist limit cycles at rest now decay to ~1e-17; a DK pot sweep's 2.8 mV Nyquist alternation gone; a saturating-choke stage that drifted 1.2–1.4 mV per 0.1 s settles to 3e-9 V); 0 marginal Newton renders (a preamp deck had 3). The measurements below were taken under the whole-system form. The rows involved are the diode node of a clipper behind an op-amp's output cap, and any capless nonlinear node. No per-sample tolerance bounds the sum. On the overdrive deck in `OPAMP_RAIL_MODES.md`, with transition-BE resetting it at every pin and release, it is bounded: flat per-second maxima over 30 s, at most 1.9 µA, inside the main loop's own row tolerance (~3–5 µA). Two injection sites are measured:
  - **The pinned resolve** (both sub-paths, rail plateaus): it accepts on the node step (`1e-3·|v| + 1e-6`), so the committed pair is off KCL by about `½·g′·dv²`, ~0.05 µA per sample.
  - **The full-LU main loop in fast free swings — FIXED 2026-09-28.** A chord (reused Jacobian) step leaves a KCL residual `(J − J_chord)·Δ`, first order in the step: up to 2.2 µA per sample, invisible to the node-step test because a 0.64 mV node tolerance on a stiff diode row is ~45 µA of current. A chord-accepted iterate whose max node-row residual exceeds the row test's 1e-9 A floor now takes one refactored Newton step; full-LU matches the Schur sub-path in every measured cell. Measured and rejected: a simplified-Newton stopping rule on the chord's contraction rate (no effect: the accepted steps were already 12× inside the node tolerance), a tighter chord-refresh threshold (a tuned constant), refactoring every iteration (up to 2×), and an ungated exit step (2–3.4× at silence).
  - **Tolerance units, for the one-definition work:** node steps are tested in volts, residuals in amps. On high-conductance rows the volt tolerance is loose in current terms; a current-residual acceptance is the other principled form (a larger change, not taken).
  - **Structural item: the companion (charge) form — implemented (2026-09-29).** It was upstream of five open decisions: the latch release, the BDF2 case, the retirement re-measures for transition-BE, pot/switch breakpoint-BE and the gated exit step, the pinned-resolve residual, and the g10 wander. SPICE applies trapezoidal integration to charge and flux states only, so a capless row has no memory; melange's generated integrator now does the same. It does not remove trap ringing on the reactive states themselves. melange has no nonlinear junction charge today (diode `CJO` is constant; BJT `CJE`/`CJC` are linearized at the DC operating point); the saturating inductor's flux row carries `Φ(i)` in both the RHS history and the `q_dot` update.
- **Re-measure under the charge form (opened 2026-09-29).** Each mechanism below still emitted is unchanged; whether each still earns its keep now that the walk is gone is unmeasured. Do not remove any without its own measurement:
  - ~~**Transition-BE**~~ **RETIRED (2026-09-29).** Without it the diode-node residual sits at the floor with zero BE samples, and the waveform error against the ngspice twin (1001.3 Hz, unaligned RMS) is lower in every cell: 0.1 V 2.51 / 0.61 / 0.21 mV vs 3.20 / 0.84 / 0.29 mV, 0.5 V 9.74 / 3.55 / 0.85 mV vs 10.91 / 10.09 / 0.91 mV (48 / 96 / 192k). The 0.5 V worst-case per-cycle peak favoured it at 48k/96k (1.43 / 0.43 % vs 2.17 / 0.81 %) through a 0.2–0.35-sample timing shift; see `OPAMP_RAIL_MODES.md`.
  - **Pot/switch breakpoint-BE**: the whole-system swap-sample 2×Δg does not exist under the charge form (history carries no `G`; pot setters do not touch `A_neg`). Re-measure the capless-node two-node residual oracle and the first-sample deflection with and without it.
  - **The gated chord exit step** (full-LU, one refactored Newton step when a chord-accepted iterate's node-row residual exceeds 1e-9 A): its reason was the per-sample residual feeding the walk.
  - **The runtime BE-latch**: re-measure its firing set (sat-core-open 4× fire was the algebraic walk); feeds the latch-release and BDF2 items above.
  - ~~**The stability discriminators' operator**~~ **DONE (2026-09-29).** Auto-BE promotion is the ring predicate on the charge propagator linearised at the DC OP ([RING_PREDICATE.md](RING_PREDICATE.md)); 15 corpus decks return to trapezoidal, one is newly promoted. `spectral_radius_s_aneg` (the Schur-versus-full-LU input) and the router's DK-kernel estimate still use the whole-system operator; neither decides the integrator.
- **The ring verdict is taken at the compiled rate** (2026-09-29). A host rate above it is conservative (stiff rings decay faster, residues shrink); below it slightly optimistic. The latch's memory follows `set_sample_rate`; the route does not. Open: whether a runtime rate change should re-evaluate the verdict (the stored continuous poles make `|z|` cheap; the residue moves too) and flip the route through the BE machinery the latch already uses. [RING_PREDICATE.md](RING_PREDICATE.md).
- **Index-2 decks (an exact `z = −1` under trapezoidal: champ-5f1, noyce-smps-ripple, sat-core-loaded, wurli-power-amp)** — an analog-EE modelling question, not a solver one: an inductor-only cutset that trap never damps suggests the netlist idealises away a physical parasitic (winding capacitance, core loss). Until decided, the runtime latch holds its program reference on these decks, so a ring a quiet passage excites long after a loud one under-latches (conservative). [RING_PREDICATE.md](RING_PREDICATE.md).
- ~~**Thermal voltage 300.00 K vs SPICE's 300.15 K**~~ **DONE (2026-09-29).** `VT_ROOM` is kT/q at SPICE's TNOM, 27 °C (300.15 K), for diodes, BJTs, JFETs and MOSFETs, matching the self-heating `TAMB` default.
- **DK has no runtime ring latch** (2026-09-29, documentation, not work). The runtime BE-latch is nodal-only. The compile-time ring predicate covers DK builds too, and the corpus DK decks returned to trapezoidal (gold-press-riaa, noyce-ef86, noyce-smps-ripple, noyce-triode-12ax7) sit more than 100 dB under its −60 dB threshold on the 60 s hostile program, so nothing depends on it today.
- **Self-starting oscillators stay off DK (2026-09-29).** A DK build whose DC operating point has a growing pole (trapezoidal spectral radius > `TRAP_BE_PROMOTION_RHO` — under the charge form exactly a right-half-plane pole of the DC-OP-linearised system) is refused (`CodegenError::SelfStartingOscillator`): the auto route rebuilds on nodal and says why, a forced `--solver dk` fails with the reason. Scope: self-starting only. A kick-started or driven regenerative circuit (an astable seeded by `IC=`, a flip-flop) has a stable DC operating point and is caught at runtime by `diag_unsolved_sample_count` (present on every build; refused by `simulate`/`validate`/golden), not here. **External dependency:** the runtime half of that coverage is a plugin test asserting the counters stay zero. As of 2026-09-29 oomox reports 15 circuit plugins that assert no solver-health counter at all, so for those plugins the runtime half does not exist yet (oomox is adding the assertion deck by deck at regeneration). Corpus: no deck has such a pole; the G10 master oscillator fixture (ρ 1.0149 at 48 kHz) is the witness (`cli_integration::test_g10_self_starting_oscillator_is_refused_on_dk`).
- ~~**Four build assemblies disagreed**~~ **UNIFIED (2026-09-29).** `melange compile`, `simulate`, `analyze` and `melange-validate` each assembled the `pipeline.rs` steps in their own sequence, so a verb other than `compile` could check a different circuit than `compile` ships. All four now call `melange_solver::build::build`; what a verb legitimately varies is a field of `BuildOptions`. The disagreements, each measured before/after on the 85-deck local corpus (`MELANGE_DUMP_SOURCE`) and attributed: the DK-kernel failure fallback (now the augmented kernel everywhere; no generated-code change); the Newton budget (`simulate`/`analyze`/validate tuned it for the trapezoidal rule on `.integrator be` decks: `MAX_ITER` 250 where `compile` ships 100); `.inject` (`analyze` and validate left the source impedance out; on wurli-preamp-okona that left the drive node unterminated and changed its route); and junction caps on `.linearize` decks (below). The validate suite's verdicts and printed metrics are unchanged (139/139).
  **⚠️ Validate verdicts on `.linearize` decks from 6bc3ef1 (2026-09-02) until this unification were measured on DOUBLED junction caps:** the `.linearize` rebuild re-stamps the caps, then validate's own preflight stamped them again (measured on wurli-power-amp: every differing C entry is exactly one extra zero-bias CJE/CJC from its `.model` cards). Re-measured as shipped: wurli-power-amp 0.0924 % RMS, correlation 0.9999996 (was 0.0947 % with doubled caps), still over the 2e-2 V peak tolerance at 3.3e-2 V.
- **Reductions are off by default and fail loud (2026-09-29).** The forward-active BJT reduction checked its premise (the junction never leaves forward-active) once, at the DC operating point; a common-emitter stage driven with 50 mV saturated and the reduced build put its output on the -10 V clamp (ngspice -4.152 V, full model -4.13 V), with `diag_region_exit_count` the only trace. `--bjt-fa` now defaults to `off`; a region exit on a REDUCED device (forward-active BJT, grid-off pentode) counts as an unsolved sample (`diag_reduced_model_exit_count`) and every verb refuses it. `diag_region_exit_count` stays as characterization on full models. Witness: `golden_ref_tests::bjt_ce_with_forward_active_reduction_counts_its_saturation_unsolved`. Exposure found: none (no corpus deck and no openwurli/oomox generated file used the reduction).
- **Test builders now build what ships (2026-09-29).** The solver tests' `support` helpers generated code from the raw MNA (no junction caps, no reductions or `.linearize`, no cap preflight, no routing gates, the default Newton budget), so they tested a circuit that does not ship. They now call `melange_solver::build::build`. `config_for_spice` refuses a deck with no `in` node instead of taking node 0 (on several fixtures a supply rail). Fixture defects this surfaced were fixed (ideal sources on the input, inert supply sources, inputs on supply rails).
- **Companion-inductor codegen deleted (2026-09-29).** No shipped build reached it: the build makes every inductor kernel with `DkKernel::from_mna_augmented`, so the codegen for companion-modelled inductors (IR inductor/coupled/transformer lists, `IND_*`/`CI_*`/`XFMR_*` constants and state, the companion `rebuild_matrices` stamps, the `recompute_dc_op` winding guard, the BE-fallback gate) ran only under tests that bypassed the build. `CircuitIR::from_kernel` now refuses a kernel carrying companion inductors and names `from_mna_augmented`. The library `DkKernel::from_mna` companion and the runtime `LinearSolver` are unchanged. Golden: 188/188 renders identical; generated code differs only in one corrected doc line and dropped blank lines on DK decks.
- **Validate cannot pass a third of the corpus (2026-09-29, open, after the test-builder stage 3).** 33 of 85 local-corpus decks fail `melange validate` both before and after the build unification. A suite that fails 39 % of the corpus cannot discriminate. Triage each into (a) a real model mismatch against ngspice, (b) a harness or reference-twin limitation (unsupported element, auto-scaling, probe mapping), (c) a deck broken on its own. Start with wurli-power-amp (the only one that ships): 0.09 % RMS and correlation 0.9999996, yet it fails a 20 mV absolute peak tolerance at 33 mV on a stage swinging tens of volts — decide whether that tolerance is meaningful (tolerance presets) before counting it as a mismatch. Add one validate fixture each for `.linearize`, `.inject` and `.integrator`, which the suite lacked and which is how the build assemblies drifted unseen.
- **(iv) DK containment parity (logged 2026-09-29, not scheduled).** DK has no unsolved-sample containment (a timestep cut at a fold); both nodal sub-paths run the same sub-step ladder (local refinement down to T/2^12, at most 64 attempts per sample). Whether DK gets it or gives way to nodal on nonlinear switching decks is open.
  **HARD COUPLING: (iv) must NOT land without the exact seed.** Once DK can sub-step, its failed jumps stop being loud: a rescued sample can converge on whatever branch its start favours, which is exactly the nodal-Schur 192 kHz situation. So when (iv) is built, the exact seed goes in with it, gated on the same witnesses. The DK exact seed is built and parked on branch `dk-exact-seed-for-iv` (f317d10).
  Why DK keeps the first-order predictor until then: without regeneration (positive feedback) each step's equations have a single root, so the start changes the iteration count, never the answer. With regeneration DK has no rescue, so a failed jump exhausts MAX_ITER and is counted in `diag_unsolved_sample_count` and refused: loud. The remaining gap is a start that converges onto the wrong branch within MAX_ITER on DK; no DK render built so far shows it (the IC astable and a driven BJT Schmitt trigger are refused first), and the RHP gate plus routing keep such decks off DK by default. The exact seed measured on DK (2026-09-29): +21–100 % CPU on every DK deck (noyce-triode-12ax7 184 → 284 ns/sample), no deck faster, and more loud failures on the regenerative decks DK cannot solve (Schmitt trigger at 192 kHz: 189 → 764 unsolved).
  **Reopen trigger before (iv):** any DK render with 0 unsolved samples but a switching-edge mismatch against a settled reference. Witness: the IC-seeded G10 astable fixture in `cli_integration.rs` — on DK 1330 of 2400 samples unsolved at 1× (tests pass `--allow-nr-hold`, guard boundedness only).
- ~~**Sub-step ladder depth at a regenerative fold, at base rates**~~ **FIXED (2026-09-29).** The 64× ladder restarted the whole sample at a uniform 2ⁿ subdivision and never reached the IC-seeded G10 astable's switching edges at 48/96 kHz: over 0.8 s, 1320 / 1174 held samples at 48 kHz (Schur / full-LU), 1083 / 856 at 96 kHz, the period +2–3 % against ngspice's 1.1662 ms. The ladder now refines locally: it bisects only the failing sub-step, keeps the converged prefix and grows the step back once aligned, down to T/2^12 and within 64 attempts per sample (`SUBSTEP_MAX_POWER`, `SUBSTEP_BUDGET`). Measured need on that deck: depth 2^9, at most 31 attempts, mean ~12. Now 0 held on both sub-paths at every rate; period 1.1629 / 1.1613 ms at 48 kHz, 1.1638 / 1.1633 at 96 kHz, 1.1662 / 1.1647 at 192 kHz. A driven BJT Schmitt trigger (held 17–28 samples before) switches at 1.3909 / −0.8551 V against ngspice's 1.3986 / −0.8338 V at 192 kHz, 0 held. Witnesses: `cli_integration::test_ic_seeded_astable_base_rate_solves_every_sample`, `test_schmitt_trigger_switches_at_spice_thresholds`.
- ~~**Nodal Schur picked a different root at a regenerative fold**~~ **FIXED (2026-09-29) on nodal Schur; DK pending (same class, next).** On the IC-seeded G10 astable at 192 kHz, forced Schur settled to a 0.4617 ms period (ngspice 1.1662 ms, full-LU 1.1645) with every sample KCL-valid and no counter moving: its first-order `i_nl` warm start started the Newton on another branch. Linearising the first iteration at `N_v·v_prev` is not enough (0.6428 ms): Schur's limiter still steps from its own start. The fix makes the first iterate identical to full-LU's, solving `K·i_nl = N_v·v_prev − p` (rank-revealing LU of `K`, cached and refactored when `K` changes; a singular but consistent `K`, e.g. an antiparallel diode pair, is solved exactly, since any solution gives the same Newton sequence; an unreachable start falls back to the predictor and is counted in `diag_warm_start_fallback_count`, 0 on every corpus render). Now 1.1658 ms (`cli_integration::test_ic_seeded_astable_schur_period_matches_spice`). **DK keeps the first-order predictor, by design (2026-09-29); see the (iv) entry for why that is safe until DK can sub-step, and the coupling that ends it.** No counter can see a branch jump in general; an LTE check at edges is the open item.
- **Op-amp pin outcome differs by nodal sub-path (2026-09-29, unification item).** On a Schur build with devices an engaged pin decides the sample (`converged` = the pinned solve's outcome, so a converged pin clears an unpinned failure and a failed pin goes to backward Euler, then the hold). Full-LU runs the pin only on a converged sample and commits a failed pin (`diag_nr_unconverged_commit_count`). Both are counted in `diag_unsolved_sample_count`; the rule itself is not yet one definition.
- **five-watt-freddie (champ-5f1, junk-grade deck): trapezoidal Newton fails on almost every sample** (2026-09-29, low priority). The old auto-BE promotion hid it; under the ring predicate the deck is trapezoidal (no lasting Nyquist-side pole) and the golden renders show the BE fallback on 95 615 of 96 000 sine1k samples and 87 147 of the sweep, the runtime latch engaging, and the sweep reaching the output clamp (5 samples). A convergence defect of trapezoidal Newton on this deck, not a ring: attribute it (which rows fail, first failing sample) before anything routes around it.
- ~~**`melange simulate --switch "Label=pos"`**~~ **DONE** (verified 2026-09-22): the flag exists on `simulate` (`--switch <NAME=POS>`, `main.rs:4439` builds `switch_calls` from `switch_runtime_overrides`) and a `--switch "Tone=2"` run resolves and applies the position. This entry was stale.
- **Switch/pot z=−1 intrinsic to hard-switching (not just `.switch`)** (measured under the whole-system form; re-measure under the charge form, where capless subspaces carry no memory): beyond the `.switch` G-swap fix (Deferred), the trap z=−1 mode on capless subspaces is re-excited by ANY hard-switching edge (BJT edges in the g10 divider re-excite it every ~160 samples → openfarf's residual wanders 0.9–4.9% with no decay). Existing nodal auto-BE gates on ρ>1.002; a marginal z=−1 (ρ=1) may slip it — likely gap. Grader for this case: CLI == codegen window-for-window on the g10 chain keyed closed (not a 1e-7 target).
- **Neve 1073**: EQ section (Stage 3), integration (Stage 4), plugin (Stage 5). Stages 1 & 2 BA283 amps SPICE-validated.
- **Oomox plugin roadmap**: `.runtime` VS, named constants, DC op accessor, warmup constant, runtime DC OP recompute
- **Performance**: DK parasitic BJTs (power amp 0.41×, K_eff approach planned); hot/cold state split; fast_powf for Koren tube model
- **Documentation**: user-facing docs, example circuits, getting-started guide
- **Multi-language codegen**: `Emitter` trait + `CircuitIR` are language-agnostic by design. In progress: C++. Planned: Python/NumPy, MATLAB/Octave. **FAUST: explored, ruled out (2026-09-02)** — FAUST's generated code is not Turing-complete by design, so a data-dependent NR iteration count is inexpressible; only circuits emitting no NR loop at all would work (6 of 41 corpus circuits). Note the predicate is "no NR loop emitted", NOT `M == 0`: behavioural B-sources route nodal and get Newton regardless of M.
- **wurli-power-amp residual — FIXED 2026-08-03** (raised by melange-circuits 2026-07-25, after the auto-BE router-corroboration fix `b0dcb27` closed the timeout/explosion bug). Prior text here ("~10 dB past clipping, output still reaches 353 V") was itself stale — the raw internal-node blowup was far worse and erratic across amplitude (not monotonic with drive): amp 0.05 → 16,079 V, amp 1.00 → 27,977 V, amp 2.00 → 22,201 V internal, while amp 0.10/0.30/0.50 stayed physical (20–32 V) — convergence-path-dependent, not a clipping-level threshold. Root cause: `emit_nodal`'s per-iteration "global node voltage damping" (`nodal_emitter.rs`, both the primary NR loop and the Backward Euler fallback loop) capped the damping ratio with `.max(0.01)`, so a single NR iteration's LU solve producing a raw voltage delta many orders of magnitude beyond the intended cap (observed 3.8e7 V at a class-AB crossover device-state transition) still let a multi-kV single-iteration jump through (1% of 3.8e7 ≫ the ≤10 V ceiling). The BE-fallback's voltage-step-only convergence check then falsely accepted the resulting nonphysical fixed point (its relative tolerance scales with the already-diverged node voltage). Fixed by removing the `.max(0.01)` floor so the ratio divides uncapped, bounding every iteration's worst-case node step at exactly the intended threshold regardless of raw delta magnitude. All amplitudes now stay within 20–32 V internal; `nr_max_iter_count`/`be_fallback_count` also dropped 10–70× (bad state no longer cascades into subsequent samples). Regression: `nodal_be_fallback_alpha_floor_tests.rs::test_nodal_full_lu_node_damping_has_no_ratio_floor`.
- **BJT forward-active (FA) reduction rule re-check** (raised by melange-circuits 2026-07-25): on wurli-power-amp, 7 of 8 BJTs clear the `Vbc < -0.5 V` FA threshold (`DEBUGGING.md` — device evaluated at Vbc ≈ -20 V) yet all 8 stay full 2D in the shipped codegen. Open question whether the FA rule is still being applied as documented for this circuit, or whether something else (e.g. `--tube-grid-fa`-style override, K-conditioning skip-expansion gate) is suppressing it. Gates a downstream CPU-budget decision in openwurli. Not yet investigated.
- ~~**BJT analogue of `--tube-grid-fa off`**~~ **SHIPPED** (verified 2026-09-22): `--bjt-fa {auto,force,off}` exists on `compile`. `auto` reduces only pure Ebers-Moll BJTs (where the 1-D forward-active model is exact) and is byte-identical to prior codegen; `force` also 1-D-reduces Gummel-Poon / ISE / parasitic BJTs with a per-device warning, dropping the `qb` base-charge term (~1–2 dB under hard drive); `off` keeps every BJT full 2-D. Self-heating BJTs are never force-reduced. This entry was stale.

### Deferred
- **OPEN FINDING: pipe-shouter has no DC operating point under `--opamp-rail-mode boyle-diodes`** (explicit-only mode; the build refuses it, `--allow-unconverged-dc-op` overrides). State at `5dd0dcc`: Direct NR, source stepping and Gmin stepping each fail (200 / 400 / 200 iterations; KCL residual 2.5e3 A at an internal row). Two single-supply JRC4558 stages (`VCC=9 VEE=0`): the linear start puts each Boyle internal gain node (Gm 0.2 S against its 1 µS self-load) near 900 V, the junction clamp then moves the source-fixed catch-diode reference with it (the source row restores it on step 1), and Newton does not recover. Unchanged by the in-Newton rail active set, the grounded-junction clamp and the held-pin port; the other rail modes converge on this deck. Likely the same internal-node conditioning as "The BoyleDiodes heavy-clip problem" in OPAMP_RAIL_MODES.md. Not traced further.
- **DC-OP cold start: SPICE's zero-node start with `MODEINITJCT`** (the principled alternative to the junction clamp in `clamp_junction_voltages`, see DC_OP.md "Direct NR"). Take it if the clamp meets a topology it cannot handle. Measured 2026-09-29 as `MODEINITJCT` layered on the LINEAR-GUESS start (not SPICE's pairing, whose node vector starts at zero): grounded-emitter BJT control 193 → 11 iterations (the clamp: 7), but the Wurlitzer preamp DirectNr 8 → Failed 211 (all three ladder strategies fail; excluding parasitic BJTs gives SourceStepping 194), and +1..+7 iterations on gravity, 4kbuscomp, moonladder, pipe-shouter, opamp-pin-control, 1073, zener. The zero-node pairing itself is unmeasured.
- **`.switch`/`.pot`/`.runtime R` G-swap first-sample 2× artifact** (root-caused 2026-08-15, raised by melange-circuits/openfarf; user-gated fix). **Status under the charge form:** the 2× mechanism below is a property of the whole-system history `a_neg = (2/T)C − g`; the charge-form history `(2/T)C` carries no conductance, so a swap does not count Δg twice. The z=−1 manifestations below are the whole-system walk. Both are pending re-measurement (Pending Work, "Re-measure under the charge form"). Record as measured under the whole-system form: A conductance changed mid-run produces an output at the *swap sample* exactly 2.000× the physical value (drive-independent, deterministic), correct one sample later. Mechanism: trapezoidal puts every conductance in BOTH `A = g+(2/T)C` and `a_neg = (2/T)C−g`, so a switch of Δg adds +Δg to the forward matrix and −Δg to the history matrix; on the swap sample `v[n] = v_prev − 2·A_new⁻¹·Δg·v_prev` — Δg counted twice (`rebuild_matrices` correctly rebuilds `a_neg`; this is inherent to the trap formulation, not staleness). It is a **bug, not a contract** — a resistive divider responds instantly, so the swap sample should be physical; do NOT document it as "undefined." Fix: use the pre-switch conductance in the history term for the one transition sample (one-sample old-g lag on the switched conductance in `a_neg`) — a core-solver change touching every `.switch`/`.pot`/`.runtime R` circuit, needs full golden/SPICE validation. Repro: any `.switch` onto a resistive path, toggle mid-run, compare first-sample deflection to settled ratio. **Second artifact (same event):** the swap also excites a trapezoidal z=−1 Nyquist marginal-stability mode that rings for ~1000 samples (damped by circuit RC; parity-split-into-two-smooth-sequences signature; verified undamped in a no-cap repro, and gone under `--backward-euler`). The old-g lag does NOT kill this mode — it only shrinks the exciting impulse. **Complete fix is two parts:** (a) the one-sample old-g history lag (kills the 2×); (b) breakpoint-style force-BE for 1–2 samples after any `.switch`/`.pot`/`.runtime R` event (damps the Nyquist mode at the source — commercial-SPICE breakpoint practice; the switch-triggered analog of the existing nodal auto-BE). **Third manifestation (reproduced 2026-08-15): PERSISTENT residual on capless switched nodes.** On a purely-resistive (algebraic, no-cap) node the z=−1 mode is *undamped*, so the swap excitation never decays — the two-node residual stays non-zero for as long as you rest in the non-default position (openfarf's g10-ref busbar: 1.44% held-pos1, RC-damped by the chain's 1µF; a no-cap minimal repro shows ±0.32 undamped). Confirmed the trap-marginal mode, not a rebuilt-matrix error: `--backward-euler` held-pos1 residual = −3.5e-13 (consistent), null Δg=0 exact. **Breakpoint-BE is load-bearing** — it fixes both the decaying ring (capped nodes) and this persistent residual (capless nodes); the old-g lag alone does not fix the capless case. Regression oracle: a purely resistive node obeys `node − ratio·other ≈ 0` at all times — grade the two-node residual of a CAPLESS node held in a non-default position (must read ~1e-7), NOT node-vs-DC (a damping cap hides the bug). Workaround (openfarf): fit transients from closure+1; measurements while resting in a non-default position on a capless node are contaminated under trap. See memory `switch_gswap_trap_2x_first_sample_2026_08_15`.
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
