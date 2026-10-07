# Melange Status Reference

Quick-reference for AI agents. For math details see other aidocs. For architecture see CLAUDE.md.
Release history lives in `CHANGELOG.md`; this file states what is true now and what is open.

> **Latest release: v0.1.16 (2026-10-05).**
>
> Triode grid current is the Dempwolf & Zölzer eq. (11) law, which **fails** its Philips ECC83
> acceptance test 15/15 and ships as the less-wrong model, replacing a law that was further out
> in the same direction; see Pending Work for the specified fix.

## SPICE Validation Results

Tests in `crates/melange-validate/tests/spice_validation.rs`. Run with
`cargo test -p melange-validate --test spice_validation -- --include-ignored --nocapture`
(requires `ngspice` on PATH). Each test calls `run_melange_codegen()`, which
generates Rust code from the netlist via the codegen pipeline, compiles it
with `rustc -O`, runs it as a subprocess, and compares the output samples
against ngspice with `.OPTIONS INTERP` for sample alignment.

**Baseline: 2026-07-18** (HEAD `b421358`, ngspice-42, single-VIN
Thevenin-PWL decks, ngspice `reltol=1e-4`). Every gated test carries a cited
measured value in its gate comment (`crates/melange-validate/tests/spice_validation.rs`);
the table below is derived from those comments as of that baseline.

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
| Op-amp overdrive, diode feedback clipper | 0.99999032 | 0.442% | 9.9e-3 V | Op-amp + 1N4148, THD err 0.04 dB |
| Op-amp overdrive (wiper, pos=0.85) | 0.99838696 | 5.71% | — | Volume divider + simplified tone; THD err 0.11 dB |
| Wurli preamp | 0.99999734 | 0.235% | 1.28e-3 V | 2× 2N5089, M=5, gain ratio 1.0006, 10 ms settle |
| 3-BJT transformer-coupled output amp | 0.99999952 | 0.107% | 1.06e-4 V | 3 BJT + output xfmr, gain 6.7×, ratio 1.0000, 10 ms settle |
| 3-BJT microphone preamp | 1.00000000 | 0.0346% | 1.12e-4 V | 3× BC184C, gain 26.0×, ratio 0.9996, 64 ms settle |
| Pot static off-nominal | 0.99999995 | 0.0374% | 4.06e-3 V | `.pot` rebuild vs fixed-R deck, THD err 0.01 dB |
| Pot modulation (5 kHz R sweep) | 0.99991763 | 1.28% | 3.93e-2 V | vs native ngspice B-source; residual is per-sample ZOH of R(t) |

The static off-nominal pot test sits at the 0.037% floor — the same floor as
the nominal-position diode tests — which confirms the 1.28% modulation residual
is R(t) zero-order-hold discretization at 5 kHz mod / 48 kHz fs, not the
`.pot` rebuild mechanism.

The corpus-wide comparison (`melange validate` on every circuits-repo deck) is
under Validated Circuits below.

## Device Model Features

- **Junction capacitances**: CCG/CGP/CCP (tube), CJE/CJC (BJT), CGS/CGD (JFET/MOSFET), CJO (diode)
- **Parasitic resistances**: diode RS, BJT RB/RC/RE, triode RGI (JFET/MOSFET RD/RS are refused, see below)
- **BJT extras**: NF/ISE/NE (emission/leakage), Gummel-Poon (VAF/VAR/IKF/IKR), self-heating (RTH/CTH/XTI/EG/TAMB) — disabled by default (RTH=∞)
- **Diode**: BV/IBV (Zener breakdown), self-heating (RTH/CTH/XTI/EG/TAMB) — disabled by default (RTH=∞); analytic-validated 2026-04-21 against `Tj_ss = TAMB + P·Rth` and exponential τ = RTH·CTH; ngspice parity not applicable (SPICE3f5 BJT/diode silently drop RTH)
- **Device temperature**: diode and BJT cards take `TAMB` (kelvin, default 300.15 K = SPICE's TNOM of 27 °C) and scale from TNOM with the SPICE3 law: IS through XTI and EG, VT in proportion, BJT BF/BR through XTB, ISE/ISC through both. `VT_ROOM` = kT/q at 300.15 K (`melange_primitives::util`) for diodes, BJTs, JFETs and MOSFETs. TNOM itself is fixed; `.temp` is ignored with a warning.
- **MOSFET**: GAMMA/PHI (body effect)
- **Op-amp**: Boyle VCCS macromodel (no bandwidth pole: GBW only defaults the rails), rail-clamping modes (`auto/none/hard/active-set/boyle-diodes`, see `--opamp-rail-mode` CLI flag), and optional slew-rate limiting via `.model OA(SR=13)` in V/μs (per-sample `|Δv_out| ≤ SR·dt` clamp, all 3 codegen paths, default `SR=∞` → zero code emitted)
- **BJT Gummel-Poon**: matches ngspice `bjtload.c` line-for-line (q2 uses `cbe/IKF + cbc/IKR` with `cbe = IS*(exp(Vbe/(NF*VT))-1)`; Ib ideal forward NOT divided by qb)
- **Pentode / beam tetrode**: Three screen-current equation families selected per-slot by a `ScreenForm` discriminator on `TubeParams`. **Rational** (Reefman Derk §4.4, `1/(1+β·Vp)`, 9 params) for true pentodes; **Exponential** (Reefman DerkE §4.5, `exp(-(β·Vp)^{3/2})`, 9 params) for beam tetrodes with critical-compensation knees; **Classical** (Norman Koren 1996 / Cohen-Hélie 2010, `arctan(Vpk/Kvb)` + Vp-independent screen, 6 params) as a fallback for tubes without Reefman fits. Optionally blended via Reefman §5 two-section Koren (variable-mu) for remote-cutoff tubes. Catalog: EL84/6BQ5, EL34/6CA7, EF86/6267 (Rational, `-P` suffix); 6L6GC/5881, 6V6GT (Exponential, `-T` suffix); KT88, 6550 (Classical, no suffix); 6K7, EF89 (variable-mu Rational/Exponential). Element prefix `P` (`P n_plate n_grid n_cathode n_screen [n_suppressor] model`; the suppressor is modelled cathode-tied, and a 5th node other than the cathode is refused) and `VP` model token. A grid-off reduction (3D→2D) is opt-in (`--tube-grid-fa on`).
- **Not implemented**: temperature coefficients on resistors (TC1/TC2), op-amp `EN_FC`/`IN_FC` 1/f corner (accepted with a compile notice; Phase 4 is white-band only in v1), DK-path BJT parasitic-R (rbb′) thermal noise (nodal only), diode `RS` / tube `RGI` thermal noise, tube microphonics (Phase 6), 6386/6BA6/6BC8 datasheet fits for varimu compressors (phase 1d deferred). All five noise phases (thermal, shot, junction+resistor flicker, op-amp en/in, pentode partition) ARE shipped — see "Circuit Noise" under Feature Inventory.
- **Known model limitations**:
  - Diode BV: exponential reverse breakdown (matches codegen template), evaluated in both codegen and the DC OP solver
  - DC OP diode: a 1e-12 S Newton-conditioning conductance on the diode Jacobian only, not on the current (the diode current at the fixed point is the diode law's, as in the transient)
  - DC OP op-amps: AOL is capped at 1000 in the DC G as a homotopy aid against multi-equilibrium NR in precision rectifiers; the solve finishes at the full AOL from that point (`dc_op.rs`, `AOL_DC_MAX`)
  - DC OP input ports: the DC OP solves the `mna.g` the build stamped (every input port's conductance counted once, as the transient counts it); `DcOpConfig` has no input fields. The reported KCL residual is against the circuit's own G, not the working copy with its solver aids, so it carries the node Gmin leak (~1e-12·|v| A). Witness: a divider tapped through a 1 MΩ port matches ngspice (v(in) 2.992519 V) in the baked `DC_OP`, `recompute_dc_op` and `melange dc-op`
  - DC OP failed convergence: low-rate warmup (200 Hz × 1000 samples = 5s circuit time) charges coupling caps before transient NR. Settled state cached for `reset()`. 4kbuscomp: BE fallback <1%, stable at all amplitudes.
  - BJT GP Q1: singularity guard at `q1_denom <= 0` (physically near Early voltage limit)
  - Tube Koren: no space-charge, no transit-time effects
  - JFET (level 1) / MOSFET subthreshold: hardcoded 2×VT slope (real devices: 60-120 mV/decade); a JFET card with `LEVEL=2` has the Parker–Skellern subthreshold law (`VST`, `MVST`)
  - VCA noise_floor field exists but unused
  - No automatic transient AOL cap on op-amps: under the charge form and active-set pinning the uncapped solve converges on precision rectifiers, and an automatic cap moved a biased rectifier's answer by 4.5 mV. `.model OA(AOL_TRANSIENT_CAP=N)` remains an author's key and routes nodal.

## Codegen Device Support

There is no runtime circuit solver: all device handling lives in the codegen pipeline.
The library `LinearSolver` (M=0 only) is deprecated since 0.1.14 and removed in the
next release: no build uses it, and its whole-system discretisation is not the shipped
charge form (`COMPANION_MODELS.md`, last section).

| Device | NR Dim | Model |
|--------|--------|-------|
| Diode | 1D | Shockley + RS + BV |
| BJT | 2D (Vbe→Ic, Vbc→Ib) | Gummel-Poon / Ebers-Moll |
| BJT (forward-active reduced) | 1D (Vbe→Ic, Ib = Ic/βF) | Opt-in only: `--bjt-fa auto\|force` (default `off`); a sample on which a reduced BJT leaves forward-active is counted unsolved and refused |
| BJT (linearized) | 0D (removed from NR) | `.linearize`: device Jacobian at the DC OP stamped into G |
| JFET | 2D | Shichman-Hodges (`LEVEL=1`, default); Parker–Skellern (`LEVEL=2`, ngspice JFET2 at zero dispersion) |
| MOSFET | 2D | Level 1 SPICE |
| Tube (triode) | 2D (Vgk→Ip, Vpk→Ig) | Koren plate + Dempwolf & Zölzer grid |
| Tube (pentode) | 3D (Vgk→Ip, Vpk→Ig2, Vg2k→Ig1) | Reefman Derk §4.4 / DerkE §4.5 / Classical + Leach |
| Tube (pentode, grid-off) | 2D (Vgk→Ip, Vpk→Ig2, Vg2k frozen) | Opt-in only (`--tube-grid-fa on`, warned); `auto` keeps full 3D — the freeze is not accuracy-neutral (cathode-referenced Vg2k, +2–12% measured) |
| VCA | 2D (Vsig, Vctrl) | THAT 2180 exponential |
| Op-amp | Linear (no NR dim) | Boyle VCCS + rail clamp; no GBW pole (GBW only defaults ±13 V rails, with a notice) |

M=1 direct, M=2 Cramer's, M=3..32 Gaussian elimination with partial pivoting
(`MAX_M` = 32, `crates/melange-solver/src/dk.rs`; above it every route refuses).

## Codegen Solver Routing

| Path | When Selected | Cost | Notes |
|------|--------------|------|-------|
| DK Schur | M<10, ≤1 xfmr, K well-conditioned, no op-amp needing active-set rail handling | O(N²+M³)/sample | *(no measured figure — see README table)* |
| Nodal Schur | M≥10 or 2+ xfmr, K well-conditioned | O(N²+M³)/sample | Medium-complexity circuits |
| Nodal full LU | saturating inductor or behavioral source (structural); K≈0 (VCA), positive K diag, K ill-cond, S ill-cond, unstable Schur prediction | O(N³)/sample | Universal; chord + sparse LU |

K≈0 detection: max|K| < 1e-6 with M > 0.

A clamped op-amp whose rail mode resolves to `active-set`/`active-set-be` (an
AC-coupled output under `auto`, or asked for explicitly) routes nodal: only
nodal implements the pin-and-resolve, and forcing DK is refused. See
`OPAMP_RAIL_MODES.md` → "Which solver runs which mode".

The routing estimates "trapezoidal unstable" (DK vs nodal, `routing.rs`) and
`spectral_radius_s_aneg` (nodal Schur vs full LU) use the whole-system operator
`S·(αC − G)`. Neither decides the integrator: backward-Euler promotion is the
ring predicate on the charge propagator linearised at the DC OP
([RING_PREDICATE.md](RING_PREDICATE.md)).

**Self-starting oscillators stay off DK.** A DK build whose DC operating point
has a growing pole (trapezoidal spectral radius > `TRAP_BE_PROMOTION_RHO` —
under the charge form exactly a right-half-plane pole of the DC-OP-linearised
system) is refused (`CodegenError::SelfStartingOscillator`): the auto route
rebuilds on nodal and says why, a forced `--solver dk` fails with the reason.
Scope: self-starting only. A kick-started or driven regenerative circuit (an
astable seeded by `IC=`, a flip-flop) has a stable DC operating point and is
caught at runtime by `diag_unsolved_sample_count` (present on every build;
refused by `simulate`/`validate`/golden), not here. Witness: the G10 master
oscillator fixture (ρ 1.0149 at 48 kHz),
`cli_integration::test_lc_master_oscillator_self_starting_is_refused_on_dk`.

## Circuit Library Status

Circuits live in a separate repository. **Public: https://gitlab.com/oomox-group/melange-circuits** (43 circuits — a FILTERED set; the full catalog is the private `melange-circuits-private`, checked out locally at `../melange-circuits`). Short names resolve through the repo's `circuits-index.json`; see `docs/CIRCUIT_INDEX.md`.
All circuits are in `unstable/` until the user manually tests and approves promotion.

The compiler validation status of circuits known to exercise specific solver paths
(routes as compiled by v0.1.13; throughput figures only where `bench.sh` measured them,
see Performance):

| Topology class | N | M | Solver | Notes |
|----------------|---|---|--------|-------|
| Linear RC | 2 | 0 | Linear | Smoke test |
| 2-stage BJT preamp (wurli-preamp) | 13 | 5 | Nodal Schur | 2N5089 Ebers-Moll, `.integrator be` |
| 2-stage triode preamp (twas-preamp) | 14 | 4 | DK | 2× 12AX7, pot + switch |
| 4-tube passive EQ + 3 xfmrs (passive-eq1a) | 52 | 8 | Nodal Schur | Multi-transformer forces nodal |
| 8-BJT Class AB power amp (wurli-power-amp) | 23 | 14 | Nodal full LU | Parasitic R; full-LU trigger: unstable Schur prediction |
| 4-opamp + diode clipper | 44 | 10 | Nodal full LU (auto) | Clean clipping verified under ActiveSetBe; auto resolves ActiveSet, not re-run on this deck; BoyleDiodes diverges at heavy clip |
| Op-amp overdrive + diodes | — | — | DK | Single-op-amp diode-feedback clipping |
| VCA compressor + sidechain | 21 | 3 | Nodal full LU | Current-mode VCA, K≈0 |
| VCA bus compressor (4kbuscomp) | — | — | Nodal full LU | 12 op-amps, 2 VCAs; full-LU trigger: S ill-conditioned |
| Pentode single stage | — | 3 | DK | Full 3D by default; grid-off M=3→2 only with `--tube-grid-fa on` |
| Push-pull pentode amp + OT | — | — | Nodal | Transformer forces nodal path |
| Variable-mu pentode | — | 3 | DK | M=3, no grid-off reduction |

Only circuits using standard SPICE models (D, NPN/PNP, NJF/PJF, NM/PM) have a direct
ngspice twin; `melange validate` also translates triodes, sharp pentodes and op-amps
into twins (see `docs/limitations.md` "SPICE Validation Scope"). VCA, LDR, NEON and
variable-mu pentodes have none and are checked with `compile`/`analyze`/`simulate`.

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
- DK kernel with proper trapezoidal discretization; NR solver 1D / 2D / M-dimensional (M≤32)
- **Integrator: trapezoidal in the charge (companion) form** on every generated path (DK and nodal; Schur, full-LU, adaptive sub-steps, sub-sample fire): `A·x_{n+1} − N_i·i_nl(x_{n+1}) = RHS_CONST + αC·x_n + q_dot_n + b_{n+1}`, with the capacitor currents `q_dot = C·ẋ` carried as state (`pub q_dot` on trapezoidal builds) and every source (DC, input, `.inject`, `.runtime`, noise) entering once, at n+1. KCL of a committed sample is that sample's own solve residual — no `z = −1` walk on capless rows. Backward Euler (fallback, breakpoint-BE, BE-latch, auto-BE promotion, `--backward-euler`) has the same shape with `C/T` and no `q_dot`. `RHS_CONST` is ×1 on every row under both. Full reference: [COMPANION_MODELS.md](COMPANION_MODELS.md) "Charge (Companion) Form"
- **Breakpoint-BE** arms only where a reactance changes: a `.switch` that swaps a capacitor or an inductor (the carried `q_dot` was built on the old value) and glow strikes. A conductance change (pot, resistor switch, `.runtime R`) is exact under the charge form with no special sample (`breakpoint_be_tests.rs`, `a_resistor_switch_is_exact_without_breakpoint_be`).
- **Auto-BE promotion** is the ring predicate on the charge propagator linearised at the DC OP ([RING_PREDICATE.md](RING_PREDICATE.md)); it covers DK and nodal builds. The verdict is taken at the compiled rate.
- Codegen for diode, BJT, JFET, MOSFET, tube/triode/pentode (Gaussian elimination M=3..32)
- Per-device `.model` params (heterogeneous models supported per device)
- Parasitic cap auto-insertion (10pF junction caps) when nonlinear circuit has no caps
- Sparsity-aware emission (systematic skipping of absent entries in A_neg, N_v, K, S*N_i); K and S patterns are structural, derived from the topology (`structural.rs`), so they do not depend on the sample rate or on rounding; rounding noise outside the pattern is bounded, then set to exactly zero (LINEAR_ALGEBRA.md "Structural Sparsity")
- Runtime sample rate: `set_sample_rate()` recomputes matrices from G+C (the route does not change; compile per host rate)
- Inductors are built on augmented MNA rows (`DkKernel::from_mna_augmented`) on every shipped route; `CircuitIR::from_kernel` refuses a kernel carrying companion-modelled inductors

### DC Operating Point
- LU with partial pivoting, logarithmic junction-aware voltage limiting, source + Gmin stepping
- Internal nodes for parasitic BJTs (basePrime/colPrime/emitPrime, ngspice-style)
- Op-amp seeding + per-iteration rail clamp + AOL=1000 homotopy aid in DC G, finished at full AOL (precision rectifiers)
- Diode BV/IBV breakdown; a 1e-12 S Newton-conditioning conductance on the diode Jacobian only (the diode current at the fixed point is the diode law's, as in the transient; node Gmin is still an open item, below)
- Low-rate DC warmup (200 Hz × 1000 samples) for failed-DC-OP circuits; settled state cached
- `DC_NL_I` constant initializes `i_nl_prev` in generated code

### Device Models
- **BJT**: Gummel-Poon (VAF/VAR/IKF/IKR, CJE/CJC, NF/ISE/NE) matching ngspice `bjtload.c` line-for-line; Ebers-Moll fallback; device temperature `TAMB` (the per-card analogue of SPICE's `.temp`: IS/BF/BR/ISE/ISC/VT scaled from TNOM, ngspice-gated); self-heating (RTH/CTH/TAMB); RB/RC/RE parasitic R. Forward-active reduction is opt-in (`--bjt-fa {off,auto,force}`, default `off`); a region exit on a reduced device (forward-active BJT, grid-off pentode) counts as an unsolved sample (`diag_reduced_model_exit_count`) and every verb refuses it. `diag_region_exit_count` stays as characterization on full models. Witness: `golden_ref_tests::bjt_ce_with_forward_active_reduction_counts_its_saturation_unsolved`.
- **JFET/MOSFET**: 2D Shichman-Hodges / Level 1; JFET `LEVEL=2` is Parker–Skellern (subthreshold softplus `VST`/`MVST`, dual power law `P`/`Q`, smooth early saturation `Z`/`XI`/`MXI`/`PB`), the law of ngspice's JFET2 with dispersion and thermal reduction at zero (those keys refused when nonzero), pinned against it by `jfet2_twin_tests.rs`; JFET gate-source and gate-drain junctions (IS, N); CGS/CGD junction caps; MOSFET body effect (GAMMA/PHI), evaluated at the live Newton iterate on every path with gmb = gm·dVT/dVsb in the Jacobian, and in the DC OP the same way (pinned against ngspice by `mosfet_body_effect_tests.rs`). Card RD/RS are refused: not in the solution (internal drain/source nodes queued)
- **Diode**: Shockley + RS + CJO + BV/IBV Zener; device temperature `TAMB` (IS and N·VT scaled from TNOM, ngspice-gated); optional self-heating (RTH/CTH/XTI/EG/TAMB) using the same quasi-static electrothermal model as BJT, with `IS(T) = IS_amb·(Tj/TAMB)^(XTI/N)·exp(EG/(N·VT_amb)·(1−TAMB/Tj))` and `N·VT(T) = (N·VT)_amb·(Tj/TAMB)`. pipe-shouter uses RTH=500 CTH=2e-4 on the 1N4148 clippers; sad-bastard uses RTH=1200 CTH=1e-4 EG=0.67 on the 1N34A Ge clippers. Dead code when RTH=∞ (default).
- **Tube (triode)**: Koren plate + Dempwolf & Zölzer grid current, early-effect lambda, CCG/CGP/CCP junction caps, RGI grid-stop (the DC OP evaluates the triode at its internal grid behind RGI, as the transient does)
- **Tube (pentode)**: 3 screen-current equation families — Rational (Reefman §4.4), Exponential (DerkE §4.5), Classical Koren. `--tube-grid-fa {auto,on,off}`: `on` reduces 3D→2D (warned, not accuracy-neutral); `auto` == `off` == full 3D. `diag_region_exit_count` counts grid-conduction / BJT-saturation samples on every path
- **Op-amp**: Boyle VCCS macromodel (no GBW pole), VCC/VEE asymmetric rails, optional `SR=` slew-rate limiting (V/μs), rail modes `auto/none/hard/active-set/active-set-be/boyle-diodes`, `AOL_TRANSIENT_CAP` override
- **VCA**: THAT 2180 / DBX 2150 current-mode exponential gain with gain-dependent THD
- **Saturating inductor / transformer**: anhysteretic flux `Φ = L_mag·Isat·tanh(i/Isat) + L_air·i` with an air-core floor (`LAIR=`/`CORE=gapped|steel|nickel`, default 3e-4 with a notice); datasheet ratings `ISAT_DROP=`/`ISAT_BASIS=`/`L_AT_IDC=` converted exactly; two-winding shared cores saturate on the T-model's magnetizing branch. A flux device inside the nodal full-LU Newton loop at every site (main, sub-step, BE fallback, active-set pin); DK and nodal Schur refused. Refuses k ≤ 0.8 groups, 3+ windings without a stated core, conflicting ISAT/floors. Full reference: [SATURATING_TRANSFORMERS.md](SATURATING_TRANSFORMERS.md).
- **Card keys**: each `.model` key is honoured, refused, or listed as unimplemented with a reason (`model_params.rs`); the key-effect suite (`model_key_effect.rs`) requires every honoured key to move the DC operating point on a biased witness unless exempt with a reason. Its known-defect list is empty.

### Unit Variation
- `.seed <u64>`: sets master RNG seed (default 0). Shared by `.mismatch` and `.tolerance`.
- `.mismatch D IS=tol N=tol RS=tol` / `.mismatch Q IS=tol BF=tol BR=tol`: per-device parameter jitter, baked at codegen. Two diodes on the same `.model` land at distinct `DEVICE_N_IS` constants — the thing that makes antiparallel clippers and push-pull pairs audibly asymmetric. `T` / `J` / `M` are IR-wired too: `T` (triode+pentode) jitters MU/EX/KG1/KP/KVB (+KG2 pentode), `J` IDSS/VP/LAMBDA, `M` KP/VT/LAMBDA. Byte-identical when the directive is absent; `analyze` applies it. Per-device tube mismatch is the physically-honest H2 source in a balanced push-pull stage (identical halves cancel evens exactly) — see `UNIT_VARIATION.md` and `SATURATING_TRANSFORMERS.md` §8-Q1.
- `.tolerance R=0.01 C=0.02 L=0.005`: fixed-passive value jitter, applied at end of `Netlist::parse()`. Skips components under `.pot`/`.wiper`/`.switch`/`.runtime R` control so UI-driven mappings stay intact.
- Deterministic: `FNV(seed, class_tag, name) → SplitMix64 → [-1, 1]`. Same seed always produces the same unit personality. Absent directives ⇒ byte-identical output (regression-guarded).
- Full reference: [UNIT_VARIATION.md](UNIT_VARIATION.md).

### Dynamic Parameters
- `.pot R min max [default] [label]`: per-block O(N³) rebuild on change; per-sample smoother via `.smoothed.next()`; reseed-free setter — use `recompute_dc_op()` for preset-recall NR refresh (DK only; nodal falls back to NR catch-up)
- `.wiper R_cw R_ccw total [pos] [label]`: two-resistor wiper; position-0..1 UI param
- `.switch R/C/L pos0 pos1 ...`: up to 16 switches; G/C/L stamped at pos-0 baseline (not static) so initial state is self-consistent
- `.gang "Label" m1 m2 ...`: links multiple `.pot`/`.wiper` members under one parameter; `!` prefix inverts; `.runtime R` members rejected at parse time (drive multiple setters from one plugin envelope instead)
- `.runtime V as <field>`: binds existing VS to `pub <field>: f64` on `CircuitState`; host writes per sample, RHS stamp uses `VSOURCE_<NAME>_RHS_ROW`
- `.runtime R min max as <field>`: audio-rate resistor modulation; emits `set_runtime_R_<field>(r)` WITHOUT the `.pot R` 20% DC-OP warm re-init (that snap clicks at envelope-follower rates); emits `RUNTIME_R_<FIELD>_MIN/_MAX/_NOMINAL` consts + `<field>()` getter; no nih-plug knob
- `.inject <node> <field> R=<ohms>|RSHUNT=<ohms> [rate=host|inner]` + `.tap <node> [name]` (`--format code` only, single `-i`): runtime single-ended sources supplied as `process_sample` arguments, and raw per-inner-sample node taps returned with the outputs. Impedance mandatory, stamped into G before the kernel; the value enters the RHS at n+1 (`v/R` Thevenin, `v` Norton). **Rates:** `host` (default) = one value per host sample, upsampled through a per-injection copy of the input's half-band up-filter (same coefficients, 4x cascade, resets → same group delay as `input`); `inner` = one value per inner sub-step, unfiltered, caller band-limits — the form for a tap→inject loop closed at the inner rate. Identical at 1x. Unknown `rate=`, duplicate `rate=`, or any other trailing token = parse error. API: `process_sample(input, injections_host: &[f64; NUM_INJECT_HOST], injections_inner: &[[f64; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR], state) -> (outputs, taps_inner)`, zero-length array for an absent kind; constants `NUM_INJECT`/`INJECT_*`, `INJECT_IS_HOST`, `NUM_INJECT_{HOST,INNER}`/`INJECT_{HOST,INNER}_*`, `NUM_TAP`/`TAP_*`. Witness: `crates/melange-validate/tests/inject_oracle.rs` (input vs `rate=host` injection max-abs 0 at 1x/2x/4x on DK, Schur and full LU, Thevenin and Norton). User docs: `docs/CODE_API.md`, `docs/spice-grammar.md`

### Behavioral Sources (B) — nodal codegen, oracle-tested
- `B... V={expr}` / `I={expr}` arbitrary-expression sources on the **nodal** path: exprs over node voltages, `time`, `ddt` (backward diff), `idt`, and `.param`/`.runtime` params. Oracle-validated in `behavioral_source_tests.rs` (current/voltage multiplier, tanh clipper, ddt-of-time, idt-ramp, slew-opamp compile).
- **Not yet**: branch-current references in exprs (errors loudly), and the DK path (B-source circuits route nodal or error). Full surface: [BEHAVIORAL_SOURCES.md](BEHAVIORAL_SOURCES.md).

### Circuit Noise (Phases 1–5, opt-in via `--noise {thermal|shot|full}`)
- **Calibration (all phases)**: every stamp is the PHYSICAL noise current at n+1, one draw per source per sample, under both integrators (the charge form enters each source once; `A − A_neg = G`). The trapezoidal integrator's own `(1 + z⁻¹)` nulls Nyquist on the charge-carrying rows; capless rows carry no history. No lag state (`*_w_prev`) exists. The BE-fallback replay of the cached currents is exact. See [NOISE.md](NOISE.md) "Constant derivation".
- **Thermal (Phase 1)**: Johnson-Nyquist on every fixed R + every dynamic R (`.pot`/`.wiper`/`.runtime R`/`.switch` R). Per-sample `sqrt(2·k_B·T·fs)·sqrt(1/R)·N(0,1)`. Shipped on both DK and nodal codegen paths via the shared `build_noise_emission()`.
- **Shot (Phase 2)**: per-junction stamp, amplitude `sqrt(Γ²)·sqrt(q·|I_prev|·fs)` from one-sample-lagged `state.i_nl_prev` (Γ²=1 for plain junctions). Diode 1 src, BJT 2 (1 when forward-active-reduced, **plus** a base-shot src Γ²=1/BF), JFET/MOSFET 1, Tube 1 — triode plate **space-charge smoothed** (Γ²=10·k·T₀·gm/(2·q·I_p) at the DC OP; `SHOT_GAMMA2=` override, 1.0 restores bare shot). Pentode plate → Phase 5 partition, not bare shot. VCA/op-amp skipped.
- **Flicker (Phase 3 junction + Phase 3.5 resistor)**: per-junction and per-resistor 1/f via Paul Kellett 7-pole pink cascade. **fs/OS-invariant** calibration: white input `sqrt(0.5·KF/K_pink)·|I|^(AF/2)` (K_pink≈6e-3 analytic; the ×0.11 tail is NOT unit-gain — K_pink sets the level), one draw per sample. Junctions opt-in `.model NAME TYPE(KF=… AF=…)`, AF default 1.0; resistors opt-in per-element `R1 a b 10k KF=… AF=…`, Hooge bias-squared, AF default 2.0 (unbiased R → thermal only). KF=0 (default) → byte-identical. Shared `set_flicker_gain`.
- **Partition (Phase 5)**: pentode plate stamp `sqrt(q·I_p·I_s/(I_p+I_s)·fs)·PARTITION_F` **replaces** bare plate shot (reuses `shot_gain`/`set_shot_gain`). `PARTITION_F` default 1.0 (process knob; ~0.6 for selected low-noise EF86). Triode-only / passive circuits emit zero partition codegen.
- **Op-amp en/in (Phase 4, v1 white-only)**: three Norton streams via `.model NAME OA(EN=… IN=…)` — en at in+ (`EN·noise_opamp_en_g_diag·sqrt(0.5·fs)`), in at in+ and in- (`IN·sqrt(0.5·fs)`). Runtime `set_opamp_input_gain` (signal-independent, distinct from `shot_gain`). `EN_FC`/`IN_FC` are accepted with a compile notice and **not modelled** in v1. Op-amps without EN/IN → byte-identical.
- Runtime: `set_noise_enabled(bool)`, `set_noise_gain`, `set_thermal_gain`/`set_shot_gain`/`set_flicker_gain`/`set_opamp_input_gain`, `set_temperature_k(K)` (290 K default; only thermal scales with T), `set_seed(u64)` (0 → entropy from system clock, nonzero → deterministic). Each method emitted only when its mechanism is present. Salted per-phase streams so thermal/shot/flicker/partition/op-amp never share a prefix under one master seed.
- Calibration validated by kTC theorem (`tests/noise_psd_validation.rs`): `V²_rms = k_B·T/C` ±15% on a 10 kΩ / 100 nF RC, and on an RC + diode compiled both trapezoidal and BE at 96 kHz (measured 3.888e-14 V² each vs kT/C 4.004e-14 V²). Nyquist regression: `thermal_noise_no_nyquist_artifact_on_resistor_only_output_node` asserts lag-1 > -0.5 AND RMS < 100 µV on a diode + series-R circuit. Full reference: [NOISE.md](NOISE.md).
- **BJT parasitic-R thermal**: `rbb′`/RC/RE thermal noise collected on the **nodal** path only (real internal-node injection); **skipped on the DK path** (no node pair for the Norton stamp) with a `log::warn!`. Route `--solver nodal` to include it. Diode `RS` / tube `RGI` are still not thermal-noise sources.
- The per-phase detail above is synced to [NOISE.md](NOISE.md) (fs-invariant flicker, triode space-charge smoothing, FA base shot, Phase 4/5, charge-form one-draw calibration). [NOISE.md](NOISE.md) remains the authoritative reference for derivations and validation. User-facing guide: [../NOISE_GUIDE.md](../NOISE_GUIDE.md).

### Codegen Infrastructure
- **DK codegen** with augmented MNA (≤1 transformer group, M<10, K well-conditioned)
- **Nodal Schur** (medium complexity), **Nodal full LU** (K≈0 / positive K / ill-cond K or S / structural)
- Full-LU optimizations stacked: chord method + cross-timestep Jacobian persistence + compile-time sparse LU (AMD ordering, symbolic factorization); a chord-accepted sample whose node-row residual exceeds 1e-9 A takes one refactored Newton step (the gated exit step)
- Nodal sub-step ladder for a sample whose Newton fails: local refinement (bisects only the failing sub-step, keeps the converged prefix) down to T/2^12 within 64 attempts per sample (`SUBSTEP_MAX_POWER`, `SUBSTEP_BUDGET`); then backward Euler on a trapezoidal build; a sample no path solves is held and counted in `diag_unsolved_sample_count`. DK has no sub-step ladder.
- Nodal Schur Newton starts at full-LU's point (`v_prev`): the first iterate solves `K·i_nl = N_v·v_prev − p` (rank-revealing LU of `K`; an unreachable start falls back to the predictor and is counted in `diag_warm_start_fallback_count`). DK keeps the first-order predictor (see the (iv) items under Pending Work).
- Oversampling 2x/4x: self-contained polyphase half-band IIR, no runtime dependencies
- `--solver {auto|dk|nodal}`, `--nodal-subpath {auto|schur|full-lu}`, `--backward-euler`, `--force-trap`, `--oversampling {1,2,4}`, `--opamp-rail-mode`, `--bjt-fa`, `--tube-grid-fa`, `--subsample-fire`
- **Runtime BE-latch** (nodal trapezoidal builds only; DK has no runtime ring latch): a cheap input-aware lag-1 detector on the mean-removed output. For one mode x = A·zⁿ its ratio is z; for a mixture it is the power-weighted mean of the components' factors. It engages at ratio ≤ −exp(−α), α = 1/(τ·fs) the estimator's forgetting rate at the internal rate: an alternating mode that outlives the window and carries the output. An alternation inside the solver's node tolerance (1e-3·|v| + 1e-6 V) is not evidence, and neither is one below −60 dB of the program that excited it: the floor is the larger of the node tolerance and max(1e-3, E_BE) × a program reference (E_BE = backward Euler's in-band damage, the ring predicate's own comparison, 0 where that comparison does not hold), the smaller of passband gain × input amplitude and the output's own excursion from its operating point (the program the output actually carries, so a clipping circuit is not judged against a linear extrapolation it never delivers), each remembered as long as the slowest Nyquist-side ring the linearised circuit carries at the running rate (held on index-2 circuits). The latch and the compile-time ring predicate share one threshold ([RING_PREDICATE.md](RING_PREDICATE.md)). At 4× the threshold sits closer to −1 per internal sample; an open-secondary leakage ring that trap nearly damps unaided at 4× latches late or not at all, which is the criterion working. If the solver falls into a self-sustaining Nyquist `(-1)^n` limit cycle at a large-signal operating point (which the compile-time ring predicate cannot see — jeffreys-tube V2 class), it latches that instance to the L-stable BE path for the rest of the stream (cleared by `reset()`, exposed via `diag_be_latch_count`). Not emitted for BE/force-trap/passive builds. Emitted for saturating-inductor circuits, including M = 0 ones. On both nodal sub-paths a latched sample skips the trapezoidal solve and runs the same backward-Euler routine a `--backward-euler` build of that sub-path runs (full-LU: main loop, sub-step, pin; Schur: M-dim Newton, pin), bit-identically from the same state. The corpus DK decks that are trapezoidal (gold-press-riaa, noyce-ef86, noyce-smps-ripple, noyce-triode-12ax7) sit more than 100 dB under the ring predicate's −60 dB threshold on a 60 s hostile program, so nothing depends on a DK latch today. On a core with no air-core floor (`LAIR=0`) it catches the deep-saturation trapezoidal ring (inductor current 20 % over the V/R ceiling at 20× Isat; 4× oversampling does not cure it); with a floor (default 3e-4 of L0) the saturating RL at 10-20 V and the choke-loaded common-source stage at 1-30 V do not ring and it does not fire. It still fires on a step into an open saturating transformer (golden `sat-core-open/step`), and that ring is not saturation: with `ISAT=100` (core linear throughout) it latches identically, with a 600 Ω load it does not (measured 2026-09-28; the open secondary's stiff leakage-into-1 MΩ mode, see SATURATING_TRANSFORMERS.md §3.4). **Cost, because the latch is sticky:** a ring that is really a transient commits that instance to BE for the rest of the stream; measured with `LAIR=0`, H1 −1.8e-4 (saturating RL at 10 V) and output −0.28 % (choke-loaded stage at 5 V). Release is the reopened item under Pending Work.
- **`.integrator {trap|be}` netlist directive**: deterministic compile-time integrator pin so a fleet regen can't silently change it. `be` ⇒ backward Euler; `trap` ⇒ trapezoidal + opt out of auto-promotion AND the runtime BE-latch net (same as `--force-trap`). Explicit CLI flags override the directive.
- **`.oversampling {1|2|4}` netlist directive**: a deck declares its recommended oversampling factor to control aliasing from nonlinear distortion products. It is an accuracy **minimum/recommendation, not a mandate** — rate costs CPU/latency (the plugin author's call). Resolution on compile/simulate/analyze: an explicit `--oversampling` always wins (even when lower — logs a `log::warn!`), else the deck value, else 1. **`validate` does NOT read the directive** — it takes `--oversampling {1|2|4}` explicitly (default 1) and reports what it was asked to measure; the reference is unfiltered and aligned by one best-fit constant delay, so the half-bands' phase stays in the number (see OVERSAMPLING.md § Validating an oversampled build). Stripped for ngspice via `MELANGE_ONLY_DIRECTIVES`.

### CLI
- `melange compile` → Rust code (`--format code`, default; any number of output nodes) or a nih-plug plugin project (`--format plugin`; 1 output node = mono, 2 = stereo with one node per channel, more refused; `--stereo` makes a 1-output circuit a 2-in/2-out plugin with one circuit instance per channel, identical component values incl. `.tolerance`/`.mismatch` draws, per-channel noise seeds (channel 0 = the mono seed), refused with `--format code`, `--mono` or 2+ output nodes). A plugin whose circuit has runtime noise (`--noise`) gets a "Circuit Noise" `BoolParam` (id `circuit_noise`, default on) applied to every instance in every layout at initialize, reset and the top of each block; without `--noise` the plugin is unchanged. `--max-iter` unset = auto-tuned (raised to `NODAL_MAX_ITER_FLOOR` = 100 on a nodal build), set = pinned; a pin below 100 on a nodal build is refused (the Armijo-globalized Newton needs the headroom to cross a saturation knee within a sample). The console and the `Build:` line report the budget the code ships
- `melange simulate` → parse → MNA → DK/nodal → render a WAV (`--input-audio`, or a generated 1 kHz sine at `--amplitude`); with `--input-audio` builds at the WAV's rate and refuses a conflicting `--sample-rate`; reads 16/24-bit PCM and float32, plain or WAVE_FORMAT_EXTENSIBLE; refuses a render with any unsolved sample unless `--allow-nr-hold`; `--inject FIELD=sine:<Hz>:<amp>|dc:<v>` drives a `.inject` field (`rate=host` once per host sample through the up-filter, `rate=inner` per sub-step; undriven fields are 0)
- `melange analyze` → frequency response with `--pot`/`--switch` overrides; `--harmonics` for THD/gain at clean bins. Default sample rate 48 kHz (as `compile`/`simulate`); `--freq <Hz>` measures one point instead of the log sweep; dBc values below −200 print `-inf`; `nyquist_dbc` is an fs/2 limit-cycle detector, not an aliasing measure; the closing `Steady state:` line says whether every point settled within the cap. Each point is measured at steady state at its own drive: driven for `--preroll-secs` (0.25 s) and re-measured until two windows agree within 0.1 %, capped by `--preroll-max-secs` (2 s; 0 = no check; off under `--noise`), with an unsettled point named in a warning. A point whose samples were not all solved (held, unconverged commit, reduced-model exit) is refused, as is the zero-drive settle before the first point (`--allow-nr-hold` reports it). `thd_pct` sums H2..HN below 20 kHz and below Nyquist (`nan` when none is in band)
- `melange validate` → compare against an ngspice reference (requires `ngspice`)
- `melange dc-op` → DC operating point; `melange nodes` → nodes, nonlinear devices, op-amps and controls (a `.wiper` shown as one control with its two halves)
- `melange import` (KiCad), `melange sources` / `builtins` / `cache` / `index` (circuit sources and the compiled-binary cache; the binary cache is LRU-capped, 2 GiB default, `MELANGE_BINARY_CACHE_MAX_MB`, 0 = unlimited; `cache clear --binaries`). A source index with `schema` above 1 is refused
- Output: by default a verb prints a one-line solver summary, what it wrote, `simulate`'s `Output peak: X V (Y dBFS…)`, and every warning; a solver counter prints by default only when nonzero and warning-worthy (`simulate` and `validate` alike; `nr_max_iter_count`, `region_exit_count` and other effort counters are `-v` only). Global `-v`/`--verbose` adds build steps, matrix sizes, the routing detail, rail-mode detail, every counter and the raw `DIAG:` lines. An unknown `.model` parameter is refused naming the card, its device class and its netlist line. `--solver` is checked (`auto|dk|nodal`) on every verb that takes it
- Plugin shipability flags: `--vendor`, `--vendor-url`, `--email`, `--vst3-id`, `--clap-id`
- Plugin level params: Input Level + Output Level (±24 dB), on by default; `--no-level-params` (or `--with-level-params=false`) to opt out

### Validation & Quality
- SPICE validation infrastructure (ngspice correlation); every verb (`compile`, `simulate`, `analyze`, `melange-validate`) assembles the circuit through one entry point, `melange_solver::build::build`, so a verb cannot check a different circuit than `compile` ships
- **The golden corpus is a change detector, not an accuracy oracle**: it compares melange against melange, so deck quality is irrelevant to catching a refactor that moves output. *Agreement* is robust to deck quality (a bad circuit cannot manufacture agreement between two implementations); *disagreement* is not. A broken deck may **detect** a defect; it can never **be** the evidence for one.
- Parser hardening: input-size caps (10M bytes, 50k elements, 1k models, 256 name len), non-ASCII normalization
- cargo-fuzz target (parser → MNA → DkKernel → CircuitIR)
- Error types: `#[non_exhaustive]` enums, no panicking library code
- Logging via `log` crate (no `eprintln!` in library code)
- Real-time safety: no alloc/locks/syscalls in audio processing, all buffers preallocated

## Performance

**Measured 2026-09-30** on an idle AMD Ryzen 9 7950X pinned to one CCD (single core, noiseless, `-C target-cpu=x86-64-v3`, via `tools/perf-harness/bench.sh`); host-dependent. Only `bench.sh` measurements attributed to a named deck and host are quoted here. Measured: nonlinear audio circuits ≈6.6–46× RT; light stages ~156× (single 12AX7); trivial linear ~2930× (7.1 ns/sample).

- Passive EQ (N=52, M=8, 3 xfmrs, nodal Schur): **~18.4×** realtime (1134 ns/sample)
- Wurlitzer preamp (2 BJT, full GP): ~46.1× · Tweed 5F1 amp: ~16.7× · Ge diode network: ~11.9× · bus comp (full, 12 op-amps + 2 VCAs): **~6.6×** · 12AX7 stage: ~156.0×
- **Attributed costs of shipped mechanisms** (same-session before/after on the same 7950X): the Dempwolf & Zölzer grid-current law costs the three triode rows 14–29 %; the full-LU gated exit step costs the bus compressor ~7.5 % (other rows within ±1 %; on a 0.1 V 1 kHz sine the full-LU corpus decks +3..+32 %, moonladder worst; at silence within noise); the nodal-Schur exact Newton start costs about one extra Newton iteration per sample on smooth signals (README nodal-Schur rows +7..+31 %), and is faster where the old predictor started badly (pipe-shouter 3344 → 1573 ns/sample).
- VCA compressor and the 8-BJT power amp have no `bench.sh` figure.

## Known Limitations

- Parasitic caps (10pF) auto-inserted across junctions for purely resistive nonlinear circuits
- Tube Koren: lambda parameter models finite plate resistance; no space-charge or transit-time effects
- BJT GP: no substrate current or avalanche breakdown
- Device temperature is per `.model` (`TAMB` on diode/BJT cards, scaled from a fixed TNOM of 27 °C with XTI/EG/XTB); JFETs, MOSFETs and tubes without self-heating run at 27 °C; no resistor TC1/TC2; `.temp` is ignored with a warning
- `MAX_M=32` — bound on NR dimension (the fully unrolled elimination's code size and compile time; every route refuses above it). A loop-based elimination is under Deferred.
- Full-LU NR + ill-conditioned A (cond(A) > ~1000): Schur preferred when K well-conditioned. No known circuit needs both pathological K and ill-conditioned A. See DEBUGGING.md "Known Full-LU NR Limitations"
- Linear coupled inductors use the exact `[L]` coupled-inductor path; the ideal-transformer T-model (leakage + ideal couplings + one magnetizing L) is built only for saturating two-winding cores, on nodal full-LU. The coupled-inductor approach is sufficient for the passive EQ at +1.8 dB.
- Saturating transformers of three or more windings need a stated core (`TURNS=` on every winding, `LM=` on one; SATURATING_TRANSFORMERS.md §2.2); without one they are refused, with the three-winding star split printed. A core with more than one flux path (multi-leg) cannot be stated.
- **Glow/neon relaxation-oscillator (`N … NEON(…)`, Phase 0c; EXPERIMENTAL; the glow work is parked).** Reset model = **Option A maintaining LINE** `i=(v−V0)/RS`, intercept `V0=VM−RS·IK` DERIVED (`.model NEON(VO VM IK RS IHOLD ROFF)`) — the reservoir-cap reset floor emerges at ~89 V (measured 93→88.95 V, both routes). Slope RS is `placeholder-pending-ZA1001` (ZA1004 form-transfer, sourced ~2.5–4.25 kΩ). **Root cause of the Philicorda VO=128 divide-fail (design review + control run, 2026-09-10): the RESET FLOOR parks too high (static V_m≈89 V vs cap-dependent V_m≈82 V), a DISCHARGE-side defect — NOT RS, NOT the strike/extinction model.** Control: lowering V0 ~6.5 V with RS held at 3k opens the both-hold (B5-lock ∧ B6-÷2) window at VO=128. A deionisation-state extinction model ("v2") fixes the self-extinguish CEILING (held-out Sheet C: v1 falsified, v2 confirmed) but STRUCTURALLY BREAKS the divider at every t_r (re-ignition depression → stages fire too easily), and field-dependence can't reconcile (~1.7× too weak); v2 is NOT deployed. **Real fix, when the glow work resumes = a physics-derived V_m(C) reset model, validated vs held-out Sheet C via pre-registered ceiling predictions (never Sheet B — retired).** VO=135 remains the working card (openphilicorda 12/12 boards, 49/49 keys offline).
  - **Edge timing (`--subsample-fire {auto|on|off}`).** `auto` = on for glow decks on nodal Schur; inert on DK and full LU (`on` is refused there), and every glow deck records `{mode, active, reason}` in its provenance header (`"dk-route"` on DK). A firing sample is split at the crossing fraction, strike and extinction resolved in temporal order, with lit-phase sub-stepping (default factor 1.0; `--subsample-lit-factor` is a diagnostic knob). Against the converged ngspice oracle the Philicorda divider gives 12/12 verdict agreement (11 divide 1:1; C8 an agreed 3:2 non-divide, a product model-completeness gap, not a mechanism exclusion), where a fixed grid at inner 96 kHz divided only 8/12 pitch classes. At the top of a divider chain the defect is injection-lock breakage, which oversampling cannot reach (12 pitches would need inner ≈768 kHz). Bounded per sample at 2·n_lamps + `LIT_SUBSTEPS_MAX`(32) + 2 segments, then safe abandon (`diag_subsample_fire_abandon_count`, asserted 0 in `subsample_fire_tests.rs`). ⚠️ The lit-phase τ is a conservative heuristic (`glow_lit_tau_min` = RS × largest terminal-node C-diagonal, min over lamps), a lower bound on the real loop τ (1.8–15× larger on the divider); its failure direction is CPU, not accuracy. The static lock margin was flat across an 8× lit sub-step range on that divider family (lock is set by strike timing and reset depth, not lit duration) — a finding for that family only; a latched device coupled by pulse WIDTH needs the sweep again, and static-pull margins are an upper bound on the vibrato margin.
  - **Edge aliasing:** strike/extinguish edges on a bare oscillator node alias at base rate. Anti-alias = whole-circuit oversampling; an output BLEP was rejected (ill-posed for a mixed multi-oscillator output) and a cosmetic output filter is forbidden by the accuracy-over-output-mapping rule. For RC-loaded dividers the reservoir cap band-limits the discharge (τ=RS·C≈30 µs → a fast ramp, not an ideal step), so base-rate aliasing is modest (an FFT estimate of ≈ −35 dB on a ~115 Hz divider, `glow_relaxation_tests.rs`) and 4× OS does not worsen it; OS=4 preserves the oscillator physics exactly. Recommend OS≥4 for glow-bearing decks. A trustworthy cross-divider alias study (a naive FFT ASR on a few-sample-period self-oscillator is artifact-dominated) + a listening pass are open.

## Validated Circuits

Circuit netlists live in a separate repository (public subset at https://gitlab.com/oomox-group/melange-circuits; full catalog private)
(locally `../melange-circuits`). Circuit-specific tests use `.test.toml` sidecars. All circuits
start in `unstable/`; promotion to `stable/` requires user DAW sign-off (SPICE correlation
and compilation are necessary but not sufficient).

### Against ngspice, converged reference (2026-09-30)

`melange validate` defaults on the 85 circuits-repo decks (`unstable/`,
`testing/`): 48 kHz, oversampling off, 1 kHz sine at 0.1 V for 1 s, the
reference driven by the analytic `SIN` and shown converged before anything
is graded against it (`reference.rs`: refined in maximum step and `reltol`,
or `trtol` where ngspice cannot start at a tighter `reltol`, until both
refinements move it by at most 0.05 %, a tenth of the 0.5 % RMS tolerance),
compared on the requested clock, nominal values, isothermal, the
tube/JFET/linearized/parasitic twins as melange built them. Measured at
f36e99c.

**48 PASS, 9 FAIL, 1 reference unavailable, 2 reference not converged,
1 timed out, 24 refused or without a reference.**

- **PASS** (RMS, 1x): gold-press-cab 0.0001 %, steve-1073-presence 0.0001 %,
  sympathy-drive 0.0001 %, noyce-4558 0.0002 %, noyce-ne5534 0.0002 %,
  sympathy-frontend 0.0002 %, jfet-booster 0.0003 %, gold-press-mastering
  0.0004 %, mosfet-source-follower 0.0004 %, noyce-clean-rc 0.0004 %,
  noyce-triode-12ax7 0.0004 %, noyce-smps-ripple 0.0005 %, el84-single-stage
  0.0006 %, steve-1073-output 0.0006 %, gold-press-cartridge 0.0007 %,
  noyce-6bq5 0.0008 %, kt88-pp-stage 0.0009 %, twill-deluxe 0.0016 %,
  funkyinduct 0.0019 %, noyce-germanium-cluster 0.0022 %, passive-eq1a
  0.0044 %, steve-1073-eq 0.0047 %, steve-1073-eqpres 0.0047 %, velvet-elvis
  0.0064 %, noyce-tape-head 0.0068 %, noyce-ef86 0.0088 %,
  noyce-carbon-comp-bank 0.015 %, sad-bastard 0.021 %, tube-preamp 0.022 %,
  gold-press-overdrive 0.022 %, farfisa-voicing 0.064 %, wurli-power-amp
  0.071 %, noyce-cascaded-triodes 0.094 %, basic-bitch 0.115 %,
  gold-press-riaa 0.117 %, twas-preamp 0.135 %, rc-lowpass 0.139 %,
  philicorda-voicing 0.181 %, noyce-transformer-triode 0.209 %,
  noyce-amp-at-idle 0.210 %, pretty-baby 0.242 %, pipe-shouter 0.247 %,
  jeffreys-tube 0.247 %, warpony 0.285 %, moonladder 0.290 %,
  mosfet-choke-load 0.321 %, noyce-boiler-room 0.391 %, vurli-leveler
  0.417 %. Reference self-checks 0.0001–0.022 %.
- **FAIL, backward Euler pinned by the deck** (`.integrator be`, first
  order): wurli-preamp 0.675 %, tungsten-glow 0.611 %, steve-1073-preamp
  2.05 %. steve-1073-preamp clips into pulses with ~18 V edges; 95–99.5 % of
  its error sits on edge samples, mostly one sample per edge where the
  output leaves its flat top between samples. Its trace is open (see
  SPICE_VALIDATION.md, rate sweep).
- **FAIL, 1x discretization:** noyce-cascade-idle 2.95 % (THD −12.8 dB,
  heavily clipped). With a stimulus incommensurate with the rate (1001 Hz)
  its error falls monotonically, 4.28 / 1.33 / 0.81 / 0.26 % at 48 / 96 /
  192 / 384 kHz, unaligned equal to aligned: it converges; at exactly 1 kHz
  the clipping corners fall at a fixed sub-sample phase per rate and the
  rate sweep reads a false PLATEAU.
- **Reference unavailable:** rexi-mockup. At 48 kHz its reference converges
  only through `trtol` (self-check 0.027 %) and validate reads 3.67 %; at 96
  and 192 kHz ngspice cannot start the transient at either tighter tolerance
  ("timestep too small" at 16 ns), so the rate sweep has no converged
  reference and the error is not classified as melange's.
- **FAIL, no meaningful comparison:** noyce-zener-junction (the THD gate on
  a nanovolt-level output; RMS 0.033 %), wurli-preamp-okona, radio-am,
  radio-fm (reference exactly 0), philicorda-voicing-coupled 13.7 % (the
  phili work is parked).
- **Reference not converged** (no verdict given): philicorda-master is a
  free-running oscillator whose phase moves between refinements;
  farfisa-se15-reverb's output is at the nanovolt level. champ-5f1's
  reference was still refining after half an hour (an hour in an earlier
  run), and the run was stopped.
- **Refused or no reference:** devices ngspice cannot simulate (VCA, LDR:
  4kbuscomp, 4kbuscomp-audiopath, gravity, gravity-stereo, farfisa-mtb,
  farfisa-swell, farfisa-se15-preamp, six philicorda note boards); no `in` /
  `out` node (farfisa-fa10, -fd10, farfisa-g10-ref, -3key, wurli-tremolo and
  the three `*-detailed` libraries); a dangling node (philicorda-divider); a
  variable-mu pentode (6k7-varimu-stage, no twin); ngspice abandons the
  transient (axe-15); a `.linearize`d cathode follower driven out of its
  region (basic-bitch, sad-bastard).

### Per-circuit notes

- **Passive tube EQ** (passive-eq1a): 4 tubes, 3 transformers, 7 pots, 3 switches, global NFB. Amp § from Sowter DWG E-72,658-2. N=52, M=8, nodal Schur (~18.4× RT, Performance). Flat ±1 dB 20Hz–15kHz, 21 dB differential NFB.
- **Wurlitzer 200A preamp** (wurli-preamp): N=13, M=5, 2N5089 Ebers-Moll, nodal Schur. Against a converged ngspice reference at 48 kHz with the deck's pinned backward Euler: 0.675 % RMS (table above).
- **Wurlitzer 200A power amp** (wurli-power-amp): N=23, M=14, quasi-complementary class AB, nodal full LU. Validate: 0.071 % RMS (table above).
- **Tweed-style 2-stage 12AX7 preamp** (twas-preamp): N=14, M=4, DK. 50 mV → 549 mV (+20.8 dB). Zero NR divergence.
- **VCA bus compressor** (4kbuscomp): 12 op-amps, 2 VCAs, 6 diodes, 2 pots, 2 switches; nodal full LU. The DC OP's precision-rectifier basin is handled by post-fallback refinement NR (`b771512`); the transient's chord-NR false convergence on DC-railed op-amps by the residual check on ActiveSetBe/ActiveSet (`c3d3eae`). Long renders also depend on the deck's TL07x swing: `.model OA_TL074 ... VSAT=13.5` (TI datasheet swing on ±15 V); with the earlier VSAT=11 it was stable only to d ≤ 2 s. See DEBUGGING.md "ActiveSetBe Chord-NR False Convergence" and "Precision Rectifier DC OP Convergence".
- **VCR audio ALC compressor**: N=21, M=3, nodal full LU. Key: 100Ω Rdecouple between VCA sig- and I-V converter fixes positive K diagonal.
- **4-op-amp overdrive with diode clipper**: verified bounded under ActiveSetBe at amp=[0.01..0.50]; auto resolves ActiveSet, not re-run on this deck. BoyleDiodes opt-in only (heavy-clip divergence at amp ≥ 0.05 unsolved — not a blocker, see DEBUGGING.md).
- **Single-op-amp diode-clipper overdrive** (pipe-shouter): PASS against ngspice, 0.247 % RMS (table above).
- **Pentode stages**: EL84 single stage, a 6V6GT push-pull stage with output transformer (twill-deluxe), 6K7 varimu (no ngspice twin; see Deferred), a 4×EL34 + 3×12AX7 power amp (grid-off M=18→14 only under `--tube-grid-fa on` — full 3D by default routes it nodal). twill-deluxe, el84-single-stage, noyce-6bq5 and noyce-ef86 PASS full 3D against a converged ngspice reference (table above).

## Pending Work

- **After the cleanup release (ruled 2026-10-01), in no fixed order:**
  - **pnjlim is skipped for parasitic BJTs with internal nodes in the DC-OP Newton** (`dc_op.rs`: `if internal_junctions && bp.has_parasitics() { continue }`). The limit's correction, spread back to the nodes through `N_v^T`, pushed the weakly-tied internal base far from the external one, so for these devices the correction is dropped and the parasitic resistances in `G_aug` are left to damp the step. Without the limit a bad start can overshoot a power junction by volts. Measure with and without it on the parasitic-BJT corpus and on the wurli power-amp snapshot (`tests/data/wurli_power_amp_snapshot.cir`, the alpha-floor fixture) before changing it.
  - **The baked node Gmin** (1e-12 S in the nodal `G`, none in DK's): the Deferred item "Baked node Gmin" below, measured the way the emitted second term was before its removal.
  - **validate's reference rungs in parallel**: a future opt-in; see "First-user gaps" below.
- **Throughput fell 9–13 % over v0.1.11..v0.1.12 on three README rows** (bus compressor 7.5× → 6.6×, tweed amp 19.0× → 16.7×, passive tube EQ 20.3× → 18.4×; idle re-bench 2026-09-30, `bench.sh`). Not attributed to individual changes. Bisect over the v0.1.11..v0.1.12 range with `bench.sh` on those three decks before any perf work claims a win. Measured 2026-10-01 (idle, interleaved, best of 7): v0.1.14 is equal to or faster than v0.1.13 on every README row (bus compressor 3196 vs 3221 ns, passive EQ 1148 vs 1190, Wurlitzer preamp 459 vs 500, single-ended amp 1047 vs 1104), so the loss sits in v0.1.11..v0.1.12 and was not added to since.
- **No aliasing measurement in melange.** `analyze`'s `nyquist_dbc` detects only a component at exactly fs/2 (a limit-cycle signature); aliases fold to `fs − k·f` anywhere. The docs teach render-and-compare with an incommensurate tone. A first-party inharmonic-product metric would close it.
- **First-user gaps (fresh-clone passes, 2026-09-30 and 2026-10-01).** Still open:
  - `validate` refuses a vanilla pedal deck on output-peak tolerance; candidate fix is named tolerance presets.
  - Did-you-mean suggestions miss transposed letters (low priority; unknown names are already refused loudly).
  - The SI infix form (`4k7`) is accepted at parse time without comment (melange reads 4.7 kΩ); only `validate` refuses it, because ngspice reads `4k7` as 4 kΩ.
  - The built-in demo triggers its own `--no-dc-block` advisory on `compile`.
  - `validate passive-eq1a` takes about 3 minutes, 99 % of it three sequential ngspice reference runs (35.5 / 68.0 / 67.3 s; running the reference-refinement rungs in parallel is a future opt-in, ruled: left as is for now).
  - Generated plugin pot knobs map linearly in ohms (`PLUGIN_GUIDE.md` shows where to reshape the taper).

  Closed: default output trimmed to the solver summary, what was written, the output peak in V and dBFS, and warnings, with the rest behind `-v` (`6686606`); `analyze` defaults to 48 kHz and takes `--freq` (`6686606`); `compile --stereo` (`7fd2b5d`); the plugin "Circuit Noise" parameter (`dbcf56a`); the compiled-binary cache is LRU-capped at 2 GiB (`MELANGE_BINARY_CACHE_MAX_MB`, 0 = unlimited).

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
- **Runtime latch misses a ring under a loud program — trial latch evaluated and REJECTED (2026-09-30).** The lag-1 entry test cannot see a Nyquist ring under program (steve-1073-preamp unpinned: 1.0 V at 48 kHz, 2.1 V at 96 kHz after every onset; never latches). A trial latch (trip on a demodulated Nyquist estimator, 3 ms of backward Euler, keep it if the content collapses) separated every witness once the program reference was corrected, but a trial inside wurli-power-amp's start-up transient bent the output by −39 dB of the program, and it was rejected on that. Reopen on a trial-scheduling rule derived from the circuit, not tuned. Until then, edge-dominated decks at 1x need a static `.integrator be` pin. Design, tables and witnesses: [RING_PREDICATE.md](RING_PREDICATE.md), "A ring under a loud program".
- **L-stable integrator (BDF2 / TR-BDF2) — reopen evidence recorded (2026-09-28), not scheduled.** A permanently stiff linear mode (sat-core-open's open-secondary leakage mode, trap factor −0.996 / −0.998 at 48 kHz) keeps the runtime latch engaged for the rest of the stream once excited, and a latched stream costs up to 15 % on H3 at 1× (table above). An integrator that damps stiff modes without BE's first-order error is the remedy, not a latch release. Re-measure the latch's firing set after the charge form: the case is "a latch that fires only on permanently stiff linear modes". Second deck (2026-09-29): noyce-transformer-triode keeps trapezoidal under the ring predicate (single-event ring −63 dB, 45× better in-band sine error than BE) but its stiff mode (z = −0.99997, τ 0.7 s) accumulates under phase-coherent even-period clicks to −46 dB, and in the default build the latch engages 9 ms after the first impulse of the 0.1 V hostile program and holds BE for the rest of the stream.
- **Re-measure under the charge form (opened 2026-09-29).** Each mechanism below is still emitted; whether each still earns its keep now that the capless-row walk is gone is unmeasured. Do not remove any without its own measurement:
  - **The gated chord exit step** (full-LU, one refactored Newton step when a chord-accepted iterate's node-row residual exceeds 1e-9 A): its reason was the per-sample residual feeding the walk.
  - **The runtime BE-latch**: re-measure its firing set (sat-core-open 4× fire was the algebraic walk); feeds the latch-release and BDF2 items above.
- **Tolerance units, for the one-definition work:** node steps are tested in volts, residuals in amps. On high-conductance rows the volt tolerance is loose in current terms (a 0.64 mV node tolerance on a stiff diode row is ~45 µA); a current-residual acceptance is the other principled form (a larger change, not taken). The pinned resolve (both sub-paths, rail plateaus) accepts on the node step (`1e-3·|v| + 1e-6`), so the committed pair is off KCL by about `½·g′·dv²`.
- **The ring verdict is taken at the compiled rate** (2026-09-29). A host rate above it is conservative (stiff rings decay faster, residues shrink); below it slightly optimistic. The latch's memory follows `set_sample_rate`; the route does not. Open: whether a runtime rate change should re-evaluate the verdict (the stored continuous poles make `|z|` cheap; the residue moves too) and flip the route through the BE machinery the latch already uses. [RING_PREDICATE.md](RING_PREDICATE.md).
- **Index-2 decks (an exact `z = −1` under trapezoidal: champ-5f1, noyce-smps-ripple, sat-core-loaded, wurli-power-amp)** — an analog-EE modelling question, not a solver one: an inductor-only cutset that trap never damps suggests the netlist idealises away a physical parasitic (winding capacitance, core loss). Until decided, the runtime latch holds its program reference on these decks, so a ring a quiet passage excites long after a loud one under-latches (conservative). [RING_PREDICATE.md](RING_PREDICATE.md).
- **Runtime half of the unsolved-sample coverage is downstream.** A kick-started or driven regenerative circuit is caught at runtime by `diag_unsolved_sample_count`, not at compile time (see Codegen Solver Routing, "Self-starting oscillators stay off DK"). In a plugin that coverage exists only if the plugin's tests assert the counters stay zero. As of 2026-09-29 oomox reports 15 circuit plugins that assert no solver-health counter at all (oomox is adding the assertion deck by deck at regeneration).
- **Validate fails 36 of the 85 `unstable/` decks at default settings (triage 2026-09-30, open).** Default `melange validate` (node `out`, input `in`, 0.1 V 1 kHz sine, strict, 1 s). Most failures say nothing about solver accuracy:
  - **Not about melange, by construction (21):** 12 refusals of elements ngspice has no model for (VCA, LDR, neon lamp), which can never pass; 5 decks with no `in`/`out` node (3 of them pass at the ports their test sidecar names); 4 decks not validatable as written (1 broken deck with a dangling node, 3 subcircuit-only libraries with no top-level circuit).
  - **Reference, twin or stimulus limits (8):** a variable-mu pentode (no twin); four where the ngspice reference is not converged or ngspice abandons or does not finish the transient; two decks silent at their defaults (a `.runtime` carrier defaults to 0); one `.inject`-driven deck the plain input does not drive.
  - **Candidate model mismatches (7):** rexi-mockup (3.67 % RMS), noyce-cascade-idle (2.95 %), steve-1073-preamp (1.43 % at 0.25 s, 2.06 % at 1 s), zener-feedback-limiter (1.18 %; op-amp GBW not modelled as a pole, not attributed), tungsten-glow (0.607 % against 0.5 %), noyce-zener-junction (THD at a µV-level output, possibly a numeric floor), philicorda-voicing-coupled (13.7 % RMS at a µV-level output; parked). steve-1073-preamp is out of region on almost every sample and melange ignores its BJT `TR` (reverse transit time), which ngspice models.
  - **Harness asymmetry:** validate refuses a deck when ngspice ignores a model parameter, but only warns when melange ignores one (the `TR` case above), so it compares two different circuits. A THD comparison that cannot be measured says which side lacks a fundamental and whether it is silent ("THD not measurable: …").
  - Still to do: decide whether wurli-power-amp's (now `testing/`) 20 mV absolute peak tolerance means anything on a stage swinging tens of volts (0.09 % RMS, correlation 0.9999996, fails at 33 mV). Fixtures: `.linearize` runs against ngspice in `linearize_twin_tests`; `.inject` has a rustc-only oracle (`inject_oracle`, no ngspice comparison; its four tests run by default); `.integrator` has no validate fixture.
- **(iv) A self-starting two-transistor astable is refused at its first regenerative switching edge, on nodal, under either integrator (2026-09-30).** Primary witness: the textbook NPN astable (9 V; 1k collector loads; 47k base resistors to the supply; 100 nF cross-coupling caps, one with IC=5; NPN IS=1e-14 BF=100 CJE=10p CJC=4p TF=0.3n; converged ngspice period 6.667 ms, c2 0.03..9.00 V). The DC OP is correct (source stepping) and samples before the edge solve in 1–3 Newton iterations. At the first fold (Q2's base reaching ~0.55 V) the Schur Newton cycles, the sub-step ladder fails at T/2^12 within its 64 attempts, the backward-Euler fallback fails, and the hold then re-poses the identical problem: every later sample unsolved and refused (23,938 of 24,000 at 48 kHz on trap; 4,738 of 4,800 under `--backward-euler`, 9,109 of 9,600 at 96 kHz). A deeper ladder (T/2^16, 4096 attempts) crosses the fold but settles into a 4-sample limit cycle, not the astable, so it is not a fix. It routes nodal because its DC OP has a growing pole (the self-starting-oscillator refusal on DK); 0.1.11 built it on DK and matched the period (6.667 ms) with ~7 % of samples capped and a 1.3 V overshoot. Passing contrast: the IC-seeded G10 divider astable crosses its folds on both nodal sub-paths with 0 held samples (below). The lead: G10's PNPs carry RB/RC/RE and VAF under moderate bias, which soften the regenerative fold; the ideal-ish NPNs hard-saturated through 1k/47k make the sharpest fold. Whatever (iv) builds must pass this deck. **Refused at the default Newton budget only (measured 2026-09-30).**
  - **Raised budget on the witness:** `--max-iter 1000` solves every sample, exit 0. The render is a spurious cycle 4–6 samples long: 0.083 ms (no IC) / 0.125 ms (IC=5) at 48 kHz, c2 up to 10.7 / 12.1 V on the 9 V supply.
  - **Not route-specific:** identical on forced Schur and full-LU (0.0833 / 0.0832 ms), so it is a genuine solution of the discrete step equations, and no counter moves. Ruling it out is what (iv) has to do.
  - **Split by model term** (no IC, `--max-iter 1000`): no junction charge → 249 unsolved, loud; `CJE`/`CJC` only → cycle, 5 unsolved, loud; `TF` only → cycle, 0 unsolved, silent.
  - **`TF` only, other rates:** 96 kHz 0.148 ms, 192 kHz 0.469 ms (c2 −3.5..10.2 V), OS4 0.854 ms, all 0 unsolved. ngspice does not settle on the `TF`-only variant (0.008–6.5 ms across TMAX/reltol), so only the full witness has a reference.
  - **Without junction or transit-time charge** (IS=6.734f BF=416 VAF=74), `--max-iter 1000` solves correctly: 6.687 ms vs ngspice 6.693 ms.
  - **v0.1.11, built from the tag:** auto-routed this deck to DK, `--max-iter 1000` 6.684 ms, c2 −0.045..9.77 V; forced nodal failed loudly (417 of 14400 unsolved at 1000, c2 to −2027 V at the default).
  - **Open sub-question:** whether the `capbe` diffusion-cap change (a6c624a) contributes to the silent nodal cycle, or only the nodal-path changes since 0.1.11 do. Not separated.
  - **CLI warning:** the NR-starvation WARNING does not recommend a bare "try 1000"; it says a larger budget can settle on a spurious oscillation on oscillator/switching circuits.
  - **Candidate safeguards, not built (ruled 2026-09-30):**
    - a short-period limit-cycle detector. Objection: forced program content at 4–6 samples per period is legitimate at 1× for HF material, the same class of feature-based output test that failed for the trial latch;
    - a rail/stored-energy sanity check. Objection: inductive kicks and transformer secondaries legitimately exceed the supply.
- **(iv) DK containment parity (logged 2026-09-29, not scheduled).** DK has no unsolved-sample containment (a timestep cut at a fold); both nodal sub-paths run the same sub-step ladder (local refinement down to T/2^12, at most 64 attempts per sample). Whether DK gets it or gives way to nodal on nonlinear switching decks is open.
  **HARD COUPLING: (iv) must NOT land without the exact seed.** Once DK can sub-step, its failed jumps stop being loud: a rescued sample can converge on whatever branch its start favours, which is exactly the nodal-Schur 192 kHz situation. So when (iv) is built, the exact seed goes in with it, gated on the same witnesses. The DK exact seed is built and parked on branch `dk-exact-seed-for-iv` (f317d10).
  Why DK keeps the first-order predictor until then: without regeneration (positive feedback) each step's equations have a single root, so the start changes the iteration count, never the answer. With regeneration DK has no rescue, so a failed jump exhausts MAX_ITER and is counted in `diag_unsolved_sample_count` and refused: loud. The remaining gap is a start that converges onto the wrong branch within MAX_ITER on DK; no DK render built so far shows it (the IC astable and a driven BJT Schmitt trigger are refused first), and the RHP gate plus routing keep such decks off DK by default. The exact seed measured on DK (2026-09-29): +21–100 % CPU on every DK deck (noyce-triode-12ax7 184 → 284 ns/sample), no deck faster, and more loud failures on the regenerative decks DK cannot solve (Schmitt trigger at 192 kHz: 189 → 764 unsolved).
  **Reopen trigger before (iv):** any DK render with 0 unsolved samples but a switching-edge mismatch against a settled reference. Witness: the IC-seeded G10 astable fixture in `cli_integration.rs` — on DK 1330 of 2400 samples unsolved at 1× (tests pass `--allow-nr-hold`, guard boundedness only).
  No counter can see a branch jump in general; an LTE check at edges is the open item. (Nodal Schur now starts its Newton at full-LU's point and reaches the same root at a regenerative fold: `cli_integration::test_ic_seeded_astable_schur_period_matches_spice`.)
- **Op-amp pin outcome differs by nodal sub-path (2026-09-29, unification item).** On a Schur build with devices an engaged pin decides the sample (`converged` = the pinned solve's outcome, so a converged pin clears an unpinned failure and a failed pin goes to backward Euler, then the hold). Full-LU runs the pin only on a converged sample and commits a failed pin (`diag_nr_unconverged_commit_count`). Both are counted in `diag_unsolved_sample_count`; the rule itself is not yet one definition.
- **five-watt-freddie (champ-5f1, junk-grade deck): trapezoidal Newton fails on almost every sample** (2026-09-29, low priority). The old auto-BE promotion hid it; under the ring predicate the deck is trapezoidal (no lasting Nyquist-side pole) and the golden renders show the BE fallback on 95 615 of 96 000 sine1k samples and 87 147 of the sweep, the runtime latch engaging, and the sweep reaching the output clamp (5 samples). A convergence defect of trapezoidal Newton on this deck, not a ring: attribute it (which rows fail, first failing sample) before anything routes around it.
- **Hard-switching edges and the trapezoidal `z = −1` mode — re-measure under the charge form.** Under the whole-system form, ANY hard-switching edge (not just a `.switch` flip) re-excited the trap `z = −1` mode on capless subspaces: BJT edges in the g10 divider re-excited it every ~160 samples, and openfarf's residual wandered 0.9–4.9 % with no decay. The charge form gives capless rows no memory, so the mode should be gone there; re-measure before closing. Grader for this case: CLI == codegen window-for-window on the g10 chain keyed closed (not a 1e-7 target).
- **Section glow is route-dependent; its full-LU parity test is ignored** (`glow_relaxation_tests.rs`, `test_glow_sections_route_portable_schur_vs_full_lu`, `#[ignore = "KNOWN GAP: …"]`). A relaxing-section / delayed-overvoltage / KSUB lit branch (`has_sections()`) is honoured on nodal Schur (`glow_lit_eval`), but the full-LU device evaluation solves the static maintaining line `i=(v−V0)/RS` while the strike seed and extinction test read `glow_lit_eval`: a mixed model within one sample. Codegen therefore refuses such a deck on full LU (`--allow-static-glow-on-full-lu` makes the sections inert, for diagnosis). Gating the full-LU sites onto `glow_lit_eval` is not the fix: on the test's synthetic deck (K1=1, KSUB=2, C=10n, 48 kHz) the outer Newton's fixed-point gain ≈ I·R_thev/|s_eff| exceeds 1 and the deck goes dead (strikes once, sticks lit); with the gate it oscillates only at ≥768 kHz. Real ZA1001 key sets (Σk≈43.3, |s_eff|≈41) would converge, and the real divider routes Schur. Deferred behind the glow work; when scheduled, validate on the real rig at a real rate (read `nr_max_iter_count` and the route line beside every number), then un-ignore the test as the acceptance gate.
- **Sub-sample fire: remaining items, dormant with the glow work.** Ship-path CPU for the divider route decision (nodal Schur + subsample-fire at 96 kHz vs DK at 16× oversampling, worst-case and steady-state µs/board/sample, against openphilicorda's 1.74 µs/board/sample bar); oracle convergence two halvings below the envelope; and the consumer's on-path lock CI test. All dormant until a nodal-routed divider ships: openphilicorda's compiled route is DK, where the feature is inert (`"dk-route"`).
- **steve-1073 channel decks** (circuits repository): EQ section, integration, plugin. The two amplifier stages are SPICE-validated (table at the top).
- **Performance ideas, not scheduled**: hot/cold state split; a fast `powf` for the Koren tube model (accuracy-gated like every optimization).
- **Multi-language codegen**: `Emitter` trait + `CircuitIR` are language-agnostic by design. Only Rust is emitted today. Planned: C++ (the next roadmap target), then Python/NumPy, MATLAB/Octave. **FAUST: explored, ruled out (2026-09-02)** — FAUST's generated code is not Turing-complete by design, so a data-dependent NR iteration count is inexpressible; only circuits emitting no NR loop at all would work (6 of 41 corpus circuits). Note the predicate is "no NR loop emitted", NOT `M == 0`: behavioural B-sources route nodal and get Newton regardless of M.

### Deferred
- **Measurement policy for `analyze` (agreed 2026-10-01; post-release work).**
  Drift between overlapping measurement tools has come from definitions, not
  duplicated code (`thd_pct` once summed to Nyquist; `nyquist_dbc` was quoted
  as an aliasing figure). The plan:
  1. **Scope line.** `analyze` characterises the circuit's response to its own
     stimulus (gain, phase, THD vs frequency) and refuses points that are not
     solutions; it adds no instrument-class meters (aliasing, loudness, IMD,
     decay, noise floor, level of an arbitrary capture), which belong to a bench
     instrument. State this in `analyze --help` and the user docs. (User docs
     done: README "Simulate Without Compiling", GETTING_STARTED. `analyze
     --help` does not state it yet.)
  2. **Definitions are canonical in melange's public docs**, stated
     explicitly: THD = harmonics H2..H13 that fall below 20 kHz (and below
     Nyquist), each in dBc of the fundamental.
  3. **Agreement test.** Checked-in fixture renders plus an external bench
     instrument's JSON for them (keyed on its source and exe hashes; a missing
     or renamed key fails, never passes); `analyze` must agree within: THD
     |Δ| ≤ max(0.1 % relative, 0.002 percentage points); each harmonic above
     −80 dBc |Δ| ≤ 0.05 dB; harmonics below −80 dBc reported, not gated.
     Fixtures are refreshed only deliberately.
  4. **Optional external-analyzer hook** (only if a headless analyzer binary
     is distributed): configured explicitly (no auto-detect), every figure
     labelled with its instrument, refused if requested and missing, and no
     figure ever derived from both instruments' numbers.
  5. **Rename `nyquist_dbc`** to a name that says what it measures (a
     limit-cycle detector at exactly fs/2, e.g. `fs2_limit_cycle_dbc`), with
     the old name a deprecated alias for one release; correct any doc that
     quoted it as aliasing in the same change.
- **Generated terms are chosen by structure, never by magnitude.** The K and S
  patterns come from the topology (`structural.rs`; LINEAR_ALGEBRA.md
  "Structural Sparsity"), so a roundoff entry outside the pattern (~1e-19 on
  farfisa-se15-preamp's `K_BE`) is set to zero and emits nothing, while a
  genuine small coupling inside it (gravity-stereo's 1e-24..1e-37 DC-path
  entries) is always emitted. A numeric cutoff would get both wrong.
- **Seed parasitic-BJT internal nodes from the `.linearize` bias point.** The linearized circuit's DC solve starts at the bias point by node name, but parasitic-BJT internal nodes (RB/RC/RE) still get their fixed-offset initialisation (emitter at base − 0.65 V), so decks with them converge in more than 2 iterations (wurli-power-amp 9, farfisa-se15-preamp 21, against 2–3 elsewhere). Seeding them from the bias solve's device state would make the seed complete. Low value: those decks already converge.
- **Loop-based Gaussian elimination above the unrolled range.** `MAX_M` (32) exists only because every route emits its Newton solve as fully unrolled elimination, about M³/3 statements. A loop-based elimination for M above the unrolled range would remove the per-M ceiling, leaving the real cost limits. Input, measured 2026-09-30 on a synthetic diode-pair ladder (Ryzen 9 7950X, `rustc -O`, x86-64-v3, one codegen unit; perf-harness ns/sample): M=24 → M=32 source 346 → 547 kB (DK), 596 → 918 kB (Schur), 466 → 677 kB (full LU); compile 0.48 → 0.70 s, 3.18 → 5.68 s (663 MB peak), 2.54 → 3.53 s; 5.9 → 9.9, 6.3 → 10.3, 6.8 → 8.9 µs/sample. Not queued.
- **Pentode plate kink at `Vpk = 0` (model question, for analog-EE review).** Below `Vpk = 0` the pentode models hold the plate current at zero; above it the current rises with a finite slope, so the plate current has a derivative jump there. Newton straddling it can cycle between the two sides: on axe-15 (push-pull EL84 whose plate the transformer drives to the cathode), six samples at 0.1 V exhaust the primary Newton in a period-2 cycle with the root at `Vpk ≈ 0.27 V`; the sub-step ladder resolves all six. Whether a real pentode's plate current near `Vpk = 0` should be smoothed (and how) is a device-physics call, not a solver one; nothing changed.
- **ngspice twin abandons two tube decks.** `validate` has no reference for axe-15 (ngspice stops at 3.57 ms, "timestep too small", trouble with node `pi_p`, the cathodyne plate) or champ-5f1 (0.47 ms, node `el_p`). melange renders both. Twin-side: the tube B-source translation near the plate floor is the first suspect; not traced.
- **Variable-mu pentodes have no ngspice twin.** `validate` refuses a pentode card with `SVAR > 0` (`melange-validate/src/pentode_translate.rs`, sharp pentodes only) rather than modelling it sharp, so the corpus's 6K7 stage (`6k7-varimu-stage.cir`) has no reference: it is listed under Validated Circuits but was never compared against ngspice. The twin needs the Reefman §5 two-section Koren blend written as B-source expressions, mirroring the plate and screen equations codegen emits for `svar > 0`. Like the sharp twin it would arbitrate the solver, not the model. Not queued.
- **ls_fail rising with clean convergence (warpony, wurli-power-amp).** The Armijo line search's failure counter roughly doubled on wurli-power-amp once every nodal build expanded its parasitic internal nodes (golden sine1k 12384 → 22014, sweep 6455 → 17039) while every render converged and matched; warpony showed the same pattern earlier. The line search's merit or acceptance may be rejecting steps Newton then finishes anyway: CPU left on the table, or a merit that no longer matches the convergence definition now that it covers the internal rows. The same deck's sweep render also reaches the Newton iteration cap more often, one sample per change to its linearized Vbe multiplier or its PNP caps on 2026-09-29 (3 → 4 → 5 → 6 of 192000), then back to 3 with the SPICE diffusion capacitance; each recovered by a sub-step, none held, with no audible change. A trajectory perturbation at a few hard samples, not a trend. Trigger to look: the count keeps climbing under unrelated changes, or any sample is held. For a later look; not queued.
- **Sub-step ladder has no truncation-error control (parked, by ruling).** The nodal sub-step ladder refines a sample only when its Newton fails, never for accuracy: it solves its step equations correctly (checked against a converged 256× reference, 64× agreeing to 0.46 mV), but at whatever sub-step Newton converged at, so a rescue can cross a fast event at T/8 and carry that step's truncation error. Measured 2026-09-29 on wurli-power-amp with expansion forced and the pre-`4219abb` row gap: rescued samples 27 mV rms off the 256× reference against 7.8 mV for ordinary steps; one rescue that bisected to T/64 landed within 1 mV. Base-rate steps across the same events have no LTE control either, so accurate rescues alone would be inconsistent; the accuracy lever for fast events is oversampling, or a global step control (a much larger design question). (The rescues on that deck were themselves a symptom of the row gap `4219abb` closed; on the fixed code it does not rescue.)
- **OPEN FINDING: pipe-shouter has no DC operating point under `--opamp-rail-mode boyle-diodes`** (explicit-only mode; the build refuses it, `--allow-unconverged-dc-op` overrides). State at `5dd0dcc`: Direct NR, source stepping and Gmin stepping each fail (200 / 400 / 200 iterations; KCL residual 2.5e3 A at an internal row). Two single-supply JRC4558 stages (`VCC=9 VEE=0`): the linear start puts each Boyle internal gain node (Gm 0.2 S against its 1 µS self-load) near 900 V, the junction clamp then moves the source-fixed catch-diode reference with it (the source row restores it on step 1), and Newton does not recover. Unchanged by the in-Newton rail active set, the grounded-junction clamp and the held-pin port; the other rail modes converge on this deck. Likely the same internal-node conditioning as "The BoyleDiodes heavy-clip problem" in OPAMP_RAIL_MODES.md. Not traced further.
- **DC-OP cold start: SPICE's zero-node start with `MODEINITJCT`** (the principled alternative to the junction clamp in `clamp_junction_voltages`, see DC_OP.md "Direct NR"). Take it if the clamp meets a topology it cannot handle. Measured 2026-09-29 as `MODEINITJCT` layered on the LINEAR-GUESS start (not SPICE's pairing, whose node vector starts at zero): grounded-emitter BJT control 193 → 11 iterations (the clamp: 7), but the Wurlitzer preamp DirectNr 8 → Failed 211 (all three ladder strategies fail; excluding parasitic BJTs gives SourceStepping 194), and +1..+7 iterations on gravity, 4kbuscomp, moonladder, pipe-shouter, opamp-pin-control, 1073, zener. The zero-node pairing itself is unmeasured.
- **Model question (recorded 2026-09-30, for analog-EE review; low priority): JFET `CGS`/`CGD` as SPICE depletion capacitances.** melange stamps them as constant capacitors; SPICE's level-1 JFET makes each a junction depletion capacitance, `C(V) = C0 / (1 − V/PB)^(1/2)` with `PB` and the forward-bias linearization at `FC`. pF-scale at audio, so the effect is small; implementing it means `PB`/`FC` keys and a bias-dependent cap on both routes, as the BJT's CJE/CJC have. Meanwhile validate's reference mirrors melange's constant caps (`jfet_translate.rs`), and the constant law is documented in `docs/spice-grammar.md`.
- **Model questions behind refused card keys.** The key-effect suite (`model_key_effect.rs`) refuses, when nonzero, a triode card's variable-mu keys (`MU_B`/`SVAR`/`EX_B`, which reach neither the DC OP nor the transient: both evaluate the sharp Koren law) and a pentode's `LAMBDA` and `RGI` (stored but read by neither). Variable-mu on a triode, a lambda term in the Derk pentode law, and an internal-grid solve for the pentode's control grid (the triode's `evaluate_with_rgi` with the 3D chain rule) are model questions, not built until a deck needs one.
- **BJT DC-OP seed at a fixed junction current (parked, by ruling, 2026-09-29).** The alternative to seeding every BJT junction at a fixed 0.65 V (`clamp_junction_voltages`): seed each at the voltage that carries a reference current, the current IS = 1e-14 A carries at 0.65 V (about 0.8 mA), i.e. `NF·Vt·ln(I_ref/IS)`, stated in code as that current. Not needed now: with the IS-aware junction exponential (`a061e40`) alone, an NF = 1 card swept from IS = 1e-13 to 2.9e-22 and a FET-limiter output stage (NF = 2) all converge on Direct NR in 10–26 iterations. The stall it was meant to cure was Newton overshooting into the old flat clamp at x = 40, not the seed. Reopen when a deck's DC OP is slow or fails and traces to the fixed 0.65 V seed; the diode's 0.6/0.8 V seed constants would be measured the same way.
- **G10 divider hard-switching NR overshoot** (root-caused 2026-08-14, user-deferred as non-blocking). Germanium-PNP astable divider (`melange-circuits/local-docs/repro-ic-vcvs-blowup.cir` / `repro-wav-input-blowup.cir`) overshoots to ~80 V on an 8 V rail under a full-strength switching trigger (ground truth from melange-circuits: real full chain swings inside 0–8 V, duty 79.6%). Signature: BE-fallback storm (~88 % of samples vs ~1 % when physical). Root cause: at a switching edge the astable's positive-feedback K coupling pins a junction at v/vt ≈ 300 (v_d ≈ 7.8 V), `i_dev` saturates at `IS·exp(40)` ≈ 7e10 A, the Jacobian goes catastrophically ill-conditioned, and per-sample direct Newton stalls (‖f‖ flat at 7e10, exhausts MAX_ITER=90); the BE fallback inherits the same pinned state. Decisively ruled out by experiment: warm-start predictor, **line-search** (flat region, no descent direction), pnjlim sub-threshold-skip removal, and absolute v_d clamping. **Fix = port the DC-OP continuation (gmin/source stepping, `dc_op.rs`) into the per-sample `solve_nonlinear`** — substantial, higher-risk; validate against 24 SPICE + solver tests + oomox plugin-render golden gate. NOT blocking the working full G10 chain (which converges to rail). Prior "true Newton / Anderson" guess for this class is superseded by the gmin/source-stepping direction. **Acceptance criteria when this lands (from melange-circuits 2026-08-15):** (1) the ~80 V overshoot case stays bounded/physical; (2) NEW — *converged-vs-starved pitch agreement*: on the free-running G10 astable cascade, `--max-iter 70` (~11% NR starvation, which does NOT latch and looks healthy) leaves the master oscillator **3.8% flat** (a third of a semitone) with duty drifting ~3 points vs `--max-iter 1000`, because every non-converged sample leaves a slightly-wrong state that *integrates into pitch* on an oscillator. A failure-fraction warning (see the NR-starvation warning, `fe5c12a`) fundamentally cannot catch this sub-threshold detune class — the continuation is the real fix. See memory `g10_divider_hard_switching_overshoot_2026_08_14`.
  Note before reopening: a per-sample Gmin continuation was built and reverted for marginal astables (net-negative on every form; DEBUGGING.md "Hard-Switching NR Starvation on Marginal Astables"), and the local sub-step ladder now crosses the IC-seeded G10 astable's edges with 0 held samples. Re-measure this repro on current code before choosing a direction.
- **Per-timestep junction-charge re-linearization** — blocks `TR`, would make `TF` exact. Build charge-first; see `DEVICE_MODELS.md`.
- **Multi-input** is restricted to linear (`M=0`) circuits, which is exactly the case superposition already covers — so the nonlinear-mixing case it exists for is unreachable, and no deck uses it.
- Phase 6a/6b type safety (NodeIdx newtype, field visibility)
- Phase 7 crate split (extract melange-parser, melange-codegen)
- BoyleDiodes heavy-clip Anderson acceleration / BoyleDiodes→ActiveSetBe hybrid (low priority)

- **Knee re-solve for a railing op-amp into a saturating inductor at 1× — PARKED** (2026-09-28). At 1× with an active-set rail mode, the inductor's internal current overshoots 5-13 % where the op-amp rails into the core (oa_sat choke deck; output H1 within 0.21 %); 4× is accurate (i_L 1.3 %, H1 0.1 %), and compile prints a notice below 4×. The overshoot is born where the core crosses its knee within one sample with the full rail across it (per-sample trace: 1× peaks 6.00 mA against 5.34 at 4×, then alternates at ~−0.5/sample for ~5 samples). Measured and rejected: one backward-Euler sample at each pin-state change, either form (does not reduce it; worse H1); the existing recovery sub-step ladder triggered on the L_diff collapse ratio (catastrophic: fired on the unpinned iterate, the full-step pin re-solve discarded the sub-steps). The collapse ratio is not a sound trigger by itself: a saturating RL at 20 V collapses harder (r 0.012 vs 0.06) and does not ring; the discriminator is the voltage across the core at the knee. **Reopen when** (i) a real deck needs internal-current accuracy at 1× (e.g. an op-amp-driven output transformer where i_L feeds something audible), or (ii) ship-path CPU rules out 4× for such a deck. **Requirements for whoever reopens it:** trigger on the exact per-element stiffness (Z_k from the factored matrix), possibly ANDed with the knee collapse, not the ratio alone; trigger on the committed post-pin solution; each sub-step converged to the main loop's own definition; the pin detected and resolved inside each sub-step; the saturating residual at each sub-step. Prior art (local, not in the repo): the per-element stiffness guard branch, a transition-BE patch and a first knee sub-step patch.
- **Node Gmin per path (resolved 2026-10-01, `25df40c`).** One constant, `GMIN_REGULARISATION = 1e-12` (`codegen/ir/mod.rs`), is stamped on the nodal `G` node diagonals in `build_nodal` (so into every runtime `A`, `A_be`, `S`, `K`; not into `A_neg`), and every emitted nodal solve (Schur and full-LU main, backward-Euler, M=0 direct, sub-step, chord, op-amp active-set pin) is built from that `G` and adds none of its own. Per path: DK transient 0; every nodal path 1e-12; the compile-time DC OP 1e-12 on node rows (`dc_op.rs`, `build_dc_system`), so DK's baked `DC_OP` and its transient differ by it while nodal solves the circuit its DC OP was solved on. The nodal emitter's second `+1e-12` (M=0 direct solve, sub-step, chord LU, op-amp pin), which made those solves 2e-12, is removed: on the corpus it had no conditioning role (every Newton health counter identical without it) and it moved the solution as predicted (split-band-triode-drive's stage-1 plate at rest by 2.5e-5 V = 1e-12 S × 222 V × 100 kΩ); golden silence renders changed at the µV level and signal renders by ≤ 1e-5 dB. Witness: `nodal_linear_direct_solve_tests.rs::full_lu_m0_direct_solve_settles_on_the_analytic_dc` (a node held only by 1 GΩ resistors).
- **Baked node Gmin: nodal `G` carries 1e-12 S per node, DK's carries none** (open; opened 2026-09-28 as "node Gmin moves the fixed point"). A regularisation must not move the solution, and this one does, by ~1e-12·R_node·V. Measured before `25df40c` on a 1 MΩ/1 MΩ follower bias divider (so its full-LU figure still includes the removed second term): gate 12.0000 V on DK, 11.999987 on nodal Schur, 11.999981 on full-LU, 11.999988 in the DC OP; ngspice (no rshunt) 12.0; not re-measured since. At 10–100 MΩ grid leaks or piezo/electret nodes the shift becomes an audible-scale bias error. **To decide whether nodal needs it, use the same method as the emitted-term removal:** build the corpus with and without the baked term and compare singularity and conditioning evidence (every Newton health counter, LU pivot failures, NaN resets, decks with weakly-tied or floating nodes) and the golden renders; remove it only if nothing needs it, otherwise apply it in residual form or compensate the RHS with Gmin·v_iter so it cancels at convergence. Target: every path equal to ngspice and DK/nodal agreement to 1e-9. Whether DK's transient should carry the DC OP's floor is part of the same decision. Interim tripwire: `mosfet_body_effect_tests.rs::follower_paths_agree_within_the_node_gmin_tripwire`. Two more witnesses (2026-10-04): (1) a JFET voltage-variable resistor (27 kΩ feed, gate on a 1 MΩ/1 MΩ half-drain divider biased below pinch-off, 1 nF on the drain, 48 kHz, 2 V drive, level 1 or 2) whose DK, nodal Schur and full-LU outputs differ pairwise by 2.6–3.7 µV on a 0.62 V peak (≈ 5e-6, against a 1e-9 Newton tolerance); first check is to drop the baked term on that deck and see whether Schur and full-LU collapse onto DK. (2) A drain node tied only to a cut-off level-2 JFET channel and a capacitor: an indeterminate node, settled in the DC OP by the 1e-12 S floor against the gate junction's −IS (10 mV; 0 V with IS = 0).
- **Observation, not a task (2026-09-29): full-LU and Schur disagree by 0.006–0.015 dB on a FET-limiter draft.** The same deck with one emitter follower `.linearize`d (M = 23, nodal Schur) and full (M = 25, nodal full-LU at max\|K\| = 1e12, "device nodes lack resistive paths") differs in small-signal gain by 0.015 dB (`analyze`, 96 kHz, flat from 100 Hz to 1 kHz and the same at 1e-4 and 1e-5 V drive) and 0.006 dB (`validate`, 48 kHz). Against ngspice the full deck is +0.024 dB and the linearized one +0.030 dB. The linearization is not the cause: the follower's card in isolation, at a similar bias, gives linearized = full = ngspice, and both builds reach the same DC operating point to 1e-6 V. Unexplained; one candidate, not tested, was the full-LU second node Gmin, since removed (`25df40c`, above); the deck has not been re-measured. If a later item touches full-LU conditioning, this deck (the circuits repository's unpublished FET-limiter stage-3 draft, with and without its output follower linearized) is its witness.

## Cross-Compilation (macOS from Linux)

Zig 0.13 + cargo-zigbuild + macOS SDK 13.3 + rcodesign (ad-hoc signing).
`cargo zigbuild --release --target universal2-apple-darwin` produces universal Mac binaries.
melange-cli does NOT cross-compile (ureq/dirs need CoreFoundation), but generated plugins do.
