# Debugging Guide

## Verified Test Circuits

### 1. Voltage Divider (Unity Gain Test)
```spice
R1 in out 10k
R2 out 0 10k
```
**Expected**: Gain = 0.5 (with 10k input R)
**If wrong**: Check input conductance stamping

### 2. Diode Clipper (Nonlinearity Test)
```spice
R1 in 1 10k
D1 1 0 D1N4148
D2 0 1 D1N4148
R2 1 out 10k
```
**Expected**: Unity gain below 0.7V, hard clip above
**If wrong**: Check NR solver or K matrix sign

### 3. RC Lowpass (Frequency Test)
```spice
R1 in out 10k
C1 out 0 0.1u
```
**Expected**: -3dB @ 159Hz
**If wrong**: Check C matrix stamping or alpha value

## Problem: No Output / Very Quiet

### Check Input Resistance
The default INPUT_RESISTANCE is 1 ohm (near-ideal voltage source). If set too high
(e.g. 10k), coupling caps in the circuit will form a voltage divider with the
source impedance, attenuating the signal before it reaches the nonlinear devices.

### Check Input Conductance Stamping
```rust
// In MnaSystem:
pub fn stamp_input_conductance(&mut self, node: usize, conductance: f64) {
    if conductance > 0.0 {
        self.g[node][node] += conductance;
    }
}
```

### Check S Matrix Magnitude
```rust
// S[output][input] should be reasonable
// If ~0.001: input conductance not stamped into G matrix
// If > 1e6: matrix singular
```

**Diagnostic**: Print S matrix diagonal - should be reasonable (< 1e6).

## Problem: Output Explosion (NaN/Inf)

### Check K Matrix
```rust
// K = N_v * S * N_i (NO negation!)
// K is naturally negative for stable circuits.
// Do NOT add an extra negation — that creates positive feedback.
let k_2d = mat_mul(&mna.n_v, &s_ni);
// Use k_2d directly, no sign flip
```

### Check Jacobian Formula

Generated code (block-diagonal sum over the device block that owns row `i`):
```rust
let j_ij = delta_ij - sum_k(jdev_{ik} * K[k][j]);
```

Since K is naturally negative, J > 0 (convergent). The runtime solver that
used a full-matrix `J = I - J_dev * K` form has been removed.

### Check Voltage Limiting
```rust
// MUST have SPICE-style voltage limiting:
// 1. Compute implied voltage change: dv = -K * delta
// 2. Apply pnjlim (PN junctions) or fetlim (FETs) per device dimension
// 3. Compute scalar damping factor: alpha = min(limited / raw) across dimensions
// 4. Apply damped step: i_nl -= alpha * delta
// Per-device VCRIT constants precomputed from pn_vcrit(vt, is)

// WITHOUT limiting:
i_nl[0] -= delta;  // Can explode on sharp nonlinearities
```

### Check A Neg Formula
```rust
// WRONG: Double-counting history
rhs[i] += cap_history[i];  // DON'T DO THIS
rhs[i] += A_NEG[i][j] * v_prev[j];

// RIGHT: A_neg includes alpha*C (history)
rhs[i] = A_NEG[i][j] * v_prev[j];
```

## Problem: Wrong Frequency Response

### Check Alpha Value
```rust
const ALPHA: f64 = 2.0 * SAMPLE_RATE;  // Trapezoidal
// If using 1/T: backward Euler (different response)
```

### Check C Matrix Stamping
```rust
// Capacitor between i,j stamps into C matrix:
C[i,i] += C; C[j,j] += C;
C[i,j] -= C; C[j,i] -= C;

// NOT into G matrix directly!
```

## Problem: DC Offset Accumulation

### Check A Neg Construction
```rust
// In mna.rs:
a_neg = alpha*C - G  // Correct
// a_neg = G - alpha*C  // Wrong - causes DC drift
```

### Check No Separate History
```rust
// WRONG: Adding history twice
let history = alpha*C*v_prev;
rhs += history + A_neg*v_prev;  // Double!

// RIGHT: A_neg already has alpha*C
rhs = A_neg * v_prev;
```

## Problem: BJT Circuit Produces No Output / Wrong Bias

### Check DC Operating Point Initialization

BJT amplifiers require a DC bias point. Without it, all transistors start in cutoff
(v=0) and produce no output.

Check that `DC_NL_I` constant is present and non-zero in generated code:
```rust
pub const DC_NL_I: [f64; M] = [...];  // Should be non-zero for BJT circuits
```

If `DC_NL_I` is all zeros or missing, the DC OP solver did not converge — check
the compile-time log output for `DcOpMethod::Failed`.

### Check DC OP Convergence

```rust
let result = dc_op::solve_dc_operating_point(&mna, &slots, &config);
log::debug!("DC OP: converged={}, method={:?}, iters={}", result.converged, result.method, result.iterations);
for (name, &idx) in &mna.node_map {
    if idx > 0 { log::debug!("  V({}) = {:.4}V", name, result.v_node[idx - 1]); }
}
```

### Expected DC OP for BJT CE Amplifier

```
V(base) ≈ voltage divider output (e.g., 2.16V for 12V/100k/22k)
V(emitter) ≈ V(base) - 0.65V
V(collector) ≈ VCC - Ic*RC
```

If V(base) is correct but V(collector) ≈ VCC, the BJT is in cutoff (wrong DC OP).

### DC OP Jacobian Sign

The companion formulation uses **subtraction**:
```
G_aug = G_dc - N_i · J_dev · N_v
```

Using addition causes NR divergence. See `DC_OP.md` for the mathematical derivation.

## Quick Diagnostics

```rust
// Add to process_sample for debugging:
#[cfg(debug_assertions)]
{
    println!("Input: {:.4}, Output: {:.4}", input, output);
    println!("v_pred: {:?}", v_pred);
    println!("i_nl: {:?}", i_nl);
    println!("NR iterations: {}", state.last_nr_iterations);

    // Sanity checks
    assert!(v.iter().all(|x| x.is_finite()), "NaN/Inf in v");
    assert!(v.iter().all(|x| x.abs() < 100.0), "v too large");
    assert!(state.last_nr_iterations < 20, "NR not converging");
}
```

## Key Sign Conventions

| Component | Convention | Sign |
|-----------|-----------|------|
| N_i[anode] | Current extracted | -1 |
| N_i[cathode] | Current injected | +1 |
| K = N_v*S*N_i | Naturally negative | Correct feedback |
| Codegen J | `I - J_dev * K` (block-diag) | Positive (convergent) |
| DC OP G_aug | `G_dc - N_i*J_dev*N_v` (subtract!) | Diagonal-dominant |

## Verified Working Values

| Parameter | Value | Notes |
|-----------|-------|-------|
| INPUT_RESISTANCE | 1 ohm | Near-ideal voltage source |
| Voltage limiting | SPICE pnjlim/fetlim | pnjlim for diode/BJT/tube (VCRIT per device), fetlim for JFET/MOSFET |
| MAX_ITER | 100 | NR iteration limit (codegen) |
| TOLERANCE | 1e-9 | NR convergence |
| alpha | 2/T | Trapezoidal rule |
| K | N_v*S*N_i | Naturally negative, no extra negation |
| DC OP tolerance | 1e-9 | DC OP NR convergence |
| DC OP max_iter | 200 | DC OP NR iteration limit |
| DC OP source_steps | 10 | Source stepping stages |
| DC OP voltage_limit | logarithmic (Vt-scaled) | Junction-aware: `sign * Vt * ln(\|delta\|/Vt + 1)` |

## Transformer-Coupled Circuit Failure Signatures

| Symptom | Cause | Fix |
|---------|-------|-----|
| NR diverges exponentially on transformer nodes | Non-positive-definite inductance matrix from inconsistent k values | Ensure all windings on same core have similar k. MNA builder warns on non-PD matrices |
| i_nl/v_prev inconsistent after max_iter | v updated one step ahead of i_nl on non-convergence | Re-evaluate devices at final v when NR fails (FIXED) |
| 1e-6 absolute tol on 290V circuit | Demands 3.4 ppb — unreasonable for LU precision | Use SPICE RELTOL: 1e-3 * max(\|v\|) + 1e-6 (FIXED) |
| Trapezoidal NR ringing | Marginally stable oscillatory mode at Nyquist | BE fallback catches these samples automatically (FIXED) |
| Incomplete transformer coupling matrix | Missing K directive between windings on same core | Add K for ALL winding pairs; non-PD det warns |

## Op-amp BoyleDiodes Failure Signatures

`OpampRailMode::BoyleDiodes` synthesizes catch diodes between each clamped op-amp's
internal gain node and rail-reference voltage sources (`VCC − VOH_DROP`,
`VEE + VOL_DROP`). The catch diodes have very abrupt knees (`IS=1e-15 N=1`,
Boyle-standard silicon) and exhibit failure modes that don't appear with smoother
nonlinear devices.

| Symptom | Cause | Fix |
|---------|-------|-----|
| `state.a[input_node][input_node]` near zero in augmented MNA — first non-zero input sample produces wildly wrong v[buf_in] | `MnaSystem::from_netlist(&augmented_netlist)` builds a fresh MNA that doesn't preserve the in-place input-conductance stamp the CLI applied to the original | Re-stamp `g[input][input] += 1/R_in` and `stamp_device_junction_caps` after the augmented rebuild in `generate_nodal` (FIXED, commit `5544c8a`) |
| Trap NR "converges" with stale chord_j_dev → wildly wrong v on first signal sample | Voltage-step convergence check is necessary but not sufficient when `chord_j_dev` is many OOM stale; the LU back-solve produces a "fixed point" that satisfies the linearised system but not actual KCL | Add a residual check: re-evaluate `i_nl_fresh` from device equations at post-step v and require it to match the `i_nl` the LU was solved against, with `tol = RELTOL * max(\|new\|, \|old\|, 1e-9) + ABSTOL`. Mirrors DK Schur path's convergence criterion. BoyleDiodes-gated. (FIXED, commit `39397d1`) |
| Catch diode `j_dev` jumps 32 OOM (1e-31 reverse → 1e+1 forward) within one NR loop, chord stays stale until iter 5 refactor | The default `iter % CHORD_REFACTOR == 0` refactor is too coarse for diodes whose Jacobian changes by orders of magnitude | Adaptive refactor trigger: at the start of each NR iteration, force refactor if any device's `\|j_dev[k][k]\| / \|chord_j_dev[k][k]\|` exceeds 50% relative change. BoyleDiodes-gated. (FIXED, commit `39397d1`) |
| Heavy clipping (amp ≥ 0.05 V on Klon) under `--opamp-rail-mode boyle-diodes`: raw output node `state.v_prev[OUTPUT_NODES[0]]` diverges to 45–3068 V, NR fails every sample. The generated `output[i].clamp(-10.0, 10.0)` safety rail masks this to a visible 10 V, so superficial inspection shows "output=10 V" while the actual solver state is ±3000 V. Always measure raw node voltage when debugging BoyleDiodes convergence. | Three fix candidates empirically tested 2026-04-08 fourth session with a disciplined amp sweep [0.01, 0.03, 0.05, 0.07, 0.10, 0.15, 0.20, 0.30, 0.50]: (a) targeted Gmin bump on `_oa_int_*` rows — destroys linear-regime op-amp gain; (b) force `need_refactor = true` in BoyleDiodes (ngspice-style refactor-every-iter) — preserves linear, doesn't fix heavy clip; (c) disable global `damp_thresh` step cap — preserves linear, doesn't fix heavy clip. None satisfied the confirmation criteria (zero NR failures at all amps, peak in [10.0, 11.0] V). The chord-LU Newton direction appears to be wrong at the catch-diode knee, not just the magnitude, so no form of step damping or single-row regularization fixes the underlying issue. Prior "bistable chord-LU fixed points" and "PTC doesn't work because of static pivots" diagnoses were both retracted; see `task_12_bistable_oscillation_finding.md` "FOURTH SESSION" for the full audit and sweep data. | **Not blocking for Klon release.** Klon auto-detect routes to `active-set-be`, which was empirically verified in the same sweep: raw peak bounded 10.70–10.76 V at every tested amplitude, trap NR falls through to BE fallback at heavy clip and BE converges every time. BoyleDiodes mode remains opt-in (`--opamp-rail-mode boyle-diodes`) and is a known limitation at heavy clip; it works correctly for light clip (amp ≤ 0.03 V on Klon) and for control-path topologies. If heavy-clip BoyleDiodes convergence becomes a priority later, the next tier of escalation candidates are: Anderson acceleration (m=3, Walker-Ni + Zhang-Peng-Ouyang safeguards), trust-region Newton with actual-vs-predicted ratio, or a BoyleDiodes → ActiveSetBe failure-hybrid that tries BoyleDiodes trap and falls through to ActiveSetBe on divergence. |

## ActiveSetBe Chord-NR False Convergence (Precision Rectifiers)

Solver bug class: when an op-amp is DC-railed (`v[n_out] = ±VSAT` exactly
every sample) and a feedback diode then sits in deep forward bias from the
pinned rail down to a cathode node below `-VSAT`, the chord-NR reports
converged on a KCL-violating fixed point. The residual leaks through
`A_neg·v_prev` / `N_I·i_nl_prev` to the next sample, drifts sub-Hz, and
eventually blows up exponentially.

The first diagnostic question is always **"does the op-amp's VSAT match its
actual supply rails?"** — a mis-calibrated VSAT unnaturally DC-rails an
op-amp that real hardware would not, and triggers this bug class on the
solver side. Fix the netlist first (see "Symptoms" below), then treat any
residual divergence as a genuine solver problem.

### 4kbuscomp: both contributions (2026-04-17)

On 4kbuscomp the bug had two independent contributions:

**Netlist-side (primary)**: `.model OA_TL074 OA(... VSAT=11 ...)` was
appropriate for a ±12 V-supplied TL074, but the 4kbuscomp netlist runs on
±15 V supplies (`Vpos vcc15 0 DC 15`, `Vneg vee15 0 DC -15`). On ±15 V,
a real TL074 saturates at about ±13.5 V (datasheet: output swing = VCC − 1.5 V).
U8 and U9 in the precision rectifier have `n_plus = vee12 = -12 V`; in real
hardware that's inside both the input CMR *and* the output range, and the
op-amp is **not** DC-railed. In melange with VSAT=11 it *is* DC-railed at
−11 V. Fix: set `VSAT=13.5` (or `VCC=13.5 VEE=-13.5`) on the TL074 model.
This change lives in `melange-circuits/unstable/dynamics/4kbuscomp.cir` and
as of 2026-04-17 is uncommitted in the working tree of that repo.

**Solver-side (general hardening)**: the residual check in `nodal_emitter.rs`
shipped in `c3d3eae` — see "Residual check" below. This is load-bearing for
any topology where an op-amp gets DC-railed regardless of VSAT correctness.

With the solver-side residual check alone, 4kbuscomp is stable at `d ≤ 2 s`
at all amps (zero NR/BE events), but `d = 5 s` still diverges with 82,899
NR max-iter hits — see `memory/project_4kbuscomp_chord_false_convergence.md`
for the remaining open tail and four ranked candidate next steps. With the
netlist-side VSAT=13.5 fix combined, 4kbuscomp is stable at `d = 5 s` across
`amp ∈ {0.01, 0.1, 0.5}` with zero NR max-iter hits and zero BE fallbacks.

### Mechanism (when the op-amp really is DC-railed)

1. Generated NR emits `if v_new[n_out] < -VSAT { v_new[n_out] = -VSAT }` after
   every LU back-solve (`nodal_emitter.rs`, "Per-iteration op-amp output rail
   clamp"). This is ALWAYS emitted in the trap path, regardless of rail mode.
2. With `v[rect_a_out]` pinned at exactly `-VSAT`, a forward-biased feedback
   diode (e.g. 4kbuscomp D2, anode=rect_a_out, cathode=jct_b) sees
   `V_d = -VSAT − v[jct_b]`. If the physical operating point of the cathode
   node sits below `-VSAT` (which happens specifically when VSAT is
   mis-calibrated against the supplies), `V_d` goes deep into forward bias.
   The emitted `diode_current` clamps `V_d` to `40·N·VT ≈ 1.81 V`, but even
   clamped, `I_D ≈ 10¹ A` — non-physical.
3. The chord NR uses `G_aug = A − N_i · chord_j_dev · N_v`. With large
   `chord_j_dev[D][D] ≈ 330 S`, the LU back-solve DOES satisfy the
   linearised equation. But because `v[n_out]` was clamped (not a KCL
   solution), the linearised system enforces KCL only at the un-clamped
   nodes — the rail-pinned node's residual goes "out through the clamp."
4. `active_set_engaged` (the flag that gates the BE fallback + constrained
   `emit_nodal_active_set_resolve`) uses strict inequality against the rail:
   `v[n_out] > hi || v[n_out] < lo`. Since the clamp makes `v[n_out]` exactly
   equal to the rail, the engagement check NEVER fires. BE fallback never
   runs. No constrained resolve. The cap-history `A_neg · v_prev` and
   `N_I · i_nl_prev` carry the sub-sample KCL residual to the next sample.
5. Over thousands of samples the residual drifts slowly, then enters a
   sub-Hz oscillation (period ~2 s on 4kbuscomp, coupled through the
   22 µF sidechain coupling cap) whose amplitude grows until `V_D` exceeds
   the clamp and `i_nl[D]` runs away exponentially.

### Symptoms

| Signature | Notes |
|-----------|-------|
| An op-amp's non-inverting input sits outside `[-VSAT, +VSAT]` at the DC operating point | Model-calibration check. Compare the input node's DC voltage to the `.model`'s VSAT; if the input is outside the rail, the sim will DC-rail the op-amp even when real hardware would not. This is the first thing to check before reaching for solver changes. |
| `state.i_nl_prev[D]` far larger than any real diode rating (e.g. 15 A on a 1N4148) at sample 0, growing exponentially over 40 000+ samples | Diode current is self-consistent with the clamped `v[n_out]` and the drifted cathode voltage, but KCL at the cathode is violated by the full `i_nl[D]` magnitude. |
| `state.diag_nr_max_iter_count = 0` and `diag_be_fallback_count = 0` for the entire stable phase | The voltage-step convergence criterion is fooled; NR reports converged on the false fixed point. |
| `v[n_out] = ±VSAT` exactly (all digits) every sample | Clamp signature. If you see this with no BE fallbacks, `active_set_engaged` is starved. |
| Sub-Hz oscillation of the cathode node growing in amplitude until blow-up, with lower amplitudes taking *longer* to diverge rather than shorter | The instability is in the DC-rail regime — independent of signal amplitude. |

### Residual check (shipped 2026-04-17 commit `c3d3eae`, load-bearing)

The BoyleDiodes residual check in `nodal_emitter.rs` is extended to
`BoyleDiodes | ActiveSetBe | ActiveSet`. After the damped NR step,
re-evaluate `i_nl_fresh` from device equations at post-step `v` and set
`max_step_exceeded = true` if any device current differs from the chord's
`i_nl` by more than `1e-3 · max(|fresh|, |chord|, 1e-9) + 1e-12`.

Catches the same bug class on any topology (rail-engaged op-amp + stale
`chord_j_dev` producing a false fixed point). Costs only M device-equation
calls per NR iter. Zero regressions on the validated-circuits suite. Gets
4kbuscomp stable to `d ≤ 2 s` without the netlist-side VSAT fix; `d = 5 s`
remains open on the original netlist (see the chord memory's ranked next
steps: tighter RELTOL, adaptive refactor on `|j_dev/chord_j_dev| > 1.5`,
KCL-consistent active-set resolve that stamps device Jacobian into `g_as`,
or every-iter refactor when a rail is engaged).

See also `memory/project_4kbuscomp_chord_false_convergence.md` for the
investigation trail and `memory/project_4kbuscomp_basin_trap.md` for the
DC-OP basin-trap entry — different bug class (DC solver), same circuit.

### Positive Definiteness Rule for Transformer Coupling
All windings on the same core must have coupling coefficients that form a positive-definite
inductance matrix. For a 4-winding transformer with k_ab=0.95 and k_ac=0.95, k_bc must be
≥~0.88 for the matrix to be PD. A k_bc of 0.50 gives det<0 (physically impossible) and
causes NR divergence. The MNA builder validates this and emits `log::warn`.

## Cap-Only Nodes and Schur NR Failure (Transistor Ladders)

Circuits where intermediate nodes connect ONLY through BJT junctions and bridging
capacitors (no resistors) produce ill-conditioned A = G + 2C/T matrices. These nodes
have G ≈ Gmin (1e-12), so S = A^{-1} has extreme entries (>1e6). This causes:

1. **DK kernel failure**: A is near-singular, `from_mna()` returns error at the
   problematic column. Routes to nodal automatically.
2. **Nodal Schur failure**: K = N_V·S·N_I has entries spanning 10+ orders of magnitude.
   The Schur NR Jacobian J = I - J_dev·K is swamped (J_dev·K >> I), producing
   numerically garbage solutions (flat output, wrong gain).

**Detection**: `max|S| > 1e6` routes to full LU NR. This check is invariant to FA
reduction — FA changes N_V/N_I/K dimensions but NOT A/S conditioning, since the
problematic nodes still lack resistive paths.

**Example**: Moog-style 8-BJT transistor ladder filter (moonladder). Bridging caps
between left/right columns at each stage. Nodes eL2-eR4 have only Gmin + cap.
S entries ~1e8, K entries ~5e11 at M=16 (pre-FA), ~1e4 at M=8 (post-FA), but
S remains extreme in both cases.

**FA detection threshold**: Lowered from -1.0V to -0.5V (2026-04-10). Common-base
BJTs in cascade topologies have Vbc ≈ -0.85V — clearly forward-active but missed
the old -1.0V threshold. The -0.5V threshold catches all common-base stages while
still excluding saturated BJTs (Vbc > -0.5V).

**FA undo removed**: The blanket "undo FA for nodal path" policy was removed.
If the DC OP confirms Vbc < -0.5V, the BJT has adequate margin for audio-level
transients. FA reduction is preserved on all codegen paths.

## Precision Rectifier DC OP Convergence

Circuits with precision rectifiers (op-amp + diode feedback, e.g., SSL bus compressor
sidechain) have **two self-consistent DC equilibria** — one at each op-amp rail. The
NR may converge to the wrong one.

**Root cause**: The VCCS model with AOL=200,000 overshoots from one rail to the other
in a single NR iteration, overwhelming diode feedback. The correct equilibrium has the
op-amp at one rail with one diode conducting; the wrong one has the opposite rail with
a different diode conducting.

**Fixes applied (2026-04-15)**:
- **AOL capping**: `build_dc_system()` caps op-amp AOL at 1000 in the DC G matrix.
  1000 gives 60 dB open-loop gain — accurate to 0.1% for virtual grounds, but low
  enough for NR convergence.
- **Output seeding**: `seed_opamp_outputs()` initializes op-amp outputs to the rail
  matching sign(V+ - V-) at all init points (direct NR, source stepping, Gmin stepping).
- **Per-iteration rail clamp**: Op-amp outputs clamped to VCC/VEE after each NR solve
  step in dc_op.rs.
- **Seeded linear fallback**: When all strategies fail, the linear fallback applies
  op-amp seeding before returning.

**Status (2026-04-15)**: DC OP for the 4kbuscomp now produces correct polarity (−11 V at sidechain
TL074 outputs instead of +11 V). DC_OP_CONVERGED=false (formal convergence not achieved),
but the values are physically correct.

**Status (2026-04-16, surfaced)**: A distinct failure surfaced in the precision-rectifier
op-amps themselves (`U8`, `U9` — not the sidechain buffer). The op-amp output rails at −VSAT
while the clamp diode (D1, D3 — anode = inv input, cathode = op-amp output) sits at +11 V
forward bias, giving `I_diode ≈ 1e14 A`. This is a valid NR fixed point but non-physical —
the correct basin has the op-amp following `v+ = vee12` via D1 feedback with
`v(out) ≈ v+ + Vd`. The AOL cap (200 k → 1 k) did not escape this basin: both equilibria
remain self-consistent at AOL = 1 k, and the linear initial guess + exponential diode
Jacobian lands and stays in the wrong one.

**Status (2026-04-17, FIXED commit `b771512`)**: the originally-greenlit plan (AOL
continuation `[1, 10, 100, 1000, target]`) did not work alone — at AOL=1 the physical
basin is `v_out ≈ AOL·(VEE−0.65)/(1+AOL) ≈ −6.3 V` (op-amp does not rail), 5 V from the
`v_out = VEE` seed; cascaded diodes (D5/D6, D2/D4) create additional local minima.

The shipped fix is a **post-fallback refinement NR** (gated on `has_sidechain_rectifier`):

1. `seed_sr_feedback_diodes()` — after `seed_opamp_outputs` places op-amp outputs at rail,
   clamp each direct-feedback diode's non-output terminal to `v_out ± 0.65 V` if currently
   forward-biased > 1 V. Cascade diodes left to NR.
2. **Fallback-refinement NR tail** — after Strategies 1–4 (Direct NR, Source Stepping,
   Gmin Stepping, AOL Stepping) all fail, run direct NR in `aol_cont_mode = true`
   (widened rails) from the synthesized linear-fallback + diode-consistency state. For
   4kbuscomp this converges in **411 iters** and pulls diode currents from the bogus
   4.3 mA down to 16 µA, satisfying KCL.

The infrastructure for AOL continuation (Strategy 4 `AolStepping`, `patch_g_dc_for_aol`,
`dc_opamp_is_sidechain_rectifier` / Rule D' DC-OP reimplementation, `aol_cont_mode`
rail-widening in `nr_dc_solve`) is all present and exercised; the refinement tail is what
actually finds the correct basin. Post-fix values in `memory/project_4kbuscomp_basin_trap.md`:
v(rect_a_inv)=−12.01 V, v(rect_a_out)=−12.41 V, D1/D3 forward at ~16–32 µA.

Also added in the same commit: op-amp `.model OA(IB=… RIN=…)` input-stage parasitics
(signed bias current A, shunt conductance Ω). Defaults `IB=0` / `RIN=+∞` produce
byte-identical generated code (verified via md5).

The pre-revert `4kbuscomp.cir` in `melange-circuits` had `Rsc_vca = 1 Ω` (undocumented
solver-stability workaround, reverted 2026-04-16 to the schematic-accurate `1 MEG`).
The `1 Ω` was masking this bug — every "validated" claim for 4kbuscomp prior to 2026-04-16
refers to that workaround being in place.

## Low-Rate DC Warmup (for Failed DC OP)

When DC OP doesn't converge (`DC_OP_CONVERGED = false`), the generated code's `warmup()`
method fast-forwards to the DC steady state at a low sample rate (200 Hz) before running
the normal 50-sample warmup at the target rate.

**Why**: Coupling caps (e.g. 22µF × 27K = 0.6s RC) take ~3 seconds to charge. At 48kHz,
that's ~143,000 samples — far too many for init. The 50-sample default warmup only covers
~1ms, leaving the circuit deeply in its charging transient. With garbage `DC_NL_I` values
(e.g. 1.21e11 A from the failed DC OP), the NR never recovers.

**How**: `warmup()` calls `rebuild_matrices(200.0)`, runs 1000 silent samples (= 5 seconds
of circuit time), then restores `rebuild_matrices(target_rate)`. The DC steady state is
rate-independent (`A - A_neg = 2G`, no rate terms), so values found at 200 Hz are valid at
any target rate. The settled state is cached in `dc_operating_point` and `settled_i_nl`
for subsequent `reset()` calls — the expensive low-rate phase runs only once.

**Result**: 4kbuscomp BE fallback reduced from ~100% of samples to <1% (3-34 out of 4800+).
Sub-step NR handles the rest. Circuit stable at all amplitudes (0.001–1.0V).

## Precision Rectifier Transient NR — VCCS Back-Sub Contamination (FIXED 2026-04-16)

The full-LU NR path had a structural problem with high-gain VCCS op-amps:

1. The NR Jacobian `g_aug = A - N_I*J_dev*N_V` inherits Gm ≈ 2000 S from A = G + alpha*C
2. LU back-substitution computes v_new at op-amp outputs (400kV+ before clamp)
3. Neighboring nodes are computed FROM the unclamped op-amp output during back-sub
4. Post-solve rail clamp fixes v_new[op_out] → 11V, but v_new[neighbor] is already 8000V+
5. NR convergence check only monitors device nodes, not linear neighbors — declares converged
6. Contaminated v_prev propagates: `state.v_prev = v`, next sample's `A_neg * v_prev` feeds
   the extreme values back into the RHS. Accumulated to 1.18 BILLION volts at `cv_to_vcas`
   in 4kbuscomp.

**Fix: Selective op-amp VCCS Gm cap.** A blanket cap (AOL 200k → 1k on all op-amps)
eliminates the contamination but kills audio-path gain. Selectively capping only op-amps
that match the precision-rectifier / comparator topology preserves audio-path gain while
bounding the LU back-sub voltage at the offending sites.

**Rule D' classifier** (`opamp_is_sidechain_rectifier` in `crates/melange-solver/src/codegen/ir.rs`):

1. **`n_plus` is on a non-zero DC rail.** Detected by walking `mna.voltage_sources`
   and matching either terminal of any source with `dc_value != 0`. Ground is
   intentionally excluded — soft-clipper topologies (e.g. Klon Centaur, where the
   clipping op-amp's `n_plus` is ground) MUST NOT be classified as sidechain
   rectifiers.
2. **A diode connects the op-amp output to the inverting input,** optionally through a
   pure-resistor path (covers full-wave summing rectifiers like 4kbuscomp `U9`, where
   the diode goes through the summing R network). The R-only BFS in
   `r_only_path_exists` traverses only `Element::Resistor` edges.

When both conditions hold, `effective_aol_cap` returns `AOL_SUB_MAX = 1000`. The cap is
applied at G-matrix build time (constant cost, baked into the emitted G/A/A_neg) and
propagates automatically to A, A_neg, A_be, A_neg_be, the sub-step `a_sub`, and the
runtime `rebuild_matrices` path.

**User override**: `.model OA(AOL_TRANSIENT_CAP=N)` forces a specific cap on a single
op-amp model regardless of Rule D'. Use this when:
- A circuit has a precision rectifier that the auto-detect misses (e.g. `n_plus` is
  ground because the part runs on a virtual-ground bias not represented as a DC source
  in the netlist) — set `AOL_TRANSIENT_CAP=1000`.
- A circuit hits a false positive — set `AOL_TRANSIENT_CAP=200000` (i.e. ≥ AOL) to
  fully disable the cap on that op-amp.

**Verification on 4kbuscomp**: `max_abs_v_prev` drops from 1.18 billion → 15.0 V.
Rule D' correctly fires only on `U8` and `U9` (the two sidechain rectifiers, both with
`n_plus = vee12`), not on the 10 audio-path / virtual-ground op-amps (all with
`n_plus = 0`). Klon (horseface) output is byte-identical before/after the change —
Rule D' correctly excludes its op-amps despite `vbias = 4.5 V` on `n_plus` (Condition 1
passes but Condition 2 fails because Klon's diodes are not in the op-amp feedback path).

## Known Full-LU NR Limitations

The full-LU nodal NR path (used when K is degenerate, has positive diagonal, or is ill-conditioned)
has a known vulnerability with **ill-conditioned A matrices** (cond(A) > ~1000):

- Large coupling caps (e.g., 10µF Cout between drain and output) create near-unity off-diagonal
  ratios in A, making S = A^{-1} entries ~1000. The chord method amplifies stale-Jacobian errors
  by this factor, and the relative convergence check can't detect the resulting false convergence.
- Both trapezoidal and backward Euler are affected — the ill-conditioning is in the circuit
  topology (cap conductance >> resistive conductance), not the integration method.
- The Schur path handles these circuits correctly because the M-dim NR operates on the
  well-conditioned K matrix, isolating the solver from S's ill-conditioning.

**Current mitigation**: Routing prefers Schur when K is well-conditioned, even with marginal
spectral radius (up to 1.002). Only circuits with pathological K AND ill-conditioned A would
hit this — no known circuit triggers both conditions simultaneously.

**Future hardening** (if a circuit is found that needs both):
- Unconditional residual check in full-LU NR (currently
  `BoyleDiodes | ActiveSetBe | ActiveSet`-gated since the 4kbuscomp partial
  fix — see "ActiveSetBe Chord-NR False Convergence" above)
- Iterative refinement in the chord back-solve (one extra O(N²) pass)
- Schur-complement-within-full-LU hybrid (M-dim correction inside N-dim NR)

## Historical Failure Signatures

Catalog of previously-diagnosed failure modes. Commit hashes link to the fix;
dated entries are narrative notes preserved for pattern-matching. Use this as
a "have we seen this before?" lookup before starting a new debugging session.

| Symptom | Cause | Fix / Status |
|---------|-------|--------------|
| NodalSolver NaN after DC OP | Inductor currents not initialized | Copy full v_node (incl. inductor branch currents) from DC OP |
| NodalSolver wrong A_neg | Inductor rows zeroed | Zero n_nodes..n_aug (all augmented), NOT n_aug..n_nodal (inductor branches) |
| Codegen diverges ~5000 samples | Boyle A_neg trapezoidal instability | A_neg must zero ALL augmented rows (Gm ±2000 creates spectral radius > 1) |
| Codegen stable but wrong level | K≈0, Schur NR has J=I (no damping) | Route K≈0 circuits to full N×N LU NR (device Jacobian in G_aug) |
| BoyleDiodes: augmented system input row zero, first signal sample explodes | `MnaSystem::from_netlist(&augmented)` rebuilds MNA, losing the in-place G_in stamp | Re-stamp `g[input][input] += 1/R_in` + junction caps after rebuild in `generate_nodal` (FIXED `5544c8a`) |
| BoyleDiodes false convergence: trap NR declares converged with wildly wrong v[buf_in] on first signal sample | Voltage-step convergence check passes when chord_j_dev is many OOM stale; no residual gate | Residual check + adaptive refactor trigger (>50% j_dev change), BoyleDiodes-gated (FIXED `39397d1`) |
| BoyleDiodes heavy clipping (amp ≥ 0.05 on Klon): NR diverges, raw 45–3068 V | Chord-LU Newton direction wrong (not just magnitude). Gmin/line-search can't fix wrong direction | Not a blocker. Auto-routes to ActiveSetBe. BoyleDiodes opt-in only. See "Op-amp BoyleDiodes Failure Signatures" above |
| DK kernel singular at col N (transistor ladder / cap-only nodes) | Intermediate nodes connected only through BJT junctions + bridging caps have G≈Gmin (1e-12). A = G+2C/T nearly singular | Routes to nodal automatically. DK limitation for series-BJT topologies without parallel resistors |
| Nodal Schur flat output (+9 dB, no filtering) on BJT ladder | S = A⁻¹ has extreme entries (>1e6) at cap-only nodes → K spans 10^11 → J = I-J_dev·K swamped | S magnitude check: `max\|S\| > 1e6` routes to full LU NR. Invariant to FA reduction (FIXED 2026-04-10) |
| simulate/analyze crash "Augmented fallback failed" | DK fails, augmented DK also fails, no nodal fallback | Dummy kernel with dk_failed=true, falls through to nodal codegen (FIXED 2026-04-10) |
| BJT diverges from ngspice at NF ≠ 1 or high injection | q2 used bare VT and omitted `-1`; Ib forward was divided by qb | q2 uses `cbe/IKF + cbc/IKR` where `cbe = IS*(exp(Vbe/(NF*VT))-1)`; Ib ideal forward NOT divided by qb. Matches `bjtload.c:571,618` (FIXED `d7427c4`) |
| Parser panics on non-ASCII component value (e.g. `1ſ`) | `to_uppercase()` changes byte length; byte-slice landed mid-codepoint | `parse_value()` normalizes non-ASCII at entry (µ/μ → u; else `Err`) (FIXED `e96c340`) |
| Click on every pot/switch move in generated plugin | `rebuild_matrices()` zeroed DC blocker + oversampler state | Removed filter state resets from DK Schur `rebuild_matrices` (FIXED `07712a0`) |
| Zipper noise on knob automation | Pot values read once per buffer via `.value()`, smoother unused | Per-sample `.smoothed.next()` read in plugin template (FIXED `7c0fc02`) |
| NR max-iter on wide-range pot jumps (preset recall, automation step) | Stale `v_prev`/`i_nl_prev` from different operating point; NR starts far from new bias | Warm DC-OP re-init in `set_pot_N`/`set_switch_N` when `|r - r_prev|/r_prev > 0.20` (FIXED `e8e18a7`). Earlier SM removal (`eaee955`) treated a symptom — SM is mathematically exact; stale DC-OP seed was the root cause |
| DC OP wrong polarity for precision rectifier op-amps (e.g., 4kbuscomp sidechain TL074 at +11V instead of -11V) | Multi-equilibrium: AOL=200K overshoots NR between rails in one iteration | AOL capped at 1000 in DC G matrix, op-amp output seeding, per-iteration rail clamp in DC OP NR (FIXED 2026-04-15) |
| DC OP diode reverse bias: nodes float to non-physical (91kV) when diodes off | DC OP `evaluate_devices_inner()` ignored BV/IBV and had zero reverse-bias conductance | BV/IBV breakdown added to DC OP diode evaluation; device-level Gmin 1e-12 S (FIXED 2026-04-15) |
| Precision rectifier transient: trap NR never converges, every sample falls to BE | DC OP fails → `DC_NL_I` garbage (1.21e11 A) → coupling caps uncharged after 50-sample warmup (RC > 0.5s needs ~143K samples at 48kHz) | Low-rate DC warmup: 200 Hz × 1000 samples (5s circuit time) charges caps before transient NR. BE fallback <1% (FIXED 2026-04-16). See "Low-Rate DC Warmup" above |
| Full-LU NR: VCCS back-sub contamination (1.18B V at linear neighbor nodes) | Op-amp VCCS Gm ≈ 2000 in A. LU back-sub computes v_new[op_out]=400kV, uses it for neighbors before post-solve clamp. NR convergence checks device nodes only | Selective op-amp Gm cap via Rule D' topology classifier (n_plus on non-zero DC rail AND diode connects output→inv-input through R-only path). User override: `.model OA(AOL_TRANSIENT_CAP=N)` (FIXED 2026-04-16). See "Precision Rectifier Transient NR" above |
| DC OP basin trap: precision-rectifier op-amp railed at −VSAT with clamp diode at 11 V forward bias even after AOL cap | Multi-equilibrium: correct basin + pathological basin both self-consistent. Homotopy-path problem, not cap-magnitude | FIXED 2026-04-17 (commit `b771512`). AOL continuation alone did not converge (physical basin at AOL=1 is v_out ≈ −6.3 V, not the VEE seed). Actual fix is post-fallback refinement NR: `seed_sr_feedback_diodes` clamps feedback-diode terminals to `v_out ± 0.65 V`, then direct NR in `aol_cont_mode` runs from the synthesized state and converges in 411 iters. Gated on `has_sidechain_rectifier`. See "Precision Rectifier DC OP Convergence" above |
| Transient chord-NR false convergence on DC-railed op-amp: `v[n_out] = ±VSAT` every sample, forward-biased feedback diode drawing 10+ A, sub-Hz drift into exponential blow-up at d ≥ 0.9 s (amp 0.1) / d ≥ 3.3 s (amp 0.01) | Strict-inequality `active_set_engaged` check doesn't count clamped-to-rail as engaged; BE fallback and active-set resolve never fire; residual leaks through `A_neg·v_prev` / `N_I·i_nl_prev` | PARTIAL FIX 2026-04-17 (commit `c3d3eae`): residual check in `nodal_emitter.rs` extended from `BoyleDiodes` to `BoyleDiodes \| ActiveSetBe \| ActiveSet`. After damped NR step, re-evaluate `i_nl_fresh` and force `max_step_exceeded = true` on >1e-3 relative mismatch. Gets d ≤ 2 s stable at all amps with zero NR/BE events. **d = 5 s still diverges on the original netlist (82,899 NR max-iter hits)** — closed only when combined with netlist-side `.model OA_TL074 VSAT=11 → 13.5` fix (uncommitted in melange-circuits as of 2026-04-17). See "ActiveSetBe Chord-NR False Convergence" above and `memory/project_4kbuscomp_chord_false_convergence.md` for candidate next steps (tighter residual tol, adaptive refactor, KCL-consistent active-set resolve, every-iter refactor on rail engagement) |
| `.switch` resistor off by (1/static − 1/pos_0) on every switch change, `set_switch_N(0)` silent no-op at init | Initial `switch_position = 0` but G stamped at static netlist value, not pos-0. `rebuild_matrices` computes delta against static; set_switch_N(0) short-circuits when already at 0 | `MnaSystem::switch_default_overrides` stamps G/C/L at pos-0 (canonical baseline). `SwitchComponentInfo.nominal_value` stores pos-0 so codegen delta baseline matches. `log::info!` when static ≠ pos-0 (FIXED 2026-04-17 commit `586c2a8`) |

## References
- TU Delft Analog Electronics Webbook: MNA stamps
- Hack Audio Tutorial: DK method and NR solver
- Pillage & Rohrer: Companion models
