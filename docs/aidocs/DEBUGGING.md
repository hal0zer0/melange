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

// RIGHT (charge form): A_neg = alpha*C plus the carried q_dot is the history
rhs[i] = A_NEG[i][j] * v_prev[j] + q_dot[i];
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
// Generated code (charge form, both integrators):
a_neg = alpha*C          // Correct — no G term; trap adds q_dot
// a_neg = alpha*C - G   // Whole-system form: only valid with b(n)+b(n+1) sources
//                       // and N_i*i_nl_prev in the RHS, and no q_dot
```
The library `MnaSystem::get_a_neg_matrix` / `DkKernel` still return the
whole-system `alpha*C - G` (used by the stability discriminators and the
runtime `LinearSolver`). Mixing terms of the two forms double-counts a source.

### Check No Separate History
```rust
// WRONG: Adding history twice
let history = alpha*C*v_prev;
rhs += history + A_neg*v_prev;  // Double!

// RIGHT: A_neg*v_prev + q_dot is the whole history; sources once, at n+1
rhs = A_neg * v_prev + q_dot + b_next;
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

## Newton Converging Linearly (Full Steps, ~1 % per Iteration)

**Signature:** samples exhaust `MAX_ITER` (`diag_nr_max_iter_count`, often
held), yet a per-iteration trace shows every step taken in full (no limiter,
`global_alpha = 1`) and the residual falling by a steady ~1 % per iteration.
Newton with the right Jacobian is quadratic near a root; a steady linear rate
means the Jacobian is not the derivative of the function being solved.
`--max-iter 500` making every sample converge is the same tell.

**Check:** at the stalled iterate, compare each device's analytic Jacobian
with a finite difference of the currents it returns. Guards are the usual
culprit: a clamp such as `vpk.max(0.0)` makes the current constant below it,
but a Jacobian evaluated at the clamped value reports the slope at the guard.

**Instance (fixed 2026-09-29):** pentode `Vpk < 0` and `Vg2k < 1e-3`, and the
triode `Vpk < 1e-3` floor. A push-pull EL84 output stage drove a plate below
its cathode: 5779 of 48000 samples unsolved at 0.1 V; zero after the guarded
columns were zeroed. See DEVICE_MODELS.md, pentode "NR-Stability Guards".

**Related, not the same:** residual alternating between two values
(period-2) with full steps is Newton straddling a genuine derivative jump
(the root sits next to a kink); the sub-step ladder resolves those.

## Transformer-Coupled Circuit Failure Signatures

| Symptom | Cause | Fix |
|---------|-------|-----|
| NR diverges exponentially on transformer nodes | Non-positive-definite inductance matrix from inconsistent k values | Ensure all windings on same core have similar k. MNA builder warns on non-PD matrices |
| i_nl/v_prev inconsistent after max_iter | v updated one step ahead of i_nl on non-convergence | Re-evaluate devices at final v when NR fails (FIXED) |
| 1e-6 absolute tol on 290V circuit | Demands 3.4 ppb — unreasonable for LU precision | Use SPICE RELTOL: 1e-3 * max(\|v\|) + 1e-6 (FIXED) |
| Trapezoidal NR ringing | Marginally stable oscillatory mode at Nyquist | BE fallback catches these samples automatically (FIXED) |
| Output latches to full-scale near-DC/Nyquist on program transients; sine & two-tone sweeps pass; compile-time auto-BE never fires (quiescent rho stable) | Self-sustaining trapezoidal Nyquist `(-1)^n` limit cycle reached at a **large-signal** operating point (e.g. a one-way bias servo driven to cutoff), which the compile-time quiescent-OP spectral-radius promotion cannot see. The per-sample max-iter BE fallback fires once at onset but the cycle then converges cleanly each sample, so it re-latches. (jeffreys-tube V2, oomox 2026-07-28) | Runtime BE-latch net (nodal trap builds): lag-1 anti-correlation detector on the output, **input-aware** (only latches when the output is anti-correlated AND the input does not explain it — a bright near-Nyquist input tone is not a limit cycle), sticky→BE for the rest of the stream, cleared by `reset()`. Exposed via `diag_be_latch_count`. Emitted for saturating-inductor circuits since 2026-09-28 (it had been excluded because the old decimated saturation update did not reach the BE matrices; that path is gone). Deterministic pin: `.integrator {trap\|be}`. (FIXED 2026-07-28) |
| Incomplete transformer coupling matrix | Missing K directive between windings on same core | Add K for ALL winding pairs; non-PD det warns |
| `diag_be_latch_count` = 1 on a step into a transformer with an open (megohm-loaded) secondary; a few-mV sample-to-sample alternation after the edge | Not saturation: the secondary's leakage into the megohm load is a stiff linear mode (τ = L_leak/R ≪ T, trapezoidal factor ≈ −1). Measured on golden `sat-core-open/step`: identical with the core made linear (`ISAT=100`), gone with a 600 Ω load | Nothing to fix in the core model; the latch removes the ring (corr 0.9999994). See SATURATING_TRANSFORMERS.md §3.4 |

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
converged on a KCL-violating fixed point. The false fixed point is committed
and carried to the next sample through the history, drifts sub-Hz, and
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
   runs. No constrained resolve. The committed (KCL-violating) state is the
   next sample's history.
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

## The Whole-System `z = −1` Walk (Algebraic Rows Held Only on Average)

The whole-system trapezoidal form sums KCL at `n` and `n+1`
(`A_neg = alpha·C − G`, sources as `b(n) + b(n+1)`, `N_i·i_nl_prev` in the
RHS). On a row with no capacitor or inductor (a KCL row at a capless node,
a source constraint), and on any combination whose capacitor currents
cancel (e.g. `KCL(a) + KCL(b)` across a coupling capacitor), that form
enforces only the AVERAGE of the equation over the step,
`(f(n) + f(n−1))/2 = 0`, not `f(n) = 0`. An accepted solve's residual on
such a row does not decay: it is fed into the next sample with a pole at
`z = −1`, `e(n+1) = ρ(n+1) − e(n)`.

**Fingerprint:** the row's KCL residual alternates in sign every sample,
`r(n) + r(n−1) ≈ 0`, with `|r|` far above the solver floor. Read the
residual of the capless row (or the cap-cancelling combination) itself; a
lag-1 detector on a filtered output misses it when the network downstream
removes the Nyquist content.

**Deck class:** nonlinear circuits whose accepted solves leave a residual
(NR tolerance, iteration cap, held samples, chord steps, op-amp active-set
pins) on capless rows or across coupling capacitors: diode clippers behind a
coupling cap, railing op-amps, saturating chokes, DK pot sweeps.

**Fix: the charge (companion) form** (`COMPANION_MODELS.md`). The generated
integrator carries the capacitor currents `q_dot = C·ẋ` as state, enters
every source once at `n+1`, and enforces KCL at `n+1` on every row; the KCL
residual of a committed sample is that sample's own solve residual.
Measured on a cap-coupled diode witness (nodal full-LU, forced trap,
`KCL(a) + KCL(b)` over the last 0.1 s): 1.31–1.52 µA at 96 kHz under the
whole-system form vs ≤ 0.011 µA under the charge form (0.43–0.52 vs
≤ 0.005 µA at 192 kHz).

Related consequences:

- **Hand-poked state.** Under the charge form the history is
  `alpha·C·v_prev + q_dot`: a capless row carries no `v_prev` history at
  all and a stiff (small-C) row only `alpha·C·v_prev`, so writing `v_prev`
  alone barely moves the next sample there, and `input_prev` is not in the
  RHS (only the sub-step input ramp reads it). To excite a mode in a test,
  kick `q_dot` (as `be_latch_entry_tests.rs` does). Set `q_dot` consistently with `v_prev` (from KCL:
  `q_dot = RHS_CONST + N_i·i_nl − G·v` on the charge-carrying rows), or use
  `reset()` / `set_dc_operating_point()`. Under the whole-system form a
  poked `input_prev` without the matching input-node voltage alternated
  ±53 V at `in` for a whole render on a saturating-core fixture.
- **Mid-run component changes** (a switch, a pot rebuild) start from a
  `q_dot` built on the old values. Breakpoint-BE (one backward-Euler sample
  after the change) does not read that `q_dot`, re-seeds it, and damps the
  mode the step excited; a capacitor change needs it
  (`charge_form_c_switch_tests.rs`). An op-amp pin or release takes no BE
  sample: the pinned solve commits a consistent `q_dot`
  (`OPAMP_RAIL_MODES.md`).

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
rate-independent (`A - A_neg = G`, no rate terms), so values found at 200 Hz are valid at
any target rate. The settled state is cached in `dc_operating_point` and `settled_i_nl`
for subsequent `reset()` calls — the expensive low-rate phase runs only once.

**Result**: 4kbuscomp BE fallback reduced from ~100% of samples to <1% (3-34 out of 4800+).
Sub-step NR handles the rest. Circuit stable at all amplitudes (0.001–1.0V).

## Precision Rectifier Transient NR — VCCS Back-Sub Contamination (FIXED 2026-04-16)

> **Superseded 2026-09-29: the automatic cap is removed.** Under the charge form and active-set pinning the uncapped solve converges on every sample, and the cap moved the fixed point (a biased half-wave rectifier sat 4.5 mV off ngspice with it, 0.2 µV without; see DEVICE_MODELS.md "Transient AOL Cap"). `AOL_TRANSIENT_CAP` remains as an author's key and routes nodal. The DC-OP copy of the classifier (`dc_opamp_is_sidechain_rectifier`) still seeds the DC homotopy, which finishes at full AOL. The history below is the original fix.

The full-LU NR path had a structural problem with high-gain VCCS op-amps:

1. The NR Jacobian `g_aug = A - N_I*J_dev*N_V` inherits Gm ≈ 2000 S from A = G + alpha*C
2. LU back-substitution computes v_new at op-amp outputs (400kV+ before clamp)
3. Neighboring nodes are computed FROM the unclamped op-amp output during back-sub
4. Post-solve rail clamp fixes v_new[op_out] → 11V, but v_new[neighbor] is already 8000V+
5. NR convergence check only monitors device nodes, not linear neighbors — declares converged
6. Contaminated v_prev propagates: `state.v_prev = v`, next sample's history (`A_neg * v_prev`) feeds
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
applied at G-matrix build time (constant cost, baked into the emitted G/A) and
propagates automatically to A, A_be, the sub-step `a_sub`, and the runtime
`rebuild_matrices` path. The charge-form history matrices (`A_neg = alpha*C`,
`A_neg_be`) carry no `G` and are unaffected.

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

## Hard-Switching NR Starvation on Marginal Astables — do NOT force convergence (gmin-continuation, tried + reverted 2026-08-15)

On a stiff hard-switching sample a positive-feedback junction can pin to v/vt≈300
with `i_dev` at the `safe_exp` ceiling, leaving the trap Jacobian ill-conditioned
and the NR residual stuck flat → the sample hits `MAX_ITER` and falls to the BE
fallback ("NR starvation", e.g. ~2–3% of samples on the Farfisa G10 divider).

It is tempting to fix this by porting the DC-OP solver's **Gmin stepping**
(DC_OP.md) into the per-sample solve — add `gmin` to the node diagonals, ramp
1e-2→1e-12 warm-started, accept the final gmin≈0 solve. This was built as an
opt-in `--gmin-continuation` flag (trap, BE-fallback, and hybrid trap+breakpoint-BE
variants) and **REVERTED**: on a **marginal self-oscillator it is net-negative in
every form.** Certified with an interval-histogram deglitch rig
(`openfarf tools/divider_intervals.py` on a `simulate --probe` CSV — a raw
crossing detector reads trap G10 as chaos and must not be used), term_2d
spurious-transition rate (lower = better):

| config | term_2d spurious |
|---|---|
| plain BE fallback ("starved", no flag) | **8.9%** (optimal; hardware-faithful) |
| gmin in BE fallback | 12.2% |
| gmin in trap loop | 32.9% |
| hybrid (trap gmin + breakpoint-BE tick) | 62.3% (amplitude inflated ~7 V vs 4.4) |

**Why forcing convergence loses on an astable:** trap-converging the stiff sample
excited the whole-system trap form's marginal `z=-1` mode on capless nodes
(its g-in-both-A-and-A_neg ring; this comparison was measured under that form) → glitches + amplitude drift; and even BE-converging
is *slightly worse* than the plain (non-converged) BE fallback, because the
BE-fallback state sits more consistently on the astable's limit cycle than a
gmin-converged one. **More solver effort = worse output here.** The right tool
for a marginal astable is the BE fallback / auto-BE / `--backward-euler` / the
"starved" recipe — accept the trap `nr_max_iter_count` and let BE catch it.

**When Gmin-continuation WOULD be the right fix (resurrect from git `041ac79`
/ `aa2c7ce` for this):** a **non-oscillating, convergence-limited** stiff circuit
— a hard clipper / heavy-clip stage / rectifier that genuinely fails per-sample
NR at a switching edge but has NO marginal limit cycle, so the CONVERGED trap
solution is exactly what you want (accuracy-limited, not stability-limited).
Profile first to confirm convergence-limited vs marginal, and certify with the
interval-histogram rig before shipping. Implementation note: warm-start the
homotopy (reset `v` to `v_prev` at level 1 only), gate off BE and
saturating-inductor builds.

## Codegen Emission Footguns

Rules for writing code that emits Rust. Surfaced across Phase E.5 of the
Oomox DC-OP recompute work — none are solver bugs, but all of them either
produce code that fails `rustc` or produce code that silently disagrees
with the compile-time solver and makes tests hunt the wrong ghost.

### 1. `format!("{x}")` on an `f64` const emits integer literals

`const DAMP_THRESHOLD: f64 = 10.0;` formatted via `format!("{DAMP_THRESHOLD}")`
writes the literal `10` into the generated source — no trailing `.0`,
because Rust's `Display` for `f64` drops the trailing zero on round
values. If the emitted line puts that literal next to an `f64` local,
`rustc` rejects it (`expected f64, found integer`).

```rust
// Wrong — emits "if max_delta > 10 { ... }" which fails to typecheck.
format!("if max_delta > {DAMP_THRESHOLD} {{ ... }}")

// Right — emits "if max_delta > 10.0_f64 { ... }".
format!("if max_delta > {DAMP_THRESHOLD:.1}_f64 {{ ... }}")
```

Always emit explicit suffixes (`_f64`) when using `f64` consts inside
emitted code — it's cheap, unambiguous, and survives `{x}` vs `{x:?}`
formatter choice changes. The `fmt_f64` helper already does this for
matrix entries; extend the same habit to thresholds and clamp limits.

### 2. Test helper unconditionally stamps `mna.g[0][0] += 1.0`

`tests/dc_op_recompute_tests.rs::generate_dk` (and similar helpers
elsewhere) unconditionally stamp 1 S of input conductance at node 0
before building the kernel — the convention is "input signal arrives
at node 0 through a 1 Ω Thevenin source." The codegen config also
passes `input_resistance: 1.0` with `input_node: 0`, so the compile-
time DC OP stamps `g_dc[0][0] += 1/R_in = 1.0` a **second** time on
its own working copy.

When a test netlist accidentally puts a VCC-bearing node at MNA index 0
(i.e. the first non-ground node to appear in the netlist is also the
supply rail), the compile-time DC OP sees 2 S of shunt to ground at
that node while the runtime NR's baked `G` matrix sees only 1 S. On
simple resistive networks the error shows up as a **2× discrepancy on
the VS branch-current variable** (the VCC aug row), while node
voltages themselves can still look plausible. Symptom: runtime
`recompute_dc_op` converges to values that differ from the baked
`DC_OP` constant by a factor of 2 at the augmented row (and by the
same ratio through every node KCL that touches the rail).

**Fix**: declare an isolated signal-side resistor first in the test
netlist so node 0 is always a benign input, not a power-supply node:

```spice
* Good — node 0 = "in", VCC at node ≥ 1. Single input-conductance stamp.
R_in_load in 0 10k
VCC vcc 0 5.0
R1 vcc mid 1k
...

* Bad — VCC is the first element, so mna.node_map("vcc") = 0.
* Test helper stamps g[0][0] += 1 at VCC node; codegen DC OP stamps
* again via input_resistance. Runtime NR converges to half the
* baked DC_OP on the VS branch current.
VCC vcc 0 5.0
R1 vcc mid 1k
...
```

Not a bug in the emitter, but a test-writing contract: *any* helper
that force-stamps input conductance assumes node 0 is the signal input.

### 3. `RHS_CONST` is ×1 on every row — use it verbatim as the DC RHS

The emitted `RHS_CONST` carries every DC source once (current sources
on node rows, `V_dc` on VS / VCVS / ideal-transformer aug rows), under
both integrators: the charge form enters each source at `n+1` only.
When reusing `RHS_CONST` to derive a **DC fixed-point RHS**:

```rust
let b_dc = RHS_CONST;   // verbatim: A - A_neg = G on every row
```

Scaling any row changes the fixed point: a halved VS row gives
`v_plus - v_minus = V_dc / 2`, a halved node row half the DC
current-source injection — both converge silently and sound wrong
later. `.runtime` voltage-source fields write to their VS aug row and
are added to `b_dc` as they are. (The library `dk::build_rhs_const`
is the whole-system form and doubles the node rows; do not use it as
the source of an emitted `RHS_CONST`.)

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
| BoyleDiodes: `.inject` conductance and `.linearize` reduction silently gone; Newton budget tuned for the circuit without the diodes | `generate_nodal` rebuilt the MNA from the augmented netlist and restamped only G_in and junction caps | Augment the netlist in `build::build` before the MNA is assembled; the mode routes nodal; codegen refuses an un-augmented `BoyleDiodes` netlist (FIXED 2026-09-29) |
| BoyleDiodes false convergence: trap NR declares converged with wildly wrong v[buf_in] on first signal sample | Voltage-step convergence check passes when chord_j_dev is many OOM stale; no residual gate | Residual check + adaptive refactor trigger (>50% j_dev change), BoyleDiodes-gated (FIXED `39397d1`) |
| BoyleDiodes heavy clipping (amp ≥ 0.05 on Klon): NR diverges, raw 45–3068 V | Chord-LU Newton direction wrong (not just magnitude). Gmin/line-search can't fix wrong direction | Not a blocker. Auto-routes to ActiveSetBe. BoyleDiodes opt-in only. See "Op-amp BoyleDiodes Failure Signatures" above |
| DK kernel singular at col N (transistor ladder / cap-only nodes) | Intermediate nodes connected only through BJT junctions + bridging caps have G≈Gmin (1e-12). A = G+2C/T nearly singular | Routes to nodal automatically. DK limitation for series-BJT topologies without parallel resistors |
| Nodal Schur flat output (+9 dB, no filtering) on BJT ladder | S = A⁻¹ has extreme entries (>1e6) at cap-only nodes → K spans 10^11 → J = I-J_dev·K swamped | S magnitude check: `max\|S\| > 1e6` routes to full LU NR. Invariant to FA reduction (FIXED 2026-04-10) |
| simulate/analyze crash "Augmented fallback failed" | DK fails, augmented DK also fails, no nodal fallback | Dummy kernel with dk_failed=true, falls through to nodal codegen (FIXED 2026-04-10) |
| BJT diverges from ngspice at NF ≠ 1 or high injection | q2 used bare VT and omitted `-1`; Ib forward was divided by qb | q2 uses `cbe/IKF + cbc/IKR` where `cbe = IS*(exp(Vbe/(NF*VT))-1)`; Ib ideal forward NOT divided by qb. Matches `bjtload.c:571,618` (FIXED `d7427c4`) |
| Op-amp clips the WRONG rail (asymmetric supplies); comparators/rectifiers settle backwards | VCCS differential polarity inverted in all 3 mna.rs stamp paths (V_out = −AOL·(V+−V−)); validation refs hand-encoded the same flipped element, so ngspice agreed | Stamps flipped (`np -= gm / nm += gm`), refs rebuilt, goldens regenerated. Closed-loop gain differs only at O(1/AOL) — which is why 6-nines validation never saw it (FIXED `3e246cb`) |
| Triode stages ~half gain/current vs datasheet; catalog "fits" needed Kg1 inflation | Koren triode missing the ×2 factor (`(PWR+PWRS)/KG1`); catalog carried Koren's ×2-fitted constants; 12AX7 refit against a misread datasheet row (1.2 mA is at Vgk=−2, not 0) | ×2 restored in devices + all template sites; 12AX7 back to Koren 1060; catalog test bounds re-derived from true datasheet rows (FIXED `3e246cb`) |
| 2x/4x oversampled plugins: HF droop (~−1.5 dB @ 11 kHz class) and aliasing under drive | Decimator clocked both allpass chains at internal rate (not polyphase; stopband ≈ 0 dB) AND HB coefficient tables were invalid designs (−16/−20 dB best-case) | True polyphase decimator (hiir convention) + hiir-designed tables (−87 dB measured); 4x stage order corrected; measurement tests added — old suite only tested DC settling (FIXED chunk-3 campaign) |
| DC OP converges DirectNr to a NON-PHYSICAL basin (e.g. floating series diode pair at Vf=−96 V) or period-2 oscillates | pnjlim corrections distributed via N_vᵀ without ‖row‖² normalization → floating junctions got 2× correction (direction-reversing); grounded-junction test circuits masked it | `distribute_junction_correction()` normalizes by Σ N_v[idx][j]² (pseudo-inverse); applied to diode/BJT/FET arms (FIXED chunk-3 campaign) |
| Diode-connected BJT (`Q1 b b e`) / gate-strapped JFET evaluates at phantom voltage | N_v/N_i stamps assigned (`=`) instead of accumulated — tied terminals overwrote instead of cancelling to 0 | All device stamps accumulate `+=` into zeroed rows/cols; K-diagonal validator handles the resulting zero rows via nodal fallback (FIXED `3e246cb`) |
| Vendor .model card silently produces absurd currents (`IS=6.734f` → 6.7 A) | Femto suffix was dead code (trailing-F stripped as a unit letter before suffix match); also paren-less cards dropped ALL params silently | Femto parsed in model-param context; paren-less form supported; unknown trailing element tokens hard-error (FIXED `3e246cb`) |
| `simulate` renders digital silence / a plugin is silent, with clean health counters and exit 0 | A mistyped node name (`C3 n3 n4` written `C3 n33 n4`) invents a node and floats the stage; every number melange printed was correct for the circuit it was handed | `melange_solver::topology` refuses a node that appears on exactly ONE terminal of a two-terminal element, naming the node, the element, the source line and the nearest existing spelling. Runs in `compile`/`simulate`/`analyze`/`validate` via `pipeline::topology_gate`; `nodes`/`dc-op` report it without refusing (2026-09-23) |
| A node with no DC path to ground passes silently (gmin regularizes it) | Cap-only DC island. The old validate-side scan unioned EVERY terminal of every non-capacitor element and had no port stamps, so it cried wolf on the input node of every cap-coupled deck and missed islands held together by a terminal that does not conduct at DC (op-amp input with default `RIN=+inf`, MOSFET/JFET gate, tube grid) | Island check rebuilt on the DC graph melange actually stamps, in `melange_solver::topology::dc_edges`, with the input Thevenin/`.inject` conductances in the graph. WARN-grade: true islands exist (non-polar electrolytic pairs, capacitive dividers) (2026-09-23) |
| Parser panics on non-ASCII component value (e.g. `1ſ`) | `to_uppercase()` changes byte length; byte-slice landed mid-codepoint | `parse_value()` normalizes non-ASCII at entry (µ/μ → u; else `Err`) (FIXED `e96c340`) |
| Click on every pot/switch move in generated plugin | `rebuild_matrices()` zeroed DC blocker + oversampler state | Removed filter state resets from DK Schur `rebuild_matrices` (FIXED `07712a0`) |
| Zipper noise on knob automation | Pot values read once per buffer via `.value()`, smoother unused | Per-sample `.smoothed.next()` read in plugin template (FIXED `7c0fc02`) |
| NR max-iter on wide-range pot jumps (preset recall, automation step) | Stale `v_prev`/`i_nl_prev` from different operating point; NR starts far from new bias | Explicit `state.recompute_dc_op()` after `set_pot_N` (DK path resolves new operating point; nodal is a stub — NR catches up over ~`WARMUP_SAMPLES_RECOMMENDED` samples). Shipped 2026-04-19 in Phase E. Supersedes the 2026-04-11 warm DC-OP re-init (commit `e8e18a7`, stripped 2026-04-20) — see next row |
| Click on every per-block knob update on a log-taper `.pot` (ratio ≥ 1000:1) | Warm DC-OP re-init gate (`|r - r_prev|/r_prev > 0.20`) fired on every block at DAW-typical sizes, snapping `v_prev`/`i_nl_prev` to DC_OP mid-signal | Strip the gate from `set_pot_N`/`set_switch_N`/`set_runtime_R_<field>` (FIXED 2026-04-20). Setters are reseed-free; `recompute_dc_op()` is the caller-driven mechanism for preset recall. Regression: `pot_tests::log_taper_block_sweep_no_clicks` |
| DC OP wrong polarity for precision rectifier op-amps (e.g., 4kbuscomp sidechain TL074 at +11V instead of -11V) | Multi-equilibrium: AOL=200K overshoots NR between rails in one iteration | AOL capped at 1000 in DC G matrix, op-amp output seeding, per-iteration rail clamp in DC OP NR (FIXED 2026-04-15) |
| DC OP diode reverse bias: nodes float to non-physical (91kV) when diodes off | DC OP `evaluate_devices_inner()` ignored BV/IBV and had zero reverse-bias conductance | BV/IBV breakdown added to DC OP diode evaluation; device-level Gmin 1e-12 S (FIXED 2026-04-15) |
| Precision rectifier transient: trap NR never converges, every sample falls to BE | DC OP fails → `DC_NL_I` garbage (1.21e11 A) → coupling caps uncharged after 50-sample warmup (RC > 0.5s needs ~143K samples at 48kHz) | Low-rate DC warmup: 200 Hz × 1000 samples (5s circuit time) charges caps before transient NR. BE fallback <1% (FIXED 2026-04-16). See "Low-Rate DC Warmup" above |
| Full-LU NR: VCCS back-sub contamination (1.18B V at linear neighbor nodes) | Op-amp VCCS Gm ≈ 2000 in A. LU back-sub computes v_new[op_out]=400kV, uses it for neighbors before post-solve clamp. NR convergence checks device nodes only | Selective op-amp Gm cap via Rule D' topology classifier (n_plus on non-zero DC rail AND diode connects output→inv-input through R-only path). User override: `.model OA(AOL_TRANSIENT_CAP=N)` (FIXED 2026-04-16). See "Precision Rectifier Transient NR" above |
| DC OP basin trap: precision-rectifier op-amp railed at −VSAT with clamp diode at 11 V forward bias even after AOL cap | Multi-equilibrium: correct basin + pathological basin both self-consistent. Homotopy-path problem, not cap-magnitude | FIXED 2026-04-17 (commit `b771512`). AOL continuation alone did not converge (physical basin at AOL=1 is v_out ≈ −6.3 V, not the VEE seed). Actual fix is post-fallback refinement NR: `seed_sr_feedback_diodes` clamps feedback-diode terminals to `v_out ± 0.65 V`, then direct NR in `aol_cont_mode` runs from the synthesized state and converges in 411 iters. Gated on `has_sidechain_rectifier`. See "Precision Rectifier DC OP Convergence" above |
| Transient chord-NR false convergence on DC-railed op-amp: `v[n_out] = ±VSAT` every sample, forward-biased feedback diode drawing 10+ A, sub-Hz drift into exponential blow-up at d ≥ 0.9 s (amp 0.1) / d ≥ 3.3 s (amp 0.01) | Strict-inequality `active_set_engaged` check doesn't count clamped-to-rail as engaged; BE fallback and active-set resolve never fire; residual leaks through `A_neg·v_prev` / `N_I·i_nl_prev` | PARTIAL FIX 2026-04-17 (commit `c3d3eae`): residual check in `nodal_emitter.rs` extended from `BoyleDiodes` to `BoyleDiodes \| ActiveSetBe \| ActiveSet`. After damped NR step, re-evaluate `i_nl_fresh` and force `max_step_exceeded = true` on >1e-3 relative mismatch. Gets d ≤ 2 s stable at all amps with zero NR/BE events. **d = 5 s still diverges on the original netlist (82,899 NR max-iter hits)** — closed only when combined with netlist-side `.model OA_TL074 VSAT=11 → 13.5` fix (uncommitted in melange-circuits as of 2026-04-17). See "ActiveSetBe Chord-NR False Convergence" above and `memory/project_4kbuscomp_chord_false_convergence.md` for candidate next steps (tighter residual tol, adaptive refactor, KCL-consistent active-set resolve, every-iter refactor on rail engagement) |
| `.switch` resistor off by (1/static − 1/pos_0) on every switch change, `set_switch_N(0)` silent no-op at init | Initial `switch_position = 0` but G stamped at static netlist value, not pos-0. `rebuild_matrices` computes delta against static; set_switch_N(0) short-circuits when already at 0 | `MnaSystem::switch_default_overrides` stamps G/C/L at pos-0 (canonical baseline). `SwitchComponentInfo.nominal_value` stores pos-0 so codegen delta baseline matches. `log::info!` when static ≠ pos-0 (FIXED 2026-04-17 commit `586c2a8`) |
| Generated Rust fails to compile at `if max_delta > 10 {` / `delta.clamp(-50, 50)` | `format!("{DAMP_THRESHOLD}")` on a `const f64 = 10.0` emits literal `10` (no trailing `.0`) | Use explicit suffix: `format!("{DAMP_THRESHOLD:.1}_f64")`. See "Codegen Emission Footguns" above (Phase E.5 commit `6322bf7`) |
| Runtime `recompute_dc_op` converges to half the baked `DC_OP` on the VS branch-current variable | Test netlist puts a VCC node at MNA index 0; test helper stamps 1 S input conductance there; compile-time DC OP stamps a second 1 S via `config.input_resistance` | Put an isolated signal-side resistor first in the test netlist so node 0 is a benign input. See "Codegen Emission Footguns #2" above (Phase E.5 commit `6322bf7`) |
| Runtime DC OP VS row converges to `V_dc / 2` (or node row converges to 2× the compile-time value) | DC RHS rows scaled inconsistently with the per-sample `RHS_CONST` (at the time, a whole-system ×2 on node rows, copied verbatim or halved uniformly) | Under the charge form `RHS_CONST` is ×1 on every row and `b_dc = RHS_CONST` verbatim. See "Codegen Emission Footguns #3" above (original fix Phase E.5 commit `6322bf7`) |
| Thermal-noise output ~150× hot vs ngspice on multi-node feedback amp; output RMS scales with `fs` instead of being fs-independent (e.g. wurli-preamp at zero input: 1.3 mV @ 48k → 1.7 mV @ 88.2k → 3.6 mV @ 176k vs ngspice ~8 µV) | Single-draw white-Gaussian thermal stamp injected energy at all frequencies including Nyquist. Trapezoidal MNA gives `A_neg[i][i] = -G[i][i]` for purely resistive nodes (no shunt cap) → eigenvalue `z = -1` at `fs/2` → noise at Nyquist accumulates as a stationary `(+1, -1, +1, …)` mode (lag-1 autocorrelation ≈ -0.9999). The kTC theorem test missed it because the test's RC has a cap at the output that absorbs Nyquist | Two-draw stamp `i_n[n] = w[n] + w[n-1]` with `w[n] = (scale/2)·sqrt(1/R)·g`. Sum's PSD ∝ `4cos²(πf/fs)` — exactly zero at Nyquist, ~flat at audio. Low-frequency PSD preserved → kTC variance unchanged. New state `noise_thermal_w_prev[NOISE_THERMAL_N]`, zeroed at `default()` / `reset()` / `set_seed()`. Shipped on **both DK and nodal codegen paths** via shared `build_noise_emission()`. Regression: `thermal_noise_no_nyquist_artifact_on_resistor_only_output_node` (FIXED 2026-04-24 commit `a5dff8c`). The two-draw pair was the whole-system form's image of the physical current; under the charge form every noise source is one physical draw at `n+1` and the integrator's `(1 + z⁻¹)` nulls Nyquist on charge-carrying rows — see NOISE.md "Constant derivation" and "Whole-system Nyquist history" |
| Codegen panics OOB on the LAST FA-reduced parasitic BJT in `rebuild_matrices` (`k[3][3] -= …`, M=3) **OR** silently corrupts K_DEFAULT for every preceding FA-reduced parasitic BJT when no `.pot`/`.switch` is present. Symptom seen on openwurli's full-GP wurli-preamp (2 FA-reduced BJTs + `.pot R_ldr` → panic) and inferred on steve-1073-output (3 FA-reduced parasitic BJTs, no pot → silent K_DEFAULT drift, ~30% perturbation on diagonal cells from BJT0 contaminating BJT1's slot, BJT1 contaminating BJT2's slot, BJT2's RB dropped off the end) | K_eff parasitic-BJT absorption (commit `ee884cf`, 2026-05-08) stamped `k[s][s], k[s][s+1], k[s+1][s], k[s+1][s+1]` based on `slot.start_idx`, assuming the full 2D BJT layout. For FA-reduced BJTs (`slot.dimension == 1`), `s+1` lands in the **next device's slot**, not the same BJT's Vbc row. The 1D FA path explicitly ignores parasitics by design (`ir.rs::detect_forward_active_bjts`: "Ib·RB ≪ Vbe for forward-active"), so K_eff stamping for FA-reduced slots is physics-incorrect regardless of OOB. The MNA-side parasitic internal-node expansion at `mna.rs:1420` already correctly gates on `DeviceType::Bjt` (not `BjtForwardActive`); K_eff codegen missed the corresponding guard | Gate both K_eff stamp sites on `slot.dimension == 2`: `rust_emitter/dk_emitter.rs:1589` (runtime `rebuild_matrices` stamp) and `rust_emitter/dk_emitter.rs:4451` (`parasitic_r_p_dk` codegen helper, which feeds both K_DEFAULT and K_BE_DEFAULT). FA-reduced 1D BJTs fall back to the FA path's existing no-parasitics handling. Steve-1073-output K_DEFAULT changes substantially after the fix; **previously-validated BA283 6-nines correlation needs re-running against ngspice**. Regressions: `forward_active_bjt_tests::test_k_eff_skips_fa_reduced_parasitic_bjts` (codegen identity vs parasitics-stripped baseline) + `test_k_eff_no_oob_panic_with_pot_rebuild` (runtime no-panic with `.pot`) (FIXED 2026-05-22) |
| Audio-rate `.runtime R` modulation produces a 4× low-frequency "pump" at HEAD vs pin `47b2702`, while static gain at every operating point is unchanged (e.g. wurli-preamp `.runtime R_ldr` tremolo: raw pump 7.96→~30 dB) | **NOT a broken solver — changed DEFAULTS between the pins** (fully reproduced 2026-05-28 with openwurli's repro kit; matching OLD's config on HEAD gives 7.96 dB byte-for-byte). Four compounding default changes: (1) `ae21d6b` auto-BE Nyquist discriminator (`stability.rs::trap_needs_be`, `ρ>0.999 && sign<0`) promotes wurli trap→**BE-primary**, and BE-primary pumps under per-sample rebuild (main source); (2) MAX_ITER auto-tune (`melange-cli/src/main.rs:1476`) gives wurli 85 iters but its marginal-Nyquist trap needs ~186 to converge → at 85 it falls to BE every sample (flat); (3) `ae21d6b` removed the trap-primary BE-fallback `N_I·i_nl_prev` term; (4) DC blocker added (metric-baseline shift only). wurli's Nyquist mode is real but INAUDIBLE (gain ~4 → µV limit cycle) vs noyce-cascaded-triodes (gain ~3800 → 28 mV audible) — so wurli should stay on trap, noyce/pipe-shouter stay BE | **FIXED 2026-05-28.** 3-part fix, validated end-to-end against openwurli's repro kit (preamp RAW 7.96 / SHADOW 5.67 dB, exact OLD match): (a) **gain-aware discriminator** — `stability.rs::trap_needs_be` second clause now also requires `max_abs_s > 1e5`, so low-gain marginal circuits (wurli-preamp, max\|S\|≈3e4) stay on trap while high-gain ones (noyce, ≈5e5) stay BE; first clause (ρ>1.002) unchanged so power-amp/pipe-shouter stay BE; (b) **MAX_ITER auto-tune** (`melange-cli/src/main.rs`) gates a +200 stiffness bonus on `stays_trap` (replicates the auto-BE decision) so marginal-trap circuits get enough iters (wurli 85→265, needs 186) while BE circuits keep their small bound (power-amp stays 70); (c) ~~restored BE-fallback `N_I·i_nl_prev` term (`process_sample.rs.tera`) for trap-primary history continuity~~ **REVERTED 2026-09-14 (v0.1.8.1)** — the stamped fallback double-counts every device's bias current (`N_I·(i_prev + i_n)` on a BE step) and is therefore not a fixed point of the DC OP; the "history continuity" rationale was measured in a MAX_ITER-starved config where the fallback fired every sample, and the final validated openwurli config runs `be_fb=0`, so the term was never load-bearing for the 7.96 dB match. See the philicorda-voicing-coupled row below; `test_be_codegen_omits_n_i_i_nl_prev_from_rhs` now asserts the term is absent from BOTH BE-primary build_rhs and the trap-primary fallback; also restored `#![allow(non_snake_case)]` to the codegen header. The ~186-iter trap is status-quo (OLD had it too), not a new CPU cost. NOTE: the related tremolo "regression" was NOT this bug — it was the default DC blocker stripping an LFO oscillator's DC offset (collapsing the CdS-driven r_ldr range); compile oscillator/LFO circuits with `--no-dc-block`. Stamp-into-BE-primary fixes FAILED (don't retry). Full analysis: memory `wurli_runtime_r_pump_investigation.md` |
| Extreme-IS diode (wide-bandgap card, e.g. `IS=1e-30 N=2.0`, Vf ≈ 3.2 V) in op-amp feedback clipper: DK silently emits open-loop output (78.7 V at gain 48, violating both diode law and ±13 V rail) with `nr_max_iter_count = 0`; nodal path maxes NR on ~every sample and latches DC | Legacy `40·n_vt` flat exp clamp in `diode_current`/`diode_conductance` caps current at `is·e^40` ≈ 2.4e-13 A for IS=1e-30 — clamp sits BELOW the device's vcrit (3.4 V), so the diode is **electrically absent at any voltage**, the circuit is linear, and NR genuinely converges on the diode-free system (the always-on current-residual gate is satisfied — this was NOT a chord/false-convergence bug; DK has no chord). Second compounding bug: `diode_*_with_rs` inner NR (0.7 V seed, 8 iters, flat 4·n_vt steps) cannot reach a 3 V junction knee | IS-aware extended exponential: for `x = v/n_vt > 40`, continue `i = e^(x + ln is)` in ln-current space (no overflow, dodges fast_exp ±40 clamp) up to `MAX_DIODE_FWD_I = 1e3 A`, then linear extension; legacy path bit-exact below 40·n_vt. Inner RS solve reseeded at vcrit with pnjlim steps, 32 iters. Fixed in `device_diode.rs.tera` + `melange-devices/diode.rs` (covers DC OP). Regressions: `diode::tests::test_wide_bandgap_*`, `numerical_edge_case_tests::test_wide_bandgap_clipper_clamps_at_diode_knee` (FIXED 2026-06-09). **ngspice caveat: ngspice silently floors IS at 1e-28** (verify with `showmod`), so SPICE correlation on IS<1e-28 cards shows a systematic Vf offset of `n_vt·ln(IS_ngspice/IS_card)` (+0.24 V for 1e-30 at N=2) — melange honors the card; cross-validate at IS=1e-28 (melange 3.331 V vs ngspice 3.316 V, 0.45%) |
| Shot-noise output on a stiff reverse-breakdown junction (`--noise shot`/`full`) is seed-dependent (σ varies 13–17 dB across RNG seeds), ~46 dB hotter than the physical `sqrt(4·q·I·fs)·Rz` prediction, and non-Gaussian (crest ~5 dB vs ~13.6 expected). Seen on the Noyce Zener source (`D(BV=5.1 IBV=1e-3)` at its 1 mA knee). Static gain at every operating point is unchanged — purely a noise-realization effect. lag-1 autocorrelation of the output ≈ −1.000 | Same z=−1 Nyquist pole as the thermal bug above, but on the **shot** path, which was never given the two-draw treatment (the 2026-04-24 thermal fix assumed junction parasitic caps kill the pole). A stiff breakdown junction defeats that: dynamic resistance `Rz ≈ n_vt/IBV ≈ 26 Ω`, so the 10 pF Cak pole sits at ~600 MHz — four decades above fs/2, leaving the node resistor-only at Nyquist. Single-draw shot injection excites the z=−1 pole into an fs/2 limit cycle, and the breakdown exponential **rectifies** it into the audio band (amplitude-dependent → seed-dependent σ; asymmetric → low crest; down-converted → hot). PRE-EXISTING since Phase 2 shot (2026-04-20), NOT a campaign regression — byte-identical at pre-campaign `dab5653` | Two-draw pair on the trap shot stamp `i_n = w[n] + w[n−1]`, per-draw `sqrt(4·q·|I|·fs)·0.5` (mirrors thermal); BE-primary stays single-draw. New state `noise_shot_w_prev`, zeroed at `default()`/`reset()`/`set_seed()`/NaN recovery; zero-current guard sets `w_new=0` when `|I|<1e-15` so the lagged half flushes. Validated @96k: σ spread 17→0.40 dB, level +46 dB→~140 nV, crest→13 dB, lag-1 −1.000→+0.48. Regression: `noise_psd_validation.rs::shot_noise_no_nyquist_artifact_on_stiff_breakdown_junction`. (FIXED 2026-07-19 commit `a472807`.) Under the charge form shot is one physical draw `sqrt(q·|I|·fs)` at `n+1`, no lag state — see NOISE.md "Whole-system Nyquist history" |
| Nodal path routes to nodal *citing* "trapezoidal unstable (spectral radius X > 1.002)" but then ships `Integration: Trapezoidal` and runs it anyway; `simulate` times out (wurli-power-amp: >100 s per 0.5 s of audio) or explodes under `--force-trap` (2721 V on ±22 V rails, `nr_max_iter_count`/`be_fallback_count` ≈ every sample) | Two independent spectral-radius estimators disagree across the 1.002 threshold. `routing::auto_route`'s `dk_unstable` check (`routing.rs::compute_spectral_radius`) is a fixed-20-iteration, non-deflected power method on the DK kernel; the nodal auto-BE gate re-measures with the shared, converged, input-deflated `stability::analyze_trap_stability_deflated` on its own matrices. On wurli-power-amp: router 1.0040 vs nodal-local 1.0005 (both correctly computed on the SAME underlying kernel matrix — the router's number is simply less accurate, not a different valid signal; verified by recomputing the accurate estimator directly on the router's own matrices). The nodal-local number is genuinely marginal (ρ>0.999, negative dominant eigenvalue) but under the existing `max\|S\|>1e5` gain-gate (calibrated for *audibility* of a roundoff-level marginal mode, not applicable when ρ is robustly, reproducibly >1) declines promotion on its own | Do NOT trust the router's raw number unconditionally — verified it also force-promotes circuits whose accurate local estimate is comfortably stable (tungsten-thunder-horse: 0.8157 local vs 1.1163 raw router number), causing a severe non-benign golden-audio regression (correlation collapsing to 0.006–0.6). Fix: `stability::router_corroborates_marginal_instability` — the router's finding only lifts the gain-gate when the LOCAL accurate estimate independently corroborates a marginal mode (ρ>0.999 && dominant_sign<0). `CodegenConfig.router_dk_unstable`/`router_dk_spectral_radius` threaded from `routing::auto_route` at all CLI + melange-validate call sites. FIXED 2026-07-25 commit `b0dcb27`. Also resolved a previously-separately-reported wurli-power-amp overdrive divergence (same bug: input 0.5 V went from 2091 V to 20.40 V, correct clip) — verified independently in melange-circuits (`c1fe92a`). **Residual, not covered by this fix — FIXED 2026-08-03**: erratic, convergence-path-dependent internal-node blowup (not a clipping-level threshold — e.g. amp 0.05 → 16,079 V, amp 1.00 → 27,977 V internal, while amp 0.10/0.30/0.50 stayed physical). Root cause: the nodal full-LU BE-fallback's (and primary loop's) "global node voltage damping" ratio had a `.max(0.01)` floor that let a fixed ≥1% fraction of an arbitrarily large raw NR step through — at a class-AB crossover device-state transition the raw step reached 3.8e7 V, so even floored at 1% the applied step was ~3.8 kV, launching the trajectory into a nonphysical regime the voltage-step-only BE convergence check then falsely accepted (its relative tolerance scales with the already-diverged voltage). Fix: removed the `.max(0.01)` floor (`nodal_emitter.rs`, primary-loop damping and BE-fallback damping) so the ratio divides uncapped, keeping every iteration's worst-case node step at exactly the intended ceiling regardless of raw delta size. All amplitudes now stay within 20–32 V internal. Regression: `nodal_be_fallback_alpha_floor_tests.rs` |
| A trap-primary nodal/DK build rings at exactly fs/2 on an internal node from the FIRST sample after any `set_switch_*`/`set_pot_*` (or after a max-iter fallback / runtime BE-latch), on silence, with `nan_reset`/`magnitude_reset` = 0; amplitude is drive-independent (philicorda-voicing-coupled: anode 46–57 V p-p, 48 V sample-to-sample step, r1 = −0.9975, fs/2 energy fraction 0.999) and invisible at a filtered output node (`out` r1 = +0.9998); a static build at the same switch positions is clean; a probed capless node under `--backward-euler` is clean; the latched/fallback steady state sits at a shifted DC (anode 183.9 V vs 192.6 V) | The per-sample BE **fallback** RHS (shared by breakpoint-BE, max-iter fallback, runtime BE-latch, glow lit-hold) stamped the trap-midpoint `N_I·i_nl_prev` on top of the BE step's own `S_ni_be·i_nl(n)` — every device's bias current counted twice — so a fallback sample is not a fixed point of the DC OP (one sample from the exact OP moved the anode 192.64 → 162.93 V; exact python replay of the emitted matrices reproduces 162.929 / 168.907 to 3 decimals). The excursion lands in `null(C)`, which is the exact `z = −1` eigenspace of the trap operator (`(αC−G)x = −(G+αC)x ⇔ Cx = 0`; 26 such modes on that deck), so the resumed trap carries it undamped and the tube bounds it. The compile-time promoter missed it because `power_iterate_rho_sign` does not converge on a 26-fold degenerate −1 cluster (read 0.964/+1 at the 500-iteration cap; exact spectrum ρ = 1, sign −1, max\|S\| = 7.7e5) and the runtime latch missed it because it only watches `OUTPUT_NODES[0]` — both are separate open items (design review). Golden corroboration: `qapla-1a` potsweep render was carrying this (fs/2 band −8 dBFS, peak at the clamp, 91909 fallback samples, latch fired) | **FIXED 2026-09-14 (v0.1.8.1).** Drop `N_I·i_nl_prev` from every BE-fallback RHS (`nodal_emitter.rs` Schur + full-LU fallbacks, `dk_emitter.rs` `be_rhs_lines`, `process_sample.rs.tera`); at the time the trap-primary Step 1 / `build_rhs` kept its midpoint half; under the charge form no RHS stamps `N_I·i_nl_prev` at all. A clean BE step annihilates `null(C)` history exactly, so breakpoint-BE and the latch now do what they were designed to do. The 2026-05-28 "history continuity" rationale for the stamp is retired (see the wurli row above). Regression: `be_fallback_fixed_point_tests.rs` (codegen-string on all three emitters + compile-and-run `set_switch` at the DC OP on silence: anode stays within 1 mV of DC, pre-fix 25–30 V; driven lag-1 > 0.9) |
| `melange simulate`/`analyze --noise <mode>` produces byte-identical, seed-invariant output to `--noise off` — a clean-looking measurement of a circuit whose noise is actually silently disabled, no warning | CLI plumbing and codegen are both correct (`--noise full` DOES emit the full noise machinery — RNG state, per-sample injection, `set_noise_enabled` method). The generated `CircuitState`'s `noise_enabled` runtime master switch defaults `false` (correct for the plugin seam, where the host UI opts in), but neither `simulate` nor `analyze` ever called `set_noise_enabled(true)` — for these CLI commands, passing `--noise <mode>` IS the opt-in, and nothing wired that through | `generate_simulate_main`/`generate_analyze_main` (`codegen_runner.rs`) gained a `noise_enabled: bool` param; when true (caller passes `opts.noise_mode != NoiseMode::Off`), the generated `main()` emits `state.set_noise_enabled(true)` right after `CircuitState::default()`. Regression: `cli_integration::test_simulate_noise_full_actually_injects_noise` (asserts `--noise full` differs from `--noise off` and two different `--noise-seed` values differ from each other). Found by melange-circuits 2026-07-25; FIXED same day |
| Tube shot/flicker noise (`--noise shot`/`full`) is ~100–1000× too hot on triode circuits (and lands on the wrong port for pentodes) | The Phase 2 shot collector and Phase 3 flicker collector hard-coded `nodes[0]`/`nodes.last()` with a comment wrongly claiming triodes were ordered `[plate, grid, cathode]`. Actual `mna.rs` order is `[grid, plate, cathode]` for triodes and `[plate, grid, cathode, screen, [supp]]` for pentodes — so shot/flicker stamped at **(grid, cathode)** for triodes (high-Z grid node × full stage gain → the 100–1000× inflation) and **(plate, screen)/(plate, suppressor)** for pentodes instead of (plate, cathode) | Replaced the magic-index lookups in both collectors with the typed `NonlinearDeviceInfo::junction_current_ports()` accessor (`mna.rs`), so the device class owns its port mapping and the collectors can't drift from the node ordering. Locked by 8 `codegen_verification_tests::shot_flicker_ports_match_*_element` tests. FIXED `ae21d6b` 2026-05-15. **RE-VERIFIED 2026-07-28** (melange-circuits re-flagged it as still open — it was not): fresh + installed binaries emit `NOISE_SHOT/FLICKER_NODE_I=plate, _J=cathode` on DK and nodal; pentode Ip goes through the Phase-5 partition source at (plate, cathode), not a bare-shot port; 4-node and 5-node identical. If this is reported again, it is a **stale-status phantom** (pre-May source or pre-May-15 generated code) — check the binary before acting. See memory `triode_shot_node_already_fixed.md` |
| Generated self-oscillating circuit (astable / LC oscillator) runs for ~10–30 ms then FREEZES to **exactly** 0.0 Vpp indefinitely under deterministic zero input (`--amplitude 0`); toggling a `.switch` restarts oscillation briefly, then it re-freezes | NOT a codegen/dissipation bug and NOT a `--force-trap` misapplication (verified: `--force-trap` pins Trapezoidal correctly, CLI and generated paths behave identically). A perfectly-noiseless, perfectly-symmetric, zero-input oscillator simulation has no perturbation to hold the trajectory off the circuit's equilibrium/latch, so the deterministic solve parks there — the standard oscillator-startup problem (SPICE needs an IC/pulse/noise too). Verified NON-dissipative: µV-level `--noise thermal` sustains a full limit cycle indefinitely, and tiny noise could not overcome real numerical dissipation, so the limit cycle is genuinely self-sustaining. Measured on `farfisa-g10-ref.cir` term_1d @192k `--force-trap`: zero-input 3.95→**0.0** Vpp by 40 ms; `--noise thermal` 4.56→**4.31**; `--amplitude 1e-6` keep-alive 4.25→**4.43**. Deterministic (md5-identical across runs) | Not a bug — supply a perturbation, as real oscillators get from thermal noise. Either compile `--noise thermal` (physically authentic, but RNG-seeded → non-deterministic reference) or drive a ≥1e-9 keep-alive input (deterministic — preferred for a bit-stable calibration reference). **Distinct root cause from** the astable stiff-switching NR-overshoot frontier (`g10_divider_hard_switching_overshoot`, a real deferred bug) — do not conflate; the noise test rules out shared dissipation. Verified 2026-08-14 (openfarf g10-ref freeze report + melange-circuits latch diagnosis) |
| Pentode deck validates cleanly with `--tube-grid-fa off` but fails under `auto`/`on` with a pure gain error (corr ≈ 1.0, RMS +2.2% noyce-ef86, +3.0% noyce-6bq5, +12.3% el84-single-stage) that is already present at 1 mV drive | Grid-off reduction freezes `Vg2k = V[screen] − V[cathode]` at its DC value. The value is cathode-referenced: an unbypassed cathode resistor (all three decks) or an unbypassed screen stop (el84) makes Vg2k move with signal, and freezing it discards the local negative feedback through `dIp/dVg2k`. Linearized prediction `(1+Rk'(gm+gs)+gs·|Zs|·r)/(1+Rk'·gm)` reproduces all three to four digits; el84 is 4× the others because its plate sits in the knee (`dIs/dIp = 1.09`) with a bare 1 kΩ screen. Confirmed not a codegen bug: a scratch copy with a 10 mF screen bypass (screen node swing 0 V) still shows the error. NOT grid conduction — Vgk never comes within 7 V of zero on any of the four decks at 0.1 V drive | `--tube-grid-fa auto` no longer reduces (full 3D, == `off`); `on` is a warned opt-in. `diag_region_exit_count` added on both emitters (pentode `Vgk > 0`, BJT `vbc_eff > 0`, counted on full models too). Regressions: `region_exit_diag_tests.rs`. See DEVICE_MODELS.md "Grid-Off Reduction" (FIXED 2026-09-04) |
| twill-deluxe (2×6V6GT push-pull + coupled-inductor OT) validates at 0.063% under `--tube-grid-fa off` but 22% with a waveform change under `auto`/`on`; `simulate --solver dk` shows the DK plate mean ABOVE B+ (323 V vs 320 V) and a −0.23 V DC offset on a capacitively coupled grid | **Not the reduction.** Forced `--solver dk`, the 3D (M=10) and grid-off (M=8) models agree to 0.003%. The reduction lowers M from 10 to 8, which flips the auto-router from nodal ("large nonlinear dimension") to DK Schur, and the DK route is wrong on this deck (nodal: peak 6.43 V, plate mean 312.5 V = B+ − Ip·DCR; DK: peak 7.07 V, non-physical DC). The reduction TRIGGERS the DK route; it does not cause the defect | **DEFERRED — separate ticket, NOT repaired.** The safety fix (full-3D default + route-parity pre-check in `apply_grid_off_reduction`) dodges it by keeping twill on nodal; the DK defect on coupled-inductor circuits is untouched. Population is CORPUS-WIDE, not pentode- or openwurli-bounded: the class is "coupled inductors on a DK route". Census 2026-09-04 (pre-change binary, 14 corpus decks with `K` cards): 4 routed DK — axe-15, kt88-pp-stage, twill-deluxe (all three only because grid-off pulled M under 10) and noyce-amp-at-idle (M=6, DK on its own merits). After the fix: 1 (noyce-amp-at-idle, M=7). openwurli has no coupled inductors (safety answer, not the scope). Reproduce: `simulate twill-deluxe.cir --solver dk --tube-grid-fa off --probe pa_1 --probe el1_g` vs `--solver nodal` |
| Full-GP BJT with parasitic RB/RC/RE (internal-node expansion active) on a nodal **trapezoidal** build: thermal-noise variance at the output is **63× the same stage built with an explicit base resistor**, and a capacitor-less collector row carries a drive-independent ~50 mV `(-1)^n` ring that starts one sample after the DC OP (one impulse away on program material); static gain at every operating point is unchanged | The augmented-row re-zero in `build_discretized_matrix` — which blanks `A_neg` history on the algebraic VS / inductor / VCA / behavioral rows — also swept up the parasitic-BJT internal nodes that `expand_bjt_internal_nodes` appends into `[n_nodes, n_aug)`. Those are **physical** nodes with real G/C stamps and must keep their trapezoidal history (`A_neg = αC − G`); zeroing them makes the DC OP not a trap fixed point, so the first trap step kicks the capless collector row into a `z = −1` limit cycle. **The spec was already right and one implementation was not**: the `MnaSystem::n_aug` doc comment (`mna.rs:49`) already specified the exclusion ("excluded from the A_neg zeroing, see `build_discretized_matrix`") and the DK kernel builder (`dk.rs:787`) already did it; the nodal `from_mna` inline loop and the shared `zero_augmented_history_rows` helper (plus its DK-path callers) did not | Give the shared helper an `is_bjt_internal` mask mirroring `dk.rs` and route every nodal/DK zeroing through it (`mna.rs:2109`, `codegen/ir/mod.rs:1520` + all callers). Under forced trap the expanded parasitic-RB noise test matches the explicit resistor (ratio ~1.0, was 63×); golden corpus byte-identical (no corpus deck is an expanded parasitic-BJT trap build). Found via `bjt_parasitic_rb_thermal_matches_explicit_base_resistor_nodal` (design review). FIXED 2026-09-13, shipped v0.1.8 `b6c08a7` (pre-squash `a10f3d4`) |
| Runtime BE-latch engages on a benign transient (one impulse, a program stop) and holds backward Euler for the rest of the stream; harmonic accuracy degrades from that point on (H3 up to −15 % at 1× on a saturating deck), with no ring visible before the latch | The entry threshold on the output's lag-1 ratio was a tuned −0.6. An impulse through an output high-pass leaves a 2-sample anti-correlated tail with the input silent, which the input gate does not explain. (gold-press-mastering, 46.86 s of a 10 Hz impulse train; with the latch disabled the condition held for 2 samples, never again) | Entry at ratio ≤ −exp(−α), the estimator's own forgetting rate: the alternating mode must outlive the window and carry the output. A floor at the solver's node tolerance, since a rest alternation below it is convergence noise. `be_latch_entry_tests.rs`. FIXED 2026-09-28; the sticky release is still open (STATUS) |
| Runtime BE-latch never fires (`diag_be_latch_count` = 0) on a build that is demonstrably in a self-sustained fs/2 `(-1)^n` cycle: the safety net is **silently inert on any DC-biased output node** (e.g. a collector sitting at several volts), and had been since it shipped 2026-07-28 | The detector correlated the RAW `v[OUTPUT_NODES[0]]`. On a biased node the lag-1 products are bias-dominated (r1 ≈ +1), so a mV-scale `(-1)^n` ring riding on the bias never crosses the anti-correlation threshold (`BE_LATCH_R1_ENTER`) — the detector cannot see AC structure it never separated from the DC. Found via the parasitic-RB trap-ring investigation (design review) | Track the output DC with an EMA (`state.be_x_mean`, same coefficient as the correlator) and correlate the AC residual — `nodal_emitter.rs::emit_be_latch_detector`. Golden-safe: both `runtime_be_latch` corpus decks (passive-eq1a, 4kbuscomp) stay byte-identical, i.e. the latch stays inert on real decks and the mean-removal does not introduce a false fire. FIXED 2026-09-13, shipped v0.1.8 `b6c08a7` (pre-squash `9404e04`) |
| A deck that oscillates by design is auto-promoted to backward Euler and runs its limit cycle at **2.8× amplitude and 6% wrong frequency** vs trapezoidal (philicorda-master; the promotion log offers no warning that the swap is not behaviour-preserving) | `stability::trap_needs_be` clause 1 (ρ > 1.002) was sign-blind. A POSITIVE `dominant_sign` is a real growing pole — a regenerative oscillator/latch on an unstable DC bias, by design — and BE over-damps that physical limit cycle without stabilising it. **False start worth recording:** the obvious gate (clause 1 only when `dominant_sign < 0`) is too coarse and regressed `bjt_parasitic_rb_thermal_matches_explicit_base_resistor_nodal` — expanded common-emitter stages also read `dominant_sign` +1 (ρ ≈ 1.06) without being oscillators (trap holds a marginal +1 mode whose impulse never returns to the OP, which BE removes), and promoting them was load-bearing for noise fidelity: on trap that stage reads 63× an explicit resistor (see the parasitic-BJT `A_neg` row above) | Revised policy (design review): on a positive dominant sign, promote only if BE **actually stabilises the mode** — `rho_be = ρ(S_be·A_neg_be) <= BE_POST_PROMOTION_LIMIT`, the same quantity `log_be_post_promotion_check` reports, measured on the already-built BE matrices (`codegen/ir/mod.rs:3216`). `rho_be <=` limit → promote (a numerical marginal mode BE fixes: the CE class); `rho_be >` limit → keep trapezoidal and log INFO (a real growing pole BE cannot fix: the oscillator/master class). `dominant_sign < 0` (Nyquist) and clause 2 unchanged; `trap_needs_be` itself unchanged. The same pass rewrote the post-promotion warning text, which claimed BE "does not meaningfully change this circuit's transient behavior … within ~1%" (false — 2.8× / 6% measured on a high-Q tank) on the stale premise that BE remains the shipped default for that class; it now states that BE does not stabilise a real growing pole (`rho_be` still > 1), that trapezoidal is the physical integrator, and that the warning is only reachable when BE was FORCED (`--backward-euler` / `.integrator be`) — `stability.rs::log_be_post_promotion_check`. Validated: noise test green, philicorda-master stays trapezoidal, golden corpus byte-identical across all 38 decks (every existing promotion is `-1`, untouched). FIXED 2026-09-13, shipped v0.1.8 `b6c08a7` (pre-squash `7774e99`; warning text `7f8518b`) |
| `melange dc-op` reports `converged: true` on a deck with ≥2 same-direction **parallel** junctions (paralleled diodes, Darlington, germanium cluster) at a solution violating KCL by **thousands of amperes**; DirectNR "converges" in ~3 iterations to a deep-reverse point and the node vector disagrees with ngspice `.op` | Two coupled defects. (1) The convergence test was step-only (`\|delta\| < reltol·\|v\| + tolerance`): once diode voltage limiting pinned the parallel junctions into a limiter fixed point the step collapsed to ~0 — a self-consistency test built from the ITERATE cannot see an equation that is wrong at its own fixed point; the missing test is one built from the EQUATION. (2) The pnjlim back-projection was applied per junction and SUMMED, so K junctions on one `N_v` row landed at `Σ v_lim − (K−1)·v_raw` (`5159b8c`'s single-row doubling re-emerging through a second parallel junction) — a direction-REVERSING step whenever `v_raw > 2·v_lim`, which is exactly how the deep-reverse false root is reached in one iteration; merely overlapping rows (Vbe/Vbc share the base, a differential pair shares the emitter) get half-amplitude cross-talk instead. Reported by melange-circuits; resolved by design ruling | Two lanes. **Lane 1** — acceptance also requires a per-voltage-row KCL residual gate `\|F_i\| <= reltol·scale_i + DC_OP_KCL_ABSTOL_AMPS` (1e-9 **amperes**, deliberately named apart from `DcOpConfig::tolerance`'s 1e-9 **volts** step floor — same digits, different quantity), NaN-safe (`!(x <= bound)` so a NaN residual fails), `F` evaluated at the accepted post-damping/post-clamp `v`; op-amp rows are exempt ONLY while pinned by the post-NR rail clamp on that iteration and never under BoyleDiodes, and an exempted row still reports its residual. On iteration exhaustion the solver returns `converged: false` with `kcl_residual_max` / worst row, surfaced in `melange dc-op` (human + `--format json`, since sidecars gate on it), and the strategy ladder falls through to source stepping, which reaches the true root. Folded in: `solution_has_active_junction`'s diode branch used `\|v_nl\| > 0.5·vcrit` and read a deep-reverse diode as "active" — now uses the forward direction like the BJT branch. **Lane 2** — `apply_junction_corrections` collects every limited `(row, correction)` pair, dedupes rows identical up to sign (keeping the most restrictive) and solves the joint minimum-norm node update `delta = N_L^T (N_L N_L^T)^-1 c`: full-rank Gram solved EXACTLY (each junction lands precisely on its `v_lim`), with ridge `JUNCTION_GRAM_RIDGE_REL` (1e-9 × max Gram diagonal) applied ONLY when the Gram-Schmidt rank test `JUNCTION_ROW_DEPENDENCE_TOL` finds dependent rows; a single limited row short-circuits to the unchanged `distribute_junction_correction` (bit-identical to before). Corpus-neutral: 42/42 corpus + openwurli `dc-op --format json` node vectors byte-identical, only two iteration counts moved (noyce-germanium-cluster 8→9, wurli-tremolo 122→125, same root); every parallel-junction repro now converges by DirectNR instead of source stepping, within 1.5 mV of ngspice (uniform ~0.8 mV = VT at 300 K vs 300.15 K). Mechanism detail in DC_OP.md "Limiter back-projection: joint minimum-norm" and "Convergence: step test AND KCL residual gate". **KNOWN GAP (still open as of 0.1.9)**: the emitted runtime `recompute_dc_op` is step-only and can still accept the false root until the gate is mirrored there (commented in place, `dc_op_emitter.rs:638`). FIXED 2026-09-13, shipped v0.1.8 `b6c08a7` (pre-squash `7e50812` lane 1, `d6f4f08` lane 2) |
| Nodal-Schur **subsample-fire** (glow) build: `diag_subsample_fire_schur_builds` rises by at least one on EVERY host sample even with pots/switches untouched — an O(N³) Schur triple rebuilt identical to the previous sample's — while `diag_subsample_fire_schur_reuses` only ever counts within-sample hits | The subsample-fire Schur memo (`ssf_sub` triple + key/valid) was declared as per-sample block-locals, so the first lit build of each host sample always missed even though `g_work`/`c_work` were unchanged since the previous sample. Within a sample G/C are fixed; across samples they move only via `rebuild_matrices` (pot/switch/runtime-R/`set_sample_rate`), the saturating-inductor in-place SM patch, and `reset()` | Move the cache onto `CircuitState`, keyed on `(rate.to_bits(), be)`, dropped wholesale by an explicit validity flag (mirroring `chord_valid`) at exactly those three sites — no per-lookup matrix compare. Landed in two steps: single slot (pre-squash `de0514b`, 1.06× on philicorda-note-board over the already-shipped per-sample memo `0507242`, ~83.4% hit), then a fixed 8-entry cross-sample cache with a round-robin eviction cursor so the several distinct within-sample rates (pre-flip / rest / lit) coexist instead of only the last one (pre-squash `5b2d2d9`: 90.6% hit, builds −43.7%, 1.28× over the per-sample memo). Misses build zero-copy into the target slot; a singular sub-step never commits a matchable entry; a `#[cfg(debug_assertions)]` shadow rebuild bit-compares every hit. Size 8 not 16: same 1.28× and 90% hit at half the footprint (~217 KB/solver; footprint scales `N²·SIZE`) and half the ~1.3% regression on k=1 decks that gain nothing. Bit-identity is the hard gate and holds (576k-sample cross-binary lockstep vs the pre-cache baseline byte-identical, shadow assertion clean); gated behind `subsample_fire`, so non-ssf / DK / full-LU decks emit byte-identical code. NB openphilicorda's reported 2.38× was a NO-memo comparison — the per-sample memo alone is already ~3.5× over no-memo; and the v0.1.8 banner/CHANGELOG figure of ~2.49× is ALSO real and is not in conflict: it was measured by openphilicorda (2026-09-14) on their `philicorda-divider-off192-563-6stage.cir` at 192 kHz / 4× oversampling, over v0.1.7's per-sample memo, with a 384 000-sample cross-binary lockstep confirming bit-identical `v_prev`. The three numbers differ by DECK k-structure, not by baseline: the note-board is k=32, where the per-sample memo already coalesces 31 of 32 same-rate lit sub-steps and leaves the cross-sample cache almost nothing to catch; the 563 six-stage deck is k=3 with three distinct-low-bit segment rates, where the per-sample memo catches far less. Quote whichever matches the deck shape in front of you, and say which deck — do not average them or treat one as superseding another. **CAVEATS**: the saturating-inductor and pot/switch invalidation hooks are code-verified and correctly gated but UNEXERCISED at runtime — no available ssf deck has saturating-L, pots or switches (note-board's coupled inductors are linear, const G/C) — and `philicorda-divider.cir` has its NEON lines commented out, emits no ssf code, and is therefore not a usable ssf test. SHIPPED 2026-09-13/14 in v0.1.8 `b6c08a7` (pre-squash `de0514b`, `5b2d2d9`) |
| DC-biased saturating inductor: `i_L` drifts one way against an independent recurrence over the L/R time constant (0.2 % on H1 at 2 s with L/R ≈ 1 s; 0.7 % with L/R = 100 s), while the same deck without bias matches to ~1e-8. No sub-steps, no unsolved samples, one NR iteration per sample | The flux-row residual accepted `1e-3·den` with `den` the per-sample increment `alpha·ΔΦ`, so Newton's FIRST iterate always passed. Its remainder `alpha·Φ''·Δi²/2` is one-signed when a bias fixes the sign of `Φ''`, and the flux integrates it. Without bias the sign alternates with `i` and cancels, which is why unbiased tests never saw it | Flux-row residual tolerance `max(1e-5·den, 64·eps·max(\|alpha·Φ\|, \|rhs[k]\|))`: about one more iteration per sample, drift ~1e-7 relative. The floor sits on the magnitudes that cancel in `alpha·Φ − rhs[k]`, not on `den`; a fixed absolute floor would reopen the drift at small increments. Pinned by `saturation_knee_regression_tests.rs` C3 (1×/256× recurrence + exact flux-drive table). The same check runs at every Newton site that can commit a sample: the main loop, the adaptive sub-step, the BE fallback and the active-set pinned Newton. (FIXED 2026-09-28) |
| Every other sample alternates by a large, constant amount at a node with no capacitor, forever, after test code set part of the state by hand (e.g. `input_prev` without the node voltages it implies); node damping then clips each step and NR takes 10+ iterations per sample | Trapezoidal integration enforces an algebraic row (a node with no capacitor) only as the AVERAGE of this sample and the last, `(v + v_prev)`, so an initial state that violates the row pointwise persists as an undamped `(-1)^n` component. melange's own state always starts consistent (DC OP); only hand-poked state hits this | Set every quantity the poked one implies (e.g. the input node's voltage along with `input_prev`), or start from `reset()`/the DC OP. Not a solver bug. (NOTED 2026-09-28.) This is the whole-system form's `z = −1` walk; under the charge form a capless row carries no history and `input_prev` is not in the RHS — poke `v_prev` and `q_dot` together, see the walk row below |
| Saturating inductor: current peak overshoots the physical ceiling V/R in deep saturation (~20 % at 20× Isat) while H1 and the spectrum look right; oversampling does not cure it (4× is worse at 20 V) | The trapezoidal rule is A-stable but not L-stable. As L_diff → 0 the RL step factor tends to −1, so the stiff mode rings sample to sample instead of decaying | The runtime BE-latch catches the alternation and switches that instance to backward Euler; the peak lands on the ceiling (`c1_deep_saturation_ring_is_caught_by_the_latch`). The latch is sticky, so the rest of the stream pays BE's first-order error: H1 −1.8e-4 at 10 V on the saturating RL, output −0.28 % on a choke-loaded stage at 5 V. Pin `.integrator be` to take that cost deliberately; `.integrator trap` / `--force-trap` keeps the ring. (CLOSED ON THE LATCH 2026-09-28) |
| Op-amp stage rings at exactly fs/2 from the first sample at ZERO input (±0.48 V on a single-supply stage), or the runtime BE-latch fires inside the 50-sample default warmup; `dc-op` reports a tiny KCL residual | The DC solver capped op-amp gain at AOL = 1000 (a homotopy aid for precision rectifiers) and returned the capped solution: every virtual ground ~0.1 % off, and the residual was measured against the capped system, so it looked clean. The transient runs the full AOL, starts away from its own equilibrium, and the trapezoidal rule holds the op-amp's fast row only on average | The ladder still solves capped, then `solve_dc_operating_point` finishes at the full AOL (`uncap_opamp_aol`) and reports the residual against the full system; a finish that fails warns. Tests: `dc_operating_point_uses_the_full_open_loop_gain`, `railing_choke_stage_starts_at_its_own_equilibrium`. (FIXED 2026-09-28) |
| On a nonlinear trapezoidal build, the KCL residual of a capless row — or of a combination whose capacitor currents cancel, e.g. `KCL(a) + KCL(b)` across a coupling cap — alternates in sign every sample, `r_n + r_(n−1) ≈ 0`, with `|r|` far above the solver floor (cap-coupled diode witness: 1.31–1.52 µA at 96 kHz, 0.43–0.52 µA at 192 kHz, vs a ≤ 0.011 µA floor); a railing op-amp into a diode clipper shows 105–2650 µA on the clipper row without transition-BE. Invisible at a filtered output. Companion symptoms: stationary Nyquist limit cycles at rest (±3.9 µV, ±75 nV), a persistent 2.8 mV Nyquist alternation after a DK pot sweep, a saturating-choke stage that never reaches periodic steady state (1.2–1.4 mV drift per 0.1 s); at 48 kHz vs a 768 kHz reference the railing deck's op-amp output was 0.49 V RMS off | Deck class: nonlinear circuits whose accepted solves leave a residual (NR tolerance, iteration cap, held samples, chord steps, active-set pins) on capless rows or cap-coupled node pairs — diode clippers behind coupling caps, railing op-amps, saturating chokes, DK pot sweeps. Cause: the whole-system trapezoidal form (`A_neg = αC − G`, sources as `b(n)+b(n+1)`, `N_i·i_nl_prev` in the RHS) enforces only `KCL(n) + KCL(n+1)`; projected on a left null vector of `C` it is `e(n+1) = ρ(n+1) − e(n)`, a `z = −1` memory that carries every accepted residual forward undamped | **Charge (companion) form** (`COMPANION_MODELS.md`): history `αC·v_prev + q_dot`, every source once at `n+1`, KCL of a committed sample = its own solve residual. Charge form on the same decks: ≤ 0.011 / ≤ 0.005 µA (witness), 0.29–0.43 µA (railing deck without transition-BE, the floor), op-amp output 1.1 mV RMS vs 768 kHz. Golden corpus (188 renders): 107 bit-identical (BE builds), linear decks within 1e-13, 7 changed — every change traced to the walk (Nyquist limit cycles at rest now decay to ~1e-17, pot-sweep alternation gone, choke stage settles to 3e-9 V); 0 marginal Newton renders (a preamp deck had 3). See "The Whole-System `z = −1` Walk" above. (FIXED 2026-09-29) |
| A `.linearize`d BJT stage's gain disagrees with the same deck unlinearized and with ngspice: +0.71 dB on a bypassed common-emitter stage with a Gummel-Poon card (VAF = 50); 16.7 dB low on an NR = 2 stage with its B-C junction forward | The linearized stamp was rebuilt from bare Ebers-Moll formulas instead of the device model: the B-C junction at Vt instead of NR·Vt, no Gummel-Poon qb (no Early output resistance, no high injection), no ISE/ISC leakage, no RB/RC/RE, and B-C conductances stamped symmetrically into the base and collector rows, which the device does not draw | Stamp the device's own 2×2 terminal-current Jacobian from the evaluator the bias solve uses (`dc_op::bjt_eval`, external terminal pairs so the parasitics fold in). Witness: `linearize_bjt_small_signal_tests.rs`, linearized slope vs the central difference of the nonlinear DC OP, three cards (FIXED 2026-09-29) |
| A PNP stage with CJC/CJE rolls off far too early: a PNP common-emitter stage reads 10 dB low at 10 kHz and 18 dB low at 100 kHz while its NPN mirror, and ngspice for both, agree; the NPN is right | The DC-OP cap re-linearization fed the terminal differences V(b) − V(e), V(b) − V(c) into the depletion formula, which takes forward voltage as positive. For a PNP both are sign-flipped: the reverse-biased B-C junction was evaluated as forward biased on the FC·VJ tangent extension (C_bc 92.6 pF where ngspice's cmu is 11.35 pF, 8.2×), the forward B-E junction as reverse biased | `linearized_junction_caps` applies the polarity (`is_pnp`) to the voltages it is given. Witness: `bjt_junction_cap_polarity_tests.rs`, the PNP stage's C matrix equals its NPN mirror's and C_bc equals ngspice's cmu (FIXED 2026-09-29) |
| A `.linearize`d BJT stage has no high-frequency rolloff: a CJC = 100 pF common-emitter stage from a 10 kΩ source reads flat (+12.3 dB at 20 kHz) where the unlinearized deck and ngspice roll off through the Miller pole | The `.linearize` rebuild re-stamps junction caps only for the devices left in the nonlinear system, so a linearized BJT's CJE/CJC/TF were never stamped | `stamp_linearized_bjts` stamps the caps `linearized_junction_caps` gives at the bias point, between the external terminals. Witness: `linearize_bjt_small_signal_tests.rs`, the linearized circuit's C matrix equals the full circuit's after its DC-OP cap re-linearization, NPN and PNP (FIXED 2026-09-29) |
| A BJT's B-E capacitance at the DC OP disagrees with ngspice's `capbe` when NF ≠ 1 (1.5× high at NF = 1.5) or when Gummel-Poon qb is live (20 % high with IKF = 5 mA at 1.4 mA); a forward-active (1D, `--bjt-fa`) BJT's B-C cap stays at CJC whatever its bias (1.77× at Vbc = −3.5 V) | The diffusion cap was `TF·\|Ic\|/Vt`, the NF = 1, qb = 1 special case of `TF·d(I_F/qb)/dVbe`; the cap re-linearization read Vbc from the 1D slot, which carries only Vbe, as 0 | Diffusion from the device model (`forward_diffusion_capacitance`, ngspice `capbe = tf*gbe`); the 1D slot's Vbc from the node voltages. Witness: `bjt_charge_storage_tests.rs` against ngspice `.op` (FIXED 2026-09-29) |
| A JFET gate driven positive sits at the drive voltage (+2 V through 100 kΩ: 2.000 V, ngspice 0.546 V); a JFET resistor driven so its drain swings below the gate shows no gate-drain conduction | The JFET had no gate junctions: the generated `jfet_ig` returned 0 and the DC OP matched it | Gate-source and gate-drain junctions (`IS`, `N`; `Jfet::evaluate` / generated `jfet_evaluate`, shared `junction_exp`), ngspice-order pnjlim+fetlim limiting. Witness: `jfet_gate_junction_tests.rs` against ngspice with GMIN off (FIXED 2026-09-29) |
| A `.linearize`d triode stays nonlinear with only a warning, or a `.linearize`d BJT fails every sample of every render | The triode was skipped (kept nonlinear) when its grid was within 0.5 V of onset at DC, overriding the directive; the BJT was linearized although saturated at its own operating point (Vbc0 > 0) | Refused at compile time with the operating-point evidence (`PipelineError::Linearize`), as is a name that is not a BJT or triode. Witness: `linearize_refusal_tests.rs` (FIXED 2026-09-30) |
| A `.linearize`d triode stage has no Miller rolloff: CGP = 100 pF behind 100 kΩ reads flat (+34.5 dB) to 20 kHz where the unlinearized deck is at −3.5 dB | The cap re-stamp after the linearize rebuild covers only devices left in the nonlinear system, so a linearized triode's CCG/CGP/CCP were dropped (the BJT twin of this was fixed in 715a515) | `stamp_linearized_triodes` stamps the three inter-electrode caps (`LinearizedTriodeInfo::{ccg,cgp,ccp}`). Witness: `linearize_triode_caps_tests.rs`, the linearized C matrix equals the full one (FIXED 2026-09-30) |
| A triode with `RGI` and a conducting grid drifts away from its DC operating point at silence (grid +3 V through 22k, RGI = 2k: the grid moves 76 mV, the plate 0.70 V) | The DC OP evaluated the triode at its terminal grid; the transient evaluates it at the internal grid behind RGI | The DC OP solves the same internal-grid equation (`KorenTriode::evaluate_with_rgi`). Witness: `triode_rgi_dc_op_tests.rs`, an RGI card against an explicit series resistor, and the DK and nodal transients at rest (FIXED 2026-09-30) |
| A triode with `RGI` whose grid terminal is driven more than ~9.5 V positive (RGI = 2k) passes the wrong grid and plate current, silently: 17 V short of the internal-grid root at 35 V | The generated inner grid solve took at most 8 Newton steps of at most 1 V each | No step limit (the equation is convex and increasing, so Newton from the terminal voltage cannot overshoot) and a 50-step ceiling; at most 10 steps measured over -100..+300 V, RGI 100 Ω..1 MΩ. Witness: `template_triode_rgi_matches_devices_crate` (FIXED 2026-09-30) |
| A high-impedance node behind a reverse diode sits low at the DC OP: a reverse diode fed from 10 V through 1 GΩ reads 9.980 V where the diode law gives 9.99999 V | The DC OP added its 1e-12 S diode GMIN to the current as well as the Jacobian (the transient has none), so the two solved different diode laws | GMIN in the DC OP Jacobian only (Newton conditioning); the node then reads 9.990 V, the remainder being node Gmin (STATUS "Node Gmin moves the fixed point"). Witness: `diode_dc_op_fixed_point_tests.rs` against ngspice with GMIN off (FIXED 2026-09-29) |

## References
- TU Delft Analog Electronics Webbook: MNA stamps
- Hack Audio Tutorial: DK method and NR solver
- Pillage & Rohrer: Companion models
