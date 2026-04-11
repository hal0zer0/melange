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
- Unconditional residual check in full-LU NR (currently BoyleDiodes-only)
- Iterative refinement in the chord back-solve (one extra O(N²) pass)
- Schur-complement-within-full-LU hybrid (M-dim correction inside N-dim NR)

## References
- TU Delft Analog Electronics Webbook: MNA stamps
- Hack Audio Tutorial: DK method and NR solver
- Pillage & Rohrer: Companion models
