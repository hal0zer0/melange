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
whole-system `alpha*C - G` (used by the routing estimates — the DK/nodal
"trapezoidal unstable" trigger and the nodal Schur-vs-full-LU gate — and by
the deprecated library `LinearSolver`; backward-Euler promotion is the ring predicate,
`RING_PREDICATE.md`, not this operator). Mixing terms of the two forms double-counts a source.

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
| Heavy clipping (amp ≥ 0.05 V on the 4-op-amp overdrive deck) under `--opamp-rail-mode boyle-diodes`: raw output node `state.v_prev[OUTPUT_NODES[0]]` diverges to 45–3068 V, NR fails every sample. The generated `output[i].clamp(-10.0, 10.0)` safety rail masks this to a visible 10 V, so superficial inspection shows "output=10 V" while the actual solver state is ±3000 V. Always measure raw node voltage when debugging BoyleDiodes convergence. | Three fix candidates empirically tested 2026-04-08 fourth session with a disciplined amp sweep [0.01, 0.03, 0.05, 0.07, 0.10, 0.15, 0.20, 0.30, 0.50]: (a) targeted Gmin bump on `_oa_int_*` rows — destroys linear-regime op-amp gain; (b) force `need_refactor = true` in BoyleDiodes (ngspice-style refactor-every-iter) — preserves linear, doesn't fix heavy clip; (c) disable global `damp_thresh` step cap — preserves linear, doesn't fix heavy clip. None satisfied the confirmation criteria (zero NR failures at all amps, peak in [10.0, 11.0] V). The chord-LU Newton direction appears to be wrong at the catch-diode knee, not just the magnitude, so no form of step damping or single-row regularization fixes the underlying issue. Prior "bistable chord-LU fixed points" and "PTC doesn't work because of static pivots" diagnoses were both retracted; see `task_12_bistable_oscillation_finding.md` "FOURTH SESSION" for the full audit and sweep data. | **Not blocking.** `active-set-be` was verified in the same sweep: raw peak bounded 10.70–10.76 V at every tested amplitude, trap NR falls through to BE fallback at heavy clip and BE converges every time. `auto` resolves this deck to `active-set`, which was not re-run on it. BoyleDiodes mode remains opt-in (`--opamp-rail-mode boyle-diodes`) and is a known limitation at heavy clip; it works correctly for light clip (amp ≤ 0.03 V on that deck) and for control-path topologies. If heavy-clip BoyleDiodes convergence becomes a priority later, the next tier of escalation candidates are: Anderson acceleration (m=3, Walker-Ni + Zhang-Peng-Ouyang safeguards), trust-region Newton with actual-vs-predicted ratio, or a BoyleDiodes → ActiveSetBe failure-hybrid that tries BoyleDiodes trap and falls through to ActiveSetBe on divergence. |

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

**Example**: 8-BJT transistor ladder filter (moonladder). Bridging caps
between left/right columns at each stage. Nodes eL2-eR4 have only Gmin + cap.
S entries ~1e8, K entries ~5e11 at M=16 (pre-FA), ~1e4 at M=8 (post-FA), but
S remains extreme in both cases.

**FA detection threshold** (only when the reduction is asked for:
`--bjt-fa auto|force`; the default is `off`): Vbc < -0.5 V at the DC OP.
Common-base BJTs in cascade topologies sit at Vbc ≈ -0.85 V, which this
catches, while saturated BJTs (Vbc > -0.5 V) are excluded. A sample on which
a reduced BJT leaves forward-active is counted unsolved and refused
(`diag_reduced_model_exit_count`).

## Precision Rectifier DC OP Convergence

Circuits with precision rectifiers (op-amp + diode feedback, e.g., a VCA bus compressor's
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

## Precision Rectifier Transient NR — No Automatic AOL Cap

melange applies no automatic transient AOL cap to op-amps. Under the charge
form and active-set pinning the uncapped solve converges on precision
rectifiers, and a cap moves the fixed point (a biased half-wave rectifier sat
4.5 mV off ngspice with it, 0.2 µV without; DEVICE_MODELS.md "Transient AOL
Cap"). `.model OA(AOL_TRANSIENT_CAP=N)` remains an author's key and routes the
circuit nodal (`effective_aol_cap`, `codegen/ir/mod.rs`). The DC-OP
classifier `dc_opamp_is_sidechain_rectifier` (`dc_op.rs`) still seeds the DC
homotopy, which finishes at full AOL. The removed transient cap and the
back-substitution contamination it addressed are recorded in
[DEBUGGING_HISTORY.md](DEBUGGING_HISTORY.md).

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

The catalog of fixed and closed failure signatures (symptom, cause, fix
commit or date) lives in [DEBUGGING_HISTORY.md](DEBUGGING_HISTORY.md).
Check it before starting a new debugging session: many symptoms have been
seen before.

## References
- TU Delft Analog Electronics Webbook: MNA stamps
- Hack Audio Tutorial: DK method and NR solver
- Pillage & Rohrer: Companion models
