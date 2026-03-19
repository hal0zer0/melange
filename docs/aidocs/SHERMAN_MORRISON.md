# Sherman-Morrison Rank-1 Updates (Dynamic Potentiometers)

## Purpose

Enable real-time resistance changes without O(N^3) matrix re-inversion. A potentiometer
change is a rank-1 perturbation to the conductance matrix G, which propagates to S, K,
A_neg, and S*N_i via the Sherman-Morrison formula at O(N^2) cost.

## Source Files

| Component | File | Lines |
|-----------|------|-------|
| `SmPotData` struct | `melange-solver/src/dk.rs` | 89-116 |
| Precomputation | `melange-solver/src/dk.rs` | 294-340 |
| `PotInfo` struct | `melange-solver/src/mna.rs` | 265-282 |
| `PotDirective` struct | `melange-solver/src/parser.rs` | 33-48 |
| `PotentiometerIR` struct | `melange-solver/src/codegen/ir.rs` | 127-150 |
| SM scale helper (codegen) | `melange-solver/src/codegen/rust_emitter.rs` | 1320-1342 |
| Sequential SM corrections | `melange-solver/src/codegen/rust_emitter.rs` | 1984-2094 |
| State template | `melange-solver/templates/rust/state.rs.tera` | 101-134 |
| Sample rate rebuild | `melange-solver/templates/rust/state.rs.tera` | 518-555 |

## Netlist Syntax

```spice
R1 node_a node_b 10k
.pot R1 1k 100k           ; marks R1 as variable, range [1k, 100k]
.pot R2 500 500k Volume   ; optional label for plugin UI
```

Max 32 pots per circuit.

## Mathematical Foundation

### Sherman-Morrison Formula

Given invertible matrix A with inverse S = A^{-1}, and a rank-1 update
A' = A + delta_g * u * u^T:

```
S' = (A + delta_g * u * u^T)^{-1}
   = S - (delta_g / (1 + delta_g * u^T * S * u)) * (S*u) * (S*u)^T
   = S - scale * su * su^T
```

Where:
```
u       = node difference vector: u[p] = +1, u[q] = -1 (0 elsewhere)
su      = S * u                    (N-vector, precomputed)
usu     = u^T * S * u             (scalar, precomputed)
delta_g = 1/R_new - 1/R_nom       (conductance change)
scale   = delta_g / (1 + delta_g * usu)
```

### Why It Works

Changing a resistor from R_nom to R_new modifies the MNA G matrix by:
```
G' = G + delta_g * (e_p - e_q) * (e_p - e_q)^T
```

where e_p, e_q are standard basis vectors. Since A = G + alpha*C and the
capacitance matrix C is unchanged, A' = A + delta_g * u * u^T.

## Precomputed Vectors

Computed once during `DkKernel::from_mna()` and stored in `SmPotData`:

```
u[i] = 0 for all i
if node_p > 0: u[node_p - 1] = 1.0
if node_q > 0: u[node_q - 1] = -1.0

su    = S * u           (N-vector)
usu   = u^T * su        (scalar)
nv_su = N_v * su         (M-vector, for K correction)
u_ni  = su^T * N_i       (M-vector, for S*N_i correction)
```

## Corrections Applied in process_sample

### 1. Scale Factor Computation

```rust
fn sm_scale(state: &CircuitState) -> (f64, f64) {
    let r = state.pot_resistance.clamp(MIN_R, MAX_R);
    let delta_g = 1.0 / r - G_NOM;
    let denom = 1.0 + delta_g * usu;
    let scale = if |denom| > 1e-15 { delta_g / denom } else { 0.0 };
    (delta_g, scale)
}
```

### 2. A_neg Correction (RHS)

The conductance change affects the RHS via the A_neg * v_prev term:
```
rhs[node_p - 1] -= delta_g * v_prev[node_p - 1]
rhs[node_q - 1] -= delta_g * v_prev[node_q - 1]
```

Note: uses `delta_g` directly (not `scale`) because this corrects the A_neg matrix
contribution, not the S matrix.

### 3. S Matrix Correction (Linear Prediction)

After computing v_pred = S * rhs:
```
su_dot_rhs = sum_k su[k] * rhs[k]
factor = scale * su_dot_rhs
v_pred[k] -= factor * su[k]    for all k
```

This applies: v_pred' = (S - scale * su * su^T) * rhs = S' * rhs

### 4. K Matrix Correction (NR Jacobian)

For nonlinear circuits (M > 0), the K matrix used in NR must be corrected:
```
K_eff[i][j] = K[i][j] - scale * nv_su[i] * u_ni[j]
```

This is: K' = N_v * S' * N_i = K - scale * (N_v * su) * (su^T * N_i)

### 5. S*N_i Correction (Final Voltages)

After NR solve, final voltages include S*N_i*i_nl correction:
```
u_ni_dot_inl = sum_j u_ni[j] * i_nl[j]
sni_factor = scale * u_ni_dot_inl
v[k] -= sni_factor * su[k]    for all k
```

## Multi-Pot Sequential Updates

For circuits with multiple pots, corrections are applied **sequentially** with
cross-correction to prevent compound errors:

```
For pot k = 0..num_pots:
  if k == 0:
    su_c = su_0                     (no prior corrections)
  else:
    su_c = su_k
    for j < k:
      dot = su_c_j^T * u_k          (cross-coupling)
      su_c -= scale_c_j * su_c_j * dot

  usu_c = u_k^T * su_c
  scale_c = delta_g_k / (1 + delta_g_k * usu_c)

  Similarly cross-correct nv_su and u_ni vectors.
```

Each pot's SU vector is corrected for the cumulative effect of all prior pots.

## Sample Rate Rebuild

When `set_sample_rate()` is called, a new S matrix is computed from G+C at the
new rate. All SM vectors must be recomputed from scratch:

```
su    = new_S * u
usu   = u^T * su
nv_su = N_v * su
u_ni  = su^T * N_i
```

This happens in the generated `set_sample_rate()` function.

## Generated Constants

Per pot `idx`:
```rust
const POT_{idx}_SU: [f64; N] = [...];
const POT_{idx}_USU: f64 = ...;
const POT_{idx}_NV_SU: [f64; M] = [...];
const POT_{idx}_U_NI: [f64; M] = [...];
const POT_{idx}_G_NOM: f64 = ...;
const POT_{idx}_MIN_R: f64 = ...;
const POT_{idx}_MAX_R: f64 = ...;
const POT_{idx}_NODE_P: usize = ...;
const POT_{idx}_NODE_Q: usize = ...;
```

## State Fields

Per pot `idx` in `CircuitState`:
```rust
pot_{idx}_resistance: f64,       // Current resistance (set by user)
pot_{idx}_su: [f64; N],          // S * u (recomputed on sample rate change)
pot_{idx}_usu: f64,              // u^T * S * u
pot_{idx}_nv_su: [f64; M],       // N_v * su
pot_{idx}_u_ni: [f64; M],        // su^T * N_i
```

## Complexity

| Operation | Without SM | With SM |
|-----------|-----------|---------|
| Resistance change | O(N^3) re-invert | O(N^2) rank-1 update |
| Per-sample overhead | 0 | O(N) per pot (dot products) |
| Memory | 0 | O(N+M) per pot |

## Common Bugs

| Symptom | Cause | Fix |
|---------|-------|-----|
| No effect when turning pot | delta_g = 0 | Check R_nom matches netlist |
| Explosion at extreme R | Denominator near zero | Check clamp to [MIN_R, MAX_R] |
| Wrong tone after rate change | SM vectors stale | Verify set_sample_rate recomputes |
| Multi-pot interaction wrong | Missing cross-correction | Check sequential update loop |
