# Code Generation Reference

## Generated Structure

### Constants (Compile-Time)
```rust
const N: usize = 4;           // Number of nodes
const M: usize = 2;           // Nonlinear device dimensions
const SAMPLE_RATE: f64 = 48000.0;
const INPUT_RESISTANCE: f64 = 1.0;  // 1Ω near-ideal voltage source

// Precomputed matrices
const S: [[f64; N]; N] = [...];       // S = A^{-1}
const A_NEG: [[f64; N]; N] = [...];   // A_neg = alpha*C - G
const K: [[f64; M]; M] = [...];       // K = N_v*S*N_i (naturally negative)
const N_V: [[f64; N]; M] = [...];
const N_I: [[f64; N]; M] = [...];
```

### State (Runtime, Per-Channel)
```rust
struct CircuitState {
    v_prev: [f64; N],        // Previous node voltages
    i_nl_prev: [f64; M],     // Previous nonlinear currents
    dc_operating_point: [f64; N],
    last_nr_iterations: u32,
}
```

## Device Model Functions

The codegen generates device-specific functions based on the circuit:

### Diode (1D per device)
```rust
fn diode_current(v_d: f64) -> f64;       // IS * (exp(v/(N*VT)) - 1)
fn diode_conductance(v_d: f64) -> f64;   // (IS/(N*VT)) * exp(v/(N*VT))
```

### BJT (2D per device)
```rust
fn bjt_ic(vbe: f64, vbc: f64) -> f64;              // Ebers-Moll Ic
fn bjt_ib(vbe: f64, vbc: f64) -> f64;              // Ebers-Moll Ib
fn bjt_jacobian(vbe: f64, vbc: f64) -> [f64; 4];   // [dIc/dVbe, dIc/dVbc, dIb/dVbe, dIb/dVbc]
```

## Device Map and M-Dimension Assignment

Nonlinear devices occupy M dimensions in order of appearance in the netlist:
- Diode: 1 dimension (controlling voltage Vd, current Id)
- BJT: 2 dimensions (Vbe->Ic, Vbc->Ib)
- JFET: 1 dimension (Vgs->Id) -- not yet supported in NR codegen
- MOSFET: 1 dimension (Vgs->Id) -- not yet supported in NR codegen

Example for a circuit with D1, Q1, D2:
```
M index:  0      1       2       3
Device:   D1     Q1(Ic)  Q1(Ib)  D2
Dim:      1D     -----2D-----    1D
```

## Functions

### build_rhs
```rust
fn build_rhs(input: f64, input_prev: f64, state: &CircuitState) -> [f64; N] {
    // RHS_CONST (DC sources, if any)
    // + A_neg * v_prev  (includes capacitor history via alpha*C!)
    // + N_i^T * i_nl_prev
    // + (input + input_prev) / INPUT_RESISTANCE  (proper trapezoidal)
}
```
A_neg already contains alpha*C. Do NOT add separate cap_history.
The input uses proper trapezoidal integration: `(V_in(n+1) + V_in(n)) * G_in`.

### solve_nonlinear
```rust
fn solve_nonlinear(p: &[f64; M], state: &mut CircuitState) -> [f64; M] {
    // Newton-Raphson with:
    // - Warm start from i_nl_prev
    // - Step clamping (STEP_CLAMP = 0.01)
    // - Block-diagonal device Jacobian (see below)
    // - Singular check: |det| > 1e-15
    // - NaN/Inf protection
}
```

#### NR Iteration Body
```rust
// 1. Controlling voltages: v_d = p + K * i_nl
let v_d0 = p[0] + K[0][0] * i_nl[0] + K[0][1] * i_nl[1];
// ...

// 2. Device currents and Jacobian entries (device-type-specific)
// For diode at index d:
let i_dev_d = diode_current(v_d_d);
let jdev_d_d = diode_conductance(v_d_d);

// For BJT at indices (s, s+1):
let i_dev_s = bjt_ic(v_d_s, v_d_s1);
let i_dev_s1 = bjt_ib(v_d_s, v_d_s1);
let bjt_jac = bjt_jacobian(v_d_s, v_d_s1);
let jdev_s_s = bjt_jac[0];    // dIc/dVbe
let jdev_s_s1 = bjt_jac[1];   // dIc/dVbc
let jdev_s1_s = bjt_jac[2];   // dIb/dVbe
let jdev_s1_s1 = bjt_jac[3];  // dIb/dVbc

// 3. Residuals
let f_i = i_nl[i] - i_dev_i;

// 4. NR Jacobian (block-diagonal chain rule)
// J[i][j] = delta_ij - sum_k jdev_{ik} * K[k][j]
// where k ranges over the device block that owns row i
```

### process_sample
```rust
pub fn process_sample(input: f64, state: &mut CircuitState) -> f64 {
    let rhs = build_rhs(input, state.input_prev, state);
    let v_pred = mat_vec_mul_s(&rhs);
    let p = extract_controlling_voltages(&v_pred);
    let i_nl = solve_nonlinear(&p, state);
    let v = compute_final_voltages(&v_pred, &i_nl, state);

    state.v_prev = v;
    state.i_nl_prev = i_nl;
    state.input_prev = input;  // Track for trapezoidal RHS

    v[OUTPUT_NODE]
}
```

## Key Implementation Details

### K Matrix Sign
```rust
// K = N_v * S * N_i (no negation needed)
// K is naturally negative for stable circuits because:
//   N_i[anode] = -1 (current extracted), N_i[cathode] = +1 (current injected)
//   This makes K negative, providing correct negative feedback
```

### Generalized Block-Diagonal Jacobian
```rust
// For each row i, the device Jacobian J_dev is block-diagonal:
// - Diode at index d: J_dev[d][d] = g_d (1x1 block)
// - BJT at (s, s+1): J_dev is 2x2 block [dIc/dVbe, dIc/dVbc; dIb/dVbe, dIb/dVbc]
//
// NR Jacobian uses chain rule:
// J[i][j] = delta_ij - sum_k(jdev_{ik} * K[k][j])
// where k ranges over indices in i's device block
//
// For diode row d:  j_{dj} = delta_{dj} - jdev_{d}_{d} * K[d][j]
// For BJT Ic row s: j_{sj} = delta_{sj} - jdev_{s}_{s} * K[s][j] - jdev_{s}_{s+1} * K[s+1][j]
```

### History Handling
```rust
// WRONG: Separate history vector
let history = alpha * C * v_prev;
rhs += history;

// RIGHT: A_neg contains history
// A_neg = alpha*C - G
// rhs = A_neg * v_prev = alpha*C*v_prev - G*v_prev
//       ^^^^^^^^^^^^^ history term included!
```

### Linear Solve by M Size
- M=1: Direct division
- M=2: Cramer's rule (explicit 2x2 inverse)
- M=3, M=4: Inline Gaussian elimination with partial pivoting
- M>4: Not supported

## Verification Checklist
- [ ] INPUT_RESISTANCE matches G matrix stamping (default: 1 ohm)
- [ ] A_NEG contains alpha*C terms
- [ ] No separate cap_history in build_rhs
- [ ] K has no extra negation (naturally negative from kernel)
- [ ] Jacobian uses block-diagonal jdev entries (not just diagonal g_dev)
- [ ] Step clamping present (STEP_CLAMP = 0.01)
- [ ] is_finite() checks present
- [ ] S matrix values reasonable (< 1e6)

## Differences: Runtime vs Generated

| Aspect | Runtime Solver | Generated Code |
|--------|---------------|----------------|
| Jacobian | Full `J = I - J_dev * K` | Block-diagonal `J = I - J_dev * K` |
| Device Jacobian | Dense matrix from devices | Block-diagonal jdev entries |
| Linear solve | Gaussian elimination (any size) | Explicit for M<=4 |
| Handles | All device types | Diodes (1D) and BJTs (2D) |
