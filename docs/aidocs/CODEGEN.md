# Code Generation Reference

## Generated Structure

### Constants (Compile-Time)
```rust
const N: usize = 4;           // Number of nodes
const M: usize = 2;           // Nonlinear device dimensions
const SAMPLE_RATE: f64 = 48000.0;
const INPUT_RESISTANCE: f64 = 1.0;  // 1Ω near-ideal voltage source
const OVERSAMPLING_FACTOR: usize = 2;       // 1, 2, or 4
const INTERNAL_SAMPLE_RATE: f64 = 96000.0;  // Only when factor > 1

// G and C stored for runtime recomputation via set_sample_rate()
const G: [[f64; N]; N] = [...];        // Conductance matrix
const C: [[f64; N]; N] = [...];        // Capacitance matrix
const N_V: [[f64; N]; M] = [...];      // Voltage extraction (constant)
const N_I: [[f64; N]; M] = [...];      // Current injection (constant)

// Per-device constants
const DEVICE_0_IS: f64 = 1e-12;
const DEVICE_0_N_VT: f64 = 0.026;
const DEVICE_1_IDSS: f64 = 2e-3;      // JFET
const DEVICE_1_VP: f64 = -2.0;
const DEVICE_1_SIGN: f64 = 1.0;       // +1.0 N-ch, -1.0 P-ch
```

### State (Runtime, Mutable Matrices)
```rust
struct CircuitState {
    // Matrices recomputed by set_sample_rate()
    s: [[f64; N]; N],          // S = A^{-1} (at internal rate)
    a_neg: [[f64; N]; N],      // A_neg = alpha*C - G
    k: [[f64; M]; M],          // K = N_v*S*N_i
    s_ni: [[f64; M]; N],       // S*N_i (for final voltages)
    // ...
}
```

### DC Operating Point Constants
```rust
// For circuits with nonlinear devices where DC OP has non-zero i_nl:
pub const DC_NL_I: [f64; M] = [1.234e-3, 5.678e-6];  // From dc_op solver
```

Emitted when `has_dc_nl` is true (M > 0 and any dc_nl_current is nonzero).
Used to initialize `i_nl_prev` in both `Default` and `reset()`.

### State (Runtime, Per-Channel)
```rust
struct CircuitState {
    v_prev: [f64; N],        // Previous node voltages
    i_nl_prev: [f64; M],     // Previous nonlinear currents (init from DC_NL_I if present)
    input_prev: f64,          // Previous input (for trapezoidal RHS)
    dc_operating_point: [f64; N],
    last_nr_iterations: u32,
    // Runtime matrices (recomputed by set_sample_rate)
    s: [[f64; N]; N],
    a_neg: [[f64; N]; N],
    k: [[f64; M]; M],
    s_ni: [[f64; M]; N],
    // Oversampler state (when factor > 1)
    os_up_state: [f64; STATE_SIZE],
    os_dn_state: [f64; STATE_SIZE],
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

### JFET (1D per device)
```rust
fn jfet_id(vgs: f64, idss: f64, vp: f64, sign: f64) -> f64;          // Saturation: IDSS*(1-Vgs/Vp)^2
fn jfet_conductance(vgs: f64, idss: f64, vp: f64, sign: f64) -> f64; // 2*IDSS*Vgst/Vp^2
```

1D saturation-only model: ignores Vds (no triode/ohmic region). Vgst is clamped to
`[0, |Vp|]` so drain current never exceeds IDSS (prevents runaway for forward-biased gate).

- N-channel (`.model name NJ(...)`): sign=+1.0, default VTO=-2.0
- P-channel (`.model name PJ(...)`): sign=-1.0, default VTO=+2.0
- Default IDSS=2e-3 A, lambda=0.001 (stored but unused in 1D model)
- Parameter lookup: `VTO` only (no `VT` alias, to avoid confusion with thermal voltage)

### Tube/Triode (2D per device)
```rust
fn tube_ip(vgk: f64, vpk: f64, mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64) -> f64;  // Koren plate current
fn tube_ig(vgk: f64, ig_max: f64, vgk_onset: f64) -> f64;                                // Leach grid current
fn tube_jacobian(vgk: f64, vpk: f64, mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64,
                 ig_max: f64, vgk_onset: f64) -> [f64; 4];  // [dIp/dVgk, dIp/dVpk, dIg/dVgk, dIg/dVpk]
```

2D model: plate current (Ip) at `start_idx`, grid current (Ig) at `start_idx+1`.

- **Plate current**: Koren model — `Ip = (E1^ex / kg1) * atan(vpk / kvb)` where `E1 = vpk/kp * ln(1 + exp(kp * (1/mu + vgk/vpk)))`
- **Grid current**: Leach power-law — `Ig = ig_max * max(0, (vgk - vgk_onset) / vgk_onset)^2` (zero for vgk < vgk_onset)
- **Jacobian**: 4-element `[dIp/dVgk, dIp/dVpk, dIg/dVgk, dIg/dVpk]` — dIg/dVpk = 0 (grid current independent of plate voltage)
- Constants: `DEVICE_{n}_MU`, `DEVICE_{n}_EX`, `DEVICE_{n}_KG1`, `DEVICE_{n}_KP`, `DEVICE_{n}_KVB`, `DEVICE_{n}_IG_MAX`, `DEVICE_{n}_VGK_ONSET`
- Netlist syntax: `X1 plate grid cathode modelname` with `.model modelname TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)`
- Default params: MU=100, EX=1.4, KG1=1060, KP=600, KVB=300, IG_MAX=2e-3, VGK_ONSET=0.5
- Template: `device_tube.rs.tera`

## Device Map and M-Dimension Assignment

Nonlinear devices occupy M dimensions in order of appearance in the netlist:
- Diode: 1 dimension (controlling voltage Vd, current Id)
- BJT: 2 dimensions (Vbe->Ic, Vbc->Ib)
- JFET: 1 dimension (Vgs->Id)
- Tube: 2 dimensions (Vgk->Ip, Vpk->Ig)
- MOSFET: 1 dimension (Vgs->Id) — not yet supported in NR codegen

Example for a circuit with D1, Q1, X1 (triode):
```
M index:  0      1       2       3       4
Device:   D1     Q1(Ic)  Q1(Ib)  X1(Ip)  X1(Ig)
Dim:      1D     -----2D-----    ------2D------
```

## Functions

### build_rhs
```rust
fn build_rhs(input: f64, input_prev: f64, state: &CircuitState) -> [f64; N] {
    // RHS_CONST (DC sources, if any)
    // + A_neg * v_prev  (includes capacitor history via alpha*C!)
    // + N_i^T * i_nl_prev  (part of trapezoidal nonlinear integration)
    // + (input + input_prev) / INPUT_RESISTANCE  (proper trapezoidal)
}
```
A_neg already contains alpha*C. Do NOT add separate cap_history.
The input uses proper trapezoidal integration: `(V_in(n+1) + V_in(n)) * G_in`.
The `N_i * i_nl_prev` term, combined with `S * N_i * i_nl` in `compute_final_voltages`,
gives the trapezoidal average `N_i * (i_nl[n+1] + i_nl[n])`.

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
// Without oversampling (factor=1):
pub fn process_sample(input: f64, state: &mut CircuitState) -> f64 { ... }

// With oversampling (factor=2):
fn process_sample_inner(input: f64, state: &mut CircuitState) -> f64 { ... }
pub fn process_sample(input: f64, state: &mut CircuitState) -> f64 {
    // Upsample: [input, halfband_up(input)]
    // Process both samples through process_sample_inner
    // Downsample: halfband_dn(out0) → return out1
}
```

The inner function contains the standard DK pipeline:
```rust
fn process_sample_inner(input: f64, state: &mut CircuitState) -> f64 {
    let rhs = build_rhs(input, state.input_prev, state);
    let v_pred = mat_vec_mul_s(&rhs, state);  // Uses state.s (runtime matrix)
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
- M=3..16: Inline Gaussian elimination with partial pivoting
- M>16: Not supported (MAX_M=16)

## Verification Checklist
- [ ] INPUT_RESISTANCE matches G matrix stamping (default: 1 ohm)
- [ ] A_NEG contains alpha*C terms
- [ ] No separate cap_history in build_rhs
- [ ] K has no extra negation (naturally negative from kernel)
- [ ] Jacobian uses block-diagonal jdev entries (not just diagonal g_dev)
- [ ] Step clamping present (STEP_CLAMP = 0.01)
- [ ] is_finite() checks present
- [ ] S matrix values reasonable (< 1e6)

## DC Operating Point in Codegen

The codegen pipeline calls `dc_op::solve_dc_operating_point()` to find the DC bias:

```rust
// In CircuitIR::from_kernel():
let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);
ir.dc_operating_point = dc_result.v_node;
ir.dc_nl_currents = dc_result.i_nl;
ir.dc_op_converged = dc_result.converged;
```

The emitter checks `has_dc_nl = M > 0 && dc_nl_currents.iter().any(|&x| x.abs() > 1e-30)`:
- If true: emits `DC_NL_I` constant, initializes `i_nl_prev` from it
- If false: initializes `i_nl_prev` to `[0.0; M]`

`CodegenConfig` fields: `dc_op_max_iterations` (default 200), `dc_op_tolerance` (default 1e-9).

## Oversampling

Generated code supports 2x and 4x oversampling via `CodegenConfig.oversampling_factor` (default: 1).

### How It Works
When factor > 1, the codegen:
1. Recomputes all matrices (S, A_neg, K, S_NI) at `sample_rate * factor` from stored G+C
2. Renames `process_sample` → `process_sample_inner` (private)
3. Emits a public `process_sample` wrapper that upsamples → processes → downsamples
4. Emits self-contained polyphase allpass half-band filter functions inline (no external dependencies)

### Filter Design
- **2x**: 3-section polyphase allpass half-band (~80dB stopband rejection)
- **4x**: Cascaded 2x — outer stage uses 2-section (~60dB), inner stage uses 3-section (~80dB)
- Coefficients from `melange-primitives/src/oversampling.rs` (HB_3SECTION, HB_2SECTION)

### State Fields
- `os_up_state`, `os_dn_state`: allpass filter state for upsample/downsample (2x)
- `os_up_state_outer`, `os_dn_state_outer`: additional state for outer stage (4x only)
- All reset in `reset()` and reinitialized in `set_sample_rate()`

### `set_sample_rate()` with Oversampling
When oversampling is active, `set_sample_rate(sr)` computes `internal_rate = sr * OVERSAMPLING_FACTOR`
and recomputes all matrices at the internal rate.

## Sparsity-Aware Emission

The IR includes `SparseInfo` with per-matrix nonzero entry lists (threshold: `|x| < 1e-20` = structural zero).
The emitter skips zero entries uniformly in:
- `build_rhs`: A_neg * v_prev multiplication
- `extract_controlling_voltages`: N_v * v_pred
- `solve_nonlinear`: K * i_nl
- `compute_final_voltages`: S * N_i * i_nl

This reduces generated code size and improves performance for sparse circuits.

## Runtime Sample Rate Support

Generated code stores G and C as constants and recomputes S, A_neg, K, S_NI at runtime:

```rust
impl CircuitState {
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        let internal_rate = sample_rate * OVERSAMPLING_FACTOR as f64;
        let alpha = 2.0 * internal_rate;
        // Recompute A = G + alpha*C, then S = A^{-1}
        // Recompute A_neg = alpha*C - G
        // Recompute K = N_v * S * N_i
        // Recompute S_NI = S * N_i (for final voltage computation)
        // Recompute inductor g_eq and pot SM vectors if applicable
    }
}
```

Matrix inversion uses inline Gaussian elimination (`invert_n()` helper emitted in generated code).

## Differences: Runtime vs Generated

| Aspect | Runtime Solver | Generated Code |
|--------|---------------|----------------|
| Jacobian | Full `J = I - J_dev * K` | Block-diagonal `J = I - J_dev * K` |
| Device Jacobian | Dense matrix from devices | Block-diagonal jdev entries |
| Linear solve | Gaussian elimination (any size) | Explicit for M<=16 (Gauss elim for M>=3) |
| Handles | All device types | Diodes (1D), BJTs (2D), JFETs (1D), Tubes (2D) |
| DC OP init | `initialize_dc_op()` (opt-in) | `DC_NL_I` constant (automatic) |
| Oversampling | Not available | 2x/4x polyphase half-band IIR |
| Sparsity | Dense | Zero entries skipped in emission |
| Sample rate | Fixed at construction | `set_sample_rate()` recomputes from G+C |
