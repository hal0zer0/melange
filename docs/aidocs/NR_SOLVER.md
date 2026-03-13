# Newton-Raphson Nonlinear Solver

## Purpose
Solve M-dimensional nonlinear system: `f(i) = i - i_d(v(i)) = 0`

## References
- Hack Audio Tutorial Chapter 7: https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/

## Residual Function
```
v = p + K*i              // Controlling voltage
i_d = f_device(v)        // Device current (e.g., diode exponential)
f(i) = i - i_d           // Residual (should be 0 at solution)
```

K = N_v*S*N_i is naturally negative for stable circuits.
So `v = p + K*i = p - |K|*i` gives correct negative feedback.

**Note on trapezoidal nonlinear integration**: The prediction `p = N_v * v_pred` already
includes `K * i_nl_prev` from the RHS (via `N_i * i_nl_prev` in `build_rhs`). The NR
correction then adds `S * N_i * i_nl` (full, not delta). The combined effect is
`N_i * (i_nl_prev + i_nl)` — proper trapezoidal averaging. The NR equations themselves
(`f(i) = i - i_dev(p + K*i)` and `J = I - J_dev*K`) are unchanged by this choice.

## Jacobian Formula

### General Form (both runtime and codegen)
```
f(i) = i - i_dev(p + K*i)
J[i][j] = delta_ij - sum_k J_dev[i][k] * K[k][j]
```

**Note**: This formula is in current-space (M×M). The runtime `solve_md` uses the equivalent
voltage-space form `K * J_dev` to compute `J[i][j] = delta_ij - sum_k K[i][k] * J_dev[k][j]`.
Both are correct when J_dev is block-diagonal, since K*J_dev and J_dev*K produce the same
NR Jacobian entries for the relevant device blocks.

where `J_dev[i][k] = di_dev_i / dv_k` is the device Jacobian, which is block-diagonal:
- Diode at index d: `J_dev[d][d] = g_d` (1x1 block)
- BJT at indices (s, s+1): `J_dev` is a 2x2 block:
  ```
  [dIc/dVbe  dIc/dVbc]
  [dIb/dVbe  dIb/dVbc]
  ```

Since K is naturally negative, J > 0 (always convergent).

### In Generated Code (jdev naming)
```rust
// Diode at index d:
let jdev_d_d = diode_conductance(v_d);

// BJT at indices (s, s+1):
let bjt_jac = bjt_jacobian(v_d_s, v_d_s1);
let jdev_s_s   = bjt_jac[0];  // dIc/dVbe
let jdev_s_s1  = bjt_jac[1];  // dIc/dVbc
let jdev_s1_s  = bjt_jac[2];  // dIb/dVbe
let jdev_s1_s1 = bjt_jac[3];  // dIb/dVbc

// NR Jacobian entry:
// j_{ij} = delta_ij - sum_k(jdev_{ik} * K[k][j])
// where k ranges over the device block that owns row i
```

## Update Step
```
delta = J^{-1} * f            // Newton step (in current space)
dv = -K * delta               // Implied voltage change per device
alpha = min over devices of voltage_limit(dv, v_prev) / |dv|
i_nl -= alpha * delta          // Damped update
```

Voltage limiting uses SPICE3f5-style per-device limiters applied in voltage space:
- **pnjlim** (PN junction): Logarithmic compression for large forward steps above vcrit.
  Used for diodes (1D), BJTs (both junctions), tubes (grid current dimension).
  `vcrit = vt * ln(vt / (sqrt(2) * is))` precomputed per device as a constant.
- **fetlim** (FET): Prevents gate-source voltage from jumping across threshold.
  Used for JFETs and MOSFETs.

The scalar damping factor `alpha` (0 < alpha <= 1) is the minimum ratio across all
device dimensions, ensuring no single junction or gate voltage overshoots. When all
voltage steps are within safe bounds, alpha = 1.0 (full Newton step).

## Convergence Criteria
```
|delta| < tolerance (1e-9)  or  max_iter (100) reached
```

## Safety Measures

### SPICE-Style Voltage Limiting (Essential!)
```rust
// After computing Newton step delta in current space:
// 1. Compute implied voltage change: dv = -K * delta
// 2. Apply per-device pnjlim/fetlim to each voltage dimension
// 3. Compute scalar damping factor alpha = min(limited_dv / dv) across all dimensions
// 4. Apply damped step: i_nl -= alpha * delta
//
// Per-device VCRIT constants precomputed from pn_vcrit(vt, is):
const DEVICE_0_VCRIT: f64 = 0.6145;  // e.g., for diode with IS=1e-12
```

### Singular Jacobian Check
```rust
if det.abs() < 1e-15 {
    return current_best_guess;  // Don't divide by near-zero
}
```

### NaN/Inf Check
```rust
if !i_nl.iter().all(|x| x.is_finite()) {
    i_nl.fill(0.0);  // Reset to safe values
}
```

## Linear Solve by M Size

- M=1: Direct division
- M=2: Cramer's rule (explicit 2x2 inverse)
  ```
  det = j00*j11 - j01*j10
  delta0 = (j11*f0 - j01*f1) / det
  delta1 = (-j10*f0 + j00*f1) / det
  ```
- M=3..16: Inline Gaussian elimination with partial pivoting
- M>16: Not supported (MAX_M=16)

## Warm Start
Use `i_nl_prev` from previous time sample as initial guess. Reduces iterations from 10-20 to 3-5.

## BJT Self-Heating (Quasi-Static Thermal Update)

When self-heating is enabled (finite RTH), the junction temperature is updated
**once per sample, outside the NR loop** (quasi-static approximation). This means:

1. NR solves for currents at the current `Tj` (using temperature-adjusted IS and VT)
2. After NR converges, compute power dissipation: `P = Vce*Ic + Vbe*Ib`
3. Update `Tj` via thermal RC: `dTj/dt = (P - (Tj-Tamb)/Rth) / Cth`
4. Recompute `IS(Tj)` and `VT(Tj)` for the next sample

This avoids coupling thermal dynamics into the NR iteration (which would require
a larger Jacobian and slower convergence). The quasi-static approximation is valid
because thermal time constants (ms to s) are much slower than electrical time
constants (us to ms at audio rates).

## DC Operating Point Initialization

For circuits with DC bias (BJT amplifiers), `i_nl_prev` should be initialized from
the DC operating point, not zeros. See [DC_OP.md](DC_OP.md).

- **Codegen**: `DC_NL_I` constant auto-generated, initializes `i_nl_prev` in `Default`
- **Runtime**: Call `solver.initialize_dc_op(&mna, &device_slots)` after construction

**Note**: The DC OP solver uses a different NR formulation than the time-stepping solver.
The DC OP solves for node voltages directly (companion formulation on the N×N system),
while the time-stepping NR solves for nonlinear currents (M-dimensional system via DK reduction).

## Common Failures
1. **Extra K negation** -> Wrong feedback polarity, immediate divergence
2. **No voltage limiting** -> Oscillation on sharp corners
3. **Cold start (i=0)** -> Slower convergence
4. **Wrong VT temperature** -> Wrong operating point
5. **Diagonal-only Jacobian for BJT** -> Ignores cross-coupling, poor convergence

## Implementation Note
Both the runtime solver and codegen use the same Jacobian formula:
`J[i][j] = delta_ij - sum_k(J_dev[i][k] * K[k][j])`. The codegen generates
the device Jacobian as block-diagonal `jdev_` entries (1x1 for diodes, 2x2 for
BJTs), while the runtime solver computes the full device Jacobian matrix. Both
handle coupled multi-port devices (e.g., BJTs where Ic depends on both Vbe and Vbc).

## References
- Hack Audio Tutorial Chapter 7: Newton-Raphson Method
