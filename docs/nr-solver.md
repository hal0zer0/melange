# Newton-Raphson Solver for Real-Time Circuit Simulation

Newton-Raphson (NR) iteration solves the nonlinear equations that arise in circuit simulation after the DK method has reduced the problem to an M×M system. This document covers the theory, implementation strategies, and real-time constraints specific to audio DSP.

---

## 1. NR for Circuit Simulation

### The Nonlinear Equation

After DK reduction, the circuit reduces to solving for the voltages across M nonlinear devices:

```
f(v) = v - p - K·i(v) = 0
```

Where:
- **v** ∈ ℝᴹ: Nonlinear port voltages (unknowns)
- **p** ∈ ℝᴹ: Predicted voltages from linear solve (`p = N_v · S · rhs`)
- **K** ∈ ℝᴹˣᴹ: Nonlinear kernel matrix (`K = N_v · S · N_i`)
- **i(v)**: Vector of nonlinear device currents as function of port voltages
- **M**: Number of nonlinear devices (typically 1-4 for audio circuits)

### Why M is Typically 1-4 for Audio Circuits

Audio circuits are designed for predictable, low-distortion behavior. This constrains the topology:

| M | Common Cases | Example Circuits |
|---|--------------|------------------|
| 1 | Single diode, BJT common-emitter, tube triode | Fuzz pedal, simple preamp stage |
| 2 | Differential pair, long-tailed pair, two BJTs | Opamp input stage, tube phase inverter |
| 3 | Three-transistor clipping stage | Multiple gain stages in series |
| 4 | Complex tube stages, multiple diode clippers | Guitar amp preamp, compressor |

Circuits with M > 4 exist (e.g., full opamp macro-models) but are rarely needed for real-time audio where simplified device models suffice.

### The General M-Dimensional Case

For M nonlinear devices, we solve the system:

```
J(v_n) · Δv = -f(v_n)
v_{n+1} = v_n + Δv
```

Where the Jacobian is:

```
J = I - K · G(v)
```

And G(v) = diag(g₁, g₂, ..., gₘ) contains the small-signal conductances of each nonlinear device:

```
gⱼ = diⱼ/dvⱼ  (transconductance of device j)
```

---

## 2. 1D NR (Scalar Case)

The scalar case (M=1) is the most common in audio circuits: a single diode, BJT, or tube stage.

### The Algorithm

```
f(v) = v - p - K·i(v) = 0
f'(v) = 1 - K·g(v)     where g(v) = di/dv

v_{n+1} = v_n - f(v_n)/f'(v_n)
        = v_n - (v_n - p - K·i(v_n)) / (1 - K·g(v_n))
```

### Pseudocode (Hot Path)

```rust
/// Solve 1D nonlinear equation f(v) = v - p - K*i(v) = 0
/// Returns (solution, iterations_used) or error if non-convergent
fn nr_solve_1d(
    p: f64,           // Predicted voltage
    k: f64,           // Kernel coefficient (scalar K)
    i_fn: impl Fn(f64) -> f64,      // Current function i(v)
    g_fn: impl Fn(f64) -> f64,      // Conductance function g(v) = di/dv
    v0: f64,          // Initial guess (warm start)
    tol: f64,         // Convergence tolerance (e.g., 1e-9)
    max_iter: usize,  // Maximum iterations (e.g., 20)
) -> Result<(f64, usize), NrError> {
    let mut v = v0;
    
    for iter in 0..max_iter {
        let i = i_fn(v);
        let g = g_fn(v);
        
        // Residual
        let f = v - p - k * i;
        
        // Check convergence on residual
        if f.abs() < tol {
            return Ok((v, iter + 1));
        }
        
        // Jacobian (scalar): J = 1 - K*g
        let j = 1.0 - k * g;
        
        // Newton step
        let dv = f / j;
        
        // Damping: limit step to 2*Vt for diode-like devices
        // Vt = kT/q ≈ 25.85mV at room temperature
        const MAX_DV: f64 = 2.0 * 0.02585;  // ~52mV
        let dv_clamped = dv.clamp(-MAX_DV, MAX_DV);
        
        v -= dv_clamped;
        
        // Check convergence on step size
        if dv_clamped.abs() < tol {
            return Ok((v, iter + 1));
        }
        
        // NaN/inf guard
        if !v.is_finite() {
            return Err(NrError::Divergence);
        }
    }
    
    Err(NrError::MaxIterations)
}
```

### Clamping/Damping Strategies

**Voltage clamping**: Limit the step size to prevent overshoot in the exponential region of diode/BJT characteristics.

```
|Δv| ≤ 2·Vt  where Vt ≈ 25.85 mV at 300K
```

This bound comes from the diode equation: a 2·Vt change in voltage changes current by e² ≈ 7.4×. Limiting to this range prevents the iteration from jumping into regions where the linear approximation is invalid.

**Hybrid damping**: Use full NR step when close to solution, clamped step when far:

```
α = min(1.0, 2·Vt / |Δv|)  // Damping factor
v_{new} = v - α·Δv
```

### Convergence Criteria

Two complementary tests:

1. **Residual test**: `|f(v)| < tol_f` — the equation is satisfied
2. **Step test**: `|Δv| < tol_v` — the solution has stabilized

Typical tolerances for audio:
- `tol_f = 1e-9` to `1e-12` (relative to signal level)
- `tol_v = 1e-12` to `1e-15` (absolute voltage)

Early termination when either is satisfied saves cycles. The residual test usually triggers first when far from solution; the step test triggers near convergence.

### Max Iteration Handling

Real-time systems cannot iterate indefinitely. Common bounds:

- **Typical**: 10-20 iterations (sufficient for most audio circuits)
- **Tight**: 50 iterations (pathological cases)
- **Hard limit**: Must terminate before audio buffer deadline

If max iterations are reached without convergence:
1. Return the best estimate (may audibly glitch)
2. Log/flag for analysis (debug builds)
3. Fall back to previous solution (with smoothing)

---

## 3. 2D NR (Common Audio Case)

The 2D case arises with differential pairs, push-pull stages, and coupled transistor pairs.

### System Structure

```
v = [v₁, v₂]ᵀ
p = [p₁, p₂]ᵀ
K = [[K₁₁, K₁₂],
     [K₂₁, K₂₂]]
i(v) = [i₁(v₁), i₂(v₂)]ᵀ
```

The nonlinear equations:

```
f₁(v₁, v₂) = v₁ - p₁ - (K₁₁·i₁(v₁) + K₁₂·i₂(v₂)) = 0
f₂(v₁, v₂) = v₂ - p₂ - (K₂₁·i₁(v₁) + K₂₂·i₂(v₂)) = 0
```

### Jacobian: J = I - K·G

```
G = diag(g₁, g₂) = [[g₁,  0 ],
                    [ 0,  g₂]]

J = [[1 - K₁₁·g₁,   -K₁₂·g₂  ],
     [  -K₂₁·g₁,   1 - K₂₂·g₂]]
```

### Cramer's Rule for 2×2 Solve

For a 2×2 system `J·Δv = -f`, Cramer's rule is faster than generic LU:

```
det = J₁₁·J₂₂ - J₁₂·J₂₁

Δv₁ = (-f₁·J₂₂ - J₁₂·(-f₂)) / det
    = (J₁₂·f₂ - J₂₂·f₁) / det

Δv₂ = (J₁₁·(-f₂) - (-f₁)·J₂₁) / det
    = (J₂₁·f₁ - J₁₁·f₂) / det
```

### Pseudocode (Hot Path)

```rust
/// Solve 2D nonlinear system using Cramer's rule
fn nr_solve_2d(
    p: [f64; 2],           // Predicted voltages
    k: [[f64; 2]; 2],      // Kernel matrix
    i_fn: impl Fn([f64; 2]) -> [f64; 2],   // Currents [i1(v1), i2(v2)]
    g_fn: impl Fn([f64; 2]) -> [f64; 2],   // Conductances [g1, g2]
    v0: [f64; 2],          // Initial guess
    tol: f64,
    max_iter: usize,
) -> Result<([f64; 2], usize), NrError> {
    const MAX_DV: f64 = 2.0 * 0.02585;
    const VT: f64 = 0.02585;
    
    let mut v = v0;
    
    for iter in 0..max_iter {
        let i = i_fn(v);
        let g = g_fn(v);
        
        // Residual: f = v - p - K*i
        let f = [
            v[0] - p[0] - (k[0][0]*i[0] + k[0][1]*i[1]),
            v[1] - p[1] - (k[1][0]*i[0] + k[1][1]*i[1]),
        ];
        
        // Convergence test
        let f_norm = (f[0]*f[0] + f[1]*f[1]).sqrt();
        if f_norm < tol {
            return Ok((v, iter + 1));
        }
        
        // Jacobian: J = I - K*G
        let j11 = 1.0 - k[0][0] * g[0];
        let j12 =     - k[0][1] * g[1];
        let j21 =     - k[1][0] * g[0];
        let j22 = 1.0 - k[1][1] * g[1];
        
        // Cramer's rule for 2x2
        let det = j11*j22 - j12*j21;
        
        if det.abs() < 1e-18 {
            return Err(NrError::SingularJacobian);
        }
        
        let inv_det = 1.0 / det;
        let dv1 = (j12 * f[1] - j22 * f[0]) * inv_det;
        let dv2 = (j21 * f[0] - j11 * f[1]) * inv_det;
        
        // Per-dimension clamping
        let dv = [
            dv1.clamp(-MAX_DV, MAX_DV),
            dv2.clamp(-MAX_DV, MAX_DV),
        ];
        
        v[0] -= dv[0];
        v[1] -= dv[1];
        
        // Convergence on step size
        let dv_norm = (dv[0]*dv[0] + dv[1]*dv[1]).sqrt();
        if dv_norm < tol {
            return Ok((v, iter + 1));
        }
        
        // NaN guard
        if !v[0].is_finite() || !v[1].is_finite() {
            return Err(NrError::Divergence);
        }
    }
    
    Err(NrError::MaxIterations)
}
```

### Step Damping with Per-Dimension Clamping

Different devices may have different scales. For circuits mixing diodes (Vt ~ 26mV) and tubes (operating at higher voltages), use adaptive clamping:

```rust
// Adaptive clamping based on device type
let max_dv = [2.0 * vt_diode, 0.5];  // Diode vs tube stage
let dv_clamped = [
    dv[0].clamp(-max_dv[0], max_dv[0]),
    dv[1].clamp(-max_dv[1], max_dv[1]),
];
```

---

## 4. General M×M NR

For M > 2, we need a general linear solver. M > 4 is rare in audio but handled for completeness.

### Gaussian Elimination with Partial Pivoting

```rust
/// Solve J·Δv = -f using Gaussian elimination with partial pivoting
/// J is M×M, modified in-place (or use stack-allocated copy)
fn solve_linear_m<const M: usize>(
    j: &mut [[f64; M]; M],  // Jacobian (modified in-place)
    f: &[f64; M],           // RHS (residual, negated)
    dv: &mut [f64; M],      // Output: solution
) -> Result<(), NrError> {
    // Forward elimination with partial pivoting
    for col in 0..M {
        // Find pivot
        let mut max_row = col;
        let mut max_val = j[col][col].abs();
        for row in (col+1)..M {
            if j[row][col].abs() > max_val {
                max_val = j[row][col].abs();
                max_row = row;
            }
        }
        
        if max_val < 1e-18 {
            return Err(NrError::SingularJacobian);
        }
        
        // Swap rows
        if max_row != col {
            j.swap(col, max_row);
            // Would also need to swap f elements (track separately)
        }
        
        // Eliminate column
        let pivot = j[col][col];
        for row in (col+1)..M {
            let factor = j[row][col] / pivot;
            for k in col..M {
                j[row][k] -= factor * j[col][k];
            }
            // Update RHS
            f[row] -= factor * f[col];  // Track f separately
        }
    }
    
    // Back substitution
    for i in (0..M).rev() {
        let mut sum = f[i];
        for j_idx in (i+1)..M {
            sum -= j[i][j_idx] * dv[j_idx];
        }
        dv[i] = sum / j[i][i];
    }
    
    Ok(())
}
```

### LU Decomposition (Precomputed Pattern)

For circuits with fixed topology but time-varying conductances, the Jacobian sparsity pattern is fixed. LU decomposition with symbolic factorization can reuse the elimination pattern:

```rust
// Symbolic factorization (once, at circuit compile time)
let lu_pattern = compute_lu_pattern(jacobian_sparsity);

// Numeric factorization (each NR iteration)
lu_decompose_numeric(&mut j_values, &lu_pattern);

// Solve
lu_solve(&j_values, &lu_pattern, &f, &mut dv);
```

This is overkill for M ≤ 4 but essential for M > 10 (e.g., full transistor-level opamp models).

### When M > 4 Is Needed

| Scenario | M | Strategy |
|----------|---|----------|
| Full opamp macro-model | 8-16 | Simplify to behavioral model, or use MNA directly |
| Multi-stage compressor | 6-8 | Hierarchical solve (stage by stage) |
| Complex tube amp | 5-6 | Accept higher cost, or table-lookup for some devices |
| Coupled inductors + nonlinear | 4-8 | Use fixed-point iteration for weak coupling |

For real-time audio, prefer circuit simplification over complex solvers.

---

## 5. Warm Starting

The initial guess `v₀` significantly affects convergence speed. Audio signals have temporal coherence—we can exploit this.

### Previous Timestep Solution

The simplest warm start: use the solution from the previous sample:

```rust
v_nl[n] = nr_solve(p[n], v_nl[n-1])
```

This works well for:
- Low-frequency signals (slow variation)
- Oversampled systems (smaller per-sample changes)
- Circuits with dominant poles (natural smoothing)

### Linear Prediction

For rapidly changing signals, extrapolate from previous samples:

```rust
// First-order prediction (linear trend)
v_pred = 2*v[n-1] - v[n-2]

// Second-order prediction
v_pred = 3*v[n-1] - 3*v[n-2] + v[n-3]
```

With clamping to prevent wild extrapolation:

```rust
let trend = v[n-1] - v[n-2];
let max_extrapolation = 4.0 * Vt;  // Limit predicted change
let v0 = v[n-1] + trend.clamp(-max_extrapolation, max_extrapolation);
```

### DC Operating Point Reset

When to abandon warm start and reset:

1. **NaN/inf detection**: Solution diverged, circuit state corrupted
2. **Large input transients**: Step change in input (e.g., switch click)
3. **Parameter change**: Potentiometer swept rapidly
4. **NR failure**: Max iterations reached

DC operating point is expensive to compute (iterative solve with no warm start), so only use as last resort:

```rust
enum WarmStartStrategy {
    PreviousSolution,   // Normal operation
    LinearPrediction,   // Rapid signal
    DCOperatingPoint,   // Recovery
}
```

---

## 6. Convergence Aids

### Homotopy Methods (Source Stepping)

For circuits with difficult DC convergence (latch-up, multiple stable points), use continuation:

```rust
// Source stepping: gradually ramp up input from 0
for lambda in [0.1, 0.3, 0.6, 1.0] {
    let p_step = lambda * p_target;
    v = nr_solve(p_step, v)?;
}
```

Each solution provides warm start for the next. This is primarily for DC analysis, not per-sample.

### Line Search / Damping

Globalize convergence with backtracking line search:

```rust
let mut alpha = 1.0;
let f_norm_current = f(v).norm();

while alpha > 0.01 {
    let v_trial = v - alpha * dv;
    let f_norm_trial = f(v_trial).norm();
    
    if f_norm_trial < f_norm_current {
        v = v_trial;
        break;
    }
    alpha *= 0.5;
}
```

Too expensive for per-sample hot path; use fixed damping instead:

```rust
// Fixed under-relaxation (safer but slower)
const ALPHA: f64 = 0.7;
v -= ALPHA * dv;
```

### Step Limiting Heuristics

Beyond voltage clamping, limit based on device physics:

```rust
// For BJTs: limit Vbe to valid range (0.3V to 0.9V typical)
let vbe_clamped = vbe.clamp(0.3, 0.9);

// For tubes: limit grid-cathode voltage to avoid cutoff/saturation
let vgk_clamped = vgk.clamp(-3.0, 0.0);  // Typical operating range
```

---

## 7. Real-Time Constraints

### Worst-Case Iteration Bounds

Audio callbacks have hard deadlines. Compute worst-case NR cost:

```
Cost per iteration = M device evals + linear solve + convergence checks
Total cost = max_iter × cost_per_iteration

At 48kHz, 64-sample buffer: deadline = 64/48000 = 1.33ms
At 96kHz, 32-sample buffer: deadline = 0.33ms
```

For 1D NR with 10 iterations:
- ~10 exponential evaluations (diode/BJT)
- ~100 FLOPs for arithmetic
- Well within budget even at 192kHz

For 2D NR with 20 iterations:
- ~40 device evaluations
- ~200 FLOPs for Cramer's rule
- Still acceptable, but monitor carefully

### Detecting and Recovering from Non-Convergence

```rust
match nr_solve(params) {
    Ok((v, iters)) => {
        // Track iteration count for statistics
        iter_histogram[iters] += 1;
        v_nl = v;
    }
    Err(NrError::MaxIterations) => {
        // Use best estimate but flag quality
        v_nl = v_best;
        quality = Quality::Approximate;
        
        // Consider resetting warm start for next sample
        reset_warm_start = true;
    }
    Err(NrError::Divergence) => {
        // Fall back to DC operating point
        v_nl = v_dc;
        quality = Quality::Reset;
    }
}
```

### NaN/Inf Detection and Reset

```rust
fn reset_if_invalid(v: &mut [f64; M]) -> bool {
    let invalid = v.iter().any(|x| !x.is_finite());
    if invalid {
        *v = [0.0; M];  // Reset to zero (safe but inaccurate)
        // Or use precomputed DC operating point
    }
    invalid
}
```

For production, maintain a "safe mode" circuit model that evaluates quickly without NR (e.g., linearized small-signal model) for use when NR fails.

---

## 8. Implementation in melange-primitives

### nr_solve_1d() Signature

```rust
/// Error types for NR solver
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NrError {
    MaxIterations,
    Divergence,       // NaN or inf encountered
    SingularJacobian,
}

/// Solve scalar nonlinear equation f(v) = v - p - K*i(v) = 0
/// 
/// # Arguments
/// * `p` - Predicted voltage from linear solve
/// * `k` - Kernel coefficient (scalar K from DK method)
/// * `i_fn` - Device current function i(v)
/// * `g_fn` - Device conductance function g(v) = di/dv
/// * `v0` - Initial guess (warm start)
/// * `tol` - Convergence tolerance (default: 1e-9)
/// * `max_iter` - Maximum iterations (default: 20)
///
/// # Returns
/// * `Ok((v, iterations))` - Solution and iteration count
/// * `Err(NrError)` - Non-convergence
///
/// # Real-Time Safety
/// - No heap allocation
/// - Fixed iteration bound
/// - Branch-free hot path (with care)
pub fn nr_solve_1d<F, G>(
    p: f64,
    k: f64,
    i_fn: F,
    g_fn: G,
    v0: f64,
    tol: f64,
    max_iter: usize,
) -> Result<(f64, usize), NrError>
where
    F: Fn(f64) -> f64,  // i(v)
    G: Fn(f64) -> f64,  // g(v) = di/dv
{
    // ... implementation ...
}
```

### nr_solve_2d() with Cramer's Rule

```rust
/// Solve 2D nonlinear system f(v) = v - p - K*i(v) = 0
/// 
/// Uses Cramer's rule for the 2x2 linear solve (faster than LU).
pub fn nr_solve_2d<F, G>(
    p: [f64; 2],
    k: [[f64; 2]; 2],
    i_fn: F,
    g_fn: G,
    v0: [f64; 2],
    tol: f64,
    max_iter: usize,
) -> Result<([f64; 2], usize), NrError>
where
    F: Fn([f64; 2]) -> [f64; 2],
    G: Fn([f64; 2]) -> [f64; 2],
{
    // ... implementation ...
}
```

### Const-Generic M×M Solver

```rust
/// General M-dimensional NR solver using Gaussian elimination
/// 
/// M is a compile-time constant for stack allocation and optimization.
pub fn nr_solve_nd<const M: usize, F, G>(
    p: [f64; M],
    k: [[f64; M]; M],
    i_fn: F,
    g_fn: G,
    v0: [f64; M],
    tol: f64,
    max_iter: usize,
) -> Result<([f64; M], usize), NrError>
where
    F: Fn([f64; M]) -> [f64; M],
    G: Fn([f64; M]) -> [f64; M],
{
    // ... implementation using stack-allocated arrays ...
}
```

### Error Handling and Return Values

The solver returns rich information for diagnostics:

```rust
pub struct NrResult<const M: usize> {
    pub solution: [f64; M],
    pub iterations: usize,
    pub residual_norm: f64,
    pub converged: bool,
}

// Simplified API for production
pub fn nr_solve_1d_simple<F, G>(
    p: f64,
    k: f64,
    i_fn: F,
    g_fn: G,
    v0: f64,
) -> f64
where
    F: Fn(f64) -> f64,
    G: Fn(f64) -> f64,
{
    match nr_solve_1d(p, k, i_fn, g_fn, v0, 1e-9, 20) {
        Ok((v, _)) => v,
        Err(_) => v0,  // Fall back to initial guess
    }
}
```

### Complete Hot Path Example

```rust
/// Per-sample processing for a diode clipper (1D NR)
pub fn process_sample(&mut self, input: f64) -> f64 {
    // 1. Build RHS from history and input
    let rhs = self.a_neg * self.v_prev + input;
    
    // 2. Linear prediction
    let v_pred = self.s * rhs;
    let p = self.nv * v_pred;
    
    // 3. NR solve for nonlinear voltage
    let (v_nl, iters) = nr_solve_1d(
        p,
        self.k,
        |v| diode_current(v, self.is, self.vt),   // i(v)
        |v| diode_conductance(v, self.is, self.vt), // g(v)
        self.v_nl_prev,  // Warm start
        1e-10,
        15,
    ).unwrap_or((self.v_nl_prev, 0));  // Fall back on failure
    
    // 4. Update state
    self.v_nl_prev = v_nl;
    let i_nl = diode_current(v_nl, self.is, self.vt);
    let v_out = v_pred + self.s * self.ni * i_nl;
    self.v_prev = v_out;
    
    v_out
}
```

---

## References

- Yeh, D. T. (2009). "Digital Implementation of Musical Distortion Circuits by Analysis and Simulation." Stanford University.
- OrCAD/PSpice A/D Reference Manual - Newton-Raphson implementation details
- Quarteroni, Sacco, Saleri: "Numerical Mathematics" - convergence theory
- H. K. Thornquist et al.: "Aria: A Library for Iterative Solvers" - real-time considerations
