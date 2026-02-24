# DK Method Implementation Reference

David Yeh's Discrete K-Method (Stanford, 2009) for circuit simulation.

## Core Idea

Solve N-node linear system analytically → reduce to M×M nonlinear kernel (M = nonlinear device count). M is typically 1-4 even for complex circuits.

## Matrices

**G** (conductance): Stamps from resistors, companion conductances  
**C** (capacitance): Stamps from capacitors  
**N_v** (M×N): Extracts nonlinear voltages from node voltages  
**N_i** (N×M): Injects nonlinear currents into circuit

## Trapezoidal Discretization

```
A = 2C/T + G          // Forward matrix
A_neg = 2C/T - G      // History matrix (uses SAME G)
S = A⁻¹               // Precomputed
K = N_v * S * N_i     // Nonlinear kernel (M×M)
```

## Per-Sample Solve

```
// 1. Build RHS from history
rhs = A_neg * v[n-1] + 2w + N_i * i_nl[n-1] + sources

// 2. Predict (linear solve)
v_pred = S * rhs
p = N_v * v_pred       // Predicted NL voltages

// 3. NR solve on M×M system
// v_nl = p + K * i_nl(v_nl)
for iter in 0..max_iter:
    i = nonlinear_current(v_nl)
    g = nonlinear_gradient(v_nl)  // di/dv
    
    // Residual: f(v) = v - p - K*i(v) = 0
    // Jacobian: J = I - K*g
    // Solve: J * dv = f, v -= dv

// 4. Final currents
i_nl[n] = i_nl(v_nl)

// 5. Update
v[n] = v_pred + S * N_i * i_nl[n]
```

## Sherman-Morrison (Time-Varying Elements)

For R_ldr varying at audio rate, don't recompute S. Use rank-1 update:

```
S_eff = S_base - k * s_col * s_row^T
where k = g_ldr / (1 + s_fb_fb * g_ldr)
```

Corrects both prediction `v_pred` and kernel `K_eff`.

## Key Implementation Notes

- **Const generics:** Matrix sizes at compile time (`[f64; N]`, not `Vec`)
- **Warm start:** Use previous `v_nl` as NR initial guess
- **Clamping:** Limit `v_nl` step to `2*Vt` per iteration
- **NaN guard:** Detect divergence, reset to DC operating point
- **Shadow subtraction:** Run parallel solver with zero input, subtract to cancel LFO pump
