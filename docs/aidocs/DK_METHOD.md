# Discrete Kirchhoff (DK) Method

## Purpose
Reduce N-node linear system to M-dimensional nonlinear system for real-time solving.

## References
- Hack Audio: https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/
- Yeh, Abel, Smith. "Simplified, physically-informed models of distortion and overdrive guitar effects pedals." (2007)
- Yeh, David. "Digital Implementation of Musical Distortion Circuits by Analysis and Simulation" (PhD thesis, 2009)

## Key Matrices

### A Matrix (System Matrix)
```
A = G + alpha*C    where alpha = 2/T (trapezoidal)
A_neg = alpha*C - G
```

### S Matrix (Inverse System Matrix)
```
S = A^{-1}  (NxN inverse)
```
Used for: `v_pred = S * rhs`

Properties:
- S*A = I (identity, within numerical tolerance)
- S[i][j] has units of resistance (ohms)

### K Matrix (Nonlinear Kernel)
```
K = N_v * S * N_i  (MxM)   [NO NEGATION]
```

No negation needed. N_i uses the "current injection" convention:
- N_i[anode] = -1 (current extracted from anode)
- N_i[cathode] = +1 (current injected into cathode)
- K is naturally negative for stable circuits, providing correct negative feedback

### N_v, N_i (Selection Matrices)
- **N_v**: Extracts controlling voltages from node voltages (MxN)
  - Row i has +1 at anode node, -1 at cathode node for device i
- **N_i**: Injects nonlinear currents into nodes (NxM)
  - Column i has -1 at anode, +1 at cathode for device i
  - Uses injection convention (positive = current INTO node)

### Device Dimension Assignment
Devices occupy M dimensions in netlist order:
- Diode: 1 dimension (Vd -> Id)
- BJT: 2 dimensions (Vbe -> Ic, Vbc -> Ib)
- JFET: 1 dimension (Vgs -> Id)
- MOSFET: 1 dimension (Vgs -> Id)

## Algorithm

### Step 1: Build RHS
```
rhs = A_neg * v_prev + N_i * i_nl_prev + 2 * V_in * G_in + rhs_const
```
Note: A_neg already contains capacitor history via alpha*C. Do NOT add separate cap_history.

### Step 2: Linear Prediction
```
v_pred = S * rhs          // Linear solution
p = N_v * v_pred          // Controlling voltages for nonlinear devices
```

### Step 3: Nonlinear Solve (Newton-Raphson)

#### Residual and Jacobian:
```
f(i) = i - i_dev(p + K*i)
J[i][j] = delta_ij - sum_k J_dev[i][k] * K[k][j]
```
where J_dev is the block-diagonal device Jacobian:
- Diode block: 1x1 (conductance g_d)
- BJT block: 2x2 (dIc/dVbe, dIc/dVbc, dIb/dVbe, dIb/dVbc)

K is naturally negative, so J > 0 (always convergent).

### Step 4: Final Voltage
```
v = v_pred + S * N_i * (i_nl - i_nl_prev)
```

## Sign Convention Summary

| Component | Convention | Sign in K |
|-----------|-----------|-----------|
| N_i | Current injection (+ = into node) | N/A |
| N_i[anode] | -1 (current extracted) | Makes K negative |
| N_i[cathode] | +1 (current injected) | Makes K negative |
| K = N_v*S*N_i | Naturally negative | Correct feedback |

## Verification
- S*A ~ I (within 1e-12 relative tolerance)
- K = N_v*S*N_i (no negation!)
- K[i][i] < 0 for stable circuits (negative feedback)
- S[i][i] > 0 (positive diagonal)
- |S[i][j]| < 1e6 (reasonable magnitude for audio)

## Common Bugs
1. **Extra negation of K** -> NR diverges (positive feedback)
2. **Double-counting history** -> DC offset, instability
3. **Wrong N_i ground-reference sign** -> Wrong K sign for grounded devices
4. **Missing correction term** -> Wrong output amplitude

## References
- Hack Audio Tutorial (Chapters 5-8): https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/
- Yeh & Smith. "Simulating guitar distortion circuits using wave digital and nonlinear state-space formulations." (2008)
