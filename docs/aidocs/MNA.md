# Modified Nodal Analysis (MNA)

## Purpose

Convert circuit netlist to system of linear equations: `(G + sC) * v = i`.
The MNA system is the foundation for everything downstream: DK kernel, DC OP,
and the codegen pipeline.

## References

- Ho, Ruehli, Brennan. "The Modified Nodal Approach to Network Analysis." IEEE Trans. CAS (1975)
- TU Delft: https://analog-electronics.ewi.tudelft.nl/webbook/SED/

## Source File

All stamping in `crates/melange-solver/src/mna.rs`.

## Matrices

- **G** (NxN): Conductance matrix — resistors, voltage source Norton equivalents, input conductance
- **C** (NxN): Capacitance matrix — capacitors, parasitic junction caps
- **N_v** (MxN): Voltage extraction — maps node voltages to device controlling voltages
- **N_i** (NxM): Current injection — maps device currents back to node equations
- **rhs_const** (N): DC sources (voltage source Norton currents, current sources)

N = number of circuit nodes (excluding ground node 0).
M = number of nonlinear device dimensions.

For augmented MNA (voltage sources, VCVS, ideal transformers), N includes extra
rows/columns for branch currents.

## Stamping Rules

### Resistor (R between nodes i, j)

```
g = 1/R
G[i,i] += g,  G[j,j] += g
G[i,j] -= g,  G[j,i] -= g
```

Nodes are 0-indexed matrix coordinates. Ground node (0 in SPICE) is excluded;
grounded components stamp only the diagonal entry.

### Capacitor (C between nodes i, j)

```
C[i,i] += C,  C[j,j] += C
C[i,j] -= C,  C[j,i] -= C
```

Stamped into the C matrix, NOT G. The DK method combines them: `A = G + (2/T)*C`.

### Inductor (Companion Model — DK/LinearSolver/Codegen)

Trapezoidal companion: inductor becomes conductance `g_eq = T/(2L)`.

```
In A matrix (g_sign = +1):
  stamp_conductance(node_i, node_j, +(T/2)/L)

In A_neg matrix (g_sign = -1):
  stamp_conductance(node_i, node_j, -(T/2)/L)
```

Stamped directly into the discretized matrices during `build_discretized_matrix()`,
not into G or C separately. History current i_hist injected into RHS each sample.

**Limitation**: For large inductors (L > ~1H at audio rates), g_eq ≈ 8e-8 S.
This creates ill-conditioned A matrices (cond ~ 1e9+). See augmented MNA below.

### Inductor (Augmented MNA — Nodal Codegen Path)

The codegen "nodal" routing path uses augmented MNA for inductors (Ho et al.
1975): each inductor winding adds an extra variable (branch current j_L) with
inductance L in the C matrix. Built by `MnaSystem::build_augmented_matrices` in
`crates/melange-solver/src/mna.rs`.

```
For inductor between nodes i, j with inductance L, at augmented index k:

KCL (column k): j_L enters node_i, exits node_j
  G[node_i - 1, k] += 1.0
  G[node_j - 1, k] -= 1.0

KVL (row k): -V_i + V_j + L·dj_L/dt = 0
  G[k, node_i - 1] -= 1.0
  G[k, node_j - 1] += 1.0

Self-inductance:
  C[k, k] = L

For coupled pairs, mutual inductance M = k·√(L1·L2):
  C[k1, k2] = M,  C[k2, k1] = M

For transformer groups, full inductance sub-matrix:
  C[ki, kj] = coupling[i][j] · √(Li · Lj)
```

This gives A[k][k] = 2L/T (large, well-conditioned) instead of T/(2L) (tiny).
A_neg handles all trapezoidal history — no separate inductor history injection needed.

**A_neg zeroing**: Only VS/VCVS/ideal_xfmr rows (n_nodes..n_aug) are zeroed.
Inductor rows (n_aug..n_nodal) keep their values (they have real history).

**DC OP**: The DC OP solver uses its own augmented inductor system with the same
variable ordering. v_node from DC OP includes inductor DC branch currents at
indices n_aug..n_dc, which map directly to the nodal codegen path's
v_prev[n_aug..n_nodal] in the generated state.

### Voltage Source (Augmented MNA)

Each voltage source adds one extra unknown (branch current j_vs) and one extra
equation (KVL constraint). Matrix dimension N grows by 1.

For source V between n_plus and n_minus, at augmented index k:

```
Current injection (column k):
  G[n_plus - 1, k]  += 1.0
  G[n_minus - 1, k] -= 1.0

KVL constraint (row k):
  G[k, n_plus - 1]  += 1.0
  G[k, n_minus - 1] -= 1.0

RHS (rhs_const):
  rhs[k] = V_dc          (algebraic constraint, NOT multiplied by 2)
```

### Current Source

No G matrix modification. RHS injection only:
```
rhs[n_plus - 1]  += I_dc
rhs[n_minus - 1] -= I_dc
```

**Current-source node rows** in `rhs_const` are multiplied by 2.0 for trapezoidal
consistency (see `build_rhs_const` in `crates/melange-solver/src/dk.rs`).
**Voltage-source augmented rows are NOT** multiplied — they are algebraic KVL
constraints, not differential equations, so `rhs[k] = V_dc` is set directly.

### Input Conductance (Thevenin Source)

```
G_in = 1 / INPUT_RESISTANCE     (default: 1.0 S for 1 ohm)
G[input_node, input_node] += G_in
```

**CRITICAL**: Must be stamped BEFORE building DK kernel (`DkKernel::from_mna()`).
The DK kernel bakes G into `S = A^{-1}`, so the input stamp MUST be part of A.

RHS contribution (in solver, not MNA):
```
rhs[input_node] += (V_in[n+1] + V_in[n]) * G_in
```

### Op-Amp (Linear VCCS)

```
Gm = AOL / ROUT     (transconductance)
Go = 1 / ROUT       (output conductance)

G[out, n_plus]  += Gm
G[out, n_minus] -= Gm
G[out, out]     += Go
```

Defaults: AOL = 200000, ROUT = 1 ohm. **Linear only** — does not add to M.

### VCCS (Voltage-Controlled Current Source)

```
I = gm * (V_ctrl_p - V_ctrl_n)

G[out_p, ctrl_p] += gm
G[out_p, ctrl_n] -= gm
G[out_n, ctrl_p] -= gm
G[out_n, ctrl_n] += gm
```

### VCVS (Voltage-Controlled Voltage Source, Augmented)

Adds extra unknown (branch current) at augmented index k:

```
Current injection (column k):
  G[out_p - 1, k] += 1.0
  G[out_n - 1, k] -= 1.0

KVL constraint (row k):
  G[k, out_p - 1]  += 1.0
  G[k, out_n - 1]  -= 1.0
  G[k, ctrl_p - 1] -= gain
  G[k, ctrl_n - 1] += gain
```

### Coupled Inductors (2-Winding Transformer)

Mutual inductance M = k * sqrt(L1 * L2), where k is coupling coefficient.

```
det = L1*L2 - M^2
g_eq_factor = T/2

g_self_1 = g_sign * g_eq_factor * L2 / det
g_self_2 = g_sign * g_eq_factor * L1 / det
g_mutual = g_sign * (-g_eq_factor) * M / det

Stamp self-conductances via stamp_conductance(node_i, node_j, g_self)
Stamp mutual conductances:
  mat[a-1][c-1] += g_mutual
  mat[b-1][d-1] += g_mutual
  mat[a-1][d-1] -= g_mutual
  mat[b-1][c-1] -= g_mutual
```

### Multi-Winding Transformer Group (3+ Windings)

Full NxN inductance matrix inversion:

```
L_mat[i][j] = coupling_matrix[i][j] * sqrt(inductances[i] * inductances[j])
Y_raw = inv(L_mat)                     (via invert_small_matrix)
y_ij = g_sign * (T/2) * Y_raw[i][j]

Stamp all diagonal and off-diagonal entries using
stamp_conductance and stamp_mutual_conductance.
```

### Ideal Transformer (Augmented MNA)

For tightly coupled transformers (k > 0.8). Adds constraint rows.

```
turns_ratio = sqrt(L_secondary / L_primary) * sign(k)

KVL constraint (row k):
  G[k, sec_p - 1]  += 1.0
  G[k, sec_n - 1]  -= 1.0
  G[k, pri_p - 1]  -= turns_ratio
  G[k, pri_n - 1]  += turns_ratio

Current injection (column k, power conservation):
  G[sec_p - 1, k]  += 1.0
  G[sec_n - 1, k]  -= 1.0
  G[pri_p - 1, k]  += turns_ratio
  G[pri_n - 1, k]  -= turns_ratio
```

## Nonlinear Device Matrices (N_v, N_i)

### Convention

- **N_v** (MxN): Row i has +1 at positive terminal, -1 at negative terminal
- **N_i** (NxM): Column i has **-1 at anode** (current extracted), **+1 at cathode** (current injected)
- This "injection convention" makes K = N_v * S * N_i **naturally negative**

### Diode (1D)

```
N_v[idx, anode]   = +1
N_v[idx, cathode] = -1

N_i[anode, idx]   = -1    (current extracted from anode)
N_i[cathode, idx] = +1    (current injected into cathode)
```

### BJT (2D: Ic at idx, Ib at idx+1)

```
N_v[idx, base]      = +1    (Vbe = V_base - V_emitter)
N_v[idx, emitter]   = -1
N_v[idx+1, base]    = +1    (Vbc = V_base - V_collector)
N_v[idx+1, collector] = -1

N_i[collector, idx]  = -1   (Ic extracted from collector)
N_i[emitter, idx]    = +1   (Ic injected into emitter)
N_i[base, idx+1]     = -1   (Ib extracted from base)
N_i[emitter, idx+1]  = +1   (Ib injected into emitter)
```

**Forward-active BJT variant** (`NonlinearDeviceType::BjtForwardActive` in
`crates/melange-solver/src/mna.rs`): when DC OP detects strong forward bias,
the BJT is given **dimension 1** instead of 2. Only the Vbe→Ic NR row is
kept; Ib is computed algebraically from Ic via `Ib = Ic / β_F` after the NR
converges. The Vbc junction is not in the NR system.

**Fully-linearized BJT** (`MnaSystem::linearized_bjts` field): when DC OP
detects the BJT is in the forward-active linear region, it can be removed
from the nonlinear system entirely. After DC OP, the small-signal `g_m`,
`g_π`, and `r_o` are stamped into G as a four-terminal small-signal model,
and the BJT is dropped from `nonlinear_devices`. M decreases by 2 per
linearized BJT. Grep `linearized_bjts` and `LinearizedBjtInfo` in `mna.rs`
for the dispatch.

### JFET (2D: Id at idx, Ig at idx+1)

```
N_v[idx, drain]    = +1     (Vds = V_drain - V_source)
N_v[idx, source]   = -1
N_v[idx+1, gate]   = +1     (Vgs = V_gate - V_source)
N_v[idx+1, source] = -1

N_i[drain, idx]    = -1     (Id extracted from drain)
N_i[source, idx]   = +1     (Id injected into source)
N_i[gate, idx+1]   = -1     (Ig extracted from gate)
N_i[source, idx+1] = +1     (Ig injected into source)
```

### MOSFET (2D: Id at idx, Ig at idx+1)

Same N_v/N_i pattern as JFET. Nodes: [drain, gate, source, bulk].

### Tube/Triode (2D: Ip at idx, Ig at idx+1)

```
N_v[idx, grid]     = +1     (Vgk = V_grid - V_cathode)
N_v[idx, cathode]  = -1
N_v[idx+1, plate]  = +1     (Vpk = V_plate - V_cathode)
N_v[idx+1, cathode] = -1

N_i[plate, idx]    = -1     (Ip extracted from plate)
N_i[cathode, idx]  = +1     (Ip injected into cathode)
N_i[grid, idx+1]   = -1     (Ig extracted from grid)
N_i[cathode, idx+1] = +1    (Ig injected into cathode)
```

## Parasitic Cap Auto-Insertion

Called via `MnaSystem::add_parasitic_caps()` when nonlinear circuit has no caps.
Stamps `PARASITIC_CAP = 10e-12 F` across each device junction:

| Device | Junctions |
|--------|-----------|
| Diode | anode-cathode |
| BJT | base-emitter + base-collector |
| JFET | gate-source + gate-drain |
| MOSFET | gate-source + gate-drain |
| Tube | grid-cathode + plate-cathode |

Caps across junctions (not to ground). Emits `log::warn!` when triggered.

## Helper Functions

```
stamp_conductance_to_ground(mat, i, j, g):
  Both non-ground: stamp 4 entries (symmetric)
  One grounded: stamp diagonal only
  Both grounded: no-op

stamp_mutual_conductance(mat, a, b, c, d, g):
  mat[a][c] += g, mat[b][d] += g
  mat[a][d] -= g, mat[b][c] -= g
  (handles ground nodes)

inject_rhs_current(rhs, node, current):
  if node > 0: rhs[node-1] += current
```

## Verification

- G should be symmetric positive semi-definite
- Diagonal entries >= sum of off-diagonal entries (diagonal dominance)
- Input node G[i,i] must include source conductance
- C should be symmetric positive semi-definite
- For augmented rows: symmetry in coupling entries

## Common Bugs

| Symptom | Cause | Fix |
|---------|-------|-----|
| Wrong gain | Input conductance not stamped | Add G[in,in] += G_in before kernel |
| Negative resistance | Sign error in off-diagonals | Check stamp_conductance signs |
| Wrong frequency | Stamping C into G | Use separate C matrix |
| DC offset | Ground node in matrix | Node 0 excluded (1-indexed to 0-indexed) |
| Singular A | Missing parasitic caps | Call add_parasitic_caps() |
| Augmented row issues | RHS not multiplied by 2 | Check trapezoidal scaling |
