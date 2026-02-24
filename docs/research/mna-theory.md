# Modified Nodal Analysis (MNA) Theory

**Research Document for AI Agent Consumption**

Modified Nodal Analysis is the foundation of modern circuit simulation. This document provides the theoretical underpinnings for implementing MNA-based solvers in real-time audio DSP contexts.

---

## Table of Contents

1. [Classical Nodal Analysis vs MNA](#1-classical-nodal-analysis-vs-mna)
2. [MNA Matrix Structure](#2-mna-matrix-structure)
3. [Component Stamping Patterns](#3-component-stamping-patterns)
4. [Extended MNA for Voltage Sources](#4-extended-mna-for-voltage-sources)
5. [Multi-Dimensional Device Handling](#5-multi-dimensional-device-handling)
6. [Numerical Properties](#6-numerical-properties)
7. [Common Implementation Pitfalls](#7-common-implementation-pitfalls)

---

## 1. Classical Nodal Analysis vs MNA

### 1.1 Classical Nodal Analysis (KCL-Based)

**Foundation**: Kirchhoff's Current Law (KCL) — the algebraic sum of currents entering any node equals zero.

For a circuit with $n$ non-ground nodes, classical nodal analysis formulates:

$$\mathbf{G} \mathbf{v} = \mathbf{i}_{src}$$

Where:
- $\mathbf{G}$: $n \times n$ conductance matrix
- $\mathbf{v}$: $n \times 1$ node voltage vector (relative to ground)
- $\mathbf{i}_{src}$: $n \times 1$ current source vector

**The Problem**: Classical analysis only works for circuits where every element can be expressed as a conductance (admittance) between two nodes.

**Elements that BREAK classical nodal analysis:**

| Element | Issue | Mathematical Manifestation |
|---------|-------|---------------------------|
| Independent voltage source | Infinite conductance | Cannot express $I = GV$ when $G \to \infty$ |
| Current-controlled source | Requires current as variable | Current through element is not a node voltage difference |
| Zero-resistance wire | Indefinite conductance | $G = \infty$ creates numerical singularity |
| Ideal transformer | Coupled voltages | Violates simple nodal admittance formulation |

### 1.2 The MNA Solution

**Key Insight**: Augment the unknown vector with additional variables for problematic elements.

For each independent voltage source, add its current as an unknown:

$$\mathbf{v}_{MNA} = \underbrace{[v_1, v_2, \ldots, v_n}_{\text{node voltages}} | \underbrace{i_{V1}, i_{V2}, \ldots, i_{Vm}}_{\text{voltage source currents}}]^T$$

The system becomes:

$$\underbrace{\begin{bmatrix} \mathbf{G}_n & \mathbf{B} \\ \mathbf{B}^T & \mathbf{0} \end{bmatrix}}_{\mathbf{Y}} \underbrace{\begin{bmatrix} \mathbf{v}_n \\ \mathbf{i}_v \end{bmatrix}}_{\mathbf{x}} = \underbrace{\begin{bmatrix} \mathbf{i}_{src} \\ \mathbf{v}_{src} \end{bmatrix}}_{\mathbf{b}}$$

Where:
- $\mathbf{G}_n$: $n \times n$ nodal conductance matrix
- $\mathbf{B}$: $n \times m$ voltage source connection matrix
- $\mathbf{B}^T$: $m \times n$ KVL constraint matrix
- $\mathbf{0}$: $m \times m$ zero block (voltage sources have no self-conductance)

### 1.3 Comparison Summary

| Aspect | Classical Nodal | Modified Nodal (MNA) |
|--------|-----------------|---------------------|
| Unknowns | $n$ node voltages | $n$ node voltages + $m$ source currents |
| Matrix size | $n \times n$ | $(n+m) \times (n+m)$ |
| Voltage sources | Cannot handle directly | Full support via augmentation |
| Matrix structure | Symmetric, positive definite | Saddle-point structure (indefinite) |
| Sparsity | High ($O(n)$ nonzeros) | Still sparse, but less so |
| Implementation | Simpler | More complex but general |

---

## 2. MNA Matrix Structure

### 2.1 The Time-Domain MNA Equation

For transient analysis, the complete MNA formulation includes reactive elements:

$$\mathbf{G}\mathbf{v}(t) + \mathbf{C}\frac{d\mathbf{v}(t)}{dt} = \mathbf{s}(t)$$

**Matrix dimensions for a circuit with:**
- $n$ non-ground nodes
- $m$ independent voltage sources
- $p$ inductors

| Matrix | Dimension | Description |
|--------|-----------|-------------|
| $\mathbf{G}$ | $(n+m+p) \times (n+m+p)$ | Conductance matrix (resistors, companion models) |
| $\mathbf{C}$ | $(n+m+p) \times (n+m+p)$ | Capacitance/inductance matrix |
| $\mathbf{v}(t)$ | $(n+m+p) \times 1$ | Unknown vector (voltages + currents) |
| $\mathbf{s}(t)$ | $(n+m+p) \times 1$ | Source vector |

### 2.2 Block Structure Analysis

Partition the unknown vector as $\mathbf{v} = [\mathbf{v}_n^T, \mathbf{i}_v^T, \mathbf{i}_l^T]^T$ (node voltages, voltage source currents, inductor currents).

The block structure becomes:

$$\begin{bmatrix} 
\mathbf{G}_{nn} & \mathbf{B}_v & \mathbf{B}_l \\
\mathbf{B}_v^T & \mathbf{0} & \mathbf{0} \\
\mathbf{B}_l^T & \mathbf{0} & \mathbf{0}
\end{bmatrix}
\begin{bmatrix} \mathbf{v}_n \\ \mathbf{i}_v \\ \mathbf{i}_l \end{bmatrix} +
\begin{bmatrix} 
\mathbf{C}_{nn} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{L}
\end{bmatrix}
\frac{d}{dt}\begin{bmatrix} \mathbf{v}_n \\ \mathbf{i}_v \\ \mathbf{i}_l \end{bmatrix} =
\begin{bmatrix} \mathbf{i}_{src} \\ \mathbf{v}_{src} \\ \mathbf{0} \end{bmatrix}$$

**Key observations:**

1. **Saddle-point structure**: The zero blocks create a saddle-point system, which is indefinite (not positive definite)
2. **Decoupled dynamics**: $\mathbf{C}_{nn}$ contains capacitor stamps; $\mathbf{L}$ contains inductor flux linkages
3. **Constraint rows**: Voltage source and inductor rows enforce KVL constraints

### 2.3 The G Matrix (Conductance)

$$\mathbf{G} = \begin{bmatrix} 
G_{11} & G_{12} & \cdots & G_{1n} & B_{1,1} & \cdots & B_{1,m} \\
G_{21} & G_{22} & \cdots & G_{2n} & B_{2,1} & \cdots & B_{2,m} \\
\vdots & \vdots & \ddots & \vdots & \vdots & \ddots & \vdots \\
G_{n1} & G_{n2} & \cdots & G_{nn} & B_{n,1} & \cdots & B_{n,m} \\
B_{1,1} & B_{2,1} & \cdots & B_{n,1} & 0 & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots & \ddots & \vdots \\
B_{1,m} & B_{2,m} & \cdots & B_{n,m} & 0 & \cdots & 0
\end{bmatrix}$$

**Properties:**
- Symmetric for passive circuits: $\mathbf{G} = \mathbf{G}^T$
- Diagonal dominance: $|G_{ii}| \geq \sum_{j \neq i} |G_{ij}|$ (with equality for nodes connected only to capacitors)
- Positive semi-definite for passive circuits; positive definite if no floating nodes

### 2.4 The C Matrix (Capacitance)

$$\mathbf{C} = \begin{bmatrix} 
C_{11} & C_{12} & \cdots & C_{1n} & 0 & \cdots & 0 \\
C_{21} & C_{22} & \cdots & C_{2n} & 0 & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots & \ddots & \vdots \\
C_{n1} & C_{n2} & \cdots & C_{nn} & 0 & \cdots & 0 \\
0 & 0 & \cdots & 0 & 0 & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & 0 & 0 & \cdots & 0
\end{bmatrix}$$

**Properties:**
- Only nonzero in the upper-left $n \times n$ block (node voltages only)
- Symmetric: $\mathbf{C} = \mathbf{C}^T$
- Positive semi-definite (positive definite if no capacitor loops)
- Zero rows/columns for voltage source and inductor current variables

### 2.5 The B Matrix (Voltage Source Connections)

The $\mathbf{B}$ matrix encodes voltage source topology:

$$B_{i,j} = \begin{cases}
+1 & \text{if source } j \text{ enters node } i \\
-1 & \text{if source } j \text{ exits node } i \\
0 & \text{otherwise}
\end{cases}$$

**Example:** Three-node circuit with voltage source $V_1$ from node 2 to node 1:

$$\mathbf{B} = \begin{bmatrix} -1 \\ +1 \\ 0 \end{bmatrix}$$

The constraint row enforces: $-v_1 + v_2 = V_1$ or equivalently $v_2 - v_1 = V_1$

---

## 3. Component Stamping Patterns

**Stamping**: The process of adding a component's contribution to the MNA matrices.

### 3.1 Resistor (R between nodes i and j)

**Stamp:**
```
      i       j
   ┌─────┬─────┐
i  │ 1/R │-1/R │
   ├─────┼─────┤
j  │-1/R │ 1/R │
   └─────┴─────┘
```

**Matrix entries:**
- $G[i][i] += 1/R$
- $G[j][j] += 1/R$
- $G[i][j] -= 1/R$
- $G[j][i] -= 1/R$

**When connected to ground (j = 0):**
- $G[i][i] += 1/R$ (only diagonal entry)

### 3.2 Capacitor (C between nodes i and j)

**Stamp (in C matrix):**
```
      i       j
   ┌─────┬─────┐
i  │  C  │ -C  │
   ├─────┼─────┤
j  │ -C  │  C  │
   └─────┴─────┘
```

**Matrix entries:**
- $C[i][i] += C$
- $C[j][j] += C$
- $C[i][j] -= C$
- $C[j][i] -= C$

### 3.3 Inductor (L between nodes i and j)

**MNA treatment**: Inductors add an auxiliary current variable $i_L$.

**Stamp:**
```
      i    j    k(i_L)
   ┌────┬────┬────┐
i  │  0 │  0 │  1 │
   ├────┼────┼────┤
j  │  0 │  0 │ -1 │
   ├────┼────┼────┤
k  │  1 │ -1 │  0 │  ← KVL: v_i - v_j = L(di_L/dt)
   └────┴────┴────┘
```

**G matrix entries:**
- $G[i][k] = 1$
- $G[j][k] = -1$
- $G[k][i] = 1$
- $G[k][j] = -1$

**C matrix entries:**
- $C[k][k] = -L$ (note sign for consistent formulation)

### 3.4 Independent Current Source (I into node i, out of node j)

**Stamp:** Only affects RHS vector

$$\mathbf{s}[i] += I$$
$$\mathbf{s}[j] -= I$$

### 3.5 Voltage-Controlled Current Source (VCCS)

Current from node $i$ to node $j$, controlled by voltage between nodes $k$ and $l$:

$$I_{ij} = g_m (v_k - v_l)$$

**Stamp:**
```
      k       l
   ┌───────┬───────┐
i  │  g_m  │ -g_m  │
   ├───────┼───────┤
j  │ -g_m  │  g_m  │
   └───────┴───────┘
```

**Matrix entries:**
- $G[i][k] += g_m$
- $G[i][l] -= g_m$
- $G[j][k] -= g_m$
- $G[j][l] += g_m$

**Note**: VCCS breaks symmetry unless carefully placed.

### 3.6 Voltage-Controlled Voltage Source (VCVS)

$$v_i - v_j = A(v_k - v_l)$$

**Stamp** (adds row $m$ for source current):
```
      i    j    k     l     m
   ┌────┬────┬─────┬─────┬────┐
i  │  0 │  0 │   0 │   0 │  1 │
   ├────┼────┼─────┼─────┼────┤
j  │  0 │  0 │   0 │   0 │ -1 │
   ├────┼────┼─────┼─────┼────┤
m  │  1 │ -1 │  -A │   A │  0 │  ← KVL constraint
   └────┴────┴─────┴─────┴────┘
```

### 3.7 Ideal Op-Amp

Virtual short constraint: $v_p = v_n$ (non-inverting input = inverting input)

**Stamp** (adds row for output current, assumes output at node $o$):
```
      p    n    o    m
   ┌────┬────┬────┬────┐
m  │  1 │ -1 │  0 │  0 │  = 0 (virtual short)
   └────┴────┴────┴────┘
```

---

## 4. Extended MNA for Voltage Sources

### 4.1 The Augmentation Strategy

Each independent voltage source requires:
1. One additional unknown (the source current)
2. One additional equation (KVL constraint)

**Why this works:**
- We add as many unknowns as equations — system remains determined
- The current through the voltage source becomes available for other calculations
- Zero-resistance paths are handled without numerical issues

### 4.2 Complete Voltage Source Stamp

For voltage source $V_k$ connected between nodes $i$ (positive) and $j$ (negative), with source current index $m$:

**G matrix entries:**
```
G[i][m] =  1   // Current enters node i
G[j][m] = -1   // Current exits node j
G[m][i] =  1   // KVL: +coefficient for node i
G[m][j] = -1   // KVL: -coefficient for node j
```

**RHS entry:**
```
s[m] = V_k   // The voltage source value
```

### 4.3 Matrix Example: Simple RC with Voltage Source

**Circuit:**
```
    R1=1k     C1=1µF
0 ---[===]----+----+--- Vsrc = 1V
              |    |
             R2    |
            2k      |
              |    |
             GND   |
                   |
                  GND
```

**Nodes:** 1 (between R1, C1, R2), 2 (source positive terminal)  
**Ground:** 0  
**Voltage source:** Vsrc between node 2 and ground (index 3 in extended system)

**G matrix (3×3):**
```
          node1   node2   i_Vsrc
        ┌────────┬───────┬─────┐
node1   │1/R1+1/R│  -1/R1│  0  │
        ├────────┼───────┼─────┤
node2   │ -1/R1  │ 1/R1  │  1  │
        ├────────┼───────┼─────┤
i_Vsrc  │   0    │   1   │  0  │  = 1V
        └────────┴───────┴─────┘
```

**C matrix (3×3):**
```
          node1  node2  i_Vsrc
        ┌─────┬─────┬─────┐
node1   │ C1  │ -C1 │  0  │
        ├─────┼─────┼─────┤
node2   │ -C1 │ C1  │  0  │
        ├─────┼─────┼─────┤
i_Vsrc  │  0  │  0  │  0  │
        └─────┴─────┴─────┘
```

### 4.4 Numerical Considerations

**The zero diagonal block creates a saddle-point system:**

$$\begin{bmatrix} \mathbf{G}_{nn} & \mathbf{B} \\ \mathbf{B}^T & \mathbf{0} \end{bmatrix}$$

**Properties:**
- Not positive definite (indefinite)
- LU factorization with partial pivoting is stable
- Banded structure is preserved if voltage sources connect nearby nodes
- Condition number may be worse than pure nodal analysis

---

## 5. Multi-Dimensional Device Handling

### 5.1 The Challenge

Multi-terminal devices (BJT, MOSFET, tubes) have:
- Multiple controlling voltages
- Multiple terminal currents (interrelated by device equations)
- Nonlinear constitutive relations

### 5.2 The Separation Principle

**MNA handles the linear network**  
**Nonlinear devices are treated as current sources**

For the linear MNA system:
$$\mathbf{G}\mathbf{v} + \mathbf{C}\frac{d\mathbf{v}}{dt} = \mathbf{s}(t) + \mathbf{N}_i \mathbf{i}_{nl}$$

Where:
- $\mathbf{i}_{nl}$: Vector of nonlinear device currents (dimension $M$)
- $\mathbf{N}_i$: $N \times M$ injection matrix (maps device currents to nodes)

### 5.3 The N_v and N_i Matrices

**$\mathbf{N}_v$ (M×N)**: Extracts controlling voltages from node voltages

For device $d$ with controlling voltage $v_{ctrl,d} = v_{pos} - v_{neg}$:
$$(\mathbf{N}_v)_{d, pos} = +1, \quad (\mathbf{N}_v)_{d, neg} = -1$$

**$\mathbf{N}_i$ (N×M)**: Injects device currents into circuit nodes

For device $d$ injecting current $i_d$ into node $pos$ and extracting from node $neg$:
$$(\mathbf{N}_i)_{pos, d} = +1, \quad (\mathbf{N}_i)_{neg, d} = -1$$

**Relationship**: Often $\mathbf{N}_i = \mathbf{N}_v^T$ when currents enter/exit the same terminals used for voltage control.

### 5.4 NPN BJT Example

**Ebers-Moll model** requires two controlling voltages:
- $V_{BE} = v_B - v_E$ (base-emitter)
- $V_{BC} = v_B - v_C$ (base-collector)

**For a BJT with terminals C=node 3, B=node 2, E=node 1:**

$$\mathbf{N}_v = \begin{bmatrix}
0 & 1 & -1 & 0 & \cdots \\
0 & 1 & 0 & -1 & \cdots
\end{bmatrix}
\begin{matrix}
\leftarrow V_{BE} = v_2 - v_1 \\
\leftarrow V_{BC} = v_2 - v_3
\end{matrix}$$

**Nonlinear currents:**
- $I_C$ (collector current, entering collector)
- $I_E$ (emitter current, exiting emitter)

$$\mathbf{N}_i = \begin{bmatrix}
0 & 0 & 1 \\
0 & 0 & 0 \\
1 & 0 & 0 \\
0 & 1 & 0 \\
\vdots & \vdots & \vdots
\end{bmatrix}
\begin{matrix}
\text{node 1 (E)} \\
\text{node 2 (B)} \\
\text{node 3 (C)} \\
\text{node 4}
\end{matrix}$$

### 5.5 Triode Vacuum Tube Example

**Terminals:** Plate (P), Grid (G), Cathode (K)

**Controlling voltages:**
- $V_{GK} = v_G - v_K$ (grid-cathode)
- $V_{PK} = v_P - v_K$ (plate-cathode)

**Currents:**
- $I_P$ (plate current, Child-Langmuir law + grid influence)
- $I_G$ (grid current, usually negligible for negative grid)

$$\mathbf{N}_v = \begin{bmatrix}
0 & 1 & -1 & 0 & \cdots \\
1 & 0 & -1 & 0 & \cdots
\end{bmatrix}
\begin{matrix}
\leftarrow V_{GK} \\
\leftarrow V_{PK}
\end{matrix}$$

### 5.6 Complete System View

The full time-domain system:

$$\mathbf{G}\mathbf{v} + \mathbf{C}\frac{d\mathbf{v}}{dt} = \mathbf{s}(t) + \mathbf{N}_i \mathbf{i}_{nl}(\mathbf{N}_v \mathbf{v})$$

After companion model discretization (trapezoidal):

$$\mathbf{A}\mathbf{v}_n = \mathbf{A}_{neg}\mathbf{v}_{n-1} + \mathbf{s}_n + \mathbf{N}_i \mathbf{i}_{nl,n-1} + 2\mathbf{w}$$

Where nonlinear currents at time $n$ are solved iteratively.

---

## 6. Numerical Properties

### 6.1 Symmetry

**Theorem**: The MNA matrix is symmetric if and only if:
1. All controlled sources are VCCS
2. The VCCS connections are symmetric
3. No CCCS, CCVS, or VCVS

**For passive circuits** (resistors, capacitors, inductors, independent sources):
$$\mathbf{G} = \mathbf{G}^T, \quad \mathbf{C} = \mathbf{C}^T$$

**Symmetry breakdown** occurs with:
- Asymmetric VCCS placement
- Current-controlled sources
- Active devices in small-signal linearization

### 6.2 Positive Definiteness

**Definition**: Matrix $\mathbf{M}$ is positive definite if $\mathbf{x}^T \mathbf{M} \mathbf{x} > 0$ for all $\mathbf{x} \neq \mathbf{0}$.

**Classical nodal G matrix:**
- Positive definite if all resistances are positive (passive)
- Ensures unique solution for DC operating point
- Guarantees stable transient response

**Full MNA matrix:**
- **NOT positive definite** due to zero diagonal blocks
- Is indefinite (saddle-point structure)
- LU decomposition required instead of Cholesky

**DC conductance submatrix $\mathbf{G}_{nn}$:**
- Positive definite if no floating nodes exist
- Every node must have DC path to ground
- Check: diagonal entries must be positive, matrix diagonally dominant

### 6.3 Condition Number

$$\kappa(\mathbf{G}) = \|\mathbf{G}\| \cdot \|\mathbf{G}^{-1}\|$$

**Interpretation:**
- $\kappa = 1$: Perfect conditioning (identity matrix)
- $\kappa \gg 1$: Ill-conditioned, sensitive to roundoff
- $\kappa \to \infty$: Singular matrix

**Causes of ill-conditioning in MNA:**

| Cause | Mechanism | Solution |
|-------|-----------|----------|
| Wide dynamic range | $R_{max}/R_{min} > 10^{12}$ | Rescale, use higher precision |
| Floating nodes | No DC path to ground | Add large parallel resistors (GMIN) |
| Voltage source loops | KVL redundancy | Check topology, remove redundant source |
| Near-singular C | Capacitor loops | Use charge-based formulation |

**Practical limits for f64:**
- $\kappa < 10^{12}$: Generally safe
- $10^{12} < \kappa < 10^{15}$: Use with caution, check results
- $\kappa > 10^{15}$: Likely numerical issues

### 6.4 Sparsity Patterns

**Audio circuits exhibit high sparsity:**
- Typical node degree: 2-5 connections
- Nonzeros in G: $O(n)$ not $O(n^2)$
- For $n=50$ nodes: ~200 nonzeros (< 10% density)

**Common patterns:**

**Ladder filter** (tridiagonal):
```
┌──┬──┬  ┬  ┐
│× │× │  │  │
├──┼──┼──┼──┤
│× │× │× │  │
├──┼──┼──┼──┤
│  │× │× │× │
├──┼──┼──┼──┤
│  │  │× │× │
└──┴──┴──┴──┘
```

**Star topology** (high fan-out):
```
┌──┬──┬──┬──┐
│× │× │× │× │
├──┼──┼──┼──┤
│× │× │  │  │
├──┼──┼──┼──┤
│× │  │× │  │
├──┼──┼──┼──┤
│× │  │  │× │
└──┴──┴──┴──┘
```

### 6.5 Fill-in During Factorization

**Problem**: LU or Cholesky factorization can introduce fill-in (nonzeros where zeros existed).

**Example**: Tridiagonal matrix has $O(n)$ nonzeros; LU factors may have $O(n)$ nonzeros (no fill-in for tridiagonal).

**For general sparse matrices:**
- Use symbolic factorization to predict fill-in
- Consider reordering (AMD, nested dissection) to minimize fill
- For audio circuits ($n < 100$), dense factorization is often acceptable

---

## 7. Common Implementation Pitfalls

### 7.1 Sign Errors

**Most common bug in MNA implementations.**

**Resistor stamp off-diagonals:**
```rust
// WRONG - missing negative
G[i][j] += g;  // Should be -=
G[j][i] += g;  // Should be -=

// CORRECT
G[i][i] += g;
G[j][j] += g;
G[i][j] -= g;
G[j][i] -= g;
```

**Voltage source stamp:**
```rust
// WRONG - inconsistent signs
G[i][k] = 1.0;   // Current enters node i
G[k][i] = -1.0;  // Constraint coefficient

// CORRECT
G[i][k] = 1.0;   // Current enters node i
G[j][k] = -1.0;  // Current exits node j
G[k][i] = 1.0;   // KVL: +vi
G[k][j] = -1.0;  // KVL: -vj
```

**RHS sign for current sources:**
```rust
// Current entering node i, exiting node j
rhs[i] += I;  // Positive into node
rhs[j] -= I;  // Negative (exiting)
```

### 7.2 Ground Node Handling

**Problem**: Including ground (node 0) in the matrix creates a singular system.

**Solution**: 
1. Number nodes starting from 1 (ground = 0)
2. Omit ground from unknown vector
3. Map circuit node indices to matrix indices: `idx = node - 1`

```rust
fn node_to_index(node: usize) -> Option<usize> {
    if node == 0 { None }      // Ground - no matrix entry
    else { Some(node - 1) }    // Shift by 1
}

// Stamping with ground check
if let Some(i) = node_to_index(node_i) {
    if let Some(j) = node_to_index(node_j) {
        // Both non-ground: full 4-entry stamp
        G[i][i] += g;
        G[j][j] += g;
        G[i][j] -= g;
        G[j][i] -= g;
    } else {
        // node_j is ground: only diagonal
        G[i][i] += g;
    }
} // if both are None (shouldn't happen), nothing to stamp
```

### 7.3 Floating Nodes

**Definition**: A node with no DC path to ground.

**Detection**: Diagonal entry of G is zero (or very small):
```rust
for i in 0..n {
    if G[i][i].abs() < 1e-12 {
        panic!("Floating node detected at index {}", i);
    }
}
```

**Common causes:**
- Capacitor-only connections to ground
- Inductor series combinations without resistance
- Unintended schematic errors

**Fix**: Add large resistor (GMIN, e.g., $10^{-12}$ S) from floating node to ground.

### 7.4 Unit Consistency

**SPICE uses base units:**
| Quantity | Unit | Common Mistake |
|----------|------|----------------|
| Resistance | Ohms | Using kΩ directly (4.7 instead of 4700) |
| Capacitance | Farads | µF not converted ($10\mu F \to 10e-6$) |
| Inductance | Henrys | mH not converted ($10mH \to 10e-3$) |
| Frequency | Hz | Confusion with rad/s |
| Time | Seconds | ms/µs confusion |

**Verification:**
```rust
// Time constant check: RC should be in seconds
let rc = r * c;  // Ohms * Farads = seconds
assert!(rc > 0.0 && rc < 1.0, "RC = {}s seems wrong", rc);

// Corner frequency: 1/(2πRC) in Hz
let fc = 1.0 / (2.0 * PI * rc);
```

### 7.5 Capacitor Loops

**Problem**: Two capacitors in parallel create a loop with no DC path.

**Effect**: C matrix becomes singular (null space exists).

**Solution**: 
- Check for parallel capacitors and merge them
- Add small parallel resistance if modeling loss
- Use charge-based formulation for ideal capacitors

### 7.6 Voltage Source Loops

**Problem**: Two voltage sources in parallel with different values violate KVL.

**Effect**: System is inconsistent (no solution).

**Detection**: Row of B matrix becomes linearly dependent.

### 7.7 Inductor Cut-Sets

**Problem**: Cut-set of inductors (inductors sharing both terminals) creates infinite current.

**Effect**: G matrix singular for DC analysis.

### 7.8 Discretization Errors

**Trapezoidal rule companion conductance:**
```rust
// WRONG - forgetting factor of 2
let gc = c / T;  // Backward Euler value

// CORRECT
let gc = 2.0 * c / T;  // Trapezoidal
```

**History term update:**
```rust
// WRONG - sign error
I_hist = gc * v_prev - i_prev;

// CORRECT
I_hist = gc * v_prev + i_prev;
```

### 7.9 Numerical Overflow

**Problem**: Large $g_c = 2C/T$ for large C or small T.

**Check:**
```rust
let gc = 2.0 * c / T;
if !gc.is_finite() {
    panic!("g_c overflow: C={}, T={}", c, T);
}
```

**Mitigation:**
- Use f64 (not f32) for all MNA calculations
- Consider sub-stepping for very stiff circuits
- Use backward Euler for initial timesteps if needed

### 7.10 Matrix Factorization Cache Invalidation

**Problem**: Reusing factorization after G matrix changes.

**When to refactor:**
- Sample rate change
- Potentiometer adjustment (variable resistance)
- Temperature-dependent model updates

**When to reuse:**
- Same G, different RHS (multiple solves with same topology)
- Time-stepping with fixed G (transient analysis)

---

## Appendix A: Quick Reference Card

### Matrix Dimensions

| Circuit Element | G Entries | C Entries | Unknowns Added |
|-----------------|-----------|-----------|----------------|
| Resistor | 4 (or 1 to ground) | 0 | 0 |
| Capacitor | 0 | 4 (or 1 to ground) | 0 |
| Inductor | 4 | 1 | 1 (current) |
| Voltage Source | 4 | 0 | 1 (current) |
| Current Source | 0 | 0 | 0 |
| VCCS | 4 | 0 | 0 |
| VCVS | 4 | 0 | 1 (current) |
| Nonlinear device | via N_i | 0 | 0 (handled externally) |

### Stamp Template

```rust
/// Generic stamp function pattern
fn stamp_conductance(G: &mut [Vec<f64>], i: usize, j: usize, g: f64) {
    if i == j {
        // Connected to ground
        G[i][i] += g;
    } else {
        G[i][i] += g;
        G[j][j] += g;
        G[i][j] -= g;
        G[j][i] -= g;
    }
}
```

### Verification Checklist

- [ ] All diagonal entries G[i][i] > 0
- [ ] Off-diagonal entries G[i][j] ≤ 0 for passive circuits
- [ ] Row sums approximately zero (KCL) for passive nodes
- [ ] Symmetry: G[i][j] == G[j][i] (if no VCCS)
- [ ] No NaN or Inf values
- [ ] Condition number < 1e12
- [ ] DC operating point converges
- [ ] Transient response matches SPICE

---

## References

1. Ho, C.W., Ruehli, A.E., & Brennan, P.A. (1975). "The Modified Nodal Approach to Network Analysis." *IEEE Transactions on Circuits and Systems*, 22(6), 504-509.

2. Pillage, L.T., Rohrer, R.A., & Visweswariah, C. (1995). *Electronic Circuit and System Simulation Methods*. McGraw-Hill.

3. Ogrodzki, J. (1994). *Circuit Simulation Methods and Algorithms*. CRC Press.

4. Kundert, K.S. (1995). *The Designer's Guide to Spice and Spectre*. Kluwer Academic.

5. Yeh, D.T. (2009). *A Detailed Digital Model of the Fender Bassman Push-Pull Power Amp*. Stanford University, CCRMA.
