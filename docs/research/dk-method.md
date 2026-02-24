# The Discrete K-Method (DK Method)

**A Practical Guide for Real-Time Circuit Simulation**

---

## 1. Introduction and Motivation

### 1.1 The Problem

Traditional circuit simulation using Modified Nodal Analysis (MNA) requires solving large sparse linear systems at every time step. For a circuit with N nodes, this is typically O(N³) using direct methods (LU/QR) or requires iterative solvers with unpredictable convergence.

For real-time audio (44.1-96kHz, <1ms latency), we need:
- **Deterministic computation time** — no iteration counts that vary with signal
- **Cache-friendly data structures** — predictable memory access patterns
- **Vectorization opportunities** — SIMD-friendly inner loops

### 1.2 David Yeh's Insight (2009)

In his PhD thesis *"Digital Implementation of Musical Distortion Circuits by Analysis and Simulation"* (Stanford CCRMA, 2009), David Yeh observed that **most audio circuits have few nonlinear elements** (diodes, tubes, transistors) compared to linear elements (resistors, capacitors, inductors).

**Key insight:** Instead of solving the full N×N MNA system with Newton-Raphson at each step, separate the circuit into:
1. A **linear subcircuit** (solved once, offline)
2. A **nonlinear kernel** (small M×M system, solved per-sample)

Where M = number of nonlinear elements (typically 1-4 for audio circuits), and N = total nodes (can be 20-100+).

### 1.3 The Separation Principle

```
┌─────────────────────────────────────────────────────────────┐
│                      Circuit Netlist                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                   │
│  │ Linear   │  │ Nonlinear│  │ Linear   │  ...              │
│  │ R, C, L  │  │ Diode    │  │ R, C, L  │                   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                   │
│       │             │             │                         │
│       └─────────────┼─────────────┘                         │
│                     │                                       │
│              ┌─────────────┐                                │
│              │ M nonlinear │                                │
│              │ ports       │                                │
│              └─────────────┘                                │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Offline (once):        │  Runtime (per-sample):            │
│  ┌─────────────────┐    │  ┌─────────────────────────┐      │
│  │ Factor linear   │    │  │ Solve M×M nonlinear     │      │
│  │ network         │    │  │ system via NR           │      │
│  │ → A, S matrices │    │  │ → port voltages/currents│      │
│  └─────────────────┘    │  └─────────────────────────┘      │
│                              │                              │
│                              ▼                              │
│                         ┌────────────┐                      │
│                         │ Update node│                      │
│                         │ voltages   │                      │
│                         └────────────┘                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Mathematical Foundation

### 2.1 Modified Nodal Analysis (MNA) Review

The MNA system for a linear circuit:

```
(G + sC)X(s) = W(s)
```

Where:
- **G**: Conductance matrix (N×N, symmetric)
- **C**: Capacitance/inductance matrix (N×N)
- **X**: Node voltages + inductor currents
- **W**: Independent sources

In time domain after trapezoidal discretization (see §3):

```
(G + (2/h)C)x[n] = w[n] + (2/h)C x[n-1] - C ẋ[n-1]
                = w[n] + b[n-1]    (b = "history term")
```

Or compactly:

```
Gx̃[n] = w[n] + b[n-1]
```

where G̃ = G + (2/h)C is the effective conductance matrix.

### 2.2 Port Extraction

Introduce M nonlinear ports. Each port connects two circuit nodes (positive and negative terminals). Define:

- **i[n]**: M×1 vector of port currents (into positive terminal)
- **v[n]**: M×1 vector of port voltages (V⁺ - V⁻)

The extended MNA system with ports as current sources:

```
G̃x[n] + Aᵀi[n] = w[n] + b[n-1]          (1)
```

Where **A** is the M×N **port incidence matrix**:

```
Aₘₙ = +1 if port m positive terminal is node n
Aₘₙ = -1 if port m negative terminal is node n
Aₘₙ =  0 otherwise
```

**Key observation:** Equation (1) is linear in x[n] if i[n] were known.

### 2.3 The A Matrix (Port-to-Node Mapping)

For a circuit with N nodes (including ground) and M nonlinear ports:

```
     nodes 1...N
        ↓
     ┌─────────────────────┐
A =  │  +1  -1   0   ...  │  ← port 1 between nodes 1 & 2
     │   0   0  +1  -1 ... │  ← port 2 between nodes 3 & 4
     │  ...                │
     └─────────────────────┘
        M × N
```

**Properties:**
- Each row has exactly two nonzeros: +1 and -1
- A is full row-rank if ports are independent
- Aᵀ maps port currents to node injections

### 2.4 The S Matrix (Port Impedance Matrix)

Define the **port impedance matrix** S (M×M):

```
v[n] = -S i[n] + v₀[n]                    (2)
```

Where:
- **S**: The Thévenin equivalent impedance seen at the ports with all internal sources zeroed
- **v₀[n]**: The open-circuit port voltages (i = 0) due to internal sources w[n] and history b[n-1]

**Derivation:**

From (1), set i = 0 to find open-circuit solution:

```
x₀ = G̃⁻¹(w + b)                          (3)
v₀ = A x₀ = A G̃⁻¹(w + b)                 (4)
```

Now consider only port currents (set w = 0, b = 0):

```
G̃x = -Aᵀi
x = -G̃⁻¹Aᵀi
v = Ax = -A G̃⁻¹Aᵀ i
```

Therefore:

```
S = A G̃⁻¹ Aᵀ                             (5)
```

**S is symmetric positive definite** when G̃ is SPD (passive circuit).

### 2.5 The K Matrix (Nonlinear Characteristic)

Each nonlinear element has a constitutive relation:

```
iₘ = fₘ(vₘ)   or   vₘ = gₘ(iₘ)
```

**Diode example:**
```
i = Iₛ(exp(v/(nVₜ)) - 1)
```

Collect into vector function:

```
i = f(v)                                  (6)
```

### 2.6 The Nonlinear System

Combine (2) and (6):

```
v = -S f(v) + v₀
v + S f(v) = v₀                           (7)
```

Or rearranged:

```
f(v) + S⁻¹(v - v₀) = 0                    (8)
```

This is the **M×M nonlinear system** to solve at each time step.

**Dimension check:**
- Original: N×N (could be 100×100)
- DK reduced: M×M (typically 1×1 to 4×4)

---

## 3. Trapezoidal Discretization and Companion Models

### 3.1 The Trapezoidal Rule

For differential equation ẋ = f(x):

```
x[n] = x[n-1] + (h/2)(f(x[n]) + f(x[n-1]))
```

where h = sample period (1/fs).

Rearranging:

```
x[n] - (h/2)f(x[n]) = x[n-1] + (h/2)f(x[n-1])
```

### 3.2 Capacitor Companion Model

Capacitor: i = C dv/dt → ṽ = i/C

Trapezoidal discretization:

```
v[n] - v[n-1] = (h/2C)(i[n] + i[n-1])
```

Solve for i[n]:

```
i[n] = (2C/h)v[n] - [(2C/h)v[n-1] + i[n-1]]
i[n] = G_eq v[n] + I_eq
```

Where:
- **G_eq = 2C/h** (equivalent conductance, time-independent!)
- **I_eq = -[(2C/h)v[n-1] + i[n-1]]** (equivalent current source, "history term")

```
        G_eq = 2C/h
    ┌────//\/\────┐
 ───┤              ├───
    │   ↓ I_eq     │
   +│              │-
   v[n]           v[n]
    └──────────────┘
    
    Companion model: resistor + current source
```

**History update:**
```
I_eq[n] = -[(2C/h)v[n] + i[n]]
        = -[G_eq v[n] + i[n]]
```

### 3.3 Inductor Companion Model

Inductor: v = L di/dt

Similarly:

```
v[n] = (2L/h)i[n] - [(2L/h)i[n-1] + v[n-1]]
v[n] = R_eq i[n] + V_eq
```

Where:
- **R_eq = 2L/h** (equivalent resistance)
- **V_eq = -[(2L/h)i[n-1] + v[n-1]]** (equivalent voltage source)

### 3.4 Companion Model Summary Table

| Element | Continuous | Discrete Companion | G_eq | History Update |
|---------|-----------|-------------------|------|----------------|
| Resistor | i = Gv | i = Gv | G | — |
| Capacitor | i = C dv/dt | i = G_eq v + I_eq | 2C/h | I_eq[n] = -(G_eq v[n] + i[n]) |
| Inductor | v = L di/dt | v = R_eq i + V_eq | 2L/h | V_eq[n] = -(R_eq i[n] + v[n]) |

### 3.5 Numerical Integration Stability

| Method | Stability | Accuracy | Companion G_eq |
|--------|-----------|----------|----------------|
| Forward Euler | Conditional | O(h) | C/h |
| Backward Euler | Unconditional | O(h) | C/h |
| Trapezoidal | Unconditional | O(h²) | 2C/h |
| BDF2 | Unconditional | O(h²) | 3C/(2h) |

**Trapezoidal is preferred for audio** due to:
- Unconditional stability
- Second-order accuracy
- **No numerical damping** (preserves resonances)

*Caution:* Trapezoidal can exhibit "ringing" with ideal switches/limiters. Backward Euler may be preferred for such cases.

---

## 4. Nonlinear Kernel Extraction

### 4.1 The Two-Phase Algorithm

**Phase 1: Offline (Once)**

```
Input: Netlist with N nodes, M nonlinear elements

1. Build G, C matrices (N×N) from linear elements
2. Compute G̃ = G + (2/h)C
3. Build A matrix (M×N) from port definitions
4. Factor G̃ = LU (or compute G̃⁻¹ if N small)
5. Compute S = A G̃⁻¹ Aᵀ  (M×M)

Output: S matrix, G̃ factorization, A matrix
```

**Phase 2: Runtime (Per-Sample)**

```
Input: Input sample x[n], history b[n-1], S, A, G̃⁻¹

1. Build source vector w[n] from inputs
2. Compute open-circuit voltages: v₀ = A G̃⁻¹(w + b)
3. Solve M×M nonlinear system: v + S f(v) = v₀
   (via Newton-Raphson)
4. Compute port currents: i = f(v)
5. Compute node voltages: x = G̃⁻¹(w + b - Aᵀi)
6. Update history terms for reactive elements
7. Output: selected node voltage(s)
```

### 4.2 Newton-Raphson for the Kernel

Solve: **F(v) = v + S f(v) - v₀ = 0**

Jacobian:

```
J(v) = I + S diag(f'(v))
     = I + S G_d(v)
```

where G_d(v) = diag(∂fₘ/∂vₘ) is the differential conductance of each nonlinear element.

**NR iteration:**

```
J(v⁽ᵏ⁾) Δv⁽ᵏ⁾ = -F(v⁽ᵏ⁾)
v⁽ᵏ⁺¹⁾ = v⁽ᵏ⁾ + Δv⁽ᵏ⁾
```

For M ≤ 4, solve directly via Cramer's rule or pre-computed inverse formula.

### 4.3 Pseudocode: Complete DK Solver

```python
class DKSolver:
    def __init__(self, circuit, sample_rate):
        self.h = 1.0 / sample_rate
        self.N = circuit.num_nodes
        self.M = circuit.num_nonlinear
        
        # Build G, C matrices
        self.G = build_conductance_matrix(circuit)
        self.C = build_reactance_matrix(circuit)
        self.G_tilde = self.G + (2.0/self.h) * self.C
        
        # Factor G_tilde (LU decomposition)
        self.LU = lu_factor(self.G_tilde)
        
        # Build port incidence matrix
        self.A = build_port_matrix(circuit)  # M×N
        
        # Compute S = A @ G_tilde^-1 @ A.T
        # Efficiently: solve G_tilde X = A.T, then S = A @ X
        X = lu_solve(self.LU, self.A.T)  # N×M
        self.S = self.A @ X  # M×M
        
        # History vector for reactive elements
        self.history = zeros(self.N)
        
    def process_sample(self, input_sample):
        # 1. Build source vector
        w = self.build_source_vector(input_sample)
        
        # 2. Compute v0 (open-circuit port voltages)
        rhs = w + self.history
        x_oc = lu_solve(self.LU, rhs)  # N×1
        v0 = self.A @ x_oc  # M×1
        
        # 3. Solve nonlinear system v + S @ f(v) = v0
        v = self.solve_nonlinear(v0)
        
        # 4. Compute port currents
        i = self.nonlinear_function(v)
        
        # 5. Compute final node voltages
        rhs = w + self.history - self.A.T @ i
        x = lu_solve(self.LU, rhs)
        
        # 6. Update history
        self.update_history(x, i)
        
        # 7. Return output (e.g., specific node voltage)
        return x[self.output_node]
    
    def solve_nonlinear(self, v0):
        """Newton-Raphson for M×M system"""
        v = v0.copy()  # Initial guess
        
        for iteration in range(MAX_ITER):
            i, Gd = self.nonlinear_with_derivative(v)
            
            # F(v) = v + S @ i - v0
            F = v + self.S @ i - v0
            
            # J = I + S @ diag(Gd)
            J = eye(self.M) + self.S * diag(Gd)
            
            # Solve J @ delta = -F
            delta = solve(J, -F)
            
            v = v + delta
            
            if norm(delta) < TOLERANCE:
                break
                
        return v
```

---

## 5. Sherman-Morrison for Time-Varying Elements

### 5.1 The Problem

What if a resistor (e.g., potentiometer, LDR) changes value during operation?

G̃ changes → need to recompute S = A G̃⁻¹ Aᵀ → O(N³) forbidden for real-time!

### 5.2 Sherman-Morrison Formula

For a rank-1 update to a matrix:

```
If  B = A + uvᵀ
Then B⁻¹ = A⁻¹ - (A⁻¹uvᵀA⁻¹) / (1 + vᵀA⁻¹u)
```

**Cost:** O(N²) instead of O(N³)

### 5.3 Conductance Update

Changing a resistor between nodes p and q from G_old to G_new:

```
ΔG = G_new - G_old

G̃_new = G̃_old + ΔG × (e_p - e_q)(e_p - e_q)ᵀ
```

This is a rank-1 update! Apply Sherman-Morrison to update G̃⁻¹.

### 5.4 Port Impedance Update

After updating G̃⁻¹, we need new S:

```
S_new = A G̃_new⁻¹ Aᵀ
```

If M << N, we can update S directly using the SM formula applied to the inverse operator.

**Efficient approach:** Precompute:

```
P = G̃⁻¹ Aᵀ   (N×M, compute once)
Q = A G̃⁻¹    (M×N, compute once) = Pᵀ if G̃ symmetric
```

When G̃ changes via rank-1 update, update P and Q using SM, then:

```
S = A P = Q Aᵀ = Q G̃ P   (all equivalent)
```

### 5.5 Pseudocode: Time-Varying Update

```python
class TimeVaryingDKSolver(DKSolver):
    def update_resistor(self, node_p, node_q, G_new):
        """Update a resistor value in real-time"""
        G_old = self.get_conductance(node_p, node_q)
        dG = G_new - G_old
        
        # u = e_p - e_q (difference of unit vectors)
        u = zeros(self.N)
        u[node_p] = 1
        u[node_q] = -1
        
        # Sherman-Morrison update of G_tilde^-1
        # G_new^-1 = G_old^-1 - dG * (G_old^-1 @ u @ u.T @ G_old^-1) / (1 + dG * u.T @ G_old^-1 @ u)
        
        Ginv_u = lu_solve(self.LU, u)  # G_tilde^-1 @ u
        denom = 1 + dG * dot(u, Ginv_u)
        
        # Store rank-1 correction for lazy evaluation
        # or update factorization incrementally
        self.Ginv_correction = (dG / denom) * outer(Ginv_u, Ginv_u)
        
        # Update S efficiently
        # S = A @ (Ginv - correction) @ A.T
        A_Ginv_u = self.A @ Ginv_u  # M×1
        self.S = self.S - (dG / denom) * outer(A_Ginv_u, A_Ginv_u)
        
        self.set_conductance(node_p, node_q, G_new)
```

### 5.6 Multiple Time-Varying Elements

For K time-varying resistors, use **Sherman-Morrison-Woodbury** (rank-K update):

```
If  B = A + UCVᵀ
Then B⁻¹ = A⁻¹ - A⁻¹U(C⁻¹ + VᵀA⁻¹U)⁻¹VᵀA⁻¹
```

Where U, V are N×K and C is K×K.

For K=2 (stereo pot), this is still efficient.

---

## 6. Comparison with Other Methods

### 6.1 Wave Digital Filters (WDF)

| Aspect | DK Method | WDF |
|--------|-----------|-----|
| **Variable type** | Kirchhoff (V, I) | Wave (a = V + RI, b = V - RI) |
| **Topology** | Any (extracted from netlist) | Must be structured as tree |
| **Adaptors** | Implicit in S matrix | Explicit 3-port adaptors |
| **Nonlinear handling** | Direct M×M solve | Root-finding at nonlinearity |
| **Multiport** | Natural via A matrix | Requires reflection-free ports |
| **Real-time constraints** | Fixed M, variable N | Fixed tree structure |
| **Automation** | Fully automatic from netlist | Requires manual structure design |

**When to use WDF:**
- Well-understood circuits with tree topology
- Need absolute minimum computation (WDF can be slightly faster)
- Teaching/understanding wave principles

**When to use DK:**
- Arbitrary netlists (non-tree topologies)
- Automatic code generation from SPICE
- Circuits with multiple interacting nonlinearities

### 6.2 K-Method (Kirchhoff Method)

The "K-method" (Kirchhoff method) is a precursor to DK that works in continuous time:

```
v = -Z(s) i + v₀(s)
```

Where Z(s) is the impedance matrix in Laplace domain.

| Aspect | K-Method | DK Method |
|--------|----------|-----------|
| **Domain** | Continuous (s) | Discrete (z) |
| **Discretization** | Apply to final equations | Built into companion models |
| **Time-varying** | Complex (modulate Z(s)) | Simple (update G̃) |
| **Delay-free loops** | Implicit handling | Explicit via S matrix |
| **Implementation** | State-space + NLSolve | Direct sample processing |

**DK is essentially the K-method with built-in trapezoidal discretization**, making it more suitable for direct implementation.

### 6.3 Direct MNA with Newton-Raphson

| Aspect | Direct MNA | DK Method |
|--------|-----------|-----------|
| **System size** | N×N | M×M (nonlinear) + N×N (linear, factored) |
| **Per-sample work** | Factor/refactor N×N | Solve M×M + back-subst N×N |
| **Complexity** | O(N³) worst case | O(N²) setup + O(M³ + N²) runtime |
| **Numerical stability** | Good | Better (smaller system) |
| **Circuit changes** | Rebuild matrices | Update S (Sherman-Morrison) |

### 6.4 State-Space Methods

```
x[n+1] = A x[n] + B u[n] + C f(D x[n] + E u[n])
```

| Aspect | State-Space | DK Method |
|--------|-------------|-----------|
| **Dimension reduction** | Model order reduction | Topological extraction |
| **Physical meaning** | Lost in reduction | Preserved (node voltages) |
| **Parameter variation** | Affects entire system | Localized to S update |
| **Automation** | Requires MOR expertise | Direct from netlist |

---

## 7. Real-Time Audio Implementation

### 7.1 Memory Layout

**Requirements:**
- No heap allocation in audio thread
- Contiguous arrays for cache efficiency
- Fixed-size arrays (const generics in Rust)

```rust
// Template parameters: N nodes, M nonlinear ports, P input sources
pub struct DKSolver<const N: usize, const M: usize, const P: usize> {
    // Precomputed (setup)
    g_tilde_lu: LU<N>,           // Factored G̃ matrix
    s_matrix: SMatrix<f64, M, M>, // S = A G̃⁻¹ Aᵀ
    a_matrix: SMatrix<f64, M, N>, // Port incidence
    
    // State (per-sample mutable)
    history: SVector<f64, N>,     // Reactive element history
    v_prev: SVector<f64, M>,      // Previous NR solution (warm start)
}
```

### 7.2 Newton-Raphson Optimizations

**Warm starting:**
```rust
// Use previous solution as initial guess
let mut v = self.v_prev;
```

**Early termination:**
```rust
for iter in 0..MAX_ITER {
    let (i, g_d) = nonlinear_eval(v);
    let f = v + self.s_matrix * i - v0;
    
    if f.norm() < ABS_TOL {
        break;
    }
    // ... NR step
}
self.v_prev = v;  // Save for next sample
```

**Fixed iteration count:** For hard real-time, use exactly K iterations with warm start. Works well for audio signals with small inter-sample variation.

### 7.3 SIMD Optimization

For the linear algebra operations:

```rust
// N×N back-substitution (dominant cost for large N)
// Can be vectorized if N is multiple of SIMD width

// M×M solve (M small, 1-4)
// Unroll completely, use Cramer's rule for M≤3
```

### 7.4 Computational Budget Analysis

For a circuit with N=20 nodes, M=2 nonlinear elements:

| Operation | Cost | @ 48kHz budget (20µs) |
|-----------|------|----------------------|
| Build w vector | O(P) | Negligible |
| Back-subst for v₀ | O(N²) ≈ 400 mults | ~400 ns |
| NR (3 iter × M³) | O(3×8) ≈ 24 mults | ~24 ns |
| Back-subst for x | O(N²) ≈ 400 mults | ~400 ns |
| History update | O(N) | ~20 ns |
| **Total** | ~850 flops | **~1 µs** |

Plenty of headroom! Even N=100, M=4:
- Back-subst: 2 × 10,000 = 20,000 mults
- NR: 3 × 64 = 192 mults
- Total: ~20k flops → ~20 µs (still feasible at 48kHz)

### 7.5 Numerical Considerations

**Matrix conditioning:**
- G̃ can be ill-conditioned if circuit has floating nodes
- Use pivoting in LU factorization
- Check condition number at build time

**Newton-Raphson convergence:**
- Some nonlinearities (diode, tube) have exponential terms
- Use **continuation methods** or **damping** if needed
- For audio, signal levels are usually well-behaved

**DC operating point:**
- Must be found before transient simulation
- Same DK machinery works with h → ∞ (capacitors open, inductors short)

### 7.6 Code Generation Strategy

For a circuit compiler:

```
Netlist → Parser → MNA matrices → LU factorization → 
    ↓
Generate Rust code:
    - Const N, M from circuit
    - Hard-coded L, U factors (sparse)
    - Hard-coded S matrix
    - Nonlinear function match on element type
```

**Example generated structure:**

```rust
pub struct CircuitPreamp {
    // LU factors of G_tilde (hard-coded)
    l_factor: [[f64; 20]; 20],
    u_factor: [[f64; 20]; 20],
    
    // S matrix (hard-coded 2x2)
    s_matrix: [[f64; 2]; 2],
    
    // State
    history: [f64; 20],
    v_prev: [f64; 2],
}

impl CircuitPreamp {
    pub fn process(&mut self, input: f64) -> f64 {
        // Generated code for this specific circuit
        // All loops unrolled, all indices constant
        // ...
    }
}
```

---

## 8. References

1. **David T. Yeh**, "Digital Implementation of Musical Distortion Circuits by Analysis and Simulation," PhD Thesis, Stanford University, 2009.
   - *The foundational DK method thesis*

2. **David T. Yeh and Julius O. Smith III**, "Discretization of the '59 Fender Bassman Tone Stack," Proc. DAFx, 2006.
   - *Early application of the approach*

3. **Kurt James Werner et al.**, "Wave Digital Filter Modeling of Circuits with Operational Amplifiers," Proc. DAFx, 2016.
   - *Comparison with WDF approaches*

4. **Julius O. Smith III**, "Physical Audio Signal Processing," online book.
   - *WDF theory and applications*

5. **J. Vlach and K. Singhal**, "Computer Methods for Circuit Analysis and Design," Van Nostrand Reinhold, 1994.
   - *MNA and companion models*

6. **L. O. Chua and P.-M. Lin**, "Computer-Aided Analysis of Electronic Circuits: Algorithms and Computational Techniques," Prentice-Hall, 1975.
   - *Classical circuit simulation theory*

---

## Appendix A: Complete Worked Example

### Diode Clipper Circuit

```
        R1
    ┌──//\/\──┬────────┐
    │          │        │
   Vin      C1─┤─    D1 ⤓
    │          │        │
   GND       GND      GND
```

**Circuit parameters:**
- R1 = 10kΩ
- C1 = 10nF
- D1: Is = 1e-12 A, n = 1, Vt = 26mV

**Nodes:** 1 (junction), 0 (ground/reference)
**Ports:** 1 (diode, between node 1 and ground)

**M = 1, N = 1**

**Step 1: Build matrices**

```
G = [1/R1] = [0.0001]
C = [C1] = [1e-8]
G̃ = G + 2C/h = [0.0001 + 2e-8/h]

For h = 1/44100: G̃ ≈ [0.0001 + 0.000882] = [0.000982]
```

**Step 2: Port matrix**

```
A = [1]  (diode between node 1 and ground)
```

**Step 3: Compute S**

```
S = A G̃⁻¹ Aᵀ = [1] × [1/0.000982] × [1] = [1018.3] Ω
```

**Step 4: Nonlinear function**

```
i(v) = Is × (exp(v/(nVt)) - 1)
i'(v) = Is/(nVt) × exp(v/(nVt))
```

**Step 5: NR solve per sample**

```
Solve: v + S × Is × (exp(v/(nVt)) - 1) = v₀
```

Where v₀ = A × G̃⁻¹ × (w + b) = Vin × (1/R1) / G̃ + history_term

---

*Document version: 1.0*
*Last updated: 2026-02-23*
