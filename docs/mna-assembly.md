# MNA Matrix Assembly

This document describes how to construct Modified Nodal Analysis (MNA) matrices from circuit netlists for the melange-solver.

## Overview

Modified Nodal Analysis (MNA) extends classical nodal analysis to handle voltage sources and other elements that don't fit the simple conductance model. It forms the foundation of circuit simulation (SPICE) and is the starting point for the DK method used in melange.

## 1. MNA Basics

### Why MNA?

Classical nodal analysis cannot directly handle:
- **Voltage sources** (infinite conductance)
- **Current-controlled elements** (CCCS, CCVS)
- **Zero-resistance paths**

MNA solves this by augmenting the system with additional variables for voltage source currents.

### Matrix Formulation

The MNA system for time-domain analysis:

```
G·v(t) + C·dv(t)/dt = s(t)
```

Where:
- **G** (N×N): Conductance matrix (DC, resistive elements)
- **C** (N×N): Capacitance matrix (reactive elements)
- **v(t)** (N×1): Unknown vector (node voltages + voltage source currents)
- **s(t)** (N×1): Source vector (independent current/voltage sources)

For the DK method, we split G and C because reactive elements are handled via companion models in discrete time.

### Unknown Vector Structure

For a circuit with:
- `n` non-ground nodes (0 to n-1, where 0 = ground)
- `m` independent voltage sources

The unknown vector `v` has dimension **(n + m)**:

```
v = [v₁, v₂, ..., vₙ | i_V₁, i_V₂, ..., i_Vₘ]ᵀ
    └─ node voltages ─┘ └─ voltage source currents ─┘
```

Voltage source currents are positive when flowing from positive to negative terminal.

### System Matrix Structure

The combined MNA matrix has block structure:

```
┌───────────┬───────────┐┌─────┐   ┌─────┐
│  G_nodes  │   B       ││v_n  │   │i_src│
├───────────┼───────────┤├─────┤ = ├─────┤
│  Bᵀ       │   0       ││i_v  │   │v_src│
└───────────┴───────────┘└─────┘   └─────┘
```

Where:
- **G_nodes**: Nodal conductance matrix (n×n)
- **B**: Voltage source connection matrix (n×m), entries are +1/-1
- **Bᵀ**: KVL constraints (m×n)
- **0**: Zero block (voltage sources have no "self-conductance")

## 2. Component Stamps

Each component contributes to specific matrix positions. This section shows the stamp patterns.

### Resistor (R between nodes i and j)

Conductance `g = 1/R`

```
      i      j
   ┌─────┬─────┐
i  │  g  │ -g  │
   ├─────┼─────┤
j  │ -g  │  g  │
   └─────┴─────┘
```

In code:
```rust
G[i][i] += g;
G[j][j] += g;
G[i][j] -= g;
G[j][i] -= g;
```

### Capacitor (C between nodes i and j)

Same pattern in the C matrix:

```
      i      j
   ┌─────┬─────┐
i  │  C  │ -C  │
   ├─────┼─────┤
j  │ -C  │  C  │
   └─────┴─────┘
```

In code:
```rust
C[i][i] += c;
C[j][j] += c;
C[i][j] -= c;
C[j][i] -= c;
```

### Independent Voltage Source (V between nodes i and j)

Adds one row and column to the system. Let the new index be `k` (after all node voltages).

```
      i      j      k
   ┌─────┬─────┬─────┐
i  │     │     │  1  │
   ├─────┼─────┼─────┤
j  │     │     │ -1  │
   ├─────┼─────┼─────┤
k  │  1  │ -1  │  0  │
   └─────┴─────┴─────┘
```

In code:
```rust
// B matrix entries (G block)
G[i][k] = 1.0;
G[j][k] = -1.0;

// Bᵀ matrix entries (constraint rows)
G[k][i] = 1.0;
G[k][j] = -1.0;

// RHS: voltage value
rhs[k] = v_value;
```

The row `k` enforces KVL: `v_i - v_j = V_source`

### Independent Current Source (I into node i, out of node j)

Enters the RHS vector only:

```
rhs[i] += I
rhs[j] -= I
```

Positive current flows into the positive node.

### Voltage-Controlled Current Source (VCCS)

Current from node i to node j, controlled by voltage between nodes k and l:

```
i = g_m · (v_k - v_l)
```

Stamp (4 entries in G):

```
      k       l
   ┌───────┬───────┐
i  │  g_m  │ -g_m  │
   ├───────┼───────┤
j  │ -g_m  │  g_m  │
   └───────┴───────┘
```

In code:
```rust
G[i][k] += g_m;
G[i][l] -= g_m;
G[j][k] -= g_m;
G[j][l] += g_m;
```

### Voltage-Controlled Voltage Source (VCVS)

Voltage between nodes i and j equals gain × (v_k - v_l):

```
v_i - v_j = A · (v_k - v_l)
```

Adds one row/column for the source current. Let `m` be the new index.

Stamp:
```
      i      j      k      l      m
   ┌─────┬─────┬─────┬─────┬─────┐
i  │     │     │     │     │  1  │
   ├─────┼─────┼─────┼─────┼─────┤
j  │     │     │     │     │ -1  │
   ├─────┼─────┼─────┼─────┼─────┤
m  │  1  │ -1  │ -A  │  A  │  0  │
   └─────┴─────┴─────┴─────┴─────┘
```

### Current-Controlled Current Source (CCCS)

Current from i to j equals gain × (current through controlling V_source):

Requires the controlling voltage source to already exist in the system (index `k` for its current variable).

Stamp:
```
      k
   ┌─────┐
i  │  β  │
   ├─────┤
j  │ -β  │
   └─────┘
```

### Current-Controlled Voltage Source (CCVS)

Voltage between i and j equals transresistance × (controlling current):

Similar to VCVS but controlled by a current variable (voltage source current) rather than node voltages.

### Ideal Op-Amp

Output at node o, inputs at nodes p (non-inverting) and n (inverting):

Ideal op-amp enforces: v_p = v_n (virtual short)

Stamp adds constraint row:
```
      p      n      o      m (new row)
   ┌─────┬─────┬─────┬─────┐
m  │  1  │ -1  │  0  │  0  │  = 0  (virtual short constraint)
   └─────┴─────┴─────┴─────┘
```

Note: Output node o has no constraint; the op-amp supplies whatever current is needed.

## 3. Nonlinear Device Handling

Nonlinear devices (diodes, BJTs, tubes) are handled separately from the linear MNA system. The DK method extracts them into a smaller nonlinear system.

### N_v Matrix (M×N)

Extracts controlling voltages for M nonlinear devices from N node voltages.

For a nonlinear device d with controlling voltage between nodes i and j:

```
N_v[d][i] = 1
N_v[d][j] = -1
```

The controlling voltage for device d:
```
v_ctrl[d] = N_v[d] · v_nodes = v_i - v_j
```

### N_i Matrix (N×M)

Injects nonlinear currents back into circuit nodes (transpose relationship):

```
N_i = N_vᵀ
```

If device d injects current into node i and extracts from node j:
```
N_i[i][d] = 1
N_i[j][d] = -1
```

### Constructing from Netlist

For each nonlinear device in the netlist:

1. **Identify terminals**: Collector, Base, Emitter (BJT); Plate, Grid, Cathode (tube); Anode, Cathode (diode)

2. **Map to node indices** using the circuit's node numbering

3. **Define controlling voltage**: Usually V_BE for BJT, V_GK or V_PK for tubes

4. **Stamp N_v row**: +1 at positive terminal, -1 at negative terminal

5. **Stamp N_i column**: Same pattern (current enters positive terminal node)

### Example: NPN BJT

For an NPN with terminals C, B, E at nodes c, b, e:

Controlling voltage: V_BE = v_b - v_e  
Output current: I_C (collector current, entering collector node)

```
N_v = [0, 1, -1]  // row for this device: [v_c, v_b, v_e] → v_b - v_e

N_i column for this device:
    ┌───┐
v_c │ 1 │  // I_C enters here
    ├───┤
v_b │ 0 │
    ├───┤
v_e │-1 │  // I_C returns here (or exits via emitter current)
    └───┘
```

Note: Full BJT model includes both I_C and I_E, requiring two rows in N_v and two columns in N_i.

## 4. Ground Reference

### Node 0 is Ground

Ground serves as the voltage reference: v_ground = 0V always.

### Matrix Reduction

Ground node is **omitted** from the unknown vector and matrices:
- G matrix: n×n (n = non-ground nodes)
- C matrix: n×n
- Unknown vector: n node voltages + m source currents

### Node Numbering Convention

```
Node 0: Ground (always)
Node 1: First non-ground node
Node 2: Second non-ground node
...
Node n-1: Last non-ground node
```

When stamping components connected to ground:
- For a resistor between node i and ground: only G[i][i] gets +1/R
- For a voltage source between node i and ground: only G[i][k] and G[k][i] are set

### Handling Ground in Stamps

When either i or j is 0 (ground), simplify:

```rust
// Resistor between node i and ground
if j == 0 {
    G[i][i] += g;
} else if i == 0 {
    G[j][j] += g;
} else {
    // Normal case: both non-ground
    G[i][i] += g;
    G[j][j] += g;
    G[i][j] -= g;
    G[j][i] -= g;
}
```

## 5. Matrix Properties

### Symmetry

**G and C matrices are symmetric** for circuits containing only:
- Resistors
- Capacitors
- Independent voltage/current sources

VCCS breaks symmetry (G[i][j] ≠ G[j][i] in general).

The full MNA matrix is symmetric **if and only if**:
- All controlled sources are VCCS with symmetric placement, OR
- CCCS/CCVS are represented via equivalent circuits

### Positive Definiteness

The G matrix is positive definite when:
1. No negative resistance (passive circuit)
2. No zero-resistance loops
3. All voltage sources have series resistance (or are handled via limiting)

For the DK method, we require G to be invertible (well-defined DC operating point).

### Sparsity

Audio circuits typically exhibit high sparsity:
- Each node connects to 2-5 components
- G matrix has O(n) nonzeros, not O(n²)
- Typical sparsity: < 5% nonzeros for n > 20

Sparsity pattern example for a ladder filter:
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

Banded matrices (tridiagonal) are common in filter structures.

### Conditioning

The MNA matrix can be ill-conditioned when:
- Very large and very small conductances coexist (wide dynamic range)
- Floating nodes (no DC path to ground)
- Voltage source loops

Condition number κ(G) = ||G|| · ||G⁻¹|| should be < 10¹² for f64.

## 6. Example Walkthrough

Let's build matrices for a common-emitter BJT amplifier stage.

### Circuit

```
     Vcc
      │
      Rc
      │
      ├──┬─── Vo
      │  │
    C│  ╱ C
     B│ ╱   E
      │╱    │
      │     Re
      │     │
      │     ═══
      │     Gnd
      Rb
      │
      ├──── Vi
      │
     Cin
      │
     Gnd
```

### Netlist

```spice
Vcc 1 0 DC 9V
Vin 4 0 AC 1
Rb 1 2 100k
Rc 1 3 4.7k
Re 5 0 1k
Cin 4 2 10u
Q1 3 2 5 NPN
```

### Node Numbering

| Node | Connection |
|------|------------|
| 0 | Ground |
| 1 | Vcc, Rb, Rc |
| 2 | Rb, Cin, Q1 Base |
| 3 | Rc, Q1 Collector |
| 4 | Vin, Cin |
| 5 | Q1 Emitter, Re |

### Initial G Matrix (before nonlinear handling)

Non-ground nodes: 1, 2, 3, 4, 5 (5×5)

```
        1        2        3        4        5
     ┌────────┬────────┬────────┬────────┬────────┐
1    │1/Rb+1/Rc│ -1/Rb  │ -1/Rc  │   0    │   0    │
     ├────────┼────────┼────────┼────────┼────────┤
2    │ -1/Rb  │ 1/Rb   │   0    │   0    │   0    │
     ├────────┼────────┼────────┼────────┼────────┤
3    │ -1/Rc  │   0    │ 1/Rc   │   0    │   0    │
     ├────────┼────────┼────────┼────────┼────────┤
4    │   0    │   0    │   0    │   0    │   0    │
     ├────────┼────────┼────────┼────────┼────────┤
5    │   0    │   0    │   0    │   0    │ 1/Re   │
     └────────┴────────┴────────┴────────┴────────┘
```

### C Matrix (Capacitors only)

Cin between nodes 4 and 2:

```
        1    2     3    4     5
     ┌────┬─────┬────┬─────┬────┐
1    │ 0  │  0  │ 0  │  0  │ 0  │
     ├────┼─────┼────┼─────┼────┤
2    │ 0  │ Cin │ 0  │-Cin │ 0  │
     ├────┼─────┼────┼─────┼────┤
3    │ 0  │  0  │ 0  │  0  │ 0  │
     ├────┼─────┼────┼─────┼────┤
4    │ 0  │-Cin │ 0  │ Cin │ 0  │
     ├────┼─────┼────┼─────┼────┤
5    │ 0  │  0  │ 0  │  0  │ 0  │
     └────┴─────┴────┴─────┴────┘
```

### Voltage Source Stamps

Vcc (node 1 to ground) adds row/column 6:
```
G[1][6] = 1, G[6][1] = 1, rhs[6] = 9V
```

Vin (node 4 to ground) adds row/column 7:
```
G[4][7] = 1, G[7][4] = 1, rhs[7] = Vin(t)
```

### Final G Structure (7×7)

```
        1        2        3        4    5   6   7
     ┌────────┬────────┬────────┬────┬───┬───┬───┐
1    │ G11    │ -1/Rb  │ -1/Rc  │ 0  │ 0 │ 1 │ 0 │
     ├────────┼────────┼────────┼────┼───┼───┼───┤
2    │ -1/Rb  │ 1/Rb   │ 0      │ 0  │ 0 │ 0 │ 0 │
     ├────────┼────────┼────────┼────┼───┼───┼───┤
3    │ -1/Rc  │ 0      │ 1/Rc   │ 0  │ 0 │ 0 │ 0 │
     ├────────┼────────┼────────┼────┼───┼───┼───┤
4    │ 0      │ 0      │ 0      │ 0  │ 0 │ 0 │ 1 │
     ├────────┼────────┼────────┼────┼───┼───┼───┤
5    │ 0      │ 0      │ 0      │ 0  │1/Re│0 │ 0 │
     ├────────┼────────┼────────┼────┼───┼───┼───┤
6    │ 1      │ 0      │ 0      │ 0  │ 0 │ 0 │ 0 │  = 9V
     ├────────┼────────┼────────┼────┼───┼───┼───┤
7    │ 0      │ 0      │ 0      │ 1  │ 0 │ 0 │ 0 │  = Vin(t)
     └────────┴────────┴────────┴────┴───┴───┴───┘
```

### Nonlinear Matrices (BJT Q1)

Q1 is an NPN with terminals: C=3, B=2, E=5

For Ebers-Moll or Gummel-Poon model, we need two controlling voltages:
- V_BE = v_2 - v_5 (base-emitter)
- V_BC = v_2 - v_3 (base-collector)

**N_v matrix** (2×5, extracting from nodes 1-5):

```
        1    2     3    4    5
     ┌────┬─────┬─────┬────┬─────┐
V_BE │ 0  │  1  │  0  │ 0  │ -1  │  (v_2 - v_5)
     ├────┼─────┼─────┼────┼─────┤
V_BC │ 0  │  1  │ -1  │ 0  │  0  │  (v_2 - v_3)
     └────┴─────┴─────┴────┴─────┘
```

**N_i matrix** (5×2, injecting currents into nodes 1-5):

```
      I_C  I_E
     ┌────┬────┐
1    │ 0  │ 0  │
     ├────┼────┤
2    │ 0  │ 0  │  (base current handled separately or negligible)
     ├────┼────┤
3    │ 1  │ 0  │  (I_C enters collector)
     ├────┼────┤
4    │ 0  │ 0  │
     ├────┼────┤
5    │ 0  │ -1 │  (I_E exits emitter, so -I_E enters)
     └────┴────┘
```

Note: Actual BJT models may use different formulations (transport model vs. injection model). The above shows the principle.

## 7. Implementation Algorithm

### Phase 1: Parse Netlist

```rust
struct Netlist {
    nodes: HashSet<String>,      // "0", "1", "2", ...
    components: Vec<Component>,
}

enum Component {
    Resistor { name: String, nodes: [usize; 2], value: f64 },
    Capacitor { name: String, nodes: [usize; 2], value: f64 },
    VoltageSource { name: String, nodes: [usize; 2], value: f64 },
    CurrentSource { name: String, nodes: [usize; 2], value: f64 },
    VCCS { name: String, i_nodes: [usize; 2], v_nodes: [usize; 2], gm: f64 },
    Bjt { name: String, c: usize, b: usize, e: usize, model: String },
    // ...
}
```

### Phase 2: Node Discovery

```rust
fn assign_node_indices(netlist: &Netlist) -> HashMap<String, usize> {
    let mut indices = HashMap::new();
    let mut next_idx = 1;  // Start at 1, reserve 0 for ground
    
    for node in &netlist.nodes {
        if node == "0" || node == "GND" {
            indices.insert(node.clone(), 0);
        } else if !indices.contains_key(node) {
            indices.insert(node.clone(), next_idx);
            next_idx += 1;
        }
    }
    indices
}
```

### Phase 3: Matrix Allocation

```rust
let n_nodes = indices.len() - 1;  // Exclude ground
let n_vsrc = netlist.components.iter()
    .filter(|c| matches!(c, Component::VoltageSource { .. }))
    .count();

let dim = n_nodes + n_vsrc;
let mut G = vec![vec![0.0; dim]; dim];
let mut C = vec![vec![0.0; dim]; dim];
let mut rhs = vec![0.0; dim];
```

### Phase 4: Component Iteration

```rust
let mut vsrc_idx = n_nodes;  // Voltage source rows start after node voltages

for comp in &netlist.components {
    match comp {
        Component::Resistor { nodes: [n1, n2], value, .. } => {
            let (i, j) = map_nodes(*n1, *n2, &indices);
            let g = 1.0 / value;
            stamp_resistor(&mut G, i, j, g);
        }
        Component::Capacitor { nodes: [n1, n2], value, .. } => {
            let (i, j) = map_nodes(*n1, *n2, &indices);
            stamp_capacitor(&mut C, i, j, *value);
        }
        Component::VoltageSource { nodes: [n1, n2], value, .. } => {
            let (i, j) = map_nodes(*n1, *n2, &indices);
            stamp_voltage_source(&mut G, &mut rhs, i, j, vsrc_idx, *value);
            vsrc_idx += 1;
        }
        // ... etc
    }
}
```

### Phase 5: Validation

```rust
fn validate_mna(G: &Matrix, n_nodes: usize) -> Result<(), MnaError> {
    // Check for floating nodes (no DC path to ground)
    for i in 0..n_nodes {
        if G[i][i] == 0.0 {
            return Err(MnaError::FloatingNode(i));
        }
    }
    
    // Check diagonal dominance (sufficient but not necessary)
    for i in 0..n_nodes {
        let diag = G[i][i].abs();
        let off_diag_sum: f64 = (0..G[i].len())
            .filter(|&j| j != i)
            .map(|j| G[i][j].abs())
            .sum();
        
        if diag < off_diag_sum * 0.001 {
            eprintln!("Warning: Node {} may be ill-conditioned", i);
        }
    }
    
    // Check for symmetry in passive part
    for i in 0..n_nodes {
        for j in 0..i {
            if (G[i][j] - G[j][i]).abs() > 1e-10 {
                eprintln!("Warning: G[{}][{}] != G[{}][{}]", i, j, j, i);
            }
        }
    }
    
    Ok(())
}
```

### Phase 6: Nonlinear Device Setup

```rust
let n_nonlinear = netlist.components.iter()
    .filter(|c| is_nonlinear(c))
    .count();

let mut N_v = vec![vec![0.0; n_nodes]; n_nonlinear];
let mut N_i = vec![vec![0.0; n_nonlinear]; n_nodes];

let mut nl_idx = 0;
for comp in &netlist.components {
    if let Component::Bjt { c, b, e, .. } = comp {
        let (c, b, e) = (indices[c], indices[b], indices[e]);
        
        // V_BE controlling voltage
        if b != 0 { N_v[nl_idx][b - 1] = 1.0; }
        if e != 0 { N_v[nl_idx][e - 1] = -1.0; }
        
        // I_C injection
        if c != 0 { N_i[c - 1][nl_idx] = 1.0; }
        if e != 0 { N_i[e - 1][nl_idx] = -1.0; }
        
        nl_idx += 1;
    }
}
```

### Performance Considerations

For melange's compile-time approach:

```rust
// Use const generics for matrix dimensions
struct MnaSystem<const N: usize, const M: usize> {
    G: [[f64; N]; N],      // Fixed-size conductance matrix
    C: [[f64; N]; N],      // Fixed-size capacitance matrix
    N_v: [[f64; N]; M],    // M nonlinear devices, N nodes
    N_i: [[f64; M]; N],    // N nodes, M nonlinear devices
}
```

This allows:
- Stack allocation (no heap)
- Loop unrolling by LLVM
- No bounds checking in release mode
- Cache-friendly access patterns

### References

1. Ho, Ruehli, and Brennan. "The Modified Nodal Approach to Network Analysis." IEEE Transactions on Circuits and Systems, 1975.
2. Pillage, Rohrer, and Visweswariah. "Electronic Circuit and System Simulation Methods." McGraw-Hill, 1995.
3. Ogrodzki. "Circuit Simulation Methods and Algorithms." CRC Press, 1994.
