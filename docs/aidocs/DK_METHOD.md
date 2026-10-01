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
H = alpha*C        (history matrix; stored as `a_neg` / `A_NEG_DEFAULT`)
```

The generated solver uses the charge (companion) form: `H` has no `-G`
term, and the carried capacitor currents `q_dot = C*dx/dt` complete the
history. Derivation, update rules and the reason for this form:
`COMPANION_MODELS.md` "Charge (Companion) Form". The library `DkKernel`
still builds the whole-system `a_neg = alpha*C - G`. Its spectral radius
`rho(S*(alpha*C - G))` feeds two routing estimates only: the DK-versus-nodal
"trapezoidal unstable" trigger (`codegen/routing.rs`) and the nodal
Schur-versus-full-LU gate (`spectral_radius_s_aneg`, `codegen/stability.rs`).
It does not decide the integrator: backward-Euler promotion is the ring
predicate on the charge-form propagator linearised at the DC operating
point (`RING_PREDICATE.md`). The whole-system operator puts every algebraic
direction at `z = -1`, which is why it is the wrong operator to judge
ringing on.

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
- JFET: 2 dimensions (Vgs,Vds -> Id, Vgs -> Ig)
- MOSFET: 2 dimensions (Vgs,Vds -> Id, Ig=0)
- Tube: 2 dimensions (Vgk -> Ip, Vpk -> Ig)

## Algorithm

### Step 1: Build RHS
```
rhs = rhs_const + H * v_prev + q_dot + V_in(n+1) * G_in      (+ injections, runtime sources, noise at n+1)
```
`H * v_prev + q_dot` is the whole capacitor history. Do NOT add a separate
cap_history, and do NOT add `-G * v_prev`, `V_in(n) * G_in` or
`N_i * i_nl_prev`: those are terms of the whole-system form, and each one
added to the charge form double-counts a source.

`rhs_const` is ×1 on every row. The input is stamped once, at `n+1`.

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
v = v_pred + S * N_i * i_nl
```

This uses the full `i_nl` at `n+1` (not a delta against `i_nl_prev`): in the
charge form the nonlinear current enters the step at `n+1` only, exactly
as KCL at `n+1` requires. Its effect on the capacitor charge reaches the next
sample through `q_dot`.

### Step 5: Commit
```
q_dot = alpha*C*(v - v_prev) - q_dot       (trapezoidal sample)
q_dot = (C/T)*(v - v_prev)                  (BE-fallback sample)
v_prev = v
```

## Inductors in the DK Formulation

Generated code builds every inductor as an augmented branch row (its `L` sits
in `C`, so its history is part of `H` and `q_dot`); code generation refuses a
companion-model kernel.

Capacitor-free rows (null(C)) carry no history under the charge form: their
equation each sample is KCL at `n+1`, so no period-2 mode lives there. The
deprecated library companion path (`DkKernel::from_mna` on an inductor deck,
used only by `LinearSolver`; removed in the next release, `COMPANION_MODELS.md`
last section) companion-models inductors
in the whole-system form (`g_eq = T/(2L)` in both `A` and `a_neg`,
`i_hist = 2*i_L[n]`), verified against an exact trapezoidal reference in
`dk_math_verification.rs`.

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

## Parasitic Cap Auto-Insertion

The DK method uses the trapezoidal rule: `A = G + (2/T)*C`. When the circuit has
nonlinear devices but no capacitors (`C = 0`), the system matrix degenerates to
`A = G` and the history matrix `H = alpha*C` to zero: every row is algebraic
and each sample is a static solve. The whole-system operator that the stability
discriminators evaluate degenerates to `S*(alpha*C - G) = -I` — every
eigenvalue at `z = -1`, the period-2 instability of purely resistive nonlinear
circuits under that form. Junction capacitance is also simply physical.

**Solution**: Call `MnaSystem::add_parasitic_caps()` before building the DK kernel.
This stamps 10pF (`PARASITIC_CAP = 10e-12`) across each physical device junction:

- Diode: anode-cathode
- BJT: base-emitter + base-collector
- JFET: gate-source + gate-drain
- MOSFET: gate-source + gate-drain
- Tube: grid-cathode + plate-cathode

These are physically realistic values for small-signal semiconductor packages. Caps
are stamped across junctions (not node-to-ground) to avoid introducing artificial
ground coupling. The build warns when it inserts them, naming each (see
[DEVICE_MODELS.md](DEVICE_MODELS.md#parasitic-cap-auto-insertion)).

With parasitic caps present, the C matrix is non-trivial, `A` has proper frequency
dependence via `(2/T)*C`, and `H*v_prev + q_dot` carries the junction
capacitors' history.

## Common Bugs
1. **Extra negation of K** -> NR diverges (positive feedback)
2. **Double-counting history** -> DC offset, instability
3. **Wrong N_i ground-reference sign** -> Wrong K sign for grounded devices
4. **Missing correction term** -> Wrong output amplitude

## Common Integration Pitfalls

> **Noise scope note:** on the DK path, parasitic-BJT RB/RC/RE are absorbed
> via K_eff (no internal nodes), so their thermal (rbb′) noise sources are
> NOT collected — a codegen warning fires. The nodal path models them fully.
> See NOISE.md "BJT parasitic-R thermal noise".

### Input Handling

**WRONG** — Building the kernel before the input conductance is in G:
```rust
let mna = MnaSystem::from_netlist(&netlist)?;
let kernel = DkKernel::from_mna(&mna, sample_rate)?;  // S computed WITHOUT input
// any later attempt to "inject" input conductance into the codegen state is too late
```

**RIGHT** — Stamp into MNA G matrix *before* building the kernel:
```rust
let mut mna = MnaSystem::from_netlist(&netlist)?;
// Stamp input conductance BEFORE building kernel
mna.g[input_node][input_node] += input_conductance;
let kernel = DkKernel::from_mna(&mna, sample_rate)?;  // S now includes input
let ir = CircuitIR::from_kernel(&kernel, &mna, &config)?;
let code = CodeGenerator::new(ir).generate()?;
```

**Why**: The DK kernel computes `S = A⁻¹` where `A = 2C/T + G`. If input
conductance isn't in G, it's not part of the circuit topology, and the
generated solver's per-sample input injection creates an inconsistent system.
This is the single most common SPICE-correlation failure for new circuits.

### Time-Varying Inputs

**WRONG** (stamps the source twice — the whole-system form's `b(n) + b(n+1)`
without its `-G*v_prev` history):
```rust
rhs[input_node] += 2.0 * input * G_in;            // wrong
rhs[input_node] += (input + input_prev) * G_in;   // also wrong (whole-system input stamp)
```

**RIGHT** (charge form: every source once, at `n+1`; `q_dot` carries the history):
```rust
rhs[input_node] += input * G_in;
```

The same holds for `.inject` sources (Norton `I`, Thevenin `V/R`) and
`.runtime` sources. `input_prev` survives only for the linear input ramp
across adaptive sub-steps and sub-sample-fire segments.

### Input Conductance Value for Validation

When validating against SPICE voltage source (ideal, 0Ω output):
- Use `input_conductance = 1.0` (1Ω, near-ideal)
- NOT the circuit's input resistor value (e.g., 10kΩ)
- The voltage source is the reference; we approximate it with low-Z Thevenin

## References
- Hack Audio Tutorial (Chapters 5-8): https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/
- Yeh & Smith. "Simulating guitar distortion circuits using wave digital and nonlinear state-space formulations." (2008)
