# DC Operating Point Solver

## Purpose

Computes the steady-state bias point of circuits with nonlinear devices (diodes, BJTs)
by solving the nonlinear system:

```
G_dc · v = b_dc + N_i · i_nl(N_v · v)
```

Without a DC OP solver, circuits with DC bias (e.g., BJT amplifiers powered by VCC)
start from v=0 (all devices in cutoff), producing no output. SPICE always computes
a DC operating point before transient analysis.

## Module

`crates/melange-solver/src/dc_op.rs`

Called from the codegen pipeline (`CircuitIR::from_kernel()`). The pre-removal
runtime entry-point (`CircuitSolver::initialize_dc_op()`) no longer exists.

## References

- SPICE DC analysis: Pillage & Rohrer Ch. 4
- Source stepping: Nagel (1975), "SPICE2: A Computer Program to Simulate Semiconductor Circuits"
- Gmin stepping: ngspice manual §15.3

## Newton-Raphson Companion Formulation

At each NR iteration, linearize the nonlinear devices as Norton equivalents:

```
F(v) = G_dc · v - b_dc - N_i · i_nl(N_v · v) = 0

dF/dv = G_dc - N_i · J_dev · N_v

G_aug = G_dc - N_i · J_dev · N_v       (augmented conductance)
rhs   = b_dc + N_i · (i_nl - J_dev · v_nl)  (companion Norton current)
v_new = G_aug^{-1} · rhs               (solve)
```

### Sign Convention (Critical!)

The Jacobian stamping uses **subtraction**:
```
G_aug = G_dc - N_i · J_dev · N_v
```

This is correct because:
- `F(v) = G·v - b - N_i·i_nl = 0`
- `dF/dv = G - N_i · J_dev · N_v` (derivative of `-N_i·i_nl` w.r.t. `v`)
- N_i[anode] = -1, so `-(-1) · g_d = +g_d` stamps device conductance **positively**
- This gives correct diagonal dominance for convergence

**Do NOT use addition** (`G_dc + N_i · J_dev · N_v`) — that would double-count
the device conductance with wrong sign, causing divergence.

### Voltage Limiting

Each NR iteration uses junction-aware logarithmic voltage limiting:
```rust
let vt = 0.026; // thermal voltage
let delta = v_new[i] - v[i];
let limited = if delta.abs() < vt {
    delta  // small steps pass unchanged
} else {
    delta.signum() * vt * (delta.abs() / vt + 1.0).ln()
};
v[i] += limited;
```

This compresses large voltage steps (e.g. 300V → ~0.3V) while passing small
steps (~26mV) nearly unchanged — much more effective than a flat clamp for
circuits with both low-voltage junctions and high-voltage supply nodes.

### Linear System Solve

The DC OP solver uses LU decomposition with partial pivoting to solve
`G_aug * v = rhs` each NR iteration, rather than full matrix inversion.
This is both faster (O(N²) per solve vs O(N³) per inversion) and more
numerically stable.

## Convergence Strategies

The solver tries three strategies in order:

### 1. Direct NR (DcOpMethod::DirectNr)

Start from the linear DC OP (no nonlinear devices) and iterate NR.
Works for simple circuits (single diode with VCC).

### 2. Source Stepping (DcOpMethod::SourceStepping)

Scale all DC sources from 0 → full value in `source_steps` stages (default 50).
At each stage, run NR to convergence, then warm-start the next stage.
Start from v=0 (not linear guess).

Works for BJT amplifier bias networks where direct NR fails because
the linear initial guess puts BJT junctions too far from their operating point.

### 3. Gmin Stepping (DcOpMethod::GminStepping)

Add conductance `gmin` across each nonlinear device's controlling nodes.
Ramp logarithmically from `gmin_start` (1e-2) to `gmin_end` (1e-12).
Final solve with gmin=0.

Works for circuits where source stepping fails (rare).

### 4. Fallback (DcOpMethod::Failed)

Return the linear DC OP with `converged: false`. The solver will still work
but start from a wrong bias point, producing transient artifacts.

## DC System Construction

At DC steady state:
- **Resistors**: Normal G-matrix stamps (already in `mna.g`)
- **Capacitors**: Open circuit (C not stamped — `i_C = C·dv/dt = 0` at DC)
- **Inductors**: Short circuit (`VS_CONDUCTANCE` between terminals)
- **Voltage sources**: Norton equivalent (`VS_CONDUCTANCE` + current injection)
- **Input**: `1/input_resistance` added to `g_dc[input_node][input_node]`

### BJT Junction Cap Linearization

When BJT charge storage parameters are specified (CJE, CJC, TF), the junction
capacitances are evaluated at the DC operating point and stamped into the MNA C matrix:

- **Depletion cap**: `Cj = CJ0 / (1 - Vj/VJ)^MJ` (with FC=0.5 linear extension for forward bias)
- **Diffusion cap**: `Cd = TF * |Ic| / VT`
- Total B-E cap: `CJE_linearized + Cd` stamped across base-emitter nodes
- Total B-C cap: `CJC_linearized` stamped across base-collector nodes

The DK framework requires linear C, so these caps are fixed at DC OP values.
This happens during `CircuitIR::from_kernel()` after the DC OP solve.

### Parasitic Caps

If the circuit has nonlinear devices but zero capacitors, `MnaSystem::add_parasitic_caps()`
should be called before building the DK kernel. This inserts 10pF across each device
junction (see [DEVICE_MODELS.md](DEVICE_MODELS.md#parasitic-cap-auto-insertion)),
ensuring the C matrix is non-trivial for stable trapezoidal integration. Parasitic caps
are inserted before the DC OP solve so that the linearized junction caps (if any) add
to the parasitic base.

## Device Evaluation

Uses `DeviceSlot` params from `codegen::ir`:

- **Diode**: `i = IS * (exp(v/N_VT) - 1)`, `g = (IS/N_VT) * exp(v/N_VT)`
- **BJT**: Ebers-Moll transport model with polarity sign (+1 NPN, -1 PNP)
  - `Ic = sign * IS * (exp(Vbe_eff/VT) - exp(Vbc_eff/VT)) - sign * (IS/BR) * (exp(Vbc_eff/VT) - 1)`
  - `Ib = sign * (IS/BF) * (exp(Vbe_eff/VT) - 1) + sign * (IS/BR) * (exp(Vbc_eff/VT) - 1)`
- **JFET**: 2D Shichman-Hodges with triode + saturation regions, channel-length modulation (lambda)
  - `Id = sign * IDSS * f(Vgs, Vds, Vp) * (1 + lambda*|Vds|)`, `Ig ≈ 0`
  - N-channel (sign=+1) / P-channel (sign=-1)
- **MOSFET**: 2D Level 1 SPICE with triode + saturation regions, channel-length modulation (lambda)
  - `Id = sign * KP * f(Vgs, Vds, Vt) * (1 + lambda*|Vds|)`, `Ig = 0`
  - N-channel (sign=+1) / P-channel (sign=-1)
- **Tube**: 2D Koren plate current (with Early-effect lambda) + Leach grid current
  - `Ip = Ip_koren * (1 + lambda*Vpk)` where `Ip_koren = E1^ex / Kg1`; `Ig = ig_max * (vgk/vgk_onset)^1.5` for vgk > 0
- **Clamping**: `safe_exp(x) = x.clamp(-40, 40).exp()` matching codegen/runtime

## API

### Types

```rust
pub struct DcOpConfig {
    pub tolerance: f64,        // 1e-9
    pub max_iterations: usize, // 200
    pub source_steps: usize,   // 10
    pub gmin_start: f64,       // 1e-2
    pub gmin_end: f64,         // 1e-12
    pub gmin_steps: usize,     // 10
    pub input_node: usize,     // 0-indexed
    pub input_resistance: f64, // ohms
}

pub struct DcOpResult {
    pub v_node: Vec<f64>,   // N-vector: node voltages
    pub v_nl: Vec<f64>,     // M-vector: controlling voltages (N_v · v)
    pub i_nl: Vec<f64>,     // M-vector: device currents at bias point
    pub converged: bool,
    pub method: DcOpMethod,
    pub iterations: usize,
}

pub enum DcOpMethod { Linear, DirectNr, SourceStepping, GminStepping, Failed }
```

### Entry Point

```rust
pub fn solve_dc_operating_point(
    mna: &MnaSystem,
    device_slots: &[DeviceSlot],
    config: &DcOpConfig,
) -> DcOpResult
```

## Integration

### Codegen Path

In `CircuitIR::from_kernel()`:
```rust
let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);
// dc_result.v_node → dc_operating_point constant
// dc_result.i_nl  → dc_nl_currents → DC_NL_I constant
```

Generated code gets a `DC_NL_I` constant that initializes `i_nl_prev`:
```rust
pub const DC_NL_I: [f64; M] = [1.234e-3, 5.678e-6];  // From DC OP solve

// In Default impl:
i_nl_prev: DC_NL_I,  // Start from bias point, not zeros

// In reset():
self.i_nl_prev = DC_NL_I;
```

### Runtime Path

**Removed.** The runtime `CircuitSolver`/`NodalSolver`/`initialize_dc_op()` API has
been deleted. All circuit processing now flows through the codegen pipeline above
(`CircuitIR::from_kernel` → emitted `DC_NL_I` constant). If you find references to
`CircuitSolver::initialize_dc_op` in older docs or examples, they are dead code.

## DK Trapezoidal Steady State

The DK solver uses **trapezoidal** integration for both linear and nonlinear currents.
The net nonlinear contribution is `N_i * (i_nl[n+1] + i_nl[n])`, which is a proper
trapezoidal average matching the linear discretization.

**Warm-up**: The generated `warmup()` method runs after construction and `reset()`.
Two phases:

1. **Low-rate DC settling** (nodal path only, when `DC_OP_CONVERGED = false`):
   `rebuild_matrices(200.0)` → 1000 silent samples (5 seconds circuit time) →
   `rebuild_matrices(target_rate)`. This charges coupling caps (e.g. 22µF × 27K =
   0.6s RC) that the failed DC OP left uncharged. The DC steady state is rate-
   independent (`A - A_neg = 2G`, no rate terms), so values found at 200 Hz are
   exact at any target rate. The settled state is cached in `dc_operating_point`
   and `settled_i_nl` — the expensive phase runs only once; subsequent `reset()`
   calls reuse the cached values.

2. **Standard warmup**: 50 silent samples at the target rate. Settles the DK/nodal
   solver from any residual mismatch and pulls high-gain op-amp circuits into the
   physically correct basin of attraction.

Emitted from `nodal_emitter.rs` (grep `warmup`). The DK path (`dk_emitter.rs`)
only emits the standard 50-sample warmup (DK circuits have well-conditioned DC OP).

## Expected DC OP Values (Verification)

### Single Diode + VCC
```
VCC=5V, R=1k, D1 to GND
V(anode) ≈ 0.65V
I_D ≈ (5 - 0.65) / 1k ≈ 4.35mA
```

### BJT Common Emitter (12V, 2N2222A-like)
```
VCC=12V, R1=100k, R2=22k (base divider), RC=6.8k, RE=1k
V(base) ≈ 2.16V  (divider: 12 * 22k / (100k + 22k))
V(emit) ≈ 1.51V  (V(base) - 0.65V)
I_C ≈ 1.51mA     (V(emit) / RE)
V(coll) ≈ 1.73V  (12 - 1.51e-3 * 6800)
```

## Common Failures

| Symptom | Cause | Fix |
|---------|-------|-----|
| NR diverges (oscillating v) | Wrong Jacobian sign (using + instead of -) | Use `G_aug = G_dc - N_i·J_dev·N_v` |
| Converges to wrong point | Linear initial guess too far | Source stepping will fix automatically |
| PNP BJT wrong polarity | Missing sign parameter | Check `is_pnp` flag in DeviceParams |
| V_node all zeros | Input conductance not stamped | Check `config.input_resistance > 0` |
| BJT oscillation in transient | Mixed integration mismatch | Fixed: DK now uses trapezoidal for nonlinear currents |
