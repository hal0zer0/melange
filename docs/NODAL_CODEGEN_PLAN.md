# NodalSolver Codegen Plan

## Motivation

melange's purpose is compiling SPICE netlists into optimized Rust code. The current
DK method codegen works well for simple circuits (M≤4, no large inductors) but fails
for transformer-coupled circuits like the Pultec EQP-1A (ill-conditioned A matrix,
K diagonal issues). The NodalSolver handles these circuits but only exists as a
runtime solver — its features don't reach the generated code.

This plan adds NodalSolver codegen alongside DK codegen, auto-selected based on
circuit properties. Every improvement to the solver math ships directly to users.

## Architecture: Coexist, Don't Replace

| Path | Space | Per-sample cost | Best for |
|------|-------|----------------|----------|
| DK codegen | M-dim NR | O(N² + M³) | Simple circuits, M≤4, no large L |
| Nodal codegen | N-dim NR | O(N³) | Transformers, global NFB, large L, M>16 |

Auto-select: try DK first. If it fails (singular A, positive K diagonal, large inductors),
fall back to nodal. CLI flag `--solver dk|nodal|auto` for manual override.

## Generated Code Structure (Nodal Path)

### Constants
```rust
const N: usize = 22;       // n_nodal (augmented)
const N_NODES: usize = 13; // circuit nodes only
const N_AUG: usize = 14;   // n + num_vs (VS/VCVS boundary)
const M: usize = 8;        // nonlinear dimensions

const G: [[f64; N]; N] = [...];        // Conductance + inductor KCL/KVL
const C: [[f64; N]; N] = [...];        // Capacitance + inductance
const N_V: [[f64; N]; M] = [...];      // Voltage extraction
const N_I: [[f64; N]; M] = [...];      // Current injection
const RHS_CONST: [f64; N] = [...];     // DC sources (trapezoidal)
const RHS_CONST_BE: [f64; N] = [...];  // DC sources (backward Euler)
const DC_OP: [f64; N] = [...];         // DC operating point
const DC_NL_I: [f64; M] = [...];       // DC nonlinear currents
// Per-device constants (same as DK)
```

### State
```rust
struct CircuitState {
    a: [[f64; N]; N],            // A = G + alpha*C (trapezoidal)
    a_neg: [[f64; N]; N],        // alpha*C - G
    a_be: [[f64; N]; N],         // G + (1/T)*C (BE fallback)
    a_neg_be: [[f64; N]; N],     // (1/T)*C
    v_prev: [f64; N],
    i_nl_prev: [f64; M],
    input_prev: f64,
    // NR working buffers (pre-allocated, no heap)
    rhs: [f64; N],
    v_nl: [f64; M],
    i_nl: [f64; M],
    j_dev: [[f64; M]; M],
    g_aug: [[f64; N]; N],
    rhs_work: [f64; N],
    // DC blocking, diagnostics
}
```

### Per-Sample Flow
```
1. Build RHS = RHS_CONST + A_neg * v_prev + N_i * i_nl_prev + input
2. NR loop (trapezoidal, max_iter iterations):
   a. v_nl = N_v * v
   b. Evaluate devices → i_nl, j_dev
   c. g_aug = A - N_i * J_dev * N_v
   d. rhs_work = rhs + N_i * (i_nl - J_dev * v_nl)
   e. LU solve: v_new = g_aug⁻¹ * rhs_work
   f. SPICE pnjlim/fetlim + ngspice 10V node damping
   g. Damped update: v += alpha * (v_new - v)
   h. RELTOL convergence check
3. BE fallback if trapezoidal NR failed
4. Update state: v_prev, i_nl_prev
5. DC blocking + output extraction
```

### LU Solve Strategy
- N ≤ 50: inline Gaussian elimination with partial pivoting
- Fixed-size arrays, no allocation
- Same algorithm as DK's M=3..16 Gauss path, scaled to N

## Shared Code (Not Duplicated)

These are identical between DK and Nodal codegen:
- Device model functions (diode_current, bjt_ic, tube_ip, etc.)
- SPICE voltage limiting (pnjlim, fetlim, pn_vcrit)
- Oversampling (polyphase half-band IIR wrapper)
- DC blocking filter
- safe_exp, sanitize_input, clamp_output
- Plugin template (nih-plug boilerplate)

## Potentiometers

DK uses Sherman-Morrison (O(N²) per pot change via precomputed S*u vectors).
Nodal uses direct G re-stamp: unstamp old conductance, stamp new, recompute
A = G + alpha*C. This is O(N²) and happens only on pot change, not per sample.
Simpler than SM and negligible cost since we LU every sample anyway.

## Switches

Same as DK: `set_switch_N()` modifies G/C, calls `rebuild_matrices()`.
For nodal: rebuild_matrices recomputes A/A_neg from G+C (no S/K inversion needed).

## IR Changes

Add to CircuitIR:
```rust
pub enum SolverMode { Dk, Nodal }
```

Add `CircuitIR::from_mna()` that bypasses DkKernel entirely for the nodal path.
Builds augmented G/C, computes A/A_neg/A_be/A_neg_be, runs DC OP, resolves devices.

The existing `from_kernel()` remains for DK path. Both return `CircuitIR` with
different `solver_mode`.

## Emitter Changes

`RustEmitter::emit()` dispatches on `ir.solver_mode`:
- `Dk` → existing `emit_dk()` (unchanged)
- `Nodal` → new `emit_nodal()`

New templates:
- `nodal_constants.rs.tera` — A, A_be, A_neg_be, RHS_CONST_BE (no S, K, S_NI)
- `nodal_state.rs.tera` — NR working buffers, LU storage
- `nodal_process_sample.rs.tera` — full-nodal NR with BE fallback
- `nodal_nr_loop.rs.tera` — Jacobian stamp, LU solve, limiting, damping
- `nodal_rebuild_matrices.rs.tera` — recompute A/A_neg from G+C

## Implementation Phases

### Phase 1: Infrastructure
- SolverMode enum in CircuitIR
- CircuitIR::from_mna() (builds IR without DkKernel)
- A/A_be/A_neg_be matrices in Matrices struct
- CodeGenerator::generate_nodal()

### Phase 2: Nodal Emitter
- emit_nodal() in RustEmitter
- New templates: constants, state, process_sample, NR loop
- Inline LU solver function
- BE fallback, RELTOL, node damping
- Reuse device model templates

### Phase 3: Features
- Pot support (direct G re-stamp)
- Switch support (rebuild_matrices)
- Oversampling
- set_sample_rate
- Sparsity-aware Jacobian stamping

### Phase 4: CLI + Plugin
- --solver flag on melange compile
- Auto-detection (large inductors → nodal)
- Plugin template for nodal-generated code

### Phase 5: Validation
- SPICE validation for nodal-generated circuits
- Performance benchmarks (DK vs Nodal)
- Pultec as first nodal-codegen circuit

## Performance

For Pultec (N=22, M=8) at 48kHz:
- DK: ~3K FLOP/sample → 144M FLOP/s → <1% of one core
- Nodal: ~30K FLOP/sample → 1.44G FLOP/s → ~5% of one core
- 10x more expensive but well within real-time budget
- With 4x oversampling: ~20% of one core (still feasible)

For simple circuits (N=3, M=1): DK stays the fast path.

## Key Principle

Codegen is the product. Runtime solvers are R&D tools. Every solver
improvement must reach codegen to matter. With two codegen paths
(DK + Nodal), both auto-selected, any valid circuit "just works."
