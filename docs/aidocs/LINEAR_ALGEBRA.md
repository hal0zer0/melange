# Linear Algebra Primitives

## Purpose

All matrix operations used in the melange solver pipeline. Reference this when
modifying any matrix computation in `dk.rs`, `dc_op.rs`, `lu.rs`, or the
codegen-emitted templates.

> **Note on runtime removal.** The runtime `solver.rs:gauss_solve_inplace`,
> `solve_md`, and `CircuitSolver` paths have been deleted. Live linear-algebra
> code now lives in `dk.rs` (DK kernel inversion), `dc_op.rs` (DC OP LU),
> `lu.rs` (sparse + chord LU for codegen), `mna.rs` (small-matrix utilities),
> and the `templates/rust/state.rs.tera` template (`invert_n` for runtime
> sample-rate rebuilds).

## Constants

```
SINGULARITY_THRESHOLD = 1e-15  (dk.rs, solver.rs, codegen)
LU singularity pivot  = 1e-30  (dc_op.rs, mna.rs, state.rs.tera)
Condition warning     = 1e12   (dk.rs)
```

## Algorithms by Location

### 1. Equilibrated LU with Iterative Refinement — `dk.rs:invert_matrix()`

Primary matrix inversion for the DK kernel (`S = A^{-1}`). Five steps:

```
Step 1 — Equilibration:
  D = diag(1 / sqrt(|A[i][i]|))
  A_eq = D * A * D
  (normalizes diagonal, improves conditioning)

Step 2 — LU factorization with partial pivoting:
  For k = 0..N:
    Find pivot row p = argmax_{i>=k} |A_eq[i][k]|
    Swap rows k, p (track permutation)
    For i = k+1..N:
      A_eq[i][k] /= A_eq[k][k]        (L factor below diagonal)
      For j = k+1..N:
        A_eq[i][j] -= A_eq[i][k] * A_eq[k][j]  (U factor above diagonal)

Step 3 — Solve each column via forward/backward substitution:
  For each column c of identity:
    Forward:  L * y = P * e_c
    Backward: U * x = y

Step 4 — Iterative refinement (one round):
  r = P*e_c - (P*A_eq)*x
  Solve LU * dx = r
  x += dx
  (adds ~6 correct digits for cond > 1e9)

Step 5 — Undo equilibration:
  S = D * S_eq * D
```

**When to use**: DK kernel construction (A matrix inversion). Critical for circuits
with large inductors where cond(A) > 1e9.

### 2. LU Decomposition — `dc_op.rs:lu_decompose()` + `lu_solve()`

Standard LU with partial pivoting for DC operating point NR iterations.

```
lu_decompose(A) -> (LU, pivot)
  Singularity check: max_val < 1e-30 -> None
  Returns in-place LU matrix + permutation vector

lu_solve(LU, pivot, b) -> x
  Forward substitution:  L * y = P * b
  Backward substitution: U * x = y
```

**When to use**: Each NR iteration in `solve_dc_operating_point()`. O(N^2) per solve
vs O(N^3) per inversion. LU factored once per iteration, reused for solve.

### 3. Gaussian Elimination (Codegen, Inlined) — `rust_emitter/dk_solver.rs:generate_gauss_elim`

Codegen emits inline Gaussian elimination with partial pivoting for the
M-dimensional NR Jacobian. The shape of the emitted code depends on the routing
mode:

- **DK Schur path** — `generate_gauss_elim` emits a fully-unrolled M×M solver
  (M ≤ 16). Used when the kernel has small M and the K matrix is well-conditioned.
- **Nodal Schur path** — `generate_schur_gauss_elim` emits a slightly different
  structure that consumes the precomputed `S = A^{-1}` and solves the M×M system
  via the same Gaussian elimination shape.
- **Nodal full LU path** — for K≈0, ill-conditioned K, or large M, the codegen
  emits a per-iteration N×N LU (`lu_factor` / `lu_back_solve`) with chord-method
  refactor cadence and AMD-ordered sparse LU. See `lu.rs` and `chord_method.md`.

All three paths produce straight-line code with no allocations. Singular pivots
fall through to a "best guess" return at `SINGULARITY_THRESHOLD = 1e-15`.

**When to use**: not callable directly — emitted automatically by the codegen
based on the circuit's routing decision (`--solver auto|dk|nodal`).

### 4. Gauss-Jordan Inversion — `mna.rs:invert_small_matrix()`

Small matrix inversion for multi-winding transformer inductance matrices.

```
invert_small_matrix(A) -> A^{-1}

Build augmented [A | I]
Forward elimination with partial pivoting
Back-substitute to get [I | A^{-1}]
Singularity fallback: returns identity with log::warn
```

**When to use**: Transformer group inductance matrix (typically 2x2 to 4x4).

### 5. Gauss-Jordan (Generated) — `state.rs.tera:invert_n()`

Same algorithm as #4 but emitted in generated code for runtime sample-rate changes.

```
fn invert_n(a: [[f64; N]; N]) -> ([[f64; N]; N], bool)

Returns (inverse, is_singular) tuple.
Singular fallback: returns identity matrix.
```

**When to use**: `set_sample_rate()` recomputes `S = A^{-1}` at new rate.

### 6. Cramer's Rule (2x2) — Codegen M=2 NR

```
det = J[0][0]*J[1][1] - J[0][1]*J[1][0]
if |det| < SINGULARITY_THRESHOLD: fallback
inv_det = 1 / det
delta[0] = inv_det * (J[1][1]*f[0] - J[0][1]*f[1])
delta[1] = inv_det * (-J[1][0]*f[0] + J[0][0]*f[1])
```

**When to use**: Generated code for M=2 circuits (single BJT, single JFET, etc).

### 7. Direct Division (1x1) — Codegen M=1 NR

```
if |J| < SINGULARITY_THRESHOLD: fallback
delta = f / J
```

**When to use**: Generated code for M=1 circuits (single diode).

## Sherman-Morrison Rank-1 Update

See [SHERMAN_MORRISON.md](SHERMAN_MORRISON.md) for full derivation.

Core formula for conductance change delta_g:
```
S' = S - scale * (su * su^T)
scale = delta_g / (1 + delta_g * u^T * S * u)
su = S * u
```

## Chord Method + Sparse LU (Nodal Full-LU Path)

For circuits routed to the nodal full-LU path (K≈0, ill-conditioned K, or
spectral radius > 0.999), the codegen emits a per-iteration N×N LU solve.
Three optimizations stack to keep this real-time:

1. **Chord method** — `lu_factor` runs only every `CHORD_REFACTOR=5` NR
   iterations; intervening iterations use `lu_back_solve` (O(N²)) on the
   stale factorization with the saved `chord_j_dev`. This trades a few
   extra NR iterations for ~5× factorization savings.

2. **Cross-timestep persistence** — `chord_lu`, `chord_j_dev`, `chord_valid`
   live in `CircuitState`. Smooth audio signals reuse the previous sample's
   factorization across many timesteps, dropping factorizations to near-zero.

3. **Compile-time sparse LU** — AMD ordering and symbolic factorization run
   at codegen time. The emitter writes `sparse_lu_factor(a, d)` /
   `sparse_lu_back_solve(a_lu, d, b)` as straight-line code on the original
   indices (no runtime permutation). Pultec example: 536 factor FLOPs vs
   ~22973 dense (43× reduction). See `chord_method.md` in memory.

Source: `crates/melange-solver/src/lu.rs` and the emit sites in
`crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs`.

## Condition Number Estimation

```
cond(A) ~= ||A||_inf * ||A^{-1}||_inf

||M||_inf = max_i (sum_j |M[i][j]|)   (infinity norm = max absolute row sum)

Warning threshold: cond > 1e12
```

Used in `dk.rs` after computing S = A^{-1}. Not a hard error; diagnostic only.

## Matrix Utilities — `dk.rs`

```
mat_mul(A, B) -> C         C[i][j] = sum_k A[i][k] * B[k][j]
mat_vec_mul(A, x) -> y     y[i] = sum_j A[i][j] * x[j]
infinity_norm(A) -> f64    max absolute row sum
flatten_matrix(M, r, c)    2D -> 1D row-major (index = row * cols + col)
```

## NR Linear Solve Selection by M

| M | Method | Location |
|---|--------|----------|
| 1 | Direct division | Codegen template |
| 2 | Cramer's rule | Codegen template |
| 3-16 | Gaussian elimination (unrolled) | Codegen template |
| Any (full LU path) | Sparse LU + chord refactor | `lu.rs` + codegen-emitted straight-line code |
| >16 | Not supported (DK/Nodal Schur paths) | MAX_M = 16 |

## Singularity Thresholds by Context

| Context | Threshold | Fallback |
|---------|-----------|----------|
| DK kernel inversion (`dk.rs:invert_matrix`) | 1e-15 | Error |
| DC OP LU decomposition (`dc_op.rs:lu_decompose`) | 1e-30 | None (try next strategy) |
| Codegen NR Gauss elimination | 1e-15 | Return current best guess |
| Sparse LU (codegen full-LU path) | 1e-15 | NaN reset, restore from DC OP |
| Transformer inversion (`mna.rs:invert_small_matrix`) | 1e-30 | Identity matrix + warning |
| Codegen sample-rate rebuild (`state.rs.tera:invert_n`) | 1e-30 | Identity matrix + flag |
| SM denominator | 1e-15 | scale = 0 (no correction) |
