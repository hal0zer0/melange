# Linear Algebra Primitives

## Purpose

All matrix operations used in the melange solver pipeline. Reference this when modifying
any matrix computation in `dk.rs`, `dc_op.rs`, `solver.rs`, or generated code.

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

### 3. Gaussian Elimination (Flat) — `solver.rs:gauss_solve_inplace()`

Runtime M-dimensional NR solver. Row-major flat array format.

```
gauss_solve_inplace(a: &mut [f64], b: &mut [f64], n: usize) -> bool

Index formula: A[i][j] = a[i * n + j]

Forward elimination with partial pivoting:
  For k = 0..n:
    Find pivot: argmax_{i>=k} |a[i*n + k]|
    If pivot < SINGULARITY_THRESHOLD: return false
    Swap rows k and pivot_row
    Eliminate below: a[i*n+j] -= factor * a[k*n+j]

Back substitution:
  For i = n-1..0:
    b[i] = (b[i] - sum_{j>i} a[i*n+j] * b[j]) / a[i*n+i]
```

**When to use**: Runtime NR solver for M >= 3 (`solve_md()`).

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
| Any | Gaussian elimination (flat array) | solver.rs runtime |
| >16 | Not supported | MAX_M = 16 |

## Singularity Thresholds by Context

| Context | Threshold | Fallback |
|---------|-----------|----------|
| DK kernel inversion | 1e-15 | Error |
| DC OP LU decomposition | 1e-30 | None (try next strategy) |
| NR Gauss elimination | 1e-15 | Return current best guess |
| Transformer inversion | 1e-30 | Identity matrix + warning |
| Codegen sample-rate rebuild | 1e-30 | Identity matrix + flag |
| SM denominator | 1e-15 | scale = 0 (no correction) |
