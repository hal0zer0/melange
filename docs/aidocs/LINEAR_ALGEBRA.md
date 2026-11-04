# Linear Algebra Primitives

## Purpose

All matrix operations used in the melange solver pipeline. Reference this when
modifying any matrix computation in `dk.rs`, `dc_op.rs`, `lu.rs`, or the
codegen-emitted templates.

> **Note on runtime removal.** The runtime `solver.rs:gauss_solve_inplace`,
> `solve_md`, and `CircuitSolver` paths have been deleted. Live linear-algebra
> code now lives in `dk.rs` (DK kernel inversion), `dc_op.rs` (DC OP LU),
> `lu.rs` (sparse + chord LU for codegen), `mna/helpers.rs` (small-matrix utilities),
> and the `templates/rust/state.rs.tera` template (`invert_n` for runtime
> sample-rate rebuilds).

## Constants

```
SINGULARITY_THRESHOLD = 1e-15  (dk.rs, codegen)
LU singularity pivot  = 1e-30  (dc_op.rs, mna/helpers.rs, state.rs.tera)
Condition warning     = 1e13   (dk.rs)
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
  (M ≤ 32, MAX_M). Used when the kernel has small M and the K matrix is well-conditioned.
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

### 4. Gauss-Jordan Inversion — `mna/helpers.rs:invert_small_matrix()`

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

For circuits routed to the nodal full-LU path (structural: saturating
inductor or behavioral source; or by conditioning: K≈0, a positive K
diagonal, ill-conditioned K or S, or an unstable Schur prediction — the
whole-system `spectral_radius_s_aneg` above 1.002 on a well-conditioned K;
see `emit_nodal` in `rust_emitter/nodal_emitter/mod.rs`), the codegen emits a
per-iteration N×N LU solve.
Three optimizations stack to keep this real-time:

1. **Chord method** — `lu_factor` runs only every `CHORD_REFACTOR=5` NR
   iterations; intervening iterations use `lu_back_solve` (O(N²)) on the
   stale factorization with the saved `chord_j_dev`. This trades a few
   extra NR iterations for ~5× factorization savings.

2. **Cross-timestep persistence** — `chord_lu`, `chord_dr`, `chord_dc`,
   `chord_perm`, `chord_j_dev`, `chord_valid` live in `CircuitState` (the
   backward-Euler instance has its own `chord_be_*` set). Smooth audio signals
   reuse the previous sample's factorization across many timesteps, dropping
   factorizations to near-zero. The Newton loop factors into and back-solves
   from the state fields **in place** (`NewtonSite::chord` names the prefix);
   only `chord_j_dev`, the flags and `chord_body_gmb` are loaded into locals
   per sample. Copying the N×N factor to a local and back every sample was
   2·8·N² bytes of memmove per sample (92 KB at N = 76) and about half of
   that circuit's per-sample time. A failed `lu_factor` leaves a
   half-eliminated matrix in the storage and sets `chord_valid = false`, as
   does the NaN/magnitude reset, so the next solve refactors before any
   back-solve reads it.

3. **Compile-time sparse LU** — AMD ordering and symbolic factorization run
   at codegen time. The emitter writes `sparse_lu_factor(a, d)` /
   `sparse_lu_back_solve(a_lu, d, b)` as straight-line code on the original
   indices (no runtime permutation). Passive-EQ example: 536 factor FLOPs vs
   ~22973 dense (43× reduction). See `chord_method.md` in memory. The
   schedule is built for every full-LU build whose stamp geometry is
   complete (`build_nodal.rs`), `M = 0` behavioral and saturating-inductor
   builds included; N < 8 or a G_aug density of 40 % or more keeps the
   dense kernels.

   The static row swaps are decided on the G_aug pattern **without the GMIN
   floor** (`lu::plan_sparse_lu`). GMIN = 1e-12 sits on every node diagonal,
   so on the floored pattern no node row looks structurally zero, and a node
   whose diagonal holds nothing but the floor (an unnamed shared-core
   transformer node: G row {GMIN, ±1, ±1}, no C entry, no device column, as
   on `steve-1073-output` rows 15 and 16) becomes a 1e-12 static pivot that
   the growth check rejects on every factorization. A diagonal counts as
   floor-only when it held no G stamp before the floor, still holds exactly
   the floor, has no C stamp and no device or behavioral stamp. The floor
   stays in the matrix values and the elimination schedule is traced on the
   full pattern; only the swap decision ignores it. Setter positions (pots,
   wipers, `.runtime R`, switch components) count as stamps whatever their
   nominal value, so their diagonals are never floor-only
   (`lu::floor_only_diagonals`). The swap decision does not trace fill-in,
   and it must not: a fill-aware variant (swap only a row whose diagonal is
   still empty at its elimination step) was measured and reproduces the old
   swap sets, because the structural fill that reaches a floor-only row
   cancels numerically and the runtime pivot is GMIN again;
   `steve-1073-output` stays rejected on every factorization under it. The
   growth check guards every factorization either way.

4. **Exact-zero skips in the dense kernels** — `lu_solve`, `lu_factor` and
   `lu_back_solve` skip a row update whose multiplier `a[row][col]` is an
   exact zero and a substitution term whose L/U entry is an exact zero. This
   is bit-identical to the full loops for finite data (the skipped terms are
   `±0`, which cannot change a nonzero value or a `+0` running sum, and a
   zero's sign never reaches a nonzero value; the pivot is already rejected
   below 1e-30 before the elimination), so the circuit matrix's sparsity is
   recovered even where no symbolic schedule exists. The derivation is in
   the `emit_nodal_lu_factor` doc comment.

5. **G_aug built on the touched positions only** — a chord refactor that
   takes the sparse schedule writes `chord_lu` (or `chord_be_lu`) from
   `state.a` (`state.a_be`) only at the positions `sparse_lu_factor` and
   `sparse_lu_back_solve` read or write (`equil::sparse_lu_touch_pattern`:
   the equilibration pattern, plus every pivot, L/U operand and fill target of
   the elimination, the growth-check pattern and the back-solve's entries,
   mapped back through the static row swaps), then stamps the device
   Jacobian. Positions outside that set are never read on the sparse path, so
   they may hold anything (a previous dense factorization, a row the swaps
   moved). A rejected sparse factor rebuilds the full N² matrix with the same
   operations before the dense `lu_factor`, instead of restoring a saved copy.
   Both are bit-identical to copying all of `state.a` and saving the stamped
   matrix. The two N² copies cost 17 % of the cycles of an N = 84 build that
   refactors on most samples. Builds without the structural equilibration
   pattern (behavioral sources, N < 20, or denser than 18 %) keep the full
   copy, because their equilibration and growth check sweep all N².

6. **Straight-line equilibration and growth check** — with the structural
   pattern, `sparse_lu_factor` emits its row/column max-norm equilibration
   and its growth-factor sweep as straight-line code with constant indices
   (`equil::emit_equilibration_unrolled`, `emit_growth_check_unrolled`)
   instead of loops over the `(u16, u16)` index table. Each row's and each
   column's maximum is a register (the table loop carried it through a stack
   array, a store-to-load chain per entry, with two bounds checks per
   access); the growth maximum runs on four independent chains. A running
   max by `if v > m { m = v; }` from 0 does not depend on the order or
   grouping of its inputs (NaN is skipped either way), and every entry is
   still scaled by its row factor and then its column factor, so the factors
   and the scaled matrix are bit-identical. `lu_factor` and `lu_solve` (the
   dense fallback and the sub-step and pin solves) keep the table loop.

7. **Packed storage of the sparse factor** — with the structural pattern,
   the sparse factor does not live in the N x N `chord_lu` (`chord_be_lu`)
   but in `chord_lu_pk` (`chord_be_lu_pk`), a `[f64; LU_SLOTS]` with one
   slot per touch-pattern position (item 5), in that row-major
   input-coordinate order (`equil::LuPack`). The G_aug build, the device,
   body-effect and saturating-inductor stamps, `sparse_lu_factor` and
   `sparse_lu_back_solve` address the slots; the static row swaps become an
   index remap fixed at codegen time (after the swaps, position `(i, j)`
   holds input row `row_src[i]`), so no rows move at runtime. The
   operations, their order and their operands are unchanged, so every
   result is bit-identical; what changes is the memory the hot loop walks
   (an N = 84 build: 488 slots, 3.9 KB, against a 56 KB array that does not
   fit in L1). The N x N array stays the dense fallback's storage, which a
   rejected sparse factor rebuilds in full as before. Builds that stamp
   saturating inductors keep the N x N layout: on the one corpus deck with
   them (N = 73, 16 saturating inductors, a refactor every iteration) the
   packed layout measured 3 % slower despite 62 % fewer L1 misses, with the
   FP-scheduler stalls doubled; the mechanism is not pinned down
   (`LuPack::for_build`). A stamp whose index is
   neither a literal nor a known row constant, or that lands outside the
   touch pattern, panics at codegen instead of going where the factor cannot
   see it.

8. **Reciprocal pivots** — `sparse_lu_factor` forms each pivot's
   reciprocal once, after that column's 1e-30 tiny-pivot test, and the
   column's L factors multiply by it instead of dividing; pivots with no L
   factor below them take theirs after the elimination, untested, as the
   back-solve divided by them untested before. The growth-factor sweep reads
   the true pivots; only after it passes does the factor write the
   reciprocals onto the diagonal, where `sparse_lu_back_solve`, the factor's
   one consumer, multiplies by them. A rejected factor returns before that
   write, and the dense fallback rebuilds its own matrix, so it never sees a
   reciprocal. This changes bits: `fl(x * fl(1/p))` carries up to 2u
   relative error against u for `fl(x / p)`, one extra rounding per L
   multiplier and per back-solve pivot step. On identical input matrices the
   accept/reject decision is the same and the growth factor agrees to about
   growth x u (4.5e-12 relative at growth 2.4e4, the corpus worst case).
   Rendered output moves at the solve's forward-error scale, and where an
   iterate sits at a certification threshold the Newton path can take one
   iteration more or fewer.

Source: `crates/melange-solver/src/lu.rs` and the emit sites in
`crates/melange-solver/src/codegen/rust_emitter/nodal_emitter/` (`lu.rs` for the
emitted factor and back-solve, `full_lu_newton.rs` for the chord loop).

## Condition Number Estimation

```
cond(A) ~= ||A||_inf * ||A^{-1}||_inf

||M||_inf = max_i (sum_j |M[i][j]|)   (infinity norm = max absolute row sum)

Warning threshold: cond > 1e13
```

Used in `dk.rs` after computing S = A^{-1}. Not a hard error; diagnostic only.

## Matrix Utilities — `dk.rs`

```
mat_mul(A, B) -> C         C[i][j] = sum_k A[i][k] * B[k][j]
mat_vec_mul(A, x) -> y     y[i] = sum_j A[i][j] * x[j]   (test-only)
infinity_norm(A) -> f64    max absolute row sum
flatten_matrix(M, r, c)    2D -> 1D row-major (index = row * cols + col)
```

## NR Linear Solve Selection by M

| M | Method | Location |
|---|--------|----------|
| 1 | Direct division | Codegen template |
| 2 | Cramer's rule | Codegen template |
| 3-32 | Gaussian elimination (unrolled) | Codegen template |
| ≤ 32 (full LU path) | Sparse LU + chord refactor | `lu.rs` + codegen-emitted straight-line code |
| >32 | Refused on every route, full LU included | MAX_M = 32 |

## Singularity Thresholds by Context

| Context | Threshold | Fallback |
|---------|-----------|----------|
| DK kernel inversion (`dk.rs:invert_matrix`) | 1e-15 | Error |
| DC OP LU decomposition (`dc_op.rs:lu_decompose`) | 1e-30 | None (try next strategy) |
| Codegen NR Gauss elimination | 1e-15 | Return current best guess |
| Sparse LU (codegen full-LU path) | 1e-15 | NaN reset, restore from DC OP |
| Transformer inversion (`mna/helpers.rs:invert_small_matrix`) | 1e-30 | Error: a singular or non-finite inductance matrix is refused (`DkError::SingularMatrix` / `MnaError::TopologyError`) |
| Codegen sample-rate rebuild (`state.rs.tera:invert_n`) | 1e-30 | Identity matrix + flag |
| SM denominator | 1e-15 | scale = 0 (no correction) |

## Structural Sparsity

> `structural.rs` (patterns), `codegen/ir/matrix_helpers.rs::settle_structural_sparsity`
> (the settlement every IR build runs before `SparseInfo` is filled)

Which entries of the inverse-derived matrices (`S = A⁻¹`, `K = N_v·S·N_i`, and their
backward-Euler twins) the generated code treats as present is decided from the circuit's
**topology**, never from the computed values. Pivoted inversion leaves rounding noise
(1e-35 to 1e-19 against entries near 1e5) at positions that are zero in exact arithmetic,
and which noise entries clear any magnitude cutoff changes with the sample rate; a cutoff
can also drop a genuine small coupling. The pattern has neither problem.

```
A_pat[i][j]  = G[i][j] != 0  or  C[i][j] != 0         (stamped positions; rate-free)
S_pat        = structural inverse of A_pat:
                 1. perfect matching of columns to rows (Kuhn), so the permuted
                    B = P·A has a structural nonzero on every diagonal; the
                    voltage-source branch rows of MNA have zero diagonals
                 2. (B⁻¹)[i][k] may be nonzero iff k is reachable from i in the
                    graph of B  (Gilbert 1994, "Predicting structure in sparse
                    matrix computations", SIAM J. Matrix Anal. Appl. 15(1))
                 3. A⁻¹ = B⁻¹·P maps column k back to column row_of_col[k]
K_pat        = N_v_pat · S_pat · N_i_pat                (boolean products)
               + the 2×2 block of every parasitic-absorbed BJT on the DK route
                 (state.k holds K − R_p there)
```

`S_pat` is an upper bound: an entry it admits may still be zero for particular values,
never the other way round. A structurally singular `A_pat` (no matching exists) is refused.

**Settlement.** Every entry of S, S_BE, K and K_BE outside its pattern is provably zero,
so the computed value there is inversion noise. Each is checked against the rounding
bound below and then set to exactly `0.0`, so shipped constants and runtime seeds carry
no noise. A value above the bound means the pattern missed a stamped position, a melange
bug: the build is refused naming the matrix entry. The bound follows how
`invert_flat_matrix` works (row then column equilibration, then pivoted elimination):

```
Â = diag(dr)·A·diag(dc)       equilibrated system;  Ŝ = Â⁻¹,  S = diag(dc)·Ŝ·diag(dr)
E = c·n·eps·cond∞(Â)·max|Ŝ|   per-entry error of the pivoted inverse of Â, c = 10
                               (STRUCTURAL_NOISE_SAFETY: a stated multiple of the
                               classical n·eps·cond bound)
|S[i][j]|  must be ≤ E·dc[i]·dr[j]
|K[i][j]|  must be ≤ E·(Σ_a |N_v[i][a]|·dc[a])·(Σ_b dr[b]·|N_i[b][j]|)
```

Each build path passes the `A` its inversion actually used (nodal: `a_matrix`,
`a_matrix_be`; DK: the trap or BE `A` the branch formed, the BE-fallback `A_be`), so the
estimate is of that matrix and there is no fallback. The largest `|entry|/bound` met is
`sparsity_noise_ratio` in provenance (≤ 1 on every build that was not refused; corpus
2026-10-05, 282 builds: largest 4.2e-4 on `overdrive_pedal` at 4×, 21 builds above 1e-6,
most below 1e-12).

**Consistency checks (fail loud).** The nodal emitter refuses a build whose recorded
dynamic-parameter setter positions fall outside `A_pat` (`sparsity.a`); the DK emitter
refuses one whose parasitic-BJT block falls outside `K_pat`. Every other pattern user
(`lu::compute_g_aug_pattern`, the equilibration pattern, the KCL residual and the
saturating-inductor residual) reads `sparsity.a` or exact `!= 0.0` on stamp-assembled
matrices.

**Measured on the corpus (112 decks × 1/2/4×, 2026-10-05):** K terms 7819 → 7615: 261
rounding-noise terms dropped (15 builds), 57 genuine sub-1e-20 couplings added (gravity,
gravity-stereo; rate-independent DC-path entries the old cutoff silently dropped; both are
full-LU builds, where K's pattern governs no emitted product, so only their baked
literals changed). No route changed. Renders moved only on the five decks that lost
terms, by ≤ 1.1e-8 relative: the Newton termination band (`|step| ≤ 1e-3·|v| + 1e-6`),
reproduced by changing any present K entry by one ULP. The largest-loss deck
(rexi-mockup, 1096 → 992 terms) benched 4564 → 4395 ns/sample, inside the harness's noise
width.

