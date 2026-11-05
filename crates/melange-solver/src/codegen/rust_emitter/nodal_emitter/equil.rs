//! Structural equilibration pattern and the equilibration emitters.

use crate::codegen::ir::{CircuitIR, LuOp};

/// Structural superset of the nonzeros any matrix passed to `lu_solve`,
/// `lu_factor` or `sparse_lu_factor` can ever hold, at any sample rate and any
/// pot / switch / `.runtime R` / wiper setting.
///
/// Used to skip structural zeros in the row/column equilibration those routines
/// run before factorizing. Skipping is exact: `max` over a set that also
/// contains exact zeros equals `max` over the nonzeros alone (`0.0 > m` is
/// false for any `m >= 0`), and `0.0 * d == 0.0` for the strictly positive
/// scale factors `dr`/`dc`. The transform is therefore byte-identical, *given*
/// that the set really is a superset.
///
/// ## Why it is a superset (this is the load-bearing argument)
///
/// Every matrix these three routines receive is built from `state.a`,
/// `state.a_be`, or a local `a_sub`, each of which `rebuild_matrices` computes
/// elementwise as `g_work[i][j] + alpha * c_work[i][j]`, plus a fixed set of
/// stamps. So:
///
/// 1. `g_work` starts at `G` and `c_work` at `C`; the only writes to either
///    after construction are the `.pot` / `.switch` / `.runtime R` / `.wiper`
///    setters, and every one of those is emitted with a *literal* index that
///    this pattern records as it is emitted (`stamps`). There is no
///    variable-indexed or data-dependent write path into `g_work`/`c_work`.
/// 2. Therefore an entry that is zero in both `G` and `C` and that no setter
///    writes stays exactly zero forever — for every `alpha`, hence for every
///    sample rate. `set_sample_rate` changes only `alpha`, never a position.
/// 3. The remaining writes are the per-sample stamps: the block-diagonal
///    device Jacobian `N_i·J_dev·N_v` and the saturating inductor's
///    augmented-row diagonal. The second is covered by including every
///    diagonal; the first is the device envelope unioned in below.
/// 4. The active-set op-amp resolve only *zeroes* entries and writes a
///    diagonal, so it cannot leave a nonzero outside the pattern.
///
/// Behavioral B-sources stamp `∂f/∂V` at positions derived from expression
/// references rather than from the MNA topology; rather than reproduce that
/// geometry here, circuits carrying them keep the dense equilibration.
#[derive(Debug, Clone)]
pub(super) struct EquilPattern {
    /// Row-major, sorted, deduplicated `(row, col)` pairs.
    entries: Vec<(usize, usize)>,
    n: usize,
}

impl EquilPattern {
    pub(super) fn density(&self) -> f64 {
        self.entries.len() as f64 / (self.n * self.n) as f64
    }

    /// Measured crossover (equilibration kernel, shipping ISA `x86-64`, at the
    /// (N, nnz) shapes taken from the generated corpus):
    ///
    /// | N  | density | sparse vs dense |
    /// |----|---------|-----------------|
    /// | 10 | 30.0 %  | 0.66x  (loss)   |
    /// | 20 | 21.2 %  | 1.02x  (none)   |
    /// | 20 | 16.5 %  | 1.25x           |
    /// | 25 | 12.8 %  | 1.38x           |
    /// | 35 | 10.7 %  | 2.07x           |
    /// | 46 |  7.8 %  | 3.29x           |
    /// | 80 |  3.8 %  | 4.55x           |
    ///
    /// Below N=20, and above ~18 % density, the indexed gather loses to the
    /// contiguous dense sweep (which the compiler vectorizes and which is
    /// entirely L1-resident at these sizes). Gate accordingly.
    fn worth_emitting(&self) -> bool {
        self.n >= 20 && self.density() <= 0.18
    }

    pub(super) fn len(&self) -> usize {
        self.entries.len()
    }
}

/// Build the equilibration pattern. `stamps` holds the literal `(row, col)`
/// positions recorded while the `.pot`/`.switch`/`.runtime R`/`.wiper` setters
/// were emitted — see `EquilPattern`'s safety argument, point 1.
///
/// Returns `None` when the circuit is outside the proven envelope (behavioral
/// B-sources) or when the pattern is not worth emitting.
pub(super) fn build_equil_pattern(
    ir: &CircuitIR,
    stamps: &std::collections::BTreeSet<(usize, usize)>,
) -> Option<EquilPattern> {
    use std::collections::BTreeSet;

    let n = ir.topology.n;
    let m = ir.topology.m;
    if n == 0 {
        return None;
    }
    // Behavioral Jacobian geometry is not reproduced here — keep those
    // circuits on the dense equilibration.
    if !ir.behavioral_sources.is_empty() {
        return None;
    }
    let g = &ir.matrices.g_matrix;
    let c = &ir.matrices.c_matrix;
    if g.len() != n * n || c.len() != n * n {
        return None;
    }

    let mut set: BTreeSet<(usize, usize)> = BTreeSet::new();
    // (1) structural nonzeros of G and C — unioned rather than taking A's
    // nonzeros, so an entry that cancels in `G + alpha*C` at the codegen rate
    // but not at some other host rate is still covered.
    for i in 0..n {
        for j in 0..n {
            if g[i * n + j] != 0.0 || c[i * n + j] != 0.0 {
                set.insert((i, j));
            }
        }
    }
    // (2) every position the emitted setters write.
    set.extend(stamps.iter().copied());
    // (3) every diagonal: covers the saturating inductor augmented-row
    // Jacobian and the active-set pin's `= 1.0`.
    for i in 0..n {
        set.insert((i, i));
    }
    // (4) block-diagonal device Jacobian envelope N_i[:, dev_i] * N_v[dev_j, :].
    if m > 0 && ir.matrices.n_i.len() == n * m && ir.matrices.n_v.len() == m * n {
        let mut ni_rows: Vec<Vec<usize>> = vec![Vec::new(); m];
        for a in 0..n {
            for i in 0..m {
                if ir.matrices.n_i[a * m + i] != 0.0 {
                    ni_rows[i].push(a);
                }
            }
        }
        for slot in &ir.device_slots {
            let s = slot.start_idx;
            for di in 0..slot.dimension {
                for dj in 0..slot.dimension {
                    if s + di >= m || s + dj >= m {
                        continue;
                    }
                    for b in 0..n {
                        if ir.matrices.n_v[(s + dj) * n + b] != 0.0 {
                            for &a in &ni_rows[s + di] {
                                set.insert((a, b));
                            }
                        }
                    }
                }
            }
        }
    }

    let pat = EquilPattern {
        entries: set.into_iter().collect(),
        n,
    };
    if pat.worth_emitting() {
        Some(pat)
    } else {
        None
    }
}

/// Possibly-nonzero positions of the factored matrix at the point `sparse_lu_factor`
/// runs its growth-factor check, in row-major sorted order.
///
/// The check computes `max |a[i][j]|` (and an `is_finite` test) over the whole
/// N×N matrix, purely to reject a static-pivot factorization whose elements blew
/// up. Every position outside this set holds an exact `0.0` at that point, so
/// including it in the `max` cannot change the result and `0.0` is finite — the
/// sparse sweep is therefore byte-identical to the dense one.
///
/// ## Why it is a superset (load-bearing, mirrors `EquilPattern`)
///
/// `sparse_lu_factor` does exactly three things to the matrix: equilibration
/// (scales `EQUIL_PAT` entries in place — no new nonzeros), the static row swaps,
/// then the straight-line elimination. So the only positions that can be nonzero
/// at the growth check are:
///   1. the input nonzeros, which `EquilPattern` is already a proven superset of
///      (see its doc — runtime-verified 0 violations across the sparse corpus),
///      carried through the row swaps that reorder them; plus
///   2. every position the elimination writes (`DivPivot` L factors and `SubMul`
///      updates / fill-in), which are exactly the op LHS targets.
/// Any position touched by neither started at `0.0` and is never written, so it
/// stays `0.0`. The union of (1) and (2) is that superset.
pub(super) fn build_growth_pattern(
    pat: &EquilPattern,
    row_swaps: &[(usize, usize)],
    ops: &[LuOp],
) -> Vec<(usize, usize)> {
    use std::collections::BTreeSet;
    // (1) input pattern, carried through the static row swaps (which the emitted
    //     elimination applies before it runs, so op indices are in swapped space).
    let mut set: BTreeSet<(usize, usize)> = pat.entries.iter().copied().collect();
    for &(r1, r2) in row_swaps {
        set = set
            .into_iter()
            .map(|(i, j)| {
                let ni = if i == r1 {
                    r2
                } else if i == r2 {
                    r1
                } else {
                    i
                };
                (ni, j)
            })
            .collect();
    }
    // (2) every position the straight-line elimination writes.
    for op in ops {
        match op {
            LuOp::DivPivot { row, col } => {
                set.insert((*row, *col));
            }
            LuOp::SubMul { row, j, .. } => {
                set.insert((*row, *j));
            }
        }
    }
    set.into_iter().collect()
}

/// Emit the equilibration prologue shared by `lu_solve`, `lu_factor` and
/// `sparse_lu_factor`: row max / row scale / column max / column scale.
///
/// `dr_decl` is emitted before the sweeps (the three callers declare `dr`/`dc`
/// differently — two take them as `&mut` parameters, one declares them local).
/// When `pat` is `Some`, the sweeps walk the static pattern table instead of
/// all N² entries; the arithmetic and its ordering are otherwise unchanged.
pub(super) fn emit_equilibration(code: &mut String, pat: Option<&EquilPattern>, dr_guarded: bool) {
    // Indented for the dense branch (inside `for i in 0..N {`); the sparse
    // branch nests one level deeper and re-indents.
    let (set_dr, set_dc) = if dr_guarded {
        // lu_solve: `dr`/`dc` are pre-initialised to 1.0 and only overwritten
        // when the max clears the guard.
        (
            "        if row_max > 1e-30 { dr[i] = 1.0 / row_max; }\n",
            "        if col_max > 1e-30 { dc[j] = 1.0 / col_max; }\n",
        )
    } else {
        (
            "        dr[i] = if row_max > 1e-30 { 1.0 / row_max } else { 1.0 };\n",
            "        dc[j] = if col_max > 1e-30 { 1.0 / col_max } else { 1.0 };\n",
        )
    };
    if pat.is_none() {
        code.push_str("    // Row scaling: dr[i] = 1/max_j(|A[i][j]|)\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let mut row_max = 0.0f64;\n");
        code.push_str(
            "        for j in 0..N { let v = a[i][j].abs(); if v > row_max { row_max = v; } }\n",
        );
        code.push_str(set_dr);
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dr[i]; }\n");
        code.push_str("    }\n");
        code.push_str("    // Column scaling: dc[j] = 1/max_i(|A[i][j]|) (after row scaling)\n");
        code.push_str("    for j in 0..N {\n");
        code.push_str("        let mut col_max = 0.0f64;\n");
        code.push_str(
            "        for i in 0..N { let v = a[i][j].abs(); if v > col_max { col_max = v; } }\n",
        );
        code.push_str(set_dc);
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dc[j]; }\n");
        code.push_str("    }\n\n");
        return;
    }
    // Sparsity-aware form. Structural zeros contribute nothing to either max
    // (0.0 never exceeds a non-negative running max) and are unchanged by the
    // scaling (0.0 * d == 0.0), so the result is byte-identical.
    code.push_str(
        "    // Equilibration over EQUIL_PAT — every entry outside it is a\n\
         \x20   // structural zero, which cannot raise a max and is unchanged by\n\
         \x20   // scaling, so this is byte-identical to sweeping all N*N.\n",
    );
    code.push_str("    // Row scaling: dr[i] = 1/max_j(|A[i][j]|)\n");
    code.push_str("    {\n");
    code.push_str("        let mut row_max_all = [0.0f64; N];\n");
    code.push_str("        for &(i, j) in EQUIL_PAT.iter() {\n");
    code.push_str("            let v = a[i as usize][j as usize].abs();\n");
    code.push_str("            if v > row_max_all[i as usize] { row_max_all[i as usize] = v; }\n");
    code.push_str("        }\n");
    code.push_str("        for i in 0..N {\n");
    code.push_str("            let row_max = row_max_all[i];\n");
    code.push_str(&format!("    {}", set_dr));
    code.push_str("        }\n");
    code.push_str("    }\n");
    code.push_str(
        "    for &(i, j) in EQUIL_PAT.iter() { a[i as usize][j as usize] *= dr[i as usize]; }\n",
    );
    code.push_str("    // Column scaling: dc[j] = 1/max_i(|A[i][j]|) (after row scaling)\n");
    code.push_str("    {\n");
    code.push_str("        let mut col_max_all = [0.0f64; N];\n");
    code.push_str("        for &(i, j) in EQUIL_PAT.iter() {\n");
    code.push_str("            let v = a[i as usize][j as usize].abs();\n");
    code.push_str("            if v > col_max_all[j as usize] { col_max_all[j as usize] = v; }\n");
    code.push_str("        }\n");
    code.push_str("        for j in 0..N {\n");
    code.push_str("            let col_max = col_max_all[j];\n");
    code.push_str(&format!("    {}", set_dc));
    code.push_str("        }\n");
    code.push_str("    }\n");
    code.push_str(
        "    for &(i, j) in EQUIL_PAT.iter() { a[i as usize][j as usize] *= dc[j as usize]; }\n\n",
    );
}

/// Emit the `EQUIL_PAT` table consumed by the sparse equilibration.
pub(super) fn emit_equil_pattern_table(pat: &EquilPattern) -> String {
    let mut code = String::new();
    code.push_str(&format!(
        "/// Structural nonzero pattern of the equilibrated matrices\n\
         /// ({} of {} entries, {:.1}% density). Superset over every sample rate\n\
         /// and every pot/switch/runtime-R setting — see `EquilPattern` in the\n\
         /// emitter for why. Entries outside it are exactly 0.0.\n\
         const EQUIL_PAT: [(u16, u16); {}] = [",
        pat.len(),
        pat.n * pat.n,
        100.0 * pat.density(),
        pat.len()
    ));
    for (k, (i, j)) in pat.entries.iter().enumerate() {
        if k % 8 == 0 {
            code.push_str("\n    ");
        }
        code.push_str(&format!("({i},{j}),"));
    }
    code.push_str("\n];\n\n");
    code
}

/// Every position of the input matrix that `sparse_lu_factor` and
/// `sparse_lu_back_solve` read or write, in input (pre-row-swap) coordinates,
/// row-major sorted.
///
/// The Newton loop builds the matrix it hands to `sparse_lu_factor` from
/// `state.a` (or `state.a_be`) plus the device stamps. Copying only these
/// positions, instead of all N², feeds the sparse factor and back-solve the
/// same values at every position they touch, so their results are
/// bit-identical. Positions outside the set are never read on the sparse path;
/// they may hold anything (a previous dense factorization, a row the static
/// swaps moved). The dense fallback does not use this set: it rebuilds the
/// full matrix before `lu_factor` reads all N².
///
/// The set is the union of:
///   1. `pat` itself (the equilibration sweeps read and scale exactly these
///      positions, before the row swaps);
///   2. every position the elimination reads or writes (pivots, L factors,
///      U-row operands, `SubMul` targets), the growth check reads (`growth`)
///      and the back-solve reads (`l_nnz`, `u_nnz`, the pivots), all of which
///      are in post-swap coordinates and are mapped back through the swaps.
///
/// Positions in (2) that are not in (1) are structural zeros of the input
/// (`EquilPattern` is a superset of every input nonzero), so initialising them
/// from `state.a` writes the same exact `0.0` the full copy did.
pub(super) fn sparse_lu_touch_pattern(
    pat: &EquilPattern,
    lu: &crate::codegen::ir::LuSparsity,
    growth: &[(usize, usize)],
) -> Vec<(usize, usize)> {
    use std::collections::BTreeSet;
    let n = pat.n;
    // row_src[i] = input row that sits at row i after the static swaps, which
    // `sparse_lu_factor` applies in order with `a.swap(r1, r2)`.
    let mut row_src: Vec<usize> = (0..n).collect();
    for &(r1, r2) in &lu.row_swaps {
        row_src.swap(r1, r2);
    }
    let mut post: BTreeSet<(usize, usize)> = growth.iter().copied().collect();
    for op in &lu.ops {
        match *op {
            LuOp::DivPivot { row, col } => {
                post.insert((row, col));
                post.insert((col, col));
            }
            LuOp::SubMul { row, col, j } => {
                post.insert((row, j));
                post.insert((row, col));
                post.insert((col, j));
            }
        }
    }
    post.extend(lu.l_nnz.iter().copied());
    post.extend(lu.u_nnz.iter().copied());
    for &p in &lu.elim_order {
        post.insert((p, p));
    }
    let mut set: BTreeSet<(usize, usize)> = pat.entries.iter().copied().collect();
    set.extend(post.into_iter().map(|(i, j)| (row_src[i], j)));
    set.into_iter().collect()
}

/// Straight-line form of the sparse equilibration in `sparse_lu_factor`:
/// the same row max, row scale, column max and column scale over `EQUIL_PAT`,
/// with constant indices instead of a loop over the index table.
///
/// Bit-identical to the table loop of [`emit_equilibration`]:
/// - each scale factor is `1/max` of the same set of `|a|` values. A running
///   max by `if v > m { m = v; }` from `m = 0` is independent of the order the
///   values arrive in (a NaN never compares greater and is skipped either way;
///   `abs` never yields `-0`), so taking a row's or a column's entries
///   together instead of interleaved with other rows gives the same `m`;
/// - every scaling is one multiply of one entry by one factor, and each
///   entry's two multiplies keep their order (row factor, then column
///   factor), because every column max is taken after every row is scaled.
///
/// The table loop kept the running maxima in a stack array indexed through
/// the table, a store-to-load chain on every entry of a row, plus two bounds
/// checks per access; here each row's and each column's max is a register.
pub(super) fn emit_equilibration_unrolled(
    code: &mut String,
    pat: &EquilPattern,
    at: &dyn Fn(usize, usize) -> String,
) {
    let n = pat.n;
    let mut cols_of_row: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut rows_of_col: Vec<Vec<usize>> = vec![Vec::new(); n];
    for &(i, j) in &pat.entries {
        cols_of_row[i].push(j);
        rows_of_col[j].push(i);
    }
    code.push_str(
        "    // Equilibration over EQUIL_PAT (straight-line) — every entry outside\n\
         \x20   // it is a structural zero, which cannot raise a max and is unchanged\n\
         \x20   // by scaling, so this is byte-identical to sweeping all N*N.\n\
         \x20   // Row scaling: dr[i] = 1/max_j(|A[i][j]|)\n",
    );
    for i in 0..n {
        code.push_str("    {\n        let mut m = 0.0f64;\n");
        for &j in &cols_of_row[i] {
            code.push_str(&format!(
                "        {{ let v = {}.abs(); if v > m {{ m = v; }} }}\n",
                at(i, j)
            ));
        }
        code.push_str(&format!(
            "        dr[{i}] = if m > 1e-30 {{ 1.0 / m }} else {{ 1.0 }};\n"
        ));
        for &j in &cols_of_row[i] {
            code.push_str(&format!("        {} *= dr[{i}];\n", at(i, j)));
        }
        code.push_str("    }\n");
    }
    code.push_str("    // Column scaling: dc[j] = 1/max_i(|A[i][j]|) (after row scaling)\n");
    for j in 0..n {
        code.push_str("    {\n        let mut m = 0.0f64;\n");
        for &i in &rows_of_col[j] {
            code.push_str(&format!(
                "        {{ let v = {}.abs(); if v > m {{ m = v; }} }}\n",
                at(i, j)
            ));
        }
        code.push_str(&format!(
            "        dc[{j}] = if m > 1e-30 {{ 1.0 / m }} else {{ 1.0 }};\n"
        ));
        for &i in &rows_of_col[j] {
            code.push_str(&format!("        {} *= dc[{j}];\n", at(i, j)));
        }
        code.push_str("    }\n");
    }
    code.push('\n');
}

/// Straight-line growth-factor sweep over the factored pattern `growth` (see
/// [`build_growth_pattern`]), into four running maxima combined at the end.
///
/// Bit-identical to the single running max over the table: the max of a set
/// by `if v > m { m = v; }` from 0 skips NaN and does not depend on order or
/// grouping, and none of the partial maxima can be NaN. Four independent
/// chains instead of one shorten the dependency chain of the compares.
pub(super) fn emit_growth_check_unrolled(
    code: &mut String,
    growth: &[(usize, usize)],
    at: &dyn Fn(usize, usize) -> String,
) {
    // Every lane must be written, or the generated code carries an unused
    // `mut`. The pattern holds every diagonal and is only built for N >= 20.
    assert!(
        growth.len() >= 4,
        "growth pattern shorter than its four lanes"
    );
    code.push_str("    let (mut g0, mut g1, mut g2, mut g3) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);\n");
    for (k, &(i, j)) in growth.iter().enumerate() {
        let lane = k % 4;
        code.push_str(&format!(
            "    {{ let v = {}.abs(); if v > g{lane} {{ g{lane} = v; }} }}\n",
            at(i, j)
        ));
    }
    code.push_str("    let mut growth = g0;\n");
    code.push_str("    if g1 > growth { growth = g1; }\n");
    code.push_str("    if g2 > growth { growth = g2; }\n");
    code.push_str("    if g3 > growth { growth = g3; }\n");
}

/// Packed storage of the sparse chord factor: one slot per position of
/// [`sparse_lu_touch_pattern`], in that (row-major, input-coordinate) order.
///
/// The sparse factor and back-solve read and write only those positions, so
/// keeping them in a dense `[f64; LU_SLOTS]` instead of an `N x N` array runs
/// the same operations, in the same order, on the same values: the results are
/// bit-identical. What changes is the memory footprint the hot loop walks
/// (on an N = 84 deck, 488 slots = 3.9 KB against 56 KB for the full array,
/// which does not fit in L1). The static row swaps become an index remap
/// fixed at codegen time: after the swaps, position `(i, j)` holds input row
/// `row_src[i]`.
pub(super) struct LuPack {
    /// Slot `k` holds input position `slots[k]`.
    pub slots: Vec<(usize, usize)>,
    index: std::collections::BTreeMap<(usize, usize), usize>,
    row_src: Vec<usize>,
}

impl LuPack {
    /// The packing a build takes, or `None` for the N x N layout: packed
    /// whenever the structural pattern exists and the build has chord state
    /// to pack (`sites::has_chord_state`: an M = 0 linear build solves a
    /// direct dense LU per sample and stores no factor, so there is nothing
    /// to pack and no `chord_lu` to put the packed fields beside), except on
    /// builds that stamp saturating inductors. There the packed layout measured slower (one
    /// corpus deck, N = 73 with 16 saturating inductors, +3 % per sample with
    /// 62 % fewer L1 misses and twice the FP-scheduler stalls; the cause is
    /// not identified), so those builds keep the N x N array. The exclusion
    /// is a measurement, not a structural rule: re-measure it before
    /// carrying it into any change to the flux rows or the pack.
    pub(super) fn for_build(ir: &CircuitIR, pat: Option<&EquilPattern>) -> Option<Self> {
        match (&ir.sparsity.lu, pat) {
            (Some(lu), Some(p))
                if ir.saturating_inductors.is_empty() && super::sites::has_chord_state(ir) =>
            {
                Some(Self::new(p, lu))
            }
            _ => None,
        }
    }

    fn new(pat: &EquilPattern, lu: &crate::codegen::ir::LuSparsity) -> Self {
        let growth = build_growth_pattern(pat, &lu.row_swaps, &lu.ops);
        let slots = sparse_lu_touch_pattern(pat, lu, &growth);
        let index = slots.iter().enumerate().map(|(k, &p)| (p, k)).collect();
        let mut row_src: Vec<usize> = (0..pat.n).collect();
        for &(r1, r2) in &lu.row_swaps {
            row_src.swap(r1, r2);
        }
        Self {
            slots,
            index,
            row_src,
        }
    }

    /// Slot of input (pre-swap) position `(i, j)`. Every position the sparse
    /// path reads or writes is in the touch pattern; a miss is a codegen bug
    /// (an entry the factor would silently never see), so it fails loudly.
    pub(super) fn pre(&self, i: usize, j: usize) -> usize {
        match self.index.get(&(i, j)) {
            Some(&k) => k,
            None => panic!(
                "packed sparse LU: position ({i}, {j}) is outside the touch pattern; \
                 this is a melange bug, please report it with the deck"
            ),
        }
    }

    /// Slot of post-swap position `(i, j)` (inside the elimination, the
    /// growth sweep and the back-solve).
    pub(super) fn post(&self, i: usize, j: usize) -> usize {
        self.pre(self.row_src[i], j)
    }

    /// Rewrite every `{name}[i][j]` in `text` (an emitted stamp block, input
    /// coordinates) to `{name}_pk[slot]`. An index is a literal or a named
    /// constant that `consts` resolves (the saturating-inductor branch rows).
    /// Anything else fails loudly: a stamp left in the N x N array would be
    /// invisible to the packed factor, a silently wrong Jacobian.
    pub(super) fn remap_stamps(
        &self,
        text: &str,
        name: &str,
        consts: &dyn Fn(&str) -> Option<usize>,
    ) -> String {
        let needle = format!("{name}[");
        let index = |tok: &str| -> usize {
            tok.parse::<usize>()
                .ok()
                .or_else(|| consts(tok))
                .unwrap_or_else(|| {
                    panic!(
                        "packed sparse LU: stamp index `{tok}` into `{name}` is neither a literal \
                     nor a known constant; this is a melange bug, please report it with the deck"
                    )
                })
        };
        let mut out = String::with_capacity(text.len());
        let mut rest = text;
        while let Some(at) = rest.find(&needle) {
            // A longer identifier ending in `name` is not a match.
            let boundary = at == 0
                || !rest[..at]
                    .chars()
                    .next_back()
                    .is_some_and(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.');
            let after = &rest[at + needle.len()..];
            if !boundary {
                out.push_str(&rest[..at + needle.len()]);
                rest = after;
                continue;
            }
            let bad = || -> ! {
                panic!(
                    "packed sparse LU: unparseable stamp `{name}[{}` ; this is a melange bug",
                    &after[..after.len().min(40)]
                )
            };
            let i_end = after.find(']').unwrap_or_else(|| bad());
            let tail = after[i_end + 1..]
                .strip_prefix('[')
                .unwrap_or_else(|| bad());
            let j_end = tail.find(']').unwrap_or_else(|| bad());
            let (i, j) = (index(&after[..i_end]), index(&tail[..j_end]));
            out.push_str(&rest[..at]);
            out.push_str(&format!("{name}_pk[{}]", self.pre(i, j)));
            rest = &tail[j_end + 1..];
        }
        out.push_str(rest);
        out
    }
}

#[cfg(test)]
mod lu_pack_tests {
    use super::LuPack;
    use std::collections::BTreeMap;

    fn pack() -> LuPack {
        // Rows 0 and 1 swapped: post-swap (0, j) holds input row 1.
        let slots = vec![(0, 0), (0, 2), (1, 1), (2, 2)];
        let index: BTreeMap<_, _> = slots.iter().enumerate().map(|(k, &p)| (p, k)).collect();
        LuPack {
            slots,
            index,
            row_src: vec![1, 0, 2],
        }
    }

    #[test]
    fn remaps_literal_and_constant_indices() {
        let pk = pack();
        let consts = |t: &str| (t == "ROW_TWO").then_some(2);
        let text = "state.chord_lu[0][2] -= x;\nstate.chord_lu[ROW_TWO][ROW_TWO] += y;\nxstate.chord_lu[0][0] = z;\n";
        let out = pk.remap_stamps(text, "state.chord_lu", &consts);
        assert_eq!(
            out,
            "state.chord_lu_pk[1] -= x;\nstate.chord_lu_pk[3] += y;\nxstate.chord_lu[0][0] = z;\n"
        );
        assert_eq!(pk.post(0, 1), 2, "post-swap row 0 is input row 1");
    }

    #[test]
    #[should_panic(expected = "neither a literal nor a known constant")]
    fn unknown_index_fails_loudly() {
        pack().remap_stamps("state.chord_lu[K][0] -= x;", "state.chord_lu", &|_| None);
    }

    #[test]
    #[should_panic(expected = "outside the touch pattern")]
    fn position_outside_the_pattern_fails_loudly() {
        pack().remap_stamps("state.chord_lu[2][0] -= x;", "state.chord_lu", &|_| None);
    }
}
