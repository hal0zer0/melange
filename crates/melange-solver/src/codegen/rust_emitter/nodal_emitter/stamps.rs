//! Device-Jacobian, body-effect and companion stamps shared by the nodal Newton sites.

use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::helpers::body_effect_mosfets;

/// Resolve behavioral param names to generated-Rust strings: `.param` constants
/// to a baked literal, plugin scalars to their `state.<field>`.
/// Transpose of the N_i sparsity pattern: `out[i]` lists the node rows `a`
/// where `N_i[a][i]` is nonzero, for each device dim `i`. Every nodal
/// device-stamp site needs this same transpose; built once here instead of
/// inline at each call site.
fn ni_nonzeros_by_dev(ir: &CircuitIR, m: usize) -> Vec<Vec<usize>> {
    let mut out = vec![Vec::new(); m];
    for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
        for &i in cols {
            out[i].push(a);
        }
    }
    out
}

// Saturating (iron-core) inductor — in-NR-loop companion stamps. On a shared
// core this is the T-model's single magnetizing branch.
//
// The augmented branch row `k = aug_row` holds branch current `i_k`. A linear
// inductor bakes `alpha·L0` into the base matrix at `[k][k]` and `alpha·L0·i_prev`
// into the history (`A_neg·v_prev`). A saturating inductor replaces the linear
// flux `L0·i` with `Φ(i) = L_mag·Isat·tanh(i/Isat) + L_air·i`; the Jacobian uses
// the differential `L_diff(i) = L_mag/cosh²(i/Isat) + L_air`. See
// `SATURATING_TRANSFORMERS.md` §3.2 for the stamps and sign convention (verified
// against `mna.rs::build_augmented_matrices`).
//
// `alpha` is the site-local integrator scalar expression (`2·rate·OS` trap,
// `1·rate·OS` BE, `alpha_sub` sub-step) — this is what makes BE composition free.

/// Stamp each body-effect MOSFET's `gmb` into the node-space Jacobian `mat`:
/// `∂Id/∂V(source) = −gmb`, `∂Id/∂V(bulk) = +gmb` (see
/// `dc_op::mosfet_gmb_terms` for the derivation), injected through the Id
/// column of N_I with the same `MAT −= N_I·∂i/∂v` convention as
/// [`emit_nodal_jacobian_stamp`]. `lu::compute_g_aug_pattern` carries the same
/// positions.
pub(super) fn emit_body_gmb_stamp(
    code: &mut String,
    ir: &CircuitIR,
    mat: &str,
    gmb: &str,
    indent: &str,
) {
    let (n, m) = (ir.topology.n, ir.topology.m);
    for (k, _dev_num, slot, mp) in body_effect_mosfets(ir) {
        let row = slot.start_idx;
        for a in 0..n {
            let ni = ir.matrices.n_i[a * m + row];
            if ni == 0.0 {
                continue;
            }
            if mp.source_node > 0 {
                code.push_str(&format!(
                    "{indent}{mat}[{a}][{}] += N_I[{a}][{row}] * {gmb}[{k}];\n",
                    mp.source_node - 1
                ));
            }
            if mp.bulk_node > 0 {
                code.push_str(&format!(
                    "{indent}{mat}[{a}][{}] -= N_I[{a}][{row}] * {gmb}[{k}];\n",
                    mp.bulk_node - 1
                ));
            }
        }
    }
}

/// Companion term matching [`emit_body_gmb_stamp`]: the linearisation of Id's
/// source/bulk dependence at the iterate `it`, `+gmb·(V(source) − V(bulk))`
/// on the Id column, so the Newton fixed point is unchanged.
pub(super) fn emit_body_gmb_companion(
    code: &mut String,
    ir: &CircuitIR,
    rhs: &str,
    gmb: &str,
    it: &str,
    indent: &str,
) {
    let (n, m) = (ir.topology.n, ir.topology.m);
    for (k, _dev_num, slot, mp) in body_effect_mosfets(ir) {
        let row = slot.start_idx;
        let node = |idx: usize| {
            if idx > 0 {
                format!("{it}[{}]", idx - 1)
            } else {
                "0.0".to_string()
            }
        };
        let (vs, vb) = (node(mp.source_node), node(mp.bulk_node));
        for a in 0..n {
            if ir.matrices.n_i[a * m + row] == 0.0 {
                continue;
            }
            code.push_str(&format!(
                "{indent}{rhs}[{a}] += N_I[{a}][{row}] * {gmb}[{k}] * ({vs} - {vb});\n"
            ));
        }
    }
}

/// Emit the block-diagonal NR Jacobian stamp `MAT[a][b] -= N_I[a][i]·j_dev[i,j]
/// ·N_V[j][b]` over every device slot's nonzero N_i/N_v entries. Shared by the
/// trap (chord_lu), sub-step (g_s) and BE (g_aug) paths — they differ only in
/// the target matrix variable and indentation.
pub(super) fn emit_nodal_jacobian_stamp(
    code: &mut String,
    ir: &CircuitIR,
    m: usize,
    mat: &str,
    indent: &str,
) {
    let ni_nz_by_dev = ni_nonzeros_by_dev(ir, m);
    for slot in &ir.device_slots {
        let s = slot.start_idx;
        let dim = slot.dimension;
        for di in 0..dim {
            let i = s + di;
            let ni_nodes = &ni_nz_by_dev[i];
            for dj in 0..dim {
                let j = s + dj;
                let nv_nodes = &ir.sparsity.n_v.nz_by_row[j];
                if ni_nodes.is_empty() || nv_nodes.is_empty() {
                    continue;
                }
                let jd_ij = i * m + j;
                for &a in ni_nodes {
                    for &b in nv_nodes {
                        code.push_str(&format!(
                            "{indent}{mat}[{}][{}] -= N_I[{}][{}] * j_dev[{}] * N_V[{}][{}];\n",
                            a, b, a, i, jd_ij, j, b
                        ));
                    }
                }
            }
        }
    }
}

/// Emit the companion-RHS stamp `RHS[a] += N_I[a][i]·(i_nl[i] − Σ_j
/// jdev[i,j]·v_nl[j])` over every device slot. Shared by the trap, sub-step and
/// BE paths — they differ in the RHS variable, the j_dev source (chord_j_dev on
/// the trap path to match the persisted LU factorization, j_dev otherwise) and
/// indentation (inner stamps are indented two spaces past `indent`).
pub(super) fn emit_nodal_companion_rhs(
    code: &mut String,
    ir: &CircuitIR,
    m: usize,
    rhs: &str,
    jdev: &str,
    indent: &str,
) {
    let inner = format!("{indent}  ");
    let ni_nz_by_dev = ni_nonzeros_by_dev(ir, m);
    for slot in &ir.device_slots {
        let s = slot.start_idx;
        let dim = slot.dimension;
        for di in 0..dim {
            let i = s + di;
            let jdv_terms: Vec<String> = (0..dim)
                .map(|dj| {
                    let j = s + dj;
                    let jd_ij = i * m + j;
                    format!("{jdev}[{}] * v_nl[{}]", jd_ij, j)
                })
                .collect();
            code.push_str(&format!(
                "{indent}{{ let i_comp = i_nl[{}] - ({});\n",
                i,
                jdv_terms.join(" + ")
            ));
            for &a in &ni_nz_by_dev[i] {
                code.push_str(&format!(
                    "{inner}{rhs}[{}] += N_I[{}][{}] * i_comp;\n",
                    a, a, i
                ));
            }
            code.push_str(&format!("{indent}}}\n"));
        }
    }
}

/// Emit the per-iteration Hard-mode op-amp output rail clamp
/// (`if T[o] > hi { T[o] = hi }` / `< lo`) over every clampable op-amp. Shared
/// by the trap, sub-step and BE NR loops, which differ only in the target array,
/// indent, an optional leading comment and whether a trailing blank line
/// follows. Emits nothing when no op-amp has a finite rail.
pub(super) fn emit_hard_rail_clamp(
    code: &mut String,
    ir: &CircuitIR,
    target: &str,
    indent: &str,
    comment: Option<&str>,
    trailing_nl: bool,
) {
    let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
        .opamps
        .iter()
        .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
        .collect();
    if clampable.is_empty() {
        return;
    }
    if let Some(c) = comment {
        code.push_str(c);
    }
    for oa in &clampable {
        let o = oa.n_out_idx;
        if oa.vclamp_hi.is_finite() {
            code.push_str(&format!(
                "{indent}if {target}[{o}] > {hi:.17e} {{ {target}[{o}] = {hi:.17e}; }}\n",
                hi = oa.vclamp_hi
            ));
        }
        if oa.vclamp_lo.is_finite() {
            code.push_str(&format!(
                "{indent}if {target}[{o}] < {lo:.17e} {{ {target}[{o}] = {lo:.17e}; }}\n",
                lo = oa.vclamp_lo
            ));
        }
    }
    if trailing_nl {
        code.push('\n');
    }
}
