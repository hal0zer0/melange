//! Behavioral (B-source) expression emission for the nodal NR loop.

use crate::codegen::ir::{BehavioralSourceIR, CircuitIR};
use crate::codegen::rust_emitter::helpers::fmt_f64;
use crate::expr::{strip_outer_parens, ExprResolver, Var};

/// Resolves behavioral-expression leaves to generated-Rust strings against the
/// nodal NR loop's `v[]` voltage vector. Node indices in
/// `referenced_node_indices` are 1-based (0 = ground); matrix/`v[]` index is
/// `idx - 1`. Only the algebraic `I={}` surface (nodes) is wired today; the
/// other resolvers are placeholders guarded off in `behavioral_emitter_supported`.
struct NodalBsrcResolver<'a> {
    node_idx: &'a std::collections::BTreeMap<String, usize>,
    params: &'a std::collections::BTreeMap<String, String>,
}

impl ExprResolver for NodalBsrcResolver<'_> {
    fn node_v(&self, name: &str) -> String {
        match self.node_idx.get(name).copied().unwrap_or(0) {
            0 => "0.0".to_string(),
            i => format!("v[{}]", i - 1),
        }
    }
    fn branch_i(&self, _name: &str) -> String {
        "0.0".to_string()
    }
    fn param(&self, name: &str) -> String {
        // Pre-validated in generate_nodal; default to 0.0 if somehow unresolved.
        self.params
            .get(name)
            .cloned()
            .unwrap_or_else(|| "0.0".to_string())
    }
    fn time(&self) -> String {
        "state.sim_time".to_string()
    }
    fn inv_dt(&self) -> String {
        "bsrc_inv_dt".to_string()
    }
    fn half_dt(&self) -> String {
        "bsrc_half_dt".to_string()
    }
    fn x_prev(&self, slot: usize) -> String {
        format!("state.bsrc_x_prev[{slot}]")
    }
    fn integ_prev(&self, slot: usize) -> String {
        format!("state.bsrc_int_prev[{slot}]")
    }
}

fn bsrc_param_map(ir: &CircuitIR) -> std::collections::BTreeMap<String, String> {
    let mut m = std::collections::BTreeMap::new();
    for (name, val) in &ir.behavioral_param_consts {
        m.insert(name.clone(), fmt_param_const(*val));
    }
    for r in &ir.behavioral_scalar_runtimes {
        m.insert(r.name.clone(), format!("state.{}", r.field_name));
    }
    m
}

/// Format an `f64` param constant as a Rust literal.
///
/// Non-finite values route through `fmt_f64` — `{:e}` would render them as
/// the invalid Rust tokens `inf` / `-inf` / `NaN` instead of
/// `f64::INFINITY` / `f64::NEG_INFINITY` / `f64::NAN`.
fn fmt_param_const(v: f64) -> String {
    if !v.is_finite() {
        fmt_f64(v)
    } else if v == v.trunc() && v.abs() < 1e15 {
        format!("{:.1}", v)
    } else {
        format!("{:e}", v)
    }
}

/// Non-ground referenced nodes as `(name, matrix_index)` pairs, deterministically
/// ordered (BTreeMap iteration). `matrix_index = node_map_index - 1`.
fn bsrc_ref_cols(b: &BehavioralSourceIR) -> Vec<(&String, usize)> {
    b.referenced_node_indices
        .iter()
        .filter(|(_, &idx)| idx > 0)
        .map(|(name, &idx)| (name, idx - 1))
        .collect()
}

/// Emit `let bsrc_<si>_f = <expr>;` and one `let bsrc_<si>_g_<col> = <∂expr/∂node>;`
/// per referenced node, evaluated at the current iterate `v`. Placed before the
/// refactor/companion blocks so both can use them.
pub(super) fn emit_behavioral_evals(code: &mut String, ir: &CircuitIR, indent: &str) {
    let params = bsrc_param_map(ir);
    for (si, b) in ir.behavioral_sources.iter().enumerate() {
        let res = NodalBsrcResolver {
            node_idx: &b.referenced_node_indices,
            params: &params,
        };
        let f = b.expr.simplify();
        code.push_str(&format!(
            "{indent}let bsrc_{si}_f = {};\n",
            strip_outer_parens(&f.to_rust(&res))
        ));
        for (name, col) in bsrc_ref_cols(b) {
            // Lagged Jacobian: ddt is treated as constant (∂ddt/∂v = 0) so the
            // discriminator's inv_dt-scaled partials don't ill-condition the NR.
            // The value (bsrc_f) still uses the full ddt, so the fixed point is
            // exact. See Expr::diff_jacobian.
            let g = b.expr.diff_jacobian(&Var::Node(name.clone())).simplify();
            code.push_str(&format!(
                "{indent}let bsrc_{si}_g_{col} = {};\n",
                strip_outer_parens(&g.to_rust(&res))
            ));
        }
    }
}

/// Stamp the behavioral Jacobian into `target` (the site's chord LU storage,
/// = G_aug).
///
/// - `I={}` (current source): `∂f/∂V(k)` adds at row n+, subtracts at row n-.
/// - `V={}` (voltage constraint `V(n+)-V(n-)=f`): `−∂f/∂V(k)` into the
///   augmented constraint row `r` (the `±1` on n+/n- is already in base G).
pub(super) fn emit_behavioral_jacobian(
    code: &mut String,
    ir: &CircuitIR,
    target: &str,
    indent: &str,
) {
    for (si, b) in ir.behavioral_sources.iter().enumerate() {
        if b.is_voltage {
            let r = b.aug_row.expect("V={} source must have an aug_row");
            for (_name, col) in bsrc_ref_cols(b) {
                code.push_str(&format!(
                    "{indent}{target}[{r}][{col}] -= bsrc_{si}_g_{col};\n"
                ));
            }
        } else {
            let (np, nm) = (b.n_plus_idx, b.n_minus_idx);
            for (_name, col) in bsrc_ref_cols(b) {
                if np > 0 {
                    code.push_str(&format!(
                        "{indent}{target}[{}][{col}] += bsrc_{si}_g_{col};\n",
                        np - 1
                    ));
                }
                if nm > 0 {
                    code.push_str(&format!(
                        "{indent}{target}[{}][{col}] -= bsrc_{si}_g_{col};\n",
                        nm - 1
                    ));
                }
            }
        }
    }
}

/// Stamp the behavioral companion `comp = f(v) - Σ_k (∂f/∂V(k))·v[k]` into
/// `rhs_work`.
///
/// - `I={}`: subtract at n+, add at n- (current injection).
/// - `V={}`: add into the augmented constraint row `r`.
pub(super) fn emit_behavioral_rhs(code: &mut String, ir: &CircuitIR, indent: &str) {
    for (si, b) in ir.behavioral_sources.iter().enumerate() {
        let mut comp = format!("bsrc_{si}_f");
        for (_name, col) in bsrc_ref_cols(b) {
            comp.push_str(&format!(" - bsrc_{si}_g_{col} * v[{col}]"));
        }
        code.push_str(&format!("{indent}let bsrc_{si}_comp = {comp};\n"));
        if b.is_voltage {
            let r = b.aug_row.expect("V={} source must have an aug_row");
            code.push_str(&format!("{indent}rhs_work[{r}] += bsrc_{si}_comp;\n"));
        } else {
            let (np, nm) = (b.n_plus_idx, b.n_minus_idx);
            if np > 0 {
                code.push_str(&format!(
                    "{indent}rhs_work[{}] -= bsrc_{si}_comp;\n",
                    np - 1
                ));
            }
            if nm > 0 {
                code.push_str(&format!(
                    "{indent}rhs_work[{}] += bsrc_{si}_comp;\n",
                    nm - 1
                ));
            }
        }
    }
}

/// Emit the post-convergence `ddt`/`idt` companion-state update at the
/// converged `v`: store each inner-expression value into `bsrc_x_prev` (after
/// advancing `bsrc_int_prev` for `idt`, which needs the old value), then advance
/// `sim_time` by one `dt`.
pub(super) fn emit_behavioral_time_update(code: &mut String, ir: &CircuitIR, indent: &str) {
    let params = bsrc_param_map(ir);
    for b in &ir.behavioral_sources {
        let res = NodalBsrcResolver {
            node_idx: &b.referenced_node_indices,
            params: &params,
        };
        for (slot, is_idt, inner) in b.expr.collect_time_ops() {
            let x = inner.simplify().to_rust(&res);
            let x = strip_outer_parens(&x);
            code.push_str(&format!("{indent}let bsrc_slot_{slot}_x = {x};\n"));
            if is_idt {
                code.push_str(&format!(
                    "{indent}state.bsrc_int_prev[{slot}] += bsrc_half_dt * (bsrc_slot_{slot}_x + state.bsrc_x_prev[{slot}]);\n"
                ));
            }
            code.push_str(&format!(
                "{indent}state.bsrc_x_prev[{slot}] = bsrc_slot_{slot}_x;\n"
            ));
        }
    }
    code.push_str(&format!("{indent}state.sim_time += 2.0 * bsrc_half_dt;\n"));
}

/// Node indices (0-based) driven by behavioral `V={}` sources. These are set
/// algebraically (`V = f`, solved exactly each NR iteration with the lagged
/// Jacobian), so the global node-damping must NOT throttle on their step — a
/// legitimate large value (e.g. the `ddt` discriminator's startup spike) would
/// otherwise crush every node's step and stall convergence. Returns a Rust
/// array literal of the excluded indices (empty `[]` when none).
pub(super) fn behavioral_damp_skip_literal(ir: &CircuitIR) -> String {
    let mut nodes: Vec<usize> = Vec::new();
    for b in &ir.behavioral_sources {
        if b.is_voltage && b.n_plus_idx > 0 {
            nodes.push(b.n_plus_idx - 1);
        }
    }
    nodes.sort();
    nodes.dedup();
    let items: Vec<String> = nodes.iter().map(|n| format!("{n}usize")).collect();
    format!("[{}]", items.join(", "))
}

/// Node indices (0-based) touched by behavioral sources — terminals + referenced
/// nodes — so the NR convergence check includes them (behavioral circuits are
/// often `M=0`, where the device-node set would otherwise be empty).
pub(super) fn behavioral_convergence_nodes(ir: &CircuitIR) -> Vec<usize> {
    let mut nodes = Vec::new();
    for b in &ir.behavioral_sources {
        if b.n_plus_idx > 0 {
            nodes.push(b.n_plus_idx - 1);
        }
        if b.n_minus_idx > 0 {
            nodes.push(b.n_minus_idx - 1);
        }
        if let Some(r) = b.aug_row {
            nodes.push(r);
        }
        for (_name, col) in bsrc_ref_cols(b) {
            nodes.push(col);
        }
    }
    nodes.sort();
    nodes.dedup();
    nodes
}

/// True when the KCL residual helpers evaluate a behavioral expression (any
/// behavioral source: `I={}` currents into `q`, `V={}` row residuals), which
/// reads `state` (`sim_time`, runtime scalars, `bsrc_x_prev`). Then
/// `kcl_residual_inl` takes `state`; otherwise it does not, because an unread
/// parameter is a warning in the generated crate.
pub(super) fn residual_needs_state(ir: &CircuitIR) -> bool {
    !ir.behavioral_sources.is_empty()
}

/// The trailing `state` argument the emitted `kcl_residual_inl` takes (see
/// [`residual_needs_state`]), or nothing.
pub(super) fn kcl_residual_inl_state_arg(ir: &CircuitIR) -> &'static str {
    if residual_needs_state(ir) {
        ", state"
    } else {
        ""
    }
}

/// Every behavioral source's expression as generated Rust at the residual
/// helpers' own `v`: `(source index, is_voltage, f, n+ index, n− index)`.
fn bsrc_residual_exprs(ir: &CircuitIR) -> Vec<(usize, bool, String, usize, usize)> {
    let params = bsrc_param_map(ir);
    ir.behavioral_sources
        .iter()
        .enumerate()
        .map(|(si, b)| {
            let res = NodalBsrcResolver {
                node_idx: &b.referenced_node_indices,
                params: &params,
            };
            let f = strip_outer_parens(&b.expr.simplify().to_rust(&res)).to_string();
            (si, b.is_voltage, f, b.n_plus_idx, b.n_minus_idx)
        })
        .collect()
}

/// Emit the TRUE residual of every behavioral `V={}` row into the KCL residual
/// helpers: `F[r] = (v[n+] − v[n−]) − f(v)` in volts, summed into `norm_sq` and
/// gated at `RELTOL·den + VNTOL` (1e-3·den + 1e-6 V; `den` = the row's largest
/// term). The base `A`
/// and `rhs` cannot give this row: the per-iteration solve carries the row as
/// a linearisation (`−∂f/∂V` in the chord factor, the companion in
/// `rhs_work`), so `A·v − rhs` on it is neither the linearised nor the true
/// residual. Reads the `bsrc_inv_dt` / `bsrc_half_dt` locals
/// [`emit_behavioral_current_residual`] binds, so it is emitted after it.
/// These rows never enter `max_abs` (the KCL-only infinity norm).
pub(super) fn emit_behavioral_voltage_row_residual(
    code: &mut String,
    ir: &CircuitIR,
    indent: &str,
) {
    for (si, is_voltage, f, np, nm) in bsrc_residual_exprs(ir) {
        if !is_voltage {
            continue;
        }
        let vp = if np > 0 {
            format!("v[{}]", np - 1)
        } else {
            "0.0_f64".to_string()
        };
        let vm = if nm > 0 {
            format!("v[{}]", nm - 1)
        } else {
            "0.0_f64".to_string()
        };
        code.push_str(&format!(
            "{indent}{{ // behavioral V={{}} source {si}: true row residual (V)
             {indent}    let f = {f};
             {indent}    let acc = ({vp} - {vm}) - f;
             {indent}    let den = {vp}.abs().max({vm}.abs()).max(f.abs());
             {indent}    norm_sq += acc * acc;
             {indent}    if !(acc.abs() <= RELTOL * den + VNTOL) {{ ok = false; }}
             {indent}}}
"
        ));
    }
}

/// Emit the behavioral `I={}` currents into the KCL residual's device-current
/// vector `q` (`F = A·v − rhs − q`, `q = N_i·i_nl` on entry), evaluated at the
/// residual's own `v`.
///
/// A current source's current enters the per-iteration solve only through the
/// companion `emit_behavioral_rhs` stamps into `rhs_work` and the Jacobian
/// `emit_behavioral_jacobian` stamps into the chord LU; the residual helpers
/// read the BASE `rhs` and `A`, so without this term the residual at the true
/// solution is off by `f(v)` on the terminal rows and the convergence gate never
/// passes on an `M > 0` build (one `I={}` source and one diode: 2300 of 2400
/// samples held). Same sign as the companion: `f` leaves `n+`, enters `n−`, so
/// `q[n+] -= f`, `q[n−] += f`. `V={}` sources need nothing here: their branch
/// current is a Newton unknown already in `A·v`.
pub(super) fn emit_behavioral_current_residual(code: &mut String, ir: &CircuitIR, indent: &str) {
    let exprs = bsrc_residual_exprs(ir);
    if exprs.is_empty() {
        return;
    }
    // `ddt`/`idt` surfaces read these two locals; bind them only when some
    // expression (current or voltage row, emitted later) reads them.
    for (local, field) in [
        ("bsrc_inv_dt", "bsrc_inv_dt"),
        ("bsrc_half_dt", "bsrc_half_dt"),
    ] {
        if exprs.iter().any(|(_, _, f, _, _)| f.contains(local)) {
            code.push_str(&format!("{indent}let {local} = state.{field};\n"));
        }
    }
    let sources: Vec<(usize, String, usize, usize)> = exprs
        .into_iter()
        .filter(|(_, is_voltage, _, _, _)| !is_voltage)
        .map(|(si, _, f, np, nm)| (si, f, np, nm))
        .collect();
    if sources.is_empty() {
        return;
    }
    code.push_str(&format!(
        "{indent}// Behavioral I={{}} currents: not Newton unknowns, evaluated at this v.\n"
    ));
    for (si, f, np, nm) in sources {
        code.push_str(&format!("{indent}let bsrc_{si}_f_resid = {f};\n"));
        if np > 0 {
            code.push_str(&format!("{indent}q[{}] -= bsrc_{si}_f_resid;\n", np - 1));
        }
        if nm > 0 {
            code.push_str(&format!("{indent}q[{}] += bsrc_{si}_f_resid;\n", nm - 1));
        }
    }
}
