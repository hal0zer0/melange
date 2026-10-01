//! Integrator sites (`PinSite`, `NewtonSite`, `SchurSite`), the noise mode,
//! the history / `q_dot` emitters, and the per-route predicates.

use super::sat_ind::emit_sat_ind_q_dot;
use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::dk_emitter::emit_inject_rhs_stamp;
use crate::codegen::rust_emitter::helpers::{
    body_effect_mosfets, carries_q_dot, has_latched_device,
};

/// Where an active-set pinned resolve is emitted.
///
/// The two nodal routes surface an unsolved pin differently. Schur counts it at
/// commit, from `last_nr_iterations`. Full-LU has no such check (its own
/// failures take the hold path), so the resolve counts the failure itself, at
/// the internal rate: a count derived from `last_nr_iterations` at the end of
/// a host sample would miss every oversampled sub-step but the last.
#[derive(Clone, Copy)]
pub(super) enum PinSite<'a> {
    Schur,
    FullLu {
        /// The site's integrator scalar for the saturating-inductor stamps.
        sat_alpha: &'a str,
        setter_stamps: &'a std::collections::BTreeSet<(usize, usize)>,
    },
}

/// The G and C a matrix build must read: the working copies the `.pot` /
/// `.switch` setters write when the circuit has any, else the compile-time
/// constants (identical then). `rebuild_matrices` and the full-LU sub-step
/// both take them from here, so a sub-step can never solve the nominal
/// circuit while the knobs say otherwise. `recv` is `self` or `state`.
pub(super) fn live_g_c(ir: &CircuitIR, full_nodal: bool, recv: &str) -> (String, String) {
    if ir.pots.is_empty() && ir.switches.is_empty() {
        return ("G".to_string(), "C".to_string());
    }
    let cold = if full_nodal { "cold." } else { "" };
    (
        format!("{recv}.{cold}g_work"),
        format!("{recv}.{cold}c_work"),
    )
}

/// Which integrator one emitted full-LU solve runs, and the state it reads.
/// The primary solve uses the build's own matrices (`state.a` is
/// backward-Euler-baked on a BE build); the BE instance of a trapezoidal build
/// uses `state.a_be` / `state.a_neg_be` / `RHS_CONST_BE`, which the IR bakes by
/// the same expressions as a BE build's `a` / `a_neg` / `RHS_CONST`, so the two
/// are the same arithmetic.
pub(super) struct NewtonSite {
    pub(super) be: bool,
    pub(super) a: &'static str,
    pub(super) a_neg: &'static str,
    be_instance: bool,
    pub(super) rhs_const: Option<&'static str>,
    pub(super) sat_alpha: String,
    pub(super) iter_budget: &'static str,
}

const SAT_INT_RATE: &str = "state.current_sample_rate * OVERSAMPLING_FACTOR as f64";

impl NewtonSite {
    pub(super) fn primary(ir: &CircuitIR) -> Self {
        let be = ir.solver_config.backward_euler;
        NewtonSite {
            be,
            a: "state.a",
            a_neg: "state.a_neg",
            be_instance: false,
            rhs_const: ir.has_dc_sources.then_some("RHS_CONST"),
            sat_alpha: if be {
                format!("({SAT_INT_RATE})")
            } else {
                format!("(2.0 * {SAT_INT_RATE})")
            },
            iter_budget: "MAX_ITER",
        }
    }

    pub(super) fn be_instance(ir: &CircuitIR) -> Self {
        NewtonSite {
            be: true,
            a: "state.a_be",
            a_neg: "state.a_neg_be",
            be_instance: true,
            rhs_const: (ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty())
                .then_some("RHS_CONST_BE"),
            sat_alpha: format!("({SAT_INT_RATE})"),
            iter_budget: if ir.solver_config.breakpoint_be {
                "be_iter_budget"
            } else {
                "MAX_ITER"
            },
        }
    }

    pub(super) fn a_neg_sparsity<'a>(
        &self,
        ir: &'a CircuitIR,
    ) -> &'a crate::codegen::ir::MatrixSparsity {
        if self.be_instance {
            &ir.sparsity.a_neg_be
        } else {
            &ir.sparsity.a_neg
        }
    }
}

/// The Schur counterpart of [`NewtonSite`]: which integrator one emitted
/// Schur solve runs and the precomputed kernel it reads. A BE build's
/// `s`/`k`/`s_ni`/`a`/`a_neg`/`RHS_CONST` are backward-Euler-baked; a
/// trapezoidal build's BE instance reads the `_be` twins, which the IR bakes
/// by the same expressions.
pub(super) struct SchurSite {
    pub(super) be: bool,
    be_instance: bool,
    pub(super) s: &'static str,
    pub(super) k: &'static str,
    pub(super) s_ni: &'static str,
    pub(super) a: &'static str,
    pub(super) a_neg: &'static str,
    pub(super) rhs_const: Option<&'static str>,
    pub(super) iter_budget: &'static str,
}

impl SchurSite {
    pub(super) fn primary(ir: &CircuitIR) -> Self {
        SchurSite {
            be: ir.solver_config.backward_euler,
            be_instance: false,
            s: "state.s",
            k: "state.k",
            s_ni: "state.s_ni",
            a: "state.a",
            a_neg: "state.a_neg",
            rhs_const: ir.has_dc_sources.then_some("RHS_CONST"),
            iter_budget: "MAX_ITER",
        }
    }

    pub(super) fn be_instance(ir: &CircuitIR) -> Self {
        SchurSite {
            be: true,
            be_instance: true,
            s: "state.s_be",
            k: "state.k_be",
            s_ni: "state.s_ni_be",
            a: "state.a_be",
            a_neg: "state.a_neg_be",
            rhs_const: (ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty())
                .then_some("RHS_CONST_BE"),
            iter_budget: if ir.solver_config.breakpoint_be {
                "be_iter_budget"
            } else {
                "MAX_ITER"
            },
        }
    }

    pub(super) fn a_neg_sparsity<'a>(
        &self,
        ir: &'a CircuitIR,
    ) -> &'a crate::codegen::ir::MatrixSparsity {
        if self.be_instance {
            &ir.sparsity.a_neg_be
        } else {
            &ir.sparsity.a_neg
        }
    }

    pub(super) fn k_sparsity<'a>(
        &self,
        ir: &'a CircuitIR,
    ) -> &'a crate::codegen::ir::MatrixSparsity {
        if self.be_instance {
            &ir.sparsity.k_be
        } else {
            &ir.sparsity.k
        }
    }
}

/// Noise in one solve's RHS: draw fresh (the sample's primary solve), replay
/// the draw cached this sample (a fallback), or decide at runtime.
pub(super) enum NoiseMode {
    Draw,
    Replay,
    DrawIf(&'static str),
}

/// Step 1 of every solve: the constant sources, the integrator history and
/// the inputs, into a new local `rhs`.
///
/// Both integrators take the charge (companion) form: the history is
/// `H·v_prev` with `H = alpha·C` (`a_neg`), and every source enters once, at
/// `n+1`. A trapezoidal solve (`trap`) adds the carried charge derivative
/// `q_dot`. With KCL exact at `n`, this equals the whole-system trapezoidal RHS
/// `2·RHS_CONST + (alpha·C − G)·v_n + N_i·i_nl(n) + b(n) + b(n+1)` on the
/// charge-carrying rows (`COMPANION_MODELS.md`). When the accepted solve
/// leaves a KCL residual, the whole-system form feeds it back into the next
/// sample on the algebraic rows as a z = −1 memory; this form does not. The
/// nonlinear current enters through the Newton solve at `n+1` alone.
pub(super) fn emit_history_rhs(
    code: &mut String,
    ir: &CircuitIR,
    rhs_const: Option<&str>,
    a_neg: &str,
    a_neg_sparsity: &crate::codegen::ir::MatrixSparsity,
    trap: bool,
) {
    let n = ir.topology.n;
    code.push_str(if trap {
        "    // Step 1: RHS = constant sources + alpha*C*v_prev + q_dot + inputs at n+1\n"
    } else {
        "    // Step 1: RHS = constant sources + (1/T)*C*v_prev + inputs at n+1\n"
    });
    match rhs_const {
        Some(rc) => code.push_str(&format!("    let mut rhs = {rc};\n")),
        None => code.push_str("    let mut rhs = [0.0f64; N];\n"),
    }
    for i in 0..n {
        for &j in &a_neg_sparsity.nz_by_row[i] {
            code.push_str(&format!(
                "    rhs[{i}] += {a_neg}[{i}][{j}] * state.v_prev[{j}];\n"
            ));
        }
        if trap && !a_neg_sparsity.nz_by_row[i].is_empty() {
            code.push_str(&format!("    rhs[{i}] += state.q_dot[{i}];\n"));
        }
    }
    code.push('\n');
    if ir.solver_config.num_inputs() > 1 {
        code.push_str("    // Input sources (Thevenin, V_in(n+1) * G_in per port)\n");
        code.push_str("    for k in 0..NUM_INPUTS {\n        rhs[INPUT_NODES[k]] += inputs[k] / INPUT_RESISTANCES[k];\n    }\n");
    } else {
        code.push_str("    // Input source (Thevenin, V_in(n+1) * G_in)\n");
        code.push_str("    let input_conductance = 1.0 / INPUT_RESISTANCE;\n");
        code.push_str("    rhs[INPUT_NODE] += input * input_conductance;\n");
    }
    if ir.solver_config.has_inject_or_tap() {
        code.push_str(&emit_inject_rhs_stamp(ir, "rhs", "    "));
    }
}

/// Charge form, trapezoidal builds: the per-sample locals that record how the
/// committed sample was integrated. `q_be` means a backward-Euler solve
/// produced it. `q_sub` holds its `q_dot` when that is already known: from
/// adaptive sub-steps or a sub-sample fire, or unchanged on a hold.
/// `be_possible`: the build has a backward-Euler solve (else `q_be` is not
/// declared); must match the [`emit_q_dot_commit`] argument.
pub(super) fn emit_q_dot_locals(
    code: &mut String,
    ir: &CircuitIR,
    indent: &str,
    be_possible: bool,
) {
    if !carries_q_dot(ir) {
        return;
    }
    code.push_str(&format!(
        "{indent}// Charge form: how the committed sample is integrated (see q_dot).\n"
    ));
    if be_possible {
        code.push_str(&format!("{indent}let mut q_be = false;\n"));
    }
    code.push_str(&format!(
        "{indent}#[allow(unused_mut)]\n\
         {indent}let mut q_sub: Option<[f64; N]> = None;\n"
    ));
}

/// Charge form: `q_dot` for the committed `v`, before `v_prev` moves. A
/// trapezoidal step gives `alpha·C·(v − v_prev) − q_dot`, a backward-Euler
/// step `(1/T)·C·(v − v_prev)`. A backward-Euler step thus re-seeds `q_dot`
/// from its own capacitor current. On a saturating inductor's branch row the
/// charge is the flux `Φ(i)`, not `L0·i`. `be_possible`: the build has a
/// backward-Euler solve.
pub(super) fn emit_q_dot_commit(
    code: &mut String,
    ir: &CircuitIR,
    indent: &str,
    be_possible: bool,
) {
    if !carries_q_dot(ir) {
        return;
    }
    let n = ir.topology.n;
    let rate = "state.current_sample_rate * OVERSAMPLING_FACTOR as f64";
    let update = |code: &mut String,
                  mat: &str,
                  sp: &crate::codegen::ir::MatrixSparsity,
                  trap: bool,
                  alpha: &str,
                  ind: &str| {
        for i in 0..n {
            let terms: Vec<String> = sp.nz_by_row[i]
                .iter()
                .map(|&j| format!("{mat}[{i}][{j}] * (v[{j}] - state.v_prev[{j}])"))
                .collect();
            if terms.is_empty() {
                continue;
            }
            let tail = if trap {
                format!(" - state.q_dot[{i}]")
            } else {
                String::new()
            };
            code.push_str(&format!("{ind}q[{i}] = {}{tail};\n", terms.join(" + ")));
        }
        emit_sat_ind_q_dot(code, ir, "q", "v", "state.v_prev", alpha, ind);
    };
    code.push_str(&format!(
        "{indent}// Charge form: q_dot = C*dx/dt at the committed sample.\n\
         {indent}state.q_dot = match q_sub {{\n\
         {indent}    Some(q) => q,\n\
         {indent}    None => {{\n\
         {indent}        let mut q = [0.0f64; N];\n"
    ));
    let ind2 = format!("{indent}            ");
    let ind1 = format!("{indent}        ");
    if be_possible {
        code.push_str(&format!("{ind1}if q_be {{\n"));
        update(
            code,
            "state.a_neg_be",
            &ir.sparsity.a_neg_be,
            false,
            &format!("({rate})"),
            &ind2,
        );
        code.push_str(&format!("{ind1}}} else {{\n"));
        update(
            code,
            "state.a_neg",
            &ir.sparsity.a_neg,
            true,
            &format!("(2.0 * {rate})"),
            &ind2,
        );
        code.push_str(&format!("{ind1}}}\n"));
    } else {
        update(
            code,
            "state.a_neg",
            &ir.sparsity.a_neg,
            true,
            &format!("(2.0 * {rate})"),
            &ind1,
        );
    }
    code.push_str(&format!("{ind1}q\n{indent}    }}\n{indent}}};\n"));
}

/// A trapezoidal full-LU build with a Newton loop carries a backward-Euler
/// instance of the same solve, run on a failed trapezoidal sample, while
/// latched, on a breakpoint, or when ActiveSetBe sees a rail. Behavioral
/// circuits (BE-primary) and BE builds have only the primary.
pub(super) fn has_be_instance(ir: &CircuitIR) -> bool {
    (ir.topology.m > 0 || !ir.saturating_inductors.is_empty())
        && ir.behavioral_sources.is_empty()
        && !ir.solver_config.backward_euler
}

/// The cross-timestep chord cache, as `let mut chord_* = {src}*;` locals
/// (`src` = `state.chord_` or `state.chord_be_`), or stored back.
pub(super) fn emit_chord_cache(
    code: &mut String,
    ir: &CircuitIR,
    src: &str,
    indent: &str,
    load: bool,
) {
    let mut fields = vec!["lu", "dr", "dc", "perm", "j_dev"];
    if !body_effect_mosfets(ir).is_empty() {
        fields.push("body_gmb");
    }
    fields.push("valid");
    if ir.sparsity.lu.is_some() {
        fields.push("dense");
    }
    for f in fields {
        if load {
            code.push_str(&format!("{indent}let mut chord_{f} = {src}{f};\n"));
        } else {
            code.push_str(&format!("{indent}{src}{f} = chord_{f};\n"));
        }
    }
}

/// True when a Newton solve in this build can end unsolved, so the
/// death-spiral hold and its `diag_nr_hold_count` exist: any nonlinear device,
/// behavioral source or saturating inductor. The same rule on both nodal
/// routes (behavioral sources and saturating inductors always route full-LU).
pub(super) fn emits_hold(ir: &CircuitIR) -> bool {
    ir.topology.m > 0 || !ir.behavioral_sources.is_empty() || !ir.saturating_inductors.is_empty()
}

/// Whether a failed op-amp pin at `site` is committed (and counted in
/// `diag_nr_unconverged_commit_count`) rather than handed to the sample's
/// `converged`. See [`RustEmitter::emit_nodal_active_set_resolve`].
pub(super) fn pin_failure_is_committed(ir: &CircuitIR, site: &PinSite<'_>) -> bool {
    matches!(site, PinSite::FullLu { .. }) || ir.topology.m == 0
}

/// True when some pin in this build commits its failure: the build declares
/// `diag_nr_unconverged_commit_count`.
pub(super) fn counts_unconverged_commit(ir: &CircuitIR, full_nodal: bool) -> bool {
    emits_active_set_resolve(ir) && (full_nodal || ir.topology.m == 0)
}

/// Whether a nodal build can emit the active-set pinned resolve at all.
fn emits_active_set_resolve(ir: &CircuitIR) -> bool {
    matches!(
        ir.solver_config.opamp_rail_mode,
        crate::codegen::OpampRailMode::ActiveSet | crate::codegen::OpampRailMode::ActiveSetBe
    ) && ir
        .opamps
        .iter()
        .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
}

/// True when a Schur build seeds its Newton at full-LU's starting point
/// ([`RustEmitter::emit_schur_warm_start`]) and so carries the cached `K`
/// factorisations and `diag_warm_start_fallback_count`.
pub(super) fn schur_exact_seed(ir: &CircuitIR, full_nodal: bool) -> bool {
    !full_nodal && ir.topology.m > 0 && !has_latched_device(ir)
}

/// `k_factor` / `k_solve`: a rank-revealing LU (full pivoting) of the M×M
/// kernel `K`, for the Newton warm start that solves `K·i_nl = N_v·v_prev − p`.
///
/// `K` is singular whenever two devices share a controlling voltage (an
/// antiparallel diode pair: `K = [[−a, a], [a, −a]]`). The system is then still
/// consistent, and ANY solution gives the same Newton sequence: the first
/// update depends on `i_nl` only through `K·i_nl` (= `v_d − p`), and so does
/// every step after it. So a singular `K` is solved with its free variables at
/// zero. Only an inconsistent system (the start is not reachable through the
/// device currents) returns `None`.
pub(super) fn emit_k_seed_helpers() -> String {
    "/// Rank-revealing LU (full pivoting) of the M×M kernel K, for the Newton\n\
     /// warm start. Returns (factors, row order, column order, rank).\n\
     fn k_factor(k: &[[f64; M]; M]) -> ([[f64; M]; M], [usize; M], [usize; M], usize) {\n\
     \x20   let mut a = *k;\n\
     \x20   let mut pr = [0usize; M];\n\
     \x20   let mut pc = [0usize; M];\n\
     \x20   for i in 0..M { pr[i] = i; pc[i] = i; }\n\
     \x20   let mut scale = 0.0f64;\n\
     \x20   for r in 0..M { for c in 0..M { scale = scale.max(a[r][c].abs()); } }\n\
     \x20   let tol = (scale * 1e-12).max(1e-300);\n\
     \x20   let mut rank = 0;\n\
     \x20   for c in 0..M {\n\
     \x20       let (mut br, mut bc, mut bv) = (c, c, 0.0f64);\n\
     \x20       for r in c..M { for q in c..M { let v = a[r][q].abs(); if v > bv { bv = v; br = r; bc = q; } } }\n\
     \x20       if !(bv > tol) { break; }\n\
     \x20       a.swap(c, br);\n\
     \x20       pr.swap(c, br);\n\
     \x20       for row in a.iter_mut() { row.swap(c, bc); }\n\
     \x20       pc.swap(c, bc);\n\
     \x20       for r in c + 1..M {\n\
     \x20           let f = a[r][c] / a[c][c];\n\
     \x20           a[r][c] = f;\n\
     \x20           for q in c + 1..M { a[r][q] -= f * a[c][q]; }\n\
     \x20       }\n\
     \x20       rank += 1;\n\
     \x20   }\n\
     \x20   (a, pr, pc, rank)\n\
     }\n\n\
     /// A solution of K·x = b from `k_factor`'s factors (free variables zero),\n\
     /// or None when the system is inconsistent.\n\
     #[inline]\n\
     fn k_solve(lu: &[[f64; M]; M], pr: &[usize; M], pc: &[usize; M], rank: usize, b: [f64; M]) -> Option<[f64; M]> {\n\
     \x20   let mut y = [0.0f64; M];\n\
     \x20   let mut bmax = 0.0f64;\n\
     \x20   for i in 0..M { y[i] = b[pr[i]]; bmax = bmax.max(b[i].abs()); }\n\
     \x20   for c in 0..rank { for r in c + 1..M { y[r] -= lu[r][c] * y[c]; } }\n\
     \x20   for r in rank..M { if !(y[r].abs() <= 1e-9 * bmax) { return None; } }\n\
     \x20   let mut z = [0.0f64; M];\n\
     \x20   for c in (0..rank).rev() {\n\
     \x20       let mut acc = y[c];\n\
     \x20       for q in c + 1..rank { acc -= lu[c][q] * z[q]; }\n\
     \x20       z[c] = acc / lu[c][c];\n\
     \x20   }\n\
     \x20   let mut x = [0.0f64; M];\n\
     \x20   for i in 0..M { x[pc[i]] = z[i]; }\n\
     \x20   Some(x)\n\
     }\n\n"
        .to_string()
}
