//! The shared front-end pipeline.
//!
//! Everything a consumer must do between parsing a netlist and generating code,
//! in one place — because for a long time it lived in four places and they drifted.
//!
//! `melange compile`, `melange simulate`, `melange analyze` and `melange-validate`
//! each grew their own copy of this sequence. Measured 2026-09-02:
//!
//! | step | compile | simulate | analyze | validate |
//! |---|---|---|---|---|
//! | [`apply_linearize_reductions`] | yes | yes | yes | **no** |
//! | [`expand_internal_nodes`] (then gated at `min diag(K) < -100`) | yes | yes | **unconditional** | **unconditional** |
//! | [`auto_tune_max_iter`] | yes | yes | yes | **no** |
//!
//! **The sequence itself now lives in one place, [`crate::build::build`]**, which
//! every verb calls; the table below is the history of why.
//!
//! **That table is the 2026-09-02 state and every cell is now closed**, but two
//! of them outlived the commit that is usually credited with closing them, so
//! read it as history and not as a status board:
//!
//! * `6bc3ef1` fixed the **validate** column. It did not touch
//!   `crates/melange-validate/tests/spice_validation.rs` — the harness the CI
//!   "SPICE validation" gate actually runs — which had its own `from_netlist`
//!   build and ran none of these steps. Closed 2026-09-03; that harness now
//!   delegates to `melange_validate::run_melange_solver_from_str`.
//! * The **analyze** cell above stayed literally true until 2026-09-03. Analyze
//!   expanded internal nodes unconditionally long after the same defect was
//!   removed from validate, so `melange analyze` reported the response of a
//!   different circuit than compile ships for any deck with
//!   `k_diag_min < -100` (measured on wurli-power-amp: compile skipped
//!   expansion, analyze expanded). All four consumers now call
//!   [`expand_internal_nodes`]; nobody hand-rolls it.
//!
//! Forward-active and grid-off reduction were never in this table and were
//! private to `melange-cli` until 2026-09-03. They are now
//! [`apply_forward_active_reduction`] and [`apply_grid_off_reduction`].
//!
//! On `wurli-power-amp` — the shipped OpenWurli power stage — the CLI built an
//! N=20, M=14 system while `melange validate` built N=44, M=16: more than twice
//! the nodes, and a different solver sub-path. Validation reported 1319% RMS
//! error and correlation 0.0002 against ngspice, which read as a catastrophic
//! solver defect and was nothing of the kind. Driven through the build that
//! ships, the same circuit validates at 0.0924 % RMS, correlation 0.9999996
//! (`melange validate` defaults, 1 s; peak error 3.3e-2 V against a 2e-2 V
//! tolerance; measured 2026-09-29).
//!
//! **A verification instrument that builds a different system than the one it is
//! verifying is worse than no instrument, because it is believed.**
//!
//! Diagnostics go through a `rep` callback rather than `println!`, so a library
//! caller stays silent while the CLI prints exactly what it always did.

/// Route a diagnostic line to the caller's reporter.
///
/// Shaped so a moved function needs `println!(` swapped for `report!(rep, ` and
/// nothing else — no closing-paren surgery on the multi-line calls, which is
/// where transcription errors get into a refactor.
macro_rules! report {
    ($rep:expr, $($arg:tt)*) => { ($rep)(format_args!($($arg)*)) };
}

/// A reporter for human-facing progress lines. The CLI passes
/// `&|a| println!("{a}")`; library callers pass [`silent`].
pub type Reporter<'a> = &'a dyn Fn(std::fmt::Arguments<'_>);

/// A no-op reporter, for callers that want the pipeline without the narration.
pub fn silent(_: std::fmt::Arguments<'_>) {}

/// Failure inside the shared pipeline.
#[derive(Debug)]
pub enum PipelineError {
    /// The DC operating point needed for `.linearize` could not be solved.
    DcOp(String),
    /// An MNA rebuild for a dimension reduction (forward-active / grid-off)
    /// failed.
    Mna(String),
    /// A `.linearize` directive that cannot hold: a device outside the
    /// region its small-signal model assumes at its own operating point, or a
    /// name that is not a BJT or triode.
    Linearize(String),
}

impl std::fmt::Display for PipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DcOp(m) => write!(f, "linearize: DC operating point failed: {m}"),
            Self::Mna(m) => write!(f, "{m}"),
            Self::Linearize(m) => write!(f, ".linearize: {m}"),
        }
    }
}

impl std::error::Error for PipelineError {}

#[derive(Default, Debug, Clone)]
pub struct LinearizeOutcome {
    pub bjts_linearized: usize,
    pub triodes_linearized: usize,
    /// When the bias solve the small-signal parameters came from did not
    /// converge: what it reached (method, iterations, residual). The build
    /// refuses it unless `--allow-unconverged-dc-op`.
    pub bias_unconverged: Option<String>,
}

/// Stamp each `(node, conductance)` to ground into `mna.g`: every input port
/// and every `.inject` source the build stamped. A reduction that rebuilds the
/// MNA from the netlist starts from an unstamped G and must restamp all of
/// them, or the shipped circuit loses a port's impedance.
fn stamp_ports(mna: &mut crate::mna::MnaSystem, port_stamps: &[(usize, f64)]) {
    for &(node, g) in port_stamps {
        if node < mna.n {
            mna.g[node][node] += g;
        }
    }
}

/// Apply `.linearize` directives to `mna` in-place.
///
/// Computes a DC OP on the current MNA to extract small-signal g-parameters for
/// flagged BJTs and triodes, then rebuilds the MNA via
/// `from_netlist_with_all_reductions` with those devices collapsed from
/// active-NR (2D per device) to linear stamps (0D). Re-stamps junction caps
/// against the reduced dimension and restamps every port (`port_stamps`).
///
/// No-op when the netlist has no `.linearize` directives, in which case any
/// FA / grid-off rebuilds the caller already did remain in place and this
/// returns `LinearizeOutcome::default()` without modifying `mna`.
///
/// **Skipping this step changes which solver sub-path the emitter picks.** The
/// `linearized_bypass` gate in `emit_nodal` (`nodal_emitter/mod.rs`) is the only thing routing
/// some circuits to full-LU; without a linearized device they get Schur NR
/// instead, which on an expanded-parasitic system diverges. That is precisely
/// how `melange validate` came to report 1319% RMS error on a circuit the
/// shipped build validates at 0.246%.
#[allow(clippy::too_many_arguments)]
pub fn apply_linearize_reductions(
    mna: &mut crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
    forward_active: &std::collections::HashSet<String>,
    grid_off_pentodes: &std::collections::HashMap<String, f64>,
    port_stamps: &[(usize, f64)],
    dc_request: crate::codegen::ir::DcOpRequest,
    rep: Reporter<'_>,
) -> Result<LinearizeOutcome, PipelineError> {
    use crate::parser::Element;

    // Partition .linearize names into BJT vs triode sets by matching against
    // element types. Warn on names that are neither.
    let linearize_names: std::collections::HashSet<String> =
        netlist.linearize_devices.iter().cloned().collect();
    let mut linearized_bjts_set: std::collections::HashSet<String> =
        std::collections::HashSet::new();
    let mut linearized_triodes_set: std::collections::HashSet<String> =
        std::collections::HashSet::new();
    if !linearize_names.is_empty() {
        for elem in &netlist.elements {
            match elem {
                Element::Bjt { name, .. }
                    if linearize_names.contains(&name.to_ascii_uppercase()) =>
                {
                    linearized_bjts_set.insert(name.to_ascii_uppercase());
                }
                Element::Triode { name, .. }
                    if linearize_names.contains(&name.to_ascii_uppercase()) =>
                {
                    linearized_triodes_set.insert(name.to_ascii_uppercase());
                }
                _ => {}
            }
        }
        for name in &linearize_names {
            let upper = name.to_ascii_uppercase();
            if !linearized_bjts_set.contains(&upper) && !linearized_triodes_set.contains(&upper) {
                return Err(PipelineError::Linearize(format!(
                    "'{name}' is not a BJT or triode of this deck; only those can be linearized"
                )));
            }
        }
    }

    let has_linearized = !linearized_bjts_set.is_empty() || !linearized_triodes_set.is_empty();
    if !has_linearized {
        return Ok(LinearizeOutcome::default());
    }

    // DC OP on the current (post-FA, pre-linearize) MNA to get the bias
    // point for small-signal g-parameter extraction.
    let device_slots =
        crate::codegen::ir::CircuitIR::build_device_info_with_mna(netlist, Some(mna))
            .map_err(|e| PipelineError::DcOp(format!("device models: {e}")))?;
    let dc_result = crate::dc_op::solve_dc_operating_point(
        mna,
        &device_slots,
        &crate::codegen::ir::dc_op_config(mna, dc_request),
    );

    // Extract BJT small-signal g-params (gm, gpi, gmu, go) at DC bias.
    let mut bjt_lin_infos = Vec::new();
    for slot in &device_slots {
        if let crate::codegen::ir::DeviceParams::Bjt(bp) = &slot.params {
            let dev = mna
                .nonlinear_devices
                .iter()
                .find(|d| d.start_idx == slot.start_idx);
            if let Some(dev) = dev {
                if linearized_bjts_set.contains(&dev.name.to_ascii_uppercase()) {
                    let s = slot.start_idx;
                    let (nc, nb, ne) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    // Node-voltage lookup (1-indexed device nodes, 0 = ground).
                    let v_at = |idx: usize| -> f64 {
                        if idx > 0 {
                            dc_result.v_node.get(idx - 1).copied().unwrap_or(0.0)
                        } else {
                            0.0
                        }
                    };
                    // Bias currents are the ones the bias solve balanced KCL with.
                    let ic = dc_result.i_nl.get(s).copied().unwrap_or(0.0);
                    // Guard on slot.dimension: for an FA-reduced (1D) BJT,
                    // i_nl[s+1] belongs to the NEXT device's slot.
                    // FA contract: Ib = Ic / BF.
                    let ib = if slot.dimension == 2 {
                        dc_result.i_nl.get(s + 1).copied().unwrap_or(0.0)
                    } else {
                        ic / bp.beta_f
                    };

                    // Norton operating-point voltages in EXTERNAL node space
                    // (the linearized device is stamped between the external
                    // terminals, so the I0 - J·v0 constant must use external
                    // node differences — see LinearizedBjtInfo docs).
                    let vbe0 = v_at(nb) - v_at(ne);
                    let vbc0 = v_at(nb) - v_at(nc);
                    // The small-signal model assumes forward active, and the
                    // runtime check (LinearizedCheck::Bjt) refuses every sample
                    // outside it; a device already outside at its own
                    // operating point is refused here, with the evidence.
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    if sign * vbc0 > 0.0 {
                        return Err(PipelineError::Linearize(format!(
                            "{} is saturated at its own operating point (Vbc = {vbc0:+.3} V, \
                             B-C junction forward): its small-signal model assumes forward \
                             active, so the linearization is invalid here",
                            dev.name
                        )));
                    }
                    // Cut off: the B-E junction not forward biased (a real
                    // device's leakage keeps Ic of the forward sign even
                    // there), or no forward collector current at all.
                    if sign * vbe0 <= 0.0 || sign * ic <= 0.0 {
                        return Err(PipelineError::Linearize(format!(
                            "{} is cut off at its own operating point (Vbe = {vbe0:+.3} V, \
                             Ic = {ic:e} A): its small-signal model assumes forward active, so \
                             the linearization is invalid here",
                            dev.name
                        )));
                    }
                    // The small-signal model is the device's own Jacobian at the
                    // bias point, from the evaluator the bias solve used: NF/NR,
                    // Gummel-Poon qb (Early, high injection), ISE/ISC leakage,
                    // and RB/RC/RE folded in through the terminal-pair solve.
                    let (_, _, jac) = crate::dc_op::bjt_eval(bp, vbe0, vbc0, bp.has_parasitics());
                    let [dic_dvbe, dic_dvbc, dib_dvbe, dib_dvbc] = jac;
                    // Junction capacitances at the bias point's terminal
                    // voltages, as `MnaSystem::relinearize_bjt_caps_at_dc_op`
                    // evaluates an unlinearized 2D BJT.
                    let (cbe, cbc) = bp.linearized_junction_caps(vbe0, vbc0);
                    report!(
                        rep,
                        "  Linearized {}: dIc/dVbe={:.4e} dIc/dVbc={:.4e} dIb/dVbe={:.4e} \
                         dIb/dVbc={:.4e} Ic_dc={:.4e} Ib_dc={:.4e}",
                        dev.name,
                        dic_dvbe,
                        dic_dvbc,
                        dib_dvbe,
                        dib_dvbc,
                        ic,
                        ib
                    );
                    bjt_lin_infos.push(crate::mna::LinearizedBjtInfo {
                        name: dev.name.clone(),
                        nc,
                        nb,
                        ne,
                        dic_dvbe,
                        dic_dvbc,
                        dib_dvbe,
                        dib_dvbc,
                        cbe,
                        cbc,
                        ic_dc: ic,
                        ib_dc: ib,
                        vbe0,
                        vbc0,
                        is_pnp: bp.is_pnp,
                    });
                }
            }
        }
    }

    // Extract triode small-signal params (gm, rp=1/gp) at DC bias. A triode
    // cut off or with its grid past the conduction onset at its own operating
    // point is outside the region the linearization assumes: refused.
    let mut triode_lin_infos = Vec::new();
    for slot in &device_slots {
        if let crate::codegen::ir::DeviceParams::Tube(tp) = &slot.params {
            if tp.is_pentode() {
                continue;
            }
            let dev = mna
                .nonlinear_devices
                .iter()
                .find(|d| d.start_idx == slot.start_idx);
            if let Some(dev) = dev {
                if linearized_triodes_set.contains(&dev.name.to_ascii_uppercase()) {
                    let s = slot.start_idx;
                    let vgk = dc_result.v_nl.get(s).copied().unwrap_or(0.0);
                    let vpk = dc_result.v_nl.get(s + 1).copied().unwrap_or(0.0);
                    let ip_dc = dc_result.i_nl.get(s).copied().unwrap_or(0.0);
                    let ig_dc = dc_result.i_nl.get(s + 1).copied().unwrap_or(0.0);

                    // The grid's conduction onset: the Vgk at which the fitted
                    // D&Z grid law reaches the manufacturers' +0.3 uA
                    // starting-point criterion.
                    let onset = melange_devices::KorenTriode {
                        mu: tp.mu,
                        ex: tp.ex,
                        kg1: tp.kg1,
                        kp: tp.kp,
                        kvb: tp.kvb,
                        gg: tp.gg,
                        xi: tp.xi,
                        cg: tp.cg,
                        lambda: tp.lambda,
                        mu_b: tp.mu_b,
                        svar: tp.svar,
                        ex_b: tp.ex_b,
                    }
                    .grid_voltage_at_current(melange_devices::tube::GRID_START_CRITERION_A)
                    .unwrap_or(0.0);
                    // The region's edges, as the runtime check
                    // (LinearizedCheck::Triode) enforces them: a device outside
                    // at its own operating point is refused with the evidence,
                    // never silently kept nonlinear against the directive.
                    if vgk > onset {
                        return Err(PipelineError::Linearize(format!(
                            "{}'s grid is past its conduction onset at its own operating \
                             point (Vgk = {vgk:+.3} V, onset {onset:+.3} V, Ig = {ig_dc:e} A): \
                             its small-signal model assumes a non-conducting grid, so the \
                             linearization is invalid here",
                            dev.name
                        )));
                    }
                    if ip_dc <= 0.0 {
                        return Err(PipelineError::Linearize(format!(
                            "{} is cut off at its own operating point (Ip = {ip_dc:e} A): its \
                             small-signal model needs plate current, so the linearization is \
                             invalid here",
                            dev.name
                        )));
                    }

                    let triode = melange_devices::KorenTriode {
                        mu: tp.mu,
                        ex: tp.ex,
                        kg1: tp.kg1,
                        kp: tp.kp,
                        kvb: tp.kvb,
                        gg: tp.gg,
                        xi: tp.xi,
                        cg: tp.cg,
                        lambda: tp.lambda,
                        mu_b: tp.mu_b,
                        svar: tp.svar,
                        ex_b: tp.ex_b,
                    };
                    // Through RGI, as the DC OP and the transient evaluate it.
                    let (_, _, jac) = triode.evaluate_with_rgi(vgk, vpk, tp.rgi);
                    let gm = jac[0]; // dIp/dVgk
                    let gp = jac[1]; // dIp/dVpk = 1/rp

                    let (ng, np, nk) = (
                        dev.node_indices[0], // grid
                        dev.node_indices[1], // plate
                        dev.node_indices[2], // cathode
                    );
                    // Norton operating-point voltages in EXTERNAL node space
                    // (see LinearizedTriodeInfo docs). For triodes without
                    // internal nodes these equal v_nl[s]/v_nl[s+1].
                    let v_at = |idx: usize| -> f64 {
                        if idx > 0 {
                            dc_result.v_node.get(idx - 1).copied().unwrap_or(0.0)
                        } else {
                            0.0
                        }
                    };
                    let vgk0 = v_at(ng) - v_at(nk);
                    let vpk0 = v_at(np) - v_at(nk);
                    let rp = if gp.abs() > 1e-30 {
                        1.0 / gp
                    } else {
                        f64::INFINITY
                    };
                    report!(
                        rep,
                        "  Linearized {}: gm={:.4e} rp={:.0} Ip_dc={:.4e} Vgk={:.2}V Vpk={:.1}V",
                        dev.name,
                        gm,
                        rp,
                        ip_dc,
                        vgk,
                        vpk
                    );
                    triode_lin_infos.push(crate::mna::LinearizedTriodeInfo {
                        name: dev.name.clone(),
                        ng,
                        np,
                        nk,
                        gm,
                        gp,
                        ip_dc,
                        ig_dc,
                        vgk0,
                        vpk0,
                        ccg: tp.ccg,
                        cgp: tp.cgp,
                        ccp: tp.ccp,
                        grid_onset: onset,
                    });
                }
            }
        }
    }

    // Rebuild MNA with all three reduction classes combined (FA +
    // linearized + grid-off). This supersedes any prior FA-only or
    // grid-off-only rebuild the caller performed.
    // A bias solve that did not converge is not a point to linearize at: the
    // linearized circuit's own DC operating point would then solve cleanly
    // around parameters taken from a non-solution. Described here, while the
    // pre-rebuild node names still index its residual; the build refuses it.
    let bias_unconverged = (!dc_result.converged).then(|| {
        let names = mna.node_names_in_index_order();
        let worst = dc_result
            .kcl_worst_row
            .and_then(|row| names.get(row + 1).copied())
            .filter(|n| !n.is_empty())
            .map(|n| format!(" at v({n})"))
            .unwrap_or_default();
        format!(
            "the .linearize bias solve did not converge ({:?}, {} iterations; KCL residual \
             {:.3e} A{worst})",
            dc_result.method, dc_result.iterations, dc_result.kcl_residual_max
        )
    });
    // The point the devices were linearized at, by node name (the rebuild may
    // renumber): the linearized system's DC operating point starts there.
    let bias_nodes: Option<std::collections::BTreeMap<String, f64>> =
        dc_result.converged.then(|| {
            mna.node_map
                .iter()
                .filter(|&(_, &idx)| idx > 0)
                .filter_map(|(name, &idx)| {
                    dc_result.v_node.get(idx - 1).map(|&v| (name.clone(), v))
                })
                .collect()
        });
    *mna = crate::mna::MnaSystem::from_netlist_with_all_reductions(
        netlist,
        forward_active,
        &linearized_bjts_set,
        &linearized_triodes_set,
        grid_off_pentodes,
    )
    .map_err(|e| PipelineError::DcOp(format!("rebuild MNA with linearized devices: {e}")))?;
    stamp_ports(mna, port_stamps);
    mna.linearize_bias_nodes = bias_nodes;

    // Stamp linearized g-parameters into G. Must precede the junction-cap
    // re-stamp so `build_device_info_with_mna` can skip linearized devices
    // (it checks `mna.linearized_bjts` / `mna.linearized_triodes`).
    if !bjt_lin_infos.is_empty() {
        mna.linearized_bjts = bjt_lin_infos;
        mna.stamp_linearized_bjts();
        report!(
            rep,
            "  Linearized {} BJTs (M reduced by {})",
            linearized_bjts_set.len(),
            linearized_bjts_set.len() * 2
        );
    }
    if !triode_lin_infos.is_empty() {
        mna.linearized_triodes = triode_lin_infos;
        mna.stamp_linearized_triodes();
        report!(
            rep,
            "  Linearized {} triodes (M reduced by {})",
            linearized_triodes_set.len(),
            linearized_triodes_set.len() * 2
        );
    }

    // Re-stamp junction caps against the reduced-dimension MNA.
    let ds = crate::codegen::ir::CircuitIR::build_device_info_with_mna(netlist, Some(mna))
        .map_err(|e| {
            PipelineError::Mna(format!(
                "device models after the .linearize MNA rebuild: {e}"
            ))
        })?;
    if !ds.is_empty() {
        mna.stamp_device_junction_caps(&ds);
    }

    Ok(LinearizeOutcome {
        bjts_linearized: linearized_bjts_set.len(),
        triodes_linearized: linearized_triodes_set.len(),
        bias_unconverged,
    })
}

/// Auto-tune the NR iteration budget from routing and the integrator (Tier 3b).
///
/// Shared by `compile`, `simulate`, and `analyze` so every command runs the
/// same budget — simulate/analyze previously hardcoded 100 while compile
/// auto-tuned up to 50 + 5·M + 200, meaning a circuit could converge in the
/// shipped plugin but falsely "diverge" under `melange simulate`.
///
/// `user_max_iter = Some(n)` (an explicit `--max-iter`) always wins.
///
/// Rationale for the numbers (kept verbatim from the original compile-path
/// implementation): nodal full-LU is O(N³) per iteration — expensive iters
/// that converge reliably, so a flat 50; DK Schur is O(M³) — cheap iters
/// that may need more, so 50 + 5·M. A marginal-Nyquist circuit on
/// TRAPEZOIDAL has damped-NR convergence that slows sharply as ρ→1 (e.g.
/// wurli-preamp, ρ≈1.0000, needs ~186 iters/sample), hence the +200 bonus
/// when ρ > 0.999 and `trapezoidal`; a backward-Euler build converges in a
/// few iters and must NOT inherit that worst-case bound.
///
/// Whether a default build stays trapezoidal is decided later, on the
/// finished IR (the ring predicate, `codegen::ring`). Callers therefore pass
/// the budget for `trapezoidal = !backward_euler` as `max_iterations` and
/// the `trapezoidal = false` budget as `max_iterations_be_promoted`.
pub fn auto_tune_max_iter(
    user_max_iter: Option<usize>,
    kernel: &crate::dk::DkKernel,
    routing: &crate::codegen::routing::RoutingDecision,
    trapezoidal: bool,
) -> usize {
    if let Some(n) = user_max_iter {
        return n;
    }
    if kernel.m == 0 {
        return 50;
    }
    let base = if routing.route == crate::codegen::routing::SolverRoute::Nodal {
        50
    } else {
        50 + kernel.m * 5 // DK: scale with M (M=8 → 90 iters)
    };
    let stiffness_bonus = if routing.spectral_radius > 0.999 && trapezoidal {
        200
    } else if routing.spectral_radius > 0.95 {
        20
    } else {
        0
    };
    base + stiffness_bonus
}
/// Expand parasitic-BJT internal nodes: RB/RC/RE become explicit MNA nodes,
/// where their thermal noise is injected (NOISE.md). Only the nodal route calls
/// this; DK keeps them inside the device (K_eff).
///
/// Every nodal build expands. A gate at `min(diag(K)) < -100` used to decline
/// it, derived from one deck's divergence. Measured 2026-09-29 on every gated
/// corpus deck, once the nodal convergence checks covered the internal rows
/// (`4219abb`): no deck held a sample expanded, expansion removed a transformer-coupled
/// console preamp's held sample, and the shipped wurli-power-amp expanded agrees with its
/// unexpanded render to -176 dB, equals it against ngspice, and runs 2.3-2.7x
/// faster (the unexpanded device runs its own inner Newton for RB/RC/RE).
///
/// Returns `true` if expansion was actually applied.
pub fn expand_internal_nodes(
    mna: &mut crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
) -> bool {
    let device_slots =
        crate::codegen::ir::CircuitIR::build_device_info_with_mna(netlist, Some(mna))
            .unwrap_or_default();
    if mna.expandable_bjt_internal_node_count(&device_slots) == 0 {
        return false;
    }
    mna.expand_bjt_internal_nodes(&device_slots);
    true
}

/// Would the un-reduced circuit route to the nodal solver?
///
/// Forward-active reduction is skipped when the answer is yes, for two
/// reasons: the FA-reduced DC OP can converge to a parasitic equilibrium on
/// push-pull topologies, and nodal handles full-dimension BJTs natively, so
/// the reduction buys nothing there.
///
/// Routes at the **internal (oversampled) rate**, matching what codegen ships
/// via `internal_rate = sample_rate * oversampling_factor`. Routing at the base
/// host rate can miss trap/BE instability that only appears at the oversampled
/// rate the generated solver actually runs (a circuit measured rho=1.315 at
/// 192 kHz read comfortably stable through the un-oversampled 48 kHz kernel,
/// so the router never rerouted a genuinely DK-Schur-unstable circuit).
pub fn should_skip_fa_for_nodal_reroute(
    mna: &crate::mna::MnaSystem,
    sample_rate: f64,
    oversampling: usize,
    opamp_rail_mode: crate::codegen::OpampRailMode,
) -> bool {
    use crate::codegen::routing::{self, SolverRoute};
    use crate::dk::DkKernel;

    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    let routing_rate = sample_rate * oversampling.max(1) as f64;
    let kernel_result = if has_inductors {
        DkKernel::from_mna_augmented(mna, routing_rate)
    } else {
        DkKernel::from_mna(mna, routing_rate)
    };
    let (kernel, dk_failed) = match kernel_result {
        Ok(k) => (k, false),
        Err(_) => {
            log::info!("Pre-route: DK kernel failed on un-reduced MNA → skip FA");
            return true;
        }
    };
    let decision = routing::auto_route(&kernel, mna, dk_failed, opamp_rail_mode);
    log::info!(
        "Pre-route (un-reduced MNA, N={}, M={}): route={:?}, reason={}",
        kernel.n,
        kernel.m,
        decision.route,
        decision.reason
    );
    decision.route == SolverRoute::Nodal
}

/// Detect forward-active BJTs and rebuild `mna` with them reduced to 1D.
///
/// Returns the set of reduced device names — which callers must thread into
/// [`apply_grid_off_reduction`] and [`apply_linearize_reductions`], since both
/// rebuild the MNA from the netlist and would otherwise silently discard this
/// reduction.
///
/// Skipped (returning an empty set, `mna` untouched) when the circuit will end
/// up on the nodal solver — either because the caller forced it with
/// `--solver nodal` or because [`should_skip_fa_for_nodal_reroute`] says
/// auto-routing will send it there.
///
/// **This step was private to `melange-cli` until 2026-09-03**, so
/// `melange validate` and the SPICE test harness built full-2D systems for
/// circuits the shipped build reduces. Measured on the `wurli_preamp`
/// validation deck: shipped M=3, harness M=5. See `SPICE_VALIDATION.md`.
#[allow(clippy::too_many_arguments)]
pub fn apply_forward_active_reduction(
    mna: &mut crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
    fa_config: &crate::codegen::CodegenConfig,
    solver_override: &str,
    sample_rate: f64,
    oversampling: usize,
    port_stamps: &[(usize, f64)],
    rep: Reporter<'_>,
) -> Result<std::collections::HashSet<String>, PipelineError> {
    use crate::codegen::ir::CircuitIR;
    use crate::mna::MnaSystem;

    let forward_active = if solver_override == "nodal"
        || (solver_override == "auto"
            && should_skip_fa_for_nodal_reroute(
                mna,
                sample_rate,
                oversampling,
                fa_config.opamp_rail_mode,
            )) {
        std::collections::HashSet::new()
    } else {
        CircuitIR::detect_forward_active_bjts(mna, netlist, fa_config)
    };

    if !forward_active.is_empty() {
        report!(
            rep,
            "  Forward-active BJTs: {:?} (M reduces by {})",
            forward_active,
            forward_active.len()
        );
        *mna = MnaSystem::from_netlist_forward_active(netlist, &forward_active).map_err(|e| {
            PipelineError::Mna(format!(
                "Failed to rebuild MNA for forward-active BJTs: {e}"
            ))
        })?;
        stamp_ports(mna, port_stamps);
        // `build_device_info_with_mna` (not the bare netlist builder) so the
        // FA-reduced BJT dimensions are reflected, giving the correct
        // `start_idx` for junction-cap stamping.
        let device_slots =
            CircuitIR::build_device_info_with_mna(netlist, Some(&*mna)).map_err(|e| {
                PipelineError::Mna(format!(
                    "device models after the forward-active MNA rebuild: {e}"
                ))
            })?;
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    Ok(forward_active)
}

/// Apply the grid-off pentode reduction (3D → 2D NR block with `Vg2k`
/// frozen and `Ig1` dropped) and rebuild `mna` with it.
///
/// `tube_grid_fa` is the `--tube-grid-fa` mode: `on` reduces every
/// non-variable-mu pentode (warned per device — the reduction is not
/// accuracy-neutral, see [`CircuitIR::detect_grid_off_pentodes`]); `off`
/// and `auto` both keep the full 3D model. `auto` is reserved for a
/// reduction that is provably neutral; none exists today.
///
/// Route parity: skipped when the circuit will end up on the nodal solver,
/// by the same pre-route check forward-active reduction uses
/// ([`should_skip_fa_for_nodal_reroute`]). A reduction lowers M, and M is a
/// routing input, so without this check reducing could move a circuit from
/// nodal to DK Schur — which is how the twill-deluxe validation failure was
/// reached (M 10 → 8 flipped the route onto a DK defect the reduction
/// itself had nothing to do with; see `DEBUGGING.md`).
#[allow(clippy::too_many_arguments)]
pub fn apply_grid_off_reduction(
    mna: &mut crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
    fa_config: &crate::codegen::CodegenConfig,
    forward_active: &std::collections::HashSet<String>,
    tube_grid_fa: &str,
    solver_override: &str,
    sample_rate: f64,
    oversampling: usize,
    port_stamps: &[(usize, f64)],
) -> Result<std::collections::HashMap<String, f64>, PipelineError> {
    use crate::codegen::ir::CircuitIR;
    use crate::mna::MnaSystem;

    let grid_off_pentodes = if tube_grid_fa == "off" || solver_override == "nodal" {
        std::collections::HashMap::new()
    } else if tube_grid_fa != "on" {
        // `auto`: never reduces today; the call only logs, per pentode, why
        // the full 3D model is kept (no DC OP is solved on this path).
        CircuitIR::detect_grid_off_pentodes(mna, netlist, fa_config, false)
    } else if solver_override == "auto"
        && should_skip_fa_for_nodal_reroute(
            mna,
            sample_rate,
            oversampling,
            fa_config.opamp_rail_mode,
        )
    {
        std::collections::HashMap::new()
    } else {
        CircuitIR::detect_grid_off_pentodes(mna, netlist, fa_config, true)
    };

    if !grid_off_pentodes.is_empty() {
        // Compose grid-off WITH the forward-active reduction the caller
        // already applied. Rebuilding from the netlist with only the grid-off
        // map would silently discard FA (the compile summary would still print
        // the FA line while the shipped MNA had full-dimension BJT blocks) —
        // a circuit with both BJT bias stages and power pentodes needs both.
        *mna = MnaSystem::from_netlist_with_grid_off_and_fa(
            netlist,
            forward_active,
            &grid_off_pentodes,
        )
        .map_err(|e| {
            PipelineError::Mna(format!(
                "Failed to rebuild MNA for grid-off pentodes (+ FA BJTs): {e}"
            ))
        })?;
        stamp_ports(mna, port_stamps);
        let device_slots =
            CircuitIR::build_device_info_with_mna(netlist, Some(&*mna)).map_err(|e| {
                PipelineError::Mna(format!("device models after the grid-off MNA rebuild: {e}"))
            })?;
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    Ok(grid_off_pentodes)
}

/// Format the grid-off detection result for user output.
///
/// `None` when nothing was reduced — the caller decides whether to log at all
/// and on which stream (`println!` for compile/simulate progress, `eprintln!`
/// for analyze, which writes CSV to stdout).
pub fn format_grid_off_log(
    grid_off_pentodes: &std::collections::HashMap<String, f64>,
) -> Option<String> {
    if grid_off_pentodes.is_empty() {
        return None;
    }
    let mut pretty: Vec<(String, f64)> = grid_off_pentodes
        .iter()
        .map(|(k, v)| (k.clone(), *v))
        .collect();
    pretty.sort_by(|a, b| a.0.cmp(&b.0));
    let pretty_str: String = pretty
        .iter()
        .map(|(n, v)| format!("{n}(Vg2k={v:.1}V)"))
        .collect::<Vec<_>>()
        .join(", ");
    Some(format!(
        "  Grid-off pentodes: [{}] (M reduces by {})",
        pretty_str,
        grid_off_pentodes.len()
    ))
}

// ---------------------------------------------------------------------------
// Topology gate
// ---------------------------------------------------------------------------

/// A deck refused by the topology pass.
///
/// Carries every refusing finding, not just the first — a single typo usually
/// orphans both sides of the edit, and fixing one at a time is a worse
/// experience than being told both.
#[derive(Debug, Clone)]
pub struct TopologyRefusal {
    /// The findings whose severity is [`crate::topology::Severity::Refuse`].
    pub findings: Vec<crate::topology::Finding>,
}

impl std::fmt::Display for TopologyRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let n = self.findings.len();
        write!(
            f,
            "netlist topology: {n} defect{} that would silently produce the wrong circuit",
            if n == 1 { "" } else { "s" }
        )?;
        for finding in &self.findings {
            write!(f, "\n  - {}", finding.message())?;
        }
        Ok(())
    }
}

impl std::error::Error for TopologyRefusal {}

/// Run [`crate::topology::check`] and decide what its findings cost.
///
/// Warnings are reported through `rep` and the build continues; refusals stop
/// it. Every verb that builds a netlist calls this — `compile` in every output
/// format, `simulate`, `analyze` and `validate` — because a defect that is real
/// in a plugin is equally real in a render, and a deck that simulates but
/// refuses as a plugin teaches people the check is arbitrary.
///
/// `melange nodes` and `melange dc-op` call this too and are never refused by
/// it, because neither takes an `-o` and so both pass
/// [`crate::topology::Ports::unknown`] — a dangling finding cannot reach
/// [`Severity::Refuse`] without port knowledge to rest on (see
/// [`crate::topology::Finding::severity`]). The exemption is a property of what
/// those verbs know, not a list of names to keep in sync.
pub fn topology_gate(
    netlist: &crate::parser::Netlist,
    ports: &crate::topology::Ports,
    rep: Reporter<'_>,
) -> Result<(), TopologyRefusal> {
    use crate::topology::Severity;

    let findings = crate::topology::check(netlist, ports);
    let (refusals, warnings): (Vec<_>, Vec<_>) = findings
        .into_iter()
        .partition(|f| f.severity() == Severity::Refuse);

    for w in &warnings {
        report!(rep, "  warning: {}", w.message());
    }

    if refusals.is_empty() {
        Ok(())
    } else {
        Err(TopologyRefusal { findings: refusals })
    }
}

/// Run [`crate::topology::check`] and REPORT every finding, at any severity,
/// without ever refusing.
///
/// This is `melange nodes`, and the guarantee is structural: the verb a user
/// reaches for to FIND a wiring defect cannot be stopped by one, so it does not
/// call the gate at all rather than relying on the findings it happens to be
/// able to produce. That distinction started to matter with `.port`: a
/// declaration naming a node the deck does not have is
/// [`crate::topology::Severity::Refuse`] no matter what the caller knows about
/// its ports (see [`crate::topology::Finding::severity`]), so `nodes` would
/// otherwise have died on exactly the deck it was needed for.
pub fn topology_report(
    netlist: &crate::parser::Netlist,
    ports: &crate::topology::Ports,
    rep: Reporter<'_>,
) {
    for f in crate::topology::check(netlist, ports) {
        report!(rep, "  warning: {}", f.message());
    }
}
