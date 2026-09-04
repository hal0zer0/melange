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
//! | [`expand_internal_nodes_if_conditioned`] | yes | yes | **unconditional** | **unconditional** |
//! | [`auto_tune_max_iter`] | yes | yes | yes | **no** |
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
//!   [`expand_internal_nodes_if_conditioned`]; nobody hand-rolls the −100 gate.
//!
//! Forward-active and grid-off reduction were never in this table and were
//! private to `melange-cli` until 2026-09-03. They are now
//! [`apply_forward_active_reduction`] and [`apply_grid_off_reduction`].
//!
//! On `wurli-power-amp` — the shipped OpenWurli power stage — the CLI built an
//! N=20, M=14 system while `melange validate` built N=44, M=16: more than twice
//! the nodes, and a different solver sub-path. Validation reported 1319% RMS
//! error and correlation 0.0002 against ngspice, which read as a catastrophic
//! solver defect and was nothing of the kind. Driven through the CLI's pipeline
//! the same circuit validates at 0.228% RMS, correlation 1.000000.
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
}

impl std::fmt::Display for PipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DcOp(m) => write!(f, "linearize: DC operating point failed: {m}"),
            Self::Mna(m) => write!(f, "{m}"),
        }
    }
}

impl std::error::Error for PipelineError {}

#[derive(Default, Debug, Clone, Copy)]
pub struct LinearizeOutcome {
    pub bjts_linearized: usize,
    pub triodes_linearized: usize,
}

/// Apply `.linearize` directives to `mna` in-place.
///
/// Computes a DC OP on the current MNA to extract small-signal g-parameters for
/// flagged BJTs and triodes, then rebuilds the MNA via
/// `from_netlist_with_all_reductions` with those devices collapsed from
/// active-NR (2D per device) to linear stamps (0D). Re-stamps junction caps
/// against the reduced dimension and restamps the input conductance.
///
/// No-op when the netlist has no `.linearize` directives, in which case any
/// FA / grid-off rebuilds the caller already did remain in place and this
/// returns `LinearizeOutcome::default()` without modifying `mna`.
///
/// **Skipping this step changes which solver sub-path the emitter picks.** The
/// `linearized_bypass` gate in `nodal_emitter.rs` is the only thing routing
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
    input_node_idx: usize,
    input_conductance: f64,
    input_resistance: f64,
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
                report!(
                    rep,
                    "  Warning: .linearize device '{}' is not a BJT or triode (ignored)",
                    name
                );
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
            .unwrap_or_default();
    let dc_op_config = crate::dc_op::DcOpConfig {
        input_node: input_node_idx,
        input_resistance,
        ..crate::dc_op::DcOpConfig::default()
    };
    let dc_result = crate::dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);

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
                    let vbe = dc_result.v_nl.get(s).copied().unwrap_or(0.0);
                    let ic = dc_result.i_nl.get(s).copied().unwrap_or(0.0);
                    // Guard on slot.dimension: for an FA-reduced (1D) BJT,
                    // v_nl[s+1]/i_nl[s+1] belong to the NEXT device's slot.
                    // FA contract: Vbc from node voltages, Ib = Ic / BF.
                    let (vbc, ib) = if slot.dimension == 2 {
                        (
                            dc_result.v_nl.get(s + 1).copied().unwrap_or(0.0),
                            dc_result.i_nl.get(s + 1).copied().unwrap_or(0.0),
                        )
                    } else {
                        (v_at(nb) - v_at(nc), ic / bp.beta_f)
                    };

                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    let vbe_eff = sign * vbe;
                    let vbc_eff = sign * vbc;
                    let nf_vt = bp.nf * bp.vt;
                    let exp_be = (vbe_eff / nf_vt).clamp(-40.0, 40.0).exp();
                    let exp_bc = (vbc_eff / bp.vt).clamp(-40.0, 40.0).exp();

                    let gm = bp.is / nf_vt * exp_be;
                    let gmu = (bp.is / bp.vt * exp_bc + bp.is / (bp.beta_r * bp.vt) * exp_bc).abs();
                    let gpi = bp.is / (bp.beta_f * nf_vt) * exp_be;
                    let go = bp.is / (bp.beta_r * bp.vt) * exp_bc;

                    // Norton operating-point voltages in EXTERNAL node space
                    // (the linearized conductances are stamped between the
                    // external terminals, so the I0 - g·v0 constant must use
                    // external node differences — see LinearizedBjtInfo docs).
                    let vbe0 = v_at(nb) - v_at(ne);
                    let vbc0 = v_at(nb) - v_at(nc);
                    report!(rep,
                        "  Linearized {}: gm={:.4e} gpi={:.4e} gmu={:.4e} Ic_dc={:.4e} Ib_dc={:.4e}",
                        dev.name, gm, gpi, gmu, ic, ib
                    );
                    bjt_lin_infos.push(crate::mna::LinearizedBjtInfo {
                        name: dev.name.clone(),
                        nc,
                        nb,
                        ne,
                        gm,
                        gpi,
                        gmu,
                        go,
                        ic_dc: ic,
                        ib_dc: ib,
                        vbe0,
                        vbc0,
                    });
                }
            }
        }
    }

    // Extract triode small-signal params (gm, rp=1/gp) at DC bias. Skip
    // (and drop from the linearize set) when the grid is conducting or
    // Vgk is near the onset — the small-signal linearization is invalid
    // in the grid-current regime.
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

                    if ig_dc.abs() > 1e-9 {
                        report!(rep,
                            "  Warning: triode '{}' has Ig={:.4e} at DC OP (grid conducting), skipping linearization",
                            dev.name, ig_dc
                        );
                        linearized_triodes_set.remove(&dev.name.to_ascii_uppercase());
                        continue;
                    }
                    if vgk > -(tp.vgk_onset + 0.5) {
                        report!(rep,
                            "  Warning: triode '{}' has Vgk={:.2}V (near grid conduction onset {:.2}V), skipping linearization",
                            dev.name, vgk, tp.vgk_onset
                        );
                        linearized_triodes_set.remove(&dev.name.to_ascii_uppercase());
                        continue;
                    }

                    use melange_devices::NonlinearDevice;
                    let triode = melange_devices::KorenTriode {
                        mu: tp.mu,
                        ex: tp.ex,
                        kg1: tp.kg1,
                        kp: tp.kp,
                        kvb: tp.kvb,
                        ig_max: tp.ig_max,
                        vgk_onset: tp.vgk_onset,
                        lambda: tp.lambda,
                        mu_b: tp.mu_b,
                        svar: tp.svar,
                        ex_b: tp.ex_b,
                    };
                    let jac = triode.jacobian(&[vgk, vpk]);
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
                    });
                }
            }
        }
    }

    // Rebuild MNA with all three reduction classes combined (FA +
    // linearized + grid-off). This supersedes any prior FA-only or
    // grid-off-only rebuild the caller performed.
    *mna = crate::mna::MnaSystem::from_netlist_with_all_reductions(
        netlist,
        forward_active,
        &linearized_bjts_set,
        &linearized_triodes_set,
        grid_off_pentodes,
    )
    .map_err(|e| PipelineError::DcOp(format!("rebuild MNA with linearized devices: {e}")))?;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

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
        .unwrap_or_default();
    if !ds.is_empty() {
        mna.stamp_device_junction_caps(&ds);
    }

    Ok(LinearizeOutcome {
        bjts_linearized: linearized_bjts_set.len(),
        triodes_linearized: linearized_triodes_set.len(),
    })
}

/// Auto-tune the NR iteration budget from routing + trap stability (Tier 3b).
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
/// that may need more, so 50 + 5·M. A marginal-Nyquist circuit kept on TRAP
/// has damped-NR convergence that slows sharply as ρ→1 (e.g. wurli-preamp,
/// ρ≈1.0000, needs ~186 iters/sample), hence the +200 bonus when ρ > 0.999
/// and the circuit actually stays on trapezoidal; a BE-promoted circuit
/// converges in a few iters and must NOT inherit that worst-case bound.
pub fn auto_tune_max_iter(
    user_max_iter: Option<usize>,
    kernel: &crate::dk::DkKernel,
    routing: &crate::codegen::routing::RoutingDecision,
    backward_euler: bool,
    force_trap: bool,
    input_node_idx: usize,
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
    // Will this circuit actually run on trapezoidal? Replicate the codegen
    // auto-BE decision (ir.rs `auto_be`) so the iteration budget matches the
    // integrator that ships.
    let stays_trap = !backward_euler
        && (force_trap
            || !crate::codegen::stability::trap_needs_be(
                crate::codegen::stability::analyze_trap_stability_deflated(
                    &kernel.s,
                    &kernel.a_neg,
                    kernel.n,
                    &[input_node_idx],
                ),
            ));
    let stiffness_bonus = if routing.spectral_radius > 0.999 && stays_trap {
        200
    } else if routing.spectral_radius > 0.95 {
        20
    } else {
        0
    };
    base + stiffness_bonus
}
/// Expand parasitic-BJT internal nodes — but only when the Schur reduction will
/// actually be used.
///
/// Skipped when `min(diag(K)) < -100`, which routes the emitter to the full
/// N x N LU path; that path handles parasitics via `bjt_with_parasitics()`
/// directly, so expanding would inflate N for no benefit. Worse than no
/// benefit, in fact: Schur NR on an expanded parasitic system DIVERGES where
/// the same circuit converges unexpanded (measured on `wurli-power-amp` —
/// Schur + expanded parasitics blows up at the first non-zero input sample,
/// while full-LU + expanded and Schur unexpanded are both clean).
///
/// Returns `true` if expansion was applied.
pub fn expand_internal_nodes_if_conditioned(
    mna: &mut crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
    kernel: &crate::dk::DkKernel,
    rep: Reporter<'_>,
) -> bool {
    let k_diag_min = if kernel.m > 0 {
        (0..kernel.m)
            .map(|i| kernel.k[i * kernel.m + i])
            .fold(0.0_f64, f64::min)
    } else {
        0.0
    };
    if k_diag_min < -100.0 {
        report!(
            rep,
            "  Skipping internal node expansion (K ill-conditioned, using full LU)"
        );
        return false;
    }
    let device_slots =
        crate::codegen::ir::CircuitIR::build_device_info_with_mna(netlist, Some(mna))
            .unwrap_or_default();
    if !device_slots.is_empty() {
        mna.expand_bjt_internal_nodes(&device_slots);
        return true;
    }
    false
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
    let decision = routing::auto_route(&kernel, mna, dk_failed);
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
    input_node_idx: usize,
    input_conductance: f64,
    rep: Reporter<'_>,
) -> Result<std::collections::HashSet<String>, PipelineError> {
    use crate::codegen::ir::CircuitIR;
    use crate::mna::MnaSystem;

    let forward_active = if solver_override == "nodal"
        || (solver_override == "auto"
            && should_skip_fa_for_nodal_reroute(mna, sample_rate, oversampling))
    {
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
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        // `build_device_info_with_mna` (not the bare netlist builder) so the
        // FA-reduced BJT dimensions are reflected, giving the correct
        // `start_idx` for junction-cap stamping.
        let device_slots =
            CircuitIR::build_device_info_with_mna(netlist, Some(&*mna)).unwrap_or_default();
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
    input_node_idx: usize,
    input_conductance: f64,
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
        && should_skip_fa_for_nodal_reroute(mna, sample_rate, oversampling)
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
        // plexi-class circuits need both.
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
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        let device_slots =
            CircuitIR::build_device_info_with_mna(netlist, Some(&*mna)).unwrap_or_default();
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
