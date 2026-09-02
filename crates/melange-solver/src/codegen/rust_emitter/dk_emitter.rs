//! DK-path code emission methods.
//!
//! Contains the DK entry point (`emit_dk`) and all template-based emission
//! methods for constants, state, device models, switches, pots, RHS, and
//! process_sample. Also includes the shared methods used by the nodal path
//! (emit_header, emit_device_models, emit_oversampler).

use tera::Context;

use super::helpers::{
    coupled_inductor_template_data, device_param_template_data, emit_device_const,
    emit_stateful_default_fields, emit_stateful_set_sample_rate_body, emit_stateful_state_fields,
    emit_stateful_state_restore, emit_stateful_update, emit_stateful_update_fns,
    emit_thermal_tj_advance, fmt_f64, format_matrix_rows, inductor_template_data,
    named_const_entries, oversampling_info, recommended_warmup_samples, section_banner,
    self_heating_device_data, stateful_device_data, transformer_group_template_data,
    warmup_estimate_capped, SwitchCompTemplateData, SwitchTemplateData,
};
use super::RustEmitter;
use crate::codegen::ir::{CircuitIR, DeviceParams, DeviceType};
use crate::codegen::{CodegenError, NoiseMode};

/// Insert the multi-input-port Tera variables (`multi_input`, `num_inputs`,
/// `input_nodes_values`, `input_resistances_values`) into a template context.
///
/// `multi_input` gates every input-related emission divergence; when it is
/// false (the single-input case) the templates emit the historical single-input
/// code byte-for-byte. Called for every local context whose template references
/// these variables (constants, state, build_rhs, process_sample). See
/// `local-docs/multi-input-ports-plan.md`.
fn insert_multi_input_ctx(ctx: &mut Context, ir: &CircuitIR) {
    let num_inputs = ir.solver_config.num_inputs();
    ctx.insert("multi_input", &(num_inputs > 1));
    ctx.insert("num_inputs", &num_inputs);
    let input_nodes_values = ir
        .solver_config
        .input_node_indices()
        .iter()
        .map(|n| n.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    ctx.insert("input_nodes_values", &input_nodes_values);
    let input_resistances_values = ir
        .solver_config
        .input_resistance_values()
        .iter()
        .map(|r| fmt_f64(*r))
        .collect::<Vec<_>>()
        .join(", ");
    ctx.insert("input_resistances_values", &input_resistances_values);
}

/// Insert the `.inject` / `.tap` Tera variables (shared by both DK and nodal
/// template consumers: constants, state, build_rhs, process_sample).
///
/// `inject_or_tap` gates every API-shape divergence: when it is false (no
/// `.inject` and no `.tap`), the emitted `process_sample(input, state)` and
/// every related fragment are byte-identical to the pre-inject path. `.inject`
/// and multi-input are mutually exclusive (rejected at the CLI), so the two
/// context helpers never both activate their divergent branches.
pub(super) fn insert_inject_ctx(ctx: &mut Context, ir: &CircuitIR) {
    let inj = &ir.solver_config.injections;
    let taps = &ir.solver_config.taps;
    let has_inject = !inj.is_empty();
    let has_tap = !taps.is_empty();
    ctx.insert("has_inject", &has_inject);
    ctx.insert("has_tap", &has_tap);
    ctx.insert("inject_or_tap", &(has_inject || has_tap));
    ctx.insert("num_inject", &inj.len());
    ctx.insert("num_tap", &taps.len());
    ctx.insert(
        "inject_nodes_values",
        &inj.iter()
            .map(|i| i.node.to_string())
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "inject_names_values",
        &inj.iter()
            .map(|i| format!("{:?}", i.name))
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "inject_resistances_values",
        &inj.iter()
            .map(|i| fmt_f64(i.resistance))
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "inject_is_norton_values",
        &inj.iter()
            .map(|i| i.norton.to_string())
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "tap_nodes_values",
        &taps
            .iter()
            .map(|t| t.node.to_string())
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "tap_names_values",
        &taps
            .iter()
            .map(|t| format!("{:?}", t.name))
            .collect::<Vec<_>>()
            .join(", "),
    );
}

/// Emit the `.inject` / `.tap` constant block for the NODAL path (which builds
/// constants inline rather than via `constants.rs.tera`). Byte-for-byte the
/// same text the template emits under `{% if inject_or_tap %}`, so the two
/// paths stay in lockstep. Empty when there is no `.inject`/`.tap`.
pub(super) fn emit_inject_tap_constants(ir: &CircuitIR) -> String {
    let inj = &ir.solver_config.injections;
    let taps = &ir.solver_config.taps;
    if inj.is_empty() && taps.is_empty() {
        return String::new();
    }
    let inject_nodes = inj
        .iter()
        .map(|i| i.node.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let inject_names = inj
        .iter()
        .map(|i| format!("{:?}", i.name))
        .collect::<Vec<_>>()
        .join(", ");
    let inject_res = inj
        .iter()
        .map(|i| fmt_f64(i.resistance))
        .collect::<Vec<_>>()
        .join(", ");
    let inject_norton = inj
        .iter()
        .map(|i| i.norton.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let tap_nodes = taps
        .iter()
        .map(|t| t.node.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let tap_names = taps
        .iter()
        .map(|t| format!("{:?}", t.name))
        .collect::<Vec<_>>()
        .join(", ");
    format!(
        "\n// -----------------------------------------------------------------------------\n\
         // Runtime feedback injection (`.inject`) + raw inner-rate taps (`.tap`).\n\
         //\n\
         // When either directive is present `process_sample` takes per-inner-sample\n\
         // injection arrays and returns per-inner-sample raw taps (see its doc-comment).\n\
         // Both counts are always emitted so the API shape is uniform; one may be 0.\n\
         // -----------------------------------------------------------------------------\n\n\
         /// Number of runtime feedback-injection sources (`.inject`). May be 0.\n\
         pub const NUM_INJECT: usize = {num_inject};\n\n\
         /// Injection node indices (0-indexed), one per `.inject` source, in directive\n\
         /// order — the SAME order as the `injections` argument to `process_sample`.\n\
         pub const INJECT_NODES: [usize; NUM_INJECT] = [{inject_nodes}];\n\n\
         /// Injection source names (INJECT order), for the caller's index mapping.\n\
         pub const INJECT_NAMES: [&str; NUM_INJECT] = [{inject_names}];\n\n\
         /// Injection source impedances in ohms (series R for Thevenin, shunt R for\n\
         /// Norton). The conductance `1/INJECT_RESISTANCES[k]` is already baked into\n\
         /// the G matrix (stamped before the kernel), so it never enters the NR loop.\n\
         pub const INJECT_RESISTANCES: [f64; NUM_INJECT] = [{inject_res}];\n\n\
         /// Per-injection Norton flag: `true` = the runtime value is a CURRENT\n\
         /// (`rhs[node] += val`); `false` = a VOLTAGE behind R\n\
         /// (`rhs[node] += (val + val_prev) / R` for trap, `val / R` for BE).\n\
         pub const INJECT_IS_NORTON: [bool; NUM_INJECT] = [{inject_norton}];\n\n\
         /// Number of raw inner-rate taps (`.tap`). May be 0.\n\
         pub const NUM_TAP: usize = {num_tap};\n\n\
         /// Tap node indices (0-indexed), read RAW (pre-decimation, pre-DC-block,\n\
         /// pre-scale) each inner sample — see the `taps_inner` return of\n\
         /// `process_sample`. Emitted separately from OUTPUT_NODES even when a node\n\
         /// coincides (different semantics).\n\
         pub const TAP_NODES: [usize; NUM_TAP] = [{tap_nodes}];\n\n\
         /// Tap names (TAP order), for the caller's index mapping.\n\
         pub const TAP_NAMES: [&str; NUM_TAP] = [{tap_names}];\n",
        num_inject = inj.len(),
        num_tap = taps.len(),
    )
}

/// Emit the `.inject` RHS stamp loop for the NODAL path (inline RHS builders).
///
/// `rhs_var` is the target RHS array (`rhs`, `rhs_be`, `rhs_s`, …); `indent`
/// is the leading whitespace; `be` selects the backward-Euler form (no
/// trapezoidal history term). Empty when there is no `.inject`. Mirrors the
/// audio-input stamp: the source value is known at sample start, so it enters
/// the RHS as a constant and the NR loop never sees it.
///
/// Both source kinds carry the trapezoidal `+ prev` history under trap and drop
/// it under BE, matching melange's proper-trap convention (`(V+V_prev)*G`, NOT
/// `2*V*G`). A Norton current source `I` is the exact equivalent of a Thevenin
/// `V=I/G_sh` behind `R=1/G_sh`, so its RHS is `(I+I_prev)` under trap — the
/// SAME `(val+val_prev)` as Thevenin, just without the `/R` (the value is
/// already a current). Empirically confirmed: instantaneous `val` produces
/// exactly half the correct node voltage (Norton oracle, `inject_oracle.rs`).
pub(super) fn emit_inject_rhs_stamp(
    ir: &CircuitIR,
    rhs_var: &str,
    indent: &str,
    be: bool,
) -> String {
    if ir.solver_config.injections.is_empty() {
        return String::new();
    }
    let (norton, thevenin) = if be {
        (
            format!("{rhs_var}[INJECT_NODES[k]] += injections[k];"),
            format!("{rhs_var}[INJECT_NODES[k]] += injections[k] / INJECT_RESISTANCES[k];"),
        )
    } else {
        (
            format!("{rhs_var}[INJECT_NODES[k]] += injections[k] + state.injections_prev[k];"),
            format!(
                "{rhs_var}[INJECT_NODES[k]] += (injections[k] + state.injections_prev[k]) / INJECT_RESISTANCES[k];"
            ),
        )
    };
    format!(
        "{indent}// Runtime feedback injections (.inject): source value known at sample\n\
         {indent}// start enters the RHS as a constant (NR never sees it). Trap carries\n\
         {indent}// the (val + val_prev) history (Norton current or Thevenin V/R alike).\n\
         {indent}for k in 0..NUM_INJECT {{\n\
         {indent}    if INJECT_IS_NORTON[k] {{\n\
         {indent}        {norton}\n\
         {indent}    }} else {{\n\
         {indent}        {thevenin}\n\
         {indent}    }}\n\
         {indent}}}\n"
    )
}

/// Emit an INTERNAL zero-input `process_sample(...)` call line (warmup /
/// DC-OP fast-forward loops), matching the public signature: multi-input
/// takes `[0.0; NUM_INPUTS]`; `.inject`/`.tap` decks take the extra
/// all-zero per-inner-sample injection array and return a tuple (discarded in
/// statement position). `indent` is the emitted-code leading whitespace.
pub(super) fn emit_warmup_call(ir: &CircuitIR, indent: &str, let_bind: bool) -> String {
    // `let_bind` reproduces the exact pre-inject statement form at each call
    // site (the DC-OP settle loop used `let _ = …`; the nodal warmup loops a
    // bare call) so no-inject decks stay byte-identical.
    let lhs = if let_bind { "let _ = " } else { "" };
    if ir.solver_config.has_inject_or_tap() {
        format!(
            "{indent}{lhs}process_sample(0.0, &[[0.0; NUM_INJECT]; OVERSAMPLING_FACTOR], self);\n"
        )
    } else if ir.solver_config.num_inputs() > 1 {
        format!("{indent}{lhs}process_sample([0.0; NUM_INPUTS], self);\n")
    } else {
        format!("{indent}{lhs}process_sample(0.0, self);\n")
    }
}

/// Emit the `.inject` RHS stamp for a nodal micro-SUB-STEP (ActiveSetBe /
/// stiff-sample recovery). The sub-step matrices are TRAPEZOIDAL (`alpha =
/// 2·rate·subdiv`), so BOTH source kinds interpolate their injection across the
/// sub-step ramp exactly like the audio input (`inp_s`/`inp_prev_s`) and carry
/// the trap `(inj_s + inj_prev_s)` history — Norton without the `/R` (its value
/// is already a current), Thevenin with it. `ndiv` is the sub-division count
/// variable in scope (`N_SUB` on the Schur path, `subdiv` on the full-LU path);
/// `step` is the loop index in scope. Empty when there is no `.inject`.
pub(super) fn emit_inject_substep_stamp(
    ir: &CircuitIR,
    rhs_var: &str,
    indent: &str,
    ndiv: &str,
) -> String {
    if ir.solver_config.injections.is_empty() {
        return String::new();
    }
    format!(
        "{indent}for k in 0..NUM_INJECT {{\n\
         {indent}    let inj_step_k = (injections[k] - state.injections_prev[k]) / {ndiv} as f64;\n\
         {indent}    let inj_s = state.injections_prev[k] + inj_step_k * (step + 1) as f64;\n\
         {indent}    let inj_prev_s = state.injections_prev[k] + inj_step_k * step as f64;\n\
         {indent}    if INJECT_IS_NORTON[k] {{\n\
         {indent}        {rhs_var}[INJECT_NODES[k]] += inj_s + inj_prev_s;\n\
         {indent}    }} else {{\n\
         {indent}        {rhs_var}[INJECT_NODES[k]] += (inj_s + inj_prev_s) / INJECT_RESISTANCES[k];\n\
         {indent}    }}\n\
         {indent}}}\n"
    )
}

/// Resolved forward-active BJT reduction, read off the device slots (which
/// reflect the outcome of `detect_forward_active_bjts` — i.e. `--bjt-fa` AFTER
/// resolution, not the requested flag). Returns `(forward_active, full_2d)`.
/// `--bjt-fa force` reduces GP/ISE BJTs that `auto`/`off` leave full-2D, so the
/// counts differ and the provenance line differentiates the builds (the
/// previously-identical `Build:` line was the FOLLOWUPS gap).
fn bjt_fa_resolution(ir: &CircuitIR) -> (usize, usize) {
    let mut fa = 0usize;
    let mut full = 0usize;
    for slot in &ir.device_slots {
        match slot.device_type {
            DeviceType::BjtForwardActive => fa += 1,
            DeviceType::Bjt => full += 1,
            _ => {}
        }
    }
    (fa, full)
}

/// The `MAX_ITER` value the emitted code ACTUALLY uses, so provenance never
/// disagrees with the const it describes.
///
/// The nodal path (full-LU + Schur) floors the auto-tuned budget at 100 — a
/// ceiling, not a target: a sample converging in 8 iterations still exits at 8,
/// so converging samples pay nothing, while the Armijo line search gets enough
/// headroom to crawl a full-scale transient's operating-point move through the
/// saturation knee within one sample. The DK path is deliberately NOT floored.
/// Mirror of the `MAX_ITER` const emission in `nodal_emitter.rs` — keep the two
/// in lockstep.
pub(super) fn effective_max_iter(ir: &CircuitIR) -> usize {
    match ir.solver_mode {
        crate::codegen::ir::SolverMode::Nodal => ir.solver_config.max_iterations.max(100),
        crate::codegen::ir::SolverMode::Dk => ir.solver_config.max_iterations,
    }
}

/// Full RESOLVED DSP-affecting flag set for the human-readable `Build:` line.
///
/// Every entry reflects the value AFTER netlist directives + auto-promotion.
/// Flags whose value cannot change a circuit's DSP are omitted for that circuit
/// (e.g. `opamp-rail` only when clamped op-amps exist, `bjt-fa` only with BJTs)
/// so the line stays signal, not boilerplate.
fn resolved_build_flags(ir: &CircuitIR) -> String {
    let mut build = format!(
        "integration={}, max_iter={}, oversampling={}x",
        ir.integrator_selection.label(),
        effective_max_iter(ir),
        ir.solver_config.oversampling_factor
    );
    // DC blocking is a fourth (5 Hz) output highpass that is otherwise invisible
    // in the header — always disclose it.
    build.push_str(&format!(
        ", dc-block={}",
        if ir.dc_block { "on" } else { "off" }
    ));
    // Noise mode (off/thermal/shot/full) — resolved from --noise + per-device KF.
    build.push_str(&format!(", noise={}", ir.noise.mode.as_str()));
    // Op-amp rail saturation strategy — only meaningful when a clamped op-amp is
    // present (ir.opamps is populated only for finite-VSAT op-amps).
    if !ir.opamps.is_empty() {
        build.push_str(&format!(
            ", opamp-rail={}",
            ir.solver_config.opamp_rail_mode.as_str()
        ));
    }
    // Forward-active BJT reduction, resolved (see bjt_fa_resolution).
    let (fa, full) = bjt_fa_resolution(ir);
    if fa + full > 0 {
        build.push_str(&format!(", bjt-fa={fa}fa/{full}full"));
    }
    if ir.solver_config.breakpoint_be {
        build.push_str(", breakpoint-be");
    }
    if ir.solver_config.runtime_be_latch {
        build.push_str(", runtime-be-latch");
    }
    build
}

/// One-line machine-readable JSON (embedded in a comment) mirroring the
/// resolved `Build:` flags plus build identity. Hand-formatted — melange-solver
/// has no non-dev `serde_json`, and every value here is controlled (semver,
/// hex/`unknown`, enum tokens, numbers, bools), so no user text is interpolated
/// and no escaping is required.
fn provenance_json(ir: &CircuitIR, version: &str, commit: &str) -> String {
    let scheme = if ir.integrator_selection.is_backward_euler() {
        "backward-euler"
    } else {
        "trapezoidal"
    };
    let mut s = String::from("{");
    s.push_str(&format!("\"melange\":\"{version}\","));
    s.push_str(&format!("\"commit\":\"{commit}\","));
    s.push_str(&format!("\"integration\":\"{scheme}\","));
    s.push_str(&format!(
        "\"integration_source\":\"{}\",",
        ir.integrator_selection.integration_source()
    ));
    s.push_str(&format!(
        "\"backward_euler\":{},",
        ir.integrator_selection.is_backward_euler()
    ));
    s.push_str(&format!("\"max_iter\":{},", effective_max_iter(ir)));
    s.push_str(&format!(
        "\"oversampling\":{},",
        ir.solver_config.oversampling_factor
    ));
    s.push_str(&format!("\"dc_block\":{},", ir.dc_block));
    s.push_str(&format!("\"noise\":\"{}\"", ir.noise.mode.as_str()));
    if !ir.opamps.is_empty() {
        s.push_str(&format!(
            ",\"opamp_rail\":\"{}\"",
            ir.solver_config.opamp_rail_mode.as_str()
        ));
    }
    let (fa, full) = bjt_fa_resolution(ir);
    if fa + full > 0 {
        s.push_str(&format!(",\"bjt_fa_reduced\":{fa},\"bjt_full\":{full}"));
    }
    if ir.solver_config.breakpoint_be {
        s.push_str(",\"breakpoint_be\":true");
    }
    if ir.solver_config.runtime_be_latch {
        s.push_str(",\"runtime_be_latch\":true");
    }
    s.push('}');
    s
}

impl RustEmitter {
    /// Emit DK-method generated code (original path).
    pub(super) fn emit_dk(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = String::new();
        let noise = self.build_noise_emission(ir);

        code.push_str(&self.emit_header(ir)?);
        code.push_str(&self.emit_constants(ir)?);
        code.push_str(&self.emit_pot_constants(ir));
        if noise.enabled {
            code.push_str(&noise.top_level);
        }
        code.push_str(&self.emit_state(ir, &noise)?);
        code.push_str(&Self::emit_transformer_group_helpers(ir));
        code.push_str(&self.emit_device_models(ir)?);
        // (Stateful-device update() hooks are emitted inside emit_device_models,
        //  which both the DK and nodal generate paths call — single source.)
        // SM pot helpers (sm_scale_N) removed — per-block rebuild replaces SM
        code.push_str(&self.emit_build_rhs(ir, &noise)?);
        code.push_str(&self.emit_mat_vec_mul_s(ir)?);
        code.push_str(&self.emit_extract_voltages(ir)?);
        self.generate_solve_nonlinear(&mut code, ir)?;
        code.push_str(&self.emit_final_voltages(ir)?);
        code.push_str(&self.emit_update_history()?);
        code.push_str(&self.emit_process_sample(ir, &noise)?);

        if ir.solver_config.oversampling_factor > 1 {
            code.push_str(&Self::emit_oversampler(ir));
        } else if ir.solver_config.has_inject_or_tap() {
            // No oversampling, but `.inject`/`.tap` still emit a private
            // process_sample_inner; wrap it in the array-API public entry.
            code.push_str(&Self::emit_inject_wrapper_1x(ir));
        }

        Ok(code)
    }
}

// ============================================================================
// Template-based emission methods
// ============================================================================

impl RustEmitter {
    pub(super) fn emit_header(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        // Sanitize title: replace newlines and control characters with spaces
        // to prevent template injection through a crafted SPICE netlist title line.
        let sanitized_title: String = ir
            .metadata
            .title
            .chars()
            .map(|c| if c.is_control() { ' ' } else { c })
            .collect();
        ctx.insert("title", &sanitized_title);

        // Build-provenance identity (oomox thread 214). Version is the melange
        // crate version at *melange* build time; commit is captured by build.rs
        // (falls back to "unknown" for a packaged crate / no git). Local builds
        // between tags are normal, so both are recorded.
        let melange_version = env!("CARGO_PKG_VERSION");
        let melange_commit = option_env!("MELANGE_GIT_COMMIT").unwrap_or("unknown");
        ctx.insert("melange_version", melange_version);
        ctx.insert("melange_commit", melange_commit);

        // Provenance line: the FULL RESOLVED flag set — every flag that changes
        // emitted DSP, AFTER netlist-directive application + auto-promotion (not
        // the user-requested subset). The `(auto-promoted)` style is carried by
        // `IntegratorSelection::label()`.
        let build = resolved_build_flags(ir);
        ctx.insert("build", &build);

        // Machine-readable one-line JSON so a consumer can assert the build
        // contract at compile time (replaces oomox's hand-written
        // `oversampling_contract_is_2x` / `dc_block_contract_is_disabled` guards).
        let provenance_json = provenance_json(ir, melange_version, melange_commit);
        ctx.insert("provenance_json", &provenance_json);

        self.render("header", &ctx)
    }

    fn emit_constants(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("n", &n);
        ctx.insert("m", &m);
        // n_nodes: original circuit node count (before augmented VS/VCVS variables).
        // Used to zero out augmented rows in A_neg during rebuild_matrices.
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        ctx.insert("n_nodes", &n_nodes);
        let has_augmented = n_nodes < n;
        ctx.insert("has_augmented", &has_augmented);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("n_aug", &ir.topology.n_aug);

        // When augmented_inductors is true, companion model constants (IND_*_G_EQ,
        // CI_*_G_SELF/MUTUAL, XFMR_*_Y) are not needed. The G/C matrices already
        // contain inductor stamps and A_neg handles history.
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, true));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            ctx.insert("transformer_groups", &transformer_group_template_data(ir));
        }
        ctx.insert(
            "sample_rate",
            &format!("{:.1}", ir.solver_config.sample_rate),
        );
        ctx.insert("oversampling_factor", &ir.solver_config.oversampling_factor);
        if ir.solver_config.oversampling_factor > 1 {
            let internal_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            ctx.insert("internal_sample_rate", &format!("{:.1}", internal_rate));
        }
        ctx.insert("alpha", &fmt_f64(ir.solver_config.alpha));
        ctx.insert("input_node", &ir.solver_config.input_node);
        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);
        let output_nodes_values = ir
            .solver_config
            .output_nodes
            .iter()
            .map(|n| n.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        ctx.insert("output_nodes_values", &output_nodes_values);
        let output_scales_values = ir
            .solver_config
            .output_scales
            .iter()
            .map(|s| fmt_f64(*s))
            .collect::<Vec<_>>()
            .join(", ");
        ctx.insert("output_scales_values", &output_scales_values);
        ctx.insert(
            "input_resistance",
            &fmt_f64(ir.solver_config.input_resistance),
        );
        // Multi-input ports (M=0 only): see `insert_multi_input_ctx`.
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        // Named topology constants (Oomox P2 + P3). Always inserted so the
        // template can unconditionally reference `named_nodes`, `named_vsources`,
        // `named_pots` — empty lists produce no emission.
        ctx.insert(
            "named_nodes",
            &named_const_entries(&ir.named_constants.nodes),
        );
        ctx.insert(
            "named_vsources",
            &named_const_entries(&ir.named_constants.vsources),
        );
        ctx.insert("named_pots", &named_const_entries(&ir.named_constants.pots));

        // NODE_NAMES parallel array + dc_op_by_name lookup (openfarf thread 218).
        // `node_names_values` is the `[&str; N]` body; `has_dc_op` gates the
        // lookup fn (needs the DC_OP const, emitted in state.rs.tera).
        ctx.insert(
            "node_names_values",
            &super::helpers::node_names_array_body(ir),
        );
        ctx.insert("dc_op_by_name_fn", super::helpers::DC_OP_BY_NAME_FN);
        ctx.insert("has_dc_op", &ir.has_dc_op);

        // Runtime voltage sources (.runtime directive). Always insert the list
        // (possibly empty) so state.rs.tera and build_rhs.rs.tera can use
        // `runtime_sources | length > 0` guards unconditionally.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // WARMUP_SAMPLES_RECOMMENDED (Oomox P5): 5τ_max at the internal sample
        // rate, rounded up, minimum 1. Plugins driving per-instance parameter
        // jitter (e.g. SeriesOfTubes) use this to size the silent warmup loop.
        ctx.insert(
            "warmup_samples_recommended",
            &recommended_warmup_samples(ir),
        );
        // Companion flag: true when the value above hit the sanity cap (upper
        // bound, not a measured settle). Never launder a capped estimate into a
        // plausible number silently (oomox 2026-08-15).
        ctx.insert("warmup_estimate_capped", &warmup_estimate_capped(ir));

        // G and C matrices (sample-rate independent)
        ctx.insert("g_rows", &format_matrix_rows(n, n, |i, j| ir.g(i, j)));
        ctx.insert("c_rows", &format_matrix_rows(n, n, |i, j| ir.c(i, j)));

        ctx.insert("s_rows", &format_matrix_rows(n, n, |i, j| ir.s(i, j)));
        ctx.insert(
            "a_neg_rows",
            &format_matrix_rows(n, n, |i, j| ir.a_neg(i, j)),
        );

        if ir.has_dc_sources {
            let rhs_const_values = (0..n)
                .map(|i| fmt_f64(ir.matrices.rhs_const[i]))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("rhs_const_values", &rhs_const_values);
        }

        // K_eff: pre-subtract parasitic-BJT R_p so the controlling-voltage map
        // v_d = p + state.k * i feeds bjt_evaluate the *internal* junction
        // voltage. Replaces the inner-NR cost of bjt_with_parasitics. R_p is
        // exact (linear in i), so K_eff has the same fixed point.
        ctx.insert(
            "k_rows",
            &format_matrix_rows(m, m, |i, j| ir.k(i, j) - parasitic_r_p_dk(ir, i, j)),
        );
        ctx.insert("n_v_rows", &format_matrix_rows(m, n, |i, j| ir.n_v(i, j)));
        // N_i in operator shape: N_I[node][device] = n_i[node][device].
        // Normalized to N x M to match the nodal emitter — one layout for one
        // public symbol (see the `N_I` doc comment in constants.rs.tera).
        ctx.insert("n_i_rows", &format_matrix_rows(n, m, |i, j| ir.n_i(i, j)));

        // S*N_i product: precomputed for final voltage correction
        // S_NI[node][device] = sum_k S[node][k] * N_i[k][device]
        let s_ni_rows: Vec<String> = (0..n)
            .map(|i| {
                (0..m)
                    .map(|j| {
                        let mut val = 0.0;
                        for k in 0..n {
                            val += ir.s(i, k) * ir.n_i(k, j);
                        }
                        fmt_f64(val)
                    })
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .collect();
        ctx.insert("s_ni_rows", &s_ni_rows);

        // Backward Euler fallback constants (for BE fallback in DK NR solver)
        let has_be_fallback = !ir.matrices.s_be.is_empty() && m > 0;
        ctx.insert("has_be_fallback", &has_be_fallback);
        if has_be_fallback {
            ctx.insert("s_be_rows", &format_matrix_rows(n, n, |i, j| ir.s_be(i, j)));
            // BE fallback K also gets the K_eff treatment — same parasitic
            // BJT absorption applies because the BE NR uses the same K-mediated
            // controlling-voltage map.
            ctx.insert(
                "k_be_rows",
                &format_matrix_rows(m, m, |i, j| ir.k_be(i, j) - parasitic_r_p_dk(ir, i, j)),
            );
            ctx.insert(
                "a_neg_be_rows",
                &format_matrix_rows(n, n, |i, j| ir.a_neg_be(i, j)),
            );

            // S_NI_be = S_be * N_i (N x M)
            let s_ni_be_rows: Vec<String> = (0..n)
                .map(|i| {
                    (0..m)
                        .map(|j| {
                            let mut val = 0.0;
                            for k in 0..n {
                                val += ir.s_be(i, k) * ir.n_i(k, j);
                            }
                            fmt_f64(val)
                        })
                        .collect::<Vec<_>>()
                        .join(", ")
                })
                .collect();
            ctx.insert("s_ni_be_rows", &s_ni_be_rows);

            // RHS_CONST_BE (backward Euler: DC sources x1).
            //
            // Guard on `has_dc_sources` ONLY — must stay symmetric with the
            // template pair: constants.rs.tera emits the constant under
            // `{% if has_dc_sources %}` (inside `has_be_fallback`) and
            // process_sample.rs.tera references it under the same condition.
            // A short/empty `rhs_const_be` vec is padded with zeros below so
            // the constant is always well-formed when the guard fires.
            if ir.has_dc_sources {
                let rhs_const_be_values = (0..n)
                    .map(|i| {
                        if i < ir.matrices.rhs_const_be.len() {
                            fmt_f64(ir.matrices.rhs_const_be[i])
                        } else {
                            fmt_f64(0.0)
                        }
                    })
                    .collect::<Vec<_>>()
                    .join(", ");
                ctx.insert("rhs_const_be_values", &rhs_const_be_values);
            }
        }

        // Switch constants
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        if num_switches > 0 {
            let switch_data: Vec<SwitchTemplateData> = ir
                .switches
                .iter()
                .map(|sw| {
                    let components: Vec<SwitchCompTemplateData> = sw
                        .components
                        .iter()
                        .map(|comp| SwitchCompTemplateData {
                            node_p: comp.node_p,
                            node_q: comp.node_q,
                            nominal: fmt_f64(comp.nominal_value),
                            comp_type: comp.component_type,
                            inductor_index: comp.inductor_index.map(|i| i as i64).unwrap_or(-1),
                        })
                        .collect();
                    let position_rows: Vec<String> = sw
                        .positions
                        .iter()
                        .map(|pos| {
                            pos.iter()
                                .map(|v| fmt_f64(*v))
                                .collect::<Vec<_>>()
                                .join(", ")
                        })
                        .collect();
                    SwitchTemplateData {
                        index: sw.index,
                        label: sw.label.clone(),
                        num_positions: sw.num_positions,
                        num_components: sw.components.len(),
                        components,
                        position_rows,
                    }
                })
                .collect();
            ctx.insert("switches", &switch_data);
        }

        // DC block coefficient: R = 1 - 2*pi*5/sr
        let internal_rate =
            ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
        let dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;
        ctx.insert("dc_block_r", &format!("{:.17e}", dc_block_r));
        ctx.insert("dc_block", &ir.dc_block);
        ctx.insert("dc_op_converged", &ir.dc_op_converged);

        // Op-amp slew-rate constants. One entry per op-amp whose .model
        // card set a finite SR (V/μs, converted to V/s in MNA). Emitted by
        // constants.rs.tera as `const OA{idx}_SR: f64 = …;`. Consumed by
        // process_sample.rs.tera in the slew-limit block.
        let opamp_slew: Vec<std::collections::HashMap<&str, String>> = ir
            .opamps
            .iter()
            .enumerate()
            .filter(|(_, oa)| oa.sr.is_finite())
            .map(|(idx, oa)| {
                let mut m = std::collections::HashMap::new();
                m.insert("idx", idx.to_string());
                m.insert("out_idx", oa.n_out_idx.to_string());
                m.insert("sr", format!("{:.17e}", oa.sr));
                m
            })
            .collect();
        if !opamp_slew.is_empty() {
            ctx.insert("opamp_slew", &opamp_slew);
        }

        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR, noise: &NoiseEmission) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        // Noise fragments (empty strings when noise is off → template blocks become no-ops)
        ctx.insert("noise_enabled_emit", &noise.enabled);
        ctx.insert("noise_state_fields", &noise.state_fields);
        ctx.insert("noise_default_stmts", &noise.default_stmts);
        ctx.insert("noise_default_fields", &noise.default_fields);
        ctx.insert("noise_reset_body", &noise.reset_body);
        ctx.insert("noise_set_sample_rate_body", &noise.set_sample_rate_body);
        ctx.insert("noise_methods", &noise.methods);
        // Stateful-device opaque state block (Phase 0c). Empty strings when no
        // device is stateful → template blocks are no-ops and the deck is
        // byte-identical. NaN-recovery + after-solve update live in
        // process_sample.rs.tera (emit_process_sample), not here.
        let stateful_devs = stateful_device_data(ir);
        let has_stateful = !stateful_devs.is_empty();
        ctx.insert("has_stateful", &has_stateful);
        ctx.insert(
            "stateful_state_fields",
            &emit_stateful_state_fields(&stateful_devs),
        );
        ctx.insert(
            "stateful_default_fields",
            &emit_stateful_default_fields(&stateful_devs),
        );
        ctx.insert(
            "stateful_reset_body",
            &emit_stateful_state_restore(&stateful_devs, "self."),
        );
        ctx.insert(
            "stateful_set_sample_rate_body",
            &emit_stateful_set_sample_rate_body(ir, &stateful_devs),
        );
        ctx.insert("has_dc_op", &ir.has_dc_op);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("n_aug", &ir.topology.n_aug);
        ctx.insert("n_nodes", &ir.topology.n_nodes);
        // When augmented_inductors is true, companion model state (ind_i_prev, ci_i_hist,
        // xfmr_y, etc.) is not needed. A_neg handles history through augmented G/C.
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);

        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            ctx.insert("os_state_size", &os_info.state_size);
            ctx.insert("oversampling_4x", &(os_factor == 4));
            if os_factor == 4 {
                ctx.insert("os_state_size_outer", &os_info.state_size_outer);
            }
        } else {
            ctx.insert("oversampling_4x", &false);
        }

        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, true));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            ctx.insert("transformer_groups", &transformer_group_template_data(ir));

            // Generate set_sample_rate recomputation lines procedurally
            let mut xfmr_ssr_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                xfmr_ssr_lines.push_str(
                    "        {\n\
                     \x20           let half_t = t / 2.0;\n",
                );
                // Build L matrix
                for i in 0..w {
                    for j in 0..w {
                        xfmr_ssr_lines.push_str(&format!(
                            "            let l_{i}_{j} = XFMR_{gi}_COUPLING[{}] * (XFMR_{gi}_INDUCTANCES[{i}] * XFMR_{gi}_INDUCTANCES[{j}]).sqrt();\n",
                            i * w + j,
                        ));
                    }
                }
                // Call inversion helper
                xfmr_ssr_lines.push_str(&format!("            let y = invert_xfmr_{gi}(["));
                for i in 0..w {
                    if i > 0 {
                        xfmr_ssr_lines.push_str(", ");
                    }
                    xfmr_ssr_lines.push('[');
                    for j in 0..w {
                        if j > 0 {
                            xfmr_ssr_lines.push_str(", ");
                        }
                        xfmr_ssr_lines.push_str(&format!("l_{i}_{j}"));
                    }
                    xfmr_ssr_lines.push(']');
                }
                xfmr_ssr_lines.push_str("]);\n");
                // Store Y and stamp
                for i in 0..w {
                    for j in 0..w {
                        xfmr_ssr_lines.push_str(&format!(
                            "            self.xfmr_{gi}_y[{}] = half_t * y[{i}][{j}];\n",
                            i * w + j,
                        ));
                    }
                }
                // Stamp self-conductances
                for i in 0..w {
                    let flat = i * w + i;
                    xfmr_ssr_lines.push_str(&format!(
                        "            stamp_conductance(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], self.xfmr_{gi}_y[{flat}]);\n\
                         \x20           stamp_conductance(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], -self.xfmr_{gi}_y[{flat}]);\n"
                    ));
                }
                // Stamp mutual conductances
                for i in 0..w {
                    for j in 0..w {
                        if i == j {
                            continue;
                        }
                        let flat = i * w + j;
                        xfmr_ssr_lines.push_str(&format!(
                            "            stamp_mutual(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], self.xfmr_{gi}_y[{flat}]);\n\
                             \x20           stamp_mutual(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], -self.xfmr_{gi}_y[{flat}]);\n"
                        ));
                    }
                }
                xfmr_ssr_lines.push_str("        }\n");
            }
            // Reset transformer group transient state
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                xfmr_ssr_lines.push_str(&format!(
                    "        self.xfmr_{gi}_i_prev = [0.0; {}];\n\
                     \x20       self.xfmr_{gi}_v_prev = [0.0; {}];\n\
                     \x20       self.xfmr_{gi}_i_hist = [0.0; {}];\n",
                    g.num_windings, g.num_windings, g.num_windings,
                ));
            }
            ctx.insert("xfmr_set_sample_rate_lines", &xfmr_ssr_lines);
        }

        let pot_defaults: Vec<String> =
            ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
        ctx.insert("pot_defaults", &pot_defaults);

        if ir.has_dc_op {
            let dc_op_values = ir
                .dc_operating_point
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_op_values", &dc_op_values);
        }

        // IC=-bearing capacitors: initial-state seed for `v_prev` (see
        // `docs/aidocs/DC_OP.md` "IC= initial condition"). Independent of
        // `has_dc_op` — a circuit with no DC sources but an IC= cap still
        // needs this constant.
        let has_cap_ic = ir.v_prev_ic_seed.is_some();
        ctx.insert("has_cap_ic", &has_cap_ic);
        if let Some(v_prev_ic) = &ir.v_prev_ic_seed {
            let v_prev_ic_values = v_prev_ic
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("v_prev_ic_values", &v_prev_ic_values);
        }

        // DC nonlinear currents: emit DC_NL_I constant if M > 0 and any i_nl is nonzero
        let has_dc_nl = ir.topology.m > 0
            && !ir.dc_nl_currents.is_empty()
            && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
        ctx.insert("has_dc_nl", &has_dc_nl);
        if has_dc_nl {
            let dc_nl_i_values = ir
                .dc_nl_currents
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_nl_i_values", &dc_nl_i_values);
        }

        // DC nonlinear currents at the IC-seeded operating point — paired
        // ONLY with V_PREV_IC_SEED (never with the plain DC_OP/DC_NL_I pair
        // used by the reset fallback). See the pairing comment on
        // `dc_nl_currents_ic_seed` in `codegen/ir/mod.rs`.
        let has_dc_nl_ic_seed = ir.topology.m > 0 && ir.dc_nl_currents_ic_seed.is_some();
        ctx.insert("has_dc_nl_ic_seed", &has_dc_nl_ic_seed);
        if has_dc_nl_ic_seed {
            let dc_nl_i_ic_seed_values = ir
                .dc_nl_currents_ic_seed
                .as_ref()
                .unwrap()
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_nl_i_ic_seed_values", &dc_nl_i_ic_seed_values);
        }

        // Switch data
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        // Always provide switch_indices (empty when no switches) so template can iterate safely
        let switch_indices: Vec<usize> = (0..num_switches).collect();
        ctx.insert("switch_indices", &switch_indices);
        // Generate pot/switch methods procedurally (rebuild_matrices, set_pot_N, set_switch_N)
        if num_switches > 0 || num_pots > 0 {
            let switch_methods = self.emit_switch_methods(ir, noise);
            ctx.insert("switch_methods", &switch_methods);
        }

        // Device parameter state fields (runtime-adjustable)
        let device_params = device_param_template_data(ir);
        let num_device_params = device_params.len();
        ctx.insert("num_device_params", &num_device_params);
        if num_device_params > 0 {
            ctx.insert("device_params", &device_params);
        }

        // BJT self-heating thermal state
        let thermal_devices = self_heating_device_data(ir);
        let num_thermal_devices = thermal_devices.len();
        ctx.insert("num_thermal_devices", &num_thermal_devices);
        if num_thermal_devices > 0 {
            ctx.insert("thermal_devices", &thermal_devices);
        }

        ctx.insert("dc_block", &ir.dc_block);

        // `current_sample_rate` is read by rebuild_matrices (pots/switches),
        // by the op-amp slew-rate limiter's per-sample dt, AND by the device
        // self-heating thermal update's dt (all in process_sample.rs.tera /
        // emit_thermal_tj_advance). Emit the field whenever any consumer
        // exists — mirrors the nodal-emitter fix (7e32bf8): a DK circuit
        // with a finite-SR op-amp (or a thermal device) and no pots would
        // otherwise fail rustc.
        let needs_current_sr = num_pots > 0
            || num_switches > 0
            || ir.opamps.iter().any(|oa| oa.sr.is_finite())
            || num_thermal_devices > 0
            || has_stateful;
        ctx.insert("needs_current_sr", &needs_current_sr);

        // DK rail-mode consumption: the DK path implements only the Hard
        // post-NR clamp. ActiveSet/ActiveSetBe degrade to Hard (+ the BE
        // fallback that DK already carries), so surface a one-shot runtime
        // warning at construction of the generated state. `None` suppresses
        // the clamp entirely (handled in emit_process_sample); Hard and
        // BoyleDiodes (guarded off DK upstream) emit no warning.
        let has_clamped_opamp = ir
            .opamps
            .iter()
            .any(|oa| oa.vclamp_lo.is_finite() || oa.vclamp_hi.is_finite());
        if has_clamped_opamp
            && matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::ActiveSet
                    | crate::codegen::OpampRailMode::ActiveSetBe
            )
        {
            ctx.insert(
                "opamp_rail_degrade_warn",
                &format!(
                    "melange: warning: opamp rail mode '{}' degrades to Hard+BE-fallback on the DK path",
                    ir.solver_config.opamp_rail_mode.as_str()
                ),
            );
        }

        // Backward Euler fallback state fields (for BE fallback in DK NR solver)
        let has_be_fallback = !ir.matrices.s_be.is_empty() && ir.topology.m > 0;
        ctx.insert("has_be_fallback", &has_be_fallback);
        ctx.insert("backward_euler", &ir.solver_config.backward_euler);

        // Runtime voltage sources (.runtime directive). Same list is used by
        // state.rs.tera for field + default + reset emission; build_rhs.rs.tera
        // emits the per-sample `rhs[row] += state.<field>` stamps.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // Named nodes for the dc_op_dump() pretty printer (Oomox P4).
        ctx.insert(
            "named_nodes",
            &named_const_entries(&ir.named_constants.nodes),
        );

        // K_eff twin for the no-pot set_sample_rate rebuild (template body).
        // The baked K_DEFAULT/K_BE_DEFAULT and the pot-path rebuild_matrices
        // both absorb parasitic-BJT RB/RC/RE into K; a genuine rate change
        // through the template body must apply the same absorption or the
        // parasitics are electrically deleted at any non-codegen host rate.
        // The fragments render byte-neutral (empty string, no extra lines)
        // when no slot qualifies — which is every shipped circuit today.
        let k_eff_stmts = k_eff_adjust_stmts(ir, "k", "        ");
        let k_eff_fragment = if k_eff_stmts.is_empty() {
            String::new()
        } else {
            format!(
                "\n\n        // K_eff: absorb parasitic-BJT R drops into K so v_d = p + state.k * i\n\
                 \x20       // gives the internal junction voltage directly. Lets bjt_evaluate\n\
                 \x20       // (intrinsic) replace the inner-NR cost of bjt_with_parasitics.\n{}",
                k_eff_stmts.trim_end_matches('\n')
            )
        };
        ctx.insert("k_eff_adjust_lines", &k_eff_fragment);
        let k_be_eff_stmts = k_eff_adjust_stmts(ir, "k_be", "        ");
        let k_be_eff_fragment = if k_be_eff_stmts.is_empty() {
            String::new()
        } else {
            format!(
                "\n        // K_eff for the BE kernel (same parasitic-BJT absorption as trap K)\n{}",
                k_be_eff_stmts.trim_end_matches('\n')
            )
        };
        ctx.insert("k_be_eff_adjust_lines", &k_be_eff_fragment);

        // Runtime DC operating point recompute (Oomox P6 / Phase E). The
        // template renders a stub body when this flag is on; full device
        // eval + NR loop emission is layered on in subsequent commits.
        let emit_dc_op_recompute = ir.solver_config.emit_dc_op_recompute;
        ctx.insert("emit_dc_op_recompute", &emit_dc_op_recompute);
        if emit_dc_op_recompute {
            let body = super::dc_op_emitter::emit_recompute_dc_op_body_dk(ir)?;
            ctx.insert("recompute_dc_op_body", &body);
            ctx.insert(
                "settle_dc_op_body",
                &super::dc_op_emitter::emit_settle_dc_op_body(ir),
            );
        }

        // #1 Step-6c damping: cache the damping threshold in state instead of
        // recomputing max|dc_operating_point| every sample. `damp_thresh_init`
        // is the has_dc_op-branch value (dc_operating_point == DC_OP at Default/
        // reset); the has_dc_op-false branch is a literal 2.0 (max of zeros).
        // Computed with the exact same fold + mul_add(0.05, 2.0) as the old
        // per-sample loop so the cached value is byte-identical. n_nodes is also
        // needed by the refresh_damp_thresh() helper's loop bound.
        let n_nodes = ir.topology.n_nodes;
        ctx.insert("n_nodes", &n_nodes);
        let damp_thresh_init = ir
            .dc_operating_point
            .iter()
            .take(n_nodes)
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max)
            .mul_add(0.05, 2.0);
        ctx.insert("damp_thresh_init", &format!("{:.17e}", damp_thresh_init));

        self.render("state", &ctx)
    }

    pub(super) fn emit_device_models(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = section_banner("DEVICE MODELS");

        let mut has_diode = false;
        let mut has_bjt = false;
        let mut has_jfet = false;
        let mut has_mosfet = false;
        let mut has_tube = false;
        let mut has_vca = false;
        let mut has_self_heating = false;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                DeviceParams::Diode(dp) => {
                    has_diode = true;
                    if dp.has_self_heating() {
                        has_self_heating = true;
                    }
                    emit_device_const(&mut code, dev_num, "IS", dp.is);
                    emit_device_const(&mut code, dev_num, "N_VT", dp.n_vt);
                    // Precomputed critical voltage for SPICE pnjlim
                    let vcrit = dp.n_vt * (dp.n_vt / (std::f64::consts::SQRT_2 * dp.is)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    if dp.has_rs() {
                        emit_device_const(&mut code, dev_num, "RS", dp.rs);
                    }
                    if dp.has_bv() {
                        emit_device_const(&mut code, dev_num, "BV", dp.bv);
                        emit_device_const(&mut code, dev_num, "IBV", dp.ibv);
                    }
                    if dp.has_self_heating() {
                        emit_device_const(&mut code, dev_num, "RTH", dp.rth);
                        emit_device_const(&mut code, dev_num, "CTH", dp.cth);
                        emit_device_const(&mut code, dev_num, "XTI", dp.xti);
                        emit_device_const(&mut code, dev_num, "EG", dp.eg);
                        emit_device_const(&mut code, dev_num, "TAMB", dp.tamb);
                        emit_device_const(&mut code, dev_num, "IS_NOM", dp.is);
                        emit_device_const(&mut code, dev_num, "N_VT_NOM", dp.n_vt);
                    }
                    code.push('\n');
                }
                DeviceParams::Bjt(bp) => {
                    has_bjt = true;
                    if bp.has_self_heating() {
                        has_self_heating = true;
                    }
                    emit_device_const(&mut code, dev_num, "IS", bp.is);
                    emit_device_const(&mut code, dev_num, "VT", bp.vt);
                    emit_device_const(&mut code, dev_num, "BETA_F", bp.beta_f);
                    emit_device_const(&mut code, dev_num, "BETA_R", bp.beta_r);
                    emit_device_const(&mut code, dev_num, "NF", bp.nf);
                    emit_device_const(&mut code, dev_num, "NR", bp.nr);
                    emit_device_const(&mut code, dev_num, "ISE", bp.ise);
                    emit_device_const(&mut code, dev_num, "NE", bp.ne);
                    emit_device_const(&mut code, dev_num, "ISC", bp.isc);
                    emit_device_const(&mut code, dev_num, "NC", bp.nc);
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n",
                        dev_num, sign
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_USE_GP: bool = {};\n",
                        dev_num,
                        bp.is_gummel_poon()
                    ));
                    emit_device_const(&mut code, dev_num, "VAF", bp.vaf);
                    emit_device_const(&mut code, dev_num, "VAR", bp.var);
                    emit_device_const(&mut code, dev_num, "IKF", bp.ikf);
                    emit_device_const(&mut code, dev_num, "IKR", bp.ikr);
                    // Precomputed critical voltage for SPICE pnjlim (both Vbe and Vbc junctions)
                    let vcrit = bp.vt * (bp.vt / (std::f64::consts::SQRT_2 * bp.is)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    if bp.has_parasitics() {
                        emit_device_const(&mut code, dev_num, "RB", bp.rb);
                        emit_device_const(&mut code, dev_num, "RC", bp.rc);
                        emit_device_const(&mut code, dev_num, "RE", bp.re);
                    }
                    if bp.has_self_heating() {
                        emit_device_const(&mut code, dev_num, "RTH", bp.rth);
                        emit_device_const(&mut code, dev_num, "CTH", bp.cth);
                        emit_device_const(&mut code, dev_num, "XTI", bp.xti);
                        emit_device_const(&mut code, dev_num, "XTB", bp.xtb);
                        emit_device_const(&mut code, dev_num, "EG", bp.eg);
                        emit_device_const(&mut code, dev_num, "TAMB", bp.tamb);
                        emit_device_const(&mut code, dev_num, "IS_NOM", bp.is);
                    }
                    code.push('\n');
                }
                DeviceParams::Jfet(jp) => {
                    has_jfet = true;
                    emit_device_const(&mut code, dev_num, "IDSS", jp.idss);
                    emit_device_const(&mut code, dev_num, "VP", jp.vp);
                    emit_device_const(&mut code, dev_num, "LAMBDA", jp.lambda);
                    if jp.has_rd_rs() {
                        emit_device_const(&mut code, dev_num, "RD", jp.rd);
                        emit_device_const(&mut code, dev_num, "RS", jp.rs);
                    }
                    let sign = if jp.is_p_channel { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
                DeviceParams::Mosfet(mp) => {
                    has_mosfet = true;
                    emit_device_const(&mut code, dev_num, "KP", mp.kp);
                    emit_device_const(&mut code, dev_num, "VT", mp.vt);
                    emit_device_const(&mut code, dev_num, "LAMBDA", mp.lambda);
                    if mp.has_rd_rs() {
                        emit_device_const(&mut code, dev_num, "RD", mp.rd);
                        emit_device_const(&mut code, dev_num, "RS", mp.rs);
                    }
                    if mp.has_body_effect() {
                        emit_device_const(&mut code, dev_num, "GAMMA", mp.gamma);
                        emit_device_const(&mut code, dev_num, "PHI", mp.phi);
                        code.push_str(&format!(
                            "const DEVICE_{}_SOURCE_NODE: usize = {};\n",
                            dev_num, mp.source_node
                        ));
                        code.push_str(&format!(
                            "const DEVICE_{}_BULK_NODE: usize = {};\n",
                            dev_num, mp.bulk_node
                        ));
                    }
                    let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
                DeviceParams::Tube(tp) => {
                    has_tube = true;
                    if tp.has_self_heating() {
                        has_self_heating = true;
                    }
                    emit_device_const(&mut code, dev_num, "MU", tp.mu);
                    emit_device_const(&mut code, dev_num, "EX", tp.ex);
                    emit_device_const(&mut code, dev_num, "KG1", tp.kg1);
                    emit_device_const(&mut code, dev_num, "KP", tp.kp);
                    emit_device_const(&mut code, dev_num, "KVB", tp.kvb);
                    emit_device_const(&mut code, dev_num, "IG_MAX", tp.ig_max);
                    emit_device_const(&mut code, dev_num, "VGK_ONSET", tp.vgk_onset);
                    emit_device_const(&mut code, dev_num, "LAMBDA", tp.lambda);
                    if tp.has_rgi() {
                        emit_device_const(&mut code, dev_num, "RGI", tp.rgi);
                    }
                    // Pentode-only constants (Reefman Derk §4.4). Triodes leave
                    // these unset so the emitted output is byte-identical for
                    // pure-triode circuits.
                    if tp.is_pentode() {
                        emit_device_const(&mut code, dev_num, "KG2", tp.kg2);
                        emit_device_const(&mut code, dev_num, "ALPHA_S", tp.alpha_s);
                        emit_device_const(&mut code, dev_num, "A_FACTOR", tp.a_factor);
                        emit_device_const(&mut code, dev_num, "BETA_FACTOR", tp.beta_factor);
                    }
                    // Grid-off reduced pentode constant: the DC-OP-converged
                    // screen voltage Vg2k that the 2D reduced NR block uses in
                    // place of the dropped NR dimension. Read from the
                    // `DeviceSlot`, not `TubeParams` — it's runtime state
                    // captured by the DC-OP grid-off detection pass.
                    if tp.is_grid_off_pentode() {
                        emit_device_const(&mut code, dev_num, "VG2K_FROZEN", slot.vg2k_frozen);
                    }
                    // Variable-mu §5 constants. Emitted only when `svar > 0`
                    // to preserve byte-identity for sharp (phase 1a/1a.1)
                    // circuits. Applies to BOTH triodes and pentodes.
                    if tp.is_variable_mu() {
                        emit_device_const(&mut code, dev_num, "MU_B", tp.mu_b);
                        emit_device_const(&mut code, dev_num, "SVAR", tp.svar);
                        emit_device_const(&mut code, dev_num, "EX_B", tp.ex_b);
                    }
                    // Precomputed critical voltage for SPICE pnjlim (grid current onset)
                    let vt_tube = tp.vgk_onset / 3.0;
                    let vcrit = vt_tube * (vt_tube / (std::f64::consts::SQRT_2 * 1e-10)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    // Self-heating constants. Only the thermal gate (RTH) is
                    // checked here — `has_self_heating()` is already false for
                    // pentodes, so this block is triode-only in phase 1.
                    if tp.has_self_heating() {
                        emit_device_const(&mut code, dev_num, "RTH", tp.rth);
                        emit_device_const(&mut code, dev_num, "CTH", tp.cth);
                        emit_device_const(&mut code, dev_num, "VBIAS_ALPHA", tp.vbias_alpha);
                        emit_device_const(&mut code, dev_num, "TAMB", tp.tamb);
                    }
                    code.push('\n');
                }
                DeviceParams::Vca(vp) => {
                    has_vca = true;
                    emit_device_const(&mut code, dev_num, "VSCALE", vp.vscale);
                    emit_device_const(&mut code, dev_num, "G0", vp.g0);
                    emit_device_const(&mut code, dev_num, "THD", vp.thd);
                    code.push('\n');
                }
                DeviceParams::Ldr(lp) => {
                    // Opto/LDR: model params consumed by the after-solve update()
                    // hook (target-R power law + attack/release taus). The live
                    // resistance is the opaque `device_{n}_state` block, not a
                    // const. The NR eval reads `1/state[0]` — no const needed there.
                    emit_device_const(&mut code, dev_num, "RMIN", lp.r_min);
                    emit_device_const(&mut code, dev_num, "RMAX", lp.r_max);
                    emit_device_const(&mut code, dev_num, "GAMMA", lp.gamma);
                    emit_device_const(&mut code, dev_num, "TAU_A", lp.attack_tau);
                    emit_device_const(&mut code, dev_num, "TAU_R", lp.release_tau);
                    code.push('\n');
                }
            }
        }

        // Boltzmann constant / elementary charge (k/q in eV/K)
        if has_self_heating {
            code.push_str("/// Boltzmann constant / elementary charge [eV/K]\n");
            code.push_str("const BOLTZMANN_Q: f64 = 8.617333262e-5;\n\n");
        }

        // Fast exp() approximation (needed by all nonlinear device models)
        if has_diode || has_bjt || has_jfet || has_mosfet || has_tube || has_vca {
            code.push_str(&Self::emit_fast_exp());
        }

        // SPICE voltage limiting functions (needed by all nonlinear devices)
        if has_diode || has_bjt || has_jfet || has_mosfet || has_tube || has_vca {
            code.push_str(&self.render("spice_limiting", &Context::new())?);
        }

        if has_diode {
            code.push_str(&self.render("device_diode", &Context::new())?);
        }
        if has_bjt {
            code.push_str(&self.render("device_bjt", &Context::new())?);
        }
        if has_jfet {
            code.push_str(&self.render("device_jfet", &Context::new())?);
        }
        if has_mosfet {
            code.push_str(&self.render("device_mosfet", &Context::new())?);
        }
        if has_tube {
            // Five Tera guards for pentode-family tubes:
            //   any_pentode            — sharp Rational (Derk §4.4): EL84/EL34/EF86
            //   any_beam_tetrode       — sharp Exponential (DerkE §4.5): 6L6GC/6V6GT
            //   any_variable_mu_pentode       — §5 two-section Rational: 6K7
            //   any_variable_mu_beam_tetrode  — §5 two-section Exponential: EF89
            //   any_classical_pentode  — Classical Norman Koren (Cohen-Hélie §2): KT88/6550
            //
            // Byte-identity guarantee: a circuit containing only non-pentode tubes
            // (pure-triode) has all five false and emits the phase-1a triode block
            // unchanged. A circuit containing only sharp pentodes (svar=0) has
            // `any_variable_mu_*` and `any_classical_pentode` false and emits
            // phase-1a.1 output byte-identical. A pure-Derk (Rational/Exponential)
            // circuit has `any_classical_pentode` false and emits phase-1c output
            // byte-identical. Variable-mu Classical is rejected by
            // `TubeParams::validate()`, so no `any_variable_mu_classical_pentode`
            // flag exists.
            // Grid-off pentode guard: excluded from the sharp flags below so
            // the existing sharp / variable-mu / Classical helper families
            // stay byte-identical for circuits with zero grid-off slots.
            // `any_grid_off_pentode` gates a fresh block of thin 2D wrapper
            // helpers that delegate back to the sharp 3D helpers with
            // `vg2k_frozen` substituted for the live Vg2k dimension.
            let any_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational)
                        && !tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_beam_tetrode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential)
                        && !tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_variable_mu_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational)
                        && tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_variable_mu_beam_tetrode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential)
                        && tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_classical_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Classical)
                        && !tp.is_grid_off_pentode())
            });
            let any_grid_off_pentode = ir.device_slots.iter().any(
                |slot| matches!(&slot.params, DeviceParams::Tube(tp) if tp.is_grid_off_pentode()),
            );
            // Grid-off wrapper helpers delegate to the sharp 3D helpers of
            // the matching screen form, so whichever sharp family a grid-off
            // slot uses MUST have its own helper emitted too. Force the
            // matching sharp flag on whenever a grid-off slot exists of that
            // screen form. This keeps the template simple (a single
            // `{% if any_grid_off_pentode %}` block can reference both the
            // wrapper and the 3D helper it delegates to) without leaking
            // sharp helpers into circuits that have neither.
            let any_grid_off_rational = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational))
            });
            let any_grid_off_exponential = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential))
            });
            let any_grid_off_classical = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Classical))
            });
            let any_pentode = any_pentode || any_grid_off_rational;
            let any_beam_tetrode = any_beam_tetrode || any_grid_off_exponential;
            let any_classical_pentode = any_classical_pentode || any_grid_off_classical;
            let mut tube_ctx = Context::new();
            tube_ctx.insert("any_pentode", &any_pentode);
            tube_ctx.insert("any_beam_tetrode", &any_beam_tetrode);
            tube_ctx.insert("any_variable_mu_pentode", &any_variable_mu_pentode);
            tube_ctx.insert(
                "any_variable_mu_beam_tetrode",
                &any_variable_mu_beam_tetrode,
            );
            tube_ctx.insert("any_classical_pentode", &any_classical_pentode);
            tube_ctx.insert("any_grid_off_pentode", &any_grid_off_pentode);
            tube_ctx.insert("any_grid_off_rational", &any_grid_off_rational);
            tube_ctx.insert("any_grid_off_exponential", &any_grid_off_exponential);
            tube_ctx.insert("any_grid_off_classical", &any_grid_off_classical);
            code.push_str(&self.render("device_tube", &tube_ctx)?);
        }
        if has_vca {
            code.push_str(&self.render("device_vca", &Context::new())?);
        }

        // Stateful-device (Phase 0c) update() hooks. Shared by BOTH emitters —
        // `emit_device_models` is called from the DK and nodal generate paths —
        // so the update signature and body can never drift. Empty when no
        // device is stateful (byte-identical to before this machinery existed).
        code.push_str(&emit_stateful_update_fns(
            &stateful_device_data(ir),
            &ir.device_slots,
        ));

        Ok(code)
    }

    /// Emit fast exp() approximation function.
    ///
    /// Default: polynomial range reduction + 5th-order minimax (~6 cycles, <0.0004% error).
    /// Opt-in: `--cfg melange_precise_exp` uses hardware/libm exp (~38 cycles).
    ///
    /// Accuracy of polynomial path: <0.0004% max relative error over [-40, 40].
    pub(super) fn emit_fast_exp() -> String {
        let mut code = String::new();
        code.push_str(
            "/// Fast exp() for audio circuit simulation.\n\
             /// Input clamped to [-40, 40] (matches melange safe_exp convention).\n\
             ///\n\
             /// Default: polynomial approximation (<0.0004% error, ~6x faster than libm).\n\
             /// To use hardware/libm exp, compile with: `--cfg melange_precise_exp`\n\
             #[inline(always)]\n\
             fn fast_exp(x: f64) -> f64 {\n\
             \x20   #[cfg(melange_precise_exp)]\n\
             \x20   { x.clamp(-40.0, 40.0).exp() }\n\
             \x20   #[cfg(not(melange_precise_exp))]\n\
             \x20   {\n\
             \x20       // Range reduction + 5th-order minimax polynomial. <0.0004% max relative error.\n\
             \x20       // No lookup tables, no libm dependency, branchless hot path.\n\
             \x20       let x = x.clamp(-40.0, 40.0);\n\
             \x20       const LN2_INV: f64 = std::f64::consts::LOG2_E;\n\
             \x20       const LN2_HI: f64 = 0.6931471803691238;\n\
             \x20       const LN2_LO: f64 = 1.9082149292705877e-10;\n\
             \x20       const SHIFT: f64 = 6755399441055744.0; // 2^52 + 2^51\n\
             \x20       let z = x * LN2_INV + SHIFT;\n\
             \x20       let n_i64 = z.to_bits() as i64 - SHIFT.to_bits() as i64;\n\
             \x20       let n = n_i64 as f64;\n\
             \x20       let f = (x - n * LN2_HI) - n * LN2_LO;\n\
             \x20       let p = 1.0 + f * (1.0 + f * (0.5 + f * (0.16666666666666607\n\
             \x20           + f * (0.04166666666665876 + f * 0.008333333333492337))));\n\
             \x20       let pow2n = f64::from_bits(((1023 + n_i64) as u64) << 52);\n\
             \x20       p * pow2n\n\
             \x20   }\n\
             }\n\n",
        );
        // fast_ln: used in tube softplus ln(1+exp(x))
        code.push_str(
            "/// Fast ln() for audio circuit simulation.\n\
             /// Symmetric log series (~0.005% max relative error, ~3x faster than libm).\n\
             /// Only valid for positive inputs.\n\
             #[inline(always)]\n\
             fn fast_ln(x: f64) -> f64 {\n\
             \x20   #[cfg(melange_precise_exp)]\n\
             \x20   { x.ln() }\n\
             \x20   #[cfg(not(melange_precise_exp))]\n\
             \x20   {\n\
             \x20       // Extract exponent and mantissa from IEEE 754 double\n\
             \x20       let bits = x.to_bits();\n\
             \x20       let e = ((bits >> 52) & 0x7FF) as i64 - 1023;\n\
             \x20       // Normalize mantissa to [1, 2)\n\
             \x20       let m_bits = (bits & 0x000F_FFFF_FFFF_FFFF) | 0x3FF0_0000_0000_0000;\n\
             \x20       let m = f64::from_bits(m_bits);\n\
             \x20       // Symmetric log series: u = (m-1)/(m+1), ln(m) = 2u(1 + u²/3 + u⁴/5 + u⁶/7)\n\
             \x20       // For m in [1,2), u in [0,1/3): converges much faster than Taylor.\n\
             \x20       let u = (m - 1.0) / (m + 1.0);\n\
             \x20       let u2 = u * u;\n\
             \x20       let ln_m = 2.0 * u * (1.0 + u2 * (0.3333333333333333 + u2 * (0.2 + u2 * 0.14285714285714285)));\n\
             \x20       ln_m + (e as f64) * std::f64::consts::LN_2\n\
             \x20   }\n\
             }\n\n",
        );
        code
    }

    /// Generate switch setter methods and rebuild_matrices() procedurally.
    fn emit_switch_methods(&self, ir: &CircuitIR, noise: &NoiseEmission) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let num_pots = ir.pots.len();
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        let mut code = String::new();

        // Emit set_switch_N() for each switch (DK path)
        for sw in &ir.switches {
            // Authentic-noise coefficient refresh for any R-type components
            // in this switch that back a thermal noise source. Emitted only
            // when noise is compiled in AND this switch has at least one
            // noise-tracked R. DK uses the 2D const array
            // `SWITCH_<N>_VALUES[position][comp_idx]`, distinct from
            // the nodal path's per-component arrays.
            let noise_update: String = if noise.enabled {
                let idx = sw.index;
                let mut out = String::new();
                if let Some(slots) = noise.switch_comp_to_noise_slot.get(idx) {
                    for (ci, maybe_slot) in slots.iter().enumerate() {
                        if let Some(slot) = maybe_slot {
                            out.push_str(&format!(
                                "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / SWITCH_{idx}_VALUES[position][{ci}]).sqrt();\n",
                            ));
                        }
                    }
                }
                if let Some(slots) = noise.switch_comp_to_r_flicker_slot.get(idx) {
                    for (ci, maybe_slot) in slots.iter().enumerate() {
                        if let Some(slot) = maybe_slot {
                            out.push_str(&format!(
                                "        self.noise_r_flicker_inv_r[{slot}] = 1.0 / SWITCH_{idx}_VALUES[position][{ci}];\n",
                            ));
                        }
                    }
                }
                // Phase 4: switch R at an op-amp in+ shifts the en Norton
                // conversion factor — absolute recompute (previously the
                // switch hook was missing entirely; only pots refreshed).
                // Runs after `self.switch_N_position = position`.
                if noise
                    .switch_to_opamp_en_refresh
                    .get(idx)
                    .copied()
                    .unwrap_or(false)
                {
                    out.push_str("        self.refresh_opamp_en_g_diag();\n");
                }
                out
            } else {
                String::new()
            };
            code.push_str(&format!(
                "    /// Set switch {} position (0..{}).\n\
                 \x20   ///\n\
                 \x20   /// Marks matrices dirty. Rebuild deferred to next `process_sample()`.\n\
                 \x20   /// A switch flip is a topology step — follow with `recompute_dc_op()`\n\
                 \x20   /// to refresh the NR seed and avoid audible glitches on the next sample.\n\
                 \x20   pub fn set_switch_{}(&mut self, position: usize) {{\n\
                 \x20       if position >= SWITCH_{}_NUM_POSITIONS {{ return; }}\n\
                 \x20       if self.switch_{}_position == position {{ return; }}\n\
                 \x20       self.switch_{}_position = position;\n\
                 \x20       self.matrices_dirty = true;\n\
{noise_update}\
                 \x20   }}\n\n",
                sw.index,
                sw.num_positions - 1,
                sw.index,
                sw.index,
                sw.index,
                sw.index,
            ));
        }

        // Emit set_pot_N() / set_runtime_R_<field>() methods for DK path.
        //
        // Both setters have identical bodies: clamp → skip-if-unchanged →
        // stamp the dirty flag → refresh noise coefficient. No NR-state
        // reseed — callers that need one (preset recall, raw unsmoothed
        // jumps) should follow the setter with `recompute_dc_op()`.
        // Plugin-template callers feed `.smoothed.next()` values; nih-plug
        // smoothing keeps per-sample deltas tiny, so NR stays within basin
        // on the previous sample's seed.
        for (idx, pot) in ir.pots.iter().enumerate() {
            // Authentic-noise coefficient refresh — emitted only when
            // noise is enabled at codegen AND this pot backs a thermal
            // source. Keeps `state.noise_thermal_sqrt_inv_r[k]` in sync
            // with the live R so Johnson-Nyquist variance tracks the knob.
            // Phase 4: op-amp `en_g_diag` refresh follows the same hook
            // for pots that touch an op-amp's non-inverting input.
            let opamp_refresh_targets: &[usize] = noise
                .pot_to_opamp_en_refresh
                .get(idx)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            let noise_update: String = if noise.enabled {
                let mut out = String::new();
                if let Some(slot) = noise.pot_to_noise_slot.get(idx).copied().flatten() {
                    out.push_str(&format!(
                        "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / r).sqrt();\n",
                    ));
                }
                if let Some(slot) = noise.pot_to_r_flicker_slot.get(idx).copied().flatten() {
                    out.push_str(&format!(
                        "        self.noise_r_flicker_inv_r[{slot}] = 1.0 / r;\n",
                    ));
                }
                // Phase 4: op-amp en_g_diag refresh for pots touching an
                // op-amp in+. Absolute recompute (BASE + Σ live dynamic G) —
                // see refresh_opamp_en_g_diag; runs after the new resistance
                // is stored so it reads the fresh value.
                if !opamp_refresh_targets.is_empty() {
                    out.push_str("        self.refresh_opamp_en_g_diag();\n");
                }
                out
            } else {
                String::new()
            };

            match &pot.runtime_field {
                None => {
                    code.push_str(&format!(
                        "    /// Set potentiometer {idx} resistance (clamped to [{:.1}..{:.1}] ohms).\n\
                         \x20   ///\n\
                         \x20   /// Marks matrices dirty. Rebuild deferred to next `process_sample()`.\n\
                         \x20   /// For preset recall / unsmoothed jumps, follow with `recompute_dc_op()`.\n\
                         \x20   pub fn set_pot_{idx}(&mut self, resistance: f64) {{\n\
                         \x20       if !resistance.is_finite() {{ return; }}\n\
                         \x20       let r = resistance.clamp(POT_{idx}_MIN_R, POT_{idx}_MAX_R);\n\
                         \x20       if (r - self.pot_{idx}_resistance).abs() < 1e-12 {{ return; }}\n\
                         \x20       self.pot_{idx}_resistance = r;\n\
                         \x20       self.matrices_dirty = true;\n\
{noise_update}\
                         \x20   }}\n\n",
                        pot.min_resistance, pot.max_resistance,
                    ));
                }
                Some(field) => {
                    let field_upper = field.to_ascii_uppercase();
                    code.push_str(&format!(
                        "    /// Current resistance of runtime resistor `{field}` (ohms).\n\
                         \x20   ///\n\
                         \x20   /// Read-only accessor; use `set_runtime_R_{field}` to update.\n\
                         \x20   #[inline]\n\
                         \x20   pub fn {field}(&self) -> f64 {{ self.pot_{idx}_resistance }}\n\n\
                         \x20   /// Set runtime resistor `{field}` (clamped to [{:.1}..{:.1}] ohms).\n\
                         \x20   ///\n\
                         \x20   /// Audio-rate safe: no internal smoothing; caller (plugin-side\n\
                         \x20   /// envelope follower) is the smoother.\n\
                         \x20   /// Marks matrices dirty. Rebuild deferred to next `process_sample()`.\n\
                         \x20   pub fn set_runtime_R_{field}(&mut self, resistance: f64) {{\n\
                         \x20       if !resistance.is_finite() {{ return; }}\n\
                         \x20       let r = resistance.clamp(RUNTIME_R_{field_upper}_MIN, RUNTIME_R_{field_upper}_MAX);\n\
                         \x20       if (r - self.pot_{idx}_resistance).abs() < 1e-12 {{ return; }}\n\
                         \x20       self.pot_{idx}_resistance = r;\n\
                         \x20       self.matrices_dirty = true;\n\
{noise_update}\
                         \x20   }}\n\n",
                        pot.min_resistance, pot.max_resistance,
                    ));
                }
            }
        }

        // Emit rebuild_matrices()
        code.push_str(
            "    /// Rebuild all sample-rate-dependent matrices from G/C constants.\n\
             \x20   ///\n\
             \x20   /// Applies switch/pot deltas to G/C, then rebuilds A, S, K, S*N_i.\n\
             \x20   /// Called by `set_switch_N()`, `set_pot_N()`, and `set_sample_rate()`.\n\
             \x20   fn rebuild_matrices(&mut self) {\n\
             \x20       let internal_rate = self.current_sample_rate * OVERSAMPLING_FACTOR as f64;\n",
        );
        if ir.solver_config.backward_euler {
            code.push_str("        let alpha = internal_rate; // backward Euler: alpha = 1/T\n");
        } else {
            code.push_str("        let alpha = 2.0 * internal_rate; // trapezoidal: alpha = 2/T\n");
        }
        if num_inductors > 0 {
            code.push_str("        let t = 1.0 / internal_rate;\n");
        }

        // Start from constant G, C
        let has_r_switch = ir
            .switches
            .iter()
            .any(|sw| sw.components.iter().any(|c| c.component_type == 'R'));
        let has_c_switch = ir
            .switches
            .iter()
            .any(|sw| sw.components.iter().any(|c| c.component_type == 'C'));
        let g_mut = if has_r_switch || num_pots > 0 {
            "mut "
        } else {
            ""
        };
        let c_mut = if has_c_switch { "mut " } else { "" };
        code.push_str(&format!(
            "\n\
             \x20       // Start from constant G and C matrices\n\
             \x20       let {}g_eff = G;\n\
             \x20       let {}c_eff = C;\n",
            g_mut, c_mut,
        ));

        // Apply switch deltas
        code.push_str(
            "\n\
             \x20       // Apply switch position deltas\n",
        );
        for sw in &ir.switches {
            for (ci, comp) in sw.components.iter().enumerate() {
                let nominal = fmt_f64(comp.nominal_value);
                code.push_str(&format!(
                    "        {{\n\
                     \x20           let new_val = SWITCH_{}_VALUES[self.switch_{}_position][{}];\n",
                    sw.index, sw.index, ci,
                ));
                match comp.component_type {
                    'R' => {
                        code.push_str(&format!(
                            "            let delta_g = 1.0 / new_val - 1.0 / {};\n\
                             \x20           stamp_conductance(&mut g_eff, SWITCH_{}_COMP_{}_NODE_P, SWITCH_{}_COMP_{}_NODE_Q, delta_g);\n",
                            nominal, sw.index, ci, sw.index, ci,
                        ));
                    }
                    'C' => {
                        code.push_str(&format!(
                            "            let delta_c = new_val - {};\n\
                             \x20           stamp_conductance(&mut c_eff, SWITCH_{}_COMP_{}_NODE_P, SWITCH_{}_COMP_{}_NODE_Q, delta_c);\n",
                            nominal, sw.index, ci, sw.index, ci,
                        ));
                    }
                    'L' => {
                        if let Some(aug_row) = comp.augmented_row {
                            // Augmented MNA: L value lives on diagonal of branch variable row
                            code.push_str(&format!(
                                "            let delta_l = new_val - {};\n\
                                 \x20           c_eff[{}][{}] += delta_l;\n",
                                nominal, aug_row, aug_row,
                            ));
                        } else {
                            // DK companion model: handled in inductor companion stamp below
                            code.push_str(
                                "            // Inductor: handled in companion model stamp below\n",
                            );
                            code.push_str("            let _ = new_val;\n");
                        }
                    }
                    _ => {}
                }
                code.push_str("        }\n");
            }
        }

        // Apply pot conductance deltas (relative to nominal)
        if num_pots > 0 {
            code.push_str(
                "\n\
                 \x20       // Apply pot conductance deltas (current resistance vs nominal)\n",
            );
            for (idx, pot) in ir.pots.iter().enumerate() {
                let np = pot.node_p;
                let nq = pot.node_q;
                code.push_str(&format!(
                    "        {{\n\
                     \x20           let delta_g = 1.0 / self.pot_{idx}_resistance - POT_{idx}_G_NOM;\n",
                ));
                if np > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] += delta_g;\n",
                        np - 1,
                        np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] += delta_g;\n",
                        nq - 1,
                        nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] -= delta_g;\n\
                         \x20           g_eff[{}][{}] -= delta_g;\n",
                        np - 1,
                        nq - 1,
                        nq - 1,
                        np - 1
                    ));
                }
                code.push_str("        }\n");
            }
        }

        // Build A = g_eff + alpha * c_eff
        code.push_str(
            "\n\
             \x20       // Build A = G_eff + alpha * C_eff\n\
             \x20       let mut a = [[0.0f64; N]; N];\n\
             \x20       for i in 0..N {\n\
             \x20           for j in 0..N {\n\
             \x20               a[i][j] = g_eff[i][j] + alpha * c_eff[i][j];\n\
             \x20           }\n\
             \x20       }\n\n",
        );
        // A_neg formula: trapezoidal = alpha*C - G; backward Euler = alpha*C (no G term)
        let a_neg_formula = if ir.solver_config.backward_euler {
            "alpha * c_eff[i][j]"
        } else {
            "alpha * c_eff[i][j] - g_eff[i][j]"
        };
        code.push_str(&format!(
            "        // Build A_neg: {}\n\
             \x20       let mut a_neg = [[0.0f64; N]; N];\n\
             \x20       for i in 0..N {{\n\
             \x20           for j in 0..N {{\n\
             \x20               a_neg[i][j] = {};\n\
             \x20           }}\n\
             \x20       }}\n",
            if ir.solver_config.backward_euler {
                "alpha*C (backward Euler, no G)"
            } else {
                "alpha*C - G (trapezoidal)"
            },
            a_neg_formula
        ));
        // Zero augmented rows in A_neg (algebraic constraints for VS/VCVS)
        // When augmented_inductors, only zero n_nodes..n_aug (not inductor rows)
        let n_aug = ir.topology.n_aug;
        let a_neg_zero_end = if ir.topology.augmented_inductors {
            n_aug
        } else {
            n
        };
        if n_nodes < a_neg_zero_end {
            code.push_str(&format!(
                "        // Zero VS/VCVS algebraic rows in A_neg (NOT inductor rows)\n\
                 \x20       for i in {n_nodes}..{a_neg_zero_end} {{\n\
                 \x20           for j in 0..N {{\n\
                 \x20               a_neg[i][j] = 0.0;\n\
                 \x20           }}\n\
                 \x20       }}\n"
            ));
        }

        // Inductor companion stamps (with switch-aware inductance)
        if num_inductors > 0 {
            code.push_str("\n        // Add inductor companion model conductances\n");
            for (li, ind) in ir.inductors.iter().enumerate() {
                // Check if any switch controls this inductor
                let mut switched = false;
                for sw in &ir.switches {
                    for (ci, comp) in sw.components.iter().enumerate() {
                        if comp.component_type == 'L' && comp.inductor_index == Some(li) {
                            code.push_str(&format!(
                                "        {{\n\
                                 \x20           let inductance = SWITCH_{}_VALUES[self.switch_{}_position][{}];\n\
                                 \x20           let g_eq = t / (2.0 * inductance);\n\
                                 \x20           self.ind_g_eq[{}] = g_eq;\n\
                                 \x20           stamp_conductance(&mut a, {}, {}, g_eq);\n\
                                 \x20           stamp_conductance(&mut a_neg, {}, {}, -g_eq);\n\
                                 \x20       }}\n",
                                sw.index, sw.index, ci,
                                li,
                                ind.node_i, ind.node_j,
                                ind.node_i, ind.node_j,
                            ));
                            switched = true;
                            break;
                        }
                    }
                    if switched {
                        break;
                    }
                }
                if !switched {
                    // Non-switched inductor: use constant
                    code.push_str(&format!(
                        "        {{\n\
                         \x20           let g_eq = t / (2.0 * IND_{}_INDUCTANCE);\n\
                         \x20           self.ind_g_eq[{}] = g_eq;\n\
                         \x20           stamp_conductance(&mut a, IND_{}_NODE_I, IND_{}_NODE_J, g_eq);\n\
                         \x20           stamp_conductance(&mut a_neg, IND_{}_NODE_I, IND_{}_NODE_J, -g_eq);\n\
                         \x20       }}\n",
                        li, li, li, li, li, li,
                    ));
                }
            }
            code.push_str(&format!(
                "        // Preserve inductor transient state across rebuilds: zeroing\n\
                 \x20       // i_prev here would dump any standing DC current on every\n\
                 \x20       // pot/switch move (per-sample pot smoothing calls rebuild\n\
                 \x20       // continuously → audible DC thump). Refresh the history from\n\
                 \x20       // the preserved current: i_hist = 2*i_prev (doubled-trapezoidal\n\
                 \x20       // form, independent of g_eq).\n\
                 \x20       for li in 0..{} {{\n\
                 \x20           self.ind_i_hist[li] = 2.0 * self.ind_i_prev[li];\n\
                 \x20       }}\n",
                num_inductors,
            ));
        }

        // Coupled inductor companion stamps
        let num_coupled = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        if num_coupled > 0 {
            if num_inductors == 0 {
                code.push_str("        let t = 1.0 / internal_rate;\n");
            }
            code.push_str("\n        // Add coupled inductor companion model conductances\n");
            for (ci_idx, _ci) in ir.coupled_inductors.iter().enumerate() {
                // Check if either winding is switch-controlled
                let mut l1_switch: Option<(usize, usize)> = None; // (sw_index, comp_index)
                let mut l2_switch: Option<(usize, usize)> = None;
                for sw in &ir.switches {
                    for (comp_i, comp) in sw.components.iter().enumerate() {
                        if comp.coupled_inductor_index == Some(ci_idx) {
                            match comp.coupled_winding {
                                Some(1) => l1_switch = Some((sw.index, comp_i)),
                                Some(2) => l2_switch = Some((sw.index, comp_i)),
                                _ => {}
                            }
                        }
                    }
                }

                let l1_expr = if let Some((sw_idx, comp_idx)) = l1_switch {
                    format!(
                        "SWITCH_{}_COMP_{}_VALUES[self.switch_{}_position]",
                        sw_idx, comp_idx, sw_idx
                    )
                } else {
                    format!("CI_{}_L1_INDUCTANCE", ci_idx)
                };
                let l2_expr = if let Some((sw_idx, comp_idx)) = l2_switch {
                    format!(
                        "SWITCH_{}_COMP_{}_VALUES[self.switch_{}_position]",
                        sw_idx, comp_idx, sw_idx
                    )
                } else {
                    format!("CI_{}_L2_INDUCTANCE", ci_idx)
                };

                code.push_str(&format!(
                    "        {{\n\
                     \x20           let l1_val = {l1};\n\
                     \x20           let l2_val = {l2};\n\
                     \x20           let m_val = CI_{ci}_COUPLING * (l1_val * l2_val).sqrt();\n\
                     \x20           let det = l1_val * l2_val - m_val * m_val;\n\
                     \x20           let half_t = t / 2.0;\n\
                     \x20           let gs1 = half_t * l2_val / det;\n\
                     \x20           let gs2 = half_t * l1_val / det;\n\
                     \x20           let gm = -half_t * m_val / det;\n\
                     \x20           self.ci_g_self_1[{ci}] = gs1;\n\
                     \x20           self.ci_g_self_2[{ci}] = gs2;\n\
                     \x20           self.ci_g_mutual[{ci}] = gm;\n\
                     \x20           stamp_conductance(&mut a, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, gs1);\n\
                     \x20           stamp_conductance(&mut a_neg, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, -gs1);\n\
                     \x20           stamp_conductance(&mut a, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, gs2);\n\
                     \x20           stamp_conductance(&mut a_neg, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, -gs2);\n\
                     \x20           stamp_mutual(&mut a, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, gm);\n\
                     \x20           stamp_mutual(&mut a, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, gm);\n\
                     \x20           stamp_mutual(&mut a_neg, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, -gm);\n\
                     \x20           stamp_mutual(&mut a_neg, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, -gm);\n\
                     \x20       }}\n",
                    l1 = l1_expr,
                    l2 = l2_expr,
                    ci = ci_idx,
                ));
            }
            code.push_str(&format!(
                "        // Preserve coupled-inductor transient state across rebuilds\n\
                 \x20       // (see uncoupled comment); refresh history from preserved\n\
                 \x20       // winding currents: i_hist = 2*i_prev.\n\
                 \x20       for ci in 0..{n} {{\n\
                 \x20           self.ci_i1_hist[ci] = 2.0 * self.ci_i1_prev[ci];\n\
                 \x20           self.ci_i2_hist[ci] = 2.0 * self.ci_i2_prev[ci];\n\
                 \x20       }}\n",
                n = num_coupled,
            ));
        }

        // Transformer group companion stamps
        let num_xfmr_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        if num_xfmr_groups > 0 {
            if num_inductors == 0 && num_coupled == 0 {
                code.push_str("        let t = 1.0 / internal_rate;\n");
            }
            code.push_str("\n        // Add transformer group companion model conductances\n");
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                // Build L matrix from inductances and couplings, invert, multiply by T/2
                code.push_str(
                    "        {\n\
                     \x20           let half_t = t / 2.0;\n\
                     \x20           // Build inductance matrix L[i][j] = k[i][j] * sqrt(Li*Lj)\n",
                );
                // Emit L matrix construction
                for i in 0..w {
                    for j in 0..w {
                        code.push_str(&format!(
                            "            let l_{i}_{j} = XFMR_{gi}_COUPLING[{flat}] * (XFMR_{gi}_INDUCTANCES[{i}] * XFMR_{gi}_INDUCTANCES[{j}]).sqrt();\n",
                            flat = i * w + j,
                        ));
                    }
                }
                // Inline Gauss elimination to invert W x W matrix
                code.push_str(&format!("            let y = invert_xfmr_{gi}(["));
                for i in 0..w {
                    if i > 0 {
                        code.push_str(", ");
                    }
                    code.push('[');
                    for j in 0..w {
                        if j > 0 {
                            code.push_str(", ");
                        }
                        code.push_str(&format!("l_{i}_{j}"));
                    }
                    code.push(']');
                }
                code.push_str("]);\n");
                // Store Y = half_t * inv(L) and stamp
                for i in 0..w {
                    for j in 0..w {
                        code.push_str(&format!(
                            "            self.xfmr_{gi}_y[{flat}] = half_t * y[{i}][{j}];\n",
                            flat = i * w + j,
                        ));
                    }
                }
                // Stamp self-conductances (diagonal)
                for i in 0..w {
                    let flat = i * w + i;
                    code.push_str(&format!(
                        "            stamp_conductance(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], self.xfmr_{gi}_y[{flat}]);\n\
                         \x20           stamp_conductance(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], -self.xfmr_{gi}_y[{flat}]);\n",
                    ));
                }
                // Stamp mutual conductances (off-diagonal)
                for i in 0..w {
                    for j in 0..w {
                        if i == j {
                            continue;
                        }
                        let flat = i * w + j;
                        code.push_str(&format!(
                            "            stamp_mutual(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], self.xfmr_{gi}_y[{flat}]);\n",
                        ));
                        code.push_str(&format!(
                            "            stamp_mutual(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], -self.xfmr_{gi}_y[{flat}]);\n",
                        ));
                    }
                }
                code.push_str("        }\n");
            }
            // Preserve transformer-group transient state across rebuilds
            // (see uncoupled comment); refresh history from preserved
            // winding currents: i_hist = 2*i_prev.
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                code.push_str(&format!(
                    "        for wk in 0..{w} {{\n\
                     \x20           self.xfmr_{gi}_i_hist[wk] = 2.0 * self.xfmr_{gi}_i_prev[wk];\n\
                     \x20       }}\n",
                ));
            }
        }

        // Invert A → S, compute S_NI, K. Use the equilibrated invert so
        // partial-pivoting LU keeps cond(A) inside f64 precision even when
        // G_in (≈1 S) dominates internal conductances (1e-4 to 1e-6 S) by
        // 4-6 decades.
        code.push_str(&format!(
            "\n\
             \x20       // Invert A to get S (asymmetric row/column equilibration)\n\
             \x20       let (s, singular) = invert_n_equilibrated(a);\n\
             \x20       if singular {{ self.diag_singular_matrix_count += 1; }}\n\n\
             \x20       // Compute S * N_i product (N x M)\n\
             \x20       let mut s_ni = [[0.0f64; M]; N];\n\
             \x20       for i in 0..N {{\n\
             \x20           for j in 0..M {{\n\
             \x20               let mut sum = 0.0;\n\
             \x20               for kk in 0..N {{\n\
             \x20                   sum += s[i][kk] * N_I[kk][j];\n\
             \x20               }}\n\
             \x20               s_ni[i][j] = sum;\n\
             \x20           }}\n\
             \x20       }}\n\n\
             \x20       // Compute K = N_v * S_NI (M x M)\n\
             \x20       let mut k = [[0.0f64; {m}]; {m}];\n\
             \x20       for i in 0..M {{\n\
             \x20           for j in 0..M {{\n\
             \x20               let mut sum = 0.0;\n\
             \x20               for n_idx in 0..N {{\n\
             \x20                   sum += N_V[i][n_idx] * s_ni[n_idx][j];\n\
             \x20               }}\n\
             \x20               k[i][j] = sum;\n\
             \x20           }}\n\
             \x20       }}\n",
            m = m,
        ));

        // K_eff: subtract parasitic-BJT R_p from each affected 2x2 block.
        // Mirrors the codegen-time K_DEFAULT adjustment, so state.k holds
        // K_eff after both default-init and any pot/switch rebuild.
        // Gated on slot.dimension == 2: FA-reduced (1D) BJTs ignore parasitics
        // by design (see ir.rs::detect_forward_active_bjts comment), and their
        // start_idx+1 lands in the next device's slot, not the same BJT's Vbc row.
        let k_eff_stmts = k_eff_adjust_stmts(ir, "k", "        ");
        if !k_eff_stmts.is_empty() {
            code.push_str(
                "\n        // K_eff: absorb parasitic-BJT R drops into K so v_d = p + state.k * i\n\
                 \x20       // gives the internal junction voltage directly. Lets bjt_evaluate\n\
                 \x20       // (intrinsic) replace the inner-NR cost of bjt_with_parasitics.\n",
            );
            code.push_str(&k_eff_stmts);
        }

        code.push_str(
            "\n        self.s = s;\n\
             \x20       self.a_neg = a_neg;\n\
             \x20       self.k = k;\n\
             \x20       self.s_ni = s_ni;\n",
        );

        // Rebuild the backward-Euler fallback matrix set from the same
        // g_eff/c_eff. Without this, s_be/k_be/a_neg_be/s_ni_be go stale on
        // every pot/switch move — and permanently wrong after
        // set_sample_rate (which delegates here on pot/switch circuits, so
        // the no-pot template's BE rebuild never runs). The BE fallback
        // fires on exactly the stressed samples, so a stale BE set corrupts
        // the samples that most need it. Mirrors the no-pot
        // set_sample_rate body in state.rs.tera.
        let has_be_fallback = !ir.matrices.s_be.is_empty() && ir.topology.m > 0;
        if has_be_fallback {
            code.push_str(
                "\n        // Rebuild backward-Euler fallback matrices (stale BE set\n\
                 \x20       // would corrupt exactly the stressed samples that trigger\n\
                 \x20       // the fallback)\n\
                 \x20       let alpha_be = internal_rate; // 1/T for backward Euler\n\
                 \x20       let mut a_be = [[0.0f64; N]; N];\n\
                 \x20       for i in 0..N {\n\
                 \x20           for j in 0..N {\n\
                 \x20               a_be[i][j] = g_eff[i][j] + alpha_be * c_eff[i][j];\n\
                 \x20           }\n\
                 \x20       }\n\
                 \x20       let mut a_neg_be = [[0.0f64; N]; N];\n\
                 \x20       for i in 0..N {\n\
                 \x20           for j in 0..N {\n\
                 \x20               a_neg_be[i][j] = alpha_be * c_eff[i][j];\n\
                 \x20           }\n\
                 \x20       }\n",
            );
            if n_nodes < a_neg_zero_end {
                code.push_str(&format!(
                    "        // Zero VS/VCVS algebraic rows in A_neg_be (NOT inductor rows)\n\
                     \x20       for i in {n_nodes}..{a_neg_zero_end} {{\n\
                     \x20           for j in 0..N {{\n\
                     \x20               a_neg_be[i][j] = 0.0;\n\
                     \x20           }}\n\
                     \x20       }}\n"
                ));
            }
            code.push_str(
                "        let (s_be, singular_be) = invert_n_equilibrated(a_be);\n\
                 \x20       if singular_be { self.diag_singular_matrix_count += 1; }\n\
                 \x20       let mut s_ni_be = [[0.0f64; M]; N];\n\
                 \x20       for i in 0..N {\n\
                 \x20           for j in 0..M {\n\
                 \x20               let mut sum = 0.0;\n\
                 \x20               for kk in 0..N {\n\
                 \x20                   sum += s_be[i][kk] * N_I[kk][j];\n\
                 \x20               }\n\
                 \x20               s_ni_be[i][j] = sum;\n\
                 \x20           }\n\
                 \x20       }\n\
                 \x20       let mut k_be = [[0.0f64; M]; M];\n\
                 \x20       for i in 0..M {\n\
                 \x20           for j in 0..M {\n\
                 \x20               let mut sum = 0.0;\n\
                 \x20               for n_idx in 0..N {\n\
                 \x20                   sum += N_V[i][n_idx] * s_ni_be[n_idx][j];\n\
                 \x20               }\n\
                 \x20               k_be[i][j] = sum;\n\
                 \x20           }\n\
                 \x20       }\n",
            );
            // K_eff: same parasitic-BJT absorption as the trap K above —
            // K_BE_DEFAULT is emitted with this adjustment, so the rebuild
            // must apply it too.
            let k_be_eff_stmts = k_eff_adjust_stmts(ir, "k_be", "        ");
            if !k_be_eff_stmts.is_empty() {
                code.push_str(
                    "        // K_eff for the BE kernel (same parasitic-BJT absorption as trap K)\n",
                );
                code.push_str(&k_be_eff_stmts);
            }
            code.push_str(
                "        self.s_be = s_be;\n\
                 \x20       self.k_be = k_be;\n\
                 \x20       self.s_ni_be = s_ni_be;\n\
                 \x20       self.a_neg_be = a_neg_be;\n",
            );
        }

        // SM pot recomputation removed — per-block rebuild handles pots exactly

        // Intentionally do NOT reset oversampler state or DC blocker state here.
        // rebuild_matrices() is called on every pot/switch change, but pot/switch
        // changes do not invalidate filter history. Zeroing os_up_state/os_dn_state
        // on every knob move causes an audible click from the half-band filter
        // ringing through; zeroing dc_block_x_prev/y_prev produces an instant DC
        // step and multi-second HPF re-settle. The `dc_block_r` coefficient depends
        // only on the sample rate, not on pots/switches, so it is not recomputed
        // here — set_sample_rate recomputes it on a genuine rate change (in BOTH
        // the pot/switch and no-pot template variants) and restores DC_BLOCK_R on
        // same-rate calls. (Legitimate filter resets happen only on a genuine
        // rate change in set_sample_rate(), plus reset()'s blocker reseed.)

        code.push_str("    }\n");
        code
    }

    fn emit_pot_constants(&self, ir: &CircuitIR) -> String {
        if ir.pots.is_empty() {
            return String::new();
        }
        let mut code = section_banner("POTENTIOMETER CONSTANTS");

        for (idx, pot) in ir.pots.iter().enumerate() {
            // G_NOM: nominal conductance for rebuild_matrices delta computation
            code.push_str(&format!(
                "const POT_{}_G_NOM: f64 = {};\n",
                idx,
                fmt_f64(pot.g_nominal)
            ));
            code.push_str(&format!(
                "const POT_{}_MIN_R: f64 = {};\n",
                idx,
                fmt_f64(pot.min_resistance)
            ));
            code.push_str(&format!(
                "const POT_{}_MAX_R: f64 = {};\n",
                idx,
                fmt_f64(pot.max_resistance)
            ));
            // For .runtime R entries, also emit discoverable public aliases
            // keyed on the runtime field name so plugin code can read the
            // clamp range without knowing the pot index.
            if let Some(field) = &pot.runtime_field {
                let u = field.to_ascii_uppercase();
                code.push_str(&format!(
                    "pub const RUNTIME_R_{u}_MIN: f64 = POT_{}_MIN_R;\n",
                    idx
                ));
                code.push_str(&format!(
                    "pub const RUNTIME_R_{u}_MAX: f64 = POT_{}_MAX_R;\n",
                    idx
                ));
                code.push_str(&format!(
                    "pub const RUNTIME_R_{u}_NOMINAL: f64 = 1.0 / POT_{}_G_NOM;\n",
                    idx
                ));
            }
            code.push('\n');
        }
        code
    }

    /// Emit inline Gaussian elimination inversion functions for transformer groups.
    ///
    /// Each transformer group of size W gets its own `invert_xfmr_{n}` function
    /// that inverts a W x W matrix. The size is known at codegen time so the
    /// function is fully unrolled.
    fn emit_transformer_group_helpers(ir: &CircuitIR) -> String {
        if ir.transformer_groups.is_empty() {
            return String::new();
        }
        let mut code = section_banner("TRANSFORMER GROUP INVERSION HELPERS");

        for (gi, g) in ir.transformer_groups.iter().enumerate() {
            let w = g.num_windings;
            code.push_str(&format!(
                "/// Invert a {w}x{w} matrix for transformer group {gi} using Gaussian elimination.\n\
                 #[inline(always)]\n\
                 fn invert_xfmr_{gi}(a: [[f64; {w}]; {w}]) -> [[f64; {w}]; {w}] {{\n\
                 \x20   let mut aug = [[0.0f64; {w2}]; {w}];\n\
                 \x20   for i in 0..{w} {{\n\
                 \x20       for j in 0..{w} {{\n\
                 \x20           aug[i][j] = a[i][j];\n\
                 \x20       }}\n\
                 \x20       aug[i][{w} + i] = 1.0;\n\
                 \x20   }}\n\n",
                w2 = w * 2,
            ));
            // Forward elimination with partial pivoting
            code.push_str(&format!(
                "    for col in 0..{w} {{\n\
                 \x20       let mut max_row = col;\n\
                 \x20       let mut max_val = aug[col][col].abs();\n\
                 \x20       for row in (col + 1)..{w} {{\n\
                 \x20           if aug[row][col].abs() > max_val {{\n\
                 \x20               max_val = aug[row][col].abs();\n\
                 \x20               max_row = row;\n\
                 \x20           }}\n\
                 \x20       }}\n\
                 \x20       if max_val < 1e-30 {{\n\
                 \x20           let mut result = [[0.0f64; {w}]; {w}];\n\
                 \x20           for i in 0..{w} {{ result[i][i] = 1.0; }}\n\
                 \x20           return result;\n\
                 \x20       }}\n\
                 \x20       if max_row != col {{ aug.swap(col, max_row); }}\n\
                 \x20       let pivot = aug[col][col];\n\
                 \x20       for row in (col + 1)..{w} {{\n\
                 \x20           let factor = aug[row][col] / pivot;\n\
                 \x20           for j in col..{w2} {{\n\
                 \x20               aug[row][j] -= factor * aug[col][j];\n\
                 \x20           }}\n\
                 \x20       }}\n\
                 \x20   }}\n\n",
                w2 = w * 2,
            ));
            // Back-substitution
            code.push_str(&format!(
                "    for col in (0..{w}).rev() {{\n\
                 \x20       let pivot = aug[col][col];\n\
                 \x20       if pivot.abs() < 1e-30 {{\n\
                 \x20           let mut result = [[0.0f64; {w}]; {w}];\n\
                 \x20           for i in 0..{w} {{ result[i][i] = 1.0; }}\n\
                 \x20           return result;\n\
                 \x20       }}\n\
                 \x20       for j in 0..{w2} {{ aug[col][j] /= pivot; }}\n\
                 \x20       for row in 0..col {{\n\
                 \x20           let factor = aug[row][col];\n\
                 \x20           for j in 0..{w2} {{ aug[row][j] -= factor * aug[col][j]; }}\n\
                 \x20       }}\n\
                 \x20   }}\n\n",
                w2 = w * 2,
            ));
            // Extract result
            code.push_str(&format!(
                "    let mut result = [[0.0f64; {w}]; {w}];\n\
                 \x20   for i in 0..{w} {{\n\
                 \x20       for j in 0..{w} {{\n\
                 \x20           result[i][j] = aug[i][{w} + j];\n\
                 \x20       }}\n\
                 \x20   }}\n\
                 \x20   result\n\
                 }}\n\n",
            ));
        }
        code
    }

    fn emit_build_rhs(
        &self,
        ir: &CircuitIR,
        _noise: &NoiseEmission,
    ) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("has_dc_sources", &ir.has_dc_sources);
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("backward_euler", &ir.solver_config.backward_euler);
        // Runtime voltage sources: emit `rhs[row] += state.<field>` per entry
        // after the input stamp. Safe on both trapezoidal and BE paths — the
        // field is the raw per-sample value, not rate-dependent.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // When augmented_inductors is true, companion model history is handled by A_neg,
        // so num_inductors/num_coupled_inductors/num_transformer_groups should be 0
        // for the build_rhs template (no history current injection).
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            let mut xfmr_rhs_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                for k in 0..g.num_windings {
                    if g.winding_node_i[k] > 0 {
                        xfmr_rhs_lines.push_str(&format!(
                            "    rhs[{}] -= state.xfmr_{}_i_hist[{}];\n",
                            g.winding_node_i[k] - 1,
                            gi,
                            k
                        ));
                    }
                    if g.winding_node_j[k] > 0 {
                        xfmr_rhs_lines.push_str(&format!(
                            "    rhs[{}] += state.xfmr_{}_i_hist[{}];\n",
                            g.winding_node_j[k] - 1,
                            gi,
                            k
                        ));
                    }
                }
            }
            ctx.insert("xfmr_rhs_lines", &xfmr_rhs_lines);
        }

        // A_neg * v_prev lines (using pre-analyzed sparsity)
        let assign_op = if ir.has_dc_sources { "+=" } else { "=" };
        let mut a_neg_lines = String::new();
        for i in 0..n {
            let nz_cols = &ir.sparsity.a_neg.nz_by_row[i];
            if nz_cols.is_empty() {
                if !ir.has_dc_sources {
                    a_neg_lines.push_str(&format!("    rhs[{}] = 0.0;\n", i));
                }
            } else {
                let terms: Vec<String> = nz_cols
                    .iter()
                    .map(|&j| format!("state.a_neg[{}][{}] * state.v_prev[{}]", i, j, j))
                    .collect();
                a_neg_lines.push_str(&format!(
                    "    rhs[{}] {} {};\n",
                    i,
                    assign_op,
                    terms.join(" + ")
                ));
            }
        }
        ctx.insert("a_neg_lines", &a_neg_lines);

        // N_i * i_nl_prev lines (using pre-analyzed sparsity).
        //
        // Trap rule splits the trap-averaged nonlinear stamp
        // `S * N_i * (i_nl_n + i_nl_prev) / 2` into a v_pred contribution
        // (`S * N_i * i_nl_prev`, baked here into RHS) and a v_correction
        // (`S_NI * i_nl_n`, applied in compute_final_voltages). The two
        // halves combine to the correct trap average.
        //
        // Backward Euler does NOT trap-average — its nonlinear stamp is
        // just `S_BE * N_i * i_nl_n` at the current sample. Including
        // `N_I * i_nl_prev` in the BE RHS adds a spurious `S_BE * N_i *
        // i_nl_prev` term that breaks the BE fixed point: at v_prev =
        // DC_OP and i_nl_prev = DC_NL_I the next sample drifts by
        // `S_BE * N_i * DC_NL_I` (nonzero whenever any device carries DC
        // bias current). On high-gain cap-coupled cascades this seeds
        // a multi-second clipped startup transient that has no physical
        // basis. Gate on `!backward_euler` so BE skips the term entirely.
        let has_nl_prev = m > 0 && !ir.solver_config.backward_euler;
        ctx.insert("has_nl_prev", &has_nl_prev);
        if has_nl_prev {
            let mut nl_prev_lines = String::new();
            for i in 0..n {
                for &j in &ir.sparsity.n_i.nz_by_row[i] {
                    nl_prev_lines.push_str(&format!(
                        "    rhs[{}] += N_I[{}][{}] * state.i_nl_prev[{}];\n",
                        i, i, j, j
                    ));
                }
            }
            ctx.insert("nl_prev_lines", &nl_prev_lines);
        }

        self.render("build_rhs", &ctx)
    }

    fn emit_mat_vec_mul_s(&self, _ir: &CircuitIR) -> Result<String, CodegenError> {
        // The template now uses a runtime loop over state.s, no context needed
        self.render("mat_vec_mul_s", &Context::new())
    }

    fn emit_extract_voltages(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let m = ir.topology.m;
        let mut ctx = Context::new();

        // N_v extraction lines (using pre-analyzed sparsity)
        let mut extract_lines = String::new();
        for i in 0..m {
            extract_lines.push_str("        ");
            let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
            if nz_cols.is_empty() {
                extract_lines.push_str("0.0");
            } else {
                let mut first = true;
                for &j in nz_cols {
                    let coeff = ir.n_v(i, j);
                    let abs_val = coeff.abs();
                    let is_negative = coeff < 0.0;

                    if first {
                        if is_negative {
                            extract_lines.push('-');
                        }
                    } else if is_negative {
                        extract_lines.push_str(" - ");
                    } else {
                        extract_lines.push_str(" + ");
                    }

                    if (abs_val - 1.0).abs() < 1e-15 {
                        extract_lines.push_str(&format!("v_pred[{}]", j));
                    } else {
                        extract_lines.push_str(&format!("{} * v_pred[{}]", fmt_f64(abs_val), j));
                    }
                    first = false;
                }
            }
            extract_lines.push_str(",\n");
        }
        ctx.insert("extract_lines", &extract_lines);

        self.render("extract_voltages", &ctx)
    }

    fn emit_final_voltages(&self, _ir: &CircuitIR) -> Result<String, CodegenError> {
        // The template now uses a runtime loop over state.s_ni, no context needed
        self.render("final_voltages", &Context::new())
    }

    fn emit_update_history(&self) -> Result<String, CodegenError> {
        self.render("update_history", &Context::new())
    }

    fn emit_process_sample(
        &self,
        ir: &CircuitIR,
        noise: &NoiseEmission,
    ) -> Result<String, CodegenError> {
        // Noise fragment is built once by the caller (emit_dk) and threaded in
        // — build_noise_emission resolves every noise source and assembles ~500
        // lines of strings, so re-deriving it here doubled that codegen work.
        let mut ctx = Context::new();
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        ctx.insert("noise_enabled_emit", &noise.enabled);
        ctx.insert("noise_rhs_stamp", &noise.rhs_stamp);
        ctx.insert("noise_rhs_stamp_be", &noise.rhs_stamp_be);
        ctx.insert("noise_nan_recovery", &noise.nan_recovery_body);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            ir.topology.n
        };
        ctx.insert("n_nodes", &n_nodes);
        // BE fallback in process_sample: only for circuits NOT using auto-BE (which
        // already uses BE for ALL samples). Avoids changing output of well-conditioned
        // circuits where the fallback would trigger on legitimate transient overshoots.
        let has_be_fallback =
            !ir.matrices.s_be.is_empty() && ir.topology.m > 0 && !ir.solver_config.backward_euler;
        ctx.insert("has_be_fallback", &has_be_fallback);
        // #P1: sparse-prune the BE-fallback matvecs. rhs_be = A_neg_be·v_prev +
        // N_I·i_nl_prev and p_be = N_V·v_pred_be are all structurally sparse;
        // S_be / S_ni_be are dense inverses and stay looped in the template.
        // Byte-identical — skipped entries are exactly zero (A_neg_be uses its
        // OWN pattern, not a_neg's, to avoid the αC−G near-cancellation trap).
        let (be_rhs_lines, be_p_lines) = if has_be_fallback {
            let mut rhs = String::new();
            for i in 0..ir.topology.n {
                let mut terms: Vec<String> = Vec::new();
                if ir.has_dc_sources {
                    terms.push(format!("RHS_CONST_BE[{i}]"));
                }
                for &j in &ir.sparsity.a_neg_be.nz_by_row[i] {
                    terms.push(format!("state.a_neg_be[{i}][{j}] * state.v_prev[{j}]"));
                }
                for &j in &ir.sparsity.n_i.nz_by_row[i] {
                    terms.push(format!("N_I[{i}][{j}] * state.i_nl_prev[{j}]"));
                }
                if terms.is_empty() {
                    rhs.push_str(&format!("        rhs_be[{i}] = 0.0;\n"));
                } else {
                    rhs.push_str(&format!("        rhs_be[{i}] = {};\n", terms.join(" + ")));
                }
            }
            let mut p = String::new();
            for i in 0..ir.topology.m {
                let terms: Vec<String> = ir.sparsity.n_v.nz_by_row[i]
                    .iter()
                    .map(|&j| format!("N_V[{i}][{j}] * v_pred_be[{j}]"))
                    .collect();
                if terms.is_empty() {
                    p.push_str(&format!("        p_be[{i}] = 0.0;\n"));
                } else {
                    p.push_str(&format!("        p_be[{i}] = {};\n", terms.join(" + ")));
                }
            }
            (rhs, p)
        } else {
            (String::new(), String::new())
        };
        ctx.insert("be_rhs_lines", &be_rhs_lines);
        ctx.insert("be_p_lines", &be_p_lines);
        ctx.insert("has_dc_sources", &ir.has_dc_sources);
        ctx.insert("max_iter", &ir.solver_config.max_iterations);
        // V_MAX_DC: maximum physically reasonable node voltage (supply rails + margin).
        // Used by BE fallback to detect trapezoidal ringing artifacts.
        let v_max_dc = ir
            .dc_operating_point
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max)
            .max(1.0)
            * 3.0
            + 10.0; // 3× margin + 10V headroom (generous for transients)
        ctx.insert("v_max_dc", &format!("{:.17e}", v_max_dc));
        // When augmented_inductors is true, companion model state update is not needed —
        // A_neg handles all inductor history through the augmented G/C matrices.
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            // Generate transformer group state update code procedurally
            let mut xfmr_update_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                xfmr_update_lines.push_str("    {\n");
                // Extract winding voltages
                for k in 0..w {
                    let v_i = if g.winding_node_i[k] > 0 {
                        format!("v[{}]", g.winding_node_i[k] - 1)
                    } else {
                        "0.0".to_string()
                    };
                    let v_j = if g.winding_node_j[k] > 0 {
                        format!("v[{}]", g.winding_node_j[k] - 1)
                    } else {
                        "0.0".to_string()
                    };
                    xfmr_update_lines
                        .push_str(&format!("        let v_new_{k} = {v_i} - {v_j};\n"));
                }
                // Compute new currents: i_new[k] = i_prev[k] + sum_j Y[k][j] * (v_prev[j] + v_new[j])
                for k in 0..w {
                    xfmr_update_lines.push_str(&format!(
                        "        let i_new_{k} = state.xfmr_{gi}_i_prev[{k}]"
                    ));
                    for j in 0..w {
                        xfmr_update_lines.push_str(&format!(
                            " + state.xfmr_{gi}_y[{}] * (state.xfmr_{gi}_v_prev[{j}] + v_new_{j})",
                            k * w + j,
                        ));
                    }
                    xfmr_update_lines.push_str(";\n");
                }
                // Doubled-trapezoidal history: i_hist[k] = 2 * i_new[k].
                // The Y*(v[n]+v[n-1]) part of i[n]+i[n-1] is already carried
                // by the admittance stamps in A / A_neg (mirrors
                // DkKernel::update_transformer_groups).
                for k in 0..w {
                    xfmr_update_lines.push_str(&format!(
                        "        state.xfmr_{gi}_i_hist[{k}] = 2.0 * i_new_{k};\n"
                    ));
                }
                // Update i_prev and v_prev
                for k in 0..w {
                    xfmr_update_lines.push_str(&format!(
                        "        state.xfmr_{gi}_i_prev[{k}] = i_new_{k};\n\
                         \x20       state.xfmr_{gi}_v_prev[{k}] = v_new_{k};\n"
                    ));
                }
                xfmr_update_lines.push_str("    }\n");
            }
            ctx.insert("xfmr_update_lines", &xfmr_update_lines);

            // Generate NaN reset lines
            let mut xfmr_nan_reset_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                xfmr_nan_reset_lines.push_str(&format!(
                    "        state.xfmr_{gi}_i_prev = [0.0; {}];\n\
                     \x20       state.xfmr_{gi}_v_prev = [0.0; {}];\n\
                     \x20       state.xfmr_{gi}_i_hist = [0.0; {}];\n",
                    g.num_windings, g.num_windings, g.num_windings,
                ));
            }
            ctx.insert("xfmr_nan_reset_lines", &xfmr_nan_reset_lines);
        }

        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            ctx.insert("os_state_size", &os_info.state_size);
            ctx.insert("oversampling_4x", &(os_factor == 4));
            if os_factor == 4 {
                ctx.insert("os_state_size_outer", &os_info.state_size_outer);
            }
        } else {
            ctx.insert("oversampling_4x", &false);
        }

        // DC nonlinear currents availability (for NaN reset path)
        let has_dc_nl = ir.topology.m > 0
            && !ir.dc_nl_currents.is_empty()
            && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
        ctx.insert("has_dc_nl", &has_dc_nl);

        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);

        // Output clamp bound (CodegenConfig::output_clamp_v, default ±10 V).
        // Used for the final output clamp, the diag_clamp_count threshold,
        // and the NaN-recovery return path. High-voltage circuits (e.g. the
        // Wurlitzer power amp at ±30 V) set this above the default; the
        // nodal emitter already honors it (nodal_emitter.rs) — this threads
        // it into the DK template as well.
        ctx.insert(
            "output_clamp_v",
            &format!("{:e}", ir.solver_config.output_clamp_v),
        );

        ctx.insert("max_iter", &ir.solver_config.max_iterations);
        ctx.insert("m", &ir.topology.m);

        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);

        // Runtime voltage sources (.runtime directive): the BE-fallback RHS
        // in process_sample.rs.tera stamps the same rows/values as build_rhs
        // (VS algebraic rows are integration-scheme-independent).
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // Per-sample SM pot corrections removed — pot changes are handled by
        // per-block rebuild_matrices (Batch D). No sm_scale_lines /
        // a_neg_correction / s_correction / sni_correction context vars are
        // emitted; the templates never referenced them.

        // MOSFET body effect: compute VT_eff from v_pred before NR (DK path only)
        let mut body_effect_update = String::new();
        if ir.solver_mode == crate::codegen::ir::SolverMode::Dk {
            for (dev_num, slot) in ir.device_slots.iter().enumerate() {
                if let DeviceParams::Mosfet(mp) = &slot.params {
                    if mp.has_body_effect() {
                        // Extract Vsb from v_pred (node indices are 1-based; 0 = ground)
                        let vs_expr = if mp.source_node > 0 {
                            format!("v_pred[{}]", mp.source_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let vb_expr = if mp.bulk_node > 0 {
                            format!("v_pred[{}]", mp.bulk_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                        // The body-effect shift must be applied in the
                        // channel's effective (magnitude) space: the stored VT
                        // is signed (negative for PMOS — device_mosfet.rs.tera
                        // does vt_eff = sign * vt), so the GAMMA term is
                        // multiplied by the channel sign. Reverse body bias
                        // then always *increases* |VT|; without the sign a
                        // PMOS |VT| would shrink instead.
                        body_effect_update.push_str(&format!(
                                "    {{ // MOSFET {dev_num} body effect\n\
                                 \x20       let vsb = ({sign:.1}) * ({vs_expr} - {vb_expr});\n\
                                 \x20       state.device_{dev_num}_vt = DEVICE_{dev_num}_VT + ({sign:.1}) * DEVICE_{dev_num}_GAMMA * ((DEVICE_{dev_num}_PHI + vsb.max(0.0)).sqrt() - DEVICE_{dev_num}_PHI.sqrt());\n\
                                 \x20   }}\n"
                            ));
                    }
                }
            }
        }
        if !body_effect_update.is_empty() {
            ctx.insert("body_effect_update", &body_effect_update);
        }

        // Device self-heating thermal update (after NR, before state save).
        // Runs for each BJT/diode whose `.model` sets a finite RTH. Uses the
        // converged `i_nl` and node voltages from this sample to compute P
        // and advances Tj by one exact-exponential step of the RC thermal
        // ODE (see `emit_thermal_tj_advance`).
        let mut thermal_update = String::new();
        // #P4: extract_controlling_voltages(&v) (= N_v·v) is identical for every
        // self-heating device — a function of the converged v only. Emit it once
        // here rather than re-extracting inside each device's thermal block.
        let has_self_heating = ir.device_slots.iter().any(|slot| match &slot.params {
            DeviceParams::Bjt(bp) => bp.has_self_heating() && slot.device_type == DeviceType::Bjt,
            DeviceParams::Diode(dp) => dp.has_self_heating(),
            DeviceParams::Tube(tp) => tp.has_self_heating(),
            _ => false,
        });
        if has_self_heating {
            thermal_update.push_str("    let v_nl_th = extract_controlling_voltages(&v);\n");
        }
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                // The `device_type == Bjt` guard (not BjtForwardActive) is
                // belt-and-braces against slot aliasing: this arm reads the
                // 2D slot pair (i_nl[s], i_nl[s+1]); on a 1D FA-reduced slot
                // s+1 would be the NEXT device's slot (or out of bounds).
                // detect_forward_active_bjts excludes self-heating BJTs from
                // FA reduction, so this guard should never fire — but if that
                // gating ever regresses, aliasing stays structurally
                // impossible here.
                DeviceParams::Bjt(bp)
                    if bp.has_self_heating() && slot.device_type == DeviceType::Bjt =>
                {
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    let tj_advance = emit_thermal_tj_advance(dev_num, bp.cth);
                    // Extract Ic, Ib from converged i_nl; compute Vbe, Vbc from final v
                    thermal_update.push_str(&format!(
                            "    {{ // BJT {dev_num} self-heating thermal update\n\
                             \x20       let ic = i_nl[{s}];\n\
                             \x20       let ib = i_nl[{s1}];\n\
                             \x20       let vbe = v_nl_th[{s}];\n\
                             \x20       let vbc = v_nl_th[{s1}];\n\
                             \x20       let vce = vbe - vbc;\n\
                             \x20       let p = vce * ic + vbe * ib;\n\
                             {tj_advance}\
                             \x20       state.device_{dev_num}_vt = BOLTZMANN_Q * state.device_{dev_num}_tj;\n\
                             \x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                             \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                             \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                             \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI)\n\
                             \x20           * fast_exp((DEVICE_{dev_num}_EG / vt_nom) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                             \x20       // Beta temperature dependence (SPICE XTB). XTB defaults to 0.0,\n\
                             \x20       // so `powf` returns exactly 1.0 and this is inert on cards that\n\
                             \x20       // omit it — but a self-heating device whose beta never moved was\n\
                             \x20       // physically wrong, not merely incomplete.\n\
                             \x20       state.device_{dev_num}_bf = DEVICE_{dev_num}_BETA_F * t_ratio.powf(DEVICE_{dev_num}_XTB);\n\
                             \x20       state.device_{dev_num}_br = DEVICE_{dev_num}_BETA_R * t_ratio.powf(DEVICE_{dev_num}_XTB);\n\
                             \x20   }}\n"
                        ));
                }
                DeviceParams::Diode(dp) if dp.has_self_heating() => {
                    let s = slot.start_idx;
                    let tj_advance = emit_thermal_tj_advance(dev_num, dp.cth);
                    // SPICE3f5 diode temperature scaling includes the emission
                    // coefficient N:
                    //   IS(T) = IS_NOM · (Tj/TAMB)^(XTI/N)
                    //                  · exp((EG/(N·vt_nom)) · (1 − TAMB/Tj))
                    // N is not stored separately in DiodeParams — recover it
                    // from n_vt = N · VT_ROOM (ir/mod.rs resolve_diode_params)
                    // and bake it into the emitted expression at codegen time.
                    let n_emission = dp.n_vt / melange_primitives::VT_ROOM;
                    thermal_update.push_str(&format!(
                            "    {{ // Diode {dev_num} self-heating thermal update\n\
                             \x20       let id = i_nl[{s}];\n\
                             \x20       let vd = v_nl_th[{s}];\n\
                             \x20       let p = vd * id;\n\
                             {tj_advance}\
                             \x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                             \x20       state.device_{dev_num}_n_vt = DEVICE_{dev_num}_N_VT_NOM * t_ratio;\n\
                             \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                             \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                             \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI / {n_emission:.17e})\n\
                             \x20           * fast_exp((DEVICE_{dev_num}_EG / ({n_emission:.17e} * vt_nom)) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                             \x20   }}\n"
                        ));
                }
                DeviceParams::Tube(tp) if tp.has_self_heating() => {
                    // Sharp-cutoff triode only in phase 1 (gated by
                    // TubeParams::has_self_heating). Pdiss ≈ Ip·Vpk + Ig·Vgk;
                    // the Ig·Vgk term is tiny in normal Class-A operation
                    // (nA to µA Ig, mV Vgk) but carried through for
                    // correctness under grid-current clip. No IS(T) /
                    // VT(T) drift — the Vgk bias shift lives at the NR call
                    // site in `nr_helpers.rs`, not here.
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    let tj_advance = emit_thermal_tj_advance(dev_num, tp.cth);
                    thermal_update.push_str(&format!(
                        "    {{ // Triode {dev_num} self-heating thermal update\n\
                             \x20       let ip = i_nl[{s}];\n\
                             \x20       let ig = i_nl[{s1}];\n\
                             \x20       let vgk = v_nl_th[{s}];\n\
                             \x20       let vpk = v_nl_th[{s1}];\n\
                             \x20       let p = ip * vpk + ig * vgk;\n\
                             {tj_advance}\
                             \x20   }}\n"
                    ));
                }
                _ => {}
            }
        }
        if !thermal_update.is_empty() {
            ctx.insert("thermal_update", &thermal_update);
            let thermal_devices = self_heating_device_data(ir);
            ctx.insert("num_thermal_devices", &thermal_devices.len());
            ctx.insert("thermal_devices", &thermal_devices);
        } else {
            ctx.insert("num_thermal_devices", &0usize);
        }

        // Stateful-device (Phase 0c) after-solve update + NaN-recovery restore.
        // Both come from the shared helpers so DK and nodal cannot drift. Only
        // inserted when non-empty; the template guards on `is defined and != ""`
        // so a non-stateful deck is byte-identical.
        let stateful_devs = stateful_device_data(ir);
        let stateful_update = emit_stateful_update(&stateful_devs);
        if !stateful_update.is_empty() {
            ctx.insert("stateful_update", &stateful_update);
        }
        let stateful_nan_recovery = emit_stateful_state_restore(&stateful_devs, "state.");
        if !stateful_nan_recovery.is_empty() {
            ctx.insert("stateful_nan_recovery", &stateful_nan_recovery);
        }

        ctx.insert("dc_block", &ir.dc_block);

        // Op-amp supply rail clamping data for DK template. Skip op-amps
        // whose VCC/VEE are both infinite — those entries exist in
        // `ir.opamps` only because `sr` is finite (slew limiting is handled
        // via a separate `opamp_slew` context block below).
        //
        // Rail-mode consumption (DK path): `--opamp-rail-mode none` suppresses
        // the clamp vars entirely; every other mode emits the Hard post-NR
        // clamp (ActiveSet/ActiveSetBe degrade to Hard+BE-fallback here — a
        // one-shot runtime warning is emitted from the generated Default,
        // see emit_state).
        if !ir.opamps.is_empty()
            && ir.solver_config.opamp_rail_mode != crate::codegen::OpampRailMode::None
        {
            let opamp_clamps: Vec<std::collections::HashMap<&str, String>> = ir
                .opamps
                .iter()
                .filter(|oa| oa.vclamp_lo.is_finite() || oa.vclamp_hi.is_finite())
                .map(|oa| {
                    let mut m = std::collections::HashMap::new();
                    // Single-sided supplies (only one of VCC/VEE given) leave
                    // the other bound infinite. `{:.17e}` renders f64 infinity
                    // as `inf` — not a valid Rust token — so emit the
                    // `f64::INFINITY` const path instead. `x.clamp(f64::NEG_INFINITY, hi)`
                    // behaves exactly as the one-sided `.min(hi)` (and
                    // symmetrically for a missing hi bound).
                    let fmt_bound = |v: f64| {
                        if v.is_finite() {
                            format!("{v:.17e}")
                        } else if v > 0.0 {
                            "f64::INFINITY".to_string()
                        } else {
                            "f64::NEG_INFINITY".to_string()
                        }
                    };
                    m.insert("out_idx", oa.n_out_idx.to_string());
                    m.insert("lo", fmt_bound(oa.vclamp_lo));
                    m.insert("hi", fmt_bound(oa.vclamp_hi));
                    m
                })
                .collect();
            if !opamp_clamps.is_empty() {
                ctx.insert("opamp_clamps", &opamp_clamps);
            }
        }

        // Op-amp slew-rate limiting data for DK template. Entries are
        // emitted as a per-sample voltage-delta clamp on the output node,
        // applied before the state update. Only op-amps with finite `sr`
        // contribute; when all op-amps have infinite SR the Tera template
        // emits no slew code at all, keeping generated output byte-
        // identical to the pre-slew behaviour.
        if !ir.opamps.is_empty() {
            let opamp_slew: Vec<std::collections::HashMap<&str, String>> = ir
                .opamps
                .iter()
                .enumerate()
                .filter(|(_, oa)| oa.sr.is_finite())
                .map(|(idx, oa)| {
                    let mut m = std::collections::HashMap::new();
                    m.insert("idx", idx.to_string());
                    m.insert("out_idx", oa.n_out_idx.to_string());
                    m.insert("sr", format!("{:.17e}", oa.sr));
                    m
                })
                .collect();
            if !opamp_slew.is_empty() {
                ctx.insert("opamp_slew", &opamp_slew);
            }
        }

        self.render("process_sample", &ctx)
    }

    /// Emit oversampling wrapper: constants, allpass helper, halfband, and process_sample.
    pub(super) fn emit_oversampler(ir: &CircuitIR) -> String {
        let factor = ir.solver_config.oversampling_factor;
        let info = oversampling_info(factor);
        let mut code = section_banner("OVERSAMPLING");

        // Emit coefficients as constants
        code.push_str("/// Half-band filter coefficients for allpass polyphase oversampler.\n");
        let coeffs_str = info
            .coeffs
            .iter()
            .map(|c| fmt_f64(*c))
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!(
            "const OS_COEFFS: [f64; {}] = [{}];\n\n",
            info.num_sections, coeffs_str
        ));

        if factor == 4 {
            let coeffs_outer_str = info
                .coeffs_outer
                .iter()
                .map(|c| fmt_f64(*c))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "const OS_COEFFS_OUTER: [f64; {}] = [{}];\n\n",
                info.num_sections_outer, coeffs_outer_str
            ));
        }

        // Emit allpass inline function (takes slice + offset to avoid double &mut borrow)
        code.push_str(
            "/// First-order allpass section: y = c*(x - y1) + x1\n\
             /// State layout: state[base] = x1, state[base+1] = y1\n\
             #[inline(always)]\n\
             fn os_allpass(x: f64, c: f64, state: &mut [f64], base: usize) -> f64 {\n\
             \x20   let y = c * x + state[base] - c * state[base + 1];\n\
             \x20   state[base] = x;\n\
             \x20   state[base + 1] = y;\n\
             \x20   y\n\
             }\n\n",
        );

        // Emit polyphase interpolator + decimator functions.
        // Upsampler and downsampler use SEPARATE state arrays; each branch of
        // each filter is clocked exactly once per LOW-rate sample (polyphase).
        Self::emit_halfband_fn(&mut code, "os_halfband", &info.coeffs, info.state_size);
        Self::emit_halfband_down_fn(&mut code, "os_halfband_down", &info.coeffs, info.state_size);
        if factor == 4 {
            Self::emit_halfband_fn(
                &mut code,
                "os_halfband_outer",
                &info.coeffs_outer,
                info.state_size_outer,
            );
            Self::emit_halfband_down_fn(
                &mut code,
                "os_halfband_down_outer",
                &info.coeffs_outer,
                info.state_size_outer,
            );
        }

        let num_outputs = ir.solver_config.output_nodes.len();
        let inject_or_tap = ir.solver_config.has_inject_or_tap();

        // Emit the public process_sample wrapper
        code.push_str("/// Process a single audio sample through the circuit with oversampling.\n");
        code.push_str("///\n");
        code.push_str(&format!(
            "/// Runs the circuit at {}x the host sample rate to reduce aliasing.\n",
            factor
        ));
        if inject_or_tap {
            code.push_str(
                "///\n\
                 /// `injections_inner[k]` supplies the `.inject` values for internal sample `k`\n\
                 /// (already at the internal rate — routed straight to the inner solve, NOT\n\
                 /// through the anti-alias up-filter). Returns the decimated outputs plus the\n\
                 /// RAW per-inner-sample `.tap` node voltages `taps_inner` (un-decimated). In a\n\
                 /// feedback loop, `injections_inner` must be derived from a PRIOR sample's tap —\n\
                 /// this is >=1 sample of delay by construction. Inner-sample order: 2x [even,\n\
                 /// odd]; 4x [e0, o0, e1, o1].\n",
            );
        }
        code.push_str("#[inline]\n");
        if inject_or_tap {
            code.push_str(
                "pub fn process_sample(input: f64, injections_inner: &[[f64; NUM_INJECT]; OVERSAMPLING_FACTOR], state: &mut CircuitState) -> ([f64; NUM_OUTPUTS], [[f64; NUM_TAP]; OVERSAMPLING_FACTOR]) {\n",
            );
        } else {
            code.push_str(
                "pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n",
            );
        }
        code.push_str(
            "    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n\n",
        );

        if factor == 2 {
            Self::emit_2x_wrapper(
                &mut code,
                num_outputs,
                ir.dc_block,
                ir.solver_config.output_clamp_v,
                inject_or_tap,
            );
        } else if factor == 4 {
            Self::emit_4x_wrapper(
                &mut code,
                num_outputs,
                ir.dc_block,
                ir.solver_config.output_clamp_v,
                inject_or_tap,
            );
        }

        code.push_str("}\n\n");
        code
    }

    /// Emit the 1x public `process_sample` wrapper for a circuit with `.inject`
    /// / `.tap` but no oversampling. The inner body is emitted as a private
    /// `process_sample_inner` (the template does this whenever `inject_or_tap`),
    /// so this thin wrapper adapts the per-inner-sample array API (arity 1).
    pub(super) fn emit_inject_wrapper_1x(ir: &CircuitIR) -> String {
        debug_assert_eq!(ir.solver_config.oversampling_factor, 1);
        let mut code = String::new();
        code.push_str("/// Process a single audio sample through the circuit.\n");
        code.push_str(
            "///\n\
             /// `injections_inner[0]` supplies the `.inject` values for this sample. Returns\n\
             /// the outputs plus the RAW `.tap` node voltages `taps_inner[0]`. In a feedback\n\
             /// loop, `injections_inner` must be derived from a PRIOR sample's tap — this is\n\
             /// >=1 sample of delay by construction.\n",
        );
        code.push_str("#[inline]\n");
        code.push_str(
            "pub fn process_sample(input: f64, injections_inner: &[[f64; NUM_INJECT]; OVERSAMPLING_FACTOR], state: &mut CircuitState) -> ([f64; NUM_OUTPUTS], [[f64; NUM_TAP]; OVERSAMPLING_FACTOR]) {\n",
        );
        code.push_str(
            "    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n",
        );
        code.push_str(
            "    let (output, tap) = process_sample_inner(input, injections_inner[0], state);\n",
        );
        code.push_str("    (output, [tap])\n");
        code.push_str("}\n\n");
        code
    }

    /// Emit a polyphase half-band interpolator step: one low-rate input
    /// produces the two internal-rate samples `(out[2n], out[2n+1])` from the
    /// even (A0) and odd (A1) allpass branches. Each branch is clocked once
    /// per call.
    pub(super) fn emit_halfband_fn(
        code: &mut String,
        name: &str,
        coeffs: &[f64],
        state_size: usize,
    ) {
        let num_sections = coeffs.len();
        let even_count = num_sections.div_ceil(2);
        let odd_count = num_sections / 2;

        code.push_str(&format!(
            "/// Half-band interpolator step: (even, odd) = (out[2n], out[2n+1]).\n\
             /// Each allpass branch is clocked once per low-rate input sample.\n\
             #[inline(always)]\n\
             fn {name}(input: f64, coeffs: &[f64; {num_sections}], state: &mut [f64; {state_size}]) -> (f64, f64) {{\n"
        ));

        // Even chain: coefficients at indices 0, 2, 4, ...
        // State layout: even sections first, then odd sections
        code.push_str("    let mut even = input;\n");
        for i in 0..even_count {
            let coeff_idx = i * 2; // even-indexed coefficients
            let state_base = i * 2; // sequential state storage for even chain
            code.push_str(&format!(
                "    even = os_allpass(even, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        // Odd chain: coefficients at indices 1, 3, 5, ...
        code.push_str("    let mut odd = input;\n");
        let odd_state_offset = even_count * 2;
        for i in 0..odd_count {
            let coeff_idx = i * 2 + 1; // odd-indexed coefficients
            let state_base = odd_state_offset + i * 2;
            code.push_str(&format!(
                "    odd = os_allpass(odd, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        code.push_str("    (even, odd)\n");
        code.push_str("}\n\n");
    }

    /// Emit a polyphase half-band decimator step: a pair of internal-rate
    /// samples (`x0` earlier, `x1` later) produces one low-rate output.
    ///
    /// hiir convention (`Downsampler2x::process_sample`): the even (A0)
    /// branch filters the LATER sample, the odd (A1) branch the EARLIER
    /// sample; output is their average. Each branch is clocked exactly once
    /// per output sample — clocking a branch twice per output (the pre-2026-07
    /// bug) collapses the allpass cells to first-order in the internal-rate z
    /// and destroys the stopband entirely.
    pub(super) fn emit_halfband_down_fn(
        code: &mut String,
        name: &str,
        coeffs: &[f64],
        state_size: usize,
    ) {
        let num_sections = coeffs.len();
        let even_count = num_sections.div_ceil(2);
        let odd_count = num_sections / 2;

        code.push_str(&format!(
            "/// Half-band decimator step: y[n] = (A_even(x[2n+1]) + A_odd(x[2n])) / 2.\n\
             /// Each allpass branch is clocked once per low-rate output sample.\n\
             #[inline(always)]\n\
             fn {name}(x0: f64, x1: f64, coeffs: &[f64; {num_sections}], state: &mut [f64; {state_size}]) -> f64 {{\n"
        ));

        // Even chain (coefficients 0, 2, 4, ...) consumes the LATER sample.
        // State layout matches the interpolator: even sections first.
        code.push_str("    let mut even = x1;\n");
        for i in 0..even_count {
            let coeff_idx = i * 2;
            let state_base = i * 2;
            code.push_str(&format!(
                "    even = os_allpass(even, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        // Odd chain (coefficients 1, 3, 5, ...) consumes the EARLIER sample.
        code.push_str("    let mut odd = x0;\n");
        let odd_state_offset = even_count * 2;
        for i in 0..odd_count {
            let coeff_idx = i * 2 + 1;
            let state_base = odd_state_offset + i * 2;
            code.push_str(&format!(
                "    odd = os_allpass(odd, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        code.push_str("    (even + odd) * 0.5\n");
        code.push_str("}\n\n");
    }

    /// Emit the 2x oversampling wrapper body.
    pub(super) fn emit_2x_wrapper(
        code: &mut String,
        _num_outputs: usize,
        dc_block: bool,
        clamp_v: f64,
        inject_or_tap: bool,
    ) {
        // Upsample: polyphase interpolator, 1 input → 2 internal-rate samples
        code.push_str(
            "    // Upsample: interpolator produces (out[2n], out[2n+1]) at internal rate\n\
             \x20   let (up_even, up_odd) = os_halfband(input, &OS_COEFFS, &mut state.os_up_state);\n\n",
        );

        // Process both at 2x rate. Injections BYPASS the up-filter (already
        // inner-rate): injections_inner[0]=even, [1]=odd. Taps are raw.
        if inject_or_tap {
            code.push_str(
                "    // Process both samples at 2x rate (up_even is the earlier sample)\n\
                 \x20   let (out_even, tap_even) = process_sample_inner(up_even, injections_inner[0], state);\n\
                 \x20   let (out_odd, tap_odd) = process_sample_inner(up_odd, injections_inner[1], state);\n\n",
            );
        } else {
            code.push_str(
                "    // Process both samples at 2x rate (up_even is the earlier sample)\n\
                 \x20   let out_even = process_sample_inner(up_even, state);\n\
                 \x20   let out_odd = process_sample_inner(up_odd, state);\n\n",
            );
        }

        // Downsample per-output: ONE decimator step per output sample
        code.push_str("    // Downsample: per-output polyphase decimator, 2 samples → 1\n");
        code.push_str("    let mut result = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let v = os_halfband_down(out_even[out_idx], out_odd[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        if dc_block {
            code.push_str(&format!(
                "        result[out_idx] = if v.is_finite() {{ v.clamp(-{clamp_v:e}, {clamp_v:e}) }} else {{ 0.0 }};\n",
            ));
        } else {
            code.push_str("        result[out_idx] = if v.is_finite() { v } else { 0.0 };\n");
        }
        code.push_str("    }\n");
        if inject_or_tap {
            // Taps are RAW / un-decimated — one array per internal sample.
            code.push_str("    (result, [tap_even, tap_odd])\n");
        } else {
            code.push_str("    result\n");
        }
    }

    /// Emit the 4x oversampling wrapper body (cascaded 2x stages).
    pub(super) fn emit_4x_wrapper(
        code: &mut String,
        _num_outputs: usize,
        dc_block: bool,
        clamp_v: f64,
        inject_or_tap: bool,
    ) {
        // Outer upsample: 1 → 2 at 2x rate (steep base-Nyquist filter)
        code.push_str(
            "    // Outer upsample: 1 → 2 samples at 2x rate (steep filter)\n\
             \x20   let (outer_even, outer_odd) = os_halfband_outer(\n\
             \x20       input, &OS_COEFFS_OUTER, &mut state.os_up_state_outer,\n\
             \x20   );\n\n",
        );

        // Inner upsample + process for each outer sample. Injections BYPASS
        // both up-filters (already inner-rate): order [e0, o0, e1, o1]. Taps raw.
        if inject_or_tap {
            code.push_str(
                "    // Inner upsample + process: each 2x sample → 2 samples at 4x rate\n\
                 \x20   let (inner_e0, inner_o0) = os_halfband(outer_even, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let (proc_e0, tap_e0) = process_sample_inner(inner_e0, injections_inner[0], state);\n\
                 \x20   let (proc_o0, tap_o0) = process_sample_inner(inner_o0, injections_inner[1], state);\n\n",
            );
        } else {
            code.push_str(
                "    // Inner upsample + process: each 2x sample → 2 samples at 4x rate\n\
                 \x20   let (inner_e0, inner_o0) = os_halfband(outer_even, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let proc_e0 = process_sample_inner(inner_e0, state);\n\
                 \x20   let proc_o0 = process_sample_inner(inner_o0, state);\n\n",
            );
        }

        // Inner decimator per-output for first 2x pair (one step per pair)
        code.push_str("    let mut inner_out0 = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        inner_out0[out_idx] = os_halfband_down(proc_e0[out_idx], proc_o0[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("    }\n\n");

        // Second inner upsample + process pair
        if inject_or_tap {
            code.push_str(
                "    let (inner_e1, inner_o1) = os_halfband(outer_odd, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let (proc_e1, tap_e1) = process_sample_inner(inner_e1, injections_inner[2], state);\n\
                 \x20   let (proc_o1, tap_o1) = process_sample_inner(inner_o1, injections_inner[3], state);\n\n",
            );
        } else {
            code.push_str(
                "    let (inner_e1, inner_o1) = os_halfband(outer_odd, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let proc_e1 = process_sample_inner(inner_e1, state);\n\
                 \x20   let proc_o1 = process_sample_inner(inner_o1, state);\n\n",
            );
        }

        // Inner decimator per-output for second 2x pair
        code.push_str("    let mut inner_out1 = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        inner_out1[out_idx] = os_halfband_down(proc_e1[out_idx], proc_o1[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("    }\n\n");

        // Outer decimator per-output: 2 samples at 2x rate → 1 at host rate
        code.push_str("    // Outer downsample: per-output polyphase decimator (steep filter)\n");
        code.push_str("    let mut result = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let v = os_halfband_down_outer(\n");
        code.push_str("            inner_out0[out_idx], inner_out1[out_idx], &OS_COEFFS_OUTER, &mut state.os_dn_state_outer[out_idx],\n");
        code.push_str("        );\n");
        if dc_block {
            code.push_str(&format!(
                "        result[out_idx] = if v.is_finite() {{ v.clamp(-{clamp_v:e}, {clamp_v:e}) }} else {{ 0.0 }};\n",
            ));
        } else {
            code.push_str("        result[out_idx] = if v.is_finite() { v } else { 0.0 };\n");
        }
        code.push_str("    }\n");
        if inject_or_tap {
            // Taps are RAW / un-decimated — one array per internal sample,
            // in the same [e0, o0, e1, o1] order as injections_inner.
            code.push_str("    (result, [tap_e0, tap_o0, tap_e1, tap_o1])\n");
        } else {
            code.push_str("    result\n");
        }
    }
}

// ============================================================================
// Noise emission (Phase 1: Johnson-Nyquist thermal)
// ============================================================================

/// Per-family noise source counts, kept so the replay can be re-emitted at any
/// RHS-rebuild site instead of only at the BE fallback.
#[derive(Debug, Default, Clone, Copy)]
pub(super) struct NoiseReplayCounts {
    pub shot: usize,
    pub flicker: usize,
    pub r_flicker: usize,
    pub partition: usize,
    pub opamp: usize,
}

/// Emit the noise *replay*: re-stamp every source's cached `i_n` into `target`
/// without touching the RNG.
///
/// **Every from-scratch RHS rebuild inside a sample must emit this.** The draws
/// for the sample were already consumed by `rhs_stamp` when the primary RHS was
/// built, and the resulting per-source currents cached in `noise_*_last_i_n`. A
/// rebuild that omits the replay drops the sample's noise while the RNG stream
/// stays aligned — so the loss is inaudible to any determinism check and shows
/// up only as a noise-floor stutter correlated with whatever triggered the
/// rebuild. Drawing fresh values here instead would break determinism outright:
/// the same seed would give different audio depending on how many samples
/// happened to sub-step.
///
/// `base` is the indent of the `if state.noise_enabled {` line; the body is
/// indented 4 and 8 further.
pub(super) fn emit_noise_replay_body(
    counts: NoiseReplayCounts,
    target: &str,
    base: &str,
) -> String {
    let mut s = String::new();
    let i1 = format!("{base}    ");
    let i2 = format!("{base}        ");
    s.push_str(&format!("{base}if state.noise_enabled {{\n"));
    let two_terminal = |s: &mut String, present: bool, upper: &str, field: &str| {
        if !present {
            return;
        }
        s.push_str(&format!("{i1}for k in 0..NOISE_{upper}_N {{\n"));
        s.push_str(&format!("{i2}let i_n = state.noise_{field}_last_i_n[k];\n"));
        s.push_str(&format!("{i2}let ni = NOISE_{upper}_NODE_I[k];\n"));
        s.push_str(&format!("{i2}let nj = NOISE_{upper}_NODE_J[k];\n"));
        s.push_str(&format!("{i2}if ni > 0 {{ {target}[ni - 1] += i_n; }}\n"));
        s.push_str(&format!("{i2}if nj > 0 {{ {target}[nj - 1] -= i_n; }}\n"));
        s.push_str(&format!("{i1}}}\n"));
    };
    two_terminal(&mut s, true, "THERMAL", "thermal");
    two_terminal(&mut s, counts.shot > 0, "SHOT", "shot");
    two_terminal(&mut s, counts.flicker > 0, "FLICKER", "flicker");
    two_terminal(&mut s, counts.r_flicker > 0, "R_FLICKER", "r_flicker");
    two_terminal(&mut s, counts.partition > 0, "PARTITION", "partition");
    if counts.opamp > 0 {
        // Op-amp en/in replay: stamp each cached current at its single input
        // node (single-sided — en is voltage-source-to-ground, in is
        // current-source-to-ground). No counter-stamp needed because the
        // "other terminal" of each source is ground, not a circuit node.
        s.push_str(&format!("{i1}for k in 0..NOISE_OPAMP_N {{\n"));
        s.push_str(&format!("{i2}let np = NOISE_OPAMP_NODE_PLUS[k];\n"));
        s.push_str(&format!("{i2}let nm = NOISE_OPAMP_NODE_MINUS[k];\n"));
        s.push_str(&format!(
            "{i2}let i_en = state.noise_opamp_en_last_i_n[k];\n"
        ));
        s.push_str(&format!(
            "{i2}let i_in_p = state.noise_opamp_in_last_i_n[2 * k];\n"
        ));
        s.push_str(&format!(
            "{i2}let i_in_m = state.noise_opamp_in_last_i_n[2 * k + 1];\n"
        ));
        s.push_str(&format!(
            "{i2}if np > 0 {{ {target}[np - 1] += i_en + i_in_p; }}\n"
        ));
        s.push_str(&format!(
            "{i2}if nm > 0 {{ {target}[nm - 1] += i_in_m; }}\n"
        ));
        s.push_str(&format!("{i1}}}\n"));
    }
    s.push_str(&format!("{base}}}\n"));
    s
}

/// All code fragments produced for authentic circuit noise.
///
/// When the IR's noise mode is `Off` or no eligible sources are present,
/// every field is the empty string and no emitted Rust token differs from a
/// noiseless build. Fragments are injected at specific points in the
/// emitted file by the DK and nodal paths.
#[derive(Debug, Default)]
pub(super) struct NoiseEmission {
    /// Self-contained block: constants (K_B, T_ROOM_K, NOISE_THERMAL_N, …),
    /// RNG struct, SplitMix64, Marsaglia polar Gaussian helper. Emitted once
    /// between `emit_constants` and `emit_state`.
    pub top_level: String,
    /// Struct-field declarations for `CircuitState` (inside the struct body).
    pub state_fields: String,
    /// Initialiser expressions for `Default::default()` (the statements come
    /// first, the `self` fields come as trailing `field: value,` assignments).
    pub default_stmts: String,
    pub default_fields: String,
    /// Per-sample stamp into `build_rhs` (after existing RHS construction,
    /// before return). Empty when no sources. Also caches per-source `i_n`
    /// into `state.noise_*_last_i_n[k]` so the BE-fallback replay
    /// (`rhs_stamp_be`) can re-inject the same noise without consuming
    /// fresh RNG samples (which would break determinism: same seed →
    /// same audio output regardless of how many samples trip BE).
    pub rhs_stamp: String,
    /// BE-fallback noise replay. Reads cached per-source `i_n` from
    /// `state.noise_*_last_i_n` and stamps into `rhs_be`. Injected into
    /// the BE fallback block right after `rhs_be[INPUT_NODE] += …;`.
    /// Empty when no sources. Trap-MNA 2× compensation is left in (BE
    /// will be ~+3 dB hot vs strict physics during BE samples — bounded,
    /// rare, far below the dominating signal that triggered BE in the
    /// first place; preferable to noise dropouts during BE cooldowns).
    pub rhs_stamp_be: String,
    /// Body of `reset()` (re-seed RNG, clear gaussian cache, clear
    /// per-source caches).
    pub reset_body: String,
    /// Snippet emitted into the NaN-recovery block to clear
    /// noise-specific transient state (`noise_thermal_w_prev` and the
    /// per-source `last_i_n` caches). Does NOT re-seed the RNG —
    /// determinism contract says `set_seed` is the only re-seed entry.
    pub nan_recovery_body: String,
    /// Body to append inside `set_sample_rate` — recomputes `thermal_scale`.
    pub set_sample_rate_body: String,
    /// `impl CircuitState` methods: set_noise_enabled, set_noise_gain, …
    pub methods: String,
    /// `true` when any code is emitted (for template `{% if noise_enabled %}`).
    pub enabled: bool,
    /// Count of thermal sources. Kept for debug logging.
    #[allow(dead_code)]
    pub thermal_n: usize,
    /// Per-family source counts, so a consumer outside this module can emit the
    /// replay into its own RHS buffer (see `emit_noise_replay_body`).
    pub replay_counts: NoiseReplayCounts,
    /// Reverse lookup: `pot_index → noise source index` for dynamic
    /// sources (`.pot` / `.wiper` / `.runtime R` members). Empty vec of
    /// length `mna.pots.len()` when noise is off. A `Some(k)` entry means
    /// the pot setter for that index should update
    /// `state.noise_thermal_sqrt_inv_r[k]` after writing the new resistance.
    pub pot_to_noise_slot: Vec<Option<usize>>,
    /// Reverse lookup: `[switch_idx][comp_idx] → noise source index` for
    /// R-type switch components. The outer Vec is indexed by the switch
    /// number (matching `ir.switches`), the inner by the component index
    /// within that switch. A `Some(k)` entry means `set_switch_N(position)`
    /// should update `state.noise_thermal_sqrt_inv_r[k]` from the
    /// position-indexed R value. C/L components always map to `None`.
    pub switch_comp_to_noise_slot: Vec<Vec<Option<usize>>>,
    /// Reverse lookup: `pot_index → r_flicker source index` (Phase 3.5).
    /// `Some(k)` means the pot setter should also update
    /// `state.noise_r_flicker_inv_r[k]`. Sparse — most pots have no
    /// resistor-flicker source unless the user opted in with `KF=…` on
    /// the pot's resistor line.
    pub pot_to_r_flicker_slot: Vec<Option<usize>>,
    /// Reverse lookup: `[switch_idx][comp_idx] → r_flicker source index`.
    /// Same shape as `switch_comp_to_noise_slot`; sparse.
    pub switch_comp_to_r_flicker_slot: Vec<Vec<Option<usize>>>,
    /// Per-switch flag: `true` when the switch controls an R component
    /// touching some op-amp's non-inverting input, so the emitted
    /// `set_switch_N(position)` must call `refresh_opamp_en_g_diag()`
    /// (absolute recompute — see `methods`). Empty when noise is off or no
    /// op-amp noise sources exist.
    pub switch_to_opamp_en_refresh: Vec<bool>,
    /// Reverse lookup: `pot_index → list of op-amp source indices` whose
    /// `noise_opamp_en_g_diag[k]` must be refreshed when this pot's
    /// resistance changes (Phase 4). A pot between (a, b) with conductance
    /// `g_pot` contributes `+g_pot` to `G[a, a]` AND `G[b, b]`. Non-empty
    /// entries make the corresponding `set_pot_N` / `set_runtime_R_<field>`
    /// body call `refresh_opamp_en_g_diag()` — an **absolute recompute**
    /// from `NOISE_OPAMP_EN_G_BASE[k]` + every live dynamic conductance at
    /// in+ (2026-07-18; replaces the old incremental `+= 1/r − 1/r_old`
    /// accumulation, which drifted in FP over unbounded knob rides).
    /// Empty for pots that don't touch any op-amp's in+ (zero codegen
    /// overhead for circuits with fixed-resistor input networks — the
    /// common case).
    pub pot_to_opamp_en_refresh: Vec<Vec<usize>>,
}

impl RustEmitter {
    /// Produce all code fragments for noise emission — every phase
    /// (thermal, shot incl. Γ² smoothing, junction/resistor flicker,
    /// pentode partition, op-amp en/in). Single source of truth for both
    /// the DK and nodal codegen paths.
    ///
    /// Returns all-empty `NoiseEmission` (enabled=false) when
    /// `ir.noise.mode == NoiseMode::Off` or every per-phase source list
    /// is empty.
    /// Resolve the shot-suppression multiplier Γ² for one shot source.
    ///
    /// - `Junction` → 1.0 (full Schottky shot `2·q·|I|`).
    /// - `FaBase` → `1/BF`: the FA-reduced NR slot carries `Ic`; the base
    ///   junction's shot PSD is `2·q·Ib = (1/BF)·2·q·Ic`.
    /// - `TriodePlate` → van der Ziel space-charge smoothing via the
    ///   Thompson/North/Harris equivalent noise resistance `R_eq ≈ 2.5/gm`
    ///   referenced to `T₀ = 290 K` (RCA Review, Jan 1940; van der Ziel,
    ///   *Noise*, 1954 §14; standard audio form R_eq[triode] = 2.5/gm):
    ///   `S_i = 4·k·T₀·R_eq·gm² = 10·k·T₀·gm  [A²/Hz]`, so relative to
    ///   full shot `Γ² = 10·k·T₀·gm / (2·q·I_p)`, evaluated at the DC
    ///   operating point (gm by central difference of the Koren plate
    ///   current at the OP Vgk/Vpk) and clamped to (0, 1] — smoothing can
    ///   never exceed full shot. `.model TUBE(SHOT_GAMMA2=…)` overrides the
    ///   computation; `SHOT_GAMMA2=1.0` restores the legacy full-shot
    ///   emission exactly (no `NOISE_SHOT_GAMMA` const, byte-identical).
    ///   The Γ² *ratio* is a codegen-time constant; the stamp still tracks
    ///   the live `|i_nl_prev|`, so signal-dependent shot modulation is
    ///   preserved (the ratio drift of gm/Ip over the swing is second-order).
    ///   Falls back to 1.0 (full shot) when the OP data is unavailable or
    ///   degenerate (cutoff bias, gm ≤ 0) — at cutoff the amplitude is ~0
    ///   anyway.
    fn resolve_shot_gamma2(ir: &CircuitIR, src: &crate::codegen::ir::ShotNoiseSource) -> f64 {
        use crate::codegen::ir::ShotSourceKind;
        const K_B: f64 = 1.380649e-23;
        const Q_E: f64 = 1.602176634e-19;
        const T0_K: f64 = 290.0;

        // Explicit override always wins (validated finite > 0 at collection).
        if let Some(g2) = src.gamma2_override {
            return g2;
        }
        let slot_params = ir
            .device_slots
            .iter()
            .find(|s| s.start_idx == src.slot_idx)
            .map(|s| &s.params);
        match src.kind {
            ShotSourceKind::Junction => 1.0,
            ShotSourceKind::FaBase => match slot_params {
                Some(DeviceParams::Bjt(bp)) if bp.beta_f.is_finite() && bp.beta_f > 0.0 => {
                    1.0 / bp.beta_f
                }
                _ => 1.0,
            },
            ShotSourceKind::TriodePlate { grid_node } => {
                let Some(DeviceParams::Tube(tp)) = slot_params else {
                    return 1.0;
                };
                let ip_dc = ir
                    .dc_nl_currents
                    .get(src.slot_idx)
                    .copied()
                    .unwrap_or(0.0)
                    .abs();
                if !(ip_dc.is_finite() && ip_dc > 1e-12) {
                    return 1.0;
                }
                let v_at = |node: usize| -> f64 {
                    if node == 0 {
                        0.0
                    } else {
                        ir.dc_operating_point.get(node - 1).copied().unwrap_or(0.0)
                    }
                };
                let v_k = v_at(src.node_j); // cathode
                let vgk = v_at(grid_node) - v_k;
                let vpk = v_at(src.node_i) - v_k;
                let triode = melange_devices::tube::KorenTriode {
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
                let h = 1e-3;
                let gm = (triode.plate_current(vgk + h, vpk) - triode.plate_current(vgk - h, vpk))
                    / (2.0 * h);
                if !(gm.is_finite() && gm > 0.0) {
                    return 1.0;
                }
                let gamma2 = 10.0 * K_B * T0_K * gm / (2.0 * Q_E * ip_dc);
                if gamma2.is_finite() && gamma2 > 0.0 {
                    gamma2.min(1.0)
                } else {
                    1.0
                }
            }
        }
    }

    pub(super) fn build_noise_emission(&self, ir: &CircuitIR) -> NoiseEmission {
        let thermal_n = ir.noise.thermal_sources.len();
        let shot_n = ir.noise.shot_sources.len();
        let flicker_n = ir.noise.flicker_sources.len();
        let r_flicker_n = ir.noise.resistor_flicker_sources.len();
        let partition_n = ir.noise.partition_sources.len();
        let opamp_n = ir.noise.opamp_noise_sources.len();
        if ir.noise.mode == NoiseMode::Off
            || (thermal_n == 0
                && shot_n == 0
                && flicker_n == 0
                && r_flicker_n == 0
                && partition_n == 0
                && opamp_n == 0)
        {
            return NoiseEmission::default();
        }
        // BE-primary calibration (auto-BE promotion or --backward-euler):
        // every noise scale below carries trap-MNA compensation sized for the
        // trapezoidal kernel's halved LF gain (A − A_neg = 2G). Under
        // BE-primary the kernel has FULL LF gain (A − A_neg = G, 2x trap in
        // amplitude), so the trap-calibrated stamps come out +6 dB hot at LF.
        // Mirror of the DC-source branch (rhs_const x1 under BE vs x2 trap):
        // every BE stamp must be HALF the trap stamp's LF amplitude.
        //   - two-draw phases (thermal, partition, opamp en/in): emit a
        //     single draw at the trap per-draw amplitude. Dropping the
        //     w[n] + w[n-1] pair halves the LF amplitude exactly AND removes
        //     the pair-sum's cos^2 envelope (a spurious -3 dB @ fs/4 droop
        //     under BE — BE damps z = -1 by itself, no anti-alias pair
        //     needed).
        //   - single-draw phases (shot, flicker, r-flicker): halve the
        //     amplitude via the scale constant (variance / 4).
        // Numerically validated against the trap kTC anchor in
        // tests/noise_psd_validation.rs::thermal_noise_be_primary_matches_trap_anchor.
        // See NOISE.md "Backward-Euler-primary stamps".
        let be_primary = ir.solver_config.backward_euler;
        // The Kellett pink filter helper is shared between junction flicker
        // (Phase 3) and resistor flicker (Phase 3.5). Emit when either is
        // present.
        let need_kellett = flicker_n > 0 || r_flicker_n > 0;
        // Flicker absolute calibration (2026-07-18): both flicker phases use
        // an fs/OS-INVARIANT white-input amplitude scale
        //   trap:        sqrt(2  · 1/K_pink)   (×2 amplitude = trap-MNA comp)
        //   BE-primary:  sqrt(0.5 · 1/K_pink)  (physical, full-gain kernel)
        // where K_pink = kellett_pink_normalized_gain() ≈ 6.0e-3 is the
        // cascade's |H(ν)|² ≈ K_pink/ν gain constant, computed analytically
        // at codegen time. Full derivation in ir/noise.rs and NOISE.md
        // "Flicker calibration".
        let kellett_k = crate::codegen::ir::kellett_pink_normalized_gain();
        let flicker_scale_trap = (2.0 / kellett_k).sqrt();
        let flicker_scale_be = (0.5 / kellett_k).sqrt();
        // `Q_E`, `noise_shot_scale`, `shot_gain` and `set_shot_gain` are
        // shared between Phase 2 shot and Phase 5 pentode partition — both
        // need `sqrt(4·q·…·fs)` amplitudes and the `shot_gain` runtime knob
        // (partition is shot at the screen-divert barrier; one mute call
        // silences both, matching the user-facing convention agreed with
        // Noyce). Gate the shared infrastructure on whichever phase needs it.
        let need_q_scale = shot_n > 0 || partition_n > 0;

        // Shot-suppression amplitude multipliers sqrt(Γ²) (tube plate
        // space-charge smoothing, FA base shot 1/BF). Emitted only when at
        // least one source has Γ² ≠ 1 so plain-junction circuits — and
        // tube circuits carded with SHOT_GAMMA2=1.0 — remain byte-identical
        // to pre-Γ builds. See `resolve_shot_gamma2`.
        let shot_gamma_amp: Vec<f64> = ir
            .noise
            .shot_sources
            .iter()
            .map(|s| Self::resolve_shot_gamma2(ir, s).sqrt())
            .collect();
        let shot_gamma_needed = shot_gamma_amp.iter().any(|g| (g - 1.0).abs() > 1e-12);

        // Phase 4 en-stamp dynamic conductance survey (2026-07-18): for each
        // op-amp noise source with an active en stamp, find every `.pot` /
        // `.wiper` / `.runtime R` and every `.switch` R component with a
        // terminal on `node_plus`. These drive the `refresh_opamp_en_g_diag`
        // ABSOLUTE recompute: `en_g_diag[k] = BASE[k] + Σ live dynamic G`,
        // where BASE strips the codegen-time dynamic contributions out of
        // the static `G[in+, in+]`. Replaces the old incremental
        // `+= 1/r − 1/r_old` (FP drift over unbounded knob rides) and adds
        // the previously-missing `.switch` refresh hook.
        let opamp_dyn_pots: Vec<Vec<usize>> = ir
            .noise
            .opamp_noise_sources
            .iter()
            .map(|src| {
                if !(src.en > 0.0 && src.node_plus > 0) {
                    return Vec::new();
                }
                ir.pots
                    .iter()
                    .enumerate()
                    .filter(|(_, p)| p.node_p == src.node_plus || p.node_q == src.node_plus)
                    .map(|(i, _)| i)
                    .collect()
            })
            .collect();
        let opamp_dyn_switches: Vec<Vec<(usize, usize)>> = ir
            .noise
            .opamp_noise_sources
            .iter()
            .map(|src| {
                if !(src.en > 0.0 && src.node_plus > 0) {
                    return Vec::new();
                }
                let mut v = Vec::new();
                for (si, sw) in ir.switches.iter().enumerate() {
                    for (ci, comp) in sw.components.iter().enumerate() {
                        if comp.component_type == 'R'
                            && (comp.node_p == src.node_plus || comp.node_q == src.node_plus)
                        {
                            v.push((si, ci));
                        }
                    }
                }
                v
            })
            .collect();
        let opamp_any_dynamic = opamp_dyn_pots.iter().any(|v| !v.is_empty())
            || opamp_dyn_switches.iter().any(|v| !v.is_empty());

        let mut top = String::new();
        top.push_str("// ----------------------------------------------------------------------\n");
        top.push_str("// Authentic circuit noise — Phases 1 (thermal) + 2 (shot) + 3 (flicker)\n");
        top.push_str("// Generated when --noise {thermal|shot|full}. See docs/aidocs/NOISE.md\n");
        top.push_str(
            "// ----------------------------------------------------------------------\n\n",
        );

        top.push_str("/// Boltzmann constant [J/K] (exact SI 2019).\n");
        top.push_str("pub const K_B: f64 = 1.380649e-23;\n");
        top.push_str("/// Standard lab noise temperature [K] (16.85 °C, the \"kT\" reference).\n");
        top.push_str("pub const T_ROOM_K: f64 = 290.0;\n");
        if need_q_scale {
            top.push_str("/// Elementary charge [C] (exact SI 2019). Used for shot-noise PSD\n");
            top.push_str("/// (Phase 2) and pentode partition noise (Phase 5).\n");
            top.push_str("pub const Q_E: f64 = 1.602176634e-19;\n");
        }
        top.push_str(
            &"/// Default master seed baked in by codegen. `0` → entropy-seeded at Default.\n"
                .to_string(),
        );
        top.push_str(&format!(
            "pub const NOISE_MASTER_SEED_DEFAULT: u64 = {};\n\n",
            ir.noise.master_seed
        ));

        top.push_str(&format!(
            "pub const NOISE_THERMAL_N: usize = {};\n",
            thermal_n
        ));
        // Emit 1-indexed node arrays; 0 = ground (matches MNA convention)
        let fmt_usize_arr = |items: &[usize]| -> String {
            items
                .iter()
                .map(|v| v.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        };
        let node_i: Vec<usize> = ir.noise.thermal_sources.iter().map(|s| s.node_i).collect();
        let node_j: Vec<usize> = ir.noise.thermal_sources.iter().map(|s| s.node_j).collect();
        top.push_str(&format!(
            "pub(crate) const NOISE_THERMAL_NODE_I: [usize; NOISE_THERMAL_N] = [{}];\n",
            fmt_usize_arr(&node_i)
        ));
        top.push_str(&format!(
            "pub(crate) const NOISE_THERMAL_NODE_J: [usize; NOISE_THERMAL_N] = [{}];\n",
            fmt_usize_arr(&node_j)
        ));
        // Precomputed sqrt(1/R) default values — one per source. Static
        // entries (fixed resistors) are baked once and never change;
        // dynamic entries (`.pot` / `.wiper` / `.runtime R`) start from the
        // pot's nominal R and get refreshed in `set_pot_N` /
        // `set_runtime_R_<field>` so the per-sample coefficient tracks the
        // live resistance. The runtime mirror lives in
        // `state.noise_thermal_sqrt_inv_r[k]` — both are read from the same
        // index in the RHS stamp.
        let sqrt_inv_r: Vec<String> = ir
            .noise
            .thermal_sources
            .iter()
            .map(|s| fmt_f64((1.0 / s.resistance).sqrt()))
            .collect();
        top.push_str(&format!(
            "pub(crate) const NOISE_THERMAL_SQRT_INV_R_DEFAULT: [f64; NOISE_THERMAL_N] = [{}];\n\n",
            sqrt_inv_r.join(", ")
        ));

        // Shot-noise source table: one entry per forward-biased junction.
        // `SLOT_IDX` indexes `state.i_nl_prev`; the per-sample coefficient
        // is `sqrt(4·q·|I_prev|·fs)` with the same 2× trap-MNA calibration
        // as thermal (see `docs/aidocs/NOISE.md` "Constant derivation").
        // Gated on `shot_n > 0` so thermal-only builds stay byte-identical
        // to pre-Step-4 codegen — no dead `NOISE_SHOT_N = 0` constants leak.
        if shot_n > 0 {
            top.push_str(&format!("pub const NOISE_SHOT_N: usize = {};\n", shot_n));
            let shot_slot: Vec<usize> = ir.noise.shot_sources.iter().map(|s| s.slot_idx).collect();
            let shot_ni: Vec<usize> = ir.noise.shot_sources.iter().map(|s| s.node_i).collect();
            let shot_nj: Vec<usize> = ir.noise.shot_sources.iter().map(|s| s.node_j).collect();
            top.push_str(&format!(
                "pub(crate) const NOISE_SHOT_SLOT_IDX: [usize; NOISE_SHOT_N] = [{}];\n",
                fmt_usize_arr(&shot_slot)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_SHOT_NODE_I: [usize; NOISE_SHOT_N] = [{}];\n",
                fmt_usize_arr(&shot_ni)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_SHOT_NODE_J: [usize; NOISE_SHOT_N] = [{}];\n\n",
                fmt_usize_arr(&shot_nj)
            ));
            if shot_gamma_needed {
                let gamma_amp: Vec<String> = shot_gamma_amp.iter().map(|g| fmt_f64(*g)).collect();
                top.push_str(
                    "/// Per-source shot amplitude multiplier `sqrt(Γ²)`. Γ² = 1 for plain\n\
                     /// junctions; triode plates carry van der Ziel space-charge smoothing\n\
                     /// Γ² = 10·k·T₀·gm/(2·q·I_p) at the DC OP (R_eq ≈ 2.5/gm form,\n\
                     /// Thompson/North/Harris 1940; `.model TUBE(SHOT_GAMMA2=…)` overrides);\n\
                     /// FA-reduced BJT base-shot sources carry Γ² = 1/BF (slot current is\n\
                     /// Ic, physical base shot is 2·q·Ic/BF).\n",
                );
                top.push_str(&format!(
                    "pub(crate) const NOISE_SHOT_GAMMA_AMP: [f64; NOISE_SHOT_N] = [{}];\n\n",
                    gamma_amp.join(", ")
                ));
            }
        }

        // Flicker (1/f) noise source table. Per-sample amplitude is
        //   sqrt(2/K_pink) · sqrt(KF) · |I_prev|^(AF/2) · N(0,1)   (trap)
        // fed into a Paul Kellett 7-pole pink filter — fs/OS-invariant,
        // output PSD S_i(f) = KF·I^AF/f one-sided (see NOISE.md "Flicker
        // calibration"). Source collection only adds devices whose `.model`
        // supplies `KF > 0`, so zero-KF builds leak no flicker constants.
        if flicker_n > 0 {
            top.push_str(&format!(
                "pub const NOISE_FLICKER_N: usize = {};\n",
                flicker_n
            ));
            let fl_slot: Vec<usize> = ir
                .noise
                .flicker_sources
                .iter()
                .map(|s| s.slot_idx)
                .collect();
            let fl_ni: Vec<usize> = ir.noise.flicker_sources.iter().map(|s| s.node_i).collect();
            let fl_nj: Vec<usize> = ir.noise.flicker_sources.iter().map(|s| s.node_j).collect();
            let fl_sqrt_kf: Vec<String> = ir
                .noise
                .flicker_sources
                .iter()
                .map(|s| fmt_f64(s.kf.sqrt()))
                .collect();
            let fl_half_af: Vec<String> = ir
                .noise
                .flicker_sources
                .iter()
                .map(|s| fmt_f64(0.5 * s.af))
                .collect();
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_SLOT_IDX: [usize; NOISE_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&fl_slot)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_NODE_I: [usize; NOISE_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&fl_ni)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_NODE_J: [usize; NOISE_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&fl_nj)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_SQRT_KF: [f64; NOISE_FLICKER_N] = [{}];\n",
                fl_sqrt_kf.join(", ")
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_HALF_AF: [f64; NOISE_FLICKER_N] = [{}];\n\n",
                fl_half_af.join(", ")
            ));
        }

        // Resistor flicker (Hooge bias-squared, Phase 3.5). Per-sample
        // amplitude is `sqrt(2·KF/K_pink) · |I_R|^(AF/2) · N(0,1)` (trap;
        // same fs/OS-invariant calibration as junction flicker) fed into the
        // shared Kellett 7-pole pink filter. `I_R = (V_+ − V_−)/R` is read
        // live from `state.v_prev` at each sample. The opt-in collector
        // emits no entries when no resistor sets `KF`, so zero-KF builds
        // leak no constants.
        if r_flicker_n > 0 {
            top.push_str(&format!(
                "pub const NOISE_R_FLICKER_N: usize = {};\n",
                r_flicker_n
            ));
            let rf_ni: Vec<usize> = ir
                .noise
                .resistor_flicker_sources
                .iter()
                .map(|s| s.node_i)
                .collect();
            let rf_nj: Vec<usize> = ir
                .noise
                .resistor_flicker_sources
                .iter()
                .map(|s| s.node_j)
                .collect();
            let rf_sqrt_kf: Vec<String> = ir
                .noise
                .resistor_flicker_sources
                .iter()
                .map(|s| fmt_f64(s.kf.sqrt()))
                .collect();
            let rf_half_af: Vec<String> = ir
                .noise
                .resistor_flicker_sources
                .iter()
                .map(|s| fmt_f64(0.5 * s.af))
                .collect();
            let rf_inv_r: Vec<String> = ir
                .noise
                .resistor_flicker_sources
                .iter()
                .map(|s| fmt_f64(1.0 / s.resistance))
                .collect();
            top.push_str(&format!(
                "pub(crate) const NOISE_R_FLICKER_NODE_I: [usize; NOISE_R_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&rf_ni)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_R_FLICKER_NODE_J: [usize; NOISE_R_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&rf_nj)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_R_FLICKER_SQRT_KF: [f64; NOISE_R_FLICKER_N] = [{}];\n",
                rf_sqrt_kf.join(", ")
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_R_FLICKER_HALF_AF: [f64; NOISE_R_FLICKER_N] = [{}];\n",
                rf_half_af.join(", ")
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_R_FLICKER_INV_R_DEFAULT: [f64; NOISE_R_FLICKER_N] = [{}];\n\n",
                rf_inv_r.join(", ")
            ));
        }

        // Pentode partition noise table (Phase 5). One entry per pentode
        // device. Per-sample plate-current amplitude is
        //   sqrt(4·q · I_p · I_s / (I_p + I_s) · fs) · PARTITION_F · N(0,1)
        // = noise_shot_scale · sqrt(I_p·I_s/(I_p+I_s)) · PARTITION_F · N(0,1)
        // with `I_p = state.i_nl_prev[IP_SLOT[k]]` and
        // `I_s = state.i_nl_prev[IS_SLOT[k]]` (both one-sample lagged).
        // Replaces the Phase 2 bare plate-shot for pentodes (the shot
        // collector filters those out — `collect_shot_noise_sources` in ir.rs).
        // Same 2× trap-MNA compensation as thermal/shot/junction-flicker;
        // two-draw Nyquist anti-alias preserves the kTC-style invariant.
        if partition_n > 0 {
            top.push_str(&format!(
                "pub const NOISE_PARTITION_N: usize = {};\n",
                partition_n
            ));
            let p_ip: Vec<usize> = ir
                .noise
                .partition_sources
                .iter()
                .map(|s| s.ip_slot_idx)
                .collect();
            let p_is: Vec<usize> = ir
                .noise
                .partition_sources
                .iter()
                .map(|s| s.is_slot_idx)
                .collect();
            let p_ni: Vec<usize> = ir
                .noise
                .partition_sources
                .iter()
                .map(|s| s.node_i)
                .collect();
            let p_nj: Vec<usize> = ir
                .noise
                .partition_sources
                .iter()
                .map(|s| s.node_j)
                .collect();
            let p_f: Vec<String> = ir
                .noise
                .partition_sources
                .iter()
                .map(|s| fmt_f64(s.partition_f))
                .collect();
            top.push_str(&format!(
                "pub(crate) const NOISE_PARTITION_IP_SLOT: [usize; NOISE_PARTITION_N] = [{}];\n",
                fmt_usize_arr(&p_ip)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_PARTITION_IS_SLOT: [usize; NOISE_PARTITION_N] = [{}];\n",
                fmt_usize_arr(&p_is)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_PARTITION_NODE_I: [usize; NOISE_PARTITION_N] = [{}];\n",
                fmt_usize_arr(&p_ni)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_PARTITION_NODE_J: [usize; NOISE_PARTITION_N] = [{}];\n",
                fmt_usize_arr(&p_nj)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_PARTITION_F: [f64; NOISE_PARTITION_N] = [{}];\n\n",
                p_f.join(", ")
            ));
        }

        // Op-amp input-referred noise table (Phase 4). One entry per op-amp
        // whose `.model OA(EN=…)` or `.model OA(IN=…)` opts in. Three Norton
        // streams per source:
        //   en  at NODE_PLUS : amp = en  · noise_opamp_en_g_diag[k] · sqrt(fs)
        //   in+ at NODE_PLUS : amp = in  · sqrt(fs)
        //   in- at NODE_MINUS: amp = in  · sqrt(fs)
        // All three use two-draw Nyquist anti-alias and inherit the
        // `opamp_input_gain` runtime knob (signal-independent; conceptually
        // distinct from shot_gain, per the Noyce response letter).
        // `g_diag_plus_default` is the static `G[in+, in+]` at codegen time;
        // dynamic-R refresh is reserved for v1.5 (state field exists so the
        // future refresh wiring lands without breaking calling-side API).
        if opamp_n > 0 {
            top.push_str(&format!("pub const NOISE_OPAMP_N: usize = {};\n", opamp_n));
            top.push_str(&format!(
                "pub const NOISE_OPAMP_IN_N: usize = {};\n",
                2 * opamp_n
            ));
            let oa_np: Vec<usize> = ir
                .noise
                .opamp_noise_sources
                .iter()
                .map(|s| s.node_plus)
                .collect();
            let oa_nm: Vec<usize> = ir
                .noise
                .opamp_noise_sources
                .iter()
                .map(|s| s.node_minus)
                .collect();
            let oa_en: Vec<String> = ir
                .noise
                .opamp_noise_sources
                .iter()
                .map(|s| fmt_f64(s.en))
                .collect();
            let oa_in: Vec<String> = ir
                .noise
                .opamp_noise_sources
                .iter()
                .map(|s| fmt_f64(s.in_amps))
                .collect();
            let oa_g: Vec<String> = ir
                .noise
                .opamp_noise_sources
                .iter()
                .map(|s| fmt_f64(s.g_diag_plus_default))
                .collect();
            top.push_str(&format!(
                "pub(crate) const NOISE_OPAMP_NODE_PLUS: [usize; NOISE_OPAMP_N] = [{}];\n",
                fmt_usize_arr(&oa_np)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_OPAMP_NODE_MINUS: [usize; NOISE_OPAMP_N] = [{}];\n",
                fmt_usize_arr(&oa_nm)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_OPAMP_EN: [f64; NOISE_OPAMP_N] = [{}];\n",
                oa_en.join(", ")
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_OPAMP_IN: [f64; NOISE_OPAMP_N] = [{}];\n",
                oa_in.join(", ")
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_OPAMP_EN_G_DIAG_DEFAULT: [f64; NOISE_OPAMP_N] = [{}];\n\n",
                oa_g.join(", ")
            ));
            if opamp_any_dynamic {
                let oa_base: Vec<String> = ir
                    .noise
                    .opamp_noise_sources
                    .iter()
                    .enumerate()
                    .map(|(k, src)| {
                        let mut base = src.g_diag_plus_default;
                        for &p in &opamp_dyn_pots[k] {
                            base -= ir.pots[p].g_nominal;
                        }
                        for &(s, c) in &opamp_dyn_switches[k] {
                            let nominal = ir.switches[s].components[c].nominal_value;
                            if nominal.is_finite() && nominal > 0.0 {
                                base -= 1.0 / nominal;
                            }
                        }
                        fmt_f64(base)
                    })
                    .collect();
                top.push_str(
                    "/// Static (non-dynamic) part of `G[in+, in+]` per op-amp source —\n\
                     /// the codegen-time diagonal with every `.pot`/`.wiper`/`.runtime R`/\n\
                     /// `.switch`-R contribution at in+ stripped out. `refresh_opamp_en_g_diag`\n\
                     /// rebuilds the live diagonal as BASE + Σ current dynamic conductances\n\
                     /// (absolute recompute — immune to FP drift, correct after any\n\
                     /// pot/switch history).\n",
                );
                top.push_str(&format!(
                    "pub(crate) const NOISE_OPAMP_EN_G_BASE: [f64; NOISE_OPAMP_N] = [{}];\n\n",
                    oa_base.join(", ")
                ));
            }
        }

        // xoshiro256++ RNG: fast, high-quality, 256-bit state per stream.
        top.push_str("#[derive(Clone, Copy, Debug)]\n");
        top.push_str("pub struct Xoshiro256pp { pub s: [u64; 4] }\n\n");
        top.push_str("impl Xoshiro256pp {\n");
        top.push_str("    #[inline(always)]\n");
        top.push_str("    pub fn next_u64(&mut self) -> u64 {\n");
        top.push_str("        let result = self.s[0].wrapping_add(self.s[3]).rotate_left(23).wrapping_add(self.s[0]);\n");
        top.push_str("        let t = self.s[1] << 17;\n");
        top.push_str("        self.s[2] ^= self.s[0];\n");
        top.push_str("        self.s[3] ^= self.s[1];\n");
        top.push_str("        self.s[1] ^= self.s[2];\n");
        top.push_str("        self.s[0] ^= self.s[3];\n");
        top.push_str("        self.s[2] ^= t;\n");
        top.push_str("        self.s[3] = self.s[3].rotate_left(45);\n");
        top.push_str("        result\n");
        top.push_str("    }\n");
        top.push_str("    /// Uniform f64 in [0, 1). Upper 53 bits of the u64.\n");
        top.push_str("    #[inline(always)]\n");
        top.push_str("    pub fn next_f64(&mut self) -> f64 {\n");
        top.push_str("        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)\n");
        top.push_str("    }\n");
        top.push_str("}\n\n");

        // SplitMix64: derives per-stream seeds from one master seed.
        top.push_str("#[inline(always)]\n");
        top.push_str("fn splitmix64(state: &mut u64) -> u64 {\n");
        top.push_str("    *state = state.wrapping_add(0x9E3779B97F4A7C15);\n");
        top.push_str("    let mut z = *state;\n");
        top.push_str("    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);\n");
        top.push_str("    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);\n");
        top.push_str("    z ^ (z >> 31)\n");
        top.push_str("}\n\n");

        top.push_str(
            "/// Seed an array of Xoshiro256pp streams from one master seed via SplitMix64.\n",
        );
        top.push_str("/// Every stream gets statistically independent state — no cross-source correlation.\n");
        top.push_str("fn seed_noise_rngs<const N: usize>(master: u64) -> [Xoshiro256pp; N] {\n");
        top.push_str("    seed_noise_rngs_salted::<N>(master, 0)\n");
        top.push_str("}\n\n");
        top.push_str("/// Seed an array of Xoshiro256pp streams with an additional 64-bit salt.\n");
        top.push_str("/// Used to derive per-phase streams (thermal vs shot) from one master\n");
        top.push_str("/// seed without any cross-phase prefix overlap. Salt is XORed into the\n");
        top.push_str("/// SplitMix64 state after the standard entropy resolution.\n");
        top.push_str("fn seed_noise_rngs_salted<const N: usize>(master: u64, salt: u64) -> [Xoshiro256pp; N] {\n");
        top.push_str("    let mut sm = if master == 0 {\n");
        top.push_str("        // master=0 → entropy from system clock. Plugin hosts wanting\n");
        top.push_str("        // determinism should call set_seed(nonzero) before processing.\n");
        top.push_str("        std::time::SystemTime::now()\n");
        top.push_str("            .duration_since(std::time::UNIX_EPOCH)\n");
        top.push_str("            .map(|d| d.as_nanos() as u64)\n");
        top.push_str("            .unwrap_or(0x0123456789ABCDEF)\n");
        top.push_str("    } else { master };\n");
        top.push_str("    sm ^= salt;\n");
        top.push_str("    // First mix to avoid weak seeds\n");
        top.push_str("    let _ = splitmix64(&mut sm);\n");
        top.push_str("    let mut out = [Xoshiro256pp { s: [0; 4] }; N];\n");
        top.push_str("    for k in 0..N {\n");
        top.push_str("        out[k].s[0] = splitmix64(&mut sm);\n");
        top.push_str("        out[k].s[1] = splitmix64(&mut sm);\n");
        top.push_str("        out[k].s[2] = splitmix64(&mut sm);\n");
        top.push_str("        out[k].s[3] = splitmix64(&mut sm);\n");
        top.push_str("        // xoshiro requires at least one nonzero state word.\n");
        top.push_str("        if out[k].s == [0; 4] { out[k].s[0] = 1; }\n");
        top.push_str("    }\n");
        top.push_str("    out\n");
        top.push_str("}\n\n");
        top.push_str("/// Shot-noise salt. Distinct 64-bit constant applied to the master\n");
        top.push_str("/// seed so shot streams do not share any prefix with thermal streams\n");
        top.push_str("/// under a deterministic seed. Chosen to be a high-entropy value.\n");
        top.push_str("pub const NOISE_SHOT_SALT: u64 = 0xA5A5_DEAD_BEEF_CAFE;\n\n");

        if flicker_n > 0 {
            top.push_str("/// Flicker-noise salt. Distinct from thermal and shot salts so\n");
            top.push_str("/// every pink-filter input stream is independent under a\n");
            top.push_str("/// deterministic master seed.\n");
            top.push_str("pub const NOISE_FLICKER_SALT: u64 = 0xC0DE_BABE_DEAD_BEEF;\n\n");
        }
        if r_flicker_n > 0 {
            top.push_str("/// Resistor-flicker salt. Distinct from junction flicker so\n");
            top.push_str("/// resistor 1/f streams cannot share a prefix with junction\n");
            top.push_str("/// flicker streams under a deterministic master seed.\n");
            top.push_str("pub const NOISE_R_FLICKER_SALT: u64 = 0xCA12_B0CC_F11C_E12E;\n\n");
        }
        if partition_n > 0 {
            top.push_str("/// Pentode partition salt (Phase 5). Distinct from thermal /\n");
            top.push_str("/// shot / flicker so partition streams never share a prefix\n");
            top.push_str("/// with the cathode-shot streams in the same circuit.\n");
            top.push_str("pub const NOISE_PARTITION_SALT: u64 = 0x9E27_0DE9_A271_710E;\n\n");
        }
        if opamp_n > 0 {
            top.push_str("/// Op-amp `en` (voltage-noise) salt (Phase 4). Distinct from\n");
            top.push_str("/// op-amp `in` so the two phases of input-referred noise\n");
            top.push_str("/// cannot share a prefix under a deterministic master seed.\n");
            top.push_str("pub const NOISE_OPAMP_EN_SALT: u64 = 0x0FA3_94E5_E700_5A17;\n\n");
            top.push_str("/// Op-amp `in` (current-noise) salt (Phase 4). Seeds the\n");
            top.push_str("/// 2N stream array — index `2k` is the in+ source, `2k+1`\n");
            top.push_str("/// is the in- source for op-amp k. Distinct from the EN\n");
            top.push_str("/// salt so en and in streams never share a prefix.\n");
            top.push_str("pub const NOISE_OPAMP_IN_SALT: u64 = 0x0FA3_94E5_1700_5A17;\n\n");
        }
        if need_kellett {
            // Paul Kellett 7-pole pink filter (musicdsp.org pk3 variant,
            // ±0.05 dB over 9.2 octaves). White in, pink out with a ~1/f
            // PSD shape. NOTE on the `* 0.11` tail: it does NOT normalize
            // the cascade to unit gain — the measured white power gain of
            // the tailed cascade is ≈ 0.113 (RMS gain ≈ 0.34). Absolute
            // flicker level is therefore NOT calibrated here; it is set by
            // the analytic K_pink constant (|H(ν)|² ≈ K_pink/ν ≈ 6.0e-3/ν,
            // includes the 0.11 tail) baked into `noise_flicker_scale` /
            // `noise_r_flicker_sqrt_fs`. Do not retune 0.11 — it would
            // silently shift K_pink out from under the baked constants;
            // both are derived together at codegen time. Shared between
            // junction flicker (Phase 3) and resistor flicker (Phase 3.5).
            top.push_str("#[inline(always)]\n");
            top.push_str("fn kellett_pink(white: f64, state: &mut [f64; 7]) -> f64 {\n");
            top.push_str("    state[0] = 0.99886 * state[0] + white * 0.0555179;\n");
            top.push_str("    state[1] = 0.99332 * state[1] + white * 0.0750759;\n");
            top.push_str("    state[2] = 0.96900 * state[2] + white * 0.1538520;\n");
            top.push_str("    state[3] = 0.86650 * state[3] + white * 0.3104856;\n");
            top.push_str("    state[4] = 0.55000 * state[4] + white * 0.5329522;\n");
            top.push_str("    state[5] = -0.7616 * state[5] - white * 0.0168980;\n");
            top.push_str("    let pink = state[0] + state[1] + state[2] + state[3]\n");
            top.push_str("        + state[4] + state[5] + state[6] + white * 0.5362;\n");
            top.push_str("    state[6] = white * 0.115926;\n");
            top.push_str("    pink * 0.11\n");
            top.push_str("}\n\n");
        }

        // Gaussian via Marsaglia polar method.
        top.push_str("/// Standard-normal (µ=0, σ=1) sample via Marsaglia polar method.\n");
        top.push_str(
            "/// One RNG pair yields two Gaussians; the second is cached for the next call.\n",
        );
        top.push_str("#[inline(always)]\n");
        top.push_str("fn gaussian(rng: &mut Xoshiro256pp, cache: &mut Option<f64>) -> f64 {\n");
        top.push_str("    if let Some(z) = cache.take() { return z; }\n");
        top.push_str("    loop {\n");
        top.push_str("        let u = 2.0 * rng.next_f64() - 1.0;\n");
        top.push_str("        let v = 2.0 * rng.next_f64() - 1.0;\n");
        top.push_str("        let s = u * u + v * v;\n");
        top.push_str("        if s > 0.0 && s < 1.0 {\n");
        top.push_str("            let factor = (-2.0 * s.ln() / s).sqrt();\n");
        top.push_str("            *cache = Some(v * factor);\n");
        top.push_str("            return u * factor;\n");
        top.push_str("        }\n");
        top.push_str("    }\n");
        top.push_str("}\n\n");

        // State fields — injected inside CircuitState struct body
        let mut state_fields = String::new();
        state_fields.push_str(
            &"    /// Per-source xoshiro256++ state — independent streams, no cross-correlation.\n"
                .to_string(),
        );
        state_fields.push_str(&"    pub noise_rng: [Xoshiro256pp; NOISE_THERMAL_N],\n".to_string());
        state_fields
            .push_str(&"    /// Cached second Gaussian from Marsaglia polar pair.\n".to_string());
        state_fields.push_str(
            &"    pub noise_gaussian_cache: [Option<f64>; NOISE_THERMAL_N],\n".to_string(),
        );
        state_fields.push_str("    /// Master noise switch — runtime. Default false (opt-in).\n");
        state_fields.push_str("    pub noise_enabled: bool,\n");
        state_fields.push_str("    /// Master scalar applied to every noise source.\n");
        state_fields.push_str("    pub noise_gain: f64,\n");
        state_fields.push_str("    /// Scalar applied only to Johnson-Nyquist thermal sources.\n");
        state_fields.push_str("    pub thermal_gain: f64,\n");
        state_fields.push_str("    /// Circuit temperature [K]. Runtime-settable.\n");
        state_fields.push_str("    pub temperature_k: f64,\n");
        state_fields.push_str("    /// Master seed recorded for reset() re-derivation.\n");
        state_fields.push_str("    pub noise_master_seed: u64,\n");
        if be_primary {
            state_fields.push_str(
                "    /// Precomputed sqrt(2·K_B·T·fs_internal) — BE-primary single-draw\n",
            );
            state_fields.push_str(
                "    /// calibration (un-doubled; see NOISE.md 'Backward-Euler-primary\n",
            );
            state_fields.push_str("    /// stamps').\n");
        } else {
            state_fields.push_str(
                "    /// Precomputed sqrt(8·K_B·T·fs_internal). Halved in the two-draw stamp\n",
            );
            state_fields.push_str(
                "    /// (each draw uses scale/2 so consecutive-draw sum preserves kTC; see\n",
            );
            state_fields.push_str(
                "    /// NOISE.md 'Constant derivation' and 'Nyquist anti-aliasing' sections).\n",
            );
        }
        state_fields.push_str("    pub noise_thermal_scale: f64,\n");
        state_fields.push_str(
            "    /// Effective internal sample rate (host_rate × OVERSAMPLING_FACTOR).\n",
        );
        state_fields.push_str(
            "    /// Tracked so `set_temperature_k` can recompute `noise_thermal_scale`.\n",
        );
        state_fields.push_str("    pub noise_fs: f64,\n");
        state_fields.push_str(
            "    /// Per-source `sqrt(1/R)` — live mirror of `NOISE_THERMAL_SQRT_INV_R_DEFAULT`.\n",
        );
        state_fields.push_str(
            "    /// Static entries stay at their baked value. Dynamic entries (`.pot` /\n",
        );
        state_fields.push_str(
            "    /// `.wiper` / `.runtime R` members) are refreshed inside the matching\n",
        );
        state_fields.push_str("    /// `set_pot_N` / `set_runtime_R_<field>` setter.\n");
        state_fields.push_str("    pub noise_thermal_sqrt_inv_r: [f64; NOISE_THERMAL_N],\n");
        state_fields.push_str(
            "    /// Per-source previous-draw buffer for the two-draw Nyquist-anti-alias\n",
        );
        state_fields.push_str(
            "    /// scheme. Each thermal stamp uses `w_new + w_prev` where w_new is the\n",
        );
        state_fields.push_str(
            "    /// current draw (amplitude scale/2) and w_prev is the previous draw.\n",
        );
        state_fields.push_str("    /// Zeroed at default(), reset(), and set_seed(). See NOISE.md 'Nyquist anti-aliasing'.\n");
        state_fields.push_str("    pub noise_thermal_w_prev: [f64; NOISE_THERMAL_N],\n");
        state_fields
            .push_str("    /// Per-source last-stamped `i_n` cache. Populated by the trap rhs\n");
        state_fields
            .push_str("    /// stamp; replayed by the BE-fallback stamp so BE samples carry the\n");
        state_fields
            .push_str("    /// same noise content as the trap solve they replaced (no audible\n");
        state_fields
            .push_str("    /// dropout during BE cooldowns; same RNG sequence regardless of how\n");
        state_fields.push_str("    /// many samples trip BE).\n");
        state_fields.push_str("    pub noise_thermal_last_i_n: [f64; NOISE_THERMAL_N],\n");
        if need_q_scale {
            state_fields
                .push_str("    /// Scalar applied to shot-noise sources (Phase 2) and pentode\n");
            state_fields
                .push_str("    /// partition noise (Phase 5). Shared because partition is shot\n");
            state_fields.push_str("    /// at a different barrier; one mute call silences both.\n");
            state_fields.push_str("    pub shot_gain: f64,\n");
            state_fields.push_str(
                "    /// Precomputed `sqrt(4·Q_E·fs_internal)`. Shared between shot and\n",
            );
            state_fields
                .push_str("    /// partition: shot uses `noise_shot_scale · sqrt(|I_prev|)`,\n");
            state_fields
                .push_str("    /// partition uses `noise_shot_scale · sqrt(I_p·I_s/(I_p+I_s))`.\n");
            state_fields.push_str("    /// Updated by `set_sample_rate`.\n");
            state_fields.push_str("    pub noise_shot_scale: f64,\n");
        }
        if shot_n > 0 {
            state_fields.push_str(
                "    /// Per-source xoshiro256++ state for shot noise — salted so streams\n",
            );
            state_fields.push_str(
                "    /// cannot share a prefix with thermal streams under any master seed.\n",
            );
            state_fields.push_str("    pub noise_shot_rng: [Xoshiro256pp; NOISE_SHOT_N],\n");
            state_fields.push_str(
                "    /// Cached second Gaussian from Marsaglia polar pair (shot stream).\n",
            );
            state_fields
                .push_str("    pub noise_shot_gaussian_cache: [Option<f64>; NOISE_SHOT_N],\n");
            state_fields.push_str(
                "    /// Per-source previous half-draw for the two-draw Nyquist-anti-alias\n\
                 \x20   /// pair (trap path only; BE-primary is single-draw). Zeroed at\n\
                 \x20   /// default(), reset(), and set_seed(). See NOISE.md 'Nyquist anti-aliasing'.\n",
            );
            state_fields.push_str("    pub noise_shot_w_prev: [f64; NOISE_SHOT_N],\n");
            state_fields
                .push_str("    /// Per-source last-stamped `i_n` cache (BE-fallback replay).\n");
            state_fields.push_str("    pub noise_shot_last_i_n: [f64; NOISE_SHOT_N],\n");
        }
        if flicker_n > 0 {
            state_fields
                .push_str("    /// Per-source xoshiro256++ state for the Kellett pink-filter\n");
            state_fields
                .push_str("    /// input — salted distinct from thermal and shot streams.\n");
            state_fields.push_str("    pub noise_flicker_rng: [Xoshiro256pp; NOISE_FLICKER_N],\n");
            state_fields.push_str(
                "    /// Cached second Gaussian from Marsaglia polar pair (flicker stream).\n",
            );
            state_fields.push_str(
                "    pub noise_flicker_gaussian_cache: [Option<f64>; NOISE_FLICKER_N],\n",
            );
            state_fields.push_str(
                "    /// Per-source 7-pole Kellett filter state. Zeroed at `default()`\n",
            );
            state_fields
                .push_str("    /// and at `reset()`; settles in a handful of samples once audio\n");
            state_fields.push_str("    /// processing begins.\n");
            state_fields.push_str("    pub noise_flicker_state: [[f64; 7]; NOISE_FLICKER_N],\n");
            // `flicker_gain` is shared with resistor flicker (Phase 3.5), so
            // it lives on `CircuitState` whenever any flicker source exists.
            // Emitting it here vs. in the r_flicker block matters only for
            // builds that have r_flicker but no junction flicker — handled
            // by the r_flicker block below.
            state_fields
                .push_str("    /// Scalar applied to flicker sources (Phase 3 junction +\n");
            state_fields.push_str("    /// Phase 3.5 resistor). Runtime.\n");
            state_fields.push_str("    pub flicker_gain: f64,\n");
            state_fields.push_str(
                "    /// Flicker white-input scale `sqrt(2/K_pink)` (trap; `sqrt(0.5/K_pink)`\n",
            );
            state_fields
                .push_str("    /// BE-primary) — fs/OS-INVARIANT. Per-sample amplitude is\n");
            state_fields.push_str(
                "    /// `noise_flicker_scale · NOISE_FLICKER_SQRT_KF[k] · |I_prev|^(AF/2)`\n",
            );
            state_fields.push_str("    /// before being shaped by the Kellett pink filter,\n");
            state_fields.push_str(
                "    /// landing the output PSD at S_i(f) = KF·|I|^AF / f (one-sided).\n",
            );
            state_fields.push_str("    pub noise_flicker_scale: f64,\n");
            state_fields
                .push_str("    /// Per-source last-stamped `i_n` cache (BE-fallback replay).\n");
            state_fields.push_str("    pub noise_flicker_last_i_n: [f64; NOISE_FLICKER_N],\n");
            if !be_primary {
                state_fields.push_str(
                    "    /// Previous half-stamp of the flicker Nyquist anti-alias pair\n\
                     \x20   /// (`i_n = w[n] + w[n-1]`, w = 0.5·amp·pink). The Kellett cascade\n\
                     \x20   /// only attenuates Nyquist ~14 dB; on resistive junction nodes the\n\
                     \x20   /// trap z=-1 pole amplifies that tail by tens of dB and the device\n\
                     \x20   /// nonlinearity intermodulates it into the audio band (same failure\n\
                     \x20   /// class as the 2026-04-24 thermal artifact). The pair-sum zeroes\n\
                     \x20   /// the Nyquist bin while leaving the audio-band 1/f calibration\n\
                     \x20   /// unchanged (cos²(πf/fs) ≈ 1). Zeroed at default/reset/set_seed.\n",
                );
                state_fields.push_str("    pub noise_flicker_w_prev: [f64; NOISE_FLICKER_N],\n");
            }
        }
        if r_flicker_n > 0 {
            state_fields
                .push_str("    /// Per-source xoshiro256++ state for resistor 1/f (Phase 3.5).\n");
            state_fields
                .push_str("    /// Salted distinct from thermal/shot/junction-flicker streams.\n");
            state_fields
                .push_str("    pub noise_r_flicker_rng: [Xoshiro256pp; NOISE_R_FLICKER_N],\n");
            state_fields.push_str(
                "    /// Cached second Gaussian from Marsaglia polar pair (r-flicker stream).\n",
            );
            state_fields.push_str(
                "    pub noise_r_flicker_gaussian_cache: [Option<f64>; NOISE_R_FLICKER_N],\n",
            );
            state_fields.push_str(
                "    /// Per-source 7-pole Kellett filter state. Zeroed at `default()`\n",
            );
            state_fields
                .push_str("    /// and `reset()`. Frozen across samples where the resistor\n");
            state_fields
                .push_str("    /// carries < 1e-15 A — same convention as junction flicker.\n");
            state_fields
                .push_str("    pub noise_r_flicker_state: [[f64; 7]; NOISE_R_FLICKER_N],\n");
            state_fields.push_str(
                "    /// Per-source `1/R` — live mirror of `NOISE_R_FLICKER_INV_R_DEFAULT`.\n",
            );
            state_fields
                .push_str("    /// Static entries stay at their baked value; dynamic entries\n");
            state_fields.push_str(
                "    /// (`.pot` / `.wiper` / `.runtime R` / `.switch` R) are refreshed\n",
            );
            state_fields
                .push_str("    /// inside the matching `set_pot_N` / `set_runtime_R_<field>` /\n");
            state_fields
                .push_str("    /// `set_switch_N` setter so 1/f tracks the live resistance.\n");
            state_fields.push_str("    pub noise_r_flicker_inv_r: [f64; NOISE_R_FLICKER_N],\n");
            state_fields
                .push_str("    /// Per-source last-stamped `i_n` cache (BE-fallback replay).\n");
            state_fields.push_str("    pub noise_r_flicker_last_i_n: [f64; NOISE_R_FLICKER_N],\n");
            if !be_primary {
                state_fields.push_str(
                    "    /// Previous half-stamp of the r-flicker Nyquist anti-alias pair\n\
                     \x20   /// (same scheme as `noise_flicker_w_prev`).\n",
                );
                state_fields
                    .push_str("    pub noise_r_flicker_w_prev: [f64; NOISE_R_FLICKER_N],\n");
            }
            state_fields
                .push_str("    /// Resistor-flicker white-input scale — same fs/OS-INVARIANT\n");
            state_fields.push_str(
                "    /// `sqrt(2/K_pink)` (trap) / `sqrt(0.5/K_pink)` (BE) constant as\n",
            );
            state_fields
                .push_str("    /// junction flicker. Field name is legacy (held `sqrt(fs)`\n");
            state_fields
                .push_str("    /// before the 2026-07-18 calibration fix). Per-sample amplitude\n");
            state_fields
                .push_str("    /// `· NOISE_R_FLICKER_SQRT_KF[k] · |I_R|^(AF/2)` before the\n");
            state_fields
                .push_str("    /// Kellett filter. No T coupling — Hooge 1/f is bias-driven\n");
            state_fields.push_str("    /// and T-independent.\n");
            state_fields.push_str("    pub noise_r_flicker_sqrt_fs: f64,\n");
            if flicker_n == 0 {
                // r_flicker without junction flicker still needs flicker_gain.
                state_fields
                    .push_str("    /// Scalar applied to flicker sources (Phase 3.5 resistor;\n");
                state_fields
                    .push_str("    /// no junction flicker present in this build). Runtime.\n");
                state_fields.push_str("    pub flicker_gain: f64,\n");
            }
        }
        if partition_n > 0 {
            state_fields.push_str(
                "    /// Per-source xoshiro256++ state for pentode partition (Phase 5).\n",
            );
            state_fields
                .push_str("    /// Salted distinct from shot so partition streams never share a\n");
            state_fields.push_str("    /// prefix with the cathode-shot streams.\n");
            state_fields
                .push_str("    pub noise_partition_rng: [Xoshiro256pp; NOISE_PARTITION_N],\n");
            state_fields.push_str(
                "    /// Cached second Gaussian from Marsaglia polar pair (partition).\n",
            );
            state_fields.push_str(
                "    pub noise_partition_gaussian_cache: [Option<f64>; NOISE_PARTITION_N],\n",
            );
            state_fields.push_str(
                "    /// Per-source previous-draw buffer for two-draw Nyquist anti-alias.\n",
            );
            state_fields.push_str(
                "    /// Each partition stamp uses `w_new + w_prev`. Zeroed at default(),\n",
            );
            state_fields.push_str(
                "    /// reset(), set_seed(), and NaN recovery. Same pattern as thermal.\n",
            );
            state_fields.push_str("    pub noise_partition_w_prev: [f64; NOISE_PARTITION_N],\n");
            state_fields
                .push_str("    /// Per-source last-stamped `i_n` cache (BE-fallback replay).\n");
            state_fields.push_str("    pub noise_partition_last_i_n: [f64; NOISE_PARTITION_N],\n");
        }
        if opamp_n > 0 {
            state_fields
                .push_str("    /// Op-amp `en` xoshiro256++ state (Phase 4). One stream per\n");
            state_fields.push_str(
                "    /// op-amp, salted distinct from `in` and from every other phase.\n",
            );
            state_fields.push_str("    pub noise_opamp_en_rng: [Xoshiro256pp; NOISE_OPAMP_N],\n");
            state_fields.push_str("    /// Cached second Gaussian for the en stream.\n");
            state_fields
                .push_str("    pub noise_opamp_en_gaussian_cache: [Option<f64>; NOISE_OPAMP_N],\n");
            state_fields
                .push_str("    /// Op-amp `in` xoshiro256++ state (Phase 4). 2N streams —\n");
            state_fields
                .push_str("    /// `2k` = in+, `2k+1` = in- for op-amp k. SplitMix64 derivation\n");
            state_fields.push_str("    /// gives independent prefixes per stream.\n");
            state_fields
                .push_str("    pub noise_opamp_in_rng: [Xoshiro256pp; NOISE_OPAMP_IN_N],\n");
            state_fields
                .push_str("    /// Cached second Gaussian for in streams (same 2N layout).\n");
            state_fields.push_str(
                "    pub noise_opamp_in_gaussian_cache: [Option<f64>; NOISE_OPAMP_IN_N],\n",
            );
            state_fields.push_str("    /// Live `G[in+, in+]` mirror — initialized to\n");
            state_fields
                .push_str("    /// `NOISE_OPAMP_EN_G_DIAG_DEFAULT`; recomputed absolutely by\n");
            state_fields
                .push_str("    /// `refresh_opamp_en_g_diag()` from every pot/runtime-R/switch\n");
            state_fields
                .push_str("    /// setter whose element touches in+; restored to the default on\n");
            state_fields.push_str("    /// `reset()` / `set_seed()`.\n");
            state_fields.push_str("    pub noise_opamp_en_g_diag: [f64; NOISE_OPAMP_N],\n");
            state_fields.push_str("    /// Two-draw lag buffers — separate for en, in+, in-.\n");
            state_fields.push_str("    pub noise_opamp_en_w_prev: [f64; NOISE_OPAMP_N],\n");
            state_fields.push_str("    pub noise_opamp_in_w_prev: [f64; NOISE_OPAMP_IN_N],\n");
            state_fields
                .push_str("    /// BE-fallback replay caches — separate for en, in (2N).\n");
            state_fields.push_str("    pub noise_opamp_en_last_i_n: [f64; NOISE_OPAMP_N],\n");
            state_fields.push_str("    pub noise_opamp_in_last_i_n: [f64; NOISE_OPAMP_IN_N],\n");
            state_fields
                .push_str("    /// Scalar applied to en and in stamps (Phase 4). Runtime knob;\n");
            state_fields
                .push_str("    /// signal-independent (op-amp hiss is constant, unlike shot).\n");
            state_fields.push_str("    pub opamp_input_gain: f64,\n");
            state_fields
                .push_str("    /// Precomputed `sqrt(2·fs_internal)`. Per-sample amplitudes are\n");
            state_fields.push_str(
                "    /// `NOISE_OPAMP_EN[k] · noise_opamp_en_g_diag[k] · sqrt_2fs` (en)\n",
            );
            state_fields
                .push_str("    /// and `NOISE_OPAMP_IN[k] · sqrt_2fs` (in). Refreshed in\n");
            state_fields
                .push_str("    /// `set_sample_rate`. The `2·fs` factor folds the trap-MNA 4×\n");
            state_fields.push_str(
                "    /// PSD compensation (output voltage is half via `(A-A_neg) = 2G`,\n",
            );
            state_fields
                .push_str("    /// so input PSD must be 4×) into a single state scalar shared\n");
            state_fields.push_str("    /// across en/in streams.\n");
            state_fields.push_str("    pub noise_opamp_sqrt_fs: f64,\n");
        }

        // Default impl: compute thermal_scale and seed RNGs
        let mut default_stmts = String::new();
        default_stmts.push_str("        // Noise state (thermal only in Phase 1)\n");
        default_stmts
            .push_str("        let fs_internal = SAMPLE_RATE * OVERSAMPLING_FACTOR as f64;\n");
        // Per-sample Norton-current variance:  σ² = 8·k_B·T·fs / R
        // (The physically correct one-sided PSD  S_i = 4·k_B·T/R  over [0, fs/2]
        //  would naively give σ² = 2·k_B·T·fs/R, but melange's DK-trap
        //  formulation satisfies  (A - A_neg)·v_ss = stamp,  so a steady
        //  current source gets half the continuous-time DC gain. The input
        //  stamp compensates via (V_new + V_prev)·G_in; here we compensate
        //  by doubling the per-sample variance instead of caching a second
        //  RNG draw. Net: output V²_rms = kT/C across every RC lowpass,
        //  matching the Nyquist kTC equilibrium. Validated by the kTC
        //  theorem test in tests/noise_psd_validation.rs.)
        if be_primary {
            default_stmts.push_str(
                "        // BE-primary: single-draw stamp at the un-doubled amplitude\n\
                 \x20       // sqrt(2*K_B*T*fs) — BE's LF kernel gain (A - A_neg = G) is 2x\n\
                 \x20       // trap's (2G), so the trap pair-sum stamp would be +6 dB hot.\n\
                 \x20       // See NOISE.md 'Backward-Euler-primary stamps'.\n",
            );
            default_stmts.push_str(
                "        let noise_thermal_scale = (2.0 * K_B * T_ROOM_K * fs_internal).sqrt();\n",
            );
        } else {
            default_stmts.push_str(
                "        let noise_thermal_scale = (8.0 * K_B * T_ROOM_K * fs_internal).sqrt();\n",
            );
        }
        default_stmts.push_str("        let noise_rng = seed_noise_rngs::<NOISE_THERMAL_N>(NOISE_MASTER_SEED_DEFAULT);\n");
        if need_q_scale {
            default_stmts.push_str("        // Shared shot/partition amplitude: sqrt(4·Q_E·fs).\n");
            default_stmts
                .push_str("        // Per-sample variance: σ² = 4·Q_E·|I|·fs. Same 2× trap-MNA\n");
            default_stmts
                .push_str("        // compensation as thermal (PSD 2·q·|I| on [0, fs/2] gives\n");
            default_stmts
                .push_str("        // naive σ² = q·|I|·fs; doubling to preserve DK-trap DC gain\n");
            default_stmts.push_str(
                "        // yields 2·q·|I|·fs; amplitude expressed as sqrt(X·Q_E·|I|·fs)\n",
            );
            default_stmts
                .push_str("        // has X=4). Partition reuses this scale with a different\n");
            default_stmts.push_str(
                "        // per-sample sqrt(...) factor (I_p·I_s/(I_p+I_s) rather than\n",
            );
            default_stmts.push_str("        // |I|).\n");
            if be_primary {
                default_stmts.push_str(
                    "        // BE-primary: half amplitude (X=1) — BE's full LF kernel gain\n\
                     \x20       // makes the trap X=4 stamp +6 dB hot. Partition consumes this\n\
                     \x20       // scale at full amplitude single-draw (its trap x0.5 pair-sum\n\
                     \x20       // factor is dropped in the stamp).\n",
                );
                default_stmts
                    .push_str("        let noise_shot_scale = (Q_E * fs_internal).sqrt();\n");
            } else {
                default_stmts
                    .push_str("        let noise_shot_scale = (4.0 * Q_E * fs_internal).sqrt();\n");
            }
        }
        if shot_n > 0 {
            default_stmts
                .push_str("        // Shot-noise streams: salted distinct from thermal streams.\n");
            default_stmts.push_str("        let noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_SHOT_SALT);\n");
        }
        if flicker_n > 0 {
            default_stmts.push_str(
                "        // Flicker streams: salted distinct from thermal/shot streams.\n",
            );
            default_stmts.push_str(
                "        // Per-source white-input variance fed into the Kellett filter:\n",
            );
            default_stmts.push_str(&format!(
                "        //   σ_w² = {}·KF·|I|^AF / K_pink,  K_pink ≈ {:.4e}\n",
                if be_primary { "0.5" } else { "2" },
                kellett_k
            ));
            default_stmts.push_str(
                "        // K_pink is the Kellett cascade's normalized-frequency gain\n\
                 \x20       // constant (|H(ν)|² ≈ K_pink/ν), computed analytically at codegen\n\
                 \x20       // time (kellett_pink_normalized_gain in ir/noise.rs). Because the\n\
                 \x20       // pink filter's gain at fixed physical f scales as K_pink·fs/f,\n\
                 \x20       // the white-input variance must be fs-INDEPENDENT for the output\n\
                 \x20       // PSD to land at S_i(f) = KF·I^AF/f (one-sided, ngspice KF/AF\n\
                 \x20       // semantics) at every fs and oversampling factor. The 2 (vs the\n\
                 \x20       // physical 0.5) carries the trap-MNA ×2-amplitude compensation;\n\
                 \x20       // BE-primary keeps the physical 0.5 (full-gain kernel).\n",
            );
            default_stmts.push_str(&format!(
                "        let noise_flicker_scale = {};\n",
                fmt_f64(if be_primary {
                    flicker_scale_be
                } else {
                    flicker_scale_trap
                })
            ));
            default_stmts.push_str("        let noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_FLICKER_SALT);\n");
        }
        if r_flicker_n > 0 {
            default_stmts
                .push_str("        // Resistor-flicker streams (Hooge bias-squared, Phase 3.5).\n");
            default_stmts.push_str(&format!(
                "        // Per-source white-input variance: σ_w² = {}·KF·|I_R|^AF / K_pink —\n",
                if be_primary { "0.5" } else { "2" }
            ));
            default_stmts.push_str(
                "        // identical calibration to junction flicker (same Kellett cascade,\n\
                 \x20       // same Norton RHS stamp through the same trap/BE kernel — the old\n\
                 \x20       // claim that r-flicker bypasses the (A − A_neg) = 2G companion\n\
                 \x20       // gain was false), so output PSD lands at S_i = KF·I_R^AF/f\n\
                 \x20       // one-sided, fs/OS-invariant. Field name `noise_r_flicker_sqrt_fs`\n\
                 \x20       // is legacy (pre-2026-07-18 it held sqrt(fs)); it now carries the\n\
                 \x20       // fs-independent scale constant.\n",
            );
            default_stmts.push_str(&format!(
                "        let noise_r_flicker_sqrt_fs = {};\n",
                fmt_f64(if be_primary {
                    flicker_scale_be
                } else {
                    flicker_scale_trap
                })
            ));
            default_stmts.push_str("        let noise_r_flicker_rng = seed_noise_rngs_salted::<NOISE_R_FLICKER_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_R_FLICKER_SALT);\n");
        }
        if partition_n > 0 {
            default_stmts
                .push_str("        // Pentode partition streams (Phase 5). Salted distinct from\n");
            default_stmts.push_str(
                "        // shot so partition does not share a prefix with cathode-shot\n",
            );
            default_stmts.push_str(
                "        // under any master seed. Amplitude scaling reuses noise_shot_scale.\n",
            );
            default_stmts.push_str("        let noise_partition_rng = seed_noise_rngs_salted::<NOISE_PARTITION_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_PARTITION_SALT);\n");
        }
        if opamp_n > 0 {
            default_stmts
                .push_str("        // Op-amp en/in streams (Phase 4). Two salts: EN seeds the\n");
            default_stmts.push_str(
                "        // per-op-amp en RNG; IN seeds the 2N stream array (in+ at 2k,\n",
            );
            default_stmts.push_str(
                "        // in- at 2k+1). Shared `sqrt(2·fs)` factor — see state field doc.\n",
            );
            if be_primary {
                default_stmts.push_str(
                    "        // BE-primary: sqrt(0.5*fs) = trap per-draw amplitude; the stamp\n\
                     \x20       // is single-draw at full scale (no x0.5, no w_prev pair).\n",
                );
                default_stmts
                    .push_str("        let noise_opamp_sqrt_fs = (0.5 * fs_internal).sqrt();\n");
            } else {
                default_stmts
                    .push_str("        let noise_opamp_sqrt_fs = (2.0 * fs_internal).sqrt();\n");
            }
            default_stmts.push_str("        let noise_opamp_en_rng = seed_noise_rngs_salted::<NOISE_OPAMP_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_OPAMP_EN_SALT);\n");
            default_stmts.push_str("        let noise_opamp_in_rng = seed_noise_rngs_salted::<NOISE_OPAMP_IN_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_OPAMP_IN_SALT);\n");
        }

        let mut default_fields = String::new();
        default_fields.push_str("            noise_rng,\n");
        default_fields.push_str("            noise_gaussian_cache: [None; NOISE_THERMAL_N],\n");
        default_fields.push_str("            noise_enabled: false,\n");
        default_fields.push_str("            noise_gain: 1.0,\n");
        default_fields.push_str("            thermal_gain: 1.0,\n");
        default_fields.push_str("            temperature_k: T_ROOM_K,\n");
        default_fields.push_str("            noise_master_seed: NOISE_MASTER_SEED_DEFAULT,\n");
        default_fields.push_str("            noise_thermal_scale,\n");
        default_fields.push_str("            noise_fs: fs_internal,\n");
        default_fields
            .push_str("            noise_thermal_sqrt_inv_r: NOISE_THERMAL_SQRT_INV_R_DEFAULT,\n");
        default_fields.push_str("            noise_thermal_w_prev: [0.0; NOISE_THERMAL_N],\n");
        default_fields.push_str("            noise_thermal_last_i_n: [0.0; NOISE_THERMAL_N],\n");
        if need_q_scale {
            default_fields.push_str("            shot_gain: 1.0,\n");
            default_fields.push_str("            noise_shot_scale,\n");
        }
        if shot_n > 0 {
            default_fields.push_str("            noise_shot_rng,\n");
            default_fields
                .push_str("            noise_shot_gaussian_cache: [None; NOISE_SHOT_N],\n");
            default_fields.push_str("            noise_shot_w_prev: [0.0; NOISE_SHOT_N],\n");
            default_fields.push_str("            noise_shot_last_i_n: [0.0; NOISE_SHOT_N],\n");
        }
        if flicker_n > 0 {
            default_fields.push_str("            noise_flicker_rng,\n");
            default_fields
                .push_str("            noise_flicker_gaussian_cache: [None; NOISE_FLICKER_N],\n");
            default_fields
                .push_str("            noise_flicker_state: [[0.0; 7]; NOISE_FLICKER_N],\n");
            default_fields.push_str("            flicker_gain: 1.0,\n");
            default_fields.push_str("            noise_flicker_scale,\n");
            default_fields
                .push_str("            noise_flicker_last_i_n: [0.0; NOISE_FLICKER_N],\n");
            if !be_primary {
                default_fields
                    .push_str("            noise_flicker_w_prev: [0.0; NOISE_FLICKER_N],\n");
            }
        }
        if r_flicker_n > 0 {
            default_fields.push_str("            noise_r_flicker_rng,\n");
            default_fields.push_str(
                "            noise_r_flicker_gaussian_cache: [None; NOISE_R_FLICKER_N],\n",
            );
            default_fields
                .push_str("            noise_r_flicker_state: [[0.0; 7]; NOISE_R_FLICKER_N],\n");
            default_fields
                .push_str("            noise_r_flicker_inv_r: NOISE_R_FLICKER_INV_R_DEFAULT,\n");
            default_fields
                .push_str("            noise_r_flicker_last_i_n: [0.0; NOISE_R_FLICKER_N],\n");
            if !be_primary {
                default_fields
                    .push_str("            noise_r_flicker_w_prev: [0.0; NOISE_R_FLICKER_N],\n");
            }
            default_fields.push_str("            noise_r_flicker_sqrt_fs,\n");
            if flicker_n == 0 {
                default_fields.push_str("            flicker_gain: 1.0,\n");
            }
        }
        if partition_n > 0 {
            default_fields.push_str("            noise_partition_rng,\n");
            default_fields.push_str(
                "            noise_partition_gaussian_cache: [None; NOISE_PARTITION_N],\n",
            );
            default_fields
                .push_str("            noise_partition_w_prev: [0.0; NOISE_PARTITION_N],\n");
            default_fields
                .push_str("            noise_partition_last_i_n: [0.0; NOISE_PARTITION_N],\n");
        }
        if opamp_n > 0 {
            default_fields.push_str("            noise_opamp_en_rng,\n");
            default_fields
                .push_str("            noise_opamp_en_gaussian_cache: [None; NOISE_OPAMP_N],\n");
            default_fields.push_str("            noise_opamp_in_rng,\n");
            default_fields
                .push_str("            noise_opamp_in_gaussian_cache: [None; NOISE_OPAMP_IN_N],\n");
            default_fields
                .push_str("            noise_opamp_en_g_diag: NOISE_OPAMP_EN_G_DIAG_DEFAULT,\n");
            default_fields.push_str("            noise_opamp_en_w_prev: [0.0; NOISE_OPAMP_N],\n");
            default_fields
                .push_str("            noise_opamp_in_w_prev: [0.0; NOISE_OPAMP_IN_N],\n");
            default_fields.push_str("            noise_opamp_en_last_i_n: [0.0; NOISE_OPAMP_N],\n");
            default_fields
                .push_str("            noise_opamp_in_last_i_n: [0.0; NOISE_OPAMP_IN_N],\n");
            default_fields.push_str("            opamp_input_gain: 1.0,\n");
            default_fields.push_str("            noise_opamp_sqrt_fs,\n");
        }

        // reset() — reseed RNG and clear gaussian cache; keep user settings.
        let mut reset_body = String::new();
        reset_body.push_str(
            "        // Re-seed noise RNGs (keeps noise_enabled, gains, temperature untouched).\n",
        );
        reset_body.push_str("        self.noise_rng = seed_noise_rngs::<NOISE_THERMAL_N>(self.noise_master_seed);\n");
        reset_body.push_str("        self.noise_gaussian_cache = [None; NOISE_THERMAL_N];\n");
        reset_body.push_str(
            "        // Restore per-source sqrt(1/R) to defaults. Dynamic sources will\n",
        );
        reset_body.push_str(
            "        // be re-updated by any subsequent set_pot_N / set_runtime_R call;\n",
        );
        reset_body.push_str(
            "        // this mirrors how reset() restores pot_<i>_resistance to nominal.\n",
        );
        reset_body.push_str(
            "        self.noise_thermal_sqrt_inv_r = NOISE_THERMAL_SQRT_INV_R_DEFAULT;\n",
        );
        reset_body.push_str("        // Clear two-draw buffer + BE-replay cache so silence after reset is true zero.\n");
        reset_body.push_str("        self.noise_thermal_w_prev = [0.0; NOISE_THERMAL_N];\n");
        reset_body.push_str("        self.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            reset_body.push_str(
                "        // Re-seed shot RNGs (same master, distinct salt from thermal).\n",
            );
            reset_body.push_str("        self.noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(self.noise_master_seed, NOISE_SHOT_SALT);\n");
            reset_body.push_str("        self.noise_shot_gaussian_cache = [None; NOISE_SHOT_N];\n");
            reset_body.push_str("        self.noise_shot_w_prev = [0.0; NOISE_SHOT_N];\n");
            reset_body.push_str("        self.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            reset_body
                .push_str("        // Re-seed flicker RNGs + zero Kellett pink-filter state.\n");
            reset_body.push_str("        self.noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(self.noise_master_seed, NOISE_FLICKER_SALT);\n");
            reset_body
                .push_str("        self.noise_flicker_gaussian_cache = [None; NOISE_FLICKER_N];\n");
            reset_body
                .push_str("        self.noise_flicker_state = [[0.0; 7]; NOISE_FLICKER_N];\n");
            reset_body.push_str("        self.noise_flicker_last_i_n = [0.0; NOISE_FLICKER_N];\n");
            if !be_primary {
                reset_body
                    .push_str("        self.noise_flicker_w_prev = [0.0; NOISE_FLICKER_N];\n");
            }
        }
        if r_flicker_n > 0 {
            reset_body.push_str(
                "        // Re-seed resistor-flicker RNGs + zero Kellett state + restore 1/R.\n",
            );
            reset_body.push_str("        self.noise_r_flicker_rng = seed_noise_rngs_salted::<NOISE_R_FLICKER_N>(self.noise_master_seed, NOISE_R_FLICKER_SALT);\n");
            reset_body.push_str(
                "        self.noise_r_flicker_gaussian_cache = [None; NOISE_R_FLICKER_N];\n",
            );
            reset_body
                .push_str("        self.noise_r_flicker_state = [[0.0; 7]; NOISE_R_FLICKER_N];\n");
            reset_body
                .push_str("        self.noise_r_flicker_last_i_n = [0.0; NOISE_R_FLICKER_N];\n");
            if !be_primary {
                reset_body
                    .push_str("        self.noise_r_flicker_w_prev = [0.0; NOISE_R_FLICKER_N];\n");
            }
            reset_body
                .push_str("        self.noise_r_flicker_inv_r = NOISE_R_FLICKER_INV_R_DEFAULT;\n");
        }
        if partition_n > 0 {
            reset_body.push_str(
                "        // Re-seed partition RNGs + clear two-draw lag + BE-replay cache.\n",
            );
            reset_body.push_str("        self.noise_partition_rng = seed_noise_rngs_salted::<NOISE_PARTITION_N>(self.noise_master_seed, NOISE_PARTITION_SALT);\n");
            reset_body.push_str(
                "        self.noise_partition_gaussian_cache = [None; NOISE_PARTITION_N];\n",
            );
            reset_body
                .push_str("        self.noise_partition_w_prev = [0.0; NOISE_PARTITION_N];\n");
            reset_body
                .push_str("        self.noise_partition_last_i_n = [0.0; NOISE_PARTITION_N];\n");
        }
        if opamp_n > 0 {
            reset_body.push_str(
                "        // Re-seed op-amp en/in RNGs + clear two-draw lag + BE-replay cache.\n",
            );
            reset_body
                .push_str("        // Restore en_g_diag to its codegen-time default (dynamic-R\n");
            reset_body
                .push_str("        // refresh is deferred to v1.5; today the state field tracks\n");
            reset_body.push_str("        // the const default through reset / set_seed only).\n");
            reset_body.push_str("        self.noise_opamp_en_rng = seed_noise_rngs_salted::<NOISE_OPAMP_N>(self.noise_master_seed, NOISE_OPAMP_EN_SALT);\n");
            reset_body.push_str("        self.noise_opamp_in_rng = seed_noise_rngs_salted::<NOISE_OPAMP_IN_N>(self.noise_master_seed, NOISE_OPAMP_IN_SALT);\n");
            reset_body
                .push_str("        self.noise_opamp_en_gaussian_cache = [None; NOISE_OPAMP_N];\n");
            reset_body.push_str(
                "        self.noise_opamp_in_gaussian_cache = [None; NOISE_OPAMP_IN_N];\n",
            );
            reset_body
                .push_str("        self.noise_opamp_en_g_diag = NOISE_OPAMP_EN_G_DIAG_DEFAULT;\n");
            reset_body.push_str("        self.noise_opamp_en_w_prev = [0.0; NOISE_OPAMP_N];\n");
            reset_body.push_str("        self.noise_opamp_in_w_prev = [0.0; NOISE_OPAMP_IN_N];\n");
            reset_body.push_str("        self.noise_opamp_en_last_i_n = [0.0; NOISE_OPAMP_N];\n");
            reset_body
                .push_str("        self.noise_opamp_in_last_i_n = [0.0; NOISE_OPAMP_IN_N];\n");
        }

        // set_sample_rate tail: recompute noise scales at the new rate.
        // BE-primary builds use the un-doubled/halved factors — keep in sync
        // with the Default derivation above (see BE-primary comment there).
        let mut ssr_body = String::new();
        ssr_body.push_str("        // Noise: recompute rate-dependent scales for the new rate.\n");
        ssr_body.push_str("        self.noise_fs = sample_rate * OVERSAMPLING_FACTOR as f64;\n");
        if be_primary {
            ssr_body.push_str("        self.noise_thermal_scale = (2.0 * K_B * self.temperature_k * self.noise_fs).sqrt();\n");
            if need_q_scale {
                ssr_body
                    .push_str("        self.noise_shot_scale = (Q_E * self.noise_fs).sqrt();\n");
            }
            if opamp_n > 0 {
                ssr_body
                    .push_str("        self.noise_opamp_sqrt_fs = (0.5 * self.noise_fs).sqrt();\n");
            }
        } else {
            ssr_body.push_str("        self.noise_thermal_scale = (8.0 * K_B * self.temperature_k * self.noise_fs).sqrt();\n");
            if need_q_scale {
                ssr_body.push_str(
                    "        self.noise_shot_scale = (4.0 * Q_E * self.noise_fs).sqrt();\n",
                );
            }
            if opamp_n > 0 {
                ssr_body
                    .push_str("        self.noise_opamp_sqrt_fs = (2.0 * self.noise_fs).sqrt();\n");
            }
        }
        if flicker_n > 0 || r_flicker_n > 0 {
            ssr_body.push_str(
                "        // Flicker scales are fs/OS-INVARIANT by construction (the Kellett\n\
                 \x20       // pink filter's K_pink·fs/f gain at fixed physical f cancels the\n\
                 \x20       // white input's 1/fs PSD) — nothing to recompute here.\n",
            );
        }

        // Public API
        let mut methods = String::new();
        methods.push_str("\n    // --- Noise controls (Phase 1) ---\n\n");
        methods.push_str("    /// Turn circuit noise on or off. When off, all per-sample RNG calls are skipped.\n");
        methods.push_str(
            "    pub fn set_noise_enabled(&mut self, on: bool) { self.noise_enabled = on; }\n\n",
        );
        methods
            .push_str("    /// Master scalar applied to every noise source (post-per-category).\n");
        methods.push_str(
            "    pub fn set_noise_gain(&mut self, gain: f64) { self.noise_gain = gain; }\n\n",
        );
        methods.push_str("    /// Scalar applied only to Johnson-Nyquist thermal sources.\n");
        methods.push_str(
            "    pub fn set_thermal_gain(&mut self, gain: f64) { self.thermal_gain = gain; }\n\n",
        );
        methods.push_str("    /// Circuit temperature in Kelvin. 290 K is standard (~16.85 °C).\n");
        methods.push_str(
            "    /// Cold gear is quieter: 77 K (liquid N2) ≈ −5.76 dB, 3 K ≈ −19.9 dB.\n",
        );
        methods.push_str("    pub fn set_temperature_k(&mut self, kelvin: f64) {\n");
        methods.push_str("        if !(kelvin.is_finite() && kelvin > 0.0) { return; }\n");
        methods.push_str("        self.temperature_k = kelvin;\n");
        methods.push_str("        // Recompute thermal_scale at the currently-set sample rate.\n");
        if be_primary {
            methods.push_str(
                "        self.noise_thermal_scale = (2.0 * K_B * kelvin * self.noise_fs).sqrt();\n",
            );
        } else {
            methods.push_str(
                "        self.noise_thermal_scale = (8.0 * K_B * kelvin * self.noise_fs).sqrt();\n",
            );
        }
        methods.push_str("    }\n\n");
        if need_q_scale {
            methods
                .push_str("    /// Scalar applied to shot (Phase 2 junction) noise and pentode\n");
            methods.push_str(
                "    /// partition (Phase 5) sources. Both are shot at different barriers\n",
            );
            methods.push_str(
                "    /// — one knob mutes both. Runtime-settable. Set to `0.0` to mute\n",
            );
            methods.push_str("    /// shot/partition content without touching thermal.\n");
            methods.push_str(
                "    pub fn set_shot_gain(&mut self, gain: f64) { self.shot_gain = gain; }\n\n",
            );
        }
        // Single `set_flicker_gain` covers both Phase 3 (junction flicker)
        // and Phase 3.5 (resistor flicker) — they share `state.flicker_gain`
        // so one mute call silences all 1/f character. Emitted whenever
        // either source kind exists.
        if flicker_n > 0 || r_flicker_n > 0 {
            methods.push_str("    /// Scalar applied to all flicker (1/f) noise sources —\n");
            methods
                .push_str("    /// junction flicker (Phase 3) and resistor flicker (Phase 3.5).\n");
            methods
                .push_str("    /// Runtime-settable. Set to `0.0` to mute 1/f without touching\n");
            methods.push_str("    /// thermal or shot.\n");
            methods.push_str(
                "    pub fn set_flicker_gain(&mut self, gain: f64) { self.flicker_gain = gain; }\n\n",
            );
        }
        // `set_opamp_input_gain` mutes both en (voltage-noise) and in
        // (current-noise) op-amp streams. They share a knob because they
        // both produce constant "op-amp IC hiss" — conceptually distinct
        // from shot's signal-dependent crackle. Endorsed by Noyce
        // 2026-05-15 over the alternative of overloading set_thermal_gain
        // / set_shot_gain (which would conflate musically different controls).
        if opamp_n > 0 {
            methods.push_str("    /// Scalar applied to op-amp input-referred noise (Phase 4).\n");
            methods
                .push_str("    /// Covers both en (voltage-noise at non-inverting input) and in\n");
            methods
                .push_str("    /// (current-noise at each input). Signal-independent — distinct\n");
            methods.push_str(
                "    /// from shot/partition (`set_shot_gain`) which is bias-modulated.\n",
            );
            methods.push_str("    /// Runtime-settable; set to `0.0` to mute op-amp IC hiss.\n");
            methods.push_str("    pub fn set_opamp_input_gain(&mut self, gain: f64) { self.opamp_input_gain = gain; }\n\n");
            if opamp_any_dynamic {
                let is_nodal = matches!(ir.solver_mode, crate::codegen::ir::SolverMode::Nodal);
                methods.push_str(
                    "    /// Absolute recompute of the live `G[in+, in+]` diagonal used as the\n\
                     \x20   /// en-stamp Norton conversion factor. Called by every `set_pot_N` /\n\
                     \x20   /// `set_runtime_R_<field>` / `set_switch_N` whose element touches an\n\
                     \x20   /// op-amp non-inverting input. Absolute (BASE + Σ live dynamic G)\n\
                     \x20   /// rather than incremental so unbounded knob rides cannot\n\
                     \x20   /// accumulate FP drift.\n",
                );
                methods.push_str("    fn refresh_opamp_en_g_diag(&mut self) {\n");
                for (k, _src) in ir.noise.opamp_noise_sources.iter().enumerate() {
                    if opamp_dyn_pots[k].is_empty() && opamp_dyn_switches[k].is_empty() {
                        continue;
                    }
                    methods.push_str(&format!(
                        "        self.noise_opamp_en_g_diag[{k}] = NOISE_OPAMP_EN_G_BASE[{k}]"
                    ));
                    for &p in &opamp_dyn_pots[k] {
                        methods.push_str(&format!("\n            + 1.0 / self.pot_{p}_resistance"));
                    }
                    for &(s, c) in &opamp_dyn_switches[k] {
                        if is_nodal {
                            methods.push_str(&format!(
                                "\n            + 1.0 / SWITCH_{s}_COMP_{c}_VALUES[self.switch_{s}_position]"
                            ));
                        } else {
                            methods.push_str(&format!(
                                "\n            + 1.0 / SWITCH_{s}_VALUES[self.switch_{s}_position][{c}]"
                            ));
                        }
                    }
                    methods.push_str(";\n");
                }
                methods.push_str("    }\n\n");
            }
        }
        methods.push_str("    /// Set the master seed. `0` → entropy-seeded from system clock.\n");
        methods.push_str(
            "    /// Any nonzero value → deterministic (same seed → bit-identical noise).\n",
        );
        methods.push_str("    pub fn set_seed(&mut self, master: u64) {\n");
        methods.push_str("        self.noise_master_seed = master;\n");
        methods.push_str("        self.noise_rng = seed_noise_rngs::<NOISE_THERMAL_N>(master);\n");
        methods.push_str("        self.noise_gaussian_cache = [None; NOISE_THERMAL_N];\n");
        // Clear two-draw lag buffer so sample 0 after re-seed is deterministic
        // from the new RNG stream alone, not paired with a stale draw from the
        // previous stream. Without this, set_seed(42); set_seed(42); produces
        // two different sample-0 outputs.
        methods.push_str("        self.noise_thermal_w_prev = [0.0; NOISE_THERMAL_N];\n");
        methods.push_str("        self.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            methods.push_str("        self.noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(master, NOISE_SHOT_SALT);\n");
            methods.push_str("        self.noise_shot_gaussian_cache = [None; NOISE_SHOT_N];\n");
            methods.push_str("        self.noise_shot_w_prev = [0.0; NOISE_SHOT_N];\n");
            methods.push_str("        self.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            methods.push_str("        self.noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(master, NOISE_FLICKER_SALT);\n");
            methods
                .push_str("        self.noise_flicker_gaussian_cache = [None; NOISE_FLICKER_N];\n");
            methods.push_str("        self.noise_flicker_state = [[0.0; 7]; NOISE_FLICKER_N];\n");
            methods.push_str("        self.noise_flicker_last_i_n = [0.0; NOISE_FLICKER_N];\n");
            if !be_primary {
                methods.push_str("        self.noise_flicker_w_prev = [0.0; NOISE_FLICKER_N];\n");
            }
        }
        if r_flicker_n > 0 {
            methods.push_str("        self.noise_r_flicker_rng = seed_noise_rngs_salted::<NOISE_R_FLICKER_N>(master, NOISE_R_FLICKER_SALT);\n");
            methods.push_str(
                "        self.noise_r_flicker_gaussian_cache = [None; NOISE_R_FLICKER_N];\n",
            );
            methods
                .push_str("        self.noise_r_flicker_state = [[0.0; 7]; NOISE_R_FLICKER_N];\n");
            methods.push_str("        self.noise_r_flicker_last_i_n = [0.0; NOISE_R_FLICKER_N];\n");
            if !be_primary {
                methods
                    .push_str("        self.noise_r_flicker_w_prev = [0.0; NOISE_R_FLICKER_N];\n");
            }
        }
        if partition_n > 0 {
            methods.push_str("        self.noise_partition_rng = seed_noise_rngs_salted::<NOISE_PARTITION_N>(master, NOISE_PARTITION_SALT);\n");
            methods.push_str(
                "        self.noise_partition_gaussian_cache = [None; NOISE_PARTITION_N];\n",
            );
            methods.push_str("        self.noise_partition_w_prev = [0.0; NOISE_PARTITION_N];\n");
            methods.push_str("        self.noise_partition_last_i_n = [0.0; NOISE_PARTITION_N];\n");
        }
        if opamp_n > 0 {
            methods.push_str("        self.noise_opamp_en_rng = seed_noise_rngs_salted::<NOISE_OPAMP_N>(master, NOISE_OPAMP_EN_SALT);\n");
            methods.push_str("        self.noise_opamp_in_rng = seed_noise_rngs_salted::<NOISE_OPAMP_IN_N>(master, NOISE_OPAMP_IN_SALT);\n");
            methods
                .push_str("        self.noise_opamp_en_gaussian_cache = [None; NOISE_OPAMP_N];\n");
            methods.push_str(
                "        self.noise_opamp_in_gaussian_cache = [None; NOISE_OPAMP_IN_N];\n",
            );
            methods.push_str("        self.noise_opamp_en_w_prev = [0.0; NOISE_OPAMP_N];\n");
            methods.push_str("        self.noise_opamp_in_w_prev = [0.0; NOISE_OPAMP_IN_N];\n");
            methods.push_str("        self.noise_opamp_en_last_i_n = [0.0; NOISE_OPAMP_N];\n");
            methods.push_str("        self.noise_opamp_in_last_i_n = [0.0; NOISE_OPAMP_IN_N];\n");
        }
        methods.push_str("    }\n");

        // build_rhs stamp
        let mut rhs_stamp = String::new();
        rhs_stamp.push_str("\n    // Authentic circuit noise — Phases 1 (thermal) + 2 (shot).\n");
        rhs_stamp
            .push_str("    // Skipped entirely (zero RNG calls) when noise_enabled is false.\n");
        rhs_stamp.push_str("    if state.noise_enabled {\n");
        // Two-draw Nyquist anti-alias: stamp = w_new + w_prev where each draw has
        // amplitude scale/2. The sum zeros the Nyquist bin (|1 + e^{-jπ}|² = 0)
        // while preserving low-frequency PSD. kTC theorem holds: the sum of two
        // draws each with variance (scale/2)² × (1/R) reconstructs the same
        // kT/C equilibrium as the old single-draw-at-scale approach, because the
        // cos²(πf/fs) envelope integrates to the same low-frequency PSD as flat
        // white when convolved with the RC lowpass (fc << fs/2). Verified in
        // tests/noise_psd_validation.rs::thermal_noise_matches_ktc_theorem.
        if be_primary {
            rhs_stamp.push_str(
                "        // BE-primary single-draw thermal stamp (un-doubled amplitude; no\n\
                 \x20       // w[n]+w[n-1] pair — BE damps z=-1 itself, and the pair-sum's\n\
                 \x20       // cos^2 envelope would add a spurious -3 dB @ fs/4 droop).\n",
            );
            rhs_stamp.push_str("        let scale_th = state.noise_thermal_scale * state.noise_gain * state.thermal_gain;\n");
            rhs_stamp.push_str("        if scale_th != 0.0 {\n");
            rhs_stamp.push_str("            for k in 0..NOISE_THERMAL_N {\n");
            rhs_stamp.push_str("                let g = gaussian(&mut state.noise_rng[k], &mut state.noise_gaussian_cache[k]);\n");
            rhs_stamp.push_str(
                "                let i_n = scale_th * state.noise_thermal_sqrt_inv_r[k] * g;\n",
            );
            rhs_stamp.push_str("                state.noise_thermal_last_i_n[k] = i_n;\n");
        } else {
            rhs_stamp.push_str(
            "        // Two-draw thermal stamp: w_new + w_prev (Nyquist-zeroed, kTC-calibrated).\n",
        );
            rhs_stamp.push_str("        let scale_half = state.noise_thermal_scale * state.noise_gain * state.thermal_gain * 0.5;\n");
            rhs_stamp.push_str("        if scale_half != 0.0 {\n");
            rhs_stamp.push_str("            for k in 0..NOISE_THERMAL_N {\n");
            rhs_stamp.push_str("                let g = gaussian(&mut state.noise_rng[k], &mut state.noise_gaussian_cache[k]);\n");
            rhs_stamp.push_str(
                "                let w_new = scale_half * state.noise_thermal_sqrt_inv_r[k] * g;\n",
            );
            rhs_stamp
                .push_str("                let i_n = w_new + state.noise_thermal_w_prev[k];\n");
            rhs_stamp.push_str("                state.noise_thermal_w_prev[k] = w_new;\n");
            rhs_stamp.push_str("                state.noise_thermal_last_i_n[k] = i_n;\n");
        }
        rhs_stamp.push_str("                let ni = NOISE_THERMAL_NODE_I[k];\n");
        rhs_stamp.push_str("                let nj = NOISE_THERMAL_NODE_J[k];\n");
        rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
        rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
        rhs_stamp.push_str("            }\n");
        rhs_stamp.push_str("        } else {\n");
        rhs_stamp.push_str(
            "            // scale==0 (gain or thermal_gain muted): clear cache so BE replay\n",
        );
        rhs_stamp.push_str("            // doesn't re-inject the last enabled-mode i_n.\n");
        rhs_stamp.push_str("            state.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        rhs_stamp.push_str("        }\n");
        if shot_n > 0 {
            rhs_stamp.push_str(
                "        // Shot: `|I_prev|` is the one-sample-lagged junction current from\n\
                 \x20       // `state.i_nl_prev` (inaudible lag; lets the stamp run before NR).\n\
                 \x20       // The gate includes `shot_gain` so hosts can mute shot alone.\n",
            );
            let gamma_factor = if shot_gamma_needed {
                "NOISE_SHOT_GAMMA_AMP[k] * "
            } else {
                ""
            };
            if be_primary {
                // BE-primary: single-draw, un-doubled amplitude. BE damps the
                // z=-1 pole itself, so no two-draw pair (its cos^2 envelope would
                // add a spurious droop) — `noise_shot_scale` is already sqrt(q·fs).
                rhs_stamp.push_str(
                    "        // BE-primary: single-draw sqrt(q·|I_prev|·fs) (BE damps z=-1 itself).\n",
                );
                rhs_stamp.push_str("        let shot_scale = state.noise_shot_scale * state.noise_gain * state.shot_gain;\n");
                rhs_stamp.push_str("        if shot_scale != 0.0 {\n");
                rhs_stamp.push_str("            for k in 0..NOISE_SHOT_N {\n");
                rhs_stamp.push_str(
                    "                let i_abs = state.i_nl_prev[NOISE_SHOT_SLOT_IDX[k]].abs();\n",
                );
                rhs_stamp.push_str("                if i_abs < 1e-15 { continue; }\n");
                rhs_stamp.push_str("                let g = gaussian(&mut state.noise_shot_rng[k], &mut state.noise_shot_gaussian_cache[k]);\n");
                rhs_stamp.push_str(&format!(
                    "                let i_n = shot_scale * {gamma_factor}i_abs.sqrt() * g;\n"
                ));
                rhs_stamp.push_str("                state.noise_shot_last_i_n[k] = i_n;\n");
                rhs_stamp.push_str("                let ni = NOISE_SHOT_NODE_I[k];\n");
                rhs_stamp.push_str("                let nj = NOISE_SHOT_NODE_J[k];\n");
                rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
                rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
                rhs_stamp.push_str("            }\n");
                rhs_stamp.push_str("        } else {\n");
                rhs_stamp
                    .push_str("            state.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
                rhs_stamp.push_str("        }\n");
            } else {
                // Trapezoidal: two-draw Nyquist anti-alias pair `w_new + w_prev`,
                // each draw at half amplitude (sqrt(4·q·|I|·fs)·0.5). Without it,
                // the single-draw white shot injection excites the trap z=-1 pole
                // on a stiff / high-impedance junction node (e.g. a reverse-
                // breakdown Zener, whose ~10 pF Cak pole sits far above fs/2, so
                // the node is resistor-only at Nyquist) into an fs/2 limit cycle
                // that the junction exponential rectifies down into the audio band
                // (seed-dependent, hot, non-Gaussian). Mirrors the thermal
                // two-draw scheme; see NOISE.md "Nyquist anti-aliasing". When |I|
                // collapses below the floor, w_new = 0 so the lagged half flushes
                // over one sample instead of freezing a stale draw.
                rhs_stamp.push_str(
                    "        // Trap: two-draw Nyquist pair w_new + w_prev, sqrt(4·q·|I_prev|·fs)·0.5.\n",
                );
                rhs_stamp.push_str("        let shot_scale = state.noise_shot_scale * state.noise_gain * state.shot_gain * 0.5;\n");
                rhs_stamp.push_str("        if shot_scale != 0.0 {\n");
                rhs_stamp.push_str("            for k in 0..NOISE_SHOT_N {\n");
                rhs_stamp.push_str(
                    "                let i_abs = state.i_nl_prev[NOISE_SHOT_SLOT_IDX[k]].abs();\n",
                );
                rhs_stamp.push_str("                let w_new = if i_abs < 1e-15 {\n");
                rhs_stamp.push_str("                    0.0\n");
                rhs_stamp.push_str("                } else {\n");
                rhs_stamp.push_str("                    let g = gaussian(&mut state.noise_shot_rng[k], &mut state.noise_shot_gaussian_cache[k]);\n");
                rhs_stamp.push_str(&format!(
                    "                    shot_scale * {gamma_factor}i_abs.sqrt() * g\n"
                ));
                rhs_stamp.push_str("                };\n");
                rhs_stamp
                    .push_str("                let i_n = w_new + state.noise_shot_w_prev[k];\n");
                rhs_stamp.push_str("                state.noise_shot_w_prev[k] = w_new;\n");
                rhs_stamp.push_str("                state.noise_shot_last_i_n[k] = i_n;\n");
                rhs_stamp.push_str("                let ni = NOISE_SHOT_NODE_I[k];\n");
                rhs_stamp.push_str("                let nj = NOISE_SHOT_NODE_J[k];\n");
                rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
                rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
                rhs_stamp.push_str("            }\n");
                rhs_stamp.push_str("        } else {\n");
                rhs_stamp.push_str("            state.noise_shot_w_prev = [0.0; NOISE_SHOT_N];\n");
                rhs_stamp
                    .push_str("            state.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
                rhs_stamp.push_str("        }\n");
            }
        }
        if flicker_n > 0 {
            rhs_stamp.push_str(
                "        // Flicker (1/f): white draw → sqrt(2·KF/K_pink)·|I|^(AF/2) scale →\n",
            );
            rhs_stamp
                .push_str("        // Kellett 7-pole pink filter → RHS. `|I_prev|` comes from\n");
            rhs_stamp.push_str(
                "        // `state.i_nl_prev` (one-sample lag, same as shot). The per-\n",
            );
            rhs_stamp.push_str(
                "        // source sqrt(KF) and AF/2 are compile-time baked so the hot\n",
            );
            rhs_stamp.push_str("        // loop is branch-free.\n");
            rhs_stamp.push_str("        let flicker_scale = state.noise_flicker_scale * state.noise_gain * state.flicker_gain;\n");
            rhs_stamp.push_str("        if flicker_scale != 0.0 {\n");
            rhs_stamp.push_str("            for k in 0..NOISE_FLICKER_N {\n");
            rhs_stamp.push_str(
                "                let i_abs = state.i_nl_prev[NOISE_FLICKER_SLOT_IDX[k]].abs();\n",
            );
            rhs_stamp.push_str("                if i_abs < 1e-15 { continue; }\n");
            rhs_stamp.push_str("                let white = gaussian(&mut state.noise_flicker_rng[k], &mut state.noise_flicker_gaussian_cache[k]);\n");
            rhs_stamp.push_str("                let pink = kellett_pink(white, &mut state.noise_flicker_state[k]);\n");
            // #3: specialize the flicker exponent AF/2 when it is uniform.
            // AF/2 == 1.0 (resistor-style AF=2) is identity; AF/2 == 0.5
            // (junction AF=1) is sqrt — both far cheaper than a general powf.
            // Uniform in every shipped circuit; mixed/other exponents keep powf.
            // Stays branch-free (the choice is baked at codegen).
            {
                let hf: Vec<f64> = ir
                    .noise
                    .flicker_sources
                    .iter()
                    .map(|s| 0.5 * s.af)
                    .collect();
                let base = if !hf.is_empty() && hf.iter().all(|&h| h == 1.0) {
                    "i_abs"
                } else if !hf.is_empty() && hf.iter().all(|&h| h == 0.5) {
                    "i_abs.sqrt()"
                } else {
                    "i_abs.powf(NOISE_FLICKER_HALF_AF[k])"
                };
                rhs_stamp.push_str(&format!("                let amp = flicker_scale * NOISE_FLICKER_SQRT_KF[k] * {base};\n"));
            }
            if be_primary {
                rhs_stamp.push_str(
                    "                let i_n = amp * pink; // BE-primary: single-draw (BE damps z=-1)\n",
                );
            } else {
                rhs_stamp.push_str(
                    "                // Nyquist anti-alias pair on the pink output: the Kellett\n\
                     \x20               // cascade leaves ~-14 dB at fs/2, which the trap z=-1 pole\n\
                     \x20               // on resistive junction nodes amplifies and the device\n\
                     \x20               // nonlinearity folds into the audio band. The pair-sum\n\
                     \x20               // (x cos²(πf/fs)) zeroes fs/2 and leaves audio unchanged.\n",
                );
                rhs_stamp.push_str("                let w_new = 0.5 * amp * pink;\n");
                rhs_stamp
                    .push_str("                let i_n = w_new + state.noise_flicker_w_prev[k];\n");
                rhs_stamp.push_str("                state.noise_flicker_w_prev[k] = w_new;\n");
            }
            rhs_stamp.push_str("                state.noise_flicker_last_i_n[k] = i_n;\n");
            rhs_stamp.push_str("                let ni = NOISE_FLICKER_NODE_I[k];\n");
            rhs_stamp.push_str("                let nj = NOISE_FLICKER_NODE_J[k];\n");
            rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
            rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
            rhs_stamp.push_str("            }\n");
            rhs_stamp.push_str("        } else {\n");
            rhs_stamp
                .push_str("            state.noise_flicker_last_i_n = [0.0; NOISE_FLICKER_N];\n");
            rhs_stamp.push_str("        }\n");
        }
        if opamp_n > 0 {
            // Op-amp input-referred noise (Phase 4). Three Norton streams
            // per source — en at in+, in+ at in+, in- at in-. All three use
            // two-draw Nyquist anti-alias (input nodes often lack a shunt
            // cap to ground; same precaution as thermal).
            rhs_stamp.push_str(
                "        // Op-amp en/in: 3 streams per source, two-draw Nyquist anti-alias.\n",
            );
            rhs_stamp
                .push_str("        // en amp = EN · noise_opamp_en_g_diag · sqrt(2·fs)  at in+\n");
            rhs_stamp.push_str(
                "        // in amp = IN · sqrt(2·fs)                          at in+ and in-\n",
            );
            rhs_stamp
                .push_str("        let oa_scale = state.opamp_input_gain * state.noise_gain;\n");
            if be_primary {
                // BE-primary: single-draw at full scale — noise_opamp_sqrt_fs
                // is already the halved sqrt(0.5*fs) per-draw factor.
                rhs_stamp.push_str("        let oa_scale_half = oa_scale;\n");
            } else {
                rhs_stamp.push_str("        let oa_scale_half = oa_scale * 0.5;\n");
            }
            rhs_stamp.push_str("        if oa_scale_half != 0.0 {\n");
            rhs_stamp.push_str("            let sqrt_2fs = state.noise_opamp_sqrt_fs;\n");
            rhs_stamp.push_str("            for k in 0..NOISE_OPAMP_N {\n");
            rhs_stamp.push_str("                let np = NOISE_OPAMP_NODE_PLUS[k];\n");
            rhs_stamp.push_str("                let nm = NOISE_OPAMP_NODE_MINUS[k];\n");
            rhs_stamp.push_str("                let en = NOISE_OPAMP_EN[k];\n");
            rhs_stamp.push_str("                let in_a = NOISE_OPAMP_IN[k];\n");
            rhs_stamp.push_str("                // en stream — voltage source in series with in+, Norton via G_diag.\n");
            rhs_stamp.push_str("                if en > 0.0 && np > 0 {\n");
            rhs_stamp
                .push_str("                    let g_diag = state.noise_opamp_en_g_diag[k];\n");
            rhs_stamp.push_str("                    let amp = en * g_diag * sqrt_2fs;\n");
            rhs_stamp.push_str("                    let g = gaussian(&mut state.noise_opamp_en_rng[k], &mut state.noise_opamp_en_gaussian_cache[k]);\n");
            rhs_stamp.push_str("                    let w_new = oa_scale_half * amp * g;\n");
            if be_primary {
                rhs_stamp
                    .push_str("                    let i_n = w_new; // BE-primary: single-draw\n");
            } else {
                rhs_stamp.push_str(
                    "                    let i_n = w_new + state.noise_opamp_en_w_prev[k];\n",
                );
                rhs_stamp.push_str("                    state.noise_opamp_en_w_prev[k] = w_new;\n");
            }
            rhs_stamp.push_str("                    state.noise_opamp_en_last_i_n[k] = i_n;\n");
            rhs_stamp.push_str("                    rhs[np - 1] += i_n;\n");
            rhs_stamp.push_str("                } else {\n");
            rhs_stamp.push_str("                    state.noise_opamp_en_last_i_n[k] = 0.0;\n");
            rhs_stamp.push_str("                }\n");
            rhs_stamp.push_str(
                "                // in+ stream — current source at non-inverting input.\n",
            );
            rhs_stamp.push_str("                if in_a > 0.0 && np > 0 {\n");
            rhs_stamp.push_str("                    let amp = in_a * sqrt_2fs;\n");
            rhs_stamp.push_str("                    let g = gaussian(&mut state.noise_opamp_in_rng[2 * k], &mut state.noise_opamp_in_gaussian_cache[2 * k]);\n");
            rhs_stamp.push_str("                    let w_new = oa_scale_half * amp * g;\n");
            if be_primary {
                rhs_stamp
                    .push_str("                    let i_n = w_new; // BE-primary: single-draw\n");
            } else {
                rhs_stamp.push_str(
                    "                    let i_n = w_new + state.noise_opamp_in_w_prev[2 * k];\n",
                );
                rhs_stamp
                    .push_str("                    state.noise_opamp_in_w_prev[2 * k] = w_new;\n");
            }
            rhs_stamp.push_str("                    state.noise_opamp_in_last_i_n[2 * k] = i_n;\n");
            rhs_stamp.push_str("                    rhs[np - 1] += i_n;\n");
            rhs_stamp.push_str("                } else {\n");
            rhs_stamp.push_str("                    state.noise_opamp_in_last_i_n[2 * k] = 0.0;\n");
            rhs_stamp.push_str("                }\n");
            rhs_stamp
                .push_str("                // in- stream — current source at inverting input.\n");
            rhs_stamp.push_str("                if in_a > 0.0 && nm > 0 {\n");
            rhs_stamp.push_str("                    let amp = in_a * sqrt_2fs;\n");
            rhs_stamp.push_str("                    let g = gaussian(&mut state.noise_opamp_in_rng[2 * k + 1], &mut state.noise_opamp_in_gaussian_cache[2 * k + 1]);\n");
            rhs_stamp.push_str("                    let w_new = oa_scale_half * amp * g;\n");
            if be_primary {
                rhs_stamp
                    .push_str("                    let i_n = w_new; // BE-primary: single-draw\n");
            } else {
                rhs_stamp.push_str(
                    "                    let i_n = w_new + state.noise_opamp_in_w_prev[2 * k + 1];\n",
                );
                rhs_stamp.push_str(
                    "                    state.noise_opamp_in_w_prev[2 * k + 1] = w_new;\n",
                );
            }
            rhs_stamp
                .push_str("                    state.noise_opamp_in_last_i_n[2 * k + 1] = i_n;\n");
            rhs_stamp.push_str("                    rhs[nm - 1] += i_n;\n");
            rhs_stamp.push_str("                } else {\n");
            rhs_stamp
                .push_str("                    state.noise_opamp_in_last_i_n[2 * k + 1] = 0.0;\n");
            rhs_stamp.push_str("                }\n");
            rhs_stamp.push_str("            }\n");
            rhs_stamp.push_str("        } else {\n");
            rhs_stamp
                .push_str("            state.noise_opamp_en_last_i_n = [0.0; NOISE_OPAMP_N];\n");
            rhs_stamp
                .push_str("            state.noise_opamp_in_last_i_n = [0.0; NOISE_OPAMP_IN_N];\n");
            rhs_stamp.push_str("        }\n");
        }
        if partition_n > 0 {
            // Pentode partition noise (Phase 5). Per-sample amplitude is
            //   noise_shot_scale · sqrt(I_p·I_s/(I_p+I_s)) · PARTITION_F · g
            // where I_p and I_s come from state.i_nl_prev (one-sample lag).
            // Two-draw Nyquist anti-alias: i_n[n] = w[n] + w[n-1], with
            // w = (scale_half)·amp·g. Zero total current → skip (zero amp,
            // pre-Kellett zero-current guard convention).
            rhs_stamp.push_str("        // Pentode partition: i_n = w_new + w_prev, w_new = (shot_scale * shot_gain * noise_gain / 2)\n");
            rhs_stamp.push_str("        //                                                 · sqrt(I_p·I_s/(I_p+I_s)) · PARTITION_F · N(0,1).\n");
            rhs_stamp.push_str(
                "        // Reuses shot_gain (partition is shot at a different barrier).\n",
            );
            if be_primary {
                // BE-primary: single-draw at full scale — noise_shot_scale is
                // already halved to sqrt(Q_E*fs) (the trap per-draw amplitude).
                rhs_stamp.push_str("        let part_scale_half = state.noise_shot_scale * state.noise_gain * state.shot_gain;\n");
            } else {
                rhs_stamp.push_str("        let part_scale_half = state.noise_shot_scale * state.noise_gain * state.shot_gain * 0.5;\n");
            }
            rhs_stamp.push_str("        if part_scale_half != 0.0 {\n");
            rhs_stamp.push_str("            for k in 0..NOISE_PARTITION_N {\n");
            rhs_stamp.push_str(
                "                let ip = state.i_nl_prev[NOISE_PARTITION_IP_SLOT[k]].abs();\n",
            );
            rhs_stamp.push_str(
                "                let is_c = state.i_nl_prev[NOISE_PARTITION_IS_SLOT[k]].abs();\n",
            );
            rhs_stamp.push_str("                let total = ip + is_c;\n");
            rhs_stamp.push_str("                if total < 1e-15 {\n");
            rhs_stamp.push_str(
                "                    // Pre-bias / unbiased: emit zero, keep w_prev so the\n",
            );
            rhs_stamp.push_str(
                "                    // first non-zero sample doesn't double-count the lag.\n",
            );
            rhs_stamp.push_str("                    state.noise_partition_last_i_n[k] = 0.0;\n");
            rhs_stamp.push_str("                    continue;\n");
            rhs_stamp.push_str("                }\n");
            rhs_stamp.push_str("                let psd_coef = ip * is_c / total;\n");
            rhs_stamp.push_str("                let g = gaussian(&mut state.noise_partition_rng[k], &mut state.noise_partition_gaussian_cache[k]);\n");
            rhs_stamp.push_str("                let w_new = part_scale_half * psd_coef.sqrt() * NOISE_PARTITION_F[k] * g;\n");
            if be_primary {
                rhs_stamp.push_str("                let i_n = w_new; // BE-primary: single-draw\n");
            } else {
                rhs_stamp.push_str(
                    "                let i_n = w_new + state.noise_partition_w_prev[k];\n",
                );
                rhs_stamp.push_str("                state.noise_partition_w_prev[k] = w_new;\n");
            }
            rhs_stamp.push_str("                state.noise_partition_last_i_n[k] = i_n;\n");
            rhs_stamp.push_str("                let ni = NOISE_PARTITION_NODE_I[k];\n");
            rhs_stamp.push_str("                let nj = NOISE_PARTITION_NODE_J[k];\n");
            rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
            rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
            rhs_stamp.push_str("            }\n");
            rhs_stamp.push_str("        } else {\n");
            rhs_stamp.push_str(
                "            state.noise_partition_last_i_n = [0.0; NOISE_PARTITION_N];\n",
            );
            rhs_stamp.push_str("        }\n");
        }
        if r_flicker_n > 0 {
            // Resistor flicker (Hooge bias-squared, Phase 3.5). Reads
            // `state.v_prev` directly to compute live resistor current.
            // Zero current → continue (skips RNG advance + Kellett tick),
            // so unbiased resistors emit no excess 1/f. Same convention
            // as junction flicker.
            rhs_stamp.push_str("        // Resistor flicker (Hooge bias-squared, Phase 3.5):\n");
            rhs_stamp.push_str("        // i_R = (V_+ − V_−)/R from v_prev → amp = sqrt(2·KF/K_pink)·|i_R|^(AF/2)\n");
            rhs_stamp.push_str(
                "        // → Kellett 7-pole pink → RHS. Zero current → zero excess 1/f.\n",
            );
            rhs_stamp.push_str(
                "        // Shares `flicker_gain` with junction flicker so a single mute\n",
            );
            rhs_stamp.push_str("        // call silences all 1/f character.\n");
            rhs_stamp.push_str("        let r_fl_scale = state.noise_r_flicker_sqrt_fs * state.noise_gain * state.flicker_gain;\n");
            rhs_stamp.push_str("        if r_fl_scale != 0.0 {\n");
            rhs_stamp.push_str("            for k in 0..NOISE_R_FLICKER_N {\n");
            rhs_stamp.push_str("                let ni = NOISE_R_FLICKER_NODE_I[k];\n");
            rhs_stamp.push_str("                let nj = NOISE_R_FLICKER_NODE_J[k];\n");
            rhs_stamp.push_str(
                "                let v_i = if ni > 0 { state.v_prev[ni - 1] } else { 0.0 };\n",
            );
            rhs_stamp.push_str(
                "                let v_j = if nj > 0 { state.v_prev[nj - 1] } else { 0.0 };\n",
            );
            rhs_stamp.push_str(
                "                let i_r = (v_i - v_j) * state.noise_r_flicker_inv_r[k];\n",
            );
            rhs_stamp.push_str("                let i_abs = i_r.abs();\n");
            rhs_stamp.push_str("                if i_abs < 1e-15 { continue; }\n");
            rhs_stamp.push_str("                let white = gaussian(&mut state.noise_r_flicker_rng[k], &mut state.noise_r_flicker_gaussian_cache[k]);\n");
            rhs_stamp.push_str("                let pink = kellett_pink(white, &mut state.noise_r_flicker_state[k]);\n");
            // #3: specialize the resistor-flicker exponent AF/2 when uniform
            // (see junction-flicker note above). Byte-exact for AF/2 == 1.0.
            {
                let hf: Vec<f64> = ir
                    .noise
                    .resistor_flicker_sources
                    .iter()
                    .map(|s| 0.5 * s.af)
                    .collect();
                let base = if !hf.is_empty() && hf.iter().all(|&h| h == 1.0) {
                    "i_abs"
                } else if !hf.is_empty() && hf.iter().all(|&h| h == 0.5) {
                    "i_abs.sqrt()"
                } else {
                    "i_abs.powf(NOISE_R_FLICKER_HALF_AF[k])"
                };
                rhs_stamp.push_str(&format!(
                    "                let amp = r_fl_scale * NOISE_R_FLICKER_SQRT_KF[k] * {base};\n"
                ));
            }
            if be_primary {
                rhs_stamp.push_str(
                    "                let i_n = amp * pink; // BE-primary: single-draw (BE damps z=-1)\n",
                );
            } else {
                rhs_stamp.push_str(
                    "                // Nyquist anti-alias pair (see junction-flicker stamp).\n",
                );
                rhs_stamp.push_str("                let w_new = 0.5 * amp * pink;\n");
                rhs_stamp.push_str(
                    "                let i_n = w_new + state.noise_r_flicker_w_prev[k];\n",
                );
                rhs_stamp.push_str("                state.noise_r_flicker_w_prev[k] = w_new;\n");
            }
            rhs_stamp.push_str("                state.noise_r_flicker_last_i_n[k] = i_n;\n");
            rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
            rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
            rhs_stamp.push_str("            }\n");
            rhs_stamp.push_str("        } else {\n");
            rhs_stamp.push_str(
                "            state.noise_r_flicker_last_i_n = [0.0; NOISE_R_FLICKER_N];\n",
            );
            rhs_stamp.push_str("        }\n");
        }
        rhs_stamp.push_str("    } else {\n");
        rhs_stamp.push_str(
            "        // noise_enabled=false: clear caches so a future BE replay during a\n",
        );
        rhs_stamp.push_str(
            "        // disabled-noise span doesn't re-inject the last enabled-mode i_n.\n",
        );
        rhs_stamp.push_str("        state.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            rhs_stamp.push_str("        state.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            rhs_stamp.push_str("        state.noise_flicker_last_i_n = [0.0; NOISE_FLICKER_N];\n");
        }
        if r_flicker_n > 0 {
            rhs_stamp
                .push_str("        state.noise_r_flicker_last_i_n = [0.0; NOISE_R_FLICKER_N];\n");
        }
        if partition_n > 0 {
            rhs_stamp
                .push_str("        state.noise_partition_last_i_n = [0.0; NOISE_PARTITION_N];\n");
        }
        if opamp_n > 0 {
            rhs_stamp.push_str("        state.noise_opamp_en_last_i_n = [0.0; NOISE_OPAMP_N];\n");
            rhs_stamp
                .push_str("        state.noise_opamp_in_last_i_n = [0.0; NOISE_OPAMP_IN_N];\n");
        }
        rhs_stamp.push_str("    }\n");

        // BE-fallback noise replay. Reads cached per-source `i_n` from the
        // arrays populated by `rhs_stamp` and re-stamps into `rhs_be`. Same
        // node table, same sign convention. Gated on `state.noise_enabled`
        // so a disabled-noise span doesn't replay stale-cache values.
        // Trap-MNA 2× compensation is left in place — BE samples are ~+3 dB
        // hot vs strict physics during the short (typically 64-sample
        // cooldown) BE-fallback windows. This is bounded, rare, and far
        // below the dominating signal that triggered BE; preferable to
        // noise dropouts during BE windows. See NOISE.md "BE-fallback
        // noise calibration" for the math.
        let replay_counts = NoiseReplayCounts {
            shot: shot_n,
            flicker: flicker_n,
            r_flicker: r_flicker_n,
            partition: partition_n,
            opamp: opamp_n,
        };
        let mut rhs_stamp_be = String::new();
        rhs_stamp_be.push_str("\n        // BE-fallback noise replay (re-stamps cached trap-stamp i_n into rhs_be).\n");
        rhs_stamp_be.push_str(&emit_noise_replay_body(replay_counts, "rhs_be", "        "));

        // NaN-recovery noise reset: clear the two-draw lag buffer and the
        // BE-replay caches so a NaN-induced state.v_prev = DC_OP recovery
        // also produces a clean noise sequence (no stale draw paired with
        // the post-recovery sample). RNG itself is NOT re-seeded here —
        // determinism contract says set_seed is the only re-seed entry.
        let mut nan_recovery_body = String::new();
        nan_recovery_body.push_str(
            "        // Noise: clear two-draw lag + BE-replay caches (RNG seed preserved).\n",
        );
        nan_recovery_body
            .push_str("        state.noise_thermal_w_prev = [0.0; NOISE_THERMAL_N];\n");
        nan_recovery_body
            .push_str("        state.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            nan_recovery_body.push_str("        state.noise_shot_w_prev = [0.0; NOISE_SHOT_N];\n");
            nan_recovery_body
                .push_str("        state.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_flicker_last_i_n = [0.0; NOISE_FLICKER_N];\n");
            if !be_primary {
                nan_recovery_body
                    .push_str("        state.noise_flicker_w_prev = [0.0; NOISE_FLICKER_N];\n");
            }
            nan_recovery_body
                .push_str("        state.noise_flicker_state = [[0.0; 7]; NOISE_FLICKER_N];\n");
        }
        if r_flicker_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_r_flicker_last_i_n = [0.0; NOISE_R_FLICKER_N];\n");
            if !be_primary {
                nan_recovery_body
                    .push_str("        state.noise_r_flicker_w_prev = [0.0; NOISE_R_FLICKER_N];\n");
            }
            nan_recovery_body
                .push_str("        state.noise_r_flicker_state = [[0.0; 7]; NOISE_R_FLICKER_N];\n");
        }
        if partition_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_partition_w_prev = [0.0; NOISE_PARTITION_N];\n");
            nan_recovery_body
                .push_str("        state.noise_partition_last_i_n = [0.0; NOISE_PARTITION_N];\n");
        }
        if opamp_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_opamp_en_w_prev = [0.0; NOISE_OPAMP_N];\n");
            nan_recovery_body
                .push_str("        state.noise_opamp_in_w_prev = [0.0; NOISE_OPAMP_IN_N];\n");
            nan_recovery_body
                .push_str("        state.noise_opamp_en_last_i_n = [0.0; NOISE_OPAMP_N];\n");
            nan_recovery_body
                .push_str("        state.noise_opamp_in_last_i_n = [0.0; NOISE_OPAMP_IN_N];\n");
        }

        // Reverse lookup populated from each source's `pot_slot`. Pots with
        // no noise source (there aren't any in the current pipeline, but
        // future FA reductions / skip lists may produce them) stay `None`.
        let mut pot_to_noise_slot = vec![None; ir.pots.len()];
        for (k, src) in ir.noise.thermal_sources.iter().enumerate() {
            if let Some(p) = src.pot_slot {
                if p < pot_to_noise_slot.len() {
                    pot_to_noise_slot[p] = Some(k);
                }
            }
        }

        // Same for switches: a per-(switch, component) reverse lookup so
        // the emitted `set_switch_N(position)` can spot-update every
        // R-backed noise slot. C/L components map to `None` by default.
        let mut switch_comp_to_noise_slot: Vec<Vec<Option<usize>>> = ir
            .switches
            .iter()
            .map(|sw| vec![None; sw.components.len()])
            .collect();
        for (k, src) in ir.noise.thermal_sources.iter().enumerate() {
            if let Some((sw, comp)) = src.switch_slot {
                if sw < switch_comp_to_noise_slot.len()
                    && comp < switch_comp_to_noise_slot[sw].len()
                {
                    switch_comp_to_noise_slot[sw][comp] = Some(k);
                }
            }
        }

        // Parallel pot/switch reverse lookups for resistor flicker (Phase 3.5).
        // Sparse — most pot/switch resistors have no `KF` set, so most
        // entries stay `None`. The setters check `Option::Some(k)` exactly
        // like the thermal path.
        let mut pot_to_r_flicker_slot = vec![None; ir.pots.len()];
        for (k, src) in ir.noise.resistor_flicker_sources.iter().enumerate() {
            if let Some(p) = src.pot_slot {
                if p < pot_to_r_flicker_slot.len() {
                    pot_to_r_flicker_slot[p] = Some(k);
                }
            }
        }
        let mut switch_comp_to_r_flicker_slot: Vec<Vec<Option<usize>>> = ir
            .switches
            .iter()
            .map(|sw| vec![None; sw.components.len()])
            .collect();
        for (k, src) in ir.noise.resistor_flicker_sources.iter().enumerate() {
            if let Some((sw, comp)) = src.switch_slot {
                if sw < switch_comp_to_r_flicker_slot.len()
                    && comp < switch_comp_to_r_flicker_slot[sw].len()
                {
                    switch_comp_to_r_flicker_slot[sw][comp] = Some(k);
                }
            }
        }

        // Phase 4: pot → op-amp en_g_diag refresh lookup. A pot between
        // (node_p, node_q) contributes `+g_pot` to the G-matrix diagonal at
        // BOTH endpoints. For each op-amp source whose `node_plus` equals
        // either endpoint, the pot's setter must update
        // `state.noise_opamp_en_g_diag[oa_idx]` by the conductance delta.
        // Empty lists for pots that don't touch any op-amp's in+ — zero
        // refresh code is emitted in those setters (byte-identical to
        // pre-Phase-4 builds for circuits with fixed-resistor op-amp
        // input networks, which is the common case).
        let mut pot_to_opamp_en_refresh: Vec<Vec<usize>> = vec![Vec::new(); ir.pots.len()];
        for (pot_idx, pot) in ir.pots.iter().enumerate() {
            for (oa_idx, src) in ir.noise.opamp_noise_sources.iter().enumerate() {
                if src.en > 0.0
                    && src.node_plus > 0
                    && (pot.node_p == src.node_plus || pot.node_q == src.node_plus)
                {
                    pot_to_opamp_en_refresh[pot_idx].push(oa_idx);
                }
            }
        }

        // Switches whose R components touch an op-amp in+ → their setters
        // must call refresh_opamp_en_g_diag() (previously missing entirely:
        // only pots hooked the refresh, so a `.switch` R at in+ left the en
        // Norton factor stale).
        let mut switch_to_opamp_en_refresh = vec![false; ir.switches.len()];
        for list in &opamp_dyn_switches {
            for &(s, _) in list {
                if s < switch_to_opamp_en_refresh.len() {
                    switch_to_opamp_en_refresh[s] = true;
                }
            }
        }

        NoiseEmission {
            top_level: top,
            state_fields,
            default_stmts,
            default_fields,
            rhs_stamp,
            rhs_stamp_be,
            reset_body,
            nan_recovery_body,
            set_sample_rate_body: ssr_body,
            methods,
            enabled: true,
            thermal_n,
            replay_counts,
            pot_to_noise_slot,
            switch_comp_to_noise_slot,
            pot_to_r_flicker_slot,
            switch_comp_to_r_flicker_slot,
            switch_to_opamp_en_refresh,
            pot_to_opamp_en_refresh,
        }
    }
}

/// Block-diagonal parasitic-R coupling matrix R_p_full[i][j] for parasitic
/// BJTs in the DK path. Returns 0 for entries outside any parasitic-BJT block,
/// and `[RE, RB+RE, -RC, RB]` for the 2×2 block at each affected slot.
///
/// K_eff = K - R_p_full lets `bjt_evaluate` (intrinsic) replace `bjt_with_parasitics`
/// (inner 2D NR) on the DK path, because v_d = p + K_eff * i then equals the
/// internal junction voltage exactly. Only applied when the slot lacks
/// internal MNA nodes (DK transient path; nodal path expands internal nodes
/// instead, and DC-OP recompute uses node-voltage NR which doesn't touch K).
///
/// Gated on slot.dimension == 2: FA-reduced (1D) BJTs ignore parasitics
/// by design (see ir.rs::detect_forward_active_bjts comment) and their
/// start_idx+1 lands in the next device's slot, not the same BJT's Vbc row.
pub(super) fn parasitic_r_p_dk(ir: &CircuitIR, i: usize, j: usize) -> f64 {
    for slot in &ir.device_slots {
        if let DeviceParams::Bjt(bp) = &slot.params {
            if bp.has_parasitics() && !slot.has_internal_mna_nodes && slot.dimension == 2 {
                let s = slot.start_idx;
                if i == s && j == s {
                    return bp.re;
                }
                if i == s && j == s + 1 {
                    return bp.rb + bp.re;
                }
                if i == s + 1 && j == s {
                    return -bp.rc;
                }
                if i == s + 1 && j == s + 1 {
                    return bp.rb;
                }
            }
        }
    }
    0.0
}

/// Emit the per-slot K_eff adjustment statements (`{var}[s][s] -= RE;` …) for
/// every parasitic-absorbed BJT. Gating is identical to [`parasitic_r_p_dk`]:
/// `has_parasitics() && !has_internal_mna_nodes && dimension == 2`.
///
/// Shared by the three K-rebuild sites that must agree byte-for-byte on the
/// absorption: the baked `K_DEFAULT`/`K_BE_DEFAULT` (via `parasitic_r_p_dk`),
/// the pot/switch `rebuild_matrices` body, and the no-pot `set_sample_rate`
/// template body (passed in as `k_eff_adjust_lines`/`k_be_eff_adjust_lines`
/// context). Returns an empty string when no slot qualifies.
fn k_eff_adjust_stmts(ir: &CircuitIR, var: &str, indent: &str) -> String {
    let mut out = String::new();
    for slot in &ir.device_slots {
        if let DeviceParams::Bjt(bp) = &slot.params {
            if bp.has_parasitics() && !slot.has_internal_mna_nodes && slot.dimension == 2 {
                let s = slot.start_idx;
                let s1 = s + 1;
                out.push_str(&format!(
                    "{indent}{var}[{s}][{s}] -= {re};\n\
                     {indent}{var}[{s}][{s1}] -= {rb_re};\n\
                     {indent}{var}[{s1}][{s}] -= {neg_rc};\n\
                     {indent}{var}[{s1}][{s1}] -= {rb};\n",
                    re = fmt_f64(bp.re),
                    rb_re = fmt_f64(bp.rb + bp.re),
                    neg_rc = fmt_f64(-bp.rc),
                    rb = fmt_f64(bp.rb),
                ));
            }
        }
    }
    out
}

// The junction-temperature advance for a self-heating device now lives in
// `super::helpers::emit_thermal_tj_advance` — a single source of truth shared
// verbatim by the DK and nodal emitters (imported above). See that helper and
// the `thermal_tj_advance_dk_nodal_string_identity` codegen test.
