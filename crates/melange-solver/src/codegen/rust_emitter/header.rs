//! File header and build-provenance emission shared by the DK and nodal paths.

use tera::Context;

use super::RustEmitter;
use crate::codegen::ir::{CircuitIR, DeviceType};
use crate::codegen::CodegenError;

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

/// Glow sub-sample-fire provenance, emitted UNCONDITIONALLY for every
/// glow-bearing deck (not by-presence) so an artifact self-reports WHY the
/// variable-dt glow-strike re-solve is or isn't active — including `dk-route`
/// (the shipped DK route bakes `S=A^-1` and cannot carry it) and
/// `nodal-full-lu:<trigger>` (the Schur sub-path it needs was not taken).
/// `tau_source`/`lit_integrator` are the lit-integration manifest axes, so a
/// measurement can never be silently paired with the wrong build. design review
/// ruling, cross-project review.
pub(super) struct GlowProvenance {
    /// The deck instantiates at least one glow (latched) device.
    pub present: bool,
    /// Requested mode: `auto` | `on` | `off`.
    pub mode: &'static str,
    /// Resolved: the re-solve is actually emitted (nodal-Schur only).
    pub active: bool,
    /// Why: `nodal-schur` | `nodal-full-lu:<trigger>` | `dk-route` | `off`.
    pub reason: String,
    /// Lit discharge tau source: `heuristic` (RS*C_diag lower bound) | `true`.
    pub tau_source: &'static str,
    /// Lit-segment integrator: `be` | `trap`.
    pub lit_integrator: &'static str,
    /// Lit sub-step multiplier (`factor * tau`); 0.5 shipping default.
    pub lit_factor: f64,
}

/// True when the deck instantiates a glow (latched) device.
pub(super) fn deck_has_glow(ir: &CircuitIR) -> bool {
    ir.device_slots
        .iter()
        .any(|s| matches!(s.params, crate::codegen::ir::DeviceParams::Glow(_)))
}

impl GlowProvenance {
    /// DK route: the kernel bakes `S=A^-1`, so a glow deck here is always
    /// `active:false` with reason `dk-route` (or `off` when disabled).
    pub(super) fn for_dk(ir: &CircuitIR) -> Self {
        let present = deck_has_glow(ir);
        let mode = ir.solver_config.subsample_fire_mode.as_str();
        let reason = if present && mode != "off" {
            "dk-route"
        } else {
            "off"
        }
        .to_string();
        GlowProvenance {
            present,
            mode,
            active: false,
            reason,
            tau_source: "heuristic",
            lit_integrator: "be",
            lit_factor: ir.solver_config.subsample_lit_factor,
        }
    }

    /// Nodal route: `active` mirrors the resolved `subsample_fire` flag (already
    /// cleared on the full-LU sub-path by the caller); `reason` names the taken
    /// sub-path, and the trigger when full-LU displaced Schur.
    pub(super) fn for_nodal(ir: &CircuitIR, use_full_nodal: bool, full_lu_trigger: &str) -> Self {
        let present = deck_has_glow(ir);
        let mode = ir.solver_config.subsample_fire_mode.as_str();
        let active = ir.solver_config.subsample_fire;
        let reason = if !present || mode == "off" {
            "off".to_string()
        } else if active {
            "nodal-schur".to_string()
        } else if use_full_nodal {
            format!("nodal-full-lu:{full_lu_trigger}")
        } else {
            "off".to_string()
        };
        GlowProvenance {
            present,
            mode,
            active,
            reason,
            tau_source: "heuristic",
            lit_integrator: "be",
            lit_factor: ir.solver_config.subsample_lit_factor,
        }
    }
}

/// Full RESOLVED DSP-affecting flag set for the human-readable `Build:` line.
///
/// Every entry reflects the value AFTER netlist directives + auto-promotion.
/// Flags whose value cannot change a circuit's DSP are omitted for that circuit
/// (e.g. `opamp-rail` only when clamped op-amps exist, `bjt-fa` only with BJTs)
/// so the line stays signal, not boilerplate.
fn resolved_build_flags(ir: &CircuitIR, glow: &GlowProvenance) -> String {
    let mut build = format!(
        "integration={}, max_iter={}, oversampling={}x",
        ir.integrator_selection.label(),
        ir.effective_max_iter(),
        ir.solver_config.oversampling_factor
    );
    if let Some(rt) = &ir.solver_config.runtime_oversampling {
        let set: Vec<String> = rt.factors.iter().map(|f| format!("{f}x")).collect();
        build.push_str(&format!(" (runtime: {})", set.join("/")));
    }
    // DC blocking is a fourth (5 Hz) output highpass that is otherwise invisible
    // in the header — always disclose it.
    build.push_str(&format!(
        ", dc-block={}",
        if ir.dc_block { "on" } else { "off" }
    ));
    // Noise mode (off/thermal/shot/full) — resolved from --noise + per-device KF.
    build.push_str(&format!(", noise={}", ir.noise.mode.as_str()));
    // The output clamp bound: it changes the emitted DSP, default or flag.
    build.push_str(&format!(
        ", output-clamp=±{} V",
        ir.solver_config.output_clamp_v
    ));
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
    // Glow sub-sample-fire: shown for every glow-bearing deck with its resolved
    // reason (e.g. `subsample-fire=dk-route` = present but inert on this route),
    // not only when active — so the Build: line and the JSON never disagree.
    if glow.present {
        if glow.active {
            // Show the lit sub-step factor on the human line too (design review:
            // a swept condition must be visible in the Build: line, not only JSON).
            build.push_str(&format!(
                ", subsample-fire={} (lit×{})",
                glow.reason, glow.lit_factor
            ));
        } else {
            build.push_str(&format!(", subsample-fire={}", glow.reason));
        }
    }
    build
}

/// One-line machine-readable JSON (embedded in a comment) mirroring the
/// resolved `Build:` flags plus build identity. Hand-formatted — melange-solver
/// has no non-dev `serde_json`, and every value here is controlled (semver,
/// hex/`unknown`, enum tokens, numbers, bools), so no user text is interpolated
/// and no escaping is required.
fn provenance_json(
    ir: &CircuitIR,
    version: &str,
    commit: &str,
    glow: &GlowProvenance,
    nodal_sub_path: Option<crate::codegen::NodalSubPath>,
) -> String {
    let scheme = if ir.integrator_selection.is_backward_euler() {
        "backward-euler"
    } else {
        "trapezoidal"
    };
    // Solver route (DK vs full-nodal). The commonest "wrong output" confusion
    // is "compiled DK when I expected nodal" (or vice-versa); the route is
    // announced at compile time but was NOT recorded in the artifact, so a
    // `.rs`/plugin could not self-report which numerical path generated it.
    // (The nodal Schur-vs-full-LU sub-path is resolved later inside emit_nodal
    // and travels in the build meta `nodal_sub_path`, not here.)
    let solver = match ir.solver_mode {
        crate::codegen::ir::SolverMode::Dk => "dk",
        crate::codegen::ir::SolverMode::Nodal => "nodal",
    };
    // Exact build identity: a runtime hash of the melange binary that emitted
    // this code. version+commit are a source-side POINTER that cannot see a
    // dirty tree or a different feature/profile build; the exe hash is computed
    // from the artifact and is exact (design review ruling). It
    // is MASKED in the golden harness alongside melange/commit, so a clean
    // rebuild at one commit does not churn codegen diffs.
    let exe = crate::build_identity::current_exe_hash_or_unknown();
    let mut s = String::from("{");
    s.push_str(&format!("\"melange\":\"{version}\","));
    s.push_str(&format!("\"commit\":\"{commit}\","));
    // Algorithm-qualified key: a digest's algorithm IS its unit, so it belongs
    // in the name — a bare `exe` slot invites a consumer to fill it with a
    // different digest of the same file and read a MATCH failure as two
    // binaries (cross-project review). Matches oomox's `..._fnv1a64` convention.
    s.push_str(&format!("\"exe_fnv1a64\":\"{exe}\","));
    s.push_str(&format!("\"solver\":\"{solver}\","));
    // Nodal Schur-vs-full-LU sub-path. Absent on the DK route (None). Recorded
    // so a deck can ASSERT its numerical sub-path: a silent Schur↔full-LU flip
    // (turned by conditioning — any resistor, inductor, or rate — not just a
    // flag) otherwise reaches a consumer only as an unread stderr WARN and has
    // been read as device behaviour (design review).
    if let Some(sp) = nodal_sub_path {
        s.push_str(&format!("\"nodal_subpath\":\"{sp}\","));
    }
    // Fail-loud stamp (design review): when the operator overrode the full-LU
    // section-glow refusal (`--allow-static-glow-on-full-lu`), the section keys
    // ran INERT (the lit branch is the static maintaining line). Record it so a
    // consumer never mistakes a static-line result for the relaxing-section model.
    let glow_sections_inert = matches!(nodal_sub_path, Some(crate::codegen::NodalSubPath::FullLu))
        && ir.solver_config.allow_static_glow_on_full_lu
        && ir.device_slots.iter().any(|slot| {
            matches!(&slot.params,
                crate::codegen::ir::DeviceParams::Glow(gp)
                    if gp.has_sections() || gp.has_d() || gp.ksub > 0.0)
        });
    if glow_sections_inert {
        s.push_str("\"glow_sections\":\"inert (full-lu)\",");
    }
    s.push_str(&format!("\"integration\":\"{scheme}\","));
    s.push_str(&format!(
        "\"integration_source\":\"{}\",",
        ir.integrator_selection.integration_source()
    ));
    if !ir.integration_reason.is_empty() {
        s.push_str(&format!(
            "\"integration_reason\":\"{}\",",
            ir.integration_reason
                .replace('\\', "\\\\")
                .replace('"', "\\\"")
        ));
    }
    s.push_str(&format!(
        "\"backward_euler\":{},",
        ir.integrator_selection.is_backward_euler()
    ));
    s.push_str(&format!("\"max_iter\":{},", ir.effective_max_iter()));
    if !ir.dc_op_rail_pin.is_empty() && ir.dc_op_rail_pin != "none" {
        s.push_str(&format!(
            "\"dc_op_rail_pin\":\"{}\",",
            ir.dc_op_rail_pin.replace('\\', "\\\\").replace('"', "\\\"")
        ));
    }
    if ir.linearize_bias_unconverged {
        s.push_str("\"linearize_bias_unconverged\":true,");
    }
    s.push_str(&format!(
        "\"oversampling\":{},",
        ir.solver_config.oversampling_factor
    ));
    // Runtime-selectable oversampling: `oversampling` above is the default
    // factor; the set, where it came from, the factors below the deck's
    // `.oversampling` recommendation, and each factor's ring verdict.
    if let Some(rt) = &ir.solver_config.runtime_oversampling {
        let list = |v: &[usize]| {
            v.iter()
                .map(|f| f.to_string())
                .collect::<Vec<_>>()
                .join(",")
        };
        let below: Vec<usize> = rt
            .factors
            .iter()
            .copied()
            .filter(|&f| rt.recommended.is_some_and(|r| f < r))
            .collect();
        let reasons: Vec<String> = rt
            .per_factor
            .iter()
            .map(|p| {
                format!(
                    "\"{}\":\"{}\"",
                    p.factor,
                    p.integration_reason
                        .replace('\\', "\\\\")
                        .replace('"', "\\\"")
                )
            })
            .collect();
        s.push_str(&format!(
            "\"oversampling_set\":{{\"factors\":[{}],\"default\":{},\"source\":\"{}\",\"below_recommendation\":[{}],\"integration_reason\":{{{}}}}},",
            list(&rt.factors),
            rt.default,
            rt.source,
            list(&below),
            reasons.join(",")
        ));
    }
    s.push_str(&format!("\"dc_block\":{},", ir.dc_block));
    // The output clamp bound, whether from `--output-clamp` or the default: a
    // consumer asserts it here instead of parsing the emitted clamp literal.
    s.push_str(&format!(
        "\"output_clamp_v\":{},",
        ir.solver_config.output_clamp_v
    ));
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
    // Glow sub-sample-fire: an object, always present for a glow-bearing deck,
    // carrying mode/active/reason plus the lit-integration manifest axes. A
    // consumer reads `active:false, reason:"dk-route"` instead of inferring
    // inertness from an absent key (design review, Q1b).
    if glow.present {
        s.push_str(&format!(
            ",\"subsample_fire\":{{\"mode\":\"{}\",\"active\":{},\"reason\":\"{}\",\"tau_source\":\"{}\",\"lit_integrator\":\"{}\",\"lit_factor\":{}}}",
            glow.mode, glow.active, glow.reason, glow.tau_source, glow.lit_integrator, glow.lit_factor
        ));
    }
    s.push('}');
    s
}

impl RustEmitter {
    pub(super) fn emit_header(
        &self,
        ir: &CircuitIR,
        glow: &GlowProvenance,
        nodal_sub_path: Option<crate::codegen::NodalSubPath>,
    ) -> Result<String, CodegenError> {
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

        // Build-provenance identity. Version is the melange crate version at
        // *melange* build time; commit is `build_identity::GIT_COMMIT` (the
        // same label `melange --version` prints). Local builds between tags
        // are normal, so both are recorded.
        let melange_version = env!("CARGO_PKG_VERSION");
        let melange_commit = crate::build_identity::GIT_COMMIT;
        ctx.insert("melange_version", melange_version);
        ctx.insert("melange_commit", melange_commit);
        // Exact identity of the emitting binary (see provenance_json). Masked in
        // the golden harness, so it never churns codegen diffs.
        ctx.insert(
            "melange_exe",
            crate::build_identity::current_exe_hash_or_unknown(),
        );

        // Provenance line: the FULL RESOLVED flag set — every flag that changes
        // emitted DSP, AFTER netlist-directive application + auto-promotion (not
        // the user-requested subset). The `(auto-promoted)` style is carried by
        // `IntegratorSelection::label()`.
        let build = resolved_build_flags(ir, glow);
        ctx.insert("build", &build);

        // Machine-readable one-line JSON so a consumer can assert the build
        // contract at compile time (replaces oomox's hand-written
        // `oversampling_contract_is_2x` / `dc_block_contract_is_disabled` guards).
        let provenance_json =
            provenance_json(ir, melange_version, melange_commit, glow, nodal_sub_path);
        ctx.insert("provenance_json", &provenance_json);

        self.render("header", &ctx)
    }
}
