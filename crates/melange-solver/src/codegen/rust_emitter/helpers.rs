//! Template data structures, formatting utilities, and helper functions
//! shared across DK and nodal emitter paths.

use serde::Serialize;

use crate::codegen::ir::{CircuitIR, DeviceParams};

/// Inductor data passed to Tera templates.
#[derive(Serialize)]
pub(super) struct InductorTemplateData {
    pub(super) name: String,
    pub(super) node_i: usize,
    pub(super) node_j: usize,
    /// Formatted g_eq string for constants template (empty when not needed)
    pub(super) g_eq: String,
    /// Formatted inductance string for constants template (empty when not needed)
    pub(super) inductance: String,
}

/// Coupled inductor data passed to Tera templates.
#[derive(Serialize)]
pub(super) struct CoupledInductorTemplateData {
    pub(super) name: String,
    l1_node_i: usize,
    l1_node_j: usize,
    l2_node_i: usize,
    l2_node_j: usize,
    l1_inductance: String,
    l2_inductance: String,
    pub(super) coupling: String,
    g_self_1: String,
    g_self_2: String,
    pub(super) g_mutual: String,
}

/// Transformer group data passed to Tera templates.
#[derive(Serialize)]
pub(super) struct TransformerGroupTemplateData {
    pub(super) index: usize,
    pub(super) name: String,
    pub(super) num_windings: usize,
    pub(super) winding_node_i: Vec<usize>,
    pub(super) winding_node_j: Vec<usize>,
    pub(super) inductances: Vec<String>,
    pub(super) coupling_flat: Vec<String>,
    pub(super) y_matrix: Vec<String>,
}

/// Named constant entry for Tera templates.
///
/// Used for `NODE_<name>`, `VSOURCE_<name>_RHS_ROW`, and `POT_<name>_INDEX`
/// constants emitted by `constants.rs.tera`. Kept as a struct with named
/// fields rather than a `(String, usize)` tuple because Tera's tuple-indexing
/// syntax is fragile across versions.
#[derive(Serialize)]
pub(super) struct NamedConstEntry {
    pub(super) name: String,
    pub(super) value: usize,
}

/// Convert `NamedConstantsIR` lists into Tera template data.
pub(super) fn named_const_entries(pairs: &[(String, usize)]) -> Vec<NamedConstEntry> {
    pairs
        .iter()
        .map(|(n, v)| NamedConstEntry {
            name: n.clone(),
            value: *v,
        })
        .collect()
}

/// Estimate the slowest settling time constant (seconds) of the circuit.
///
/// Per-node heuristic: `τ_i = C[i][i] / G[i][i]` when both are positive,
/// where the diagonals already aggregate the node's total cap and
/// conductance to ground (coupling caps between nodes i and j stamp into
/// both `C[i][i] += C` and `C[j][j] += C`, so the diagonal dominates the
/// node's RC time). Returns the maximum τ across the augmented system,
/// clamped to a small floor to avoid zero-length warmups.
///
/// Fast O(n) scan. Not a replacement for a proper eigen-analysis of
/// `C⁻¹G` for tightly-coupled RC chains — see the Phase 5 plan in
/// [oomox_missing_functionality_roadmap.md] — but adequate for the common
/// "one dominant pole somewhere" case that covers most audio circuits.
pub(super) fn estimate_settle_time_seconds(ir: &CircuitIR) -> f64 {
    let n = ir.topology.n;
    let mut tau_max: f64 = 0.0;
    for i in 0..n {
        let g_ii = ir.g(i, i);
        let c_ii = ir.c(i, i);
        if c_ii > 0.0 && g_ii > 0.0 {
            let tau = c_ii / g_ii;
            if tau > tau_max {
                tau_max = tau;
            }
        }
    }
    tau_max
}

/// Translate the estimated settling time into a sample count using a 5τ
/// safety factor (99.3% settled) and rounding up. Minimum 1 sample so the
/// constant is always positive.
pub(super) fn recommended_warmup_samples(ir: &CircuitIR) -> usize {
    let tau = estimate_settle_time_seconds(ir);
    let internal_rate =
        ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
    let samples = (5.0 * tau * internal_rate).ceil() as i64;
    samples.max(1) as usize
}

/// Switch component data passed to Tera templates.
#[derive(Serialize)]
pub(super) struct SwitchCompTemplateData {
    pub(super) node_p: usize,
    pub(super) node_q: usize,
    pub(super) nominal: String,
    pub(super) comp_type: char,
    pub(super) inductor_index: i64, // -1 if not an inductor
}

/// Switch data passed to Tera templates.
#[derive(Serialize)]
pub(super) struct SwitchTemplateData {
    pub(super) index: usize,
    pub(super) num_positions: usize,
    pub(super) num_components: usize,
    pub(super) components: Vec<SwitchCompTemplateData>,
    pub(super) position_rows: Vec<String>,
}

/// Device parameter data passed to Tera templates for runtime-adjustable state fields.
///
/// Each device slot produces one entry. The `params` vec contains (field_suffix, const_name)
/// pairs for parameters that should become CircuitState fields.
/// SIGN, USE_GP, and Gummel-Poon params (VAF/VAR/IKF/IKR) remain as const-only.
#[derive(Serialize)]
pub(super) struct DeviceParamTemplateData {
    /// Device index (0-based)
    pub(super) dev_num: usize,
    /// Device type tag for conditional template logic
    pub(super) device_type: String,
    /// Runtime-adjustable parameter entries: (lowercase_field_suffix, CONST_SUFFIX)
    pub(super) params: Vec<DeviceParamEntry>,
}

/// A single runtime-adjustable device parameter.
#[derive(Serialize)]
pub(super) struct DeviceParamEntry {
    /// Lowercase field name suffix, e.g. "is", "n_vt", "bf"
    pub(super) field_suffix: String,
    /// Uppercase const name suffix, e.g. "IS", "N_VT", "BETA_F"
    pub(super) const_suffix: String,
}

/// Data for a BJT with self-heating enabled (passed to Tera templates).
#[derive(Serialize)]
pub(super) struct SelfHeatingDeviceData {
    /// Device index (0-based)
    pub(super) dev_num: usize,
    /// Start index in M-dimensional NR space (Vbe/Ic slot)
    pub(super) start_idx: usize,
}

/// Collect BJT devices that have self-heating enabled.
pub(super) fn self_heating_device_data(ir: &CircuitIR) -> Vec<SelfHeatingDeviceData> {
    ir.device_slots
        .iter()
        .enumerate()
        .filter_map(|(dev_num, slot)| {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.has_self_heating() {
                    return Some(SelfHeatingDeviceData {
                        dev_num,
                        start_idx: slot.start_idx,
                    });
                }
            }
            None
        })
        .collect()
}

/// Build `DeviceParamTemplateData` for each device slot in the IR.
///
/// Only parameters that should be runtime-adjustable are included.
/// SIGN constants and Gummel-Poon parameters (VAF, VAR, IKF, IKR, USE_GP)
/// are intentionally excluded — they remain as compile-time constants.
pub(super) fn device_param_template_data(ir: &CircuitIR) -> Vec<DeviceParamTemplateData> {
    ir.device_slots
        .iter()
        .enumerate()
        .map(|(dev_num, slot)| {
            let (device_type, params) = match &slot.params {
                DeviceParams::Diode(_) => (
                    "Diode".to_string(),
                    vec![
                        DeviceParamEntry {
                            field_suffix: "is".into(),
                            const_suffix: "IS".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "n_vt".into(),
                            const_suffix: "N_VT".into(),
                        },
                    ],
                ),
                DeviceParams::Bjt(_) => (
                    "Bjt".to_string(),
                    vec![
                        DeviceParamEntry {
                            field_suffix: "is".into(),
                            const_suffix: "IS".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "vt".into(),
                            const_suffix: "VT".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "bf".into(),
                            const_suffix: "BETA_F".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "br".into(),
                            const_suffix: "BETA_R".into(),
                        },
                    ],
                ),
                DeviceParams::Jfet(_) => (
                    "Jfet".to_string(),
                    vec![
                        DeviceParamEntry {
                            field_suffix: "idss".into(),
                            const_suffix: "IDSS".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "vp".into(),
                            const_suffix: "VP".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "lambda".into(),
                            const_suffix: "LAMBDA".into(),
                        },
                    ],
                ),
                DeviceParams::Mosfet(_) => (
                    "Mosfet".to_string(),
                    vec![
                        DeviceParamEntry {
                            field_suffix: "kp".into(),
                            const_suffix: "KP".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "vt".into(),
                            const_suffix: "VT".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "lambda".into(),
                            const_suffix: "LAMBDA".into(),
                        },
                    ],
                ),
                DeviceParams::Tube(_) => (
                    "Tube".to_string(),
                    vec![
                        DeviceParamEntry {
                            field_suffix: "mu".into(),
                            const_suffix: "MU".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "ex".into(),
                            const_suffix: "EX".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "kg1".into(),
                            const_suffix: "KG1".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "kp".into(),
                            const_suffix: "KP".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "kvb".into(),
                            const_suffix: "KVB".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "ig_max".into(),
                            const_suffix: "IG_MAX".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "vgk_onset".into(),
                            const_suffix: "VGK_ONSET".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "lambda".into(),
                            const_suffix: "LAMBDA".into(),
                        },
                    ],
                ),
                DeviceParams::Vca(_) => (
                    "Vca".to_string(),
                    vec![
                        DeviceParamEntry {
                            field_suffix: "vscale".into(),
                            const_suffix: "VSCALE".into(),
                        },
                        DeviceParamEntry {
                            field_suffix: "g0".into(),
                            const_suffix: "G0".into(),
                        },
                    ],
                ),
            };
            DeviceParamTemplateData {
                dev_num,
                device_type,
                params,
            }
        })
        .collect()
}

// ============================================================================
// Formatting helpers — reduce repetition in string-building code
// ============================================================================

/// Format a float with full precision for codegen constants.
pub(super) fn fmt_f64(v: f64) -> String {
    if v.is_infinite() {
        if v > 0.0 {
            "f64::INFINITY".to_string()
        } else {
            "f64::NEG_INFINITY".to_string()
        }
    } else if v.is_nan() {
        "f64::NAN".to_string()
    } else {
        format!("{:.17e}", v)
    }
}

/// Format a matrix as rows of comma-separated full-precision floats.
///
/// `rows` x `cols` elements are read from `get(i, j)`.
pub(super) fn format_matrix_rows(rows: usize, cols: usize, get: impl Fn(usize, usize) -> f64) -> Vec<String> {
    (0..rows)
        .map(|i| {
            (0..cols)
                .map(|j| fmt_f64(get(i, j)))
                .collect::<Vec<_>>()
                .join(", ")
        })
        .collect()
}

/// Build `InductorTemplateData` from IR inductors.
///
/// When `with_g_eq` is true, each entry includes the formatted g_eq value
/// (needed by the constants template). Otherwise g_eq is left empty.
pub(super) fn inductor_template_data(ir: &CircuitIR, with_g_eq: bool) -> Vec<InductorTemplateData> {
    ir.inductors
        .iter()
        .map(|ind| InductorTemplateData {
            name: ind.name.clone(),
            node_i: ind.node_i,
            node_j: ind.node_j,
            g_eq: if with_g_eq {
                fmt_f64(ind.g_eq)
            } else {
                String::new()
            },
            inductance: if with_g_eq {
                fmt_f64(ind.inductance)
            } else {
                String::new()
            },
        })
        .collect()
}

/// Build `CoupledInductorTemplateData` from IR coupled inductors.
pub(super) fn coupled_inductor_template_data(ir: &CircuitIR) -> Vec<CoupledInductorTemplateData> {
    ir.coupled_inductors
        .iter()
        .map(|ci| CoupledInductorTemplateData {
            name: ci.name.clone(),
            l1_node_i: ci.l1_node_i,
            l1_node_j: ci.l1_node_j,
            l2_node_i: ci.l2_node_i,
            l2_node_j: ci.l2_node_j,
            l1_inductance: fmt_f64(ci.l1_inductance),
            l2_inductance: fmt_f64(ci.l2_inductance),
            coupling: fmt_f64(ci.coupling),
            g_self_1: fmt_f64(ci.g_self_1),
            g_self_2: fmt_f64(ci.g_self_2),
            g_mutual: fmt_f64(ci.g_mutual),
        })
        .collect()
}

/// Build `TransformerGroupTemplateData` from IR transformer groups.
pub(super) fn transformer_group_template_data(ir: &CircuitIR) -> Vec<TransformerGroupTemplateData> {
    ir.transformer_groups
        .iter()
        .enumerate()
        .map(|(idx, g)| TransformerGroupTemplateData {
            index: idx,
            name: g.name.clone(),
            num_windings: g.num_windings,
            winding_node_i: g.winding_node_i.clone(),
            winding_node_j: g.winding_node_j.clone(),
            inductances: g.inductances.iter().map(|v| fmt_f64(*v)).collect(),
            coupling_flat: g.coupling_flat.iter().map(|v| fmt_f64(*v)).collect(),
            y_matrix: g.y_matrix.iter().map(|v| fmt_f64(*v)).collect(),
        })
        .collect()
}

/// Emit a section banner comment.
pub(super) fn section_banner(title: &str) -> String {
    format!(
        "// =============================================================================\n\
         // {}\n\
         // =============================================================================\n\n",
        title
    )
}

/// Collapse 3+ consecutive blank lines down to 2.
///
/// Post-processing pass applied to generated code for cleaner output.
pub(super) fn collapse_blank_lines(code: &str) -> String {
    let mut result = String::with_capacity(code.len());
    let mut blank_count = 0;
    for line in code.lines() {
        if line.trim().is_empty() {
            blank_count += 1;
            if blank_count <= 2 {
                result.push('\n');
            }
        } else {
            blank_count = 0;
            result.push_str(line);
            result.push('\n');
        }
    }
    result
}

/// Emit a single `const DEVICE_{n}_{suffix}: f64 = ...;` line.
pub(super) fn emit_device_const(code: &mut String, dev_num: usize, suffix: &str, value: f64) {
    code.push_str(&format!(
        "const DEVICE_{}_{}: f64 = {};\n",
        dev_num,
        suffix,
        fmt_f64(value)
    ));
}

/// Arguments for a pentode helper call, computed once per slot and shared
/// across the DK, nodal-Schur, and nodal-full-LU codegen paths.
///
/// Every `tube_*_<suffix>(..)` helper in `device_tube.rs.tera` has a fixed
/// parameter order for a given `screen_form`, but the parameter COUNT and
/// CONTENT differs between families:
///
///   - Derk §4.4 Rational / DerkE §4.5 Exponential: 14 args
///     (μ, Ex, Kg1, Kg2, Kp, Kvb, αs, A, β, ig_max, vgk_onset + v/v/v)
///   - §5 variable-mu on either Derk base: +3 args (μ_b, svar, ex_b)
///   - Classical Norman Koren: only 11 args (no αs/A/β — those fields are
///     ignored entirely by the Classical helpers; the signature is
///     μ, Ex, Kg1, Kg2, Kp, Kvb, ig_max, vgk_onset + v/v/v)
///
/// Separating the `eval_args`, `ip_args`, `is_args` fields lets each call
/// site stitch together the matching helper signature without re-deriving
/// the `screen_form` decision. `ip_args` and `is_args` omit `ig_max`/
/// `vgk_onset` because `tube_ip_*` / `tube_is_*` helpers take only the
/// plate/screen parameters (grid current lives in the separate `tube_ig`).
pub(super) struct PentodeDispatch {
    /// Helper family suffix: `pentode`, `beam_tetrode`, `pentode_v`,
    /// `beam_tetrode_v`, `pentode_classical`, `pentode_grid_off`,
    /// `beam_tetrode_grid_off`, or `pentode_classical_grid_off`.
    pub(super) suffix: &'static str,
    /// Full argument list (AFTER the voltage prefix) for a
    /// `tube_evaluate_{suffix}(..)` call. For 3D helpers the voltage prefix is
    /// `v_d{s}, v_d{s1}, v_d{s2}`; for grid-off (2D) helpers it's the same
    /// three-voltage signature but the third argument is always
    /// `DEVICE_{d}_VG2K_FROZEN` (emitted at the call site) rather than a
    /// live NR state — see [`PentodeDispatch::dim_count`] for how dispatch
    /// sites decide how many NR dimensions to stamp.
    pub(super) eval_args: String,
    /// Full argument list for `tube_ip_{suffix}(..)` calls from the nodal
    /// full-LU final `i_nl` stamping pass.
    pub(super) ip_args: String,
    /// Full argument list for `tube_is_{suffix}(..)` calls from the nodal
    /// full-LU final `i_nl` stamping pass.
    pub(super) is_args: String,
    /// Number of NR dimensions this slot contributes: 3 for sharp pentodes
    /// (Vgk→Ip, Vpk→Ig2, Vg2k→Ig1), 2 for grid-off reduced pentodes
    /// (Vgk→Ip, Vpk→Ig2; Ig1 dropped, Vg2k frozen). Dispatch sites use
    /// [`PentodeDispatch::is_grid_off`] directly for the two-way branch;
    /// this field is kept alongside for call sites that need the numeric
    /// dimension count (e.g. generic jdev-loop emission paths).
    #[allow(dead_code)]
    pub(super) dim_count: usize,
    /// True if this is a grid-off reduced pentode (`TubeKind::
    /// SharpPentodeGridOff`). Signals dispatch sites to:
    ///   - use `DEVICE_{d}_VG2K_FROZEN` as the third voltage argument in
    ///     helper calls (instead of `v_d{s+2}`),
    ///   - emit only 2×2 Jacobian stamps (upper-left of the returned `[f64;4]`),
    ///   - skip Ig1 output entirely (the helper returns `(ip, ig2, jac4)`).
    pub(super) is_grid_off: bool,
}

pub(super) fn pentode_dispatch(
    tp: &crate::device_types::TubeParams,
    dev_num: usize,
) -> PentodeDispatch {
    use crate::device_types::ScreenForm;
    // Grid-off reduction is orthogonal to screen_form and variable-mu in the
    // math, but phase 1b only supports sharp (non-variable-mu) grid-off. The
    // variable-mu + grid-off combo is rejected here: variable-mu tubes exist
    // specifically to model dynamic bias (AGC sidechain), which is the exact
    // opposite of the static-cutoff assumption grid-off reduction makes.
    let is_grid_off = tp.is_grid_off_pentode();
    if is_grid_off && tp.is_variable_mu() {
        unreachable!(
            "variable-mu grid-off pentode is not a supported reduction \
             (variable-mu needs dynamic bias, grid-off assumes static cutoff)"
        );
    }
    let suffix = match (tp.screen_form, tp.is_variable_mu(), is_grid_off) {
        (ScreenForm::Rational, false, false) => "pentode",
        (ScreenForm::Exponential, false, false) => "beam_tetrode",
        (ScreenForm::Classical, false, false) => "pentode_classical",
        (ScreenForm::Rational, true, false) => "pentode_v",
        (ScreenForm::Exponential, true, false) => "beam_tetrode_v",
        (ScreenForm::Rational, false, true) => "pentode_grid_off",
        (ScreenForm::Exponential, false, true) => "beam_tetrode_grid_off",
        (ScreenForm::Classical, false, true) => "pentode_classical_grid_off",
        (ScreenForm::Classical, true, _) => {
            // Variable-mu + Classical is rejected by `TubeParams::validate()`
            // — Reefman §5 is built on the Derk softplus structure, not the
            // Classical arctan knee. If we reach this branch the validator
            // is broken; fail loud rather than emit junk helper names.
            unreachable!(
                "variable-mu Classical Koren pentode should be rejected by TubeParams::validate()"
            );
        }
        (_, true, true) => {
            // Already rejected above by the `is_grid_off && is_variable_mu`
            // guard; this arm exists only to satisfy exhaustiveness.
            unreachable!("variable-mu grid-off already rejected above");
        }
    };

    // Variable-mu suffix (appended to Derk base args). Empty for sharp.
    let vmu_suffix = if tp.is_variable_mu() {
        format!(", DEVICE_{dev_num}_MU_B, DEVICE_{dev_num}_SVAR, DEVICE_{dev_num}_EX_B")
    } else {
        String::new()
    };

    let (eval_args, ip_args, is_args) = match tp.screen_form {
        ScreenForm::Rational | ScreenForm::Exponential => {
            // Derk / DerkE family: full 11-arg plate-model signature +
            // ig_max/vgk_onset for the combined evaluate form.
            let shared = format!(
                "state.device_{dev_num}_mu, state.device_{dev_num}_ex, \
                 state.device_{dev_num}_kg1, DEVICE_{dev_num}_KG2, \
                 state.device_{dev_num}_kp, state.device_{dev_num}_kvb, \
                 DEVICE_{dev_num}_ALPHA_S, DEVICE_{dev_num}_A_FACTOR, \
                 DEVICE_{dev_num}_BETA_FACTOR"
            );
            let eval = format!(
                "{shared}, state.device_{dev_num}_ig_max, \
                 state.device_{dev_num}_vgk_onset{vmu_suffix}"
            );
            let ip = format!("{shared}{vmu_suffix}");
            // tube_is_* drops the A_FACTOR but keeps the rest; mirror the
            // helper signature by reconstructing the `is` arg list directly.
            let is = format!(
                "state.device_{dev_num}_mu, state.device_{dev_num}_ex, \
                 state.device_{dev_num}_kg1, DEVICE_{dev_num}_KG2, \
                 state.device_{dev_num}_kp, state.device_{dev_num}_kvb, \
                 DEVICE_{dev_num}_ALPHA_S, DEVICE_{dev_num}_BETA_FACTOR{vmu_suffix}"
            );
            (eval, ip, is)
        }
        ScreenForm::Classical => {
            // Classical Koren: 6-parameter signature (μ, Ex, Kg1, Kg2, Kp,
            // Kvb). No αs/A/β — those fields are unused by this equation
            // family and MUST NOT be passed to the helper functions.
            let shared = format!(
                "state.device_{dev_num}_mu, state.device_{dev_num}_ex, \
                 state.device_{dev_num}_kg1, DEVICE_{dev_num}_KG2, \
                 state.device_{dev_num}_kp, state.device_{dev_num}_kvb"
            );
            let eval = format!(
                "{shared}, state.device_{dev_num}_ig_max, \
                 state.device_{dev_num}_vgk_onset"
            );
            // tube_ip_pentode_classical / tube_is_pentode_classical take the
            // same 6-parameter plate-model list (no ig_max/vgk_onset).
            let ip_is = shared.clone();
            (eval, ip_is.clone(), ip_is)
        }
    };

    let dim_count = if is_grid_off { 2 } else { 3 };
    PentodeDispatch {
        suffix,
        eval_args,
        ip_args,
        is_args,
        dim_count,
        is_grid_off,
    }
}

/// Emit one pentode / beam-tetrode NR block for the DK Schur dispatch family
/// (both the primary `generate_solve_nonlinear` path and the BE fallback
/// `emit_dk_device_eval_for_nodal_schur_indented` path).
///
/// Emits a single `tube_evaluate_{suffix}(..)` call bound to
/// `(i_dev{s}, i_dev{s1}[, i_dev{s2}], pentode{d}_jac)`, followed by the
/// `jdev_{r}_{c}` Jacobian let-bindings (2×2 for grid-off slots where Vg2k
/// is frozen, 3×3 for sharp / variable-mu / Classical). All arguments are
/// threaded through [`pentode_dispatch`] so the helper family and argument
/// list are derived once from `tp`.
///
/// The nodal full-LU sites ([`emit_nodal_device_evaluation_body`] /
/// [`emit_nodal_device_evaluation_final`]) use a different stamping scheme
/// (`j_dev[r*m+c]` flat array + `vgk`/`vpk`/`vg2k` local names + wrapped
/// `{{ // Pentode N }}` block) and are not shared with this helper.
pub(super) fn emit_pentode_nr_dk_stamp(
    code: &mut String,
    tp: &crate::device_types::TubeParams,
    dev_num: usize,
    s: usize,
    indent: &str,
) {
    let d = dev_num;
    let s1 = s + 1;
    let dispatch = pentode_dispatch(tp, d);
    let suffix = dispatch.suffix;
    let eval_args = &dispatch.eval_args;
    if dispatch.is_grid_off {
        // Grid-off 2D reduction: Vg2k is DEVICE_{d}_VG2K_FROZEN, Ig1 dropped.
        // Wrapper helper returns (ip, ig2, [f64;4]) — 2×2 stamps only.
        code.push_str(&format!(
            "{indent}let (i_dev{s}, i_dev{s1}, pentode{d}_jac) = tube_evaluate_{suffix}(v_d{s}, v_d{s1}, DEVICE_{d}_VG2K_FROZEN, {eval_args});\n"
        ));
        code.push_str(&format!(
            "{indent}let jdev_{s}_{s} = pentode{d}_jac[0];\n\
             {indent}let jdev_{s}_{s1} = pentode{d}_jac[1];\n\
             {indent}let jdev_{s1}_{s} = pentode{d}_jac[2];\n\
             {indent}let jdev_{s1}_{s1} = pentode{d}_jac[3];\n"
        ));
    } else {
        let s2 = s + 2;
        code.push_str(&format!(
            "{indent}let (i_dev{s}, i_dev{s1}, i_dev{s2}, pentode{d}_jac) = tube_evaluate_{suffix}(v_d{s}, v_d{s1}, v_d{s2}, {eval_args});\n"
        ));
        // Row-major 3×3 Jacobian: [dIp/dVgk, dIp/dVpk, dIp/dVg2k,
        //                          dIg2/dVgk, dIg2/dVpk, dIg2/dVg2k,
        //                          dIg1/dVgk, dIg1/dVpk, dIg1/dVg2k]
        code.push_str(&format!(
            "{indent}let jdev_{s}_{s} = pentode{d}_jac[0];\n\
             {indent}let jdev_{s}_{s1} = pentode{d}_jac[1];\n\
             {indent}let jdev_{s}_{s2} = pentode{d}_jac[2];\n\
             {indent}let jdev_{s1}_{s} = pentode{d}_jac[3];\n\
             {indent}let jdev_{s1}_{s1} = pentode{d}_jac[4];\n\
             {indent}let jdev_{s1}_{s2} = pentode{d}_jac[5];\n\
             {indent}let jdev_{s2}_{s} = pentode{d}_jac[6];\n\
             {indent}let jdev_{s2}_{s1} = pentode{d}_jac[7];\n\
             {indent}let jdev_{s2}_{s2} = pentode{d}_jac[8];\n"
        ));
    }
}

// ============================================================================
// Oversampling configuration
// ============================================================================

/// Half-band filter coefficients from melange-primitives/src/oversampling.rs.
/// 3-section (~80dB rejection, balanced quality/cost).
const HB_3SECTION: [f64; 3] = [0.036681502163648017, 0.2746317593794541, 0.7856959333713522];

/// 2-section half-band (minimal CPU, ~60dB rejection) for 4x outer stage.
const HB_2SECTION: [f64; 2] = [0.07986642623635751, 0.5453536510716122];

/// Oversampling stage configuration.
pub(super) struct OversamplingInfo {
    /// Number of allpass sections per filter (for inner 2x stage)
    pub(super) num_sections: usize,
    /// Coefficients for inner 2x stage
    pub(super) coeffs: Vec<f64>,
    /// State size per filter = num_sections * 2 (x1, y1 per section)
    pub(super) state_size: usize,
    /// For 4x: state size of the outer 2x stage
    pub(super) state_size_outer: usize,
    /// For 4x: coefficients for outer 2x stage
    pub(super) coeffs_outer: Vec<f64>,
    /// For 4x: number of sections in outer stage
    pub(super) num_sections_outer: usize,
}

/// Get oversampling configuration for a given factor.
pub(super) fn oversampling_info(factor: usize) -> OversamplingInfo {
    match factor {
        2 => {
            let num_sections = HB_3SECTION.len();
            OversamplingInfo {
                num_sections,
                coeffs: HB_3SECTION.to_vec(),
                state_size: num_sections * 2,
                state_size_outer: 0,
                coeffs_outer: Vec::new(),
                num_sections_outer: 0,
            }
        }
        4 => {
            // Inner 2x stage uses 3-section, outer uses 2-section
            let inner = HB_3SECTION.len();
            let outer = HB_2SECTION.len();
            OversamplingInfo {
                num_sections: inner,
                coeffs: HB_3SECTION.to_vec(),
                state_size: inner * 2,
                state_size_outer: outer * 2,
                coeffs_outer: HB_2SECTION.to_vec(),
                num_sections_outer: outer,
            }
        }
        _ => OversamplingInfo {
            num_sections: 0,
            coeffs: Vec::new(),
            state_size: 0,
            state_size_outer: 0,
            coeffs_outer: Vec::new(),
            num_sections_outer: 0,
        },
    }
}
