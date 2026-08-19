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
    // Effectively-floating node floor. A node whose total conductance to the
    // rest of the circuit is below this is high-impedance to the point of being
    // disconnected for audio purposes (>100 MΩ), so its RC time is not audible
    // settling and must not gate warmup. The canonical trigger is a `.switch`
    // OFF position encoded as a huge static (e.g. `1e9`): that leg stamps
    // `G[i][i] ≈ 1e-9 S` into an otherwise-capacitive node with no real path to
    // the output, and `C[i][i]/1e-9` blows τ up by ~11 orders (oomox 2026-08-15:
    // steve-1073-eq reported a 28.9-DAY warmup for a circuit that settles in
    // 0.25 s). Real audio node impedances sit far below 100 MΩ (a 22 MΩ grid
    // leak is 4.5e-8 S, an order above this floor), so no real node is excluded
    // — byte-identical warmup for circuits without such a dead leg.
    const G_SETTLE_FLOOR: f64 = 1e-8;
    let n = ir.topology.n;
    let mut tau_max: f64 = 0.0;
    for i in 0..n {
        let g_ii = ir.g(i, i);
        let c_ii = ir.c(i, i);
        if c_ii > 0.0 && g_ii > G_SETTLE_FLOOR {
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
///
/// The unit is HOST-rate `process_sample` calls: each public call advances
/// `OVERSAMPLING_FACTOR` internal solver steps but only `1/sample_rate`
/// seconds of circuit time, so τ converts at the host rate. Multiplying by
/// the oversampling factor here (the pre-2026-07 behavior) made oversampled
/// warmup loops `factor`× longer than the physics requires.
/// Sanity backstop ceiling on the warmup recommendation, in seconds of circuit
/// time. No audio circuit needs more than this to settle (even a slow PSU
/// electrolytic is well inside it); a raw estimate past it is a heuristic
/// blow-up, not a real answer.
const WARMUP_CAP_SECONDS: f64 = 30.0;

pub(super) fn recommended_warmup_samples(ir: &CircuitIR) -> usize {
    let raw = raw_warmup_samples(ir);
    let cap = (WARMUP_CAP_SECONDS * ir.solver_config.sample_rate).ceil() as i64;
    raw.clamp(1, cap.max(1)) as usize
}

/// Uncapped 5τ estimate (may be absurd for a pathological RC). Split out so
/// [`warmup_estimate_capped`] can tell whether the backstop actually fired.
fn raw_warmup_samples(ir: &CircuitIR) -> i64 {
    let tau = estimate_settle_time_seconds(ir);
    (5.0 * tau * ir.solver_config.sample_rate).ceil() as i64
}

/// True when [`recommended_warmup_samples`] hit the sanity cap — i.e. the raw
/// per-node heuristic produced an implausibly large estimate (a moderate
/// switch-OFF static, or a slow node the O(n) estimate can't discount as
/// output-unobservable) and the recommendation is an UPPER BOUND, not a real
/// settle time. Emitted as a companion const so this is never laundered into a
/// plausible-looking number (oomox 2026-08-15: an absurd 28.9-day value
/// announces itself and gets caught; a plausible 30 s value does not).
pub(super) fn warmup_estimate_capped(ir: &CircuitIR) -> bool {
    let cap = (WARMUP_CAP_SECONDS * ir.solver_config.sample_rate).ceil() as i64;
    raw_warmup_samples(ir) > cap.max(1)
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
    pub(super) label: String,
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

/// Data for a device with self-heating enabled (passed to Tera templates).
///
/// Covers BJTs, diodes, and triodes — three flavors that share the same
/// envelope-temperature state field (`state.device_N_tj`) but differ in what
/// downstream fields the thermal update drifts and how NR consumes them:
///
/// * **IS-scaling** (BJT, diode): thermal update also writes
///   `state.device_N_is` and `state.device_N_{vt|n_vt}`. NR reads those
///   drifted values directly inside the junction-current evaluation.
///   `has_is_scaling = true` and `vt_field_name` / `vt_const_name` carry
///   the correct identifier so NaN-recovery can restore nominal values.
/// * **Vgk-bias-shift** (sharp-cutoff triode): thermal update touches only
///   `state.device_N_tj`. The Koren coefficients are untouched; instead the
///   NR call site adds `VBIAS_ALPHA · (Tj - TAMB)` to the Vgk input,
///   modeling contact-potential drift with cathode temperature.
///   `has_is_scaling = false` and the `vt_*` fields are unused placeholders.
#[derive(Serialize)]
pub(super) struct SelfHeatingDeviceData {
    /// Device index (0-based)
    pub(super) dev_num: usize,
    /// Start index in M-dimensional NR space (Vd for diodes, Vbe/Ic for BJTs,
    /// Vgk/Ip for triodes).
    pub(super) start_idx: usize,
    /// Whether this flavor drifts an ideality-scaled thermal voltage and an
    /// IS(T) saturation current on each sample. BJT/diode: true (NR reads
    /// both drifted fields). Triode: false (Koren coefficients are
    /// untouched; drift rides on a Vgk bias-shift injected at the call
    /// site instead).
    pub(super) has_is_scaling: bool,
    /// Lowercase state-field suffix for the ideality-scaled thermal voltage
    /// (`"vt"` for BJT, `"n_vt"` for diode). Empty for flavors where
    /// `has_is_scaling = false`; Tera templates guard accesses on
    /// `has_is_scaling` so the empty string is never rendered.
    pub(super) vt_field_name: String,
    /// Uppercase const-suffix for the same, e.g. `"VT"` or `"N_VT"`. Empty
    /// for flavors without IS-scaling.
    pub(super) vt_const_name: String,
}

/// Collect all devices that have self-heating enabled (BJTs, diodes, and
/// sharp-cutoff triodes).
pub(super) fn self_heating_device_data(ir: &CircuitIR) -> Vec<SelfHeatingDeviceData> {
    ir.device_slots
        .iter()
        .enumerate()
        .filter_map(|(dev_num, slot)| match &slot.params {
            DeviceParams::Bjt(bp) if bp.has_self_heating() => Some(SelfHeatingDeviceData {
                dev_num,
                start_idx: slot.start_idx,
                has_is_scaling: true,
                vt_field_name: "vt".to_string(),
                vt_const_name: "VT".to_string(),
            }),
            DeviceParams::Diode(dp) if dp.has_self_heating() => Some(SelfHeatingDeviceData {
                dev_num,
                start_idx: slot.start_idx,
                has_is_scaling: true,
                vt_field_name: "n_vt".to_string(),
                vt_const_name: "N_VT".to_string(),
            }),
            DeviceParams::Tube(tp) if tp.has_self_heating() => Some(SelfHeatingDeviceData {
                dev_num,
                start_idx: slot.start_idx,
                has_is_scaling: false,
                vt_field_name: String::new(),
                vt_const_name: String::new(),
            }),
            _ => None,
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
                // LDR carries no runtime-adjustable scalar CircuitState fields —
                // its live state is the opaque `device_{n}_state` block; RMIN/…/
                // TAU_R stay compile-time consts read directly by update().
                DeviceParams::Ldr(_) => ("Ldr".to_string(), vec![]),
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
pub(super) fn format_matrix_rows(
    rows: usize,
    cols: usize,
    get: impl Fn(usize, usize) -> f64,
) -> Vec<String> {
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

/// Emit the per-sample junction-temperature advance for a self-heating device.
///
/// **Single source of truth for the thermal state-advance inner block, shared
/// verbatim by BOTH the DK (`dk_emitter`) and nodal (`nodal_emitter`) emitters.**
/// The two emitters author independent scaffolding (Tera loop + hot
/// `CircuitState` on DK; procedural `push_str` + `Box<CircuitStateCold>` on
/// nodal), which makes every per-device state fragment a twin-divergence
/// hazard. This helper collapses the thermal advance to one function so the
/// emitted inner expression can never drift between paths — enforced by the
/// `thermal_tj_advance_dk_nodal_string_identity` codegen test. The block is
/// spliced in at 8-space indent after the caller has computed the dissipated
/// power `p` (W) into scope; it leaves the `[200, 500] K` clamped junction
/// temperature in `state.device_{dev_num}_tj`.
///
/// Exact solution of the first-order RC thermal ODE over one internal sample:
///   Tss = TAMB + P·RTH,   τ = RTH·CTH,   Tj += (Tss − Tj)·(1 − e^(−dt/τ))
///
/// This replaces the forward-Euler step `Tj += (P − (Tj−TAMB)/RTH)/CTH·dt`,
/// which overshoots (and can oscillate or blow through the clamp) whenever
/// dt > τ. The exponential form is unconditionally stable for any dt/τ and
/// converges to the same trajectory as Euler in the dt ≪ τ limit.
///
/// `dt` is the INTERNAL (oversampled) sample period. The rate is read from
/// `state.current_sample_rate` (host-rate semantics, kept live by
/// `set_sample_rate`) × OVERSAMPLING_FACTOR, NOT the baked
/// SAMPLE_RATE/INTERNAL_SAMPLE_RATE consts — a 96 kHz host running a
/// 44.1k-compiled circuit would otherwise integrate the thermal ODE ~2.2× too
/// fast per second of audio. Both emitters gate the `state.current_sample_rate`
/// field on via `needs_current_sr` (DK: `emit_state`; nodal:
/// `has_thermal_sr_consumer`) so the field always exists here.
///
/// When CTH ≤ 0 the thermal pole is instantaneous — emit the quasi-static
/// form `Tj = Tss` directly. Both forms keep the [200, 500] K runaway clamp.
pub(super) fn emit_thermal_tj_advance(dev_num: usize, cth: f64) -> String {
    if cth > 0.0 {
        let dt_expr = "1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64)";
        format!(
            "        let dt = {dt_expr};\n\
             \x20       let tss = DEVICE_{dev_num}_TAMB + p * DEVICE_{dev_num}_RTH;\n\
             \x20       let tau = DEVICE_{dev_num}_RTH * DEVICE_{dev_num}_CTH;\n\
             \x20       state.device_{dev_num}_tj += (tss - state.device_{dev_num}_tj) * (1.0 - (-dt / tau).exp());\n\
             \x20       state.device_{dev_num}_tj = state.device_{dev_num}_tj.clamp(200.0, 500.0);\n"
        )
    } else {
        format!(
            "        let tss = DEVICE_{dev_num}_TAMB + p * DEVICE_{dev_num}_RTH;\n\
             \x20       state.device_{dev_num}_tj = tss.clamp(200.0, 500.0);\n"
        )
    }
}

// ============================================================================
// Stateful-device interface (Phase 0c) — device-agnostic opaque state block
// ============================================================================
//
// A stateful device carries an opaque `[f64; N]` state block that is FROZEN
// during the NR solve and advanced AFTER it, on the converged driving-node
// voltage(s) — the same "Step 7e" ordering the thermal self-heating advance
// uses. Every string below is authored ONCE here and spliced verbatim by BOTH
// the DK (Tera-injected) and nodal (procedural) emitters, exactly like
// `emit_thermal_tj_advance`. That is what makes the two paths unable to drift.
//
// The framework is device-agnostic by construction: `state_size`,
// `driving_nodes.len()`, and `terminal_nodes.len()` are all read from the
// per-device `StatefulSpec` — nothing here assumes N == 1, two terminals, or a
// single driving node. A future 4-terminal / 2-threshold / 2-rail glow tube is
// a new leaf (a new `DeviceParams` arm supplying its own eval + `update` body),
// not a change to any of these functions.
//
// The device-SPECIFIC math — the NR current/Jacobian contribution that reads
// the frozen state, and the body of the per-device `update()` hook — is
// dispatched on `DeviceParams` in [`stateful_update_body`] and in the two
// device-evaluation emitters. In Phase 1a no `DeviceParams` variant is stateful
// yet, so [`stateful_device_data`] is empty for every real circuit and all of
// these emitters return `""` — a deck with no stateful device is byte-identical
// to before this machinery existed.

/// One resolved stateful device: its index + its opaque spec.
pub(super) struct StatefulDeviceData<'a> {
    pub(super) dev_num: usize,
    pub(super) spec: &'a crate::device_types::StatefulSpec,
}

/// Collect every device slot carrying an opaque stateful state block.
///
/// Empty for the overwhelming majority of circuits. All the `emit_stateful_*`
/// helpers short-circuit to `""` on an empty slice, so this being empty is the
/// byte-identity guarantee for non-stateful decks.
pub(super) fn stateful_device_data(ir: &CircuitIR) -> Vec<StatefulDeviceData<'_>> {
    ir.device_slots
        .iter()
        .enumerate()
        .filter_map(|(dev_num, slot)| {
            slot.stateful
                .as_ref()
                .map(|spec| StatefulDeviceData { dev_num, spec })
        })
        .collect()
}

/// Format a state-block seed as a Rust array literal, e.g. `[1.0e7, 7.5e1]`.
///
/// Length is `state_size`; if `state_seed` is short it is padded with `0.0`
/// (defensive — construction should always supply exactly `state_size`).
fn stateful_seed_literal(spec: &crate::device_types::StatefulSpec) -> String {
    let mut vals: Vec<String> = spec.state_seed.iter().map(|v| fmt_f64(*v)).collect();
    while vals.len() < spec.state_size {
        vals.push("0.0".to_string());
    }
    vals.truncate(spec.state_size);
    format!("[{}]", vals.join(", "))
}

/// Node-voltage expression for a 1-based node index (`0` = ground → `0.0`).
/// `base` is `"v"` (this sample's converged voltages) or `"state.v_prev"`
/// (the previous sample's).
fn node_volt_expr(base: &str, node: usize) -> String {
    if node == 0 {
        "0.0".to_string()
    } else {
        format!("{base}[{}]", node - 1)
    }
}

/// Struct-field declarations for every stateful device's opaque state block.
/// 4-space indent (struct body). Shared by DK and nodal.
pub(super) fn emit_stateful_state_fields(devs: &[StatefulDeviceData]) -> String {
    if devs.is_empty() {
        return String::new();
    }
    let mut s = String::from("    // --- Stateful-device opaque state blocks (Phase 0c) ---\n");
    for d in devs {
        let n = d.dev_num;
        let sz = d.spec.state_size;
        s.push_str(&format!(
            "    /// Device {n} opaque stateful state block. Device-agnostic\n\
             \x20   /// `[f64; {sz}]` — size is device-declared, never a bool. Frozen\n\
             \x20   /// during the NR solve, advanced after it on the driving node(s).\n\
             \x20   pub device_{n}_state: [f64; {sz}],\n"
        ));
    }
    s
}

/// `Default`-impl struct-literal entries for each state block (12-space indent).
pub(super) fn emit_stateful_default_fields(devs: &[StatefulDeviceData]) -> String {
    let mut s = String::new();
    for d in devs {
        let n = d.dev_num;
        s.push_str(&format!(
            "            device_{n}_state: {},\n",
            stateful_seed_literal(d.spec)
        ));
    }
    s
}

/// Assignment statements restoring each state block to its seed (8-space
/// indent). Used for BOTH `reset()` (`receiver = "self."`) and the NaN-recovery
/// path (`receiver = "state."`) — the only difference between those two sites.
pub(super) fn emit_stateful_state_restore(devs: &[StatefulDeviceData], receiver: &str) -> String {
    let mut s = String::new();
    for d in devs {
        let n = d.dev_num;
        s.push_str(&format!(
            "        {receiver}device_{n}_state = {};\n",
            stateful_seed_literal(d.spec)
        ));
    }
    s
}

/// `set_sample_rate` body fragment: re-bake any rate-dependent coefficient a
/// stateful device stashes in its state block. Dispatched on `DeviceParams`;
/// empty in Phase 1a (no stateful variant yet). The splice point exists so a
/// device that bakes coefficients at `set_sample_rate` time (rather than
/// recomputing them live from `current_sample_rate` inside `update`, as CdsLdr
/// does) needs no new wiring.
pub(super) fn emit_stateful_set_sample_rate_body(
    _ir: &CircuitIR,
    _devs: &[StatefulDeviceData],
) -> String {
    // Phase 1b will dispatch on `DeviceParams` here for any device with a
    // rate-baked coefficient. No stateful DeviceParams variant exists yet.
    String::new()
}

/// The after-solve `update()` CALL block for every stateful device.
///
/// Spliced BEFORE `state.v_prev = v` on all three process_sample paths (DK
/// template + nodal Schur + nodal full-LU), so `v` holds this sample's
/// converged node voltages and `state.v_prev` still holds the previous
/// sample's — the two the hook needs. The call passes the driving-node
/// voltages as fixed-size arrays, so the reserved `v_prev` argument (v1 devices
/// ignore it) is threaded WITHOUT any call-site change when sub-sample crossing
/// interpolation is added later.
pub(super) fn emit_stateful_update(devs: &[StatefulDeviceData]) -> String {
    let mut s = String::new();
    for d in devs {
        let n = d.dev_num;
        let dn = &d.spec.driving_nodes;
        let dcount = dn.len();
        let prev = dn
            .iter()
            .map(|&node| node_volt_expr("state.v_prev", node))
            .collect::<Vec<_>>()
            .join(", ");
        let conv = dn
            .iter()
            .map(|&node| node_volt_expr("v", node))
            .collect::<Vec<_>>()
            .join(", ");
        s.push_str(&format!(
            "    {{ // Device {n} stateful update — after solve, on converged driving-node voltage(s)\n\
             \x20       let dt = 1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64);\n\
             \x20       let v_prev_drive: [f64; {dcount}] = [{prev}];\n\
             \x20       let v_conv_drive: [f64; {dcount}] = [{conv}];\n\
             \x20       // The returned StatefulUpdate is RESERVED (sub-sample fractional-fire\n\
             \x20       // signal for a future within-sample-firing device). No v1 device sets\n\
             \x20       // it and the caller-side correction is not applied yet (Stage 3), so\n\
             \x20       // it is discarded here — reserving the slot without a signature rebuild.\n\
             \x20       let _ = stateful_update_dev{n}(&mut state.device_{n}_state, &v_prev_drive, &v_conv_drive, dt);\n\
             \x20   }}\n"
        ));
    }
    s
}

/// The `StatefulUpdate` return type, emitted ONCE (only when at least one
/// stateful device exists, so a non-stateful deck never sees it → byte-identical).
///
/// This is the reserved return slot of every `update()` hook. Phase-1 devices
/// return `StatefulUpdate::default()` — no sub-sample firing. The slot exists so
/// a future within-sample-firing device (e.g. a glow-discharge tube) can report
/// a fractional-fire signal — a BLEP-style crossing fraction `alpha ∈ [0,1)` and
/// a `fired` flag — that the caller applies to the in-flight sample `n+1` output
/// WITHOUT a signature rebuild. Rationale (device owner): a strike between
/// samples `n` and `n+1` has `alpha = (Vo − V_n)/(V_{n+1} − V_n)`, not computable
/// until after solve `n+1`; only a RETURN lets the caller correct `n+1` in-flight
/// (a global one-sample output latency is rejected — it taxes every plugin).
/// Stage 1 only RESERVES the slot; the caller-side correction lands in Stage 3.
fn stateful_update_return_type() -> &'static str {
    "/// Reserved return of a stateful device's `update()` hook (Phase 0c).\n\
     ///\n\
     /// v1 devices return `StatefulUpdate::default()` (no sub-sample firing).\n\
     /// Reserved so a within-sample-firing device (glow-discharge) can later\n\
     /// report a BLEP-style fractional-fire signal (`alpha` + `fired`) that the\n\
     /// caller applies to sample n+1 in-flight, with no signature rebuild. The\n\
     /// caller-side correction is NOT applied yet (Stage 3).\n\
     #[derive(Clone, Copy, Default)]\n\
     #[allow(dead_code)]\n\
     struct StatefulUpdate {\n\
     \x20   /// True when the device fired within this sample interval. Reserved.\n\
     \x20   fired: bool,\n\
     \x20   /// Sub-sample crossing fraction in [0, 1). Reserved.\n\
     \x20   alpha: f64,\n\
     }\n\n"
}

/// The per-device `update()` FUNCTIONS, with the fixed, device-agnostic
/// signature `update(&mut state_block, v_prev, v_converged, dt) -> StatefulUpdate`.
///
/// Two reservations keep the interface glow-survivable without a later rebuild:
/// * `v_prev` is RESERVED — v1 devices ignore it, but it is in the signature so
///   sub-sample crossing interpolation is addable without touching the call site.
/// * the `-> StatefulUpdate` RETURN is RESERVED — v1 devices return the default;
///   a future within-sample-firing device populates it (see
///   [`stateful_update_return_type`]).
///
/// The body is dispatched on `DeviceParams` via [`stateful_update_body`]. Phase
/// 1a supplies a scaffold no-op body; Phase 1b replaces it per device with the
/// real state advance. Every body ends by returning a `StatefulUpdate`.
pub(super) fn emit_stateful_update_fns(
    devs: &[StatefulDeviceData],
    slots: &[crate::device_types::DeviceSlot],
) -> String {
    if devs.is_empty() {
        return String::new();
    }
    // The return type is emitted once, ahead of the per-device hooks.
    let mut s = String::from(stateful_update_return_type());
    for d in devs {
        let n = d.dev_num;
        let sz = d.spec.state_size;
        let dcount = d.spec.driving_nodes.len();
        let slot = &slots[d.dev_num];
        let body = stateful_update_body(n, slot, d.spec);
        s.push_str(&format!(
            "/// Stateful update hook for device {n}. `v_prev` and the returned\n\
             /// `StatefulUpdate` are RESERVED (sub-sample interpolation / fractional-\n\
             /// fire, unused in v1) so glow drops in without a signature rebuild.\n\
             /// Advances the opaque `[f64; {sz}]` state on the driving-node voltage(s).\n\
             #[inline]\n\
             fn stateful_update_dev{n}(state: &mut [f64; {sz}], v_prev: &[f64; {dcount}], v_converged: &[f64; {dcount}], dt: f64) -> StatefulUpdate {{\n\
             {body}\
             }}\n\n"
        ));
    }
    s
}

/// Device-specific body of `stateful_update_dev{n}`. Dispatched on
/// `DeviceParams`. Every arm ends by returning a `StatefulUpdate`; v1 devices
/// return the no-op default (no sub-sample firing).
fn stateful_update_body(
    dev_num: usize,
    slot: &crate::device_types::DeviceSlot,
    _spec: &crate::device_types::StatefulSpec,
) -> String {
    use crate::device_types::DeviceParams;
    let d = dev_num;
    match &slot.params {
        // CdsLdr — mirrors `melange-devices/src/ldr.rs::update` (lines 82-97).
        // The control signal V(ctrl+)−V(ctrl-) is the normalized brightness in
        // [0,1] (clamped INSIDE here so an out-of-range control node degrades
        // gracefully, never errors); resistance chases the power-law target
        // with asymmetric attack/release. Coefficients are computed LIVE from
        // `dt` (the internal sample period) so a host-rate change is tracked
        // exactly — no baked-rate constant. State-mutation only; the returned
        // StatefulUpdate is the no-op default (CdsLdr has no sub-sample firing).
        DeviceParams::Ldr(_) => format!(
            "    let _ = v_prev; // reserved (sub-sample interpolation; unused in v1)\n\
             \x20   let cv = (v_converged[0] - v_converged[1]).clamp(0.0, 1.0);\n\
             \x20   let target_r = DEVICE_{d}_RMIN\n\
             \x20       + (DEVICE_{d}_RMAX - DEVICE_{d}_RMIN) * (1.0 - cv).powf(DEVICE_{d}_GAMMA);\n\
             \x20   let r = state[0];\n\
             \x20   // Asymmetric envelope: attack (target below current, getting\n\
             \x20   // brighter) is faster than release (getting darker).\n\
             \x20   let tau = if target_r < r {{ DEVICE_{d}_TAU_A }} else {{ DEVICE_{d}_TAU_R }};\n\
             \x20   let coef = (-dt / tau).exp();\n\
             \x20   state[0] = target_r + (r - target_r) * coef;\n\
             \x20   StatefulUpdate::default()\n"
        ),
        // No stateful math for other device kinds (none are stateful yet).
        _ => "    // No device-specific state advance for this DeviceParams.\n\
              \x20   let _ = (v_prev, v_converged, dt);\n\
              \x20   let _ = &mut *state;\n\
              \x20   StatefulUpdate::default()\n"
            .to_string(),
    }
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

/// Half-band filter coefficients. Must stay identical to
/// melange-primitives/src/oversampling.rs `coefficients` (twin-drift hazard).
///
/// Generated with the published hiir designer
/// `PolyphaseIir2Designer::compute_coefs_spec_order_tbw(n, tbw)`
/// (Laurent de Soras, 2005, WTFPL; <http://ldesoras.free.fr/prod.html#src_hiir>,
/// mirrored at <https://github.com/unevens/hiir>). `tbw` is normalized to the
/// filter's running rate; passband edge = (0.5 - tbw)/2 of that rate.
///
/// 7-section steep half-band: tbw = 0.04, -86.9 dB worst-case stopband,
/// passband to 0.23 * f_internal (20.3 kHz at a 44.1 kHz host for a 2x stage).
/// Used for 2x oversampling and the base-Nyquist (outer) stage of 4x.
const HB_STEEP_7SECTION: [f64; 7] = [
    0.05180201146164933,
    0.1879784418196106,
    0.3650536901969154,
    0.5423273752059077,
    0.6977781305199374,
    0.8282265929955739,
    0.9431266539721422,
];

/// 3-section wide-transition half-band for the inner (4x-rate) stage:
/// tbw = 0.27 per the hiir cascade rule TBW[stage] = (TBW[stage-1]+0.5)/2,
/// -95.1 dB over its design stopband. The inner stage only needs to protect
/// the spectrum the steep outer stage keeps.
const HB_WIDE_3SECTION: [f64; 3] = [0.06687030230470327, 0.2756202830232181, 0.6763597685457587];

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

#[cfg(test)]
mod stateful_interface_tests {
    //! Phase 0c Stage 1a — device-agnostic stateful-device interface.
    //!
    //! These tests exercise the shared emission scaffolding on a HYPOTHETICAL
    //! glow-discharge–shaped device WITHOUT any netlist device existing: a
    //! `state_size = 2` block (two thresholds), a 4-terminal node list, and TWO
    //! foreign driving nodes spanning two rails. Nothing in the emitters is
    //! specialized to CdsLdr (N=1, 2 terminals, 1 driving node) — the glow shape
    //! goes through the identical code, which is the concrete, in-code proof
    //! that the interface "survives GlowDischarge without a rebuild".
    //!
    //! The DK-vs-nodal *generated-code* byte-identity of these blocks is
    //! guaranteed by construction (both emitters splice the SAME functions
    //! below); the end-to-end netlist-reachable string-identity test lands in
    //! Stage 1c once CdsLdr makes a stateful device reachable from a `.cir`.
    use super::*;
    use crate::device_types::{DeviceSlot, DeviceType, DiodeParams, StatefulSpec};

    /// A glow-shaped spec: 2 state slots, 4 terminals, 2 driving nodes (one of
    /// which is ground, to exercise the `0 → 0.0` path).
    fn glow_spec() -> StatefulSpec {
        StatefulSpec {
            state_size: 2,
            state_seed: vec![1.0, 2.0],
            terminal_nodes: vec![1, 2, 3, 4],
            driving_nodes: vec![5, 0], // node 5 and ground
        }
    }

    #[test]
    fn empty_slice_emits_nothing() {
        // Byte-identity guarantee for non-stateful decks: every emitter is "".
        let devs: Vec<StatefulDeviceData> = Vec::new();
        assert_eq!(emit_stateful_state_fields(&devs), "");
        assert_eq!(emit_stateful_default_fields(&devs), "");
        assert_eq!(emit_stateful_state_restore(&devs, "self."), "");
        assert_eq!(emit_stateful_state_restore(&devs, "state."), "");
        assert_eq!(emit_stateful_update(&devs), "");
        assert_eq!(emit_stateful_update_fns(&devs, &[]), "");
    }

    #[test]
    fn glow_shape_state_block_is_opaque_n() {
        let spec = glow_spec();
        let devs = vec![StatefulDeviceData {
            dev_num: 0,
            spec: &spec,
        }];
        // Opaque [f64; N], N device-declared — never a bool.
        assert!(emit_stateful_state_fields(&devs).contains("pub device_0_state: [f64; 2],"));
        // Default / reset / NaN-recovery all restore the full 2-slot seed.
        assert!(emit_stateful_default_fields(&devs).contains("device_0_state: [1"));
        let reset = emit_stateful_state_restore(&devs, "self.");
        assert!(reset.starts_with("        self.device_0_state = ["));
        let nan = emit_stateful_state_restore(&devs, "state.");
        assert!(nan.starts_with("        state.device_0_state = ["));
    }

    #[test]
    fn glow_shape_update_call_threads_two_driving_nodes_and_reserved_v_prev() {
        let spec = glow_spec();
        let devs = vec![StatefulDeviceData {
            dev_num: 0,
            spec: &spec,
        }];
        let call = emit_stateful_update(&devs);
        // Two driving nodes → [f64; 2] arrays; node 5 → index 4, ground → 0.0.
        assert!(call.contains("let v_prev_drive: [f64; 2] = [state.v_prev[4], 0.0];"));
        assert!(call.contains("let v_conv_drive: [f64; 2] = [v[4], 0.0];"));
        // Reserved v_prev is threaded through the fixed signature, and the
        // reserved return is discarded at the call site (Stage 3 will apply it).
        assert!(call.contains(
            "let _ = stateful_update_dev0(&mut state.device_0_state, &v_prev_drive, &v_conv_drive, dt);"
        ));
        // dt is the internal (oversampled) period from the LIVE rate.
        assert!(call.contains("state.current_sample_rate * OVERSAMPLING_FACTOR as f64"));
    }

    #[test]
    fn glow_shape_update_fn_signature_is_device_agnostic() {
        let spec = glow_spec();
        let devs = vec![StatefulDeviceData {
            dev_num: 0,
            spec: &spec,
        }];
        // A throwaway slot to satisfy the body dispatch (params irrelevant to
        // the Phase-1a scaffold body).
        let slot = DeviceSlot {
            device_type: DeviceType::Diode,
            start_idx: 0,
            dimension: 1,
            params: crate::device_types::DeviceParams::Diode(DiodeParams {
                is: 1e-14,
                n_vt: 0.026,
                cjo: 0.0,
                rs: 0.0,
                bv: f64::INFINITY,
                ibv: 1e-3,
                rth: f64::INFINITY,
                cth: 1e-3,
                xti: 3.0,
                eg: 1.11,
                tamb: 300.15,
            }),
            has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
            stateful: Some(spec.clone()),
        };
        let fns = emit_stateful_update_fns(&devs, std::slice::from_ref(&slot));
        // The exact reserved-v_prev + reserved-return signature, generic over N/D.
        assert!(fns.contains(
            "fn stateful_update_dev0(state: &mut [f64; 2], v_prev: &[f64; 2], v_converged: &[f64; 2], dt: f64) -> StatefulUpdate"
        ));
        // The reserved return type is emitted exactly once, ahead of the hooks.
        assert!(fns.contains("struct StatefulUpdate {"));
        assert_eq!(fns.matches("struct StatefulUpdate {").count(), 1);
        // Scaffold body yields the no-op default return.
        assert!(fns.contains("StatefulUpdate::default()"));
    }

    #[test]
    fn reserved_return_type_dormant_for_non_stateful() {
        // No stateful device → no StatefulUpdate type, no hooks: byte-identical.
        assert_eq!(emit_stateful_update_fns(&[], &[]), "");
    }
}

/// Get oversampling configuration for a given factor.
pub(super) fn oversampling_info(factor: usize) -> OversamplingInfo {
    match factor {
        2 => {
            let num_sections = HB_STEEP_7SECTION.len();
            OversamplingInfo {
                num_sections,
                coeffs: HB_STEEP_7SECTION.to_vec(),
                state_size: num_sections * 2,
                state_size_outer: 0,
                coeffs_outer: Vec::new(),
                num_sections_outer: 0,
            }
        }
        4 => {
            // The STEEP filter guards the base-Nyquist boundary (outer,
            // 1x<->2x); the cheap wide-band filter runs at the inner
            // 2x<->4x boundary where only a wide transition band is needed.
            let inner = HB_WIDE_3SECTION.len();
            let outer = HB_STEEP_7SECTION.len();
            OversamplingInfo {
                num_sections: inner,
                coeffs: HB_WIDE_3SECTION.to_vec(),
                state_size: inner * 2,
                state_size_outer: outer * 2,
                coeffs_outer: HB_STEEP_7SECTION.to_vec(),
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
