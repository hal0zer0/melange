//! Noise source IR types and collection from the parsed MNA system.
//!
//! All circuit-noise simulation passes through this module:
//! thermal (Johnson-Nyquist), shot, junction flicker (1/f), resistor flicker
//! (Hooge), op-amp en/in, and pentode plate-partition. Source-of-truth for
//! calibration constants lives in `docs/aidocs/NOISE.md`. Collector functions
//! walk the parsed `MnaSystem` and emit zero entries when noise is disabled
//! per-device (default), so generated code is byte-identical when noise
//! params are absent.

use serde::{Deserialize, Serialize};

use crate::mna::MnaSystem;
use crate::parser::{Element, Netlist};

/// Johnson-Nyquist (thermal) noise source stamped at one fixed resistor.
///
/// Emitted as a Norton current source in the MNA RHS via a two-draw
/// Nyquist-anti-alias scheme (shipped 2026-04-24):
/// `i_n[n] = w[n] + w[n-1]` where `w[n] = (scale/2)·sqrt(1/R)·N(0,1)`
/// and `scale = sqrt(8·k_B·T·fs)`. The two-draw sum has PSD ∝
/// `4·cos²(πf/fs)` — zero at Nyquist, ≈ flat at audio. Single source of
/// truth: `RustEmitter::build_noise_emission` (used by both DK and
/// nodal codegen paths). See `docs/aidocs/NOISE.md` for the full
/// derivation, including the `8` (not `4` or `2`) calibration constant
/// and the BE-fallback replay path that re-injects the cached `i_n`
/// from `state.noise_thermal_last_i_n[k]` into `rhs_be`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalNoiseSource {
    /// Resistor component name (for debug / future per-source overrides).
    pub name: String,
    /// 1-indexed positive node (0 = ground).
    pub node_i: usize,
    /// 1-indexed negative node (0 = ground).
    pub node_j: usize,
    /// Resistance in ohms at the default operating point.
    ///
    /// For dynamic sources (`pot_slot` or `switch_slot` is `Some(_)`) this
    /// is the codegen-time default; the runtime value lives in
    /// `state.noise_thermal_sqrt_inv_r` and is refreshed by the matching
    /// `set_pot_N` / `set_runtime_R_<field>` / `set_switch_N` setter.
    pub resistance: f64,
    /// `Some(i)` when this R is a `.pot` / `.runtime R` / `.wiper` member
    /// backed by `mna.pots[i]`. The emitter injects a coefficient update
    /// into the corresponding pot / runtime-R setter so the per-sample
    /// `sqrt(1/R)` tracks the live resistance. `None` for fixed resistors
    /// and switch-R components.
    #[serde(default)]
    pub pot_slot: Option<usize>,
    /// `Some((switch_idx, comp_idx))` when this R is an R-type component
    /// under a `.switch` directive. The emitter injects a coefficient
    /// update into the corresponding `set_switch_N(position)` setter so
    /// the per-sample `sqrt(1/R)` tracks the discrete R value selected by
    /// the current position. `None` for fixed and pot-backed resistors.
    #[serde(default)]
    pub switch_slot: Option<(usize, usize)>,
}

/// Shot (junction) noise source stamped at one forward-biased PN junction.
///
/// Emitted as a Norton current source in the MNA RHS: for sample rate `fs`
/// and instantaneous bias current `|I(t)|` (read from `state.i_nl_prev`),
/// the per-sample current is `sqrt(4·q·|I|·fs) · N(0,1)` injected at
/// `node_i` and extracted at `node_j`. The `4·q·fs` matches thermal's
/// trap-MNA calibration (2× the textbook one-sided `2·q·|I|` PSD). See
/// `docs/aidocs/NOISE.md` "Constant derivation" for why.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShotNoiseSource {
    /// Device name (for debug / future per-device overrides).
    pub name: String,
    /// Index into `state.i_nl_prev` providing the instantaneous bias current.
    pub slot_idx: usize,
    /// 1-indexed positive injection node (0 = ground).
    pub node_i: usize,
    /// 1-indexed negative injection node (0 = ground).
    pub node_j: usize,
}

/// Flicker (1/f) noise source stamped at one current-carrying junction.
///
/// Emitted as a Norton current source whose amplitude is shaped by a Paul
/// Kellett 7-pole pink filter (≈ -3 dB/oct slope, ±0.5 dB over 10 Hz-20 kHz).
/// For sample rate `fs`, instantaneous bias current `|I(t)|` (read from
/// `state.i_nl_prev`), device-specific `KF` and `AF` model params, the
/// per-sample injected current is
/// `kellett(sqrt(4·KF·fs) · |I|^(AF/2) · N(0,1))`.
/// Same `4·…·fs` 2× trap-MNA compensation as thermal and shot; the Kellett
/// cascade has ~unity RMS power gain so the white-input PSD shapes into
/// 1/f at the output. See `docs/aidocs/NOISE.md`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlickerNoiseSource {
    /// Device name (and port suffix where relevant, e.g. "Q1.Ic").
    pub name: String,
    /// Index into `state.i_nl_prev` providing the instantaneous bias current.
    pub slot_idx: usize,
    /// 1-indexed positive injection node (0 = ground).
    pub node_i: usize,
    /// 1-indexed negative injection node (0 = ground).
    pub node_j: usize,
    /// Flicker coefficient `KF` from the device's `.model` (0 → source is
    /// filtered out before this struct is built; every emitted source has
    /// `kf > 0`).
    pub kf: f64,
    /// Flicker exponent `AF` from the device's `.model`. ngspice default 1.0.
    pub af: f64,
}

/// Resistor flicker (1/f) noise source — Hooge bias-squared form (Phase 3.5).
///
/// Emitted as a Norton current source whose amplitude is shaped by the same
/// Paul Kellett 7-pole pink filter used by junction flicker. Per-sample
/// injected current at sample rate `fs` is
/// `kellett(sqrt(KF·fs) · |I_R(t)|^(AF/2) · N(0,1))`,
/// where `I_R(t) = (V_+ − V_−) / R` is the **live** current through this
/// resistor at the previous sample (read from `state.v_prev`). A resistor
/// with no current carries only thermal — there is no constant pink floor.
/// AF defaults to `2.0` at codegen time (Hooge's exponent for resistors;
/// `Element::Resistor.af` is `Option<f64>` so an unspecified AF is filled
/// in here, not at the parser).
///
/// Per-source baked constants `NOISE_R_FLICKER_SQRT_KF[k] = sqrt(KF)` and
/// `NOISE_R_FLICKER_HALF_AF[k] = AF / 2`. Live `1/R` lives in
/// `state.noise_r_flicker_inv_r[k]` so `.pot` / `.wiper` / `.switch` /
/// `.runtime R` setters can refresh it without re-emitting code. The
/// scalar `state.noise_r_flicker_sqrt_fs` tracks `sqrt(fs)` and is
/// refreshed in `set_sample_rate`. See `docs/aidocs/NOISE.md` Phase 3.5.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResistorFlickerNoiseSource {
    /// Resistor component name (debug + future per-source overrides).
    pub name: String,
    /// 1-indexed positive node (0 = ground).
    pub node_i: usize,
    /// 1-indexed negative node (0 = ground).
    pub node_j: usize,
    /// Resistance in ohms at the default operating point. For dynamic
    /// sources (`pot_slot` / `switch_slot` Some) this is the codegen-time
    /// default; the live `1/R` is in `state.noise_r_flicker_inv_r[k]`.
    pub resistance: f64,
    /// Hooge KF coefficient (validated `> 0` finite by the parser; this
    /// struct is only built for opted-in resistors).
    pub kf: f64,
    /// Hooge AF exponent. Default 2.0 at codegen time when the parser
    /// captured `af = None`. Per-element overrides allowed.
    pub af: f64,
    /// `Some(i)` when this R is a `.pot` / `.wiper` / `.runtime R` member
    /// backed by `mna.pots[i]`. The emitter hooks the matching setter to
    /// refresh `state.noise_r_flicker_inv_r[k]`.
    #[serde(default)]
    pub pot_slot: Option<usize>,
    /// `Some((switch_idx, comp_idx))` when this R is an R-type switch
    /// component. Same setter-refresh contract as `pot_slot`.
    #[serde(default)]
    pub switch_slot: Option<(usize, usize)>,
}

/// Op-amp input-referred noise source (Phase 4).
///
/// Datasheet `en` (V/√Hz) and `in` (A/√Hz) input-referred noise — three
/// independent Norton current streams per op-amp:
/// - **en stream** at `node_plus`, amplitude `en · G_diag(in+) · sqrt(fs)`.
///   Equivalent to "voltage source `en` in series with non-inverting input",
///   Norton-transformed via the existing diagonal admittance at `n_plus_idx`
///   so no netlist resistor is inserted.
/// - **in+ stream** at `node_plus`, amplitude `in_amps · sqrt(fs)`.
/// - **in- stream** at `node_minus`, amplitude `in_amps · sqrt(fs)`.
/// All three use the two-draw Nyquist anti-alias and 2× trap-MNA compensation
/// (per-sample `sqrt(4·…·fs)` form for in; en uses `sqrt(2·en²·fs)` because
/// the source is voltage and the Norton transform absorbs the conductance).
///
/// `g_diag_plus_default` is the static `G[in+, in+]` at codegen time. v1
/// ships with the default baked into a runtime state field but **not yet
/// refreshed on dynamic-R commits** — the response-letter promise of
/// per-setter refresh is reserved for v1.5. Most planned op-amp circuits
/// (NE5534, 4558 single-stage) have fixed input networks, so static G_diag
/// matches measurement; circuits with level pots in series with `in+` need
/// the v1.5 refresh to track the knob.
///
/// `en_fc` and `in_fc` (1/f corner frequencies) are parsed and stored but
/// **not yet wired to codegen** in v1 — Kellett-pink blend is the planned
/// extension. Per the response-letter agreement with Noyce, white-band
/// en/in magnitude dominates the audible op-amp signature; the 1/f shaping
/// is a v1.5 enhancement.
///
/// `in+` and `in-` streams are uncorrelated (independent xoshiro states
/// salted via `NOISE_OPAMP_IN_SALT`). The literature treats `en`/`in`
/// correlation as zero 90 % of the time and the result lands within ~1 dB
/// of measurement; uncorrelated is the v1 target.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpampNoiseSource {
    /// Op-amp name (debug + future per-device overrides).
    pub name: String,
    /// 1-indexed non-inverting input node. `0` = grounded (en stamp skipped
    /// for that source; in+ stamp also skipped).
    pub node_plus: usize,
    /// 1-indexed inverting input node. `0` = grounded (in- stamp skipped).
    pub node_minus: usize,
    /// Input-referred voltage noise spectral density [V/√Hz].
    pub en: f64,
    /// Input-referred current noise spectral density [A/√Hz]. Stamps
    /// independently at `node_plus` and `node_minus` (two streams).
    pub in_amps: f64,
    /// 1/f corner for en [Hz]. Parsed/stored; not yet wired to codegen
    /// (v1 is white-only).
    pub en_fc: f64,
    /// 1/f corner for in [Hz]. Same v1 semantics as [`en_fc`].
    pub in_fc: f64,
    /// Static `G[in+, in+]` at codegen time — diagonal admittance seen at
    /// the non-inverting input. The `en` stamp uses this as the
    /// voltage-to-Norton-current conversion factor: `i_n = en · G_diag · g`.
    /// v1 bakes this as a const default into `state.noise_opamp_en_g_diag[k]`;
    /// runtime refresh on dynamic-R commits is deferred to v1.5.
    pub g_diag_plus_default: f64,
}

/// Pentode partition noise source (Phase 5).
///
/// Pentode plate current is set by grid voltage, but the *partition* between
/// plate and screen is statistical — the screen-grid divert thins the cathode
/// beam reaching the plate. Per Schottky 1918 the resulting plate-current
/// noise PSD is **lower** than bare shot:
/// ```text
/// S_i(plate) = 2·q · I_p · I_s / (I_p + I_s)       [A²/Hz]
/// ```
/// At matched `I_p` a pentode is *quieter* than a triode would be — the
/// suppression factor `I_s / (I_p + I_s)` lands in 0.05–0.30 for typical
/// pentode bias. This **replaces** the bare-shot stamp at the plate; the
/// `collect_shot_noise_sources` collector filters pentode plate ports out
/// when the partition collector picks them up.
///
/// Per-sample injected current at sample rate `fs` is
/// `sqrt(4·q · I_p·I_s/(I_p+I_s) · fs) · PARTITION_F · N(0,1)`,
/// stamped with the same two-draw Nyquist anti-alias as thermal/shot and
/// the same 2× trap-MNA compensation factor. `I_p` and `I_s` are read
/// one-sample-lagged from `state.i_nl_prev[ip_slot]` / `[is_slot]`.
///
/// `PARTITION_F` is a process-variation knob from `.model TUBE(PARTITION_F=…)`,
/// default 1.0 (textbook). The dominant control over pentode noise floor is
/// the bias network in the `.cir` (sets `I_s/I_p`); this is the secondary
/// matched-tube character knob.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PentodePartitionSource {
    /// Device name (debug + future per-device overrides).
    pub name: String,
    /// Index into `state.i_nl_prev` for plate current `I_p`.
    pub ip_slot_idx: usize,
    /// Index into `state.i_nl_prev` for screen current `I_s` (Ig2).
    pub is_slot_idx: usize,
    /// 1-indexed positive injection node (plate). 0 = ground.
    pub node_i: usize,
    /// 1-indexed negative injection node (cathode). 0 = ground.
    pub node_j: usize,
    /// Partition-multiplier `PARTITION_F`. Defaults to 1.0 (textbook
    /// Schottky); values < 1.0 model low-noise-selected pentode batches.
    pub partition_f: f64,
}

/// Noise configuration baked into the generated code.
///
/// Built from [`crate::codegen::NoiseMode`] + a scan of `netlist.elements`.
/// Emitters gate all noise-related emission on `mode != NoiseMode::Off` and
/// iterate the per-phase source vectors.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct NoiseIR {
    /// Compile-time mode. `NoiseMode::Off` → zero emission, zero runtime cost.
    #[serde(default)]
    pub mode: crate::codegen::NoiseMode,
    /// Master seed for deterministic noise. `0` → system entropy at `default()`.
    #[serde(default)]
    pub master_seed: u64,
    /// Johnson-Nyquist noise sources (Phase 1).
    #[serde(default)]
    pub thermal_sources: Vec<ThermalNoiseSource>,
    /// Shot (junction) noise sources (Phase 2). Populated only when
    /// `mode.includes_shot()`.
    #[serde(default)]
    pub shot_sources: Vec<ShotNoiseSource>,
    /// Flicker (1/f) noise sources (Phase 3). Populated only when
    /// `mode.includes_full()` AND the device's `.model` supplies a positive
    /// `KF`. Zero-`KF` devices produce no entry — zero codegen overhead.
    #[serde(default)]
    pub flicker_sources: Vec<FlickerNoiseSource>,
    /// Resistor flicker (1/f) noise sources — Hooge bias-squared (Phase 3.5).
    /// Populated only when `mode.includes_full()` AND the resistor line
    /// supplies positive `KF`. Default-zero like junction flicker; netlists
    /// without per-resistor `KF` produce byte-identical code to a
    /// pre-Phase-3.5 build under `--noise full`.
    #[serde(default)]
    pub resistor_flicker_sources: Vec<ResistorFlickerNoiseSource>,
    /// Pentode partition noise sources (Phase 5). One per pentode device.
    /// Populated only when `mode.includes_full()`. **Replaces** the bare
    /// plate-shot stamp for pentodes — see `collect_shot_noise_sources` for
    /// the corresponding filter.
    #[serde(default)]
    pub partition_sources: Vec<PentodePartitionSource>,
    /// Op-amp input-referred noise sources (Phase 4). One per op-amp whose
    /// `.model OA(EN=… IN=…)` supplies a positive `en` or `in` parameter.
    /// Populated only when `mode.includes_full()`. Zero-en zero-in op-amps
    /// are filtered, so circuits without these params produce byte-identical
    /// code to pre-Phase-4 builds under `--noise full`.
    #[serde(default)]
    pub opamp_noise_sources: Vec<OpampNoiseSource>,
}

/// Collect Johnson-Nyquist thermal noise sources from the netlist.
///
/// Includes every `Element::Resistor` that maps to a circuit node, with
/// three source kinds (all indexed uniformly at
/// `state.noise_thermal_sqrt_inv_r[k]`):
///
/// - **Static** (both `pot_slot` and `switch_slot` are `None`) —
///   fixed-value resistor. The per-sample coefficient `sqrt(1/R)` is baked
///   at codegen time.
/// - **Pot-backed** (`pot_slot = Some(i)`) — `.pot` / `.wiper` /
///   `.runtime R` member backed by `mna.pots[i]`. The coefficient is
///   state-backed and refreshed inside `set_pot_N` /
///   `set_runtime_R_<field>`.
/// - **Switch-backed** (`switch_slot = Some((sw, comp))`) — R-type
///   component under a `.switch` directive. The coefficient is refreshed
///   inside `set_switch_N(position)` using the position-indexed value
///   array emitted by each solver's constants block.
///
/// Node indices are MNA 1-indexed (0 = ground) via `mna.node_map`. Resistors
/// whose terminals fail to resolve (would collapse to ground–ground) are
/// filtered out — they inject nothing useful.
pub fn collect_thermal_noise_sources(
    netlist: &Netlist,
    mna: &MnaSystem,
) -> Vec<ThermalNoiseSource> {
    use std::collections::HashMap;

    // Upper-cased name → pot index in mna.pots. Covers .pot, .runtime R,
    // and wiper-derived pot entries (wipers decompose into cw/ccw pots
    // that both live in mna.pots).
    let mut pot_slot: HashMap<String, usize> = HashMap::new();
    for (i, p) in mna.pots.iter().enumerate() {
        pot_slot.insert(p.name.to_ascii_uppercase(), i);
    }

    // Upper-cased name → (switch_idx, comp_idx) for R-type switch
    // components. C and L components don't generate thermal noise and are
    // skipped here — they also aren't Element::Resistor at the netlist
    // level, so the loop below never sees them.
    let mut switch_slot_map: HashMap<String, (usize, usize)> = HashMap::new();
    for (sw_idx, sw) in mna.switches.iter().enumerate() {
        for (ci, c) in sw.components.iter().enumerate() {
            if c.component_type == 'R' {
                switch_slot_map.insert(c.name.to_ascii_uppercase(), (sw_idx, ci));
            }
        }
    }

    let mut sources = Vec::new();
    for el in &netlist.elements {
        if let Element::Resistor {
            name,
            n_plus,
            n_minus,
            value,
            ..
        } = el
        {
            let upper = name.to_ascii_uppercase();
            if !value.is_finite() || *value <= 0.0 {
                continue;
            }
            let node_i = mna.node_map.get(n_plus).copied().unwrap_or(0);
            let node_j = mna.node_map.get(n_minus).copied().unwrap_or(0);
            if node_i == 0 && node_j == 0 {
                continue;
            }
            let pot_idx = pot_slot.get(&upper).copied();
            let sw_idx = switch_slot_map.get(&upper).copied();
            // A single R cannot be under both `.pot` and `.switch` in the
            // parser (those directives validate mutually exclusive
            // membership), so at most one of pot_idx / sw_idx is Some.
            // Resolve initial resistance from the backing structure:
            //   - pot: use `1/g_nominal` (MNA-canonical value)
            //   - switch: use position-0 value (matches Default::default)
            //   - neither: use the netlist literal value
            let resistance = if let Some(i) = pot_idx {
                let g = mna.pots[i].g_nominal;
                if g > 0.0 && g.is_finite() {
                    1.0 / g
                } else {
                    *value
                }
            } else if let Some((sw, comp)) = sw_idx {
                let positions = &mna.switches[sw].positions;
                let r0 = positions.first().and_then(|row| row.get(comp)).copied();
                match r0 {
                    Some(v) if v.is_finite() && v > 0.0 => v,
                    _ => *value,
                }
            } else {
                *value
            };
            sources.push(ThermalNoiseSource {
                name: name.clone(),
                node_i,
                node_j,
                resistance,
                pot_slot: pot_idx,
                switch_slot: sw_idx,
            });
        }
    }
    sources
}

/// Collect shot-noise sources from the nonlinear device list.
///
/// Emits one `ShotNoiseSource` per current-carrying junction, with the
/// `slot_idx` pointing at the relevant entry of `state.i_nl_prev`:
///
/// - Diode: 1 source at (anode, cathode), slot = start_idx (Id).
/// - BJT (2D): 2 sources — (collector, emitter) at slot=start_idx (Ic),
///   (base, emitter) at slot=start_idx+1 (Ib).
/// - BJT forward-active (1D): 1 source at (collector, emitter),
///   slot=start_idx. The Ib shot is folded in via the BF stamping.
/// - JFET / MOSFET: 1 source at (drain, source), slot=start_idx (Id).
///   Gate shot is ≈ 0 for MOS and deferred for JFET reverse-bias leakage.
/// - Tube (triode or pentode): 1 source at (plate, cathode),
///   slot=start_idx (Ip). Pentode partition noise is Step 7 work.
/// - VCA: skipped — shot at a control port is not physically meaningful.
///
/// The returned sources stamp Norton currents at the **external** device
/// nodes even when the nodal path has internal-node expansion active.
/// In that case the parasitic series R shapes the injection from outside,
/// not from inside the junction — a small-magnitude approximation that is
/// inaudible at audio rates and avoids threading internal-node indices
/// through the collector. Revisit if BJT RB/RE become tonally relevant
/// for shot content.
pub fn collect_shot_noise_sources(mna: &MnaSystem) -> Vec<ShotNoiseSource> {
    let mut sources = Vec::new();
    for dev in &mna.nonlinear_devices {
        // Phase 5: pentode plate noise is handled by `collect_pentode_partition_sources`
        // — it's partition-suppressed, not bare Schottky shot. Skip the Ip
        // (slot_offset=0) port for pentodes (4 or 5 node_indices); pentodes
        // contribute nothing else to shot today (Ig2/Ig1 are unused), so the
        // whole device drops out of the shot loop. Triodes (3 nodes) keep
        // their Ip port — full shot.
        let is_pentode = matches!(dev.device_type, crate::mna::NonlinearDeviceType::Tube)
            && matches!(dev.node_indices.len(), 4 | 5);
        for port in dev.junction_current_ports() {
            if is_pentode && port.slot_offset == 0 {
                continue;
            }
            let name = if port.label.is_empty() {
                dev.name.clone()
            } else {
                format!("{}.{}", dev.name, port.label)
            };
            sources.push(ShotNoiseSource {
                name,
                slot_idx: dev.start_idx + port.slot_offset,
                node_i: port.node_pos,
                node_j: port.node_neg,
            });
        }
    }
    sources
}

/// Collect pentode partition noise sources (Phase 5).
///
/// Walks `mna.nonlinear_devices` for pentodes (4 or 5 `node_indices`,
/// `device_type == Tube`) and emits one [`PentodePartitionSource`] per device.
/// The plate slot is `start_idx + 0`, the screen slot is `start_idx + 1`
/// (same layout for both `SharpPentode` 3D and `SharpPentodeGridOff` 2D — in
/// the grid-off case `Ig1` is dropped but `Ig2` stays at slot 1).
///
/// `partition_f` is read from the model's `PARTITION_F` parameter via the
/// netlist; defaults to 1.0 (textbook Schottky 1918) when omitted.
///
/// Pentodes whose `node_indices` don't expose plate (`n[0]`) and cathode
/// (`n[2]`) are filtered defensively — matches the [`collect_shot_noise_sources`]
/// convention for malformed nodes.
pub fn collect_pentode_partition_sources(
    netlist: &Netlist,
    mna: &MnaSystem,
) -> Vec<PentodePartitionSource> {
    use std::collections::HashMap;

    let mut model_for: HashMap<String, String> = HashMap::new();
    for el in &netlist.elements {
        if let Element::Pentode { name, model, .. } = el {
            model_for.insert(name.to_ascii_uppercase(), model.clone());
        }
    }

    // PARTITION_F resolver: explicit `.model TUBE(PARTITION_F=…)` wins; default
    // 1.0 (textbook). Validated `> 0 && finite` by `TubeParams::validate()`,
    // which the pentode resolver in `resolve_pentode_params` already runs —
    // so by the time we get here every PARTITION_F is safe to use.
    let partition_f_for = |dev_name: &str| -> f64 {
        let Some(model) = model_for.get(&dev_name.to_ascii_uppercase()) else {
            return 1.0;
        };
        let Some(m) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
        else {
            return 1.0;
        };
        for (k, v) in &m.params {
            if k.eq_ignore_ascii_case("PARTITION_F") && v.is_finite() && *v > 0.0 {
                return *v;
            }
        }
        1.0
    };

    let mut sources = Vec::new();
    for dev in &mna.nonlinear_devices {
        if !matches!(dev.device_type, crate::mna::NonlinearDeviceType::Tube) {
            continue;
        }
        let n = &dev.node_indices;
        if !matches!(n.len(), 4 | 5) {
            continue;
        }
        // Pentode node order: [plate, grid, cathode, screen, [supp]]
        let node_plate = n[0];
        let node_cathode = n[2];
        if node_plate == 0 && node_cathode == 0 {
            continue;
        }
        sources.push(PentodePartitionSource {
            name: dev.name.clone(),
            ip_slot_idx: dev.start_idx,
            is_slot_idx: dev.start_idx + 1,
            node_i: node_plate,
            node_j: node_cathode,
            partition_f: partition_f_for(&dev.name),
        });
    }
    sources
}

/// Collect op-amp input-referred noise sources (Phase 4).
///
/// Walks `mna.opamps`, filters on `en > 0.0 || in_amps > 0.0`, and for each
/// opted-in op-amp captures the static `G[in+, in+]` diagonal — the
/// voltage-to-Norton-current conversion factor for `en` (equivalent
/// thermal-R Norton transform, but applied to whatever bias network the
/// user already has at the input). Op-amps with grounded `n_plus_idx == 0`
/// store `g_diag_plus_default = 0.0`; their en stamp becomes a no-op.
///
/// The static G_diag is baked into a runtime state field; v1.5 will refresh
/// it on dynamic-R commits (`.pot` / `.switch` setters touching the in+
/// node). For v1 the field exists but is only restored to default on
/// `reset()` / `set_seed()`.
pub fn collect_opamp_noise_sources(mna: &MnaSystem) -> Vec<OpampNoiseSource> {
    let mut sources = Vec::new();
    for oa in &mna.opamps {
        let en = oa.en;
        let in_amps = oa.in_amps;
        if !((en > 0.0 && en.is_finite()) || (in_amps > 0.0 && in_amps.is_finite())) {
            continue;
        }
        let np = oa.n_plus_idx;
        let nm = oa.n_minus_idx;
        // Static G diagonal at in+. For grounded in+ (np == 0) there is no
        // diagonal — en stamp will skip because we test `np > 0` per stamp.
        let g_diag = if np > 0 && np <= mna.g.len() {
            mna.g[np - 1][np - 1]
        } else {
            0.0
        };
        sources.push(OpampNoiseSource {
            name: oa.name.clone(),
            node_plus: np,
            node_minus: nm,
            en: if en.is_finite() && en > 0.0 { en } else { 0.0 },
            in_amps: if in_amps.is_finite() && in_amps > 0.0 {
                in_amps
            } else {
                0.0
            },
            en_fc: if oa.en_fc.is_finite() && oa.en_fc > 0.0 {
                oa.en_fc
            } else {
                0.0
            },
            in_fc: if oa.in_fc.is_finite() && oa.in_fc > 0.0 {
                oa.in_fc
            } else {
                0.0
            },
            g_diag_plus_default: if g_diag.is_finite() { g_diag } else { 0.0 },
        });
    }
    sources
}

/// Collect flicker (1/f) noise sources from the nonlinear device list.
///
/// Per-device port layout mirrors `collect_shot_noise_sources` — Diode 1,
/// BJT 2 (Ic + Ib), BjtForwardActive 1 (Ic only), JFET/MOSFET 1 (Id), Tube 1
/// (Ip), VCA skipped — but each device is filtered on `KF > 0` so circuits
/// with zero-`KF` (the default) models produce no flicker entries and incur
/// no per-sample cost. `AF` defaults to 1.0 (ngspice convention).
///
/// Model lookup walks `netlist.elements` once to map device name → model
/// name, then `netlist.models` for the KF/AF pair. The cost is O(N_dev +
/// N_elem) per codegen invocation; negligible compared to matrix assembly.
pub fn collect_flicker_noise_sources(
    netlist: &Netlist,
    mna: &MnaSystem,
) -> Vec<FlickerNoiseSource> {
    use std::collections::HashMap;

    // Build device-name → model-name map once.
    let mut model_for: HashMap<String, String> = HashMap::new();
    for el in &netlist.elements {
        match el {
            Element::Diode { name, model, .. }
            | Element::Bjt { name, model, .. }
            | Element::Jfet { name, model, .. }
            | Element::Mosfet { name, model, .. }
            | Element::Triode { name, model, .. }
            | Element::Pentode { name, model, .. } => {
                model_for.insert(name.to_ascii_uppercase(), model.clone());
            }
            _ => {}
        }
    }

    // Resolve (KF, AF) for a given device by its name, returning `None` when
    // the device is not modeled (shouldn't happen — defensive) or KF <= 0.
    let kf_af = |dev_name: &str| -> Option<(f64, f64)> {
        let model = model_for.get(&dev_name.to_ascii_uppercase())?;
        let m = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))?;
        let mut kf = 0.0_f64;
        let mut af = 1.0_f64;
        for (k, v) in &m.params {
            match k.to_ascii_uppercase().as_str() {
                "KF" => kf = *v,
                "AF" => af = *v,
                _ => {}
            }
        }
        if kf > 0.0 && kf.is_finite() && af > 0.0 && af.is_finite() {
            Some((kf, af))
        } else {
            None
        }
    };

    let mut sources = Vec::new();
    for dev in &mna.nonlinear_devices {
        let Some((kf, af)) = kf_af(&dev.name) else {
            continue;
        };
        for port in dev.junction_current_ports() {
            let name = if port.label.is_empty() {
                dev.name.clone()
            } else {
                format!("{}.{}", dev.name, port.label)
            };
            sources.push(FlickerNoiseSource {
                name,
                slot_idx: dev.start_idx + port.slot_offset,
                node_i: port.node_pos,
                node_j: port.node_neg,
                kf,
                af,
            });
        }
    }
    sources
}

/// Collect resistor flicker (1/f) noise sources from the netlist.
///
/// Walks `Element::Resistor` entries, filtered on `kf == Some(v) if v > 0`,
/// using the same fixed / pot / switch enumeration as
/// [`collect_thermal_noise_sources`]:
///
/// - **Static** (both `pot_slot` and `switch_slot` are `None`) —
///   fixed-value resistor. Live `1/R` baked at codegen time into
///   `state.noise_r_flicker_inv_r[k]` at `Default::default()`.
/// - **Pot-backed** (`pot_slot = Some(i)`) — `.pot` / `.wiper` /
///   `.runtime R` member. The matching setter (`set_pot_N` /
///   `set_runtime_R_<field>`) refreshes `inv_r[k]`.
/// - **Switch-backed** (`switch_slot = Some((sw, comp))`) — R-type
///   component under a `.switch`. `set_switch_N(position)` refreshes
///   `inv_r[k]` from the position-indexed value array.
///
/// `AF` defaults to `2.0` here when the parser captured `af = None` (the
/// Hooge exponent for resistors). Resistors whose terminals collapse to
/// ground–ground are filtered (consistent with the thermal collector).
pub fn collect_resistor_flicker_noise_sources(
    netlist: &Netlist,
    mna: &MnaSystem,
) -> Vec<ResistorFlickerNoiseSource> {
    use std::collections::HashMap;

    let mut pot_slot: HashMap<String, usize> = HashMap::new();
    for (i, p) in mna.pots.iter().enumerate() {
        pot_slot.insert(p.name.to_ascii_uppercase(), i);
    }

    let mut switch_slot_map: HashMap<String, (usize, usize)> = HashMap::new();
    for (sw_idx, sw) in mna.switches.iter().enumerate() {
        for (ci, c) in sw.components.iter().enumerate() {
            if c.component_type == 'R' {
                switch_slot_map.insert(c.name.to_ascii_uppercase(), (sw_idx, ci));
            }
        }
    }

    let mut sources = Vec::new();
    for el in &netlist.elements {
        let Element::Resistor {
            name,
            n_plus,
            n_minus,
            value,
            kf,
            af,
        } = el
        else {
            continue;
        };
        // Opt-in gate. Parser already normalizes KF=0 / KF unset to `None`
        // and strips an AF-without-KF, so a `Some(v) if v > 0` here is
        // fully validated.
        let Some(kf) = *kf else {
            continue;
        };
        if !kf.is_finite() || kf <= 0.0 {
            continue;
        }
        if !value.is_finite() || *value <= 0.0 {
            continue;
        }
        let node_i = mna.node_map.get(n_plus).copied().unwrap_or(0);
        let node_j = mna.node_map.get(n_minus).copied().unwrap_or(0);
        if node_i == 0 && node_j == 0 {
            continue;
        }
        let upper = name.to_ascii_uppercase();
        let pot_idx = pot_slot.get(&upper).copied();
        let sw_idx = switch_slot_map.get(&upper).copied();
        // Resolve initial resistance the same way the thermal collector does
        // so the dynamic-R refresh path agrees on the codegen-time default.
        let resistance = if let Some(i) = pot_idx {
            let g = mna.pots[i].g_nominal;
            if g > 0.0 && g.is_finite() {
                1.0 / g
            } else {
                *value
            }
        } else if let Some((sw, comp)) = sw_idx {
            let positions = &mna.switches[sw].positions;
            let r0 = positions.first().and_then(|row| row.get(comp)).copied();
            match r0 {
                Some(v) if v.is_finite() && v > 0.0 => v,
                _ => *value,
            }
        } else {
            *value
        };
        // AF default applied here, not in the parser, so an `Element::Resistor`
        // round-trips through serde without the parser's defaulting being
        // baked into the netlist representation.
        let af_eff = af.unwrap_or(2.0);
        sources.push(ResistorFlickerNoiseSource {
            name: name.clone(),
            node_i,
            node_j,
            resistance,
            kf,
            af: af_eff,
            pot_slot: pot_idx,
            switch_slot: sw_idx,
        });
    }
    sources
}
