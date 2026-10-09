//! Circuit-noise emission fragments shared by the DK and nodal paths.

use super::helpers::fmt_f64;
use super::RustEmitter;
use crate::codegen::ir::{CircuitIR, DeviceParams};
use crate::codegen::NoiseMode;

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
    /// noise-specific transient state (the per-source `last_i_n`
    /// caches). Does NOT re-seed the RNG —
    /// determinism contract says `set_seed` is the only re-seed entry.
    pub nan_recovery_body: String,
    /// Body to append inside `set_sample_rate` — recomputes `thermal_scale`.
    pub set_sample_rate_body: String,
    /// `impl CircuitState` methods: set_noise_enabled, set_noise_gain, …
    pub methods: String,
    /// `true` when any code is emitted (for template `{% if noise_enabled %}`).
    pub enabled: bool,
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
                    gg: tp.gg,
                    xi: tp.xi,
                    cg: tp.cg,
                    lambda: tp.lambda,
                    mu_b: tp.mu_b,
                    svar: tp.svar,
                    ex_b: tp.ex_b,
                };
                // The tube's own gm, at its internal grid behind RGI.
                let vgk = triode.internal_grid_voltage(vgk, tp.rgi);
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
        // Calibration: every stamp is the PHYSICAL noise current at n+1, one
        // draw per source per sample, under both integrators. The charge form
        // enters each source once (at n+1), so `A − A_neg = G` on every row and
        // the kernel has the circuit's own LF gain. The whole-system trapezoidal
        // form entered b(n+1) + b(n); its two-draw stamp w[n] + w[n-1] (and the
        // x2 flicker amplitude) was that image of the same physical current, so
        // the transfer is unchanged. The Nyquist zero it gave comes from the
        // trapezoidal integrator itself on the charge-carrying rows. Validated
        // against the kTC theorem for both integrators in
        // tests/noise_psd_validation.rs::thermal_noise_be_primary_matches_trap_anchor.
        // See NOISE.md.
        // The Kellett pink filter helper is shared between junction flicker
        // (Phase 3) and resistor flicker (Phase 3.5). Emit when either is
        // present.
        let need_kellett = flicker_n > 0 || r_flicker_n > 0;
        // Flicker absolute calibration (2026-07-18): both flicker phases use
        // an fs/OS-INVARIANT white-input amplitude scale sqrt(0.5 · 1/K_pink)
        // (the physical current), where K_pink = kellett_pink_normalized_gain() ≈ 6.0e-3 is the
        // cascade's |H(ν)|² ≈ K_pink/ν gain constant, computed analytically
        // at codegen time. Full derivation in ir/noise.rs and NOISE.md
        // "Flicker calibration".
        let kellett_k = crate::codegen::ir::kellett_pink_normalized_gain();
        let flicker_scale = (0.5 / kellett_k).sqrt();
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
        // is the physical `sqrt(q·|I_prev|·fs)` (PSD 2·q·|I| on [0, fs/2]; see
        // `docs/aidocs/NOISE.md` "Constant derivation").
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
        // One draw per source per sample at the physical amplitude, like
        // thermal and shot.
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
        // All three are one physical draw per sample and inherit the
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
        state_fields.push_str(
            "    /// Precomputed sqrt(2·K_B·T·fs_internal): the physical Johnson current\n",
        );
        state_fields.push_str("    /// per sqrt(1/R), one draw per sample (see NOISE.md).\n");
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
                "    /// Flicker white-input scale `sqrt(0.5/K_pink)` — fs/OS-INVARIANT.\n",
            );
            state_fields.push_str("    /// Per-sample amplitude is\n");
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
            state_fields.push_str(
                "    /// `set_sample_rate`. Holds sqrt(0.5·fs) (one-sided PSD over [0, fs/2]),\n",
            );
            state_fields.push_str("    /// shared across en/in streams.\n");
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
        default_stmts.push_str(
            "        // One draw per sample at the physical amplitude sqrt(2*K_B*T*fs)\n\
             \x20       // (the kernel's LF gain is the circuit's own: A - A_neg = G).\n",
        );
        default_stmts.push_str(
            "        let noise_thermal_scale = (2.0 * K_B * T_ROOM_K * fs_internal).sqrt();\n",
        );
        default_stmts.push_str("        let noise_rng = seed_noise_rngs::<NOISE_THERMAL_N>(NOISE_MASTER_SEED_DEFAULT);\n");
        if need_q_scale {
            default_stmts.push_str(
                "        // Shared shot/partition amplitude: sqrt(Q_E·fs) — PSD 2·q·|I| on\n",
            );
            default_stmts.push_str(
                "        // [0, fs/2] gives the per-sample variance q·|I|·fs, one draw per\n",
            );
            default_stmts.push_str(
                "        // sample (partition uses I_p·I_s/(I_p+I_s) in place of |I|).\n",
            );
            default_stmts.push_str("        let noise_shot_scale = (Q_E * fs_internal).sqrt();\n");
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
                "0.5", kellett_k
            ));
            default_stmts.push_str(
                "        // K_pink is the Kellett cascade's normalized-frequency gain\n\
                 \x20       // constant (|H(ν)|² ≈ K_pink/ν), computed analytically at codegen\n\
                 \x20       // time (kellett_pink_normalized_gain in ir/noise.rs). Because the\n\
                 \x20       // pink filter's gain at fixed physical f scales as K_pink·fs/f,\n\
                 \x20       // the white-input variance must be fs-INDEPENDENT for the output\n\
                 \x20       // PSD to land at S_i(f) = KF·I^AF/f (one-sided, ngspice KF/AF\n\
                 \x20       // semantics) at every fs and oversampling factor (the physical 0.5).\n",
            );
            default_stmts.push_str(&format!(
                "        let noise_flicker_scale = {};\n",
                fmt_f64(flicker_scale)
            ));
            default_stmts.push_str("        let noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_FLICKER_SALT);\n");
        }
        if r_flicker_n > 0 {
            default_stmts
                .push_str("        // Resistor-flicker streams (Hooge bias-squared, Phase 3.5).\n");
            default_stmts.push_str(&format!(
                "        // Per-source white-input variance: σ_w² = {}·KF·|I_R|^AF / K_pink —\n",
                "0.5"
            ));
            default_stmts.push_str(
                "        // identical calibration to junction flicker (same Kellett cascade,\n\
                 \x20       // same Norton RHS stamp through the same trap/BE kernel), so\n\
                 \x20       // output PSD lands at S_i = KF·I_R^AF/f one-sided, fs/OS-invariant.\n\
                 \x20       // `noise_r_flicker_sqrt_fs` carries this fs-independent scale\n\
                 \x20       // constant (the name notwithstanding).\n",
            );
            default_stmts.push_str(&format!(
                "        let noise_r_flicker_sqrt_fs = {};\n",
                fmt_f64(flicker_scale)
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
            default_stmts.push_str(
                "        // sqrt(0.5*fs): the physical per-sample amplitude, one draw.\n",
            );
            default_stmts
                .push_str("        let noise_opamp_sqrt_fs = (0.5 * fs_internal).sqrt();\n");
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
        default_fields.push_str("            noise_thermal_last_i_n: [0.0; NOISE_THERMAL_N],\n");
        if need_q_scale {
            default_fields.push_str("            shot_gain: 1.0,\n");
            default_fields.push_str("            noise_shot_scale,\n");
        }
        if shot_n > 0 {
            default_fields.push_str("            noise_shot_rng,\n");
            default_fields
                .push_str("            noise_shot_gaussian_cache: [None; NOISE_SHOT_N],\n");
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
        reset_body.push_str(
            "        // Clear the BE-replay cache so silence after reset is true zero.\n",
        );
        reset_body.push_str("        self.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            reset_body.push_str(
                "        // Re-seed shot RNGs (same master, distinct salt from thermal).\n",
            );
            reset_body.push_str("        self.noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(self.noise_master_seed, NOISE_SHOT_SALT);\n");
            reset_body.push_str("        self.noise_shot_gaussian_cache = [None; NOISE_SHOT_N];\n");
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
            reset_body
                .push_str("        self.noise_r_flicker_inv_r = NOISE_R_FLICKER_INV_R_DEFAULT;\n");
        }
        if partition_n > 0 {
            reset_body.push_str("        // Re-seed partition RNGs + clear the BE-replay cache.\n");
            reset_body.push_str("        self.noise_partition_rng = seed_noise_rngs_salted::<NOISE_PARTITION_N>(self.noise_master_seed, NOISE_PARTITION_SALT);\n");
            reset_body.push_str(
                "        self.noise_partition_gaussian_cache = [None; NOISE_PARTITION_N];\n",
            );
            reset_body
                .push_str("        self.noise_partition_last_i_n = [0.0; NOISE_PARTITION_N];\n");
        }
        if opamp_n > 0 {
            reset_body
                .push_str("        // Re-seed op-amp en/in RNGs + clear the BE-replay cache.\n");
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
            reset_body.push_str("        self.noise_opamp_en_last_i_n = [0.0; NOISE_OPAMP_N];\n");
            reset_body
                .push_str("        self.noise_opamp_in_last_i_n = [0.0; NOISE_OPAMP_IN_N];\n");
        }

        // set_sample_rate tail: recompute noise scales at the new rate — keep
        // in sync with the Default derivation above.
        let mut ssr_body = String::new();
        ssr_body.push_str("        // Noise: recompute rate-dependent scales for the new rate.\n");
        ssr_body.push_str("        self.noise_fs = sample_rate * OVERSAMPLING_FACTOR as f64;\n");
        ssr_body.push_str("        self.noise_thermal_scale = (2.0 * K_B * self.temperature_k * self.noise_fs).sqrt();\n");
        if need_q_scale {
            ssr_body.push_str("        self.noise_shot_scale = (Q_E * self.noise_fs).sqrt();\n");
        }
        if opamp_n > 0 {
            ssr_body.push_str("        self.noise_opamp_sqrt_fs = (0.5 * self.noise_fs).sqrt();\n");
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
            "    /// A non-finite gain is counted in `diag_runtime_nan_count` and ignored.\n\
             \x20   pub fn set_noise_gain(&mut self, gain: f64) { if gain.is_finite() { self.noise_gain = gain; } else { self.diag_runtime_nan_count += 1; } }\n\n",
        );
        methods.push_str("    /// Scalar applied only to Johnson-Nyquist thermal sources.\n");
        methods.push_str(
            "    /// A non-finite gain is counted in `diag_runtime_nan_count` and ignored.\n\
             \x20   pub fn set_thermal_gain(&mut self, gain: f64) { if gain.is_finite() { self.thermal_gain = gain; } else { self.diag_runtime_nan_count += 1; } }\n\n",
        );
        methods.push_str("    /// Circuit temperature in Kelvin. 290 K is standard (~16.85 °C).\n");
        methods.push_str(
            "    /// Cold gear is quieter: 77 K (liquid N2) ≈ −5.76 dB, 3 K ≈ −19.9 dB.\n",
        );
        methods.push_str(
            "    /// A non-finite value is counted in `diag_runtime_nan_count` and ignored;\n",
        );
        methods.push_str("    /// a finite value at or below 0 K is ignored.\n");
        methods.push_str("    pub fn set_temperature_k(&mut self, kelvin: f64) {\n");
        methods.push_str(
            "        if !kelvin.is_finite() { self.diag_runtime_nan_count += 1; return; }\n",
        );
        methods.push_str("        if !(kelvin > 0.0) { return; }\n");
        methods.push_str("        self.temperature_k = kelvin;\n");
        methods.push_str("        // Recompute thermal_scale at the currently-set sample rate.\n");
        methods.push_str(
            "        self.noise_thermal_scale = (2.0 * K_B * kelvin * self.noise_fs).sqrt();\n",
        );
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
                "    /// A non-finite gain is counted in `diag_runtime_nan_count` and ignored.\n\
                 \x20   pub fn set_shot_gain(&mut self, gain: f64) { if gain.is_finite() { self.shot_gain = gain; } else { self.diag_runtime_nan_count += 1; } }\n\n",
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
                "    /// A non-finite gain is counted in `diag_runtime_nan_count` and ignored.\n\
                 \x20   pub fn set_flicker_gain(&mut self, gain: f64) { if gain.is_finite() { self.flicker_gain = gain; } else { self.diag_runtime_nan_count += 1; } }\n\n",
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
            methods.push_str(
                "    /// A non-finite gain is counted in `diag_runtime_nan_count` and ignored.\n",
            );
            methods.push_str("    pub fn set_opamp_input_gain(&mut self, gain: f64) { if gain.is_finite() { self.opamp_input_gain = gain; } else { self.diag_runtime_nan_count += 1; } }\n\n");
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
        // Clear the replay caches so sample 0 after re-seed depends on the new
        // RNG stream alone.
        methods.push_str("        self.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            methods.push_str("        self.noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(master, NOISE_SHOT_SALT);\n");
            methods.push_str("        self.noise_shot_gaussian_cache = [None; NOISE_SHOT_N];\n");
            methods.push_str("        self.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            methods.push_str("        self.noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(master, NOISE_FLICKER_SALT);\n");
            methods
                .push_str("        self.noise_flicker_gaussian_cache = [None; NOISE_FLICKER_N];\n");
            methods.push_str("        self.noise_flicker_state = [[0.0; 7]; NOISE_FLICKER_N];\n");
            methods.push_str("        self.noise_flicker_last_i_n = [0.0; NOISE_FLICKER_N];\n");
        }
        if r_flicker_n > 0 {
            methods.push_str("        self.noise_r_flicker_rng = seed_noise_rngs_salted::<NOISE_R_FLICKER_N>(master, NOISE_R_FLICKER_SALT);\n");
            methods.push_str(
                "        self.noise_r_flicker_gaussian_cache = [None; NOISE_R_FLICKER_N];\n",
            );
            methods
                .push_str("        self.noise_r_flicker_state = [[0.0; 7]; NOISE_R_FLICKER_N];\n");
            methods.push_str("        self.noise_r_flicker_last_i_n = [0.0; NOISE_R_FLICKER_N];\n");
        }
        if partition_n > 0 {
            methods.push_str("        self.noise_partition_rng = seed_noise_rngs_salted::<NOISE_PARTITION_N>(master, NOISE_PARTITION_SALT);\n");
            methods.push_str(
                "        self.noise_partition_gaussian_cache = [None; NOISE_PARTITION_N];\n",
            );
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
        // One draw per source per sample: the physical Johnson current at n+1,
        // sqrt(4kT/R · fs/2). kTC verified in
        // tests/noise_psd_validation.rs::thermal_noise_matches_ktc_theorem.
        rhs_stamp
            .push_str("        // Thermal: the physical current at n+1, one draw per source.\n");
        rhs_stamp.push_str("        let scale_th = state.noise_thermal_scale * state.noise_gain * state.thermal_gain;\n");
        rhs_stamp.push_str("        if scale_th != 0.0 {\n");
        rhs_stamp.push_str("            for k in 0..NOISE_THERMAL_N {\n");
        rhs_stamp.push_str("                let g = gaussian(&mut state.noise_rng[k], &mut state.noise_gaussian_cache[k]);\n");
        rhs_stamp.push_str(
            "                let i_n = scale_th * state.noise_thermal_sqrt_inv_r[k] * g;\n",
        );
        rhs_stamp.push_str("                state.noise_thermal_last_i_n[k] = i_n;\n");
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
            // One draw per sample at the physical sqrt(q·|I|·fs).
            rhs_stamp.push_str(
                "        // Shot: sqrt(q·|I_prev|·fs), the physical current at n+1, one draw.\n",
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
            rhs_stamp.push_str("            state.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
            rhs_stamp.push_str("        }\n");
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
            rhs_stamp.push_str(
                "                let i_n = amp * pink; // the physical current at n+1, one draw\n",
            );
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
            // per source — en at in+, in+ at in+, in- at in-, one physical
            // draw each per sample.
            rhs_stamp.push_str(
                "        // Op-amp en/in: 3 streams per source, one draw each per sample.\n",
            );
            rhs_stamp
                .push_str("        // en amp = EN · noise_opamp_en_g_diag · sqrt(2·fs)  at in+\n");
            rhs_stamp.push_str(
                "        // in amp = IN · sqrt(2·fs)                          at in+ and in-\n",
            );
            rhs_stamp
                .push_str("        let oa_scale = state.opamp_input_gain * state.noise_gain;\n");
            // noise_opamp_sqrt_fs is the physical sqrt(0.5*fs) factor.
            rhs_stamp.push_str("        let oa_scale_half = oa_scale;\n");
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
            rhs_stamp.push_str(
                "                    let i_n = w_new; // one draw: the physical current at n+1\n",
            );
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
            rhs_stamp.push_str(
                "                    let i_n = w_new; // one draw: the physical current at n+1\n",
            );
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
            rhs_stamp.push_str(
                "                    let i_n = w_new; // one draw: the physical current at n+1\n",
            );
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
            // One draw per sample. Zero total current → skip (zero amp,
            // pre-Kellett zero-current guard convention).
            rhs_stamp.push_str(
                "        // Pentode partition: i_n = (shot_scale * shot_gain * noise_gain)\n",
            );
            rhs_stamp.push_str("        //                                                 · sqrt(I_p·I_s/(I_p+I_s)) · PARTITION_F · N(0,1).\n");
            rhs_stamp.push_str(
                "        // Reuses shot_gain (partition is shot at a different barrier).\n",
            );
            // noise_shot_scale is the physical sqrt(Q_E*fs).
            rhs_stamp.push_str("        let part_scale_half = state.noise_shot_scale * state.noise_gain * state.shot_gain;\n");
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
            rhs_stamp.push_str("                    // Pre-bias / unbiased: emit zero.\n");
            rhs_stamp.push_str("                    state.noise_partition_last_i_n[k] = 0.0;\n");
            rhs_stamp.push_str("                    continue;\n");
            rhs_stamp.push_str("                }\n");
            rhs_stamp.push_str("                let psd_coef = ip * is_c / total;\n");
            rhs_stamp.push_str("                let g = gaussian(&mut state.noise_partition_rng[k], &mut state.noise_partition_gaussian_cache[k]);\n");
            rhs_stamp.push_str("                let w_new = part_scale_half * psd_coef.sqrt() * NOISE_PARTITION_F[k] * g;\n");
            rhs_stamp.push_str(
                "                let i_n = w_new; // one draw: the physical current at n+1\n",
            );
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
            rhs_stamp.push_str(
                "                let i_n = amp * pink; // the physical current at n+1, one draw\n",
            );
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
        rhs_stamp_be.push_str("\n        // BE-fallback noise replay (re-stamps this sample's cached i_n into rhs_be).\n");
        rhs_stamp_be.push_str(&emit_noise_replay_body(replay_counts, "rhs_be", "        "));

        // NaN-recovery noise reset: clear the BE-replay caches so a NaN-induced state.v_prev = DC_OP recovery
        // also produces a clean noise sequence (no stale draw paired with
        // the post-recovery sample). RNG itself is NOT re-seeded here —
        // determinism contract says set_seed is the only re-seed entry.
        let mut nan_recovery_body = String::new();
        nan_recovery_body
            .push_str("        // Noise: clear the BE-replay caches (RNG seed preserved).\n");
        nan_recovery_body
            .push_str("        state.noise_thermal_last_i_n = [0.0; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_shot_last_i_n = [0.0; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_flicker_last_i_n = [0.0; NOISE_FLICKER_N];\n");
            nan_recovery_body
                .push_str("        state.noise_flicker_state = [[0.0; 7]; NOISE_FLICKER_N];\n");
        }
        if r_flicker_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_r_flicker_last_i_n = [0.0; NOISE_R_FLICKER_N];\n");
            nan_recovery_body
                .push_str("        state.noise_r_flicker_state = [[0.0; 7]; NOISE_R_FLICKER_N];\n");
        }
        if partition_n > 0 {
            nan_recovery_body
                .push_str("        state.noise_partition_last_i_n = [0.0; NOISE_PARTITION_N];\n");
        }
        if opamp_n > 0 {
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
