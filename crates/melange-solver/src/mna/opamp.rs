//! Op-amp info, default card constants and output-swing resolution.

/// Op-amp information for VCCS stamping.
///
/// Modeled as a voltage-controlled current source with transconductance
/// Gm = AOL / Rout, plus output conductance Go = 1 / Rout. This stamps
/// directly into the G matrix and does NOT add nonlinear dimensions.
#[derive(Debug, Clone)]
pub struct OpampInfo {
    pub name: String,
    /// Non-inverting input node index (0 = ground)
    pub n_plus_idx: usize,
    /// Inverting input node index (0 = ground)
    pub n_minus_idx: usize,
    /// Output node index (0 = ground)
    pub n_out_idx: usize,
    /// Open-loop gain (default 200,000)
    pub aol: f64,
    /// Open-loop small-signal output resistance \[Ω\] (`ROUT`, default
    /// [`OPAMP_DEFAULT_ROUT_OHM`]). The linear model's output impedance; in
    /// closed loop it is divided by the loop gain.
    pub r_out: f64,
    /// Saturated output sag \[Ω\] (`R_SAG`, default [`OPAMP_DEFAULT_R_SAG_OHM`]):
    /// a railed output sits at `limit − R_SAG·I_load`. A different mechanism
    /// from `r_out` (output-stage series resistance plus drive starvation at
    /// clip), 2–4× larger on the audio parts.
    pub r_sag: f64,
    /// Highest output voltage the op-amp can drive \[V\] — its upper swing
    /// limit, the level every rail mode clamps or pins at (default +inf =
    /// none). Resolved from the card by [`resolve_opamp_swing`]:
    /// `VCC − VOH_DROP`, or `+VSAT`, or +13 V when only `GBW` is given.
    pub vcc: f64,
    /// Lowest output voltage the op-amp can drive \[V\] — its lower swing limit
    /// (default −inf = none): `VEE + VOL_DROP`, or `−VSAT`, or −13 V when
    /// only `GBW` is given.
    pub vee: f64,
    /// Gain-bandwidth product \[Hz\] (default: infinity = no dominant pole).
    /// When finite, a dominant pole capacitor C = AOL / (2π × GBW × ROUT)
    /// is stamped at an internal Boyle gain node.
    /// Typical NE5534: 10e6. TL074: 3e6.
    pub gbw: f64,
    /// Slew rate [V/s] (default: infinity = no slew limiting).
    ///
    /// Models the large-signal output voltage-rate limit of real op-amps.
    /// Parsed from the `.model` card in V/μs (SPICE convention) and converted
    /// to V/s internally. Examples: TL072 = 13 V/μs → `sr = 13e6`; NE5532 =
    /// 9 V/μs → `sr = 9e6`; LM358 = 0.3 V/μs → `sr = 0.3e6`.
    ///
    /// Emitted as a per-sample voltage-delta clamp on the op-amp output node
    /// (equivalent to clamping the Boyle dominant-pole integrator input current
    /// to ±`SR * C_dom`). When `sr` is infinite no clamp is emitted, so circuits
    /// without `SR=` in their .model get byte-identical generated code to the
    /// pre-slew-rate behaviour.
    pub sr: f64,
    /// Input bias current \[A\] (default 0 = ideal, no bias).
    ///
    /// Models the small DC current that flows into (or out of) each op-amp
    /// input pin on real hardware. Parsed from `.model OA(IB=...)`; typical
    /// values: TL074 = 30e-12 (30 pA JFET input), LM358 = 45e-9 (45 nA
    /// bipolar), NE5532 = 200e-9, OPA134 = 100e-15.
    ///
    /// Stamped as a symmetric DC current source pushing `IB` into both
    /// `n_plus` and `n_minus` from the external circuit's perspective (sign
    /// convention: positive IB = current flows out of the op-amp input pin
    /// into the external node — appropriate for PNP-input bipolar op-amps and
    /// JFET-input parts where gate leakage is outgoing). For NPN-input op-amps
    /// specify `IB=-45n` etc.
    ///
    /// Physical significance beyond DC offset: at integrator nodes the real-
    /// hardware bias current + finite input resistance (modeled via `RIN`)
    /// together drain accumulated charge off the feedback cap, bounding
    /// integrator wind-up on any residual DC offset at the input. A purely
    /// ideal op-amp with no IB and infinite input impedance winds up
    /// unbounded when the upstream network imposes a DC offset — a known
    /// failure mode of melange-emitted transient NR on circuits like the
    /// sidechain integrator of a bus compressor.
    pub ib: f64,
    /// Input resistance \[Ω\] from each input pin to ground (default +∞ = no
    /// leakage path, ideal). Typical values: TL074 (JFET) = 1e12, LM358
    /// (bipolar) = 1e6 to 1e7, NE5532 = 3e5.
    ///
    /// Stamped as a shunt conductance `1/RIN` at each input node. Provides
    /// a DC leakage path that bounds integrator wind-up: a 33-µs integrator
    /// with 1 TΩ input resistance will have a 10-second wind-up decay
    /// envelope. Finite RIN is what makes real integrator circuits stable
    /// under DC offsets that would otherwise cause ideal-op-amp models to
    /// drift without bound.
    pub rin: f64,
    /// `.model OA(AOL_TRANSIENT_CAP=N)`: the AOL the transient solve uses, for
    /// this op-amp only (default `INFINITY` = full AOL). The DC operating point
    /// keeps the full AOL. Applied by the nodal IR builder; a card that sets it
    /// routes nodal (`routing::auto_route`).
    pub aol_transient_cap: f64,
    /// Boyle internal gain node index (1-indexed, 0 = none).
    /// No longer used with IIR op-amp model (always 0).
    pub n_internal_idx: usize,
    /// Dominant pole capacitance for IIR filter: C_dom = AOL / (2*pi*GBW*ROUT).
    /// Set during MNA stamping when GBW is finite; 0.0 when GBW is infinite.
    pub iir_c_dom: f64,
    /// `BoyleDiodes`-mode internal gain node index (1-indexed, 0 = not in
    /// BoyleDiodes mode). When non-zero, the op-amp's transconductance is
    /// stamped at this node instead of at `n_out_idx`, with the high
    /// impedance load `R_BOYLE_INT_LOAD` setting both Gm and Go so the
    /// catch diodes (placed externally between `n_int_idx` and rail
    /// references) only have to balance against ~1 µS instead of the
    /// op-amp's nominal Gm = AOL/r_out (typically 4000 S for TL072). The
    /// op-amp output node is then driven by an external unity-gain buffer
    /// VCCS sourced from this node — see
    /// [`crate::codegen::ir::augment_netlist_with_boyle_diodes`].
    ///
    /// Auto-detected during MNA stamping by looking up
    /// `_oa_int_{safe_name}` in `node_map`. The augment helper synthesizes
    /// that name (referenced from buffer/diode elements), so the field is
    /// non-zero only when the op-amp has gone through the augmentation pass.
    pub n_int_idx: usize,
    /// Input-referred voltage noise spectral density [V/√Hz] at the non-
    /// inverting input (Phase 4). Default 0.0 = no en noise emitted; users
    /// opt in via `.model OA(EN=…)`. Typical datasheet values: NE5534 =
    /// 3.5e-9, 4558 = 8e-9, TL072 = 18e-9, OP07 = 10e-9.
    /// Stamped as a Norton current at `n_plus_idx` scaled by the live
    /// diagonal `G[n_plus, n_plus]` — voltage-source-in-series-with-input
    /// equivalent without inserting a netlist resistor.
    pub en: f64,
    /// Input-referred current noise spectral density [A/√Hz] at each input
    /// (Phase 4). Default 0.0 = no in noise emitted. Typical datasheet:
    /// NE5534 = 1.5e-12, 4558 = 0.5e-12, TL072 = 0.01e-12 (FET-input).
    /// Stamped as an independent Norton current at each of `n_plus_idx`
    /// and `n_minus_idx` — two uncorrelated streams per op-amp.
    pub in_amps: f64,
}

/// Drop from a supply rail to the op-amp's zero-load swing limit when the card
/// sets `VCC`/`VEE` without `VOH_DROP`/`VOL_DROP` \[V\]. The load sag is
/// `R_SAG·I_load` on top, so this is the zero-load intercept of the
/// datasheet V_OM-versus-load line at ±15 V: vintage TL072 1.06 V (TI SLOS080
/// rev D), 741 0.73 V (SLOS094 rev B), 4558 0.82 V (SLOS073 rev H), NE5532
/// 1.27 V (Philips 1997). A modern TL072 die is about 0.2 V.
pub const OPAMP_DEFAULT_RAIL_DROP_V: f64 = 1.0;

/// Open-loop output resistance when the card sets no `ROUT` \[Ω\]: the 741's
/// documented r_o (TI SLOS094 rev B, note 5). A new TL07x die is 125 Ω at
/// 1 MHz (SLOS080 rev W), an NE5532 about 10 Ω.
pub const OPAMP_DEFAULT_ROUT_OHM: f64 = 75.0;

/// Saturated output sag when the card sets no `R_SAG` \[Ω\]: the slope of the
/// datasheet V_OM-versus-load line at ±15 V, 10 kΩ to 2 kΩ. Vintage TL072
/// 323 Ω (SLOS080 rev D), 741 196 Ω (SLOS094 rev B), 4558 55 Ω (rev H) or
/// 196 Ω (rev G), NE5532 34 Ω (2 kΩ to 600 Ω, Philips 1997). A linear fit;
/// not valid near the output's current limit.
pub const OPAMP_DEFAULT_R_SAG_OHM: f64 = 200.0;

/// Swing limit an op-amp card with `GBW` but no `VCC`/`VEE`/`VSAT` gets \[V\].
pub const OPAMP_GBW_DEFAULT_SWING_V: f64 = 13.0;

/// The swing-related keys of an op-amp `.model` card, as written.
#[derive(Debug, Clone, Default)]
pub struct OpampSwingCard {
    pub vcc: Option<f64>,
    pub vee: Option<f64>,
    pub vsat: Option<f64>,
    pub voh_drop: Option<f64>,
    pub vol_drop: Option<f64>,
}

/// An op-amp's output swing limits, and the notices resolving them produced.
#[derive(Debug, Clone)]
pub struct OpampSwing {
    pub high: f64,
    pub low: f64,
    pub notices: Vec<String>,
}

/// Resolve an op-amp card's output swing limits — the one level every rail
/// mode (hard clamp, active-set pin, Boyle catch diodes) uses.
///
/// `VCC`/`VEE` are the supply rails; the output reaches `VCC − VOH_DROP` and
/// `VEE + VOL_DROP` (drop default [`OPAMP_DEFAULT_RAIL_DROP_V`], with a
/// notice). `VSAT` gives a symmetric swing limit directly. `has_gbw` without
/// any of them gives ±[`OPAMP_GBW_DEFAULT_SWING_V`]. Each side resolves on
/// its own, so `VCC=9` alone leaves the lower side to the GBW default or
/// unlimited.
///
/// Refused: `VSAT` together with `VCC` or `VEE` (two keys claiming the same
/// limit); a drop without its rail (it would be inert); a negative or
/// non-finite drop; an empty swing.
pub fn resolve_opamp_swing(
    name: &str,
    card: &OpampSwingCard,
    has_gbw: bool,
) -> Result<OpampSwing, String> {
    if card.vsat.is_some() && (card.vcc.is_some() || card.vee.is_some()) {
        return Err(format!(
            "Op-amp {name}: the card sets VSAT and VCC/VEE. VSAT sets the swing limit \
             directly; with VCC/VEE use VOH_DROP/VOL_DROP instead."
        ));
    }
    for (drop, rail, drop_key, rail_key) in [
        (card.voh_drop, card.vcc, "VOH_DROP", "VCC"),
        (card.vol_drop, card.vee, "VOL_DROP", "VEE"),
    ] {
        if let Some(d) = drop {
            if rail.is_none() {
                return Err(format!(
                    "Op-amp {name}: {drop_key} is the drop from {rail_key}, and the card \
                     sets no {rail_key}. Set {rail_key}, or give the swing limit directly \
                     with VSAT."
                ));
            }
            if !(d.is_finite() && d >= 0.0) {
                return Err(format!(
                    "Op-amp {name}: {drop_key} must be non-negative and finite, got {d}"
                ));
            }
        }
    }
    let mut notices = Vec::new();
    let mut side =
        |rail: Option<f64>, drop: Option<f64>, sign: f64, rail_key: &str, drop_key: &str| {
            if let Some(r) = rail {
                let d = drop.unwrap_or(OPAMP_DEFAULT_RAIL_DROP_V);
                let limit = r - sign * d;
                if drop.is_none() {
                    notices.push(format!(
                        "Op-amp {name}: zero-load swing limit assumed {rail_key} {} \
                     {OPAMP_DEFAULT_RAIL_DROP_V} V = {limit} V (a railed output sags R_SAG·I_load \
                     below it); set {drop_key} (0 for rail-to-rail parts).",
                        if sign > 0.0 { "−" } else { "+" }
                    ));
                }
                limit
            } else if let Some(v) = card.vsat {
                sign * v
            } else if has_gbw {
                sign * OPAMP_GBW_DEFAULT_SWING_V
            } else {
                sign * f64::INFINITY
            }
        };
    let high = side(card.vcc, card.voh_drop, 1.0, "VCC", "VOH_DROP");
    let low = side(card.vee, card.vol_drop, -1.0, "VEE", "VOL_DROP");
    // Both finite and crossed would panic inside `f64::clamp(min, max)` on the
    // audio thread at the first sample; a one-sided limit is legal.
    if high.is_finite() && low.is_finite() && high <= low {
        return Err(format!(
            "Op-amp {name}: the output swing is empty: upper limit {high} V <= lower \
             limit {low} V. VCC − VOH_DROP must stay above VEE + VOL_DROP, and VSAT \
             must be positive."
        ));
    }
    Ok(OpampSwing { high, low, notices })
}

/// Effective output resistance for the
/// [`OpampRailMode::BoyleDiodes`](crate::codegen::OpampRailMode::BoyleDiodes) internal gain node (Ω).
///
/// Both `Gm_int = AOL / R_BOYLE_INT_LOAD` and `Go_int = 1 / R_BOYLE_INT_LOAD`
/// are derived from this single value, so the open-loop voltage gain
/// from V+/V- to the internal node is preserved at AOL: a `Gm * v_diff`
/// current source feeding a `1/R` shunt sets `V_int = AOL · v_diff`
/// regardless of the absolute value of `R`. The choice trades two
/// constraints:
///
/// - **Catch-diode dominance**: at deep saturation the catch diode runs
///   at ~1 S forward conductance, so we need `1/R_BOYLE_INT_LOAD` to be
///   ≪ 1 S for the diode to anchor V_int. ⇒ R ≫ 1 Ω.
/// - **MNA pivoting against the rest of the circuit**: the off-diagonal
///   `Gm_int = AOL/R` should be in the same magnitude range as the
///   user circuit's typical entries (1 µS to 1 mS) to keep partial
///   pivoting numerically well-conditioned. With `AOL = 200000`,
///   `R = 1 MΩ` gives `Gm = 0.2 S` — comfortably above the user
///   circuit's ~1 mS ceiling so the int-node row is consistently
///   chosen as the pivot for the V+/V- columns, and the diode load
///   `Go = 1 µS` is well below the diode's full-on 1 S so the diode
///   still anchors the rail.
pub const R_BOYLE_INT_LOAD: f64 = 1.0e6;
