//! Shared device type definitions used by both runtime solvers and code generation.
//!
//! These types are extracted from `codegen::ir` so that runtime code (`solver`, `mna`,
//! `dc_op`) can use them without pulling in the `tera` template engine dependency.

use serde::{Deserialize, Serialize};

/// Per-device resolved parameters, stored in each `DeviceSlot`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceParams {
    Diode(DiodeParams),
    Bjt(BjtParams),
    Jfet(JfetParams),
    Mosfet(MosfetParams),
    Tube(TubeParams),
    Vca(VcaParams),
}

impl DeviceParams {}

/// Diode model parameters (resolved from `.model` directive or defaults).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiodeParams {
    /// Saturation current
    pub is: f64,
    /// Ideality factor * thermal voltage
    pub n_vt: f64,
    /// Zero-bias junction capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cjo: f64,
    /// Series resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rs: f64,
    /// Reverse breakdown voltage [V] (infinity = disabled)
    #[serde(
        default = "default_infinity",
        deserialize_with = "deserialize_f64_or_infinity"
    )]
    pub bv: f64,
    /// Reverse breakdown current [A] (default 1e-10)
    #[serde(default = "default_ibv")]
    pub ibv: f64,
}

impl DiodeParams {
    /// Returns true if series resistance is enabled.
    pub fn has_rs(&self) -> bool {
        self.rs > 0.0
    }
    /// Returns true if reverse breakdown is enabled.
    pub fn has_bv(&self) -> bool {
        self.bv.is_finite()
    }
}

/// BJT parameters (Ebers-Moll or Gummel-Poon, resolved from `.model` directive).
///
/// When `vaf`, `var`, `ikf`, `ikr` are all infinite (the default), this reduces
/// to the basic Ebers-Moll model (backward compatible). Any finite GP parameter
/// activates the Gummel-Poon extension with Early effect and high-injection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BjtParams {
    /// Saturation current
    pub is: f64,
    /// Thermal voltage
    pub vt: f64,
    /// Forward current gain
    pub beta_f: f64,
    /// Reverse current gain
    pub beta_r: f64,
    /// True if PNP (false = NPN)
    #[serde(default)]
    pub is_pnp: bool,
    /// Forward Early voltage [V] (inf = no Early effect)
    #[serde(
        default = "default_infinity",
        deserialize_with = "deserialize_f64_or_infinity"
    )]
    pub vaf: f64,
    /// Reverse Early voltage [V] (inf = no Early effect)
    #[serde(
        default = "default_infinity",
        deserialize_with = "deserialize_f64_or_infinity"
    )]
    pub var: f64,
    /// Forward knee current [A] (inf = no high injection)
    #[serde(
        default = "default_infinity",
        deserialize_with = "deserialize_f64_or_infinity"
    )]
    pub ikf: f64,
    /// Reverse knee current [A] (inf = no high injection)
    #[serde(
        default = "default_infinity",
        deserialize_with = "deserialize_f64_or_infinity"
    )]
    pub ikr: f64,
    /// Base-emitter junction capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cje: f64,
    /// Base-collector junction capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cjc: f64,
    /// Forward emission coefficient (1.0 = ideal, default)
    #[serde(default = "default_one")]
    pub nf: f64,
    /// B-E leakage saturation current [A] (0.0 = disabled, default)
    #[serde(default)]
    pub ise: f64,
    /// B-E leakage emission coefficient (default 1.5)
    #[serde(default = "default_ne")]
    pub ne: f64,
    /// Reverse emission coefficient (1.0 = ideal, default)
    #[serde(default = "default_one")]
    pub nr: f64,
    /// B-C leakage saturation current [A] (0.0 = disabled, default)
    #[serde(default)]
    pub isc: f64,
    /// B-C leakage emission coefficient (default 2.0)
    #[serde(default = "default_nc")]
    pub nc: f64,
    /// Base series resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rb: f64,
    /// Collector series resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rc: f64,
    /// Emitter series resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub re: f64,
    /// Thermal resistance [K/W] (inf = disabled, default)
    #[serde(
        default = "default_infinity",
        deserialize_with = "deserialize_f64_or_infinity"
    )]
    pub rth: f64,
    /// Thermal capacitance [J/K] (default 1e-3, typical TO-92)
    #[serde(default = "default_cth")]
    pub cth: f64,
    /// IS temperature exponent (default 3.0)
    #[serde(default = "default_xti")]
    pub xti: f64,
    /// Bandgap energy [eV] (default 1.11, silicon)
    #[serde(default = "default_eg")]
    pub eg: f64,
    /// Ambient temperature [K] (default 300.15 = 27C)
    #[serde(default = "default_tamb")]
    pub tamb: f64,
}

impl BjtParams {
    /// Returns true if any Gummel-Poon parameter is finite.
    pub fn is_gummel_poon(&self) -> bool {
        self.vaf.is_finite() || self.var.is_finite() || self.ikf.is_finite() || self.ikr.is_finite()
    }
    /// Returns true if non-ideal emission coefficient (NF != 1.0) is active.
    pub fn has_nf(&self) -> bool {
        (self.nf - 1.0).abs() > 1e-15
    }
    /// Returns true if B-E leakage current (ISE) is enabled.
    pub fn has_ise(&self) -> bool {
        self.ise > 0.0
    }
    /// Returns true if non-ideal reverse emission coefficient (NR != 1.0) is active.
    pub fn has_nr(&self) -> bool {
        (self.nr - 1.0).abs() > 1e-15
    }
    /// Returns true if B-C leakage current (ISC) is enabled.
    pub fn has_isc(&self) -> bool {
        self.isc > 0.0
    }
    /// Returns true if any parasitic resistance (RB/RC/RE) is enabled.
    pub fn has_parasitics(&self) -> bool {
        self.rb > 0.0 || self.rc > 0.0 || self.re > 0.0
    }
    /// Returns true if self-heating is enabled (RTH is finite).
    pub fn has_self_heating(&self) -> bool {
        self.rth.is_finite()
    }
    /// Returns the 2x2 parasitic R coupling matrix R_p for K_eff = K - R_p.
    ///
    /// Layout: `[R_p[be,be], R_p[be,bc], R_p[bc,be], R_p[bc,bc]]`
    /// where be = Vbe→Ic dimension (start_idx), bc = Vbc→Ib dimension (start_idx+1).
    ///
    /// The DK NR residual with parasitic absorption is:
    ///   f(i) = i - i_device(p + K_eff * i)
    /// where K_eff = K_original - R_p, and i_device uses the intrinsic model.
    pub fn r_p_matrix(&self) -> [f64; 4] {
        [
            self.re,           // R_p[be,be]: Vbe drop from Ic * RE
            self.rb + self.re, // R_p[be,bc]: Vbe drop from Ib * (RB + RE)
            -self.rc,          // R_p[bc,be]: Vbc drop from -Ic * RC
            self.rb,           // R_p[bc,bc]: Vbc drop from Ib * RB
        ]
    }
}

/// A slot in the nonlinear system: maps a device to its M-dimension range.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceSlot {
    /// Device type tag (for NR dispatch)
    pub device_type: DeviceType,
    /// Starting index in the M-dimension vectors
    pub start_idx: usize,
    /// Number of dimensions this device occupies
    pub dimension: usize,
    /// Per-device resolved parameters (from `.model` directive or defaults)
    pub params: DeviceParams,
    /// True if parasitic R is handled via MNA internal nodes (not inner NR loop).
    /// When true, codegen emits direct bjt_evaluate() instead of bjt_with_parasitics().
    #[serde(default)]
    pub has_internal_mna_nodes: bool,
    /// Frozen screen-grid voltage for grid-off pentode reduction (phase 1b).
    ///
    /// When `params` is a `TubeParams` with `kind == SharpPentodeGridOff`,
    /// this field holds the DC-OP-converged value of `Vg2k = V[screen] −
    /// V[cathode]`, which the reduced 2D math uses as a constant in place
    /// of the third NR dimension. Written by the DC-OP grid-off detection
    /// pass; re-computed on warm DC-OP re-init after large pot/switch jumps.
    ///
    /// For all other device types (and for sharp `SharpPentode` slots) this
    /// field is unused and stays at its default of 0.0.
    #[serde(default)]
    pub vg2k_frozen: f64,
}

/// JFET model parameters (resolved from `.model` directive or defaults).
///
/// Codegen uses 2D Shichman-Hodges: Vgs and Vds control Id (triode + saturation regions).
/// Gate current Ig (dimension 2) is effectively zero for reverse-biased gate.
/// This matches the MNA stamping where JFET is 2D (dimension=2, controlling voltages=Vgs, Vds).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JfetParams {
    /// Saturation current IDSS [A]
    pub idss: f64,
    /// Pinch-off voltage [V] (negative for N-channel, positive for P-channel)
    pub vp: f64,
    /// Channel length modulation [1/V]
    pub lambda: f64,
    /// True if P-channel (false = N-channel)
    pub is_p_channel: bool,
    /// Gate-source junction capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cgs: f64,
    /// Gate-drain junction capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cgd: f64,
    /// Drain ohmic resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rd: f64,
    /// Source ohmic resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rs: f64,
}

impl JfetParams {
    /// Returns true if either drain or source resistance is enabled.
    pub fn has_rd_rs(&self) -> bool {
        self.rd > 0.0 || self.rs > 0.0
    }
}

/// MOSFET model parameters (Level 1 SPICE, triode + saturation).
///
/// Codegen uses 2D: Vgs and Vds control Id (triode + saturation regions).
/// Gate current Ig (dimension 2) is zero (insulated gate).
/// MNA stamping: 2D (dimension=2, controlling voltages=Vgs, Vds).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MosfetParams {
    /// Transconductance parameter KP [A/V²]
    pub kp: f64,
    /// Threshold voltage VT [V] (positive for N-channel, negative for P-channel)
    pub vt: f64,
    /// Channel length modulation [1/V]
    pub lambda: f64,
    /// True if P-channel (false = N-channel)
    pub is_p_channel: bool,
    /// Gate-source capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cgs: f64,
    /// Gate-drain capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cgd: f64,
    /// Drain ohmic resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rd: f64,
    /// Source ohmic resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rs: f64,
    /// Body effect coefficient GAMMA [V^0.5] (0.0 = disabled)
    #[serde(default)]
    pub gamma: f64,
    /// Surface potential PHI [V] (default 0.6)
    #[serde(default = "default_phi")]
    pub phi: f64,
    /// Source node index in N-dimensional system (needed for body effect Vsb computation)
    #[serde(default)]
    pub source_node: usize,
    /// Bulk node index in N-dimensional system (needed for body effect Vsb computation)
    #[serde(default)]
    pub bulk_node: usize,
}

impl MosfetParams {
    /// Returns true if either drain or source resistance is enabled.
    pub fn has_rd_rs(&self) -> bool {
        self.rd > 0.0 || self.rs > 0.0
    }
    /// Returns true if body effect is enabled.
    pub fn has_body_effect(&self) -> bool {
        self.gamma > 0.0
    }
}

/// Tube kind discriminator.
///
/// Distinguishes between 2D triode (Vgk → Ip, Vpk → Ig) and 3D pentode-family
/// tubes (Vgk → Ip, Vpk → Ig2, Vg2k → Ig1). The dimension of the NR block is
/// 2 for `SharpTriode` and 3 for `SharpPentode`.
///
/// Both true pentodes (EL84/EL34/EF86) and beam tetrodes (6L6/6V6/KT88) share
/// the `SharpPentode` kind — they differ only in the screen-current functional
/// form, which is selected by the separate `ScreenForm` discriminator on
/// `TubeParams`. See [`ScreenForm`] for details.
///
/// Future variants (not yet implemented): `RemoteTriode`, `RemotePentode` for
/// variable-mu (remote-cutoff) tubes used in varimu compressors; `*GridOff`
/// variants for the Vgk<0 → Ig≈0 dimension reduction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[non_exhaustive]
pub enum TubeKind {
    /// Sharp-cutoff triode (2D: Vgk → Ip, Vpk → Ig). The default.
    #[default]
    SharpTriode,
    /// Sharp-cutoff pentode or beam tetrode (3D: Vgk → Ip, Vpk → Ig2, Vg2k → Ig1).
    /// Uses Reefman's "Derk" §4.4 or "DerkE" §4.5 equations depending on
    /// `TubeParams.screen_form`. Includes true pentodes (EL84, EL34, EF86),
    /// beam tetrodes (6L6, 6V6), Classical entries (KT88, 6550), and
    /// variable-mu tubes (6K7, EF89).
    SharpPentode,
    /// Grid-off reduced pentode (2D: Vgk → Ip, Vpk → Ig2).
    ///
    /// Phase 1b reduction: when DC-OP confirms `Vgk < -vgk_onset - margin`
    /// (i.e. well below grid cutoff) across the operating point, the
    /// control-grid current `Ig1 ≡ 0` and the screen voltage `Vg2k` is
    /// approximately held constant by the external screen bypass cap.
    /// Melange takes advantage of this by:
    ///
    /// 1. Dropping the `Ig1` current output entirely (it's identically zero)
    /// 2. Freezing `Vg2k` at the DC-OP-converged value (stored on the
    ///    `DeviceSlot` as runtime state, not in `TubeParams`)
    /// 3. Reducing the NR block from 3D (Vgk/Vpk/Vg2k → Ip/Ig2/Ig1) to
    ///    2D (Vgk/Vpk → Ip/Ig2), with Vg2k passed as a per-slot constant
    ///
    /// This is the BJT-FA analog for pentodes. Unlike BJT FA (which drops
    /// Vbc because Ic really doesn't depend on Vbc in forward-active),
    /// pentode grid-off is a **physics approximation**: the screen voltage
    /// is not truly constant, it just sags by <1% in amps with heavily
    /// bypassed screens (47 µF + 1 kΩ screen-stop is standard). Under
    /// hard plate clipping the real tube's screen current rises as Vp
    /// falls and Vg2k sags slightly; the frozen model misses that motion.
    /// The error is similar in character to Classical Koren's Vp-independent
    /// screen — audible under heavy clipping but not catastrophic.
    ///
    /// **Triode grid-off is NOT a variant of this**. Unlike pentodes where
    /// the screen-grid voltage can be frozen as an approximation, Koren
    /// triode `Ip` genuinely requires both `Vgk` and `Vpk`, and neither
    /// can be frozen (freezing Vpk would destroy the plate-swing signal
    /// we're trying to track). Triodes stay 2D regardless of bias.
    ///
    /// Activated by `--tube-grid-fa auto` (default) when DC-OP detects
    /// a stable grid-cutoff bias. Override with `--tube-grid-fa off` to
    /// force the full 3D path or `--tube-grid-fa on` to force grid-off
    /// on every pentode regardless of bias (for testing).
    SharpPentodeGridOff,
}

/// Screen-current functional form for pentode / beam tetrode math.
///
/// Three equation families are supported, reflecting the historical evolution
/// of audio-pentode SPICE models:
///
/// * **`Rational`** — Reefman "Derk" §4.4 (2016), Eq 23:
///   `Ig2 = (Ip0/Kg2) · (1 + αs / (1 + β·Vp))`
///   Modern, Vp-dependent screen current. Best fit for true pentodes with
///   smooth screen-current rolloff under clipping (EL84, EL34, EF86).
///   Requires 9 parameters including αs/A/β.
///
/// * **`Exponential`** — Reefman "DerkE" §4.5 (2016), Eq 28:
///   `Ig2 = (Ip0/Kg2) · (1 + αs · exp(-(β·Vp)^{3/2}))`
///   Same 9 parameters as Rational, but the `1/(1+β·Vp)` factor is replaced
///   with `exp(-(β·Vp)^{3/2})` in both Ip and Ig2. Required for beam tetrodes
///   with sharper "critical compensation" knees that the rational form can't
///   capture (6L6GC, 6V6GT). The 1.5-power exponent models the faster
///   screen-current falloff of focussed-beam tube geometry.
///
/// * **`Classical`** — Norman Koren 1996 / Cohen-Hélie 2010, Eqs 1-3:
///   `Ip = (E1^Ex / Kg1) · (1 + sgn(E1)) · arctan(Vp/Kvb)`,
///   `Ig2 = (Vg2/μ + Vg1)^Ex / Kg2` (Vp-INDEPENDENT).
///   The original "phenomenological" Koren pentode model. Uses only 6
///   parameters (μ, Ex, Kg1, Kg2, Kp, Kvb — no αs/A/β). The E1 softplus
///   argument uses `Vg1/Vg2` directly (NOT `Vg1/sqrt(Kvb+Vg2²)`), and Kvb
///   plays an entirely different role: it's the arctan knee scale, NOT the
///   softplus denominator. Screen current is Vp-independent, which is a
///   known accuracy gap under heavy plate clipping but is still the best
///   publicly-available fit for tubes without Reefman-style fits (KT88,
///   6550). Used as a bootstrap / fallback; upgrade to Derk when a fitted
///   `(αs, A, β)` triple becomes available.
///
/// The Derk and DerkE forms share the same α = 1 − (Kg1/Kg2)·(1+αs) identity
/// (Eq 27 = Eq 32). Classical does not use α and computes Ig2 directly from
/// the grid-plus-screen drive term.
///
/// Default: `Rational` — preserves phase 1a behavior for all previously
/// serialized TubeParams and for the EL84/EL34/EF86 catalog entries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[non_exhaustive]
pub enum ScreenForm {
    /// Reefman "Derk" §4.4 — rational `1/(1+β·Vp)` screen scaling (phase 1a).
    /// The default for `SharpPentode` when no form is specified.
    /// Fits true pentodes (EL84, EL34, EF86) accurately.
    #[default]
    Rational,
    /// Reefman "DerkE" §4.5 — exponential `exp(-(β·Vp)^{3/2})` screen scaling
    /// (phase 1a.1). Required for beam tetrodes (6L6GC, 6V6GT) whose
    /// critical-compensation knees the rational form cannot capture.
    Exponential,
    /// Classical Norman Koren pentode (phase 1a.2) — `arctan(Vp/Kvb)` plate
    /// knee with Vp-independent screen current. Fallback for tubes without
    /// published Reefman Derk fits (KT88, 6550). Only 6 parameters; the
    /// `alpha_s`/`a_factor`/`beta_factor` fields on `TubeParams` are ignored
    /// when `screen_form == Classical`.
    Classical,
}

/// Tube/triode/pentode model parameters (Koren triode + Reefman "Derk" pentode).
///
/// The `kind` field discriminates triode vs pentode geometry and selects the
/// equation set used by MNA stamping, NR Jacobian shape, and codegen emission.
/// Pentode-only fields (`kg2`, `alpha_s`, `a_factor`, `beta_factor`) are
/// ignored when `kind == SharpTriode`.
///
/// For `kind == SharpPentode`, melange uses Reefman's "Derk" §4.4 equation set
/// (see `docs/aidocs/DEVICE_MODELS.md` and
/// <https://www.dos4ever.com/uTracer3/Theory.pdf>). `kg2` and `alpha_s` must
/// both be strictly positive; `a_factor` and `beta_factor` are typically small
/// positive values and MAY be zero (though the Derk model does not cleanly
/// reduce to classical Koren pentode when αs=0, so fitted data with `alpha_s>0`
/// is required — enforced by `validate()`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TubeParams {
    /// Tube kind (triode vs pentode). Defaults to `SharpTriode` for backward
    /// compatibility with pre-pentode serialized data.
    #[serde(default)]
    pub kind: TubeKind,
    /// Amplification factor (mu)
    pub mu: f64,
    /// Exponent for Koren's equation
    pub ex: f64,
    /// Kg1 coefficient (plate-current sensitivity; inversely proportional)
    pub kg1: f64,
    /// Kp coefficient
    pub kp: f64,
    /// Kvb coefficient (triode: softplus denominator; pentode: arctan knee in Ip)
    pub kvb: f64,
    /// Maximum grid current [A]
    pub ig_max: f64,
    /// Grid current onset voltage [V]
    pub vgk_onset: f64,
    /// Channel-length modulation coefficient [1/V]. 0.0 = disabled (default).
    #[serde(default)]
    pub lambda: f64,
    /// Cathode-grid capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub ccg: f64,
    /// Grid-plate capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub cgp: f64,
    /// Cathode-plate capacitance [F] (0.0 = disabled)
    #[serde(default)]
    pub ccp: f64,
    /// Grid internal resistance [Ohms] (0.0 = disabled)
    #[serde(default)]
    pub rgi: f64,
    /// Kg2: pentode screen-grid current sensitivity (inversely proportional).
    /// Only meaningful when `kind == SharpPentode`. 0.0 = triode default.
    #[serde(default)]
    pub kg2: f64,
    /// Reefman Derk αs: screen-current magnitude scaler in `1 + αs/(1+β·Vp)`
    /// term. Pentode-only; 0.0 = triode default. Typical fitted values: 4–20.
    #[serde(default)]
    pub alpha_s: f64,
    /// Reefman Derk A: plate-voltage linear gain in the `A·Vp/Kg1` term of the
    /// Derk F-factor. Pentode-only; 0.0 is acceptable when fitting pinpoints
    /// it. Typical fitted values: 1e-8 to 1e-3 (A/V).
    #[serde(default)]
    pub a_factor: f64,
    /// Reefman Derk β: reciprocal plate-voltage scale. For `Rational` screen
    /// form (Derk §4.4) this appears in `1/(1+β·Vp)`; for `Exponential`
    /// (DerkE §4.5) it appears in `exp(-(β·Vp)^{3/2})`. Pentode-only;
    /// 0.0 = triode default. Typical fitted values: 0.05–0.3 (1/V) for
    /// Rational, 0.05–0.1 (1/V) for Exponential.
    #[serde(default)]
    pub beta_factor: f64,
    /// Screen-current functional form (`Rational` for true pentodes / Derk §4.4,
    /// `Exponential` for beam tetrodes / DerkE §4.5). Defaults to `Rational`
    /// to preserve pre-phase-1a.1 behavior for all existing pentode catalog
    /// entries and serialized data.
    #[serde(default)]
    pub screen_form: ScreenForm,
    /// Reefman §5 variable-mu section-B amplification factor (μ_b).
    /// When `svar > 0`, the device blends two Koren currents with two different
    /// amplification factors: section A uses the shared `mu` field (as μ_a),
    /// section B uses this `mu_b`. Section A is the high-mu "normal" section
    /// and section B is the low-mu section that dominates at deep cutoff.
    /// Typical values: `μ_a / μ_b ≈ 3–5` for variable-mu pentodes (6K7, 6BA6,
    /// EF89). When `svar == 0` this field is ignored and the math reduces to
    /// the sharp single-section Koren. 0.0 = sharp default.
    #[serde(default)]
    pub mu_b: f64,
    /// Reefman §5 variable-mu blend fraction `s_var ∈ [0, 1]`.
    /// `I_P,Koren_v = (1 − s_var)·I_P,Koren_a + s_var·I_P,Koren_b` (Eq 33).
    /// When `svar == 0` (default) the device is sharp-cutoff and `mu_b`/`ex_b`
    /// are unused. Typical fitted values: `s_var ≈ 0.05–0.1` for variable-mu
    /// pentodes — the smaller section B carries a minority of the total
    /// current but its flatter gm(Vgk) curve dominates under deep bias.
    /// Values outside [0, 1] are rejected by `validate()`.
    #[serde(default)]
    pub svar: f64,
    /// Reefman §5 variable-mu section-B Koren exponent (`x_b` in Eq 34).
    /// Each section of a variable-mu tube has its own exponent — section A
    /// uses the shared `ex` field (as x_a), section B uses `ex_b`. The two
    /// may differ slightly due to per-section geometry. When `svar == 0`
    /// this field is ignored. 0.0 = sharp default.
    #[serde(default)]
    pub ex_b: f64,
}

impl TubeParams {
    /// Returns true if grid internal resistance is enabled.
    pub fn has_rgi(&self) -> bool {
        self.rgi > 0.0
    }

    /// Returns true if this tube is a pentode (in any form — sharp or
    /// grid-off reduced). Both kinds share the same underlying Koren
    /// pentode math, just at different NR dimensionality.
    pub fn is_pentode(&self) -> bool {
        matches!(
            self.kind,
            TubeKind::SharpPentode | TubeKind::SharpPentodeGridOff
        )
    }

    /// Returns true if this tube is specifically a grid-off (reduced)
    /// pentode. Phase 1b optimization: Ig1 dropped, Vg2k frozen at DC-OP
    /// value, NR dimension 3 → 2.
    pub fn is_grid_off_pentode(&self) -> bool {
        matches!(self.kind, TubeKind::SharpPentodeGridOff)
    }

    /// Returns the NR dimension contributed by this tube:
    /// - 2 for triodes (Vgk, Vpk)
    /// - 3 for sharp pentodes (Vgk, Vpk, Vg2k)
    /// - 2 for grid-off pentodes (Vgk, Vpk — Vg2k frozen)
    pub fn dimension(&self) -> usize {
        match self.kind {
            TubeKind::SharpTriode => 2,
            TubeKind::SharpPentode => 3,
            TubeKind::SharpPentodeGridOff => 2,
        }
    }

    /// Validate pentode-only invariants. Returns `Err` with a descriptive message
    /// if the kind requires parameters that aren't set or are out of range.
    pub fn validate(&self) -> Result<(), String> {
        if !self.mu.is_finite() || self.mu <= 0.0 {
            return Err(format!("tube MU must be positive and finite, got {}", self.mu));
        }
        if !self.ex.is_finite() || self.ex <= 0.0 {
            return Err(format!("tube EX must be positive and finite, got {}", self.ex));
        }
        if !self.kg1.is_finite() || self.kg1 <= 0.0 {
            return Err(format!("tube KG1 must be positive and finite, got {}", self.kg1));
        }
        if !self.kp.is_finite() || self.kp <= 0.0 {
            return Err(format!("tube KP must be positive and finite, got {}", self.kp));
        }
        if !self.kvb.is_finite() || self.kvb <= 0.0 {
            return Err(format!("tube KVB must be positive and finite, got {}", self.kvb));
        }
        if self.is_pentode() {
            if !self.kg2.is_finite() || self.kg2 <= 0.0 {
                return Err(format!(
                    "pentode KG2 must be positive and finite, got {}",
                    self.kg2
                ));
            }
            // The Reefman Derk / DerkE variants require αs>0 because αs=0
            // makes Ip=0 identically (see memory/pentode_equations.md). The
            // Classical Koren variant does not use αs/A/β at all, so those
            // fields are allowed (and expected) to be zero for Classical
            // entries. Skip the Derk-specific invariants when Classical.
            let uses_derk_shape = !matches!(self.screen_form, ScreenForm::Classical);
            if uses_derk_shape {
                if !self.alpha_s.is_finite() || self.alpha_s <= 0.0 {
                    return Err(format!(
                        "pentode ALPHA_S (Reefman Derk αs) must be positive and finite, got {}",
                        self.alpha_s
                    ));
                }
                // A and β are allowed to be zero (some fits pinpoint them at 0).
                if !self.a_factor.is_finite() || self.a_factor < 0.0 {
                    return Err(format!(
                        "pentode A_FACTOR (Reefman Derk A) must be non-negative and finite, got {}",
                        self.a_factor
                    ));
                }
                if !self.beta_factor.is_finite() || self.beta_factor < 0.0 {
                    return Err(format!(
                        "pentode BETA_FACTOR (Reefman Derk β) must be non-negative and finite, got {}",
                        self.beta_factor
                    ));
                }
            }
        }
        // Variable-mu §5 (Reefman two-section Koren) constraints — apply to
        // BOTH triodes and pentodes. `svar == 0` means sharp single-section
        // (the default), no further checks. `svar > 0` activates the blend
        // and requires both section-B parameters to be present.
        if !self.svar.is_finite() || self.svar < 0.0 || self.svar > 1.0 {
            return Err(format!(
                "tube SVAR must be in [0, 1] and finite, got {}",
                self.svar
            ));
        }
        if self.svar > 0.0 {
            if !self.mu_b.is_finite() || self.mu_b <= 0.0 {
                return Err(format!(
                    "variable-mu tube MU_B must be positive and finite when svar>0, got {}",
                    self.mu_b
                ));
            }
            if !self.ex_b.is_finite() || self.ex_b <= 0.0 {
                return Err(format!(
                    "variable-mu tube EX_B must be positive and finite when svar>0, got {}",
                    self.ex_b
                ));
            }
            // Variable-mu Classical is not implemented. Reefman §5 two-section
            // Koren is built on top of the Derk softplus structure, not the
            // Classical arctan knee; no known tube needs this combination.
            // Reject at validation time so the codegen can stay single-branch.
            if self.is_pentode() && matches!(self.screen_form, ScreenForm::Classical) {
                return Err(
                    "variable-mu Classical Koren pentodes are not implemented; \
                     use ScreenForm::Rational (6K7/EF89 pattern) for variable-mu tubes"
                        .to_string(),
                );
            }
        }
        Ok(())
    }

    /// Returns true when the tube is a Reefman §5 variable-mu (remote-cutoff)
    /// type, i.e. `svar > 0`. Applies to both triodes and pentodes.
    pub fn is_variable_mu(&self) -> bool {
        self.svar > 0.0
    }
}

/// VCA (Voltage-Controlled Amplifier) parameters.
///
/// Models a THAT 2180 / DBX 2150 Blackmer-style current-mode exponential gain element.
/// Gain law: `I_signal = G0 * exp(-V_control / VSCALE) * V_signal`
/// With THD: `I = gain * (V_sig + thd_factor * V_sig^3)`, `thd_factor = thd * (1 - gain.min(1))`
/// Control port draws no current (ideal high-Z).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VcaParams {
    /// Control voltage scaling (V/neper)
    pub vscale: f64,
    /// Unity-gain conductance (S)
    pub g0: f64,
    /// THD coefficient for gain-dependent cubic distortion (0.0 = ideal, default)
    #[serde(default)]
    pub thd: f64,
}

/// Nonlinear device type tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum DeviceType {
    Diode,
    Bjt,
    /// Forward-active BJT: 1D model (Vbe→Ic only). Vbc is always reverse-biased.
    /// Ib = Ic/BF is folded into N_i stamping. Reduces M by 1.
    BjtForwardActive,
    Jfet,
    Mosfet,
    Tube,
    Vca,
}

// --- Serde helper functions ---

fn default_infinity() -> f64 {
    f64::INFINITY
}

fn default_ibv() -> f64 {
    1e-10
}

fn default_one() -> f64 {
    1.0
}

fn default_ne() -> f64 {
    1.5
}

fn default_nc() -> f64 {
    2.0
}

fn default_cth() -> f64 {
    1e-3
}

fn default_xti() -> f64 {
    3.0
}

fn default_eg() -> f64 {
    1.11
}

fn default_tamb() -> f64 {
    300.15
}

fn default_phi() -> f64 {
    0.6
}

/// Deserialize an f64 that may be null (JSON cannot represent infinity).
/// Maps null → f64::INFINITY so old serialized data is handled gracefully.
fn deserialize_f64_or_infinity<'de, D>(deserializer: D) -> Result<f64, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let opt: Option<f64> = Option::deserialize(deserializer)?;
    Ok(opt.unwrap_or(f64::INFINITY))
}

#[cfg(test)]
mod tube_params_tests {
    use super::*;

    fn triode_12ax7() -> TubeParams {
        TubeParams {
            kind: TubeKind::SharpTriode,
            mu: 100.0,
            ex: 1.4,
            kg1: 3000.0,
            kp: 600.0,
            kvb: 300.0,
            ig_max: 2e-3,
            vgk_onset: 0.5,
            lambda: 0.0,
            ccg: 0.0,
            cgp: 0.0,
            ccp: 0.0,
            rgi: 0.0,
            kg2: 0.0,
            alpha_s: 0.0,
            a_factor: 0.0,
            beta_factor: 0.0,
            screen_form: ScreenForm::Rational,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    fn variable_mu_pentode_6k7() -> TubeParams {
        // Reefman "PenthodeVD" fit from TubeLib.inc — variable-mu pentode
        // with Rational (Derk §4.4) screen form. Same 9 Derk params plus
        // the §5 variable-mu trio (mu_b, svar, ex_b).
        // Values from the background parameter-survey report (2026-04-11).
        TubeParams {
            kind: TubeKind::SharpPentode,
            mu: 15.5,       // μ_a (section A — high-mu region)
            ex: 1.573,      // ex_a (section A exponent)
            kg1: 1407.7,
            kp: 36.0,
            kvb: 1309.0,
            ig_max: 6e-3,
            vgk_onset: 0.5,
            lambda: 0.0,
            ccg: 0.0,
            cgp: 0.0,
            ccp: 0.0,
            rgi: 0.0,
            kg2: 8335.8,
            alpha_s: 4.07,
            a_factor: 1.55e-9, // effectively zero per Reefman fit
            beta_factor: 0.15,
            screen_form: ScreenForm::Rational,
            mu_b: 3.4,
            svar: 0.083,
            ex_b: 1.223,
        }
    }

    fn pentode_el84() -> TubeParams {
        // Reefman "Derk" fit from TubeLib.inc (2016-01-23):
        // BTetrodeD EL84 — plain Derk §4.4, `1/(1+β·Vp)` screen form.
        // `A` is reconstructed from the file's `Aokg1 = A/kg1 = 3.7e-6`, so
        // `A = Aokg1 * kg1 = 3.7e-6 * 117.4 ≈ 4.344e-4`.
        TubeParams {
            kind: TubeKind::SharpPentode,
            mu: 23.36,
            ex: 1.138,
            kg1: 117.4,
            kp: 152.4,
            kvb: 4015.8,
            ig_max: 8e-3,
            vgk_onset: 0.7,
            lambda: 0.0,
            ccg: 0.0,
            cgp: 0.0,
            ccp: 0.0,
            rgi: 0.0,
            kg2: 1275.0,
            alpha_s: 7.66,
            a_factor: 4.344e-4,
            beta_factor: 0.148,
            screen_form: ScreenForm::Rational,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    fn classical_pentode_kt88() -> TubeParams {
        // Cohen-Hélie 2010 DAFx Table 2 (originally Norman Koren 1996).
        // Classical Koren pentode — 6 parameters only, no αs/A/β.
        // Vp-independent screen current, arctan(Vpk/Kvb) plate knee.
        TubeParams {
            kind: TubeKind::SharpPentode,
            mu: 8.8,
            ex: 1.35,
            kg1: 730.0,
            kp: 32.0,
            kvb: 16.0,
            ig_max: 10e-3,
            vgk_onset: 0.7,
            lambda: 0.0,
            ccg: 0.0,
            cgp: 0.0,
            ccp: 0.0,
            rgi: 0.0,
            kg2: 4200.0,
            alpha_s: 0.0, // unused by Classical
            a_factor: 0.0,
            beta_factor: 0.0,
            screen_form: ScreenForm::Classical,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    fn beam_tetrode_6l6gc() -> TubeParams {
        // Reefman "DerkE" fit from TubeLib.inc (2016-01-23):
        // BTetrodeDE 6L6GC — DerkE §4.5, `exp(-(β·Vp)^{3/2})` screen form.
        TubeParams {
            kind: TubeKind::SharpPentode,
            mu: 9.41,
            ex: 1.306,
            kg1: 446.6,
            kp: 45.2,
            kvb: 3205.1,
            ig_max: 10e-3,
            vgk_onset: 0.7,
            lambda: 0.0,
            ccg: 0.0,
            cgp: 0.0,
            ccp: 0.0,
            rgi: 0.0,
            kg2: 6672.5,
            alpha_s: 8.10,
            a_factor: 4.91e-4,
            beta_factor: 0.069,
            screen_form: ScreenForm::Exponential,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    #[test]
    fn triode_dimension_is_2() {
        assert_eq!(triode_12ax7().dimension(), 2);
        assert!(!triode_12ax7().is_pentode());
    }

    #[test]
    fn pentode_dimension_is_3() {
        assert_eq!(pentode_el84().dimension(), 3);
        assert!(pentode_el84().is_pentode());
    }

    #[test]
    fn triode_validates() {
        triode_12ax7().validate().expect("12AX7 should validate");
    }

    #[test]
    fn pentode_validates_with_kg2() {
        pentode_el84().validate().expect("EL84 with KG2 should validate");
    }

    #[test]
    fn pentode_rejects_missing_kg2() {
        let mut bad = pentode_el84();
        bad.kg2 = 0.0;
        assert!(
            bad.validate().is_err(),
            "Pentode with KG2=0 must fail validation"
        );
    }

    #[test]
    fn pentode_rejects_zero_alpha_s() {
        // Reefman Derk §4.4 with αs=0 produces Ip=0 identically — this is a
        // known degeneracy, not a feature. Reject at validation time.
        let mut bad = pentode_el84();
        bad.alpha_s = 0.0;
        assert!(
            bad.validate().is_err(),
            "Pentode with αs=0 must fail validation (Ip would be 0)"
        );
    }

    #[test]
    fn pentode_accepts_zero_a_and_beta() {
        // `A` and `β` are allowed to be 0 (some fits pinpoint them there).
        let mut ok = pentode_el84();
        ok.a_factor = 0.0;
        ok.beta_factor = 0.0;
        ok.validate()
            .expect("Pentode with A=β=0 but αs>0 should validate");
    }

    #[test]
    fn rejects_zero_mu() {
        let mut bad = triode_12ax7();
        bad.mu = 0.0;
        assert!(bad.validate().is_err());
    }

    #[test]
    fn rejects_nan_kg1() {
        let mut bad = triode_12ax7();
        bad.kg1 = f64::NAN;
        assert!(bad.validate().is_err());
    }

    /// Pre-pentode serialized TubeParams (no `kind`, no `kg2` fields) must
    /// deserialize cleanly with `kind = SharpTriode` and `kg2 = 0.0`.
    #[test]
    fn deserialize_pre_pentode_json_is_triode() {
        let legacy_json = r#"{
            "mu": 100.0,
            "ex": 1.4,
            "kg1": 3000.0,
            "kp": 600.0,
            "kvb": 300.0,
            "ig_max": 0.002,
            "vgk_onset": 0.5
        }"#;
        let tp: TubeParams =
            serde_json::from_str(legacy_json).expect("legacy JSON should round-trip");
        assert!(matches!(tp.kind, TubeKind::SharpTriode));
        assert_eq!(tp.kg2, 0.0);
        assert_eq!(tp.mu, 100.0);
    }

    #[test]
    fn pentode_roundtrip_serde() {
        let el84 = pentode_el84();
        let json = serde_json::to_string(&el84).expect("serialize");
        let back: TubeParams = serde_json::from_str(&json).expect("deserialize");
        assert!(matches!(back.kind, TubeKind::SharpPentode));
        assert_eq!(back.kg2, 1275.0);
        assert_eq!(back.mu, 23.36);
        assert_eq!(back.kg1, 117.4);
        assert_eq!(back.alpha_s, 7.66);
        assert!((back.a_factor - 4.344e-4).abs() < 1e-12);
        assert_eq!(back.beta_factor, 0.148);
    }

    #[test]
    fn tube_kind_default_is_sharp_triode() {
        let k: TubeKind = Default::default();
        assert!(matches!(k, TubeKind::SharpTriode));
    }

    #[test]
    fn screen_form_default_is_rational() {
        let sf: ScreenForm = Default::default();
        assert!(matches!(sf, ScreenForm::Rational));
    }

    #[test]
    fn beam_tetrode_validates() {
        let t = beam_tetrode_6l6gc();
        t.validate().expect("6L6GC DerkE fit should validate");
        assert!(matches!(t.screen_form, ScreenForm::Exponential));
        assert!(t.is_pentode());
        assert_eq!(t.dimension(), 3);
    }

    #[test]
    fn beam_tetrode_rejects_zero_alpha_s() {
        let mut bad = beam_tetrode_6l6gc();
        bad.alpha_s = 0.0;
        assert!(
            bad.validate().is_err(),
            "Beam tetrode with αs=0 must fail validation (same constraint as Derk)"
        );
    }

    #[test]
    fn legacy_pentode_json_without_screen_form_is_rational() {
        // Pre-phase-1a.1 serialized pentode TubeParams (no `screen_form` field).
        // Must deserialize as Rational (backward compat).
        let legacy_json = r#"{
            "kind": "SharpPentode",
            "mu": 23.36,
            "ex": 1.138,
            "kg1": 117.4,
            "kp": 152.4,
            "kvb": 4015.8,
            "ig_max": 0.008,
            "vgk_onset": 0.7,
            "kg2": 1275.0,
            "alpha_s": 7.66,
            "a_factor": 4.344e-4,
            "beta_factor": 0.148
        }"#;
        let tp: TubeParams =
            serde_json::from_str(legacy_json).expect("legacy pentode JSON should round-trip");
        assert!(matches!(tp.screen_form, ScreenForm::Rational));
        assert!(tp.is_pentode());
    }

    #[test]
    fn beam_tetrode_roundtrip_serde() {
        let t = beam_tetrode_6l6gc();
        let json = serde_json::to_string(&t).expect("serialize");
        let back: TubeParams = serde_json::from_str(&json).expect("deserialize");
        assert!(matches!(back.screen_form, ScreenForm::Exponential));
        assert_eq!(back.mu, 9.41);
        assert_eq!(back.kg2, 6672.5);
        assert_eq!(back.alpha_s, 8.10);
    }

    #[test]
    fn variable_mu_pentode_validates() {
        let t = variable_mu_pentode_6k7();
        t.validate().expect("6K7 variable-mu pentode should validate");
        assert!(t.is_variable_mu());
        assert_eq!(t.svar, 0.083);
        assert_eq!(t.mu_b, 3.4);
        assert_eq!(t.ex_b, 1.223);
    }

    #[test]
    fn sharp_pentode_is_not_variable_mu() {
        assert!(!pentode_el84().is_variable_mu());
        assert!(!beam_tetrode_6l6gc().is_variable_mu());
    }

    #[test]
    fn variable_mu_rejects_out_of_range_svar() {
        let mut bad = variable_mu_pentode_6k7();
        bad.svar = 1.5;
        assert!(bad.validate().is_err(), "SVAR > 1 must fail");
        bad.svar = -0.1;
        assert!(bad.validate().is_err(), "SVAR < 0 must fail");
    }

    #[test]
    fn variable_mu_rejects_missing_mu_b() {
        let mut bad = variable_mu_pentode_6k7();
        bad.mu_b = 0.0;
        assert!(
            bad.validate().is_err(),
            "svar>0 with mu_b=0 must fail validation"
        );
    }

    #[test]
    fn variable_mu_rejects_missing_ex_b() {
        let mut bad = variable_mu_pentode_6k7();
        bad.ex_b = 0.0;
        assert!(
            bad.validate().is_err(),
            "svar>0 with ex_b=0 must fail validation"
        );
    }

    #[test]
    fn sharp_pentode_allows_zero_mu_b_ex_b() {
        // With svar=0, mu_b and ex_b must be allowed at 0 (the existing
        // sharp pentode case).
        let t = pentode_el84();
        assert_eq!(t.svar, 0.0);
        assert_eq!(t.mu_b, 0.0);
        assert_eq!(t.ex_b, 0.0);
        t.validate().expect("sharp pentode with mu_b=ex_b=0 must validate");
    }

    #[test]
    fn legacy_pentode_json_without_variable_mu_is_sharp() {
        // Pre-phase-1c serialized pentode TubeParams (no `mu_b`/`svar`/`ex_b`
        // fields). Must deserialize as sharp (svar=0, mu_b=0, ex_b=0).
        let legacy_json = r#"{
            "kind": "SharpPentode",
            "mu": 23.36,
            "ex": 1.138,
            "kg1": 117.4,
            "kp": 152.4,
            "kvb": 4015.8,
            "ig_max": 0.008,
            "vgk_onset": 0.7,
            "kg2": 1275.0,
            "alpha_s": 7.66,
            "a_factor": 4.344e-4,
            "beta_factor": 0.148,
            "screen_form": "Rational"
        }"#;
        let tp: TubeParams = serde_json::from_str(legacy_json)
            .expect("legacy pentode JSON (no variable-mu fields) should round-trip");
        assert!(!tp.is_variable_mu());
        assert_eq!(tp.svar, 0.0);
        assert_eq!(tp.mu_b, 0.0);
        assert_eq!(tp.ex_b, 0.0);
    }

    #[test]
    fn variable_mu_pentode_roundtrip_serde() {
        let t = variable_mu_pentode_6k7();
        let json = serde_json::to_string(&t).expect("serialize");
        let back: TubeParams = serde_json::from_str(&json).expect("deserialize");
        assert!(back.is_variable_mu());
        assert_eq!(back.svar, 0.083);
        assert_eq!(back.mu_b, 3.4);
        assert_eq!(back.ex_b, 1.223);
        assert!(matches!(back.screen_form, ScreenForm::Rational));
    }

    #[test]
    fn classical_pentode_validates_with_zero_alpha_s() {
        // Classical Koren pentodes don't use αs/A/β — validate() must allow
        // them to be zero, unlike the Derk/DerkE paths.
        let t = classical_pentode_kt88();
        t.validate()
            .expect("KT88 Classical pentode should validate with αs=A=β=0");
        assert!(matches!(t.screen_form, ScreenForm::Classical));
        assert_eq!(t.alpha_s, 0.0);
        assert_eq!(t.a_factor, 0.0);
        assert_eq!(t.beta_factor, 0.0);
        assert!(t.is_pentode());
        assert!(!t.is_variable_mu());
    }

    #[test]
    fn classical_pentode_rejects_missing_kg2() {
        let mut bad = classical_pentode_kt88();
        bad.kg2 = 0.0;
        assert!(
            bad.validate().is_err(),
            "Classical pentode with KG2=0 must still fail validation"
        );
    }

    #[test]
    fn classical_pentode_rejects_variable_mu_combination() {
        // Variable-mu §5 two-section Koren is defined on top of the Derk
        // softplus structure, not the Classical arctan knee. The combination
        // is not implemented and validation should reject it.
        let mut bad = classical_pentode_kt88();
        bad.svar = 0.1;
        bad.mu_b = 3.0;
        bad.ex_b = 1.2;
        assert!(
            bad.validate().is_err(),
            "Variable-mu + Classical must be rejected"
        );
    }

    #[test]
    fn sharp_pentode_still_requires_alpha_s() {
        // Regression guard: the Classical relaxation must NOT weaken the
        // αs>0 requirement for Derk/DerkE pentodes (EL84, 6L6GC, etc).
        let mut bad = pentode_el84();
        bad.alpha_s = 0.0;
        assert!(
            bad.validate().is_err(),
            "Derk/Rational pentode with αs=0 must still fail validation"
        );

        let mut bad_bt = beam_tetrode_6l6gc();
        bad_bt.alpha_s = 0.0;
        assert!(
            bad_bt.validate().is_err(),
            "DerkE/Exponential beam tetrode with αs=0 must still fail validation"
        );
    }

    #[test]
    fn classical_pentode_roundtrip_serde() {
        let t = classical_pentode_kt88();
        let json = serde_json::to_string(&t).expect("serialize");
        let back: TubeParams = serde_json::from_str(&json).expect("deserialize");
        assert!(matches!(back.screen_form, ScreenForm::Classical));
        assert_eq!(back.mu, 8.8);
        assert_eq!(back.kg2, 4200.0);
        assert_eq!(back.alpha_s, 0.0);
    }

    #[test]
    fn screen_form_default_is_rational_after_classical_added() {
        // Regression: adding the Classical variant to the non_exhaustive enum
        // must not change the Default impl.
        let sf: ScreenForm = Default::default();
        assert!(matches!(sf, ScreenForm::Rational));
    }
}
