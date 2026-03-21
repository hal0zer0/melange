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
    pub rs_param: f64,
}

impl JfetParams {
    /// Returns true if either drain or source resistance is enabled.
    pub fn has_rd_rs(&self) -> bool {
        self.rd > 0.0 || self.rs_param > 0.0
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
    pub rs_param: f64,
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
        self.rd > 0.0 || self.rs_param > 0.0
    }
    /// Returns true if body effect is enabled.
    pub fn has_body_effect(&self) -> bool {
        self.gamma > 0.0
    }
}

/// Tube/triode model parameters (Koren + improved grid current).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TubeParams {
    /// Amplification factor (mu)
    pub mu: f64,
    /// Exponent for Koren's equation
    pub ex: f64,
    /// Kg1 coefficient
    pub kg1: f64,
    /// Kp coefficient
    pub kp: f64,
    /// Kvb coefficient (for knee shaping)
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
}

impl TubeParams {
    /// Returns true if grid internal resistance is enabled.
    pub fn has_rgi(&self) -> bool {
        self.rgi > 0.0
    }
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
