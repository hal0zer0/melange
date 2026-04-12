//! Tube/triode catalog entries with Koren model parameters.
//!
//! All parameters are for triode-mode operation (pentodes are triode-connected).
//! Sources are cited per entry for traceability.

use crate::tube::ScreenForm;

/// A catalog entry for a vacuum tube (triode-mode Koren parameters).
#[derive(Debug, Clone, Copy)]
pub struct TubeCatalogEntry {
    /// Part number aliases (e.g., ["12AX7", "ECC83", "7025", "CV4004"])
    pub names: &'static [&'static str],
    /// Amplification factor (mu)
    pub mu: f64,
    /// Exponent for Koren's equation
    pub ex: f64,
    /// Kg1 coefficient
    pub kg1: f64,
    /// Kp coefficient
    pub kp: f64,
    /// Kvb coefficient (knee shaping)
    pub kvb: f64,
    /// Maximum grid current [A]
    pub ig_max: f64,
    /// Grid current onset voltage [V]
    pub vgk_onset: f64,
    /// Channel-length modulation (Early effect) [1/V]
    pub lambda: f64,
    /// Source citation
    pub source: &'static str,
}

/// The tube catalog.
pub const CATALOG: &[TubeCatalogEntry] = &[
    // 12AX7 / ECC83 — high-mu twin triode, the workhorse of guitar amps.
    // Koren 1996 mu/ex/Kp/Kvb with Kg1=3000 (datasheet-accurate).
    // Ip ∝ 1/Kg1; Kg1=3000 gives ~1.2mA at Vgk=0, Vpk=250V (matches RCA datasheet).
    //   Default:  Vgk=0 → 1.20mA,  Vgk=-1 → 0.60mA,  Vgk=-2 → 0.17mA  (at Vpk=250V)
    //   Datasheet: Vgk=0 → ~1.2mA,  Vgk=-1 → ~0.5mA,  Vgk=-2 → ~0.1mA  (RCA 12AX7)
    TubeCatalogEntry {
        names: &["12AX7", "ECC83", "7025", "CV4004"],
        mu: 100.0,
        ex: 1.4,
        kg1: 3000.0,
        kp: 600.0,
        kvb: 300.0,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Koren 1996 with Kg1=3000 (datasheet-accurate)",
    },
    // 12AX7K — Koren 1996 original parameters (Kg1=1060).
    // Overestimates plate current vs RCA datasheet (~3.4mA vs ~1.2mA at Vgk=0, Vpk=250V).
    // Retained for backwards compatibility with circuits calibrated to Koren 1996.
    //   Koren:    Vgk=0 → 3.40mA,  Vgk=-1 → 1.69mA,  Vgk=-2 → 0.48mA  (at Vpk=250V)
    TubeCatalogEntry {
        names: &["12AX7K", "ECC83K"],
        mu: 100.0,
        ex: 1.4,
        kg1: 1060.0,
        kp: 600.0,
        kvb: 300.0,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Koren 1996 Table 1 (original Kg1=1060)",
    },
    // 12AX7F — 12AX7 fitted alias (same as default 12AX7 with Kg1=3000).
    // Kept for backwards compatibility with netlists using "12AX7F".
    TubeCatalogEntry {
        names: &["12AX7F", "ECC83F"],
        mu: 100.0,
        ex: 1.4,
        kg1: 3000.0,
        kp: 600.0,
        kvb: 300.0,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Koren 1996 with Kg1 re-fit to RCA 12AX7 datasheet (Ip=1.2mA at Vgk=0, Vpk=250V)",
    },
    // 12AU7 / ECC82 — medium-mu twin triode (mu≈17).
    // Koren fit to Sylvania data: mu=21.5, ex=1.3, Kg1=1180, Kp=84, Kvb=300
    // Note: published Koren fits vary; these values give best plate curve match
    // to Sylvania 12AU7 datasheet at typical audio operating points.
    TubeCatalogEntry {
        names: &["12AU7", "ECC82", "5963", "CV4003"],
        mu: 21.5,
        ex: 1.3,
        kg1: 1180.0,
        kp: 84.0,
        kvb: 300.0,
        ig_max: 4e-3,
        vgk_onset: 0.7,
        lambda: 0.0,
        source: "Koren fit to Sylvania 12AU7 datasheet",
    },
    // 12AT7 / ECC81 — medium-mu triode (mu≈60).
    // Koren fit: mu=60, ex=1.35, Kg1=460, Kp=300, Kvb=300
    TubeCatalogEntry {
        names: &["12AT7", "ECC81", "6201", "CV4024"],
        mu: 60.0,
        ex: 1.35,
        kg1: 460.0,
        kp: 300.0,
        kvb: 300.0,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Koren fit, verified against GE 12AT7 datasheet",
    },
    // 6SL7 / 6SL7GT — high-mu octal triode.
    // Koren fit: mu=70, ex=1.3, Kg1=1600, Kp=260, Kvb=300
    TubeCatalogEntry {
        names: &["6SL7", "6SL7GT"],
        mu: 70.0,
        ex: 1.3,
        kg1: 1600.0,
        kp: 260.0,
        kvb: 300.0,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Koren fit to RCA 6SL7GT datasheet",
    },
    // 6SN7 / 6SN7GT — medium-mu dual triode (mu=20), the "Fairchild tube".
    // Koren fit to Sylvania/RCA 6SN7GT datasheet curves.
    // Like all Koren single-mu fits, overestimates current at Vgk=0.
    //   Koren:    Vgk=0 → ~20mA,  Vgk=-4 → ~12mA,  Vgk=-8 → ~5mA  (at Vpk=250V)
    //   Datasheet: Vgk=0 → ~9mA,  Vgk=-4 → ~3mA,  Vgk=-8 → ~0.5mA (RCA 6SN7GT)
    TubeCatalogEntry {
        names: &["6SN7", "6SN7GT", "6SN7GTA", "6SN7GTB"],
        mu: 20.0,
        ex: 1.4,
        kg1: 1680.0,
        kp: 600.0,
        kvb: 300.0,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Koren fit to Sylvania/RCA 6SN7GT datasheet",
    },
    // 6J5 / 6J5GT — single triode version of 6SN7 (same electrical characteristics).
    // Identical Koren parameters; only physical packaging differs (single vs dual triode).
    TubeCatalogEntry {
        names: &["6J5", "6J5GT"],
        mu: 20.0,
        ex: 1.4,
        kg1: 1680.0,
        kp: 600.0,
        kvb: 300.0,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Same as 6SN7GT (single triode version)",
    },
    // 6V6 / 6V6GT — beam power tube (triode-connected).
    // Koren fit for triode mode: mu=8.7, ex=1.35, Kg1=1460, Kp=48, Kvb=12
    TubeCatalogEntry {
        names: &["6V6", "6V6GT"],
        mu: 8.7,
        ex: 1.35,
        kg1: 1460.0,
        kp: 48.0,
        kvb: 12.0,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        lambda: 0.0,
        source: "Published Koren triode-mode fit (Duncan Amps)",
    },
    // EL84 / 6BQ5 — small power pentode (triode-connected).
    // Koren fit for triode mode: mu=11.5, ex=1.35, Kg1=600, Kp=200, Kvb=300
    TubeCatalogEntry {
        names: &["EL84", "6BQ5"],
        mu: 11.5,
        ex: 1.35,
        kg1: 600.0,
        kp: 200.0,
        kvb: 300.0,
        ig_max: 8e-3,
        vgk_onset: 0.7,
        lambda: 0.0,
        source: "Duncan Amps Koren triode-mode fit",
    },
    // EL34 / 6CA7 — power pentode (triode-connected).
    // Koren fit for triode mode: mu=11, ex=1.35, Kg1=650, Kp=60, Kvb=24
    TubeCatalogEntry {
        names: &["EL34", "6CA7"],
        mu: 11.0,
        ex: 1.35,
        kg1: 650.0,
        kp: 60.0,
        kvb: 24.0,
        ig_max: 10e-3,
        vgk_onset: 0.7,
        lambda: 0.0,
        source: "Duncan Amps Koren triode-mode fit",
    },
    // 6L6 / 6L6GC / 5881 — beam power tube (triode-connected).
    // Koren fit for triode mode: mu=8.7, ex=1.35, Kg1=890, Kp=48, Kvb=12
    TubeCatalogEntry {
        names: &["6L6", "6L6GC", "5881"],
        mu: 8.7,
        ex: 1.35,
        kg1: 890.0,
        kp: 48.0,
        kvb: 12.0,
        ig_max: 8e-3,
        vgk_onset: 0.7,
        lambda: 0.0,
        source: "Published Koren triode-mode fit",
    },
];

/// Look up a tube by part number (case-insensitive).
pub fn lookup(name: &str) -> Option<&'static TubeCatalogEntry> {
    CATALOG
        .iter()
        .find(|entry| entry.names.iter().any(|n| n.eq_ignore_ascii_case(name)))
}

/// Catalog entry for a sharp-cutoff pentode using Reefman Derk §4.4 math.
///
/// Parameters are fitted by Derk Reefman (Reefman TubeLib.inc, 2016) using the
/// ExtractModel tool. The model uses αs/A/β extensions to Koren's pentode equation
/// to capture screen-grid current and the knee region.
///
/// **Naming convention**: True-pentode entries use a `-P` suffix (e.g. `EL84-P`,
/// `EL34-P`) to avoid colliding with the existing triode-connected catalog entries
/// (`EL84`, `EL34`). Small-signal pentodes with no triode-connected counterpart
/// (e.g. `EF86`) use the plain part number.
#[derive(Debug, Clone, Copy)]
pub struct PentodeCatalogEntry {
    /// Part number aliases (e.g., ["EL84-P", "EL84P", "6BQ5-P"])
    pub names: &'static [&'static str],
    /// Amplification factor (mu)
    pub mu: f64,
    /// Exponent for Reefman/Koren pentode equation
    pub ex: f64,
    /// Kg1 coefficient (plate current scaling)
    pub kg1: f64,
    /// Kg2 coefficient (screen current scaling)
    pub kg2: f64,
    /// Kp coefficient
    pub kp: f64,
    /// Kvb coefficient (knee shaping)
    pub kvb: f64,
    /// αs — Reefman §4.4 secondary-emission / knee-region coefficient
    pub alpha_s: f64,
    /// A — Reefman §4.4 raw A factor (not Aokg1; A = Aokg1 × kg1)
    pub a_factor: f64,
    /// β — Reefman §4.4 beta factor
    pub beta_factor: f64,
    /// Maximum grid current [A]
    pub ig_max: f64,
    /// Grid current onset voltage [V]
    pub vgk_onset: f64,
    /// Screen-current functional form:
    /// - [`ScreenForm::Rational`] — Reefman §4.4 (`1/(1+β·Vp)`), true pentodes.
    /// - [`ScreenForm::Exponential`] — Reefman §4.5 (`exp(-(β·Vp)^{3/2})`), beam tetrodes.
    pub screen_form: ScreenForm,
    /// Reefman §5 variable-mu section-B amplification factor (μ_b).
    /// `svar > 0` → two-section blend; 0.0 = sharp single-mu default.
    pub mu_b: f64,
    /// Reefman §5 variable-mu blend fraction (s_var in Eq 33).
    /// 0.0 = sharp default; typical fitted values 0.05–0.10.
    pub svar: f64,
    /// Reefman §5 variable-mu section-B Koren exponent (x_b in Eq 34).
    /// 0.0 = sharp default.
    pub ex_b: f64,
    /// Source citation
    pub source: &'static str,
}

/// The pentode / beam-tetrode catalog (Reefman Derk §4.4 and DerkE §4.5 fits).
pub const PENTODE_CATALOG: &[PentodeCatalogEntry] = &[
    // EL84 / 6BQ5 — small power pentode (true pentode mode).
    // Reefman TubeLib.inc BTetrodeD fit (§4.4).
    // Use "-P" suffix to disambiguate from the triode-connected EL84 in CATALOG.
    PentodeCatalogEntry {
        names: &["EL84-P", "EL84P", "6BQ5-P"],
        mu: 23.36,
        ex: 1.138,
        kg1: 117.4,
        kg2: 1275.0,
        kp: 152.4,
        kvb: 4015.8,
        alpha_s: 7.66,
        a_factor: 4.344e-4,
        beta_factor: 0.148,
        ig_max: 8e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // EL34 / 6CA7 — power pentode (true pentode mode).
    // Reefman TubeLib.inc BTetrodeD fit (§4.4).
    PentodeCatalogEntry {
        names: &["EL34-P", "EL34P", "6CA7-P"],
        mu: 12.50,
        ex: 1.363,
        kg1: 217.7,
        kg2: 1950.2,
        kp: 50.5,
        kvb: 1282.7,
        alpha_s: 6.09,
        a_factor: 3.48e-4,
        beta_factor: 0.105,
        ig_max: 10e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // EF86 / 6267 — small-signal sharp-cutoff pentode.
    // Reefman TubeLib.inc PenthodeD fit (§4.4).
    // No triode-connected counterpart in CATALOG, so plain "EF86" is unambiguous.
    PentodeCatalogEntry {
        names: &["EF86", "6267"],
        mu: 40.8,
        ex: 1.327,
        kg1: 675.8,
        kg2: 4089.6,
        kp: 350.7,
        kvb: 1886.8,
        alpha_s: 4.24,
        a_factor: 5.95e-5,
        beta_factor: 0.28,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), PenthodeD fit",
    },
    // 6L6GC / 5881 — beam power tube (beam-tetrode mode).
    // Reefman TubeLib.inc BTetrodeDE fit (§4.5 exponential screen form).
    // Uses a "-T" (for "tetrode") suffix on every alias to avoid colliding with
    // the triode-connected 6L6 / 6L6GC / 5881 entries in CATALOG.
    PentodeCatalogEntry {
        names: &["6L6-T", "6L6GC-T", "6L6GCT", "5881-T"],
        mu: 9.41,
        ex: 1.306,
        kg1: 446.6,
        kg2: 6672.5,
        kp: 45.2,
        kvb: 3205.1,
        alpha_s: 8.10,
        a_factor: 4.91e-4,
        beta_factor: 0.069,
        ig_max: 10e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeDE fit",
    },
    // 6V6 / 6V6GT — beam power tube (beam-tetrode mode).
    // Reefman TubeLib.inc BTetrodeDE fit (§4.5 exponential screen form).
    // "-T" suffix on every alias to avoid colliding with the triode-connected
    // 6V6 / 6V6GT entries in CATALOG.
    PentodeCatalogEntry {
        names: &["6V6-T", "6V6GT-T", "6V6GTT"],
        mu: 10.56,
        ex: 1.306,
        kg1: 609.8,
        kg2: 17267.3,
        kp: 47.9,
        kvb: 2171.5,
        alpha_s: 18.72,
        a_factor: 3.48e-4,
        beta_factor: 0.068,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeDE fit",
    },
    // 6K7 / 6K7G / 6K7GT — variable-mu (remote-cutoff) RF/IF pentode.
    // Reefman TubeLib.inc PenthodeVD fit (§5 variable-mu Derk, rational screen form).
    // The two-section blend (μ_a=15.5 sharp + μ_b=3.4 soft) captures the
    // exponentially-wound grid that gives remote-cutoff AGC behaviour.
    // No triode-connected counterpart in CATALOG, so bare "6K7" is unambiguous.
    PentodeCatalogEntry {
        names: &["6K7", "6K7G", "6K7GT"],
        mu: 15.5,
        ex: 1.573,
        kg1: 1407.7,
        kg2: 8335.8,
        kp: 36.0,
        kvb: 1309.0,
        alpha_s: 4.07,
        a_factor: 1.55e-9,
        beta_factor: 0.15,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Rational,
        mu_b: 3.4,
        svar: 0.083,
        ex_b: 1.223,
        source: "Reefman TubeLib.inc (2016), PenthodeVD fit (variable-mu Derk §5)",
    },
    // EF89 / 6DA6 — variable-mu (remote-cutoff) IF pentode.
    // Reefman TubeLib.inc PenthodeVDE fit (§5 variable-mu DerkE, exponential screen form).
    // Background-research Kvb=0 clamped to 1.0 for numerical safety in Koren knee.
    // a_factor not reported in source → defaulted to 0.0.
    PentodeCatalogEntry {
        names: &["EF89", "6DA6"],
        mu: 25.0,
        ex: 1.418,
        kg1: 328.3,
        kg2: 1199.3,
        kp: 58.8,
        kvb: 1.0,
        alpha_s: 2.07,
        a_factor: 0.0,
        beta_factor: 0.122,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Exponential,
        mu_b: 7.8,
        svar: 0.068,
        ex_b: 0.978,
        source: "Reefman TubeLib.inc (2016), PenthodeVDE fit (variable-mu DerkE §5)",
    },
    // --- Additional power pentodes / beam tetrodes (Reefman TubeLib.inc 2016) ---

    // EL41 — European small-power pentode (9W), used in Philips/Mullard Hi-Fi amps.
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["EL41"],
        mu: 25.06,
        ex: 1.264,
        kg1: 172.1,
        kg2: 1327.5,
        kp: 133.6,
        kvb: 1952.2,
        alpha_s: 4.52,
        a_factor: 5.507e-4,
        beta_factor: 0.193,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // EL86 / 6CW5 — European power pentode (11W), Grundig/Telefunken amps.
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["EL86", "6CW5"],
        mu: 7.99,
        ex: 1.723,
        kg1: 906.0,
        kg2: 31112.7,
        kp: 71.9,
        kvb: 6950.2,
        alpha_s: 27.20,
        a_factor: 3.624e-4,
        beta_factor: 0.291,
        ig_max: 8e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // 6AQ5 / 6005 — Small beam power tube (4.5W), budget guitar amps, Supro/Valco.
    // Reefman TubeLib.inc BTetrodeDE fit (§4.5 exponential screen).
    PentodeCatalogEntry {
        names: &["6AQ5", "6005"],
        mu: 10.41,
        ex: 1.292,
        kg1: 642.1,
        kg2: 8511.7,
        kp: 59.6,
        kvb: 597.2,
        alpha_s: 9.11,
        a_factor: 4.623e-4,
        beta_factor: 0.118,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeDE fit",
    },
    // 807 — Transmitter beam tetrode, used in some guitar amps (e.g. Hiwatt DR504).
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["807"],
        mu: 9.70,
        ex: 1.347,
        kg1: 480.7,
        kg2: 7551.7,
        kp: 42.4,
        kvb: 1308.7,
        alpha_s: 11.71,
        a_factor: 4.807e-4,
        beta_factor: 0.236,
        ig_max: 8e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // 6F6 / 6F6G — Early power pentode (3.2W), prewar amps, Zenith consoles.
    // Reefman TubeLib.inc PenthodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["6F6", "6F6G"],
        mu: 7.5,
        ex: 1.375,
        kg1: 1186.2,
        kg2: 8976.8,
        kp: 400.0,
        kvb: 3706.0,
        alpha_s: 3.56,
        a_factor: 1.423e-4,
        beta_factor: 0.09,
        ig_max: 4e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), PenthodeD fit",
    },
    // 6W6 / 6W6GT — Beam power tube (9W), console organs, jukebox amps.
    // Reefman TubeLib.inc BTetrodeDE fit (§4.5 exponential screen).
    PentodeCatalogEntry {
        names: &["6W6", "6W6GT"],
        mu: 8.53,
        ex: 1.212,
        kg1: 270.0,
        kg2: 3396.9,
        kp: 45.2,
        kvb: 2585.0,
        alpha_s: 8.75,
        a_factor: 7.020e-4,
        beta_factor: 0.134,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeDE fit",
    },
    // PL82 / 16A5 — European power pentode (5.5W), portable radios, record players.
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["PL82", "16A5"],
        mu: 11.24,
        ex: 1.262,
        kg1: 163.5,
        kg2: 908.1,
        kp: 77.3,
        kvb: 3419.7,
        alpha_s: 3.68,
        a_factor: 5.069e-4,
        beta_factor: 0.113,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // PL84 / 15CW5 — European power pentode (5.7W), early Vox amps, radiograms.
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["PL84", "15CW5"],
        mu: 8.37,
        ex: 1.191,
        kg1: 223.7,
        kg2: 4961.0,
        kp: 73.1,
        kvb: 1365.2,
        alpha_s: 17.60,
        a_factor: 2.908e-4,
        beta_factor: 0.228,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // PL83 / 15A6 — European output pentode (8W), European receivers.
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["PL83", "15A6"],
        mu: 26.58,
        ex: 1.390,
        kg1: 188.6,
        kg2: 1888.0,
        kp: 174.5,
        kvb: 511.1,
        alpha_s: 5.25,
        a_factor: 5.658e-5,
        beta_factor: 0.146,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // 6K6 / 6K6GT — Power pentode (3.4W), vintage amps, Hammond tonewheel organs.
    // Reefman TubeLib.inc PenthodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["6K6", "6K6GT"],
        mu: 10.4,
        ex: 1.333,
        kg1: 786.5,
        kg2: 5968.8,
        kp: 27.6,
        kvb: 4529.3,
        alpha_s: 3.81,
        a_factor: 1.573e-4,
        beta_factor: 0.13,
        ig_max: 4e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), PenthodeD fit",
    },

    // --- Additional small-signal pentodes ---

    // EF80 / 6BX6 — Sharp-cutoff small-signal pentode, used in mic preamps (Philips).
    // Reefman TubeLib.inc PenthodeDE fit (§4.5 exponential screen).
    PentodeCatalogEntry {
        names: &["EF80", "6BX6"],
        mu: 54.7,
        ex: 1.202,
        kg1: 136.4,
        kg2: 584.2,
        kp: 340.3,
        kvb: 4670.0,
        alpha_s: 3.09,
        a_factor: 3.137e-4,
        beta_factor: 0.054,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), PenthodeDE fit",
    },
    // EF91 / 6AM6 / CV138 — Sharp-cutoff pentode, used in guitar preamps (Watkins Copicat).
    // Reefman TubeLib.inc PenthodeDE fit (§4.5 exponential screen).
    PentodeCatalogEntry {
        names: &["EF91", "6AM6", "CV138"],
        mu: 72.4,
        ex: 1.288,
        kg1: 149.7,
        kg2: 641.0,
        kp: 262.4,
        kvb: 2760.2,
        alpha_s: 3.04,
        a_factor: 2.844e-4,
        beta_factor: 0.057,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), PenthodeDE fit",
    },
    // EF37 / EF37A — Small-signal pentode, Neumann CMV3 mic preamp, BBC equipment.
    // Reefman TubeLib.inc PenthodeD fit (§4.4 rational screen).
    // a_factor is essentially zero (8.37e-14) — similar pattern to 6K7 (1.55e-9).
    PentodeCatalogEntry {
        names: &["EF37", "EF37A"],
        mu: 29.5,
        ex: 1.266,
        kg1: 465.3,
        kg2: 2259.4,
        kp: 136.4,
        kvb: 1701.0,
        alpha_s: 3.85,
        a_factor: 8.375e-14,
        beta_factor: 0.15,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), PenthodeD fit",
    },
    // EF85 — Variable-mu (remote-cutoff) pentode, receiver IF/RF stages.
    // Reefman TubeLib.inc PenthodeVDE fit (§5 variable-mu DerkE, exponential screen).
    // Low Kvb (69.6) is valid — the source data shows kVB=69.6, not a clamped zero.
    PentodeCatalogEntry {
        names: &["EF85"],
        mu: 28.8,
        ex: 1.247,
        kg1: 204.8,
        kg2: 823.6,
        kp: 142.1,
        kvb: 69.6,
        alpha_s: 2.57,
        a_factor: 2.867e-4,
        beta_factor: 0.080,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Exponential,
        mu_b: 7.6,
        svar: 0.047,
        ex_b: 0.896,
        source: "Reefman TubeLib.inc (2016), PenthodeVDE fit (variable-mu DerkE §5)",
    },
    // 6GH8 / 6GH8A — Combination tube, pentode section. Used in FM tuners, instrument amps.
    // Reefman TubeLib.inc PenthodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["6GH8", "6GH8A"],
        mu: 56.2,
        ex: 1.236,
        kg1: 224.6,
        kg2: 1016.7,
        kp: 102.0,
        kvb: 2316.6,
        alpha_s: 3.91,
        a_factor: 2.920e-6,
        beta_factor: 0.13,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), PenthodeD fit",
    },

    // --- Combo tube pentode sections ---

    // ECL82 / 6BM8 pentode section — European combo tube (pentode + triode),
    // popular in Japanese amps (e.g. Sony, Sansui). Pentode Pd=9W.
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["ECL82", "6BM8"],
        mu: 11.49,
        ex: 1.264,
        kg1: 181.9,
        kg2: 991.9,
        kp: 53.9,
        kvb: 2066.3,
        alpha_s: 2.92,
        a_factor: 1.110e-3,
        beta_factor: 0.097,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // PCL86 / ECL86 / 6GW8 pentode section — European combo tube (pentode + triode),
    // Philips EL3549, European small-power stereo amps. Pentode Pd=9W.
    // ECL86 references PCL86 in TubeLib.inc (identical electrical characteristics).
    // Reefman TubeLib.inc BTetrodeDE fit (§4.5 exponential screen).
    PentodeCatalogEntry {
        names: &["PCL86", "ECL86", "6GW8"],
        mu: 22.90,
        ex: 1.271,
        kg1: 155.0,
        kg2: 905.2,
        kp: 150.7,
        kvb: 3788.2,
        alpha_s: 2.75,
        a_factor: 5.425e-4,
        beta_factor: 0.093,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeDE fit",
    },

    // --- Russian tubes (audio-relevant) ---

    // 4P1L — Russian directly-heated beam tetrode, popular in SE amp DIY community.
    // Reefman TubeLib.inc BTetrodeD fit (§4.4 rational screen).
    PentodeCatalogEntry {
        names: &["4P1L"],
        mu: 9.91,
        ex: 1.295,
        kg1: 300.4,
        kg2: 3317.6,
        kp: 71.1,
        kvb: 1414.3,
        alpha_s: 6.47,
        a_factor: 2.613e-4,
        beta_factor: 0.093,
        ig_max: 6e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Rational,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeD fit",
    },
    // 6E5P — Russian sharp-cutoff beam tetrode, popular in DIY headphone amps.
    // Reefman TubeLib.inc BTetrodeDE fit (§4.5 exponential screen).
    // Source kVB=0.0 clamped to 1.0 for numerical safety in Koren knee.
    PentodeCatalogEntry {
        names: &["6E5P"],
        mu: 31.61,
        ex: 1.909,
        kg1: 105.0,
        kg2: 669.2,
        kp: 1999.9,
        kvb: 1.0,
        alpha_s: 4.87,
        a_factor: 6.300e-4,
        beta_factor: 0.043,
        ig_max: 4e-3,
        vgk_onset: 0.5,
        screen_form: ScreenForm::Exponential,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Reefman TubeLib.inc (2016), BTetrodeDE fit",
    },

    // --- Classical Koren pentodes (Cohen-Hélie 2010 / Norman Koren 1996) ---

    // KT88 — Classical Koren beam power tetrode.
    // Cohen-Hélie 2010 DAFx Table 2 (originally Norman Koren 1996).
    // Uses ScreenForm::Classical — no αs / A / β extensions, no variable-mu blend.
    // Bare "KT88" name is unambiguous: no KT88 entry exists in the triode CATALOG,
    // so we skip the "-P" / "-T" suffix convention used by phase 1a / 1a.1 entries.
    PentodeCatalogEntry {
        names: &["KT88", "KT-88"],
        mu: 8.8,
        ex: 1.35,
        kg1: 730.0,
        kg2: 4200.0,
        kp: 32.0,
        kvb: 16.0,
        alpha_s: 0.0, // Classical Koren does not use αs
        a_factor: 0.0, // Classical does not use A
        beta_factor: 0.0, // Classical does not use β
        ig_max: 10e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Classical,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Cohen-Hélie 2010 DAFx Table 2 (originally Norman Koren 1996) — Classical Koren fit",
    },
    // 6550 — Classical Koren beam power tetrode, US military designation.
    // Cohen-Hélie 2010 DAFx Table 2 lists KT88 and 6550 with identical parameters
    // (they are essentially the same tube electrically — 6550 is the US military
    // designation and KT88 is the Genalex/Marconi civilian name). Kept as a
    // separate entry for clarity and to let netlists pick either name.
    // Bare "6550" name is unambiguous: no 6550 entry in the triode CATALOG.
    PentodeCatalogEntry {
        names: &["6550", "6550A", "6550C"],
        mu: 8.8,
        ex: 1.35,
        kg1: 730.0,
        kg2: 4200.0,
        kp: 32.0,
        kvb: 16.0,
        alpha_s: 0.0,
        a_factor: 0.0,
        beta_factor: 0.0,
        ig_max: 10e-3,
        vgk_onset: 0.7,
        screen_form: ScreenForm::Classical,
        mu_b: 0.0,
        svar: 0.0,
        ex_b: 0.0,
        source: "Cohen-Hélie 2010 DAFx Table 2 (originally Norman Koren 1996) — Classical Koren fit",
    },
];

/// Look up a pentode by part number (case-insensitive).
pub fn lookup_pentode(name: &str) -> Option<&'static PentodeCatalogEntry> {
    PENTODE_CATALOG
        .iter()
        .find(|entry| entry.names.iter().any(|n| n.eq_ignore_ascii_case(name)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::KorenTriode;

    #[test]
    fn test_all_aliases_resolve() {
        // Every name in the catalog should resolve
        for entry in CATALOG {
            for name in entry.names {
                assert!(lookup(name).is_some(), "Alias '{}' should resolve", name);
            }
        }
    }

    #[test]
    fn test_case_insensitive() {
        assert!(lookup("12ax7").is_some());
        assert!(lookup("12AX7").is_some());
        assert!(lookup("ecc83").is_some());
        assert!(lookup("ECC83").is_some());
        assert!(lookup("Ecc83").is_some());
    }

    #[test]
    fn test_no_duplicate_names() {
        let mut all_names: Vec<&str> = Vec::new();
        for entry in CATALOG {
            for name in entry.names {
                let lower = name.to_ascii_lowercase();
                assert!(
                    !all_names.iter().any(|n| n.eq_ignore_ascii_case(&lower)),
                    "Duplicate catalog name: {}",
                    name
                );
                all_names.push(name);
            }
        }
    }

    #[test]
    fn test_all_params_valid() {
        for entry in CATALOG {
            assert!(
                entry.mu > 0.0 && entry.mu.is_finite(),
                "{}: mu",
                entry.names[0]
            );
            assert!(
                entry.ex > 0.0 && entry.ex.is_finite(),
                "{}: ex",
                entry.names[0]
            );
            assert!(
                entry.kg1 > 0.0 && entry.kg1.is_finite(),
                "{}: kg1",
                entry.names[0]
            );
            assert!(
                entry.kp > 0.0 && entry.kp.is_finite(),
                "{}: kp",
                entry.names[0]
            );
            assert!(
                entry.kvb > 0.0 && entry.kvb.is_finite(),
                "{}: kvb",
                entry.names[0]
            );
            assert!(
                entry.ig_max > 0.0 && entry.ig_max.is_finite(),
                "{}: ig_max",
                entry.names[0]
            );
            assert!(
                entry.vgk_onset > 0.0 && entry.vgk_onset.is_finite(),
                "{}: vgk_onset",
                entry.names[0]
            );
            assert!(
                entry.lambda >= 0.0 && entry.lambda.is_finite(),
                "{}: lambda",
                entry.names[0]
            );
        }
    }

    #[test]
    fn test_unknown_returns_none() {
        assert!(lookup("NONEXISTENT_TUBE").is_none());
    }

    // --- Tier 1: Factory method consistency ---

    #[test]
    fn test_ecc83_matches_catalog() {
        // ecc83() factory now uses catalog "12AX7" (Kg1=3000, datasheet-accurate)
        let factory = KorenTriode::ecc83();
        let cat = lookup("12AX7").unwrap();
        assert_eq!(factory.mu, cat.mu);
        assert_eq!(factory.ex, cat.ex);
        assert_eq!(factory.kg1, cat.kg1);
        assert_eq!(factory.kp, cat.kp);
        assert_eq!(factory.kvb, cat.kvb);
    }

    #[test]
    fn test_ecc82_matches_catalog() {
        let factory = KorenTriode::ecc82();
        let cat = lookup("12AU7").unwrap();
        assert_eq!(factory.mu, cat.mu);
        assert_eq!(factory.ex, cat.ex);
        assert_eq!(factory.kg1, cat.kg1);
        assert_eq!(factory.kp, cat.kp);
        assert_eq!(factory.kvb, cat.kvb);
    }

    #[test]
    fn test_ecc81_matches_catalog() {
        let factory = KorenTriode::ecc81();
        let cat = lookup("12AT7").unwrap();
        assert_eq!(factory.mu, cat.mu);
        assert_eq!(factory.ex, cat.ex);
        assert_eq!(factory.kg1, cat.kg1);
        assert_eq!(factory.kp, cat.kp);
        assert_eq!(factory.kvb, cat.kvb);
    }

    #[test]
    fn test_6sl7_matches_catalog() {
        let factory = KorenTriode::_6sl7();
        let cat = lookup("6SL7").unwrap();
        assert_eq!(factory.mu, cat.mu);
        assert_eq!(factory.ex, cat.ex);
        assert_eq!(factory.kg1, cat.kg1);
        assert_eq!(factory.kp, cat.kp);
        assert_eq!(factory.kvb, cat.kvb);
    }

    #[test]
    fn test_12ax7f_matches_catalog() {
        let factory = KorenTriode::ecc83_fitted();
        let cat = lookup("12AX7F").unwrap();
        assert_eq!(factory.mu, cat.mu);
        assert_eq!(factory.ex, cat.ex);
        assert_eq!(factory.kg1, cat.kg1);
        assert_eq!(factory.kp, cat.kp);
        assert_eq!(factory.kvb, cat.kvb);
    }

    // --- Tier 2: Datasheet operating point verification ---

    fn make_tube(entry: &TubeCatalogEntry) -> KorenTriode {
        KorenTriode::with_all_params(
            entry.mu,
            entry.ex,
            entry.kg1,
            entry.kp,
            entry.kvb,
            entry.ig_max,
            entry.vgk_onset,
            entry.lambda,
        )
    }

    fn assert_ip_approx(actual: f64, expected: f64, tolerance: f64, label: &str) {
        assert!(
            (actual - expected).abs() < tolerance,
            "{}: Ip = {:.4}mA, expected {:.4}mA ± {:.4}mA",
            label,
            actual * 1e3,
            expected * 1e3,
            tolerance * 1e3,
        );
    }

    #[test]
    fn test_12ax7_operating_points() {
        let t = make_tube(lookup("12AX7").unwrap());
        // Kg1=3000 (datasheet-accurate): Vgk=0V, Vpk=250V → Ip ≈ 1.2mA
        let ip0 = t.plate_current(0.0, 250.0);
        assert_ip_approx(ip0, 1.2e-3, 0.2e-3, "12AX7 Vgk=0");
        // Vgk=-1V, Vpk=250V: ~0.6mA (datasheet: ~0.5mA)
        let ip_m1 = t.plate_current(-1.0, 250.0);
        assert_ip_approx(ip_m1, 0.5e-3, 0.15e-3, "12AX7 Vgk=-1");
        // Monotonicity: more negative grid → less current
        assert!(
            ip_m1 < ip0,
            "12AX7: Ip should decrease with more negative Vgk"
        );
        // Cutoff: Vgk=-4V → Ip ≈ 0
        assert!(
            t.plate_current(-4.0, 250.0) < 0.01e-3,
            "12AX7 should be near cutoff at Vgk=-4"
        );
    }

    #[test]
    fn test_12ax7f_operating_points() {
        let t = make_tube(lookup("12AX7F").unwrap());
        // Fitted to RCA 12AX7 datasheet: Vgk=0, Vpk=250V → ~1.2mA
        let ip0 = t.plate_current(0.0, 250.0);
        assert_ip_approx(ip0, 1.2e-3, 0.1e-3, "12AX7F Vgk=0");
        // Vgk=-1V, Vpk=250V: fitted → ~0.59mA (datasheet: ~0.5mA)
        let ip_m1 = t.plate_current(-1.0, 250.0);
        assert_ip_approx(ip_m1, 0.5e-3, 0.15e-3, "12AX7F Vgk=-1");
        // Vgk=-2V, Vpk=250V: fitted → ~0.17mA (datasheet: ~0.1mA)
        let ip_m2 = t.plate_current(-2.0, 250.0);
        assert_ip_approx(ip_m2, 0.1e-3, 0.1e-3, "12AX7F Vgk=-2");
        // Monotonicity
        assert!(
            ip_m1 < ip0,
            "12AX7F: Ip should decrease with more negative Vgk"
        );
        assert!(ip_m2 < ip_m1, "12AX7F: Ip should decrease further");
        // Cutoff: Vgk=-4V → Ip near zero
        assert!(
            t.plate_current(-4.0, 250.0) < 0.01e-3,
            "12AX7F should be near cutoff at Vgk=-4"
        );
    }

    #[test]
    fn test_12au7_operating_points() {
        let t = make_tube(lookup("12AU7").unwrap());
        // Koren model at Vgk=0V, Vpk=250V → Ip ≈ 20mA
        // (Sylvania datasheet: ~11.5mA; Koren single-mu overestimates at zero grid bias)
        let ip0 = t.plate_current(0.0, 250.0);
        assert!(
            ip0 > 5e-3 && ip0 < 40e-3,
            "12AU7 Vgk=0: Ip={:.1}mA",
            ip0 * 1e3
        );
        // Vgk=-8.5V, Vpk=250V: Koren gives ~5.2mA (datasheet: ~2.3mA)
        let ip_neg = t.plate_current(-8.5, 250.0);
        assert!(
            ip_neg > 0.5e-3 && ip_neg < 15e-3,
            "12AU7 Vgk=-8.5: Ip={:.1}mA",
            ip_neg * 1e3
        );
        assert!(
            ip_neg < ip0,
            "12AU7: current should decrease with negative grid"
        );
        // Deep cutoff: Vgk=-30V → Ip ≈ 0 (12AU7 has low mu, needs deeper cutoff)
        assert!(t.plate_current(-30.0, 250.0) < 0.01e-3, "12AU7 cutoff");
    }

    #[test]
    fn test_12at7_operating_points() {
        let t = make_tube(lookup("12AT7").unwrap());
        // GE datasheet: Vgk=0V, Vpk=250V → Ip ≈ 15mA
        assert_ip_approx(t.plate_current(0.0, 250.0), 15.0e-3, 7.0e-3, "12AT7 Vgk=0");
        // Vgk=-2V, Vpk=250V → Ip ≈ 5mA (typical op point)
        assert_ip_approx(t.plate_current(-2.0, 250.0), 5.0e-3, 3.0e-3, "12AT7 Vgk=-2");
    }

    #[test]
    fn test_6sl7_operating_points() {
        let t = make_tube(lookup("6SL7").unwrap());
        // RCA datasheet: Vgk=0V, Vpk=250V → Ip ≈ 2.3mA
        assert_ip_approx(t.plate_current(0.0, 250.0), 2.3e-3, 1.5e-3, "6SL7 Vgk=0");
        // Vgk=-2V, Vpk=250V → Ip ≈ 0.8mA
        assert_ip_approx(t.plate_current(-2.0, 250.0), 0.8e-3, 0.6e-3, "6SL7 Vgk=-2");
    }

    #[test]
    fn test_6sn7_lookup() {
        let entry = lookup("6SN7").unwrap();
        assert_eq!(entry.mu, 20.0);
        assert_eq!(entry.ex, 1.4);
        assert_eq!(entry.kg1, 1680.0);
        assert_eq!(entry.kp, 600.0);
        assert_eq!(entry.kvb, 300.0);
        // All aliases should resolve to the same entry
        assert!(lookup("6SN7GT").is_some());
        assert!(lookup("6SN7GTA").is_some());
        assert!(lookup("6SN7GTB").is_some());
        assert!(lookup("6sn7").is_some()); // case insensitive
    }

    #[test]
    fn test_6j5_matches_6sn7() {
        let sn7 = lookup("6SN7").unwrap();
        let j5 = lookup("6J5").unwrap();
        // 6J5 is the single triode version of 6SN7 — same electrical params
        assert_eq!(j5.mu, sn7.mu);
        assert_eq!(j5.ex, sn7.ex);
        assert_eq!(j5.kg1, sn7.kg1);
        assert_eq!(j5.kp, sn7.kp);
        assert_eq!(j5.kvb, sn7.kvb);
        assert_eq!(j5.ig_max, sn7.ig_max);
        assert_eq!(j5.vgk_onset, sn7.vgk_onset);
        assert_eq!(j5.lambda, sn7.lambda);
        // 6J5GT alias
        assert!(lookup("6J5GT").is_some());
    }

    #[test]
    fn test_6sn7_operating_points() {
        let t = make_tube(lookup("6SN7").unwrap());
        // RCA 6SN7GT datasheet: Vgk=0V, Vpk=250V → Ip ≈ 9mA
        // Koren single-mu model overestimates (~20mA), same pattern as 12AU7.
        // Accept wide range: model accuracy vs datasheet is known limitation.
        let ip0 = t.plate_current(0.0, 250.0);
        assert!(
            ip0 > 5e-3 && ip0 < 40e-3,
            "6SN7 Vgk=0: Ip={:.1}mA (datasheet ~9mA, Koren overestimates)",
            ip0 * 1e3
        );
        // RCA datasheet: Vgk=-8V, Vpk=250V → Ip ≈ 0.5mA (near cutoff)
        // Koren model gives ~5mA (overestimates near cutoff)
        let ip_m8 = t.plate_current(-8.0, 250.0);
        assert!(
            ip_m8 > 0.1e-3 && ip_m8 < 15e-3,
            "6SN7 Vgk=-8: Ip={:.1}mA (datasheet ~0.5mA)",
            ip_m8 * 1e3
        );
        // Monotonicity: more negative grid → less current
        assert!(
            ip_m8 < ip0,
            "6SN7: Ip should decrease with more negative Vgk"
        );
        // Deep cutoff: Vgk=-20V → Ip ≈ 0 (6SN7 has low mu, needs deeper cutoff like 12AU7)
        assert!(
            t.plate_current(-20.0, 250.0) < 0.1e-3,
            "6SN7 should be near cutoff at Vgk=-20"
        );
    }

    #[test]
    fn test_6v6_triode_operating_points() {
        let t = make_tube(lookup("6V6").unwrap());
        // 6V6 triode-connected: Vgk=0V, Vpk=250V → Ip ≈ 40-60mA
        let ip = t.plate_current(0.0, 250.0);
        assert!(ip > 15e-3 && ip < 100e-3, "6V6 Vgk=0: Ip={:.1}mA", ip * 1e3);
        // Vgk=-20V, Vpk=250V → should be reduced substantially
        let ip_neg = t.plate_current(-20.0, 250.0);
        assert!(ip_neg < ip, "6V6 negative grid should reduce current");
    }

    #[test]
    fn test_el84_triode_operating_points() {
        let t = make_tube(lookup("EL84").unwrap());
        // EL84 triode-connected: Vgk=0V, Vpk=250V → Ip in tens of mA
        let ip = t.plate_current(0.0, 250.0);
        assert!(ip > 5e-3 && ip < 150e-3, "EL84 Vgk=0: Ip={:.1}mA", ip * 1e3);
    }

    #[test]
    fn test_el34_triode_operating_points() {
        let t = make_tube(lookup("EL34").unwrap());
        // EL34 triode-connected: Vgk=0V, Vpk=250V → Ip in tens of mA
        let ip = t.plate_current(0.0, 250.0);
        assert!(
            ip > 10e-3 && ip < 200e-3,
            "EL34 Vgk=0: Ip={:.1}mA",
            ip * 1e3
        );
    }

    #[test]
    fn test_6l6_triode_operating_points() {
        let t = make_tube(lookup("6L6").unwrap());
        // 6L6 triode-connected: Vgk=0V, Vpk=250V → Ip in tens of mA
        let ip = t.plate_current(0.0, 250.0);
        assert!(ip > 15e-3 && ip < 150e-3, "6L6 Vgk=0: Ip={:.1}mA", ip * 1e3);
    }

    // --- Tier 3: Jacobian consistency for each tube entry ---

    fn check_jacobian(entry: &TubeCatalogEntry) {
        let t = make_tube(entry);
        let eps = 1e-6;
        for &vgk in &[-5.0, -2.0, -1.0, 0.0] {
            for &vpk in &[50.0, 150.0, 250.0] {
                let jac = t.jacobian(&[vgk, vpk]);
                let dip_dvgk = (t.plate_current(vgk + eps, vpk) - t.plate_current(vgk - eps, vpk))
                    / (2.0 * eps);
                let dip_dvpk = (t.plate_current(vgk, vpk + eps) - t.plate_current(vgk, vpk - eps))
                    / (2.0 * eps);

                if dip_dvgk.abs() > 1e-12 {
                    let rel = (jac[0] - dip_dvgk).abs() / dip_dvgk.abs();
                    assert!(
                        rel < 1e-3,
                        "{} dIp/dVgk at ({},{}): analytic={:.6e} fd={:.6e} rel={:.2e}",
                        entry.names[0],
                        vgk,
                        vpk,
                        jac[0],
                        dip_dvgk,
                        rel
                    );
                }
                if dip_dvpk.abs() > 1e-12 {
                    let rel = (jac[1] - dip_dvpk).abs() / dip_dvpk.abs();
                    assert!(
                        rel < 1e-3,
                        "{} dIp/dVpk at ({},{}): analytic={:.6e} fd={:.6e} rel={:.2e}",
                        entry.names[0],
                        vgk,
                        vpk,
                        jac[1],
                        dip_dvpk,
                        rel
                    );
                }
            }
        }
    }

    // Use NonlinearDevice trait for jacobian
    use crate::NonlinearDevice;

    #[test]
    fn test_12ax7_jacobian() {
        check_jacobian(lookup("12AX7").unwrap());
    }
    #[test]
    fn test_12ax7f_jacobian() {
        check_jacobian(lookup("12AX7F").unwrap());
    }
    #[test]
    fn test_12au7_jacobian() {
        check_jacobian(lookup("12AU7").unwrap());
    }
    #[test]
    fn test_12at7_jacobian() {
        check_jacobian(lookup("12AT7").unwrap());
    }
    #[test]
    fn test_6sl7_jacobian() {
        check_jacobian(lookup("6SL7").unwrap());
    }
    #[test]
    fn test_6sn7_jacobian() {
        check_jacobian(lookup("6SN7").unwrap());
    }
    #[test]
    fn test_6j5_jacobian() {
        check_jacobian(lookup("6J5").unwrap());
    }
    #[test]
    fn test_6v6_jacobian() {
        check_jacobian(lookup("6V6").unwrap());
    }
    #[test]
    fn test_el84_jacobian() {
        check_jacobian(lookup("EL84").unwrap());
    }
    #[test]
    fn test_el34_jacobian() {
        check_jacobian(lookup("EL34").unwrap());
    }
    #[test]
    fn test_6l6_jacobian() {
        check_jacobian(lookup("6L6").unwrap());
    }

    // --- Tier 5: Known discrepancies ---

    #[test]
    fn test_12au7_mu_discrepancy() {
        // The Koren model uses a single mu parameter but real 12AU7 mu varies
        // from ~17 at low Ip to ~20 at high Ip. Our Koren fit uses mu=21.5 for
        // best overall plate curve match.
        let t = make_tube(lookup("12AU7").unwrap());
        // Approximate mu at Vpk=250V by dIp/dVgk * rp
        let vgk = -8.5;
        let vpk = 250.0;
        let eps = 0.01;
        let gm = (t.plate_current(vgk + eps, vpk) - t.plate_current(vgk - eps, vpk)) / (2.0 * eps);
        let gp = (t.plate_current(vgk, vpk + eps) - t.plate_current(vgk, vpk - eps)) / (2.0 * eps);
        let measured_mu = gm / gp;
        // Accept within range [14, 25] — datasheet says 17, Koren fit gives higher
        assert!(
            measured_mu > 14.0 && measured_mu < 25.0,
            "12AU7 mu={:.1} (datasheet: 17, Koren model range: 14-25)",
            measured_mu
        );
    }

    // --- Pentode catalog tests (Reefman Derk §4.4 fits) ---

    #[test]
    fn test_pentode_catalog_el84_lookup() {
        let entry = lookup_pentode("EL84-P").expect("EL84-P should resolve");
        assert_eq!(entry.mu, 23.36);
        assert_eq!(entry.kg2, 1275.0);
        assert_eq!(entry.alpha_s, 7.66);
    }

    #[test]
    fn test_pentode_catalog_case_insensitive() {
        let lower = lookup_pentode("el84-p").expect("lowercase should resolve");
        let upper = lookup_pentode("EL84-P").expect("uppercase should resolve");
        // Same entry → identical fields
        assert_eq!(lower.mu, upper.mu);
        assert_eq!(lower.ex, upper.ex);
        assert_eq!(lower.kg1, upper.kg1);
        assert_eq!(lower.kg2, upper.kg2);
        assert_eq!(lower.kp, upper.kp);
        assert_eq!(lower.kvb, upper.kvb);
        assert_eq!(lower.alpha_s, upper.alpha_s);
        assert_eq!(lower.a_factor, upper.a_factor);
        assert_eq!(lower.beta_factor, upper.beta_factor);
        assert_eq!(lower.names.as_ptr(), upper.names.as_ptr());
    }

    #[test]
    fn test_pentode_catalog_ef86_no_suffix() {
        let ef86 = lookup_pentode("EF86").expect("EF86 should resolve");
        let alias = lookup_pentode("6267").expect("6267 alias should resolve");
        // Same entry → identical fields and same names slice pointer
        assert_eq!(ef86.mu, alias.mu);
        assert_eq!(ef86.kg1, alias.kg1);
        assert_eq!(ef86.kg2, alias.kg2);
        assert_eq!(ef86.alpha_s, alias.alpha_s);
        assert_eq!(ef86.names.as_ptr(), alias.names.as_ptr());
    }

    #[test]
    fn test_pentode_catalog_no_triode_collision() {
        // No pentode alias should collide with a triode catalog name.
        for pentode in PENTODE_CATALOG {
            for pname in pentode.names {
                assert!(
                    lookup(pname).is_none(),
                    "Pentode alias '{}' must not resolve in the triode CATALOG",
                    pname
                );
            }
        }
        // The original triode-connected EL84/EL34 entries still resolve via lookup().
        let triode_el84 = lookup("EL84").expect("triode EL84 still in CATALOG");
        assert_eq!(triode_el84.mu, 11.5);
        let triode_el34 = lookup("EL34").expect("triode EL34 still in CATALOG");
        assert_eq!(triode_el34.mu, 11.0);
        // And lookup_pentode() returns None for the bare names (must use -P suffix).
        assert!(lookup_pentode("EL84").is_none());
        assert!(lookup_pentode("EL34").is_none());
    }

    #[test]
    fn test_pentode_catalog_derk_params_positive() {
        for entry in PENTODE_CATALOG {
            assert!(entry.mu > 0.0, "{}: mu", entry.names[0]);
            assert!(entry.kg1 > 0.0, "{}: kg1", entry.names[0]);
            assert!(entry.kg2 > 0.0, "{}: kg2", entry.names[0]);
            // Classical Koren pentodes (KT88, 6550) don't use αs / A / β at all
            // — those are Reefman Derk §4.4 / DerkE §4.5 extensions. Only gate
            // the αs strict-positive check on the non-Classical screen forms.
            if !matches!(entry.screen_form, ScreenForm::Classical) {
                assert!(entry.alpha_s > 0.0, "{}: alpha_s", entry.names[0]);
            }
            // Sanity-check the rest while we're here
            assert!(entry.ex > 0.0, "{}: ex", entry.names[0]);
            assert!(entry.kp > 0.0, "{}: kp", entry.names[0]);
            assert!(entry.kvb > 0.0, "{}: kvb", entry.names[0]);
            // a_factor and beta_factor are allowed to be exactly 0 — some
            // Reefman fits pinpoint them at zero (e.g. EF89 has A=0), and
            // Classical Koren entries have them all at zero by construction.
            // Only require non-negative.
            assert!(entry.a_factor >= 0.0, "{}: a_factor", entry.names[0]);
            assert!(entry.beta_factor >= 0.0, "{}: beta_factor", entry.names[0]);
            assert!(entry.ig_max > 0.0, "{}: ig_max", entry.names[0]);
            assert!(entry.vgk_onset > 0.0, "{}: vgk_onset", entry.names[0]);
        }
    }

    #[test]
    fn test_triode_catalog_unchanged() {
        // Byte-identity guard: existing triode-connected EL84/EL34/6L6/6V6 entries
        // must not have been modified by the pentode addition.
        let el84 = lookup("EL84").expect("triode EL84");
        assert_eq!(el84.mu, 11.5);
        assert_eq!(el84.ex, 1.35);
        assert_eq!(el84.kg1, 600.0);
        assert_eq!(el84.kp, 200.0);
        assert_eq!(el84.kvb, 300.0);

        let el34 = lookup("EL34").expect("triode EL34");
        assert_eq!(el34.mu, 11.0);
        assert_eq!(el34.ex, 1.35);
        assert_eq!(el34.kg1, 650.0);
        assert_eq!(el34.kp, 60.0);
        assert_eq!(el34.kvb, 24.0);

        let six_l6 = lookup("6L6").expect("triode 6L6");
        assert_eq!(six_l6.mu, 8.7);
        assert_eq!(six_l6.kg1, 890.0);
        assert!(lookup("6L6GC").is_some());
        assert!(lookup("5881").is_some());

        let six_v6 = lookup("6V6").expect("triode 6V6");
        assert_eq!(six_v6.mu, 8.7);
        assert_eq!(six_v6.kg1, 1460.0);
        assert!(lookup("6V6GT").is_some());
    }

    // --- Beam tetrode catalog tests (Reefman DerkE §4.5 fits) ---

    #[test]
    fn test_pentode_catalog_6l6gc_lookup() {
        let entry = lookup_pentode("6L6-T").expect("6L6-T should resolve");
        assert_eq!(entry.mu, 9.41);
        assert_eq!(entry.kg2, 6672.5);
        assert_eq!(entry.alpha_s, 8.10);
        assert_eq!(entry.screen_form, ScreenForm::Exponential);

        // The 6L6GC-T alias must resolve to the same entry.
        let alias = lookup_pentode("6L6GC-T").expect("6L6GC-T alias");
        assert_eq!(alias.mu, 9.41);
        assert_eq!(alias.kg2, 6672.5);
        assert_eq!(alias.alpha_s, 8.10);
        assert_eq!(alias.screen_form, ScreenForm::Exponential);
        assert_eq!(entry.names.as_ptr(), alias.names.as_ptr());
    }

    #[test]
    fn test_pentode_catalog_6v6gt_lookup() {
        let entry = lookup_pentode("6V6-T").expect("6V6-T should resolve");
        assert_eq!(entry.mu, 10.56);
        assert_eq!(entry.kg2, 17267.3);
        assert_eq!(entry.alpha_s, 18.72);
        assert_eq!(entry.screen_form, ScreenForm::Exponential);
    }

    #[test]
    fn test_pentode_catalog_el84_still_rational() {
        // Phase 1a EL84-P entry must remain the Rational §4.4 fit.
        let el84p = lookup_pentode("EL84-P").expect("EL84-P should resolve");
        assert_eq!(el84p.screen_form, ScreenForm::Rational);
        // And so should the other phase-1a pentodes.
        let el34p = lookup_pentode("EL34-P").expect("EL34-P should resolve");
        assert_eq!(el34p.screen_form, ScreenForm::Rational);
        let ef86 = lookup_pentode("EF86").expect("EF86 should resolve");
        assert_eq!(ef86.screen_form, ScreenForm::Rational);
    }

    #[test]
    fn test_pentode_catalog_case_insensitive_beam_tetrode() {
        let lower = lookup_pentode("6l6-t").expect("lowercase 6l6-t");
        let upper = lookup_pentode("6L6-T").expect("uppercase 6L6-T");
        assert_eq!(lower.mu, upper.mu);
        assert_eq!(lower.kg2, upper.kg2);
        assert_eq!(lower.alpha_s, upper.alpha_s);
        assert_eq!(lower.screen_form, upper.screen_form);
        // Exact same &'static slice → same entry.
        assert_eq!(lower.names.as_ptr(), upper.names.as_ptr());
    }

    #[test]
    fn test_triode_catalog_6l6_unchanged() {
        // Byte-identity guard: phase-1a triode-connected 6L6 entry unmodified.
        let six_l6 = lookup("6L6").expect("triode 6L6");
        assert_eq!(six_l6.mu, 8.7);
        assert_eq!(six_l6.kg1, 890.0);
        // Aliases still resolve through CATALOG (not PENTODE_CATALOG).
        assert!(lookup("6L6GC").is_some());
        assert!(lookup("5881").is_some());
        // And lookup_pentode() must NOT resolve the bare triode name.
        assert!(lookup_pentode("6L6").is_none());
        assert!(lookup_pentode("6L6GC").is_none());
        assert!(lookup_pentode("5881").is_none());
    }

    #[test]
    fn test_triode_catalog_6v6_unchanged() {
        // Byte-identity guard: phase-1a triode-connected 6V6 entry unmodified.
        let six_v6 = lookup("6V6").expect("triode 6V6");
        assert_eq!(six_v6.mu, 8.7);
        assert_eq!(six_v6.kg1, 1460.0);
        assert!(lookup("6V6GT").is_some());
        // lookup_pentode() must NOT resolve the bare triode name.
        assert!(lookup_pentode("6V6").is_none());
        assert!(lookup_pentode("6V6GT").is_none());
    }

    #[test]
    fn test_pentode_catalog_beam_tetrode_distinct_from_pentode() {
        let six_l6_t = lookup_pentode("6L6-T").expect("6L6-T should resolve");
        let el84p = lookup_pentode("EL84-P").expect("EL84-P should resolve");
        assert_eq!(six_l6_t.screen_form, ScreenForm::Exponential);
        assert_eq!(el84p.screen_form, ScreenForm::Rational);
        assert_ne!(six_l6_t.screen_form, el84p.screen_form);
    }

    // --- Classical Koren pentode catalog tests (phase 1a.2: KT88, 6550) ---

    #[test]
    fn test_pentode_catalog_kt88_lookup() {
        let entry = lookup_pentode("KT88").expect("KT88 should resolve");
        assert_eq!(entry.mu, 8.8);
        assert_eq!(entry.kg1, 730.0);
        assert_eq!(entry.kg2, 4200.0);
        assert_eq!(entry.kp, 32.0);
        assert_eq!(entry.kvb, 16.0);
        assert_eq!(entry.ex, 1.35);
        assert_eq!(entry.alpha_s, 0.0);
        assert_eq!(entry.screen_form, ScreenForm::Classical);
    }

    #[test]
    fn test_pentode_catalog_6550_lookup() {
        let entry = lookup_pentode("6550").expect("6550 should resolve");
        // 6550 and KT88 are the same Cohen-Hélie 2010 fit — identical params.
        let kt88 = lookup_pentode("KT88").expect("KT88 should resolve");
        assert_eq!(entry.mu, kt88.mu);
        assert_eq!(entry.ex, kt88.ex);
        assert_eq!(entry.kg1, kt88.kg1);
        assert_eq!(entry.kg2, kt88.kg2);
        assert_eq!(entry.kp, kt88.kp);
        assert_eq!(entry.kvb, kt88.kvb);
        assert_eq!(entry.alpha_s, kt88.alpha_s);
        assert_eq!(entry.a_factor, kt88.a_factor);
        assert_eq!(entry.beta_factor, kt88.beta_factor);
        assert_eq!(entry.ig_max, kt88.ig_max);
        assert_eq!(entry.vgk_onset, kt88.vgk_onset);
        assert_eq!(entry.screen_form, kt88.screen_form);
        assert_eq!(entry.mu_b, kt88.mu_b);
        assert_eq!(entry.svar, kt88.svar);
        assert_eq!(entry.ex_b, kt88.ex_b);
    }

    #[test]
    fn test_pentode_catalog_kt88_alias_kt_dash_88() {
        let kt88 = lookup_pentode("KT88").expect("KT88 should resolve");
        let aliased = lookup_pentode("KT-88").expect("KT-88 alias should resolve");
        // Same &'static slice → guaranteed same entry.
        assert_eq!(kt88.names.as_ptr(), aliased.names.as_ptr());
        assert_eq!(kt88.mu, aliased.mu);
        assert_eq!(kt88.kg1, aliased.kg1);
        assert_eq!(kt88.screen_form, aliased.screen_form);
    }

    #[test]
    fn test_pentode_catalog_6550_aliases() {
        let base = lookup_pentode("6550").expect("6550 should resolve");
        let a = lookup_pentode("6550A").expect("6550A alias should resolve");
        let c = lookup_pentode("6550C").expect("6550C alias should resolve");
        // All three aliases resolve to the same &'static slice.
        assert_eq!(base.names.as_ptr(), a.names.as_ptr());
        assert_eq!(base.names.as_ptr(), c.names.as_ptr());
        assert_eq!(base.mu, a.mu);
        assert_eq!(base.mu, c.mu);
        assert_eq!(base.kg1, a.kg1);
        assert_eq!(base.kg1, c.kg1);
        assert_eq!(base.screen_form, ScreenForm::Classical);
        assert_eq!(a.screen_form, ScreenForm::Classical);
        assert_eq!(c.screen_form, ScreenForm::Classical);
    }

    #[test]
    fn test_pentode_catalog_kt88_case_insensitive() {
        let lower = lookup_pentode("kt88").expect("lowercase kt88 should resolve");
        let upper = lookup_pentode("KT88").expect("uppercase KT88 should resolve");
        assert_eq!(lower.names.as_ptr(), upper.names.as_ptr());
        assert_eq!(lower.mu, upper.mu);
        assert_eq!(lower.kg1, upper.kg1);
        assert_eq!(lower.screen_form, upper.screen_form);
    }

    #[test]
    fn test_pentode_catalog_kt88_no_triode_collision() {
        // No pre-existing KT88 / 6550 entry should live in the triode CATALOG.
        // Unlike -P / -T suffixed phase-1a/1a.1 entries (which disambiguate from
        // triode-connected versions), KT88 and 6550 use bare names because there
        // is no collision to avoid.
        assert!(
            lookup("KT88").is_none(),
            "KT88 must not exist in the triode CATALOG"
        );
        assert!(
            lookup("KT-88").is_none(),
            "KT-88 must not exist in the triode CATALOG"
        );
        assert!(
            lookup("6550").is_none(),
            "6550 must not exist in the triode CATALOG"
        );
        assert!(
            lookup("6550A").is_none(),
            "6550A must not exist in the triode CATALOG"
        );
        assert!(
            lookup("6550C").is_none(),
            "6550C must not exist in the triode CATALOG"
        );
    }

    #[test]
    fn test_pentode_catalog_classical_entries_have_zero_derk_params() {
        // Classical Koren (Cohen-Hélie 2010) does not use any of the Reefman
        // Derk §4.4 / DerkE §4.5 knee extensions or the §5 variable-mu blend.
        // All of αs / A / β / svar / μ_b / ex_b must be exactly zero.
        for name in &["KT88", "6550"] {
            let entry = lookup_pentode(name)
                .unwrap_or_else(|| panic!("{} should resolve", name));
            assert_eq!(entry.screen_form, ScreenForm::Classical, "{}: screen_form", name);
            assert_eq!(entry.alpha_s, 0.0, "{}: alpha_s", name);
            assert_eq!(entry.a_factor, 0.0, "{}: a_factor", name);
            assert_eq!(entry.beta_factor, 0.0, "{}: beta_factor", name);
            assert_eq!(entry.svar, 0.0, "{}: svar", name);
            assert_eq!(entry.mu_b, 0.0, "{}: mu_b", name);
            assert_eq!(entry.ex_b, 0.0, "{}: ex_b", name);
        }
    }

    #[test]
    fn test_pentode_catalog_existing_entries_unchanged() {
        // Byte-identity guard: adding KT88 / 6550 must not disturb any existing
        // phase-1a (EL84-P, EL34-P, EF86), phase-1a.1 (6L6-T, 6V6GT-T), or
        // phase-1c (6K7, EF89) entry.
        let el84p = lookup_pentode("EL84-P").expect("EL84-P should resolve");
        assert_eq!(el84p.mu, 23.36);
        assert_eq!(el84p.ex, 1.138);
        assert_eq!(el84p.kg1, 117.4);
        assert_eq!(el84p.kg2, 1275.0);
        assert_eq!(el84p.alpha_s, 7.66);
        assert_eq!(el84p.screen_form, ScreenForm::Rational);

        let six_l6_t = lookup_pentode("6L6-T").expect("6L6-T should resolve");
        assert_eq!(six_l6_t.mu, 9.41);
        assert_eq!(six_l6_t.kg1, 446.6);
        assert_eq!(six_l6_t.kg2, 6672.5);
        assert_eq!(six_l6_t.alpha_s, 8.10);
        assert_eq!(six_l6_t.screen_form, ScreenForm::Exponential);

        let six_k7 = lookup_pentode("6K7").expect("6K7 should resolve");
        assert_eq!(six_k7.mu, 15.5);
        assert_eq!(six_k7.kg1, 1407.7);
        assert_eq!(six_k7.kg2, 8335.8);
        assert_eq!(six_k7.alpha_s, 4.07);
        assert_eq!(six_k7.mu_b, 3.4);
        assert_eq!(six_k7.svar, 0.083);
        assert_eq!(six_k7.ex_b, 1.223);
        assert_eq!(six_k7.screen_form, ScreenForm::Rational);

        let ef89 = lookup_pentode("EF89").expect("EF89 should resolve");
        assert_eq!(ef89.mu, 25.0);
        assert_eq!(ef89.ex, 1.418);
        assert_eq!(ef89.kg1, 328.3);
        assert_eq!(ef89.kg2, 1199.3);
        assert_eq!(ef89.kvb, 1.0);
        assert_eq!(ef89.alpha_s, 2.07);
        assert_eq!(ef89.a_factor, 0.0);
        assert_eq!(ef89.mu_b, 7.8);
        assert_eq!(ef89.svar, 0.068);
        assert_eq!(ef89.ex_b, 0.978);
        assert_eq!(ef89.screen_form, ScreenForm::Exponential);
    }
}
