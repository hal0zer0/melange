//! Vacuum tube (valve) models.
//!
//! Koren's models for triodes and pentodes.

use crate::NonlinearDevice;

/// Koren triode model with improved grid current.
///
/// Based on Norman Koren's improved vacuum tube models.
/// Parameters from SPICE model cards.
///
/// Grid current uses a smooth power-law model (Leach-style):
///   Ig = ig_max * max(0, Vgk/vgk_onset)^1.5
/// which is physically motivated and has a well-defined analytical Jacobian.
///
/// Optional extension:
/// - **Channel-length modulation (`lambda`)**: `Ip_final = Ip_koren * (1 + lambda * Vpk)`.
///   Gives plate resistance rp = 1/(lambda * Ip) at the operating point.
///   Default `0.0` (no correction, backward compatible).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KorenTriode {
    /// Amplification factor (mu). When `svar > 0` this is `μ_a`, the
    /// high-mu section of a Reefman §5 variable-mu triode.
    pub mu: f64,
    /// Exponent for Koren's equation. When `svar > 0` this is `x_a`, the
    /// exponent of the high-mu section.
    pub ex: f64,
    /// Kg1 coefficient
    pub kg1: f64,
    /// Kp coefficient
    pub kp: f64,
    /// Kvb coefficient (for knee shaping)
    pub kvb: f64,
    /// Maximum grid current [A] at Vgk = vgk_onset
    pub ig_max: f64,
    /// Grid current onset voltage [V]
    pub vgk_onset: f64,
    /// Channel-length modulation coefficient [1/V]. 0.0 = disabled (default).
    /// Ip_final = Ip_koren * (1 + lambda * Vpk).
    /// Gives plate resistance rp approximately 1/(lambda * Ip) at the operating point.
    pub lambda: f64,
    /// Reefman §5 variable-mu section-B amplification factor (μ_b).
    /// `svar > 0` → two-section blend; `svar == 0` → ignored (sharp single-mu).
    /// 0.0 = sharp default.
    pub mu_b: f64,
    /// Reefman §5 variable-mu blend fraction (s_var in Eq 33).
    /// `Ip0_v = (1 − s_var)·Ip0_a + s_var·Ip0_b`. 0.0 = sharp default.
    pub svar: f64,
    /// Reefman §5 variable-mu section-B Koren exponent (x_b in Eq 34).
    /// Used only when `svar > 0`. 0.0 = sharp default.
    pub ex_b: f64,
}

/// Default grid current maximum [A].
const DEFAULT_IG_MAX: f64 = 2e-3;
/// Default grid current onset voltage [V].
const DEFAULT_VGK_ONSET: f64 = 0.5;

impl KorenTriode {
    /// Create a new triode model with default grid current parameters.
    /// Produces a sharp-cutoff triode (`svar = 0`); variable-mu presets
    /// must use a custom constructor that sets `mu_b`/`svar`/`ex_b`.
    pub fn new(mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64) -> Self {
        Self {
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max: DEFAULT_IG_MAX,
            vgk_onset: DEFAULT_VGK_ONSET,
            lambda: 0.0,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    /// Create a new triode model with custom grid current parameters.
    pub fn with_grid_params(
        mu: f64,
        ex: f64,
        kg1: f64,
        kp: f64,
        kvb: f64,
        ig_max: f64,
        vgk_onset: f64,
    ) -> Self {
        Self {
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            lambda: 0.0,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    /// Create a new triode model with all parameters including lambda (Early effect).
    #[allow(clippy::too_many_arguments)]
    pub fn with_all_params(
        mu: f64,
        ex: f64,
        kg1: f64,
        kp: f64,
        kvb: f64,
        ig_max: f64,
        vgk_onset: f64,
        lambda: f64,
    ) -> Self {
        Self {
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            lambda,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    /// 12AX7 (ECC83) - high-mu twin triode, common in guitar amps.
    ///
    /// Uses original Koren 1996 parameters (Kg1=1060). Overestimates plate current
    /// vs datasheet (~3.4mA vs ~1.2mA at Vgk=0, Vpk=250V). For datasheet-accurate
    /// plate current, use [`ecc83_fitted`](Self::ecc83_fitted).
    pub fn ecc83() -> Self {
        let c = crate::catalog::tubes::lookup("12AX7").expect("12AX7 catalog entry");
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 12AX7 (ECC83) fitted to RCA datasheet - high-mu twin triode.
    ///
    /// Kg1 re-fit from 1060 to 3000 to match RCA 12AX7 datasheet:
    /// ~1.2mA at Vgk=0, Vpk=250V (vs ~3.4mA with original Koren params).
    /// All other parameters identical to [`ecc83`](Self::ecc83).
    pub fn ecc83_fitted() -> Self {
        let c = crate::catalog::tubes::lookup("12AX7F").expect("12AX7F catalog entry");
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 12AU7 (ECC82) - medium-mu twin triode.
    pub fn ecc82() -> Self {
        let c = crate::catalog::tubes::lookup("12AU7").expect("12AU7 catalog entry");
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 12AT7 (ECC81) - medium-mu triode.
    pub fn ecc81() -> Self {
        let c = crate::catalog::tubes::lookup("12AT7").expect("12AT7 catalog entry");
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 6SL7 - high-mu octal triode.
    pub fn _6sl7() -> Self {
        let c = crate::catalog::tubes::lookup("6SL7").expect("6SL7 catalog entry");
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// Evaluate one Koren section for a given `(μ, ex)` pair. Returns
    /// `Some((e1, sigmoid, softplus, s, ip_koren))` when the section is
    /// above the deep-cutoff guard (`e1 > 0`), `None` otherwise.
    ///
    /// Shared helper for the sharp (`svar == 0`) legacy path and the
    /// Reefman §5 variable-mu blend. `s = sqrt(Kvb + Vpk²)` is used both
    /// as the softplus denominator and as the chain-rule pivot in the
    /// Jacobian, so it's returned alongside the current.
    #[inline]
    fn koren_section(&self, vgk: f64, vpk_safe: f64, mu: f64, ex: f64) -> Option<TriodeSection> {
        let s = (self.kvb + vpk_safe * vpk_safe).sqrt();
        let inner = self.kp * (1.0 / mu + vgk / s);

        // Numerically stable softplus + sigmoid.
        let (sigmoid, softplus) = if inner > 20.0 {
            (1.0, inner)
        } else if inner < -20.0 {
            (0.0, 0.0)
        } else {
            let exp_inner = inner.exp();
            (exp_inner / (1.0 + exp_inner), (1.0 + exp_inner).ln())
        };

        let e1 = (vpk_safe / self.kp) * softplus;
        if e1 <= 0.0 {
            return None;
        }

        let ip_koren = e1.powf(ex) / self.kg1;
        Some(TriodeSection {
            e1,
            sigmoid,
            softplus,
            s,
            ip_koren,
        })
    }

    /// Calculate plate current given Vgk and Vpk.
    ///
    /// Vgk = grid-cathode voltage
    /// Vpk = plate-cathode voltage
    ///
    /// Uses Koren 1996 equation with optional Early-effect multiplier:
    ///   inner = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
    ///   E1 = (Vpk / Kp) * ln(1 + exp(inner))
    ///   Ip_koren = E1^ex / Kg1   (if E1 > 0)
    ///   Ip = Ip_koren * (1 + lambda * Vpk)
    ///
    /// When `svar > 0`, the plate current is the Reefman §5 variable-mu blend
    /// of two sections sharing `Kp`/`Kvb` but with independent `(μ, ex)`:
    ///   Ip_koren_v = (1 − svar) · Ip_koren_a + svar · Ip_koren_b
    /// The `(1 + lambda·Vpk)` Early-effect multiplier is applied after the
    /// blend, unchanged.
    pub fn plate_current(&self, vgk: f64, vpk: f64) -> f64 {
        let vpk = vpk.max(1e-3); // Soft floor: prevents hard zero discontinuity

        let ip_koren = if self.svar == 0.0 {
            // Sharp single-mu path: byte-identical to the pre-P1c solver.
            match self.koren_section(vgk, vpk, self.mu, self.ex) {
                Some(sec) => sec.ip_koren,
                None => 0.0,
            }
        } else {
            let w_a = 1.0 - self.svar;
            let w_b = self.svar;
            let ip_a = self
                .koren_section(vgk, vpk, self.mu, self.ex)
                .map(|s| s.ip_koren)
                .unwrap_or(0.0);
            let ip_b = self
                .koren_section(vgk, vpk, self.mu_b, self.ex_b)
                .map(|s| s.ip_koren)
                .unwrap_or(0.0);
            w_a * ip_a + w_b * ip_b
        };

        // Early-effect multiplier (channel-length modulation)
        ip_koren * (1.0 + self.lambda * vpk)
    }

    /// Grid current using smooth power-law model (Leach-style).
    ///
    /// Ig = ig_max * max(0, Vgk / vgk_onset)^1.5
    ///
    /// Physically motivated: grid acts as a diode when positive, with
    /// a smooth onset and well-defined Jacobian. Returns 0 for Vgk <= 0.
    pub fn grid_current(&self, vgk: f64) -> f64 {
        if vgk <= 0.0 {
            return 0.0;
        }
        let x = vgk / self.vgk_onset;
        self.ig_max * x * x.sqrt() // x^1.5 = x * sqrt(x)
    }

    /// Grid current Jacobian: dIg/dVgk.
    ///
    /// dIg/dVgk = ig_max * 1.5 * (Vgk/vgk_onset)^0.5 / vgk_onset
    pub fn grid_current_jacobian(&self, vgk: f64) -> f64 {
        if vgk <= 0.0 {
            return 0.0;
        }
        let x = vgk / self.vgk_onset;
        self.ig_max * 1.5 * x.sqrt() / self.vgk_onset
    }
}

impl KorenTriode {
    /// Contribution of one Koren section to `(Ip_koren, dIp_koren/dVgk,
    /// dIp_koren/dVpk)`. Returns `None` below the deep-cutoff guard
    /// (`e1 ≤ 1e-10`), in which case the section contributes zero to
    /// both the current and both Jacobian columns.
    ///
    /// The `ex`-parameter enters both `dip_de1 = ex · E1^(ex−1) / Kg1` and
    /// the `E1` / `Ip_koren` values themselves, so it must match the section
    /// being evaluated.
    #[inline]
    fn section_chain(
        &self,
        vgk: f64,
        vpk_safe: f64,
        mu: f64,
        ex: f64,
    ) -> Option<(f64, f64, f64)> {
        let sec = self.koren_section(vgk, vpk_safe, mu, ex)?;
        // Extra guard matching the legacy Jacobian path: below 1e-10 the
        // E1^(ex-1) term blows up numerically for fractional exponents.
        if sec.e1 <= 1e-10 {
            return None;
        }
        let TriodeSection {
            e1,
            sigmoid,
            softplus,
            s,
            ip_koren,
        } = sec;

        // dIp_koren/dE1 = ex · E1^(ex − 1) / Kg1
        let dip_de1 = ex * e1.powf(ex - 1.0) / self.kg1;
        // dE1/dVgk = Vpk · σ(inner) / s
        let de1_dvgk = vpk_safe * sigmoid / s;
        // dE1/dVpk = softplus/Kp − σ·Vgk·Vpk² / s³
        let de1_dvpk = softplus / self.kp - sigmoid * vgk * vpk_safe * vpk_safe / (s * s * s);

        let dip_koren_dvgk = dip_de1 * de1_dvgk;
        let dip_koren_dvpk = dip_de1 * de1_dvpk;
        Some((ip_koren, dip_koren_dvgk, dip_koren_dvpk))
    }
}

impl NonlinearDevice<2> for KorenTriode {
    /// Input: [Vgk, Vpk]
    /// Output: Plate current (Ip)
    fn current(&self, v: &[f64; 2]) -> f64 {
        self.plate_current(v[0], v[1])
    }

    /// Jacobian: [dIp/dVgk, dIp/dVpk]
    ///
    /// Computed via chain rule through the Koren E1 equation with Early-effect:
    ///   Ip_koren = E1^ex / Kg1
    ///   Ip = Ip_koren * (1 + lambda * Vpk)
    ///   dIp/dVgk = dIp_koren/dVgk * (1 + lambda * Vpk)
    ///   dIp/dVpk = dIp_koren/dVpk * (1 + lambda * Vpk) + Ip_koren * lambda
    ///
    /// When `svar > 0`, both `Ip_koren` and its gradient are weighted sums
    /// over the two Reefman §5 sections:
    ///   Ip_koren_v      = (1 − svar) · Ip_a      + svar · Ip_b
    ///   dIp_koren_v/d·  = (1 − svar) · dIp_a/d·  + svar · dIp_b/d·
    /// The `lambda`-Early multiplier is applied once to the blended values.
    fn jacobian(&self, v: &[f64; 2]) -> [f64; 2] {
        let vgk = v[0];
        let vpk = v[1].max(1e-3); // Soft floor: matches plate_current

        let (ip_koren, dip_koren_dvgk, dip_koren_dvpk) = if self.svar == 0.0 {
            // Sharp single-mu path: byte-identical to the pre-P1c solver.
            match self.section_chain(vgk, vpk, self.mu, self.ex) {
                Some(triple) => triple,
                None => return [0.0, 0.0],
            }
        } else {
            let w_a = 1.0 - self.svar;
            let w_b = self.svar;
            let sec_a = self.section_chain(vgk, vpk, self.mu, self.ex);
            let sec_b = self.section_chain(vgk, vpk, self.mu_b, self.ex_b);

            if sec_a.is_none() && sec_b.is_none() {
                return [0.0, 0.0];
            }

            let mut ip_k = 0.0;
            let mut dip_dvgk = 0.0;
            let mut dip_dvpk = 0.0;
            if let Some((ip, dgk, dpk)) = sec_a {
                ip_k += w_a * ip;
                dip_dvgk += w_a * dgk;
                dip_dvpk += w_a * dpk;
            }
            if let Some((ip, dgk, dpk)) = sec_b {
                ip_k += w_b * ip;
                dip_dvgk += w_b * dgk;
                dip_dvpk += w_b * dpk;
            }
            (ip_k, dip_dvgk, dip_dvpk)
        };

        let lambda_factor = 1.0 + self.lambda * vpk;

        // Chain rule with Early-effect multiplier:
        // dIp/dVgk = dIp_koren/dVgk * (1 + lambda*Vpk)
        // dIp/dVpk = dIp_koren/dVpk * (1 + lambda*Vpk) + Ip_koren * lambda
        [
            dip_koren_dvgk * lambda_factor,
            dip_koren_dvpk * lambda_factor + ip_koren * self.lambda,
        ]
    }
}

/// Chain-rule pieces for one triode Koren section. Returned by
/// [`KorenTriode::koren_section`] so the shared values (`e1`, `sigmoid`,
/// `softplus`, `s`) can be reused between `plate_current` and `jacobian`.
#[derive(Clone, Copy)]
struct TriodeSection {
    /// `E1 = (Vpk / Kp) · softplus(inner)`
    e1: f64,
    /// `σ(inner)` — sigmoid of the softplus argument.
    sigmoid: f64,
    /// `softplus(inner) = ln(1 + exp(inner))`
    softplus: f64,
    /// `s = sqrt(Kvb + Vpk²)`
    s: f64,
    /// `Ip_koren = E1^ex / Kg1`
    ip_koren: f64,
}

/// Screen-current functional form for [`KorenPentode`].
///
/// Three equation families are supported:
///
/// - [`ScreenForm::Rational`] — Reefman "Derk" §4.4 (phase 1a): `1 / (1 + β·Vp)`.
///   Fits true pentodes with smooth plate-voltage knees (EL84, EL34, EF86).
/// - [`ScreenForm::Exponential`] — Reefman "DerkE" §4.5 (phase 1a.1):
///   `exp(-(β·Vp)^{3/2})`. Required for beam tetrodes whose critical-distance
///   electron-beam physics produces sharper screen-current compression
///   (6L6GC, 6V6GT).
/// - [`ScreenForm::Classical`] — Norman Koren 1996 / Cohen-Hélie 2010 (phase 1a.2).
///   Uses `arctan(Vpk/Kvb)` plate knee with Vp-independent screen current.
///   Fallback for tubes without published Reefman Derk fits (KT88, 6550).
///   Only uses μ/Ex/Kg1/Kg2/Kp/Kvb — αs/A/β are ignored.
///
/// The first two share the same Reefman Ip0/E1 softplus core; `Classical`
/// has a different E1 softplus argument (uses `Vgk/Vg2` directly instead of
/// `Vgk/sqrt(Kvb+Vg2²)`) and a fundamentally different plate-knee shape.
///
/// Duplicated locally in `melange-devices`; the matching solver-side enum is
/// [`melange_solver::device_types::ScreenForm`] and is kept in sync by the
/// codegen path that builds `KorenPentode` from `TubeParams`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ScreenForm {
    /// Reefman "Derk" §4.4 — rational `1/(1+β·Vp)` screen scaling.
    /// Default for pentode models (EL84, EL34, EF86).
    #[default]
    Rational,
    /// Reefman "DerkE" §4.5 — exponential `exp(-(β·Vp)^{3/2})` screen scaling.
    /// Required for beam tetrodes (6L6GC, 6V6GT).
    Exponential,
    /// Classical Norman Koren pentode (1996) — `arctan(Vpk/Kvb)` plate knee,
    /// Vp-independent screen current. Fallback for tubes without Reefman fits
    /// (KT88, 6550). Uses only μ/Ex/Kg1/Kg2/Kp/Kvb — the αs/A/β fields on
    /// [`KorenPentode`] are ignored when `screen_form == Classical`.
    Classical,
}

/// Reefman pentode / beam-tetrode model (plate + screen + grid currents).
///
/// Implements both Reefman D., "Spice models for vacuum tubes using the
/// uTracer" (2016) §4.4 ("Derk", true pentodes) and §4.5 ("DerkE", beam
/// tetrodes). Three independent currents — plate `Ip`, screen `Ig2`, and
/// control-grid `Ig1` — each as a function of three voltages `Vgk`, `Vpk`,
/// `Vg2k`. The control-grid current reuses the same Leach power-law form as
/// [`KorenTriode`]. The `screen_form` field selects §4.4 vs §4.5.
///
/// State variables and equations (`α = 1 − (Kg1/Kg2)·(1+αs)` is derived,
/// identical in both forms):
///
/// ```text
/// inner = Kp · (1/μ + Vgk / sqrt(Kvb + Vg2k²))
/// E1    = (Vg2k / Kp) · softplus(inner)
/// Ip0   = E1^Ex                                       (when E1 > 0)
///
/// // §4.4 Rational (Derk)
/// F(Vp) = 1/Kg1 − 1/Kg2 + A·Vp/Kg1
///         − (α/Kg1 + αs/Kg2) / (1 + β·Vp)
/// H(Vp) = (1 + αs/(1 + β·Vp)) / Kg2
///
/// // §4.5 Exponential (DerkE)
/// F(Vp) = 1/Kg1 − 1/Kg2 + A·Vp/Kg1
///         − exp(-(β·Vp)^{3/2}) · (α/Kg1 + αs/Kg2)
/// H(Vp) = (1 + αs · exp(-(β·Vp)^{3/2})) / Kg2
///
/// Ip    = Ip0 · F(Vpk)
/// Ig2   = Ip0 · H(Vpk)
/// Ig1   = Leach power-law (positive grid only)
/// ```
///
/// The 3×3 analytic Jacobian is provided by [`jacobian_3x3`](Self::jacobian_3x3);
/// rows are `[Ip, Ig2, Ig1]` and columns are `[Vgk, Vpk, Vg2k]`. The MNA /
/// codegen pipeline consumes the per-current methods directly — this struct
/// does NOT implement [`NonlinearDevice`], because that trait expects a single
/// scalar current per device.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KorenPentode {
    /// Amplification factor (μ).
    pub mu: f64,
    /// Exponent on E1.
    pub ex: f64,
    /// Plate-current scaling (Kg1).
    pub kg1: f64,
    /// Screen-current scaling (Kg2).
    pub kg2: f64,
    /// Knee softness coefficient (Kp).
    pub kp: f64,
    /// Triode-style knee shaping (Kvb). Plays the same role as Reefman's
    /// triode Kvb, NOT the classical Koren pentode arctan knee.
    pub kvb: f64,
    /// Screen-current asymmetry coefficient (αs in the Reefman paper).
    pub alpha_s: f64,
    /// Linear plate-curve slope (A in the Reefman paper). Named `a_factor`
    /// to avoid colliding with SPICE `.AC` directive parsing.
    pub a_factor: f64,
    /// Plate-knee curvature (β in the Reefman paper).
    pub beta_factor: f64,
    /// Maximum control-grid current [A] at `Vgk = vgk_onset` (Leach model).
    pub ig_max: f64,
    /// Control-grid current onset voltage [V] (Leach model).
    pub vgk_onset: f64,
    /// Screen-current functional form: Rational (§4.4) or Exponential (§4.5).
    pub screen_form: ScreenForm,
    /// Reefman §5 variable-mu section-B amplification factor (μ_b).
    /// `svar > 0` → two-section blend (PenthodeVD/VDE in TubeLib.inc);
    /// `svar == 0` → ignored, sharp single-mu pentode. 0.0 = sharp default.
    pub mu_b: f64,
    /// Reefman §5 variable-mu blend fraction (s_var in Eq 33).
    /// `Ip0_v = (1 − s_var)·Ip0_a + s_var·Ip0_b`. 0.0 = sharp default.
    pub svar: f64,
    /// Reefman §5 variable-mu section-B Koren exponent (x_b in Eq 34).
    /// Used only when `svar > 0`. 0.0 = sharp default.
    pub ex_b: f64,
}

impl KorenPentode {
    /// EL84 / 6BQ5 — popular small power pentode (BTetrodeD fit from
    /// Reefman TubeLib.inc, Jan 23 2016). Uses the §4.4 rational screen form.
    pub fn el84() -> Self {
        Self {
            mu: 23.36,
            ex: 1.138,
            kg1: 117.4,
            kg2: 1275.0,
            kp: 152.4,
            kvb: 4015.8,
            alpha_s: 7.66,
            a_factor: 4.344e-4,
            beta_factor: 0.148,
            ig_max: DEFAULT_IG_MAX,
            vgk_onset: DEFAULT_VGK_ONSET,
            screen_form: ScreenForm::Rational,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    /// EL34 / 6CA7 — power pentode (BTetrodeD fit from Reefman
    /// TubeLib.inc, Jan 23 2016). Uses the §4.4 rational screen form.
    pub fn el34() -> Self {
        Self {
            mu: 12.50,
            ex: 1.363,
            kg1: 217.7,
            kg2: 1950.2,
            kp: 50.5,
            kvb: 1282.7,
            alpha_s: 6.09,
            a_factor: 3.48e-4,
            beta_factor: 0.105,
            ig_max: DEFAULT_IG_MAX,
            vgk_onset: DEFAULT_VGK_ONSET,
            screen_form: ScreenForm::Rational,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    /// EF86 / 6267 — true small-signal pentode (PenthodeD fit from
    /// Reefman TubeLib.inc, Jan 23 2016). Uses the §4.4 rational screen form.
    pub fn ef86() -> Self {
        Self {
            mu: 40.8,
            ex: 1.327,
            kg1: 675.8,
            kg2: 4089.6,
            kp: 350.7,
            kvb: 1886.8,
            alpha_s: 4.24,
            a_factor: 5.95e-5,
            beta_factor: 0.28,
            ig_max: DEFAULT_IG_MAX,
            vgk_onset: DEFAULT_VGK_ONSET,
            screen_form: ScreenForm::Rational,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    /// 6L6GC beam tetrode — Reefman TubeLib.inc (2016) BTetrodeDE fit.
    /// Uses the §4.5 exponential screen form (beam-tetrode critical-distance
    /// screen compression).
    pub fn tetrode_6l6gc() -> Self {
        Self {
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
        }
    }

    /// 6V6GT beam tetrode — Reefman TubeLib.inc (2016) BTetrodeDE fit.
    /// Uses the §4.5 exponential screen form (beam-tetrode critical-distance
    /// screen compression).
    pub fn tetrode_6v6gt() -> Self {
        Self {
            mu: 10.56,
            ex: 1.306,
            kg1: 609.8,
            kg2: 17267.3,
            kp: 47.9,
            kvb: 2171.5,
            alpha_s: 18.72,
            a_factor: 3.48e-4,
            beta_factor: 0.068,
            ig_max: 8e-3,
            vgk_onset: 0.7,
            screen_form: ScreenForm::Exponential,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        }
    }

    /// KT88 / 6550 — classical Norman Koren beam-power tetrode fit from
    /// Cohen-Hélie 2010 DAFx "Simulation of Guitar Amplifier Tube Stages"
    /// Table 2. Uses the [`ScreenForm::Classical`] path (arctan(Vpk/Kvb)
    /// plate knee, Vp-independent screen current). Reefman has no Derk
    /// fit for KT88/6550, so Classical is the fallback.
    ///
    /// Parameters (Table 2, "KT88" row): μ=8.8, Ex=1.35, Kg1=730,
    /// Kg2=4200, Kp=32, Kvb=16. The αs/A/β/mu_b/svar/ex_b fields are
    /// unused by the Classical path and are set to 0.
    pub fn kt88() -> Self {
        Self {
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
        }
    }

    /// Derived `α = 1 − (Kg1/Kg2)·(1 + αs)`.
    #[inline]
    fn alpha(&self) -> f64 {
        1.0 - (self.kg1 / self.kg2) * (1.0 + self.alpha_s)
    }

    /// Compute the shared `(E1, Ip0)` chain along with the softplus pieces
    /// needed for both currents and Jacobian rows. Returns `None` when the
    /// device is in the deep-cutoff guard window (`E1 ≤ 1e-30`).
    ///
    /// This is the legacy single-section entry point used by the sharp
    /// (`svar == 0`) path. It delegates to [`shared_e1_with`], passing the
    /// struct's top-level `μ` (a.k.a. `μ_a`) and `ex` (a.k.a. `x_a`).
    /// Currently unused in the hot path (sharp and variable-mu both call
    /// `shared_e1_with` directly); kept as documentation of the sharp-path
    /// semantics for readers.
    #[inline]
    #[allow(dead_code)]
    fn shared_e1(&self, vgk: f64, vg2k_safe: f64) -> Option<SharedE1> {
        self.shared_e1_with(vgk, vg2k_safe, self.mu, self.ex)
    }

    /// Compute the `(E1, Ip0)` chain for an arbitrary `(μ, ex)` pair. This is
    /// the primitive used by the Reefman §5 variable-mu blend, where the two
    /// sections share `Kp` and `Kvb` (hence `s`) but have distinct
    /// amplification factors and exponents.
    ///
    /// Returns `None` when `E1 ≤ 1e-30` (deep-cutoff guard, per-section).
    #[inline]
    fn shared_e1_with(
        &self,
        vgk: f64,
        vg2k_safe: f64,
        mu: f64,
        ex: f64,
    ) -> Option<SharedE1> {
        let s = (self.kvb + vg2k_safe * vg2k_safe).sqrt();
        let inner = self.kp * (1.0 / mu + vgk / s);

        // Numerically stable softplus and sigmoid (mirrors KorenTriode).
        let (sigmoid, softplus) = if inner > 20.0 {
            (1.0, inner)
        } else if inner < -20.0 {
            (0.0, 0.0)
        } else {
            let exp_inner = inner.exp();
            (exp_inner / (1.0 + exp_inner), (1.0 + exp_inner).ln())
        };

        let e1 = (vg2k_safe / self.kp) * softplus;
        if e1 <= 1e-30 {
            return None;
        }

        let ip0 = e1.powf(ex);
        Some(SharedE1 {
            s,
            sigmoid,
            softplus,
            e1,
            ip0,
        })
    }

    /// Reefman §5 variable-mu `Ip0_v` and its `(Vgk, Vg2k)` gradient.
    ///
    /// Returns `(ip0_v, dip0_dvgk, dip0_dvg2k)` where
    /// `ip0_v = (1 − svar)·ip0_a + svar·ip0_b` and the gradient is the
    /// corresponding weighted sum of the per-section `dip0_de1 · dE1/d*`
    /// chains. The two sections share `Kp`, `Kvb`, and therefore `s`, but
    /// have independent `μ`, `ex`, inner / softplus / sigmoid / E1 / Ip0.
    ///
    /// When `svar == 0.0` this bypasses section B entirely (which would be
    /// undefined for `μ_b == 0`) and reduces byte-identically to the legacy
    /// sharp single-mu path. When both sections are in deep cutoff the
    /// function returns `(0.0, 0.0, 0.0)` — the caller still has to route
    /// through `compute_f_h` to produce a result; passing `ip0=0` there
    /// propagates the zero correctly through the Ip / Ig2 rows.
    #[inline]
    fn compute_ip0_v(&self, vgk: f64, vg2k_safe: f64) -> Ip0VResult {
        if self.svar == 0.0 {
            // Sharp single-mu path: byte-identical to the pre-P1c solver.
            // Section B is NOT computed (it would be undefined for μ_b = 0).
            let Some(shared_a) = self.shared_e1_with(vgk, vg2k_safe, self.mu, self.ex) else {
                return Ip0VResult::zero();
            };
            let SharedE1 {
                s,
                sigmoid,
                softplus,
                e1,
                ip0,
            } = shared_a;
            // dIp0_a / dE1_a = ex_a · E1_a^(ex_a − 1)
            let dip0_de1 = self.ex * e1.powf(self.ex - 1.0);
            let de1_dvgk = vg2k_safe * sigmoid / s;
            let de1_dvg2k =
                softplus / self.kp - sigmoid * vgk * vg2k_safe * vg2k_safe / (s * s * s);
            return Ip0VResult {
                ip0_v: ip0,
                dip0_dvgk: dip0_de1 * de1_dvgk,
                dip0_dvg2k: dip0_de1 * de1_dvg2k,
            };
        }

        // Variable-mu path: two independent (μ, ex) sections blended per Eq 33.
        // s is shared between sections (same Kp, Kvb), so each call re-derives
        // the same `s` — the cost is negligible compared to the two softplus
        // evaluations that drive the branch.
        let w_a = 1.0 - self.svar;
        let w_b = self.svar;

        let section_a = self.shared_e1_with(vgk, vg2k_safe, self.mu, self.ex);
        let section_b = self.shared_e1_with(vgk, vg2k_safe, self.mu_b, self.ex_b);

        // Both sections in deep cutoff → zero device current and gradient.
        if section_a.is_none() && section_b.is_none() {
            return Ip0VResult::zero();
        }

        let mut ip0_v = 0.0;
        let mut dip0_dvgk = 0.0;
        let mut dip0_dvg2k = 0.0;

        if let Some(SharedE1 {
            s,
            sigmoid,
            softplus,
            e1,
            ip0,
        }) = section_a
        {
            // dIp0_a/dE1_a = x_a · E1_a^(x_a − 1)
            let dip0_de1 = self.ex * e1.powf(self.ex - 1.0);
            let de1_dvgk = vg2k_safe * sigmoid / s;
            let de1_dvg2k =
                softplus / self.kp - sigmoid * vgk * vg2k_safe * vg2k_safe / (s * s * s);
            ip0_v += w_a * ip0;
            dip0_dvgk += w_a * dip0_de1 * de1_dvgk;
            dip0_dvg2k += w_a * dip0_de1 * de1_dvg2k;
        }

        if let Some(SharedE1 {
            s,
            sigmoid,
            softplus,
            e1,
            ip0,
        }) = section_b
        {
            // dIp0_b/dE1_b = x_b · E1_b^(x_b − 1)
            let dip0_de1 = self.ex_b * e1.powf(self.ex_b - 1.0);
            let de1_dvgk = vg2k_safe * sigmoid / s;
            let de1_dvg2k =
                softplus / self.kp - sigmoid * vgk * vg2k_safe * vg2k_safe / (s * s * s);
            ip0_v += w_b * ip0;
            dip0_dvgk += w_b * dip0_de1 * de1_dvgk;
            dip0_dvg2k += w_b * dip0_de1 * de1_dvg2k;
        }

        Ip0VResult {
            ip0_v,
            dip0_dvgk,
            dip0_dvg2k,
        }
    }

    /// Compute the shape functions `F(Vp)`, `H(Vp)` and their Vp-derivatives
    /// for the active `screen_form`. `vpk_safe` must already be clamped to
    /// `>= 0` by the caller.
    #[inline]
    fn compute_f_h(&self, vpk_safe: f64) -> FHShape {
        let alpha = self.alpha();
        let coeff = alpha / self.kg1 + self.alpha_s / self.kg2;
        let one_over_kg1_minus_kg2 = 1.0 / self.kg1 - 1.0 / self.kg2;
        let linear_term = self.a_factor * vpk_safe / self.kg1;

        match self.screen_form {
            ScreenForm::Rational => {
                // §4.4: scale = 1 / (1 + β·Vp)
                let one_plus_bvp = 1.0 + self.beta_factor * vpk_safe;
                let inv_obvp = 1.0 / one_plus_bvp;
                let inv_obvp_sq = inv_obvp * inv_obvp;

                let f = one_over_kg1_minus_kg2 + linear_term - coeff * inv_obvp;
                let h = (1.0 + self.alpha_s * inv_obvp) / self.kg2;

                // d(1/(1+βVp))/dVp = -β / (1+βVp)^2
                // dF/dVp = A/Kg1 + β·coeff / (1+βVp)^2
                let df_dvpk = self.a_factor / self.kg1 + self.beta_factor * coeff * inv_obvp_sq;
                // dH/dVp = -β·αs / (Kg2·(1+βVp)^2)
                let dh_dvpk = -self.beta_factor * self.alpha_s * inv_obvp_sq / self.kg2;

                FHShape {
                    f,
                    h,
                    df_dvpk,
                    dh_dvpk,
                }
            }
            ScreenForm::Exponential => {
                // §4.5: scale = exp(-(β·Vp)^{3/2})
                // u = β·Vp clamped >= 0 (vpk_safe already >= 0, β > 0)
                let u = (self.beta_factor * vpk_safe).max(0.0);
                // u^{3/2}. When u == 0, both u^{3/2} = 0 AND sqrt(u) = 0 so
                // derivatives remain finite (no (-)^{1.5} NaN exposure).
                let u_32 = u * u.sqrt(); // = u^{3/2}
                let ex_factor = (-u_32).exp();
                let sqrt_u = u.sqrt();

                let f = one_over_kg1_minus_kg2 + linear_term - ex_factor * coeff;
                let h = (1.0 + self.alpha_s * ex_factor) / self.kg2;

                // d(u^{3/2})/dVp = 1.5 · β · sqrt(u)   (u in Vp units → β factor)
                // d(exp(-u^{3/2}))/dVp = -1.5 · β · sqrt(u) · ex_factor
                // dF/dVp = A/Kg1 + 1.5·β·sqrt(u)·ex_factor · coeff
                let deriv_prefactor = 1.5 * self.beta_factor * sqrt_u * ex_factor;
                let df_dvpk = self.a_factor / self.kg1 + deriv_prefactor * coeff;
                // dH/dVp = -1.5·β·sqrt(u)·ex_factor·αs / Kg2
                let dh_dvpk = -deriv_prefactor * self.alpha_s / self.kg2;

                FHShape {
                    f,
                    h,
                    df_dvpk,
                    dh_dvpk,
                }
            }
            ScreenForm::Classical => {
                // Classical Koren pentode uses a fundamentally different
                // plate-knee formulation (arctan(Vpk/Kvb) factor outside
                // the F/H split) and a different E1 softplus argument
                // (Vgk/Vg2 instead of Vgk/sqrt(Kvb+Vg2²)). It cannot
                // reuse `compute_f_h`; callers (`plate_current`,
                // `screen_current`, `jacobian_3x3`) short-circuit to
                // `plate_current_classical` etc. before reaching this
                // point when `screen_form == Classical`. If we land
                // here something upstream failed to short-circuit.
                unreachable!(
                    "compute_f_h called on Classical pentode — caller must \
                     short-circuit to classical-specific path (task P1a2-02)"
                );
            }
        }
    }

    /// Plate current `Ip(Vgk, Vpk, Vg2k)`.
    ///
    /// When `svar > 0`, the variable-mu `Ip0_v = (1 − svar)·Ip0_a + svar·Ip0_b`
    /// (Reefman §5 Eq 33) is used in place of the sharp `Ip0`, with F(Vp)
    /// unchanged between the two sections (Eq 36).
    pub fn plate_current(&self, vgk: f64, vpk: f64, vg2k: f64) -> f64 {
        if matches!(self.screen_form, ScreenForm::Classical) {
            return self.plate_current_classical(vgk, vpk, vg2k);
        }
        let vg2k_safe = vg2k.max(1e-3);
        let vpk_safe = vpk.max(0.0);

        let Ip0VResult { ip0_v, .. } = self.compute_ip0_v(vgk, vg2k_safe);
        if ip0_v == 0.0 {
            return 0.0;
        }

        let FHShape { f, .. } = self.compute_f_h(vpk_safe);
        ip0_v * f
    }

    /// Screen-grid current `Ig2(Vgk, Vpk, Vg2k)`.
    ///
    /// Uses the same variable-mu `Ip0_v` as [`plate_current`], with H(Vp)
    /// unchanged (Eq 37). Variable-mu is orthogonal to `screen_form`.
    pub fn screen_current(&self, vgk: f64, vpk: f64, vg2k: f64) -> f64 {
        if matches!(self.screen_form, ScreenForm::Classical) {
            return self.screen_current_classical(vgk, vpk, vg2k);
        }
        let vg2k_safe = vg2k.max(1e-3);
        let vpk_safe = vpk.max(0.0);

        let Ip0VResult { ip0_v, .. } = self.compute_ip0_v(vgk, vg2k_safe);
        if ip0_v == 0.0 {
            return 0.0;
        }

        let FHShape { h, .. } = self.compute_f_h(vpk_safe);
        ip0_v * h
    }

    /// Control-grid (Ig1) current using the same Leach power-law as the
    /// triode. Returns 0 for `Vgk ≤ 0`. Shared across all three screen
    /// forms (Rational / Exponential / Classical) — the grid current
    /// model does not depend on screen_form.
    pub fn grid_current(&self, vgk: f64) -> f64 {
        if vgk <= 0.0 {
            return 0.0;
        }
        let x = vgk / self.vgk_onset;
        self.ig_max * x * x.sqrt() // x^1.5
    }

    // ---------------------------------------------------------------------
    // Classical Norman Koren pentode path (phase 1a.2).
    //
    // Separate from the Reefman Derk / DerkE path because the softplus
    // argument, plate-knee shape, and screen-current formula are all
    // structurally different. The `plate_current` / `screen_current` /
    // `jacobian_3x3` entry points short-circuit to these methods when
    // `self.screen_form == ScreenForm::Classical`.
    //
    // Equations from Cohen-Hélie 2010 DAFx Eqs 1-3 (originally Norman
    // Koren 1996):
    //
    //     E1   = (Vg2k/Kp) · log(1 + exp(Kp · (1/μ + Vgk/Vg2k)))
    //     Ip   = (E1^Ex / Kg1) · (1 + sgn(E1)) · arctan(Vpk/Kvb)
    //     Ig2  = (Vg2k/μ + Vgk)^Ex / Kg2     (Vp-INDEPENDENT)
    //
    // Classical uses only 6 parameters: μ, Ex, Kg1, Kg2, Kp, Kvb. The
    // αs/A/β fields on [`KorenPentode`] are ignored for Classical
    // entries. Kvb plays the role of the arctan knee scale, NOT the
    // softplus denominator (where the Derk path uses sqrt(Kvb+Vg2²)).
    // ---------------------------------------------------------------------

    /// Classical Koren softplus chain. Returns `(E1, sigmoid, softplus)`
    /// where
    ///
    /// ```text
    /// inner    = Kp · (1/μ + Vgk / Vg2k_safe)
    /// softplus = ln(1 + exp(inner))
    /// sigmoid  = exp(inner) / (1 + exp(inner))
    /// E1       = (Vg2k_safe / Kp) · softplus
    /// ```
    ///
    /// `vg2k_safe` must already be clamped to `≥ 1e-3` by the caller.
    /// The `inner ∈ [-20, 20]` clamp mirrors the NR-safety pattern in
    /// the existing Derk path (`shared_e1_with`) and prevents softplus
    /// overflow / sigmoid underflow during NR probing.
    #[inline]
    fn classical_e1(&self, vgk: f64, vg2k_safe: f64) -> (f64, f64, f64) {
        let inner = self.kp * (1.0 / self.mu + vgk / vg2k_safe);
        let (sigmoid, softplus) = if inner > 20.0 {
            // softplus(x) → x, σ(x) → 1
            (1.0, inner)
        } else if inner < -20.0 {
            // softplus(x) → 0, σ(x) → 0
            (0.0, 0.0)
        } else {
            let e = inner.exp();
            (e / (1.0 + e), (1.0 + e).ln())
        };
        let e1 = (vg2k_safe / self.kp) * softplus;
        (e1, sigmoid, softplus)
    }

    /// Classical Koren plate current. Cohen-Hélie 2010 Eq 2 with the
    /// `(1 + sgn(E1))` hard step smoothed to 2 in the `E1 > 0` regime
    /// (softplus already provides the smoothing). Guards: `Vg2k ≥ 1e-3`
    /// prevents the `Vgk/Vg2k` softplus argument from singularity;
    /// `Vpk ≥ 0` prevents non-physical `arctan(Vpk/Kvb)` sign flip
    /// during NR probing.
    fn plate_current_classical(&self, vgk: f64, vpk: f64, vg2k: f64) -> f64 {
        let vg2k_safe = vg2k.max(1e-3);
        let vpk_safe = vpk.max(0.0);
        let (e1, _sigmoid, _softplus) = self.classical_e1(vgk, vg2k_safe);
        if e1 <= 1e-30 {
            return 0.0;
        }
        let g = (vpk_safe / self.kvb).atan();
        let ip0 = 2.0 * e1.powf(self.ex) / self.kg1;
        ip0 * g
    }

    /// Classical Koren screen current. Cohen-Hélie 2010 Eq 3 with the
    /// `/Kg2` divisor restored (the printed paper omits it; the
    /// corrected form matches Norman Koren's 1996 original and makes
    /// Kg2 from Table 2 a live parameter). Vp-independent by design —
    /// this is the classical simplification that makes the screen
    /// current depend only on `Vgk` and `Vg2k`.
    fn screen_current_classical(&self, vgk: f64, _vpk: f64, vg2k: f64) -> f64 {
        let vg2k_safe = vg2k.max(1e-3);
        let x = vg2k_safe / self.mu + vgk;
        if x <= 0.0 {
            return 0.0;
        }
        x.powf(self.ex) / self.kg2
    }

    /// Classical Koren 3×3 analytic Jacobian. Rows: `[Ip, Ig2, Ig1]`.
    /// Columns: `[Vgk, Vpk, Vg2k]`. Structural zeros:
    /// - `[1][1] = 0` (Ig2 is Vp-independent)
    /// - `[2][1] = [2][2] = 0` (Ig1 depends only on Vgk)
    ///
    /// Safety guards mirror [`plate_current_classical`] and
    /// [`screen_current_classical`]: `Vg2k ≥ 1e-3`, `Vpk ≥ 0` (the
    /// `arctan` derivative at clamped `Vpk=0` gives `1/Kvb`, the
    /// tangent slope at the origin — consistent with the clamped
    /// plate current at the same input).
    fn jacobian_3x3_classical(
        &self,
        vgk: f64,
        vpk: f64,
        vg2k: f64,
    ) -> [[f64; 3]; 3] {
        let vg2k_safe = vg2k.max(1e-3);
        let vpk_safe = vpk.max(0.0);

        // Ig1 row (Leach power-law, shared with Derk path). Independent
        // of Vpk / Vg2k so it's the only nonzero entry on row [2].
        let dig1_dvgk = if vgk > 0.0 {
            let x = vgk / self.vgk_onset;
            self.ig_max * 1.5 * x.sqrt() / self.vgk_onset
        } else {
            0.0
        };

        // --- Ip row (derived from Ip = 2·E1^Ex/Kg1 · arctan(Vpk/Kvb)) ---
        let (e1, sigmoid, softplus) = self.classical_e1(vgk, vg2k_safe);

        // Deep-cutoff guard: when softplus underflowed to 0, Ip is 0
        // and its whole row is zero. Ig2 row is handled separately
        // below (its gate is `x > 0`, not `E1 > 0`).
        let (dip_dvgk, dip_dvpk, dip_dvg2k) = if e1 <= 1e-30 {
            (0.0, 0.0, 0.0)
        } else {
            let g = (vpk_safe / self.kvb).atan();
            // dG/dVpk = (1/Kvb) / (1 + (Vpk/Kvb)^2)
            let r = vpk_safe / self.kvb;
            let dg_dvpk = (1.0 / self.kvb) / (1.0 + r * r);

            // Ip0 = 2·E1^Ex / Kg1
            let ip0 = 2.0 * e1.powf(self.ex) / self.kg1;
            // dIp0/dE1 = 2·Ex·E1^(Ex-1) / Kg1
            let dip0_de1 = 2.0 * self.ex * e1.powf(self.ex - 1.0) / self.kg1;

            // E1 partials (chain rule through softplus):
            //   dE1/dVgk  = sigmoid
            //   dE1/dVg2k = softplus/Kp − sigmoid · Vgk / Vg2k_safe
            //   dE1/dVpk  = 0
            let de1_dvgk = sigmoid;
            let de1_dvg2k =
                softplus / self.kp - sigmoid * vgk / vg2k_safe;

            // Chain rule with G(Vpk) = arctan(Vpk/Kvb):
            //   dIp/dVgk  = (dIp0/dE1)·(dE1/dVgk) · G
            //   dIp/dVpk  = Ip0 · dG/dVpk
            //   dIp/dVg2k = (dIp0/dE1)·(dE1/dVg2k) · G
            (
                dip0_de1 * de1_dvgk * g,
                ip0 * dg_dvpk,
                dip0_de1 * de1_dvg2k * g,
            )
        };

        // --- Ig2 row (Ig2 = x^Ex / Kg2, x = Vg2k_safe/μ + Vgk) ---
        let x = vg2k_safe / self.mu + vgk;
        let (dig2_dvgk, dig2_dvg2k) = if x > 0.0 {
            let prefactor = self.ex * x.powf(self.ex - 1.0) / self.kg2;
            // dx/dVgk  = 1
            // dx/dVg2k = 1/μ
            (prefactor, prefactor / self.mu)
        } else {
            (0.0, 0.0)
        };
        // Ig2 is Vp-independent → [1][1] is structurally 0.
        let dig2_dvpk = 0.0;

        [
            [dip_dvgk, dip_dvpk, dip_dvg2k],
            [dig2_dvgk, dig2_dvpk, dig2_dvg2k],
            [dig1_dvgk, 0.0, 0.0],
        ]
    }

    /// 3×3 analytic Jacobian.
    ///
    /// Rows: `[Ip, Ig2, Ig1]`. Columns: `[Vgk, Vpk, Vg2k]`. Returned in
    /// row-major order. The bottom row (Ig1) is sparse: only `[2][0]` is
    /// nonzero, since `Ig1` depends on `Vgk` alone.
    ///
    /// When `svar > 0`, the `Ip0` row-prefactor becomes the Reefman §5 blend
    /// `Ip0_v = (1 − svar)·Ip0_a + svar·Ip0_b` and its gradient is the
    /// weighted sum of per-section `(dIp0/dE1)·(dE1/d·)` chains. F(Vp),
    /// H(Vp), dF/dVp, dH/dVp are unchanged (orthogonal to variable-mu).
    pub fn jacobian_3x3(&self, vgk: f64, vpk: f64, vg2k: f64) -> [[f64; 3]; 3] {
        if matches!(self.screen_form, ScreenForm::Classical) {
            return self.jacobian_3x3_classical(vgk, vpk, vg2k);
        }
        let vg2k_safe = vg2k.max(1e-3);
        let vpk_safe = vpk.max(0.0);

        // Ig1 row is independent of Vpk / Vg2k.
        let dig1_dvgk = if vgk > 0.0 {
            let x = vgk / self.vgk_onset;
            self.ig_max * 1.5 * x.sqrt() / self.vgk_onset
        } else {
            0.0
        };

        let Ip0VResult {
            ip0_v,
            dip0_dvgk,
            dip0_dvg2k,
        } = self.compute_ip0_v(vgk, vg2k_safe);

        // Both sections in deep cutoff → Ip / Ig2 rows are identically zero.
        if ip0_v == 0.0 {
            return [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [dig1_dvgk, 0.0, 0.0],
            ];
        }

        // F, H and their Vp derivatives — branches on screen_form (§4.4 vs §4.5).
        // These are shared between A/B sections (Reefman §5 makes variable-mu
        // orthogonal to screen_form), so they're computed once here.
        let FHShape {
            f,
            h,
            df_dvpk,
            dh_dvpk,
        } = self.compute_f_h(vpk_safe);

        // Ip row: dIp/dVgk = (dIp0_v/dVgk)·F, dIp/dVpk = Ip0_v·dF/dVpk,
        //         dIp/dVg2k = (dIp0_v/dVg2k)·F
        let dip_dvgk = dip0_dvgk * f;
        let dip_dvpk = ip0_v * df_dvpk;
        let dip_dvg2k = dip0_dvg2k * f;

        // Ig2 row: same prefactor, with H instead of F.
        let dig2_dvgk = dip0_dvgk * h;
        let dig2_dvpk = ip0_v * dh_dvpk;
        let dig2_dvg2k = dip0_dvg2k * h;

        // If NR probed Vpk < 0, the F/H derivatives w.r.t. Vpk are computed
        // with vpk_safe (clamped to 0), so dIp/dVpk and dIg2/dVpk reduce to
        // the values at Vpk = 0. That mirrors what plate_current/screen_current
        // return on the same input, keeping FD comparisons consistent.

        [
            [dip_dvgk, dip_dvpk, dip_dvg2k],
            [dig2_dvgk, dig2_dvpk, dig2_dvg2k],
            [dig1_dvgk, 0.0, 0.0],
        ]
    }
}

/// Result of [`KorenPentode::compute_ip0_v`] — variable-mu `Ip0_v` with its
/// `(Vgk, Vg2k)` gradient. Reduces to the sharp single-section
/// `(Ip0, dIp0/dVgk, dIp0/dVg2k)` when `svar == 0`.
#[derive(Clone, Copy)]
struct Ip0VResult {
    /// `Ip0_v = (1 − svar)·Ip0_a + svar·Ip0_b`.
    ip0_v: f64,
    /// Weighted sum of `dIp0_{a,b}/dVgk`.
    dip0_dvgk: f64,
    /// Weighted sum of `dIp0_{a,b}/dVg2k`.
    dip0_dvg2k: f64,
}

impl Ip0VResult {
    #[inline]
    fn zero() -> Self {
        Self {
            ip0_v: 0.0,
            dip0_dvgk: 0.0,
            dip0_dvg2k: 0.0,
        }
    }
}

/// Shared chain-rule pieces produced by [`KorenPentode::shared_e1`].
#[derive(Clone, Copy)]
struct SharedE1 {
    /// `sqrt(Kvb + Vg2k²)`
    s: f64,
    /// `sigmoid(inner)`
    sigmoid: f64,
    /// `softplus(inner)`
    softplus: f64,
    /// `E1 = (Vg2k / Kp) · softplus(inner)`
    e1: f64,
    /// `Ip0 = E1^Ex`
    ip0: f64,
}

/// Shape-function values and Vp-derivatives returned by
/// [`KorenPentode::compute_f_h`]. Branches internally on `ScreenForm`.
#[derive(Clone, Copy)]
struct FHShape {
    /// `F(Vpk)` — plate-current shape factor.
    f: f64,
    /// `H(Vpk)` — screen-current shape factor.
    h: f64,
    /// `dF/dVpk`.
    df_dvpk: f64,
    /// `dH/dVpk`.
    dh_dvpk: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triode_cutoff() {
        let tube = KorenTriode::ecc83();

        // Negative grid voltage should reduce current
        // The Koren model doesn't have a hard cutoff, just reduced current
        let ip_off = tube.plate_current(-2.0, 250.0);
        let ip_on = tube.plate_current(0.0, 250.0);
        let ip_very_off = tube.plate_current(-5.0, 250.0);

        // More negative grid should give less current
        assert!(ip_off < ip_on, "More negative grid should reduce current");
        assert!(
            ip_very_off < ip_off,
            "Even more negative should reduce further"
        );
    }

    #[test]
    fn test_triode_conduction() {
        let tube = KorenTriode::ecc83();

        // At Vgk=0, Vpk=250V, should have milliamps of plate current
        let ip = tube.plate_current(0.0, 250.0);
        assert!(ip > 0.001, "Ip should be in mA range: {} A", ip);
        assert!(ip < 0.01, "Ip should not be excessive: {} A", ip);
    }

    #[test]
    fn test_triode_mu() {
        let tube = KorenTriode::ecc83();

        // Test that mu amplifies grid voltage effect
        let ip_vgk_0 = tube.plate_current(0.0, 250.0);
        let ip_vgk_minus1 = tube.plate_current(-1.0, 250.0);
        let ip_vgk_minus2 = tube.plate_current(-2.0, 250.0);

        assert!(ip_vgk_minus1 < ip_vgk_0);
        assert!(ip_vgk_minus2 < ip_vgk_minus1);

        let ratio = ip_vgk_0 / ip_vgk_minus2;
        assert!(
            ratio > 2.0,
            "Mu effect should be noticeable, ratio = {}",
            ratio
        );
    }

    #[test]
    fn test_triode_trait() {
        let tube = KorenTriode::ecc83();

        let ip = tube.current(&[0.0, 250.0]);
        let jac = tube.jacobian(&[0.0, 250.0]);

        assert!(ip > 0.0);
        assert!(jac[0] > 0.0); // dIp/dVgk > 0 (transconductance)
        assert!(jac[1] > 0.0); // dIp/dVpk > 0 (output conductance)
    }

    #[test]
    fn test_12ax7_plate_curves() {
        let tube = KorenTriode::ecc83();

        let ip_0 = tube.plate_current(0.0, 250.0);
        assert!(
            ip_0 > 0.5e-3 && ip_0 < 5.0e-3,
            "Ip(Vgk=0, Vpk=250) = {:.3}mA, expected 0.5-5.0mA",
            ip_0 * 1000.0
        );

        let ip_m1 = tube.plate_current(-1.0, 250.0);
        assert!(
            ip_m1 > 0.1e-3 && ip_m1 < 3.0e-3,
            "Ip(Vgk=-1, Vpk=250) = {:.3}mA, expected 0.1-3.0mA",
            ip_m1 * 1000.0
        );

        let ip_m2 = tube.plate_current(-2.0, 250.0);
        assert!(
            ip_m2 > 0.01e-3 && ip_m2 < 1.5e-3,
            "Ip(Vgk=-2, Vpk=250) = {:.3}mA, expected 0.01-1.5mA",
            ip_m2 * 1000.0
        );

        assert!(ip_0 > ip_m1 && ip_m1 > ip_m2);

        let ip_m4 = tube.plate_current(-4.0, 250.0);
        assert!(
            ip_m4 < 0.01e-3,
            "Ip(Vgk=-4, Vpk=250) = {:.4}mA, expected near cutoff",
            ip_m4 * 1000.0
        );
    }

    #[test]
    fn test_12ax7_fitted_plate_curves() {
        let tube = KorenTriode::ecc83_fitted();

        // Fitted to RCA datasheet: Vgk=0, Vpk=250V → ~1.2mA
        let ip_0 = tube.plate_current(0.0, 250.0);
        assert!(
            ip_0 > 1.0e-3 && ip_0 < 1.4e-3,
            "Ip(Vgk=0, Vpk=250) = {:.4}mA, expected ~1.2mA",
            ip_0 * 1000.0
        );

        // Vgk=-1V: fitted → ~0.59mA (datasheet: ~0.5mA)
        let ip_m1 = tube.plate_current(-1.0, 250.0);
        assert!(
            ip_m1 > 0.3e-3 && ip_m1 < 0.8e-3,
            "Ip(Vgk=-1, Vpk=250) = {:.4}mA, expected ~0.5-0.6mA",
            ip_m1 * 1000.0
        );

        // Vgk=-2V: fitted → ~0.17mA (datasheet: ~0.1mA)
        let ip_m2 = tube.plate_current(-2.0, 250.0);
        assert!(
            ip_m2 > 0.05e-3 && ip_m2 < 0.3e-3,
            "Ip(Vgk=-2, Vpk=250) = {:.4}mA, expected ~0.1-0.2mA",
            ip_m2 * 1000.0
        );

        // Monotonicity and cutoff
        assert!(ip_0 > ip_m1 && ip_m1 > ip_m2);
        assert!(
            tube.plate_current(-4.0, 250.0) < 0.01e-3,
            "Should be near cutoff at Vgk=-4"
        );

        // Fitted parameters now match default (both use Kg1=3000)
        let default = KorenTriode::ecc83();
        assert_eq!(tube.mu, default.mu);
        assert_eq!(tube.ex, default.ex);
        assert_eq!(tube.kp, default.kp);
        assert_eq!(tube.kvb, default.kvb);
        assert_eq!(
            tube.kg1, default.kg1,
            "Fitted and default now use same Kg1=3000"
        );
    }

    /// Verify triode Jacobian against finite differences.
    #[test]
    fn test_triode_jacobian_finite_difference() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-6;

        for &(vgk, vpk) in &[(0.0, 250.0), (-1.0, 200.0), (-2.0, 100.0), (-0.5, 300.0)] {
            let jac = tube.jacobian(&[vgk, vpk]);

            let ip_plus = tube.plate_current(vgk + eps, vpk);
            let ip_minus = tube.plate_current(vgk - eps, vpk);
            let fd_dvgk = (ip_plus - ip_minus) / (2.0 * eps);

            let ip_plus = tube.plate_current(vgk, vpk + eps);
            let ip_minus = tube.plate_current(vgk, vpk - eps);
            let fd_dvpk = (ip_plus - ip_minus) / (2.0 * eps);

            let rel_err_gk = if fd_dvgk.abs() > 1e-15 {
                (jac[0] - fd_dvgk).abs() / fd_dvgk.abs()
            } else {
                jac[0].abs()
            };
            let rel_err_pk = if fd_dvpk.abs() > 1e-15 {
                (jac[1] - fd_dvpk).abs() / fd_dvpk.abs()
            } else {
                jac[1].abs()
            };

            assert!(
                rel_err_gk < 1e-4,
                "dIp/dVgk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[0],
                fd_dvgk,
                rel_err_gk
            );
            assert!(
                rel_err_pk < 1e-4,
                "dIp/dVpk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[1],
                fd_dvpk,
                rel_err_pk
            );
        }
    }

    #[test]
    fn test_el84_operating_point_class_a() {
        // Typical EL84 Class A bias: Vgk=-7, Vpk=Vg2k=300V.
        // The Reefman TubeLib.inc fit predicts Ip ≈ 67.7 mA / Ig2 ≈ 7.2 mA
        // here. The datasheet idle is closer to ~48 mA, but Reefman fits
        // optimize over the whole curve family rather than the bias point,
        // so the bound is intentionally generous (10-100 mA / 1-15 mA) — it
        // catches "wrong order of magnitude" or "negative current" bugs
        // without locking the test to one specific fit's bias-point error.
        let pent = KorenPentode::el84();
        let ip = pent.plate_current(-7.0, 300.0, 300.0);
        let ig2 = pent.screen_current(-7.0, 300.0, 300.0);

        assert!(
            (10e-3..=100e-3).contains(&ip),
            "EL84 Ip(Vgk=-7, Vpk=Vg2k=300) = {:.2}mA, expected 10-100mA",
            ip * 1000.0
        );
        assert!(
            (1e-3..=15e-3).contains(&ig2),
            "EL84 Ig2(Vgk=-7, Vpk=Vg2k=300) = {:.2}mA, expected 1-15mA",
            ig2 * 1000.0
        );
        assert!(
            ip > ig2,
            "Pentode plate current must exceed screen current in Class A: \
             Ip={:.3}mA, Ig2={:.3}mA",
            ip * 1000.0,
            ig2 * 1000.0
        );
    }

    #[test]
    fn test_el84_cutoff() {
        // Reefman EL84 has a slow cutoff: it takes Vgk ≈ -30 V before Ip
        // drops below ~1 µA, and Vgk ≈ -40 V before nA range. Use -40 V
        // here so the cutoff threshold is meaningful.
        let pent = KorenPentode::el84();
        let ip = pent.plate_current(-40.0, 300.0, 300.0);
        let ig2 = pent.screen_current(-40.0, 300.0, 300.0);
        assert!(
            ip < 1e-6,
            "EL84 Ip at deep cutoff (Vgk=-40) should be < 1uA, got {:.3e} A",
            ip
        );
        assert!(
            ig2 < 1e-6,
            "EL84 Ig2 at deep cutoff (Vgk=-40) should be < 1uA, got {:.3e} A",
            ig2
        );
    }

    #[test]
    fn test_el84_monotonic_in_vgk() {
        let pent = KorenPentode::el84();
        let mut prev = -1.0;
        for k in 0..10 {
            let vgk = -10.0 + k as f64 * 1.0; // -10..-1 in 1V steps
            let ip = pent.plate_current(vgk, 300.0, 300.0);
            assert!(
                ip > prev,
                "EL84 Ip should increase monotonically in Vgk: \
                 Vgk={}, Ip={:.6e}, prev={:.6e}",
                vgk,
                ip,
                prev
            );
            prev = ip;
        }
    }

    #[test]
    fn test_el84_jacobian_finite_difference() {
        let pent = KorenPentode::el84();
        let eps = 1e-6;

        // Use a moderate Class-A operating point to avoid the deep-cutoff
        // FD step landing on the E1<=1e-30 guard.
        let (vgk, vpk, vg2k) = (-5.0, 300.0, 300.0);
        let jac = pent.jacobian_3x3(vgk, vpk, vg2k);

        // Helpers to compute each current with one perturbed coordinate.
        let plate = |a: f64, b: f64, c: f64| pent.plate_current(a, b, c);
        let screen = |a: f64, b: f64, c: f64| pent.screen_current(a, b, c);

        // (current_idx, name, fn) for the rows we want to FD-check.
        let fd = |f: &dyn Fn(f64, f64, f64) -> f64, dim: usize| -> f64 {
            let mut p = [vgk, vpk, vg2k];
            let mut m = [vgk, vpk, vg2k];
            p[dim] += eps;
            m[dim] -= eps;
            (f(p[0], p[1], p[2]) - f(m[0], m[1], m[2])) / (2.0 * eps)
        };

        let row_specs: [(usize, &str, &dyn Fn(f64, f64, f64) -> f64); 2] =
            [(0, "Ip", &plate), (1, "Ig2", &screen)];

        for (row, name, f) in row_specs {
            for col in 0..3 {
                let analytic = jac[row][col];
                let numerical = fd(f, col);
                let rel_err = if numerical.abs() > 1e-15 {
                    (analytic - numerical).abs() / numerical.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 1e-3,
                    "d{}/dV[{}] mismatch at (Vgk={}, Vpk={}, Vg2k={}): \
                     analytic={:.6e} fd={:.6e} rel_err={:.2e}",
                    name,
                    col,
                    vgk,
                    vpk,
                    vg2k,
                    analytic,
                    numerical,
                    rel_err
                );
            }
        }

        // Ig1 row[2] checks: at Vgk=-5 the grid is reverse-biased, so the
        // entire row is hardcoded zero. Verify that and skip Vpk/Vg2k cols.
        assert_eq!(jac[2][0], 0.0, "Ig1 row should be zero at Vgk=-5");
        assert_eq!(jac[2][1], 0.0, "dIg1/dVpk is structurally zero");
        assert_eq!(jac[2][2], 0.0, "dIg1/dVg2k is structurally zero");

        // And finally a positive-grid sanity check for the Ig1 dVgk entry.
        let vgk_pos = 0.5;
        let jac_pos = pent.jacobian_3x3(vgk_pos, vpk, vg2k);
        let fd_ig1 = (pent.grid_current(vgk_pos + eps) - pent.grid_current(vgk_pos - eps))
            / (2.0 * eps);
        let rel_err = (jac_pos[2][0] - fd_ig1).abs() / fd_ig1.abs();
        assert!(
            rel_err < 1e-3,
            "dIg1/dVgk mismatch at Vgk={}: analytic={:.6e} fd={:.6e}",
            vgk_pos,
            jac_pos[2][0],
            fd_ig1
        );
    }

    #[test]
    fn test_pentode_negative_vg2k_safe() {
        let pent = KorenPentode::el84();
        let ip = pent.plate_current(-5.0, 300.0, -0.5);
        let ig2 = pent.screen_current(-5.0, 300.0, -0.5);
        let jac = pent.jacobian_3x3(-5.0, 300.0, -0.5);
        assert!(ip.is_finite() && !ip.is_nan(), "Ip must be finite: {}", ip);
        assert!(
            ig2.is_finite() && !ig2.is_nan(),
            "Ig2 must be finite: {}",
            ig2
        );
        for row in 0..3 {
            for col in 0..3 {
                assert!(
                    jac[row][col].is_finite() && !jac[row][col].is_nan(),
                    "jacobian_3x3[{}][{}] must be finite: {}",
                    row,
                    col,
                    jac[row][col]
                );
            }
        }
    }

    #[test]
    fn test_pentode_negative_vpk_safe() {
        let pent = KorenPentode::el84();
        let ip = pent.plate_current(-5.0, -10.0, 300.0);
        let ig2 = pent.screen_current(-5.0, -10.0, 300.0);
        let jac = pent.jacobian_3x3(-5.0, -10.0, 300.0);
        assert!(ip.is_finite() && !ip.is_nan(), "Ip must be finite: {}", ip);
        assert!(
            ig2.is_finite() && !ig2.is_nan(),
            "Ig2 must be finite: {}",
            ig2
        );
        for row in 0..3 {
            for col in 0..3 {
                assert!(
                    jac[row][col].is_finite() && !jac[row][col].is_nan(),
                    "jacobian_3x3[{}][{}] must be finite: {}",
                    row,
                    col,
                    jac[row][col]
                );
            }
        }
    }

    #[test]
    fn test_grid_current_model() {
        let tube = KorenTriode::ecc83();
        assert_eq!(tube.grid_current(-1.0), 0.0);
        assert_eq!(tube.grid_current(0.0), 0.0);

        let ig = tube.grid_current(0.5);
        assert!(ig > 0.0);
        assert!((ig - tube.ig_max).abs() < 1e-10);

        let ig2 = tube.grid_current(1.0);
        assert!(ig2 > ig);
    }

    #[test]
    fn test_grid_current_jacobian_finite_difference() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        for &vgk in &[0.1, 0.3, 0.5, 1.0, 2.0] {
            let jac = tube.grid_current_jacobian(vgk);
            let fd = (tube.grid_current(vgk + eps) - tube.grid_current(vgk - eps)) / (2.0 * eps);
            let rel_err = if fd.abs() > 1e-15 {
                (jac - fd).abs() / fd.abs()
            } else {
                jac.abs()
            };
            assert!(
                rel_err < 1e-4,
                "dIg/dVgk mismatch at Vgk={}: analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                jac,
                fd,
                rel_err
            );
        }
        assert_eq!(tube.grid_current_jacobian(0.0), 0.0);
        assert_eq!(tube.grid_current_jacobian(-1.0), 0.0);
    }

    /// Verify Jacobian is bounded for fractional exponents (ex < 1.0).
    /// Previously e1.powf(ex-1.0) produced huge values near the guard threshold.
    #[test]
    fn test_fractional_exponent_jacobian_bounded() {
        // ex=0.5 means e1^(ex-1) = e1^(-0.5) which diverges for small e1
        let tube = KorenTriode::new(100.0, 0.5, 1060.0, 600.0, 300.0);

        // Test at low Vpk where e1 is very small
        for &vpk in &[0.1, 1.0, 5.0, 10.0] {
            let jac = tube.jacobian(&[-3.0, vpk]);
            assert!(jac[0].is_finite(), "dIp/dVgk must be finite at vpk={}", vpk);
            assert!(jac[1].is_finite(), "dIp/dVpk must be finite at vpk={}", vpk);
            assert!(
                jac[0].abs() < 1e6,
                "dIp/dVgk must be bounded at vpk={}, got {:.2e}",
                vpk,
                jac[0]
            );
            assert!(
                jac[1].abs() < 1e6,
                "dIp/dVpk must be bounded at vpk={}, got {:.2e}",
                vpk,
                jac[1]
            );
        }

        // Normal operating point should still work
        let jac = tube.jacobian(&[0.0, 250.0]);
        assert!(jac[0] > 0.0, "Transconductance should be positive");
        assert!(jac[1] > 0.0, "Output conductance should be positive");
    }

    /// Verify lambda multiplier increases plate current proportional to Vpk.
    #[test]
    fn test_lambda_early_effect() {
        let tube_no_lambda =
            KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.0);
        let tube_with_lambda =
            KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.001);

        let vgk = -1.0;
        let vpk = 250.0;
        let ip_without = tube_no_lambda.plate_current(vgk, vpk);
        let ip_with = tube_with_lambda.plate_current(vgk, vpk);

        let expected_ratio = 1.0 + 0.001 * vpk;
        let actual_ratio = ip_with / ip_without;
        assert!(
            (actual_ratio - expected_ratio).abs() < 1e-10,
            "Lambda should multiply by (1+lambda*Vpk): ratio={:.6e}, expected={:.6e}",
            actual_ratio,
            expected_ratio
        );
    }

    /// Verify lambda=0.0 is backward compatible.
    #[test]
    fn test_lambda_zero_backward_compat() {
        let tube_old = KorenTriode::new(100.0, 1.4, 1060.0, 600.0, 300.0);
        let tube_new =
            KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.0);

        for &(vgk, vpk) in &[(0.0, 250.0), (-1.0, 200.0), (-2.0, 100.0)] {
            let ip_old = tube_old.plate_current(vgk, vpk);
            let ip_new = tube_new.plate_current(vgk, vpk);
            assert_eq!(
                ip_old, ip_new,
                "lambda=0.0 should be backward compatible at ({}, {})",
                vgk, vpk
            );
        }
    }

    /// Verify plate current and Jacobian with positive grid voltage (grid current region).
    #[test]
    fn test_triode_grid_current_region_fd_jacobian() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        // Grid current region: Vgk > 0
        let grid_positive_points: &[(f64, f64, &str)] = &[
            (0.5, 250.0, "Vgk=0.5 moderate grid current"),
            (1.0, 250.0, "Vgk=1.0 strong grid current"),
            (2.0, 200.0, "Vgk=2.0 heavy grid current"),
            (0.1, 300.0, "Vgk=0.1 onset of grid current"),
        ];

        for &(vgk, vpk, desc) in grid_positive_points {
            // Check plate current Jacobian
            let jac = tube.jacobian(&[vgk, vpk]);

            let fd_dvgk = (tube.plate_current(vgk + eps, vpk) - tube.plate_current(vgk - eps, vpk))
                / (2.0 * eps);
            let fd_dvpk = (tube.plate_current(vgk, vpk + eps) - tube.plate_current(vgk, vpk - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in
                [("dIp/dVgk", jac[0], fd_dvgk), ("dIp/dVpk", jac[1], fd_dvpk)]
            {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "Grid current region {} at {} (Vgk={}, Vpk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name, desc, vgk, vpk, analytic, fd, rel_err
                );
            }

            // Also check grid current Jacobian
            let ig_jac = tube.grid_current_jacobian(vgk);
            let ig_fd = (tube.grid_current(vgk + eps) - tube.grid_current(vgk - eps)) / (2.0 * eps);
            let ig_err = if ig_fd.abs() > 1e-15 {
                (ig_jac - ig_fd).abs() / ig_fd.abs()
            } else {
                ig_jac.abs()
            };
            assert!(
                ig_err < 0.01,
                "Grid Ig Jacobian at {} (Vgk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                desc,
                vgk,
                ig_jac,
                ig_fd,
                ig_err
            );

            // Grid current should be positive for Vgk > 0
            let ig = tube.grid_current(vgk);
            assert!(
                ig > 0.0,
                "Grid current should be positive at Vgk={}: {:.6e}",
                vgk,
                ig
            );
        }
    }

    /// Verify Jacobian at low plate voltage where E1 is small.
    #[test]
    fn test_triode_low_plate_voltage_fd_jacobian() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        let low_vpk_points: &[(f64, f64, &str)] = &[
            (0.0, 5.0, "Vgk=0 Vpk=5"),
            (0.0, 10.0, "Vgk=0 Vpk=10"),
            (-1.0, 5.0, "Vgk=-1 Vpk=5"),
            (-1.0, 20.0, "Vgk=-1 Vpk=20"),
            (0.0, 1.0, "Vgk=0 Vpk=1"),
        ];

        for &(vgk, vpk, desc) in low_vpk_points {
            let jac = tube.jacobian(&[vgk, vpk]);

            // Both Jacobian entries must be finite
            assert!(
                jac[0].is_finite(),
                "dIp/dVgk must be finite at {} (Vgk={}, Vpk={}): {}",
                desc,
                vgk,
                vpk,
                jac[0]
            );
            assert!(
                jac[1].is_finite(),
                "dIp/dVpk must be finite at {} (Vgk={}, Vpk={}): {}",
                desc,
                vgk,
                vpk,
                jac[1]
            );

            let fd_dvgk = (tube.plate_current(vgk + eps, vpk) - tube.plate_current(vgk - eps, vpk))
                / (2.0 * eps);
            let fd_dvpk = (tube.plate_current(vgk, vpk + eps) - tube.plate_current(vgk, vpk - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in
                [("dIp/dVgk", jac[0], fd_dvgk), ("dIp/dVpk", jac[1], fd_dvpk)]
            {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "Low Vpk {} at {} (Vgk={}, Vpk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgk,
                    vpk,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }
    }

    /// Verify Jacobian near cutoff where plate current approaches zero.
    #[test]
    fn test_triode_near_cutoff_fd_jacobian() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        // Near-cutoff: very negative Vgk where Ip is very small but nonzero
        let cutoff_points: &[(f64, f64, &str)] = &[
            (-3.0, 250.0, "Vgk=-3 near cutoff"),
            (-3.5, 250.0, "Vgk=-3.5 deep cutoff"),
            (-2.5, 150.0, "Vgk=-2.5 low Vpk"),
            (-4.0, 300.0, "Vgk=-4 very deep cutoff"),
        ];

        for &(vgk, vpk, desc) in cutoff_points {
            let ip = tube.plate_current(vgk, vpk);
            let jac = tube.jacobian(&[vgk, vpk]);

            // Plate current should be very small but non-negative
            assert!(
                ip >= 0.0,
                "Ip should be non-negative near cutoff at {}: {:.6e}",
                desc,
                ip
            );
            assert!(ip.is_finite(), "Ip must be finite at {}", desc);

            // Jacobian must be finite
            assert!(
                jac[0].is_finite(),
                "dIp/dVgk must be finite at {}: {}",
                desc,
                jac[0]
            );
            assert!(
                jac[1].is_finite(),
                "dIp/dVpk must be finite at {}: {}",
                desc,
                jac[1]
            );

            let fd_dvgk = (tube.plate_current(vgk + eps, vpk) - tube.plate_current(vgk - eps, vpk))
                / (2.0 * eps);
            let fd_dvpk = (tube.plate_current(vgk, vpk + eps) - tube.plate_current(vgk, vpk - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in
                [("dIp/dVgk", jac[0], fd_dvgk), ("dIp/dVpk", jac[1], fd_dvpk)]
            {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "Near-cutoff {} at {} (Vgk={}, Vpk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgk,
                    vpk,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }
    }

    /// Verify Jacobian with lambda against finite differences.
    #[test]
    fn test_jacobian_with_lambda_finite_difference() {
        let tube = KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.001);
        let eps = 1e-6;

        for &(vgk, vpk) in &[(0.0, 250.0), (-1.0, 200.0), (-2.0, 100.0), (-0.5, 300.0)] {
            let jac = tube.jacobian(&[vgk, vpk]);

            let ip_plus = tube.plate_current(vgk + eps, vpk);
            let ip_minus = tube.plate_current(vgk - eps, vpk);
            let fd_dvgk = (ip_plus - ip_minus) / (2.0 * eps);

            let ip_plus = tube.plate_current(vgk, vpk + eps);
            let ip_minus = tube.plate_current(vgk, vpk - eps);
            let fd_dvpk = (ip_plus - ip_minus) / (2.0 * eps);

            let rel_err_gk = if fd_dvgk.abs() > 1e-15 {
                (jac[0] - fd_dvgk).abs() / fd_dvgk.abs()
            } else {
                jac[0].abs()
            };
            let rel_err_pk = if fd_dvpk.abs() > 1e-15 {
                (jac[1] - fd_dvpk).abs() / fd_dvpk.abs()
            } else {
                jac[1].abs()
            };

            assert!(
                rel_err_gk < 1e-4,
                "dIp/dVgk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[0],
                fd_dvgk,
                rel_err_gk
            );
            assert!(
                rel_err_pk < 1e-4,
                "dIp/dVpk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[1],
                fd_dvpk,
                rel_err_pk
            );
        }
    }

    // ---------------------------------------------------------------
    // Reefman §4.5 DerkE (Exponential screen form) — beam tetrodes
    // ---------------------------------------------------------------

    #[test]
    fn test_6l6gc_operating_point() {
        // Class-A bias for a 6L6GC: Vgk=-20, Vpk=300, Vg2k=250.
        let tet = KorenPentode::tetrode_6l6gc();
        assert_eq!(tet.screen_form, ScreenForm::Exponential);

        let ip = tet.plate_current(-20.0, 300.0, 250.0);
        let ig2 = tet.screen_current(-20.0, 300.0, 250.0);

        assert!(ip.is_finite() && !ip.is_nan(), "6L6GC Ip must be finite: {}", ip);
        assert!(
            ig2.is_finite() && !ig2.is_nan(),
            "6L6GC Ig2 must be finite: {}",
            ig2
        );
        assert!(ip > 0.0, "6L6GC Ip must be positive at Class A: {:.6e}", ip);
        assert!(
            ig2 > 0.0,
            "6L6GC Ig2 must be positive at Class A: {:.6e}",
            ig2
        );
        assert!(
            ip > ig2,
            "Beam tetrode Ip must exceed Ig2 in Class A: Ip={:.3e}, Ig2={:.3e}",
            ip,
            ig2
        );
    }

    #[test]
    fn test_6v6gt_operating_point() {
        // Class-A bias for a 6V6GT: Vgk=-12, Vpk=300, Vg2k=250.
        let tet = KorenPentode::tetrode_6v6gt();
        assert_eq!(tet.screen_form, ScreenForm::Exponential);

        let ip = tet.plate_current(-12.0, 300.0, 250.0);
        let ig2 = tet.screen_current(-12.0, 300.0, 250.0);

        assert!(ip.is_finite() && !ip.is_nan(), "6V6GT Ip must be finite: {}", ip);
        assert!(
            ig2.is_finite() && !ig2.is_nan(),
            "6V6GT Ig2 must be finite: {}",
            ig2
        );
        assert!(ip > 0.0, "6V6GT Ip must be positive at Class A: {:.6e}", ip);
        assert!(
            ig2 > 0.0,
            "6V6GT Ig2 must be positive at Class A: {:.6e}",
            ig2
        );
        assert!(
            ip > ig2,
            "Beam tetrode Ip must exceed Ig2 in Class A: Ip={:.3e}, Ig2={:.3e}",
            ip,
            ig2
        );
    }

    #[test]
    fn test_6l6gc_jacobian_fd() {
        // Finite-difference check of analytic 3x3 for DerkE form at a
        // moderate Class-A point that's well away from the E1 guard.
        let tet = KorenPentode::tetrode_6l6gc();
        let eps = 1e-6;

        let (vgk, vpk, vg2k) = (-15.0, 300.0, 250.0);
        let jac = tet.jacobian_3x3(vgk, vpk, vg2k);

        let plate = |a: f64, b: f64, c: f64| tet.plate_current(a, b, c);
        let screen = |a: f64, b: f64, c: f64| tet.screen_current(a, b, c);

        let fd = |f: &dyn Fn(f64, f64, f64) -> f64, dim: usize| -> f64 {
            let mut p = [vgk, vpk, vg2k];
            let mut m = [vgk, vpk, vg2k];
            p[dim] += eps;
            m[dim] -= eps;
            (f(p[0], p[1], p[2]) - f(m[0], m[1], m[2])) / (2.0 * eps)
        };

        let row_specs: [(usize, &str, &dyn Fn(f64, f64, f64) -> f64); 2] =
            [(0, "Ip", &plate), (1, "Ig2", &screen)];

        for (row, name, f) in row_specs {
            for col in 0..3 {
                let analytic = jac[row][col];
                let numerical = fd(f, col);
                let rel_err = if numerical.abs() > 1e-15 {
                    (analytic - numerical).abs() / numerical.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 1e-3,
                    "6L6GC d{}/dV[{}] mismatch at (Vgk={}, Vpk={}, Vg2k={}): \
                     analytic={:.6e} fd={:.6e} rel_err={:.2e}",
                    name,
                    col,
                    vgk,
                    vpk,
                    vg2k,
                    analytic,
                    numerical,
                    rel_err
                );
            }
        }

        // Ig1 row: Vgk=-15 is reverse-biased, so the full row should be zero.
        assert_eq!(jac[2][0], 0.0);
        assert_eq!(jac[2][1], 0.0);
        assert_eq!(jac[2][2], 0.0);
    }

    #[test]
    fn test_derke_cutoff_smoothness() {
        // Near Vpk=0 the (β·Vp)^{3/2} term and its sqrt(β·Vp) derivative
        // must remain finite for DerkE. This is the main risk point of
        // the §4.5 form.
        let tet = KorenPentode::tetrode_6l6gc();

        // Sweep Vpk through the small-Vp region including exactly zero
        // and a slightly-negative probe (which gets clamped to 0 internally).
        for &vpk in &[-0.5_f64, 0.0, 0.5, 1e-9, 1e-6, 1.0] {
            let ip = tet.plate_current(-15.0, vpk, 250.0);
            let ig2 = tet.screen_current(-15.0, vpk, 250.0);
            let jac = tet.jacobian_3x3(-15.0, vpk, 250.0);

            assert!(
                ip.is_finite() && !ip.is_nan(),
                "DerkE Ip must be finite at Vpk={}: {}",
                vpk,
                ip
            );
            assert!(
                ig2.is_finite() && !ig2.is_nan(),
                "DerkE Ig2 must be finite at Vpk={}: {}",
                vpk,
                ig2
            );
            for row in 0..3 {
                for col in 0..3 {
                    assert!(
                        jac[row][col].is_finite() && !jac[row][col].is_nan(),
                        "DerkE jacobian_3x3[{}][{}] must be finite at Vpk={}: {}",
                        row,
                        col,
                        vpk,
                        jac[row][col]
                    );
                }
            }
        }
    }

    #[test]
    fn test_derke_rational_distinct() {
        // Build two pentodes with IDENTICAL Koren parameters that differ
        // only in screen_form. At a non-zero Vpk the two shape functions
        // must diverge, confirming the branch is actually taken.
        let base = KorenPentode::tetrode_6l6gc();
        let mut rational_clone = base;
        rational_clone.screen_form = ScreenForm::Rational;

        let (vgk, vpk, vg2k) = (-15.0, 250.0, 250.0);
        let ip_e = base.plate_current(vgk, vpk, vg2k);
        let ig2_e = base.screen_current(vgk, vpk, vg2k);
        let ip_r = rational_clone.plate_current(vgk, vpk, vg2k);
        let ig2_r = rational_clone.screen_current(vgk, vpk, vg2k);

        assert!(ip_e.is_finite() && ip_r.is_finite());
        assert!(ig2_e.is_finite() && ig2_r.is_finite());

        let ip_rel = (ip_e - ip_r).abs() / ip_e.abs().max(ip_r.abs()).max(1e-20);
        let ig2_rel = (ig2_e - ig2_r).abs() / ig2_e.abs().max(ig2_r.abs()).max(1e-20);

        assert!(
            ip_rel > 1e-3,
            "DerkE and Rational Ip should differ at Vpk={}: Ip_e={:.6e} Ip_r={:.6e} rel={:.2e}",
            vpk,
            ip_e,
            ip_r,
            ip_rel
        );
        assert!(
            ig2_rel > 1e-3,
            "DerkE and Rational Ig2 should differ at Vpk={}: Ig2_e={:.6e} Ig2_r={:.6e} rel={:.2e}",
            vpk,
            ig2_e,
            ig2_r,
            ig2_rel
        );

        // And at Vpk=0, the two forms should agree (both scale factors → 1).
        let ip_e0 = base.plate_current(vgk, 0.0, vg2k);
        let ip_r0 = rational_clone.plate_current(vgk, 0.0, vg2k);
        let rel0 = (ip_e0 - ip_r0).abs() / ip_e0.abs().max(1e-20);
        assert!(
            rel0 < 1e-12,
            "At Vpk=0 both forms should agree (scale=1): Ip_e={:.6e} Ip_r={:.6e} rel={:.2e}",
            ip_e0,
            ip_r0,
            rel0
        );
    }

    // -----------------------------------------------------------------
    // Classical Norman Koren pentode (phase 1a.2) — KT88 tests.
    //
    // Cohen-Hélie 2010 DAFx Eqs 1-3, Table 2 KT88 parameters. Gate
    // numerical bounds wide enough to tolerate Koren's known ~2-3× bias
    // overestimate (acknowledged in the paper and confirmed against KT88
    // datasheet Ia-Vak curves).
    // -----------------------------------------------------------------

    #[test]
    fn test_classical_kt88_operating_point() {
        let tube = KorenPentode::kt88();
        assert_eq!(tube.screen_form, ScreenForm::Classical);

        // Typical Class AB bias: Vgk=-20, Vpk=350, Vg2k=300. Sanity
        // check the math: 78 mA back-of-envelope calculation.
        let vgk = -20.0;
        let vpk = 350.0;
        let vg2k = 300.0;
        let ip = tube.plate_current(vgk, vpk, vg2k);
        let ig2 = tube.screen_current(vgk, vpk, vg2k);

        assert!(ip.is_finite() && ig2.is_finite());
        assert!(
            ip > 10e-3 && ip < 250e-3,
            "KT88 Ip(Vgk=-20, Vpk=350, Vg2k=300) = {:.2} mA, expected 10-250 mA",
            ip * 1000.0
        );
        assert!(
            ig2 > 0.5e-3 && ig2 < 20e-3,
            "KT88 Ig2(Vgk=-20, Vpk=350, Vg2k=300) = {:.2} mA, expected 0.5-20 mA",
            ig2 * 1000.0
        );
        assert!(
            ip > ig2,
            "Plate current must exceed screen current in normal operation: Ip={:.2}mA Ig2={:.2}mA",
            ip * 1000.0,
            ig2 * 1000.0,
        );
    }

    #[test]
    fn test_classical_kt88_deep_cutoff() {
        // KT88 has μ=8.8, Kp=32 — the Classical softplus argument
        // `Kp·(1/μ + Vgk/Vg2k)` only becomes strongly negative when
        // Vgk is several times larger (in magnitude) than Vg2k/μ.
        // At Vgk=-100, Vg2k=300: inner ≈ 32·(0.114 − 0.333) ≈ −7.0
        // → softplus ≈ 9e-4 → Ip ≈ 7 μA, Ig2 (gated on x>0) = 0.
        let tube = KorenPentode::kt88();
        let ip = tube.plate_current(-100.0, 350.0, 300.0);
        let ig2 = tube.screen_current(-100.0, 350.0, 300.0);
        assert!(
            ip < 10e-6,
            "KT88 deep cutoff Ip = {:.3e} A, expected < 10 μA",
            ip
        );
        assert!(
            ig2 < 10e-6,
            "KT88 deep cutoff Ig2 = {:.3e} A, expected < 10 μA",
            ig2
        );
    }

    #[test]
    fn test_classical_kt88_monotonic_in_vgk() {
        let tube = KorenPentode::kt88();
        let vpk = 350.0;
        let vg2k = 300.0;
        let mut prev = tube.plate_current(-40.0, vpk, vg2k);
        for i in 1..=10 {
            // -40 → -5 in 10 steps
            let vgk = -40.0 + (35.0 * i as f64 / 10.0);
            let ip = tube.plate_current(vgk, vpk, vg2k);
            assert!(
                ip > prev,
                "Ip not monotonic in Vgk at Vgk={}: prev={:.3e} current={:.3e}",
                vgk,
                prev,
                ip
            );
            prev = ip;
        }
    }

    #[test]
    fn test_classical_kt88_monotonic_in_vpk() {
        let tube = KorenPentode::kt88();
        let vgk = -20.0;
        let vg2k = 300.0;
        let mut prev = tube.plate_current(vgk, 10.0, vg2k);
        for i in 1..=10 {
            // 10 → 500 in 10 steps
            let vpk = 10.0 + (490.0 * i as f64 / 10.0);
            let ip = tube.plate_current(vgk, vpk, vg2k);
            assert!(
                ip > prev,
                "Ip not monotonic in Vpk at Vpk={}: prev={:.3e} current={:.3e}",
                vpk,
                prev,
                ip
            );
            prev = ip;
        }
    }

    #[test]
    fn test_classical_kt88_ig2_vp_independent() {
        // Defining feature of the Classical Koren path: screen current
        // depends only on Vgk / Vg2k, not Vpk. Three Vpk values must
        // produce numerically IDENTICAL Ig2.
        let tube = KorenPentode::kt88();
        let vgk = -20.0;
        let vg2k = 300.0;
        let ig2_100 = tube.screen_current(vgk, 100.0, vg2k);
        let ig2_250 = tube.screen_current(vgk, 250.0, vg2k);
        let ig2_500 = tube.screen_current(vgk, 500.0, vg2k);
        assert_eq!(
            ig2_100, ig2_250,
            "Ig2 must be Vp-independent: Vpk=100 → {:.6e}, Vpk=250 → {:.6e}",
            ig2_100, ig2_250
        );
        assert_eq!(
            ig2_250, ig2_500,
            "Ig2 must be Vp-independent: Vpk=250 → {:.6e}, Vpk=500 → {:.6e}",
            ig2_250, ig2_500
        );
    }

    #[test]
    fn test_classical_kt88_jacobian_fd() {
        let tube = KorenPentode::kt88();
        let eps = 1e-4;
        let (vgk, vpk, vg2k) = (-15.0, 300.0, 300.0);

        let jac = tube.jacobian_3x3(vgk, vpk, vg2k);

        // Central differences for each of the 2 currents (Ip, Ig2)
        // against each of the 3 voltages (Vgk, Vpk, Vg2k). Ig1 row is
        // excluded per the Derk test convention — the Leach power-law
        // has its own dedicated test elsewhere.
        let ip_plus = |dgk: f64, dpk: f64, dg2k: f64| {
            tube.plate_current(vgk + dgk, vpk + dpk, vg2k + dg2k)
        };
        let ig2_plus = |dgk: f64, dpk: f64, dg2k: f64| {
            tube.screen_current(vgk + dgk, vpk + dpk, vg2k + dg2k)
        };

        let fd_ip_vgk = (ip_plus(eps, 0.0, 0.0) - ip_plus(-eps, 0.0, 0.0)) / (2.0 * eps);
        let fd_ip_vpk = (ip_plus(0.0, eps, 0.0) - ip_plus(0.0, -eps, 0.0)) / (2.0 * eps);
        let fd_ip_vg2k = (ip_plus(0.0, 0.0, eps) - ip_plus(0.0, 0.0, -eps)) / (2.0 * eps);

        let fd_ig2_vgk = (ig2_plus(eps, 0.0, 0.0) - ig2_plus(-eps, 0.0, 0.0)) / (2.0 * eps);
        let fd_ig2_vpk = (ig2_plus(0.0, eps, 0.0) - ig2_plus(0.0, -eps, 0.0)) / (2.0 * eps);
        let fd_ig2_vg2k = (ig2_plus(0.0, 0.0, eps) - ig2_plus(0.0, 0.0, -eps)) / (2.0 * eps);

        let check = |name: &str, analytic: f64, fd: f64| {
            let rel_err = if fd.abs() > 1e-15 {
                (analytic - fd).abs() / fd.abs()
            } else {
                analytic.abs()
            };
            assert!(
                rel_err < 1e-3,
                "{}: analytic={:.6e} fd={:.6e} rel_err={:.2e}",
                name,
                analytic,
                fd,
                rel_err
            );
        };

        check("dIp/dVgk", jac[0][0], fd_ip_vgk);
        check("dIp/dVpk", jac[0][1], fd_ip_vpk);
        check("dIp/dVg2k", jac[0][2], fd_ip_vg2k);
        check("dIg2/dVgk", jac[1][0], fd_ig2_vgk);
        // [1][1] is structurally 0 and FD should also give 0 to machine precision.
        assert_eq!(
            jac[1][1], 0.0,
            "Ig2 Vp-independence → dIg2/dVpk must be exactly 0, got {:.6e}",
            jac[1][1]
        );
        assert!(
            fd_ig2_vpk.abs() < 1e-12,
            "FD dIg2/dVpk should vanish to machine precision, got {:.6e}",
            fd_ig2_vpk
        );
        check("dIg2/dVg2k", jac[1][2], fd_ig2_vg2k);
    }

    #[test]
    fn test_classical_kt88_safety_guards() {
        let tube = KorenPentode::kt88();

        // Case 1: Vpk < 0 AND Vg2k < 0 (NR probing pathological region).
        let ip1 = tube.plate_current(-10.0, -5.0, -2.0);
        let ig2_1 = tube.screen_current(-10.0, -5.0, -2.0);
        let jac1 = tube.jacobian_3x3(-10.0, -5.0, -2.0);
        assert!(ip1.is_finite() && !ip1.is_nan());
        assert!(ig2_1.is_finite() && !ig2_1.is_nan());
        for row in &jac1 {
            for &v in row {
                assert!(
                    v.is_finite() && !v.is_nan(),
                    "Classical Jacobian produced non-finite entry at pathological input"
                );
            }
        }

        // Case 2: very low positive Vg2k (near the 1e-3 clamp).
        let ip2 = tube.plate_current(-30.0, 250.0, 0.5);
        let ig2_2 = tube.screen_current(-30.0, 250.0, 0.5);
        let jac2 = tube.jacobian_3x3(-30.0, 250.0, 0.5);
        assert!(ip2.is_finite() && !ip2.is_nan());
        assert!(ig2_2.is_finite() && !ig2_2.is_nan());
        for row in &jac2 {
            for &v in row {
                assert!(
                    v.is_finite() && !v.is_nan(),
                    "Classical Jacobian produced non-finite entry at low Vg2k"
                );
            }
        }
    }
}
