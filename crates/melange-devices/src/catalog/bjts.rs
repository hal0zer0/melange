//! BJT catalog entries with SPICE model parameters.
//!
//! Parameters sourced from manufacturer SPICE models and datasheets.

/// A catalog entry for a BJT.
#[derive(Debug, Clone, Copy)]
pub struct BjtCatalogEntry {
    /// Part number aliases (e.g., ["2N2222", "2N2222A", "PN2222A"])
    pub names: &'static [&'static str],
    /// Saturation current IS [A]
    pub is: f64,
    /// Thermal voltage VT [V]
    pub vt: f64,
    /// Forward current gain BF
    pub beta_f: f64,
    /// Reverse current gain BR
    pub beta_r: f64,
    /// True if PNP
    pub is_pnp: bool,
    /// Forward Early voltage [V] (infinity = pure Ebers-Moll)
    pub vaf: f64,
    /// Reverse Early voltage [V]
    pub var: f64,
    /// Forward high-injection knee current [A]
    pub ikf: f64,
    /// Reverse high-injection knee current [A]
    pub ikr: f64,
    /// Source citation
    pub source: &'static str,
}

/// The BJT catalog.
pub const CATALOG: &[BjtCatalogEntry] = &[
    // 2N2222 / 2N2222A / PN2222A — NPN general purpose
    // ON Semi SPICE model: IS=1.26e-14, BF=200, BR=3, VAF=100
    BjtCatalogEntry {
        names: &["2N2222", "2N2222A", "PN2222A"],
        is: 1.26e-14,
        vt: 0.025851991,
        beta_f: 200.0,
        beta_r: 3.0,
        is_pnp: false,
        vaf: 100.0,
        var: 10.0,
        ikf: 0.3,
        ikr: 0.006,
        source: "ON Semiconductor SPICE model",
    },
    // 2N3904 — NPN general purpose
    // ON Semi SPICE model: IS=6.73e-15, BF=416, BR=0.737, VAF=74.03
    BjtCatalogEntry {
        names: &["2N3904"],
        is: 6.73e-15,
        vt: 0.025851991,
        beta_f: 416.0,
        beta_r: 0.737,
        is_pnp: false,
        vaf: 74.03,
        var: 28.0,
        ikf: 0.0667,
        ikr: 1.2e-4,
        source: "ON Semiconductor SPICE model",
    },
    // 2N3906 — PNP general purpose
    // ON Semi SPICE model: IS=1.27e-14, BF=207, BR=1.68, VAF=115.7
    BjtCatalogEntry {
        names: &["2N3906"],
        is: 1.27e-14,
        vt: 0.025851991,
        beta_f: 207.0,
        beta_r: 1.68,
        is_pnp: true,
        vaf: 115.7,
        var: 18.0,
        ikf: 0.0708,
        ikr: 3.0e-4,
        source: "ON Semiconductor SPICE model",
    },
    // 2N5088 / 2N5089 — NPN high-gain, low-noise (used in fuzz/boost pedals)
    BjtCatalogEntry {
        names: &["2N5088", "2N5089"],
        is: 4.8e-14,
        vt: 0.025851991,
        beta_f: 600.0,
        beta_r: 3.5,
        is_pnp: false,
        vaf: 90.0,
        var: f64::INFINITY,
        ikf: 0.04,
        ikr: f64::INFINITY,
        source: "ON Semiconductor / Central Semi SPICE model",
    },
    // BC547B / BC547C — NPN small signal (European standard)
    BjtCatalogEntry {
        names: &["BC547B", "BC547C"],
        is: 1.8e-14,
        vt: 0.025851991,
        beta_f: 400.0,
        beta_r: 35.0,
        is_pnp: false,
        vaf: 80.0,
        var: 12.5,
        ikf: 0.1,
        ikr: 3.0e-4,
        source: "Philips/NXP SPICE model",
    },
    // BC557B — PNP small signal (complement of BC547)
    BjtCatalogEntry {
        names: &["BC557B"],
        is: 2.0e-14,
        vt: 0.025851991,
        beta_f: 330.0,
        beta_r: 10.0,
        is_pnp: true,
        vaf: 69.0,
        var: 12.0,
        ikf: 0.08,
        ikr: 2.0e-4,
        source: "Philips/NXP SPICE model",
    },
    // AC128 — PNP germanium (vintage fuzz: Fuzz Face, Tone Bender)
    // Germanium parameters estimated from measurements
    BjtCatalogEntry {
        names: &["AC128"],
        is: 1e-6,
        vt: 0.025851991,
        beta_f: 70.0,
        beta_r: 3.0,
        is_pnp: true,
        vaf: 30.0,
        var: f64::INFINITY,
        ikf: f64::INFINITY,
        ikr: f64::INFINITY,
        source: "Measured/estimated germanium parameters",
    },
];

/// Look up a BJT by part number (case-insensitive).
pub fn lookup(name: &str) -> Option<&'static BjtCatalogEntry> {
    CATALOG
        .iter()
        .find(|entry| entry.names.iter().any(|n| n.eq_ignore_ascii_case(name)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BjtEbersMoll, BjtPolarity};

    #[test]
    fn test_all_aliases_resolve() {
        for entry in CATALOG {
            for name in entry.names {
                assert!(lookup(name).is_some(), "Alias '{}' should resolve", name);
            }
        }
    }

    #[test]
    fn test_case_insensitive() {
        assert!(lookup("2n2222a").is_some());
        assert!(lookup("2N2222A").is_some());
        assert!(lookup("bc547b").is_some());
    }

    #[test]
    fn test_no_duplicate_names() {
        let mut all_names: Vec<&str> = Vec::new();
        for entry in CATALOG {
            for name in entry.names {
                assert!(
                    !all_names.iter().any(|n| n.eq_ignore_ascii_case(name)),
                    "Duplicate: {}",
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
                entry.is > 0.0 && entry.is.is_finite(),
                "{}: IS",
                entry.names[0]
            );
            assert!(
                entry.vt > 0.0 && entry.vt.is_finite(),
                "{}: VT",
                entry.names[0]
            );
            assert!(
                entry.beta_f > 0.0 && entry.beta_f.is_finite(),
                "{}: BF",
                entry.names[0]
            );
            assert!(
                entry.beta_r > 0.0 && entry.beta_r.is_finite(),
                "{}: BR",
                entry.names[0]
            );
            if entry.vaf.is_finite() {
                assert!(entry.vaf > 0.0, "{}: VAF", entry.names[0]);
            }
            if entry.var.is_finite() {
                assert!(entry.var > 0.0, "{}: VAR", entry.names[0]);
            }
            if entry.ikf.is_finite() {
                assert!(entry.ikf > 0.0, "{}: IKF", entry.names[0]);
            }
            if entry.ikr.is_finite() {
                assert!(entry.ikr > 0.0, "{}: IKR", entry.names[0]);
            }
        }
    }

    // --- Tier 1: Factory method consistency ---

    #[test]
    fn test_2n2222a_matches_catalog() {
        let factory = BjtEbersMoll::npn_2n2222a();
        let cat = lookup("2N2222A").unwrap();
        assert_eq!(factory.is, cat.is);
        assert_eq!(factory.beta_f, cat.beta_f);
        assert_eq!(factory.beta_r, cat.beta_r);
    }

    #[test]
    fn test_2n3904_matches_catalog() {
        let factory = BjtEbersMoll::npn_2n3904();
        let cat = lookup("2N3904").unwrap();
        assert_eq!(factory.is, cat.is);
        assert_eq!(factory.beta_f, cat.beta_f);
        assert_eq!(factory.beta_r, cat.beta_r);
    }

    #[test]
    fn test_2n3906_matches_catalog() {
        let factory = BjtEbersMoll::pnp_2n3906();
        let cat = lookup("2N3906").unwrap();
        assert_eq!(factory.is, cat.is);
        assert_eq!(factory.beta_f, cat.beta_f);
        assert_eq!(factory.beta_r, cat.beta_r);
    }

    #[test]
    fn test_ac128_matches_catalog() {
        let factory = BjtEbersMoll::pnp_ac128();
        let cat = lookup("AC128").unwrap();
        assert_eq!(factory.is, cat.is);
        assert_eq!(factory.beta_f, cat.beta_f);
        assert_eq!(factory.beta_r, cat.beta_r);
    }

    // --- Tier 2: Datasheet operating point verification ---

    fn make_bjt(entry: &BjtCatalogEntry) -> BjtEbersMoll {
        let polarity = if entry.is_pnp {
            BjtPolarity::Pnp
        } else {
            BjtPolarity::Npn
        };
        BjtEbersMoll::new(entry.is, entry.vt, entry.beta_f, entry.beta_r, polarity)
    }

    #[test]
    fn test_2n2222a_operating_points() {
        let b = make_bjt(lookup("2N2222A").unwrap());
        // ON Semi: forward active, Ic=1mA → Vbe ≈ 0.6V
        // At Vbe=0.6V, Vbc=0.6-10=-9.4V (forward active)
        let ic = b.collector_current(0.6, -9.4);
        assert!(
            ic > 0.1e-3 && ic < 10e-3,
            "2N2222A Ic at Vbe=0.6V: {:.3}mA",
            ic * 1e3
        );
        // Forward beta check
        let ib = b.base_current(0.6, -9.4);
        if ib > 1e-12 {
            let measured_beta = ic / ib;
            assert!(
                measured_beta > 50.0 && measured_beta < 500.0,
                "2N2222A beta={:.0} (expected ~200)",
                measured_beta
            );
        }
    }

    #[test]
    fn test_2n3904_operating_points() {
        let b = make_bjt(lookup("2N3904").unwrap());
        let ic = b.collector_current(0.6, -9.4);
        assert!(
            ic > 0.01e-3 && ic < 10e-3,
            "2N3904 Ic at Vbe=0.6V: {:.3}mA",
            ic * 1e3
        );
        let ib = b.base_current(0.6, -9.4);
        if ib > 1e-12 {
            let measured_beta = ic / ib;
            assert!(
                measured_beta > 100.0 && measured_beta < 800.0,
                "2N3904 beta={:.0} (expected ~416)",
                measured_beta
            );
        }
    }

    #[test]
    fn test_2n3906_operating_points() {
        let b = make_bjt(lookup("2N3906").unwrap());
        // PNP: Vbe=-0.6V, Vbc=-0.6-(-10)=9.4V → forward active
        let ic = b.collector_current(-0.6, 9.4);
        assert!(
            ic < -0.01e-3 && ic > -10e-3,
            "2N3906 Ic at Vbe=-0.6V: {:.3}mA",
            ic * 1e3
        );
    }

    #[test]
    fn test_ac128_germanium_operating_points() {
        let b = make_bjt(lookup("AC128").unwrap());
        // Germanium: lower Vbe (≈0.2V), higher Is
        // At Vbe=-0.2V (PNP), should have measurable current
        let ic = b.collector_current(-0.2, 9.0);
        assert!(
            ic < 0.0,
            "AC128 PNP should have negative Ic, got {:.6}mA",
            ic * 1e3
        );
    }

    #[test]
    fn test_2n5088_operating_points() {
        let b = make_bjt(lookup("2N5088").unwrap());
        let ic = b.collector_current(0.6, -9.4);
        assert!(ic > 0.0, "2N5088 should conduct at Vbe=0.6V");
        let ib = b.base_current(0.6, -9.4);
        if ib > 1e-12 {
            let measured_beta = ic / ib;
            assert!(
                measured_beta > 200.0,
                "2N5088 beta={:.0} (expected ~600)",
                measured_beta
            );
        }
    }

    #[test]
    fn test_bc547b_operating_points() {
        let b = make_bjt(lookup("BC547B").unwrap());
        let ic = b.collector_current(0.6, -4.4);
        assert!(ic > 0.0, "BC547B should conduct at Vbe=0.6V");
        let ib = b.base_current(0.6, -4.4);
        if ib > 1e-12 {
            let measured_beta = ic / ib;
            assert!(measured_beta > 100.0, "BC547B beta={:.0}", measured_beta);
        }
    }

    // --- Tier 3: Jacobian consistency ---

    use crate::NonlinearDevice;

    fn check_bjt_jacobian(entry: &BjtCatalogEntry) {
        let b = make_bjt(entry);
        let eps = 1e-6;
        let sign = if entry.is_pnp { -1.0 } else { 1.0 };
        for &vbe_mag in &[0.4, 0.5, 0.6] {
            let vbe = sign * vbe_mag;
            let vbc = sign * (vbe_mag - 10.0); // forward active
            let jac = b.jacobian(&[vbe, vbc]);
            let fd0 = (b.collector_current(vbe + eps, vbc) - b.collector_current(vbe - eps, vbc))
                / (2.0 * eps);
            let fd1 = (b.collector_current(vbe, vbc + eps) - b.collector_current(vbe, vbc - eps))
                / (2.0 * eps);
            if fd0.abs() > 1e-12 {
                let rel = (jac[0] - fd0).abs() / fd0.abs();
                assert!(
                    rel < 1e-3,
                    "{} dIc/dVbe: a={:.6e} fd={:.6e}",
                    entry.names[0],
                    jac[0],
                    fd0
                );
            }
            if fd1.abs() > 1e-12 {
                let rel = (jac[1] - fd1).abs() / fd1.abs();
                assert!(
                    rel < 1e-3,
                    "{} dIc/dVbc: a={:.6e} fd={:.6e}",
                    entry.names[0],
                    jac[1],
                    fd1
                );
            }
        }
    }

    #[test]
    fn test_2n2222a_jacobian() {
        check_bjt_jacobian(lookup("2N2222A").unwrap());
    }
    #[test]
    fn test_2n3904_jacobian() {
        check_bjt_jacobian(lookup("2N3904").unwrap());
    }
    #[test]
    fn test_2n3906_jacobian() {
        check_bjt_jacobian(lookup("2N3906").unwrap());
    }
    #[test]
    fn test_ac128_jacobian() {
        check_bjt_jacobian(lookup("AC128").unwrap());
    }
    #[test]
    fn test_2n5088_jacobian() {
        check_bjt_jacobian(lookup("2N5088").unwrap());
    }
    #[test]
    fn test_bc547b_jacobian() {
        check_bjt_jacobian(lookup("BC547B").unwrap());
    }
    #[test]
    fn test_bc557b_jacobian() {
        check_bjt_jacobian(lookup("BC557B").unwrap());
    }
}
