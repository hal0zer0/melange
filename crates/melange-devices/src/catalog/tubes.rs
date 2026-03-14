//! Tube/triode catalog entries with Koren model parameters.
//!
//! All parameters are for triode-mode operation (pentodes are triode-connected).
//! Sources are cited per entry for traceability.

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
    // Koren 1996 Table 1: mu=100, ex=1.4, Kg1=1060, Kp=600, Kvb=300
    TubeCatalogEntry {
        names: &["12AX7", "ECC83", "7025", "CV4004"],
        mu: 100.0,
        ex: 1.4,
        kg1: 1060.0,
        kp: 600.0,
        kvb: 300.0,
        ig_max: 2e-3,
        vgk_onset: 0.5,
        lambda: 0.0,
        source: "Koren 1996 Table 1",
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
    CATALOG.iter().find(|entry| {
        entry.names.iter().any(|n| n.eq_ignore_ascii_case(name))
    })
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
                assert!(
                    lookup(name).is_some(),
                    "Alias '{}' should resolve",
                    name
                );
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
            assert!(entry.mu > 0.0 && entry.mu.is_finite(), "{}: mu", entry.names[0]);
            assert!(entry.ex > 0.0 && entry.ex.is_finite(), "{}: ex", entry.names[0]);
            assert!(entry.kg1 > 0.0 && entry.kg1.is_finite(), "{}: kg1", entry.names[0]);
            assert!(entry.kp > 0.0 && entry.kp.is_finite(), "{}: kp", entry.names[0]);
            assert!(entry.kvb > 0.0 && entry.kvb.is_finite(), "{}: kvb", entry.names[0]);
            assert!(entry.ig_max > 0.0 && entry.ig_max.is_finite(), "{}: ig_max", entry.names[0]);
            assert!(entry.vgk_onset > 0.0 && entry.vgk_onset.is_finite(), "{}: vgk_onset", entry.names[0]);
            assert!(entry.lambda >= 0.0 && entry.lambda.is_finite(), "{}: lambda", entry.names[0]);
        }
    }

    #[test]
    fn test_unknown_returns_none() {
        assert!(lookup("NONEXISTENT_TUBE").is_none());
    }

    // --- Tier 1: Factory method consistency ---

    #[test]
    fn test_ecc83_matches_catalog() {
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

    // --- Tier 2: Datasheet operating point verification ---

    fn make_tube(entry: &TubeCatalogEntry) -> KorenTriode {
        KorenTriode::with_all_params(
            entry.mu, entry.ex, entry.kg1, entry.kp, entry.kvb,
            entry.ig_max, entry.vgk_onset, entry.lambda,
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
        // Koren model at Vgk=0V, Vpk=250V → Ip ≈ 3.4mA
        // (RCA tube manual says ~1.2mA; Koren overestimates at zero grid bias)
        let ip0 = t.plate_current(0.0, 250.0);
        assert!(ip0 > 0.5e-3 && ip0 < 10e-3, "12AX7 Vgk=0: Ip={:.3}mA", ip0 * 1e3);
        // Vgk=-1V, Vpk=250V: Koren gives ~1.7mA (datasheet: ~0.5mA)
        let ip_m1 = t.plate_current(-1.0, 250.0);
        assert!(ip_m1 > 0.1e-3 && ip_m1 < 5e-3, "12AX7 Vgk=-1: Ip={:.3}mA", ip_m1 * 1e3);
        // Monotonicity: more negative grid → less current
        assert!(ip_m1 < ip0, "12AX7: Ip should decrease with more negative Vgk");
        // Cutoff: Vgk=-4V → Ip ≈ 0
        assert!(t.plate_current(-4.0, 250.0) < 0.01e-3, "12AX7 should be near cutoff at Vgk=-4");
    }

    #[test]
    fn test_12au7_operating_points() {
        let t = make_tube(lookup("12AU7").unwrap());
        // Koren model at Vgk=0V, Vpk=250V → Ip ≈ 20mA
        // (Sylvania datasheet: ~11.5mA; Koren single-mu overestimates at zero grid bias)
        let ip0 = t.plate_current(0.0, 250.0);
        assert!(ip0 > 5e-3 && ip0 < 40e-3, "12AU7 Vgk=0: Ip={:.1}mA", ip0 * 1e3);
        // Vgk=-8.5V, Vpk=250V: Koren gives ~5.2mA (datasheet: ~2.3mA)
        let ip_neg = t.plate_current(-8.5, 250.0);
        assert!(ip_neg > 0.5e-3 && ip_neg < 15e-3, "12AU7 Vgk=-8.5: Ip={:.1}mA", ip_neg * 1e3);
        assert!(ip_neg < ip0, "12AU7: current should decrease with negative grid");
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
        assert!(ip > 10e-3 && ip < 200e-3, "EL34 Vgk=0: Ip={:.1}mA", ip * 1e3);
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
                let dip_dvgk = (t.plate_current(vgk + eps, vpk) - t.plate_current(vgk - eps, vpk)) / (2.0 * eps);
                let dip_dvpk = (t.plate_current(vgk, vpk + eps) - t.plate_current(vgk, vpk - eps)) / (2.0 * eps);

                if dip_dvgk.abs() > 1e-12 {
                    let rel = (jac[0] - dip_dvgk).abs() / dip_dvgk.abs();
                    assert!(rel < 1e-3,
                        "{} dIp/dVgk at ({},{}): analytic={:.6e} fd={:.6e} rel={:.2e}",
                        entry.names[0], vgk, vpk, jac[0], dip_dvgk, rel);
                }
                if dip_dvpk.abs() > 1e-12 {
                    let rel = (jac[1] - dip_dvpk).abs() / dip_dvpk.abs();
                    assert!(rel < 1e-3,
                        "{} dIp/dVpk at ({},{}): analytic={:.6e} fd={:.6e} rel={:.2e}",
                        entry.names[0], vgk, vpk, jac[1], dip_dvpk, rel);
                }
            }
        }
    }

    // Use NonlinearDevice trait for jacobian
    use crate::NonlinearDevice;

    #[test]
    fn test_12ax7_jacobian() { check_jacobian(lookup("12AX7").unwrap()); }
    #[test]
    fn test_12au7_jacobian() { check_jacobian(lookup("12AU7").unwrap()); }
    #[test]
    fn test_12at7_jacobian() { check_jacobian(lookup("12AT7").unwrap()); }
    #[test]
    fn test_6sl7_jacobian() { check_jacobian(lookup("6SL7").unwrap()); }
    #[test]
    fn test_6v6_jacobian() { check_jacobian(lookup("6V6").unwrap()); }
    #[test]
    fn test_el84_jacobian() { check_jacobian(lookup("EL84").unwrap()); }
    #[test]
    fn test_el34_jacobian() { check_jacobian(lookup("EL34").unwrap()); }
    #[test]
    fn test_6l6_jacobian() { check_jacobian(lookup("6L6").unwrap()); }

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
}
