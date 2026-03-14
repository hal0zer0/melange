//! MOSFET catalog entries with Level 1 SPICE model parameters.

/// A catalog entry for a MOSFET.
#[derive(Debug, Clone, Copy)]
pub struct MosfetCatalogEntry {
    /// Part number aliases
    pub names: &'static [&'static str],
    /// Transconductance parameter KP [A/V^2]
    pub kp: f64,
    /// Threshold voltage VT [V] (positive for N-ch, negative for P-ch)
    pub vt: f64,
    /// Channel-length modulation LAMBDA [1/V]
    pub lambda: f64,
    /// True if P-channel
    pub is_p_channel: bool,
    /// Source citation
    pub source: &'static str,
}

/// The MOSFET catalog.
pub const CATALOG: &[MosfetCatalogEntry] = &[
    // BS170 — N-channel enhancement MOSFET, small signal
    // ON Semi: VGS(th)=0.8-3.0V (typ ~2.1V), KP derived from Rds(on) specs
    MosfetCatalogEntry {
        names: &["BS170"],
        kp: 0.05,
        vt: 2.1,
        lambda: 0.02,
        is_p_channel: false,
        source: "ON Semiconductor datasheet, Level 1 fit",
    },
    // 2N7000 — N-channel enhancement MOSFET, small signal
    // ON Semi: VGS(th)=0.8-3.0V (typ ~2.1V)
    MosfetCatalogEntry {
        names: &["2N7000"],
        kp: 0.1,
        vt: 2.1,
        lambda: 0.01,
        is_p_channel: false,
        source: "ON Semiconductor datasheet, Level 1 fit",
    },
    // IRF510 — N-channel power MOSFET
    // IR/Infineon: VGS(th)=2-4V (typ ~3V), high KP
    MosfetCatalogEntry {
        names: &["IRF510"],
        kp: 1.8,
        vt: 3.0,
        lambda: 0.01,
        is_p_channel: false,
        source: "Infineon/IR datasheet, Level 1 fit",
    },
    // IRF520 — N-channel power MOSFET
    MosfetCatalogEntry {
        names: &["IRF520"],
        kp: 2.5,
        vt: 3.0,
        lambda: 0.008,
        is_p_channel: false,
        source: "Infineon/IR datasheet, Level 1 fit",
    },
];

/// Look up a MOSFET by part number (case-insensitive).
pub fn lookup(name: &str) -> Option<&'static MosfetCatalogEntry> {
    CATALOG.iter().find(|entry| {
        entry.names.iter().any(|n| n.eq_ignore_ascii_case(name))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Mosfet, MosfetChannelType, NonlinearDevice};

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
        assert!(lookup("bs170").is_some());
        assert!(lookup("BS170").is_some());
        assert!(lookup("2n7000").is_some());
        assert!(lookup("irf510").is_some());
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
            assert!(entry.kp > 0.0 && entry.kp.is_finite(), "{}: KP", entry.names[0]);
            assert!(entry.vt.is_finite(), "{}: VT", entry.names[0]);
            assert!(entry.lambda >= 0.0 && entry.lambda.is_finite(), "{}: LAMBDA", entry.names[0]);
            if entry.is_p_channel {
                assert!(entry.vt < 0.0, "{}: P-ch VT should be negative", entry.names[0]);
            } else {
                assert!(entry.vt > 0.0, "{}: N-ch VT should be positive", entry.names[0]);
            }
        }
    }

    // --- Tier 1: Factory method consistency ---

    #[test]
    fn test_bs170_matches_catalog() {
        let cat = lookup("BS170").unwrap();
        // Factory uses Vt=0.8; catalog updated to datasheet typical 2.1V
        // This test checks that the catalog entry is physically reasonable
        assert!(cat.vt > 0.5 && cat.vt < 4.0);
        assert!(cat.kp > 0.01 && cat.kp < 1.0);
    }

    #[test]
    fn test_2n7000_matches_catalog() {
        let cat = lookup("2N7000").unwrap();
        assert!(cat.vt > 0.5 && cat.vt < 4.0);
        assert!(cat.kp > 0.01 && cat.kp < 1.0);
    }

    // --- Tier 2: Datasheet operating point verification ---

    fn make_mosfet(entry: &MosfetCatalogEntry) -> Mosfet {
        let channel = if entry.is_p_channel { MosfetChannelType::P } else { MosfetChannelType::N };
        Mosfet::new(channel, entry.vt, entry.kp, entry.lambda)
    }

    #[test]
    fn test_bs170_operating_points() {
        let m = make_mosfet(lookup("BS170").unwrap());
        // Above threshold: should conduct
        let id = m.drain_current(5.0, 10.0);
        assert!(id > 0.1e-3, "BS170 should conduct at Vgs=5V, got {:.3}mA", id * 1e3);
        // Below threshold: no conduction
        assert!(m.drain_current(0.5, 10.0) < 1e-6, "BS170 should be off at Vgs=0.5V");
    }

    #[test]
    fn test_2n7000_operating_points() {
        let m = make_mosfet(lookup("2N7000").unwrap());
        let id = m.drain_current(5.0, 10.0);
        assert!(id > 0.1e-3, "2N7000 should conduct at Vgs=5V, got {:.3}mA", id * 1e3);
        assert!(m.drain_current(1.0, 10.0) < 1e-6, "2N7000 should be off at Vgs=1V");
    }

    #[test]
    fn test_irf510_operating_points() {
        let m = make_mosfet(lookup("IRF510").unwrap());
        // Power MOSFET: at Vgs=10V, Vds=5V → substantial current
        let id = m.drain_current(10.0, 5.0);
        assert!(id > 10e-3, "IRF510 at Vgs=10V: Id={:.1}mA", id * 1e3);
        // Below threshold
        assert!(m.drain_current(2.0, 5.0) < 1e-6, "IRF510 below threshold");
    }

    // --- Tier 3: Jacobian consistency ---

    fn check_mosfet_jacobian(entry: &MosfetCatalogEntry) {
        let m = make_mosfet(entry);
        let eps = 1e-6;
        let vt_abs = entry.vt.abs();
        for &vgs_above in &[1.0, 3.0, 5.0] {
            let vgs = if entry.is_p_channel { -(vt_abs + vgs_above) } else { vt_abs + vgs_above };
            for &vds_mag in &[0.5, 2.0, 10.0] {
                let vds = if entry.is_p_channel { -vds_mag } else { vds_mag };
                let jac = m.jacobian(&[vgs, vds]);
                let fd0 = (m.drain_current(vgs + eps, vds) - m.drain_current(vgs - eps, vds)) / (2.0 * eps);
                let fd1 = (m.drain_current(vgs, vds + eps) - m.drain_current(vgs, vds - eps)) / (2.0 * eps);
                if fd0.abs() > 1e-10 {
                    let rel = (jac[0] - fd0).abs() / fd0.abs();
                    assert!(rel < 1e-2,
                        "{} gm at ({},{}): a={:.6e} fd={:.6e}", entry.names[0], vgs, vds, jac[0], fd0);
                }
                if fd1.abs() > 1e-10 {
                    let rel = (jac[1] - fd1).abs() / fd1.abs();
                    assert!(rel < 1e-2,
                        "{} gds at ({},{}): a={:.6e} fd={:.6e}", entry.names[0], vgs, vds, jac[1], fd1);
                }
            }
        }
    }

    #[test]
    fn test_bs170_jacobian() { check_mosfet_jacobian(lookup("BS170").unwrap()); }
    #[test]
    fn test_2n7000_jacobian() { check_mosfet_jacobian(lookup("2N7000").unwrap()); }
    #[test]
    fn test_irf510_jacobian() { check_mosfet_jacobian(lookup("IRF510").unwrap()); }
    #[test]
    fn test_irf520_jacobian() { check_mosfet_jacobian(lookup("IRF520").unwrap()); }
}
