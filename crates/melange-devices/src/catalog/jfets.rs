//! JFET catalog entries with Shichman-Hodges model parameters.

/// A catalog entry for a JFET.
#[derive(Debug, Clone, Copy)]
pub struct JfetCatalogEntry {
    /// Part number aliases
    pub names: &'static [&'static str],
    /// Saturation drain current IDSS [A]
    pub idss: f64,
    /// Pinch-off voltage VP [V] (negative for N-channel, positive for P-channel)
    pub vp: f64,
    /// Channel-length modulation LAMBDA [1/V]
    pub lambda: f64,
    /// True if P-channel
    pub is_p_channel: bool,
    /// Source citation
    pub source: &'static str,
}

/// The JFET catalog.
pub const CATALOG: &[JfetCatalogEntry] = &[
    // 2N5457 — N-channel, general purpose (audio, switching)
    // ON Semi: IDSS=1-5mA (typ 3mA), VP=-0.5 to -6.0V (typ -2.5V)
    JfetCatalogEntry {
        names: &["2N5457"],
        idss: 3e-3,
        vp: -2.5,
        lambda: 0.004,
        is_p_channel: false,
        source: "ON Semiconductor datasheet (typical values)",
    },
    // J201 — N-channel, low-noise audio JFET
    // ON Semi: IDSS=0.2-1.0mA (typ 0.6mA), VP=-0.3 to -1.5V (typ -0.8V)
    JfetCatalogEntry {
        names: &["J201"],
        idss: 6e-4,
        vp: -0.8,
        lambda: 0.004,
        is_p_channel: false,
        source: "ON Semiconductor datasheet (typical values)",
    },
    // 2N3819 — N-channel, general purpose
    // ON Semi: IDSS=2-20mA (typ 5mA), VP=-0.5 to -8V (typ -3V)
    JfetCatalogEntry {
        names: &["2N3819"],
        idss: 2e-3,
        vp: -3.0,
        lambda: 0.005,
        is_p_channel: false,
        source: "ON Semiconductor datasheet",
    },
    // 2N5460 — P-channel, general purpose
    // ON Semi: IDSS=1-5mA (typ 3mA), VP=0.75-6V (typ 2.5V)
    JfetCatalogEntry {
        names: &["2N5460"],
        idss: 3e-3,
        vp: 2.5,
        lambda: 0.004,
        is_p_channel: true,
        source: "ON Semiconductor datasheet (typical values)",
    },
];

/// Look up a JFET by part number (case-insensitive).
pub fn lookup(name: &str) -> Option<&'static JfetCatalogEntry> {
    CATALOG
        .iter()
        .find(|entry| entry.names.iter().any(|n| n.eq_ignore_ascii_case(name)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Jfet, JfetChannel, NonlinearDevice};

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
        assert!(lookup("j201").is_some());
        assert!(lookup("J201").is_some());
        assert!(lookup("2n5457").is_some());
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
                entry.idss > 0.0 && entry.idss.is_finite(),
                "{}: IDSS",
                entry.names[0]
            );
            assert!(
                entry.vp.abs() > 1e-15 && entry.vp.is_finite(),
                "{}: VP",
                entry.names[0]
            );
            assert!(
                entry.lambda >= 0.0 && entry.lambda.is_finite(),
                "{}: LAMBDA",
                entry.names[0]
            );
            // N-channel should have negative Vp, P-channel positive
            if entry.is_p_channel {
                assert!(
                    entry.vp > 0.0,
                    "{}: P-ch VP should be positive",
                    entry.names[0]
                );
            } else {
                assert!(
                    entry.vp < 0.0,
                    "{}: N-ch VP should be negative",
                    entry.names[0]
                );
            }
        }
    }

    // --- Tier 1: Factory method consistency ---

    #[test]
    fn test_2n5457_matches_catalog() {
        let factory = Jfet::n_2n5457();
        let cat = lookup("2N5457").unwrap();
        assert_eq!(factory.idss, cat.idss);
        assert_eq!(factory.vp, cat.vp);
    }

    #[test]
    fn test_j201_matches_catalog() {
        let factory = Jfet::n_j201();
        let cat = lookup("J201").unwrap();
        assert_eq!(factory.idss, cat.idss);
        assert_eq!(factory.vp, cat.vp);
    }

    #[test]
    fn test_2n3819_matches_catalog() {
        let factory = Jfet::n_2n3819();
        let cat = lookup("2N3819").unwrap();
        assert_eq!(factory.idss, cat.idss);
        assert_eq!(factory.vp, cat.vp);
    }

    #[test]
    fn test_2n5460_matches_catalog() {
        let factory = Jfet::p_2n5460();
        let cat = lookup("2N5460").unwrap();
        assert_eq!(factory.idss, cat.idss);
        assert_eq!(factory.vp, cat.vp);
    }

    // --- Tier 2: Datasheet operating point verification ---

    fn make_jfet(entry: &JfetCatalogEntry) -> Jfet {
        let channel = if entry.is_p_channel {
            JfetChannel::P
        } else {
            JfetChannel::N
        };
        let mut j = Jfet::new(channel, entry.vp, entry.idss);
        j.lambda = entry.lambda;
        j
    }

    #[test]
    fn test_j201_operating_points() {
        let j = make_jfet(lookup("J201").unwrap());
        // Vgs=0, Vds=10V → Id ≈ IDSS (0.6mA, range 0.2-1.0mA)
        let id = j.drain_current(0.0, 10.0);
        assert!(
            id > 0.2e-3 && id < 1.5e-3,
            "J201 IDSS: {:.3}mA (expected 0.2-1.0mA)",
            id * 1e3
        );
        // At Vgs=Vp, Id ≈ 0
        let id_cutoff = j.drain_current(j.vp, 10.0);
        assert!(
            id_cutoff.abs() < 1e-6,
            "J201 cutoff: {:.6}mA",
            id_cutoff * 1e3
        );
    }

    #[test]
    fn test_2n5457_operating_points() {
        let j = make_jfet(lookup("2N5457").unwrap());
        let id = j.drain_current(0.0, 10.0);
        assert!(
            id > 1e-3 && id < 10e-3,
            "2N5457 IDSS: {:.3}mA (expected ~3mA)",
            id * 1e3
        );
        let id_cutoff = j.drain_current(j.vp, 10.0);
        assert!(id_cutoff.abs() < 1e-6, "2N5457 cutoff");
    }

    #[test]
    fn test_2n3819_operating_points() {
        let j = make_jfet(lookup("2N3819").unwrap());
        let id = j.drain_current(0.0, 10.0);
        assert!(id > 1e-3 && id < 25e-3, "2N3819 IDSS: {:.3}mA", id * 1e3);
    }

    #[test]
    fn test_2n5460_operating_points() {
        let j = make_jfet(lookup("2N5460").unwrap());
        // P-channel: Vgs=0, Vds=-10V
        let id = j.drain_current(0.0, -10.0);
        assert!(
            id < -0.5e-3 && id > -10e-3,
            "2N5460 IDSS: {:.3}mA",
            id * 1e3
        );
    }

    // --- Tier 3: Jacobian consistency ---

    fn check_jfet_jacobian(entry: &JfetCatalogEntry) {
        let j = make_jfet(entry);
        let eps = 1e-6;
        let (vds_sign, vgs_vals): (f64, &[f64]) = if entry.is_p_channel {
            (-1.0, &[0.0, 1.0, 2.0])
        } else {
            (1.0, &[0.0, -0.5, -1.0])
        };
        for &vgs in vgs_vals {
            for &vds_mag in &[1.0, 5.0, 10.0] {
                let vds = vds_sign * vds_mag;
                let jac = j.jacobian(&[vgs, vds]);
                let fd0 = (j.drain_current(vgs + eps, vds) - j.drain_current(vgs - eps, vds))
                    / (2.0 * eps);
                let fd1 = (j.drain_current(vgs, vds + eps) - j.drain_current(vgs, vds - eps))
                    / (2.0 * eps);
                if fd0.abs() > 1e-10 {
                    let rel = (jac[0] - fd0).abs() / fd0.abs();
                    assert!(
                        rel < 1e-2,
                        "{} gm at ({},{}): a={:.6e} fd={:.6e}",
                        entry.names[0],
                        vgs,
                        vds,
                        jac[0],
                        fd0
                    );
                }
                if fd1.abs() > 1e-10 {
                    let rel = (jac[1] - fd1).abs() / fd1.abs();
                    assert!(
                        rel < 1e-2,
                        "{} gds at ({},{}): a={:.6e} fd={:.6e}",
                        entry.names[0],
                        vgs,
                        vds,
                        jac[1],
                        fd1
                    );
                }
            }
        }
    }

    #[test]
    fn test_2n5457_jacobian() {
        check_jfet_jacobian(lookup("2N5457").unwrap());
    }
    #[test]
    fn test_j201_jacobian() {
        check_jfet_jacobian(lookup("J201").unwrap());
    }
    #[test]
    fn test_2n3819_jacobian() {
        check_jfet_jacobian(lookup("2N3819").unwrap());
    }
    #[test]
    fn test_2n5460_jacobian() {
        check_jfet_jacobian(lookup("2N5460").unwrap());
    }
}
