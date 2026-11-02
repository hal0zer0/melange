//! JFET catalog entries with Shichman-Hodges model parameters.

/// A catalog entry for a JFET.
#[derive(Debug, Clone, Copy)]
pub struct JfetCatalogEntry {
    /// Part number aliases
    pub names: &'static [&'static str],
    /// Saturation drain current IDSS \[A\]
    pub idss: f64,
    /// Pinch-off voltage VP \[V\] (negative for N-channel, positive for P-channel)
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
    // Sheets (ON Semiconductor 2N5457/D Rev. 4, 2003, p. 2; Fairchild 1997,
    // p. 2): IDSS 1.0 min / 3.0 typ / 5.0 max mA; VGS(off) -0.5 min / -6.0 max
    // V at ID = 10 nA, NO typical printed; VGS typ -2.5 V at ID = 100 µA,
    // VDS = 15 V. `vp` -3.06 V is the pinch-off that reproduces BOTH printed
    // typicals under the catalogue's Shichman-Hodges law: ID = IDSS·(1 −
    // VGS/VP)² = 100 µA at VGS = −2.5 V with IDSS = 3 mA gives
    // VP = −2.5 / (1 − sqrt(0.1/3)) ≈ −3.06 V (typicals of separate
    // parameters need not describe one device; this is the sheet-consistent
    // inference, not a printed value). gm0 = 2·IDSS/|VP| = 1.96 mS, inside
    // the sheet's 1.0–5.0 mmho |Yfs| band. `lambda` is unsourced (a sheet
    // gives output admittance, not lambda).
    JfetCatalogEntry {
        names: &["2N5457"],
        idss: 3e-3,
        vp: -3.06,
        lambda: 0.004,
        is_p_channel: false,
        source: "ON Semiconductor / Fairchild datasheets: typ IDSS; vp inferred from typ IDSS and typ VGS at 100 µA (SH); lambda assumed",
    },
    // J201 — N-channel, low-noise audio JFET
    // Sheet (Fairchild J201/J202, Rev. 1.0.0, January 2008, p. 2; the only
    // sheet on file, min/max columns only): IDSS 0.2 min / 1.0 max mA at
    // VDS = 20 V; VGS(off) -0.3 min / -1.5 max V at ID = 10 nA; yfs 500 µmho
    // min at 1 kHz. NO typical is printed for any row and there is no
    // "VGS at ID" row, so nothing on the sheet anchors a single value:
    // `idss` 0.6 mA and `vp` -0.8 V are assumed mid-band values (gm0 =
    // 2·IDSS/|VP| = 1.5 mS, above the 500 µmho minimum); `lambda` is
    // unsourced.
    JfetCatalogEntry {
        names: &["J201"],
        idss: 6e-4,
        vp: -0.8,
        lambda: 0.004,
        is_p_channel: false,
        source: "Fairchild datasheet: IDSS and VGS(off) bands only (no typicals); idss, vp and lambda assumed",
    },
    // 2N3819 — N-channel, general purpose
    // Sheets (Fairchild 2N3819 Rev. A1, 2002; InterFET IF35039.R00, 2019):
    // IDSS 2 min / 20 max mA, NO typical printed; VGS -0.5 to -7.5 V at
    // ID = 200 µA, no typical. `idss` 2 mA is the printed MINIMUM (there is
    // no typical to anchor to); `vp` -3 V is assumed; `lambda` is unsourced.
    JfetCatalogEntry {
        names: &["2N3819"],
        idss: 2e-3,
        vp: -3.0,
        lambda: 0.005,
        is_p_channel: false,
        source: "Fairchild / InterFET datasheets: idss = printed IDSS minimum; vp and lambda assumed",
    },
    // 2N5460 — P-channel, general purpose
    // Sheet (ON Semiconductor 2N5460/D Rev. 5, March 2006, PDF p. 3; the only
    // sheet on file): IDSS -1.0 min / -5.0 max mA at VDS 15 V; VGS(off) 0.75
    // min / 6.0 max V at ID = 1.0 µA; VGS 0.5 min / 4.0 max V at ID = 0.1 mA
    // (min/max, not a typical); |yfs| 1000 min / 4000 max µmho. The Typ
    // column is blank for every DC row, so nothing on the sheet anchors a
    // single value: `idss` 3 mA and `vp` 2.5 V are assumed mid-band values
    // (gm0 = 2·IDSS/|VP| = 2.4 mS, inside the |yfs| band); `lambda` is
    // unsourced (|yos| 75 µmho max is the sheet's output admittance).
    JfetCatalogEntry {
        names: &["2N5460"],
        idss: 3e-3,
        vp: 2.5,
        lambda: 0.004,
        is_p_channel: true,
        source: "ON Semiconductor datasheet: IDSS and VGS bands only (no DC typicals); idss, vp and lambda assumed",
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
