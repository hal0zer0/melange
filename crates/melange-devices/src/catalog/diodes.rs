//! Diode catalog entries with Shockley equation parameters.
//!
//! Parameters sourced from manufacturer datasheets and SPICE models.

/// A catalog entry for a diode.
#[derive(Debug, Clone, Copy)]
pub struct DiodeCatalogEntry {
    /// Part number aliases
    pub names: &'static [&'static str],
    /// Saturation current IS [A]
    pub is: f64,
    /// Ideality factor N
    pub n: f64,
    /// Source citation
    pub source: &'static str,
}

/// The diode catalog.
pub const CATALOG: &[DiodeCatalogEntry] = &[
    // 1N4148 / 1N914 — fast switching signal diode (silicon)
    // ON Semi SPICE model: IS=2.52e-9, N=1.752
    DiodeCatalogEntry {
        names: &["1N4148", "1N914"],
        is: 2.52e-9,
        n: 1.752,
        source: "ON Semiconductor SPICE model",
    },
    // 1N4001..1N4007 — general purpose rectifier (silicon)
    // ON Semi: IS=14.11e-9, N=1.984
    DiodeCatalogEntry {
        names: &[
            "1N4001", "1N4002", "1N4003", "1N4004", "1N4005", "1N4006", "1N4007",
        ],
        is: 14.11e-9,
        n: 1.984,
        source: "ON Semiconductor SPICE model",
    },
    // 1N34A — germanium point-contact diode
    DiodeCatalogEntry {
        names: &["1N34A", "1N34"],
        is: 2e-7,
        n: 1.05,
        source: "Germanium datasheet (typical)",
    },
    // BAT41 / BAT46 — Schottky barrier diode
    // Vishay: IS≈2e-8, N≈1.04
    DiodeCatalogEntry {
        names: &["BAT41", "BAT46"],
        is: 2e-8,
        n: 1.04,
        source: "Vishay SPICE model",
    },
    // OA91 — vintage germanium diode
    DiodeCatalogEntry {
        names: &["OA91"],
        is: 1e-7,
        n: 1.1,
        source: "Vintage germanium (estimated from measurements)",
    },
    // 1N5819 — 1A Schottky rectifier
    DiodeCatalogEntry {
        names: &["1N5819"],
        is: 1e-8,
        n: 1.04,
        source: "ON Semiconductor SPICE model",
    },
];

/// Look up a diode by part number (case-insensitive).
pub fn lookup(name: &str) -> Option<&'static DiodeCatalogEntry> {
    CATALOG
        .iter()
        .find(|entry| entry.names.iter().any(|n| n.eq_ignore_ascii_case(name)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DiodeShockley;

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
        assert!(lookup("1n4148").is_some());
        assert!(lookup("1N4148").is_some());
        assert!(lookup("bat41").is_some());
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
                entry.n > 0.0 && entry.n.is_finite(),
                "{}: N",
                entry.names[0]
            );
        }
    }

    // --- Tier 1: Factory method consistency ---

    #[test]
    fn test_1n4148_matches_catalog() {
        let factory = DiodeShockley::silicon_1n4148();
        let cat = lookup("1N4148").unwrap();
        assert_eq!(factory.is, cat.is);
        assert_eq!(factory.n, cat.n);
    }

    #[test]
    fn test_germanium_matches_catalog() {
        let factory = DiodeShockley::germanium();
        let cat = lookup("1N34A").unwrap();
        assert_eq!(factory.is, cat.is);
        assert_eq!(factory.n, cat.n);
    }

    #[test]
    fn test_schottky_matches_1n5819() {
        // schottky() factory uses slightly different IS from 1N5819 catalog
        // (1e-8 vs 1e-8) — verify same concept
        let factory = DiodeShockley::schottky();
        let cat = lookup("1N5819").unwrap();
        assert_eq!(factory.is, cat.is);
        assert_eq!(factory.n, cat.n);
    }

    // --- Tier 2: Datasheet forward voltage verification ---

    fn solve_vf(diode: &DiodeShockley, target_i: f64) -> f64 {
        let mut v_lo = 0.0;
        let mut v_hi = 2.0;
        for _ in 0..100 {
            let v_mid = (v_lo + v_hi) / 2.0;
            if diode.current_at(v_mid) < target_i {
                v_lo = v_mid;
            } else {
                v_hi = v_mid;
            }
        }
        (v_lo + v_hi) / 2.0
    }

    fn make_diode(entry: &DiodeCatalogEntry) -> DiodeShockley {
        DiodeShockley::new_room_temp(entry.is, entry.n)
    }

    #[test]
    fn test_1n4148_forward_voltage() {
        let d = make_diode(lookup("1N4148").unwrap());
        // ON Semi datasheet: Vf ≈ 0.72V @ 5mA
        let vf_5ma = solve_vf(&d, 5e-3);
        assert!(
            (vf_5ma - 0.72).abs() < 0.15,
            "1N4148 Vf@5mA = {:.3}V (expected ~0.72V)",
            vf_5ma
        );
        // At 100mA, the ideal Shockley model (no Rs) underestimates Vf because
        // real 1N4148 has ~0.6 ohm series resistance. Our model gives ~0.79V
        // vs datasheet ~1.0V. This is a known limitation of the Rs-free model.
        let vf_100ma = solve_vf(&d, 100e-3);
        assert!(
            vf_100ma > 0.6 && vf_100ma < 1.2,
            "1N4148 Vf@100mA = {:.3}V (Shockley: ~0.8V, datasheet w/Rs: ~1.0V)",
            vf_100ma
        );
    }

    #[test]
    fn test_1n4001_forward_voltage() {
        let d = make_diode(lookup("1N4001").unwrap());
        // ON Semi: Vf ≈ 1.1V @ 1A (our model won't be exact at high current)
        // At 10mA: Vf ≈ 0.7-0.9V
        let vf_10ma = solve_vf(&d, 10e-3);
        assert!(
            vf_10ma > 0.5 && vf_10ma < 1.2,
            "1N4001 Vf@10mA = {:.3}V",
            vf_10ma
        );
    }

    #[test]
    fn test_1n34a_forward_voltage() {
        let d = make_diode(lookup("1N34A").unwrap());
        // Germanium: Vf ≈ 0.3V @ 1mA
        let vf_1ma = solve_vf(&d, 1e-3);
        assert!(
            (vf_1ma - 0.3).abs() < 0.15,
            "1N34A Vf@1mA = {:.3}V (expected ~0.3V)",
            vf_1ma
        );
    }

    #[test]
    fn test_bat41_forward_voltage() {
        let d = make_diode(lookup("BAT41").unwrap());
        // Schottky: Vf ≈ 0.35V @ 1mA
        let vf_1ma = solve_vf(&d, 1e-3);
        assert!(
            vf_1ma > 0.15 && vf_1ma < 0.55,
            "BAT41 Vf@1mA = {:.3}V (expected ~0.35V)",
            vf_1ma
        );
    }

    #[test]
    fn test_oa91_forward_voltage() {
        let d = make_diode(lookup("OA91").unwrap());
        // Vintage germanium: Vf ≈ 0.3-0.4V @ 1mA
        let vf_1ma = solve_vf(&d, 1e-3);
        assert!(
            vf_1ma > 0.15 && vf_1ma < 0.5,
            "OA91 Vf@1mA = {:.3}V",
            vf_1ma
        );
    }

    // --- Tier 3: Conductance consistency ---

    fn check_diode_conductance(entry: &DiodeCatalogEntry) {
        let d = make_diode(entry);
        let eps = 1e-8;
        for &v in &[0.2, 0.4, 0.5, 0.6] {
            let g = d.conductance_at(v);
            let fd = (d.current_at(v + eps) - d.current_at(v - eps)) / (2.0 * eps);
            let rel = (g - fd).abs() / (fd.abs() + 1e-15);
            assert!(
                rel < 1e-3,
                "{} conductance at {:.1}V: g={:.6e} fd={:.6e}",
                entry.names[0],
                v,
                g,
                fd
            );
        }
    }

    #[test]
    fn test_1n4148_conductance() {
        check_diode_conductance(lookup("1N4148").unwrap());
    }
    #[test]
    fn test_1n4001_conductance() {
        check_diode_conductance(lookup("1N4001").unwrap());
    }
    #[test]
    fn test_1n34a_conductance() {
        check_diode_conductance(lookup("1N34A").unwrap());
    }
    #[test]
    fn test_bat41_conductance() {
        check_diode_conductance(lookup("BAT41").unwrap());
    }
    #[test]
    fn test_oa91_conductance() {
        check_diode_conductance(lookup("OA91").unwrap());
    }
}
