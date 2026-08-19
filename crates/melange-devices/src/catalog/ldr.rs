//! Opto/LDR (CdS photoresistor) catalog entries.
//!
//! Parameters mirror the shipped presets in `melange-devices/src/ldr.rs`
//! (`CdsLdr::vtl5c3` / `vtl5c4` / `nsl32`). A `.model NAME LDR()` card with
//! empty parens resolves its parameters here by part number; explicit
//! `.model … LDR(RMIN=… …)` params override the catalog entry.

/// A catalog entry for an opto/LDR photoresistor.
#[derive(Debug, Clone, Copy)]
pub struct LdrCatalogEntry {
    /// Part-number aliases (case-insensitive).
    pub names: &'static [&'static str],
    /// Minimum resistance (brightest light) [Ω].
    pub r_min: f64,
    /// Maximum (dark) resistance [Ω].
    pub r_max: f64,
    /// Power-law exponent (brightness → resistance).
    pub gamma: f64,
    /// Attack time constant [s] (resistance decreasing / getting brighter).
    pub attack_tau: f64,
    /// Release time constant [s] (resistance increasing / getting darker).
    pub release_tau: f64,
    /// Source citation.
    pub source: &'static str,
}

/// The LDR/optocoupler catalog. Values match `ldr.rs`'s named constructors.
pub const CATALOG: &[LdrCatalogEntry] = &[
    // VTL5C3 — typical Vactrol-style optocoupler (datasheet r_min ≈ 75 Ω).
    LdrCatalogEntry {
        names: &["VTL5C3"],
        r_min: 75.0,
        r_max: 10e6,
        gamma: 0.7,
        attack_tau: 0.005,
        release_tau: 0.2,
        source: "ldr.rs CdsLdr::vtl5c3 (Vactrol VTL5C3 typical)",
    },
    // VTL5C4 — faster Vactrol (datasheet r_min ≈ 40 Ω).
    LdrCatalogEntry {
        names: &["VTL5C4"],
        r_min: 40.0,
        r_max: 5e6,
        gamma: 0.7,
        attack_tau: 0.001,
        release_tau: 0.05,
        source: "ldr.rs CdsLdr::vtl5c4 (Vactrol VTL5C4 typical)",
    },
    // NSL-32 (JHS-style) — datasheet r_min ≈ 75 Ω.
    LdrCatalogEntry {
        names: &["NSL32", "NSL-32"],
        r_min: 75.0,
        r_max: 1e6,
        gamma: 0.8,
        attack_tau: 0.002,
        release_tau: 0.1,
        source: "ldr.rs CdsLdr::nsl32 (NSL-32 typical)",
    },
];

/// Look up an LDR by part number (case-insensitive).
pub fn lookup(name: &str) -> Option<&'static LdrCatalogEntry> {
    CATALOG
        .iter()
        .find(|entry| entry.names.iter().any(|n| n.eq_ignore_ascii_case(name)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_aliases_resolve() {
        for entry in CATALOG {
            for name in entry.names {
                assert!(lookup(name).is_some(), "Alias '{}' should resolve", name);
            }
        }
    }

    #[test]
    fn vtl5c3_matches_ldr_rs() {
        let e = lookup("vtl5c3").unwrap();
        assert_eq!(e.r_min, 75.0);
        assert_eq!(e.r_max, 10e6);
        assert_eq!(e.gamma, 0.7);
    }
}
