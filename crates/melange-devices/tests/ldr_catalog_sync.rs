//! The LDR presets live twice: as `CdsLdr` named constructors in `src/ldr.rs`
//! and as `catalog::ldr::CATALOG` entries (which `.model NAME LDR()` cards
//! resolve against). This test holds the two copies equal field-for-field, and
//! fails if a preset is added to one side without the other.

use melange_devices::catalog::ldr::{self, LdrCatalogEntry};
use melange_devices::CdsLdr;

/// A `CdsLdr` named constructor (takes the sample rate).
type Preset = fn(f64) -> CdsLdr;

/// Every `CdsLdr` named constructor, keyed by its canonical part number.
/// A new constructor must be added here AND to `catalog::ldr::CATALOG`.
const PRESETS: &[(&str, Preset)] = &[
    ("VTL5C3", CdsLdr::vtl5c3),
    ("VTL5C4", CdsLdr::vtl5c4),
    ("NSL32", CdsLdr::nsl32),
];

fn assert_entry_matches(name: &str, entry: &LdrCatalogEntry, preset: &CdsLdr) {
    // Exact equality: the catalog is a copy, not a refit.
    assert_eq!(entry.r_min, preset.r_min, "{name}: r_min");
    assert_eq!(entry.r_max, preset.r_max, "{name}: r_max");
    assert_eq!(entry.gamma, preset.gamma, "{name}: gamma");
    assert_eq!(entry.attack_tau, preset.attack_tau, "{name}: attack_tau");
    assert_eq!(entry.release_tau, preset.release_tau, "{name}: release_tau");
}

#[test]
fn every_constructor_preset_matches_its_catalog_entry() {
    for &(name, ctor) in PRESETS {
        let entry = ldr::lookup(name)
            .unwrap_or_else(|| panic!("CdsLdr preset {name} has no catalog::ldr entry"));
        assert_entry_matches(name, entry, &ctor(48_000.0));
    }
}

#[test]
fn every_catalog_entry_has_a_matching_constructor() {
    assert_eq!(
        ldr::CATALOG.len(),
        PRESETS.len(),
        "catalog::ldr::CATALOG and the CdsLdr constructors differ in count"
    );
    for entry in ldr::CATALOG {
        let (name, ctor) = PRESETS
            .iter()
            .find(|(n, _)| entry.names.iter().any(|a| a.eq_ignore_ascii_case(n)))
            .unwrap_or_else(|| {
                panic!(
                    "catalog::ldr entry {:?} has no CdsLdr constructor",
                    entry.names
                )
            });
        assert_entry_matches(name, entry, &ctor(48_000.0));
        // Every alias must land on this same entry. (Compared by alias list,
        // not address: `CATALOG` is a `const`, so its address is not unique.)
        for alias in entry.names {
            let hit = ldr::lookup(alias).expect("alias resolves");
            assert_eq!(
                hit.names, entry.names,
                "alias {alias} resolves to a different entry"
            );
        }
    }
}
