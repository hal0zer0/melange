//! Component catalog for common audio devices.
//!
//! Provides verified, sourced parameters for standard parts, discoverable by
//! part number string. Each entry carries parameter values, name aliases, and
//! a source citation.
//!
//! # Usage
//!
//! ```
//! use melange_devices::catalog;
//!
//! // Look up by part number (case-insensitive)
//! if let Some(result) = catalog::lookup("12AX7") {
//!     // Use parameters...
//! }
//!
//! // Or look up within a specific family
//! if let Some(entry) = catalog::tubes::lookup("ECC83") {
//!     println!("mu = {}", entry.mu);
//! }
//! ```

pub mod tubes;
pub mod bjts;
pub mod diodes;
pub mod jfets;
pub mod mosfets;

/// Result of a cross-family catalog lookup.
#[derive(Debug, Clone, Copy)]
pub enum CatalogResult {
    Tube(&'static tubes::TubeCatalogEntry),
    Bjt(&'static bjts::BjtCatalogEntry),
    Diode(&'static diodes::DiodeCatalogEntry),
    Jfet(&'static jfets::JfetCatalogEntry),
    Mosfet(&'static mosfets::MosfetCatalogEntry),
}

/// Look up a component by part number across all device families.
///
/// Returns the first match found. Search order: tubes, BJTs, diodes, JFETs, MOSFETs.
pub fn lookup(name: &str) -> Option<CatalogResult> {
    if let Some(e) = tubes::lookup(name) { return Some(CatalogResult::Tube(e)); }
    if let Some(e) = bjts::lookup(name) { return Some(CatalogResult::Bjt(e)); }
    if let Some(e) = diodes::lookup(name) { return Some(CatalogResult::Diode(e)); }
    if let Some(e) = jfets::lookup(name) { return Some(CatalogResult::Jfet(e)); }
    if let Some(e) = mosfets::lookup(name) { return Some(CatalogResult::Mosfet(e)); }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cross_family_lookup() {
        // Tubes
        assert!(matches!(lookup("12AX7"), Some(CatalogResult::Tube(_))));
        assert!(matches!(lookup("ECC83"), Some(CatalogResult::Tube(_))));

        // BJTs
        assert!(matches!(lookup("2N2222A"), Some(CatalogResult::Bjt(_))));
        assert!(matches!(lookup("2N3906"), Some(CatalogResult::Bjt(_))));

        // Diodes
        assert!(matches!(lookup("1N4148"), Some(CatalogResult::Diode(_))));
        assert!(matches!(lookup("1N34A"), Some(CatalogResult::Diode(_))));

        // JFETs
        assert!(matches!(lookup("J201"), Some(CatalogResult::Jfet(_))));
        assert!(matches!(lookup("2N5460"), Some(CatalogResult::Jfet(_))));

        // MOSFETs
        assert!(matches!(lookup("BS170"), Some(CatalogResult::Mosfet(_))));
        assert!(matches!(lookup("IRF510"), Some(CatalogResult::Mosfet(_))));
    }

    #[test]
    fn test_unknown_returns_none() {
        assert!(lookup("NONEXISTENT").is_none());
    }

    #[test]
    fn test_cross_family_case_insensitive() {
        assert!(lookup("12ax7").is_some());
        assert!(lookup("2n3904").is_some());
        assert!(lookup("1n4148").is_some());
        assert!(lookup("j201").is_some());
        assert!(lookup("bs170").is_some());
    }
}
