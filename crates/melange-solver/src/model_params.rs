//! Central `.model` parameter tables — one source of truth for which keys each
//! device class accepts, what they cost when melange does not model them, and
//! which of them define the device's electrical identity.
//!
//! # Why this module exists
//!
//! `.model` keys used to be validated in whichever place happened to read them:
//! the codegen resolvers hard-errored on an unknown key for diodes, BJTs, JFETs,
//! MOSFETs, tubes, VCAs, LDRs and glow lamps, while op-amps and VCAs carried a
//! second, independent `_ =>` arm in [`crate::mna`] that warned against a
//! *different* key set. That divergence was not hypothetical: the op-amp arm was
//! the only place a `.model` typo was ever reported for an op-amp, and the VCA
//! arm warned about `THD` — a key the VCA resolver **honors**.
//!
//! Every consumer now reads its accepted keys from here:
//!
//! * `codegen::ir::CircuitIr::check_model_params` — hard error on an unknown key
//!   for a *referenced* device model (unchanged behaviour).
//! * [`crate::mna`] op-amp / VCA model resolution — `log::warn!` on an unknown
//!   key (unchanged behaviour; the key set is now this table's).
//! * [`crate::parser`] — `log::warn!` on an unknown key for a `.model` card that
//!   **no element references**, which nothing checked at all before: an orphan
//!   card never reaches a codegen resolver, so `.model 2N3904 NPN(ZORP=5)` sat
//!   in a deck completely silently.
//!
//! # Drift
//!
//! A table that lists keys nothing reads (or omits keys something does read) is
//! worse than no table: `SHOT_GAMMA2` was missing from the triode list for
//! exactly that reason, so melange warned "unrecognized" about a parameter it
//! was using. `tests/model_param_table_drift_tests.rs` scans the crate source
//! for every literal `.model`-key read and asserts the two directions agree, so
//! the tables cannot silently drift from the code that consumes them.
//!
//! **This module is a diagnostics surface only.** Adding a key here does not
//! make melange honor it — the resolver that reads it decides that. Adding a key
//! here without a reader makes melange *silently accept and ignore* it, which
//! the drift test rejects.

/// A `.model` device class — the granularity at which melange defines an
/// accepted parameter set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModelClass {
    Diode,
    Bjt,
    Jfet,
    Mosfet,
    Triode,
    Pentode,
    Opamp,
    Vca,
    Ldr,
    Glow,
}

/// Every class, for exhaustive iteration in tests and validation passes.
pub const ALL_CLASSES: [ModelClass; 10] = [
    ModelClass::Diode,
    ModelClass::Bjt,
    ModelClass::Jfet,
    ModelClass::Mosfet,
    ModelClass::Triode,
    ModelClass::Pentode,
    ModelClass::Opamp,
    ModelClass::Vca,
    ModelClass::Ldr,
    ModelClass::Glow,
];

impl ModelClass {
    /// Map a `.model NAME TYPE(...)` type token to its class.
    ///
    /// Mirrors the type rules the parser enforces when an element references a
    /// model (`parser.rs`, `TypeRule::Exact` / `TypeRule::Prefix`) — polarity is
    /// decided by prefix, so short forms like `NJ`/`PM` are legal. Returns
    /// `None` for a type melange has no parameter table for; callers must stay
    /// silent in that case rather than guess.
    pub fn from_model_type(model_type: &str) -> Option<Self> {
        let t = model_type.trim().to_ascii_uppercase();
        match t.as_str() {
            "D" => return Some(ModelClass::Diode),
            "TRIODE" | "VT" | "TUBE" => return Some(ModelClass::Triode),
            "VP" | "PENTODE" => return Some(ModelClass::Pentode),
            "OA" => return Some(ModelClass::Opamp),
            "VCA" => return Some(ModelClass::Vca),
            "LDR" => return Some(ModelClass::Ldr),
            "NEON" => return Some(ModelClass::Glow),
            _ => {}
        }
        if t.starts_with("NPN") || t.starts_with("PNP") {
            Some(ModelClass::Bjt)
        } else if t.starts_with("NJ") || t.starts_with("PJ") {
            Some(ModelClass::Jfet)
        } else if t.starts_with("NM") || t.starts_with("PM") {
            Some(ModelClass::Mosfet)
        } else {
            None
        }
    }

    /// Human-readable class name for diagnostics.
    pub fn label(self) -> &'static str {
        match self {
            ModelClass::Diode => "diode",
            ModelClass::Bjt => "BJT",
            ModelClass::Jfet => "JFET",
            ModelClass::Mosfet => "MOSFET",
            ModelClass::Triode => "triode",
            ModelClass::Pentode => "pentode",
            ModelClass::Opamp => "op-amp",
            ModelClass::Vca => "VCA",
            ModelClass::Ldr => "LDR",
            ModelClass::Glow => "glow lamp",
        }
    }

    /// Keys melange reads. Silent when present on a card.
    pub fn honored(self) -> &'static [&'static str] {
        match self {
            ModelClass::Diode => DIODE_HONORED,
            ModelClass::Bjt => BJT_HONORED,
            ModelClass::Jfet => JFET_HONORED,
            ModelClass::Mosfet => MOSFET_HONORED,
            ModelClass::Triode => TRIODE_HONORED,
            ModelClass::Pentode => PENTODE_HONORED,
            ModelClass::Opamp => OPAMP_HONORED,
            ModelClass::Vca => VCA_HONORED,
            ModelClass::Ldr => LDR_HONORED,
            ModelClass::Glow => GLOW_HONORED,
        }
    }

    /// Real SPICE keys melange does not model yet, each paired with what the
    /// omission costs. Warned about, never an error — these arrive on authentic
    /// vendor cards and refusing them would mean melange rejects genuine SPICE
    /// decks over a gap of its own.
    pub fn unimplemented(self) -> &'static [(&'static str, &'static str)] {
        match self {
            ModelClass::Bjt => BJT_UNIMPLEMENTED,
            _ => &[],
        }
    }

    /// The class's electrical-identity keys: `honored()` minus universal
    /// add-ons (KF/AF/RTH/CTH/XTI/EG/TAMB…), which do not say *which* device
    /// this is. Used by the "declared but underspecified card compiled as the
    /// default device" warning. Empty when the class has no such warning.
    pub fn defining(self) -> &'static [&'static str] {
        match self {
            ModelClass::Bjt => BJT_DEFINING,
            ModelClass::Jfet => JFET_DEFINING,
            ModelClass::Mosfet => MOSFET_DEFINING,
            ModelClass::Triode => TRIODE_DEFINING,
            ModelClass::Pentode => PENTODE_DEFINING,
            ModelClass::Ldr => LDR_DEFINING,
            _ => &[],
        }
    }

    /// True when `key` (any case) is honored by this class.
    pub fn is_honored(self, key: &str) -> bool {
        self.honored().iter().any(|k| k.eq_ignore_ascii_case(key))
    }

    /// The "melange does not model this" note for `key`, if it is on the
    /// recognized-but-unimplemented list.
    pub fn unimplemented_note(self, key: &str) -> Option<&'static str> {
        self.unimplemented()
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(key))
            .map(|(_, effect)| *effect)
    }
}

/// Emit the standard unrecognized-parameter warning unless `key` is a key this
/// class accepts (honored, or recognized-but-unimplemented — the latter is
/// reported by the codegen resolver with its full cost note, so warning twice
/// here would only add noise).
///
/// Shared by the op-amp / VCA resolution loops in [`crate::mna`] and the orphan
/// `.model` card pass in [`crate::parser`] so all three emit an identical line.
pub fn warn_if_unknown(model_name: &str, class: ModelClass, key: &str) {
    if class.is_honored(key) || class.unimplemented_note(key).is_some() {
        return;
    }
    // The hint carries its own leading space and is usually empty; appending a
    // bare "." in that case would change the line for every key that has no
    // hint, which is nearly all of them.
    let hint = alias_hint(class, key);
    if hint.is_empty() {
        log::warn!(
            ".model {}: unrecognized parameter '{}' (ignored)",
            model_name,
            key
        );
    } else {
        log::warn!(
            ".model {}: unrecognized parameter '{}' (ignored).{}",
            model_name,
            key,
            hint
        );
    }
}

/// Warn about unrecognized `.model` keys on cards that an element **does**
/// reference — the half of the check that a lightweight inspection command
/// cannot get any other way.
///
/// `melange compile` / `simulate` / `analyze` reach these cards through the
/// codegen resolvers, which hard-error on an unknown key. `melange nodes`
/// never builds the IR, so on that command `.model 1N4148 D(RSS=100)` — `RS`
/// mistyped, on the most common nonlinear part in a pedal — was reported by
/// nothing at all, while the op-amp card two lines down warned. That
/// inconsistency teaches the author to trust the silence.
///
/// **Warn, never error.** `nodes` is an inspection command; refusing to show a
/// deck's pot ranges because one `.model` key is misspelled would be a worse
/// trade than reporting it and carrying on. The hard error still stands on the
/// commands that actually compile the card.
///
/// Op-amp and VCA cards are skipped: `nodes` builds the MNA, and the op-amp /
/// VCA resolution loops in [`crate::mna`] already warn from this same table, so
/// including them here would print every such line twice. Cards that **no**
/// element references are likewise skipped — the parser's orphan-card pass
/// covers those, on every command. Between the three, each key is reported
/// exactly once.
pub fn warn_unknown_keys_on_referenced_models(netlist: &crate::parser::Netlist) {
    let mut reported: std::collections::HashSet<(String, String)> =
        std::collections::HashSet::new();
    for elem in &netlist.elements {
        let Some(model_ref) = elem.model_name() else {
            continue;
        };
        let Some(model) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_ref))
        else {
            // Undefined model reference — already a hard parse error.
            continue;
        };
        // No table for this type: stay silent rather than guess (a deck may
        // legitimately carry cards for device types melange does not model).
        let Some(class) = ModelClass::from_model_type(&model.model_type) else {
            continue;
        };
        if matches!(class, ModelClass::Opamp | ModelClass::Vca) {
            continue;
        }
        for (key, _) in &model.params {
            // One line per (card, key), however many elements share the card.
            if reported.insert((model.name.to_ascii_lowercase(), key.to_ascii_uppercase())) {
                warn_if_unknown(&model.name, class, key);
            }
        }
    }
}

/// A pointed hint for keys that are a plausible *confusion* rather than a typo
/// — a datasheet symbol, or the same quantity under another standard's name.
///
/// Returned with a leading space so it drops straight into a sentence; `""`
/// when nothing is known about the key.
///
/// Lives here rather than beside the codegen resolver that first needed it so
/// that every consumer of the tables gets the same hint. It did not: the hard
/// error for `VP` on a *referenced* JFET card explained the VTO confusion and
/// the sign trap, while the warning for the identical key on an *unreferenced*
/// card said only "unrecognized parameter" — the same key, the same mistake,
/// two different qualities of answer depending on whether some other line of
/// the deck happened to reference the card.
pub fn alias_hint(class: ModelClass, key: &str) -> &'static str {
    let upper = key.to_ascii_uppercase();
    match (class, upper.as_str()) {
        // Datasheets give a JFET's pinch-off as VP (and a MOSFET's threshold
        // occasionally the same way); SPICE spells it VTO, and the sign
        // convention is not the same, so a silent drop biases correctly off the
        // catalog default and hides the mistake.
        (ModelClass::Jfet | ModelClass::Mosfet, "VP") => {
            " Did you mean VTO? `VP` is the datasheet symbol for pinch-off; \
             SPICE spells it VTO, and note the sign convention differs (VTO is \
             negative for an N-channel JFET)."
        }
        _ => "",
    }
}

// ---------------------------------------------------------------------------
// Tables. Moved verbatim from the codegen resolvers / mna.rs match arms — no
// key was added or removed in the move. Each entry must have a reader in the
// crate (the drift test enforces it).
// ---------------------------------------------------------------------------

/// `KF`/`AF` (flicker) are read by `codegen::ir::noise`, not by the resolvers.
const DIODE_HONORED: &[&str] = &[
    "IS", "N", "CJO", "RS", "BV", "IBV", "KF", "AF", "RTH", "CTH", "XTI", "EG", "TAMB",
];

const BJT_HONORED: &[&str] = &[
    "IS", "VT", "BF", "BR", "VAF", "VA", "VAR", "VB", "IKF", "JBF", "IKR", "JBR", "CJE", "CJC",
    "VJE", "MJE", "VJC", "MJC", "FC", "TF", "NF", "NR", "ISE", "NE", "ISC", "NC", "RB", "RC", "RE",
    "RTH", "CTH", "XTI", "XTB", "EG", "TAMB", "KF", "AF",
];

const BJT_UNIMPLEMENTED: &[(&str, &str)] = &[
    (
        "TR",
        "reverse transit time — melange's junction charge is linearized \
         at the DC operating point, so the time-varying BC diffusion \
         charge TR describes cannot be represented (measured: honoring \
         it moves melange AWAY from ngspice). Blocked on per-timestep \
         charge re-linearization.",
    ),
    (
        "XCJC",
        "base-collector depletion capacitance split across the internal \
         base node — melange places all of CJC at the internal base, \
         which is XCJC=1.0 (the SPICE default). Only XCJC<1 is affected.",
    ),
];

const BJT_DEFINING: &[&str] = &[
    "IS", "VT", "BF", "BR", "VAF", "VA", "VAR", "VB", "IKF", "JBF", "IKR", "JBR", "CJE", "CJC",
    "VJE", "MJE", "VJC", "MJC", "FC", "TF", "NF", "NR", "ISE", "NE", "ISC", "NC", "RB", "RC", "RE",
    "XTB",
];

const JFET_HONORED: &[&str] = &[
    "VTO", "BETA", "IDSS", "LAMBDA", "CGS", "CGD", "RD", "RS", "KF", "AF",
];

const JFET_DEFINING: &[&str] = &["VTO", "BETA", "IDSS", "LAMBDA", "CGS", "CGD", "RD", "RS"];

const MOSFET_HONORED: &[&str] = &[
    "KP", "VTO", "VT", "LAMBDA", "CGS", "CGD", "RD", "RS", "GAMMA", "PHI", "KF", "AF",
];

const MOSFET_DEFINING: &[&str] = &[
    "KP", "VTO", "VT", "LAMBDA", "CGS", "CGD", "RD", "RS", "GAMMA", "PHI",
];

/// `SHOT_GAMMA2` is consumed by `codegen::ir::noise` (shot-noise Gamma-squared
/// override), NOT by the triode resolver — which is precisely why it was
/// missing from this list until the unknown-key check became a hard error and a
/// test caught it. `KF`/`AF` are likewise noise-only.
const TRIODE_HONORED: &[&str] = &[
    "MU",
    "EX",
    "KG1",
    "KP",
    "KVB",
    "IG_MAX",
    "VGK_ONSET",
    "LAMBDA",
    "CCG",
    "CGP",
    "CCP",
    "RGI",
    "MU_B",
    "SVAR",
    "EX_B",
    "KF",
    "AF",
    "RTH",
    "CTH",
    "VBIAS_ALPHA",
    "TAMB",
    "SHOT_GAMMA2",
];

const TRIODE_DEFINING: &[&str] = &[
    "MU",
    "EX",
    "KG1",
    "KP",
    "KVB",
    "IG_MAX",
    "VGK_ONSET",
    "LAMBDA",
    "CCG",
    "CGP",
    "CCP",
    "RGI",
    "MU_B",
    "SVAR",
    "EX_B",
];

const PENTODE_HONORED: &[&str] = &[
    "MU",
    "EX",
    "KG1",
    "KG2",
    "KP",
    "KVB",
    "ALPHA_S",
    "A_FACTOR",
    "BETA_FACTOR",
    "PARTITION_F",
    "SCREEN_FORM",
    "IG_MAX",
    "VGK_ONSET",
    "LAMBDA",
    "CCG",
    "CGP",
    "CCP",
    "RGI",
    "MU_B",
    "SVAR",
    "EX_B",
    "KF",
    "AF",
];

const PENTODE_DEFINING: &[&str] = &[
    "MU",
    "EX",
    "KG1",
    "KG2",
    "KP",
    "KVB",
    "ALPHA_S",
    "A_FACTOR",
    "BETA_FACTOR",
    "PARTITION_F",
    "SCREEN_FORM",
    "IG_MAX",
    "VGK_ONSET",
    "LAMBDA",
    "CCG",
    "CGP",
    "CCP",
    "RGI",
    "MU_B",
    "SVAR",
    "EX_B",
];

/// Op-amp keys are read by the `Element::Opamp` resolution loop in
/// [`crate::mna`] (`build()`), not by a codegen resolver — an op-amp is a 0D
/// linear VCCS stamped straight into `G`. `EN`/`IN`/`EN_FC`/`IN_FC` are the
/// Phase 4 input-referred noise parameters; `EN_FC`/`IN_FC` are parsed and
/// stored but not yet wired into the noise IR (v1 is white-only).
const OPAMP_HONORED: &[&str] = &[
    "AOL",
    "ROUT",
    "VSAT",
    "VCC",
    "VEE",
    "GBW",
    "SR",
    "VOH_DROP",
    "VOL_DROP",
    "AOL_TRANSIENT_CAP",
    "IB",
    "RIN",
    "EN",
    "IN",
    "EN_FC",
    "IN_FC",
];

/// `MODE` is read in [`crate::mna`] (`current_mode`); `VSCALE`/`G0`/`THD` by the
/// codegen VCA resolver.
const VCA_HONORED: &[&str] = &["VSCALE", "G0", "THD", "MODE"];

const LDR_HONORED: &[&str] = &["RMIN", "RMAX", "GAMMA", "TAU_A", "TAU_R"];

const LDR_DEFINING: &[&str] = &["RMIN", "RMAX", "GAMMA", "TAU_A", "TAU_R"];

/// `K1..K4` / `TAU1..TAU4` are read by name-construction (`format!("K{}", …)`)
/// in the glow resolver's section loop, not as string literals.
const GLOW_HONORED: &[&str] = &[
    "VO", "VM", "IK", "RS", "IHOLD", "ROFF", "RT", "K1", "K2", "K3", "K4", "TAU1", "TAU2", "TAU3",
    "TAU4", "IFLOOR", "KSUB", "D_AMP", "D_TKNEE", "D_THOLD",
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_type_tokens_map_to_classes() {
        assert_eq!(ModelClass::from_model_type("D"), Some(ModelClass::Diode));
        assert_eq!(ModelClass::from_model_type("d"), Some(ModelClass::Diode));
        assert_eq!(ModelClass::from_model_type("NPN"), Some(ModelClass::Bjt));
        assert_eq!(ModelClass::from_model_type("PNP2"), Some(ModelClass::Bjt));
        assert_eq!(ModelClass::from_model_type("NJF"), Some(ModelClass::Jfet));
        assert_eq!(ModelClass::from_model_type("PJ"), Some(ModelClass::Jfet));
        assert_eq!(
            ModelClass::from_model_type("NMOS"),
            Some(ModelClass::Mosfet)
        );
        assert_eq!(ModelClass::from_model_type("PM"), Some(ModelClass::Mosfet));
        assert_eq!(
            ModelClass::from_model_type("TRIODE"),
            Some(ModelClass::Triode)
        );
        assert_eq!(
            ModelClass::from_model_type("TUBE"),
            Some(ModelClass::Triode)
        );
        assert_eq!(ModelClass::from_model_type("VT"), Some(ModelClass::Triode));
        assert_eq!(ModelClass::from_model_type("VP"), Some(ModelClass::Pentode));
        assert_eq!(
            ModelClass::from_model_type("PENTODE"),
            Some(ModelClass::Pentode)
        );
        assert_eq!(ModelClass::from_model_type("OA"), Some(ModelClass::Opamp));
        assert_eq!(ModelClass::from_model_type("VCA"), Some(ModelClass::Vca));
        assert_eq!(ModelClass::from_model_type("LDR"), Some(ModelClass::Ldr));
        assert_eq!(ModelClass::from_model_type("NEON"), Some(ModelClass::Glow));
        // Unknown types have no table — callers must stay silent, not guess.
        assert_eq!(ModelClass::from_model_type("ZZTOP"), None);
        assert_eq!(ModelClass::from_model_type("SW"), None);
    }

    #[test]
    fn tables_have_no_duplicate_keys() {
        for class in ALL_CLASSES {
            let keys = class.honored();
            for (i, a) in keys.iter().enumerate() {
                for b in &keys[i + 1..] {
                    assert!(
                        !a.eq_ignore_ascii_case(b),
                        "{} honored table lists '{}' twice",
                        class.label(),
                        a
                    );
                }
            }
        }
    }

    #[test]
    fn tables_are_uppercase() {
        // `.model` keys are uppercased at parse time; a lowercase table entry
        // would still match (comparisons are case-insensitive) but would print
        // wrong in the "Accepted for this device: …" error list.
        for class in ALL_CLASSES {
            for k in class.honored() {
                assert_eq!(*k, k.to_ascii_uppercase(), "{} table", class.label());
            }
            for (k, _) in class.unimplemented() {
                assert_eq!(*k, k.to_ascii_uppercase(), "{} table", class.label());
            }
        }
    }

    #[test]
    fn defining_keys_are_a_subset_of_honored() {
        for class in ALL_CLASSES {
            for k in class.defining() {
                assert!(
                    class.is_honored(k),
                    "{}: defining key '{}' is not in the honored table",
                    class.label(),
                    k
                );
            }
        }
    }

    #[test]
    fn unimplemented_keys_are_not_also_honored() {
        // A key cannot be both read and "not modeled yet" — that pairing makes
        // the diagnostic a lie in one direction or the other.
        for class in ALL_CLASSES {
            for (k, _) in class.unimplemented() {
                assert!(
                    !class.is_honored(k),
                    "{}: '{}' is listed as both honored and unimplemented",
                    class.label(),
                    k
                );
            }
        }
    }
}
