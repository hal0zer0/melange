//! Shared `.model` card validation and lookup helpers.

use super::*;

impl CircuitIR {
    /// Warn on unrecognized .model parameters (typo protection).
    /// Check every `.model` parameter against what melange does with it.
    ///
    /// Three outcomes, because a `.model` key can be wrong in two very different
    /// ways and collapsing them serves neither:
    ///
    /// * **Honored** — melange reads it. Silent.
    /// * **Recognized but unimplemented** — a real SPICE parameter melange does
    ///   not model yet (`unimplemented`). Warns, naming what the omission costs.
    ///   NOT an error: these arrive on authentic vendor model cards, and
    ///   refusing them would mean melange rejects genuine SPICE decks over a gap
    ///   of its own. The warning is the honest report of that gap.
    /// * **Unknown** — not a valid parameter for this device type at all. **Hard
    ///   error**, with the accepted keys and an alias hint where one is known.
    ///
    /// The third case used to warn and continue, which is how `VP=` on a JFET
    /// card (the datasheet spelling; SPICE uses `VTO`, and the sign convention
    /// differs) could be silently discarded while the deck still biased
    /// correctly off the built-in catalog — producing a right answer for the
    /// wrong reason, which is worse than a wrong answer.
    pub(super) fn check_model_params(
        netlist: &Netlist,
        model_name: &str,
        class: ModelClass,
    ) -> Result<(), CodegenError> {
        let honored = class.honored();
        // Every refusal names the card and its device class; `melange` adds
        // the card's netlist line (line numbers are not carried past parsing).
        let card = format!(".model {model_name} ({} card)", class.label());
        let Some(m) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
        else {
            return Ok(());
        };
        for (key, value) in &m.params {
            let upper = key.to_ascii_uppercase();
            if honored.iter().any(|k| k.eq_ignore_ascii_case(&upper)) {
                continue;
            }
            if let Some(note) = class.refused_note(&upper) {
                if *value == 0.0 {
                    continue;
                }
                return Err(CodegenError::InvalidConfig(format!(
                    "{card}: {upper}={value} is refused: {note}."
                )));
            }
            if crate::model_params::notice_if_unimplemented(model_name, class, &upper) {
                continue;
            }
            // A RETIRED key is refused with its conversion, not reported as a
            // typo: the deck is not misspelled, it is written against a device
            // law melange no longer has.
            if let Some(note) = class.retired_note(&upper) {
                return Err(CodegenError::InvalidConfig(format!(
                    "{card}: parameter '{key}' is RETIRED — {note}. \
                     Accepted for this device: {}",
                    honored.join(", ")
                )));
            }
            let hint = crate::model_params::alias_hint(class, &upper);
            return Err(CodegenError::InvalidConfig(format!(
                "{card}: unknown parameter '{key}'.{hint} Accepted for this device: {}",
                honored.join(", ")
            )));
        }
        Ok(())
    }

    /// Warn when a `.model` card resolves entirely to the hardcoded default
    /// device because its name matches no built-in catalog part **and** it
    /// supplies none of its device-defining parameters.
    ///
    /// This is the "plausible numbers, wrong circuit" trap: a typo'd model name
    /// (`.model 12AX8 TRIODE()`) compiles silently as the default device (a
    /// 12AX7 triode, EL84 pentode, SPICE-default diode, …) with no diagnostic.
    ///
    /// It only fires for a *declared-but-underspecified* card. A device that
    /// references a **never-declared** model already hard-errors in the parser
    /// (`references model '…' which is not defined`), so that case never reaches
    /// here. Stays silent on a catalog hit and on any card that specifies a
    /// defining parameter — a fully custom off-catalog part is legitimate and
    /// common, so specifying even one defining key suppresses the warning.
    ///
    /// The class's electrical-identity parameter set (`ModelClass::defining()`
    /// — the recognized keys minus universal add-ons like KF/AF/RTH/CTH/TAMB,
    /// which do not define which device this is) decides "underspecified". A
    /// class with no such set never warns.
    pub(super) fn warn_unresolved_model(
        netlist: &Netlist,
        model_name: &str,
        catalog_hit: bool,
        class: ModelClass,
        default_desc: &str,
    ) {
        let defining_keys = class.defining();
        if defining_keys.is_empty() || catalog_hit {
            return;
        }
        let Some(card) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
        else {
            return;
        };
        let supplied_defining = card.params.iter().any(|(key, _)| {
            let upper = key.to_ascii_uppercase();
            defining_keys.iter().any(|k| k.eq_ignore_ascii_case(&upper))
        });
        if !supplied_defining {
            crate::diag_warn!(
                ".model {}: name matches no built-in catalog part and no \
                 device-defining parameter was supplied — compiling as {}. A \
                 typo'd model name silently becomes the default device; use an \
                 exact catalog name or specify the device parameters.",
                model_name,
                default_desc,
            );
        }
    }

    /// Look up a parameter from a `.model` directive, case-insensitive.
    pub(super) fn lookup_model_param(
        netlist: &Netlist,
        model_name: &str,
        param_name: &str,
    ) -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param_name))
                    .map(|(_, v)| *v)
            })
    }
}
