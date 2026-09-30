//! JFET `.model` cards in the ngspice reference, as melange resolved them.
//!
//! ngspice's level-1 JFET is parameterised by `BETA` and `VTO`; melange also
//! reads `IDSS` (winning over `BETA`), fills an unset card from its tube-style
//! catalog or its defaults, and stores a P-channel pinch-off positive. A card
//! passed through as written can therefore describe a different device to
//! ngspice: `IDSS=` is "unrecognized ... ignored" there, leaving the reference
//! at its default `BETA` (65 uA at Vgs = 0 where melange ran 600 uA).
//!
//! So each JFET model card is replaced by the device melange resolved:
//! `BETA = IDSS / VP^2` (both engines' square law, `IDSS (1 - Vgs/VP)^2`),
//! `VTO` in the SPICE sign convention, and melange's `LAMBDA` and `IS`.
//! ngspice's level-1 gate junction has no emission coefficient (it is 1), so a
//! card whose `N` is not 1 has no ngspice twin and is refused.
//! melange stamps `CGS`/`CGD` as constant capacitors, where ngspice's are
//! bias-dependent depletion capacitances, so the card carries none and each
//! device gets explicit constant `C` elements instead: the reference is the
//! circuit melange built.

use std::collections::HashMap;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::device_types::DeviceParams;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::{Element, Netlist, ParseOptions};

use crate::spice_runner::SpiceError;

/// ngspice card for one resolved JFET model, or why there is none.
fn model_card(
    name: &str,
    p: &melange_solver::device_types::JfetParams,
) -> Result<String, SpiceError> {
    if p.n != 1.0 {
        return Err(SpiceError::DeckNotComparable(format!(
            "JFET model {name} has a gate emission coefficient N={}; ngspice's level-1 JFET \
             fixes it at 1, so no reference card matches the device melange builds",
            p.n
        )));
    }
    let (kind, vto) = if p.is_p_channel {
        ("PJF", -p.vp)
    } else {
        ("NJF", p.vp)
    };
    Ok(format!(
        ".model {name} {kind}(BETA={:e} VTO={:e} LAMBDA={:e} IS={:e})",
        p.idss / (p.vp * p.vp),
        vto,
        p.lambda,
        p.is,
    ))
}

/// `content` with every JFET `.model` statement (continuation lines included)
/// replaced by the card melange resolved, and each JFET's gate capacitances
/// added as constant capacitors. `source` is the pristine deck, parsed by
/// melange with unit variation off, as the melange side is. Unchanged when
/// the deck has no JFET.
pub(crate) fn translate_jfets_for_ngspice(
    content: &str,
    source: &str,
) -> Result<String, SpiceError> {
    let err = |e: String| SpiceError::ParseError(format!("JFET translation: {e}"));
    // No JFET element line (title excluded): nothing to translate, and the
    // deck need not be one melange parses (a hand-written ngspice reference).
    let has_jfet_line = source.lines().skip(1).any(|l| {
        crate::deck_guard::strip_inline_comment(l)
            .trim_start()
            .starts_with(['J', 'j'])
    });
    if !has_jfet_line {
        return Ok(content.to_string());
    }
    let mut netlist = Netlist::parse_with_options(
        source,
        ParseOptions {
            disable_unit_variation: true,
        },
    )
    .map_err(|e| err(e.to_string()))?;
    if !netlist
        .elements
        .iter()
        .any(|e| matches!(e, Element::Jfet { .. }))
    {
        return Ok(content.to_string());
    }
    netlist
        .expand_subcircuits()
        .map_err(|e| err(e.to_string()))?;
    let mna = MnaSystem::from_netlist(&netlist).map_err(|e| err(e.to_string()))?;
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna))
        .map_err(|e| err(e.to_string()))?;

    // Model name (upper-cased) -> card; one per model, which a nominal build
    // guarantees (no per-device jitter).
    let mut cards: HashMap<String, String> = HashMap::new();
    let mut caps = String::new();
    for (dev, slot) in mna.nonlinear_devices.iter().zip(&slots) {
        let DeviceParams::Jfet(p) = &slot.params else {
            continue;
        };
        let Some((model, ng, nd, ns)) = netlist.elements.iter().find_map(|e| match e {
            Element::Jfet {
                name,
                nd,
                ng,
                ns,
                model,
            } if name.eq_ignore_ascii_case(&dev.name) => Some((model, ng, nd, ns)),
            _ => None,
        }) else {
            return Err(err(format!(
                "{} is not a JFET element of the deck",
                dev.name
            )));
        };
        let card = model_card(model, p)?;
        if let Some(prev) = cards.insert(model.to_ascii_uppercase(), card.clone()) {
            if prev != card {
                return Err(err(format!(
                    "model {model} resolves to two different devices ({prev} / {card})"
                )));
            }
        }
        for (label, a, b, c) in [("cgs", ng, ns, p.cgs), ("cgd", ng, nd, p.cgd)] {
            if c > 0.0 {
                caps.push_str(&format!("C_melange_{label}_{} {a} {b} {c:e}\n", dev.name));
            }
        }
    }

    let mut out = String::with_capacity(content.len() + caps.len() + 256);
    let mut skipping = false;
    let mut placed_caps = false;
    for line in content.lines() {
        let t = crate::deck_guard::strip_inline_comment(line)
            .trim()
            .to_string();
        if skipping {
            if t.starts_with('+') {
                continue;
            }
            skipping = false;
        }
        let toks: Vec<&str> = t.split_whitespace().collect();
        if toks.len() >= 2 && toks[0].eq_ignore_ascii_case(".model") {
            let name = toks[1].split('(').next().unwrap_or(toks[1]);
            if let Some(card) = cards.get(&name.to_ascii_uppercase()) {
                out.push_str(card);
                out.push('\n');
                skipping = true;
                continue;
            }
        }
        if !placed_caps && t.eq_ignore_ascii_case(".end") {
            out.push_str(&caps);
            placed_caps = true;
        }
        out.push_str(line);
        out.push('\n');
    }
    if !placed_caps {
        out.push_str(&caps);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idss_card_becomes_the_resolved_beta_card() {
        let deck = "t\nVCC vcc 0 DC 9\nRd vcc d 22k\nJ1 d g s JX\nRg g 0 1Meg\nRs s 0 2k\n\
                    .model JX NJF(IDSS=6e-4 VTO=-0.8\n+ LAMBDA=0.004 CGS=2p)\n.end\n";
        let out = translate_jfets_for_ngspice(deck, deck).unwrap();
        let card = out.lines().find(|l| l.starts_with(".model JX")).unwrap();
        let beta: f64 = card
            .split("BETA=")
            .nth(1)
            .and_then(|r| r.split_whitespace().next())
            .unwrap()
            .parse()
            .unwrap();
        assert!((beta - 6e-4 / 0.64).abs() < 1e-18, "{card}");
        assert!(card.ends_with("VTO=-8e-1 LAMBDA=4e-3 IS=1e-14)"), "{card}");
        assert!(!out.contains("IDSS") && !out.contains("+ LAMBDA"), "{out}");
        assert!(out.contains("C_melange_cgs_J1 g s 2e-12\n.end"), "{out}");
    }

    #[test]
    fn a_p_channel_card_keeps_the_spice_sign() {
        let deck = "t\nVEE vee 0 DC -9\nRd vee d 22k\nJ1 d g s JP\nRg g 0 1Meg\nRs s 0 2k\n\
                    .model JP PJF(BETA=1e-3 VTO=-2)\n.end\n";
        let out = translate_jfets_for_ngspice(deck, deck).unwrap();
        assert!(out.contains(".model JP PJF(BETA=1e-3 VTO=-2e0"), "{out}");
    }

    #[test]
    fn a_gate_emission_coefficient_other_than_1_has_no_twin() {
        let deck =
            "t\nJ1 d g 0 JX\nRd d 0 1k\nRg g 0 1k\n.model JX NJF(BETA=1e-3 VTO=-2 N=1.5)\n.end\n";
        assert!(matches!(
            translate_jfets_for_ngspice(deck, deck),
            Err(SpiceError::DeckNotComparable(_))
        ));
    }

    #[test]
    fn a_deck_without_a_jfet_is_unchanged() {
        let deck = "t\nR1 a 0 1k\n.end\n";
        assert_eq!(translate_jfets_for_ngspice(deck, deck).unwrap(), deck);
    }
}
