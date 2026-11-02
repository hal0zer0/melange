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
//! level-1 card whose `N` is not 1 has no ngspice twin and is refused. A
//! `LEVEL=2` (Parker–Skellern) device becomes an ngspice level-2 card carrying
//! BETA and every law key melange resolved, and `N` (JFET2 has it). A
//! `LEVEL=3` device (melange's symmetric subthreshold law) has no ngspice
//! law at all: ngspice never sees the card or the level. Its `J` element
//! becomes an `X` call on a generated `.subckt` whose channel and gate
//! junctions are B-sources transcribing melange's own formulas, the
//! junctions with the same ±40 exponent clamp and forward continuation as
//! `junction_exp` (an ngspice `D` element switches to SPICE2's cubic reverse
//! form below −3·N·Vt, which is not melange's law), so the twin checks
//! melange's solve of the law, not the law's physics.
//! melange stamps `CGS`/`CGD` as constant capacitors, where ngspice's are
//! bias-dependent depletion capacitances, so the card carries none and each
//! device gets explicit constant `C` elements instead: the reference is the
//! circuit melange built.

use std::collections::{BTreeMap, HashMap};

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::device_types::{DeviceParams, JfetParams, JfetSymParams};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::{Element, Netlist, ParseOptions};

use crate::spice_runner::SpiceError;

/// The ngspice twin of one resolved JFET model.
#[derive(Clone, PartialEq, Debug)]
enum Twin {
    /// A `.model` card ngspice reads with its own JFET law.
    Card(String),
    /// A `.subckt` transcribing melange's law (LEVEL=3, which ngspice lacks);
    /// the `.model` card is dropped and the `J` element becomes an `X` call.
    Subckt(String),
}

/// The subcircuit name ngspice instantiates for a LEVEL=3 model.
fn sym_subckt_name(model_uc: &str) -> String {
    format!("MELANGE_JFET3_{model_uc}")
}

/// The `.subckt` for a LEVEL=3 model: melange's symmetric subthreshold
/// channel and its gate junctions as B-sources, port order `d g s` (the `J`
/// element's). Every number is the resolved device's, in N-channel
/// equivalent form with the polarity `S` applied as melange applies it, so
/// the expression is the generated `jfet_sym_evaluate` line for line:
/// `G(x) = (a·(max(z, 0) + ln(1 + e^(−|z|))))²`, `z = x/a`, `a = 2·NSLOPE·Vt`;
/// `Id = S·BETA·(1 + λ|Vds|)·[G(S·Vgs − VTO) − G(S·Vgd − VTO)]`; each
/// junction `S·IS·(E(S·V/(N·Vt)) − 1)` with `E` the extended exponential of
/// `junction_exp` (clamped at −40, true exponential to `x_max`, then linear).
fn sym_subckt(model_uc: &str, p: &JfetParams, sym: &JfetSymParams) -> String {
    let s: f64 = if p.is_p_channel { -1.0 } else { 1.0 };
    let vto_eff = s * p.vp; // negative: the N-channel equivalent pinch-off
    let beta = p.idss / (p.vp * p.vp);
    let a = sym.scale();
    let g = |x: &str| format!("(({a:e})*(max(({x})/({a:e}),0)+ln(1+exp(-abs(({x})/({a:e}))))))");
    let gs = g(&format!("(({s})*V(g,s)-({vto_eff:e}))"));
    let gd = g(&format!("(({s})*V(g,d)-({vto_eff:e}))"));
    let mut out = format!(
        "* melange JFET symmetric subthreshold law, model {model_uc}: VTO={:e} IDSS={:e} \
         LAMBDA={:e} NSLOPE={:e} IS={:e} N={:e}, transcribed from the generated code\n\
         .subckt {} d g s\n\
         B_ch d s I = {{({s})*({beta:e})*(1+({:e})*abs(V(d,s)))*({gs}*{gs}-{gd}*{gd})}}\n",
        p.vp,
        p.idss,
        p.lambda,
        sym.nslope,
        p.is,
        p.n,
        sym_subckt_name(model_uc),
        p.lambda,
    );
    if p.is > 0.0 {
        let n_vt = p.gate_n_vt();
        // junction_exp's forward continuation: a true exponential up to the
        // current ceiling 1e3 A, then linear with the slope at the join.
        let x_max = 40f64.max(1e3f64.ln() - p.is.ln());
        let e_x_max = x_max.exp();
        let e = |x: &str| {
            format!(
                "(({x})<=40 ? exp(max(({x}),-40)) : (({x})<=({x_max:e}) ? exp({x}) : \
                 ({e_x_max:e})*(1+(({x})-({x_max:e})))))"
            )
        };
        let xgs = format!("(({s})*V(g,s)/({n_vt:e}))");
        let xgd = format!("(({s})*V(g,d)/({n_vt:e}))");
        out.push_str(&format!(
            "B_gs g s I = {{({s})*({is:e})*({}-1)}}\nB_gd g d I = {{({s})*({is:e})*({}-1)}}\n",
            e(&xgs),
            e(&xgd),
            is = p.is,
        ));
    }
    out.push_str(".ends\n");
    out
}

/// ngspice twin for one resolved JFET model, or why there is none.
fn model_card(name: &str, p: &JfetParams) -> Result<Twin, SpiceError> {
    let (kind, vto) = if p.is_p_channel {
        ("PJF", -p.vp)
    } else {
        ("NJF", p.vp)
    };
    if let Some(sym) = &p.sym {
        return Ok(Twin::Subckt(sym_subckt(&name.to_ascii_uppercase(), p, sym)));
    }
    if let Some(ps) = &p.ps {
        return Ok(Twin::Card(format!(
            ".model {name} {kind}(LEVEL=2 BETA={:e} VTO={:e} LAMBDA={:e} IS={:e} N={:e} \
             VST={:e} MVST={:e} P={:e} Q={:e} Z={:e} XI={:e} MXI={:e} PB={:e})",
            ps.beta,
            vto,
            p.lambda,
            p.is,
            p.n,
            ps.vst,
            ps.mvst,
            ps.p,
            ps.q,
            ps.z,
            ps.xi,
            ps.mxi,
            ps.vbi,
        )));
    }
    if p.n != 1.0 {
        return Err(SpiceError::DeckNotComparable(format!(
            "JFET model {name} has a gate emission coefficient N={}; ngspice's level-1 JFET \
             fixes it at 1, so no reference card matches the device melange builds",
            p.n
        )));
    }
    Ok(Twin::Card(format!(
        ".model {name} {kind}(BETA={:e} VTO={:e} LAMBDA={:e} IS={:e})",
        p.idss / (p.vp * p.vp),
        vto,
        p.lambda,
        p.is,
    )))
}

/// `content` with every JFET `.model` statement (continuation lines included)
/// replaced by the card melange resolved, and each JFET's gate capacitances
/// added as constant capacitors. `source` is the pristine deck, parsed by
/// melange with unit variation off, as the melange side is. Unchanged when
/// the deck has no JFET.
pub fn translate_jfets_for_ngspice(content: &str, source: &str) -> Result<String, SpiceError> {
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
            disable_self_heating: true,
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

    // Model name (upper-cased) -> twin; one per model, which a nominal build
    // guarantees (no per-device jitter).
    let mut cards: HashMap<String, Twin> = HashMap::new();
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
                    "model {model} resolves to two different devices ({prev:?} / {card:?})"
                )));
            }
        }
        for (label, a, b, c) in [("cgs", ng, ns, p.cgs), ("cgd", ng, nd, p.cgd)] {
            if c > 0.0 {
                caps.push_str(&format!("C_melange_{label}_{} {a} {b} {c:e}\n", dev.name));
            }
        }
    }

    // The LEVEL=3 subcircuits, appended once each in a deterministic order.
    let subckts: BTreeMap<&String, &String> = cards
        .iter()
        .filter_map(|(m, t)| match t {
            Twin::Subckt(s) => Some((m, s)),
            Twin::Card(_) => None,
        })
        .collect();
    let mut tail = caps;
    for s in subckts.values() {
        tail.push_str(s);
    }

    let mut out = String::with_capacity(content.len() + tail.len() + 256);
    let mut skipping = false;
    let mut placed_tail = false;
    for (i, line) in content.lines().enumerate() {
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
            match cards.get(&name.to_ascii_uppercase()) {
                Some(Twin::Card(card)) => {
                    out.push_str(card);
                    out.push('\n');
                    skipping = true;
                    continue;
                }
                // ngspice never sees a LEVEL=3 card: the subcircuit is the twin.
                Some(Twin::Subckt(_)) => {
                    skipping = true;
                    continue;
                }
                None => {}
            }
        }
        // A `J` element of a LEVEL=3 model becomes a call on its subcircuit
        // (the title line is never an element).
        if i > 0 && toks.len() >= 5 && toks[0].starts_with(['J', 'j']) {
            let model_uc = toks[4].to_ascii_uppercase();
            if subckts.contains_key(&model_uc) {
                out.push_str(&format!(
                    "X{} {} {} {} {}\n",
                    toks[0],
                    toks[1],
                    toks[2],
                    toks[3],
                    sym_subckt_name(&model_uc)
                ));
                continue;
            }
        }
        if !placed_tail && t.eq_ignore_ascii_case(".end") {
            out.push_str(&tail);
            placed_tail = true;
        }
        out.push_str(line);
        out.push('\n');
    }
    if !placed_tail {
        out.push_str(&tail);
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

    /// A LEVEL=3 device reaches ngspice as a subcircuit of B-sources: the
    /// level and its card never do.
    #[test]
    fn a_level3_device_becomes_a_subckt_and_ngspice_never_sees_the_level() {
        let deck = "t\nR1 in d 27k\nJ1 d g 0 JX  ; the VVR\nRg g 0 1Meg\nC1 d 0 1n\n\
                    .model JX NJF(LEVEL=3 VTO=-2.5 BETA=1.6m\n+ LAMBDA=4m NSLOPE=1.2 CGS=2p)\n.end\n";
        let out = translate_jfets_for_ngspice(deck, deck).unwrap();
        assert!(!out.to_ascii_uppercase().contains("LEVEL"), "{out}");
        assert!(
            !out.contains(".model") && !out.contains("NSLOPE=1.2 CGS"),
            "{out}"
        );
        assert!(out.contains("\nXJ1 d g 0 MELANGE_JFET3_JX\n"), "{out}");
        assert!(!out.contains("\nJ1 "), "{out}");
        let sub = out
            .split(".subckt MELANGE_JFET3_JX d g s\n")
            .nth(1)
            .expect("one subckt");
        assert!(
            sub.contains("B_ch d s I = {(1)*(1.6e-3)*(1+(4e-3)*abs(V(d,s)))*("),
            "{sub}"
        );
        // a = 2·1.2·Vt, VTO as the N-channel equivalent.
        let a = 2.0 * 1.2 * melange_devices::VT_ROOM;
        assert!(
            sub.contains("(1)*V(g,s)-(-2.5e0)") && sub.contains(&format!("/({a:e}),0)")),
            "{sub}"
        );
        assert!(
            sub.contains("B_gs g s I = {(1)*(1e-14)*(") && sub.contains("B_gd g d I ="),
            "{sub}"
        );
        assert!(sub.contains("<=40 ? exp(max("), "{sub}");
        assert!(out.contains("C_melange_cgs_J1 g 0 2e-12\n"), "{out}");
        assert!(out.trim_end().ends_with(".ends\n.end"), "{out}");
        // The J line is gone entirely (its inline comment with it): the name
        // appears on the X call and the .subckt line only.
        assert_eq!(out.matches("MELANGE_JFET3_JX").count(), 2, "{out}");
    }

    /// IS = 0 is the law without the junctions; a P-channel device carries
    /// the polarity as melange applies it.
    #[test]
    fn a_level3_subckt_follows_is_and_polarity() {
        let deck = "t\nR1 in d 27k\nJ1 d g 0 JP\nRg g 0 1Meg\n\
                    .model JP PJF(LEVEL=3 VTO=-1.5 BETA=1m IS=0)\n.end\n";
        let out = translate_jfets_for_ngspice(deck, deck).unwrap();
        assert!(!out.contains("B_gs") && !out.contains("B_gd"), "{out}");
        assert!(
            out.contains("B_ch d s I = {(-1)*(1e-3)*(1+(0e0)*abs(V(d,s)))*("),
            "{out}"
        );
        assert!(out.contains("((-1)*V(g,s)-(-1.5e0))"), "{out}");
    }
}
