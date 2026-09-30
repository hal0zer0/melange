//! Translate melange triode (`T`) elements into Koren B-source subcircuits for
//! the ngspice **reference** deck.
//!
//! ngspice parses a `T`-prefixed card as a lossy transmission line, so a
//! melange netlist containing a triode cannot be fed to ngspice unmodified
//! ("t_rec: transmission line z0 must be given"). This module rewrites each
//! `T<name> n_grid n_plate n_cathode MODEL` element into an `X` subcircuit call
//! plus a generated `.subckt` whose plate/grid currents are a **B-source twin
//! of melange's own Koren equation** (`crates/melange-devices/src/tube.rs`).
//!
//! What this buys the validate harness: a self-consistent cross-check of
//! melange's transient **solver** (NR + trapezoidal/BE integration + timestep)
//! against ngspice's solver, given an identical device equation. It does **not**
//! independently validate melange's tube *physics* (nor its non-standard
//! Leach-style grid current) — the twin reproduces melange's own equation. The
//! plate expression here is the one validated to 2.54e-3 V against ngspice-42 in
//! the solver crate's golden deck
//! (`crates/melange-solver/tests/golden/ngspice_ref/triode_cc_small_ref.cir.tmpl`),
//! extended with the grid-current source, `Vpk` floor, and optional Early-effect
//! multiplier a general translator needs.
//!
//! Scope (P1): sharp triodes (`svar = 0`), which is every netlist-authored
//! triode — the variable-mu blend and grid parameters are not `.model`-settable.
//! Variable-mu and pentodes are out of scope.

use std::collections::{BTreeSet, HashMap};

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::device_types::{DeviceParams, TubeParams};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::{Element, Netlist, ParseOptions};

use crate::spice_runner::SpiceError;

/// Melange triode `.model` types (any of these prefix a Koren triode).
const TRIODE_MODEL_TYPES: [&str; 3] = ["TRIODE", "VT", "TUBE"];

/// A triode `.model` as melange resolved it (card, catalog, defaults), for
/// the reference: the Koren plate law, the D&Z grid law, the grid's internal
/// resistance and the inter-electrode capacitances.
#[derive(PartialEq)]
struct TriodeParams {
    mu: f64,
    ex: f64,
    kg1: f64,
    kp: f64,
    kvb: f64,
    lambda: f64,
    gg: f64,
    xi: f64,
    cg: f64,
    rgi: f64,
    ccg: f64,
    cgp: f64,
    ccp: f64,
}

impl TriodeParams {
    fn from_resolved(tp: &TubeParams) -> Self {
        Self {
            mu: tp.mu,
            ex: tp.ex,
            kg1: tp.kg1,
            kp: tp.kp,
            kvb: tp.kvb,
            lambda: tp.lambda,
            gg: tp.gg,
            xi: tp.xi,
            cg: tp.cg,
            rgi: tp.rgi,
            ccg: tp.ccg,
            cgp: tp.cgp,
            ccp: tp.ccp,
        }
    }

    /// Emit the `.subckt` for this triode model. Port order `g p k` mirrors the
    /// `T` element's node order (grid-plate-cathode), so the `X` call binds
    /// `n_grid n_plate n_cathode` positionally.
    ///
    /// With `RGI`, both currents are evaluated at an internal grid `gi` behind
    /// a resistor `RGI` from the terminal, which is melange's model (the root
    /// of `v + RGI*Ig(v) = Vgk`). The inter-electrode capacitances sit between
    /// the terminals, as melange stamps them. Parasitic caps melange adds to a
    /// capacitor-free deck come from its build record
    /// (`with_parasitic_caps`), not from here.
    fn subckt(&self, model_name: &str) -> String {
        // Vpk floored at 1e-3 (mirrors tube.rs `plate_current`).
        let vpk = "max(V(p,k),1e-3)";
        // inner = KP*(1/MU + Vgk/sqrt(KVB + Vpk^2))
        let inner = format!(
            "{kp}*(1/{mu}+V(gi,k)/sqrt({kvb}+{vpk}*{vpk}))",
            kp = self.kp,
            mu = self.mu,
            kvb = self.kvb,
        );
        // E1 = (Vpk/KP)*ln(1+exp(inner))  (softplus; ngspice exp() only overflows
        // near inner≈709, unreachable for realistic triodes, and for inner>40
        // ln(1+exp(inner))≈inner agrees with melange's clamped safe_exp).
        let e1 = format!("({vpk}/{kp})*ln(1+exp({inner}))", kp = self.kp);
        // Ip_koren = 2*E1^EX/KG1 for E1>0; pwr(uramp(E1),EX) folds in the
        // (1+sgn(E1)) 2x convention and the E1<=0 -> 0 branch.
        let ip = format!(
            "2*pwr(uramp({e1}),{ex})/{kg1}",
            ex = self.ex,
            kg1 = self.kg1
        );
        // Early-effect multiplier only when non-zero (keeps the common lambda=0
        // deck identical to the validated golden form).
        let plate = if self.lambda != 0.0 {
            format!("({ip})*(1+{lambda}*{vpk})", lambda = self.lambda)
        } else {
            ip
        };
        // Grid current: Dempwolf & Zölzer DAFx-11 eq. (11),
        //   Ig = Gg * (softplus(Cg*Vgk)/Cg)^xi.
        // The softplus is written in the branch-free stable form
        //   softplus(x) = max(x,0) + ln(1 + exp(-|x|)),
        // so the exponent is never positive and ngspice cannot overflow it
        // while probing a large trial Vgk — a bare exp(Cg*Vgk) blows up past
        // Vgk ~ 71 V at the default Cg. `pwr` is |x|^y and the argument is
        // non-negative by construction.
        let x = format!("{cg}*V(gi,k)", cg = self.cg);
        let softplus = format!("(max({x},0)+ln(1+exp(-abs({x}))))/{cg}", cg = self.cg);
        let grid = format!("{gg}*pwr({softplus},{xi})", gg = self.gg, xi = self.xi,);
        // The internal grid: behind RGI, or the terminal itself.
        let grid_stopper = if self.rgi > 0.0 {
            format!("RGI g gi {:e}\n", self.rgi)
        } else {
            "VGI g gi DC 0\n".to_string()
        };
        let mut caps = String::new();
        for (label, a, b, c) in [
            ("CCG", "k", "g", self.ccg),
            ("CGP", "g", "p", self.cgp),
            ("CCP", "k", "p", self.ccp),
        ] {
            if c > 0.0 {
                caps.push_str(&format!("{label} {a} {b} {c:e}\n"));
            }
        }
        format!(
            "* Koren B-source twin of `.model {name} (TRIODE|VT|TUBE)` — self-consistent with melange/tube.rs.\n\
             .subckt MELANGE_TRIODE_{name} g p k\n\
             {grid_stopper}\
             BP p k I={plate}\n\
             BG gi k I={grid}\n\
             {caps}.ends\n",
            name = model_name,
        )
    }
}

/// Is `line` a triode element card (`T<name> g p k model`, exactly 5 tokens)?
/// The caller must exclude the title line (line 0) — a 5-word title beginning
/// with "T…" would otherwise look like a triode.
fn is_triode_line(line: &str) -> bool {
    let t = crate::deck_guard::strip_inline_comment(line).trim();
    if t.is_empty() || t.starts_with('*') || t.starts_with('.') {
        return false;
    }
    let toks: Vec<&str> = t.split_whitespace().collect();
    toks.len() == 5
        && toks[0]
            .chars()
            .next()
            .is_some_and(|c| c.eq_ignore_ascii_case(&'T'))
}

/// Translate all triode elements in `content` into Koren B-source subcircuits.
/// Returns `content` unchanged when it contains no triode element.
pub(crate) fn translate_tubes_for_ngspice(content: &str) -> Result<String, SpiceError> {
    // Fast path: skip the title line (line 0), scan the rest for a triode card.
    if !content.lines().skip(1).any(is_triode_line) {
        return Ok(content.to_string());
    }

    // Each triode model as melange resolved it (card, catalog, defaults;
    // nominal, as the validate build is), through the same resolver the
    // build uses.
    let err = |e: String| SpiceError::ParseError(format!("tube translation: {e}"));
    let mut netlist = Netlist::parse_with_options(
        content,
        ParseOptions {
            disable_unit_variation: true,
            disable_self_heating: true,
        },
    )
    .map_err(|e| err(e.to_string()))?;
    netlist
        .expand_subcircuits()
        .map_err(|e| err(e.to_string()))?;
    let mna = MnaSystem::from_netlist(&netlist).map_err(|e| err(e.to_string()))?;
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna))
        .map_err(|e| err(e.to_string()))?;
    // Every triode-type card is dropped (ngspice cannot parse the type), used
    // or not: a card whose triodes are all `.linearize`d has no X call left.
    let triode_cards: BTreeSet<String> = netlist
        .models
        .iter()
        .filter(|m| {
            TRIODE_MODEL_TYPES
                .iter()
                .any(|t| m.model_type.eq_ignore_ascii_case(t))
        })
        .map(|m| m.name.to_ascii_uppercase())
        .collect();
    let mut tube_models: HashMap<String, TriodeParams> = HashMap::new();
    for (dev, slot) in mna.nonlinear_devices.iter().zip(&slots) {
        let DeviceParams::Tube(tp) = &slot.params else {
            continue;
        };
        if tp.is_pentode() {
            continue;
        }
        let Some(model) = netlist.elements.iter().find_map(|e| match e {
            Element::Triode { name, model, .. } if name.eq_ignore_ascii_case(&dev.name) => {
                Some(model.to_ascii_uppercase())
            }
            _ => None,
        }) else {
            return Err(err(format!(
                "{} is not a triode element of the deck",
                dev.name
            )));
        };
        let params = TriodeParams::from_resolved(tp);
        if let Some(prev) = tube_models.get(&model) {
            if *prev != params {
                return Err(err(format!(
                    "model {model} resolves to two different triodes"
                )));
            }
        }
        tube_models.insert(model, params);
    }

    let mut out = String::with_capacity(content.len() + 256);
    let mut used: BTreeSet<String> = BTreeSet::new(); // deterministic subckt order

    for (i, line) in content.lines().enumerate() {
        // The title line is never an element or directive — pass it through.
        if i == 0 {
            out.push_str(line);
            out.push('\n');
            continue;
        }

        // Drop tube `.model` cards — ngspice cannot parse type TRIODE/VT/TUBE,
        // and the generated .subckt replaces them.
        if is_tube_model_line(line, &triode_cards) {
            continue;
        }

        if is_triode_line(line) {
            let toks: Vec<&str> = crate::deck_guard::strip_inline_comment(line)
                .split_whitespace()
                .collect();
            let (name, g, p, k, model) = (toks[0], toks[1], toks[2], toks[3], toks[4]);
            let model_uc = model.to_ascii_uppercase();
            if !tube_models.contains_key(&model_uc) {
                return Err(SpiceError::ParseError(format!(
                    "tube translation: triode '{name}' references model '{model}', \
                     which is not a triode (TRIODE/VT/TUBE) .model"
                )));
            }
            used.insert(model_uc.clone());
            // T<name> -> X<name>: 'X' prefix makes ngspice treat it as a subckt
            // call; original name is unique so no collision with melange X-cells.
            out.push_str(&format!("X{name} {g} {p} {k} MELANGE_TRIODE_{model_uc}\n"));
            continue;
        }

        out.push_str(line);
        out.push('\n');
    }

    // Append one subckt per distinct triode model actually used.
    for model_uc in &used {
        out.push_str(&tube_models[model_uc].subckt(model_uc));
    }

    Ok(out)
}

/// Is `line` a `.model <name> <TRIODE|VT|TUBE>(...)` card for a known triode
/// model? Matched by name against the resolved model set so we only drop cards
/// the translator is replacing.
fn is_tube_model_line(line: &str, triode_cards: &BTreeSet<String>) -> bool {
    let t = line.trim();
    // `.get(..6)` (not `t[..6]`) — a byte slice panics when byte 6 falls inside
    // a multibyte char (e.g. an em-dash in a comment/header, common in real
    // decks). `.get` returns None on a non-char-boundary, and a line whose
    // first 6 bytes span a multibyte char cannot be ".model" anyway, so the
    // semantics are exactly preserved.
    if !t.get(..6).is_some_and(|p| p.eq_ignore_ascii_case(".model")) {
        return false;
    }
    // `.model <name> <type>...` — the name is token 1.
    let toks: Vec<&str> = t.split_whitespace().collect();
    toks.get(1)
        .is_some_and(|name| triode_cards.contains(&name.to_ascii_uppercase()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DECK: &str = "12AX7 CC stage\n\
        VIN in 0 DC 0\n\
        Rin in 0 1Meg\n\
        Cin in grid 100n\n\
        Rg grid 0 1Meg\n\
        T1 grid plate cathode 12AX7\n\
        Rk cathode 0 1.5k\n\
        Rp vcc plate 100k\n\
        Vcc vcc 0 DC 250\n\
        .model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)\n\
        .end\n";

    /// An inline comment on the `T` line is not a token (see the op-amp twin).
    #[test]
    fn an_inline_comment_does_not_hide_a_triode() {
        let deck = DECK.replace(
            "T1 grid plate cathode 12AX7",
            "T1 grid plate cathode 12AX7 ; V1A",
        );
        let out = translate_tubes_for_ngspice(&deck).unwrap();
        assert!(
            out.contains("XT1 grid plate cathode MELANGE_TRIODE_12AX7"),
            "{out}"
        );
    }

    #[test]
    fn rewrites_triode_to_subckt_call() {
        let out = translate_tubes_for_ngspice(DECK).unwrap();
        // T element replaced by an X call with grid-plate-cathode order preserved.
        assert!(out.contains("XT1 grid plate cathode MELANGE_TRIODE_12AX7"));
        assert!(!out.lines().any(is_triode_line));
        // Tube .model card dropped (its param body is gone; the subckt bakes
        // KG1 as "/1060", never "KG1=1060").
        assert!(!out.contains("KG1=1060"));
        assert!(!out.lines().any(|l| {
            let u = l.trim().to_uppercase();
            u.starts_with(".MODEL") && u.contains("12AX7")
        }));
        assert!(out.contains(".subckt MELANGE_TRIODE_12AX7 g p k"));
        assert!(out.contains("BP p k I="));
        assert!(out.contains("BG gi k I="));
        // No RGI: the internal grid is the terminal.
        assert!(out.contains("VGI g gi DC 0"));
        // No parasitic caps here: melange's are added from its build record.
        assert!(!out.contains("10p"));
        assert!(out.contains(".ends"));
        // Baked Koren params appear in the plate expression.
        assert!(out.contains("/1060")); // KG1
        assert!(out.contains("1/100")); // 1/MU
    }

    /// The card's inter-electrode capacitances and grid resistance reach the
    /// reference, as melange stamps and solves them (they used to be dropped:
    /// a CGP = 1.7 pF two-stage preamp read 2.7 % against its reference).
    #[test]
    fn caps_and_rgi_reach_the_reference() {
        let deck = DECK.replace("KVB=300)", "KVB=300 CCG=1.6p CGP=1.7p CCP=0.46p RGI=2k)");
        let out = translate_tubes_for_ngspice(&deck).unwrap();
        assert!(out.contains("RGI g gi 2e3\n"), "{out}");
        assert!(out.contains("CCG k g 1.6e-12\n"), "{out}");
        assert!(out.contains("CGP g p 1.69999"), "{out}");
        assert!(out.contains("CCP k p 4.6000"), "{out}");
    }

    #[test]
    fn passthrough_when_no_triode() {
        let deck = "RC lowpass\nR1 in out 1k\nC1 out 0 100n\n.end\n";
        assert_eq!(translate_tubes_for_ngspice(deck).unwrap(), deck);
    }

    #[test]
    fn title_line_starting_with_t_is_not_a_triode() {
        // 5-word title beginning with "Tube…" must not be rewritten.
        let deck = "Tube Amp Test Deck\nR1 in out 1k\nC1 out 0 100n\n.end\n";
        assert_eq!(translate_tubes_for_ngspice(deck).unwrap(), deck);
    }

    /// A card without the Koren parameters gets melange's resolved values,
    /// as the build does, rather than a reference of its own.
    #[test]
    fn an_underspecified_card_gets_the_resolved_values() {
        let deck = DECK.replace(
            "TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)",
            "TRIODE(MU=100 EX=1.4 KG1=1060)",
        );
        let out = translate_tubes_for_ngspice(&deck).unwrap();
        assert!(out.contains("/1060"), "{out}");
    }

    #[test]
    fn multibyte_comment_does_not_panic() {
        // Regression: is_tube_model_line used a byte slice `t[..6]` that panicked
        // when byte 6 fell inside a multibyte char. "* a — b" puts an em-dash
        // (3 bytes) at bytes 4..7, so byte 6 is a non-char-boundary — exactly the
        // real-deck header case that fired on every melange-circuits tube netlist.
        let deck = "12AX7 CC stage\n\
            * a — b\n\
            VIN in 0 DC 0\n\
            Cin in grid 100n\n\
            T1 grid plate cathode 12AX7\n\
            Rk cathode 0 1.5k\n\
            Rp vcc plate 100k\n\
            Vcc vcc 0 DC 250\n\
            .model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)\n\
            .end\n";
        let out = translate_tubes_for_ngspice(deck).unwrap();
        assert!(out.contains("XT1 grid plate cathode MELANGE_TRIODE_12AX7"));
        assert!(out.contains("* a — b")); // the multibyte comment survives verbatim
    }
}
