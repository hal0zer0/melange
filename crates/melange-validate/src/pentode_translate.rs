//! Translate melange pentode (`P`) elements into Reefman/Koren B-source
//! subcircuits for the ngspice **reference** deck. Analogous to
//! [`crate::tube_translate`] for triodes.
//!
//! ngspice has no native pentode and parses a `P`-prefixed card as nothing it
//! can solve, so a melange netlist containing a pentode cannot be fed to
//! ngspice unmodified. This module rewrites each
//! `P<name> plate grid cath screen [supp] MODEL` element into an `X` subcircuit
//! call plus a generated `.subckt` whose plate / screen / grid currents are a
//! **B-source twin of melange's own pentode equations**
//! (`crates/melange-devices/src/tube.rs`, `KorenPentode`).
//!
//! What this buys the validate harness: a self-consistent cross-check of
//! melange's transient **solver** (NR + trapezoidal/BE integration + timestep)
//! against ngspice's solver, given an identical device equation. It does **not**
//! independently validate melange's pentode *physics* — the twin reproduces
//! melange's own Reefman "Derk"/"DerkE" and Classical-Koren equations. (Same
//! honesty caveat as the triode twin: it arbitrates the solver, not the model.)
//!
//! Scope (this pass): **sharp pentodes** (`svar == 0`) across all three screen
//! forms — Rational (Reefman §4.4), Exponential (Reefman "DerkE" §4.5), and
//! Classical (Norman Koren 1996 / Cohen-Hélie 2010). **Variable-mu** pentodes
//! (`svar > 0`) are a NAMED FOLLOW-UP, not covered here: the sole such deck in
//! the library is `6k7-varimu-stage.cir`, and a var-mu card errors out of this
//! translator rather than being silently modeled sharp.
//!
//! Parameters come from the **catalog**: every netlist pentode card is a bare
//! `VP()` whose model *name* is the entire specification. Resolution mirrors
//! codegen's `resolve_pentode_params` (explicit `.model` param > catalog entry),
//! and an unrecognized model name is a **hard error** — a silent default would
//! model a different tube and make validation lie.

use std::collections::{BTreeSet, HashMap};

use melange_devices::catalog::tubes::lookup_pentode;
use melange_devices::tube::ScreenForm;
use melange_solver::parser::{Element, Model, Netlist};

use crate::spice_runner::SpiceError;

/// Melange pentode `.model` types (either prefixes a pentode).
const PENTODE_MODEL_TYPES: [&str; 2] = ["VP", "PENTODE"];

/// Default grid-current parameters, mirroring `melange_devices::tube`
/// (`DEFAULT_IG_MAX` / `DEFAULT_VGK_ONSET`), used only when neither the
/// `.model` card nor the catalog entry supplies a value.
const DEFAULT_IG_MAX: f64 = 2e-3;
const DEFAULT_VGK_ONSET: f64 = 0.5;

/// Resolved sharp-pentode parameters for one `.model`.
struct PentodeParams {
    screen_form: ScreenForm,
    mu: f64,
    ex: f64,
    kg1: f64,
    kg2: f64,
    kp: f64,
    kvb: f64,
    // Reefman §4.4/§4.5 shape params (ignored on the Classical path).
    alpha_s: f64,
    a_factor: f64,
    beta_factor: f64,
    // Leach grid-current params (shared across all forms).
    ig_max: f64,
    vgk_onset: f64,
}

impl PentodeParams {
    /// Resolve a pentode model, mirroring codegen's `resolve_pentode_params`:
    /// explicit `.model` param > catalog entry (matched by name) > error. Never
    /// falls back to a hardcoded default on an unrecognized name.
    fn from_model(m: &Model) -> Result<Self, SpiceError> {
        let cat = lookup_pentode(&m.name);
        let get = |key: &str| {
            m.params
                .iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(key))
                .map(|(_, v)| *v)
        };

        // screen_form: explicit SCREEN_FORM=0|1|2 > catalog > default Rational.
        let screen_form = if let Some(v) = get("SCREEN_FORM") {
            if v == 0.0 {
                ScreenForm::Rational
            } else if v == 1.0 {
                ScreenForm::Exponential
            } else if v == 2.0 {
                ScreenForm::Classical
            } else {
                return Err(SpiceError::ParseError(format!(
                    "pentode translation: model '{}' SCREEN_FORM must be 0 (Rational), \
                     1 (Exponential) or 2 (Classical), got {v}",
                    m.name
                )));
            }
        } else {
            cat.map(|c| c.screen_form).unwrap_or(ScreenForm::Rational)
        };

        // Variable-mu is out of scope this pass — refuse rather than model sharp.
        let svar = get("SVAR").or_else(|| cat.map(|c| c.svar)).unwrap_or(0.0);
        if svar > 0.0 {
            return Err(SpiceError::ParseError(format!(
                "pentode translation: model '{}' is variable-mu (SVAR={svar} > 0), which \
                 is not yet supported by the ngspice translator (sharp pentodes only). \
                 Variable-mu ngspice translation is a pending follow-up.",
                m.name
            )));
        }

        // Core Koren/Derk params: explicit > catalog > hard error.
        let require = |key: &str, cat_val: Option<f64>| -> Result<f64, SpiceError> {
            get(key).or(cat_val).ok_or_else(|| {
                SpiceError::ParseError(format!(
                    "pentode translation: model '{}' is missing required parameter {} and is \
                     not a known catalog pentode — specify MU, EX, KG1, KG2, KP, KVB (and for \
                     Rational/Exponential screen forms ALPHA_S) on the .model card, or use an \
                     exact catalog part name",
                    m.name, key
                ))
            })
        };

        let mu = require("MU", cat.map(|c| c.mu))?;
        let ex = require("EX", cat.map(|c| c.ex))?;
        let kg1 = require("KG1", cat.map(|c| c.kg1))?;
        let kg2 = require("KG2", cat.map(|c| c.kg2))?;
        let kp = require("KP", cat.map(|c| c.kp))?;
        let kvb = require("KVB", cat.map(|c| c.kvb))?;

        // Derk shape params: required on the §4.4/§4.5 paths, ignored on
        // Classical (which uses only mu/ex/kg1/kg2/kp/kvb).
        let (alpha_s, a_factor, beta_factor) = if matches!(screen_form, ScreenForm::Classical) {
            (0.0, 0.0, 0.0)
        } else {
            let alpha_s = require("ALPHA_S", cat.map(|c| c.alpha_s))?;
            if alpha_s <= 0.0 {
                return Err(SpiceError::ParseError(format!(
                    "pentode translation: model '{}' ALPHA_S must be > 0 for a Reefman Derk \
                     pentode (the model degenerates to Ip=0 at ALPHA_S=0), got {alpha_s}",
                    m.name
                )));
            }
            let a_factor = get("A_FACTOR")
                .or_else(|| cat.map(|c| c.a_factor))
                .unwrap_or(0.0);
            let beta_factor = get("BETA_FACTOR")
                .or_else(|| cat.map(|c| c.beta_factor))
                .unwrap_or(0.0);
            (alpha_s, a_factor, beta_factor)
        };

        let ig_max = get("IG_MAX")
            .or_else(|| cat.map(|c| c.ig_max))
            .unwrap_or(DEFAULT_IG_MAX);
        let vgk_onset = get("VGK_ONSET")
            .or_else(|| cat.map(|c| c.vgk_onset))
            .unwrap_or(DEFAULT_VGK_ONSET);

        Ok(Self {
            screen_form,
            mu,
            ex,
            kg1,
            kg2,
            kp,
            kvb,
            alpha_s,
            a_factor,
            beta_factor,
            ig_max,
            vgk_onset,
        })
    }

    /// Emit the `.subckt` for this pentode model. Port order `p g k s`
    /// (plate-grid-cathode-screen) mirrors the `P` element's node order, so the
    /// `X` call binds `plate grid cath screen` positionally.
    ///
    /// Currents are B-source twins of `KorenPentode`'s `plate_current` /
    /// `screen_current` / `grid_current`. Pure-constant subexpressions (α, the
    /// coefficient group, the `1/Kg1 − 1/Kg2` base) are precomputed here and
    /// baked as numbers; only the voltage-dependent parts stay symbolic.
    fn subckt(&self, model_name: &str, add_parasitics: bool) -> Result<String, SpiceError> {
        // Shared guards (mirror tube.rs clamps).
        let vg2k = "max(V(s,k),1e-3)";
        let vpk = "max(V(p,k),0)";

        let (ip, ig2) = match self.screen_form {
            ScreenForm::Rational | ScreenForm::Exponential => {
                // Derk shared Ip0 chain.
                let s_denom = format!("sqrt({kvb}+{vg2k}*{vg2k})", kvb = self.kvb);
                let inner = format!(
                    "{kp}*(1/{mu}+V(g,k)/{s})",
                    kp = self.kp,
                    mu = self.mu,
                    s = s_denom
                );
                let softplus = format!("ln(1+exp({inner}))");
                let e1 = format!("({vg2k}/{kp})*{sp}", kp = self.kp, sp = softplus);
                // Ip0 = E1^ex, with uramp giving the deep-cutoff (E1<=0) -> 0.
                let ip0 = format!("pwr(uramp({e1}),{ex})", ex = self.ex);

                // Pure constants.
                let alpha = 1.0 - (self.kg1 / self.kg2) * (1.0 + self.alpha_s);
                let coeff = alpha / self.kg1 + self.alpha_s / self.kg2;
                let base = 1.0 / self.kg1 - 1.0 / self.kg2;
                let linear = format!("{a}*{vpk}/{kg1}", a = self.a_factor, kg1 = self.kg1);

                let (f, h) = match self.screen_form {
                    ScreenForm::Rational => {
                        // scale = 1/(1+β·Vp)
                        let inv = format!("1/(1+{b}*{vpk})", b = self.beta_factor);
                        let f = format!(
                            "(({base})+{linear}-({coeff})*({inv}))",
                            base = base,
                            coeff = coeff,
                        );
                        let h = format!(
                            "((1+{alpha_s}*({inv}))/{kg2})",
                            alpha_s = self.alpha_s,
                            kg2 = self.kg2,
                        );
                        (f, h)
                    }
                    ScreenForm::Exponential => {
                        // scale = exp(-(β·Vp)^{3/2})
                        let u = format!("max({b}*{vpk},0)", b = self.beta_factor);
                        let exf = format!("exp(-(({u})*sqrt({u})))");
                        let f = format!(
                            "(({base})+{linear}-({coeff})*({exf}))",
                            base = base,
                            coeff = coeff,
                        );
                        let h = format!(
                            "((1+{alpha_s}*({exf}))/{kg2})",
                            alpha_s = self.alpha_s,
                            kg2 = self.kg2,
                        );
                        (f, h)
                    }
                    ScreenForm::Classical => unreachable!(),
                };

                let ip = format!("({ip0})*{f}");
                let ig2 = format!("({ip0})*{h}");
                (ip, ig2)
            }
            ScreenForm::Classical => {
                // Classical Koren: softplus argument uses Vgk/Vg2k (not the Derk
                // sqrt(Kvb+Vg2²)), a plate arctan(Vpk/Kvb) knee, and a
                // Vp-independent screen current.
                let inner = format!("{kp}*(1/{mu}+V(g,k)/{vg2k})", kp = self.kp, mu = self.mu);
                let softplus = format!("ln(1+exp({inner}))");
                let e1 = format!("({vg2k}/{kp})*{sp}", kp = self.kp, sp = softplus);
                let g = format!("atan({vpk}/{kvb})", kvb = self.kvb);
                let ip = format!(
                    "2*pwr(uramp({e1}),{ex})/{kg1}*{g}",
                    ex = self.ex,
                    kg1 = self.kg1
                );
                // Ig2 = (Vg2k/mu + Vgk)^ex / Kg2, with uramp giving the x<=0 -> 0.
                let xg2 = format!("({vg2k}/{mu}+V(g,k))", mu = self.mu);
                let ig2 = format!("pwr(uramp({xg2}),{ex})/{kg2}", ex = self.ex, kg2 = self.kg2);
                (ip, ig2)
            }
        };

        // Leach grid current (identical across all screen forms).
        let grid = format!(
            "{ig}*pwr(uramp(V(g,k)/{vo}),1.5)",
            ig = self.ig_max,
            vo = self.vgk_onset,
        );

        // Junction parasitics: mirror `DkKernel::from_mna`, which auto-inserts
        // 10 pF caps ONLY when the circuit's C matrix is all zeros (a purely
        // resistive nonlinear circuit). Every pentode deck in the library
        // carries a real coupling cap, so this branch is never exercised by a
        // real target; the exact pentode junction-cap set melange inserts is not
        // yet mirrored here, so refuse rather than emit a silently-wrong twin.
        if add_parasitics {
            return Err(SpiceError::ParseError(format!(
                "pentode translation: model '{model_name}' is used in a purely resistive \
                 (no C/L) deck, where melange auto-inserts junction parasitics; the pentode \
                 parasitic twin is not yet implemented (no such deck exists in the library). \
                 Add a coupling/parasitic cap to the deck, or extend the translator."
            )));
        }

        Ok(format!(
            "* Reefman/Koren B-source twin of `.model {name} (VP|PENTODE)` — self-consistent with melange/tube.rs.\n\
             .subckt MELANGE_PENTODE_{name} p g k s\n\
             BP p k I={ip}\n\
             BG2 s k I={ig2}\n\
             BG g k I={grid}\n\
             .ends\n",
            name = model_name,
        ))
    }
}

/// Is `line` a pentode element card (`P<name> plate grid cath screen [supp] model`,
/// 6 or 7 tokens)? The caller must exclude the title line (line 0).
fn is_pentode_line(line: &str) -> bool {
    let t = line.trim();
    if t.is_empty() || t.starts_with('*') || t.starts_with('.') {
        return false;
    }
    let toks: Vec<&str> = t.split_whitespace().collect();
    (toks.len() == 6 || toks.len() == 7)
        && toks[0]
            .chars()
            .next()
            .is_some_and(|c| c.eq_ignore_ascii_case(&'P'))
}

/// Translate all pentode elements in `content` into B-source subcircuits.
/// Returns `content` unchanged when it contains no pentode element.
///
/// `content` is the deck to rewrite; `source` is the netlist to PARSE for model
/// parameters and the reactive-element check. These differ when the triode
/// translator has already run: `content` then carries injected ngspice `.subckt`
/// blocks and `X` calls that melange's own parser cannot re-parse, so we parse
/// the pristine original (`source`) for models/elements while rewriting the
/// chained `content`. Pentode `P` lines and `VP` `.model` cards are untouched by
/// the triode pass, so they are still present in `content` to rewrite/drop, and
/// the triode pass's appended subckts fall through this loop unchanged.
pub(crate) fn translate_pentodes_for_ngspice(
    content: &str,
    source: &str,
) -> Result<String, SpiceError> {
    // Fast path: skip the title line (line 0), scan the rest for a pentode card.
    if !content.lines().skip(1).any(is_pentode_line) {
        return Ok(content.to_string());
    }

    let netlist = Netlist::parse(source)
        .map_err(|e| SpiceError::ParseError(format!("pentode translation: {e}")))?;

    let mut pentode_models: HashMap<String, PentodeParams> = HashMap::new();
    for m in &netlist.models {
        if PENTODE_MODEL_TYPES
            .iter()
            .any(|t| m.model_type.eq_ignore_ascii_case(t))
        {
            pentode_models.insert(m.name.to_ascii_uppercase(), PentodeParams::from_model(m)?);
        }
    }

    // Mirror from_mna's parasitic-cap rule (see PentodeParams::subckt).
    let has_reactive = netlist
        .elements
        .iter()
        .any(|e| matches!(e, Element::Capacitor { .. } | Element::Inductor { .. }));
    let add_parasitics = !has_reactive;

    let mut out = String::with_capacity(content.len() + 512);
    let mut used: BTreeSet<String> = BTreeSet::new(); // deterministic subckt order

    for (i, line) in content.lines().enumerate() {
        // The title line is never an element or directive — pass it through.
        if i == 0 {
            out.push_str(line);
            out.push('\n');
            continue;
        }

        // Drop pentode `.model` cards — ngspice cannot parse type VP/PENTODE,
        // and the generated .subckt replaces them.
        if is_pentode_model_line(line, &pentode_models) {
            continue;
        }

        if is_pentode_line(line) {
            let toks: Vec<&str> = line.split_whitespace().collect();
            // Plate-first node order: name plate grid cath screen [supp] model.
            let name = toks[0];
            let (plate, grid, cath, screen) = (toks[1], toks[2], toks[3], toks[4]);
            let model = toks[toks.len() - 1]; // last token; skips a 5th (suppressor) node
            let model_uc = model.to_ascii_uppercase();
            if !pentode_models.contains_key(&model_uc) {
                return Err(SpiceError::ParseError(format!(
                    "pentode translation: pentode '{name}' references model '{model}', \
                     which is not a pentode (VP/PENTODE) .model"
                )));
            }
            used.insert(model_uc.clone());
            // P<name> -> X<name>: 'X' makes ngspice treat it as a subckt call.
            // Suppressor node (7-token form) is intentionally not bound —
            // melange's pentode model has no suppressor voltage.
            out.push_str(&format!(
                "X{name} {plate} {grid} {cath} {screen} MELANGE_PENTODE_{model_uc}\n"
            ));
            continue;
        }

        out.push_str(line);
        out.push('\n');
    }

    // Append one subckt per distinct pentode model actually used.
    for model_uc in &used {
        out.push_str(&pentode_models[model_uc].subckt(model_uc, add_parasitics)?);
    }

    Ok(out)
}

/// Is `line` a `.model <name> <VP|PENTODE>(...)` card for a known pentode model?
fn is_pentode_model_line(line: &str, pentode_models: &HashMap<String, PentodeParams>) -> bool {
    let t = line.trim();
    // `.get(..6)` (not `t[..6]`) — a byte slice panics when byte 6 falls inside
    // a multibyte char (e.g. an em-dash in a comment/header). A line whose first
    // 6 bytes span a multibyte char cannot be ".model" anyway.
    if !t.get(..6).is_some_and(|p| p.eq_ignore_ascii_case(".model")) {
        return false;
    }
    let toks: Vec<&str> = t.split_whitespace().collect();
    toks.get(1)
        .is_some_and(|name| pentode_models.contains_key(&name.to_ascii_uppercase()))
}

#[cfg(test)]
mod tests {
    use super::*;

    // EF86 is a Rational-form catalog pentode; bare VP() pulls all params from
    // the catalog (the model NAME is the whole spec).
    const RATIONAL_DECK: &str = "EF86 pentode stage\n\
        VIN in 0 DC 0\n\
        Cin in grid 100n\n\
        Rg grid 0 1Meg\n\
        P1 plate grid cathode screen EF86\n\
        Rk cathode 0 1k\n\
        Rp vcc plate 100k\n\
        Rs vcc screen 220k\n\
        Vcc vcc 0 DC 250\n\
        .model EF86 VP()\n\
        .end\n";

    #[test]
    fn rewrites_pentode_to_subckt_call() {
        let out = translate_pentodes_for_ngspice(RATIONAL_DECK, RATIONAL_DECK).unwrap();
        // P element replaced by an X call, plate-grid-cath-screen order preserved.
        assert!(out.contains("XP1 plate grid cathode screen MELANGE_PENTODE_EF86"));
        assert!(!out.lines().any(is_pentode_line));
        // Pentode .model card dropped.
        assert!(!out.lines().any(|l| {
            let u = l.trim().to_uppercase();
            u.starts_with(".MODEL") && u.contains("EF86")
        }));
        assert!(out.contains(".subckt MELANGE_PENTODE_EF86 p g k s"));
        assert!(out.contains("BP p k I="));
        assert!(out.contains("BG2 s k I="));
        assert!(out.contains("BG g k I="));
        assert!(out.contains(".ends"));
    }

    #[test]
    fn bare_vp_resolves_catalog_params() {
        // The subckt must contain a real Kg1 number from the catalog, proving
        // params were resolved (a bare VP() carries none itself).
        let out = translate_pentodes_for_ngspice(RATIONAL_DECK, RATIONAL_DECK).unwrap();
        let ef86 = lookup_pentode("EF86").expect("EF86 in catalog");
        // Kg1 appears as a divisor in the plate expression.
        assert!(out.contains(&format!("/{}", ef86.kg1)));
    }

    #[test]
    fn classical_uses_arctan_knee() {
        // KT88 is a Classical-Koren catalog entry (bare VP() → catalog params).
        let deck = "KT88 classical\n\
            VIN in 0 DC 0\n\
            Cin in grid 100n\n\
            Rg grid 0 1Meg\n\
            P1 plate grid cathode screen KT88\n\
            Rk cathode 0 130\n\
            Rp vcc plate 100k\n\
            Rs vcc screen 1k\n\
            Vcc vcc 0 DC 300\n\
            .model KT88 VP()\n\
            .end\n";
        let out = translate_pentodes_for_ngspice(deck, deck).unwrap();
        assert!(out.contains("MELANGE_PENTODE_KT88"));
        // Classical plate current carries the arctan(Vpk/Kvb) knee.
        assert!(out.contains("atan("));
    }

    #[test]
    fn unrecognized_model_errors() {
        let deck = "bad pentode\n\
            VIN in 0 DC 0\n\
            Cin in grid 100n\n\
            P1 plate grid cathode screen NOPENT\n\
            Rp vcc plate 100k\n\
            Vcc vcc 0 DC 250\n\
            .model NOPENT VP()\n\
            .end\n";
        let err = translate_pentodes_for_ngspice(deck, deck).unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("NOPENT") && msg.to_lowercase().contains("missing"));
    }

    #[test]
    fn variable_mu_errors_as_out_of_scope() {
        // SVAR>0 explicit override → var-mu → refused (named follow-up).
        let deck = "varimu\n\
            VIN in 0 DC 0\n\
            Cin in grid 100n\n\
            P1 plate grid cathode screen EF86\n\
            Rp vcc plate 100k\n\
            Vcc vcc 0 DC 250\n\
            .model EF86 VP(SVAR=0.083)\n\
            .end\n";
        let err = translate_pentodes_for_ngspice(deck, deck).unwrap_err();
        assert!(format!("{err}").to_lowercase().contains("variable-mu"));
    }

    #[test]
    fn passthrough_when_no_pentode() {
        let deck = "RC lowpass\nR1 in out 1k\nC1 out 0 100n\n.end\n";
        assert_eq!(translate_pentodes_for_ngspice(deck, deck).unwrap(), deck);
    }

    #[test]
    fn title_line_starting_with_p_is_not_a_pentode() {
        // A title beginning with "P…" with 6 tokens must not be rewritten.
        let deck = "Pentode Amp Test Deck Rev A\nR1 in out 1k\nC1 out 0 100n\n.end\n";
        assert_eq!(translate_pentodes_for_ngspice(deck, deck).unwrap(), deck);
    }
}
