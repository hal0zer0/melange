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

use melange_solver::parser::{Element, Model, Netlist};

use crate::spice_runner::SpiceError;

/// Melange triode `.model` types (any of these prefix a Koren triode).
const TRIODE_MODEL_TYPES: [&str; 3] = ["TRIODE", "VT", "TUBE"];

/// Default grid-current parameters, mirroring
/// `melange_devices::tube` (`DEFAULT_IG_MAX` / `DEFAULT_VGK_ONSET`). These are
/// not `.model`-settable, so netlist triodes always use these values.
const DEFAULT_IG_MAX: f64 = 2e-3;
const DEFAULT_VGK_ONSET: f64 = 0.5;

/// Resolved Koren triode parameters for one `.model`.
struct TriodeParams {
    mu: f64,
    ex: f64,
    kg1: f64,
    kp: f64,
    kvb: f64,
    lambda: f64,
    ig_max: f64,
    vgk_onset: f64,
}

impl TriodeParams {
    fn from_model(m: &Model) -> Result<Self, SpiceError> {
        let get = |key: &str| {
            m.params
                .iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(key))
                .map(|(_, v)| *v)
        };
        let require = |key: &str| {
            get(key).ok_or_else(|| {
                SpiceError::ParseError(format!(
                    "tube translation: triode model '{}' is missing required Koren \
                     parameter {} — specify MU, EX, KG1, KP and KVB on the .model card",
                    m.name, key
                ))
            })
        };
        Ok(Self {
            mu: require("MU")?,
            ex: require("EX")?,
            kg1: require("KG1")?,
            kp: require("KP")?,
            kvb: require("KVB")?,
            // LAMBDA (Early effect) defaults to 0 in melange; grid params are not
            // .model-settable and always take the tube.rs defaults.
            lambda: get("LAMBDA").unwrap_or(0.0),
            ig_max: DEFAULT_IG_MAX,
            vgk_onset: DEFAULT_VGK_ONSET,
        })
    }

    /// Emit the `.subckt` for this triode model. Port order `g p k` mirrors the
    /// `T` element's node order (grid-plate-cathode), so the `X` call binds
    /// `n_grid n_plate n_cathode` positionally.
    ///
    /// `add_parasitics` mirrors `DkKernel::from_mna`, which auto-inserts 10 pF
    /// junction caps ONLY when the circuit's C matrix is all zeros (a purely
    /// resistive nonlinear circuit that needs regularization). For any circuit
    /// carrying a real cap or inductor, melange inserts nothing, so the twin
    /// must not either — an unconditional 10 pF was a spurious mismatch,
    /// negligible on low-Z resistive plate loads (triode_cc: 0.10% → 0.0035%
    /// once removed) but material on high-Z cap-loaded nodes.
    fn subckt(&self, model_name: &str, add_parasitics: bool) -> String {
        // Vpk floored at 1e-3 (mirrors tube.rs `plate_current`).
        let vpk = "max(V(p,k),1e-3)";
        // inner = KP*(1/MU + Vgk/sqrt(KVB + Vpk^2))
        let inner = format!(
            "{kp}*(1/{mu}+V(g,k)/sqrt({kvb}+{vpk}*{vpk}))",
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
        // Grid current: melange's Leach-style Ig = ig_max*max(0,Vgk/vgk_onset)^1.5.
        let grid = format!(
            "{ig}*pwr(uramp(V(g,k)/{vo}),1.5)",
            ig = self.ig_max,
            vo = self.vgk_onset,
        );
        // Only mirror from_mna's 10 pF junction parasitics for purely resistive
        // nonlinear circuits (see `add_parasitics` doc).
        let parasitics = if add_parasitics {
            "Cgk g k 10p\nCpk p k 10p\n"
        } else {
            ""
        };
        format!(
            "* Koren B-source twin of `.model {name} (TRIODE|VT|TUBE)` — self-consistent with melange/tube.rs.\n\
             .subckt MELANGE_TRIODE_{name} g p k\n\
             BP p k I={plate}\n\
             BG g k I={grid}\n\
             {parasitics}.ends\n",
            name = model_name,
        )
    }
}

/// Is `line` a triode element card (`T<name> g p k model`, exactly 5 tokens)?
/// The caller must exclude the title line (line 0) — a 5-word title beginning
/// with "T…" would otherwise look like a triode.
fn is_triode_line(line: &str) -> bool {
    let t = line.trim();
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

    // Parse to resolve model parameters (defaults applied by TriodeParams).
    let netlist = Netlist::parse(content)
        .map_err(|e| SpiceError::ParseError(format!("tube translation: {e}")))?;

    let mut tube_models: HashMap<String, TriodeParams> = HashMap::new();
    for m in &netlist.models {
        if TRIODE_MODEL_TYPES
            .iter()
            .any(|t| m.model_type.eq_ignore_ascii_case(t))
        {
            tube_models.insert(m.name.to_ascii_uppercase(), TriodeParams::from_model(m)?);
        }
    }

    // Mirror from_mna's parasitic-cap rule: melange auto-inserts 10 pF junction
    // caps ONLY when the C matrix is all zeros (no reactive element). A cap or
    // inductor anywhere in the deck means melange inserts none, so the twin
    // must not either.
    let has_reactive = netlist
        .elements
        .iter()
        .any(|e| matches!(e, Element::Capacitor { .. } | Element::Inductor { .. }));
    let add_parasitics = !has_reactive;

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
        if is_tube_model_line(line, &tube_models) {
            continue;
        }

        if is_triode_line(line) {
            let toks: Vec<&str> = line.split_whitespace().collect();
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
        out.push_str(&tube_models[model_uc].subckt(model_uc, add_parasitics));
    }

    Ok(out)
}

/// Is `line` a `.model <name> <TRIODE|VT|TUBE>(...)` card for a known triode
/// model? Matched by name against the resolved model set so we only drop cards
/// the translator is replacing.
fn is_tube_model_line(line: &str, tube_models: &HashMap<String, TriodeParams>) -> bool {
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
        .is_some_and(|name| tube_models.contains_key(&name.to_ascii_uppercase()))
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
        assert!(out.contains("BG g k I="));
        // DECK has a real cap (Cin) → melange inserts no parasitics, so the twin
        // must not either (matches from_mna's C-all-zeros rule).
        assert!(!out.contains("Cgk g k 10p"));
        assert!(!out.contains("Cpk p k 10p"));
        assert!(out.contains(".ends"));
        // Baked Koren params appear in the plate expression.
        assert!(out.contains("/1060")); // KG1
        assert!(out.contains("1/100")); // 1/MU
    }

    #[test]
    fn resistive_only_tube_deck_gets_parasitics() {
        // No cap/inductor → melange's C matrix is all zeros → from_mna auto-inserts
        // 10 pF junction caps → the twin must mirror them.
        let deck = "resistive tube\n\
            VIN in 0 DC 0\n\
            Rg in grid 1Meg\n\
            T1 grid plate cathode 12AX7\n\
            Rk cathode 0 1.5k\n\
            Rp vcc plate 100k\n\
            Vcc vcc 0 DC 250\n\
            .model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)\n\
            .end\n";
        let out = translate_tubes_for_ngspice(deck).unwrap();
        assert!(out.contains("Cgk g k 10p"));
        assert!(out.contains("Cpk p k 10p"));
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

    #[test]
    fn missing_koren_param_errors_clearly() {
        let deck = "bad\nT1 g p k M\n.model M TRIODE(MU=100 EX=1.4 KG1=1060)\n.end\n";
        let err = translate_tubes_for_ngspice(deck).unwrap_err();
        assert!(format!("{err}").contains("KP"));
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
