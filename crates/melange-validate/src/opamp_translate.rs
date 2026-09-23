//! Translate melange op-amp (`U`) elements into their ngspice **reference**
//! equivalent: the same linear VCCS macromodel melange itself stamps.
//!
//! ngspice has no op-amp element and no `OA` model type — it reads a `U` card
//! as a uniform distributed-RC line, then fails on the model lookup and blames
//! the author's (correct) `.model` card. Until now `validate` refused every
//! deck containing an op-amp for that reason, which took the single most common
//! hobbyist circuit class (a pedal with an op-amp gain stage) out of reach of
//! the one command that answers "did I write this netlist right?".
//!
//! It was never a real limitation, only a missing translator. melange's op-amp
//! is a **linear** Boyle-style VCCS (`crates/melange-solver/src/mna.rs`, "Stamp
//! op-amps as VCCS into G matrix"), and every stamp it makes has a one-to-one
//! ngspice primitive:
//!
//! | melange stamp (`mna.rs`) | ngspice card emitted here |
//! |---|---|
//! | `G[out][n_plus] -= Gm`, `G[out][n_minus] += Gm`, `Gm = AOL/ROUT` | `GOA_<n> out 0 <n_minus> <n_plus> {Gm}` |
//! | `G[out][out] += Go`, `Go = 1/ROUT` | `ROA_<n> out 0 {ROUT}` |
//! | `G[n_plus][n_plus] += 1/RIN` (only when `RIN` finite, `n_plus != 0`) | `RINP_<n> <n_plus> 0 {RIN}` |
//! | `G[n_minus][n_minus] += 1/RIN` (ditto) | `RINM_<n> <n_minus> 0 {RIN}` |
//! | `CurrentSourceInfo { n_plus, 0, IB }` (only when `IB != 0`) | `IBP_<n> 0 <n_plus> {IB}` |
//!
//! ### Why the control nodes are swapped on the `G` card
//!
//! melange's convention is `G[k][j]·v_j` = current **leaving** node `k`, and it
//! stamps `-Gm` at `[out][n_plus]` / `+Gm` at `[out][n_minus]` — i.e. the
//! op-amp *injects* `Gm·(V+ − V−)` into the output node. ngspice's `G` element
//! is the opposite sense: positive current flows *from* `n+`, through the
//! source, *to* `n-`, so `GOA out 0 nc+ nc-` **draws** `gm·(V(nc+) − V(nc−))`
//! out of `out`. Swapping the control pair (`nc+ = n_minus`, `nc- = n_plus`)
//! makes the two agree exactly, and is what the shipped hand-expanded deck
//! `tests/data/opamp_inverting/circuit.cir` already does (`G1 out 0 inv 0
//! 200000` with `V+` = ground).
//!
//! ### What is deliberately NOT emitted
//!
//! * **`GBW`** — melange computes `iir_c_dom` from it in `mna.rs` but **no
//!   codegen path consumes it** (grep: `iir_c_dom` appears only in `ir/mod.rs`
//!   test fixtures). Its only live effect is triggering the ±13 V default
//!   rails. Emitting a dominant pole here would give ngspice a filter melange
//!   does not have.
//! * **`VCC` / `VEE` / `VSAT` / `SR`** — melange's rail clamp and slew clamp are
//!   post-NR operations on the solved node voltage, in one of five modes
//!   (`docs/aidocs/OPAMP_RAIL_MODES.md`). No ngspice primitive reproduces
//!   `ActiveSetBe`'s pin-and-BE-resolve, and a clamp that does not match
//!   melange's *mode* exactly is worse than none. These are handled by
//!   refusing the run instead — see [`rail_probes`].
//! * **`VOH_DROP` / `VOL_DROP`** — `BoyleDiodes`-only, and that mode is opt-in.
//! * **`EN` / `IN`** — noise sources, off in the validate codegen config.
//!
//! ### Scope limit, enforced
//!
//! The emitted twin is linear. A run whose op-amp output actually reaches a
//! rail, or actually hits the slew limit, is comparing a clamped melange
//! against an unclamped ngspice — the exact "two different circuits" class
//! [`crate::deck_guard`] exists to prevent. [`rail_probes`] +
//! [`check_rail_probes`] detect that on the reference trace and refuse, so a
//! correlation is never printed across a clamp event. See [`check_rail_probes`]
//! for why observing the *reference* is sufficient.

use std::collections::{BTreeSet, HashMap};

use melange_solver::parser::{Element, Model, Netlist};

use crate::spice_runner::SpiceError;

/// The only `.model` type melange accepts on a `U` element.
const OPAMP_MODEL_TYPE: &str = "OA";

// Defaults, mirroring the `OpampInfo` literal in `mna.rs` (`categorize_element`,
// `Element::Opamp` arm). A `U` element whose `.model` card is absent or omits a
// key gets these on melange's side, so the twin must use them too.
const DEFAULT_AOL: f64 = 200_000.0;
const DEFAULT_ROUT: f64 = 1.0;

/// Rail melange auto-defaults to when `GBW` is specified but no explicit
/// VCC/VEE/VSAT is (`mna.rs`, "Resolve op-amp output voltage clamps"). Applied
/// per side, so `OA(VCC=9 GBW=3MEG)` really does resolve to a 9 V / −13 V
/// window — mirrored here rather than "fixed", because the point is to match
/// melange.
const GBW_DEFAULT_RAIL: f64 = 13.0;

/// Op-amp parameters resolved exactly as `mna.rs` resolves them.
#[derive(Debug, Clone)]
pub(crate) struct OpampParams {
    aol: f64,
    r_out: f64,
    rin: f64,
    ib: f64,
    /// Resolved upper clamp (VCC > +VSAT > GBW default > +inf).
    vcc: f64,
    /// Resolved lower clamp (VEE > −VSAT > −GBW default > −inf).
    vee: f64,
    /// Slew rate in V/s (`.model` gives V/µs; melange multiplies by 1e6).
    sr: f64,
    /// `AOL_TRANSIENT_CAP` as given. Finite means melange's transient G matrix
    /// uses a different Gm than the DC stamp, which this twin cannot express.
    aol_transient_cap: f64,
}

impl Default for OpampParams {
    fn default() -> Self {
        Self {
            aol: DEFAULT_AOL,
            r_out: DEFAULT_ROUT,
            rin: f64::INFINITY,
            ib: 0.0,
            vcc: f64::INFINITY,
            vee: f64::NEG_INFINITY,
            sr: f64::INFINITY,
            aol_transient_cap: f64::INFINITY,
        }
    }
}

impl OpampParams {
    /// Resolve one `.model … OA(…)` card. Unknown keys are ignored here (the
    /// parser and `mna.rs` already report them); only the keys that change the
    /// emitted twin or the rail window are read.
    fn from_model(m: &Model) -> Result<Self, SpiceError> {
        if !m.model_type.eq_ignore_ascii_case(OPAMP_MODEL_TYPE) {
            return Err(SpiceError::ParseError(format!(
                "op-amp translation: model '{}' has type '{}', expected '{}'",
                m.name, m.model_type, OPAMP_MODEL_TYPE
            )));
        }
        let mut p = Self::default();
        let mut vsat = f64::INFINITY;
        let mut gbw = f64::INFINITY;
        for (key, val) in &m.params {
            match key.to_ascii_uppercase().as_str() {
                "AOL" => p.aol = *val,
                "ROUT" => p.r_out = *val,
                "RIN" => p.rin = *val,
                "IB" => p.ib = *val,
                "VCC" => p.vcc = *val,
                "VEE" => p.vee = *val,
                "VSAT" => vsat = *val,
                "GBW" => gbw = *val,
                // SPICE convention: V/µs on the card, V/s internally.
                "SR" => p.sr = *val * 1.0e6,
                "AOL_TRANSIENT_CAP" => p.aol_transient_cap = *val,
                _ => {}
            }
        }
        // Rail resolution, in melange's priority order (`mna.rs`).
        if !p.vcc.is_finite() {
            if vsat.is_finite() {
                p.vcc = vsat;
            } else if gbw.is_finite() {
                p.vcc = GBW_DEFAULT_RAIL;
            }
        }
        if !p.vee.is_finite() {
            if vsat.is_finite() {
                p.vee = -vsat;
            } else if gbw.is_finite() {
                p.vee = -GBW_DEFAULT_RAIL;
            }
        }
        if p.r_out == 0.0 {
            return Err(SpiceError::ParseError(format!(
                "op-amp translation: model '{}' has ROUT=0, which gives an \
                 infinite transconductance (Gm = AOL/ROUT)",
                m.name
            )));
        }
        Ok(p)
    }

    /// True when melange applies a post-NR clamp this linear twin cannot.
    fn has_clamp(&self) -> bool {
        self.vcc.is_finite() || self.vee.is_finite() || self.sr.is_finite()
    }
}

/// Is `line` an op-amp element card (`U<name> n+ n- out MODEL`, exactly 5
/// tokens)? The caller must skip the title line (line 0) — a 5-word title
/// starting with "U…" would otherwise read as an op-amp, the same trap
/// `tube_translate` guards.
fn is_opamp_line(line: &str) -> bool {
    let t = line.trim();
    if t.is_empty() || t.starts_with('*') || t.starts_with('.') {
        return false;
    }
    let toks: Vec<&str> = t.split_whitespace().collect();
    toks.len() == 5
        && toks[0]
            .chars()
            .next()
            .is_some_and(|c| c.eq_ignore_ascii_case(&'U'))
}

/// Is `line` a `.model <name> OA(...)` card for one of the models being
/// replaced? Matched by name against the resolved set so only cards this
/// translator supersedes are dropped.
fn is_opamp_model_line(line: &str, models: &HashMap<String, OpampParams>) -> bool {
    let t = line.trim();
    // `.get(..6)`, not `t[..6]`: a byte slice panics when byte 6 lands inside a
    // multibyte char (em-dashes in header comments are common in real decks).
    if !t.get(..6).is_some_and(|p| p.eq_ignore_ascii_case(".model")) {
        return false;
    }
    let toks: Vec<&str> = t.split_whitespace().collect();
    toks.get(1)
        .is_some_and(|name| models.contains_key(&name.to_ascii_uppercase()))
}

/// Collect every `OA` model in the deck, keyed by upper-cased name.
fn collect_opamp_models(netlist: &Netlist) -> Result<HashMap<String, OpampParams>, SpiceError> {
    let mut out = HashMap::new();
    for m in &netlist.models {
        if m.model_type.eq_ignore_ascii_case(OPAMP_MODEL_TYPE) {
            out.insert(m.name.to_ascii_uppercase(), OpampParams::from_model(m)?);
        }
    }
    Ok(out)
}

/// SPICE-safe number: enough digits that the twin's Gm is bit-for-bit the one
/// melange stamps, in a form ngspice's own value parser reads back exactly
/// (plain exponential, no SI suffix).
fn num(v: f64) -> String {
    format!("{:.17e}", v)
}

/// Emit the ngspice cards for one op-amp instance.
///
/// The `n_plus != "0"` / `n_minus != "0"` guards mirror `mna.rs`, which skips
/// the RIN and IB stamps on a grounded input pin (`if np > 0`). Emitting them
/// anyway would put a resistor and a current source across ground.
fn emit_instance(name: &str, n_plus: &str, n_minus: &str, n_out: &str, p: &OpampParams) -> String {
    let gm = p.aol / p.r_out;
    let mut s = format!(
        "* melange op-amp {name}: linear VCCS twin of mna.rs (Gm = AOL/ROUT, Go = 1/ROUT)\n\
         GOA_{name} {n_out} 0 {n_minus} {n_plus} {gm}\n\
         ROA_{name} {n_out} 0 {rout}\n",
        gm = num(gm),
        rout = num(p.r_out),
    );
    if p.rin.is_finite() && p.rin > 0.0 {
        if !is_ground(n_plus) {
            s.push_str(&format!("RINP_{name} {n_plus} 0 {}\n", num(p.rin)));
        }
        if !is_ground(n_minus) {
            s.push_str(&format!("RINM_{name} {n_minus} 0 {}\n", num(p.rin)));
        }
    }
    if p.ib != 0.0 {
        // melange injects +IB INTO each input node (`inject_rhs_current`, whose
        // RHS is injected current). ngspice's I element flows from n+ through
        // the source to n-, so `I 0 <pin>` injects into the pin.
        if !is_ground(n_plus) {
            s.push_str(&format!("IBP_{name} 0 {n_plus} {}\n", num(p.ib)));
        }
        if !is_ground(n_minus) {
            s.push_str(&format!("IBM_{name} 0 {n_minus} {}\n", num(p.ib)));
        }
    }
    s
}

/// Ground, as melange's parser resolves it: `0`, and the `gnd`/`ground`
/// aliases it folds to node 0 (`parser::normalize_node_name`). ngspice folds
/// the same three, so a `RIN` shunt or `IB` source emitted on one of them would
/// be a component with both ends on ground — which is exactly why `mna.rs`
/// skips those stamps (`if np > 0`).
fn is_ground(node: &str) -> bool {
    let n = node.trim();
    n == "0" || n.eq_ignore_ascii_case("gnd") || n.eq_ignore_ascii_case("ground")
}

/// Translate every op-amp element in `content` into its ngspice VCCS twin.
/// Returns `content` unchanged when the deck contains no `U` card — including
/// a deck that hand-expanded its op-amps already (e.g. the shipped
/// `opamp_inverting` deck), which has nothing for this pass to double-translate.
///
/// `source` is the **pristine** deck, parsed for the `.model` parameters, while
/// `content` is the string being rewritten. The two differ because this pass
/// runs after the triode/pentode passes, whose emitted ngspice `.subckt` / `X`
/// bodies melange's own parser cannot read back — same split, and same reason,
/// as `pentode_translate::translate_pentodes_for_ngspice`. A deck carrying both
/// a tube and an op-amp is not exotic (any tube preamp with an op-amp
/// buffer/effects loop), so this matters.
pub(crate) fn translate_opamps_for_ngspice(
    content: &str,
    source: &str,
) -> Result<String, SpiceError> {
    if !content.lines().skip(1).any(is_opamp_line) {
        return Ok(content.to_string());
    }

    let netlist = Netlist::parse(source)
        .map_err(|e| SpiceError::ParseError(format!("op-amp translation: {e}")))?;
    let models = collect_opamp_models(&netlist)?;

    let mut out = String::with_capacity(content.len() + 256);
    // Deterministic order, and one report per model rather than per instance.
    let mut missing: BTreeSet<String> = BTreeSet::new();

    for (i, line) in content.lines().enumerate() {
        if i == 0 {
            out.push_str(line);
            out.push('\n');
            continue;
        }
        if is_opamp_model_line(line, &models) {
            // ngspice cannot parse type OA; the emitted cards replace it.
            continue;
        }
        if is_opamp_line(line) {
            let toks: Vec<&str> = line.split_whitespace().collect();
            let (name, np, nm, no, model) = (toks[0], toks[1], toks[2], toks[3], toks[4]);
            let model_uc = model.to_ascii_uppercase();
            let params = match models.get(&model_uc) {
                Some(p) => p.clone(),
                None => {
                    // mna.rs silently falls back to the OpampInfo defaults when
                    // no matching card exists, so the twin does too — but say so,
                    // because a deck that meant to set ROUT=75 and misspelled the
                    // model name is validating a 1 Ω part.
                    missing.insert(model.to_string());
                    OpampParams::default()
                }
            };
            out.push_str(&emit_instance(name, np, nm, no, &params));
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }

    for m in &missing {
        log::warn!(
            "op-amp translation: no `.model {m} OA(...)` card in the deck — \
             melange and the ngspice twin both fall back to AOL={DEFAULT_AOL}, \
             ROUT={DEFAULT_ROUT}"
        );
    }

    Ok(out)
}

/// One op-amp output node the reference run has to be watched on, because
/// melange applies a post-NR clamp there that the linear twin does not.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RailProbe {
    /// Element name, for the refusal message.
    pub name: String,
    /// Output node name, as written in the deck.
    pub node: String,
    pub vcc: f64,
    pub vee: f64,
    /// Slew rate in V/s (`INFINITY` = no slew clamp).
    pub sr: f64,
}

/// Every op-amp output node that carries a clamp melange applies and the
/// reference does not.
///
/// Returns an empty vec for a deck with no op-amp, for op-amps whose `.model`
/// sets no `VCC`/`VEE`/`VSAT`/`GBW`/`SR` (melange's `OpampRailMode::None` —
/// a pure linear VCCS, which the twin reproduces exactly), and for a deck
/// melange's own parser cannot read (that fails loudly on its own).
pub(crate) fn rail_probes(content: &str) -> Vec<RailProbe> {
    let Ok(netlist) = Netlist::parse(content) else {
        return Vec::new();
    };
    let Ok(models) = collect_opamp_models(&netlist) else {
        return Vec::new();
    };
    let mut probes = Vec::new();
    for elem in &netlist.elements {
        if let Element::Opamp {
            name, n_out, model, ..
        } = elem
        {
            let p = models
                .get(&model.to_ascii_uppercase())
                .cloned()
                .unwrap_or_default();
            if p.has_clamp() {
                probes.push(RailProbe {
                    name: name.clone(),
                    node: n_out.clone(),
                    vcc: p.vcc,
                    vee: p.vee,
                    sr: p.sr,
                });
            }
        }
    }
    probes
}

/// Op-amps whose transient Gm melange changes out from under the DC stamp via
/// an explicit `AOL_TRANSIENT_CAP`, which one ngspice `G` card cannot express.
///
/// Returns `(element name, model name, cap)` per affected instance. The
/// auto-detected Rule D' cap (`opamp_is_sidechain_rectifier`) is deliberately
/// not reproduced here: it fires only on precision-rectifier topologies, which
/// are *designed* to sit on a rail and are therefore already refused by
/// [`check_rail_probes`].
pub(crate) fn transient_aol_cap_opamps(content: &str) -> Vec<(String, String, f64)> {
    let Ok(netlist) = Netlist::parse(content) else {
        return Vec::new();
    };
    let Ok(models) = collect_opamp_models(&netlist) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for elem in &netlist.elements {
        if let Element::Opamp { name, model, .. } = elem {
            if let Some(p) = models.get(&model.to_ascii_uppercase()) {
                if p.aol_transient_cap.is_finite() {
                    out.push((name.clone(), model.clone(), p.aol_transient_cap));
                }
            }
        }
    }
    out
}

/// Flag a rail as engaged this close to it. An op-amp output within a
/// millivolt of its supply is at the supply for every practical purpose, and
/// the margin absorbs the two engines' own truncation error so a clamp event
/// is not missed because ngspice landed a microvolt short of it.
const RAIL_MARGIN_V: f64 = 1.0e-3;

/// Refuse the comparison when the reference trace shows the melange-side clamp
/// would have engaged.
///
/// **Why watching the reference is sufficient.** Both engines integrate the
/// same circuit; melange's rail and slew clamps are post-NR operations that do
/// nothing until the solved output crosses the threshold. So the two
/// trajectories are identical up to melange's *first* clamp event, and at that
/// sample the reference — being the same trajectory — is also at or past the
/// threshold. Detecting on the reference therefore catches every clamp event at
/// its first occurrence. It errs toward refusing (the unclamped reference keeps
/// running past a rail that melange pins), which is the safe direction: the
/// cost of a false refusal is one edit, the cost of a missed one is a
/// correlation between two different circuits.
///
/// `dt` must be the reference's own uniform timestep — the same `1/sample_rate`
/// melange runs at — because melange's slew clamp is a *per-sample* delta
/// limit (`|Δv| ≤ SR·dt`), not a continuous-time derivative.
pub(crate) fn check_rail_probes(
    probes: &[RailProbe],
    voltages: &HashMap<String, Vec<f64>>,
    dt: f64,
) -> Result<(), SpiceError> {
    for probe in probes {
        let key = probe.node.trim().to_lowercase();
        let Some(trace) = voltages.get(&key) else {
            // The probe node was requested but ngspice printed no column for
            // it. Not silently ignorable: it means the guard did not run.
            return Err(SpiceError::DeckNotComparable(format!(
                "op-amp {} declares a supply rail or slew limit, so validate has to watch \
                 its output node '{}' to know whether melange's clamp engaged — but ngspice \
                 printed no trace for that node. Refusing rather than reporting a correlation \
                 the rail guard never checked.",
                probe.name, probe.node
            )));
        };

        if probe.vcc.is_finite() {
            if let Some((idx, v)) = trace
                .iter()
                .enumerate()
                .find(|(_, v)| **v >= probe.vcc - RAIL_MARGIN_V)
                .map(|(i, v)| (i, *v))
            {
                return Err(rail_refusal(probe, "VCC", probe.vcc, v, idx, dt));
            }
        }
        if probe.vee.is_finite() {
            if let Some((idx, v)) = trace
                .iter()
                .enumerate()
                .find(|(_, v)| **v <= probe.vee + RAIL_MARGIN_V)
                .map(|(i, v)| (i, *v))
            {
                return Err(rail_refusal(probe, "VEE", probe.vee, v, idx, dt));
            }
        }
        if probe.sr.is_finite() && dt > 0.0 {
            let max_dv = probe.sr * dt;
            for (idx, pair) in trace.windows(2).enumerate() {
                let delta = pair[1] - pair[0];
                if delta.abs() > max_dv {
                    return Err(SpiceError::DeckNotComparable(format!(
                        "op-amp {} hit its slew limit during the reference run: at t = {:.6} s the \
                         output node '{}' moved {:.4} V in one sample, and `.model … OA(SR=…)` caps \
                         melange at {:.4} V per sample ({:.4} V/µs x {:.3e} s).\n\n\
                         melange applies that slew clamp after each NR solve; the ngspice reference \
                         has no equivalent, so past this sample the two engines are running \
                         different circuits and a correlation between them means nothing. \
                         Refusing instead of printing one.\n\n\
                         To validate this circuit: drop `SR=` from the `.model` card for the \
                         validation run (that measures the solver, which is what validate is for), \
                         or lower the drive level until the output stays inside the slew limit.",
                        probe.name,
                        (idx + 1) as f64 * dt,
                        probe.node,
                        delta,
                        max_dv,
                        probe.sr / 1.0e6,
                        dt,
                    )));
                }
            }
        }
    }
    Ok(())
}

fn rail_refusal(
    probe: &RailProbe,
    rail_name: &str,
    rail: f64,
    observed: f64,
    idx: usize,
    dt: f64,
) -> SpiceError {
    SpiceError::DeckNotComparable(format!(
        "op-amp {} reached its {} rail during the reference run: at t = {:.6} s the output node \
         '{}' is at {:.4} V against {} = {:.4} V.\n\n\
         melange clamps that node to the rail (one of the five modes in \
         docs/aidocs/OPAMP_RAIL_MODES.md, chosen from the topology); the ngspice reference is the \
         linear VCCS macromodel and keeps going. From this sample on the two engines are running \
         different circuits, so a correlation between them measures the clamp, not the solver. \
         Refusing instead of printing one.\n\n\
         To validate this circuit, pick one:\n\
         * drop `VCC`/`VEE`/`VSAT` (and `GBW`, which auto-defaults the rails to +/-13 V) from the \
         `.model … OA(...)` card for the validation run — that validates the solver on the same \
         circuit both engines can express;\n\
         * lower the drive level until the op-amp stays inside its rails, then validate;\n\
         * keep the rails and validate the stages around the op-amp separately.",
        probe.name,
        rail_name,
        idx as f64 * dt,
        probe.node,
        observed,
        rail_name,
        rail,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DECK: &str = "Inverting amp\n\
        R1 in inv 10k\n\
        R2 inv out 100k\n\
        U1 0 inv out OA1\n\
        Cstab out 0 1p\n\
        .model OA1 OA(AOL=200000 ROUT=1)\n\
        .end\n";

    #[test]
    fn rewrites_opamp_to_vccs_and_rout() {
        let out = translate_opamps_for_ngspice(DECK, DECK).unwrap();
        // No U card survives.
        assert!(!out.lines().skip(1).any(is_opamp_line), "{out}");
        // The OA model card is dropped (ngspice cannot parse type OA).
        assert!(!out.to_uppercase().contains("OA(AOL"), "{out}");
        // Control nodes swapped: nc+ = n_minus, nc- = n_plus.
        assert!(out.contains("GOA_U1 out 0 inv 0 "), "{out}");
        assert!(out.contains("ROA_U1 out 0 "), "{out}");
        // Gm = AOL/ROUT = 2e5, Go = 1/ROUT = 1 ohm.
        assert!(out.contains("2.00000000000000000e5"), "{out}");
        // Untouched lines survive verbatim.
        assert!(out.contains("R2 inv out 100k"));
        assert!(out.contains("Cstab out 0 1p"));
    }

    /// The emitted twin must reproduce melange's own numbers, so the shipped
    /// hand-expanded deck and a native-`U` deck describe one circuit.
    #[test]
    fn emitted_gm_matches_the_hand_expanded_shipped_deck() {
        let out = translate_opamps_for_ngspice(DECK, DECK).unwrap();
        let g_line = out
            .lines()
            .find(|l| l.starts_with("GOA_U1"))
            .expect("G card");
        let gm: f64 = g_line.split_whitespace().last().unwrap().parse().unwrap();
        // tests/data/opamp_inverting/circuit.cir: `G1 out 0 inv 0 200000`.
        assert_eq!(gm, 200_000.0);
        let r_line = out
            .lines()
            .find(|l| l.starts_with("ROA_U1"))
            .expect("R card");
        let rout: f64 = r_line.split_whitespace().last().unwrap().parse().unwrap();
        // `Rout out 0 1`.
        assert_eq!(rout, 1.0);
    }

    #[test]
    fn deck_without_opamps_is_returned_unchanged() {
        let deck = "RC\nR1 in out 1k\nC1 out 0 10n\n.end\n";
        assert_eq!(translate_opamps_for_ngspice(deck, deck).unwrap(), deck);
    }

    /// The shipped `opamp_inverting` deck hand-expands its op-amp already. It
    /// carries no `U` card, so this pass must be a strict no-op on it — no
    /// double translation, no conflicting second VCCS.
    #[test]
    fn hand_expanded_deck_is_not_double_translated() {
        let deck = "\
Op-Amp Inverting Amplifier
R1 in inv 10k
R2 inv out 100k
G1 out 0 inv 0 200000
Rout out 0 1
Cstab out 0 1p
.END
";
        assert_eq!(translate_opamps_for_ngspice(deck, deck).unwrap(), deck);
        assert!(rail_probes(deck).is_empty());
    }

    /// A 5-token title starting with "U" must not be read as an op-amp.
    #[test]
    fn title_line_is_never_an_opamp() {
        let deck = "Unity gain buffer test deck\nR1 a b 1k\n.end\n";
        assert_eq!(translate_opamps_for_ngspice(deck, deck).unwrap(), deck);
    }

    #[test]
    fn rin_and_ib_are_emitted_only_on_ungrounded_pins() {
        let deck = "amp\n\
            R1 in inv 10k\n\
            U1 0 inv out OA1\n\
            C1 out 0 1p\n\
            .model OA1 OA(AOL=1e5 ROUT=75 RIN=1e6 IB=45n)\n\
            .end\n";
        let out = translate_opamps_for_ngspice(deck, deck).unwrap();
        // n_plus is ground: mna.rs skips both stamps there, so must this.
        assert!(!out.contains("RINP_U1"), "{out}");
        assert!(!out.contains("IBP_U1"), "{out}");
        assert!(out.contains("RINM_U1 inv 0 "), "{out}");
        // IB is injected INTO the pin, so the ngspice source runs 0 -> pin.
        assert!(out.contains("IBM_U1 0 inv "), "{out}");
    }

    #[test]
    fn rails_resolve_in_melanges_priority_order() {
        let mk = |card: &str| {
            let deck = format!("amp\nR1 in inv 10k\nU1 0 inv out OA1\nC1 out 0 1p\n{card}\n.end\n");
            rail_probes(&deck)
        };
        // No rail keys at all -> no probe (OpampRailMode::None, exact twin).
        assert!(mk(".model OA1 OA(AOL=1e5 ROUT=75)").is_empty());
        // VSAT resolves symmetric.
        let p = mk(".model OA1 OA(AOL=1e5 ROUT=75 VSAT=13)");
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].vcc, 13.0);
        assert_eq!(p[0].vee, -13.0);
        // VCC/VEE win over VSAT.
        let p = mk(".model OA1 OA(AOL=1e5 ROUT=75 VSAT=13 VCC=9 VEE=-9)");
        assert_eq!((p[0].vcc, p[0].vee), (9.0, -9.0));
        // GBW alone triggers the +/-13 V auto-default.
        let p = mk(".model OA1 OA(AOL=1e5 ROUT=75 GBW=3e6)");
        assert_eq!((p[0].vcc, p[0].vee), (13.0, -13.0));
        // GBW's default applies PER SIDE: a single-supply card gets 9 / -13.
        let p = mk(".model OA1 OA(AOL=1e5 ROUT=75 VCC=9 GBW=3e6)");
        assert_eq!((p[0].vcc, p[0].vee), (9.0, -13.0));
        // SR alone is a clamp too, even with infinite rails.
        let p = mk(".model OA1 OA(AOL=1e5 ROUT=75 SR=13)");
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].sr, 13.0e6);
        assert!(p[0].vcc.is_infinite());
    }

    fn probe(vcc: f64, vee: f64, sr: f64) -> RailProbe {
        RailProbe {
            name: "U1".into(),
            node: "out".into(),
            vcc,
            vee,
            sr,
        }
    }

    #[test]
    fn rail_excursion_is_refused_and_names_the_sample() {
        let mut v = HashMap::new();
        v.insert("out".to_string(), vec![0.0, 5.0, 9.5, 2.0]);
        let err = check_rail_probes(&[probe(9.0, -9.0, f64::INFINITY)], &v, 1e-5).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("VCC"), "{msg}");
        assert!(msg.contains("op-amp U1"), "{msg}");
        // Sample 2 at dt = 1e-5.
        assert!(msg.contains("0.000020"), "{msg}");
    }

    #[test]
    fn staying_inside_the_rails_passes() {
        let mut v = HashMap::new();
        v.insert("out".to_string(), vec![0.0, 5.0, 8.9, -8.9]);
        assert!(check_rail_probes(&[probe(9.0, -9.0, f64::INFINITY)], &v, 1e-5).is_ok());
    }

    #[test]
    fn slew_limit_is_refused() {
        let mut v = HashMap::new();
        // dt = 1e-5 s, SR = 1 V/us = 1e6 V/s -> max 10 V per sample.
        v.insert("out".to_string(), vec![0.0, 5.0, 20.0]);
        let err = check_rail_probes(&[probe(f64::INFINITY, f64::NEG_INFINITY, 1.0e6)], &v, 1e-5)
            .unwrap_err();
        assert!(err.to_string().contains("slew limit"), "{err}");
        // A 9 V step is inside the 10 V budget.
        let mut ok = HashMap::new();
        ok.insert("out".to_string(), vec![0.0, 9.0, 18.0]);
        assert!(
            check_rail_probes(&[probe(f64::INFINITY, f64::NEG_INFINITY, 1.0e6)], &ok, 1e-5).is_ok()
        );
    }

    #[test]
    fn a_missing_probe_trace_refuses_rather_than_passes() {
        let v = HashMap::new();
        let err = check_rail_probes(&[probe(9.0, -9.0, f64::INFINITY)], &v, 1e-5).unwrap_err();
        assert!(err.to_string().contains("printed no trace"), "{err}");
    }

    /// `gnd` / `ground` fold to node 0 in melange's parser AND in ngspice, so
    /// the RIN/IB stamps must be skipped on them exactly as on a literal `0` —
    /// otherwise the twin carries a resistor and a source with both ends on
    /// ground, which melange never stamps.
    #[test]
    fn gnd_alias_counts_as_ground_for_the_input_stamps() {
        let deck = "amp\n\
            R1 in inv 10k\n\
            U1 GND inv out OA1\n\
            C1 out 0 1p\n\
            .model OA1 OA(AOL=1e5 ROUT=75 RIN=1e6 IB=45n)\n\
            .end\n";
        let out = translate_opamps_for_ngspice(deck, deck).unwrap();
        assert!(!out.contains("RINP_U1"), "{out}");
        assert!(!out.contains("IBP_U1"), "{out}");
        assert!(out.contains("RINM_U1 inv 0 "), "{out}");
    }

    /// This pass runs AFTER the triode/pentode passes, whose emitted ngspice
    /// `.subckt` bodies melange's own parser cannot read back. Model params
    /// therefore come from the pristine deck while the rewrite is applied to
    /// the translated one. A tube preamp with an op-amp buffer is an ordinary
    /// deck, so parsing the wrong string here would be a live bug.
    #[test]
    fn model_params_come_from_the_pristine_deck_not_the_translated_one() {
        let pristine = "Tube preamp with op-amp buffer\n\
            T1 grid plate cathode 12AX7\n\
            Rp vcc plate 100k\n\
            Rk cathode 0 1.5k\n\
            Vcc vcc 0 DC 250\n\
            Cc plate inv 100n\n\
            U1 0 inv out OA1\n\
            C1 out 0 1p\n\
            .model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)\n\
            .model OA1 OA(AOL=150000 ROUT=50)\n\
            .end\n";
        // What the triode pass hands us: ngspice B-source subckt text that
        // `Netlist::parse` cannot read.
        let translated = crate::tube_translate::translate_tubes_for_ngspice(pristine).unwrap();
        assert!(melange_solver::parser::Netlist::parse(&translated).is_err());

        let out = translate_opamps_for_ngspice(&translated, pristine).unwrap();
        // Gm = AOL/ROUT = 150000/50 = 3000, from the PRISTINE .model card.
        let gm: f64 = out
            .lines()
            .find(|l| l.starts_with("GOA_U1"))
            .expect("G card")
            .split_whitespace()
            .last()
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(gm, 3000.0);
        // The triode subckt the earlier pass emitted survives untouched.
        assert!(out.contains(".subckt MELANGE_TRIODE_12AX7"), "{out}");
    }

    #[test]
    fn explicit_transient_aol_cap_is_reported() {
        let deck = "amp\n\
            R1 in inv 10k\n\
            U1 0 inv out OA1\n\
            C1 out 0 1p\n\
            .model OA1 OA(AOL=2e5 ROUT=75 AOL_TRANSIENT_CAP=1000)\n\
            .end\n";
        let caps = transient_aol_cap_opamps(deck);
        assert_eq!(caps.len(), 1);
        assert_eq!(caps[0].0, "U1");
        assert_eq!(caps[0].2, 1000.0);
        // And a deck without the key reports nothing.
        assert!(transient_aol_cap_opamps(DECK).is_empty());
    }
}
