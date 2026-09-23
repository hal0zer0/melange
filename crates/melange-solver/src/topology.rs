//! Netlist topology checks — the wiring defects a solver cannot see.
//!
//! This pass runs on the parsed [`Netlist`](crate::parser::Netlist) *before*
//! anything is built, and answers one question the MNA/DK layer structurally
//! cannot: **is this the circuit the author meant to write?**
//!
//! # Why this exists
//!
//! A cold first-user test typo'd a node name — `C3 n3 n4 220n` became
//! `C3 n33 n4 220n` — which invents node `n33` and floats the tone stage.
//! `melange simulate` printed "Compiled successfully", clean health counters,
//! exit 0, wrote a 192 KB WAV, and reported `peak: 0.000000` without reacting.
//! Every number melange printed was correct *for the circuit it was given*.
//! `docs/NETLIST_GUIDE.md`'s own "Common Mistakes" list has floating nodes at
//! number one.
//!
//! A symptom check (`melange simulate` warning on a silent render) catches that
//! after the fact, and only on the one verb that renders audio;
//! `compile --format plugin` never renders, so a floated node shipped straight
//! into a plugin. This is the cause-side check.
//!
//! # The two checks
//!
//! ## 1. Dangling node (the sharp signature of a typo)
//!
//! A node that appears on **exactly one** element terminal in the whole deck,
//! ground excluded. `C3 n33 n4` puts `n33` on one pin of one element — and
//! usually orphans the original `n3` the same way, so a single typo produces
//! two findings that point at each other. This also catches resistor typos,
//! which the island check below misses whenever the orphaned side still has
//! another resistive path.
//!
//! A declared input or output PORT counts as a connection: `Cout vol out`
//! feeding the output port is a complete circuit, not a dangling cap. So does a
//! board pin the deck declares with `.port` — see "Board pins" below.
//!
//! ## Board pins (`.port`)
//!
//! A multi-output board — an organ filter board with ten numbered pins, a
//! divider board read one note at a time — is compiled one output at a time,
//! and its OTHER pins are then nodes named exactly once. They are not typos,
//! and `-i`/`-n` cannot say so: which pin this build reads is a property of the
//! invocation, while which pins EXIST is a property of the circuit. `.port`
//! records the second one in the deck, where the check can see it.
//!
//! The directive is **direction-neutral on purpose**: `farfisa-voicing`'s
//! undriven `in16`/`in4`/`fd_p11` are INPUT pins left unconnected in a build
//! that drives a different one, and they need exactly the same cover as an
//! output tap. It is a pin declaration, not an output list.
//!
//! A declared pin counts as **one connection for the dangling check and as
//! nothing else**. In particular it is NOT a DC path: it stamps nothing (the
//! input Thevenin conductance belongs to `-i`, not to a pin declaration), so an
//! undriven input pin behind a coupling cap is still a floating island and
//! still warns. Conflating the two would re-hide the typo'd op-amp bias
//! resistor the island check exists to catch.
//!
//! A `.port` name that is not a node in the netlist is itself
//! [`Severity::Refuse`], with the same nearest-name suggestion — otherwise the
//! declaration is just a new place for a typo to hide.
//!
//! ## 2. True cap-only DC island
//!
//! A set of nodes whose every path to the rest of the circuit is open at DC.
//! The DC operating point is singular over such an island; melange's per-node
//! gmin (and ngspice's `rshunt`) regularize it, which turns a wiring defect
//! into a quiet pass.
//!
//! Two corrections separate this from a naive "everything except capacitors is
//! a wire" scan, and both are correctness, not tuning:
//!
//! * **The port stamps are in the graph.** The input's Thevenin conductance
//!   (`MnaSystem::stamp_input_conductance`, and every `.inject` source) is a
//!   real DC path to ground. Without it, the node behind a guitar pedal's input
//!   coupling cap reads as an island — which is to say the check cried wolf on
//!   the entire pedal category.
//! * **Terminals that do not conduct at DC do not union.** A control terminal
//!   joins the graph only when its own model gives a DC path: an op-amp input
//!   (`RIN` defaults to `+inf`), a MOSFET gate (never conducts), a JFET gate and
//!   a tube grid (conduct only when forward-biased, so a floating one charges to
//!   cutoff) are all open at DC. Unioning them produced false negatives on
//!   exactly the cases that matter — a cap-coupled non-inverting op-amp input
//!   whose bias resistor was typo'd away read as "connected to the output" and
//!   passed clean.
//!
//! True islands still exist after both corrections — back-to-back electrolytics
//! wired as a non-polar pair, a capacitive divider, a node between two coupling
//! caps — so an island is a WARNING, with the hedge to confirm intent.
//!
//! # Severity
//!
//! Severity splits by **confidence**, not by output format:
//!
//! | finding | severity |
//! |---|---|
//! | dangling node on a two-terminal element (R, C, L, D, V, I, B, N), ports declared | [`Severity::Refuse`] |
//! | `.port` names a node the netlist does not have | [`Severity::Refuse`] |
//! | dangling terminal on a multi-terminal element | [`Severity::Warn`] |
//! | true cap-only DC island | [`Severity::Warn`] |
//!
//! A two-terminal element with a dangling terminal is electrically inert — it
//! carries no current, in any circuit, ever — so there is no meaningful false
//! positive to protect. A multi-terminal element is a different case: an unused
//! pot lug on a rheostat, or a transistor terminal left off in a deliberately
//! partial deck, is common and legitimate.
//!
//! # Where it is called
//!
//! One implementation, consumed by everything that builds a netlist:
//! `melange compile` (every format), `simulate`, `analyze`, `dc-op`, `nodes`
//! and `validate` all run it through [`crate::pipeline::topology_gate`], and
//! `melange-validate` reports the island findings it used to scan for itself.
//!
//! `melange nodes` REPORTS the findings and never refuses, whatever the deck
//! says: it is the tool a user reaches for to FIND a defect, so it has to stay
//! usable on a deck that has one. That is structural — it calls
//! [`crate::pipeline::topology_report`] rather than the gate, so no finding at
//! any severity can stop it, including one (`.port` naming a node that does not
//! exist) whose confidence does not depend on port knowledge at all.
//!
//! `melange dc-op` takes no `-o`, so on a deck that says nothing about its own
//! edges it passes [`Ports::unknown`]/[`Ports::inputs_only`]: without knowing
//! where the output port is, a node named once (`Cout vol out`) cannot be
//! distinguished from an orphan, and it can only warn (see
//! [`Finding::severity`]). A deck that declares its pins HAS said where its
//! edges are, so [`Ports::with_deck_pins`] hands `dc-op` that knowledge and it
//! refuses like everything else.

use crate::parser::{Element, Netlist};
use std::collections::{BTreeMap, BTreeSet};

/// How much confidence the pass has that a finding is a defect.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// Certainly wrong: refuse to build.
    Refuse,
    /// Probably wrong, legitimately possible: say so and continue.
    Warn,
}

/// One topology defect.
#[derive(Debug, Clone, PartialEq)]
pub enum Finding {
    /// A node that appears on exactly one element terminal in the whole deck.
    DanglingNode {
        /// The orphaned node name.
        node: String,
        /// The one element that names it.
        element: String,
        /// That element's label for the terminal the node sits on (`"n+"`,
        /// `"base"`, `"ctrl-"`, …).
        terminal: String,
        /// 1-based raw source line of the element, or 0 when unknown (an
        /// element synthesized by subcircuit expansion has no authored line).
        line: usize,
        /// Whether the element is two-terminal, and therefore inert.
        two_terminal: bool,
        /// Existing node names closest to `node` by edit distance, nearest
        /// first. Empty when nothing is close enough to be worth guessing.
        suggestions: Vec<String>,
        /// Whether the caller declared its input/output ports. When it did
        /// not, a port node legitimately looks dangling and the message says so.
        ports_known: bool,
    },
    /// A `.port` declaration naming something that is not a node in the deck.
    UnknownPort {
        /// The declared name, as written (normalized).
        node: String,
        /// 1-based raw source line of the `.port` statement.
        line: usize,
        /// Existing node names closest to `node` by edit distance, nearest
        /// first. Empty when nothing is close enough to be worth guessing.
        suggestions: Vec<String>,
    },
    /// A set of nodes with no DC path to ground.
    FloatingIsland {
        /// The island's nodes, sorted.
        nodes: Vec<String>,
    },
}

impl Finding {
    /// Severity of this finding. See the module table.
    ///
    /// A dangling node refuses only when the caller **declared its ports**. A
    /// verb that does not know where the circuit's input and output are cannot
    /// tell an orphaned node from an output port — `Cout vol out` names `out`
    /// exactly once — so it has no grounds to refuse. That is what keeps
    /// `dc-op` (which takes no `-o`) warn-only on an unannotated deck without
    /// anyone maintaining a list of exempt verbs: it passes
    /// [`Ports::unknown`], and the confidence the refusal rests on is simply
    /// not there. A deck that declares its own pins supplies that confidence —
    /// see [`Ports::with_deck_pins`].
    ///
    /// [`Finding::UnknownPort`] is the one finding that does NOT rest on port
    /// knowledge — the deck declares a pin and the same deck has no such node,
    /// a contradiction inside one file — so it refuses unconditionally. A verb
    /// that must never refuse anything (`melange nodes`) takes
    /// [`crate::pipeline::topology_report`] instead of the gate.
    pub fn severity(&self) -> Severity {
        match self {
            Finding::DanglingNode {
                two_terminal,
                ports_known,
                ..
            } => {
                if *two_terminal && *ports_known {
                    Severity::Refuse
                } else {
                    Severity::Warn
                }
            }
            Finding::UnknownPort { .. } => Severity::Refuse,
            Finding::FloatingIsland { .. } => Severity::Warn,
        }
    }

    /// The whole diagnostic as one line: what is wrong, where, and what to do.
    pub fn message(&self) -> String {
        match self {
            Finding::DanglingNode {
                node,
                element,
                terminal,
                line,
                two_terminal,
                suggestions,
                ports_known,
            } => {
                let at = if *line > 0 {
                    format!(" (line {line})")
                } else {
                    String::new()
                };
                let guess = match suggestions.len() {
                    0 => String::new(),
                    1 => format!(" Did you mean '{}'?", suggestions[0]),
                    _ => format!(
                        " Did you mean '{}'? (also close: {})",
                        suggestions[0],
                        suggestions[1..].join(", ")
                    ),
                };
                let port_caveat = if *ports_known {
                    ""
                } else {
                    " No input/output port was declared for this run, so if this node is \
                     one, it is connected and this is not a defect."
                };
                // Name the remedy at the moment it is needed: on a
                // multi-output board this finding IS the pin, and nothing else
                // in the output says how to tell melange so.
                let pin_hint = format!(
                    " If '{node}' is a board pin — an output tap, or an input this build \
                     leaves undriven — declare it with `.port {node}` and it counts as a \
                     connection."
                );
                if *two_terminal {
                    format!(
                        "dangling node '{node}': the whole deck names it once, as the \
                         {terminal} terminal of {element}{at}. A two-terminal element with a \
                         dangling terminal carries no current, so {element} is inert and \
                         whatever it was meant to reach is not connected.{guess}{pin_hint} If \
                         {element} is deliberately unconnected, delete it.{port_caveat}"
                    )
                } else {
                    format!(
                        "dangling terminal on {element}{at}: node '{node}' is named once in \
                         the whole deck, as its {terminal} terminal. An unconnected lug on a \
                         multi-terminal part is legitimate (an unused rheostat lug, a \
                         deliberately partial deck), so this is a warning — but if '{node}' \
                         is a typo, nothing drives that terminal.{guess}{pin_hint}{port_caveat}"
                    )
                }
            }
            Finding::UnknownPort {
                node,
                line,
                suggestions,
            } => {
                let at = if *line > 0 {
                    format!(" (line {line})")
                } else {
                    String::new()
                };
                let guess = match suggestions.len() {
                    0 => String::new(),
                    1 => format!(" Did you mean '{}'?", suggestions[0]),
                    _ => format!(
                        " Did you mean '{}'? (also close: {})",
                        suggestions[0],
                        suggestions[1..].join(", ")
                    ),
                };
                format!(
                    "`.port` declares pin '{node}'{at}, but no element in the deck names that \
                     node, so there is no such pin to declare.{guess} A `.port` line covers a \
                     node the circuit already has; it does not create one. Fix the spelling, \
                     or delete the declaration."
                )
            }
            Finding::FloatingIsland { nodes } => format!(
                "floating cap-only DC island {{{}}} — every path out of it is open at DC, so \
                 it has no DC path to ground; melange regularizes it with per-node gmin and \
                 ngspice would go singular here. Confirm this is an intended coupling-cap \
                 island, not a wiring defect.",
                nodes.join(", ")
            ),
        }
    }
}

impl std::fmt::Display for Finding {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message())
    }
}

/// The circuit's declared ports, as the verb running the check knows them.
///
/// The input port is not merely a label: `melange` stamps a Thevenin
/// conductance from it to ground before the DK kernel is built, so it is a real
/// DC path and must be in the island graph. The output port is a probe — it
/// counts as a connection for the dangling check but conducts nothing.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Ports {
    /// Input node names (normalized), in declaration order. Each gets the
    /// Thevenin conductance to ground.
    pub inputs: Vec<String>,
    /// Output node names (normalized). Probes only.
    pub outputs: Vec<String>,
    /// `true` when the caller actually knows the ports. `melange nodes` takes
    /// no `-i`/`-o`, so it passes [`Ports::unknown`] and the messages say that
    /// a port node would look dangling here.
    pub declared: bool,
}

impl Ports {
    /// Ports as a verb that takes `-i`/`-o` knows them.
    pub fn declared(
        inputs: impl IntoIterator<Item = String>,
        outputs: impl IntoIterator<Item = String>,
    ) -> Self {
        Self {
            inputs: inputs.into_iter().collect(),
            outputs: outputs.into_iter().collect(),
            declared: true,
        }
    }

    /// No port information available.
    pub fn unknown() -> Self {
        Self::default()
    }

    /// Ports for a verb that has no `-o`: whatever inputs it can name, no
    /// outputs, and `declared = false` so nothing here can refuse.
    ///
    /// `inputs` still matters — the input's Thevenin conductance is a real DC
    /// path and belongs in the island graph either way; it is the missing
    /// OUTPUT that makes a node named once (`Cout vol out`) indistinguishable
    /// from an orphan, and therefore un-refusable.
    pub fn inputs_only(inputs: impl IntoIterator<Item = String>) -> Self {
        Self {
            inputs: inputs.into_iter().collect(),
            outputs: Vec::new(),
            declared: false,
        }
    }

    /// The best a verb with no port flags at all can do: if the deck has a node
    /// literally named `in`, assume that is the input port.
    ///
    /// This is the same fallback `melange dc-op` already documents for an
    /// omitted `-i`, applied to the topology graph so `nodes` and `dc-op` do
    /// not report the input coupling cap as an island on decks where the other
    /// verbs would not. It is a guess, so it is never enough to refuse:
    /// `declared` stays `false`.
    pub fn inferred(netlist: &Netlist) -> Self {
        let has_in = netlist.elements.iter().any(|e| e.nodes().contains(&"in"));
        Self::inputs_only(has_in.then(|| "in".to_string()))
    }

    /// Take port knowledge from the deck's own `.port` declaration, if it has
    /// one.
    ///
    /// For a verb with no `-o` this is the difference between guessing and
    /// knowing. [`Ports::inferred`]'s fallback — "if there is a node called
    /// `in`, that is probably the input" — is a guess, and a guess cannot
    /// support a refusal, so such a verb can only ever warn. A deck that
    /// declares its pins has stated where its edges are; a node named exactly
    /// once that is NOT one of them is dangling on the deck's own account, and
    /// the finding stands on the same confidence every other verb's does.
    ///
    /// It only ever raises confidence. The pins themselves are read straight
    /// off the netlist by [`check`] — they are not copied into `inputs`,
    /// because a pin is not a DC path and must not stamp anything into the
    /// island graph.
    pub fn with_deck_pins(mut self, netlist: &Netlist) -> Self {
        self.declared |= !netlist.ports.is_empty();
        self
    }
}

/// Run both topology checks over `netlist`.
///
/// Findings come back in a stable order — dangling nodes by node name, then
/// islands by their first node — so two runs of the same deck print the same
/// thing. Never fails: deciding what a finding *costs* is the caller's job
/// ([`crate::pipeline::topology_gate`] refuses on [`Severity::Refuse`];
/// `melange nodes` refuses on nothing).
pub fn check(netlist: &Netlist, ports: &Ports) -> Vec<Finding> {
    let mut findings = check_declared_ports(netlist);
    findings.extend(check_dangling(netlist, ports));
    let dangling: BTreeSet<String> = findings
        .iter()
        .filter_map(|f| match f {
            Finding::DanglingNode { node, .. } => Some(node.clone()),
            _ => None,
        })
        .collect();
    findings.extend(check_islands(netlist, ports, &dangling));
    findings
}

// ---------------------------------------------------------------------------
// Check 0 — `.port` declarations name real nodes
// ---------------------------------------------------------------------------

/// Every `.port` pin must be a node some element names.
///
/// Without this the declaration is simply a second place for a typo to hide,
/// and a worse one than the first: a mistyped `.port` name silently fails to
/// cover the pin it was written for, so the deck goes back to being refused
/// for a reason that now looks wrong. Findings come back in declaration order.
fn check_declared_ports(netlist: &Netlist) -> Vec<Finding> {
    if netlist.ports.is_empty() {
        return Vec::new();
    }
    let existing: BTreeSet<&str> = netlist
        .elements
        .iter()
        .flat_map(|e| e.nodes())
        .filter(|n| *n != "0")
        .collect();
    let all_names: Vec<&str> = existing.iter().copied().collect();
    netlist
        .ports
        .iter()
        .filter(|pin| !existing.contains(pin.node.as_str()))
        .map(|pin| Finding::UnknownPort {
            node: pin.node.clone(),
            line: pin.line,
            suggestions: nearest_names(&pin.node, &all_names),
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Check 1 — dangling nodes
// ---------------------------------------------------------------------------

fn check_dangling(netlist: &Netlist, ports: &Ports) -> Vec<Finding> {
    // One entry per node: how many element terminals name it, and the first
    // (element index, terminal index) that does.
    let mut terminals: BTreeMap<&str, (usize, usize, usize)> = BTreeMap::new();
    for (ei, elem) in netlist.elements.iter().enumerate() {
        for (ti, node) in elem.nodes().iter().enumerate() {
            let entry = terminals.entry(node).or_insert((0, ei, ti));
            entry.0 += 1;
        }
    }

    // Everything else that names a node without being a terminal. These do not
    // conduct, but they prove the author meant the node to exist, so they stop
    // it reading as an orphan: a `.tap`/`.inject`/`.delay_feedback` target, a
    // node sensed inside a behavioral `B` expression, a `.port` board pin, and
    // the ports this invocation declared.
    //
    // A `.port` pin lands HERE and nowhere else in this file: it is one
    // connection for this check, and not a DC path for `check_islands`.
    let mut referenced: BTreeSet<String> = BTreeSet::new();
    for pin in &netlist.ports {
        referenced.insert(pin.node.clone());
    }
    for inj in &netlist.injections {
        referenced.insert(inj.node.clone());
    }
    for tap in &netlist.taps {
        referenced.insert(tap.node.clone());
    }
    for n in &netlist.delay_feedback_nodes {
        referenced.insert(n.clone());
    }
    for elem in &netlist.elements {
        if let Element::BSource { expr, .. } = elem {
            referenced.extend(expr.referenced_nodes());
        }
    }
    for n in ports.inputs.iter().chain(ports.outputs.iter()) {
        referenced.insert(n.clone());
    }

    // Candidate spellings for the "did you mean" — every node the deck names,
    // INCLUDING the other dangling ones. A single typo orphans both sides
    // (`C3 n3 n4` → `C3 n33 n4` leaves `n3` on one terminal too), so excluding
    // dangling nodes would drop the one suggestion that matters.
    let all_names: Vec<&str> = terminals.keys().copied().collect();

    let mut findings = Vec::new();
    for (node, (count, ei, ti)) in &terminals {
        if *node == "0" || *count != 1 || referenced.contains(*node) {
            continue;
        }
        let elem = &netlist.elements[*ei];
        findings.push(Finding::DanglingNode {
            node: (*node).to_string(),
            element: elem.name().to_string(),
            terminal: terminal_label(elem, *ti),
            line: netlist
                .element_lines
                .get(&elem.name().to_ascii_lowercase())
                .copied()
                .unwrap_or(0),
            two_terminal: is_two_terminal(elem),
            suggestions: nearest_names(node, &all_names),
            ports_known: ports.declared,
        });
    }
    findings
}

/// The element's own name for terminal `idx`, in the node order
/// [`Element::nodes`] returns.
fn terminal_label(elem: &Element, idx: usize) -> String {
    let labels: &[&str] = match elem {
        Element::Resistor { .. }
        | Element::Capacitor { .. }
        | Element::Inductor { .. }
        | Element::VoltageSource { .. }
        | Element::CurrentSource { .. }
        | Element::BSource { .. } => &["n+", "n-"],
        Element::Diode { .. } | Element::Glow { .. } => &["anode", "cathode"],
        Element::Bjt { .. } => &["collector", "base", "emitter"],
        Element::Jfet { .. } => &["drain", "gate", "source"],
        Element::Mosfet { .. } => &["drain", "gate", "source", "bulk"],
        Element::Opamp { .. } => &["+in", "-in", "out"],
        Element::Triode { .. } => &["grid", "plate", "cathode"],
        Element::Pentode { .. } => &["plate", "grid", "cathode", "screen", "suppressor"],
        Element::Vca { .. } => &["sig+", "sig-", "ctrl+", "ctrl-"],
        Element::Ldr { .. } => &["r+", "r-", "ctrl+", "ctrl-"],
        Element::Vcvs { .. } | Element::Vccs { .. } => &["out+", "out-", "ctrl+", "ctrl-"],
        Element::SubcktInstance { .. } => &[],
    };
    labels
        .get(idx)
        .map(|s| (*s).to_string())
        .unwrap_or_else(|| format!("pin {}", idx + 1))
}

/// Whether a dangling terminal makes this element inert.
///
/// Matched on the variant rather than on `nodes().len()`: a two-pin subcircuit
/// instance has two nodes and is not a two-terminal component, and the
/// difference decides whether a deck is refused or warned about.
fn is_two_terminal(elem: &Element) -> bool {
    matches!(
        elem,
        Element::Resistor { .. }
            | Element::Capacitor { .. }
            | Element::Inductor { .. }
            | Element::VoltageSource { .. }
            | Element::CurrentSource { .. }
            | Element::Diode { .. }
            | Element::Glow { .. }
            | Element::BSource { .. }
    )
}

/// Existing node names closest to `query` by Levenshtein distance, nearest
/// first, at most three.
///
/// The threshold is deliberately tight — one edit for a short name, two for a
/// longer one — because a wrong guess in an error message is worse than none.
///
/// Ties break toward a name one of which is a prefix of the other, because a
/// stray or dropped keystroke at the end of a name is the typo this check
/// exists for: `n3` is one edit from `n1`, `n2` AND `n33`, and `n33` is the one
/// worth naming first.
fn nearest_names(query: &str, candidates: &[&str]) -> Vec<String> {
    let limit = if query.chars().count() <= 3 { 1 } else { 2 };
    let mut scored: Vec<(usize, u8, &str)> = candidates
        .iter()
        .filter(|c| **c != query && **c != "0")
        .filter_map(|c| {
            let d = edit_distance(query, c);
            let affix = u8::from(!(c.starts_with(query) || query.starts_with(*c)));
            (d <= limit && d < query.chars().count()).then_some((d, affix, *c))
        })
        .collect();
    scored.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.1.cmp(&b.1))
            .then_with(|| a.2.cmp(b.2))
    });
    scored.truncate(3);
    scored.into_iter().map(|(_, _, n)| n.to_string()).collect()
}

/// Levenshtein edit distance (insert / delete / substitute, all cost 1).
fn edit_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    if a.is_empty() {
        return b.len();
    }
    if b.is_empty() {
        return a.len();
    }
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for i in 1..=a.len() {
        cur[0] = i;
        for j in 1..=b.len() {
            let sub = prev[j - 1] + usize::from(a[i - 1] != b[j - 1]);
            cur[j] = sub.min(prev[j] + 1).min(cur[j - 1] + 1);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}

// ---------------------------------------------------------------------------
// Check 2 — cap-only DC islands
// ---------------------------------------------------------------------------

/// Union-find over the DC graph melange actually stamps.
struct Dsu {
    index: BTreeMap<String, usize>,
    parent: Vec<usize>,
}

impl Dsu {
    fn new() -> Self {
        Self {
            index: BTreeMap::new(),
            parent: Vec::new(),
        }
    }

    fn intern(&mut self, name: &str) -> usize {
        if let Some(&i) = self.index.get(name) {
            return i;
        }
        let i = self.parent.len();
        self.parent.push(i);
        self.index.insert(name.to_string(), i);
        i
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]]; // path halving
            x = self.parent[x];
        }
        x
    }

    /// Union two nodes. Both are interned first, so a DC edge to ground
    /// registers ground even in a deck that never wrote `0`.
    fn union(&mut self, a: &str, b: &str) {
        let (ai, bi) = (self.intern(a), self.intern(b));
        let (ra, rb) = (self.find(ai), self.find(bi));
        if ra != rb {
            self.parent[ra] = rb;
        }
    }
}

fn check_islands(netlist: &Netlist, ports: &Ports, dangling: &BTreeSet<String>) -> Vec<Finding> {
    let mut dsu = Dsu::new();

    // Register every node the deck names, so a node reachable only through
    // capacitors exists as its own singleton island rather than vanishing.
    for elem in &netlist.elements {
        for n in elem.nodes() {
            dsu.intern(n);
        }
    }

    // Ground must be in the graph for "does not contain ground" to mean
    // anything. A deck that never mentions `0` has a different (and louder)
    // problem, which `Parser::warn_if_no_ground` already reports.
    if !dsu.index.contains_key("0") {
        return Vec::new();
    }

    for elem in &netlist.elements {
        for (a, b) in dc_edges(elem, netlist) {
            dsu.union(&a, &b);
        }
    }

    // The port and `.inject` stamps. The input's Thevenin conductance
    // (`stamp_input_conductance`) and every `.inject` source's mandatory
    // R= / RSHUNT= are real conductances to ground, stamped before the DK
    // kernel is built — without them every cap-coupled input node in every
    // guitar pedal reads as an island. The OUTPUT port is a probe and stamps
    // nothing, so it is deliberately absent here.
    for n in &ports.inputs {
        dsu.union(n, "0");
    }
    for inj in &netlist.injections {
        dsu.union(&inj.node, "0");
    }

    let ground_root = {
        let g = dsu.index["0"];
        dsu.find(g)
    };

    let names: Vec<String> = dsu.index.keys().cloned().collect();
    let mut components: BTreeMap<usize, Vec<String>> = BTreeMap::new();
    for name in names {
        let id = dsu.index[&name];
        let root = dsu.find(id);
        if root == ground_root {
            continue;
        }
        components.entry(root).or_default().push(name);
    }

    components
        .into_values()
        .filter(|nodes| {
            // An island made up entirely of nodes already reported as dangling
            // is the same defect said twice; the dangling finding names the
            // element and the line, so it is the better of the two.
            !nodes.iter().all(|n| dangling.contains(n))
        })
        .map(|mut nodes| {
            nodes.sort();
            Finding::FloatingIsland { nodes }
        })
        .collect()
}

/// The DC conduction edges an element contributes, as node-name pairs.
///
/// This is the table the whole island check rests on, and every entry is a
/// statement about what `mna.rs` stamps:
///
/// * **Capacitor** — open at DC. No edge. (The one entry the old validate-side
///   scan had right.)
/// * **Op-amp** — the output stage stamps `go = 1/ROUT` from the output node to
///   ground (`mna.rs`, "Simple VCCS"), so the output is DC-tied to ground. The
///   inputs stamp `1/RIN` to ground **only when `RIN` is finite**; it defaults
///   to `+inf`, which is the ideal op-amp, and an ideal op-amp input is an open
///   circuit. This is the arbiter's concrete miss: unioning the three op-amp
///   terminals made a cap-coupled non-inverting input whose bias resistor was
///   typo'd away read as "connected to the output".
/// * **JFET / MOSFET** — channel only. The MOSFET gate is insulated
///   (`mna.rs` stamps its `N_i` column "for framework" with zero current) and
///   its bulk terminal is not stamped at all; the JFET gate is a gate-source
///   diode whose reverse branch is the constant `-IS` (`jfet.rs`
///   `gate_current`), i.e. zero conductance, so a floating gate charges to
///   cutoff instead of being biased.
/// * **Triode / pentode** — plate-cathode, plus screen-cathode on a pentode
///   (the screen draws real DC current). NOT the grid: `Ig = ig_max ·
///   max(0, Vgk/vgk_onset)^1.5` (`tube.rs`) is exactly zero below onset, so a
///   grid with no leak resistor has no DC path. The pentode suppressor is
///   modeled as cathode-tied and stamps nothing.
/// * **BJT** — both junctions. Unlike a gate or a grid, a base-emitter diode
///   has nonzero conductance at every bias, so it does anchor the base.
/// * **VCA / LDR** — signal (resistance) path only; the control pair is sensed
///   and carries `I_ctrl = 0` by construction.
/// * **VCVS / VCCS** — output pair only; the control pair is sensed.
///
/// Deliberately NOT modeled as opens, though a strict reading of "conductance
/// to ground" would: an independent current source, a `B` current source, and a
/// VCCS output pair. All three are opens at DC in the same sense a MOSFET gate
/// is, but they are not *control* terminals, nobody has reported a defect they
/// hide, and treating them as conductive is what this check has always done.
/// Changing that belongs in its own pass with its own corpus sweep.
fn dc_edges(elem: &Element, netlist: &Netlist) -> Vec<(String, String)> {
    let pair = |a: &str, b: &str| vec![(a.to_string(), b.to_string())];
    match elem {
        // Open at DC — the entire point of the check.
        Element::Capacitor { .. } => Vec::new(),

        Element::Resistor {
            n_plus, n_minus, ..
        }
        | Element::Inductor {
            n_plus, n_minus, ..
        }
        | Element::VoltageSource {
            n_plus, n_minus, ..
        }
        | Element::CurrentSource {
            n_plus, n_minus, ..
        }
        | Element::Diode {
            n_plus, n_minus, ..
        }
        | Element::BSource {
            n_plus, n_minus, ..
        } => pair(n_plus, n_minus),

        Element::Glow {
            n_anode, n_cathode, ..
        } => pair(n_anode, n_cathode),

        Element::Bjt { nc, nb, ne, .. } => {
            vec![(nc.clone(), ne.clone()), (nb.clone(), ne.clone())]
        }
        Element::Jfet { nd, ns, .. } => pair(nd, ns),
        Element::Mosfet { nd, ns, .. } => pair(nd, ns),

        Element::Triode {
            n_plate, n_cathode, ..
        } => pair(n_plate, n_cathode),
        Element::Pentode {
            n_plate,
            n_cathode,
            n_screen,
            ..
        } => vec![
            (n_plate.clone(), n_cathode.clone()),
            (n_screen.clone(), n_cathode.clone()),
        ],

        Element::Opamp {
            n_plus,
            n_minus,
            n_out,
            model,
            ..
        } => {
            let mut edges = pair(n_out, "0");
            if opamp_rin_conducts(netlist, model) {
                edges.push((n_plus.clone(), "0".to_string()));
                edges.push((n_minus.clone(), "0".to_string()));
            }
            edges
        }

        Element::Vca {
            n_sig_p, n_sig_n, ..
        } => pair(n_sig_p, n_sig_n),
        Element::Ldr {
            n_plus, n_minus, ..
        } => pair(n_plus, n_minus),

        Element::Vcvs { out_p, out_n, .. } | Element::Vccs { out_p, out_n, .. } => {
            pair(out_p, out_n)
        }

        // Unexpanded subcircuit instance: its internals are unknown here, so
        // assume every pin is connected rather than invent an island. Every
        // shipping verb expands before this pass runs.
        Element::SubcktInstance { nodes, .. } => nodes
            .windows(2)
            .map(|w| (w[0].clone(), w[1].clone()))
            .collect(),
    }
}

/// Whether this op-amp's `.model` gives its inputs a DC path to ground.
///
/// `RIN` defaults to `+inf` (ideal op-amp, no input conductance stamped); a
/// finite positive `RIN` stamps `1/RIN` from each input pin to ground.
fn opamp_rin_conducts(netlist: &Netlist, model_name: &str) -> bool {
    netlist
        .models
        .iter()
        .find(|m| m.name.eq_ignore_ascii_case(model_name))
        .and_then(|m| {
            m.params
                .iter()
                .find(|(k, _)| k.eq_ignore_ascii_case("RIN"))
                .map(|(_, v)| *v)
        })
        .map(|rin| rin.is_finite() && rin > 0.0)
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn findings(deck: &str, ports: &Ports) -> Vec<Finding> {
        let netlist = Netlist::parse(deck).expect("deck parses");
        check(&netlist, ports)
    }

    fn ports_in_out() -> Ports {
        Ports::declared(["in".to_string()], ["out".to_string()])
    }

    #[test]
    fn typoed_cap_node_refuses_and_suggests_the_original() {
        // The cold-first-user case: `C3 n3 n4` written as `C3 n33 n4`.
        let deck = "typo\n\
                    R1 in n3 10k\n\
                    C3 n33 n4 220n\n\
                    R2 n4 0 10k\n\
                    R3 n4 out 1k\n\
                    Rload out 0 100k\n";
        let f = findings(deck, &ports_in_out());
        let dangling: Vec<&Finding> = f
            .iter()
            .filter(|f| matches!(f, Finding::DanglingNode { .. }))
            .collect();
        assert_eq!(dangling.len(), 2, "both sides of the typo orphan: {f:?}");
        let n33 = dangling
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "n33"))
            .expect("n33 reported");
        assert_eq!(n33.severity(), Severity::Refuse);
        assert!(
            n33.message().contains("Did you mean 'n3'?"),
            "{}",
            n33.message()
        );
        assert!(n33.message().contains("line 3"), "{}", n33.message());
        assert!(n33.message().contains("C3"), "{}", n33.message());
    }

    #[test]
    fn port_nodes_are_connections_not_dangling_terminals() {
        // `Cout vol out` with nothing else on `out` is a complete circuit when
        // `out` is the declared output port.
        let deck = "port\n\
                    R1 in vol 10k\n\
                    Rvol vol 0 100k\n\
                    Cout vol out 100n\n";
        assert!(
            findings(deck, &ports_in_out())
                .iter()
                .all(|f| !matches!(f, Finding::DanglingNode { .. })),
            "{:?}",
            findings(deck, &ports_in_out())
        );
    }

    /// A two-output board: `outb` is a real pin, but this build reads `outa`.
    const BOARD: &str = "board\n\
                         R1 in n1 10k\n\
                         R2 n1 0 100k\n\
                         R3 n1 outa 10k\n\
                         R4 n1 outb 22k\n\
                         Rl outa 0 100k\n";

    #[test]
    fn an_undeclared_board_pin_still_refuses() {
        // No grandfather clause: a deck that declares nothing is exactly the
        // deck the refusal was built for (the cold tester had no annotations).
        let f = findings(
            BOARD,
            &Ports::declared(["in".to_string()], ["outa".to_string()]),
        );
        let d = f
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "outb"))
            .expect("outb reported");
        assert_eq!(d.severity(), Severity::Refuse);
    }

    #[test]
    fn the_dangling_refusal_names_the_port_directive() {
        // The fix has to be discoverable at the moment it is needed: this
        // message is the only place a board author is told `.port` exists.
        let f = findings(
            BOARD,
            &Ports::declared(["in".to_string()], ["outa".to_string()]),
        );
        let d = f
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "outb"))
            .expect("outb reported");
        assert!(d.message().contains("`.port outb`"), "{}", d.message());
    }

    #[test]
    fn a_declared_board_pin_is_a_connection() {
        let deck = format!("{BOARD}.port outb\n");
        let f = findings(
            &deck,
            &Ports::declared(["in".to_string()], ["outa".to_string()]),
        );
        assert!(f.is_empty(), "{f:?}");
    }

    #[test]
    fn a_declared_pin_is_direction_neutral() {
        // farfisa-voicing's `in16`/`in4`: INPUT pins this build leaves
        // undriven. One directive has to cover them exactly as it covers an
        // output tap — `.port` declares a pin, not an output.
        let deck = "voicing\n\
                    R_bar16 in16 bus 470k\n\
                    R_bar8 in bus 470k\n\
                    Rb bus 0 100k\n\
                    Ro bus out 10k\n\
                    Rl out 0 100k\n\
                    .port in16\n";
        let f = findings(deck, &ports_in_out());
        assert!(f.is_empty(), "{f:?}");
    }

    #[test]
    fn a_declared_pin_is_not_a_dc_path_so_the_island_still_warns() {
        // The one conflation that would re-hide the typo'd op-amp bias
        // resistor: an undriven input pin behind a coupling cap has no DC path
        // to ground, and declaring the pin says nothing about that.
        let deck = "pin\n\
                    R1 in n1 10k\n\
                    R2 n1 0 100k\n\
                    R3 n1 out 10k\n\
                    Rl out 0 100k\n\
                    C2 pin2 n1 100n\n\
                    .port pin2\n";
        let f = findings(deck, &ports_in_out());
        assert!(
            !f.iter()
                .any(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "pin2")),
            "the pin is declared, so it is not dangling: {f:?}"
        );
        let island = f
            .iter()
            .find(|f| matches!(f, Finding::FloatingIsland { nodes } if nodes == &["pin2".to_string()]))
            .expect("a declared pin is not a DC path — the island must still be reported");
        assert_eq!(island.severity(), Severity::Warn);
    }

    #[test]
    fn a_port_naming_a_node_the_deck_lacks_refuses_with_a_suggestion() {
        // `.port outbb` for `outb` — a stray keystroke in the declaration is
        // exactly the failure this check exists to stop hiding.
        let deck = format!("{BOARD}.port outa outbb\n");
        let f = findings(
            &deck,
            &Ports::declared(["in".to_string()], ["outa".to_string()]),
        );
        let u = f
            .iter()
            .find(|f| matches!(f, Finding::UnknownPort { node, .. } if node == "outbb"))
            .expect("outbb reported");
        assert_eq!(u.severity(), Severity::Refuse);
        assert!(
            u.message().contains("Did you mean 'outb'?"),
            "{}",
            u.message()
        );
        assert!(u.message().contains("line 7"), "{}", u.message());
    }

    #[test]
    fn a_port_refuses_even_when_the_verb_knows_no_ports() {
        // Unlike a dangling node, this finding does not rest on port
        // knowledge: the deck contradicts itself. `melange nodes` stays usable
        // by not calling the gate at all (`pipeline::topology_report`).
        let deck = format!("{BOARD}.port outc\n");
        let netlist = Netlist::parse(&deck).expect("parses");
        let f = check(&netlist, &Ports::inferred(&netlist));
        assert!(
            f.iter()
                .any(|f| matches!(f, Finding::UnknownPort { .. })
                    && f.severity() == Severity::Refuse),
            "{f:?}"
        );
    }

    #[test]
    fn pedal_input_behind_a_coupling_cap_is_not_an_island() {
        // The false positive that made the old scan useless on every pedal:
        // `in` sees only a coupling cap, but the input port stamps a Thevenin
        // conductance to ground there.
        let deck = "pedal\n\
                    C1 in g1 100n\n\
                    R1 g1 0 1meg\n\
                    R2 g1 out 10k\n\
                    Rl out 0 100k\n";
        let f = findings(deck, &ports_in_out());
        assert!(f.is_empty(), "{f:?}");
    }

    #[test]
    fn opamp_input_with_its_bias_resistor_gone_is_an_island() {
        // The arbiter's concrete miss. Unioning all three op-amp terminals made
        // `np` read as connected through the op-amp to its output.
        let deck = "opamp\n\
                    .model OA OA(AOL=200000 ROUT=100)\n\
                    C1 in np 100n\n\
                    U1 np nm vo OA\n\
                    R1 nm 0 1k\n\
                    R2 nm vo 10k\n\
                    Rl vo 0 100k\n";
        let f = findings(
            deck,
            &Ports::declared(["in".to_string()], ["vo".to_string()]),
        );
        assert_eq!(
            f.iter()
                .filter(|f| matches!(f, Finding::FloatingIsland { nodes } if nodes == &["np".to_string()]))
                .count(),
            1,
            "{f:?}"
        );
    }

    #[test]
    fn opamp_input_with_a_bias_resistor_is_clean() {
        let deck = "opamp\n\
                    .model OA OA(AOL=200000 ROUT=100)\n\
                    C1 in np 100n\n\
                    Rb np 0 1meg\n\
                    U1 np nm vo OA\n\
                    R1 nm 0 1k\n\
                    R2 nm vo 10k\n\
                    Rl vo 0 100k\n";
        let f = findings(
            deck,
            &Ports::declared(["in".to_string()], ["vo".to_string()]),
        );
        assert!(f.is_empty(), "{f:?}");
    }

    #[test]
    fn finite_rin_gives_the_opamp_inputs_a_dc_path() {
        let deck = "opamp\n\
                    .model OA OA(AOL=200000 ROUT=100 RIN=1e12)\n\
                    C1 in np 100n\n\
                    U1 np nm vo OA\n\
                    R1 nm 0 1k\n\
                    R2 nm vo 10k\n\
                    Rl vo 0 100k\n";
        let f = findings(
            deck,
            &Ports::declared(["in".to_string()], ["vo".to_string()]),
        );
        assert!(f.is_empty(), "RIN=1e12 stamps 1/RIN to ground: {f:?}");
    }

    #[test]
    fn floating_tube_grid_is_an_island() {
        let deck = "tube\n\
                    .model T12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 RGI=2000)\n\
                    Vcc vcc 0 300\n\
                    C1 in g 22n\n\
                    T1 g p k T12AX7\n\
                    Rp vcc p 100k\n\
                    Rk k 0 1500\n\
                    Rl p 0 1meg\n";
        let f = findings(
            deck,
            &Ports::declared(["in".to_string()], ["p".to_string()]),
        );
        assert_eq!(
            f.iter()
                .filter(|f| matches!(f, Finding::FloatingIsland { nodes } if nodes == &["g".to_string()]))
                .count(),
            1,
            "a grid with no leak resistor has no DC path: {f:?}"
        );
    }

    #[test]
    fn grid_leak_resistor_clears_the_tube_grid() {
        let deck = "tube\n\
                    .model T12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 RGI=2000)\n\
                    Vcc vcc 0 300\n\
                    C1 in g 22n\n\
                    Rg g 0 1meg\n\
                    T1 g p k T12AX7\n\
                    Rp vcc p 100k\n\
                    Rk k 0 1500\n\
                    Rl p 0 1meg\n";
        let f = findings(
            deck,
            &Ports::declared(["in".to_string()], ["p".to_string()]),
        );
        assert!(f.is_empty(), "{f:?}");
    }

    #[test]
    fn unused_rheostat_lug_warns_rather_than_refuses() {
        // A three-terminal part with a lug left off is legitimate.
        let deck = "pot\n\
                    .model J1 NJF(VTO=-2 BETA=1e-3)\n\
                    R1 in mid 10k\n\
                    R2 mid 0 10k\n\
                    J1 mid unused_g src J1\n\
                    Rs src 0 1k\n\
                    Ro mid out 1k\n\
                    Rl out 0 100k\n";
        let f = findings(deck, &ports_in_out());
        let d = f
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "unused_g"))
            .expect("gate reported");
        assert_eq!(d.severity(), Severity::Warn, "{}", d.message());
    }

    #[test]
    fn back_to_back_electrolytics_still_warn() {
        // A true island survives both corrections — that is why it is a warning
        // and not a refusal.
        let deck = "nonpolar\n\
                    R1 in a 10k\n\
                    C1 a mid 10u\n\
                    C2 mid out 10u\n\
                    Ra a 0 100k\n\
                    Rl out 0 100k\n";
        let f = findings(deck, &ports_in_out());
        assert_eq!(
            f,
            vec![Finding::FloatingIsland {
                nodes: vec!["mid".to_string()]
            }]
        );
    }

    #[test]
    fn a_clean_deck_produces_nothing() {
        let deck = "rc\n\
                    R1 in out 1k\n\
                    C1 out 0 1u\n";
        assert!(findings(deck, &ports_in_out()).is_empty());
    }

    #[test]
    fn inject_directive_is_a_dc_path_to_ground() {
        let deck = "inject\n\
                    R1 in out 1k\n\
                    C1 out 0 1u\n\
                    C2 out fb 100n\n\
                    .inject fb fb_drive RSHUNT=1meg\n";
        assert!(findings(deck, &ports_in_out()).is_empty());
    }

    #[test]
    fn edit_distance_is_symmetric_and_counts_single_edits() {
        assert_eq!(edit_distance("n33", "n3"), 1);
        assert_eq!(edit_distance("n3", "n33"), 1);
        assert_eq!(edit_distance("vol", "vo1"), 1);
        assert_eq!(edit_distance("", "abc"), 3);
        assert_eq!(edit_distance("abc", "abc"), 0);
    }

    #[test]
    fn a_dropped_keystroke_outranks_an_equally_close_substitution() {
        // `n3` is one edit from `n1`, `n2` and `n33`. The typo that produced
        // this deck is the last one.
        assert_eq!(
            nearest_names("n3", &["n1", "n2", "n33", "n4"]),
            vec!["n33", "n1", "n2"]
        );
    }

    #[test]
    fn inferred_ports_stamp_the_input_but_cannot_refuse() {
        // What `nodes` / `dc-op` see: no port flags at all, but a node named
        // `in` is the input by the convention `dc-op` already documents.
        let deck = "pedal\n\
                    C1 in g1 100n\n\
                    R1 g1 0 1meg\n\
                    R2 g1 mid 10k\n\
                    Cout mid out 100n\n\
                    Rm mid 0 100k\n";
        let netlist = Netlist::parse(deck).expect("parses");
        let f = check(&netlist, &Ports::inferred(&netlist));
        assert!(
            !f.iter().any(
                |f| matches!(f, Finding::FloatingIsland { nodes } if nodes == &["in".to_string()])
            ),
            "the inferred input port stamps a DC path: {f:?}"
        );
        // `out` sits on one terminal and no output port was declared, so it is
        // reported — but only ever as a warning.
        let out = f
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "out"))
            .expect("out reported");
        assert_eq!(out.severity(), Severity::Warn);
    }

    #[test]
    fn a_deck_declaration_lets_a_verb_with_no_output_flag_refuse() {
        // `dc-op` guesses its ports, so it can only warn — until the deck says
        // where its edges are. Then a node named once that is not one of them
        // is dangling on the deck's own account.
        let deck = format!("{BOARD}.port outa\n");
        let netlist = Netlist::parse(&deck).expect("parses");
        let guessing = check(&netlist, &Ports::inferred(&netlist));
        let knowing = check(
            &netlist,
            &Ports::inferred(&netlist).with_deck_pins(&netlist),
        );
        let sev = |fs: &[Finding]| {
            fs.iter()
                .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "outb"))
                .expect("outb reported")
                .severity()
        };
        assert_eq!(sev(&guessing), Severity::Warn);
        assert_eq!(sev(&knowing), Severity::Refuse);
    }

    #[test]
    fn a_deck_with_no_declaration_keeps_the_guess_and_its_warning() {
        let netlist = Netlist::parse(BOARD).expect("parses");
        let f = check(
            &netlist,
            &Ports::inferred(&netlist).with_deck_pins(&netlist),
        );
        let d = f
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "outb"))
            .expect("outb reported");
        assert_eq!(d.severity(), Severity::Warn);
        assert!(
            d.message().contains("No input/output port was declared"),
            "{}",
            d.message()
        );
    }

    #[test]
    fn suggestions_stay_quiet_when_nothing_is_close() {
        let deck = "far\n\
                    R1 in out 1k\n\
                    C1 out 0 1u\n\
                    Rx completely_different_name 0 1k\n";
        let f = findings(deck, &ports_in_out());
        let d = f
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "completely_different_name"))
            .expect("reported");
        match d {
            Finding::DanglingNode { suggestions, .. } => assert!(suggestions.is_empty()),
            _ => unreachable!(),
        }
    }

    #[test]
    fn undeclared_ports_say_so_in_the_message() {
        let deck = "port\n\
                    R1 in vol 10k\n\
                    Rvol vol 0 100k\n\
                    Cout vol out 100n\n";
        let f = findings(deck, &Ports::unknown());
        let d = f
            .iter()
            .find(|f| matches!(f, Finding::DanglingNode { node, .. } if node == "out"))
            .expect("out looks dangling with no ports declared");
        assert!(
            d.message().contains("No input/output port was declared"),
            "{}",
            d.message()
        );
        assert_eq!(
            d.severity(),
            Severity::Warn,
            "a verb that does not know its ports cannot refuse: {}",
            d.message()
        );
    }
}
