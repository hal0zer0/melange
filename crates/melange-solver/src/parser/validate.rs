//! Post-parse validation and parse-time warnings.

use super::*;

impl Parser {
    /// SPICE semantics: line 1 is ALWAYS the title, even if it looks like an
    /// element or directive. Warn when the consumed title is plausibly a
    /// circuit line so a missing title line fails loudly instead of silently
    /// dropping the first component.
    pub(super) fn warn_if_title_looks_like_content(title: &str) {
        let t = title.trim();
        if t.is_empty() {
            return;
        }
        if t.starts_with('.') {
            log::warn!(
                "first line '{}' was consumed as the netlist title (SPICE semantics) but looks \
                 like a directive — add a title line above it if this was unintended",
                t
            );
            return;
        }
        let toks: Vec<&str> = t.split_whitespace().collect();
        let first = toks[0];
        let first_char = first.chars().next().unwrap_or(' ').to_ascii_uppercase();
        let known_element_letter = matches!(
            first_char,
            'R' | 'C'
                | 'L'
                | 'V'
                | 'I'
                | 'D'
                | 'Q'
                | 'J'
                | 'M'
                | 'T'
                | 'P'
                | 'U'
                | 'Y'
                | 'O'
                | 'E'
                | 'G'
                | 'X'
                | 'B'
                | 'K'
        );
        // Require an element-name shape (letter + digit somewhere, identifier
        // chars only) to keep prose titles like "RC lowpass test" quiet.
        let looks_like_element_name = known_element_letter
            && first.chars().skip(1).any(|c| c.is_ascii_digit())
            && first.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
        if looks_like_element_name && toks.len() >= 3 {
            log::warn!(
                "first line '{}' was consumed as the netlist title (SPICE semantics) but looks \
                 like a circuit element — add a title line above it if this was unintended",
                t
            );
        }
    }

    /// Warn when nothing in the deck references the ground node.
    ///
    /// MNA takes node 0 as the reference and deletes its row, so a deck that
    /// never mentions `0` (or `gnd`/`ground`, which normalize to it) still
    /// produces a square system — one whose solution is only defined up to an
    /// arbitrary offset that melange picks silently. `dc-op` then reports
    /// "Converged: true" and a KCL residual of ~1e-19, which reads as a
    /// *verified* answer. ngspice refuses the same deck outright
    /// ("no ground node found"); melange should at minimum say so.
    ///
    /// Warn rather than error: a fragment under test, a deck whose only ground
    /// is inside a `.subckt` body, and a floating-supply topology are all
    /// legitimate reasons to see no top-level `0`, and this runs before
    /// subcircuit expansion. Subcircuit bodies are scanned too, so a ground
    /// that lives only inside one does not trip the warning.
    pub(super) fn warn_if_no_ground(netlist: &Netlist) {
        if netlist.elements.is_empty() {
            return;
        }
        let touches_ground = |elems: &[Element]| {
            elems
                .iter()
                .any(|e| e.nodes().iter().any(|n| n.trim() == "0"))
        };
        if touches_ground(&netlist.elements)
            || netlist
                .subcircuits
                .iter()
                .any(|sc| touches_ground(&sc.elements))
        {
            return;
        }
        log::warn!(
            "no element references the ground node '0' — MNA is solving relative to a \
             reference melange picked on its own, so node voltages are defined only up to \
             an arbitrary offset and a DC operating point will report convergence on a \
             circuit that has no ground. ngspice rejects this deck outright. Connect one \
             node to '0' (or 'gnd'/'ground', which alias to it)."
        );
    }

    /// Post-parse validation: duplicate names, model references, pot targets.
    pub(super) fn validate_netlist(&self, netlist: &Netlist) -> Result<(), ParseError> {
        // Check for duplicate component names (case-insensitive)
        let mut seen_names = std::collections::HashSet::new();
        for elem in &netlist.elements {
            if !seen_names.insert(elem.name().to_ascii_lowercase()) {
                return Err(ParseError {
                    line: self.dup_line_of_element(elem.name()),
                    message: format!("Duplicate component name: '{}'", elem.name()),
                });
            }
        }

        // Check for duplicate .model names (case-insensitive). Before this
        // check the FIRST definition silently won — the project's worst
        // failure mode (a wrong circuit that solves perfectly).
        let mut seen_model_names = std::collections::HashSet::new();
        for model in &netlist.models {
            if !seen_model_names.insert(model.name.to_ascii_lowercase()) {
                return Err(ParseError {
                    line: self.dup_line_of_model(&model.name),
                    message: format!(
                        "Duplicate .model name: '{}' is defined more than once",
                        model.name
                    ),
                });
            }
        }

        // Check that devices referencing models have matching .model definitions
        // AND that the model's type matches the element kind (a diode bound to
        // an NPN card previously sailed through and produced a wrong circuit).
        for elem in &netlist.elements {
            if let Some(model_ref) = elem.model_name() {
                let model = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_ref));
                let Some(model) = model else {
                    // The deck usually DOES declare the model the author meant;
                    // a typo is far likelier than a missing card, and the
                    // candidates are right there to compare against.
                    let near = nearest_model_names(model_ref, &netlist.models);
                    let hint = if !near.is_empty() {
                        format!(" Did you mean: {}?", near.join(", "))
                    } else if netlist.models.is_empty() {
                        " This netlist declares no `.model` cards at all.".to_string()
                    } else {
                        let mut all: Vec<&str> =
                            netlist.models.iter().map(|m| m.name.as_str()).collect();
                        all.sort_unstable();
                        format!(" The models this deck declares: {}.", all.join(", "))
                    };
                    return Err(ParseError {
                        line: self.line_of_element(elem.name()),
                        message: format!(
                            "Component '{}' references model '{}' which is not defined.{}",
                            elem.name(),
                            model_ref,
                            hint
                        ),
                    });
                };
                // Allowed type sets follow the de facto contract in mna.rs /
                // codegen/ir (polarity is decided by starts_with("PNP"),
                // starts_with("PJ"), starts_with("PM"), so short forms like
                // NJ/PM are legal). `TUBE` is an accepted triode type
                // alongside TRIODE/VT.
                enum TypeRule {
                    Exact(&'static [&'static str]),
                    Prefix(&'static [&'static str]),
                }
                let expected: Option<(TypeRule, &str)> = match elem {
                    Element::Diode { .. } => Some((TypeRule::Exact(&["D"]), "diode")),
                    Element::Bjt { .. } => Some((TypeRule::Prefix(&["NPN", "PNP"]), "BJT")),
                    Element::Jfet { .. } => Some((TypeRule::Prefix(&["NJ", "PJ"]), "JFET")),
                    Element::Mosfet { .. } => Some((TypeRule::Prefix(&["NM", "PM"]), "MOSFET")),
                    Element::Triode { .. } => {
                        Some((TypeRule::Exact(&["TRIODE", "VT", "TUBE"]), "triode"))
                    }
                    Element::Pentode { .. } => {
                        Some((TypeRule::Exact(&["VP", "PENTODE"]), "pentode"))
                    }
                    Element::Opamp { .. } => Some((TypeRule::Exact(&["OA"]), "op-amp")),
                    Element::Vca { .. } => Some((TypeRule::Exact(&["VCA"]), "VCA")),
                    Element::Ldr { .. } => Some((TypeRule::Exact(&["LDR"]), "LDR")),
                    Element::Glow { .. } => Some((TypeRule::Exact(&["NEON"]), "NEON")),
                    _ => None,
                };
                if let Some((rule, kind)) = expected {
                    let mt = model.model_type.trim().to_ascii_uppercase();
                    let (ok, allowed_desc) = match rule {
                        TypeRule::Exact(set) => (set.iter().any(|a| *a == mt), set.join(", ")),
                        TypeRule::Prefix(set) => (
                            set.iter().any(|a| mt.starts_with(a)),
                            set.iter()
                                .map(|a| format!("{a}*"))
                                .collect::<Vec<_>>()
                                .join(", "),
                        ),
                    };
                    if !ok {
                        return Err(ParseError {
                            line: self.line_of_element(elem.name()),
                            message: format!(
                                "Component '{}' is a {} but references model '{}' of type '{}' \
                                 (expected one of: {})",
                                elem.name(),
                                kind,
                                model.name,
                                model.model_type,
                                allowed_desc
                            ),
                        });
                    }
                }
            }
        }

        // Verify all .pot directives reference existing resistors
        // (deferred so .pot can appear before the resistor in the netlist)
        // Names containing '.' are expanded subcircuit refs (e.g. "X1.R1") —
        // skip validation here; they'll be checked after expand_subcircuits().
        for pot in &netlist.pots {
            if pot.resistor_name.contains('.') {
                continue; // Will be validated after subcircuit expansion
            }
            let nominal = netlist.elements.iter().find_map(|e| match e {
                Element::Resistor { name, value, .. }
                    if name.eq_ignore_ascii_case(&pot.resistor_name) =>
                {
                    Some(*value)
                }
                _ => None,
            });
            let Some(nominal) = nominal else {
                return Err(ParseError {
                    line: self.line_of_directive(&[".pot", ".wiper"], &pot.resistor_name),
                    message: format!(
                        ".pot references resistor '{}' which was not found in the netlist",
                        pot.resistor_name
                    ),
                });
            };
            // With no explicit default the knob starts at the resistor's own
            // value, so that value has to be one the knob can reach — the same
            // rule an explicit default and `--pot` are held to.
            if pot.default_value.is_none() && (nominal < pot.min_value || nominal > pot.max_value) {
                return Err(ParseError {
                    line: self.line_of_directive(&[".pot"], &pot.resistor_name),
                    message: format!(
                        ".pot {name}: the resistor's netlist value ({nominal}) is outside the \
                         pot's range {min}..{max}, and with no explicit default the knob would \
                         start at a setting it cannot reach. Put {name}'s value inside the \
                         range, or give a default: `.pot {name} {min} {max} <default> \"Label\"`.",
                        name = pot.resistor_name,
                        min = pot.min_value,
                        max = pot.max_value,
                    ),
                });
            }
        }

        // Verify all .wiper directives: both resistors must share exactly one node
        for wiper in &netlist.wipers {
            if wiper.resistor_cw.contains('.') || wiper.resistor_ccw.contains('.') {
                continue; // Will be validated after subcircuit expansion
            }
            let find_nodes = |name: &str| -> Option<(String, String)> {
                netlist.elements.iter().find_map(|e| {
                    if let Element::Resistor {
                        name: rn,
                        n_plus,
                        n_minus,
                        ..
                    } = e
                    {
                        if rn.eq_ignore_ascii_case(name) {
                            Some((n_plus.clone(), n_minus.clone()))
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                })
            };
            if let (Some((cw_a, cw_b)), Some((ccw_a, ccw_b))) = (
                find_nodes(&wiper.resistor_cw),
                find_nodes(&wiper.resistor_ccw),
            ) {
                let cw_nodes = [cw_a.as_str(), cw_b.as_str()];
                let ccw_nodes = [ccw_a.as_str(), ccw_b.as_str()];
                let shared: Vec<&&str> = cw_nodes
                    .iter()
                    .filter(|n| ccw_nodes.iter().any(|m| m.eq_ignore_ascii_case(n)))
                    .collect();
                if shared.is_empty() {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".wiper"], &wiper.resistor_cw),
                        message: format!(
                            ".wiper resistors '{}' and '{}' do not share a node (wiper terminal)",
                            wiper.resistor_cw, wiper.resistor_ccw
                        ),
                    });
                }
            }
            // If resistors not found, the pot validation above already caught it
        }

        // Verify all .switch directives reference existing components
        // Names containing '.' are expanded subcircuit refs — skip here.
        for sw in &netlist.switches {
            for comp_name in &sw.component_names {
                if comp_name.contains('.') {
                    continue; // Will be validated after subcircuit expansion
                }
                let first_char = comp_name.chars().next().unwrap_or(' ').to_ascii_uppercase();
                let exists = netlist.elements.iter().any(|e| match (first_char, e) {
                    ('R', Element::Resistor { name, .. }) => name.eq_ignore_ascii_case(comp_name),
                    ('C', Element::Capacitor { name, .. }) => name.eq_ignore_ascii_case(comp_name),
                    ('L', Element::Inductor { name, .. }) => name.eq_ignore_ascii_case(comp_name),
                    _ => false,
                });
                if !exists {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".switch"], comp_name),
                        message: format!(
                            ".switch references component '{}' which was not found in the netlist",
                            comp_name
                        ),
                    });
                }
            }
        }

        // A component may not be claimed by BOTH a .pot/.wiper and a .switch —
        // the two runtime-update mechanisms would fight over the same stamp.
        // (pot↔wiper and pot↔runtime-R are already cross-validated elsewhere;
        // this closes the pot↔switch gap. Wiper legs are covered because
        // expand_wipers() has already pushed them into `netlist.pots`.)
        for pot in &netlist.pots {
            let claimed_by_switch = netlist.switches.iter().any(|sw| {
                sw.component_names
                    .iter()
                    .any(|n| n.eq_ignore_ascii_case(&pot.resistor_name))
            });
            if claimed_by_switch {
                return Err(ParseError {
                    line: self.line_of_element(&pot.resistor_name),
                    message: format!(
                        "Component '{}' is claimed by both a .pot/.wiper and a .switch directive — \
                         a component can only have one runtime-update mechanism",
                        pot.resistor_name
                    ),
                });
            }
        }

        // Verify all .runtime directives reference existing voltage sources,
        // each VS is only bound once, and no two directives claim the same
        // field name. Duplicate VS or field names would produce generated
        // code that fails to compile (two `pub field: f64` declarations), so
        // rejecting here gives a better error than waiting for rustc.
        //
        // .runtime R (resistor) field names share the namespace with
        // .runtime V so a circuit cannot bind both `V1 as foo` and `R1 as foo`.
        {
            let mut seen_sources = std::collections::HashSet::new();
            let mut seen_fields = std::collections::HashSet::new();
            for rt in &netlist.runtime_sources {
                let exists = netlist.elements.iter().any(|e| {
                    matches!(e, Element::VoltageSource { name, .. }
                             if name.eq_ignore_ascii_case(&rt.vs_name))
                });
                if !exists {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rt.vs_name),
                        message: format!(
                            ".runtime references voltage source '{}' which was not found in the netlist",
                            rt.vs_name
                        ),
                    });
                }
                let vs_key = rt.vs_name.to_ascii_uppercase();
                if !seen_sources.insert(vs_key) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rt.vs_name),
                        message: format!(
                            ".runtime declares voltage source '{}' more than once",
                            rt.vs_name
                        ),
                    });
                }
                if !seen_fields.insert(rt.field_name.clone()) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rt.field_name),
                        message: format!(
                            ".runtime declares field name '{}' more than once",
                            rt.field_name
                        ),
                    });
                }
            }

            // Same checks for .runtime R: resistor exists, unique resistor,
            // unique field name (shared namespace with runtime_sources), and
            // the resistor is not already claimed by a .pot or .wiper.
            let mut seen_resistors = std::collections::HashSet::new();
            let pot_claimed: std::collections::HashSet<String> = netlist
                .pots
                .iter()
                .map(|p| p.resistor_name.to_ascii_uppercase())
                .collect();
            let wiper_claimed: std::collections::HashSet<String> = netlist
                .wipers
                .iter()
                .flat_map(|w| {
                    [
                        w.resistor_cw.to_ascii_uppercase(),
                        w.resistor_ccw.to_ascii_uppercase(),
                    ]
                })
                .collect();
            let switch_claimed: std::collections::HashSet<String> = netlist
                .switches
                .iter()
                .flat_map(|sw| sw.component_names.iter().map(|n| n.to_ascii_uppercase()))
                .collect();
            for rr in &netlist.runtime_resistors {
                let exists = netlist.elements.iter().any(|e| {
                    matches!(e, Element::Resistor { name, .. }
                             if name.eq_ignore_ascii_case(&rr.resistor_name))
                });
                if !exists {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rr.resistor_name),
                        message: format!(
                            ".runtime R references resistor '{}' which was not found in the netlist",
                            rr.resistor_name
                        ),
                    });
                }
                let rkey = rr.resistor_name.to_ascii_uppercase();
                if pot_claimed.contains(&rkey) || wiper_claimed.contains(&rkey) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rr.resistor_name),
                        message: format!(
                            ".runtime R resistor '{}' is already claimed by a .pot or .wiper directive",
                            rr.resistor_name
                        ),
                    });
                }
                if switch_claimed.contains(&rkey) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rr.resistor_name),
                        message: format!(
                            ".runtime R resistor '{}' is already claimed by a .switch directive — \
                             a component can only have one runtime-update mechanism (both would \
                             stamp the same conductance)",
                            rr.resistor_name
                        ),
                    });
                }
                if !seen_resistors.insert(rkey) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rr.resistor_name),
                        message: format!(
                            ".runtime R declares resistor '{}' more than once",
                            rr.resistor_name
                        ),
                    });
                }
                if !seen_fields.insert(rr.field_name.clone()) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".runtime"], &rr.field_name),
                        message: format!(
                            ".runtime declares field name '{}' more than once",
                            rr.field_name
                        ),
                    });
                }
            }
        }

        // Verify `.inject` field names are unique (they name INJECT_NAMES
        // entries and give each injection a stable order), and `.tap` names
        // are unique. Node existence is validated later against `node_map` at
        // MNA/CLI resolution. Impedance-mandatory is enforced at parse time.
        {
            let mut seen_inject_fields = std::collections::HashSet::new();
            for inj in &netlist.injections {
                if !seen_inject_fields.insert(inj.field_name.clone()) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".inject"], &inj.field_name),
                        message: format!(
                            ".inject declares field name '{}' more than once",
                            inj.field_name
                        ),
                    });
                }
            }
            let mut seen_tap_names = std::collections::HashSet::new();
            for tap in &netlist.taps {
                if !seen_tap_names.insert(tap.name.clone()) {
                    return Err(ParseError {
                        line: self.line_of_directive(&[".tap"], &tap.name),
                        message: format!(".tap declares name '{}' more than once", tap.name),
                    });
                }
            }
        }

        // Verify all coupling (K) directives: no duplicate names, reference existing inductors.
        // An inductor MAY appear in multiple K directives (multi-winding transformers).
        {
            let mut seen_coupling_names = std::collections::HashSet::new();
            let mut seen_pairs = std::collections::HashSet::new();
            for coupling in &netlist.couplings {
                if !seen_coupling_names.insert(coupling.name.to_ascii_lowercase()) {
                    return Err(ParseError {
                        line: self.dup_line_of_element(&coupling.name),
                        message: format!("Duplicate coupling name: '{}'", coupling.name),
                    });
                }
                let l1_exists = netlist.elements.iter().any(|e| {
                    matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(&coupling.inductor1_name))
                });
                if !l1_exists {
                    return Err(ParseError {
                        line: self.line_of_element(&coupling.name),
                        message: format!(
                            "Coupling '{}' references inductor '{}' which was not found in the netlist",
                            coupling.name, coupling.inductor1_name
                        ),
                    });
                }
                let l2_exists = netlist.elements.iter().any(|e| {
                    matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(&coupling.inductor2_name))
                });
                if !l2_exists {
                    return Err(ParseError {
                        line: self.line_of_element(&coupling.name),
                        message: format!(
                            "Coupling '{}' references inductor '{}' which was not found in the netlist",
                            coupling.name, coupling.inductor2_name
                        ),
                    });
                }
                // Reject duplicate pairs (same two inductors coupled twice)
                let l1_lower = coupling.inductor1_name.to_ascii_lowercase();
                let l2_lower = coupling.inductor2_name.to_ascii_lowercase();
                let pair = if l1_lower < l2_lower {
                    (l1_lower, l2_lower)
                } else {
                    (l2_lower, l1_lower)
                };
                if !seen_pairs.insert(pair) {
                    return Err(ParseError {
                        line: self.line_of_element(&coupling.name),
                        message: format!(
                            "Inductors '{}' and '{}' are coupled by multiple K directives",
                            coupling.inductor1_name, coupling.inductor2_name
                        ),
                    });
                }
            }
        }

        // Verify subcircuit instances reference defined subcircuits with matching port count
        for elem in &netlist.elements {
            if let Element::SubcktInstance {
                name,
                nodes,
                subckt,
            } = elem
            {
                let sc = netlist
                    .subcircuits
                    .iter()
                    .find(|s| s.name.eq_ignore_ascii_case(subckt));
                match sc {
                    None => {
                        return Err(ParseError {
                            line: self.line_of_element(name),
                            message: format!(
                                "Subcircuit instance '{}' references undefined subcircuit '{}'",
                                name, subckt
                            ),
                        });
                    }
                    Some(s) if nodes.len() != s.nodes.len() => {
                        return Err(ParseError {
                            line: self.line_of_element(name),
                            message: format!(
                                "Subcircuit instance '{}' has {} nodes but '{}' expects {}",
                                name,
                                nodes.len(),
                                subckt,
                                s.nodes.len()
                            ),
                        });
                    }
                    _ => {}
                }
            }
        }

        // Validate .model parameter ranges for device models.
        // Catches obviously wrong values early instead of at solver runtime.
        for model in &netlist.models {
            self.validate_model_params(model)?;
        }

        // Unrecognized-parameter check for `.model` cards that no element
        // references.
        //
        // A *referenced* card is checked where its device is resolved: the
        // codegen resolvers hard-error on an unknown key (naming the accepted
        // set), and op-amps/VCAs warn from the resolution loops in `mna.rs`. An
        // **unreferenced** card reaches neither, so `.model 2N3904 NPN(ZORP=5)`
        // sitting in a deck — a shared model library, a part swapped out
        // mid-edit, a card whose device got commented out — was accepted in
        // total silence while the identical typo one line down was a hard
        // error. That inconsistency is worse than no check at all: seeing one
        // card report teaches the author to trust the silence on the others.
        //
        // Warn, do not error: an unused card cannot affect the simulation, and
        // a shared `.model` library legitimately carries parts this deck does
        // not use. The accepted key set comes from `model_params`, so this pass
        // cannot drift from the resolvers'.
        {
            let mut referenced: std::collections::HashSet<String> = netlist
                .elements
                .iter()
                .filter_map(|e| e.model_name())
                .map(|m| m.to_ascii_lowercase())
                .collect();
            // Subcircuit bodies are not expanded yet at validation time; a card
            // used only inside a `.subckt` is referenced, not orphaned.
            for sc in &netlist.subcircuits {
                referenced.extend(
                    sc.elements
                        .iter()
                        .filter_map(|e| e.model_name())
                        .map(|m| m.to_ascii_lowercase()),
                );
            }
            for model in &netlist.models {
                if referenced.contains(&model.name.to_ascii_lowercase()) {
                    continue;
                }
                // No table for this model type — stay silent rather than guess.
                // A deck may legitimately carry cards for device types melange
                // does not model at all.
                let Some(class) =
                    crate::model_params::ModelClass::from_model_type(&model.model_type)
                else {
                    continue;
                };
                for (key, _) in &model.params {
                    crate::model_params::warn_if_unknown(&model.name, class, key);
                }
            }
        }

        // Validate .gang directives: each member must exist in .pot or .wiper,
        // no member may appear in multiple gangs.
        {
            let mut gang_claimed: std::collections::HashSet<String> =
                std::collections::HashSet::new();
            for gang in &netlist.gangs {
                for member in &gang.members {
                    let name_upper = member.resistor_name.to_ascii_uppercase();

                    // Check if this member is already claimed by another gang
                    if !gang_claimed.insert(name_upper.clone()) {
                        return Err(ParseError {
                            line: self.line_of_directive(&[".gang"], &member.resistor_name),
                            message: format!(
                                ".gang: resistor '{}' appears in multiple .gang directives",
                                member.resistor_name
                            ),
                        });
                    }

                    // Check if this member exists in a .pot or .wiper directive
                    let in_pot = netlist
                        .pots
                        .iter()
                        .any(|p| p.resistor_name.eq_ignore_ascii_case(&name_upper));
                    let in_wiper = netlist.wipers.iter().any(|w| {
                        w.resistor_cw.eq_ignore_ascii_case(&name_upper)
                            || w.resistor_ccw.eq_ignore_ascii_case(&name_upper)
                    });
                    let in_runtime_r = netlist
                        .runtime_resistors
                        .iter()
                        .any(|r| r.resistor_name.eq_ignore_ascii_case(&name_upper));
                    if in_runtime_r {
                        // `.gang` is a UI construct — one nih-plug FloatParam
                        // drives every member in lockstep. `.runtime R` is
                        // explicitly NOT a knob; the plugin drives the setter
                        // directly from an envelope/LFO. The two surfaces do
                        // not compose. Drive multiple runtime-R fields from
                        // the same plugin-side envelope instead.
                        return Err(ParseError {
                            line: self.line_of_directive(&[".gang"], &member.resistor_name),
                            message: format!(
                                ".gang member '{}' is a .runtime R target — .gang only accepts .pot and .wiper members. \
                                 Drive multiple .runtime R fields from the same plugin-side envelope/LFO by calling \
                                 each set_runtime_R_<field> from one follower tick.",
                                member.resistor_name
                            ),
                        });
                    }
                    if !in_pot && !in_wiper {
                        return Err(ParseError {
                            line: self.line_of_directive(&[".gang"], &member.resistor_name),
                            message: format!(
                                ".gang: member '{}' not found in any .pot or .wiper directive",
                                member.resistor_name
                            ),
                        });
                    }
                }
            }
        }

        Ok(())
    }

    /// Validate that .model parameters are physically reasonable.
    fn validate_model_params(&self, model: &Model) -> Result<(), ParseError> {
        for (key, value) in &model.params {
            let key_upper = key.to_ascii_uppercase();

            // Check for NaN/Inf in any parameter
            if !value.is_finite() {
                return Err(ParseError {
                    line: self.line_of_model(&model.name),
                    message: format!(
                        ".model '{}': parameter {}={} is not finite",
                        model.name, key, value
                    ),
                });
            }

            // A JFET's IS is its gate junctions' saturation current: 0 disables
            // them (the model without gate junctions), so only negative is
            // unphysical there.
            let jfet = crate::model_params::ModelClass::from_model_type(&model.model_type)
                == Some(crate::model_params::ModelClass::Jfet);
            // Parameters that must be strictly positive
            match key_upper.as_str() {
                "IS" if jfet => {
                    if *value < 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be >= 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                "IS" | "IDSS" | "G0" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // B-E / B-C leakage saturation currents: 0 disables the leakage
                // term (the SPICE default, and melange's own default), so it is
                // valid — only negative is unphysical.
                "ISE" | "ISC" => {
                    if *value < 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be >= 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Forward/reverse gain must be positive
                "BF" | "BR" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Emission coefficients must be positive
                "N" | "NF" | "NR" | "NE" | "NC" | "EX" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Resistances must be non-negative
                "RS" | "RB" | "RC" | "RE" | "RD" | "RGI" | "ROUT" => {
                    if *value < 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be >= 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Capacitances must be non-negative
                "CJE" | "CJC" | "CGS" | "CGD" | "CJO" | "CCG" | "CGP" | "CCP" => {
                    if *value < 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be >= 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // KP (transconductance) must be positive
                "KP" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Early voltages, knee currents must be positive when specified
                "VAF" | "VAR" | "IKF" | "IKR" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Tube parameters: MU, KG1, KVB must be positive (KP handled above)
                "MU" | "KG1" | "KVB" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // AOL (open-loop gain) must be positive
                "AOL" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Channel-length modulation (JFET, MOSFET, tube) must be non-negative
                "LAMBDA" => {
                    if *value < 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be >= 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // VCA THD coefficient must be non-negative
                "THD" => {
                    if *value < 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be >= 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // VCA gain-law scale voltage (denominator in exp) must be positive
                "VSCALE" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Op-amp saturation voltage must be positive
                "VSAT" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Op-amp gain-bandwidth product must be positive
                "GBW" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Op-amp slew rate must be positive (V/μs in .model card,
                // converted to V/s internally during MNA stamping).
                "SR" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Op-amp supply rails: finite check already done above;
                // no additional single-param range constraint needed
                "VCC" | "VEE" => {}
                // Op-amp input bias current (IB) is signed — positive for
                // JFET/PNP-input parts, negative for NPN-input. No range check.
                "IB" => {}
                // Op-amp input resistance must be positive. Values larger than
                // 1 PΩ are accepted but effectively mean "infinite" (the shunt
                // conductance 1/RIN rounds to 0 in any realistic MNA context).
                "RIN" => {
                    if *value <= 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be > 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Flicker (1/f) noise coefficient must be non-negative.
                // Zero (default) disables per-device flicker generation —
                // the NoiseIR collector skips the device when KF <= 0.
                "KF" => {
                    if *value < 0.0 {
                        return Err(ParseError {
                            line: self.line_of_model(&model.name),
                            message: format!(
                                ".model '{}': {} must be >= 0, got {}",
                                model.name, key, value
                            ),
                        });
                    }
                }
                // Flicker exponent. ngspice default is 1.0; must be positive
                // so `|I|^AF` is well-defined for nonzero currents.
                "AF" if *value <= 0.0 => {
                    return Err(ParseError {
                        line: self.line_of_model(&model.name),
                        message: format!(
                            ".model '{}': {} must be > 0, got {}",
                            model.name, key, value
                        ),
                    });
                }
                _ => {} // Unknown params: no range check (warned elsewhere)
            }
        }
        Ok(())
    }
}
