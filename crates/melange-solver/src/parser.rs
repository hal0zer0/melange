//! SPICE netlist parser.
//!
//! Parses a subset of SPICE sufficient for audio circuits:
//! - Components: R, C, L, V (DC/AC), I, D, Q, J, M, U (op-amp), E (VCVS), G (VCCS), X
//! - Directives: .model, .subckt, .param, .pot, .switch, .input_impedance, .end
//!
//! The parser builds an AST (Abstract Syntax Tree) representation
//! of the circuit that can be processed by the MNA assembler.

/// A parsed SPICE netlist.
#[derive(Debug, Clone, PartialEq)]
pub struct Netlist {
    /// Circuit title (first line)
    pub title: String,
    /// Circuit elements
    pub elements: Vec<Element>,
    /// Model definitions
    pub models: Vec<Model>,
    /// Subcircuit definitions
    pub subcircuits: Vec<Subcircuit>,
    /// Global parameters
    pub params: Vec<Parameter>,
    /// Potentiometer directives (.pot)
    pub pots: Vec<PotDirective>,
    /// Switch directives (.switch)
    pub switches: Vec<SwitchDirective>,
    /// Coupling directives (K elements for coupled inductors / transformers)
    pub couplings: Vec<CouplingDirective>,
    /// Delay feedback node names (.delay_feedback node1 node2 ...)
    /// These nodes are frozen at their previous-sample values during NR iteration,
    /// breaking feedback loops through transformers.
    pub delay_feedback_nodes: Vec<String>,
    /// Input impedance directive (.input_impedance)
    pub input_impedance: Option<f64>,
}

/// A potentiometer directive (.pot Rname min max).
///
/// Marks a resistor as runtime-variable with a min/max range.
/// The resistor's existing value in the netlist becomes the nominal
/// value for Sherman-Morrison precomputation.
#[derive(Debug, Clone, PartialEq)]
pub struct PotDirective {
    /// Name of the resistor this pot controls (e.g. "R1")
    pub resistor_name: String,
    /// Minimum resistance value (ohms)
    pub min_value: f64,
    /// Maximum resistance value (ohms)
    pub max_value: f64,
    /// Default resistance for plugin parameter (ohms). If None, uses netlist nominal.
    pub default_value: Option<f64>,
    /// Optional human-readable label (e.g. "Volume")
    pub label: Option<String>,
}

/// A switch directive (.switch C1,L1 val0a/val0b val1a/val1b ...).
///
/// Defines a rotary switch that selects among discrete component values.
/// Multiple components can be ganged (changed simultaneously) by listing
/// names separated by commas, with position values separated by slashes.
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchDirective {
    /// Component names controlled by this switch (e.g. ["C_hfb", "L_hfb"])
    pub component_names: Vec<String>,
    /// Position values: positions[pos][comp] = value for that component at that position
    pub positions: Vec<Vec<f64>>,
    /// Optional human-readable label (e.g. "Bright")
    pub label: Option<String>,
}

/// A coupling directive (K element) for coupled inductors / transformers.
///
/// Standard SPICE syntax: `K1 L1 L2 0.95`
/// Couples two inductors with mutual inductance M = k * sqrt(L1 * L2).
#[derive(Debug, Clone, PartialEq)]
pub struct CouplingDirective {
    /// Name of the coupling element (e.g. "K1")
    pub name: String,
    /// Name of the first inductor (e.g. "L1")
    pub inductor1_name: String,
    /// Name of the second inductor (e.g. "L2")
    pub inductor2_name: String,
    /// Coupling coefficient k (0 < k < 1)
    pub coupling: f64,
}

impl Netlist {
    /// Create an empty netlist.
    pub fn new(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            elements: Vec::new(),
            models: Vec::new(),
            subcircuits: Vec::new(),
            params: Vec::new(),
            pots: Vec::new(),
            switches: Vec::new(),
            couplings: Vec::new(),
            delay_feedback_nodes: Vec::new(),
            input_impedance: None,
        }
    }

    /// Parse a netlist from a string.
    pub fn parse(input: &str) -> Result<Self, ParseError> {
        Parser::new(input).parse()
    }

    /// Expand all subcircuit instances (`X` elements) into their constituent elements.
    ///
    /// Each `SubcktInstance` is replaced by the elements from its subcircuit definition,
    /// with component names prefixed (`X1.R1`) and internal nodes remapped (`X1.mid`).
    /// Port nodes are mapped to the caller's actual connection nodes. Ground ("0") is
    /// always global and never prefixed.
    ///
    /// This must be called between `parse()` and `MnaSystem::from_netlist()`.
    /// Nested subcircuits are handled by iterative expansion (max 32 passes).
    ///
    /// # Errors
    /// - Undefined subcircuit reference
    /// - Port count mismatch
    /// - Recursive subcircuit cycle
    /// - Duplicate subcircuit names
    pub fn expand_subcircuits(&mut self) -> Result<(), ParseError> {
        if self.subcircuits.is_empty() {
            return Ok(());
        }

        // Build lookup: lowercase name → index into self.subcircuits
        let mut lookup: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
        for (i, sc) in self.subcircuits.iter().enumerate() {
            let key = sc.name.to_ascii_lowercase();
            if lookup.contains_key(&key) {
                return Err(ParseError {
                    line: 0,
                    message: format!("Duplicate subcircuit definition: '{}'", sc.name),
                });
            }
            lookup.insert(key, i);
        }

        // Cycle detection
        detect_subcircuit_cycles(&self.subcircuits, &lookup)?;

        // Iterative expansion (max 32 passes for deeply nested subcircuits)
        for _pass in 0..32 {
            let has_instances = self.elements.iter().any(|e| matches!(e, Element::SubcktInstance { .. }));
            if !has_instances {
                break;
            }

            let mut new_elements = Vec::with_capacity(self.elements.len());
            for elem in &self.elements {
                if let Element::SubcktInstance { name: inst_name, nodes: inst_nodes, subckt } = elem {
                    let key = subckt.to_ascii_lowercase();
                    let sc_idx = lookup.get(&key).ok_or_else(|| ParseError {
                        line: 0,
                        message: format!(
                            "Subcircuit instance '{}' references undefined subcircuit '{}'",
                            inst_name, subckt
                        ),
                    })?;
                    let sc = &self.subcircuits[*sc_idx];

                    // Validate port count
                    if inst_nodes.len() != sc.nodes.len() {
                        return Err(ParseError {
                            line: 0,
                            message: format!(
                                "Subcircuit instance '{}' has {} nodes but '{}' expects {}",
                                inst_name, inst_nodes.len(), subckt, sc.nodes.len()
                            ),
                        });
                    }

                    // Build node map: subckt port name → caller's actual node
                    let mut node_map: std::collections::HashMap<String, String> =
                        std::collections::HashMap::new();
                    for (port, actual) in sc.nodes.iter().zip(inst_nodes.iter()) {
                        node_map.insert(port.to_ascii_lowercase(), actual.clone());
                    }

                    // Expand each element from the subcircuit definition
                    for sc_elem in &sc.elements {
                        new_elements.push(sc_elem.remap_for_subcircuit(inst_name, &node_map));
                    }
                } else {
                    new_elements.push(elem.clone());
                }
            }
            self.elements = new_elements;
        }

        // Verify no unexpanded instances remain
        for elem in &self.elements {
            if let Element::SubcktInstance { name, subckt, .. } = elem {
                return Err(ParseError {
                    line: 0,
                    message: format!(
                        "Subcircuit instance '{}' (of '{}') could not be fully expanded after 32 passes",
                        name, subckt
                    ),
                });
            }
        }

        // Clear subcircuit definitions — they've been inlined
        self.subcircuits.clear();
        Ok(())
    }
}

/// A circuit element (component).
#[derive(Debug, Clone, PartialEq)]
pub enum Element {
    /// Resistor: Rname n+ n- value
    Resistor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
    },
    /// Capacitor: Cname n+ n- value [IC=initial]
    Capacitor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
        ic: Option<f64>,
    },
    /// Inductor: Lname n+ n- value
    Inductor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
    },
    /// Voltage source: Vname n+ n- [DC value] [AC mag phase]
    VoltageSource {
        name: String,
        n_plus: String,
        n_minus: String,
        dc: Option<f64>,
        ac: Option<(f64, f64)>,
    },
    /// Current source: Iname n+ n- [DC value]
    CurrentSource {
        name: String,
        n_plus: String,
        n_minus: String,
        dc: Option<f64>,
    },
    /// Diode: Dname n+ n- modelname
    Diode {
        name: String,
        n_plus: String,
        n_minus: String,
        model: String,
    },
    /// BJT: Qname nc nb ne [ns] modelname
    Bjt {
        name: String,
        nc: String,
        nb: String,
        ne: String,
        model: String,
    },
    /// JFET: Jname nd ng ns modelname
    Jfet {
        name: String,
        nd: String,
        ng: String,
        ns: String,
        model: String,
    },
    /// MOSFET: Mname nd ng ns nb modelname
    Mosfet {
        name: String,
        nd: String,
        ng: String,
        ns: String,
        nb: String,
        model: String,
    },
    /// Op-amp: Uname n_plus n_minus n_out modelname
    ///
    /// Modeled as a high-gain VCCS -- does NOT add nonlinear dimensions.
    Opamp {
        name: String,
        /// Non-inverting input node
        n_plus: String,
        /// Inverting input node
        n_minus: String,
        /// Output node
        n_out: String,
        /// Model name (references .model with OA type)
        model: String,
    },
    /// Triode: Tname n_grid n_plate n_cathode modelname
    ///
    /// M=2 per triode: plate current (Ip) and grid current (Ig).
    Triode {
        name: String,
        /// Grid node
        n_grid: String,
        /// Plate (anode) node
        n_plate: String,
        /// Cathode node
        n_cathode: String,
        /// Model name (references .model with TUBE type)
        model: String,
    },
    /// Voltage-Controlled Voltage Source: Ename out+ out- ctrl+ ctrl- gain
    ///
    /// Modeled as Norton equivalent (VCCS + output conductance).
    /// Does NOT add nonlinear dimensions.
    Vcvs {
        name: String,
        /// Positive output node
        out_p: String,
        /// Negative output node
        out_n: String,
        /// Positive control node
        ctrl_p: String,
        /// Negative control node
        ctrl_n: String,
        /// Voltage gain
        gain: f64,
    },
    /// Voltage-Controlled Current Source: Gname out+ out- ctrl+ ctrl- gm
    ///
    /// Direct G matrix stamp. Does NOT add nonlinear dimensions.
    Vccs {
        name: String,
        /// Positive output node (current flows into this node)
        out_p: String,
        /// Negative output node (current flows out of this node)
        out_n: String,
        /// Positive control node
        ctrl_p: String,
        /// Negative control node
        ctrl_n: String,
        /// Transconductance (A/V)
        gm: f64,
    },
    /// Subcircuit instance: Xname nodes... subcktname
    SubcktInstance {
        name: String,
        nodes: Vec<String>,
        subckt: String,
    },
}

impl Element {
    /// Get the element name.
    pub fn name(&self) -> &str {
        match self {
            Element::Resistor { name, .. }
            | Element::Capacitor { name, .. }
            | Element::Inductor { name, .. }
            | Element::VoltageSource { name, .. }
            | Element::CurrentSource { name, .. }
            | Element::Diode { name, .. }
            | Element::Bjt { name, .. }
            | Element::Jfet { name, .. }
            | Element::Mosfet { name, .. }
            | Element::Opamp { name, .. }
            | Element::Triode { name, .. }
            | Element::Vcvs { name, .. }
            | Element::Vccs { name, .. }
            | Element::SubcktInstance { name, .. } => name,
        }
    }

    /// Get the referenced model name, if this is a device that requires a `.model` card.
    pub fn model_name(&self) -> Option<&str> {
        match self {
            Element::Diode { model, .. }
            | Element::Bjt { model, .. }
            | Element::Jfet { model, .. }
            | Element::Mosfet { model, .. }
            | Element::Opamp { model, .. }
            | Element::Triode { model, .. } => Some(model),
            _ => None,
        }
    }

    /// Clone this element with remapped nodes and prefixed name for subcircuit expansion.
    ///
    /// - Component name is prefixed: `{prefix}.{name}`
    /// - Nodes are remapped: port nodes → caller's nodes, internal → `{prefix}.{node}`, "0" → "0"
    /// - Model names are NOT remapped (models are global).
    fn remap_for_subcircuit(
        &self,
        prefix: &str,
        node_map: &std::collections::HashMap<String, String>,
    ) -> Element {
        let remap = |node: &str| -> String {
            if node == "0" {
                return "0".to_string();
            }
            let key = node.to_ascii_lowercase();
            if let Some(actual) = node_map.get(&key) {
                actual.clone()
            } else {
                format!("{}.{}", prefix, node)
            }
        };
        let prefixed = |name: &str| -> String {
            format!("{}.{}", prefix, name)
        };

        match self {
            Element::Resistor { name, n_plus, n_minus, value } => Element::Resistor {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                value: *value,
            },
            Element::Capacitor { name, n_plus, n_minus, value, ic } => Element::Capacitor {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                value: *value,
                ic: *ic,
            },
            Element::Inductor { name, n_plus, n_minus, value } => Element::Inductor {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                value: *value,
            },
            Element::VoltageSource { name, n_plus, n_minus, dc, ac } => Element::VoltageSource {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                dc: *dc,
                ac: *ac,
            },
            Element::CurrentSource { name, n_plus, n_minus, dc } => Element::CurrentSource {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                dc: *dc,
            },
            Element::Diode { name, n_plus, n_minus, model } => Element::Diode {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                model: model.clone(),
            },
            Element::Bjt { name, nc, nb, ne, model } => Element::Bjt {
                name: prefixed(name),
                nc: remap(nc),
                nb: remap(nb),
                ne: remap(ne),
                model: model.clone(),
            },
            Element::Jfet { name, nd, ng, ns, model } => Element::Jfet {
                name: prefixed(name),
                nd: remap(nd),
                ng: remap(ng),
                ns: remap(ns),
                model: model.clone(),
            },
            Element::Mosfet { name, nd, ng, ns, nb, model } => Element::Mosfet {
                name: prefixed(name),
                nd: remap(nd),
                ng: remap(ng),
                ns: remap(ns),
                nb: remap(nb),
                model: model.clone(),
            },
            Element::Opamp { name, n_plus, n_minus, n_out, model } => Element::Opamp {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                n_out: remap(n_out),
                model: model.clone(),
            },
            Element::Triode { name, n_grid, n_plate, n_cathode, model } => Element::Triode {
                name: prefixed(name),
                n_grid: remap(n_grid),
                n_plate: remap(n_plate),
                n_cathode: remap(n_cathode),
                model: model.clone(),
            },
            Element::Vcvs { name, out_p, out_n, ctrl_p, ctrl_n, gain } => Element::Vcvs {
                name: prefixed(name),
                out_p: remap(out_p),
                out_n: remap(out_n),
                ctrl_p: remap(ctrl_p),
                ctrl_n: remap(ctrl_n),
                gain: *gain,
            },
            Element::Vccs { name, out_p, out_n, ctrl_p, ctrl_n, gm } => Element::Vccs {
                name: prefixed(name),
                out_p: remap(out_p),
                out_n: remap(out_n),
                ctrl_p: remap(ctrl_p),
                ctrl_n: remap(ctrl_n),
                gm: *gm,
            },
            Element::SubcktInstance { name, nodes, subckt } => Element::SubcktInstance {
                name: prefixed(name),
                nodes: nodes.iter().map(|n| remap(n)).collect(),
                subckt: subckt.clone(),
            },
        }
    }
}

/// Detect cycles in subcircuit definitions (A contains instance of B which contains instance of A).
fn detect_subcircuit_cycles(
    subcircuits: &[Subcircuit],
    lookup: &std::collections::HashMap<String, usize>,
) -> Result<(), ParseError> {
    // Build adjacency list: subckt index → list of subckt indices it references
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); subcircuits.len()];
    for (i, sc) in subcircuits.iter().enumerate() {
        for elem in &sc.elements {
            if let Element::SubcktInstance { subckt, .. } = elem {
                let key = subckt.to_ascii_lowercase();
                if let Some(&j) = lookup.get(&key) {
                    adj[i].push(j);
                }
                // Missing references will be caught during expansion
            }
        }
    }

    // DFS cycle detection
    let n = subcircuits.len();
    let mut color = vec![0u8; n]; // 0=white, 1=gray (in stack), 2=black (done)
    let mut stack = Vec::new();

    for start in 0..n {
        if color[start] != 0 {
            continue;
        }
        stack.clear();
        stack.push((start, 0usize)); // (node, next_neighbor_index)
        color[start] = 1;

        while let Some((node, ni)) = stack.last_mut() {
            if *ni < adj[*node].len() {
                let neighbor = adj[*node][*ni];
                *ni += 1;
                if color[neighbor] == 1 {
                    // Found a cycle — build cycle path for error message
                    let mut cycle_names: Vec<String> = stack
                        .iter()
                        .skip_while(|(idx, _)| *idx != neighbor)
                        .map(|(idx, _)| subcircuits[*idx].name.clone())
                        .collect();
                    cycle_names.push(subcircuits[neighbor].name.clone());
                    return Err(ParseError {
                        line: 0,
                        message: format!(
                            "Recursive subcircuit cycle detected: {}",
                            cycle_names.join(" -> ")
                        ),
                    });
                } else if color[neighbor] == 0 {
                    color[neighbor] = 1;
                    stack.push((neighbor, 0));
                }
            } else {
                color[*node] = 2;
                stack.pop();
            }
        }
    }

    Ok(())
}

/// A model definition (.model).
#[derive(Debug, Clone, PartialEq)]
pub struct Model {
    pub name: String,
    pub model_type: String,
    pub params: Vec<(String, f64)>,
}

/// A subcircuit definition (.subckt).
#[derive(Debug, Clone, PartialEq)]
pub struct Subcircuit {
    pub name: String,
    pub nodes: Vec<String>,
    pub elements: Vec<Element>,
}

/// A parameter definition (.param).
#[derive(Debug, Clone, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub value: f64,
}

/// Parse error.
#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Parse error at line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for ParseError {}

/// SPICE netlist parser.
struct Parser {
    /// Pre-processed lines (after continuation joining and comment stripping)
    processed_lines: Vec<String>,
    /// Current position in processed_lines
    pos: usize,
    /// Line number tracking for error messages
    line_num: usize,
}

impl Parser {
    fn new(input: &str) -> Self {
        // Pre-process: join continuation lines and strip comments
        let raw_lines: Vec<&str> = input.lines().collect();
        let mut processed = Vec::new();
        let mut i = 0;

        while i < raw_lines.len() {
            // Strip inline comments (semicolon and $ delimiter)
            let line = raw_lines[i]
                .split([';', '$'])
                .next()
                .unwrap_or(raw_lines[i])
                .trim();

            // Accumulate continuation lines: next line starts with '+'
            let mut accumulated = line.to_string();
            while i + 1 < raw_lines.len() {
                let next = raw_lines[i + 1]
                    .split([';', '$'])
                    .next()
                    .unwrap_or(raw_lines[i + 1])
                    .trim();
                if let Some(stripped) = next.strip_prefix('+') {
                    // Continuation line: strip '+' and append
                    accumulated.push(' ');
                    accumulated.push_str(stripped.trim());
                    i += 1;
                } else {
                    break;
                }
            }

            processed.push(accumulated);
            i += 1;
        }

        Self {
            processed_lines: processed,
            pos: 0,
            line_num: 0,
        }
    }

    fn parse(mut self) -> Result<Netlist, ParseError> {
        // First line is title
        let title = self.next_line().unwrap_or_default();
        let mut netlist = Netlist::new(title);

        while let Some(line) = self.next_line() {
            let line = line.trim().to_string();

            // Skip empty lines and comments
            if line.is_empty() || line.starts_with('*') {
                continue;
            }

            // Parse directive or element
            if line.starts_with('.') {
                self.parse_directive(&line, &mut netlist)?;
            } else if line.starts_with('K') || line.starts_with('k') {
                if netlist.couplings.len() >= 16 {
                    return Err(self.error("Maximum of 16 coupling (K) directives supported"));
                }
                let coupling = self.parse_coupling(&line)?;
                netlist.couplings.push(coupling);
            } else {
                let element = self.parse_element(&line)?;
                netlist.elements.push(element);
            }
        }

        Self::validate_netlist(&netlist)?;
        Ok(netlist)
    }

    /// Post-parse validation: duplicate names, model references, pot targets.
    fn validate_netlist(netlist: &Netlist) -> Result<(), ParseError> {
        // Check for duplicate component names (case-insensitive)
        let mut seen_names = std::collections::HashSet::new();
        for elem in &netlist.elements {
            if !seen_names.insert(elem.name().to_ascii_lowercase()) {
                return Err(ParseError {
                    line: 0,
                    message: format!("Duplicate component name: '{}'", elem.name()),
                });
            }
        }

        // Check that devices referencing models have matching .model definitions
        for elem in &netlist.elements {
            if let Some(model_ref) = elem.model_name() {
                let model_exists = netlist.models.iter().any(|m| m.name.eq_ignore_ascii_case(model_ref));
                if !model_exists {
                    return Err(ParseError {
                        line: 0,
                        message: format!(
                            "Component '{}' references model '{}' which is not defined",
                            elem.name(), model_ref
                        ),
                    });
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
            let exists = netlist.elements.iter().any(|e| {
                matches!(e, Element::Resistor { name, .. } if name.eq_ignore_ascii_case(&pot.resistor_name))
            });
            if !exists {
                return Err(ParseError {
                    line: 0,
                    message: format!(
                        ".pot references resistor '{}' which was not found in the netlist",
                        pot.resistor_name
                    ),
                });
            }
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
                        line: 0,
                        message: format!(
                            ".switch references component '{}' which was not found in the netlist",
                            comp_name
                        ),
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
                        line: 0,
                        message: format!(
                            "Duplicate coupling name: '{}'",
                            coupling.name
                        ),
                    });
                }
                let l1_exists = netlist.elements.iter().any(|e| {
                    matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(&coupling.inductor1_name))
                });
                if !l1_exists {
                    return Err(ParseError {
                        line: 0,
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
                        line: 0,
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
                        line: 0,
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
            if let Element::SubcktInstance { name, nodes, subckt } = elem {
                let sc = netlist.subcircuits.iter().find(|s| s.name.eq_ignore_ascii_case(subckt));
                match sc {
                    None => {
                        return Err(ParseError {
                            line: 0,
                            message: format!(
                                "Subcircuit instance '{}' references undefined subcircuit '{}'",
                                name, subckt
                            ),
                        });
                    }
                    Some(s) if nodes.len() != s.nodes.len() => {
                        return Err(ParseError {
                            line: 0,
                            message: format!(
                                "Subcircuit instance '{}' has {} nodes but '{}' expects {}",
                                name, nodes.len(), subckt, s.nodes.len()
                            ),
                        });
                    }
                    _ => {}
                }
            }
        }

        Ok(())
    }

    fn next_line(&mut self) -> Option<String> {
        if self.pos < self.processed_lines.len() {
            let line = self.processed_lines[self.pos].clone();
            self.pos += 1;
            self.line_num += 1;
            Some(line)
        } else {
            None
        }
    }

    fn error(&self, message: impl Into<String>) -> ParseError {
        ParseError {
            line: self.line_num,
            message: message.into(),
        }
    }

    fn parse_directive(&mut self, line: &str, netlist: &mut Netlist) -> Result<(), ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return Ok(());
        }

        match parts[0].to_lowercase().as_str() {
            ".model" => {
                self.require_parts(&parts, 3, "name and type")?;
                let model = self.parse_model(&parts)?;
                netlist.models.push(model);
            }
            ".param" => {
                let param = self.parse_param(&parts)?;
                netlist.params.push(param);
            }
            ".subckt" => {
                self.require_parts(&parts, 2, "a name")?;
                let subckt_name = parts[1].to_string();
                let subckt_nodes: Vec<String> = parts[2..].iter().map(|s| s.to_string()).collect();
                let mut subckt_elements = Vec::new();

                // Collect elements until .ends
                let mut found_ends = false;
                while let Some(sub_line) = self.next_line() {
                    let sub_line = sub_line.trim().to_string();
                    if sub_line.is_empty() || sub_line.starts_with('*') {
                        continue;
                    }
                    if sub_line.to_lowercase().starts_with(".ends") {
                        found_ends = true;
                        break;
                    }
                    if sub_line.starts_with('.') {
                        // Nested directives inside subckt (e.g. .model)
                        let sub_parts: Vec<&str> = sub_line.split_whitespace().collect();
                        if sub_parts[0].to_lowercase() == ".model" {
                            let model = self.parse_model(&sub_parts)?;
                            netlist.models.push(model);
                        }
                        continue;
                    }
                    let elem = self.parse_element(&sub_line)?;
                    subckt_elements.push(elem);
                }

                if !found_ends {
                    return Err(self.error(format!(
                        "Subcircuit '{}' missing .ends directive", subckt_name
                    )));
                }

                netlist.subcircuits.push(Subcircuit {
                    name: subckt_name,
                    nodes: subckt_nodes,
                    elements: subckt_elements,
                });
            }
            ".pot" => {
                let pot = self.parse_pot_directive(&parts, netlist)?;
                netlist.pots.push(pot);
            }
            ".switch" => {
                let sw = self.parse_switch_directive(&parts, netlist)?;
                netlist.switches.push(sw);
            }
            ".delay_feedback" => {
                // .delay_feedback node1 node2 ... — node names to freeze during NR
                if parts.len() < 2 {
                    return Err(self.error(".delay_feedback requires at least one node name"));
                }
                for name in &parts[1..] {
                    netlist.delay_feedback_nodes.push(name.to_string());
                }
            }
            ".input_impedance" => {
                self.parse_input_impedance_directive(&parts, netlist)?;
            }
            ".end" | ".ends" => {
                // End of netlist or subcircuit
            }
            other => {
                log::warn!("Unknown directive: {}", other);
            }
        }

        Ok(())
    }

    fn parse_model(&self, parts: &[&str]) -> Result<Model, ParseError> {
        let name = parts[1].to_string();
        let mut params = Vec::new();

        // Reconstruct everything after ".model NAME" to handle TYPE(PARAMS...) correctly
        // e.g. ".model D1N4148 D(IS=1e-15)" or ".model 2N2222 NPN(IS=1e-15 BF=200)"
        let rest: String = parts[2..].join(" ");

        // Extract model type: everything before the first '('
        let model_type = if let Some(paren_pos) = rest.find('(') {
            rest[..paren_pos].trim().to_ascii_uppercase()
        } else {
            rest.trim().to_ascii_uppercase()
        };

        // Extract params from between parentheses (if any)
        if let (Some(open), Some(close)) = (rest.find('('), rest.rfind(')')) {
            let params_str = &rest[open + 1..close];
            for token in params_str.split_whitespace() {
                if let Some(eq_pos) = token.find('=') {
                    let key = token[..eq_pos].to_ascii_uppercase();
                    let value_str = &token[eq_pos + 1..];
                    let value = parse_value(value_str)
                        .map_err(|_| self.error(format!("Invalid model parameter value: {}", value_str)))?;
                    params.push((key, value));
                }
            }
        }

        Ok(Model {
            name,
            model_type,
            params,
        })
    }

    fn parse_param(&self, parts: &[&str]) -> Result<Parameter, ParseError> {
        self.require_parts(parts, 2, "name=value")?;
        let param_str = parts[1];
        let eq_pos = param_str.find('=')
            .ok_or_else(|| self.error("Parameter must be name=value format"))?;
        let name = param_str[..eq_pos].to_string();
        let value_str = &param_str[eq_pos + 1..];
        let value = parse_value(value_str)
            .map_err(|_| self.error(format!("Invalid parameter value: {}", value_str)))?;
        Ok(Parameter { name, value })
    }

    /// Parse a K (coupling) element: `K1 L1 L2 0.95`
    fn parse_coupling(&self, line: &str) -> Result<CouplingDirective, ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 4 {
            return Err(self.error("Coupling element requires: Kname L1 L2 coupling_coeff"));
        }

        let name = parts[0].to_string();
        let inductor1_name = parts[1].to_string();
        let inductor2_name = parts[2].to_string();

        // Validate inductor names start with L/l
        if !inductor1_name.starts_with('L') && !inductor1_name.starts_with('l') {
            return Err(self.error(format!(
                "Coupling '{}': first reference must be an inductor (L), got '{}'",
                name, inductor1_name
            )));
        }
        if !inductor2_name.starts_with('L') && !inductor2_name.starts_with('l') {
            return Err(self.error(format!(
                "Coupling '{}': second reference must be an inductor (L), got '{}'",
                name, inductor2_name
            )));
        }

        // Reject self-coupling
        if inductor1_name.eq_ignore_ascii_case(&inductor2_name) {
            return Err(self.error(format!(
                "Coupling '{}': cannot couple an inductor to itself ('{}')",
                name, inductor1_name
            )));
        }

        let coupling = parse_value(parts[3])
            .map_err(|_| self.error(format!("Invalid coupling coefficient: {}", parts[3])))?;

        if coupling <= 0.0 || coupling >= 1.0 {
            return Err(self.error(format!(
                "Coupling coefficient must be in (0, 1) exclusive, got {}",
                coupling
            )));
        }

        Ok(CouplingDirective {
            name,
            inductor1_name,
            inductor2_name,
            coupling,
        })
    }

    fn parse_pot_directive(&self, parts: &[&str], netlist: &Netlist) -> Result<PotDirective, ParseError> {
        // .pot Rname min max
        self.require_parts(parts, 4, ".pot Rname min_value max_value")?;

        let resistor_name = parts[1].to_string();

        // Check component type prefix: either starts with R, or for expanded subcircuit
        // names like "X1.R1", the part after the last dot starts with R.
        let base_name = resistor_name.rsplit('.').next().unwrap_or(&resistor_name);
        if !base_name.starts_with('R') && !base_name.starts_with('r') {
            return Err(self.error(format!(
                ".pot target must be a resistor (name starting with R), got '{}'",
                resistor_name
            )));
        }

        let min_value = self.parse_positive_value(parts[2], ".pot min")?;
        let max_value = self.parse_positive_value(parts[3], ".pot max")?;

        if min_value >= max_value {
            return Err(self.error(format!(
                ".pot min ({}) must be less than max ({})",
                min_value, max_value
            )));
        }
        if netlist.pots.iter().any(|p| p.resistor_name.eq_ignore_ascii_case(&resistor_name)) {
            return Err(self.error(format!(
                "Duplicate .pot directive for resistor '{}'",
                resistor_name
            )));
        }
        if netlist.pots.len() >= 32 {
            return Err(self.error("Maximum of 32 .pot directives supported"));
        }

        // Optional default value and/or quoted label:
        //   .pot Rname min max "Label"           — default = netlist nominal
        //   .pot Rname min max default "Label"   — explicit default
        let mut default_value = None;
        let mut label_start = 4;

        if parts.len() > 4 && !parts[4].starts_with('"') {
            // 5th token is not a quoted label — must be the default value
            default_value = Some(self.parse_positive_value(parts[4], ".pot default")?);
            let dv = default_value.unwrap();
            if dv < min_value || dv > max_value {
                return Err(self.error(format!(
                    ".pot default ({}) must be between min ({}) and max ({})",
                    dv, min_value, max_value
                )));
            }
            label_start = 5;
        }

        let label = if parts.len() > label_start {
            let rest = parts[label_start..].join(" ");
            if rest.starts_with('"') {
                let trimmed = rest.trim_matches('"');
                if trimmed.is_empty() {
                    None
                } else {
                    Some(trimmed.to_string())
                }
            } else {
                None
            }
        } else {
            None
        };

        // Resistor existence is validated after full parse (order-independent)
        Ok(PotDirective {
            resistor_name,
            min_value,
            max_value,
            default_value,
            label,
        })
    }

    fn parse_switch_directive(&self, parts: &[&str], netlist: &Netlist) -> Result<SwitchDirective, ParseError> {
        // .switch C1,L1 val0a/val0b val1a/val1b ...
        // Minimum: .switch <names> <pos0> <pos1>  (at least 2 positions)
        self.require_parts(parts, 4, ".switch names pos0 pos1 [pos2 ...]")?;

        // Parse component names (comma-separated)
        let component_names: Vec<String> = parts[1]
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        if component_names.is_empty() {
            return Err(self.error(".switch requires at least one component name"));
        }

        // Validate component name prefixes: for expanded subcircuit names like "X1.C1",
        // check the part after the last dot.
        for name in &component_names {
            let base = name.rsplit('.').next().unwrap_or(name);
            let first = base.chars().next().unwrap_or(' ').to_ascii_uppercase();
            if !matches!(first, 'R' | 'C' | 'L') {
                return Err(self.error(format!(
                    ".switch component '{}' must start with R, C, or L", name
                )));
            }
        }

        let num_comps = component_names.len();

        // Separate position values from optional trailing quoted label
        let value_parts = &parts[2..];
        let label_start = value_parts.iter().position(|p| p.starts_with('"'));
        let (pos_parts, label) = if let Some(idx) = label_start {
            let label_text = value_parts[idx..].join(" ");
            let trimmed = label_text.trim_matches('"');
            let label = if trimmed.is_empty() { None } else { Some(trimmed.to_string()) };
            (&value_parts[..idx], label)
        } else {
            (value_parts, None)
        };

        // Parse position values
        let mut positions = Vec::new();
        for &pos_str in pos_parts {
            let values: Vec<f64> = pos_str
                .split('/')
                .map(|v| {
                    let val = parse_value(v.trim())
                        .map_err(|_| self.error(format!("Invalid .switch value: '{}'", v)))?;
                    if val <= 0.0 || !val.is_finite() {
                        return Err(self.error(format!(
                            ".switch value must be positive and finite, got {}", val
                        )));
                    }
                    Ok(val)
                })
                .collect::<Result<Vec<_>, _>>()?;

            if values.len() != num_comps {
                return Err(self.error(format!(
                    ".switch position '{}' has {} values but {} components were specified",
                    pos_str, values.len(), num_comps
                )));
            }
            positions.push(values);
        }

        if positions.len() < 2 {
            return Err(self.error(".switch requires at least 2 positions"));
        }
        if positions.len() > 32 {
            return Err(self.error("Maximum of 32 positions per switch"));
        }

        // Check for duplicate component names across all switches
        for name in &component_names {
            if netlist.switches.iter().any(|sw| {
                sw.component_names.iter().any(|n| n.eq_ignore_ascii_case(name))
            }) {
                return Err(self.error(format!(
                    "Component '{}' is already used in another .switch directive", name
                )));
            }
        }

        if netlist.switches.len() >= 16 {
            return Err(self.error("Maximum of 16 .switch directives supported"));
        }

        Ok(SwitchDirective {
            component_names,
            positions,
            label,
        })
    }

    fn parse_input_impedance_directive(&self, parts: &[&str], netlist: &mut Netlist) -> Result<(), ParseError> {
        // .input_impedance <value>
        self.require_parts(parts, 2, ".input_impedance <value>")?;

        if netlist.input_impedance.is_some() {
            return Err(self.error("Duplicate .input_impedance directive"));
        }

        let value = self.parse_positive_value(parts[1], ".input_impedance")?;
        netlist.input_impedance = Some(value);
        Ok(())
    }

    /// Validate that two nodes are not the same (self-connected component).
    fn check_self_connection(&self, n1: &str, n2: &str, component: &str) -> Result<(), ParseError> {
        if n1.eq_ignore_ascii_case(n2) {
            return Err(self.error(format!(
                "Component '{}' has both terminals connected to the same node '{}'",
                component, n1
            )));
        }
        Ok(())
    }

    /// Validate minimum field count for a component line.
    fn require_parts(&self, parts: &[&str], min: usize, description: &str) -> Result<(), ParseError> {
        if parts.len() < min {
            return Err(self.error(format!("{} requires: {}", parts.first().unwrap_or(&"?"), description)));
        }
        Ok(())
    }

    /// Parse a component value and validate it is positive and finite.
    ///
    /// Used by R, C, L parsers which all require strictly positive values.
    fn parse_positive_value(&self, raw: &str, component_type: &str) -> Result<f64, ParseError> {
        let value = parse_value(raw)
            .map_err(|_| self.error(format!("Invalid {} value: {}", component_type, raw)))?;
        if value <= 0.0 || !value.is_finite() {
            return Err(self.error(format!(
                "{} value must be positive and finite, got {}", component_type, value
            )));
        }
        Ok(value)
    }

    fn parse_element(&self, line: &str) -> Result<Element, ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return Err(self.error("Empty element line"));
        }

        let name = parts[0];
        let first_char = name.chars().next().unwrap_or(' ').to_ascii_uppercase();

        match first_char {
            'R' => self.parse_resistor(&parts),
            'C' => self.parse_capacitor(&parts),
            'L' => self.parse_inductor(&parts),
            'V' => self.parse_voltage_source(&parts),
            'I' => self.parse_current_source(&parts),
            'D' => self.parse_diode(&parts),
            'Q' => self.parse_bjt(&parts),
            'J' => self.parse_jfet(&parts),
            'M' => self.parse_mosfet(&parts),
            'T' => self.parse_triode(&parts),
            'U' => self.parse_opamp(&parts),
            'E' => self.parse_vcvs(&parts),
            'G' => self.parse_vccs(&parts),
            'X' => self.parse_subckt_instance(&parts),
            _ => Err(self.error(format!("Unknown element type: {}", first_char))),
        }
    }

    fn parse_resistor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 4, "Rname n+ n- value")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        let value = self.parse_positive_value(parts[3], "Resistor")?;
        Ok(Element::Resistor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
        })
    }

    fn parse_capacitor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 4, "Cname n+ n- value")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        let value = self.parse_positive_value(parts[3], "Capacitor")?;

        let ic = parts[4..].iter()
            .find(|p| p.to_uppercase().starts_with("IC="))
            .map(|p| parse_value(&p[3..]))
            .transpose()
            .map_err(|_| self.error("Invalid IC value"))?;

        Ok(Element::Capacitor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
            ic,
        })
    }

    fn parse_inductor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 4, "Lname n+ n- value")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        let value = self.parse_positive_value(parts[3], "Inductor")?;
        Ok(Element::Inductor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
        })
    }

    fn parse_voltage_source(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 3, "Vname n+ n-")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;

        let mut dc = None;
        let mut ac = None;

        // Parse DC and AC values
        let mut i = 3;
        while i < parts.len() {
            let part_upper = parts[i].to_uppercase();
            if part_upper == "DC" && i + 1 < parts.len() {
                dc = Some(parse_value(parts[i + 1])
                    .map_err(|_| self.error(format!("Invalid DC value: {}", parts[i + 1])))?);
                i += 2;
            } else if part_upper == "AC" && i + 1 < parts.len() {
                let mag = parse_value(parts[i + 1])
                    .map_err(|_| self.error(format!("Invalid AC magnitude: {}", parts[i + 1])))?;
                let phase = if i + 2 < parts.len() {
                    parse_value(parts[i + 2]).unwrap_or(0.0)
                } else {
                    0.0
                };
                ac = Some((mag, phase));
                i += 3;
            } else if dc.is_none() {
                // Bare value is DC
                dc = Some(parse_value(parts[i])
                    .map_err(|_| self.error(format!("Invalid DC value: {}", parts[i])))?);
                i += 1;
            } else {
                i += 1;
            }
        }

        Ok(Element::VoltageSource {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            dc,
            ac,
        })
    }

    fn parse_current_source(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 3, "Iname n+ n-")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;

        let dc = match parts.get(3) {
            Some(p) if p.to_uppercase() == "DC" => {
                let val_str = parts.get(4).ok_or_else(|| self.error("DC keyword requires a value"))?;
                Some(parse_value(val_str)
                    .map_err(|_| self.error(format!("Invalid DC value: {}", val_str)))?)
            }
            Some(p) => Some(parse_value(p)
                .map_err(|_| self.error(format!("Invalid DC value: {}", p)))?),
            None => None,
        };

        Ok(Element::CurrentSource {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            dc,
        })
    }

    fn parse_diode(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 4, "Dname n+ n- modelname")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        Ok(Element::Diode {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            model: parts[3].to_string(),
        })
    }

    fn parse_bjt(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Qname nc nb ne [ns] modelname
        self.require_parts(parts, 5, "Qname nc nb ne modelname")?;
        // Model is always the last part (substrate node, if present, is skipped)
        Ok(Element::Bjt {
            name: parts[0].to_string(),
            nc: parts[1].to_string(),
            nb: parts[2].to_string(),
            ne: parts[3].to_string(),
            model: parts[parts.len() - 1].to_string(),
        })
    }

    fn parse_jfet(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 5, "Jname nd ng ns modelname")?;
        Ok(Element::Jfet {
            name: parts[0].to_string(),
            nd: parts[1].to_string(),
            ng: parts[2].to_string(),
            ns: parts[3].to_string(),
            model: parts[4].to_string(),
        })
    }

    fn parse_mosfet(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 6, "Mname nd ng ns nb modelname")?;
        Ok(Element::Mosfet {
            name: parts[0].to_string(),
            nd: parts[1].to_string(),
            ng: parts[2].to_string(),
            ns: parts[3].to_string(),
            nb: parts[4].to_string(),
            model: parts[5].to_string(),
        })
    }

    fn parse_opamp(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Uname n_plus n_minus n_out modelname
        self.require_parts(parts, 5, "Uname n_plus n_minus n_out modelname")?;
        Ok(Element::Opamp {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            n_out: parts[3].to_string(),
            model: parts[4].to_string(),
        })
    }

    fn parse_triode(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Tname n_grid n_plate n_cathode modelname
        self.require_parts(parts, 5, "Tname n_grid n_plate n_cathode modelname")?;
        Ok(Element::Triode {
            name: parts[0].to_string(),
            n_grid: parts[1].to_string(),
            n_plate: parts[2].to_string(),
            n_cathode: parts[3].to_string(),
            model: parts[4].to_string(),
        })
    }

    fn parse_vcvs(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Ename out+ out- ctrl+ ctrl- gain
        self.require_parts(parts, 6, "Ename out+ out- ctrl+ ctrl- gain")?;
        let gain = parse_value(parts[5])
            .map_err(|_| self.error(format!("Invalid VCVS gain: {}", parts[5])))?;
        if !gain.is_finite() || gain == 0.0 {
            return Err(self.error(format!(
                "VCVS gain must be finite and non-zero, got {}", gain
            )));
        }
        Ok(Element::Vcvs {
            name: parts[0].to_string(),
            out_p: parts[1].to_string(),
            out_n: parts[2].to_string(),
            ctrl_p: parts[3].to_string(),
            ctrl_n: parts[4].to_string(),
            gain,
        })
    }

    fn parse_vccs(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Gname out+ out- ctrl+ ctrl- gm
        self.require_parts(parts, 6, "Gname out+ out- ctrl+ ctrl- gm")?;
        let gm = parse_value(parts[5])
            .map_err(|_| self.error(format!("Invalid VCCS transconductance: {}", parts[5])))?;
        if !gm.is_finite() || gm == 0.0 {
            return Err(self.error(format!(
                "VCCS transconductance must be finite and non-zero, got {}", gm
            )));
        }
        Ok(Element::Vccs {
            name: parts[0].to_string(),
            out_p: parts[1].to_string(),
            out_n: parts[2].to_string(),
            ctrl_p: parts[3].to_string(),
            ctrl_n: parts[4].to_string(),
            gm,
        })
    }

    fn parse_subckt_instance(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 3, "Xname nodes... subcktname")?;
        Ok(Element::SubcktInstance {
            name: parts[0].to_string(),
            nodes: parts[1..parts.len() - 1].iter().map(|s| s.to_string()).collect(),
            subckt: parts[parts.len() - 1].to_string(),
        })
    }
}

/// Try to parse infix notation where a scale character replaces the decimal point.
///
/// Examples: "6n8" → 6.8e-9, "3n3" → 3.3e-9, "4k7" → 4.7e3, "1m5" → 1.5e-3
///
/// Pattern: `<digits><scale_char><digits>` where scale_char is one of T,G,K,M,U,N,P.
fn try_parse_infix(s: &str) -> Option<f64> {
    // Need at least 3 chars: digit, scale, digit
    if s.len() < 3 {
        return None;
    }

    // Find the scale character (must not be first or last, and must be alphabetic)
    let bytes = s.as_bytes();
    let mut scale_pos = None;
    for (i, &b) in bytes.iter().enumerate() {
        if i == 0 { continue; }
        let c = (b as char).to_ascii_uppercase();
        if matches!(c, 'T' | 'G' | 'K' | 'M' | 'U' | 'N' | 'P') {
            // Check: digits before, digits after
            let before = &s[..i];
            let after = &s[i + 1..];
            if !before.is_empty()
                && !after.is_empty()
                && before.chars().all(|c| c.is_ascii_digit())
                && after.chars().all(|c| c.is_ascii_digit())
            {
                scale_pos = Some((i, c));
                break;
            }
        }
    }

    let (pos, scale_char) = scale_pos?;
    let before = &s[..pos];
    let after = &s[pos + 1..];
    let decimal_str = format!("{}.{}", before, after);
    let base: f64 = decimal_str.parse().ok()?;
    let scale = match scale_char {
        'T' => 1e12,
        'G' => 1e9,
        'K' => 1e3,
        'M' => 1e-3,
        'U' => 1e-6,
        'N' => 1e-9,
        'P' => 1e-12,
        _ => return None,
    };
    let result = base * scale;
    if result.is_finite() { Some(result) } else { None }
}

/// Parse a SPICE value with optional scale suffix.
///
/// Examples:
/// - "1k" -> 1000.0
/// - "4.7u" -> 4.7e-6
/// - "10pF" -> 10e-12
/// - "1F" -> 1.0 (Farad, not femto)
/// - "6n8" -> 6.8e-9 (infix notation)
/// Parse a component value string with engineering notation (e.g. "10k", "4n7", "1Meg").
pub fn parse_value(s: &str) -> Result<f64, ParseFloatError> {
    let s = s.trim();
    if s.is_empty() {
        return Err(ParseFloatError);
    }

    // Try infix notation first (e.g. "6n8" → 6.8e-9)
    if let Some(val) = try_parse_infix(s) {
        return Ok(val);
    }

    let s_upper = s.to_uppercase();

    // Check for MEG first (must be before stripping units)
    if s_upper.ends_with("MEG") {
        let num_part = &s[..s.len()-3];
        if num_part.is_empty() {
            return Err(ParseFloatError);
        }
        let value: f64 = num_part.parse().map_err(|_| ParseFloatError)?;
        return Ok(value * 1e6);
    }

    // Strip unit suffixes FIRST (F, H, V, A, OHM, HZ)
    // Note: We remove from s_upper to get the uppercase version, then apply to original s
    let stripped_upper = s_upper.trim_end_matches(|c: char| {
        matches!(c.to_ascii_uppercase(), 'F' | 'H' | 'V' | 'A' | 'S' | 'Z')
    });

    // Calculate how much we stripped
    let chars_stripped = s_upper.len() - stripped_upper.len();
    let mut num_part = &s[..s.len().saturating_sub(chars_stripped)];
    let mut scale = 1.0;

    // Now parse scale suffix from what's left
    if let Some(c) = stripped_upper.chars().last() {
        let (new_num, new_scale) = match c {
            'T' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e12),
            'G' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e9),
            'K' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e3),
            'M' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-3),  // M = milli
            'U' | 'µ' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-6),
            'N' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-9),
            'P' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-12),
            // Note: 'F' as femto is only recognized if no unit was stripped
            // This avoids ambiguity with Farad
            'F' if chars_stripped == 0 && num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-15),
            _ => (num_part, 1.0),
        };
        num_part = new_num;
        scale = new_scale;
    }

    if num_part.is_empty() {
        return Err(ParseFloatError);
    }

    let value: f64 = num_part.parse().map_err(|_| ParseFloatError)?;
    let result = value * scale;
    if !result.is_finite() {
        return Err(ParseFloatError);
    }
    Ok(result)
}

/// Error type for float parsing failures.
#[derive(Debug, Clone, Copy)]
pub struct ParseFloatError;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_resistor() {
        let netlist = Netlist::parse("Test Circuit\nR1 1 0 1k\n").unwrap();
        assert_eq!(netlist.elements.len(), 1);
        match &netlist.elements[0] {
            Element::Resistor { name, n_plus, n_minus, value } => {
                assert_eq!(name, "R1");
                assert_eq!(n_plus, "1");
                assert_eq!(n_minus, "0");
                assert_eq!(*value, 1000.0);
            }
            _ => panic!("Expected resistor"),
        }
    }

    #[test]
    fn test_parse_capacitor() {
        let netlist = Netlist::parse("Test\nC1 1 0 10u\n").unwrap();
        match &netlist.elements[0] {
            Element::Capacitor { name, value, .. } => {
                assert_eq!(name, "C1");
                assert!((value - 10e-6).abs() < 1e-15, "Expected ~10uF, got {}", value);
            }
            _ => panic!("Expected capacitor"),
        }
    }

    #[test]
    fn test_parse_bjt() {
        let netlist = Netlist::parse("Test\nQ1 3 2 1 2N2222\n.model 2N2222 NPN(IS=1e-15 BF=200)\n").unwrap();
        match &netlist.elements[0] {
            Element::Bjt { name, nc, nb, ne, model } => {
                assert_eq!(name, "Q1");
                assert_eq!(nc, "3");
                assert_eq!(nb, "2");
                assert_eq!(ne, "1");
                assert_eq!(model, "2N2222");
            }
            _ => panic!("Expected BJT"),
        }
    }

    #[test]
    fn test_parse_model() {
        let netlist = Netlist::parse("Test\n.model 2N2222 NPN(IS=1e-15 BF=200)\n").unwrap();
        assert_eq!(netlist.models.len(), 1);
        let model = &netlist.models[0];
        assert_eq!(model.name, "2N2222");
        assert!(model.model_type.starts_with("NPN"), "Model type should be NPN, got {}", model.model_type);
    }

    #[test]
    fn test_parse_value() {
        assert_eq!(parse_value("1k").unwrap(), 1e3);
        assert_eq!(parse_value("4.7u").unwrap(), 4.7e-6);
        assert_eq!(parse_value("10pF").unwrap(), 10e-12);
        assert_eq!(parse_value("1MEG").unwrap(), 1e6);
    }

    #[test]
    fn test_parse_valid_scale_suffixes() {
        assert!((parse_value("1T").unwrap() - 1e12).abs() < 1e3);
        assert!((parse_value("1G").unwrap() - 1e9).abs() < 1.0);
        assert!((parse_value("1MEG").unwrap() - 1e6).abs() < 1.0);
        assert!((parse_value("1k").unwrap() - 1e3).abs() < 1e-6);
        assert!((parse_value("1m").unwrap() - 1e-3).abs() < 1e-12);
        assert!((parse_value("1u").unwrap() - 1e-6).abs() < 1e-15);
        assert!((parse_value("1n").unwrap() - 1e-9).abs() < 1e-18);
        assert!((parse_value("1p").unwrap() - 1e-12).abs() < 1e-21);
        // "1f" is ambiguous with Farad unit suffix; femto requires no unit suffix after it
        // "1F" = 1.0 Farad (F is unit, not scale), by design
        assert!((parse_value("1F").unwrap() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_parse_negative_resistance_rejected() {
        let result = Netlist::parse("Test\nR1 1 0 -1k\n");
        assert!(result.is_err(), "Negative resistance should be rejected");
    }

    #[test]
    fn test_parse_zero_resistance_rejected() {
        let result = Netlist::parse("Test\nR1 1 0 0\n");
        assert!(result.is_err(), "Zero resistance should be rejected");
    }

    #[test]
    fn test_parse_nan_value_rejected() {
        let result = parse_value("NaN");
        assert!(result.is_err(), "NaN value should be rejected");
    }

    #[test]
    fn test_parse_inf_value_rejected() {
        let result = parse_value("inf");
        assert!(result.is_err(), "Infinity value should be rejected");
    }

    #[test]
    fn test_parse_negative_capacitance_rejected() {
        let result = Netlist::parse("Test\nC1 1 0 -1u\n");
        assert!(result.is_err(), "Negative capacitance should be rejected");
    }

    #[test]
    fn test_parse_zero_inductance_rejected() {
        let result = Netlist::parse("Test\nL1 1 0 0\n");
        assert!(result.is_err(), "Zero inductance should be rejected");
    }

    #[test]
    fn test_parse_missing_value() {
        let result = Netlist::parse("Test\nR1 1 0\n");
        assert!(result.is_err(), "Missing value should be rejected");
    }

    #[test]
    fn test_parse_pot_directive() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.pots.len(), 1);
        assert_eq!(netlist.pots[0].resistor_name, "R1");
        assert_eq!(netlist.pots[0].min_value, 1e3);
        assert_eq!(netlist.pots[0].max_value, 100e3);
    }

    #[test]
    fn test_parse_pot_two_pots() {
        let spice = "Test\nR1 1 0 10k\nR2 2 0 5k\n.pot R1 1k 100k\n.pot R2 500 50k\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.pots.len(), 2);
        assert_eq!(netlist.pots[0].resistor_name, "R1");
        assert_eq!(netlist.pots[1].resistor_name, "R2");
    }

    #[test]
    fn test_parse_pot_missing_resistor() {
        let spice = "Test\nR1 1 0 10k\n.pot R2 1k 100k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Pot referencing non-existent resistor should fail");
    }

    #[test]
    fn test_parse_pot_not_a_resistor() {
        let spice = "Test\nC1 1 0 10u\n.pot C1 1u 100u\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Pot targeting non-resistor should fail");
    }

    #[test]
    fn test_parse_pot_min_gte_max() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 100k 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Pot with min >= max should fail");
    }

    #[test]
    fn test_parse_pot_duplicate() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n.pot R1 2k 50k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Duplicate pot for same resistor should fail");
    }

    #[test]
    fn test_parse_pot_max_exceeded() {
        let mut spice = String::from("Test\n");
        for i in 1..=33 {
            spice.push_str(&format!("R{i} {i} 0 10k\n"));
        }
        for i in 1..=33 {
            spice.push_str(&format!(".pot R{i} 1k 100k\n"));
        }
        let result = Netlist::parse(&spice);
        assert!(result.is_err(), "More than 32 pots should fail");
    }

    #[test]
    fn test_parse_pot_four_pots_ok() {
        let spice = "Test\nR1 1 0 10k\nR2 2 0 5k\nR3 3 0 3k\nR4 4 0 2k\n\
                      .pot R1 1k 100k\n.pot R2 500 50k\n.pot R3 100 10k\n.pot R4 200 20k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "4 pots should be allowed: {:?}", result.err());
        let netlist = result.unwrap();
        assert_eq!(netlist.pots.len(), 4);
    }

    #[test]
    fn test_parse_pot_negative_min() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 -1k 100k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Negative min value should fail");
    }

    #[test]
    fn test_parse_pot_missing_args() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Missing max value should fail");
    }

    #[test]
    fn test_parse_pot_case_insensitive() {
        // SPICE is case-insensitive: .pot r1 should match R1
        let spice = "Test\nR1 1 0 10k\n.pot r1 1k 100k\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.pots.len(), 1);
        assert_eq!(netlist.pots[0].resistor_name, "r1");
    }

    #[test]
    fn test_parse_pot_order_independent() {
        // .pot can appear before the resistor it references
        let spice = "Test\n.pot R1 1k 100k\nR1 1 0 10k\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.pots.len(), 1);
        assert_eq!(netlist.pots[0].resistor_name, "R1");
    }

    #[test]
    fn test_parse_pot_case_insensitive_duplicate() {
        // r1 and R1 should be treated as the same resistor for duplicate detection
        let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n.pot r1 2k 50k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Case-insensitive duplicate should fail");
    }

    // ======================================================================
    // .pot / .switch label tests
    // ======================================================================

    #[test]
    fn test_parse_pot_with_label() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k \"Volume\"\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.pots[0].label, Some("Volume".to_string()));
    }

    #[test]
    fn test_parse_pot_with_multi_word_label() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k \"HF Boost\"\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.pots[0].label, Some("HF Boost".to_string()));
    }

    #[test]
    fn test_parse_pot_without_label() {
        let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.pots[0].label, None);
    }

    #[test]
    fn test_parse_switch_with_label() {
        let spice = "Test\nC1 1 0 100n\n.switch C1 100n 220n 470n \"Bright\"\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.switches[0].label, Some("Bright".to_string()));
        assert_eq!(netlist.switches[0].positions.len(), 3);
    }

    #[test]
    fn test_parse_switch_with_multi_word_label() {
        let spice = "Test\nC1 1 0 100n\nL1 1 0 100m\n.switch C1,L1 100n/100m 220n/176m \"HF Boost Freq\"\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.switches[0].label, Some("HF Boost Freq".to_string()));
        assert_eq!(netlist.switches[0].positions.len(), 2);
    }

    #[test]
    fn test_parse_switch_without_label() {
        let spice = "Test\nC1 1 0 100n\n.switch C1 100n 220n\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.switches[0].label, None);
    }

    // ======================================================================
    // .input_impedance directive tests
    // ======================================================================

    #[test]
    fn test_parse_input_impedance_basic() {
        let spice = "Test\nR1 1 0 1k\n.input_impedance 600\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.input_impedance, Some(600.0));
    }

    #[test]
    fn test_parse_input_impedance_engineering_notation() {
        let spice = "Test\nR1 1 0 1k\n.input_impedance 10k\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.input_impedance, Some(10_000.0));
    }

    #[test]
    fn test_parse_input_impedance_default_none() {
        let spice = "Test\nR1 1 0 1k\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.input_impedance, None);
    }

    #[test]
    fn test_parse_input_impedance_duplicate_error() {
        let spice = "Test\nR1 1 0 1k\n.input_impedance 600\n.input_impedance 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Duplicate .input_impedance should fail");
        let err = result.unwrap_err();
        assert!(err.message.contains("Duplicate"), "Error: {}", err.message);
    }

    #[test]
    fn test_common_emitter_amp() {
        let spice = r#"Common Emitter Amplifier
Vcc vcc 0 9V
Vin in 0 DC 0 AC 1V
R1 vcc base 100k
R2 base 0 22k
Rc vcc coll 4.7k
Re emit 0 1k
C1 in base 10u
C2 coll out 10u
Q1 coll base emit 2N2222
.model 2N2222 NPN(IS=1e-15 BF=200)
.end
"#;
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.elements.len(), 9);
        assert_eq!(netlist.models.len(), 1);
    }

    // ======================================================================
    // Edge case tests for parser error handling
    // ======================================================================

    // 1. Missing model card: device references a .model that doesn't exist
    #[test]
    fn test_missing_model_card_diode() {
        let spice = "Test\nD1 1 2 UnknownModel\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Diode referencing undefined model should be rejected");
        let err = result.unwrap_err();
        assert!(
            err.message.contains("UnknownModel"),
            "Error should mention the missing model name, got: {}", err.message
        );
    }

    #[test]
    fn test_missing_model_card_bjt() {
        let spice = "Test\nQ1 3 2 1 NonExistent\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "BJT referencing undefined model should be rejected");
    }

    #[test]
    fn test_missing_model_card_jfet() {
        let spice = "Test\nJ1 3 2 1 NoSuchModel\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "JFET referencing undefined model should be rejected");
    }

    #[test]
    fn test_missing_model_card_mosfet() {
        let spice = "Test\nM1 3 2 1 0 GhostModel\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "MOSFET referencing undefined model should be rejected");
    }

    #[test]
    fn test_model_reference_exists() {
        // Positive test: model IS defined, should parse OK
        let spice = "Test\nD1 1 2 MyDiode\n.model MyDiode D(IS=1e-14)\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Diode with defined model should succeed: {:?}", result.err());
    }

    #[test]
    fn test_model_reference_case_insensitive() {
        // SPICE model references are case-insensitive
        let spice = "Test\nD1 1 2 mydiode\n.model MYDIODE D(IS=1e-14)\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Model reference should be case-insensitive: {:?}", result.err());
    }

    // 2. Duplicate component names
    #[test]
    fn test_duplicate_component_names_resistors() {
        let spice = "Test\nR1 1 0 1k\nR1 2 0 2k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Duplicate component names should be rejected");
        let err = result.unwrap_err();
        assert!(
            err.message.contains("Duplicate") && err.message.contains("R1"),
            "Error should mention duplicate and the name, got: {}", err.message
        );
    }

    #[test]
    fn test_duplicate_component_names_case_insensitive() {
        // SPICE names are case-insensitive: R1 and r1 are the same component
        let spice = "Test\nR1 1 0 1k\nr1 2 0 2k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Case-insensitive duplicate names should be rejected");
    }

    #[test]
    fn test_duplicate_names_different_types() {
        // Even different component types with the same name should be rejected
        let spice = "Test\nR1 1 0 1k\nC1 2 0 10u\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Different component names should be accepted: {:?}", result.err());
    }

    // 3. Invalid node names / special characters
    #[test]
    fn test_node_names_with_alphanumeric() {
        // Standard alphanumeric node names should work fine
        let spice = "Test\nR1 node_a node_b 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Alphanumeric node names should work: {:?}", result.err());
    }

    #[test]
    fn test_node_names_numeric() {
        // Purely numeric node names (common in SPICE)
        let spice = "Test\nR1 1 0 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Numeric node names should work: {:?}", result.err());
    }

    // 4. Invalid number format
    #[test]
    fn test_invalid_number_format_alpha() {
        let spice = "Test\nR1 1 2 abc\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Alphabetic value 'abc' should be rejected");
    }

    #[test]
    fn test_invalid_number_format_special_chars() {
        let spice = "Test\nR1 1 2 @#$\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Special character value should be rejected");
    }

    #[test]
    fn test_invalid_number_format_empty_suffix() {
        // Just a suffix with no numeric part
        let result = parse_value("k");
        assert!(result.is_err(), "Bare suffix with no number should be rejected");
    }

    #[test]
    fn test_invalid_number_format_double_dot() {
        let result = parse_value("1.2.3");
        assert!(result.is_err(), "Double decimal point should be rejected");
    }

    // 5. Empty netlist (title only, no components)
    #[test]
    fn test_empty_netlist() {
        let spice = "My Empty Circuit\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Empty netlist (title only) should parse: {:?}", result.err());
        let netlist = result.unwrap();
        assert_eq!(netlist.title, "My Empty Circuit");
        assert!(netlist.elements.is_empty());
        assert!(netlist.models.is_empty());
    }

    #[test]
    fn test_empty_netlist_with_end() {
        let spice = "My Circuit\n.end\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Netlist with only .end should parse: {:?}", result.err());
        let netlist = result.unwrap();
        assert!(netlist.elements.is_empty());
    }

    #[test]
    fn test_empty_netlist_only_comments() {
        let spice = "My Circuit\n* This is a comment\n* Another comment\n.end\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Comment-only netlist should parse: {:?}", result.err());
        let netlist = result.unwrap();
        assert!(netlist.elements.is_empty());
    }

    #[test]
    fn test_completely_empty_input() {
        let spice = "";
        let result = Netlist::parse(spice);
        // Empty string should still parse (empty title)
        assert!(result.is_ok(), "Completely empty input should parse: {:?}", result.err());
        let netlist = result.unwrap();
        assert_eq!(netlist.title, "");
        assert!(netlist.elements.is_empty());
    }

    // 6. Floating nodes (single-terminal connections)
    // Note: The parser itself doesn't validate node connectivity; that's an MNA concern.
    // This test documents that a node connected to only one component terminal parses OK.
    #[test]
    fn test_floating_node_parses_ok() {
        // Node "3" is only connected to R2's n+ — it's electrically floating
        // The parser accepts this; downstream MNA assembly would detect the issue
        let spice = "Test\nR1 1 0 1k\nR2 3 0 2k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Parser doesn't validate connectivity (MNA's job): {:?}", result.err());
    }

    // 7. Duplicate node connections (self-connected component)
    #[test]
    fn test_self_connected_resistor() {
        let spice = "Test\nR1 1 1 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Resistor from node to itself should be rejected");
        let err = result.unwrap_err();
        assert!(
            err.message.contains("same node"),
            "Error should mention same node, got: {}", err.message
        );
    }

    #[test]
    fn test_self_connected_capacitor() {
        let spice = "Test\nC1 2 2 10u\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Capacitor from node to itself should be rejected");
    }

    #[test]
    fn test_self_connected_inductor() {
        let spice = "Test\nL1 out out 100m\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Inductor from node to itself should be rejected");
    }

    #[test]
    fn test_self_connected_diode() {
        let spice = "Test\nD1 1 1 MyDiode\n.model MyDiode D(IS=1e-14)\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Diode from node to itself should be rejected");
    }

    #[test]
    fn test_self_connected_case_insensitive() {
        // SPICE node names are case-insensitive: "VCC" and "vcc" are the same node
        let spice = "Test\nR1 VCC vcc 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Case-insensitive self-connection should be rejected");
    }

    #[test]
    fn test_self_connected_voltage_source() {
        // Voltage source from node to itself is physically meaningless and should be rejected
        let spice = "Test\nV1 0 0 DC 5\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Voltage source self-connection should be rejected");
    }

    #[test]
    fn test_self_connected_current_source() {
        // Current source from node to itself is physically meaningless and should be rejected
        let spice = "Test\nI1 1 1 DC 1m\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Current source self-connection should be rejected");
    }

    // 8. Unknown element type
    #[test]
    fn test_unknown_element_type() {
        let spice = "Test\nZ1 1 2 100\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Unknown element type 'Z' should be rejected");
        let err = result.unwrap_err();
        assert!(
            err.message.contains("Unknown element type"),
            "Error should mention unknown element type, got: {}", err.message
        );
    }

    // 9. Insufficient fields for various components
    #[test]
    fn test_missing_value_capacitor() {
        let result = Netlist::parse("Test\nC1 1 0\n");
        assert!(result.is_err(), "Capacitor without value should be rejected");
    }

    #[test]
    fn test_missing_value_inductor() {
        let result = Netlist::parse("Test\nL1 1 0\n");
        assert!(result.is_err(), "Inductor without value should be rejected");
    }

    #[test]
    fn test_missing_nodes_voltage_source() {
        let result = Netlist::parse("Test\nV1 1\n");
        assert!(result.is_err(), "Voltage source with one node should be rejected");
    }

    #[test]
    fn test_missing_model_diode() {
        let result = Netlist::parse("Test\nD1 1 2\n");
        assert!(result.is_err(), "Diode without model name should be rejected");
    }

    #[test]
    fn test_missing_fields_bjt() {
        let result = Netlist::parse("Test\nQ1 3 2\n");
        assert!(result.is_err(), "BJT with too few fields should be rejected");
    }

    #[test]
    fn test_missing_fields_jfet() {
        let result = Netlist::parse("Test\nJ1 3 2\n");
        assert!(result.is_err(), "JFET with too few fields should be rejected");
    }

    #[test]
    fn test_missing_fields_mosfet() {
        let result = Netlist::parse("Test\nM1 3 2 1\n");
        assert!(result.is_err(), "MOSFET with too few fields should be rejected");
    }

    // 10. Zero component values
    #[test]
    fn test_zero_capacitance_rejected() {
        let result = Netlist::parse("Test\nC1 1 0 0\n");
        assert!(result.is_err(), "Zero capacitance should be rejected");
    }

    // 11. Continuation lines and comments
    #[test]
    fn test_continuation_line() {
        let spice = "Test\nR1 1 0\n+ 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Continuation line should work: {:?}", result.err());
        let netlist = result.unwrap();
        assert_eq!(netlist.elements.len(), 1);
        match &netlist.elements[0] {
            Element::Resistor { value, .. } => {
                assert_eq!(*value, 1000.0);
            }
            _ => panic!("Expected resistor"),
        }
    }

    #[test]
    fn test_inline_comment_semicolon() {
        let spice = "Test\nR1 1 0 1k ; this is a comment\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Inline semicolon comment should be stripped: {:?}", result.err());
    }

    #[test]
    fn test_inline_comment_dollar() {
        let spice = "Test\nR1 1 0 1k $ this is a comment\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Inline dollar comment should be stripped: {:?}", result.err());
    }

    #[test]
    fn test_star_comment_line() {
        let spice = "Test\n* This is a comment\nR1 1 0 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Star comment lines should be ignored: {:?}", result.err());
        let netlist = result.unwrap();
        assert_eq!(netlist.elements.len(), 1);
    }

    // 12. Edge cases for parse_value
    #[test]
    fn test_parse_value_empty_string() {
        let result = parse_value("");
        assert!(result.is_err(), "Empty string value should be rejected");
    }

    #[test]
    fn test_parse_value_whitespace_only() {
        let result = parse_value("   ");
        assert!(result.is_err(), "Whitespace-only value should be rejected");
    }

    #[test]
    fn test_parse_value_negative() {
        // parse_value itself accepts negative numbers; component parsers reject them
        let result = parse_value("-1k");
        assert!(result.is_ok(), "parse_value should accept negative numbers");
        assert_eq!(result.unwrap(), -1000.0);
    }

    #[test]
    fn test_parse_value_scientific_notation() {
        let result = parse_value("1e-12");
        assert!(result.is_ok(), "Scientific notation should work");
        assert!((result.unwrap() - 1e-12).abs() < 1e-24);
    }

    #[test]
    fn test_parse_value_bare_meg() {
        let result = parse_value("MEG");
        assert!(result.is_err(), "Bare 'MEG' with no number should be rejected");
    }

    // 13. Model parsing edge cases
    #[test]
    fn test_model_without_params() {
        let spice = "Test\n.model MyDiode D\nD1 1 2 MyDiode\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Model without params should parse: {:?}", result.err());
        let netlist = result.unwrap();
        assert_eq!(netlist.models[0].params.len(), 0);
    }

    #[test]
    fn test_model_missing_type() {
        let spice = "Test\n.model MyDiode\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), ".model without type should be rejected");
    }

    // 14. Directive edge cases
    #[test]
    fn test_unknown_directive_ignored() {
        let spice = "Test\n.options RELTOL=1e-3\nR1 1 0 1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), "Unknown directives should be ignored: {:?}", result.err());
    }

    #[test]
    fn test_param_directive() {
        let spice = "Test\n.param Rval=1k\n";
        let result = Netlist::parse(spice);
        assert!(result.is_ok(), ".param directive should parse: {:?}", result.err());
        let netlist = result.unwrap();
        assert_eq!(netlist.params.len(), 1);
        assert_eq!(netlist.params[0].name, "Rval");
        assert_eq!(netlist.params[0].value, 1000.0);
    }

    #[test]
    fn test_param_missing_equals() {
        let spice = "Test\n.param Rval\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), ".param without = should be rejected");
    }

    // ===== Op-amp parsing tests =====

    #[test]
    fn test_parse_opamp_basic() {
        let spice = "Test\nU1 3 2 6 opamp\n.model opamp OA(AOL=200000)\n";
        let netlist = Netlist::parse(spice).unwrap();
        match &netlist.elements[0] {
            Element::Opamp { name, n_plus, n_minus, n_out, model } => {
                assert_eq!(name, "U1");
                assert_eq!(n_plus, "3");
                assert_eq!(n_minus, "2");
                assert_eq!(n_out, "6");
                assert_eq!(model, "opamp");
            }
            _ => panic!("Expected Opamp, got {:?}", netlist.elements[0]),
        }
    }

    #[test]
    fn test_parse_opamp_model_params() {
        let spice = "Test\nU1 3 2 6 myoa\n.model myoa OA(AOL=100000 GBW=1e6 ROUT=75)\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.models.len(), 1);
        let model = &netlist.models[0];
        assert_eq!(model.model_type, "OA");
        let aol = model.params.iter().find(|(k, _)| k == "AOL").map(|(_, v)| *v);
        assert_eq!(aol, Some(100_000.0));
        let rout = model.params.iter().find(|(k, _)| k == "ROUT").map(|(_, v)| *v);
        assert_eq!(rout, Some(75.0));
    }

    #[test]
    fn test_parse_opamp_no_model() {
        let spice = "Test\nU1 3 2 6 missing_model\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Op-amp without .model should be rejected");
    }

    #[test]
    fn test_parse_opamp_in_circuit() {
        let spice = r#"Inverting Amplifier
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.elements.len(), 3);
        assert!(matches!(&netlist.elements[2], Element::Opamp { .. }));
    }

    // ======================================================================
    // VCVS (E) and VCCS (G) controlled source tests
    // ======================================================================

    #[test]
    fn test_parse_vcvs_basic() {
        let spice = "Test\nE1 out 0 in 0 10\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.elements.len(), 1);
        match &netlist.elements[0] {
            Element::Vcvs { name, out_p, out_n, ctrl_p, ctrl_n, gain } => {
                assert_eq!(name, "E1");
                assert_eq!(out_p, "out");
                assert_eq!(out_n, "0");
                assert_eq!(ctrl_p, "in");
                assert_eq!(ctrl_n, "0");
                assert_eq!(*gain, 10.0);
            }
            _ => panic!("Expected VCVS"),
        }
    }

    #[test]
    fn test_parse_vccs_basic() {
        let spice = "Test\nG1 out 0 in 0 1m\n";
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.elements.len(), 1);
        match &netlist.elements[0] {
            Element::Vccs { name, out_p, out_n, ctrl_p, ctrl_n, gm } => {
                assert_eq!(name, "G1");
                assert_eq!(out_p, "out");
                assert_eq!(out_n, "0");
                assert_eq!(ctrl_p, "in");
                assert_eq!(ctrl_n, "0");
                assert!((gm - 1e-3).abs() < 1e-12, "Expected 1m = 1e-3, got {}", gm);
            }
            _ => panic!("Expected VCCS"),
        }
    }

    #[test]
    fn test_parse_vcvs_engineering_notation() {
        let spice = "Test\nE1 out 0 in 0 1k\n";
        let netlist = Netlist::parse(spice).unwrap();
        match &netlist.elements[0] {
            Element::Vcvs { gain, .. } => {
                assert_eq!(*gain, 1000.0);
            }
            _ => panic!("Expected VCVS"),
        }
    }

    #[test]
    fn test_parse_vccs_engineering_notation() {
        let spice = "Test\nG1 out 0 in 0 100u\n";
        let netlist = Netlist::parse(spice).unwrap();
        match &netlist.elements[0] {
            Element::Vccs { gm, .. } => {
                assert!((*gm - 100e-6).abs() < 1e-15, "Expected 100u, got {}", gm);
            }
            _ => panic!("Expected VCCS"),
        }
    }

    #[test]
    fn test_parse_vcvs_negative_gain() {
        // Negative gain is valid for VCVS (e.g., inverting amplifier)
        let spice = "Test\nE1 out 0 in 0 -10\n";
        let netlist = Netlist::parse(spice).unwrap();
        match &netlist.elements[0] {
            Element::Vcvs { gain, .. } => {
                assert_eq!(*gain, -10.0);
            }
            _ => panic!("Expected VCVS"),
        }
    }

    #[test]
    fn test_parse_vccs_negative_gm() {
        // Negative gm is valid for VCCS
        let spice = "Test\nG1 out 0 in 0 -0.01\n";
        let netlist = Netlist::parse(spice).unwrap();
        match &netlist.elements[0] {
            Element::Vccs { gm, .. } => {
                assert_eq!(*gm, -0.01);
            }
            _ => panic!("Expected VCCS"),
        }
    }

    #[test]
    fn test_parse_vcvs_zero_gain_rejected() {
        let spice = "Test\nE1 out 0 in 0 0\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Zero VCVS gain should be rejected");
    }

    #[test]
    fn test_parse_vccs_zero_gm_rejected() {
        let spice = "Test\nG1 out 0 in 0 0\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Zero VCCS gm should be rejected");
    }

    #[test]
    fn test_parse_vcvs_missing_nodes() {
        let spice = "Test\nE1 out 0 in\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "VCVS with missing nodes should be rejected");
    }

    #[test]
    fn test_parse_vccs_missing_value() {
        let spice = "Test\nG1 out 0 in 0\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "VCCS with missing gm should be rejected");
    }

    #[test]
    fn test_parse_vcvs_duplicate_name() {
        let spice = "Test\nE1 out 0 in 0 10\nE1 out2 0 in2 0 5\n";
        let result = Netlist::parse(spice);
        assert!(result.is_err(), "Duplicate E1 should be rejected");
    }

    #[test]
    fn test_parse_vcvs_four_nodes() {
        // All four nodes non-ground
        let spice = "Test\nE1 out_p out_n ctrl_p ctrl_n 2.5\n";
        let netlist = Netlist::parse(spice).unwrap();
        match &netlist.elements[0] {
            Element::Vcvs { out_p, out_n, ctrl_p, ctrl_n, gain, .. } => {
                assert_eq!(out_p, "out_p");
                assert_eq!(out_n, "out_n");
                assert_eq!(ctrl_p, "ctrl_p");
                assert_eq!(ctrl_n, "ctrl_n");
                assert_eq!(*gain, 2.5);
            }
            _ => panic!("Expected VCVS"),
        }
    }
}
