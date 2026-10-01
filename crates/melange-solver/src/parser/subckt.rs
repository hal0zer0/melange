//! Subcircuit expansion, cycle detection and nesting depth.

use super::*;

impl Netlist {
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
    /// - Nesting depth exceeds 8 levels
    /// - Cumulative element count exceeds 10,000
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

        // Nesting depth limit
        const MAX_NESTING_DEPTH: usize = 8;
        let depth = max_subcircuit_depth(&self.subcircuits, &lookup);
        if depth > MAX_NESTING_DEPTH {
            return Err(ParseError {
                line: 0,
                message: format!(
                    "Subcircuit nesting depth {} exceeds maximum of {}",
                    depth, MAX_NESTING_DEPTH
                ),
            });
        }

        // Iterative expansion (max 32 passes for deeply nested subcircuits).
        // Track cumulative element count across all passes to prevent
        // deeply nested subcircuits from exceeding the limit incrementally.
        const MAX_ELEMENTS: usize = 10_000;
        let mut cumulative_elements: usize = 0;

        for _pass in 0..32 {
            let has_instances = self
                .elements
                .iter()
                .any(|e| matches!(e, Element::SubcktInstance { .. }));
            if !has_instances {
                break;
            }

            let mut new_elements = Vec::with_capacity(self.elements.len());
            let mut elements_added_this_pass: usize = 0;
            for elem in &self.elements {
                if let Element::SubcktInstance {
                    name: inst_name,
                    nodes: inst_nodes,
                    subckt,
                } = elem
                {
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
                                inst_name,
                                inst_nodes.len(),
                                subckt,
                                sc.nodes.len()
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
                        elements_added_this_pass += 1;
                    }
                } else {
                    new_elements.push(elem.clone());
                }
            }
            self.elements = new_elements;
            cumulative_elements += elements_added_this_pass;

            // Element count limit: prevent combinatorial explosion from nested subcircuits.
            // Check both the current element count and the cumulative count across all passes.
            if self.elements.len() > MAX_ELEMENTS || cumulative_elements > MAX_ELEMENTS {
                return Err(ParseError {
                    line: 0,
                    message: format!(
                        "Element count exceeds limit of {} after subcircuit expansion \
                         (current: {}, cumulative elements created: {})",
                        MAX_ELEMENTS,
                        self.elements.len(),
                        cumulative_elements
                    ),
                });
            }
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

impl Element {
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
        let prefixed = |name: &str| -> String { format!("{}.{}", prefix, name) };

        match self {
            Element::Resistor {
                name,
                n_plus,
                n_minus,
                value,
                kf,
                af,
            } => Element::Resistor {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                value: *value,
                kf: *kf,
                af: *af,
            },
            Element::Capacitor {
                name,
                n_plus,
                n_minus,
                value,
                ic,
            } => Element::Capacitor {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                value: *value,
                ic: *ic,
            },
            Element::Inductor {
                name,
                n_plus,
                n_minus,
                value,
                isat,
                isat_spec,
                air_floor,
                turns,
                lm,
            } => Element::Inductor {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                value: *value,
                isat: *isat,
                isat_spec: *isat_spec,
                air_floor: *air_floor,
                turns: *turns,
                lm: *lm,
            },
            Element::VoltageSource {
                name,
                n_plus,
                n_minus,
                dc,
                ac,
            } => Element::VoltageSource {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                dc: *dc,
                ac: *ac,
            },
            Element::CurrentSource {
                name,
                n_plus,
                n_minus,
                dc,
            } => Element::CurrentSource {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                dc: *dc,
            },
            Element::Diode {
                name,
                n_plus,
                n_minus,
                model,
            } => Element::Diode {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                model: model.clone(),
            },
            Element::Bjt {
                name,
                nc,
                nb,
                ne,
                model,
            } => Element::Bjt {
                name: prefixed(name),
                nc: remap(nc),
                nb: remap(nb),
                ne: remap(ne),
                model: model.clone(),
            },
            Element::Jfet {
                name,
                nd,
                ng,
                ns,
                model,
            } => Element::Jfet {
                name: prefixed(name),
                nd: remap(nd),
                ng: remap(ng),
                ns: remap(ns),
                model: model.clone(),
            },
            Element::Mosfet {
                name,
                nd,
                ng,
                ns,
                nb,
                model,
            } => Element::Mosfet {
                name: prefixed(name),
                nd: remap(nd),
                ng: remap(ng),
                ns: remap(ns),
                nb: remap(nb),
                model: model.clone(),
            },
            Element::Opamp {
                name,
                n_plus,
                n_minus,
                n_out,
                model,
            } => Element::Opamp {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                n_out: remap(n_out),
                model: model.clone(),
            },
            Element::Triode {
                name,
                n_grid,
                n_plate,
                n_cathode,
                model,
            } => Element::Triode {
                name: prefixed(name),
                n_grid: remap(n_grid),
                n_plate: remap(n_plate),
                n_cathode: remap(n_cathode),
                model: model.clone(),
            },
            Element::Pentode {
                name,
                n_plate,
                n_grid,
                n_cathode,
                n_screen,
                n_suppressor,
                model,
            } => Element::Pentode {
                name: prefixed(name),
                n_plate: remap(n_plate),
                n_grid: remap(n_grid),
                n_cathode: remap(n_cathode),
                n_screen: remap(n_screen),
                n_suppressor: n_suppressor.as_ref().map(|n| remap(n)),
                model: model.clone(),
            },
            Element::Vca {
                name,
                n_sig_p,
                n_sig_n,
                n_ctrl_p,
                n_ctrl_n,
                model,
            } => Element::Vca {
                name: prefixed(name),
                n_sig_p: remap(n_sig_p),
                n_sig_n: remap(n_sig_n),
                n_ctrl_p: remap(n_ctrl_p),
                n_ctrl_n: remap(n_ctrl_n),
                model: model.clone(),
            },
            Element::Ldr {
                name,
                n_plus,
                n_minus,
                n_ctrl_p,
                n_ctrl_n,
                model,
            } => Element::Ldr {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                n_ctrl_p: remap(n_ctrl_p),
                n_ctrl_n: remap(n_ctrl_n),
                model: model.clone(),
            },
            Element::Glow {
                name,
                n_anode,
                n_cathode,
                model,
            } => Element::Glow {
                name: prefixed(name),
                n_anode: remap(n_anode),
                n_cathode: remap(n_cathode),
                model: model.clone(),
            },
            Element::Vcvs {
                name,
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                gain,
            } => Element::Vcvs {
                name: prefixed(name),
                out_p: remap(out_p),
                out_n: remap(out_n),
                ctrl_p: remap(ctrl_p),
                ctrl_n: remap(ctrl_n),
                gain: *gain,
            },
            Element::Vccs {
                name,
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                gm,
            } => Element::Vccs {
                name: prefixed(name),
                out_p: remap(out_p),
                out_n: remap(out_n),
                ctrl_p: remap(ctrl_p),
                ctrl_n: remap(ctrl_n),
                gm: *gm,
            },
            Element::SubcktInstance {
                name,
                nodes,
                subckt,
            } => Element::SubcktInstance {
                name: prefixed(name),
                nodes: nodes.iter().map(|n| remap(n)).collect(),
                subckt: subckt.clone(),
            },
            Element::BSource {
                name,
                n_plus,
                n_minus,
                kind,
                expr,
            } => Element::BSource {
                name: prefixed(name),
                n_plus: remap(n_plus),
                n_minus: remap(n_minus),
                kind: *kind,
                // Remap node/branch identifiers referenced inside the
                // expression too — they live in the same naming scope as
                // the source's terminals.
                expr: expr.remap_idents(&remap),
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

/// Compute the maximum nesting depth of subcircuit definitions.
///
/// A subcircuit with no nested `X` instances has depth 1.
/// A subcircuit that instantiates another subcircuit of depth D has depth D+1.
/// Returns the maximum depth across all subcircuits, or 0 if there are none.
///
/// Assumes no cycles (call `detect_subcircuit_cycles` first).
fn max_subcircuit_depth(
    subcircuits: &[Subcircuit],
    lookup: &std::collections::HashMap<String, usize>,
) -> usize {
    let n = subcircuits.len();
    if n == 0 {
        return 0;
    }

    // Pre-compute children for each subcircuit (indices of subcircuits it instantiates)
    let children_of: Vec<Vec<usize>> = subcircuits
        .iter()
        .map(|sc| {
            sc.elements
                .iter()
                .filter_map(|e| {
                    if let Element::SubcktInstance { subckt, .. } = e {
                        lookup.get(&subckt.to_ascii_lowercase()).copied()
                    } else {
                        None
                    }
                })
                .collect()
        })
        .collect();

    // Memoized iterative DFS depth computation
    let mut depth: Vec<Option<usize>> = vec![None; n];

    for start in 0..n {
        if depth[start].is_some() {
            continue;
        }
        // Work stack: (subckt_idx, child_cursor)
        let mut stack: Vec<(usize, usize)> = vec![(start, 0)];
        while let Some(&mut (node, ref mut cursor)) = stack.last_mut() {
            let children = &children_of[node];

            // Advance cursor past already-resolved children
            let mut pushed = false;
            while *cursor < children.len() {
                let child = children[*cursor];
                *cursor += 1;
                if depth[child].is_none() {
                    stack.push((child, 0));
                    pushed = true;
                    break;
                }
            }

            if !pushed {
                // All children resolved — compute this node's depth
                let max_child_depth = children.iter().filter_map(|&c| depth[c]).max().unwrap_or(0);
                depth[node] = Some(max_child_depth + 1);
                stack.pop();
            }
        }
    }

    depth.iter().filter_map(|d| *d).max().unwrap_or(0)
}
