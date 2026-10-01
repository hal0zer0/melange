//! Element node collection and categorisation for the MNA builder.

use super::*;

impl MnaBuilder {
    pub(super) fn collect_nodes(&mut self, element: &Element) -> Result<(), MnaError> {
        let nodes = match element {
            Element::Resistor {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Capacitor {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Inductor {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::VoltageSource {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::CurrentSource {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Diode {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Bjt { nc, nb, ne, .. } => vec![nc, nb, ne],
            Element::Jfet { nd, ng, ns, .. } => vec![nd, ng, ns],
            Element::Mosfet { nd, ng, ns, nb, .. } => vec![nd, ng, ns, nb],
            Element::Triode {
                n_grid,
                n_plate,
                n_cathode,
                ..
            } => vec![n_grid, n_plate, n_cathode],
            Element::Pentode {
                n_plate,
                n_grid,
                n_cathode,
                n_screen,
                n_suppressor,
                ..
            } => {
                let mut nodes = vec![n_plate, n_grid, n_cathode, n_screen];
                if let Some(ns) = n_suppressor {
                    nodes.push(ns);
                }
                nodes
            }
            Element::Opamp {
                n_plus,
                n_minus,
                n_out,
                ..
            } => vec![n_plus, n_minus, n_out],
            Element::Vcvs {
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                ..
            } => vec![out_p, out_n, ctrl_p, ctrl_n],
            Element::Vccs {
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                ..
            } => vec![out_p, out_n, ctrl_p, ctrl_n],
            Element::Vca {
                n_sig_p,
                n_sig_n,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => vec![n_sig_p, n_sig_n, n_ctrl_p, n_ctrl_n],
            Element::Ldr {
                n_plus,
                n_minus,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => vec![n_plus, n_minus, n_ctrl_p, n_ctrl_n],
            Element::Glow {
                n_anode, n_cathode, ..
            } => vec![n_anode, n_cathode],
            Element::BSource {
                n_plus,
                n_minus,
                expr,
                ..
            } => {
                let mut nodes = vec![n_plus, n_minus];
                // Register nodes the expression reads so they become circuit
                // nodes even if no other element touches them.
                nodes.extend(expr.referenced_node_refs());
                nodes
            }
            Element::SubcktInstance { name, .. } => {
                return Err(MnaError::TopologyError(format!(
                    "subcircuit instance '{}' not supported (expand subcircuits before MNA)",
                    name
                )));
            }
        };

        for node in nodes {
            if !self.node_map.contains_key(node) {
                self.node_map.insert(node.clone(), self.next_node_idx);
                self.next_node_idx += 1;
            }
        }

        Ok(())
    }

    pub(super) fn categorize_element(&mut self, element: &Element) -> Result<(), MnaError> {
        match element {
            Element::Resistor {
                name,
                n_plus,
                n_minus,
                value,
                ..
            } => {
                self.elements.push(ElementInfo {
                    element_type: ElementType::Resistor,
                    nodes: vec![self.node_map[n_plus], self.node_map[n_minus]],
                    value: *value,
                    name: name.clone(),
                });
            }
            Element::Capacitor {
                name,
                n_plus,
                n_minus,
                value,
                ic,
            } => {
                let node_i = self.node_map[n_plus];
                let node_j = self.node_map[n_minus];
                self.elements.push(ElementInfo {
                    element_type: ElementType::Capacitor,
                    nodes: vec![node_i, node_j],
                    value: *value,
                    name: name.clone(),
                });
                if let Some(ic_val) = ic {
                    self.capacitor_ics.push(CapacitorIcInfo {
                        name: name.clone(),
                        node_i,
                        node_j,
                        ic: *ic_val,
                    });
                }
            }
            Element::Inductor {
                name,
                n_plus,
                n_minus,
                value,
                isat,
                isat_spec,
                air_floor,
                // TURNS=/LM= are checked with the coupled groups.
                turns: _,
                lm: _,
            } => {
                let node_i = self.node_map[n_plus];
                let node_j = self.node_map[n_minus];
                self.elements.push(ElementInfo {
                    element_type: ElementType::Inductor,
                    nodes: vec![node_i, node_j],
                    value: *value,
                    name: name.clone(),
                });
                // Also add to inductors list for DK kernel companion model
                self.inductors.push(InductorElement {
                    name: name.clone(),
                    node_i,
                    node_j,
                    value: *value,
                    // A datasheet rating is converted against this inductor's
                    // own law (k = 1). A winding of a shared core is converted
                    // again, against the core, when the T-model is built.
                    isat: match (*isat, *isat_spec) {
                        (Some(i), Some(spec)) => Some(isat_from_datasheet(
                            name,
                            i,
                            spec,
                            *value,
                            1.0,
                            crate::parser::resolve_air_floor(*air_floor).0,
                        )?),
                        (i, _) => i,
                    },
                    air_floor: *air_floor,
                    shared_core: None,
                });
            }
            Element::VoltageSource {
                name,
                n_plus,
                n_minus,
                dc,
                ..
            } => {
                // Record element for augmented MNA stamping (done after matrix expansion).
                // Do NOT add Norton equivalent conductance here.
                self.elements.push(ElementInfo {
                    element_type: ElementType::VoltageSource,
                    nodes: vec![self.node_map[n_plus], self.node_map[n_minus]],
                    value: dc.unwrap_or(0.0),
                    name: name.clone(),
                });
                // ext_idx = 0-based index within voltage sources (used as offset in augmented rows)
                let ext_idx = self.voltage_sources.len();
                self.voltage_sources.push(VoltageSourceInfo {
                    name: name.clone(),
                    n_plus: n_plus.clone(),
                    n_minus: n_minus.clone(),
                    n_plus_idx: self.node_map[n_plus],
                    n_minus_idx: self.node_map[n_minus],
                    dc_value: dc.unwrap_or(0.0),
                    ext_idx,
                });
            }
            Element::CurrentSource {
                name,
                n_plus,
                n_minus,
                dc,
            } => {
                // SPICE convention: `I n+ n- val` drives `val` amps from n+ to n-
                // *through the source*, i.e. current is extracted from n+ and injected
                // into n- (ngspice: `I1 n1 0 1m` with R to ground gives V(n1) = -1 V).
                // The internal CurrentSourceInfo convention is the opposite — dc_value
                // is injected at n_plus_idx and extracted at n_minus_idx (see the
                // dk.rs/dc_op.rs consumers and the BJT/tube/op-amp companion sources
                // above, all authored to that convention). Bridge SPICE -> internal by
                // negating: extracting `val` from n+ == injecting `-val` at n_plus_idx.
                // Without this negation an independent current source produces output of
                // the opposite sign to ngspice.
                self.current_sources.push(CurrentSourceInfo {
                    name: name.clone(),
                    n_plus_idx: self.node_map[n_plus],
                    n_minus_idx: self.node_map[n_minus],
                    dc_value: -dc.unwrap_or(0.0),
                });
            }
            Element::Diode {
                name,
                n_plus,
                n_minus,
                ..
            } => {
                let node_indices = vec![self.node_map[n_plus], self.node_map[n_minus]];
                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "diode '{}' has both terminals grounded",
                        name
                    )));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // Diode is 1-dimensional
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Diode,
                    dimension: 1,
                    start_idx,
                    nodes: vec![n_plus.clone(), n_minus.clone()],
                    node_indices,
                    vg2k_frozen: 0.0,
                });
            }
            Element::Ldr {
                name,
                n_plus,
                n_minus,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => {
                // node_indices = [r+, r-, ctrl+, ctrl-]. Only the resistance
                // pair (first two) forms the NR dimension; the control pair is
                // carried so codegen can read its converged voltage to drive the
                // after-solve state advance (it draws no current, like a VCA
                // control port). The resistance path must not be fully grounded.
                let rp = self.node_map[n_plus];
                let rn = self.node_map[n_minus];
                if rp == 0 && rn == 0 {
                    return Err(MnaError::TopologyError(format!(
                        "LDR '{}' has both resistance terminals grounded",
                        name
                    )));
                }
                let node_indices = vec![rp, rn, self.node_map[n_ctrl_p], self.node_map[n_ctrl_n]];
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // LDR resistance path is 1-dimensional
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Ldr,
                    dimension: 1,
                    start_idx,
                    nodes: vec![
                        n_plus.clone(),
                        n_minus.clone(),
                        n_ctrl_p.clone(),
                        n_ctrl_n.clone(),
                    ],
                    node_indices,
                    vg2k_frozen: 0.0,
                });
            }
            Element::Glow {
                name,
                n_anode,
                n_cathode,
                ..
            } => {
                // node_indices = [a, k]. 2-terminal resistance path forming one
                // NR dimension (the monotone resistor selected by the frozen
                // latch). Must not be fully grounded.
                let na = self.node_map[n_anode];
                let nk = self.node_map[n_cathode];
                if na == 0 && nk == 0 {
                    return Err(MnaError::TopologyError(format!(
                        "glow lamp '{}' has both terminals grounded",
                        name
                    )));
                }
                let node_indices = vec![na, nk];
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // glow resistance path is 1-dimensional
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Glow,
                    dimension: 1,
                    start_idx,
                    nodes: vec![n_anode.clone(), n_cathode.clone()],
                    node_indices,
                    vg2k_frozen: 0.0,
                });
            }
            Element::Bjt {
                name, nc, nb, ne, ..
            } => {
                let node_indices = vec![self.node_map[nc], self.node_map[nb], self.node_map[ne]];
                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "BJT '{}' has all terminals grounded",
                        name
                    )));
                }
                let is_linearized = self.linearized_bjts.contains(&name.to_ascii_uppercase());
                if is_linearized {
                    // Linearized BJTs are removed from the nonlinear system entirely.
                    // Their small-signal conductances are stamped into G after DC OP.
                    // Don't push to nonlinear_devices, don't increment total_dimension.
                    log::info!(
                        "BJT '{}' linearized at DC OP (removed from NR, M reduced by 2)",
                        name
                    );
                } else {
                    let is_forward_active = self
                        .forward_active_bjts
                        .contains(&name.to_ascii_uppercase());
                    let (device_type, dimension) = if is_forward_active {
                        (NonlinearDeviceType::BjtForwardActive, 1)
                    } else {
                        (NonlinearDeviceType::Bjt, 2)
                    };
                    let start_idx = self.total_dimension;
                    self.total_dimension += dimension;
                    self.nonlinear_devices.push(NonlinearDeviceInfo {
                        name: name.clone(),
                        device_type,
                        dimension,
                        start_idx,
                        nodes: vec![nc.clone(), nb.clone(), ne.clone()],
                        node_indices,
                        vg2k_frozen: 0.0,
                    });
                }
            }
            Element::Jfet {
                name, nd, ng, ns, ..
            } => {
                let node_indices = vec![self.node_map[nd], self.node_map[ng], self.node_map[ns]];
                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "JFET '{}' has all terminals grounded",
                        name
                    )));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 2; // 2D: Id(Vgs,Vds) + Ig(Vgs)
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Jfet,
                    dimension: 2,
                    start_idx,
                    nodes: vec![nd.clone(), ng.clone(), ns.clone()],
                    node_indices,
                    vg2k_frozen: 0.0,
                });
            }
            Element::Mosfet {
                name,
                nd,
                ng,
                ns,
                nb,
                ..
            } => {
                let node_indices = vec![
                    self.node_map[nd],
                    self.node_map[ng],
                    self.node_map[ns],
                    self.node_map[nb],
                ];
                // Check drain/gate/source (not bulk) for all-grounded
                if node_indices[..3].iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "MOSFET '{}' has all terminals grounded",
                        name
                    )));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 2; // 2D: Id(Vgs,Vds) + Ig(=0, insulated gate)
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Mosfet,
                    dimension: 2,
                    start_idx,
                    nodes: vec![nd.clone(), ng.clone(), ns.clone(), nb.clone()],
                    node_indices,
                    vg2k_frozen: 0.0,
                });
            }
            Element::Triode {
                name,
                n_grid,
                n_plate,
                n_cathode,
                ..
            } => {
                let node_indices = vec![
                    self.node_map[n_grid],
                    self.node_map[n_plate],
                    self.node_map[n_cathode],
                ];
                let is_linearized = self.linearized_triodes.contains(&name.to_ascii_uppercase());
                if is_linearized {
                    // Linearized triodes are removed from the nonlinear system entirely.
                    // Their small-signal gm + 1/rp are stamped into G after DC OP.
                    log::info!(
                        "Triode '{}' linearized at DC OP (removed from NR, M reduced by 2)",
                        name
                    );
                } else {
                    let start_idx = self.total_dimension;
                    self.total_dimension += 2; // 2D: plate current + grid current
                    self.nonlinear_devices.push(NonlinearDeviceInfo {
                        name: name.clone(),
                        device_type: NonlinearDeviceType::Tube,
                        dimension: 2,
                        start_idx,
                        nodes: vec![n_grid.clone(), n_plate.clone(), n_cathode.clone()],
                        node_indices,
                        vg2k_frozen: 0.0,
                    });
                }
            }
            Element::Pentode {
                name,
                n_plate,
                n_grid,
                n_cathode,
                n_screen,
                n_suppressor,
                ..
            } => {
                // 3D (sharp-cutoff) contribution: Ip (Vgk, Vpk, Vg2k),
                // Ig2 (same voltages), Ig1 (Vgk-only). Phase 1b adds an
                // optional 2D grid-off reduction when DC-OP confirms Vgk
                // is below cutoff — Ig1 drops out entirely and Vg2k is
                // frozen at its DC-OP value (stored per-slot in
                // `DeviceSlot.vg2k_frozen`).
                //
                // Node layout in `nodes` / `node_indices` is plate-grid-cathode-screen
                // (+ optional suppressor). Downstream MNA stamping code must use
                // this order for N_v / N_i construction in BOTH the 3D and 2D
                // grid-off paths.
                let mut nodes = vec![
                    n_plate.clone(),
                    n_grid.clone(),
                    n_cathode.clone(),
                    n_screen.clone(),
                ];
                let mut node_indices = vec![
                    self.node_map[n_plate],
                    self.node_map[n_grid],
                    self.node_map[n_cathode],
                    self.node_map[n_screen],
                ];
                if let Some(ns) = n_suppressor {
                    // The suppressor (g3) is modelled as tied to the cathode:
                    // no N_v / N_i entries are stamped for it. A suppressor
                    // on any other node would be simulated as cathode-tied,
                    // silently, so that wiring is refused.
                    if self.node_map[ns] != self.node_map[n_cathode] {
                        return Err(MnaError::TopologyError(format!(
                            "pentode '{name}': its suppressor (5th node) is wired to node \
                             '{ns}', not to its cathode node '{n_cathode}'. melange models \
                             the suppressor as tied to the cathode and does not support any \
                             other suppressor wiring. Tie the suppressor to the cathode node, \
                             or omit the 5th node."
                        )));
                    }
                    nodes.push(ns.clone());
                    node_indices.push(self.node_map[ns]);
                }
                let name_upper = name.to_ascii_uppercase();
                let grid_off_entry = self.grid_off_pentodes.get(&name_upper).copied();
                let is_grid_off = grid_off_entry.is_some();
                // Grid-off pentode: 2D NR block (Vgk→Ip, Vpk→Ig2). The
                // device type stays `Tube` — the `TubeKind::SharpPentodeGridOff`
                // discriminator lives on `TubeParams.kind` at the codegen
                // layer, not on `NonlinearDeviceType`.
                let dimension = if is_grid_off { 2 } else { 3 };
                let start_idx = self.total_dimension;
                self.total_dimension += dimension;
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Tube,
                    dimension,
                    start_idx,
                    nodes,
                    node_indices,
                    vg2k_frozen: grid_off_entry.unwrap_or(0.0),
                });
                log::info!(
                    "Pentode '{}' using {} NR block (grid-off = {})",
                    name,
                    if is_grid_off { "2D" } else { "3D" },
                    is_grid_off
                );
            }
            Element::Opamp {
                name,
                n_plus,
                n_minus,
                n_out,
                ..
            } => {
                let np_idx = self.node_map[n_plus];
                let nm_idx = self.node_map[n_minus];
                let no_idx = self.node_map[n_out];

                if no_idx == 0 {
                    return Err(MnaError::TopologyError(format!(
                        "op-amp '{}' has output connected to ground",
                        name
                    )));
                }
                if np_idx == 0 && nm_idx == 0 {
                    return Err(MnaError::TopologyError(format!(
                        "op-amp '{}' has both inputs grounded",
                        name
                    )));
                }

                // Op-amps are LINEAR — do NOT add to nonlinear dimension M
                self.opamps.push(OpampInfo {
                    name: name.clone(),
                    n_plus_idx: np_idx,
                    n_minus_idx: nm_idx,
                    n_out_idx: no_idx,
                    aol: 200_000.0,
                    r_out: OPAMP_DEFAULT_ROUT_OHM,
                    r_sag: OPAMP_DEFAULT_R_SAG_OHM,
                    vcc: f64::INFINITY,
                    vee: f64::NEG_INFINITY,
                    gbw: f64::INFINITY,
                    sr: f64::INFINITY,
                    ib: 0.0,
                    rin: f64::INFINITY,
                    aol_transient_cap: f64::INFINITY,
                    n_internal_idx: 0,
                    iir_c_dom: 0.0,
                    n_int_idx: 0,
                    // Phase 4 noise — opt-in via `.model OA(EN=… IN=…)`.
                    // Defaults of 0.0 mean zero per-source emission and
                    // byte-identical codegen to pre-Phase-4 builds.
                    en: 0.0,
                    in_amps: 0.0,
                });
            }
            Element::Vcvs {
                name,
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                gain,
            } => {
                // VCVS is LINEAR — do NOT add to nonlinear dimension M
                // Stored as ElementInfo for G matrix stamping (Norton equivalent)
                self.elements.push(ElementInfo {
                    element_type: ElementType::Vcvs,
                    nodes: vec![
                        self.node_map[out_p],
                        self.node_map[out_n],
                        self.node_map[ctrl_p],
                        self.node_map[ctrl_n],
                    ],
                    value: *gain,
                    name: name.clone(),
                });
            }
            Element::Vccs {
                name,
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                gm,
            } => {
                // VCCS is LINEAR — do NOT add to nonlinear dimension M
                // Stored as ElementInfo for G matrix stamping
                self.elements.push(ElementInfo {
                    element_type: ElementType::Vccs,
                    nodes: vec![
                        self.node_map[out_p],
                        self.node_map[out_n],
                        self.node_map[ctrl_p],
                        self.node_map[ctrl_n],
                    ],
                    value: *gm,
                    name: name.clone(),
                });
            }
            Element::Vca {
                name,
                n_sig_p,
                n_sig_n,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => {
                let sp = self.node_map[n_sig_p];
                let sn = self.node_map[n_sig_n];
                let cp = self.node_map[n_ctrl_p];
                let cn = self.node_map[n_ctrl_n];

                let node_indices = vec![sp, sn, cp, cn];

                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "VCA '{}' has all terminals grounded",
                        name
                    )));
                }

                let start_idx = self.total_dimension;
                self.total_dimension += 2; // 2D: I_signal(V_sig, V_ctrl) + I_control(=0)

                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Vca,
                    dimension: 2,
                    start_idx,
                    nodes: vec![
                        n_sig_p.clone(),
                        n_sig_n.clone(),
                        n_ctrl_p.clone(),
                        n_ctrl_n.clone(),
                    ],
                    node_indices,
                    vg2k_frozen: 0.0,
                });

                self.vcas.push(VcaInfo {
                    name: name.clone(),
                    n_sig_p_idx: sp,
                    n_sig_n_idx: sn,
                    n_ctrl_p_idx: cp,
                    n_ctrl_n_idx: cn,
                    vscale: 0.05298, // default, resolved from model later
                    g0: 1.0,
                    current_mode: false,
                    n_sense_idx: 0,
                    n_internal_idx: 0,
                });
            }
            Element::SubcktInstance { name, .. } => {
                return Err(MnaError::TopologyError(format!(
                    "subcircuit instance '{}' not supported (expand subcircuits before MNA)",
                    name
                )));
            }
            Element::BSource {
                name,
                n_plus,
                n_minus,
                kind,
                expr,
            } => {
                let n_plus_idx = self.node_map[n_plus];
                let n_minus_idx = self.node_map[n_minus];

                // Assign globally-unique ddt/idt companion-state slots.
                let mut expr = expr.clone();
                expr.assign_state_slots(&mut self.behavioral_state_slots);

                // Resolve every node the expression references to its index.
                let mut referenced_node_indices = std::collections::BTreeMap::new();
                for node in expr.referenced_nodes() {
                    let idx = *self.node_map.get(&node).ok_or_else(|| {
                        MnaError::TopologyError(format!(
                            "behavioral source '{}' references unknown node '{}'",
                            name, node
                        ))
                    })?;
                    referenced_node_indices.insert(node, idx);
                }

                // For V={} sources, reserve a behavioral-VS index (used to
                // allocate the augmented branch-current row at codegen time).
                let v_ext_idx = match kind {
                    crate::parser::BSourceKind::Voltage => Some(
                        self.behavioral_sources
                            .iter()
                            .filter(|b| b.v_ext_idx.is_some())
                            .count(),
                    ),
                    crate::parser::BSourceKind::Current => None,
                };

                self.behavioral_sources.push(BehavioralSourceInfo {
                    name: name.clone(),
                    kind: *kind,
                    n_plus: n_plus.clone(),
                    n_minus: n_minus.clone(),
                    n_plus_idx,
                    n_minus_idx,
                    referenced_node_indices,
                    expr,
                    v_ext_idx,
                    aug_row: None,
                });
            }
        }

        Ok(())
    }
}
