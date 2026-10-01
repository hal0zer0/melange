//! The netlist-to-MNA builder and the `MnaSystem::from_netlist*` entry points.

use super::*;

mod elements;

impl MnaSystem {
    /// Assemble MNA matrices from a netlist.
    pub fn from_netlist(netlist: &Netlist) -> Result<Self, MnaError> {
        let builder = MnaBuilder::new();
        builder.build(netlist)
    }

    /// Build MNA system with specified BJTs using forward-active (1D) model.
    ///
    /// BJTs whose names are in `forward_active` are modeled as 1D (Vbe→Ic only),
    /// reducing M by 1 per forward-active BJT. Used after DC OP analysis confirms
    /// Vbc is deeply reverse-biased.
    pub fn from_netlist_forward_active(
        netlist: &Netlist,
        forward_active: &std::collections::HashSet<String>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.forward_active_bjts = forward_active.clone();
        builder.build(netlist)
    }

    /// Build MNA system with specified pentodes using grid-off (2D) reduction.
    ///
    /// Pentodes whose names appear as keys in `grid_off` are modeled as 2D
    /// (Vgk → Ip, Vpk → Ig2, Ig1 dropped, Vg2k frozen at the map value),
    /// reducing M by 1 per grid-off pentode. Used after DC-OP analysis
    /// confirms Vgk is below cutoff across the operating point — the
    /// per-pentode map value is the DC-OP-converged Vg2k to freeze.
    ///
    /// Mirrors `from_netlist_forward_active` for BJTs, but with a map
    /// rather than a set so the per-slot `vg2k_frozen` field on
    /// [`NonlinearDeviceInfo`] can be populated in one pass.
    pub fn from_netlist_with_grid_off(
        netlist: &Netlist,
        grid_off: &std::collections::HashMap<String, f64>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.grid_off_pentodes = grid_off.clone();
        builder.build(netlist)
    }

    /// Build MNA system with both BJT forward-active and pentode grid-off reductions.
    ///
    /// Used by circuits (e.g. a guitar amp with a BJT-biased front end and a
    /// pentode power stage) that simultaneously benefit
    /// from FA-reduced biasing BJTs and grid-off-reduced power pentodes.
    pub fn from_netlist_with_grid_off_and_fa(
        netlist: &Netlist,
        forward_active: &std::collections::HashSet<String>,
        grid_off: &std::collections::HashMap<String, f64>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.forward_active_bjts = forward_active.clone();
        builder.grid_off_pentodes = grid_off.clone();
        builder.build(netlist)
    }

    /// Build MNA system with all device reductions applied simultaneously.
    ///
    /// Combines forward-active BJTs, linearized BJTs, linearized triodes,
    /// and grid-off pentodes in a single rebuild. Use this when a circuit
    /// benefits from multiple reduction types (e.g. triode cascade with
    /// FA-reduced biasing BJTs and linearized clean stages).
    pub fn from_netlist_with_all_reductions(
        netlist: &Netlist,
        forward_active: &std::collections::HashSet<String>,
        linearized_bjts: &std::collections::HashSet<String>,
        linearized_triodes: &std::collections::HashSet<String>,
        grid_off: &std::collections::HashMap<String, f64>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.forward_active_bjts = forward_active.clone();
        builder.linearized_bjts = linearized_bjts.clone();
        builder.linearized_triodes = linearized_triodes.clone();
        builder.grid_off_pentodes = grid_off.clone();
        builder.build(netlist)
    }
}

/// Builder for MNA systems.
struct MnaBuilder {
    node_map: HashMap<String, usize>,
    next_node_idx: usize,
    nonlinear_devices: Vec<NonlinearDeviceInfo>,
    voltage_sources: Vec<VoltageSourceInfo>,
    behavioral_sources: Vec<BehavioralSourceInfo>,
    /// Running global counter for `ddt`/`idt` companion-state slot ids across
    /// all behavioral sources.
    behavioral_state_slots: usize,
    current_sources: Vec<CurrentSourceInfo>,
    capacitor_ics: Vec<CapacitorIcInfo>,
    inductors: Vec<InductorElement>,
    opamps: Vec<OpampInfo>,
    vcas: Vec<VcaInfo>,
    elements: Vec<ElementInfo>,
    /// Total voltage dimension accumulated so far
    total_dimension: usize,
    /// BJT names to model as forward-active (1D instead of 2D)
    forward_active_bjts: std::collections::HashSet<String>,
    /// BJT names to linearize at DC OP (removed from nonlinear system entirely)
    linearized_bjts: std::collections::HashSet<String>,
    /// Triode names to linearize at DC OP (removed from nonlinear system entirely, M-2 each)
    linearized_triodes: std::collections::HashSet<String>,
    /// Pentode names to model with grid-off reduction (2D instead of 3D).
    /// Phase 1b: when DC-OP confirms Vgk is below cutoff, drop the Ig1 NR
    /// dimension and freeze Vg2k at its DC-OP value (stored per-slot in
    /// `DeviceSlot.vg2k_frozen`).
    /// Phase 1b grid-off pentode map: name → frozen Vg2k value.
    /// The key set acts as the "is this pentode grid-off?" discriminator
    /// during `categorize_element`; the per-entry value is written into
    /// the resulting `NonlinearDeviceInfo.vg2k_frozen` so downstream
    /// codegen can emit it as a per-slot constant.
    grid_off_pentodes: std::collections::HashMap<String, f64>,
}

struct ElementInfo {
    element_type: ElementType,
    nodes: Vec<usize>,
    value: f64,
    name: String,
}

#[derive(Debug, Clone, Copy)]
enum ElementType {
    Resistor,
    Capacitor,
    Inductor,
    VoltageSource,
    Vcvs,
    Vccs,
}

impl MnaBuilder {
    fn new() -> Self {
        let mut node_map = HashMap::new();
        node_map.insert("0".to_string(), 0); // Ground is always node 0

        Self {
            node_map,
            next_node_idx: 1,
            nonlinear_devices: Vec::new(),
            voltage_sources: Vec::new(),
            behavioral_sources: Vec::new(),
            behavioral_state_slots: 0,
            current_sources: Vec::new(),
            capacitor_ics: Vec::new(),
            inductors: Vec::new(),
            opamps: Vec::new(),
            vcas: Vec::new(),
            elements: Vec::new(),
            total_dimension: 0,
            forward_active_bjts: std::collections::HashSet::new(),
            linearized_bjts: std::collections::HashSet::new(),
            linearized_triodes: std::collections::HashSet::new(),
            grid_off_pentodes: std::collections::HashMap::new(),
        }
    }

    fn build(mut self, netlist: &Netlist) -> Result<MnaSystem, MnaError> {
        // First pass: collect all node names and assign indices
        for element in &netlist.elements {
            self.collect_nodes(element)?;
        }

        // Second pass: categorize elements and build device info
        for element in &netlist.elements {
            self.categorize_element(element)?;
        }

        // Resolve op-amp model parameters from netlist .model directives
        for (oa, elem) in self.opamps.iter_mut().zip(
            netlist
                .elements
                .iter()
                .filter(|e| matches!(e, Element::Opamp { .. })),
        ) {
            if let Element::Opamp { model, .. } = elem {
                if let Some(m) = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model))
                {
                    if m.model_type != "OA" {
                        return Err(MnaError::InvalidParameter(format!(
                            "Op-amp {} references model '{}' with type '{}', expected 'OA'",
                            model, m.name, m.model_type
                        )));
                    }
                    let mut swing = OpampSwingCard::default();
                    for (key, val) in &m.params {
                        match key.to_ascii_uppercase().as_str() {
                            "AOL" => oa.aol = *val,
                            "ROUT" => oa.r_out = *val,
                            "R_SAG" => oa.r_sag = *val,
                            "VSAT" => swing.vsat = Some(*val),
                            "VCC" => swing.vcc = Some(*val),
                            "VEE" => swing.vee = Some(*val),
                            "GBW" => oa.gbw = *val,
                            // SR is specified in V/μs (SPICE convention) and
                            // stored in V/s internally — multiply by 1e6.
                            "SR" => oa.sr = *val * 1.0e6,
                            "VOH_DROP" => swing.voh_drop = Some(*val),
                            "VOL_DROP" => swing.vol_drop = Some(*val),
                            "AOL_TRANSIENT_CAP" => oa.aol_transient_cap = *val,
                            "IB" => oa.ib = *val,
                            "RIN" => oa.rin = *val,
                            // Phase 4 input-referred noise (datasheet en/in).
                            // The 1/f corners EN_FC/IN_FC are on the op-amp
                            // `unimplemented` list: accepted with a notice.
                            "EN" => oa.en = *val,
                            "IN" => oa.in_amps = *val,
                            // Accepted-key set lives in `model_params` so this
                            // arm, the codegen resolvers and the orphan-card
                            // pass in the parser cannot drift apart. This is the
                            // warning for commands that stop at the MNA
                            // (`nodes`); code generation refuses an unknown key
                            // and gives the notice for an unimplemented one
                            // (`build_device_info_with_mna`).
                            _ => crate::model_params::warn_if_unknown(
                                &m.name,
                                crate::model_params::ModelClass::Opamp,
                                key,
                            ),
                        }
                    }
                    for (key, value) in [("ROUT", oa.r_out), ("R_SAG", oa.r_sag)] {
                        if !(value.is_finite() && value > 0.0) {
                            return Err(MnaError::InvalidParameter(format!(
                                "Op-amp {}: {key} must be positive and finite, got {value}",
                                oa.name
                            )));
                        }
                    }
                    let resolved = resolve_opamp_swing(&oa.name, &swing, oa.gbw.is_finite())
                        .map_err(MnaError::InvalidParameter)?;
                    for notice in &resolved.notices {
                        crate::diag_warn!("{notice}");
                    }
                    oa.vcc = resolved.high;
                    oa.vee = resolved.low;
                }
            }
        }

        // GBW is parsed but no bandwidth pole reaches any solver path (the IIR
        // dominant-pole model was removed as dead code in e91e4ed), so the
        // gain is AOL at every frequency. Its one live effect is defaulting
        // the rails below. Say so: the card reads as a pole.
        let gbw_named: Vec<&str> = self
            .opamps
            .iter()
            .filter(|oa| oa.gbw.is_finite())
            .map(|oa| oa.name.as_str())
            .collect();
        if !gbw_named.is_empty() {
            crate::diag_warn!(
                "Op-amp {}: GBW is not modelled as a bandwidth pole (the gain is AOL at \
                 every frequency). It only sets the default +/-13 V rails when VCC, VEE \
                 and VSAT are absent.",
                gbw_named.join(", ")
            );
        }

        // Resolve VCA model parameters from netlist .model directives
        for (vca, elem) in self.vcas.iter_mut().zip(
            netlist
                .elements
                .iter()
                .filter(|e| matches!(e, Element::Vca { .. })),
        ) {
            if let Element::Vca { model, .. } = elem {
                if let Some(m) = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model))
                {
                    if m.model_type != "VCA" {
                        return Err(MnaError::InvalidParameter(format!(
                            "VCA {} references model '{}' with type '{}', expected 'VCA'",
                            model, m.name, m.model_type
                        )));
                    }
                    for (key, val) in &m.params {
                        match key.to_ascii_uppercase().as_str() {
                            "VSCALE" => vca.vscale = *val,
                            "G0" => vca.g0 = *val,
                            "MODE" => vca.current_mode = *val != 0.0,
                            // `THD` is read by the codegen VCA resolver, not
                            // here — warning against this match alone reported
                            // an honored parameter as unrecognized. The key set
                            // now comes from `model_params`.
                            _ => crate::model_params::warn_if_unknown(
                                &m.name,
                                crate::model_params::ModelClass::Vca,
                                key,
                            ),
                        }
                    }
                }
            }
        }

        // Create MNA system with correct dimensions
        let n = self.next_node_idx - 1; // Exclude ground
        if n > MAX_N {
            return Err(MnaError::TopologyError(format!(
                "Circuit has {} nodes, exceeding MAX_N={}",
                n, MAX_N
            )));
        }
        let m = self.total_dimension;
        let num_devices = self.nonlinear_devices.len();
        let num_vs = self.voltage_sources.len();
        let mut mna = MnaSystem::new(n, m, num_devices, num_vs);

        // Resolve pot directives before moving node_map
        for pot_dir in &netlist.pots {
            let resistor = netlist.elements.iter().find(|e| {
                matches!(e, Element::Resistor { name, .. } if name.eq_ignore_ascii_case(&pot_dir.resistor_name))
            });
            if let Some(Element::Resistor {
                n_plus,
                n_minus,
                value,
                ..
            }) = resistor
            {
                let node_p = self.node_map[n_plus];
                let node_q = self.node_map[n_minus];
                let grounded = node_p == 0 || node_q == 0;
                // Use pot default if specified, otherwise fall back to component value
                let nominal_r = pot_dir.default_value.unwrap_or(*value);
                mna.pots.push(PotInfo {
                    name: pot_dir.resistor_name.clone(),
                    node_p,
                    node_q,
                    g_nominal: 1.0 / nominal_r,
                    min_resistance: pot_dir.min_value,
                    max_resistance: pot_dir.max_value,
                    grounded,
                    runtime_field: None,
                });
            } else {
                return Err(MnaError::TopologyError(format!(
                    ".pot references resistor '{}' which was not found",
                    pot_dir.resistor_name
                )));
            }
        }

        // Resolve .runtime R directives. These share the pot table so
        // rebuild_matrices / DK-kernel / nodal-emitter machinery applies
        // unchanged; `runtime_field = Some(...)` tells downstream codegen
        // to emit `set_runtime_R_<field>` (with a read-only accessor)
        // rather than `set_pot_N`. Setter bodies are otherwise identical
        // since the 2026-04-20 reseed strip.
        for rr_dir in &netlist.runtime_resistors {
            let resistor = netlist.elements.iter().find(|e| {
                matches!(e, Element::Resistor { name, .. } if name.eq_ignore_ascii_case(&rr_dir.resistor_name))
            });
            if let Some(Element::Resistor {
                n_plus,
                n_minus,
                value,
                ..
            }) = resistor
            {
                let node_p = self.node_map[n_plus];
                let node_q = self.node_map[n_minus];
                let grounded = node_p == 0 || node_q == 0;
                mna.pots.push(PotInfo {
                    name: rr_dir.resistor_name.clone(),
                    node_p,
                    node_q,
                    g_nominal: 1.0 / *value,
                    min_resistance: rr_dir.min_value,
                    max_resistance: rr_dir.max_value,
                    grounded,
                    runtime_field: Some(rr_dir.field_name.clone()),
                });
            } else {
                return Err(MnaError::TopologyError(format!(
                    ".runtime R references resistor '{}' which was not found",
                    rr_dir.resistor_name
                )));
            }
        }

        // Build pot default override map: resistor name → default resistance.
        // When a .pot has a default value that differs from the component value,
        // the G matrix should be stamped at the pot default, not the component value.
        let pot_default_overrides: std::collections::HashMap<String, f64> = netlist
            .pots
            .iter()
            .filter_map(|p| {
                p.default_value
                    .map(|dv| (p.resistor_name.to_ascii_uppercase(), dv))
            })
            .collect();
        mna.pot_default_overrides = pot_default_overrides;

        // Resolve wiper group directives: find the two pot indices for each wiper
        for wiper_dir in &netlist.wipers {
            let cw_idx = mna
                .pots
                .iter()
                .position(|p| p.name.eq_ignore_ascii_case(&wiper_dir.resistor_cw));
            let ccw_idx = mna
                .pots
                .iter()
                .position(|p| p.name.eq_ignore_ascii_case(&wiper_dir.resistor_ccw));
            if let (Some(cw), Some(ccw)) = (cw_idx, ccw_idx) {
                mna.wiper_groups.push(WiperGroupInfo {
                    cw_pot_index: cw,
                    ccw_pot_index: ccw,
                    total_resistance: wiper_dir.total_resistance,
                    default_position: wiper_dir.default_position.unwrap_or(0.5),
                    label: wiper_dir.label.clone(),
                });
            }
            // If not found, the pot validation already caught it
        }

        // Resolve gang directives: link pot/wiper indices
        for gang_dir in &netlist.gangs {
            let mut pot_members = Vec::new();
            let mut wiper_members = Vec::new();

            for member in &gang_dir.members {
                let name_upper = member.resistor_name.to_ascii_uppercase();

                // Check if this member is a pot
                if let Some(pot_idx) = mna
                    .pots
                    .iter()
                    .position(|p| p.name.eq_ignore_ascii_case(&name_upper))
                {
                    // Check if this pot belongs to a wiper group
                    let in_wiper =
                        mna.wiper_groups.iter().enumerate().find(|(_, wg)| {
                            wg.cw_pot_index == pot_idx || wg.ccw_pot_index == pot_idx
                        });
                    if let Some((wg_idx, _)) = in_wiper {
                        // This is a wiper member — add the wiper group (avoid duplicates)
                        if !wiper_members
                            .iter()
                            .any(|&(idx, _): &(usize, bool)| idx == wg_idx)
                        {
                            wiper_members.push((wg_idx, member.inverted));
                        }
                    } else {
                        // This is a standalone pot member
                        pot_members.push((pot_idx, member.inverted));
                    }
                }
                // If not found in pots, it might be a wiper resistor name that wasn't
                // expanded. The parser validation already caught missing references.
            }

            mna.gang_groups.push(GangGroupInfo {
                label: gang_dir.label.clone(),
                pot_members,
                wiper_members,
                default_position: gang_dir.default_position.unwrap_or(0.5),
            });
        }

        // Resolve switch directives.
        //
        // The initial state of a switch is always position 0, so G/C/L are stamped
        // at the position-0 value (the "canonical baseline") rather than the static
        // netlist declaration. This keeps the initial state self-consistent: the
        // default matrices reflect the default position, and `set_switch_N(non-zero)`
        // applies the correct incremental delta from the pos-0 baseline.
        //
        // Circuit authors who write `R_gain oa_neg 0 10k` but then list `43k 10k ...`
        // as the switch positions no longer need the declaration to match pos-0 — we
        // log a note so the mismatch is visible, but the simulation behaves correctly.
        for sw_dir in &netlist.switches {
            let mut components = Vec::new();
            for (ci, comp_name) in sw_dir.component_names.iter().enumerate() {
                // For expanded subcircuit names like "X1.C1", use base name after last dot
                let base = comp_name.rsplit('.').next().unwrap_or(comp_name);
                let first_char = base.chars().next().unwrap_or(' ').to_ascii_uppercase();
                let pos_0 = sw_dir.positions[0][ci];
                let (node_p, node_q, static_value) = match first_char {
                    'R' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Resistor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Resistor {
                            n_plus,
                            n_minus,
                            value,
                            ..
                        }) = elem
                        {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found",
                                comp_name
                            )));
                        }
                    }
                    'C' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Capacitor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Capacitor {
                            n_plus,
                            n_minus,
                            value,
                            ..
                        }) = elem
                        {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found",
                                comp_name
                            )));
                        }
                    }
                    'L' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Inductor {
                            n_plus,
                            n_minus,
                            value,
                            ..
                        }) = elem
                        {
                            if let Some(sat) = saturating_core_member(netlist, comp_name) {
                                return Err(MnaError::TopologyError(format!(
                                    ".switch cannot change {comp_name}: {sat}. Its flux law \
                                     (L0, ISAT, air floor) is fixed at compile time, so a \
                                     switched value would solve a different device than the \
                                     one named."
                                )));
                            }
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found",
                                comp_name
                            )));
                        }
                    }
                    _ => {
                        return Err(MnaError::TopologyError(format!(
                            ".switch component '{}' must start with R, C, or L",
                            comp_name
                        )));
                    }
                };
                if (static_value - pos_0).abs() > static_value.abs() * 1e-12 {
                    log::info!(
                        ".switch {}: component '{}' declared at {:.6e} but pos-0 is {:.6e}; \
                         stamping pos-0 as the initial value",
                        comp_name,
                        comp_name,
                        static_value,
                        pos_0,
                    );
                }
                mna.switch_default_overrides
                    .insert(comp_name.to_ascii_uppercase(), (first_char, pos_0));
                components.push(SwitchComponentInfo {
                    name: comp_name.clone(),
                    component_type: first_char,
                    node_p,
                    node_q,
                    nominal_value: pos_0,
                });
            }
            mna.switches.push(SwitchInfo {
                components,
                positions: sw_dir.positions.clone(),
                label: sw_dir.label.clone(),
            });
        }

        // Apply switch pos-0 overrides to already-collected uncoupled inductor values.
        // `self.inductors` was populated during `categorize_element` (before switch
        // resolution), so its `value` field still holds the static netlist declaration.
        // `build_augmented_matrices` and the DK companion model read this directly, so
        // we normalize it here to keep the augmented MNA consistent with G/C.
        for ind in &mut self.inductors {
            if let Some((_, pos_0)) = mna
                .switch_default_overrides
                .get(&ind.name.to_ascii_uppercase())
            {
                ind.value = *pos_0;
            }
        }

        // Resolve coupling (K) directives: group inductors into transformer groups.
        // 2-winding pairs (inductors that appear in exactly one K directive each)
        // use the existing CoupledInductorInfo path. Multi-winding groups (3+
        // inductors connected by multiple K directives) use TransformerGroupInfo.
        let mut coupled_inductor_names = std::collections::HashSet::new();

        // TURNS=/LM= state a winding of a saturating shared core, so the
        // inductor must be K-coupled.
        for e in &netlist.elements {
            if let Element::Inductor {
                name, turns, lm, ..
            } = e
            {
                let coupled = netlist.couplings.iter().any(|c| {
                    c.inductor1_name.eq_ignore_ascii_case(name)
                        || c.inductor2_name.eq_ignore_ascii_case(name)
                });
                if (turns.is_some() || lm.is_some()) && !coupled {
                    return Err(MnaError::TopologyError(format!(
                        "{name}: TURNS= and LM= state a winding of a saturating shared core, \
                         but {name} is not K-coupled to anything."
                    )));
                }
            }
        }

        // Collect all inductor info referenced by K directives
        struct InductorRef {
            name: String,
            node_i: usize,
            node_j: usize,
            value: f64,
            isat: Option<f64>,
            isat_spec: Option<crate::parser::IsatSpec>,
            air_floor: Option<crate::parser::SatFloor>,
            turns: Option<f64>,
            lm: Option<f64>,
        }
        let mut inductor_refs: std::collections::HashMap<String, InductorRef> =
            std::collections::HashMap::new();
        for coupling in &netlist.couplings {
            for ind_name in [&coupling.inductor1_name, &coupling.inductor2_name] {
                let lower = ind_name.to_ascii_lowercase();
                if inductor_refs.contains_key(&lower) {
                    continue;
                }
                if let Some(Element::Inductor { name, n_plus, n_minus, value, isat, isat_spec, air_floor, turns, lm }) =
                    netlist.elements.iter().find(|e| {
                        matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(ind_name))
                    })
                {
                    // Apply switch pos-0 override so coupled-inductor paths
                    // (CoupledInductorInfo, TransformerGroupInfo, ideal-transformer
                    // decomposition) use the same initial L as the augmented C matrix.
                    let effective_value = mna
                        .switch_default_overrides
                        .get(&name.to_ascii_uppercase())
                        .filter(|(kind, _)| *kind == 'L')
                        .map(|(_, v)| *v)
                        .unwrap_or(*value);
                    inductor_refs.insert(lower, InductorRef {
                        name: name.clone(),
                        node_i: self.node_map[n_plus],
                        node_j: self.node_map[n_minus],
                        value: effective_value,
                        isat: *isat,
                        isat_spec: *isat_spec,
                        air_floor: *air_floor,
                        turns: *turns,
                        lm: *lm,
                    });
                }
            }
        }

        // Netlist-appearance position of an inductor (lowercased name). Used as
        // the deterministic ordering key everywhere below — HashMap key/iteration
        // order is randomized per process, so anything derived from raw HashMap
        // order would permute the emitted matrices build-to-build for the same
        // netlist. `usize::MAX` for names with no matching element (shouldn't
        // happen; keeps the sort total).
        let ind_pos = |m: &str| -> usize {
            netlist
                .elements
                .iter()
                .position(|e| matches!(e, Element::Inductor { name, .. } if name.to_ascii_lowercase() == m))
                .unwrap_or(usize::MAX)
        };

        // Build a graph of inductor connections via K directives (union-find).
        // Sorted by netlist position so union roots and grouping are stable.
        let mut ind_names: Vec<String> = inductor_refs.keys().cloned().collect();
        ind_names.sort_by_key(|m| ind_pos(m));
        let mut parent: std::collections::HashMap<String, String> =
            ind_names.iter().map(|n| (n.clone(), n.clone())).collect();
        fn find(parent: &mut std::collections::HashMap<String, String>, x: &str) -> String {
            let p = parent[x].clone();
            if p == x {
                return p;
            }
            let root = find(parent, &p);
            parent.insert(x.to_string(), root.clone());
            root
        }
        fn union(parent: &mut std::collections::HashMap<String, String>, a: &str, b: &str) {
            let ra = find(parent, a);
            let rb = find(parent, b);
            if ra != rb {
                parent.insert(ra, rb);
            }
        }
        for coupling in &netlist.couplings {
            let l1 = coupling.inductor1_name.to_ascii_lowercase();
            let l2 = coupling.inductor2_name.to_ascii_lowercase();
            union(&mut parent, &l1, &l2);
        }

        // Group inductors by their root in the union-find.
        let mut groups: std::collections::HashMap<String, Vec<String>> =
            std::collections::HashMap::new();
        for name in &ind_names {
            let root = find(&mut parent, name);
            groups.entry(root).or_default().push(name.clone());
        }
        // Deterministic processing order. Sort each group's members by netlist
        // position, then order the groups themselves by their first member's
        // position. Iterating the `groups` HashMap directly would assign the
        // augmented transformer/coupled-inductor branch rows in a per-process
        // random order — a pure row permutation (correctness-neutral, hence it
        // validates equivalent) but non-reproducible codegen, which breaks
        // byte-identical regen and commit-pinned provenance.
        let mut ordered_groups: Vec<Vec<String>> = groups.into_values().collect();
        for members in &mut ordered_groups {
            members.sort_by_key(|m| ind_pos(m));
        }
        ordered_groups
            .sort_by_key(|members| members.first().map(|m| ind_pos(m)).unwrap_or(usize::MAX));

        // Track internal nodes added by ideal transformer decomposition.
        // Internal nodes are 1-indexed, starting after the last circuit node.
        let mut next_internal_node = n + 1;

        // Process each group
        for members in ordered_groups {
            for m in &members {
                coupled_inductor_names.insert(m.clone());
            }

            // The group's tightest coupling (a saturating core needs k > 0.8).
            let max_k = netlist
                .couplings
                .iter()
                .filter(|c| {
                    let a = c.inductor1_name.to_ascii_lowercase();
                    let b = c.inductor2_name.to_ascii_lowercase();
                    members.contains(&a) && members.contains(&b)
                })
                .map(|c| c.coupling)
                .fold(0.0_f64, f64::max);

            // A group carrying ISAT on any winding is a SATURATING shared core:
            // one saturating magnetizing branch (its current is the net
            // magnetizing current) behind ideal couplings, and linear leakage,
            // `λ = Lm(φ)·n nᵀ·i + L_leak·i` (saturating_core.rs). Non-saturating
            // groups take the exact coupled-inductor [L] path below.
            let group_saturating = members.iter().any(|m| inductor_refs[m].isat.is_some());
            // Windings as the deck spells them, for messages.
            let spelled: Vec<&str> = members
                .iter()
                .map(|m| inductor_refs[m].name.as_str())
                .collect();
            let names = spelled.join(", ");
            let declares_core = members
                .iter()
                .any(|m| inductor_refs[m].turns.is_some() || inductor_refs[m].lm.is_some());
            if declares_core && !group_saturating {
                return Err(MnaError::TopologyError(format!(
                    "coupled inductors {{{names}}} carry TURNS= or LM=, which state a saturating \
                     shared core, but no winding carries ISAT. Add ISAT to the core, or remove \
                     TURNS=/LM= to simulate the group linearly."
                )));
            }
            if group_saturating {
                // Saturating coupled groups the shared-core model does not cover
                // are refused, not approximated.
                if max_k <= IDEAL_XFMR_K_THRESHOLD {
                    return Err(MnaError::TopologyError(format!(
                        "coupled inductors {{{names}}} carry ISAT (a saturating shared core) \
                         but their largest coupling is k={max_k}. A closed iron core puts \
                         k above 0.99: leakage is a small air-path fraction. k <= 0.8 means \
                         either no shared core, in which case give each inductor its own \
                         ISAT and drop the K line, or a deliberate iron leakage path \
                         (ballast, neon or welding transformers) whose leakage flux itself \
                         saturates, which melange does not model."
                    )));
                }
                let w = members.len();
                let refs: Vec<&InductorRef> = members.iter().map(|m| &inductor_refs[m]).collect();

                // The linear inductance matrix: self-inductances and K lines.
                let mut l_mat = vec![vec![0.0; w]; w];
                for i in 0..w {
                    l_mat[i][i] = refs[i].value;
                }
                for c in &netlist.couplings {
                    let a = c.inductor1_name.to_ascii_lowercase();
                    let b = c.inductor2_name.to_ascii_lowercase();
                    if let (Some(ia), Some(ib)) = (
                        members.iter().position(|m| *m == a),
                        members.iter().position(|m| *m == b),
                    ) {
                        let m_ab = c.coupling * (refs[ia].value * refs[ib].value).sqrt();
                        l_mat[ia][ib] = m_ab;
                        l_mat[ib][ia] = m_ab;
                    }
                }

                // The split into core and leakage: stated (TURNS= on every
                // winding, LM= on one), or the implicit two-winding form.
                let split = if declares_core {
                    let missing: Vec<&str> = (0..w)
                        .filter(|&i| refs[i].turns.is_none())
                        .map(|i| spelled[i])
                        .collect();
                    if !missing.is_empty() {
                        return Err(MnaError::TopologyError(format!(
                            "saturating core {{{names}}}: TURNS= is given on some windings but \
                             not on {}. The core's turns vector needs every winding's relative \
                             turns.",
                            missing.join(", ")
                        )));
                    }
                    let with_lm: Vec<usize> = (0..w).filter(|&i| refs[i].lm.is_some()).collect();
                    match with_lm.as_slice() {
                        [a] => {
                            let turns: Vec<f64> =
                                refs.iter().map(|r| r.turns.unwrap_or_default()).collect();
                            crate::saturating_core::CoreSplit::explicit(
                                &l_mat,
                                &turns,
                                *a,
                                refs[*a].lm.unwrap_or_default(),
                            )
                        }
                        [] => {
                            return Err(MnaError::TopologyError(format!(
                                "saturating core {{{names}}} gives TURNS= but no LM=. State the \
                                 core's magnetizing inductance, seen from one winding, with LM= \
                                 on that winding."
                            )))
                        }
                        many => {
                            let on: Vec<&str> = many.iter().map(|&i| spelled[i]).collect();
                            return Err(MnaError::TopologyError(format!(
                                "saturating core {{{names}}} gives LM= on {}. One core has one \
                                 magnetizing inductance: give LM= on one winding only.",
                                on.join(" and ")
                            )));
                        }
                    }
                } else if w == 2 {
                    crate::saturating_core::CoreSplit::implicit_pair(
                        [refs[0].value, refs[1].value],
                        max_k,
                    )
                } else {
                    let hint = match crate::saturating_core::star_decomposition(&l_mat) {
                        Some((a, lm, turns)) => format!(
                            " The diagonal-leakage (star) decomposition of this [L] is LM={lm:.6e} \
                             on {} with {}. Add these to accept that assumption, which decides \
                             where saturation sits, or state your core.",
                            spelled[a],
                            spelled
                                .iter()
                                .zip(&turns)
                                .map(|(m, t)| format!("TURNS={t:.6e} on {m}"))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ),
                        None if w == 3 => " This [L] has no diagonal-leakage (star) \
                                           decomposition (a winding sits between the other two, \
                                           or a coupling is missing), so the core must be stated."
                            .to_string(),
                        None => String::new(),
                    };
                    return Err(MnaError::TopologyError(format!(
                        "saturating transformer {{{names}}} has {w} windings, and its linear [L] \
                         does not fix how much of it is the saturating core. State the core: \
                         TURNS= (relative turns) on every winding and LM= (the core's \
                         magnetizing inductance seen from that winding) on one. Declaring LM= \
                         declares one core loop linking every winding.{hint}"
                    )));
                };
                if let Err((value, vector)) = split.leakage_positive_definite() {
                    let direction: Vec<String> = spelled
                        .iter()
                        .zip(&vector)
                        .map(|(m, v)| format!("{v:+.3} {m}"))
                        .collect();
                    return Err(MnaError::TopologyError(format!(
                        "saturating core {{{names}}}: the leakage L - LM*n*n^T left by the stated \
                         core is not positive-definite (eigenvalue {value:.3e} H along \
                         [{}]). Leakage is air-flux energy and must be positive in every \
                         direction: LM is too large for the couplings, or TURNS= disagrees with \
                         the self-inductances and K.",
                        direction.join(", ")
                    )));
                }
                let a = split.ref_idx;
                let l_aa = l_mat[a][a];
                let n2 = |i: usize| split.n[i] * split.n[i];

                // One core, one magnetizing air floor, in henries on the
                // reference winding. An authored LAIR is its winding's TOTAL
                // air-core self-inductance, less that winding's leakage; CORE=
                // and the default are the magnetizing floor itself, a class
                // fraction of the reference winding's inductance.
                let mut floors: Vec<(usize, f64, crate::parser::SatFloor)> = Vec::new();
                for (i, r) in refs.iter().enumerate() {
                    let Some(f) = r.air_floor else {
                        continue;
                    };
                    let value = match f {
                        crate::parser::SatFloor::Explicit(lair) => {
                            let v = (lair * r.value - split.l_leak[i][i]) / n2(i);
                            if v <= 0.0 {
                                let leak = split.l_leak[i][i] / r.value;
                                return Err(MnaError::TopologyError(format!(
                                    "{}: LAIR={lair:e} is the winding's total air-core \
                                     inductance, but its leakage already is {leak:.3e} of it, \
                                     leaving no magnetizing air floor. Real audio iron has \
                                     leakage ~ 1e-5..1e-4. Give LAIR > {leak:.3e}, or CORE= to \
                                     set the core's magnetizing floor directly.",
                                    spelled[i]
                                )));
                            }
                            v
                        }
                        other => crate::parser::resolve_air_floor(Some(other)).0 * l_aa,
                    };
                    floors.push((i, value, f));
                }
                let (floor_h, core_air_floor) =
                    match floors.iter().min_by(|x, y| x.1.total_cmp(&y.1)).copied() {
                        None => (crate::parser::resolve_air_floor(None).0 * l_aa, None),
                        Some((_, least, f)) => {
                            let most = floors.iter().map(|x| x.1).fold(0.0_f64, f64::max);
                            let listing = || {
                                floors
                                    .iter()
                                    .map(|(i, v, _)| format!("{} {:.3e} H", spelled[*i], v))
                                    .collect::<Vec<_>>()
                                    .join(", ")
                            };
                            if most > FLOOR_AGREEMENT_BAND * least {
                                return Err(MnaError::TopologyError(format!(
                                "coupled inductors {{{names}}} share one core but their air-core \
                                 declarations imply magnetizing floors more than \
                                 {FLOOR_AGREEMENT_BAND}x apart ({}). A core has one floor, and \
                                 the declarations are good to that band at best: put LAIR= or \
                                 CORE= on one winding only, or reconcile them.",
                                listing()
                            )));
                            }
                            if most > least * (1.0 + 1e-12) {
                                crate::diag_warn!(
                                "Saturating shared core {{{names}}}: the air-core declarations \
                                 imply different magnetizing floors ({}), within the {}x band \
                                 they are good to; using the least, {least:.3e} H.",
                                listing(),
                                FLOOR_AGREEMENT_BAND
                            );
                            }
                            (least, Some(f))
                        }
                    };
                if floor_h >= split.lm {
                    return Err(MnaError::TopologyError(format!(
                        "saturating core {{{names}}}: the magnetizing air floor {floor_h:.3e} H is \
                         not below the core's magnetizing inductance {:.3e} H, so nothing is \
                         left to saturate.",
                        split.lm
                    )));
                }
                let floor_reading = match core_air_floor {
                    Some(crate::parser::SatFloor::Explicit(_)) => {
                        "LAIR=, total air-core self-inductance less leakage".to_string()
                    }
                    other => format!(
                        "{}; magnetizing air floor, leakage from K",
                        crate::parser::resolve_air_floor(other).1
                    ),
                };

                // One shared core has one saturation current. A datasheet
                // rating converts against its winding's own terminal law (its
                // magnetizing fraction LM*n_i^2/L_i and floor), and every ISAT
                // refers to the reference winding by turns: an MMF, n_i*I.
                let rated: Vec<usize> = (0..w).filter(|&i| refs[i].isat.is_some()).collect();
                if rated.len() > 1 && rated.iter().any(|&i| refs[i].isat_spec.is_some()) {
                    return Err(MnaError::TopologyError(format!(
                        "coupled inductors {{{names}}} share one core, and more than one carries \
                         ISAT with a datasheet form (ISAT_DROP= or L_AT_IDC=). Rate the core on \
                         one winding only."
                    )));
                }
                let mut referred: Vec<(usize, f64)> = Vec::new();
                for &i in &rated {
                    let r = refs[i];
                    let authored = r.isat.unwrap_or_default();
                    let model = match r.isat_spec {
                        Some(spec) => isat_from_datasheet(
                            spelled[i],
                            authored,
                            spec,
                            r.value,
                            split.lm * n2(i) / r.value,
                            floor_h * n2(i) / r.value,
                        )?,
                        None => authored,
                    };
                    let v = model * split.n[i];
                    if !(v.is_finite() && v > 0.0) {
                        return Err(MnaError::InvalidParameter(format!(
                            "{}: ISAT {model:e} A referred to the reference winding (x {:e}) is \
                             {v:e}, not a usable saturation current.",
                            spelled[i], split.n[i]
                        )));
                    }
                    referred.push((i, v));
                }
                let (first_i, core_isat) = referred[0];
                if let Some((i, other)) = referred
                    .iter()
                    .find(|(_, v)| (v - core_isat).abs() > 1e-9 * core_isat.abs().max(v.abs()))
                {
                    return Err(MnaError::TopologyError(format!(
                        "coupled inductors {{{names}}} share one core but carry different \
                         saturation currents: {} and {} refer to {core_isat:e} A and {other:e} A \
                         on the reference winding. A core has one saturation current; put ISAT \
                         on one winding only.",
                        spelled[first_i], spelled[*i]
                    )));
                }

                // Realization. Per winding, leakage from its node to an internal
                // node; the magnetizing inductor on the reference winding's
                // internal node; ideal couplings (turns n_i) to the others.
                let mut internal_nodes_p = Vec::with_capacity(w);
                for _ in 0..w {
                    internal_nodes_p.push(next_internal_node);
                    next_internal_node += 1;
                }
                if split.leakage_is_diagonal() {
                    // No leakage minimum: this path is nodal full-LU only, where
                    // a small leakage is an inductor branch row tending to a 0 V
                    // source, which is well-posed.
                    for (i, r) in refs.iter().enumerate() {
                        self.inductors.push(InductorElement {
                            name: format!("{}_leak", r.name),
                            node_i: r.node_i,
                            node_j: internal_nodes_p[i],
                            value: split.l_leak[i][i],
                            isat: None,
                            air_floor: None,
                            shared_core: None,
                        });
                    }
                } else {
                    let inductances: Vec<f64> = (0..w).map(|i| split.l_leak[i][i]).collect();
                    let coupling_matrix = (0..w)
                        .map(|i| {
                            (0..w)
                                .map(|j| {
                                    split.l_leak[i][j] / (inductances[i] * inductances[j]).sqrt()
                                })
                                .collect()
                        })
                        .collect();
                    mna.transformer_groups.push(TransformerGroupInfo {
                        name: format!("{}_leakage", refs[a].name),
                        num_windings: w,
                        winding_names: refs.iter().map(|r| format!("{}_leak", r.name)).collect(),
                        winding_node_i: refs.iter().map(|r| r.node_i).collect(),
                        winding_node_j: internal_nodes_p.clone(),
                        inductances,
                        coupling_matrix,
                    });
                }
                self.inductors.push(InductorElement {
                    name: format!("{}_mag", refs[a].name),
                    node_i: internal_nodes_p[a],
                    node_j: refs[a].node_j,
                    value: split.lm,
                    isat: Some(core_isat),
                    air_floor: core_air_floor,
                    shared_core: Some(SharedCore {
                        floor_frac: floor_h / split.lm,
                        floor_reading,
                        implicit_k: (!declares_core).then_some(max_k),
                    }),
                });
                for i in (0..w).filter(|&i| i != a) {
                    mna.ideal_transformers.push(IdealTransformerCoupling {
                        name: format!("ideal_{}_{}", refs[a].name, refs[i].name),
                        pri_node_p: internal_nodes_p[a],
                        pri_node_n: refs[a].node_j,
                        sec_node_p: internal_nodes_p[i],
                        sec_node_n: refs[i].node_j,
                        turns_ratio: split.n[i],
                    });
                }
                log::info!(
                    "Saturating shared core {{{names}}}: {w} windings, LM={:.3e} H on {}, {} \
                     leakage",
                    split.lm,
                    spelled[a],
                    if split.leakage_is_diagonal() {
                        "diagonal"
                    } else {
                        "coupled"
                    }
                );
                continue;
            }

            if members.len() == 2 {
                // 2-winding: use existing CoupledInductorInfo path
                let r1 = &inductor_refs[&members[0]];
                let r2 = &inductor_refs[&members[1]];
                // Find the coupling between these two
                let k_val = netlist
                    .couplings
                    .iter()
                    .find(|c| {
                        let a = c.inductor1_name.to_ascii_lowercase();
                        let b = c.inductor2_name.to_ascii_lowercase();
                        (a == members[0] && b == members[1]) || (a == members[1] && b == members[0])
                    })
                    .map(|c| c.coupling)
                    .unwrap_or(0.0);
                let k_name = netlist
                    .couplings
                    .iter()
                    .find(|c| {
                        let a = c.inductor1_name.to_ascii_lowercase();
                        let b = c.inductor2_name.to_ascii_lowercase();
                        (a == members[0] && b == members[1]) || (a == members[1] && b == members[0])
                    })
                    .map(|c| c.name.clone())
                    .unwrap_or_default();
                mna.coupled_inductors.push(CoupledInductorInfo {
                    name: k_name,
                    l1_name: r1.name.clone(),
                    l2_name: r2.name.clone(),
                    l1_node_i: r1.node_i,
                    l1_node_j: r1.node_j,
                    l2_node_i: r2.node_i,
                    l2_node_j: r2.node_j,
                    l1_value: r1.value,
                    l2_value: r2.value,
                    coupling: k_val,
                });
            } else {
                // Multi-winding (3+): build NxN inductance matrix and invert.
                // The per-pair approach incorrectly stamps self-conductance once per K
                // directive an inductor appears in, giving wrong effective inductances.
                // The NxN approach computes the correct admittance matrix Y = inv(L).
                let w = members.len();
                let mut coupling_matrix = vec![vec![0.0f64; w]; w];
                for i in 0..w {
                    coupling_matrix[i][i] = 1.0; // Self-coupling = 1.0
                }
                // Fill in coupling coefficients from K directives
                for coupling in &netlist.couplings {
                    let a = coupling.inductor1_name.to_ascii_lowercase();
                    let b = coupling.inductor2_name.to_ascii_lowercase();
                    if let (Some(ia), Some(ib)) = (
                        members.iter().position(|m| *m == a),
                        members.iter().position(|m| *m == b),
                    ) {
                        coupling_matrix[ia][ib] = coupling.coupling;
                        coupling_matrix[ib][ia] = coupling.coupling;
                    }
                }
                let mut winding_node_i = Vec::with_capacity(w);
                let mut winding_node_j = Vec::with_capacity(w);
                let mut inductances = Vec::with_capacity(w);
                let mut winding_names = Vec::with_capacity(w);
                for m in &members {
                    let r = &inductor_refs[m];
                    winding_node_i.push(r.node_i);
                    winding_node_j.push(r.node_j);
                    inductances.push(r.value);
                    winding_names.push(r.name.clone());
                }
                // Validate: check that the inductance matrix is positive definite.
                // A non-PD matrix means the coupling coefficients are physically
                // inconsistent (e.g., k_ab=0.95, k_bc=0.95, k_ac=0.50 is impossible).
                //
                // NOTE: for w >= 3 this is NOT a positive-definiteness test. "Min
                // diagonal of the inverse > 0" is necessary for PD, not
                // sufficient: an indefinite L can pass it. Known, and left
                // unfixed in 0.1.14 (the w == 2 branch is an exact determinant).
                {
                    let mut l_mat = vec![vec![0.0f64; w]; w];
                    for i in 0..w {
                        for j in 0..w {
                            l_mat[i][j] =
                                coupling_matrix[i][j] * (inductances[i] * inductances[j]).sqrt();
                        }
                    }
                    // Check via Cholesky-like: all leading minors must be positive.
                    // For small w (≤8), compute determinant directly.
                    let det = if w == 2 {
                        l_mat[0][0] * l_mat[1][1] - l_mat[0][1] * l_mat[1][0]
                    } else {
                        // Use the invert_small_matrix helper — if it returns near-zero
                        // diagonal entries, the matrix is singular or non-PD. A matrix
                        // it cannot invert (singular, non-finite) is not PD either:
                        // report 0.0 so the warning below fires.
                        match invert_small_matrix(&l_mat) {
                            // Check: all diagonal entries of inv should be positive for PD
                            Ok(inv) => inv
                                .iter()
                                .enumerate()
                                .map(|(i, row)| row[i])
                                .fold(f64::INFINITY, f64::min),
                            Err(_) => 0.0,
                        }
                    };
                    if det <= 0.0 || !det.is_finite() {
                        crate::diag_warn!(
                            "Transformer group '{}' ({} windings) has non-positive-definite inductance matrix. \
                             This means the coupling coefficients are physically inconsistent. \
                             Check that all K values are compatible (all windings on the same core \
                             should have similar coupling coefficients).",
                            format!("xfmr_{}", mna.transformer_groups.len()),
                            w
                        );
                    }
                }

                let group_idx = mna.transformer_groups.len();
                mna.transformer_groups.push(TransformerGroupInfo {
                    name: format!("xfmr_{}", group_idx),
                    num_windings: w,
                    winding_names,
                    winding_node_i,
                    winding_node_j,
                    inductances,
                    coupling_matrix,
                });
            }
        }

        // Remove coupled inductors from the uncoupled inductors list
        self.inductors
            .retain(|ind| !coupled_inductor_names.contains(&ind.name.to_ascii_lowercase()));

        // Expand MNA matrices if ideal transformer decomposition added internal nodes
        let num_internal = next_internal_node - (n + 1);
        if num_internal > 0 {
            let new_n = n + num_internal;
            for row in &mut mna.g {
                row.resize(new_n, 0.0);
            }
            for row in &mut mna.c {
                row.resize(new_n, 0.0);
            }
            for _ in 0..num_internal {
                mna.g.push(vec![0.0; new_n]);
                mna.c.push(vec![0.0; new_n]);
            }
            for row in &mut mna.n_v {
                row.resize(new_n, 0.0);
            }
            for _ in 0..num_internal {
                mna.n_i.push(vec![0.0; mna.m]);
            }
            mna.n = new_n;
        }

        mna.node_map = self.node_map;
        mna.nonlinear_devices = self.nonlinear_devices;
        mna.voltage_sources = self.voltage_sources;
        mna.behavioral_sources = self.behavioral_sources;
        mna.current_sources = self.current_sources;
        mna.capacitor_ics = self.capacitor_ics;
        mna.inductors = self.inductors;
        mna.opamps = self.opamps;
        mna.vcas = self.vcas;

        // -----------------------------------------------------------------
        // Expand matrices for augmented MNA (voltage sources + VCVS).
        //
        // Count VCVS elements in element order.
        // Use mna.n (which may have grown due to internal nodes from ideal transformers).
        let n_base = mna.n;
        let num_vs = mna.voltage_sources.len();
        let num_vcvs = self
            .elements
            .iter()
            .filter(|e| matches!(e.element_type, ElementType::Vcvs))
            .count();
        let num_ideal_xfmr = mna.ideal_transformers.len();
        // IIR op-amp model: GBW op-amps no longer create internal nodes.
        // The dominant pole is modeled as an external IIR filter in codegen.
        let num_opamp_internal = 0;
        // Count current-mode VCAs (each needs an augmented row for current sensing)
        // Each current-mode VCA needs 2 augmented rows:
        // [0] internal node (sig+_int) with dummy R to ground
        // [1] sensing source branch current
        let num_vca_augmented = mna.vcas.iter().filter(|v| v.current_mode).count() * 2;
        // Behavioral `V={}` sources each need a branch-current augmented row.
        let num_behavioral_v = mna
            .behavioral_sources
            .iter()
            .filter(|b| b.v_ext_idx.is_some())
            .count();
        let behavioral_v_base =
            n_base + num_vs + num_vcvs + num_ideal_xfmr + num_opamp_internal + num_vca_augmented;
        let n_aug = behavioral_v_base + num_behavioral_v;
        mna.n_aug = n_aug;

        // Expand G, C, N_v, N_i to n_aug dimensions (extra rows/cols are zero).
        if n_aug > n_base {
            // Expand each existing row by appending zeros
            for row in &mut mna.g {
                row.resize(n_aug, 0.0);
            }
            for row in &mut mna.c {
                row.resize(n_aug, 0.0);
            }
            for row in &mut mna.n_v {
                row.resize(n_aug, 0.0);
            }
            // Add new zero rows for the augmented variables
            for _ in n_base..n_aug {
                mna.g.push(vec![0.0; n_aug]);
                mna.c.push(vec![0.0; n_aug]);
            }
            // N_i is n×m: its row width stays m; add n_aug-n zero rows for the
            // augmented variables.
            for _ in n_base..n_aug {
                mna.n_i.push(vec![0.0; mna.m]);
            }
        }

        // Stamp voltage sources with augmented MNA.
        // For VS between n+ and n- with current j_vs (extra unknown at row/col k):
        //   KVL constraint row k: G[k][n+] = +1, G[k][n-] = -1
        //   Current injection col k: G[n+][k] = +1, G[n-][k] = -1
        for vs in &mna.voltage_sources {
            let k = n_base + vs.ext_idx; // augmented row/col index
            let np = vs.n_plus_idx;
            let nm = vs.n_minus_idx;
            // Current injection column: j_vs enters n+, exits n-
            if np > 0 {
                mna.g[np - 1][k] += 1.0;
                mna.g[k][np - 1] += 1.0;
            }
            if nm > 0 {
                mna.g[nm - 1][k] -= 1.0;
                mna.g[k][nm - 1] -= 1.0;
            }
        }

        // Stamp behavioral `V={}` sources: same linear coupling as a voltage
        // source (branch current + constraint row). The nonlinear `f(v)` part of
        // the constraint `V(n+) - V(n-) = f(v)` is added by the nodal emitter's
        // NR loop (residual + Jacobian); here we lay only the linear B^T part.
        for b in &mut mna.behavioral_sources {
            let Some(v_idx) = b.v_ext_idx else { continue };
            let k = behavioral_v_base + v_idx;
            b.aug_row = Some(k);
            if b.n_plus_idx > 0 {
                mna.g[b.n_plus_idx - 1][k] += 1.0;
                mna.g[k][b.n_plus_idx - 1] += 1.0;
            }
            if b.n_minus_idx > 0 {
                mna.g[b.n_minus_idx - 1][k] -= 1.0;
                mna.g[k][b.n_minus_idx - 1] -= 1.0;
            }
        }

        // Resolve `.runtime` directives to aug-MNA rows. Parser already
        // validated each directive's VS name exists; here we translate the
        // name to its stamped row. The row is `n_base + ext_idx` to match
        // the aug-MNA convention used by the VS stamping loop above.
        for rt in &netlist.runtime_sources {
            let vs_idx = mna
                .voltage_sources
                .iter()
                .position(|v| v.name.eq_ignore_ascii_case(&rt.vs_name))
                .ok_or_else(|| {
                    MnaError::TopologyError(format!(
                        ".runtime references voltage source '{}' that was not resolved in MNA",
                        rt.vs_name
                    ))
                })?;
            let vs = &mna.voltage_sources[vs_idx];
            mna.runtime_sources.push(RuntimeSourceInfo {
                vs_name: vs.name.clone(),
                field_name: rt.field_name.clone(),
                vs_row: n_base + vs.ext_idx,
            });
        }

        // Stamp VCVS elements with augmented MNA.
        // For VCVS: V_out+ - V_out- = gain*(V_ctrl+ - V_ctrl-)
        //   Extra unknown j_vcvs at row/col k = n + num_vs + vcvs_idx
        //   Current injection column k: enters out+, exits out-
        //   KVL constraint row k: G[k][out+]=+1, G[k][out-]=-1,
        //                          G[k][ctrl+]=-gain, G[k][ctrl-]=+gain
        let mut vcvs_idx = 0;
        for elem in &self.elements {
            if let ElementType::Vcvs = elem.element_type {
                if elem.nodes.len() >= 4 {
                    let k = n_base + num_vs + vcvs_idx;
                    let out_p = elem.nodes[0];
                    let out_n = elem.nodes[1];
                    let ctrl_p = elem.nodes[2];
                    let ctrl_n = elem.nodes[3];
                    let gain = elem.value;

                    // Current injection column: j_vcvs enters out+, exits out-
                    if out_p > 0 {
                        mna.g[out_p - 1][k] += 1.0;
                    }
                    if out_n > 0 {
                        mna.g[out_n - 1][k] -= 1.0;
                    }
                    // KVL constraint row: V_out+ - V_out- - gain*(V_ctrl+ - V_ctrl-) = 0
                    if out_p > 0 {
                        mna.g[k][out_p - 1] += 1.0;
                    }
                    if out_n > 0 {
                        mna.g[k][out_n - 1] -= 1.0;
                    }
                    if ctrl_p > 0 {
                        mna.g[k][ctrl_p - 1] -= gain;
                    }
                    if ctrl_n > 0 {
                        mna.g[k][ctrl_n - 1] += gain;
                    }

                    mna.vcvs_sources.push(VcvsAugInfo { aug_idx: vcvs_idx });
                    vcvs_idx += 1;
                }
            }
        }

        // Stamp ideal transformer couplings with augmented MNA.
        // For ideal transformer with turns ratio n:
        //   V_sec = n * V_pri   (voltage coupling)
        //   I_pri = -n * I_sec  (current coupling — power-conserving dot convention:
        //                        power flows THROUGH, so V_pri·I_pri + V_sec·I_sec = 0)
        //
        // Extra unknown j (secondary current) at row/col k = n + num_vs + num_vcvs + xfmr_idx.
        // The current-injection COLUMN must be the TRANSPOSE of the KVL constraint ROW
        // (standard symmetric MNA constraint stamp). The primary entries below were
        // previously NEGATED relative to the row, realizing I_pri = +n·I_sec — a
        // non-power-conserving, non-symmetric [L] that made the T-model's transfer
        // ~5% high (growing with frequency), verified against the exact [L]-matrix
        // solve and ngspice. This was latent while the T-model was disabled
        // (non-saturating groups never take it); enabling it for saturating transformers
        // exposed it.
        // KVL row k: G[k][sec+]=+1, G[k][sec-]=-1, G[k][pri+]=-n, G[k][pri-]=+n
        // Current col k: G[sec+][k]=+1, G[sec-][k]=-1, G[pri+][k]=-n, G[pri-][k]=+n
        for (xi, xfmr) in mna.ideal_transformers.iter().enumerate() {
            let k = n_base + num_vs + num_vcvs + xi;
            let sp = xfmr.sec_node_p;
            let sn = xfmr.sec_node_n;
            let pp = xfmr.pri_node_p;
            let pn = xfmr.pri_node_n;
            let nr = xfmr.turns_ratio;

            // KVL constraint row: V(sec+) - V(sec-) - n*(V(pri+) - V(pri-)) = 0
            if sp > 0 {
                mna.g[k][sp - 1] += 1.0;
            }
            if sn > 0 {
                mna.g[k][sn - 1] -= 1.0;
            }
            if pp > 0 {
                mna.g[k][pp - 1] -= nr;
            }
            if pn > 0 {
                mna.g[k][pn - 1] += nr;
            }

            // Current injection column = transpose of the KVL constraint row above.
            // j enters sec+, exits sec-; -n*j at pri+, +n*j at pri- (I_pri = -n*I_sec).
            if sp > 0 {
                mna.g[sp - 1][k] += 1.0;
            }
            if sn > 0 {
                mna.g[sn - 1][k] -= 1.0;
            }
            if pp > 0 {
                mna.g[pp - 1][k] -= nr;
            }
            if pn > 0 {
                mna.g[pn - 1][k] += nr;
            }
        }

        // Stamp linear elements (resistors, capacitors, VCCS; skip VS/VCVS now handled above)
        for elem in &self.elements {
            match elem.element_type {
                ElementType::Resistor => {
                    if elem.nodes.len() >= 2 && elem.value != 0.0 {
                        // Override priority: switch pos-0 > pot default > static value.
                        // Switch wins because a component can only be in one .switch and
                        // pos-0 defines the canonical initial stamp. Pots and switches
                        // on the same resistor are nonsensical; switch takes precedence.
                        let key = elem.name.to_ascii_uppercase();
                        let r = mna
                            .switch_default_overrides
                            .get(&key)
                            .filter(|(kind, _)| *kind == 'R')
                            .map(|(_, v)| *v)
                            .or_else(|| mna.pot_default_overrides.get(&key).copied())
                            .unwrap_or(elem.value);
                        // Guard against a 0-Ω resolved override (closed-switch or
                        // zero-pot position): the caller's zero-check covers only
                        // the DECLARED value, not a resolved switch/pot default,
                        // so a 0-Ω position would stamp an infinite conductance.
                        // Floor R so a "short" becomes a finite near-short.
                        let g = 1.0 / r.max(1e-9);
                        stamp_conductance_to_ground(&mut mna.g, elem.nodes[0], elem.nodes[1], g);
                    }
                }
                ElementType::Capacitor => {
                    if elem.nodes.len() >= 2 {
                        // Switch pos-0 override for switched capacitors (e.g. tone-switch caps)
                        let c_val = mna
                            .switch_default_overrides
                            .get(&elem.name.to_ascii_uppercase())
                            .filter(|(kind, _)| *kind == 'C')
                            .map(|(_, v)| *v)
                            .unwrap_or(elem.value);
                        stamp_conductance_to_ground(
                            &mut mna.c,
                            elem.nodes[0],
                            elem.nodes[1],
                            c_val,
                        );
                    }
                }
                ElementType::Inductor => {
                    // Inductors are handled in DK kernel with companion model.
                    // Stamping happens at kernel creation since we need sample rate.
                }
                ElementType::VoltageSource => {
                    // Handled above with augmented MNA stamping (not Norton equivalent).
                }
                ElementType::Vcvs => {
                    // Handled above with augmented MNA stamping (not Norton equivalent).
                }
                ElementType::Vccs => {
                    // Direct VCCS stamp into G matrix
                    if elem.nodes.len() >= 4 {
                        let out_p = elem.nodes[0];
                        let out_n = elem.nodes[1];
                        let ctrl_p = elem.nodes[2];
                        let ctrl_n = elem.nodes[3];
                        let gm = elem.value;

                        stamp_vccs(&mut mna.g, out_p, out_n, ctrl_p, ctrl_n, gm);
                    }
                }
            }
        }

        // Stamp op-amps as VCCS into G matrix.
        //
        // Three stamping modes, in priority order:
        //
        // 1. **BoyleDiodes internal gain node** — auto-detected by looking up
        //    `_oa_int_{safe_name}` in `node_map`. The augment helper
        //    `codegen::ir::augment_netlist_with_boyle_diodes` synthesizes
        //    catch diodes + a unity-gain output buffer that reference this
        //    node, so its presence in `node_map` is the signal that this
        //    op-amp is in BoyleDiodes mode. Gm/Go are stamped at the
        //    internal node with `R_BOYLE_INT_LOAD = 1 MΩ` as the effective
        //    output resistance — making the catch diode's exponential
        //    conductance only have to balance against ~1 µS instead of
        //    ~4000 S. The original output node is left untouched (the
        //    buffer VCCS handles it).
        //
        // 2. **GBW IIR model** — when `oa.gbw` is finite, stamp at the
        //    output for DC OP. The dominant pole is stripped and re-applied
        //    in codegen as an external IIR filter (currently disabled
        //    behind `if !has_vca && false`, kept for future re-enable).
        //
        // 3. **Simple linear VCCS** — direct stamp at the output node.
        //    Original behavior; used for all op-amps that aren't in the
        //    BoyleDiodes augmentation path.
        for oa in &mut mna.opamps {
            let out = oa.n_out_idx;
            let np = oa.n_plus_idx;
            let nm = oa.n_minus_idx;

            if out == 0 {
                continue;
            }

            // Input-stage parasitics (IB + RIN): tiny effects that matter for
            // circuits where the op-amp input node is a high-impedance
            // integrator (e.g. a bus-compressor sidechain where a 3.3 MΩ /
            // 10 pF integrator winds up unboundedly under any DC offset
            // without a bleed path to ground). Default IB=0 / RIN=+∞
            // preserves ideal-op-amp behavior byte-identically.
            //
            // Stamped BEFORE the output-stage mode dispatch because these
            // touch only the INPUT nodes and apply to every mode — the
            // BoyleDiodes branch below `continue`s early and used to skip
            // IB/RIN entirely.
            //
            // IB sign convention: positive IB injects `+IB` at both input
            // nodes (current flowing out of the op-amp pin into the external
            // circuit — PNP-input/JFET default). For NPN-input parts specify
            // negative IB in the .model card.
            if oa.ib != 0.0 {
                if np > 0 {
                    mna.current_sources.push(CurrentSourceInfo {
                        name: format!("_{}_IB_plus", oa.name),
                        n_plus_idx: np,
                        n_minus_idx: 0,
                        dc_value: oa.ib,
                    });
                }
                if nm > 0 {
                    mna.current_sources.push(CurrentSourceInfo {
                        name: format!("_{}_IB_minus", oa.name),
                        n_plus_idx: nm,
                        n_minus_idx: 0,
                        dc_value: oa.ib,
                    });
                }
            }
            // RIN: shunt conductance 1/RIN from each input pin to ground.
            // Physical significance: bounds integrator wind-up via a DC
            // leakage path (τ_leak = RIN · C_integ). For TL074 JFET input
            // RIN=1e12 gives g_in=1pS, effectively zero but finite — enough
            // to pull any accumulated offset to ground over ~seconds of
            // circuit time, preventing unbounded growth that the infinite-
            // input-Z ideal model produces.
            if oa.rin.is_finite() && oa.rin > 0.0 {
                let g_in = 1.0 / oa.rin;
                if np > 0 {
                    mna.g[np - 1][np - 1] += g_in;
                }
                if nm > 0 {
                    mna.g[nm - 1][nm - 1] += g_in;
                }
            }

            // BoyleDiodes auto-detection: look up the synthesized internal
            // node by name. If present, switch to internal-node stamping for
            // this op-amp and skip the rest of the output-stage dispatch
            // (input parasitics above are already stamped).
            let safe_name: String = oa
                .name
                .chars()
                .map(|c| {
                    if c.is_ascii_alphanumeric() || c == '_' {
                        c
                    } else {
                        '_'
                    }
                })
                .collect();
            let int_node_key = format!("_oa_int_{}", safe_name);
            if let Some(&int_idx_one) = mna.node_map.get(&int_node_key) {
                // 1-indexed in node_map; convert to 0-indexed matrix row.
                let int_row = int_idx_one - 1;
                // Effective Gm/Go derived from R_BOYLE_INT_LOAD so the
                // catch diode (anchored externally to this node) only
                // fights ~1 µS, not the op-amp's nominal 1/r_out.
                let gm_int = oa.aol / R_BOYLE_INT_LOAD;
                let go_int = 1.0 / R_BOYLE_INT_LOAD;
                // Convention: G[k][j]·Vj = current LEAVING node k. The op-amp
                // injects +Gm·(V+ − V−) INTO the internal node, so the current
                // leaving is −Gm·V+ + Gm·V−.
                if np > 0 {
                    mna.g[int_row][np - 1] -= gm_int;
                }
                if nm > 0 {
                    mna.g[int_row][nm - 1] += gm_int;
                }
                mna.g[int_row][int_row] += go_int;

                oa.n_int_idx = int_idx_one;
                log::debug!(
                    "Op-amp {} (BoyleDiodes): int_node={}, Gm_int={:.3e}, Go_int={:.3e}",
                    oa.name,
                    int_idx_one,
                    gm_int,
                    go_int,
                );
                continue;
            }

            let gm = oa.aol / oa.r_out;
            let go = 1.0 / oa.r_out;
            let o = out - 1;

            let has_gbw = oa.gbw.is_finite() && oa.gbw > 0.0;

            if has_gbw {
                // IIR op-amp model: stamp Gm at output (same as non-GBW) for DC OP.
                // The GBW dominant pole is modeled as an external IIR filter in codegen,
                // which strips Gm from G before building A for transient simulation.
                // NO internal node is created — this avoids Boyle's Gm~4000 S conditioning disaster.
                // Convention: G[k][j]·Vj = current LEAVING node k. The op-amp
                // injects +Gm·(V+ − V−) INTO the output node, so the current
                // leaving is −Gm·V+ + Gm·V−.
                if np > 0 {
                    mna.g[o][np - 1] -= gm;
                }
                if nm > 0 {
                    mna.g[o][nm - 1] += gm;
                }
                mna.g[o][o] += go;

                // Store IIR parameters for codegen
                let c_dom = oa.aol / (2.0 * std::f64::consts::PI * oa.gbw * oa.r_out);
                oa.iir_c_dom = c_dom;
                // n_internal_idx stays 0 (no internal node)

                log::debug!(
                    "Op-amp {} (IIR): GBW={:.0}Hz, C_dom={:.3e}F, Gm={:.2}, Go={:.4}",
                    oa.name,
                    oa.gbw,
                    c_dom,
                    gm,
                    go,
                );
            } else {
                // Simple VCCS (no GBW): direct stamp at output.
                // Convention: G[k][j]·Vj = current LEAVING node k. The op-amp
                // injects +Gm·(V+ − V−) INTO the output node, so the current
                // leaving is −Gm·V+ + Gm·V−.
                if np > 0 {
                    mna.g[o][np - 1] -= gm;
                }
                if nm > 0 {
                    mna.g[o][nm - 1] += gm;
                }
                mna.g[o][o] += go;
            }
        }

        // Allocate augmented rows for current-mode VCA sensing sources
        let vca_aug_base = n_base + num_vs + num_vcvs + num_ideal_xfmr + num_opamp_internal;
        let mut vca_aug_idx = 0;
        for vca in &mut mna.vcas {
            if vca.current_mode {
                let int_node = vca_aug_base + vca_aug_idx; // sig+_int
                let sense_row = vca_aug_base + vca_aug_idx + 1; // branch current
                vca_aug_idx += 2;
                vca.n_internal_idx = int_node + 1; // 1-indexed
                vca.n_sense_idx = sense_row + 1; // 1-indexed

                // Stamp virtual-ground terminator from sig+_int to ground.
                //
                // A real current-mode VCA (THAT 2180 / DBX 2150) holds sig+
                // at virtual ground via the log-antilog feedback loop. The
                // sense source between sig+ and sig+_int copies that
                // potential to sig+_int; the terminator here forces sig+_int
                // toward 0 V so sig+ sits near ground. Rdummy must be
                // substantially smaller than any realistic series drive
                // resistor, otherwise the drive resistor and Rdummy form a
                // current divider that attenuates the sensed input current
                // by Rdummy / (Rdrive + Rdummy).
                //
                // The original Rdummy=1 MΩ caused a ~32 dB passband loss on
                // circuits like 4kbuscomp-audiopath (Rdrive=27 kΩ): measured
                // −36.77 dB instead of the expected −5.1 dB (15K/27K
                // transimpedance ratio). At Rdummy=1 Ω the divider error is
                // <0.001 dB for any drive resistor ≥1 Ω.
                let g_dummy = 1.0; // 1 ohm
                mna.g[int_node][int_node] += g_dummy;

                log::debug!(
                    "VCA {} (current mode): internal={}, sense={}, Rdummy=1 ohm",
                    vca.name,
                    int_node,
                    sense_row
                );
            }
        }

        // Stamp nonlinear devices (collect indices first to avoid borrow issues)
        let device_info: Vec<_> = mna
            .nonlinear_devices
            .iter()
            .map(|d| {
                (
                    d.device_type,
                    d.dimension,
                    d.start_idx,
                    d.node_indices.clone(),
                    d.name.clone(),
                )
            })
            .collect();

        for (dev_type, dim, start_idx, node_indices, dev_name) in device_info {
            match dev_type {
                // The LDR resistance path is electrically a 2-terminal nonlinear
                // element identical in N_v/N_i structure to a diode: v_d =
                // V(r+)−V(r-) is the sole controlling voltage and the current
                // flows r+→r-. Only the eval differs (i = v_d/R vs the diode
                // exponential); the reduction stamp is shared verbatim.
                NonlinearDeviceType::Diode
                | NonlinearDeviceType::Ldr
                | NonlinearDeviceType::Glow => {
                    if node_indices.len() >= 2 {
                        let node_i = node_indices[0];
                        let node_j = node_indices[1];

                        // For diode from node_i (anode) to node_j (cathode):
                        // v_d = v_anode - v_cathode
                        // If anode grounded (v_i=0): v_d = -v_j, so N_v[j] = -1 extracts -v_j
                        // If cathode grounded (v_j=0): v_d = v_i, so N_v[i] = 1 extracts v_i
                        //
                        // N_i convention: positive = current INJECTED INTO node
                        // For current i_d flowing anode→cathode:
                        // - Extracted from anode: N_i[anode] = -1
                        // - Injected into cathode: N_i[cathode] = +1
                        if node_i == 0 && node_j > 0 {
                            let j = node_j - 1;
                            mna.n_v[start_idx][j] += -1.0; // v_d = 0 - v_j = -v_j
                            mna.n_i[j][start_idx] += 1.0; // Current injected into cathode
                        } else if node_j == 0 && node_i > 0 {
                            let i = node_i - 1;
                            mna.n_v[start_idx][i] += 1.0; // v_d = v_i - 0 = v_i
                            mna.n_i[i][start_idx] += -1.0; // Current extracted from anode
                        } else if node_i > 0 && node_j > 0 {
                            let i = node_i - 1;
                            let j = node_j - 1;
                            mna.stamp_nonlinear_2terminal(start_idx, i, j);
                        }
                    }
                }
                NonlinearDeviceType::Bjt => {
                    if node_indices.len() >= 3 {
                        let c_raw = node_indices[0];
                        let b_raw = node_indices[1];
                        let e_raw = node_indices[2];

                        // All-grounded BJTs are rejected in categorize_element,
                        // so c/b/e can't all be 0 here.
                        if c_raw > 0 && b_raw > 0 && e_raw > 0 {
                            // No grounded terminals — use standard stamp
                            mna.stamp_bjt(start_idx, c_raw - 1, b_raw - 1, e_raw - 1);
                        } else {
                            // Per-terminal ground handling.
                            // Accumulate (`+=`) so tied non-ground terminals
                            // cancel — see `stamp_nonlinear_2terminal`.
                            // N_v row 0 (Vbe): +1 at B, -1 at E
                            if b_raw > 0 {
                                mna.n_v[start_idx][b_raw - 1] += 1.0;
                            }
                            if e_raw > 0 {
                                mna.n_v[start_idx][e_raw - 1] += -1.0;
                            }
                            // N_v row 1 (Vbc): +1 at B, -1 at C
                            if b_raw > 0 {
                                mna.n_v[start_idx + 1][b_raw - 1] += 1.0;
                            }
                            if c_raw > 0 {
                                mna.n_v[start_idx + 1][c_raw - 1] += -1.0;
                            }
                            // N_i col 0 (Ic): -1 at C, +1 at E
                            if c_raw > 0 {
                                mna.n_i[c_raw - 1][start_idx] += -1.0;
                            }
                            if e_raw > 0 {
                                mna.n_i[e_raw - 1][start_idx] += 1.0;
                            }
                            // N_i col 1 (Ib): -1 at B, +1 at E
                            if b_raw > 0 {
                                mna.n_i[b_raw - 1][start_idx + 1] += -1.0;
                            }
                            if e_raw > 0 {
                                mna.n_i[e_raw - 1][start_idx + 1] += 1.0;
                            }
                        }
                    }
                }
                NonlinearDeviceType::BjtForwardActive => {
                    if node_indices.len() >= 3 {
                        let c_raw = node_indices[0];
                        let b_raw = node_indices[1];
                        let e_raw = node_indices[2];

                        // Look up BF from the netlist model for BF-scaled N_i
                        let beta_f = netlist
                            .elements
                            .iter()
                            .find_map(|e| {
                                if let crate::parser::Element::Bjt { name: n, model, .. } = e {
                                    if n.eq_ignore_ascii_case(&dev_name) {
                                        netlist
                                            .models
                                            .iter()
                                            .find(|m| m.name.eq_ignore_ascii_case(model))
                                            .and_then(|m| {
                                                m.params
                                                    .iter()
                                                    .find(|(k, _)| k.eq_ignore_ascii_case("BF"))
                                                    .map(|(_, v)| *v)
                                            })
                                    } else {
                                        None
                                    }
                                } else {
                                    None
                                }
                            })
                            .unwrap_or(200.0);

                        if c_raw > 0 && b_raw > 0 && e_raw > 0 {
                            mna.stamp_bjt_forward_active(
                                start_idx,
                                c_raw - 1,
                                b_raw - 1,
                                e_raw - 1,
                                beta_f,
                            );
                        } else {
                            // Per-terminal ground handling for 1D forward-active.
                            // Accumulate (`+=`) so tied non-ground terminals
                            // cancel — see `stamp_nonlinear_2terminal`.
                            if b_raw > 0 {
                                mna.n_v[start_idx][b_raw - 1] += 1.0;
                            }
                            if e_raw > 0 {
                                mna.n_v[start_idx][e_raw - 1] += -1.0;
                            }
                            if c_raw > 0 {
                                mna.n_i[c_raw - 1][start_idx] += -1.0;
                            }
                            if b_raw > 0 {
                                mna.n_i[b_raw - 1][start_idx] += -1.0 / beta_f;
                            }
                            if e_raw > 0 {
                                mna.n_i[e_raw - 1][start_idx] += 1.0 + 1.0 / beta_f;
                            }
                        }
                    }
                }
                NonlinearDeviceType::Jfet => {
                    // JFET: 2D — dim 0: (Vds, Id), dim 1: (Vgs, Ig)
                    //
                    // Dimension pairing for stable K diagonal (K[i][i] < 0):
                    //   dim 0: N_v row extracts Vds, N_i col injects Id (drain current drives
                    //          drain-source voltage → K[0][0] = dVds/dId < 0 always)
                    //   dim 1: N_v row extracts Vgs, N_i col injects Ig (gate current drives
                    //          gate-source voltage → K[1][1] = dVgs/dIg < 0 always)
                    //
                    // This ordering ensures K[i][i] < 0 even when source is VS-pinned,
                    // because drain always has a finite load (never VS-pinned in typical circuits).
                    // Nodes: [nd, ng, ns]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // Accumulate (`+=`) so tied terminals (e.g. gate-source
                        // strap `J1 out ctl ctl`) cancel — see `stamp_nonlinear_2terminal`.
                        // N_v row 0 (Vds): +1 at D, -1 at S
                        if d_raw > 0 {
                            mna.n_v[start_idx][d_raw - 1] += 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx][s_raw - 1] += -1.0;
                        }
                        // N_v row 1 (Vgs): +1 at G, -1 at S
                        if g_raw > 0 {
                            mna.n_v[start_idx + 1][g_raw - 1] += 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx + 1][s_raw - 1] += -1.0;
                        }

                        // N_i col 0 (Id): -1 at D (extracted), +1 at S (injected)
                        if d_raw > 0 {
                            mna.n_i[d_raw - 1][start_idx] += -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx] += 1.0;
                        }
                        // N_i col 1 (Ig): -1 at G (extracted), +1 at S (injected)
                        if g_raw > 0 {
                            mna.n_i[g_raw - 1][start_idx + 1] += -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx + 1] += 1.0;
                        }
                    }
                }
                NonlinearDeviceType::Mosfet => {
                    // MOSFET: 2D — dim 0: (Vds, Id), dim 1: (Vgs, Ig=0)
                    //
                    // Dimension pairing for stable K diagonal (K[i][i] < 0):
                    //   dim 0: N_v row extracts Vds, N_i col injects Id (drain current drives
                    //          drain-source voltage → K[0][0] = dVds/dId < 0 always, even when
                    //          source is VS-pinned, because drain always has a finite load resistor)
                    //   dim 1: N_v row extracts Vgs, N_i col injects Ig (gate current drives
                    //          gate-source voltage → K[1][1] = dVgs/dIg < 0 always because
                    //          gate always has at least a parasitic capacitor)
                    // Nodes: [nd, ng, ns, nb]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // Accumulate (`+=`) so tied terminals (e.g. G-S tie)
                        // cancel — see `stamp_nonlinear_2terminal`.
                        // N_v row 0 (Vds): +1 at D, -1 at S
                        if d_raw > 0 {
                            mna.n_v[start_idx][d_raw - 1] += 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx][s_raw - 1] += -1.0;
                        }
                        // N_v row 1 (Vgs): +1 at G, -1 at S
                        if g_raw > 0 {
                            mna.n_v[start_idx + 1][g_raw - 1] += 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx + 1][s_raw - 1] += -1.0;
                        }

                        // N_i col 0 (Id): -1 at D (extracted), +1 at S (injected)
                        if d_raw > 0 {
                            mna.n_i[d_raw - 1][start_idx] += -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx] += 1.0;
                        }
                        // N_i col 1 (Ig): effectively zero (insulated gate), but stamp for framework
                        if g_raw > 0 {
                            mna.n_i[g_raw - 1][start_idx + 1] += -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx + 1] += 1.0;
                        }
                    }
                }
                NonlinearDeviceType::Tube => {
                    // Tube device family. Three shapes:
                    //   - Triode (dim=2, 3 nodes): [grid, plate, cathode]
                    //   - Pentode (dim=3, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    //   - Grid-off pentode (dim=2, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    // The node ordering inside `node_indices` matches the layout
                    // produced by `categorize_element`. Triode vs grid-off pentode
                    // are distinguished by node_indices.len() (3 vs 4+).
                    let is_pentode_shape = node_indices.len() >= 4;
                    if dim == 2 && is_pentode_shape {
                        // Grid-off pentode: 2D NR (Vgk→Ip, Vpk→Ig2). Ig1 is
                        // dropped (grid-cutoff); Vg2k is frozen in the device
                        // math, not stamped as an N_v row.
                        let p_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let k_raw = node_indices[2];
                        let s_raw = node_indices[3]; // screen (g2)

                        // Accumulate (`+=`) so tied terminals cancel —
                        // see `stamp_nonlinear_2terminal`.
                        // N_v row 0 (Vgk): +1 at grid, -1 at cathode
                        if g_raw > 0 {
                            mna.n_v[start_idx][g_raw - 1] += 1.0;
                        }
                        if k_raw > 0 {
                            mna.n_v[start_idx][k_raw - 1] += -1.0;
                        }
                        // N_v row 1 (Vpk): +1 at plate, -1 at cathode
                        if p_raw > 0 {
                            mna.n_v[start_idx + 1][p_raw - 1] += 1.0;
                        }
                        if k_raw > 0 {
                            mna.n_v[start_idx + 1][k_raw - 1] += -1.0;
                        }

                        // N_i col 0 (Ip): -1 at plate, +1 at cathode
                        if p_raw > 0 {
                            mna.n_i[p_raw - 1][start_idx] += -1.0;
                        }
                        if k_raw > 0 {
                            mna.n_i[k_raw - 1][start_idx] += 1.0;
                        }
                        // N_i col 1 (Ig2): -1 at screen, +1 at cathode
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx + 1] += -1.0;
                        }
                        if k_raw > 0 {
                            mna.n_i[k_raw - 1][start_idx + 1] += 1.0;
                        }
                    } else if dim == 3 {
                        // Pentode 3D layout (rows / columns must agree so the
                        // 3x3 device Jacobian block stays consistent):
                        //   row/col 0: Ip   ↔ Vgk
                        //   row/col 1: Ig2  ↔ Vpk
                        //   row/col 2: Ig1  ↔ Vg2k
                        //
                        // The codegen template (device_tube.rs.tera) and the
                        // DC-OP solver consume this exact ordering — do NOT
                        // reshuffle it without updating those consumers.
                        //
                        // The optional suppressor node (node_indices[4], when
                        // present) is tied to the cathode: `categorize_element`
                        // refuses any other wiring, and no N_v / N_i entries are
                        // stamped for it. This is the universal case for audio
                        // power tubes (6L6/6V6/KT88 beam tetrodes, EL84/EL34
                        // strapped pentodes, and EF86 whose suppressor is wired
                        // to cathode externally).
                        if node_indices.len() >= 4 {
                            let p_raw = node_indices[0];
                            let g_raw = node_indices[1];
                            let k_raw = node_indices[2];
                            let s_raw = node_indices[3]; // screen (g2)

                            // Accumulate (`+=`) so tied terminals cancel —
                            // see `stamp_nonlinear_2terminal`.
                            // N_v row 0 (Vgk): +1 at grid, -1 at cathode
                            if g_raw > 0 {
                                mna.n_v[start_idx][g_raw - 1] += 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx][k_raw - 1] += -1.0;
                            }
                            // N_v row 1 (Vpk): +1 at plate, -1 at cathode
                            if p_raw > 0 {
                                mna.n_v[start_idx + 1][p_raw - 1] += 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx + 1][k_raw - 1] += -1.0;
                            }
                            // N_v row 2 (Vg2k): +1 at screen, -1 at cathode
                            if s_raw > 0 {
                                mna.n_v[start_idx + 2][s_raw - 1] += 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx + 2][k_raw - 1] += -1.0;
                            }

                            // N_i col 0 (Ip): -1 at plate (extracted), +1 at cathode (injected)
                            if p_raw > 0 {
                                mna.n_i[p_raw - 1][start_idx] += -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx] += 1.0;
                            }
                            // N_i col 1 (Ig2): -1 at screen, +1 at cathode
                            if s_raw > 0 {
                                mna.n_i[s_raw - 1][start_idx + 1] += -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx + 1] += 1.0;
                            }
                            // N_i col 2 (Ig1): -1 at grid, +1 at cathode
                            if g_raw > 0 {
                                mna.n_i[g_raw - 1][start_idx + 2] += -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx + 2] += 1.0;
                            }
                        }
                    } else if node_indices.len() >= 3 {
                        // Triode 2D — Ip (plate current) + Ig (grid current)
                        // Nodes: [ng, np, nk] (grid, plate, cathode)
                        let g_raw = node_indices[0];
                        let p_raw = node_indices[1];
                        let k_raw = node_indices[2];

                        if g_raw > 0 && p_raw > 0 && k_raw > 0 {
                            mna.stamp_triode(start_idx, g_raw - 1, p_raw - 1, k_raw - 1);
                        } else {
                            // Per-terminal ground handling.
                            // Accumulate (`+=`) so tied non-ground terminals
                            // cancel — see `stamp_nonlinear_2terminal`.
                            // N_v row 0 (Vgk): +1 at grid, -1 at cathode
                            if g_raw > 0 {
                                mna.n_v[start_idx][g_raw - 1] += 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx][k_raw - 1] += -1.0;
                            }
                            // N_v row 1 (Vpk): +1 at plate, -1 at cathode
                            if p_raw > 0 {
                                mna.n_v[start_idx + 1][p_raw - 1] += 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx + 1][k_raw - 1] += -1.0;
                            }
                            // N_i col 0 (Ip): -1 at plate, +1 at cathode
                            if p_raw > 0 {
                                mna.n_i[p_raw - 1][start_idx] += -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx] += 1.0;
                            }
                            // N_i col 1 (Ig): -1 at grid, +1 at cathode
                            if g_raw > 0 {
                                mna.n_i[g_raw - 1][start_idx + 1] += -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx + 1] += 1.0;
                            }
                        }
                    }
                }
                NonlinearDeviceType::Vca => {
                    // VCA: 2D — dim 0: signal, dim 1: control (I_ctrl = 0)
                    // 4 terminals: [sig_p, sig_n, ctrl_p, ctrl_n]
                    if node_indices.len() >= 4 {
                        let sp_raw = node_indices[0]; // sig+
                        let sn_raw = node_indices[1]; // sig-
                        let cp_raw = node_indices[2]; // ctrl+
                        let cn_raw = node_indices[3]; // ctrl-

                        // Find the VcaInfo for this device to check current_mode
                        let vca_idx = mna
                            .vcas
                            .iter()
                            .position(|v| v.name == dev_name)
                            .unwrap_or(0);
                        let is_current_mode = mna.vcas[vca_idx].current_mode;

                        if is_current_mode {
                            // CURRENT MODE (THAT 2180 style translinear):
                            //   I_out = G(Vc) * I_in
                            //
                            // Uses an internal node (sig+_int) to break the path between
                            // sig+ and sig-. The 0V sensing source between sig+ and
                            // sig+_int measures I_in. A dummy 1 Ω resistor (g_dummy = 1.0,
                            // stamped during augmented-row allocation above) terminates
                            // sig+_int to ground, holding sig+ at virtual ground — see
                            // the Rdummy divider-error discussion at the allocation site
                            // (the original 1 MΩ caused ~32 dB passband loss).
                            // N_i injects G*I_in at sig- independently — no linear
                            // pass-through from sig+ to sig-.
                            //
                            // This matches ngspice's CCCS topology:
                            //   Vsense sig+ sig+_int 0
                            //   Rdummy sig+_int 0 1
                            //   Bvca 0 sig- I={G*I(Vsense)}
                            let k = mna.vcas[vca_idx].n_sense_idx;
                            let int = mna.vcas[vca_idx].n_internal_idx;
                            if k > 0 && int > 0 {
                                let k0 = k - 1; // sense row (0-indexed)
                                let int0 = int - 1; // internal node (0-indexed)

                                // 0V sensing source between sig+ and sig+_int
                                // KVL: V(sig+) - V(sig+_int) = 0
                                if sp_raw > 0 {
                                    mna.g[k0][sp_raw - 1] += 1.0;
                                    mna.g[sp_raw - 1][k0] += 1.0;
                                }
                                mna.g[k0][int0] -= 1.0;
                                mna.g[int0][k0] -= 1.0;
                                // (Rdummy at sig+_int already stamped during allocation)

                                // N_v row 0: extract branch current (sense variable)
                                mna.n_v[start_idx][k0] += 1.0;

                                // N_i col 0: inject I_out = G*I_in at sig- ONLY
                                // No injection at sig+ — decoupled input/output
                                if sn_raw > 0 {
                                    mna.n_i[sn_raw - 1][start_idx] += 1.0;
                                }
                            }
                        } else {
                            // VOLTAGE MODE (original): I_out = G(Vc) * V_sig
                            // Accumulate (`+=`) so tied terminals cancel —
                            // see `stamp_nonlinear_2terminal`.
                            // N_v row 0 (V_signal): +1 at sig+, -1 at sig-
                            if sp_raw > 0 {
                                mna.n_v[start_idx][sp_raw - 1] += 1.0;
                            }
                            if sn_raw > 0 {
                                mna.n_v[start_idx][sn_raw - 1] += -1.0;
                            }
                            // N_i col 0 (I_signal): -1 at sig+ (extracted), +1 at sig- (injected)
                            if sp_raw > 0 {
                                mna.n_i[sp_raw - 1][start_idx] += -1.0;
                            }
                            if sn_raw > 0 {
                                mna.n_i[sn_raw - 1][start_idx] += 1.0;
                            }
                        }

                        // N_v row 1 (V_control): +1 at ctrl+, -1 at ctrl- (same for both modes)
                        if cp_raw > 0 {
                            mna.n_v[start_idx + 1][cp_raw - 1] += 1.0;
                        }
                        if cn_raw > 0 {
                            mna.n_v[start_idx + 1][cn_raw - 1] += -1.0;
                        }
                        // N_i col 1 (I_control): NO stamping — control draws no current
                    }
                }
            }
        }

        // --- Voltage-mode VCA washout diagnostic (compile-time WARN only) ---
        //
        // A voltage-mode VCA (MODE=0, the default) models I_sig = G(Vc)·V_sig
        // with G(Vc) in siemens. When its signal-input node is driven through a
        // series/source resistance R_drive such that R_drive·G0 ≫ 1, the very
        // high transconductance clamps V_sig ≈ 0 (near-virtual-ground) and the
        // stage degenerates into a fixed passive network (e.g. a −Rfb/Rdrive
        // inverter) whose gain is INDEPENDENT of the control voltage — the CV
        // silently washes out. The correct model for a current-drive Blackmer
        // topology is MODE=1 (current-mode). This warning surfaces the footgun;
        // it changes no generated code.
        //
        // R_drive estimate: the Thévenin resistance looking out of a signal
        // terminal into the rest of the linear network = 1 / G[node][node]
        // (the diagonal conductance sums every resistor tied to that node). A
        // voltage-mode VCA stamps NOTHING into G at its signal nodes (only the
        // current-mode path adds a dummy conductance at an internal node), so
        // the diagonal is exactly the source conductance the transconductance
        // competes against. We take the larger Thévenin R over the two
        // non-ground signal terminals (the weakest-driven terminal governs the
        // washout). Nodes with zero resistive diagonal (purely cap/inductor
        // coupled at DC) are skipped — the DC G-matrix estimator does not apply.
        for vca in &mna.vcas {
            if vca.current_mode {
                continue;
            }
            let mut r_drive: Option<f64> = None;
            for node_1idx in [vca.n_sig_p_idx, vca.n_sig_n_idx] {
                if node_1idx == 0 {
                    continue; // ground terminal
                }
                let diag = mna.g[node_1idx - 1][node_1idx - 1];
                if diag > 0.0 {
                    let r_thev = 1.0 / diag;
                    r_drive = Some(r_drive.map_or(r_thev, |prev| prev.max(r_thev)));
                }
            }
            if let Some(r_drive) = r_drive {
                let metric = r_drive * vca.g0;
                if metric >= 10.0 {
                    crate::diag_warn!(
                        "VCA {} voltage-mode with R_drive·G0 ≈ {:.1} ≫ 1: \
                         control-voltage response is suppressed by the source \
                         resistance — did you mean MODE=1 (current-mode)?",
                        vca.name,
                        metric
                    );
                }
            }
        }

        Ok(mna)
    }
}
