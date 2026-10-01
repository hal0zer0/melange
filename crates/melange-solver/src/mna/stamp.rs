//! Element stamps: capacitors, junction capacitances, input conductance and nonlinear-device N_v/N_i.

use super::*;

impl MnaSystem {
    /// Stamp a resistor between two nodes.
    ///
    /// G[i,i] += g, G[j,j] += g, G[i,j] -= g, G[j,i] -= g
    #[deprecated(
        since = "0.1.14",
        note = "unused: the MNA builder does not call it (its indices are raw 0-based \
                matrix rows, with no ground handling). It will be removed in the next release."
    )]
    pub fn stamp_resistor(&mut self, i: usize, j: usize, resistance: f64) {
        if resistance == 0.0 {
            return; // Short circuit - handled differently
        }
        let g = 1.0 / resistance;
        self.g[i][i] += g;
        self.g[j][j] += g;
        self.g[i][j] -= g;
        self.g[j][i] -= g;
    }

    /// Stamp a capacitor between two nodes.
    pub fn stamp_capacitor(&mut self, i: usize, j: usize, capacitance: f64) {
        self.c[i][i] += capacitance;
        self.c[j][j] += capacitance;
        self.c[i][j] -= capacitance;
        self.c[j][i] -= capacitance;
    }

    /// Stamp a capacitor between two 1-indexed nodes (0 = ground).
    ///
    /// Unlike `stamp_capacitor` which takes 0-indexed node numbers (MNA internal),
    /// this takes netlist-style 1-indexed nodes where 0 means ground. This is
    /// useful for stamping junction capacitances computed from device node indices.
    pub fn stamp_capacitor_raw(&mut self, node_a: usize, node_b: usize, cap: f64) {
        if cap <= 0.0 {
            return;
        }
        match (node_a > 0, node_b > 0) {
            (true, true) => {
                let (a, b) = (node_a - 1, node_b - 1);
                self.c[a][a] += cap;
                self.c[b][b] += cap;
                self.c[a][b] -= cap;
                self.c[b][a] -= cap;
            }
            (true, false) => {
                self.c[node_a - 1][node_a - 1] += cap;
            }
            (false, true) => {
                self.c[node_b - 1][node_b - 1] += cap;
            }
            (false, false) => {}
        }
    }

    /// Stamp junction capacitances from device model parameters into the C matrix.
    ///
    /// Reads cap fields from each `DeviceSlot`'s params and stamps them across
    /// the appropriate device junctions. Only stamps non-zero values.
    ///
    /// Must be called BEFORE building the DK kernel (so caps are included in A = G + 2C/T).
    ///
    /// Device slots must be in the same order as `self.nonlinear_devices`.
    pub fn stamp_device_junction_caps(&mut self, device_slots: &[crate::device_types::DeviceSlot]) {
        use crate::device_types::DeviceParams;

        // Collect (node_a, node_b, cap) tuples first to avoid borrow conflict
        // (self.nonlinear_devices borrowed immutably, self.stamp_capacitor_raw borrows mutably).
        let mut caps: Vec<(usize, usize, f64)> = Vec::new();

        for (dev_info, slot) in self.nonlinear_devices.iter().zip(device_slots.iter()) {
            match &slot.params {
                DeviceParams::Tube(p) => {
                    // Node layout differs by tube shape (same dispatch as
                    // `add_parasitic_caps`; see `categorize_element`):
                    //   - Triode (3 nodes):      [grid, plate, cathode]
                    //   - Pentode (4-5 nodes):   [plate, grid, cathode, screen, (suppressor?)]
                    // Without the dispatch a pentode's CCG landed
                    // plate-cathode and CCP grid-cathode (swapped).
                    let (ng, np, nk) = if dev_info.node_indices.len() >= 4 {
                        (
                            dev_info.node_indices[1],
                            dev_info.node_indices[0],
                            dev_info.node_indices[2],
                        )
                    } else {
                        (
                            dev_info.node_indices[0],
                            dev_info.node_indices[1],
                            dev_info.node_indices[2],
                        )
                    };
                    if p.ccg > 0.0 {
                        caps.push((nk, ng, p.ccg));
                    }
                    if p.cgp > 0.0 {
                        caps.push((ng, np, p.cgp));
                    }
                    if p.ccp > 0.0 {
                        caps.push((nk, np, p.ccp));
                    }
                }
                DeviceParams::Bjt(p) => {
                    // node_indices: [collector, base, emitter] (1-indexed, 0=ground)
                    let (nc, nb, ne) = (
                        dev_info.node_indices[0],
                        dev_info.node_indices[1],
                        dev_info.node_indices[2],
                    );
                    // If internal nodes exist, stamp caps at internal nodes (not external)
                    let int = self
                        .bjt_internal_nodes
                        .iter()
                        .find(|n| n.start_idx == slot.start_idx);
                    // CJE: base-emitter junction
                    if p.cje > 0.0 {
                        let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                        let cap_e = int.and_then(|n| n.int_emitter).map(|i| i + 1).unwrap_or(ne);
                        caps.push((cap_b, cap_e, p.cje));
                    }
                    // CJC: base-collector junction
                    if p.cjc > 0.0 {
                        let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                        let cap_c = int
                            .and_then(|n| n.int_collector)
                            .map(|i| i + 1)
                            .unwrap_or(nc);
                        caps.push((cap_b, cap_c, p.cjc));
                    }
                }
                DeviceParams::Jfet(p) => {
                    // node_indices: [drain, gate, source]
                    let (nd, ng, ns) = (
                        dev_info.node_indices[0],
                        dev_info.node_indices[1],
                        dev_info.node_indices[2],
                    );
                    if p.cgs > 0.0 {
                        caps.push((ng, ns, p.cgs));
                    }
                    if p.cgd > 0.0 {
                        caps.push((ng, nd, p.cgd));
                    }
                }
                DeviceParams::Mosfet(p) => {
                    // node_indices: [drain, gate, source, bulk]
                    let (nd, ng, ns) = (
                        dev_info.node_indices[0],
                        dev_info.node_indices[1],
                        dev_info.node_indices[2],
                    );
                    if p.cgs > 0.0 {
                        caps.push((ng, ns, p.cgs));
                    }
                    if p.cgd > 0.0 {
                        caps.push((ng, nd, p.cgd));
                    }
                }
                DeviceParams::Diode(p) => {
                    // node_indices: [anode, cathode]
                    let (na, nc) = (dev_info.node_indices[0], dev_info.node_indices[1]);
                    if p.cjo > 0.0 {
                        caps.push((na, nc, p.cjo));
                    }
                }
                DeviceParams::Vca(_) => {
                    // VCA has no junction capacitances
                }
                DeviceParams::Ldr(_) => {
                    // LDR is a pure (variable) resistor — no junction cap.
                }
                DeviceParams::Glow(_) => {
                    // Glow lamp is a pure (latched) resistor — no junction cap.
                }
            }
        }

        for (node_a, node_b, cap) in &caps {
            self.stamp_capacitor_raw(*node_a, *node_b, *cap);
            log::debug!(
                "Junction cap: node({})-node({}) = {:.2e} F",
                node_a,
                node_b,
                cap,
            );
        }
    }

    /// Re-linearize BJT junction capacitances at the supplied DC operating
    /// point. Must be called AFTER `stamp_device_junction_caps` has set the
    /// zero-bias CJE/CJC baseline, and AFTER the DC OP has been solved.
    ///
    /// For each BJT with a non-trivial charge-storage profile (any of
    /// `CJE`, `CJC`, `TF` non-zero, or `VJE`/`MJE`/`VJC`/`MJC`/`FC` deviating
    /// from SPICE defaults) this computes the voltage-dependent depletion
    /// cap at `Vbe_op`/`Vbc_op` plus the diffusion cap `TF·d(I_F/qb)/dVbe`, and
    /// stamps the *delta* relative to the zero-bias baseline into `self.c`.
    ///
    /// Delta stamping lets us avoid a clear-and-restamp pass — the C matrix
    /// keeps whatever upstream contributions have been accumulated (device
    /// parasitic caps, user capacitors, parasitic auto-insertion, etc.) and
    /// only the BJT junction row/col entries move by the voltage-dependent
    /// correction. When the circuit has no forward-biased junctions (bias
    /// at zero), the delta is zero and the call is a no-op, preserving the
    /// pre-2026-04-21 behaviour byte-for-byte.
    ///
    /// Augmented-MNA systems are supported: `v_nl` already carries the
    /// appropriate node-difference voltages (it is `N_v · v_node`), so no
    /// node-vector re-indexing is needed.
    ///
    /// `v_nl` is the M-vector of nonlinear controlling voltages and `v_node`
    /// the node voltages (from `DcOpResult`). For 2D BJTs
    /// `v_nl[start_idx] = Vbe` and `v_nl[start_idx+1] = Vbc`; a
    /// forward-active (1D) BJT tracks only Vbe, so its Vbc is read from
    /// the node voltages: its caps are evaluated at the bias it sits at.
    pub fn relinearize_bjt_caps_at_dc_op(
        &mut self,
        device_slots: &[crate::device_types::DeviceSlot],
        v_nl: &[f64],
        v_node: &[f64],
    ) {
        use crate::device_types::{DeviceParams, DeviceType};

        let mut deltas: Vec<(usize, usize, f64)> = Vec::new();

        for (dev_info, slot) in self.nonlinear_devices.iter().zip(device_slots.iter()) {
            let bp = match &slot.params {
                DeviceParams::Bjt(bp) => bp,
                _ => continue,
            };
            if bp.cje == 0.0 && bp.cjc == 0.0 && bp.tf == 0.0 {
                continue;
            }

            // node_indices: [c, b, e] (1-indexed, 0 = ground). If parasitic-R
            // internal nodes were expanded, junction caps live between the
            // primed nodes instead.
            let (nc, nb, ne) = (
                dev_info.node_indices[0],
                dev_info.node_indices[1],
                dev_info.node_indices[2],
            );
            let v_at = |idx: usize| {
                if idx > 0 {
                    v_node.get(idx - 1).copied().unwrap_or(0.0)
                } else {
                    0.0
                }
            };

            let vbe_op = v_nl.get(slot.start_idx).copied().unwrap_or(0.0);
            let vbc_op = match slot.device_type {
                DeviceType::BjtForwardActive => v_at(nb) - v_at(nc),
                _ => v_nl.get(slot.start_idx + 1).copied().unwrap_or(0.0),
            };

            let (cbe_eff, cbc_eff) = bp.linearized_junction_caps(vbe_op, vbc_op);
            let int = self
                .bjt_internal_nodes
                .iter()
                .find(|n| n.start_idx == slot.start_idx);

            let cbe_delta = cbe_eff - bp.cje;
            if cbe_delta != 0.0 {
                let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                let cap_e = int.and_then(|n| n.int_emitter).map(|i| i + 1).unwrap_or(ne);
                deltas.push((cap_b, cap_e, cbe_delta));
            }
            let cbc_delta = cbc_eff - bp.cjc;
            if cbc_delta != 0.0 {
                let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                let cap_c = int
                    .and_then(|n| n.int_collector)
                    .map(|i| i + 1)
                    .unwrap_or(nc);
                deltas.push((cap_b, cap_c, cbc_delta));
            }
        }

        for (node_a, node_b, delta) in deltas {
            self.stamp_signed_cap_delta(node_a, node_b, delta);
        }
    }

    /// Signed cap-shaped stamp used by `relinearize_bjt_caps_at_dc_op` to
    /// adjust a previously-stamped zero-bias cap by a voltage-dependent
    /// delta. Bypasses `stamp_capacitor_raw`'s `cap <= 0` early return so a
    /// reverse-bias junction (whose effective cap is smaller than zero-bias
    /// CJE/CJC) can have its contribution reduced. The final C matrix entry
    /// will still be non-negative because the SPICE depletion formula never
    /// returns a value smaller than `CJ · (1 - 0.95·FC)^(-MJ)` over the
    /// relevant voltage range.
    fn stamp_signed_cap_delta(&mut self, node_a: usize, node_b: usize, cap: f64) {
        if cap == 0.0 {
            return;
        }
        match (node_a > 0, node_b > 0) {
            (true, true) => {
                let (a, b) = (node_a - 1, node_b - 1);
                self.c[a][a] += cap;
                self.c[b][b] += cap;
                self.c[a][b] -= cap;
                self.c[b][a] -= cap;
            }
            (true, false) => self.c[node_a - 1][node_a - 1] += cap,
            (false, true) => self.c[node_b - 1][node_b - 1] += cap,
            (false, false) => {}
        }
    }

    /// Stamp input conductance to ground at a node.
    ///
    /// This represents the Thevenin equivalent of the input source:
    /// a voltage source in series with a resistance.
    /// The conductance g = 1/R is stamped from the node to ground.
    pub fn stamp_input_conductance(&mut self, node: usize, conductance: f64) {
        if conductance > 0.0 {
            self.g[node][node] += conductance;
        }
    }

    /// Stamp a two-terminal nonlinear device (diode).
    ///
    /// The device current flows from anode (i) to cathode (j).
    /// Controlling voltage is V_i - V_j.
    ///
    /// N_v/N_i entries ACCUMULATE (`+=`) into the device-owned rows/columns
    /// (freshly zeroed at MNA construction) so that two terminals tied to the
    /// SAME node cancel to 0 instead of the second write overwriting the
    /// first. This applies to every device stamp below — a diode-connected
    /// BJT (`Q1 x x e`), a gate-source-strapped JFET, a MOSFET G-S tie, or a
    /// triode grid-cathode tie must see a zero controlling-voltage row
    /// (V = 0 exactly) and a cancelled KCL column, not a phantom ±V(x).
    pub fn stamp_nonlinear_2terminal(&mut self, device_idx: usize, i: usize, j: usize) {
        // N_v extracts controlling voltage: v_nl = v_i - v_j
        self.n_v[device_idx][i] += 1.0;
        self.n_v[device_idx][j] += -1.0;

        // N_i injects current: i_i = -i_nl, i_j = i_nl
        self.n_i[i][device_idx] += -1.0;
        self.n_i[j][device_idx] += 1.0;
    }

    /// Stamp a BJT (2-dimensional device).
    ///
    /// Controlling voltages: Vbe (base - emitter), Vbc (base - collector)
    /// Currents: Ic (collector current), Ib (base current)
    ///
    /// start_idx: starting row/column in N_v/N_i for this device
    pub fn stamp_bjt(&mut self, start_idx: usize, nc: usize, nb: usize, ne: usize) {
        // Accumulate (`+=`) so tied terminals (e.g. diode-connected Q1 x x e)
        // cancel to a zero row/column — see `stamp_nonlinear_2terminal`.
        // Row start_idx: Vbe = Vb - Ve
        self.n_v[start_idx][nb] += 1.0;
        self.n_v[start_idx][ne] += -1.0;

        // Row start_idx+1: Vbc = Vb - Vc
        self.n_v[start_idx + 1][nb] += 1.0;
        self.n_v[start_idx + 1][nc] += -1.0;

        // Column start_idx: Ic enters collector, exits emitter
        // N_i convention: positive = current entering node from device
        self.n_i[nc][start_idx] += -1.0; // Ic enters collector (current into device = negative)
        self.n_i[ne][start_idx] += 1.0; // Ic exits emitter (current out of device = positive by KCL)

        // Column start_idx+1: Ib enters base, exits emitter
        self.n_i[nb][start_idx + 1] += -1.0; // Ib enters base
        self.n_i[ne][start_idx + 1] += 1.0; // Ib exits emitter (KCL conservation)
    }

    /// Stamp forward-active BJT nonlinear matrices (1D).
    ///
    /// Only tracks Vbe→Ic. Base current Ib = Ic/BF is folded into N_i,
    /// so KCL at all three terminals is satisfied with a single NR dimension.
    ///
    /// N_v: single row extracting Vbe = Vb - Ve
    /// N_i: single column with Ic at collector, Ic/BF at base, (Ic + Ic/BF) at emitter
    pub fn stamp_bjt_forward_active(
        &mut self,
        start_idx: usize,
        nc: usize,
        nb: usize,
        ne: usize,
        beta_f: f64,
    ) {
        // Accumulate (`+=`) so tied terminals cancel — see `stamp_nonlinear_2terminal`.
        // N_v: extract Vbe = Vb - Ve
        self.n_v[start_idx][nb] += 1.0;
        self.n_v[start_idx][ne] += -1.0;

        // N_i: single column with Ic + Ib = Ic * (1 + 1/BF) for KCL
        self.n_i[nc][start_idx] += -1.0; // Ic extracted from collector
        self.n_i[nb][start_idx] += -1.0 / beta_f; // Ib = Ic/BF extracted from base
        self.n_i[ne][start_idx] += 1.0 + 1.0 / beta_f; // Ic + Ib injected into emitter
    }

    /// Stamp triode nonlinear matrices.
    ///
    /// Triode is 2D per device:
    /// - Dimension 0: Ip (plate current), controlled by Vgk
    /// - Dimension 1: Ig (grid current), controlled by Vpk (for Jacobian coupling)
    ///
    /// N_v extracts controlling voltages:
    /// - Row start_idx: Vgk = V_grid - V_cathode
    /// - Row start_idx+1: Vpk = V_plate - V_cathode
    ///
    /// N_i injects currents:
    /// - Column start_idx: Ip flows plate→cathode
    /// - Column start_idx+1: Ig flows grid→cathode
    pub fn stamp_triode(&mut self, start_idx: usize, ng: usize, np: usize, nk: usize) {
        // Accumulate (`+=`) so tied terminals (e.g. grid-cathode strap)
        // cancel — see `stamp_nonlinear_2terminal`.
        // Row start_idx: Vgk = V_grid - V_cathode
        self.n_v[start_idx][ng] += 1.0;
        self.n_v[start_idx][nk] += -1.0;

        // Row start_idx+1: Vpk = V_plate - V_cathode
        self.n_v[start_idx + 1][np] += 1.0;
        self.n_v[start_idx + 1][nk] += -1.0;

        // Column start_idx: Ip enters plate, exits cathode
        self.n_i[np][start_idx] += -1.0; // Ip enters plate (current into device)
        self.n_i[nk][start_idx] += 1.0; // Ip exits cathode

        // Column start_idx+1: Ig enters grid, exits cathode
        self.n_i[ng][start_idx + 1] += -1.0; // Ig enters grid
        self.n_i[nk][start_idx + 1] += 1.0; // Ig exits cathode
    }
}
