//! Parasitic junction capacitances for capacitor-free nonlinear circuits.

use super::*;

impl MnaSystem {
    /// Add parasitic junction capacitances across nonlinear device terminals.
    ///
    /// Stamps a small capacitor across each physical junction of every nonlinear
    /// device. This models real semiconductor junction capacitance and ensures
    /// the C matrix is non-trivial for purely resistive nonlinear circuits,
    /// preventing the trapezoidal-rule A matrix from being singular.
    ///
    /// Junction topology per device type:
    /// - **Diode**: anode-cathode (Cj)
    /// - **BJT**: base-emitter (Cje) + base-collector (Cjc)
    /// - **JFET**: gate-source (Cgs) + gate-drain (Cgd)
    /// - **MOSFET**: gate-source (Cgs) + gate-drain (Cgd)
    /// - **Triode** (Tube, dim=2): grid-cathode (Cgk) + plate-cathode (Cpk)
    /// - **Pentode** (Tube, dim=3): grid-cathode (Cgk) + grid-plate (Cgp,
    ///   Miller cap) + plate-cathode (Cpk) + screen-cathode (Csk) +
    ///   screen-plate (Csp). Suppressor (if present) is cathode-tied (any other
    ///   wiring is refused) and contributes no extra parasitic caps. TODO(phase 1b):
    ///   honor user-provided explicit Cgk/Cgp/Cpk/Csk/Csp from `.model`.
    ///
    /// Uses [`PARASITIC_CAP`] (10pF) and [`stamp_capacitor_raw`](Self::stamp_capacitor_raw).
    pub fn add_parasitic_caps(&mut self) {
        for (name, node_a, node_b) in self.parasitic_junctions() {
            self.stamp_capacitor_raw(node_a, node_b, PARASITIC_CAP);
            log::debug!(
                "Parasitic cap {}: node({})-node({}) = {:.0e} F",
                name,
                node_a,
                node_b,
                PARASITIC_CAP,
            );
        }
    }

    /// Whether a build adds [`PARASITIC_CAP`] across every device junction:
    /// the circuit has nonlinear devices and no capacitance at all, so the
    /// trapezoidal `A = G + (2/T)C` would be `G` and the integrator would
    /// have no state. The one test every build path uses.
    pub fn needs_parasitic_caps(&self) -> bool {
        self.m > 0 && !self.c.iter().any(|row| row.iter().any(|&v| v != 0.0))
    }

    /// The capacitors [`add_parasitic_caps`](Self::add_parasitic_caps)
    /// stamps, by device and node name: what a SPICE twin of the deck needs
    /// to simulate the same circuit.
    ///
    /// Device terminals are always netlist nodes (internal-node expansion
    /// does not rewrite them), so every name is the netlist's own.
    pub fn parasitic_caps(&self) -> Vec<ParasiticCap> {
        let name = |idx: usize| -> String {
            if idx == 0 {
                return "0".to_string();
            }
            self.node_map
                .iter()
                .find(|(_, &i)| i == idx)
                .map(|(n, _)| n.clone())
                .unwrap_or_default()
        };
        self.parasitic_junctions()
            .into_iter()
            .map(|(device, a, b)| ParasiticCap {
                device,
                node_a: name(a),
                node_b: name(b),
            })
            .collect()
    }

    /// Junctions that receive a parasitic cap: (device, node, node), node
    /// indices 1-based with 0 = ground.
    fn parasitic_junctions(&self) -> Vec<(String, usize, usize)> {
        let mut junctions: Vec<(String, usize, usize)> = Vec::new();

        for dev in &self.nonlinear_devices {
            match dev.device_type {
                NonlinearDeviceType::Diode => {
                    // node_indices: [anode, cathode]
                    junctions.push((dev.name.clone(), dev.node_indices[0], dev.node_indices[1]));
                }
                NonlinearDeviceType::Bjt | NonlinearDeviceType::BjtForwardActive => {
                    // node_indices: [collector, base, emitter]
                    let (nc, nb, ne) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    // B-E junction (Cje)
                    junctions.push((dev.name.clone(), nb, ne));
                    // B-C junction (Cjc) — still present even for forward-active (linear cap)
                    junctions.push((dev.name.clone(), nb, nc));
                }
                NonlinearDeviceType::Jfet => {
                    // node_indices: [drain, gate, source]
                    let (nd, ng, ns) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    // G-S junction (Cgs)
                    junctions.push((dev.name.clone(), ng, ns));
                    // G-D junction (Cgd)
                    junctions.push((dev.name.clone(), ng, nd));
                }
                NonlinearDeviceType::Mosfet => {
                    // node_indices: [drain, gate, source, bulk]
                    let (nd, ng, ns) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    // G-S junction (Cgs)
                    junctions.push((dev.name.clone(), ng, ns));
                    // G-D junction (Cgd)
                    junctions.push((dev.name.clone(), ng, nd));
                }
                NonlinearDeviceType::Tube => {
                    // Tube device family. Three shapes:
                    //   - Triode (dim=2, 3 nodes): [grid, plate, cathode]
                    //   - Pentode (dim=3, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    //   - Grid-off pentode (dim=2, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    // Node layout differs — see the `Element::Triode` /
                    // `Element::Pentode` arms in `categorize_element`.
                    // Suppressor is cathode-tied in phase 1a and gets no caps.
                    let is_pentode_shape = dev.node_indices.len() >= 4;
                    if dev.dimension == 3 && is_pentode_shape {
                        // Sharp-cutoff pentode: 5 junction caps.
                        // Cgk, Cgp, Cpk, Csk, Csp.
                        let np = dev.node_indices[0];
                        let ng = dev.node_indices[1];
                        let nk = dev.node_indices[2];
                        let nscr = dev.node_indices[3];
                        // Grid-cathode (Cgk)
                        junctions.push((dev.name.clone(), ng, nk));
                        // Grid-plate (Cgp, Miller)
                        junctions.push((dev.name.clone(), ng, np));
                        // Plate-cathode (Cpk)
                        junctions.push((dev.name.clone(), np, nk));
                        // Screen-cathode (Csk)
                        junctions.push((dev.name.clone(), nscr, nk));
                        // Screen-plate (Csp)
                        junctions.push((dev.name.clone(), nscr, np));
                    } else if dev.dimension == 2 && is_pentode_shape {
                        // Grid-off pentode: screen is an input, not an NR
                        // unknown, so only 3 junction caps are meaningful.
                        // Drop CSK/CSP; keep CGK (input), CGP (Miller), CPK.
                        let np = dev.node_indices[0];
                        let ng = dev.node_indices[1];
                        let nk = dev.node_indices[2];
                        // Grid-cathode (Cgk)
                        junctions.push((dev.name.clone(), ng, nk));
                        // Grid-plate (Cgp, Miller)
                        junctions.push((dev.name.clone(), ng, np));
                        // Plate-cathode (Cpk)
                        junctions.push((dev.name.clone(), np, nk));
                    } else {
                        // Triode: nodes are [grid, plate, cathode].
                        let (ng, np, nk) = (
                            dev.node_indices[0],
                            dev.node_indices[1],
                            dev.node_indices[2],
                        );
                        // Grid-cathode (Cgk)
                        junctions.push((dev.name.clone(), ng, nk));
                        // Plate-cathode (Cpk)
                        junctions.push((dev.name.clone(), np, nk));
                    }
                }
                NonlinearDeviceType::Vca => {
                    // node_indices: [sig_p, sig_n, ctrl_p, ctrl_n]
                    // Signal path junction (sig+ to sig-)
                    junctions.push((dev.name.clone(), dev.node_indices[0], dev.node_indices[1]));
                }
                NonlinearDeviceType::Ldr => {
                    // node_indices: [r+, r-, ctrl+, ctrl-]. Resistance path
                    // (r+ to r-) — mirrors the VCA signal-path parasitic so an
                    // otherwise cap-free resistive LDR deck stays DK-conditioned.
                    junctions.push((dev.name.clone(), dev.node_indices[0], dev.node_indices[1]));
                }
                NonlinearDeviceType::Glow => {
                    // node_indices: [a, k]. Resistance path (a to k) — same
                    // parasitic as the LDR so a cap-free glow deck stays
                    // DK-conditioned.
                    junctions.push((dev.name.clone(), dev.node_indices[0], dev.node_indices[1]));
                }
            }
        }

        junctions
    }
}

/// A [`PARASITIC_CAP`] a build adds across a device junction of a
/// capacitor-free nonlinear circuit ([`MnaSystem::needs_parasitic_caps`]). It
/// is part of the simulated circuit, so a SPICE twin of the deck needs it too.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParasiticCap {
    /// The device whose junction carries it.
    pub device: String,
    /// One terminal node, by name ("0" for ground).
    pub node_a: String,
    /// The other terminal node.
    pub node_b: String,
}

/// Parasitic junction capacitance for nonlinear device stabilization \[F\].
///
/// 10pF is representative of small-signal semiconductor junction capacitances
/// (typical Cj = 2-10pF for diodes, Cbc = 2-8pF for BJTs). At audio
/// frequencies this is negligible (10pF @ 20kHz = ~800kOhm impedance) but
/// provides enough reactance for the trapezoidal-rule DK discretization to
/// form a well-conditioned A matrix (2C/T ~ 8.8e-7 at 44.1kHz).
///
/// Caps are stamped *across device junctions* (not node-to-ground) to model
/// physical junction capacitance without introducing artificial ground coupling.
pub const PARASITIC_CAP: f64 = 10e-12;
