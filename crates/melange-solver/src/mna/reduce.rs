//! Stamps for BJTs and triodes linearized out of the nonlinear system.

use super::*;

impl MnaSystem {
    /// Stamp linearized BJT small-signal Jacobians into G matrix.
    ///
    /// Must be called AFTER DC OP computation provides the Jacobian.
    /// Each linearized BJT's terminal currents are linear in
    /// (Vbe, Vbc) = (Vb - Ve, Vb - Vc):
    /// - collector: dIc/dVbe·Vbe + dIc/dVbc·Vbc
    /// - base:      dIb/dVbe·Vbe + dIb/dVbc·Vbc
    /// - emitter:   -(collector + base)
    ///
    /// plus its junction capacitances into C and DC bias currents as current
    /// source injections.
    pub fn stamp_linearized_bjts(&mut self) {
        for bjt in &self.linearized_bjts.clone() {
            let nc = bjt.nc; // 1-indexed
            let nb = bjt.nb;
            let ne = bjt.ne;

            // Current drawn out of each terminal's node, per volt of Vbe and Vbc.
            let terminals = [
                (nc, bjt.dic_dvbe, bjt.dic_dvbc),
                (nb, bjt.dib_dvbe, bjt.dib_dvbc),
                (
                    ne,
                    -(bjt.dic_dvbe + bjt.dib_dvbe),
                    -(bjt.dic_dvbc + bjt.dib_dvbc),
                ),
            ];
            for &(row, d_vbe, d_vbc) in &terminals {
                if row == 0 {
                    continue;
                }
                // ∂/∂Vb = d_vbe + d_vbc, ∂/∂Ve = -d_vbe, ∂/∂Vc = -d_vbc;
                // a ground column drops out (its voltage is 0).
                for (col, val) in [(nb, d_vbe + d_vbc), (ne, -d_vbe), (nc, -d_vbc)] {
                    if col > 0 {
                        self.g[row - 1][col - 1] += val;
                    }
                }
            }

            // Junction capacitances, between the external terminals as the
            // route that does not expand parasitic-BJT internal nodes places
            // them (RB/RC/RE in series with a junction cap put its pole in
            // the MHz range).
            self.stamp_capacitor_raw(nb, ne, bjt.cbe);
            self.stamp_capacitor_raw(nb, nc, bjt.cbc);

            // DC bias injections — proper Norton companion constants.
            //
            // The stamps above implement a linear terminal-current model
            // I_lin(v) evaluated against FULL node voltages, not deviations
            // from the operating point. The exact companion of
            // I(v) ≈ I0 + J·(v − v0) therefore needs the constant injection
            // I_lin(v0) − I0 at each terminal (current INTO the node), so
            // that v = v0 is exactly the DC fixed point of the linearized
            // circuit. Injecting raw ±I0 alone (the pre-2026-07 behavior)
            // left the huge g·v0 term uncancelled — e.g. a 12AX7-class gm
            // against Vbe0 ≈ 0.65 V produced multi-mA KCL error at the
            // collector row and tens of volts of bias shift.
            //
            // vbe0/vbc0 are node differences with a grounded terminal at
            // 0 V, so I_lin(v0) is exact whichever terminals are grounded.
            let vbe0 = bjt.vbe0;
            let vbc0 = bjt.vbc0;
            let i_lin_c = bjt.dic_dvbe * vbe0 + bjt.dic_dvbc * vbc0;
            let i_lin_b = bjt.dib_dvbe * vbe0 + bjt.dib_dvbc * vbc0;
            let i_lin_e = -(i_lin_c + i_lin_b);

            // Terminal DC currents drawn OUT of each node by the real device:
            // collector ic_dc, base ib_dc, emitter -(ic_dc + ib_dc).
            // Injection INTO node = I_lin(v0) − I_dc(terminal).
            // (CurrentSourceInfo convention: dc_value injected at n_plus_idx,
            // extracted at n_minus_idx — see dc_op.rs / dk.rs consumers.)
            if nc > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ic_dc", bjt.name),
                    n_plus_idx: nc,
                    n_minus_idx: 0,
                    dc_value: i_lin_c - bjt.ic_dc,
                });
            }
            if nb > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ib_dc", bjt.name),
                    n_plus_idx: nb,
                    n_minus_idx: 0,
                    dc_value: i_lin_b - bjt.ib_dc,
                });
            }
            if ne > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ie_dc", bjt.name),
                    n_plus_idx: ne,
                    n_minus_idx: 0,
                    dc_value: i_lin_e + bjt.ic_dc + bjt.ib_dc,
                });
            }
        }
    }

    /// Stamp linearized triode small-signal conductances into G matrix.
    ///
    /// Must be called AFTER DC OP computation provides the g-parameters.
    /// Each linearized triode gets:
    /// - gm (VCCS): Ip = gm * Vgk, current flows P→K controlled by G-K voltage
    /// - gp (conductance): 1/rp between plate and cathode
    /// - DC bias currents as current source injections
    pub fn stamp_linearized_triodes(&mut self) {
        for tube in &self.linearized_triodes.clone() {
            let ng = tube.ng; // 1-indexed (0 = ground)
            let np = tube.np;
            let nk = tube.nk;

            // gm: VCCS — Ip = gm * Vgk, current from plate to cathode, controlled by G-K
            // Stamp: G[p][g] += gm, G[p][k] -= gm, G[k][g] -= gm, G[k][k] += gm
            if tube.gm.abs() > 1e-30 {
                if np > 0 && ng > 0 {
                    self.g[np - 1][ng - 1] += tube.gm;
                }
                if np > 0 && nk > 0 {
                    self.g[np - 1][nk - 1] -= tube.gm;
                }
                if nk > 0 && ng > 0 {
                    self.g[nk - 1][ng - 1] -= tube.gm;
                }
                if nk > 0 {
                    self.g[nk - 1][nk - 1] += tube.gm;
                }
            }

            // gp = 1/rp: plate conductance between plate and cathode
            // Stamp: G[p][p] += gp, G[k][k] += gp, G[p][k] -= gp, G[k][p] -= gp
            if tube.gp.abs() > 1e-30 {
                if np > 0 && nk > 0 {
                    let p = np - 1;
                    let k = nk - 1;
                    self.g[p][p] += tube.gp;
                    self.g[k][k] += tube.gp;
                    self.g[p][k] -= tube.gp;
                    self.g[k][p] -= tube.gp;
                } else if np > 0 {
                    self.g[np - 1][np - 1] += tube.gp;
                } else if nk > 0 {
                    self.g[nk - 1][nk - 1] += tube.gp;
                }
            }

            // DC bias injections — proper Norton companion constants.
            // Same invariant as `stamp_linearized_bjts`: the gm/gp stamps
            // above act on FULL node voltages, so the constant injection at
            // each terminal must be I_lin(v0) − I_dc(terminal) (current INTO
            // the node), making v = v0 the exact DC fixed point. The grid has
            // no stamped conductance (no grid-conduction model in the
            // linearized triode), so its linear-model current is zero and the
            // injection stays −ig_dc.
            let vgk0 = tube.vgk0;
            let vpk0 = tube.vpk0;
            let mut i_lin_p = 0.0;
            let mut i_lin_k = 0.0;
            if tube.gm.abs() > 1e-30 {
                if np > 0 {
                    i_lin_p += tube.gm * vgk0;
                }
                if nk > 0 {
                    i_lin_k -= tube.gm * vgk0;
                }
            }
            if tube.gp.abs() > 1e-30 {
                if np > 0 {
                    i_lin_p += tube.gp * vpk0;
                }
                if nk > 0 {
                    i_lin_k -= tube.gp * vpk0;
                }
            }

            // Terminal DC currents drawn OUT of each node by the real device:
            // plate ip_dc, grid ig_dc, cathode -(ip_dc + ig_dc).
            if np > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ip_dc", tube.name),
                    n_plus_idx: np,
                    n_minus_idx: 0,
                    dc_value: i_lin_p - tube.ip_dc,
                });
            }
            // Grid injection (only non-negligible when tube is near grid conduction)
            if ng > 0 && tube.ig_dc.abs() > 1e-15 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ig_dc", tube.name),
                    n_plus_idx: 0,
                    n_minus_idx: ng,
                    dc_value: tube.ig_dc,
                });
            }
            if nk > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ik_dc", tube.name),
                    n_plus_idx: nk,
                    n_minus_idx: 0,
                    dc_value: i_lin_k + tube.ip_dc + tube.ig_dc,
                });
            }

            // Inter-electrode capacitances: the rebuild's junction-cap pass
            // covers only the devices left in the nonlinear system, so a
            // linearized triode's are stamped here (same orientation as
            // `stamp_device_junction_caps`).
            self.stamp_capacitor_raw(nk, ng, tube.ccg);
            self.stamp_capacitor_raw(ng, np, tube.cgp);
            self.stamp_capacitor_raw(nk, np, tube.ccp);
        }
    }
}
