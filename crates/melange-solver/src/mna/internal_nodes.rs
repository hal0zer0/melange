//! Expansion of BJT parasitic-resistance internal nodes into the transient MNA.

use super::*;

impl MnaSystem {
    /// How many BJTs in this system have internal nodes to expand.
    ///
    /// Mirrors the collection loop in [`Self::expand_bjt_internal_nodes`]
    /// exactly: a count of 0 means that function is a guaranteed no-op.
    ///
    /// [`crate::pipeline::expand_internal_nodes`] reads it to skip the device
    /// walk when there is nothing to expand.
    pub fn expandable_bjt_internal_node_count(
        &self,
        device_slots: &[crate::device_types::DeviceSlot],
    ) -> usize {
        use crate::device_types::{DeviceParams, DeviceType};
        self.nonlinear_devices
            .iter()
            .zip(device_slots.iter())
            .filter(|(dev_info, slot)| {
                slot.device_type == DeviceType::Bjt
                    && matches!(&slot.params, DeviceParams::Bjt(bp)
                        if bp.has_parasitics() && dev_info.node_indices.len() >= 3)
            })
            .count()
    }

    /// Expand MNA with internal nodes for parasitic BJTs (RB/RC/RE).
    ///
    /// For each non-forward-active BJT with non-zero parasitic resistances, creates
    /// internal nodes (basePrime, collectorPrime, emitterPrime) in the MNA system.
    /// Stamps parasitic conductances (1/R) in G between external and internal nodes,
    /// and redirects N_v/N_i to reference internal nodes for the intrinsic model.
    ///
    /// This eliminates the inner 2D NR loop per parasitic BJT in generated code —
    /// the DK kernel absorbs the parasitic R into S = A⁻¹ automatically.
    ///
    /// Must be called AFTER the MNA is fully built (all matrices at n_aug dimension)
    /// but BEFORE DK kernel construction.
    pub fn expand_bjt_internal_nodes(&mut self, device_slots: &[crate::device_types::DeviceSlot]) {
        use crate::device_types::{DeviceParams, DeviceType};

        // Collect expansion info first
        struct Expansion {
            start_idx: usize,
            device_name: String,
            ext_c: usize, // 1-indexed external nodes (0=ground)
            ext_b: usize,
            ext_e: usize,
            rb: f64,
            rc: f64,
            re: f64,
        }

        let mut expansions = Vec::new();
        for (dev_info, slot) in self.nonlinear_devices.iter().zip(device_slots.iter()) {
            if slot.device_type == DeviceType::Bjt {
                if let DeviceParams::Bjt(bp) = &slot.params {
                    if bp.has_parasitics() && dev_info.node_indices.len() >= 3 {
                        expansions.push(Expansion {
                            start_idx: slot.start_idx,
                            device_name: dev_info.name.clone(),
                            ext_c: dev_info.node_indices[0],
                            ext_b: dev_info.node_indices[1],
                            ext_e: dev_info.node_indices[2],
                            rb: bp.rb,
                            rc: bp.rc,
                            re: bp.re,
                        });
                    }
                }
            }
        }

        if expansions.is_empty() {
            return;
        }

        // Count new internal nodes needed
        let mut num_new = 0usize;
        for exp in &expansions {
            if exp.rb > 0.0 {
                num_new += 1;
            }
            if exp.rc > 0.0 {
                num_new += 1;
            }
            if exp.re > 0.0 {
                num_new += 1;
            }
        }

        let old_n_aug = self.n_aug;
        let new_n_aug = old_n_aug + num_new;

        // Expand G, C matrices: extend existing rows, add new rows
        for row in self.g.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }
        for row in self.c.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }
        for _ in old_n_aug..new_n_aug {
            self.g.push(vec![0.0; new_n_aug]);
            self.c.push(vec![0.0; new_n_aug]);
        }

        // Expand N_v (m × n_aug): extend each row
        for row in self.n_v.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }

        // Expand N_i (n_aug × m): add new zero rows
        for _ in old_n_aug..new_n_aug {
            self.n_i.push(vec![0.0; self.m]);
        }

        // Allocate internal nodes and stamp
        let mut next_idx = old_n_aug; // 0-indexed internal node index

        for exp in &expansions {
            let s = exp.start_idx;

            // Create internal node indices (0-indexed into the matrix)
            let int_b = if exp.rb > 0.0 {
                let idx = next_idx;
                next_idx += 1;
                Some(idx)
            } else {
                None
            };
            let int_c = if exp.rc > 0.0 {
                let idx = next_idx;
                next_idx += 1;
                Some(idx)
            } else {
                None
            };
            let int_e = if exp.re > 0.0 {
                let idx = next_idx;
                next_idx += 1;
                Some(idx)
            } else {
                None
            };

            // External node indices (0-indexed; None if grounded)
            let ext_b_0 = if exp.ext_b > 0 {
                Some(exp.ext_b - 1)
            } else {
                None
            };
            let ext_c_0 = if exp.ext_c > 0 {
                Some(exp.ext_c - 1)
            } else {
                None
            };
            let ext_e_0 = if exp.ext_e > 0 {
                Some(exp.ext_e - 1)
            } else {
                None
            };

            // Stamp parasitic conductances in G: 1/R between external and internal
            if let Some(ib) = int_b {
                let g = 1.0 / exp.rb;
                self.g[ib][ib] += g;
                if let Some(eb) = ext_b_0 {
                    self.g[eb][eb] += g;
                    self.g[eb][ib] -= g;
                    self.g[ib][eb] -= g;
                }
            }
            if let Some(ic) = int_c {
                let g = 1.0 / exp.rc;
                self.g[ic][ic] += g;
                if let Some(ec) = ext_c_0 {
                    self.g[ec][ec] += g;
                    self.g[ec][ic] -= g;
                    self.g[ic][ec] -= g;
                }
            }
            if let Some(ie) = int_e {
                let g = 1.0 / exp.re;
                self.g[ie][ie] += g;
                if let Some(ee) = ext_e_0 {
                    self.g[ee][ee] += g;
                    self.g[ee][ie] -= g;
                    self.g[ie][ee] -= g;
                }
            }

            // Effective nodes for N_v/N_i (internal if parasitic R, else external)
            let eff_b = int_b.or(ext_b_0);
            let eff_c = int_c.or(ext_c_0);
            let eff_e = int_e.or(ext_e_0);

            // Clear existing N_v/N_i entries for this BJT
            for j in 0..new_n_aug {
                self.n_v[s][j] = 0.0;
                self.n_v[s + 1][j] = 0.0;
            }
            for i in 0..new_n_aug {
                self.n_i[i][s] = 0.0;
                self.n_i[i][s + 1] = 0.0;
            }

            // Re-stamp with accumulation (`+=`) into the freshly-zeroed
            // rows/cols above: if two effective terminals resolve to the SAME
            // node (tied external terminals with no parasitic R on either),
            // the entries must cancel to 0, not overwrite.
            // Re-stamp N_v: Vbe_int = V(basePrime) - V(emitterPrime)
            if let Some(b) = eff_b {
                self.n_v[s][b] += 1.0;
            }
            if let Some(e) = eff_e {
                self.n_v[s][e] += -1.0;
            }
            // N_v: Vbc_int = V(basePrime) - V(collectorPrime)
            if let Some(b) = eff_b {
                self.n_v[s + 1][b] += 1.0;
            }
            if let Some(c) = eff_c {
                self.n_v[s + 1][c] += -1.0;
            }

            // N_i: Ic enters collectorPrime, exits emitterPrime
            if let Some(c) = eff_c {
                self.n_i[c][s] += -1.0;
            }
            if let Some(e) = eff_e {
                self.n_i[e][s] += 1.0;
            }
            // N_i: Ib enters basePrime, exits emitterPrime
            if let Some(b) = eff_b {
                self.n_i[b][s + 1] += -1.0;
            }
            if let Some(e) = eff_e {
                self.n_i[e][s + 1] += 1.0;
            }

            self.bjt_internal_nodes.push(BjtTransientInternalNodes {
                device_name: exp.device_name.clone(),
                start_idx: s,
                int_base: int_b,
                int_collector: int_c,
                int_emitter: int_e,
            });

            log::debug!(
                "BJT {} internal nodes: base={:?} collector={:?} emitter={:?} (RB={} RC={} RE={})",
                exp.device_name,
                int_b,
                int_c,
                int_e,
                exp.rb,
                exp.rc,
                exp.re,
            );
        }

        // Update n_aug only — do NOT increment self.n.
        // self.n stays at the original circuit node count. The internal nodes are
        // appended after n_aug, like inductor branch variables. The A_neg zeroing
        // targets rows self.n..n_aug (VS/VCVS algebraic constraints), which must
        // NOT include the internal nodes (they need trapezoidal history terms).
        self.n_aug = new_n_aug;
    }
}
