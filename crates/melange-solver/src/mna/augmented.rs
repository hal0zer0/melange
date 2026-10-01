//! Augmented-MNA growth: `IC=` voltage-source rows and inductor branch variables.

use super::*;

/// Result of building augmented G/C matrices with inductor branch variables.
///
/// Each inductor winding adds an extra variable (branch current) in the system.
/// The inductance L appears in the C matrix diagonal. The total dimension is
/// `n_nodal = n_aug + n_inductor_vars`.
#[derive(Debug, Clone)]
pub struct AugmentedMatrices {
    /// Augmented G matrix (n_nodal x n_nodal)
    pub g: Vec<Vec<f64>>,
    /// Augmented C matrix (n_nodal x n_nodal)
    pub c: Vec<Vec<f64>>,
    /// Total nodal dimension = n_aug + n_inductor_vars
    pub n_nodal: usize,
    /// Number of inductor branch variables added
    pub n_inductor_vars: usize,
}
impl MnaSystem {
    /// Append one augmented voltage-source row/column per `IC=`-bearing
    /// capacitor, enforcing `v(node_i) - v(node_j) = ic` as an algebraic
    /// constraint (same augmented-MNA mechanism as a real voltage source —
    /// see `docs/aidocs/MNA.md` "Voltage Source (Augmented MNA)").
    ///
    /// Intended for use on a throwaway clone of the real `MnaSystem` used
    /// only to build the initial-state solve that seeds `v_prev`: it grows
    /// `n_aug`/`g`/`n_v`/`n_i` and appends to `voltage_sources`, which would
    /// be wrong to do on the `MnaSystem` that drives the actual transient
    /// kernel (that one must keep the IC capacitor as an ordinary capacitor
    /// in `C`, not as a permanent extra voltage-source unknown). No-op when
    /// `capacitor_ics` is empty.
    pub fn append_ic_voltage_sources(&mut self) {
        if self.capacitor_ics.is_empty() {
            return;
        }
        let n = self.n;
        let old_n_aug = self.n_aug;
        let num_new = self.capacitor_ics.len();
        let new_n_aug = old_n_aug + num_new;

        // Grow G to new_n_aug x new_n_aug.
        for row in self.g.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }
        for _ in 0..num_new {
            self.g.push(vec![0.0; new_n_aug]);
        }
        // Grow N_v (M x n_aug) columns and N_i (n_aug x M) rows to match.
        for row in self.n_v.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }
        for _ in 0..num_new {
            self.n_i.push(vec![0.0; self.m]);
        }

        for (i, cap_ic) in self.capacitor_ics.clone().iter().enumerate() {
            let k = old_n_aug + i;
            let np = cap_ic.node_i;
            let nm = cap_ic.node_j;
            // Current injection (column k):
            if np > 0 {
                self.g[np - 1][k] += 1.0;
            }
            if nm > 0 {
                self.g[nm - 1][k] -= 1.0;
            }
            // KVL constraint (row k): V(np) - V(nm) = ic
            if np > 0 {
                self.g[k][np - 1] += 1.0;
            }
            if nm > 0 {
                self.g[k][nm - 1] -= 1.0;
            }
            self.voltage_sources.push(VoltageSourceInfo {
                name: format!("__ic_{}", cap_ic.name),
                n_plus: String::new(),
                n_minus: String::new(),
                n_plus_idx: np,
                n_minus_idx: nm,
                dc_value: cap_ic.ic,
                ext_idx: k - n,
            });
        }
        self.n_aug = new_n_aug;
    }

    /// Build augmented G and C matrices with inductor branch variables.
    ///
    /// Copies `self.g` / `self.c` into `n_nodal x n_nodal` matrices, then stamps
    /// KCL/KVL/inductance for all inductor types (uncoupled, coupled pairs, and
    /// transformer groups).
    ///
    /// **Does NOT add Gmin** — that is caller-specific (e.g. NodalSolver adds it,
    /// DK kernel does not).
    ///
    /// Returns `AugmentedMatrices` with the expanded G, C, total dimension, and
    /// the number of inductor variables added.
    pub fn build_augmented_matrices(&self) -> AugmentedMatrices {
        let n_aug = self.n_aug;

        // Count inductor winding variables
        let n_uncoupled = self.inductors.len();
        let n_coupled_windings: usize = self.coupled_inductors.len() * 2;
        let n_xfmr_windings: usize = self.transformer_groups.iter().map(|g| g.num_windings).sum();
        let n_inductor_vars = n_uncoupled + n_coupled_windings + n_xfmr_windings;
        let n_nodal = n_aug + n_inductor_vars;

        // Copy base MNA matrices into expanded dimension
        let mut g_nod = vec![vec![0.0; n_nodal]; n_nodal];
        let mut c_nod = vec![vec![0.0; n_nodal]; n_nodal];
        for i in 0..n_aug {
            for j in 0..n_aug {
                g_nod[i][j] = self.g[i][j];
                c_nod[i][j] = self.c[i][j];
            }
        }

        // Stamp inductor augmented variables
        let mut var_idx = n_aug;

        // Uncoupled inductors: 1 variable each
        for ind in &self.inductors {
            let k = var_idx;
            let ni = ind.node_i; // 1-indexed, 0 = ground
            let nj = ind.node_j;

            // KCL: j_L enters node_i, exits node_j
            if ni > 0 {
                g_nod[ni - 1][k] += 1.0;
            }
            if nj > 0 {
                g_nod[nj - 1][k] -= 1.0;
            }
            // KVL row: -V_i + V_j (= -L * dj_L/dt, with L in C)
            if ni > 0 {
                g_nod[k][ni - 1] -= 1.0;
            }
            if nj > 0 {
                g_nod[k][nj - 1] += 1.0;
            }
            // Self-inductance in C matrix
            c_nod[k][k] = ind.value;

            var_idx += 1;
        }

        // Coupled inductor pairs: 2 variables each
        for ci in &self.coupled_inductors {
            let k1 = var_idx;
            let k2 = var_idx + 1;

            // Winding 1 KCL/KVL
            if ci.l1_node_i > 0 {
                g_nod[ci.l1_node_i - 1][k1] += 1.0;
            }
            if ci.l1_node_j > 0 {
                g_nod[ci.l1_node_j - 1][k1] -= 1.0;
            }
            if ci.l1_node_i > 0 {
                g_nod[k1][ci.l1_node_i - 1] -= 1.0;
            }
            if ci.l1_node_j > 0 {
                g_nod[k1][ci.l1_node_j - 1] += 1.0;
            }

            // Winding 2 KCL/KVL
            if ci.l2_node_i > 0 {
                g_nod[ci.l2_node_i - 1][k2] += 1.0;
            }
            if ci.l2_node_j > 0 {
                g_nod[ci.l2_node_j - 1][k2] -= 1.0;
            }
            if ci.l2_node_i > 0 {
                g_nod[k2][ci.l2_node_i - 1] -= 1.0;
            }
            if ci.l2_node_j > 0 {
                g_nod[k2][ci.l2_node_j - 1] += 1.0;
            }

            // Self-inductances
            c_nod[k1][k1] = ci.l1_value;
            c_nod[k2][k2] = ci.l2_value;
            // Mutual inductance M = k * sqrt(L1 * L2)
            let m_val = ci.coupling * (ci.l1_value * ci.l2_value).sqrt();
            c_nod[k1][k2] = m_val;
            c_nod[k2][k1] = m_val;

            var_idx += 2;
        }

        // Transformer groups: N variables each
        for group in &self.transformer_groups {
            let w = group.num_windings;
            let base_k = var_idx;

            for widx in 0..w {
                let k = base_k + widx;
                let ni = group.winding_node_i[widx];
                let nj = group.winding_node_j[widx];

                // KCL/KVL stamps for winding
                if ni > 0 {
                    g_nod[ni - 1][k] += 1.0;
                }
                if nj > 0 {
                    g_nod[nj - 1][k] -= 1.0;
                }
                if ni > 0 {
                    g_nod[k][ni - 1] -= 1.0;
                }
                if nj > 0 {
                    g_nod[k][nj - 1] += 1.0;
                }

                // Inductance sub-matrix: L[i][j] = k_ij * sqrt(Li * Lj)
                for widx2 in 0..w {
                    let k2 = base_k + widx2;
                    c_nod[k][k2] = group.coupling_matrix[widx][widx2]
                        * (group.inductances[widx] * group.inductances[widx2]).sqrt();
                }
            }

            var_idx += w;
        }

        AugmentedMatrices {
            g: g_nod,
            c: c_nod,
            n_nodal,
            n_inductor_vars,
        }
    }
}
