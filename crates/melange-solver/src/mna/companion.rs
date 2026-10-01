//! Trapezoidal A / A_neg assembly with the deprecated (0.1.14) inductor companion stamps.

use super::*;

impl MnaSystem {
    /// Build a discretized system matrix from G and C with inductor companion models.
    ///
    /// Computes `result[i][j] = g_sign * G[i][j] + alpha * C[i][j]` for each element,
    /// then stamps inductor companion conductances with the given `g_sign`.
    ///
    /// - `g_sign = +1`: produces A = G + (2/T)*C  (forward matrix)
    /// - `g_sign = -1`: produces A_neg = (2/T)*C - G  (history matrix)
    ///
    /// For augmented MNA (voltage sources/VCVS present), rows n..n_aug-1 are algebraic
    /// constraints with no capacitance. In A_neg (g_sign < 0), those rows must be ALL
    /// ZEROS because there is no trapezoidal history for algebraic constraints.
    ///
    /// **Deprecated (0.1.14):** the inductor, coupled-inductor and transformer
    /// companion stamps below serve only the deprecated
    /// [`crate::LinearSolver`], a second, library-only linear solver whose
    /// whole-system trapezoidal discretisation differs from the charge form
    /// every generated solver ships (generated code carries inductors as
    /// augmented branch rows, [`crate::dk::DkKernel::from_mna_augmented`]). No
    /// build uses them; they will be removed in the next release. Use
    /// `melange_solver::build::build` and code generation. The G/C part of this
    /// function is live.
    #[allow(clippy::needless_range_loop)]
    fn build_discretized_matrix(
        &self,
        sample_rate: f64,
        g_sign: f64,
    ) -> Result<Vec<Vec<f64>>, MnaError> {
        if !(sample_rate > 0.0 && sample_rate.is_finite()) {
            return Err(MnaError::InvalidParameter(format!(
                "invalid sample_rate: {} (must be positive and finite)",
                sample_rate
            )));
        }
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t; // 2/T for trapezoidal
        let n_aug = self.n_aug;

        let mut mat = vec![vec![0.0; n_aug]; n_aug];
        for i in 0..n_aug {
            for j in 0..n_aug {
                mat[i][j] = g_sign * self.g[i][j] + alpha * self.c[i][j];
            }
        }

        // Zero A_neg rows for algebraic constraints. These rows have no
        // capacitance — no trapezoidal history term — and carrying the -G
        // part forward as "history" couples the constraint to the previous
        // sample's residual (a marginally-stable ±1 mode; the classic
        // Nyquist-rate artifact on algebraic rows).
        //
        // Blanket-zero ALL augmented rows n..n_aug rather than enumerating
        // per element type: the old per-type enumeration (VS, VCVS,
        // ideal-xfmr) missed current-mode VCA rows (internal sig+_int node +
        // sensing-source branch) and behavioral `V={}` branch rows. This
        // matches the generated `rebuild_matrices` (dk_emitter) and the
        // nodal IR builder, which blanket-zero the same range.
        //
        // Exclusions/non-issues, verified against the index layout:
        // - BJT transient internal nodes (`expand_bjt_internal_nodes`) are
        //   appended AFTER the algebraic rows, inside n..n_aug. They are
        //   physical nodes carrying G (RB/RC/RE) and C (junction caps) and
        //   NEED their trapezoidal history, so they are excluded here. (In
        //   practice the CLI only expands them after routing to the nodal
        //   path, where this function is no longer consulted — the exclusion
        //   is defensive.)
        // - Inductor branch variables NEVER appear in n..n_aug on this path:
        //   `build_discretized_matrix` handles all inductor types via
        //   companion-model conductance stamps into node rows (below).
        //   Branch-current variables only exist in the separate
        //   `build_augmented_matrices` output, at rows n_aug..n_nodal of a
        //   matrix that never reaches this function.
        if g_sign < 0.0 && n_aug > self.n {
            let mut is_bjt_internal = vec![false; n_aug];
            for bn in &self.bjt_internal_nodes {
                for idx in [bn.int_base, bn.int_collector, bn.int_emitter]
                    .into_iter()
                    .flatten()
                {
                    if idx < n_aug {
                        is_bjt_internal[idx] = true;
                    }
                }
            }
            for row in self.n..n_aug {
                if is_bjt_internal[row] {
                    continue;
                }
                for j in 0..n_aug {
                    mat[row][j] = 0.0;
                }
            }
        }

        // Deprecated (0.1.14), with everything down to the end of this
        // function: companion-model inductor stamps for the deprecated
        // `LinearSolver` only; removed in the next release (see the doc
        // comment). The transformer group's builder-side "positive-
        // definiteness" check (`build`, multi-winding K groups) is not a PD
        // test: it checks that the minimum diagonal of L^-1 is > 0, which a
        // non-PD matrix can pass.
        //
        // Inductor companion model conductances: g_eq = T/(2L).
        // In A (g_sign=+1) inductors add +g_eq (like a resistor).
        // In A_neg (g_sign=-1) inductors add -g_eq (opposite sign).
        let g_eq_factor = t / 2.0;
        for ind in &self.inductors {
            let g = g_sign * g_eq_factor / ind.value;
            stamp_conductance_to_ground(&mut mat, ind.node_i, ind.node_j, g);
        }

        // Coupled inductor companion model: self + mutual conductances.
        // For two coupled inductors L1, L2 with coupling k:
        //   M = k * sqrt(L1 * L2)
        //   det = L1*L2 - M^2
        //   g_self_1 = (T/2) * L2 / det,  g_self_2 = (T/2) * L1 / det
        //   g_mutual = -(T/2) * M / det
        for ci in &self.coupled_inductors {
            let m = ci.coupling * (ci.l1_value * ci.l2_value).sqrt();
            let det = ci.l1_value * ci.l2_value - m * m;
            // Guard the companion-conductance division: for perfect coupling
            // (k → 1) det = L1*L2 - M² → 0, giving NaN/inf conductances. The DK
            // kernel path already rejects this (dk.rs), but this matrix is built
            // via get_a_matrix BEFORE that check, so guard it here too. A
            // perfectly-coupled pair is degenerate in the companion model and
            // must use the ideal-transformer decomposition instead.
            if det <= 0.0 {
                return Err(MnaError::TopologyError(format!(
                    "Coupled inductors '{}'-'{}': det = L1*L2 - M² = {:.6e} <= 0 \
                     (coupling k={} too close to 1). Use the ideal-transformer \
                     decomposition for perfectly-coupled windings.",
                    ci.l1_name, ci.l2_name, det, ci.coupling
                )));
            }
            let gs1 = g_sign * g_eq_factor * ci.l2_value / det;
            let gs2 = g_sign * g_eq_factor * ci.l1_value / det;
            let gm = g_sign * (-g_eq_factor) * m / det;

            // Self-conductances (stamped like regular inductors)
            stamp_conductance_to_ground(&mut mat, ci.l1_node_i, ci.l1_node_j, gs1);
            stamp_conductance_to_ground(&mut mat, ci.l2_node_i, ci.l2_node_j, gs2);

            // Mutual conductance cross-coupling between L1 and L2 (symmetric)
            stamp_mutual_conductance(
                &mut mat,
                ci.l1_node_i,
                ci.l1_node_j,
                ci.l2_node_i,
                ci.l2_node_j,
                gm,
            );
            stamp_mutual_conductance(
                &mut mat,
                ci.l2_node_i,
                ci.l2_node_j,
                ci.l1_node_i,
                ci.l1_node_j,
                gm,
            );
        }

        // Multi-winding transformer groups: NxN admittance stamping.
        // Build the full inductance matrix, invert it, multiply by T/2,
        // and stamp all self and mutual admittance entries.
        for group in &self.transformer_groups {
            let w = group.num_windings;
            // Build inductance matrix L[i][j] = k[i][j] * sqrt(L_i * L_j)
            let mut l_mat = vec![vec![0.0f64; w]; w];
            for i in 0..w {
                for j in 0..w {
                    l_mat[i][j] = group.coupling_matrix[i][j]
                        * (group.inductances[i] * group.inductances[j]).sqrt();
                }
            }
            // Invert: Y_raw = inv(L). A singular or non-finite L has no
            // companion admittance; refuse rather than stamp a stand-in.
            let y_raw = invert_small_matrix(&l_mat).map_err(|reason| {
                MnaError::TopologyError(format!(
                    "Transformer group '{}' ({} windings): cannot invert the \
                     inductance matrix: {}. Check the winding inductances and \
                     that the K coefficients are not all 1.",
                    group.name, w, reason
                ))
            })?;
            // Scale by T/2 and apply sign
            let scale = g_sign * g_eq_factor;
            // Stamp admittance entries
            for i in 0..w {
                // Self-conductance Y[i][i]
                let y_self = scale * y_raw[i][i];
                stamp_conductance_to_ground(
                    &mut mat,
                    group.winding_node_i[i],
                    group.winding_node_j[i],
                    y_self,
                );
                // Mutual conductance Y[i][j] for j > i (stamp both directions)
                for j in (i + 1)..w {
                    let y_mut = scale * y_raw[i][j];
                    stamp_mutual_conductance(
                        &mut mat,
                        group.winding_node_i[i],
                        group.winding_node_j[i],
                        group.winding_node_i[j],
                        group.winding_node_j[j],
                        y_mut,
                    );
                    stamp_mutual_conductance(
                        &mut mat,
                        group.winding_node_i[j],
                        group.winding_node_j[j],
                        group.winding_node_i[i],
                        group.winding_node_j[i],
                        y_mut,
                    );
                }
            }
        }

        Ok(mat)
    }

    /// Get the A matrix for a given sample rate (trapezoidal discretization).
    ///
    /// A = G + (2/T)*C (includes inductor companion model conductances)
    ///
    /// **Deprecated (0.1.14):** the inductor companion conductances serve only
    /// the deprecated [`crate::LinearSolver`]; see `build_discretized_matrix`.
    ///
    /// Returns `Err(MnaError::InvalidParameter)` if `sample_rate` is not positive and finite.
    pub fn get_a_matrix(&self, sample_rate: f64) -> Result<Vec<Vec<f64>>, MnaError> {
        self.build_discretized_matrix(sample_rate, 1.0)
    }

    /// Get the A_neg matrix for history term (trapezoidal discretization).
    ///
    /// A_neg = (2/T)*C - G (includes inductor companion model)
    ///
    /// **Deprecated (0.1.14):** the inductor companion conductances serve only
    /// the deprecated [`crate::LinearSolver`]; see `build_discretized_matrix`.
    ///
    /// Returns `Err(MnaError::InvalidParameter)` if `sample_rate` is not positive and finite.
    pub fn get_a_neg_matrix(&self, sample_rate: f64) -> Result<Vec<Vec<f64>>, MnaError> {
        self.build_discretized_matrix(sample_rate, -1.0)
    }
}

/// Stamp mutual conductance between two 2-terminal elements.
///
/// For a mutual conductance `g` between element 1 (nodes a, b) and
/// element 2 (nodes c, d), the stamp adds cross-coupling:
///   mat[a][c] += g, mat[b][d] += g, mat[a][d] -= g, mat[b][c] -= g
///
/// Node indices use MNA convention: 0 = ground (excluded from matrix).
fn stamp_mutual_conductance(mat: &mut [Vec<f64>], a: usize, b: usize, c: usize, d: usize, g: f64) {
    // a-c coupling
    if a > 0 && c > 0 {
        mat[a - 1][c - 1] += g;
    }
    // b-d coupling
    if b > 0 && d > 0 {
        mat[b - 1][d - 1] += g;
    }
    // a-d coupling (negative)
    if a > 0 && d > 0 {
        mat[a - 1][d - 1] -= g;
    }
    // b-c coupling (negative)
    if b > 0 && c > 0 {
        mat[b - 1][c - 1] -= g;
    }
}
