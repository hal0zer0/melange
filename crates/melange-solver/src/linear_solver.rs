//! Linear circuit solver.
//!
//! Fast solver for circuits with no nonlinear devices (M=0).
//! Uses the DK method with pre-computed S=A⁻¹ for O(N²) per sample.
//!
//! All processing uses pre-allocated buffers — no heap allocation in the audio thread.

use crate::dk::DkKernel;

/// Error type for solver construction and validation.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum SolverError {
    /// Invalid circuit topology
    InvalidTopology(String),
    /// Device model error
    DeviceModel(String),
    /// DK kernel error (forwarded)
    Dk(crate::dk::DkError),
}

impl std::fmt::Display for SolverError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SolverError::InvalidTopology(s) => write!(f, "Invalid topology: {}", s),
            SolverError::DeviceModel(s) => write!(f, "Device model error: {}", s),
            SolverError::Dk(e) => write!(f, "DK error: {}", e),
        }
    }
}

impl std::error::Error for SolverError {}

impl From<crate::dk::DkError> for SolverError {
    fn from(e: crate::dk::DkError) -> Self {
        SolverError::Dk(e)
    }
}

/// Sanitize an audio input sample: replace NaN/Inf with 0, clamp to [-100, 100].
fn sanitize_input(input: f64) -> f64 {
    if input.is_finite() {
        input.clamp(-100.0, 100.0)
    } else {
        0.0
    }
}

/// Clamp a raw output voltage to safe audio range [-10, 10], replacing NaN/Inf with 0.
fn clamp_output(raw: f64) -> f64 {
    if raw.is_finite() {
        raw.clamp(-10.0, 10.0)
    } else {
        0.0
    }
}

/// Reset all inductor companion model state to zero.
fn reset_inductors(inductors: &mut [crate::dk::InductorInfo]) {
    for ind in inductors {
        ind.i_hist = 0.0;
        ind.i_prev = 0.0;
        ind.v_prev = 0.0;
    }
}

/// Reset all coupled inductor companion model state to zero.
fn reset_coupled_inductors(coupled: &mut [crate::dk::CoupledInductorState]) {
    for ci in coupled {
        ci.i1_hist = 0.0;
        ci.i2_hist = 0.0;
        ci.i1_prev = 0.0;
        ci.i2_prev = 0.0;
        ci.v1_prev = 0.0;
        ci.v2_prev = 0.0;
    }
}

/// Reset all transformer group companion model state to zero.
fn reset_transformer_groups(groups: &mut [crate::dk::TransformerGroupState]) {
    for g in groups {
        for v in &mut g.i_hist {
            *v = 0.0;
        }
        for v in &mut g.i_prev {
            *v = 0.0;
        }
        for v in &mut g.v_prev {
            *v = 0.0;
        }
    }
}

/// Linear circuit solver using the DK method.
///
/// For circuits with M=0 (no nonlinear devices), the DK method reduces to
/// a simple matrix-vector multiply: `v[n+1] = S * (rhs_const + A_neg * v[n])`.
/// No Newton-Raphson iteration needed.
pub struct LinearSolver {
    kernel: DkKernel,
    v_prev: Vec<f64>,
    rhs: Vec<f64>,
    v_pred: Vec<f64>,
    input_node: usize,
    output_node: usize,
    /// Input conductance (1/R_in) for proper Thevenin source modeling
    pub(crate) input_conductance: f64,
    /// Previous input sample for trapezoidal integration
    input_prev: f64,
    /// DC blocking filter state (previous input)
    pub(crate) dc_block_x_prev: f64,
    /// DC blocking filter state (previous output)
    pub(crate) dc_block_y_prev: f64,
    /// DC blocking filter coefficient (1 - 2*pi*5/sr)
    pub(crate) dc_block_r: f64,
    /// Diagnostics: peak absolute output seen
    pub(crate) diag_peak_output: f64,
    /// Diagnostics: number of samples that exceeded clamp threshold
    pub(crate) diag_clamp_count: u64,
    /// Diagnostics: number of times state was reset due to NaN
    pub(crate) diag_nan_reset_count: u64,
}

impl LinearSolver {
    /// Create a new linear solver.
    pub fn new(kernel: DkKernel, input_node: usize, output_node: usize) -> Self {
        let n = kernel.n;
        let dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / kernel.sample_rate;
        Self {
            kernel,
            v_prev: vec![0.0; n],
            rhs: vec![0.0; n],
            v_pred: vec![0.0; n],
            input_node,
            output_node,
            input_conductance: 1.0,
            input_prev: 0.0,
            dc_block_x_prev: 0.0,
            dc_block_y_prev: 0.0,
            dc_block_r,
            diag_peak_output: 0.0,
            diag_clamp_count: 0,
            diag_nan_reset_count: 0,
        }
    }

    /// Get the input conductance (1/R_in).
    pub fn input_conductance(&self) -> f64 {
        self.input_conductance
    }

    /// Set the input conductance (1/R_in) for proper Thevenin source modeling.
    pub fn set_input_conductance(&mut self, g: f64) {
        self.input_conductance = g;
    }

    /// Diagnostics: peak absolute output seen.
    pub fn diag_peak_output(&self) -> f64 {
        self.diag_peak_output
    }

    /// Diagnostics: number of samples that exceeded clamp threshold.
    pub fn diag_clamp_count(&self) -> u64 {
        self.diag_clamp_count
    }

    /// Diagnostics: number of times state was reset due to NaN.
    pub fn diag_nan_reset_count(&self) -> u64 {
        self.diag_nan_reset_count
    }

    /// Process a single sample.
    #[allow(clippy::needless_range_loop)]
    pub fn process_sample(&mut self, input: f64) -> f64 {
        let input = sanitize_input(input);
        let n = self.kernel.n;

        // Build RHS: rhs_const + A_neg * v_prev
        for i in 0..n {
            let mut sum = self.kernel.rhs_const[i];
            let a_neg_row_offset = i * n;
            for j in 0..n {
                sum += self.kernel.a_neg[a_neg_row_offset + j] * self.v_prev[j];
            }
            self.rhs[i] = sum;
        }

        // Add inductor history contribution
        for ind in &self.kernel.inductors {
            let i_hist = ind.i_hist;
            if ind.node_i > 0 {
                self.rhs[ind.node_i - 1] -= i_hist;
            }
            if ind.node_j > 0 {
                self.rhs[ind.node_j - 1] += i_hist;
            }
        }

        // Add coupled inductor history contribution
        for ci in &self.kernel.coupled_inductors {
            if ci.l1_node_i > 0 {
                self.rhs[ci.l1_node_i - 1] -= ci.i1_hist;
            }
            if ci.l1_node_j > 0 {
                self.rhs[ci.l1_node_j - 1] += ci.i1_hist;
            }
            if ci.l2_node_i > 0 {
                self.rhs[ci.l2_node_i - 1] -= ci.i2_hist;
            }
            if ci.l2_node_j > 0 {
                self.rhs[ci.l2_node_j - 1] += ci.i2_hist;
            }
        }

        // Add transformer group history contribution
        for group in &self.kernel.transformer_groups {
            for k in 0..group.num_windings {
                let i_hist = group.i_hist[k];
                if group.winding_node_i[k] > 0 {
                    self.rhs[group.winding_node_i[k] - 1] -= i_hist;
                }
                if group.winding_node_j[k] > 0 {
                    self.rhs[group.winding_node_j[k] - 1] += i_hist;
                }
            }
        }

        // Add input source (Thevenin: V_in through R_in, trapezoidal rule)
        if self.input_node < n {
            self.rhs[self.input_node] += (input + self.input_prev) * self.input_conductance;
        }
        self.input_prev = input;

        // Predict: v_pred = S * rhs
        for i in 0..n {
            let mut sum = 0.0;
            let s_row_offset = i * n;
            for j in 0..n {
                sum += self.kernel.s[s_row_offset + j] * self.rhs[j];
            }
            self.v_pred[i] = sum;
        }

        std::mem::swap(&mut self.v_prev, &mut self.v_pred);

        // Sanitize state BEFORE updating inductors to prevent NaN poisoning
        if self.v_prev.iter().any(|v| !v.is_finite()) {
            self.v_prev.fill(0.0);
            self.input_prev = 0.0;
            reset_inductors(&mut self.kernel.inductors);
            reset_coupled_inductors(&mut self.kernel.coupled_inductors);
            reset_transformer_groups(&mut self.kernel.transformer_groups);
            self.diag_nan_reset_count += 1;
        }

        self.kernel.update_inductors(&self.v_prev);
        self.kernel.update_coupled_inductors(&self.v_prev);
        self.kernel.update_transformer_groups(&self.v_prev);

        // DC blocking filter (5Hz HPF)
        let raw_out = if self.output_node < n {
            self.v_prev[self.output_node]
        } else {
            0.0
        };
        let raw_out = if raw_out.is_finite() { raw_out } else { 0.0 };
        let dc_blocked =
            raw_out - self.dc_block_x_prev + self.dc_block_r * self.dc_block_y_prev;
        self.dc_block_x_prev = raw_out;
        self.dc_block_y_prev = dc_blocked.clamp(-100.0, 100.0);

        // Diagnostics
        let abs_out = dc_blocked.abs();
        if abs_out > self.diag_peak_output {
            self.diag_peak_output = abs_out;
        }
        if abs_out > 10.0 {
            self.diag_clamp_count += 1;
        }

        clamp_output(dc_blocked)
    }

    /// Process multiple samples (batch).
    pub fn process_samples(&mut self, input: &[f64], output: &mut [f64]) {
        for (i, &inp) in input.iter().enumerate() {
            output[i] = self.process_sample(inp);
        }
    }

    /// Reset solver state.
    pub fn reset(&mut self) {
        self.v_prev.fill(0.0);
        self.input_prev = 0.0;
        reset_inductors(&mut self.kernel.inductors);
        reset_coupled_inductors(&mut self.kernel.coupled_inductors);
        reset_transformer_groups(&mut self.kernel.transformer_groups);
        self.dc_block_x_prev = 0.0;
        self.dc_block_y_prev = 0.0;
        self.diag_peak_output = 0.0;
        self.diag_clamp_count = 0;
        self.diag_nan_reset_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mna::MnaSystem;
    use crate::parser::Netlist;

    #[test]
    fn test_linear_rc_solver() {
        let spice = "RC Circuit\nR1 in 0 1k\nC1 in 0 1u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let mut solver = LinearSolver::new(kernel, 0, 0);
        solver.input_conductance = 0.001;

        let mut output = vec![0.0; 500];
        for i in 0..500 {
            output[i] = solver.process_sample(1.0);
        }

        let peak = output.iter().fold(0.0_f64, |a, &b| a.max(b));
        assert!(peak > 0.8, "RC peak = {:.4}, expected > 0.8", peak);
        assert!(output[0] > 0.0, "First sample should be positive");
        for i in 1..100 {
            assert!(
                output[i] >= output[i - 1] - 1e-10,
                "Monotonic: [{}]={:.6} < [{}]={:.6}",
                i, output[i], i - 1, output[i - 1]
            );
        }
    }
}
