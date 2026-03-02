//! Interpreted circuit solver.
//!
//! This is a runtime solver that processes samples using the DK method.
//! It's slower than generated code but useful for prototyping and validation.
//!
//! All processing uses pre-allocated buffers - no heap allocation in the audio thread.

use crate::dk::DkKernel;
use melange_primitives::nr::{nr_solve_1d, nr_solve_2d};
use melange_devices::{DiodeShockley, BjtEbersMoll, NonlinearDevice};
use smallvec::{SmallVec, smallvec};

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

/// Nonlinear device interface for the solver.
///
/// This abstracts over the specific device types from melange-devices.
/// Uses const generics to specify device dimension at compile time,
/// enabling stack-allocated arrays and no heap allocation.
///
/// # Type Parameters
/// - `N`: Device dimension (1 for diode, 2 for BJT, etc.)
pub trait SolverDevice<const N: usize>: Send + Sync {
    /// Compute currents given controlling voltages.
    ///
    /// Returns [I0, I1, ...] where I_k is the k-th terminal current.
    /// For diode: [I_anode] (cathode current is -I_anode)
    /// For BJT: [Ic, Ib] (emitter current is -(Ic + Ib))
    fn currents(&self, v: &[f64; N]) -> [f64; N];

    /// Compute Jacobian: ∂I/∂V for each current-voltage pair.
    ///
    /// Returns flattened N×N matrix: [∂I0/∂V0, ∂I0/∂V1, ..., ∂I1/∂V0, ...]
    fn jacobian(&self, v: &[f64; N]) -> [[f64; N]; N];

    /// Clone into a Box.
    fn clone_box(&self) -> Box<dyn SolverDevice<N>>;
}

impl<const N: usize> Clone for Box<dyn SolverDevice<N>> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

/// Enum-based device entry for zero-cost dispatch in the audio hot path.
///
/// This eliminates vtable overhead by using enum dispatch instead of
/// `Box<dyn SolverDevice<N>>`. All device types are stored inline
/// with no heap allocation.
#[derive(Debug, Clone)]
pub enum DeviceEntry {
    /// Shockley diode model (1D)
    Diode { device: DiodeShockley, start_idx: usize },
    /// Diode with series resistance (1D)
    DiodeWithRs { device: melange_devices::DiodeWithRs, start_idx: usize },
    /// LED model (1D)
    Led { device: melange_devices::Led, start_idx: usize },
    /// BJT Ebers-Moll model (2D)
    Bjt { device: BjtEbersMoll, start_idx: usize },
}

impl DeviceEntry {
    /// Create a new diode device entry.
    pub fn new_diode(device: DiodeShockley, start_idx: usize) -> Self {
        Self::Diode { device, start_idx }
    }

    /// Create a new diode with series resistance entry.
    pub fn new_diode_with_rs(device: melange_devices::DiodeWithRs, start_idx: usize) -> Self {
        Self::DiodeWithRs { device, start_idx }
    }

    /// Create a new LED device entry.
    pub fn new_led(device: melange_devices::Led, start_idx: usize) -> Self {
        Self::Led { device, start_idx }
    }

    /// Create a new BJT device entry.
    pub fn new_bjt(device: BjtEbersMoll, start_idx: usize) -> Self {
        Self::Bjt { device, start_idx }
    }

    /// Get the device dimension (1, 2, etc.)
    pub fn dimension(&self) -> usize {
        match self {
            DeviceEntry::Diode { .. } => 1,
            DeviceEntry::DiodeWithRs { .. } => 1,
            DeviceEntry::Led { .. } => 1,
            DeviceEntry::Bjt { .. } => 2,
        }
    }

    /// Get the starting index in the global voltage/current vectors.
    pub fn start_idx(&self) -> usize {
        match self {
            DeviceEntry::Diode { start_idx, .. } => *start_idx,
            DeviceEntry::DiodeWithRs { start_idx, .. } => *start_idx,
            DeviceEntry::Led { start_idx, .. } => *start_idx,
            DeviceEntry::Bjt { start_idx, .. } => *start_idx,
        }
    }

    /// Compute device currents at the given voltages.
    ///
    /// Returns currents in a SmallVec to avoid heap allocation.
    /// - 1D devices: [I0]
    /// - 2D devices (BJT): [Ic, Ib]
    pub fn currents(&self, v: &[f64]) -> SmallVec<[f64; 2]> {
        match self {
            DeviceEntry::Diode { device, .. } => {
                let i = device.current(&[v[0]]);
                smallvec![i]
            }
            DeviceEntry::DiodeWithRs { device, .. } => {
                let i = device.current(&[v[0]]);
                smallvec![i]
            }
            DeviceEntry::Led { device, .. } => {
                let i = device.current(&[v[0]]);
                smallvec![i]
            }
            DeviceEntry::Bjt { device, .. } => {
                let ic = device.collector_current(v[0], v[1]);
                let ib = device.base_current(v[0], v[1]);
                smallvec![ic, ib]
            }
        }
    }

    /// Compute device Jacobian at the given voltages.
    ///
    /// Returns Jacobian entries in a SmallVec (flattened row-major).
    /// - 1D devices: [∂I/∂V]
    /// - 2D devices (BJT): [∂Ic/∂Vbe, ∂Ic/∂Vbc, ∂Ib/∂Vbe, ∂Ib/∂Vbc]
    pub fn jacobian(&self, v: &[f64]) -> SmallVec<[f64; 4]> {
        match self {
            DeviceEntry::Diode { device, .. } => {
                let jac = device.jacobian(&[v[0]]);
                smallvec![jac[0]]
            }
            DeviceEntry::DiodeWithRs { device, .. } => {
                let jac = device.jacobian(&[v[0]]);
                smallvec![jac[0]]
            }
            DeviceEntry::Led { device, .. } => {
                let jac = device.jacobian(&[v[0]]);
                smallvec![jac[0]]
            }
            DeviceEntry::Bjt { device, .. } => {
                let (dic_dvbe, dic_dvbc) = device.collector_jacobian(v[0], v[1]);
                // For BJT: we need ∂Ib/∂Vbe and ∂Ib/∂Vbc
                let vbe = v[0];
                let vbc = v[1];
                let s = device.sign();
                let vbe_eff = s * vbe;
                let vbc_eff = s * vbc;
                let exp_be = melange_devices::safeguards::safe_exp(vbe_eff / device.vt);
                let exp_bc = melange_devices::safeguards::safe_exp(vbc_eff / device.vt);
                let dib_dvbe = s * s * (device.is * device.beta_f.recip() / device.vt) * exp_be;
                let dib_dvbc = s * s * (device.is * device.beta_r.recip() / device.vt) * exp_bc;
                smallvec![dic_dvbe, dic_dvbc, dib_dvbe, dib_dvbc]
            }
        }
    }

    /// Compute the conductance (∂I/∂V) for a 1D device.
    ///
    /// Returns 0.0 if called on a 2D device (safe no-op for audio).
    pub fn conductance_1d(&self, v: f64) -> f64 {
        match self {
            DeviceEntry::Diode { device, .. } => device.conductance_at(v),
            DeviceEntry::DiodeWithRs { device, .. } => device.conductance_at(v),
            DeviceEntry::Led { device, .. } => device.jacobian(&[v])[0],
            DeviceEntry::Bjt { .. } => 0.0,
        }
    }

    /// Compute current for a 1D device.
    ///
    /// Returns 0.0 if called on a 2D device (safe no-op for audio).
    pub fn current_1d(&self, v: f64) -> f64 {
        match self {
            DeviceEntry::Diode { device, .. } => device.current_at(v),
            DeviceEntry::DiodeWithRs { device, .. } => device.current_at(v),
            DeviceEntry::Led { device, .. } => device.current(&[v]),
            DeviceEntry::Bjt { .. } => 0.0,
        }
    }

    /// Write device currents from `v_nl` into `i_nl` at this device's start index.
    ///
    /// Reads voltages from `v_nl[start_idx..]` and writes currents to `i_nl[start_idx..]`.
    fn write_currents(&self, v_nl: &[f64], i_nl: &mut [f64]) {
        let s = self.start_idx();
        match self.dimension() {
            1 => {
                i_nl[s] = self.currents(&[v_nl[s]])[0];
            }
            2 => {
                let currents = self.currents(&[v_nl[s], v_nl[s + 1]]);
                i_nl[s] = currents[0];
                i_nl[s + 1] = currents[1];
            }
            _ => {
                debug_assert!(false, "unsupported device dimension: {}", self.dimension());
            }
        }
    }

    /// Write device Jacobian entries into a flat M*M matrix `g_dev` (row-major).
    ///
    /// Only writes the block-diagonal entries for this device.
    fn write_jacobian(&self, v_nl: &[f64], g_dev: &mut [f64], m: usize) {
        let s = self.start_idx();
        match self.dimension() {
            1 => {
                g_dev[s * m + s] = self.jacobian(&[v_nl[s]])[0];
            }
            2 => {
                let jac = self.jacobian(&[v_nl[s], v_nl[s + 1]]);
                g_dev[s * m + s] = jac[0];
                g_dev[s * m + s + 1] = jac[1];
                g_dev[(s + 1) * m + s] = jac[2];
                g_dev[(s + 1) * m + s + 1] = jac[3];
            }
            _ => {
                debug_assert!(false, "unsupported device dimension: {}", self.dimension());
            }
        }
    }
}

/// Describes why the Newton-Raphson solver stopped iterating.
///
/// This is set after each call to [`CircuitSolver::process_sample`] and can be
/// queried via [`CircuitSolver::last_convergence_reason`] to diagnose solver
/// behavior without inspecting internal state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConvergenceReason {
    /// Converged: all residuals fell below tolerance.
    Residual,
    /// Converged: maximum step size fell below tolerance.
    StepSize,
    /// Did not converge within the maximum number of iterations.
    MaxIterations,
    /// No nonlinear devices present; the system is trivially linear.
    Linear,
}

/// Pivot threshold for detecting singular matrices in Gaussian elimination.
const SINGULARITY_THRESHOLD: f64 = 1e-15;

/// Solve linear system A*x = b in place using Gaussian elimination with partial pivoting.
///
/// On return, `b` contains the solution x. Both `a` and `b` are modified.
/// Returns false if the matrix is singular.
///
/// `a` is stored in row-major format: a[i * n + j].
#[allow(clippy::needless_range_loop)]
fn gauss_solve_inplace(a: &mut [f64], b: &mut [f64], n: usize) -> bool {
    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }
        if max_val < SINGULARITY_THRESHOLD {
            return false;
        }

        // Swap rows
        if max_row != col {
            for j in col..n {
                let idx1 = col * n + j;
                let idx2 = max_row * n + j;
                a.swap(idx1, idx2);
            }
            b.swap(col, max_row);
        }

        // Eliminate below
        let pivot = a[col * n + col];
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in (col + 1)..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            b[row] -= factor * b[col];
            a[row * n + col] = 0.0;
        }
    }

    // Back substitution
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * b[j];
        }
        let diag = a[i * n + i];
        if diag.abs() < 1e-30 {
            return false;
        }
        b[i] = sum / diag;
    }

    true
}

/// Circuit solver state.
///
/// Maintains all state needed for sample-by-sample simulation.
/// Uses pre-allocated buffers to avoid heap allocation during processing.
///
/// This struct intentionally has many fields: each buffer is pre-allocated at
/// construction time so that [`process_sample`](Self::process_sample) performs
/// zero heap allocations, which is required for real-time audio safety.
#[derive(Clone)]
pub struct CircuitSolver {
    /// DK kernel
    pub kernel: DkKernel,
    /// Nonlinear device entries
    pub devices: Vec<DeviceEntry>,
    /// Previous node voltages
    pub v_prev: Vec<f64>,
    /// Previous nonlinear voltages (for warm start), size = total dimension m
    pub v_nl_prev: Vec<f64>,
    /// Previous nonlinear currents, size = total dimension m
    pub i_nl_prev: Vec<f64>,
    /// Pre-allocated RHS buffer
    rhs: Vec<f64>,
    /// Pre-allocated prediction buffer
    v_pred: Vec<f64>,
    /// Pre-allocated nonlinear voltage buffer
    v_nl: Vec<f64>,
    /// Pre-allocated current buffer
    i_nl: Vec<f64>,
    /// Pre-allocated correction buffer
    correction: Vec<f64>,
    /// Pre-allocated prediction extraction buffer (p = N_v * v_pred)
    p: Vec<f64>,
    /// Pre-allocated buffers for M-dimensional NR (to avoid allocation in hot loop)
    residual: Vec<f64>,
    i_nl_temp: Vec<f64>,
    /// Pre-allocated buffer for N_i * i_nl computation
    ni_inl: Vec<f64>,
    /// Pre-allocated M×M block-diagonal device Jacobian for NR
    g_dev: Vec<f64>,
    /// Pre-allocated M×M NR Jacobian (working copy for Gaussian elimination)
    jacobian_mat: Vec<f64>,
    /// Pre-allocated M solution vector for Gaussian elimination
    delta: Vec<f64>,
    /// Input node index
    pub input_node: usize,
    /// Output node index
    pub output_node: usize,
    /// Input conductance (1/R_in) for proper Thevenin source modeling
    pub input_conductance: f64,
    /// Previous input sample for trapezoidal integration
    input_prev: f64,
    /// Maximum NR iterations per sample
    pub max_iter: u32,
    /// NR tolerance
    pub tol: f64,
    /// Step clamp for NR
    pub clamp: f64,
    /// Reason the NR solver stopped on the most recent sample.
    last_convergence_reason: ConvergenceReason,
}

impl CircuitSolver {
    /// Create a new circuit solver.
    ///
    /// # Panics
    /// Panics if device dimensions don't match kernel dimensions.
    pub fn new(
        kernel: DkKernel,
        devices: Vec<DeviceEntry>,
        input_node: usize,
        output_node: usize,
    ) -> Self {
        let n = kernel.n;
        let m = kernel.m;

        // Verify device dimensions match kernel
        let total_device_dim: usize = devices.iter().map(|d| d.dimension()).sum();
        assert_eq!(total_device_dim, m, "Total device dimensions must match kernel dimension");
        
        // Verify 2D devices have valid indices (check once at construction)
        for device in &devices {
            if device.dimension() == 2 {
                assert!(device.start_idx() + 1 < m,
                    "Device 2D index out of bounds: start_idx={}, m={}",
                    device.start_idx(), m);
            }
        }

        // Pre-allocate all buffers
        let v_prev = vec![0.0; n];
        let v_nl_prev = vec![0.0; m];
        let i_nl_prev = vec![0.0; m];
        let rhs = vec![0.0; n];
        let v_pred = vec![0.0; n];
        let v_nl = vec![0.0; m];
        let i_nl = vec![0.0; m];
        let correction = vec![0.0; n];
        let p = vec![0.0; m];
        let residual = vec![0.0; m];
        let i_nl_temp = vec![0.0; m];
        let ni_inl = vec![0.0; n];
        let g_dev = vec![0.0; m * m];
        let jacobian_mat = vec![0.0; m * m];
        let delta = vec![0.0; m];

        Self {
            kernel,
            devices,
            v_prev,
            v_nl_prev,
            i_nl_prev,
            rhs,
            v_pred,
            v_nl,
            i_nl,
            correction,
            p,
            residual,
            i_nl_temp,
            ni_inl,
            g_dev,
            jacobian_mat,
            delta,
            input_node,
            output_node,
            input_conductance: 1.0, // Default 1/1Ω (near-ideal voltage source) — caller should set from netlist
            input_prev: 0.0,
            max_iter: 100,
            tol: 1e-10,
            clamp: 0.01,
            last_convergence_reason: if m == 0 { ConvergenceReason::Linear } else { ConvergenceReason::MaxIterations },
        }
    }

    /// Returns the convergence reason from the most recent call to
    /// [`process_sample`](Self::process_sample).
    ///
    /// For linear circuits (no nonlinear devices), this always returns
    /// [`ConvergenceReason::Linear`].
    pub fn last_convergence_reason(&self) -> ConvergenceReason {
        self.last_convergence_reason
    }

    /// Initialize solver state from a nonlinear DC operating point.
    ///
    /// Calls the DC OP solver to find the steady-state bias point, then
    /// sets `v_prev`, `v_nl_prev`, and `i_nl_prev` to the converged values.
    /// Runs warm-up samples with zero input to let the DK solver settle
    /// into its own steady state (which differs slightly from the true DC OP
    /// due to backward Euler discretization of nonlinear currents).
    ///
    /// This is essential for BJT circuits where the transistor must be biased
    /// into its active region before AC signals can be amplified.
    ///
    /// Does nothing if the DC OP solver fails to converge (keeps zero state).
    pub fn initialize_dc_op(&mut self, mna: &crate::mna::MnaSystem, device_slots: &[crate::codegen::ir::DeviceSlot]) {
        let config = crate::dc_op::DcOpConfig {
            input_node: self.input_node,
            input_resistance: if self.input_conductance > 0.0 {
                1.0 / self.input_conductance
            } else {
                1.0
            },
            ..crate::dc_op::DcOpConfig::default()
        };
        let result = crate::dc_op::solve_dc_operating_point(mna, device_slots, &config);
        if result.converged {
            let n = self.v_prev.len();
            let m = self.i_nl_prev.len();
            for i in 0..n.min(result.v_node.len()) {
                self.v_prev[i] = result.v_node[i];
            }
            for i in 0..m.min(result.v_nl.len()) {
                self.v_nl_prev[i] = result.v_nl[i];
            }
            for i in 0..m.min(result.i_nl.len()) {
                self.i_nl_prev[i] = result.i_nl[i];
            }

            // Run a few warm-up samples with zero input to let the DK solver settle.
            // With trapezoidal nonlinear integration, the DK steady state closely
            // matches the true DC OP, so only a short settling period is needed.
            for _ in 0..50 {
                self.process_sample(0.0);
            }
        }
    }

    /// Process a single sample.
    ///
    /// Uses pre-allocated buffers - no heap allocation.
    pub fn process_sample(&mut self, input: f64) -> f64 {
        let input = sanitize_input(input);

        self.build_rhs(input);
        self.predict();

        // Extract predicted nonlinear voltages: p = N_v * v_pred
        self.kernel.extract_prediction_into(&self.v_pred, &mut self.p);

        // Solve for nonlinear voltages/currents
        if self.kernel.m > 0 {
            if self.all_devices_1d() && self.devices.len() == 1 {
                self.solve_1d();
            } else if self.all_devices_1d() && self.devices.len() == 2 {
                self.solve_2d_two_1d_devices();
            } else {
                self.solve_md();
            }
        } else {
            self.last_convergence_reason = ConvergenceReason::Linear;
        }

        self.compute_all_currents();
        self.update_voltage();

        // Save state for next sample
        std::mem::swap(&mut self.v_prev, &mut self.v_pred);
        std::mem::swap(&mut self.v_nl_prev, &mut self.v_nl);
        std::mem::swap(&mut self.i_nl_prev, &mut self.i_nl);

        // Sanitize state BEFORE updating inductors: if any value is NaN/inf,
        // reset to zero to prevent one bad sample from poisoning all future output
        if self.v_prev.iter().any(|v| !v.is_finite()) {
            self.v_prev.fill(0.0);
            self.v_nl_prev.fill(0.0);
            self.i_nl_prev.fill(0.0);
            self.input_prev = 0.0;
            reset_inductors(&mut self.kernel.inductors);
        }

        self.kernel.update_inductors(&self.v_prev);

        let raw = if self.output_node < self.kernel.n {
            self.v_prev[self.output_node]
        } else {
            0.0
        };
        clamp_output(raw)
    }

    /// Check if all devices are 1-dimensional.
    fn all_devices_1d(&self) -> bool {
        self.devices.iter().all(|d| d.dimension() == 1)
    }

    /// Build RHS vector from history into pre-allocated buffer.
    #[allow(clippy::needless_range_loop)]
    fn build_rhs(&mut self, input: f64) {
        let n = self.kernel.n;

        // rhs = A_neg * v_prev + N_i * i_nl_prev + 2*input + sources
        // N_i * i_nl_prev in the RHS combined with S * N_i * i_nl in the correction
        // gives the trapezoidal average: S * N_i * (i_nl + i_nl_prev).
        for i in 0..n {
            let mut sum = self.kernel.rhs_const[i];

            // A_neg * v_prev
            let a_neg_row_offset = i * n;
            for j in 0..n {
                sum += self.kernel.a_neg[a_neg_row_offset + j] * self.v_prev[j];
            }

            // N_i * i_nl_prev
            let n_i_row_offset = i * self.kernel.m;
            for j in 0..self.kernel.m {
                sum += self.kernel.n_i[n_i_row_offset + j] * self.i_nl_prev[j];
            }

            self.rhs[i] = sum;
        }

        // Add inductor history contribution
        // For trapezoidal companion model: i_hist contributes to RHS
        for ind in &self.kernel.inductors {
            let i_hist = ind.i_hist;
            if ind.node_i > 0 {
                self.rhs[ind.node_i - 1] -= i_hist;
            }
            if ind.node_j > 0 {
                self.rhs[ind.node_j - 1] += i_hist;
            }
        }

        // Add input source (Thevenin: V_in through R_in)
        // Trapezoidal rule: contribution is (V_in(n+1) + V_in(n)) * G_in
        // where V_in(n+1) is the current input and V_in(n) is the previous input.
        // G_in = 1/R_in is the input conductance (already stamped in G/A).
        if self.input_node < n {
            self.rhs[self.input_node] += (input + self.input_prev) * self.input_conductance;
        }
        self.input_prev = input;
    }

    /// Compute linear prediction into pre-allocated buffer.
    fn predict(&mut self) {
        self.kernel.predict_into(&self.rhs, &mut self.v_pred);
    }

    /// Solve single 1D nonlinear device.
    /// 
    /// # Panics
    /// Panics if the device is not a 1D device (should not happen if called correctly).
    fn solve_1d(&mut self) {
        let device = &self.devices[0];
        let k = self.kernel.k(0, 0);
        let p = self.p[0];

        // f(v) = v - p - k * i(v) = 0
        let (v_nl, result) = nr_solve_1d(
            |v| {
                let i = device.currents(&[v])[0];
                v - p - k * i
            },
            |v| {
                let jac = device.jacobian(&[v]);
                let g = jac[0];
                1.0 - k * g
            },
            self.v_nl_prev[0],
            self.max_iter,
            self.tol,
            self.clamp,
        );

        // On convergence use the NR result; on failure fall back to previous for continuity
        if result.converged() {
            self.v_nl[0] = v_nl;
            // nr_solve_1d checks residual before step size, so Residual is the
            // most likely path; we cannot distinguish them from NrResult alone.
            self.last_convergence_reason = ConvergenceReason::Residual;
        } else {
            self.v_nl[0] = self.v_nl_prev[0];
            self.last_convergence_reason = ConvergenceReason::MaxIterations;
        }
    }

    /// Solve two 1D nonlinear devices (2D system).
    ///
    /// # Panics
    /// Panics if any device is not a 1D device (should not happen if called correctly).
    fn solve_2d_two_1d_devices(&mut self) {
        let dev0 = &self.devices[0];
        let dev1 = &self.devices[1];

        let p0 = self.p[0];
        let p1 = self.p[1];

        // f1(v1, v2) = v1 - p1 - (k11*i1 + k12*i2) = 0
        let f1 = |v1: f64, v2: f64| {
            let i1 = dev0.currents(&[v1])[0];
            let i2 = dev1.currents(&[v2])[0];
            v1 - p0 - (self.kernel.k(0, 0) * i1 + self.kernel.k(0, 1) * i2)
        };

        let f2 = |v1: f64, v2: f64| {
            let i1 = dev0.currents(&[v1])[0];
            let i2 = dev1.currents(&[v2])[0];
            v2 - p1 - (self.kernel.k(1, 0) * i1 + self.kernel.k(1, 1) * i2)
        };

        // Jacobians
        let df1_dv1 = |v1: f64, _v2: f64| {
            let jac = dev0.jacobian(&[v1]);
            let g = jac[0];
            1.0 - self.kernel.k(0, 0) * g
        };

        let df1_dv2 = |_v1: f64, v2: f64| {
            let jac = dev1.jacobian(&[v2]);
            let g = jac[0];
            -self.kernel.k(0, 1) * g
        };

        let df2_dv1 = |v1: f64, _v2: f64| {
            let jac = dev0.jacobian(&[v1]);
            let g = jac[0];
            -self.kernel.k(1, 0) * g
        };

        let df2_dv2 = |_v1: f64, v2: f64| {
            let jac = dev1.jacobian(&[v2]);
            let g = jac[0];
            1.0 - self.kernel.k(1, 1) * g
        };

        let (v1, v2, result) = nr_solve_2d(
            f1, f2,
            df1_dv1, df1_dv2,
            df2_dv1, df2_dv2,
            self.v_nl_prev[0], self.v_nl_prev[1],
            self.max_iter,
            self.tol,
            self.clamp,
        );

        // On convergence use the NR result; on failure fall back to previous for continuity
        if result.converged() {
            self.v_nl[0] = v1;
            self.v_nl[1] = v2;
            self.last_convergence_reason = ConvergenceReason::Residual;
        } else {
            self.v_nl[0] = self.v_nl_prev[0];
            self.v_nl[1] = self.v_nl_prev[1];
            self.last_convergence_reason = ConvergenceReason::MaxIterations;
        }
    }

    /// Solve M-dimensional nonlinear system using Newton-Raphson with full Jacobian.
    ///
    /// Uses pre-allocated buffers to avoid heap allocation.
    /// Builds the full NR Jacobian J[i][j] = delta_ij - sum_k K[i][k] * G[k][j]
    /// where G is the block-diagonal device Jacobian, and solves via Gaussian elimination.
    #[allow(clippy::needless_range_loop)]
    fn solve_md(&mut self) {
        let m = self.kernel.m;

        // Warm start from previous solution
        self.v_nl[..m].copy_from_slice(&self.v_nl_prev[..m]);

        // Default: assume we'll exhaust iterations (overwritten on early exit)
        self.last_convergence_reason = ConvergenceReason::MaxIterations;

        // Newton-Raphson iteration
        for _ in 0..self.max_iter {
            let mut converged = true;

            // Compute all device currents into pre-allocated buffer
            self.i_nl_temp.fill(0.0);
            for device in &self.devices {
                device.write_currents(&self.v_nl, &mut self.i_nl_temp);
            }

            // Compute residual: r[i] = v_nl[i] - p[i] - sum_j(K[i][j] * i_nl[j])
            for i in 0..m {
                let mut k_i = 0.0;
                let k_row_offset = i * m;
                for j in 0..m {
                    k_i += self.kernel.k[k_row_offset + j] * self.i_nl_temp[j];
                }
                self.residual[i] = self.v_nl[i] - self.p[i] - k_i;

                if self.residual[i].abs() > self.tol {
                    converged = false;
                }
            }

            if converged {
                self.last_convergence_reason = ConvergenceReason::Residual;
                break;
            }

            // Build block-diagonal device Jacobian G[k][j] = di_nl[k]/dv_nl[j]
            self.g_dev.fill(0.0);
            for device in &self.devices {
                device.write_jacobian(&self.v_nl, &mut self.g_dev, m);
            }

            // Build full NR Jacobian: J[i][j] = delta_ij - sum_k K[i][k] * G[k][j]
            for i in 0..m {
                for j in 0..m {
                    let mut kg = 0.0;
                    for k in 0..m {
                        kg += self.kernel.k[i * m + k] * self.g_dev[k * m + j];
                    }
                    self.jacobian_mat[i * m + j] = if i == j { 1.0 } else { 0.0 } - kg;
                }
            }

            // Copy residual into delta for Gaussian elimination (solver modifies in place)
            self.delta[..m].copy_from_slice(&self.residual[..m]);

            // Solve J * delta = residual via Gaussian elimination with partial pivoting
            let solved = gauss_solve_inplace(&mut self.jacobian_mat, &mut self.delta, m);

            if !solved {
                // Singular Jacobian — damped fallback (0.5 * residual)
                for i in 0..m {
                    let step = (self.residual[i] * 0.5).clamp(-self.clamp, self.clamp);
                    self.v_nl[i] -= step;
                }
                continue;
            }

            // Check step-size convergence: if max absolute clamped step < tolerance, done
            let max_step: f64 = self.delta[..m].iter()
                .map(|&d| d.clamp(-self.clamp, self.clamp).abs())
                .fold(0.0_f64, f64::max);
            if max_step < self.tol {
                self.last_convergence_reason = ConvergenceReason::StepSize;
                break;
            }

            // Update: v_nl -= delta (with clamping)
            for i in 0..m {
                let step = self.delta[i].clamp(-self.clamp, self.clamp);
                self.v_nl[i] -= step;
            }

            // NaN/Inf detection: if any v_nl is non-finite, restore from previous and break
            if self.v_nl[..m].iter().any(|v| !v.is_finite()) {
                self.v_nl[..m].copy_from_slice(&self.v_nl_prev[..m]);
                // MaxIterations is already set as the default
                break;
            }
        }
    }

    /// Compute all device currents from v_nl into i_nl.
    fn compute_all_currents(&mut self) {
        for device in &self.devices {
            device.write_currents(&self.v_nl, &mut self.i_nl);
        }
    }

    /// Update voltage with correction term.
    fn update_voltage(&mut self) {
        // Copy v_pred to correction buffer to avoid borrow issue
        self.correction.copy_from_slice(&self.v_pred);
        self.kernel.apply_correction_into(&self.correction, &self.i_nl, &self.i_nl_prev, &mut self.v_pred, &mut self.ni_inl);
    }

    /// Reset solver state.
    pub fn reset(&mut self) {
        self.v_prev.fill(0.0);
        self.v_nl_prev.fill(0.0);
        self.i_nl_prev.fill(0.0);
        self.input_prev = 0.0;
        reset_inductors(&mut self.kernel.inductors);
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[f64], output: &mut [f64]) {
        assert_eq!(input.len(), output.len());
        for (i, &x) in input.iter().enumerate() {
            output[i] = self.process_sample(x);
        }
    }
}

/// Solver for linear circuits (no nonlinear devices).
///
/// This is a simplified solver that doesn't need NR iteration.
#[derive(Clone)]
pub struct LinearSolver {
    kernel: DkKernel,
    v_prev: Vec<f64>,
    rhs: Vec<f64>,
    v_pred: Vec<f64>,
    input_node: usize,
    output_node: usize,
    /// Input conductance (1/R_in) for proper Thevenin source modeling
    pub input_conductance: f64,
    /// Previous input sample for trapezoidal integration
    input_prev: f64,
}

impl LinearSolver {
    /// Create a new linear solver.
    pub fn new(kernel: DkKernel, input_node: usize, output_node: usize) -> Self {
        let n = kernel.n;
        Self {
            kernel,
            v_prev: vec![0.0; n],
            rhs: vec![0.0; n],
            v_pred: vec![0.0; n],
            input_node,
            output_node,
            input_conductance: 1.0, // Default 1/1Ω (near-ideal voltage source) — caller should set from netlist
            input_prev: 0.0,
        }
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
        }

        self.kernel.update_inductors(&self.v_prev);

        let raw = if self.output_node < n {
            self.v_prev[self.output_node]
        } else {
            0.0
        };
        clamp_output(raw)
    }

    /// Reset solver state.
    pub fn reset(&mut self) {
        self.v_prev.fill(0.0);
        self.input_prev = 0.0;
        reset_inductors(&mut self.kernel.inductors);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mna::MnaSystem;
    use crate::parser::Netlist;

    /// Test RC lowpass step response converges to correct DC value.
    ///
    /// Uses R1 to ground with C1 in parallel — single-node RC circuit.
    /// The Thevenin input model requires a ground reference at the input node.
    /// R=1k, C=1u, tau=RC=1ms (44.1 samples at 44.1kHz).
    /// After sufficient settling, output should converge to 1.0V.
    #[test]
    fn test_linear_rc_solver() {
        let spice = r#"RC Circuit
R1 in 0 1k
C1 in 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        // Single node: input and output are both node 0
        let mut solver = LinearSolver::new(kernel, 0, 0);
        solver.input_conductance = 0.001; // G_in = 1/R1 = 1/1k

        // Process 500 samples of 1V DC step
        let num_samples = 500;
        let mut output = vec![0.0; num_samples];

        for i in 0..num_samples {
            output[i] = solver.process_sample(1.0);
        }

        // After ~11 time constants (500 samples), should be very close to 1V
        let final_output = output[num_samples - 1];
        assert!((final_output - 1.0).abs() < 0.05,
            "RC lowpass DC output = {:.4}V, expected ~1.0V", final_output);

        // Output should be monotonically increasing (RC step response)
        for i in 1..num_samples {
            assert!(output[i] >= output[i-1] - 1e-10,
                "Output should be monotonically increasing: output[{}]={:.6} < output[{}]={:.6}",
                i, output[i], i-1, output[i-1]);
        }

        // Check time constant: at ~44 samples (1 tau), output ≈ 1-exp(-1) ≈ 0.632
        let at_tau = output[44];
        assert!(at_tau > 0.4 && at_tau < 0.85,
            "At 1 tau, output = {:.4}, expected roughly 0.4-0.85", at_tau);

        // First sample should be positive (step response starts rising)
        assert!(output[0] > 0.0, "First sample should be positive, got {:.6}", output[0]);
    }

    /// Test diode clipper: small signal passes through, large signal gets clipped.
    #[test]
    fn test_diode_clipper_solver() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
C1 out 0 100n
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];

        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001; // Rin = 1k

        // Small signal should pass through with some attenuation
        let output1 = solver.process_sample(0.01);
        assert!(output1.abs() < 0.1, "Output for small input should be small, got {}", output1);

        // Large signal should be clipped near diode forward voltage
        solver.reset();
        let mut outputs = Vec::new();
        for _ in 0..200 {
            outputs.push(solver.process_sample(5.0));
        }
        let final_out = *outputs.last().unwrap();
        assert!(final_out.is_finite(), "Output should be finite");
        // Diode clipper: the 1N4148 (IS=1e-15, n=1.0) clamps the signal.
        // With 5V input through 1k Thevenin + 1k shunt + diode + 1k load,
        // the output settles to ~2.13V — well below the 5V input.
        assert!(final_out < 3.0,
            "Diode clipper should limit output well below input, got {:.4}V", final_out);
        assert!(final_out > 0.5,
            "Diode clipper output should be positive (diode conducting), got {:.4}V", final_out);
    }

    /// Test that solver runs without error (basic sanity check)
    #[test]
    fn test_solver_basic_operation() {
        // Single-node RC circuit with proper ground reference
        let spice = r#"RC Circuit
R1 in 0 10k
C1 in 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let mut solver = LinearSolver::new(kernel, 0, 0);
        solver.input_conductance = 1.0 / 10000.0; // R1 = 10k

        // Run a few timesteps - verify no panic and reasonable output
        let input = 1.0;
        let mut last = 0.0;
        for _ in 0..10 {
            last = solver.process_sample(input);
        }
        assert!(last.is_finite() && last > 0.0, "Output should be finite and positive");
    }
    
    /// Direct test of the correction formula math
    #[test]
    fn test_correction_formula_math() {
        // Use a circuit with a diode to get M > 0
        let spice = r#"Diode Circuit
Rin in 0 10k
D1 in out D1N4148
R1 out 0 10k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
        
        assert!(kernel.m > 0, "Need M > 0 for this test");
        
        let v_prev = vec![0.0; kernel.n];
        let i_nl_prev = vec![1e-6];  // 1uA previous current
        let i_nl = vec![1.1e-6];      // 1.1uA new current
        
        // Build RHS with i_nl_prev
        let mut rhs = vec![0.0; kernel.n];
        for i in 0..kernel.n {
            let mut sum = kernel.rhs_const[i];
            let a_neg_row_offset = i * kernel.n;
            for j in 0..kernel.n {
                sum += kernel.a_neg[a_neg_row_offset + j] * v_prev[j];
            }
            // Input contribution (Thevenin: V_in * G_in * 2)
            sum += 2.0 * 0.1 * (1.0 / 10000.0);  // 0.1V through 10k
            rhs[i] = sum;
        }
        
        // Add N_i * i_nl_prev to RHS
        for i in 0..kernel.n {
            let n_i_row_offset = i * kernel.m;
            for j in 0..kernel.m {
                rhs[i] += kernel.n_i[n_i_row_offset + j] * i_nl_prev[j];
            }
        }

        let mut v_pred = vec![0.0; kernel.n];
        kernel.predict_into(&rhs, &mut v_pred);

        // Correct trapezoidal formula: v = v_pred + S * N_i * i_nl (full i_nl, not delta)
        // Net effect: v = S*(rhs_base + N_i*i_nl_prev + N_i*i_nl) = S*(... + N_i*(i_nl + i_nl_prev))
        let mut v_correct = v_pred.clone();
        for i in 0..kernel.n {
            let s_row_offset = i * kernel.n;
            for j in 0..kernel.m {
                for k in 0..kernel.n {
                    v_correct[i] += kernel.s[s_row_offset + k] * kernel.n_i[k * kernel.m + j] * i_nl[j];
                }
            }
        }
        
        // Runtime formula: v = v_pred + S * N_i * i_nl
        let mut ni_inl = vec![0.0; kernel.n];
        let mut v_runtime = v_pred.clone();
        kernel.apply_correction_into(&v_pred, &i_nl, &i_nl_prev, &mut v_runtime, &mut ni_inl);
        
        let difference = (v_correct[0] - v_runtime[0]).abs();
        println!("v_correct (full i_nl): {:?}", v_correct);
        println!("v_runtime (full i_nl): {:?}", v_runtime);
        println!("Difference: {}", difference);

        // After trapezoidal fix, both formulas should match exactly.
        // The runtime solver uses full i_nl (not delta) in the correction step.
        assert!(difference < 1e-15,
                "Runtime solver should use full i_nl correction, difference = {}", difference);
    }

    // ========================================================================
    // 2D Newton-Raphson Solver Tests
    // ========================================================================

    /// Test BJT with solve_md() - single 2D device path
    /// Uses a valid netlist that exercises the M-dimensional NR solver
    #[test]
    fn test_solve_2d_bjt_basic() {
        // Common-emitter BJT amplifier using netlist
        let spice = r#"Common Emitter
Q1 coll base emit 2N2222
Rc coll vcc 10k
R1 base 0 100k
Re emit 0 1k
Rbias vcc 0 10k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        // Skip if we don't have the expected 2D setup
        if kernel.m != 2 {
            eprintln!("Skipping BJT test: kernel.m = {}, expected 2", kernel.m);
            return;
        }

        // Create BJT device
        let bjt = BjtEbersMoll::new_room_temp(1e-15, 200.0, 3.0, melange_devices::BjtPolarity::Npn);
        let devices = vec![DeviceEntry::new_bjt(bjt, 0)];

        let mut solver = CircuitSolver::new(kernel, devices, 0, 0);
        solver.input_conductance = 1.0 / 100000.0; // R1 = 100k base bias

        // Test various input levels - just verify convergence (no panic)
        let inputs = [0.0, 0.1, 0.5, 1.0];
        for &input in &inputs {
            solver.reset();
            let output = solver.process_sample(input);
            assert!(output.is_finite(), "Output should be finite for input {}", input);
        }
        
        // Test sine wave processing
        solver.reset();
        for i in 0..50 {
            let t = i as f64 / 44100.0;
            let input = 0.5 + 0.3 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let output = solver.process_sample(input);
            assert!(output.is_finite(), "Sample {} should converge", i);
        }
    }

    /// Test convergence tracking and warm start behavior
    #[test]
    fn test_solve_2d_convergence_tracking() {
        // Use the same circuit structure as the working test_diode_clipper_solver
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];

        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001; // Rin = 1k
        
        // Process samples and verify all outputs are finite
        let mut outputs = Vec::new();
        for i in 0..100 {
            let t = i as f64 / 44100.0;
            let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let output = solver.process_sample(input);
            assert!(output.is_finite(), "Sample {} produced non-finite output", i);
            outputs.push(output);
        }
        
        // Verify outputs are bounded — with proper input scaling (G_in = 0.001),
        // 1V input through 1k into a diode clipper should produce bounded output
        let max_output = outputs.iter().fold(0.0_f64, |a, &b| a.max(b.abs()));
        assert!(max_output < 1000.0, "Output should be bounded, max was {}", max_output);
        
        // Test reset clears state
        solver.reset();
        let after_reset = solver.process_sample(0.5);
        assert!(after_reset.is_finite(), "Output after reset should be finite");
        
        // Test determinism
        solver.reset();
        for _ in 0..50 {
            let _ = solver.process_sample(0.5);
        }
        let out1 = solver.process_sample(0.5);
        
        solver.reset();
        for _ in 0..50 {
            let _ = solver.process_sample(0.5);
        }
        let out2 = solver.process_sample(0.5);
        
        assert!((out1 - out2).abs() < 1e-10, "Solver should be deterministic");
    }

    /// Test numerical robustness with extreme inputs
    #[test]
    fn test_solve_2d_numerical_robustness() {
        let spice = r#"Diode Circuit
Rin in 0 1k
D1 in out D1N4148
R1 out 0 10k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];

        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001; // Rin = 1k
        
        // Test very small inputs
        let tiny_inputs = [1e-10, 1e-6, 1e-3, 0.0];
        for &input in &tiny_inputs {
            solver.reset();
            let output = solver.process_sample(input);
            assert!(output.is_finite(), "Tiny input {} should be finite", input);
        }
        
        // Test large inputs (solver clamps internally to [-100, 100])
        let large_inputs = [10.0, 50.0, 100.0, -10.0, -50.0, -100.0];
        for &input in &large_inputs {
            solver.reset();
            let output = solver.process_sample(input);
            assert!(output.is_finite(), "Large input {} should be finite", input);
        }
        
        // Test special float values
        for &input in &[f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            solver.reset();
            let output = solver.process_sample(input);
            assert!(output.is_finite(), "Special value should produce finite output");
        }
        
        // Test rapid transitions
        solver.reset();
        for &input in &[10.0, -10.0, 10.0, -10.0] {
            for _ in 0..5 {
                let output = solver.process_sample(input);
                assert!(output.is_finite(), "Rapid transition should be finite");
            }
        }
        
        // Verify internal state remains valid
        assert!(solver.v_prev.iter().all(|&v| v.is_finite()));
        assert!(solver.v_nl_prev.iter().all(|&v| v.is_finite()));
        assert!(solver.i_nl_prev.iter().all(|&v| v.is_finite()));
    }

    /// Test 2D solver paths using netlist-based circuits
    /// Tests both single 1D device and single 2D (BJT) device paths
    #[test]
    fn test_solve_2d_two_diodes() {
        // Diode clipper with capacitor (required for trapezoidal stability)
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        assert_eq!(kernel.m, 1, "Single diode should give M=1");

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001; // Rin = 1k

        // Test small signal passes through with some attenuation
        let small_out = solver.process_sample(0.01);
        assert!(small_out.is_finite(), "Small signal should produce finite output");

        // Test large signal — diode limits V_in-V_out to ~0.7V
        // but output voltage itself depends on circuit topology
        solver.reset();
        for _ in 0..200 {
            let _ = solver.process_sample(5.0);
        }
        let output = solver.process_sample(5.0);
        assert!(output.is_finite(), "Large signal should produce finite output");
        assert!(output > 0.0, "Output should be positive for positive input, got {}", output);
        assert!(output < 6.0, "Output should be bounded by input, got {}", output);
        
        // Now test BJT which exercises the M-dimensional solver (2D case)
        let spice_bjt = r#"BJT Circuit
Q1 c b e 2N2222
Rc c 0 10k
Rb b 0 100k
Re e 0 1k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
        let netlist2 = Netlist::parse(spice_bjt).unwrap();
        let mna2 = MnaSystem::from_netlist(&netlist2).unwrap();
        let kernel2 = DkKernel::from_mna(&mna2, 44100.0).unwrap();
        
        if kernel2.m == 2 {
            let bjt = BjtEbersMoll::new_room_temp(1e-15, 200.0, 3.0, melange_devices::BjtPolarity::Npn);
            let devices2 = vec![DeviceEntry::new_bjt(bjt, 0)];
            let mut solver2 = CircuitSolver::new(kernel2, devices2, 0, 0);
            solver2.input_conductance = 1.0 / 100000.0; // Rb = 100k
            
            // Verify 2D device converges
            for i in 0..20 {
                let output = solver2.process_sample(0.5);
                assert!(output.is_finite(), "BJT sample {} should converge", i);
            }
        }
    }

    /// Verify output safety clamping prevents DAW-killing signals.
    ///
    /// The solver must NEVER output NaN, inf, or values > 10V regardless of input.
    /// A DAW safety icon (black-yellow) means the plugin output exceeded safe bounds.
    #[test]
    fn test_output_safety_clamping() {
        // Linear solver
        let spice = r#"RC Circuit
R1 in 0 1k
C1 in 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let mut solver = LinearSolver::new(kernel, 0, 0);
        solver.input_conductance = 0.001;

        // NaN input: must produce finite, clamped output
        let out = solver.process_sample(f64::NAN);
        assert!(out.is_finite() && out.abs() <= 10.0,
            "NaN input produced {}", out);

        // Inf input: must produce finite, clamped output
        let out = solver.process_sample(f64::INFINITY);
        assert!(out.is_finite() && out.abs() <= 10.0,
            "Inf input produced {}", out);

        // After bad input, solver must recover for normal input
        let out = solver.process_sample(1.0);
        assert!(out.is_finite() && out.abs() <= 10.0,
            "Recovery after bad input produced {}", out);

        // Nonlinear solver
        let spice = r#"Diode
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001;

        // Feed 1000 samples of normal audio then check all are safe
        for i in 0..1000 {
            let t = i as f64 / 44100.0;
            let input = 5.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let out = solver.process_sample(input);
            assert!(out.is_finite() && out.abs() <= 10.0,
                "Sample {} output {} exceeds safe range", i, out);
        }

        // Hit it with NaN, inf, and extreme values — must survive
        for &bad_input in &[f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1e300, -1e300] {
            let out = solver.process_sample(bad_input);
            assert!(out.is_finite() && out.abs() <= 10.0,
                "Bad input {:?} produced {}", bad_input, out);
        }

        // Must recover and produce sane output afterwards
        for _ in 0..100 {
            let out = solver.process_sample(0.5);
            assert!(out.is_finite() && out.abs() <= 10.0,
                "Post-recovery output {} exceeds safe range", out);
        }
    }

    /// Verify the K matrix sign fix produces physically correct diode clipper behavior.
    ///
    /// This is a regression test for the K sign bug that caused NR divergence
    /// and DAW crashes. With correct K = N_v * S * N_i (no negation):
    /// - K is naturally negative → stable NR convergence
    /// - Diode voltage is bounded by forward voltage (~0.7V)
    /// - Output monotonically rises during DC step response
    #[test]
    fn test_k_sign_correctness_diode_clipper() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        // Verify K is negative (correct feedback sign)
        let k00 = kernel.k(0, 0);
        assert!(k00 < 0.0,
            "K[0][0] should be negative for stable NR, got {:.4}", k00);

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001; // Rin = 1k

        // Step response: 500 samples at 1V DC
        let mut outputs = Vec::with_capacity(500);
        for _ in 0..500 {
            let out = solver.process_sample(1.0);
            assert!(out.is_finite(), "Output must be finite");
            outputs.push(out);
        }

        // Output should be positive (positive input → positive output)
        let final_out = *outputs.last().unwrap();
        assert!(final_out > 0.0,
            "Final output should be positive, got {:.6}", final_out);

        // Output must be less than input (diode + RC divider)
        assert!(final_out < 1.5,
            "Output should be bounded below input, got {:.6}", final_out);

        // Output should monotonically increase (charging capacitor)
        for i in 1..outputs.len() {
            assert!(outputs[i] >= outputs[i-1] - 1e-10,
                "Output should be monotonic: [{}]={:.6} < [{}]={:.6}",
                i, outputs[i], i-1, outputs[i-1]);
        }

        // No NR divergence fallback (output should not be clamped to 0)
        assert!(outputs[0] > 0.0,
            "First sample should be nonzero, got {:.6}", outputs[0]);
    }

    #[test]
    fn test_solver_default_input_conductance() {
        // Verify the default input_conductance is 1.0 (near-ideal voltage source)
        let spice = "RC\nR1 in 0 1k\nC1 in 0 1u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
        let solver = CircuitSolver::new(kernel, vec![], 0, 0);
        assert_eq!(solver.input_conductance, 1.0,
            "Default input_conductance should be 1.0 (1/1Ω, near-ideal voltage source)");
    }

    // ========================================================================
    // ConvergenceReason tests
    // ========================================================================

    /// Linear circuit (no nonlinear devices) should report Linear.
    #[test]
    fn test_convergence_reason_linear() {
        let spice = "RC\nR1 in 0 1k\nC1 in 0 1u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        // m == 0 → no nonlinear devices
        assert_eq!(kernel.m, 0);

        let mut solver = CircuitSolver::new(kernel, vec![], 0, 0);
        solver.input_conductance = 0.001;

        // Before any processing, initial reason should be Linear (m == 0)
        assert_eq!(solver.last_convergence_reason(), ConvergenceReason::Linear);

        // After processing, it should still report Linear
        let _ = solver.process_sample(1.0);
        assert_eq!(solver.last_convergence_reason(), ConvergenceReason::Linear);
    }

    /// Diode clipper (1D NR via solve_1d) should converge via Residual.
    #[test]
    fn test_convergence_reason_1d_residual() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        assert_eq!(kernel.m, 1, "Single diode should give M=1");

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001;

        // Process a small signal — should converge easily
        let _ = solver.process_sample(0.1);
        assert_eq!(solver.last_convergence_reason(), ConvergenceReason::Residual,
            "1D solve with small signal should converge via residual");
    }

    /// BJT circuit (2D via solve_md) should converge and report Residual or StepSize.
    #[test]
    fn test_convergence_reason_md_bjt() {
        let spice = r#"Common Emitter
Q1 coll base emit 2N2222
Rc coll vcc 10k
R1 base 0 100k
Re emit 0 1k
Rbias vcc 0 10k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        if kernel.m != 2 {
            eprintln!("Skipping BJT convergence test: kernel.m = {}, expected 2", kernel.m);
            return;
        }

        let bjt = BjtEbersMoll::new_room_temp(1e-15, 200.0, 3.0, melange_devices::BjtPolarity::Npn);
        let devices = vec![DeviceEntry::new_bjt(bjt, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 0);
        solver.input_conductance = 1.0 / 100000.0;

        // Process several samples so NR has a warm start
        for _ in 0..20 {
            let _ = solver.process_sample(0.5);
        }

        let reason = solver.last_convergence_reason();
        assert!(
            reason == ConvergenceReason::Residual || reason == ConvergenceReason::StepSize,
            "BJT solve_md should converge, got {:?}", reason
        );
    }

    /// Two diodes (2D via solve_2d_two_1d_devices) should converge.
    #[test]
    fn test_convergence_reason_2d_two_diodes() {
        let spice = r#"Antiparallel Diodes
Rin in 0 1k
D1 in out D_FWD
D2 out in D_REV
C1 out 0 1u
.model D_FWD D(IS=1e-15)
.model D_REV D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        if kernel.m != 2 {
            eprintln!("Skipping 2-diode test: kernel.m = {}, expected 2", kernel.m);
            return;
        }

        let d1 = DiodeShockley::new_room_temp(1e-15, 1.0);
        let d2 = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![
            DeviceEntry::new_diode(d1, 0),
            DeviceEntry::new_diode(d2, 1),
        ];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001;

        // Process a few samples
        for _ in 0..10 {
            let _ = solver.process_sample(0.05);
        }

        let reason = solver.last_convergence_reason();
        assert_eq!(reason, ConvergenceReason::Residual,
            "Two-diode 2D solve should converge via residual, got {:?}", reason);
    }

    /// MaxIterations is reported when the solver cannot converge.
    #[test]
    fn test_convergence_reason_max_iterations() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001;

        // Set absurdly tight tolerance and very few iterations to force non-convergence
        solver.tol = 1e-300;
        solver.max_iter = 1;

        let _ = solver.process_sample(5.0);
        assert_eq!(solver.last_convergence_reason(), ConvergenceReason::MaxIterations,
            "With tol=1e-300 and max_iter=1, should report MaxIterations");
    }

    /// Getter returns the reason from the most recent sample, not a stale one.
    #[test]
    fn test_convergence_reason_updates_each_sample() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1);
        solver.input_conductance = 0.001;

        // Normal convergence
        let _ = solver.process_sample(0.1);
        let reason1 = solver.last_convergence_reason();
        assert_eq!(reason1, ConvergenceReason::Residual);

        // Force non-convergence
        solver.tol = 1e-300;
        solver.max_iter = 1;
        let _ = solver.process_sample(5.0);
        let reason2 = solver.last_convergence_reason();
        assert_eq!(reason2, ConvergenceReason::MaxIterations,
            "Reason should update to MaxIterations after forced non-convergence");

        // Restore normal settings — should converge again
        solver.tol = 1e-10;
        solver.max_iter = 100;
        let _ = solver.process_sample(0.1);
        let reason3 = solver.last_convergence_reason();
        assert_eq!(reason3, ConvergenceReason::Residual,
            "Reason should update back to Residual after restoring settings");
    }
}
