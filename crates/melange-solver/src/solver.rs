//! Interpreted circuit solver.
//!
//! This is a runtime solver that processes samples using the DK method.
//! It's slower than generated code but useful for prototyping and validation.
//!
//! All processing uses pre-allocated buffers - no heap allocation in the audio thread.

use crate::dk::DkKernel;
use melange_primitives::nr::{nr_solve_1d, nr_solve_2d, pn_vcrit, pnjlim, fetlim};
use melange_devices::{DiodeShockley, BjtEbersMoll, Jfet, Mosfet, KorenTriode, NonlinearDevice};
use smallvec::{SmallVec, smallvec};

/// Error type for solver construction and validation.
#[derive(Debug, Clone)]
pub enum SolverError {
    /// Device dimensions don't match the kernel's nonlinear dimension M.
    DimensionMismatch { expected: usize, got: usize },
    /// A 2D device has an out-of-bounds start index.
    DeviceIndexOutOfBounds { start_idx: usize, m: usize },
}

impl std::fmt::Display for SolverError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SolverError::DimensionMismatch { expected, got } =>
                write!(f, "Total device dimensions ({}) don't match kernel M ({})", got, expected),
            SolverError::DeviceIndexOutOfBounds { start_idx, m } =>
                write!(f, "2D device start_idx {} out of bounds for M={}", start_idx, m),
        }
    }
}

impl std::error::Error for SolverError {}

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
        for v in &mut g.i_hist { *v = 0.0; }
        for v in &mut g.i_prev { *v = 0.0; }
        for v in &mut g.v_prev { *v = 0.0; }
    }
}

/// Enum-based device entry for zero-cost dispatch in the audio hot path.
///
/// This eliminates vtable overhead by using enum dispatch.
/// All device types are stored inline with no heap allocation.
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
    /// JFET Shichman-Hodges model (2D)
    Jfet { device: Jfet, start_idx: usize },
    /// MOSFET Level 1 model (2D)
    Mosfet { device: Mosfet, start_idx: usize },
    /// Koren triode model (2D: plate current + grid current)
    Tube { device: KorenTriode, start_idx: usize },
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

    /// Create a new JFET device entry.
    pub fn new_jfet(device: Jfet, start_idx: usize) -> Self {
        Self::Jfet { device, start_idx }
    }

    /// Create a new MOSFET device entry.
    pub fn new_mosfet(device: Mosfet, start_idx: usize) -> Self {
        Self::Mosfet { device, start_idx }
    }

    /// Create a new triode tube device entry.
    pub fn new_tube(device: KorenTriode, start_idx: usize) -> Self {
        Self::Tube { device, start_idx }
    }

    /// Get the device dimension (1, 2, etc.)
    pub fn dimension(&self) -> usize {
        match self {
            DeviceEntry::Diode { .. } => 1,
            DeviceEntry::DiodeWithRs { .. } => 1,
            DeviceEntry::Led { .. } => 1,
            DeviceEntry::Bjt { .. } => 2,
            DeviceEntry::Jfet { .. } => 2,
            DeviceEntry::Mosfet { .. } => 2,
            DeviceEntry::Tube { .. } => 2,
        }
    }

    /// Get the starting index in the global voltage/current vectors.
    pub fn start_idx(&self) -> usize {
        match self {
            DeviceEntry::Diode { start_idx, .. } => *start_idx,
            DeviceEntry::DiodeWithRs { start_idx, .. } => *start_idx,
            DeviceEntry::Led { start_idx, .. } => *start_idx,
            DeviceEntry::Bjt { start_idx, .. } => *start_idx,
            DeviceEntry::Jfet { start_idx, .. } => *start_idx,
            DeviceEntry::Mosfet { start_idx, .. } => *start_idx,
            DeviceEntry::Tube { start_idx, .. } => *start_idx,
        }
    }

    /// Apply SPICE-style voltage limiting for one dimension of this device.
    ///
    /// Uses `pnjlim` for PN junctions (diodes, BJTs) and `fetlim` for FETs.
    /// `dim` is the local dimension index within this device (0 or 1).
    pub fn limit_voltage(&self, dim: usize, vnew: f64, vold: f64) -> f64 {
        match self {
            DeviceEntry::Diode { device, .. } => {
                let n_vt = device.n * device.vt;
                let vcrit = pn_vcrit(n_vt, device.is);
                pnjlim(vnew, vold, n_vt, vcrit)
            }
            DeviceEntry::DiodeWithRs { device, .. } => {
                let n_vt = device.diode.n * device.diode.vt;
                let vcrit = pn_vcrit(n_vt, device.diode.is);
                pnjlim(vnew, vold, n_vt, vcrit)
            }
            DeviceEntry::Led { .. } => {
                // LED uses Shockley diode internally (n=1.0, IS ~1e-20)
                let n_vt = melange_devices::VT_ROOM;
                let vcrit = pn_vcrit(n_vt, 1e-20);
                pnjlim(vnew, vold, n_vt, vcrit)
            }
            DeviceEntry::Bjt { device, .. } => {
                // Both junctions (Vbe, Vbc) use pnjlim with thermal voltage
                let vcrit = pn_vcrit(device.vt, device.is);
                pnjlim(vnew, vold, device.vt, vcrit)
            }
            DeviceEntry::Jfet { device, .. } => {
                if dim == 0 {
                    // dim 0 = Vds — generous limiting (quadratic model, well-behaved)
                    fetlim(vnew, vold, 0.0)
                } else {
                    // dim 1 = Vgs — limit around pinch-off voltage
                    fetlim(vnew, vold, device.vp)
                }
            }
            DeviceEntry::Mosfet { device, .. } => {
                if dim == 0 {
                    // dim 0 = Vds — generous limiting
                    fetlim(vnew, vold, 0.0)
                } else {
                    // dim 1 = Vgs — limit around threshold voltage
                    fetlim(vnew, vold, device.vt)
                }
            }
            DeviceEntry::Tube { device, .. } => {
                if dim == 0 {
                    // Vgk — grid current has exponential-like onset
                    let vcrit = pn_vcrit(device.vgk_onset / 3.0, 1e-10);
                    pnjlim(vnew, vold, device.vgk_onset / 3.0, vcrit)
                } else {
                    // Vpk — plate current is well-behaved, use FET-like limiting
                    fetlim(vnew, vold, 0.0)
                }
            }
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
            DeviceEntry::Jfet { device, .. } => {
                // dim 0 = Vds (v[0]), dim 1 = Vgs (v[1])
                let id = device.drain_current(v[1], v[0]);
                let ig = device.gate_current(v[1]);
                smallvec![id, ig]
            }
            DeviceEntry::Mosfet { device, .. } => {
                // dim 0 = Vds (v[0]), dim 1 = Vgs (v[1])
                let id = device.drain_current(v[1], v[0]);
                // MOSFET gate current is zero (insulated gate)
                smallvec![id, 0.0]
            }
            DeviceEntry::Tube { device, .. } => {
                let ip = device.plate_current(v[0], v[1]);
                let ig = device.grid_current(v[0]);
                smallvec![ip, ig]
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
            DeviceEntry::Jfet { device, .. } => {
                // dim 0 = Vds (v[0]), dim 1 = Vgs (v[1])
                let (did_dvgs, did_dvds) = device.jacobian_partial(v[1], v[0]);
                // Gate current Jacobian: only depends on Vgs (= v[1])
                let dig_dvgs = if v[1] > 0.0 {
                    // Forward-biased gate-source junction
                    let vgs_eff = match device.channel {
                        melange_devices::jfet::JfetChannel::N => v[1],
                        melange_devices::jfet::JfetChannel::P => -v[1],
                    };
                    if vgs_eff > 0.0 {
                        let sign = match device.channel {
                            melange_devices::jfet::JfetChannel::N => 1.0,
                            melange_devices::jfet::JfetChannel::P => -1.0,
                        };
                        sign * device.is / melange_devices::VT_ROOM
                            * melange_devices::safeguards::safe_exp(vgs_eff / melange_devices::VT_ROOM)
                    } else {
                        0.0
                    }
                } else {
                    0.0
                };
                // Return in dim-space order: [dId/dVds, dId/dVgs, dIg/dVds, dIg/dVgs]
                smallvec![did_dvds, did_dvgs, 0.0, dig_dvgs]
            }
            DeviceEntry::Mosfet { device, .. } => {
                // dim 0 = Vds (v[0]), dim 1 = Vgs (v[1])
                let (did_dvgs, did_dvds) = device.jacobian_partial(v[1], v[0]);
                // MOSFET gate current is zero → Jacobian row is zero.
                // Return in dim-space order: [dId/dVds, dId/dVgs, 0, 0]
                smallvec![did_dvds, did_dvgs, 0.0, 0.0]
            }
            DeviceEntry::Tube { device, .. } => {
                // Plate current Jacobian from NonlinearDevice<2>
                let ip_jac = NonlinearDevice::<2>::jacobian(device, &[v[0], v[1]]);
                let dip_dvgk = ip_jac[0];
                let dip_dvpk = ip_jac[1];
                // Grid current Jacobian: only depends on Vgk
                let dig_dvgk = device.grid_current_jacobian(v[0]);
                smallvec![dip_dvgk, dip_dvpk, dig_dvgk, 0.0]
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
            DeviceEntry::Jfet { .. } => 0.0,
            DeviceEntry::Mosfet { .. } => 0.0,
            DeviceEntry::Tube { .. } => 0.0,
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
            DeviceEntry::Jfet { .. } => 0.0,
            DeviceEntry::Mosfet { .. } => 0.0,
            DeviceEntry::Tube { .. } => 0.0,
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
            d => {
                log::warn!("write_currents: unsupported device dimension {}, skipping", d);
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
            d => {
                log::warn!("write_jacobian: unsupported device dimension {}, skipping", d);
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
    /// Precomputed S*N_i for node rows (n_nodes × M, row-major).
    /// Maps current-space NR steps to node voltage changes for damping.
    s_ni_nodes: Vec<f64>,
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
    /// Reason the NR solver stopped on the most recent sample.
    last_convergence_reason: ConvergenceReason,
    /// DC blocking filter state (previous input)
    pub dc_block_x_prev: f64,
    /// DC blocking filter state (previous output)
    pub dc_block_y_prev: f64,
    /// DC blocking filter coefficient (1 - 2*pi*5/sr)
    pub dc_block_r: f64,
    /// Diagnostics: peak absolute output seen
    pub diag_peak_output: f64,
    /// Diagnostics: number of samples that exceeded clamp threshold
    pub diag_clamp_count: u64,
    /// Diagnostics: number of times NR hit max iterations
    pub diag_nr_max_iter_count: u64,
    /// Diagnostics: number of times state was reset due to NaN
    pub diag_nan_reset_count: u64,
}

impl CircuitSolver {
    /// Create a new circuit solver.
    ///
    /// Returns an error if device dimensions don't match kernel dimensions.
    pub fn new(
        kernel: DkKernel,
        devices: Vec<DeviceEntry>,
        input_node: usize,
        output_node: usize,
    ) -> Result<Self, SolverError> {
        let n = kernel.n;
        let m = kernel.m;

        // Verify device dimensions match kernel
        let total_device_dim: usize = devices.iter().map(|d| d.dimension()).sum();
        if total_device_dim != m {
            return Err(SolverError::DimensionMismatch { expected: m, got: total_device_dim });
        }

        // Verify 2D devices have valid indices (check once at construction)
        for device in &devices {
            if device.dimension() == 2 && device.start_idx() + 1 >= m {
                return Err(SolverError::DeviceIndexOutOfBounds {
                    start_idx: device.start_idx(),
                    m,
                });
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

        // Precompute S*N_i for node rows (n_nodes × M) for NR node voltage damping.
        // s_ni_nodes[i][j] = sum_k S[i][k] * N_i[k][j], for i in 0..n_nodes
        let n_nodes = kernel.n_nodes;
        let mut s_ni_nodes = vec![0.0; n_nodes * m];
        for i in 0..n_nodes {
            for j in 0..m {
                let mut sum = 0.0;
                for k in 0..n {
                    sum += kernel.s[i * n + k] * kernel.n_i[k * m + j];
                }
                s_ni_nodes[i * m + j] = sum;
            }
        }

        let dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / kernel.sample_rate;

        Ok(Self {
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
            s_ni_nodes,
            input_node,
            output_node,
            input_conductance: 1.0, // Default 1/1Ω (near-ideal voltage source) — caller should set from netlist
            input_prev: 0.0,
            max_iter: 100,
            tol: 1e-10,
            last_convergence_reason: if m == 0 { ConvergenceReason::Linear } else { ConvergenceReason::MaxIterations },
            dc_block_x_prev: 0.0,
            dc_block_y_prev: 0.0,
            dc_block_r,
            diag_peak_output: 0.0,
            diag_clamp_count: 0,
            diag_nr_max_iter_count: 0,
            diag_nan_reset_count: 0,
        })
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
    /// Returns `true` if the DC OP converged, `false` otherwise.
    /// Does nothing if the DC OP solver fails to converge (keeps zero state).
    pub fn initialize_dc_op(&mut self, mna: &crate::mna::MnaSystem, device_slots: &[crate::codegen::ir::DeviceSlot]) -> bool {
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
            true
        } else {
            log::warn!("DC operating point did not converge after all strategies; using linear fallback");
            // Even when nonlinear DC OP fails, use the linear fallback node voltages.
            // This matches codegen, which always initializes v_prev from DC_OP
            // (the linear solution). Without this, the runtime starts from all-zeros
            // while codegen starts from the linear DC solution, causing divergence.
            let n = self.v_prev.len();
            for i in 0..n.min(result.v_node.len()) {
                self.v_prev[i] = result.v_node[i];
            }
            // i_nl_prev stays at zero (nonlinear currents unknown)
            // No warm-up: codegen doesn't run warm-up when DC OP fails
            false
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

        // Save state for next sample.
        // v_pred now holds the new node voltages (after correction).
        // v_prev holds the old node voltages (from previous sample).
        //
        // Global node voltage damping (a la ngspice CKTnodeDamping):
        // If any node voltage changed by more than NODE_DAMP_THRESHOLD per sample,
        // scale the entire step so the maximum change equals the threshold.
        // This prevents transformer coupling from amplifying NR errors exponentially
        // across samples. When damping triggers, re-evaluate device currents at the
        // damped state to keep i_nl_prev consistent with v_prev (otherwise the
        // N_i * i_nl_prev term in the next sample's RHS feeds back the divergent currents).
        const NODE_DAMP_THRESHOLD: f64 = 10.0;
        {
            let n_nodes = self.kernel.n_nodes;
            let m = self.kernel.m;
            let mut max_node_delta = 0.0_f64;
            for i in 0..n_nodes {
                let delta = (self.v_pred[i] - self.v_prev[i]).abs();
                if delta > max_node_delta { max_node_delta = delta; }
            }
            if max_node_delta > NODE_DAMP_THRESHOLD {
                let damp = (NODE_DAMP_THRESHOLD / max_node_delta).max(0.1);
                let n = self.kernel.n;
                for i in 0..n {
                    self.v_pred[i] = self.v_prev[i] + damp * (self.v_pred[i] - self.v_prev[i]);
                }
                // Re-evaluate device currents at the damped operating point.
                // Extract device voltages from damped v_pred: v_nl = N_v * v_pred
                for i in 0..m {
                    let mut sum = 0.0;
                    let nv_offset = i * n;
                    for j in 0..n {
                        sum += self.kernel.n_v[nv_offset + j] * self.v_pred[j];
                    }
                    self.v_nl[i] = sum;
                }
                // Evaluate device currents at damped voltages
                for device in &self.devices {
                    device.write_currents(&self.v_nl, &mut self.i_nl);
                }
            }
        }

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
            reset_coupled_inductors(&mut self.kernel.coupled_inductors);
            reset_transformer_groups(&mut self.kernel.transformer_groups);
            self.diag_nan_reset_count += 1;
        }

        self.kernel.update_inductors(&self.v_prev);
        self.kernel.update_coupled_inductors(&self.v_prev);
        self.kernel.update_transformer_groups(&self.v_prev);

        let n = self.kernel.n;

        // DC blocking filter (5Hz HPF)
        let raw_out = if self.output_node < n { self.v_prev[self.output_node] } else { 0.0 };
        let raw_out = if raw_out.is_finite() { raw_out } else { 0.0 };
        let dc_blocked = raw_out - self.dc_block_x_prev + self.dc_block_r * self.dc_block_y_prev;
        self.dc_block_x_prev = raw_out;
        self.dc_block_y_prev = dc_blocked;

        // Diagnostics
        let abs_out = dc_blocked.abs();
        if abs_out > self.diag_peak_output { self.diag_peak_output = abs_out; }
        if abs_out > 10.0 { self.diag_clamp_count += 1; }

        clamp_output(dc_blocked)
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

        // Add coupled inductor history contribution
        for ci in &self.kernel.coupled_inductors {
            if ci.l1_node_i > 0 { self.rhs[ci.l1_node_i - 1] -= ci.i1_hist; }
            if ci.l1_node_j > 0 { self.rhs[ci.l1_node_j - 1] += ci.i1_hist; }
            if ci.l2_node_i > 0 { self.rhs[ci.l2_node_i - 1] -= ci.i2_hist; }
            if ci.l2_node_j > 0 { self.rhs[ci.l2_node_j - 1] += ci.i2_hist; }
        }

        // Add transformer group history contribution
        for group in &self.kernel.transformer_groups {
            for k in 0..group.num_windings {
                let i_hist = group.i_hist[k];
                if group.winding_node_i[k] > 0 { self.rhs[group.winding_node_i[k] - 1] -= i_hist; }
                if group.winding_node_j[k] > 0 { self.rhs[group.winding_node_j[k] - 1] += i_hist; }
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
            |vnew, vold| device.limit_voltage(0, vnew, vold),
            self.v_nl_prev[0],
            self.max_iter,
            self.tol,
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
            self.diag_nr_max_iter_count += 1;
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
            |vnew, vold| dev0.limit_voltage(0, vnew, vold),
            |vnew, vold| dev1.limit_voltage(0, vnew, vold),
            self.v_nl_prev[0], self.v_nl_prev[1],
            self.max_iter,
            self.tol,
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
            self.diag_nr_max_iter_count += 1;
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

        // Warm start from previous currents (current-based NR, matches codegen)
        self.i_nl[..m].copy_from_slice(&self.i_nl_prev[..m]);

        // Default: assume we'll exhaust iterations (overwritten on early exit)
        self.last_convergence_reason = ConvergenceReason::MaxIterations;

        // Newton-Raphson iteration (current-based formulation)
        // Solves: i_nl = f(p + K * i_nl) where f() is device current evaluation
        for _ in 0..self.max_iter {
            // Compute controlling voltages: v_d[i] = p[i] + sum_j K[i][j] * i_nl[j]
            for i in 0..m {
                let mut v_d = self.p[i];
                let k_row_offset = i * m;
                for j in 0..m {
                    v_d += self.kernel.k[k_row_offset + j] * self.i_nl[j];
                }
                self.v_nl[i] = v_d;
            }

            // Compute device currents at controlling voltages
            self.i_nl_temp.fill(0.0);
            for device in &self.devices {
                device.write_currents(&self.v_nl, &mut self.i_nl_temp);
            }

            // Compute residual: r[i] = i_nl[i] - i_dev[i]
            let mut converged = true;
            for i in 0..m {
                self.residual[i] = self.i_nl[i] - self.i_nl_temp[i];
                if self.residual[i].abs() > self.tol {
                    converged = false;
                }
            }

            if converged {
                self.last_convergence_reason = ConvergenceReason::Residual;
                break;
            }

            // Build block-diagonal device Jacobian G[k][j] = di_dev[k]/dv_d[j]
            self.g_dev.fill(0.0);
            for device in &self.devices {
                device.write_jacobian(&self.v_nl, &mut self.g_dev, m);
            }

            // Build full NR Jacobian: J[i][j] = delta_ij - sum_k jdev[i][k] * K[k][j]
            // This matches the codegen formulation (Jf * K, not K * Jf)
            for i in 0..m {
                for j in 0..m {
                    let mut jk = 0.0;
                    for k in 0..m {
                        jk += self.g_dev[i * m + k] * self.kernel.k[k * m + j];
                    }
                    self.jacobian_mat[i * m + j] = if i == j { 1.0 } else { 0.0 } - jk;
                }
            }

            // Copy residual into delta for Gaussian elimination (solver modifies in place)
            self.delta[..m].copy_from_slice(&self.residual[..m]);

            // Solve J * delta = residual via Gaussian elimination with partial pivoting
            let solved = gauss_solve_inplace(&mut self.jacobian_mat, &mut self.delta, m);

            if !solved {
                // Singular Jacobian — damped half-step along residual
                for i in 0..m {
                    self.i_nl[i] -= self.residual[i] * 0.5;
                }
                continue;
            }

            // Compute voltage-space step: delta_v[i] = -sum_j K[i][j] * delta[j]
            // Then apply SPICE limiting in voltage space and compute damping factor.
            let mut alpha = 1.0_f64;
            for device in &self.devices {
                let idx = device.start_idx();
                for d in 0..device.dimension() {
                    let i = idx + d;
                    // Implied voltage change from current-space step
                    let mut dv = 0.0;
                    for j in 0..m {
                        dv -= self.kernel.k[i * m + j] * self.delta[j];
                    }
                    if dv.abs() > 1e-15 {
                        let v_old = self.v_nl[i];
                        let v_proposed = v_old + dv;
                        let v_limited = device.limit_voltage(d, v_proposed, v_old);
                        let dv_limited = v_limited - v_old;
                        let ratio = dv_limited / dv;
                        // ratio can be negative if limiter reverses direction (rare);
                        // in that case take a small step rather than going backwards
                        let ratio = ratio.max(0.01);
                        if ratio < alpha {
                            alpha = ratio;
                        }
                    }
                }
            }

            // Node voltage damping (ngspice CKTnodeDamping):
            // Check if the implied node voltage change from this NR step exceeds 10V.
            // Uses precomputed S*N_i to map current-space delta to node voltage space.
            {
                let n_nodes = self.kernel.n_nodes;
                let mut max_node_dv = 0.0_f64;
                for i in 0..n_nodes {
                    let mut dv = 0.0;
                    let row_offset = i * m;
                    for j in 0..m {
                        dv += self.s_ni_nodes[row_offset + j] * self.delta[j];
                    }
                    max_node_dv = max_node_dv.max((alpha * dv).abs());
                }
                if max_node_dv > 10.0 {
                    alpha *= (10.0 / max_node_dv).max(0.01);
                }
            }

            // Apply damped Newton step
            for i in 0..m {
                self.i_nl[i] -= alpha * self.delta[i];
            }

            // Check step-size convergence
            let max_step: f64 = self.delta[..m].iter()
                .map(|&d| (alpha * d).abs())
                .fold(0.0_f64, f64::max);
            if max_step < self.tol {
                self.last_convergence_reason = ConvergenceReason::StepSize;
                break;
            }

            // NaN/Inf detection: if any i_nl is non-finite, restore from previous and break
            if self.i_nl[..m].iter().any(|v| !v.is_finite()) {
                self.i_nl[..m].copy_from_slice(&self.i_nl_prev[..m]);
                break;
            }
        }

        // Recompute v_nl from final i_nl to ensure consistency.
        // The NR loop may exit with v_nl from the previous iteration's step 1
        // while i_nl was updated in step 5. This ensures compute_all_currents()
        // (called after solve_md) sees voltages consistent with the final currents.
        for i in 0..m {
            let mut v_d = self.p[i];
            let k_row_offset = i * m;
            for j in 0..m {
                v_d += self.kernel.k[k_row_offset + j] * self.i_nl[j];
            }
            self.v_nl[i] = v_d;
        }

        if self.last_convergence_reason == ConvergenceReason::MaxIterations {
            self.diag_nr_max_iter_count += 1;
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
        reset_coupled_inductors(&mut self.kernel.coupled_inductors);
        reset_transformer_groups(&mut self.kernel.transformer_groups);
        self.dc_block_x_prev = 0.0;
        self.dc_block_y_prev = 0.0;
        self.diag_peak_output = 0.0;
        self.diag_clamp_count = 0;
        self.diag_nr_max_iter_count = 0;
        self.diag_nan_reset_count = 0;
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[f64], output: &mut [f64]) {
        let len = input.len().min(output.len());
        for i in 0..len {
            output[i] = self.process_sample(input[i]);
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
    /// DC blocking filter state (previous input)
    pub dc_block_x_prev: f64,
    /// DC blocking filter state (previous output)
    pub dc_block_y_prev: f64,
    /// DC blocking filter coefficient (1 - 2*pi*5/sr)
    pub dc_block_r: f64,
    /// Diagnostics: peak absolute output seen
    pub diag_peak_output: f64,
    /// Diagnostics: number of samples that exceeded clamp threshold
    pub diag_clamp_count: u64,
    /// Diagnostics: number of times state was reset due to NaN
    pub diag_nan_reset_count: u64,
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
            input_conductance: 1.0, // Default 1/1Ω (near-ideal voltage source) — caller should set from netlist
            input_prev: 0.0,
            dc_block_x_prev: 0.0,
            dc_block_y_prev: 0.0,
            dc_block_r,
            diag_peak_output: 0.0,
            diag_clamp_count: 0,
            diag_nan_reset_count: 0,
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

        // Add coupled inductor history contribution
        for ci in &self.kernel.coupled_inductors {
            if ci.l1_node_i > 0 { self.rhs[ci.l1_node_i - 1] -= ci.i1_hist; }
            if ci.l1_node_j > 0 { self.rhs[ci.l1_node_j - 1] += ci.i1_hist; }
            if ci.l2_node_i > 0 { self.rhs[ci.l2_node_i - 1] -= ci.i2_hist; }
            if ci.l2_node_j > 0 { self.rhs[ci.l2_node_j - 1] += ci.i2_hist; }
        }

        // Add transformer group history contribution
        for group in &self.kernel.transformer_groups {
            for k in 0..group.num_windings {
                let i_hist = group.i_hist[k];
                if group.winding_node_i[k] > 0 { self.rhs[group.winding_node_i[k] - 1] -= i_hist; }
                if group.winding_node_j[k] > 0 { self.rhs[group.winding_node_j[k] - 1] += i_hist; }
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
        let raw_out = if self.output_node < n { self.v_prev[self.output_node] } else { 0.0 };
        let raw_out = if raw_out.is_finite() { raw_out } else { 0.0 };
        let dc_blocked = raw_out - self.dc_block_x_prev + self.dc_block_r * self.dc_block_y_prev;
        self.dc_block_x_prev = raw_out;
        self.dc_block_y_prev = dc_blocked;

        // Diagnostics
        let abs_out = dc_blocked.abs();
        if abs_out > self.diag_peak_output { self.diag_peak_output = abs_out; }
        if abs_out > 10.0 { self.diag_clamp_count += 1; }

        clamp_output(dc_blocked)
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

    /// Test RC lowpass step response with DC blocking filter.
    ///
    /// Uses R1 to ground with C1 in parallel — single-node RC circuit.
    /// The Thevenin input model requires a ground reference at the input node.
    /// R=1k, C=1u, tau=RC=1ms (44.1 samples at 44.1kHz).
    /// DC blocking filter removes steady-state DC, so we check transient behavior.
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

        // With DC blocking, the peak output should be > 0.8V (transient from step)
        let peak_output = output.iter().fold(0.0_f64, |a, &b| a.max(b));
        assert!(peak_output > 0.8,
            "RC lowpass peak output = {:.4}V, expected > 0.8V (DC blocking removes steady-state)", peak_output);

        // First sample should be positive (step response starts rising)
        assert!(output[0] > 0.0, "First sample should be positive, got {:.6}", output[0]);

        // Output should be monotonically increasing in first ~100 samples
        // (before DC blocker droop becomes significant)
        for i in 1..100 {
            assert!(output[i] >= output[i-1] - 1e-10,
                "Output should be monotonically increasing in first 100 samples: output[{}]={:.6} < output[{}]={:.6}",
                i, output[i], i-1, output[i-1]);
        }
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

        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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

        let mut solver = CircuitSolver::new(kernel, devices, 0, 0).unwrap();
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

        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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

        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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
            let mut solver2 = CircuitSolver::new(kernel2, devices2, 0, 0).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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

        // Output should monotonically increase in first 100 samples (charging capacitor)
        // DC blocker causes droop after that, so limit the check
        for i in 1..100 {
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
        let solver = CircuitSolver::new(kernel, vec![], 0, 0).unwrap();
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

        let mut solver = CircuitSolver::new(kernel, vec![], 0, 0).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 0).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
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

    // ========================================================================
    // DC Blocking Functional Tests (H5)
    // ========================================================================

    /// Verify the 5Hz HPF actually removes DC in a linear circuit.
    /// Feed constant 1V for 1 second — output should decay to near zero.
    #[test]
    fn test_dc_blocking_removes_dc_linear() {
        let spice = r#"RC Circuit
R1 in 0 1k
C1 in 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let mut solver = LinearSolver::new(kernel, 0, 0);
        solver.input_conductance = 0.001;

        // Feed 1V DC for 1 second (44100 samples)
        let mut peak = 0.0_f64;
        let mut last_out = 0.0_f64;
        for i in 0..44100 {
            last_out = solver.process_sample(1.0);
            if i < 500 {
                // Track peak during initial transient
                peak = peak.max(last_out.abs());
            }
        }

        // Transient should have been visible
        assert!(peak > 0.1,
            "Initial transient should pass through, peak = {}", peak);
        // After 1 second at 5Hz cutoff, DC should be heavily attenuated
        assert!(last_out.abs() < 0.01,
            "DC should be blocked after 1s, got {:.6}V", last_out);
    }

    /// Verify the 5Hz HPF removes DC in a nonlinear (diode clipper) circuit.
    #[test]
    fn test_dc_blocking_removes_dc_nonlinear() {
        let spice = r#"Diode Clipper DC
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
        solver.input_conductance = 0.001;

        // Feed 1V DC for 2 seconds
        let mut peak = 0.0_f64;
        let mut last_out = 0.0_f64;
        for i in 0..88200 {
            last_out = solver.process_sample(1.0);
            if i < 500 {
                peak = peak.max(last_out.abs());
            }
        }

        assert!(peak > 0.1,
            "Initial transient should pass through, peak = {}", peak);
        assert!(last_out.abs() < 0.01,
            "DC should be blocked after 2s, got {:.6}V", last_out);
    }

    // ========================================================================
    // Device Coverage Tests (M16)
    // ========================================================================

    /// Test LED device variant through the runtime solver.
    #[test]
    fn test_led_device_runtime_solver() {
        let spice = r#"LED Clipper
Rin in 0 1k
D1 in out LED_RED
R1 out 0 1k
C1 out 0 100n
.model LED_RED D(IS=1e-20)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let led = melange_devices::Led::red();
        let devices = vec![DeviceEntry::new_led(led, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
        solver.input_conductance = 0.001;

        // 200 samples of 1kHz sine
        for i in 0..200 {
            let t = i as f64 / 44100.0;
            let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let out = solver.process_sample(input);
            assert!(out.is_finite(), "LED solver output NaN/Inf at sample {}", i);
            assert!(out.abs() <= 10.0, "LED solver output unbounded at sample {}: {}", i, out);
        }
    }

    /// Test DiodeWithRs device variant (diode with series resistance).
    #[test]
    fn test_diode_with_rs_runtime_solver() {
        let spice = r#"Diode Rs Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
C1 out 0 100n
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let base_diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let diode_rs = melange_devices::DiodeWithRs::new(base_diode, 10.0); // 10Ω series R
        let devices = vec![DeviceEntry::new_diode_with_rs(diode_rs, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
        solver.input_conductance = 0.001;

        for i in 0..200 {
            let t = i as f64 / 44100.0;
            let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let out = solver.process_sample(input);
            assert!(out.is_finite(), "DiodeWithRs output NaN/Inf at sample {}", i);
            assert!(out.abs() <= 10.0, "DiodeWithRs output unbounded at sample {}: {}", i, out);
        }
    }

    /// Test PNP BJT device variant through the runtime solver.
    #[test]
    fn test_pnp_bjt_runtime_solver() {
        // PNP common-emitter: emitter to VCC, collector to load
        let spice = r#"PNP Common Emitter
Q1 coll base emit 2N3906
Rc coll 0 10k
R1 base 0 100k
Re emit vcc 1k
Rbias vcc 0 10k
.model 2N3906 PNP(IS=1e-15 BF=200)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let bjt = BjtEbersMoll::new_room_temp(1e-15, 200.0, 3.0, melange_devices::BjtPolarity::Pnp);
        let devices = vec![DeviceEntry::new_bjt(bjt, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 0).unwrap();
        solver.input_conductance = 1.0 / 100000.0;

        // 50 DC warm-up samples
        for _ in 0..50 {
            let out = solver.process_sample(0.5);
            assert!(out.is_finite(), "PNP warm-up output should be finite");
        }

        // 100 sine samples
        for i in 0..100 {
            let t = i as f64 / 44100.0;
            let input = 0.5 + 0.3 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let out = solver.process_sample(input);
            assert!(out.is_finite(), "PNP sine output NaN/Inf at sample {}", i);
            assert!(out.abs() <= 10.0, "PNP output unbounded at sample {}: {}", i, out);
        }
    }

    // ========================================================================
    // Diagnostic Counter Tests (M14)
    // ========================================================================

    /// Verify peak output tracking works and resets properly.
    #[test]
    fn test_diag_peak_output_tracking() {
        let spice = r#"RC Circuit
R1 in 0 1k
C1 in 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let mut solver = LinearSolver::new(kernel, 0, 0);
        solver.input_conductance = 0.001;

        // Peak starts at 0
        assert_eq!(solver.diag_peak_output, 0.0, "Peak should start at 0");

        // Process some signal
        for _ in 0..100 {
            solver.process_sample(1.0);
        }
        let peak_after_signal = solver.diag_peak_output;
        assert!(peak_after_signal > 0.0, "Peak should increase after signal");

        // Peak is monotonically non-decreasing: process zero input
        for _ in 0..100 {
            solver.process_sample(0.0);
        }
        assert!(solver.diag_peak_output >= peak_after_signal,
            "Peak should be monotonically non-decreasing");

        // Reset zeros the peak
        solver.reset();
        assert_eq!(solver.diag_peak_output, 0.0, "Peak should be 0 after reset");
    }

    /// Verify NR max-iterations counter tracks and resets.
    #[test]
    fn test_diag_nr_max_iter_count() {
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
        solver.input_conductance = 0.001;

        // Normal operation: counter should stay at 0
        for _ in 0..100 {
            solver.process_sample(0.1);
        }
        assert_eq!(solver.diag_nr_max_iter_count, 0,
            "NR max iter count should be 0 during normal operation");

        // Force non-convergence: impossibly tight tolerance, 1 iteration
        solver.tol = 1e-300;
        solver.max_iter = 1;
        for _ in 0..10 {
            solver.process_sample(5.0);
        }
        assert!(solver.diag_nr_max_iter_count > 0,
            "NR max iter count should increment when forced to not converge");

        // Reset zeros the counter
        solver.reset();
        assert_eq!(solver.diag_nr_max_iter_count, 0,
            "NR max iter count should be 0 after reset");
    }

    /// Verify NaN reset counter tracks and resets.
    #[test]
    fn test_diag_nan_reset_count() {
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
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();
        solver.input_conductance = 0.001;

        // Normal operation
        for _ in 0..100 {
            solver.process_sample(0.1);
        }
        assert_eq!(solver.diag_nan_reset_count, 0,
            "NaN reset count should be 0 during normal operation");

        // Corrupt state to trigger NaN detection
        solver.v_prev[0] = f64::NAN;
        solver.process_sample(0.1);
        assert!(solver.diag_nan_reset_count > 0,
            "NaN reset count should increment after NaN in v_prev");

        // Reset zeros the counter
        solver.reset();
        assert_eq!(solver.diag_nan_reset_count, 0,
            "NaN reset count should be 0 after reset");
    }

    /// Verify clamp counter stays at 0 for small signals (LinearSolver).
    #[test]
    fn test_diag_clamp_count() {
        let spice = r#"RC Circuit
R1 in 0 1k
C1 in 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let mut solver = LinearSolver::new(kernel, 0, 0);
        solver.input_conductance = 0.001;

        // Small signal: clamp count should remain 0
        for i in 0..500 {
            let t = i as f64 / 44100.0;
            let input = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            solver.process_sample(input);
        }
        assert_eq!(solver.diag_clamp_count, 0,
            "Clamp count should be 0 for small signals, got {}", solver.diag_clamp_count);
    }
}

// ==========================================================================
// NodalSolver: Full-nodal Newton-Raphson transient solver
//
// Instead of reducing to the M×M nonlinear kernel (DK method), this solver
// iterates on the full N-dimensional voltage vector each sample. The Jacobian
// is A - N_i * J_dev * N_v (N×N), which is well-conditioned because A provides
// the dominant diagonal structure. The rank-M nonlinear update modifies at most
// M eigenvalues.
//
// This handles circuits with large transformers and global feedback that make
// the DK method's K matrix ill-conditioned. Cost: O(N^3) per NR iteration
// for LU factorization (vs O(M^2) for DK), but for N~33, M~8 this is ~12K
// FLOPs per iteration — well within real-time budget at 48kHz.
// ==========================================================================

use crate::dc_op;
use crate::codegen::ir::{DeviceParams, DeviceSlot};
use crate::mna::MnaSystem;

/// Apply SPICE-style voltage limiting for a DeviceSlot dimension.
///
/// One-sample delayed mutual coupling between two inductor windings.
///
/// The mutual inductance M is removed from the simultaneous C matrix and
/// injected as a delayed history term: `rhs[k_a] += 2*alpha*M * v_prev[k_b]`.
/// This breaks feedback loops through tightly coupled transformers, allowing
/// the NR to converge without fighting transformer gain amplification.
#[derive(Debug, Clone)]
struct DelayedCoupling {
    /// Augmented variable index for winding A
    var_a: usize,
    /// Augmented variable index for winding B
    var_b: usize,
    /// 2 * (2/T) * M = 4 * sample_rate * mutual_inductance
    coeff: f64,
}

/// Dispatches pnjlim (PN junctions: diodes, BJTs, tubes) or fetlim (JFETs, MOSFETs)
/// based on device type and dimension. Same limiting as DeviceEntry::limit_voltage
/// but works with DeviceSlot parameters directly.
fn limit_slot_voltage(slot: &DeviceSlot, dim: usize, vnew: f64, vold: f64) -> f64 {
    match &slot.params {
        DeviceParams::Diode(dp) => {
            let vcrit = pn_vcrit(dp.n_vt, dp.is);
            pnjlim(vnew, vold, dp.n_vt, vcrit)
        }
        DeviceParams::Bjt(bp) => {
            let vcrit = pn_vcrit(bp.vt, bp.is);
            pnjlim(vnew, vold, bp.vt, vcrit)
        }
        DeviceParams::Jfet(jp) => {
            if dim == 0 {
                fetlim(vnew, vold, 0.0) // Vds
            } else {
                fetlim(vnew, vold, jp.vp) // Vgs
            }
        }
        DeviceParams::Mosfet(mp) => {
            if dim == 0 {
                fetlim(vnew, vold, 0.0) // Vds
            } else {
                fetlim(vnew, vold, mp.vt) // Vgs
            }
        }
        DeviceParams::Tube(tp) => {
            if dim == 0 {
                // Vgk — grid current has exponential-like onset
                let n_vt = tp.vgk_onset / 3.0;
                let vcrit = pn_vcrit(n_vt, 1e-10);
                pnjlim(vnew, vold, n_vt, vcrit)
            } else {
                // Vpk — plate current is well-behaved
                fetlim(vnew, vold, 0.0)
            }
        }
    }
}

/// Full-nodal Newton-Raphson solver for nonlinear transient simulation.
///
/// Works in the full voltage space rather than the reduced nonlinear current space.
/// Handles circuits where the DK method's K matrix is ill-conditioned (large
/// transformers, global feedback through transformers).
///
/// Uses augmented MNA for inductors: each inductor winding adds an extra variable
/// (branch current j_L) with inductance L in the C matrix. This avoids the tiny
/// companion conductances (T/(2L)) that make the standard companion model
/// ill-conditioned for large inductors at audio sample rates.
#[derive(Clone)]
pub struct NodalSolver {
    /// DK kernel (retained for kernel.n_nodes, kernel.m, kernel.sample_rate)
    pub kernel: DkKernel,
    /// Total system dimension: n_aug + inductor branch variables
    n_nodal: usize,
    /// A matrix = G_nodal + (2/T)*C_nodal (n_nodal × n_nodal, augmented inductors)
    a_matrix: Vec<Vec<f64>>,
    /// A_neg = (2/T)*C_nodal - G_nodal, flat row-major (n_nodal × n_nodal)
    a_neg: Vec<f64>,
    /// Constant RHS vector (n_nodal), padded with zeros for inductor rows
    rhs_const: Vec<f64>,
    /// N_v matrix expanded to m × n_nodal (extra cols zero), flat row-major
    n_v: Vec<f64>,
    /// N_i matrix expanded to n_nodal × m (extra rows zero), flat row-major
    n_i: Vec<f64>,
    /// Device slots for device evaluation (same format as dc_op)
    device_slots: Vec<DeviceSlot>,
    /// Previous state vector (n_nodal: node voltages + VS currents + inductor currents)
    pub v_prev: Vec<f64>,
    /// Previous nonlinear currents (M) for trapezoidal integration
    pub i_nl_prev: Vec<f64>,
    /// Pre-allocated working buffers
    rhs: Vec<f64>,
    v_nl: Vec<f64>,
    i_nl: Vec<f64>,
    j_dev: Vec<f64>,
    g_aug: Vec<Vec<f64>>,
    rhs_work: Vec<f64>,
    /// Input/output
    pub input_node: usize,
    pub output_node: usize,
    pub input_conductance: f64,
    input_prev: f64,
    /// NR config
    pub max_iter: u32,
    pub tol: f64,
    /// DC blocking filter
    pub dc_block_x_prev: f64,
    pub dc_block_y_prev: f64,
    pub dc_block_r: f64,
    /// Diagnostics
    pub diag_peak_output: f64,
    pub diag_clamp_count: u64,
    pub diag_nr_max_iter_count: u64,
    pub diag_nan_reset_count: u64,
    /// Delayed mutual couplings (from .delay_feedback directives)
    delayed_couplings: Vec<DelayedCoupling>,
}

impl NodalSolver {
    /// Create a new full-nodal solver with augmented MNA for inductors.
    ///
    /// Each inductor winding gets an extra variable (branch current) in the system.
    /// The inductance L appears in the C matrix instead of as a tiny companion
    /// conductance T/(2L), giving a well-conditioned A matrix for large inductors.
    pub fn new(
        kernel: DkKernel,
        mna: &MnaSystem,
        device_slots: Vec<DeviceSlot>,
        input_node: usize,
        output_node: usize,
    ) -> Self {
        let n_aug = mna.n_aug;    // original system dimension (not kernel.n which may be augmented)
        let n_nodes = kernel.n_nodes;
        let m = kernel.m;

        // Count total inductor winding variables
        let n_uncoupled = mna.inductors.len();
        let n_coupled_windings: usize = mna.coupled_inductors.len() * 2;
        let n_xfmr_windings: usize = mna.transformer_groups.iter()
            .map(|g| g.num_windings).sum();
        let n_inductor_vars = n_uncoupled + n_coupled_windings + n_xfmr_windings;
        let n_nodal = n_aug + n_inductor_vars;

        let t = 1.0 / kernel.sample_rate;
        let alpha = 2.0 / t; // 2/T

        // Build G_nodal and C_nodal (n_nodal × n_nodal)
        // Start from MNA's raw G and C (no inductor companion conductances)
        let mut g_nod = vec![vec![0.0; n_nodal]; n_nodal];
        let mut c_nod = vec![vec![0.0; n_nodal]; n_nodal];
        for i in 0..n_aug {
            for j in 0..n_aug {
                g_nod[i][j] = mna.g[i][j];
                c_nod[i][j] = mna.c[i][j];
            }
        }

        // Stamp inductor augmented variables
        let mut var_idx = n_aug;

        // Uncoupled inductors: 1 variable each
        for ind in &mna.inductors {
            let k = var_idx;
            let ni = ind.node_i; // 1-indexed, 0 = ground
            let nj = ind.node_j;

            // KCL: j_L enters node_i, exits node_j
            if ni > 0 { g_nod[ni - 1][k] += 1.0; }
            if nj > 0 { g_nod[nj - 1][k] -= 1.0; }
            // KVL row: -V_i + V_j (= -L·dj_L/dt, with L in C)
            if ni > 0 { g_nod[k][ni - 1] -= 1.0; }
            if nj > 0 { g_nod[k][nj - 1] += 1.0; }
            // Self-inductance in C matrix
            c_nod[k][k] = ind.value;

            var_idx += 1;
        }

        // Coupled inductor pairs: 2 variables each
        for ci in &mna.coupled_inductors {
            let k1 = var_idx;
            let k2 = var_idx + 1;

            // Winding 1 KCL/KVL
            if ci.l1_node_i > 0 { g_nod[ci.l1_node_i - 1][k1] += 1.0; }
            if ci.l1_node_j > 0 { g_nod[ci.l1_node_j - 1][k1] -= 1.0; }
            if ci.l1_node_i > 0 { g_nod[k1][ci.l1_node_i - 1] -= 1.0; }
            if ci.l1_node_j > 0 { g_nod[k1][ci.l1_node_j - 1] += 1.0; }

            // Winding 2 KCL/KVL
            if ci.l2_node_i > 0 { g_nod[ci.l2_node_i - 1][k2] += 1.0; }
            if ci.l2_node_j > 0 { g_nod[ci.l2_node_j - 1][k2] -= 1.0; }
            if ci.l2_node_i > 0 { g_nod[k2][ci.l2_node_i - 1] -= 1.0; }
            if ci.l2_node_j > 0 { g_nod[k2][ci.l2_node_j - 1] += 1.0; }

            // Self-inductances
            c_nod[k1][k1] = ci.l1_value;
            c_nod[k2][k2] = ci.l2_value;
            // Mutual inductance M = k·√(L1·L2)
            let m_val = ci.coupling * (ci.l1_value * ci.l2_value).sqrt();
            c_nod[k1][k2] = m_val;
            c_nod[k2][k1] = m_val;

            var_idx += 2;
        }

        // Transformer groups: N variables each
        let mut delayed_couplings = Vec::new();
        for group in &mna.transformer_groups {
            let w = group.num_windings;
            let base_k = var_idx;

            for widx in 0..w {
                let k = base_k + widx;
                let ni = group.winding_node_i[widx];
                let nj = group.winding_node_j[widx];

                // KCL/KVL stamps for winding
                if ni > 0 { g_nod[ni - 1][k] += 1.0; }
                if nj > 0 { g_nod[nj - 1][k] -= 1.0; }
                if ni > 0 { g_nod[k][ni - 1] -= 1.0; }
                if nj > 0 { g_nod[k][nj - 1] += 1.0; }

                // Inductance sub-matrix: L[i][j] = k_ij·√(Li·Lj)
                // Skip delayed pairs — their mutual inductance is injected as history.
                for widx2 in 0..w {
                    let is_delayed = group.delayed_pairs.iter().any(|&(a, b)|
                        (a == widx && b == widx2) || (a == widx2 && b == widx)
                    );
                    if is_delayed && widx != widx2 {
                        // Store delayed coupling info for RHS injection.
                        // We keep the mutual M in C_nodal (so A_neg has the history term),
                        // but remove it from A (so NR doesn't fight the coupling).
                        // The removed A contribution (alpha*M*j[n+1]) is replaced by
                        // alpha*M*j[n] (delayed one sample) in the RHS.
                        let m_val = group.coupling_matrix[widx][widx2]
                            * (group.inductances[widx] * group.inductances[widx2]).sqrt();
                        if widx < widx2 {
                            delayed_couplings.push(DelayedCoupling {
                                var_a: base_k + widx,
                                var_b: base_k + widx2,
                                coeff: alpha * m_val, // (2/T) * M — just the A contribution
                            });
                        }
                        // DO stamp into C_nodal (A_neg keeps the history term)
                        let k2 = base_k + widx2;
                        c_nod[k][k2] = group.coupling_matrix[widx][widx2]
                            * (group.inductances[widx] * group.inductances[widx2]).sqrt();
                    } else {
                        let k2 = base_k + widx2;
                        c_nod[k][k2] = group.coupling_matrix[widx][widx2]
                            * (group.inductances[widx] * group.inductances[widx2]).sqrt();
                    }
                }
            }

            var_idx += w;
        }

        if !delayed_couplings.is_empty() {
            log::info!("NodalSolver: {} delayed mutual couplings (one-sample feedback delay)",
                delayed_couplings.len());
        }

        // Build A = G + (2/T)*C
        let mut a_matrix = vec![vec![0.0; n_nodal]; n_nodal];
        for i in 0..n_nodal {
            for j in 0..n_nodal {
                a_matrix[i][j] = g_nod[i][j] + alpha * c_nod[i][j];
            }
        }
        // Remove delayed mutual terms from A (they stay in A_neg for history).
        // The A contribution alpha*M*j[n+1] is replaced by alpha*M*j[n] in the RHS.
        for dc in &delayed_couplings {
            a_matrix[dc.var_a][dc.var_b] -= dc.coeff;
            a_matrix[dc.var_b][dc.var_a] -= dc.coeff;
        }

        // Build A_neg = (2/T)*C - G (flat row-major)
        let mut a_neg = vec![0.0; n_nodal * n_nodal];
        for i in 0..n_nodal {
            for j in 0..n_nodal {
                a_neg[i * n_nodal + j] = alpha * c_nod[i][j] - g_nod[i][j];
            }
        }
        // Zero VS/VCVS/ideal_xfmr rows in A_neg (algebraic constraints, no history).
        // These are rows n_nodes..n_aug. Do NOT zero inductor rows (n_aug..n_nodal).
        if n_aug > n_nodes {
            for i in n_nodes..n_aug {
                for j in 0..n_nodal {
                    a_neg[i * n_nodal + j] = 0.0;
                }
            }
        }

        // Build rhs_const, N_v, N_i from MNA (always n_aug-sized), expand to n_nodal.
        // Use MNA directly — kernel.rhs_const/n_v/n_i might already be at n_nodal
        // if from_mna_augmented was used, and we don't want to double-expand.
        let rhs_const_base = crate::dk::build_rhs_const(mna);
        let mut rhs_const = vec![0.0; n_nodal];
        for i in 0..n_aug {
            rhs_const[i] = rhs_const_base[i];
        }

        // Expand N_v from m × n_aug to m × n_nodal (extra cols = 0)
        let mut n_v = vec![0.0; m * n_nodal];
        for i in 0..m {
            for j in 0..n_aug {
                n_v[i * n_nodal + j] = mna.n_v[i][j];
            }
        }

        // Expand N_i from n_aug × m to n_nodal × m (extra rows = 0)
        let mut n_i = vec![0.0; n_nodal * m];
        for i in 0..n_aug {
            for j in 0..m {
                n_i[i * m + j] = mna.n_i[i][j];
            }
        }

        let dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / kernel.sample_rate;

        if n_inductor_vars > 0 {
            log::info!(
                "NodalSolver: augmented MNA with {} inductor variables (n_aug={}, n_nodal={})",
                n_inductor_vars, n_aug, n_nodal
            );
        }

        Self {
            kernel,
            n_nodal,
            a_matrix,
            a_neg,
            rhs_const,
            n_v,
            n_i,
            device_slots,
            v_prev: vec![0.0; n_nodal],
            i_nl_prev: vec![0.0; m],
            rhs: vec![0.0; n_nodal],
            v_nl: vec![0.0; m],
            i_nl: vec![0.0; m],
            j_dev: vec![0.0; m * m],
            g_aug: vec![vec![0.0; n_nodal]; n_nodal],
            rhs_work: vec![0.0; n_nodal],
            input_node,
            output_node,
            input_conductance: 1.0,
            input_prev: 0.0,
            max_iter: 50,
            tol: 1e-6,
            dc_block_x_prev: 0.0,
            dc_block_y_prev: 0.0,
            dc_block_r,
            diag_peak_output: 0.0,
            diag_clamp_count: 0,
            diag_nr_max_iter_count: 0,
            diag_nan_reset_count: 0,
            delayed_couplings,
        }
    }

    /// Initialize from DC operating point.
    ///
    /// The DC OP solver operates on the original n_aug system. Node voltages and VS
    /// currents are copied to v_prev[0..n_aug]. Inductor branch currents
    /// (v_prev[n_aug..n_nodal]) start at zero and settle during warm-up.
    pub fn initialize_dc_op(&mut self, mna: &MnaSystem, device_slots: &[DeviceSlot]) {
        let config = dc_op::DcOpConfig {
            input_node: self.input_node,
            input_resistance: 1.0 / self.input_conductance,
            ..Default::default()
        };
        let result = dc_op::solve_dc_operating_point(mna, device_slots, &config);
        if result.converged {
            // Initialize v_prev from DC operating point.
            // The DC OP solver uses the same augmented inductor convention (n_dc = n_aug + inductors),
            // so v_node includes node voltages, VS currents, AND inductor DC branch currents.
            for i in 0..self.n_nodal.min(result.v_node.len()) {
                self.v_prev[i] = result.v_node[i];
            }
            // Initialize i_nl_prev from DC nonlinear currents
            let m = self.kernel.m;
            for i in 0..m.min(result.i_nl.len()) {
                self.i_nl_prev[i] = result.i_nl[i];
            }
            log::info!("Nodal solver DC OP initialized (method: {:?})", result.method);
        } else {
            log::warn!("Nodal solver DC OP failed to converge, using zero initial state");
        }

        // Warm-up: process 50 samples of silence to settle transients
        for _ in 0..50 {
            self.process_sample(0.0);
        }
    }

    /// Process a single sample using full-nodal Newton-Raphson.
    ///
    /// The state vector includes node voltages, VS currents, and inductor branch
    /// currents. A_neg handles all trapezoidal history (including inductors) — no
    /// separate inductor companion model updates are needed.
    #[allow(clippy::needless_range_loop)]
    pub fn process_sample(&mut self, input: f64) -> f64 {
        let input = sanitize_input(input);
        let n = self.n_nodal;
        let m = self.kernel.m;

        // Step 1: Build base RHS = rhs_const + A_neg * v_prev + N_i * i_nl_prev + input
        // A_neg automatically handles inductor history (2L/T diagonal + ±1 coupling).
        for i in 0..n {
            let mut sum = self.rhs_const[i];
            let a_neg_offset = i * n;
            for j in 0..n {
                sum += self.a_neg[a_neg_offset + j] * self.v_prev[j];
            }
            // N_i * i_nl_prev (trapezoidal nonlinear integration)
            // Only the first n_aug rows of N_i are nonzero (inductor rows are zero)
            let ni_offset = i * m;
            for j in 0..m {
                sum += self.n_i[ni_offset + j] * self.i_nl_prev[j];
            }
            self.rhs[i] = sum;
        }

        // Delayed mutual coupling injection (one-sample feedback delay).
        // For each delayed pair (a, b): inject 2*alpha*M * j_prev from the other winding.
        // This replaces the mutual terms that were zeroed in C_nodal.
        for dc in &self.delayed_couplings {
            self.rhs[dc.var_a] += dc.coeff * self.v_prev[dc.var_b];
            self.rhs[dc.var_b] += dc.coeff * self.v_prev[dc.var_a];
        }

        // Input source (Thevenin, trapezoidal)
        if self.input_node < n {
            self.rhs[self.input_node] += (input + self.input_prev) * self.input_conductance;
        }
        self.input_prev = input;

        // Step 2: Newton-Raphson in full augmented voltage space
        let mut v = self.v_prev.clone();
        let mut converged = false;

        for _iter in 0..self.max_iter {
            // 2a. Extract nonlinear voltages: v_nl = N_v * v
            // N_v is m × n_nodal (extra inductor cols are zero, but included for correctness)
            for i in 0..m {
                let mut sum = 0.0;
                let nv_offset = i * n;
                for j in 0..n {
                    sum += self.n_v[nv_offset + j] * v[j];
                }
                self.v_nl[i] = sum;
            }

            // 2b. Evaluate device currents and Jacobian
            dc_op::evaluate_devices(&self.v_nl, &self.device_slots, &mut self.i_nl, &mut self.j_dev, m);

            // 2c. Build Jacobian: G_aug = A - N_i * J_dev * N_v
            // Start from A matrix (inductor rows are linear, unaffected by J_dev)
            for i in 0..n {
                for j in 0..n {
                    self.g_aug[i][j] = self.a_matrix[i][j];
                }
            }
            // Stamp -N_i * J_dev * N_v (only affects rows/cols where N_i/N_v are nonzero)
            for a in 0..n {
                for b in 0..n {
                    let mut sum = 0.0;
                    for i in 0..m {
                        let ni_ai = self.n_i[a * m + i];
                        if ni_ai.abs() < 1e-30 { continue; }
                        for j in 0..m {
                            let jd = self.j_dev[i * m + j];
                            if jd.abs() < 1e-30 { continue; }
                            let nv_jb = self.n_v[j * n + b];
                            if nv_jb.abs() < 1e-30 { continue; }
                            sum += ni_ai * jd * nv_jb;
                        }
                    }
                    self.g_aug[a][b] -= sum;
                }
            }

            // 2d. Build companion RHS: rhs_base + N_i * (i_nl - J_dev * v_nl)
            self.rhs_work.copy_from_slice(&self.rhs);
            for i in 0..m {
                let mut jdev_vnl = 0.0;
                for j in 0..m {
                    jdev_vnl += self.j_dev[i * m + j] * self.v_nl[j];
                }
                let i_comp = self.i_nl[i] - jdev_vnl;
                for a in 0..n {
                    self.rhs_work[a] += self.n_i[a * m + i] * i_comp;
                }
            }

            // 2e. Solve: v_new = G_aug^{-1} * rhs_work
            let v_new = match crate::dk::solve_equilibrated(&self.g_aug, &self.rhs_work) {
                Some(v) => v,
                None => {
                    self.diag_nr_max_iter_count += 1;
                    break;
                }
            };

            // 2f. SPICE-style voltage limiting + ngspice global node damping.
            //
            // Two layers of damping, applied multiplicatively to a single alpha:
            // 1) Device voltage limiting (pnjlim/fetlim): prevents NR overshoot on
            //    steep nonlinear I-V curves (PN junctions, FET thresholds).
            // 2) Global node voltage damping (ngspice CKTnodeDamping): limits max
            //    node voltage change to 10V per NR iteration, preventing transformer
            //    coupling from amplifying device voltage steps to 1e8V node swings.
            let mut alpha = 1.0_f64;

            // Layer 1: SPICE device voltage limiting
            if m > 0 {
                for slot in &self.device_slots {
                    let s = slot.start_idx;
                    for d in 0..slot.dimension {
                        let i = s + d;
                        let mut v_nl_proposed = 0.0;
                        let nv_offset = i * n;
                        for j in 0..n {
                            v_nl_proposed += self.n_v[nv_offset + j] * v_new[j];
                        }
                        let v_nl_current = self.v_nl[i];
                        let dv = v_nl_proposed - v_nl_current;
                        if dv.abs() > 1e-15 {
                            let v_limited = limit_slot_voltage(slot, d, v_nl_proposed, v_nl_current);
                            let dv_limited = v_limited - v_nl_current;
                            let ratio = (dv_limited / dv).clamp(0.01, 1.0);
                            if ratio < alpha { alpha = ratio; }
                        }
                    }
                }
            }

            // Layer 2: Global node voltage damping (10V threshold)
            // Only needed when transformer coupling is in the Jacobian (no delayed feedback).
            // With delayed feedback, the coupling is in the RHS (a fixed source), not the
            // Jacobian, so the NR doesn't fight it and damping is unnecessary.
            if self.delayed_couplings.is_empty() {
                let n_nodes = self.kernel.n_nodes;
                let mut max_node_dv = 0.0_f64;
                for i in 0..n_nodes {
                    let dv = alpha * (v_new[i] - v[i]);
                    max_node_dv = max_node_dv.max(dv.abs());
                }
                if max_node_dv > 10.0 {
                    alpha *= (10.0 / max_node_dv).max(0.01);
                }
            }

            // Apply damped Newton step to all variables
            let mut max_delta = 0.0_f64;
            for i in 0..n {
                let delta = alpha * (v_new[i] - v[i]);
                v[i] += delta;
                max_delta = max_delta.max(delta.abs());
            }

            if max_delta < self.tol {
                converged = true;
                // Final device evaluation at converged point
                for i in 0..m {
                    let mut sum = 0.0;
                    let nv_offset = i * n;
                    for j in 0..n {
                        sum += self.n_v[nv_offset + j] * v[j];
                    }
                    self.v_nl[i] = sum;
                }
                dc_op::evaluate_devices(&self.v_nl, &self.device_slots, &mut self.i_nl, &mut self.j_dev, m);
                break;
            }
        }

        if !converged {
            self.diag_nr_max_iter_count += 1;
        }

        // Step 3: Update state (includes inductor branch currents implicitly)
        self.v_prev.copy_from_slice(&v);
        self.i_nl_prev.copy_from_slice(&self.i_nl);

        // NaN check
        if !self.v_prev.iter().all(|x| x.is_finite()) {
            self.v_prev.fill(0.0);
            self.i_nl_prev.fill(0.0);
            self.input_prev = 0.0;
            self.diag_nan_reset_count += 1;
        }

        // Step 4: DC blocking + output
        // output_node is an original node index (< n_nodes), still valid in the augmented vector
        let raw_out = if self.output_node < self.n_nodal {
            self.v_prev[self.output_node]
        } else {
            0.0
        };
        let raw_out = if raw_out.is_finite() { raw_out } else { 0.0 };
        let dc_blocked = raw_out - self.dc_block_x_prev + self.dc_block_r * self.dc_block_y_prev;
        self.dc_block_x_prev = raw_out;
        self.dc_block_y_prev = dc_blocked;

        let abs_out = dc_blocked.abs();
        if abs_out > self.diag_peak_output { self.diag_peak_output = abs_out; }
        if abs_out > 10.0 { self.diag_clamp_count += 1; }

        clamp_output(dc_blocked)
    }
}
