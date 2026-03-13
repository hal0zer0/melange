//! Nonlinear DC operating point solver.
//!
//! Computes the steady-state bias point of a circuit by solving:
//!   G_dc · v = b_dc + N_i · i_nl(N_v · v)
//!
//! Uses Newton-Raphson with source stepping and Gmin stepping fallbacks
//! for robust convergence on BJT amplifier bias networks.
//!
//! Both `CircuitIR::from_kernel()` (codegen) and `CircuitSolver` (runtime)
//! call into this module.

use crate::codegen::ir::{DeviceParams, DeviceSlot, DeviceType};
use crate::mna::{inject_rhs_current, MnaSystem, VS_CONDUCTANCE};
use melange_devices::bjt::{BjtEbersMoll, BjtGummelPoon, BjtPolarity};
use melange_devices::diode::DiodeShockley;
use melange_devices::tube::KorenTriode;

/// Configuration for the DC operating point solver.
#[derive(Debug, Clone)]
pub struct DcOpConfig {
    /// Convergence tolerance (V)
    pub tolerance: f64,
    /// Maximum NR iterations per solve attempt
    pub max_iterations: usize,
    /// Number of source stepping stages
    pub source_steps: usize,
    /// Starting Gmin conductance for Gmin stepping
    pub gmin_start: f64,
    /// Ending Gmin conductance for Gmin stepping
    pub gmin_end: f64,
    /// Number of Gmin stepping stages
    pub gmin_steps: usize,
    /// Input node index (0-indexed into N-vector)
    pub input_node: usize,
    /// Input resistance (ohms)
    pub input_resistance: f64,
}

impl Default for DcOpConfig {
    fn default() -> Self {
        Self {
            tolerance: 1e-9,
            max_iterations: 200,
            source_steps: 10,
            gmin_start: 1e-2,
            gmin_end: 1e-12,
            gmin_steps: 10,
            input_node: 0,
            input_resistance: 1.0,
        }
    }
}

/// Result of the DC operating point solve.
#[derive(Debug, Clone)]
pub struct DcOpResult {
    /// N-vector: node voltages at DC operating point
    pub v_node: Vec<f64>,
    /// M-vector: nonlinear controlling voltages (N_v · v_node)
    pub v_nl: Vec<f64>,
    /// M-vector: nonlinear device currents at operating point
    pub i_nl: Vec<f64>,
    /// Whether the solver converged
    pub converged: bool,
    /// Which method achieved convergence
    pub method: DcOpMethod,
    /// Total NR iterations used
    pub iterations: usize,
}

/// Which convergence method was used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DcOpMethod {
    /// No nonlinear devices — used linear solve
    Linear,
    /// Direct NR from linear initial guess converged
    DirectNr,
    /// Source stepping was needed
    SourceStepping,
    /// Gmin stepping was needed
    GminStepping,
    /// All methods failed — returned linear fallback
    Failed,
}

/// Evaluate all nonlinear device currents and Jacobian entries.
///
/// # Arguments
/// * `v_nl` - M-vector of controlling voltages
/// * `device_slots` - Device slot descriptors
/// * `i_nl` - Output M-vector of device currents
/// * `j_dev` - Output M×M block-diagonal Jacobian (row-major)
fn evaluate_devices(
    v_nl: &[f64],
    device_slots: &[DeviceSlot],
    i_nl: &mut [f64],
    j_dev: &mut [f64],
    m: usize,
) {
    // Zero outputs
    for x in i_nl.iter_mut() { *x = 0.0; }
    for x in j_dev.iter_mut() { *x = 0.0; }

    for slot in device_slots {
        let s = slot.start_idx;
        match (&slot.device_type, &slot.params) {
            (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                // Use canonical DiodeShockley from melange-devices.
                // DiodeParams stores pre-multiplied n_vt, so use n=1.0, vt=n_vt.
                let diode = DiodeShockley::new(dp.is, 1.0, dp.n_vt);
                let v = v_nl[s];
                i_nl[s] = diode.current_at(v);
                j_dev[s * m + s] = diode.conductance_at(v);
            }
            (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                let polarity = if bp.is_pnp { BjtPolarity::Pnp } else { BjtPolarity::Npn };
                let em = BjtEbersMoll::new(bp.is, bp.vt, bp.beta_f, bp.beta_r, polarity);
                let vbe = v_nl[s];
                let vbc = v_nl[s + 1];

                if bp.is_gummel_poon() {
                    // Gummel-Poon model (Early effect + high injection)
                    let gp = BjtGummelPoon::new(em, bp.vaf, bp.var, bp.ikf, bp.ikr);
                    use melange_devices::NonlinearDevice;
                    i_nl[s] = gp.collector_current(vbe, vbc);
                    i_nl[s + 1] = em.base_current(vbe, vbc); // base current unchanged by GP
                    let jac = gp.jacobian(&[vbe, vbc]);
                    j_dev[s * m + s] = jac[0];        // dIc/dVbe
                    j_dev[s * m + (s + 1)] = jac[1];  // dIc/dVbc
                    j_dev[(s + 1) * m + s] = em.base_current_jacobian_dvbe(vbe, vbc);
                    j_dev[(s + 1) * m + (s + 1)] = em.base_current_jacobian_dvbc(vbe, vbc);
                } else {
                    // Standard Ebers-Moll
                    i_nl[s] = em.collector_current(vbe, vbc);
                    i_nl[s + 1] = em.base_current(vbe, vbc);
                    let (dic_dvbe, dic_dvbc) = em.collector_jacobian(vbe, vbc);
                    j_dev[s * m + s] = dic_dvbe;
                    j_dev[s * m + (s + 1)] = dic_dvbc;
                    j_dev[(s + 1) * m + s] = em.base_current_jacobian_dvbe(vbe, vbc);
                    j_dev[(s + 1) * m + (s + 1)] = em.base_current_jacobian_dvbc(vbe, vbc);
                }
            }
            (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                // 2D JFET: Vgs at start_idx, Vds at start_idx+1
                let channel = if jp.is_p_channel {
                    melange_devices::jfet::JfetChannel::P
                } else {
                    melange_devices::jfet::JfetChannel::N
                };
                let mut jfet = melange_devices::jfet::Jfet::new(channel, jp.vp, jp.idss);
                jfet.lambda = jp.lambda;
                let vgs = v_nl[s];
                let vds = v_nl[s + 1];
                i_nl[s] = jfet.drain_current(vgs, vds);
                i_nl[s + 1] = jfet.gate_current(vgs);
                let (gm, gds) = jfet.jacobian_partial(vgs, vds);
                j_dev[s * m + s] = gm;           // dId/dVgs
                j_dev[s * m + (s + 1)] = gds;     // dId/dVds
                j_dev[(s + 1) * m + s] = 0.0;     // dIg/dVgs ≈ 0
                j_dev[(s + 1) * m + (s + 1)] = 0.0; // dIg/dVds = 0
            }
            (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                // 2D MOSFET: Vgs at start_idx, Vds at start_idx+1
                let channel = if mp.is_p_channel {
                    melange_devices::mosfet::ChannelType::P
                } else {
                    melange_devices::mosfet::ChannelType::N
                };
                let mos = melange_devices::mosfet::Mosfet::new(channel, mp.vt, mp.kp, mp.lambda);
                let vgs = v_nl[s];
                let vds = v_nl[s + 1];
                i_nl[s] = mos.drain_current(vgs, vds);
                i_nl[s + 1] = 0.0; // Insulated gate — no gate current
                let (gm, gds) = mos.jacobian_partial(vgs, vds);
                j_dev[s * m + s] = gm;           // dId/dVgs
                j_dev[s * m + (s + 1)] = gds;     // dId/dVds
                j_dev[(s + 1) * m + s] = 0.0;     // dIg/dVgs = 0
                j_dev[(s + 1) * m + (s + 1)] = 0.0; // dIg/dVds = 0
            }
            (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                // Use canonical KorenTriode from melange-devices.
                // 2D: Vgk at start_idx, Vpk at start_idx+1.
                let tube = KorenTriode::with_all_params(
                    tp.mu, tp.ex, tp.kg1, tp.kp, tp.kvb, tp.ig_max, tp.vgk_onset,
                    tp.lambda,
                );
                let vgk = v_nl[s];
                let vpk = v_nl[s + 1];

                // Plate current (dimension 0) and grid current (dimension 1)
                i_nl[s] = tube.plate_current(vgk, vpk);
                i_nl[s + 1] = tube.grid_current(vgk);

                // Jacobian: [[dIp/dVgk, dIp/dVpk], [dIg/dVgk, 0]]
                use melange_devices::NonlinearDevice;
                let plate_jac = tube.jacobian(&[vgk, vpk]);
                j_dev[s * m + s] = plate_jac[0];         // dIp/dVgk
                j_dev[s * m + (s + 1)] = plate_jac[1];   // dIp/dVpk
                j_dev[(s + 1) * m + s] = tube.grid_current_jacobian(vgk); // dIg/dVgk
                j_dev[(s + 1) * m + (s + 1)] = 0.0;      // dIg/dVpk = 0
            }
            _ => {
                // Mismatched type/params — warn instead of silently skipping
                log::warn!(
                    "DC OP: unexpected device type/params combination at device index {}",
                    slot.start_idx
                );
            }
        }
    }
}

/// Build the DC conductance matrix and source vector from MNA.
///
/// At DC:
/// - Capacitors are open circuits (not stamped)
/// - Inductors are short circuits (stamped as VS_CONDUCTANCE)
/// - Voltage sources are Norton equivalents (VS_CONDUCTANCE)
/// - Input conductance is included
///
/// Returns (g_dc, b_dc) as N-sized structures.
fn build_dc_system(mna: &MnaSystem, config: &DcOpConfig) -> (Vec<Vec<f64>>, Vec<f64>) {
    let n = mna.n;

    // Start from conductance matrix (already has resistor stamps)
    let mut g_dc = mna.g.clone();

    // Stamp input conductance
    if config.input_node < n && config.input_resistance > 0.0 {
        g_dc[config.input_node][config.input_node] += 1.0 / config.input_resistance;
    }

    // Stamp inductors as short circuits at DC
    for ind in &mna.inductors {
        let ni = ind.node_i;
        let nj = ind.node_j;
        // Stamp VS_CONDUCTANCE between inductor terminals (same as voltage source)
        if ni > 0 {
            g_dc[ni - 1][ni - 1] += VS_CONDUCTANCE;
            if nj > 0 {
                g_dc[ni - 1][nj - 1] -= VS_CONDUCTANCE;
            }
        }
        if nj > 0 {
            g_dc[nj - 1][nj - 1] += VS_CONDUCTANCE;
            if ni > 0 {
                g_dc[nj - 1][ni - 1] -= VS_CONDUCTANCE;
            }
        }
    }

    // Build DC source vector
    let mut b_dc = vec![0.0; n];

    // Voltage sources (Norton equivalent)
    for vs in &mna.voltage_sources {
        let current = vs.dc_value * VS_CONDUCTANCE;
        inject_rhs_current(&mut b_dc, vs.n_plus_idx, current);
        inject_rhs_current(&mut b_dc, vs.n_minus_idx, -current);
    }

    // Current sources
    for src in &mna.current_sources {
        inject_rhs_current(&mut b_dc, src.n_plus_idx, src.dc_value);
        inject_rhs_current(&mut b_dc, src.n_minus_idx, -src.dc_value);
    }

    (g_dc, b_dc)
}

/// Compute v_nl = N_v · v (extract controlling voltages from node voltages).
fn extract_nl_voltages(mna: &MnaSystem, v: &[f64], v_nl: &mut [f64]) {
    for (i, v_nl_i) in v_nl.iter_mut().enumerate().take(mna.m) {
        *v_nl_i = mna.n_v[i].iter().zip(v.iter()).map(|(nv, vi)| nv * vi).sum();
    }
}

/// LU decomposition with partial pivoting.
///
/// Returns (LU, pivot) where LU holds L (below diagonal) and U (on+above diagonal).
/// Returns None if the matrix is singular.
fn lu_decompose(a: &[Vec<f64>]) -> Option<(Vec<Vec<f64>>, Vec<usize>)> {
    let n = a.len();
    let mut lu: Vec<Vec<f64>> = a.to_vec();
    let mut pivot: Vec<usize> = (0..n).collect();

    for k in 0..n {
        // Partial pivoting: find row with largest absolute value in column k
        let mut max_val = lu[k][k].abs();
        let mut max_row = k;
        for i in (k + 1)..n {
            if lu[i][k].abs() > max_val {
                max_val = lu[i][k].abs();
                max_row = i;
            }
        }
        if max_val < 1e-30 {
            return None; // Singular
        }
        if max_row != k {
            lu.swap(k, max_row);
            pivot.swap(k, max_row);
        }

        let diag = lu[k][k];
        for i in (k + 1)..n {
            lu[i][k] /= diag;
            let factor = lu[i][k];
            for j in (k + 1)..n {
                lu[i][j] -= factor * lu[k][j];
            }
        }
    }
    Some((lu, pivot))
}

/// Solve Ax = b given LU decomposition with pivoting.
fn lu_solve(lu: &[Vec<f64>], pivot: &[usize], b: &[f64]) -> Vec<f64> {
    let n = lu.len();
    // Permute b
    let mut x: Vec<f64> = pivot.iter().map(|&p| b[p]).collect();

    // Forward substitution (L * y = Pb)
    for i in 1..n {
        let mut sum = 0.0;
        for j in 0..i {
            sum += lu[i][j] * x[j];
        }
        x[i] -= sum;
    }

    // Back substitution (U * x = y)
    for i in (0..n).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..n {
            sum += lu[i][j] * x[j];
        }
        x[i] = (x[i] - sum) / lu[i][i];
    }
    x
}

/// Solve a linear system g_aug · v = rhs using LU decomposition.
///
/// Returns None if the matrix is singular.
fn solve_linear(g_aug: &[Vec<f64>], rhs: &[f64]) -> Option<Vec<f64>> {
    let (lu, pivot) = lu_decompose(g_aug)?;
    Some(lu_solve(&lu, &pivot, rhs))
}

/// Run Newton-Raphson DC operating point solve.
///
/// Solves: F(v) = G_dc · v - b_dc - N_i · i_nl(N_v · v) = 0
///
/// Immutable circuit data for DC NR iterations.
struct DcCircuit<'a> {
    g_dc: &'a [Vec<f64>],
    b_dc: &'a [f64],
    mna: &'a MnaSystem,
    device_slots: &'a [DeviceSlot],
    config: &'a DcOpConfig,
}

/// Uses companion formulation at each iteration:
///   G_aug = G_dc + N_i · J_dev · N_v
///   rhs = b_dc + N_i · (i_nl - J_dev · v_nl)
///   v_new = G_aug^{-1} · rhs
///
/// Returns (converged, iterations).
fn nr_dc_solve(
    circuit: &DcCircuit,
    v: &mut [f64],
    v_nl: &mut [f64],
    i_nl: &mut [f64],
    source_scale: f64,
    gmin: f64,
) -> (bool, usize) {
    let n = circuit.mna.n;
    let m = circuit.mna.m;

    // Pre-allocate working buffers
    let mut j_dev = vec![0.0; m * m];
    let mut g_aug = vec![vec![0.0; n]; n];
    let mut rhs = vec![0.0; n];

    let mna = circuit.mna;
    let device_slots = circuit.device_slots;
    let config = circuit.config;
    let g_dc = circuit.g_dc;
    let b_dc = circuit.b_dc;

    for iter in 0..config.max_iterations {
        // 1. Extract controlling voltages: v_nl = N_v · v
        extract_nl_voltages(mna, v, v_nl);

        // 2. Evaluate device currents and Jacobian
        evaluate_devices(v_nl, device_slots, i_nl, &mut j_dev, m);

        // 3. Build augmented conductance: G_aug = G_dc + N_i · J_dev · N_v + gmin_diag
        for i in 0..n {
            for j in 0..n {
                g_aug[i][j] = g_dc[i][j];
            }
        }

        // Add Gmin conductance across each nonlinear device's controlling nodes
        if gmin > 0.0 {
            for slot in device_slots {
                for d in 0..slot.dimension {
                    let nl_idx = slot.start_idx + d;
                    // Add gmin to the diagonal entries corresponding to
                    // the nodes that this NL voltage spans
                    for (j, &nv_val) in mna.n_v[nl_idx].iter().enumerate() {
                        if nv_val.abs() > 1e-15 {
                            g_aug[j][j] += gmin * nv_val.abs();
                        }
                    }
                }
            }
        }

        // Stamp -N_i · J_dev · N_v into G_aug (companion linearization)
        // G_aug = G_dc - N_i · J_dev · N_v
        // The negative sign is correct: N_i has -1 at anodes, so
        // -(-1) · g_d = +g_d stamps device conductance positively.
        for (a, g_aug_a) in g_aug.iter_mut().enumerate().take(n) {
            for (b, g_aug_ab) in g_aug_a.iter_mut().enumerate().take(n) {
                let mut sum = 0.0;
                for i in 0..m {
                    let ni_ai = mna.n_i[a][i];
                    if ni_ai.abs() < 1e-30 { continue; }
                    for j in 0..m {
                        let jd = j_dev[i * m + j];
                        if jd.abs() < 1e-30 { continue; }
                        let nv_jb = mna.n_v[j][b];
                        if nv_jb.abs() < 1e-30 { continue; }
                        sum += ni_ai * jd * nv_jb;
                    }
                }
                *g_aug_ab -= sum;
            }
        }

        // 4. Build companion RHS: rhs = b_dc_scaled + N_i · (i_nl - J_dev · v_nl)
        //    with source scaling for source stepping
        for i in 0..n {
            rhs[i] = b_dc[i] * source_scale;
        }

        // Compute companion current: i_companion = i_nl - J_dev · v_nl
        // Then inject: rhs += N_i · i_companion
        for i in 0..m {
            let mut jdev_vnl_i = 0.0;
            for j in 0..m {
                jdev_vnl_i += j_dev[i * m + j] * v_nl[j];
            }
            let i_comp = i_nl[i] - jdev_vnl_i;

            // Inject into RHS: rhs[a] += N_i[a][i] * i_comp
            for (a, rhs_a) in rhs.iter_mut().enumerate().take(n) {
                *rhs_a += mna.n_i[a][i] * i_comp;
            }
        }

        // 5. Solve: v_new = G_aug^{-1} · rhs
        let v_new = match solve_linear(&g_aug, &rhs) {
            Some(v) => v,
            None => return (false, iter + 1), // Singular matrix
        };

        // 6. Junction-aware logarithmic voltage limiting and convergence check
        //    Large steps are compressed: sign * vt * ln(|delta|/vt + 1)
        //    Small steps (~26mV) pass nearly unchanged.
        let vt = 0.026; // thermal voltage for limiting
        let mut max_delta = 0.0_f64;
        for i in 0..n {
            let delta = v_new[i] - v[i];
            let limited = if delta.abs() < vt {
                delta // Small steps pass through unchanged
            } else {
                delta.signum() * vt * (delta.abs() / vt + 1.0).ln()
            };
            v[i] += limited;
            max_delta = max_delta.max(limited.abs());
        }

        if max_delta < config.tolerance {
            // Final evaluation at converged point
            extract_nl_voltages(mna, v, v_nl);
            evaluate_devices(v_nl, device_slots, i_nl, &mut j_dev, m);
            return (true, iter + 1);
        }
    }

    // Final evaluation even on non-convergence
    extract_nl_voltages(mna, v, v_nl);
    evaluate_devices(v_nl, device_slots, i_nl, &mut vec![0.0; m * m], m);
    (false, config.max_iterations)
}

/// Compute the DC operating point for a circuit with nonlinear devices.
///
/// Strategy:
/// 1. Compute linear DC OP (G^{-1} · b_dc) as initial guess
/// 2. Try direct NR from linear guess
/// 3. If NR fails, try source stepping (ramp DC sources from 0→full)
/// 4. If source stepping fails, try Gmin stepping
/// 5. Fallback: return linear DC OP with converged=false
pub fn solve_dc_operating_point(
    mna: &MnaSystem,
    device_slots: &[DeviceSlot],
    config: &DcOpConfig,
) -> DcOpResult {
    let n = mna.n;
    let m = mna.m;

    // Build DC system
    let (g_dc, b_dc) = build_dc_system(mna, config);

    // Compute linear initial guess
    let v_linear = solve_linear(&g_dc, &b_dc).unwrap_or_else(|| vec![0.0; n]);

    // If no nonlinear devices, return linear result
    if m == 0 || device_slots.is_empty() {
        return DcOpResult {
            v_node: v_linear,
            v_nl: vec![0.0; m],
            i_nl: vec![0.0; m],
            converged: true,
            method: DcOpMethod::Linear,
            iterations: 0,
        };
    }

    let circuit = DcCircuit { g_dc: &g_dc, b_dc: &b_dc, mna, device_slots, config };

    let mut v = v_linear.clone();
    let mut v_nl = vec![0.0; m];
    let mut i_nl = vec![0.0; m];

    // Strategy 1: Direct NR from linear initial guess
    let (converged, iters) = nr_dc_solve(
        &circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, 0.0,
    );
    if converged {
        return DcOpResult {
            v_node: v,
            v_nl,
            i_nl,
            converged: true,
            method: DcOpMethod::DirectNr,
            iterations: iters,
        };
    }

    // Strategy 2: Source stepping
    // Ramp DC sources from 0 to full value, NR at each stage
    let mut v = vec![0.0; n]; // Start from zero for source stepping
    let mut total_iters = 0;
    let mut source_stepping_converged = true;

    for step in 1..=config.source_steps {
        let scale = step as f64 / config.source_steps as f64;
        let (converged, iters) = nr_dc_solve(
            &circuit, &mut v, &mut v_nl, &mut i_nl, scale, 0.0,
        );
        total_iters += iters;
        if !converged {
            source_stepping_converged = false;
            break;
        }
    }

    if source_stepping_converged {
        return DcOpResult {
            v_node: v,
            v_nl,
            i_nl,
            converged: true,
            method: DcOpMethod::SourceStepping,
            iterations: total_iters,
        };
    }

    // Strategy 3: Gmin stepping
    // Add large conductance across nonlinear devices, then ramp down
    let mut v = v_linear.clone(); // Start from linear guess
    let mut total_iters = 0;
    let mut gmin_converged = true;

    // Logarithmic ramp from gmin_start to gmin_end
    let log_start = config.gmin_start.ln();
    let log_end = config.gmin_end.ln();

    for step in 0..=config.gmin_steps {
        let frac = step as f64 / config.gmin_steps as f64;
        let gmin = (log_start + frac * (log_end - log_start)).exp();

        let (converged, iters) = nr_dc_solve(
            &circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, gmin,
        );
        total_iters += iters;
        if !converged {
            gmin_converged = false;
            break;
        }
    }

    // Final solve without Gmin
    if gmin_converged {
        let (converged, iters) = nr_dc_solve(
            &circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, 0.0,
        );
        total_iters += iters;
        if converged {
            return DcOpResult {
                v_node: v,
                v_nl,
                i_nl,
                converged: true,
                method: DcOpMethod::GminStepping,
                iterations: total_iters,
            };
        }
    }

    // All strategies failed — return linear fallback
    DcOpResult {
        v_node: v_linear,
        v_nl: vec![0.0; m],
        i_nl: vec![0.0; m],
        converged: false,
        method: DcOpMethod::Failed,
        iterations: total_iters,
    }
}
