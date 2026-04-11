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

use crate::device_types::{DeviceParams, DeviceSlot, DeviceType, TubeKind};
use crate::mna::{inject_rhs_current, MnaSystem};
use melange_devices::bjt::{BjtEbersMoll, BjtGummelPoon, BjtPolarity};
use melange_devices::diode::DiodeShockley;
use melange_devices::tube::{KorenPentode, KorenTriode};
use melange_primitives::nr::{pn_vcrit, pnjlim};

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
            source_steps: 50,
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
pub fn evaluate_devices(
    v_nl: &[f64],
    device_slots: &[DeviceSlot],
    i_nl: &mut [f64],
    j_dev: &mut [f64],
    m: usize,
) {
    evaluate_devices_inner(v_nl, device_slots, i_nl, j_dev, m, false);
}

/// Like `evaluate_devices`, but when `internal_junctions` is true, skip the
/// parasitic inner loop because v_nl already represents internal junction
/// voltages (from DC system with expanded internal nodes).
fn evaluate_devices_inner(
    v_nl: &[f64],
    device_slots: &[DeviceSlot],
    i_nl: &mut [f64],
    j_dev: &mut [f64],
    m: usize,
    internal_junctions: bool,
) {
    // Zero outputs
    for x in i_nl.iter_mut() {
        *x = 0.0;
    }
    for x in j_dev.iter_mut() {
        *x = 0.0;
    }

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
            (DeviceType::BjtForwardActive, DeviceParams::Bjt(bp)) => {
                // 1D forward-active BJT: only Vbe→Ic, Ib=Ic/BF folded into N_i
                let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                let vbe = v_nl[s];
                let vbe_eff = sign * vbe;
                let exp_be = (vbe_eff / (bp.nf * bp.vt)).clamp(-40.0, 40.0).exp();
                i_nl[s] = sign * bp.is * (exp_be - 1.0);
                j_dev[s * m + s] = bp.is / (bp.nf * bp.vt) * exp_be;
            }
            (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                if s + 1 >= v_nl.len() {
                    log::warn!(
                        "DC OP: BJT at start_idx={} needs 2 dims but v_nl.len()={}",
                        s,
                        v_nl.len()
                    );
                    continue;
                }
                let polarity = if bp.is_pnp {
                    BjtPolarity::Pnp
                } else {
                    BjtPolarity::Npn
                };
                let em = BjtEbersMoll::new(bp.is, bp.vt, bp.beta_f, bp.beta_r, polarity)
                    .with_nf(bp.nf)
                    .with_nr(bp.nr);
                let vbe = v_nl[s];
                let vbc = v_nl[s + 1];

                // If the BJT has parasitic resistances and we're NOT using
                // internal junction nodes, iteratively estimate internal junction
                // voltages. When internal_junctions is true, v_nl already contains
                // the correct internal voltages from dc_n_v.
                let (vbe_int, vbc_int) = if bp.has_parasitics() && !internal_junctions {
                    let mut vbe_i = vbe;
                    let mut vbc_i = vbc;
                    for _ in 0..3 {
                        let ic = em.collector_current(vbe_i, vbc_i);
                        let ib = em.base_current(vbe_i, vbc_i);
                        vbe_i = vbe - ib * bp.rb - (ic + ib) * bp.re;
                        vbc_i = vbc - ib * bp.rb + ic * bp.rc;
                    }
                    (vbe_i, vbc_i)
                } else {
                    (vbe, vbc)
                };

                if bp.is_gummel_poon() {
                    // Gummel-Poon model (Early effect + high injection)
                    let gp = BjtGummelPoon::new(em, bp.vaf, bp.var, bp.ikf, bp.ikr);
                    use melange_devices::NonlinearDevice;
                    i_nl[s] = gp.collector_current(vbe_int, vbc_int);
                    i_nl[s + 1] = em.base_current(vbe_int, vbc_int);
                    let jac = gp.jacobian(&[vbe_int, vbc_int]);
                    j_dev[s * m + s] = jac[0]; // dIc/dVbe
                    j_dev[s * m + (s + 1)] = jac[1]; // dIc/dVbc
                    j_dev[(s + 1) * m + s] = em.base_current_jacobian_dvbe(vbe_int, vbc_int);
                    j_dev[(s + 1) * m + (s + 1)] = em.base_current_jacobian_dvbc(vbe_int, vbc_int);
                } else {
                    // Standard Ebers-Moll
                    i_nl[s] = em.collector_current(vbe_int, vbc_int);
                    i_nl[s + 1] = em.base_current(vbe_int, vbc_int);
                    let (dic_dvbe, dic_dvbc) = em.collector_jacobian(vbe_int, vbc_int);
                    j_dev[s * m + s] = dic_dvbe;
                    j_dev[s * m + (s + 1)] = dic_dvbc;
                    j_dev[(s + 1) * m + s] = em.base_current_jacobian_dvbe(vbe_int, vbc_int);
                    j_dev[(s + 1) * m + (s + 1)] = em.base_current_jacobian_dvbc(vbe_int, vbc_int);
                }
            }
            (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                // 2D JFET: dim 0 = Vds (at start_idx), dim 1 = Vgs (at start_idx+1)
                let channel = if jp.is_p_channel {
                    melange_devices::jfet::JfetChannel::P
                } else {
                    melange_devices::jfet::JfetChannel::N
                };
                let mut jfet = melange_devices::jfet::Jfet::new(channel, jp.vp, jp.idss);
                jfet.lambda = jp.lambda;
                let vds = v_nl[s]; // dim 0 = Vds
                let vgs = v_nl[s + 1]; // dim 1 = Vgs
                i_nl[s] = jfet.drain_current(vgs, vds); // Id (dim 0 current)
                i_nl[s + 1] = jfet.gate_current(vgs); // Ig (dim 1 current)
                let (gm, gds) = jfet.jacobian_partial(vgs, vds);
                j_dev[s * m + s] = gds; // dId/dVds (dim0 curr, dim0 volt)
                j_dev[s * m + (s + 1)] = gm; // dId/dVgs (dim0 curr, dim1 volt)
                j_dev[(s + 1) * m + s] = 0.0; // dIg/dVds ≈ 0
                j_dev[(s + 1) * m + (s + 1)] = 0.0; // dIg/dVgs ≈ 0 (reverse-biased)
            }
            (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                // 2D MOSFET: dim 0 = Vds (at start_idx), dim 1 = Vgs (at start_idx+1)
                let channel = if mp.is_p_channel {
                    melange_devices::mosfet::ChannelType::P
                } else {
                    melange_devices::mosfet::ChannelType::N
                };
                let mos = melange_devices::mosfet::Mosfet::new(channel, mp.vt, mp.kp, mp.lambda);
                let vds = v_nl[s]; // dim 0 = Vds
                let vgs = v_nl[s + 1]; // dim 1 = Vgs
                i_nl[s] = mos.drain_current(vgs, vds); // Id (dim 0 current)
                i_nl[s + 1] = 0.0; // Insulated gate — no gate current (dim 1 current)
                let (gm, gds) = mos.jacobian_partial(vgs, vds);
                j_dev[s * m + s] = gds; // dId/dVds (dim0 curr, dim0 volt)
                j_dev[s * m + (s + 1)] = gm; // dId/dVgs (dim0 curr, dim1 volt)
                j_dev[(s + 1) * m + s] = 0.0; // dIg/dVds = 0
                j_dev[(s + 1) * m + (s + 1)] = 0.0; // dIg/dVgs = 0
            }
            (DeviceType::Tube, DeviceParams::Tube(tp)) => match tp.kind {
                TubeKind::SharpTriode => {
                    // Use canonical KorenTriode from melange-devices.
                    // 2D: Vgk at start_idx, Vpk at start_idx+1.
                    if s + 1 >= v_nl.len() {
                        log::warn!(
                            "DC OP: Triode at start_idx={} needs 2 dims but v_nl.len()={}",
                            s,
                            v_nl.len()
                        );
                        continue;
                    }
                    let tube = KorenTriode::with_all_params(
                        tp.mu,
                        tp.ex,
                        tp.kg1,
                        tp.kp,
                        tp.kvb,
                        tp.ig_max,
                        tp.vgk_onset,
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
                    j_dev[s * m + s] = plate_jac[0]; // dIp/dVgk
                    j_dev[s * m + (s + 1)] = plate_jac[1]; // dIp/dVpk
                    j_dev[(s + 1) * m + s] = tube.grid_current_jacobian(vgk); // dIg/dVgk
                    j_dev[(s + 1) * m + (s + 1)] = 0.0; // dIg/dVpk = 0
                }
                TubeKind::SharpPentode => {
                    // Reefman "Derk" §4.4 pentode (3D NR block).
                    // MNA layout (matches mna.rs Pentode stamping):
                    //   v_nl[s    ] = Vgk   →  i_nl[s    ] = Ip   (plate current)
                    //   v_nl[s + 1] = Vpk   →  i_nl[s + 1] = Ig2  (screen current)
                    //   v_nl[s + 2] = Vg2k  →  i_nl[s + 2] = Ig1  (control-grid current)
                    //
                    // The KorenPentode device in melange-devices implements the
                    // canonical Reefman equations (see pentode_equations.md memory
                    // ref). We delegate plate/screen/grid currents and the 3×3
                    // Jacobian to it so this dispatch arm stays declarative and
                    // line-for-line matches the reference math.
                    if s + 2 >= v_nl.len() {
                        log::warn!(
                            "DC OP: Pentode at start_idx={} needs 3 dims but v_nl.len()={}",
                            s,
                            v_nl.len()
                        );
                        continue;
                    }
                    let pentode = KorenPentode {
                        mu: tp.mu,
                        ex: tp.ex,
                        kg1: tp.kg1,
                        kg2: tp.kg2,
                        kp: tp.kp,
                        kvb: tp.kvb,
                        alpha_s: tp.alpha_s,
                        a_factor: tp.a_factor,
                        beta_factor: tp.beta_factor,
                        ig_max: tp.ig_max,
                        vgk_onset: tp.vgk_onset,
                        // Cross-crate enum conversion: melange-devices owns the
                        // runtime `KorenPentode::ScreenForm`; melange-solver
                        // owns the serialized `TubeParams::ScreenForm`. Two
                        // identical-shape enums in different crates (devices
                        // can't depend on solver); map 1:1 here.
                        screen_form: match tp.screen_form {
                            crate::device_types::ScreenForm::Rational => {
                                melange_devices::tube::ScreenForm::Rational
                            }
                            crate::device_types::ScreenForm::Exponential => {
                                melange_devices::tube::ScreenForm::Exponential
                            }
                        },
                    };
                    let vgk = v_nl[s];
                    let vpk = v_nl[s + 1];
                    let vg2k = v_nl[s + 2];

                    // Currents (rows 0..3)
                    i_nl[s] = pentode.plate_current(vgk, vpk, vg2k);
                    i_nl[s + 1] = pentode.screen_current(vgk, vpk, vg2k);
                    i_nl[s + 2] = pentode.grid_current(vgk);

                    // 3×3 analytic Jacobian (rows = [Ip, Ig2, Ig1],
                    // cols = [Vgk, Vpk, Vg2k]). KorenPentode hands it back in
                    // the same row/col order our MNA expects.
                    let jac = pentode.jacobian_3x3(vgk, vpk, vg2k);
                    j_dev[s * m + s] = jac[0][0]; // dIp/dVgk
                    j_dev[s * m + (s + 1)] = jac[0][1]; // dIp/dVpk
                    j_dev[s * m + (s + 2)] = jac[0][2]; // dIp/dVg2k
                    j_dev[(s + 1) * m + s] = jac[1][0]; // dIg2/dVgk
                    j_dev[(s + 1) * m + (s + 1)] = jac[1][1]; // dIg2/dVpk
                    j_dev[(s + 1) * m + (s + 2)] = jac[1][2]; // dIg2/dVg2k
                    j_dev[(s + 2) * m + s] = jac[2][0]; // dIg1/dVgk
                    j_dev[(s + 2) * m + (s + 1)] = 0.0; // dIg1/dVpk = 0
                    j_dev[(s + 2) * m + (s + 2)] = 0.0; // dIg1/dVg2k = 0
                }
            },
            (DeviceType::Vca, DeviceParams::Vca(vp)) => {
                // 2D VCA: dim 0 = V_signal (at start_idx), dim 1 = V_control (at start_idx+1)
                let v_sig = v_nl[s];
                let v_ctrl = v_nl[s + 1];
                let vca = melange_devices::Vca::new(vp.vscale, vp.g0);
                i_nl[s] = vca.current(v_sig, v_ctrl);
                i_nl[s + 1] = 0.0; // Control port draws no current
                let jac = vca.jacobian(v_sig, v_ctrl);
                j_dev[s * m + s] = jac[0]; // dI_sig/dV_sig
                j_dev[s * m + (s + 1)] = jac[1]; // dI_sig/dV_ctrl
                j_dev[(s + 1) * m + s] = 0.0; // dI_ctrl/dV_sig = 0
                j_dev[(s + 1) * m + (s + 1)] = 0.0; // dI_ctrl/dV_ctrl = 0
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

/// Internal node mapping for a BJT with parasitic resistances in the DC system.
/// When RB/RC/RE > 0, internal nodes are created so the intrinsic BJT model
/// operates on internal junction voltages while parasitic R is in the global matrix.
struct BjtInternalNodes {
    start_idx: usize, // M-dimension start index
    ext_base: usize,  // 1-indexed external node (0=ground)
    ext_collector: usize,
    ext_emitter: usize,
    int_base: usize, // 0-indexed into g_dc (same as ext if R=0)
    int_collector: usize,
    int_emitter: usize,
}

/// Extended DC system info including internal nodes for parasitic BJTs.
struct DcSystemInfo {
    g_dc: Vec<Vec<f64>>,
    b_dc: Vec<f64>,
    n_dc: usize,
    /// DC-local N_v (M × n_dc) — uses internal nodes for parasitic BJTs
    dc_n_v: Vec<Vec<f64>>,
    /// DC-local N_i (n_dc × M) — uses internal nodes for parasitic BJTs
    dc_n_i: Vec<Vec<f64>>,
    /// Internal node mappings (empty if no parasitic BJTs)
    bjt_internal: Vec<BjtInternalNodes>,
    /// Index where internal BJT nodes start (for damping).
    internal_node_start: usize,
}

/// Build the DC conductance matrix and source vector from MNA.
///
/// At DC:
/// - Capacitors are open circuits (not stamped, so C rows in G are absent)
/// - Inductors are short circuits (augmented MNA constraints)
/// - Voltage sources use augmented MNA (already in mna.g)
/// - BJTs with parasitic R get internal nodes (like ngspice basePrime/colPrime/emitPrime)
///
/// Returns DcSystemInfo with expanded matrices.
fn build_dc_system(
    mna: &MnaSystem,
    device_slots: &[DeviceSlot],
    config: &DcOpConfig,
) -> DcSystemInfo {
    let n = mna.n;
    let n_aug = mna.n_aug;

    // Collect all inductor node pairs for augmented DC short-circuit constraints.
    // At DC, each inductor is V_plus - V_minus = 0 (exact short circuit).
    // We model this with augmented MNA (same as voltage sources with V_dc=0),
    // not with a large conductance hack.
    let mut inductor_pairs: Vec<(usize, usize)> = Vec::new();
    for ind in &mna.inductors {
        inductor_pairs.push((ind.node_i, ind.node_j));
    }
    for ci in &mna.coupled_inductors {
        inductor_pairs.push((ci.l1_node_i, ci.l1_node_j));
        inductor_pairs.push((ci.l2_node_i, ci.l2_node_j));
    }
    for group in &mna.transformer_groups {
        for k in 0..group.num_windings {
            inductor_pairs.push((group.winding_node_i[k], group.winding_node_j[k]));
        }
    }
    // Ideal transformer couplings: the coupled windings are already replaced by
    // leakage + magnetizing inductors (in mna.inductors), so they're covered above.

    let num_ind = inductor_pairs.len();
    let n_dc = n_aug + num_ind; // DC system dimension

    // Start from the augmented conductance matrix, expanded for inductor constraints.
    let mut g_dc = vec![vec![0.0; n_dc]; n_dc];
    // Copy the n_aug × n_aug G matrix
    for i in 0..n_aug {
        for j in 0..n_aug {
            g_dc[i][j] = mna.g[i][j];
        }
    }

    // Stamp input conductance
    if config.input_node < n && config.input_resistance > 0.0 {
        g_dc[config.input_node][config.input_node] += 1.0 / config.input_resistance;
    }

    // Stamp inductor short-circuit constraints as augmented rows/cols.
    // At DC, inductors enforce V(ni) - V(nj) = 0 (zero resistance).
    // KVL row uses inductor convention (-V_i + V_j) to match the augmented MNA
    // transient solver, so the branch current polarity is consistent.
    for (idx, &(ni, nj)) in inductor_pairs.iter().enumerate() {
        let k = n_aug + idx; // augmented row/col for this inductor
                             // KVL row: -V(ni) + V(nj) = 0 (inductor convention)
        if ni > 0 {
            g_dc[k][ni - 1] -= 1.0;
        }
        if nj > 0 {
            g_dc[k][nj - 1] += 1.0;
        }
        // KCL: current j_ind enters node_i, exits node_j
        if ni > 0 {
            g_dc[ni - 1][k] += 1.0;
        }
        if nj > 0 {
            g_dc[nj - 1][k] -= 1.0;
        }
    }

    // Build DC source vector (n_dc sized).
    let mut b_dc = vec![0.0; n_dc];

    // Voltage sources: augmented row = V_dc.
    for vs in &mna.voltage_sources {
        b_dc[n + vs.ext_idx] = vs.dc_value;
    }

    // Inductor constraints: b_dc[n_aug + idx] = 0 (short circuit, already zero).

    // Current sources inject into node rows (0..n-1)
    for src in &mna.current_sources {
        inject_rhs_current(&mut b_dc, src.n_plus_idx, src.dc_value);
        inject_rhs_current(&mut b_dc, src.n_minus_idx, -src.dc_value);
    }

    // === BJT internal nodes for parasitic resistances ===
    // Like ngspice basePrime/colPrime/emitPrime: create internal nodes so the
    // intrinsic model operates at correct junction voltages while parasitic R
    // is in the global matrix. This fixes DC OP convergence for Class AB amps.
    let mut bjt_internal = Vec::new();
    let internal_node_start = n_dc; // where internal nodes begin
    let mut n_dc = n_dc; // will grow as we add internal nodes

    for (slot_idx, slot) in device_slots.iter().enumerate() {
        if let DeviceParams::Bjt(bp) = &slot.params {
            // Skip if MNA already has internal nodes for this BJT
            let has_mna_internal = mna
                .bjt_internal_nodes
                .iter()
                .any(|n| n.start_idx == slot.start_idx);
            if bp.has_parasitics() && slot.dimension == 2 && !has_mna_internal {
                // Get external node indices (1-indexed, 0=ground)
                if slot_idx < mna.nonlinear_devices.len() {
                    let dev = &mna.nonlinear_devices[slot_idx];
                    let ext_c = dev.node_indices[0]; // collector
                    let ext_b = dev.node_indices[1]; // base
                    let ext_e = dev.node_indices[2]; // emitter

                    // Create internal nodes for non-zero resistances
                    let int_b = if bp.rb > 0.0 {
                        let idx = n_dc;
                        n_dc += 1;
                        idx
                    } else if ext_b > 0 {
                        ext_b - 1
                    } else {
                        0
                    };
                    let int_c = if bp.rc > 0.0 {
                        let idx = n_dc;
                        n_dc += 1;
                        idx
                    } else if ext_c > 0 {
                        ext_c - 1
                    } else {
                        0
                    };
                    let int_e = if bp.re > 0.0 {
                        let idx = n_dc;
                        n_dc += 1;
                        idx
                    } else if ext_e > 0 {
                        ext_e - 1
                    } else {
                        0
                    };

                    bjt_internal.push(BjtInternalNodes {
                        start_idx: slot.start_idx,
                        ext_base: ext_b,
                        ext_collector: ext_c,
                        ext_emitter: ext_e,
                        int_base: int_b,
                        int_collector: int_c,
                        int_emitter: int_e,
                    });
                }
            }
        }
    }

    // Expand g_dc and b_dc to new n_dc if internal nodes were added
    if n_dc > g_dc.len() {
        let old_n = g_dc.len();
        for row in g_dc.iter_mut() {
            row.resize(n_dc, 0.0);
        }
        for _ in old_n..n_dc {
            g_dc.push(vec![0.0; n_dc]);
        }
        b_dc.resize(n_dc, 0.0);
    }

    // Stamp parasitic conductances between external and internal nodes
    for bjt in &bjt_internal {
        if let Some(slot) = device_slots.iter().find(|s| s.start_idx == bjt.start_idx) {
            if let DeviceParams::Bjt(bp) = &slot.params {
                // RB: external base ↔ internal base
                if bp.rb > 0.0 {
                    let g = 1.0 / bp.rb;
                    if bjt.ext_base > 0 {
                        let eb = bjt.ext_base - 1;
                        g_dc[eb][eb] += g;
                        g_dc[eb][bjt.int_base] -= g;
                        g_dc[bjt.int_base][eb] -= g;
                    }
                    g_dc[bjt.int_base][bjt.int_base] += g;
                }
                // RC: external collector ↔ internal collector
                if bp.rc > 0.0 {
                    let g = 1.0 / bp.rc;
                    if bjt.ext_collector > 0 {
                        let ec = bjt.ext_collector - 1;
                        g_dc[ec][ec] += g;
                        g_dc[ec][bjt.int_collector] -= g;
                        g_dc[bjt.int_collector][ec] -= g;
                    }
                    g_dc[bjt.int_collector][bjt.int_collector] += g;
                }
                // RE: external emitter ↔ internal emitter
                if bp.re > 0.0 {
                    let g = 1.0 / bp.re;
                    if bjt.ext_emitter > 0 {
                        let ee = bjt.ext_emitter - 1;
                        g_dc[ee][ee] += g;
                        g_dc[ee][bjt.int_emitter] -= g;
                        g_dc[bjt.int_emitter][ee] -= g;
                    }
                    g_dc[bjt.int_emitter][bjt.int_emitter] += g;
                }
            }
        }
    }

    // Regularize: Gmin on all circuit nodes AND internal nodes
    let gmin_floor = 1e-12;
    for i in 0..n {
        g_dc[i][i] += gmin_floor;
    }
    // Also regularize internal nodes
    for bjt in &bjt_internal {
        if let Some(slot) = device_slots.iter().find(|s| s.start_idx == bjt.start_idx) {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.rb > 0.0 {
                    g_dc[bjt.int_base][bjt.int_base] += gmin_floor;
                }
                if bp.rc > 0.0 {
                    g_dc[bjt.int_collector][bjt.int_collector] += gmin_floor;
                }
                if bp.re > 0.0 {
                    g_dc[bjt.int_emitter][bjt.int_emitter] += gmin_floor;
                }
            }
        }
    }

    // Build DC-local N_v (M × n_dc) and N_i (n_dc × M)
    // Start from MNA's N_v/N_i, then redirect parasitic BJTs to internal nodes
    let m = mna.m;
    let mut dc_n_v = vec![vec![0.0; n_dc]; m];
    let mut dc_n_i = vec![vec![0.0; m]; n_dc];

    // Copy MNA matrices (n_aug portion)
    for i in 0..m {
        for j in 0..n_aug.min(n_dc) {
            dc_n_v[i][j] = mna.n_v[i][j];
        }
    }
    for i in 0..n_aug.min(n_dc) {
        for j in 0..m {
            dc_n_i[i][j] = mna.n_i[i][j];
        }
    }

    // Redirect parasitic BJTs to internal nodes
    for bjt in &bjt_internal {
        let s = bjt.start_idx;
        if s + 1 >= m {
            continue;
        }

        // Clear external entries for this BJT
        for j in 0..n_dc {
            dc_n_v[s][j] = 0.0;
            dc_n_v[s + 1][j] = 0.0;
        }
        for i in 0..n_dc {
            dc_n_i[i][s] = 0.0;
            dc_n_i[i][s + 1] = 0.0;
        }

        // N_v: extract Vbe_int = V(basePrime) - V(emitterPrime)
        dc_n_v[s][bjt.int_base] = 1.0;
        dc_n_v[s][bjt.int_emitter] = -1.0;
        // N_v: extract Vbc_int = V(basePrime) - V(collectorPrime)
        dc_n_v[s + 1][bjt.int_base] = 1.0;
        dc_n_v[s + 1][bjt.int_collector] = -1.0;

        // N_i: Ic enters collectorPrime, exits emitterPrime
        dc_n_i[bjt.int_collector][s] = -1.0;
        dc_n_i[bjt.int_emitter][s] = 1.0;
        // N_i: Ib enters basePrime, exits emitterPrime
        dc_n_i[bjt.int_base][s + 1] = -1.0;
        dc_n_i[bjt.int_emitter][s + 1] = 1.0;
    }

    DcSystemInfo {
        g_dc,
        b_dc,
        n_dc,
        dc_n_v,
        dc_n_i,
        bjt_internal,
        internal_node_start,
    }
}

/// Compute v_nl = N_v · v (extract controlling voltages from node voltages).
pub fn extract_nl_voltages(mna: &MnaSystem, v: &[f64], v_nl: &mut [f64]) {
    extract_nl_voltages_with(mna.m, &mna.n_v, v, v_nl);
}

/// Compute v_nl = N_v · v using an explicit N_v matrix (M × n_dc).
fn extract_nl_voltages_with(m: usize, n_v: &[Vec<f64>], v: &[f64], v_nl: &mut [f64]) {
    for (i, v_nl_i) in v_nl.iter_mut().enumerate().take(m) {
        *v_nl_i = n_v[i].iter().zip(v.iter()).map(|(nv, vi)| nv * vi).sum();
    }
}

/// LU decomposition with partial pivoting.
///
/// Returns (LU, pivot) where LU holds L (below diagonal) and U (on+above diagonal).
/// Returns None if the matrix is singular.
pub fn lu_decompose(a: &[Vec<f64>]) -> Option<(Vec<Vec<f64>>, Vec<usize>)> {
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
pub fn lu_solve(lu: &[Vec<f64>], pivot: &[usize], b: &[f64]) -> Vec<f64> {
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
pub fn solve_linear(g_aug: &[Vec<f64>], rhs: &[f64]) -> Option<Vec<f64>> {
    let (lu, pivot) = lu_decompose(g_aug)?;
    Some(lu_solve(&lu, &pivot, rhs))
}

/// Clamp junction voltages in the linear initial guess to prevent NR divergence.
///
/// For BJT circuits, the linear solution (no nonlinear currents) often produces
/// junction voltages of several volts (e.g., Vbe = 2.8V for a direct-coupled stage)
/// because the BJT is not conducting in the linear model. These extreme voltages
/// cause exp(V/Vt) to saturate at exp(40), producing enormous currents and
/// Jacobians that make NR oscillate.
///
/// This function detects junctions with |V_nl| > V_CLAMP and adjusts the
/// appropriate node voltages to bring controlling voltages within range.
/// Diodes and BJTs get full PN junction pre-bias. Sharp pentodes get a Class A
/// power-stage seed (Vg2k clamped to ≤250V, Vgk clamped to ≈-2V) so the Reefman
/// Derk equations start near a real bias point — without it the linear solve
/// puts both grids at 0V where Ip is enormous and NR diverges.
/// Other tubes (sharp triodes) and FETs are left alone.
fn clamp_junction_voltages(
    mna: &MnaSystem,
    device_slots: &[DeviceSlot],
    v_linear: &[f64],
) -> Vec<f64> {
    let mut v = v_linear.to_vec();
    let m = mna.m;
    let n_aug = mna.n_aug.min(v.len());

    let mut v_nl = vec![0.0; m];
    extract_nl_voltages(mna, &v, &mut v_nl);

    // For each BJT, set the emitter node so that Vbe ≈ 0.65V, and set the
    // collector node for reasonable Vce. This adjusts the "dependent" nodes
    // (emitter, collector) rather than the base, preserving resistor-divider bias.
    // For diodes, adjust the cathode node to bring the junction to ±0.6V.
    for slot in device_slots {
        match slot.device_type {
            DeviceType::Diode => {
                let nl_idx = slot.start_idx;
                if nl_idx >= m {
                    continue;
                }
                let v_diode = v_nl[nl_idx];
                if v_diode.abs() <= 0.8 {
                    continue;
                }

                // Find anode (+N_v) and cathode (-N_v) nodes
                let nv_row = &mna.n_v[nl_idx];
                let mut anode = None;
                let mut cathode = None;
                for j in 0..n_aug {
                    if nv_row[j] > 0.5 {
                        anode = Some(j);
                    }
                    if nv_row[j] < -0.5 {
                        cathode = Some(j);
                    }
                }
                // Set cathode voltage so V_junction = sign * 0.6V
                if let (Some(a), Some(c)) = (anode, cathode) {
                    let target = 0.6 * v_diode.signum();
                    v[c] = v[a] - target;
                }
            }
            DeviceType::Bjt | DeviceType::BjtForwardActive => {
                let vbe_idx = slot.start_idx;
                let vbc_idx = slot.start_idx + 1;
                if vbe_idx >= m || (slot.dimension > 1 && vbc_idx >= m) {
                    continue;
                }

                // Determine PNP vs NPN. For PNP, forward-active Vbe < 0.
                let is_pnp = matches!(&slot.params, DeviceParams::Bjt(bp) if bp.is_pnp);
                let sign: f64 = if is_pnp { -1.0 } else { 1.0 };

                // Find base and emitter nodes from Vbe N_v row
                let nv_vbe = &mna.n_v[vbe_idx];
                let mut base_node = None;
                let mut emitter_node = None;
                for j in 0..n_aug {
                    if nv_vbe[j] > 0.5 {
                        base_node = Some(j);
                    }
                    if nv_vbe[j] < -0.5 {
                        emitter_node = Some(j);
                    }
                }

                // Set emitter so Vbe = sign * 0.65V (positive for NPN, negative for PNP)
                if let (Some(b), Some(e)) = (base_node, emitter_node) {
                    let new_ve = v[b] - sign * 0.65;
                    if (v[e] - new_ve).abs() > 0.1 {
                        v[e] = new_ve;
                    }
                }

                // Find collector node from Vbc N_v row
                if slot.dimension > 1 {
                    let nv_vbc = &mna.n_v[vbc_idx];
                    let mut collector_node = None;
                    for j in 0..n_aug {
                        if nv_vbc[j] < -0.5 {
                            collector_node = Some(j);
                        }
                    }

                    // Set collector for forward-active mode: Vbc reverse biased.
                    // NPN: collector ABOVE base (Vbc < 0). PNP: collector BELOW base (Vbc > 0).
                    if let (Some(b), Some(c)) = (base_node, collector_node) {
                        let vbc = v[b] - v[c];
                        let need_adjust = if is_pnp { vbc < 0.5 } else { vbc > -0.5 };
                        if need_adjust {
                            if is_pnp {
                                // PNP: collector below base. Find lowest voltage as "ground".
                                let v_ground =
                                    v.iter().take(mna.n).cloned().fold(f64::MAX, f64::min);
                                v[c] = v[b] - (v[b] - v_ground) * 0.3;
                            } else {
                                // NPN: collector above base. Find highest voltage as supply.
                                let v_supply =
                                    v.iter().take(mna.n).cloned().fold(0.0_f64, f64::max);
                                v[c] = v[b] + (v_supply - v[b]) * 0.3;
                            }
                        }
                    }
                }
            }
            DeviceType::Tube => {
                // Pentodes only — sharp triodes are well-conditioned from the
                // linear solve (Vgk ≈ 0 → Ip in mA, NR converges fine).
                // Sharp pentodes need a Class A bias seed because:
                //   * The Reefman Derk Ip0 = E1^Ex term explodes when Vgk ≈ 0
                //     and Vg2k is at the supply rail (100s of volts), pushing
                //     `inner` past the softplus knee.
                //   * Linear solve places the cathode at GND and screen at B+,
                //     so Vg2k = full supply (e.g. 300V on EL84) before NR even
                //     starts. We need to clamp Vg2k to a sane operating point
                //     and force Vgk negative.
                let (is_pentode, _kg2_present) = match &slot.params {
                    DeviceParams::Tube(tp) => (
                        matches!(tp.kind, TubeKind::SharpPentode),
                        tp.kg2 > 0.0,
                    ),
                    _ => (false, false),
                };
                if !is_pentode || slot.dimension < 3 {
                    continue;
                }
                let vgk_idx = slot.start_idx;
                let vpk_idx = slot.start_idx + 1;
                let vg2k_idx = slot.start_idx + 2;
                if vg2k_idx >= m {
                    continue;
                }

                // Locate the four pentode terminal nodes via the N_v rows that
                // mna.rs populated for this device:
                //   row vgk_idx : +1 grid, -1 cathode
                //   row vpk_idx : +1 plate, -1 cathode
                //   row vg2k_idx: +1 screen, -1 cathode
                let (mut grid_node, mut cathode_node, mut plate_node, mut screen_node) =
                    (None, None, None, None);
                let nv_vgk = &mna.n_v[vgk_idx];
                let nv_vpk = &mna.n_v[vpk_idx];
                let nv_vg2k = &mna.n_v[vg2k_idx];
                for j in 0..n_aug {
                    if nv_vgk[j] > 0.5 {
                        grid_node = Some(j);
                    }
                    if nv_vgk[j] < -0.5 {
                        cathode_node = Some(j);
                    }
                    if nv_vpk[j] > 0.5 {
                        plate_node = Some(j);
                    }
                    if nv_vg2k[j] > 0.5 {
                        screen_node = Some(j);
                    }
                }

                // Estimate B+ from the highest circuit-node voltage in the
                // linear solve. (Augmented variables — VS branch currents,
                // inductor branch currents — live at indices ≥ mna.n and must
                // not be sampled here.)
                let v_supply = v.iter().take(mna.n).cloned().fold(0.0_f64, f64::max);

                // Cathode reference (default to ground if cathode_node is None
                // or grounded). For typical class A power stages this is 0V on
                // the linear solve because Rk is fixed and Ip is zero.
                let v_cath = cathode_node.map(|c| v[c]).unwrap_or(0.0);

                // Vgk: drive the grid roughly 2V below the cathode. The grid
                // is usually a high-impedance node biased through a grid-leak
                // resistor to ground, so the linear solve leaves it at the
                // cathode potential. Pulling it 2V down gives the Reefman
                // softplus a finite, conducting starting point.
                if let Some(g) = grid_node {
                    let target_vg = v_cath - 2.0;
                    if (v[g] - target_vg).abs() > 0.05 {
                        v[g] = target_vg;
                    }
                }

                // Vg2k: clamp the screen to a sane Class A operating value.
                // Real audio amps run the screen at ~250V (EL84/EL34) or
                // wherever the screen-dropping resistor lands; if the supply
                // is below 250V (e.g. small signal bias network) we use
                // supply - 50V to leave headroom for the screen-grid drop.
                if let Some(s_node) = screen_node {
                    let target_vg2 = v_cath + (v_supply - v_cath - 50.0).min(250.0).max(20.0);
                    let v_g2 = v[s_node];
                    if v_g2 - v_cath > 260.0 || v_g2 - v_cath < 10.0 {
                        v[s_node] = target_vg2;
                    }
                }

                // Vpk: clamp plate near the screen voltage as a starting
                // guess (real Class A plates sit at supply minus IR_load).
                // Only adjust if the linear plate is far above the screen,
                // which happens when the plate-load resistor is large and Ip
                // was zero in the linear solve.
                if let (Some(p), Some(s_node)) = (plate_node, screen_node) {
                    let v_g2 = v[s_node];
                    if v[p] - v_g2 > 50.0 {
                        v[p] = v_g2;
                    }
                }
            }
            _ => {}
        }
    }

    v
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
    /// DC-local N_v (M × n_dc) — may have internal nodes for parasitic BJTs
    dc_n_v: &'a [Vec<f64>],
    /// DC-local N_i (n_dc × M) — may have internal nodes for parasitic BJTs
    dc_n_i: &'a [Vec<f64>],
    /// DC system dimension (may be > n_aug due to inductors + internal nodes)
    n_dc: usize,
    /// Whether internal junction nodes are present (skip parasitic inner loop)
    has_internal_nodes: bool,
    /// Index where internal BJT nodes start (for damping).
    /// Augmented variables (n..n_aug_plus_ind) are NOT damped, but internal
    /// nodes (n_aug_plus_ind..n_dc) MUST be damped like circuit nodes.
    internal_node_start: usize,
}

/// Uses companion formulation at each iteration:
///   G_aug = G_dc - N_i · J_dev · N_v
///   rhs = b_dc + N_i · (i_nl - J_dev · v_nl)
///   v_new = G_aug^{-1} · rhs
///
/// Returns (converged, iterations).
fn nr_dc_solve(
    circuit: &DcCircuit,
    v: &mut Vec<f64>,
    v_nl: &mut [f64],
    i_nl: &mut [f64],
    source_scale: f64,
    gmin: f64,
) -> (bool, usize) {
    let n_dc = circuit.n_dc;
    let m = circuit.mna.m;

    // Pre-allocate working buffers (all n_dc-sized)
    let mut j_dev = vec![0.0; m * m];
    let mut g_aug = vec![vec![0.0; n_dc]; n_dc];
    let mut rhs = vec![0.0; n_dc];

    let mna = circuit.mna;
    let device_slots = circuit.device_slots;
    let config = circuit.config;
    let g_dc = circuit.g_dc;
    let b_dc = circuit.b_dc;
    let dc_n_v = circuit.dc_n_v;
    let dc_n_i = circuit.dc_n_i;

    // Ensure v is the right size for the DC system (may be larger than MNA due to internal nodes)
    if v.len() < n_dc {
        v.resize(n_dc, 0.0);
    }

    let internal_junctions = circuit.has_internal_nodes;

    for iter in 0..config.max_iterations {
        // 1. Extract controlling voltages: v_nl = dc_N_v · v  (dc_N_v is M × n_dc)
        extract_nl_voltages_with(m, dc_n_v, v, v_nl);

        // 2. Evaluate device currents and Jacobian
        // When internal nodes are present, v_nl already has true junction voltages
        // — skip the parasitic inner loop to avoid double-correction.
        evaluate_devices_inner(v_nl, device_slots, i_nl, &mut j_dev, m, internal_junctions);

        // Check for NaN/Inf in device evaluation
        if i_nl.iter().any(|x| !x.is_finite()) || j_dev.iter().any(|x| !x.is_finite()) {
            log::warn!("DC NR iter {}: NaN/Inf in device evaluation", iter);
            return (false, iter + 1);
        }

        if iter < 3 && m >= 4 {
            log::info!(
                "DC NR iter {} (scale={:.2}, gmin={:.1e}): v_nl[0..4]=[{:.3}, {:.3}, {:.3}, {:.3}]",
                iter,
                source_scale,
                gmin,
                v_nl[0],
                v_nl[1],
                v_nl[2],
                v_nl[3]
            );
        }

        // 3. Build augmented conductance: G_aug = G_dc (n_dc × n_dc copy)
        for i in 0..n_dc {
            for j in 0..n_dc {
                g_aug[i][j] = g_dc[i][j];
            }
        }

        // Add Gmin conductance across each nonlinear device's controlling nodes.
        // dc_n_v[nl_idx] has n_dc entries; only some are nonzero.
        if gmin > 0.0 {
            for slot in device_slots {
                for d in 0..slot.dimension {
                    let nl_idx = slot.start_idx + d;
                    for (j, &nv_val) in dc_n_v[nl_idx].iter().enumerate() {
                        if nv_val.abs() > 1e-15 {
                            g_aug[j][j] += gmin * nv_val.abs();
                        }
                    }
                }
            }
        }

        // Stamp -N_i · J_dev · N_v into G_aug (companion linearization).
        // G_aug = G_dc - dc_N_i · J_dev · dc_N_v
        // dc_N_i is n_dc × M, dc_N_v is M × n_dc.
        for (a, g_aug_a) in g_aug.iter_mut().enumerate().take(n_dc) {
            for (b, g_aug_ab) in g_aug_a.iter_mut().enumerate().take(n_dc) {
                let mut sum = 0.0;
                for i in 0..m {
                    let ni_ai = dc_n_i[a][i];
                    if ni_ai.abs() < 1e-30 {
                        continue;
                    }
                    for j in 0..m {
                        let jd = j_dev[i * m + j];
                        if jd.abs() < 1e-30 {
                            continue;
                        }
                        let nv_jb = dc_n_v[j][b];
                        if nv_jb.abs() < 1e-30 {
                            continue;
                        }
                        sum += ni_ai * jd * nv_jb;
                    }
                }
                *g_aug_ab -= sum;
            }
        }

        // 4. Build companion RHS: rhs = b_dc_scaled + dc_N_i · (i_nl - J_dev · v_nl)
        //    Source stepping scales ALL sources (both current sources in node rows AND
        //    voltage source constraints in augmented rows). This ramps the supply from
        //    0V to full, letting BJTs gradually turn on at each step.
        //    Inductor short-circuit constraints (V=0) are unaffected by scaling.
        let n_aug = mna.n_aug;
        for i in 0..n_aug {
            rhs[i] = b_dc[i] * source_scale;
        }
        // Inductor short-circuit constraints and internal nodes: no source scaling
        for i in n_aug..n_dc {
            rhs[i] = b_dc[i]; // b_dc[i] = 0 for inductor constraints and internal nodes
        }

        // Compute companion current: i_companion = i_nl - J_dev · v_nl
        // Then inject: rhs += dc_N_i · i_companion
        for i in 0..m {
            let mut jdev_vnl_i = 0.0;
            for j in 0..m {
                jdev_vnl_i += j_dev[i * m + j] * v_nl[j];
            }
            let i_comp = i_nl[i] - jdev_vnl_i;

            // Inject into RHS: rhs[a] += dc_N_i[a][i] * i_comp
            for (a, rhs_a) in rhs.iter_mut().enumerate().take(n_dc) {
                *rhs_a += dc_n_i[a][i] * i_comp;
            }
        }

        // 5. Solve: v_new = G_aug^{-1} · rhs
        let v_new = match solve_linear(&g_aug, &rhs) {
            Some(v) => v,
            None => {
                log::warn!(
                    "DC NR iter {}: Jacobian singular (scale={}, gmin={:.2e})",
                    iter,
                    source_scale,
                    gmin
                );
                return (false, iter + 1);
            }
        };

        // 6. Voltage limiting and convergence check.
        //    Apply SPICE pnjlim to PN junction dimensions, global node damping
        //    for large updates, and a generous flat clamp for non-junction nodes.

        // First, apply pnjlim to PN junction controlling voltages.
        // Extract old and new v_nl to apply junction-aware limiting.
        let mut v_nl_new = vec![0.0; m];
        extract_nl_voltages_with(m, dc_n_v, &v_new, &mut v_nl_new);

        // Apply pnjlim per PN junction dimension, accumulate corrections
        // into node voltages via dc_N_v^T (pseudo-inverse: just distribute back).
        let mut pn_corrections = vec![0.0; n_dc];
        for slot in device_slots {
            match (&slot.device_type, &slot.params) {
                (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                    let idx = slot.start_idx;
                    let vt = dp.n_vt; // pre-multiplied n*Vt
                    let vcrit = pn_vcrit(vt, dp.is);
                    let v_old = v_nl[idx];
                    let v_raw = v_nl_new[idx];
                    let v_lim = pnjlim(v_raw, v_old, vt, vcrit);
                    if (v_lim - v_raw).abs() > 1e-15 {
                        let correction = v_lim - v_raw;
                        for j in 0..n_dc {
                            pn_corrections[j] += dc_n_v[idx][j] * correction;
                        }
                    }
                }
                (DeviceType::Bjt, DeviceParams::Bjt(bp))
                | (DeviceType::BjtForwardActive, DeviceParams::Bjt(bp)) => {
                    // When internal nodes are present for parasitic BJTs, skip pnjlim
                    // correction distribution. The N_v^T distribution pushes corrections
                    // equally to both internal nodes, which badly destabilizes the weakly-
                    // connected internal base (dragging it far from ext_base). The parasitic
                    // R in G_aug provides natural damping — let the NR handle it.
                    if internal_junctions && bp.has_parasitics() {
                        continue;
                    }

                    let nf_vt = bp.nf * bp.vt;
                    let nr_vt = bp.nr * bp.vt;
                    let vcrit_be = pn_vcrit(nf_vt, bp.is);
                    // Vbe dimension
                    let be_idx = slot.start_idx;
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    let v_old_be = sign * v_nl[be_idx];
                    let v_raw_be = sign * v_nl_new[be_idx];
                    let v_lim_be = pnjlim(v_raw_be, v_old_be, nf_vt, vcrit_be);
                    if (v_lim_be - v_raw_be).abs() > 1e-15 {
                        let correction = sign * (v_lim_be - v_raw_be);
                        for j in 0..n_dc {
                            pn_corrections[j] += dc_n_v[be_idx][j] * correction;
                        }
                    }
                    // Vbc dimension (if 2D)
                    if slot.dimension > 1 {
                        let bc_idx = slot.start_idx + 1;
                        let vcrit_bc = pn_vcrit(nr_vt, bp.is);
                        let v_old_bc = sign * v_nl[bc_idx];
                        let v_raw_bc = sign * v_nl_new[bc_idx];
                        let v_lim_bc = pnjlim(v_raw_bc, v_old_bc, nr_vt, vcrit_bc);
                        if (v_lim_bc - v_raw_bc).abs() > 1e-15 {
                            let correction = sign * (v_lim_bc - v_raw_bc);
                            for j in 0..n_dc {
                                pn_corrections[j] += dc_n_v[bc_idx][j] * correction;
                            }
                        }
                    }
                }
                _ => {} // FETs and tubes: no PN junction limiting needed
            }
        }

        // Apply pnjlim corrections to v_new
        let v_limited: Vec<f64> = v_new
            .iter()
            .zip(pn_corrections.iter())
            .map(|(&vn, &corr)| vn + corr)
            .collect();

        // Global node voltage damping: if max change > 10V, scale all node updates.
        // Includes both external circuit nodes (0..n) and internal parasitic nodes.
        let n = mna.n;
        let int_start = circuit.internal_node_start;
        let v_max_step = 50.0;
        let mut max_delta = 0.0_f64;
        // Compute max delta across circuit nodes AND internal nodes
        for i in 0..n {
            let delta = (v_limited[i] - v[i]).abs();
            max_delta = max_delta.max(delta);
        }
        for i in int_start..n_dc {
            let delta = (v_limited[i] - v[i]).abs();
            max_delta = max_delta.max(delta);
        }
        let damping = if max_delta > 10.0 {
            (10.0 / max_delta).max(0.1)
        } else {
            1.0
        };

        let mut final_max_delta = 0.0_f64;
        for i in 0..n_dc {
            let delta = v_limited[i] - v[i];
            let is_node = i < n || i >= int_start;
            let limited = if is_node {
                // Circuit node or internal parasitic node: apply damping + flat clamp
                let damped = delta * damping;
                damped.clamp(-v_max_step, v_max_step)
            } else {
                // Augmented variables (VS currents, inductor currents): apply directly
                delta
            };
            v[i] += limited;
            final_max_delta = final_max_delta.max(limited.abs());
        }
        let max_delta = final_max_delta;

        if max_delta < config.tolerance {
            // Final evaluation at converged point
            extract_nl_voltages_with(m, dc_n_v, v, v_nl);
            evaluate_devices_inner(v_nl, device_slots, i_nl, &mut j_dev, m, internal_junctions);
            return (true, iter + 1);
        }
    }

    // Final evaluation even on non-convergence
    extract_nl_voltages_with(m, dc_n_v, v, v_nl);
    evaluate_devices_inner(
        v_nl,
        device_slots,
        i_nl,
        &mut vec![0.0; m * m],
        m,
        internal_junctions,
    );
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
    let m = mna.m;

    // Build DC system with internal nodes for parasitic BJTs.
    // Dimension n_dc = n_aug + num_inductors + num_internal_nodes.
    let dc_sys = build_dc_system(mna, device_slots, config);
    let n_dc = dc_sys.n_dc;

    let has_mna_internal = !mna.bjt_internal_nodes.is_empty();
    if !dc_sys.bjt_internal.is_empty() {
        log::info!(
            "DC OP: {} parasitic BJTs expanded to internal nodes, n_dc={} (mna.n_aug={})",
            dc_sys.bjt_internal.len(),
            n_dc,
            mna.n_aug
        );
    }

    // Compute linear initial guess. The g_dc matrix already has Gmin floor
    // regularization for floating nodes, so this should not be singular.
    let v_linear = solve_linear(&dc_sys.g_dc, &dc_sys.b_dc).unwrap_or_else(|| vec![0.0; n_dc]);

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

    let has_internal_nodes = !dc_sys.bjt_internal.is_empty() || has_mna_internal;
    let circuit = DcCircuit {
        g_dc: &dc_sys.g_dc,
        b_dc: &dc_sys.b_dc,
        mna,
        device_slots,
        config,
        dc_n_v: &dc_sys.dc_n_v,
        dc_n_i: &dc_sys.dc_n_i,
        n_dc,
        has_internal_nodes,
        internal_node_start: if has_mna_internal {
            // MNA internal nodes start at the lowest internal node index
            mna.bjt_internal_nodes
                .iter()
                .flat_map(|n| {
                    [n.int_base, n.int_collector, n.int_emitter]
                        .into_iter()
                        .flatten()
                })
                .min()
                .unwrap_or(dc_sys.internal_node_start)
        } else {
            dc_sys.internal_node_start
        },
    };

    // Clamp junction voltages in the linear initial guess to prevent
    // extreme forward bias (which causes exp() saturation and NR divergence).
    // For multi-stage direct-coupled BJT circuits, the linear guess often has
    // junction voltages of several volts because the BJT is not conducting.
    let mut v_clamped = clamp_junction_voltages(mna, device_slots, &v_linear);
    // Extend to n_dc if internal nodes were added
    v_clamped.resize(n_dc, 0.0);
    // Initialize internal node voltages from clamped external nodes
    for bjt in &dc_sys.bjt_internal {
        if let Some(slot) = device_slots.iter().find(|s| s.start_idx == bjt.start_idx) {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.rb > 0.0 && bjt.ext_base > 0 {
                    v_clamped[bjt.int_base] = v_clamped[bjt.ext_base - 1];
                }
                if bp.rc > 0.0 && bjt.ext_collector > 0 {
                    v_clamped[bjt.int_collector] = v_clamped[bjt.ext_collector - 1];
                }
                if bp.re > 0.0 && bjt.ext_emitter > 0 {
                    let base_v = if bp.rb > 0.0 {
                        v_clamped[bjt.int_base]
                    } else if bjt.ext_base > 0 {
                        v_clamped[bjt.ext_base - 1]
                    } else {
                        0.0
                    };
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    v_clamped[bjt.int_emitter] = base_v - sign * 0.65;
                }
            }
        }
    }

    // Also initialize MNA-level internal nodes (from expand_bjt_internal_nodes)
    for mna_bjt in &mna.bjt_internal_nodes {
        if let Some(slot) = device_slots
            .iter()
            .find(|s| s.start_idx == mna_bjt.start_idx)
        {
            if let DeviceParams::Bjt(bp) = &slot.params {
                // Find external node indices from MNA nonlinear device info
                let dev_info = mna
                    .nonlinear_devices
                    .iter()
                    .find(|d| d.start_idx == mna_bjt.start_idx);
                if let Some(dev) = dev_info {
                    let ext_c = dev.node_indices[0]; // 1-indexed
                    let ext_b = dev.node_indices[1];
                    let ext_e = dev.node_indices[2];

                    if let Some(ib) = mna_bjt.int_base {
                        if ext_b > 0 && ib < v_clamped.len() {
                            v_clamped[ib] = v_clamped[ext_b - 1];
                        }
                    }
                    if let Some(ic) = mna_bjt.int_collector {
                        if ext_c > 0 && ic < v_clamped.len() {
                            v_clamped[ic] = v_clamped[ext_c - 1];
                        }
                    }
                    if let Some(ie) = mna_bjt.int_emitter {
                        if ext_e > 0 && ie < v_clamped.len() {
                            let base_v = if let Some(ib) = mna_bjt.int_base {
                                v_clamped[ib]
                            } else if ext_b > 0 {
                                v_clamped[ext_b - 1]
                            } else {
                                0.0
                            };
                            let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                            v_clamped[ie] = base_v - sign * 0.65;
                        }
                    }
                }
            }
        }
    }

    let mut v = v_clamped.clone();
    let mut v_nl = vec![0.0; m];
    let mut i_nl = vec![0.0; m];

    // Strategy 1: Direct NR from junction-clamped linear guess
    let (converged, iters) = nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, 0.0);
    // Sanity check: verify the solution isn't degenerate (all nodes at supply rails).
    // A degenerate solution has all BJTs off — check that at least one junction is forward-biased.
    let mut has_active_junction = m == 0; // linear circuits are always OK
    if converged && m > 0 {
        for slot in device_slots {
            match &slot.params {
                DeviceParams::Bjt(bp) => {
                    let vbe = v_nl[slot.start_idx];
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    // Forward-biased junction: |Vbe| > 0.3V
                    if (sign * vbe) > 0.3 {
                        has_active_junction = true;
                        break;
                    }
                }
                DeviceParams::Diode(_) => {
                    if v_nl[slot.start_idx].abs() > 0.3 {
                        has_active_junction = true;
                        break;
                    }
                }
                _ => {
                    has_active_junction = true;
                    break;
                } // non-BJT devices: trust the result
            }
        }
    }
    log::info!(
        "DC OP Strategy 1 (Direct NR): converged={} iters={} active_junctions={}",
        converged,
        iters,
        has_active_junction
    );
    if converged && has_active_junction {
        // Return only the MNA-sized portion (truncate internal nodes)
        v.truncate(n_dc);
        return DcOpResult {
            v_node: v,
            v_nl,
            i_nl,
            converged: true,
            method: DcOpMethod::DirectNr,
            iterations: iters,
        };
    }

    // Strategy 2: Source stepping (two attempts)
    // Attempt A: Start from v=0, ramp sources from 0 to full.
    // Attempt B: Start from clamped linear guess (pre-biased junctions).
    //
    // For feedback amplifiers, starting from v=0 often converges to the degenerate
    // "all off" solution because the feedback loop is open. Starting from the clamped
    // guess pre-biases junctions so the NR maintains active operation as sources ramp.
    let ss_starts: [Vec<f64>; 2] = [
        vec![0.0; n_dc],   // Classic source stepping from zero
        v_clamped.clone(), // Source stepping from clamped guess
    ];

    let mut v = vec![0.0; n_dc];
    let mut total_iters = 0;
    let mut source_stepping_converged = false;

    for (attempt, start) in ss_starts.iter().enumerate() {
        v.clone_from(start);
        let mut this_iters = 0;
        let mut this_converged = true;

        for step in 1..=config.source_steps {
            let scale = step as f64 / config.source_steps as f64;
            let (converged, iters) =
                nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, scale, 0.0);
            this_iters += iters;
            if !converged {
                this_converged = false;
                break;
            }
        }
        total_iters += this_iters;

        if this_converged {
            // Check for degenerate solution
            extract_nl_voltages_with(m, &dc_sys.dc_n_v, &v, &mut v_nl);
            let mut has_active = m == 0;
            for slot in device_slots {
                if let DeviceParams::Bjt(bp) = &slot.params {
                    if slot.start_idx < v_nl.len() {
                        let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                        if (sign * v_nl[slot.start_idx]) > 0.3 {
                            has_active = true;
                            break;
                        }
                    }
                } else {
                    has_active = true;
                    break;
                }
            }
            if has_active {
                log::info!("DC OP Strategy 2 (Source stepping attempt {}, {} steps): converged, active, iters={}",
                    attempt, config.source_steps, this_iters);
                source_stepping_converged = true;
                break;
            }
            log::info!(
                "DC OP Strategy 2 attempt {}: converged but degenerate",
                attempt
            );
        }
    }

    log::info!(
        "DC OP Strategy 2 (Source stepping): converged={} iters={}",
        source_stepping_converged,
        total_iters
    );
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
    // Start Gmin from the clamped guess (better for BJT circuits than raw linear).
    // Source stepping may have found a degenerate solution; Gmin from the clamped
    // guess has a better chance of finding the active OP because the junction
    // voltages are pre-set to reasonable values.
    let mut v = clamp_junction_voltages(mna, device_slots, &v_linear);
    v.resize(n_dc, 0.0);
    // Initialize internal nodes for Gmin stepping too
    for bjt in &dc_sys.bjt_internal {
        if let Some(slot) = device_slots.iter().find(|s| s.start_idx == bjt.start_idx) {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.rb > 0.0 && bjt.ext_base > 0 {
                    v[bjt.int_base] = v[bjt.ext_base - 1];
                }
                if bp.rc > 0.0 && bjt.ext_collector > 0 {
                    v[bjt.int_collector] = v[bjt.ext_collector - 1];
                }
                if bp.re > 0.0 && bjt.ext_emitter > 0 {
                    let base_v = if bp.rb > 0.0 {
                        v[bjt.int_base]
                    } else if bjt.ext_base > 0 {
                        v[bjt.ext_base - 1]
                    } else {
                        0.0
                    };
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    v[bjt.int_emitter] = base_v - sign * 0.65;
                }
            }
        }
    }
    // Also init MNA-level internal nodes for Gmin stepping
    for mna_bjt in &mna.bjt_internal_nodes {
        if let Some(slot) = device_slots
            .iter()
            .find(|s| s.start_idx == mna_bjt.start_idx)
        {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if let Some(dev) = mna
                    .nonlinear_devices
                    .iter()
                    .find(|d| d.start_idx == mna_bjt.start_idx)
                {
                    let (ext_c, ext_b, ext_e) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    if let Some(ib) = mna_bjt.int_base {
                        if ext_b > 0 && ib < v.len() {
                            v[ib] = v[ext_b - 1];
                        }
                    }
                    if let Some(ic) = mna_bjt.int_collector {
                        if ext_c > 0 && ic < v.len() {
                            v[ic] = v[ext_c - 1];
                        }
                    }
                    if let Some(ie) = mna_bjt.int_emitter {
                        if ext_e > 0 && ie < v.len() {
                            let base_v = mna_bjt.int_base.map(|ib| v[ib]).unwrap_or_else(|| {
                                if ext_b > 0 {
                                    v[ext_b - 1]
                                } else {
                                    0.0
                                }
                            });
                            let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                            v[ie] = base_v - sign * 0.65;
                        }
                    }
                }
            }
        }
    }
    let mut total_iters = 0;
    let mut gmin_converged = true;

    // Logarithmic ramp from gmin_start to gmin_end
    let log_start = config.gmin_start.ln();
    let log_end = config.gmin_end.ln();

    for step in 0..=config.gmin_steps {
        let frac = step as f64 / config.gmin_steps as f64;
        let gmin = (log_start + frac * (log_end - log_start)).exp();

        let (converged, iters) = nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, gmin);
        total_iters += iters;
        if !converged {
            gmin_converged = false;
            break;
        }
    }

    log::info!(
        "DC OP Strategy 3 (Gmin stepping): converged={} iters={}",
        gmin_converged,
        total_iters
    );
    // Final solve without Gmin
    if gmin_converged {
        let (converged, iters) = nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, 0.0);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device_types::{BjtParams, DiodeParams};

    // ── LU decomposition unit tests ──────────────────────────────────

    #[test]
    fn test_lu_decompose_identity() {
        let a = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let (lu, pivot) = lu_decompose(&a).unwrap();
        // U should be identity (L is identity with 1s on diagonal)
        for i in 0..3 {
            assert_eq!(pivot[i], i, "pivot should be identity permutation");
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (lu[i][j] - expected).abs() < 1e-15,
                    "LU[{i}][{j}] = {}, expected {expected}",
                    lu[i][j]
                );
            }
        }
    }

    #[test]
    fn test_lu_decompose_known_3x3() {
        // A = [[2, 1, 1], [4, 3, 3], [8, 7, 9]]
        // Hand-computed: P*A = L*U where
        //   P swaps row 0 and row 2 (pivot on 8)
        //   After elimination: well-defined L and U
        let a = vec![
            vec![2.0, 1.0, 1.0],
            vec![4.0, 3.0, 3.0],
            vec![8.0, 7.0, 9.0],
        ];
        let (lu, pivot) = lu_decompose(&a).unwrap();

        // Verify PA = LU by reconstructing and solving
        let b = vec![1.0, 2.0, 3.0];
        let x = lu_solve(&lu, &pivot, &b);
        // Verify Ax = b
        for i in 0..3 {
            let row_sum: f64 = (0..3).map(|j| a[i][j] * x[j]).sum();
            assert!(
                (row_sum - b[i]).abs() < 1e-12,
                "row {i}: Ax={row_sum}, b={}",
                b[i]
            );
        }
    }

    #[test]
    fn test_lu_solve_known_system() {
        // 3x3 system with known solution: x = [1, 2, 3]
        // A = [[1, 2, 3], [0, 1, 4], [5, 6, 0]]
        // b = A * [1, 2, 3] = [1+4+9, 0+2+12, 5+12+0] = [14, 14, 17]
        let a = vec![
            vec![1.0, 2.0, 3.0],
            vec![0.0, 1.0, 4.0],
            vec![5.0, 6.0, 0.0],
        ];
        let x_expected = vec![1.0, 2.0, 3.0];
        let b: Vec<f64> = (0..3)
            .map(|i| (0..3).map(|j| a[i][j] * x_expected[j]).sum())
            .collect();

        let x = solve_linear(&a, &b).unwrap();
        for i in 0..3 {
            assert!(
                (x[i] - x_expected[i]).abs() < 1e-12,
                "x[{i}] = {}, expected {}",
                x[i],
                x_expected[i]
            );
        }
    }

    #[test]
    fn test_lu_solve_roundtrip_sizes() {
        // Test roundtrip for 2x2, 4x4, 6x6 matrices
        for n in [2, 4, 6] {
            // Diagonally dominant matrix (guaranteed nonsingular)
            let mut a = vec![vec![0.0; n]; n];
            for i in 0..n {
                a[i][i] = (n as f64) * 10.0;
                for j in 0..n {
                    if i != j {
                        a[i][j] = ((i + j) as f64) * 0.1;
                    }
                }
            }
            // Known solution
            let x_known: Vec<f64> = (0..n).map(|i| (i + 1) as f64).collect();
            let b: Vec<f64> = (0..n)
                .map(|i| (0..n).map(|j| a[i][j] * x_known[j]).sum())
                .collect();

            let x = solve_linear(&a, &b).unwrap();
            for i in 0..n {
                assert!(
                    (x[i] - x_known[i]).abs() < 1e-10,
                    "n={n}, x[{i}] = {}, expected {}",
                    x[i],
                    x_known[i]
                );
            }
        }
    }

    #[test]
    fn test_lu_decompose_pivoting() {
        // Zero on diagonal — requires pivoting
        let a = vec![vec![0.0, 1.0], vec![1.0, 0.0]];
        let (lu, pivot) = lu_decompose(&a).unwrap();
        // Should swap rows
        assert_ne!(pivot[0], 0, "pivoting should have occurred");

        // Verify solve works
        let b = vec![3.0, 5.0];
        let x = lu_solve(&lu, &pivot, &b);
        // A * x = b: 0*x0 + 1*x1 = 3, 1*x0 + 0*x1 = 5 => x = [5, 3]
        assert!((x[0] - 5.0).abs() < 1e-14);
        assert!((x[1] - 3.0).abs() < 1e-14);
    }

    #[test]
    fn test_lu_decompose_singular_returns_none() {
        let a = vec![vec![1.0, 2.0], vec![2.0, 4.0]]; // rank 1
        assert!(
            lu_decompose(&a).is_none(),
            "singular matrix should return None"
        );
    }

    #[test]
    fn test_lu_decompose_near_singular() {
        // Condition number ~ 1e8 but NOT singular
        let eps = 1e-8;
        let a = vec![vec![1.0, 1.0], vec![1.0, 1.0 + eps]];
        let result = lu_decompose(&a);
        assert!(result.is_some(), "near-singular but not singular");

        let b = vec![2.0, 2.0 + eps];
        let x = solve_linear(&a, &b).unwrap();
        // Exact solution: x = [1, 1]
        // With condition number ~1e8, accuracy degrades but shouldn't NaN
        assert!(x[0].is_finite(), "solution should be finite");
        assert!(x[1].is_finite(), "solution should be finite");
    }

    #[test]
    fn test_lu_solve_preserves_rhs() {
        // Verify the function doesn't have aliasing bugs with b
        let a = vec![
            vec![4.0, -1.0, 0.0],
            vec![-1.0, 4.0, -1.0],
            vec![0.0, -1.0, 4.0],
        ];
        let b = vec![1.0, 5.0, 3.0];
        let b_copy = b.clone();
        let _x = solve_linear(&a, &b).unwrap();
        assert_eq!(b, b_copy, "b should not be modified");
    }

    // ── evaluate_devices unit tests ──────────────────────────────────

    #[test]
    fn test_evaluate_devices_diode_isolated() {
        let vt = 0.025851991;
        let is = 2.52e-9;
        let n = 1.752;
        let n_vt = n * vt;

        let slot = DeviceSlot {
            device_type: DeviceType::Diode,
            start_idx: 0,
            dimension: 1,
            params: DeviceParams::Diode(DiodeParams {
                is,
                n_vt,
                cjo: 0.0,
                rs: 0.0,
                bv: f64::INFINITY,
                ibv: 1e-10,
            }),
            has_internal_mna_nodes: false,
        };

        let m = 1;
        let v_nl = vec![0.6]; // Forward biased
        let mut i_nl = vec![0.0];
        let mut j_dev = vec![0.0];

        evaluate_devices(&v_nl, &[slot], &mut i_nl, &mut j_dev, m);

        // Compare against DiodeShockley directly
        let diode = DiodeShockley::new(is, 1.0, n_vt);
        let expected_i = diode.current_at(0.6);
        let expected_g = diode.conductance_at(0.6);

        assert!(
            (i_nl[0] - expected_i).abs() < 1e-15,
            "current mismatch: {} vs {}",
            i_nl[0],
            expected_i
        );
        assert!(
            (j_dev[0] - expected_g).abs() < 1e-15,
            "conductance mismatch: {} vs {}",
            j_dev[0],
            expected_g
        );
    }

    #[test]
    fn test_evaluate_devices_bjt_isolated() {
        let is = 1e-14;
        let vt = 0.025851991;

        let slot = DeviceSlot {
            device_type: DeviceType::Bjt,
            start_idx: 0,
            dimension: 2,
            params: DeviceParams::Bjt(BjtParams {
                is,
                vt,
                beta_f: 200.0,
                beta_r: 3.0,
                is_pnp: false,
                vaf: f64::INFINITY,
                var: f64::INFINITY,
                ikf: f64::INFINITY,
                ikr: f64::INFINITY,
                cje: 0.0,
                cjc: 0.0,
                nf: 1.0,
                ise: 0.0,
                ne: 1.5,
                nr: 1.0,
                isc: 0.0,
                nc: 2.0,
                rb: 0.0,
                rc: 0.0,
                re: 0.0,
                rth: f64::INFINITY,
                cth: 1e-3,
                xti: 3.0,
                eg: 1.11,
                tamb: 300.15,
            }),
            has_internal_mna_nodes: false,
        };

        let m = 2;
        let v_nl = vec![0.65, -5.0]; // Forward active: Vbe=0.65V, Vbc=-5V
        let mut i_nl = vec![0.0; 2];
        let mut j_dev = vec![0.0; 4];

        evaluate_devices(&v_nl, &[slot], &mut i_nl, &mut j_dev, m);

        // Compare against BjtEbersMoll directly
        let em = BjtEbersMoll::new(is, vt, 200.0, 3.0, BjtPolarity::Npn);
        let expected_ic = em.collector_current(0.65, -5.0);
        let expected_ib = em.base_current(0.65, -5.0);

        assert!(
            (i_nl[0] - expected_ic).abs() / expected_ic.abs().max(1e-20) < 1e-10,
            "Ic mismatch: {} vs {}",
            i_nl[0],
            expected_ic
        );
        assert!(
            (i_nl[1] - expected_ib).abs() / expected_ib.abs().max(1e-20) < 1e-10,
            "Ib mismatch: {} vs {}",
            i_nl[1],
            expected_ib
        );
        // Jacobian should be populated (4 entries for 2x2)
        assert!(j_dev[0].abs() > 0.0, "dIc/dVbe should be nonzero");
    }

    #[test]
    fn test_evaluate_devices_zero_voltage() {
        // At v_nl = 0 for all devices, currents should be near zero
        let diode_slot = DeviceSlot {
            device_type: DeviceType::Diode,
            start_idx: 0,
            dimension: 1,
            params: DeviceParams::Diode(DiodeParams {
                is: 1e-14,
                n_vt: 0.025851991,
                cjo: 0.0,
                rs: 0.0,
                bv: f64::INFINITY,
                ibv: 1e-10,
            }),
            has_internal_mna_nodes: false,
        };

        let m = 1;
        let v_nl = vec![0.0];
        let mut i_nl = vec![0.0];
        let mut j_dev = vec![0.0];

        evaluate_devices(&v_nl, &[diode_slot], &mut i_nl, &mut j_dev, m);

        // At V=0, I = Is*(exp(0) - 1) = 0
        assert!(
            i_nl[0].abs() < 1e-20,
            "diode current at 0V should be ~0, got {}",
            i_nl[0]
        );
        // Conductance should be Is/nVt (small but nonzero)
        assert!(j_dev[0] > 0.0, "conductance at 0V should be positive");
    }

    #[test]
    fn test_evaluate_devices_multiple_no_crosstalk() {
        let m = 3; // 2 diodes (1D each) + 1 more (but we'll just use 3 diodes)
        let slots = vec![
            DeviceSlot {
                device_type: DeviceType::Diode,
                start_idx: 0,
                dimension: 1,
                params: DeviceParams::Diode(DiodeParams {
                    is: 1e-14,
                    n_vt: 0.025851991,
                    cjo: 0.0,
                    rs: 0.0,
                    bv: f64::INFINITY,
                    ibv: 1e-10,
                }),
                has_internal_mna_nodes: false,
            },
            DeviceSlot {
                device_type: DeviceType::Diode,
                start_idx: 1,
                dimension: 1,
                params: DeviceParams::Diode(DiodeParams {
                    is: 1e-12, // Different IS
                    n_vt: 0.045,
                    cjo: 0.0,
                    rs: 0.0,
                    bv: f64::INFINITY,
                    ibv: 1e-10,
                }),
                has_internal_mna_nodes: false,
            },
            DeviceSlot {
                device_type: DeviceType::Diode,
                start_idx: 2,
                dimension: 1,
                params: DeviceParams::Diode(DiodeParams {
                    is: 5e-9,
                    n_vt: 0.025851991 * 1.752,
                    cjo: 0.0,
                    rs: 0.0,
                    bv: f64::INFINITY,
                    ibv: 1e-10,
                }),
                has_internal_mna_nodes: false,
            },
        ];

        let v_nl = vec![0.6, 0.3, 0.5];
        let mut i_nl = vec![0.0; 3];
        let mut j_dev = vec![0.0; 9]; // 3x3

        evaluate_devices(&v_nl, &slots, &mut i_nl, &mut j_dev, m);

        // Check that off-diagonal Jacobian entries are zero (no crosstalk)
        for i in 0..3 {
            for j in 0..3 {
                if i != j {
                    assert!(
                        j_dev[i * m + j].abs() < 1e-30,
                        "j_dev[{i}][{j}] should be 0 (no crosstalk), got {}",
                        j_dev[i * m + j]
                    );
                }
            }
        }

        // Each device's current should match independent evaluation
        for (idx, slot) in slots.iter().enumerate() {
            let mut i_single = vec![0.0; m];
            let mut j_single = vec![0.0; m * m];
            evaluate_devices(&v_nl, &[slot.clone()], &mut i_single, &mut j_single, m);
            assert!(
                (i_nl[idx] - i_single[idx]).abs() < 1e-20,
                "device {idx}: current differs when evaluated together vs alone"
            );
        }
    }
}
