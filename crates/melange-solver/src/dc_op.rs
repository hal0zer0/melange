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
use melange_devices::diode::{DiodeShockley, DiodeWithRs};
use melange_devices::tube::{KorenPentode, KorenTriode};
use melange_primitives::nr::{fetlim, pn_vcrit, pnjlim};

/// Configuration for the DC operating point solver.
#[derive(Debug, Clone)]
pub struct DcOpConfig {
    /// Absolute convergence tolerance (V). Acts as ABSTOL in the SPICE-style
    /// per-variable check `|delta_i| < reltol·|v_i| + tolerance`.
    pub tolerance: f64,
    /// Relative convergence tolerance (dimensionless). DC OP uses a much
    /// tighter RELTOL than the transient path's 1e-3: at 1e-6 low-voltage
    /// circuits keep (effectively) the legacy absolute-only behavior while
    /// high-voltage circuits (300 V rails) stop false-failing on the LU
    /// round-off floor, which is proportional to |v|.
    pub reltol: f64,
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
    /// Input node index (0-indexed into N-vector) of the primary input port.
    pub input_node: usize,
    /// Input resistance (ohms) of the primary input port.
    pub input_resistance: f64,
    /// Extra input ports for multi-input (M=0) circuits: `(node_index,
    /// resistance)` for each port beyond the primary. Empty for single-input.
    /// Each is stamped with its own `1/R` Thevenin conductance in the DC-OP
    /// G matrix, matching the per-sample input stamping. See
    /// `local-docs/multi-input-ports-plan.md`.
    pub extra_inputs: Vec<(usize, f64)>,
}

impl Default for DcOpConfig {
    fn default() -> Self {
        Self {
            tolerance: 1e-9,
            reltol: 1e-6,
            max_iterations: 200,
            source_steps: 50,
            gmin_start: 1e-2,
            gmin_end: 1e-12,
            gmin_steps: 10,
            input_node: 0,
            input_resistance: 1.0,
            extra_inputs: Vec::new(),
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
    /// AOL continuation stepping was needed (precision rectifier multi-equilibrium)
    AolStepping,
    /// All methods failed — returned linear fallback
    Failed,
    /// The linear DC system itself was singular — returned zeros, NOT converged.
    /// Callers should treat this like `Failed` (warmup settling required), but the
    /// distinct variant lets them tell "NR could not converge" apart from
    /// "the G matrix is structurally broken".
    SingularLinear,
}

/// Implausibility bounds for a *finite* DC OP result. A NaN/Inf iterate is
/// already caught by the `is_finite()` guards throughout this module; these
/// bounds catch the gap where a solve saturates (e.g. the codegen exp()
/// clamp at ±40, or a near-singular — not exactly singular — linear solve)
/// and returns an astronomically large but technically finite value.
///
/// Justified against the melange-circuits corpus (`grep -rhoE 'DC [0-9.]+'`
/// on shipped decks): the highest real supply rail is 480 V (EL34/6L6
/// plate supplies); the smallest series resistor is 1 mΩ. A dead short
/// across the highest rail through the smallest resistor — a fault
/// condition, not anything melange's solver should produce as a *bias
/// point* — is ~4.8e5 A. `DC_OP_MAX_PLAUSIBLE_VOLTAGE` (1e6 V) and
/// `DC_OP_MAX_PLAUSIBLE_CURRENT` (1e7 A) sit ~2000x and ~20x above those
/// extremes respectively: comfortably clear of any legitimate circuit
/// (including 250 V+ tube plate supplies) while still orders of magnitude
/// below an observed runaway (~1e8 A / 1e11 mA on a real Ge PNP repro; up
/// to ~1e300 on a trap-unstable BJT divider — see
/// dc_op_ge_active_junction_threshold.md and
/// dk_backward_euler_ignored_trap_unstable.md memory notes).
const DC_OP_MAX_PLAUSIBLE_VOLTAGE: f64 = 1e6;
const DC_OP_MAX_PLAUSIBLE_CURRENT: f64 = 1e7;

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
    evaluate_devices_inner(v_nl, device_slots, i_nl, j_dev, m, false, None);
}

/// Like [`evaluate_devices`], but with the node-voltage vector available so
/// node-referenced device corrections (currently the MOSFET body effect,
/// which needs `Vsb = V(source) − V(bulk)`) can be applied. `v_node` is the
/// N-dimensional (or n_dc-dimensional) node vector; 1-based MNA node index
/// `i` maps to `v_node[i - 1]`, index 0 is ground.
pub fn evaluate_devices_with_nodes(
    v_nl: &[f64],
    device_slots: &[DeviceSlot],
    i_nl: &mut [f64],
    j_dev: &mut [f64],
    m: usize,
    v_node: &[f64],
) {
    evaluate_devices_inner(v_nl, device_slots, i_nl, j_dev, m, false, Some(v_node));
}

/// Like `evaluate_devices`, but when `internal_junctions` is true, skip the
/// parasitic inner loop because v_nl already represents internal junction
/// voltages (from DC system with expanded internal nodes).
///
/// `v_node` (when available) is the current node-voltage iterate, used for
/// device corrections that reference node voltages beyond the M controlling
/// voltages (MOSFET body effect). `None` falls back to nominal parameters,
/// which is exact whenever Vsb = 0.
fn evaluate_devices_inner(
    v_nl: &[f64],
    device_slots: &[DeviceSlot],
    i_nl: &mut [f64],
    j_dev: &mut [f64],
    m: usize,
    internal_junctions: bool,
    v_node: Option<&[f64]>,
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
                if dp.has_rs() {
                    // Series resistance: the transient runtime solves the
                    // junction voltage V_j from V_j + I(V_j)·RS = V (see
                    // diode_current_with_rs in device_diode.rs.tera), so the
                    // DC OP must use the same I-V law or the baked bias point
                    // and DC_NL_I disagree with the transient fixed point.
                    // DiodeWithRs is the canonical devices-crate form of that
                    // same junction solve (pnjlim-limited scalar NR).
                    let drs = DiodeWithRs::new(diode, dp.rs);
                    i_nl[s] = drs.current_at(v);
                    j_dev[s * m + s] = drs.conductance_at(v);
                } else {
                    i_nl[s] = diode.current_at(v);
                    j_dev[s * m + s] = diode.conductance_at(v);
                }
                // Reverse breakdown: evaluated at the TERMINAL voltage, not
                // the RS-adjusted junction voltage — this mirrors the runtime
                // composition exactly (nr_helpers.rs emits
                // `diode_current_with_rs(v) + diode_breakdown_current(v)`).
                // Reverse breakdown: I_bv = -IBV * exp(-(v + BV) / n_vt)
                // Matches codegen template (device_diode.rs.tera:57-70)
                if dp.has_bv() {
                    let x = (-(v + dp.bv) / dp.n_vt).clamp(-40.0, 40.0);
                    let exp_x = x.exp();
                    i_nl[s] += -dp.ibv * exp_x;
                    j_dev[s * m + s] += dp.ibv / dp.n_vt * exp_x;
                }
                // Device-level Gmin: minimum junction conductance (1 TΩ).
                // Standard SPICE practice (ngspice GMIN default = 1e-12 S).
                // Without this, reverse-biased diode conductance ≈ 1e-25 S
                // and NR Jacobian entries are effectively zero, preventing
                // convergence in precision rectifier circuits.
                const DIODE_GMIN: f64 = 1e-12;
                i_nl[s] += DIODE_GMIN * v;
                j_dev[s * m + s] += DIODE_GMIN;
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
                // Leakage diodes (ISE/NE, ISC/NC): the transient runtime's
                // `bjt_evaluate` (device_bjt.rs.tera) always includes these
                // terms in Ib and its Jacobian. The DC OP must carry the same
                // params or the bias point (and the baked DC_NL_I seeds)
                // converge to a different fixed point than the transient —
                // for a 2N3904 at Vbe≈0.65 the leakage term exceeds the ideal
                // Ib, which is volts of error through MΩ-class bias networks.
                let em = BjtEbersMoll::new(bp.is, bp.vt, bp.beta_f, bp.beta_r, polarity)
                    .with_nf(bp.nf)
                    .with_nr(bp.nr)
                    .with_leakage(bp.ise, bp.ne, bp.isc, bp.nc);
                // Gummel-Poon wraps the SAME leakage-carrying Ebers-Moll core:
                // GP modulates only the transport (collector) current by qb;
                // Ib and its Jacobian delegate to the base Ebers-Moll model —
                // exactly the composition of the runtime `bjt_evaluate`.
                let gp = if bp.is_gummel_poon() {
                    Some(BjtGummelPoon::new(em, bp.vaf, bp.var, bp.ikf, bp.ikr))
                } else {
                    None
                };
                let vbe = v_nl[s];
                let vbc = v_nl[s + 1];

                // If the BJT has parasitic resistances and we're NOT using
                // internal junction nodes, solve for the internal junction
                // voltages with the same damped 2D Newton the generated
                // runtime uses (`bjt_with_parasitics`). When internal_junctions
                // is true, v_nl already contains the correct internal voltages
                // from dc_n_v.
                let (ic, ib, jac) = if bp.has_parasitics() && !internal_junctions {
                    bjt_with_parasitics_dc(&em, gp.as_ref(), vbe, vbc, bp.rb, bp.rc, bp.re, bp.vt)
                } else {
                    bjt_eval_intrinsic(&em, gp.as_ref(), vbe, vbc)
                };
                i_nl[s] = ic;
                i_nl[s + 1] = ib;
                j_dev[s * m + s] = jac[0]; // dIc/dVbe
                j_dev[s * m + (s + 1)] = jac[1]; // dIc/dVbc
                j_dev[(s + 1) * m + s] = jac[2]; // dIb/dVbe
                j_dev[(s + 1) * m + (s + 1)] = jac[3]; // dIb/dVbc
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
                                                        // Ig ≡ 0: the transient runtime's `jfet_ig` returns exactly 0
                                                        // (device_jfet.rs.tera) with zero partials. Evaluating a real
                                                        // exponential gate diode here while zeroing its Jacobian gives
                                                        // an exponential residual with no Newton direction (wrecks DC
                                                        // NR at forward gate bias) AND disagrees with the transient
                                                        // contract. DC OP must not lead the runtime — real gate-
                                                        // junction wiring is a transient-side follow-up.
                i_nl[s + 1] = 0.0; // Ig (dim 1 current)
                let (gm, gds) = jfet.jacobian_partial(vgs, vds);
                j_dev[s * m + s] = gds; // dId/dVds (dim0 curr, dim0 volt)
                j_dev[s * m + (s + 1)] = gm; // dId/dVgs (dim0 curr, dim1 volt)
                j_dev[(s + 1) * m + s] = 0.0; // dIg/dVds = 0
                j_dev[(s + 1) * m + (s + 1)] = 0.0; // dIg/dVgs = 0
            }
            (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                // 2D MOSFET: dim 0 = Vds (at start_idx), dim 1 = Vgs (at start_idx+1)
                let channel = if mp.is_p_channel {
                    melange_devices::mosfet::ChannelType::P
                } else {
                    melange_devices::mosfet::ChannelType::N
                };
                // Body effect (GAMMA/PHI): the DC OP must evaluate the SAME
                // device the transient runtime evaluates. Both generated
                // paths adjust VT from Vsb per iteration/sample:
                //   dk_emitter.rs ~L2200 (body_effect_update, from v_pred)
                //   nodal_emitter.rs ~L7325/L3806 (vt_eff, each NR iteration)
                // with the channel-signed magnitude-space formula
                //   vsb    = sign · (V(source) − V(bulk))     (sign = −1 PMOS)
                //   vt_eff = VT + sign · GAMMA · (√(PHI + max(vsb,0)) − √PHI)
                // so reverse body bias always increases |VT| (VT stays signed;
                // PMOS VT < 0). Keep this in lockstep with both emitter sites.
                //
                // source_node/bulk_node are 1-based MNA node indices resolved
                // by CircuitIR::resolve_mosfet_nodes (0 = ground / unresolved,
                // reading 0 V). When no node vector is available (legacy
                // `evaluate_devices` callers), fall back to nominal VT — the
                // exact result whenever Vsb = 0.
                let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                let vt_eff = match (mp.has_body_effect(), v_node) {
                    (true, Some(v)) => {
                        let vs = if mp.source_node > 0 {
                            v.get(mp.source_node - 1).copied().unwrap_or(0.0)
                        } else {
                            0.0
                        };
                        let vb = if mp.bulk_node > 0 {
                            v.get(mp.bulk_node - 1).copied().unwrap_or(0.0)
                        } else {
                            0.0
                        };
                        let vsb = sign * (vs - vb);
                        mp.vt + sign * mp.gamma * ((mp.phi + vsb.max(0.0)).sqrt() - mp.phi.sqrt())
                    }
                    _ => mp.vt,
                };
                let mos = melange_devices::mosfet::Mosfet::new(channel, vt_eff, mp.kp, mp.lambda);
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
                            crate::device_types::ScreenForm::Classical => {
                                melange_devices::tube::ScreenForm::Classical
                            }
                        },
                        // Phase 1c: variable-mu §5 fields pass through. When
                        // tp.svar == 0.0, the math reduces to sharp single-mu
                        // (byte-identical to phase 1a/1a.1 behavior).
                        mu_b: tp.mu_b,
                        svar: tp.svar,
                        ex_b: tp.ex_b,
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
                TubeKind::SharpPentodeGridOff => {
                    // Phase 1b reduced pentode: 2D NR block (Vgk → Ip, Vpk → Ig2).
                    // Vg2k is frozen at the DC-OP-converged value stored in
                    // `slot.vg2k_frozen`; Ig1 is dropped entirely (identically 0
                    // in grid-cutoff). The 2×2 Jacobian is the upper-left 2×2
                    // submatrix of the full 3×3, since d/dVg2k rows/cols are
                    // absent when Vg2k is not an NR unknown.
                    if s + 1 >= v_nl.len() {
                        log::warn!(
                            "DC OP: Grid-off pentode at start_idx={} needs 2 dims but v_nl.len()={}",
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
                        screen_form: match tp.screen_form {
                            crate::device_types::ScreenForm::Rational => {
                                melange_devices::tube::ScreenForm::Rational
                            }
                            crate::device_types::ScreenForm::Exponential => {
                                melange_devices::tube::ScreenForm::Exponential
                            }
                            crate::device_types::ScreenForm::Classical => {
                                melange_devices::tube::ScreenForm::Classical
                            }
                        },
                        mu_b: tp.mu_b,
                        svar: tp.svar,
                        ex_b: tp.ex_b,
                    };
                    let vgk = v_nl[s];
                    let vpk = v_nl[s + 1];
                    let vg2k = slot.vg2k_frozen;

                    // Currents: Ip and Ig2 only (Ig1 dropped).
                    i_nl[s] = pentode.plate_current(vgk, vpk, vg2k);
                    i_nl[s + 1] = pentode.screen_current(vgk, vpk, vg2k);

                    // 2×2 Jacobian: upper-left submatrix of the full 3×3.
                    // d/dVg2k entries are dropped along with the Vg2k NR dim.
                    // The math agent (task P1b-02) will add explicit
                    // `jacobian_2x2_grid_off` wrappers; for now the DC-OP path
                    // just reuses `jacobian_3x3` and discards the third row/col.
                    let jac = pentode.jacobian_3x3(vgk, vpk, vg2k);
                    j_dev[s * m + s] = jac[0][0]; // dIp/dVgk
                    j_dev[s * m + (s + 1)] = jac[0][1]; // dIp/dVpk
                    j_dev[(s + 1) * m + s] = jac[1][0]; // dIg2/dVgk
                    j_dev[(s + 1) * m + (s + 1)] = jac[1][1]; // dIg2/dVpk
                }
            },
            (DeviceType::Vca, DeviceParams::Vca(vp)) => {
                // 2D VCA: dim 0 = V_signal (at start_idx), dim 1 = V_control (at start_idx+1)
                let v_sig = v_nl[s];
                let v_ctrl = v_nl[s + 1];
                // Carry THD: the generated runtime evaluates the VCA with
                // DEVICE_THD (gain-dependent cubic distortion), so the DC OP
                // must too — `Vca::new` would silently zero it and bias any
                // DC-offset signal port away from the runtime's fixed point.
                let vca = melange_devices::Vca::new_with_thd(vp.vscale, vp.g0, vp.thd);
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

/// Evaluate the intrinsic BJT (Ebers-Moll, optionally Gummel-Poon-modulated
/// transport current) at internal junction voltages.
///
/// Returns `(Ic, Ib, [dIc/dVbe, dIc/dVbc, dIb/dVbe, dIb/dVbc])` — the same
/// contract as the generated runtime's `bjt_evaluate` (device_bjt.rs.tera):
/// GP modulates the transport current only; Ib and its Jacobian come from
/// the leakage-carrying Ebers-Moll base model.
fn bjt_eval_intrinsic(
    em: &BjtEbersMoll,
    gp: Option<&BjtGummelPoon>,
    vbe: f64,
    vbc: f64,
) -> (f64, f64, [f64; 4]) {
    use melange_devices::NonlinearDevice;
    let (ic, dic) = match gp {
        Some(g) => (g.collector_current(vbe, vbc), g.jacobian(&[vbe, vbc])),
        None => {
            let (dic_dvbe, dic_dvbc) = em.collector_jacobian(vbe, vbc);
            (em.collector_current(vbe, vbc), [dic_dvbe, dic_dvbc])
        }
    };
    let ib = em.base_current(vbe, vbc);
    let dib_dvbe = em.base_current_jacobian_dvbe(vbe, vbc);
    let dib_dvbc = em.base_current_jacobian_dvbc(vbe, vbc);
    (ic, ib, [dic[0], dic[1], dib_dvbe, dib_dvbc])
}

/// BJT with parasitic resistances (RB, RC, RE), evaluated at EXTERNAL
/// terminal-pair voltages.
///
/// Mirrors the generated runtime's `bjt_with_parasitics`
/// (device_bjt.rs.tera): a damped 2D Newton on the internal junction
/// voltages (Vbe_int, Vbc_int) with residuals
///   F1 = Vbe_int - Vbe_ext + Ib·RB + (Ic+Ib)·RE
///   F2 = Vbc_int - Vbc_ext + Ib·RB - Ic·RC
/// analytic 2×2 J_F from the device Jacobian, and a ±4·VT step clamp.
///
/// This replaces the previous 3-iteration UNDAMPED fixed point
/// `vbe_i = vbe - ib·rb - (ic+ib)·re`, which diverges (oscillates between
/// cutoff and saturation) whenever gm·(RE + RB/β) ≳ 1 — i.e. Ic > VT/RE,
/// only 2.6 mA at RE = 10 Ω, well inside power-BJT operating range.
///
/// Returned Jacobian is the EXTERNAL one: J_ext = J_dev · J_F⁻¹ (implicit-
/// function chain rule through the internal solve), matching the runtime
/// template. The previous code stamped the raw internal Jacobian against
/// external controlling voltages, which is an inconsistent Newton
/// linearization (overestimates conductance by the local-feedback factor
/// 1 + gm·RE + ...); the companion stamp `G_aug = G_dc - N_i·J_dev·N_v`
/// and the companion RHS `i_nl - J_dev·v_nl` both consume this Jacobian
/// against external v_nl, so the external form is the coherent choice.
#[allow(clippy::too_many_arguments)]
fn bjt_with_parasitics_dc(
    em: &BjtEbersMoll,
    gp: Option<&BjtGummelPoon>,
    vbe_ext: f64,
    vbc_ext: f64,
    rb: f64,
    rc: f64,
    re: f64,
    vt: f64,
) -> (f64, f64, [f64; 4]) {
    const INNER_MAX_ITER: usize = 50;
    const INNER_TOL: f64 = 1e-12;

    // Initial guess: internal = external
    let mut vbe_int = vbe_ext;
    let mut vbc_int = vbc_ext;

    for _ in 0..INNER_MAX_ITER {
        let (ic, ib, j) = bjt_eval_intrinsic(em, gp, vbe_int, vbc_int);
        let (dic_dvbe, dic_dvbc, dib_dvbe, dib_dvbc) = (j[0], j[1], j[2], j[3]);

        // Residuals (RE term uses Ie = -(Ic+Ib):
        // V_E_int = V_E_ext - Ie·RE = V_E_ext + (Ic+Ib)·RE)
        let f1 = vbe_int - vbe_ext + ib * rb + (ic + ib) * re;
        let f2 = vbc_int - vbc_ext + ib * rb - ic * rc;

        if f1.abs() < INNER_TOL && f2.abs() < INNER_TOL {
            break;
        }

        // Jacobian of F w.r.t. (Vbe_int, Vbc_int)
        let j11 = 1.0 + dib_dvbe * rb + (dic_dvbe + dib_dvbe) * re;
        let j12 = dib_dvbc * rb + (dic_dvbc + dib_dvbc) * re;
        let j21 = dib_dvbe * rb - dic_dvbe * rc;
        let j22 = 1.0 + dib_dvbc * rb - dic_dvbc * rc;

        // Solve 2x2 via Cramer's rule
        let det = j11 * j22 - j12 * j21;
        if det.abs() < 1e-30 {
            break;
        }
        let inv_det = 1.0 / det;
        let dvbe = (j22 * f1 - j12 * f2) * inv_det;
        let dvbc = (j11 * f2 - j21 * f1) * inv_det;

        // Step limiting: max ±4·VT per iteration (damping)
        let max_step = 4.0 * vt;
        let dvbe = dvbe.clamp(-max_step, max_step);
        let dvbc = dvbc.clamp(-max_step, max_step);

        vbe_int -= dvbe;
        vbc_int -= dvbc;
    }

    // Final evaluation at converged internal voltages
    let (ic, ib, j) = bjt_eval_intrinsic(em, gp, vbe_int, vbc_int);
    let (dic_dvbe, dic_dvbc, dib_dvbe, dib_dvbc) = (j[0], j[1], j[2], j[3]);

    // External Jacobian: J_ext = J_device · J_F⁻¹ (recompute J_F at the
    // converged point, same as the runtime template).
    let j11 = 1.0 + dib_dvbe * rb + (dic_dvbe + dib_dvbe) * re;
    let j12 = dib_dvbc * rb + (dic_dvbc + dib_dvbc) * re;
    let j21 = dib_dvbe * rb - dic_dvbe * rc;
    let j22 = 1.0 + dib_dvbc * rb - dic_dvbc * rc;

    let det = j11 * j22 - j12 * j21;
    if det.abs() < 1e-30 {
        // Fallback: return intrinsic Jacobian
        return (ic, ib, j);
    }
    let inv_det = 1.0 / det;

    // J_F⁻¹ = [[j22, -j12], [-j21, j11]] / det
    let fi11 = j22 * inv_det;
    let fi12 = -j12 * inv_det;
    let fi21 = -j21 * inv_det;
    let fi22 = j11 * inv_det;

    // J_ext = J_device · J_F⁻¹
    let dic_dvbe_ext = dic_dvbe * fi11 + dic_dvbc * fi21;
    let dic_dvbc_ext = dic_dvbe * fi12 + dic_dvbc * fi22;
    let dib_dvbe_ext = dib_dvbe * fi11 + dib_dvbc * fi21;
    let dib_dvbc_ext = dib_dvbe * fi12 + dib_dvbc * fi22;

    (
        ic,
        ib,
        [dic_dvbe_ext, dic_dvbc_ext, dib_dvbe_ext, dib_dvbc_ext],
    )
}

/// Internal node mapping for a BJT with parasitic resistances in the DC system.
/// When RB/RC/RE > 0, internal nodes are created so the intrinsic BJT model
/// operates on internal junction voltages while parasitic R is in the global matrix.
struct BjtInternalNodes {
    start_idx: usize, // M-dimension start index
    ext_base: usize,  // 1-indexed external node (0=ground)
    ext_collector: usize,
    ext_emitter: usize,
    /// 0-indexed row in `g_dc`: `Some(new internal row)` when the terminal's
    /// parasitic R > 0, `Some(ext - 1)` when R = 0 on a non-ground terminal,
    /// `None` when R = 0 on a grounded terminal. Ground has no matrix row —
    /// the old `else { 0 }` fallback aliased it onto circuit node index 0,
    /// injecting device current into whatever node happened to be first.
    int_base: Option<usize>,
    int_collector: Option<usize>,
    int_emitter: Option<usize>,
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
    /// Per-row classification of the DC system: `true` for rows that carry a
    /// node VOLTAGE (circuit nodes 0..n, MNA-level BJT internal nodes, and
    /// DC-added BJT internal nodes), `false` for algebraic/branch-current rows
    /// (VS/VCVS extension rows, inductor short-circuit branch currents).
    /// Voltage rows are subject to NR damping and the ±50 V flat clamp;
    /// current rows (amperes, not volts) must never be clamped.
    is_voltage_row: Vec<bool>,
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

    // Cap op-amp VCCS gain in the DC system to prevent NR instability.
    // Precision rectifiers create multi-equilibrium landscapes where
    // AOL=200,000 makes the NR overshoot from one rail to the other
    // in a single iteration. Capping at AOL_DC=1000 keeps the NR stable
    // while maintaining accurate virtual grounds (0.1% error).
    // The full AOL is used in the transient codegen path where the
    // active-set rail resolver handles convergence.
    const AOL_DC_MAX: f64 = 1000.0;
    for oa in &mna.opamps {
        let out = oa.n_out_idx;
        if out == 0 || oa.aol <= AOL_DC_MAX {
            continue;
        }
        let o = out - 1;
        if o >= n_aug {
            continue;
        }
        let np = oa.n_plus_idx;
        let nm = oa.n_minus_idx;
        let gm_full = oa.aol / oa.r_out;
        let gm_capped = AOL_DC_MAX / oa.r_out;
        let delta_gm = gm_full - gm_capped;
        // Undo the excess Gm from the MNA stamps. The corrected VCCS stamp is
        // np -= gm / nm += gm, so removing delta_gm reverses those signs.
        if np > 0 && np - 1 < n_aug {
            g_dc[o][np - 1] += delta_gm;
        }
        if nm > 0 && nm - 1 < n_aug {
            g_dc[o][nm - 1] -= delta_gm;
        }
    }

    // Stamp input conductance
    for &(in_node, r) in &config.extra_inputs {
        if in_node < n && r > 0.0 {
            g_dc[in_node][in_node] += 1.0 / r;
        }
    }
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

                    // Create internal nodes for non-zero resistances.
                    // R = 0 on a grounded terminal maps to None (ground has
                    // no matrix row; stamping is skipped for it).
                    let int_b = if bp.rb > 0.0 {
                        let idx = n_dc;
                        n_dc += 1;
                        Some(idx)
                    } else if ext_b > 0 {
                        Some(ext_b - 1)
                    } else {
                        None
                    };
                    let int_c = if bp.rc > 0.0 {
                        let idx = n_dc;
                        n_dc += 1;
                        Some(idx)
                    } else if ext_c > 0 {
                        Some(ext_c - 1)
                    } else {
                        None
                    };
                    let int_e = if bp.re > 0.0 {
                        let idx = n_dc;
                        n_dc += 1;
                        Some(idx)
                    } else if ext_e > 0 {
                        Some(ext_e - 1)
                    } else {
                        None
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
                // R > 0 always allocates a fresh internal row, so the
                // `if let` bindings below are infallible under each guard.
                // RB: external base ↔ internal base
                if bp.rb > 0.0 {
                    if let Some(ib) = bjt.int_base {
                        let g = 1.0 / bp.rb;
                        if bjt.ext_base > 0 {
                            let eb = bjt.ext_base - 1;
                            g_dc[eb][eb] += g;
                            g_dc[eb][ib] -= g;
                            g_dc[ib][eb] -= g;
                        }
                        g_dc[ib][ib] += g;
                    }
                }
                // RC: external collector ↔ internal collector
                if bp.rc > 0.0 {
                    if let Some(ic) = bjt.int_collector {
                        let g = 1.0 / bp.rc;
                        if bjt.ext_collector > 0 {
                            let ec = bjt.ext_collector - 1;
                            g_dc[ec][ec] += g;
                            g_dc[ec][ic] -= g;
                            g_dc[ic][ec] -= g;
                        }
                        g_dc[ic][ic] += g;
                    }
                }
                // RE: external emitter ↔ internal emitter
                if bp.re > 0.0 {
                    if let Some(ie) = bjt.int_emitter {
                        let g = 1.0 / bp.re;
                        if bjt.ext_emitter > 0 {
                            let ee = bjt.ext_emitter - 1;
                            g_dc[ee][ee] += g;
                            g_dc[ee][ie] -= g;
                            g_dc[ie][ee] -= g;
                        }
                        g_dc[ie][ie] += g;
                    }
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
                    if let Some(ib) = bjt.int_base {
                        g_dc[ib][ib] += gmin_floor;
                    }
                }
                if bp.rc > 0.0 {
                    if let Some(ic) = bjt.int_collector {
                        g_dc[ic][ic] += gmin_floor;
                    }
                }
                if bp.re > 0.0 {
                    if let Some(ie) = bjt.int_emitter {
                        g_dc[ie][ie] += gmin_floor;
                    }
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

        // Zero-then-ACCUMULATE (rows/cols were cleared above), mirroring
        // mna.rs `expand_bjt_internal_nodes`. Assign-stamps overwrote the
        // first entry when two terminals share a matrix row (diode-connected
        // `Q x x e`, strapped junctions with a zero parasitic R): e.g. the
        // Vbc row got `-1` instead of the canceled `+1 - 1 = 0`, sensing a
        // phantom `Vbc = -V(x)`. `None` (grounded, R = 0) contributes
        // nothing — ground is 0 V in N_v and swallows N_i current.
        // N_v: extract Vbe_int = V(basePrime) - V(emitterPrime)
        if let Some(ib) = bjt.int_base {
            dc_n_v[s][ib] += 1.0;
        }
        if let Some(ie) = bjt.int_emitter {
            dc_n_v[s][ie] -= 1.0;
        }
        // N_v: extract Vbc_int = V(basePrime) - V(collectorPrime)
        if let Some(ib) = bjt.int_base {
            dc_n_v[s + 1][ib] += 1.0;
        }
        if let Some(ic) = bjt.int_collector {
            dc_n_v[s + 1][ic] -= 1.0;
        }

        // N_i: Ic enters collectorPrime, exits emitterPrime
        if let Some(ic) = bjt.int_collector {
            dc_n_i[ic][s] -= 1.0;
        }
        if let Some(ie) = bjt.int_emitter {
            dc_n_i[ie][s] += 1.0;
        }
        // N_i: Ib enters basePrime, exits emitterPrime
        if let Some(ib) = bjt.int_base {
            dc_n_i[ib][s + 1] -= 1.0;
        }
        if let Some(ie) = bjt.int_emitter {
            dc_n_i[ie][s + 1] += 1.0;
        }
    }

    // Classify every DC-system row as voltage-carrying (clamp/damp) or
    // algebraic/branch-current (never clamp). Layout:
    //   [0..n)                      circuit nodes                → voltage
    //   [n..n_aug)                  VS/VCVS ext rows             → current
    //     ... except MNA-level BJT internal nodes (appended to the tail of
    //         n_aug by `expand_bjt_internal_nodes`)              → voltage
    //   [n_aug..n_aug+num_ind)      inductor short branch rows   → current
    //   [internal_node_start..n_dc) DC-added BJT internal nodes  → voltage
    let mut is_voltage_row = vec![false; n_dc];
    for flag in is_voltage_row.iter_mut().take(n) {
        *flag = true;
    }
    for mna_bjt in &mna.bjt_internal_nodes {
        for idx in [mna_bjt.int_base, mna_bjt.int_collector, mna_bjt.int_emitter]
            .into_iter()
            .flatten()
        {
            if idx < n_dc {
                is_voltage_row[idx] = true;
            }
        }
    }
    for flag in is_voltage_row
        .iter_mut()
        .take(n_dc)
        .skip(internal_node_start)
    {
        *flag = true;
    }

    DcSystemInfo {
        g_dc,
        b_dc,
        n_dc,
        dc_n_v,
        dc_n_i,
        bjt_internal,
        is_voltage_row,
    }
}

/// Distribute a junction-space voltage-limiting correction back into node
/// space through the pseudo-inverse of the junction's N_v row.
///
/// A raw `pn_corrections[j] += n_v[idx][j] * correction` (plain transpose)
/// applies a junction-space change of `‖N_v row‖² · correction`: exact for
/// grounded junctions (‖row‖² = 1) but DOUBLED for floating junctions
/// (row = [+1, −1] → ‖row‖² = 2). A strong limiting event then overshoots
/// past the limited target into a REVERSED step (e.g. v_old = 0.5, raw 5.0,
/// v_lim = 0.634 lands at −3.73 V), which is the classic period-2 NR
/// oscillation signature. Normalizing by ‖row‖² (the Moore-Penrose
/// pseudo-inverse of the 1×n row) makes the post-correction junction voltage
/// land exactly on `v_lim` — the minimum-norm node update with that property.
///
/// Zero rows (tied terminals — degenerate junction) are skipped: there is no
/// node update that can change such a junction voltage.
fn distribute_junction_correction(n_v_row: &[f64], correction: f64, pn_corrections: &mut [f64]) {
    let norm_sq: f64 = n_v_row.iter().map(|x| x * x).sum();
    if norm_sq < 1e-30 {
        return;
    }
    let scaled = correction / norm_sq;
    for (corr, &nv) in pn_corrections.iter_mut().zip(n_v_row.iter()) {
        if nv != 0.0 {
            *corr += nv * scaled;
        }
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
    lu_decompose_in_place(a.to_vec())
}

/// Same as [`lu_decompose`] but consumes the input — avoids an extra clone
/// when the matrix has already been built locally (e.g. after equilibration).
fn lu_decompose_in_place(mut lu: Vec<Vec<f64>>) -> Option<(Vec<Vec<f64>>, Vec<usize>)> {
    let n = lu.len();
    let mut pivot: Vec<usize> = (0..n).collect();

    for k in 0..n {
        let mut max_val = lu[k][k].abs();
        let mut max_row = k;
        for i in (k + 1)..n {
            if lu[i][k].abs() > max_val {
                max_val = lu[i][k].abs();
                max_row = i;
            }
        }
        if max_val < 1e-30 {
            return None;
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

/// Asymmetric row/column equilibration: scales `a` in place so each row's
/// max-norm and each column's max-norm equal 1. Returns the row/col scaling
/// vectors `(dr, dc)` for de-equilibrating the solve.
///
/// Without this, melange's input row (Thevenin `G_in = 1 S`) dominates the
/// MNA matrix by 4-6 decades over typical neighbouring conductances (1e-4 to
/// 1e-6 S), so partial-pivoting LU enters factorisation with `cond(A) >= 1e6`
/// and loses ~6 digits of f64 precision before solving anything.
///
/// Keep in sync with `nodal_emitter::emit_nodal_lu_factor` — the generated
/// nodal path uses the same algorithm.
fn equilibrate(a: &mut [Vec<f64>]) -> (Vec<f64>, Vec<f64>) {
    let n = a.len();
    let mut dr = vec![1.0; n];
    let mut dc = vec![1.0; n];

    // Row scaling: dr[i] = 1 / max_j |A[i][j]|
    for i in 0..n {
        let mut row_max = 0.0_f64;
        for j in 0..n {
            let v = a[i][j].abs();
            if v > row_max {
                row_max = v;
            }
        }
        dr[i] = if row_max > 1e-30 { 1.0 / row_max } else { 1.0 };
    }
    for i in 0..n {
        let s = dr[i];
        for v in a[i].iter_mut() {
            *v *= s;
        }
    }

    // Column scaling: dc[j] = 1 / max_i |A[i][j]| (against the row-scaled matrix)
    for j in 0..n {
        let mut col_max = 0.0_f64;
        for row in a.iter() {
            let v = row[j].abs();
            if v > col_max {
                col_max = v;
            }
        }
        dc[j] = if col_max > 1e-30 { 1.0 / col_max } else { 1.0 };
    }
    for row in a.iter_mut() {
        for (j, v) in row.iter_mut().enumerate() {
            *v *= dc[j];
        }
    }

    (dr, dc)
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

/// Solve a linear system g_aug · v = rhs using equilibrated LU decomposition.
///
/// Applies asymmetric row/column scaling before factorisation to keep `cond(A)`
/// inside f64 precision regardless of unit imbalance between G_in (≈ 1 S) and
/// neighbouring conductances (1e-4 to 1e-6 S). Returns None if the matrix is
/// singular after equilibration.
pub fn solve_linear(g_aug: &[Vec<f64>], rhs: &[f64]) -> Option<Vec<f64>> {
    let mut a = g_aug.to_vec();
    let (dr, dc) = equilibrate(&mut a);
    let (lu, pivot) = lu_decompose_in_place(a)?;
    let b_scaled: Vec<f64> = rhs.iter().zip(dr.iter()).map(|(&b, &d)| d * b).collect();
    let y = lu_solve(&lu, &pivot, &b_scaled);
    Some(
        y.into_iter()
            .zip(dc.iter())
            .map(|(yi, &d)| d * yi)
            .collect(),
    )
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
                    DeviceParams::Tube(tp) => {
                        (matches!(tp.kind, TubeKind::SharpPentode), tp.kg2 > 0.0)
                    }
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
                    let target_vg2 = v_cath + (v_supply - v_cath - 50.0).clamp(20.0, 250.0);
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
    /// Per-row voltage/current classification (n_dc entries; see
    /// [`DcSystemInfo::is_voltage_row`]). Voltage rows (circuit nodes + BJT
    /// internal nodes) are damped and flat-clamped; branch-current rows
    /// (VS/VCVS/inductor augmented variables) are updated directly.
    is_voltage_row: &'a [bool],
}

/// Uses companion formulation at each iteration:
///   G_aug = G_dc - N_i · J_dev · N_v
///   rhs = b_dc + N_i · (i_nl - J_dev · v_nl)
///   v_new = G_aug^{-1} · rhs
///
/// Returns (converged, iterations).
///
/// `aol_cont_mode`: when true, op-amp rail clamping for sidechain-rectifier
/// op-amps is widened to the actual DC supply voltage at n_plus rather than
/// the model's VSAT. This is required during AOL continuation steps where the
/// physical equilibrium has the output near the n_plus reference voltage
/// (e.g. -12 V for a TL074 in a half-wave rectifier referenced to vee12=-12V),
/// which may be below the conservative VSAT model limit (-11V for VSAT=11).
fn nr_dc_solve(
    circuit: &DcCircuit,
    v: &mut Vec<f64>,
    v_nl: &mut [f64],
    i_nl: &mut [f64],
    source_scale: f64,
    gmin: f64,
    aol_cont_mode: bool,
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
        evaluate_devices_inner(
            v_nl,
            device_slots,
            i_nl,
            &mut j_dev,
            m,
            internal_junctions,
            Some(v),
        );

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
        if iter < 3 && m > 4 {
            log::debug!(
                "DC NR iter {} (scale={:.2}): v_nl[4..]=[{}]",
                iter,
                source_scale,
                v_nl[4..]
                    .iter()
                    .enumerate()
                    .map(|(i, x)| format!("[{}]={:.4}", i + 4, x))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        if iter < 3 && aol_cont_mode {
            for oa in &mna.opamps {
                let out = oa.n_out_idx;
                if out > 0 && out - 1 < v.len() {
                    let np_v = if oa.n_plus_idx > 0 {
                        v[oa.n_plus_idx - 1]
                    } else {
                        0.0
                    };
                    let nm_v = if oa.n_minus_idx > 0 {
                        v[oa.n_minus_idx - 1]
                    } else {
                        0.0
                    };
                    log::debug!(
                        "  opamp out_idx={} v_out={:.3} v+={:.3} v-={:.3} sr={}",
                        out,
                        v[out - 1],
                        np_v,
                        nm_v,
                        dc_opamp_is_sidechain_rectifier(oa, mna)
                    );
                }
            }
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
                        distribute_junction_correction(
                            &dc_n_v[idx],
                            v_lim - v_raw,
                            &mut pn_corrections,
                        );
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
                        distribute_junction_correction(
                            &dc_n_v[be_idx],
                            correction,
                            &mut pn_corrections,
                        );
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
                            distribute_junction_correction(
                                &dc_n_v[bc_idx],
                                correction,
                                &mut pn_corrections,
                            );
                        }
                    }
                }
                // JFET/MOSFET: SPICE fetlim, mirroring the transient dispatch
                // table (nr_helpers.rs): dim 0 = Vds → fetlim(·, ·, 0.0),
                // dim 1 = Vgs → fetlim(·, ·, vto) with vto = raw pinch-off /
                // threshold in the device's own sign convention (jp.vp is
                // negative for N-channel JFETs, mp.vt negative for PMOS —
                // exactly the value the transient path passes as
                // `state.device_N_vp` / `state.device_N_vt`; no polarity flip).
                (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                    let ds_idx = slot.start_idx;
                    let v_lim_ds = fetlim(v_nl_new[ds_idx], v_nl[ds_idx], 0.0);
                    if (v_lim_ds - v_nl_new[ds_idx]).abs() > 1e-15 {
                        distribute_junction_correction(
                            &dc_n_v[ds_idx],
                            v_lim_ds - v_nl_new[ds_idx],
                            &mut pn_corrections,
                        );
                    }
                    if slot.dimension > 1 {
                        let gs_idx = slot.start_idx + 1;
                        let v_lim_gs = fetlim(v_nl_new[gs_idx], v_nl[gs_idx], jp.vp);
                        if (v_lim_gs - v_nl_new[gs_idx]).abs() > 1e-15 {
                            distribute_junction_correction(
                                &dc_n_v[gs_idx],
                                v_lim_gs - v_nl_new[gs_idx],
                                &mut pn_corrections,
                            );
                        }
                    }
                }
                (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                    let ds_idx = slot.start_idx;
                    let v_lim_ds = fetlim(v_nl_new[ds_idx], v_nl[ds_idx], 0.0);
                    if (v_lim_ds - v_nl_new[ds_idx]).abs() > 1e-15 {
                        distribute_junction_correction(
                            &dc_n_v[ds_idx],
                            v_lim_ds - v_nl_new[ds_idx],
                            &mut pn_corrections,
                        );
                    }
                    if slot.dimension > 1 {
                        let gs_idx = slot.start_idx + 1;
                        let v_lim_gs = fetlim(v_nl_new[gs_idx], v_nl[gs_idx], mp.vt);
                        if (v_lim_gs - v_nl_new[gs_idx]).abs() > 1e-15 {
                            distribute_junction_correction(
                                &dc_n_v[gs_idx],
                                v_lim_gs - v_nl_new[gs_idx],
                                &mut pn_corrections,
                            );
                        }
                    }
                }
                // Tubes: intentionally unlimited here. The transient path uses
                // pnjlim(Vgk, ·, vgk_onset/3, VCRIT) + fetlim(Vpk, ·, 0.0), but
                // the tube VCRIT constant is derived in the codegen emitters
                // and is not trivially available at this call site; the pentode
                // bias seeding in `clamp_junction_voltages` covers the known
                // divergence cases. Port the tube dispatch if a DC OP tube
                // oscillation ever shows up.
                _ => {}
            }
        }

        // Apply pnjlim corrections to v_new
        let v_limited: Vec<f64> = v_new
            .iter()
            .zip(pn_corrections.iter())
            .map(|(&vn, &corr)| vn + corr)
            .collect();

        // Global node voltage damping: if max change > 10V, scale all node updates.
        // Only VOLTAGE rows participate (circuit nodes + BJT internal nodes, per
        // the explicit is_voltage_row classification). Branch-current rows
        // (VS/VCVS extension rows, inductor DC-short branch currents) are in
        // amperes — sweeping them into the voltage damping scan or the ±50 "V"
        // flat clamp would stall convergence whenever a branch current
        // legitimately exceeds 50 A (or trigger phantom damping from large
        // current steps).
        let v_max_step = 50.0;
        let mut max_delta = 0.0_f64;
        for i in 0..n_dc {
            if circuit.is_voltage_row[i] {
                let delta = (v_limited[i] - v[i]).abs();
                max_delta = max_delta.max(delta);
            }
        }
        let damping = if max_delta > 10.0 {
            (10.0 / max_delta).max(0.1)
        } else {
            1.0
        };

        // Per-variable SPICE-style convergence check on the ACTUALLY APPLIED
        // step: |delta_i| < reltol·|v_i| + abstol. The relative term keeps
        // high-voltage circuits (300 V rails) from false-failing on the LU
        // round-off floor; abstol = config.tolerance preserves the legacy
        // behavior for small-signal nodes.
        let mut all_within_tol = true;
        for i in 0..n_dc {
            let delta = v_limited[i] - v[i];
            let limited = if circuit.is_voltage_row[i] {
                // Circuit node or internal parasitic node: apply damping + flat clamp
                let damped = delta * damping;
                damped.clamp(-v_max_step, v_max_step)
            } else {
                // Augmented variables (VS currents, inductor currents): apply directly
                delta
            };
            v[i] += limited;
            if limited.abs() >= config.reltol * v[i].abs() + config.tolerance {
                all_within_tol = false;
            }
        }

        // Clamp op-amp output voltages to rail limits.
        // Without this, the linear op-amp model drives outputs to
        // unphysical voltages during DC solve, preventing convergence
        // in precision rectifier circuits (e.g. SSL bus compressor).
        //
        // During source stepping (source_scale < 1): scale the rail limits by
        // source_scale to match the ramped supplies. Without this, an op-amp with
        // VSAT=11V would clamp to -11V while the physical supply is only -0.6V
        // (at scale=0.04), creating huge phantom voltage differentials across
        // diodes and resistors that prevent NR convergence.
        //
        // In AOL continuation mode, SR op-amps use an expanded VEE limit equal to
        // the actual n_plus supply node voltage minus one diode drop. This allows
        // the output to reach the physically correct equilibrium (e.g. n_plus=-12V
        // → output can reach -12.6V, below VSAT=-11V).
        for oa in &mna.opamps {
            let out = oa.n_out_idx; // 1-indexed
            if out > 0 {
                let o = out - 1; // 0-indexed
                if o < mna.n {
                    // Determine effective rail limits for this op-amp.
                    // Base rails scale with source_scale so op-amp output tracks the
                    // ramping supply during source stepping.
                    let base_vcc = if oa.vcc.is_finite() {
                        oa.vcc * source_scale
                    } else {
                        oa.vcc
                    };
                    let base_vee = if oa.vee.is_finite() {
                        oa.vee * source_scale
                    } else {
                        oa.vee
                    };

                    let (eff_vcc, eff_vee) =
                        if aol_cont_mode && dc_opamp_is_sidechain_rectifier(oa, mna) {
                            // SR op-amps in AOL continuation: widen the VEE limit to allow
                            // the output to reach the n_plus reference voltage (which is the
                            // actual DC reference for the precision rectifier). Use the
                            // current node voltage at n_plus rather than the model's VSAT.
                            let v_nplus = if oa.n_plus_idx > 0 && oa.n_plus_idx - 1 < v.len() {
                                v[oa.n_plus_idx - 1]
                            } else {
                                base_vee
                            };
                            // Allow output to go 1V below n_plus (headroom for diode drop).
                            // Take the min with base_vee so output can't go below the supply.
                            let eff_vee = (v_nplus - 1.0).min(base_vee);
                            (base_vcc, eff_vee)
                        } else {
                            (base_vcc, base_vee)
                        };
                    if eff_vcc.is_finite() && v[o] > eff_vcc {
                        v[o] = eff_vcc;
                    }
                    if eff_vee.is_finite() && v[o] < eff_vee {
                        v[o] = eff_vee;
                    }
                }
            }
        }

        if all_within_tol {
            // Final evaluation at converged point
            extract_nl_voltages_with(m, dc_n_v, v, v_nl);
            evaluate_devices_inner(
                v_nl,
                device_slots,
                i_nl,
                &mut j_dev,
                m,
                internal_junctions,
                Some(v),
            );
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
        Some(v),
    );
    if aol_cont_mode {
        log::debug!(
            "nr_dc_solve NON-CONVERGENCE (scale={:.3}, aol_mode): v_nl[4..]=[{}] op-amp-outs=[{}]",
            source_scale,
            v_nl.iter()
                .enumerate()
                .skip(4)
                .map(|(i, x)| format!("[{}]={:.3}", i, x))
                .collect::<Vec<_>>()
                .join(", "),
            mna.opamps
                .iter()
                .filter_map(|oa| {
                    let out = oa.n_out_idx;
                    if out > 0 && out - 1 < v.len() {
                        Some(format!("{:.3}", v[out - 1]))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    (false, config.max_iterations)
}

/// Degenerate-solution check shared by all convergence strategies.
///
/// A converged NR solution with every PN junction OFF ("all devices at supply
/// rails") is usually the spurious open-loop fixed point of a feedback bias
/// network — unless the circuit has no DC excitation at all (`b_dc` all zero),
/// in which case all-off IS the true operating point and the check is skipped.
///
/// Semantics are any-device-validates: a single forward-biased junction (or
/// any non-junction device — FET/tube/VCA/op-amp results are trusted as-is)
/// validates the whole solution. Per-device validation would be stricter but
/// is a larger behavior change (multi-stage circuits where one stage is
/// legitimately off would start false-failing) — keep any-device semantics.
fn solution_has_active_junction(
    device_slots: &[DeviceSlot],
    v_nl: &[f64],
    has_dc_excitation: bool,
) -> bool {
    if !has_dc_excitation {
        // No DC sources: all-junctions-off is the physically correct OP.
        return true;
    }
    for slot in device_slots {
        match &slot.params {
            DeviceParams::Bjt(bp) => {
                if slot.start_idx < v_nl.len() {
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    // Forward-biased B-E junction: sign·Vbe > threshold.
                    //
                    // The threshold used to be a hardcoded 0.3V, which assumes
                    // a silicon-typical turn-on (Vf ~ 0.6-0.7V). Low-IS devices
                    // — germanium (IS ~ 1e-7 to 1e-6), Schottky — turn on at a
                    // much lower forward voltage (Ge Vbe ~ 0.15-0.3V), so a
                    // fixed 0.3V cutoff misclassifies their correct, converged
                    // forward-active Direct-NR solution as "degenerate" and
                    // discards it in favor of source/Gmin stepping — which is
                    // markedly less robust (see the OC74 PNP common-emitter
                    // repro: Direct NR lands on the textbook OP at every
                    // tested R_E, but the fixed 0.3V check rejected all of
                    // them because Vbe ≈ 0.17-0.20V for this Ge device, and
                    // Gmin stepping then diverges outright for R_E in
                    // [1.4k, 2.5k]).
                    //
                    // Scale the threshold with the device's own turn-on
                    // voltage instead: half of `pn_vcrit` (the same quantity
                    // NR's own pnjlim uses to decide "this step crossed the
                    // exponential knee"). For silicon (IS ~ 1e-14, vcrit ~
                    // 0.6-0.65V) this reproduces the old ~0.3V behavior almost
                    // exactly; for germanium (IS ~ 3e-7, vcrit ~ 0.29V) it
                    // drops to ~0.14V, correctly accepting Ge's lower Vf.
                    let nf_vt = bp.nf * bp.vt;
                    let vcrit = pn_vcrit(nf_vt, bp.is);
                    let threshold = 0.5 * vcrit;
                    if sign * v_nl[slot.start_idx] > threshold {
                        return true;
                    }
                }
            }
            DeviceParams::Diode(dp) => {
                if slot.start_idx < v_nl.len() {
                    // Same device-scaled threshold as the BJT case above.
                    let vcrit = pn_vcrit(dp.n_vt, dp.is);
                    if v_nl[slot.start_idx].abs() > 0.5 * vcrit {
                        return true;
                    }
                }
            }
            // Non-junction devices: trust the converged result.
            _ => return true,
        }
    }
    // Junction-only circuit with every junction off under DC excitation:
    // degenerate (also covers empty device_slots, which callers gate on m == 0
    // before reaching this point).
    device_slots.is_empty()
}

/// Seed op-amp outputs from (V+ - V-) sign to select the correct equilibrium.
/// Precision rectifiers have two self-consistent DC operating points (one at
/// each rail). Without this, the NR may converge to the wrong one.
/// For each sidechain-rectifier op-amp, pre-seed the inverting-input node
/// at `v_out + 0.65 V` (one diode drop above the rail-clamped output) so the
/// feedback diode starts at its physical forward-bias drop rather than
/// the 11 V forward bias implied by a linear-DC-OP seed.
///
/// Physics: at quiescent with `v_plus` on a negative rail (e.g. vee12),
/// the op-amp drives `v_out` to the VSAT rail. The feedback diode conducts
/// forward current set by the series resistor network (~500 µA through
/// Rrect_a_in / Rfb). The diode equation gives `Vd ≈ 0.64 V` at that
/// current. So `v_minus = v_out + Vd ≈ VSAT + 0.65`. With VEE < VSAT (e.g.
/// vee12 = −12, VSAT = 11 → v_out = −11, v_minus = −10.35), `v_minus`
/// sits above `v_plus` by the VSAT/VEE difference. `v_out` is rail-pinned,
/// NOT at `v_minus − 0.65` as ideal op-amp feedback would predict.
///
/// Without this seed, `seed_opamp_outputs` places `v_out` at VEE/VSAT but
/// leaves `v_minus` at its linear-DC-OP value (often near 0 V). The feedback
/// diode then sees `v_nl = |v_minus − v_out| ≈ 11 V` forward bias, producing
/// `i_nl ≈ 1e14 A` on the first NR iteration and poisoning the Newton step
/// regardless of pnjlim.
fn seed_sr_feedback_diodes(v: &mut [f64], mna: &MnaSystem) {
    for oa in &mna.opamps {
        if !dc_opamp_is_sidechain_rectifier(oa, mna) {
            continue;
        }
        let out = oa.n_out_idx;
        if out == 0 {
            continue;
        }
        let o = out - 1;
        if o >= v.len() || !oa.vee.is_finite() {
            continue;
        }
        let v_out = v[o];
        if v_out > -1.0 {
            continue;
        }
        for dev in &mna.nonlinear_devices {
            if dev.device_type != crate::mna::NonlinearDeviceType::Diode
                || dev.node_indices.len() < 2
            {
                continue;
            }
            let (a, k) = (dev.node_indices[0], dev.node_indices[1]);
            if a != oa.n_out_idx && k != oa.n_out_idx {
                continue;
            }
            let other_1idx = if a == oa.n_out_idx { k } else { a };
            if other_1idx == 0 {
                continue;
            }
            let other = other_1idx - 1;
            if other >= v.len() {
                continue;
            }
            let is_cathode_at_out = k == oa.n_out_idx;
            let v_nl_current = if is_cathode_at_out {
                v[other] - v_out
            } else {
                v_out - v[other]
            };
            if v_nl_current > 1.0 {
                v[other] = if is_cathode_at_out {
                    v_out + 0.65
                } else {
                    v_out - 0.65
                };
            }
        }
    }
}

fn seed_opamp_outputs(v: &mut [f64], mna: &MnaSystem) {
    for oa in &mna.opamps {
        let out = oa.n_out_idx;
        if out == 0 {
            continue;
        }
        let o = out - 1;
        if o >= v.len() {
            continue;
        }
        let vp = if oa.n_plus_idx > 0 && oa.n_plus_idx - 1 < v.len() {
            v[oa.n_plus_idx - 1]
        } else {
            0.0
        };
        let vm = if oa.n_minus_idx > 0 && oa.n_minus_idx - 1 < v.len() {
            v[oa.n_minus_idx - 1]
        } else {
            0.0
        };
        let diff = vp - vm;
        if oa.vcc.is_finite() && oa.vee.is_finite() && diff.abs() > 0.1 {
            v[o] = if diff > 0.0 { oa.vcc } else { oa.vee };
        }
    }
}

/// BFS over the resistor network (nonzero off-diagonal G entries) to find
/// whether `start` can reach `goal`. Avoids op-amp output nodes (which are
/// voltage-source-driven and would create false paths) and ground (index 0).
/// Returns `true` iff a purely-resistive path exists between the two nodes
/// (both specified as 1-indexed MNA node indices, 0 = ground).
fn g_matrix_r_path_exists(start: usize, goal: usize, mna: &MnaSystem) -> bool {
    if start == 0 || goal == 0 {
        return false;
    }
    if start == goal {
        return true;
    }
    // Collect op-amp output node indices to exclude from BFS
    // (they are voltage-controlled and don't represent resistive connections)
    let oa_out_set: std::collections::HashSet<usize> = mna
        .opamps
        .iter()
        .filter(|oa| oa.n_out_idx > 0)
        .map(|oa| oa.n_out_idx - 1)
        .collect();

    let n = mna.n;
    let mut visited = vec![false; n + 1];
    // Use a simple stack-based DFS; nodes are 1-indexed externally, 0-indexed in g
    let mut stack = vec![start - 1]; // convert to 0-indexed
    visited[start - 1] = true;
    let goal_0 = goal - 1;

    while let Some(node) = stack.pop() {
        if node == goal_0 {
            return true;
        }
        if oa_out_set.contains(&node) {
            continue; // don't traverse through op-amp output rows
        }
        for neighbor in 0..n {
            if visited[neighbor] {
                continue;
            }
            // A nonzero off-diagonal G entry indicates a resistive connection
            if mna.g[node][neighbor].abs() > 1e-30 || mna.g[neighbor][node].abs() > 1e-30 {
                visited[neighbor] = true;
                stack.push(neighbor);
            }
        }
    }
    false
}

/// Returns `true` when the op-amp matches the precision-rectifier topology
/// for DC OP AOL continuation purposes.
///
/// Matches the codegen-side `opamp_is_sidechain_rectifier` Rule D' classifier,
/// implemented using only MNA data (no netlist BFS needed):
/// 1. `n_plus` is on a non-zero DC voltage source rail (not ground).
/// 2. At least one diode connects the op-amp output node to the inverting input,
///    either directly or through a purely-resistive path (detected via G-matrix BFS).
fn dc_opamp_is_sidechain_rectifier(oa: &crate::mna::OpampInfo, mna: &MnaSystem) -> bool {
    // Condition 1: n_plus on a non-zero DC rail (not ground)
    if oa.n_plus_idx == 0 {
        return false;
    }
    let on_rail = mna.voltage_sources.iter().any(|vs| {
        vs.dc_value != 0.0 && (vs.n_plus_idx == oa.n_plus_idx || vs.n_minus_idx == oa.n_plus_idx)
    });
    if !on_rail {
        return false;
    }
    // Condition 2: n_minus and n_out are non-ground
    if oa.n_minus_idx == 0 || oa.n_out_idx == 0 {
        return false;
    }
    // Condition 3: at least one diode with one terminal at n_out, the other at n_minus
    // (directly or through a resistor path in the G matrix)
    mna.nonlinear_devices.iter().any(|dev| {
        if dev.device_type != crate::mna::NonlinearDeviceType::Diode {
            return false;
        }
        if dev.node_indices.len() < 2 {
            return false;
        }
        let (a, k) = (dev.node_indices[0], dev.node_indices[1]);
        let touches_out = a == oa.n_out_idx || k == oa.n_out_idx;
        if !touches_out {
            return false;
        }
        let other = if a == oa.n_out_idx { k } else { a };
        if other == oa.n_minus_idx {
            return true;
        }
        // Allow a resistor path from `other` to n_minus
        g_matrix_r_path_exists(other, oa.n_minus_idx, mna)
    })
}

/// Patch the DC conductance matrix for AOL continuation: replace the
/// VCCS entries of all sidechain-rectifier op-amps with the entries
/// corresponding to `aol_step`. The `base_g_dc` is the original matrix
/// from `build_dc_system` (with `AOL_DC_MAX = 1000.0` already applied);
/// this function further lowers (or restores) it to `aol_step`.
///
/// Returns the patched copy. Only the op-amp VCCS rows are modified.
fn patch_g_dc_for_aol(base_g_dc: &[Vec<f64>], mna: &MnaSystem, aol_step: f64) -> Vec<Vec<f64>> {
    let mut g = base_g_dc.to_vec();
    let n_aug = mna.n_aug;

    // AOL_DC_MAX is the cap already baked into base_g_dc (from build_dc_system).
    // We want to further lower to aol_step for ALL op-amps (not just SR ones).
    // Applying global AOL reduction ensures the entire circuit converges at
    // uniform gain levels — if only SR op-amps are reduced, the remaining
    // high-gain op-amps (AOL=1000) still create large NR steps that push the
    // SR op-amp outputs to the wrong rail via resistive feedback paths.
    const AOL_DC_MAX: f64 = 1000.0;

    for oa in &mna.opamps {
        let out = oa.n_out_idx;
        if out == 0 {
            continue;
        }
        let o = out - 1;
        if o >= n_aug {
            continue;
        }
        let np = oa.n_plus_idx;
        let nm = oa.n_minus_idx;

        // base_g_dc already has gm_capped = AOL_DC_MAX / r_out (for each op-amp).
        // We want gm_step = aol_step / r_out.
        // Delta to remove: (AOL_DC_MAX - aol_step) / r_out
        let gm_current = AOL_DC_MAX.min(oa.aol) / oa.r_out;
        let gm_target = aol_step / oa.r_out;
        let delta_gm = gm_current - gm_target; // positive: reduce VCCS

        if delta_gm.abs() < 1e-30 {
            continue;
        }

        // The corrected MNA VCCS stamp (current leaving `out`):
        //   G[out][n_plus]  -= Gm
        //   G[out][n_minus] += Gm
        // so removing excess Gm reverses those signs (same convention as
        // build_dc_system's AOL_DC_MAX undo). We apply an additional undo
        // on top of that for the homotopy step.
        if np > 0 && np - 1 < n_aug {
            g[o][np - 1] += delta_gm;
        }
        if nm > 0 && nm - 1 < n_aug {
            g[o][nm - 1] -= delta_gm;
        }
    }
    g
}

/// Build the DC-OP limitation warning for behavioral B-sources, if any are
/// present in the MNA system.
///
/// The DC OP currently has NO model for behavioral sources: `I={expr}`
/// sources are treated as open circuits (I = 0) and `V={expr}` sources'
/// augmented rows enforce `V+ − V− = 0` (V = 0) instead of `= f(v_dc)`.
/// The transient solver evaluates the real expressions and converges to the
/// true operating point over warmup. Full DC evaluation is a recorded
/// follow-up — see `docs/aidocs/BEHAVIORAL_SOURCES.md` §6; do not silently
/// "fix" this here without implementing that section.
///
/// Returns `Some(message)` naming each behavioral source and its DC
/// treatment, or `None` when the circuit has no behavioral sources.
pub fn behavioral_dc_op_warning(mna: &MnaSystem) -> Option<String> {
    if mna.behavioral_sources.is_empty() {
        return None;
    }
    let listing: Vec<String> = mna
        .behavioral_sources
        .iter()
        .map(|b| match b.kind {
            crate::parser::BSourceKind::Current => {
                format!("{} (I={{}} treated as I=0/open)", b.name)
            }
            crate::parser::BSourceKind::Voltage => {
                format!("{} (V={{}} treated as V=0 constraint)", b.name)
            }
        })
        .collect();
    Some(format!(
        "DC OP: behavioral source(s) [{}] have no DC model — expressions are \
         ignored at the operating point (transient warmup converges to the true \
         OP). Full DC support is a recorded follow-up (BEHAVIORAL_SOURCES.md §6).",
        listing.join(", ")
    ))
}

/// Compute the DC operating point for a circuit with nonlinear devices.
///
/// Strategy:
/// 1. Compute linear DC OP (G^{-1} · b_dc) as initial guess
/// 2. Try direct NR from linear guess
/// 3. If NR fails, try source stepping (ramp DC sources from 0→full)
/// 4. If source stepping fails, try Gmin stepping
/// 5. If Gmin fails, try AOL continuation (precision rectifier multi-equilibrium)
/// 6. Fallback: return linear DC OP with converged=false
/// Solve the initial-state operating point used to seed `v_prev` for
/// circuits containing one or more `IC=`-bearing capacitors (SPICE
/// `.IC`/UIC semantics).
///
/// Each IC-bearing capacitor is temporarily replaced by an ideal DC voltage
/// source (`v(n+) - v(n-) = IC`) on a throwaway clone of `mna`, and the
/// resulting augmented system is solved with the exact same NR machinery as
/// [`solve_dc_operating_point`] (Direct NR → source stepping → Gmin
/// stepping → fallback). Every other element in the circuit — resistors,
/// nonlinear devices, other (non-IC) capacitors opened at DC, other voltage
/// sources — is unchanged, so the result is fully KCL-consistent given the
/// prescribed cap voltages: "IC wins for that cap's initial voltage,
/// everything else stays DC-OP consistent."
///
/// Returns `None` when the circuit has no `IC=`-bearing capacitors, so
/// callers can cheaply skip the extra solve (and the codegen IR can leave
/// the corresponding field `None`, keeping IC-free circuits byte-identical
/// to before this feature existed).
pub fn solve_ic_seeded_operating_point(
    mna: &MnaSystem,
    device_slots: &[DeviceSlot],
    config: &DcOpConfig,
) -> Option<DcOpResult> {
    if mna.capacitor_ics.is_empty() {
        return None;
    }
    let mut mna_ic = mna.clone();
    mna_ic.append_ic_voltage_sources();
    Some(solve_dc_operating_point(&mna_ic, device_slots, config))
}

pub fn solve_dc_operating_point(
    mna: &MnaSystem,
    device_slots: &[DeviceSlot],
    config: &DcOpConfig,
) -> DcOpResult {
    let m = mna.m;

    // Behavioral B-sources have no DC model yet (BEHAVIORAL_SOURCES.md §6
    // follow-up) — be honest about it instead of silently biasing the OP.
    if let Some(msg) = behavioral_dc_op_warning(mna) {
        log::warn!("{}", msg);
    }

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
    // If it IS singular, the G matrix is structurally broken — report it
    // honestly instead of silently returning zeros as a "converged" OP.
    let v_linear = match solve_linear(&dc_sys.g_dc, &dc_sys.b_dc) {
        Some(v) => v,
        None => {
            log::warn!(
                "DC OP: linear DC system is singular (n_dc={}) — G matrix is \
                 structurally defective (floating subcircuit or conflicting \
                 constraints). Returning zeros with converged=false.",
                n_dc
            );
            return DcOpResult {
                v_node: vec![0.0; n_dc],
                v_nl: vec![0.0; m],
                i_nl: vec![0.0; m],
                converged: false,
                method: DcOpMethod::SingularLinear,
                iterations: 0,
            };
        }
    };

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
        is_voltage_row: &dc_sys.is_voltage_row,
    };

    // Clamp junction voltages in the linear initial guess to prevent
    // extreme forward bias (which causes exp() saturation and NR divergence).
    // For multi-stage direct-coupled BJT circuits, the linear guess often has
    // junction voltages of several volts because the BJT is not conducting.
    let mut v_clamped = clamp_junction_voltages(mna, device_slots, &v_linear);
    // Extend to n_dc if internal nodes were added
    v_clamped.resize(n_dc, 0.0);

    seed_opamp_outputs(&mut v_clamped, mna);
    seed_sr_feedback_diodes(&mut v_clamped, mna);
    // Initialize internal node voltages from clamped external nodes
    for bjt in &dc_sys.bjt_internal {
        if let Some(slot) = device_slots.iter().find(|s| s.start_idx == bjt.start_idx) {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.rb > 0.0 && bjt.ext_base > 0 {
                    if let Some(ib) = bjt.int_base {
                        v_clamped[ib] = v_clamped[bjt.ext_base - 1];
                    }
                }
                if bp.rc > 0.0 && bjt.ext_collector > 0 {
                    if let Some(ic) = bjt.int_collector {
                        v_clamped[ic] = v_clamped[bjt.ext_collector - 1];
                    }
                }
                if bp.re > 0.0 && bjt.ext_emitter > 0 {
                    if let Some(ie) = bjt.int_emitter {
                        let base_v = if bp.rb > 0.0 {
                            bjt.int_base.map(|ib| v_clamped[ib]).unwrap_or(0.0)
                        } else if bjt.ext_base > 0 {
                            v_clamped[bjt.ext_base - 1]
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

    // With no DC excitation (b_dc all zero — no DC sources), the all-off
    // solution IS the true operating point; skip the degeneracy check.
    let has_dc_excitation = dc_sys.b_dc.iter().any(|x| x.abs() > 0.0);

    // Strategy 1: Direct NR from junction-clamped linear guess
    let (converged, iters) = nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, 0.0, false);
    // Sanity check: verify the solution isn't degenerate (all nodes at supply rails).
    // A degenerate solution has all BJTs off — check that at least one junction is forward-biased.
    let has_active_junction =
        converged && solution_has_active_junction(device_slots, &v_nl, has_dc_excitation);
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
    let mut ss_zero = vec![0.0; n_dc];
    seed_opamp_outputs(&mut ss_zero, mna);
    let ss_starts: [Vec<f64>; 2] = [
        ss_zero,           // Source stepping from zero with op-amp seeds
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
                nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, scale, 0.0, false);
            this_iters += iters;
            if !converged {
                this_converged = false;
                break;
            }
        }
        total_iters += this_iters;

        if this_converged {
            // Check for degenerate solution (same predicate as Strategy 1;
            // pre-unification this site skipped the diode check, so a
            // diode-only circuit that source-stepped to all-off passed).
            extract_nl_voltages_with(m, &dc_sys.dc_n_v, &v, &mut v_nl);
            let has_active = solution_has_active_junction(device_slots, &v_nl, has_dc_excitation);
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
    seed_opamp_outputs(&mut v, mna);
    // Initialize internal nodes for Gmin stepping too
    for bjt in &dc_sys.bjt_internal {
        if let Some(slot) = device_slots.iter().find(|s| s.start_idx == bjt.start_idx) {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.rb > 0.0 && bjt.ext_base > 0 {
                    if let Some(ib) = bjt.int_base {
                        v[ib] = v[bjt.ext_base - 1];
                    }
                }
                if bp.rc > 0.0 && bjt.ext_collector > 0 {
                    if let Some(ic) = bjt.int_collector {
                        v[ic] = v[bjt.ext_collector - 1];
                    }
                }
                if bp.re > 0.0 && bjt.ext_emitter > 0 {
                    if let Some(ie) = bjt.int_emitter {
                        let base_v = if bp.rb > 0.0 {
                            bjt.int_base.map(|ib| v[ib]).unwrap_or(0.0)
                        } else if bjt.ext_base > 0 {
                            v[bjt.ext_base - 1]
                        } else {
                            0.0
                        };
                        let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                        v[ie] = base_v - sign * 0.65;
                    }
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

        let (converged, iters) =
            nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, gmin, false);
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
        let (converged, iters) =
            nr_dc_solve(&circuit, &mut v, &mut v_nl, &mut i_nl, 1.0, 0.0, false);
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

    // Strategy 4: AOL continuation (precision rectifier / comparator multi-equilibrium)
    //
    // Problem: circuits with sidechain-rectifier op-amps (Rule D' topology: n_plus on
    // a non-zero DC rail, diode in output→inv-input feedback path) have two self-
    // consistent NR fixed points — one at each rail. The linear initial guess (or any
    // seed after failed source/Gmin stepping) may place the circuit near the wrong
    // basin. At full AOL=200,000, the VCCS produces a 200 kV NR step that flips the
    // output from one rail to the other in a single iteration, and the exponential diode
    // Jacobian then locks the solution into the non-physical equilibrium.
    //
    // Solution: two-dimensional homotopy — joint source-stepping and AOL-ramping.
    // At the lowest AOL (1.0), the VCCS contribution is negligible; we use source
    // stepping from v=0 to walk the supplies up gradually. The diode feedback at
    // near-zero supply is benign (small currents, no basin-flip). Once the sources
    // are at full scale at AOL=1, we warm-start and ramp AOL geometrically to
    // AOL_DC_MAX=1000, using a single NR solve per step (the source ramp already
    // found the physical basin).
    //
    // Only activates when the circuit contains at least one sidechain-rectifier op-amp.
    // Circuits without such topology skip this strategy entirely (no performance cost).
    let has_sidechain_rectifier = mna
        .opamps
        .iter()
        .any(|oa| dc_opamp_is_sidechain_rectifier(oa, mna));
    if has_sidechain_rectifier {
        // AOL continuation steps: start very low, ramp geometrically to AOL_DC_MAX.
        // The first step uses source stepping from v=0. Each subsequent step
        // warm-starts from the previous converged solution.
        const AOL_DC_MAX: f64 = 1000.0;
        let aol_steps: &[f64] = &[1.0, 3.0, 10.0, 30.0, 100.0, 300.0, AOL_DC_MAX];

        let mut aol_total_iters = 0;
        let mut aol_converged = true;
        let mut v_aol: Option<Vec<f64>> = None;

        for &aol in aol_steps {
            // Build a patched g_dc with this AOL level
            let g_dc_patched = patch_g_dc_for_aol(&dc_sys.g_dc, mna, aol);
            let circuit_aol = DcCircuit {
                g_dc: &g_dc_patched,
                b_dc: &dc_sys.b_dc,
                mna,
                device_slots,
                config,
                dc_n_v: &dc_sys.dc_n_v,
                dc_n_i: &dc_sys.dc_n_i,
                n_dc,
                has_internal_nodes,
                is_voltage_row: &dc_sys.is_voltage_row,
            };

            let step_ok;
            let step_iters;

            if let Some(ref prev) = v_aol {
                // Warm-start from previous AOL step's converged solution.
                // The solution is already in the physical basin, so a single NR
                // solve should converge quickly.
                let mut v_warm = prev.clone();
                let (ok, iters) = nr_dc_solve(
                    &circuit_aol,
                    &mut v_warm,
                    &mut v_nl,
                    &mut i_nl,
                    1.0,
                    0.0,
                    true,
                );
                step_ok = ok;
                step_iters = iters;
                if ok {
                    v_aol = Some(v_warm);
                }
            } else {
                // First AOL step: seed from `v_clamped` which already has op-amp
                // outputs at the expected rail and SR feedback-diode nodes at
                // v_out ± 0.65 V (applied via `seed_sr_feedback_diodes` during
                // initialization). This is the physically consistent starting
                // point for a precision rectifier at quiescent DC.
                let mut v_seed = v_clamped.clone();
                let (ok, iters) = nr_dc_solve(
                    &circuit_aol,
                    &mut v_seed,
                    &mut v_nl,
                    &mut i_nl,
                    1.0,
                    0.0,
                    true,
                );
                step_ok = ok;
                step_iters = iters;
                if ok {
                    v_aol = Some(v_seed);
                }
            }

            aol_total_iters += step_iters;
            log::info!(
                "DC OP Strategy 4 (AOL continuation, aol={:.0}): converged={} iters={}",
                aol,
                step_ok,
                step_iters
            );
            if !step_ok {
                aol_converged = false;
                break;
            }
        }

        total_iters += aol_total_iters;

        if aol_converged {
            if let Some(mut v_final) = v_aol {
                // Final solve on the full-AOL g_dc (base already at AOL_DC_MAX)
                let (converged, iters) = nr_dc_solve(
                    &circuit,
                    &mut v_final,
                    &mut v_nl,
                    &mut i_nl,
                    1.0,
                    0.0,
                    false,
                );
                total_iters += iters;
                if converged {
                    log::info!(
                        "DC OP Strategy 4 (AOL continuation): final NR converged, total_iters={}",
                        total_iters
                    );
                    v_final.truncate(n_dc);
                    return DcOpResult {
                        v_node: v_final,
                        v_nl,
                        i_nl,
                        converged: true,
                        method: DcOpMethod::AolStepping,
                        iterations: total_iters,
                    };
                }
            }
        }
    }

    // All strategies failed — return linear fallback with op-amp rail clamping.
    // The linear solution doesn't account for nonlinear devices, but seeding
    // op-amp outputs to the correct rail prevents precision rectifier circuits
    // from starting at the wrong equilibrium.
    //
    // CRITICAL: run the RAW linear guess through the same junction-voltage
    // clamp used to seed every other strategy (`clamp_junction_voltages`,
    // used above for Direct NR / Gmin seeding). Without this, an unclamped
    // linear guess can put a BJT/diode junction volts away from its knee;
    // `evaluate_devices_inner` below saturates its exp() at the codegen
    // [-40,40] clamp and returns a *finite* current that scales with the
    // device's IS (observed ~1e11 mA on a real Ge PNP repro — see
    // dc_op_ge_active_junction_threshold memory note). That runaway-but-
    // finite current bypasses every `is_finite()` guard downstream and gets
    // baked straight into `DC_NL_I`, detonating the transient on sample 1.
    // Clamping here is not a symptom mask: it is the exact seed every
    // convergent strategy already uses, applied consistently to the one
    // path that skipped it.
    let mut v_fallback = clamp_junction_voltages(mna, device_slots, &v_linear);
    v_fallback.resize(n_dc, 0.0);
    seed_opamp_outputs(&mut v_fallback, mna);

    // Precision rectifier diode consistency fixup:
    // When all NR strategies fail, the fallback state has op-amp outputs seeded
    // to VEE (correct) but the nodes connected to those outputs through feedback
    // diodes still at their linear-OP values (typically near 0V). This creates
    // v_nl(D1) = 0 - VEE = 11V → i_nl = 1e14A, which poisons DC_NL_I and causes
    // catastrophic transient NR failure on the very first sample.
    //
    // Physical constraint: for each SR op-amp, the feedback diode(s) connecting
    // n_out to n_minus MUST be forward biased at ~0.6-0.7V (the only physically
    // consistent state for a precision rectifier at quiescent DC). Therefore,
    // for each such feedback diode, if v_nl > 1V, fix it to ~0.65V by clamping
    // the non-output terminal to v_out + 0.65V.
    //
    // This is NOT a NaN mask — it enforces the Kirchhoff constraint that the
    // precision rectifier feedback loop establishes: v(diode_node) ≈ v_out + V_D.
    for oa in &mna.opamps {
        if !dc_opamp_is_sidechain_rectifier(oa, mna) {
            continue;
        }
        let out = oa.n_out_idx;
        if out == 0 {
            continue;
        }
        let o = out - 1;
        if o >= v_fallback.len() || !oa.vee.is_finite() {
            continue;
        }
        let v_out = v_fallback[o];
        // Only fix if the output is near VEE (op-amp saturated negative)
        if v_out > -1.0 {
            continue;
        }
        // For each feedback diode touching n_out, clamp the non-output
        // terminal to `v_out ± 0.65 V`. The resistor network in a precision
        // rectifier can only deliver ~500 µA, which gives a physical forward
        // drop of ~0.64 V via the Shockley equation — close enough to 0.65 V
        // for the refinement NR to pull into the correct basin in a few iters.
        for dev in &mna.nonlinear_devices {
            if dev.device_type != crate::mna::NonlinearDeviceType::Diode
                || dev.node_indices.len() < 2
            {
                continue;
            }
            let (a, k) = (dev.node_indices[0], dev.node_indices[1]);
            if a != oa.n_out_idx && k != oa.n_out_idx {
                continue;
            }
            let other_1idx = if a == oa.n_out_idx { k } else { a };
            if other_1idx == 0 {
                continue;
            }
            let other = other_1idx - 1;
            if other >= v_fallback.len() {
                continue;
            }
            let is_cathode_at_out = k == oa.n_out_idx;
            let v_other = v_fallback[other];
            let v_nl_current = if is_cathode_at_out {
                v_other - v_out
            } else {
                v_out - v_other
            };
            if v_nl_current > 1.0 {
                v_fallback[other] = if is_cathode_at_out {
                    v_out + 0.65
                } else {
                    v_out - 0.65
                };
            }
        }
    }

    // Final refinement (SR op-amp circuits only): the synthesized fallback
    // state has physically plausible node voltages (op-amp at rail, feedback-
    // diode nodes at v_out ± 0.65 V) but KCL is typically violated because we
    // only constrained a handful of nodes by topology. `evaluate_devices` on
    // this raw state yields device currents that don't balance KCL (e.g. D1
    // Vd = 0.65 V → i_D1 = 4.3 mA, but the series resistor network can only
    // deliver ~500 µA through Rrect_a_in).
    //
    // Run direct NR from this seed. For precision-rectifier circuits this
    // pulls the state into the correct basin in ~10-20 iterations (verified
    // on SSL 4kbuscomp), giving DC_OP / DC_NL_I enough KCL fidelity that the
    // transient warmup can pick up and settle to steady state instead of
    // diverging from a KCL-inconsistent start.
    //
    // Gated on `has_sidechain_rectifier` because the synthesized seed is only
    // meaningful for SR topologies — for other circuits (simple BJT amps
    // whose DC OP naturally fails) this refinement would move the fallback
    // state away from what downstream detection logic (FA classification,
    // etc.) has historically seen, causing regressions unrelated to the SR
    // fix at hand.
    let mut v_nl_fb = vec![0.0; m];
    let mut i_nl_fb = vec![0.0; m];
    let (v_out, conv_flag, conv_method) = if has_sidechain_rectifier {
        // Refinement NR in `aol_cont_mode = true`: widens the SR op-amp rail
        // to `v_nplus - 1`, giving NR enough room to escape the multi-basin
        // landscape and converge into the correct (output-near-VEE) basin.
        // Without this widening NR diverges on iter 1 (diode exponential blows
        // up when the strict VSAT clamp pins v_out but lets v_minus drift).
        //
        // NB: the converged state has v_out ≈ VEE (e.g. -12.4 V), which is
        // below the transient's hard VSAT clamp. For a purely AC-coupled
        // circuit this is fine — the VSAT clamp pulls v_out back to -11 V
        // at sample 1 and the circuit quickly settles. The 4kbuscomp
        // sidechain has a 3.3 MΩ / 10 pF integrator with τ ≈ 33 µs that
        // AMPLIFIES tiny DC offsets into unbounded CV drift; clamping DC
        // OP v_out to VSAT to match the runtime helps, but isn't strictly
        // required if the sidechain is stable enough to absorb the offset.
        let mut v_refined = v_fallback.clone();
        let (refined_ok, refine_iters) = nr_dc_solve(
            &circuit,
            &mut v_refined,
            &mut v_nl_fb,
            &mut i_nl_fb,
            1.0,
            0.0,
            true,
        );
        total_iters += refine_iters;

        if refined_ok {
            log::info!(
                "DC OP fallback-refine NR: converged after {} iters",
                refine_iters
            );
            (v_refined, true, DcOpMethod::DirectNr)
        } else if v_refined.iter().all(|x| x.is_finite()) && v_refined.iter().all(|x| x.abs() < 1e6)
        {
            log::info!(
                "DC OP fallback-refine NR: partial progress after {} iters (not formally converged)",
                refine_iters
            );
            (v_refined, false, DcOpMethod::Failed)
        } else {
            log::info!(
                "DC OP fallback-refine NR: diverged after {} iters, using raw fallback state",
                refine_iters
            );
            (v_fallback, false, DcOpMethod::Failed)
        }
    } else {
        (v_fallback, false, DcOpMethod::Failed)
    };

    extract_nl_voltages_with(m, &dc_sys.dc_n_v, &v_out, &mut v_nl_fb);
    evaluate_devices_inner(
        &v_nl_fb,
        device_slots,
        &mut i_nl_fb,
        &mut vec![0.0; m * m],
        m,
        has_internal_nodes,
        Some(&v_out),
    );

    // Final safety net: reject a finite-but-implausible result before it can
    // be baked into `DC_NL_I` / `dc_operating_point` and propagate into the
    // transient. Every strategy above already guards its own NR loop with
    // `is_finite()` checks (`nr_dc_solve`, line ~1629); this catches the
    // residual case where a *finite* value is nonetheless physically
    // impossible (see `DC_OP_MAX_PLAUSIBLE_VOLTAGE`/`_CURRENT` doc comment).
    // Falls back to the all-zero bias point — the same "structurally could
    // not determine a bias point" state already returned by the
    // `SingularLinear` path above — rather than propagate a value that
    // would detonate the transient on sample 1.
    let v_implausible = v_out
        .iter()
        .any(|x| !x.is_finite() || x.abs() > DC_OP_MAX_PLAUSIBLE_VOLTAGE);
    let i_implausible = i_nl_fb
        .iter()
        .any(|x| !x.is_finite() || x.abs() > DC_OP_MAX_PLAUSIBLE_CURRENT);
    if v_implausible || i_implausible {
        log::warn!(
            "DC OP: fallback result exceeds plausibility bounds (max|v|={:.3e}, max|i_nl|={:.3e}) \
             — rejecting and returning zero bias point instead of propagating a runaway-but-finite \
             iterate into DC_NL_I",
            v_out.iter().fold(0.0_f64, |m, x| m.max(x.abs())),
            i_nl_fb.iter().fold(0.0_f64, |m, x| m.max(x.abs())),
        );
        return DcOpResult {
            v_node: vec![0.0; n_dc],
            v_nl: vec![0.0; m],
            i_nl: vec![0.0; m],
            converged: false,
            method: DcOpMethod::Failed,
            iterations: total_iters,
        };
    }

    DcOpResult {
        v_node: v_out,
        v_nl: v_nl_fb,
        i_nl: i_nl_fb,
        converged: conv_flag,
        method: conv_method,
        iterations: total_iters,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device_types::{BjtParams, DiodeParams};
    use crate::parser::Netlist;

    // ── Finite-but-implausible fallback guard ────────────────────────
    //
    // Regression coverage for the "finite runaway" gap: a diverged DC OP
    // iterate that is astronomically large but `is_finite()` bypasses every
    // NaN/Inf guard and gets baked into `DC_NL_I`. See
    // dc_op_ge_active_junction_threshold.md memory note ("STILL OPEN").

    #[test]
    fn dc_op_failed_fallback_rejects_runaway_finite_iterate() {
        // D1's two terminals are each tied to ONE resistor and nothing
        // else. Diodes are never stamped into the LINEAR g_dc system (only
        // R/L/V are), so the pure-linear guess ignores D1 entirely: anode
        // settles near VCC (50 V) through R1, cathode settles near 0 V
        // through R2. The unclamped linear guess therefore puts D1 at
        // ~50 V forward bias — the exact kind of pathological seed that
        // detonates through the codegen exp()-clamp-at-40 if it reaches
        // `evaluate_devices_inner` unclamped.
        //
        // `max_iterations = 0` makes every NR strategy (Direct/Source/Gmin)
        // fail on its very first call (the iteration loop body never runs),
        // deterministically and cheaply forcing the "all strategies failed"
        // fallback path without needing a circuit that genuinely fails to
        // converge.
        let spice = "Diode runaway repro\n\
V1 vcc 0 DC 50\n\
R1 vcc anode 1\n\
D1 anode cat DMOD\n\
R2 cat 0 1meg\n\
.model DMOD D(IS=1e-12 N=1)\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let device_slots = crate::codegen::ir::CircuitIR::build_device_info(&netlist)
            .expect("device info build failed");
        assert_eq!(device_slots.len(), 1, "expected exactly one diode slot");

        let config = DcOpConfig {
            max_iterations: 0,
            ..DcOpConfig::default()
        };
        let result = solve_dc_operating_point(&mna, &device_slots, &config);

        assert_eq!(result.method, DcOpMethod::Failed);
        assert!(!result.converged);

        for (i, &i_nl) in result.i_nl.iter().enumerate() {
            assert!(i_nl.is_finite(), "i_nl[{i}] must be finite, got {i_nl}");
            // A junction-clamped fallback (~0.6 V) gives a milliamp-scale
            // diode current. 1 A is already generous headroom; the
            // pre-fix runaway measures in the hundreds of kiloamps
            // (IS * exp(40) with IS=1e-12 ~ 2.35e5 A).
            assert!(
                i_nl.abs() < 1.0,
                "i_nl[{i}] = {i_nl} — runaway iterate leaked into the DC_NL_I seed"
            );
        }
        for (i, &v) in result.v_node.iter().enumerate() {
            assert!(
                v.is_finite() && v.abs() < 1e3,
                "v_node[{i}] = {v} implausible for a 50 V-rail circuit"
            );
        }
    }

    // ── IC= initial-state solve ──────────────────────────────────────

    #[test]
    fn ic_seeded_op_returns_none_without_ic_caps() {
        let spice = "RC no IC\nV1 in 0 DC 0\nR1 in out 1k\nC1 out 0 10u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let config = DcOpConfig::default();
        let result = solve_ic_seeded_operating_point(&mna, &[], &config);
        assert!(
            result.is_none(),
            "circuit with no IC= capacitors must skip the extra solve"
        );
    }

    #[test]
    fn ic_seeded_op_cap_to_ground_forces_exact_node_voltage() {
        // Cap between a real node and ground: the IC constraint alone fully
        // determines that node's voltage (v(out) - v(gnd) = 5 => v(out) = 5),
        // independent of the surrounding resistor network.
        let spice = "RC discharge IC\nV1 in 0 DC 0\nR1 in out 1k\nC1 out 0 10u IC=5\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        assert_eq!(mna.capacitor_ics.len(), 1);
        let config = DcOpConfig::default();
        let result = solve_ic_seeded_operating_point(&mna, &[], &config)
            .expect("circuit has an IC= capacitor");
        assert!(result.converged, "linear IC-seeded solve must converge");
        let out_idx = mna.node_map["out"] - 1;
        assert!(
            (result.v_node[out_idx] - 5.0).abs() < 1e-9,
            "v(out) = {}, expected exactly 5.0",
            result.v_node[out_idx]
        );
    }

    #[test]
    fn ic_seeded_op_cap_between_two_nonground_nodes() {
        // Cap between two non-ground nodes (c3, b4) in a resistive divider
        // network. The IC constraint (v(c3) - v(b4) = -4) must hold exactly,
        // and the rest of the node voltages must be fully KCL-consistent
        // with it — pinned against an independently hand-derived solve of
        // the same augmented linear system (see the corresponding review
        // notes / task description for the derivation).
        let spice = "Two-node IC test\n\
V1 in 0 DC 0\n\
R1 in c3 1k\n\
R2 c3 0 10k\n\
R3 c3 b4 5k\n\
R4 b4 0 10k\n\
Cx c3 b4 6n IC=-4\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        assert_eq!(mna.capacitor_ics.len(), 1);
        let config = DcOpConfig::default();
        let result = solve_ic_seeded_operating_point(&mna, &[], &config)
            .expect("circuit has an IC= capacitor");
        assert!(result.converged, "linear IC-seeded solve must converge");

        let c3 = mna.node_map["c3"] - 1;
        let b4 = mna.node_map["b4"] - 1;
        let v_c3 = result.v_node[c3];
        let v_b4 = result.v_node[b4];

        assert!(
            (v_c3 - v_b4 - (-4.0)).abs() < 1e-9,
            "v(c3) - v(b4) = {}, expected -4.0 exactly",
            v_c3 - v_b4
        );
        // Independently hand-solved (numpy) augmented linear system for the
        // same topology — see task notes. V1 pins v(in) to exactly 0 V (an
        // augmented algebraic constraint, not a resistor), so R1 effectively
        // becomes a 1k leg from c3 to ground: v(c3) = -1/3, v(b4) = 11/3.
        // Pins the exact node voltages, not just the IC constraint itself.
        assert!(
            (v_c3 - (-1.0 / 3.0)).abs() < 1e-6,
            "v(c3) = {v_c3}, expected -1/3"
        );
        assert!(
            (v_b4 - (11.0 / 3.0)).abs() < 1e-6,
            "v(b4) = {v_b4}, expected 11/3"
        );
    }

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
        let x_expected = [1.0, 2.0, 3.0];
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

    #[test]
    fn test_solve_linear_equilibration_recovers_precision() {
        // Regression: 6-decade row imbalance (G_in = 1.0 vs internal-node
        // conductances at 1e-6 S) is the typical melange MNA shape. Without
        // equilibration, partial-pivoting LU loses ~6 digits of precision.
        // With equilibration the residual ‖Ax-b‖∞ / ‖b‖∞ stays at the f64
        // floor regardless of imbalance.
        let g_in: f64 = 1.0;
        let g_internal: f64 = 1.0e-6;
        let a = vec![
            vec![g_in + g_internal, -g_internal, 0.0],
            vec![-g_internal, 2.0 * g_internal, -g_internal],
            vec![0.0, -g_internal, g_internal + 1.0e-7],
        ];
        let b = vec![1.0, 0.0, 0.0];

        let x = solve_linear(&a, &b).unwrap();

        // Residual ‖Ax - b‖∞ / ‖b‖∞
        let mut residual_inf: f64 = 0.0;
        for i in 0..3 {
            let row_dot: f64 = (0..3).map(|j| a[i][j] * x[j]).sum();
            let r = (row_dot - b[i]).abs();
            if r > residual_inf {
                residual_inf = r;
            }
        }
        let b_norm: f64 = b.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
        let rel_residual = residual_inf / b_norm;

        // Pre-equilibration this was around 1e-10. Post-equilibration it
        // should be at the f64 floor (~1e-15). Pin at 1e-13 to leave headroom
        // for unrelated FP noise.
        assert!(
            rel_residual < 1.0e-13,
            "equilibrated solve residual too large: {} (matrix has 6-decade row imbalance)",
            rel_residual
        );
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
                rth: f64::INFINITY,
                cth: 1e-3,
                xti: 3.0,
                eg: 1.11,
                tamb: 300.15,
            }),
            has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
            stateful: None,
        };

        let m = 1;
        let v_nl = vec![0.6]; // Forward biased
        let mut i_nl = vec![0.0];
        let mut j_dev = vec![0.0];

        evaluate_devices(&v_nl, &[slot], &mut i_nl, &mut j_dev, m);

        // Compare against DiodeShockley + device Gmin (1e-12 S)
        let diode = DiodeShockley::new(is, 1.0, n_vt);
        let gmin = 1e-12;
        let v = 0.6;
        let expected_i = diode.current_at(v) + gmin * v;
        let expected_g = diode.conductance_at(v) + gmin;

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
                tf: 0.0,
                vje: 0.75,
                mje: 0.33,
                vjc: 0.75,
                mjc: 0.33,
                fc: 0.5,
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
            vg2k_frozen: 0.0,
            stateful: None,
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
                rth: f64::INFINITY,
                cth: 1e-3,
                xti: 3.0,
                eg: 1.11,
                tamb: 300.15,
            }),
            has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
            stateful: None,
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
                    rth: f64::INFINITY,
                    cth: 1e-3,
                    xti: 3.0,
                    eg: 1.11,
                    tamb: 300.15,
                }),
                has_internal_mna_nodes: false,
                vg2k_frozen: 0.0,
                stateful: None,
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
                    rth: f64::INFINITY,
                    cth: 1e-3,
                    xti: 3.0,
                    eg: 1.11,
                    tamb: 300.15,
                }),
                has_internal_mna_nodes: false,
                vg2k_frozen: 0.0,
                stateful: None,
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
                    rth: f64::INFINITY,
                    cth: 1e-3,
                    xti: 3.0,
                    eg: 1.11,
                    tamb: 300.15,
                }),
                has_internal_mna_nodes: false,
                vg2k_frozen: 0.0,
                stateful: None,
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

    // ── AOL homotopy patch polarity regression ───────────────────────

    /// Pins the sign convention of `patch_g_dc_for_aol` so a future
    /// "harmonization" can't silently flip it back.
    ///
    /// The MNA VCCS stamp is `G[out][n_plus] -= Gm`, `G[out][n_minus] += Gm`
    /// (current LEAVING `out` is −Gm·V+ + Gm·V−). `build_dc_system` undoes
    /// the excess above AOL_DC_MAX by REVERSING those signs (`+= delta` /
    /// `-= delta`), and `patch_g_dc_for_aol` applies a further undo down to
    /// `aol_step` with the SAME reversal. Therefore after patching, the
    /// effective entry must be
    ///     g[o][np−1] == −(aol_step / r_out) + (base non-opamp contribution)
    ///     g[o][nm−1] == +(aol_step / r_out) + (base non-opamp contribution)
    /// where the base contributions here are the (negative off-diagonal)
    /// resistor stamps. A polarity flip would instead ADD conductance and
    /// blow both assertions up by ~2·Gm.
    #[test]
    fn test_patch_g_dc_for_aol_polarity_pinned() {
        use crate::mna::OpampInfo;

        // Minimal 3-node system: n_plus = node 1, n_minus = node 2,
        // out = node 3 (1-based MNA indices; matrix rows are index−1).
        let aol = 200_000.0;
        let r_out = 50.0;
        let mut mna = MnaSystem::new(3, 0, 0, 0);
        mna.opamps.push(OpampInfo {
            name: "U_TEST".to_string(),
            n_plus_idx: 1,
            n_minus_idx: 2,
            n_out_idx: 3,
            aol,
            r_out,
            vsat: f64::INFINITY,
            vcc: f64::INFINITY,
            vee: f64::NEG_INFINITY,
            voh_drop: 1.5,
            vol_drop: 1.5,
            gbw: f64::INFINITY,
            sr: f64::INFINITY,
            ib: 0.0,
            rin: f64::INFINITY,
            aol_transient_cap: f64::INFINITY,
            n_internal_idx: 0,
            iir_c_dom: 0.0,
            n_int_idx: 0,
            en: 0.0,
            in_amps: 0.0,
            en_fc: 0.0,
            in_fc: 0.0,
        });

        // Base non-opamp contributions: feedback resistors from `out` to each
        // input. Standard resistor stamp — off-diagonals are NEGATIVE.
        let g_fb_np = 5e-4; // 2 kΩ out ↔ n_plus
        let g_fb_nm = 1e-3; // 1 kΩ out ↔ n_minus
        mna.g[2][0] -= g_fb_np;
        mna.g[0][2] -= g_fb_np;
        mna.g[0][0] += g_fb_np;
        mna.g[2][2] += g_fb_np;
        mna.g[2][1] -= g_fb_nm;
        mna.g[1][2] -= g_fb_nm;
        mna.g[1][1] += g_fb_nm;
        mna.g[2][2] += g_fb_nm;

        // Full-AOL VCCS stamp, exactly as MnaSystem::build's simple-VCCS
        // path does it (mna.rs "Simple VCCS (no GBW)" arm).
        let gm_full = aol / r_out;
        mna.g[2][0] -= gm_full;
        mna.g[2][1] += gm_full;
        mna.g[2][2] += 1.0 / r_out;

        // build_dc_system bakes the AOL_DC_MAX=1000 cap into base g_dc.
        let config = DcOpConfig::default();
        let dc_sys = build_dc_system(&mna, &[], &config);
        const AOL_DC_MAX: f64 = 1000.0; // must match dc_op.rs constants
        let gm_capped = AOL_DC_MAX / r_out;
        assert!(
            (dc_sys.g_dc[2][0] - (-gm_capped - g_fb_np)).abs() < 1e-9,
            "base g_dc[out][np] should be −Gm_capped − g_fb_np, got {}",
            dc_sys.g_dc[2][0]
        );

        // Homotopy step: lower AOL further to aol_step.
        let aol_step = 10.0;
        let g = patch_g_dc_for_aol(&dc_sys.g_dc, &mna, aol_step);
        let gm_step = aol_step / r_out;

        // THE pinned polarity: effective VCCS at n_plus column stays NEGATIVE
        // (−Gm·V+ leaving-current convention), reduced in magnitude to
        // aol_step/r_out, with the base resistor stamp preserved.
        assert!(
            (g[2][0] - (-gm_step - g_fb_np)).abs() < 1e-9,
            "patched g[out][np] must be −(aol_step/r_out) − g_fb_np = {}, got {}",
            -gm_step - g_fb_np,
            g[2][0]
        );
        assert!(
            g[2][0] < 0.0,
            "patched g[out][np] must stay negative (VCCS polarity flipped!)"
        );
        // Mirror column: n_minus stays POSITIVE.
        assert!(
            (g[2][1] - (gm_step - g_fb_nm)).abs() < 1e-9,
            "patched g[out][nm] must be +(aol_step/r_out) − g_fb_nm = {}, got {}",
            gm_step - g_fb_nm,
            g[2][1]
        );
        // Output self-conductance and all other entries untouched by the patch.
        assert!(
            (g[2][2] - dc_sys.g_dc[2][2]).abs() < 1e-12,
            "patch must not touch g[out][out]"
        );
        assert!(
            (g[0][2] - dc_sys.g_dc[0][2]).abs() < 1e-12,
            "patch must not touch input rows"
        );
    }

    // =========================================================================
    // pnjlim correction distribution — pseudo-inverse normalization (2026-07-18)
    // =========================================================================

    /// Grounded junction (N_v row = [1, 0, ...], ‖row‖² = 1): the applied
    /// junction-space change must equal the correction exactly. This was
    /// already exact pre-fix; guard that the normalization didn't break it.
    #[test]
    fn test_distribute_correction_grounded_junction_exact() {
        let row = vec![1.0, 0.0, 0.0];
        let mut corrections = vec![0.0; 3];
        let correction = -4.366; // strong pnjlim event: v_lim - v_raw
        distribute_junction_correction(&row, correction, &mut corrections);
        // Applied junction-space change = row · corrections
        let applied: f64 = row.iter().zip(&corrections).map(|(a, b)| a * b).sum();
        assert!(
            (applied - correction).abs() < 1e-12,
            "grounded junction: applied {} != correction {}",
            applied,
            correction
        );
    }

    /// Floating junction (N_v row = [1, -1, 0], ‖row‖² = 2): the raw transpose
    /// distribution applied 2× the correction, turning a limited step into a
    /// REVERSED step (v_old=0.5, raw 5.0, v_lim=0.634 landed at −3.73 V — the
    /// period-2 NR oscillation signature). With the pseudo-inverse
    /// normalization the post-limit junction voltage must equal v_lim exactly.
    #[test]
    fn test_distribute_correction_floating_junction_lands_on_v_lim() {
        let row = vec![1.0, -1.0, 0.0];
        let v_old = 0.5_f64;
        let v_raw = 5.0_f64;
        // Realistic pnjlim outcome for a strong forward step (vt=0.026):
        let v_lim = pnjlim(v_raw, v_old, 0.026, 0.615);
        assert!(v_lim < 1.0, "sanity: pnjlim should compress the 5 V step");
        let correction = v_lim - v_raw;

        let mut corrections = vec![0.0; 3];
        distribute_junction_correction(&row, correction, &mut corrections);
        let applied: f64 = row.iter().zip(&corrections).map(|(a, b)| a * b).sum();
        let v_post = v_raw + applied;
        assert!(
            (v_post - v_lim).abs() < 1e-12,
            "floating junction must land exactly on v_lim={}, got {} \
             (raw-transpose doubling would give {})",
            v_lim,
            v_post,
            v_raw + 2.0 * correction
        );
        // And explicitly: the step must NOT reverse direction past v_old.
        assert!(
            v_post > v_old,
            "post-limit voltage {} reversed past v_old {} — doubling bug",
            v_post,
            v_old
        );
    }

    /// Zero N_v row (tied terminals): no node update can change the junction
    /// voltage — the distribution must be a no-op, not a division by zero.
    #[test]
    fn test_distribute_correction_zero_row_noop() {
        let row = vec![0.0, 0.0, 0.0];
        let mut corrections = vec![0.0; 3];
        distribute_junction_correction(&row, -1.0, &mut corrections);
        assert!(
            corrections.iter().all(|&c| c == 0.0),
            "zero row must not distribute anything, got {:?}",
            corrections
        );
    }
}
