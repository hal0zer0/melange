//! A BJT's small-signal capacitances at its DC operating point match
//! ngspice's `.op` values (`capbe`, `capbc`), with the parts of the model
//! that move them live.
//!
//! - Diffusion: `C_de = TF·d(I_F/qb)/dVbe`, ngspice's `capbe = tf*gbe`. It
//!   was `TF·|Ic|/Vt`, which ignores NF (1.5× high at NF = 1.5) and qb
//!   (20 % high with IKF = 5 mA at 1.4 mA).
//! - A forward-active (1D) BJT's B-C depletion cap was evaluated at
//!   Vbc = 0, since that slot tracks only Vbe: CJC instead of its value at
//!   the bias (1.77× at Vbc = −3.5 V).
//!
//! References: ngspice `.op` on the same stage (CJE = 0 where capbe is
//! checked, so it is the diffusion term alone).

use melange_solver::build::preflight_relinearize_bjt_caps;
use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::{BjtFaMode, CodegenConfig, OpampRailMode};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn stage(model: &str) -> String {
    format!(
        "common emitter
VCC vcc 0 DC 12
R1 vcc b 100k
R2 b 0 22k
RC vcc c 4.7k
Q1 c b e QX
RE e 0 1k
.model QX NPN({model})
"
    )
}

/// (C_be, C_bc, M) after the zero-bias caps are stamped and re-linearized at
/// the DC operating point, as the build does.
fn caps(netlist: &Netlist, mut mna: MnaSystem) -> (f64, f64, usize) {
    let slots = CircuitIR::build_device_info_with_mna(netlist, Some(&mna)).unwrap();
    mna.stamp_device_junction_caps(&slots);
    let dc = preflight_relinearize_bjt_caps(&mut mna, netlist, OpampRailMode::Auto.into())
        .expect("a BJT to re-linearize");
    assert!(dc.converged);
    let (b, c, e) = (
        mna.node_map["b"] - 1,
        mna.node_map["c"] - 1,
        mna.node_map["e"] - 1,
    );
    (-mna.c[b][e], -mna.c[b][c], mna.m)
}

fn assert_close(what: &str, got: f64, want: f64) {
    assert!(
        (got - want).abs() <= 2e-3 * want,
        "{what}: melange {got:e}, ngspice {want:e}"
    );
}

#[test]
fn diffusion_cap_carries_nf() {
    let netlist = Netlist::parse(&stage("IS=1e-14 BF=200 NF=1.5 TF=1n")).unwrap();
    let (cbe, _, _) = caps(&netlist, MnaSystem::from_netlist(&netlist).unwrap());
    assert_close("capbe, NF = 1.5", cbe, 2.77343e-11);
}

#[test]
fn diffusion_cap_carries_qb() {
    let netlist = Netlist::parse(&stage("IS=1.5e-14 BF=200 VAF=50 IKF=5m TF=1n")).unwrap();
    let (cbe, _, _) = caps(&netlist, MnaSystem::from_netlist(&netlist).unwrap());
    assert_close("capbe, IKF = 5 mA, VAF = 50", cbe, 4.36335e-11);
}

#[test]
fn forward_active_bc_cap_is_evaluated_at_its_bias() {
    let netlist = Netlist::parse(&stage("IS=1.5e-14 BF=200 CJC=20p")).unwrap();
    let full = MnaSystem::from_netlist(&netlist).unwrap();

    // `--bjt-fa auto`: the reduction is opt-in.
    let config = CodegenConfig {
        bjt_fa_mode: BjtFaMode::Auto,
        ..CodegenConfig::default()
    };
    let fa = CircuitIR::detect_forward_active_bjts(&full, &netlist, &config);
    assert!(
        fa.contains("Q1"),
        "the stage must be FA-reduced or this is no witness"
    );
    let reduced = MnaSystem::from_netlist_forward_active(&netlist, &fa).unwrap();

    let (_, cbc_full, m_full) = caps(&netlist, full);
    let (_, cbc_fa, m_fa) = caps(&netlist, reduced);
    assert_eq!((m_full, m_fa), (2, 1));
    // ngspice: vbc = -3.47601 V, cmu = CJC·(1 + 3.47601/0.75)^(-0.33).
    assert_close("capbc, full 2D", cbc_full, 1.13043e-11);
    assert_close("capbc, forward-active 1D", cbc_fa, 1.13043e-11);
}
