//! A PNP's junction capacitances are evaluated at its junctions' forward
//! voltages, the same as an NPN's.
//!
//! The DC-OP re-linearization of CJE/CJC fed the terminal differences
//! V(b) − V(e) and V(b) − V(c) straight into the depletion formula, which
//! takes forward voltage as positive. For a PNP both signs are flipped: its
//! reverse-biased B-C junction was evaluated as forward biased, past the
//! FC·VJ knee on the tangent extension, and its forward-biased B-E junction
//! as reverse biased. A PNP common-emitter stage with CJC = CJE = 20 pF read
//! 10 dB low at 10 kHz and 18 dB low at 100 kHz, where its NPN mirror and
//! ngspice (identical for both polarities) agree.
//!
//! Oracle: mirror symmetry. A PNP stage on a negative supply is the NPN
//! stage with every voltage and current negated, so its capacitance matrix
//! is the NPN's exactly. The B-C value is also checked against ngspice's
//! `cmu` at this bias point.

use melange_solver::build::preflight_relinearize_bjt_caps;
use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::OpampRailMode;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const NPN: &str = "npn common emitter
.model QN NPN(IS=1.5e-14 BF=200 VAF=50 CJC=20p CJE=20p)
VCC vcc 0 DC 12
Cin in b 10u
R1 vcc b 100k
R2 b 0 22k
RC vcc out 4.7k
Q1 out b e QN
RE e 0 1k
CE e 0 10m
";

const PNP: &str = "pnp common emitter, the npn stage mirrored
.model QP PNP(IS=1.5e-14 BF=200 VAF=50 CJC=20p CJE=20p)
VCC vcc 0 DC -12
Cin in b 10u
R1 vcc b 100k
R2 b 0 22k
RC vcc out 4.7k
Q1 out b e QP
RE e 0 1k
CE e 0 10m
";

/// The capacitance matrix after the zero-bias junction caps are stamped
/// and re-linearized at the DC operating point, as the build does.
fn relinearized_c(deck: &str) -> (MnaSystem, Vec<Vec<f64>>) {
    let netlist = Netlist::parse(deck).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    mna.stamp_device_junction_caps(&slots);
    let dc = preflight_relinearize_bjt_caps(&mut mna, &netlist, OpampRailMode::Auto.into())
        .expect("a BJT to re-linearize");
    assert!(dc.converged);
    let c = mna.c.clone();
    (mna, c)
}

#[test]
fn a_pnp_stage_has_its_npn_mirror_s_junction_caps() {
    let (mna_n, c_n) = relinearized_c(NPN);
    let (mna_p, c_p) = relinearized_c(PNP);
    assert_eq!(mna_n.node_map, mna_p.node_map);
    for (i, (rn, rp)) in c_n.iter().zip(&c_p).enumerate() {
        for (j, (&n, &p)) in rn.iter().zip(rp).enumerate() {
            assert!(
                (n - p).abs() <= 1e-9 * n.abs().max(1e-15),
                "C[{i}][{j}]: npn {n:e}, pnp {p:e}"
            );
        }
    }
}

#[test]
fn the_reverse_biased_bc_cap_is_ngspice_s_cmu_for_both_polarities() {
    // ngspice .op for this stage: vbc = -3.4258 V (polarity-normalised),
    // cmu = 1.1349e-11 F, i.e. CJC · (1 + 3.4258/0.75)^(-0.33).
    for deck in [NPN, PNP] {
        let (mna, c) = relinearized_c(deck);
        let b = mna.node_map["b"] - 1;
        let out = mna.node_map["out"] - 1;
        let cbc = -c[b][out];
        assert!(
            (cbc - 1.1349e-11).abs() <= 1e-3 * 1.1349e-11,
            "C_bc = {cbc:e}, ngspice cmu 1.1349e-11"
        );
    }
}
