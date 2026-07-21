//! Regression tests for MNA-layer stamping fixes (2026-07):
//!
//! 1. N_v/N_i device stamps ACCUMULATE so tied terminals cancel to zero
//!    (diode-connected BJTs, gate-source-strapped JFETs) instead of the
//!    second write overwriting the first (phantom controlling voltage).
//! 2. Pentode junction caps (CCG/CGP/CCP) use the pentode node layout
//!    [plate, grid, cathode, screen], not the triode layout.
//! 3. A_neg blanket-zeroes ALL augmented algebraic rows (VS, VCVS,
//!    current-mode VCA internal+sense, behavioral-V), not just the
//!    VS/VCVS/ideal-xfmr enumeration.
//! 4. Linearized BJT/triode DC injections carry the proper Norton constant
//!    I_lin(v0) - I0 so the linearized circuit's DC fixed point IS the
//!    nonlinear operating point.
//! 5. BoyleDiodes op-amps still get IB/RIN input parasitics (the mode
//!    dispatch used to `continue` before the IB/RIN stamps).

use std::collections::{HashMap, HashSet};

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::device_types::DeviceParams;
use melange_solver::mna::{LinearizedBjtInfo, LinearizedTriodeInfo, MnaSystem};
use melange_solver::parser::Netlist;

fn node(mna: &MnaSystem, name: &str) -> usize {
    *mna.node_map
        .get(name)
        .unwrap_or_else(|| panic!("node '{}' not in node_map", name))
}

// ────────────────────────────────────────────────────────────────────
// Fix 1: tied-terminal N_v/N_i accumulation
// ────────────────────────────────────────────────────────────────────

/// Diode-connected BJT (`Q1 x x 0`): the Vbc N_v row must cancel to all
/// zeros (Vbc = 0 exactly), and the DC OP must match an electrically
/// identical circuit where the base-collector tie is an explicit 0 V
/// voltage source (a genuine Vbc = 0 evaluation).
#[test]
fn diode_connected_bjt_dc_op_matches_explicit_vbc_zero() {
    let tied = "\
Diode-Connected BJT (tied)
V1 vcc 0 DC 5
R1 vcc x 4.7k
Q1 x x 0 QN
.model QN NPN(IS=1e-14 BF=200)
";
    let netlist_a = Netlist::parse(tied).expect("parse tied netlist");
    let mna_a = MnaSystem::from_netlist(&netlist_a).expect("build tied MNA");

    let dev = &mna_a.nonlinear_devices[0];
    let s = dev.start_idx;
    let x = node(&mna_a, "x");

    // Vbc row (s+1): base and collector are the same node — the +1 and -1
    // must cancel to a zero row (previously the -1 overwrote the +1 and the
    // device saw a phantom Vbc = -V(x)).
    for (j, &v) in mna_a.n_v[s + 1].iter().enumerate() {
        assert!(
            v.abs() < 1e-30,
            "Vbc N_v row must be all zeros for a diode-connected BJT, found {} at col {}",
            v,
            j
        );
    }
    // Vbe row unchanged: +1 at x (emitter grounded).
    assert!((mna_a.n_v[s][x - 1] - 1.0).abs() < 1e-12);

    let slots_a = CircuitIR::build_device_info_with_mna(&netlist_a, Some(&mna_a))
        .expect("device slots (tied)");
    let dc_a = solve_dc_operating_point(&mna_a, &slots_a, &DcOpConfig::default());
    assert!(dc_a.converged, "tied-BJT DC OP must converge");

    // Vbc extraction is exactly zero by construction.
    assert_eq!(dc_a.v_nl[s + 1], 0.0, "Vbc must extract as exactly 0");
    let vx_a = dc_a.v_node[x - 1];
    assert!(
        (0.4..0.85).contains(&vx_a),
        "diode-connected BJT should sit at a diode drop, got {} V",
        vx_a
    );

    // KCL at x: current through R1 equals Ic + Ib evaluated at (Vbe=vx, Vbc=0).
    let i_r1 = (5.0 - vx_a) / 4.7e3;
    let i_dev = dc_a.i_nl[s] + dc_a.i_nl[s + 1];
    assert!(
        (i_r1 - i_dev).abs() < 1e-7,
        "KCL at tied node: R current {} vs device current {}",
        i_r1,
        i_dev
    );

    // Reference: same circuit with the tie made explicit via a 0 V source,
    // so the device genuinely evaluates at Vbc = V(b) - V(c) = 0.
    let explicit = "\
Diode-Connected BJT (explicit tie)
V1 vcc 0 DC 5
R1 vcc c 4.7k
Vtie c b DC 0
Q1 c b 0 QN
.model QN NPN(IS=1e-14 BF=200)
";
    let netlist_b = Netlist::parse(explicit).expect("parse explicit netlist");
    let mna_b = MnaSystem::from_netlist(&netlist_b).expect("build explicit MNA");
    let slots_b = CircuitIR::build_device_info_with_mna(&netlist_b, Some(&mna_b))
        .expect("device slots (explicit)");
    let dc_b = solve_dc_operating_point(&mna_b, &slots_b, &DcOpConfig::default());
    assert!(dc_b.converged, "explicit-tie DC OP must converge");

    let c_b = node(&mna_b, "c");
    let vx_b = dc_b.v_node[c_b - 1];
    assert!(
        (vx_a - vx_b).abs() < 1e-6,
        "tied ({} V) and explicit-0V-tie ({} V) circuits must agree",
        vx_a,
        vx_b
    );
}

/// JFET with gate strapped to source (`J1 d ctl ctl`): Vgs row and Ig
/// column must cancel to zero and the device must conduct at IDSS
/// (Vgs = 0, saturation, LAMBDA = 0).
#[test]
fn jfet_gate_source_strapped_conducts_at_idss() {
    let spice = "\
JFET current source (G-S strap)
V1 vdd 0 DC 12
R1 vdd d 1k
J1 d ctl ctl NJ1
R2 ctl 0 1k
.model NJ1 NJF(VTO=-2.0 IDSS=2e-3)
";
    let netlist = Netlist::parse(spice).expect("parse JFET netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("build JFET MNA");

    let dev = &mna.nonlinear_devices[0];
    let s = dev.start_idx;

    // Row s+1 extracts Vgs: gate and source are the same node → all zeros.
    for (j, &v) in mna.n_v[s + 1].iter().enumerate() {
        assert!(
            v.abs() < 1e-30,
            "Vgs N_v row must cancel to zero for a G-S strap, found {} at col {}",
            v,
            j
        );
    }
    // Col s+1 injects Ig between gate and source: same node → all zeros.
    for (i, row) in mna.n_i.iter().enumerate() {
        assert!(
            row[s + 1].abs() < 1e-30,
            "Ig N_i column must cancel to zero for a G-S strap, found {} at row {}",
            row[s + 1],
            i
        );
    }

    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).expect("device slots");
    let dc = solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
    assert!(dc.converged, "JFET DC OP must converge");

    // Vgs = 0 exactly; Id = IDSS·(1 + lambda·Vds) — saturation at zero bias.
    // (The JFET model carries a nonzero default LAMBDA, so fold in the
    // channel-length-modulation term at the solved Vds.)
    assert_eq!(dc.v_nl[s + 1], 0.0, "Vgs must extract as exactly 0");
    let id = dc.i_nl[s];
    let jp = match &slots[0].params {
        DeviceParams::Jfet(jp) => jp,
        other => panic!("expected JFET params, got {:?}", other),
    };
    let vds = dc.v_nl[s];
    let expected = jp.idss * (1.0 + jp.lambda * vds);
    assert!(
        (id - expected).abs() < 2e-6,
        "G-S-strapped JFET must conduct at IDSS·(1+lambda·Vds) = {} A, got {} A",
        expected,
        id
    );
    assert!(
        (id - 2e-3).abs() < 5e-5,
        "Id should sit at ~IDSS = 2 mA (small lambda correction only), got {} A",
        id
    );

    // KCL sanity: drain resistor carries Id, source resistor lifts ctl to Id*R2.
    let d = node(&mna, "d");
    let ctl = node(&mna, "ctl");
    let vd = dc.v_node[d - 1];
    let vctl = dc.v_node[ctl - 1];
    assert!(((12.0 - vd) / 1e3 - id).abs() < 1e-6);
    assert!((vctl - id * 1e3).abs() < 1e-5);
}

// ────────────────────────────────────────────────────────────────────
// Fix 2: pentode junction-cap node layout
// ────────────────────────────────────────────────────────────────────

/// Pentode CCG/CGP/CCP must land on grid-cathode / grid-plate /
/// plate-cathode using the pentode node layout [plate, grid, cathode,
/// screen]. The old code assumed the triode layout [grid, plate, cathode],
/// putting CCG on plate-cathode and CCP on grid-cathode.
#[test]
fn pentode_junction_caps_use_pentode_node_layout() {
    let spice = "\
EL84 junction caps
Rg grid 0 1Meg
P1 plate grid cathode screen EL84
Rk cathode 0 130
Rscr vcc screen 1k
Rp vcc plate 4.7k
V1 vcc 0 DC 300
.model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275.0 KP=152.4 KVB=4015.8 ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148 CCG=1e-12 CGP=2e-12 CCP=3e-12)
";
    let netlist = Netlist::parse(spice).expect("parse pentode netlist");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("build pentode MNA");
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).expect("device slots");

    let g = node(&mna, "grid") - 1;
    let p = node(&mna, "plate") - 1;
    let k = node(&mna, "cathode") - 1;

    let c_before = mna.c.clone();
    mna.stamp_device_junction_caps(&slots);

    let ccg = 1e-12;
    let cgp = 2e-12;
    let ccp = 3e-12;
    let eps = 1e-18;

    // Off-diagonal couplings (negative of the cap value).
    assert!(
        (mna.c[g][k] - c_before[g][k] + ccg).abs() < eps,
        "CCG must couple grid-cathode: delta C[g][k] = {}",
        mna.c[g][k] - c_before[g][k]
    );
    assert!(
        (mna.c[g][p] - c_before[g][p] + cgp).abs() < eps,
        "CGP must couple grid-plate: delta C[g][p] = {}",
        mna.c[g][p] - c_before[g][p]
    );
    assert!(
        (mna.c[p][k] - c_before[p][k] + ccp).abs() < eps,
        "CCP must couple plate-cathode: delta C[p][k] = {}",
        mna.c[p][k] - c_before[p][k]
    );

    // Diagonals accumulate every cap touching the node.
    assert!((mna.c[g][g] - c_before[g][g] - (ccg + cgp)).abs() < eps);
    assert!((mna.c[p][p] - c_before[p][p] - (cgp + ccp)).abs() < eps);
    assert!((mna.c[k][k] - c_before[k][k] - (ccg + ccp)).abs() < eps);
}

// ────────────────────────────────────────────────────────────────────
// Fix 3: A_neg blanket zero of ALL augmented algebraic rows
// ────────────────────────────────────────────────────────────────────

/// A_neg rows for EVERY augmented algebraic row — VS, VCVS, current-mode
/// VCA (internal sig+_int node + sensing branch), and behavioral `V={}`
/// branch — must be all zeros. The old per-type enumeration missed the VCA
/// and behavioral rows, leaving stale -G history on algebraic constraints.
#[test]
fn a_neg_zeroes_all_augmented_rows_including_vca_and_behavioral() {
    let spice = "\
Aneg blanket zero
V1 vcc 0 DC 15
E1 e_out 0 in 0 2.0
Rin in 0 10k
Rvcc vcc 0 100k
Re e_out 0 10k
Rdrv in sigp 10k
Y1 sigp sigm cv 0 VCAM
Rterm sigm 0 10k
Rcv cv 0 10k
Csig sigm 0 10n
B1 bx 0 V={ tanh(V(in)) }
Rb bx 0 1k
.model VCAM VCA(MODE=1)
";
    let netlist = Netlist::parse(spice).expect("parse A_neg netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("build A_neg MNA");

    // Layout: 1 VS + 1 VCVS + 2 current-mode-VCA rows + 1 behavioral-V row.
    assert_eq!(
        mna.n_aug,
        mna.n + 5,
        "expected 5 augmented rows (VS + VCVS + VCA internal + VCA sense + behavioral-V)"
    );

    let a = mna.get_a_matrix(48000.0).expect("A matrix");
    let a_neg = mna.get_a_neg_matrix(48000.0).expect("A_neg matrix");

    for row in mna.n..mna.n_aug {
        // The forward matrix keeps the constraint/branch coupling…
        assert!(
            a[row].iter().any(|v| v.abs() > 1e-30),
            "A row {} should carry the algebraic constraint stamps",
            row
        );
        // …but A_neg must be zero: algebraic rows have no trapezoidal history.
        for (j, &v) in a_neg[row].iter().enumerate() {
            assert!(
                v == 0.0,
                "A_neg[{}][{}] = {} — augmented algebraic row must be blanket-zeroed \
                 (VCA internal/sense and behavioral-V rows were previously missed)",
                row,
                j,
                v
            );
        }
    }
}

// ────────────────────────────────────────────────────────────────────
// Fix 4: linearized-device Norton constants
// ────────────────────────────────────────────────────────────────────

/// The linearized circuit's DC solution must reproduce the nonlinear
/// operating point at every node. With the Norton constant I_lin(v0) - I0
/// this holds for ANY conductance values (the g's only set sensitivity,
/// not the fixed point); the old raw-I0 injection left the g·v0 term
/// uncancelled and shifted the bias by volts.
#[test]
fn linearized_bjt_dc_fixed_point_matches_nonlinear_op() {
    let spice = "\
CE stage for linearization
V1 vcc 0 DC 9
R1 vcc b 47k
R2 b 0 10k
RC vcc c 4.7k
RE e 0 1k
Q1 c b e QN
.model QN NPN(IS=1e-14 BF=200)
";
    let netlist = Netlist::parse(spice).expect("parse CE netlist");

    // Nonlinear reference OP.
    let mna_nl = MnaSystem::from_netlist(&netlist).expect("build nonlinear MNA");
    let slots =
        CircuitIR::build_device_info_with_mna(&netlist, Some(&mna_nl)).expect("device slots");
    let dc_nl = solve_dc_operating_point(&mna_nl, &slots, &DcOpConfig::default());
    assert!(dc_nl.converged, "nonlinear DC OP must converge");

    let dev = &mna_nl.nonlinear_devices[0];
    let s = dev.start_idx;
    let (nc, nb, ne) = (
        dev.node_indices[0],
        dev.node_indices[1],
        dev.node_indices[2],
    );
    let v_at = |idx: usize| -> f64 {
        if idx > 0 {
            dc_nl.v_node[idx - 1]
        } else {
            0.0
        }
    };
    let ic = dc_nl.i_nl[s];
    let ib = dc_nl.i_nl[s + 1];
    assert!(ic > 1e-5, "CE stage should be biased active, Ic = {}", ic);

    // Plausible small-signal conductances. The fixed-point invariant holds
    // for any values — only the Norton constant matters.
    let bp = match &slots[0].params {
        DeviceParams::Bjt(bp) => bp,
        other => panic!("expected BJT params, got {:?}", other),
    };
    let gm = ic / (bp.nf * bp.vt);
    let gpi = gm / bp.beta_f;
    let gmu = 1e-9;
    let go = 1e-9;

    // Linearized rebuild + stamp (mirrors the CLI .linearize flow).
    let mut lin_set = HashSet::new();
    lin_set.insert("Q1".to_string());
    let mut mna_lin = MnaSystem::from_netlist_with_all_reductions(
        &netlist,
        &HashSet::new(),
        &lin_set,
        &HashSet::new(),
        &HashMap::new(),
    )
    .expect("build linearized MNA");
    assert_eq!(mna_lin.m, 0, "linearized BJT should leave M = 0");

    mna_lin.linearized_bjts = vec![LinearizedBjtInfo {
        name: "Q1".to_string(),
        nc,
        nb,
        ne,
        gm,
        gpi,
        gmu,
        go,
        ic_dc: ic,
        ib_dc: ib,
        vbe0: v_at(nb) - v_at(ne),
        vbc0: v_at(nb) - v_at(nc),
    }];
    mna_lin.stamp_linearized_bjts();

    // Linear DC solve of the linearized circuit.
    let dc_lin = solve_dc_operating_point(&mna_lin, &[], &DcOpConfig::default());
    assert!(dc_lin.converged, "linearized DC solve must converge");

    for name in ["b", "c", "e"] {
        let idx = node(&mna_nl, name) - 1;
        let v_ref = dc_nl.v_node[idx];
        let v_lin = dc_lin.v_node[idx];
        assert!(
            (v_ref - v_lin).abs() < 1e-5,
            "linearized DC at node '{}' must match nonlinear OP: {} vs {}",
            name,
            v_lin,
            v_ref
        );
    }
}

/// Same invariant for a linearized 12AX7 stage. Pre-fix, the missing
/// gm·Vgk0 + gp·Vpk0 subtraction injected ~mA of phantom current into the
/// plate row (tens of volts of bias shift through a 100k plate load).
#[test]
fn linearized_triode_dc_fixed_point_matches_nonlinear_op() {
    let spice = "\
12AX7 stage for linearization
V1 vcc 0 DC 250
RP vcc p 100k
RG g 0 1Meg
RK k 0 1.5k
T1 g p k 12AX7
.model 12AX7 VT(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";
    let netlist = Netlist::parse(spice).expect("parse triode netlist");

    let mna_nl = MnaSystem::from_netlist(&netlist).expect("build nonlinear MNA");
    let slots =
        CircuitIR::build_device_info_with_mna(&netlist, Some(&mna_nl)).expect("device slots");
    let dc_nl = solve_dc_operating_point(&mna_nl, &slots, &DcOpConfig::default());
    assert!(dc_nl.converged, "nonlinear triode DC OP must converge");

    let dev = &mna_nl.nonlinear_devices[0];
    let s = dev.start_idx;
    let (ng, np, nk) = (
        dev.node_indices[0],
        dev.node_indices[1],
        dev.node_indices[2],
    );
    let v_at = |idx: usize| -> f64 {
        if idx > 0 {
            dc_nl.v_node[idx - 1]
        } else {
            0.0
        }
    };
    let ip_dc = dc_nl.i_nl[s];
    let ig_dc = dc_nl.i_nl[s + 1];
    assert!(ip_dc > 1e-5, "triode should conduct, Ip = {}", ip_dc);

    // Plausible 12AX7 small-signal values (fixed point is g-independent).
    let gm = 1.6e-3;
    let gp = 1.0 / 62.5e3;

    let mut lin_set = HashSet::new();
    lin_set.insert("T1".to_string());
    let mut mna_lin = MnaSystem::from_netlist_with_all_reductions(
        &netlist,
        &HashSet::new(),
        &HashSet::new(),
        &lin_set,
        &HashMap::new(),
    )
    .expect("build linearized MNA");
    assert_eq!(mna_lin.m, 0, "linearized triode should leave M = 0");

    mna_lin.linearized_triodes = vec![LinearizedTriodeInfo {
        name: "T1".to_string(),
        ng,
        np,
        nk,
        gm,
        gp,
        ip_dc,
        ig_dc,
        vgk0: v_at(ng) - v_at(nk),
        vpk0: v_at(np) - v_at(nk),
    }];
    mna_lin.stamp_linearized_triodes();

    let dc_lin = solve_dc_operating_point(&mna_lin, &[], &DcOpConfig::default());
    assert!(dc_lin.converged, "linearized triode DC solve must converge");

    for name in ["g", "p", "k"] {
        let idx = node(&mna_nl, name) - 1;
        let v_ref = dc_nl.v_node[idx];
        let v_lin = dc_lin.v_node[idx];
        assert!(
            (v_ref - v_lin).abs() < 1e-3,
            "linearized triode DC at node '{}' must match nonlinear OP: {} vs {}",
            name,
            v_lin,
            v_ref
        );
    }
}

// ────────────────────────────────────────────────────────────────────
// Fix 5 (LOW): BoyleDiodes op-amps keep IB/RIN input parasitics
// ────────────────────────────────────────────────────────────────────

#[test]
fn boyle_diodes_opamp_still_stamps_ib_and_rin() {
    let spice = "\
Boyle op-amp with input parasitics
V1 vcc 0 DC 15
Rvcc vcc 0 100k
Rin in 0 10k
Rf out inv 100k
Rg inv 0 10k
U1 in inv out OA1
Rload out 0 10k
.model OA1 OA(AOL=100000 ROUT=100 VCC=15 VEE=-15 IB=1e-8 RIN=1e9)
";
    let netlist = Netlist::parse(spice).expect("parse opamp netlist");
    let mna_pre = MnaSystem::from_netlist(&netlist).expect("build pre-augment MNA");

    // Synthesize the BoyleDiodes internal node + catch diodes, then rebuild.
    let augmented =
        melange_solver::codegen::ir::augment_netlist_with_boyle_diodes(&netlist, &mna_pre);
    let mna = MnaSystem::from_netlist(&augmented).expect("build Boyle MNA");

    // The op-amp must actually be in BoyleDiodes mode (internal node found).
    let oa = &mna.opamps[0];
    assert!(
        oa.n_int_idx > 0,
        "op-amp should have taken the BoyleDiodes internal-node path"
    );

    // IB current sources at BOTH inputs (the old code `continue`d out of the
    // dispatch before stamping them for BoyleDiodes op-amps).
    let ib_plus = mna
        .current_sources
        .iter()
        .find(|cs| cs.name.contains("IB_plus"));
    let ib_minus = mna
        .current_sources
        .iter()
        .find(|cs| cs.name.contains("IB_minus"));
    assert!(ib_plus.is_some(), "IB_plus source missing on Boyle path");
    assert!(ib_minus.is_some(), "IB_minus source missing on Boyle path");
    assert!((ib_plus.unwrap().dc_value - 1e-8).abs() < 1e-20);

    // RIN shunt conductance at both input pins.
    let np = oa.n_plus_idx;
    let nm = oa.n_minus_idx;
    let g_in = 1.0 / 1e9;
    // in-pin: Rin 10k to ground + RIN shunt
    let expect_p = 1.0 / 10e3 + g_in;
    assert!(
        (mna.g[np - 1][np - 1] - expect_p).abs() < 1e-15,
        "non-inverting input should carry the 1/RIN shunt, got {}",
        mna.g[np - 1][np - 1]
    );
    // inv-pin: Rf 100k + Rg 10k + RIN shunt
    let expect_m = 1.0 / 100e3 + 1.0 / 10e3 + g_in;
    assert!(
        (mna.g[nm - 1][nm - 1] - expect_m).abs() < 1e-15,
        "inverting input should carry the 1/RIN shunt, got {}",
        mna.g[nm - 1][nm - 1]
    );
}
