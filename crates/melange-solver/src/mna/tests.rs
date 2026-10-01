//! MNA assembly tests.

use super::*;
use crate::parser::{BSourceKind, Element, Netlist};

// ── IC= capacitor plumbing ────────────────────────────────────────

#[test]
fn capacitor_ic_is_parsed_and_populates_mna() {
    let spice = "IC test\nV1 in 0 DC 0\nR1 in out 1k\nC1 out 0 10u IC=5\nC2 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    // Only C1 (with IC=) should appear; C2 (no IC=) must not.
    assert_eq!(mna.capacitor_ics.len(), 1);
    let ic = &mna.capacitor_ics[0];
    assert_eq!(ic.name, "C1");
    assert_eq!(ic.ic, 5.0);
    let out_idx = mna.node_map["out"];
    assert_eq!(ic.node_i, out_idx);
    assert_eq!(ic.node_j, 0); // ground
}

#[test]
fn append_ic_voltage_sources_is_noop_without_ic_caps() {
    let spice = "No IC\nV1 in 0 DC 0\nR1 in out 1k\nC1 out 0 10u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let n_aug_before = mna.n_aug;
    let g_before = mna.g.clone();
    let vs_before = mna.voltage_sources.len();
    mna.append_ic_voltage_sources();
    assert_eq!(mna.n_aug, n_aug_before);
    assert_eq!(mna.g, g_before);
    assert_eq!(mna.voltage_sources.len(), vs_before);
}

#[test]
fn append_ic_voltage_sources_stamps_augmented_kvl_kcl() {
    let spice = "IC test\nV1 in 0 DC 0\nR1 in out 1k\nC1 out 0 10u IC=5\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let n = mna.n;
    let n_aug_before = mna.n_aug;
    let num_vs_before = mna.voltage_sources.len();

    mna.append_ic_voltage_sources();

    assert_eq!(mna.n_aug, n_aug_before + 1);
    assert_eq!(mna.voltage_sources.len(), num_vs_before + 1);
    let new_vs = mna.voltage_sources.last().unwrap();
    assert_eq!(new_vs.dc_value, 5.0);
    assert_eq!(new_vs.ext_idx, n_aug_before - n);

    // KVL row / KCL column at the new augmented index k enforce
    // V(out) - V(gnd) = 5 exactly as the voltage-source stamping
    // convention in docs/aidocs/MNA.md.
    let k = n_aug_before;
    let out_idx = mna.node_map["out"] - 1;
    assert_eq!(mna.g[k][out_idx], 1.0);
    assert_eq!(mna.g[out_idx][k], 1.0);
    // Matrix must have grown to n_aug x n_aug (square).
    assert_eq!(mna.g.len(), mna.n_aug);
    for row in &mna.g {
        assert_eq!(row.len(), mna.n_aug);
    }
    // N_v/N_i must be padded to the new dimension too.
    for row in &mna.n_v {
        assert_eq!(row.len(), mna.n_aug);
    }
    assert_eq!(mna.n_i.len(), mna.n_aug);
}

#[test]
fn bsource_parses_v_and_i_forms() {
    let spice = r#"Behavioral source test
Vqi qi 0 DC 0
R1 qi 0 1meg
B_clip out 0 V={ tanh(V(qi)) }
B_inj  n2 0 I={ V(qi) * V(qi) }
R2 n2 0 1k
R3 out 0 1k
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let bsources: Vec<&Element> = netlist
        .elements
        .iter()
        .filter(|e| matches!(e, Element::BSource { .. }))
        .collect();
    assert_eq!(bsources.len(), 2);

    if let Element::BSource { name, kind, .. } = bsources[0] {
        assert_eq!(name, "B_clip");
        assert_eq!(*kind, BSourceKind::Voltage);
    }
    if let Element::BSource { name, kind, .. } = bsources[1] {
        assert_eq!(name, "B_inj");
        assert_eq!(*kind, BSourceKind::Current);
    }

    // MNA must carry both behavioral sources with resolved node indices.
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert_eq!(mna.behavioral_sources.len(), 2);
    let clip = mna
        .behavioral_sources
        .iter()
        .find(|b| b.name == "B_clip")
        .unwrap();
    assert_eq!(clip.kind, BSourceKind::Voltage);
    assert_eq!(clip.v_ext_idx, Some(0));
    // expression references node `qi` — must resolve to a real node index.
    assert!(clip.referenced_node_indices.contains_key("qi"));
    assert_eq!(clip.referenced_node_indices["qi"], mna.node_map["qi"]);

    let inj = mna
        .behavioral_sources
        .iter()
        .find(|b| b.name == "B_inj")
        .unwrap();
    assert_eq!(inj.kind, BSourceKind::Current);
    assert_eq!(inj.v_ext_idx, None);
}

#[test]
fn bsource_ddt_idt_slots_are_globally_unique() {
    // Two sources each with a ddt → slots 0 and 1.
    let spice = r#"ddt slot test
Vqi qi 0 DC 0
R1 qi 0 1meg
B_a a 0 V={ ddt(V(qi)) }
B_b b 0 V={ ddt(V(qi)) + idt(V(qi)) }
Ra a 0 1k
Rb b 0 1k
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let total_slots: usize = mna
        .behavioral_sources
        .iter()
        .map(|b| b.expr.state_slot_count())
        .sum();
    assert_eq!(total_slots, 3); // B_a: 1 ddt; B_b: 1 ddt + 1 idt
}

#[test]
fn radio_iq_discriminator_netlist_parses_and_builds_mna() {
    // The Subspace FM front-end (limiter + discriminator) from the request.
    let spice = r#"FM discriminator
Viq_i iq_i 0 DC 0
Viq_q iq_q 0 DC 0
Ri iq_i 0 1meg
Rq iq_q 0 1meg
B_lim_i lim_i 0 V={ V(iq_i) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_lim_q lim_q 0 V={ V(iq_q) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_demod audio 0 V={ ddt( atan2(V(lim_q), V(lim_i)) ) }
Rli lim_i 0 1meg
Rlq lim_q 0 1meg
Ra audio 0 1k
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert_eq!(mna.behavioral_sources.len(), 3);
    // The discriminator's ddt got a state slot.
    let demod = mna
        .behavioral_sources
        .iter()
        .find(|b| b.name == "B_demod")
        .unwrap();
    assert_eq!(demod.expr.state_slot_count(), 1);
    assert!(demod.expr.is_time_dependent());
    // It reads the two limiter outputs.
    assert!(demod.referenced_node_indices.contains_key("lim_i"));
    assert!(demod.referenced_node_indices.contains_key("lim_q"));
}

#[test]
fn test_mna_rc_circuit() {
    let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.n, 2); // in, out
    assert_eq!(mna.m, 0); // No nonlinear devices

    // Check resistor stamp (1k = 0.001 S)
    assert!((mna.g[0][0] - 0.001).abs() < 1e-10);
    assert!((mna.g[1][1] - 0.001).abs() < 1e-10);
    assert!((mna.g[0][1] + 0.001).abs() < 1e-10);
    assert!((mna.g[1][0] + 0.001).abs() < 1e-10);

    // Check capacitor stamp
    assert!((mna.c[1][1] - 1e-6).abs() < 1e-15);
}

#[test]
fn test_mna_diode_clipper() {
    let spice = r#"Diode Clipper
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.n, 2); // in, out
    assert_eq!(mna.m, 1); // 1 diode = 1 dimension
    assert_eq!(mna.num_devices, 1);

    // Check N_v: v_d = v_in - v_out
    assert_eq!(mna.n_v[0][0], 1.0);
    assert_eq!(mna.n_v[0][1], -1.0);

    // Check N_i: current injected at in and out
    assert_eq!(mna.n_i[0][0], -1.0);
    assert_eq!(mna.n_i[1][0], 1.0);
}

#[test]
fn test_mna_bjt_dimensions() {
    let spice = r#"Common Emitter
Q1 coll base emit 2N2222
R1 coll vcc 1k
R2 base 0 100k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Nodes: coll, base, emit, vcc (but vcc has no DC path, may be dropped)
    // Actually vcc connects through R1, so all 4 nodes exist
    assert_eq!(mna.n, 4); // coll, base, emit, vcc (minus ground)
    assert_eq!(mna.m, 2); // 1 BJT = 2 dimensions (Vbe, Vbc)
    assert_eq!(mna.num_devices, 1);

    // Find the BJT device
    let bjt = &mna.nonlinear_devices[0];
    assert_eq!(bjt.device_type, NonlinearDeviceType::Bjt);
    assert_eq!(bjt.dimension, 2);
    assert_eq!(bjt.start_idx, 0);

    // Check N_v for BJT
    // Row 0: Vbe = Vb - Ve
    // Row 1: Vbc = Vb - Vc
    let base_idx = bjt.node_indices[1] - 1; // 0-indexed
    let emit_idx = bjt.node_indices[2] - 1;
    let coll_idx = bjt.node_indices[0] - 1;

    assert_eq!(mna.n_v[0][base_idx], 1.0);
    assert_eq!(mna.n_v[0][emit_idx], -1.0);
    assert_eq!(mna.n_v[1][base_idx], 1.0);
    assert_eq!(mna.n_v[1][coll_idx], -1.0);
}

// ===== Op-amp MNA tests =====

#[test]
fn test_mna_opamp_basic() {
    let spice = r#"Opamp Test
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.n, 3); // in, inv, out
    assert_eq!(mna.m, 0); // Op-amp is linear
    assert_eq!(mna.num_devices, 0);
    assert_eq!(mna.opamps.len(), 1);
    assert_eq!(mna.opamps[0].aol, 200_000.0);
}

#[test]
fn test_mna_opamp_vccs_stamping() {
    let spice = r#"Opamp VCCS Test
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
.model opamp OA(AOL=200000 ROUT=1)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let inv_idx = *mna.node_map.get("inv").unwrap();
    let out_idx = *mna.node_map.get("out").unwrap();

    let o = out_idx - 1;
    let i = inv_idx - 1;

    // inv is the INVERTING input (V−). Correct polarity: the op-amp
    // injects +Gm·(V+ − V−) into out, so current leaving out gets
    // +Gm·V_inv → G[out,inv] = +Gm. R2 (inv↔out) adds −g_R2.
    let g_r2 = 1.0 / 100_000.0;
    assert!(
        (mna.g[o][i] - (200_000.0 - g_r2)).abs() < 1e-6,
        "G[out,inv] should be +Gm - g_R2, got {}",
        mna.g[o][i]
    );

    // G[out, out] should have Go + g_R2
    let expected_go = 1.0 + g_r2;
    assert!(
        (mna.g[o][o] - expected_go).abs() < 1e-6,
        "G[out,out] should include Go={}, got {}",
        expected_go,
        mna.g[o][o]
    );
}

#[test]
fn test_mna_opamp_output_grounded_error() {
    let spice = r#"Opamp Output Grounded
R1 in inv 10k
U1 inp inv 0 opamp
.model opamp OA(AOL=200000)
"#;
    let result = Netlist::parse(spice).and_then(|n| {
        MnaSystem::from_netlist(&n).map_err(|e| crate::parser::ParseError {
            line: 0,
            message: format!("{}", e),
        })
    });
    assert!(result.is_err(), "Op-amp with grounded output should error");
}

#[test]
fn test_mna_opamp_no_nonlinear_dimensions() {
    let spice = r#"Opamp With Diode
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
D1 out 0 D1N4148
.model opamp OA(AOL=200000)
.model D1N4148 D(IS=1e-15)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.m, 1); // Only diode
    assert_eq!(mna.num_devices, 1);
    assert_eq!(mna.opamps.len(), 1);
}

// ===== VCCS (G element) MNA tests =====

#[test]
fn test_mna_vccs_basic() {
    // G1 out 0 in 0 0.01 — SPICE G-element convention: the source DRAWS
    // gm*(V_in - 0) OUT of node `out` (and injects it into ground), hence
    // the +gm stamp at G[out][in] (G·v = current leaving the node).
    // This is the OPPOSITE of the op-amp VCCS orientation — do not
    // "harmonize" the two.
    let spice = r#"VCCS Test
R1 in 0 1k
R2 out 0 1k
G1 out 0 in 0 0.01
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.m, 0); // VCCS is linear
    assert_eq!(mna.num_devices, 0);

    let in_idx = *mna.node_map.get("in").unwrap();
    let out_idx = *mna.node_map.get("out").unwrap();

    let o = out_idx - 1;
    let i = in_idx - 1;

    // G[out, in] should have +gm = +0.01
    assert!(
        (mna.g[o][i] - 0.01).abs() < 1e-15,
        "G[out,in] should be +gm=0.01, got {}",
        mna.g[o][i]
    );

    // G[out, out] should only have resistor conductance (1/1k = 0.001)
    assert!(
        (mna.g[o][o] - 0.001).abs() < 1e-15,
        "G[out,out] should be 0.001, got {}",
        mna.g[o][o]
    );
}

#[test]
fn test_mna_vccs_differential() {
    // G1 out 0 inp inn 0.01 — ctrl is differential
    let spice = r#"VCCS Differential
R1 out 0 1k
G1 out 0 inp inn 0.01
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let out_idx = *mna.node_map.get("out").unwrap();
    let inp_idx = *mna.node_map.get("inp").unwrap();
    let inn_idx = *mna.node_map.get("inn").unwrap();

    let o = out_idx - 1;

    // G[out, inp] should have +gm
    assert!(
        (mna.g[o][inp_idx - 1] - 0.01).abs() < 1e-15,
        "G[out,inp] should be +gm, got {}",
        mna.g[o][inp_idx - 1]
    );

    // G[out, inn] should have -gm
    assert!(
        (mna.g[o][inn_idx - 1] - (-0.01)).abs() < 1e-15,
        "G[out,inn] should be -gm, got {}",
        mna.g[o][inn_idx - 1]
    );
}

#[test]
fn test_mna_vccs_out_n_not_ground() {
    // G1 out_p out_n ctrl 0 0.01 — output negative node is not ground
    let spice = r#"VCCS Non-Ground Output
R1 out_p 0 1k
R2 out_n 0 1k
G1 out_p out_n ctrl 0 0.01
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let op = *mna.node_map.get("out_p").unwrap() - 1;
    let on = *mna.node_map.get("out_n").unwrap() - 1;
    let ctrl = *mna.node_map.get("ctrl").unwrap() - 1;

    // G[out_p, ctrl] += gm
    assert!((mna.g[op][ctrl] - 0.01).abs() < 1e-15);
    // G[out_n, ctrl] -= gm
    assert!((mna.g[on][ctrl] - (-0.01)).abs() < 1e-15);
}

// ===== VCVS (E element) MNA tests =====

#[test]
fn test_mna_vcvs_basic() {
    // E1 out 0 in 0 10 — voltage gain of 10
    // With augmented MNA, VCVS adds an extra row/col at index n + num_vs + 0.
    let spice = r#"VCVS Test
R1 in 0 1k
R2 out 0 1k
E1 out 0 in 0 10
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.m, 0); // VCVS is linear
    assert_eq!(mna.num_devices, 0);

    let n = mna.n;
    // n_aug = n + 0 VS + 1 VCVS = n + 1
    assert_eq!(mna.n_aug, n + 1);
    assert_eq!(mna.g.len(), n + 1, "G should be n_aug × n_aug");

    let in_idx = *mna.node_map.get("in").unwrap();
    let out_idx = *mna.node_map.get("out").unwrap();
    let o = out_idx - 1;
    let i = in_idx - 1;
    let k = n; // augmented row for VCVS (no voltage sources, so k = n + 0 + 0)

    // G[out][k] should be +1 (current injection column: j_vcvs enters out+)
    assert!(
        (mna.g[o][k] - 1.0).abs() < 1e-15,
        "G[out][k] should be +1 for VCVS current injection, got {}",
        mna.g[o][k]
    );

    // G[k][out] should be +1 (KVL row: V_out+)
    assert!(
        (mna.g[k][o] - 1.0).abs() < 1e-15,
        "G[k][out] should be +1 for KVL constraint, got {}",
        mna.g[k][o]
    );

    // G[k][in] should be -gain = -10 (KVL row: -gain * V_ctrl+)
    assert!(
        (mna.g[k][i] - (-10.0)).abs() < 1e-15,
        "G[k][in] should be -gain=-10 for KVL constraint, got {}",
        mna.g[k][i]
    );

    // G[out][out] should only have 1/R2 (no VS_CONDUCTANCE in augmented MNA)
    let g_r2 = 1.0 / 1000.0;
    assert!(
        (mna.g[o][o] - g_r2).abs() < 1e-10,
        "G[out][out] should only have 1/R2={}, got {} (augmented MNA: no Norton equiv)",
        g_r2,
        mna.g[o][o]
    );
}

#[test]
fn test_mna_vcvs_differential_output() {
    // E1 out_p out_n in 0 5 — VCVS with non-ground output negative
    // With augmented MNA, VCVS adds an extra row/col (no VS in this circuit).
    let spice = r#"VCVS Diff Output
R1 out_p 0 1k
R2 out_n 0 1k
E1 out_p out_n in 0 5
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let n = mna.n;
    assert_eq!(mna.n_aug, n + 1, "One VCVS adds 1 augmented dimension");

    let op = *mna.node_map.get("out_p").unwrap() - 1;
    let on = *mna.node_map.get("out_n").unwrap() - 1;
    let inp = *mna.node_map.get("in").unwrap() - 1;
    let k = n; // augmented row for the VCVS

    // Current injection column: j_vcvs enters out+ (G[out_p][k] = +1), exits out- (G[out_n][k] = -1)
    let g_r = 1.0 / 1000.0;
    // G[out_p][out_p] should only have 1/R1 (no Norton equivalent conductance)
    assert!(
        (mna.g[op][op] - g_r).abs() < 1e-10,
        "G[op][op] should only be 1/R1 in augmented MNA, got {}",
        mna.g[op][op]
    );
    assert!(
        (mna.g[on][on] - g_r).abs() < 1e-10,
        "G[on][on] should only be 1/R2 in augmented MNA, got {}",
        mna.g[on][on]
    );

    // Current injection: j_vcvs enters out+, exits out-
    assert!(
        (mna.g[op][k] - 1.0).abs() < 1e-15,
        "G[out_p][k] should be +1 (current injection), got {}",
        mna.g[op][k]
    );
    assert!(
        (mna.g[on][k] - (-1.0)).abs() < 1e-15,
        "G[out_n][k] should be -1 (current injection), got {}",
        mna.g[on][k]
    );

    // KVL constraint row: G[k][out_p] = +1, G[k][out_n] = -1, G[k][in] = -gain = -5
    assert!(
        (mna.g[k][op] - 1.0).abs() < 1e-15,
        "G[k][out_p] should be +1 (KVL), got {}",
        mna.g[k][op]
    );
    assert!(
        (mna.g[k][on] - (-1.0)).abs() < 1e-15,
        "G[k][out_n] should be -1 (KVL), got {}",
        mna.g[k][on]
    );
    assert!(
        (mna.g[k][inp] - (-5.0)).abs() < 1e-15,
        "G[k][in] should be -gain=-5 (KVL), got {}",
        mna.g[k][inp]
    );
}

#[test]
fn test_mna_vccs_with_opamp() {
    // VCCS and op-amp coexist in same circuit
    let spice = r#"VCCS With Opamp
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
G1 out2 0 out 0 0.001
R3 out2 0 1k
.model opamp OA(AOL=200000)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.m, 0); // Both linear
    assert_eq!(mna.opamps.len(), 1);

    let out_idx = *mna.node_map.get("out").unwrap();
    let out2_idx = *mna.node_map.get("out2").unwrap();

    let o2 = out2_idx - 1;
    let o = out_idx - 1;

    // G[out2, out] should have VCCS gm = 0.001
    assert!(
        (mna.g[o2][o] - 0.001).abs() < 1e-15,
        "G[out2,out] should be VCCS gm=0.001, got {}",
        mna.g[o2][o]
    );
}

#[test]
fn test_mna_vcvs_no_nonlinear_dimensions() {
    // VCVS + diode: only the diode adds nonlinear dimensions
    let spice = r#"VCVS With Diode
R1 in 0 1k
E1 mid 0 in 0 10
D1 mid out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.m, 1); // Only diode
    assert_eq!(mna.num_devices, 1);
}

// ===== Parasitic capacitance tests =====

#[test]
fn test_parasitic_cap_value() {
    assert!(
        (PARASITIC_CAP - 10e-12).abs() < 1e-25,
        "PARASITIC_CAP should be 10pF"
    );
}

#[test]
fn test_parasitic_cap_diode_across_junction() {
    // Diode: one cap between anode and cathode (across junction)
    let spice = r#"Diode Parasitic
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    // C matrix should be all zeros before parasitic caps
    for i in 0..mna.n {
        for j in 0..mna.n {
            assert_eq!(
                mna.c[i][j], 0.0,
                "C[{i}][{j}] should be 0 before parasitic caps"
            );
        }
    }

    mna.add_parasitic_caps();

    let anode = *mna.node_map.get("in").unwrap();
    let cathode = *mna.node_map.get("out").unwrap();
    let a = anode - 1; // 0-indexed
    let k = cathode - 1;

    // Diagonal: both nodes get +PARASITIC_CAP
    assert!(
        (mna.c[a][a] - PARASITIC_CAP).abs() < 1e-25,
        "C[anode][anode] should be PARASITIC_CAP, got {}",
        mna.c[a][a]
    );
    assert!(
        (mna.c[k][k] - PARASITIC_CAP).abs() < 1e-25,
        "C[cathode][cathode] should be PARASITIC_CAP, got {}",
        mna.c[k][k]
    );

    // Off-diagonal: negative (cap between nodes, not to ground)
    assert!(
        (mna.c[a][k] + PARASITIC_CAP).abs() < 1e-25,
        "C[anode][cathode] should be -PARASITIC_CAP, got {}",
        mna.c[a][k]
    );
    assert!(
        (mna.c[k][a] + PARASITIC_CAP).abs() < 1e-25,
        "C[cathode][anode] should be -PARASITIC_CAP, got {}",
        mna.c[k][a]
    );
}

#[test]
fn test_parasitic_cap_bjt_two_junctions() {
    // BJT: two caps — B-E and B-C
    let spice = r#"BJT Parasitic
Q1 coll base emit 2N2222
R1 coll 0 1k
R2 base 0 100k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.add_parasitic_caps();

    let nc = *mna.node_map.get("coll").unwrap() - 1;
    let nb = *mna.node_map.get("base").unwrap() - 1;
    let ne = *mna.node_map.get("emit").unwrap() - 1;

    // Base gets caps from both B-E and B-C junctions: 2 * PARASITIC_CAP
    assert!(
        (mna.c[nb][nb] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
        "C[base][base] should be 2*PARASITIC_CAP, got {}",
        mna.c[nb][nb]
    );

    // Collector gets cap from B-C junction only
    assert!(
        (mna.c[nc][nc] - PARASITIC_CAP).abs() < 1e-25,
        "C[coll][coll] should be PARASITIC_CAP, got {}",
        mna.c[nc][nc]
    );

    // Emitter gets cap from B-E junction only
    assert!(
        (mna.c[ne][ne] - PARASITIC_CAP).abs() < 1e-25,
        "C[emit][emit] should be PARASITIC_CAP, got {}",
        mna.c[ne][ne]
    );

    // Off-diagonal: B-E junction
    assert!(
        (mna.c[nb][ne] + PARASITIC_CAP).abs() < 1e-25,
        "C[base][emit] should be -PARASITIC_CAP, got {}",
        mna.c[nb][ne]
    );
    assert!(
        (mna.c[ne][nb] + PARASITIC_CAP).abs() < 1e-25,
        "C[emit][base] should be -PARASITIC_CAP, got {}",
        mna.c[ne][nb]
    );

    // Off-diagonal: B-C junction
    assert!(
        (mna.c[nb][nc] + PARASITIC_CAP).abs() < 1e-25,
        "C[base][coll] should be -PARASITIC_CAP, got {}",
        mna.c[nb][nc]
    );
    assert!(
        (mna.c[nc][nb] + PARASITIC_CAP).abs() < 1e-25,
        "C[coll][base] should be -PARASITIC_CAP, got {}",
        mna.c[nc][nb]
    );

    // Collector-Emitter: no direct parasitic cap
    assert!(
        (mna.c[nc][ne]).abs() < 1e-25,
        "C[coll][emit] should be 0, got {}",
        mna.c[nc][ne]
    );
}

#[test]
fn test_parasitic_cap_jfet_two_junctions() {
    // JFET: two caps — G-S and G-D
    let spice = r#"JFET Parasitic
J1 drain gate source JN
R1 drain 0 1k
R2 gate 0 1M
.model JN NJ(VTO=-2.0)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.add_parasitic_caps();

    let nd = *mna.node_map.get("drain").unwrap() - 1;
    let ng = *mna.node_map.get("gate").unwrap() - 1;
    let ns = *mna.node_map.get("source").unwrap() - 1;

    // Gate gets caps from both G-S and G-D junctions: 2 * PARASITIC_CAP
    assert!(
        (mna.c[ng][ng] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
        "C[gate][gate] should be 2*PARASITIC_CAP, got {}",
        mna.c[ng][ng]
    );

    // Off-diagonal: G-S
    assert!(
        (mna.c[ng][ns] + PARASITIC_CAP).abs() < 1e-25,
        "C[gate][source] should be -PARASITIC_CAP, got {}",
        mna.c[ng][ns]
    );

    // Off-diagonal: G-D
    assert!(
        (mna.c[ng][nd] + PARASITIC_CAP).abs() < 1e-25,
        "C[gate][drain] should be -PARASITIC_CAP, got {}",
        mna.c[ng][nd]
    );

    // Drain-Source: no direct parasitic cap
    assert!(
        (mna.c[nd][ns]).abs() < 1e-25,
        "C[drain][source] should be 0, got {}",
        mna.c[nd][ns]
    );
}

#[test]
fn test_parasitic_cap_mosfet_two_junctions() {
    // MOSFET: two caps — G-S and G-D
    let spice = r#"MOSFET Parasitic
M1 drain gate source source NM1
R1 drain 0 1k
.model NM1 NM(VTO=2.0 KP=0.1)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.add_parasitic_caps();

    let nd = *mna.node_map.get("drain").unwrap() - 1;
    let ng = *mna.node_map.get("gate").unwrap() - 1;
    let ns = *mna.node_map.get("source").unwrap() - 1;

    // Gate gets caps from G-S and G-D: 2 * PARASITIC_CAP
    assert!(
        (mna.c[ng][ng] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
        "C[gate][gate] should be 2*PARASITIC_CAP, got {}",
        mna.c[ng][ng]
    );

    // Off-diagonal: G-S
    assert!(
        (mna.c[ng][ns] + PARASITIC_CAP).abs() < 1e-25,
        "C[gate][source] should be -PARASITIC_CAP, got {}",
        mna.c[ng][ns]
    );

    // Off-diagonal: G-D
    assert!(
        (mna.c[ng][nd] + PARASITIC_CAP).abs() < 1e-25,
        "C[gate][drain] should be -PARASITIC_CAP, got {}",
        mna.c[ng][nd]
    );
}

#[test]
fn test_parasitic_cap_tube_two_junctions() {
    // Tube: two caps — grid-cathode (Cgk) and plate-cathode (Cpk)
    let spice = r#"Tube Parasitic
T1 grid plate cathode 12AX7
R1 plate 0 100k
R2 grid 0 1M
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.add_parasitic_caps();

    let ng = *mna.node_map.get("grid").unwrap() - 1;
    let np = *mna.node_map.get("plate").unwrap() - 1;
    let nk = *mna.node_map.get("cathode").unwrap() - 1;

    // Cathode gets caps from both Cgk and Cpk: 2 * PARASITIC_CAP
    assert!(
        (mna.c[nk][nk] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
        "C[cathode][cathode] should be 2*PARASITIC_CAP, got {}",
        mna.c[nk][nk]
    );

    // Grid gets cap from Cgk only
    assert!(
        (mna.c[ng][ng] - PARASITIC_CAP).abs() < 1e-25,
        "C[grid][grid] should be PARASITIC_CAP, got {}",
        mna.c[ng][ng]
    );

    // Plate gets cap from Cpk only
    assert!(
        (mna.c[np][np] - PARASITIC_CAP).abs() < 1e-25,
        "C[plate][plate] should be PARASITIC_CAP, got {}",
        mna.c[np][np]
    );

    // Off-diagonal: grid-cathode
    assert!(
        (mna.c[ng][nk] + PARASITIC_CAP).abs() < 1e-25,
        "C[grid][cathode] should be -PARASITIC_CAP, got {}",
        mna.c[ng][nk]
    );
    assert!(
        (mna.c[nk][ng] + PARASITIC_CAP).abs() < 1e-25,
        "C[cathode][grid] should be -PARASITIC_CAP, got {}",
        mna.c[nk][ng]
    );

    // Off-diagonal: plate-cathode
    assert!(
        (mna.c[np][nk] + PARASITIC_CAP).abs() < 1e-25,
        "C[plate][cathode] should be -PARASITIC_CAP, got {}",
        mna.c[np][nk]
    );
    assert!(
        (mna.c[nk][np] + PARASITIC_CAP).abs() < 1e-25,
        "C[cathode][plate] should be -PARASITIC_CAP, got {}",
        mna.c[nk][np]
    );

    // Grid-Plate: no direct parasitic cap
    assert!(
        (mna.c[ng][np]).abs() < 1e-25,
        "C[grid][plate] should be 0, got {}",
        mna.c[ng][np]
    );
}

#[test]
fn test_parasitic_cap_count_per_device() {
    // Verify correct number of junction caps: 1 for diode, 2 for BJT
    // All device terminals are non-ground so every junction produces off-diagonal entries
    let spice = r#"Mixed Devices
D1 in mid D1N4148
Q1 out mid emit 2N2222
R1 out 0 1k
R2 emit 0 100
.model D1N4148 D(IS=1e-15)
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.add_parasitic_caps();

    // Count total nonzero off-diagonal C entries (each junction adds 2 off-diag entries)
    let mut off_diag_count = 0;
    for i in 0..mna.n {
        for j in 0..mna.n {
            if i != j && mna.c[i][j].abs() > 1e-25 {
                off_diag_count += 1;
            }
        }
    }
    // Diode: 1 junction (anode-cathode) = 2 off-diagonal entries
    // BJT: 2 junctions (B-E + B-C, all non-ground) = 4 off-diagonal entries
    // Total: 6 off-diagonal entries
    assert_eq!(
            off_diag_count, 6,
            "Expected 6 off-diagonal C entries (1 diode junction + 2 BJT junctions), got {off_diag_count}"
        );
}

// ===== VCA MNA tests =====

#[test]
fn test_mna_vca_basic() {
    let spice = r#"VCA Test
R1 sig_in 0 1k
R2 sig_out 0 1k
R3 ctrl 0 100k
Y1 sig_in sig_out ctrl 0 vca1
.model vca1 VCA(VSCALE=0.05298 G0=1.0)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.m, 2, "VCA should add M=2 (signal + control dimensions)");
    assert_eq!(mna.num_devices, 1, "Should have 1 nonlinear device");
    assert_eq!(mna.vcas.len(), 1, "Should have 1 VCA");
    assert_eq!(
        mna.nonlinear_devices[0].device_type,
        NonlinearDeviceType::Vca
    );
    assert_eq!(mna.nonlinear_devices[0].dimension, 2);
    assert_eq!(mna.nonlinear_devices[0].start_idx, 0);

    // Check VCA model parameters resolved
    assert!((mna.vcas[0].vscale - 0.05298).abs() < 1e-10);
    assert!((mna.vcas[0].g0 - 1.0).abs() < 1e-10);
}

#[test]
fn test_mna_vca_nv_stamping() {
    // VCA with all terminals non-ground
    let spice = r#"VCA N_v Test
R1 sp 0 1k
R2 sn 0 1k
R3 cp 0 100k
R4 cn 0 100k
Y1 sp sn cp cn vca1
.model vca1 VCA(VSCALE=0.05298 G0=1.0)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let sp_idx = *mna.node_map.get("sp").unwrap();
    let sn_idx = *mna.node_map.get("sn").unwrap();
    let cp_idx = *mna.node_map.get("cp").unwrap();
    let cn_idx = *mna.node_map.get("cn").unwrap();

    // N_v row 0 (V_signal): +1 at sig+, -1 at sig-
    assert_eq!(mna.n_v[0][sp_idx - 1], 1.0, "N_v[0][sig+] should be +1");
    assert_eq!(mna.n_v[0][sn_idx - 1], -1.0, "N_v[0][sig-] should be -1");

    // N_v row 1 (V_control): +1 at ctrl+, -1 at ctrl-
    assert_eq!(mna.n_v[1][cp_idx - 1], 1.0, "N_v[1][ctrl+] should be +1");
    assert_eq!(mna.n_v[1][cn_idx - 1], -1.0, "N_v[1][ctrl-] should be -1");

    // Verify no cross-contamination: signal row has no ctrl nodes
    assert_eq!(mna.n_v[0][cp_idx - 1], 0.0, "N_v[0][ctrl+] should be 0");
    assert_eq!(mna.n_v[0][cn_idx - 1], 0.0, "N_v[0][ctrl-] should be 0");
    // Control row has no signal nodes
    assert_eq!(mna.n_v[1][sp_idx - 1], 0.0, "N_v[1][sig+] should be 0");
    assert_eq!(mna.n_v[1][sn_idx - 1], 0.0, "N_v[1][sig-] should be 0");
}

#[test]
fn test_mna_vca_ni_stamping() {
    // VCA: signal current at sig+/sig-, no control current
    let spice = r#"VCA N_i Test
R1 sp 0 1k
R2 sn 0 1k
R3 cp 0 100k
R4 cn 0 100k
Y1 sp sn cp cn vca1
.model vca1 VCA(VSCALE=0.05298 G0=1.0)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let sp_idx = *mna.node_map.get("sp").unwrap();
    let sn_idx = *mna.node_map.get("sn").unwrap();
    let cp_idx = *mna.node_map.get("cp").unwrap();
    let cn_idx = *mna.node_map.get("cn").unwrap();

    // N_i col 0 (I_signal): -1 at sig+ (extracted), +1 at sig- (injected)
    assert_eq!(
        mna.n_i[sp_idx - 1][0],
        -1.0,
        "N_i[sig+][0] should be -1 (current extracted)"
    );
    assert_eq!(
        mna.n_i[sn_idx - 1][0],
        1.0,
        "N_i[sig-][0] should be +1 (current injected)"
    );

    // N_i col 1 (I_control): NO stamping — control draws no current
    assert_eq!(
        mna.n_i[cp_idx - 1][1],
        0.0,
        "N_i[ctrl+][1] should be 0 (no control current)"
    );
    assert_eq!(
        mna.n_i[cn_idx - 1][1],
        0.0,
        "N_i[ctrl-][1] should be 0 (no control current)"
    );

    // Also verify no signal current in control nodes
    assert_eq!(mna.n_i[cp_idx - 1][0], 0.0, "N_i[ctrl+][0] should be 0");
    assert_eq!(mna.n_i[cn_idx - 1][0], 0.0, "N_i[ctrl-][0] should be 0");

    // No control current in signal nodes either (col 1)
    assert_eq!(mna.n_i[sp_idx - 1][1], 0.0, "N_i[sig+][1] should be 0");
    assert_eq!(mna.n_i[sn_idx - 1][1], 0.0, "N_i[sig-][1] should be 0");
}

// ===== Pentode MNA tests (phase 1a) =====
//
// Pentode 3D layout — locked in by row/col convention shared with the
// codegen template (device_tube.rs.tera) and the DC-OP solver:
//   row/col 0: Ip   ↔ Vgk
//   row/col 1: Ig2  ↔ Vpk
//   row/col 2: Ig1  ↔ Vg2k
// The suppressor (n_suppressor) must be on the cathode node (any other
// wiring is refused); no N_v/N_i entries are stamped for it.

/// EL84 model directive used by the pentode tests below.
/// Beam-tetrode-friendly Reefman params (matches parser.rs unit tests).
const EL84_MODEL: &str = ".model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 \
                              KP=152.4 KVB=4015.8 ALPHA_S=7.66 \
                              A_FACTOR=4.344e-4 BETA_FACTOR=0.148)";

#[test]
fn test_pentode_dimension_is_3() {
    let spice = format!(
        "Pentode dimension test\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.m, 3, "Pentode should add M=3 (Ip + Ig2 + Ig1)");
    assert_eq!(mna.num_devices, 1, "Should have exactly 1 nonlinear device");
    let dev = &mna.nonlinear_devices[0];
    assert_eq!(dev.device_type, NonlinearDeviceType::Tube);
    assert_eq!(dev.dimension, 3);
    assert_eq!(dev.start_idx, 0);
}

#[test]
fn test_pentode_nv_ni_stamping() {
    // Minimal pentode test rig: V1 on plate, resistors on grid/screen/cathode.
    let spice = format!(
        "Pentode N_v / N_i stamping test\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let p_idx = *mna.node_map.get("plate").unwrap() - 1;
    let g_idx = *mna.node_map.get("grid").unwrap() - 1;
    let k_idx = *mna.node_map.get("cath").unwrap() - 1;
    let s_idx = *mna.node_map.get("screen").unwrap() - 1;

    // ----- N_v rows (controlling voltages extracted from node voltages) -----
    // Row 0 = Vgk = V_grid - V_cathode
    assert_eq!(mna.n_v[0][g_idx], 1.0, "N_v[0][grid] should be +1 (Vgk)");
    assert_eq!(mna.n_v[0][k_idx], -1.0, "N_v[0][cath] should be -1 (Vgk)");
    assert_eq!(mna.n_v[0][p_idx], 0.0, "N_v[0][plate] should be 0");
    assert_eq!(mna.n_v[0][s_idx], 0.0, "N_v[0][screen] should be 0");

    // Row 1 = Vpk = V_plate - V_cathode
    assert_eq!(mna.n_v[1][p_idx], 1.0, "N_v[1][plate] should be +1 (Vpk)");
    assert_eq!(mna.n_v[1][k_idx], -1.0, "N_v[1][cath] should be -1 (Vpk)");
    assert_eq!(mna.n_v[1][g_idx], 0.0, "N_v[1][grid] should be 0");
    assert_eq!(mna.n_v[1][s_idx], 0.0, "N_v[1][screen] should be 0");

    // Row 2 = Vg2k = V_screen - V_cathode
    assert_eq!(mna.n_v[2][s_idx], 1.0, "N_v[2][screen] should be +1 (Vg2k)");
    assert_eq!(mna.n_v[2][k_idx], -1.0, "N_v[2][cath] should be -1 (Vg2k)");
    assert_eq!(mna.n_v[2][p_idx], 0.0, "N_v[2][plate] should be 0");
    assert_eq!(mna.n_v[2][g_idx], 0.0, "N_v[2][grid] should be 0");

    // ----- N_i columns (currents injected into node voltages) -----
    // Col 0 = Ip: extracted from plate, injected into cathode
    assert_eq!(
        mna.n_i[p_idx][0], -1.0,
        "N_i[plate][0] should be -1 (Ip out)"
    );
    assert_eq!(mna.n_i[k_idx][0], 1.0, "N_i[cath][0] should be +1 (Ip in)");
    assert_eq!(mna.n_i[g_idx][0], 0.0, "N_i[grid][0] should be 0");
    assert_eq!(mna.n_i[s_idx][0], 0.0, "N_i[screen][0] should be 0");

    // Col 1 = Ig2: extracted from screen, injected into cathode
    assert_eq!(
        mna.n_i[s_idx][1], -1.0,
        "N_i[screen][1] should be -1 (Ig2 out)"
    );
    assert_eq!(mna.n_i[k_idx][1], 1.0, "N_i[cath][1] should be +1 (Ig2 in)");
    assert_eq!(mna.n_i[p_idx][1], 0.0, "N_i[plate][1] should be 0");
    assert_eq!(mna.n_i[g_idx][1], 0.0, "N_i[grid][1] should be 0");

    // Col 2 = Ig1: extracted from grid, injected into cathode
    assert_eq!(
        mna.n_i[g_idx][2], -1.0,
        "N_i[grid][2] should be -1 (Ig1 out)"
    );
    assert_eq!(mna.n_i[k_idx][2], 1.0, "N_i[cath][2] should be +1 (Ig1 in)");
    assert_eq!(mna.n_i[p_idx][2], 0.0, "N_i[plate][2] should be 0");
    assert_eq!(mna.n_i[s_idx][2], 0.0, "N_i[screen][2] should be 0");
}

#[test]
fn test_pentode_cathode_tied_suppressor_matches_4_node() {
    // Build the same circuit two ways: without a suppressor node, and
    // with the suppressor named explicitly and wired to the cathode (the
    // only suppressor wiring melange accepts). The suppressor stamps
    // nothing, so the N_v / N_i blocks must be identical.
    let spice4 = format!(
        "4-node pentode\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    // Same circuit with the suppressor as an explicit 5th terminal,
    // tied to the cathode node.
    let spice5 = format!(
        "5-node pentode (suppressor on the cathode)\n\
             P1 plate grid cath screen cath EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );

    let netlist4 = Netlist::parse(&spice4).unwrap();
    let netlist5 = Netlist::parse(&spice5).unwrap();
    let mna4 = MnaSystem::from_netlist(&netlist4).unwrap();
    let mna5 = MnaSystem::from_netlist(&netlist5).unwrap();

    // Both registrations share the same nonlinear dimension (3D).
    assert_eq!(mna4.m, 3);
    assert_eq!(mna5.m, 3);

    // Look up the four pentode nodes in BOTH MNAs and verify the
    // N_v / N_i entries match exactly.
    let p4 = *mna4.node_map.get("plate").unwrap() - 1;
    let g4 = *mna4.node_map.get("grid").unwrap() - 1;
    let k4 = *mna4.node_map.get("cath").unwrap() - 1;
    let s4 = *mna4.node_map.get("screen").unwrap() - 1;
    let p5 = *mna5.node_map.get("plate").unwrap() - 1;
    let g5 = *mna5.node_map.get("grid").unwrap() - 1;
    let k5 = *mna5.node_map.get("cath").unwrap() - 1;
    let s5 = *mna5.node_map.get("screen").unwrap() - 1;

    for row in 0..3 {
        assert_eq!(
            mna4.n_v[row][p4], mna5.n_v[row][p5],
            "N_v row {} plate",
            row
        );
        assert_eq!(mna4.n_v[row][g4], mna5.n_v[row][g5], "N_v row {} grid", row);
        assert_eq!(mna4.n_v[row][k4], mna5.n_v[row][k5], "N_v row {} cath", row);
        assert_eq!(
            mna4.n_v[row][s4], mna5.n_v[row][s5],
            "N_v row {} screen",
            row
        );
    }
    for col in 0..3 {
        assert_eq!(
            mna4.n_i[p4][col], mna5.n_i[p5][col],
            "N_i col {} plate",
            col
        );
        assert_eq!(mna4.n_i[g4][col], mna5.n_i[g5][col], "N_i col {} grid", col);
        assert_eq!(mna4.n_i[k4][col], mna5.n_i[k5][col], "N_i col {} cath", col);
        assert_eq!(
            mna4.n_i[s4][col], mna5.n_i[s5][col],
            "N_i col {} screen",
            col
        );
    }
    assert_eq!(mna4.n_v, mna5.n_v, "whole N_v identical");
    assert_eq!(mna4.n_i, mna5.n_i, "whole N_i identical");
}

#[test]
fn test_pentode_suppressor_off_cathode_refused() {
    // A suppressor wired anywhere but the cathode would be simulated as
    // cathode-tied (melange stamps nothing for it), silently. It is
    // refused, naming the device, the node and the supported wiring.
    for (supp, extra) in [("sup", "Rsup sup 0 1\n"), ("0", ""), ("screen", "")] {
        let spice = format!(
            "5-node pentode, suppressor off the cathode\n\
                 P1 plate grid cath screen {supp} EL84\n\
                 V1 plate 0 250\n\
                 R1 grid 0 1Meg\n\
                 R2 screen 0 470k\n\
                 R3 cath 0 130\n\
                 {extra}\
                 {EL84_MODEL}\n"
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let err = match MnaSystem::from_netlist(&netlist) {
            Ok(_) => panic!("suppressor on '{supp}' must be refused"),
            Err(e) => e.to_string(),
        };
        assert!(err.contains("pentode 'P1'"), "names the device: {err}");
        assert!(
            err.contains(&format!("node '{supp}'")),
            "names the node: {err}"
        );
        assert!(
            err.contains("cathode node 'cath'"),
            "names the cathode: {err}"
        );
        assert!(
            err.contains("Tie the suppressor to the cathode node, or omit the 5th node"),
            "states the supported wiring: {err}"
        );
    }

    // A suppressor and cathode on the same node by another name (both
    // grounded) is the cathode-tied wiring and builds.
    let spice = format!(
        "grounded cathode, grounded suppressor\n\
             P1 plate grid 0 screen 0 EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             {EL84_MODEL}\n"
    );
    let netlist = Netlist::parse(&spice).unwrap();
    MnaSystem::from_netlist(&netlist).expect("suppressor on the cathode node builds");
}

#[test]
fn test_pentode_parasitic_caps() {
    // Purely-resistive pentode circuit (no explicit caps in the netlist)
    // should pick up 5 auto-inserted junction caps:
    //   Cgk, Cgp, Cpk, Csk, Csp
    // Each junction adds 2 off-diagonal C entries (symmetric stamp), so
    // 5 junctions => 10 off-diagonal entries.
    let spice = format!(
        "Pentode parasitic\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.add_parasitic_caps();

    let p = *mna.node_map.get("plate").unwrap() - 1;
    let g = *mna.node_map.get("grid").unwrap() - 1;
    let k = *mna.node_map.get("cath").unwrap() - 1;
    let s = *mna.node_map.get("screen").unwrap() - 1;

    // Each pentode terminal accumulates one diagonal cap per junction it
    // touches. Topology recap: Cgk(g,k), Cgp(g,p), Cpk(p,k), Csk(s,k),
    // Csp(s,p). Resulting junction counts:
    //   plate: Cgp + Cpk + Csp = 3 caps
    //   grid : Cgk + Cgp        = 2 caps
    //   cath : Cgk + Cpk + Csk  = 3 caps
    //   screen: Csk + Csp       = 2 caps
    let approx = |actual: f64, expected_n: usize| {
        let expected = (expected_n as f64) * PARASITIC_CAP;
        (actual - expected).abs() < 1e-25
    };

    assert!(
        approx(mna.c[p][p], 3),
        "C[plate][plate] = {} (expected 3*PARASITIC_CAP)",
        mna.c[p][p]
    );
    assert!(
        approx(mna.c[g][g], 2),
        "C[grid][grid] = {} (expected 2*PARASITIC_CAP)",
        mna.c[g][g]
    );
    assert!(
        approx(mna.c[k][k], 3),
        "C[cath][cath] = {} (expected 3*PARASITIC_CAP)",
        mna.c[k][k]
    );
    assert!(
        approx(mna.c[s][s], 2),
        "C[screen][screen] = {} (expected 2*PARASITIC_CAP)",
        mna.c[s][s]
    );

    // Each junction must show up as a NEGATIVE off-diagonal entry on both
    // sides of the symmetric stamp (the `stamp_capacitor_raw` convention).
    let off = |a: usize, b: usize| (mna.c[a][b] + PARASITIC_CAP).abs() < 1e-25;
    assert!(off(g, k), "C[grid][cath] = {} (Cgk)", mna.c[g][k]);
    assert!(off(k, g), "C[cath][grid] = {} (Cgk)", mna.c[k][g]);
    assert!(off(g, p), "C[grid][plate] = {} (Cgp)", mna.c[g][p]);
    assert!(off(p, g), "C[plate][grid] = {} (Cgp)", mna.c[p][g]);
    assert!(off(p, k), "C[plate][cath] = {} (Cpk)", mna.c[p][k]);
    assert!(off(k, p), "C[cath][plate] = {} (Cpk)", mna.c[k][p]);
    assert!(off(s, k), "C[screen][cath] = {} (Csk)", mna.c[s][k]);
    assert!(off(k, s), "C[cath][screen] = {} (Csk)", mna.c[k][s]);
    assert!(off(s, p), "C[screen][plate] = {} (Csp)", mna.c[s][p]);
    assert!(off(p, s), "C[plate][screen] = {} (Csp)", mna.c[p][s]);

    // Sanity: total off-diagonal nonzero count over the 4 pentode
    // terminals should be exactly 10 (5 junctions x 2 entries each).
    let pentode_nodes = [p, g, k, s];
    let mut off_diag_count = 0;
    for &i in &pentode_nodes {
        for &j in &pentode_nodes {
            if i != j && mna.c[i][j].abs() > 1e-25 {
                off_diag_count += 1;
            }
        }
    }
    assert_eq!(
        off_diag_count, 10,
        "Pentode should have 10 off-diagonal C entries (5 junctions), got {}",
        off_diag_count
    );
}

// ===== Grid-off pentode (phase 1b) tests =====
//
// Grid-off reduction drops the Ig1 NR dimension (row/col 2) when DC-OP
// confirms Vgk is below cutoff, and freezes Vg2k at its DC-OP value.
// Remaining NR shape:
//   row/col 0: Ip  ↔ Vgk
//   row/col 1: Ig2 ↔ Vpk
// The resulting N_v has 4 nonzero entries; N_i has 4 nonzero entries.

#[test]
fn test_grid_off_pentode_stamp_shape() {
    // Pins the N_v / N_i shape the MNA builder stamps for a grid-off
    // pentode (the live path: `from_netlist_with_grid_off`), not a
    // stand-alone helper. Every terminal sits on its own node so no
    // entry cancels.
    let spice = format!(
        "grid-off stamp shape\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();
    let mut grid_off = std::collections::HashMap::new();
    grid_off.insert("P1".to_string(), 250.0);
    let mna = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off).unwrap();
    assert_eq!(mna.m, 2, "grid-off pentode is a 2D block");
    let start = mna.nonlinear_devices[0].start_idx;
    assert_eq!(start, 0);

    let p = *mna.node_map.get("plate").unwrap() - 1;
    let g = *mna.node_map.get("grid").unwrap() - 1;
    let k = *mna.node_map.get("cath").unwrap() - 1;
    let s = *mna.node_map.get("screen").unwrap() - 1;

    // ----- N_v rows -----
    // Row 0 = Vgk
    assert_eq!(mna.n_v[0][g], 1.0, "N_v[0][grid] = +1 (Vgk)");
    assert_eq!(mna.n_v[0][k], -1.0, "N_v[0][cath] = -1 (Vgk)");
    assert_eq!(mna.n_v[0][p], 0.0, "N_v[0][plate] = 0");
    assert_eq!(mna.n_v[0][s], 0.0, "N_v[0][screen] = 0 (Vg2k dropped)");

    // Row 1 = Vpk
    assert_eq!(mna.n_v[1][p], 1.0, "N_v[1][plate] = +1 (Vpk)");
    assert_eq!(mna.n_v[1][k], -1.0, "N_v[1][cath] = -1 (Vpk)");
    assert_eq!(mna.n_v[1][g], 0.0, "N_v[1][grid] = 0");
    assert_eq!(mna.n_v[1][s], 0.0, "N_v[1][screen] = 0");

    // Count nonzero entries — must be exactly 4.
    let nv_nz: usize = mna
        .n_v
        .iter()
        .map(|row| row.iter().filter(|&&v| v != 0.0).count())
        .sum();
    assert_eq!(nv_nz, 4, "grid-off N_v must have exactly 4 nonzero entries");

    // ----- N_i columns -----
    // Col 0 = Ip
    assert_eq!(mna.n_i[p][0], -1.0, "N_i[plate][0] = -1 (Ip out)");
    assert_eq!(mna.n_i[k][0], 1.0, "N_i[cath][0] = +1 (Ip in)");
    assert_eq!(mna.n_i[g][0], 0.0, "N_i[grid][0] = 0 (Ig1 dropped)");
    assert_eq!(mna.n_i[s][0], 0.0, "N_i[screen][0] = 0");

    // Col 1 = Ig2
    assert_eq!(mna.n_i[s][1], -1.0, "N_i[screen][1] = -1 (Ig2 out)");
    assert_eq!(mna.n_i[k][1], 1.0, "N_i[cath][1] = +1 (Ig2 in)");
    assert_eq!(mna.n_i[p][1], 0.0, "N_i[plate][1] = 0");
    assert_eq!(mna.n_i[g][1], 0.0, "N_i[grid][1] = 0");

    let ni_nz: usize = mna
        .n_i
        .iter()
        .map(|row| row.iter().filter(|&&v| v != 0.0).count())
        .sum();
    assert_eq!(ni_nz, 4, "grid-off N_i must have exactly 4 nonzero entries");
}

#[test]
fn test_from_netlist_with_grid_off_reduces_dimension() {
    let spice = format!(
        "grid-off dimension reduction\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();

    let mna_full = MnaSystem::from_netlist(&netlist).unwrap();
    assert_eq!(mna_full.m, 3, "sharp-cutoff pentode should have M=3");

    // vg2k_frozen value is a structural test — 250.0 V is a typical
    // EL84 screen bias, but the math isn't exercised here; only the
    // MNA dimension reduction is checked.
    let mut grid_off = std::collections::HashMap::new();
    grid_off.insert("P1".to_string(), 250.0);
    let mna_reduced = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off).unwrap();

    assert_eq!(
        mna_reduced.m,
        mna_full.m - 1,
        "grid-off pentode should reduce M by exactly 1"
    );
    assert_eq!(mna_reduced.m, 2, "grid-off pentode should have M=2");
    assert_eq!(mna_reduced.num_devices, 1);
    assert_eq!(mna_reduced.nonlinear_devices[0].dimension, 2);
}

#[test]
fn test_grid_off_pentode_stays_tube_device_type() {
    // The TubeKind::SharpPentodeGridOff discriminator lives on
    // TubeParams.kind at the codegen layer, not on NonlinearDeviceType.
    // The MNA layer must keep reporting `NonlinearDeviceType::Tube`
    // so that the same device-type switches in codegen / DK / NR
    // continue to dispatch correctly.
    let spice = format!(
        "grid-off device type\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();
    let mut grid_off = std::collections::HashMap::new();
    grid_off.insert("P1".to_string(), 250.0);
    let mna = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off).unwrap();

    let dev = &mna.nonlinear_devices[0];
    assert_eq!(
        dev.device_type,
        NonlinearDeviceType::Tube,
        "grid-off pentode must still report NonlinearDeviceType::Tube"
    );
    assert_eq!(dev.dimension, 2);
    assert_eq!(
        dev.nodes.len(),
        4,
        "nodes vector must still be plate/grid/cath/screen"
    );
}

#[test]
fn test_from_netlist_without_grid_off_unchanged() {
    // Byte-identity guard: passing an empty grid-off set must produce
    // a structurally identical MNA system to the default constructor.
    let spice = format!(
        "grid-off empty set identity\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();
    let mna_default = MnaSystem::from_netlist(&netlist).unwrap();
    let empty: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
    let mna_empty = MnaSystem::from_netlist_with_grid_off(&netlist, &empty).unwrap();

    assert_eq!(mna_default.n, mna_empty.n);
    assert_eq!(mna_default.m, mna_empty.m);
    assert_eq!(mna_default.g, mna_empty.g);
    assert_eq!(mna_default.c, mna_empty.c);
    assert_eq!(mna_default.n_v, mna_empty.n_v);
    assert_eq!(mna_default.n_i, mna_empty.n_i);
    assert_eq!(mna_default.num_devices, mna_empty.num_devices);
    assert_eq!(
        mna_default.nonlinear_devices[0].dimension,
        mna_empty.nonlinear_devices[0].dimension,
    );
    assert_eq!(
        mna_default.nonlinear_devices[0].device_type,
        mna_empty.nonlinear_devices[0].device_type,
    );
}

#[test]
fn test_grid_off_pentode_parasitic_caps_three_junctions() {
    // Grid-off pentodes should emit 3 junction caps (Cgk, Cgp, Cpk),
    // NOT the 5 of the sharp-cutoff case. CSK and CSP are dropped
    // because the screen is effectively an input (Vg2k is frozen),
    // not an NR unknown.
    let spice = format!(
        "grid-off parasitic caps\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
        EL84_MODEL
    );
    let netlist = Netlist::parse(&spice).unwrap();
    let mut grid_off = std::collections::HashMap::new();
    grid_off.insert("P1".to_string(), 250.0);
    let mut mna = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off).unwrap();
    mna.add_parasitic_caps();

    let p = *mna.node_map.get("plate").unwrap() - 1;
    let g = *mna.node_map.get("grid").unwrap() - 1;
    let k = *mna.node_map.get("cath").unwrap() - 1;
    let s = *mna.node_map.get("screen").unwrap() - 1;

    // Screen must not have any parasitic cap entries (CSK and CSP dropped).
    assert!(
        mna.c[s][k].abs() < 1e-25,
        "CSK must be absent for grid-off pentode, got {}",
        mna.c[s][k]
    );
    assert!(
        mna.c[k][s].abs() < 1e-25,
        "CSK (transpose) must be absent for grid-off pentode"
    );
    assert!(
        mna.c[s][p].abs() < 1e-25,
        "CSP must be absent for grid-off pentode, got {}",
        mna.c[s][p]
    );
    assert!(
        mna.c[p][s].abs() < 1e-25,
        "CSP (transpose) must be absent for grid-off pentode"
    );

    // Remaining junctions (Cgk, Cgp, Cpk) must still be present.
    let off = |a: usize, b: usize| (mna.c[a][b] + PARASITIC_CAP).abs() < 1e-25;
    assert!(off(g, k), "Cgk must still be present");
    assert!(off(g, p), "Cgp must still be present");
    assert!(off(p, k), "Cpk must still be present");

    // Off-diagonal count across the 4 pentode terminals should be 6
    // (3 junctions x 2 entries each).
    let pentode_nodes = [p, g, k, s];
    let mut off_diag_count = 0;
    for &i in &pentode_nodes {
        for &j in &pentode_nodes {
            if i != j && mna.c[i][j].abs() > 1e-25 {
                off_diag_count += 1;
            }
        }
    }
    assert_eq!(
        off_diag_count, 6,
        "grid-off pentode should have 6 off-diagonal C entries (3 junctions), got {}",
        off_diag_count
    );
}

// ── Triode linearization tests ──────────────────────────────────────

const TRIODE_12AX7_MODEL: &str = ".model 12AX7 VT(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)";

#[test]
fn test_linearized_triode_reduces_m_by_2() {
    let spice = format!(
        "triode linearization M reduction\n\
             V1 plate1 0 250\n\
             R1 grid1 0 1Meg\n\
             R2 cath1 0 1.5k\n\
             V2 plate2 0 250\n\
             R3 grid2 0 1Meg\n\
             R4 cath2 0 1.5k\n\
             T1 grid1 plate1 cath1 12AX7\n\
             T2 grid2 plate2 cath2 12AX7\n\
             {}\n",
        TRIODE_12AX7_MODEL
    );
    let netlist = crate::parser::Netlist::parse(&spice).unwrap();

    // Full MNA: both triodes as nonlinear → M=4
    let mna_full = MnaSystem::from_netlist(&netlist).unwrap();
    assert_eq!(mna_full.m, 4, "2 triodes should give M=4");

    // Linearize T1 → M should reduce by 2
    let mut lin_set = std::collections::HashSet::new();
    lin_set.insert("T1".to_string());
    let mna_lin1 = MnaSystem::from_netlist_with_all_reductions(
        &netlist,
        &std::collections::HashSet::new(),
        &std::collections::HashSet::new(),
        &lin_set,
        &std::collections::HashMap::new(),
    )
    .unwrap();
    assert_eq!(mna_lin1.m, 2, "linearizing T1 should give M=2");
    assert_eq!(
        mna_lin1.num_devices, 1,
        "only T2 should remain as nonlinear"
    );

    // Linearize both → M=0
    lin_set.insert("T2".to_string());
    let mna_lin2 = MnaSystem::from_netlist_with_all_reductions(
        &netlist,
        &std::collections::HashSet::new(),
        &std::collections::HashSet::new(),
        &lin_set,
        &std::collections::HashMap::new(),
    )
    .unwrap();
    assert_eq!(mna_lin2.m, 0, "linearizing both triodes should give M=0");
    assert_eq!(mna_lin2.num_devices, 0);
}

#[test]
fn test_stamp_linearized_triodes_gm() {
    // Verify gm VCCS stamping: G[p][g] += gm, G[p][k] -= gm,
    //                           G[k][g] -= gm, G[k][k] += gm
    let spice = format!(
        "triode gm stamp\n\
             R1 grid 0 1Meg\n\
             R2 plate 0 100k\n\
             R3 cath 0 1.5k\n\
             T1 grid plate cath 12AX7\n\
             {}\n",
        TRIODE_12AX7_MODEL
    );
    let netlist = crate::parser::Netlist::parse(&spice).unwrap();

    // Build linearized MNA (triode removed from NR)
    let mut lin_set = std::collections::HashSet::new();
    lin_set.insert("T1".to_string());
    let mut mna = MnaSystem::from_netlist_with_all_reductions(
        &netlist,
        &std::collections::HashSet::new(),
        &std::collections::HashSet::new(),
        &lin_set,
        &std::collections::HashMap::new(),
    )
    .unwrap();
    assert_eq!(mna.m, 0, "linearized triode should have M=0");

    // Save G matrix before stamping
    let g_before: Vec<Vec<f64>> = mna.g.clone();

    let g = *mna.node_map.get("grid").unwrap();
    let p = *mna.node_map.get("plate").unwrap();
    let k = *mna.node_map.get("cath").unwrap();

    // Stamp with known gm and gp values
    let gm = 1.5e-3; // typical 12AX7 gm
    let gp = 6.25e-5; // rp = 16k → 1/rp
    mna.linearized_triodes = vec![LinearizedTriodeInfo {
        name: "T1".to_string(),
        ng: g,
        np: p,
        nk: k,
        gm,
        gp,
        ip_dc: 1.2e-3,
        ig_dc: 0.0,
        vgk0: -1.5,
        vpk0: 150.0,
        ccg: 0.0,
        cgp: 0.0,
        ccp: 0.0,
        grid_onset: 0.0,
    }];
    mna.stamp_linearized_triodes();

    // Check combined gm + gp stamps (0-indexed)
    let gi = g - 1;
    let pi = p - 1;
    let ki = k - 1;
    let eps = 1e-12;

    // Expected deltas from gm VCCS + gp plate conductance:
    //   G[p][g] += gm
    //   G[p][k] -= gm - gp   (both gm VCCS and gp shunt contribute)
    //   G[p][p] += gp
    //   G[k][g] -= gm
    //   G[k][k] += gm + gp   (both contribute)
    //   G[k][p] -= gp
    assert!(
        (mna.g[pi][gi] - g_before[pi][gi] - gm).abs() < eps,
        "G[p][g] += gm"
    );
    assert!(
        (mna.g[pi][ki] - g_before[pi][ki] + gm + gp).abs() < eps,
        "G[p][k] -= (gm + gp)"
    );
    assert!(
        (mna.g[pi][pi] - g_before[pi][pi] - gp).abs() < eps,
        "G[p][p] += gp"
    );
    assert!(
        (mna.g[ki][gi] - g_before[ki][gi] + gm).abs() < eps,
        "G[k][g] -= gm"
    );
    assert!(
        (mna.g[ki][ki] - g_before[ki][ki] - gm - gp).abs() < eps,
        "G[k][k] += (gm + gp)"
    );
    assert!(
        (mna.g[ki][pi] - g_before[ki][pi] + gp).abs() < eps,
        "G[k][p] -= gp"
    );
}

#[test]
fn test_stamp_linearized_triode_dc_bias_currents() {
    let spice = format!(
        "triode dc bias\n\
             R1 grid 0 1Meg\n\
             R2 plate 0 100k\n\
             R3 cath 0 1.5k\n\
             T1 grid plate cath 12AX7\n\
             {}\n",
        TRIODE_12AX7_MODEL
    );
    let netlist = crate::parser::Netlist::parse(&spice).unwrap();

    let mut lin_set = std::collections::HashSet::new();
    lin_set.insert("T1".to_string());
    let mut mna = MnaSystem::from_netlist_with_all_reductions(
        &netlist,
        &std::collections::HashSet::new(),
        &std::collections::HashSet::new(),
        &lin_set,
        &std::collections::HashMap::new(),
    )
    .unwrap();

    let cs_count_before = mna.current_sources.len();
    let ip_dc = 1.2e-3;
    let ig_dc = 0.0; // no grid current (valid linearization)
    let gm = 1.5e-3;
    let gp = 6.25e-5;
    let vgk0 = -1.5;
    let vpk0 = 150.0;

    let g = *mna.node_map.get("grid").unwrap();
    let p = *mna.node_map.get("plate").unwrap();
    let k = *mna.node_map.get("cath").unwrap();

    mna.linearized_triodes = vec![LinearizedTriodeInfo {
        name: "T1".to_string(),
        ng: g,
        np: p,
        nk: k,
        gm,
        gp,
        ip_dc,
        ig_dc,
        vgk0,
        vpk0,
        ccg: 0.0,
        cgp: 0.0,
        ccp: 0.0,
        grid_onset: 0.0,
    }];
    mna.stamp_linearized_triodes();

    // Should add Ip_dc and Ik_dc current sources (Ig_dc skipped since ~0)
    assert_eq!(
        mna.current_sources.len(),
        cs_count_before + 2,
        "should add Ip_dc + Ik_dc (Ig_dc skipped when ~0)"
    );

    // Norton companion constants: injection INTO node = I_lin(v0) - I_dc.
    // I_lin at plate = gm*Vgk0 + gp*Vpk0 (the linear-model current drawn
    // out of the plate node by the gm/gp stamps at the OP).
    let i_lin_p = gm * vgk0 + gp * vpk0;

    // Ip_dc: injection at the plate node = i_lin_p - ip_dc
    let ip_src = mna
        .current_sources
        .iter()
        .find(|cs| cs.name.contains("Ip_dc"));
    assert!(ip_src.is_some(), "should have Ip_dc current source");
    let ip_src = ip_src.unwrap();
    assert_eq!(ip_src.n_plus_idx, p);
    assert_eq!(ip_src.n_minus_idx, 0);
    assert!(
        (ip_src.dc_value - (i_lin_p - ip_dc)).abs() < 1e-15,
        "plate Norton constant should be I_lin(v0) - Ip_dc, got {}",
        ip_src.dc_value
    );

    // Ik_dc: injection at the cathode node = -I_lin(v0) + Ip + Ig
    let ik_src = mna
        .current_sources
        .iter()
        .find(|cs| cs.name.contains("Ik_dc"));
    assert!(ik_src.is_some(), "should have Ik_dc current source");
    let ik_src = ik_src.unwrap();
    assert_eq!(ik_src.n_plus_idx, k);
    assert_eq!(ik_src.n_minus_idx, 0);
    assert!(
        (ik_src.dc_value - (-i_lin_p + ip_dc + ig_dc)).abs() < 1e-15,
        "cathode Norton constant should be -I_lin(v0) + Ip + Ig, got {}",
        ik_src.dc_value
    );
}

#[test]
fn test_linearized_triode_empty_set_identity() {
    // Empty linearized_triodes set should produce identical MNA to default
    let spice = format!(
        "triode empty set identity\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 cath 0 1.5k\n\
             T1 grid plate cath 12AX7\n\
             {}\n",
        TRIODE_12AX7_MODEL
    );
    let netlist = crate::parser::Netlist::parse(&spice).unwrap();

    let mna_default = MnaSystem::from_netlist(&netlist).unwrap();
    let mna_empty = MnaSystem::from_netlist_with_all_reductions(
        &netlist,
        &std::collections::HashSet::new(),
        &std::collections::HashSet::new(),
        &std::collections::HashSet::new(),
        &std::collections::HashMap::new(),
    )
    .unwrap();

    assert_eq!(mna_default.n, mna_empty.n);
    assert_eq!(mna_default.m, mna_empty.m);
    assert_eq!(mna_default.g, mna_empty.g);
    assert_eq!(mna_default.c, mna_empty.c);
    assert_eq!(mna_default.n_v, mna_empty.n_v);
    assert_eq!(mna_default.n_i, mna_empty.n_i);
    assert_eq!(mna_default.num_devices, mna_empty.num_devices);
}
