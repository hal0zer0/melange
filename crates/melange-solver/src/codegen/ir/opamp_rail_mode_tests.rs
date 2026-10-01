//! Op-amp rail-mode resolver tests.

use super::*;
use crate::codegen::OpampRailMode;
use crate::mna::{MnaSystem, OpampInfo};

/// Build a minimal MNA with `opamps` attached. We don't care about the rest of
/// the MNA state for resolver tests — the resolver only reads `mna.opamps`.
fn mna_with_opamps(opamps: Vec<OpampInfo>) -> MnaSystem {
    let mut mna = MnaSystem::new(1, 0, 0, 0);
    mna.opamps = opamps;
    mna
}

fn opamp_with_rails(vcc: f64, vee: f64) -> OpampInfo {
    OpampInfo {
        name: "U_TEST".to_string(),
        n_plus_idx: 1,
        n_minus_idx: 2,
        n_out_idx: 3,
        aol: 200_000.0,
        r_out: 50.0,
        r_sag: crate::mna::OPAMP_DEFAULT_R_SAG_OHM,
        vcc,
        vee,
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
    }
}

#[test]
fn resolver_honors_explicit_user_choice_hard() {
    let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Hard);
    assert_eq!(r.mode, OpampRailMode::Hard);
    assert_eq!(r.reason, OpampRailModeReason::UserRequested);
}

#[test]
fn resolver_honors_explicit_user_choice_active_set() {
    let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::ActiveSet);
    assert_eq!(r.mode, OpampRailMode::ActiveSet);
    assert_eq!(r.reason, OpampRailModeReason::UserRequested);
}

#[test]
fn resolver_honors_explicit_user_choice_boyle() {
    let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::BoyleDiodes);
    assert_eq!(r.mode, OpampRailMode::BoyleDiodes);
    assert_eq!(r.reason, OpampRailModeReason::UserRequested);
}

#[test]
fn resolver_honors_explicit_none_even_with_clamped_opamps() {
    // User override must not be silently upgraded even when the circuit
    // would benefit from clamping. The escape hatch has to be trustworthy.
    let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::None);
    assert_eq!(r.mode, OpampRailMode::None);
    assert_eq!(r.reason, OpampRailModeReason::UserRequested);
}

#[test]
fn resolver_auto_no_opamps_picks_none() {
    let mna = mna_with_opamps(vec![]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::None);
    assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
}

#[test]
fn resolver_auto_opamps_without_rails_picks_none() {
    // Op-amps with infinite VCC and VEE are ideal VCCSs — no clamp needed.
    let mna = mna_with_opamps(vec![opamp_with_rails(f64::INFINITY, f64::NEG_INFINITY)]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::None);
    assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
}

// --- Cap-coupling helpers for resolver topology tests ---
//
// `mna_with_opamps` creates a 1-node MNA which isn't enough for cap
// stamps. This helper grows the C matrix to the requested node count
// and returns a ready-to-use MnaSystem with op-amps attached. Node
// numbering is 1-indexed (0 = ground) to match the MnaSystem convention.
fn mna_with_opamps_and_caps(
    n_nodes: usize,
    opamps: Vec<OpampInfo>,
    caps: &[(usize, usize, f64)],
) -> MnaSystem {
    let mut mna = MnaSystem::new(n_nodes, 0, 0, 0);
    mna.opamps = opamps;
    for &(i, j, c) in caps {
        // Convert 1-indexed inputs to 0-indexed matrix indices; skip
        // ground (0) terminals as usual.
        if i == 0 || j == 0 {
            continue;
        }
        let ii = i - 1;
        let jj = j - 1;
        mna.c[ii][ii] += c;
        mna.c[jj][jj] += c;
        mna.c[ii][jj] -= c;
        mna.c[jj][ii] -= c;
    }
    mna
}

fn opamp_at_nodes(np: usize, nm: usize, out: usize, vcc: f64, vee: f64) -> OpampInfo {
    OpampInfo {
        name: "U_TEST".to_string(),
        n_plus_idx: np,
        n_minus_idx: nm,
        n_out_idx: out,
        aol: 200_000.0,
        r_out: 50.0,
        r_sag: crate::mna::OPAMP_DEFAULT_R_SAG_OHM,
        vcc,
        vee,
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
    }
}

#[test]
fn resolver_auto_opamps_with_single_rail_picks_hard_when_dc_coupled() {
    // Single finite rail, no cap coupling → Hard mode.
    // Single-node MNA: out_idx=1, no caps at all.
    let mna = mna_with_opamps_and_caps(
        3,
        vec![opamp_at_nodes(2, 1, 1, 9.0, f64::NEG_INFINITY)],
        &[],
    );
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::Hard);
    assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
}

#[test]
fn resolver_auto_opamps_with_both_rails_picks_hard_when_dc_coupled() {
    let mna = mna_with_opamps_and_caps(3, vec![opamp_at_nodes(2, 1, 1, 9.0, 0.0)], &[]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::Hard);
    assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
}

#[test]
fn resolver_auto_feedback_cap_alone_is_dc_coupled() {
    // Op-amp with only a feedback cap between output (node 1) and its
    // own inverting input (node 1 here — a unity follower has nm = out).
    // No downstream coupling → Hard is safe.
    //
    // Topology: unity follower where output feeds its own - input.
    // feedback cap from output (1) to - input (1): self-loop, ignored.
    // Add a separate coupling cap from a different node (2) to ground
    // (doesn't touch op-amp output).
    let opamps = vec![opamp_at_nodes(3, 1, 1, 9.0, 0.0)]; // np=3, nm=1, out=1
    let mna = mna_with_opamps_and_caps(
        3,
        opamps,
        &[(2, 0, 1e-6)], // cap from node 2 to ground, not touching op-amp
    );
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::Hard);
    assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
}

#[test]
fn resolver_auto_feedback_cap_from_out_to_minus_input_stays_hard() {
    // Inverting amp: op-amp out=3, nm=2, np=1 (vbias). A feedback cap
    // from out (3) to nm (2) should NOT trigger ActiveSet — it's a
    // feedback cap, not a downstream coupling cap.
    let opamps = vec![opamp_at_nodes(1, 2, 3, 9.0, 0.0)];
    let mna = mna_with_opamps_and_caps(
        3,
        opamps,
        &[(3, 2, 820e-12)], // feedback cap between out (3) and - input (2)
    );
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::Hard);
    assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
}

#[test]
fn resolver_auto_cap_from_out_to_downstream_picks_active_set() {
    // Op-amp out=3, nm=2, np=1. Output coupling cap from node 3 to a
    // downstream node 4 (which is not the inverting input). This is
    // the output-coupling-cap pattern of an op-amp overdrive — must
    // trigger ActiveSet.
    let opamps = vec![opamp_at_nodes(1, 2, 3, 9.0, 0.0)];
    let mna = mna_with_opamps_and_caps(
        4,
        opamps,
        &[(3, 4, 4.7e-6)], // coupling cap from out (3) to downstream (4)
    );
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::ActiveSet);
    assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
}

#[test]
fn resolver_auto_mixed_opamps_one_ac_coupled_forces_active_set() {
    // Two op-amps: one with only feedback cap (safe on Hard), one with
    // downstream coupling cap (needs ActiveSet). Any offender forces
    // ActiveSet for the whole circuit because modes are global.
    let opamps = vec![
        opamp_at_nodes(1, 2, 3, 9.0, 0.0), // feedback only
        opamp_at_nodes(4, 5, 6, 9.0, 0.0), // will have downstream cap
    ];
    let mna = mna_with_opamps_and_caps(
        7,
        opamps,
        &[
            (3, 2, 820e-12), // feedback cap on first op-amp — OK
            (6, 7, 4.7e-6),  // downstream coupling on second op-amp — triggers ActiveSet
        ],
    );
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::ActiveSet);
    assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
}

#[test]
fn resolver_auto_opamp_with_zero_out_idx_ignored() {
    // An op-amp whose output is ground (out_idx = 0) can't be clamped;
    // the MNA builder would have dropped it, but defensively the resolver
    // should treat it as "not a clamp candidate".
    let mut oa = opamp_with_rails(9.0, 0.0);
    oa.n_out_idx = 0;
    let mna = mna_with_opamps(vec![oa]);
    let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(r.mode, OpampRailMode::None);
    assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
}

// Integration tests using synthetic circuits that exercise the same
// auto-detection code paths as the real circuits (which now live in
// the melange-audio/circuits repo).

fn parse_and_resolve(spice: &str) -> ResolvedOpampRailMode {
    let netlist =
        crate::parser::Netlist::parse(spice).unwrap_or_else(|e| panic!("failed to parse: {}", e));
    let mna =
        MnaSystem::from_netlist(&netlist).unwrap_or_else(|e| panic!("failed to build MNA: {}", e));
    resolve_opamp_rail_mode(&mna, OpampRailMode::Auto)
}

#[test]
fn opamp_with_ac_coupled_downstream_picks_active_set() {
    // Synthetic: op-amp with AC-coupled downstream stage.
    // Exercises the same AcCoupledDownstream path as an op-amp overdrive's topology.
    let spice = "\
Opamp AC-Coupled Downstream Test
R1 in sum 4.7k
R2 sum out 47k
C1 out out_ac 100n
R3 out_ac 0 100k
U1 0 sum out OA1
.model OA1 OA(AOL=100k GBW=3e6 ROUT=75 VCC=4.5 VEE=-4.5)
";
    let r = parse_and_resolve(spice);
    assert_eq!(r.mode, OpampRailMode::ActiveSet);
    assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
}

#[test]
fn circuit_without_opamps_picks_none() {
    // Synthetic: tubes and passives, no op-amps. Same path as a passive tube EQ.
    let spice = "\
No Op-Amp Test
R1 in grid 68k
R2 plate 0 100k
C1 grid 0 22p
V1 plate 0 DC 250
";
    let r = parse_and_resolve(spice);
    assert_eq!(r.mode, OpampRailMode::None);
    assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
}

#[test]
fn single_opamp_no_downstream_coupling_picks_concrete_mode() {
    // Synthetic: single op-amp, no AC-coupled downstream.
    // Must resolve to a concrete mode (never Auto).
    let spice = "\
Single Op-Amp Test
R1 in neg 10k
R2 neg out 100k
U1 in neg out OA1
.model OA1 OA(AOL=100k GBW=1e6 ROUT=100 VCC=15 VEE=-15)
";
    let r = parse_and_resolve(spice);
    assert_ne!(r.mode, OpampRailMode::Auto);
}

#[test]
fn augment_netlist_with_boyle_diodes_produces_valid_mna() {
    // Synthetic: 2 op-amps with finite rails. Tests that the Boyle
    // catch-diode augmentation helper synthesizes the correct elements
    // and that the augmented MNA builds without error.
    use crate::codegen::OpampRailMode;
    let spice = "\
Boyle Diodes Augmentation Test
R1 in sum1 4.7k
R2 sum1 out1 47k
C1 out1 out1_ac 100n
R3 out1_ac sum2 10k
R4 sum2 out 47k
C2 out out_ac 100n
R5 out_ac 0 100k
U1 0 sum1 out1 OA1
U2 0 sum2 out OA1
.model OA1 OA(AOL=100k GBW=3e6 ROUT=75 VCC=4.5 VEE=-4.5)
";
    let netlist = crate::parser::Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");

    // Sanity: 2 op-amps with finite rails.
    let clamped_opamps = mna
        .opamps
        .iter()
        .filter(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
        .count();
    assert_eq!(clamped_opamps, 2, "Should have 2 clamped op-amps");

    let aug_netlist = augment_netlist_with_boyle_diodes(&netlist, &mna);

    // Exactly one D_BOYLE_CATCH model.
    let catch_models = aug_netlist
        .models
        .iter()
        .filter(|m| m.name == BOYLE_CATCH_DIODE_MODEL)
        .count();
    assert_eq!(catch_models, 1);

    // Diode Is/N match Boyle-standard silicon.
    let catch_model = aug_netlist
        .models
        .iter()
        .find(|m| m.name == BOYLE_CATCH_DIODE_MODEL)
        .unwrap();
    assert_eq!(catch_model.model_type, "D");
    let is_val = catch_model
        .params
        .iter()
        .find(|(k, _)| k == "IS")
        .map(|(_, v)| *v)
        .unwrap();
    assert!(
        (is_val - 1e-15).abs() < 1e-20,
        "Is should be 1e-15, got {is_val}"
    );
    let n_val = catch_model
        .params
        .iter()
        .find(|(k, _)| k == "N")
        .map(|(_, v)| *v)
        .unwrap();
    assert!((n_val - 1.0).abs() < 1e-12, "N should be 1.0, got {n_val}");

    // Count the synthesized elements:
    //   * 2 op-amps × 2 rails × (1 VS + 1 diode) = 4 VS + 4 diodes
    //   * 2 op-amps × 1 buffer VCVS               = 2 VCVS
    let vs_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::VoltageSource { name, .. } if name.starts_with("V_boyle_"))
            })
            .count();
    let diodes_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::Diode { name, .. } if name.starts_with("D_boyle_"))
            })
            .count();
    let buffer_vcvs_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::Vcvs { name, .. } if name.starts_with("E_oa_buf_"))
            })
            .count();
    assert_eq!(
        vs_added, 4,
        "Expected 4 rail-reference voltage sources (2 op-amps × 2 rails)"
    );
    assert_eq!(
        diodes_added, 4,
        "Expected 4 catch diodes (2 op-amps × 2 rails)"
    );
    assert_eq!(
        buffer_vcvs_added, 2,
        "Expected 2 output-buffer VCVS (1 per clamped op-amp)"
    );

    let r_ro_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::Resistor { name, .. } if name.starts_with("R_oa_ro_"))
            })
            .count();
    assert_eq!(
        r_ro_added, 2,
        "Expected 2 output-buffer series resistors (1 per clamped op-amp)"
    );

    // Rebuild MNA from the augmented netlist — this must succeed.
    let aug_mna = MnaSystem::from_netlist(&aug_netlist).expect("augmented MNA build");

    // Dimensions grow by:
    //   * +8 nodes: 2 internal gain + 2 buffer-output + 4 rail-reference
    //   * +4 nonlinear devices (catch diodes)
    //   * +4 voltage sources (rail-reference DC sources)
    assert_eq!(
        aug_mna.n,
        mna.n + 8,
        "augmented n should grow by 4 per clamped op-amp (int + buf_out + 2 rail-ref)"
    );
    assert_eq!(
        aug_mna.m,
        mna.m + 4,
        "augmented m should grow by 2 per clamped op-amp"
    );
    assert_eq!(
        aug_mna.voltage_sources.len(),
        mna.voltage_sources.len() + 4,
        "augmented VS count should grow by 2 per clamped op-amp"
    );

    // Original nodes keep their indices.
    assert_eq!(mna.node_map["in"], aug_mna.node_map["in"]);
    assert_eq!(mna.node_map["out"], aug_mna.node_map["out"]);

    // Each clamped op-amp must now have a non-zero n_int_idx.
    for oa in &aug_mna.opamps {
        if oa.vcc.is_finite() || oa.vee.is_finite() {
            assert_ne!(
                oa.n_int_idx, 0,
                "op-amp {} should be in BoyleDiodes mode",
                oa.name
            );
        }
    }

    // Auto-detect on the un-augmented MNA picks ActiveSet (AC-coupled downstream).
    let resolved = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
    assert_eq!(resolved.mode, OpampRailMode::ActiveSet);
}

#[test]
fn multi_opamp_ac_coupled_picks_active_set() {
    // Synthetic: 3 op-amps with AC-coupled downstream stages.
    // Exercises the same path as a multi-op-amp leveling-amplifier topology.
    let spice = "\
Multi Op-Amp AC-Coupled Test
R1 in sum1 10k
R2 sum1 out1 100k
C1 out1 mid 100n
R3 mid sum2 10k
R4 sum2 out2 100k
C2 out2 out_ac 100n
R5 out_ac sum3 10k
R6 sum3 out 100k
R7 out 0 100k
U1 0 sum1 out1 OA1
U2 0 sum2 out2 OA1
U3 0 sum3 out OA1
.model OA1 OA(AOL=100k GBW=1e6 ROUT=100 VSAT=13)
";
    let r = parse_and_resolve(spice);
    assert_eq!(r.mode, OpampRailMode::ActiveSet);
    assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
}

/// Audio-path (a feedback clipper, output cap-coupled) and control-path (the
/// output drives a rectifier through a resistor) topologies used to resolve
/// to different modes, because a backward-Euler step on every rail-engaged
/// sample was thought to suit one and not the other. Both now resolve to
/// `ActiveSet`: under the charge form a pin or release leaves no carried
/// residual on capless rows in either topology, and `ActiveSetBe` is an
/// explicit mode only.
#[test]
fn audio_and_control_path_topologies_both_resolve_to_active_set() {
    let feedback_clipper = "\
Feedback Clipper Test (overdrive pedal pattern)
R1 in sum 10k
R2 sum clip_out 100k
C1 clip_out ac_out 1u
R3 ac_out 0 100k
D1 clip_out sum DCLIP
D2 sum clip_out DCLIP
U1 0 sum clip_out OA1
.model DCLIP D(IS=1e-14 N=1.9)
.model OA1 OA(AOL=200k ROUT=75 VCC=4.5 VEE=-4.5)
";
    let sidechain = "\
Sidechain Rectifier Test (compressor/ALC pattern)
R1 in sum 10k
R2 sum op_out 100k
C1 op_out ac_out 100n
R3 ac_out 0 100k
Rsc op_out sc_node 10k
D1 sc_node cv_node DRECT
Rrel cv_node 0 2MEG
U1 0 sum op_out OA1
.model DRECT D(IS=2e-9 N=1.906)
.model OA1 OA(AOL=200k ROUT=75 VCC=9 VEE=-9)
";
    for spice in [feedback_clipper, sidechain] {
        let r = parse_and_resolve(spice);
        assert_eq!(r.mode, OpampRailMode::ActiveSet);
        assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
    }
}
