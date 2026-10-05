//! JFET `LEVEL=2` (Parker–Skellern): the card is resolved as ngspice's JFET2
//! reads it, the keys melange has not built are refused, `.mismatch J` jitters
//! the level-2 strength (BETA) independently of VP, and the generated
//! `jfet_ps_evaluate` runs on every solver route. The law itself is pinned to
//! the devices crate by `template_primitive_sync_tests` and to ngspice by the
//! devices crate's own tests and `melange-validate`'s JFET2 twin.

mod support;

use melange_solver::codegen::ir::{CircuitIR, DeviceParams, JfetParams};
use melange_solver::codegen::{CodegenConfig, NodalSubPathOverride};
use melange_solver::parser::Netlist;

fn jfets(spice: &str) -> Result<Vec<JfetParams>, String> {
    let netlist = Netlist::parse(spice).map_err(|e| e.to_string())?;
    let slots = CircuitIR::build_device_info(&netlist).map_err(|e| e.to_string())?;
    Ok(slots
        .into_iter()
        .filter_map(|s| match s.params {
            DeviceParams::Jfet(jp) => Some(jp),
            _ => None,
        })
        .collect())
}

fn deck(card: &str) -> String {
    format!("jfet\nR1 in d 27k\nJ1 d g 0 JX\nRG g 0 1Meg\nC1 d 0 1n\n.model JX {card}\n.end\n")
}

fn refusal(card: &str) -> String {
    jfets(&deck(card)).expect_err(&format!("{card} must be refused"))
}

#[test]
fn level2_card_resolves_at_ngspice_defaults() {
    let jp = &jfets(&deck("NJF(LEVEL=2)")).unwrap()[0];
    let ps = jp.ps.expect("LEVEL=2 resolves the Parker–Skellern law");
    assert_eq!(jp.vp, -2.0);
    assert_eq!(jp.lambda, 0.0);
    assert_eq!(
        (ps.beta, ps.vst, ps.mvst, ps.p, ps.q, ps.z, ps.xi, ps.mxi, ps.vbi),
        (1e-4, 0.0, 0.0, 2.0, 2.0, 1.0, 1000.0, 0.0, 1.0)
    );
    let jp = &jfets(&deck(
        "NJF(LEVEL=2 VTO=-2.5 BETA=1.6m LAMBDA=4m VST=0.026 MVST=0.3 P=2.5 Q=1.8 Z=0.3 XI=3 \
         MXI=0.2 PB=0.8)",
    ))
    .unwrap()[0];
    let ps = jp.ps.unwrap();
    assert_eq!(
        (jp.vp, jp.lambda, ps.beta, ps.vst, ps.mvst, ps.p, ps.q, ps.z, ps.xi, ps.mxi, ps.vbi),
        (-2.5, 4e-3, 1.6e-3, 0.026, 0.3, 2.5, 1.8, 0.3, 3.0, 0.2, 0.8)
    );
}

#[test]
fn level1_is_shichman_hodges_with_or_without_the_key() {
    for card in ["NJF(VTO=-2.5 BETA=1.6m)", "NJF(LEVEL=1 VTO=-2.5 BETA=1.6m)"] {
        let jp = &jfets(&deck(card)).unwrap()[0];
        assert!(jp.ps.is_none(), "{card}");
        assert_eq!(jp.idss, 1.6e-3 * 2.5 * 2.5, "{card}");
    }
}

#[test]
fn unsupported_level_is_refused() {
    let e = refusal("NJF(LEVEL=3)");
    assert!(e.contains("LEVEL=3") && e.contains("LEVEL=2"), "{e}");
}

#[test]
fn parker_skellern_keys_on_a_level1_card_are_refused() {
    for key in [
        "VST=26m", "MVST=0.1", "P=2", "Q=2", "Z=1", "XI=1000", "MXI=0", "PB=1",
    ] {
        let e = refusal(&format!("NJF(VTO=-2.5 BETA=1.6m {key})"));
        assert!(e.contains("Add LEVEL=2"), "{key}: {e}");
    }
}

#[test]
fn idss_at_level2_is_refused_naming_beta_and_vto() {
    let e = refusal("NJF(LEVEL=2 VTO=-2.5 IDSS=10m)");
    assert!(e.contains("IDSS") && e.contains("set BETA and VTO"), "{e}");
}

#[test]
fn a_catalog_part_at_level2_needs_beta_and_vto() {
    let card = |params: &str| {
        format!("jfet\nR1 in d 27k\nJ1 d g 0 J201\nRG g 0 1Meg\n.model J201 NJF({params})\n.end\n")
    };
    let e = jfets(&card("LEVEL=2 VTO=-0.8")).unwrap_err();
    assert!(e.contains("give BETA and VTO"), "{e}");
    let jp = &jfets(&card("LEVEL=2 VTO=-0.8 BETA=1m")).unwrap()[0];
    assert_eq!((jp.vp, jp.ps.unwrap().beta), (-0.8, 1e-3));
}

#[test]
fn unbuilt_parker_skellern_keys_are_refused_when_nonzero() {
    for key in [
        "LFGAM", "LFG1", "LFG2", "HFGAM", "HFG1", "HFG2", "HFETA", "HFE1", "HFE2", "TAUG", "DELTA",
        "TAUD", "IBD", "VBD", "FC", "ACGAM", "XC",
    ] {
        let e = refusal(&format!("NJF(LEVEL=2 VTO=-2.5 BETA=1.6m {key}=0.1)"));
        assert!(e.contains("is refused"), "{key}: {e}");
        jfets(&deck(&format!("NJF(LEVEL=2 VTO=-2.5 BETA=1.6m {key}=0)")))
            .unwrap_or_else(|e| panic!("{key}=0 is the model without it: {e}"));
    }
}

#[test]
fn level2_shape_keys_are_validated() {
    for (key, why) in [
        ("BETA=0", "BETA"),
        ("VST=-1m", "VST"),
        ("P=0", "P must"),
        ("Q=-1", "Q must"),
        ("Z=-0.5", "Z must"),
        ("XI=0", "XI"),
        ("MXI=-0.1", "MXI"),
        ("PB=-3", "PB"),
    ] {
        let e = refusal(&format!("NJF(LEVEL=2 VTO=-2.5 {key})"));
        assert!(e.contains(why), "{key}: {e}");
    }
}

const MISMATCH: &str = "mismatch\nR1 in d 27k\nJ1 d g 0 JX\nJ2 d g 0 JX\nRG g 0 1Meg\n\
                        .model JX NJF(LEVEL=2 VTO=-2.5 BETA=1.6m)\n.seed 3\n";

/// At LEVEL=2 the card's BETA and VTO are the device's state: `.mismatch J`
/// jitters BETA and VP independently (ngspice's view of a VTO mismatch with
/// BETA fixed), and the displayed IDSS follows as BETA·VP².
#[test]
fn level2_mismatch_jitters_beta_and_vp_independently() {
    let vp_only = jfets(&format!("{MISMATCH}.mismatch J VP=0.05\n.end\n")).unwrap();
    for jp in &vp_only {
        assert_eq!(
            jp.ps.unwrap().beta,
            1.6e-3,
            "BETA must hold under a VP mismatch"
        );
        assert!(jp.vp != -2.5 && (jp.vp + 2.5).abs() <= 0.05 * 2.5);
        assert_eq!(jp.idss, 1.6e-3 * jp.vp * jp.vp);
    }
    assert_ne!(vp_only[0].vp, vp_only[1].vp, "per-device draws");
    let both = jfets(&format!("{MISMATCH}.mismatch J BETA=0.1 VP=0.05\n.end\n")).unwrap();
    for (jp, vp_jp) in both.iter().zip(&vp_only) {
        let beta = jp.ps.unwrap().beta;
        assert!(beta != 1.6e-3 && (beta / 1.6e-3 - 1.0).abs() <= 0.1);
        assert_eq!(jp.vp, vp_jp.vp, "the VP draw does not depend on BETA's");
    }
}

#[test]
fn a_mismatch_strength_key_no_jfet_reads_is_refused() {
    let e = jfets(&format!("{MISMATCH}.mismatch J IDSS=0.1\n.end\n")).unwrap_err();
    assert!(e.contains("IDSS") && e.contains("use BETA"), "{e}");
    let l1 = "mismatch\nR1 in d 27k\nJ1 d g 0 JX\nRG g 0 1Meg\n\
              .model JX NJF(VTO=-2.5 BETA=1.6m)\n.mismatch J BETA=0.1\n.end\n";
    let e = jfets(l1).unwrap_err();
    assert!(e.contains("BETA") && e.contains("use IDSS"), "{e}");
    // A deck with both levels reads both keys.
    let mixed = "mismatch\nR1 in d 27k\nJ1 d g 0 JA\nJ2 d g 0 JB\nRG g 0 1Meg\n\
                 .model JA NJF(VTO=-2.5 BETA=1.6m)\n.model JB NJF(LEVEL=2 VTO=-2.5 BETA=1.6m)\n\
                 .mismatch J IDSS=0.1 BETA=0.1\n.end\n";
    let j = jfets(mixed).unwrap();
    assert!(j[0].idss != 1.6e-3 * 6.25 && j[1].ps.unwrap().beta != 1.6e-3);
}

/// A voltage-variable resistor biased below pinch-off, its gate fed half the
/// drain swing: driven through cut-off, subthreshold, conduction and reverse
/// mode.
const VVR: &str = "jfet vvr level 2
R1 in out 27k
J1 out g 0 JX
RA out g 1Meg
RB g b 1Meg
VB b 0 DC -5.2
C1 out 0 1n
.model JX NJF(LEVEL=2 VTO=-2.5 BETA=1.6m LAMBDA=4m VST=26m MVST=0.2)
.end
";

#[test]
fn level2_code_is_emitted_only_for_level2_devices() {
    let config = support::config_in_out_or_node1(VVR, 48000.0);
    let (code, _, _) = support::generate_circuit_code(VVR, &config);
    assert!(code.contains("fn jfet_ps_evaluate("));
    assert!(code.contains("const DEVICE_0_PS: [f64; 8]"));
    assert!(code.contains("state.device_0_beta"));
    assert!(!code.contains("device_0_idss"));
    let l1 = VVR.replace("LEVEL=2 ", "").replace(" VST=26m MVST=0.2", "");
    let config = support::config_in_out_or_node1(&l1, 48000.0);
    let (code, _, _) = support::generate_circuit_code(&l1, &config);
    assert!(
        !code.contains("jfet_ps"),
        "a level-1 deck carries no level-2 code"
    );
}

/// The generated Parker–Skellern code compiles and runs on DK, nodal Schur and
/// nodal full-LU, and the three routes produce the same circuit. The bound is
/// the routes' own spread: the level-1 version of this deck differs pairwise
/// by 2.6–3.7 µV on a 0.62 V peak (≈ 5e-6), as this one does.
#[test]
fn level2_runs_on_every_route_and_the_routes_agree() {
    const FS: f64 = 48000.0;
    let mut runs = Vec::new();
    for (route, sub) in [
        ("dk", NodalSubPathOverride::Auto),
        ("nodal", NodalSubPathOverride::Schur),
        ("nodal", NodalSubPathOverride::FullLu),
    ] {
        let config = CodegenConfig {
            nodal_sub_path_override: sub,
            ..support::config_in_out_or_node1(VVR, FS)
        };
        let tag = format!("jfet_ps_{route}_{sub:?}");
        let circuit = if route == "dk" {
            support::build_circuit(VVR, &config, &tag)
        } else {
            support::build_circuit_nodal(VVR, &config, &tag)
        };
        let out = support::run_sine(&circuit, 1000.0, 2.0, 4800, FS);
        support::assert_finite(&out);
        support::assert_peak_above(&out, 0.3);
        runs.push((format!("{route}/{sub:?}"), out));
    }
    let peak = runs[0].1.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    for (i, (name_a, a)) in runs.iter().enumerate() {
        for (name_b, b) in &runs[i + 1..] {
            let worst = a
                .iter()
                .zip(b)
                .fold(0.0f64, |m, (x, y)| m.max((x - y).abs()));
            assert!(
                worst <= 2e-5 * peak,
                "{name_b} differs from {name_a} by {worst:e} V (peak {peak:e} V)"
            );
        }
    }
}
