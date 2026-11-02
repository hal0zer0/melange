//! JFET `LEVEL=3` (the symmetric subthreshold law): the card is the level-1
//! card plus the slope factor `NSLOPE`, the level-2 keys are refused on it
//! and `NSLOPE` on the other levels, `.mismatch J` jitters it as a level-1
//! device, and the generated `jfet_sym_evaluate` runs on every solver route.
//! The law itself is pinned to the devices crate by
//! `template_primitive_sync_tests` and to its ngspice B-source transcription
//! by `melange-validate`'s `jfet3_twin_tests`.

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
fn level3_card_resolves_as_the_level1_card_plus_nslope() {
    let jp = &jfets(&deck("NJF(LEVEL=3)")).unwrap()[0];
    let sym = jp
        .sym
        .expect("LEVEL=3 resolves the symmetric subthreshold law");
    assert!(jp.ps.is_none());
    assert_eq!(
        (jp.vp, jp.idss, jp.lambda, sym.nslope),
        (-2.0, 1e-4 * 4.0, 0.0, 1.0)
    );
    assert_eq!(sym.scale(), 2.0 * melange_devices::VT_ROOM);

    let jp = &jfets(&deck(
        "NJF(LEVEL=3 VTO=-2.5 BETA=1.6m LAMBDA=4m NSLOPE=1.2)",
    ))
    .unwrap()[0];
    let sym = jp.sym.unwrap();
    assert_eq!(
        (jp.vp, jp.idss, jp.lambda, sym.nslope),
        (-2.5, 1.6e-3 * 6.25, 4e-3, 1.2)
    );
    assert_eq!(sym.scale(), 2.0 * 1.2 * melange_devices::VT_ROOM);

    // IDSS is accepted (it is BETA·VTO²), as is a catalog part.
    let jp = &jfets(&deck("NJF(LEVEL=3 VTO=-2.5 IDSS=10m)")).unwrap()[0];
    assert_eq!((jp.vp, jp.idss), (-2.5, 1e-2));
    let catalog =
        "jfet\nR1 in d 27k\nJ1 d g 0 J201\nRG g 0 1Meg\nC1 d 0 1n\n.model J201 NJF(LEVEL=3)\n.end\n";
    let jp = &jfets(catalog).unwrap()[0];
    let c = melange_devices::catalog::jfets::lookup("J201").unwrap();
    assert_eq!((jp.vp, jp.idss, jp.lambda), (c.vp, c.idss, c.lambda));
    assert!(jp.sym.is_some());

    // A P-channel card keeps the level-1 sign convention.
    let jp = &jfets(&deck("PJF(LEVEL=3 VTO=-1.5 BETA=1m)")).unwrap()[0];
    assert!(jp.is_p_channel && jp.vp == 1.5 && jp.sym.is_some());
}

#[test]
fn level3_refuses_idss_and_beta_together() {
    let e = refusal("NJF(LEVEL=3 VTO=-2.5 IDSS=10m BETA=1.6m)");
    assert!(
        e.contains("not both") && e.contains("IDSS = BETA·VTO²"),
        "{e}"
    );
    // Level 1 keeps its legacy preference for IDSS.
    let jp = &jfets(&deck("NJF(VTO=-2.5 IDSS=10m BETA=1m)")).unwrap()[0];
    assert_eq!(jp.idss, 1e-2);
}

#[test]
fn nslope_on_a_level1_or_level2_card_is_refused() {
    let e = refusal("NJF(VTO=-2.5 BETA=1.6m NSLOPE=1)");
    assert!(e.contains("NSLOPE") && e.contains("Add LEVEL=3"), "{e}");
    let e = refusal("NJF(LEVEL=2 VTO=-2.5 BETA=1.6m NSLOPE=1)");
    assert!(
        e.contains("NSLOPE") && e.contains("Parker–Skellern") && e.contains("Add LEVEL=3"),
        "{e}"
    );
}

#[test]
fn parker_skellern_keys_on_a_level3_card_are_refused() {
    for key in [
        "VST=26m", "MVST=0.1", "P=2", "Q=2", "Z=1", "XI=1000", "MXI=0", "PB=1",
    ] {
        let e = refusal(&format!("NJF(LEVEL=3 VTO=-2.5 BETA=1.6m {key})"));
        assert!(
            e.contains("LEVEL=3") && e.contains("Add LEVEL=2"),
            "{key}: {e}"
        );
    }
}

#[test]
fn nslope_is_validated() {
    for bad in ["NSLOPE=0.9", "NSLOPE=0", "NSLOPE=-1"] {
        let e = refusal(&format!("NJF(LEVEL=3 VTO=-2.5 BETA=1.6m {bad})"));
        assert!(
            e.contains("NSLOPE must be finite and at least 1"),
            "{bad}: {e}"
        );
    }
    jfets(&deck("NJF(LEVEL=3 VTO=-2.5 BETA=1.6m NSLOPE=1.0)")).unwrap();
}

#[test]
fn unsupported_level_names_all_three() {
    let e = refusal("NJF(LEVEL=4)");
    assert!(
        e.contains("LEVEL=4") && e.contains("LEVEL=2") && e.contains("LEVEL=3"),
        "{e}"
    );
}

const MISMATCH: &str = "mismatch\nR1 in d 27k\nJ1 d g 0 JX\nJ2 d g 0 JX\nRG g 0 1Meg\n\
                        .model JX NJF(LEVEL=3 VTO=-2.5 BETA=1.6m)\n.seed 3\n";

/// At LEVEL=3 the strength is IDSS, as at level 1: `.mismatch J IDSS` and
/// `VP` jitter the device, `BETA` is refused as the level-2 strength.
#[test]
fn level3_mismatch_is_the_level1_mismatch() {
    let j = jfets(&format!("{MISMATCH}.mismatch J IDSS=0.1 VP=0.05\n.end\n")).unwrap();
    for jp in &j {
        assert!(jp.idss != 1.6e-3 * 6.25 && (jp.idss / (1.6e-3 * 6.25) - 1.0).abs() <= 0.1);
        assert!(jp.vp != -2.5 && (jp.vp + 2.5).abs() <= 0.05 * 2.5);
        assert_eq!(jp.sym.unwrap().nslope, 1.0, "NSLOPE is not jittered");
    }
    assert_ne!(j[0].idss, j[1].idss, "per-device draws");
    let e = jfets(&format!("{MISMATCH}.mismatch J BETA=0.1\n.end\n")).unwrap_err();
    assert!(e.contains("BETA") && e.contains("use IDSS"), "{e}");
}

/// The 1176's gain-reduction element: a voltage-variable resistor biased
/// 0.1 V below pinch-off, its gate fed half the drain swing, driven through
/// the thermal tail, the knee, conduction and both signs of Vds.
const VVR: &str = "jfet vvr level 3
R1 in out 27k
J1 out g 0 JX
RA out g 1Meg
RB g b 1Meg
VB b 0 DC -5.2
C1 out 0 1n
.model JX NJF(LEVEL=3 VTO=-2.5 BETA=1.6m LAMBDA=4m)
.end
";

#[test]
fn level3_code_is_emitted_only_for_level3_devices() {
    let config = support::config_in_out_or_node1(VVR, 48000.0);
    let (code, _, _) = support::generate_circuit_code(VVR, &config);
    assert!(code.contains("fn jfet_sym_evaluate("));
    assert!(code.contains("const DEVICE_0_SYM_SCALE: f64"));
    assert!(code.contains("jfet_sym_evaluate(") && code.contains("state.device_0_idss"));
    assert!(
        !code.contains("jfet_ps"),
        "a level-3 deck carries no level-2 code"
    );
    let l1 = VVR.replace("LEVEL=3 ", "");
    let config = support::config_in_out_or_node1(&l1, 48000.0);
    let (code, _, _) = support::generate_circuit_code(&l1, &config);
    assert!(
        !code.contains("jfet_sym") && !code.contains("SYM_SCALE"),
        "a level-1 deck carries no level-3 code"
    );
}

/// The generated symmetric-subthreshold code compiles and runs on DK, nodal
/// Schur and nodal full-LU, and the three routes produce the same circuit.
/// The bound is the routes' own spread (the level-1 and level-2 versions of
/// this deck differ pairwise by about 5e-6 of the peak).
#[test]
fn level3_runs_on_every_route_and_the_routes_agree() {
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
        let tag = format!("jfet_sym_{route}_{sub:?}");
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

/// Below pinch-off the level-3 element conducts where level 1 is its 1e-12
/// leak: the same VVR deck's rest operating point has a drain node pulled
/// measurably toward the source by the thermal tail.
#[test]
fn level3_conducts_below_pinch_off_where_level1_does_not() {
    use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
    use melange_solver::mna::MnaSystem;
    // Rest the gate 60 mV below pinch-off, drain fed 10 mV through 27k (a
    // small-Vds resistor reading): under the smooth law the channel is
    // BETA·G'(−60 mV) ≈ 1.07e-5 S, a 93k resistor (drain at 7.7 mV); under
    // Shichman–Hodges it is open (drain at the 10 mV feed).
    let rest = |card: &str| {
        let spice = format!(
            "rest\nVD s 0 DC 10m\nR1 s d 27k\nJ1 d g 0 JX\nVG g 0 DC -2.56\nRin in 0 1k\n\
             .model JX {card}\n.end\n"
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
        let dc = solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
        assert!(dc.converged, "{card}: {:?}", dc.method);
        dc.v_node[mna.node_map["d"] - 1]
    };
    let l1 = rest("NJF(VTO=-2.5 BETA=1.6m)");
    let l3 = rest("NJF(LEVEL=3 VTO=-2.5 BETA=1.6m)");
    assert!(
        (l1 - 0.01).abs() < 1e-8,
        "level 1 is open below pinch-off: {l1}"
    );
    assert!(
        (l3 - 0.00775).abs() < 0.0003,
        "level 3 conducts (a ~93k resistor, drain at ~7.7 mV): {l3}"
    );
}
