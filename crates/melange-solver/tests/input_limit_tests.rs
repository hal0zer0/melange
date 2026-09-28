//! The generated code clamps its input to +/-INPUT_LIMIT_V and replaces
//! NaN/Inf by 0 V. Both change what the circuit is driven with, so both are
//! counted (`diag_input_clamp_count`, `diag_input_nan_count`) on every route,
//! for single- and multi-input builds. The CLI and validate refuse on either.

mod support;

use melange_solver::codegen::NodalSubPathOverride;

/// Drive [0, 150, -150, NaN, +inf, 50, -100] and print
/// "clamp nan limit" from the build's counters.
fn counts(code: &str, multi: bool, tag: &str) -> (u64, u64, f64) {
    let call = if multi {
        "let _ = process_sample([x, 0.0], &mut s);"
    } else {
        "let _ = process_sample(x, &mut s);"
    };
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    for x in [0.0f64, 150.0, -150.0, f64::NAN, f64::INFINITY, 50.0, -100.0] {{ {call} }}
    println!(\"{{}} {{}} {{}}\", s.diag_input_clamp_count, s.diag_input_nan_count, INPUT_LIMIT_V);
}}"
    );
    let out = support::compile_and_run(code, &main, tag).stdout;
    let v: Vec<f64> = out.split_whitespace().map(|t| t.parse().unwrap()).collect();
    (v[0] as u64, v[1] as u64, v[2])
}

const RC: &str = "rc\nR1 in out 1k\nC1 out 0 100n\n";
const DIODE: &str =
    "clipper\nR1 in out 1k\nD1 out 0 DX\nD2 0 out DX\n.model DX D(IS=2.52n N=1.752)\n";
const SAT: &str = "sat\nR1 in out 99\nL1 out 0 1 ISAT=10m LAIR=3e-4\n";
const TWO_IN: &str = "two inputs\nR1 in out 1k\nR2 in2 out 1k\nC1 out 0 100n\n";

/// Exactly the two out-of-range samples are clamped (-100 is AT the limit, not
/// beyond it) and exactly the two non-finite ones are replaced, on DK, nodal
/// Schur, nodal full-LU (saturating) and a two-input build.
#[test]
fn input_clamp_and_nan_are_counted_on_every_route() {
    let dk = support::generate_circuit_code(DIODE, &support::config_for_spice(DIODE, 48000.0)).0;
    let mut schur_cfg = support::config_for_spice(DIODE, 48000.0);
    schur_cfg.nodal_sub_path_override = NodalSubPathOverride::Schur;
    let schur = support::generate_circuit_code_nodal(DIODE, &schur_cfg).0;
    let full_lu =
        support::generate_circuit_code_nodal(SAT, &support::config_for_spice(SAT, 48000.0)).0;
    let linear = support::generate_circuit_code(RC, &support::config_for_spice(RC, 48000.0)).0;
    let mut multi_cfg = support::config_for_spice(TWO_IN, 48000.0);
    let in2 = {
        let netlist = melange_solver::parser::Netlist::parse(TWO_IN).unwrap();
        melange_solver::mna::MnaSystem::from_netlist(&netlist)
            .unwrap()
            .node_map["in2"]
            - 1
    };
    multi_cfg.extra_input_nodes = vec![in2];
    multi_cfg.extra_input_resistances = vec![1.0];
    let multi = support::generate_circuit_code_nodal(TWO_IN, &multi_cfg).0;
    for (name, code, is_multi) in [
        ("dk", &dk, false),
        ("schur", &schur, false),
        ("full_lu", &full_lu, false),
        ("linear", &linear, false),
        ("multi", &multi, true),
    ] {
        let (clamp, nan, limit) = counts(code, is_multi, &format!("inlim_{name}"));
        assert_eq!(limit, 100.0, "{name}: INPUT_LIMIT_V");
        assert_eq!((clamp, nan), (2, 2), "{name}: (clamped, NaN/Inf) counts");
    }
}
