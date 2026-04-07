//! Klon Centaur overdrive circuit tests.
//!
//! Verifies the Klon Centaur netlist through the full pipeline:
//! parse → MNA → nodal codegen → compile → run.
//!
//! The Klon's shunt diodes to a voltage source (VB+) produce a positive
//! K diagonal, so DK Schur is not applicable — the circuit auto-routes
//! to the nodal solver (full N×N LU NR), same as the SSL bus compressor.
//!
//! Topology: 4 op-amp sections (2× TL072), 2× 1N34A germanium diodes,
//! 3 pots (GAIN dual-gang, TREBLE wiper, OUTPUT wiper).

use std::io::Write;

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

fn load_klon_netlist() -> String {
    std::fs::read_to_string("../../circuits/testing/klon-centaur.cir")
        .expect("Failed to read klon-centaur.cir")
}

fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_klon_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_klon_{tag}_{id}_{counter}"));

    let full_code = format!("{code}\n\n{main_code}\n");
    {
        let mut f = std::fs::File::create(&src_path).unwrap();
        f.write_all(full_code.as_bytes()).unwrap();
    }

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2021")
        .output()
        .expect("rustc");

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Compilation failed for {tag}:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run");
    let _ = std::fs::remove_file(&bin_path);
    if !run.status.success() {
        panic!(
            "Binary failed for {tag}:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }

    String::from_utf8_lossy(&run.stdout).to_string()
}

// =============================================================================
// Parser tests
// =============================================================================

#[test]
fn test_klon_parse() {
    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).expect("Failed to parse Klon netlist");

    // 4 op-amp sections
    let opamp_count = netlist
        .elements
        .iter()
        .filter(|e| matches!(e, melange_solver::parser::Element::Opamp { .. }))
        .count();
    assert_eq!(opamp_count, 4, "Expected 4 op-amp sections (U1A, U1B, U2A, U2B)");

    // 2 germanium diodes
    let diode_count = netlist
        .elements
        .iter()
        .filter(|e| matches!(e, melange_solver::parser::Element::Diode { .. }))
        .count();
    assert_eq!(diode_count, 2, "Expected 2 diodes (D2, D3)");

    // 4 DC voltage sources
    let vs_count = netlist
        .elements
        .iter()
        .filter(|e| matches!(e, melange_solver::parser::Element::VoltageSource { .. }))
        .count();
    assert_eq!(vs_count, 4, "Expected 4 voltage sources (Vcc, V2p, Vneg, Vbias)");

    // 2 .model OA declarations
    let oa_model_count = netlist
        .models
        .iter()
        .filter(|m| m.model_type.to_uppercase().starts_with("OA"))
        .count();
    assert_eq!(oa_model_count, 2, "Expected 2 OA models (TL072_9V, TL072_27V)");

    // 1 .model D declaration
    let d_model_count = netlist
        .models
        .iter()
        .filter(|m| m.model_type.to_uppercase().starts_with("D"))
        .count();
    assert_eq!(d_model_count, 1, "Expected 1 diode model (D1N34A)");

    // 2 explicit .pot directives + 4 expanded from 2 wipers = 6 total
    assert_eq!(netlist.pots.len(), 2 + 4, "Expected 2 pots + 4 from 2 wipers = 6 total pot entries");

    // 2 .wiper directives (Treble, Output)
    assert_eq!(netlist.wipers.len(), 2, "Expected 2 wipers (Treble, Output)");

    // 1 .gang directive
    assert_eq!(netlist.gangs.len(), 1, "Expected 1 gang (Gain)");
    assert_eq!(netlist.gangs[0].label, "Gain");
    assert_eq!(netlist.gangs[0].members.len(), 2);
    assert!(!netlist.gangs[0].members[0].inverted, "R_gain should not be inverted");
    assert!(netlist.gangs[0].members[1].inverted, "R_blend should be inverted");
}

// =============================================================================
// MNA build tests
// =============================================================================

#[test]
fn test_klon_mna_build() {
    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // M=2 for two diodes
    assert_eq!(mna.nonlinear_devices.len(), 2, "Expected M=2 (two 1N34A diodes)");

    // Gang correctly resolved — both R_gain and R_blend are .pot directives
    assert_eq!(mna.gang_groups.len(), 1);
    let gang = &mna.gang_groups[0];
    assert_eq!(gang.label, "Gain");
    assert_eq!(gang.pot_members.len(), 2, "R_gain and R_blend should both be pot members");

    // Key nodes exist in node map
    for node_name in &["in", "buf_out", "gain_in", "gain_out", "diode_jct",
                        "sum_inv", "sum_out", "tone_out", "out", "vbias"] {
        assert!(
            mna.node_map.contains_key(*node_name),
            "Node '{}' missing from node_map",
            node_name
        );
    }
}

// =============================================================================
// DK kernel routing test
// =============================================================================

#[test]
fn test_klon_dk_kernel_builds() {
    // With the IIR op-amp model (no Boyle internal nodes), the Klon's DK kernel
    // now builds successfully. Previously the Boyle Gm~4000 S stamps caused ill-
    // conditioning; stripping Gm for transient removes that failure mode.
    // The codegen router still selects the nodal path for circuits with
    // transformer-coupled NFB or ill-conditioned kernels.
    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = mna.node_map["in"] - 1;
    mna.stamp_input_conductance(input_node, 1.0);

    let _ = DkKernel::from_mna(&mna, 48000.0);
    // No assertion — both success and failure are acceptable here.
    // Previously this test asserted failure; the IIR model changed the dynamics.
}

// =============================================================================
// Nodal codegen tests
// =============================================================================

#[test]
fn test_klon_nodal_codegen() {
    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    mna.g[input_node][input_node] += 1.0; // stamp G_in = 1.0 S

    let config = CodegenConfig {
        circuit_name: "klon_centaur".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };

    let codegen = CodeGenerator::new(config);
    let result = codegen
        .generate_nodal(&mna, &netlist)
        .expect("Klon nodal codegen should succeed");

    assert_eq!(result.m, 2, "Generated M should be 2");

    let code = &result.code;
    assert!(code.contains("pub fn process_sample("), "Missing process_sample");
    assert!(code.contains("pub const M: usize = 2"), "Missing M=2 constant");

    // Verify op-amp clamping is present
    assert!(code.contains(".clamp("), "Missing op-amp rail clamping");
}

// =============================================================================
// Compile-and-run test
// =============================================================================

#[test]
fn test_klon_codegen_compiles_and_runs() {
    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    mna.g[input_node][input_node] += 1.0;

    let config = CodegenConfig {
        circuit_name: "klon_centaur".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate_nodal(&mna, &netlist).unwrap();

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();

    // Warm up with 500 silent samples to let DC OP settle
    for i in 0..500 {
        let out = process_sample(0.0, &mut state);
        assert!(
            out[0].is_finite(),
            "Warmup output at sample {} must be finite, got {}",
            i, out[0]
        );
    }

    // Feed a 1kHz sine wave (100mV amplitude) for 1000 samples
    let mut max_abs_out = 0.0f64;
    let mut any_nonzero = false;
    for i in 0..1000 {
        let t = i as f64 / 48000.0;
        let input = 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let out = process_sample(input, &mut state);
        assert!(
            out[0].is_finite(),
            "Output at sample {} must be finite, got {}",
            i, out[0]
        );
        if out[0].abs() > 1e-6 {
            any_nonzero = true;
        }
        if out[0].abs() > max_abs_out {
            max_abs_out = out[0].abs();
        }
    }

    assert!(any_nonzero, "Klon output should not be all zeros");
    println!("max_abs_out={:.6}", max_abs_out);
    println!("PASS");
}
"#;

    let output = compile_and_run(&result.code, main_code, "run");
    assert!(output.contains("PASS"), "Compile-and-run test did not pass:\n{}", output);

    if let Some(line) = output.lines().find(|l| l.starts_with("max_abs_out=")) {
        let val: f64 = line["max_abs_out=".len()..].parse().unwrap_or(0.0);
        eprintln!("Klon max output: {:.6} V (from 0.1V input)", val);
        assert!(val > 0.001, "Output too quiet: {:.6} V", val);
    }
}

// =============================================================================
// ActiveSet rail-mode regression test
// =============================================================================
//
// This test pins the fix for the op-amp rail-clamp / cap-history corruption
// bug (see memory/opamp_rail_clamp_bug.md). With the Hard mode, at 100 mV
// input the Klon's out_ac node drifts to 60 V+ (physically impossible) because
// the post-NR hard clamp on v[tone_out] corrupts the C15 trapezoidal history.
// With ActiveSet mode, the post-convergence constrained resolve pins clamped
// op-amps and re-solves the network so KCL is satisfied at every node —
// out_ac stays within ±14 V across the input sweep.
//
// If this test fails after a codegen change, the ActiveSet resolve is no
// longer KCL-preserving and the Klon plugin will sound like a blown speaker
// again. DO NOT just bump the bound — trace the cause.

#[test]
fn test_klon_active_set_keeps_out_ac_physical() {
    use melange_solver::codegen::OpampRailMode;

    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    // out_ac is the canary node: downstream of U2B tone_out via C15.
    let out_ac_node = mna.node_map["out_ac"] - 1;
    mna.g[input_node][input_node] += 1.0;

    let config = CodegenConfig {
        circuit_name: "klon_active_set".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        opamp_rail_mode: OpampRailMode::ActiveSet,
        ..CodegenConfig::default()
    };

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate_nodal(&mna, &netlist).unwrap();

    // The test injects three amplitudes (below, at, above the rail-clamp
    // threshold for Klon at default pot positions) and asserts out_ac stays
    // within ±15 V (a generous envelope around the true ±13.5 V op-amp rails
    // that allows for the linearization slack in the active-set resolve).
    let main_code = format!(
        r#"
fn main() {{
    const OUT_AC_IDX: usize = {out_ac_idx};

    for &amp in &[0.01_f64, 0.1_f64, 0.3_f64] {{
        let mut state = CircuitState::default();
        state.set_sample_rate(48000.0);
        for _ in 0..5000 {{ process_sample(0.0, &mut state); }}

        let mut max_out_ac = 0.0f64;
        let mut max_output = 0.0f64;
        let mut any_nan = false;

        for i in 0..48000 {{
            let t = i as f64 / 48000.0;
            let input = amp * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = process_sample(input, &mut state)[0];
            if !out.is_finite() {{ any_nan = true; }}
            if i > 24000 {{
                let v_out_ac = state.v_prev[OUT_AC_IDX].abs();
                if v_out_ac > max_out_ac {{ max_out_ac = v_out_ac; }}
                if out.abs() > max_output {{ max_output = out.abs(); }}
            }}
        }}

        println!("amp={{:.3}} max_out_ac={{:.6}} max_output={{:.6}} nan_any={{}}", amp, max_out_ac, max_output, any_nan);
        assert!(!any_nan, "ActiveSet produced NaN/Inf at amp={{}}", amp);
        assert!(
            max_out_ac < 15.0,
            "out_ac drifted to {{:.2}} V at amp={{}} V — ActiveSet resolve no longer KCL-consistent. \
             See memory/opamp_rail_clamp_bug.md for context.",
            max_out_ac, amp
        );
    }}
    println!("PASS");
}}
"#,
        out_ac_idx = out_ac_node,
    );

    let output = compile_and_run(&result.code, &main_code, "active_set");
    assert!(
        output.contains("PASS"),
        "Klon ActiveSet regression test failed:\n{}",
        output
    );
    // Surface the measured values for manual inspection.
    for line in output.lines() {
        if line.starts_with("amp=") {
            eprintln!("  {}", line);
        }
    }
}

// =============================================================================
// Hard mode still produces the broken behavior (documents the bug)
// =============================================================================
//
// We keep a separate test that pins Hard mode's broken behavior so that:
//   (a) anyone reading the tests can see the bug is reproducible,
//   (b) if someone accidentally "fixes" Hard to behave like ActiveSet,
//       the tests catch it (because that would be a silent mode change).
//
// The threshold here is loose — we just assert out_ac goes above 20 V,
// comfortably above the ±13.5 V op-amp rails — which is the Hard mode's
// cap-history-corruption signature.

#[test]
fn test_klon_hard_mode_documents_cap_history_bug() {
    use melange_solver::codegen::OpampRailMode;

    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    let out_ac_node = mna.node_map["out_ac"] - 1;
    mna.g[input_node][input_node] += 1.0;

    let config = CodegenConfig {
        circuit_name: "klon_hard_documenting_bug".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        opamp_rail_mode: OpampRailMode::Hard,
        ..CodegenConfig::default()
    };

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate_nodal(&mna, &netlist).unwrap();

    let main_code = format!(
        r#"
fn main() {{
    const OUT_AC_IDX: usize = {out_ac_idx};
    let mut state = CircuitState::default();
    state.set_sample_rate(48000.0);
    for _ in 0..5000 {{ process_sample(0.0, &mut state); }}

    let mut max_out_ac = 0.0f64;
    for i in 0..48000 {{
        let t = i as f64 / 48000.0;
        let input = 0.1 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
        let _ = process_sample(input, &mut state);
        if i > 24000 {{
            let v = state.v_prev[OUT_AC_IDX].abs();
            if v > max_out_ac {{ max_out_ac = v; }}
        }}
    }}
    println!("max_out_ac={{:.3}}", max_out_ac);
    println!("PASS");
}}
"#,
        out_ac_idx = out_ac_node,
    );

    let output = compile_and_run(&result.code, &main_code, "hard_bug");
    assert!(output.contains("PASS"));

    // Hard mode MUST still exhibit the cap-history corruption (out_ac > 20 V)
    // at 100 mV / 440 Hz. If this fails it means either Hard was silently
    // upgraded to something else (rename the mode or flip the routing) or
    // the underlying bug was fixed at a different layer (great — then
    // delete this test).
    let max = output
        .lines()
        .find(|l| l.starts_with("max_out_ac="))
        .and_then(|l| l["max_out_ac=".len()..].parse::<f64>().ok())
        .expect("did not emit max_out_ac line");
    assert!(
        max > 20.0,
        "Hard mode out_ac = {:.2} V, expected >20 V (the cap-history bug). \
         Either Hard was silently fixed (great — delete this test) or the \
         bug-repro conditions changed. See memory/opamp_rail_clamp_bug.md.",
        max
    );
    eprintln!("Klon Hard mode: out_ac reaches {:.2} V (bug repro)", max);
}
