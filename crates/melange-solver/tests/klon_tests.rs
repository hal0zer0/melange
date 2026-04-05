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
fn test_klon_dk_kernel_fails_positive_k() {
    // The Klon's shunt diodes to VB+ (a voltage source) produce positive K diagonal.
    // This is expected — the circuit routes to nodal solver automatically.
    let spice = load_klon_netlist();
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = mna.node_map["in"] - 1;
    mna.stamp_input_conductance(input_node, 1.0);

    let result = DkKernel::from_mna(&mna, 48000.0);
    assert!(
        result.is_err(),
        "DK kernel should fail for Klon (positive K diagonal from shunt diodes to VS)"
    );
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
