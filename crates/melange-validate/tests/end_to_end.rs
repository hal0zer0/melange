//! Minimal end-to-end test - Parse netlist → Build solver → Process samples
//!
//! This test verifies the basic workflow works without requiring ngspice.
//! Tests use the codegen compile-and-run pipeline.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::io::Write;

/// Simple RC lowpass test circuit.
const RC_LOWPASS_SPICE: &str = "RC Circuit\nR1 in out 1k\nC1 out 0 1u\n";

/// Diode clipper test circuit (with cap to prevent singular matrix)
const DIODE_CLIPPER_SPICE: &str = "\
Diode Clipper
Rin in out 1k
D1 out 0 D1N4148
C1 out 0 1u
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// Compile and run generated circuit code with a custom main.
fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let id = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let src = tmp_dir.join(format!("melange_e2e_{tag}_{pid}_{id}.rs"));
    let bin = tmp_dir.join(format!("melange_e2e_{tag}_{pid}_{id}"));

    let full = format!("{code}\n\n{main_code}\n");
    { let mut f = std::fs::File::create(&src).unwrap(); f.write_all(full.as_bytes()).unwrap(); }

    let c = std::process::Command::new("rustc")
        .arg(&src).arg("-o").arg(&bin).arg("--edition=2024").arg("-O")
        .output().expect("rustc");
    let _ = std::fs::remove_file(&src);
    if !c.status.success() {
        let _ = std::fs::remove_file(&bin);
        panic!("Compile failed for {tag}:\n{}", String::from_utf8_lossy(&c.stderr));
    }

    let r = std::process::Command::new(&bin).output().expect("run");
    let _ = std::fs::remove_file(&bin);
    if !r.status.success() {
        panic!("Run failed for {tag}:\n{}", String::from_utf8_lossy(&r.stderr));
    }
    String::from_utf8_lossy(&r.stdout).to_string()
}

fn generate_code(spice: &str, sample_rate: f64) -> String {
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let in_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    let out_node = mna.node_map.get("out").copied().unwrap_or(2).saturating_sub(1);
    mna.g[in_node][in_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, sample_rate).unwrap();
    let config = CodegenConfig {
        circuit_name: "e2e".to_string(),
        sample_rate,
        input_node: in_node,
        output_nodes: vec![out_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config).generate(&kernel, &mna, &netlist).unwrap().code
}

// =========================================================================
// Parser / MNA / kernel tests (no solver needed)
// =========================================================================

#[test]
fn test_parse_rc_lowpass() {
    let netlist = Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    assert_eq!(netlist.elements.len(), 2, "Should parse 2 elements");
    let has_r1 = netlist.elements.iter().any(|e| {
        matches!(e, melange_solver::parser::Element::Resistor { name, .. } if name == "R1")
    });
    assert!(has_r1, "Should have R1 resistor");
}

#[test]
fn test_build_mna_rc_lowpass() {
    let netlist = Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(mna.n >= 1, "Should have at least 1 non-ground node");
}

#[test]
fn test_create_dk_kernel_rc() {
    let netlist = Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    assert_eq!(kernel.m, 0, "RC circuit should have no nonlinear ports");
    assert!(kernel.n >= 1, "Should have at least 1 node");
}

#[test]
fn test_diode_clipper_has_nonlinearity() {
    let netlist = Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    assert_eq!(kernel.m, 1, "Diode clipper should have 1 nonlinear port");
}

// =========================================================================
// End-to-end codegen tests
// =========================================================================

/// End-to-end RC lowpass: verify output converges with correct dynamics.
#[test]
fn test_full_solver_rc_lowpass() {
    let code = generate_code(RC_LOWPASS_SPICE, 44100.0);
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    for _ in 0..500 {
        let out = process_sample(1.0, &mut state);
        println!("{:.15e}", out[0]);
    }
}
"#;
    let stdout = compile_and_run(&code, main_code, "rc_e2e");
    let output: Vec<f64> = stdout.lines().filter_map(|l| l.trim().parse().ok()).collect();
    assert_eq!(output.len(), 500);

    // Peak should exceed 0.8V
    let peak = output.iter().cloned().fold(0.0f64, f64::max);
    assert!(peak > 0.8, "RC lowpass peak should exceed 0.8V, got {:.4}V", peak);

    // Monotonically increasing in first 100 samples
    for i in 1..100 {
        assert!(
            output[i] >= output[i - 1] - 1e-10,
            "Monotonic: output[{}]={:.6} < output[{}]={:.6}", i, output[i], i - 1, output[i - 1]
        );
    }

    assert!(output[0] > 0.0, "First sample should be positive");
}

/// End-to-end diode clipper: verify clipping behavior.
#[test]
fn test_diode_clipper_solver() {
    let code = generate_code(DIODE_CLIPPER_SPICE, 44100.0);
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    // Large signal: 200 samples at 2V
    for _ in 0..200 {
        let out = process_sample(2.0, &mut state);
        println!("{:.15e}", out[0]);
    }
}
"#;
    let stdout = compile_and_run(&code, main_code, "diode_e2e");
    let output: Vec<f64> = stdout.lines().filter_map(|l| l.trim().parse().ok()).collect();
    assert_eq!(output.len(), 200);

    for &v in &output {
        assert!(v.is_finite(), "Output should be finite");
    }

    let final_out = *output.last().unwrap();
    assert!(final_out > 0.0, "Output should be positive for positive input");
    assert!(final_out < 1.5, "Diode clipper should limit output below ~1V, got {:.4}V", final_out);
}
