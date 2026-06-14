//! End-to-end tests for behavioral (`B`) sources: generate nodal code, compile
//! it with `rustc`, run it, and compare to the analytic oracle.
//!
//! Covers the algebraic `I={}` class (current source over node voltages). The
//! `V={}` constraint form, `ddt`/`idt`, and named parameters are guarded off in
//! codegen until wired — see `docs/aidocs/BEHAVIORAL_SOURCES.md`.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::io::Write;

/// Generate nodal code with the input conductance stamped at `input_node`.
fn generate_nodal(spice: &str, input_node: usize, output_node: usize) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    // Mirror the CLI: stamp the 1Ω input Thevenin conductance.
    mna.g[input_node][input_node] += 1.0;
    let cfg = CodegenConfig {
        circuit_name: "bsrc_test".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        output_scales: vec![1.0],
        input_resistance: 1.0,
        // These oracles produce DC outputs; the default output DC-blocker would
        // high-pass them away.
        dc_block: false,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .expect("codegen")
        .code
}

/// Compile `code` (with an appended `main`), run it, return stdout.
fn compile_and_run(code: &str, main: &str, label: &str) -> String {
    let full = format!("{code}\n\n{main}");
    let tmp = std::env::temp_dir();
    let src = tmp.join(format!("melange_bsrc_{label}.rs"));
    let bin = tmp.join(format!("melange_bsrc_{label}"));
    std::fs::File::create(&src)
        .unwrap()
        .write_all(full.as_bytes())
        .unwrap();

    let out = std::process::Command::new("rustc")
        .args([
            src.to_str().unwrap(),
            "-o",
            bin.to_str().unwrap(),
            "--edition",
            "2021",
            "-A",
            "warnings",
        ])
        .output()
        .expect("rustc");
    let _ = std::fs::remove_file(&src);
    if !out.status.success() {
        let _ = std::fs::remove_file(&bin);
        panic!(
            "[{label}] generated code failed to compile:\n{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
    let run = std::process::Command::new(&bin).output().expect("run");
    let _ = std::fs::remove_file(&bin);
    assert!(
        run.status.success(),
        "[{label}] generated binary failed at runtime:\n{}",
        String::from_utf8_lossy(&run.stderr)
    );
    String::from_utf8_lossy(&run.stdout).trim().to_string()
}

/// Run the generated `process_sample` for `n` samples at zero input and print
/// the final output. The behavioral source is driven by stiff DC sources, so
/// the output settles to its steady value within a few NR-converged samples.
fn settle_main(n: usize) -> String {
    format!(
        "fn main() {{\n\
        \x20   let mut s = CircuitState::default();\n\
        \x20   let mut y = [0.0f64; NUM_OUTPUTS];\n\
        \x20   for _ in 0..{n} {{ y = process_sample(0.0, &mut s); }}\n\
        \x20   println!(\"{{:.9}}\", y[0]);\n\
        }}\n"
    )
}

#[test]
fn behavioral_current_multiplier_matches_oracle() {
    // B1 sources I = V(a)*V(b) from `out` to ground through a 1Ω resistor.
    // KCL at out: V(out)/1 + I_source = 0  ⇒  V(out) = -(V(a)*V(b)).
    let spice = "\
Behavioral multiplier
Va a 0 DC 0.6
Vb b 0 DC 0.4
B1 out 0 I={ V(a)*V(b) }
Rout out 0 1
Cout out 0 1u
";
    // Nodes (0-based): a=0, b=1, out=2.
    let code = generate_nodal(spice, 0, 2);
    let out = compile_and_run(&code, &settle_main(64), "multiplier");
    let v_out: f64 = out.parse().unwrap_or_else(|_| panic!("bad output: {out:?}"));
    let expected = -(0.6 * 0.4);
    assert!(
        (v_out - expected).abs() < 1e-5,
        "multiplier: got {v_out}, expected {expected}"
    );
}

#[test]
fn behavioral_current_tanh_clipper_matches_oracle() {
    // I = tanh(V(a)) ⇒ V(out) = -tanh(0.6).
    let spice = "\
Behavioral tanh clipper
Va a 0 DC 0.6
B1 out 0 I={ tanh(V(a)) }
Rout out 0 1
Cout out 0 1u
";
    // Nodes (0-based): a=0, out=1.
    let code = generate_nodal(spice, 0, 1);
    let out = compile_and_run(&code, &settle_main(64), "tanh");
    let v_out: f64 = out.parse().unwrap_or_else(|_| panic!("bad output: {out:?}"));
    let expected = -(0.6f64.tanh());
    assert!(
        (v_out - expected).abs() < 1e-5,
        "tanh clipper: got {v_out}, expected {expected}"
    );
}

#[test]
fn v_form_is_rejected_until_wired() {
    // V={} must error cleanly (not silently drop) until the augmented-row path
    // lands.
    let spice = "\
Behavioral voltage source
Va a 0 DC 0.5
B1 out 0 V={ tanh(V(a)) }
Rout out 0 1
";
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    mna.g[0][0] += 1.0;
    let cfg = CodegenConfig {
        circuit_name: "vform".to_string(),
        sample_rate: 48000.0,
        input_node: 0,
        output_nodes: vec![1],
        output_scales: vec![1.0],
        ..CodegenConfig::default()
    };
    let err = CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .unwrap_err();
    let msg = format!("{err}");
    assert!(msg.contains("V={expr}") || msg.to_lowercase().contains("voltage"), "unexpected: {msg}");
}
