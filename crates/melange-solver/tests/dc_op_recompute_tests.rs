//! Tests for `CircuitState::recompute_dc_op()` (Oomox P6, Phase E).
//!
//! The MVP emits a runtime DC-operating-point solver behind the
//! `CodegenConfig::emit_dc_op_recompute` flag. These tests guard:
//!
//! - **Flag OFF (default)** — generated code is byte-identical to Phase D
//!   output. Protects every subsequent emission change from accidentally
//!   leaking into the default codegen path.
//! - **Flag ON** — `recompute_dc_op` is emitted, callable, and produces a
//!   value that matches the baked `DC_OP` constant at nominal pots.
//!
//! Skeleton-only assertions land alongside Phase E.2 (the no-op stub). The
//! jittered-match and `.switch`-change behavioral tests come in E7 once the
//! solver proper lands.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn generate_dk(spice: &str, emit_recompute: bool) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");
    let cfg = CodegenConfig {
        circuit_name: "dc_op_recompute_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![if kernel.n > 1 { 1 } else { 0 }],
        input_resistance: 1.0,
        emit_dc_op_recompute: emit_recompute,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(cfg)
        .generate(&kernel, &mna, &netlist)
        .expect("codegen")
        .code
}

const REGRESSION_NETLIST: &str = "\
Phase E regression guard — simple RC with named nodes
R1 in mid 10k
R2 mid out 10k
C1 out 0 100n
";

/// With the flag OFF, codegen output must not emit a `recompute_dc_op` method
/// definition. (The existing `dc_op()` doc comment from Phase C already
/// mentions the name in prose — that's fine; we guard on the `fn` signature.)
#[test]
fn flag_off_does_not_emit_recompute_dc_op() {
    let code = generate_dk(REGRESSION_NETLIST, false);
    assert!(
        !code.contains("fn recompute_dc_op"),
        "flag OFF must not emit `fn recompute_dc_op`; found in:\n{}",
        code.lines()
            .filter(|l| l.contains("fn recompute_dc_op"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// With the flag OFF, generated code is byte-identical to `CodegenConfig::default()`.
/// Guards against accidental emission changes sneaking into the default path.
#[test]
fn flag_off_byte_identical_to_default() {
    let default_code = generate_dk(REGRESSION_NETLIST, false);

    // Generate again with the default config (which should also have the
    // flag OFF). If the default ever flips, this test will fail loudly.
    let netlist = Netlist::parse(REGRESSION_NETLIST).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");
    let cfg = CodegenConfig {
        circuit_name: "dc_op_recompute_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![if kernel.n > 1 { 1 } else { 0 }],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let fresh = CodeGenerator::new(cfg)
        .generate(&kernel, &mna, &netlist)
        .expect("codegen")
        .code;
    assert_eq!(
        default_code, fresh,
        "flag-off output must be byte-identical to CodegenConfig::default()"
    );
}

/// `CodegenConfig::default()` keeps `emit_dc_op_recompute` at `false`.
/// Documented contract — plugins opt in, they don't get it for free.
#[test]
fn codegen_config_default_has_recompute_off() {
    let cfg = CodegenConfig::default();
    assert!(
        !cfg.emit_dc_op_recompute,
        "CodegenConfig default must leave emit_dc_op_recompute disabled"
    );
}

/// With the flag ON, codegen emits `pub fn recompute_dc_op(&mut self)` in
/// the `CircuitState` impl. Guards the emission pathway end-to-end.
#[test]
fn flag_on_emits_recompute_dc_op_signature() {
    let code = generate_dk(REGRESSION_NETLIST, true);
    assert!(
        code.contains("pub fn recompute_dc_op(&mut self)"),
        "flag ON must emit `pub fn recompute_dc_op(&mut self)`;\nrelevant lines:\n{}",
        code.lines()
            .filter(|l| l.contains("recompute_dc_op"))
            .collect::<Vec<_>>()
            .join("\n")
    );
    // Phase E.2 skeleton: the body is a documented no-op. Later phases will
    // replace the marker with actual NR-solve code — updating this assertion
    // then is a deliberate synchronization point.
    assert!(
        code.contains("Phase E MVP skeleton"),
        "flag ON should emit the Phase E skeleton comment"
    );
}

/// With the flag ON, the generated code compiles and the emitted method
/// is callable. Proves the flag plumbing end-to-end without requiring the
/// full NR solve to be implemented yet.
#[test]
fn flag_on_compiles_and_method_callable() {
    use std::io::Write;

    let code = generate_dk(REGRESSION_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        // Phase E skeleton: no-op, but must be callable without panic.\n\
        state.recompute_dc_op();\n\
        // DC OP accessor still returns a consistent value after the call.\n\
        let dc = state.dc_op();\n\
        assert_eq!(dc.len(), N);\n\
        println!(\"ok\");\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_flag_on.rs");
    let bin = tmp.join("melange_dc_op_recompute_flag_on");
    std::fs::File::create(&src)
        .unwrap()
        .write_all(full.as_bytes())
        .unwrap();
    let compile = std::process::Command::new("rustc")
        .args([
            src.to_str().unwrap(),
            "-o",
            bin.to_str().unwrap(),
            "--edition",
            "2021",
        ])
        .output()
        .expect("rustc");
    let _ = std::fs::remove_file(&src);
    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin);
        panic!(
            "compile failed:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }
    let run = std::process::Command::new(&bin).output().expect("run");
    let _ = std::fs::remove_file(&bin);
    assert!(
        run.status.success(),
        "binary failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&run.stdout),
        String::from_utf8_lossy(&run.stderr)
    );
    assert!(
        String::from_utf8_lossy(&run.stdout).contains("ok"),
        "unexpected output: {}",
        String::from_utf8_lossy(&run.stdout)
    );
}
