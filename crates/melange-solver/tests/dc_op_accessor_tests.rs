//! Tests for `CircuitState::dc_op()` and `dc_op_dump()` (Oomox P4).

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::io::Write;

fn generate_dk(spice: &str) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");
    let cfg = CodegenConfig {
        circuit_name: "dc_op_accessor_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![if kernel.n > 1 { 1 } else { 0 }],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(cfg)
        .generate(&kernel, &mna, &netlist)
        .expect("codegen")
        .code
}

#[test]
fn codegen_emits_dc_op_accessor() {
    let spice = "\
DC OP Accessor Test
R1 in out 10k
C1 out 0 100n
";
    let code = generate_dk(spice);
    assert!(
        code.contains("pub fn dc_op(&self) -> &[f64; N]"),
        "missing dc_op() accessor signature:\n{}",
        excerpt(&code, "dc_op")
    );
    assert!(
        code.contains("&self.dc_operating_point"),
        "dc_op() must return &self.dc_operating_point"
    );
}

#[test]
fn codegen_emits_dc_op_dump_when_nodes_named() {
    let spice = "\
DC OP Dump Test
R1 in mid 10k
R2 mid out 10k
C1 out 0 100n
";
    let code = generate_dk(spice);
    // Named nodes drive the dump emission.
    assert!(
        code.contains("pub fn dc_op_dump(&self)"),
        "missing dc_op_dump when nodes are named"
    );
    assert!(
        code.contains("V(IN)"),
        "dump should reference NODE_IN name"
    );
    assert!(
        code.contains("V(MID)"),
        "dump should reference NODE_MID name"
    );
    assert!(
        code.contains("V(OUT)"),
        "dump should reference NODE_OUT name"
    );
}

#[test]
fn dc_op_accessor_runtime_roundtrip() {
    // Generate, compile, run. Verify state.dc_op()[node] matches DC_OP[node]
    // at init, and that dc_op_dump() runs without panic.
    let spice = "\
DC OP Runtime Test
R1 in out 10k
C1 out 0 100n
";
    let code = generate_dk(spice);
    // Linear RC circuits don't run the DC OP solver (no nonlinear devices),
    // so `DC_OP` is not emitted as a const — assert the accessor returns the
    // same value that `v_prev` was initialized with, which is the authoritative
    // source for bias voltages on either path.
    let main = "\n\nfn main() {\n\
        let state = CircuitState::default();\n\
        let dc = state.dc_op();\n\
        assert_eq!(dc.len(), N);\n\
        for i in 0..N { assert_eq!(dc[i], state.v_prev[i]); }\n\
        state.dc_op_dump();\n\
        println!(\"ok\");\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_accessor.rs");
    let bin = tmp.join("melange_dc_op_accessor");
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
    // dc_op_dump() should have written to stderr in the form `V(NAME) = ... V`.
    let stderr = String::from_utf8_lossy(&run.stderr);
    assert!(
        stderr.contains("V(IN)") && stderr.contains("V(OUT)"),
        "dc_op_dump output missing expected V(NAME) lines:\n{}",
        stderr
    );
}

fn excerpt(code: &str, needle: &str) -> String {
    code.lines()
        .filter(|l| l.contains(needle))
        .collect::<Vec<_>>()
        .join("\n")
}
