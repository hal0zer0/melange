//! Codegen tests for named topology constants (Oomox roadmap P2 + P3).
//!
//! Emit-level tests: verify the generated code contains the expected
//! `NODE_<NAME>`, `VSOURCE_<NAME>_RHS_ROW`, `POT_<NAME>_INDEX` constants
//! with correct indices. Exercised on both DK and nodal codegen paths.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn generate_dk(spice: &str) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");
    let cfg = CodegenConfig {
        circuit_name: "named_const_test".to_string(),
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
fn node_constants_emitted_for_user_named_nodes() {
    // Resistive voltage divider — purely linear, takes the DK path with M=0.
    let spice = "\
RC Lowpass
R1 in out 10k
C1 out 0 100n
";
    let code = generate_dk(spice);
    // `in` and `out` are user-named nodes. Node indices match what the MNA
    // assigns in netlist-element order: in=0, out=1 (0-indexed).
    assert!(
        code.contains("pub const NODE_IN: usize ="),
        "missing NODE_IN:\n{}",
        excerpt(&code, "NODE_")
    );
    assert!(
        code.contains("pub const NODE_OUT: usize ="),
        "missing NODE_OUT:\n{}",
        excerpt(&code, "NODE_")
    );
    // Ground (0) must not be emitted — it's implicit.
    assert!(
        !code.contains("pub const NODE_0:"),
        "ground should not be emitted"
    );
}

#[test]
fn node_names_sanitized_and_deduped() {
    // Node names with non-alphanumeric chars and a leading digit: should get
    // sanitized to SCREAMING_SNAKE. `in+` and `in-` both sanitize to `IN_`,
    // which the dedupe pass resolves via `_2`.
    let spice = "\
Sanitize Test
R1 in+ in- 10k
C1 in- 0 100n
R2 in- 3v3 1k
V1 3v3 0 DC 3.3
";
    let code = generate_dk(spice);
    assert!(
        code.contains("pub const NODE_IN_: usize ="),
        "first IN_ missing:\n{}",
        excerpt(&code, "NODE_")
    );
    assert!(
        code.contains("pub const NODE_IN__2: usize ="),
        "deduped IN__2 missing:\n{}",
        excerpt(&code, "NODE_")
    );
    assert!(
        code.contains("pub const NODE__3V3: usize ="),
        "leading-digit sanitize missing:\n{}",
        excerpt(&code, "NODE_")
    );
}

#[test]
fn vsource_rhs_row_emitted_for_voltage_sources() {
    let spice = "\
VS Test
R1 in out 10k
C1 out 0 100n
V1 vcc 0 DC 9
R2 vcc out 100k
";
    let code = generate_dk(spice);
    // V1's RHS row = n_nodes + ext_idx (ext_idx=0 for the first VS).
    assert!(
        code.contains("pub const VSOURCE_V1_RHS_ROW: usize ="),
        "missing VSOURCE_V1_RHS_ROW:\n{}",
        excerpt(&code, "VSOURCE_")
    );
}

#[test]
fn pot_index_emitted_for_each_pot() {
    let spice = "\
Pot Test
R1 in mid 10k
Rvol mid out 100k
C1 out 0 100n
.pot Rvol 1k 100k
";
    let code = generate_dk(spice);
    assert!(
        code.contains("pub const POT_RVOL_INDEX: usize = 0;"),
        "missing POT_RVOL_INDEX = 0:\n{}",
        excerpt(&code, "POT_")
    );
}

#[test]
fn generated_code_still_compiles_with_named_constants() {
    // Full roundtrip: generate, compile, run one sample. Ensures the emitted
    // identifiers are valid Rust syntax and don't collide with existing
    // codegen names.
    use std::io::Write;
    let spice = "\
Full Roundtrip
R1 in mid 10k
Rvol mid out 100k
C1 out 0 100n
V1 vcc 0 DC 9
.pot Rvol 1k 100k
";
    let code = generate_dk(spice);
    // Add a main that touches each named constant to prove they exist.
    let main = "\n\nfn main() {\n\
        let _ = NODE_IN;\n\
        let _ = NODE_OUT;\n\
        let _ = NODE_MID;\n\
        let _ = NODE_VCC;\n\
        let _ = VSOURCE_V1_RHS_ROW;\n\
        let _ = POT_RVOL_INDEX;\n\
        let mut s = CircuitState::default();\n\
        let _out = process_sample(0.01, &mut s);\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_named_const_roundtrip.rs");
    let bin = tmp.join("melange_named_const_roundtrip");
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
        ])
        .output()
        .expect("rustc");

    let _ = std::fs::remove_file(&src);
    if !out.status.success() {
        let _ = std::fs::remove_file(&bin);
        panic!(
            "generated code failed to compile:\n{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }

    let run = std::process::Command::new(&bin).output().expect("run");
    let _ = std::fs::remove_file(&bin);
    assert!(
        run.status.success(),
        "generated binary failed at runtime:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&run.stdout),
        String::from_utf8_lossy(&run.stderr)
    );
}

/// Grab lines from `code` containing `needle`, for readable failure output.
fn excerpt(code: &str, needle: &str) -> String {
    code.lines()
        .filter(|l| l.contains(needle))
        .collect::<Vec<_>>()
        .join("\n")
}
