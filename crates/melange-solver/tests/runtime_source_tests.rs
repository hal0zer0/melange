//! Tests for the `.runtime Vname as field_name` directive (Oomox P1).
//!
//! Parser validation + codegen emission + full compile-and-run roundtrip
//! verifying that `state.<field>` stamps into the correct RHS row.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::io::Write;

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

#[test]
fn parser_accepts_runtime_directive() {
    let spice = "\
Runtime Test
R1 in out 10k
Vctrl ctrl 0 DC 0
R2 ctrl 0 100k
.runtime Vctrl as ctrl_voltage
";
    let netlist = Netlist::parse(spice).expect("parse");
    assert_eq!(netlist.runtime_sources.len(), 1);
    assert_eq!(netlist.runtime_sources[0].vs_name, "Vctrl");
    assert_eq!(netlist.runtime_sources[0].field_name, "ctrl_voltage");
}

#[test]
fn parser_rejects_missing_as_keyword() {
    // Well-formed arity (4 tokens) but wrong separator — must be `as`.
    let spice = "\
Runtime No-As Test
R1 in out 10k
Vctrl ctrl 0 DC 0
.runtime Vctrl to ctrl_voltage
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("expects 'as'"),
        "expected 'as' keyword error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_non_voltage_source() {
    let spice = "\
Runtime Non-VS Test
R1 in out 10k
.runtime R1 as foo
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("must be a voltage source"),
        "expected VS-name error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_invalid_field_name() {
    let spice = "\
Runtime Bad-Ident Test
R1 in out 10k
Vctrl ctrl 0 DC 0
.runtime Vctrl as 1bad
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("not a valid Rust identifier"),
        "expected identifier validation error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_unknown_voltage_source() {
    let spice = "\
Runtime Unknown VS
R1 in out 10k
Vreal vcc 0 DC 9
.runtime Vghost as foo
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("voltage source 'Vghost'"),
        "expected unknown-VS error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_duplicate_source_binding() {
    let spice = "\
Runtime Dup VS
R1 in out 10k
Vctrl ctrl 0 DC 0
.runtime Vctrl as a
.runtime Vctrl as b
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("more than once"),
        "expected duplicate-VS error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_duplicate_field_name() {
    let spice = "\
Runtime Dup Field
R1 in out 10k
Va a 0 DC 0
Vb b 0 DC 0
.runtime Va as ctrl
.runtime Vb as ctrl
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("field name 'ctrl'"),
        "expected duplicate-field error, got: {}",
        err
    );
}

// ---------------------------------------------------------------------------
// Codegen emission
// ---------------------------------------------------------------------------

fn generate_dk(spice: &str) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    // Stamp input conductance the same way the CLI does.
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");
    let cfg = CodegenConfig {
        circuit_name: "runtime_test".to_string(),
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
fn codegen_emits_runtime_field_on_state() {
    let spice = "\
Runtime Codegen
R1 in out 10k
Vctrl ctrl 0 DC 0
R2 ctrl 0 100k
.runtime Vctrl as ctrl_voltage
";
    let code = generate_dk(spice);
    assert!(
        code.contains("pub ctrl_voltage: f64"),
        "missing runtime field on CircuitState:\n{}",
        excerpt(&code, "ctrl_voltage")
    );
    assert!(
        code.contains("ctrl_voltage: 0.0"),
        "runtime field not initialized to 0:\n{}",
        excerpt(&code, "ctrl_voltage")
    );
    assert!(
        code.contains("self.ctrl_voltage = 0.0"),
        "runtime field not zeroed in reset():\n{}",
        excerpt(&code, "ctrl_voltage")
    );
}

#[test]
fn codegen_stamps_runtime_field_into_rhs() {
    let spice = "\
Runtime RHS Stamp
R1 in out 10k
Vctrl ctrl 0 DC 0
R2 ctrl 0 100k
.runtime Vctrl as ctrl_voltage
";
    let code = generate_dk(spice);
    // build_rhs must stamp `rhs[row] += state.ctrl_voltage`.
    assert!(
        code.contains("+= state.ctrl_voltage;"),
        "missing rhs stamp for runtime field:\n{}",
        excerpt(&code, "ctrl_voltage")
    );
}

// ---------------------------------------------------------------------------
// Compile-and-run: verify the runtime field actually drives the circuit
// ---------------------------------------------------------------------------

#[test]
fn runtime_field_drives_circuit_output() {
    // Circuit: Vin→R1→out, plus a runtime voltage source Vctrl feeding `out`
    // through R2. With Vctrl.dc = 0 and state.ctrl_voltage varied, the
    // steady-state voltage at `out` should track ctrl_voltage (scaled by the
    // resistive divider).
    let spice = "\
Runtime Drive Test
R1 in out 1k
R2 out ctrl 1k
Vctrl ctrl 0 DC 0
.runtime Vctrl as ctrl_voltage
";
    let code = generate_dk(spice);

    // Three scenarios: ctrl=0, ctrl=1V, ctrl=-1V. With R1=R2=1k and input=0,
    // out is just a resistor divider between ctrl and ground through R1 so
    // out ≈ ctrl_voltage / 2. Small-tolerance check — values settle within
    // a handful of samples because there are no caps.
    // `process_sample` returns `[f64; NUM_OUTPUTS]` in the generated code, so
    // we index [0] to read the single configured output channel.
    let main = "\n\nfn main() {\n\
        let mut s = CircuitState::default();\n\
        s.ctrl_voltage = 0.0;\n\
        for _ in 0..400 { let _ = process_sample(0.0, &mut s); }\n\
        let out_zero = process_sample(0.0, &mut s)[0];\n\
        s.ctrl_voltage = 1.0;\n\
        for _ in 0..400 { let _ = process_sample(0.0, &mut s); }\n\
        let out_pos = process_sample(0.0, &mut s)[0];\n\
        s.ctrl_voltage = -1.0;\n\
        for _ in 0..400 { let _ = process_sample(0.0, &mut s); }\n\
        let out_neg = process_sample(0.0, &mut s)[0];\n\
        println!(\"zero={} pos={} neg={}\", out_zero, out_pos, out_neg);\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_runtime_drive.rs");
    let bin = tmp.join("melange_runtime_drive");
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
            "generated code failed to compile:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin).output().expect("run");
    let _ = std::fs::remove_file(&bin);
    assert!(
        run.status.success(),
        "binary failed:\nstderr: {}",
        String::from_utf8_lossy(&run.stderr)
    );
    let stdout = String::from_utf8_lossy(&run.stdout);

    // Parse out the three values.
    let parse = |k: &str| -> f64 {
        stdout
            .split_whitespace()
            .find(|t| t.starts_with(&format!("{}=", k)))
            .unwrap_or_else(|| panic!("missing {k} in output: {stdout}"))
            .split_once('=')
            .unwrap()
            .1
            .parse::<f64>()
            .unwrap_or_else(|_| panic!("bad number for {k}: {stdout}"))
    };
    let out_zero = parse("zero");
    let out_pos = parse("pos");
    let out_neg = parse("neg");

    // With ctrl=0 the output should sit near 0 (modulo DC-blocker settling).
    assert!(
        out_zero.abs() < 0.1,
        "out at ctrl=0 should be ~0, got {}",
        out_zero
    );
    // Sign and magnitude should track ctrl_voltage. Exact DC gain is shaped by
    // the 5 Hz DC blocker (enabled by default), so we only assert sign and
    // order of magnitude — not the closed-form resistive-divider value.
    assert!(
        out_pos > 0.1,
        "out should be positive when ctrl=+1V, got {}",
        out_pos
    );
    assert!(
        out_neg < -0.1,
        "out should be negative when ctrl=-1V, got {}",
        out_neg
    );
}

fn excerpt(code: &str, needle: &str) -> String {
    code.lines()
        .filter(|l| l.contains(needle))
        .collect::<Vec<_>>()
        .join("\n")
}
