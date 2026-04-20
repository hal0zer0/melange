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
fn parser_rejects_non_vs_or_r_target() {
    // `.runtime` dispatches on the first character of the target name:
    // V → voltage source, R → resistor. Everything else is rejected at
    // the dispatch stage, before argument parsing.
    let spice = "\
Runtime Non-VS Test
C1 in out 1u
.runtime C1 as foo
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("must start with V (voltage source) or R (resistor)"),
        "expected V/R dispatch error, got: {}",
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

// ===========================================================================
// `.runtime R Rname min max as field` — audio-rate resistor modulation
// ===========================================================================

// --- Parser -----------------------------------------------------------------

#[test]
fn parser_accepts_runtime_resistor_directive() {
    let spice = "\
Runtime R Test
R1 in out 10k
R2 bias 0 10k
.runtime R2 2k 12k as bias_r
";
    let netlist = Netlist::parse(spice).expect("parse");
    assert_eq!(netlist.runtime_resistors.len(), 1);
    let rr = &netlist.runtime_resistors[0];
    assert_eq!(rr.resistor_name, "R2");
    assert!((rr.min_value - 2e3).abs() < 1.0);
    assert!((rr.max_value - 12e3).abs() < 1.0);
    assert_eq!(rr.field_name, "bias_r");
}

#[test]
fn parser_rejects_runtime_r_min_ge_max() {
    let spice = "\
Runtime R Bad Range
R1 in out 10k
R2 bias 0 10k
.runtime R2 12k 2k as bias_r
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("min") && format!("{}", err).contains("less than max"),
        "expected min<max error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_runtime_r_missing_resistor() {
    let spice = "\
Runtime R Ghost
R1 in out 10k
.runtime Rghost 2k 12k as bias_r
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("resistor 'Rghost'"),
        "expected unknown-resistor error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_runtime_r_duplicate_resistor() {
    let spice = "\
Runtime R Dup
R1 in out 10k
R2 bias 0 10k
.runtime R2 2k 12k as a
.runtime R2 1k 20k as b
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("more than once"),
        "expected duplicate-resistor error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_runtime_r_collides_with_pot() {
    let spice = "\
Runtime R vs Pot
R1 in out 10k
R2 bias 0 10k
.pot R2 1k 100k
.runtime R2 2k 12k as bias_r
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("already claimed"),
        "expected already-claimed error, got: {}",
        err
    );
}

#[test]
fn parser_rejects_gang_member_that_is_runtime_r() {
    // `.gang` is a UI construct; `.runtime R` intentionally has no knob.
    // Mixing them would be nonsensical, so reject with a pointer to the
    // right pattern (drive multiple setters from one plugin-side envelope).
    let spice = "\
Gang Over Runtime R
R1 in out 10k
R2 a 0 10k
R3 b 0 10k
.runtime R2 2k 12k as bias_a
.runtime R3 2k 12k as bias_b
.gang \"Bias\" R2 R3
";
    let err = Netlist::parse(spice).unwrap_err();
    let msg = format!("{}", err);
    assert!(
        msg.contains(".runtime R target") && msg.contains("plugin-side envelope"),
        "expected runtime-R gang rejection with guidance, got: {}",
        err
    );
}

#[test]
fn parser_rejects_runtime_r_field_collision_with_vs() {
    // Shared field namespace between .runtime V and .runtime R — a field
    // name bound to one cannot be rebound to the other.
    let spice = "\
Runtime Shared Field
R1 in out 10k
R2 bias 0 10k
Vctrl ctrl 0 DC 0
.runtime Vctrl as foo
.runtime R2 2k 12k as foo
";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        format!("{}", err).contains("field name 'foo'"),
        "expected duplicate-field error, got: {}",
        err
    );
}

// --- Codegen: DK path -------------------------------------------------------

#[test]
fn codegen_emits_runtime_r_setter_without_dc_op_snap() {
    // Basic bias circuit: Vin → R1 → node_out, node_out → Rbias → 0.
    // Rbias becomes a `.runtime R` target so the generated code must emit
    // `set_runtime_R_bias_r` WITHOUT the 20% DC-OP warm re-init block.
    let spice = "\
Runtime R DK
R1 in out 1k
Rbias out 0 10k
.runtime Rbias 2k 12k as bias_r
";
    let code = generate_dk(spice);

    assert!(
        code.contains("pub fn set_runtime_R_bias_r("),
        "missing runtime-R setter:\n{}",
        excerpt(&code, "runtime_R_bias_r")
    );
    assert!(
        !code.contains("set_pot_0("),
        "runtime-R should not emit set_pot_0:\n{}",
        excerpt(&code, "set_pot_")
    );
    // Extract the set_runtime_R_bias_r function body and verify it does NOT
    // contain the warm DC-OP re-init sentinel string.
    let setter_start = code
        .find("pub fn set_runtime_R_bias_r(")
        .expect("setter present");
    let setter_end = setter_start
        + code[setter_start..]
            .find("\n    }\n")
            .expect("setter end brace");
    let setter_body = &code[setter_start..setter_end];
    assert!(
        !setter_body.contains("rel_delta"),
        "runtime-R setter must NOT emit the warm DC-OP re-init block; got:\n{}",
        setter_body
    );
    assert!(
        !setter_body.contains("DC_OP"),
        "runtime-R setter must NOT touch DC_OP; got:\n{}",
        setter_body
    );
    assert!(
        setter_body.contains("clamp(RUNTIME_R_BIAS_R_MIN, RUNTIME_R_BIAS_R_MAX)"),
        "runtime-R setter must clamp to the declared range; got:\n{}",
        setter_body
    );
    assert!(
        setter_body.contains("self.matrices_dirty = true"),
        "runtime-R setter must mark matrices dirty; got:\n{}",
        setter_body
    );

    // Public alias consts are present.
    assert!(code.contains("pub const RUNTIME_R_BIAS_R_MIN"));
    assert!(code.contains("pub const RUNTIME_R_BIAS_R_MAX"));
    assert!(code.contains("pub const RUNTIME_R_BIAS_R_NOMINAL"));

    // Read-only accessor method.
    assert!(
        code.contains("pub fn bias_r(&self) -> f64"),
        "missing read accessor for runtime-R field:\n{}",
        excerpt(&code, "bias_r(")
    );
}

#[test]
fn codegen_pot_setter_has_no_reseed_block() {
    // `.pot` and `.runtime R` setters now share the same reseed-free shape.
    // The historical warm DC-OP gate (Batch D Phase 2) was removed because
    // it produced audible clicks on per-block log-taper sweeps; explicit
    // `recompute_dc_op()` is the supported reseed mechanism for preset
    // recall or unsmoothed jumps.
    let spice = "\
Pot Reseed Regression
R1 in out 1k
Rknob out 0 10k
.pot Rknob 1k 100k
";
    let code = generate_dk(spice);
    let setter_start = code
        .find("pub fn set_pot_0(")
        .expect("pot setter present");
    let setter_end = setter_start
        + code[setter_start..]
            .find("\n    }\n")
            .expect("setter end brace");
    let setter_body = &code[setter_start..setter_end];
    assert!(
        !setter_body.contains("rel_delta"),
        "pot setter must NOT emit the warm DC-OP gate (stripped 2026-04-20); got:\n{}",
        setter_body
    );
    assert!(
        !setter_body.contains("DC_OP"),
        "pot setter must NOT touch DC_OP; got:\n{}",
        setter_body
    );
}

// --- Codegen: nodal path ----------------------------------------------------

#[test]
fn nodal_codegen_emits_runtime_r_setter_without_dc_op_snap() {
    // Force the nodal path via a cap-only node arrangement that triggers
    // the "S ill-conditioned" routing check; verify the nodal emitter also
    // respects the runtime_field contract.
    use melange_solver::codegen::CodegenConfig;
    use melange_solver::parser::Netlist;
    let spice = "\
Runtime R Nodal
R1 in n1 1k
C1 n1 out 100n
Rbias out 0 10k
.runtime Rbias 2k 12k as bias_r
";
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).expect("mna");
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let cfg = CodegenConfig {
        circuit_name: "runtime_r_nodal".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![2],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let code = melange_solver::codegen::CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .expect("codegen")
        .code;

    assert!(
        code.contains("pub fn set_runtime_R_bias_r("),
        "nodal path missing runtime-R setter:\n{}",
        excerpt(&code, "runtime_R_bias_r")
    );
    assert!(
        !code.contains("pub fn set_pot_0("),
        "nodal path should not emit set_pot_0 for a runtime-R:\n{}",
        excerpt(&code, "set_pot_")
    );
    let setter_start = code
        .find("pub fn set_runtime_R_bias_r(")
        .expect("setter present");
    let setter_end = setter_start
        + code[setter_start..]
            .find("\n    }\n")
            .expect("setter end brace");
    let setter_body = &code[setter_start..setter_end];
    assert!(
        !setter_body.contains("rel_delta"),
        "nodal runtime-R setter must NOT emit the DC-OP gate; got:\n{}",
        setter_body
    );
    assert!(
        setter_body.contains("self.matrices_dirty = true"),
        "nodal runtime-R setter must mark matrices dirty; got:\n{}",
        setter_body
    );
}

// --- Compile-and-run: sweep R across ±20% on a bias resistor ----------------

#[test]
fn runtime_r_sweep_no_nan_and_no_interstate_discontinuity() {
    // Validation path from the oomox handoff: 1 kHz input, 5 ms sweep of
    // Rbias across the full declared ±20% window. Assert no NaN / Inf,
    // inter-sample jumps bounded.
    //
    // Topology: input → R1 → out, out → Rbias → 0. Rbias is `.runtime R`.
    // For a pure divider out = Vin * Rbias/(R1+Rbias), so at Rbias=2k the
    // divider is 0.667 and at Rbias=12k it is 0.923 — a smooth sweep must
    // not produce any sample-to-sample jump larger than the difference
    // across ~10 samples of a 1 kHz sine.
    let spice = "\
Runtime R Sweep
R1 in out 1k
Rbias out 0 10k
.runtime Rbias 2k 12k as bias_r
";
    let code = generate_dk(spice);

    // Sweep 2k↔12k over 220 samples (≈ 5 ms at 44.1 kHz) while feeding a
    // 1 kHz sine. Record the peak inter-sample jump seen.
    let main = "\n\nfn main() {\n\
        use std::f64::consts::PI;\n\
        let mut s = CircuitState::default();\n\
        let mut prev = 0.0f64;\n\
        let mut max_jump = 0.0f64;\n\
        let mut any_nan = false;\n\
        let sr = 44100.0f64;\n\
        // Warm-up: fixed R, input ramp-up.\n\
        for i in 0..1000 {\n\
            let t = i as f64 / sr;\n\
            let v = 0.3 * (2.0 * PI * 1000.0 * t).sin();\n\
            let y = process_sample(v, &mut s)[0];\n\
            if !y.is_finite() { any_nan = true; }\n\
            prev = y;\n\
        }\n\
        // Sweep phase: drive setter every sample across the full window.\n\
        let sweep_len = 220;\n\
        for i in 0..sweep_len {\n\
            let t = (i + 1000) as f64 / sr;\n\
            let v = 0.3 * (2.0 * PI * 1000.0 * t).sin();\n\
            // Triangle sweep across [2k, 12k].\n\
            let frac = (i as f64 / sweep_len as f64) * 2.0 - 1.0;\n\
            let r = 7000.0 + 5000.0 * frac.abs() - 5000.0 * (1.0 - frac.abs());\n\
            s.set_runtime_R_bias_r(r);\n\
            let y = process_sample(v, &mut s)[0];\n\
            if !y.is_finite() { any_nan = true; }\n\
            let jump = (y - prev).abs();\n\
            if jump > max_jump { max_jump = jump; }\n\
            prev = y;\n\
        }\n\
        println!(\"nan={} max_jump={:.9}\", any_nan, max_jump);\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_runtime_r_sweep.rs");
    let bin = tmp.join("melange_runtime_r_sweep");
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

    assert!(
        stdout.contains("nan=false"),
        "runtime-R sweep produced NaN/Inf: {}",
        stdout
    );
    // Max jump bound: at 1 kHz, 44.1 kHz sample rate, a pure 0.3 V sine has
    // per-sample step of at most 0.3 * 2π*1000/44100 ≈ 0.0428 V. Sweeping R
    // across the full ±20% window adds some extra jump from the divider
    // moving, but the total should stay under ~0.05 V. If the DC-OP snap
    // were firing (the bug this feature fixes), jumps would hit ~1 V or more.
    let max_jump: f64 = stdout
        .split_whitespace()
        .find(|t| t.starts_with("max_jump="))
        .expect("max_jump in stdout")
        .split_once('=')
        .unwrap()
        .1
        .parse()
        .unwrap();
    assert!(
        max_jump < 0.1,
        "inter-sample jump {:.4} V too large — DC-OP snap may be firing; stdout: {}",
        max_jump,
        stdout
    );
}
