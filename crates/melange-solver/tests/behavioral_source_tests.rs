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
fn behavioral_ddt_of_time_is_unity() {
    // ddt(time) = (t - t_prev)/dt = 1 every sample after the first.
    // I = ddt(time) into 1Ω ⇒ V(out) = -1.
    let spice = "\
Behavioral ddt(time)
B1 out 0 I={ ddt(time) }
Rout out 0 1
Cout out 0 1u
Rin in 0 1meg
";
    // Nodes (0-based): out=0, in=1. Drive input on the isolated `in` node.
    let code = generate_nodal(spice, 1, 0);
    let out = compile_and_run(&code, &settle_main(64), "ddt_time");
    let v_out: f64 = out.parse().unwrap_or_else(|_| panic!("bad output: {out:?}"));
    assert!(
        (v_out - (-1.0)).abs() < 1e-4,
        "ddt(time): got {v_out}, expected -1.0"
    );
}

#[test]
fn behavioral_idt_of_constant_is_a_ramp() {
    // idt(V(a)) with V(a)=1 accumulates ≈ N·dt. I = idt(1) into 1Ω ⇒
    // V(out) ≈ -N·dt. At 48 kHz over 4800 samples: ≈ -0.1.
    let spice = "\
Behavioral idt(const)
Va a 0 DC 1
B1 out 0 I={ idt(V(a)) }
Rout out 0 1
Cout out 0 1u
";
    // Nodes (0-based): a=0, out=1.
    let code = generate_nodal(spice, 0, 1);
    let out = compile_and_run(&code, &settle_main(4800), "idt_const");
    let v_out: f64 = out.parse().unwrap_or_else(|_| panic!("bad output: {out:?}"));
    let expected = -(4800.0 / 48000.0);
    assert!(
        (v_out - expected).abs() < 5e-3,
        "idt(const): got {v_out}, expected ≈ {expected}"
    );
}

#[test]
fn behavioral_voltage_tanh_matches_oracle() {
    // V={} sets V(out) - V(0) = tanh(V(a)) directly (positive, unlike I={}).
    let spice = "\
Behavioral voltage source
Va a 0 DC 0.6
B1 out 0 V={ tanh(V(a)) }
Rout out 0 1meg
";
    // Nodes (0-based): a=0, out=1.
    let code = generate_nodal(spice, 0, 1);
    let out = compile_and_run(&code, &settle_main(64), "vtanh");
    let v_out: f64 = out.parse().unwrap_or_else(|_| panic!("bad output: {out:?}"));
    let expected = 0.6f64.tanh();
    assert!(
        (v_out - expected).abs() < 1e-5,
        "V=tanh: got {v_out}, expected {expected}"
    );
}

#[test]
fn fm_limiter_phase_chain_compiles_and_runs() {
    // The Subspace FM front-end limiter (normalize I/Q) feeding a phase detector
    // (atan2 of the limited I/Q). Exercises V={}, atan2, sqrt, and chained
    // behavioral sources reading each other's nodes. A STATIC I/Q vector gives a
    // constant phase = atan2(0.8, 0.6) ≈ 0.927 rad, and the limiter normalizes
    // lim_i = iq_i/|iq| = 0.6.
    //
    // NOTE: the *ddt* discriminator (instantaneous frequency = ddt(phase)) is a
    // separate known issue — its inv_dt-scaled Jacobian (~SR·∂atan2) ill-
    // conditions the coupled NR. See docs/aidocs/BEHAVIORAL_SOURCES.md
    // "ddt discriminator stiffness". This test covers the rest of the chain.
    let spice = "\
FM limiter + phase
Viq_i iq_i 0 DC 0.6
Viq_q iq_q 0 DC 0.8
B_lim_i lim_i 0 V={ V(iq_i) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_lim_q lim_q 0 V={ V(iq_q) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_phase phase 0 V={ atan2(V(lim_q), V(lim_i)) }
Rli lim_i 0 1meg
Rlq lim_q 0 1meg
Rp phase 0 1meg
";
    // Nodes (0-based): iq_i=0, iq_q=1, lim_i=2, lim_q=3, phase=4.
    let code_li = generate_nodal(spice, 0, 2);
    let lim_i: f64 = compile_and_run(&code_li, &settle_main(64), "fm_lim_i")
        .parse()
        .unwrap();
    assert!(
        (lim_i - 0.6).abs() < 1e-4,
        "limiter normalize: lim_i got {lim_i}, expected 0.6"
    );

    let code_ph = generate_nodal(spice, 0, 4);
    let phase: f64 = compile_and_run(&code_ph, &settle_main(64), "fm_phase")
        .parse()
        .unwrap();
    let expected = 0.8f64.atan2(0.6);
    assert!(
        (phase - expected).abs() < 1e-4,
        "phase detector: got {phase}, expected {expected}"
    );
}

#[test]
fn named_params_resolve_const_and_scalar_runtime() {
    // `.param gain` is a baked constant; `.runtime amp` is a plugin scalar set
    // via set_runtime_amp. I = gain*amp*V(a) ⇒ V(out) = -gain*amp*V(a).
    let spice = "\
Behavioral with params
Va a 0 DC 1
.param gain = 0.5
.runtime amp 0 2 as amp
B1 out 0 I={ gain * amp * V(a) }
Rout out 0 1
Cout out 0 1u
";
    // Nodes (0-based): a=0, out=1.
    let code = generate_nodal(spice, 0, 1);
    let main = "fn main() {\n\
        \x20   let mut s = CircuitState::default();\n\
        \x20   s.set_runtime_amp(2.0);\n\
        \x20   let mut y = [0.0f64; NUM_OUTPUTS];\n\
        \x20   for _ in 0..256 { y = process_sample(0.0, &mut s); }\n\
        \x20   println!(\"{:.9}\", y[0]);\n\
        }\n";
    let out = compile_and_run(&code, main, "params");
    let v_out: f64 = out.parse().unwrap_or_else(|_| panic!("bad output: {out:?}"));
    let expected = -(0.5 * 2.0 * 1.0);
    assert!(
        (v_out - expected).abs() < 1e-4,
        "params: got {v_out}, expected {expected}"
    );
}

#[test]
fn unknown_param_is_rejected() {
    // A bare identifier with no .param / .runtime definition must error clearly.
    let spice = "\
Behavioral unknown param
Va a 0 DC 0.5
B1 out 0 V={ mystery * V(a) }
Rout out 0 1meg
";
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    mna.g[0][0] += 1.0;
    let cfg = CodegenConfig {
        circuit_name: "unk".to_string(),
        sample_rate: 48000.0,
        input_node: 0,
        output_nodes: vec![1],
        output_scales: vec![1.0],
        ..CodegenConfig::default()
    };
    let err = CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .unwrap_err();
    assert!(
        format!("{err}").to_lowercase().contains("mystery"),
        "unexpected: {err}"
    );
}
