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
fn fm_discriminator_static_phase_is_zero() {
    // Full FM front-end: limiter (normalize I/Q) + discriminator
    // (ddt of the instantaneous phase). A STATIC I/Q vector → constant phase →
    // the demodulated output (instantaneous frequency) settles to ~0, and the
    // limiter normalizes lim_i = iq_i/|iq| = 0.6. Exercises the ddt(atan2)
    // discriminator with the lagged-Jacobian fix (no NR stiffness collapse).
    let spice = "\
FM discriminator (static)
Viq_i iq_i 0 DC 0.6
Viq_q iq_q 0 DC 0.8
B_lim_i lim_i 0 V={ V(iq_i) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_lim_q lim_q 0 V={ V(iq_q) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_demod audio 0 V={ ddt( atan2(V(lim_q), V(lim_i)) ) }
Rli lim_i 0 1meg
Rlq lim_q 0 1meg
Ra audio 0 1meg
";
    // Nodes (0-based): iq_i=0, iq_q=1, lim_i=2, lim_q=3, audio=4.
    let code_li = generate_nodal(spice, 0, 2);
    let lim_i: f64 = compile_and_run(&code_li, &settle_main(64), "fm_lim_i")
        .parse()
        .unwrap();
    assert!(
        (lim_i - 0.6).abs() < 1e-4,
        "limiter normalize: lim_i got {lim_i}, expected 0.6 (no NR collapse)"
    );

    let code_d = generate_nodal(spice, 0, 4);
    let audio: f64 = compile_and_run(&code_d, &settle_main(64), "fm_demod")
        .parse()
        .unwrap();
    assert!(
        audio.is_finite() && audio.abs() < 1e-3,
        "static-phase discriminator should be ~0, got {audio}"
    );
}

#[test]
fn fm_discriminator_recovers_frequency() {
    // The discriminator must recover instantaneous frequency: a rotating carrier
    // at f Hz (phase = idt(2π·f)) → ddt(atan2(Q, I)) = 2π·f between phase wraps.
    // This is the whole point of the FM discriminator and the load-bearing
    // ddt(atan2) path. f = 100 Hz ⇒ 2π·100 ≈ 628.3 rad/s.
    let spice = "\
FM discriminator (rotating)
.runtime f_test 0 10000 as f_test
B_ph ph 0 V={ idt(2*pi*f_test) }
B_ci iq_i 0 V={ cos(V(ph)) }
B_cq iq_q 0 V={ sin(V(ph)) }
B_lim_i lim_i 0 V={ V(iq_i) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_lim_q lim_q 0 V={ V(iq_q) / sqrt(V(iq_i)*V(iq_i) + V(iq_q)*V(iq_q) + 1e-9) }
B_demod audio 0 V={ ddt( atan2(V(lim_q), V(lim_i)) ) }
Rph ph 0 1meg
Rii iq_i 0 1meg
Riq iq_q 0 1meg
Rli lim_i 0 1meg
Rlq lim_q 0 1meg
Ra audio 0 1meg
";
    // audio node index: ph=0, iq_i=1, iq_q=2, lim_i=3, lim_q=4, audio=5 (0-based).
    let code = generate_nodal(spice, 0, 5);
    // f=100 Hz: phase reaches π near sample 240; average over pre-wrap samples.
    let main = "fn main() {\n\
        \x20   let mut s = CircuitState::default();\n\
        \x20   s.set_runtime_f_test(100.0);\n\
        \x20   let mut acc = 0.0f64; let mut n = 0u32;\n\
        \x20   for k in 0..200 { let y = process_sample(0.0, &mut s); if k >= 10 { acc += y[0]; n += 1; } }\n\
        \x20   println!(\"{:.6}\", acc / (n as f64));\n\
        }\n";
    let mean: f64 = compile_and_run(&code, main, "fm_freq").parse().unwrap();
    let expected = 2.0 * std::f64::consts::PI * 100.0;
    assert!(
        (mean - expected).abs() < 1.0,
        "discriminator frequency: got {mean}, expected {expected}"
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
fn bsource_with_slew_opamp_no_pots_compiles() {
    // Regression: the B-source (full-LU nodal) path omitted `current_sample_rate`
    // when there were no pots/switches, but the op-amp slew block (finite SR)
    // references it → E0609. A B-source circuit with a slew-limited op-amp and no
    // pots must still rustc-compile. (Mirrors the AM antenna's JRC4558 SR=1.)
    let spice = "\
Behavioral + slew op-amp, no pots
Va a 0 DC 0.5
B1 sig 0 I={ tanh(V(a)) }
Rsig sig 0 1
Csig sig 0 1u
.model OP OA(AOL=200000 GBW=3MEG ROUT=75 SR=1 VCC=15 VEE=-15)
U1 0 vn out OP
Rg sig vn 1k
Rf out vn 50k
Rl out 0 100k
";
    // Just needs to generate AND rustc-compile (the bug is a compile error).
    let code = generate_nodal(spice, 0, 0);
    assert!(
        code.contains("pub current_sample_rate: f64"),
        "current_sample_rate field must be declared when an op-amp has finite SR"
    );
    // A trivial main that constructs state and runs one sample proves it compiles.
    let main = "fn main() { let mut s = CircuitState::default(); let _ = process_sample(0.0, &mut s); }\n";
    let _ = compile_and_run(&code, main, "slew_nopots");
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

/// Regression: the compile-time sparse-LU pattern must include behavioral
/// B-source Jacobian stamp positions.
///
/// `emit_behavioral_jacobian` stamps aug/terminal rows × every referenced-node
/// column. Before the fix, `lu::compute_g_aug_pattern` knew nothing of
/// behavioral sources, so on a sparse-eligible circuit (N >= 8, density < 40%)
/// with a B-source coexisting with M >= 1 devices, those positions were absent
/// from the symbolic elimination schedule — the stamps were silently never
/// eliminated and NR converged to a wrong fixed point with no diagnostic.
///
/// The test uses a `V={}` source referencing non-terminal nodes plus a diode
/// (M=1) on an RC ladder (N=10 incl. the behavioral aug row), and compares the
/// sparse-path output against the dense path (forced at the emitted refactor
/// site, which also exercises the runtime sparse→dense fallback wiring) to 1e-9.
#[test]
fn behavioral_sparse_lu_pattern_matches_dense() {
    let spice = "\
Bsrc sparse LU pattern regression
R1 in n1 1k
C1 n1 0 10n
R2 n1 n2 1k
C2 n2 0 10n
R3 n2 n3 1k
C3 n3 0 10n
R4 n3 n4 1k
C4 n4 0 10n
R5 n4 n5 1k
C5 n5 0 10n
R6 n5 n6 1k
C6 n6 0 10n
D1 n6 0 DCLIP
.model DCLIP D(IS=2.52e-9 N=1.752)
B1 bout 0 V={ tanh(2.0*V(n3)) + 0.5*V(n6) }
Rl bout out 1k
Rout out 0 10k
Cout out 0 10n
";
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    mna.g[input_node][input_node] += 1.0;
    let cfg = CodegenConfig {
        circuit_name: "bsrc_sparse".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        output_scales: vec![1.0],
        input_resistance: 1.0,
        dc_block: false,
        ..CodegenConfig::default()
    };
    let code = CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .expect("codegen")
        .code;

    // The circuit must actually take the sparse path, or this test is vacuous.
    assert!(
        code.contains("fn sparse_lu_factor"),
        "expected sparse LU to be emitted for this circuit (N >= 8, low density)"
    );
    // The behavioral V={} Jacobian stamp must appear in the refactor block.
    assert!(
        code.contains("bsrc_0_g_"),
        "expected behavioral Jacobian partials in generated code"
    );

    // Dense variant: force the runtime sparse->dense fallback at the emitted
    // call site. This is the SAME binary logic path used when the growth-factor
    // check rejects a sparse factorization.
    let dense_code = code.replace("if sparse_lu_factor(", "if false && sparse_lu_factor(");
    assert_ne!(code, dense_code, "dense forcing substitution did not apply");

    let main = "fn main() {\n\
        \x20   let mut s = CircuitState::default();\n\
        \x20   for i in 0..512u32 {\n\
        \x20       let t = i as f64 / 48000.0;\n\
        \x20       let input = 2.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
        \x20       let y = process_sample(input, &mut s);\n\
        \x20       println!(\"{:.15e}\", y[0]);\n\
        \x20   }\n\
        }\n";

    let sparse_out = compile_and_run(&code, main, "sparse_pattern_sparse");
    let dense_out = compile_and_run(&dense_code, main, "sparse_pattern_dense");

    let parse = |s: &str| -> Vec<f64> {
        s.lines()
            .filter_map(|l| l.trim().parse::<f64>().ok())
            .collect()
    };
    let sparse_v = parse(&sparse_out);
    let dense_v = parse(&dense_out);
    assert_eq!(sparse_v.len(), 512);
    assert_eq!(dense_v.len(), 512);

    // Output must be non-trivial (signal + nonlinearities actually engaged).
    let peak = sparse_v.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
    assert!(peak > 1e-3, "output should be non-trivial, peak={peak:.3e}");
    assert!(
        sparse_v.iter().all(|v| v.is_finite()),
        "sparse outputs must be finite"
    );

    let max_diff = sparse_v
        .iter()
        .zip(dense_v.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    assert!(
        max_diff < 1e-9,
        "sparse vs dense mismatch: max_diff={max_diff:.3e} — behavioral stamp \
         positions missing from the sparse-LU pattern?"
    );
}
