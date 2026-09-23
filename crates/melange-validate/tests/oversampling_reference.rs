//! The oversampling reference compensation must BE the shipped filter.
//!
//! `melange validate --oversampling {2|4}` builds the melange side with the
//! same codegen option the plugin ships with, and puts the ngspice reference
//! through the same half-band round trip so the filters' known response sits
//! inside the comparison instead of being charged to the solver (the arbiter's
//! rule: include the known response, or gate in-band — never widen the anchor
//! around an effect whose size is known).
//!
//! That only holds if the compensation filter really is the emitted one. The
//! coefficient tables and the up/down topology exist TWICE — in
//! `melange-primitives` (which the compensation uses) and in the codegen
//! emitter (which ships) — and `docs/aidocs/OVERSAMPLING.md` flags the drift
//! hazard explicitly. These tests pin the two together from the validate side:
//!
//! 1. `emitted_coefficients_match_the_compensation_filter` — the constants the
//!    emitter bakes into generated code equal the primitives' tables, bit for
//!    bit.
//! 2. `compensation_reproduces_the_emitted_round_trip_2x` / `_4x` — the
//!    compensation output equals what the GENERATED, COMPILED oversampled code
//!    does to the same input on a circuit whose response is a pure gain. Any
//!    drift in coefficients, branch split, clocking or stage assignment shows
//!    up here as a numeric difference.
//!
//! If either fails, the compensation is silently subtracting the wrong filter
//! and every oversampled validate number is wrong. That is the "compiler
//! silently doing the wrong thing" failure the project treats as a showstopper,
//! so these assert exactly rather than within a comfortable band.

use melange_primitives::oversampling::coefficients::{HB_STEEP_7SECTION, HB_WIDE_3SECTION};
use melange_solver::codegen::{routing, CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_validate::apply_oversampling_round_trip;

/// A purely resistive divider: its response is a rate-independent gain, so the
/// only thing that can differ between a 1x and a 2x/4x build of it IS the
/// oversampling round trip. No caps, no devices, no integrator, nothing that
/// changes when the solver's timestep changes.
const DIVIDER: &str = "\
resistive divider
Rin in mid 1k
Rload mid 0 1k
";

const SAMPLE_RATE: f64 = 48_000.0;

/// Generate, compile and run the divider at `oversampling`, returning the
/// output for `input`.
fn run_divider(oversampling: usize, input: &[f64]) -> Vec<f64> {
    let netlist = Netlist::parse(DIVIDER).expect("parse divider");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");

    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["mid"] - 1;
    // Thevenin input stamp, before the kernel — see the input-modeling contract.
    mna.g[input_node][input_node] += 1.0;

    // Rate-dependent build happens at the INTERNAL rate, as the shipped
    // compile path does it.
    let routing_rate = SAMPLE_RATE * oversampling as f64;
    let kernel = DkKernel::from_mna(&mna, routing_rate).expect("dk kernel");
    let decision = routing::auto_route(&kernel, &mna, false);
    assert_eq!(
        decision.route,
        routing::SolverRoute::DkSchur,
        "divider should route DK"
    );

    let config = CodegenConfig {
        circuit_name: "os_ref".to_string(),
        sample_rate: SAMPLE_RATE,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: false,
        oversampling_factor: oversampling,
        ..CodegenConfig::default()
    };
    let generated = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .unwrap_or_else(|e| panic!("codegen at {oversampling}x: {e}"));

    let main_body = "fn main() {\n\
        let mut state = CircuitState::default();\n\
        let stdin = std::io::stdin();\n\
        let mut line = String::new();\n\
        loop {\n\
            line.clear();\n\
            if stdin.read_line(&mut line).unwrap() == 0 { break; }\n\
            if let Ok(v) = line.trim().parse::<f64>() {\n\
                println!(\"{:.17e}\", process_sample(v, &mut state)[0]);\n\
            }\n\
        }\n\
    }\n";

    let out = compile_and_run(&format!("{}\n{}", generated.code, main_body), input);
    assert_eq!(out.len(), input.len(), "one output per input sample");
    out
}

fn compile_and_run(source: &str, input: &[f64]) -> Vec<f64> {
    use std::io::Write;
    use std::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);

    let tmp = std::env::temp_dir();
    let n = COUNTER.fetch_add(1, Ordering::SeqCst);
    let pid = std::process::id();
    let src = tmp.join(format!("os_ref_{pid}_{n}.rs"));
    let bin = tmp.join(format!("os_ref_{pid}_{n}"));
    std::fs::write(&src, source).expect("write src");

    let compile = std::process::Command::new("rustc")
        .arg(&src)
        .arg("-o")
        .arg(&bin)
        .arg("--edition=2021")
        .arg("-O")
        .output()
        .expect("rustc spawn");
    let _ = std::fs::remove_file(&src);
    assert!(
        compile.status.success(),
        "generated code failed to compile:\n{}",
        String::from_utf8_lossy(&compile.stderr)
    );

    let mut child = std::process::Command::new(&bin)
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .spawn()
        .expect("spawn generated binary");
    let data: String = input.iter().map(|v| format!("{v:.17e}\n")).collect();
    let mut stdin = child.stdin.take().expect("stdin");
    std::thread::spawn(move || {
        let _ = stdin.write_all(data.as_bytes());
    });
    let out = child.wait_with_output().expect("run generated binary");
    let _ = std::fs::remove_file(&bin);
    assert!(out.status.success(), "generated binary failed");

    String::from_utf8_lossy(&out.stdout)
        .lines()
        .filter_map(|l| l.trim().parse::<f64>().ok())
        .collect()
}

/// Pull `const NAME: [f64; N] = [..];` out of emitted source.
fn emitted_const_array(source: &str, name: &str) -> Vec<f64> {
    let needle = format!("const {name}: [f64;");
    let start = source
        .find(&needle)
        .unwrap_or_else(|| panic!("emitted code has no `{name}`"));
    let open = source[start..].find('[').unwrap() + start;
    let open = source[open + 1..].find('[').unwrap() + open + 2;
    let close = source[open..].find(']').unwrap() + open;
    source[open..close]
        .split(',')
        .map(|t| {
            t.trim()
                .parse::<f64>()
                .unwrap_or_else(|_| panic!("unparsable coefficient `{t}` in {name}"))
        })
        .collect()
}

fn generated_source(oversampling: usize) -> String {
    let netlist = Netlist::parse(DIVIDER).expect("parse divider");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["mid"] - 1;
    mna.g[input_node][input_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE * oversampling as f64).expect("dk kernel");
    let config = CodegenConfig {
        circuit_name: "os_ref".to_string(),
        sample_rate: SAMPLE_RATE,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: false,
        oversampling_factor: oversampling,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("codegen")
        .code
}

#[test]
fn emitted_coefficients_match_the_compensation_filter() {
    let src2 = generated_source(2);
    assert_eq!(
        emitted_const_array(&src2, "OS_COEFFS"),
        HB_STEEP_7SECTION.to_vec(),
        "2x emitted OS_COEFFS drifted from melange-primitives' HB_STEEP_7SECTION, \
         which is what `melange validate --oversampling` compensates the reference with"
    );

    let src4 = generated_source(4);
    assert_eq!(
        emitted_const_array(&src4, "OS_COEFFS"),
        HB_WIDE_3SECTION.to_vec(),
        "4x inner stage drifted from HB_WIDE_3SECTION"
    );
    assert_eq!(
        emitted_const_array(&src4, "OS_COEFFS_OUTER"),
        HB_STEEP_7SECTION.to_vec(),
        "4x outer stage drifted from HB_STEEP_7SECTION"
    );
}

/// The test signal: a 1 kHz tone (what `melange validate` drives) plus a short
/// step, so the comparison sees both steady state and a transient the filters'
/// phase response acts on hardest.
fn test_input() -> Vec<f64> {
    (0..2048)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            let tone = 0.3 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let step = if (600..610).contains(&i) { 0.2 } else { 0.0 };
            tone + step
        })
        .collect()
}

fn assert_compensation_matches_emitted(factor: usize) {
    let input = test_input();
    let base = run_divider(1, &input);
    let oversampled = run_divider(factor, &input);

    // The divider's response is a pure gain, so the 1x output IS the
    // "reference" and the only difference the oversampled build can introduce
    // is the round trip. Compensate the reference exactly as
    // `validate_circuit_with_options` does.
    let mut compensated = base.clone();
    apply_oversampling_round_trip(&mut compensated, factor, SAMPLE_RATE);

    let worst = compensated
        .iter()
        .zip(oversampled.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    let peak = oversampled.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(
        peak > 0.05,
        "test signal did not reach the output (peak {peak:e})"
    );
    assert!(
        worst < 1e-12,
        "compensation filter is not the emitted {factor}x round trip: worst \
         sample difference {worst:e} on a pure-gain circuit (peak {peak:e}). \
         Coefficients, branch split, clocking or stage assignment have drifted \
         between melange-primitives and the codegen emitter."
    );

    // And confirm the thing being compensated for is real: without it, the
    // same comparison is dominated by the round trip's group delay.
    let raw_worst = base
        .iter()
        .zip(oversampled.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    assert!(
        raw_worst > 1e-3,
        "uncompensated {factor}x difference is only {raw_worst:e}; this test is \
         no longer proving anything"
    );
}

#[test]
fn compensation_reproduces_the_emitted_round_trip_2x() {
    assert_compensation_matches_emitted(2);
}

#[test]
fn compensation_reproduces_the_emitted_round_trip_4x() {
    assert_compensation_matches_emitted(4);
}

/// The round trip is allpass: magnitude flat everywhere, all of its response in
/// the phase. This is why compensating the reference with it cannot hide a
/// magnitude error in the solver — it has no magnitude of its own to hide one
/// behind. (The 2026-07 history in `docs/aidocs/OVERSAMPLING.md` is the case in
/// point: the broken filters had 0.4–0.5 dB of passband droop PER STAGE, which
/// a magnitude-flat compensation would leave fully visible.)
#[test]
fn round_trip_is_magnitude_flat() {
    for &factor in &[2usize, 4] {
        for &freq in &[100.0f64, 1_000.0, 5_000.0, 10_000.0, 18_000.0] {
            let n = 16_384;
            let skip = 4_096; // let the IIR state settle before measuring
            let mut sig: Vec<f64> = (0..n)
                .map(|i| (2.0 * std::f64::consts::PI * freq * i as f64 / SAMPLE_RATE).sin())
                .collect();
            let rms_in = rms(&sig[skip..]);
            apply_oversampling_round_trip(&mut sig, factor, SAMPLE_RATE);
            let rms_out = rms(&sig[skip..]);
            let db = 20.0 * (rms_out / rms_in).log10();
            assert!(
                db.abs() < 0.01,
                "{factor}x round trip is not magnitude-flat at {freq} Hz: {db:.4} dB"
            );
        }
    }
}

fn rms(x: &[f64]) -> f64 {
    (x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64).sqrt()
}

/// `factor == 1` must leave the reference untouched — the default path.
#[test]
fn factor_one_is_a_no_op() {
    let mut sig = test_input();
    let before = sig.clone();
    apply_oversampling_round_trip(&mut sig, 1, SAMPLE_RATE);
    assert_eq!(sig, before);
}
