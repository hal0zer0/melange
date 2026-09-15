//! BE-fallback fixed-point regression tests.
//!
//! The per-sample backward-Euler fallback inside a trapezoidal-primary solver
//! (max-iter fallback, breakpoint-BE after a `.switch`/`.pot` event, runtime
//! BE-latch) used to rebuild its RHS as
//! `RHS_CONST_BE + A_neg_be·v_prev + N_I·i_nl_prev` and then add `S_ni_be·i_nl(n)`
//! — the trap-midpoint average `i(n) + i(n-1)` on a BE step, which counts every
//! device's bias current twice. From an exact DC operating point on silence one
//! such sample moved a triode anode by tens of volts (philicorda-voicing-coupled:
//! 192.64 V → 162.93 V on a grid-side switch flip), and because the kick lands in
//! `null(C)` — the exact `z = -1` eigenspace of the trap operator — trap then
//! carried it as an undamped `(-1)^n` ring bounded only by the tube (46–57 V p-p,
//! drive-independent, invisible at a filtered output node).
//!
//! A BE step is `(G + C/T) v(n) = (C/T) v_prev + u(n) + N_I·i(n)`: the only
//! nonlinear term is the current at the NEW sample. These tests pin that:
//!  - the emitted fallback RHS (nodal Schur, nodal full-LU, DK template) carries
//!    no `N_I·i_nl_prev` term;
//!  - end to end, a `set_switch` at the DC operating point on silence leaves
//!    every node at the DC operating point (the breakpoint-BE sample is a true
//!    fixed point), and a capless node driven afterwards shows no Nyquist ring
//!    (positive lag-1 autocorrelation).

use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

use melange_solver::codegen::{CodeGenerator, CodegenConfig, NodalSubPathOverride};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// One triode stage with a switched grid-side resistor. The anode carries no
/// capacitor to ground, so it lies in `null(C)` and rings at exactly `z = -1`
/// under trap once anything kicks it. Mirrors the B20b stage of
/// philicorda-voicing-coupled (Ra 47k, Rk 1k bypassed, +250 V rail).
const SWITCHED_TRIODE: &str = "\
Triode stage with switched grid divider
VHT hp 0 DC 250
Ra hp anode 47k
Rk cath 0 1k
Ck cath 0 100u
T1 g anode cath ECC83
Rin in g1 100k
Rsw g1 g 0.01
Rg g 0 27k
.switch Rsw 0.01 1e9 \"SK\"
.model ECC83 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";

static COUNTER: AtomicU32 = AtomicU32::new(0);

fn generate_nodal(spice: &str, sub_path: NodalSubPathOverride) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["anode"] - 1;
    mna.g[input_node][input_node] += 1.0;
    let config = CodegenConfig {
        circuit_name: "be_fallback_fp".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        nodal_sub_path_override: sub_path,
        dc_block: false,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

fn generate_dk(spice: &str) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["anode"] - 1;
    mna.g[input_node][input_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, 48000.0).expect("dk kernel");
    let config = CodegenConfig {
        circuit_name: "be_fallback_fp_dk".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: false,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("dk codegen")
        .code
}

/// Every `rhs_be` build block in `code` (each starts at `let mut rhs_be` and
/// ends where the input is stamped into it).
fn fallback_rhs_blocks(code: &str) -> Vec<String> {
    code.split("let mut rhs_be = ")
        .skip(1)
        .map(|after| {
            after
                .split("rhs_be[INPUT_NODE]")
                .next()
                .unwrap_or("")
                .to_string()
        })
        .collect()
}

fn assert_no_midpoint_stamp(label: &str, code: &str) {
    let blocks = fallback_rhs_blocks(code);
    assert!(
        !blocks.is_empty(),
        "{label}: expected a trap-primary BE fallback block (let mut rhs_be = ...)"
    );
    for (k, block) in blocks.iter().enumerate() {
        assert!(
            !(block.contains("N_I[") && block.contains("state.i_nl_prev")),
            "{label}: BE fallback RHS block {k} still carries the trap-midpoint \
             N_I * i_nl_prev stamp — a BE step must stamp only N_I * i_nl(n):\n{block}"
        );
    }
}

#[test]
fn nodal_schur_fallback_rhs_has_no_midpoint_stamp() {
    let code = generate_nodal(SWITCHED_TRIODE, NodalSubPathOverride::Schur);
    assert!(code.contains("pub const BREAKPOINT_BE_SAMPLES"));
    assert_no_midpoint_stamp("nodal Schur", &code);
}

#[test]
fn nodal_full_lu_fallback_rhs_has_no_midpoint_stamp() {
    let code = generate_nodal(SWITCHED_TRIODE, NodalSubPathOverride::FullLu);
    assert_no_midpoint_stamp("nodal full-LU", &code);
}

#[test]
fn dk_fallback_rhs_has_no_midpoint_stamp() {
    let code = generate_dk(SWITCHED_TRIODE);
    assert_no_midpoint_stamp("DK", &code);
    // The trap-primary build_rhs itself MUST keep the midpoint half (the trap
    // average is split between build_rhs and compute_final_voltages).
    let build_rhs = code
        .split("fn build_rhs")
        .nth(1)
        .and_then(|after| after.split("fn ").next())
        .unwrap_or("");
    assert!(
        build_rhs.contains("N_I[") && build_rhs.contains("state.i_nl_prev"),
        "DK trap-primary build_rhs lost its N_I * i_nl_prev half"
    );
}

/// Runtime harness: start at the DC operating point, flip the switch (arms one
/// breakpoint-BE sample), run silence, then a small 1 kHz sine. Reports the
/// largest excursion of the anode from its DC operating point during the silent
/// phase, the breakpoint-BE sample count, and the lag-1 autocorrelation of the
/// mean-removed anode during the driven phase.
const MAIN: &str = r#"
fn main() {
    let mut state = CircuitState::default();
    let anode = OUTPUT_NODES[0];
    let dc = state.dc_operating_point[anode];
    state.set_switch_0(1);

    let mut max_dev = 0.0f64;
    let mut first_dev = 0.0f64;
    for n in 0..200 {
        process_sample(0.0, &mut state);
        let dev = (state.v_prev[anode] - dc).abs();
        if n == 0 { first_dev = dev; }
        if dev > max_dev { max_dev = dev; }
    }
    let fb_after_silence = state.diag_be_fallback_count;

    let mut xs = Vec::with_capacity(4800);
    for n in 0..4800usize {
        let t = n as f64 / SAMPLE_RATE;
        let u = 0.05 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        process_sample(u, &mut state);
        xs.push(state.v_prev[anode]);
    }
    let mean = xs.iter().sum::<f64>() / xs.len() as f64;
    let mut num = 0.0; let mut den = 0.0;
    for k in 1..xs.len() {
        let a = xs[k] - mean; let b = xs[k - 1] - mean;
        num += a * b; den += a * a;
    }
    let r1 = if den > 1e-30 { num / den } else { 1.0 };
    let mut max_step = 0.0f64;
    for k in 1..xs.len() { let s = (xs[k] - xs[k - 1]).abs(); if s > max_step { max_step = s; } }

    println!("dc_anode={dc}");
    println!("first_dev={first_dev}");
    println!("max_dev={max_dev}");
    println!("fb_after_silence={fb_after_silence}");
    println!("r1={r1}");
    println!("max_step={max_step}");
    println!("nr_max_iter={}", state.diag_nr_max_iter_count);
    println!("nan_reset={}", state.diag_nan_reset_count);
}
"#;

fn compile_and_run(code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_befp_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_befp_{tag}_{id}_{counter}"));

    let full_code = format!("{code}\n\n{MAIN}\n");
    std::fs::File::create(&src_path)
        .unwrap()
        .write_all(full_code.as_bytes())
        .unwrap();

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2021")
        .arg("-O")
        .output()
        .expect("rustc");
    let _ = std::fs::remove_file(&src_path);
    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Compilation failed for {tag}:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }
    let run = std::process::Command::new(&bin_path).output().expect("run");
    let _ = std::fs::remove_file(&bin_path);
    if !run.status.success() {
        panic!(
            "Binary failed for {tag}:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
    String::from_utf8_lossy(&run.stdout).to_string()
}

fn parse_kv(output: &str, key: &str) -> f64 {
    output
        .lines()
        .find(|l| l.starts_with(&format!("{key}=")))
        .unwrap_or_else(|| panic!("key '{key}' not found in:\n{output}"))
        .split('=')
        .nth(1)
        .unwrap()
        .trim()
        .parse()
        .unwrap()
}

fn assert_switch_at_dc_op_is_fixed_point(label: &str, code: &str) {
    let out = compile_and_run(code, label);
    let dc = parse_kv(&out, "dc_anode");
    let first_dev = parse_kv(&out, "first_dev");
    let max_dev = parse_kv(&out, "max_dev");
    let fb = parse_kv(&out, "fb_after_silence");
    let r1 = parse_kv(&out, "r1");
    let max_step = parse_kv(&out, "max_step");
    let nr_max = parse_kv(&out, "nr_max_iter");
    let nan = parse_kv(&out, "nan_reset");

    assert!(
        dc > 100.0,
        "{label}: anode DC OP {dc} V is not a biased triode stage"
    );
    // The breakpoint-BE sample from an exact DC operating point on silence must
    // BE a fixed point. With the trap-midpoint stamp it read ~25 V below DC
    // (162.93 V on the philicorda deck) and then rang at z = -1 forever. The
    // 1 mV bound sits four orders below the defect and comfortably above the
    // full-LU BE loop's NR convergence tolerance (measured ~4 uV).
    assert!(
        first_dev < 1e-3,
        "{label}: first sample after set_switch left the DC OP by {first_dev} V \
         (BE fallback double-counts N_I * i_nl_prev)\n{out}"
    );
    assert!(
        max_dev < 1e-3,
        "{label}: anode left the DC OP by {max_dev} V on silence after set_switch\n{out}"
    );
    // Exactly the one breakpoint-BE sample fell back; nothing else (no
    // max-iter fallback, no runtime latch) was needed on silence.
    assert!(
        (fb - 1.0).abs() < 0.5,
        "{label}: expected exactly one breakpoint-BE sample after set_switch, got {fb}\n{out}"
    );
    // Driven phase: a 1 kHz tone at 48 kHz has lag-1 autocorrelation ~ +0.99;
    // a (-1)^n ring on the capless anode would drive it negative.
    assert!(
        r1 > 0.9,
        "{label}: anode lag-1 autocorrelation {r1} — Nyquist ring on a capless node\n{out}"
    );
    assert!(
        max_step < 5.0,
        "{label}: anode sample-to-sample step {max_step} V under a 50 mV drive\n{out}"
    );
    assert_eq!(nr_max, 0.0, "{label}: trap NR hit MAX_ITER\n{out}");
    assert_eq!(nan, 0.0, "{label}: NaN reset fired\n{out}");
}

#[test]
fn nodal_schur_switch_at_dc_op_is_fixed_point() {
    let code = generate_nodal(SWITCHED_TRIODE, NodalSubPathOverride::Schur);
    assert_switch_at_dc_op_is_fixed_point("schur", &code);
}

#[test]
fn nodal_full_lu_switch_at_dc_op_is_fixed_point() {
    let code = generate_nodal(SWITCHED_TRIODE, NodalSubPathOverride::FullLu);
    assert_switch_at_dc_op_is_fixed_point("full_lu", &code);
}
