//! Op-amp rail-mode regression tests for the nodal emitter.
//!
//! Guards the 2026-07 rail-mode fixes:
//!
//! 1. **Active-set resolve dead on full-LU**: the resolve's violation
//!    detection used strict `>` / `<` against the same literal the (then
//!    unconditional) per-iteration clamp pinned `v` to, so at convergence
//!    `v == rail` exactly, `any_pinned` never fired, and ActiveSet degraded
//!    to Hard semantics on the full-LU path. Detection is now inclusive
//!    (mirroring `emit_nodal_active_set_check`) and the per-iteration clamp
//!    is Hard-gated, so the resolve fires on genuine violations. Observable
//!    via the `diag_active_set_pin_count` counter.
//!
//! 2. **Mode `None` not honored**: per-iteration clamps ran in ALL modes
//!    (and the Schur BE-fallback dispatch shared a `Hard | None` arm), so
//!    None-mode output was silently bounded. None now emits no rail handling
//!    anywhere — the raw, unbounded output diagnostic works again.
//!
//! 3. **M=0 emits no rail handling**: both linear (M=0) nodal branches
//!    (Schur `v = v_pred` and full-LU direct solve) ignored the rail mode
//!    entirely. They now emit the mode-appropriate handling.
//!
//! 4. **`fmt_param_const` non-finite rendering**: `{:e}` rendered infinite
//!    `.param` constants as the invalid Rust token `inf`; now routed through
//!    `fmt_f64` (`f64::INFINITY`).

use melange_solver::codegen::{CodeGenerator, CodegenConfig, OpampRailMode};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

// ─── Circuits ───────────────────────────────────────────────────────
//
// All three share the same overdriven inverting amp: gain −10 (Rfb/Rin),
// rails ±12 V, driven with a 3 V sine → ideal ±30 V output, well past the
// rails. `Cout`/`Rld2` add an AC-coupled downstream load (the topology the
// ActiveSet family exists for).

/// Nonlinear-NR-path variant: the inert behavioral source (`I ≈ 1 nA·V(baux)`
/// on an otherwise-grounded node) forces the full-LU nodal routing and the
/// trap-NR loop without changing the electrical behavior of the amp.
const OVERDRIVEN_AMP_FULL_LU_NR: &str = "\
Overdriven inverting amp (full-LU NR path via inert B-source)
.model OA_TEST OA(AOL=100000 ROUT=100 VCC=12 VEE=-12)
Rin   in     inv    10K
U1    0      inv    out   OA_TEST
Rfb   out    inv    100K
Rload out    0      10K
Cout  out    out_ac 1U
Rld2  out_ac 0      100K
Baux  baux   0      I={ 1e-9 * V(baux) }
Rbaux baux   0      1K
.END
";

/// Linear (M=0) variant that routes to the nodal Schur path.
const OVERDRIVEN_AMP_M0_SCHUR: &str = "\
Overdriven inverting amp (M=0, Schur)
.model OA_TEST OA(AOL=100000 ROUT=100 VCC=12 VEE=-12)
Rin   in     inv    10K
U1    0      inv    out   OA_TEST
Rfb   out    inv    100K
Rload out    0      10K
Cout  out    out_ac 1U
Rld2  out_ac 0      100K
.END
";

/// Linear (M=0) variant that routes to the nodal full-LU path: the cap-only
/// node `x1` (two series 1 pF caps, no resistive path) makes `max|S| > 1e6`,
/// which trips the `s_ill_conditioned` full-LU routing guard.
const OVERDRIVEN_AMP_M0_FULL_LU: &str = "\
Overdriven inverting amp (M=0, full-LU via cap-only node)
.model OA_TEST OA(AOL=100000 ROUT=100 VCC=12 VEE=-12)
Rin   in     inv    10K
U1    0      inv    out   OA_TEST
Rfb   out    inv    100K
Rload out    0      10K
Cout  out    out_ac 1U
Rld2  out_ac 0      100K
Cxa   in     x1     1P
Cxb   x1     0      1P
.END
";

// ─── Helpers ────────────────────────────────────────────────────────

static COUNTER: AtomicU32 = AtomicU32::new(0);

fn generate_nodal(spice: &str, rail_mode: OpampRailMode) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    mna.g[input_node][input_node] += 1.0;

    let config = CodegenConfig {
        circuit_name: "rail_mode_test".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        output_scales: vec![1.0],
        input_resistance: 1.0,
        // No DC block: output[0] is the raw op-amp output-node voltage, so
        // the peak assertions below measure the actual rail behavior rather
        // than a high-passed/clamped copy.
        dc_block: false,
        opamp_rail_mode: rail_mode,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_railmode_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_railmode_{tag}_{id}_{counter}"));

    let full_code = format!("{code}\n\n{main_code}\n");
    {
        let mut f = std::fs::File::create(&src_path).unwrap();
        f.write_all(full_code.as_bytes()).unwrap();
    }

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2021")
        .arg("-O")
        .arg("-A")
        .arg("warnings")
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

/// Drive a 3 V, 1 kHz sine for 0.1 s and report the raw output peak plus the
/// active-set pin counter.
const DRIVE_MAIN: &str = r#"
fn main() {
    let sr = 48000.0;
    let mut state = CircuitState::default();
    let mut peak = 0.0f64;
    for i in 0..4800 {
        let t = i as f64 / sr;
        let input = 3.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let y = process_sample(input, &mut state);
        if y[0].abs() > peak { peak = y[0].abs(); }
    }
    println!("peak={:.9}", peak);
    println!("pins={}", state.diag_active_set_pin_count);
}
"#;

fn parse_kv(output: &str, key: &str) -> f64 {
    output
        .lines()
        .find_map(|l| l.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("missing `{key}=` in output:\n{output}"))
        .trim()
        .parse()
        .unwrap_or_else(|e| panic!("bad `{key}` value: {e}\n{output}"))
}

const RAIL: f64 = 12.0;
/// The active-set resolve pins the output node to the rail exactly; Hard
/// clamps it exactly. Allow only float noise above the rail.
const RAIL_EPS: f64 = 1e-6;
/// Ideal unclamped peak is ≈30 V; anything comfortably past the rail proves
/// the output is genuinely unbounded.
const UNBOUNDED_FLOOR: f64 = 20.0;

// ─── Fix 1 + 2: full-LU NR path ─────────────────────────────────────

/// None mode on the full-LU NR path: raw output exceeds the rails (the
/// Klon-style raw diagnostic). Before the fix, the per-iteration clamps ran
/// in ALL modes and silently bounded this at ±12 V.
#[test]
fn full_lu_nr_none_mode_output_is_unbounded() {
    let code = generate_nodal(OVERDRIVEN_AMP_FULL_LU_NR, OpampRailMode::None);
    assert!(
        code.contains("Cross-timestep chord"),
        "circuit no longer routes to the full-LU nodal path — test premise broken"
    );
    let out = compile_and_run(&code, DRIVE_MAIN, "fulllu_nr_none");
    let peak = parse_kv(&out, "peak");
    assert!(
        peak > UNBOUNDED_FLOOR,
        "None mode should leave the op-amp output unbounded (~30 V ideal), got peak={peak:.3} — \
         a rail clamp is firing in None mode"
    );
}

/// Hard mode on the full-LU NR path still clamps (the death-spiral
/// protection the per-iteration clamp was introduced for is retained where
/// the mode contract wants hard bounding).
#[test]
fn full_lu_nr_hard_mode_output_is_clamped() {
    let code = generate_nodal(OVERDRIVEN_AMP_FULL_LU_NR, OpampRailMode::Hard);
    let out = compile_and_run(&code, DRIVE_MAIN, "fulllu_nr_hard");
    let peak = parse_kv(&out, "peak");
    assert!(
        peak <= RAIL + RAIL_EPS,
        "Hard mode must clamp the output at the ±12 V rails, got peak={peak:.6}"
    );
}

/// ActiveSet mode on the full-LU NR path: the KCL-consistent pin-and-resolve
/// actually RUNS. Before the fix the strict `>`/`<` detection never fired
/// against the clamp-pinned value and ActiveSet degraded to Hard semantics
/// (resolve emitted but dead, pin counter would have stayed 0).
#[test]
fn full_lu_nr_active_set_resolve_fires() {
    let code = generate_nodal(OVERDRIVEN_AMP_FULL_LU_NR, OpampRailMode::ActiveSet);
    assert!(
        code.contains("Active-set op-amp rail resolve"),
        "ActiveSet code must contain the constrained-resolve block"
    );
    let out = compile_and_run(&code, DRIVE_MAIN, "fulllu_nr_activeset");
    let peak = parse_kv(&out, "peak");
    let pins = parse_kv(&out, "pins");
    assert!(
        pins > 0.0,
        "active-set resolve never pinned on a circuit driven 2.5x past its rails — \
         the resolve is dead again (peak={peak:.3})"
    );
    assert!(
        peak <= RAIL + RAIL_EPS,
        "ActiveSet must bound the output at the pinned rail value, got peak={peak:.6}"
    );
}

// ─── Fix 3: M=0 linear paths ────────────────────────────────────────

#[test]
fn m0_schur_hard_mode_respects_rails() {
    let code = generate_nodal(OVERDRIVEN_AMP_M0_SCHUR, OpampRailMode::Hard);
    assert!(
        code.contains("Linear circuit: v = v_pred (no NR needed)"),
        "circuit no longer routes to the M=0 Schur branch — test premise broken"
    );
    assert!(
        code.contains("Op-amp supply rail clamp (Hard mode, linear circuit)"),
        "M=0 Schur branch must emit the Hard rail clamp"
    );
    let out = compile_and_run(&code, DRIVE_MAIN, "m0_schur_hard");
    let peak = parse_kv(&out, "peak");
    assert!(
        peak <= RAIL + RAIL_EPS,
        "M=0 Schur Hard mode must clamp at the ±12 V rails, got peak={peak:.6}"
    );
}

#[test]
fn m0_schur_none_mode_output_is_unbounded() {
    let code = generate_nodal(OVERDRIVEN_AMP_M0_SCHUR, OpampRailMode::None);
    let out = compile_and_run(&code, DRIVE_MAIN, "m0_schur_none");
    let peak = parse_kv(&out, "peak");
    assert!(
        peak > UNBOUNDED_FLOOR,
        "M=0 Schur None mode should be unbounded (~30 V ideal), got peak={peak:.3}"
    );
}

#[test]
fn m0_schur_active_set_resolve_fires() {
    let code = generate_nodal(OVERDRIVEN_AMP_M0_SCHUR, OpampRailMode::ActiveSet);
    assert!(
        code.contains("Active-set op-amp rail resolve"),
        "M=0 Schur ActiveSet code must contain the constrained-resolve block"
    );
    let out = compile_and_run(&code, DRIVE_MAIN, "m0_schur_activeset");
    let peak = parse_kv(&out, "peak");
    let pins = parse_kv(&out, "pins");
    assert!(pins > 0.0, "M=0 Schur active-set resolve never pinned (peak={peak:.3})");
    assert!(
        peak <= RAIL + RAIL_EPS,
        "M=0 Schur ActiveSet must bound the output at the rail, got peak={peak:.6}"
    );
}

#[test]
fn m0_full_lu_hard_mode_respects_rails() {
    let code = generate_nodal(OVERDRIVEN_AMP_M0_FULL_LU, OpampRailMode::Hard);
    assert!(
        code.contains("Linear circuit: direct LU solve (no NR needed)"),
        "circuit no longer routes to the M=0 full-LU branch — test premise broken"
    );
    assert!(
        code.contains("Op-amp supply rail clamp (Hard mode, linear circuit)"),
        "M=0 full-LU branch must emit the Hard rail clamp"
    );
    let out = compile_and_run(&code, DRIVE_MAIN, "m0_fulllu_hard");
    let peak = parse_kv(&out, "peak");
    assert!(
        peak <= RAIL + RAIL_EPS,
        "M=0 full-LU Hard mode must clamp at the ±12 V rails, got peak={peak:.6}"
    );
}

#[test]
fn m0_full_lu_none_mode_output_is_unbounded() {
    let code = generate_nodal(OVERDRIVEN_AMP_M0_FULL_LU, OpampRailMode::None);
    let out = compile_and_run(&code, DRIVE_MAIN, "m0_fulllu_none");
    let peak = parse_kv(&out, "peak");
    assert!(
        peak > UNBOUNDED_FLOOR,
        "M=0 full-LU None mode should be unbounded (~30 V ideal), got peak={peak:.3}"
    );
}

#[test]
fn m0_full_lu_active_set_resolve_fires() {
    let code = generate_nodal(OVERDRIVEN_AMP_M0_FULL_LU, OpampRailMode::ActiveSet);
    let out = compile_and_run(&code, DRIVE_MAIN, "m0_fulllu_activeset");
    let peak = parse_kv(&out, "peak");
    let pins = parse_kv(&out, "pins");
    assert!(pins > 0.0, "M=0 full-LU active-set resolve never pinned (peak={peak:.3})");
    assert!(
        peak <= RAIL + RAIL_EPS,
        "M=0 full-LU ActiveSet must bound the output at the rail, got peak={peak:.6}"
    );
}

// ─── Fix 4: fmt_param_const non-finite rendering ────────────────────

/// Non-finite `.param` constants must never reach codegen as bare `inf` /
/// `NaN` tokens. The FIRST line of defense is the parser, which rejects
/// overflowing values (`1e400` → f64 +inf → parse error) — this test pins
/// that rejection. The SECOND line is `fmt_param_const` in the nodal
/// emitter, which now routes non-finite values through `fmt_f64`
/// (`f64::INFINITY` / `f64::NEG_INFINITY` / `f64::NAN`) instead of `{:e}`
/// (which rendered the invalid Rust tokens `inf` / `NaN`) in case a
/// non-finite constant ever arrives through a non-parser path.
#[test]
fn non_finite_param_const_rejected_at_parse() {
    let spice = "\
Infinite param constant
.param big=1e400
Rin  in  a   1K
Ra   a   0   1K
B1   out 0   I={ V(a) / big }
Rout out 0   1K
.END
";
    let err = Netlist::parse(spice).expect_err(
        "parser accepted a .param value that overflows f64 to +inf — if this \
         is ever relaxed, fmt_param_const's fmt_f64 routing becomes the only \
         guard against invalid `inf` tokens in generated code",
    );
    let msg = format!("{err:?}");
    assert!(
        msg.contains("1e400"),
        "rejection should name the offending value, got: {msg}"
    );
}
