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
    // Phase E.3 G_aug build: the body builds a local `g_aug` matrix from
    // `G` and the live pot/switch fields but doesn't yet consume it. Later
    // phases replace the `let _ = g_aug;` bookmark with actual NR-solve
    // code — updating these assertions then is a deliberate synchronization
    // point.
    assert!(
        code.contains("Phase E MVP (Oomox P6)"),
        "flag ON should emit the Phase E header comment"
    );
    assert!(
        code.contains("let mut g_aug: [[f64; N]; N] = G;"),
        "flag ON (E.3) should build a local g_aug matrix from const G"
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

// -----------------------------------------------------------------------------
// Phase E.3: G_aug construction from live pot/switch state.
//
// These tests guard that `recompute_dc_op` reads the mutable per-instance
// fields (`self.pot_*_resistance`, `self.switch_*_position`) rather than
// any frozen codegen-time constant. Crucial for the Phase E goal: applying
// per-instance jitter before calling `recompute_dc_op` must change what the
// method solves.
// -----------------------------------------------------------------------------

const POT_NETLIST: &str = "\
Phase E.3 pot circuit — simple voltage divider with a .pot on R2
R1 in mid 10k
R2 mid 0 10k
.pot R2 1k 100k
C1 mid out 100n
R3 out 0 100k
";

const SWITCH_NETLIST: &str = "\
Phase E.3 switch circuit — tone stack with a 3-position R switch
R1 in mid 10k
R2 mid out 22k
.switch R2 10k 22k 47k
C1 out 0 100n
R3 out 0 100k
";

/// Pot circuits: the emitted `g_aug` build reads `self.pot_0_resistance`
/// and subtracts `POT_0_G_NOM` — not a frozen constant. Per-instance
/// jitter applied before the call must change what the solver sees.
#[test]
fn flag_on_pot_build_reads_live_resistance() {
    let code = generate_dk(POT_NETLIST, true);
    assert!(
        code.contains("1.0 / self.pot_0_resistance - POT_0_G_NOM"),
        "E.3 pot emission must read the live self.pot_0_resistance;\n\
         first 5 lines mentioning pot_0 inside recompute_dc_op:\n{}",
        code.lines()
            .skip_while(|l| !l.contains("pub fn recompute_dc_op"))
            .take_while(|l| !l.trim_start().starts_with("}"))
            .filter(|l| l.contains("pot_0"))
            .take(5)
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// Switch circuits: the emitted `g_aug` build reads
/// `SWITCH_0_VALUES[self.switch_0_position][...]`. Ensures a position change
/// will actually re-stamp at recompute time (once the NR loop lands in E.4+).
#[test]
fn flag_on_switch_build_reads_live_position() {
    let code = generate_dk(SWITCH_NETLIST, true);
    assert!(
        code.contains("SWITCH_0_VALUES[self.switch_0_position][0]"),
        "E.3 switch emission must index SWITCH_0_VALUES by the live \
         self.switch_0_position"
    );
    assert!(
        code.contains("stamp_conductance(&mut g_aug, SWITCH_0_COMP_0_NODE_P"),
        "E.3 switch emission must stamp the R-component delta onto g_aug \
         using stamp_conductance"
    );
}

/// End-to-end: pot circuit with the flag ON compiles, `recompute_dc_op` is
/// callable after a `set_pot_0` jitter, and the G_aug build executes
/// without panic. The method is still a no-op semantically (E.3 doesn't
/// run NR yet) — this test only proves the emitted code is valid Rust and
/// the live-field reads don't trip any out-of-bounds / type mismatch.
#[test]
fn flag_on_pot_compiles_and_runs_after_jitter() {
    use std::io::Write;

    let code = generate_dk(POT_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        // Simulate per-instance jitter: push R2 to +10% of nominal.\n\
        state.set_pot_0(11_000.0);\n\
        state.recompute_dc_op();\n\
        // Now the other direction, -10%.\n\
        state.set_pot_0(9_000.0);\n\
        state.recompute_dc_op();\n\
        // And at both extremes (clamped by POT_0_MIN_R / _MAX_R).\n\
        state.set_pot_0(POT_0_MIN_R);\n\
        state.recompute_dc_op();\n\
        state.set_pot_0(POT_0_MAX_R);\n\
        state.recompute_dc_op();\n\
        let dc = state.dc_op();\n\
        assert_eq!(dc.len(), N);\n\
        println!(\"ok\");\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_pot.rs");
    let bin = tmp.join("melange_dc_op_recompute_pot");
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

/// End-to-end: switch circuit with the flag ON compiles, `recompute_dc_op`
/// is callable at each switch position, and G_aug rebuilds cleanly.
#[test]
fn flag_on_switch_compiles_and_runs_at_each_position() {
    use std::io::Write;

    let code = generate_dk(SWITCH_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        for pos in 0..SWITCH_0_NUM_POSITIONS {\n\
            state.set_switch_0(pos);\n\
            state.recompute_dc_op();\n\
        }\n\
        let dc = state.dc_op();\n\
        assert_eq!(dc.len(), N);\n\
        println!(\"ok\");\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_switch.rs");
    let bin = tmp.join("melange_dc_op_recompute_switch");
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

// -----------------------------------------------------------------------------
// Phase E.4: per-device i_nl + dense Jacobian evaluation inside recompute_dc_op
//
// These tests guard that the new evaluator block is emitted when the circuit
// has at least one nonlinear device, mirrors the transient solver's device
// dispatch, packs locals into `i_nl` + `j_dev`, and still compiles. The NR
// loop itself is E.5 — here the outputs are discarded with `let _ = ...;`
// and `recompute_dc_op` remains a no-op semantically.
// -----------------------------------------------------------------------------

const DIODE_NETLIST: &str = "\
Phase E.4 diode circuit — one diode clamped to ground
R1 in mid 10k
D1 mid 0 DMOD
R2 mid out 10k
C1 out 0 100n
R3 out 0 100k
.model DMOD D(IS=2.5e-9 N=1.8)
";

// JFET stays on the DK path and exercises the 2D-slot packing in E.4.
// (A BJT at this dimensionality would route to nodal full-LU via the
// `K≈0 / positive K diagonal` heuristic, so `generate_dk` — which force-
// constructs a DkKernel — rejects it.)
const JFET_NETLIST: &str = "\
Phase E.4 JFET circuit — common-source JFET stage (2D NR block)
R1 in g 1meg
J1 d g 0 JMOD
R2 d out 10k
R3 out 0 100k
C1 out 0 100n
.model JMOD NJF(VTO=-3.5 BETA=1e-3 LAMBDA=0.01)
";

/// Diode flag-on: recompute_dc_op body contains the v_nl extraction reading
/// `v_node`, the shared diode evaluator call, and the `j_dev[...]` pack.
#[test]
fn e4_diode_flag_on_emits_device_eval() {
    let code = generate_dk(DIODE_NETLIST, true);

    let body_start = code
        .find("pub fn recompute_dc_op")
        .expect("recompute_dc_op must be emitted with flag ON");
    let window_end = (body_start + 8_000).min(code.len());
    let body = &code[body_start..window_end];

    assert!(
        body.contains("let state: &CircuitState = &*self;")
            && body.contains("let v_node: [f64; N] = state.v_prev;"),
        "E.4: body must alias self as state then warm-start v_node.\n\
         body preview (first 1200 chars):\n{}",
        &body[..body.len().min(1200)]
    );
    assert!(
        body.contains("let v_d0 = ") && body.contains("v_node["),
        "E.4: body must emit `let v_d{{i}} = … v_node[…]` extraction"
    );
    assert!(
        body.contains("let i_dev0 = diode_current("),
        "E.4: diode slot must produce `let i_dev0 = diode_current(…)`"
    );
    assert!(
        body.contains("let jdev_0_0 = diode_conductance("),
        "E.4: diode slot must produce `let jdev_0_0 = diode_conductance(…)`"
    );
    assert!(
        body.contains("let mut i_nl: [f64; M] = [0.0; M];")
            && body.contains("i_nl[0] = i_dev0;"),
        "E.4: must pack i_dev{{i}} into i_nl array"
    );
    assert!(
        body.contains("let mut j_dev: [[f64; M]; M] = [[0.0; M]; M];")
            && body.contains("j_dev[0][0] = jdev_0_0;"),
        "E.4: must pack jdev_{{r}}_{{c}} into dense j_dev matrix"
    );
    assert!(
        body.contains("let _ = (v_node, i_nl, j_dev);"),
        "E.4: must discard v_node/i_nl/j_dev pending E.5 NR loop"
    );
}

/// JFET flag-on: recompute_dc_op delegates to the shared `jfet_id` /
/// `jfet_jacobian` helpers (2D block) and produces a full 2×2 Jacobian
/// pack into j_dev.
#[test]
fn e4_jfet_flag_on_emits_jfet_evaluate() {
    let code = generate_dk(JFET_NETLIST, true);

    let body_start = code
        .find("pub fn recompute_dc_op")
        .expect("recompute_dc_op must be emitted with flag ON");
    let window_end = (body_start + 8_000).min(code.len());
    let body = &code[body_start..window_end];

    assert!(
        body.contains("jfet_id(v_d1, v_d0, state.device_0_idss"),
        "E.4: JFET slot must reuse the shared jfet_id helper \
         (args are (vgs=v_d1, vds=v_d0, ...)).\n\
         body preview (first 1500 chars):\n{}",
        &body[..body.len().min(1500)]
    );
    assert!(
        body.contains("let jfet0_jac = jfet_jacobian(v_d1, v_d0"),
        "E.4: JFET slot must compute the 2x2 jfet_jacobian"
    );
    for (r, c) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
        assert!(
            body.contains(&format!("j_dev[{r}][{c}] = jdev_{r}_{c};")),
            "E.4: missing j_dev[{r}][{c}] pack for JFET 2×2 block"
        );
    }
    assert!(
        body.contains("i_nl[0] = i_dev0;") && body.contains("i_nl[1] = i_dev1;"),
        "E.4: JFET 2D block must pack both i_dev0 and i_dev1"
    );
}

/// Linear-only circuit with the flag ON: no device eval block should be
/// emitted (the `if ir.topology.m > 0 && !ir.device_slots.is_empty()` guard
/// short-circuits). recompute_dc_op stays compilable and callable.
#[test]
fn e4_linear_m0_flag_on_skips_device_eval() {
    let code = generate_dk(REGRESSION_NETLIST, true);
    let body_start = code
        .find("pub fn recompute_dc_op")
        .expect("recompute_dc_op must be emitted with flag ON");
    let window_end = (body_start + 4_000).min(code.len());
    let body = &code[body_start..window_end];

    assert!(
        !body.contains("Phase E.4: evaluate device currents"),
        "E.4: linear-only circuit must NOT emit the device eval block"
    );
    assert!(
        !body.contains("let v_node: [f64; N] = state.v_prev;"),
        "E.4: linear-only circuit must NOT warm-start v_node (no devices to eval)"
    );
    assert!(
        !body.contains("let mut i_nl: [f64; M] = [0.0; M];"),
        "E.4: linear-only circuit must NOT pack i_nl (M=0)"
    );
    assert!(
        body.contains("let mut g_aug: [[f64; N]; N] = G;"),
        "E.4: linear-only circuit still emits E.3 G_aug build"
    );
}

/// End-to-end: diode circuit with the flag ON compiles under rustc, and
/// `recompute_dc_op` runs to completion without panic or UB. Catches any
/// type / scoping / naming bug in the E.4 emission.
#[test]
fn e4_diode_flag_on_compiles_and_runs() {
    use std::io::Write;

    let code = generate_dk(DIODE_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        for _ in 0..4 { state.recompute_dc_op(); }\n\
        let dc = state.dc_op();\n\
        assert_eq!(dc.len(), N);\n\
        println!(\"ok\");\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e4_diode.rs");
    let bin = tmp.join("melange_dc_op_recompute_e4_diode");
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

/// End-to-end: JFET circuit with the flag ON compiles. JFET slot is 2D so
/// this exercises packing of a 2×2 Jacobian block and two i_dev locals.
#[test]
fn e4_jfet_flag_on_compiles_and_runs() {
    use std::io::Write;

    let code = generate_dk(JFET_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        for _ in 0..4 { state.recompute_dc_op(); }\n\
        let dc = state.dc_op();\n\
        assert_eq!(dc.len(), N);\n\
        println!(\"ok\");\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e4_jfet.rs");
    let bin = tmp.join("melange_dc_op_recompute_e4_jfet");
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
