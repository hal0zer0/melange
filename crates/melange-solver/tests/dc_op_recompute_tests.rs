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
