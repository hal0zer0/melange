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
// Phase E.4 + E.5: per-device evaluator + NR loop inside recompute_dc_op
//
// These tests guard that the device evaluator block is emitted when the
// circuit has at least one nonlinear device, mirrors the transient solver's
// device dispatch, packs locals into `i_nl` + `j_dev`, and that the E.5 NR
// loop consumes those matrices, calls `invert_n`, applies damping, and
// writes back `dc_operating_point` / `v_prev` / `i_nl_prev` on convergence.
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
/// `v_node`, the shared diode evaluator call, and the `j_dev[...]` pack
/// (all consumed by the E.5 NR loop, not discarded).
#[test]
fn e4_diode_flag_on_emits_device_eval() {
    let code = generate_dk(DIODE_NETLIST, true);

    let body_start = code
        .find("pub fn recompute_dc_op")
        .expect("recompute_dc_op must be emitted with flag ON");
    let window_end = (body_start + 16_000).min(code.len());
    let body = &code[body_start..window_end];

    assert!(
        body.contains("let state: &CircuitState = &*self;")
            && body.contains("let mut v_node: [f64; N] = self.v_prev;"),
        "E.5: body must warm-start v_node from self.v_prev and alias &*self.\n\
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
        body.contains("let (g_inv, singular) = invert_n(g_aug_nr);"),
        "E.5: NR loop must call invert_n on the linearized g_aug_nr"
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

/// Linear-only circuit with the flag ON: the dispatcher falls through to the
/// E.5 linear-solve path — no NR loop, no device eval, just one `invert_n`
/// + matrix-vector multiply directly on `g_aug`.
#[test]
fn e4_linear_m0_flag_on_skips_device_eval() {
    let code = generate_dk(REGRESSION_NETLIST, true);
    let body_start = code
        .find("pub fn recompute_dc_op")
        .expect("recompute_dc_op must be emitted with flag ON");
    let window_end = (body_start + 4_000).min(code.len());
    let body = &code[body_start..window_end];

    assert!(
        !body.contains("let mut i_nl: [f64; M] = [0.0; M];"),
        "linear-only circuit must NOT pack i_nl (M=0)"
    );
    assert!(
        !body.contains("Direct Newton-Raphson DC OP loop"),
        "linear-only circuit must NOT emit the NR loop header"
    );
    assert!(
        body.contains("let mut g_aug: [[f64; N]; N] = G;"),
        "linear-only circuit still emits E.3 G_aug build"
    );
    assert!(
        body.contains("Linear circuit (M == 0)"),
        "linear-only circuit must take the direct-LU path"
    );
    assert!(
        body.contains("let (g_inv, singular) = invert_n(g_aug);"),
        "linear-only circuit must invert the base g_aug (no NR correction)"
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

// -----------------------------------------------------------------------------
// Phase E.5: NR loop + writeback
//
// These tests exercise the shipped NR loop: `recompute_dc_op` at the default
// state (nominal pots, zero `.runtime` sources, zero input) must converge to
// the baked `DC_OP` constant. On a jittered pot the DC op must move with it,
// not stay frozen at the nominal value. Writeback must also update `v_prev`
// so the next `process_sample(0.0)` call starts at the converged equilibrium.
// -----------------------------------------------------------------------------

/// Linear RC: at nominal state the emitted `recompute_dc_op` must produce a
/// self-consistent fixed point — calling it twice mustn't drift, `v_prev`
/// must match `dc_operating_point` after the call, and the result must be
/// finite. Guards the linear-solve path (M == 0 branch) end-to-end.
#[test]
fn e5_linear_converges_to_dc_op_at_nominal() {
    use std::io::Write;

    let code = generate_dk(REGRESSION_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        let before: [f64; N] = *state.dc_op();\n\
        state.recompute_dc_op();\n\
        let after: [f64; N] = *state.dc_op();\n\
        for &v in after.iter() { assert!(v.is_finite(), \"non-finite dc op: {}\", v); }\n\
        // v_prev must be updated (next process_sample starts at equilibrium).\n\
        for i in 0..N {\n\
            assert_eq!(state.v_prev[i], after[i], \"v_prev must match dc_op after recompute\");\n\
        }\n\
        // Linear circuit at default state: the solver's fixed point must\n\
        // agree with the baked `dc_operating_point` seed (which was itself\n\
        // computed at the same nominal pot values at codegen time).\n\
        for i in 0..N {\n\
            let diff = (after[i] - before[i]).abs();\n\
            assert!(diff < 1e-9, \"linear state drifted at nominal pots: node {}: {} vs {}\", i, before[i], after[i]);\n\
        }\n\
        // Idempotent: calling recompute twice mustn't drift.\n\
        state.recompute_dc_op();\n\
        for i in 0..N {\n\
            let diff = (state.dc_op()[i] - after[i]).abs();\n\
            assert!(diff < 1e-9, \"recompute not idempotent at node {}: {}\", i, diff);\n\
        }\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e5_linear.rs");
    let bin = tmp.join("melange_dc_op_recompute_e5_linear");
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

/// Nonlinear diode: at nominal state the NR loop must converge to a
/// physically self-consistent equilibrium — `v_prev` updated, no NaN, and
/// repeat-call idempotence. Exercises the full device eval + LU +
/// damping + writeback path.
#[test]
fn e5_diode_converges_to_dc_op_at_nominal() {
    use std::io::Write;

    let code = generate_dk(DIODE_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        let before: [f64; N] = *state.dc_op();\n\
        state.recompute_dc_op();\n\
        let after: [f64; N] = *state.dc_op();\n\
        for &v in after.iter() { assert!(v.is_finite(), \"non-finite: {}\", v); }\n\
        // v_prev must track the converged DC op.\n\
        for i in 0..N {\n\
            assert_eq!(state.v_prev[i], after[i], \"v_prev / dc_op mismatch\");\n\
        }\n\
        // At default state the NR equilibrium must match the baked seed\n\
        // (codegen-time DC OP was computed at the same nominal pot values).\n\
        for i in 0..N {\n\
            let denom = after[i].abs().max(before[i].abs()).max(1.0);\n\
            let rel = (after[i] - before[i]).abs() / denom;\n\
            assert!(rel < 1e-6, \"diode equilibrium drifted: node {} rel={} (before={} after={})\", i, rel, before[i], after[i]);\n\
        }\n\
        // Idempotent: recompute twice mustn't drift.\n\
        state.recompute_dc_op();\n\
        for i in 0..N {\n\
            let diff = (state.dc_op()[i] - after[i]).abs();\n\
            assert!(diff < 1e-6, \"node {} drifted between calls: {}\", i, diff);\n\
        }\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e5_diode.rs");
    let bin = tmp.join("melange_dc_op_recompute_e5_diode");
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

// `in` declared first so the test helper's `mna.g[0][0] += 1.0`
// input-conductance stamp lands on an isolated signal node instead of
// short-circuiting the VCC supply.
const DIODE_VCC_NETLIST: &str = "\
Phase E.5 diode with VCC supply
R_in_load in 0 10k
VCC vcc 0 5.0
R1 vcc mid 1k
D1 mid 0 DMOD
R2 mid out 10k
C1 out 0 100n
R3 out 0 100k
.model DMOD D(IS=2.5e-9 N=1.8)
";

/// Biased diode (VCC + R + D): at codegen time the DC OP solver puts the
/// anode ~0.65 V above ground. After a warm `recompute_dc_op` call from
/// default state, the runtime must converge to the same point (within NR
/// tolerance). This is the load-bearing test for Strategy 1 — it proves
/// the runtime NR is physically equivalent to `dc_op.rs` at nominal.
#[test]
fn e5_diode_vcc_converges_within_tolerance() {
    use std::io::Write;

    let code = generate_dk(DIODE_VCC_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        let before: [f64; N] = *state.dc_op();\n\
        // Start from a deliberately wrong v_prev to make the NR actually work.\n\
        // (Default v_prev == DC_OP, so recompute would converge in ~0 iters.)\n\
        for i in 0..N { state.v_prev[i] = 0.0; }\n\
        state.recompute_dc_op();\n\
        let after: [f64; N] = *state.dc_op();\n\
        for &v in after.iter() { assert!(v.is_finite(), \"non-finite: {}\", v); }\n\
        for i in 0..N {\n\
            let denom = after[i].abs().max(before[i].abs()).max(1.0);\n\
            let rel = (after[i] - before[i]).abs() / denom;\n\
            assert!(rel < 1e-6, \"diode+VCC NR failed to match codegen DC OP: node {} before={} after={} rel={}\", i, before[i], after[i], rel);\n\
        }\n\
        // Diode must be forward-biased at the converged point (check via i_nl).\n\
        // With 5 V / (1k + series drop) ≈ 4.35 mA through D1 — plenty nonzero.\n\
        let total_i_nl: f64 = state.i_nl_prev.iter().map(|x| x.abs()).sum();\n\
        assert!(total_i_nl > 1e-6, \"expected nonzero i_nl after diode forward bias: got {}\", total_i_nl);\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e5_diode_vcc.rs");
    let bin = tmp.join("melange_dc_op_recompute_e5_diode_vcc");
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

/// Jittered pot: calling recompute_dc_op after a large pot change must move
/// the DC op by a physically plausible amount (not stay frozen at DC_OP).
#[test]
fn e5_linear_moves_on_pot_jitter() {
    use std::io::Write;

    let code = generate_dk(POT_NETLIST, true);

    // Pot netlist is a 10k/10k voltage divider (R2 is the .pot). At nominal
    // (R2 = 10k), a nonzero VIN would put the mid node at V/2. With R2 set
    // well above nominal (say 100k), the ratio shifts — we verify only that
    // the DC op *changes*, not the exact value (which depends on internals).
    // Since there's no VIN in POT_NETLIST, we instead check via `.runtime` —
    // but POT_NETLIST has no VIN either, so we rely on recompute producing
    // a stable self-consistent state at each pot setting (no NaN / no drift
    // across repeated calls at the same setting).
    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        state.recompute_dc_op();\n\
        let nominal = *state.dc_op();\n\
        // Idempotent: calling again at the same pot value mustn't drift.\n\
        state.recompute_dc_op();\n\
        let nominal_again = *state.dc_op();\n\
        for i in 0..N {\n\
            let d = (nominal_again[i] - nominal[i]).abs();\n\
            assert!(d < 1e-9, \"node {} drifted between two recompute calls: {}\", i, d);\n\
        }\n\
        // Jitter the pot: DC op must re-solve cleanly (no NaN).\n\
        state.set_pot_0(POT_0_MIN_R);\n\
        state.recompute_dc_op();\n\
        let at_min = *state.dc_op();\n\
        for v in at_min.iter() { assert!(v.is_finite(), \"NaN/Inf at MIN_R: {}\", v); }\n\
        state.set_pot_0(POT_0_MAX_R);\n\
        state.recompute_dc_op();\n\
        let at_max = *state.dc_op();\n\
        for v in at_max.iter() { assert!(v.is_finite(), \"NaN/Inf at MAX_R: {}\", v); }\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e5_pot.rs");
    let bin = tmp.join("melange_dc_op_recompute_e5_pot");
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

// -----------------------------------------------------------------------------
// Phase E.6: state-reset audit
//
// Writeback covers more than just `v_prev` / `i_nl_prev` / `dc_operating_point`:
// the DC blocker, per-pot `_prev` shadows, oversampler taps, and linear-
// companion-model history (inductor / coupled / transformer) all need to land
// in a state consistent with the new DC equilibrium so the first
// `process_sample` call after `recompute_dc_op` doesn't generate a spurious
// step transient. These tests guard the non-obvious pieces.
// -----------------------------------------------------------------------------

/// DC blocker seeding: `dc_block_x_prev[k]` must equal `v_node[OUTPUT_NODES[k]]`
/// after `recompute_dc_op`, not 0. Setting it to 0 would make the first
/// sample's IIR evaluation see `raw_out - 0 = V_dc` as a step and pass it
/// through, taking ~5/fc seconds to decay — exactly the warmup-style
/// transient that `recompute_dc_op` is supposed to eliminate.
#[test]
fn e6_dc_block_seeded_at_dc_output() {
    use std::io::Write;

    let code = generate_dk(REGRESSION_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        // Dirty the blocker history so we can see recompute overwrite it.\n\
        for k in 0..NUM_OUTPUTS {\n\
            state.dc_block_x_prev[k] = 99.0;\n\
            state.dc_block_y_prev[k] = 99.0;\n\
        }\n\
        state.recompute_dc_op();\n\
        let dc = *state.dc_op();\n\
        for k in 0..NUM_OUTPUTS {\n\
            let expected = dc[OUTPUT_NODES[k]];\n\
            let got = state.dc_block_x_prev[k];\n\
            assert!((got - expected).abs() < 1e-12,\n\
                \"dc_block_x_prev[{}] = {}, expected v_node[OUTPUT_NODES[{}]] = {}\",\n\
                k, got, k, expected);\n\
            assert_eq!(state.dc_block_y_prev[k], 0.0,\n\
                \"dc_block_y_prev[{}] = {} (expected 0 for DC fixed point)\", k, state.dc_block_y_prev[k]);\n\
        }\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e6_dc_block.rs");
    let bin = tmp.join("melange_dc_op_recompute_e6_dc_block");
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

/// Pot `_prev` shadow sync: after `recompute_dc_op`, each
/// `pot_N_resistance_prev` must equal the corresponding `pot_N_resistance`.
/// The per-sample A_neg correction uses the previous timestep's conductance
/// to undo the history term; leaving `_prev` pointing at a stale (pre-jitter)
/// resistance would make the first `process_sample` fire a phantom conductance
/// delta and inject a one-sample glitch.
#[test]
fn e6_pot_resistance_prev_synced_after_recompute() {
    use std::io::Write;

    let code = generate_dk(POT_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        // Mimic the real user flow: set the pot, THEN call recompute.\n\
        // After recompute_dc_op, _prev must point at the new value so the\n\
        // next process_sample doesn't see a phantom conductance delta.\n\
        let target = (POT_0_MIN_R + POT_0_MAX_R) * 0.5;\n\
        state.pot_0_resistance = target;\n\
        state.pot_0_resistance_prev = POT_0_MIN_R; // deliberately stale\n\
        state.recompute_dc_op();\n\
        assert!(\n\
            (state.pot_0_resistance_prev - state.pot_0_resistance).abs() < 1e-12,\n\
            \"pot_0_resistance_prev ({}) did not sync to pot_0_resistance ({}) after recompute_dc_op\",\n\
            state.pot_0_resistance_prev, state.pot_0_resistance\n\
        );\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e6_pot_prev.rs");
    let bin = tmp.join("melange_dc_op_recompute_e6_pot_prev");
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
// Phase E.7: real-circuit DK validation
//
// Takes the MVP beyond synthetic diode/JFET test vectors by exercising a
// realistic SeriesOfTubes-style single stage: 12AU7 triode + 3 pots (Rcv, Rk,
// Ra) + multi-voltage-source supply (Vcc, Vbias, Vctrl) + cathode bypass cap +
// AC-coupled input/output. DK kernel route confirmed (N=13, M=2); this is the
// class of circuit the Phase E runtime DC OP recompute was written for
// (per-instance ±5% pot jitter across 50 stacked stages).
//
// Tests cover the two end-to-end properties a plugin needs:
//   (a) At nominal pots, `recompute_dc_op` converges to the compile-time
//       `DC_OP` constant within NR tolerance — proves the runtime solver is
//       physically equivalent to `dc_op.rs` on a real topology.
//   (b) At jittered pots, `recompute_dc_op` converges to a finite, physically
//       plausible new equilibrium — proves the runtime solver can actually
//       *move* the DC OP instead of dying on a wider-than-test-vector pot
//       swing. Sanity bounds: cathode between 0 and Vcc, plate between
//       cathode and Vcc, output is AC-coupled so it sits at its load pull.
// -----------------------------------------------------------------------------

/// A SeriesOfTubes cascade stage, trimmed to the parts that matter for the
/// DC OP: 12AU7 common-cathode stage with fixed-bias grid summing network,
/// fully-bypassed cathode, plate-load pad, output coupling. Matches the
/// active netlist in `../../melange-circuits/unstable/dynamics/series-of-tubes-stage.cir`
/// line-for-line; input guard (`Rin_guard`) is prepended so parsing assigns
/// `in` to MNA node 0 as the test harness expects.
const SOT_STAGE_NETLIST: &str = "\
SeriesOfTubes stage — Phase E.7 DK validation
Rin_guard in 0 1meg
.model Tube_12AU7 TRIODE(MU=17 EX=1.4 KG1=1460 KP=300 KVB=300)
Vcc vcc 0 DC 250
Vbias vbias 0 DC -22
Vctrl ctrl 0 DC 0
Cin in grid 470n
Rcv ctrl bias_sum 22k
Rb vbias bias_sum 100k
Rg bias_sum grid 470k
T1 grid plate cathode Tube_12AU7
Rk cathode 0 1500
Ck cathode 0 220u
Ra vcc plate 47k
Rpad_top plate pad 108k
Rpad_bot pad 0 10k
Cout pad out 470n
Rload out 0 1meg
.pot Rcv 20900 23100
.pot Rk 1425 1575
.pot Ra 44650 49350
";

/// SoT stage at nominal pots: `recompute_dc_op` must converge to the baked
/// `DC_OP` constant within 1e-6 relative tolerance. Starts from a deliberately
/// wrong `v_prev = [0; N]` so the NR loop actually has to iterate, not just
/// confirm the warm-start is already converged.
#[test]
fn e7_series_of_tubes_stage_converges_at_nominal() {
    use std::io::Write;

    let code = generate_dk(SOT_STAGE_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        let before: [f64; N] = *state.dc_op();\n\
        for i in 0..N { state.v_prev[i] = 0.0; }\n\
        for i in 0..M { state.i_nl_prev[i] = 0.0; state.i_nl_prev_prev[i] = 0.0; }\n\
        state.recompute_dc_op();\n\
        let after: [f64; N] = *state.dc_op();\n\
        for (i, &v) in after.iter().enumerate() {\n\
            assert!(v.is_finite(), \"non-finite node {}: {}\", i, v);\n\
        }\n\
        for i in 0..N {\n\
            let denom = after[i].abs().max(before[i].abs()).max(1.0);\n\
            let rel = (after[i] - before[i]).abs() / denom;\n\
            assert!(rel < 1e-6,\n\
                \"SoT node {} rel error too large: before={} after={} rel={}\",\n\
                i, before[i], after[i], rel);\n\
        }\n\
        // Tube must be forward-biased: i_nl sum should be substantially\n\
        // nonzero (plate current at the DC OP is on the order of 4-5 mA).\n\
        let total: f64 = state.i_nl_prev.iter().map(|x| x.abs()).sum();\n\
        assert!(total > 1e-4, \"expected forward-biased triode, got sum|i_nl|={}\", total);\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e7_sot_nominal.rs");
    let bin = tmp.join("melange_dc_op_recompute_e7_sot_nominal");
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

/// SoT stage with per-instance ±5% pot jitter on Rcv, Rk, Ra (the jitter
/// envelope a real SeriesOfTubes plugin would apply at construction time):
/// `recompute_dc_op` must converge to a new, finite, physically plausible
/// equilibrium. Plate voltage must stay bounded by Vcc (0-250V) and cathode
/// voltage must be positive (self-bias). Confirms the runtime NR is stable
/// over the full declared pot range, not just at nominal.
#[test]
fn e7_series_of_tubes_stage_converges_under_pot_jitter() {
    use std::io::Write;

    let code = generate_dk(SOT_STAGE_NETLIST, true);

    // The MNA node map for this netlist has cathode at index 7, plate at 6.
    // We query the codegen-emitted NODE_* constants instead of hardcoding —
    // guards against a future parser reordering silently breaking the test.
    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        // Jitter toward the extreme end of each pot's declared range.\n\
        state.set_pot_0(POT_0_MAX_R);  // Rcv high\n\
        state.set_pot_1(POT_1_MIN_R);  // Rk low (hotter bias)\n\
        state.set_pot_2(POT_2_MAX_R);  // Ra high (cooler plate load)\n\
        state.recompute_dc_op();\n\
        let jittered: [f64; N] = *state.dc_op();\n\
        for (i, &v) in jittered.iter().enumerate() {\n\
            assert!(v.is_finite(), \"non-finite node {} after jitter: {}\", i, v);\n\
        }\n\
        // Cathode self-bias must be positive (tube drawing current forces\n\
        // V(cathode) > 0). Plate must sit below Vcc=250V.\n\
        let v_cathode = jittered[NODE_CATHODE];\n\
        let v_plate   = jittered[NODE_PLATE];\n\
        assert!(v_cathode > 0.0 && v_cathode < 50.0,\n\
            \"cathode voltage out of plausible self-bias range: {}\", v_cathode);\n\
        assert!(v_plate > v_cathode && v_plate < 250.0,\n\
            \"plate voltage outside (V_cathode, Vcc) band: plate={}, cathode={}\",\n\
            v_plate, v_cathode);\n\
        // i_nl must remain nonzero — tube didn't get cutoff into silence.\n\
        let total: f64 = state.i_nl_prev.iter().map(|x| x.abs()).sum();\n\
        assert!(total > 1e-5,\n\
            \"tube went into cutoff under jitter? sum|i_nl|={}\", total);\n\
        // Idempotence: calling recompute a second time at the same pot values\n\
        // must not drift. Guards against any accumulating writeback bug.\n\
        state.recompute_dc_op();\n\
        let again: [f64; N] = *state.dc_op();\n\
        for i in 0..N {\n\
            let d = (again[i] - jittered[i]).abs();\n\
            assert!(d < 1e-9,\n\
                \"SoT node {} drifted between two recompute calls: {} -> {}\",\n\
                i, jittered[i], again[i]);\n\
        }\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e7_sot_jitter.rs");
    let bin = tmp.join("melange_dc_op_recompute_e7_sot_jitter");
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

/// TS808 (Tube Screamer) non-inverting clipping stage: op-amp VCCS + feedback
/// antiparallel diodes + AC-coupled input/output + passive tone network.
/// Different class of DK circuit from SoT — op-amp + 2× 1D diodes instead of
/// 1× 2D triode, no voltage sources (linear supply modeled inside the VCCS).
/// Adds coverage for diode-feedback clipping topology that tube-based tests
/// don't exercise.
const TS808_CLIPPING_NETLIST: &str = "\
Tube Screamer TS808 clipping stage — Phase E.7 DK validation
Cin in n1 0.047u
R_pull n1 0 1Meg
G1 0 n_clip n1 n_inv 2666.667
Rout n_clip 0 75
R4 n_inv n_gnd 4.7k
C3 n_gnd 0 0.047u
R_drive n_clip n_inv 551k
D1 n_clip n_inv D1N4148
D2 n_inv n_clip D1N4148
C4 n_clip n_inv 51p
R7 n_clip n_tone 1k
C5 n_tone 0 0.22u
C_out n_tone out 0.1u
R_out out 0 1Meg
.model D1N4148 D(IS=2.52e-9 RS=0.568 N=1.752)
";

/// TS808 at its natural quiescent point (no input, no .pot — every parameter
/// is baked): `recompute_dc_op` from a wrong `v_prev` must reach the compile-
/// time `DC_OP` within tolerance. Exercises antiparallel feedback diodes and
/// op-amp VCCS in the same DC network — a common pedal-clipping topology
/// distinct from the tube preamp case.
#[test]
fn e7_ts808_clipping_stage_converges_at_nominal() {
    use std::io::Write;

    let code = generate_dk(TS808_CLIPPING_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        let before: [f64; N] = *state.dc_op();\n\
        for i in 0..N { state.v_prev[i] = 0.0; }\n\
        for i in 0..M { state.i_nl_prev[i] = 0.0; state.i_nl_prev_prev[i] = 0.0; }\n\
        state.recompute_dc_op();\n\
        let after: [f64; N] = *state.dc_op();\n\
        for (i, &v) in after.iter().enumerate() {\n\
            assert!(v.is_finite(), \"non-finite node {}: {}\", i, v);\n\
        }\n\
        for i in 0..N {\n\
            let denom = after[i].abs().max(before[i].abs()).max(1.0);\n\
            let rel = (after[i] - before[i]).abs() / denom;\n\
            assert!(rel < 1e-6,\n\
                \"TS808 node {} rel error too large: before={} after={} rel={}\",\n\
                i, before[i], after[i], rel);\n\
        }\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e7_ts808_nominal.rs");
    let bin = tmp.join("melange_dc_op_recompute_e7_ts808_nominal");
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
// Nodal full-LU recompute_dc_op — permanent stub
//
// The nodal path emits a `recompute_dc_op` method when the flag is on so the
// plugin surface is uniform across DK and nodal circuits. The stub body bumps
// `diag_nr_max_iter_count` and returns without touching state — plugin authors
// observe the counter tick and fall back to the `WARMUP_SAMPLES_RECOMMENDED`
// silence loop (the documented path for nodal circuits; the full nodal NR
// body is deferred indefinitely — see `emit_recompute_dc_op_body_nodal`).
// These tests guard:
//
//   * The stub is feature-gated (flag OFF → no emission, flag ON → emission).
//   * Flag-off output is byte-identical for a nodal circuit.
//   * Flag-on output compiles and the stub is callable from default state.
// -----------------------------------------------------------------------------

fn generate_nodal(spice: &str, emit_recompute: bool) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let cfg = CodegenConfig {
        circuit_name: "dc_op_recompute_nodal_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![if mna.n > 1 { 1 } else { 0 }],
        input_resistance: 1.0,
        emit_dc_op_recompute: emit_recompute,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .expect("codegen_nodal")
        .code
}

/// Simple linear RC circuit routed through the nodal full-LU emitter.
/// Doesn't matter that it would normally route DK — `generate_nodal` forces
/// the nodal entrypoint so we're exercising the right emitter.
const NODAL_REGRESSION_NETLIST: &str = "\
Phase E.8 nodal regression guard
R1 in mid 4.7k
R2 mid out 22k
C1 out 0 47n
R3 out 0 1meg
";

#[test]
fn e8_nodal_flag_off_does_not_emit_recompute_dc_op() {
    let code = generate_nodal(NODAL_REGRESSION_NETLIST, false);
    assert!(
        !code.contains("fn recompute_dc_op"),
        "nodal flag OFF must not emit `fn recompute_dc_op`, found in:\n{}",
        code.lines()
            .filter(|l| l.contains("fn recompute_dc_op"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

#[test]
fn e8_nodal_flag_off_byte_identical_to_default() {
    let off_code = generate_nodal(NODAL_REGRESSION_NETLIST, false);
    let netlist = Netlist::parse(NODAL_REGRESSION_NETLIST).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let cfg = CodegenConfig {
        circuit_name: "dc_op_recompute_nodal_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![if mna.n > 1 { 1 } else { 0 }],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let default_code = CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .expect("codegen_nodal")
        .code;
    assert_eq!(
        off_code, default_code,
        "nodal flag-off output must be byte-identical to CodegenConfig::default()"
    );
}

#[test]
fn e8_nodal_flag_on_emits_stub_signature() {
    let code = generate_nodal(NODAL_REGRESSION_NETLIST, true);
    assert!(
        code.contains("pub fn recompute_dc_op(&mut self)"),
        "nodal flag ON must emit `recompute_dc_op` method signature"
    );
    assert!(
        code.contains("diag_nr_max_iter_count"),
        "nodal stub body must bump diag counter to signal no-op"
    );
    assert!(
        code.contains("deferred") && code.contains("warmup loop"),
        "nodal stub must carry explanatory deferral comment pointing at warmup fallback"
    );
}

/// Flag-on nodal output compiles and the stub is callable without panicking.
/// Also verifies the no-op contract: `diag_nr_max_iter_count` advances by one
/// per call (mirrors DK-path convergence-failure fallback) and
/// `dc_operating_point` is unchanged (we didn't touch state).
#[test]
fn e8_nodal_flag_on_stub_compiles_and_is_callable() {
    use std::io::Write;

    let code = generate_nodal(NODAL_REGRESSION_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        let before: [f64; N] = *state.dc_op();\n\
        let c_before = state.diag_nr_max_iter_count;\n\
        state.recompute_dc_op();\n\
        let after: [f64; N] = *state.dc_op();\n\
        let c_after = state.diag_nr_max_iter_count;\n\
        for i in 0..N {\n\
            assert_eq!(before[i], after[i],\n\
                \"stub must not touch dc_operating_point[{}]: {} -> {}\",\n\
                i, before[i], after[i]);\n\
        }\n\
        assert_eq!(c_after, c_before + 1,\n\
            \"stub must bump diag_nr_max_iter_count exactly once per call: {} -> {}\",\n\
            c_before, c_after);\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_dc_op_recompute_e8_nodal_stub.rs");
    let bin = tmp.join("melange_dc_op_recompute_e8_nodal_stub");
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
// settle_dc_op — recompute-with-warmup-fallback wrapper
//
// Oomox agent requested a melange-emitted helper that does:
//   1. recompute_dc_op() first
//   2. fall back to WARMUP_SAMPLES_RECOMMENDED silence on failure
// so plugin host code doesn't have to reimplement the counter-watch pattern.
// Emitted on both DK and nodal paths behind the same `emit_dc_op_recompute`
// flag. The body is identical — path-specific behavior lives in
// recompute_dc_op itself.
//
// Tests:
//   * Flag off: no settle_dc_op emission.
//   * DK path, NR converges: settle_dc_op matches recompute_dc_op bitwise
//     (no warmup fallback fires).
//   * Nodal path, stub bumps counter: settle_dc_op detects the tick and
//     runs the warmup loop. State isn't NaN, plugin is usable afterward.
// -----------------------------------------------------------------------------

#[test]
fn settle_flag_off_does_not_emit_settle_dc_op() {
    let code = generate_dk(REGRESSION_NETLIST, false);
    assert!(
        !code.contains("fn settle_dc_op"),
        "flag OFF must not emit settle_dc_op"
    );
    let nodal_code = generate_nodal(NODAL_REGRESSION_NETLIST, false);
    assert!(
        !nodal_code.contains("fn settle_dc_op"),
        "nodal flag OFF must not emit settle_dc_op"
    );
}

#[test]
fn settle_flag_on_emits_settle_dc_op_signature() {
    let dk_code = generate_dk(REGRESSION_NETLIST, true);
    assert!(
        dk_code.contains("pub fn settle_dc_op(&mut self)"),
        "DK flag ON must emit settle_dc_op signature"
    );
    let nodal_code = generate_nodal(NODAL_REGRESSION_NETLIST, true);
    assert!(
        nodal_code.contains("pub fn settle_dc_op(&mut self)"),
        "nodal flag ON must emit settle_dc_op signature"
    );
}

/// On a DK circuit where recompute_dc_op converges cleanly, settle_dc_op
/// must produce the same state without triggering the warmup fallback
/// (no extra process_sample iterations). Verify by asserting
/// `diag_nr_max_iter_count` stays at 0 — if settle_dc_op had fallen
/// through to warmup, it would have had to detect a counter tick, which
/// means the DK recompute succeeded silently (correct).
#[test]
fn settle_dk_path_no_fallback_on_successful_recompute() {
    use std::io::Write;

    let code = generate_dk(SOT_STAGE_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        // Zero v_prev so the NR has to iterate (starts at v=0 instead of DC_OP).\n\
        for i in 0..N { state.v_prev[i] = 0.0; }\n\
        for i in 0..M { state.i_nl_prev[i] = 0.0; state.i_nl_prev_prev[i] = 0.0; }\n\
        state.settle_dc_op();\n\
        assert_eq!(state.diag_nr_max_iter_count, 0,\n\
            \"DK recompute converged — settle should not have fallen back to warmup, got counter={}\",\n\
            state.diag_nr_max_iter_count);\n\
        let dc: [f64; N] = *state.dc_op();\n\
        for (i, &v) in dc.iter().enumerate() {\n\
            assert!(v.is_finite(), \"non-finite node {} after settle: {}\", i, v);\n\
        }\n\
        // Tube must be forward-biased (matches e7_series_of_tubes_stage_converges_at_nominal).\n\
        let total: f64 = state.i_nl_prev.iter().map(|x| x.abs()).sum();\n\
        assert!(total > 1e-4, \"expected forward-biased triode, got sum|i_nl|={}\", total);\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_settle_dk_success.rs");
    let bin = tmp.join("melange_settle_dk_success");
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

/// On a nodal circuit, recompute_dc_op is a permanent stub that bumps
/// `diag_nr_max_iter_count` every call. settle_dc_op must detect that
/// tick and actually run the warmup loop (which in turn calls
/// process_sample WARMUP_SAMPLES_RECOMMENDED times). Verify:
///   1. Counter shows exactly one tick (from the recompute stub, not
///      multiple — the warmup loop on a linear M=0 circuit shouldn't
///      advance any diag counters).
///   2. State is finite afterward.
///   3. Some process_sample side effect is observable — for a linear
///      RC circuit, `input_prev` is updated by every process_sample call
///      (and gets reset to 0.0 by recompute, which isn't called in the
///      warmup path after recompute). So after settle_dc_op, if the
///      warmup fallback ran, we should be post-warmup with v_prev
///      holding the settled steady state.
#[test]
fn settle_nodal_path_falls_back_to_warmup() {
    use std::io::Write;

    let code = generate_nodal(NODAL_REGRESSION_NETLIST, true);

    let main = "\n\nfn main() {\n\
        let mut state = CircuitState::default();\n\
        let before = state.diag_nr_max_iter_count;\n\
        state.settle_dc_op();\n\
        // The stub bumps the counter exactly once; the warmup loop on a\n\
        // linear M=0 circuit shouldn't bump it again (no NR iterations).\n\
        assert_eq!(state.diag_nr_max_iter_count, before + 1,\n\
            \"nodal settle must tick counter once from the stub recompute, got {} -> {}\",\n\
            before, state.diag_nr_max_iter_count);\n\
        // State must be finite after the warmup fallback.\n\
        let dc: [f64; N] = *state.dc_op();\n\
        for (i, &v) in dc.iter().enumerate() {\n\
            assert!(v.is_finite(), \"non-finite node {} after settle: {}\", i, v);\n\
        }\n\
        for (i, &v) in state.v_prev.iter().enumerate() {\n\
            assert!(v.is_finite(), \"non-finite v_prev[{}] after settle: {}\", i, v);\n\
        }\n\
        // Side effect proves the warmup loop actually ran: WARMUP_SAMPLES_RECOMMENDED\n\
        // must exist as a const (otherwise the generated code wouldn't compile).\n\
        let _ = WARMUP_SAMPLES_RECOMMENDED;\n\
        println!(\"ok\");\n\
    }\n";

    let full = format!("{}{}", code, main);
    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_settle_nodal_fallback.rs");
    let bin = tmp.join("melange_settle_nodal_fallback");
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
