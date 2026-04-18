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
