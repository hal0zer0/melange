//! Nodal solver codegen tests.
//!
//! Verifies that the nodal code generation path (full N×N NR per sample)
//! produces correct, compilable Rust code for circuits with inductors.
//!
//! Tests cover:
//! - CircuitIR::from_mna() builds valid IR
//! - Generated code compiles with rustc
//! - Generated code contains correct constants and structures
//! - Various circuit topologies (RL, diode+L, tube+transformer)

use melange_solver::codegen::ir::{CircuitIR, SolverMode};
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::io::Write;
use std::sync::atomic::{AtomicUsize, Ordering};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_mna_with_input(spice: &str, in_name: &str, r_in: f64) -> (Netlist, MnaSystem) {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let in_idx = *mna.node_map.get(in_name).unwrap();
    if in_idx > 0 {
        mna.g[in_idx - 1][in_idx - 1] += 1.0 / r_in;
    }
    (netlist, mna)
}

fn nodal_config(in_name: &str, out_name: &str, mna: &MnaSystem) -> CodegenConfig {
    let in_idx = *mna.node_map.get(in_name).unwrap() - 1;
    let out_idx = *mna.node_map.get(out_name).unwrap() - 1;
    CodegenConfig {
        circuit_name: "test_nodal".to_string(),
        sample_rate: 48000.0,
        input_node: in_idx,
        output_nodes: vec![out_idx],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

fn generate_nodal(spice: &str, in_name: &str, out_name: &str) -> String {
    let (netlist, mna) = build_mna_with_input(spice, in_name, 1.0);
    let config = nodal_config(in_name, out_name, &mna);
    let generator = CodeGenerator::new(config);
    generator
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

fn assert_compiles(code: &str, label: &str) {
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    let tmp_dir = std::env::temp_dir();
    let src = tmp_dir.join(format!("melange_nodal_test_{id}.rs"));
    let lib = tmp_dir.join(format!("melange_nodal_test_{id}.rlib"));

    {
        let mut f = std::fs::File::create(&src).expect("create temp");
        f.write_all(code.as_bytes()).expect("write temp");
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib", "-o"])
        .arg(&lib)
        .arg(&src)
        .output()
        .expect("run rustc");

    let _ = std::fs::remove_file(&src);
    let _ = std::fs::remove_file(&lib);

    assert!(
        output.status.success(),
        "Nodal codegen for '{}' failed to compile:\n{}",
        label,
        String::from_utf8_lossy(&output.stderr)
    );
}

// ---------------------------------------------------------------------------
// Circuit definitions
// ---------------------------------------------------------------------------

const RL_LOWPASS: &str = "\
RL Lowpass
R1 in out 1k
L1 out 0 100m
C1 out 0 100p
";

const DIODE_WITH_INDUCTOR: &str = "\
Diode + Inductor
R1 in mid 1k
D1 mid out D1N4148
L1 out 0 10m
C1 out 0 1n
.model D1N4148 D(IS=1e-15 N=1.5)
";

const TUBE_TRANSFORMER_FB: &str = "\
Tube + Transformer + NFB
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
Vcc vcc 0 DC 250
Cin in grid 100n
Rg grid 0 1Meg
T1 grid plate cathode 12AX7
Ra vcc plate 100k
Rk cathode 0 1.5k
L_pri plate xfmr_ct 1
L_sec out 0 0.2
L_tert fb_n fb_p 0.1
K1 L_pri L_sec 0.95
K2 L_pri L_tert 0.95
K3 L_sec L_tert 0.95
Rct xfmr_ct 0 10
Rfb fb_p cathode 1k
Rfb_gnd fb_n 0 100
Rload out 0 10k
C1 plate 0 100p
C2 out 0 100p
";

// ---------------------------------------------------------------------------
// Test 1: CircuitIR::from_mna() builds valid IR
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_ir_construction() {
    let (netlist, mna) = build_mna_with_input(RL_LOWPASS, "in", 1.0);
    let config = nodal_config("in", "out", &mna);
    let ir = CircuitIR::from_mna(&mna, &netlist, &config).unwrap();

    assert_eq!(ir.solver_mode, SolverMode::Nodal);
    assert!(ir.topology.augmented_inductors);
    assert_eq!(ir.topology.m, 0); // no nonlinear devices
                                  // n_nodal = n_aug + 1 inductor
    assert_eq!(ir.topology.n, mna.n_aug + 1);
    assert_eq!(ir.topology.n_aug, mna.n_aug);
    assert_eq!(ir.topology.n_nodes, mna.n);

    // Nodal matrices populated
    assert!(!ir.matrices.a_matrix.is_empty());
    assert!(!ir.matrices.a_matrix_be.is_empty());
    assert!(!ir.matrices.a_neg_be.is_empty());
    assert!(!ir.matrices.rhs_const_be.is_empty());
    assert!(!ir.matrices.g_matrix.is_empty());
    assert!(!ir.matrices.c_matrix.is_empty());

    // Schur complement: S is computed for nodal mode (K only if M > 0)
    assert!(
        !ir.matrices.s.is_empty(),
        "S should be computed for Schur complement"
    );

    // No companion model state
    assert!(ir.inductors.is_empty());
    assert!(ir.coupled_inductors.is_empty());
    assert!(ir.transformer_groups.is_empty());
}

// ---------------------------------------------------------------------------
// Test 2: Nodal IR for nonlinear + inductor circuit
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_ir_nonlinear_inductor() {
    let (netlist, mna) = build_mna_with_input(DIODE_WITH_INDUCTOR, "in", 1.0);
    let config = nodal_config("in", "out", &mna);
    let ir = CircuitIR::from_mna(&mna, &netlist, &config).unwrap();

    assert_eq!(ir.solver_mode, SolverMode::Nodal);
    assert_eq!(ir.topology.m, 1); // 1 diode
    assert!(ir.topology.n > mna.n_aug); // augmented with inductor var
    assert_eq!(ir.device_slots.len(), 1);
}

// ---------------------------------------------------------------------------
// Test 3: Nodal IR for tube + transformer
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_ir_tube_transformer() {
    let (netlist, mna) = build_mna_with_input(TUBE_TRANSFORMER_FB, "in", 1.0);
    let config = nodal_config("in", "out", &mna);
    let ir = CircuitIR::from_mna(&mna, &netlist, &config).unwrap();

    assert_eq!(ir.solver_mode, SolverMode::Nodal);
    assert_eq!(ir.topology.m, 2); // 1 tube = 2 dims
                                  // 3-winding transformer = 3 augmented variables
    assert_eq!(ir.topology.n, mna.n_aug + 3);
    assert!(ir.has_dc_op); // VCC = 250V
}

// ---------------------------------------------------------------------------
// Test 4: RL lowpass compiles
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_codegen_rl_compiles() {
    let code = generate_nodal(RL_LOWPASS, "in", "out");
    assert_compiles(&code, "rl_lowpass_nodal");
}

// ---------------------------------------------------------------------------
// Test 5: Diode + inductor compiles
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_codegen_diode_inductor_compiles() {
    let code = generate_nodal(DIODE_WITH_INDUCTOR, "in", "out");
    assert_compiles(&code, "diode_inductor_nodal");
}

// ---------------------------------------------------------------------------
// Test 6: Tube + transformer compiles
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_codegen_tube_transformer_compiles() {
    let code = generate_nodal(TUBE_TRANSFORMER_FB, "in", "out");
    assert_compiles(&code, "tube_xfmr_nodal");
}

// ---------------------------------------------------------------------------
// Test 7: Generated code contains essential structures
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_codegen_structure() {
    let code = generate_nodal(DIODE_WITH_INDUCTOR, "in", "out");

    // Core constants
    assert!(code.contains("const N: usize"), "missing N");
    assert!(code.contains("const M: usize"), "missing M");
    assert!(code.contains("const N_NODES: usize"), "missing N_NODES");
    assert!(code.contains("const N_AUG: usize"), "missing N_AUG");

    // Nodal matrices (A, A_neg, G, C, N_V, N_I)
    assert!(code.contains("A_DEFAULT"), "missing A matrix");
    assert!(code.contains("A_NEG_DEFAULT"), "missing A_NEG matrix");
    assert!(code.contains("A_BE_DEFAULT"), "missing A_BE matrix");
    assert!(code.contains("A_NEG_BE_DEFAULT"), "missing A_NEG_BE matrix");
    assert!(code.contains("const G:"), "missing G matrix");
    assert!(code.contains("const C:"), "missing C matrix");
    assert!(code.contains("N_V:"), "missing N_V matrix");
    assert!(code.contains("N_I:"), "missing N_I matrix");

    // State struct with NR working buffers
    assert!(
        code.contains("pub struct CircuitState"),
        "missing state struct"
    );
    assert!(code.contains("v_prev"), "missing v_prev");
    assert!(code.contains("i_nl_prev"), "missing i_nl_prev");

    // Process sample with NR loop
    assert!(code.contains("process_sample"), "missing process_sample");
    assert!(
        code.contains("v_pred") || code.contains("perm"),
        "missing Schur complement (v_pred) or LU solve (perm)"
    );

    // Device functions
    assert!(code.contains("diode_current"), "missing diode_current");

    // SPICE limiting
    assert!(
        code.contains("pnjlim") || code.contains("VCRIT"),
        "missing SPICE limiting"
    );
}

// ---------------------------------------------------------------------------
// Test 8: DC OP values present
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_codegen_dc_op() {
    let code = generate_nodal(TUBE_TRANSFORMER_FB, "in", "out");
    assert!(code.contains("const DC_OP: [f64;"), "missing DC_OP");
    assert!(code.contains("const DC_NL_I: [f64;"), "missing DC_NL_I");
}

// ---------------------------------------------------------------------------
// Test 9: set_sample_rate present and uses G/C
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_codegen_sample_rate() {
    let code = generate_nodal(RL_LOWPASS, "in", "out");
    assert!(code.contains("set_sample_rate"), "missing set_sample_rate");
    // Should rebuild A from G + alpha*C, not use S
    assert!(
        code.contains("G[") || code.contains("g_val"),
        "set_sample_rate should use G"
    );
}

// ---------------------------------------------------------------------------
// Test 10: BE fallback code present for nonlinear circuits
// ---------------------------------------------------------------------------

#[test]
fn test_nodal_codegen_be_fallback() {
    let code = generate_nodal(DIODE_WITH_INDUCTOR, "in", "out");
    // BE fallback should be emitted for nonlinear circuits
    assert!(
        code.contains("A_BE_DEFAULT") || code.contains("a_be"),
        "missing BE fallback matrices"
    );
    assert!(
        code.contains("backward") || code.contains("Backward") || code.contains("BE"),
        "missing BE fallback logic"
    );
}

// ---------------------------------------------------------------------------
// Test 11: rebuild_matrices uses correct alpha for --backward-euler circuits
//
// Regression test for the bug where rebuild_matrices always emitted
// `let alpha = 2.0 * internal_rate` (trapezoidal) even for BE-compiled
// circuits, and used `alpha*C - G` for a_neg instead of `alpha*C`.
//
// A pot change triggers rebuild_matrices. If rebuild uses trap alpha when
// BE was compiled, the resulting S/K are wrong and NR diverges on every
// sample after the first pot movement.
// ---------------------------------------------------------------------------

// Circuit that forces the nodal Schur path (multi-transformer, M >= 2) and has a pot.
// Used to verify rebuild_matrices uses the correct alpha and a_neg formula for BE.
const TUBE_TRANSFORMER_WITH_POT: &str = "\
Tube + Transformer + NFB + Pot
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
Vcc vcc 0 DC 250
Cin in grid 100n
Rg grid 0 1Meg
T1 grid plate cathode 12AX7
Ra vcc plate 100k
Rk cathode 0 1.5k
L_pri plate xfmr_ct 1
L_sec out 0 0.2
L_tert fb_n fb_p 0.1
K1 L_pri L_sec 0.95
K2 L_pri L_tert 0.95
K3 L_sec L_tert 0.95
Rct xfmr_ct 0 10
Rfb fb_p cathode 1k
Rfb_gnd fb_n 0 100
Rload out 0 10k
C1 plate 0 100p
C2 out 0 100p
Rvol out vol_w 5k
Rvol2 vol_w 0 5k
.pot Rvol 100 10000
";

fn generate_nodal_be(spice: &str, in_name: &str, out_name: &str) -> String {
    let (netlist, mna) = build_mna_with_input(spice, in_name, 1.0);
    let in_idx = *mna.node_map.get(in_name).unwrap() - 1;
    let out_idx = *mna.node_map.get(out_name).unwrap() - 1;
    let config = CodegenConfig {
        circuit_name: "test_be".to_string(),
        sample_rate: 48000.0,
        input_node: in_idx,
        output_nodes: vec![out_idx],
        input_resistance: 1.0,
        backward_euler: true,
        ..CodegenConfig::default()
    };
    let generator = CodeGenerator::new(config);
    generator
        .generate_nodal(&mna, &netlist)
        .expect("nodal BE codegen")
        .code
}

#[test]
fn test_rebuild_matrices_be_uses_correct_alpha() {
    let code = generate_nodal_be(TUBE_TRANSFORMER_WITH_POT, "in", "out");

    // The fix: rebuild_matrices must use alpha = internal_rate (BE), not 2.0 * internal_rate
    assert!(
        code.contains("let alpha = internal_rate; // backward Euler"),
        "rebuild_matrices must use alpha = internal_rate for BE circuits, not 2.0 * internal_rate"
    );
    assert!(
        !code.contains("let alpha = 2.0 * internal_rate; // trapezoidal\n        let alpha_be = internal_rate;"),
        "rebuild_matrices must not use trapezoidal alpha for BE circuits"
    );
}

#[test]
fn test_rebuild_matrices_be_a_neg_no_g_subtraction() {
    let code = generate_nodal_be(TUBE_TRANSFORMER_WITH_POT, "in", "out");

    // For BE, a_neg = alpha*C (no G subtraction).
    // The rebuild_matrices loop must NOT contain "- self.g_work[i][j]" for a_neg.
    assert!(
        code.contains("self.a_neg[i][j] = alpha * self.c_work[i][j];"),
        "rebuild_matrices BE: a_neg must be alpha*C only (no G subtraction)"
    );
    assert!(
        !code.contains("self.a_neg[i][j] = alpha * self.c_work[i][j] - self.g_work[i][j];"),
        "rebuild_matrices BE: a_neg must not subtract G (that is the trapezoidal formula)"
    );
}

/// The nodal-path NaN reset must restore the same state the DK-path resets
/// (v_prev, i_nl, pots, oversampler, DC blocker, BJT thermal, op-amp IIR) —
/// not just the minimal `v_prev` / `i_nl_prev` from before the 1.3 fix. The
/// Schur path previously returned `[0.0; NUM_OUTPUTS]` which caused an audible
/// click at the recovery edge; it now returns the DC-OP output like the
/// full-LU path and the DK template.
///
/// Uses `TUBE_TRANSFORMER_WITH_POT` because it carries a `.pot` (must reset
/// `pot_0_resistance`) and routes through the nodal emitter.
#[test]
fn test_nodal_nan_reset_includes_extended_state() {
    let code = generate_nodal(TUBE_TRANSFORMER_WITH_POT, "in", "out");

    // Shared helper signature — if this string disappears, the helper was
    // deleted or renamed; the test assertion names below will point at the
    // concrete field that regressed.
    assert!(
        code.contains("Mirrors the DK-path reset"),
        "nodal NaN reset must be emitted via the shared helper"
    );

    // Core state that was always reset — sanity check it still is.
    assert!(
        code.contains("state.v_prev = state.dc_operating_point;"),
        "v_prev must be reset on NaN"
    );
    assert!(
        code.contains("state.input_prev = 0.0;"),
        "input_prev must be reset on NaN"
    );

    // Fields added in the 1.3 fix (previously silent drift after NaN):
    assert!(
        code.contains("state.pot_0_resistance ="),
        "pot_0_resistance must be snapped back to nominal on NaN"
    );
    assert!(
        code.contains("state.pot_0_resistance_prev ="),
        "pot_0_resistance_prev must be synced to nominal on NaN"
    );

    // Return strategy: DC-OP output, not zero (Schur regression fix).
    assert!(
        code.contains("return nan_out;"),
        "nodal NaN reset must return nan_out (DC-OP output), not zero"
    );
    assert!(
        !code.contains("return [0.0; NUM_OUTPUTS];"),
        "nodal NaN reset must NOT fall back to zero return — causes click at recovery"
    );
}

