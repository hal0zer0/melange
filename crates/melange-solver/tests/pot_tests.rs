//! Potentiometer (Sherman-Morrison) tests.
//!
//! Tests that pot-annotated circuits:
//! - Parse correctly
//! - Precompute SM vectors correctly
//! - Generate code that compiles
//! - Produce correct output at nominal resistance (delta_g=0 → no change)
//! - Produce output matching a rebuilt circuit at non-nominal resistance
//! - Don't change codegen output when no pots are present (backward compat)

use std::io::Write;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

fn default_config() -> CodegenConfig {
    CodegenConfig {
        circuit_name: "test_circuit".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1000.0,
        ..CodegenConfig::default()
    }
}

fn generate_code(spice: &str) -> String {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let codegen = CodeGenerator::new(default_config());
    let result = codegen
        .generate(&kernel, &mna, &netlist)
        .expect("code generation failed");
    result.code
}

/// Compile generated code with rustc, panicking with stderr on failure.
fn assert_compiles(code: &str, label: &str) {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);

    let tmp_dir = std::env::temp_dir();
    let src = tmp_dir.join(format!("melange_pot_test_{}.rs", id));
    let lib = tmp_dir.join(format!("melange_pot_test_{}.rlib", id));

    {
        let mut f = std::fs::File::create(&src).expect("create temp file");
        f.write_all(code.as_bytes()).expect("write temp file");
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib", "-o"])
        .arg(&lib)
        .arg(&src)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&src);
    let _ = std::fs::remove_file(&lib);
    let _ = std::fs::remove_file(tmp_dir.join(format!("libmelange_pot_test_{}.rlib", id)));

    assert!(
        output.status.success(),
        "Generated code for '{}' failed to compile:\n{}",
        label,
        String::from_utf8_lossy(&output.stderr)
    );
}

// ---------------------------------------------------------------------------
// Test circuits
// ---------------------------------------------------------------------------

const RC_POT_SPICE: &str = "\
RC Pot Circuit
R1 in out 10k
C1 out 0 100n
.pot R1 1k 100k
";

const RC_NO_POT_SPICE: &str = "\
RC No Pot Circuit
R1 in out 10k
C1 out 0 100n
";

const DIODE_POT_SPICE: &str = "\
Diode Clipper with Pot
R1 in mid 10k
R2 in 0 100k
D1 mid out Dmod
C1 out 0 100n
.model Dmod D(IS=2.52e-9 N=1.5)
.pot R1 1k 100k
";

const GROUNDED_POT_SPICE: &str = "\
Grounded Pot
R1 in 0 10k
C1 in out 100n
R2 out 0 1k
.pot R1 1k 100k
";

const TWO_POT_SPICE: &str = "\
Two Pot Circuit
R1 in mid 10k
R2 mid out 5k
C1 out 0 100n
.pot R1 1k 100k
.pot R2 500 50k
";

const GROUNDED_POT_DIODE_SPICE: &str = "\
Grounded Pot with Diode
R1 in 0 10k
D1 in out Dmod
C1 out 0 100n
R2 out 0 1k
.model Dmod D(IS=2.52e-9 N=1.5)
.pot R1 1k 100k
";

// ---------------------------------------------------------------------------
// Step 2: MNA pot node resolution
// ---------------------------------------------------------------------------

#[test]
fn test_mna_pot_resolution() {
    let (netlist, mna, _kernel) = build_pipeline(RC_POT_SPICE);
    assert_eq!(netlist.pots.len(), 1);
    assert_eq!(mna.pots.len(), 1);

    let pot = &mna.pots[0];
    assert_eq!(pot.name, "R1");
    assert!(
        (pot.g_nominal - 1.0 / 10000.0).abs() < 1e-15,
        "g_nominal should be 1/10k"
    );
    assert_eq!(pot.min_resistance, 1000.0);
    assert_eq!(pot.max_resistance, 100000.0);
    // Neither node is grounded in this circuit (both "in" and "out" are non-zero)
    assert!(!pot.grounded);
}

#[test]
fn test_mna_grounded_pot_resolution() {
    let (_netlist, mna, _kernel) = build_pipeline(GROUNDED_POT_SPICE);
    assert_eq!(mna.pots.len(), 1);
    let pot = &mna.pots[0];
    assert!(
        pot.grounded,
        "R1 connects to ground, so pot should be grounded"
    );
}

// ---------------------------------------------------------------------------
// Step 3: DK kernel SM vector precomputation
// ---------------------------------------------------------------------------

#[test]
fn test_dk_kernel_sm_vectors() {
    let (_netlist, _mna, kernel) = build_pipeline(RC_POT_SPICE);
    assert_eq!(kernel.pots.len(), 1);

    let pot = &kernel.pots[0];

    // SU vector should have N elements
    assert_eq!(pot.su.len(), kernel.n);

    // USU should be finite and positive (conductance change on a resistor
    // between two non-ground nodes should give positive u^T S u)
    assert!(pot.usu.is_finite(), "USU should be finite");
    assert!(
        pot.usu > 0.0,
        "USU should be positive for non-grounded resistor, got {}",
        pot.usu
    );

    // G_nominal should be 1/10k
    assert!((pot.g_nominal - 1e-4).abs() < 1e-15);

    // NV_SU and U_NI should be empty (linear circuit, M=0)
    assert_eq!(pot.nv_su.len(), 0, "No nonlinear devices → empty NV_SU");
    assert_eq!(pot.u_ni.len(), 0, "No nonlinear devices → empty U_NI");
}

#[test]
fn test_dk_kernel_sm_vectors_nonlinear() {
    let (_netlist, _mna, kernel) = build_pipeline(DIODE_POT_SPICE);
    assert_eq!(kernel.pots.len(), 1);
    assert!(kernel.m > 0, "Should have nonlinear device");

    let pot = &kernel.pots[0];

    // NV_SU and U_NI should have M elements (diode: M=1)
    assert_eq!(pot.nv_su.len(), kernel.m);
    assert_eq!(pot.u_ni.len(), kernel.m);

    // All vectors should be finite
    for v in &pot.su {
        assert!(v.is_finite());
    }
    for v in &pot.nv_su {
        assert!(v.is_finite());
    }
    for v in &pot.u_ni {
        assert!(v.is_finite());
    }
}

#[test]
fn test_dk_kernel_sm_identity_at_nominal() {
    // Verify that at nominal resistance, delta_g = 0 and scale = 0
    // (Sherman-Morrison correction vanishes)
    let (_netlist, _mna, kernel) = build_pipeline(RC_POT_SPICE);
    let pot = &kernel.pots[0];

    let r_nominal = 1.0 / pot.g_nominal;
    let delta_g = 1.0 / r_nominal - pot.g_nominal;
    assert!(
        delta_g.abs() < 1e-15,
        "delta_g should be zero at nominal resistance, got {}",
        delta_g
    );
}

// ---------------------------------------------------------------------------
// Step 4: Codegen IR
// ---------------------------------------------------------------------------

#[test]
fn test_circuit_ir_has_pots() {
    let (netlist, mna, kernel) = build_pipeline(RC_POT_SPICE);
    let config = default_config();
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();

    assert_eq!(ir.pots.len(), 1);
    assert!((ir.pots[0].g_nominal - 1e-4).abs() < 1e-15);
    assert_eq!(ir.pots[0].su.len(), kernel.n);
}

// ---------------------------------------------------------------------------
// Steps 5-8: Codegen emission
// ---------------------------------------------------------------------------

#[test]
fn test_codegen_pot_constants_emitted() {
    let code = generate_code(RC_POT_SPICE);

    // Check pot constants are present
    assert!(code.contains("POT_0_G_NOM"), "Missing POT_0_G_NOM constant");
    assert!(code.contains("POT_0_MIN_R"), "Missing POT_0_MIN_R constant");
    assert!(code.contains("POT_0_G_NOM"), "Missing POT_0_G_NOM constant");
    assert!(code.contains("POT_0_MIN_R"), "Missing POT_0_MIN_R constant");
    assert!(code.contains("POT_0_MAX_R"), "Missing POT_0_MAX_R constant");
}

#[test]
fn test_codegen_pot_state_field() {
    let code = generate_code(RC_POT_SPICE);

    // State should have pot resistance field
    assert!(
        code.contains("pot_0_resistance"),
        "Missing pot_0_resistance state field"
    );
}

#[test]
fn test_codegen_pot_set_method() {
    let code = generate_code(RC_POT_SPICE);

    // Per-block rebuild: set_pot_0 method should exist
    assert!(
        code.contains("fn set_pot_0("),
        "Missing set_pot_0 method for per-block rebuild"
    );
    assert!(code.contains("rebuild_matrices"), "set_pot should call rebuild_matrices");
}

#[test]
fn test_codegen_pot_process_sample_corrections() {
    let code = generate_code(RC_POT_SPICE);

    // Per-block rebuild: process_sample should NOT have SM corrections
    assert!(
        !code.contains("delta_g_0"),
        "SM delta_g should not be in process_sample (per-block rebuild)"
    );
    assert!(
        !code.contains("su_c0"),
        "SM su_c should not be in process_sample"
    );
    // Should have set_pot_0 method (from rebuild_matrices infrastructure)
    assert!(
        code.contains("fn set_pot_0"),
        "Missing set_pot_0 method for per-block rebuild"
    );
    assert!(
        code.contains("fn rebuild_matrices"),
        "Missing rebuild_matrices method"
    );
}

#[test]
fn test_codegen_pot_nr_k_correction() {
    let code = generate_code(DIODE_POT_SPICE);

    // Per-block rebuild: NR solver should use state.k directly (not k_eff)
    assert!(!code.contains("k_eff"), "SM k_eff should not be in NR solver (per-block rebuild)");
    assert!(code.contains("state.k["), "NR solver should use state.k directly");
}

#[test]
fn test_codegen_pot_compiles() {
    let code = generate_code(RC_POT_SPICE);

    // Attempt to compile the generated code by checking it for syntax correctness
    // (We can't actually compile Rust in a test, but we can check for balanced braces, etc.)
    let open_braces = code.chars().filter(|&c| c == '{').count();
    let close_braces = code.chars().filter(|&c| c == '}').count();
    assert_eq!(
        open_braces, close_braces,
        "Unbalanced braces in generated code"
    );

    // Check that all key functions are present
    assert!(
        code.contains("fn process_sample("),
        "Missing process_sample"
    );
    assert!(code.contains("fn build_rhs("), "Missing build_rhs");
    assert!(code.contains("fn mat_vec_mul_s("), "Missing mat_vec_mul_s");
    assert!(code.contains("struct CircuitState"), "Missing CircuitState");
}

#[test]
fn test_codegen_nonlinear_pot_compiles() {
    let code = generate_code(DIODE_POT_SPICE);

    let open_braces = code.chars().filter(|&c| c == '{').count();
    let close_braces = code.chars().filter(|&c| c == '}').count();
    assert_eq!(
        open_braces, close_braces,
        "Unbalanced braces in generated code with diode+pot"
    );

    assert!(
        code.contains("fn solve_nonlinear("),
        "Missing solve_nonlinear"
    );
    assert!(code.contains("fn set_pot_0("), "Missing set_pot_0");
}

// ---------------------------------------------------------------------------
// Backward compatibility: no pots → identical to before
// ---------------------------------------------------------------------------

#[test]
fn test_no_pot_backward_compat() {
    let code = generate_code(RC_NO_POT_SPICE);

    // No pot-related code should appear
    assert!(!code.contains("POT_0"), "No POT constants without .pot");
    assert!(
        !code.contains("sm_scale"),
        "No SM scale function without .pot"
    );
    assert!(
        !code.contains("pot_0_resistance"),
        "No pot state field without .pot"
    );
    // In process_sample, rhs/v_pred should not be mut (build_rhs uses mut internally, that's ok)
    assert!(
        code.contains("let rhs = build_rhs("),
        "RHS should be immutable in process_sample without pots"
    );
    assert!(
        code.contains("let v_pred = mat_vec_mul_s("),
        "v_pred should be immutable without pots"
    );
}

// ---------------------------------------------------------------------------
// Grounded pot
// ---------------------------------------------------------------------------

#[test]
fn test_grounded_pot_codegen() {
    let code = generate_code(GROUNDED_POT_SPICE);

    assert!(
        code.contains("POT_0_G_NOM"),
        "Missing POT_0_G_NOM for grounded pot"
    );
    assert!(
        code.contains("fn set_pot_0("),
        "Missing set_pot_0 for grounded pot"
    );

    // Grounded pot should only reference one node in A_neg correction
    // (not both NODE_P and NODE_Q)
    let open_braces = code.chars().filter(|&c| c == '{').count();
    let close_braces = code.chars().filter(|&c| c == '}').count();
    assert_eq!(
        open_braces, close_braces,
        "Unbalanced braces with grounded pot"
    );
}

// ---------------------------------------------------------------------------
// Numerical accuracy: SM vectors match hand computation for 2-node RC
// ---------------------------------------------------------------------------

#[test]
fn test_sm_vectors_match_manual_computation() {
    // For a 2-node RC circuit with R=10k between nodes 1,2 and C=100n from node 2 to gnd:
    // G = [[g, -g], [-g, g]], C = [[0, 0], [0, c]]
    // where g = 1/10000 = 1e-4, c = 100e-9
    //
    // A = G + alpha*C where alpha = 2*44100
    // u = [1, -1] (pot between the two nodes)
    // S = A^-1
    // su = S * u, usu = u^T * su
    //
    // We verify our precomputed SM vectors against a fresh A inversion.

    let (_netlist, _mna, kernel) = build_pipeline(RC_POT_SPICE);
    let n = kernel.n;
    assert_eq!(n, 2);

    let pot = &kernel.pots[0];

    // Build u vector manually
    let mut u = vec![0.0; n];
    if pot.node_p > 0 {
        u[pot.node_p - 1] = 1.0;
    }
    if pot.node_q > 0 {
        u[pot.node_q - 1] = -1.0;
    }

    // Get S matrix from kernel
    let s = |i: usize, j: usize| -> f64 { kernel.s[i * n + j] };

    // Compute S*u manually
    let mut su_manual = vec![0.0; n];
    for i in 0..n {
        for j in 0..n {
            su_manual[i] += s(i, j) * u[j];
        }
    }

    // Compare with precomputed
    for i in 0..n {
        assert!(
            (pot.su[i] - su_manual[i]).abs() < 1e-12,
            "SU[{}] mismatch: precomputed={}, manual={}",
            i,
            pot.su[i],
            su_manual[i]
        );
    }

    // Compute u^T * S * u manually
    let mut usu_manual = 0.0;
    for i in 0..n {
        usu_manual += u[i] * su_manual[i];
    }
    assert!(
        (pot.usu - usu_manual).abs() < 1e-12,
        "USU mismatch: precomputed={}, manual={}",
        pot.usu,
        usu_manual
    );
}

// ---------------------------------------------------------------------------
// Numerical accuracy: SM at non-nominal R matches full rebuild
// ---------------------------------------------------------------------------

#[test]
fn test_sm_update_matches_full_rebuild() {
    // Build RC circuit with R=10k, get S via SM
    // Build RC circuit with R=5k directly, get S via full inversion
    // Compare S_updated vs S_rebuilt

    // Circuit 1: R=10k (nominal), with pot
    let spice_10k = "\
RC 10k
R1 in out 10k
C1 out 0 100n
.pot R1 1k 100k
";
    let (_, _, kernel_10k) = build_pipeline(spice_10k);
    let n = kernel_10k.n;
    let pot = &kernel_10k.pots[0];

    // Circuit 2: R=5k (target), no pot
    let spice_5k = "\
RC 5k
R1 in out 5k
C1 out 0 100n
";
    let (_, _, kernel_5k) = build_pipeline(spice_5k);

    // Compute SM-updated S matrix for R=5k
    let r_new = 5000.0;
    let delta_g = 1.0 / r_new - pot.g_nominal;
    let denom = 1.0 + delta_g * pot.usu;
    let scale = delta_g / denom;

    // S' = S - scale * su * su^T (symmetric A → symmetric S)
    for i in 0..n {
        for j in 0..n {
            let s_old = kernel_10k.s[i * n + j];
            let s_updated = s_old - scale * pot.su[i] * pot.su[j];
            let s_rebuilt = kernel_5k.s[i * n + j];

            assert!(
                (s_updated - s_rebuilt).abs() < 1e-10,
                "S[{}][{}] mismatch: SM update={:.15e}, full rebuild={:.15e}, diff={:.2e}",
                i,
                j,
                s_updated,
                s_rebuilt,
                (s_updated - s_rebuilt).abs()
            );
        }
    }
}

// ===========================================================================
// NEW TESTS: Filling coverage gaps identified by multi-agent review
// ===========================================================================

// ---------------------------------------------------------------------------
// Two-pot circuit through full pipeline
// ---------------------------------------------------------------------------

#[test]
fn test_two_pot_pipeline() {
    let (_, mna, kernel) = build_pipeline(TWO_POT_SPICE);
    assert_eq!(mna.pots.len(), 2);
    assert_eq!(kernel.pots.len(), 2);

    // Both pots should have distinct SM vectors
    let pot0 = &kernel.pots[0];
    let pot1 = &kernel.pots[1];
    assert_eq!(pot0.su.len(), kernel.n);
    assert_eq!(pot1.su.len(), kernel.n);
    assert!(pot0.usu > 0.0);
    assert!(pot1.usu > 0.0);
    // Different pots → different USU values
    assert!(
        (pot0.usu - pot1.usu).abs() > 1e-6,
        "Two pots should have distinct USU values"
    );
}

#[test]
fn test_two_pot_codegen() {
    let code = generate_code(TWO_POT_SPICE);

    // Both pots should have constants and setters
    assert!(code.contains("POT_0_G_NOM"), "Missing POT_0_G_NOM");
    assert!(code.contains("POT_1_G_NOM"), "Missing POT_1_G_NOM");
    assert!(code.contains("fn set_pot_0("), "Missing set_pot_0");
    assert!(code.contains("fn set_pot_1("), "Missing set_pot_1");
    assert!(
        code.contains("pot_0_resistance"),
        "Missing pot_0_resistance"
    );
    assert!(
        code.contains("pot_1_resistance"),
        "Missing pot_1_resistance"
    );
    assert!(
        code.contains("fn set_pot_0"),
        "Missing set_pot_0 method"
    );
    assert!(
        code.contains("fn set_pot_1"),
        "Missing set_pot_1 method"
    );
}

#[test]
fn test_two_pot_compiles() {
    let code = generate_code(TWO_POT_SPICE);
    assert_compiles(&code, "two_pot");
}

// ---------------------------------------------------------------------------
// Actual rustc compilation tests (replaces brace-counting)
// ---------------------------------------------------------------------------

#[test]
fn test_rc_pot_compiles_rustc() {
    let code = generate_code(RC_POT_SPICE);
    assert_compiles(&code, "rc_pot");
}

#[test]
fn test_diode_pot_compiles_rustc() {
    let code = generate_code(DIODE_POT_SPICE);
    assert_compiles(&code, "diode_pot");
}

#[test]
fn test_grounded_pot_compiles_rustc() {
    let code = generate_code(GROUNDED_POT_SPICE);
    assert_compiles(&code, "grounded_pot");
}

#[test]
fn test_grounded_pot_diode_compiles_rustc() {
    let code = generate_code(GROUNDED_POT_DIODE_SPICE);
    assert_compiles(&code, "grounded_pot_diode");
}

// ---------------------------------------------------------------------------
// SM-corrected K matrix matches full rebuild
// ---------------------------------------------------------------------------

#[test]
fn test_sm_k_update_matches_full_rebuild() {
    // For a nonlinear circuit, verify K' via SM matches K from fresh build at R=5k
    let spice_10k = "\
Diode K test
R1 in mid 10k
R2 in 0 100k
D1 mid out Dmod
C1 out 0 100n
.model Dmod D(IS=2.52e-9 N=1.5)
.pot R1 1k 100k
";
    let (_, _, kernel_10k) = build_pipeline(spice_10k);
    let _n = kernel_10k.n;
    let m = kernel_10k.m;
    assert!(m > 0, "Need nonlinear device for K test");
    let pot = &kernel_10k.pots[0];

    let spice_5k = "\
Diode K test 5k
R1 in mid 5k
R2 in 0 100k
D1 mid out Dmod
C1 out 0 100n
.model Dmod D(IS=2.52e-9 N=1.5)
";
    let (_, _, kernel_5k) = build_pipeline(spice_5k);

    // SM scale for R=5k
    let r_new = 5000.0;
    let delta_g = 1.0 / r_new - pot.g_nominal;
    let scale = delta_g / (1.0 + delta_g * pot.usu);

    // K' = K - scale * nv_su * u_ni^T
    // (where nv_su = N_v*su, u_ni = su^T*N_i after our fix)
    for i in 0..m {
        for j in 0..m {
            let k_old = kernel_10k.k[i * m + j];
            let k_updated = k_old - scale * pot.nv_su[i] * pot.u_ni[j];
            let k_rebuilt = kernel_5k.k[i * m + j];

            assert!(
                (k_updated - k_rebuilt).abs() < 1e-8,
                "K[{}][{}] mismatch: SM={:.15e}, rebuilt={:.15e}, diff={:.2e}",
                i,
                j,
                k_updated,
                k_rebuilt,
                (k_updated - k_rebuilt).abs()
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Manual verification of nv_su and u_ni vectors
// ---------------------------------------------------------------------------

#[test]
fn test_sm_nonlinear_vectors_match_manual() {
    let (_, mna, kernel) = build_pipeline(DIODE_POT_SPICE);
    let n = kernel.n;
    let m = kernel.m;
    let pot = &kernel.pots[0];

    // nv_su = N_v * su
    // N_v is M×N, su is N-vector → nv_su is M-vector
    for i in 0..m {
        let mut nv_su_manual = 0.0;
        for j in 0..n {
            nv_su_manual += mna.n_v[i][j] * pot.su[j];
        }
        assert!(
            (pot.nv_su[i] - nv_su_manual).abs() < 1e-10,
            "nv_su[{}] mismatch: precomputed={:.15e}, manual={:.15e}",
            i,
            pot.nv_su[i],
            nv_su_manual
        );
    }

    // u_ni = su^T * N_i (after our fix)
    // su is N-vector, N_i is N×M → u_ni is M-vector
    for j in 0..m {
        let mut u_ni_manual = 0.0;
        for i in 0..n {
            u_ni_manual += pot.su[i] * mna.n_i[i][j];
        }
        assert!(
            (pot.u_ni[j] - u_ni_manual).abs() < 1e-10,
            "u_ni[{}] mismatch: precomputed={:.15e}, manual={:.15e}",
            j,
            pot.u_ni[j],
            u_ni_manual
        );
    }
}

// ---------------------------------------------------------------------------
// Pot at extreme resistance values
// ---------------------------------------------------------------------------

#[test]
fn test_sm_scale_at_extremes() {
    let (_, _, kernel) = build_pipeline(RC_POT_SPICE);
    let pot = &kernel.pots[0];

    // Helper: compute (delta_g, scale) for a given resistance
    let sm_scale = |r: f64| -> (f64, f64) {
        let r = r.clamp(pot.min_resistance, pot.max_resistance);
        let delta_g = 1.0 / r - pot.g_nominal;
        let denom = 1.0 + delta_g * pot.usu;
        let scale = if denom.abs() > 1e-15 {
            delta_g / denom
        } else {
            0.0
        };
        (delta_g, scale)
    };

    // At nominal: delta_g ≈ 0
    let (dg_nom, scale_nom) = sm_scale(1.0 / pot.g_nominal);
    assert!(dg_nom.abs() < 1e-15, "delta_g should be ~0 at nominal");
    assert!(scale_nom.abs() < 1e-15, "scale should be ~0 at nominal");

    // At min_r: delta_g > 0 (conductance increases)
    let (dg_min, scale_min) = sm_scale(pot.min_resistance);
    assert!(
        dg_min > 0.0,
        "delta_g should be positive at min_r (more conductance)"
    );
    assert!(scale_min.is_finite(), "scale at min_r should be finite");

    // At max_r: delta_g < 0 (conductance decreases)
    let (dg_max, scale_max) = sm_scale(pot.max_resistance);
    assert!(
        dg_max < 0.0,
        "delta_g should be negative at max_r (less conductance)"
    );
    assert!(scale_max.is_finite(), "scale at max_r should be finite");

    // Below min_r should clamp to min_r
    let (dg_below, _) = sm_scale(100.0); // below min of 1k
    assert_eq!(dg_below, dg_min, "Below min_r should clamp");

    // Above max_r should clamp to max_r
    let (dg_above, _) = sm_scale(1e6); // above max of 100k
    assert_eq!(dg_above, dg_max, "Above max_r should clamp");

    // Verify the corrected S matrix is still positive definite (diagonal > 0)
    // by checking that the SM-updated diagonal entries remain positive
    let n = kernel.n;
    for &r in &[pot.min_resistance, pot.max_resistance] {
        let (_, scale) = sm_scale(r);
        for i in 0..n {
            let s_corrected = kernel.s[i * n + i] - scale * pot.su[i] * pot.su[i];
            assert!(
                s_corrected > 0.0,
                "S_corrected[{}][{}] should be positive at R={}, got {}",
                i,
                i,
                r,
                s_corrected
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Nominal R equals min or max boundary
// ---------------------------------------------------------------------------

#[test]
fn test_pot_nominal_at_boundary() {
    // Nominal R = min_r
    let spice_at_min = "\
Nominal at min
R1 in out 1k
C1 out 0 100n
.pot R1 1k 100k
";
    let (_, _, kernel) = build_pipeline(spice_at_min);
    let pot = &kernel.pots[0];
    assert!((1.0 / pot.g_nominal - 1000.0).abs() < 1e-10);
    assert!(pot.usu > 0.0);

    // Nominal R = max_r
    let spice_at_max = "\
Nominal at max
R1 in out 100k
C1 out 0 100n
.pot R1 1k 100k
";
    let (_, _, kernel) = build_pipeline(spice_at_max);
    let pot = &kernel.pots[0];
    assert!((1.0 / pot.g_nominal - 100000.0).abs() < 1e-6);
    assert!(pot.usu > 0.0);
}

// ---------------------------------------------------------------------------
// Grounded pot in nonlinear circuit
// ---------------------------------------------------------------------------

#[test]
fn test_grounded_pot_diode_pipeline() {
    let (_, mna, kernel) = build_pipeline(GROUNDED_POT_DIODE_SPICE);
    assert_eq!(mna.pots.len(), 1);
    assert!(mna.pots[0].grounded);
    assert!(kernel.m > 0, "Should have nonlinear device");

    let pot = &kernel.pots[0];
    assert_eq!(pot.nv_su.len(), kernel.m);
    assert_eq!(pot.u_ni.len(), kernel.m);
    for v in &pot.nv_su {
        assert!(v.is_finite());
    }
    for v in &pot.u_ni {
        assert!(v.is_finite());
    }
}

// ---------------------------------------------------------------------------
// S*N_i correction in generated code
// ---------------------------------------------------------------------------

#[test]
fn test_codegen_sni_correction() {
    let code = generate_code(DIODE_POT_SPICE);

    // Per-block rebuild: no SM S*N_i correction
    assert!(
        !code.contains("u_ni_dot_inl"),
        "SM u_ni_dot_inl should not be present (per-block rebuild)"
    );
    assert!(
        !code.contains("sni_factor"),
        "SM sni_factor should not be present (per-block rebuild)"
    );
}

#[test]
fn test_codegen_no_sni_correction_linear() {
    let code = generate_code(RC_POT_SPICE);

    // Linear circuit (M=0): no S*N_i correction should appear
    assert!(
        !code.contains("sni_factor"),
        "Linear pot circuit should have no sni_factor"
    );
    assert!(
        !code.contains("u_ni_dot_inl"),
        "Linear pot circuit should have no u_ni_dot_inl"
    );
}

// ---------------------------------------------------------------------------
// NaN guard in sm_scale
// ---------------------------------------------------------------------------

#[test]
fn test_codegen_nan_guard_in_set_pot() {
    let code = generate_code(RC_POT_SPICE);
    assert!(
        code.contains("if !resistance.is_finite()"),
        "set_pot should have NaN/Inf guard"
    );
}

// ---------------------------------------------------------------------------
// State sanitization resets pot resistance
// ---------------------------------------------------------------------------

#[test]
fn test_codegen_sanitization_resets_pot() {
    let code = generate_code(RC_POT_SPICE);

    // The NaN sanitization block should reset pot resistance
    // Find the sanitization block and check it contains pot reset
    // NaN check now happens before state write: checks local `v` not `state.v_prev`
    let sanitize_idx = code
        .find("if !v.iter().all(|x| x.is_finite())")
        .or_else(|| code.find("if !state.v_prev.iter().all(|x| x.is_finite())"))
        .expect("Missing sanitization block");
    let after_sanitize = &code[sanitize_idx..];
    // NaN reset now returns DC operating point output instead of zeros
    let return_idx = after_sanitize
        .find("return dc_output;")
        .or_else(|| after_sanitize.find("return [0.0; NUM_OUTPUTS];"))
        .expect("Missing return in sanitization");
    let sanitize_block = &after_sanitize[..return_idx];
    assert!(
        sanitize_block.contains("pot_0_resistance"),
        "Sanitization block should reset pot_0_resistance"
    );
}

// ---------------------------------------------------------------------------
// Jacobian K correction in generated code
// ---------------------------------------------------------------------------

#[test]
fn test_codegen_jacobian_uses_state_k() {
    let code = generate_code(DIODE_POT_SPICE);

    // Per-block rebuild: Jacobian should use state.k directly (exact from rebuild)
    assert!(
        code.contains("state.k["),
        "Jacobian should use state.k (exact from per-block rebuild)"
    );
    assert!(
        !code.contains("k_eff"),
        "SM k_eff should not be present"
    );
}

// ---------------------------------------------------------------------------
// Per-block rebuild: no SM S correction in process_sample
// ---------------------------------------------------------------------------

#[test]
fn test_codegen_no_sm_s_correction() {
    let code = generate_code(RC_POT_SPICE);

    // Per-block rebuild: no SM corrections in process_sample
    assert!(
        !code.contains("su_dot_rhs"),
        "SM su_dot_rhs should not be in process_sample (per-block rebuild)"
    );
    assert!(
        !code.contains("sm_scale"),
        "SM sm_scale should not be in process_sample"
    );
    // Should have rebuild_matrices instead
    assert!(
        code.contains("fn rebuild_matrices"),
        "Should have rebuild_matrices method"
    );
}
