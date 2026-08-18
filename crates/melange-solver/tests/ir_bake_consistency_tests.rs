//! Regression tests for the 2026-07 IR-bake / stability-analysis fix batch:
//!
//! 1. BE + oversampling must bake backward-Euler matrices at the internal
//!    rate (previously the os>1 branch unconditionally baked trapezoidal
//!    matrices while the emitter used BE semantics — mixed integrator,
//!    wrong DC fixed point, silent convention swap on first rebuild).
//! 2. The DK auto-BE discriminator must evaluate the internal-rate
//!    (S, A_neg) pair it actually ships, not the base-rate kernel pair.
//! 3. A_NEG_SUB must blanket-zero ALL augmented algebraic rows at bake,
//!    matching the runtime rebuild (per-type enumeration missed
//!    current-mode-VCA and behavioral-V rows).
//! 5. The DK path must NOT rail-clamp the baked DC_OP against unclamped
//!    DC_NL_I (v_prev / i_nl_prev consistency — nodal-path policy).
//! 6. Inverted op-amp rails (VCC <= VEE, both finite) must hard-error at
//!    MNA resolve time instead of panicking f64::clamp on the audio thread.
//! 7. `generate()` must validate input_node against kernel.n_nodes, not
//!    kernel.n (aug rows are not valid inputs).
//! 9. CodegenMeta sparse-LU fields must be populated from the IR.

mod support;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::{CodeGenerator, CodegenConfig, CodegenError, OpampRailMode};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ── Shared pipeline helpers ─────────────────────────────────────────────

fn build_pipeline(spice: &str, config: &CodegenConfig) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("parse netlist");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("build MNA");
    // Standard CLI pipeline: stamp input conductance before kernel build.
    let g_in = 1.0 / config.input_resistance;
    if config.input_node < mna.n {
        mna.g[config.input_node][config.input_node] += g_in;
    }
    let kernel = DkKernel::from_mna(&mna, config.sample_rate).expect("build DK kernel");
    (netlist, mna, kernel)
}

fn build_ir(spice: &str, config: &CodegenConfig) -> CircuitIR {
    let (netlist, mna, kernel) = build_pipeline(spice, config);
    CircuitIR::from_kernel_with_dc_op(&kernel, &mna, &netlist, config, None).expect("build IR")
}

/// Dense Gauss-Jordan inverse for small test matrices (flat row-major).
fn invert_dense(a: &[f64], n: usize) -> Vec<f64> {
    let mut m = vec![0.0f64; n * 2 * n];
    for i in 0..n {
        for j in 0..n {
            m[i * 2 * n + j] = a[i * n + j];
        }
        m[i * 2 * n + n + i] = 1.0;
    }
    for col in 0..n {
        // Partial pivot
        let mut piv = col;
        for r in col + 1..n {
            if m[r * 2 * n + col].abs() > m[piv * 2 * n + col].abs() {
                piv = r;
            }
        }
        if piv != col {
            for j in 0..2 * n {
                m.swap(col * 2 * n + j, piv * 2 * n + j);
            }
        }
        let d = m[col * 2 * n + col];
        assert!(d.abs() > 1e-300, "singular test matrix");
        for j in 0..2 * n {
            m[col * 2 * n + j] /= d;
        }
        for r in 0..n {
            if r == col {
                continue;
            }
            let f = m[r * 2 * n + col];
            if f != 0.0 {
                for j in 0..2 * n {
                    m[r * 2 * n + j] -= f * m[col * 2 * n + j];
                }
            }
        }
    }
    let mut out = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            out[i * n + j] = m[i * 2 * n + n + j];
        }
    }
    out
}

// ── Fix 1: BE + oversampling bakes BE matrices at the internal rate ─────

/// Diode DC-bias circuit: VCC through R into a diode+cap node, plus a DC
/// current source so the trap-vs-BE rhs_const convention (×2 vs ×1 on node
/// rows) is observable.
const DIODE_BIAS_SPICE: &str = "\
BE OS bake diode bias
V1 vcc 0 DC 5
R1 vcc a 1k
I1 0 a DC 1m
D1 a 0 DCLIP
C1 a 0 100n
R2 in a 100k
.model DCLIP D(IS=1e-14)
";

fn be_os_config(os: usize) -> CodegenConfig {
    CodegenConfig {
        circuit_name: format!("be_os{os}_bake"),
        sample_rate: 44100.0,
        input_node: 2, // node "in" (vcc=0, a=1, in=2)
        output_nodes: vec![1],
        input_resistance: 1.0,
        oversampling_factor: os,
        backward_euler: true,
        dc_block: false,
        ..CodegenConfig::default()
    }
}

#[test]
fn be_with_oversampling_bakes_be_matrices_at_internal_rate() {
    let config = be_os_config(2);
    let ir = build_ir(DIODE_BIAS_SPICE, &config);
    let n = ir.topology.n;
    let n_nodes = ir.topology.n_nodes;
    let n_aug = ir.topology.n_aug;
    assert!(ir.solver_config.backward_euler);

    let internal_rate = 44100.0 * 2.0;
    let alpha_be = internal_rate; // BE: alpha = 1/T at the INTERNAL rate
    assert_eq!(
        ir.solver_config.alpha, alpha_be,
        "solver_config.alpha must be the internal-rate BE alpha"
    );

    // A_NEG must be the BE history matrix (1/T_int)·C with algebraic rows
    // zeroed — NOT the trap alpha·C − G the old os>1 branch baked.
    for i in 0..n {
        for j in 0..n {
            let expected = if i >= n_nodes && i < n_aug {
                0.0
            } else {
                alpha_be * ir.matrices.c_matrix[i * n + j]
            };
            let got = ir.matrices.a_neg[i * n + j];
            assert!(
                (got - expected).abs() <= 1e-9 * expected.abs().max(1.0),
                "A_neg[{i}][{j}] = {got}, expected BE form {expected} \
                 (old bug: trap form alpha*C - G with alpha = 2/T)"
            );
        }
    }

    // S must be (G + (1/T_int)·C)^{-1}.
    let mut a_be = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            a_be[i * n + j] =
                ir.matrices.g_matrix[i * n + j] + alpha_be * ir.matrices.c_matrix[i * n + j];
        }
    }
    let s_expected = invert_dense(&a_be, n);
    for i in 0..n * n {
        assert!(
            (ir.matrices.s[i] - s_expected[i]).abs() <= 1e-6 * s_expected[i].abs().max(1e-6),
            "S[{i}] = {}, expected BE-at-internal-rate inverse {}",
            ir.matrices.s[i],
            s_expected[i]
        );
    }

    // rhs_const must use BE ×1 semantics: the 1 mA DC current source
    // injects 0.001 at node `a` (trap baked 0.002), and the VS row keeps 5.
    let (_, mna, _) = build_pipeline(DIODE_BIAS_SPICE, &config);
    let a_idx = *mna.node_map.get("a").unwrap() - 1;
    assert!(
        (ir.matrices.rhs_const[a_idx].abs() - 0.001).abs() < 1e-12,
        "rhs_const[a] = {} — BE bake must use current sources x1 (|0.001|), not the trap x2",
        ir.matrices.rhs_const[a_idx]
    );
    let vs_row = n_nodes + mna.voltage_sources[0].ext_idx;
    assert!(
        (ir.matrices.rhs_const[vs_row] - 5.0).abs() < 1e-12,
        "rhs_const[VS row] must be the 5 V constraint"
    );

    // Primary integrator is BE — no BE-fallback set is baked (mirrors os=1).
    assert!(ir.matrices.s_be.is_empty());
    assert!(ir.matrices.a_neg_be.is_empty());
}

/// Compile-and-run: the BE + os=2 build must settle to the SAME nonlinear
/// DC fixed point as the BE + os=1 build (BE's fixed point satisfies
/// G·v = I_dc + N_i·i_nl — rate-independent). The old mixed-integrator
/// bake shipped trap matrices against BE update equations, which halves
/// the effective nonlinear bias current and shifts the diode node by tens
/// of mV.
#[test]
fn be_os2_dc_fixed_point_matches_be_os1() {
    let main_code = "\
fn main() {
    let mut state = CircuitState::default();
    let mut y = [0.0f64; NUM_OUTPUTS];
    for _ in 0..8000 {
        y = process_sample(0.0, &mut state);
    }
    println!(\"dc={}\", y[0]);
}
";
    let mut dc = [0.0f64; 2];
    for (slot, os) in [(0usize, 1usize), (1, 2)] {
        let config = be_os_config(os);
        let (code, _, _) = support::generate_circuit_code(DIODE_BIAS_SPICE, &config);
        let out = support::compile_and_run(&code, main_code, &format!("be_os{os}_dcfp"));
        dc[slot] = out.parse_kv("dc").expect("dc= line");
    }
    // Sanity: the diode node carries a real forward bias (~0.6-0.8 V).
    assert!(
        dc[0] > 0.4 && dc[0] < 1.0,
        "os=1 BE DC fixed point should be a forward-biased diode node, got {}",
        dc[0]
    );
    assert!(
        (dc[0] - dc[1]).abs() < 1e-6,
        "BE DC fixed point must be oversampling-invariant: os1={} os2={} (delta {:.3e} V; \
         the old mixed-integrator bake shifted this by ~tens of mV)",
        dc[0],
        dc[1],
        (dc[0] - dc[1]).abs()
    );
}

// ── Fix 2: discriminator evaluates the shipped internal-rate pair ───────

#[test]
fn dk_discriminator_evaluates_internal_rate_pair_under_oversampling() {
    use melange_solver::codegen::stability::analyze_trap_stability_deflated;

    let config = CodegenConfig {
        circuit_name: "os2_trap_disc".to_string(),
        sample_rate: 44100.0,
        input_node: 2, // node "in"
        output_nodes: vec![1],
        input_resistance: 1.0,
        oversampling_factor: 2,
        ..CodegenConfig::default()
    };
    let (netlist, mna, kernel) = build_pipeline(DIODE_BIAS_SPICE, &config);
    let ir = CircuitIR::from_kernel_with_dc_op(&kernel, &mna, &netlist, &config, None)
        .expect("build IR");

    // This circuit stays on trap, so the shipped (S, A_neg) IS the
    // internal-rate trap pair. The recorded discriminator rho must match a
    // re-run of the analyzer on exactly those matrices...
    assert!(
        !ir.solver_config.backward_euler,
        "test circuit must stay trapezoidal for this check"
    );
    let n = ir.topology.n;
    let shipped =
        analyze_trap_stability_deflated(&ir.matrices.s, &ir.matrices.a_neg, n, &[config.input_node]);
    assert!(
        (ir.trap_discriminator_rho - shipped.rho).abs() < 1e-12,
        "discriminator rho {} must equal rho of the shipped internal-rate pair {}",
        ir.trap_discriminator_rho,
        shipped.rho
    );

    // ...and must NOT be the base-rate kernel rho (rho is rate-dependent;
    // if these coincide the discriminator is evaluating the wrong pair).
    let base = analyze_trap_stability_deflated(&kernel.s, &kernel.a_neg, n, &[config.input_node]);
    assert!(
        (base.rho - shipped.rho).abs() > 1e-8,
        "test premise: base-rate rho ({}) must differ measurably from internal-rate rho ({})",
        base.rho,
        shipped.rho
    );
    assert!(
        (ir.trap_discriminator_rho - base.rho).abs() > 1e-8,
        "discriminator must not evaluate the base-rate kernel pair (rho {})",
        base.rho
    );
}

// ── Fix 3: A_NEG_SUB blanket-zeroes all augmented algebraic rows ────────

#[test]
fn a_neg_sub_bake_zeroes_vca_and_behavioral_aug_rows() {
    // VS + VCVS + current-mode VCA (internal node + sense branch) +
    // behavioral V={} row — the per-type enumeration only caught the first
    // two classes. Mirrors mna_stamp_regression_tests'
    // `a_neg_zeroes_all_augmented_rows_including_vca_and_behavioral`, but
    // asserts the BAKED sub-step matrix (ActiveSetBe pin-and-resolve path).
    let spice = "\
Aneg sub blanket zero
V1 vcc 0 DC 15
E1 e_out 0 in 0 2.0
Rin in 0 10k
Rvcc vcc 0 100k
Re e_out 0 10k
Rdrv in sigp 10k
Y1 sigp sigm cv 0 VCAM
Rterm sigm 0 10k
Rcv cv 0 10k
Csig sigm 0 10n
B1 bx 0 V={ tanh(V(in)) }
Rb bx 0 1k
.model VCAM VCA(MODE=1)
";
    let netlist = Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA");
    assert_eq!(
        mna.n_aug,
        mna.n + 5,
        "expected 5 augmented rows (VS + VCVS + VCA internal + VCA sense + behavioral-V)"
    );

    let config = CodegenConfig {
        circuit_name: "aneg_sub_zero".to_string(),
        sample_rate: 48000.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        opamp_rail_mode: OpampRailMode::ActiveSetBe,
        ..CodegenConfig::default()
    };
    let ir = CircuitIR::from_mna(&mna, &netlist, &config).expect("nodal IR");
    let n = ir.topology.n;
    let n_nodes = ir.topology.n_nodes;
    let n_aug = ir.topology.n_aug;
    assert!(!ir.matrices.a_neg_sub.is_empty(), "sub-step matrices baked");

    for row in n_nodes..n_aug {
        for j in 0..n {
            let v = ir.matrices.a_neg_sub[row * n + j];
            assert!(
                v == 0.0,
                "A_NEG_SUB[{row}][{j}] = {v} — augmented algebraic row must be \
                 blanket-zeroed at bake (VCA internal/sense and behavioral-V rows \
                 were previously missed, disagreeing with the first runtime rebuild)"
            );
        }
    }
    // The forward sub-step matrix keeps the constraint stamps (sanity that
    // the rows aren't trivially empty in the system).
    let mut any_constraint = false;
    for row in n_nodes..n_aug {
        for j in 0..n {
            if ir.matrices.a_matrix[row * n + j] != 0.0 {
                any_constraint = true;
            }
        }
    }
    assert!(
        any_constraint,
        "aug rows should carry constraint stamps in A"
    );
}

// ── Fix 5: DK bake must not rail-clamp DC_OP against unclamped DC_NL_I ──

#[test]
fn dk_dc_op_bake_not_rail_clamped() {
    // Open-loop op-amp: +1 V differential input × AOL — the DC OP output
    // sits far beyond the ±9 V rails. The DK bake used to clamp the stored
    // DC_OP to the rails while DC_NL_I stayed unclamped (inconsistent
    // v_prev / i_nl_prev pair — the exact 4kbuscomp slow-drift signature
    // the nodal path deliberately avoids).
    let spice = "\
DK dc-op no clamp
V1 vp 0 DC 1
U1 vp 0 oout OAX
Rload oout 0 10k
Rd oout dx 1k
D1 dx 0 DD
C1 dx 0 10n
Rin in vp 1Meg
.model OAX OA(AOL=100000 ROUT=100 VCC=9 VEE=-9)
.model DD D(IS=1e-14)
";
    let config = CodegenConfig {
        circuit_name: "dk_noclamp".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let (netlist, mna, kernel) = build_pipeline(spice, &config);
    let oout_idx = *mna.node_map.get("oout").unwrap() - 1;

    // Run the real DC OP, then simulate a beyond-rail solve (the DC OP
    // solver clamps op-amp outputs per-iteration in its normal paths, but
    // non-converged / fallback paths can return beyond-rail voltages with
    // a MATCHING i_nl — the exact case the removed bake clamp mangled by
    // clamping v while leaving i_nl untouched).
    let device_slots =
        CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).expect("device slots");
    let dc_op_config = melange_solver::dc_op::DcOpConfig {
        input_node: config.input_node,
        input_resistance: config.input_resistance,
        ..melange_solver::dc_op::DcOpConfig::default()
    };
    let mut dc =
        melange_solver::dc_op::solve_dc_operating_point(&mna, &device_slots, &dc_op_config);
    dc.v_node[oout_idx] = 12.0; // beyond the +9 V rail

    let ir = CircuitIR::from_kernel_with_dc_op(&kernel, &mna, &netlist, &config, Some(dc))
        .expect("build IR");

    let v_out = ir.dc_operating_point[oout_idx];
    assert!(
        (v_out - 12.0).abs() < 1e-12,
        "DK DC_OP bake must preserve the solver's (possibly beyond-rail) voltage so it \
         stays consistent with DC_NL_I — the runtime rail handling + warmup cover the \
         transient. Got {v_out} V at oout; a value pinned at 9.0 means the removed \
         bake clamp regressed."
    );
}

// ── Fix 6: inverted op-amp rails error at MNA resolve time ──────────────

#[test]
fn swapped_opamp_rails_error_at_build_not_runtime() {
    let spice = "\
Swapped rails
U1 inp 0 out BADRAILS
Rin in inp 10k
Rfb out inp 100k
Rload out 0 10k
.model BADRAILS OA(AOL=100000 ROUT=100 VCC=-9 VEE=9)
";
    let netlist = Netlist::parse(spice).expect("parse");
    let err = MnaSystem::from_netlist(&netlist)
        .expect_err("VCC=-9 VEE=9 must hard-error at MNA resolve time");
    let msg = err.to_string();
    assert!(
        msg.contains("U1") && msg.contains("VCC") && msg.contains("VEE"),
        "error must name the op-amp and both rails, got: {msg}"
    );
}

#[test]
fn single_rail_opamp_still_builds() {
    // One finite rail + one infinite is a legal single-sided clamp and must
    // not trip the new validation.
    let spice = "\
Single rail
U1 inp 0 out ONESIDED
Rin in inp 10k
Rfb out inp 100k
Rload out 0 10k
.model ONESIDED OA(AOL=100000 ROUT=100 VCC=9)
";
    let netlist = Netlist::parse(spice).expect("parse");
    MnaSystem::from_netlist(&netlist).expect("single-rail op-amp must still build");
}

// ── Fix 7: input_node validated against n_nodes, not kernel.n ───────────

#[test]
fn input_node_on_aug_row_rejected() {
    let config = CodegenConfig {
        circuit_name: "aug_input".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let (netlist, mna, kernel) = build_pipeline(DIODE_BIAS_SPICE, &config);
    assert!(
        kernel.n > kernel.n_nodes,
        "test premise: circuit has augmented rows (VS present)"
    );

    // Point input_node at the first augmented row: inside kernel.n (the old
    // check passed it through) but outside the circuit-node range.
    let bad_config = CodegenConfig {
        input_node: kernel.n_nodes,
        ..config
    };
    let generator = CodeGenerator::new(bad_config);
    match generator.generate(&kernel, &mna, &netlist) {
        Err(CodegenError::InvalidConfig(msg)) => {
            assert!(
                msg.contains("input_node") && msg.contains("n_nodes"),
                "error should name input_node vs n_nodes, got: {msg}"
            );
        }
        Err(other) => panic!("expected InvalidConfig, got: {other}"),
        Ok(_) => panic!("input_node on an augmented row must be rejected"),
    }
}

// ── Fix 9: CodegenMeta sparse-LU fields populated from the IR ───────────

#[test]
fn codegen_meta_sparse_lu_fields_populated() {
    // DK path: no sparse LU, density 0.0.
    let config = be_os_config(1);
    let (netlist, mna, kernel) = build_pipeline(DIODE_BIAS_SPICE, &config);
    let generated = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("DK codegen");
    assert!(
        !generated.meta.sparse_lu_enabled,
        "DK path has no sparse LU"
    );
    assert_eq!(generated.meta.sparse_lu_density, 0.0);

    // Nodal path with m > 0: the G_aug pattern density is computed and must
    // surface (> 0), and sparse_lu_enabled must agree with the density/size
    // gate (density < 0.4 && n >= 8).
    let nodal_config = CodegenConfig {
        circuit_name: "meta_nodal".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let netlist2 = Netlist::parse(DIODE_BIAS_SPICE).expect("parse");
    let mut mna2 = MnaSystem::from_netlist(&netlist2).expect("MNA");
    mna2.g[0][0] += 1.0;
    let generated2 = CodeGenerator::new(nodal_config)
        .generate_nodal(&mna2, &netlist2)
        .expect("nodal codegen");
    assert!(
        generated2.meta.sparse_lu_density > 0.0,
        "nodal m>0 circuit must surface the G_aug pattern density, got {}",
        generated2.meta.sparse_lu_density
    );
    let n = generated2.n;
    let expect_enabled = generated2.meta.sparse_lu_density < 0.4 && n >= 8;
    assert_eq!(
        generated2.meta.sparse_lu_enabled, expect_enabled,
        "sparse_lu_enabled must agree with the density ({:.3}) / size (N={}) gate",
        generated2.meta.sparse_lu_density, n
    );
}
