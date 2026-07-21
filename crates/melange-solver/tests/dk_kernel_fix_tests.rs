//! Regression tests for the 2026-07 DK kernel / codegen fix wave:
//!
//! - F1: companion-path inductor history (i_hist = 2*i[n-1]) — the
//!   time-march reference tests live in src/dk_math_verification.rs;
//!   here we cover the emitted-code side effects.
//! - F2: emitted rebuild_matrices refreshes the backward-Euler fallback
//!   matrix set (s_be/k_be/a_neg_be/s_ni_be).
//! - F3: non-negative K diagonal (with live N_i column) forces the nodal
//!   route; zero-N_i-column dims (MOSFET gate) don't false-flag.
//! - F5: rebuild_matrices preserves inductor transient state (no DC thump
//!   from per-block pot smoothing).
//! - F6: near-singular transformer-group inductance matrices are a hard
//!   kernel-build error, not silent garbage.
//! - F7: non-finite entries in the A matrix are a hard inversion error.

mod support;

use melange_solver::codegen::routing::{auto_route, SolverRoute};
use melange_solver::dk::{DkError, DkKernel};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ───────────────────────── F7: NaN in G errors loudly ─────────────────────

#[test]
fn test_nan_in_g_matrix_errors_loudly() {
    let spice = "NaN G\nR1 in out 1k\nC1 out 0 100n\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] = f64::NAN;

    match DkKernel::from_mna(&mna, 48000.0) {
        Err(DkError::SingularMatrix(msg)) => {
            assert!(
                msg.contains("non-finite"),
                "error should identify the non-finite entry, got: {msg}"
            );
        }
        Err(e) => panic!("expected SingularMatrix for NaN G entry, got: {e}"),
        Ok(_) => panic!("kernel build must not succeed with NaN in G"),
    }
}

// ───────────── F6: near-singular transformer group is a hard error ─────────

#[test]
fn test_transformer_group_k_near_one_hard_errors() {
    // Three mutually coupled windings with k → 1: the inductance matrix is
    // singular (rank 1). invert_small_matrix's absolute 1e-30 pivot check
    // cannot catch this at O(1e-3) entry scale — the elimination produces
    // roundoff-scale pivots and garbage inverse entries. The kernel build
    // must reject the group, mirroring the 2-winding det<=0 hard error.
    let spice = "\
Near Singular Xfmr
R1 in p1 100
L1 p1 0 10m
L2 s1 0 10m
L3 s2 0 10m
K1 L1 L2 0.9999999999999999
K2 L1 L3 0.9999999999999999
K3 L2 L3 0.9999999999999999
R2 s1 0 1k
R3 s2 0 2.2k
";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert_eq!(
        mna.transformer_groups.len(),
        1,
        "should form one 3-winding group"
    );
    mna.g[0][0] += 1.0;

    match DkKernel::from_mna(&mna, 48000.0) {
        Err(DkError::SingularMatrix(_)) => {} // expected — hard error, not garbage
        Err(e) => panic!("expected SingularMatrix for k→1 group, got: {e}"),
        Ok(kernel) => {
            // If inversion somehow produced a "valid" kernel it must NOT be
            // the silent identity fallback (which pretends 1 H uncoupled
            // windings). Reaching here means the guard failed.
            panic!(
                "kernel build must reject near-singular transformer group \
                 (got Y with max entry {:.3e})",
                kernel
                    .transformer_groups
                    .first()
                    .map(|g| g.y_matrix.iter().fold(0.0_f64, |a, &v| a.max(v.abs())))
                    .unwrap_or(0.0)
            );
        }
    }
}

/// A healthy 3-winding group (k = 0.9) must still build — the residual
/// guard must not reject legitimately coupled transformers.
#[test]
fn test_transformer_group_healthy_coupling_still_builds() {
    let spice = "\
Healthy Xfmr
R1 in p1 100
L1 p1 0 10m
L2 s1 0 10m
L3 s2 0 10m
K1 L1 L2 0.9
K2 L1 L3 0.9
K3 L2 L3 0.9
R2 s1 0 1k
R3 s2 0 2.2k
";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;
    DkKernel::from_mna(&mna, 48000.0).expect("healthy k=0.9 group must build");
}

// ──────── F3: K-diagonal sign gate in auto_route ──────────────────────────

/// A genuinely positive K diagonal on a dimension with current injection
/// must force the nodal route. `from_mna_augmented` only warns on this
/// (transformer-coupled NFB circuits), so without the routing gate a
/// single-transformer NFB circuit would reach DK Schur and the NR
/// (J = I - J_dev*K) would see positive feedback and diverge.
///
/// A real positive-K circuit needs transformer feedback around an active
/// stage (e.g. the wurli power amp output transformer tertiary); here we
/// build a healthy diode kernel (which passes every other routing gate)
/// and flip the diagonal sign to isolate the K-diagonal gate.
#[test]
fn test_positive_k_diagonal_with_live_ni_column_routes_nodal() {
    let spice = "\
Diode Clipper
R1 in a 1k
D1 a 0 DMOD
C1 a 0 100n
.model DMOD D(IS=2.52e-9 N=1.752)
";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;

    let mut kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    assert_eq!(kernel.m, 1);
    assert!(
        kernel.k[0] < 0.0,
        "sanity: healthy diode K diagonal is negative"
    );

    // Healthy kernel routes normally (whatever the other gates decide)
    let healthy = auto_route(&kernel, &mna, false);
    assert!(
        !healthy.k_diag_unsafe,
        "negative K diagonal must not be flagged"
    );

    // Flip the diagonal: transformer-NFB stand-in
    kernel.k[0] = kernel.k[0].abs();
    let decision = auto_route(&kernel, &mna, false);
    assert!(
        decision.k_diag_unsafe,
        "positive K diagonal with live N_i column must set k_diag_unsafe"
    );
    assert_eq!(
        decision.route,
        SolverRoute::Nodal,
        "positive K diagonal must force nodal (reason: {})",
        decision.reason
    );
    assert!(
        decision.reason.contains("K diagonal"),
        "reason must name the mechanism, got: {}",
        decision.reason
    );
}

/// MOSFET insulated gate: K[gate][gate] = 0 with an all-zero N_i column.
/// This is benign by construction and must not trip the K-diagonal gate.
#[test]
fn test_mosfet_zero_ni_column_not_flagged() {
    let spice = "\
MOSFET Common Source
Cin in gate 10u
R1 vdd gate 47k
R2 gate 0 100k
M1 drain gate source 0 NMOD
Rd vdd drain 1k
Rs source 0 100
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 5
.model NMOD NM(VTO=2.0 KP=0.1 LAMBDA=0.01)
";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;

    let kernel = DkKernel::from_mna(&mna, 48000.0)
        .expect("MOSFET kernel must build (zero N_i column exception)");
    let decision = auto_route(&kernel, &mna, false);
    assert!(
        !decision.k_diag_unsafe,
        "zero-N_i-column K[i][i]=0 must not be flagged (reason: {})",
        decision.reason
    );
    assert_eq!(
        decision.route,
        SolverRoute::DkSchur,
        "MOSFET CS should still route DK Schur (reason: {})",
        decision.reason
    );
}

/// F3a end-to-end: `from_mna_augmented` applies the same zero-N_i-column
/// exception as `from_mna` (no warn, no flag) when an inductor forces the
/// augmented build path.
#[test]
fn test_mosfet_with_inductor_augmented_kernel_not_flagged() {
    let spice = "\
MOSFET CS with inductor load
Cin in gate 10u
R1 vdd gate 47k
R2 gate 0 100k
M1 drain gate source 0 NMOD
Rd vdd drain 1k
L1 vdd drain 100m
Rs source 0 100
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 5
.model NMOD NM(VTO=2.0 KP=0.1 LAMBDA=0.01)
";
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;

    let kernel =
        DkKernel::from_mna_augmented(&mna, 48000.0).expect("augmented MOSFET kernel must build");
    let decision = auto_route(&kernel, &mna, false);
    assert!(
        !decision.k_diag_unsafe,
        "augmented-path zero-N_i-column dims must not be flagged (reason: {})",
        decision.reason
    );
}

// ──────── F2: emitted rebuild_matrices refreshes the BE fallback set ───────

const DIODE_POT_BE_SPICE: &str = "\
Diode Pot BE
R1 in out 10k
D1 out 0 DCLIP
D2 0 out DCLIP
C1 out 0 10n
.model DCLIP D(IS=2.52e-9 N=1.752)
.pot R1 1k 100k
";

#[test]
fn test_rebuild_matrices_emits_be_rebuild() {
    let config = support::config_for_spice(DIODE_POT_BE_SPICE, 44100.0);
    let (code, _n, m) = support::generate_circuit_code(DIODE_POT_BE_SPICE, &config);
    assert!(m > 0);
    assert!(
        code.contains("s_be"),
        "diode circuit should carry a BE fallback set"
    );

    // The BE rebuild must live inside rebuild_matrices, not only in the
    // no-pot set_sample_rate template variant (which pot circuits never use).
    let rb_start = code
        .find("fn rebuild_matrices(&mut self)")
        .expect("rebuild_matrices must be emitted for a pot circuit");
    let rb_end = code[rb_start..]
        .find("\n    }")
        .map(|i| rb_start + i)
        .unwrap_or(code.len());
    let rb_body = &code[rb_start..rb_end];
    assert!(
        rb_body.contains("alpha_be") && rb_body.contains("a_neg_be"),
        "rebuild_matrices must rebuild the backward-Euler matrix set \
         (stale s_be/k_be/a_neg_be corrupt exactly the stressed samples \
         that trigger the fallback)"
    );
    assert!(
        rb_body.contains("self.s_be = s_be;")
            && rb_body.contains("self.k_be = k_be;")
            && rb_body.contains("self.s_ni_be = s_ni_be;")
            && rb_body.contains("self.a_neg_be = a_neg_be;"),
        "rebuild_matrices must store the rebuilt BE set"
    );
}

#[test]
fn test_rebuilt_be_matrices_are_consistent_after_rate_and_pot_change() {
    let config = support::config_for_spice(DIODE_POT_BE_SPICE, 44100.0);
    let (code, _n, _m) = support::generate_circuit_code(DIODE_POT_BE_SPICE, &config);

    // Node order in DIODE_POT_BE_SPICE: in = 0, out = 1 (order of first
    // appearance). The pot spans in-out, so its delta_g stamps hit
    // [0][0], [1][1], -[0][1], -[1][0].
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    state.set_sample_rate(96000.0);
    state.set_pot_0(2000.0);
    let _ = process_sample(0.0, &mut state); // trigger lazy rebuild

    // Reference A_be at the new rate with the pot delta applied
    let internal = 96000.0 * OVERSAMPLING_FACTOR as f64;
    let alpha_be = internal;
    let delta_g = 1.0 / 2000.0 - POT_0_G_NOM;
    let mut a_be = [[0.0f64; N]; N];
    for i in 0..N {
        for j in 0..N {
            a_be[i][j] = G[i][j] + alpha_be * C[i][j];
        }
    }
    a_be[0][0] += delta_g;
    a_be[1][1] += delta_g;
    a_be[0][1] -= delta_g;
    a_be[1][0] -= delta_g;

    // ||A_be * s_be - I||_inf : stale s_be (44.1kHz, nominal pot) fails this
    let mut resid = 0.0f64;
    for i in 0..N {
        for j in 0..N {
            let mut sum = 0.0;
            for k in 0..N {
                sum += a_be[i][k] * state.s_be[k][j];
            }
            let target = if i == j { 1.0 } else { 0.0 };
            let d = (sum - target).abs();
            if d > resid {
                resid = d;
            }
        }
    }
    println!("be_residual={resid:.6e}");

    // Sanity: the circuit still runs and produces finite output
    let mut peak = 0.0f64;
    for k in 0..2048 {
        let x = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / 96000.0).sin();
        let y = process_sample(x, &mut state)[0];
        assert!(y.is_finite());
        if y.abs() > peak {
            peak = y.abs();
        }
    }
    println!("peak={peak:.6e}");
}
"#;

    let output = support::compile_and_run(&code, main_code, "be_rebuild_consistency");
    let resid = output
        .parse_kv("be_residual")
        .expect("main must print be_residual");
    assert!(
        resid < 1e-6,
        "rebuilt s_be must invert the current-rate/current-pot A_be \
         (||A_be*s_be - I||_inf = {resid:.3e}); a stale BE set means \
         rebuild_matrices did not refresh it"
    );
    let peak = output.parse_kv("peak").expect("main must print peak");
    assert!(peak > 1e-4, "circuit should pass signal after rate change");
}

// ──────── F5: rebuild preserves inductor state (no pot-sweep DC thump) ─────

// Note the 100n node capacitance at `mid`: the doubled-trapezoidal DK
// formulation has A + A_neg = 2*alpha*C exactly (G cancels), so any
// node-voltage pattern in null(C) is a marginally-stable period-2
// (Nyquist) mode. The startup inconsistency between v_prev = DC_OP and
// ind_i_prev = 0 excites that mode; a capacitance on the inductor node
// (physically always present) pins it to zero. Without the cap the test
// would measure that startup artifact, not the rebuild behavior.
const POT_INDUCTOR_SPICE: &str = "\
Pot Inductor Continuity
Vcc vcc 0 DC 9
L1 vcc mid 100m
R1 mid 0 1k
C2 mid 0 100n
Rin in mid 10k
C1 mid out 10u
Rout out 0 10k
.pot R1 500 2k
";

#[test]
fn test_pot_sweep_preserves_inductor_dc_current() {
    let config = support::config_for_spice(POT_INDUCTOR_SPICE, 48000.0);
    let (code, _n, m) = support::generate_circuit_code(POT_INDUCTOR_SPICE, &config);
    assert_eq!(m, 0, "linear circuit");
    assert!(
        code.contains("ind_i_hist"),
        "companion inductor path expected"
    );

    // L1 carries a standing DC current (9 V / R1). Sweeping the pot
    // per-block must not reset ind_i_prev — pre-fix, every rebuild zeroed
    // it, injecting an ~I*R voltage spike at `mid` on every knob tick.
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    // Settle the standing current (L/R ~ 0.1 ms) and the coupling cap (0.1 s)
    for _ in 0..96000 {
        let _ = process_sample(0.0, &mut state);
    }
    let i_settled = state.ind_i_prev[0];
    println!("i_settled={i_settled:.6e}");

    // Sweep the pot from its 1k nominal to 2k in small per-block steps
    // (2.5 ohm/block — knob-smoothing granularity). Starting at nominal
    // keeps each step's *physical* transient tiny (i_L * dR ~ 25 mV);
    // pre-fix, the rebuild zeroed ind_i_prev (~10 mA standing current)
    // every block, which dumps an i*R-scale spike into `mid` regardless
    // of step size.
    let mut prev = process_sample(0.0, &mut state)[0];
    let mut max_step = 0.0f64;
    let blocks = 400;
    for b in 0..=blocks {
        let r = 1000.0 + 1000.0 * (b as f64 / blocks as f64);
        state.set_pot_0(r);
        for _ in 0..64 {
            let out = process_sample(0.0, &mut state)[0];
            assert!(out.is_finite());
            let step = (out - prev).abs();
            if step > max_step {
                max_step = step;
            }
            prev = out;
        }
    }
    println!("max_step={max_step:.6e}");
    println!("i_final={:.6e}", state.ind_i_prev[0]);
}
"#;

    let output = support::compile_and_run(&code, main_code, "pot_inductor_continuity");
    let i_settled = output
        .parse_kv("i_settled")
        .expect("main must print i_settled");
    assert!(
        i_settled.abs() > 1e-3,
        "sanity: inductor must carry standing DC current, got {i_settled:.3e} A"
    );
    let max_step = output
        .parse_kv("max_step")
        .expect("main must print max_step");
    assert!(
        max_step < 0.1,
        "per-block pot sweep must not thump the output \
         (max sample-to-sample step = {max_step:.3e} V; pre-fix the rebuild \
         zeroed ind_i_prev and re-injected the full standing current each block)"
    );
    let i_final = output.parse_kv("i_final").expect("main must print i_final");
    assert!(
        i_final.abs() > 1e-3,
        "standing current must survive the sweep, got {i_final:.3e} A"
    );
}
