//! Breakpoint-BE regression tests — event-triggered backward-Euler on a
//! `.switch`/`.pot` conductance swap.
//!
//! A mid-run conductance swap stamps `Δg` into both the forward matrix `A` and
//! the trapezoidal history `A_neg = (2/T)C − G`, so on the swap sample `Δg` is
//! double-counted (output 2×) and the kick excites trap's marginal `z=−1`
//! eigenmode — which never decays on a capless node. The fix routes exactly one
//! sample through the L-stable BE matrices (`A_neg = (1/T)C`, no `G` term): no
//! double-count, and BE damps `z=−1` at the source.
//!
//! The end-to-end runtime proof (a mid-run `set_switch` on a capless divider:
//! residual 145% → ~1e-15; and Farfisa G10 `--switch Key=1 --force-trap`
//! settling correctly instead of collapsing) is a compile-and-run harness;
//! these tests pin the codegen surface, including the load-bearing "exactly one
//! BE sample" — a second sample over-damps and collapses the G10 astable.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// Nonlinear (m>0 → full-LU) clipper with a switched load resistor.
const SWITCH_CLIPPER: &str = "\
Diode clipper with switch
Rin in mid 1k
D1 mid out D1N4148
D2 out mid D1N4148
Rload out 0 100k
C1 out 0 10n
.switch Rload 100k 10k \"Load\"
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

// Linear (m=0 → Schur) divider: capped `mid`, capless switched `out`.
const SWITCH_DIVIDER: &str = "\
Linear switched divider
Rs in mid 10k
Cmid mid 0 100n
Rk mid out 1e9
Rload out 0 22k
.switch Rk 1e9 1.0 \"Key\"
";

// Nonlinear clipper with a knob `.pot`.
const POT_CLIPPER: &str = "\
Diode clipper with pot
Rin in mid 1k
D1 mid out D1N4148
D2 out mid D1N4148
Rload out 0 100k
C1 out 0 10n
.pot Rload 10k 100k \"Load\"
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

// Nonlinear clipper with a `.runtime R` (audio-rate) and NOTHING discrete.
const RUNTIME_R_ONLY: &str = "\
Diode clipper with runtime R
Rin in mid 1k
D1 mid out D1N4148
D2 out mid D1N4148
R_ldr out 0 100k
C1 out 0 10n
.runtime R_ldr 1k 10Meg as r_test
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

// No `.switch`/`.pot`/`.runtime` at all.
const PLAIN_CLIPPER: &str = "\
Diode clipper
Rin in mid 1k
D1 mid out D1N4148
D2 out mid D1N4148
Rload out 0 100k
C1 out 0 10n
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

fn generate_nodal(spice: &str, mut tweak: impl FnMut(&mut CodegenConfig)) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let in_idx = *mna.node_map.get("in").unwrap();
    if in_idx > 0 {
        mna.g[in_idx - 1][in_idx - 1] += 1.0; // G_in = 1/1Ω
    }
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut config = CodegenConfig {
        circuit_name: "bp_test".to_string(),
        sample_rate: 48000.0,
        input_node: in_idx - 1,
        output_nodes: vec![out_idx],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    tweak(&mut config);
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

#[test]
fn switch_trap_build_emits_breakpoint_be() {
    let code = generate_nodal(SWITCH_CLIPPER, |_| {});
    assert!(
        code.contains("pub breakpoint_be: u32"),
        "trap switch build must carry the breakpoint_be countdown field"
    );
    assert!(
        code.contains("self.breakpoint_be = BREAKPOINT_BE_SAMPLES;"),
        "set_switch_* must arm the breakpoint-BE countdown"
    );
    assert!(
        code.contains("state.breakpoint_be == 0"),
        "converged must be gated so breakpoint_be>0 forces the BE fallback"
    );
    assert!(
        code.contains("BREAKPOINT_BE_MAX_ITER"),
        "forced-BE samples must get their own (larger) NR budget"
    );
}

#[test]
fn breakpoint_be_is_exactly_one_sample() {
    // Load-bearing: a SECOND BE sample over-damps and knocks a marginal
    // self-oscillator (Farfisa G10 divider under --force-trap) into the wrong
    // equilibrium. One BE sample already removes both the 2× and the z=-1 mode.
    let code = generate_nodal(SWITCH_CLIPPER, |_| {});
    assert!(
        code.contains("pub const BREAKPOINT_BE_SAMPLES: u32 = 1;"),
        "breakpoint-BE must be exactly ONE sample — do not raise it"
    );
}

#[test]
fn linear_switch_build_emits_m0_be_override() {
    // The m=0 (linear, no NR) path has no BE fallback to reuse, so it carries an
    // explicit BE re-solve branch guarded on the countdown.
    let code = generate_nodal(SWITCH_DIVIDER, |_| {});
    assert!(
        code.contains("if state.breakpoint_be > 0 {"),
        "linear switch build must emit the m=0 breakpoint-BE override branch"
    );
    assert!(
        code.contains("state.s_be[i][j]") && code.contains("state.a_neg_be["),
        "m=0 override must solve on the BE matrices"
    );
}

#[test]
fn pot_knob_build_arms_breakpoint_be() {
    let code = generate_nodal(POT_CLIPPER, |_| {});
    assert!(
        code.contains("self.breakpoint_be = BREAKPOINT_BE_SAMPLES;"),
        "a knob .pot step must arm breakpoint-BE (a swap can excite z=-1)"
    );
}

#[test]
fn plain_build_omits_breakpoint_be() {
    let code = generate_nodal(PLAIN_CLIPPER, |_| {});
    assert!(
        !code.contains("breakpoint_be"),
        "a circuit with no .switch/.pot must not emit any breakpoint-BE code"
    );
}

#[test]
fn runtime_r_only_build_omits_breakpoint_be() {
    // `.runtime R` is audio-rate/continuous and does NOT arm breakpoint-BE
    // (arming every sample would pin BE permanently). A circuit whose only
    // dynamic parameter is a `.runtime R` must stay byte-identical.
    let code = generate_nodal(RUNTIME_R_ONLY, |_| {});
    assert!(
        !code.contains("breakpoint_be"),
        "a .runtime-R-only circuit must not emit breakpoint-BE machinery"
    );
}

#[test]
fn backward_euler_switch_build_omits_breakpoint_be() {
    // A BE build (explicit or auto-promoted) has no trap z=-1 artifact to fix.
    let code = generate_nodal(SWITCH_CLIPPER, |c| c.backward_euler = true);
    assert!(
        !code.contains("breakpoint_be"),
        "--backward-euler switch build must omit breakpoint-BE (nothing to fix)"
    );
}
