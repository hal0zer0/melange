//! Breakpoint-BE regression tests — event-triggered backward-Euler on a
//! `.switch` that swaps a capacitor or an inductor.
//!
//! Under the charge form the trapezoidal history is `A_neg·v_prev + q_dot`
//! with `A_neg = (2/T)C` and `q_dot = C·dx/dt`: no `G` term. A conductance
//! change (a pot, a resistor-only switch) therefore leaves the carried state
//! consistent and needs no special sample; a backward-Euler sample there would
//! only cost first-order accuracy. (Under the old whole-system form,
//! `A_neg = (2/T)C − G`, a swap double-counted `Δg` on its sample; that is
//! what breakpoint-BE was first built for.) A reactance change still leaves
//! `q_dot` built on the old value, so a C- or L-switch routes exactly one
//! sample through the backward-Euler matrices, which re-seed `q_dot` from
//! their own capacitor currents. Exactly one: a second sample over-damps and
//! can knock a marginal self-oscillator into the wrong equilibrium.

mod support;

use melange_solver::codegen::CodegenConfig;

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

// The same clipper with its capacitor switched.
const SWITCH_CAP_CLIPPER: &str = "\
Diode clipper with a switched capacitor
Rin in mid 1k
D1 mid out D1N4148
D2 out mid D1N4148
Rload out 0 100k
C1 out 0 10n
.switch C1 10n 100n \"Cap\"
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

// Linear divider with a switched capacitor.
const SWITCH_CAP_DIVIDER: &str = "\
Linear divider with a switched capacitor
Rs in out 10k
Cout out 0 100n
Rload out 0 22k
.switch Cout 100n 1u \"Cap\"
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
    let mut config = support::config_for_spice(spice, 48000.0);
    config.circuit_name = "bp_test".to_string();
    tweak(&mut config);
    support::build_as_shipped(spice, &config, "nodal").0
}

#[test]
fn reactive_switch_trap_build_emits_breakpoint_be() {
    let code = generate_nodal(SWITCH_CAP_CLIPPER, |_| {});
    assert!(
        code.contains("pub breakpoint_be: u32"),
        "trap switch build must carry the breakpoint_be countdown field"
    );
    assert!(
        code.contains("self.breakpoint_be = BREAKPOINT_BE_SAMPLES;"),
        "set_switch_* must arm the breakpoint-BE countdown"
    );
    assert!(
        code.contains("let be_first = ")
            && code.contains("state.breakpoint_be > 0")
            && code.contains("if !be_first {"),
        "a breakpoint sample (breakpoint_be > 0) must skip the trap solve and take the BE solve"
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
    let code = generate_nodal(SWITCH_CAP_CLIPPER, |_| {});
    assert!(
        code.contains("pub const BREAKPOINT_BE_SAMPLES: u32 = 1;"),
        "breakpoint-BE must be exactly ONE sample — do not raise it"
    );
}

#[test]
fn linear_switch_build_emits_m0_be_override() {
    // The m=0 (linear, no NR) path has no BE fallback to reuse, so it carries an
    // explicit BE re-solve branch guarded on the countdown.
    let code = generate_nodal(SWITCH_CAP_DIVIDER, |_| {});
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
fn conductance_only_builds_omit_breakpoint_be() {
    for (deck, tag) in [
        (POT_CLIPPER, "knob pot"),
        (SWITCH_CLIPPER, "resistor switch"),
        (SWITCH_DIVIDER, "linear resistor switch"),
    ] {
        let code = generate_nodal(deck, |_| {});
        assert!(
            !code.contains("breakpoint_be"),
            "{tag}: a conductance change needs no breakpoint-BE under the charge form"
        );
    }
}

/// The runtime proof for a conductance-only switch: the capless divider's
/// switched node `out` is algebraic, `out = mid·Rload/(Rk + Rload)`, at every
/// sample, including the swap sample and the ones after it, on a plain
/// trapezoidal build with no breakpoint sample. Under the old whole-system form
/// the swap sample read 2x and `out` rang at fs/2.
#[test]
fn a_resistor_switch_is_exact_without_breakpoint_be() {
    let code = generate_nodal(SWITCH_DIVIDER, |_| {});
    let mna = melange_solver::mna::MnaSystem::from_netlist(
        &melange_solver::parser::Netlist::parse(SWITCH_DIVIDER).unwrap(),
    )
    .unwrap();
    let mid = mna.node_map["mid"] - 1;
    let out = mna.node_map["out"] - 1;
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let mut worst = 0.0f64;
    for k in 0..4800usize {{
        if k == 2400 {{ s.set_switch_0(1); }}
        let u = (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / 48000.0).sin();
        let _ = process_sample(u, &mut s);
        let rk = if k >= 2400 {{ 1.0 }} else {{ 1e9 }};
        let want = s.v_prev[{mid}] * 22000.0 / (rk + 22000.0);
        let got = s.v_prev[{out}];
        worst = worst.max((got - want).abs() / s.v_prev[{mid}].abs().max(1e-3));
    }}
    println!(\"worst={{:e}}\", worst);
}}"
    );
    let worst = support::compile_and_run(&code, &main, "bp_r_switch_exact")
        .parse_kv("worst")
        .unwrap();
    assert!(
        worst < 1e-9,
        "capless out left its algebraic value by {worst:e}"
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
