//! A `rate=host` injection is an audio input and reaches the circuit as one
//! does: clamped to ±INPUT_LIMIT_V (a Thevenin value is a voltage; a Norton
//! value is a current, clamped to the voltage the limit would put across its
//! shunt), counted in `diag_inject_clamp_count`, ahead of every piece of
//! state. A `rate=inner` value is not clamped. And after a state reset in one
//! inner sub-sample, the sub-samples that follow it within the same host
//! sample take the zeroed up-filters' response to that host sample, not a
//! value computed from the filter state the reset discarded.
//!
//! Witnesses, at 1x, 2x and 4x on DK and nodal full-LU:
//! - a 1e30 host value renders bit-identically to the same value pre-clamped
//!   to the limit by the caller (so the clamp sits ahead of the up-filter and
//!   the solve), and the clamp counter counts once per host sample and value;
//! - a Norton host value clamps at INPUT_LIMIT_V / RSHUNT amperes;
//! - a `rate=inner` value beyond the limit is not clamped (its render differs
//!   from the clamped one) and declares no clamp counter;
//! - a reset in sub-sample j (one absurd `rate=inner` value) leaves every
//!   up-filter in the state a fresh filter reaches after processing exactly
//!   that host sample, for j = 0 and 1 at 2x and j = 0, 1, 2 at 4x.

mod support;

use melange_solver::codegen::{CodegenConfig, NodalSubPathOverride};

/// A diode clipper with a Thevenin host injection, a Norton host injection
/// and an inner-rate injection on the clip node.
const DECK: &str = "host injection clamp witness
R1 in a 2.2k
C1 a b 47n
R2 b 0 100k
D1 b c DX
D2 c b DX
R3 b c 10k
Ctone c 0 10n
Rload c out 1k
Cout out 0 1n
.inject b side R=4.7k
.inject c pump RSHUNT=100k
.inject b loop R=22k rate=inner
.model DX D(IS=2.52n N=1.752)
.end
";

#[derive(Clone, Copy)]
enum Route {
    Dk,
    FullLu,
}

fn build(route: Route, oversampling: usize) -> String {
    let mut c: CodegenConfig = support::config_for_spice(DECK, 48000.0);
    c.oversampling_factor = oversampling;
    match route {
        Route::Dk => support::generate_circuit_code(DECK, &c).0,
        Route::FullLu => {
            c.nodal_sub_path_override = NodalSubPathOverride::FullLu;
            support::generate_circuit_code_nodal(DECK, &c).0
        }
    }
}

/// Render 400 host samples of a 1 kHz sine with the given host-injection
/// values on `side` and `pump` from sample 100 on (the sine before), and
/// print a hash of every output sample plus the counters.
fn render_main(side: &str, pump: &str, inner: &str) -> String {
    format!(
        "fn main() {{
    let mut s = CircuitState::default();
    let mut acc: u64 = 0;
    for i in 0..400usize {{
        let x = 0.2 * (i as f64 * 2.0 * std::f64::consts::PI * 1000.0 / 48000.0).sin();
        let (side, pump, lp): (f64, f64, f64) = if i >= 100 {{ ({side}, {pump}, {inner}) }} else {{ (x, 1e-6 * x, x) }};
        let inner = [[lp; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR];
        let (out, _taps) = process_sample(x, &[side, pump], &inner, &mut s);
        for v in out {{ assert!(v.is_finite()); acc = acc.wrapping_mul(1099511628211).wrapping_add(v.to_bits()); }}
    }}
    println!(\"hash={{}}\", acc);
    println!(\"clamp={{}}\", s.diag_inject_clamp_count);
    println!(\"nan={{}}\", s.diag_runtime_nan_count);
    println!(\"mag={{}}\", s.diag_magnitude_reset_count);
}}"
    )
}

fn hash_of(code: &str, main: &str, tag: &str) -> (u64, u64, u64) {
    let out = support::compile_and_run(code, main, tag);
    let get = |k: &str| {
        out.parse_kv(k)
            .unwrap_or_else(|| panic!("{tag}: no {k}= line\n{}\n{}", out.stdout, out.stderr))
            as u64
    };
    (get("hash"), get("clamp"), get("mag"))
}

#[test]
fn a_host_injection_is_clamped_ahead_of_every_piece_of_state() {
    for (route, rn) in [(Route::Dk, "dk"), (Route::FullLu, "full_lu")] {
        for os in [1usize, 2, 4] {
            let tag = format!("inject_clamp_{rn}_{os}x");
            let code = build(route, os);
            assert!(
                code.contains("pub diag_inject_clamp_count: u64"),
                "{tag}: a build with a rate=host injection declares the clamp counter"
            );
            // Thevenin: 1e30 V on `side` renders exactly as +100 V does.
            let (raw, clamped, _) = hash_of(&code, &render_main("1e30", "1e-6 * x", "x"), &tag);
            let (pre, pre_clamped, _) =
                hash_of(&code, &render_main("100.0", "1e-6 * x", "x"), &tag);
            assert_eq!(
                raw, pre,
                "{tag}: 1e30 V on a Thevenin host injection renders as 100 V"
            );
            assert_eq!(clamped, 300, "{tag}: one count per host sample and value");
            assert_eq!(pre_clamped, 0, "{tag}: a value at the limit is not counted");
            // Norton: the limit is the current that puts INPUT_LIMIT_V across
            // the 100 kΩ shunt, 1 mA.
            let (raw, clamped, _) = hash_of(&code, &render_main("x", "-1e30", "x"), &tag);
            let (pre, _, _) = hash_of(&code, &render_main("x", "-1e-3", "x"), &tag);
            assert_eq!(
                raw, pre,
                "{tag}: a Norton host injection clamps at INPUT_LIMIT_V / RSHUNT"
            );
            assert_eq!(clamped, 300, "{tag}: the Norton clamp is counted");
            // Both at once: two counts per host sample.
            let (_, clamped, _) = hash_of(&code, &render_main("-1e30", "1e30", "x"), &tag);
            assert_eq!(clamped, 600, "{tag}: each clamped value counts");
        }
    }
}

#[test]
fn an_inner_rate_injection_is_not_clamped() {
    for (route, rn) in [(Route::Dk, "dk"), (Route::FullLu, "full_lu")] {
        for os in [1usize, 2] {
            let tag = format!("inject_inner_unclamped_{rn}_{os}x");
            let code = build(route, os);
            // 300 V through 22 kΩ into the clip node: solvable, beyond the
            // limit, and not the same render as 100 V.
            let (big, clamped, mag) = hash_of(&code, &render_main("x", "1e-6 * x", "300.0"), &tag);
            let (lim, _, _) = hash_of(&code, &render_main("x", "1e-6 * x", "100.0"), &tag);
            assert_ne!(
                big, lim,
                "{tag}: a rate=inner value beyond the limit reaches the circuit"
            );
            assert_eq!(
                clamped, 0,
                "{tag}: an inner-rate value never counts as clamped"
            );
            assert_eq!(
                mag, 0,
                "{tag}: 300 V on the loop injection is a solvable drive"
            );
        }
    }
    // A deck whose only injection is inner-rate declares no clamp counter.
    let inner_only = DECK
        .replace(".inject b side R=4.7k\n", "")
        .replace(".inject c pump RSHUNT=100k\n", "");
    let mut c: CodegenConfig = support::config_for_spice(&inner_only, 48000.0);
    c.oversampling_factor = 2;
    let code = support::generate_circuit_code(&inner_only, &c).0;
    assert!(
        !code.contains("pub diag_inject_clamp_count")
            && !code.contains("diag_inject_clamp_count +="),
        "a build without a rate=host injection declares no clamp counter"
    );
}

/// After a reset in inner sub-sample `j` of host sample 100 (an absurd
/// `rate=inner` value, 1e30 V through 22 kΩ, trips the magnitude guard), the
/// up-filter states at the end of that host sample must equal the states
/// fresh filters reach after processing exactly that host sample: the input
/// through the input's filters, each host injection through its own.
fn refilter_main(os: usize, j: usize, fresh: bool) -> String {
    // Fresh filter states are zeroed copies of the state's own arrays, so
    // their lengths follow the build without naming the state size.
    let (outer_fresh, outer_cmp) = if os == 4 {
        (
            "let mut fo = s.os_up_state_outer; fo.iter_mut().for_each(|v| *v = 0.0); let (oe, oo) = os_halfband_outer(x, &OS_COEFFS_OUTER, &mut fo);
         let mut fi = s.os_up_state; fi.iter_mut().for_each(|v| *v = 0.0); let _ = os_halfband(oe, &OS_COEFFS, &mut fi); let _ = os_halfband(oo, &OS_COEFFS, &mut fi);
         let mut fo_s = s.os_inj_up_state_outer[0]; fo_s.iter_mut().for_each(|v| *v = 0.0); let (se, so) = os_halfband_outer(side, &OS_COEFFS_OUTER, &mut fo_s);
         let mut fi_s = s.os_inj_up_state[0]; fi_s.iter_mut().for_each(|v| *v = 0.0); let _ = os_halfband(se, &OS_COEFFS, &mut fi_s); let _ = os_halfband(so, &OS_COEFFS, &mut fi_s);",
            "if fresh { assert_eq!(s.os_up_state_outer, fo, \"outer input filter\"); assert_eq!(s.os_inj_up_state_outer[0], fo_s, \"outer injection filter\"); }
         else { assert!(s.os_up_state_outer.iter().all(|v| *v == 0.0), \"outer input filter zeroed\"); assert!(s.os_inj_up_state_outer[0].iter().all(|v| *v == 0.0), \"outer injection filter zeroed\"); }",
        )
    } else {
        (
            "let mut fi = s.os_up_state; fi.iter_mut().for_each(|v| *v = 0.0); let _ = os_halfband(x, &OS_COEFFS, &mut fi);
         let mut fi_s = s.os_inj_up_state[0]; fi_s.iter_mut().for_each(|v| *v = 0.0); let _ = os_halfband(side, &OS_COEFFS, &mut fi_s);",
            "",
        )
    };
    format!(
        "fn main() {{
    let mut s = CircuitState::default();
    for i in 0..101usize {{
        let x = 0.2 * (i as f64 * 2.0 * std::f64::consts::PI * 1000.0 / 48000.0).sin();
        let side = 0.5 * x;
        let mut inner = [[x; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR];
        if i == 100 {{ inner[{j}][0] = 1e30; }}
        let fresh = {fresh};
        let before = s.diag_magnitude_reset_count + s.diag_nan_reset_count;
        let (out, _taps) = process_sample(x, &[side, 1e-6 * x], &inner, &mut s);
        for v in out {{ assert!(v.is_finite()); }}
        if i == 100 {{
            assert!(s.diag_magnitude_reset_count + s.diag_nan_reset_count > before, \"premise: the absurd inner value resets the state\");
            {outer_fresh}
            {outer_cmp}
            if {fresh} {{
                assert_eq!(s.os_up_state, fi, \"inner input filter\");
                assert_eq!(s.os_inj_up_state[0], fi_s, \"inner injection filter\");
            }} else {{
                // Nothing follows the last sub-sample: the reset's zeroed
                // filters are what the next host sample starts from.
                assert!(s.os_up_state.iter().all(|v| *v == 0.0), \"inner input filter zeroed\");
                assert!(s.os_inj_up_state[0].iter().all(|v| *v == 0.0), \"inner injection filter zeroed\");
            }}
        }}
    }}
    println!(\"ok=1\");
}}"
    )
}

#[test]
fn sub_samples_after_a_reset_take_the_zeroed_filters_response() {
    // The dynamic witness runs on nodal full-LU, where an absurd inner-rate
    // value trips the magnitude guard. DK contains an unsolved sample's step
    // (VOLTAGE_LIMITING.md, per-sample containment), so no finite injection
    // reaches the guard there; its wrapper is the same emitted text, checked
    // for the re-filter block below.
    for os in [2usize, 4] {
        let dk = build(Route::Dk, os);
        let refilters = dk.matches("!= resets_before {").count();
        assert!(
            refilters == if os == 2 { 1 } else { 3 } && dk.contains("the zeroed filter"),
            "dk {os}x: the wrapper re-filters after a reset in every sub-sample but the last ({refilters} sites)"
        );
    }
    for (route, rn) in [(Route::FullLu, "full_lu")] {
        for os in [2usize, 4] {
            let code = build(route, os);
            for j in 0..os {
                let fresh = j + 1 < os;
                let tag = format!("refilter_{rn}_{os}x_j{j}");
                let out = support::compile_and_run(&code, &refilter_main(os, j, fresh), &tag);
                assert_eq!(
                    out.parse_kv("ok"),
                    Some(1.0),
                    "{tag}: filters after a reset in sub-sample {j}\n{}\n{}",
                    out.stdout,
                    out.stderr
                );
            }
        }
    }
}
