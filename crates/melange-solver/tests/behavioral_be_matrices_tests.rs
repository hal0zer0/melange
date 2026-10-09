//! A behavioral source forces backward Euler, and the build it forces must be
//! the backward-Euler build: the same baked matrices and the same samples as
//! `--backward-euler` on the same deck.
//!
//! The behavioral-forcing path once set the integrator label and the runtime
//! rebuild to backward Euler but left the coefficient that bakes the default
//! `A` and `A_neg` at the trapezoidal `2/T`, so at the compile rate every
//! capacitor integrated as twice its value: a linear RC ladder behind any
//! behavioral source, even an electrically isolated one, lost 2.7 dB at
//! 1 kHz and 7.7 dB at 10 kHz against the same ladder under the flag, while a
//! host at any other rate (which rebuilds at `1/T`) heard the right thing.

mod support;

/// A two-section RC ladder with an electrically isolated behavioral source:
/// the source changes nothing the ladder sees, only how the build integrates.
const LADDER_WITH_ISOLATED_BSOURCE: &str = "RC ladder plus an isolated behavioral source
R_dem in v_dem 2.2k
C_dem v_dem 0 33n
R_lp v_dem out 1k
C_lp out 0 10n
Rload out 0 1meg
B_iso iso 0 V={ 0.1 * V(in) }
Riso iso 0 10k
.end
";

fn build(backward_euler: bool) -> melange_solver::build::Built {
    let config = support::config_in_out_or_node1(LADDER_WITH_ISOLATED_BSOURCE, 48000.0);
    support::try_build_shipped_with(LADDER_WITH_ISOLATED_BSOURCE, &config, "auto", |o| {
        o.backward_euler = backward_euler;
    })
    .unwrap_or_else(|e| panic!("build refused: {e}"))
}

/// The text of one `const NAME: ... = [...];` item.
fn const_item<'a>(code: &'a str, name: &str) -> &'a str {
    let start = code
        .find(&format!("const {name}:"))
        .unwrap_or_else(|| panic!("no `{name}` in generated code"));
    // The type `[[f64; N]; N]` carries a `];` of its own: the item ends at
    // the first `];` after the `=`.
    let eq = code[start..].find(" = ").map(|i| start + i).unwrap();
    let end = code[eq..].find("\n];").map(|i| eq + i + 3).unwrap();
    &code[start..end]
}

#[test]
fn a_behavioral_forced_build_bakes_backward_euler_matrices() {
    let forced = build(false);
    let flagged = build(true);
    assert!(
        forced.generated.code.contains("behavioral-source forced"),
        "the deck must take the behavioral-forcing path"
    );
    assert!(flagged.generated.code.contains("--backward-euler"));
    for name in [
        "A_DEFAULT",
        "A_NEG_DEFAULT",
        "A_BE_DEFAULT",
        "A_NEG_BE_DEFAULT",
    ] {
        assert_eq!(
            const_item(&forced.generated.code, name),
            const_item(&flagged.generated.code, name),
            "{name} differs between the behavioral-forced and the flagged build"
        );
    }
    // The coefficient itself: A_neg for the 33 nF node (`v_dem`, the second
    // node named in the deck) is C/T at 48 kHz, read from the baked literal.
    let names = forced.mna.node_names_in_index_order();
    let cap_row = names
        .iter()
        .position(|n| *n == "v_dem")
        .expect("v_dem is a node")
        - 1;
    let item = const_item(&forced.generated.code, "A_NEG_DEFAULT");
    let rows: Vec<&str> = item.split("\n    [").skip(1).collect();
    let entry: f64 = rows[cap_row]
        .trim_end_matches([']', ',', ';', '\n'])
        .split(',')
        .nth(cap_row)
        .unwrap()
        .trim()
        .parse()
        .unwrap();
    let expected = 48000.0 * 33e-9;
    assert!(
        (entry - expected).abs() <= 1e-12 * expected,
        "A_NEG_DEFAULT[v_dem][v_dem] = {entry:e}, expected C/T = {expected:e} (2/T would be {:e})",
        2.0 * expected
    );
}

#[test]
fn a_behavioral_forced_build_renders_as_the_backward_euler_build() {
    let forced = build(false).generated.code;
    let flagged = build(true).generated.code;
    let main = "
fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let mut h: u64 = 0xcbf29ce484222325;
    let mut peak = 0.0f64;
    for k in 0..4800usize {
        let x = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / 48000.0).sin();
        let o = process_sample(x, &mut s);
        h = (h ^ o[0].to_bits()).wrapping_mul(0x100000001b3);
        if k >= 2400 { peak = peak.max(o[0].abs()); }
    }
    println!(\"{h:016x} {peak:.9}\");
}
";
    let a = support::compile_and_run(&forced, main, "bsrc_be_forced");
    let b = support::compile_and_run(&flagged, main, "bsrc_be_flagged");
    assert_eq!(
        a.stdout.trim(),
        b.stdout.trim(),
        "forced:\n{}\nflagged:\n{}",
        a.stderr,
        b.stderr
    );
    // And the level is the backward-Euler ladder's, not the doubled-capacitor
    // one: −1.68 dB on 0.5 V is 0.412 V; the defect read −4.33 dB, 0.304 V.
    let peak: f64 = a.stdout.split_whitespace().nth(1).unwrap().parse().unwrap();
    assert!((peak - 0.412).abs() < 0.01, "steady-state peak {peak} V");
}
