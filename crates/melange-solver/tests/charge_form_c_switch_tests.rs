//! A setter that changes C mid-stream does not carry the old charge derivative.
//!
//! The charge-form trapezoidal history is `alpha*C*v_prev + q_dot`, with
//! `q_dot` built on the C of the previous sample. After a `.switch` changes a
//! capacitor, `alpha*C_new*v_prev` would meet a `q_dot` built on `C_old`. The
//! breakpoint backward-Euler sample the switch arms does not read `q_dot`, and
//! it re-seeds `q_dot` from its own capacitor current `(C_new/T)*dv`. So the
//! render after the switch cannot depend on the `q_dot` it inherited: scrambling
//! `q_dot` at the switch leaves every later sample bit-identical. Without the
//! breakpoint sample (mutant), the first sample after the switch is trapezoidal
//! and reads the stale `q_dot`, so the scrambled render differs.

mod support;

use melange_solver::codegen::NodalSubPathOverride;

/// A coupling capacitor switched between 100 nF and 1 uF, into a diode node.
const DECK: &str = "switched coupling cap into a diode clipper\n\
R_in in a 1k\nC_c a b 100n\nD_1 b 0 DX\nD_2 0 b DX\nR_t b out 10k\nC_t out 0 22n\n\
R_l out 0 100k\n.switch C_c 100n 1u \"Cap\"\n.model DX D(IS=2.52n N=1.752)\n";

const ARM: &str = "self.breakpoint_be = BREAKPOINT_BE_SAMPLES;";

/// Render 0.1 s of a 1 kHz, 2 V sine, switching C_c at sample 2400 (and, when
/// `scramble`, overwriting `q_dot` there); print the outputs after the switch
/// as bit patterns.
fn render(code: &str, scramble: bool, tag: &str) -> Vec<u64> {
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for i in 0..4800usize {{
        if i == 2400 {{
            s.set_switch_0(1);
            if {scramble} {{ for q in s.q_dot.iter_mut() {{ *q = 1e-3 - *q * 7.0; }} }}
        }}
        let y = process_sample(2.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin(), &mut s)[0];
        if i >= 2400 {{ println!(\"y={{}}\", y.to_bits()); }}
    }}
}}"
    );
    support::compile_and_run(code, &main, tag)
        .stdout
        .lines()
        .filter_map(|l| l.strip_prefix("y="))
        .map(|v| v.parse().unwrap())
        .collect()
}

#[test]
fn a_c_switch_re_seeds_the_charge_derivative() {
    for (sub_path, tag) in [
        (NodalSubPathOverride::Schur, "cswitch_schur"),
        (NodalSubPathOverride::FullLu, "cswitch_full_lu"),
    ] {
        let mut config = support::config_for_spice(DECK, 48000.0);
        config.nodal_sub_path_override = sub_path;
        config.force_trap = true;
        let code = support::generate_circuit_code_nodal(DECK, &config).0;
        assert!(
            code.contains("pub q_dot: [f64; N]"),
            "{tag}: trapezoidal build"
        );
        assert_eq!(
            code.matches(ARM).count(),
            1,
            "{tag}: the switch setter arms BE"
        );

        let clean = render(&code, false, &format!("{tag}_clean"));
        let scrambled = render(&code, true, &format!("{tag}_scrambled"));
        assert_eq!(clean.len(), 2400);
        assert_eq!(
            clean, scrambled,
            "{tag}: the inherited q_dot leaked past the switch"
        );

        let mutant = code.replace(ARM, "");
        let m_clean = render(&mutant, false, &format!("{tag}_mutant_clean"));
        let m_scrambled = render(&mutant, true, &format!("{tag}_mutant_scrambled"));
        assert_ne!(
            m_clean, m_scrambled,
            "{tag}: the witness no longer sees the stale q_dot without the BE sample"
        );
    }
}
