//! Runtime-selectable oversampling (`.oversampling N allow=...`).
//!
//! The acceptance gate: at every factor of the set, the runtime build computes
//! exactly what the fixed build at that factor computes (its Newton budget
//! pinned to the set's largest, which the runtime build uses at every factor),
//! bit for bit, at the compile rate (baked matrices) and off it (matrices
//! rebuilt), with a pot moved and injections driven. Every node voltage is
//! compared, not only the outputs, so a sub-circuit the outputs do not see is
//! held too. The decks between them carry every rate-dependent feature: a
//! saturating inductor, an op-amp slew limit and rails, a glow lamp with
//! sub-sample fire, a trapezoidal route carrying the runtime BE latch,
//! `.inject` at both rates and `.tap`, pots, both solver families.
//!
//! Also held: a factor switch lands on exactly a fresh state at the new
//! factor; a set whose factors build different solvers is refused, naming
//! what differs; the set is in provenance.

mod support;

use melange_solver::build::OversamplingSet;
use melange_solver::codegen::NodalSubPathOverride;

/// Every rate-dependent feature at once (nodal full-LU, trapezoidal, latch).
const GUARD: &str = "Runtime oversampling guard deck
R1 in a 10k
C0 a 0 1n
U1 a fb o1 OPX
R2 o1 fb 47k
R3 fb 0 10k
R4 o1 d 1k
D1 d 0 DX
D2 0 d DX
C2 d 0 10n
R5 d li 100
L1 li 0 100m ISAT=20m CORE=steel
Vb rail 0 DC 170
Rc rail osc 1MEG
Cosc osc 0 10N
N1 osc 0 NE1
Rp d out 10k
.pot Rp 1k 100k
Cout out 0 1n
Rl out 0 100k
.inject out aux R=10k
.inject d fb RSHUNT=100k rate=inner
.tap d
.model DX D(IS=2.52n N=1.752)
.model OPX OA(AOL=200000 ROUT=50 GBW=3e6 SR=13 VCC=15 VEE=-15)
.model NE1 NEON(VO=135 VM=93 IK=1.5e-3 RS=3000 IHOLD=2e-4 ROFF=300e6)
.end
";

/// A glow relaxation oscillator on nodal Schur: sub-sample fire is active.
const GLOW: &str = "Glow relaxation oscillator driven from the input
.model NE1 NEON(VO=135 VM=93 IK=1.5e-3 RS=3000 IHOLD=2e-4 ROFF=300e6)
Vb rail 0 DC 170
Rc rail osc 1MEG
Cosc osc 0 10N
N1 osc 0 NE1
Rin in osc 1MEG
Rout osc out 100k
Cout out 0 1n
.end
";

/// A diode clipper with a pot, built on DK (the template emitter).
const DK_POT: &str = "Diode clipper with a drive pot
R1 in a 2.2k
C1 a b 47n
R2 b 0 100k
D1 b c DX
D2 c b DX
Rdrive b c 10k
.pot Rdrive 1k 100k
Ctone c 0 10n
Rload c out 1k
Cout out 0 1n
.model DX D(IS=2.52n N=1.752)
.end
";

const SET: [usize; 3] = [1, 2, 4];
const RATES: [f64; 2] = [48000.0, 96000.0];

/// The generated code of `spice` at `oversampling` (the default factor, for a
/// runtime set), or the build's refusal.
fn build(
    spice: &str,
    route: (&str, NodalSubPathOverride),
    oversampling: usize,
    set: OversamplingSet,
    max_iter: Option<usize>,
) -> Result<String, String> {
    let config = melange_solver::codegen::CodegenConfig {
        nodal_sub_path_override: route.1,
        ..support::config_in_out_or_node1(spice, 48000.0)
    };
    support::try_build_shipped_with(spice, &config, route.0, |o| {
        o.oversampling = Some(oversampling);
        o.oversampling_set = set;
        o.max_iter = max_iter;
    })
    .map(|b| b.generated.code)
}

/// A driver that, for each `(rate, factor)` (factor `None`: a fixed build),
/// starts from a fresh state, moves the first pot off nominal, and prints each
/// sample's outputs and a hash of every node voltage as bit patterns.
fn driver(code: &str, factors: &[Option<usize>]) -> String {
    let rows = if code.contains("pub const MAX_OVERSAMPLING") {
        "MAX_OVERSAMPLING"
    } else {
        "OVERSAMPLING_FACTOR"
    };
    let call = if code.contains("pub const NUM_INJECT_HOST") {
        format!(
            "let mut inner = [[0.0; NUM_INJECT_INNER]; {rows}];\n\
             for (j, r) in inner.iter_mut().enumerate() {{ for v in r.iter_mut() {{ *v = 1e-6 * x * (j as f64 + 1.0); }} }}\n\
             let (o, _t) = process_sample(x, &[0.25 * x; NUM_INJECT_HOST], &inner, &mut st);"
        )
    } else {
        "let o = process_sample(x, &mut st);".to_string()
    };
    let pot = if code.contains("pub fn set_pot_0(") {
        "st.set_pot_0(22000.0);"
    } else {
        ""
    };
    let mut body = String::new();
    for &rate in &RATES {
        for f in factors {
            let set = f.map_or(String::new(), |f| {
                format!("st.set_oversampling({f}).unwrap();")
            });
            body.push_str(&format!(
                "{{ let mut st = CircuitState::default(); {set} st.set_sample_rate({rate:.1}); {pot}\n\
                 for k in 0..2400usize {{\n\
                 let t = k as f64 / {rate:.1};\n\
                 let x = 0.8 * (2.0 * std::f64::consts::PI * 997.0 * t).sin() + 0.3 * (2.0 * std::f64::consts::PI * 3691.0 * t).sin();\n\
                 {call}\n\
                 let mut h: u64 = 0xcbf29ce484222325;\n\
                 for v in o.iter().chain(st.v_prev.iter()) {{ h = (h ^ v.to_bits()).wrapping_mul(0x100000001b3); }}\n\
                 println!(\"{rate} {label} {{}} {{h:016x}}\", k);\n\
                 }} }}\n",
                label = f.map_or("fixed".to_string(), |f| f.to_string()),
            ));
        }
    }
    format!("fn main() {{\n{body}}}\n")
}

/// Lines `(rate, sample, hash)` of a run, keyed for comparison.
fn run(code: &str, factors: &[Option<usize>], tag: &str) -> Vec<(String, String)> {
    let out = support::compile_and_run(code, &driver(code, factors), tag);
    assert!(
        !out.stdout.is_empty(),
        "{tag} produced no output:\n{}",
        out.stderr
    );
    out.stdout
        .lines()
        .map(|l| {
            let p: Vec<&str> = l.split_whitespace().collect();
            (format!("{} {} {}", p[0], p[1], p[2]), p[3].to_string())
        })
        .collect()
}

/// The Newton budget a build ships.
fn max_iter_of(code: &str) -> Option<usize> {
    let line = code
        .lines()
        .find(|l| l.contains("const MAX_ITER: usize ="))?;
    line.split('=')
        .nth(1)?
        .trim()
        .trim_end_matches(';')
        .parse()
        .ok()
}

/// The runtime build of `spice` at every factor against the fixed build at
/// that factor (budget pinned), bit for bit, on every node.
fn assert_runtime_equals_fixed(spice: &str, route: (&str, NodalSubPathOverride), tag: &str) {
    let rt = build(spice, route, 2, OversamplingSet::Set(SET.to_vec()), None)
        .unwrap_or_else(|e| panic!("{tag}: runtime build refused: {e}"));
    assert!(rt.contains("pub fn set_oversampling("));
    let budget = max_iter_of(&rt);
    let rt_lines = run(&rt, &SET.map(Some), &format!("{tag}_rt"));
    for f in SET {
        let fixed = build(spice, route, f, OversamplingSet::Off, budget)
            .unwrap_or_else(|e| panic!("{tag}: fixed {f}x build refused: {e}"));
        let fx = run(&fixed, &[None], &format!("{tag}_fx{f}"));
        let rt_f: Vec<&(String, String)> = rt_lines
            .iter()
            .filter(|(k, _)| k.split(' ').nth(1) == Some(&f.to_string()))
            .collect();
        assert_eq!(fx.len(), rt_f.len(), "{tag} {f}x: sample counts");
        for ((kf, hf), (kr, hr)) in fx.iter().zip(rt_f) {
            assert!(
                hf == hr,
                "{tag} {f}x: the runtime build departs from the fixed build at {kr} (fixed {kf})"
            );
        }
    }
}

const AUTO: (&str, NodalSubPathOverride) = ("auto", NodalSubPathOverride::Auto);

#[test]
fn every_rate_dependent_feature_is_bit_identical_to_the_fixed_builds() {
    assert_runtime_equals_fixed(GUARD, AUTO, "os_guard");
}

#[test]
fn sub_sample_fire_is_bit_identical_to_the_fixed_builds() {
    assert_runtime_equals_fixed(GLOW, ("nodal", NodalSubPathOverride::Schur), "os_glow");
}

#[test]
fn the_dk_emitter_is_bit_identical_to_the_fixed_builds() {
    assert_runtime_equals_fixed(DK_POT, ("dk", NodalSubPathOverride::Auto), "os_dk");
}

/// Switching mid-stream lands on exactly a fresh state at the new factor:
/// every rate-dependent state (filters, DC blocker, latch reference, histories)
/// is re-seeded. A factor outside the set is refused and changes nothing.
#[test]
fn a_factor_switch_lands_on_a_fresh_state() {
    let code = build(GUARD, AUTO, 2, OversamplingSet::Set(SET.to_vec()), None).unwrap();
    let main = "
fn seg(st: &mut CircuitState, k0: usize, out: &mut Vec<u64>) {
    for k in k0..k0 + 1200 {
        let x = 0.8 * (2.0 * std::f64::consts::PI * 997.0 * k as f64 / 96000.0).sin();
        let (o, _t) = process_sample(x, &[0.25 * x; NUM_INJECT_HOST], &[[0.0; NUM_INJECT_INNER]; MAX_OVERSAMPLING], st);
        let mut h: u64 = 0xcbf29ce484222325;
        for v in o.iter().chain(st.v_prev.iter()) { h = (h ^ v.to_bits()).wrapping_mul(0x100000001b3); }
        out.push(h);
    }
}
fn fresh(f: usize) -> CircuitState {
    let mut st = CircuitState::default();
    st.set_oversampling(f).unwrap();
    st.set_sample_rate(96000.0);
    st
}
fn main() {
    let mut st = CircuitState::default();
    st.set_sample_rate(96000.0);
    let mut warm = Vec::new();
    seg(&mut st, 0, &mut warm);
    st.set_oversampling(4).unwrap();
    let (mut a, mut b) = (Vec::new(), Vec::new());
    seg(&mut st, 1200, &mut a);
    seg(&mut fresh(4), 1200, &mut b);
    println!(\"to4={}\", a == b);
    st.set_oversampling(2).unwrap();
    let (mut c, mut d) = (Vec::new(), Vec::new());
    seg(&mut st, 2400, &mut c);
    seg(&mut fresh(2), 2400, &mut d);
    println!(\"to2={}\", c == d);
    println!(\"refused={}\", st.set_oversampling(3).is_err() && st.oversampling() == 2);
}
";
    let out = support::compile_and_run(&code, main, "os_switch");
    for key in ["to4=true", "to2=true", "refused=true"] {
        assert!(
            out.stdout.contains(key),
            "{key}:\n{}{}",
            out.stdout,
            out.stderr
        );
    }
}

/// The validate common-emitter deck is DK at 1x and 2x and nodal at 4x: one
/// solver cannot serve {1, 2, 4}, so that set is refused, naming the solver;
/// {1, 2} is one solver and builds.
#[test]
fn a_set_whose_factors_build_different_solvers_is_refused() {
    let deck = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../melange-validate/tests/data/bjt_common_emitter/circuit.cir"
    ))
    .unwrap();
    let e = build(&deck, AUTO, 1, OversamplingSet::Set(vec![1, 2, 4]), None).unwrap_err();
    assert!(
        e.contains("is refused") && e.contains("solver: \"dk\" at 1x, \"nodal\" at 4x"),
        "{e}"
    );
    build(&deck, AUTO, 1, OversamplingSet::Set(vec![1, 2]), None)
        .unwrap_or_else(|e| panic!("{{1, 2}} is one solver: {e}"));
}

#[test]
fn the_default_factor_must_be_in_the_set() {
    let e = build(GUARD, AUTO, 2, OversamplingSet::Set(vec![1, 4]), None).unwrap_err();
    assert!(e.contains("not in the runtime set [1, 4]"), "{e}");
}

/// The directive carries the set; `--oversampling-set off` builds one fixed
/// factor from the same deck.
#[test]
fn the_directive_declares_the_set_and_off_builds_fixed() {
    let deck = DK_POT.replace(".end\n", ".oversampling 2 allow=1,2\n.end\n");
    let code = build(&deck, AUTO, 2, OversamplingSet::Deck, None).unwrap();
    assert!(code.contains("pub const OVERSAMPLING_SET: [usize; 2] = [1, 2];"));
    let fixed = build(&deck, AUTO, 2, OversamplingSet::Off, None).unwrap();
    assert!(!fixed.contains("set_oversampling") && !fixed.contains("OVERSAMPLING_SET"));
}

/// Provenance records the set, its default and source, the factors below the
/// deck's `.oversampling` recommendation, and each factor's ring verdict.
#[test]
fn provenance_records_the_set() {
    let deck = DK_POT.replace(".end\n", ".oversampling 2 allow=1,2,4\n.end\n");
    let code = build(&deck, AUTO, 2, OversamplingSet::Deck, None).unwrap();
    let prov = code
        .lines()
        .find_map(|l| l.strip_prefix("// provenance: "))
        .unwrap();
    assert!(prov.contains("\"oversampling\":2,"), "{prov}");
    assert!(
        prov.contains(
            "\"oversampling_set\":{\"factors\":[1,2,4],\"default\":2,\"source\":\"directive\",\
             \"below_recommendation\":[1],\"integration_reason\":{\"1\":"
        ),
        "{prov}"
    );
}
