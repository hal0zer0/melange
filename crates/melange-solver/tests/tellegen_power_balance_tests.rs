//! Tellegen / power-balance verification on the generated-and-compiled code.
//!
//! Tellegen's theorem: for any lumped network obeying KCL and KVL,
//! Σ_branches v_k·i_k = 0 — instantaneous power balance, independent of the
//! elements' constitutive laws (linear or nonlinear). Because the branch
//! voltages here derive from node potentials (KVL is automatic), the sum
//! reduces to Σ_node V_node·(net current into node) = V·r, where r is the
//! nodal KCL residual. So a vanishing power sum is equivalent to KCL holding
//! at every node — a global, oracle-free correctness net.
//!
//! These tests drive the *actual generated binary* to a DC steady state,
//! capture its own solved node voltages (`v_prev`) and nonlinear branch
//! currents (`i_nl_prev`), and check the residual computed from melange's
//! own MNA matrices (`g`, `n_i`). At true DC steady state capacitor currents
//! are zero and inductors are shorts, so the physical KCL law is
//!   r = G·v_ss − N_i·i_nl_ss − I_src,   I_src = G_in·u_dc at the input node,
//! with G including the Thevenin input conductance G_in (stamped exactly as
//! codegen does). A converged, correctly-stamped circuit drives r → 0; a
//! sign/scale stamping error, a non-converged Newton solve, or an
//! inconsistent operating point shows up as a nonzero residual on *every*
//! node it touches.

mod support;

use melange_solver::codegen::CodegenConfig;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const SR: f64 = 48_000.0;
const G_IN: f64 = 1.0; // input conductance (R_in = 1Ω), matches codegen default

/// Drive the generated circuit with a constant DC input until it settles,
/// then dump its own `v_prev` (all N node voltages) and `i_nl_prev` (all M
/// nonlinear branch currents). Returns (v_ss, i_nl_ss).
fn settle_and_dump(
    spice: &str,
    dc_input: f64,
    steps: usize,
    tag: &str,
    nodal: bool,
) -> (Vec<f64>, Vec<f64>) {
    let config = CodegenConfig {
        sample_rate: SR,
        ..support::config_for_spice(spice, SR)
    };
    let (code, _n, _m) = if nodal {
        support::generate_circuit_code_nodal(spice, &config)
    } else {
        support::generate_circuit_code(spice, &config)
    };

    let main_code = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr:?});
    let u: f64 = {dc_input:?};
    for _ in 0..{steps} {{ let _ = process_sample(u, &mut state); }}
    let mut v = String::from("V");
    for x in state.v_prev.iter() {{ v.push_str(&format!(" {{:.17e}}", x)); }}
    println!("{{v}}");
    let mut i = String::from("I");
    for x in state.i_nl_prev.iter() {{ i.push_str(&format!(" {{:.17e}}", x)); }}
    println!("{{i}}");
}}
"#,
        sr = SR,
        dc_input = dc_input,
        steps = steps,
    );

    let out = support::compile_and_run(&code, &main_code, tag);
    let parse_line = |prefix: &str| -> Vec<f64> {
        out.stdout
            .lines()
            .find(|l| l.starts_with(prefix))
            .unwrap_or_else(|| panic!("missing '{prefix}' line in output:\n{}", out.stdout))
            .split_whitespace()
            .skip(1)
            .map(|t| t.parse::<f64>().unwrap())
            .collect()
    };
    (parse_line("V"), parse_line("I"))
}

/// Build the MNA system the way codegen does (G_in stamped at the input node)
/// and return the full augmented residual r = G·x − N_i·i_nl − rhs over all
/// n_aug rows. For plain node rows this is the KCL residual; for voltage-source
/// branch rows (`n + ext_idx`) it is the constraint residual v[n+]−v[n−]−v_dc.
///   rhs[input_node]      = G_in·u_dc   (Thevenin Norton input current)
///   rhs[n + vs.ext_idx]  = vs.dc_value (VS voltage constraint)
/// With no voltage sources, n_aug = n and this is exactly the earlier check.
fn augmented_residual(spice: &str, x: &[f64], i_nl_ss: &[f64], dc_input: f64) -> Vec<f64> {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    mna.g[input_node][input_node] += G_IN;

    let n_aug = mna.n_aug;
    assert_eq!(x.len(), n_aug, "captured state length must match MNA n_aug");

    // rhs vector
    let mut rhs = vec![0.0_f64; n_aug];
    rhs[input_node] = G_IN * dc_input;
    for vs in &mna.voltage_sources {
        rhs[mna.n + vs.ext_idx] = vs.dc_value;
    }

    let mut r = vec![0.0_f64; n_aug];
    for k in 0..n_aug {
        let mut acc = 0.0;
        for j in 0..n_aug {
            acc += mna.g[k][j] * x[j];
        }
        // − N_i·i_nl (augmented/VS rows carry no device stamps → row absent or 0)
        if k < mna.n_i.len() {
            for (col, &i_val) in i_nl_ss.iter().enumerate() {
                acc -= mna.n_i[k][col] * i_val;
            }
        }
        r[k] = acc - rhs[k];
    }
    r
}

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0_f64, |m, &x| m.max(x.abs()))
}

fn tellegen_power(v_ss: &[f64], r: &[f64]) -> f64 {
    v_ss.iter().zip(r).map(|(&v, &ri)| v * ri).sum()
}

// ── Dynamic (transient) Tellegen ────────────────────────────────────────
//
// DC steady state is the easy case (reactive currents vanish). The dynamic
// check drives a time-varying input and verifies power balance at *every*
// sample, including the companion capacitor currents
//   i_cap[n] = α·C·(v[n]−v[n−1]) − i_cap[n−1],   α = 2/T,
// a vector recursion seeded from the DC operating point (i_cap[−1]=0,
// v[−1]=v_dc). The instantaneous nodal residual is
//   r[n] = G·v[n] + i_cap[n] − N_i·i_nl[n] − rhs_src(u[n]).

/// One captured transient sample: (input u[n], full state v[n], nonlinear i_nl[n]).
type Sample = (f64, Vec<f64>, Vec<f64>);

/// Drive the generated circuit with an internally-generated sine and capture,
/// per sample, (u, v_prev, i_nl_prev), plus the initial DC-OP baseline v[−1].
fn dump_transient(
    spice: &str,
    freq: f64,
    amp: f64,
    steps: usize,
    tag: &str,
    nodal: bool,
) -> (Vec<f64>, Vec<Sample>) {
    let config = CodegenConfig {
        sample_rate: SR,
        ..support::config_for_spice(spice, SR)
    };
    let (code, _n, _m) = if nodal {
        support::generate_circuit_code_nodal(spice, &config)
    } else {
        support::generate_circuit_code(spice, &config)
    };

    let main_code = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr:?});
    // DC-OP baseline v[-1]
    let mut b = String::from("B");
    for x in state.v_prev.iter() {{ b.push_str(&format!(" {{:.17e}}", x)); }}
    println!("{{b}}");
    let sr: f64 = {sr:?};
    for n in 0..{steps} {{
        let t = n as f64 / sr;
        let u: f64 = {amp:?} * (2.0 * std::f64::consts::PI * {freq:?} * t).sin();
        let _ = process_sample(u, &mut state);
        let mut sline = format!("S {{:.17e}}", u);
        for x in state.v_prev.iter() {{ sline.push_str(&format!(" {{:.17e}}", x)); }}
        println!("{{sline}}");
        let mut nline = String::from("N");
        for x in state.i_nl_prev.iter() {{ nline.push_str(&format!(" {{:.17e}}", x)); }}
        println!("{{nline}}");
    }}
}}
"#,
        sr = SR,
        amp = amp,
        freq = freq,
        steps = steps,
    );

    let out = support::compile_and_run(&code, &main_code, tag);
    let nums = |l: &str| -> Vec<f64> {
        l.split_whitespace()
            .skip(1)
            .map(|t| t.parse::<f64>().unwrap())
            .collect()
    };
    let mut baseline = Vec::new();
    let mut samples: Vec<Sample> = Vec::new();
    let mut lines = out.stdout.lines();
    while let Some(l) = lines.next() {
        if let Some(rest) = l.strip_prefix("B ") {
            baseline = nums(&format!("B {rest}"));
        } else if l.starts_with("S ") {
            let sv = nums(l);
            let (u, v) = sv.split_first().unwrap();
            let nline = lines.next().expect("N line follows S line");
            let i_nl = nums(nline);
            samples.push((*u, v.to_vec(), i_nl));
        }
    }
    (baseline, samples)
}

/// Matrix·vector over the augmented dimension.
fn matvec(m: &[Vec<f64>], x: &[f64]) -> Vec<f64> {
    m.iter()
        .map(|row| row.iter().zip(x).map(|(&a, &b)| a * b).sum())
        .collect()
}

/// Return (max|r[n]|, max|r[n]+r[n−1]|) over the transient — the instantaneous
/// and trapezoidal-pairwise power-balance residuals.
fn dynamic_residual(spice: &str, baseline: &[f64], samples: &[Sample]) -> (f64, f64) {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    mna.g[input_node][input_node] += G_IN;
    let n_aug = mna.n_aug;
    let alpha = 2.0 * SR;

    let mut v_prev = baseline.to_vec();
    let mut i_cap_prev = vec![0.0_f64; n_aug];
    let mut r_prev = vec![0.0_f64; n_aug];
    let mut max_r = 0.0_f64;
    let mut max_pair = 0.0_f64;

    for (u, v, i_nl) in samples {
        let dv: Vec<f64> = v.iter().zip(&v_prev).map(|(&a, &b)| a - b).collect();
        let cdv = matvec(&mna.c, &dv);
        let i_cap: Vec<f64> = cdv
            .iter()
            .zip(&i_cap_prev)
            .map(|(&c, &ip)| alpha * c - ip)
            .collect();

        let gv = matvec(&mna.g, v);
        let ni = matvec(&mna.n_i, i_nl); // n_aug × m · m → n_aug
        let mut rhs = vec![0.0_f64; n_aug];
        rhs[input_node] = G_IN * u;
        for vs in &mna.voltage_sources {
            rhs[mna.n + vs.ext_idx] = vs.dc_value;
        }

        let r: Vec<f64> = (0..n_aug)
            .map(|k| gv[k] + i_cap[k] - ni.get(k).copied().unwrap_or(0.0) - rhs[k])
            .collect();

        max_r = max_r.max(max_abs(&r));
        for k in 0..n_aug {
            max_pair = max_pair.max((r[k] + r_prev[k]).abs());
        }

        v_prev = v.clone();
        i_cap_prev = i_cap;
        r_prev = r;
    }
    (max_r, max_pair)
}

#[allow(clippy::too_many_arguments)]
fn check(
    spice: &str,
    dc_input: f64,
    steps: usize,
    tag: &str,
    nodal: bool,
    kcl_tol: f64,
    pwr_tol: f64,
) {
    let (v_ss, i_nl_ss) = settle_and_dump(spice, dc_input, steps, tag, nodal);
    let r = augmented_residual(spice, &v_ss, &i_nl_ss, dc_input);
    let kcl = max_abs(&r);
    let pwr = tellegen_power(&v_ss, &r).abs();
    eprintln!("[{tag}] max|KCL residual| = {kcl:.3e} A,  |Tellegen power| = {pwr:.3e} W");
    assert!(
        kcl < kcl_tol,
        "[{tag}] KCL residual {kcl:.3e} A exceeds {kcl_tol:.3e} — node currents do not balance"
    );
    assert!(
        pwr < pwr_tol,
        "[{tag}] Tellegen power imbalance {pwr:.3e} W exceeds {pwr_tol:.3e}"
    );
}

// Linear resistive divider — pure KCL, no reactive or nonlinear terms.
const DIVIDER: &str = "Divider\nR1 in mid 1k\nR2 mid 0 1k\n";

// RC lowpass — adds a reactive element (zero current at DC steady state).
const RC_LOWPASS: &str = "RC Lowpass\nR1 in out 1k\nC1 out 0 1u\n";

// Anti-parallel diode clipper — the nonlinear case Tellegen is really for.
const DIODE_CLIPPER: &str = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";

// Tolerances are ~100–1000× above the measured machine-precision residuals
// (divider 0, RC 3.6e-14, clipper 1.8e-15) yet ~8 orders below the mA-scale
// residual a genuine stamp-sign or convergence bug would produce.
const KCL_TOL: f64 = 1e-11;
const PWR_TOL: f64 = 1e-11;

// BJT common-emitter with a VCC voltage source — exercises the augmented-MNA
// VS branch row AND 2D (collector/base) multi-terminal N_i stamping. Simple
// NPN model (no RB/RE/RC parasitics ⇒ no internal-node expansion). u=0 checks
// the baked DC operating point directly (no fighting the 10µF coupling caps).
const BJT_CE: &str = "\
BJT CE
VCC vcc 0 DC 12
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit NPNMOD
RC vcc coll 6.8k
RE emit 0 1k
C1 in base 10u
C2 coll out 10u
Rload out 0 100k
.model NPNMOD NPN(IS=1e-14 BF=200)
";

#[test]
fn tellegen_divider_linear() {
    check(
        DIVIDER,
        1.0,
        4_000,
        "tellegen_divider",
        false,
        KCL_TOL,
        PWR_TOL,
    );
}

#[test]
fn tellegen_rc_lowpass() {
    check(
        RC_LOWPASS,
        1.0,
        48_000,
        "tellegen_rc",
        false,
        KCL_TOL,
        PWR_TOL,
    );
}

#[test]
fn tellegen_diode_clipper_nonlinear() {
    // 2V into a 1k series with anti-parallel diodes: a diode conducts ~1.5mA,
    // so i_nl is non-trivial and the balance genuinely exercises N_i·i_nl.
    check(
        DIODE_CLIPPER,
        2.0,
        48_000,
        "tellegen_clipper",
        false,
        KCL_TOL,
        PWR_TOL,
    );
}

/// Drive a transient sine and assert instantaneous power balance at *every*
/// sample. `tol` is on max|r[n]| (the instantaneous residual), which the
/// trapezoidal solve holds exactly when seeded from a DC-steady start.
#[allow(clippy::too_many_arguments)]
fn check_dynamic(spice: &str, freq: f64, amp: f64, steps: usize, tag: &str, nodal: bool, tol: f64) {
    let (baseline, samples) = dump_transient(spice, freq, amp, steps, tag, nodal);
    let (max_r, max_pair) = dynamic_residual(spice, &baseline, &samples);
    eprintln!(
        "[{tag}] instantaneous max|r[n]| = {max_r:.3e},  pairwise max|r[n]+r[n-1]| = {max_pair:.3e}"
    );
    assert!(
        max_r < tol,
        "[{tag}] dynamic instantaneous power balance {max_r:.3e} exceeds {tol:.3e} — \
         node currents do not balance during the transient"
    );
}

#[test]
fn tellegen_dynamic_rc() {
    // Transient sine into the RC lowpass: the cap carries real current every
    // sample, so this exercises the companion i_cap term dynamic_residual adds.
    check_dynamic(RC_LOWPASS, 1000.0, 1.0, 400, "dyn_rc", false, 1e-11);
}

#[test]
fn tellegen_dynamic_diode_clipper() {
    // 2V sine drives the diodes into conduction on each half-cycle; the balance
    // holds at every point of the clipping waveform (diode + cap + node V).
    check_dynamic(DIODE_CLIPPER, 1000.0, 2.0, 400, "dyn_clip", false, 1e-11);
}

#[test]
fn tellegen_dynamic_bjt_common_emitter() {
    // Small-signal sine on the biased BJT: 2D device current + VS branch + cap
    // currents all balance every sample (floor = DC-OP/NR convergence residual).
    check_dynamic(BJT_CE, 1000.0, 0.01, 400, "dyn_bjt", true, 1e-8);
}

#[test]
fn tellegen_bjt_common_emitter() {
    // Residual floor here is the DC-OP solver's convergence residual (~1e-11 A;
    // harder multi-device solve than the clipper's runtime NR), not machine
    // epsilon. Tolerances stay ~2 orders above that yet ~6 orders below the
    // mA-scale residual a stamp-sign or VS-handling bug would produce. u=0 so
    // Ic=1.37mA, Ib=6.85µA (β=200) exercise the 2D N_i block and the VS branch.
    check(BJT_CE, 0.0, 2_000, "tellegen_bjt", true, 1e-9, 1e-8);
}
