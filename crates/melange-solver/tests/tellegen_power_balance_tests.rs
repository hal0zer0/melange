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

#[test]
fn tellegen_bjt_common_emitter() {
    // Residual floor here is the DC-OP solver's convergence residual (~1e-11 A;
    // harder multi-device solve than the clipper's runtime NR), not machine
    // epsilon. Tolerances stay ~2 orders above that yet ~6 orders below the
    // mA-scale residual a stamp-sign or VS-handling bug would produce. u=0 so
    // Ic=1.37mA, Ib=6.85µA (β=200) exercise the 2D N_i block and the VS branch.
    check(BJT_CE, 0.0, 2_000, "tellegen_bjt", true, 1e-9, 1e-8);
}
