//! A rescued sample solves its step equations as accurately as a converged
//! step of the ladder's own size.
//!
//! When a sample's Newton solve fails, the sub-step ladder re-solves it in
//! sub-steps of T/2 and finer (bisecting only a sub-step that fails). Its job
//! is to solve the same step equations on those sub-steps, so a rescued sample
//! may carry the truncation error of a T/2 step, never more. A wrong root, a
//! bad charge hand-off between sub-steps or a wrong sub-step input reads as
//! error beyond that. Mutants caught: the trapezoidal charge hand-off without
//! its `- q_dot` term (clipper, 48 mV against a 1.2 mV T/2 error), and every
//! sub-step driven by the end-of-sample input (both decks, 2x and 55x). The
//! internal-node row gap is not visible here (forced rescues of this CE stage
//! converge with or without it); `kcl_rows_cover_internal_nodes_tests` guards
//! it.
//!
//! Each witness forces the primary Newton loop to run zero iterations on a
//! set of samples of a 48 kHz build (a test-only patch; the ladder's own loop
//! is untouched), so those samples are rescued. From each forced sample's
//! pre-step state, three other builds of the same deck take the same step:
//! a 96 kHz build (two T/2 steps: the converged scheme at the ladder's first
//! sub-step size) and 64x and 256x builds, the reference. The reference is
//! shown converged (64x within a small fraction of the T/2 error of 256x),
//! then the rescued error is gated at the T/2 error.
//!
//! The ladder can bisect below T/2, which only lowers its error; the gate
//! allows 1.5x the T/2 error for the mix of sub-step sizes.

mod support;

use melange_solver::codegen::NodalSubPathOverride;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const FS: f64 = 48000.0;
const SAMPLES: usize = 1200;
/// Samples `k` with `k % FORCE_EVERY == FORCE_PHASE` are rescued.
const FORCE_EVERY: usize = 7;
const FORCE_PHASE: usize = 3;
const PRIMARY_LOOP: &str = "    for iter in 0..MAX_ITER {";

/// A diode clipper driven hard (3 V) through its knee.
const CLIPPER: &str = "diode clipper\nR1 in mid 1k\nC1 mid 0 10n\nD1 mid out DX\nD2 out mid DX\n\
                       R2 out 0 100k\nC2 out 0 1n\nRb mid 0 1Meg\n\
                       .model DX D(IS=2.52n N=1.752 RS=0.568 CJO=4p TT=20n)\n";

/// A common-emitter stage whose nodal build expands RB, RC and RE into
/// internal nodes (driven 0.3 V, into clipping).
const CE: &str = "parasitic-RB common emitter\n\
Vcc vcc 0 DC 9\nC_in in b 1u\nR_b1 vcc b 100k\nR_b2 b 0 22k\nQ1 c b e NPN1\n\
R_c vcc c 4.7k\nR_e e 0 1k\nC_e e 0 10u\nC_o c out 1u\nR_l out 0 100k\n\
.model NPN1 NPN(IS=1e-14 BF=200 RB=100 RC=10 RE=1 CJE=10p CJC=5p)\n";

/// The integrator a build solves under; the comparison builds are pinned to
/// the rescued build's, since the ladder's sub-steps run under it.
#[derive(Clone, Copy)]
enum Scheme {
    /// As the build resolves it (the rescued build).
    Shipped,
    Trap,
    BackwardEuler,
}

/// The generated code of `deck` at `rate`, as a module body (crate-level
/// attributes dropped).
fn module(deck: &str, rate: f64, scheme: Scheme) -> String {
    let mut c = support::config_for_spice(deck, rate);
    match scheme {
        Scheme::Shipped => {}
        Scheme::Trap => c.force_trap = true,
        Scheme::BackwardEuler => c.backward_euler = true,
    }
    let mna = MnaSystem::from_netlist(&Netlist::parse(deck).unwrap()).unwrap();
    c.output_nodes = vec![mna.node_map["out"] - 1];
    c.dc_block = false;
    c.nodal_sub_path_override = NodalSubPathOverride::FullLu;
    let code = support::generate_circuit_code_nodal(deck, &c).0;
    code.lines()
        .filter(|l| !l.starts_with("#![") && !l.starts_with("//!"))
        .collect::<Vec<_>>()
        .join("\n")
}

/// (rescued rms error, T/2 rms error, 64x-vs-256x rms, rescued count), all
/// against the 256x reference on the output node, in volts.
fn measure(deck: &str, amp: f64, tag: &str) -> (f64, f64, f64, usize) {
    let base = module(deck, FS, Scheme::Shipped);
    // A trapezoidal build carries the charge state `q_dot`; a backward-Euler
    // build does not.
    let trap = base.contains("pub q_dot: [f64; N]");
    let scheme = if trap {
        Scheme::Trap
    } else {
        Scheme::BackwardEuler
    };
    let copy_q = if trap { "s.q_dot = pre.q_dot;" } else { "" };
    assert!(
        base.contains("local refinement of the failing sub-step"),
        "{tag}: build has no sub-step ladder"
    );
    assert!(
        base.contains(PRIMARY_LOOP),
        "{tag}: primary Newton loop not found"
    );
    let base = base.replace(
        PRIMARY_LOOP,
        "    for iter in 0..(if crate::FORCE_FAIL.load(std::sync::atomic::Ordering::Relaxed) { 0 } else { MAX_ITER }) {",
    );
    let code = format!(
        "#![allow(warnings)]\n\
         static FORCE_FAIL: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);\n\
         mod a {{\n{base}\n}}\n\
         mod h {{\n{}\n}}\n\
         mod r64 {{\n{}\n}}\n\
         mod r256 {{\n{}\n}}\n",
        module(deck, 2.0 * FS, scheme),
        module(deck, 64.0 * FS, scheme),
        module(deck, 256.0 * FS, scheme),
    );
    let main = format!(
        "macro_rules! step_from {{ ($m:ident, $pre:expr, $x:expr, $n:expr) => {{{{
    let pre = $pre; let mut s = $m::CircuitState::default();
    s.v_prev = pre.v_prev; {copy_q} s.i_nl_prev = pre.i_nl_prev;
    s.i_nl_prev_prev = pre.i_nl_prev_prev; s.input_prev = pre.input_prev;
    for j in 1..=$n {{ let xi = pre.input_prev + ($x - pre.input_prev) * j as f64 / $n as f64; let _ = $m::process_sample(xi, &mut s); }}
    assert_eq!(s.diag_unsolved_sample_count, 0, \"a reference step held a sample\");
    s.v_prev[$m::OUTPUT_NODES[0]]
}}}} }}
fn main() {{
    let o = a::OUTPUT_NODES[0];
    let mut s = a::CircuitState::default();
    for _ in 0..10 {{ let _ = a::process_sample(0.0, &mut s); }}
    let (mut er, mut eh, mut e64, mut n) = (0.0f64, 0.0f64, 0.0f64, 0usize);
    for k in 0..{SAMPLES}usize {{
        let forced = k % {FORCE_EVERY} == {FORCE_PHASE};
        FORCE_FAIL.store(forced, std::sync::atomic::Ordering::Relaxed);
        let x = {amp:?} * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / {FS:?}).sin();
        let pre = s.clone();
        let sub0 = s.diag_substep_count;
        let _ = a::process_sample(x, &mut s);
        if forced {{
            assert!(s.diag_substep_count > sub0, \"forced sample {{k}} was not rescued\");
            let r = step_from!(r256, &pre, x, 256);
            let r64 = step_from!(r64, &pre, x, 64);
            let hv = step_from!(h, &pre, x, 2);
            er += (s.v_prev[o] - r).powi(2); eh += (hv - r).powi(2); e64 += (r64 - r).powi(2); n += 1;
        }}
    }}
    assert_eq!(s.diag_unsolved_sample_count, 0, \"the 48 kHz render held a sample\");
    println!(\"{{:e}} {{:e}} {{:e}} {{}}\", (er / n as f64).sqrt(), (eh / n as f64).sqrt(), (e64 / n as f64).sqrt(), n);
}}"
    );
    let out = support::compile_and_run(&code, &main, tag).stdout;
    let f: Vec<&str> = out.split_whitespace().collect();
    (
        f[0].parse().unwrap(),
        f[1].parse().unwrap(),
        f[2].parse().unwrap(),
        f[3].parse().unwrap(),
    )
}

fn assert_rescues_as_accurate_as_a_half_step(deck: &str, amp: f64, tag: &str) {
    let (rescued, half, r64, n) = measure(deck, amp, tag);
    eprintln!(
        "{tag}: rescued {rescued:e} V rms, T/2 step {half:e} V, 64x vs 256x {r64:e} V, n = {n}"
    );
    assert!(n > 100, "{tag}: only {n} rescued samples");
    // The reference is converged at the scale of the gate.
    assert!(
        r64 <= 0.05 * half,
        "{tag}: reference not converged: 64x vs 256x {r64:e} V against a T/2 error of {half:e} V"
    );
    assert!(
        rescued <= 1.5 * half + 1e-9,
        "{tag}: rescued samples {rescued:e} V rms off the converged solution, \
         a converged T/2 step {half:e} V (n = {n})"
    );
}

#[test]
fn a_rescued_diode_clipper_sample_is_as_accurate_as_a_half_step() {
    assert_rescues_as_accurate_as_a_half_step(CLIPPER, 3.0, "ladder_acc_clipper");
}

#[test]
fn a_rescued_sample_with_expanded_internal_nodes_is_as_accurate_as_a_half_step() {
    assert_rescues_as_accurate_as_a_half_step(CE, 0.3, "ladder_acc_ce");
}
