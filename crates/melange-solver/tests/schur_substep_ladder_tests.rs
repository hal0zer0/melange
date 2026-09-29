//! The adaptive sub-step ladder is one routine on both nodal routes.
//!
//! When a sample's Newton solve fails, the sample is re-solved in full (N×N
//! LU) at a cut timestep, under the integrator of the solve that failed. Full-LU
//! and Schur builds emit the same ladder; a Schur build is a reduced form of
//! the same equations, so its rescue solves them in full and hands the result
//! (v, charge state, device currents) back to the Schur solve of the next
//! sample.
//!
//! Each witness forces the primary Newton solve to fail on the same samples in
//! both builds: the primary loop runs zero iterations there (a test-only patch
//! of the generated code, identical on both routes; the ladder's own loop is
//! untouched). The ladder then runs on exactly those samples, so both builds
//! must report the same sub-step count, hold nothing, and render the same
//! output to within the Newton tolerance: between forced samples each route
//! solves with its own Newton, which is where the two builds differ.

mod support;

use melange_solver::codegen::{CodegenConfig, NodalSubPathOverride};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const FS: f64 = 48000.0;
const N: usize = 2400;
/// Samples `k` with `k % FORCE_EVERY == FORCE_PHASE` fail their primary solve.
const FORCE_EVERY: usize = 7;
const FORCE_PHASE: usize = 3;

const CLIPPER: &str = "diode clipper\nR1 in mid 1k\nC1 mid 0 10n\nD1 mid out DX\nD2 out mid DX\n\
                       R2 out 0 100k\nC2 out 0 1n\nRb mid 0 1Meg\n\
                       .model DX D(IS=2.52n N=1.752 RS=0.568 CJO=4p TT=20n)\n";

const PRIMARY_LOOP: &str = "    for iter in 0..MAX_ITER {";

struct Render {
    y: Vec<f64>,
    substep: u64,
    hold: u64,
}

fn node(spice: &str, name: &str) -> usize {
    let mna = MnaSystem::from_netlist(&Netlist::parse(spice).unwrap()).unwrap();
    mna.node_map[name] - 1
}

fn config(spice: &str, sub_path: NodalSubPathOverride, be: bool) -> CodegenConfig {
    let mut c = support::config_for_spice(spice, FS);
    c.output_nodes = vec![node(spice, "out")];
    c.dc_block = false;
    c.nodal_sub_path_override = sub_path;
    c.backward_euler = be;
    c
}

/// Render `N` samples of a 1 kHz, 3 V sine (after `setup` and 10 silent
/// samples). With `force`, every primary Newton loop runs zero iterations on
/// the forced samples.
fn render(spice: &str, c: &CodegenConfig, setup: &str, force: bool, tag: &str) -> Render {
    let full = support::generate_circuit_code_nodal(spice, c).0;
    assert!(
        full.contains("local refinement of the failing sub-step"),
        "{tag}: build has no sub-step ladder"
    );
    let primaries = full.matches(PRIMARY_LOOP).count();
    assert!(primaries >= 1, "{tag}: primary Newton loop not found");
    let code = full.replace(
        PRIMARY_LOOP,
        "    for iter in 0..(if FORCE_FAIL.load(std::sync::atomic::Ordering::Relaxed) { 0 } else { MAX_ITER }) {",
    );
    let main = format!(
        "static FORCE_FAIL: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    {setup}
    for _ in 0..10 {{ let _ = process_sample(0.0, &mut s); }}
    for k in 0..{N}usize {{
        FORCE_FAIL.store({force} && k % {FORCE_EVERY} == {FORCE_PHASE}, std::sync::atomic::Ordering::Relaxed);
        let x = 3.0 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / {FS:?}).sin();
        println!(\"{{:.17e}}\", process_sample(x, &mut s)[0]);
    }}
    println!(\"substep {{}} hold {{}}\", s.diag_substep_count, s.diag_nr_hold_count);
}}"
    );
    let out = support::compile_and_run(&code, &main, tag).stdout;
    let mut lines: Vec<&str> = out.lines().collect();
    let last: Vec<&str> = lines.pop().unwrap().split_whitespace().collect();
    Render {
        y: lines.iter().map(|l| l.parse().unwrap()).collect(),
        substep: last[1].parse().unwrap(),
        hold: last[3].parse().unwrap(),
    }
}

fn forced_count() -> u64 {
    (0..N).filter(|k| k % FORCE_EVERY == FORCE_PHASE).count() as u64
}

fn worst_diff(a: &[f64], b: &[f64]) -> (f64, f64) {
    let peak = b.iter().fold(0.0f64, |m, &v| m.max(v.abs()));
    let worst = a
        .iter()
        .zip(b)
        .fold(0.0f64, |m, (p, q)| m.max((p - q).abs()));
    (worst, peak)
}

fn parity(be: bool, tag: &str) {
    let schur = render(
        CLIPPER,
        &config(CLIPPER, NodalSubPathOverride::Schur, be),
        "",
        true,
        &format!("{tag}_schur"),
    );
    let full = render(
        CLIPPER,
        &config(CLIPPER, NodalSubPathOverride::FullLu, be),
        "",
        true,
        &format!("{tag}_full_lu"),
    );
    let forced = forced_count();
    assert_eq!(
        (schur.hold, full.hold),
        (0, 0),
        "{tag}: held samples (Schur {}, full-LU {})",
        schur.hold,
        full.hold
    );
    assert_eq!(
        (schur.substep, full.substep),
        (forced, forced),
        "{tag}: every forced sample must be rescued by the ladder, and only those"
    );
    let (worst, peak) = worst_diff(&schur.y, &full.y);
    // Reference: the same two builds unforced, where each solves every
    // sample with its own Newton.
    let free = |sp, t: &str| render(CLIPPER, &config(CLIPPER, sp, be), "", false, t);
    let free_s = free(
        NodalSubPathOverride::Schur,
        &format!("{tag}_schur_unforced"),
    );
    let free_f = free(
        NodalSubPathOverride::FullLu,
        &format!("{tag}_full_lu_unforced"),
    );
    let (free_worst, _) = worst_diff(&free_s.y, &free_f.y);
    eprintln!(
        "{tag}: Schur vs full-LU worst {worst:e} forced, {free_worst:e} unforced, peak {peak:e}"
    );
    assert!(peak > 0.1, "{tag}: the drive never reached the clipper");
    // Measured: 5.6e-6 (trapezoidal) and 8.0e-6 (BE) forced against 1.5e-6 /
    // 1.4e-6 unforced, on a 2.5 V peak; the ladder accepts a Newton step below
    // 1e-3 V. A Schur ladder that does not hand its charge state back moves
    // the trapezoidal render by 6.2e-3.
    assert!(
        worst <= 2e-5 * peak,
        "{tag}: Schur and full-LU differ by {worst:e} on a {peak:e} output \
         (unforced {free_worst:e})"
    );
}

#[test]
fn ladder_parity_trapezoidal() {
    parity(false, "ladder_trap");
}

#[test]
fn ladder_parity_backward_euler() {
    parity(true, "ladder_be");
}

/// A knob-free Schur build reads the compile-time G/C; a knob build must
/// sub-step the circuit its knobs describe (the working copies), not the
/// nominal one.
#[test]
fn schur_ladder_solves_the_pot_setting() {
    let knob_deck = CLIPPER.replace(
        "R1 in mid 1k\n",
        "R1 in mid 1k\n.pot R1 100 10k 1k \"Drive\"\n",
    );
    let native_deck = CLIPPER.replace("R1 in mid 1k\n", "R1 in mid 330\n");
    let knob = render(
        &knob_deck,
        &config(&knob_deck, NodalSubPathOverride::Schur, false),
        "s.set_pot_0(330.0);",
        true,
        "ladder_pot_knob",
    );
    let native = render(
        &native_deck,
        &config(&native_deck, NodalSubPathOverride::Schur, false),
        "",
        true,
        "ladder_pot_native",
    );
    assert_eq!((knob.hold, native.hold), (0, 0));
    assert_eq!(knob.substep, forced_count());
    assert_eq!(native.substep, forced_count());
    let (worst, peak) = worst_diff(&knob.y, &native.y);
    eprintln!("pot: knob vs native worst {worst:e} on peak {peak:e}");
    assert!(
        worst <= 1e-9 * peak,
        "knob build differs from native by {worst:e} on a {peak:e} output"
    );
}
