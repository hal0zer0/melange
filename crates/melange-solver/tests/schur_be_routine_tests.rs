//! One backward-Euler solve per nodal sub-path: the Schur side.
//!
//! A trapezoidal nodal-Schur build ran its latch, breakpoint, failure and
//! ActiveSetBe-rail samples through a separate BE-fallback block with its own
//! limiter and singular-matrix handling. It now runs the same Schur solve
//! routine a `--backward-euler` Schur build runs, on the `_be` kernel
//! (`s_be`/`k_be`/`s_ni_be`, `a_be`/`a_neg_be`), which the IR bakes by the same
//! expressions. So a forced latch is bit-identical to the BE build from the
//! same state, in every rail mode, and stays so across a knob move (the
//! rebuild recomputes the `_be` kernel with the trapezoidal one).
//!
//! Both builds restart with `reset()` from the baked `DC_OP` (the
//! runtime-settled operating point is per-integrator) and are pinned to the
//! Schur sub-path.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, OpampRailMode};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const FS: f64 = 48000.0;

const CLIPPER: &str =
    "clip\nR_1 in a 10k\nD1 a 0 D1N\nD2 0 a D1N\nC1 a 0 10n\nR2 a out 1k\nR3 out 0 100k\n\
                       .model D1N D(IS=2.52n N=1.752)\n.pot R_1 1k 100k 10k \"Drive\"\n";

/// An inverting stage (gain 10) railing at +/-9 V into a diode clipper.
const RAILING: &str = "inverting op-amp stage into a diode clipper\n\
R_in in nm 10k\nR_f nm oa 100k\nU1 0 nm oa OA1\nR_s oa y 1k\nD1 y 0 DCL\nD2 0 y DCL\n\
C_y y 0 10n\nR_o y out 1k\nR_l out 0 100k\n\
.model OA1 OA(AOL=200000 VCC=9 VEE=-9)\n.model DCL D(IS=2.52n N=1.752)\n\
.pot R_f 10k 470k 100k \"Gain\"\n";

fn out_node(spice: &str) -> usize {
    let mna = MnaSystem::from_netlist(&Netlist::parse(spice).unwrap()).unwrap();
    mna.node_map["out"] - 1
}

/// Render 0.5 s of a 1 kHz sine at `amp`, moving pot 0 to each `(sample,
/// ohms)` in `knobs`. `latched` forces the latch from the first sample.
fn run(
    spice: &str,
    mode: OpampRailMode,
    backward_euler: bool,
    amp: f64,
    knobs: &[(usize, f64)],
    tag: &str,
) -> Vec<f64> {
    let mut config = support::config_for_spice(spice, FS);
    config.backward_euler = backward_euler;
    config.nodal_sub_path_override = NodalSubPathOverride::Schur;
    config.opamp_rail_mode = mode;
    let code = support::generate_circuit_code_nodal(spice, &config).0;
    assert!(!code.contains("state.chord_lu"), "{tag}: not a Schur build");
    let latch = if backward_euler {
        assert!(
            !code.contains("Backward-Euler solve: the same routine"),
            "{tag}: a BE build has one solve"
        );
        ""
    } else {
        assert!(code.contains("pub be_latched"), "{tag}: latch not emitted");
        "s.be_latched = true;"
    };
    let start = if code.contains("pub const DC_OP:") {
        "DC_OP"
    } else {
        "[0.0; N]"
    };
    let restart_nl = if code.contains("pub const DC_NL_I:") {
        "s.i_nl_prev = DC_NL_I; s.i_nl_prev_prev = DC_NL_I;"
    } else {
        ""
    };
    let moves: String = knobs
        .iter()
        .map(|(k, r)| format!("if i == {k} {{ s.set_pot_0({r:?}); }}\n"))
        .collect();
    let out = out_node(spice);
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    s.reset();
    s.v_prev = {start};
    {restart_nl}
    s.input_prev = 0.0;
    {latch}
    for i in 0..24000usize {{
        {moves}
        let _ = process_sample({amp:?} * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / {FS:?}).sin(), &mut s);
        println!(\"{{:.17e}}\", s.v_prev[{out}]);
    }}
    if {unsolved} != 0 {{ println!(\"unsolved {{}}\", {unsolved}); }}
}}",
        unsolved = support::unsolved_expr(&code, "s"),
    );
    let stdout = support::compile_and_run(&code, &main, tag).stdout;
    let mut v = Vec::new();
    for l in stdout.lines() {
        assert!(!l.starts_with("unsolved"), "{tag}: {l}");
        v.push(l.parse::<f64>().unwrap());
    }
    v
}

fn assert_bitwise(a: &[f64], b: &[f64], what: &str) {
    assert_eq!(a.len(), b.len());
    if let Some(k) = (0..a.len()).find(|&k| a[k].to_bits() != b[k].to_bits()) {
        panic!(
            "{what}: latched vs BE build first differ at sample {k}: {:e} vs {:e}",
            a[k], b[k]
        );
    }
}

#[test]
fn schur_forced_latch_matches_the_backward_euler_build() {
    for (spice, mode, amp, tag) in [
        (CLIPPER, OpampRailMode::None, 2.0, "clipper"),
        (RAILING, OpampRailMode::ActiveSet, 1.0, "railing_as"),
        (RAILING, OpampRailMode::ActiveSetBe, 1.0, "railing_asbe"),
    ] {
        let latched = run(spice, mode, false, amp, &[], &format!("schur_latch_{tag}"));
        let be = run(spice, mode, true, amp, &[], &format!("schur_be_{tag}"));
        assert_bitwise(&latched, &be, tag);
    }
}

/// A pot moved while latched: the rebuild recomputes the `_be` kernel the
/// latched solve reads, exactly as a BE build recomputes its own.
#[test]
fn schur_knob_move_while_latched_matches_the_backward_euler_build() {
    for (spice, mode, amp, knobs, tag) in [
        (
            CLIPPER,
            OpampRailMode::None,
            2.0,
            [(7000, 1000.0), (15000, 50000.0)],
            "clipper",
        ),
        (
            RAILING,
            OpampRailMode::ActiveSet,
            1.0,
            [(7000, 20000.0), (15000, 400000.0)],
            "railing_as",
        ),
    ] {
        let latched = run(
            spice,
            mode,
            false,
            amp,
            &knobs,
            &format!("schur_knob_latch_{tag}"),
        );
        let be = run(
            spice,
            mode,
            true,
            amp,
            &knobs,
            &format!("schur_knob_be_{tag}"),
        );
        assert_bitwise(&latched, &be, tag);
    }
}
