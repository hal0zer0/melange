//! Transition-BE: an op-amp rail pin or release under `ActiveSet` arms one
//! backward-Euler sample (the breakpoint-BE countdown's third source).
//!
//! A pin replaces the op-amp's output row with the rail constraint; a release
//! gives it back. The sample that makes the swap is solved on trapezoidal
//! history built on the old equation set, and the mismatch lands in trap's
//! `z=-1` mode. On a capless nonlinear row (the diode node of a clipper behind
//! a coupling cap) that mode never decays: the row satisfies only the average
//! of its KCL over two samples, the residual alternates in sign, and with a
//! pin and a release every half cycle it accumulates.
//!
//! The witness is that row's own KCL residual, computed from the committed
//! node voltages with the generated code's diode constants. The mutant is the
//! same generated source with the arming statement removed.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, OpampRailMode};

/// Single-supply overdrive (gain ~107, rails 0/9 V) into a diode clipper. `n2`
/// is capless: R_1 from the coupling cap, R_t to the output filter, the diodes.
const OVERDRIVE: &str = "single-supply op-amp overdrive into a diode clipper\n\
Vcc vcc 0 DC 9\nR_b1 vcc vbias 100k\nR_b2 vbias 0 100k\nC_b vbias 0 10u\n\
C_in in np 100n\nR_in np vbias 1Meg\nU1 np nm oa TL072\nR_f oa nm 500k\n\
R_g nm ng 4.7k\nC_g ng 0 10u\nC_c oa n1 1u\nR_1 n1 n2 1k\nD_1 n2 0 D1N914\n\
D_2 0 n2 D1N914\nR_t n2 n3 10k\nC_t n3 0 22n\nC_o n3 out 1u\nR_v out 0 100k\n\
.model TL072 OA(AOL=200000 VCC=9 VEE=0)\n.model D1N914 D(IS=2.52n N=1.752)\n";

const ARM: &str = "state.breakpoint_be = state.breakpoint_be.max(BREAKPOINT_BE_SAMPLES);";

const FS: f64 = 96000.0;

fn code_for(sub_path: NodalSubPathOverride, mode: OpampRailMode) -> String {
    let mut config = support::config_for_spice(OVERDRIVE, FS);
    config.nodal_sub_path_override = sub_path;
    config.opamp_rail_mode = mode;
    support::generate_circuit_code_nodal(OVERDRIVE, &config).0
}

struct Run {
    /// max |KCL residual at n2| over the last 50 ms, amps.
    residual: f64,
    /// Pin-state changes seen in the committed op-amp output.
    pin_changes: u64,
    transition_be: u64,
    be_fallback: u64,
}

/// Render 0.5 s of a 1 kHz, 0.5 V sine (the op-amp rails every half cycle).
fn run(code: &str, tag: &str) -> Run {
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    let n = {n}usize;
    let tail = n - {tail}usize;
    let (is, nvt) = (s.device_0_is, s.device_0_n_vt);
    assert_eq!((is, nvt), (s.device_1_is, s.device_1_n_vt));
    let pin = |v: f64| if v >= 9.0 {{ 1u8 }} else if v <= 0.0 {{ 2u8 }} else {{ 0u8 }};
    let mut prev_pin = pin(s.v_prev[NODE_OA]);
    let mut changes = 0u64;
    let mut worst = 0.0f64;
    for i in 0..n {{
        let _ = process_sample(0.5 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / {FS:?}).sin(), &mut s);
        let v = s.v_prev;
        let p = pin(v[NODE_OA]);
        if p != prev_pin {{ changes += 1; }}
        prev_pin = p;
        if i >= tail {{
            let x = v[NODE_N2];
            let id = is * ((x / nvt).exp() - 1.0) - is * ((-x / nvt).exp() - 1.0);
            let r = (v[NODE_N1] - x) / 1e3 - (x - v[NODE_N3]) / 1e4 - id;
            worst = worst.max(r.abs());
        }}
    }}
    assert_eq!(s.diag_nr_unconverged_commit_count, 0, \"unsolved samples\");
    println!(\"residual={{:e}}\", worst);
    println!(\"pin_changes={{}}\", changes);
    println!(\"transition_be={{}}\", s.diag_transition_be_count);
    println!(\"be_fallback={{}}\", s.diag_be_fallback_count);
}}",
        n = (0.5 * FS) as usize,
        tail = (0.05 * FS) as usize,
    );
    let out = support::compile_and_run(code, &main, tag);
    Run {
        residual: out.parse_kv("residual").unwrap(),
        pin_changes: out.parse_kv("pin_changes").unwrap() as u64,
        transition_be: out.parse_kv("transition_be").unwrap() as u64,
        be_fallback: out.parse_kv("be_fallback").unwrap() as u64,
    }
}

#[test]
fn transition_be_is_emitted_only_where_a_pin_can_happen_on_trap() {
    let armed = code_for(NodalSubPathOverride::Auto, OpampRailMode::ActiveSet);
    assert!(armed.contains(ARM) && armed.contains("pub diag_transition_be_count"));
    for (mode, why) in [
        (
            OpampRailMode::ActiveSetBe,
            "ActiveSetBe already solves engaged samples on BE",
        ),
        (
            OpampRailMode::Hard,
            "a hard clamp does not swap the equation set",
        ),
        (OpampRailMode::None, "nothing is clamped"),
    ] {
        let code = code_for(NodalSubPathOverride::Auto, mode);
        assert!(!code.contains("pin_transition"), "{mode:?}: {why}");
    }
    let mut config = support::config_for_spice(OVERDRIVE, FS);
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    config.backward_euler = true;
    let be = support::generate_circuit_code_nodal(OVERDRIVE, &config).0;
    assert!(
        !be.contains("pin_transition"),
        "a BE build has nothing to arm"
    );
}

#[test]
fn a_pin_transition_arms_one_backward_euler_sample_on_both_sub_paths() {
    for (sub_path, tag) in [
        (NodalSubPathOverride::Schur, "tbe_schur"),
        (NodalSubPathOverride::FullLu, "tbe_full_lu"),
    ] {
        let code = code_for(sub_path, OpampRailMode::ActiveSet);
        assert_eq!(
            code.contains("state.chord_lu"),
            sub_path == NodalSubPathOverride::FullLu
        );
        assert_eq!(code.matches(ARM).count(), 1, "{tag}: one arming site");
        let armed = run(&code, tag);
        let mutant = run(&code.replace(ARM, ""), &format!("{tag}_mutant"));
        eprintln!(
            "{tag}: pin changes {}, transition-BE {}, n2 residual {:.3e} A (mutant {:.3e} A)",
            armed.pin_changes, armed.transition_be, armed.residual, mutant.residual
        );

        // Every pin and every release armed exactly one BE sample, and no other
        // sample left trapezoidal (this deck has no pot, switch or failure).
        assert!(
            armed.pin_changes >= 900,
            "{tag}: the op-amp must rail ({})",
            armed.pin_changes
        );
        assert_eq!(armed.transition_be, armed.pin_changes, "{tag}");
        assert_eq!(armed.be_fallback, armed.transition_be, "{tag}");
        assert_eq!(mutant.be_fallback, 0, "{tag}: the mutant never arms");

        // The mutant carries the alternating residual (~230 uA measured); the
        // armed build sits near the Newton acceptance floor.
        assert!(
            mutant.residual > 50e-6,
            "{tag}: mutant n2 residual {:e} A — the witness no longer sees the lock",
            mutant.residual
        );
        assert!(
            armed.residual < 5e-6,
            "{tag}: n2 KCL residual {:e} A with transition-BE",
            armed.residual
        );
    }
}

/// A linear (M=0) inverting stage railing at +/-9 V, capacitor-coupled to its
/// load. Linear circuits have their own solve on each sub-path; the rail
/// resolve there must run on the breakpoint sample's own (BE) matrices.
const LINEAR_RAILING: &str = "linear inverting stage railing into a cap-coupled load\n\
R_in in nm 10k\nR_f nm oa 100k\nU1 0 nm oa OA1\nC_c oa y 1u\nR_y y out 1k\n\
C_y out 0 10n\nR_l out 0 100k\n.model OA1 OA(AOL=200000 VCC=9 VEE=-9)\n";

/// Render 0.25 s of a 1 kHz, 2 V sine from `reset()` at the baked DC OP, printing
/// `out`. `force_be` solves every sample on the breakpoint path.
fn run_linear(sub_path: NodalSubPathOverride, backward_euler: bool, force_be: bool) -> Vec<f64> {
    let mut config = support::config_for_spice(LINEAR_RAILING, 48000.0);
    config.nodal_sub_path_override = sub_path;
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    config.backward_euler = backward_euler;
    let code = support::generate_circuit_code_nodal(LINEAR_RAILING, &config).0;
    assert_eq!(code.contains("pin_transition"), !backward_euler);
    let start = if code.contains("pub const DC_OP:") {
        "DC_OP"
    } else {
        "[0.0; N]"
    };
    let force = if force_be { "s.breakpoint_be = 1;" } else { "" };
    let tail = if backward_euler {
        String::new()
    } else {
        "eprintln!(\"transition_be={}\", s.diag_transition_be_count); eprintln!(\"be_changes={}\", changes);".into()
    };
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    s.reset();
    s.v_prev = {start};
    s.input_prev = 0.0;
    let pin = |v: f64| if v >= 9.0 {{ 1u8 }} else if v <= -9.0 {{ 2u8 }} else {{ 0u8 }};
    let mut prev = pin(s.v_prev[NODE_OA]);
    let mut changes = 0u64;
    for i in 0..12000usize {{
        {force}
        let _ = process_sample(2.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin(), &mut s);
        let p = pin(s.v_prev[NODE_OA]);
        if p != prev {{ changes += 1; }}
        prev = p;
        println!(\"{{:.17e}}\", s.v_prev[NODE_OUT]);
    }}
    let _ = changes;
    {tail}
}}"
    );
    let tag = format!("tbe_linear_{sub_path:?}_{backward_euler}_{force_be}");
    let out = support::compile_and_run(&code, &main, &tag);
    if !backward_euler && !force_be {
        let tbe = out.parse_kv("transition_be").unwrap();
        let changes = out.parse_kv("be_changes").unwrap();
        assert!(changes >= 20.0, "{tag}: the stage must rail ({changes})");
        assert_eq!(tbe, changes, "{tag}: one arm per pin change");
    }
    out.parse_samples()
}

#[test]
fn a_linear_breakpoint_sample_is_the_backward_euler_build_sample() {
    for sub_path in [NodalSubPathOverride::Schur, NodalSubPathOverride::FullLu] {
        let forced = run_linear(sub_path, false, true);
        let be = run_linear(sub_path, true, false);
        assert_eq!(forced.len(), be.len());
        if let Some(k) = (0..be.len()).find(|&k| forced[k].to_bits() != be[k].to_bits()) {
            panic!(
                "{sub_path:?}: forced breakpoint vs BE build first differ at sample {k}: {:e} vs {:e}",
                forced[k], be[k]
            );
        }
        // And the unforced trap build arms on its own pin changes.
        let _ = run_linear(sub_path, false, false);
    }
}
