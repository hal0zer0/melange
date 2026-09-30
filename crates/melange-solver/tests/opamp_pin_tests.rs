//! An op-amp rail pin or release under `ActiveSet` on a trapezoidal build.
//!
//! A pin replaces the op-amp's output row with the rail constraint; a release
//! gives it back. Under the charge form both commit a consistent capacitor
//! current (`q_dot`), so the swap leaves no carried residual and needs no
//! backward-Euler sample. A BE sample there would re-seed `q_dot` with a
//! backward difference across the edge, which costs accuracy: on this deck at
//! 96 kHz it held the output 14.7 mV rms from a 768 kHz render, against 4.1 mV
//! without it.
//!
//! The residual witness is the capless diode row's own KCL residual, computed
//! from the committed node voltages with the generated code's diode constants.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, OpampRailMode};

/// Single-supply overdrive (gain ~107, supply 0/9 V, swing 1.5/7.5 V with the
/// default 1.5 V drops) into a diode clipper. `n2`
/// is capless: R_1 from the coupling cap, R_t to the output filter, the diodes.
const OVERDRIVE: &str = "single-supply op-amp overdrive into a diode clipper\n\
Vcc vcc 0 DC 9\nR_b1 vcc vbias 100k\nR_b2 vbias 0 100k\nC_b vbias 0 10u\n\
C_in in np 100n\nR_in np vbias 1Meg\nU1 np nm oa TL072\nR_f oa nm 500k\n\
R_g nm ng 4.7k\nC_g ng 0 10u\nC_c oa n1 1u\nR_1 n1 n2 1k\nD_1 n2 0 D1N914\n\
D_2 0 n2 D1N914\nR_t n2 n3 10k\nC_t n3 0 22n\nC_o n3 out 1u\nR_v out 0 100k\n\
.model TL072 OA(AOL=200000 VCC=9 VEE=0)\n.model D1N914 D(IS=2.52n N=1.752)\n";

fn code_for(sub_path: NodalSubPathOverride, fs: f64) -> String {
    let mut config = support::config_for_spice(OVERDRIVE, fs);
    config.nodal_sub_path_override = sub_path;
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    support::generate_circuit_code_nodal(OVERDRIVE, &config).0
}

struct Run {
    /// max |KCL residual at n2| over the last 50 ms, amps.
    residual: f64,
    /// Pin-state changes seen in the committed op-amp output.
    pin_changes: u64,
    be_fallback: u64,
}

/// Render 0.5 s of a 1 kHz, 0.5 V sine at 96 kHz (the op-amp rails every half cycle).
fn run(code: &str, tag: &str) -> Run {
    let fs = 96000.0;
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({fs:?});
    let n = {n}usize;
    let tail = n - {tail}usize;
    let (is, nvt) = (s.device_0_is, s.device_0_n_vt);
    assert_eq!((is, nvt), (s.device_1_is, s.device_1_n_vt));
    let pin = |v: f64| if v >= 7.5 {{ 1u8 }} else if v <= 1.5 {{ 2u8 }} else {{ 0u8 }};
    let mut prev_pin = pin(s.v_prev[NODE_OA]);
    let mut changes = 0u64;
    let mut worst = 0.0f64;
    for i in 0..n {{
        let _ = process_sample(0.5 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / {fs:?}).sin(), &mut s);
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
    assert_eq!({unsolved}, 0, \"unsolved samples\");
    println!(\"residual={{:e}}\", worst);
    println!(\"pin_changes={{}}\", changes);
    println!(\"be_fallback={{}}\", s.diag_be_fallback_count);
}}",
        n = (0.5 * fs) as usize,
        tail = (0.05 * fs) as usize,
        unsolved = support::unsolved_expr(code, "s"),
    );
    let out = support::compile_and_run(code, &main, tag);
    Run {
        residual: out.parse_kv("residual").unwrap(),
        pin_changes: out.parse_kv("pin_changes").unwrap() as u64,
        be_fallback: out.parse_kv("be_fallback").unwrap() as u64,
    }
}

#[test]
fn a_pin_or_release_takes_no_backward_euler_sample() {
    for (sub_path, tag) in [
        (NodalSubPathOverride::Schur, "pin_schur"),
        (NodalSubPathOverride::FullLu, "pin_full_lu"),
    ] {
        let code = code_for(sub_path, 96000.0);
        assert_eq!(
            code.contains("state.chord_lu"),
            sub_path == NodalSubPathOverride::FullLu
        );
        assert!(
            !code.contains("pin_transition") && !code.contains("pub breakpoint_be"),
            "{tag}: nothing arms a BE sample on a pin"
        );
        let r = run(&code, tag);
        eprintln!(
            "{tag}: pin changes {}, BE samples {}, n2 residual {:.3e} A",
            r.pin_changes, r.be_fallback, r.residual
        );
        assert!(
            r.pin_changes >= 900,
            "{tag}: the op-amp must rail ({})",
            r.pin_changes
        );
        assert_eq!(r.be_fallback, 0, "{tag}: every sample is trapezoidal");
        // The Newton acceptance floor (measured 0.29 uA).
        assert!(
            r.residual < 5e-6,
            "{tag}: n2 KCL residual {:e} A",
            r.residual
        );
    }
}

/// Node voltages (`oa`, `out`) at 48 kHz instants over 0.2 s of a 1 kHz, 0.5 V
/// sine, rendered at `fs` (a multiple of 48 kHz).
fn trajectory(fs: f64, tag: &str) -> Vec<(f64, f64)> {
    let code = code_for(NodalSubPathOverride::Schur, fs);
    let main = format!(
        "fn main() {{
    let fs: f64 = {fs:?};
    let k = (fs / 48000.0).round() as usize;
    let mut s = CircuitState::default();
    s.set_sample_rate(fs);
    for i in 0..(0.2 * fs) as usize {{
        let _ = process_sample(0.5 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / fs).sin(), &mut s);
        if i % k == 0 {{ println!(\"{{:.17e}} {{:.17e}}\", s.v_prev[NODE_OA], s.v_prev[NODE_OUT]); }}
    }}
}}"
    );
    support::compile_and_run(&code, &main, tag)
        .stdout
        .lines()
        .map(|l| {
            let v: Vec<f64> = l.split_whitespace().map(|t| t.parse().unwrap()).collect();
            (v[0], v[1])
        })
        .collect()
}

/// The pinned deck at 96 kHz against a 768 kHz render of the same build, over
/// the last 0.1 s. Measured: op-amp output 0.35 mV rms, `out` 4.1 mV rms (the
/// latter dominated by where the rail edge falls inside a sample).
#[test]
fn a_pinned_render_converges_to_the_high_rate_render() {
    let reference = trajectory(768000.0, "pin_ref_768k");
    let render = trajectory(96000.0, "pin_96k");
    assert_eq!(reference.len(), render.len());
    let tail = &reference.len() / 2..reference.len();
    let rms = |f: &dyn Fn(usize) -> f64| {
        (tail.clone().map(|i| f(i).powi(2)).sum::<f64>() / tail.len() as f64).sqrt()
    };
    let oa = rms(&|i| render[i].0 - reference[i].0);
    let out = rms(&|i| render[i].1 - reference[i].1);
    eprintln!("96 kHz vs 768 kHz: oa {oa:.3e} V rms, out {out:.3e} V rms");
    assert!(
        oa < 1e-3,
        "op-amp output {oa:e} V rms from the 768 kHz render"
    );
    assert!(out < 6e-3, "out {out:e} V rms from the 768 kHz render");
}

/// A linear (M=0) inverting stage railing at +/-9 V, capacitor-coupled to its
/// load, with a switched capacitor so the breakpoint machinery is emitted
/// (a reactance change is what arms it). Linear circuits
/// have their own solve on each sub-path; the rail resolve there must run on
/// the breakpoint sample's own (BE) matrices.
const LINEAR_RAILING: &str = "linear inverting stage railing into a cap-coupled load\n\
R_in in nm 10k\nR_f nm oa 100k\nU1 0 nm oa OA1\nC_c oa y 1u\nR_y y out 1k\n\
C_y out 0 10n\nR_l out 0 100k\n.switch C_y 10n 22n \"Cy\"\n.model OA1 OA(AOL=200000 VCC=9 VEE=-9)\n";

/// Render 0.25 s of a 1 kHz, 2 V sine from `reset()` at the baked DC OP, printing
/// `out`. `force_be` solves every sample on the breakpoint path.
fn run_linear(sub_path: NodalSubPathOverride, backward_euler: bool, force_be: bool) -> Vec<f64> {
    let mut config = support::config_for_spice(LINEAR_RAILING, 48000.0);
    config.nodal_sub_path_override = sub_path;
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    config.backward_euler = backward_euler;
    let code = support::generate_circuit_code_nodal(LINEAR_RAILING, &config).0;
    assert_eq!(code.contains("pub breakpoint_be"), !backward_euler);
    let start = if code.contains("pub const DC_OP:") {
        "DC_OP"
    } else {
        "[0.0; N]"
    };
    let force = if force_be { "s.breakpoint_be = 1;" } else { "" };
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    s.reset();
    s.v_prev = {start};
    s.input_prev = 0.0;
    for i in 0..12000usize {{
        {force}
        let _ = process_sample(2.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin(), &mut s);
        println!(\"{{:.17e}}\", s.v_prev[NODE_OUT]);
    }}
}}"
    );
    let tag = format!("pin_linear_{sub_path:?}_{backward_euler}_{force_be}");
    support::compile_and_run(&code, &main, &tag).parse_samples()
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
    }
}
