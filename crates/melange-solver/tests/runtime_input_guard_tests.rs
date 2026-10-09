//! A non-finite runtime input never reaches the solver and is always counted
//! in `diag_runtime_nan_count`.
//!
//! - A `.runtime V` field (public, written by the plugin between calls) reads
//!   as 0 for that host sample. Before the guard, one non-finite control
//!   voltage made every later sample exhaust the Newton and sub-step budgets
//!   and reset on NaN: measured 100× slower per sample on DK and 17,000× on
//!   nodal, with no recovery until the field was finite again.
//! - A `.inject` value reads as 0 for that sample.
//! - A ranged setter (`set_pot_*`, `set_runtime_R_*`, a `.runtime` scalar)
//!   clamps ±inf to its range end, as its doc says; NaN leaves the value
//!   unchanged. Any other setter (noise gains, temperature, sample rate)
//!   leaves the value unchanged.
//!
//! The witnesses hold, on DK, nodal Schur and nodal full-LU, at 1× and 2× and
//! on a backward-Euler build: writing +inf, NaN and −inf for 1000 host
//! samples each (a) counts exactly 1000 per window, (b) never triggers a NaN
//! reset, (c) keeps the output finite, (d) costs no more per sample than the
//! finite baseline, and (e) renders bit-identically to the same windows with
//! the field held at 0. The sanitised copy is written once per host sample,
//! so the count does not scale with the oversampling factor.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, NoiseMode};

/// A diode clipper with a `.runtime V` control offset into the clip node.
const WITNESS: &str = "Runtime V guard witness
R1 in a 2.2k
C1 a b 47n
R2 b 0 100k
D1 b c DX
D2 c b DX
R3 b c 10k
Vctl ctl 0 DC 0
R4 ctl b 47k
Ctone c 0 10n
Rload c out 1k
Cout out 0 1n
.model DX D(IS=2.52n N=1.752)
.runtime Vctl as ctl_voltage
.end
";

/// A pot, a `.runtime R` and resistors for thermal noise: every setter class.
const CONTROLS: &str = "Setter guard witness
R1 in a 2.2k
C1 a b 47n
R2 b 0 100k
D1 b c DX
D2 c b DX
Rdrive b c 10k
.pot Rdrive 1k 100k
Rtone c out 4.7k
.runtime Rtone 1k 47k as tone
Cout out 0 10n
Rload out 0 100k
.model DX D(IS=2.52n N=1.752)
.end
";

/// A behavioral source scaled by a `.runtime` scalar (nodal only).
const SCALAR: &str = "Scalar setter guard witness
Va a 0 DC 1
.runtime amp 0 2 as amp
B1 out 0 I={ amp * V(a) }
Rout out 0 1
Cout out 0 1u
Rin in 0 1meg
";

/// A `.inject` deck: the injection is a plugin-written value too.
const INJECT: &str = "Inject guard witness
R1 in a 2.2k
D1 a 0 DX
D2 0 a DX
Rload a out 1k
Cout out 0 1n
.inject out aux R=10k
.model DX D(IS=2.52n N=1.752)
.end
";

const AUTO: (&str, NodalSubPathOverride) = ("auto", NodalSubPathOverride::Auto);
const DK: (&str, NodalSubPathOverride) = ("dk", NodalSubPathOverride::Auto);
const SCHUR: (&str, NodalSubPathOverride) = ("nodal", NodalSubPathOverride::Schur);
const FULL_LU: (&str, NodalSubPathOverride) = ("nodal", NodalSubPathOverride::FullLu);

fn build(
    spice: &str,
    route: (&str, NodalSubPathOverride),
    oversampling: usize,
    backward_euler: bool,
    noise: NoiseMode,
) -> String {
    let config = melange_solver::codegen::CodegenConfig {
        nodal_sub_path_override: route.1,
        ..support::config_in_out_or_node1(spice, 48000.0)
    };
    support::try_build_shipped_with(spice, &config, route.0, |o| {
        o.oversampling = Some(oversampling);
        o.backward_euler = backward_euler;
        o.noise_mode = noise;
    })
    .unwrap_or_else(|e| panic!("build refused: {e}"))
    .generated
    .code
}

/// Drives two states through the same signal: `a` with the field written
/// +inf, NaN, −inf and then 0 for `n` host samples each, `b` with the field
/// held at 0 throughout. Prints, per window, the counter, the NaN-reset
/// count, whether every output was finite, a hash of every output and node
/// voltage of both states, and the per-sample time of `a`.
fn windows_driver(n: usize) -> String {
    format!(
        "
fn window(st: &mut CircuitState, k0: usize, n: usize) -> (u64, bool, f64) {{
    let mut h: u64 = 0xcbf29ce484222325;
    let mut finite = true;
    let t0 = std::time::Instant::now();
    for k in k0..k0 + n {{
        let x = 0.3 * (2.0 * std::f64::consts::PI * 997.0 * k as f64 / 48000.0).sin();
        let o = process_sample(x, st);
        for v in o.iter().chain(st.v_prev.iter()) {{
            if !v.is_finite() {{ finite = false; }}
            h = (h ^ v.to_bits()).wrapping_mul(0x100000001b3);
        }}
    }}
    (h, finite, t0.elapsed().as_secs_f64() / n as f64)
}}
fn main() {{
    let n = {n}usize;
    let mut a = CircuitState::default();
    let mut b = CircuitState::default();
    a.set_sample_rate(48000.0);
    b.set_sample_rate(48000.0);
    // Baseline: both finite, timed three times on `a` for a stable floor.
    let (ha, _, ta) = window(&mut a, 0, n);
    let (hb, _, _) = window(&mut b, 0, n);
    println!(\"baseline hash_equal={{}} runtime_nan={{}}\", ha == hb, a.diag_runtime_nan_count);
    let values = [f64::INFINITY, f64::NAN, f64::NEG_INFINITY, 0.0];
    let names = [\"inf\", \"nan\", \"neginf\", \"finite\"];
    let mut k0 = n;
    for (v, name) in values.iter().zip(names.iter()) {{
        a.ctl_voltage = *v;
        let (ha, fa, tw) = window(&mut a, k0, n);
        let (hb, _, _) = window(&mut b, k0, n);
        println!(\"{{name}} runtime_nan={{}} nan_reset={{}} unsolved={{}} finite={{fa}} hash_equal={{}} field_bits={{:016x}} time_ratio={{:.3}}\",
            a.diag_runtime_nan_count, a.diag_nan_reset_count, a.diag_unsolved_sample_count, ha == hb,
            a.ctl_voltage.to_bits(), tw / ta);
        k0 += n;
    }}
}}
"
    )
}

/// Parses `key=value` tokens of one output line.
fn kv(line: &str, key: &str) -> String {
    line.split_whitespace()
        .find_map(|t| t.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("no `{key}` in `{line}`"))
        .to_string()
}

fn assert_guarded(code: &str, tag: &str, n: usize) {
    // Structure: every stamp reads the sanitised copy; the public field is read
    // exactly once per host sample, by the host entry.
    assert!(
        !code.contains("+= state.ctl_voltage;"),
        "{tag}: a stamp reads the public field directly"
    );
    assert_eq!(
        code.matches("state.ctl_voltage_sanitized = if state.ctl_voltage.is_finite()")
            .count(),
        1,
        "{tag}: the field must be sanitised in exactly one place"
    );
    let out = support::compile_and_run(code, &windows_driver(n), tag);
    assert!(!out.stdout.is_empty(), "{tag}: no output:\n{}", out.stderr);
    let lines: Vec<&str> = out.stdout.lines().collect();
    assert_eq!(kv(lines[0], "hash_equal"), "true", "{tag}: {}", lines[0]);
    let mut expected = 0u64;
    for (i, name) in ["inf", "nan", "neginf", "finite"].iter().enumerate() {
        let l = lines[i + 1];
        assert!(l.starts_with(name), "{tag}: {l}");
        if *name != "finite" {
            expected += n as u64;
        }
        // (a) counted once per host sample while the field is non-finite.
        assert_eq!(kv(l, "runtime_nan"), expected.to_string(), "{tag}: {l}");
        // (b) nothing non-finite reached the solver.
        assert_eq!(kv(l, "nan_reset"), "0", "{tag}: {l}");
        assert_eq!(kv(l, "unsolved"), "0", "{tag}: {l}");
        // (c) finite output throughout.
        assert_eq!(kv(l, "finite"), "true", "{tag}: {l}");
        // (e) bit-identical to the field held at 0 (the sanitised value).
        assert_eq!(kv(l, "hash_equal"), "true", "{tag}: {l}");
        // The public field is left as the caller wrote it.
        let bits = u64::from_str_radix(&kv(l, "field_bits"), 16).unwrap();
        let written = match *name {
            "inf" => f64::INFINITY,
            "nan" => f64::NAN,
            "neginf" => f64::NEG_INFINITY,
            _ => 0.0,
        };
        if written.is_nan() {
            assert!(f64::from_bits(bits).is_nan(), "{tag}: {l}");
        } else {
            assert_eq!(bits, written.to_bits(), "{tag}: {l}");
        }
        // (d) no slower than the finite baseline (2× allows timer noise; the
        // unguarded build was 100× to 17,000×).
        let ratio: f64 = kv(l, "time_ratio").parse().unwrap();
        assert!(ratio < 2.0, "{tag}: {l}");
    }
}

#[test]
fn a_non_finite_runtime_source_is_guarded_on_dk() {
    assert_guarded(
        &build(WITNESS, DK, 1, false, NoiseMode::Off),
        "rtguard_dk",
        1000,
    );
}

#[test]
fn a_non_finite_runtime_source_is_guarded_on_nodal_schur() {
    assert_guarded(
        &build(WITNESS, SCHUR, 1, false, NoiseMode::Off),
        "rtguard_schur",
        1000,
    );
}

#[test]
fn a_non_finite_runtime_source_is_guarded_on_nodal_full_lu() {
    assert_guarded(
        &build(WITNESS, FULL_LU, 1, false, NoiseMode::Off),
        "rtguard_fulllu",
        1000,
    );
}

#[test]
fn a_non_finite_runtime_source_is_guarded_at_2x_on_dk() {
    assert_guarded(
        &build(WITNESS, DK, 2, false, NoiseMode::Off),
        "rtguard_dk_2x",
        1000,
    );
}

#[test]
fn a_non_finite_runtime_source_is_guarded_at_2x_on_nodal_schur() {
    assert_guarded(
        &build(WITNESS, SCHUR, 2, false, NoiseMode::Off),
        "rtguard_schur_2x",
        1000,
    );
}

#[test]
fn a_non_finite_runtime_source_is_guarded_under_backward_euler() {
    assert_guarded(
        &build(WITNESS, DK, 1, true, NoiseMode::Off),
        "rtguard_dk_be",
        1000,
    );
}

/// The timing claim (d) at a length where the timer is meaningful.
#[test]
fn a_non_finite_runtime_source_costs_nothing_extra() {
    assert_guarded(
        &build(WITNESS, SCHUR, 1, false, NoiseMode::Off),
        "rtguard_timing",
        48000,
    );
}

/// The pot, the runtime resistor, every noise setter, the temperature and the
/// sample rate: ±inf clamps where a range is declared, NaN and any other
/// non-finite argument leaves the value unchanged, and each call counts once.
#[test]
fn setters_clamp_inf_ignore_nan_and_count() {
    let code = build(CONTROLS, DK, 1, false, NoiseMode::Thermal);
    for name in [
        "set_pot_0",
        "set_runtime_R_tone",
        "set_noise_gain",
        "set_thermal_gain",
        "set_temperature_k",
        "set_sample_rate",
    ] {
        assert!(code.contains(&format!("pub fn {name}(")), "{name} missing");
    }
    let main = "
fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    s.set_pot_0(22000.0);
    s.set_runtime_R_tone(10000.0);
    let mut c = s.diag_runtime_nan_count;
    assert_eq!(c, 0);
    s.set_pot_0(f64::INFINITY);
    assert_eq!(s.pot_0_resistance, POT_0_MAX_R); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_pot_0(f64::NEG_INFINITY);
    assert_eq!(s.pot_0_resistance, POT_0_MIN_R); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_pot_0(22000.0);
    s.set_pot_0(f64::NAN);
    assert_eq!(s.pot_0_resistance, 22000.0); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_runtime_R_tone(f64::INFINITY);
    assert_eq!(s.tone(), RUNTIME_R_TONE_MAX); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_runtime_R_tone(f64::NEG_INFINITY);
    assert_eq!(s.tone(), RUNTIME_R_TONE_MIN); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_runtime_R_tone(10000.0);
    s.set_runtime_R_tone(f64::NAN);
    assert_eq!(s.tone(), 10000.0); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_noise_gain(0.5);
    s.set_noise_gain(f64::NAN);
    assert_eq!(s.noise_gain, 0.5); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_noise_gain(f64::INFINITY);
    assert_eq!(s.noise_gain, 0.5); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_thermal_gain(f64::NAN);
    c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_temperature_k(300.0);
    s.set_temperature_k(f64::NAN);
    assert_eq!(s.temperature_k, 300.0); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_temperature_k(-5.0);
    assert_eq!(s.temperature_k, 300.0); assert_eq!(s.diag_runtime_nan_count, c);
    s.set_sample_rate(f64::NAN);
    assert_eq!(s.current_sample_rate, 48000.0); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    s.set_sample_rate(f64::INFINITY);
    assert_eq!(s.current_sample_rate, 48000.0); c += 1; assert_eq!(s.diag_runtime_nan_count, c);
    // A finite render afterwards is unaffected and the count survives it.
    let mut finite = true;
    for k in 0..480usize {
        let o = process_sample(0.2 * (k as f64 * 0.13).sin(), &mut s);
        finite &= o[0].is_finite();
    }
    assert!(finite);
    assert_eq!(s.diag_runtime_nan_count, c);
    s.reset();
    assert_eq!(s.diag_runtime_nan_count, 0);
    println!(\"ok {c}\");
}
";
    let out = support::compile_and_run(&code, main, "rtguard_setters");
    assert!(
        out.stdout.starts_with("ok 12"),
        "{}\n{}",
        out.stdout,
        out.stderr
    );
}

/// A `.runtime` scalar is a ranged setter: ±inf clamps, NaN is ignored.
#[test]
fn a_runtime_scalar_setter_clamps_inf_and_ignores_nan() {
    let code = build(SCALAR, AUTO, 1, false, NoiseMode::Off);
    let main = "
fn main() {
    let mut s = CircuitState::default();
    s.set_runtime_amp(1.5);
    s.set_runtime_amp(f64::INFINITY);
    assert_eq!(s.amp, 2.0);
    s.set_runtime_amp(f64::NEG_INFINITY);
    assert_eq!(s.amp, 0.0);
    s.set_runtime_amp(1.5);
    s.set_runtime_amp(f64::NAN);
    assert_eq!(s.amp, 1.5);
    println!(\"ok {}\", s.diag_runtime_nan_count);
}
";
    let out = support::compile_and_run(&code, main, "rtguard_scalar");
    assert_eq!(out.stdout.trim(), "ok 3", "{}\n{}", out.stdout, out.stderr);
}

/// A non-finite `.inject` value is a plugin-written value: it counts in the
/// runtime counter, not the host-input one, and reads as 0.
#[test]
fn a_non_finite_injection_counts_as_a_runtime_input() {
    let code = build(INJECT, AUTO, 1, false, NoiseMode::Off);
    let main = "
fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let mut finite = true;
    for k in 0..100usize {
        let inj = if k % 2 == 0 { f64::NAN } else { f64::INFINITY };
        let (o, _t) = process_sample(0.1, &[inj; NUM_INJECT_HOST], &[[0.0; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR], &mut s);
        finite &= o[0].is_finite();
    }
    println!(\"{} {} {} {}\", s.diag_runtime_nan_count, s.diag_input_nan_count, s.diag_nan_reset_count, finite);
}
";
    let out = support::compile_and_run(&code, main, "rtguard_inject");
    assert_eq!(
        out.stdout.trim(),
        "100 0 0 true",
        "{}\n{}",
        out.stdout,
        out.stderr
    );
}
