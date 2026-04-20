//! PSD validation for Phase-1 Johnson-Nyquist thermal noise.
//!
//! Load-bearing check is the **kTC theorem**: for an RC lowpass with
//! series R and C-to-ground, the steady-state output variance due to
//! thermal noise on R integrated over [0, ∞) is `kT/C`, independent of
//! R. This is cleaner than a bin-wise FFT PSD comparison — no window
//! bias, no arbitrary-bin tolerance, R-independent (so the test doubles
//! as a sanity check that the stamp scales the right way with R).
//!
//! A single binary runs four scenarios and emits the measured variance
//! for each; the host asserts against physics.

mod support;

use melange_solver::codegen::{CodegenConfig, NoiseMode};

// Boltzmann constant, exact SI 2019 [J/K].
const K_B: f64 = 1.380649e-23;

// Passive RC lowpass. fc = 1/(2π·10k·100n) ≈ 159 Hz, well below
// 48 kHz Nyquist (fs/2 / fc ≈ 302 → bandwidth correction < 1 %).
const RC_SPICE: &str = r#"* RC lowpass — noise PSD validation
R1 in out 10k
C1 out 0 100n
.end
"#;

const CAP_F: f64 = 100e-9;

fn main_template(n_samples: usize, warmup: usize, sample_rate: f64) -> String {
    format!(
        r#"
fn run_case(sr: f64, temp: f64, gain: f64, seed: u64) -> f64 {{
    let mut state = CircuitState::default();
    state.set_sample_rate(sr);
    state.set_seed(seed);
    state.set_noise_enabled(true);
    state.set_temperature_k(temp);
    state.set_noise_gain(gain);
    state.set_thermal_gain(1.0);

    for _ in 0..{warmup} {{
        let _ = process_sample(0.0, &mut state);
    }}

    let mut sum = 0.0_f64;
    let mut sum_sq = 0.0_f64;
    for _ in 0..{n} {{
        let v = process_sample(0.0, &mut state)[0];
        sum += v;
        sum_sq += v * v;
    }}
    let n = {n} as f64;
    let mean = sum / n;
    (sum_sq / n - mean * mean).max(0.0)
}}

fn main() {{
    let sr: f64 = {sr:?};

    let v_baseline  = run_case(sr, 290.0, 1.0, 42);
    let v_temp77    = run_case(sr,  77.0, 1.0, 42);
    let v_gain_half = run_case(sr, 290.0, 0.5, 42);
    let v_repeat    = run_case(sr, 290.0, 1.0, 42);

    println!("VAR:baseline={{:.15e}}", v_baseline);
    println!("VAR:temp77={{:.15e}}", v_temp77);
    println!("VAR:gain_half={{:.15e}}", v_gain_half);
    println!("VAR:repeat={{:.15e}}", v_repeat);
}}
"#,
        sr = sample_rate,
        warmup = warmup,
        n = n_samples
    )
}

fn generate_rc_noise_code(sample_rate: f64, seed: u64) -> String {
    let config = CodegenConfig {
        circuit_name: "rc_noise_psd".to_string(),
        sample_rate,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: seed,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(RC_SPICE, &config);
    code
}

fn generate_rc_noise_code_nodal(sample_rate: f64, seed: u64) -> String {
    let config = CodegenConfig {
        circuit_name: "rc_noise_psd_nodal".to_string(),
        sample_rate,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: seed,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code_nodal(RC_SPICE, &config);
    code
}

fn parse_var(stdout: &str, key: &str) -> f64 {
    let prefix = format!("VAR:{key}=");
    stdout
        .lines()
        .find_map(|l| l.strip_prefix(&prefix))
        .unwrap_or_else(|| panic!("missing '{key}' in stdout:\n{stdout}"))
        .trim()
        .parse::<f64>()
        .unwrap_or_else(|e| panic!("parse '{key}': {e}"))
}

/// kTC theorem: for a series-R, C-to-ground lowpass, the steady-state
/// output variance from thermal noise on R is `kT/C`, independent of R.
/// We allow a ±15 % window to absorb finite-N sampling error (N=2^17 →
/// variance stddev ≈ 0.4 %) and finite-Nyquist tail truncation (< 1 %).
#[test]
fn thermal_noise_matches_ktc_theorem() {
    let sr = 96_000.0;
    // N=2^17 samples at 96 kHz ≈ 1.37 s of audio. Warmup ≈ 10·τ = 10 ms.
    let code = generate_rc_noise_code(sr, 42);
    let main = main_template(1 << 17, 5_000, sr);
    let out = support::compile_and_run(&code, &main, "noise_psd_ktc");

    let v_baseline = parse_var(&out.stdout, "baseline");
    let v_temp77 = parse_var(&out.stdout, "temp77");
    let v_gain_half = parse_var(&out.stdout, "gain_half");
    let v_repeat = parse_var(&out.stdout, "repeat");

    // --- Load-bearing physics check -----------------------------------
    let expected = K_B * 290.0 / CAP_F;
    let ratio = v_baseline / expected;
    assert!(
        (0.85..=1.15).contains(&ratio),
        "kTC theorem violated: measured variance {v_baseline:.3e} V² vs \
         physical kT/C = {expected:.3e} V² (ratio {ratio:.3}). \
         If ratio ≈ 2.0 the noise stamp is using `sqrt(4kT·fs/R)` where \
         strict one-sided-PSD-over-Nyquist math says `sqrt(2kT·fs/R)`."
    );

    // --- Temperature scaling (variance ∝ T) ---------------------------
    // 77 K / 290 K = 0.2655. Same seed → exact fractional relationship
    // would hold in the limit, but finite-N sampling lets it drift.
    let temp_ratio = v_temp77 / v_baseline;
    let expected_temp_ratio = 77.0 / 290.0;
    let temp_err = (temp_ratio / expected_temp_ratio - 1.0).abs();
    assert!(
        temp_err < 0.15,
        "temperature scaling wrong: variance(77K)/variance(290K) = \
         {temp_ratio:.4}, expected {expected_temp_ratio:.4}"
    );

    // --- Gain scaling (variance ∝ gain²) ------------------------------
    // noise_gain=0.5 → variance × 0.25. The RNG streams are the SAME
    // (same seed), so gaussian draws are bit-identical; variance ratio
    // is therefore essentially exact up to sample-by-sample squaring.
    let gain_ratio = v_gain_half / v_baseline;
    let gain_err = (gain_ratio / 0.25 - 1.0).abs();
    assert!(
        gain_err < 0.01,
        "gain scaling wrong: variance(gain=0.5)/variance(gain=1.0) = \
         {gain_ratio:.6}, expected 0.25"
    );

    // --- Determinism: same seed → bit-identical variance --------------
    assert_eq!(
        v_repeat.to_bits(),
        v_baseline.to_bits(),
        "same seed produced different variance: baseline {v_baseline:.15e} \
         vs repeat {v_repeat:.15e}"
    );
}

/// kTC theorem on the **nodal** codegen path (Phase 1.5 Step 1).
///
/// The DK and nodal paths share the same trapezoidal MNA discretization
/// (`(A - A_neg) = 2G` at steady state), so the calibration constant `8`
/// in `sqrt(8·k_B·T·fs/R)` carries over identically. This test forces
/// the linear RC lowpass through `generate_nodal` and asserts the same
/// `k_B·T/C` equilibrium — proves the noise stamp lands in the right
/// place in the nodal RHS construction *and* that no double-counting or
/// missed-stamp regression slipped in.
#[test]
fn thermal_noise_matches_ktc_theorem_nodal() {
    let sr = 96_000.0;
    let code = generate_rc_noise_code_nodal(sr, 42);
    let main = main_template(1 << 17, 5_000, sr);
    let out = support::compile_and_run(&code, &main, "noise_psd_ktc_nodal");

    let v_baseline = parse_var(&out.stdout, "baseline");
    let v_temp77 = parse_var(&out.stdout, "temp77");
    let v_gain_half = parse_var(&out.stdout, "gain_half");
    let v_repeat = parse_var(&out.stdout, "repeat");

    let expected = K_B * 290.0 / CAP_F;
    let ratio = v_baseline / expected;
    assert!(
        (0.85..=1.15).contains(&ratio),
        "kTC theorem violated on NODAL path: measured variance \
         {v_baseline:.3e} V² vs physical kT/C = {expected:.3e} V² \
         (ratio {ratio:.3}). The DK and nodal paths share the trap-MNA \
         relation `(A - A_neg) = 2G`, so the `8·k_B·T·fs/R` calibration \
         must carry over unchanged."
    );

    let temp_ratio = v_temp77 / v_baseline;
    let expected_temp_ratio = 77.0 / 290.0;
    let temp_err = (temp_ratio / expected_temp_ratio - 1.0).abs();
    assert!(
        temp_err < 0.15,
        "nodal temperature scaling wrong: variance(77K)/variance(290K) = \
         {temp_ratio:.4}, expected {expected_temp_ratio:.4}"
    );

    let gain_ratio = v_gain_half / v_baseline;
    let gain_err = (gain_ratio / 0.25 - 1.0).abs();
    assert!(
        gain_err < 0.01,
        "nodal gain scaling wrong: variance(gain=0.5)/variance(gain=1.0) = \
         {gain_ratio:.6}, expected 0.25"
    );

    assert_eq!(
        v_repeat.to_bits(),
        v_baseline.to_bits(),
        "nodal: same seed produced different variance: baseline \
         {v_baseline:.15e} vs repeat {v_repeat:.15e}"
    );
}
