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

fn generate_rc_noise_code_os(sample_rate: f64, seed: u64, os_factor: usize) -> String {
    let config = CodegenConfig {
        circuit_name: "rc_noise_psd_os".to_string(),
        sample_rate,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: seed,
        oversampling_factor: os_factor,
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

/// Oversampling invariance of thermal-noise calibration.
///
/// The kTC theorem output variance `k_B·T/C` is a continuous-time result.
/// It should be independent of internal sample rate — what matters is the
/// end-to-end output noise in the audio band. At 4× oversampling the
/// per-internal-sample variance scales with `fs_internal`, but the
/// decimation filter discards the out-of-band portion, leaving in-band
/// output noise power unchanged. This test pins that claim empirically —
/// if a future change accidentally makes oversampled builds louder or
/// quieter than baseline by more than ~1 dB, it fails here.
///
/// Load-bearing: oomox Gold Press (4× OS, +49 dB closed-loop gain, RIAA in
/// feedback) observed ~+45 dB over expected noise floor and asked whether
/// OS was the culprit. This test answers that question at the calibration
/// level — downstream audibility is a topology/weighting discussion, not a
/// melange calibration bug.
#[test]
fn thermal_noise_ktc_invariant_under_oversampling() {
    let sr_host = 48_000.0;
    let n_samples = 1 << 17;
    let warmup = 5_000;

    let code_1x = generate_rc_noise_code_os(sr_host, 42, 1);
    let code_4x = generate_rc_noise_code_os(sr_host, 42, 4);
    let main = main_template(n_samples, warmup, sr_host);

    let out_1x = support::compile_and_run(&code_1x, &main, "noise_ktc_os1");
    let out_4x = support::compile_and_run(&code_4x, &main, "noise_ktc_os4");

    let v_1x = parse_var(&out_1x.stdout, "baseline");
    let v_4x = parse_var(&out_4x.stdout, "baseline");

    // Both must land in the kTC window at host rate. The 4× case is the
    // one the regression guards — 1× is already covered by
    // `thermal_noise_matches_ktc_theorem` but having it here makes a
    // calibration drift easier to triangulate.
    let expected = K_B * 290.0 / CAP_F;
    let r_1x = v_1x / expected;
    let r_4x = v_4x / expected;
    assert!(
        (0.85..=1.15).contains(&r_1x),
        "1× OS kTC ratio out of window: {r_1x:.3} (variance {v_1x:.3e} V² vs kT/C = {expected:.3e})"
    );
    assert!(
        (0.85..=1.15).contains(&r_4x),
        "4× OS kTC ratio out of window: {r_4x:.3} (variance {v_4x:.3e} V² vs kT/C = {expected:.3e}).\n\
         This means melange's OS path changes the absolute in-band noise\n\
         level relative to the unoversampled baseline — either the\n\
         per-internal-sample variance isn't scaling linearly with\n\
         fs_internal, or the decimation filter isn't unity-passband for\n\
         the injected noise spectrum. Inspect dk_emitter.rs:3035-3046\n\
         (noise_thermal_scale using fs_internal) and the halfband filter\n\
         coefficients in emit_oversampler."
    );

    // Direct OS-invariance check — the two should agree much more
    // tightly than the ±15% kTC window, because we're not comparing to
    // continuous-time physics, we're comparing two discrete realizations
    // that should match each other in expectation. Allow ±10% for RNG
    // sampling variance at N=2^17.
    let os_ratio = v_4x / v_1x;
    let os_err = (os_ratio - 1.0).abs();
    assert!(
        os_err < 0.10,
        "OS breaks noise invariance: var(4×)/var(1×) = {os_ratio:.4} (expected ≈ 1.0).\n\
         variance 1× = {v_1x:.3e}, variance 4× = {v_4x:.3e}.\n\
         If the ratio is ≈ 4.0 the per-sample variance is using fs_internal\n\
         without decimation compensation; if ≈ 0.25 the decimation filter\n\
         is double-discounting."
    );
}

/// Phase 1.5 Step 2: dynamic-resistor noise tracks `set_pot_N`.
///
/// An RC lowpass with `.pot R1` — the kTC theorem still gives output
/// variance `k_B·T/C` independent of `R`, so *variance* stays flat across
/// pot positions (the bandwidth shrinks as R grows, offsetting the per-
/// sample amplitude bump). The load-bearing signal here is that
/// **output changes at all** between two pot settings — if the
/// coefficient weren't refreshed by the setter, the same Gaussian stream
/// would produce a byte-identical output sequence at every R, and the
/// ratio of the first-sample magnitudes would be exactly 1.0.
///
/// We also assert that the variance ratio lands inside the kTC tolerance
/// window: integrating `4·k_B·T·R / (1 + (2πfRC)²)` from 0 to fs/2 is
/// R-independent in the infinite-bandwidth limit, so a wide R span
/// should NOT produce a ~10× variance shift. This catches two classes of
/// bug: coefficient stuck at old R (ratio ≈ R_new / R_old) and double
/// application (ratio ≈ (R_new / R_old)²).
#[test]
fn dynamic_pot_noise_tracks_set_pot() {
    const RC_SPICE_POT: &str = r#"* RC lowpass with .pot R1 — Step 2 dynamic-R noise
R1 in out 10k
C1 out 0 100n
.pot R1 1k 100k
.end
"#;
    let sr = 96_000.0;
    let config = CodegenConfig {
        circuit_name: "rc_pot_noise_psd".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(RC_SPICE_POT, &config);

    let main = format!(
        r#"
fn run_at_pot(pot_r: f64) -> (f64, f64) {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr}_f64);
    state.set_pot_0(pot_r);
    state.set_seed(42);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    for _ in 0..5_000 {{ let _ = process_sample(0.0, &mut state); }}
    let mut sum = 0.0_f64;
    let mut sum_sq = 0.0_f64;
    let mut first = 0.0_f64;
    let n = 1usize << 17;
    for i in 0..n {{
        let v = process_sample(0.0, &mut state)[0];
        if i == 0 {{ first = v; }}
        sum += v;
        sum_sq += v * v;
    }}
    let mean = sum / n as f64;
    let var = (sum_sq / n as f64 - mean * mean).max(0.0);
    (var, first)
}}

fn main() {{
    let (var_1k,  first_1k ) = run_at_pot(1_000.0);
    let (var_100k, first_100k) = run_at_pot(100_000.0);
    println!("VAR:r_1k={{:.15e}}", var_1k);
    println!("VAR:r_100k={{:.15e}}", var_100k);
    println!("VAR:first_1k={{:.15e}}", first_1k);
    println!("VAR:first_100k={{:.15e}}", first_100k);
}}
"#,
        sr = sr
    );
    let out = support::compile_and_run(&code, &main, "noise_pot_dynamic");

    let var_1k = parse_var(&out.stdout, "r_1k");
    let var_100k = parse_var(&out.stdout, "r_100k");
    let first_1k = parse_var(&out.stdout, "first_1k");
    let first_100k = parse_var(&out.stdout, "first_100k");

    let expected = K_B * 290.0 / CAP_F;
    for (var, label) in [(var_1k, "r=1k"), (var_100k, "r=100k")] {
        let ratio = var / expected;
        assert!(
            (0.70..=1.30).contains(&ratio),
            "dynamic-pot kTC violated at {label}: variance {var:.3e} V² \
             vs physical kT/C = {expected:.3e} V² (ratio {ratio:.3}). The \
             wider window vs the static-R test absorbs bandwidth effects \
             at the R=100k corner frequency."
        );
    }

    // Load-bearing: the two pot positions MUST produce different output.
    // Same seed → same Gaussian stream → the first-sample divergence is
    // driven entirely by the coefficient swap. If the setter didn't
    // refresh, first_1k would equal first_100k bit-for-bit.
    assert_ne!(
        first_1k.to_bits(),
        first_100k.to_bits(),
        "set_pot_0 did not change the first-sample noise output — \
         coefficient refresh likely not wired (first={first_1k:.6e})"
    );
}

/// Phase 2 Step 4: shot noise actually stamps into the RHS.
///
/// A clean quantitative test of shot-noise variance is hard on a voltage-
/// output circuit because the output impedance `Z_out ≈ r_d = Vt/I`
/// scales as `1/I`, so output voltage variance scales like
/// `(shot_current_variance) × Z_out² ∝ I × 1/I² = 1/I` — high bias is
/// *quieter* at the output, masking the per-source `∝ I` scaling with
/// a bias-dependent gain.
///
/// Instead this test makes three load-bearing observations:
///
/// 1. With `thermal_gain = 0` (thermal muted) and `shot_gain = 1`, the
///    forward-biased diode produces non-zero output variance. If the
///    shot stamp weren't wired, the output would be a deterministic
///    trajectory of the DC OP and variance would be < f64::EPSILON.
///
/// 2. With `shot_gain = 0` (shot muted) and `thermal_gain = 1`, variance
///    is still non-zero (thermal from R_drive). This shows the two
///    contributions are independently gated.
///
/// 3. Same seed + same DC bias → bit-identical output sequence. Proves
///    the shot RNG is deterministic under `set_seed`.
///
/// 4. Two different DC biases with `shot_gain=1, thermal_gain=0` produce
///    different first-sample outputs (same seed → same Gaussian stream;
///    the divergence comes from the `sqrt(|I_prev|)` weighting). This is
///    the Step 2 first-sample divergence pattern applied to shot — if
///    the stamp weren't reading `i_nl_prev`, the two biases would give
///    bit-identical first samples.
#[test]
fn shot_noise_is_audibly_wired_for_diode() {
    const DIODE_BIAS_SPICE: &str = r#"* Diode driven by input voltage
R_drive in a 10k
D1 a 0 D1N4148
.model D1N4148 D(IS=1e-15)
.end
"#;
    let sr = 96_000.0;
    let config = CodegenConfig {
        circuit_name: "shot_diode".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![1], // "a" = anode node (1-indexed MNA = 2?  No, solver-config uses 0-indexed)
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Shot,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(DIODE_BIAS_SPICE, &config);

    let main = format!(
        r#"
fn run_variance(thermal: f64, shot: f64, dc: f64, seed: u64) -> (f64, f64) {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr}_f64);
    state.set_seed(seed);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    state.set_thermal_gain(thermal);
    state.set_shot_gain(shot);
    state.set_noise_gain(1.0);

    for _ in 0..5_000 {{ let _ = process_sample(dc, &mut state); }}

    let n = 1usize << 16;
    let mut sum = 0.0_f64;
    let mut sum_sq = 0.0_f64;
    let mut first = 0.0_f64;
    for i in 0..n {{
        let v = process_sample(dc, &mut state)[0];
        if i == 0 {{ first = v; }}
        sum += v;
        sum_sq += v * v;
    }}
    let mean = sum / n as f64;
    let var = (sum_sq / n as f64 - mean * mean).max(0.0);
    (var, first)
}}

fn main() {{
    // Shot-only at low and high bias
    let (var_shot_lo, first_shot_lo) = run_variance(0.0, 1.0, 1.0, 42);
    let (var_shot_hi, first_shot_hi) = run_variance(0.0, 1.0, 3.0, 42);
    // Thermal-only at same bias
    let (var_thermal_only, _) = run_variance(1.0, 0.0, 1.0, 42);
    // Determinism check
    let (var_shot_lo_repeat, _) = run_variance(0.0, 1.0, 1.0, 42);
    // Both muted
    let (var_silent, _) = run_variance(0.0, 0.0, 1.0, 42);

    println!("VAR:shot_lo={{:.15e}}",   var_shot_lo);
    println!("VAR:shot_hi={{:.15e}}",   var_shot_hi);
    println!("VAR:thermal={{:.15e}}",   var_thermal_only);
    println!("VAR:repeat={{:.15e}}",    var_shot_lo_repeat);
    println!("VAR:silent={{:.15e}}",    var_silent);
    println!("VAR:first_lo={{:.15e}}",  first_shot_lo);
    println!("VAR:first_hi={{:.15e}}",  first_shot_hi);
}}
"#,
        sr = sr
    );
    let out = support::compile_and_run(&code, &main, "noise_shot_diode");

    let var_shot_lo = parse_var(&out.stdout, "shot_lo");
    let var_shot_hi = parse_var(&out.stdout, "shot_hi");
    let var_thermal = parse_var(&out.stdout, "thermal");
    let var_repeat = parse_var(&out.stdout, "repeat");
    let var_silent = parse_var(&out.stdout, "silent");
    let first_lo = parse_var(&out.stdout, "first_lo");
    let first_hi = parse_var(&out.stdout, "first_hi");

    // (0) Both-muted must be essentially zero — DC input gives steady state
    //     output = DC level; variance around it ≈ 0.
    assert!(
        var_silent < 1e-20,
        "both-muted variance should be near-zero (pure DC response), got {var_silent:.3e}"
    );

    // (1) Shot-only at either bias must produce non-zero variance.
    assert!(
        var_shot_lo > 1e-18,
        "shot-only at low bias should stamp non-zero variance, got {var_shot_lo:.3e}"
    );
    assert!(
        var_shot_hi > 1e-18,
        "shot-only at high bias should stamp non-zero variance, got {var_shot_hi:.3e}"
    );

    // (2) Thermal-only must ALSO produce non-zero variance — independent
    //     gating. Catches shot_gain/thermal_gain accidental cross-wiring.
    assert!(
        var_thermal > 1e-18,
        "thermal-only variance should be non-zero, got {var_thermal:.3e}"
    );

    // (3) Determinism: same seed → same variance.
    assert_eq!(
        var_shot_lo.to_bits(),
        var_repeat.to_bits(),
        "seed determinism violated: shot_lo={var_shot_lo:.15e} repeat={var_repeat:.15e}"
    );

    // (4) Different bias → different first-sample output. Load-bearing:
    //     proves the shot stamp reads state.i_nl_prev (otherwise first
    //     samples would be bit-identical since the RNG stream is the same).
    assert_ne!(
        first_lo.to_bits(),
        first_hi.to_bits(),
        "shot first-sample output did not change between bias levels \
         (lo={first_lo:.6e} hi={first_hi:.6e}) — stamp may not be reading \
         state.i_nl_prev"
    );
}

/// Phase 1.5 Step 2 v2: `.switch` R-component noise tracks `set_switch_N`.
///
/// Analogue of `dynamic_pot_noise_tracks_set_pot` for discrete-position
/// switch R. Two observations:
///
/// - kTC variance stays inside the wide tolerance window at BOTH switch
///   positions — the per-sample coefficient is refreshed from the new R.
/// - Same seed → different first-sample output between positions 0 and 2.
///   Load-bearing: if `set_switch_N` didn't refresh
///   `state.noise_thermal_sqrt_inv_r`, the RNG stream would produce a
///   byte-identical first sample at both positions.
#[test]
fn dynamic_switch_r_noise_tracks_set_switch() {
    const RC_SPICE_SWITCH: &str = r#"* RC lowpass with .switch R1 — Step 2 v2 switch-R noise
R1 in out 10k
C1 out 0 100n
.switch R1 1k 10k 100k "Tone"
.end
"#;
    let sr = 96_000.0;
    let config = CodegenConfig {
        circuit_name: "rc_switch_noise_psd".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(RC_SPICE_SWITCH, &config);

    let main = format!(
        r#"
fn run_at_position(pos: usize) -> (f64, f64) {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr}_f64);
    state.set_switch_0(pos);
    state.set_seed(42);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    for _ in 0..5_000 {{ let _ = process_sample(0.0, &mut state); }}
    let mut sum = 0.0_f64;
    let mut sum_sq = 0.0_f64;
    let mut first = 0.0_f64;
    let n = 1usize << 16;
    for i in 0..n {{
        let v = process_sample(0.0, &mut state)[0];
        if i == 0 {{ first = v; }}
        sum += v;
        sum_sq += v * v;
    }}
    let mean = sum / n as f64;
    let var = (sum_sq / n as f64 - mean * mean).max(0.0);
    (var, first)
}}

fn main() {{
    let (var_pos0, first_pos0) = run_at_position(0);
    let (var_pos2, first_pos2) = run_at_position(2);
    println!("VAR:pos0={{:.15e}}", var_pos0);
    println!("VAR:pos2={{:.15e}}", var_pos2);
    println!("VAR:first_pos0={{:.15e}}", first_pos0);
    println!("VAR:first_pos2={{:.15e}}", first_pos2);
}}
"#,
        sr = sr
    );
    let out = support::compile_and_run(&code, &main, "noise_switch_dynamic");

    let var_pos0 = parse_var(&out.stdout, "pos0");
    let var_pos2 = parse_var(&out.stdout, "pos2");
    let first_pos0 = parse_var(&out.stdout, "first_pos0");
    let first_pos2 = parse_var(&out.stdout, "first_pos2");

    let expected = K_B * 290.0 / CAP_F;
    for (var, label) in [(var_pos0, "pos=0 (1k)"), (var_pos2, "pos=2 (100k)")] {
        let ratio = var / expected;
        assert!(
            (0.70..=1.30).contains(&ratio),
            "switch-R kTC violated at {label}: variance {var:.3e} V² vs \
             kT/C {expected:.3e} V² (ratio {ratio:.3})"
        );
    }

    // Load-bearing: different R → different first-sample output at same
    // seed. Proves `set_switch_0(2)` refreshed the coefficient.
    assert_ne!(
        first_pos0.to_bits(),
        first_pos2.to_bits(),
        "set_switch_0 did not change the first-sample noise output — \
         coefficient refresh likely not wired (first={first_pos0:.6e})"
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

/// Phase 3 Step 5: the Kellett 7-pole pink filter (in isolation) has the
/// expected -3 dB/oct spectral signature.
///
/// Spectral slope is a property of the **filter**, not the noise-injection
/// site. When flicker is injected through a nonlinear device, the circuit's
/// small-signal impedance reshapes the output and the 1-pole-LP ratio test
/// no longer cleanly separates pink from white (see
/// `flicker_noise_is_audibly_wired` — it asserts wiring, amplitude, and
/// gating, but not slope). This test reproduces the exact Rust filter
/// emitted by `build_noise_emission` and feeds it unit-variance white,
/// then uses the same LP-ratio signature:
///
///   RMS_lp / RMS_raw ≈ √0.01 = 0.10  for white   (LP fraction ≈ 0.01)
///   RMS_lp / RMS_raw ≈ √0.50 = 0.71  for pink    (LP fraction ≈ 0.50)
///
/// Because the filter code emitted into the generated solver is literally
/// this function, any regression that breaks slope (wrong coefficient,
/// dropped state element, etc.) will fail this test.
#[test]
fn kellett_pink_filter_has_pink_slope() {
    // Paul Kellett 7-pole pink — EXACTLY what `build_noise_emission`
    // emits into the generated file. Mirrors the `top.push_str` block in
    // `dk_emitter.rs` verbatim; keep these two copies in sync.
    fn kellett_pink(white: f64, state: &mut [f64; 7]) -> f64 {
        state[0] = 0.99886 * state[0] + white * 0.0555179;
        state[1] = 0.99332 * state[1] + white * 0.0750759;
        state[2] = 0.96900 * state[2] + white * 0.1538520;
        state[3] = 0.86650 * state[3] + white * 0.3104856;
        state[4] = 0.55000 * state[4] + white * 0.5329522;
        state[5] = -0.7616 * state[5] - white * 0.0168980;
        let pink = state[0]
            + state[1]
            + state[2]
            + state[3]
            + state[4]
            + state[5]
            + state[6]
            + white * 0.5362;
        state[6] = white * 0.115926;
        pink * 0.11
    }

    // Trivial Gaussian — not the xoshiro path we ship (the emitted
    // generator produces the same statistics by construction), but it's
    // enough to feed the filter unit-variance white for a slope check.
    let mut lcg_state: u64 = 0xDEADBEEFCAFEBABE;
    let mut cached: Option<f64> = None;
    let mut gauss = || -> f64 {
        if let Some(z) = cached.take() {
            return z;
        }
        loop {
            lcg_state = lcg_state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let u = ((lcg_state >> 11) as f64) * (1.0 / (1u64 << 53) as f64) * 2.0 - 1.0;
            lcg_state = lcg_state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let v = ((lcg_state >> 11) as f64) * (1.0 / (1u64 << 53) as f64) * 2.0 - 1.0;
            let r = u * u + v * v;
            if r > 0.0 && r < 1.0 {
                let factor = (-2.0 * r.ln() / r).sqrt();
                cached = Some(v * factor);
                return u * factor;
            }
        }
    };

    let fs = 96_000.0_f64;
    let fc = 480.0_f64; // fs / 200
    let alpha = 1.0 - (-2.0 * std::f64::consts::PI * fc / fs).exp();

    let mut filter_state = [0.0_f64; 7];
    // Warm the Kellett filter and the LP.
    let mut lp = 0.0_f64;
    for _ in 0..5_000 {
        let _ = kellett_pink(gauss(), &mut filter_state);
    }
    for _ in 0..500 {
        let p = kellett_pink(gauss(), &mut filter_state);
        lp += alpha * (p - lp);
    }

    // Accumulate variance over 2^17 samples.
    let n: usize = 1 << 17;
    let (mut sr, mut sr2, mut sl, mut sl2) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for _ in 0..n {
        let p = kellett_pink(gauss(), &mut filter_state);
        lp += alpha * (p - lp);
        sr += p;
        sr2 += p * p;
        sl += lp;
        sl2 += lp * lp;
    }
    let nf = n as f64;
    let var_raw = (sr2 / nf - (sr / nf).powi(2)).max(0.0);
    let var_lp = (sl2 / nf - (sl / nf).powi(2)).max(0.0);
    let ratio = var_lp / var_raw;

    // Pink target ≈ 0.50 (derived from ∫ 1/f · |H|² over [f_low, fs/2]).
    // Tolerance [0.30, 0.75] separates pink from white (0.01) and red
    // (~0.95) while absorbing finite-N variance and Kellett's ±0.5 dB
    // slope jitter at band edges.
    assert!(
        (0.30..=0.75).contains(&ratio),
        "Kellett filter slope wrong: var(LP 480Hz)/var(raw) = {ratio:.4}, \
         expected ~0.50 for 1/f output. White gives ~0.01, red ~0.95 — \
         an out-of-range ratio means a coefficient drift or missing state \
         element in `build_noise_emission`'s kellett_pink emission."
    );
}

/// Phase 3 Step 5: flicker (1/f) noise is audibly wired end-to-end.
///
/// Quantitative spectral-slope validation lives in
/// `kellett_pink_filter_has_pink_slope`: spectral shape is a property of
/// the filter, independent of where it's injected in the circuit, so
/// that test runs in pure Rust and proves the emitted filter math is
/// pink.
///
/// This test asserts the in-circuit integration plumbing:
///
/// 1. With `thermal_gain=0`, `shot_gain=0`, flicker produces nonzero
///    variance — proves the stamp is wired end-to-end through the
///    device-current lookup, Kellett filter, and RHS injection.
/// 2. `set_flicker_gain(0.0)` → variance drops to near-DC — proves the
///    runtime gate.
/// 3. Different DC biases → different first-sample outputs — proves the
///    `|I_prev|^(AF/2)` scaling reads `state.i_nl_prev`.
/// 4. Same seed → bit-identical variance.
#[test]
fn flicker_noise_is_audibly_wired() {
    const DIODE_KF_SPICE: &str = r#"* Diode with KF for flicker
R_drive in a 10k
D1 a 0 D1N4148
.model D1N4148 D(IS=1e-15 KF=1e-14 AF=1.0)
.end
"#;
    let sr = 96_000.0;
    let config = CodegenConfig {
        circuit_name: "flicker_diode".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Full,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(DIODE_KF_SPICE, &config);

    // One-pole LPF coefficient for f_c = fs/200 = 480 Hz at 96 kHz:
    //   α = 1 - exp(-2π·f_c/fs) ≈ 0.0308
    // We bake it into the main binary so no extra deps.
    let main = format!(
        r#"
fn run(thermal: f64, shot: f64, flicker: f64, dc: f64, seed: u64) -> (f64, f64) {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr}_f64);
    state.set_seed(seed);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    state.set_thermal_gain(thermal);
    state.set_shot_gain(shot);
    state.set_flicker_gain(flicker);
    state.set_noise_gain(1.0);

    for _ in 0..10_000 {{ let _ = process_sample(dc, &mut state); }}

    let mut sum_raw = 0.0_f64;
    let mut sum_raw_sq = 0.0_f64;
    let mut first = 0.0_f64;
    let n = 1usize << 17;
    for i in 0..n {{
        let v = process_sample(dc, &mut state)[0];
        if i == 0 {{ first = v; }}
        sum_raw += v;
        sum_raw_sq += v * v;
    }}
    let nf = n as f64;
    let mean = sum_raw / nf;
    let var_raw = (sum_raw_sq / nf - mean * mean).max(0.0);
    (var_raw, first)
}}

fn main() {{
    let (var_flick, first_flick_lo) = run(0.0, 0.0, 1.0, 1.0, 42);
    let (_, first_flick_hi)         = run(0.0, 0.0, 1.0, 3.0, 42);
    let (var_gate_off, _)           = run(0.0, 0.0, 0.0, 1.0, 42);
    let (var_repeat, _)             = run(0.0, 0.0, 1.0, 1.0, 42);

    println!("VAR:flick_raw={{:.15e}}",  var_flick);
    println!("VAR:gate_off={{:.15e}}",   var_gate_off);
    println!("VAR:repeat={{:.15e}}",     var_repeat);
    println!("VAR:first_lo={{:.15e}}",   first_flick_lo);
    println!("VAR:first_hi={{:.15e}}",   first_flick_hi);
}}
"#,
        sr = sr
    );
    let out = support::compile_and_run(&code, &main, "noise_flicker_wired");

    let var_flick = parse_var(&out.stdout, "flick_raw");
    let var_gate_off = parse_var(&out.stdout, "gate_off");
    let var_repeat = parse_var(&out.stdout, "repeat");
    let first_lo = parse_var(&out.stdout, "first_lo");
    let first_hi = parse_var(&out.stdout, "first_hi");

    // (1) Flicker on → nonzero variance.
    assert!(
        var_flick > 1e-24,
        "flicker-only variance should be non-zero, got {var_flick:.3e}"
    );

    // (2) flicker_gain=0 → variance collapses toward noise-floor.
    assert!(
        var_gate_off < var_flick * 1e-3,
        "set_flicker_gain(0.0) did not gate flicker: var_gate_off={var_gate_off:.3e}, \
         var_flick={var_flick:.3e} (ratio {:.3e})",
        var_gate_off / var_flick
    );

    // (3) Bias-dependent scaling: different |I_prev| → different first sample.
    //     Same seed → same Gaussian stream; divergence comes entirely from
    //     the `|I_prev|^(AF/2)` weighting reading `state.i_nl_prev`.
    assert_ne!(
        first_lo.to_bits(),
        first_hi.to_bits(),
        "flicker first-sample output did not change between bias levels \
         (lo={first_lo:.6e} hi={first_hi:.6e}) — stamp may not be reading \
         state.i_nl_prev"
    );

    // (4) Same seed → bit-identical variance.
    assert_eq!(
        var_repeat.to_bits(),
        var_flick.to_bits(),
        "flicker seed determinism violated: var={var_flick:.15e} repeat={var_repeat:.15e}"
    );
}

/// Regression test: thermal noise on a resistor-only output node must NOT produce
/// a Nyquist oscillation (lag-1 autocorrelation close to -1).
///
/// Before the two-draw fix, nodes without shunt capacitors (i.e., purely
/// resistive nodes in the MNA) had a Nyquist pole in the trapezoidal
/// discretization. White Gaussian noise injection excited this pole, creating
/// a stationary `(+1, -1, +1, -1, ...)` alternating sequence at the output.
/// The resulting variance was ~10,000× the physical kTC prediction and scaled
/// roughly linearly with fs instead of being fs-independent.
///
/// This test uses a minimal forward-biased diode circuit where the output is
/// connected via a series resistor to the diode (no shunt cap at the output
/// node). Before the fix this circuit exhibits:
///   - lag-1 autocorrelation << -0.9 (pure Nyquist oscillation)
///   - output RMS in the mV range (vs expected sub-10 µV)
///
/// After the fix (two-draw stamp: `i_n = w_new + w_prev` with each draw at
/// scale/2) the Nyquist bin receives zero injection energy, the autocorrelation
/// is positive, and the RMS stays in the physically expected range.
#[test]
fn thermal_noise_no_nyquist_artifact_on_resistor_only_output_node() {
    // Diode driven through a series resistor, with an OUTPUT SERIES RESISTOR
    // (R_out) that has no shunt cap. R_out is the node that historically
    // developed the Nyquist artifact.
    //
    // The output is at node "out" (between R_out and load R_load-to-ground).
    // Both R_out and R_load are purely resistive — no capacitor shunts "out"
    // to ground directly, so (A - A_neg)[out][out] = 2G[out][out] ≠ 0 but
    // A_neg[out][out] = -G[out][out] < 0, which creates the Nyquist pole in
    // the single-draw path.
    const DIODE_SERIES_R_SPICE: &str = r#"* Diode + series output resistor — Nyquist artifact regression
R_drive in anode 10k
D1 anode 0 D1N4148
R_out anode out 6.8k
R_load out 0 56k
.model D1N4148 D(IS=1e-15)
.end
"#;
    let sr = 48_000.0_f64;
    let config = CodegenConfig {
        circuit_name: "noise_nyquist_regress".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![2], // "out" node index (0-indexed: in=0, anode=1, out=2)
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(DIODE_SERIES_R_SPICE, &config);

    // Measure lag-1 autocorrelation of the output to detect Nyquist oscillation.
    // Also measure RMS to detect the 1000× amplitude amplification.
    let main = format!(
        r#"
fn main() {{
    let sr = {sr}_f64;
    let n_warmup = 5_000usize;
    let n_samples = 1usize << 16;
    let dc_bias = 1.0_f64; // forward-bias the diode

    let mut state = CircuitState::default();
    state.set_sample_rate(sr);
    state.set_seed(42);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    state.set_noise_gain(1.0);
    state.set_thermal_gain(1.0);

    for _ in 0..n_warmup {{
        let _ = process_sample(dc_bias, &mut state);
    }}

    // Collect samples and compute variance + lag-1 autocorrelation.
    let mut samples = vec![0.0_f64; n_samples];
    for i in 0..n_samples {{
        samples[i] = process_sample(dc_bias, &mut state)[0];
    }}

    let mean = samples.iter().sum::<f64>() / n_samples as f64;
    let var = samples.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n_samples as f64;
    let lag1_num = samples[..n_samples-1].iter()
        .zip(samples[1..].iter())
        .map(|(&a, &b)| (a - mean) * (b - mean))
        .sum::<f64>() / (n_samples - 1) as f64;
    let lag1 = if var > 0.0 {{ lag1_num / var }} else {{ 0.0 }};

    println!("VAR={{:.15e}}", var);
    println!("RMS={{:.15e}}", var.sqrt());
    println!("LAG1={{:.8}}", lag1);
}}
"#,
        sr = sr,
    );

    let out = support::compile_and_run(&code, &main, "noise_nyquist_regress");
    let var: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("VAR=").and_then(|v| v.parse().ok()))
        .expect("VAR not found in stdout");
    let rms: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("RMS=").and_then(|v| v.parse().ok()))
        .expect("RMS not found in stdout");
    let lag1: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("LAG1=").and_then(|v| v.parse().ok()))
        .expect("LAG1 not found in stdout");

    // Load-bearing: lag-1 autocorrelation must NOT be strongly negative.
    // Before the fix: lag1 ≈ -0.9999 (pure Nyquist oscillation).
    // After the fix: lag1 should be > -0.5 for a well-behaved noise process.
    assert!(
        lag1 > -0.5,
        "Nyquist artifact detected on resistor-only output node: \
         lag1={lag1:.4} (expected > -0.5). \
         This means thermal noise is creating a stationary (+/-1) alternation \
         at the output, indicating the two-draw Nyquist anti-alias stamp is \
         not working. RMS={rms:.3e}"
    );

    // Load-bearing: output RMS must be sub-100 µV.
    // Before the fix: RMS ≈ 1-5 mV (10,000× too large due to Nyquist accumulation).
    // After the fix: RMS should be in the single-digit µV range.
    assert!(
        rms < 100e-6,
        "Thermal noise RMS {rms:.3e} V exceeds 100 µV on a resistor-only output \
         node. This likely indicates the Nyquist artifact is active — the two-draw \
         anti-alias stamp is not eliminating the Nyquist bin injection. \
         lag1={lag1:.4}"
    );

    // Sanity: noise is actually nonzero (the stamp is active).
    assert!(
        var > 1e-18,
        "Thermal noise variance {var:.3e} V² is near-zero — noise may not be \
         injected at all. Check that noise_enabled and the stamp are wired correctly."
    );
}

/// Regression: fs-sweep on the same diode + series-R circuit as the
/// `_on_resistor_only_output_node` test. The original bug's smoking gun was
/// output RMS scaling roughly linearly with fs (the Nyquist pole accumulated
/// more energy as Nyquist BW widened). After the two-draw fix, output RMS on
/// a non-band-limited circuit should scale ~ sqrt(fs) (white noise integrated
/// over Nyquist) — and on a band-limited circuit, it should be fs-independent
/// (kTC invariant). For this circuit (no shunt caps anywhere → no
/// band-limiting), sqrt(fs) is the expected scaling. We assert that the ratio
/// from 48k → 192k stays well below the linear-fs (4×) scaling that signals
/// the bug.
#[test]
fn thermal_noise_no_nyquist_artifact_fs_sweep() {
    const SPICE: &str = r#"* Diode + series output resistor — fs-sweep regression
R_drive in anode 10k
D1 anode 0 D1N4148
R_out anode out 6.8k
R_load out 0 56k
.model D1N4148 D(IS=1e-15)
.end
"#;
    fn measure_at(sr: f64) -> (f64, f64) {
        let config = CodegenConfig {
            circuit_name: format!("noise_fs_sweep_{}", sr as u64),
            sample_rate: sr,
            input_node: 0,
            output_nodes: vec![2],
            input_resistance: 1.0,
            dc_block: false,
            noise_mode: NoiseMode::Thermal,
            noise_master_seed: 42,
            ..CodegenConfig::default()
        };
        let (code, _n, _m) = support::generate_circuit_code(SPICE, &config);
        let main = format!(
            r#"
fn main() {{
    let sr = {sr}_f64;
    let mut state = CircuitState::default();
    state.set_sample_rate(sr);
    state.set_seed(42);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    state.set_noise_gain(1.0);
    state.set_thermal_gain(1.0);
    for _ in 0..5_000 {{ let _ = process_sample(1.0, &mut state); }}
    let n = 1usize << 16;
    let mut sum = 0.0f64;
    let mut sum_sq = 0.0f64;
    let mut prev = 0.0f64;
    let mut sign_flips = 0usize;
    for i in 0..n {{
        let v = process_sample(1.0, &mut state)[0];
        sum += v;
        sum_sq += v * v;
        if i > 0 && (v - prev).signum() != 0.0 {{
            // Compare to running mean baseline rather than zero — signal could
            // bias around DC. We just want a lag-1 anti-correlation proxy.
        }}
        if i > 0 {{
            let dv = v - prev;
            if dv.abs() > 1e-12 && (prev - sum / (i as f64 + 1.0)) * (v - sum / (i as f64 + 1.0)) < 0.0 {{
                sign_flips += 1;
            }}
        }}
        prev = v;
    }}
    let mean = sum / n as f64;
    let var = (sum_sq / n as f64 - mean * mean).max(0.0);
    let flip_frac = sign_flips as f64 / (n - 1) as f64;
    println!("RMS={{:.15e}}", var.sqrt());
    println!("FLIP={{:.6}}", flip_frac);
}}
"#,
            sr = sr,
        );
        let out = support::compile_and_run(&code, &main, &format!("noise_fs_sweep_{}", sr as u64));
        let rms: f64 = out
            .stdout
            .lines()
            .find_map(|l| l.strip_prefix("RMS=").and_then(|v| v.parse().ok()))
            .expect("RMS not found");
        let flip: f64 = out
            .stdout
            .lines()
            .find_map(|l| l.strip_prefix("FLIP=").and_then(|v| v.parse().ok()))
            .expect("FLIP not found");
        (rms, flip)
    }

    let (rms_48k, flip_48k) = measure_at(48_000.0);
    let (rms_192k, flip_192k) = measure_at(192_000.0);

    // Expected: sqrt(192/48) = 2.0 ratio for sqrt(fs) scaling. Bug behavior
    // was linear-fs: ratio ≈ 4.0+. We allow a generous upper bound of 2.8
    // (well below 4.0) and a lower bound of 1.4 (well above 1.0 — would
    // indicate the noise is over-attenuated, also a bug).
    let ratio = rms_192k / rms_48k;
    assert!(
        (1.4..=2.8).contains(&ratio),
        "fs-scaling ratio (192k/48k) = {ratio:.3} out of [1.4, 2.8]. \
         Expected ~2.0 (sqrt(fs) scaling for a non-band-limited circuit). \
         Ratio ≥ 4.0 indicates the Nyquist accumulation bug. \
         Ratio < 1.4 indicates over-attenuation. \
         RMS(48k)={rms_48k:.3e}, RMS(192k)={rms_192k:.3e}"
    );

    // Sign-flip fraction (~1.0 = pure alternation = Nyquist artifact).
    // After the fix, flips should be near 0.5 (white noise has uncorrelated
    // sign changes around the DC baseline).
    assert!(
        flip_48k < 0.7 && flip_192k < 0.7,
        "Sign-flip fraction high — Nyquist alternation may still be active. \
         flip(48k)={flip_48k:.3}, flip(192k)={flip_192k:.3}"
    );
}

/// Regression: purely passive resistor divider with no devices and no caps.
/// This is the simplest topology where every node is resistor-only — the
/// strongest sanity check for the Nyquist anti-alias stamp. The kTC theorem
/// test (which uses an RC) cannot exercise this case because it has a cap
/// at the output. Without the two-draw fix, every node would Nyquist-oscillate.
#[test]
fn thermal_noise_no_nyquist_artifact_passive_divider() {
    const SPICE: &str = r#"* Pure passive resistor divider — Nyquist regression
R1 in n1 1k
R2 n1 0 4k7
R_out n1 out 2k2
R_load out 0 10k
.end
"#;
    let sr = 48_000.0_f64;
    let config = CodegenConfig {
        circuit_name: "noise_nyquist_passive".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![2], // out
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(SPICE, &config);
    let main = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr}_f64);
    state.set_seed(42);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    state.set_noise_gain(1.0);
    state.set_thermal_gain(1.0);
    for _ in 0..2_000 {{ let _ = process_sample(0.0, &mut state); }}
    let n = 1usize << 16;
    let mut samples = vec![0.0f64; n];
    for i in 0..n {{ samples[i] = process_sample(0.0, &mut state)[0]; }}
    let mean = samples.iter().sum::<f64>() / n as f64;
    let var = samples.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n as f64;
    let lag1_num = samples[..n-1].iter().zip(samples[1..].iter())
        .map(|(&a, &b)| (a - mean) * (b - mean)).sum::<f64>() / (n - 1) as f64;
    let lag1 = if var > 0.0 {{ lag1_num / var }} else {{ 0.0 }};
    println!("VAR={{:.15e}}", var);
    println!("RMS={{:.15e}}", var.sqrt());
    println!("LAG1={{:.8}}", lag1);
}}
"#,
        sr = sr,
    );
    let out = support::compile_and_run(&code, &main, "noise_nyquist_passive");
    let var: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("VAR=").and_then(|v| v.parse().ok()))
        .expect("VAR not found");
    let rms: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("RMS=").and_then(|v| v.parse().ok()))
        .expect("RMS not found");
    let lag1: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("LAG1=").and_then(|v| v.parse().ok()))
        .expect("LAG1 not found");

    // The four resistors are 1k+4k7 (input divider) and 2k2+10k (output
    // divider). Thermal noise on R_load (10k) at 48 kHz Nyquist gives
    // PSD ≈ 8·k_B·T/10k ≈ 3.3e-21 A²/Hz, integrated over 24 kHz with the
    // cos²-shaped envelope. The output transimpedance from R_load's
    // current source is bounded by R_load itself (parallel with R_out
    // back through R1+R2 to ground), so output RMS is on the order of
    // a few microvolts. Without the two-draw fix, every resistor-only
    // node accumulates Nyquist energy → mV-range RMS + lag-1 ≈ -1.
    assert!(
        lag1 > -0.3,
        "Passive-divider Nyquist artifact: lag1={lag1:.4} (expected > -0.3). \
         A purely resistive node should NOT exhibit a (+/-1) alternation in noise. \
         RMS={rms:.3e}, var={var:.3e}"
    );
    assert!(
        rms < 200e-6,
        "Passive-divider noise RMS {rms:.3e} V exceeds 200 µV. \
         Expected single-digit µV for a resistor divider at room temperature. \
         lag1={lag1:.4}"
    );
    assert!(
        var > 1e-20,
        "Passive-divider noise variance {var:.3e} V² near-zero — noise stamp \
         may be silent on linear-only circuits. Check NOISE_THERMAL_N > 0."
    );
}

/// Regression: same diode + series-R Nyquist topology as the load-bearing
/// test, but compiled via the **nodal** codegen path
/// (`generate_circuit_code_nodal`). The two-draw fix lives in the shared
/// `build_noise_emission` and is supposed to apply to both codegen paths;
/// this test pins that claim.
#[test]
fn thermal_noise_no_nyquist_artifact_on_resistor_only_output_node_nodal() {
    const SPICE: &str = r#"* Diode + series output resistor — nodal-path Nyquist regression
R_drive in anode 10k
D1 anode 0 D1N4148
R_out anode out 6.8k
R_load out 0 56k
.model D1N4148 D(IS=1e-15)
.end
"#;
    let sr = 48_000.0_f64;
    let config = CodegenConfig {
        circuit_name: "noise_nyquist_regress_nodal".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![2],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Thermal,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code_nodal(SPICE, &config);
    let main = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr}_f64);
    state.set_seed(42);
    state.set_noise_enabled(true);
    state.set_temperature_k(290.0);
    state.set_noise_gain(1.0);
    state.set_thermal_gain(1.0);
    for _ in 0..5_000 {{ let _ = process_sample(1.0, &mut state); }}
    let n = 1usize << 16;
    let mut samples = vec![0.0f64; n];
    for i in 0..n {{ samples[i] = process_sample(1.0, &mut state)[0]; }}
    let mean = samples.iter().sum::<f64>() / n as f64;
    let var = samples.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n as f64;
    let lag1_num = samples[..n-1].iter().zip(samples[1..].iter())
        .map(|(&a, &b)| (a - mean) * (b - mean)).sum::<f64>() / (n - 1) as f64;
    let lag1 = if var > 0.0 {{ lag1_num / var }} else {{ 0.0 }};
    println!("RMS={{:.15e}}", var.sqrt());
    println!("LAG1={{:.8}}", lag1);
}}
"#,
        sr = sr,
    );
    let out = support::compile_and_run(&code, &main, "noise_nyquist_regress_nodal");
    let rms: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("RMS=").and_then(|v| v.parse().ok()))
        .expect("RMS not found");
    let lag1: f64 = out
        .stdout
        .lines()
        .find_map(|l| l.strip_prefix("LAG1=").and_then(|v| v.parse().ok()))
        .expect("LAG1 not found");

    assert!(
        lag1 > -0.5,
        "Nodal-path Nyquist artifact detected: lag1={lag1:.4} (expected > -0.5). \
         The two-draw stamp may not be wired into the nodal codegen path. \
         RMS={rms:.3e}"
    );
    assert!(
        rms < 100e-6,
        "Nodal-path noise RMS {rms:.3e} V exceeds 100 µV. lag1={lag1:.4}"
    );
}

// ===========================================================================
// Phase 3.5 — Resistor flicker (Hooge bias-squared)
// ===========================================================================
//
// All four tests share a passive divider:
//
//   V_in → Rin (1k) → node a → R1 (10k, KF set) → 0
//                              C1 (100n) ↓
//                              0
//
// `dc_block: false` so we measure variance in raw output. With V_in = 1 V DC
// the steady-state current through R1 is V_in / (Rin + R1) = 90.9 µA — that's
// the load-bearing bias for the "biased" cases. With V_in = 0 there is no DC
// current, so Hooge predicts zero excess 1/f.
//
// The output is sampled at node `a` (the divider midpoint). The 100 nF cap
// shunts AC components above ~159 Hz, leaving the ~1/f tail measurable in
// the audio band as low-frequency variance.

const R_FLICKER_DIVIDER_SPICE: &str = r#"* R1 with KF in passive divider
Rin in a 1k
R1 a 0 10k KF=1e-3 AF=2.0
C1 a 0 100n
.end
"#;

// Same topology, no KF — used as the "thermal-only" reference baseline.
const R_FLICKER_DIVIDER_NO_KF_SPICE: &str = r#"* R1 in same divider, no flicker
Rin in a 1k
R1 a 0 10k
C1 a 0 100n
.end
"#;

fn build_r_flicker_test_main(fs: f64, drive_dc: f64, n_warmup: usize, n_samples: usize) -> String {
    format!(
        r#"
fn run_case(thermal: f64, flicker: f64, temperature: f64, seed: u64) -> f64 {{
    let mut state = CircuitState::default();
    state.set_sample_rate({fs}_f64);
    state.set_seed(seed);
    state.set_noise_enabled(true);
    state.set_temperature_k(temperature);
    state.set_thermal_gain(thermal);
    state.set_flicker_gain(flicker);
    state.set_noise_gain(1.0);

    for _ in 0..{w} {{ let _ = process_sample({drive}_f64, &mut state); }}

    let mut sum = 0.0_f64;
    let mut sum_sq = 0.0_f64;
    let n = {n};
    for _ in 0..n {{
        let v = process_sample({drive}_f64, &mut state)[0];
        sum += v;
        sum_sq += v * v;
    }}
    let nf = n as f64;
    let mean = sum / nf;
    (sum_sq / nf - mean * mean).max(0.0)
}}
"#,
        fs = fs,
        drive = drive_dc,
        w = n_warmup,
        n = n_samples,
    )
}

fn r_flicker_circuit_code(spice: &str) -> String {
    let config = CodegenConfig {
        circuit_name: "r_flicker_divider".to_string(),
        sample_rate: 96_000.0,
        input_node: 0,
        output_nodes: vec![1], // node `a` (1-indexed; `in` is node 0 (input port), `a` is node 1)
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Full,
        noise_master_seed: 7,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(spice, &config);
    code
}

#[test]
fn r_flicker_unbiased_resistor_emits_no_excess_one_over_f() {
    // Load-bearing Hooge correctness: a resistor with KF set but zero DC
    // bias and zero signal must emit thermal-only — NO excess 1/f. This is
    // the test that distinguishes correct bias-squared physics from the
    // original spec's phenomenological constant-floor hack.
    //
    // Strategy: run the SAME circuit twice with the SAME seed.
    //   - flicker=1, thermal=1: full noise
    //   - flicker=0, thermal=1: thermal only
    // With zero drive (no current through R1) the two variances must agree
    // within ±5% — the flicker `continue` guard (i_abs < 1e-15) skips the
    // RNG advance + Kellett tick on every sample, leaving the per-sample
    // RNG sequence identical.
    let code = r_flicker_circuit_code(R_FLICKER_DIVIDER_SPICE);
    let main = format!(
        r#"{}
fn main() {{
    let var_full     = run_case(1.0, 1.0, 290.0, 7);
    let var_thermal  = run_case(1.0, 0.0, 290.0, 7);
    let var_flick_0  = run_case(0.0, 1.0, 290.0, 7);
    println!("VAR:full={{:.15e}}",     var_full);
    println!("VAR:thermal={{:.15e}}",  var_thermal);
    println!("VAR:flicker_only={{:.15e}}", var_flick_0);
}}
"#,
        build_r_flicker_test_main(96_000.0, 0.0, 8_000, 1usize << 16)
    );
    let out = support::compile_and_run(&code, &main, "r_flicker_unbiased");
    let var_full = parse_var(&out.stdout, "full");
    let var_thermal = parse_var(&out.stdout, "thermal");
    let var_flicker_only = parse_var(&out.stdout, "flicker_only");

    // (1) Unbiased: full ≈ thermal-only — no excess 1/f when zero current.
    let ratio = var_full / var_thermal;
    assert!(
        (0.95..=1.05).contains(&ratio),
        "Unbiased resistor leaked excess 1/f: var_full={:.3e}, var_thermal={:.3e}, ratio={:.4} \
         (Hooge requires zero excess at zero bias; if this fails, check the i_abs<1e-15 guard \
         in the r_flicker rhs_stamp)",
        var_full,
        var_thermal,
        ratio
    );

    // (2) Sanity: flicker-only with zero drive must be ~zero (well below
    //     the thermal baseline). Confirms the stamp doesn't inject without
    //     bias even when thermal is muted.
    assert!(
        var_flicker_only < var_thermal * 1e-3,
        "Unbiased flicker-only variance should be ~zero, got {:.3e} (thermal baseline {:.3e}, \
         ratio {:.3e})",
        var_flicker_only,
        var_thermal,
        var_flicker_only / var_thermal
    );
}

#[test]
fn r_flicker_biased_resistor_audibly_wired() {
    // Mirror of `flicker_noise_is_audibly_wired` for the resistor path.
    // With V_in = 1 V DC there is ~90 µA flowing through R1; flicker should
    // contribute audibly above the thermal floor and be mute-able via
    // `set_flicker_gain(0.0)`.
    let code = r_flicker_circuit_code(R_FLICKER_DIVIDER_SPICE);
    let main = format!(
        r#"{}
fn main() {{
    let var_full     = run_case(1.0, 1.0, 290.0, 11);
    let var_thermal  = run_case(1.0, 0.0, 290.0, 11);
    let var_repeat   = run_case(1.0, 1.0, 290.0, 11);
    println!("VAR:full={{:.15e}}",    var_full);
    println!("VAR:thermal={{:.15e}}", var_thermal);
    println!("VAR:repeat={{:.15e}}",  var_repeat);
}}
"#,
        build_r_flicker_test_main(96_000.0, 1.0, 8_000, 1usize << 16)
    );
    let out = support::compile_and_run(&code, &main, "r_flicker_biased_wired");
    let var_full = parse_var(&out.stdout, "full");
    let var_thermal = parse_var(&out.stdout, "thermal");
    let var_repeat = parse_var(&out.stdout, "repeat");

    // (1) Biased + flicker on → variance materially exceeds thermal-only.
    //     At KF=1e-3 the flicker contribution dominates by orders of
    //     magnitude; even a 2× margin is conservative.
    assert!(
        var_full > var_thermal * 2.0,
        "Biased flicker did not raise the variance floor: var_full={:.3e}, var_thermal={:.3e}, \
         ratio={:.3e} (expected >> 2)",
        var_full,
        var_thermal,
        var_full / var_thermal
    );

    // (2) Same seed → bit-identical variance (determinism contract).
    assert_eq!(
        var_full.to_bits(),
        var_repeat.to_bits(),
        "Resistor-flicker determinism violated: var_full={:.15e} repeat={:.15e}",
        var_full,
        var_repeat
    );
}

#[test]
fn r_flicker_is_temperature_independent() {
    // Hooge bias-driven 1/f is *temperature-independent* — `set_temperature_k`
    // affects only thermal noise. With thermal muted (`thermal_gain = 0`)
    // and a steady DC bias on R1, the integrated variance must be invariant
    // under T sweeps within ±10% (loose because flicker draws are noisy and
    // we're integrating over a finite window).
    //
    // This is the test that catches a future regression where someone wires
    // `r_flicker_sqrt_fs` through `noise_thermal_scale` (which carries T)
    // by accident.
    let code = r_flicker_circuit_code(R_FLICKER_DIVIDER_SPICE);
    let main = format!(
        r#"{}
fn main() {{
    let var_290 = run_case(0.0, 1.0, 290.0, 17);
    let var_77  = run_case(0.0, 1.0,  77.0, 17);
    let var_500 = run_case(0.0, 1.0, 500.0, 17);
    println!("VAR:t290={{:.15e}}", var_290);
    println!("VAR:t77={{:.15e}}",  var_77);
    println!("VAR:t500={{:.15e}}", var_500);
}}
"#,
        build_r_flicker_test_main(96_000.0, 1.0, 8_000, 1usize << 16)
    );
    let out = support::compile_and_run(&code, &main, "r_flicker_t_indep");
    let v290 = parse_var(&out.stdout, "t290");
    let v77 = parse_var(&out.stdout, "t77");
    let v500 = parse_var(&out.stdout, "t500");

    let ratio_77 = v77 / v290;
    let ratio_500 = v500 / v290;

    assert!(
        (0.90..=1.10).contains(&ratio_77),
        "1/f variance changed with T (290→77 K): ratio={:.4} (expected ~1.0). Hooge is T-independent.",
        ratio_77
    );
    assert!(
        (0.90..=1.10).contains(&ratio_500),
        "1/f variance changed with T (290→500 K): ratio={:.4} (expected ~1.0).",
        ratio_500
    );
}

#[test]
fn r_flicker_floor_scales_with_drive() {
    // Spec rev2 §validation 3 — "loud passages get hissier". With thermal
    // muted, the integrated 1/f variance should scale as |I_R|^AF = I².
    // Doubling the drive doubles the current (passive divider, linear),
    // which should ~quadruple the variance (within seed-tolerance).
    let code = r_flicker_circuit_code(R_FLICKER_DIVIDER_SPICE);
    let main = format!(
        r#"
{builder1}{builder3}
fn main() {{
    let var_drive_1v = run_drive_1v();
    let var_drive_3v = run_drive_3v();
    println!("VAR:drive_1v={{:.15e}}", var_drive_1v);
    println!("VAR:drive_3v={{:.15e}}", var_drive_3v);
}}
"#,
        builder1 = build_r_flicker_test_main(96_000.0, 1.0, 8_000, 1usize << 16)
            .replace("fn run_case", "fn run_drive_1v_inner")
            + "\nfn run_drive_1v() -> f64 { run_drive_1v_inner(0.0, 1.0, 290.0, 31) }\n",
        builder3 = build_r_flicker_test_main(96_000.0, 3.0, 8_000, 1usize << 16)
            .replace("fn run_case", "fn run_drive_3v_inner")
            + "\nfn run_drive_3v() -> f64 { run_drive_3v_inner(0.0, 1.0, 290.0, 31) }\n",
    );
    let out = support::compile_and_run(&code, &main, "r_flicker_drive_scaling");
    let v1 = parse_var(&out.stdout, "drive_1v");
    let v3 = parse_var(&out.stdout, "drive_3v");

    // I scales as drive (linear divider). With AF=2.0 the variance scales
    // as I² ⇒ V_3v / V_1v should be ~9× (3²). Loose ±25% tolerance because
    // the Kellett filter and finite integration window add seed-noise.
    let ratio = v3 / v1;
    assert!(
        (6.0..=12.0).contains(&ratio),
        "1/f variance did not scale ~9× with 3× drive (AF=2): ratio={:.3} \
         (var_drive_1v={:.3e}, var_drive_3v={:.3e})",
        ratio,
        v1,
        v3
    );
}

// (No-KF byte-identity baseline lives in
// `codegen_verification_tests.rs::noise_full_no_kf_resistors_emits_zero_r_flicker`
// — no need to re-run a compile cycle here just to assert thermal-only
// codegen on the same divider topology.)
#[allow(dead_code)]
const _R_FLICKER_NO_KF_DIVIDER_REFERENCE: &str = R_FLICKER_DIVIDER_NO_KF_SPICE;

// =============================================================================
// Phase 5: Pentode partition noise validation
// =============================================================================
//
// Stamp formula (Schottky 1918, textbook):
//   S_i(plate) = 2·q · I_p · I_s / (I_p + I_s)   [A²/Hz]
//
// Replaces the Phase 2 bare plate-shot `2q·I_p` — pentodes at matched I_p are
// quieter than triodes by the partition suppression factor `I_s/(I_p+I_s)`.
// Per-sample injected current at rate `fs`:
//   sqrt(4·q · I_p·I_s/(I_p+I_s) · fs) · PARTITION_F · N(0,1)
// with the same 2× trap-MNA compensation as thermal/shot/junction-flicker
// and the same two-draw Nyquist anti-alias.

const PENTODE_PARTITION_SPICE: &str = "\
EL84 partition-noise test bench
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
P1 plate grid cathode screen EL84
Rk cathode 0 130
Ck cathode 0 100u
Rscreen vcc screen 1k
Cscreen screen 0 47u
Rp vcc plate 4.7k
Cout plate out 1u
Rout out 0 100k
V1 vcc 0 DC 300
.model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275.0 KP=152.4 KVB=4015.8 ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148)
";

const PENTODE_PARTITION_HALF_SPICE: &str = "\
EL84 partition-noise test bench with PARTITION_F=0.5
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
P1 plate grid cathode screen EL84
Rk cathode 0 130
Ck cathode 0 100u
Rscreen vcc screen 1k
Cscreen screen 0 47u
Rp vcc plate 4.7k
Cout plate out 1u
Rout out 0 100k
V1 vcc 0 DC 300
.model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275.0 KP=152.4 KVB=4015.8 ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148 PARTITION_F=0.5)
";

const TRIODE_NO_PARTITION_SPICE: &str = "\
12AX7 single-stage common cathode (control: no partition expected)
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
T1 plate grid cathode 12AX7
Rk cathode 0 1.5k
Ck cathode 0 22u
Rp vcc plate 100k
Cout plate out 1u
Rout out 0 1Meg
V1 vcc 0 DC 250
.model 12AX7 VT(MU=100 EX=1.4 KG1=600 KP=300 KVB=300)
";

fn generate_partition_code(spice: &str, mode: NoiseMode, name: &str, seed: u64) -> String {
    let config = CodegenConfig {
        circuit_name: name.to_string(),
        sample_rate: 96_000.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: mode,
        noise_master_seed: seed,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(spice, &config);
    code
}

/// Pentode partition is collected and emitted under `--noise full`.
/// The presence of `NOISE_PARTITION_N`, the per-source const tables, the
/// salt, and the rhs stamp identifies a partition source. Defaults make
/// `PARTITION_F = 1.0` (textbook Schottky).
#[test]
fn partition_emits_constants_for_pentode_under_noise_full() {
    let code = generate_partition_code(
        PENTODE_PARTITION_SPICE,
        NoiseMode::Full,
        "partition_emit_full",
        42,
    );
    assert!(
        code.contains("pub const NOISE_PARTITION_N: usize = 1"),
        "expected NOISE_PARTITION_N=1 for single-pentode circuit"
    );
    assert!(
        code.contains("NOISE_PARTITION_IP_SLOT"),
        "expected NOISE_PARTITION_IP_SLOT array"
    );
    assert!(
        code.contains("NOISE_PARTITION_IS_SLOT"),
        "expected NOISE_PARTITION_IS_SLOT array (screen-current slot)"
    );
    assert!(
        code.contains("NOISE_PARTITION_F"),
        "expected NOISE_PARTITION_F array baking the per-device multiplier"
    );
    assert!(
        code.contains("NOISE_PARTITION_SALT"),
        "expected NOISE_PARTITION_SALT for the dedicated RNG stream"
    );
    assert!(
        code.contains("noise_partition_rng"),
        "expected noise_partition_rng state field"
    );
    assert!(
        code.contains("noise_partition_w_prev"),
        "expected two-draw lag buffer for partition (Nyquist anti-alias)"
    );
    assert!(
        code.contains("noise_partition_last_i_n"),
        "expected BE-replay cache for partition"
    );
    // Default PARTITION_F is 1.0 (textbook Schottky). `fmt_f64` emits
    // scientific notation, so 1.0 renders as `1.00000000000000000e0`.
    assert!(
        code.contains("NOISE_PARTITION_F: [f64; NOISE_PARTITION_N] = [1.0") && code.contains("e0]"),
        "default PARTITION_F should be 1.0 (textbook); got:\n{}",
        code.lines()
            .find(|l| l.contains("NOISE_PARTITION_F"))
            .unwrap_or("<not found>")
    );
}

/// `--noise shot` mode includes thermal + shot but NOT partition (partition
/// gates on `includes_full()` like junction flicker). A pentode under
/// `--noise shot` therefore emits *no* plate-side noise at all — the bare
/// plate-shot is skipped by `collect_shot_noise_sources` since partition
/// owns that port, and partition itself is gated off.
///
/// This is the intended contract: partition is a "full" feature; users who
/// want pentodes loud need `--noise full`.
#[test]
fn pentode_under_noise_shot_emits_no_plate_noise() {
    let code = generate_partition_code(
        PENTODE_PARTITION_SPICE,
        NoiseMode::Shot,
        "partition_shot_mode",
        42,
    );
    // Partition not emitted under --noise shot (it gates on includes_full).
    assert!(
        !code.contains("NOISE_PARTITION_N"),
        "partition should not be emitted under --noise shot"
    );
    // Pentode plate Ip slot=0 should NOT appear as a shot source either —
    // the collector filters pentode plate ports out (partition owns that
    // junction). For this circuit the pentode is the only nonlinear device,
    // so NOISE_SHOT_N should be absent (no other shot sources).
    assert!(
        !code.contains("NOISE_SHOT_N"),
        "single-pentode circuit under --noise shot should emit no shot \
         sources (pentode plate goes through partition, not bare shot)"
    );
}

/// `.model TUBE(PARTITION_F=0.5)` flows through the parser, the pentode IR
/// resolver, and the codegen const table. A user who selects low-noise EF86
/// batches gets the right per-sample amplitude scaling baked into the
/// generated code.
#[test]
fn partition_f_override_propagates_to_codegen() {
    let code = generate_partition_code(
        PENTODE_PARTITION_HALF_SPICE,
        NoiseMode::Full,
        "partition_f_half",
        42,
    );
    // fmt_f64 emits scientific notation: 0.5 → `5.00000000000000000e-1`.
    assert!(
        code.contains("NOISE_PARTITION_F: [f64; NOISE_PARTITION_N] = [5.0")
            && code.contains("e-1]"),
        "PARTITION_F=0.5 should propagate as 5.0…e-1; got:\n{}",
        code.lines()
            .find(|l| l.contains("NOISE_PARTITION_F"))
            .unwrap_or("<not found>")
    );
}

/// Triodes get bare Schottky plate shot, not partition. The collector skips
/// only 4/5-node Tube devices; triodes (3 nodes) emit a shot port at the
/// plate. A pure-triode circuit must produce no `NOISE_PARTITION_*` symbols
/// at all — byte-identical noise emission to a pre-Phase-5 build for
/// triode-only stages.
#[test]
fn triode_emits_no_partition_under_noise_full() {
    let code = generate_partition_code(
        TRIODE_NO_PARTITION_SPICE,
        NoiseMode::Full,
        "triode_no_partition",
        42,
    );
    assert!(
        !code.contains("NOISE_PARTITION_N"),
        "triode-only circuit must not emit NOISE_PARTITION_N"
    );
    assert!(
        !code.contains("NOISE_PARTITION_F"),
        "triode-only circuit must not emit NOISE_PARTITION_F"
    );
    assert!(
        !code.contains("noise_partition_rng"),
        "triode-only circuit must not emit partition RNG state"
    );
    assert!(
        !code.contains("NOISE_PARTITION_SALT"),
        "triode-only circuit must not emit partition salt"
    );
    // Triode plate shot is still wired (Phase 2). One Ip shot source for
    // the single triode in this circuit.
    assert!(
        code.contains("NOISE_SHOT_N: usize = 1"),
        "triode-only circuit should emit one bare plate-shot source"
    );
}

/// Circuit with no pentodes (and no per-resistor KF) under `--noise full`
/// must remain byte-identical to a pre-Phase-5 build: no partition symbols,
/// no extra state, no extra constants. Companion to the existing
/// `noise_full_no_kf_resistors_emits_zero_r_flicker` byte-identity guard.
#[test]
fn rc_lowpass_under_noise_full_emits_no_partition() {
    let config = CodegenConfig {
        circuit_name: "rc_no_partition".to_string(),
        sample_rate: 96_000.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Full,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(RC_SPICE, &config);
    assert!(
        !code.contains("NOISE_PARTITION"),
        "no-pentode circuit must not emit any NOISE_PARTITION_* symbols \
         under --noise full"
    );
}

// =============================================================================
// Phase 4: Op-amp input-referred noise (en/in) validation
// =============================================================================
//
// Three Norton streams per opted-in op-amp:
//   en  at NODE_PLUS : amp = en  · noise_opamp_en_g_diag[k] · sqrt(2·fs)
//   in+ at NODE_PLUS : amp = in  · sqrt(2·fs)
//   in- at NODE_MINUS: amp = in  · sqrt(2·fs)
// Two-draw Nyquist anti-alias on all three. `noise_opamp_en_g_diag[k]` is
// initialized to the static `G[in+, in+]` at codegen time and refreshed in
// the emitted `set_pot_N` / `set_runtime_R_<field>` body when a pot touches
// the op-amp's non-inverting input (incremental `delta_g = 1/r − 1/r_old`).

// NE5534-style single-stage non-inverting amp: input via R_source, fixed
// feedback divider, supplies via VCC/VEE. EN=3.5e-9 and IN=1.5e-12 are the
// canonical NE5534 datasheet numbers used by Noyce's noyce-ne5534.cir.
const OPAMP_NE5534_SPICE: &str = "\
NE5534 single-stage non-inverting amp
Rsrc in 1 1k
Rg1 0 2 1k
Rf 2 out 10k
U1 1 2 out NE5534
V1 vcc 0 DC 15
V2 vee 0 DC -15
Rload out 0 10k
.model NE5534 OA(AOL=200000 GBW=10MEG ROUT=75 VCC=15 VEE=-15 EN=3.5e-9 IN=1.5e-12)
";

// Same topology with a level pot in the input path — exercises the dynamic
// `noise_opamp_en_g_diag[k]` refresh in `set_pot_N`. The pot sits between
// the input network and the op-amp's non-inverting input (node 1), so a
// pot terminal coincides with `oa.n_plus_idx`. The pot's conductance
// stamps positively into `G[1, 1]`, so changing it shifts the live G_diag
// at in+. (`.pot` modifies an existing R; declare it first as Rlevel.)
const OPAMP_NE5534_WITH_INPUT_POT_SPICE: &str = "\
NE5534 with level pot in input path
Rlevel 1 0 25k
Rg1 0 2 1k
Rf 2 out 10k
U1 1 2 out NE5534
V1 vcc 0 DC 15
V2 vee 0 DC -15
Rload out 0 10k
.pot Rlevel 100 50k
.model NE5534 OA(AOL=200000 GBW=10MEG ROUT=75 VCC=15 VEE=-15 EN=3.5e-9 IN=1.5e-12)
";

// Op-amp circuit without EN/IN — must produce byte-identical codegen to a
// pre-Phase-4 build under `--noise full`. The OA model card omits both
// noise parameters; `collect_opamp_noise_sources` filters this op-amp out
// of `partition_sources`/`opamp_noise_sources` entirely.
const OPAMP_NO_NOISE_SPICE: &str = "\
Ideal op-amp with no EN/IN — no Phase 4 noise emission
Rin in 1 1k
Rg1 0 2 1k
Rf 2 out 10k
U1 1 2 out IDEAL_OA
V1 vcc 0 DC 15
V2 vee 0 DC -15
Rload out 0 10k
.model IDEAL_OA OA(AOL=1e6 ROUT=1 VCC=15 VEE=-15)
";

/// Op-amp en/in noise is collected and emitted under `--noise full`.
/// Presence of `NOISE_OPAMP_N`, EN/IN const tables, EN_G_DIAG_DEFAULT, both
/// salts, the input-gain setter, and per-source state fields identifies a
/// well-formed Phase 4 source. The static G_diag default must be positive
/// (the input network has a finite admittance at `in+`).
#[test]
fn opamp_emits_constants_for_ne5534_under_noise_full() {
    let code = generate_partition_code(OPAMP_NE5534_SPICE, NoiseMode::Full, "opamp_emit_full", 42);
    assert!(
        code.contains("pub const NOISE_OPAMP_N: usize = 1"),
        "expected NOISE_OPAMP_N=1 for single-op-amp NE5534 circuit"
    );
    assert!(
        code.contains("pub const NOISE_OPAMP_IN_N: usize = 2"),
        "expected NOISE_OPAMP_IN_N=2 (one in+ + one in- per op-amp)"
    );
    assert!(
        code.contains("NOISE_OPAMP_EN") && code.contains("NOISE_OPAMP_IN"),
        "expected NOISE_OPAMP_EN and NOISE_OPAMP_IN arrays"
    );
    assert!(
        code.contains("NOISE_OPAMP_NODE_PLUS") && code.contains("NOISE_OPAMP_NODE_MINUS"),
        "expected per-source node-index arrays"
    );
    assert!(
        code.contains("NOISE_OPAMP_EN_G_DIAG_DEFAULT"),
        "expected baked static G_diag(in+) default array"
    );
    assert!(
        code.contains("NOISE_OPAMP_EN_SALT") && code.contains("NOISE_OPAMP_IN_SALT"),
        "expected both Phase 4 salts (EN and IN distinct streams)"
    );
    assert!(
        code.contains("noise_opamp_en_rng") && code.contains("noise_opamp_in_rng"),
        "expected per-source xoshiro state for en and in streams"
    );
    assert!(
        code.contains("noise_opamp_en_g_diag"),
        "expected dynamic-refreshable G_diag(in+) mirror in state"
    );
    assert!(
        code.contains("noise_opamp_en_w_prev") && code.contains("noise_opamp_in_w_prev"),
        "expected two-draw Nyquist anti-alias buffers for en and in"
    );
    assert!(
        code.contains("noise_opamp_en_last_i_n") && code.contains("noise_opamp_in_last_i_n"),
        "expected BE-replay caches for en and in"
    );
    assert!(
        code.contains("pub fn set_opamp_input_gain"),
        "expected set_opamp_input_gain runtime knob"
    );
    assert!(
        code.contains("opamp_input_gain: 1.0"),
        "expected opamp_input_gain default of 1.0"
    );
    // EN = 3.5e-9: f64 representation rounds slightly so fmt_f64 may emit
    // `3.4…e-9` or `3.5…e-9` depending on the bit pattern. Check the
    // exponent and approximate leading digit.
    let en_line = code
        .lines()
        .find(|l| l.contains("NOISE_OPAMP_EN: [f64; NOISE_OPAMP_N]"))
        .expect("expected NOISE_OPAMP_EN line in emitted code");
    assert!(
        en_line.contains("e-9") && (en_line.contains("[3.") || en_line.contains("[3,")),
        "EN=3.5e-9 should propagate as ~3.5e-9; got line: {en_line}"
    );
    let in_line = code
        .lines()
        .find(|l| l.contains("NOISE_OPAMP_IN: [f64; NOISE_OPAMP_N]"))
        .expect("expected NOISE_OPAMP_IN line in emitted code");
    assert!(
        in_line.contains("e-12") && (in_line.contains("[1.") || in_line.contains("[1,")),
        "IN=1.5e-12 should propagate as ~1.5e-12; got line: {in_line}"
    );
}

/// `--noise shot` does NOT emit op-amp noise — Phase 4 gates on
/// `includes_full()` like junction flicker and partition. Op-amp circuits
/// under `--noise shot` get thermal + shot (no opt-in op-amps for shot
/// either, since op-amps are linear/M=0) but no en/in.
#[test]
fn opamp_under_noise_shot_emits_no_opamp_noise() {
    let code = generate_partition_code(OPAMP_NE5534_SPICE, NoiseMode::Shot, "opamp_shot_mode", 42);
    assert!(
        !code.contains("NOISE_OPAMP_N"),
        "op-amp noise should not be emitted under --noise shot"
    );
    assert!(
        !code.contains("set_opamp_input_gain"),
        "set_opamp_input_gain should not be emitted without Phase 4 sources"
    );
}

/// An op-amp `.model OA(...)` without EN or IN produces byte-identical
/// codegen to a pre-Phase-4 build — `collect_opamp_noise_sources` filters
/// it out, so no Phase 4 symbols are emitted under `--noise full`.
#[test]
fn opamp_without_en_in_emits_no_phase_4_symbols() {
    let code = generate_partition_code(OPAMP_NO_NOISE_SPICE, NoiseMode::Full, "opamp_no_noise", 42);
    assert!(
        !code.contains("NOISE_OPAMP_N"),
        "op-amp without EN/IN must not emit Phase 4 noise constants"
    );
    assert!(
        !code.contains("noise_opamp_en_rng"),
        "op-amp without EN/IN must not emit Phase 4 state fields"
    );
    assert!(
        !code.contains("set_opamp_input_gain"),
        "op-amp without EN/IN must not emit set_opamp_input_gain"
    );
}

/// Non-op-amp circuit under `--noise full`: no Phase 4 symbols emitted,
/// byte-identical to pre-Phase-4 builds. Companion to the partition no-pentode
/// guard.
#[test]
fn non_opamp_circuit_under_noise_full_emits_no_opamp_noise() {
    let config = CodegenConfig {
        circuit_name: "rc_no_opamp_noise".to_string(),
        sample_rate: 96_000.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Full,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(RC_SPICE, &config);
    assert!(
        !code.contains("NOISE_OPAMP"),
        "non-op-amp circuit must emit no NOISE_OPAMP_* symbols under --noise full"
    );
}

/// Phase 4 dynamic-R refresh: a `.pot` whose node terminates at an op-amp's
/// non-inverting input must produce per-pot refresh code in `set_pot_N`
/// that updates `state.noise_opamp_en_g_diag[k]`. Validates the
/// `pot_to_opamp_en_refresh` lookup and the incremental-delta update
/// (`+= 1/r - 1/r_old`).
#[test]
fn opamp_en_g_diag_refreshes_on_pot_touching_in_plus() {
    let code = generate_partition_code(
        OPAMP_NE5534_WITH_INPUT_POT_SPICE,
        NoiseMode::Full,
        "opamp_en_pot_refresh",
        42,
    );
    // Refresh code must be present inside the pot setter.
    assert!(
        code.contains("self.noise_opamp_en_g_diag[0] += opamp_g_delta"),
        "set_pot_N for a pot touching op-amp in+ must refresh en_g_diag[0]"
    );
    // The delta computation uses old/new pot resistances.
    assert!(
        code.contains("let opamp_g_delta = 1.0 / r - 1.0 / r_old"),
        "refresh body must compute opamp_g_delta = 1/r − 1/r_old"
    );
    // r_old must be saved BEFORE the resistance is overwritten.
    assert!(
        code.contains("let r_old = self.pot_0_resistance"),
        "refresh must save r_old before the new resistance is stored"
    );
}

/// The op-amp en/in codegen path must produce valid Rust that compiles
/// and runs without panicking. This is the structural smoke test —
/// verifies state-field declarations, default-impl initializers, the
/// `set_opamp_input_gain` runtime method, the rhs stamp, BE replay, NaN
/// recovery, and set_seed all type-check and execute end-to-end. PSD
/// validation is plugin-side (per the Noyce response letter).
#[test]
fn opamp_noise_emission_compiles_and_runs() {
    let sr = 96_000.0;
    let config = CodegenConfig {
        circuit_name: "opamp_emit_runtime".to_string(),
        sample_rate: sr,
        input_node: 0,
        output_nodes: vec![3],
        input_resistance: 1.0,
        dc_block: false,
        noise_mode: NoiseMode::Full,
        noise_master_seed: 42,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(OPAMP_NE5534_SPICE, &config);
    let main = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    state.set_sample_rate({sr}_f64);
    state.set_seed(42);
    state.set_noise_enabled(true);
    state.set_opamp_input_gain(1.0);
    // A handful of samples — just enough to exercise the rhs_stamp and
    // confirm process_sample doesn't panic / NaN on op-amp noise.
    let mut last = 0.0_f64;
    for _ in 0..32 {{
        last = process_sample(0.0, &mut state)[0];
    }}
    println!("VAR:final={{:.15e}}", last);
    // Mute path
    state.set_opamp_input_gain(0.0);
    for _ in 0..16 {{
        last = process_sample(0.0, &mut state)[0];
    }}
    println!("VAR:muted={{:.15e}}", last);
}}
"#,
        sr = sr
    );
    let out = support::compile_and_run(&code, &main, "opamp_emit_runtime");
    let final_v = parse_var(&out.stdout, "final");
    let muted_v = parse_var(&out.stdout, "muted");
    assert!(
        final_v.is_finite(),
        "op-amp noise rhs_stamp should produce finite output, got {final_v}"
    );
    assert!(
        muted_v.is_finite(),
        "muted op-amp output should be finite, got {muted_v}"
    );
}

/// Pots that don't touch any op-amp's in+ must NOT emit refresh code —
/// byte-identical to a non-op-amp build at the pot-setter level. Uses the
/// existing thermal-noise pot test circuit (RC with a pot between two nodes
/// neither of which is an op-amp input — in fact, no op-amps at all).
#[test]
fn pot_not_touching_opamp_emits_no_opamp_refresh() {
    const SIMPLE_POT_RC_SPICE: &str = "\
Simple RC with a pot — no op-amps
R1 in mid 10k
R2 mid 0 50k
C1 mid 0 100n
.pot R2 100 100k
";
    let code = generate_partition_code(
        SIMPLE_POT_RC_SPICE,
        NoiseMode::Full,
        "pot_no_opamp_refresh",
        42,
    );
    assert!(
        !code.contains("opamp_g_delta"),
        "pot setter must emit no opamp_g_delta in the absence of op-amps"
    );
    assert!(
        !code.contains("noise_opamp_en_g_diag"),
        "circuit without op-amps must emit no en_g_diag refresh code"
    );
}
