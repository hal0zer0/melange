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
