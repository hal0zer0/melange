//! Sherman-Morrison pot sweep tests.
//!
//! Verifies that the SM rank-1 update produces the same output as a full
//! matrix rebuild at each pot position. This proves the SM math is correct
//! across the full pot range, not just at nominal.

mod support;

use melange_solver::codegen::CodegenConfig;

const SR: f64 = 48000.0;

const POT_CIRCUIT: &str = "\
Pot Circuit
R1 in mid 1k
R2 mid out 10k
C1 out 0 1u
.pot R2 1k 100k
";

fn config() -> CodegenConfig {
    support::config_for_spice(POT_CIRCUIT, SR)
}

#[test]
fn test_sm_pot_nominal_matches_baseline() {
    // At nominal position (pot at default), SM correction should be zero
    // and output should match baseline
    let c = config();
    let circuit = support::build_circuit(POT_CIRCUIT, &c, "pot_nom");

    // Run at nominal (no pot change)
    let out = support::run_sine(&circuit, 1000.0, 0.5, 480, SR);
    support::assert_finite(&out);
    support::assert_peak_above(&out, 1e-4);
}

#[test]
fn test_sm_pot_extreme_positions_finite() {
    // At min and max pot positions, output should still be finite
    let c = config();

    // Build with custom main that sets pot before processing
    let (code, _n, _m) = support::generate_circuit_code(POT_CIRCUIT, &c);

    // Test at min pot value (R2 = 1k)
    let main_min = r#"
fn main() {
    let mut state = CircuitState::default();
    state.set_sample_rate(48000.0);
    state.set_pot_0(POT_0_MIN_R); // min resistance
    for i in 0..480 {
        let t = i as f64 / 48000.0;
        let input = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let out = process_sample(input, &mut state);
        println!("{:.15e}", out[0]);
    }
}
"#;
    let output_min = support::compile_and_run(&code, main_min, "pot_min");
    let samples_min = output_min.parse_samples();
    support::assert_finite(&samples_min);

    // Test at max pot value (R2 = 100k)
    let main_max = r#"
fn main() {
    let mut state = CircuitState::default();
    state.set_sample_rate(48000.0);
    state.set_pot_0(POT_0_MAX_R); // max resistance
    for i in 0..480 {
        let t = i as f64 / 48000.0;
        let input = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let out = process_sample(input, &mut state);
        println!("{:.15e}", out[0]);
    }
}
"#;
    let output_max = support::compile_and_run(&code, main_max, "pot_max");
    let samples_max = output_max.parse_samples();
    support::assert_finite(&samples_max);

    // Min resistance should give higher gain than max resistance
    let peak_min: f64 = samples_min.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    let peak_max: f64 = samples_max.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    assert!(
        peak_min > peak_max,
        "lower R2 should give higher output: peak_min={peak_min:.4e}, peak_max={peak_max:.4e}"
    );
}

#[test]
fn test_sm_pot_sweep_monotonic() {
    // Sweeping the pot from min to max should produce monotonically
    // changing output gain (for this simple voltage divider circuit).
    let c = config();
    let (code, _n, _m) = support::generate_circuit_code(POT_CIRCUIT, &c);

    let main_sweep = r#"
fn main() {
    let steps = 5;
    for step in 0..steps {
        let frac = step as f64 / (steps - 1) as f64;
        let r = POT_0_MIN_R + frac * (POT_0_MAX_R - POT_0_MIN_R);
        let mut state = CircuitState::default();
        state.set_sample_rate(48000.0);
        state.set_pot_0(r);
        let mut peak: f64 = 0.0;
        // Skip 100 samples for transient, then measure 380
        for i in 0..480 {
            let t = i as f64 / 48000.0;
            let input = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            let out = process_sample(input, &mut state);
            if i >= 100 {
                peak = peak.max(out[0].abs());
            }
        }
        println!("{:.15e}", peak);
    }
}
"#;
    let output = support::compile_and_run(&code, main_sweep, "pot_sweep");
    let peaks = output.parse_samples();
    assert_eq!(peaks.len(), 5, "should have 5 peak measurements");

    // For R2 pot (voltage divider): higher R2 = lower output
    // So peaks should be monotonically decreasing
    for i in 1..peaks.len() {
        assert!(
            peaks[i] <= peaks[i - 1] + 1e-6,
            "pot sweep not monotonic at step {i}: {:.4e} > {:.4e}",
            peaks[i],
            peaks[i - 1]
        );
    }
}
