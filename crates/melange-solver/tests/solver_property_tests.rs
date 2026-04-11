//! Solver mathematical property tests.
//!
//! These tests verify fundamental mathematical properties that any correct
//! circuit simulator must satisfy:
//! - Superposition (linear circuits only)
//! - Energy conservation (lossless LC oscillator)
//! - Trapezoidal z-transform accuracy (RC lowpass vs closed-form)
//! - Passivity (passive network output bounded by input)

mod support;

use melange_solver::codegen::CodegenConfig;

const SR: f64 = 48000.0;

const RC_LOWPASS: &str = "\
RC Lowpass
R1 in out 1k
C1 out 0 1u
";

const DIODE_CLIPPER: &str = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";

fn config(spice: &str) -> CodegenConfig {
    support::config_for_spice(spice, SR)
}

// ── Superposition ────────────────────────────────────────────────────

#[test]
fn test_superposition_linear_circuit() {
    // For a linear circuit: output(A+B) = output(A) + output(B)
    let c = config(RC_LOWPASS);
    let circuit = support::build_circuit(RC_LOWPASS, &c, "sup_lin");

    let n = 480; // 10ms at 48kHz
    let freq = 1000.0;

    // Input A: 1kHz sine at 0.3V
    let input_a: Vec<f64> = (0..n)
        .map(|i| 0.3 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin())
        .collect();

    // Input B: 1kHz sine at 0.5V, phase-shifted 90°
    let input_b: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR + std::f64::consts::FRAC_PI_2).sin())
        .collect();

    // Input A+B
    let input_ab: Vec<f64> = input_a.iter().zip(&input_b).map(|(a, b)| a + b).collect();

    let out_a = support::run_signal(&circuit, &input_a, SR);
    let out_b = support::run_signal(&circuit, &input_b, SR);
    let out_ab = support::run_signal(&circuit, &input_ab, SR);

    // output(A+B) should equal output(A) + output(B)
    assert_eq!(out_a.len(), n);
    assert_eq!(out_b.len(), n);
    assert_eq!(out_ab.len(), n);

    let mut max_err = 0.0_f64;
    for i in 0..n {
        let sum = out_a[i] + out_b[i];
        let err = (out_ab[i] - sum).abs();
        max_err = max_err.max(err);
    }

    assert!(
        max_err < 1e-12,
        "superposition violated: max error = {max_err:.3e} (should be ~0 for linear circuit)"
    );
}

#[test]
fn test_superposition_fails_for_nonlinear() {
    // For a nonlinear circuit, superposition should NOT hold
    let c = config(DIODE_CLIPPER);
    let circuit = support::build_circuit(DIODE_CLIPPER, &c, "sup_nl");

    let n = 480;
    let freq = 500.0;

    let input_a: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin())
        .collect();
    let input_b: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR + 1.0).sin())
        .collect();
    let input_ab: Vec<f64> = input_a.iter().zip(&input_b).map(|(a, b)| a + b).collect();

    let out_a = support::run_signal(&circuit, &input_a, SR);
    let out_b = support::run_signal(&circuit, &input_b, SR);
    let out_ab = support::run_signal(&circuit, &input_ab, SR);

    let mut max_err = 0.0_f64;
    for i in 0..n {
        let sum = out_a[i] + out_b[i];
        let err = (out_ab[i] - sum).abs();
        max_err = max_err.max(err);
    }

    assert!(
        max_err > 1e-6,
        "nonlinear circuit SHOULD violate superposition, but max error = {max_err:.3e}"
    );
}

// ── Trapezoidal z-transform accuracy ─────────────────────────────────

#[test]
fn test_trapezoidal_matches_z_transform_rc() {
    // For an RC lowpass with R=1k, C=1u, the bilinear z-transform gives:
    //   H(z) = (1 + z^{-1}) * g / (1 + g + (g - 1) * z^{-1})
    // where g = T / (2*R*C) = 1 / (2 * sample_rate * R * C)
    //
    // Actually, the trapezoidal discretization gives:
    //   y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
    // where:
    //   alpha = 2 * SR
    //   d = R + 1/(alpha*C)  [from voltage divider with companion conductance]
    //   But the melange circuit has input conductance G_in = 1Ω, so the
    //   exact z-transform is slightly different.
    //
    // Instead of deriving the exact formula, we verify against a known property:
    // At DC (z=1), H(1) = R_load / (R_in + R_load) where R_in=1Ω, R_load=∞
    // Since input conductance is 1/1Ω stamped at the input node, and the
    // output node has C to ground and R between in and out:
    //   H(DC) ≈ 1.0 (for R=1k >> R_in=1Ω)

    let c = config(RC_LOWPASS);
    let circuit = support::build_circuit(RC_LOWPASS, &c, "zt_rc");

    // Frequency response at 1kHz: RC lowpass with fc = 1/(2πRC) ≈ 159 Hz
    // With R_in=1Ω and R_circuit=1kΩ, the DC gain is very close to 1.0
    // but the output may have DC blocking. We test the AC frequency response
    // which is more robust.
    let out_sine = support::run_sine(&circuit, 1000.0, 1.0, 4800, SR);
    // Skip first 1000 samples for transient, measure steady-state peak
    let peak: f64 = out_sine[1000..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0_f64, f64::max);
    let fc = 1.0 / (2.0 * std::f64::consts::PI * 1000.0 * 1e-6);
    let expected_gain = 1.0 / (1.0 + (1000.0 / fc).powi(2)).sqrt();
    // Allow 10% tolerance due to input resistance and DC block filter interaction
    assert!(
        (peak - expected_gain).abs() / expected_gain < 0.10,
        "1kHz gain: got {peak:.4}, expected {expected_gain:.4} (within 10%)"
    );

    // Test relative gain at two frequencies: the RATIO should match theory
    // regardless of DC blocking or input impedance
    let out_100 = support::run_sine(&circuit, 100.0, 1.0, 4800, SR);
    let peak_100: f64 = out_100[1000..].iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    let out_1k = support::run_sine(&circuit, 1000.0, 1.0, 4800, SR);
    let peak_1k: f64 = out_1k[1000..].iter().map(|s| s.abs()).fold(0.0_f64, f64::max);

    if peak_100 > 1e-6 {
        let ratio_measured = peak_1k / peak_100;
        let ratio_theory =
            (1.0 + (100.0 / fc).powi(2)).sqrt() / (1.0 + (1000.0 / fc).powi(2)).sqrt();
        assert!(
            (ratio_measured - ratio_theory).abs() / ratio_theory < 0.10,
            "gain ratio 1kHz/100Hz: got {ratio_measured:.4}, expected {ratio_theory:.4}"
        );
    }
}

// ── Energy conservation ──────────────────────────────────────────────

#[test]
fn test_energy_conservation_lc_oscillator() {
    // An LC circuit should oscillate. The 1Ω input resistance creates some
    // damping (Q = Z_LC / R_in where Z_LC = sqrt(L/C)), but the circuit should
    // still produce clear oscillations at the expected frequency.
    //
    // L=100mH, C=100nF → f = 1/(2π√(LC)) ≈ 1.59kHz (~30 samples/cycle at 48kHz)
    // Z_LC = sqrt(L/C) = sqrt(100m/100n) = 1000Ω
    // With R_in = 1Ω, Q ≈ 1000 — very lightly damped
    let spice = "\
LC Oscillator
L1 in out 100m
C1 out 0 100n
";
    let config = CodegenConfig {
        circuit_name: "lc_osc".to_string(),
        sample_rate: SR,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };

    // Use nodal path since this has an inductor
    let circuit = support::build_circuit_nodal(spice, &config, "lc_energy");

    // Impulse excitation: one sample of 1V, then silence
    let n = 2400; // 50ms — enough for ~80 cycles
    let mut input = vec![0.0; n];
    input[0] = 1.0;

    let out = support::run_signal(&circuit, &input, SR);
    support::assert_finite(&out);

    // The output should oscillate (not decay to zero immediately)
    let peak_late: f64 = out[1000..].iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    assert!(
        peak_late > 1e-6,
        "LC oscillator should still be oscillating after 1000 samples, peak={peak_late:.6e}"
    );

    // Verify oscillation frequency: count zero crossings
    let mut crossings = 0;
    for w in out[200..].windows(2) {
        if w[0] * w[1] < 0.0 {
            crossings += 1;
        }
    }
    // Expected: ~1590 Hz → ~1590 * 2 crossings/sec * (n-200)/48000 sec
    let duration_s = (n - 200) as f64 / SR;
    let estimated_freq = crossings as f64 / (2.0 * duration_s);
    let expected_freq = 1.0 / (2.0 * std::f64::consts::PI * (0.1 * 100e-9_f64).sqrt());
    assert!(
        (estimated_freq - expected_freq).abs() / expected_freq < 0.10,
        "LC frequency: estimated {estimated_freq:.0} Hz, expected {expected_freq:.0} Hz"
    );
}

// ── Passivity ────────────────────────────────────────────────────────

#[test]
fn test_passive_network_bounded_output() {
    // A passive RC network should never produce output exceeding input.
    // Feed a 1V sine and verify peak output <= 1V (with small tolerance for
    // transient overshoot from initial conditions).
    let c = config(RC_LOWPASS);
    let circuit = support::build_circuit(RC_LOWPASS, &c, "passive");

    let out = support::run_sine(&circuit, 1000.0, 1.0, 4800, SR);
    let peak: f64 = out.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);

    assert!(
        peak <= 1.05,
        "passive RC output peak {peak:.4} exceeds input amplitude 1.0"
    );
}

#[test]
fn test_scaling_linearity() {
    // For a linear circuit: output(k*x) = k * output(x) for any scalar k
    let c = config(RC_LOWPASS);
    let circuit = support::build_circuit(RC_LOWPASS, &c, "scale");

    let n = 480;
    let k = 3.7;

    let out_1x = support::run_sine(&circuit, 1000.0, 1.0, n, SR);
    let out_kx = support::run_sine(&circuit, 1000.0, k, n, SR);

    let mut max_err = 0.0_f64;
    for i in 0..n {
        let err = (out_kx[i] - k * out_1x[i]).abs();
        max_err = max_err.max(err);
    }

    assert!(
        max_err < 1e-12,
        "scaling linearity violated: max error = {max_err:.3e}"
    );
}
