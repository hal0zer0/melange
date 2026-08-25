// ── melange perf-harness timing driver ──────────────────────────────────────
// Appended to a generated `circuit.rs` (which is standalone, no crate deps) and
// compiled with the SAME rustc flags the golden-harness uses. Measures pure
// `process_sample` throughput: the input signal is precomputed OUTSIDE the timed
// loop so only the solver's per-sample work is timed (no sin(), no I/O).
//
// This times the DEFAULT (noiseless) configuration — what a plugin ships with
// unless `--noise` is passed — so it does not touch the noise setters (which are
// only emitted when the circuit is compiled with `--noise`). To time the noise
// path instead, compile the circuit with `--noise full` and add
// `state.set_noise_enabled(true); state.set_seed(0xC0FFEE);` after construction.
//
// Output: one JSON line on stdout. All other prose goes to stderr.
fn main() {
    use std::hint::black_box;

    // Tunables via env so the harness stays reusable across circuits.
    let n: usize = std::env::var("PERF_SAMPLES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(2_000_000);
    let reps: usize = std::env::var("PERF_REPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(7);
    let warmup: usize = std::env::var("PERF_WARMUP")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(16_000);
    let label = std::env::var("PERF_LABEL").unwrap_or_else(|_| "circuit".to_string());

    let fs = SAMPLE_RATE;

    // Precompute a moderate-amplitude 220 Hz sine so NR actually iterates.
    // Done once, before timing — indexing this slice is the only per-sample
    // cost added on top of process_sample, and it is identical before/after.
    let two_pi = 2.0 * std::f64::consts::PI;
    let sig: Vec<f64> = (0..n)
        .map(|i| 0.3 * (two_pi * 220.0 * (i as f64) / fs).sin())
        .collect();

    let mut state = CircuitState::default();

    // Warmup: settle DC/smoothing, warm the branch predictor & i-cache.
    for i in 0..warmup {
        let x = 0.3 * (two_pi * 220.0 * (i as f64) / fs).sin();
        black_box(process_sample(black_box(x), &mut state));
    }

    let mut times: Vec<f64> = Vec::with_capacity(reps);
    for _ in 0..reps {
        let t0 = std::time::Instant::now();
        for &x in &sig {
            black_box(process_sample(black_box(x), &mut state));
        }
        times.push(t0.elapsed().as_secs_f64());
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let best = times[0];
    let median = times[times.len() / 2];
    let worst = times[times.len() - 1];
    let ns_min = best / n as f64 * 1e9;
    let ns_med = median / n as f64 * 1e9;
    let ns_max = worst / n as f64 * 1e9;
    // xRT: how many times faster than realtime (n output samples @ fs).
    let xrt_min = (n as f64 / fs) / best;

    println!(
        "{{\"label\":\"{label}\",\"fs\":{fs},\"os\":{OVERSAMPLING_FACTOR},\"samples\":{n},\"reps\":{reps},\
\"ns_per_sample_min\":{ns_min:.4},\"ns_per_sample_median\":{ns_med:.4},\"ns_per_sample_max\":{ns_max:.4},\
\"xrt_min\":{xrt_min:.2}}}"
    );
    eprintln!(
        "  {label:28}  {ns_med:8.2} ns/sample (median)  {ns_min:8.2} min  {xrt_min:8.1}x RT   [reps={reps}, n={n}]"
    );
}
