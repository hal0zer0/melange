//! FunkyInduct — Character Test Suite
//!
//! Property tests for the 16-band saturating-inductor passive LC EQ.
//! No real-world baseline exists (this circuit has never been built in
//! hardware), so these tests verify structural properties that MUST hold
//! if the circuit is behaving as designed:
//!
//! 1. Flat response when all bands inactive
//! 2. Each band boosts/cuts at its center frequency
//! 3. Boost/cut is frequency-selective (affects center more than neighbors)
//! 4. Inter-band coupling (multi-band boost plateaus, doesn't stack linearly)
//! 5. Adjacent boost+cut creates Pultec-style peak+dip
//! 6. Saturation is level-dependent (loud signal differs from quiet)
//! 7. Stability (no NaN/Inf at extreme settings)

use std::io::Write;

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

// ── Band center frequencies (must match gen_funkyinduct.py) ─────────

/// ERB-rate scale
fn erb_rate(f: f64) -> f64 {
    21.4 * (1.0 + 0.00437 * f).log10()
}

/// Inverse ERB-rate
fn inv_erb_rate(e: f64) -> f64 {
    (10.0_f64.powf(e / 21.4) - 1.0) / 0.00437
}

/// Compute center frequencies for N ERB-spaced bands from f_min to f_max.
fn band_freqs(n: usize, f_min: f64, f_max: f64) -> Vec<f64> {
    let e_min = erb_rate(f_min);
    let e_max = erb_rate(f_max);
    (0..n)
        .map(|i| {
            let e = e_min + i as f64 * (e_max - e_min) / (n - 1) as f64;
            inv_erb_rate(e)
        })
        .collect()
}

const N_BANDS: usize = 16;

// ── Helpers ─────────────────────────────────────────────────────────

fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_fi_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_fi_{tag}_{id}_{counter}"));

    let full_code = format!("{code}\n\n{main_code}\n");
    {
        let mut f = std::fs::File::create(&src_path).unwrap();
        f.write_all(full_code.as_bytes()).unwrap();
    }

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .expect("rustc");

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Compilation failed for {tag}:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run");
    let _ = std::fs::remove_file(&bin_path);
    if !run.status.success() {
        panic!(
            "Binary failed for {tag}:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }

    String::from_utf8_lossy(&run.stdout).to_string()
}

fn parse_kv(output: &str, key: &str) -> f64 {
    output
        .lines()
        .find(|l| l.starts_with(key))
        .and_then(|l| l[key.len()..].trim().parse().ok())
        .unwrap_or_else(|| panic!("Key '{key}' not found in output:\n{output}"))
}

fn load_funkyinduct() -> String {
    let workspace_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    // Circuit lives in the sibling melange-circuits repo
    let path = workspace_root
        .join("../melange-circuits/unstable/filters/funkyinduct.cir");
    let spice = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "Cannot load funkyinduct.cir at {}: {e}. \
             Run gen_funkyinduct.py in melange-circuits first.",
            path.display()
        )
    });

    let netlist = Netlist::parse(&spice).expect("parse funkyinduct.cir");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");

    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let output_node = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    let input_r = 600.0;
    mna.g[input_node][input_node] += 1.0 / input_r;

    let config = CodegenConfig {
        circuit_name: "funkyinduct_test".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: input_r,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate_nodal(&mna, &netlist).expect("codegen");
    result.code
}

/// Generate a main function that measures gain at multiple frequencies.
/// Each measurement: 10 second warmup of silence, then 200ms sine, measure
/// last 50ms RMS. pot_setup runs once after warmup.
fn make_sweep_main(freqs: &[f64], amplitude: f64, pot_setup: &[String]) -> String {
    let freq_array = freqs
        .iter()
        .map(|f| format!("{f:.1}"))
        .collect::<Vec<_>>()
        .join(", ");
    let pot_code = pot_setup.join("\n        ");

    format!(
        r#"
fn main() {{
    let freqs: &[f64] = &[{freq_array}];
    for &freq in freqs {{
        let mut state = CircuitState::default();
        state.set_sample_rate(48000.0);
        state.warmup();
        {pot_code}
        // Settle after pot changes
        for _ in 0..48000u32 {{
            process_sample(0.0, &mut state);
        }}
        let mut peak = 0.0f64;
        // 200ms of signal, measure last 50ms
        for i in 0..9600u32 {{
            let t = i as f64 / 48000.0;
            let input = {amplitude:?}_f64 * (2.0 * std::f64::consts::PI * freq * t).sin();
            let output = process_sample(input, &mut state);
            if i >= 7200 {{ peak = peak.max(output[0].abs()); }}
        }}
        let gain_db = if peak > 1e-15 {{
            20.0 * (peak / {amplitude:?}_f64).log10()
        }} else {{
            -120.0
        }};
        println!("freq_{{:.0}}={{}}", freq, gain_db);
    }}
}}
"#
    )
}

// ── 1. Flat response ────────────────────────────────────────────────

#[test]
fn flat_response_is_flat() {
    let code = load_funkyinduct();
    let freqs = [100.0, 300.0, 1000.0, 3000.0, 10000.0];
    let main_code = make_sweep_main(&freqs, 0.05, &[]);
    let output = compile_and_run(&code, &main_code, "flat");

    let gains: Vec<f64> = freqs
        .iter()
        .map(|f| parse_kv(&output, &format!("freq_{:.0}=", f)))
        .collect();

    eprintln!("Flat response:");
    for (f, g) in freqs.iter().zip(&gains) {
        eprintln!("  {f:.0} Hz: {g:+.1} dB");
    }

    // Flat response should be within ±3 dB across the band
    let max = gains.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min = gains.iter().cloned().fold(f64::INFINITY, f64::min);
    let spread = max - min;
    assert!(
        spread < 3.0,
        "Flat response spread should be < 3 dB, got {spread:.1} dB"
    );

    // Overall gain should be negative (passive network attenuates)
    let avg = gains.iter().sum::<f64>() / gains.len() as f64;
    assert!(
        avg < 0.0,
        "Passive EQ should attenuate in flat mode, got {avg:+.1} dB average"
    );
}

// ── 2. Each band boosts at its center frequency ─────────────────────

#[test]
fn each_band_boosts_at_center() {
    let code = load_funkyinduct();
    let centers = band_freqs(N_BANDS, 20.0, 20000.0);

    // Skip band 0 (20 Hz — too close to coupling cap rolloff) and
    // band 15 (20 kHz — near Nyquist at 48k). Test bands 1–14.
    let test_bands: Vec<usize> = (1..15).collect();

    // Build a main that measures flat and boosted for each test band
    let mut all_freqs = Vec::new();
    let mut pot_lines = Vec::new();
    for &b in &test_bands {
        all_freqs.push(centers[b]);
    }

    // First run: flat baseline
    let main_flat = make_sweep_main(&all_freqs, 0.05, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "boost_flat");

    // One run per band with boost active
    for &b in &test_bands {
        let freq = centers[b];
        pot_lines.clear();
        pot_lines.push(format!("state.set_pot_{}(100.0);", b * 2)); // Rb_N = boost

        let main_boost = make_sweep_main(&[freq], 0.05, &pot_lines.iter().map(|s| s.clone()).collect::<Vec<_>>());
        let out_boost = compile_and_run(&code, &main_boost, &format!("boost_b{b}"));

        let flat_gain = parse_kv(&out_flat, &format!("freq_{:.0}=", freq));
        let boost_gain = parse_kv(&out_boost, &format!("freq_{:.0}=", freq));
        let delta = boost_gain - flat_gain;

        eprintln!("Band {b} ({freq:.0} Hz): flat={flat_gain:+.1}, boost={boost_gain:+.1}, delta={delta:+.1}");

        assert!(
            delta > 0.5,
            "Band {b} ({freq:.0} Hz) boost should increase gain by > 0.5 dB, got {delta:+.1} dB"
        );
    }
}

// ── 3. Each band cuts at its center frequency ───────────────────────

#[test]
fn each_band_cuts_at_center() {
    let code = load_funkyinduct();
    let centers = band_freqs(N_BANDS, 20.0, 20000.0);
    let test_bands: Vec<usize> = (1..15).collect();

    let all_freqs: Vec<f64> = test_bands.iter().map(|&b| centers[b]).collect();
    let main_flat = make_sweep_main(&all_freqs, 0.05, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "cut_flat");

    for &b in &test_bands {
        let freq = centers[b];
        let pot_lines = vec![format!("state.set_pot_{}(100.0);", b * 2 + 1)]; // Rc_N = cut

        let main_cut = make_sweep_main(&[freq], 0.05, &pot_lines);
        let out_cut = compile_and_run(&code, &main_cut, &format!("cut_b{b}"));

        let flat_gain = parse_kv(&out_flat, &format!("freq_{:.0}=", freq));
        let cut_gain = parse_kv(&out_cut, &format!("freq_{:.0}=", freq));
        let delta = cut_gain - flat_gain;

        eprintln!("Band {b} ({freq:.0} Hz): flat={flat_gain:+.1}, cut={cut_gain:+.1}, delta={delta:+.1}");

        assert!(
            delta < -1.0,
            "Band {b} ({freq:.0} Hz) cut should decrease gain by > 1 dB, got {delta:+.1} dB"
        );
    }
}

// ── 4. Boost is frequency-selective ─────────────────────────────────

#[test]
fn boost_is_frequency_selective() {
    let code = load_funkyinduct();
    let centers = band_freqs(N_BANDS, 20.0, 20000.0);

    // Boost band 7 (~1.7 kHz), measure at band 7 vs band 3 (~370 Hz)
    let b = 7;
    let f_center = centers[b];
    let f_far = centers[3];

    let main_flat = make_sweep_main(&[f_center, f_far], 0.05, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "selective_flat");

    let pot_lines = vec![format!("state.set_pot_{}(100.0);", b * 2)];
    let main_boost = make_sweep_main(&[f_center, f_far], 0.05, &pot_lines);
    let out_boost = compile_and_run(&code, &main_boost, "selective_boost");

    let delta_center = parse_kv(&out_boost, &format!("freq_{:.0}=", f_center))
        - parse_kv(&out_flat, &format!("freq_{:.0}=", f_center));
    let delta_far = parse_kv(&out_boost, &format!("freq_{:.0}=", f_far))
        - parse_kv(&out_flat, &format!("freq_{:.0}=", f_far));

    eprintln!("Band {b} boost: center ({f_center:.0} Hz) delta={delta_center:+.1}, far ({f_far:.0} Hz) delta={delta_far:+.1}");

    assert!(
        delta_center > delta_far,
        "Boost should affect center ({f_center:.0} Hz: {delta_center:+.1}) more than far ({f_far:.0} Hz: {delta_far:+.1})"
    );
}

// ── 5. Inter-band coupling: multi-band boost plateaus ───────────────

#[test]
fn multi_band_boost_shows_coupling() {
    let code = load_funkyinduct();
    let centers = band_freqs(N_BANDS, 20.0, 20000.0);

    // Measure at band 7 center frequency
    let f = centers[7];

    // Flat baseline
    let main_flat = make_sweep_main(&[f], 0.05, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "coupling_flat");
    let flat_gain = parse_kv(&out_flat, &format!("freq_{:.0}=", f));

    // Single band boost (band 7)
    let main_single = make_sweep_main(
        &[f], 0.05,
        &[format!("state.set_pot_14(100.0);")], // Rb_7
    );
    let out_single = compile_and_run(&code, &main_single, "coupling_single");
    let single_delta = parse_kv(&out_single, &format!("freq_{:.0}=", f)) - flat_gain;

    // Three adjacent bands boost (6, 7, 8)
    let main_triple = make_sweep_main(
        &[f], 0.05,
        &[
            format!("state.set_pot_12(100.0);"), // Rb_6
            format!("state.set_pot_14(100.0);"), // Rb_7
            format!("state.set_pot_16(100.0);"), // Rb_8
        ],
    );
    let out_triple = compile_and_run(&code, &main_triple, "coupling_triple");
    let triple_delta = parse_kv(&out_triple, &format!("freq_{:.0}=", f)) - flat_gain;

    eprintln!("Coupling test at {f:.0} Hz:");
    eprintln!("  1 band boost: {single_delta:+.1} dB");
    eprintln!("  3 band boost: {triple_delta:+.1} dB");
    eprintln!("  Ratio: {:.2}x (linear would be 3.0x)", triple_delta / single_delta);

    // 3 bands should boost MORE than 1, but LESS than 3× (coupling plateau)
    assert!(
        triple_delta > single_delta,
        "3-band boost ({triple_delta:+.1}) should exceed single-band ({single_delta:+.1})"
    );
    assert!(
        triple_delta < single_delta * 2.5,
        "3-band boost ({triple_delta:+.1}) should be sub-linear vs 3× single ({:+.1}) — coupling should limit stacking",
        single_delta * 3.0
    );
}

// ── 6. Adjacent boost+cut: Pultec-style peak+dip ────────────────────

#[test]
fn adjacent_boost_cut_creates_peak_and_dip() {
    let code = load_funkyinduct();
    let centers = band_freqs(N_BANDS, 20.0, 20000.0);

    // Boost band 7 (~1.7k), cut band 8 (~2.4k)
    let f_boost = centers[7];
    let f_cut = centers[8];
    let f_below = centers[5]; // ~849 Hz, well below the action

    let main_flat = make_sweep_main(&[f_below, f_boost, f_cut], 0.05, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "pultec_flat");

    let pot_lines = vec![
        format!("state.set_pot_14(100.0);"), // Rb_7 = boost
        format!("state.set_pot_17(100.0);"), // Rc_8 = cut
    ];
    let main_pultec = make_sweep_main(&[f_below, f_boost, f_cut], 0.05, &pot_lines);
    let out_pultec = compile_and_run(&code, &main_pultec, "pultec_combo");

    let delta_below = parse_kv(&out_pultec, &format!("freq_{:.0}=", f_below))
        - parse_kv(&out_flat, &format!("freq_{:.0}=", f_below));
    let delta_boost = parse_kv(&out_pultec, &format!("freq_{:.0}=", f_boost))
        - parse_kv(&out_flat, &format!("freq_{:.0}=", f_boost));
    let delta_cut = parse_kv(&out_pultec, &format!("freq_{:.0}=", f_cut))
        - parse_kv(&out_flat, &format!("freq_{:.0}=", f_cut));

    eprintln!("Pultec interaction (boost {f_boost:.0} Hz + cut {f_cut:.0} Hz):");
    eprintln!("  Below ({f_below:.0} Hz): {delta_below:+.1} dB");
    eprintln!("  Boost ({f_boost:.0} Hz): {delta_boost:+.1} dB");
    eprintln!("  Cut   ({f_cut:.0} Hz):   {delta_cut:+.1} dB");

    // The signature: boost region goes up, cut region goes down
    assert!(
        delta_boost > delta_cut + 1.0,
        "Boost freq should be higher than cut freq (boost={delta_boost:+.1}, cut={delta_cut:+.1})"
    );
    assert!(
        delta_cut < -1.0,
        "Cut region should show attenuation, got {delta_cut:+.1} dB"
    );
}

// ── 7. Level-dependent saturation ───────────────────────────────────

#[test]
fn saturation_changes_response_at_high_level() {
    let code = load_funkyinduct();
    let centers = band_freqs(N_BANDS, 20.0, 20000.0);

    // Boost band 3 (bass, ~370 Hz) where saturation should be strongest
    let f = centers[3];
    let pot_lines = vec![format!("state.set_pot_6(100.0);")]; // Rb_3

    // Quiet signal (well below ISAT threshold)
    let main_quiet = make_sweep_main(&[f], 0.01, &pot_lines);
    let out_quiet = compile_and_run(&code, &main_quiet, "sat_quiet");
    let gain_quiet = parse_kv(&out_quiet, &format!("freq_{:.0}=", f));

    // Loud signal (should push inductor current past ISAT)
    let main_loud = make_sweep_main(&[f], 0.5, &pot_lines);
    let out_loud = compile_and_run(&code, &main_loud, "sat_loud");
    let gain_loud = parse_kv(&out_loud, &format!("freq_{:.0}=", f));

    let difference = (gain_loud - gain_quiet).abs();

    eprintln!("Saturation test at {f:.0} Hz (band 3, bass):");
    eprintln!("  Quiet (0.01V): {gain_quiet:+.1} dB");
    eprintln!("  Loud  (0.5V):  {gain_loud:+.1} dB");
    eprintln!("  Difference: {difference:.1} dB");

    // At loud levels, the saturating inductors should change the response
    // measurably. Even 0.1 dB difference confirms level-dependent behavior.
    // If ISAT is not active (linear inductors), the difference would be
    // purely from tube nonlinearity — which is small at these levels.
    assert!(
        difference > 0.05,
        "Response should differ between quiet and loud due to inductor saturation, \
         got only {difference:.2} dB difference"
    );
}

// ── 8. Stability ────────────────────────────────────────────────────

#[test]
fn stable_with_all_bands_active() {
    let code = load_funkyinduct();

    // Crank all boost and cut pots to minimum simultaneously
    let mut pot_lines = Vec::new();
    for i in 0..N_BANDS {
        pot_lines.push(format!("state.set_pot_{}(100.0);", i * 2));     // Rb
        pot_lines.push(format!("state.set_pot_{}(100.0);", i * 2 + 1)); // Rc
    }

    let main_code = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    state.set_sample_rate(48000.0);
    state.warmup();
    {pot_code}
    // Settle
    for _ in 0..48000u32 {{
        process_sample(0.0, &mut state);
    }}
    let mut peak = 0.0f64;
    let mut has_nan = false;
    let mut has_inf = false;
    // 200ms of signal
    for i in 0..9600u32 {{
        let t = i as f64 / 48000.0;
        let input = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let output = process_sample(input, &mut state);
        let v = output[0];
        if v.is_nan() {{ has_nan = true; }}
        if v.is_infinite() {{ has_inf = true; }}
        peak = peak.max(v.abs());
    }}
    println!("peak={{}}", peak);
    println!("nan={{}}", has_nan);
    println!("inf={{}}", has_inf);
}}
"#,
        pot_code = pot_lines.join("\n    ")
    );

    let output = compile_and_run(&code, &main_code, "stability");

    let peak = parse_kv(&output, "peak=");
    let has_nan = output.lines().any(|l| l == "nan=true");
    let has_inf = output.lines().any(|l| l == "inf=true");

    eprintln!("Stability (all 32 pots at extreme): peak={peak:.4}, nan={has_nan}, inf={has_inf}");

    assert!(!has_nan, "Output contains NaN");
    assert!(!has_inf, "Output contains Inf");
    assert!(
        peak < 50.0,
        "Peak output should be bounded, got {peak:.1}"
    );
}

#[test]
fn silence_in_silence_out() {
    let code = load_funkyinduct();

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    state.set_sample_rate(48000.0);
    state.warmup();
    // Process silence for 1 second
    let mut peak = 0.0f64;
    for _ in 0..48000u32 {
        let output = process_sample(0.0, &mut state);
        peak = peak.max(output[0].abs());
    }
    println!("peak={}", peak);
}
"#;

    let output = compile_and_run(&code, main_code, "silence");
    let peak = parse_kv(&output, "peak=");

    eprintln!("Silence test: peak={peak:.6}");

    assert!(
        peak < 0.01,
        "Silence input should produce near-silence output, got peak={peak:.4}"
    );
}
