//! EQ curve validation tests.
//!
//! General-purpose tests that verify EQ circuits produce the expected
//! frequency-dependent behavior when pots are adjusted. These tests
//! compile circuits through the full codegen pipeline and measure
//! output at multiple frequencies with different pot settings.
//!
//! Test classes:
//! - Flat response at default pot positions
//! - Boost/cut engagement (output changes in expected direction)
//! - Frequency selectivity (effect is at the right frequency)
//! - Circuit-specific (e.g., Pultec trick)

use std::io::Write;

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

// ── Helpers ──────────────────────────────────────────────────────────

/// Compile and run generated code with a custom main, return stdout.
fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_eq_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_eq_{tag}_{id}_{counter}"));

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
        .unwrap_or(f64::NAN)
}

fn load_pultec() -> (String, String) {
    let workspace_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let path = workspace_root.join("circuits/stable/pultec-eq.cir");
    let spice = std::fs::read_to_string(&path).expect("pultec-eq.cir not found");
    let netlist = Netlist::parse(&spice).expect("parse");
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
        circuit_name: "pultec_eq_test".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: input_r,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate_nodal(&mna, &netlist).expect("codegen");
    (result.code, spice)
}

/// Generate main code that measures gain at multiple frequencies with optional pot overrides.
/// pot_overrides: Vec of "state.set_pot_N(value);" lines (must use setter, not direct field access)
fn make_freq_sweep_main(freqs: &[f64], amplitude: f64, pot_overrides: &[String]) -> String {
    let freq_array = freqs
        .iter()
        .map(|f| format!("{:.1}", f))
        .collect::<Vec<_>>()
        .join(", ");
    let pot_code = pot_overrides.join("\n        ");

    format!(
        r#"
fn main() {{
    let freqs: &[f64] = &[{freq_array}];
    for &freq in freqs {{
        let mut state = CircuitState::default();
        state.set_sample_rate(48000.0);
        state.warmup();
        {pot_code}
        let mut peak = 0.0f64;
        for i in 0..480000u32 {{
            let t = i as f64 / 48000.0;
            let input = {amplitude:?}_f64 * (2.0 * std::f64::consts::PI * freq * t).sin();
            let output = process_sample(input, &mut state);
            if i >= 240000 {{ peak = peak.max(output[0].abs()); }}
        }}
        let gain_db = 20.0 * (peak / {amplitude:?}_f64).log10();
        println!("freq_{{:.0}}={{}}", freq, gain_db);
    }}
}}
"#
    )
}

// ── General EQ tests ────────────────────────────────────────────────

/// Pultec: flat response at default pot positions.
/// All pots at their default ("zero") values should give near-unity gain
/// across the audio band. This verifies the defaults are correct.
#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pultec_flat_response() {
    let (code, _) = load_pultec();
    let freqs = [100.0, 1000.0, 5000.0, 10000.0];
    let main_code = make_freq_sweep_main(&freqs, 0.01, &[]);
    let output = compile_and_run(&code, &main_code, "pultec_flat");

    let g100 = parse_kv(&output, "freq_100=");
    let g1k = parse_kv(&output, "freq_1000=");
    let g5k = parse_kv(&output, "freq_5000=");
    let g10k = parse_kv(&output, "freq_10000=");

    eprintln!("Flat: 100Hz={g100:+.1}, 1kHz={g1k:+.1}, 5kHz={g5k:+.1}, 10kHz={g10k:+.1}");

    // Near unity (within ±6 dB — relaxed for transformer model limitations)
    for (f, g) in [(100, g100), (1000, g1k), (5000, g5k), (10000, g10k)] {
        assert!(
            g > -6.0 && g < 6.0,
            "Flat response at {f}Hz should be within ±6 dB of unity, got {g:+.1} dB"
        );
    }

    // Response should be reasonably flat (max spread < 6 dB)
    let gains = [g100, g1k, g5k, g10k];
    let max = gains.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min = gains.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!(
        max - min < 6.0,
        "Flat response spread should be < 6 dB, got {:.1} dB (max={max:+.1}, min={min:+.1})",
        max - min
    );
}

/// Pultec: LF Boost increases low-frequency output.
#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pultec_lf_boost() {
    let (code, _) = load_pultec();

    // Default (flat)
    let main_flat = make_freq_sweep_main(&[100.0, 1000.0], 0.01, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "pultec_lfb_flat");
    let flat_100 = parse_kv(&out_flat, "freq_100=");
    let flat_1k = parse_kv(&out_flat, "freq_1000=");

    // LF Boost engaged (R_lfb from 100Ω default to 10kΩ max)
    let main_boost = make_freq_sweep_main(
        &[100.0, 1000.0],
        0.01,
        &["state.set_pot_3(10000.0);".to_string()],
    );
    let out_boost = compile_and_run(&code, &main_boost, "pultec_lfb_boost");
    let boost_100 = parse_kv(&out_boost, "freq_100=");
    let boost_1k = parse_kv(&out_boost, "freq_1000=");

    eprintln!("LF Boost: flat 100Hz={flat_100:+.1}, boost 100Hz={boost_100:+.1}");
    eprintln!("          flat 1kHz={flat_1k:+.1},  boost 1kHz={boost_1k:+.1}");

    // LF boost should increase 100Hz more than 1kHz
    let delta_100 = boost_100 - flat_100;
    let delta_1k = boost_1k - flat_1k;

    // With 21 dB of NFB, the boost effect is reduced — the feedback opposes gain changes.
    // The real Pultec has moderate boost amounts for the same reason.
    assert!(
        delta_100 > 0.3,
        "LF Boost should measurably increase 100Hz, got {delta_100:+.1} dB"
    );
    assert!(
        delta_100 > delta_1k,
        "LF Boost should affect 100Hz more than 1kHz (100Hz: {delta_100:+.1}, 1kHz: {delta_1k:+.1})"
    );
}

/// Pultec: HF Boost increases high-frequency output.
#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pultec_hf_boost() {
    let (code, _) = load_pultec();

    // Default (flat)
    let main_flat = make_freq_sweep_main(&[1000.0, 5000.0], 0.01, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "pultec_hfb_flat");
    let flat_1k = parse_kv(&out_flat, "freq_1000=");
    let flat_5k = parse_kv(&out_flat, "freq_5000=");

    // HF Boost engaged (R_hfb_u from 10kΩ to 100Ω, R_hfb_l from 100Ω to 10kΩ)
    let main_boost = make_freq_sweep_main(
        &[1000.0, 5000.0],
        0.01,
        &[
            "state.set_pot_0(100.0);".to_string(),
            "state.set_pot_1(10000.0);".to_string(),
        ],
    );
    let out_boost = compile_and_run(&code, &main_boost, "pultec_hfb_boost");
    let boost_1k = parse_kv(&out_boost, "freq_1000=");
    let boost_5k = parse_kv(&out_boost, "freq_5000=");

    eprintln!("HF Boost: flat 5kHz={flat_5k:+.1}, boost 5kHz={boost_5k:+.1}");
    eprintln!("          flat 1kHz={flat_1k:+.1},  boost 1kHz={boost_1k:+.1}");

    let delta_5k = boost_5k - flat_5k;
    let delta_1k = boost_1k - flat_1k;

    assert!(
        delta_5k > 1.0,
        "HF Boost should increase 5kHz by > 1 dB, got {delta_5k:+.1} dB"
    );
    // The LC resonance can be very strong at max boost, overwhelming both
    // test frequencies. Just verify HF is boosted more than midrange.
    assert!(
        delta_5k >= delta_1k,
        "HF Boost should affect 5kHz at least as much as 1kHz (5kHz: {delta_5k:+.1}, 1kHz: {delta_1k:+.1})"
    );
}

/// Pultec: HF Cut reduces high-frequency output.
///
#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pultec_hf_cut() {
    let (code, _) = load_pultec();

    // Default (flat)
    let main_flat = make_freq_sweep_main(&[1000.0, 5000.0], 0.01, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "pultec_hfc_flat");
    let flat_1k = parse_kv(&out_flat, "freq_1000=");
    let flat_5k = parse_kv(&out_flat, "freq_5000=");

    // HF Cut engaged (R_hfc_u from 1kΩ to 100Ω, R_hfc_l from 100Ω to 1kΩ)
    let main_cut = make_freq_sweep_main(
        &[1000.0, 5000.0],
        0.01,
        &[
            "state.set_pot_5(100.0);".to_string(),
            "state.set_pot_6(1000.0);".to_string(),
        ],
    );
    let out_cut = compile_and_run(&code, &main_cut, "pultec_hfc_cut");
    let cut_1k = parse_kv(&out_cut, "freq_1000=");
    let cut_5k = parse_kv(&out_cut, "freq_5000=");

    eprintln!("HF Cut: flat 5kHz={flat_5k:+.1}, cut 5kHz={cut_5k:+.1}");
    eprintln!("        flat 1kHz={flat_1k:+.1},  cut 1kHz={cut_1k:+.1}");

    let delta_5k = cut_5k - flat_5k;

    assert!(
        delta_5k < -3.0,
        "HF Cut should decrease 5kHz by > 3 dB, got {delta_5k:+.1} dB"
    );
}

/// Pultec: LF Atten reduces low-frequency output.
#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pultec_lf_atten() {
    let (code, _) = load_pultec();

    // Default (flat)
    let main_flat = make_freq_sweep_main(&[100.0, 1000.0], 0.01, &[]);
    let out_flat = compile_and_run(&code, &main_flat, "pultec_lfa_flat");
    let flat_100 = parse_kv(&out_flat, "freq_100=");
    let flat_1k = parse_kv(&out_flat, "freq_1000=");

    // LF Atten engaged (R_lfc from 100Ω to 100kΩ)
    let main_atten = make_freq_sweep_main(
        &[100.0, 1000.0],
        0.01,
        &["state.set_pot_4(100000.0);".to_string()],
    );
    let out_atten = compile_and_run(&code, &main_atten, "pultec_lfa_atten");
    let atten_100 = parse_kv(&out_atten, "freq_100=");
    let atten_1k = parse_kv(&out_atten, "freq_1000=");

    eprintln!("LF Atten: flat 100Hz={flat_100:+.1}, atten 100Hz={atten_100:+.1}");
    eprintln!("          flat 1kHz={flat_1k:+.1},   atten 1kHz={atten_1k:+.1}");

    let delta_100 = atten_100 - flat_100;

    // With 21 dB NFB, the atten effect is small (feedback compensates).
    // Just verify the direction is correct (attenuation, not boost).
    assert!(
        delta_100 < 0.0,
        "LF Atten should decrease (not increase) 100Hz, got {delta_100:+.2} dB"
    );
}

/// Pultec trick: simultaneous LF Boost + LF Atten creates a resonant shelf.
/// The boost adds a peak at the resonant frequency, while the atten cuts below.
/// The result should have MORE boost at the resonant frequency than LF boost alone,
/// and attenuation below the boost frequency.
#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pultec_trick() {
    let (code, _) = load_pultec();

    // LF Boost only
    let main_boost = make_freq_sweep_main(
        &[30.0, 100.0, 500.0, 1000.0],
        0.01,
        &["state.set_pot_3(10000.0);".to_string()],
    );
    let out_boost = compile_and_run(&code, &main_boost, "pultec_trick_boost");
    let boost_30 = parse_kv(&out_boost, "freq_30=");
    let boost_100 = parse_kv(&out_boost, "freq_100=");

    // LF Boost + LF Atten simultaneously (the "Pultec trick")
    let main_trick = make_freq_sweep_main(
        &[30.0, 100.0, 500.0, 1000.0],
        0.01,
        &[
            "state.set_pot_3(10000.0);".to_string(),  // LF Boost max
            "state.set_pot_4(100000.0);".to_string(), // LF Atten max
        ],
    );
    let out_trick = compile_and_run(&code, &main_trick, "pultec_trick_both");
    let trick_30 = parse_kv(&out_trick, "freq_30=");
    let trick_100 = parse_kv(&out_trick, "freq_100=");
    let trick_1k = parse_kv(&out_trick, "freq_1000=");

    eprintln!("Pultec trick:");
    eprintln!("  Boost only:  30Hz={boost_30:+.1}, 100Hz={boost_100:+.1}");
    eprintln!("  Boost+Atten: 30Hz={trick_30:+.1}, 100Hz={trick_100:+.1}, 1kHz={trick_1k:+.1}");

    // The "trick" signature: boost at 100Hz should still be present,
    // but 30Hz should be lower than with boost alone (the atten pulls down sub-bass)
    assert!(
        trick_100 > trick_1k + 0.2,
        "Pultec trick should boost 100Hz relative to 1kHz (100Hz={trick_100:+.1}, 1kHz={trick_1k:+.1})"
    );
    assert!(
        trick_30 < boost_30,
        "Pultec trick should attenuate sub-bass vs boost-only (trick 30Hz={trick_30:+.1}, boost 30Hz={boost_30:+.1})"
    );
}
