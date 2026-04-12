//! Neve 1073 EQ section validation tests.
//!
//! These tests verify the EQ circuit netlists through the full codegen pipeline:
//! parse → MNA → codegen → compile → run → measure frequency response.
//!
//! Test coverage:
//! - HPF (B182): rolloff slope, switch positions change cutoff
//! - LF/HF bridge (B205): opposing coupling passes signal, shelf shape, boost vs flat
//! - Coupled inductor switching: compiles, positions produce different responses
//! - Full EQ chain: HPF + bridge + amp combined

use std::io::Write;

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

// ── Helpers ──────────────────────────────────────────────────────────

/// Compile and run generated code with a custom main, return stdout.
fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_neve_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_neve_{tag}_{id}_{counter}"));

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
        .unwrap_or_else(|| panic!("Key '{}' not found in output:\n{}", key, output))
}

/// Generate codegen result from a SPICE netlist string using auto-routing
/// (same path as `melange compile`).
fn codegen_from_spice(spice: &str, input_r: f64) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
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
    mna.g[input_node][input_node] += 1.0 / input_r;

    let config = CodegenConfig {
        circuit_name: "neve_1073_eq_test".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: input_r,
        ..CodegenConfig::default()
    };

    // Build DK kernel (uses augmented MNA for circuits with inductors)
    let kernel = if mna.has_inductors() {
        DkKernel::from_mna_augmented(&mna, 48000.0).expect("dk kernel")
    } else {
        DkKernel::from_mna(&mna, 48000.0).expect("dk kernel")
    };

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen");
    result.code
}

/// Generate main() that measures peak amplitude at multiple frequencies.
/// switch_setup: optional lines like "state.set_switch_0(1);" run after warmup.
/// Uses 2 seconds settle + 1 second measure (sufficient for L/R < 10ms time constants).
fn make_freq_main(freqs: &[f64], amplitude: f64, setup: &[&str]) -> String {
    let freq_array = freqs
        .iter()
        .map(|f| format!("{:.1}", f))
        .collect::<Vec<_>>()
        .join(", ");
    let setup_code = setup.join("\n        ");

    format!(
        r#"
fn main() {{
    let freqs: &[f64] = &[{freq_array}];
    let sr = 48000.0_f64;
    let settle = (5.0 * sr) as u32;
    let measure = (2.0 * sr) as u32;
    for &freq in freqs {{
        let mut state = CircuitState::default();
        state.set_sample_rate(sr);
        {setup_code}
        // Settle
        for i in 0..settle {{
            let t = i as f64 / sr;
            let input = {amplitude:e}_f64 * (2.0 * std::f64::consts::PI * freq * t).sin();
            let _ = process_sample(input, &mut state);
        }}
        // Measure peak
        let mut peak = 0.0_f64;
        for i in 0..measure {{
            let t = (settle + i) as f64 / sr;
            let input = {amplitude:e}_f64 * (2.0 * std::f64::consts::PI * freq * t).sin();
            let output = process_sample(input, &mut state);
            peak = peak.max(output[0].abs());
        }}
        let gain_db = if peak > 1e-20 {{
            20.0 * (peak / {amplitude:e}_f64).log10()
        }} else {{
            -999.0
        }};
        println!("freq_{{:.0}}={{:.2}}", freq, gain_db);
    }}
}}
"#
    )
}

// ── HPF Tests ───────────────────────────────────────────────────────

/// HPF only (B182 board) — standalone netlist for testing.
fn hpf_spice() -> &'static str {
    "\
HPF B182
R_source in hp_in 600
C_hp1 hp_in hp_mid 220N
R_hdamp hp_mid 0 10K
R_hdcr hp_mid hp_l 490
L_hp hp_l 0 3
C_hp2 hp_mid out 220N
R_hterm out 0 5.1K
.switch C_hp1,C_hp2,L_hp,R_hdcr 1U/1U/10/1000 470N/470N/7/800 220N/220N/3/490 100N/100N/1.3/310
.END
"
}

/// HPF compiles through codegen and produces highpass rolloff.
/// At default position (3H, 220nF = ~110 Hz cutoff):
/// - 1 kHz should pass (near 0 dB attenuation)
/// - 30 Hz should be heavily attenuated (>20 dB below 1 kHz)
#[test]
fn test_neve_1073_hpf_rolloff() {
    let code = codegen_from_spice(hpf_spice(), 600.0);
    let main_code = make_freq_main(&[30.0, 100.0, 1000.0, 5000.0], 0.01, &[]);
    let output = compile_and_run(&code, &main_code, "hpf_rolloff");

    let g30 = parse_kv(&output, "freq_30=");
    let g100 = parse_kv(&output, "freq_100=");
    let g1k = parse_kv(&output, "freq_1000=");
    let g5k = parse_kv(&output, "freq_5000=");

    eprintln!("HPF rolloff: 30Hz={g30:+.1}, 100Hz={g100:+.1}, 1kHz={g1k:+.1}, 5kHz={g5k:+.1}");

    // 1 kHz and 5 kHz should be in the passband (similar gain)
    assert!(
        (g1k - g5k).abs() < 6.0,
        "Passband should be flat: 1kHz={g1k:+.1}, 5kHz={g5k:+.1}"
    );

    // 30 Hz should be significantly below passband (18 dB/oct rolloff)
    assert!(
        g1k - g30 > 15.0,
        "30 Hz should be >15 dB below 1 kHz: 1kHz={g1k:+.1}, 30Hz={g30:+.1}"
    );

    // 100 Hz should be somewhat attenuated (near cutoff)
    assert!(
        g1k - g100 > 2.0,
        "100 Hz should be at least 2 dB below 1 kHz (near cutoff): 1kHz={g1k:+.1}, 100Hz={g100:+.1}"
    );
}

/// HPF switch changes the cutoff frequency.
/// Position 0 (1µF/10H) = ~50 Hz cutoff → passes 100 Hz
/// Position 3 (100nF/1.3H) = ~300 Hz cutoff → attenuates 100 Hz
#[test]
fn test_neve_1073_hpf_switch_changes_cutoff() {
    let code = codegen_from_spice(hpf_spice(), 600.0);

    // Position 0: low cutoff (~50 Hz)
    let main_pos0 = make_freq_main(&[100.0, 1000.0], 0.01, &["state.set_switch_0(0);"]);
    let out0 = compile_and_run(&code, &main_pos0, "hpf_sw0");
    let g100_pos0 = parse_kv(&out0, "freq_100=");
    let g1k_pos0 = parse_kv(&out0, "freq_1000=");

    // Position 3: high cutoff (~300 Hz)
    let main_pos3 = make_freq_main(&[100.0, 1000.0], 0.01, &["state.set_switch_0(3);"]);
    let out3 = compile_and_run(&code, &main_pos3, "hpf_sw3");
    let g100_pos3 = parse_kv(&out3, "freq_100=");
    let g1k_pos3 = parse_kv(&out3, "freq_1000=");

    eprintln!("HPF pos0: 100Hz={g100_pos0:+.1}, 1kHz={g1k_pos0:+.1}");
    eprintln!("HPF pos3: 100Hz={g100_pos3:+.1}, 1kHz={g1k_pos3:+.1}");

    // At position 0 (50 Hz cutoff), 100 Hz should be in the passband
    let atten_pos0 = g1k_pos0 - g100_pos0;

    // At position 3 (300 Hz cutoff), 100 Hz should be heavily attenuated
    let atten_pos3 = g1k_pos3 - g100_pos3;

    assert!(
        atten_pos3 > atten_pos0 + 5.0,
        "Position 3 should attenuate 100 Hz much more than position 0: pos0={atten_pos0:.1} dB, pos3={atten_pos3:.1} dB"
    );
}

// ── B205 LF Bridge Tests ────────────────────────────────────────────

/// B205 bridge with opposing coupling + amp — full boost.
fn b205_boost_spice() -> &'static str {
    "\
B205 Bridge Boost
R_src in bridge_in 600
R39 bridge_in 0 39K
R_lf620 bridge_in node_l 620
R_rf620 bridge_in node_r 620
C_hfp node_l hf_p 22N
C_hfn node_r hf_n 22N
C_hfj node_l node_r 15N
R_hf_pa hf_p hf_wiper 5K
R_hf_pb hf_wiper hf_n 5K
R37 hf_wiper 0 1.2K
R_ldcr_l node_l lfl_ind 245
L_lf_l lfl_ind left_bot 3
R_ldcr_r node_r lfr_ind 245
L_lf_r right_bot lfr_ind 3
K1 L_lf_l L_lf_r 0.99
R_12k left_bot center_bot 12K
C_10n center_bot right_bot 10N
R45 left_bot 0 12K
C_lf_l left_bot pot_l 47N
C_lf_r right_bot pot_r 47N
R_lfpot_l pot_l 0 1
R_lfpot_r pot_r 0 50K
.model AMP2 OA(AOL=5000 ROUT=100)
U1 eq_ref amp_inv eq_out AMP2
R_eqref eq_ref 0 1K
R15 right_bot amp_inv 27K
R17 eq_out amp_inv 180K
R57 eq_out out 120
R_load out 0 5.1K
.END
"
}

/// B205 bridge with AIDING coupling (the WRONG direction) — should show
/// massive HF attenuation because common-mode sees L+M ≈ 6H.
fn b205_aiding_spice() -> &'static str {
    "\
B205 Bridge Aiding
R_src in bridge_in 600
R39 bridge_in 0 39K
R_lf620 bridge_in node_l 620
R_rf620 bridge_in node_r 620
C_hfp node_l hf_p 22N
C_hfn node_r hf_n 22N
C_hfj node_l node_r 15N
R_hf_pa hf_p hf_wiper 5K
R_hf_pb hf_wiper hf_n 5K
R37 hf_wiper 0 1.2K
R_ldcr_l node_l lfl_ind 245
L_lf_l lfl_ind left_bot 3
R_ldcr_r node_r lfr_ind 245
L_lf_r lfr_ind right_bot 3
K1 L_lf_l L_lf_r 0.99
R_12k left_bot center_bot 12K
C_10n center_bot right_bot 10N
R45 left_bot 0 12K
C_lf_l left_bot pot_l 47N
C_lf_r right_bot pot_r 47N
R_lfpot_l pot_l 0 1
R_lfpot_r pot_r 0 50K
.model AMP2 OA(AOL=5000 ROUT=100)
U1 eq_ref amp_inv eq_out AMP2
R_eqref eq_ref 0 1K
R15 right_bot amp_inv 27K
R17 eq_out amp_inv 180K
R57 eq_out out 120
R_load out 0 5.1K
.END
"
}

/// B205 bridge with opposing coupling compiles and produces signal at 1 kHz.
/// This is the critical test: opposing coupling passes signal through the bridge,
/// while aiding coupling would kill the passband.
#[test]
fn test_neve_1073_b205_opposing_coupling_passes_signal() {
    let code_opposing = codegen_from_spice(b205_boost_spice(), 600.0);
    let main_code = make_freq_main(&[1000.0], 0.01, &[]);
    let output = compile_and_run(&code_opposing, &main_code, "b205_opposing");
    let g1k = parse_kv(&output, "freq_1000=");

    eprintln!("B205 opposing coupling at 1kHz: {g1k:+.1} dB");

    // With opposing coupling, 1 kHz should pass with reasonable gain
    // (amp provides ~16 dB, bridge passes signal)
    assert!(
        g1k > 5.0,
        "Opposing coupling should pass signal at 1kHz: got {g1k:+.1} dB, expected >5 dB"
    );
}

/// B205 aiding coupling kills the passband — signal at 1 kHz is much lower.
/// This verifies that the opposing coupling fix is essential.
#[test]
fn test_neve_1073_b205_aiding_coupling_kills_passband() {
    let code_aiding = codegen_from_spice(b205_aiding_spice(), 600.0);
    let main_code = make_freq_main(&[1000.0], 0.01, &[]);
    let output = compile_and_run(&code_aiding, &main_code, "b205_aiding");
    let g1k = parse_kv(&output, "freq_1000=");

    eprintln!("B205 aiding coupling at 1kHz: {g1k:+.1} dB");

    // With aiding coupling, common-mode sees L+M ≈ 6H → massive attenuation
    assert!(
        g1k < 0.0,
        "Aiding coupling should heavily attenuate 1kHz: got {g1k:+.1} dB, expected <0 dB"
    );
}

/// B205 bridge at full boost: LF shelf shape.
/// The response should show a shelf: passband near 1-5 kHz is higher than at 50 Hz
/// (because the HPF is not present, but the bridge still has frequency-dependent behavior).
#[test]
fn test_neve_1073_b205_shelf_shape() {
    let code = codegen_from_spice(b205_boost_spice(), 600.0);
    let main_code = make_freq_main(&[50.0, 200.0, 1000.0, 5000.0], 0.01, &[]);
    let output = compile_and_run(&code, &main_code, "b205_shelf");

    let g50 = parse_kv(&output, "freq_50=");
    let g200 = parse_kv(&output, "freq_200=");
    let g1k = parse_kv(&output, "freq_1000=");
    let g5k = parse_kv(&output, "freq_5000=");

    eprintln!("B205 shelf: 50Hz={g50:+.1}, 200Hz={g200:+.1}, 1kHz={g1k:+.1}, 5kHz={g5k:+.1}");

    // All outputs should be finite and produce signal
    for (f, g) in [(50, g50), (200, g200), (1000, g1k), (5000, g5k)] {
        assert!(g.is_finite(), "Output at {f}Hz should be finite: {g}");
        assert!(g > -30.0, "Output at {f}Hz should be >-30 dB: {g:+.1}");
    }

    // The 1-5 kHz region (shelf plateau) should be higher than 50 Hz
    // (bridge rolls off at LF even with boost, since the pot mechanism is frequency-dependent)
    assert!(
        g1k > g50,
        "1kHz should be higher than 50Hz: 1kHz={g1k:+.1}, 50Hz={g50:+.1}"
    );
}

// ── Coupled Inductor Switch Tests ───────────────────────────────────

/// Coupled inductors in a .switch directive — compiles through codegen.
/// This tests the feature we just implemented.
fn coupled_switch_spice() -> &'static str {
    "\
Coupled Switch Test
R_src in bridge_in 600
R39 bridge_in 0 39K
R_lf620 bridge_in node_l 620
R_rf620 bridge_in node_r 620
R_ldcr_l node_l lfl_ind 245
L_lf_l lfl_ind left_bot 3
R_ldcr_r node_r lfr_ind 245
L_lf_r right_bot lfr_ind 3
K1 L_lf_l L_lf_r 0.99
R_12k left_bot center_bot 12K
C_10n center_bot right_bot 10N
R45 left_bot 0 12K
C_lf_l left_bot pot_l 47N
C_lf_r right_bot pot_r 47N
R_lfpot_l pot_l 0 1
R_lfpot_r pot_r 0 50K
R_load right_bot out 1
R_out out 0 100K
.switch L_lf_l,L_lf_r,R_ldcr_l,R_ldcr_r,C_lf_l,C_lf_r 3/3/245/245/47N/47N 1.3/1.3/155/155/15N/15N
.END
"
}

/// Circuit with coupled inductors in .switch compiles through nodal codegen.
#[test]
fn test_neve_1073_coupled_switch_codegen_compiles() {
    let code = codegen_from_spice(coupled_switch_spice(), 600.0);
    let main_code = make_freq_main(&[200.0], 0.01, &[]);
    let output = compile_and_run(&code, &main_code, "coupled_sw_compile");
    let g200 = parse_kv(&output, "freq_200=");

    eprintln!("Coupled switch default position at 200Hz: {g200:+.1} dB");
    assert!(g200.is_finite(), "Output should be finite: {g200}");
}

/// Switching between positions produces different frequency responses.
/// Position 0: 3H/47nF (lower frequency)
/// Position 1: 1.3H/15nF (higher frequency)
#[test]
fn test_neve_1073_coupled_switch_positions_differ() {
    let code = codegen_from_spice(coupled_switch_spice(), 600.0);

    // Position 0 (3H, 47nF)
    let main0 = make_freq_main(&[100.0, 500.0], 0.01, &["state.set_switch_0(0);"]);
    let out0 = compile_and_run(&code, &main0, "coupled_sw_pos0");
    let g100_p0 = parse_kv(&out0, "freq_100=");
    let g500_p0 = parse_kv(&out0, "freq_500=");

    // Position 1 (1.3H, 15nF)
    let main1 = make_freq_main(&[100.0, 500.0], 0.01, &["state.set_switch_0(1);"]);
    let out1 = compile_and_run(&code, &main1, "coupled_sw_pos1");
    let g100_p1 = parse_kv(&out1, "freq_100=");
    let g500_p1 = parse_kv(&out1, "freq_500=");

    eprintln!("Pos 0 (3H/47nF):   100Hz={g100_p0:+.1}, 500Hz={g500_p0:+.1}");
    eprintln!("Pos 1 (1.3H/15nF): 100Hz={g100_p1:+.1}, 500Hz={g500_p1:+.1}");

    // Both positions should produce finite output
    assert!(g100_p0.is_finite() && g500_p0.is_finite(), "Position 0 should produce finite output");
    assert!(g100_p1.is_finite() && g500_p1.is_finite(), "Position 1 should produce finite output");

    // The responses should differ — different L and C values change the frequency behavior
    let diff_100 = (g100_p0 - g100_p1).abs();
    let diff_500 = (g500_p0 - g500_p1).abs();
    assert!(
        diff_100 > 0.1 || diff_500 > 0.1,
        "Switch positions should produce different responses: \
         diff@100Hz={diff_100:.2} dB, diff@500Hz={diff_500:.2} dB"
    );
}

// ── Full EQ Chain Test ──────────────────────────────────────────────

/// Full 1073 EQ circuit (HPF + B205 bridge + BA284 §2 amp) compiles and runs.
/// Loaded from the actual circuit file.
#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_neve_1073_full_eq_codegen_compiles_and_runs() {
    let workspace_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let path = workspace_root.join("circuits/unstable/neve-1073-eq.cir");
    let spice = std::fs::read_to_string(&path).expect("neve-1073-eq.cir not found");
    let code = codegen_from_spice(&spice, 600.0);

    let main_code = make_freq_main(&[200.0, 1000.0, 5000.0], 0.01, &[]);
    let output = compile_and_run(&code, &main_code, "full_eq");

    let g200 = parse_kv(&output, "freq_200=");
    let g1k = parse_kv(&output, "freq_1000=");
    let g5k = parse_kv(&output, "freq_5000=");

    eprintln!("Full EQ: 200Hz={g200:+.1}, 1kHz={g1k:+.1}, 5kHz={g5k:+.1}");

    // All outputs should be finite and produce signal
    assert!(g200.is_finite(), "200Hz should be finite: {g200}");
    assert!(g1k.is_finite(), "1kHz should be finite: {g1k}");
    assert!(g5k.is_finite(), "5kHz should be finite: {g5k}");

    // The amp should produce positive gain (>0 dB) in the passband
    assert!(
        g1k > 0.0,
        "1kHz should have positive gain from the amp: {g1k:+.1} dB"
    );

    // 5 kHz should be similar to 1 kHz (within the shelf plateau)
    assert!(
        (g1k - g5k).abs() < 8.0,
        "1kHz and 5kHz should be within 8 dB (shelf plateau): 1kHz={g1k:+.1}, 5kHz={g5k:+.1}"
    );
}
