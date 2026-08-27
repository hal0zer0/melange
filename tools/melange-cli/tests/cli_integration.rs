//! CLI integration tests — invoke the melange binary and verify behavior.
//!
//! These tests use inline synthetic circuits to test CLI commands
//! without depending on external circuit files.

use std::path::PathBuf;
use std::process::Command;

/// Minimal RC lowpass for testing CLI commands (linear, no nonlinear devices).
const TEST_RC_LOWPASS: &str = "\
RC Lowpass Test Fixture
R1 in out 10k
C1 out 0 10n
";

/// Minimal diode clipper for testing nonlinear CLI paths.
const TEST_DIODE_CLIPPER: &str = "\
Diode Clipper Test Fixture
R1 in out 4.7k
D1 out 0 1N4148
D2 0 out 1N4148
C1 out 0 100n
.model 1N4148 D(IS=2.52e-9 N=1.752 BV=100 IBV=100u)
";

/// Resistive divider whose lower leg is a `.switch` — pos0 and pos1 give
/// clearly different divider ratios, so `simulate --switch` selecting a
/// non-rest position must change the output. The switch has both a label
/// ("Load") and a controlled component name ("Rsw") for name-resolution tests.
const TEST_SWITCH_DIVIDER: &str = "\
Switch Divider Test Fixture
R1 in out 10k
Rsw out 0 10k
C1 out 0 1n
.switch Rsw 10k 100k \"Load\"
";

/// Write a test circuit to a temp file and return the path.
fn write_test_circuit(content: &str, name: &str) -> PathBuf {
    let path = std::env::temp_dir().join(format!("melange_cli_test_{}.cir", name));
    std::fs::write(&path, content).expect("write test circuit");
    path
}

/// Get the path to the built melange binary.
fn melange_bin() -> PathBuf {
    // cargo test builds the binary in target/debug/
    let mut path = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf();
    path.push("melange");
    path
}

/// Get the project root (workspace root).
fn project_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

/// Run melange with args, assert success, return stdout.
fn run_melange(args: &[&str]) -> String {
    let output = Command::new(melange_bin())
        .args(args)
        .current_dir(project_root())
        .output()
        .expect("failed to run melange binary");

    if !output.status.success() {
        panic!(
            "melange {} failed (exit {}):\nstdout: {}\nstderr: {}",
            args.join(" "),
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        );
    }
    String::from_utf8_lossy(&output.stdout).to_string()
}

/// Run melange expecting failure, return stderr.
fn run_melange_fail(args: &[&str]) -> String {
    let output = Command::new(melange_bin())
        .args(args)
        .current_dir(project_root())
        .output()
        .expect("failed to run melange binary");

    assert!(
        !output.status.success(),
        "melange {} should have failed but succeeded:\n{}",
        args.join(" "),
        String::from_utf8_lossy(&output.stdout),
    );
    String::from_utf8_lossy(&output.stderr).to_string()
}

// ============================================================================
// builtins command
// ============================================================================

#[test]
fn test_builtins_lists_circuits() {
    let stdout = run_melange(&["builtins"]);
    // The passive-eq demo ships as the one builtin so melange can demo itself.
    assert!(
        stdout.contains("Available builtin circuits"),
        "Should print the builtins header, got: {stdout}"
    );
    assert!(
        stdout.contains("passive-eq1a"),
        "Should list the passive-eq1a builtin demo, got: {stdout}"
    );
}

// ============================================================================
// nodes command
// ============================================================================

#[test]
fn test_nodes_circuit_file() {
    let cir = write_test_circuit(TEST_RC_LOWPASS, "nodes");
    let stdout = run_melange(&["nodes", cir.to_str().unwrap()]);
    assert!(
        stdout.contains("in") || stdout.contains("out"),
        "Should list circuit nodes, got: {}",
        stdout
    );
    let _ = std::fs::remove_file(&cir);
}

// ============================================================================
// compile command
// ============================================================================

#[test]
fn test_compile_circuit_file() {
    let cir = write_test_circuit(TEST_DIODE_CLIPPER, "compile");
    let tmp = std::env::temp_dir().join("melange_cli_test_compile.rs");
    let tmp_str = tmp.to_str().unwrap();

    let stdout = run_melange(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        tmp_str,
        "--input-node",
        "in",
        "--output-node",
        "out",
    ]);

    assert!(
        stdout.contains("Done") || stdout.contains("Generated"),
        "Should indicate success: {}",
        stdout
    );

    // Verify output file exists and contains Rust code
    let code = std::fs::read_to_string(&tmp).expect("should read generated file");
    assert!(
        code.contains("process_sample"),
        "Generated code should contain process_sample function"
    );
    assert!(
        code.contains("CircuitState"),
        "Generated code should contain CircuitState struct"
    );

    let _ = std::fs::remove_file(&tmp);
    let _ = std::fs::remove_file(&cir);
}

#[test]
fn test_compile_produces_compilable_rust() {
    let cir = write_test_circuit(TEST_DIODE_CLIPPER, "compiles");
    let tmp_rs = std::env::temp_dir().join("melange_cli_test_compiles.rs");
    let tmp_rlib = std::env::temp_dir().join("melange_cli_test_compiles.rlib");

    run_melange(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        tmp_rs.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
    ]);

    // Compile the generated Rust code as a library
    let compile = Command::new("rustc")
        .arg(&tmp_rs)
        .arg("--edition=2024")
        .arg("--crate-type=lib")
        .arg("-o")
        .arg(&tmp_rlib)
        .output()
        .expect("rustc");

    let _ = std::fs::remove_file(&tmp_rs);
    let _ = std::fs::remove_file(&tmp_rlib);
    let _ = std::fs::remove_file(&cir);

    assert!(
        compile.status.success(),
        "Generated code should compile:\n{}",
        String::from_utf8_lossy(&compile.stderr)
    );
}

// ============================================================================
// simulate command
// ============================================================================

#[test]
fn test_simulate_sine_tone() {
    let cir = write_test_circuit(TEST_RC_LOWPASS, "sim");
    let tmp_wav = std::env::temp_dir().join("melange_cli_test_sim.wav");

    run_melange(&[
        "simulate",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
        "--amplitude",
        "0.1",
        "--output",
        tmp_wav.to_str().unwrap(),
        "--duration",
        "0.1",
    ]);

    // Verify WAV file exists and has content
    let metadata = std::fs::metadata(&tmp_wav).expect("WAV file should exist");
    assert!(
        metadata.len() > 100,
        "WAV file should have content, got {} bytes",
        metadata.len()
    );

    let _ = std::fs::remove_file(&tmp_wav);
    let _ = std::fs::remove_file(&cir);
}

#[test]
fn test_simulate_switch_selects_position() {
    // `simulate --switch NAME=POS` reaches non-rest switch positions at runtime
    // via `state.set_switch_N(pos)` (mirrors the generated plugin). Before this
    // feature `simulate` was pinned at position 0, forcing compile-and-run
    // harness gymnastics to observe any other position (openfarf, 2026-08-15).
    // Assert: (a) label resolution, (b) component-name resolution, (c) both
    // reach the same position -> byte-identical, and (d) that position differs
    // from the position-0 default (the switch actually restamps).
    let cir = write_test_circuit(TEST_SWITCH_DIVIDER, "switch_sel");
    let sim = |tag: &str, extra: &[&str]| -> PathBuf {
        let wav = std::env::temp_dir().join(format!("melange_cli_test_switch_{tag}.wav"));
        let mut args = vec![
            "simulate",
            cir.to_str().unwrap(),
            "--input-node",
            "in",
            "--output-node",
            "out",
            "--amplitude",
            "0.5",
            "--duration",
            "0.02",
            "--output",
            wav.to_str().unwrap(),
        ];
        args.extend_from_slice(extra);
        run_melange(&args);
        wav
    };

    let pos0 = sim("pos0", &[]);
    let pos1_label = sim("pos1_label", &["--switch", "Load=1"]);
    let pos1_name = sim("pos1_name", &["--switch", "Rsw=1"]);

    let b0 = wav_pcm_bytes(&pos0);
    let b1l = wav_pcm_bytes(&pos1_label);
    let b1n = wav_pcm_bytes(&pos1_name);

    assert_ne!(
        b0, b1l,
        "position 1 output must differ from the position-0 default"
    );
    assert_eq!(
        b1l, b1n,
        "label (Load) and component-name (Rsw) resolution must select the same switch/position"
    );

    // Unknown switch name is a hard error with a helpful available-list.
    let stderr = run_melange_fail(&[
        "simulate",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
        "--output",
        std::env::temp_dir()
            .join("melange_cli_test_switch_err.wav")
            .to_str()
            .unwrap(),
        "--switch",
        "NoSuch=1",
    ]);
    assert!(
        stderr.contains("not found") && stderr.contains("Load"),
        "error should name the missing switch and list available ones, got: {stderr}"
    );

    for p in [&pos0, &pos1_label, &pos1_name] {
        let _ = std::fs::remove_file(p);
    }
    let _ = std::fs::remove_file(&cir);
}

#[test]
fn test_analyze_integral_amplitude() {
    // Regression: `--amplitude 1.0` used to emit `let amplitude = 1;` in the
    // generated analyze main (f64 Display drops the trailing `.0`), producing
    // an integer literal that fails rustc with E0277 (cannot multiply
    // {integer} by f64). Reported by the velvet-elvis agent 2026-06-10.
    let cir = write_test_circuit(TEST_RC_LOWPASS, "analyze_amp");

    let stdout = run_melange(&[
        "analyze",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
        "--amplitude",
        "1.0",
        "--start-freq",
        "1000",
        "--end-freq",
        "2000",
        "--points-per-decade",
        "1",
    ]);

    assert!(
        stdout.contains("frequency_hz,gain_db,phase_deg"),
        "expected analyze CSV header, got:\n{stdout}"
    );

    let _ = std::fs::remove_file(&cir);
}

/// Read the raw PCM payload (bytes after the `data` chunk header) of a WAV
/// file written by melange's embedded writer. Byte-level compare is enough —
/// this test only needs "did the output change," not decoded sample values.
fn wav_pcm_bytes(path: &std::path::Path) -> Vec<u8> {
    let data = std::fs::read(path).expect("read wav");
    let idx = data
        .windows(4)
        .position(|w| w == b"data")
        .expect("wav data chunk");
    data[idx + 8..].to_vec()
}

#[test]
fn test_simulate_noise_full_actually_injects_noise() {
    // Regression: `simulate --noise <mode>` used to compile in the full
    // noise machinery but leave the runtime `noise_enabled` master switch at
    // its default (false), so output was byte-identical and seed-invariant
    // to `--noise off` — a silent no-op with no warning. Reported by
    // melange-circuits 2026-07-25
    // (local-docs/melange-bug-simulate-noise-noop-2026-07-25.md in their
    // repo). Fixed by emitting `state.set_noise_enabled(true)` in the
    // generated `main()` when the caller passed `--noise <mode>` — passing
    // the flag IS the opt-in for `simulate`/`analyze`, unlike the plugin
    // seam where noise defaults off until the host UI enables it.
    let cir = write_test_circuit(TEST_RC_LOWPASS, "noise_noop");
    let wav_off = std::env::temp_dir().join("melange_cli_test_noise_off.wav");
    let wav_full_a = std::env::temp_dir().join("melange_cli_test_noise_full_a.wav");
    let wav_full_b = std::env::temp_dir().join("melange_cli_test_noise_full_b.wav");

    let base_args = |out: &std::path::Path, extra: &[&str]| -> Vec<String> {
        let mut args = vec![
            "simulate".to_string(),
            cir.to_str().unwrap().to_string(),
            "--input-node".to_string(),
            "in".to_string(),
            "--output-node".to_string(),
            "out".to_string(),
            "--amplitude".to_string(),
            "0.0".to_string(),
            "--duration".to_string(),
            "0.1".to_string(),
            "--output".to_string(),
            out.to_str().unwrap().to_string(),
        ];
        args.extend(extra.iter().map(|s| s.to_string()));
        args
    };

    run_melange(
        &base_args(&wav_off, &["--noise", "off"])
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    );
    run_melange(
        &base_args(&wav_full_a, &["--noise", "full", "--noise-seed", "12345"])
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    );
    run_melange(
        &base_args(&wav_full_b, &["--noise", "full", "--noise-seed", "999"])
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    );

    let off = wav_pcm_bytes(&wav_off);
    let full_a = wav_pcm_bytes(&wav_full_a);
    let full_b = wav_pcm_bytes(&wav_full_b);

    assert_ne!(
        off, full_a,
        "--noise full must differ from --noise off (silent no-op regression)"
    );
    assert_ne!(
        full_a, full_b,
        "different --noise-seed values must produce different output \
         (seed-invariance means the RNG master switch is still off)"
    );

    let _ = std::fs::remove_file(&cir);
    let _ = std::fs::remove_file(&wav_off);
    let _ = std::fs::remove_file(&wav_full_a);
    let _ = std::fs::remove_file(&wav_full_b);
}

#[test]
fn test_analyze_linear_circuit_harmonics_at_numerical_floor() {
    // Regression for the DFT leakage bug: the measurement window rounded to N
    // samples while the drive frequency stayed exact, leaving up to 0.5
    // samples of cycle mismatch. The rectangular-window leakage floored
    // h2..hN at −50..−75 dBc even on a perfectly linear circuit. With the
    // drive snapped to the exact bin DFT_CYCLES·sr/N, a linear RC must show
    // every harmonic at the numerical floor (below −120 dBc) or nan
    // (above Nyquist).
    //
    // Frequencies are chosen so DFT_CYCLES·sr/f is NOT an integer — the
    // pre-fix leakage regime.
    let cir = write_test_circuit(TEST_RC_LOWPASS, "analyze_leakage");

    let stdout = run_melange(&[
        "analyze",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
        "--amplitude",
        "0.5",
        "--start-freq",
        "313",
        "--end-freq",
        "7919",
        "--points-per-decade",
        "2",
        "--harmonics",
        "5",
    ]);
    let _ = std::fs::remove_file(&cir);

    // CSV: frequency_hz,gain_db,phase_deg,thd_pct,h2_dbc..h5_dbc,nyquist_dbc
    let mut data_rows = 0;
    for line in stdout
        .lines()
        .skip_while(|l| !l.starts_with("frequency_hz"))
    {
        if line.starts_with("frequency_hz") || line.trim().is_empty() {
            continue;
        }
        let cols: Vec<&str> = line.split(',').collect();
        assert!(
            cols.len() >= 9,
            "expected >= 9 CSV columns with --harmonics 5, got {}: {line}",
            cols.len()
        );
        data_rows += 1;
        for (i, col) in cols[4..8].iter().enumerate() {
            if *col == "nan" {
                continue; // harmonic above Nyquist
            }
            let dbc: f64 = col
                .parse()
                .unwrap_or_else(|_| panic!("unparseable h{}_dbc {col:?} in {line}", i + 2));
            assert!(
                dbc < -120.0,
                "h{}_dbc = {dbc} dBc on a pure linear RC — DFT leakage floor is back \
                 (was −50..−75 dBc before the bin snap). Row: {line}",
                i + 2
            );
        }
    }
    assert!(data_rows >= 3, "expected >= 3 data rows, got:\n{stdout}");
}

// ============================================================================
// error cases
// ============================================================================

#[test]
fn test_compile_mono_with_multiple_outputs_errors() {
    // --mono + multiple output nodes cannot be represented (multi-output
    // plugins route one node per channel) and used to generate a broken
    // plugin silently.
    let cir = write_test_circuit(
        "Two Output Test\nR1 in out 10k\nC1 out 0 10n\nR2 out out2 1k\nC2 out2 0 10n\n",
        "mono_multi",
    );
    let stderr = run_melange_fail(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        "/tmp/melange_cli_mono_multi.rs",
        "--input-node",
        "in",
        "--output-node",
        "out,out2",
        "--mono",
    ]);
    let _ = std::fs::remove_file(&cir);
    assert!(
        stderr.contains("--mono") && stderr.contains("output node"),
        "should explain the --mono / multi-output conflict: {stderr}"
    );
}

#[test]
fn test_compile_missing_file() {
    let stderr = run_melange_fail(&[
        "compile",
        "nonexistent_circuit.cir",
        "--output",
        "/tmp/melange_cli_fail.rs",
        "--input-node",
        "in",
        "--output-node",
        "out",
    ]);
    assert!(
        stderr.contains("not found")
            || stderr.contains("No such file")
            || stderr.contains("error")
            || stderr.contains("Error"),
        "Should report missing file error: {}",
        stderr
    );
}

#[test]
fn test_compile_missing_node() {
    let cir = write_test_circuit(TEST_RC_LOWPASS, "missing_node");
    let stderr = run_melange_fail(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        "/tmp/melange_cli_fail.rs",
        "--input-node",
        "nonexistent_node",
        "--output-node",
        "out",
    ]);
    let _ = std::fs::remove_file(&cir);
    assert!(
        stderr.contains("node")
            || stderr.contains("Node")
            || stderr.contains("error")
            || stderr.contains("Error"),
        "Should report missing node error: {}",
        stderr
    );
}

// ============================================================================
// Routing-rate regression (2026-08-14): trap-unstable-only-at-internal-rate
// circuit must route to nodal under DEFAULT (auto) flags, not silently
// diverge on DK Schur.
// ============================================================================

/// Verbatim copy of `melange-circuits/local-docs/g10-osc-sq2-repro.cir`
/// (2026-08-14) — a Farfisa Compact G10 master LC oscillator (Ge PNP,
/// transformer-coupled regenerative feedback). Embedded so this test does
/// not depend on the sibling `melange-circuits` checkout being present.
///
/// This circuit is trap-stable-looking at the 48 kHz host rate but
/// genuinely trap-unstable at the 192 kHz internal rate `--oversampling 4`
/// ships (spectral_radius(S*A_neg) = 1.315, a real growing mode — the
/// circuit is a regenerative oscillator, unstable at its DC-OP
/// linearization by design). Before the routing-rate fix, `auto_route`
/// built its kernel at the host rate, missed the instability, and shipped
/// this on DK Schur — which has no full-LU NR fallback for the resulting
/// device-state-transition NR failure. It diverged to a nonphysical peak of
/// ~40 kV (`nr_max_iter_count` in the thousands). Routing to nodal (which
/// has the full-LU NR fallback) converges cleanly to a bounded, physical
/// oscillation.
const G10_OSCILLATOR: &str = "\
Farfisa Compact G10 reference chain (master osc + squarer + divider + keying)
Vrail rail 0 DC 8
Vvib vterm 0 DC 8
.runtime Vvib as v_g10_vterm
C_kick in b1 1n
R_e18 rail node_a 1.8k
C_e25 rail node_a 25u
L_fb node_b node_a 8.35m
R_180 node_b e1 180
Q_TN1G c1 b1 e1 SFT307
L_tank c1 tanklo 1.36
K1 L_tank L_fb 0.3
C_tank c1 tanklo 13.5n
R_47k tanklo b1 47k
R_27k2 tanklo 0 2.7k
R_10k rail b1 10k
R_27k vterm b1 27k
C_sq c1 sq_n 10n
R_sqb sq_n b2 47k
R_470k b2 0 470k
Q_TN2G c2 b2 rail SFT307
R_c2 c2 0 10k
C_fout c2 term_f 1u
R_fbleed term_f 0 100k
.model SFT307 PNP(IS=2e-7 BF=110 VAF=60 RB=50 RC=5 RE=1 CJE=60p CJC=25p TF=1n)
.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

/// Parse `"  <key>: <value>"` out of `melange simulate`'s stdout summary.
fn parse_summary_value(stdout: &str, key: &str) -> f64 {
    for line in stdout.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix(&format!("{key}:")) {
            return rest
                .trim()
                .parse()
                .unwrap_or_else(|_| panic!("could not parse '{key}' value from line: {line:?}"));
        }
    }
    panic!("'{key}:' not found in simulate output:\n{stdout}");
}

#[test]
fn test_g10_oscillator_default_routing_is_bounded() {
    let cir = write_test_circuit(G10_OSCILLATOR, "g10_osc_default");
    let tmp_wav = std::env::temp_dir().join("melange_cli_test_g10_osc_default.wav");

    // Deliberately NO --solver / --backward-euler / --force-trap override —
    // this is exactly what a plugin build with default flags ships.
    let stdout = run_melange(&[
        "simulate",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "term_f",
        "--amplitude",
        "1e-9",
        "--duration",
        "0.05",
        "--oversampling",
        "4",
        "--output",
        tmp_wav.to_str().unwrap(),
    ]);

    let _ = std::fs::remove_file(&tmp_wav);
    let _ = std::fs::remove_file(&cir);

    assert!(
        stdout.contains("Solver: nodal"),
        "default routing must select nodal for this circuit (trap-unstable at the \
         192 kHz internal rate) — DK Schur has no full-LU NR rescue for the ensuing \
         divergence. Full stdout:\n{stdout}"
    );

    let peak = parse_summary_value(&stdout, "peak");
    let nr_max_iter_count = parse_summary_value(&stdout, "nr_max_iter_count");

    // Pre-fix: peak ~39965 (a nonphysical ~-36 kV drift), nr_max_iter_count
    // ~4953 (NR hit its cap on nearly every internal sample). Post-fix:
    // peak ~8.3 V (a physical tank swing), nr_max_iter_count in single
    // digits. Generous bounds here — this pins "bounded and physical", not
    // an exact byte-for-byte oscillation amplitude.
    assert!(
        peak.is_finite() && peak.abs() < 100.0,
        "peak must be a bounded, physical voltage under default routing, got {peak} \
         (pre-fix this diverged to ~39965)"
    );
    assert!(
        nr_max_iter_count < 100.0,
        "NR must not be hitting its iteration cap on a large fraction of samples \
         under default routing, got nr_max_iter_count={nr_max_iter_count} \
         (pre-fix this was ~4953 over 2400 samples)"
    );
}

// ============================================================================
// IC= + oversampling — NR damping-floor regression (2026-08-14)
// ============================================================================

/// The reported IC=/VCVS blowup repro (melange-circuits
/// `local-docs/repro-ic-vcvs-blowup.cir`, verbatim), embedded so this test
/// is self-contained. G10-divider astable: two cross-coupled PNP stages,
/// `IC=-4` on the cross-coupling cap `C_x1` kicks the circuit off its
/// degenerate symmetric DC fixed point. Routes DK Schur (N=13, M=4),
/// trapezoidal — NOT the nodal-routing bug pinned by
/// `test_g10_oscillator_default_routing_is_bounded` above (different
/// circuit, different mechanism).
const IC_VCVS_ASTABLE: &str = "\
G10 divider lab v3 (internal PULSE drive)
Vrail rail 0 DC 8
R_green green 0 2.7k
E_amp drvsrc 0 in 0 8
R_trig drvsrc trigmid 10k
C_trig trigmid b4 1n
Q_D1A c3 b3 rail SFT352
Q_D1B c4 b4 rail SFT352
R_b3 b3 green 150k
R_b4 b4 green 150k
R_c4 c4 green 56k
R_c3 c3 ntap 2.7k
R_ntap ntap green 27k
C_x1 c3 b4 6n IC=-4
C_x2 c4 b3 6.8n
C_out ntap out 1u
R_bleed out green 100k

.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

#[test]
fn test_ic_seeded_astable_stays_bounded_at_os1() {
    let cir = write_test_circuit(IC_VCVS_ASTABLE, "ic_astable_os1");
    let tmp_wav = std::env::temp_dir().join("melange_cli_test_ic_astable_os1.wav");

    let stdout = run_melange(&[
        "simulate",
        cir.to_str().unwrap(),
        "--output-node",
        "out",
        "--amplitude",
        "1e-9",
        "--duration",
        "0.05",
        "--output",
        tmp_wav.to_str().unwrap(),
    ]);

    let _ = std::fs::remove_file(&tmp_wav);
    let _ = std::fs::remove_file(&cir);

    // This circuit is a genuine astable multivibrator: the IC seed kicks it
    // into real switching dynamics (measured plateau ~24.5 V at base rate
    // over 50 ms, not a settle back to a flat DC point) — 50 V is a loose
    // bound with real margin over that plateau, not a tight fit.
    let peak = parse_summary_value(&stdout, "peak");
    assert!(
        peak.is_finite() && peak.abs() < 50.0,
        "base-rate peak must be bounded and physical, got {peak}. Full stdout:\n{stdout}"
    );
}

/// Pins the OS4-specific NR damping-floor bug: at `--oversampling 4`
/// (192 kHz internal rate for a 48 kHz host), the same IC-seeded transient
/// that converges cleanly at base rate (peak ~1.18 V, `nr_max_iter_count`
/// O(1)) drove the DK-Schur per-iteration NR voltage-limiting into a
/// pathological regime: two `.max(0.01)`/`.max(0.1)` floors in
/// `nr_helpers.rs::emit_nr_limit_and_converge` (per-device pnjlim/fetlim
/// ratio, and the current-space backstop) each let a fixed *fraction* of an
/// arbitrarily large raw Newton step through instead of clamping the
/// *step itself* — same anti-pattern as the nodal full-LU node-step damping
/// floor removed in `e6d5db8`. Pre-fix: peak 488,012 V,
/// `nr_max_iter_count` 19014 over 2400 samples (NR exhausting its 270-iter
/// budget on nearly every internal sample, `be_fallback_count` 9507). A
/// third floor (`process_sample.rs.tera`'s Step 6c global per-sample voltage
/// damping) had the identical pattern and was fixed alongside the two above.
/// Post-fix: peak stays bounded (a legitimate — if larger than the
/// base-rate value — oscillation from the astable's IC-forced switching,
/// which the coarser base-rate step numerically damps away; OS4 resolves it
/// correctly). Generous bound here pins "bounded and physical", not an
/// exact oscillation amplitude — OS1 and OS4 are NOT expected to converge
/// to the same peak for a genuinely astable circuit.
#[test]
fn test_ic_seeded_astable_stays_bounded_at_os4() {
    let cir = write_test_circuit(IC_VCVS_ASTABLE, "ic_astable_os4");
    let tmp_wav = std::env::temp_dir().join("melange_cli_test_ic_astable_os4.wav");

    let stdout = run_melange(&[
        "simulate",
        cir.to_str().unwrap(),
        "--output-node",
        "out",
        "--amplitude",
        "1e-9",
        "--duration",
        "0.05",
        "--oversampling",
        "4",
        "--output",
        tmp_wav.to_str().unwrap(),
    ]);

    let _ = std::fs::remove_file(&tmp_wav);
    let _ = std::fs::remove_file(&cir);

    assert!(
        stdout.contains("Solver: DK"),
        "expected this circuit to route DK Schur (not the nodal-routing bug \
         pinned elsewhere) — full stdout:\n{stdout}"
    );

    let peak = parse_summary_value(&stdout, "peak");
    let magnitude_reset_count = parse_summary_value(&stdout, "magnitude_reset_count");
    let nan_reset_count = parse_summary_value(&stdout, "nan_reset_count");

    assert!(
        peak.is_finite() && peak.abs() < 200.0,
        "OS4 peak must be bounded and physical, got {peak} \
         (pre-fix this diverged to ~488,012). Full stdout:\n{stdout}"
    );
    assert_eq!(
        magnitude_reset_count, 0.0,
        "the STATE_MAX_PLAUSIBLE_MAGNITUDE safety net should not need to fire \
         once NR is actually converging — got {magnitude_reset_count}"
    );
    assert_eq!(nan_reset_count, 0.0, "no NaN/Inf should ever appear");
}

/// NR-starvation validity warning. A too-low `--max-iter` can leave the solver
/// non-converged on most/all samples, latching every node at a DC-ish value
/// that reads as a physical steady state while being numerically meaningless —
/// with nothing in the normal output flagging it. `simulate` must emit a WARNING
/// to stderr in that case. (Requested by melange-circuits 2026-08-15 after an
/// NR-starved Ge astable cascade silently latched and cost a half-day.)
#[test]
fn test_simulate_warns_on_nr_starvation() {
    let cir = write_test_circuit(TEST_DIODE_CLIPPER, "nr_starve");
    let tmp_wav = std::env::temp_dir().join("melange_cli_test_nr_starve.wav");

    // --max-iter 1 cannot converge a driven diode clipper: expect the warning.
    let starved = Command::new(melange_bin())
        .args([
            "simulate",
            cir.to_str().unwrap(),
            "-n",
            "out",
            "--amplitude",
            "0.9",
            "-d",
            "0.01",
            "--max-iter",
            "1",
            "--output",
            tmp_wav.to_str().unwrap(),
        ])
        .current_dir(project_root())
        .output()
        .expect("failed to run melange");
    let starved_err = String::from_utf8_lossy(&starved.stderr).to_string();
    assert!(
        starved.status.success(),
        "simulate should still succeed (warning is advisory), stderr:\n{starved_err}"
    );
    assert!(
        starved_err.contains("WARNING") && starved_err.contains("Newton-Raphson"),
        "expected an NR-starvation WARNING on stderr at --max-iter 1, got:\nstdout:{}\nstderr:{starved_err}",
        String::from_utf8_lossy(&starved.stdout)
    );

    // Same circuit with an adequate ceiling must NOT warn (no false positive).
    let clean = Command::new(melange_bin())
        .args([
            "simulate",
            cir.to_str().unwrap(),
            "-n",
            "out",
            "--amplitude",
            "0.9",
            "-d",
            "0.01",
            "--max-iter",
            "100",
            "--output",
            tmp_wav.to_str().unwrap(),
        ])
        .current_dir(project_root())
        .output()
        .expect("failed to run melange");
    let clean_err = String::from_utf8_lossy(&clean.stderr).to_string();

    let _ = std::fs::remove_file(&tmp_wav);
    let _ = std::fs::remove_file(&cir);

    assert!(
        !clean_err.contains("WARNING"),
        "a converging run must not emit the NR-starvation warning, got stderr:\n{clean_err}"
    );
}
