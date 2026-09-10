//! VCA control-law (active-CV) tests.
//!
//! The golden suite only ever exercises the VCA at V_ctrl = 0, so a bug that
//! silently zeroed the control response would pass every existing test. These
//! tests drive the VCA control input to non-zero voltages and assert that the
//! gain actually tracks the Blackmer exponential law G(Vc) = G0·exp(-Vc/Vscale).
//!
//! The deck is the current-mode (MODE=1) VCA isolation circuit: a 27 kΩ series
//! drive resistor feeds the virtual-ground current input, and a 15 kΩ
//! transimpedance stage converts the sensed current back to a voltage. Absolute
//! gain depends on 15K/27K and op-amp constants, so every assertion is a RATIO
//! relative to the Vc = 0 peak, which is exactly exp(-Vc/Vscale) for a correct
//! control law and independent of the surrounding topology constants.
//!
//! VSCALE = 0.05298 V/neper (THAT 2180A). At Vc = ±0.0611 V the exponent is
//! ∓1.1533, giving 0.3156 (−10 dB) and 3.169 (+10 dB).
//!
//! The companion `..._voltage_mode_washes_out` test flips the same deck to
//! MODE=0 and asserts the control response DISAPPEARS (all three peaks equal) —
//! this is the washout footgun, and it proves the active-CV test above has real
//! discriminating power (it would fail on a MODE=0 deck).

use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// VCA isolation deck. `{MODE}` and `{VCTRL}` are substituted per run.
/// Control voltage is applied directly to the VCA control node so V_ctrl equals
/// the DC value exactly (the control input is ideal high-Z, drawing no current).
const VCA_DECK_TEMPLATE: &str = "\
VCA Control-Law Isolation Test
.model VCA2180 VCA(VSCALE=0.05298 G0=1.0 MODE={MODE})
.model OA1 OA(AOL=100000 ROUT=100 VSAT=13 GBW=10MEG)
Rdrive in vca_in 27K
Vctrl vca_ctrl 0 DC {VCTRL}
Rpull vca_ctrl 0 1MEG
Y1 vca_in iv_inv vca_ctrl 0 VCA2180
U1 0 iv_inv iv_out OA1
Rfb iv_out iv_inv 15K
Rout iv_out out 100
Rload out 0 47K
.END
";

static COUNTER: AtomicU32 = AtomicU32::new(0);

fn build_deck(mode: u32, vctrl: f64) -> String {
    VCA_DECK_TEMPLATE
        .replace("{MODE}", &mode.to_string())
        .replace("{VCTRL}", &format!("{vctrl}"))
}

fn generate_nodal_code(spice: &str, sample_rate: f64) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    // Thevenin input conductance (1 Ω source), stamped BEFORE codegen.
    mna.g[input_node][input_node] += 1.0;

    let config = CodegenConfig {
        circuit_name: "vca_ctrl_test".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_vcactrl_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_vcactrl_{tag}_{id}_{counter}"));

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
        .unwrap_or_else(|| panic!("key '{key}' not found in output:\n{output}"))
        .split('=')
        .nth(1)
        .unwrap()
        .trim()
        .parse()
        .unwrap()
}

/// Drives a 1 kHz sine and returns the steady-state peak of the output.
const PEAK_MAIN: &str = r#"
fn main() {
    let mut state = CircuitState::default();
    state.warmup();

    let sr = 48000.0f64;
    let freq = 1000.0f64;
    let mut peak = 0.0f64;
    for i in 0..(sr as usize / 2) {
        let t = i as f64 / sr;
        let input = 1.0 * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = process_sample(input, &mut state);
        let v = output[0].abs();
        if i > 4800 { peak = peak.max(v); } // settle 100 ms first
    }
    println!("peak={:.9}", peak);
    println!("nr_fail={}", state.diag_nr_max_iter_count);
    println!("nan_reset={}", state.diag_nan_reset_count);
}
"#;

fn measure_peak(mode: u32, vctrl: f64, tag: &str) -> f64 {
    let code = generate_nodal_code(&build_deck(mode, vctrl), 48000.0);
    let output = compile_and_run(&code, PEAK_MAIN, tag);
    let nr_fail = parse_kv(&output, "nr_fail") as u32;
    let nan_reset = parse_kv(&output, "nan_reset") as u32;
    assert_eq!(nr_fail, 0, "NR convergence failures for {tag}");
    assert_eq!(nan_reset, 0, "NaN resets for {tag}");
    let peak = parse_kv(&output, "peak");
    assert!(
        peak.is_finite() && peak > 1e-6,
        "peak degenerate for {tag}: {peak}"
    );
    peak
}

/// Current-mode (MODE=1) VCA must track the exponential control law:
/// peak(Vc)/peak(0) = exp(-Vc/Vscale).
#[test]
fn test_vca_current_mode_control_law_modulates_gain() {
    let vscale = 0.05298;
    let vc = 0.0611;

    let peak_0 = measure_peak(1, 0.0, "cm_v0");
    let peak_pos = measure_peak(1, vc, "cm_vpos"); // gain reduced (−10 dB)
    let peak_neg = measure_peak(1, -vc, "cm_vneg"); // gain boosted (+10 dB)

    let expected_pos = (-vc / vscale).exp(); // 0.3156
    let expected_neg = (vc / vscale).exp(); // 3.169

    let ratio_pos = peak_pos / peak_0;
    let ratio_neg = peak_neg / peak_0;

    // 4% tolerance: the circuit is linear (THD=0, unclipped), so the ratio is
    // dominated by the exponential control law; slack covers settling and the
    // 1 Ω/27 kΩ current-divider term.
    assert!(
        (ratio_pos - expected_pos).abs() / expected_pos < 0.04,
        "Vc=+{vc}: peak ratio {ratio_pos:.4} should be ~{expected_pos:.4} (−10 dB); \
         peaks: v0={peak_0:.6} vpos={peak_pos:.6}"
    );
    assert!(
        (ratio_neg - expected_neg).abs() / expected_neg < 0.04,
        "Vc=-{vc}: peak ratio {ratio_neg:.4} should be ~{expected_neg:.4} (+10 dB); \
         peaks: v0={peak_0:.6} vneg={peak_neg:.6}"
    );
}

/// Companion / discrimination guard: the SAME deck in voltage mode (MODE=0)
/// with a 27 kΩ series drive (R_drive·G0 = 27000 ≫ 1) degenerates into a fixed
/// passive inverter whose gain no longer depends on V_ctrl — the washout
/// footgun. All three peaks must be equal. This both documents the failure
/// mode and proves the active-CV test above genuinely catches it (that test
/// would fail here).
#[test]
fn test_vca_voltage_mode_washes_out_control() {
    let vc = 0.0611;

    let peak_0 = measure_peak(0, 0.0, "vm_v0");
    let peak_pos = measure_peak(0, vc, "vm_vpos");
    let peak_neg = measure_peak(0, -vc, "vm_vneg");

    let ratio_pos = peak_pos / peak_0;
    let ratio_neg = peak_neg / peak_0;

    // Washed out: control voltage has (essentially) no effect on gain.
    assert!(
        (ratio_pos - 1.0).abs() < 0.02 && (ratio_neg - 1.0).abs() < 0.02,
        "voltage-mode VCA with R_drive·G0≫1 should wash out CV (ratios ~1.0), \
         got ratio_pos={ratio_pos:.4}, ratio_neg={ratio_neg:.4} \
         (v0={peak_0:.6} vpos={peak_pos:.6} vneg={peak_neg:.6})"
    );
}
