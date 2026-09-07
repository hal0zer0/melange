//! Sub-sample fire (`--subsample-fire`, nodal-Schur variable-dt glow-strike
//! breakpoint re-solve) — Stage A regression tests.
//!
//! Pins:
//!  - byte-neutrality: a non-glow deck emits identical code for off/on/auto and
//!    carries no sub-sample machinery; a glow deck with `off` carries none;
//!  - routing contract: `on` is refused on the DK route and on the nodal
//!    full-LU sub-path; `auto` is inert there;
//!  - multi-breakpoint behaviour on a 2-stage glow divider cascade at 48 kHz:
//!    strikes are detected AND resolved (not just the earliest per sample),
//!    some samples carry two breakpoints (the cascade strike lands in the same
//!    inner sample as the upstream flyback), nothing is abandoned, no NaN /
//!    magnitude resets, and the output differs from the `off` build.

use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

use melange_solver::codegen::{
    CodeGenerator, CodegenConfig, NodalSubPathOverride, SubsampleFireMode,
};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

static COUNTER: AtomicU32 = AtomicU32::new(0);

/// Two-stage ZA1001 divider (B5 -> B6 via the C36/C35 cathode coupler). The
/// upstream flyback strikes the downstream lamp within the same inner sample.
fn chain_deck() -> &'static str {
    "\
Glow 2-stage divider chain
R35 ht a5 1.5meg
R36 k5 0 47k
N5 a5 k5 ZA1001
C11 a5 m5 470p
C12 m5 0 5.6n
C36 a5 k6 22p
C35 k6 0 15p
R16 ht r16w 650k
R22 r16w a6 1.5meg
N6 a6 k6 ZA1001
D_GR1 k6 0 BA100
C13 a6 m6 1n
C14 m6 0 10n
R6 in k5 100k
R_out m6 osc 1k
R_load osc 0 1meg
VHT ht 0 DC 175
.model BA100 D(IS=2e-9 N=1.9 RS=8 CJO=1.5p BV=60)
.model ZA1001 NEON(VO=135 VM=93 IK=1.5m RS=3000 IHOLD=2e-4 ROFF=1e9)
.END
"
}

/// Non-glow nonlinear deck (diode clipper) — the byte-neutrality control.
fn clipper_deck() -> &'static str {
    "\
Diode clipper
R1 in n1 4.7k
C1 n1 osc 47n
Rl osc 0 100k
D1 osc 0 D1N4148
D2 0 osc D1N4148
.model D1N4148 D(IS=2.52e-9 N=1.752 RS=0.568 CJO=4e-12)
.END
"
}

fn make_config(
    spice: &str,
    sample_rate: f64,
    mode: SubsampleFireMode,
    sub_path: NodalSubPathOverride,
) -> (CodegenConfig, MnaSystem, Netlist) {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["osc"] - 1;
    mna.g[input_node][input_node] += 1.0;
    let config = CodegenConfig {
        circuit_name: "subsample_fire_test".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: false,
        subsample_fire: mode,
        nodal_sub_path_override: sub_path,
        ..CodegenConfig::default()
    };
    (config, mna, netlist)
}

fn nodal_code(spice: &str, sample_rate: f64, mode: SubsampleFireMode) -> String {
    let (config, mna, netlist) = make_config(spice, sample_rate, mode, NodalSubPathOverride::Auto);
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

#[test]
fn non_glow_deck_is_byte_identical_across_modes() {
    let off = nodal_code(clipper_deck(), 48000.0, SubsampleFireMode::Off);
    let on = nodal_code(clipper_deck(), 48000.0, SubsampleFireMode::On);
    let auto = nodal_code(clipper_deck(), 48000.0, SubsampleFireMode::Auto);
    assert_eq!(off, on, "non-glow deck: --subsample-fire on changed the emitted code");
    assert_eq!(off, auto, "non-glow deck: --subsample-fire auto changed the emitted code");
    assert!(
        !off.contains("subsample"),
        "non-glow deck must carry no sub-sample fire machinery"
    );
}

#[test]
fn glow_deck_off_has_no_machinery_and_auto_equals_on() {
    let off = nodal_code(chain_deck(), 48000.0, SubsampleFireMode::Off);
    let on = nodal_code(chain_deck(), 48000.0, SubsampleFireMode::On);
    let auto = nodal_code(chain_deck(), 48000.0, SubsampleFireMode::Auto);
    assert!(!off.contains("subsample"), "glow deck with off must carry no machinery");
    assert_eq!(auto, on, "glow deck on nodal-Schur: auto must resolve to on");
    for token in [
        "fn subsample_schur_build(",
        "struct SubsampleSchur",
        "SUBSAMPLE_FIRE_MAX_BREAKPOINTS: u32 = 4",
        "extinguished: bool,",
        "diag_subsample_fire_count",
        "diag_subsample_fire_detected",
        "diag_subsample_fire_resolved",
        "diag_subsample_fire_abandon_count",
        "diag_subsample_fire_segments",
        "SUBSAMPLE_FIRE_LIT_TAU_S: f64 = 1.476",
        ", subsample-fire",
        "\"subsample_fire\":true",
    ] {
        assert!(on.contains(token), "glow deck with on is missing `{token}`");
    }
}

#[test]
fn on_is_refused_on_dk_route_and_auto_is_inert_there() {
    let (config, mna, netlist) = make_config(
        chain_deck(),
        48000.0,
        SubsampleFireMode::On,
        NodalSubPathOverride::Auto,
    );
    let kernel = DkKernel::from_mna(&mna, 48000.0).expect("kernel");
    let err = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .err()
        .expect("--subsample-fire on must be refused on the DK route");
    assert!(
        format!("{err}").contains("requires the nodal route"),
        "unexpected DK refusal message: {err}"
    );
    let (config, mna, netlist) = make_config(
        chain_deck(),
        48000.0,
        SubsampleFireMode::Auto,
        NodalSubPathOverride::Auto,
    );
    let code = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("DK auto")
        .code;
    assert!(!code.contains("subsample"), "DK route must never emit sub-sample fire");
}

#[test]
fn on_is_refused_on_full_lu_subpath_and_auto_is_inert_there() {
    let (config, mna, netlist) = make_config(
        chain_deck(),
        48000.0,
        SubsampleFireMode::On,
        NodalSubPathOverride::FullLu,
    );
    let err = CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .err()
        .expect("--subsample-fire on must be refused on the full-LU sub-path");
    assert!(
        format!("{err}").contains("full-LU"),
        "unexpected full-LU refusal message: {err}"
    );
    let (config, mna, netlist) = make_config(
        chain_deck(),
        48000.0,
        SubsampleFireMode::Auto,
        NodalSubPathOverride::FullLu,
    );
    let code = CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("full-LU auto")
        .code;
    assert!(
        !code.contains("subsample"),
        "full-LU sub-path with auto must not emit sub-sample fire (incl. provenance)"
    );
}

const CHAIN_MAIN: &str = r#"
fn main() {
    let mut state = CircuitState::default();
    let n = 24000usize; // 0.5 s at 48 kHz
    let mut acc = 0.0f64;
    let mut maxabs = 0.0f64;
    for i in 0..n {
        let out = process_sample(0.0, &mut state);
        acc += out[0] * ((i % 7) as f64 + 1.0);
        for &v in state.v_prev.iter() {
            let a = v.abs();
            if a > maxabs { maxabs = a; }
        }
    }
    println!("acc={:.9}", acc);
    println!("maxabs={:.3}", maxabs);
    println!("nan_reset={}", state.diag_nan_reset_count);
    println!("magnitude_reset={}", state.diag_magnitude_reset_count);
    println!("nr_max_iter={}", state.diag_nr_max_iter_count);
    DIAG_LINES
}
"#;

fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_ssf_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_ssf_{tag}_{id}_{counter}"));
    let full_code = format!("{code}\n\n{main_code}\n");
    std::fs::File::create(&src_path)
        .unwrap()
        .write_all(full_code.as_bytes())
        .unwrap();
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
    assert!(
        run.status.success(),
        "binary failed for {tag}: {}",
        String::from_utf8_lossy(&run.stderr)
    );
    String::from_utf8_lossy(&run.stdout).to_string()
}

fn kv(output: &str, key: &str) -> f64 {
    output
        .lines()
        .find_map(|l| l.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("missing {key} in:\n{output}"))
        .trim()
        .parse()
        .unwrap()
}

#[test]
fn chain_multi_breakpoint_resolves_cascade_strikes_at_48k() {
    let diag_on = "\
    println!(\"fire_count={}\", state.diag_subsample_fire_count);
    println!(\"abandon={}\", state.diag_subsample_fire_abandon_count);
    println!(\"detected={}\", state.diag_subsample_fire_detected);
    println!(\"resolved={}\", state.diag_subsample_fire_resolved);";
    let on_code = nodal_code(chain_deck(), 48000.0, SubsampleFireMode::On);
    let off_code = nodal_code(chain_deck(), 48000.0, SubsampleFireMode::Off);
    let on = compile_and_run(&on_code, &CHAIN_MAIN.replace("DIAG_LINES", diag_on), "chain_on");
    let off = compile_and_run(&off_code, &CHAIN_MAIN.replace("DIAG_LINES", ""), "chain_off");
    eprintln!("SUBSAMPLE FIRE chain on:\n{on}\noff:\n{off}");

    for out in [&on, &off] {
        assert_eq!(kv(out, "nan_reset"), 0.0);
        assert_eq!(kv(out, "magnitude_reset"), 0.0);
        assert!(kv(out, "maxabs") < 250.0, "node voltage ran away: {out}");
    }
    let fire_count = kv(&on, "fire_count");
    let abandon = kv(&on, "abandon");
    let detected = kv(&on, "detected");
    let resolved = kv(&on, "resolved");
    // Two lamps at ~120 Hz / ~60 Hz over 0.5 s: at least ~80 strikes.
    assert!(fire_count > 50.0, "too few split samples: {fire_count}");
    assert_eq!(abandon, 0.0, "sub-sample splits were abandoned");
    assert!(detected >= resolved, "resolved ({resolved}) exceeds detected ({detected})");
    assert!(
        resolved >= 0.95 * detected,
        "fewer than 95% of strikes were sub-sample resolved: {resolved}/{detected}"
    );
    // Multi-breakpoint proof: the cascade puts BOTH lamps' strikes in one inner
    // sample on some cycles, so there must be samples with 2 breakpoints.
    assert!(
        resolved > fire_count,
        "no sample carried two breakpoints (resolved={resolved}, samples={fire_count}); \
         the cascade strike is not being re-detected on the rest segment"
    );
    assert!(
        (kv(&on, "acc") - kv(&off, "acc")).abs() > 1e-6,
        "on and off produced identical output — the re-solve is not taking effect"
    );
}
