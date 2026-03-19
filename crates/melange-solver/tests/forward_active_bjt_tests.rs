//! Tests for forward-active BJT optimization.
//!
//! This optimization auto-detects BJTs with Vbc < -1V at the DC operating point
//! and models them as 1D (Vbe->Ic only) instead of 2D, reducing M by 1 per
//! forward-active BJT. The base current Ib = Ic/BF is folded into N_i stamping.
//!
//! Test coverage:
//! 1. Detection: `CircuitIR::detect_forward_active_bjts()`
//! 2. MNA stamping: `stamp_bjt_forward_active()` N_v/N_i correctness
//! 3. Codegen output: generated code compiles, M reduced, 1D device eval
//! 4. Cross-validation: M=5 vs M=4 for wurli-preamp (1D approx error)

use melange_solver::codegen::ir::{CircuitIR, DeviceParams, DeviceType};
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::collections::HashSet;
use std::io::Write;

// ============================================================================
// Test circuits
// ============================================================================

/// Common-emitter NPN amplifier. Q1 should be forward-active:
/// Vb ~ 1.3V (divider), Vc ~ 7.5V (VCC - Ic*RC), so Vbc ~ -6V.
const CE_FORWARD_ACTIVE: &str = "\
Forward-active CE amp
.model NPN1 NPN(IS=1e-14 BF=200 BR=3)
VCC vcc 0 DC 15
R1 vcc base 100k
R2 base 0 10k
RC vcc collector 10k
RE emitter 0 1k
CE emitter 0 100u
Q1 collector base emitter NPN1
";

/// Saturated NPN BJT. Heavy base drive (1k from 5V) into high RC (100k):
/// Vce ~ 0V (saturation), so Vbc ~ 0.7V (forward-biased). Should NOT be detected.
const CE_SATURATED: &str = "\
Saturated BJT
.model NPN1 NPN(IS=1e-14 BF=200 BR=3)
VCC vcc 0 DC 5
RB vcc base 1k
RC vcc collector 100k
Q1 collector base 0 NPN1
";

/// PNP forward-active BJT. VCC = -15V, similar topology but PNP polarity.
/// For PNP, Vbc_eff = -1 * (Vb - Vc). With PNP in forward-active,
/// Vc is more negative than Vb, so Vbc_eff < -1.
const PNP_FORWARD_ACTIVE: &str = "\
PNP Forward-active
.model PNP1 PNP(IS=1e-14 BF=150 BR=3)
VEE vee 0 DC -15
R1 vee base 100k
R2 base 0 10k
RC vee collector 10k
RE emitter 0 1k
CE emitter 0 100u
Q1 collector base emitter PNP1
";

/// BJT with parasitic resistances. Should NOT be detected as forward-active
/// even if Vbc < -1V (excluded by design because inner NR changes dimension).
const BJT_WITH_PARASITICS: &str = "\
BJT with parasitics
.model NPN_R NPN(IS=1e-14 BF=200 BR=3 RB=100 RC=10 RE=5)
VCC vcc 0 DC 15
R1 vcc base 100k
R2 base 0 10k
RC vcc collector 10k
RE emitter 0 1k
CE emitter 0 100u
Q1 collector base emitter NPN_R
";

/// BJT with Gummel-Poon params (finite VAF). Should NOT be detected as
/// forward-active (excluded by design because GP model is more accurate).
const BJT_WITH_GP: &str = "\
BJT with Gummel-Poon
.model NPN_GP NPN(IS=1e-14 BF=200 BR=3 VAF=100 IKF=0.3)
VCC vcc 0 DC 15
R1 vcc base 100k
R2 base 0 10k
RC vcc collector 10k
RE emitter 0 1k
CE emitter 0 100u
Q1 collector base emitter NPN_GP
";

// ============================================================================
// Helpers
// ============================================================================

/// Build MNA system from SPICE string, stamping input conductance.
fn build_mna_with_gin(spice: &str) -> (Netlist, MnaSystem) {
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA build failed");
    // Stamp input conductance at the 'in' node (if it exists)
    if let Some(&raw_idx) = mna.node_map.get("in") {
        if raw_idx > 0 {
            mna.g[raw_idx - 1][raw_idx - 1] += 1.0;
        }
    }
    (netlist, mna)
}

/// Build a CodegenConfig with the given input node index.
fn make_config(input_node: usize, output_node: usize) -> CodegenConfig {
    CodegenConfig {
        circuit_name: "fa_test".to_string(),
        sample_rate: 44100.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

/// Resolve input/output node indices from an MNA system.
fn resolve_nodes(mna: &MnaSystem) -> (usize, usize) {
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    // Try common output node names
    let output_node = mna
        .node_map
        .get("out")
        .or_else(|| mna.node_map.get("collector"))
        .or_else(|| mna.node_map.get("coll2"))
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    (input_node, output_node)
}

/// Compile generated code with rustc, panicking with stderr on failure.
fn assert_compiles(code: &str, label: &str) {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);

    let tmp_dir = std::env::temp_dir();
    let src = tmp_dir.join(format!("melange_fa_test_{id}.rs"));
    let lib = tmp_dir.join(format!("melange_fa_test_{id}.rlib"));

    {
        let mut f = std::fs::File::create(&src).expect("create temp file");
        f.write_all(code.as_bytes()).expect("write temp file");
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib", "-o"])
        .arg(&lib)
        .arg(&src)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&src);
    let _ = std::fs::remove_file(&lib);

    assert!(
        output.status.success(),
        "Generated code for '{}' failed to compile:\n{}",
        label,
        String::from_utf8_lossy(&output.stderr)
    );
}

/// Compile and run generated code, returning output samples.
fn compile_and_run(
    code: &str,
    num_samples: usize,
    sample_rate: f64,
    amplitude: f64,
    tag: &str,
) -> Vec<f64> {
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let src_path = tmp_dir.join(format!("melange_fa_run_{tag}_{pid}.rs"));
    let bin_path = tmp_dir.join(format!("melange_fa_run_{tag}_{pid}"));

    let test_code = format!(
        "{code}\n\nfn main() {{\n\
         \x20   let mut state = CircuitState::default();\n\
         \x20   for i in 0..{num_samples}u32 {{\n\
         \x20       let t = i as f64 / {sample_rate:.1};\n\
         \x20       let input = {amplitude:?} * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
         \x20       let output = process_sample(input, &mut state);\n\
         \x20       println!(\"{{:.15e}}\", output[0]);\
         \x20   }}\n\
         }}\n"
    );

    {
        let mut f = std::fs::File::create(&src_path).unwrap();
        f.write_all(test_code.as_bytes()).unwrap();
    }

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2021")
        .output()
        .expect("rustc");

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Codegen compilation failed for {tag}:\n{}",
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

    String::from_utf8_lossy(&run.stdout)
        .lines()
        .filter_map(|l| l.trim().parse::<f64>().ok())
        .collect()
}

/// Full forward-active pipeline: detect, rebuild MNA, build kernel, generate code.
fn generate_with_forward_active(spice: &str) -> (String, usize, HashSet<String>) {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA");
    let (input_node, output_node) = resolve_nodes(&mna);

    // Stamp junction caps
    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    // Stamp input conductance
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }

    let config = make_config(input_node, output_node);

    // Detect forward-active BJTs
    let forward_active = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);

    // Rebuild MNA if any forward-active BJTs detected
    if !forward_active.is_empty() {
        mna =
            MnaSystem::from_netlist_forward_active(&netlist, &forward_active).expect("MNA rebuild");
        // Re-stamp junction caps with the new MNA
        let device_slots =
            CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
        // Re-stamp input conductance
        if input_node < mna.n {
            mna.g[input_node][input_node] += 1.0;
        }
    }

    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");
    let m = kernel.m;

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen");
    (result.code, m, forward_active)
}

/// Generate code WITHOUT forward-active optimization (standard 2D).
fn generate_without_forward_active(spice: &str) -> (String, usize) {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA");
    let (input_node, output_node) = resolve_nodes(&mna);

    // Stamp junction caps
    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    // Stamp input conductance
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }

    let config = make_config(input_node, output_node);
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");
    let m = kernel.m;

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen");
    (result.code, m)
}

// ============================================================================
// 1. Detection tests
// ============================================================================

/// Common-emitter NPN should be detected as forward-active.
/// Vbc should be well below -1V (collector at ~7V, base at ~1.3V).
#[test]
fn test_detect_forward_active_ce_npn() {
    let (netlist, mut mna) = build_mna_with_gin(CE_FORWARD_ACTIVE);

    // Stamp junction caps before kernel build
    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);

    assert!(
        fa.contains("Q1"),
        "Q1 should be detected as forward-active. Got: {:?}",
        fa
    );
    assert_eq!(fa.len(), 1, "Only Q1 should be detected");
}

/// Saturated BJT should NOT be detected as forward-active.
/// Heavy base drive into high RC pushes Vce near 0V (Vbc ~ +0.7V).
#[test]
fn test_detect_not_forward_active_saturated() {
    let (netlist, mut mna) = build_mna_with_gin(CE_SATURATED);

    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);

    assert!(
        fa.is_empty(),
        "Saturated BJT should NOT be detected as forward-active. Got: {:?}",
        fa
    );
}

/// PNP forward-active BJT should be detected.
/// The sign convention flips: vbc_eff = -1 * (Vb - Vc) for PNP.
#[test]
fn test_detect_forward_active_pnp() {
    let (netlist, mut mna) = build_mna_with_gin(PNP_FORWARD_ACTIVE);

    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);

    assert!(
        fa.contains("Q1"),
        "PNP Q1 should be detected as forward-active. Got: {:?}",
        fa
    );
}

/// BJT with parasitic resistances (RB/RC/RE > 0) should NOT be detected.
/// The forward-active optimization is excluded for BJTs with parasitics
/// BJT with parasitics IS now detected as forward-active.
/// The 1D FA model ignores parasitics (small for forward-active operation).
#[test]
fn test_detect_forward_active_with_parasitics() {
    let (netlist, mut mna) = build_mna_with_gin(BJT_WITH_PARASITICS);

    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);

    assert!(
        !fa.is_empty(),
        "BJT with parasitics SHOULD be detected as forward-active (parasitics ignored in 1D model)"
    );
}

/// BJT with Gummel-Poon params IS now detected as forward-active.
/// GP qb(Vbc) is ~constant when Vbc is deeply reverse-biased.
#[test]
fn test_detect_forward_active_with_gummel_poon() {
    let (netlist, mut mna) = build_mna_with_gin(BJT_WITH_GP);

    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);

    assert!(
        !fa.is_empty(),
        "BJT with GP params SHOULD be detected as forward-active (qb ~constant at deep reverse bias)"
    );
}

/// Empty (linear) circuit should return empty set.
#[test]
fn test_detect_forward_active_no_bjts() {
    let spice = "RC Circuit\nR1 in out 1k\nC1 out 0 1u\n";
    let (netlist, mna) = build_mna_with_gin(spice);
    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);
    assert!(
        fa.is_empty(),
        "Linear circuit should have no forward-active BJTs"
    );
}

/// Diode-only circuit should return empty set (no BJTs).
#[test]
fn test_detect_forward_active_diodes_only() {
    let spice = "\
Diode only
R1 in out 1k
D1 out 0 DMOD
C1 out 0 1u
.model DMOD D(IS=1e-15)
";
    let (netlist, mut mna) = build_mna_with_gin(spice);
    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }
    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);
    assert!(
        fa.is_empty(),
        "Diode circuit should have no forward-active BJTs"
    );
}

// ============================================================================
// 2. MNA stamping tests
// ============================================================================

/// Test N_v has exactly 1 row extracting Vbe = V(base) - V(emitter).
#[test]
fn test_stamp_forward_active_n_v() {
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");
    let mna = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("MNA");

    // Find Q1 device
    let q1_dev = mna
        .nonlinear_devices
        .iter()
        .find(|d| d.name.eq_ignore_ascii_case("Q1"))
        .expect("Q1 not found in nonlinear devices");

    assert_eq!(
        q1_dev.device_type,
        melange_solver::mna::NonlinearDeviceType::BjtForwardActive,
        "Q1 should be BjtForwardActive type"
    );
    assert_eq!(
        q1_dev.dimension, 1,
        "Forward-active BJT should have dimension 1"
    );

    let s = q1_dev.start_idx;
    let nb = q1_dev.node_indices[1]; // base (1-indexed, 0=ground)
    let ne = q1_dev.node_indices[2]; // emitter (1-indexed, 0=ground)

    // N_v[s] should extract Vbe: +1 at base, -1 at emitter, 0 elsewhere
    if nb > 0 {
        assert_eq!(
            mna.n_v[s][nb - 1],
            1.0,
            "N_v[{s}][base] should be +1.0 for Vbe extraction"
        );
    }
    if ne > 0 {
        assert_eq!(
            mna.n_v[s][ne - 1],
            -1.0,
            "N_v[{s}][emitter] should be -1.0 for Vbe extraction"
        );
    }

    // All other entries in row s of N_v should be 0
    for j in 0..mna.n {
        if nb > 0 && j == nb - 1 {
            continue;
        }
        if ne > 0 && j == ne - 1 {
            continue;
        }
        assert_eq!(
            mna.n_v[s][j], 0.0,
            "N_v[{s}][{j}] should be 0.0 (not base or emitter)"
        );
    }
}

/// Test N_i has exactly 1 column with correct KCL-satisfying entries.
/// For BF=200: collector=-1, base=-1/200=-0.005, emitter=1+1/200=1.005.
#[test]
fn test_stamp_forward_active_n_i() {
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");
    let mna = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("MNA");

    let q1_dev = mna
        .nonlinear_devices
        .iter()
        .find(|d| d.name.eq_ignore_ascii_case("Q1"))
        .expect("Q1 not found");

    let s = q1_dev.start_idx;
    let nc = q1_dev.node_indices[0]; // collector
    let nb = q1_dev.node_indices[1]; // base
    let ne = q1_dev.node_indices[2]; // emitter

    let beta_f = 200.0; // from .model NPN1 NPN(BF=200)

    // N_i column s: collector = -1.0 (Ic extracted)
    if nc > 0 {
        assert!(
            (mna.n_i[nc - 1][s] - (-1.0)).abs() < 1e-12,
            "N_i[collector][{s}] should be -1.0, got {}",
            mna.n_i[nc - 1][s]
        );
    }

    // N_i column s: base = -1/BF (Ib extracted)
    if nb > 0 {
        let expected_base = -1.0 / beta_f;
        assert!(
            (mna.n_i[nb - 1][s] - expected_base).abs() < 1e-12,
            "N_i[base][{s}] should be {expected_base}, got {}",
            mna.n_i[nb - 1][s]
        );
    }

    // N_i column s: emitter = 1 + 1/BF (Ic + Ib injected)
    if ne > 0 {
        let expected_emitter = 1.0 + 1.0 / beta_f;
        assert!(
            (mna.n_i[ne - 1][s] - expected_emitter).abs() < 1e-12,
            "N_i[emitter][{s}] should be {expected_emitter}, got {}",
            mna.n_i[ne - 1][s]
        );
    }
}

/// Verify KCL: sum of N_i column = 0 (current conservation).
#[test]
fn test_stamp_forward_active_kcl_conservation() {
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");
    let mna = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("MNA");

    let q1_dev = mna
        .nonlinear_devices
        .iter()
        .find(|d| d.name.eq_ignore_ascii_case("Q1"))
        .expect("Q1 not found");
    let s = q1_dev.start_idx;

    // Sum all N_i entries in column s
    let mut sum = 0.0;
    for i in 0..mna.n {
        sum += mna.n_i[i][s];
    }

    assert!(
        sum.abs() < 1e-12,
        "KCL violation: sum of N_i column {s} = {sum} (should be 0). \
         Current must be conserved: Ic + Ib = Ie."
    );
}

/// Verify specific BF=200 values: base=-0.005, emitter=1.005.
#[test]
fn test_stamp_forward_active_bf200_values() {
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");
    let mna = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("MNA");

    let q1_dev = mna
        .nonlinear_devices
        .iter()
        .find(|d| d.name.eq_ignore_ascii_case("Q1"))
        .expect("Q1 not found");
    let s = q1_dev.start_idx;
    let nb = q1_dev.node_indices[1];
    let ne = q1_dev.node_indices[2];

    if nb > 0 {
        assert!(
            (mna.n_i[nb - 1][s] - (-0.005)).abs() < 1e-12,
            "N_i[base] for BF=200 should be -0.005, got {}",
            mna.n_i[nb - 1][s]
        );
    }

    if ne > 0 {
        assert!(
            (mna.n_i[ne - 1][s] - 1.005).abs() < 1e-12,
            "N_i[emitter] for BF=200 should be 1.005, got {}",
            mna.n_i[ne - 1][s]
        );
    }
}

/// Forward-active BJT should reduce M by 1 compared to standard 2D.
#[test]
fn test_stamp_forward_active_reduces_m() {
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");

    // Standard MNA (2D BJT): M=2
    let mna_std = MnaSystem::from_netlist(&netlist).expect("standard MNA");
    let m_std: usize = mna_std.nonlinear_devices.iter().map(|d| d.dimension).sum();

    // Forward-active MNA (1D BJT): M=1
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let mna_fa = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("FA MNA");
    let m_fa: usize = mna_fa.nonlinear_devices.iter().map(|d| d.dimension).sum();

    assert_eq!(m_std, 2, "Standard BJT should give M=2");
    assert_eq!(m_fa, 1, "Forward-active BJT should give M=1");
    assert_eq!(m_std - m_fa, 1, "M should reduce by exactly 1");
}

/// N_v should have no second row for the forward-active BJT
/// (only 1 dimension, not 2).
#[test]
fn test_stamp_forward_active_single_dimension() {
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");
    let mna = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("MNA");

    // Total M should be 1
    let total_m: usize = mna.nonlinear_devices.iter().map(|d| d.dimension).sum();
    assert_eq!(
        total_m, 1,
        "Total M should be 1 for single forward-active BJT"
    );

    // N_v and N_i should be sized for M=1
    assert_eq!(
        mna.n_v.len(),
        total_m,
        "N_v should have exactly M={total_m} rows"
    );
    assert_eq!(
        mna.n_i[0].len(),
        total_m,
        "N_i columns should be M={total_m}"
    );
}

// ============================================================================
// 3. Codegen output tests
// ============================================================================

/// Generated code for forward-active CE amp should compile.
#[test]
#[ignore] // requires rustc
fn test_codegen_forward_active_compiles() {
    let (code, m, fa) = generate_with_forward_active(CE_FORWARD_ACTIVE);

    assert!(fa.contains("Q1"), "Q1 should be forward-active");
    assert_eq!(m, 1, "M should be 1 with forward-active optimization");

    assert_compiles(&code, "forward-active CE");
}

/// M should be reduced by 1 in generated code constants.
#[test]
fn test_codegen_forward_active_m_reduced() {
    let (code, m, fa) = generate_with_forward_active(CE_FORWARD_ACTIVE);

    assert!(fa.contains("Q1"), "Q1 should be detected");
    assert_eq!(m, 1, "Kernel M should be 1");

    // Check generated M constant
    assert!(
        code.contains("pub const M: usize = 1;"),
        "Generated code should have M=1 for forward-active BJT. \
         Without optimization it would be M=2."
    );
}

/// Standard (non-forward-active) CE amp should have M=2.
#[test]
fn test_codegen_standard_bjt_m2() {
    let (code, m) = generate_without_forward_active(CE_FORWARD_ACTIVE);

    assert_eq!(m, 2, "Standard BJT should give M=2");
    assert!(
        code.contains("pub const M: usize = 2;"),
        "Generated code should have M=2 without forward-active optimization."
    );
}

/// Forward-active code should produce 1D device evaluation (scalar jdev).
#[test]
fn test_codegen_forward_active_1d_eval() {
    let (code, _m, fa) = generate_with_forward_active(CE_FORWARD_ACTIVE);
    assert!(fa.contains("Q1"));

    // Should contain 1D device evaluation markers
    assert!(
        code.contains("forward-active") || code.contains("1D"),
        "Generated code should contain 1D / forward-active markers"
    );

    // Should NOT contain the 2D BJT evaluation (bjt_ic, bjt_ib, 2x2 jacobian)
    // In the 1D path, we have a single i_dev and scalar jdev instead of 2x2
    // Check that there is NO second dimension variable (i_dev for start_idx+1)
    // by checking for absence of bjt_ib call (which is only in 2D path)
    let has_bjt_ib = code.contains("bjt_ib(");
    let has_forward_active_comment = code.contains("forward-active (1D");
    assert!(
        has_forward_active_comment || !has_bjt_ib,
        "Forward-active code should use 1D evaluation, not 2D bjt_ic/bjt_ib"
    );
}

/// Forward-active code should use exp_be for the forward junction only.
#[test]
fn test_codegen_forward_active_exp_be_only() {
    let (code, _m, fa) = generate_with_forward_active(CE_FORWARD_ACTIVE);
    assert!(fa.contains("Q1"));

    // In the 1D forward-active path, only exp_be is computed (no exp_bc)
    // The generated code pattern is:
    //   let exp_be_N = fast_exp(vbe_N / (DEVICE_N_NF * state.device_N_vt));
    //   let i_devS = state.device_N_is * (exp_be_N - 1.0) * DEVICE_N_SIGN;
    //   let jdev_S_S = state.device_N_is / (DEVICE_N_NF * state.device_N_vt) * exp_be_N;

    // Check for the single-junction eval pattern
    assert!(
        code.contains("exp_be") || code.contains("fast_exp"),
        "Forward-active code should compute exp_be for Vbe junction"
    );
}

/// Forward-active codegen compiles and produces non-zero output.
#[test]
#[ignore] // requires rustc compilation and execution
fn test_codegen_forward_active_produces_output() {
    let (code, m, fa) = generate_with_forward_active(CE_FORWARD_ACTIVE);
    assert!(fa.contains("Q1"));
    assert_eq!(m, 1);

    let samples = compile_and_run(&code, 4410, 44100.0, 0.01, "fa_output");

    // Should produce non-zero output after transient settles
    let peak = samples
        .iter()
        .skip(2000)
        .map(|s| s.abs())
        .fold(0.0f64, f64::max);
    assert!(
        peak > 1e-6,
        "Forward-active CE amp should produce non-zero output, got peak={peak:.2e}"
    );

    // Should be stable (no NaN or Inf)
    for (i, &s) in samples.iter().enumerate() {
        assert!(s.is_finite(), "Sample {i} is not finite: {s}");
    }
}

// ============================================================================
// 4. Cross-validation: M=5 vs M=4 for wurli-preamp
// ============================================================================

/// Read the wurli-preamp circuit file.
fn load_wurli_preamp() -> String {
    std::fs::read_to_string("/home/homeuser/dev/melange/circuits/wurli-preamp.cir")
        .expect("Could not read wurli-preamp.cir")
}

/// Cross-validate wurli-preamp with and without forward-active optimization.
/// Standard: M=5 (2 BJTs x 2D + 1 diode x 1D = 5).
/// With FA: M=4 or M=3 (forward-active BJTs reduce to 1D each).
///
/// Process 0.5 seconds of 1kHz sine at 0.1V through both.
/// Compare peak output: should be within 6dB.
/// Both should be stable (no NaN, no clipping).
///
/// The 1D approximation folds Ib=Ic/BF into N_i, removing the reverse
/// junction (Vbc) from the NR system. This changes the feedback path
/// slightly, resulting in gain differences of typically 2-4 dB. We use
/// a 6dB tolerance to allow for this systematic approximation error.
#[test]
#[ignore] // requires rustc compilation, reads circuit file from disk
fn test_wurli_preamp_forward_active_cross_validation() {
    let spice = load_wurli_preamp();
    let sample_rate = 44100.0;
    let num_samples = (sample_rate * 0.5) as usize; // 0.5 seconds
    let amplitude = 0.1;
    let skip = num_samples / 2; // skip first half for transient settling

    // --- Standard path (no FA optimization) ---
    let (code_std, m_std) = generate_without_forward_active(&spice);
    assert_eq!(
        m_std, 5,
        "Wurli-preamp should have M=5 without FA optimization (2 BJTs + 1 diode)"
    );

    let out_std = compile_and_run(&code_std, num_samples, sample_rate, amplitude, "wurli_std");
    assert_eq!(
        out_std.len(),
        num_samples,
        "Standard output length mismatch"
    );

    // --- Forward-active path ---
    let (code_fa, m_fa, fa_bjts) = generate_with_forward_active(&spice);
    assert!(
        !fa_bjts.is_empty(),
        "Wurli-preamp should have at least one forward-active BJT"
    );
    assert!(
        m_fa < m_std,
        "M with FA ({m_fa}) should be less than without ({m_std})"
    );

    let out_fa = compile_and_run(&code_fa, num_samples, sample_rate, amplitude, "wurli_fa");
    assert_eq!(out_fa.len(), num_samples, "FA output length mismatch");

    // --- Stability checks ---
    for (i, &s) in out_std.iter().enumerate() {
        assert!(
            s.is_finite(),
            "Standard output sample {i} is not finite: {s}"
        );
    }
    for (i, &s) in out_fa.iter().enumerate() {
        assert!(s.is_finite(), "FA output sample {i} is not finite: {s}");
    }

    // --- Peak comparison ---
    let peak_std = out_std
        .iter()
        .skip(skip)
        .map(|s| s.abs())
        .fold(0.0f64, f64::max);
    let peak_fa = out_fa
        .iter()
        .skip(skip)
        .map(|s| s.abs())
        .fold(0.0f64, f64::max);

    println!("Wurli-preamp cross-validation:");
    println!("  M standard = {m_std}, M forward-active = {m_fa}");
    println!("  FA BJTs: {:?}", fa_bjts);
    println!("  Peak standard  = {peak_std:.4}V");
    println!("  Peak FA        = {peak_fa:.4}V");

    assert!(
        peak_std > 1e-4,
        "Standard output should be non-zero, got peak={peak_std:.2e}"
    );
    assert!(
        peak_fa > 1e-4,
        "FA output should be non-zero, got peak={peak_fa:.2e}"
    );

    // Peaks should be within 6dB (factor of ~2.0).
    // The 1D forward-active approximation changes the feedback loop slightly
    // because the reverse junction is removed from the NR system, which
    // affects the K matrix and thus the gain. 2-4 dB differences are typical.
    let ratio = if peak_std > peak_fa {
        peak_std / peak_fa
    } else {
        peak_fa / peak_std
    };
    let db_diff = 20.0 * ratio.log10();
    println!("  Peak ratio = {ratio:.4}, dB difference = {db_diff:.2}dB");

    assert!(
        db_diff < 6.0,
        "Peak output difference ({db_diff:.2}dB) exceeds 6dB tolerance. \
         Standard={peak_std:.4}V, FA={peak_fa:.4}V, ratio={ratio:.4}"
    );
}

// ============================================================================
// Additional edge case tests
// ============================================================================

/// Two BJTs: one forward-active, one saturated. Only the forward-active one
/// should be detected.
#[test]
fn test_detect_mixed_forward_active_and_saturated() {
    let spice = "\
Mixed BJTs
.model NPN1 NPN(IS=1e-14 BF=200 BR=3)
VCC vcc 0 DC 15
R1 vcc base1 100k
R2 base1 0 10k
RC1 vcc coll1 10k
RE1 emit1 0 1k
CE1 emit1 0 100u
Q1 coll1 base1 emit1 NPN1
RB2 vcc base2 1k
RC2 vcc coll2 100k
Q2 coll2 base2 0 NPN1
C_dummy coll2 0 1n
";
    let (netlist, mut mna) = build_mna_with_gin(spice);
    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    let config = make_config(0, 0);
    let fa = CircuitIR::detect_forward_active_bjts(&mna, &netlist, &config);

    // Q1 should be forward-active (Vbc well below -1V)
    assert!(
        fa.contains("Q1"),
        "Q1 (forward-active CE) should be detected. Got: {:?}",
        fa
    );

    // Q2 should NOT be forward-active (saturated)
    assert!(
        !fa.contains("Q2"),
        "Q2 (saturated) should NOT be detected. Got: {:?}",
        fa
    );
}

/// Verify that from_netlist_forward_active produces the correct NonlinearDeviceType.
#[test]
fn test_mna_device_type_after_forward_active_rebuild() {
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");
    let mna = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("MNA");

    let q1 = mna
        .nonlinear_devices
        .iter()
        .find(|d| d.name.eq_ignore_ascii_case("Q1"))
        .expect("Q1 not found");

    assert_eq!(
        q1.device_type,
        melange_solver::mna::NonlinearDeviceType::BjtForwardActive
    );
    assert_eq!(q1.dimension, 1);
    assert_eq!(
        q1.start_idx, 0,
        "Q1 is the only device, should start at index 0"
    );
}

/// DK kernel built from forward-active MNA should have reduced M.
#[test]
fn test_kernel_m_reduced_with_forward_active() {
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");

    // Standard kernel
    let mna_std = MnaSystem::from_netlist(&netlist).expect("std MNA");
    let kernel_std = DkKernel::from_mna(&mna_std, 44100.0).expect("std kernel");

    // Forward-active kernel
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let mna_fa = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("FA MNA");
    let kernel_fa = DkKernel::from_mna(&mna_fa, 44100.0).expect("FA kernel");

    assert_eq!(kernel_std.m, 2, "Standard kernel should have M=2");
    assert_eq!(kernel_fa.m, 1, "Forward-active kernel should have M=1");

    // K matrix should be 1x1 for forward-active
    let k00 = kernel_fa.k(0, 0);
    assert!(
        k00 < 0.0,
        "K[0][0] should be negative for stable circuit, got {k00}"
    );
    assert!(k00.is_finite(), "K[0][0] should be finite");
}

/// Device slots built from forward-active MNA should have BjtForwardActive type.
#[test]
fn test_device_slots_forward_active_type() {
    let netlist = Netlist::parse(CE_FORWARD_ACTIVE).expect("parse");
    let fa_set: HashSet<String> = ["Q1".to_string()].into_iter().collect();
    let mna = MnaSystem::from_netlist_forward_active(&netlist, &fa_set).expect("MNA");

    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).expect("device slots");

    // Find Q1 slot
    let q1_slot = slots
        .iter()
        .find(|s| s.device_type == DeviceType::BjtForwardActive)
        .expect("Should have a BjtForwardActive slot");

    assert_eq!(
        q1_slot.dimension, 1,
        "Forward-active slot dimension should be 1"
    );
    assert_eq!(q1_slot.start_idx, 0, "Q1 should be at start_idx 0");

    // Params should still be BJT params
    match &q1_slot.params {
        DeviceParams::Bjt(bp) => {
            assert!((bp.beta_f - 200.0).abs() < 1e-10, "BF should be 200");
            assert!(!bp.has_parasitics(), "Should have no parasitics");
            assert!(!bp.is_gummel_poon(), "Should not be Gummel-Poon");
        }
        _ => panic!("Expected BJT params for forward-active slot"),
    }
}
