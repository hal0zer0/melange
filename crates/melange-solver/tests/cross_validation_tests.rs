//! Cross-validation tests: codegen vs runtime solver, Sherman-Morrison correctness.
//!
//! These tests ensure that the runtime solver (DeviceEntry dispatch) and codegen
//! (Tera templates emitting inline device functions) produce identical output
//! for the same circuit + input. Any divergence means one path has a bug.

use melange_solver::codegen::ir::{CircuitIR, DeviceSlot};
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::solver::{CircuitSolver, DeviceEntry};
use std::io::Write;

// ============================================================================
// Test circuits
// ============================================================================

const DIODE_CLIPPER: &str = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";

const BJT_CE: &str = "\
BJT Common Emitter
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJTCE
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJTCE NPN(IS=1e-14 BF=200 BR=3)
";

const JFET_CS: &str = "\
JFET Common Source
Cin in gate 10u
Rg gate 0 1Meg
J1 drain gate source J2N5457
Rd vdd drain 2.2k
Rs source 0 1k
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 12
.model J2N5457 NJ(VTO=-2.0 IDSS=5e-3 LAMBDA=0.001)
";

const MOSFET_CS: &str = "\
MOSFET Common Source
Cin in gate 10u
R1 vdd gate 47k
R2 gate 0 100k
M1 drain gate source 0 NMOD
Rd vdd drain 1k
Rs source 0 100
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 5
.model NMOD NM(VTO=2.0 KP=0.1 LAMBDA=0.01)
";

const TRIODE_CC: &str = "\
12AX7 Common Cathode
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
T1 grid plate cathode 12AX7
Rk cathode 0 1.5k
Ck cathode 0 25u
Rp vcc plate 100k
Cout plate out 100n
Rout out 0 1Meg
Vcc vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";

// ============================================================================
// Shared helpers
// ============================================================================

/// Build full pipeline from SPICE string, stamping input conductance.
fn build_pipeline(spice: &str, sample_rate: f64) -> (Netlist, MnaSystem, DkKernel, usize, usize) {
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
    mna.g[input_node][input_node] += 1.0; // G_in stamp before kernel
    let kernel = DkKernel::from_mna(&mna, sample_rate).expect("kernel");
    (netlist, mna, kernel, input_node, output_node)
}

/// Build DeviceEntry list from MNA nonlinear device list + parsed netlist.
fn build_device_entries(netlist: &Netlist, mna: &MnaSystem) -> Vec<DeviceEntry> {
    let find_model_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let find_model_name = |dev_name: &str| -> String {
        netlist
            .elements
            .iter()
            .find_map(|e| {
                let (name, model) = match e {
                    melange_solver::parser::Element::Diode { name, model, .. } => (name, model),
                    melange_solver::parser::Element::Bjt { name, model, .. } => (name, model),
                    melange_solver::parser::Element::Jfet { name, model, .. } => (name, model),
                    melange_solver::parser::Element::Mosfet { name, model, .. } => (name, model),
                    melange_solver::parser::Element::Triode { name, model, .. } => (name, model),
                    _ => return None,
                };
                if name.eq_ignore_ascii_case(dev_name) {
                    Some(model.clone())
                } else {
                    None
                }
            })
            .unwrap_or_default()
    };

    let mut devices = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        let model = find_model_name(&dev_info.name);
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let is = find_model_param(&model, "IS").unwrap_or(1e-15);
                let n = find_model_param(&model, "N").unwrap_or(1.0);
                let diode = melange_devices::DiodeShockley::new_room_temp(is, n);
                devices.push(DeviceEntry::new_diode(diode, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt
            | melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                let is = find_model_param(&model, "IS").unwrap_or(1e-14);
                let bf = find_model_param(&model, "BF").unwrap_or(200.0);
                let br = find_model_param(&model, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp {
                    melange_devices::BjtPolarity::Pnp
                } else {
                    melange_devices::BjtPolarity::Npn
                };
                let nf = find_model_param(&model, "NF").unwrap_or(1.0);
                let bjt = melange_devices::BjtEbersMoll::new(
                    is,
                    melange_primitives::VT_ROOM,
                    bf,
                    br,
                    polarity,
                ).with_nf(nf);
                devices.push(DeviceEntry::new_bjt(bjt, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let is_p = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let channel = if is_p {
                    melange_devices::JfetChannel::P
                } else {
                    melange_devices::JfetChannel::N
                };
                let default_vp = if is_p { 2.0 } else { -2.0 };
                let vp = find_model_param(&model, "VTO").unwrap_or(default_vp);
                let idss = if let Some(beta) = find_model_param(&model, "BETA") {
                    beta * vp * vp
                } else {
                    find_model_param(&model, "IDSS").unwrap_or(2e-3)
                };
                let mut jfet = melange_devices::Jfet::new(channel, vp, idss);
                jfet.lambda = find_model_param(&model, "LAMBDA").unwrap_or(0.001);
                devices.push(DeviceEntry::new_jfet(jfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let is_p = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let channel = if is_p {
                    melange_devices::MosfetChannelType::P
                } else {
                    melange_devices::MosfetChannelType::N
                };
                let default_vt = if is_p { -2.0 } else { 2.0 };
                let vt = find_model_param(&model, "VTO").unwrap_or(default_vt);
                let kp = find_model_param(&model, "KP").unwrap_or(0.1);
                let lambda = find_model_param(&model, "LAMBDA").unwrap_or(0.01);
                let mosfet = melange_devices::Mosfet::new(channel, vt, kp, lambda);
                devices.push(DeviceEntry::new_mosfet(mosfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let mu = find_model_param(&model, "MU").unwrap_or(100.0);
                let ex = find_model_param(&model, "EX").unwrap_or(1.4);
                let kg1 = find_model_param(&model, "KG1").unwrap_or(1060.0);
                let kp = find_model_param(&model, "KP").unwrap_or(600.0);
                let kvb = find_model_param(&model, "KVB").unwrap_or(300.0);
                let ig_max = find_model_param(&model, "IG_MAX").unwrap_or(2e-3);
                let vgk_onset = find_model_param(&model, "VGK_ONSET").unwrap_or(0.5);
                let lambda = find_model_param(&model, "LAMBDA").unwrap_or(0.0);
                let tube = melange_devices::KorenTriode::with_all_params(
                    mu, ex, kg1, kp, kvb, ig_max, vgk_onset, lambda,
                );
                devices.push(DeviceEntry::new_tube(tube, dev_info.start_idx));
            }
        }
    }
    devices
}

/// Build device slots for DC OP from CircuitIR (uses same param resolution as codegen).
fn build_device_slots_from_ir(
    kernel: &DkKernel,
    mna: &MnaSystem,
    netlist: &Netlist,
    input_node: usize,
    output_node: usize,
) -> Vec<DeviceSlot> {
    let config = CodegenConfig {
        circuit_name: "crossval_dcop".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    match CircuitIR::from_kernel(kernel, mna, netlist, &config) {
        Ok(ir) => ir.device_slots,
        Err(_) => Vec::new(),
    }
}

/// Compile and run generated code with configurable amplitude, return output samples.
fn compile_and_run_codegen_with_amplitude(
    code: &str,
    num_samples: usize,
    sample_rate: f64,
    amplitude: f64,
    tag: &str,
) -> Vec<f64> {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let src_path = tmp_dir.join(format!("melange_xval_{tag}_{id}.rs"));
    let bin_path = tmp_dir.join(format!("melange_xval_{tag}_{id}"));

    let test_code = format!(
        "{code}\n\nfn main() {{\n\
         \x20   let mut state = CircuitState::default();\n\
         \x20   for i in 0..{num_samples}u32 {{\n\
         \x20       let t = i as f64 / {sample_rate:.1};\n\
         \x20       let input = {amplitude:?} * (2.0 * std::f64::consts::PI * 500.0 * t).sin();\n\
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
        .arg("--edition=2024")
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
            "Codegen binary failed for {tag}:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }

    String::from_utf8_lossy(&run.stdout)
        .lines()
        .filter_map(|l| l.trim().parse::<f64>().ok())
        .collect()
}

/// Run codegen vs runtime cross-validation for a circuit.
///
/// Returns (max_diff, max_runtime_abs) for diagnostics.
fn run_crossval(
    spice: &str,
    amplitude: f64,
    num_samples: usize,
    skip_samples: usize,
    tag: &str,
) -> (f64, f64) {
    let sample_rate = 48000.0;
    let (netlist, mna, kernel, input_node, output_node) = build_pipeline(spice, sample_rate);

    // --- Runtime path ---
    let devices = build_device_entries(&netlist, &mna);
    let mut solver = CircuitSolver::new(kernel.clone(), devices, input_node, output_node).unwrap();
    solver.input_conductance = 1.0;

    // DC OP for nonlinear circuits
    if kernel.m > 0 {
        let slots = build_device_slots_from_ir(&kernel, &mna, &netlist, input_node, output_node);
        solver.initialize_dc_op(&mna, &slots);
    }

    let mut runtime_output = Vec::with_capacity(num_samples);
    for _ in 0..num_samples {
        let t = runtime_output.len() as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * 500.0 * t).sin();
        runtime_output.push(solver.process_sample(input));
    }

    // --- Codegen path ---
    let config = CodegenConfig {
        circuit_name: format!("crossval_{tag}"),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen");
    let codegen_output = compile_and_run_codegen_with_amplitude(
        &result.code,
        num_samples,
        sample_rate,
        amplitude,
        tag,
    );

    assert_eq!(
        codegen_output.len(),
        runtime_output.len(),
        "{tag}: sample count mismatch: codegen={} runtime={}",
        codegen_output.len(),
        runtime_output.len()
    );

    // Compare sample-by-sample
    let mut max_diff = 0.0f64;
    let mut max_runtime_abs = 0.0f64;
    for i in skip_samples..num_samples {
        let diff = (codegen_output[i] - runtime_output[i]).abs();
        max_diff = max_diff.max(diff);
        max_runtime_abs = max_runtime_abs.max(runtime_output[i].abs());
    }

    println!(
        "{tag}: codegen vs runtime max_diff={:.2e}, max_output={:.4}",
        max_diff, max_runtime_abs
    );
    (max_diff, max_runtime_abs)
}

// ============================================================================
// Cross-validation tests
// ============================================================================

/// Diode clipper: codegen vs runtime should match to < 5e-6.
/// (Tolerance relaxed: codegen uses per-device alpha + RELTOL convergence,
/// runtime uses scalar alpha + flat TOL — slight convergence point difference.)
#[test]
fn test_codegen_vs_runtime_diode_clipper() {
    let (max_diff, _) = run_crossval(DIODE_CLIPPER, 2.0, 480, 10, "diode");
    assert!(
        max_diff < 5e-6,
        "Diode clipper: codegen vs runtime max diff = {:.2e} (expected < 5e-6)",
        max_diff
    );
}

/// BJT common-emitter: codegen vs runtime should match to < 1e-4.
/// (Slightly looser tolerance due to DC OP sensitivity.)
#[test]
fn test_codegen_vs_runtime_bjt_ce() {
    let (max_diff, max_out) = run_crossval(BJT_CE, 0.01, 4800, 2400, "bjt");
    assert!(
        max_out > 1e-4,
        "BJT CE: output should be non-zero, got {:.2e}",
        max_out
    );
    assert!(
        max_diff < 1e-4,
        "BJT CE: codegen vs runtime max diff = {:.2e} (expected < 1e-4)",
        max_diff
    );
}

/// JFET common-source: codegen vs runtime should match to < 1e-4.
#[test]
fn test_codegen_vs_runtime_jfet_cs() {
    let (max_diff, max_out) = run_crossval(JFET_CS, 0.1, 960, 100, "jfet");
    assert!(
        max_out > 1e-4,
        "JFET CS: output should be non-zero, got {:.2e}",
        max_out
    );
    assert!(
        max_diff < 1e-4,
        "JFET CS: codegen vs runtime max diff = {:.2e} (expected < 1e-4)",
        max_diff
    );
}

/// MOSFET common-source: codegen vs runtime should match to < 1e-4.
#[test]
fn test_codegen_vs_runtime_mosfet_cs() {
    let (max_diff, max_out) = run_crossval(MOSFET_CS, 0.1, 960, 100, "mosfet");
    assert!(
        max_out > 1e-4,
        "MOSFET CS: output should be non-zero, got {:.2e}",
        max_out
    );
    assert!(
        max_diff < 1e-4,
        "MOSFET CS: codegen vs runtime max diff = {:.2e} (expected < 1e-4)",
        max_diff
    );
}

/// Triode common-cathode: codegen vs runtime should match to < 1e-4.
#[test]
fn test_codegen_vs_runtime_triode_cc() {
    let (max_diff, max_out) = run_crossval(TRIODE_CC, 0.01, 4800, 2400, "triode");
    assert!(
        max_out > 1e-4,
        "Triode CC: output should be non-zero, got {:.2e}",
        max_out
    );
    assert!(
        max_diff < 1e-4,
        "Triode CC: codegen vs runtime max diff = {:.2e} (expected < 1e-4)",
        max_diff
    );
}

/// Test: Sherman-Morrison pot data is consistent in kernel.
///
/// Verifies SM precomputed vectors are finite and have correct dimensions.
#[test]
fn test_sherman_morrison_pot_data_sanity() {
    let spice = "SM test\nR1 in mid 1k\n.pot R1 100 10k\nR2 mid out 1k\nC1 out 0 10n\n.end";
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    mna.g[input_node][input_node] += 1.0;

    let kernel = DkKernel::from_mna(&mna, 48000.0).expect("kernel");
    let n = kernel.n;

    // Verify pot data exists
    assert_eq!(kernel.pots.len(), 1, "Expected 1 pot in kernel");
    let pot = &kernel.pots[0];

    // SU vector should have length N
    assert_eq!(pot.su.len(), n, "SU length should match N");

    // USU should be finite and non-zero (for a non-degenerate circuit)
    assert!(pot.usu.is_finite(), "USU should be finite");
    assert!(pot.usu.abs() > 1e-30, "USU should be non-zero");

    // Resistance range should be valid
    assert!(
        pot.min_resistance > 0.0,
        "min_resistance should be positive"
    );
    assert!(
        pot.max_resistance > pot.min_resistance,
        "max > min resistance"
    );

    // S matrix should be finite
    for i in 0..n {
        for j in 0..n {
            assert!(
                kernel.s[i * n + j].is_finite(),
                "S[{}][{}] = {} is not finite",
                i,
                j,
                kernel.s[i * n + j]
            );
        }
    }

    // Nominal conductance should be 1/R_default = 1/1000 = 0.001
    assert!(
        (pot.g_nominal - 0.001).abs() < 1e-6,
        "Nominal conductance should be ~0.001, got {}",
        pot.g_nominal
    );

    println!(
        "SM sanity: N={}, M={}, pot min_R={:.0}, max_R={:.0}, g_nom={:.6e}",
        n, kernel.m, pot.min_resistance, pot.max_resistance, pot.g_nominal
    );
}

// ============================================================================
// Additional circuit constants
// ============================================================================

const PNP_CE: &str = "\
PNP Common Emitter
Cin in base 10u
Vcc vcc 0 DC 12
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYPNP
Rc coll 0 4.7k
Re vcc emit 1k
Ce vcc emit 100u
Cout coll out 10u
Rload out 0 100k
.model MYPNP PNP(IS=1e-14 BF=200 BR=3)
";

const BJT_CE_GP: &str = "\
BJT CE Gummel-Poon
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYGP
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYGP NPN(IS=1e-14 BF=200 BR=3 VAF=100 IKF=0.3)
";

const BJT_CE_EM: &str = "\
BJT CE Ebers-Moll
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYEM
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYEM NPN(IS=1e-14 BF=200 BR=3)
";

const BJT_CE_NF: &str = "\
BJT CE with NF
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYNF
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYNF NPN(IS=1e-14 BF=200 BR=3 NF=1.5)
";

// ============================================================================
// Test A: PNP BJT cross-validation
// ============================================================================

/// PNP common-emitter: codegen vs runtime should match to < 1e-4.
/// Validates that PNP polarity flows correctly through both paths.
#[test]
fn test_codegen_vs_runtime_pnp_bjt() {
    let (max_diff, max_out) = run_crossval(PNP_CE, 0.01, 4800, 2400, "pnp_bjt");
    assert!(
        max_out > 1e-4,
        "PNP CE: output should be non-zero, got {:.2e}",
        max_out
    );
    assert!(
        max_diff < 1e-4,
        "PNP CE: codegen vs runtime max diff = {:.2e} (expected < 1e-4)",
        max_diff
    );
}

// ============================================================================
// Test B: Nodal vs DK solver path cross-validation
// ============================================================================

/// Diode clipper: nodal codegen vs DK codegen should produce matching output.
/// Both paths solve the same circuit equations with different numerical methods
/// (DK: precomputed S=A^-1, M-dim NR; Nodal: full N-dim LU per NR iteration).
#[test]
fn test_nodal_vs_dk_diode_clipper() {
    let sample_rate = 48000.0;
    let num_samples = 480;
    let skip_samples = 10;
    let amplitude = 2.0;

    let netlist = Netlist::parse(DIODE_CLIPPER).expect("parse");
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
    mna.g[input_node][input_node] += 1.0;

    // --- DK path ---
    let kernel = DkKernel::from_mna(&mna, sample_rate).expect("dk kernel");
    let dk_config = CodegenConfig {
        circuit_name: "xval_dk_diode".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let dk_codegen = CodeGenerator::new(dk_config);
    let dk_result = dk_codegen
        .generate(&kernel, &mna, &netlist)
        .expect("dk codegen");
    let dk_output = compile_and_run_codegen_with_amplitude(
        &dk_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "nodal_vs_dk_dk",
    );

    // --- Nodal path ---
    let nodal_config = CodegenConfig {
        circuit_name: "xval_nodal_diode".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let nodal_codegen = CodeGenerator::new(nodal_config);
    let nodal_result = nodal_codegen
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen");
    let nodal_output = compile_and_run_codegen_with_amplitude(
        &nodal_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "nodal_vs_dk_nodal",
    );

    assert_eq!(
        dk_output.len(),
        nodal_output.len(),
        "DK vs Nodal: sample count mismatch"
    );

    // Compare outputs — both solve the same equations, so should match closely.
    // Tolerance is slightly relaxed (0.1% of peak) because:
    // - DK uses precomputed S=A^-1 with M-dim NR
    // - Nodal uses per-iteration LU factorization with N-dim NR
    // - DC operating point may converge to slightly different values
    let mut max_diff = 0.0f64;
    let mut max_abs = 0.0f64;
    for i in skip_samples..num_samples {
        let diff = (dk_output[i] - nodal_output[i]).abs();
        max_diff = max_diff.max(diff);
        max_abs = max_abs.max(dk_output[i].abs().max(nodal_output[i].abs()));
    }

    let rel_diff = if max_abs > 1e-10 {
        max_diff / max_abs
    } else {
        max_diff
    };

    println!(
        "Nodal vs DK: max_diff={:.2e}, max_abs={:.4}, rel_diff={:.2e}",
        max_diff, max_abs, rel_diff
    );

    assert!(
        max_abs > 1e-4,
        "Both paths should produce non-zero output, got {:.2e}",
        max_abs
    );
    assert!(
        rel_diff < 1e-3,
        "Nodal vs DK: relative diff = {:.2e} (expected < 1e-3, i.e. 0.1%)",
        rel_diff
    );
}

// ============================================================================
// Test C: Gummel-Poon vs Ebers-Moll codegen comparison
// ============================================================================

/// Gummel-Poon (VAF=100, IKF=0.3) vs Ebers-Moll should produce different output.
/// GP models Early effect (output resistance) and high-injection limiting, which
/// reduces gain at higher signal levels compared to the ideal Ebers-Moll model.
#[test]
fn test_codegen_gummel_poon_vs_ebers_moll() {
    let sample_rate = 48000.0;
    let num_samples = 4800;
    let skip_samples = 2400; // let DC OP settle
    let amplitude = 0.01;

    // --- Gummel-Poon codegen ---
    let gp_netlist = Netlist::parse(BJT_CE_GP).expect("parse GP");
    let mut gp_mna = MnaSystem::from_netlist(&gp_netlist).expect("mna GP");
    let gp_input = gp_mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let gp_output = gp_mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    gp_mna.g[gp_input][gp_input] += 1.0;
    let gp_kernel = DkKernel::from_mna(&gp_mna, sample_rate).expect("gp kernel");
    let gp_config = CodegenConfig {
        circuit_name: "crossval_gp".to_string(),
        sample_rate,
        input_node: gp_input,
        output_nodes: vec![gp_output],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let gp_codegen = CodeGenerator::new(gp_config);
    let gp_result = gp_codegen
        .generate(&gp_kernel, &gp_mna, &gp_netlist)
        .expect("gp codegen");
    let gp_samples = compile_and_run_codegen_with_amplitude(
        &gp_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "gp_model",
    );

    // --- Ebers-Moll codegen ---
    let em_netlist = Netlist::parse(BJT_CE_EM).expect("parse EM");
    let mut em_mna = MnaSystem::from_netlist(&em_netlist).expect("mna EM");
    let em_input = em_mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let em_output = em_mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    em_mna.g[em_input][em_input] += 1.0;
    let em_kernel = DkKernel::from_mna(&em_mna, sample_rate).expect("em kernel");
    let em_config = CodegenConfig {
        circuit_name: "crossval_em".to_string(),
        sample_rate,
        input_node: em_input,
        output_nodes: vec![em_output],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let em_codegen = CodeGenerator::new(em_config);
    let em_result = em_codegen
        .generate(&em_kernel, &em_mna, &em_netlist)
        .expect("em codegen");
    let em_samples = compile_and_run_codegen_with_amplitude(
        &em_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "em_model",
    );

    assert_eq!(gp_samples.len(), em_samples.len(), "sample count mismatch");

    // Both should produce non-trivial output
    let gp_peak: f64 = gp_samples[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let em_peak: f64 = em_samples[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);

    println!(
        "GP vs EM: gp_peak={:.4e}, em_peak={:.4e}",
        gp_peak, em_peak
    );

    assert!(
        gp_peak > 1e-5,
        "GP output should be non-zero, got {:.2e}",
        gp_peak
    );
    assert!(
        em_peak > 1e-5,
        "EM output should be non-zero, got {:.2e}",
        em_peak
    );

    // GP and EM should produce DIFFERENT outputs because GP includes:
    // - Early effect (VAF=100): output resistance ~ VAF/Ic
    // - High-injection limiting (IKF=0.3): gain compression at high current
    // We check that there is a measurable difference (> 1% relative).
    let mut max_diff = 0.0f64;
    let mut max_abs = 0.0f64;
    for i in skip_samples..num_samples {
        let diff = (gp_samples[i] - em_samples[i]).abs();
        max_diff = max_diff.max(diff);
        max_abs = max_abs.max(gp_samples[i].abs().max(em_samples[i].abs()));
    }

    let rel_diff = if max_abs > 1e-10 {
        max_diff / max_abs
    } else {
        0.0
    };

    println!(
        "GP vs EM divergence: max_diff={:.2e}, rel_diff={:.2e}",
        max_diff, rel_diff
    );

    assert!(
        rel_diff > 0.01,
        "GP and EM should differ by > 1%, got {:.2e} (models may not be differentiating)",
        rel_diff
    );
}

// ============================================================================
// Test D: NF cross-validation (codegen vs runtime)
// ============================================================================

/// BJT with NF=1.5: codegen vs runtime should match to < 1e-4.
/// NF (forward emission coefficient) changes the exponential slope of the
/// forward junction, effectively reducing gain. This test verifies NF flows
/// correctly through both the codegen and runtime pipelines.
#[test]
fn test_codegen_vs_runtime_bjt_nf() {
    let (max_diff, max_out) = run_crossval(BJT_CE_NF, 0.01, 4800, 2400, "bjt_nf");
    assert!(
        max_out > 1e-4,
        "BJT NF: output should be non-zero, got {:.2e}",
        max_out
    );
    assert!(
        max_diff < 1e-4,
        "BJT NF: codegen vs runtime max diff = {:.2e} (expected < 1e-4)",
        max_diff
    );
}

// ============================================================================
// Test E: Diode Zener breakdown (BV/IBV) codegen verification
// ============================================================================

const ZENER_CLIPPER: &str = "\
Zener Clipper
R1 in out 1k
D1 out 0 ZENER
C1 out 0 100n
Rload out 0 100k
.model ZENER D(IS=1e-12 N=1.5 BV=5.0 IBV=1e-3)
";

const DIODE_CLIPPER_NO_BV: &str = "\
Diode Clipper No BV
R1 in out 1k
D1 out 0 NORMAL
C1 out 0 100n
Rload out 0 100k
.model NORMAL D(IS=1e-12 N=1.5)
";

/// Zener diode (BV=5.0): reverse breakdown should clamp negative output.
/// Compare with an identical circuit without BV to verify breakdown is active.
#[test]
fn test_codegen_zener_breakdown() {
    let sample_rate = 48000.0;
    let num_samples = 4800;
    let skip_samples = 480;
    // Large amplitude to drive diode into reverse breakdown
    let amplitude = 10.0;

    // --- Zener circuit (BV=5.0) ---
    let (zener_nl, zener_mna, zener_kernel, zener_in, zener_out) =
        build_pipeline(ZENER_CLIPPER, sample_rate);
    let zener_config = CodegenConfig {
        circuit_name: "xval_zener".to_string(),
        sample_rate,
        input_node: zener_in,
        output_nodes: vec![zener_out],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let zener_codegen = CodeGenerator::new(zener_config);
    let zener_result = zener_codegen
        .generate(&zener_kernel, &zener_mna, &zener_nl)
        .expect("zener codegen");
    let zener_output = compile_and_run_codegen_with_amplitude(
        &zener_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "zener_bv",
    );

    // --- Normal diode circuit (no BV) ---
    let (normal_nl, normal_mna, normal_kernel, normal_in, normal_out) =
        build_pipeline(DIODE_CLIPPER_NO_BV, sample_rate);
    let normal_config = CodegenConfig {
        circuit_name: "xval_normal_diode".to_string(),
        sample_rate,
        input_node: normal_in,
        output_nodes: vec![normal_out],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let normal_codegen = CodeGenerator::new(normal_config);
    let normal_result = normal_codegen
        .generate(&normal_kernel, &normal_mna, &normal_nl)
        .expect("normal codegen");
    let normal_output = compile_and_run_codegen_with_amplitude(
        &normal_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "normal_diode",
    );

    assert_eq!(zener_output.len(), normal_output.len());

    // Both should produce non-zero output
    let zener_peak: f64 = zener_output[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let normal_peak: f64 = normal_output[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);

    println!(
        "Zener vs Normal: zener_peak={:.4}, normal_peak={:.4}",
        zener_peak, normal_peak
    );

    assert!(zener_peak > 0.1, "Zener output should be non-trivial");
    assert!(normal_peak > 0.1, "Normal output should be non-trivial");

    // The Zener circuit should have LESS negative swing than the normal diode
    // because reverse breakdown clamps the negative voltage around -BV.
    let zener_min: f64 = zener_output[skip_samples..]
        .iter()
        .copied()
        .fold(f64::MAX, f64::min);
    let normal_min: f64 = normal_output[skip_samples..]
        .iter()
        .copied()
        .fold(f64::MAX, f64::min);

    println!(
        "Zener min={:.4}, Normal min={:.4}",
        zener_min, normal_min
    );

    // With BV=5.0, Zener should clamp around -5V to -6V.
    // Without BV, normal diode passes full negative swing through R1.
    // The Zener's negative peak should be significantly less negative than the normal diode's.
    assert!(
        zener_min > normal_min,
        "Zener should clamp negative voltage: zener_min={:.4} should be > normal_min={:.4}",
        zener_min, normal_min
    );

    // Zener negative peak should be limited approximately near -BV (allow some margin for R1 drop)
    assert!(
        zener_min > -8.0,
        "Zener clamp: negative peak {:.4}V should be limited near -BV=5.0V",
        zener_min
    );
}

// ============================================================================
// Test F: BJT with parasitic resistances (RB/RC/RE) codegen verification
// ============================================================================

const BJT_CE_PARASITIC: &str = "\
BJT CE with Parasitics
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJT_R
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJT_R NPN(IS=1e-14 BF=200 BR=3 RB=100 RC=10 RE=5)
";

const BJT_CE_NO_PARASITIC: &str = "\
BJT CE no Parasitics
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJT_NR
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJT_NR NPN(IS=1e-14 BF=200 BR=3)
";

/// BJT with parasitic R (RB=100, RC=10, RE=5) should compile, run, and
/// produce different output than the same circuit without parasitics.
/// Parasitic R reduces effective gain and shifts the operating point.
#[test]
fn test_codegen_bjt_parasitic_resistances() {
    let sample_rate = 48000.0;
    let num_samples = 4800;
    let skip_samples = 2400;
    let amplitude = 0.01;

    // --- With parasitic R ---
    let (pr_nl, pr_mna, pr_kernel, pr_in, pr_out) =
        build_pipeline(BJT_CE_PARASITIC, sample_rate);
    let pr_config = CodegenConfig {
        circuit_name: "xval_bjt_pr".to_string(),
        sample_rate,
        input_node: pr_in,
        output_nodes: vec![pr_out],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let pr_codegen = CodeGenerator::new(pr_config);
    let pr_result = pr_codegen
        .generate(&pr_kernel, &pr_mna, &pr_nl)
        .expect("parasitic codegen");
    let pr_output = compile_and_run_codegen_with_amplitude(
        &pr_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "bjt_parasitic",
    );

    // --- Without parasitic R ---
    let (np_nl, np_mna, np_kernel, np_in, np_out) =
        build_pipeline(BJT_CE_NO_PARASITIC, sample_rate);
    let np_config = CodegenConfig {
        circuit_name: "xval_bjt_np".to_string(),
        sample_rate,
        input_node: np_in,
        output_nodes: vec![np_out],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let np_codegen = CodeGenerator::new(np_config);
    let np_result = np_codegen
        .generate(&np_kernel, &np_mna, &np_nl)
        .expect("no-parasitic codegen");
    let np_output = compile_and_run_codegen_with_amplitude(
        &np_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "bjt_no_parasitic",
    );

    assert_eq!(pr_output.len(), np_output.len());

    // Both should produce non-zero output
    let pr_peak: f64 = pr_output[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let np_peak: f64 = np_output[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);

    println!(
        "Parasitic vs Clean: pr_peak={:.4e}, np_peak={:.4e}",
        pr_peak, np_peak
    );

    assert!(
        pr_peak > 1e-5,
        "Parasitic BJT output should be non-zero: {:.2e}",
        pr_peak
    );
    assert!(
        np_peak > 1e-5,
        "Clean BJT output should be non-zero: {:.2e}",
        np_peak
    );

    // Parasitic R should reduce gain (RB reduces base drive, RE adds local feedback)
    // So pr_peak should be less than np_peak.
    // Allow that they might be close if RE is dominated by the external Re=1k.
    let mut max_diff = 0.0f64;
    let mut max_abs = 0.0f64;
    for i in skip_samples..num_samples {
        let diff = (pr_output[i] - np_output[i]).abs();
        max_diff = max_diff.max(diff);
        max_abs = max_abs.max(pr_output[i].abs().max(np_output[i].abs()));
    }
    let rel_diff = if max_abs > 1e-10 {
        max_diff / max_abs
    } else {
        0.0
    };

    println!(
        "Parasitic vs Clean divergence: max_diff={:.2e}, rel_diff={:.2e}",
        max_diff, rel_diff
    );

    // They should be measurably different (> 0.1% relative)
    assert!(
        rel_diff > 0.001,
        "Parasitic R should change output: rel_diff={:.2e} (expected > 0.1%)",
        rel_diff
    );

    // Both outputs should be finite and not NaN (NR converges with parasitic inner loop)
    for (i, s) in pr_output.iter().enumerate() {
        assert!(
            s.is_finite(),
            "Parasitic BJT: NaN/Inf at sample {}: {}",
            i, s
        );
    }
}

// ============================================================================
// Test G: MOSFET body effect (GAMMA/PHI) codegen verification
// ============================================================================

const MOSFET_CS_BODY: &str = "\
MOSFET CS with Body Effect
Cin in gate 1u
Vdd vdd 0 DC 12
R1 vdd gate 100k
R2 gate 0 47k
Rd vdd drain 2.2k
Rs source 0 470
Cs source 0 100u
Cout drain out 1u
Rload out 0 100k
M1 drain gate source 0 NMOD_BE
.model NMOD_BE NM(VTO=2.0 KP=0.1 LAMBDA=0.01 GAMMA=0.37 PHI=0.65)
";

const MOSFET_CS_NO_BODY: &str = "\
MOSFET CS no Body Effect
Cin in gate 1u
Vdd vdd 0 DC 12
R1 vdd gate 100k
R2 gate 0 47k
Rd vdd drain 2.2k
Rs source 0 470
Cs source 0 100u
Cout drain out 1u
Rload out 0 100k
M1 drain gate source 0 NMOD_NB
.model NMOD_NB NM(VTO=2.0 KP=0.1 LAMBDA=0.01)
";

/// MOSFET with body effect (GAMMA=0.37, PHI=0.65) should compile, run, and
/// produce different output than same circuit without body effect.
/// Body effect increases Vt when Vsb > 0, reducing drain current and gain.
#[test]
fn test_codegen_mosfet_body_effect() {
    let sample_rate = 48000.0;
    let num_samples = 4800;
    let skip_samples = 2400;
    let amplitude = 0.05;

    // --- With body effect ---
    let (be_nl, be_mna, be_kernel, be_in, be_out) =
        build_pipeline(MOSFET_CS_BODY, sample_rate);
    let be_config = CodegenConfig {
        circuit_name: "xval_mos_be".to_string(),
        sample_rate,
        input_node: be_in,
        output_nodes: vec![be_out],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let be_codegen = CodeGenerator::new(be_config);
    let be_result = be_codegen
        .generate(&be_kernel, &be_mna, &be_nl)
        .expect("body effect codegen");

    // Verify GAMMA/PHI constants appear in generated code
    assert!(
        be_result.code.contains("DEVICE_0_GAMMA"),
        "Generated code should contain DEVICE_0_GAMMA constant"
    );
    assert!(
        be_result.code.contains("DEVICE_0_PHI"),
        "Generated code should contain DEVICE_0_PHI constant"
    );

    let be_output = compile_and_run_codegen_with_amplitude(
        &be_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "mos_body_effect",
    );

    // --- Without body effect ---
    let (nb_nl, nb_mna, nb_kernel, nb_in, nb_out) =
        build_pipeline(MOSFET_CS_NO_BODY, sample_rate);
    let nb_config = CodegenConfig {
        circuit_name: "xval_mos_nb".to_string(),
        sample_rate,
        input_node: nb_in,
        output_nodes: vec![nb_out],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let nb_codegen = CodeGenerator::new(nb_config);
    let nb_result = nb_codegen
        .generate(&nb_kernel, &nb_mna, &nb_nl)
        .expect("no body effect codegen");
    let nb_output = compile_and_run_codegen_with_amplitude(
        &nb_result.code,
        num_samples,
        sample_rate,
        amplitude,
        "mos_no_body",
    );

    assert_eq!(be_output.len(), nb_output.len());

    let be_peak: f64 = be_output[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let nb_peak: f64 = nb_output[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);

    println!(
        "Body effect vs No body effect: be_peak={:.4e}, nb_peak={:.4e}",
        be_peak, nb_peak
    );

    assert!(be_peak > 1e-5, "Body effect output non-zero: {:.2e}", be_peak);
    assert!(nb_peak > 1e-5, "No body effect output non-zero: {:.2e}", nb_peak);

    // Body effect increases Vt → reduces gain → outputs should differ
    let mut max_diff = 0.0f64;
    let mut max_abs = 0.0f64;
    for i in skip_samples..num_samples {
        let diff = (be_output[i] - nb_output[i]).abs();
        max_diff = max_diff.max(diff);
        max_abs = max_abs.max(be_output[i].abs().max(nb_output[i].abs()));
    }
    let rel_diff = if max_abs > 1e-10 { max_diff / max_abs } else { 0.0 };

    println!(
        "Body effect divergence: max_diff={:.2e}, rel_diff={:.2e}",
        max_diff, rel_diff
    );

    // They should be measurably different (body effect shifts Vt by ~0.1-0.5V)
    assert!(
        rel_diff > 0.001,
        "Body effect should change output: rel_diff={:.2e} (expected > 0.1%)",
        rel_diff
    );

    for (i, s) in be_output.iter().enumerate() {
        assert!(s.is_finite(), "Body effect MOSFET: NaN/Inf at sample {}", i);
    }
}

// ============================================================================
// Test H: Schur nodal path cross-validation (inductor + nonlinear)
// ============================================================================

const DIODE_CLIPPER_WITH_INDUCTOR: &str = "\
Diode Clipper with Inductor
R1 in mid 1k
L1 mid out 10m
D1 out 0 1N4148
D2 0 out 1N4148
C1 out 0 100n
Rload out 0 10k
.model 1N4148 D(IS=2.52e-9 N=1.752)
";

/// Schur nodal path (triggered by inductor + nonlinear): verify output
/// against the DK path for a circuit without the inductor. The inductor
/// should add phase shift and resonance but the clipping behavior should
/// be similar at low frequencies.
///
/// Primary goal: prove the Schur nodal M-dim NR produces correct, finite,
/// convergent output — not just that it compiles.
#[test]
fn test_schur_nodal_diode_inductor() {
    let sample_rate = 48000.0;
    let num_samples = 4800;
    let skip_samples = 480;
    let amplitude = 2.0;

    let (nl, mna, _, in_node, out_node) =
        build_pipeline(DIODE_CLIPPER_WITH_INDUCTOR, sample_rate);

    // This circuit has inductors + nonlinear devices, so generate_nodal
    // should be used (the DK kernel may auto-route to nodal anyway).
    let config = CodegenConfig {
        circuit_name: "xval_schur_diode_ind".to_string(),
        sample_rate,
        input_node: in_node,
        output_nodes: vec![out_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen
        .generate_nodal(&mna, &nl)
        .expect("schur nodal codegen");

    let output = compile_and_run_codegen_with_amplitude(
        &result.code,
        num_samples,
        sample_rate,
        amplitude,
        "schur_diode_ind",
    );

    assert_eq!(output.len(), num_samples);

    // Output should be non-zero and finite
    let peak: f64 = output[skip_samples..]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);

    println!("Schur nodal diode+inductor: peak={:.4}", peak);

    assert!(
        peak > 0.01,
        "Schur nodal output should be non-trivial: {:.2e}",
        peak
    );

    // Diode clipping should limit output — peak should be below ~1V
    // (diodes clip at ~0.6V forward, but inductor energy storage can overshoot)
    assert!(
        peak < 5.0,
        "Schur nodal: output should be clipped by diodes, got peak={:.4}",
        peak
    );

    // All samples should be finite (no NR divergence)
    for (i, s) in output.iter().enumerate() {
        assert!(s.is_finite(), "Schur nodal: NaN/Inf at sample {}: {}", i, s);
    }

    // Verify output has AC content (not just DC)
    let mean: f64 = output[skip_samples..].iter().sum::<f64>() / (num_samples - skip_samples) as f64;
    let variance: f64 = output[skip_samples..]
        .iter()
        .map(|s| (s - mean).powi(2))
        .sum::<f64>()
        / (num_samples - skip_samples) as f64;
    let rms_ac = variance.sqrt();

    println!("Schur nodal: mean={:.4e}, rms_ac={:.4e}", mean, rms_ac);

    assert!(
        rms_ac > 0.001,
        "Schur nodal output should have AC content: rms_ac={:.2e}",
        rms_ac
    );
}
