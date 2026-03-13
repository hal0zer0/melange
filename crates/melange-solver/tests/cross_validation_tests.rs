//! Cross-validation tests: codegen vs runtime solver, Sherman-Morrison correctness.
//!
//! These tests ensure that the runtime solver (DeviceEntry dispatch) and codegen
//! (Tera templates emitting inline device functions) produce identical output
//! for the same circuit + input. Any divergence means one path has a bug.

use std::io::Write;
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::codegen::ir::{CircuitIR, DeviceSlot};
use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use melange_solver::solver::{CircuitSolver, DeviceEntry};

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
Q1 coll base emit 2N2222
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model 2N2222 NPN(IS=1e-14 BF=200 BR=3)
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
    let input_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    let output_node = mna.node_map.get("out").copied().unwrap_or(2).saturating_sub(1);
    mna.g[input_node][input_node] += 1.0; // G_in stamp before kernel
    let kernel = DkKernel::from_mna(&mna, sample_rate).expect("kernel");
    (netlist, mna, kernel, input_node, output_node)
}

/// Build DeviceEntry list from MNA nonlinear device list + parsed netlist.
fn build_device_entries(
    netlist: &Netlist,
    mna: &MnaSystem,
) -> Vec<DeviceEntry> {
    let find_model_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist.models.iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| m.params.iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(param))
                .map(|(_, v)| *v))
    };

    let find_model_name = |dev_name: &str| -> String {
        netlist.elements.iter().find_map(|e| {
            let (name, model) = match e {
                melange_solver::parser::Element::Diode { name, model, .. } => (name, model),
                melange_solver::parser::Element::Bjt { name, model, .. } => (name, model),
                melange_solver::parser::Element::Jfet { name, model, .. } => (name, model),
                melange_solver::parser::Element::Mosfet { name, model, .. } => (name, model),
                melange_solver::parser::Element::Triode { name, model, .. } => (name, model),
                _ => return None,
            };
            if name.eq_ignore_ascii_case(dev_name) { Some(model.clone()) } else { None }
        }).unwrap_or_default()
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
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let is = find_model_param(&model, "IS").unwrap_or(1e-14);
                let bf = find_model_param(&model, "BF").unwrap_or(200.0);
                let br = find_model_param(&model, "BR").unwrap_or(3.0);
                let is_pnp = netlist.models.iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp { melange_devices::BjtPolarity::Pnp } else { melange_devices::BjtPolarity::Npn };
                let bjt = melange_devices::BjtEbersMoll::new(is, melange_primitives::VT_ROOM, bf, br, polarity);
                devices.push(DeviceEntry::new_bjt(bjt, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let is_p = netlist.models.iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let channel = if is_p { melange_devices::JfetChannel::P } else { melange_devices::JfetChannel::N };
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
                let is_p = netlist.models.iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let channel = if is_p { melange_devices::MosfetChannelType::P } else { melange_devices::MosfetChannelType::N };
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
                let tube = melange_devices::KorenTriode::with_grid_params(mu, ex, kg1, kp, kvb, ig_max, vgk_onset);
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
    code: &str, num_samples: usize, sample_rate: f64, amplitude: f64, tag: &str,
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
        panic!("Codegen compilation failed for {tag}:\n{}", String::from_utf8_lossy(&compile.stderr));
    }

    let run = std::process::Command::new(&bin_path).output().expect("run");
    let _ = std::fs::remove_file(&bin_path);
    if !run.status.success() {
        panic!("Codegen binary failed for {tag}:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr));
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
    spice: &str, amplitude: f64, num_samples: usize, skip_samples: usize, tag: &str,
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
        &result.code, num_samples, sample_rate, amplitude, tag,
    );

    assert_eq!(codegen_output.len(), runtime_output.len(),
        "{tag}: sample count mismatch: codegen={} runtime={}", codegen_output.len(), runtime_output.len());

    // Compare sample-by-sample
    let mut max_diff = 0.0f64;
    let mut max_runtime_abs = 0.0f64;
    for i in skip_samples..num_samples {
        let diff = (codegen_output[i] - runtime_output[i]).abs();
        max_diff = max_diff.max(diff);
        max_runtime_abs = max_runtime_abs.max(runtime_output[i].abs());
    }

    println!("{tag}: codegen vs runtime max_diff={:.2e}, max_output={:.4}", max_diff, max_runtime_abs);
    (max_diff, max_runtime_abs)
}

// ============================================================================
// Cross-validation tests
// ============================================================================

/// Diode clipper: codegen vs runtime should match to < 1e-6.
#[test]
fn test_codegen_vs_runtime_diode_clipper() {
    let (max_diff, _) = run_crossval(DIODE_CLIPPER, 2.0, 480, 10, "diode");
    assert!(max_diff < 1e-6,
        "Diode clipper: codegen vs runtime max diff = {:.2e} (expected < 1e-6)", max_diff);
}

/// BJT common-emitter: codegen vs runtime should match to < 1e-4.
/// (Slightly looser tolerance due to DC OP sensitivity.)
#[test]
fn test_codegen_vs_runtime_bjt_ce() {
    let (max_diff, max_out) = run_crossval(BJT_CE, 0.01, 4800, 2400, "bjt");
    assert!(max_out > 1e-4, "BJT CE: output should be non-zero, got {:.2e}", max_out);
    assert!(max_diff < 1e-4,
        "BJT CE: codegen vs runtime max diff = {:.2e} (expected < 1e-4)", max_diff);
}

/// JFET common-source: codegen vs runtime should match to < 1e-4.
#[test]
fn test_codegen_vs_runtime_jfet_cs() {
    let (max_diff, max_out) = run_crossval(JFET_CS, 0.1, 960, 100, "jfet");
    assert!(max_out > 1e-4, "JFET CS: output should be non-zero, got {:.2e}", max_out);
    assert!(max_diff < 1e-4,
        "JFET CS: codegen vs runtime max diff = {:.2e} (expected < 1e-4)", max_diff);
}

/// MOSFET common-source: codegen vs runtime should match to < 1e-4.
#[test]
fn test_codegen_vs_runtime_mosfet_cs() {
    let (max_diff, max_out) = run_crossval(MOSFET_CS, 0.1, 960, 100, "mosfet");
    assert!(max_out > 1e-4, "MOSFET CS: output should be non-zero, got {:.2e}", max_out);
    assert!(max_diff < 1e-4,
        "MOSFET CS: codegen vs runtime max diff = {:.2e} (expected < 1e-4)", max_diff);
}

/// Triode common-cathode: codegen vs runtime should match to < 1e-4.
#[test]
fn test_codegen_vs_runtime_triode_cc() {
    let (max_diff, max_out) = run_crossval(TRIODE_CC, 0.01, 4800, 2400, "triode");
    assert!(max_out > 1e-4, "Triode CC: output should be non-zero, got {:.2e}", max_out);
    assert!(max_diff < 1e-4,
        "Triode CC: codegen vs runtime max diff = {:.2e} (expected < 1e-4)", max_diff);
}

/// Test: Sherman-Morrison pot data is consistent in kernel.
///
/// Verifies SM precomputed vectors are finite and have correct dimensions.
#[test]
fn test_sherman_morrison_pot_data_sanity() {
    let spice = "SM test\nR1 in mid 1k\n.pot R1 100 10k\nR2 mid out 1k\nC1 out 0 10n\n.end";
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
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
    assert!(pot.min_resistance > 0.0, "min_resistance should be positive");
    assert!(pot.max_resistance > pot.min_resistance, "max > min resistance");

    // S matrix should be finite
    for i in 0..n {
        for j in 0..n {
            assert!(kernel.s[i * n + j].is_finite(),
                "S[{}][{}] = {} is not finite", i, j, kernel.s[i * n + j]);
        }
    }

    // Nominal conductance should be 1/R_default = 1/1000 = 0.001
    assert!((pot.g_nominal - 0.001).abs() < 1e-6,
        "Nominal conductance should be ~0.001, got {}", pot.g_nominal);

    println!("SM sanity: N={}, M={}, pot min_R={:.0}, max_R={:.0}, g_nom={:.6e}",
        n, kernel.m, pot.min_resistance, pot.max_resistance, pot.g_nominal);
}
