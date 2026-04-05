//! Golden reference recording from runtime solver.
//!
//! This file was used to record golden reference files BEFORE the runtime
//! solver was deleted. The golden files now exist in tests/golden/ and are
//! used by golden_ref_tests.rs.
//!
//! This file is disabled because the runtime solvers no longer exist.
#![cfg(any())]
//!
//! Each test is #[ignore] — run explicitly with:
//!   cargo test -p melange-solver --test record_golden_refs -- --include-ignored
//!
//! Golden files are written to tests/golden/*.json and should be committed.

mod support;

use melange_devices::{BjtEbersMoll, BjtPolarity, DiodeShockley, Jfet, JfetChannel, KorenTriode, Mosfet, MosfetChannelType};
use melange_primitives::VT_ROOM;
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::solver::{CircuitSolver, DeviceEntry, LinearSolver};

use std::path::PathBuf;

const SR: f64 = 48000.0;

fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

fn write_golden(filename: &str, test_name: &str, spice: &str, output: &[f64], tolerance: f64, generator: &str) {
    let dir = golden_dir();
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(filename);
    if path.exists() {
        // Don't overwrite existing golden files — delete manually to re-record
        eprintln!("Golden file already exists: {}", path.display());
        return;
    }
    let json = serde_json::json!({
        "test_name": test_name,
        "spice": spice,
        "sample_rate": SR,
        "output": output,
        "tolerance": tolerance,
        "generator": generator,
        "recorded_at": timestamp(),
    });
    let content = serde_json::to_string_pretty(&json).unwrap();
    std::fs::write(&path, &content).unwrap();
    eprintln!("Recorded golden: {} ({} samples)", path.display(), output.len());
}

fn timestamp() -> String {
    std::process::Command::new("date")
        .args(["-u", "+%Y-%m-%dT%H:%M:%SZ"])
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .unwrap_or_else(|| "unknown".to_string())
        .trim()
        .to_string()
}

// ── Pipeline helpers ───────────────────────────────────────────────────

fn build_linear(spice: &str) -> (MnaSystem, usize, usize) {
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let input_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    let output_node = mna.node_map.get("out").copied().unwrap_or(2).saturating_sub(1);
    mna.g[input_node][input_node] += 1.0;
    (mna, input_node, output_node)
}

fn run_linear_sine(spice: &str, freq: f64, amplitude: f64, num_samples: usize) -> Vec<f64> {
    let (mna, input_node, output_node) = build_linear(spice);
    let kernel = DkKernel::from_mna(&mna, SR).unwrap();
    let mut solver = LinearSolver::new(kernel, input_node, output_node);
    solver.set_input_conductance(1.0);
    (0..num_samples)
        .map(|i| {
            let t = i as f64 / SR;
            let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
            solver.process_sample(input)
        })
        .collect()
}

fn run_linear_step(spice: &str, amplitude: f64, num_samples: usize) -> Vec<f64> {
    let (mna, input_node, output_node) = build_linear(spice);
    let kernel = DkKernel::from_mna(&mna, SR).unwrap();
    let mut solver = LinearSolver::new(kernel, input_node, output_node);
    solver.set_input_conductance(1.0);
    (0..num_samples)
        .map(|_| solver.process_sample(amplitude))
        .collect()
}

fn run_nonlinear_sine(
    spice: &str,
    devices: Vec<DeviceEntry>,
    freq: f64,
    amplitude: f64,
    num_samples: usize,
) -> Vec<f64> {
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let input_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    let output_node = mna.node_map.get("out").copied().unwrap_or(2).saturating_sub(1);
    mna.g[input_node][input_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, SR).unwrap();

    let mut solver = CircuitSolver::new(kernel.clone(), devices, input_node, output_node).unwrap();
    solver.set_input_conductance(1.0);

    // DC OP initialization
    if kernel.m > 0 {
        use melange_solver::codegen::ir::CircuitIR;
        use melange_solver::codegen::CodegenConfig;
        let config = CodegenConfig {
            circuit_name: "golden_ref".to_string(),
            sample_rate: SR,
            input_node,
            output_nodes: vec![output_node],
            input_resistance: 1.0,
            ..CodegenConfig::default()
        };
        let slots = match CircuitIR::from_kernel(&kernel, &mna, &netlist, &config) {
            Ok(ir) => ir.device_slots,
            Err(_) => Vec::new(),
        };
        solver.initialize_dc_op(&mna, &slots);
    }

    (0..num_samples)
        .map(|i| {
            let t = i as f64 / SR;
            let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
            solver.process_sample(input)
        })
        .collect()
}

// ── Circuit definitions ────────────────────────────────────────────────

const RC_LOWPASS: &str = "\
RC Lowpass
R1 in out 1k
C1 out 0 1u
";

const RC_HIGHPASS: &str = "\
RC Highpass
C1 in out 1u
R1 out 0 1k
";

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

const OPAMP_INVERTING: &str = "\
Inverting Amplifier
R1 in vminus 1k
R2 vminus out 10k
U1 0 vminus out OPA
.model OPA OA(AOL=200000 ROUT=1)
";

const OPAMP_NONINVERTING: &str = "\
Non-Inverting Amplifier
R1 0 vminus 1k
R2 vminus out 10k
U1 in vminus out OPA
.model OPA OA(AOL=200000 ROUT=1)
";

// ── Recording tests ────────────────────────────────────────────────────

#[test]
#[ignore]
fn record_rc_lowpass_step() {
    let output = run_linear_step(RC_LOWPASS, 1.0, 300);
    write_golden(
        "rc_lowpass_step.json",
        "rc_lowpass_step",
        RC_LOWPASS,
        &output,
        1e-10,
        "runtime_LinearSolver",
    );
}

#[test]
#[ignore]
fn record_rc_lowpass_sine_1k() {
    let output = run_linear_sine(RC_LOWPASS, 1000.0, 0.5, 480);
    write_golden(
        "rc_lowpass_sine_1k.json",
        "rc_lowpass_sine_1k",
        RC_LOWPASS,
        &output,
        1e-10,
        "runtime_LinearSolver",
    );
}

#[test]
#[ignore]
fn record_rc_highpass_sine_1k() {
    let output = run_linear_sine(RC_HIGHPASS, 1000.0, 0.5, 480);
    write_golden(
        "rc_highpass_sine_1k.json",
        "rc_highpass_sine_1k",
        RC_HIGHPASS,
        &output,
        1e-10,
        "runtime_LinearSolver",
    );
}

#[test]
#[ignore]
fn record_opamp_inverting_sine_1k() {
    let output = run_linear_sine(OPAMP_INVERTING, 1000.0, 0.1, 480);
    write_golden(
        "opamp_inverting_sine_1k.json",
        "opamp_inverting_sine_1k",
        OPAMP_INVERTING,
        &output,
        1e-10,
        "runtime_LinearSolver",
    );
}

#[test]
#[ignore]
fn record_opamp_noninverting_sine_1k() {
    let output = run_linear_sine(OPAMP_NONINVERTING, 1000.0, 0.1, 480);
    write_golden(
        "opamp_noninverting_sine_1k.json",
        "opamp_noninverting_sine_1k",
        OPAMP_NONINVERTING,
        &output,
        1e-10,
        "runtime_LinearSolver",
    );
}

#[test]
#[ignore]
fn record_diode_clipper_sine_500hz() {
    let netlist = Netlist::parse(DIODE_CLIPPER).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let devices = vec![
        DeviceEntry::new_diode(DiodeShockley::new_room_temp(2.52e-9, 1.752), 0),
        DeviceEntry::new_diode(DiodeShockley::new_room_temp(2.52e-9, 1.752), 1),
    ];
    let output = run_nonlinear_sine(DIODE_CLIPPER, devices, 500.0, 1.0, 960);
    write_golden(
        "diode_clipper_sine_500hz.json",
        "diode_clipper_sine_500hz",
        DIODE_CLIPPER,
        &output,
        1e-6,
        "runtime_CircuitSolver",
    );
}

#[test]
#[ignore]
fn record_bjt_ce_sine_500hz() {
    let netlist = Netlist::parse(BJT_CE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let devices = {
        let mut devs = Vec::new();
        for dev_info in &mna.nonlinear_devices {
            let bjt = BjtEbersMoll::new(1e-14, VT_ROOM, 200.0, 3.0, BjtPolarity::Npn);
            devs.push(DeviceEntry::new_bjt(bjt, dev_info.start_idx));
        }
        devs
    };
    let output = run_nonlinear_sine(BJT_CE, devices, 500.0, 0.05, 960);
    write_golden(
        "bjt_ce_sine_500hz.json",
        "bjt_ce_sine_500hz",
        BJT_CE,
        &output,
        1e-4,
        "runtime_CircuitSolver",
    );
}

#[test]
#[ignore]
fn record_jfet_cs_sine_500hz() {
    let netlist = Netlist::parse(JFET_CS).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let devices = {
        let mut devs = Vec::new();
        for dev_info in &mna.nonlinear_devices {
            let mut jfet = Jfet::new(JfetChannel::N, -2.0, 5e-3);
            jfet.lambda = 0.001;
            devs.push(DeviceEntry::new_jfet(jfet, dev_info.start_idx));
        }
        devs
    };
    let output = run_nonlinear_sine(JFET_CS, devices, 500.0, 0.1, 960);
    write_golden(
        "jfet_cs_sine_500hz.json",
        "jfet_cs_sine_500hz",
        JFET_CS,
        &output,
        1e-4,
        "runtime_CircuitSolver",
    );
}

#[test]
#[ignore]
fn record_mosfet_cs_sine_500hz() {
    let netlist = Netlist::parse(MOSFET_CS).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let devices = {
        let mut devs = Vec::new();
        for dev_info in &mna.nonlinear_devices {
            let mosfet = Mosfet::new(MosfetChannelType::N, 2.0, 0.1, 0.01);
            devs.push(DeviceEntry::new_mosfet(mosfet, dev_info.start_idx));
        }
        devs
    };
    let output = run_nonlinear_sine(MOSFET_CS, devices, 500.0, 0.1, 960);
    write_golden(
        "mosfet_cs_sine_500hz.json",
        "mosfet_cs_sine_500hz",
        MOSFET_CS,
        &output,
        1e-4,
        "runtime_CircuitSolver",
    );
}

#[test]
#[ignore]
fn record_triode_cc_sine_500hz() {
    let netlist = Netlist::parse(TRIODE_CC).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let devices = {
        let mut devs = Vec::new();
        for dev_info in &mna.nonlinear_devices {
            let tube = KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.0);
            devs.push(DeviceEntry::new_tube(tube, dev_info.start_idx));
        }
        devs
    };
    let output = run_nonlinear_sine(TRIODE_CC, devices, 500.0, 0.5, 960);
    write_golden(
        "triode_cc_sine_500hz.json",
        "triode_cc_sine_500hz",
        TRIODE_CC,
        &output,
        1e-4,
        "runtime_CircuitSolver",
    );
}
