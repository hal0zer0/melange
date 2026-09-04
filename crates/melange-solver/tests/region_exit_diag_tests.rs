//! Grid-off pentode reduction policy + `diag_region_exit_count`.
//!
//! Policy under test (2026-09-04):
//! - `--tube-grid-fa auto` never reduces: the frozen-Vg2k reduction drops the
//!   cathode/screen-referenced Vg2k feedback (measured +2% to +12%
//!   small-signal gain error on cathode-biased stages) and all grid current
//!   for Vgk > 0. Only the explicit, warned `on` reduces.
//! - A reduction can never change the solver route (nodal -> DK).
//! - The emitted code counts, on the FULL models as well as the reduced ones,
//!   the device-samples where a pentode's grid conducts (Vgk > 0) or a BJT's
//!   base-collector junction goes forward (saturation): the regions the
//!   grid-off / forward-active reductions assume are never entered.

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use std::collections::HashSet;
use std::io::Write;

// ============================================================================
// Test circuits
// ============================================================================

const EL84_MODEL: &str = ".model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 KP=152.4 KVB=4015.8 \
ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148 IG_MAX=8m VGK_ONSET=0.7)\n";

/// Cathode-biased EL84 stage, unbypassed Rk, bypassed screen (the
/// noyce-6bq5 shape). Vgk ~ -8 V at the DC OP — the old `auto` reduced it,
/// and the reduction measured +3.0% small-signal gain error vs ngspice.
fn pentode_cathode_biased() -> String {
    format!(
        "Pentode cathode-biased\n{EL84_MODEL}\
VCC vcc 0 DC 300
R_iso in grid 1Meg
Rg grid 0 470k
P1 plate grid cathode screen EL84
Ra vcc plate 2.2k
Rg2 vcc screen 100
Cg2 screen 0 100n
Rk cathode 0 150
Cout plate out 100n
Rload out 0 1Meg
"
    )
}

/// Grid driven straight from the input over a 10 ohm cathode resistor
/// (the DK kernel needs a nonzero K diagonal, which a grounded cathode
/// would zero): a +/-2 V input swings Vgk through 0 on every positive
/// half-cycle.
fn pentode_grid_driven() -> String {
    format!(
        "Pentode grid-driven\n{EL84_MODEL}\
VCC vcc 0 DC 300
Rg in 0 470k
P1 plate in cathode screen EL84
Rk cathode 0 10
Ra vcc plate 2.2k
Rg2 vcc screen 1k
Cg2 screen 0 100n
Cout plate out 100n
Rload out 0 1Meg
"
    )
}

/// Grounded cathode (nodal only — see above): the emitted arm must read the
/// cathode as the literal 0.0.
fn pentode_grounded_cathode() -> String {
    format!(
        "Pentode grounded cathode\n{EL84_MODEL}\
VCC vcc 0 DC 300
Rg in 0 470k
P1 plate in 0 screen EL84
Ra vcc plate 2.2k
Rg2 vcc screen 1k
Cg2 screen 0 100n
Cout plate out 100n
Rload out 0 1Meg
"
    )
}

/// Four cathode-biased stages: M = 12 unreduced, which the auto-router
/// sends to nodal ("large nonlinear dimension"). Reducing would give M = 8
/// and flip the route to DK Schur — the flip this policy forbids.
fn four_pentodes() -> String {
    let mut s = format!("Four pentodes\n{EL84_MODEL}VCC vcc 0 DC 300\nR_iso in g1 1Meg\n");
    for i in 1..=4 {
        s.push_str(&format!(
            "Rg{i} g{i} 0 470k\nP{i} p{i} g{i} k{i} s{i} EL84\nRa{i} vcc p{i} 2.2k\n\
             Rg2{i} vcc s{i} 100\nCg2{i} s{i} 0 100n\nRk{i} k{i} 0 150\n"
        ));
        if i < 4 {
            s.push_str(&format!("Cc{i} p{i} g{} 22n\n", i + 1));
        }
    }
    s.push_str("Cout p4 out 100n\nRload out 0 1Meg\n");
    s
}

/// Pure-Ebers-Moll CE stage (FA-reduced under `--bjt-fa auto`) with the
/// input coupled into the base: a 1 V input drives it into saturation.
const CE_SATURATING: &str = "\
CE saturating
.model NPN1 NPN(IS=1e-14 BF=200 BR=3)
VCC vcc 0 DC 15
Cin in base 10u
R1 vcc base 100k
R2 base 0 10k
RC vcc collector 10k
RE emitter 0 1k
CE emitter 0 100u
Q1 collector base emitter NPN1
";

/// Triode stage: no pentode, no BJT — no region-exit arm must be emitted.
const TRIODE_ONLY: &str = "\
Triode only
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
VCC vcc 0 DC 250
Cin in grid 22n
Rg grid 0 1Meg
T1 grid plate cath 12AX7
Ra vcc plate 100k
Rk cath 0 1.5k
Cout plate out 22n
Rload out 0 1Meg
";

// ============================================================================
// Helpers
// ============================================================================

fn build_mna(spice: &str) -> (Netlist, MnaSystem, CodegenConfig) {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA");
    let input = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let output = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    if input < mna.n {
        mna.g[input][input] += 1.0;
    }
    let config = CodegenConfig {
        circuit_name: "region_exit_test".to_string(),
        sample_rate: 48000.0,
        input_node: input,
        output_nodes: vec![output],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    (netlist, mna, config)
}

/// Run the shared pipeline steps (FA + grid-off) exactly as compile does
/// and return the MNA plus the DK-emitted code.
fn generate_dk(spice: &str, tube_grid_fa: &str) -> (String, MnaSystem) {
    let (netlist, mut mna, config) = build_mna(spice);
    let input = config.input_node;
    let forward_active = melange_solver::pipeline::apply_forward_active_reduction(
        &mut mna,
        &netlist,
        &config,
        "",
        48000.0,
        1,
        input,
        1.0,
        &melange_solver::pipeline::silent,
    )
    .expect("FA step");
    melange_solver::pipeline::apply_grid_off_reduction(
        &mut mna,
        &netlist,
        &config,
        &forward_active,
        tube_grid_fa,
        "",
        48000.0,
        1,
        input,
        1.0,
    )
    .expect("grid-off step");
    let kernel = DkKernel::from_mna(&mna, 48000.0).expect("kernel");
    let code = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("codegen")
        .code;
    (code, mna)
}

fn generate_nodal(spice: &str) -> String {
    let (netlist, mna, config) = build_mna(spice);
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

fn count_increments(code: &str) -> usize {
    code.matches("state.diag_region_exit_count += 1;").count()
}

fn assert_counter_plumbed(code: &str, label: &str) {
    assert!(
        code.contains("pub diag_region_exit_count: u64,"),
        "{label}: diag_region_exit_count field must be declared"
    );
    assert!(
        code.contains("diag_region_exit_count: 0,"),
        "{label}: diag_region_exit_count must be initialized"
    );
    assert!(
        code.contains("self.diag_region_exit_count = 0;"),
        "{label}: diag_region_exit_count must be cleared in reset()"
    );
}

/// Compile with rustc, drive a 1 kHz sine, return the final
/// `diag_region_exit_count`.
fn compile_and_count(code: &str, num_samples: usize, amplitude: f64, tag: &str) -> u64 {
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let src_path = tmp_dir.join(format!("melange_region_exit_{tag}_{pid}.rs"));
    let bin_path = tmp_dir.join(format!("melange_region_exit_{tag}_{pid}"));

    let test_code = format!(
        "{code}\n\nfn main() {{\n\
         \x20   let mut state = CircuitState::default();\n\
         \x20   for i in 0..{num_samples}u32 {{\n\
         \x20       let t = i as f64 / 48000.0;\n\
         \x20       let input = {amplitude:?} * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
         \x20       let _ = process_sample(input, &mut state);\n\
         \x20   }}\n\
         \x20   println!(\"{{}}\", state.diag_region_exit_count);\n\
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
        .arg("-O")
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
    assert!(
        run.status.success(),
        "Binary failed for {tag}:\n{}",
        String::from_utf8_lossy(&run.stderr)
    );
    String::from_utf8_lossy(&run.stdout)
        .lines()
        .last()
        .and_then(|l| l.trim().parse().ok())
        .expect("counter line")
}

// ============================================================================
// 1. Reduction policy
// ============================================================================

/// `auto` must not reduce a pentode that the old DC-OP rule reduced.
#[test]
fn test_auto_never_reduces_cathode_biased_pentode() {
    let (netlist, mna, config) = build_mna(&pentode_cathode_biased());
    assert_eq!(mna.m, 3, "unreduced pentode is 3D");
    let grid_off = CircuitIR::detect_grid_off_pentodes(&mna, &netlist, &config, false);
    assert!(
        grid_off.is_empty(),
        "auto must keep the full 3D model; got {grid_off:?}"
    );
}

/// `on` still reduces (the warned opt-in), freezing the DC-OP Vg2k.
#[test]
fn test_on_reduces_and_freezes_dc_op_vg2k() {
    let (netlist, mna, config) = build_mna(&pentode_cathode_biased());
    let grid_off = CircuitIR::detect_grid_off_pentodes(&mna, &netlist, &config, true);
    let vg2k = *grid_off.get("P1").expect("P1 reduced under on");
    // ~291 V at this bias (300 V B+, 100 ohm screen stop, ~8 V cathode).
    assert!(
        (250.0..300.0).contains(&vg2k),
        "frozen Vg2k should be the DC-OP screen-cathode voltage, got {vg2k}"
    );
}

/// Through the shared pipeline step: `auto` and `off` are the same circuit
/// (M = 3), `on` is the reduced one (M = 2).
#[test]
fn test_pipeline_auto_equals_off_on_reduces() {
    let (_, mna_auto) = generate_dk(&pentode_cathode_biased(), "auto");
    let (_, mna_off) = generate_dk(&pentode_cathode_biased(), "off");
    let (_, mna_on) = generate_dk(&pentode_cathode_biased(), "on");
    assert_eq!(mna_auto.m, 3, "auto keeps the full 3D pentode");
    assert_eq!(mna_off.m, 3, "off keeps the full 3D pentode");
    assert_eq!(mna_on.m, 2, "on reduces to 2D");
    assert_eq!(mna_auto.n_v, mna_off.n_v, "auto and off build the same N_v");
    assert_eq!(mna_auto.n_i, mna_off.n_i, "auto and off build the same N_i");
}

/// Route parity: a reduction may never move a circuit from nodal to DK.
/// Four stages route nodal unreduced (M = 12); `on` under `--solver auto`
/// must therefore leave them alone, while a caller with no pre-route
/// (solver override "") still gets the reduction.
#[test]
fn test_on_does_not_reduce_when_unreduced_route_is_nodal() {
    let spice = four_pentodes();
    let (netlist, mut mna, config) = build_mna(&spice);
    assert_eq!(mna.m, 12);
    assert!(
        melange_solver::pipeline::should_skip_fa_for_nodal_reroute(&mna, 48000.0, 1),
        "precondition: the unreduced circuit auto-routes nodal"
    );
    let none = HashSet::new();
    let reduced = melange_solver::pipeline::apply_grid_off_reduction(
        &mut mna,
        &netlist,
        &config,
        &none,
        "on",
        "auto",
        48000.0,
        1,
        config.input_node,
        1.0,
    )
    .expect("grid-off step");
    assert!(reduced.is_empty(), "on + auto-route-nodal must not reduce");
    assert_eq!(mna.m, 12, "MNA untouched");

    let (netlist, mut mna, config) = build_mna(&spice);
    let reduced = melange_solver::pipeline::apply_grid_off_reduction(
        &mut mna,
        &netlist,
        &config,
        &none,
        "on",
        "",
        48000.0,
        1,
        config.input_node,
        1.0,
    )
    .expect("grid-off step");
    assert_eq!(reduced.len(), 4, "without a pre-route, on reduces all four");
    assert_eq!(mna.m, 8);
}

// ============================================================================
// 2. Counter emission (DK and nodal)
// ============================================================================

#[test]
fn test_dk_counter_emitted_for_full_3d_pentode() {
    let (code, mna) = generate_dk(&pentode_cathode_biased(), "auto");
    assert_eq!(mna.m, 3);
    assert_counter_plumbed(&code, "DK 3D pentode");
    assert_eq!(
        count_increments(&code),
        1,
        "one pentode -> one Vgk > 0 arm on the FULL model"
    );
    // grid (v[grid_idx]) minus cathode, both non-ground nodes here.
    assert!(
        code.contains("- v[") && code.contains(") > 0.0 { state.diag_region_exit_count += 1; }"),
        "arm must compare two node voltages"
    );
}

#[test]
fn test_dk_counter_emitted_for_grid_off_reduced_pentode() {
    let (code, mna) = generate_dk(&pentode_cathode_biased(), "on");
    assert_eq!(mna.m, 2);
    assert_counter_plumbed(&code, "DK grid-off pentode");
    assert_eq!(count_increments(&code), 1);
}

#[test]
fn test_nodal_counter_ground_cathode_uses_literal_zero() {
    let code = generate_nodal(&pentode_grounded_cathode());
    assert_eq!(count_increments(&code), 1);
    assert!(
        code.contains(" - 0.0) > 0.0 { state.diag_region_exit_count += 1; }"),
        "grounded cathode must read as the literal 0.0"
    );
}

#[test]
fn test_dk_counter_emitted_for_fa_reduced_and_full_bjt() {
    let (code_fa, mna_fa) = generate_dk(CE_SATURATING, "auto");
    assert_eq!(mna_fa.m, 1, "pure-EM CE stage FA-reduces to 1D");
    assert_counter_plumbed(&code_fa, "DK FA BJT");
    assert_eq!(
        count_increments(&code_fa),
        1,
        "FA-reduced BJT gets a Vbc arm"
    );

    // Same deck, FA skipped (solver override nodal makes the FA step a
    // no-op) -> full 2D BJT, still exactly one arm.
    let (netlist, mna, config) = build_mna(CE_SATURATING);
    assert_eq!(mna.m, 2);
    let kernel = DkKernel::from_mna(&mna, 48000.0).expect("kernel");
    let code_2d = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("codegen")
        .code;
    assert_eq!(count_increments(&code_2d), 1, "full 2D BJT gets a Vbc arm");
}

#[test]
fn test_dk_counter_absent_for_triode_only_deck() {
    let (code, _) = generate_dk(TRIODE_ONLY, "auto");
    assert_counter_plumbed(&code, "DK triode");
    assert_eq!(
        count_increments(&code),
        0,
        "triodes model grid current in 2D; no region-exit arm"
    );
}

#[test]
fn test_nodal_counter_emitted_for_pentode_and_bjt_absent_for_triode() {
    let pent = generate_nodal(&pentode_cathode_biased());
    assert_counter_plumbed(&pent, "nodal pentode");
    assert_eq!(count_increments(&pent), 1);

    let bjt = generate_nodal(CE_SATURATING);
    assert_counter_plumbed(&bjt, "nodal BJT");
    assert_eq!(count_increments(&bjt), 1);

    let tri = generate_nodal(TRIODE_ONLY);
    assert_counter_plumbed(&tri, "nodal triode");
    assert_eq!(count_increments(&tri), 0);
}

// ============================================================================
// 3. Runtime: the counter fires when the region is actually left
// ============================================================================

/// Grid driven through 0 V: the full 3D model AND the forced grid-off
/// model both count the positive half-cycles.
#[test]
#[ignore] // requires rustc
fn test_runtime_pentode_grid_conduction_counted_3d_and_grid_off() {
    let (code_3d, _) = generate_dk(&pentode_grid_driven(), "auto");
    let n3 = compile_and_count(&code_3d, 4800, 2.0, "pent3d");
    assert!(
        n3 > 1000,
        "full 3D: expected ~half the samples with Vgk > 0, got {n3}"
    );

    let (code_go, mna) = generate_dk(&pentode_grid_driven(), "on");
    assert_eq!(mna.m, 2);
    let n2 = compile_and_count(&code_go, 4800, 2.0, "pentgo");
    assert!(
        n2 > 1000,
        "grid-off: expected ~half the samples with Vgk > 0, got {n2}"
    );

    // Same deck, small signal: the grid never conducts.
    let n_small = compile_and_count(&code_3d, 4800, 0.0, "pentsmall");
    assert_eq!(n_small, 0, "no drive -> no region exit");
}

/// Pure-EM CE stage driven into saturation: the FA-reduced model reports
/// the region it is wrong in.
#[test]
#[ignore] // requires rustc
fn test_runtime_fa_bjt_saturation_counted() {
    let (code, mna) = generate_dk(CE_SATURATING, "auto");
    assert_eq!(mna.m, 1, "FA-reduced");
    let n = compile_and_count(&code, 9600, 1.0, "bjtsat");
    assert!(n > 100, "saturating drive must be counted, got {n}");
    let quiet = compile_and_count(&code, 9600, 0.001, "bjtquiet");
    assert_eq!(quiet, 0, "1 mV drive stays forward-active");
}
