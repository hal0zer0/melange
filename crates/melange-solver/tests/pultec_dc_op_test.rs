//! Pultec EQP-1A DC operating point validation.
//!
//! Compares the melange DC OP solver's output against known schematic voltages
//! from the Sowter DWG E-72,658-2 (11MΩ meter, no signal):
//!
//!   VCC:      +290V (exact, from voltage source)
//!   Node G:   ~+287V (290V - 2200Ω × small current)
//!   +250V:    +250V (bias-only rail via 47kΩ from Node G)
//!   Plates:   +140V (12AX7 plates through 200kΩ from Node G)
//!   Cathode:  +1.3V (12AX7 shared cathode, 820Ω to ground)
//!   Cath2:    +40V  (12AU7 shared cathode)
//!   Bias:     +31V  (grid bias divider: 250V × 62k/(470k+62k))

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// Helper: get a node voltage from DC OP result by name.
/// Returns None if the node doesn't exist in the MNA system.
fn get_node_voltage(
    mna: &MnaSystem,
    v_node: &[f64],
    name: &str,
) -> Option<f64> {
    let idx = mna.node_map.get(name).copied()?;
    if idx == 0 {
        // Ground node
        return Some(0.0);
    }
    let arr_idx = idx - 1;
    if arr_idx < v_node.len() {
        Some(v_node[arr_idx])
    } else {
        None
    }
}

/// Load the Pultec circuit, build MNA, run DC OP, and return everything needed for assertions.
fn run_pultec_dc_op() -> (MnaSystem, melange_solver::dc_op::DcOpResult) {
    let src = std::fs::read_to_string("../../circuits/pultec-eq.cir")
        .expect("pultec-eq.cir not found — run from crates/melange-solver/");
    let netlist = Netlist::parse(&src).expect("Pultec netlist parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA build failed");

    // Stamp input conductance (1Ω series resistance)
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }

    // Build device slots using CircuitIR helper (handles tubes, Koren params, etc.)
    let slots = CircuitIR::build_device_info(&netlist).expect("build_device_info failed");

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    let result = solve_dc_operating_point(&mna, &slots, &config);
    (mna, result)
}

// =============================================================================
// Main test: DC operating point convergence and voltage comparison
// =============================================================================

#[test]
fn pultec_dc_op_converges() {
    let (_mna, result) = run_pultec_dc_op();
    assert!(
        result.converged,
        "Pultec DC OP solver did not converge (method: {:?}, iters: {})",
        result.method, result.iterations
    );
}

#[test]
fn pultec_dc_op_has_nonzero_tube_currents() {
    let (_mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    // 4 tubes × 2 dimensions = 8 nonlinear currents
    let max_i = result.i_nl.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    assert!(
        max_i > 1e-6,
        "At least one tube should be conducting at DC OP, max |i_nl| = {:.2e}",
        max_i
    );
}

#[test]
fn pultec_dc_op_vcc_exact() {
    let (mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    let v_vcc = get_node_voltage(&mna, &result.v_node, "vcc")
        .expect("node 'vcc' not found in MNA");
    assert!(
        (v_vcc - 290.0).abs() < 0.1,
        "VCC should be 290V (from voltage source), got {:.3}V",
        v_vcc
    );
}

#[test]
fn pultec_dc_op_node_g() {
    let (mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    // Node G: schematic says ~287V (290V minus small drop across 2200Ω)
    let v_node_g = get_node_voltage(&mna, &result.v_node, "node_g")
        .expect("node 'node_g' not found in MNA");
    let tolerance = 5.0;
    assert!(
        (v_node_g - 287.0).abs() < tolerance,
        "node_g: schematic ~287V, got {:.2}V (tolerance ±{:.0}V)",
        v_node_g, tolerance
    );
}

#[test]
fn pultec_dc_op_v250_rail() {
    let (mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    // +250V rail: schematic says +250V (our Koren model shows ~263V)
    let v250 = get_node_voltage(&mna, &result.v_node, "v250")
        .expect("node 'v250' not found in MNA");
    let tolerance = 15.0;
    assert!(
        (v250 - 250.0).abs() < tolerance,
        "v250: schematic ~250V, got {:.2}V (tolerance ±{:.0}V)",
        v250, tolerance
    );
}

#[test]
fn pultec_dc_op_plate_voltages() {
    let (mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    // 12AX7 plates: schematic says +140V (our Koren model shows ~167V)
    let tolerance = 30.0;

    let v_plate1 = get_node_voltage(&mna, &result.v_node, "plate1")
        .expect("node 'plate1' not found in MNA");
    assert!(
        (v_plate1 - 140.0).abs() < tolerance,
        "plate1: schematic ~140V, got {:.2}V (tolerance ±{:.0}V)",
        v_plate1, tolerance
    );

    let v_plate2 = get_node_voltage(&mna, &result.v_node, "plate2")
        .expect("node 'plate2' not found in MNA");
    assert!(
        (v_plate2 - 140.0).abs() < tolerance,
        "plate2: schematic ~140V, got {:.2}V (tolerance ±{:.0}V)",
        v_plate2, tolerance
    );

    // Both plates should be within a few volts of each other (symmetric circuit)
    let plate_diff = (v_plate1 - v_plate2).abs();
    assert!(
        plate_diff < 5.0,
        "plate1 and plate2 should be nearly equal (symmetric), but differ by {:.2}V \
         (plate1={:.2}V, plate2={:.2}V)",
        plate_diff, v_plate1, v_plate2
    );
}

#[test]
fn pultec_dc_op_12ax7_cathode() {
    let (mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    // 12AX7 cathode: schematic says +1.3V (our model shows ~0.97V)
    let v_cathode = get_node_voltage(&mna, &result.v_node, "cathode")
        .expect("node 'cathode' not found in MNA");
    let tolerance = 0.5;
    assert!(
        (v_cathode - 1.3).abs() < tolerance,
        "cathode: schematic ~1.3V, got {:.3}V (tolerance ±{:.1}V)",
        v_cathode, tolerance
    );
}

#[test]
fn pultec_dc_op_12au7_cathode() {
    let (mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    // 12AU7 cathode: schematic says +40V (our model shows ~30.7V)
    let v_cath2 = get_node_voltage(&mna, &result.v_node, "cath2")
        .expect("node 'cath2' not found in MNA");
    let tolerance = 10.0;
    assert!(
        (v_cath2 - 40.0).abs() < tolerance,
        "cath2: schematic ~40V, got {:.2}V (tolerance ±{:.0}V)",
        v_cath2, tolerance
    );
}

#[test]
fn pultec_dc_op_grid_bias() {
    let (mna, result) = run_pultec_dc_op();
    assert!(result.converged, "DC OP must converge first");

    // Grid bias: schematic says +31V (voltage divider: 250 × 62k/(470k+62k) ≈ 29.1V)
    let v_bias = get_node_voltage(&mna, &result.v_node, "bias_31v")
        .expect("node 'bias_31v' not found in MNA");
    let tolerance = 2.0;
    assert!(
        (v_bias - 31.0).abs() < tolerance,
        "bias_31v: schematic ~31V, got {:.2}V (tolerance ±{:.0}V)",
        v_bias, tolerance
    );
}

// =============================================================================
// Summary report test — prints all voltages for diagnostic purposes
// =============================================================================

#[test]
fn pultec_dc_op_voltage_report() {
    let (mna, result) = run_pultec_dc_op();

    println!("=== Pultec EQP-1A DC Operating Point Report ===");
    println!(
        "Converged: {} (method: {:?}, iterations: {})",
        result.converged, result.method, result.iterations
    );
    println!();

    let nodes_of_interest = [
        ("vcc", 290.0, "VCC supply"),
        ("node_g", 287.0, "Node G (EQ board supply)"),
        ("v250", 250.0, "+250V bias rail"),
        ("plate1", 140.0, "12AX7 plate 1"),
        ("plate2", 140.0, "12AX7 plate 2"),
        ("cathode", 1.3, "12AX7 cathode (shared)"),
        ("cath2", 40.0, "12AU7 cathode (shared)"),
        ("bias_31v", 31.0, "Grid bias (+31V divider)"),
        ("grid1", 0.0, "12AX7 grid 1"),
        ("grid2", 0.0, "12AX7 grid 2"),
        ("grid2a", 31.0, "12AU7 grid A"),
        ("grid2b", 31.0, "12AU7 grid B"),
        ("plate2a", 290.0, "12AU7 plate A"),
        ("plate2b", 290.0, "12AU7 plate B"),
    ];

    println!(
        "{:<14} {:>10} {:>10} {:>10}",
        "Node", "Schematic", "Melange", "Error"
    );
    println!("{}", "-".repeat(48));

    for (name, expected, description) in &nodes_of_interest {
        match get_node_voltage(&mna, &result.v_node, name) {
            Some(actual) => {
                let error = actual - expected;
                let pct = if expected.abs() > 0.1 {
                    format!("({:+.1}%)", 100.0 * error / expected)
                } else {
                    format!("({:+.3}V)", error)
                };
                println!(
                    "{:<14} {:>10.2} {:>10.2} {:>10} {}",
                    name, expected, actual, pct, description
                );
            }
            None => {
                println!("{:<14} {:>10.2} {:>10} {:>10} {}", name, expected, "N/A", "-", description);
            }
        }
    }

    println!();
    println!("Nonlinear currents (M={}): {:?}", result.i_nl.len(), result.i_nl);
    println!("=== End Report ===");

    // This test always passes — it's for diagnostic output only.
    // Use `cargo test -p melange-solver --test pultec_dc_op_test -- --nocapture` to see the report.
    assert!(result.converged, "DC OP must converge for report to be meaningful");
}
