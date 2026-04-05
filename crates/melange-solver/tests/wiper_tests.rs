//! Tests for .wiper potentiometer support (3-terminal pots).

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ============================================================
// Helpers
// ============================================================

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

fn default_config() -> CodegenConfig {
    CodegenConfig {
        circuit_name: "test_wiper".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1000.0,
        ..CodegenConfig::default()
    }
}

fn generate_code(spice: &str) -> String {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let codegen = CodeGenerator::new(default_config());
    let result = codegen
        .generate(&kernel, &mna, &netlist)
        .expect("code generation failed");
    result.code
}

// ============================================================
// Test circuits
// ============================================================

/// RC voltage divider with a wiper pot — linear (M=0).
const RC_WIPER_SPICE: &str = "\
Wiper pot test
R_cw in wiper 5k
R_ccw wiper out 5k
C1 out 0 100n
R_load out 0 100k
.wiper R_cw R_ccw 10k \"Tone\"
.END
";

/// RC divider with wiper pot + regular pot — mixed.
const MIXED_POT_WIPER_SPICE: &str = "\
Mixed pot and wiper test
R_cw in wiper 5k
R_ccw wiper mid 5k
R_vol mid 0 50k
C1 mid out 100n
R_load out 0 100k
.wiper R_cw R_ccw 10k \"Tone\"
.pot R_vol 1k 100k \"Volume\"
.END
";

/// Diode clipper with wiper pot — nonlinear (M=1).
const DIODE_WIPER_SPICE: &str = "\
Diode wiper test
R_cw in wiper 5k
R_ccw wiper clip 5k
D1 clip 0 DTEST
C1 clip 0 100n
R_load clip 0 100k
.model DTEST D(IS=1e-12 N=1.5)
.wiper R_cw R_ccw 10k \"Drive\"
.END
";

// ============================================================
// Parser tests
// ============================================================

#[test]
fn test_parse_wiper_basic() {
    let netlist = Netlist::parse(RC_WIPER_SPICE).unwrap();
    assert_eq!(netlist.wipers.len(), 1);
    assert_eq!(netlist.wipers[0].resistor_cw, "R_cw");
    assert_eq!(netlist.wipers[0].resistor_ccw, "R_ccw");
    assert_eq!(netlist.wipers[0].total_resistance, 10e3);
    assert_eq!(netlist.wipers[0].label, Some("Tone".to_string()));
    // Wiper expands into 2 pot directives
    assert_eq!(netlist.pots.len(), 2);
    assert_eq!(netlist.pots[0].resistor_name, "R_cw");
    assert_eq!(netlist.pots[1].resistor_name, "R_ccw");
}

#[test]
fn test_parse_wiper_default_position() {
    let spice = "Test\nR_a 1 2 7k\nR_b 2 0 3k\n.wiper R_a R_b 10k 0.7\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.wipers[0].default_position, Some(0.7));
    let r_cw = netlist.pots[0].default_value.unwrap();
    let r_ccw = netlist.pots[1].default_value.unwrap();
    assert!(
        (r_cw + r_ccw - 10000.0).abs() < 1e-6,
        "R_cw + R_ccw must equal R_total"
    );
    // pos=0.7 → wiper near CW end → R_cw small, R_ccw large
    // With MIN_LEG_R=10: R_cw = (1-0.7)*(10000-20)+10 = 3004, R_ccw = 0.7*9980+10 = 6996
    assert!((r_cw - 3004.0).abs() < 0.1);
    assert!((r_ccw - 6996.0).abs() < 0.1);
}

#[test]
fn test_parse_wiper_with_label() {
    let spice = "Test\nR1 1 2 5k\nR2 2 0 5k\n.wiper R1 R2 10k \"HF Cut\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.wipers[0].label, Some("HF Cut".to_string()));
}

#[test]
fn test_parse_wiper_default_pos_and_label() {
    let spice = "Test\nR1 1 2 5k\nR2 2 0 5k\n.wiper R1 R2 10k 0.3 \"Tone\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.wipers[0].default_position, Some(0.3));
    assert_eq!(netlist.wipers[0].label, Some("Tone".to_string()));
}

#[test]
fn test_parse_wiper_no_label() {
    let spice = "Test\nR1 1 2 5k\nR2 2 0 5k\n.wiper R1 R2 10k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.wipers[0].label, None);
}

#[test]
fn test_parse_wiper_missing_resistor() {
    let spice = "Test\nR1 1 2 5k\n.wiper R1 R_missing 10k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    assert!(result.unwrap_err().message.contains("not found"));
}

#[test]
fn test_parse_wiper_not_a_resistor() {
    let spice = "Test\nR1 1 2 5k\nC1 2 0 10u\n.wiper R1 C1 10k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    assert!(result.unwrap_err().message.contains("must be a resistor"));
}

#[test]
fn test_parse_wiper_no_shared_node() {
    let spice = "Test\nR1 1 2 5k\nR2 3 4 5k\n.wiper R1 R2 10k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    assert!(result.unwrap_err().message.contains("do not share a node"));
}

#[test]
fn test_parse_wiper_overlap_with_pot() {
    let spice = "Test\nR1 1 2 5k\nR2 2 0 5k\n.pot R1 1k 10k\n.wiper R1 R2 10k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    assert!(result.unwrap_err().message.contains("already used"));
}

#[test]
fn test_parse_wiper_position_out_of_range() {
    let spice = "Test\nR1 1 2 5k\nR2 2 0 5k\n.wiper R1 R2 10k 1.5\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    assert!(result.unwrap_err().message.contains("between 0.0 and 1.0"));
}

#[test]
fn test_parse_wiper_same_resistor() {
    let spice = "Test\nR1 1 2 5k\n.wiper R1 R1 10k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    assert!(result.unwrap_err().message.contains("different resistors"));
}

#[test]
fn test_parse_wiper_total_r_invariant() {
    for pos_int in 0..=10 {
        let pos = pos_int as f64 / 10.0;
        let spice = format!(
            "Test\nR1 1 2 5k\nR2 2 0 5k\n.wiper R1 R2 10k {}\n",
            pos
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let r_cw = netlist.pots[0].default_value.unwrap();
        let r_ccw = netlist.pots[1].default_value.unwrap();
        assert!(
            (r_cw + r_ccw - 10000.0).abs() < 1e-6,
            "At pos={}, R_cw={} + R_ccw={} = {} != 10000",
            pos,
            r_cw,
            r_ccw,
            r_cw + r_ccw
        );
        assert!(r_cw >= 10.0, "R_cw must be >= 10.0 (MIN_LEG_R)");
        assert!(r_ccw >= 10.0, "R_ccw must be >= 10.0 (MIN_LEG_R)");
    }
}

// ============================================================
// MNA tests
// ============================================================

#[test]
fn test_mna_wiper_resolution() {
    let (netlist, mna, _) = build_pipeline(RC_WIPER_SPICE);
    assert_eq!(mna.pots.len(), 2);
    assert_eq!(mna.wiper_groups.len(), 1);
    let wg = &mna.wiper_groups[0];
    assert_eq!(wg.cw_pot_index, 0);
    assert_eq!(wg.ccw_pot_index, 1);
    assert_eq!(wg.total_resistance, 10e3);
    assert_eq!(wg.default_position, 0.5);
    assert_eq!(wg.label, Some("Tone".to_string()));
    let _ = netlist; // suppress warning
}

#[test]
fn test_mna_wiper_shared_node() {
    let (_, mna, _) = build_pipeline(RC_WIPER_SPICE);
    let cw = &mna.pots[0];
    let ccw = &mna.pots[1];
    let cw_nodes = [cw.node_p, cw.node_q];
    let ccw_nodes = [ccw.node_p, ccw.node_q];
    let shared = cw_nodes.iter().any(|n| ccw_nodes.contains(n));
    assert!(shared, "CW and CCW pots must share a node (wiper)");
}

#[test]
fn test_mna_wiper_default_g_stamp() {
    let (_, mna, _) = build_pipeline(RC_WIPER_SPICE);
    let cw = &mna.pots[0];
    let ccw = &mna.pots[1];
    // At pos=0.5: R = 0.5*(10000-20)+10 = 5000
    let expected_g = 1.0 / 5000.0;
    assert!((cw.g_nominal - expected_g).abs() < 1e-10);
    assert!((ccw.g_nominal - expected_g).abs() < 1e-10);
}

#[test]
fn test_mna_mixed_pot_and_wiper() {
    let (_, mna, _) = build_pipeline(MIXED_POT_WIPER_SPICE);
    // 1 regular pot + 2 wiper legs = 3 pots total
    assert_eq!(mna.pots.len(), 3);
    assert_eq!(mna.wiper_groups.len(), 1);
    // Regular pot is at index 0 (parsed before wiper expansion)
    assert_eq!(mna.pots[0].name.to_ascii_uppercase(), "R_VOL");
    // Wiper legs at indices 1, 2
    assert_eq!(mna.wiper_groups[0].cw_pot_index, 1);
    assert_eq!(mna.wiper_groups[0].ccw_pot_index, 2);
}

// ============================================================
// SM vector / codegen tests
// ============================================================

#[test]
fn test_wiper_sm_vectors() {
    let (_, _, kernel) = build_pipeline(RC_WIPER_SPICE);
    assert_eq!(kernel.pots.len(), 2);
    for pot in &kernel.pots {
        assert_eq!(pot.su.len(), kernel.n);
        assert!(pot.usu > 0.0, "usu should be positive");
    }
    assert_eq!(kernel.wiper_groups.len(), 1);
}

#[test]
fn test_wiper_codegen_compiles() {
    let code = generate_code(RC_WIPER_SPICE);
    assert!(code.contains("fn set_pot_0"), "should have set_pot_0 method");
    assert!(code.contains("fn set_pot_1"), "should have set_pot_1 method");
    assert!(code.contains("fn process_sample"));
}

#[test]
fn test_wiper_plus_regular_pot_compiles() {
    let code = generate_code(MIXED_POT_WIPER_SPICE);
    assert!(code.contains("fn set_pot_0"), "should have regular pot setter");
    assert!(code.contains("fn set_pot_1"), "should have CW pot setter");
    assert!(code.contains("fn set_pot_2"), "should have CCW pot setter");
}

#[test]
fn test_wiper_nonlinear_compiles() {
    let code = generate_code(DIODE_WIPER_SPICE);
    // Per-block rebuild: NR uses state.k directly, no SM NV_SU/U_NI
    assert!(
        code.contains("fn set_pot_0"),
        "should have set_pot_0 method"
    );
    assert!(
        code.contains("state.k["),
        "NR should use state.k directly"
    );
}

#[test]
fn test_wiper_ir_has_groups() {
    use melange_solver::codegen::ir::CircuitIR;
    let (netlist, mna, kernel) = build_pipeline(RC_WIPER_SPICE);
    let config = default_config();
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();
    assert_eq!(ir.wiper_groups.len(), 1);
    assert_eq!(ir.wiper_groups[0].cw_pot_index, 0);
    assert_eq!(ir.wiper_groups[0].ccw_pot_index, 1);
    assert_eq!(ir.wiper_groups[0].total_resistance, 10e3);
    assert_eq!(ir.wiper_groups[0].label, Some("Tone".to_string()));
}
