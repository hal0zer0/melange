//! Gang pot directive tests.
//!
//! Tests verify that the `.gang` directive correctly links multiple `.pot`
//! and `.wiper` entries under a single parameter.

use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// =============================================================================
// Parser tests
// =============================================================================

#[test]
fn test_parse_gang_basic() {
    let spice = r#"Gang Test
R1 in mid 50k
R2 mid out 50k
R3 out 0 50k
C1 out 0 100n
.pot R1 1k 100k
.wiper R2 R3 100k
.gang "Gain" R1 R2
"#;
    let netlist = Netlist::parse(spice).unwrap();

    assert_eq!(netlist.gangs.len(), 1);
    assert_eq!(netlist.gangs[0].label, "Gain");
    assert_eq!(netlist.gangs[0].members.len(), 2);
    assert_eq!(netlist.gangs[0].members[0].resistor_name, "R1");
    assert!(!netlist.gangs[0].members[0].inverted);
    assert_eq!(netlist.gangs[0].members[1].resistor_name, "R2");
    assert!(!netlist.gangs[0].members[1].inverted);
    assert_eq!(netlist.gangs[0].default_position, None);
}

#[test]
fn test_parse_gang_with_inversion_and_default() {
    let spice = r#"Gang Invert Test
R1 in mid 50k
R2 mid out 50k
R3 out 0 50k
C1 out 0 100n
.pot R1 1k 100k
.wiper R2 R3 100k
.gang "Gain" R1 !R2 0.75
"#;
    let netlist = Netlist::parse(spice).unwrap();

    assert_eq!(netlist.gangs[0].members[1].inverted, true);
    assert_eq!(netlist.gangs[0].default_position, Some(0.75));
}

#[test]
fn test_parse_gang_too_few_members() {
    let spice = r#"Gang One Member
R1 in out 50k
C1 out 0 100n
.pot R1 1k 100k
.gang "Solo" R1
"#;
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Gang with one member should fail");
    let err = result.unwrap_err();
    assert!(
        err.message.contains("at least two members") || err.message.contains("at least a label"),
        "Error should mention minimum members: {}",
        err.message
    );
}

#[test]
fn test_parse_gang_missing_label() {
    let spice = r#"Gang No Label
R1 in mid 50k
R2 mid out 50k
C1 out 0 100n
.pot R1 1k 100k
.pot R2 1k 100k
.gang R1 R2
"#;
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Gang without quoted label should fail");
}

// =============================================================================
// Validation tests
// =============================================================================

#[test]
fn test_gang_validation_nonexistent_pot() {
    let spice = r#"Gang Bad Ref
R1 in out 50k
C1 out 0 100n
.pot R1 1k 100k
.pot R1 1k 100k
.gang "Bad" R1 R_NOPE
"#;
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Gang referencing nonexistent pot should fail");
    if let Err(e) = result {
        assert!(
            e.message.contains("not found") || e.message.contains("Duplicate"),
            "Error: {}",
            e.message
        );
    }
}

#[test]
fn test_gang_validation_duplicate_member() {
    let spice = r#"Gang Duplicate
R1 in mid 50k
R2 mid out 50k
R3 out 0 50k
R4 in out2 50k
C1 out 0 100n
C2 out2 0 100n
.pot R1 1k 100k
.pot R4 1k 100k
.wiper R2 R3 100k
.gang "Gang1" R1 R2
.gang "Gang2" R1 R4
"#;
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Same pot in two gangs should fail"
    );
    if let Err(e) = result {
        assert!(
            e.message.contains("multiple .gang"),
            "Error should mention multiple gangs: {}",
            e.message
        );
    }
}

// =============================================================================
// MNA resolution tests
// =============================================================================

#[test]
fn test_gang_mna_resolution() {
    let spice = r#"Gang MNA Test
R1 in mid 50k
R2 mid out 50k
R3 out 0 50k
C1 out 0 100n
.pot R1 1k 100k
.wiper R2 R3 100k
.gang "Gain" R1 R2
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.gang_groups.len(), 1);
    let gang = &mna.gang_groups[0];
    assert_eq!(gang.label, "Gain");
    assert_eq!(gang.default_position, 0.5); // default when not specified

    // R1 should be a pot member
    assert_eq!(gang.pot_members.len(), 1);
    let (pot_idx, inverted) = gang.pot_members[0];
    assert!(!inverted);
    assert_eq!(mna.pots[pot_idx].name.to_ascii_uppercase(), "R1");

    // R2 is part of the wiper, so it should be a wiper member
    assert_eq!(gang.wiper_members.len(), 1);
    let (wg_idx, inverted) = gang.wiper_members[0];
    assert!(!inverted);
    assert_eq!(mna.wiper_groups[wg_idx].total_resistance, 100_000.0);
}

// =============================================================================
// Codegen tests
// =============================================================================

#[test]
fn test_gang_codegen_ir() {
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};
    use melange_solver::dk::DkKernel;

    let spice = r#"Gang Codegen Test
R1 in mid 50k
R2 mid out 50k
R3 out 0 50k
C1 out 0 100n
.pot R1 1k 100k
.wiper R2 R3 100k
.gang "Gain" R1 R2
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let node_map = mna.node_map.clone();

    let input_node_0 = node_map["in"] - 1;
    let output_node_0 = node_map["out"] - 1;

    mna.stamp_input_conductance(input_node_0, 1.0);
    let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

    // Verify gang_groups passed through to kernel
    assert_eq!(kernel.gang_groups.len(), 1);
    assert_eq!(kernel.gang_groups[0].label, "Gain");

    let config = CodegenConfig {
        circuit_name: "gang_test".to_string(),
        sample_rate: 44100.0,
        input_node: input_node_0,
        output_nodes: vec![output_node_0],
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    let result = generator.generate(&kernel, &mna, &netlist);
    assert!(result.is_ok(), "Codegen failed: {:?}", result.err());
}

#[test]
fn test_gang_pot_and_wiper_linked() {
    // Verify that a circuit with .gang links pot and wiper correctly through the MNA pipeline
    let spice = r#"Gang Linked
R_gain in mid 50k
R_blend_cw mid out 50k
R_blend_ccw out 0 50k
C1 out 0 100n
.pot R_gain 2k 102k
.wiper R_blend_cw R_blend_ccw 100k
.gang "Drive" R_gain R_blend_cw 0.3
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.gang_groups.len(), 1);
    let gang = &mna.gang_groups[0];
    assert_eq!(gang.label, "Drive");
    assert!((gang.default_position - 0.3).abs() < 1e-10);

    // Should have 1 pot member (R_gain) and 1 wiper member (the wiper group)
    assert_eq!(gang.pot_members.len(), 1);
    assert_eq!(gang.wiper_members.len(), 1);

    // The pot member should be R_gain with min=2k, max=102k
    let (pot_idx, _) = gang.pot_members[0];
    assert!((mna.pots[pot_idx].min_resistance - 2000.0).abs() < 1.0);
    assert!((mna.pots[pot_idx].max_resistance - 102000.0).abs() < 1.0);
}
