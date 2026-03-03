//! Tests for subcircuit expansion (.subckt / X instance support).

use melange_solver::parser::{Element, Netlist};
use melange_solver::mna::MnaSystem;

// ──────────────────────────────────────────────────────────────
// Expansion tests
// ──────────────────────────────────────────────────────────────

#[test]
fn test_basic_expansion() {
    let spice = r#"Test
.subckt buf in out
R1 in out 1k
.ends
X1 a b buf
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.elements.len(), 1);
    assert!(matches!(&netlist.elements[0], Element::SubcktInstance { .. }));

    netlist.expand_subcircuits().unwrap();
    assert_eq!(netlist.elements.len(), 1);
    match &netlist.elements[0] {
        Element::Resistor { name, n_plus, n_minus, value } => {
            assert_eq!(name, "X1.R1");
            assert_eq!(n_plus, "a");
            assert_eq!(n_minus, "b");
            assert_eq!(*value, 1000.0);
        }
        other => panic!("Expected Resistor, got {:?}", other),
    }
    // Subcircuit definitions should be cleared
    assert!(netlist.subcircuits.is_empty());
}

#[test]
fn test_multiple_instances() {
    let spice = r#"Test
.subckt filter in out
R1 in out 10k
C1 out 0 100n
.ends
X1 a b filter
X2 c d filter
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    assert_eq!(netlist.elements.len(), 4);

    // Check names are uniquely prefixed
    let names: Vec<&str> = netlist.elements.iter().map(|e| e.name()).collect();
    assert_eq!(names, vec!["X1.R1", "X1.C1", "X2.R1", "X2.C1"]);
}

#[test]
fn test_ground_passthrough() {
    let spice = r#"Test
.subckt grounded in
R1 in 0 1k
.ends
X1 sig grounded
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    match &netlist.elements[0] {
        Element::Resistor { n_plus, n_minus, .. } => {
            assert_eq!(n_plus, "sig");
            assert_eq!(n_minus, "0"); // Ground stays "0", NOT "X1.0"
        }
        other => panic!("Expected Resistor, got {:?}", other),
    }
}

#[test]
fn test_port_count_mismatch() {
    let spice = r#"Test
.subckt buf in out
R1 in out 1k
.ends
X1 a buf
"#;
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(msg.contains("1 nodes"), "Error: {}", msg);
    assert!(msg.contains("expects 2"), "Error: {}", msg);
}

#[test]
fn test_undefined_subcircuit() {
    let spice = r#"Test
X1 a b nonexistent
"#;
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(msg.contains("undefined subcircuit"), "Error: {}", msg);
}

#[test]
fn test_component_name_prefixing() {
    let spice = r#"Test
.subckt stage in out vcc
R1 in mid 10k
C1 mid out 100n
R2 mid vcc 100k
.ends
X1 sig_in sig_out vcc_rail stage
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();

    let names: Vec<&str> = netlist.elements.iter().map(|e| e.name()).collect();
    assert_eq!(names, vec!["X1.R1", "X1.C1", "X1.R2"]);

    // Internal node "mid" should be prefixed
    match &netlist.elements[0] {
        Element::Resistor { n_plus, n_minus, .. } => {
            assert_eq!(n_plus, "sig_in");  // port → caller's node
            assert_eq!(n_minus, "X1.mid"); // internal → prefixed
        }
        other => panic!("Expected Resistor, got {:?}", other),
    }
}

#[test]
fn test_model_references_preserved() {
    let spice = r#"Test
.subckt clipper in out
D1 in out 1N4148
.ends
.model 1N4148 D(IS=2.52e-9 N=1.752)
X1 a b clipper
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();

    match &netlist.elements[0] {
        Element::Diode { name, model, .. } => {
            assert_eq!(name, "X1.D1");
            assert_eq!(model, "1N4148"); // Model name NOT prefixed
        }
        other => panic!("Expected Diode, got {:?}", other),
    }
}

#[test]
fn test_nested_expansion() {
    let spice = r#"Test
.subckt inner in out
R1 in out 1k
.ends
.subckt outer in out
X1 in mid inner
X2 mid out inner
.ends
X1 a b outer
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();

    // Should have 2 resistors after full expansion
    assert_eq!(netlist.elements.len(), 2);
    let names: Vec<&str> = netlist.elements.iter().map(|e| e.name()).collect();
    assert_eq!(names, vec!["X1.X1.R1", "X1.X2.R1"]);

    // Check node remapping through nesting
    match &netlist.elements[0] {
        Element::Resistor { n_plus, n_minus, .. } => {
            assert_eq!(n_plus, "a");         // outer port "in" → "a"
            assert_eq!(n_minus, "X1.mid");   // outer internal "mid"
        }
        other => panic!("Expected Resistor, got {:?}", other),
    }
    match &netlist.elements[1] {
        Element::Resistor { n_plus, n_minus, .. } => {
            assert_eq!(n_plus, "X1.mid");    // outer internal "mid"
            assert_eq!(n_minus, "b");         // outer port "out" → "b"
        }
        other => panic!("Expected Resistor, got {:?}", other),
    }
}

#[test]
fn test_recursive_cycle_detection() {
    // A contains instance of B, B contains instance of A
    let spice = r#"Test
.subckt A in out
X1 in out B
.ends
.subckt B in out
X1 in out A
.ends
X1 a b A
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    let result = netlist.expand_subcircuits();
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(msg.contains("cycle"), "Error: {}", msg);
}

#[test]
fn test_self_recursive() {
    let spice = r#"Test
.subckt loop in out
X1 in out loop
.ends
X1 a b loop
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    let result = netlist.expand_subcircuits();
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(msg.contains("cycle"), "Error: {}", msg);
}

#[test]
fn test_no_subcircuits_noop() {
    let spice = r#"Test
R1 a b 1k
C1 b 0 100n
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    let before = netlist.elements.clone();
    netlist.expand_subcircuits().unwrap();
    assert_eq!(netlist.elements, before);
}

#[test]
fn test_nonlinear_device_in_subcircuit() {
    let spice = r#"Test
.subckt clipper in out
D1 in out 1N4148
D2 out in 1N4148
.ends
.model 1N4148 D(IS=2.52e-9 N=1.752)
X1 sig_in sig_out clipper
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    assert_eq!(netlist.elements.len(), 2);
    assert!(matches!(&netlist.elements[0], Element::Diode { name, .. } if name == "X1.D1"));
    assert!(matches!(&netlist.elements[1], Element::Diode { name, .. } if name == "X1.D2"));
}

#[test]
fn test_voltage_source_in_subcircuit() {
    let spice = r#"Test
.subckt powered in out vcc
V1 vcc 0 DC 9
R1 in out 1k
.ends
X1 a b rail powered
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    assert_eq!(netlist.elements.len(), 2);
    match &netlist.elements[0] {
        Element::VoltageSource { name, n_plus, n_minus, dc, .. } => {
            assert_eq!(name, "X1.V1");
            assert_eq!(n_plus, "rail");  // Port "vcc" → caller's "rail"
            assert_eq!(n_minus, "0");
            assert_eq!(*dc, Some(9.0));
        }
        other => panic!("Expected VoltageSource, got {:?}", other),
    }
}

#[test]
fn test_duplicate_subcircuit_name() {
    let spice = r#"Test
.subckt buf in out
R1 in out 1k
.ends
.subckt buf in out
R1 in out 2k
.ends
X1 a b buf
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    let result = netlist.expand_subcircuits();
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(msg.contains("Duplicate subcircuit"), "Error: {}", msg);
}

#[test]
fn test_bjt_in_subcircuit() {
    let spice = r#"Test
.subckt amp in out vcc
Q1 out in 0 2N2222
R1 vcc out 10k
.ends
.model 2N2222 NPN(IS=1e-14 BF=200)
X1 base collector supply amp
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    assert_eq!(netlist.elements.len(), 2);
    match &netlist.elements[0] {
        Element::Bjt { name, nc, nb, ne, model } => {
            assert_eq!(name, "X1.Q1");
            assert_eq!(nc, "collector");
            assert_eq!(nb, "base");
            assert_eq!(ne, "0");
            assert_eq!(model, "2N2222"); // Model not remapped
        }
        other => panic!("Expected Bjt, got {:?}", other),
    }
}

#[test]
fn test_mixed_elements_and_instances() {
    let spice = r#"Test
.subckt buf in out
R1 in out 1k
.ends
R_top in mid 100
X1 mid out buf
C_out out 0 10n
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    assert_eq!(netlist.elements.len(), 3);
    assert_eq!(netlist.elements[0].name(), "R_top");
    assert_eq!(netlist.elements[1].name(), "X1.R1");
    assert_eq!(netlist.elements[2].name(), "C_out");
}

// ──────────────────────────────────────────────────────────────
// MNA integration tests
// ──────────────────────────────────────────────────────────────

#[test]
fn test_expanded_builds_mna() {
    let spice = r#"Test
.subckt lowpass in out
R1 in out 10k
C1 out 0 100n
.ends
X1 sig_in sig_out lowpass
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(mna.n >= 2); // At least sig_in and sig_out
}

#[test]
fn test_expanded_node_count() {
    let spice = r#"Test
.subckt stage in out vcc
R1 in mid 10k
R2 mid out 10k
R3 mid vcc 100k
C1 out 0 100n
.ends
X1 a b supply stage
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Nodes: a, b, supply, X1.mid = 4 (ground "0" is implicit)
    assert_eq!(mna.n, 4);
    assert!(mna.node_map.contains_key("a"));
    assert!(mna.node_map.contains_key("b"));
    assert!(mna.node_map.contains_key("supply"));
    assert!(mna.node_map.contains_key("X1.mid"));
}

#[test]
fn test_unexpanded_mna_error() {
    let spice = r#"Test
.subckt buf in out
R1 in out 1k
.ends
X1 a b buf
"#;
    let netlist = Netlist::parse(spice).unwrap();
    // NOT calling expand_subcircuits — MNA should reject
    let result = MnaSystem::from_netlist(&netlist);
    assert!(result.is_err());
    let msg = format!("{}", result.unwrap_err());
    assert!(msg.contains("expand subcircuits"), "Error: {}", msg);
}

// ──────────────────────────────────────────────────────────────
// Full pipeline tests
// ──────────────────────────────────────────────────────────────

#[test]
fn test_rc_lowpass_subcircuit_codegen() {
    use melange_solver::codegen::{CodegenConfig, CodeGenerator};
    use melange_solver::dk::DkKernel;

    let spice = r#"RC Lowpass as Subcircuit
.subckt lowpass in out
R1 in out 10k
C1 out 0 100n
.ends
X1 in out lowpass
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();

    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let input_node_idx = mna.node_map["in"] - 1;
    let output_node_idx = mna.node_map["out"] - 1;
    mna.g[input_node_idx][input_node_idx] += 1.0;

    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    let config = CodegenConfig {
        circuit_name: "rc_lowpass_subckt".to_string(),
        input_node: input_node_idx,
        output_nodes: vec![output_node_idx],
        sample_rate: 48000.0,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    let result = generator.generate(&kernel, &mna, &netlist);
    assert!(result.is_ok(), "Codegen failed: {:?}", result.err());
    let code = result.unwrap().code;
    assert!(code.contains("process_sample"), "Generated code missing process_sample");
}

#[test]
fn test_pot_with_expanded_name() {
    let spice = r#"Test
.subckt tone in out
R1 in out 10k
C1 out 0 100n
.ends
X1 sig_in sig_out tone
.pot X1.R1 1k 100k
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();

    // After expansion, .pot should reference the expanded name X1.R1
    assert_eq!(netlist.pots[0].resistor_name, "X1.R1");

    // Verify it matches the expanded resistor
    let has_r = netlist.elements.iter().any(|e| {
        matches!(e, Element::Resistor { name, .. } if name == "X1.R1")
    });
    assert!(has_r, "Expanded resistor X1.R1 not found");

    // Should build MNA successfully
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(!mna.pots.is_empty(), "Pot should be recognized in MNA");
}

#[test]
fn test_switch_with_expanded_name() {
    let spice = r#"Test
.subckt filter in out
C1 in out 100n
.ends
X1 sig_in sig_out filter
.switch X1.C1 100n 220n 470n
"#;
    let mut netlist = Netlist::parse(spice).unwrap();
    netlist.expand_subcircuits().unwrap();

    assert_eq!(netlist.switches[0].component_names[0], "X1.C1");

    // Verify the expanded capacitor exists
    let has_c = netlist.elements.iter().any(|e| {
        matches!(e, Element::Capacitor { name, .. } if name == "X1.C1")
    });
    assert!(has_c, "Expanded capacitor X1.C1 not found");

    // Should build MNA successfully
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(!mna.switches.is_empty(), "Switch should be recognized in MNA");
}
