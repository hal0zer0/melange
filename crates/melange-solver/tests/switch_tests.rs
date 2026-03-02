//! Switch directive tests.
//!
//! Tests for the `.switch` directive: parser, MNA resolution, codegen, and compile-and-run.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use std::io::Write;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    // Stamp input conductance (1 ohm) at node 0 (first non-ground node)
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

fn generate(spice: &str) -> String {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let config = CodegenConfig {
        circuit_name: "switch_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: if kernel.n > 1 { 1 } else { 0 },
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");
    result.code
}

fn compile_and_run(code: &str, test_name: &str) {
    let tmp_dir = std::env::temp_dir();
    let src_path = tmp_dir.join(format!("melange_switch_{}.rs", test_name));
    let bin_path = tmp_dir.join(format!("melange_switch_{}", test_name));
    {
        let mut f = std::fs::File::create(&src_path).expect("create temp file");
        f.write_all(code.as_bytes()).expect("write temp file");
    }

    let compile = std::process::Command::new("rustc")
        .args([src_path.to_str().unwrap(), "-o", bin_path.to_str().unwrap(), "--edition", "2021"])
        .output()
        .expect("run rustc");

    if !compile.status.success() {
        let _ = std::fs::remove_file(&src_path);
        panic!(
            "Switch test '{}' failed to compile:\n{}",
            test_name,
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run test binary");
    let _ = std::fs::remove_file(&src_path);
    let _ = std::fs::remove_file(&bin_path);

    if !run.status.success() {
        panic!(
            "Switch test '{}' failed at runtime:\nstdout: {}\nstderr: {}",
            test_name,
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
}

// ===========================================================================
// Parser tests
// ===========================================================================

#[test]
fn test_parser_single_component_switch() {
    let spice = "\
Switch Test
R1 in out 1k
C1 out 0 100n
.switch C1 100n 220n 470n
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    assert_eq!(netlist.switches.len(), 1);
    assert_eq!(netlist.switches[0].component_names, vec!["C1"]);
    assert_eq!(netlist.switches[0].positions.len(), 3);
    assert!((netlist.switches[0].positions[0][0] - 100e-9).abs() < 1e-15);
    assert!((netlist.switches[0].positions[1][0] - 220e-9).abs() < 1e-15);
    assert!((netlist.switches[0].positions[2][0] - 470e-9).abs() < 1e-15);
}

#[test]
fn test_parser_ganged_switch() {
    let spice = "\
Ganged Switch Test
R1 in out 1k
C1 out mid 15n
L1 mid 0 176m
.switch C1,L1 15n/176m 10n/100m
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    assert_eq!(netlist.switches.len(), 1);
    assert_eq!(netlist.switches[0].component_names, vec!["C1", "L1"]);
    assert_eq!(netlist.switches[0].positions.len(), 2);
    assert!((netlist.switches[0].positions[0][0] - 15e-9).abs() < 1e-15);
    assert!((netlist.switches[0].positions[0][1] - 176e-3).abs() < 1e-10);
    assert!((netlist.switches[0].positions[1][0] - 10e-9).abs() < 1e-15);
    assert!((netlist.switches[0].positions[1][1] - 100e-3).abs() < 1e-10);
}

#[test]
fn test_parser_infix_notation() {
    let spice = "\
Infix Test
R1 in out 4k7
C1 out 0 6n8
.switch C1 6n8 3n3
";
    let netlist = Netlist::parse(spice).expect("parse failed");

    // Check R1 value: 4k7 = 4.7k = 4700
    let r1 = netlist.elements.iter().find(|e| e.name() == "R1").unwrap();
    if let melange_solver::parser::Element::Resistor { value, .. } = r1 {
        assert!((value - 4700.0).abs() < 0.01, "4k7 should be 4700, got {}", value);
    } else {
        panic!("R1 should be a resistor");
    }

    // Check C1 value: 6n8 = 6.8nF
    let c1 = netlist.elements.iter().find(|e| e.name() == "C1").unwrap();
    if let melange_solver::parser::Element::Capacitor { value, .. } = c1 {
        assert!((value - 6.8e-9).abs() < 1e-15, "6n8 should be 6.8e-9, got {}", value);
    } else {
        panic!("C1 should be a capacitor");
    }

    // Check switch positions use infix
    assert_eq!(netlist.switches.len(), 1);
    assert!((netlist.switches[0].positions[0][0] - 6.8e-9).abs() < 1e-15);
    assert!((netlist.switches[0].positions[1][0] - 3.3e-9).abs() < 1e-15);
}

#[test]
fn test_parser_rejects_fewer_than_2_positions() {
    let spice = "\
Bad Switch
R1 in out 1k
C1 out 0 100n
.switch C1 100n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject switch with fewer than 2 positions");
}

#[test]
fn test_parser_rejects_value_count_mismatch() {
    let spice = "\
Mismatch Test
R1 in out 1k
C1 out mid 15n
L1 mid 0 176m
.switch C1,L1 15n/176m 10n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject mismatched value count in ganged switch");
    let err = result.unwrap_err();
    assert!(err.message.contains("values") || err.message.contains("components"), "Error: {}", err.message);
}

#[test]
fn test_parser_rejects_duplicate_component() {
    let spice = "\
Dup Test
R1 in out 1k
C1 out 0 100n
.switch C1 100n 220n
.switch C1 470n 680n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject duplicate component across switches");
    let err = result.unwrap_err();
    assert!(err.message.contains("already"), "Error: {}", err.message);
}

#[test]
fn test_parser_rejects_nonexistent_component() {
    let spice = "\
Missing Test
R1 in out 1k
C1 out 0 100n
.switch C99 100n 220n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject nonexistent component reference");
}

#[test]
fn test_parser_rejects_invalid_prefix() {
    let spice = "\
Invalid Prefix
R1 in out 1k
C1 out 0 100n
D1 out 0 DMOD
.model DMOD D(IS=1e-14)
.switch D1 1e-14 1e-13
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject non-R/C/L component in switch");
}

// ===========================================================================
// MNA resolution tests
// ===========================================================================

#[test]
fn test_mna_switch_resolution() {
    let spice = "\
MNA Switch Test
R1 in out 1k
C1 out 0 100n
.switch C1 100n 220n 470n
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");
    assert_eq!(mna.switches.len(), 1);
    assert_eq!(mna.switches[0].components.len(), 1);
    assert_eq!(mna.switches[0].components[0].component_type, 'C');
    assert!((mna.switches[0].components[0].nominal_value - 100e-9).abs() < 1e-15);
    assert_eq!(mna.switches[0].positions.len(), 3);
}

#[test]
fn test_mna_ganged_switch_resolution() {
    let spice = "\
MNA Ganged Test
R1 in out 1k
C1 out mid 15n
L1 mid 0 176m
.switch C1,L1 15n/176m 10n/100m
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");
    assert_eq!(mna.switches.len(), 1);
    assert_eq!(mna.switches[0].components.len(), 2);
    assert_eq!(mna.switches[0].components[0].component_type, 'C');
    assert_eq!(mna.switches[0].components[1].component_type, 'L');
}

// ===========================================================================
// Codegen tests
// ===========================================================================

#[test]
fn test_codegen_switch_constants_emitted() {
    let code = generate("\
Switch Codegen Test
R1 in out 1k
C1 out 0 100n
.switch C1 100n 220n 470n
");
    assert!(code.contains("SWITCH_0_NUM_POSITIONS"), "Should emit SWITCH_0_NUM_POSITIONS");
    assert!(code.contains("SWITCH_0_VALUES"), "Should emit SWITCH_0_VALUES");
    assert!(code.contains("SWITCH_0_COMP_0_NODE_P"), "Should emit component node constants");
    assert!(code.contains("SWITCH_0_COMP_0_NOMINAL"), "Should emit nominal value constant");
}

#[test]
fn test_codegen_switch_state_fields() {
    let code = generate("\
Switch State Test
R1 in out 1k
C1 out 0 100n
.switch C1 100n 220n 470n
");
    assert!(code.contains("switch_0_position"), "Should have switch_0_position field");
    assert!(code.contains("current_sample_rate"), "Should have current_sample_rate field");
}

#[test]
fn test_codegen_set_switch_method() {
    let code = generate("\
Switch Method Test
R1 in out 1k
C1 out 0 100n
.switch C1 100n 220n 470n
");
    assert!(code.contains("fn set_switch_0"), "Should emit set_switch_0 method");
    assert!(code.contains("fn rebuild_matrices"), "Should emit rebuild_matrices method");
}

#[test]
fn test_codegen_no_switch_backward_compat() {
    // Circuit with no .switch directives should NOT emit switch code
    let code = generate("\
No Switch Test
R1 in out 1k
C1 out 0 100n
");
    assert!(!code.contains("SWITCH_"), "Should NOT emit SWITCH_ constants");
    assert!(!code.contains("switch_0"), "Should NOT emit switch fields");
    assert!(!code.contains("rebuild_matrices"), "Should NOT emit rebuild_matrices");
    assert!(!code.contains("current_sample_rate"), "Should NOT emit current_sample_rate");
}

// ===========================================================================
// Compile-and-run tests
// ===========================================================================

#[test]
fn test_compile_switched_cap() {
    let code = generate("\
Switched Cap Test
R1 in out 1k
C1 out 0 100n
.switch C1 100n 220n 470n
");
    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Process at default position (0)\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output must be finite\");\n\
             }}\n\
             \n\
             // Switch to position 1\n\
             state.set_switch_0(1);\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output after switch must be finite\");\n\
             }}\n\
             \n\
             // Switch to position 2\n\
             state.set_switch_0(2);\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output after switch 2 must be finite\");\n\
             }}\n\
             \n\
             // Invalid position should be no-op\n\
             state.set_switch_0(99);\n\
             let out = process_sample(0.1, &mut state);\n\
             assert!(out.is_finite(), \"output after invalid switch must be finite\");\n\
             \n\
             eprintln!(\"Switched cap test passed!\");\n\
         }}\n",
        code
    );
    compile_and_run(&test_harness, "switched_cap");
}

#[test]
fn test_compile_ganged_cap_inductor() {
    let code = generate("\
Ganged C+L Test
R1 in out 1k
C1 out mid 15n
L1 mid 0 176m
.switch C1,L1 15n/176m 10n/100m
");
    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Process at default position\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output must be finite\");\n\
             }}\n\
             \n\
             // Switch to position 1\n\
             state.set_switch_0(1);\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output after switch must be finite\");\n\
             }}\n\
             \n\
             // set_sample_rate should also work\n\
             state.set_sample_rate(48000.0);\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output after SR change must be finite\");\n\
             }}\n\
             \n\
             eprintln!(\"Ganged C+L test passed!\");\n\
         }}\n",
        code
    );
    compile_and_run(&test_harness, "ganged_cl");
}

#[test]
fn test_compile_switch_with_pot() {
    let spice = "\
Switch+Pot Test
R1 in mid 10k
C1 mid 0 100n
R2 mid out 1k
C2 out 0 47n
.switch C1 100n 220n
.pot R1 1 10k
";
    let (netlist, mna, kernel) = build_pipeline(spice);
    let config = CodegenConfig {
        circuit_name: "switch_pot_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: if kernel.n > 1 { 1 } else { 0 },
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");

    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Process with default settings\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output must be finite\");\n\
             }}\n\
             \n\
             // Change pot\n\
             state.pot_0_resistance = 5000.0;\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output after pot change must be finite\");\n\
             }}\n\
             \n\
             // Change switch\n\
             state.set_switch_0(1);\n\
             for _ in 0..100 {{\n\
                 let out = process_sample(0.1, &mut state);\n\
                 assert!(out.is_finite(), \"output after switch must be finite\");\n\
             }}\n\
             \n\
             eprintln!(\"Switch+Pot test passed!\");\n\
         }}\n",
        result.code
    );
    compile_and_run(&test_harness, "switch_pot");
}

#[test]
fn test_compile_switched_resistor() {
    let code = generate("\
Switched Resistor Test
R1 in out 1k
C1 out 0 100n
.switch R1 1k 2.2k 4.7k 10k
");
    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Process at each position\n\
             for pos in 0..4 {{\n\
                 state.set_switch_0(pos);\n\
                 for _ in 0..100 {{\n\
                     let out = process_sample(0.1, &mut state);\n\
                     assert!(out.is_finite(), \"output at pos {{}} must be finite\", pos);\n\
                 }}\n\
             }}\n\
             \n\
             eprintln!(\"Switched resistor test passed!\");\n\
         }}\n",
        code
    );
    compile_and_run(&test_harness, "switched_resistor");
}

#[test]
fn test_compile_multiple_switches() {
    let code = generate("\
Multi Switch Test
R1 in mid 1k
C1 mid 0 100n
R2 mid out 2.2k
C2 out 0 47n
.switch C1 100n 220n 470n
.switch C2 47n 100n 220n
");
    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Exercise all combinations of 2 switches\n\
             for p0 in 0..3 {{\n\
                 state.set_switch_0(p0);\n\
                 for p1 in 0..3 {{\n\
                     state.set_switch_1(p1);\n\
                     for _ in 0..50 {{\n\
                         let out = process_sample(0.1, &mut state);\n\
                         assert!(out.is_finite(), \"output at ({{}},{{}}) must be finite\", p0, p1);\n\
                     }}\n\
                 }}\n\
             }}\n\
             \n\
             eprintln!(\"Multi-switch test passed!\");\n\
         }}\n",
        code
    );
    compile_and_run(&test_harness, "multi_switch");
}

// ===========================================================================
// Infix notation standalone tests
// ===========================================================================

#[test]
fn test_infix_various_scales() {
    // Test various infix notations in component values
    let spice = "\
Infix Scale Test
R1 in mid 4k7
R2 mid out 1m5
C1 out 0 3n3
";
    let netlist = Netlist::parse(spice).expect("parse failed");

    // R1 = 4k7 = 4700
    if let melange_solver::parser::Element::Resistor { value, .. } = &netlist.elements[0] {
        assert!((value - 4700.0).abs() < 0.01, "4k7 = {}", value);
    }
    // R2 = 1m5 = 1.5e-3 ohm (milliohm)
    if let melange_solver::parser::Element::Resistor { value, .. } = &netlist.elements[1] {
        assert!((value - 1.5e-3).abs() < 1e-10, "1m5 = {}", value);
    }
    // C1 = 3n3 = 3.3nF
    if let melange_solver::parser::Element::Capacitor { value, .. } = &netlist.elements[2] {
        assert!((value - 3.3e-9).abs() < 1e-15, "3n3 = {}", value);
    }
}
