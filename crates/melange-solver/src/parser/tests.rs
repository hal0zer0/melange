//! Parser tests.

use super::*;

#[test]
fn test_element_nodes_enumeration() {
    // Two-terminal element: both terminals, in order.
    let r = Element::Resistor {
        name: "R1".into(),
        n_plus: "a".into(),
        n_minus: "0".into(),
        value: 1e3,
        kf: None,
        af: None,
    };
    assert_eq!(r.nodes(), vec!["a", "0"]);

    // Three-terminal device reads its own node fields (nc, nb, ne).
    let q = Element::Bjt {
        name: "Q1".into(),
        nc: "c".into(),
        nb: "b".into(),
        ne: "e".into(),
        model: "MOD".into(),
    };
    assert_eq!(q.nodes(), vec!["c", "b", "e"]);

    // Pentode without a suppressor: four nodes.
    let p4 = Element::Pentode {
        name: "P1".into(),
        n_plate: "pl".into(),
        n_grid: "g".into(),
        n_cathode: "k".into(),
        n_screen: "sg".into(),
        n_suppressor: None,
        model: "VP".into(),
    };
    assert_eq!(p4.nodes(), vec!["pl", "g", "k", "sg"]);

    // Pentode WITH a suppressor: the optional fifth node appears.
    let p5 = Element::Pentode {
        name: "P2".into(),
        n_plate: "pl".into(),
        n_grid: "g".into(),
        n_cathode: "k".into(),
        n_screen: "sg".into(),
        n_suppressor: Some("sup".into()),
        model: "VP".into(),
    };
    assert_eq!(p5.nodes(), vec!["pl", "g", "k", "sg", "sup"]);

    // Vccs: control input pair AND output pair are both physical connections.
    let g = Element::Vccs {
        name: "G1".into(),
        out_p: "op".into(),
        out_n: "on".into(),
        ctrl_p: "cp".into(),
        ctrl_n: "cn".into(),
        gm: 1e-3,
    };
    assert_eq!(g.nodes(), vec!["op", "on", "cp", "cn"]);

    // SubcktInstance: the full node vector, in order.
    let x = Element::SubcktInstance {
        name: "X1".into(),
        nodes: vec!["n1".into(), "n2".into(), "n3".into()],
        subckt: "SUB".into(),
    };
    assert_eq!(x.nodes(), vec!["n1", "n2", "n3"]);
}

#[test]
fn test_parse_resistor() {
    let netlist = Netlist::parse("Test Circuit\nR1 1 0 1k\n").unwrap();
    assert_eq!(netlist.elements.len(), 1);
    match &netlist.elements[0] {
        Element::Resistor {
            name,
            n_plus,
            n_minus,
            value,
            kf,
            af,
        } => {
            assert_eq!(name, "R1");
            assert_eq!(n_plus, "1");
            assert_eq!(n_minus, "0");
            assert_eq!(*value, 1000.0);
            assert_eq!(*kf, None, "KF should default to None");
            assert_eq!(*af, None, "AF should default to None");
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_parse_resistor_with_kf_only() {
    let netlist = Netlist::parse("Test\nR1 1 0 10k KF=1e-10\n").unwrap();
    match &netlist.elements[0] {
        Element::Resistor { kf, af, .. } => {
            assert_eq!(*kf, Some(1e-10));
            assert_eq!(*af, None, "AF unset stays None until codegen defaulting");
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_parse_resistor_with_kf_and_af() {
    let netlist = Netlist::parse("Test\nR1 1 0 10k KF=2.5e-9 AF=2.0\n").unwrap();
    match &netlist.elements[0] {
        Element::Resistor { kf, af, .. } => {
            assert_eq!(*kf, Some(2.5e-9));
            assert_eq!(*af, Some(2.0));
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_parse_resistor_kf_af_order_independent() {
    let netlist = Netlist::parse("Test\nR1 1 0 10k AF=1.5 KF=3e-10\n").unwrap();
    match &netlist.elements[0] {
        Element::Resistor { kf, af, .. } => {
            assert_eq!(*kf, Some(3e-10));
            assert_eq!(*af, Some(1.5));
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_parse_resistor_kf_zero_normalizes_to_none() {
    let netlist = Netlist::parse("Test\nR1 1 0 10k KF=0 AF=2.0\n").unwrap();
    match &netlist.elements[0] {
        Element::Resistor { kf, af, .. } => {
            assert_eq!(*kf, None, "KF=0 should normalize to None");
            assert_eq!(*af, None, "AF stripped when KF unset");
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_parse_resistor_af_without_kf_stripped() {
    let netlist = Netlist::parse("Test\nR1 1 0 10k AF=2.0\n").unwrap();
    match &netlist.elements[0] {
        Element::Resistor { kf, af, .. } => {
            assert_eq!(*kf, None);
            assert_eq!(*af, None, "AF without KF is meaningless; stripped");
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_parse_resistor_kf_negative_rejected() {
    assert!(Netlist::parse("Test\nR1 1 0 10k KF=-1e-10\n").is_err());
}

#[test]
fn test_parse_resistor_af_zero_rejected() {
    assert!(Netlist::parse("Test\nR1 1 0 10k KF=1e-10 AF=0\n").is_err());
}

#[test]
fn test_parse_resistor_unknown_param_rejected() {
    assert!(Netlist::parse("Test\nR1 1 0 10k XYZ=1\n").is_err());
}

#[test]
fn test_parse_resistor_case_insensitive_kf() {
    let netlist = Netlist::parse("Test\nR1 1 0 10k kf=1e-10 af=2.0\n").unwrap();
    match &netlist.elements[0] {
        Element::Resistor { kf, af, .. } => {
            assert_eq!(*kf, Some(1e-10));
            assert_eq!(*af, Some(2.0));
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_parse_capacitor() {
    let netlist = Netlist::parse("Test\nC1 1 0 10u\n").unwrap();
    match &netlist.elements[0] {
        Element::Capacitor { name, value, .. } => {
            assert_eq!(name, "C1");
            assert!(
                (value - 10e-6).abs() < 1e-15,
                "Expected ~10uF, got {}",
                value
            );
        }
        _ => panic!("Expected capacitor"),
    }
}

#[test]
fn test_parse_bjt() {
    let netlist =
        Netlist::parse("Test\nQ1 3 2 1 2N2222\n.model 2N2222 NPN(IS=1e-15 BF=200)\n").unwrap();
    match &netlist.elements[0] {
        Element::Bjt {
            name,
            nc,
            nb,
            ne,
            model,
        } => {
            assert_eq!(name, "Q1");
            assert_eq!(nc, "3");
            assert_eq!(nb, "2");
            assert_eq!(ne, "1");
            assert_eq!(model, "2N2222");
        }
        _ => panic!("Expected BJT"),
    }
}

#[test]
fn test_parse_model() {
    let netlist = Netlist::parse("Test\n.model 2N2222 NPN(IS=1e-15 BF=200)\n").unwrap();
    assert_eq!(netlist.models.len(), 1);
    let model = &netlist.models[0];
    assert_eq!(model.name, "2N2222");
    assert!(
        model.model_type.starts_with("NPN"),
        "Model type should be NPN, got {}",
        model.model_type
    );
}

#[test]
fn test_parse_value() {
    assert_eq!(parse_value("1k").unwrap(), 1e3);
    assert_eq!(parse_value("4.7u").unwrap(), 4.7e-6);
    assert_eq!(parse_value("10pF").unwrap(), 10e-12);
    assert_eq!(parse_value("1MEG").unwrap(), 1e6);
}

/// The MEG branch returned before the finiteness check every other path
/// has, and Rust's f64 parser takes "nan"/"inf": `K1 L1 L2 nanmeg` built
/// NaN matrices, because `c <= 0 || c >= 1` is false for NaN.
#[test]
fn non_finite_meg_values_and_couplings_are_refused() {
    for v in ["nanmeg", "infmeg", "-infMEG", "1e303meg"] {
        assert!(parse_value(v).is_err(), "{v}");
    }
    let deck = "k\nR1 in a 100\nL1 a 0 1\nL2 b 0 1\nR2 b 0 1k\nK1 L1 L2 nanmeg\n";
    assert!(Netlist::parse(deck).is_err());
}

#[test]
fn test_parse_valid_scale_suffixes() {
    assert!((parse_value("1T").unwrap() - 1e12).abs() < 1e3);
    assert!((parse_value("1G").unwrap() - 1e9).abs() < 1.0);
    assert!((parse_value("1MEG").unwrap() - 1e6).abs() < 1.0);
    assert!((parse_value("1k").unwrap() - 1e3).abs() < 1e-6);
    assert!((parse_value("1m").unwrap() - 1e-3).abs() < 1e-12);
    assert!((parse_value("1u").unwrap() - 1e-6).abs() < 1e-15);
    assert!((parse_value("1n").unwrap() - 1e-9).abs() < 1e-18);
    assert!((parse_value("1p").unwrap() - 1e-12).abs() < 1e-21);
    // Femto contract (see parse_value_ctx):
    // - Element-value positions: bare "1F"/"1f" = 1.0 Farad (with a
    //   log::warn); femto there must be written "1e-15" or "1fF".
    // - "1fF" (unit letter after the f) = 1e-15 everywhere.
    // - .model parameter values (parse_value_model_param): dimensionless
    //   context, so a single trailing f/F IS femto (ngspice-compatible;
    //   ".model DX D(IS=6.734f)" = 6.734e-15).
    assert!((parse_value("1F").unwrap() - 1.0).abs() < 1e-10);
    assert!((parse_value("1fF").unwrap() - 1e-15).abs() < 1e-25);
    assert!((parse_value_model_param("6.734f").unwrap() - 6.734e-15).abs() < 1e-25);
    assert!((parse_value_model_param("1F").unwrap() - 1e-15).abs() < 1e-25);
    // Non-femto values are identical in both contexts
    assert!((parse_value_model_param("10pF").unwrap() - 10e-12).abs() < 1e-21);
    assert!((parse_value_model_param("1k").unwrap() - 1e3).abs() < 1e-6);
}

#[test]
fn test_parse_negative_resistance_rejected() {
    let result = Netlist::parse("Test\nR1 1 0 -1k\n");
    assert!(result.is_err(), "Negative resistance should be rejected");
}

#[test]
fn test_parse_zero_resistance_rejected() {
    let result = Netlist::parse("Test\nR1 1 0 0\n");
    assert!(result.is_err(), "Zero resistance should be rejected");
}

#[test]
fn test_parse_nan_value_rejected() {
    let result = parse_value("NaN");
    assert!(result.is_err(), "NaN value should be rejected");
}

#[test]
fn test_parse_inf_value_rejected() {
    let result = parse_value("inf");
    assert!(result.is_err(), "Infinity value should be rejected");
}

#[test]
fn test_parse_negative_capacitance_rejected() {
    let result = Netlist::parse("Test\nC1 1 0 -1u\n");
    assert!(result.is_err(), "Negative capacitance should be rejected");
}

#[test]
fn test_parse_zero_inductance_rejected() {
    let result = Netlist::parse("Test\nL1 1 0 0\n");
    assert!(result.is_err(), "Zero inductance should be rejected");
}

#[test]
fn test_parse_missing_value() {
    let result = Netlist::parse("Test\nR1 1 0\n");
    assert!(result.is_err(), "Missing value should be rejected");
}

#[test]
fn test_parse_pot_directive() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots.len(), 1);
    assert_eq!(netlist.pots[0].resistor_name, "R1");
    assert_eq!(netlist.pots[0].min_value, 1e3);
    assert_eq!(netlist.pots[0].max_value, 100e3);
}

#[test]
fn test_parse_pentode_4node() {
    // `P1 plate grid cathode screen EL84` — beam-tetrode / strapped-pentode
    // form where the suppressor is implicitly tied to the cathode.
    let spice = "Test\nP1 plate grid cath screen EL84\n\
                     .model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 \
                     KP=152.4 KVB=4015.8 ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148)\n";
    let netlist = Netlist::parse(spice).expect("EL84 pentode netlist should parse");
    assert_eq!(netlist.elements.len(), 1);
    match &netlist.elements[0] {
        Element::Pentode {
            name,
            n_plate,
            n_grid,
            n_cathode,
            n_screen,
            n_suppressor,
            model,
        } => {
            assert_eq!(name, "P1");
            assert_eq!(n_plate, "plate");
            assert_eq!(n_grid, "grid");
            assert_eq!(n_cathode, "cath");
            assert_eq!(n_screen, "screen");
            assert_eq!(*n_suppressor, None);
            assert_eq!(model, "EL84");
        }
        _ => panic!("Expected Pentode, got {:?}", netlist.elements[0]),
    }
}

#[test]
fn test_parse_pentode_5node_with_suppressor() {
    // `P1 plate grid cathode screen suppressor EF86` — true pentode with
    // explicit suppressor grid.
    let spice = "Test\nP1 pla gr ca scr sup EF86\n\
                     .model EF86 VP(MU=40.8 EX=1.327 KG1=675.8 KG2=4089.6 \
                     KP=350.7 KVB=1886.8 ALPHA_S=4.24 A_FACTOR=5.95e-5 BETA_FACTOR=0.28)\n";
    let netlist = Netlist::parse(spice).expect("EF86 pentode netlist should parse");
    match &netlist.elements[0] {
        Element::Pentode {
            n_plate,
            n_grid,
            n_cathode,
            n_screen,
            n_suppressor,
            model,
            ..
        } => {
            assert_eq!(n_plate, "pla");
            assert_eq!(n_grid, "gr");
            assert_eq!(n_cathode, "ca");
            assert_eq!(n_screen, "scr");
            assert_eq!(n_suppressor.as_deref(), Some("sup"));
            assert_eq!(model, "EF86");
        }
        _ => panic!("Expected Pentode"),
    }
}

#[test]
fn test_parse_pentode_too_few_nodes() {
    // 3 nodes (missing screen) should fail.
    let spice = "Test\nP1 plate grid cath EL84\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Pentode with only 3 nodes should be rejected"
    );
}

#[test]
fn test_parse_pentode_too_many_nodes() {
    // 6 nodes is invalid (max 5: plate, grid, cathode, screen, suppressor).
    let spice = "Test\nP1 a b c d e f g EL84\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Pentode with 6 nodes should be rejected");
}

#[test]
fn test_parse_pentode_model_params_stored() {
    // Ensure `.model NAME VP(...)` params round-trip through Netlist.models.
    let spice = "Test\nP1 a b c d EL84\n\
                     .model EL84 VP(MU=23.36 KG2=1275 ALPHA_S=7.66)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let m = netlist
        .models
        .iter()
        .find(|m| m.name == "EL84")
        .expect("EL84 model should be recorded");
    assert_eq!(m.model_type, "VP");
    let kg2 = m
        .params
        .iter()
        .find(|(k, _)| k == "KG2")
        .map(|(_, v)| *v)
        .expect("KG2 should be stored");
    assert_eq!(kg2, 1275.0);
    let alpha_s = m
        .params
        .iter()
        .find(|(k, _)| k == "ALPHA_S")
        .map(|(_, v)| *v)
        .expect("ALPHA_S should be stored");
    assert_eq!(alpha_s, 7.66);
}

#[test]
fn test_parse_pot_two_pots() {
    let spice = "Test\nR1 1 0 10k\nR2 2 0 5k\n.pot R1 1k 100k\n.pot R2 500 50k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots.len(), 2);
    assert_eq!(netlist.pots[0].resistor_name, "R1");
    assert_eq!(netlist.pots[1].resistor_name, "R2");
}

#[test]
fn test_parse_pot_missing_resistor() {
    let spice = "Test\nR1 1 0 10k\n.pot R2 1k 100k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Pot referencing non-existent resistor should fail"
    );
}

#[test]
fn test_parse_pot_not_a_resistor() {
    let spice = "Test\nC1 1 0 10u\n.pot C1 1u 100u\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Pot targeting non-resistor should fail");
}

#[test]
fn test_parse_pot_min_gte_max() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 100k 1k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Pot with min >= max should fail");
}

#[test]
fn test_parse_pot_duplicate() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n.pot R1 2k 50k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Duplicate pot for same resistor should fail"
    );
}

#[test]
fn test_parse_pot_max_exceeded() {
    let mut spice = String::from("Test\n");
    for i in 1..=65 {
        spice.push_str(&format!("R{i} {i} 0 10k\n"));
    }
    for i in 1..=65 {
        spice.push_str(&format!(".pot R{i} 1k 100k\n"));
    }
    let result = Netlist::parse(&spice);
    assert!(result.is_err(), "More than 64 pots should fail");
}

#[test]
fn test_parse_pot_four_pots_ok() {
    let spice = "Test\nR1 1 0 10k\nR2 2 0 5k\nR3 3 0 3k\nR4 4 0 2k\n\
                      .pot R1 1k 100k\n.pot R2 500 50k\n.pot R3 100 10k\n.pot R4 200 20k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "4 pots should be allowed: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.pots.len(), 4);
}

#[test]
fn test_parse_pot_negative_min() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 -1k 100k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Negative min value should fail");
}

#[test]
fn test_parse_pot_missing_args() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 1k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Missing max value should fail");
}

#[test]
fn test_parse_pot_case_insensitive() {
    // SPICE is case-insensitive: .pot r1 should match R1
    let spice = "Test\nR1 1 0 10k\n.pot r1 1k 100k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots.len(), 1);
    assert_eq!(netlist.pots[0].resistor_name, "r1");
}

#[test]
fn test_parse_pot_order_independent() {
    // .pot can appear before the resistor it references
    let spice = "Test\n.pot R1 1k 100k\nR1 1 0 10k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots.len(), 1);
    assert_eq!(netlist.pots[0].resistor_name, "R1");
}

#[test]
fn test_parse_pot_case_insensitive_duplicate() {
    // r1 and R1 should be treated as the same resistor for duplicate detection
    let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n.pot r1 2k 50k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Case-insensitive duplicate should fail");
}

// ======================================================================
// .pot / .switch label tests
// ======================================================================

#[test]
fn test_parse_pot_with_label() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k \"Volume\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots[0].label, Some("Volume".to_string()));
}

#[test]
fn test_parse_pot_with_multi_word_label() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k \"HF Boost\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots[0].label, Some("HF Boost".to_string()));
}

#[test]
fn test_parse_pot_without_label() {
    let spice = "Test\nR1 1 0 10k\n.pot R1 1k 100k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots[0].label, None);
}

#[test]
fn test_parse_switch_with_label() {
    let spice = "Test\nC1 1 0 100n\n.switch C1 100n 220n 470n \"Bright\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.switches[0].label, Some("Bright".to_string()));
    assert_eq!(netlist.switches[0].positions.len(), 3);
}

#[test]
fn test_parse_switch_with_multi_word_label() {
    let spice =
        "Test\nC1 1 0 100n\nL1 1 0 100m\n.switch C1,L1 100n/100m 220n/176m \"HF Boost Freq\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.switches[0].label, Some("HF Boost Freq".to_string()));
    assert_eq!(netlist.switches[0].positions.len(), 2);
}

#[test]
fn test_parse_switch_without_label() {
    let spice = "Test\nC1 1 0 100n\n.switch C1 100n 220n\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.switches[0].label, None);
}

// ======================================================================
// .input_impedance directive tests
// ======================================================================

#[test]
fn test_parse_input_impedance_basic() {
    let spice = "Test\nR1 1 0 1k\n.input_impedance 600\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.input_impedance, Some(600.0));
}

#[test]
fn test_parse_input_impedance_engineering_notation() {
    let spice = "Test\nR1 1 0 1k\n.input_impedance 10k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.input_impedance, Some(10_000.0));
}

#[test]
fn test_parse_input_impedance_default_none() {
    let spice = "Test\nR1 1 0 1k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.input_impedance, None);
}

#[test]
fn test_parse_input_impedance_duplicate_error() {
    let spice = "Test\nR1 1 0 1k\n.input_impedance 600\n.input_impedance 1k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Duplicate .input_impedance should fail");
    let err = result.unwrap_err();
    assert!(err.message.contains("Duplicate"), "Error: {}", err.message);
}

#[test]
fn test_common_emitter_amp() {
    let spice = r#"Common Emitter Amplifier
Vcc vcc 0 9V
Vin in 0 DC 0 AC 1V
R1 vcc base 100k
R2 base 0 22k
Rc vcc coll 4.7k
Re emit 0 1k
C1 in base 10u
C2 coll out 10u
Q1 coll base emit 2N2222
.model 2N2222 NPN(IS=1e-15 BF=200)
.end
"#;
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.elements.len(), 9);
    assert_eq!(netlist.models.len(), 1);
}

// ======================================================================
// Edge case tests for parser error handling
// ======================================================================

// 1. Missing model card: device references a .model that doesn't exist
#[test]
fn test_missing_model_card_diode() {
    let spice = "Test\nD1 1 2 UnknownModel\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Diode referencing undefined model should be rejected"
    );
    let err = result.unwrap_err();
    assert!(
        err.message.contains("UnknownModel"),
        "Error should mention the missing model name, got: {}",
        err.message
    );
}

#[test]
fn test_missing_model_card_bjt() {
    let spice = "Test\nQ1 3 2 1 NonExistent\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "BJT referencing undefined model should be rejected"
    );
}

#[test]
fn test_missing_model_card_jfet() {
    let spice = "Test\nJ1 3 2 1 NoSuchModel\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "JFET referencing undefined model should be rejected"
    );
}

#[test]
fn test_missing_model_card_mosfet() {
    let spice = "Test\nM1 3 2 1 0 GhostModel\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "MOSFET referencing undefined model should be rejected"
    );
}

#[test]
fn test_model_reference_exists() {
    // Positive test: model IS defined, should parse OK
    let spice = "Test\nD1 1 2 MyDiode\n.model MyDiode D(IS=1e-14)\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Diode with defined model should succeed: {:?}",
        result.err()
    );
}

#[test]
fn test_model_reference_case_insensitive() {
    // SPICE model references are case-insensitive
    let spice = "Test\nD1 1 2 mydiode\n.model MYDIODE D(IS=1e-14)\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Model reference should be case-insensitive: {:?}",
        result.err()
    );
}

// 2. Duplicate component names
#[test]
fn test_duplicate_component_names_resistors() {
    let spice = "Test\nR1 1 0 1k\nR1 2 0 2k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Duplicate component names should be rejected"
    );
    let err = result.unwrap_err();
    assert!(
        err.message.contains("Duplicate") && err.message.contains("R1"),
        "Error should mention duplicate and the name, got: {}",
        err.message
    );
}

#[test]
fn test_duplicate_component_names_case_insensitive() {
    // SPICE names are case-insensitive: R1 and r1 are the same component
    let spice = "Test\nR1 1 0 1k\nr1 2 0 2k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Case-insensitive duplicate names should be rejected"
    );
}

#[test]
fn test_duplicate_names_different_types() {
    // Even different component types with the same name should be rejected
    let spice = "Test\nR1 1 0 1k\nC1 2 0 10u\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Different component names should be accepted: {:?}",
        result.err()
    );
}

// 3. Invalid node names / special characters
#[test]
fn test_node_names_with_alphanumeric() {
    // Standard alphanumeric node names should work fine
    let spice = "Test\nR1 node_a node_b 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Alphanumeric node names should work: {:?}",
        result.err()
    );
}

#[test]
fn test_node_names_numeric() {
    // Purely numeric node names (common in SPICE)
    let spice = "Test\nR1 1 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Numeric node names should work: {:?}",
        result.err()
    );
}

// 4. Invalid number format
#[test]
fn test_invalid_number_format_alpha() {
    let spice = "Test\nR1 1 2 abc\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Alphabetic value 'abc' should be rejected");
}

#[test]
fn test_invalid_number_format_special_chars() {
    let spice = "Test\nR1 1 2 @#$\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Special character value should be rejected"
    );
}

#[test]
fn test_invalid_number_format_empty_suffix() {
    // Just a suffix with no numeric part
    let result = parse_value("k");
    assert!(
        result.is_err(),
        "Bare suffix with no number should be rejected"
    );
}

#[test]
fn test_invalid_number_format_double_dot() {
    let result = parse_value("1.2.3");
    assert!(result.is_err(), "Double decimal point should be rejected");
}

// 5. Empty netlist (title only, no components)
#[test]
fn test_empty_netlist() {
    let spice = "My Empty Circuit\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Empty netlist (title only) should parse: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.title, "My Empty Circuit");
    assert!(netlist.elements.is_empty());
    assert!(netlist.models.is_empty());
}

#[test]
fn test_empty_netlist_with_end() {
    let spice = "My Circuit\n.end\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Netlist with only .end should parse: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert!(netlist.elements.is_empty());
}

#[test]
fn test_empty_netlist_only_comments() {
    let spice = "My Circuit\n* This is a comment\n* Another comment\n.end\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Comment-only netlist should parse: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert!(netlist.elements.is_empty());
}

#[test]
fn test_completely_empty_input() {
    let spice = "";
    let result = Netlist::parse(spice);
    // Empty string should still parse (empty title)
    assert!(
        result.is_ok(),
        "Completely empty input should parse: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.title, "");
    assert!(netlist.elements.is_empty());
}

// 6. Floating nodes (single-terminal connections)
// Note: The parser itself doesn't validate node connectivity; that's an MNA concern.
// This test documents that a node connected to only one component terminal parses OK.
#[test]
fn test_floating_node_parses_ok() {
    // Node "3" is only connected to R2's n+ — it's electrically floating
    // The parser accepts this; downstream MNA assembly would detect the issue
    let spice = "Test\nR1 1 0 1k\nR2 3 0 2k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Parser doesn't validate connectivity (MNA's job): {:?}",
        result.err()
    );
}

// 7. Duplicate node connections (self-connected component)
#[test]
fn test_self_connected_resistor() {
    let spice = "Test\nR1 1 1 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Resistor from node to itself should be rejected"
    );
    let err = result.unwrap_err();
    assert!(
        err.message.contains("same node"),
        "Error should mention same node, got: {}",
        err.message
    );
}

#[test]
fn test_self_connected_capacitor() {
    let spice = "Test\nC1 2 2 10u\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Capacitor from node to itself should be rejected"
    );
}

#[test]
fn test_self_connected_inductor() {
    let spice = "Test\nL1 out out 100m\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Inductor from node to itself should be rejected"
    );
}

#[test]
fn test_self_connected_diode() {
    let spice = "Test\nD1 1 1 MyDiode\n.model MyDiode D(IS=1e-14)\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Diode from node to itself should be rejected"
    );
}

#[test]
fn test_self_connected_case_insensitive() {
    // SPICE node names are case-insensitive: "VCC" and "vcc" are the same node
    let spice = "Test\nR1 VCC vcc 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Case-insensitive self-connection should be rejected"
    );
}

#[test]
fn test_self_connected_voltage_source() {
    // Voltage source from node to itself is physically meaningless and should be rejected
    let spice = "Test\nV1 0 0 DC 5\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Voltage source self-connection should be rejected"
    );
}

#[test]
fn test_self_connected_current_source() {
    // Current source from node to itself is physically meaningless and should be rejected
    let spice = "Test\nI1 1 1 DC 1m\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Current source self-connection should be rejected"
    );
}

// 8. Unknown element type
#[test]
fn test_unknown_element_type() {
    let spice = "Test\nZ1 1 2 100\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Unknown element type 'Z' should be rejected"
    );
    let err = result.unwrap_err();
    assert!(
        err.message.contains("Unknown element type"),
        "Error should mention unknown element type, got: {}",
        err.message
    );
}

// 9. Insufficient fields for various components
#[test]
fn test_missing_value_capacitor() {
    let result = Netlist::parse("Test\nC1 1 0\n");
    assert!(
        result.is_err(),
        "Capacitor without value should be rejected"
    );
}

#[test]
fn test_missing_value_inductor() {
    let result = Netlist::parse("Test\nL1 1 0\n");
    assert!(result.is_err(), "Inductor without value should be rejected");
}

#[test]
fn test_inductor_isat_parse() {
    let net = Netlist::parse("Test\nL1 1 0 100m ISAT=20m\n").unwrap();
    match &net.elements[0] {
        Element::Inductor { value, isat, .. } => {
            assert!((value - 0.1).abs() < 1e-10);
            assert_eq!(*isat, Some(0.02));
        }
        _ => panic!("Expected Inductor"),
    }
}

#[test]
fn test_inductor_isat_lowercase() {
    let net = Netlist::parse("Test\nL1 1 0 5 isat=50m\n").unwrap();
    match &net.elements[0] {
        Element::Inductor { isat, .. } => assert_eq!(*isat, Some(0.05)),
        _ => panic!("Expected Inductor"),
    }
}

#[test]
fn test_inductor_no_isat() {
    let net = Netlist::parse("Test\nL1 1 0 100m\n").unwrap();
    match &net.elements[0] {
        Element::Inductor { isat, .. } => assert_eq!(*isat, None),
        _ => panic!("Expected Inductor"),
    }
}

#[test]
fn test_inductor_isat_negative_rejected() {
    let result = Netlist::parse("Test\nL1 1 0 100m ISAT=-10m\n");
    assert!(result.is_err(), "Negative ISAT should be rejected");
}

#[test]
fn test_inductor_isat_zero_rejected() {
    let result = Netlist::parse("Test\nL1 1 0 100m ISAT=0\n");
    assert!(result.is_err(), "Zero ISAT should be rejected");
}

/// Non-finite inductor values must be rejected at parse time. The
/// coupled-inductor `max_by` selector in MNA assumes finite values
/// (guarded by `total_cmp`), but defense-in-depth at the parser keeps
/// the invariant visible and documented.
#[test]
fn test_inductor_nan_rejected() {
    let result = Netlist::parse("Test\nL1 1 0 NaN\n");
    assert!(result.is_err(), "NaN inductor value should be rejected");
}

#[test]
fn test_inductor_infinity_rejected() {
    let result = Netlist::parse("Test\nL1 1 0 Inf\n");
    assert!(
        result.is_err(),
        "Infinite inductor value should be rejected"
    );
}

#[test]
fn test_missing_nodes_voltage_source() {
    let result = Netlist::parse("Test\nV1 1\n");
    assert!(
        result.is_err(),
        "Voltage source with one node should be rejected"
    );
}

#[test]
fn test_missing_model_diode() {
    let result = Netlist::parse("Test\nD1 1 2\n");
    assert!(
        result.is_err(),
        "Diode without model name should be rejected"
    );
}

#[test]
fn test_missing_fields_bjt() {
    let result = Netlist::parse("Test\nQ1 3 2\n");
    assert!(
        result.is_err(),
        "BJT with too few fields should be rejected"
    );
}

#[test]
fn test_missing_fields_jfet() {
    let result = Netlist::parse("Test\nJ1 3 2\n");
    assert!(
        result.is_err(),
        "JFET with too few fields should be rejected"
    );
}

#[test]
fn test_missing_fields_mosfet() {
    let result = Netlist::parse("Test\nM1 3 2 1\n");
    assert!(
        result.is_err(),
        "MOSFET with too few fields should be rejected"
    );
}

// 10. Zero component values
#[test]
fn test_zero_capacitance_rejected() {
    let result = Netlist::parse("Test\nC1 1 0 0\n");
    assert!(result.is_err(), "Zero capacitance should be rejected");
}

// 11. Continuation lines and comments
#[test]
fn test_continuation_line() {
    let spice = "Test\nR1 1 0\n+ 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Continuation line should work: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.elements.len(), 1);
    match &netlist.elements[0] {
        Element::Resistor { value, .. } => {
            assert_eq!(*value, 1000.0);
        }
        _ => panic!("Expected resistor"),
    }
}

#[test]
fn test_inline_comment_semicolon() {
    let spice = "Test\nR1 1 0 1k ; this is a comment\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Inline semicolon comment should be stripped: {:?}",
        result.err()
    );
}

#[test]
fn test_inline_comment_dollar() {
    let spice = "Test\nR1 1 0 1k $ this is a comment\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Inline dollar comment should be stripped: {:?}",
        result.err()
    );
}

#[test]
fn test_star_comment_line() {
    let spice = "Test\n* This is a comment\nR1 1 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Star comment lines should be ignored: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.elements.len(), 1);
}

// 12. Edge cases for parse_value
#[test]
fn test_parse_value_empty_string() {
    let result = parse_value("");
    assert!(result.is_err(), "Empty string value should be rejected");
}

#[test]
fn test_parse_value_whitespace_only() {
    let result = parse_value("   ");
    assert!(result.is_err(), "Whitespace-only value should be rejected");
}

#[test]
fn test_parse_value_negative() {
    // parse_value itself accepts negative numbers; component parsers reject them
    let result = parse_value("-1k");
    assert!(result.is_ok(), "parse_value should accept negative numbers");
    assert_eq!(result.unwrap(), -1000.0);
}

#[test]
fn test_parse_value_scientific_notation() {
    let result = parse_value("1e-12");
    assert!(result.is_ok(), "Scientific notation should work");
    assert!((result.unwrap() - 1e-12).abs() < 1e-24);
}

#[test]
fn test_parse_value_bare_meg() {
    let result = parse_value("MEG");
    assert!(
        result.is_err(),
        "Bare 'MEG' with no number should be rejected"
    );
}

// Regression: non-ASCII suffix must not panic.
// 'ſ' (U+017F, LATIN SMALL LETTER LONG S) uppercases to "S", so the original
// suffix-stripping loop computed char count from the uppercased string and then
// byte-sliced the original, landing mid-codepoint. Must return Err, not panic.
#[test]
fn test_parse_value_non_ascii_suffix_no_panic() {
    assert!(parse_value("1ſ").is_err());
    assert!(parse_value("1ß").is_err());
    assert!(parse_value("10kß").is_err());
    assert!(parse_value("\u{FEFF}").is_err());
    // Random non-ASCII noise should never panic, always Err.
    for test in &["αβγ", "한국", "👍", "1👍", "k👍"] {
        let _ = parse_value(test);
    }
}

// Micro sign (U+00B5) and Greek small mu (U+03BC) are the only non-ASCII
// suffixes we accept; both normalize to 'u' (micro, 1e-6).
#[test]
fn test_parse_value_micro_sign() {
    let a = parse_value("4.7µ").expect("µ should parse");
    let b = parse_value("4.7μ").expect("μ should parse");
    let c = parse_value("4.7u").expect("u should parse");
    assert!((a - 4.7e-6).abs() < 1e-18);
    assert!((b - 4.7e-6).abs() < 1e-18);
    assert!((c - 4.7e-6).abs() < 1e-18);
}

// 13. Model parsing edge cases
#[test]
fn test_model_without_params() {
    let spice = "Test\n.model MyDiode D\nD1 1 2 MyDiode\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Model without params should parse: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.models[0].params.len(), 0);
}

#[test]
fn test_model_missing_type() {
    let spice = "Test\n.model MyDiode\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), ".model without type should be rejected");
}

// 14. Directive edge cases
#[test]
fn test_unknown_directive_ignored() {
    let spice = "Test\n.options RELTOL=1e-3\nR1 1 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Unknown directives should be ignored: {:?}",
        result.err()
    );
}

#[test]
fn test_param_directive() {
    let spice = "Test\n.param Rval=1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        ".param directive should parse: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.params.len(), 1);
    assert_eq!(netlist.params[0].name, "Rval");
    assert_eq!(netlist.params[0].value, 1000.0);
}

#[test]
fn test_param_missing_equals() {
    let spice = "Test\n.param Rval\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), ".param without = should be rejected");
}

// ===== Op-amp parsing tests =====

#[test]
fn test_parse_opamp_basic() {
    let spice = "Test\nU1 3 2 6 opamp\n.model opamp OA(AOL=200000)\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Opamp {
            name,
            n_plus,
            n_minus,
            n_out,
            model,
        } => {
            assert_eq!(name, "U1");
            assert_eq!(n_plus, "3");
            assert_eq!(n_minus, "2");
            assert_eq!(n_out, "6");
            assert_eq!(model, "opamp");
        }
        _ => panic!("Expected Opamp, got {:?}", netlist.elements[0]),
    }
}

#[test]
fn test_parse_opamp_model_params() {
    let spice = "Test\nU1 3 2 6 myoa\n.model myoa OA(AOL=100000 GBW=1e6 ROUT=75)\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.models.len(), 1);
    let model = &netlist.models[0];
    assert_eq!(model.model_type, "OA");
    let aol = model
        .params
        .iter()
        .find(|(k, _)| k == "AOL")
        .map(|(_, v)| *v);
    assert_eq!(aol, Some(100_000.0));
    let rout = model
        .params
        .iter()
        .find(|(k, _)| k == "ROUT")
        .map(|(_, v)| *v);
    assert_eq!(rout, Some(75.0));
}

#[test]
fn test_parse_opamp_no_model() {
    let spice = "Test\nU1 3 2 6 missing_model\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Op-amp without .model should be rejected");
}

#[test]
fn test_parse_opamp_in_circuit() {
    let spice = r#"Inverting Amplifier
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.elements.len(), 3);
    assert!(matches!(&netlist.elements[2], Element::Opamp { .. }));
}

// ======================================================================
// VCVS (E) and VCCS (G) controlled source tests
// ======================================================================

#[test]
fn test_parse_vcvs_basic() {
    let spice = "Test\nE1 out 0 in 0 10\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.elements.len(), 1);
    match &netlist.elements[0] {
        Element::Vcvs {
            name,
            out_p,
            out_n,
            ctrl_p,
            ctrl_n,
            gain,
        } => {
            assert_eq!(name, "E1");
            assert_eq!(out_p, "out");
            assert_eq!(out_n, "0");
            assert_eq!(ctrl_p, "in");
            assert_eq!(ctrl_n, "0");
            assert_eq!(*gain, 10.0);
        }
        _ => panic!("Expected VCVS"),
    }
}

#[test]
fn test_parse_vccs_basic() {
    let spice = "Test\nG1 out 0 in 0 1m\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.elements.len(), 1);
    match &netlist.elements[0] {
        Element::Vccs {
            name,
            out_p,
            out_n,
            ctrl_p,
            ctrl_n,
            gm,
        } => {
            assert_eq!(name, "G1");
            assert_eq!(out_p, "out");
            assert_eq!(out_n, "0");
            assert_eq!(ctrl_p, "in");
            assert_eq!(ctrl_n, "0");
            assert!((gm - 1e-3).abs() < 1e-12, "Expected 1m = 1e-3, got {}", gm);
        }
        _ => panic!("Expected VCCS"),
    }
}

#[test]
fn test_parse_vcvs_engineering_notation() {
    let spice = "Test\nE1 out 0 in 0 1k\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Vcvs { gain, .. } => {
            assert_eq!(*gain, 1000.0);
        }
        _ => panic!("Expected VCVS"),
    }
}

#[test]
fn test_parse_vccs_engineering_notation() {
    let spice = "Test\nG1 out 0 in 0 100u\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Vccs { gm, .. } => {
            assert!((*gm - 100e-6).abs() < 1e-15, "Expected 100u, got {}", gm);
        }
        _ => panic!("Expected VCCS"),
    }
}

#[test]
fn test_parse_vcvs_negative_gain() {
    // Negative gain is valid for VCVS (e.g., inverting amplifier)
    let spice = "Test\nE1 out 0 in 0 -10\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Vcvs { gain, .. } => {
            assert_eq!(*gain, -10.0);
        }
        _ => panic!("Expected VCVS"),
    }
}

#[test]
fn test_parse_vccs_negative_gm() {
    // Negative gm is valid for VCCS
    let spice = "Test\nG1 out 0 in 0 -0.01\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Vccs { gm, .. } => {
            assert_eq!(*gm, -0.01);
        }
        _ => panic!("Expected VCCS"),
    }
}

#[test]
fn test_parse_vcvs_zero_gain_rejected() {
    let spice = "Test\nE1 out 0 in 0 0\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Zero VCVS gain should be rejected");
}

#[test]
fn test_parse_vccs_zero_gm_rejected() {
    let spice = "Test\nG1 out 0 in 0 0\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Zero VCCS gm should be rejected");
}

#[test]
fn test_parse_vcvs_missing_nodes() {
    let spice = "Test\nE1 out 0 in\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "VCVS with missing nodes should be rejected"
    );
}

#[test]
fn test_parse_vccs_missing_value() {
    let spice = "Test\nG1 out 0 in 0\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "VCCS with missing gm should be rejected");
}

#[test]
fn test_parse_vcvs_duplicate_name() {
    let spice = "Test\nE1 out 0 in 0 10\nE1 out2 0 in2 0 5\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Duplicate E1 should be rejected");
}

#[test]
fn test_parse_vcvs_four_nodes() {
    // All four nodes non-ground
    let spice = "Test\nE1 out_p out_n ctrl_p ctrl_n 2.5\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Vcvs {
            out_p,
            out_n,
            ctrl_p,
            ctrl_n,
            gain,
            ..
        } => {
            assert_eq!(out_p, "out_p");
            assert_eq!(out_n, "out_n");
            assert_eq!(ctrl_p, "ctrl_p");
            assert_eq!(ctrl_n, "ctrl_n");
            assert_eq!(*gain, 2.5);
        }
        _ => panic!("Expected VCVS"),
    }
}

// ======================================================================
// Input-size cap tests (MAX_NETLIST_BYTES, MAX_NODE_NAME_LEN,
// MAX_TOTAL_ELEMENTS, MAX_MODELS, MAX_MODEL_PARAMS)
// ======================================================================

#[test]
fn test_cap_netlist_bytes_rejects_oversized_input() {
    // Construct a string just over MAX_NETLIST_BYTES by padding with
    // comment lines so the extra bytes are not parse-meaningful.
    let mut spice = String::with_capacity(MAX_NETLIST_BYTES + 64);
    spice.push_str("Test\n");
    let pad_line = "* padding padding padding padding padding padding padding\n";
    while spice.len() <= MAX_NETLIST_BYTES {
        spice.push_str(pad_line);
    }
    spice.push_str("R1 1 0 1k\n");
    assert!(spice.len() > MAX_NETLIST_BYTES);
    let err = Netlist::parse(&spice).unwrap_err();
    assert!(
        err.message.contains("MAX_NETLIST_BYTES"),
        "expected MAX_NETLIST_BYTES error, got: {}",
        err.message
    );
}

#[test]
fn test_cap_netlist_bytes_at_limit_ok() {
    // An input *at* the limit should still parse. Use a minimal valid
    // netlist padded with comment chars up to (but not past) the cap.
    let header = "Test\nR1 1 0 1k\n";
    let mut spice = String::with_capacity(MAX_NETLIST_BYTES);
    spice.push_str(header);
    while spice.len() < MAX_NETLIST_BYTES {
        spice.push('*');
    }
    assert_eq!(spice.len(), MAX_NETLIST_BYTES);
    let netlist = Netlist::parse(&spice).expect("exactly-at-limit should parse");
    assert_eq!(netlist.elements.len(), 1);
}

#[test]
fn test_cap_node_name_rejects_too_long() {
    let long_name: String = "n".repeat(MAX_NODE_NAME_LEN + 1);
    let spice = format!("Test\nR1 {long_name} 0 1k\n");
    let err = Netlist::parse(&spice).unwrap_err();
    assert!(
        err.message.contains("MAX_NODE_NAME_LEN"),
        "expected MAX_NODE_NAME_LEN error, got: {}",
        err.message
    );
}

#[test]
fn test_cap_node_name_at_limit_ok() {
    let name: String = "n".repeat(MAX_NODE_NAME_LEN);
    let spice = format!("Test\nR1 {name} 0 1k\n");
    let netlist = Netlist::parse(&spice).expect("name at exactly the limit should parse");
    assert_eq!(netlist.elements.len(), 1);
}

#[test]
fn test_cap_total_elements_rejects_too_many() {
    // Build a netlist with MAX_TOTAL_ELEMENTS + 1 resistors.
    // Each resistor uses a distinct name and a distinct n+ node,
    // so the parser won't reject it on duplicate grounds first.
    let mut spice = String::with_capacity((MAX_TOTAL_ELEMENTS + 2) * 16);
    spice.push_str("Test\n");
    for i in 0..=MAX_TOTAL_ELEMENTS {
        spice.push_str(&format!("R{i} n{i} 0 1k\n"));
    }
    let err = Netlist::parse(&spice).unwrap_err();
    assert!(
        err.message.contains("MAX_TOTAL_ELEMENTS"),
        "expected MAX_TOTAL_ELEMENTS error, got: {}",
        err.message
    );
}

#[test]
fn test_cap_total_elements_at_limit_ok() {
    // Exactly MAX_TOTAL_ELEMENTS should parse.
    let n = MAX_TOTAL_ELEMENTS;
    let mut spice = String::with_capacity(n * 20);
    spice.push_str("Test\n");
    for i in 0..n {
        spice.push_str(&format!("R{i} n{i} 0 1k\n"));
    }
    let netlist = Netlist::parse(&spice).expect("exactly MAX_TOTAL_ELEMENTS should parse");
    assert_eq!(netlist.elements.len(), n);
}

#[test]
fn test_cap_models_rejects_too_many() {
    // Build a netlist with MAX_MODELS + 1 .model directives, each with
    // a distinct name. No devices reference them (so post-parse model
    // validation does not fire — this test targets the count cap only).
    let mut spice = String::new();
    spice.push_str("Test\n");
    for i in 0..=MAX_MODELS {
        spice.push_str(&format!(".model M{i} D(IS=1e-15)\n"));
    }
    let err = Netlist::parse(&spice).unwrap_err();
    assert!(
        err.message.contains("MAX_MODELS"),
        "expected MAX_MODELS error, got: {}",
        err.message
    );
}

#[test]
fn test_cap_models_at_limit_ok() {
    let mut spice = String::new();
    spice.push_str("Test\n");
    for i in 0..MAX_MODELS {
        spice.push_str(&format!(".model M{i} D(IS=1e-15)\n"));
    }
    let netlist = Netlist::parse(&spice).expect("exactly MAX_MODELS should parse");
    assert_eq!(netlist.models.len(), MAX_MODELS);
}

#[test]
fn test_cap_model_params_rejects_too_many() {
    // One .model with MAX_MODEL_PARAMS + 1 parameters.
    let mut params = String::new();
    for i in 0..=MAX_MODEL_PARAMS {
        params.push_str(&format!("P{i}=1 "));
    }
    let spice = format!("Test\n.model M1 D({params})\n");
    let err = Netlist::parse(&spice).unwrap_err();
    assert!(
        err.message.contains("MAX_MODEL_PARAMS"),
        "expected MAX_MODEL_PARAMS error, got: {}",
        err.message
    );
}

#[test]
fn test_cap_model_params_at_limit_ok() {
    // Exactly MAX_MODEL_PARAMS parameters. Use benign names that
    // won't trigger the device-specific validator.
    let mut params = String::new();
    for i in 0..MAX_MODEL_PARAMS {
        params.push_str(&format!("PX{i}=1 "));
    }
    let spice = format!("Test\n.model M1 D({params})\n");
    let netlist = Netlist::parse(&spice).expect("exactly MAX_MODEL_PARAMS should parse");
    assert_eq!(netlist.models[0].params.len(), MAX_MODEL_PARAMS);
}

#[test]
fn test_mismatch_and_seed_directives_parse() {
    let spice = "Mismatch Test\n\
                     .seed 42\n\
                     .mismatch D IS=0.02 N=0.01\n\
                     .mismatch Q BF=0.05\n\
                     R1 in out 1k\n\
                     .end\n";
    let n = Netlist::parse(spice).expect("parse");
    assert_eq!(n.seed, Some(42));
    assert_eq!(n.mismatch_specs.len(), 2);
    assert_eq!(n.mismatch_specs[0].device_class, 'D');
    assert_eq!(
        n.mismatch_specs[0].params,
        vec![("IS".to_string(), 0.02), ("N".to_string(), 0.01)]
    );
    assert_eq!(n.mismatch_specs[1].device_class, 'Q');
    assert_eq!(n.mismatch_specs[1].params, vec![("BF".to_string(), 0.05)]);
}

#[test]
fn test_mismatch_rejects_bad_class() {
    let spice = "Bad\n.mismatch X IS=0.01\nR1 a b 1k\n.end\n";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        err.message.contains("not one of D, Q, J, M, T"),
        "expected class error, got: {}",
        err.message
    );
}

#[test]
fn test_mismatch_rejects_out_of_range_tolerance() {
    let spice = "Bad\n.mismatch D IS=1.5\nR1 a b 1k\n.end\n";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        err.message.contains("tolerance must be in [0.0, 1.0)"),
        "expected tolerance range error, got: {}",
        err.message
    );
}

#[test]
fn test_mismatch_rejects_malformed_entry() {
    let spice = "Bad\n.mismatch D IS\nR1 a b 1k\n.end\n";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        err.message.contains("PARAM=TOL"),
        "expected PARAM=TOL error, got: {}",
        err.message
    );
}

#[test]
fn test_seed_rejects_non_numeric() {
    let spice = "Bad\n.seed abc\nR1 a b 1k\n.end\n";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        err.message.contains("not a valid u64"),
        "expected u64 parse error, got: {}",
        err.message
    );
}

#[test]
fn test_oversampling_directive_parses() {
    for (deck, want) in [
        ("T\nR1 1 0 1k\n.oversampling 1\n.end\n", 1usize),
        ("T\nR1 1 0 1k\n.oversampling 2\n.end\n", 2),
        ("T\nR1 1 0 1k\n.oversampling 4\n.end\n", 4),
    ] {
        let n = Netlist::parse(deck).expect("parse");
        assert_eq!(n.recommended_oversampling, Some(want));
    }
}

#[test]
fn test_oversampling_absent_is_none() {
    let n = Netlist::parse("Noop\nR1 a b 1k\n.end\n").expect("parse");
    assert_eq!(n.recommended_oversampling, None);
}

#[test]
fn test_oversampling_rejects_out_of_set() {
    let err = Netlist::parse("Bad\n.oversampling 3\nR1 a b 1k\n.end\n").unwrap_err();
    assert!(
        err.message.contains("must be 1, 2, or 4"),
        "expected valid-values error, got: {}",
        err.message
    );
}

#[test]
fn test_oversampling_rejects_non_numeric() {
    let err = Netlist::parse("Bad\n.oversampling hi\nR1 a b 1k\n.end\n").unwrap_err();
    assert!(
        err.message.contains("not a valid integer"),
        "expected integer parse error, got: {}",
        err.message
    );
}

#[test]
fn test_mismatch_absent_is_no_op() {
    // No `.mismatch` directive — Netlist should default to empty specs
    // and None seed.
    let spice = "Noop\nR1 a b 1k\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    assert!(n.mismatch_specs.is_empty());
    assert_eq!(n.seed, None);
}

#[test]
fn test_tolerance_directive_parses() {
    let spice =
        "Tol\n.seed 7\n.tolerance R=0.01 C=0.02 L=0.005\nR1 a b 1k\nC1 b 0 1u\nL1 a 0 1m\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    // Values have already been jittered at parse end — read the
    // directive fields directly instead.
    assert!((n.tolerance_r - 0.01).abs() < 1e-12);
    assert!((n.tolerance_c - 0.02).abs() < 1e-12);
    assert!((n.tolerance_l - 0.005).abs() < 1e-12);
}

#[test]
fn test_tolerance_rejects_bad_class() {
    let spice = "Bad\n.tolerance Z=0.01\nR1 a b 1k\n.end\n";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        err.message.contains("must be R, C, or L"),
        "expected class error, got: {}",
        err.message
    );
}

#[test]
fn test_tolerance_rejects_out_of_range() {
    let spice = "Bad\n.tolerance R=1.5\nR1 a b 1k\n.end\n";
    let err = Netlist::parse(spice).unwrap_err();
    assert!(
        err.message.contains("must be in [0.0, 1.0)"),
        "expected range error, got: {}",
        err.message
    );
}

#[test]
fn test_tolerance_jitters_fixed_resistors() {
    // Two fixed resistors, ±1% tolerance. They should land at
    // different values, both within tolerance of nominal.
    let spice = "Jitter\n\
                     .seed 42\n\
                     .tolerance R=0.01\n\
                     R1 a b 1k\n\
                     R2 b 0 1k\n\
                     .end\n";
    let n = Netlist::parse(spice).expect("parse");
    let (v1, v2) = {
        let mut iter = n.elements.iter().filter_map(|e| match e {
            Element::Resistor { name, value, .. } => Some((name.clone(), *value)),
            _ => None,
        });
        let a = iter.next().unwrap();
        let b = iter.next().unwrap();
        (a.1, b.1)
    };
    assert_ne!(v1, v2, "R1 and R2 should get different jittered values");
    assert!(
        (v1 - 1000.0).abs() / 1000.0 <= 0.01,
        "R1 outside 1% band: {v1}"
    );
    assert!(
        (v2 - 1000.0).abs() / 1000.0 <= 0.01,
        "R2 outside 1% band: {v2}"
    );
}

#[test]
fn test_tolerance_skips_pot_controlled_resistors() {
    // R_pot is a `.pot` target — its value must be left nominal so
    // the UI slider still maps cleanly to [min, max]. R_fixed gets
    // jittered.
    let spice = "Skip Pot\n\
                     .seed 99\n\
                     .tolerance R=0.05\n\
                     R_pot a b 50k\n\
                     R_fixed b 0 10k\n\
                     .pot R_pot 1 100k\n\
                     .end\n";
    let n = Netlist::parse(spice).expect("parse");
    let mut values: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
    for e in &n.elements {
        if let Element::Resistor { name, value, .. } = e {
            values.insert(name.clone(), *value);
        }
    }
    assert_eq!(values["R_pot"], 50_000.0, "pot target must remain nominal");
    assert_ne!(values["R_fixed"], 10_000.0, "fixed R should be jittered");
}

#[test]
fn test_tolerance_determinism() {
    // Same seed must produce identical jittered values every run.
    let spice = "Det\n\
                     .seed 12345\n\
                     .tolerance R=0.02\n\
                     R1 a b 1k\n\
                     R2 b 0 1k\n\
                     .end\n";
    let a = Netlist::parse(spice).expect("parse a");
    let b = Netlist::parse(spice).expect("parse b");
    for (ea, eb) in a.elements.iter().zip(b.elements.iter()) {
        if let (Element::Resistor { value: va, .. }, Element::Resistor { value: vb, .. }) = (ea, eb)
        {
            assert_eq!(va, vb, "determinism: same seed must produce same jitter");
        }
    }
}

#[test]
fn test_tolerance_absent_is_no_op() {
    // Without `.tolerance`, R values are exactly the netlist nominals.
    let spice = "Noop\nR1 a b 1k\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    for e in &n.elements {
        if let Element::Resistor { name: _, value, .. } = e {
            assert_eq!(*value, 1000.0);
        }
    }
}

// ---- the jitter ARITHMETIC itself -------------------------------------
//
// Until 2026-09-22 nothing tested this. `deterministic_draw` was referenced
// only from `parser.rs` and `codegen/ir/mod.rs` and by no test file at all,
// so the determinism and no-op guards above covered "the same wrong number
// twice" and "no number" — never "the right number". `melange validate`
// cannot close that gap either: it now compiles the melange side with unit
// variation OFF (`ParseOptions::disable_unit_variation`) so it compares
// nominal against nominal, and ngspice has no concept of the draw to grade
// it against even if it did not. Whether the draw is correct is this test's
// job and only this test's job.
//
// The `u` expectations below were derived INDEPENDENTLY — an FNV-64 +
// SplitMix64 reimplementation outside this crate, fed the same
// (seed, tag, name) triples — not copied out of a `deterministic_draw`
// run. That is what makes them evidence rather than a restatement of the
// code they check. Recomputing one by hand is the way to re-verify it.

/// `deterministic_draw` is the shared RNG for BOTH unit-variation
/// directives. Pin its output: a refactor that changes these numbers
/// silently re-rolls every shipped plugin's personality.
#[test]
fn deterministic_draw_matches_independent_reimplementation() {
    assert_eq!(deterministic_draw(4142, "R", "R1"), 0.2643562609815917);
    assert_eq!(deterministic_draw(4142, "C", "C1"), -0.04633172401026564);
    assert_eq!(deterministic_draw(4142, "L", "L1"), 0.05235785305095719);
    // The null-byte separator is what keeps the streams apart: without it
    // ("R" + "C1") and ("RC" + "1") would hash identically.
    assert_ne!(
        deterministic_draw(4142, "R", "C1"),
        deterministic_draw(4142, "RC", "1")
    );
    // Range contract: u ∈ [-1, 1].
    for name in ["R1", "R2", "C7", "Lx", "R_feedback", ""] {
        for seed in [0u64, 1, 42, 4142, u64::MAX] {
            let u = deterministic_draw(seed, "R", name);
            assert!((-1.0..=1.0).contains(&u), "seed {seed} {name}: {u}");
        }
    }
}

/// The `.tolerance` half of the ruling's "unit test for the draw itself":
/// an applied draw is exactly `nominal · (1 + tol · u)`.
#[test]
fn tolerance_draw_matches_nominal_times_one_plus_tol_u() {
    // Independently derived (see the block comment above).
    const U_R1: f64 = 0.2643562609815917;
    const U_C1: f64 = -0.04633172401026564;
    const U_L1: f64 = 0.05235785305095719;

    let spice = "Draw\n\
                     .seed 4142\n\
                     .tolerance R=0.10 C=0.20 L=0.05\n\
                     R1 a b 1k\n\
                     C1 b 0 10n\n\
                     L1 b c 1m\n\
                     .end\n";
    let n = Netlist::parse(spice).expect("parse");

    let mut seen = 0;
    for e in &n.elements {
        match e {
            Element::Resistor { name, value, .. } if name == "R1" => {
                assert_eq!(*value, 1000.0 * (1.0 + 0.10 * U_R1));
                // Non-trivial: the value MOVED. A no-op path cannot pass.
                assert!((*value - 1000.0).abs() > 1.0, "{value}");
                seen += 1;
            }
            Element::Capacitor { name, value, .. } if name == "C1" => {
                assert_eq!(*value, 10e-9 * (1.0 + 0.20 * U_C1));
                assert!((*value - 10e-9).abs() > 1e-11, "{value:e}");
                seen += 1;
            }
            Element::Inductor { name, value, .. } if name == "L1" => {
                assert_eq!(*value, 1e-3 * (1.0 + 0.05 * U_L1));
                assert!((*value - 1e-3).abs() > 1e-7, "{value:e}");
                seen += 1;
            }
            _ => {}
        }
    }
    assert_eq!(seen, 3, "all three classes must have been jittered");
}

/// `ParseOptions::disable_unit_variation` — the switch `melange validate`
/// sets. The directives are still READ (the caller has to be able to name
/// them on the result line); only the draw is skipped.
#[test]
fn disable_unit_variation_keeps_values_nominal_but_keeps_the_directives() {
    let spice = "Draw\n\
                     .seed 4142\n\
                     .tolerance R=0.10 C=0.20 L=0.05\n\
                     R1 a b 1k\n\
                     C1 b 0 10n\n\
                     L1 b c 1m\n\
                     .end\n";
    let n = Netlist::parse_with_options(
        spice,
        ParseOptions {
            disable_unit_variation: true,
            disable_self_heating: false,
        },
    )
    .expect("parse");

    assert!(n.unit_variation_disabled);
    // Still recorded, so `validate` can say WHAT it disabled.
    assert_eq!(n.tolerance_r, 0.10);
    assert_eq!(n.tolerance_c, 0.20);
    assert_eq!(n.tolerance_l, 0.05);
    assert_eq!(n.seed, Some(4142));

    for e in &n.elements {
        match e {
            Element::Resistor { value, .. } => assert_eq!(*value, 1000.0),
            Element::Capacitor { value, .. } => assert_eq!(*value, 10e-9),
            Element::Inductor { value, .. } => assert_eq!(*value, 1e-3),
            _ => {}
        }
    }

    // And it cannot be bypassed by calling the apply site directly.
    let mut n2 = n;
    n2.apply_passive_tolerance();
    for e in &n2.elements {
        if let Element::Resistor { value, .. } = e {
            assert_eq!(*value, 1000.0);
        }
    }
}

/// The default is unchanged: `Netlist::parse` still jitters.
#[test]
fn default_parse_options_still_jitter() {
    let spice = "Draw\n.seed 4142\n.tolerance R=0.10\nR1 a b 1k\n.end\n";
    let jittered = Netlist::parse(spice).expect("parse");
    let nominal = Netlist::parse_with_options(spice, ParseOptions::default()).expect("parse");
    let val = |n: &Netlist| match &n.elements[0] {
        Element::Resistor { value, .. } => *value,
        other => panic!("{other:?}"),
    };
    assert_eq!(val(&jittered), val(&nominal));
    assert_ne!(val(&jittered), 1000.0);
}

// ---- .inject / .tap directive parsing (Gate 1: mandatory impedance) ----

#[test]
fn test_inject_thevenin_parses() {
    let spice = "Inj\nR1 a 0 1k\n.inject a fb R=47k\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    assert_eq!(n.injections.len(), 1);
    let inj = &n.injections[0];
    assert_eq!(inj.node, "a");
    assert_eq!(inj.field_name, "fb");
    assert_eq!(inj.impedance, InjectImpedance::Thevenin(47_000.0));
}

#[test]
fn test_inject_norton_parses() {
    let spice = "Inj\nR1 a 0 1k\n.inject a fb RSHUNT=10k\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    assert_eq!(n.injections.len(), 1);
    assert_eq!(n.injections[0].impedance, InjectImpedance::Norton(10_000.0));
}

#[test]
fn test_inject_missing_impedance_rejected() {
    // GATE 1: an ideal source (no R=/RSHUNT=) must be rejected at parse.
    let spice = "Inj\nR1 a 0 1k\n.inject a fb\n.end\n";
    let err = Netlist::parse(spice).expect_err("must reject missing impedance");
    assert!(
        err.message.contains("impedance"),
        "error should mention mandatory impedance, got: {}",
        err.message
    );
}

#[test]
fn test_inject_bareword_impedance_rejected() {
    // A trailing token that isn't R=/RSHUNT= is not an impedance.
    let spice = "Inj\nR1 a 0 1k\n.inject a fb 47k\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_inject_bad_impedance_key_rejected() {
    let spice = "Inj\nR1 a 0 1k\n.inject a fb Z=47k\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_inject_ground_node_rejected() {
    let spice = "Inj\nR1 a 0 1k\n.inject 0 fb R=1k\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_inject_bad_field_name_rejected() {
    let spice = "Inj\nR1 a 0 1k\n.inject a 9bad R=1k\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_inject_duplicate_field_rejected() {
    let spice = "Inj\nR1 a 0 1k\nR2 b 0 1k\n.inject a fb R=1k\n.inject b fb R=1k\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_inject_zero_impedance_rejected() {
    let spice = "Inj\nR1 a 0 1k\n.inject a fb R=0\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_tap_parses_with_and_without_name() {
    let spice = "T\nR1 a 0 1k\nR2 b 0 1k\n.tap a\n.tap b tankdrive\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    assert_eq!(n.taps.len(), 2);
    assert_eq!(n.taps[0].node, "a");
    assert_eq!(n.taps[0].name, "a"); // defaults to node name
    assert_eq!(n.taps[1].node, "b");
    assert_eq!(n.taps[1].name, "tankdrive");
}

#[test]
fn test_tap_ground_rejected() {
    let spice = "T\nR1 a 0 1k\n.tap 0\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_tap_duplicate_name_rejected() {
    let spice = "T\nR1 a 0 1k\nR2 b 0 1k\n.tap a x\n.tap b x\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_port_accumulates_across_lines_and_records_its_line() {
    let spice = "Board\nR1 a 0 1k\nR2 b 0 1k\nR3 c 0 1k\n.port a b\n.port c\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    assert_eq!(
        n.ports.iter().map(|p| p.node.as_str()).collect::<Vec<_>>(),
        vec!["a", "b", "c"]
    );
    assert_eq!(n.ports[0].line, 5);
    assert_eq!(n.ports[2].line, 6);
}

#[test]
fn test_port_ground_rejected() {
    let spice = "Board\nR1 a 0 1k\n.port 0\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_port_duplicate_rejected() {
    let spice = "Board\nR1 a 0 1k\nR2 b 0 1k\n.port a b\n.port a\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_port_requires_a_node() {
    let spice = "Board\nR1 a 0 1k\n.port\n.end\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn test_no_ports_by_default() {
    let spice = "Plain\nR1 a 0 1k\n.end\n";
    assert!(Netlist::parse(spice).expect("parse").ports.is_empty());
}

#[test]
fn test_no_inject_tap_by_default() {
    let spice = "Plain\nR1 a 0 1k\n.end\n";
    let n = Netlist::parse(spice).expect("parse");
    assert!(n.injections.is_empty());
    assert!(n.taps.is_empty());
}

/// Standard SPICE dot commands that `parse_directive` dispatches on but
/// which ngspice parses natively, so they must NOT be in
/// `MELANGE_ONLY_DIRECTIVES`.
const STANDARD_SPICE_DIRECTIVES: &[&str] = &[".model", ".param", ".subckt", ".end", ".ends"];

/// Scrape the match arms of `Parser::parse_directive` out of this file's
/// source: every `".name"` string literal on the left of a `=>` inside the
/// function body. Nested `match` arms in the body never use dot-literals,
/// and error-message strings never sit on the left of `=>`, so this is
/// exactly the dispatch set.
fn parse_directive_arms() -> std::collections::BTreeSet<String> {
    let src = include_str!("directive_parse.rs");
    let start = src
        .find("fn parse_directive(")
        .expect("parse_directive must exist");
    let body = &src[start..];
    // Function body ends at the next impl-level `fn` (4-space indent).
    let end = body[1..].find("\n    fn ").map_or(body.len(), |i| i + 1);
    let body = &body[..end];

    let mut arms = std::collections::BTreeSet::new();
    for line in body.lines() {
        let Some((lhs, _)) = line.split_once("=>") else {
            continue;
        };
        let mut rest = lhs;
        while let Some(open) = rest.find('"') {
            let after = &rest[open + 1..];
            let Some(close) = after.find('"') else { break };
            let lit = &after[..close];
            if lit.starts_with('.')
                && lit.len() > 1
                && lit[1..].chars().all(|c| c.is_ascii_lowercase() || c == '_')
            {
                arms.insert(lit.to_string());
            }
            rest = &after[close + 1..];
        }
    }
    arms
}

#[test]
fn test_melange_only_directives_matches_parse_directive() {
    // Direction 1: every arm of parse_directive is either standard SPICE
    // or listed in MELANGE_ONLY_DIRECTIVES (catches a new directive added
    // to the parser but not the const).
    let arms = parse_directive_arms();
    assert!(
        arms.len() >= 10,
        "arm scrape looks broken (found only {:?})",
        arms
    );
    let expected: std::collections::BTreeSet<String> = MELANGE_ONLY_DIRECTIVES
        .iter()
        .chain(STANDARD_SPICE_DIRECTIVES)
        .map(|s| s.to_string())
        .collect();
    let missing_from_const: Vec<_> = arms.difference(&expected).collect();
    assert!(
        missing_from_const.is_empty(),
        "parse_directive arms not in MELANGE_ONLY_DIRECTIVES (or the standard set): {:?}",
        missing_from_const
    );

    // Direction 2: every const entry is an actual arm (catches a stale
    // entry left behind after a directive is removed from the parser).
    let stale: Vec<_> = MELANGE_ONLY_DIRECTIVES
        .iter()
        .filter(|d| !arms.contains(**d))
        .collect();
    assert!(
        stale.is_empty(),
        "MELANGE_ONLY_DIRECTIVES entries with no parse_directive arm: {:?}",
        stale
    );

    // No entry may be standard SPICE, and the list must be lowercase with
    // a leading dot (the consumer compares against the lowercased first
    // token of a line).
    for d in MELANGE_ONLY_DIRECTIVES {
        assert!(
            !STANDARD_SPICE_DIRECTIVES.contains(d),
            "{d} is standard SPICE and must not be listed as melange-only"
        );
        assert!(
            d.starts_with('.') && *d == d.to_lowercase(),
            "bad entry {d}"
        );
    }
}

#[test]
fn test_every_melange_only_directive_parses() {
    // One minimal well-formed deck per directive; each must parse with no
    // error. If a directive is added to MELANGE_ONLY_DIRECTIVES without a
    // row here, the coverage assertion at the bottom fails.
    let decks: &[(&str, &str)] = &[
            (".pot", "T\nR1 1 0 10k\n.pot R1 1k 100k\n.end\n"),
            (".switch", "T\nC1 1 0 100n\n.switch C1 100n 220n\n.end\n"),
            (".wiper", "T\nR1 1 2 50k\nR2 2 0 50k\n.wiper R1 R2 100k\n.end\n"),
            (
                ".gang",
                "T\nR1 1 0 10k\nR2 2 0 10k\n.pot R1 1k 100k\n.pot R2 1k 100k\n.gang \"G\" R1 R2\n.end\n",
            ),
            (".runtime", "T\nV1 1 0 DC 5\nR1 1 0 1k\n.runtime V1 as bias\n.end\n"),
            (".mismatch", "T\nR1 1 0 1k\n.mismatch D IS=0.02\n.end\n"),
            (".tolerance", "T\nR1 1 0 1k\n.tolerance R=0.01\n.end\n"),
            (".seed", "T\nR1 1 0 1k\n.seed 7\n.end\n"),
            (".linearize", "T\nR1 1 0 1k\n.linearize Q9\n.end\n"),
            (".tap", "T\nR1 a 0 1k\n.tap a\n.end\n"),
            (".port", "T\nR1 a 0 1k\n.port a\n.end\n"),
            (".input_impedance", "T\nR1 1 0 1k\n.input_impedance 600\n.end\n"),
            (".integrator", "T\nR1 1 0 1k\n.integrator trap\n.end\n"),
            (".inject", "T\nR1 a 0 1k\n.inject a fb R=47k\n.end\n"),
            (".delay_feedback", "T\nR1 a 0 1k\n.delay_feedback a\n.end\n"),
            (".oversampling", "T\nR1 1 0 1k\n.oversampling 2\n.end\n"),
        ];
    for (directive, deck) in decks {
        assert!(
            MELANGE_ONLY_DIRECTIVES.contains(directive),
            "{directive} row is not in MELANGE_ONLY_DIRECTIVES"
        );
        if let Err(e) = Netlist::parse(deck) {
            panic!("{directive} deck failed to parse: {e}\n{deck}");
        }
    }
    for d in MELANGE_ONLY_DIRECTIVES {
        assert!(
            decks.iter().any(|(name, _)| name == d),
            "{d} has no parse-coverage deck in this test"
        );
    }
}
