//! Parser strictness regression tests.
//!
//! Every case here belongs to the "silently produces a wrong circuit that
//! solves perfectly" class — the project's worst failure mode. The contract
//! being locked in: the parser either parses CORRECTLY (matching ngspice
//! semantics) or fails LOUDLY. Never silently.

use melange_solver::parser::{parse_value, parse_value_model_param, Element, Netlist};

// ===========================================================================
// Fix 1: node-name case folding (ngspice folds case; `IN` == `in`)
// ===========================================================================

#[test]
fn node_names_fold_to_lowercase() {
    let spice = "Test\nR1 IN Mid 1k\nR2 mid OUT 1k\nC1 out 0 100n\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Resistor {
            n_plus, n_minus, ..
        } => {
            assert_eq!(n_plus, "in");
            assert_eq!(n_minus, "mid");
        }
        _ => panic!("expected resistor"),
    }
    match &netlist.elements[1] {
        Element::Resistor {
            n_plus, n_minus, ..
        } => {
            assert_eq!(n_plus, "mid", "MID and mid must be the same net");
            assert_eq!(n_minus, "out");
        }
        _ => panic!("expected resistor"),
    }
}

#[test]
fn mixed_case_nets_are_one_net_in_mna() {
    // Before the fix `IN` and `in` became two disconnected nets and the
    // circuit "worked" (solved perfectly) with the input floating.
    let spice = "Test\nR1 IN out 1k\nR2 in 0 1k\nC1 out 0 100n\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).unwrap();
    assert!(
        mna.node_map.contains_key("in"),
        "normalized net 'in' must exist"
    );
    assert!(
        !mna.node_map.contains_key("IN"),
        "uppercase alias must NOT be a separate net"
    );
}

#[test]
fn bsource_expression_nodes_fold_to_lowercase() {
    let spice = "Test\nR1 in out 1k\nB1 bo 0 V={V(IN)+V(OUT)}\nR2 bo 0 1k\nC1 out 0 1n\n";
    let netlist = Netlist::parse(spice).unwrap();
    let bsrc = netlist
        .elements
        .iter()
        .find_map(|e| match e {
            Element::BSource { expr, .. } => Some(expr),
            _ => None,
        })
        .expect("B-source present");
    let nodes = bsrc.referenced_nodes();
    assert!(nodes.contains("in"), "V(IN) must normalize to net 'in'");
    assert!(nodes.contains("out"), "V(OUT) must normalize to net 'out'");
    assert!(!nodes.contains("IN") && !nodes.contains("OUT"));
}

// ===========================================================================
// Fix 2: gnd / ground alias to node 0
// ===========================================================================

#[test]
fn gnd_aliases_to_node_zero() {
    let spice = "Test\nR1 in gnd 1k\nR2 in GROUND 2k\nC1 in 0 1n\n";
    let netlist = Netlist::parse(spice).unwrap();
    for elem in &netlist.elements[..2] {
        match elem {
            Element::Resistor { name, n_minus, .. } => {
                assert_eq!(n_minus, "0", "{}: gnd/GROUND must alias to node 0", name);
            }
            _ => panic!("expected resistor"),
        }
    }
}

#[test]
fn gnd_divider_actually_grounds() {
    // Before the fix `gnd` was a floating one-terminal net: the divider
    // output tracked the input 1:1 instead of dividing.
    let spice = "Divider\nR1 in out 1k\nR2 out gnd 1k\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).unwrap();
    assert!(
        !mna.node_map.contains_key("gnd"),
        "'gnd' must not appear as a distinct net"
    );
}

#[test]
fn gnd_to_zero_self_connection_rejected() {
    // "gnd" and "0" are the same node after aliasing — this resistor is
    // self-connected and must be rejected (pre-normalization check passed it).
    let spice = "Test\nR1 gnd 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "R across gnd/0 is self-connected and must be rejected"
    );
}

// ===========================================================================
// Fix 3: femto suffix
// ===========================================================================

#[test]
fn femto_in_model_params() {
    // The probe that exposed the bug: IS=6.734f parsed as 6.734 (15 orders
    // of magnitude off) because trailing-unit-letter stripping ate the 'f'.
    let spice = "Test\nD1 a 0 DX\nR1 a 0 1k\n.model DX D(IS=6.734f)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let is = netlist.models[0]
        .params
        .iter()
        .find(|(k, _)| k == "IS")
        .map(|(_, v)| *v)
        .expect("IS param");
    assert!(
        (is - 6.734e-15).abs() < 1e-25,
        "IS=6.734f must be 6.734e-15, got {is:e}"
    );
}

#[test]
fn femto_with_unit_letter_in_element_position() {
    let spice = "Test\nC1 a 0 1fF\nR1 a 0 1k\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Capacitor { value, .. } => {
            assert!((*value - 1e-15).abs() < 1e-25, "1fF must be 1e-15 F");
        }
        _ => panic!("expected capacitor"),
    }
}

#[test]
fn bare_farad_stays_farad_in_element_position() {
    // Element-value contract: bare trailing F after a digit = Farad (×1),
    // with a log::warn. Femto there must be written 1e-15 or 1fF.
    let spice = "Test\nC1 a 0 1F\nR1 a 0 1k\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Capacitor { value, .. } => {
            assert!((*value - 1.0).abs() < 1e-10, "1F must stay 1.0 Farad");
        }
        _ => panic!("expected capacitor"),
    }
}

#[test]
fn parse_value_femto_contract() {
    assert!((parse_value("1fF").unwrap() - 1e-15).abs() < 1e-25);
    assert!((parse_value("1F").unwrap() - 1.0).abs() < 1e-10);
    assert!((parse_value_model_param("6.734f").unwrap() - 6.734e-15).abs() < 1e-25);
    assert!((parse_value_model_param("1F").unwrap() - 1e-15).abs() < 1e-25);
    // Unrelated suffixes are unchanged in both contexts
    assert!((parse_value("10pF").unwrap() - 10e-12).abs() < 1e-21);
    assert!((parse_value_model_param("10pF").unwrap() - 10e-12).abs() < 1e-21);
}

// ===========================================================================
// Fix 4: paren-less .model, unbalanced parens, junk in params region
// ===========================================================================

#[test]
fn parenless_model_params_are_parsed() {
    let spice = "Test\nD1 a 0 DX\nR1 a 0 1k\n.model DX D IS=1e-3 N=4.0\n";
    let netlist = Netlist::parse(spice).unwrap();
    let model = &netlist.models[0];
    assert_eq!(model.model_type, "D");
    assert_eq!(
        model.params.len(),
        2,
        "paren-less params must not be dropped"
    );
    assert!(model.params.iter().any(|(k, v)| k == "IS" && *v == 1e-3));
    assert!(model.params.iter().any(|(k, v)| k == "N" && *v == 4.0));
}

#[test]
fn model_unbalanced_parens_rejected() {
    let spice = "Test\nD1 a 0 DX\nR1 a 0 1k\n.model DX D(IS=1e-3\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "unbalanced '(' must be a hard error");

    let spice2 = "Test\nD1 a 0 DX\nR1 a 0 1k\n.model DX D IS=1e-3)\n";
    let result2 = Netlist::parse(spice2);
    assert!(result2.is_err(), "stray ')' must be a hard error");
}

#[test]
fn model_junk_token_in_params_rejected() {
    let spice = "Test\nD1 a 0 DX\nR1 a 0 1k\n.model DX D(IS=1e-3 garbage)\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "non-KEY=VAL token in a .model params region must be a hard error"
    );
    let msg = result.unwrap_err().message;
    assert!(msg.contains("garbage"), "error must name the token: {msg}");
}

// ===========================================================================
// Fix 5: unconsumed tokens on V/I source lines (SIN/PULSE/PWL)
// ===========================================================================

#[test]
fn vsource_with_sin_after_dc_rejected() {
    let spice = "Test\nV1 x 0 DC 2 SIN(0 1 1k)\nR1 x 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "DC + SIN must be a hard error, not a silent drop"
    );
    let msg = result.unwrap_err().message;
    assert!(
        msg.contains("SIN") || msg.contains("transient"),
        "error must name the transient spec: {msg}"
    );
    assert!(
        msg.contains("input node"),
        "error must point at the melange input mechanism: {msg}"
    );
}

#[test]
fn vsource_with_bare_sin_rejected_loudly() {
    let spice = "Test\nV1 x 0 SIN(0 1 1k)\nR1 x 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(
        msg.contains("transient"),
        "bare SIN must get the targeted transient-spec error: {msg}"
    );
}

#[test]
fn vsource_trailing_junk_rejected() {
    let spice = "Test\nV1 x 0 5 3\nR1 x 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "trailing junk on a V line must be a hard error"
    );
}

#[test]
fn isource_with_sin_rejected() {
    let spice = "Test\nI1 x 0 DC 2 SIN(0 1 1k)\nR1 x 0 1k\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(msg.contains("transient") || msg.contains("SIN"), "{msg}");

    let spice2 = "Test\nI1 x 0 2 junk\nR1 x 0 1k\n";
    assert!(
        Netlist::parse(spice2).is_err(),
        "trailing junk on an I line must be a hard error"
    );
}

#[test]
fn vsource_dc_ac_still_parse() {
    // Regression guard: the legitimate grammar keeps working.
    let spice = "Test\nV1 x 0 DC 9\nV2 y 0 AC 1 45\nV3 z 0 5\nR1 x y 1k\nR2 y z 1k\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::VoltageSource { dc, .. } => assert_eq!(*dc, Some(9.0)),
        _ => panic!(),
    }
    match &netlist.elements[1] {
        Element::VoltageSource { ac, .. } => assert_eq!(*ac, Some((1.0, 45.0))),
        _ => panic!(),
    }
    match &netlist.elements[2] {
        Element::VoltageSource { dc, .. } => assert_eq!(*dc, Some(5.0)),
        _ => panic!(),
    }
}

// ===========================================================================
// Fix 6: whitespace around '=' in .model params
// ===========================================================================

#[test]
fn model_params_with_spaced_equals() {
    let spice = "Test\nD1 a 0 DX\nR1 a 0 1k\n.model DX D(IS =1e-3 N= 4.0 BV = 100)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let model = &netlist.models[0];
    assert_eq!(
        model.params.len(),
        3,
        "spaced '=' forms must not drop params: {:?}",
        model.params
    );
    assert!(model.params.iter().any(|(k, v)| k == "IS" && *v == 1e-3));
    assert!(model.params.iter().any(|(k, v)| k == "N" && *v == 4.0));
    assert!(model.params.iter().any(|(k, v)| k == "BV" && *v == 100.0));
}

// ===========================================================================
// Fix 7: infix M is MEGA (BS-1852), suffix m stays milli
// ===========================================================================

#[test]
fn infix_m_is_mega_suffix_m_is_milli() {
    let spice = "Test\nR1 a b 4M7\nR2 b 0 10m\nC1 b 0 1n\n";
    let netlist = Netlist::parse(spice).unwrap();
    match &netlist.elements[0] {
        Element::Resistor { value, .. } => {
            assert!(
                (*value - 4.7e6).abs() < 1.0,
                "4M7 must be 4.7 MOhm (BS-1852), got {value}"
            );
        }
        _ => panic!(),
    }
    match &netlist.elements[1] {
        Element::Resistor { value, .. } => {
            assert!(
                (*value - 10e-3).abs() < 1e-10,
                "suffix 10m must stay milli, got {value}"
            );
        }
        _ => panic!(),
    }
}

#[test]
fn parse_value_infix_mega() {
    assert!((parse_value("4M7").unwrap() - 4.7e6).abs() < 1.0);
    assert!((parse_value("1M0").unwrap() - 1.0e6).abs() < 1.0);
    assert!((parse_value("10m").unwrap() - 10e-3).abs() < 1e-10);
    assert!(
        (parse_value("10M").unwrap() - 10e-3).abs() < 1e-10,
        "suffix M is milli (SPICE)"
    );
}

// ===========================================================================
// Fix 8: trailing tokens on element lines
// ===========================================================================

#[test]
fn mosfet_geometry_params_rejected_with_guidance() {
    let spice = "Test\nM1 d g s b MX L=1u W=100u M=2\nR1 d 0 1k\n.model MX NMOS(VTO=1)\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err());
    let msg = result.unwrap_err().message;
    assert!(
        msg.contains("instance geometry parameters") && msg.contains("KP"),
        "MOSFET geometry error must point at KP/.model: {msg}"
    );
}

#[test]
fn diode_area_factor_rejected() {
    let spice = "Test\nD1 a 0 DX 2\nR1 a 0 1k\n.model DX D(IS=1e-14)\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "diode area factor must be rejected loudly");
    let msg = result.unwrap_err().message;
    assert!(msg.contains("2"), "error must list the seen token: {msg}");
}

#[test]
fn jfet_trailing_token_rejected() {
    let spice = "Test\nJ1 d g s JX 1.5\nR1 d 0 1k\n.model JX NJF(VTO=-2)\n";
    assert!(Netlist::parse(spice).is_err());
}

#[test]
fn triode_and_opamp_trailing_tokens_rejected() {
    let t = "Test\nT1 g p k 12AX7 extra\nR1 p 0 100k\n.model 12AX7 TRIODE(MU=100)\n";
    assert!(
        Netlist::parse(t).is_err(),
        "triode trailing token must error"
    );
    let u = "Test\nU1 p m o OA1 extra\nR1 o 0 1k\n.model OA1 OA(AOL=1e5)\n";
    assert!(
        Netlist::parse(u).is_err(),
        "opamp trailing token must error"
    );
}

#[test]
fn capacitor_and_inductor_junk_rejected() {
    assert!(
        Netlist::parse("Test\nC1 a 0 1u junk\nR1 a 0 1k\n").is_err(),
        "capacitor junk must error (only IC= allowed)"
    );
    assert!(
        Netlist::parse("Test\nL1 a 0 1m junk\nR1 a 0 1k\n").is_err(),
        "inductor junk must error (only ISAT= allowed)"
    );
    // The legitimate forms keep working
    assert!(Netlist::parse("Test\nC1 a 0 1u IC=2.5\nR1 a 0 1k\n").is_ok());
    assert!(Netlist::parse("Test\nL1 a 0 1m ISAT=20m\nR1 a 0 1k\n").is_ok());
}

#[test]
fn bjt_substrate_node_still_accepted() {
    // 4-node BJT (substrate) is legal; a 5th extra token is not.
    let ok = "Test\nQ1 c b e s QX\nR1 c 0 1k\n.model QX NPN(BF=100)\n";
    assert!(Netlist::parse(ok).is_ok());
    let bad = "Test\nQ1 c b e s QX 2.0\nR1 c 0 1k\n.model QX NPN(BF=100)\n";
    assert!(Netlist::parse(bad).is_err());
}

// ===========================================================================
// Fix 9: model-type validation per element kind
// ===========================================================================

#[test]
fn diode_bound_to_bjt_model_rejected() {
    let spice = "Test\nD1 a 0 QX\nR1 a 0 1k\n.model QX NPN(BF=100)\n";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "diode + NPN card must be a hard error");
    let msg = result.unwrap_err().message;
    assert!(
        msg.contains("D1") && msg.contains("QX") && msg.contains("NPN"),
        "error must name element, model, and actual type: {msg}"
    );
}

#[test]
fn model_type_matrix() {
    // Wrong pairings all rejected
    let cases = [
        ("Q1 c b e DX", ".model DX D(IS=1e-14)"),
        ("J1 d g s MX", ".model MX NMOS(VTO=1)"),
        ("M1 d g s b JX", ".model JX NJF(VTO=-2)"),
        ("T1 g p k PX", ".model PX VP(MU=10)"),
        ("U1 p m o DX2", ".model DX2 D(IS=1e-14)"),
    ];
    for (elem, model) in cases {
        let spice = format!("Test\n{elem}\nR1 c 0 1k\nR2 d 0 1k\nR3 p 0 1k\nR4 o 0 1k\n{model}\n");
        assert!(
            Netlist::parse(&spice).is_err(),
            "type mismatch must be rejected: {elem} with {model}"
        );
    }
    // Right pairings accepted
    let ok = "Test\n\
        D1 a 0 DM\n\
        Q1 c b e QM\n\
        T1 g p k TM\n\
        R1 a c 1k\nR2 b p 1k\nR3 e g 1k\nR4 k 0 1k\n\
        .model DM D(IS=1e-14)\n\
        .model QM NPN(BF=100)\n\
        .model TM TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)\n";
    assert!(
        Netlist::parse(ok).is_ok(),
        "correct type pairings must keep parsing: {:?}",
        Netlist::parse(ok).err()
    );
}

// ===========================================================================
// Fix 10: parsing stops at top-level .end
// ===========================================================================

#[test]
fn elements_after_end_are_not_included() {
    let spice = "Test\nR1 a 0 1k\n.end\nR99 x y 1k\nC99 x 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(
        netlist.elements.len(),
        1,
        "content after .end must be ignored (ngspice semantics)"
    );
    assert_eq!(netlist.elements[0].name(), "R1");
}

// ===========================================================================
// Fix 11: duplicate .model names
// ===========================================================================

#[test]
fn duplicate_model_names_rejected() {
    let spice = "Test\nD1 a 0 DX\nR1 a 0 1k\n\
                 .model DX D(IS=1e-14)\n.model dx D(IS=1e-9)\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "duplicate .model names must be a hard error (first silently won before)"
    );
    let msg = result.unwrap_err().message;
    assert!(msg.to_lowercase().contains("dx"), "{msg}");
}

// ===========================================================================
// Fix 12: directives inside .subckt bodies (warn, not silently discard)
// ===========================================================================

#[test]
fn directive_inside_subckt_is_ignored_but_parse_succeeds() {
    // The .pot inside the subckt is NOT honored (warned) — assert it did not
    // leak into the netlist-level pots.
    let spice = "Test\n\
        .subckt att in out\n\
        R1 in out 10k\n\
        .pot R1 1k 100k\n\
        R2 out 0 10k\n\
        .ends\n\
        X1 a b att\n\
        R3 b 0 1k\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert!(
        netlist.pots.is_empty(),
        ".pot inside a subckt body must not be silently applied"
    );
}

// ===========================================================================
// Fix 13: pot/gang label grammar
// ===========================================================================

#[test]
fn pot_unquoted_label_is_kept() {
    let spice = "Test\nR2 a 0 47k\n.pot R2 1k 100k 47k Tone\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots[0].default_value, Some(47e3));
    assert_eq!(
        netlist.pots[0].label.as_deref(),
        Some("Tone"),
        "unquoted trailing label must not be silently dropped"
    );
}

#[test]
fn gang_multi_token_quoted_label() {
    let spice = "Test\n\
        R1 a b 10k\nR2 b 0 10k\n\
        .pot R1 1k 100k\n.pot R2 1k 100k\n\
        .gang \"Stereo Volume\" R1 R2\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.gangs.len(), 1);
    assert_eq!(netlist.gangs[0].label, "Stereo Volume");
    assert_eq!(netlist.gangs[0].members.len(), 2);
}

#[test]
fn gang_unquoted_label_still_rejected() {
    let spice = "Test\nR1 a b 10k\nR2 b 0 10k\n.pot R1 1k 100k\n.pot R2 1k 100k\n.gang Vol R1 R2\n";
    assert!(Netlist::parse(spice).is_err());
}

// ===========================================================================
// Fix 14: pot <-> switch same-component claim
// ===========================================================================

#[test]
fn pot_and_switch_on_same_resistor_rejected() {
    let spice = "Test\nR1 a 0 10k\nC1 a 0 1n\n\
                 .pot R1 1k 100k\n.switch R1 10k 22k\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        ".pot and .switch on the same component must be rejected"
    );
    let msg = result.unwrap_err().message;
    assert!(msg.contains("R1"), "{msg}");
}

// ===========================================================================
// Fix 15: inline comment stripping respects quoted labels
// ===========================================================================

#[test]
fn semicolon_inside_quoted_label_survives() {
    let spice = "Test\nR1 a 0 10k\n.pot R1 1k 100k \"Bass; Boost\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(
        netlist.pots[0].label.as_deref(),
        Some("Bass; Boost"),
        "';' inside a quoted label must not start a comment"
    );
}

#[test]
fn dollar_inside_quoted_label_survives() {
    let spice = "Test\nR1 a 0 10k\n.pot R1 1k 100k \"Big $ Knob\"\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.pots[0].label.as_deref(), Some("Big $ Knob"));
}

#[test]
fn unquoted_inline_comments_still_stripped() {
    let spice = "Test\nR1 a 0 10k ; this is a comment\nC1 a 0 1n $ another\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.elements.len(), 2);
}

// ===========================================================================
// Fix 16: title heuristic (behavioral no-op — first line is still the title)
// ===========================================================================

#[test]
fn first_line_is_always_title_even_if_element_shaped() {
    // SPICE semantics unchanged: line 1 is the title (a warn is logged).
    let spice = "R1 in out 1k\nR2 out 0 1k\nC1 out 0 1n\n";
    let netlist = Netlist::parse(spice).unwrap();
    assert_eq!(netlist.title, "R1 in out 1k");
    assert_eq!(netlist.elements.len(), 2, "R1 was consumed as the title");
}
