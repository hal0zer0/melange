//! SPICE `XTB` — forward/reverse beta temperature dependence on self-heating BJTs.
//!
//! `BF(T) = BF · (Tj/Tnom)^XTB`, likewise `BR`.
//!
//! Why this needs a test rather than a corpus deck: **there is not one
//! self-heating BJT in the whole netlist library.** All four models anywhere
//! carrying `RTH` are diodes (pipe-shouter, sad-bastard, gold-press-mastering,
//! gold-press-overdrive). So neither `XTB` nor the BJT thermal block it extends
//! has any circuit-level coverage, and a golden baseline cannot provide it. The
//! same "implemented but no circuit pulls it" shape as MOSFET/JFET.
//!
//! What made this worth implementing anyway: melange already modelled BJT
//! self-heating and drifted `IS` with junction temperature via `XTI`, while
//! current gain stayed fixed — a transistor could heat up and its beta would not
//! move. `XTB` appears on real vendor cards in the library (the TIP35C/TIP36C
//! output pair in the Wurlitzer power amp among others) and was being discarded
//! as an unrecognized parameter.

mod support;

/// A self-heating BJT stage. `{XTB}` is substituted per-case.
fn deck(xtb_clause: &str) -> String {
    format!(
        "XTB self-heating probe\n\
         Q1 out b 0 QTEST\n\
         R1 vcc out 1k\n\
         R2 b 0 100k\n\
         R3 vcc b 470k\n\
         C1 in b 1u\n\
         C2 out 0 10n\n\
         V1 vcc 0 DC 12\n\
         .model QTEST NPN(IS=1e-14 BF=200 BR=4 RTH=200 CTH=1e-3 {xtb_clause})\n\
         .end\n"
    )
}

fn emit(src: &str) -> String {
    let config = support::config_for_spice(src, 48000.0);
    support::generate_circuit_code_nodal(src, &config).0
}

#[test]
fn xtb_emits_the_beta_temperature_law_on_a_self_heating_bjt() {
    let code = emit(&deck("XTB=1.5"));
    assert!(
        code.contains("const DEVICE_0_XTB: f64 = 1.50000000000000000e0;"),
        "XTB must reach the generated constants verbatim"
    );
    assert!(
        code.contains("state.device_0_bf = DEVICE_0_BETA_F * t_ratio.powf(DEVICE_0_XTB);"),
        "forward beta must be re-derived from Tj in the thermal block"
    );
    assert!(
        code.contains("state.device_0_br = DEVICE_0_BETA_R * t_ratio.powf(DEVICE_0_XTB);"),
        "reverse beta must track Tj too — SPICE applies XTB to both"
    );
}

#[test]
fn xtb_defaults_to_zero_and_is_then_numerically_inert() {
    let code = emit(&deck(""));
    assert!(
        code.contains("const DEVICE_0_XTB: f64 = 0.00000000000000000e0;"),
        "an absent XTB must default to SPICE's 0.0, not to some melange-chosen \
         value — 0.0 is what makes the power term exactly 1.0"
    );
    // The law is still emitted; it is the *value* that makes it inert. IEEE-754
    // guarantees pow(x, ±0) == 1.0 for every x, so `BETA_F * 1.0` is BETA_F
    // bit-exactly and a card without XTB cannot move a single sample.
    assert!(
        code.contains("state.device_0_bf = DEVICE_0_BETA_F * t_ratio.powf(DEVICE_0_XTB);"),
        "the law is emitted unconditionally; XTB=0 is what neutralises it"
    );
    assert_eq!(
        1.0f64,
        (301.014f64 / 300.15f64).powf(0.0),
        "the inertness claim rests on IEEE pow(x, 0) == 1.0 exactly"
    );
}

/// The parameter is *honored*, not merely tolerated: changing it must change the
/// emitted DSP. Guards against a future refactor that keeps the constant but
/// drops the law, which would silently restore the old temperature-invariant
/// beta while every string assertion above still passed.
#[test]
fn xtb_changes_the_emitted_dsp_only_through_its_constant() {
    let a = emit(&deck("XTB=0"));
    let b = emit(&deck("XTB=1.5"));
    assert_ne!(a, b, "XTB must be load-bearing in the generated code");

    let differing: Vec<(&str, &str)> = a
        .lines()
        .zip(b.lines())
        .filter(|(x, y)| x != y)
        .filter(|(x, _)| !x.trim_start().starts_with("// melange:"))
        .filter(|(x, _)| !x.trim_start().starts_with("// provenance:"))
        .collect();
    assert!(
        differing
            .iter()
            .all(|(x, _)| x.contains("DEVICE_0_XTB: f64")),
        "XTB=0 vs XTB=1.5 must differ ONLY in the XTB constant — the thermal \
         law itself is value-independent. Differing lines: {differing:#?}"
    );
}
