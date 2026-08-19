//! Anti-twin structural guarantee for the thermal self-heating state-advance.
//!
//! The DK and nodal emitters are independent (DK = Tera templates + hot
//! `CircuitState`; nodal = procedural `push_str` + `Box<CircuitStateCold>`), so
//! every per-device state fragment authored twice can drift. The junction-
//! temperature (`Tj`) per-sample advance already suffered exactly this: the
//! nodal path once baked `dt` from codegen-time rate consts while DK read the
//! live `state.current_sample_rate` (fixed in 746bdb8). Stage 0 of Phase 0c
//! collapses the advance to ONE shared helper
//! (`helpers::emit_thermal_tj_advance`) that BOTH emitters splice verbatim.
//!
//! This test locks that in: for a canonical `RTH` device, the inner advance
//! block emitted by the DK path and by the nodal path must be BYTE-IDENTICAL.
//! Outer scaffolding (how `p` is computed, the enclosing block) legitimately
//! differs between paths; the advance expression itself must not. This is the
//! structural guard that would have caught the original baked-dt bug at source.

mod support;

const SR: f64 = 48_000.0;

/// One self-heating diode (device 0), a cap for DK conditioning, named
/// `in`/`out` nodes. `RTH`/`CTH` finite → the `CTH > 0` exact-exponential
/// advance branch.
const RTH_DIODE: &str = "RTH diode thermal twin
Rin in n1 1k
D1 n1 out DTH
Rout out 0 47k
Cout out 0 10n
.MODEL DTH D(IS=2.52e-9 N=1.9 RTH=500 CTH=2e-4)
";

/// Extract the contiguous Tj-advance inner block for device 0: from the start
/// of the `let dt = 1.0 / (state.current_sample_rate …)` line through the
/// runaway-clamp line that ends it. Includes leading indentation so the
/// comparison is truly byte-for-byte.
fn extract_tj_advance(code: &str, path: &str) -> String {
    let dt_marker = "let dt = 1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64);";
    let s = code.find(dt_marker).unwrap_or_else(|| {
        panic!("{path}: live-rate dt binding not found in emitted thermal block")
    });
    // Back up to the beginning of the `let dt` line (capture indentation).
    let line_start = code[..s].rfind('\n').map(|i| i + 1).unwrap_or(0);
    let end_marker = ".clamp(200.0, 500.0);";
    let e_rel = code[s..]
        .find(end_marker)
        .unwrap_or_else(|| panic!("{path}: Tj runaway clamp not found after dt binding"));
    let e = s + e_rel + end_marker.len();
    code[line_start..e].to_string()
}

#[test]
fn thermal_tj_advance_dk_nodal_string_identity() {
    let config = support::config_for_spice(RTH_DIODE, SR);

    let (dk_code, _n, _m) = support::generate_circuit_code(RTH_DIODE, &config);
    let (nodal_code, _n, _m) = support::generate_circuit_code_nodal(RTH_DIODE, &config);

    // Both paths must actually emit a self-heating update for the diode.
    assert!(
        dk_code.contains("Diode 0 self-heating thermal update"),
        "DK path did not emit a self-heating block for the RTH diode"
    );
    assert!(
        nodal_code.contains("Diode 0 self-heating thermal update"),
        "nodal path did not emit a self-heating block for the RTH diode"
    );

    let dk_block = extract_tj_advance(&dk_code, "DK");
    let nodal_block = extract_tj_advance(&nodal_code, "nodal");

    assert_eq!(
        dk_block, nodal_block,
        "twin divergence: the DK and nodal thermal Tj-advance inner blocks differ.\n\
         Both must come from the single shared `helpers::emit_thermal_tj_advance`.\n\
         --- DK ---\n{dk_block}\n--- nodal ---\n{nodal_block}\n"
    );

    // Regression guard: the shared block must use the LIVE sample rate, never a
    // baked SAMPLE_RATE/INTERNAL_SAMPLE_RATE const (the original nodal bug).
    assert!(
        dk_block.contains("state.current_sample_rate * OVERSAMPLING_FACTOR as f64"),
        "thermal dt must be derived from the live current_sample_rate, not a baked const"
    );
    assert!(
        !dk_block.contains("SAMPLE_RATE;") && !dk_block.contains("INTERNAL_SAMPLE_RATE"),
        "thermal dt must not be baked from codegen-time rate consts"
    );

    // Lock the exact canonical form (DK's, per plan) so an accidental reorder or
    // rewrite on EITHER path trips this test rather than silently regenerating.
    let expected = "\
        let dt = 1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64);\n\
        \x20       let tss = DEVICE_0_TAMB + p * DEVICE_0_RTH;\n\
        \x20       let tau = DEVICE_0_RTH * DEVICE_0_CTH;\n\
        \x20       state.device_0_tj += (tss - state.device_0_tj) * (1.0 - (-dt / tau).exp());\n\
        \x20       state.device_0_tj = state.device_0_tj.clamp(200.0, 500.0);";
    assert_eq!(
        dk_block.trim_start(),
        expected.trim_start(),
        "shared thermal advance block no longer matches the canonical form"
    );
}
