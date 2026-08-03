//! DK template state-lifecycle regression tests (2026-07 campaign).
//!
//! Covers the verified bugs in `set_sample_rate` / `reset()` / the NaN
//! recovery path / the DC-blocker feedback state / the BE-fallback vs
//! companion-magnetics interaction:
//!
//! 1. Same-rate `set_sample_rate` must not revert moved pots to nominal
//!    matrices (the all_default guard now checks pot resistances too).
//! 2. The pot-variant `set_sample_rate` must recompute `dc_block_r` on a
//!    genuine rate change (previously only the no-pot variant did).
//! 3. `reset()` marks matrices dirty after restoring pot/switch defaults
//!    and restores the baked DC operating point (factory-state contract).
//! 4. The DC-blocker feedback state carries no ±100 V clamp (masking class;
//!    corrupted the HPF state for tube-level >100 V outputs).
//! 5. DK BE fallback is gated OFF for companion-model magnetics (the BE
//!    matrices/RHS lack the companion stamps and history).
//! 6. `.runtime V` sources are stamped into the BE-fallback RHS.
//! 11. Same-rate `set_sample_rate` calls preserve transient state
//!     (magnetics standing currents, filter history) — no click.

mod support;

use melange_solver::codegen::CodegenConfig;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn default_config() -> CodegenConfig {
    CodegenConfig {
        circuit_name: "lifecycle_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

/// Generate DK code with the standard harness (stamps G_in before the
/// kernel build, per the Thevenin input contract).
fn generate_with(spice: &str, config: CodegenConfig) -> String {
    support::generate_circuit_code(spice, &config).0
}

fn generate_code(spice: &str) -> String {
    generate_with(spice, default_config())
}

/// Extract the body of `pub fn reset(` up to the next `pub fn`.
fn extract_reset_body(code: &str) -> &str {
    let start = code.find("pub fn reset(").expect("missing reset()");
    let rest = &code[start..];
    let end = rest[13..].find("pub fn ").expect("missing following fn") + 13;
    &rest[..end]
}

// ---------------------------------------------------------------------------
// Circuits
// ---------------------------------------------------------------------------

/// Linear pot divider: pot position strongly changes the divider ratio.
const POT_DIVIDER: &str = "\
Pot Divider
R1 in out 10k
R2 out 0 10k
C1 out 0 100n
.pot R1 1k 100k
";

/// Pot + companion inductor: standing current through L1 at low frequency.
const POT_INDUCTOR: &str = "\
Pot Inductor Continuity
R1 in mid 1k
L1 mid out 100m
R2 out 0 1k
C1 out 0 100n
.pot R1 500 10k
";

/// Series RLC resonator, Q ≈ 10 at ~50 Hz: the cap node legitimately
/// swings > 100 V for a ~16 V drive. Exercises the DC blocker at
/// tube-level amplitudes with per-sample deltas below the global
/// voltage-damper threshold (2 V).
const RLC_RESONATOR: &str = "\
RLC Resonator
R1 in a 30
L1 a b 1
C1 b 0 10u
";

/// Nonlinear + companion inductor: BE fallback must be gated off.
const DIODE_INDUCTOR: &str = "\
Diode Inductor
R1 in a 1k
L1 a b 10m
D1 b 0 DMOD
C1 b 0 100n
.model DMOD D(IS=1e-14)
";

/// Nonlinear + coupled inductors: BE fallback must be gated off.
const DIODE_XFMR: &str = "\
Diode Transformer
R1 in p 100
L1 p 0 100m
L2 s 0 100m
K1 L1 L2 0.95
D1 s 0 DMOD
C1 s 0 100n
.model DMOD D(IS=1e-14)
";

/// Nonlinear, magnetics-free control: BE fallback stays available.
const DIODE_CLIPPER: &str = "\
Diode Clipper
R1 in a 1k
D1 a 0 DMOD
C1 a 0 100n
.model DMOD D(IS=1e-14)
";

/// Nonlinear circuit with a DC source flagged `.runtime` (host-driven).
const RUNTIME_V_DIODE: &str = "\
Runtime Diode
V1 vcc 0 DC 5
R1 vcc a 10k
R2 in a 4.7k
D1 a 0 DMOD
C1 a 0 100n
.model DMOD D(IS=1e-14)
.runtime V1 as supply_mod
";

/// Self-heating diode with NO pots/switches: the thermal dt reads
/// `state.current_sample_rate`, so the field gate must cover thermal devices.
const THERMAL_DIODE_NO_POTS: &str = "\
Thermal Diode NoPots
Rin in out 1k
D1 out 0 DTH
C1 out 0 100n
.model DTH D(IS=2.52e-9 N=1.752 RTH=500 CTH=2e-4)
";

// ---------------------------------------------------------------------------
// Fix 1 — same-rate set_sample_rate must not revert moved pots
// ---------------------------------------------------------------------------

#[test]
fn same_rate_set_sample_rate_keeps_moved_pot() {
    let code = generate_code(POT_DIVIDER);

    let main_code = r#"
fn main() {
    let sr = 44100.0_f64;
    // Path A: pot moved to 50k, then a same-rate set_sample_rate call
    // (regression: the old all_default guard checked only switches and
    // silently reloaded the nominal-pot S_DEFAULT/K_DEFAULT matrices).
    let mut a = CircuitState::default();
    a.set_pot_0(50_000.0);
    for _ in 0..500 { let _ = process_sample(0.0, &mut a); }
    a.set_sample_rate(sr);

    // Path B: pot moved to 50k, no rate call — the reference behavior.
    let mut b = CircuitState::default();
    b.set_pot_0(50_000.0);
    for _ in 0..500 { let _ = process_sample(0.0, &mut b); }

    // Path C: nominal pot — what the bug reverted Path A to.
    let mut c = CircuitState::default();
    for _ in 0..500 { let _ = process_sample(0.0, &mut c); }

    let mut max_ab = 0.0_f64;
    let mut max_ac = 0.0_f64;
    for i in 0..4410 {
        let x = (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin() * 0.5;
        let ya = process_sample(x, &mut a)[0];
        let yb = process_sample(x, &mut b)[0];
        let yc = process_sample(x, &mut c)[0];
        max_ab = max_ab.max((ya - yb).abs());
        max_ac = max_ac.max((ya - yc).abs());
    }
    println!("max_ab={:.6e}", max_ab);
    println!("max_ac={:.6e}", max_ac);
}
"#;
    let out = support::compile_and_run(&code, main_code, "same_rate_pot");
    let max_ab = out.parse_kv("max_ab").expect("missing max_ab");
    let max_ac = out.parse_kv("max_ac").expect("missing max_ac");
    // Sanity: the pot position audibly changes the divider (otherwise the
    // test proves nothing).
    assert!(
        max_ac > 1e-2,
        "pot position should change the output substantially, got {max_ac:.3e}"
    );
    // The same-rate call must be transparent for a moved pot.
    assert!(
        max_ab < 1e-9,
        "same-rate set_sample_rate reverted the moved pot: max |A-B| = {max_ab:.3e}"
    );
}

// ---------------------------------------------------------------------------
// Fix 2 — pot-variant set_sample_rate recomputes dc_block_r on rate change
// ---------------------------------------------------------------------------

#[test]
fn pot_variant_rate_change_recomputes_dc_block_r() {
    let code = generate_code(POT_DIVIDER);
    // Previously the recompute line existed only in the no-pot template
    // variant; a pot circuit's set_sample_rate ended at rebuild_matrices()
    // and kept the baked DC_BLOCK_R at any host rate.
    assert!(
        code.contains("self.dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;"),
        "pot-variant set_sample_rate must recompute dc_block_r on a genuine rate change"
    );
    // The same-rate paths restore the baked coefficient.
    assert!(
        code.contains("self.dc_block_r = DC_BLOCK_R;"),
        "same-rate paths must restore the baked DC_BLOCK_R"
    );
}

// ---------------------------------------------------------------------------
// Fixes 3 + 10 — reset(): matrices_dirty + coherent factory state
// ---------------------------------------------------------------------------

#[test]
fn reset_marks_matrices_dirty_on_pot_circuit() {
    let code = generate_code(POT_DIVIDER);
    let reset = extract_reset_body(&code);
    assert!(
        reset.contains("self.matrices_dirty = true;"),
        "reset() must force a matrix rebuild after restoring pot/switch defaults:\n{reset}"
    );
}

#[test]
fn reset_restores_baked_dc_op() {
    // Needs a circuit with a DC operating point (has_dc_op).
    let code = generate_code(RUNTIME_V_DIODE);
    let reset = extract_reset_body(&code);
    assert!(
        reset.contains("self.dc_operating_point = DC_OP;"),
        "reset() must restore the baked DC_OP so it agrees with the baked \
         DC_NL_I and nominal pot values:\n{reset}"
    );
}

// ---------------------------------------------------------------------------
// Fix 4 — DC blocker feedback state must not be magnitude-clamped
// ---------------------------------------------------------------------------

#[test]
fn dc_block_feedback_state_unclamped_text() {
    let code = generate_code(POT_DIVIDER);
    assert!(
        code.contains("state.dc_block_y_prev[out_idx] = dc_blocked + 1e-20;"),
        "DC blocker feedback must keep the denormal bias without a clamp"
    );
    assert!(
        !code.contains(".clamp(-100.0, 100.0)")
            || !code
                .lines()
                .any(|l| l.contains("dc_block_y_prev") && l.contains(".clamp(")),
        "DC blocker feedback state must not be magnitude-clamped (masking class)"
    );
}

#[test]
fn dc_block_stays_linear_above_100v() {
    // RLC resonator: ~162 V peaks across the cap at 16 V drive (Q ≈ 10).
    // With the old ±100 V clamp on dc_block_y_prev the HPF feedback state
    // saturated, breaking linearity between the 8 V and 16 V runs.
    let config = CodegenConfig {
        output_clamp_v: 1000.0,
        output_nodes: vec![2],
        ..default_config()
    };
    let code = generate_with(RLC_RESONATOR, config);

    let main_code = r#"
fn main() {
    let sr = 44100.0_f64;
    // Drive exactly at the LC resonance (~50.3 Hz). Low frequency keeps
    // per-sample output deltas (~1.2 V at 162 V peak) under the 2 V global
    // voltage-damping threshold — the damper must stay silent here.
    let f0 = 1.0 / (2.0 * std::f64::consts::PI * (1.0_f64 * 10e-6).sqrt());
    let peak_for = |amp: f64| -> (f64, u64) {
        let mut st = CircuitState::default();
        let mut peak = 0.0_f64;
        for i in 0..44100 {
            let x = (2.0 * std::f64::consts::PI * f0 * i as f64 / sr).sin() * amp;
            let y = process_sample(x, &mut st)[0];
            // Skip the resonance build-up (~Q cycles) and blocker settle
            if i > 35280 { peak = peak.max(y.abs()); }
        }
        (peak, st.diag_voltage_damp_count)
    };
    let (p_small, damp_small) = peak_for(8.0);
    let (p_large, damp_large) = peak_for(16.0);
    println!("p_small={:.6e}", p_small);
    println!("p_large={:.6e}", p_large);
    println!("damp={:.1}", (damp_small + damp_large) as f64);
}
"#;
    let out = support::compile_and_run(&code, main_code, "blocker_linear");
    let p_small = out.parse_kv("p_small").expect("missing p_small");
    let p_large = out.parse_kv("p_large").expect("missing p_large");
    let damp = out.parse_kv("damp").expect("missing damp");
    // The damper must not have fired — otherwise it, not the blocker,
    // shapes the output and the test proves nothing.
    assert_eq!(damp, 0.0, "voltage damper fired; test vehicle invalid");
    // The large run must actually exceed the old ±100 V clamp range (and
    // stay under the 1000 V output clamp) for the test to be meaningful.
    assert!(
        p_large > 120.0 && p_large < 900.0,
        "output should legitimately swing above 100 V (below the output clamp), got {p_large:.1}"
    );
    // Linearity: 2x input → 2x output (linear circuit; the old ±100 V clamp
    // on dc_block_y_prev corrupted the blocker state and broke this badly).
    let ratio = p_large / p_small;
    assert!(
        (ratio - 2.0).abs() < 0.02,
        "linear circuit must scale linearly through the DC blocker: \
         peak ratio {ratio:.4} (small {p_small:.3}, large {p_large:.3})"
    );
}

// ---------------------------------------------------------------------------
// Fix 5 — BE fallback gated off for companion-model magnetics
// ---------------------------------------------------------------------------

#[test]
fn companion_inductor_disables_be_fallback() {
    let code = generate_code(DIODE_INDUCTOR);
    assert!(
        !code.contains("S_BE_DEFAULT"),
        "companion-inductor circuit must not emit BE fallback matrices \
         (they lack the companion stamps and history)"
    );
    assert!(
        !code.contains("let mut rhs_be"),
        "companion-inductor circuit must not emit the BE fallback RHS path"
    );
}

#[test]
fn companion_transformer_disables_be_fallback() {
    let code = generate_code(DIODE_XFMR);
    assert!(
        !code.contains("S_BE_DEFAULT"),
        "coupled-inductor circuit must not emit BE fallback matrices"
    );
}

#[test]
fn magnetics_free_nonlinear_keeps_be_fallback() {
    // Control: the gate must be precise — no magnetics, fallback stays.
    let code = generate_code(DIODE_CLIPPER);
    assert!(
        code.contains("S_BE_DEFAULT"),
        "magnetics-free nonlinear circuit must keep the BE fallback"
    );
}

// ---------------------------------------------------------------------------
// Fix 6 — .runtime V sources stamped into the BE fallback RHS
// ---------------------------------------------------------------------------

#[test]
fn runtime_v_source_stamped_in_be_fallback_rhs() {
    let code = generate_code(RUNTIME_V_DIODE);
    // Sanity: BE fallback exists for this circuit (nonlinear, no magnetics).
    assert!(
        code.contains("let mut rhs_be"),
        "expected the BE fallback RHS path in this circuit"
    );
    // build_rhs stamps the runtime source...
    assert!(
        code.lines()
            .any(|l| l.trim_start().starts_with("rhs[") && l.contains("+= state.supply_mod;")),
        "build_rhs must stamp the runtime source"
    );
    // ...and the BE fallback RHS must stamp the SAME row/value (previously
    // dropped on every fallback sample).
    assert!(
        code.lines()
            .any(|l| l.trim_start().starts_with("rhs_be[") && l.contains("+= state.supply_mod;")),
        "BE fallback RHS must stamp the runtime source rows (scheme-independent)"
    );
}

#[test]
fn rhs_const_be_guard_symmetry() {
    // Fix 8 guard alignment: whenever the BE RHS references RHS_CONST_BE,
    // the constant must be emitted (both keyed on has_dc_sources).
    let code = generate_code(RUNTIME_V_DIODE);
    // The BE RHS is now emitted as sparse per-row lines (`RHS_CONST_BE[0]`,
    // `[1]`, …) rather than the old `RHS_CONST_BE[i]` loop form; match any
    // indexing reference. The const decl is `RHS_CONST_BE:` (colon), so this
    // matches only usages, not the definition.
    let referenced = code.contains("RHS_CONST_BE[");
    let emitted = code.contains("pub const RHS_CONST_BE");
    assert_eq!(
        referenced, emitted,
        "RHS_CONST_BE reference/emission must be symmetric (referenced={referenced}, emitted={emitted})"
    );
    assert!(
        emitted,
        "DC-source circuit with BE fallback must emit RHS_CONST_BE"
    );
}

// ---------------------------------------------------------------------------
// Fix 7 — thermal devices gate the current_sample_rate field (no pots)
// ---------------------------------------------------------------------------

#[test]
fn thermal_device_without_pots_has_current_sample_rate_field() {
    let code = generate_code(THERMAL_DIODE_NO_POTS);
    assert!(
        code.contains("pub current_sample_rate"),
        "thermal circuits must carry current_sample_rate (dt reads the live host rate)"
    );
    assert!(
        code.contains("let dt = 1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64);"),
        "thermal dt must use the live host rate"
    );
    // The field gate regression is a rustc failure — prove it compiles and runs.
    let main_code = r#"
fn main() {
    let mut st = CircuitState::default();
    st.set_sample_rate(96_000.0);
    let mut last = 0.0;
    for i in 0..1000 {
        let x = (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 96_000.0).sin() * 0.5;
        last = process_sample(x, &mut st)[0];
    }
    assert!(last.is_finite());
    println!("ok=1.0");
}
"#;
    let out = support::compile_and_run(&code, main_code, "thermal_no_pots");
    assert_eq!(out.parse_kv("ok"), Some(1.0));
}

// ---------------------------------------------------------------------------
// Fix 11 — same-rate set_sample_rate preserves transient state
// ---------------------------------------------------------------------------

#[test]
fn same_rate_call_preserves_transient_state() {
    let code = generate_code(POT_INDUCTOR);

    let main_code = r#"
fn main() {
    let sr = 44100.0_f64;
    let mut test = CircuitState::default();
    let mut ctrl = CircuitState::default();
    let mut max_diff = 0.0_f64;
    for i in 0..4410 {
        // 200 Hz keeps a healthy standing current in the 100 mH inductor.
        let x = (2.0 * std::f64::consts::PI * 200.0 * i as f64 / sr).sin() * 0.5;
        if i == 2205 {
            // No-op rate call mid-signal: previously zeroed the inductor
            // standing current (pot-variant early-out) → audible click.
            test.set_sample_rate(sr);
        }
        let yt = process_sample(x, &mut test)[0];
        let yc = process_sample(x, &mut ctrl)[0];
        max_diff = max_diff.max((yt - yc).abs());
    }
    println!("max_diff={:.6e}", max_diff);
}
"#;
    let out = support::compile_and_run(&code, main_code, "same_rate_preserve");
    let max_diff = out.parse_kv("max_diff").expect("missing max_diff");
    assert!(
        max_diff < 1e-12,
        "a same-rate set_sample_rate call must be transparent to transient \
         state, got max diff {max_diff:.3e}"
    );
}
