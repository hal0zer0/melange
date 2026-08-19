//! Phase 0c Stage 1c — CdsLdr (opto/LDR) end-to-end verification.
//!
//! Covers the four verification legs the plan requires for the first customer
//! of the device-agnostic stateful-device interface:
//!   1. parser + IR: `O…` element → `DeviceType::Ldr` slot carrying a
//!      `StatefulSpec` (state_size 1, seed [r_max], two driving nodes), and the
//!      `.model … LDR()` empty-parens catalog fill.
//!   2. twin string-identity: the DK and nodal paths emit a byte-identical
//!      `stateful_update_dev0` hook (shared-helper guarantee).
//!   3. two-solver output parity: the same deck compiled through DK and nodal
//!      renders matching audio (the twin backstop).
//!   4. functional attack/release oracle: the generated `r_state` trajectory
//!      matches `melange-devices` `CdsLdr` sample-for-sample (same equation —
//!      this checks the codegen path faithfully carries it, not the physics).

mod support;

use melange_solver::codegen::ir::{CircuitIR, DeviceParams, DeviceType};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::{Element, Netlist};

const SR: f64 = 44_100.0;

/// A compile-ready LDR divider with `in`/`out` nodes and a fixed DC control.
const LDR_DECK: &str = "LDR divider test
Rin in a 10k
O1 a out lfo 0 VTL5C3
Rload out 0 100k
Cout out 0 10n
Vlfo lfo 0 DC 0.5
.model VTL5C3 LDR()
";

// ── 1. Parser + IR ──────────────────────────────────────────────────────

#[test]
fn ldr_parses_to_element_with_four_nodes() {
    let netlist = Netlist::parse(LDR_DECK).expect("parse");
    let ldr = netlist
        .elements
        .iter()
        .find_map(|e| match e {
            Element::Ldr {
                name,
                n_plus,
                n_minus,
                n_ctrl_p,
                n_ctrl_n,
                model,
            } => Some((name, n_plus, n_minus, n_ctrl_p, n_ctrl_n, model)),
            _ => None,
        })
        .expect("O1 should parse to Element::Ldr");
    assert_eq!(ldr.0, "O1");
    assert_eq!(ldr.1, "a"); // r+
    assert_eq!(ldr.2, "out"); // r-
    assert_eq!(ldr.3, "lfo"); // ctrl+
    assert_eq!(ldr.4, "0"); // ctrl- (ground)
    assert_eq!(ldr.5, "VTL5C3");
}

#[test]
fn ldr_ir_slot_is_stateful_with_catalog_params() {
    let netlist = Netlist::parse(LDR_DECK).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).expect("slots");

    let slot = slots
        .iter()
        .find(|s| s.device_type == DeviceType::Ldr)
        .expect("an LDR device slot");
    assert_eq!(slot.dimension, 1, "LDR resistance path is 1-D");

    // Catalog fill (VTL5C3 empty-parens → ldr.rs vtl5c3 values).
    let lp = match &slot.params {
        DeviceParams::Ldr(lp) => lp,
        other => panic!("expected DeviceParams::Ldr, got {other:?}"),
    };
    assert_eq!(lp.r_min, 75.0);
    assert_eq!(lp.r_max, 10e6);
    assert_eq!(lp.gamma, 0.7);
    assert_eq!(lp.attack_tau, 0.005);
    assert_eq!(lp.release_tau, 0.2);

    // Opaque stateful spec: N=1, dark seed, 4 terminals, 2 driving nodes.
    let spec = slot
        .stateful
        .as_ref()
        .expect("LDR must carry a StatefulSpec");
    assert_eq!(spec.state_size, 1);
    assert_eq!(spec.state_seed, vec![10e6]);
    assert_eq!(spec.terminal_nodes.len(), 4);
    assert_eq!(spec.driving_nodes.len(), 2, "differential control pair");
    // driving_nodes are terminal_nodes[2..4] (ctrl+, ctrl-).
    assert_eq!(spec.driving_nodes[0], spec.terminal_nodes[2]);
    assert_eq!(spec.driving_nodes[1], spec.terminal_nodes[3]);
}

#[test]
fn ldr_explicit_params_override_catalog() {
    let deck = "LDR override
Rin in a 10k
O1 a out lfo 0 MYLDR
Rload out 0 100k
Cout out 0 10n
Vlfo lfo 0 DC 0.5
.model MYLDR LDR(RMIN=100 RMAX=2e6 GAMMA=1.0 TAU_A=0.001 TAU_R=0.05)
";
    let netlist = Netlist::parse(deck).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).expect("slots");
    let lp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Ldr(lp) => Some(lp),
            _ => None,
        })
        .expect("LDR slot");
    assert_eq!(lp.r_min, 100.0);
    assert_eq!(lp.r_max, 2e6);
    assert_eq!(lp.gamma, 1.0);
    assert_eq!(lp.attack_tau, 0.001);
    assert_eq!(lp.release_tau, 0.05);
}

// ── 2. Twin string-identity ─────────────────────────────────────────────

/// Extract the `stateful_update_dev0` function block (from its `fn` line
/// through the matching closing brace at column 0).
fn extract_update_fn(code: &str, path: &str) -> String {
    let start = code
        .find("fn stateful_update_dev0(")
        .unwrap_or_else(|| panic!("{path}: no stateful_update_dev0 fn emitted"));
    // The fn ends at the first line that is exactly "}" (column-0 close).
    let rest = &code[start..];
    let end_rel = rest
        .find("\n}\n")
        .unwrap_or_else(|| panic!("{path}: could not find end of stateful_update_dev0"));
    rest[..end_rel + 3].to_string()
}

#[test]
fn ldr_stateful_update_dk_nodal_string_identity() {
    let config = support::config_for_spice(LDR_DECK, SR);
    let (dk_code, _n, _m) = support::generate_circuit_code(LDR_DECK, &config);
    let (nodal_code, _n, _m) = support::generate_circuit_code_nodal(LDR_DECK, &config);

    // Both paths must actually emit the LDR hook + NR eval.
    for (code, tag) in [(&dk_code, "DK"), (&nodal_code, "nodal")] {
        assert!(
            code.contains("fn stateful_update_dev0("),
            "{tag}: missing LDR update hook"
        );
        assert!(
            code.contains("state.device_0_state[0].max(1e-12)"),
            "{tag}: missing LDR NR conductance eval"
        );
    }

    let dk_fn = extract_update_fn(&dk_code, "DK");
    let nodal_fn = extract_update_fn(&nodal_code, "nodal");
    assert_eq!(
        dk_fn, nodal_fn,
        "twin divergence: DK and nodal emit different stateful_update_dev0 bodies.\n\
         Both must come from the single shared `helpers::emit_stateful_update_fns`.\n\
         --- DK ---\n{dk_fn}\n--- nodal ---\n{nodal_fn}\n"
    );

    // The body must faithfully carry ldr.rs's equation: power-law target +
    // asymmetric attack/release + live-rate coefficient exp(-dt/tau).
    assert!(
        dk_fn.contains("(1.0 - cv).powf(DEVICE_0_GAMMA)"),
        "power-law target"
    );
    assert!(
        dk_fn.contains("if target_r < r { DEVICE_0_TAU_A } else { DEVICE_0_TAU_R }"),
        "asymmetric attack/release selection"
    );
    assert!(dk_fn.contains("(-dt / tau).exp()"), "live-rate coefficient");
    assert!(
        dk_fn.contains("StatefulUpdate::default()"),
        "reserved no-op return"
    );
    // Control clamped to [0,1] inside update() (graceful out-of-range).
    assert!(
        dk_fn.contains(".clamp(0.0, 1.0)"),
        "control clamped to [0,1]"
    );
}

// ── 3. Two-solver output parity ─────────────────────────────────────────

#[test]
fn ldr_two_solver_output_parity() {
    let config = support::config_for_spice(LDR_DECK, SR);
    let dk = support::build_circuit(LDR_DECK, &config, "ldr_dk");
    let nodal = support::build_circuit_nodal(LDR_DECK, &config, "ldr_nodal");

    // 200 Hz sine, long enough for the LDR r_state to settle to its DC-control
    // fixed point on BOTH paths. The early transient legitimately differs: the
    // nodal path's `default()` runs `warmup()` (which advances r_state toward
    // the control target), while the DK path seeds v_prev = dc_op directly and
    // lets r_state settle during processing. This is the general DK-vs-nodal
    // initialization asymmetry (it applies to every state field), NOT an LDR
    // effect. The twin backstop is therefore the SETTLED region, where both
    // r_state trajectories have converged to the same fixed point.
    let n = 8000;
    let input: Vec<f64> = (0..n)
        .map(|i| 0.3 * (2.0 * std::f64::consts::PI * 200.0 * i as f64 / SR).sin())
        .collect();

    let dk_out = support::run_signal(&dk, &input, SR);
    let nodal_out = support::run_signal(&nodal, &input, SR);
    support::assert_finite(&dk_out);
    support::assert_finite(&nodal_out);
    support::assert_peak_above(&dk_out, 1e-4);
    support::assert_peak_above(&nodal_out, 1e-4);

    // Settled region: both LDRs at their fixed point → the two solvers agree to
    // linear-algebra precision (measured ~1.7e-7; 1e-6 is a safe tight bound).
    let settle = 5000;
    support::assert_samples_match(
        &dk_out[settle..],
        &nodal_out[settle..],
        1e-6,
        "LDR DK vs nodal (settled)",
    );
}

// ── 4. Functional attack/release oracle vs ldr.rs ───────────────────────

/// Runtime-controllable LDR: `.runtime` exposes the brightness node so the
/// test can step it and read `device_0_state[0]` back.
const LDR_RUNTIME_DECK: &str = "LDR runtime control
Rin in a 10k
O1 a out ctl 0 VTL5C3
Rload out 0 100k
Cout out 0 10n
Vctl ctl 0 DC 0
.runtime Vctl as brightness
.model VTL5C3 LDR()
";

#[test]
fn ldr_attack_release_matches_cdsldr_oracle() {
    let config = support::config_for_spice(LDR_RUNTIME_DECK, SR);
    let (code, _n, _m) = support::generate_circuit_code(LDR_RUNTIME_DECK, &config);

    // Custom main: drive the brightness node, print r_state each sample.
    let n_attack = 1000usize;
    let n_release = 4000usize;
    let main = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    state.set_sample_rate({SR:.1});
    // Attack: full brightness.
    state.brightness = 1.0;
    for _ in 0..{n_attack} {{
        let _ = process_sample(0.0, &mut state);
        println!("{{:.15e}}", state.device_0_state[0]);
    }}
    // Release: dark.
    state.brightness = 0.0;
    for _ in 0..{n_release} {{
        let _ = process_sample(0.0, &mut state);
        println!("{{:.15e}}", state.device_0_state[0]);
    }}
}}
"#
    );
    let out = support::compile_and_run(&code, &main, "ldr_oracle");
    let gen = out.parse_samples();
    assert_eq!(gen.len(), n_attack + n_release, "sample count");

    // Oracle: melange-devices CdsLdr with the same VTL5C3 params + SR.
    let mut ldr = melange_devices::CdsLdr::vtl5c3(SR);
    let mut oracle = Vec::with_capacity(n_attack + n_release);
    for _ in 0..n_attack {
        ldr.update(1.0);
        oracle.push(ldr.resistance());
    }
    for _ in 0..n_release {
        ldr.update(0.0);
        oracle.push(ldr.resistance());
    }

    // Same equation, same dt (OS=1 → dt = 1/SR) → sample-for-sample match.
    // Relative tolerance (resistances span 75 Ω … 10 MΩ).
    let mut max_rel = 0.0f64;
    for (i, (&g, &o)) in gen.iter().zip(oracle.iter()).enumerate() {
        let rel = (g - o).abs() / o.abs().max(1.0);
        if rel > max_rel {
            max_rel = rel;
        }
        assert!(
            rel < 1e-9,
            "sample {i}: generated r_state {g:.6e} vs CdsLdr {o:.6e} (rel {rel:.3e})"
        );
    }

    // Sanity: attack actually pulled the resistance down from the dark seed,
    // release pushed it back up (asymmetric envelope is exercised).
    assert!(gen[0] > gen[n_attack - 1], "attack decreases resistance");
    assert!(
        gen[n_attack + n_release - 1] > gen[n_attack - 1],
        "release increases resistance"
    );
    // Attack (τ=5 ms) is much faster than release (τ=200 ms): after equal
    // sample counts from the same start, attack moves far more.
    eprintln!("LDR oracle max rel err = {max_rel:.3e}");
}
