//! Regression tests for the 2026-07-18 DK-emitter bug fixes:
//!
//! 1. `CodegenConfig::output_clamp_v` threaded into the DK process_sample
//!    template (was hardcoded ±10 V in the clamp, the diag threshold, and
//!    the NaN-recovery path — the Wurlitzer power amp at ±22 V rails needs
//!    ±30 V; the nodal emitter already honored it).
//! 2. Op-amp slew-rate limiting reads the runtime sample rate
//!    (`state.current_sample_rate`) instead of the baked `SAMPLE_RATE`
//!    const — a 96 kHz host on a 44.1k-compiled circuit got ~2.2× loose
//!    slew limiting. Mirrors the nodal fix (commit 7e32bf8).
//! 3. `--opamp-rail-mode` consumption on the DK path: `none` suppresses the
//!    clamp entirely; `active-set`/`active-set-be` degrade to Hard (+ the
//!    DK BE fallback) with a one-shot construction-time warning.
//! 4. `dc_block_x_prev` seeded from the output-node DC operating point in
//!    `Default`, `reset()`, and `set_sample_rate()` — a zero seed while
//!    `v_prev = DC_OP` produced a deterministic startup thump of
//!    ≈ DC_OP[out]·SCALE decaying at the 5 Hz blocker pole (τ ≈ 32 ms).

mod support;

use melange_solver::codegen::{CodegenConfig, OpampRailMode};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn parse_var(stdout: &str, key: &str) -> f64 {
    let prefix = format!("VAR:{key}=");
    stdout
        .lines()
        .find_map(|l| l.strip_prefix(&prefix))
        .unwrap_or_else(|| panic!("missing '{key}' in stdout:\n{stdout}"))
        .trim()
        .parse::<f64>()
        .unwrap_or_else(|e| panic!("parse '{key}': {e}"))
}

/// Node indices (0-based, codegen convention) for named nodes.
fn node_idx(spice: &str, name: &str) -> usize {
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA build failed");
    mna.node_map[name] - 1
}

// ======================================================================
// Fix 1: output_clamp_v threading
// ======================================================================

/// Wideband RC (fc ≈ 15.9 kHz) — a 100 Hz sine passes essentially
/// unattenuated, so a ±20 V input produces a ±20 V output that the old
/// hardcoded ±10 V clamp would flat-top.
const WIDEBAND_RC_SPICE: &str = "* wideband RC — output clamp threading\n\
R1 in out 1k\n\
C1 out 0 10n\n";

#[test]
fn dk_output_clamp_honors_output_clamp_v() {
    let config = CodegenConfig {
        circuit_name: "clamp30".to_string(),
        sample_rate: 44100.0,
        input_node: node_idx(WIDEBAND_RC_SPICE, "in"),
        output_nodes: vec![node_idx(WIDEBAND_RC_SPICE, "out")],
        dc_block: true,
        output_clamp_v: 30.0,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(WIDEBAND_RC_SPICE, &config);

    // Structural: the configured bound (30 → `3e1` via `{:e}`) is emitted;
    // the old hardcoded ±10 V clamp form is gone (the ±100 V input/DC-block
    // guards are unaffected — different literal).
    assert!(
        code.contains("clamp(-3e1, 3e1)"),
        "DK template must emit the configured output clamp (±30 V)"
    );
    assert!(
        code.contains("abs_out > 3e1"),
        "diag_clamp_count threshold must track output_clamp_v"
    );
    assert!(
        !code.contains("clamp(-10.0, 10.0)") && !code.contains("clamp(-1e1, 1e1)"),
        "no hardcoded ±10 V clamp may survive when output_clamp_v = 30"
    );

    // Behavioral: a >10 V sine passes unclipped through the ±30 V clamp.
    // 100 Hz at 44.1 kHz, amplitude 20 V; RC fc = 15.9 kHz and the 5 Hz
    // DC blocker are both transparent at 100 Hz.
    let main = r#"
fn main() {
    let mut state = CircuitState::default();
    let sr = 44100.0_f64;
    let mut peak = 0.0_f64;
    for i in 0..4410 {
        let x = 20.0 * (2.0 * std::f64::consts::PI * 100.0 * i as f64 / sr).sin();
        let v = process_sample(x, &mut state)[0].abs();
        if v > peak { peak = v; }
    }
    println!("VAR:peak={:.9e}", peak);
    println!("VAR:clamped={}", state.diag_clamp_count);
}
"#;
    let out = support::compile_and_run(&code, main, "dk_clamp30");
    let peak = parse_var(&out.stdout, "peak");
    assert!(
        (15.0..=25.0).contains(&peak),
        "±20 V sine should pass the ±30 V clamp unclipped (peak = {peak:.3} V; \
         ≈10.0 means the hardcoded clamp is back)"
    );
    let clamped = parse_var(&out.stdout, "clamped");
    assert_eq!(
        clamped, 0.0,
        "diag_clamp_count must stay 0 below the configured ±30 V bound"
    );
}

// ======================================================================
// Fixes 2 + 3: op-amp slew dt (runtime rate) and rail-mode consumption
// ======================================================================

/// DK-Schur op-amp + diode circuit (M=1) from opamp_tests — known to route
/// through the DK path. SR=0.01 V/µs = 1e4 V/s is deliberately slow so the
/// slew limiter binds on a step: max per-sample delta = 1e4 / sr_host.
const OPAMP_SLEW_SPICE: &str = r#"SR runtime-rate regression
R1 in inv 10k
R2 inv opout 100k
C1 opout 0 100n
U1 0 inv opout oa
Rcouple opout out 1k
D1 out 0 D1N4148
Rload out 0 10k
C2 out 0 100n
.model oa OA(AOL=200000 ROUT=75 GBW=3Meg VSAT=13 SR=0.01)
.model D1N4148 D(IS=2.52e-9 N=1.752)
"#;

fn opamp_slew_config(rail_mode: OpampRailMode) -> CodegenConfig {
    CodegenConfig {
        circuit_name: "opamp_slew_dt".to_string(),
        sample_rate: 44100.0,
        input_node: node_idx(OPAMP_SLEW_SPICE, "in"),
        // Observe the op-amp output node directly — the slew clamp applies
        // there, so the per-sample output delta IS the slew limit.
        output_nodes: vec![node_idx(OPAMP_SLEW_SPICE, "opout")],
        dc_block: false,
        opamp_rail_mode: rail_mode,
        ..CodegenConfig::default()
    }
}

#[test]
fn dk_opamp_slew_reads_runtime_sample_rate() {
    let config = opamp_slew_config(OpampRailMode::Auto);
    let (code, _n, _m) = support::generate_circuit_code(OPAMP_SLEW_SPICE, &config);

    // Structural: dt derives from the runtime rate, and the state struct
    // carries current_sample_rate even though this circuit has no pots or
    // switches (the pre-fix gate omitted the field → E0609 or baked dt).
    assert!(
        code.contains("state.current_sample_rate * OVERSAMPLING_FACTOR"),
        "slew dt must read state.current_sample_rate, not the SAMPLE_RATE const"
    );
    assert!(
        code.contains("pub current_sample_rate"),
        "state struct must carry current_sample_rate for the slew dt"
    );

    // Behavioral: drive a large step; the slew limiter bounds the
    // per-sample output delta to SR_vps/sr_host, so doubling the host rate
    // must halve the observed max delta. Pre-fix, dt was baked at the
    // compile-time 44.1 kHz and the ratio came out 1.0.
    let main = r#"
fn max_step_delta(sr_host: f64) -> f64 {
    let mut state = CircuitState::default();
    state.set_sample_rate(sr_host);
    let mut prev = 0.0_f64;
    let mut max_d = 0.0_f64;
    for _ in 0..64 {
        let v = process_sample(5.0, &mut state)[0];
        let d = (v - prev).abs();
        if d > max_d { max_d = d; }
        prev = v;
    }
    max_d
}

fn main() {
    println!("VAR:d44={:.9e}", max_step_delta(44100.0));
    println!("VAR:d88={:.9e}", max_step_delta(88200.0));
}
"#;
    let out = support::compile_and_run(&code, main, "dk_slew_runtime_rate");
    let d44 = parse_var(&out.stdout, "d44");
    let d88 = parse_var(&out.stdout, "d88");

    // SR = 0.01 V/µs = 1e4 V/s → max_dv = 1e4/44100 ≈ 0.2268 V at 44.1k.
    let expected_44 = 1.0e4 / 44100.0;
    assert!(
        (d44 / expected_44 - 1.0).abs() < 0.10,
        "44.1 kHz slew-limited step should be ≈ {expected_44:.4} V/sample, got {d44:.4}"
    );
    let ratio = d88 / d44;
    assert!(
        (ratio - 0.5).abs() < 0.05,
        "doubling the host rate must halve the slew-limited per-sample delta \
         (d88/d44 = {ratio:.3}; 1.0 means dt is still baked at the compile-time rate)"
    );
}

#[test]
fn dk_rail_mode_none_suppresses_opamp_clamp() {
    let config = opamp_slew_config(OpampRailMode::None);
    let (code, _n, _m) = support::generate_circuit_code(OPAMP_SLEW_SPICE, &config);

    assert!(
        !code.contains("Op-amp output voltage clamping")
            && !code.contains("Op-amp supply rail clamping (BE path)"),
        "--opamp-rail-mode none must emit NO rail clamp on the DK path"
    );
    assert!(
        !code.contains("degrades to Hard"),
        "mode 'none' must not emit the degrade warning"
    );
    // Slew limiting is orthogonal to rail clamping and must survive.
    assert!(
        code.contains("OA0_SR * _oa_slew_dt"),
        "slew limiter must still be emitted under rail mode 'none'"
    );
}

#[test]
fn dk_rail_mode_active_set_be_degrades_to_hard_with_warning() {
    let config = opamp_slew_config(OpampRailMode::ActiveSetBe);
    let (code, _n, _m) = support::generate_circuit_code(OPAMP_SLEW_SPICE, &config);

    assert!(
        code.contains("Op-amp output voltage clamping"),
        "active-set-be on the DK path must still emit the Hard clamp"
    );
    assert!(
        code.contains(
            "opamp rail mode 'active-set-be' degrades to Hard+BE-fallback on the DK path"
        ),
        "active-set-be must emit the one-shot degrade warning at construction"
    );
    assert!(
        code.contains("OPAMP_RAIL_DEGRADE_WARN_ONCE"),
        "degrade warning must be latched through std::sync::Once (once per process)"
    );

    // Hard mode keeps the clamp and stays silent.
    let config_hard = opamp_slew_config(OpampRailMode::Hard);
    let (code_hard, _n, _m) = support::generate_circuit_code(OPAMP_SLEW_SPICE, &config_hard);
    assert!(
        code_hard.contains("Op-amp output voltage clamping")
            && !code_hard.contains("degrades to Hard"),
        "Hard mode must emit the clamp with no degrade warning"
    );
}

// ======================================================================
// Fix 4: dc_block_x_prev seeded from the output-node DC OP
// ======================================================================

/// DC divider biased at ≈ 4.5 V on the output node. Before the fix, the
/// first samples after `Default` carried a ≈ 4.5 V transient decaying at
/// the 5 Hz DC-blocker pole (τ ≈ 32 ms) — a deterministic startup thump.
const DC_DIVIDER_SPICE: &str = "* DC divider — startup thump regression\n\
VCC vcc 0 DC 9\n\
R1 vcc out 10k\n\
R2 out 0 10k\n\
R3 in out 1meg\n\
C1 out 0 100n\n";

#[test]
fn dk_dc_block_seeded_from_dc_op_no_startup_thump() {
    let config = CodegenConfig {
        circuit_name: "dc_thump".to_string(),
        sample_rate: 44100.0,
        input_node: node_idx(DC_DIVIDER_SPICE, "in"),
        output_nodes: vec![node_idx(DC_DIVIDER_SPICE, "out")],
        dc_block: true,
        ..CodegenConfig::default()
    };
    let (code, _n, _m) = support::generate_circuit_code(DC_DIVIDER_SPICE, &config);

    // Structural: Default seeds from the baked DC_OP; reset()/set_sample_rate
    // seed from the (possibly recomputed) runtime dc_operating_point.
    assert!(
        code.contains("dc_block_x_seed"),
        "Default must seed dc_block_x_prev from DC_OP[OUTPUT_NODES[k]]"
    );
    assert!(
        code.contains("self.dc_block_x_prev[k] = self.dc_operating_point[OUTPUT_NODES[k]]"),
        "reset()/set_sample_rate must reseed dc_block_x_prev from dc_operating_point"
    );

    // Behavioral: with v_prev = DC_OP and x_prev seeded to the same value,
    // the blocker starts at its steady state — first-sample output must be
    // ≈ 0 instead of ≈ 4.5 V. Bound 0.05 V: the old thump is ~90× larger;
    // the residue is numerical (µV scale).
    let main = r#"
fn main() {
    let mut state = CircuitState::default();
    let first = process_sample(0.0, &mut state)[0].abs();
    let mut peak = first;
    for _ in 0..199 {
        let v = process_sample(0.0, &mut state)[0].abs();
        if v > peak { peak = v; }
    }
    println!("VAR:first={:.9e}", first);
    println!("VAR:peak={:.9e}", peak);

    state.reset();
    let first_reset = process_sample(0.0, &mut state)[0].abs();
    println!("VAR:first_reset={:.9e}", first_reset);

    state.set_sample_rate(88200.0);
    let first_ssr = process_sample(0.0, &mut state)[0].abs();
    println!("VAR:first_ssr={:.9e}", first_ssr);
}
"#;
    let out = support::compile_and_run(&code, main, "dk_dc_block_seed");
    for key in ["first", "peak", "first_reset", "first_ssr"] {
        let v = parse_var(&out.stdout, key);
        assert!(
            v < 0.05,
            "startup thump regression: |output| after {key} = {v:.4} V \
             (old bug: ≈ 4.5 V = DC_OP[out] stepping into a zero-seeded DC blocker)"
        );
    }
}
