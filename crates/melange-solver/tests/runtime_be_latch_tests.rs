//! Runtime BE-latch + `.integrator` directive regression tests.
//!
//! Covers the fix for the trapezoidal Nyquist `(-1)^n` limit-cycle latch that
//! a *large-signal* operating point can reach even when the compile-time
//! quiescent-OP spectral-radius analysis found trap stable (jeffreys-tube V2,
//! reported by oomox 2026-07-28). Two mechanisms:
//!
//!   1. `.integrator {trap|be}` netlist directive — deterministic compile-time
//!      pin so a fleet regen can't silently change the integrator.
//!   2. Runtime BE-latch detector — a lag-1 anti-correlation net on the output
//!      that switches a trapezoidal build to the L-stable BE path for the rest
//!      of the stream when it detects a self-sustaining Nyquist cycle.
//!
//! The end-to-end audio no-latch proof lives in oomox's jeffreys-tube latch
//! corpus (36 corners on speech); these tests pin the codegen surface.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::{IntegratorPref, Netlist};

// A minimal nonlinear circuit driven through the *nodal* codegen path. The
// nodal path is where the runtime BE-latch detector is emitted. `generate_nodal`
// forces the nodal route regardless of the auto-router.
const DIODE_CLIPPER: &str = "\
Diode clipper
Rin in mid 1k
D1 mid out D1N4148
D2 out mid D1N4148
Rload out 0 100k
C1 out 0 10n
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

fn build_mna_with_input(spice: &str, in_name: &str, r_in: f64) -> (Netlist, MnaSystem) {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let in_idx = *mna.node_map.get(in_name).unwrap();
    if in_idx > 0 {
        mna.g[in_idx - 1][in_idx - 1] += 1.0 / r_in;
    }
    (netlist, mna)
}

fn nodal_config(in_name: &str, out_name: &str, mna: &MnaSystem) -> CodegenConfig {
    let in_idx = *mna.node_map.get(in_name).unwrap() - 1;
    let out_idx = *mna.node_map.get(out_name).unwrap() - 1;
    CodegenConfig {
        circuit_name: "test_nodal".to_string(),
        sample_rate: 48000.0,
        input_node: in_idx,
        output_nodes: vec![out_idx],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

fn generate_nodal_with(spice: &str, mut tweak: impl FnMut(&mut CodegenConfig)) -> String {
    let (netlist, mna) = build_mna_with_input(spice, "in", 1.0);
    let mut config = nodal_config("in", "out", &mna);
    tweak(&mut config);
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

// ---------------------------------------------------------------------------
// Runtime BE-latch detector emission
// ---------------------------------------------------------------------------

#[test]
fn nodal_trap_build_emits_be_latch_detector() {
    let code = generate_nodal_with(DIODE_CLIPPER, |_| {});
    // Detector constants + state.
    assert!(
        code.contains("BE_LATCH_R1_ENTER"),
        "trap nodal build must emit the BE-latch entry threshold constant"
    );
    assert!(
        code.contains("pub be_latched: bool"),
        "trap nodal build must carry the be_latched state field"
    );
    assert!(
        code.contains("state.be_r1_num") && code.contains("state.be_pow"),
        "trap nodal build must carry the lag-1 autocorrelation EMAs"
    );
    // The convergence check must be gated so a latch forces the BE path.
    assert!(
        code.contains("&& !state.be_latched"),
        "converged must be gated on !be_latched so a latch forces the BE fallback"
    );
    // The observability counter (oomox ask #3) is always present.
    assert!(
        code.contains("pub diag_be_latch_count: u64"),
        "diag_be_latch_count must be exposed for plugins to surface 'solver degraded'"
    );
}

#[test]
fn force_trap_build_omits_be_latch_detector() {
    // --force-trap opts out of the runtime net entirely (raw trapezoidal).
    let code = generate_nodal_with(DIODE_CLIPPER, |c| c.force_trap = true);
    assert!(
        !code.contains("BE_LATCH_R1_ENTER") && !code.contains("state.be_r1_num"),
        "--force-trap must suppress the runtime BE-latch detector"
    );
    // ...but the diagnostic counter field stays for a stable struct/contract.
    assert!(
        code.contains("pub diag_be_latch_count: u64"),
        "diag_be_latch_count field must exist even when the detector is off"
    );
    assert!(
        !code.contains("&& !state.be_latched"),
        "no be_latched gating when the detector is suppressed"
    );
}

#[test]
fn backward_euler_build_omits_be_latch_detector() {
    // A BE build has nothing to catch — no detector.
    let code = generate_nodal_with(DIODE_CLIPPER, |c| c.backward_euler = true);
    assert!(
        !code.contains("BE_LATCH_R1_ENTER") && !code.contains("state.be_r1_num"),
        "backward-Euler build must not emit the runtime BE-latch detector"
    );
}

// ---------------------------------------------------------------------------
// `.integrator` directive
// ---------------------------------------------------------------------------

#[test]
fn integrator_directive_parses() {
    let be = Netlist::parse("t\nR1 in 0 1k\n.integrator be\n").expect("parse be");
    assert_eq!(be.integrator, Some(IntegratorPref::Be));
    let trap = Netlist::parse("t\nR1 in 0 1k\n.integrator trapezoidal\n").expect("parse trap");
    assert_eq!(trap.integrator, Some(IntegratorPref::Trap));
    let none = Netlist::parse("t\nR1 in 0 1k\n").expect("parse none");
    assert_eq!(none.integrator, None);
}

#[test]
fn integrator_directive_rejects_garbage_and_conflicts() {
    assert!(Netlist::parse("t\nR1 in 0 1k\n.integrator rk4\n").is_err());
    assert!(Netlist::parse("t\nR1 in 0 1k\n.integrator\n").is_err());
    assert!(
        Netlist::parse("t\nR1 in 0 1k\n.integrator be\n.integrator trap\n").is_err(),
        "conflicting trap/be directives must be rejected"
    );
    // Same directive repeated is fine (idempotent).
    assert!(Netlist::parse("t\nR1 in 0 1k\n.integrator be\n.integrator be\n").is_ok());
}

#[test]
fn integrator_be_directive_forces_backward_euler() {
    // `.integrator be` in the netlist ⇒ BE build (no runtime detector, BE alpha).
    let spice = format!("{DIODE_CLIPPER}.integrator be\n");
    let code = generate_nodal_with(&spice, |_| {});
    assert!(
        !code.contains("BE_LATCH_R1_ENTER"),
        ".integrator be must produce a BE build (no runtime detector)"
    );
    // ALPHA = fs (BE) not 2*fs (trap).
    assert!(
        code.contains("48000") && !code.contains("pub const ALPHA: f64 = 96000"),
        ".integrator be must use backward-Euler alpha (fs), not trap (2*fs)"
    );
}

#[test]
fn integrator_trap_directive_suppresses_runtime_latch() {
    // `.integrator trap` pins trapezoidal AND opts out of the runtime net.
    let spice = format!("{DIODE_CLIPPER}.integrator trap\n");
    let code = generate_nodal_with(&spice, |_| {});
    assert!(
        !code.contains("BE_LATCH_R1_ENTER") && !code.contains("state.be_r1_num"),
        ".integrator trap must suppress the runtime BE-latch net (like --force-trap)"
    );
}

#[test]
fn cli_backward_euler_flag_overrides_integrator_trap_directive() {
    // Explicit --backward-euler wins over `.integrator trap`.
    let spice = format!("{DIODE_CLIPPER}.integrator trap\n");
    let code = generate_nodal_with(&spice, |c| c.backward_euler = true);
    assert!(
        !code.contains("BE_LATCH_R1_ENTER"),
        "a BE build (CLI flag overriding .integrator trap) emits no detector"
    );
}
