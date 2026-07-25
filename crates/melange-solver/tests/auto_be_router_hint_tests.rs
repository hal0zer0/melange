//! Regression test for the auto-BE "router hint" fix (2026-07-25).
//!
//! When `routing::auto_route` selects the nodal path BECAUSE the DK-kernel
//! spectral radius exceeded the trap-instability threshold, but the nodal
//! path's own input-node-deflated estimator lands below that threshold (the
//! two estimators straddle 1.002 — observed on `wurli-power-amp`: DK-kernel
//! rho = 1.0040 vs nodal-deflated rho = 1.0005), the nodal auto-BE promotion
//! must honor the router's finding and enable backward Euler. Before the fix
//! it silently ran the trapezoidal integrator the router had just declared
//! unstable, and `wurli-power-amp` timed out (>100 s for 0.5 s of audio).
//!
//! The fixture below is a small nonlinear (diode) circuit that is trap-STABLE
//! on the nodal path — its nodal-local rho is below threshold, so the local
//! discriminator never promotes on its own. That isolates the router-hint
//! branch: `router_dk_unstable` is the ONLY thing that can flip it to BE.
//!
//! Note: the router's raw signal is NOT trusted unconditionally — see
//! `codegen::stability::router_corroborates_marginal_instability` and its
//! unit tests for the narrower "must be corroborated by a locally-marginal
//! estimate" gate that was added after this fixture's rho was confirmed to
//! sit in the marginal window (this fixture exercises the same regime as
//! wurli-power-amp, not the tungsten-thunder-horse false-positive case).

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const TRAP_STABLE_NONLINEAR: &str = "\
Diode RC (trap-stable nonlinear)
Rin in n1 1k
D1 n1 0 DMOD
C1 n1 0 10n
Rout n1 out 1k
Rload out 0 100k
.model DMOD D(IS=1e-14 N=1.0)
";

/// Generate nodal code for the fixture and return `(m > 0, backward_euler_auto)`.
fn generate(router_dk_unstable: bool) -> (bool, bool) {
    let netlist = Netlist::parse(TRAP_STABLE_NONLINEAR).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    // Mirror the CLI: stamp the 1 Ω input Thevenin conductance.
    mna.g[input_node][input_node] += 1.0;
    let cfg = CodegenConfig {
        circuit_name: "auto_be_router_hint".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![output_node],
        output_scales: vec![1.0],
        input_resistance: 1.0,
        router_dk_unstable,
        router_dk_spectral_radius: if router_dk_unstable { 1.0040 } else { 0.0 },
        ..CodegenConfig::default()
    };
    let g = CodeGenerator::new(cfg)
        .generate_nodal(&mna, &netlist)
        .expect("codegen");
    (g.m > 0, g.meta.backward_euler_auto)
}

/// Control: with the hint OFF this trap-stable nonlinear circuit must NOT
/// auto-promote to BE. If this ever fails, the fixture stopped being
/// trap-stable and the router-hint test below would be meaningless.
#[test]
fn baseline_no_router_hint_stays_trapezoidal() {
    let (nonlinear, be_auto) = generate(false);
    assert!(
        nonlinear,
        "fixture must be nonlinear (m>0) or the auto-BE gate is skipped entirely"
    );
    assert!(
        !be_auto,
        "control fixture must be trap-stable (nodal-local rho below threshold) so the \
         router hint is the ONLY promotion trigger in the test below"
    );
}

/// The regression: identical circuit, but the router reports trap-unstable via
/// the un-reduced DK-kernel spectral radius. The nodal auto-BE gate must honor
/// it and promote to backward Euler even though the nodal-local deflated
/// estimate is below threshold. Pre-fix, the hint did not exist / was ignored
/// and this stayed trapezoidal — the exact wurli-power-amp failure.
#[test]
fn router_dk_unstable_hint_forces_backward_euler() {
    let (_, be_auto) = generate(true);
    assert!(
        be_auto,
        "router_dk_unstable must force backward-Euler promotion on the nodal path, \
         honoring routing::auto_route's DK-kernel trap-instability finding instead of \
         letting the nodal-local deflated recompute veto it back down to trapezoidal"
    );
}
