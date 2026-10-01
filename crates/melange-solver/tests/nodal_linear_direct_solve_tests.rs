//! The nodal full-LU M=0 path: a linear circuit solved by one direct LU per
//! sample (`// Linear circuit: direct LU solve`), with no Newton iteration.
//!
//! No corpus deck takes this path, so it is covered here. The deck is the case
//! a node Gmin matters for: a node held only by 1 GΩ resistors, AC-coupled to
//! the input. Its DC value depends on the node Gmin — and on nothing else that
//! a correct solver may change.

mod support;

use melange_solver::codegen::{NodalSubPath, NodalSubPathOverride};

const SR: f64 = 48000.0;

/// The node Gmin baked into the nodal `G` (`GMIN_REGULARISATION`, 1 TΩ from
/// every circuit node to ground). The compile-time DC operating point carries
/// the same one, so it is part of the circuit the transient must settle on.
const NODE_GMIN: f64 = 1e-12;

/// `out` is a 1 GΩ / 1 GΩ divider of a 9 V rail, coupled to the input through
/// 10 pF. tau = 10 pF * 0.5 GΩ = 5 ms.
const HIGH_Z_DIVIDER: &str = "\
high-impedance node behind a coupling cap
Vcc vcc 0 DC 9
R1 vcc out 1G
R2 out 0 1G
C1 in out 10p
R3 in 0 10k
";

#[test]
fn full_lu_m0_direct_solve_settles_on_the_analytic_dc() {
    let mut config = support::config_for_spice(HIGH_Z_DIVIDER, SR);
    config.dc_block = false;
    config.nodal_sub_path_override = NodalSubPathOverride::FullLu;

    // The route: nodal, full-LU, M=0, the direct-solve branch.
    let built = support::build_shipped(HIGH_Z_DIVIDER, &config, "nodal");
    assert_eq!(built.solver_label, "nodal");
    assert_eq!(built.generated.m, 0, "the deck must be linear (M=0)");
    assert_eq!(
        built.generated.meta.nodal_sub_path,
        Some(NodalSubPath::FullLu),
        "the deck must build on the full-LU sub-path"
    );
    assert!(
        built
            .generated
            .code
            .contains("// Linear circuit: direct LU solve"),
        "the build must take the M=0 direct-solve branch"
    );

    let circuit = support::build_circuit_nodal(HIGH_Z_DIVIDER, &config, "m0_high_z");
    assert_eq!(circuit.code, built.generated.code);

    // KCL at `out` at DC (C1 open): (9 - v)/R - v/R - NODE_GMIN*v = 0.
    let g = 1.0 / 1e9;
    let analytic = 9.0 * g / (2.0 * g + NODE_GMIN);

    // A 1 V step on the input kicks `out` through C1; 40 tau later it has
    // decayed back to the DC value (e^-40 ~ 4e-18).
    let n = (0.2 * SR) as usize;
    let out = support::run_step(&circuit, 1.0, n, SR);
    support::assert_finite(&out);
    assert_eq!(out.len(), n);
    let kick = out.iter().map(|v| (v - analytic).abs()).fold(0.0, f64::max);
    assert!(
        kick > 0.1,
        "the step must move `out` (peak deviation {kick:.3e} V), or the run proves nothing"
    );
    let settled = out[n - 1];
    // A second node Gmin on the transient's matrix (as the direct solve once
    // added) settles at 9G/(2G + 2*NODE_GMIN): 2.2 mV (5e-4 relative) low.
    // The bound sits five orders below that.
    let rel = ((settled - analytic) / analytic).abs();
    assert!(
        rel <= 1e-9,
        "settled v(out) {settled:.12} V vs analytic {analytic:.12} V ({rel:.3e} relative)"
    );
}
