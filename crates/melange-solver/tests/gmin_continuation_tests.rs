//! Gmin-continuation regression tests — per-sample Gmin-stepping homotopy for
//! stiff hard-switching NR failures on the full-LU nodal path.
//!
//! When trapezoidal Newton fails at a switching transition (a positive-feedback
//! junction pinned deep into saturation leaves the Jacobian ill-conditioned and
//! the residual stuck flat), the solve retries with a Gmin homotopy — add `gmin`
//! to the node diagonals, ramp 1e-2 → 1e-12 warm-starting each level, and accept
//! ONLY the final gmin≈0 level (an intermediate level holds a regularized,
//! non-physical solution — accepting one yields a railed result). Opt-in
//! (`--gmin-continuation`), gated off under backward-Euler (which is L-stable and
//! needs no continuation).
//!
//! These pin the codegen surface (flag plumbing + gating + schedule). The
//! full-LU-path wrapper itself and its runtime effect are validated by a
//! compile-and-run on farfisa-g10-ref (192k, --force-trap): NR max-iter
//! starvation 2.268% → 0.005%, BE fallbacks 716 → 2, output physical (term_1d
//! ~4 Vpp, not railed). A minimal circuit cannot be forced onto the full-LU
//! path portably (that route is K-conditioning driven), so the wrapper's own
//! text is asserted there rather than here.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// A nonlinear (m>0) circuit on the nodal trap path.
const DIODE_CLIPPER: &str = "\
Diode clipper
Rin in mid 1k
D1 mid out D1N4148
D2 out mid D1N4148
Rload out 0 100k
C1 out 0 10n
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

fn generate_nodal(spice: &str, mut tweak: impl FnMut(&mut CodegenConfig)) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let in_idx = *mna.node_map.get("in").unwrap();
    if in_idx > 0 {
        mna.g[in_idx - 1][in_idx - 1] += 1.0;
    }
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut config = CodegenConfig {
        circuit_name: "gmin_test".to_string(),
        sample_rate: 48000.0,
        input_node: in_idx - 1,
        output_nodes: vec![out_idx],
        input_resistance: 1.0,
        // Pin trapezoidal so the auto-detector doesn't promote to backward Euler
        // (which would correctly gate Gmin continuation off).
        force_trap: true,
        ..CodegenConfig::default()
    };
    tweak(&mut config);
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

#[test]
fn gmin_continuation_flag_emits_schedule() {
    let code = generate_nodal(DIODE_CLIPPER, |c| c.gmin_continuation = true);
    assert!(
        code.contains("GMIN_CONT_SCHEDULE: [f64; 12]"),
        "--gmin-continuation must emit the homotopy schedule constant"
    );
    // Schedule starts at the ordinary 1e-12 (level 0, byte-identical fast path)
    // then ramps 1e-2 → 1e-12 for the escalated levels.
    assert!(
        code.contains("1e-12, 1e-2, 1e-3, 1e-4"),
        "schedule must start at 1e-12 (level 0) then ramp from 1e-2 downward"
    );
    assert!(
        code.contains("pub const GMIN_CONT_LEVELS: usize = GMIN_CONT_SCHEDULE.len();"),
        "must expose GMIN_CONT_LEVELS"
    );
}

#[test]
fn default_build_omits_gmin_continuation() {
    let code = generate_nodal(DIODE_CLIPPER, |_| {});
    assert!(
        !code.contains("GMIN_CONT_SCHEDULE")
            && !code.contains("gmin_level")
            && !code.contains("nr_gmin"),
        "without the flag no Gmin-continuation code may be emitted (byte-inert default)"
    );
}

#[test]
fn backward_euler_build_omits_gmin_continuation() {
    // BE is L-stable — the stuck-junction ill-conditioning does not arise, so the
    // continuation is gated off even when the flag is set.
    let code = generate_nodal(DIODE_CLIPPER, |c| {
        c.gmin_continuation = true;
        c.force_trap = false;
        c.backward_euler = true;
    });
    assert!(
        !code.contains("GMIN_CONT_SCHEDULE"),
        "a backward-Euler build must omit Gmin continuation even with the flag set"
    );
}
