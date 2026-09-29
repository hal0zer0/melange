//! `--opamp-rail-mode boyle-diodes` builds the circuit every other mode builds,
//! plus the catch diodes.
//!
//! The catch diodes used to be added inside nodal codegen, which rebuilt the
//! MNA from the augmented netlist and restamped only the input ports and the
//! junction caps: an `.inject` conductance and a `.linearize` reduction were
//! silently dropped. The build now adds the diodes to the netlist before the
//! MNA is assembled, so every step sees them once. Each witness compares the
//! shipped boyle-diodes build with the shipped hard-rail build of the same deck
//! on the quantity the rebuild used to lose, read from the emitted code.

mod support;

use melange_solver::codegen::OpampRailMode;

/// Op-amp inverting stage with an `.inject` Thevenin source at `fb`.
const INJECT_DECK: &str = "\
Boyle inject witness
R1 in inv 10k
R2 inv out 100k
U1 0 inv out oa
Rl out 0 10k
Rx out fb 1k
Rf fb 0 10k
.model oa OA(AOL=200000 VCC=9 VEE=-9)
.inject fb fbv R=1k
";

/// Op-amp stage driving a `.linearize`d BJT common-emitter stage.
const LINEARIZE_DECK: &str = "\
Boyle linearize witness
Vcc vcc 0 DC 12
R1 in inv 10k
R2 inv oa_out 100k
U1 0 inv oa_out oa
Cc oa_out b 10u
Rb1 vcc b 100k
Rb2 b 0 22k
Q1 c b e NPN1
Rc vcc c 4.7k
Re e 0 1k
Co c out 10u
Rl out 0 100k
.model oa OA(AOL=200000 VCC=9 VEE=-9)
.model NPN1 NPN(IS=1e-14 BF=200)
.linearize Q1
";

/// Catch diodes per op-amp with both rails finite.
const CATCH_DIODES_PER_OPAMP: usize = 2;

fn built(spice: &str, mode: OpampRailMode) -> melange_solver::build::Built {
    let mut config = support::config_for_spice(spice, 48000.0);
    config.opamp_rail_mode = mode;
    support::build_shipped(spice, &config, "nodal")
}

/// `G[node][node]` of the emitted `const G`, `node` looked up in `NODE_NAMES`.
fn emitted_g_diag(code: &str, node: &str) -> f64 {
    let names_line = code
        .lines()
        .find(|l| l.starts_with("pub const NODE_NAMES: [&str; N] = ["))
        .expect("NODE_NAMES");
    let names: Vec<&str> = names_line
        .split('[')
        .nth(2)
        .expect("names")
        .trim_end_matches("];")
        .split(", ")
        .map(|s| s.trim_matches('"'))
        .collect();
    let i = names
        .iter()
        .position(|&n| n == node)
        .expect("node in NODE_NAMES");
    let start = code.find("const G: [[f64; N]; N] = [").expect("const G");
    let body = &code[start..];
    let body = &body[body.find('[').unwrap() + 1..];
    let body = &body[body.find('[').unwrap() + 1..];
    let body = &body[body.find('[').unwrap()..];
    let row = body.split("],").nth(i).expect("G row");
    row.trim()
        .trim_start_matches('[')
        .split(',')
        .nth(i)
        .expect("G entry")
        .trim()
        .trim_end_matches(']')
        .parse()
        .expect("G entry parses")
}

#[test]
fn boyle_diodes_keep_the_inject_conductance() {
    let hard = built(INJECT_DECK, OpampRailMode::Hard);
    let boyle = built(INJECT_DECK, OpampRailMode::BoyleDiodes);
    let g_hard = emitted_g_diag(&hard.generated.code, "fb");
    let g_boyle = emitted_g_diag(&boyle.generated.code, "fb");
    // 1/1k (the .inject) + 1/1k (Rx) + 1/10k (Rf).
    assert!(
        (g_hard - 2.1e-3).abs() < 1e-8,
        "hard-rail G[fb][fb] = {g_hard}"
    );
    assert!(
        (g_boyle - g_hard).abs() < 1e-12,
        "boyle-diodes G[fb][fb] = {g_boyle}, hard-rail {g_hard}: the .inject stamp must survive"
    );
}

#[test]
fn boyle_diodes_keep_the_linearize_reduction() {
    let hard = built(LINEARIZE_DECK, OpampRailMode::Hard);
    let boyle = built(LINEARIZE_DECK, OpampRailMode::BoyleDiodes);
    assert_eq!(hard.generated.m, 0, "Q1 is linearized: no nonlinear dims");
    assert_eq!(
        boyle.generated.m - CATCH_DIODES_PER_OPAMP,
        hard.generated.m,
        "boyle-diodes M = {} (catch diodes + ?): the .linearize reduction must survive",
        boyle.generated.m
    );
}

#[test]
fn boyle_diodes_route_nodal_and_refuse_forced_dk() {
    let mut config = support::config_for_spice(INJECT_DECK, 48000.0);
    config.opamp_rail_mode = OpampRailMode::BoyleDiodes;
    let auto = support::build_shipped(INJECT_DECK, &config, "auto");
    assert_eq!(auto.solver_label, "nodal", "{}", auto.solver_reason);
    let err = support::try_build_shipped(INJECT_DECK, &config, "dk")
        .err()
        .expect("--solver dk with boyle-diodes must be refused");
    assert!(err.contains("boyle-diodes"), "{err}");
}
