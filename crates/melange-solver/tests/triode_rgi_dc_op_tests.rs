//! A triode's grid resistance (`RGI`) is in the DC operating point.
//!
//! The transient evaluates the triode at its internal grid, behind RGI
//! (`tube_evaluate_with_rgi`); the DC operating point evaluated it at the
//! terminal. With the grid conducting the two are different circuits, so the
//! transient started away from its own rest point and relaxed toward it. The
//! DC OP now solves the same internal-grid equation
//! (`KorenTriode::evaluate_with_rgi`).
//!
//! Oracles:
//! - RGI is a resistance carrying the grid current into the tube, so a card
//!   with `RGI = R` must reach the operating point of the same triode with
//!   `RGI = 0` behind an explicit series resistor `R` into its grid.
//! - The generated transient, started at its DC operating point and fed
//!   silence, must stay there, on the DK and nodal paths.

mod support;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const CARD: &str = "MU=100 EX=1.4 KG1=1060 KP=600 KVB=300";

/// Grid driven positive through 22k, so it conducts. `Cp` keeps the deck
/// from being capacitor-free (which would auto-insert junction parasitics).
fn deck(rgi_on_card: bool) -> String {
    let (grid, stopper, rgi) = if rgi_on_card {
        ("g", "", " RGI=2k")
    } else {
        ("gi", "RS g gi 2k\n", "")
    };
    format!(
        "triode grid stopper\nRin in 0 1k\nVCC vcc 0 DC 250\nVG gd 0 DC 3\nRG gd g 22k\n\
         {stopper}T1 {grid} p k TX\nRA vcc p 100k\nRK k 0 1k\nCp p 0 1n\n\
         .model TX TRIODE({CARD}{rgi})\n"
    )
}

/// (v(g), v(p), v(k), grid current) at the DC OP.
fn dc(spice: &str) -> [f64; 4] {
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    let dc = solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
    assert!(dc.converged, "{:?}", dc.method);
    let v = |n: &str| dc.v_node[mna.node_map[n] - 1];
    [v("g"), v("p"), v("k"), dc.i_nl[1]]
}

#[test]
fn rgi_is_a_resistance_into_the_grid() {
    let card = dc(&deck(true));
    let explicit = dc(&deck(false));
    // The stopper matters here: 2k carries a grid current of tens of uA.
    assert!(
        card[3] > 1e-5,
        "grid current {:e}: grid not conducting",
        card[3]
    );
    // Without RGI in the DC OP the card's grid sat 76 mV off. The residue,
    // 8e-9 V at the grid and x40 at the plate, is at the scale of the DC
    // solve's node Gmin on the explicit deck's extra node.
    for (k, what) in ["v(g)", "v(p)", "v(k)", "Ig"].iter().enumerate() {
        let tol = if k == 3 {
            1e-6 * explicit[3].abs()
        } else {
            1e-6
        };
        assert!(
            (card[k] - explicit[k]).abs() <= tol,
            "{what}: RGI on the card {:.12e}, explicit resistor {:.12e}",
            card[k],
            explicit[k]
        );
    }
    // And it is not the terminal evaluation: without RGI the plate moves.
    let bare = dc(&deck(true).replace(" RGI=2k", ""));
    assert!(
        (bare[1] - card[1]).abs() > 1e-3,
        "v(p) without RGI {:.6} vs with {:.6}: the witness does not see RGI",
        bare[1],
        card[1]
    );
}

#[test]
fn the_transient_rests_at_its_dc_operating_point() {
    const FS: f64 = 48000.0;
    let spice = deck(true);
    let mut config = support::config_for_spice(&spice, FS);
    config.output_nodes = vec![support::node_index(&spice, "p")];
    config.dc_block = false;
    let g = support::node_index(&spice, "g");
    let p = support::node_index(&spice, "p");
    for (path, code) in [
        ("dk", support::generate_circuit_code(&spice, &config).0),
        (
            "nodal",
            support::generate_circuit_code_nodal(&spice, &config).0,
        ),
    ] {
        let bad = support::unsolved_expr(&code, "s");
        let main = format!(
            "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    for _ in 0..4800 {{ let _ = process_sample(0.0, &mut s); }}
    println!(\"{{:e}} {{:e}} {{:e}} {{:e}} {{}}\", s.v_prev[{g}], DC_OP[{g}], s.v_prev[{p}], DC_OP[{p}], {bad});
}}"
        );
        let out = support::compile_and_run(&code, &main, &format!("triode_rgi_{path}")).stdout;
        let v: Vec<f64> = out.split_whitespace().map(|t| t.parse().unwrap()).collect();
        assert_eq!(v[4], 0.0, "{path}: unsolved samples");
        // Without RGI in the DC OP it drifted 76 mV (grid) and 0.70 V
        // (plate). What remains is the DC OP's node Gmin (STATUS "Node Gmin
        // moves the fixed point"), the same on this deck without RGI: 8e-7 V
        // at the grid, 1e-4 V at the 100k plate.
        assert!(
            (v[0] - v[1]).abs() <= 1e-5,
            "{path}: v(g) settles at {:.12e}, DC OP {:.12e}",
            v[0],
            v[1]
        );
        assert!(
            (v[2] - v[3]).abs() <= 1e-3,
            "{path}: v(p) settles at {:.12e}, DC OP {:.12e}",
            v[2],
            v[3]
        );
    }
}
