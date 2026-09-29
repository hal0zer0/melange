//! A runtime matrix rebuild reproduces the baked history matrices.
//!
//! `rebuild_matrices()` runs on every pot move, switch and host-rate change.
//! It must zero the same history rows the IR zeroes in `A_NEG_DEFAULT` /
//! `A_NEG_BE_DEFAULT`: the algebraic augmented rows, but not the
//! parasitic-BJT internal nodes that `expand_bjt_internal_nodes` appends in
//! the same index range. Those are physical G/C nodes. The emitted rebuild
//! used to zero the whole range, so the first pot move on a common-emitter
//! stage with RB/RC/RE dropped the internal nodes' history.

mod support;

use melange_solver::codegen::{CodegenConfig, NodalSubPathOverride};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const CE: &str = "parasitic-RB common emitter\n\
Vcc vcc 0 DC 9\nC_in in b 1u\nR_b1 vcc b 100k\nR_b2 b 0 22k\nQ1 c b e NPN1\n\
R_c vcc c 4.7k\nR_e e 0 1k\nC_e e 0 10u\nC_o c out 1u\nR_l out 0 100k\n\
.model NPN1 NPN(IS=1e-14 BF=200 RB=100 RC=10 RE=1 CJE=10p CJC=5p)\n\
.pot R_c 1k 10k\n";

/// The shipped nodal build, which expands the BJT internal nodes.
fn expanded_code(config: &CodegenConfig) -> String {
    let unexpanded = MnaSystem::from_netlist(&Netlist::parse(CE).expect("parse")).expect("mna");
    let built = support::build_shipped(CE, config, "nodal");
    assert_eq!(
        built.mna.n_aug,
        unexpanded.n_aug + 3,
        "RB, RC and RE each add an internal node"
    );
    built.generated.code
}

const MAIN: &str = "fn main() {
    let mut s = CircuitState::default();
    let (a_neg, a_neg_be) = (s.a_neg, s.a_neg_be);
    s.rebuild_matrices(SAMPLE_RATE * OVERSAMPLING_FACTOR as f64);
    let mut differ = 0usize;
    for i in 0..N {
        for j in 0..N {
            if a_neg[i][j].to_bits() != s.a_neg[i][j].to_bits() { differ += 1; }
            if a_neg_be[i][j].to_bits() != s.a_neg_be[i][j].to_bits() { differ += 1; }
        }
    }
    println!(\"differ={differ}\");
}";

#[test]
fn rebuild_keeps_the_parasitic_bjt_internal_rows() {
    for (sub_path, tag) in [
        (NodalSubPathOverride::Schur, "rebuild_rows_schur"),
        (NodalSubPathOverride::FullLu, "rebuild_rows_full_lu"),
    ] {
        let mut config = support::config_for_spice(CE, 48000.0);
        config.nodal_sub_path_override = sub_path;
        let code = expanded_code(&config);
        let differ = support::compile_and_run(&code, MAIN, tag)
            .parse_kv("differ")
            .unwrap();
        assert_eq!(
            differ, 0.0,
            "{tag}: the rebuild changed {differ} history entries"
        );
    }
}
