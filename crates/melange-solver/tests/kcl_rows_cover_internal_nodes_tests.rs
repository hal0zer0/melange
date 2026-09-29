//! Every Newton acceptance on the nodal route checks the parasitic-BJT
//! internal nodes an expanded build adds.
//!
//! `expand_bjt_internal_nodes` appends a BJT's RB/RC/RE nodes among the
//! augmented rows, past the circuit nodes. Those rows are where the junction
//! currents enter. The generated KCL residual (`kcl_residual`, the main
//! loop's and the sub-step ladder's acceptance gate) and the sub-step
//! ladder's step test covered the circuit nodes `0..N_NODES` only, so a
//! sub-step could be accepted with an internal node unconverged and its KCL
//! unbalanced. Both now cover every KCL row: the circuit nodes and the
//! internal nodes.

mod support;

use melange_solver::codegen::NodalSubPathOverride;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// A common-emitter stage whose shipped nodal build expands RB, RC and RE.
const CE: &str = "parasitic-RB common emitter\n\
Vcc vcc 0 DC 9\nC_in in b 1u\nR_b1 vcc b 100k\nR_b2 b 0 22k\nQ1 c b e NPN1\n\
R_c vcc c 4.7k\nR_e e 0 1k\nC_e e 0 10u\nC_o c out 1u\nR_l out 0 100k\n\
.model NPN1 NPN(IS=1e-14 BF=200 RB=100 RC=10 RE=1 CJE=10p CJC=5p)\n";

#[test]
fn the_kcl_residual_and_the_ladder_step_test_cover_the_internal_nodes() {
    let unexpanded = MnaSystem::from_netlist(&Netlist::parse(CE).expect("parse")).expect("mna");
    for sub_path in [NodalSubPathOverride::Schur, NodalSubPathOverride::FullLu] {
        let mut config = support::config_for_spice(CE, 48000.0);
        config.nodal_sub_path_override = sub_path;
        let built = support::build_shipped(CE, &config, "nodal");
        let internal: Vec<usize> = (unexpanded.n_aug..built.mna.n_aug).collect();
        assert_eq!(internal.len(), 3, "RB, RC and RE each add an internal node");
        let code = &built.generated.code;
        let residual = &code[code.find("fn kcl_residual(").expect("kcl_residual")..];
        let residual = &residual[..residual.find("\n}\n").expect("end of kcl_residual")];
        for &row in &internal {
            assert!(
                residual.contains(&format!("let mut acc = -rhs[{row}] - q[{row}];")),
                "{sub_path:?}: kcl_residual does not check internal-node row {row}"
            );
        }
        let ladder_step = code
            .lines()
            .find(|l| l.contains("let step = v_new_s[i] - v_sub[i];"))
            .expect("ladder step test");
        for &row in &internal {
            assert!(
                ladder_step.contains(&format!(" {row},"))
                    || ladder_step.contains(&format!(" {row}]")),
                "{sub_path:?}: the ladder step test skips internal-node row {row}: {ladder_step}"
            );
        }
    }
}
