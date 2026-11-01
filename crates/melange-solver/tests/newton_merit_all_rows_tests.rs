//! The nodal Newton's residual helpers (`kcl_residual` / `kcl_residual_inl`,
//! the convergence gate and the Armijo line search's merit) cover every row
//! the Newton step solves, each with its true residual: KCL rows in amps,
//! source / constraint / inductor branch rows in their own unit, behavioral
//! `V={}` rows as `(v+ − v−) − f(v)`, saturating-inductor rows with the flux
//! residual. A merit over the KCL rows alone could not see a residual carried
//! by a source or branch row, so the line search backtracked a correct step
//! to `1/32..1/1024` on every iteration; a gate over the KCL rows alone could
//! certify a solve after a damped step with such a row still unmet.
//!
//! Witnesses (both reported against v0.1.19): a rectifier supply whose
//! behavioral mains source steps 0.19 V per sample into an empty reservoir
//! held 31338 of 48000 samples at 48 kHz, every one traced to a single
//! warm-up sample that needed 250 Newton iterations; a transformer-fed
//! centre-tapped rectifier held 11 samples at 48 kHz × 4, and on one more
//! sample the gate passed with the mains constraint row 0.125 V off against a
//! 0.102 V tolerance after a limited step (alpha 0.28, 0.473 s).

mod support;

/// Rectifier charging loop (24 V behavioral mains, 13.5 mH source inductance,
/// split 450 µF reservoirs) whose field reaches a pickup into two triode
/// stages, a germanium clamp and a speaker analog. Ships as the operator's
/// small-buzz draft; here because of its warm-up sample.
const RECTIFIER_LOOP_TO_TRIODES: &str = "Rectifier loop into a pickup and two triode stages
.param f_mains=60
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 CCG=1.6p CGP=1.7p CCP=0.46p)
.model DS D(IS=1e-12 RS=0.05 N=1.2)
.model D1N34A D(IS=2.0e-7 RS=7.0 N=1.3)
R_in in 0 1Meg
B_mains mains 0 V={ 24*sin(2*pi*f_mains*time) }
Lsrc mains x 13.5m
D1 x resp DS
D2 resn x DS
Cp resp 0 450u
Rp resp 0 330
Cn resn 0 450u
Rn resn 0 330
Lpu 0 coil 2.19
Reddy 0 coil 1Meg
Kfield Lsrc Lpu 6e-6
Rdcr coil pu 5880
Cpu pu 0 583p
Rpu pu 0 111k
VCC vcc 0 DC 295
Rgs pu g1 68k
T1 g1 p1 k1 12AX7
Ra1 vcc p1 220k
Rk1 k1 0 2.2k
Ck1 k1 0 25u
Cc1 p1 g2 1.5n
Rg2 g2 0 1Meg
T2 g2 p2 k2 12AX7
Ra2 vcc p2 220k
Rk2 k2 0 2.2k
Ck2 k2 0 25u
C_clip p2 ci 100n
R_clip ci cn 470k
D_a cn 0 D1N34A
D_b 0 cn D1N34A
C1 cn a 2.3n
R1 a 0 1Meg
Rps a pk 470k
Rpl pk pt 100k
Lpk pt 0 3.66
Cpk pt 0 883p
Clp pk 0 386p
C2 pk out 1n
Rout out 0 1Meg
.end
";

/// Transformer-fed centre-tapped rectifier with its filter chain, 48 kHz × 4.
const TRANSFORMER_CT_RECTIFIER: &str = "Transformer-fed centre-tapped rectifier supply
.param f_mains=60
.oversampling 4
.model DS D(IS=1e-12 RS=0.05 N=1.2 CJO=30p)
R_in in 0 1Meg
B_mains mains 0 V={ 170*sin(2*pi*f_mains*time) }
Rpw mains pri 0.5
Lpri pri 0 10
Lsa sa 0 78.4
Lsb 0 sb 78.4
Kpa Lpri Lsa 0.999
Kpb Lpri Lsb 0.999
Kab Lsa Lsb 0.999
D1 sa res DS
D2 sb res DS
C1 res 0 50u
Rpow res 0 3k
Rf1 res rf 22k
Cf1 rf 0 4.7u
Rf2 rf vcc 100k
Cf2 vcc 0 1u
R_dec vcc vcc1 100k
C_dec vcc1 0 100n
.end
";

/// A diode behind a DC source and an inductor: the residual helpers carry a
/// voltage-source row and an inductor branch row beside the KCL rows.
const DIODE_BEHIND_SOURCE_AND_INDUCTOR: &str = "Diode behind a source and an inductor
.model DS D(IS=1e-12 N=1.2)
Rin in a 1k
L1 a b 10m
D1 b out DS
Rload out 0 1k
Cout out 0 100n
VB bias 0 DC 1
Rb bias b 100k
.end
";

fn build(spice: &str, output: &str) -> melange_solver::build::Built {
    let mut config = support::config_in_out_or_node1(spice, 48000.0);
    let netlist = support::parse_expanded(spice);
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).expect("MNA");
    let names = mna.node_names_in_index_order();
    let idx = names
        .iter()
        .position(|n| *n == output)
        .unwrap_or_else(|| panic!("no node {output}"))
        - 1;
    config.output_nodes = vec![idx];
    support::try_build_shipped_with(spice, &config, "auto", |_| {})
        .unwrap_or_else(|e| panic!("build refused: {e}"))
}

/// Renders `secs` of silence and prints the held-sample count, the ceiling
/// count and the line-search failure count.
fn silent_render_main(secs: f64) -> String {
    format!(
        "
fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for _ in 0..({secs:?} * 48000.0) as usize {{ let _ = process_sample(0.0, &mut s); }}
    println!(\"{{}} {{}} {{}}\", s.diag_nr_hold_count, s.diag_nr_max_iter_count, s.diag_ls_fail_count);
}}
"
    )
}

fn counts(out: &support::RunOutput) -> (u64, u64, u64) {
    let mut it = out
        .stdout
        .split_whitespace()
        .map(|t| t.parse::<u64>().unwrap());
    (it.next().unwrap(), it.next().unwrap(), it.next().unwrap())
}

#[test]
fn the_residual_helpers_cover_source_and_branch_rows_beside_the_kcl_rows() {
    let code = build(DIODE_BEHIND_SOURCE_AND_INDUCTOR, "out")
        .generated
        .code;
    for helper in ["fn kcl_residual(", "fn kcl_residual_inl("] {
        let body = &code[code.find(helper).unwrap_or_else(|| panic!("no {helper}"))..];
        let body = &body[..body.find("\n}\n").unwrap()];
        assert!(
            body.contains("// KCL rows (A)"),
            "{helper} has no KCL-row block"
        );
        assert!(
            body.contains("// source / constraint / inductor branch rows (V)"),
            "{helper} has no source / branch row block"
        );
        // Amps floor on the KCL rows, the node-step floor on the others.
        assert!(body.contains("1e-3 * den + 1e-9"), "{helper}: KCL floor");
        assert!(body.contains("1e-3 * den + 1e-6"), "{helper}: volt floor");
        // The infinity norm stays KCL-only: every `max_abs` update precedes
        // the source / branch block.
        let split = body
            .find("// source / constraint / inductor branch rows (V)")
            .unwrap();
        assert!(
            !body[split..].contains("max_abs = a"),
            "{helper}: a source / branch row feeds the KCL-only infinity norm"
        );
    }
    // No saturating inductor, no behavioral source: the reuse helper takes
    // neither `state` nor `sat_alpha`.
    assert!(code.contains("i_nl: &[f64; M]) -> (f64, bool, f64)"));
}

#[test]
fn a_rectifier_loop_into_triodes_converges_every_sample_within_the_shipped_budget() {
    let code = build(RECTIFIER_LOOP_TO_TRIODES, "out").generated.code;
    assert!(
        code.contains("pub const MAX_ITER: usize = 100;"),
        "the witness deck ships at the nodal floor budget"
    );
    // Behavioral V={} row: its true residual is in both helpers.
    assert_eq!(
        code.matches("// behavioral V={} source 0: true row residual (V)")
            .count(),
        2
    );
    let out = support::compile_and_run(&code, &silent_render_main(1.0), "merit_rect_loop");
    let (held, ceiling, _) = counts(&out);
    assert_eq!(
        held, 0,
        "held samples (31338 of 48000 under the KCL-only merit):\n{}",
        out.stderr
    );
    assert_eq!(ceiling, 0, "samples at the iteration ceiling");
}

#[test]
fn a_transformer_fed_rectifier_converges_every_sample_and_certifies_its_source_row() {
    let code = build(TRANSFORMER_CT_RECTIFIER, "vcc").generated.code;
    assert!(code.contains("pub const OVERSAMPLING_FACTOR: usize = 4;"));
    let out = support::compile_and_run(&code, &silent_render_main(0.5), "merit_ct_rectifier");
    let (held, ceiling, _) = counts(&out);
    assert_eq!(
        held, 0,
        "held samples (11 under the KCL-only merit):\n{}",
        out.stderr
    );
    assert_eq!(ceiling, 0, "samples at the iteration ceiling");
}
