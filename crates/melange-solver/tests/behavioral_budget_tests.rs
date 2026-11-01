//! A behavioral source forces backward Euler, and the Newton budget the build
//! ships is the backward-Euler budget. `build.rs` tunes one budget for the
//! trapezoidal system (with a +200 bonus when the DK kernel's trapezoidal
//! spectral radius exceeds 0.999) and one for backward Euler; the behavioral
//! forcing happens later, in the nodal IR builder, and used to leave the
//! trapezoidal budget in place. So a backward-Euler build inherited a bonus
//! keyed to an operator it never runs, and the shipped budget moved with the
//! host rate and the component scale: one rectifier deck shipped 100
//! iterations at 48 kHz and 250 at 44.1 kHz.

mod support;

/// Rectifier charging loop at 100× impedance behind a pickup and two triode
/// stages: its DK kernel's trapezoidal spectral radius is above 0.999 at
/// 48 kHz, so the trapezoidal budget would carry the +200 bonus.
const RECTIFIER_LOOP_HIGH_Z: &str =
    "Rectifier loop at 100x impedance into a pickup and two triode stages
.param f_mains=60
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 CCG=1.6p CGP=1.7p CCP=0.46p)
.model DS D(IS=1e-12 RS=0.05 N=1.2)
.model D1N34A D(IS=2.0e-7 RS=7.0 N=1.3)
R_in in 0 1Meg
B_mains mains 0 V={ 24*sin(2*pi*f_mains*time) }
Lsrc mains x 1.35
D1 x resp DS
D2 resn x DS
Cp resp 0 4.5u
Rp resp 0 33k
Cn resn 0 4.5u
Rn resn 0 33k
Lpu 0 coil 2.19
Reddy 0 coil 1Meg
Kfield Lsrc Lpu 6e-5
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

fn built(backward_euler_flag: bool) -> melange_solver::build::Built {
    let config = support::config_in_out_or_node1(RECTIFIER_LOOP_HIGH_Z, 48000.0);
    support::try_build_shipped_with(RECTIFIER_LOOP_HIGH_Z, &config, "auto", |o| {
        o.backward_euler = backward_euler_flag;
    })
    .unwrap_or_else(|e| panic!("build refused: {e}"))
}

fn max_iter(code: &str) -> usize {
    let line = code
        .lines()
        .find(|l| l.starts_with("pub const MAX_ITER: usize = "))
        .expect("MAX_ITER const");
    line.trim_start_matches("pub const MAX_ITER: usize = ")
        .trim_end_matches(';')
        .parse()
        .unwrap()
}

#[test]
fn a_behavioral_forced_backward_euler_build_ships_the_backward_euler_budget() {
    let forced = built(false);
    assert!(
        forced
            .generated
            .code
            .contains("\"integration_source\":\"behavioral\""),
        "the deck's build is forced to backward Euler by its behavioral source"
    );
    let flagged = built(true);
    let (forced_budget, flagged_budget) = (
        max_iter(&forced.generated.code),
        max_iter(&flagged.generated.code),
    );
    assert_eq!(
        forced_budget, flagged_budget,
        "forced {forced_budget} vs --backward-euler {flagged_budget}: the budget must be the \
         integrator's, not the trapezoidal system's"
    );
    // The nodal floor, not the trapezoidal 50 + 200 bonus this deck's DK
    // kernel (rho > 0.999) would earn.
    assert_eq!(forced_budget, 100);
}
