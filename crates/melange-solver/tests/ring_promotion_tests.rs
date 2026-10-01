//! The ring predicate's routing: which builds are promoted to backward Euler.
//!
//! `codegen::ring` decides it on the charge propagator linearised at the DC
//! operating point: promote on a lasting Nyquist-side pole (still above
//! −60 dB after 10 ms) whose input-to-output residue is at least −60 dB of
//! the passband gain AND louder than backward Euler's own worst in-band
//! change of the response, or on growth that backward Euler removes. The verdict
//! is recorded in the IR (`integration_reason`) and in the provenance JSON.

mod support;

use melange_solver::codegen::ir::IntegratorSelection;

/// Three 12AX7 stages cap-coupled (gain ~3800). The charge form carries no
/// accepted Newton residual forward on the algebraic rows, so there is no
/// fs/2 limit cycle for the ring predicate to answer: the build stays
/// trapezoidal and the output rests quietly.
const CASCADE: &str = "Cap-coupled triode cascade\n\
R_iso in g1 1Meg\nRg1 g1 0 1Meg\nT1 g1 p1 k1 12AX7\nRa1 vcc p1 100k\nRk1 k1 0 1.5k\n\
Cint12 p1 g2 100n\nRg2 g2 0 1Meg\nT2 g2 p2 k2 12AX7\nRa2 vcc p2 100k\nRk2 k2 0 1.5k\n\
Ck2 k2 0 25u\nCint23 p2 g3 100n\nRg3 g3 0 1Meg\nT3 g3 p3 k3 12AX7\nRa3 vcc p3 100k\n\
Rk3 k3 0 1.5k\nCk3 k3 0 25u\nCout p3 out 100n\nRload out 0 1Meg\nVCC vcc 0 250\n\
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)\n";

/// A linear stiff node: 1 kOhm into 10 pF (tau = 10 ns) at the output.
/// Trapezoidal maps the pole to z = −0.998 at 48 kHz; its input residue is
/// −55 dB of the passband.
const LINEAR_STIFF: &str =
    "stiff linear node\nR1 in out 1k\nC1 out 0 10p\nR2 out b 10k\nC2 b 0 1u\n";

/// A linear negative resistance (a VCCS feeding its own node) on an RC: a
/// real growing pole at rest, which backward Euler keeps too.
const GROWING: &str =
    "growing linear node\nR_in in out 1k\nR1 out 0 1k\nC1 out 0 1u\nG1 0 out out 0 3m\n";

fn provenance_source(code: &str) -> &str {
    let key = "\"integration_source\":\"";
    let start = code.find(key).expect("provenance") + key.len();
    &code[start..start + code[start..].find('"').unwrap()]
}

#[test]
fn the_triode_cascade_stays_trapezoidal_and_rests_quietly() {
    let config = support::config_for_spice(CASCADE, 48000.0);
    let (code, _, _) = support::generate_circuit_code(CASCADE, &config);
    assert_eq!(provenance_source(&code), "trap");
    assert!(code.contains("no lasting Nyquist-side pole"));
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let n = 96000usize;
    let mut worst = 0.0f64;
    let mut hist = [0.0f64; 7];
    for i in 0..n {
        let y = process_sample(0.0, &mut s)[0];
        hist.rotate_left(1);
        hist[6] = y;
        if i >= n / 2 {
            // 6th difference / 64: unity gain at fs/2, -142 dB at 1 kHz.
            let d = hist[6] - 6.0 * hist[5] + 15.0 * hist[4] - 20.0 * hist[3]
                + 15.0 * hist[2] - 6.0 * hist[1] + hist[0];
            worst = worst.max((d / 64.0).abs());
        }
    }
    println!(\"nyquist={:e}\", worst);
}";
    let out = support::compile_and_run(&code, main, "ring_cascade_rest");
    let nyq = out.parse_kv("nyquist").unwrap();
    eprintln!("cascade at rest, second second: fs/2 content {nyq:e} V");
    // Measured 7.3e-10 V (the whole-system form: a 28 mV limit cycle).
    assert!(nyq < 1e-8, "fs/2 content at rest {nyq:e} V");
}

#[test]
fn a_linear_stiff_node_is_promoted() {
    // The ring predicate runs on M = 0 circuits: a linear circuit rings at
    // fs/2 under trapezoidal integration like any other.
    let config = support::config_for_spice(LINEAR_STIFF, 48000.0);
    let (code, _, m) = support::generate_circuit_code_nodal(LINEAR_STIFF, &config);
    assert_eq!(m, 0);
    assert_eq!(provenance_source(&code), "auto-promoted");
    assert!(
        code.contains(
            "rings from the input at fs/2, louder than backward Euler's own in-band change"
        ),
        "the verdict is recorded"
    );
}

/// An output transformer (k = 0.999) driven through 100 kOhm, at 192 kHz: a
/// stiff mode rings at -56.2 dB of the passband, above the -60 dB threshold,
/// but backward Euler would change the in-band response by -43.3 dB (its
/// first-order error at the primary's 318 Hz corner). A ring quieter than
/// backward Euler's own damage stays trapezoidal: promoting it made the
/// 1 kHz response 28x less accurate (0.463 % against 0.0165 % vs ngspice).
const TRANSFORMER: &str = "output transformer
Ra in top 100k
R_pri top lo 500
L_pri lo 0 50
L_sec sl 0 0.78
R_sec sl sec 50
K1 L_pri L_sec 0.999
Cout sec out 100n
Rload out 0 1Meg
";

#[test]
fn a_ring_quieter_than_backward_eulers_cost_stays_trapezoidal() {
    let config = support::config_for_spice(TRANSFORMER, 192000.0);
    let (code, _, _) = support::generate_circuit_code_nodal(TRANSFORMER, &config);
    assert_eq!(provenance_source(&code), "trap");
    assert!(
        code.contains(
            "at or above the -60 dB ring threshold but quieter than backward Euler's own \
             in-band change"
        ),
        "the verdict is recorded"
    );
}

/// A BJT Schmitt trigger (emitter-coupled) at 192 kHz: its stiff mode rings
/// at -15 dB of the passband, and backward Euler would change the
/// small-signal response by +1.5 dB, more than the passband itself: the
/// linearisation at a regenerative circuit's operating point is
/// near-marginal, and says nothing about its switching edges. The cost
/// comparison does not hold there; the ring threshold decides, and says so.
/// (Kept trapezoidal, the switching thresholds were 0.05 V off ngspice's.)
const SCHMITT: &str = "BJT Schmitt trigger\nVcc vcc 0 DC 12\nE1 bx 0 in 0 1\nVb b1 bx DC 3\n\
Rb1 b1 base1 1k\nQ1 c1 base1 e NX\nRc1 vcc c1 4.7k\nR1 c1 base2 10k\nR2 base2 0 10k\n\
Q2 out base2 e NX\nRc2 vcc out 2.2k\nRe e 0 1k\nCm out 0 100p\n\
.model NX NPN(IS=1e-14 BF=200 VAF=100 CJE=5p CJC=3p TF=0.4n)\n";

#[test]
fn a_regenerative_circuit_falls_back_to_the_ring_threshold_and_says_so() {
    let config = support::config_for_spice(SCHMITT, 192000.0);
    let (code, _, _) = support::generate_circuit_code_nodal(SCHMITT, &config);
    assert_eq!(provenance_source(&code), "auto-promoted");
    assert!(
        code.contains("The small-signal comparison is not valid here"),
        "the fallback is announced"
    );
}

#[test]
fn a_real_growing_pole_keeps_trapezoidal() {
    let config = support::config_for_spice(GROWING, 48000.0);
    let (code, _, _) = support::generate_circuit_code_nodal(GROWING, &config);
    assert_eq!(provenance_source(&code), "trap");
    assert!(
        code.contains("a real growing pole at the DC operating point"),
        "the verdict is recorded"
    );
}

#[test]
fn pinned_integrators_are_not_decided() {
    for (force_trap, backward_euler, want) in [
        (true, false, IntegratorSelection::TrapCliFlag),
        (false, true, IntegratorSelection::BeCliFlag),
    ] {
        let mut config = support::config_for_spice(LINEAR_STIFF, 48000.0);
        config.force_trap = force_trap;
        config.backward_euler = backward_euler;
        let (code, _, _) = support::generate_circuit_code_nodal(LINEAR_STIFF, &config);
        assert!(!code.contains("\"integration_reason\""), "{want:?}");
        let want_source = if backward_euler { "explicit" } else { "trap" };
        assert_eq!(provenance_source(&code), want_source, "{want:?}");
    }
}
