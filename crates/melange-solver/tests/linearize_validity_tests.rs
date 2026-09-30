//! A `.linearize`d device is a reduced model: outside the region its
//! small-signal model assumes, the generated code's answer comes from a model
//! that no longer describes the device. Each sample it spends there is a
//! reduced-model exit, counted in `diag_reduced_model_exit_count` and in
//! `diag_unsolved_sample_count`, so every verb refuses the render.
//!
//! The regions, stated physically (`LinearizedCheck`):
//! - triode: the linear plate current stays at or above zero (a plate
//!   current cannot reverse; below zero the tube is cut off), and the grid
//!   stays below its conduction onset (the grid law's 0.3 uA starting point);
//! - BJT, forward active: the linear collector current keeps its forward
//!   sign (cut off at zero) and the B-C junction stays reverse biased.
//!
//! Each edge has its own witness, approached alone with half-wave drive. A
//! cathode follower idling at tens of uA with volts on its grid, the failure
//! that motivated this, is the cutoff witness: linearized, it passed a sine
//! the real tube clips on every negative swing.

mod support;

const FS: f64 = 48000.0;

/// (reduced-model exits, unsolved samples) over 0.1 s of a 1 kHz sine at
/// `amp` volts into `spice`, whose output node is `out`. `half` keeps only
/// the positive (+1) or negative (-1) half-cycles, so one edge of the region
/// is approached at a time; 0 is the whole sine.
fn exits(spice: &str, amp: f64, half: i32, tag: &str) -> (u64, u64) {
    let mut config = support::config_for_spice(spice, FS);
    config.dc_block = false;
    let code = support::generate_circuit_code(spice, &config).0;
    assert!(
        code.contains("diag_reduced_model_exit_count"),
        "{tag}: the build carries no linearized-device check"
    );
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    for k in 0..4800 {{
        let x = {amp:?} * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / {FS:?}).sin();
        let x = match {half} {{ 1 => x.max(0.0), -1 => x.min(0.0), _ => x }};
        let _ = process_sample(x, &mut s);
    }}
    println!(\"{{}} {{}}\", s.diag_reduced_model_exit_count, s.diag_unsolved_sample_count);
}}"
    );
    let out = support::compile_and_run(&code, &main, tag).stdout;
    let v: Vec<u64> = out.split_whitespace().map(|t| t.parse().unwrap()).collect();
    (v[0], v[1])
}

/// A cathode follower at ~25 uA idle: grid tied through 1 MOhm, cathode on
/// 100 kOhm. Linearized.
const FOLLOWER: &str = "linearized cathode follower
VCC vcc 0 DC 250
Cin in g 1u
Rg g 0 1Meg
T1 g vcc k TX
Rk k 0 100k
Cout k out 1u
Rl out 0 1Meg
.model TX TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
.linearize T1
";

/// Plate cutoff alone: negative half-cycles lower the plate current and never
/// raise the grid.
#[test]
fn a_linearized_follower_driven_into_cutoff_is_refused() {
    assert_eq!(exits(FOLLOWER, 0.1, 0, "lin_cf_small"), (0, 0));
    let (e, u) = exits(FOLLOWER, 10.0, -1, "lin_cf_cutoff");
    assert!(e > 0, "-10 V swings the linear plate current below zero");
    assert!(
        u >= e,
        "each exit is an unsolved sample: exits {e}, unsolved {u}"
    );
}

/// A common-cathode stage whose grid is driven through a small resistance,
/// linearized.
const COMMON_CATHODE: &str = "linearized common cathode
VCC vcc 0 DC 250
Cin in g 1u
Rg g 0 1Meg
T1 g p k TX
Ra vcc p 100k
Rk k 0 1.5k
Ck k 0 100u
Cout p out 100n
Rl out 0 1Meg
.model TX TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
.linearize T1
";

/// Grid onset alone: positive half-cycles raise the plate current and take
/// the grid (bias about -1.4 V) past its onset.
#[test]
fn a_linearized_grid_driven_past_its_onset_is_refused() {
    assert_eq!(exits(COMMON_CATHODE, 0.1, 0, "lin_cc_small"), (0, 0));
    let (e, u) = exits(COMMON_CATHODE, 3.0, 1, "lin_cc_onset");
    assert!(e > 0 && u >= e, "exits {e}, unsolved {u}");
}

/// A common-emitter stage, linearized, driven DC-coupled through 10 kOhm
/// with an unbypassed emitter, so no capacitor shifts the bias under a
/// half-wave drive: each half approaches only its own edge (about 1.5 mA
/// idle, Vc about 5 V).
const COMMON_EMITTER: &str = "linearized common emitter
VCC vcc 0 DC 12
Rin in b 10k
R1 vcc b 47k
Q1 c b e QX
RC vcc c 4.7k
RE e 0 1k
Cout c out 10u
Rl out 0 100k
.model QX NPN(IS=1e-14 BF=200)
.linearize Q1
";

/// NPN: positive half-cycles saturate it (B-C forward), negative ones cut it
/// off, each on its own.
#[test]
fn a_linearized_bjt_out_of_forward_active_is_refused() {
    assert_eq!(exits(COMMON_EMITTER, 0.001, 0, "lin_ce_small"), (0, 0));
    for (half, edge) in [(1, "saturation"), (-1, "cutoff")] {
        let (e, u) = exits(COMMON_EMITTER, 5.0, half, &format!("lin_ce_{edge}"));
        assert!(e > 0 && u >= e, "{edge}: exits {e}, unsolved {u}");
    }
}

/// PNP: the same edges with the signs mirrored.
#[test]
fn a_linearized_pnp_out_of_forward_active_is_refused() {
    let pnp = COMMON_EMITTER
        .replace("NPN(", "PNP(")
        .replace("VCC vcc 0 DC 12", "VCC vcc 0 DC -12");
    assert_eq!(exits(&pnp, 0.001, 0, "lin_pnp_small"), (0, 0));
    for (half, edge) in [(-1, "saturation"), (1, "cutoff")] {
        let (e, u) = exits(&pnp, 5.0, half, &format!("lin_pnp_{edge}"));
        assert!(e > 0 && u >= e, "{edge}: exits {e}, unsolved {u}");
    }
}
