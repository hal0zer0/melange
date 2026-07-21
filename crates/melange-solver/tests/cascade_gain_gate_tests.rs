//! Cascaded-gain gate (review-round-2 H4 — the "+39 dB series-of-tubes"
//! class).
//!
//! The July 2026 triode ×2 dispute demonstrated that a per-stage gain
//! error of a fraction of a dB is invisible to single-stage gates but
//! multiplies through N-stage cascades (series-of-tubes ×64, uniquorn
//! ×16, warpony ×8). These gates pin the measured gain of one 12AX7 CC
//! stage (with an inter-stage pad) and of EIGHT identical AC-coupled
//! stages against NGSPICE-derived literals — never against melange's own
//! output — so a per-stage error compounds ×8 and fails loudly:
//!   ×2 per stage  → +48 dB cumulative (fails by 47 dB)
//!   0.5 dB/stage  → +4 dB cumulative  (fails by 3 dB)
//!   0.125 dB/stage → +1 dB cumulative (gate edge)
//!
//! Reference provenance (measured 2026-07-21, ngspice-42): each stage's
//! triode is encoded as the published Koren plate current in an ngspice
//! B source, `Ip = 2·pwr(uramp(E1),EX)/KG1` with
//! E1 = (Vpk/KP)·ln(1+exp(KP·(1/MU+Vgk/√(KVB+Vpk²)))), MU=100 EX=1.4
//! KG1=1060 KP=600 KVB=300, plus the 10 pF Cgk/Cpk parasitics melange
//! auto-inserts; drive = Thevenin PWL (1 Ω), 20 mV·sin(2π·500·t),
//! 4800 samples @ 48 kHz, `.OPTIONS INTERP reltol=1e-4`; gain = settled
//! RMS (samples 2400..4800 = exactly 25 cycles) over input RMS. The same
//! decks live as the melange-vs-ngspice live comparison in
//! `crates/melange-validate/tests/data/triode_cascade8/` (run with
//! `--include-ignored`); this file pins the resulting literals so the
//! gate runs without ngspice.
//!
//!   1 stage + pad : ngspice −1.9366 dB | melange −1.9355 dB (Δ 0.0011)
//!   8 stages      : ngspice +21.7936 dB | melange +21.7969 dB (Δ 0.0033)
//!
//! The two circuits deliberately exercise BOTH codegen paths: the single
//! stage routes DK (N=8, M=2), the 8-stage cascade routes nodal (M=16),
//! matching `routing::auto_route` for each.
//!
//! RE-PIN PROTOCOL: these literals may only be re-derived from the
//! ngspice B-source decks, and any change requires an OOMOX_CONTRACT.md
//! §7 downstream impact list (per-stage delta × cascade count for every
//! shipped tube plugin) BEFORE the new value lands.
//!
//! Failure-mode verification (2026-07-21): removing the ×2 from the Koren
//! plate current in a SCRATCH COPY of the device templates (the exact
//! historical bug the campaign fixed) moves the single-stage pin by ≈ −4 dB
//! and the cascade pin by ≈ −30 dB — both far outside their gates.

mod support;

use melange_solver::codegen::CodegenConfig;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const SR: f64 = 48_000.0;
const DRIVE: f64 = 0.02;
const FREQ: f64 = 500.0;
const N_SAMPLES: usize = 4800;
const SETTLE: usize = 2400; // 25 full cycles of 500 Hz remain

/// One 12AX7 CC stage + inter-stage pad (100k/1.8k ≈ 1/56.6), the exact
/// building block of the 8-stage cascade below (output node = pad tap).
const STAGE1: &str = "\
Triode Cascade Stage
Rin in 0 1Meg
Cin in g1 100n
Rg1 g1 0 1Meg
T1 g1 p1 k1 12AX7
Rk1 k1 0 1.5k
Ck1 k1 0 25u
Rp1 vcc p1 100k
C1 p1 t1 100n
Ra1 t1 out 100k
Rb1 out 0 1.8k
Vcc vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";

/// Eight identical stages; stages 1..7 feed the next grid through the
/// 100k/1.8k pad (grid leak = Rb), stage 8 drives Cout into 1 Meg.
const CASCADE8: &str = "\
Triode Cascade 8
Rin in 0 1Meg
Cin in g1 100n
Rg1 g1 0 1Meg
T1 g1 p1 k1 12AX7
Rk1 k1 0 1.5k
Ck1 k1 0 25u
Rp1 vcc p1 100k
C1 p1 t1 100n
Ra1 t1 m1 100k
Rb1 m1 0 1.8k
T2 m1 p2 k2 12AX7
Rk2 k2 0 1.5k
Ck2 k2 0 25u
Rp2 vcc p2 100k
C2 p2 t2 100n
Ra2 t2 m2 100k
Rb2 m2 0 1.8k
T3 m2 p3 k3 12AX7
Rk3 k3 0 1.5k
Ck3 k3 0 25u
Rp3 vcc p3 100k
C3 p3 t3 100n
Ra3 t3 m3 100k
Rb3 m3 0 1.8k
T4 m3 p4 k4 12AX7
Rk4 k4 0 1.5k
Ck4 k4 0 25u
Rp4 vcc p4 100k
C4 p4 t4 100n
Ra4 t4 m4 100k
Rb4 m4 0 1.8k
T5 m4 p5 k5 12AX7
Rk5 k5 0 1.5k
Ck5 k5 0 25u
Rp5 vcc p5 100k
C5 p5 t5 100n
Ra5 t5 m5 100k
Rb5 m5 0 1.8k
T6 m5 p6 k6 12AX7
Rk6 k6 0 1.5k
Ck6 k6 0 25u
Rp6 vcc p6 100k
C6 p6 t6 100n
Ra6 t6 m6 100k
Rb6 m6 0 1.8k
T7 m6 p7 k7 12AX7
Rk7 k7 0 1.5k
Ck7 k7 0 25u
Rp7 vcc p7 100k
C7 p7 t7 100n
Ra7 t7 m7 100k
Rb7 m7 0 1.8k
T8 m7 p8 k8 12AX7
Rk8 k8 0 1.5k
Ck8 k8 0 25u
Rp8 vcc p8 100k
Cout p8 out 100n
Rout out 0 1Meg
Vcc vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";

fn config_for(spice: &str, name: &str) -> CodegenConfig {
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    CodegenConfig {
        circuit_name: name.to_string(),
        sample_rate: SR,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

fn settled_gain_db(output: &[f64]) -> f64 {
    let settled = &output[SETTLE..];
    let out_rms = (settled.iter().map(|v| v * v).sum::<f64>() / settled.len() as f64).sqrt();
    let in_rms = DRIVE / std::f64::consts::SQRT_2;
    20.0 * (out_rms / in_rms).log10()
}

/// Per-stage pin: −1.9355 dB melange vs −1.9366 dB ngspice (Δ 0.0011 dB).
/// ±0.3 dB gate = 270× the engine delta; a ×2 plate-current error moves
/// this stage by ≈ 4 dB.
#[test]
fn triode_stage_with_pad_gain_pinned() {
    let c = config_for(STAGE1, "cascade_stage1");
    let circuit = support::build_circuit(STAGE1, &c, "cascade_stage1");
    let out = support::run_sine(&circuit, FREQ, DRIVE, N_SAMPLES, SR);
    support::assert_finite(&out);
    let gain_db = settled_gain_db(&out);
    assert!(
        (gain_db - (-1.9355)).abs() < 0.3,
        "single triode stage+pad gain drifted: measured {gain_db:+.4} dB vs pinned \
         −1.9355 dB (ngspice B-source reference −1.9366 dB, 2026-07-21). \
         Per-stage deltas multiply through shipped cascades (×8/×16/×64) — \
         produce the OOMOX_CONTRACT §7 impact list before re-pinning."
    );
}

/// Cumulative 8-stage pin: +21.7969 dB melange vs +21.7936 dB ngspice
/// (Δ 0.0033 dB). ±1.0 dB gate = 300× the engine delta and catches any
/// per-stage error ≥ 0.125 dB; the ×2 class fails by ~47 dB.
#[test]
fn triode_cascade8_cumulative_gain_pinned() {
    let c = config_for(CASCADE8, "cascade8");
    // M=16 → nodal, matching routing::auto_route ("large nonlinear dimension").
    let circuit = support::build_circuit_nodal(CASCADE8, &c, "cascade8");
    let out = support::run_sine(&circuit, FREQ, DRIVE, N_SAMPLES, SR);
    support::assert_finite(&out);
    let gain_db = settled_gain_db(&out);
    assert!(
        (gain_db - 21.7969).abs() < 1.0,
        "8-stage triode cascade cumulative gain drifted: measured {gain_db:+.4} dB \
         vs pinned +21.7969 dB (ngspice B-source reference +21.7936 dB, \
         2026-07-21). This is the ×8-amplified per-stage gate: a 0.125 dB/stage \
         model drift lands exactly here, the July-2026 ×2 class fails by ~47 dB. \
         Produce the OOMOX_CONTRACT §7 impact list before re-pinning."
    );
}
