//! Saturating inductors driven INTO the knee, at low frequency.
//!
//! The golden corpus runs saturation code but never its knee: no golden
//! program takes any inductor past i/Isat = 0.37 (measured 2026-09-28), the
//! same blind spot that let a broken op-amp rail path ship. Core saturation is
//! a low-frequency effect — at 1 kHz a henry-class winding needs kilovolts to
//! reach Isat — so these tests drive at 30 Hz, into saturation.
//!
//! References, both independent of melange:
//! - a scalar trapezoidal recurrence of `V − R·i = dΦ(i)/dt`,
//!   `Φ(i) = L_mag·Isat·tanh(i/Isat) + L_air·i` with `L_air = LAIR·L0` and
//!   `L_mag = L0 − L_air`, solved by bracketed Newton to 1e-16, at the same
//!   48 kHz (checks the implementation), and
//! - the same recurrence at 1024× (≈continuous; checks the physics). At these
//!   drives the two agree to 7e-6 relative on H1, so both are gated.
//!
//! C2 is the shared-core discriminator: loaded, the winding current is far
//! above Isat while the core stays linear (Lenz: load MMFs cancel); open, the
//! magnetizing current saturates the core. Per-winding saturation got the
//! loaded case wrong (H3/H1 = 0.16); the shared core gives ~0.

mod support;

use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const FS: f64 = 48000.0;
const F: f64 = 30.0;

/// Saturating RL: R = 99 + the 1 Ω input = 100 Ω, L0 = 1 H, Isat = 10 mA,
/// air-core floor LAIR = 3e-4 (L_air = 0.3 mH).
const C1: &str =
    "saturating RL driven into the knee at LF\nR1 in a 99\nL1 a 0 1 ISAT=10m LAIR=3e-4\n";

/// (drive V, pk/Isat, H1 A [1x], H3/H1 [1x], H5/H1 [1x], H1 A [1024x], H3/H1 [1024x], H5/H1 [1024x])
type C1Row = (f64, f64, f64, f64, f64, f64, f64, f64);

const C1_REF: [C1Row; 3] = [
    (
        0.5,
        0.2380,
        2.368756e-3,
        0.004599,
        0.000039,
        2.368758e-3,
        0.004599,
        0.000039,
    ),
    (
        2.0,
        1.2885,
        1.146091e-2,
        0.103668,
        0.020073,
        1.146092e-2,
        0.103668,
        0.020073,
    ),
    (
        5.0,
        5.0000,
        4.079793e-2,
        0.272194,
        0.110529,
        4.079782e-2,
        0.272193,
        0.110527,
    ),
];

/// 1:1 shared-core transformer, 1 H windings, K = 0.99, ISAT on the primary,
/// magnetizing air floor 3e-4 of L_ref (CORE=steel; K = 0.99 is far looser than
/// real iron, which compile notes).
fn c2(load: &str) -> String {
    format!(
        "shared-core saturating transformer\nR_p in p 99\nL_pri p 0 1 ISAT=10m CORE=steel\nL_sec s 0 1\n\
         K1 L_pri L_sec 0.99\nR_L s out 1m\nR_load out 0 {load}\n"
    )
}

fn node(spice: &str, name: &str) -> usize {
    let netlist = Netlist::parse(spice).unwrap();
    MnaSystem::from_netlist(&netlist).unwrap().node_map[name] - 1
}

fn nodal_code(spice: &str, out: &str) -> String {
    let mut config = support::config_for_spice(spice, FS);
    config.output_nodes = vec![node(spice, out)];
    config.dc_block = false;
    support::generate_circuit_code_nodal(spice, &config).0
}

/// Every declared "this sample was not a clean solve" counter, summed.
fn bad_counters(code: &str) -> String {
    let fields = [
        "diag_nr_hold_count",
        "diag_nr_unconverged_commit_count",
        "diag_substep_count",
        "diag_be_fallback_count",
        "diag_nan_reset_count",
        "diag_magnitude_reset_count",
    ];
    let present: Vec<String> = fields
        .iter()
        .filter(|f| code.contains(&format!("pub {f}: ")))
        .map(|f| format!("s.{f}"))
        .collect();
    if present.is_empty() {
        "0u64".into()
    } else {
        present.join(" + ")
    }
}

/// Render 2 s of a 30 Hz sine per drive; over the 1-2 s window report the peak
/// and harmonics 1..5 of `probe` (a Rust expression over `s`), plus
/// `extra_pk` (another expression, peak only) and the bad-sample count.
fn render(code: &str, drives: &[f64], probe: &str, extra_pk: &str, tag: &str) -> Vec<Vec<f64>> {
    let drives: Vec<String> = drives.iter().map(|d| format!("{d:?}")).collect();
    let bad = bad_counters(code);
    let main = format!(
        "fn main() {{
    for amp in [{drives}] {{
        let mut s = CircuitState::default();
        s.set_sample_rate({FS:?});
        let n = (2.0 * {FS:?}) as usize;
        let mut ss: Vec<f64> = Vec::with_capacity(n / 2);
        let mut extra = 0.0f64;
        for k in 1..=n {{
            let x = amp * (2.0 * std::f64::consts::PI * {F:?} * k as f64 / {FS:?}).sin();
            let _ = process_sample(x, &mut s);
            if k > n / 2 {{ ss.push({probe}); extra = extra.max((({extra_pk}) as f64).abs()); }}
        }}
        let pk = ss.iter().fold(0.0f64, |a, &b| a.max(b.abs()));
        let mut h = [0.0f64; 6];
        for hh in 1..6 {{
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (j, &v) in ss.iter().enumerate() {{
                let w = 2.0 * std::f64::consts::PI * hh as f64 * {F:?} * j as f64 / {FS:?};
                re += v * w.cos();
                im += v * w.sin();
            }}
            h[hh] = 2.0 * (re * re + im * im).sqrt() / ss.len() as f64;
        }}
        let bad = {bad};
        println!(\"{{amp}} {{pk}} {{}} {{}} {{}} {{}} {{}} {{extra}} {{bad}}\", h[1], h[2], h[3], h[4], h[5]);
    }}
}}",
        drives = drives.join(", ")
    );
    support::compile_and_run(code, &main, tag)
        .stdout
        .lines()
        .map(|l| l.split_whitespace().map(|t| t.parse().unwrap()).collect())
        .collect()
}

/// C1's gates against both references. The 1× recurrence is held to H1 1e-5
/// relative and H3/H5 1e-4 absolute; the 1024× one to `tol_1024` (H1
/// relative, ratios absolute). `Err` names the first gate that fails.
fn c1_gates(code: &str, table: &[C1Row], tol_1024: (f64, f64), tag: &str) -> Result<(), String> {
    let drives: Vec<f64> = table.iter().map(|r| r.0).collect();
    let rows = render(code, &drives, "s.v_prev[SAT_IND_0_AUG_ROW]", "0.0", tag);
    let isat = 10e-3;
    for (row, r) in rows.iter().zip(table.iter()) {
        let (amp, pk, h1, h2, h3, h5, bad) =
            (row[0], row[1], row[2], row[3], row[4], row[6], row[8]);
        if bad != 0.0 {
            return Err(format!(
                "{amp} V: {bad} unsolved / sub-step / BE / reset samples"
            ));
        }
        let (h3r, h5r) = (h3 / h1, h5 / h1);
        for (name, h1_ref, h3_ref, h5_ref, (h1_tol, ratio_tol)) in [
            ("1x", r.2, r.3, r.4, (1e-5, 1e-4)),
            ("1024x", r.5, r.6, r.7, tol_1024),
        ] {
            if ((h1 - h1_ref) / h1_ref).abs() > h1_tol {
                return Err(format!("{amp} V: H1 {h1:.6e} vs {name} {h1_ref:.6e}"));
            }
            if (h3r - h3_ref).abs() > ratio_tol || (h5r - h5_ref).abs() > ratio_tol {
                return Err(format!(
                    "{amp} V: H3/H1 {h3r:.6} H5/H1 {h5r:.6} vs {name} {h3_ref:.6} / {h5_ref:.6}"
                ));
            }
        }
        if (pk / isat - r.1).abs() > 1e-3 * r.1 {
            return Err(format!("{amp} V: peak {:.4}·Isat vs {:.4}", pk / isat, r.1));
        }
        // The current can never pass V/R (R = 100 Ω): past it is a ring.
        if pk > amp / 100.0 * (1.0 + 1e-6) {
            return Err(format!("{amp} V: peak {pk:.6} A over the V/R ceiling"));
        }
        // A symmetric tanh core under symmetric drive makes odd harmonics only.
        if h2 / h1 > 1e-9 {
            return Err(format!(
                "{amp} V: H2/H1 {:.3e} from a symmetric core",
                h2 / h1
            ));
        }
    }
    // H1 and peak rise with drive (H3/H1 does NOT: it peaks and falls, so it is
    // not gated). 0.5 % tolerance: this catches a falling response, it is not a
    // precision criterion.
    for w in rows.windows(2) {
        if w[1][2] < w[0][2] * 0.995 || w[1][1] < w[0][1] * 0.995 {
            return Err(format!(
                "response falls with drive between {} V and {} V",
                w[0][0], w[1][0]
            ));
        }
    }
    Ok(())
}

#[test]
fn c1_saturating_rl_matches_both_references() {
    let code = nodal_code(C1, "a");
    c1_gates(&code, &C1_REF, (1e-5, 1e-4), "c1").unwrap();
}

/// The gates must fail on a broken implementation, or they gate nothing.
#[test]
fn c1_gates_catch_a_broken_flux_device() {
    let code = nodal_code(C1, "a");
    // History correction (Φ(i_prev) in place of L0·i_prev) removed everywhere.
    let no_history: String = code
        .lines()
        .filter(|l| !l.contains("(phi - SAT_IND_0_L0 * ip)"))
        .collect::<Vec<_>>()
        .join("\n");
    assert_ne!(
        no_history, code,
        "test premise: history correction not found"
    );
    assert!(
        c1_gates(&no_history, &C1_REF, (1e-5, 1e-4), "c1_nohist").is_err(),
        "history mutant passed"
    );
    // Main-loop Jacobian stamp removed.
    let no_jacobian: String = code
        .lines()
        .filter(|l| !l.contains("chord_lu[SAT_IND_0_AUG_ROW][SAT_IND_0_AUG_ROW] +="))
        .collect::<Vec<_>>()
        .join("\n");
    assert_ne!(
        no_jacobian, code,
        "test premise: main-loop Jacobian stamp not found"
    );
    assert!(
        c1_gates(&no_jacobian, &C1_REF, (1e-5, 1e-4), "c1_nojac").is_err(),
        "Jacobian mutant passed"
    );
    // No air-core floor: the pure-tanh law, 4e-5 low on H1 at 5 V.
    let no_air = nodal_code(&C1.replace("LAIR=3e-4", "LAIR=0"), "a");
    assert!(
        c1_gates(&no_air, &C1_REF, (1e-5, 1e-4), "c1_noair").is_err(),
        "zero-floor mutant passed"
    );
    // Linear inductor: no knee at all. Same topology, so its branch current is
    // on the augmented row the saturating build names SAT_IND_0_AUG_ROW (2).
    let linear = nodal_code("linear RL\nR1 in a 99\nL1 a 0 1\n", "a")
        + "\npub const SAT_IND_0_AUG_ROW: usize = 2;\n";
    assert!(
        c1_gates(&linear, &C1_REF, (1e-5, 1e-4), "c1_lin").is_err(),
        "linear inductor passed"
    );
}

/// Every Newton site that can commit a sample checks the flux row the same way,
/// so a wrong flux Jacobian is caught wherever it is. A trapezoidal build runs
/// two instances of one solve routine, trapezoidal then backward Euler, each
/// with a main loop and a sub-step: four sites, in that order. Deleting the
/// Jacobian correction while keeping the companion moves the Newton fixed point
/// off the solution; the step check alone accepts it. Deleted at the first k
/// sites, the next site down the ladder recovers the right answer; deleted at
/// all four, every sample is counted as held. With a site's residual removed as
/// well, that site accepts the wrong point and nothing is counted, which is
/// what shows the residual is the detector.
#[test]
fn c1_jacobian_deletion_is_caught_at_every_newton_site() {
    let code = nodal_code(C1, "a");
    let jac = |m: &str| format!("{m}[SAT_IND_0_AUG_ROW][SAT_IND_0_AUG_ROW] +=");
    // Sites in emission order: (Jacobian stamp line, its occurrence index).
    let (chord, sub) = (jac("chord_lu"), jac("g_s"));
    let sites = [(&chord, 0usize), (&sub, 0), (&chord, 1), (&sub, 1)];
    // The residual line of each site: (flag, occurrence index).
    let residuals = [
        ("max_step_exceeded", 0usize),
        ("sub_step_exceeded", 0),
        ("max_step_exceeded", 1),
        ("sub_step_exceeded", 1),
    ];
    // Delete the Jacobian stamp at the first `k` sites, and optionally blind
    // one site's residual.
    let mutate = |k: usize, blind: Option<usize>| -> String {
        let mut seen: std::collections::HashMap<String, usize> = Default::default();
        let mut deleted = 0usize;
        let mut blinded = 0usize;
        let out: Vec<&str> = code
            .lines()
            .filter(|l| {
                for (pat, _) in &sites {
                    if l.contains(pat.as_str()) {
                        let n = seen.entry(pat.to_string()).or_default();
                        let idx = *n;
                        *n += 1;
                        if sites[..k].iter().any(|(p, i)| p == pat && *i == idx) {
                            deleted += 1;
                            return false;
                        }
                        return true;
                    }
                }
                if l.contains("acc.abs()") {
                    for (flag, _) in &residuals {
                        let f = format!("{flag} = true; }}");
                        if l.contains(&f) {
                            let key = format!("res:{flag}");
                            let n = seen.entry(key).or_default();
                            let idx = *n;
                            *n += 1;
                            if let Some(b) = blind {
                                if residuals[b] == (*flag, idx) {
                                    blinded += 1;
                                    return false;
                                }
                            }
                            return true;
                        }
                    }
                }
                true
            })
            .collect();
        assert_eq!(deleted, k, "test premise: one stamp per site");
        assert!(
            blind.is_none() || blinded == 1,
            "test premise: the site's residual line"
        );
        out.join("\n")
    };
    // (peak i_L / Isat, sub-steps, BE entries, holds) over 1-2 s at 5 V.
    let run = |code: &str, tag: &str| -> (f64, u64, u64, u64) {
        let main_fn = "fn main() {
    let (fs, f) = (48000.0f64, 30.0f64);
    let mut s = CircuitState::default();
    s.set_sample_rate(fs);
    let mut pk = 0.0f64;
    for i in 0..(2.0 * fs) as usize {
        let _ = process_sample(5.0 * (2.0 * std::f64::consts::PI * f * i as f64 / fs).sin(), &mut s);
        if i >= fs as usize { pk = pk.max(s.v_prev[SAT_IND_0_AUG_ROW].abs()); }
    }
    println!(\"{} {} {} {}\", pk / 10e-3, s.diag_substep_count, s.diag_be_fallback_count, s.diag_nr_hold_count);
}";
        let out = support::compile_and_run(code, main_fn, tag).stdout;
        let v: Vec<f64> = out.split_whitespace().map(|t| t.parse().unwrap()).collect();
        (v[0], v[1] as u64, v[2] as u64, v[3] as u64)
    };
    let pk_ref = C1_REF[2].1;
    let right = |pk: f64| ((pk - pk_ref) / pk_ref).abs() < 1e-4;

    // Trap main deleted: the trap sub-step recovers.
    let (pk, subs, bes, holds) = run(&mutate(1, None), "c1_jac_1");
    assert!(
        subs > 1000 && bes == 0 && holds == 0 && right(pk),
        "trap main: pk {pk} sub {subs} be {bes} hold {holds}"
    );
    // Trap main + sub deleted: the BE main loop recovers.
    let (pk, _, bes, holds) = run(&mutate(2, None), "c1_jac_2");
    assert!(
        bes > 1000 && holds == 0 && right(pk),
        "trap sub-step: pk {pk} be {bes} hold {holds}"
    );
    let (pk, _, _, holds) = run(&mutate(2, Some(1)), "c1_jac_2_blind");
    assert!(
        holds == 0 && !right(pk),
        "trap sub-step without its residual must accept the wrong point: pk {pk}"
    );
    // Through BE main deleted: the BE sub-step recovers.
    let (pk, _, bes, holds) = run(&mutate(3, None), "c1_jac_3");
    assert!(
        bes > 1000 && holds == 0 && right(pk),
        "BE main: pk {pk} be {bes} hold {holds}"
    );
    let (pk, _, _, holds) = run(&mutate(3, Some(2)), "c1_jac_3_blind");
    assert!(
        holds == 0 && !right(pk),
        "BE main without its residual must accept the wrong point: pk {pk}"
    );
    // All four deleted: nothing can solve it; every such sample is held.
    let (_, _, _, holds) = run(&mutate(4, None), "c1_jac_4");
    assert!(holds > 1000, "all sites: only {holds} held samples");
    let (pk, _, _, holds) = run(&mutate(4, Some(3)), "c1_jac_4_blind");
    assert!(
        holds == 0 && !right(pk),
        "BE sub-step without its residual must accept the wrong point: pk {pk}"
    );
}

/// Deep saturation, pre-registered: with the air-core floor the circuit is
/// R + L_air once the core saturates, ωL_air ≪ R, so the current settles on
/// V/R, and the trapezoidal step factor on the air slope,
/// (αL_air − R)/(αL_air + R) = −0.55, damps any alternation: no ring and no
/// backward-Euler latch (a latched sample counts as a BE fallback, which the
/// gates refuse). 100 V is 100× Isat, and the most a drive can be: generated
/// code clamps its input to ±100 V. The 1× recurrence sits 1e-4 from the
/// 1024× one on H1 here (the trapezoidal rule at a knee a few samples wide),
/// so 1024× is held to 2e-4 on H1 and 3e-4 on the ratios.
const C1_DEEP: [C1Row; 3] = [
    (
        10.0,
        10.0000,
        9.162833e-2,
        0.191747,
        0.110251,
        9.161853e-2,
        0.191852,
        0.110260,
    ),
    (
        20.0,
        20.0000,
        1.930396e-1,
        0.108684,
        0.085239,
        1.930551e-1,
        0.108543,
        0.085158,
    ),
    (
        100.0,
        100.0000,
        9.962042e-1,
        0.023559,
        0.022648,
        9.962352e-1,
        0.023435,
        0.022534,
    ),
];

#[test]
fn c1_deep_saturation_settles_on_the_rl_limit_without_ringing() {
    let code = nodal_code(C1, "a");
    c1_gates(&code, &C1_DEEP, (2e-4, 3e-4), "c1_deep").unwrap();
}

/// With no air-core floor (`LAIR=0`) deep saturation rings under the
/// trapezoidal rule: as L_diff → 0 the RL step factor tends to −1, and the
/// current overshoots the physical ceiling V/R. Same scalar recurrence as
/// C1's reference: at 1× the peak is 11.83 and 23.94 Isat at 10 and 20 V
/// against a ceiling of 10 and 20, and 4× does not cure it (10.49, 24.69).
/// The 1024× recurrence lands on the ceiling.
///
/// The runtime BE-latch closes it: it detects the sample-to-sample
/// alternation and switches the instance to backward Euler for the rest of the
/// stream.
///
/// Do not widen the 5e-4 H1 tolerance below: it once hid reference constants
/// that were the 1× trapezoidal values, 2.8e-4 off the converged 1024× ones.
/// Check a reference's convergence (256/1024/4096× agree here to 1e-9) before
/// touching the tolerance.
///
/// (drive V, 1024× peak/Isat, 1024× H1 A)
const C1_DEEP_NO_FLOOR: [(f64, f64, f64); 2] =
    [(10.0, 10.0000, 9.165534e-2), (20.0, 20.0000, 1.930722e-1)];

#[test]
fn c1_zero_floor_deep_saturation_ring_is_caught_by_the_latch() {
    let code = nodal_code(&C1.replace("LAIR=3e-4", "LAIR=0"), "a");
    let drives: Vec<String> = C1_DEEP_NO_FLOOR
        .iter()
        .map(|r| format!("{:?}", r.0))
        .collect();
    let main = format!(
        "fn main() {{
    for amp in [{drives}] {{
        let fs = {FS:?};
        let mut s = CircuitState::default();
        s.set_sample_rate(fs);
        let (mut pk, mut re, mut im, mut cnt) = (0.0f64, 0.0f64, 0.0f64, 0usize);
        for k in 0..(2.0 * fs) as usize {{
            let w = 2.0 * std::f64::consts::PI * {F:?} * k as f64 / fs;
            let _ = process_sample(amp * w.sin(), &mut s);
            if k >= fs as usize {{
                let i = s.v_prev[SAT_IND_0_AUG_ROW];
                pk = pk.max(i.abs());
                re += i * w.cos();
                im += i * w.sin();
                cnt += 1;
            }}
        }}
        let h1 = 2.0 * (re / cnt as f64).hypot(im / cnt as f64);
        println!(\"{{}} {{}} {{}} {{}}\", pk / 10e-3, h1, s.diag_be_latch_count, s.diag_nr_hold_count + s.diag_substep_count);
    }}
}}",
        drives = drives.join(", ")
    );
    let out = support::compile_and_run(&code, &main, "c1_deep_no_floor").stdout;
    for (line, &(amp, pk_ref, h1_ref)) in out.lines().zip(C1_DEEP_NO_FLOOR.iter()) {
        let v: Vec<f64> = line
            .split_whitespace()
            .map(|t| t.parse().unwrap())
            .collect();
        let (pk, h1, latches, bad) = (v[0], v[1], v[2], v[3]);
        assert_eq!(bad, 0.0, "{amp} V: held or sub-stepped samples");
        assert!(latches >= 1.0, "{amp} V: the latch never fired on the ring");
        assert!(
            ((pk - pk_ref) / pk_ref).abs() <= 1e-2 && pk <= amp / 100.0 / 10e-3 * (1.0 + 1e-6),
            "{amp} V: peak {pk:.4} Isat vs 1024x {pk_ref} (ceiling {})",
            amp / 100.0 / 10e-3
        );
        assert!(
            ((h1 - h1_ref) / h1_ref).abs() <= 5e-4,
            "{amp} V: H1 {h1:.6e} vs 1024x {h1_ref:.6e}"
        );
    }
}

/// A single-supply op-amp at high gain railing into a deep-saturating choke:
/// at each input zero crossing the output swings rail to rail within one
/// sample, where backward-Euler Newton fails and the sub-step rescues it.
const RAILING_CHOKE: &str = "\
single-supply op-amp overdrive into a deep-saturating choke
Vcc vcc 0 DC 9
R_b1 vcc vbias 100k
R_b2 vbias 0 100k
C_b vbias 0 10u
C_in in np 100n
R_in np vbias 1Meg
U1 np nm oa TL072
R_f oa nm 500k
R_g nm ng 4.7k
C_g ng 0 10u
C_c oa n1 1u
R_1 n1 n2 1k
L_sat n2 0 100m ISAT=0.2m CORE=steel
R_t n2 out 10k
R_v out 0 100k
.model TL072 OA(AOL=200000 VCC=9 VEE=0)
";

/// A choke-loaded common-source stage driven into its choke's saturation
/// (M = 2, nodal full-LU): the deep-saturation witness with devices.
const CHOKE_STAGE: &str = "\
choke-loaded common-source stage
.model 2N7000 NMOS(KP=0.1 VTO=2.1 LAMBDA=0.01 GAMMA=0.5 PHI=0.6)
VCC vcc 0 DC 24
Cin  in     gate_i 1u
Rb1  vcc    gate_i 1MEG
Rb2  gate_i 0      170k
Rg   gate_i gate   1k
Lp   vcc    drain  5 ISAT=20m CORE=gapped
Rdcr drain  drain_d 120
Cw   vcc    drain  220p
Rw   vcc    drain  470k
M1   drain_d gate  src  0  2N7000
Rs   src    0      220
Cs   src    0      100u
Cout drain  out    100n
Rl   out    0      100k
";

/// The runtime BE-latch is armed on saturating circuits, including M = 0 ones
/// (the flux law is nonlinear on its own). A latched trapezoidal build runs the
/// SAME backward-Euler routine a `--backward-euler` build runs (main loop,
/// sub-step, pin-and-resolve), on BE matrices the IR bakes by the same
/// expressions, so from the same state the output is bit-identical. Before
/// that routine was shared, the latched path was a separate BE ladder without
/// the sub-step: it agreed to 1e-9 here, and held 1996 samples/s on a railing
/// op-amp driving a deep-saturating choke, where the BE build sub-steps.
///
/// Harness traps, worth keeping: `CircuitState::default()` warms up on each
/// build's own integrator (3e-7 apart), and the runtime-settled
/// `dc_operating_point` differs between them too. Restart both with `reset()`
/// (which also invalidates every chord) from the baked `DC_OP`.
#[test]
fn forced_latch_matches_the_backward_euler_build() {
    for (spice, out_name, amp, f, active_set, tag) in [
        (c2("1Meg"), "out", 5.0, F, false, "c2_open"),
        (CHOKE_STAGE.to_string(), "out", 3.0, F, false, "choke"),
        (
            RAILING_CHOKE.to_string(),
            "out",
            1.0,
            1000.0,
            true,
            "railing_choke",
        ),
    ] {
        let out = node(&spice, out_name);
        let run = |backward_euler: bool, sub: &str| -> Vec<f64> {
            let mut config = support::config_for_spice(&spice, FS);
            config.backward_euler = backward_euler;
            if active_set {
                config.opamp_rail_mode = melange_solver::codegen::OpampRailMode::ActiveSet;
            }
            let code = support::generate_circuit_code_nodal(&spice, &config).0;
            let latch = if backward_euler {
                assert!(!code.contains("pub be_latched"), "a BE build has no latch");
                ""
            } else {
                assert!(
                    code.contains("pub be_latched"),
                    "{tag}: the latch must be emitted on a saturating circuit"
                );
                "s.be_latched = true;"
            };
            let restart_nl = if code.contains("pub i_nl_prev: [f64; M]")
                && !code.contains("pub const M: usize = 0;")
            {
                "s.i_nl_prev = DC_NL_I;"
            } else {
                ""
            };
            // Sub-steps are solves (the BE routine's own ladder); what must
            // not occur is a held, unsolved or reset sample.
            let bad = bad_counters(&code)
                .replace("s.diag_be_fallback_count + ", "")
                .replace(" + s.diag_be_fallback_count", "")
                .replace("s.diag_substep_count + ", "")
                .replace(" + s.diag_substep_count", "");
            let start = if code.contains("pub const DC_OP:") {
                "DC_OP"
            } else {
                "[0.0; N]"
            };
            let main = format!(
                "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    s.reset();
    s.v_prev = {start};
    s.input_prev = 0.0;
    {restart_nl}
    {latch}
    let n = (2.0 * {FS:?}) as usize;
    for i in 0..n {{
        let _ = process_sample({amp:?} * (2.0 * std::f64::consts::PI * {f:?} * i as f64 / {FS:?}).sin(), &mut s);
        println!(\"{{:.17e}}\", s.v_prev[{out}]);
    }}
    let bad = {bad};
    println!(\"bad {{}}\", bad);
}}"
            );
            let stdout =
                support::compile_and_run(&code, &main, &format!("latch_{tag}_{sub}")).stdout;
            let mut v = Vec::new();
            for l in stdout.lines() {
                if let Some(b) = l.strip_prefix("bad ") {
                    assert_eq!(b.trim(), "0", "{tag} {sub}: unsolved/held/sub-step samples");
                } else {
                    v.push(l.parse::<f64>().unwrap());
                }
            }
            v
        };
        let latched = run(false, "latched");
        let be = run(true, "be");
        assert_eq!(latched.len(), be.len());
        if let Some(k) = (0..be.len()).find(|&k| latched[k].to_bits() != be[k].to_bits()) {
            panic!(
                "{tag}: forced latch vs BE build first differ at sample {k}: {:e} vs {:e}",
                latched[k], be[k]
            );
        }
    }
}

/// A pot moved while latched: the latched trapezoidal build and a
/// `--backward-euler` build take the same move and stay bit-identical. The
/// latched solve keeps its own chord cache (`chord_be_*`); a knob move (via
/// `rebuild_matrices`) must invalidate it as it does the trapezoidal one, or
/// the BE solve would restart from a factorisation of the old circuit (the
/// sub-step constant-matrix bug in a new place). The diode clipper reuses its
/// chord across samples (no saturating or behavioral stamp forces a refactor),
/// so a stale cache would change its Newton path.
#[test]
fn knob_move_while_latched_matches_the_backward_euler_build() {
    for (spice, amp, tag) in [
        (
            "clip\nR_1 in a 10k\nD1 a 0 D1N\nD2 0 a D1N\nC1 a 0 10n\nR2 a out 1k\nR3 out 0 100k\n\
             .model D1N D(IS=2.52n N=1.752)\n.pot R_1 1k 100k 10k \"Drive\"\n",
            2.0,
            "clipper",
        ),
        (
            "rl\nR_1 in out 99\nL_1 out 0 1 ISAT=10m LAIR=3e-4\n.pot R_1 10 1k 99 \"Drive\"\n",
            5.0,
            "sat_rl",
        ),
    ] {
        let out = node(spice, "out");
        let run = |backward_euler: bool, sub: &str| -> Vec<f64> {
            let mut config = support::config_for_spice(spice, FS);
            config.backward_euler = backward_euler;
            // The full-LU solve is the one under test (the clipper would
            // otherwise take the nodal Schur sub-path).
            config.nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::FullLu;
            let code = support::generate_circuit_code_nodal(spice, &config).0;
            assert!(
                code.contains("state.chord_lu"),
                "{tag}: not a full-LU build"
            );
            let latch = if backward_euler {
                ""
            } else {
                assert!(code.contains("pub be_latched"), "{tag}: latch not emitted");
                "s.be_latched = true;"
            };
            let restart_nl = if code.contains("pub const DC_NL_I:") {
                "s.i_nl_prev = DC_NL_I;"
            } else {
                ""
            };
            let start = if code.contains("pub const DC_OP:") {
                "DC_OP"
            } else {
                "[0.0; N]"
            };
            let main = format!(
                "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    s.reset();
    s.v_prev = {start};
    s.input_prev = 0.0;
    {restart_nl}
    {latch}
    for i in 0..24000usize {{
        if i == 7000 {{ s.set_pot_0(1000.0); }}
        if i == 15000 {{ s.set_pot_0(50000.0); }}
        let _ = process_sample({amp:?} * (2.0 * std::f64::consts::PI * 300.0 * i as f64 / {FS:?}).sin(), &mut s);
        println!(\"{{:.17e}}\", s.v_prev[{out}]);
    }}
}}"
            );
            support::compile_and_run(&code, &main, &format!("knob_latch_{tag}_{sub}"))
                .stdout
                .lines()
                .map(|l| l.parse::<f64>().unwrap())
                .collect()
        };
        let latched = run(false, "latched");
        let be = run(true, "be");
        assert_eq!(latched.len(), be.len());
        if let Some(k) = (0..be.len()).find(|&k| latched[k].to_bits() != be[k].to_bits()) {
            panic!(
                "{tag}: knob move while latched vs BE build first differ at sample {k}: {:e} vs {:e}",
                latched[k], be[k]
            );
        }
    }
}

#[test]
fn c2_shared_core_saturates_on_magnetizing_not_winding_current() {
    // Loaded: primary current far above Isat, core linear.
    let spice = c2("10");
    let code = nodal_code(&spice, "out");
    let (inp, p) = (node(&spice, "in"), node(&spice, "p"));
    let i_pri = format!("(s.v_prev[{inp}] - s.v_prev[{p}]) / 99.0");
    let row = &render(
        &code,
        &[5.0],
        "s.v_prev[OUTPUT_NODES[0]]",
        &i_pri,
        "c2_loaded",
    )[0];
    let (h1, h3, i_pk, bad) = (row[2], row[4], row[7], row[8]);
    assert_eq!(bad, 0.0, "loaded: unsolved samples");
    assert!(
        i_pk / 10e-3 >= 4.0,
        "test premise: i_pri/Isat {:.2}",
        i_pk / 10e-3
    );
    assert!(
        h3 / h1 <= 1e-3,
        "loaded secondary H3/H1 {:.3e}: the core saturated on winding current",
        h3 / h1
    );

    // Open: magnetizing current saturates the core. Independent 1024x
    // reference of R + linear leakage + saturating magnetizing branch:
    // H3/H1 = 0.50460 (0.50513 without the air-core floor), i_mag/Isat = 4.999.
    let spice = c2("1MEG");
    let code = nodal_code(&spice, "out");
    let row = &render(&code, &[5.0], "s.v_prev[OUTPUT_NODES[0]]", "0.0", "c2_open")[0];
    let (h1, h3, bad) = (row[2], row[4], row[8]);
    assert_eq!(bad, 0.0, "open: unsolved samples");
    assert!(
        (h3 / h1 - 0.50460).abs() <= 1e-4,
        "open secondary H3/H1 {:.5} vs reference 0.50460",
        h3 / h1
    );
}

// ─── C3: single-ended DC-biased core — where H2 comes from ─────────────────
//
// A symmetric tanh core under symmetric drive makes odd harmonics only; DC
// bias breaks the symmetry and H2 appears. Flux-drive analysis (analog-EE
// review), with φ0 = tanh(Idc/Isat) and a = AC flux / saturation flux:
//   H2/H1 ≈ φ0·a / (2(1−φ0²)),  H3/H1 ≈ (2 + 6φ0²)·a² / (24(1−φ0²)²),
// so H2 overtakes H3 once φ0 > ~a/6. The exact-FFT values below are ideal
// flux drive of this deck's law (Φ0 = Φ(Idc), AC flux a·L0·Isat), inverted by
// Newton and fitted over one period of 4096 points; with LAIR = 0 the same
// computation reproduces the review's values to 0.05 dB.
//
// The deck has to BE flux drive for those values to apply. The 1 Ω source
// makes that true only when R/(ωL) is small AND L/R is long against the 2 s
// render: with a 1 H core the DC operating point migrates over L/R ≈ 1 s and
// the deepest row lands 1.0 dB off the table, in melange and in the
// continuous-time circuit alike. A 100 H core (R/(ωL) ≈ 5e-5, L/R = 100 s)
// realises it to 0.05 dB on every row. Bias is a DC CURRENT source — a voltage
// source's DC flux would integrate away. Cosine drive switched on at t = 0,
// with `input_prev` set to the drive's t = 0 value, so the flux trajectory is
// Φ0 + a·sin(ωt) with no DC flux of its own. (With `input_prev` = 0 the first
// sample integrates half a step of drive the continuous circuit never sees, a
// DC flux of a·ωT/2 that persists for L/R.) The input node starts at the same
// value, the state the continuous circuit is in at t = 0+. Left at its DC
// value of 0 instead, the node row, which trapezoidal integration enforces
// only as an average over each step, alternates ±a·ωL0·Isat from sample to
// sample for the whole render.

/// (a, [(Idc/Isat, H2/H1 dB, H3/H1 dB)]) from exact flux drive, LAIR = 3e-4.
/// a = 0.3 stops at Idc/Isat = 0.5: at 1.0 the flux would pass saturation
/// (φ0 + a > 1), which is not flux drive any more.
const C3_REF: [(f64, &[(f64, f64, f64)]); 2] = [
    (
        0.1,
        &[
            (0.05, -51.96, -61.43),
            (0.1, -45.90, -61.11),
            (0.25, -37.63, -59.00),
            (0.5, -30.53, -52.96),
            (1.0, -20.39, -36.92),
        ],
    ),
    (
        0.3,
        &[
            (0.05, -41.87, -41.98),
            (0.1, -35.80, -41.62),
            (0.25, -27.45, -39.30),
            (0.5, -19.96, -32.40),
        ],
    ),
];

/// The same recurrence as `c3_reference`, run at 256× (≈continuous) in a
/// standalone release build and fitted on every 256th sample:
/// (a, Idc/Isat, |H1| A, H2/H1, H3/H1). Every row is within 0.05 dB of
/// `C3_REF`; the 1× recurrence is within 1.5e-6 on H1 and 2e-7 on the ratios.
/// Computed 2026-09-28.
const C3_REF_256: [(f64, f64, f64, f64, f64); 11] = [
    (0.1, 0.0, 1.002511812e-3, 2.637954878e-7, 8.372742869e-4),
    (0.1, 0.05, 1.005050611e-3, 2.521991660e-3, 8.477900284e-4),
    (0.1, 0.1, 1.012694651e-3, 5.070132708e-3, 8.798867637e-4),
    (0.1, 0.25, 1.067356841e-3, 1.313523432e-2, 1.121064519e-3),
    (0.1, 0.5, 1.279912075e-3, 2.974821512e-2, 2.247920740e-3),
    (0.1, 1.0, 2.477944612e-3, 9.542253734e-2, 1.420358373e-2),
    (0.3, 0.0, 3.070696935e-3, 2.573443884e-6, 7.855181198e-3),
    (0.3, 0.05, 3.079330282e-3, 8.053291937e-3, 7.963956272e-3),
    (0.3, 0.1, 3.105382663e-3, 1.621137109e-2, 8.295758884e-3),
    (0.3, 0.25, 3.294195808e-3, 4.238394415e-2, 1.083422220e-2),
    (0.3, 0.5, 4.080470766e-3, 1.002508694e-1, 2.391043203e-2),
];

const C3_ISAT: f64 = 10e-3;
const C3_L0: f64 = 100.0;
const C3_LAIR: f64 = 3e-4;

fn c3_deck(idc: f64) -> String {
    format!("biased core\nL1 in 0 {C3_L0:?} ISAT=10m LAIR={C3_LAIR:e}\nI_b 0 in DC {idc:e}\n")
}

/// Drive amplitude giving normalised AC flux `a` at 30 Hz: V = a·ω·L0·Isat.
fn c3_amp(a: f64) -> f64 {
    a * 2.0 * std::f64::consts::PI * F * C3_L0 * C3_ISAT
}

/// Complex harmonics 1..3 of i_L over 1-2 s, from an independent trapezoidal
/// recurrence of this exact circuit: dΦ(i)/dt = v_src − R·(i − Idc), R = 1 Ω,
/// with the drive's previous value starting at its t = 0 value (as the melange
/// run sets `input_prev`).
fn c3_reference(amp: f64, idc: f64) -> [(f64, f64); 4] {
    let (lmag, lair, r, t) = (C3_L0 * (1.0 - C3_LAIR), C3_L0 * C3_LAIR, 1.0f64, 1.0 / FS);
    let phi = |i: f64| lmag * C3_ISAT * (i / C3_ISAT).tanh() + lair * i;
    let n = (2.0 * FS) as usize;
    let (mut i, mut xprev) = (idc, amp);
    let mut ss = Vec::with_capacity(n / 2);
    for k in 1..=n {
        let x = amp * (2.0 * std::f64::consts::PI * F * k as f64 / FS).cos();
        let c = phi(i) + t / 2.0 * (x + xprev - r * (i - idc) + r * idc);
        let (mut lo, mut hi, mut y) = (-10.0f64, 10.0f64, i);
        for _ in 0..300 {
            let f = phi(y) + t / 2.0 * r * y - c;
            if f > 0.0 {
                hi = y
            } else {
                lo = y
            }
            let ch = (y / C3_ISAT).clamp(-300.0, 300.0).cosh();
            let mut yn = y - f / (lmag / (ch * ch) + lair + t / 2.0 * r);
            if !(yn > lo && yn < hi) {
                yn = 0.5 * (lo + hi);
            }
            if (yn - y).abs() <= 1e-16 * yn.abs().max(1e-12) {
                y = yn;
                break;
            }
            y = yn;
        }
        i = y;
        xprev = x;
        if k > n / 2 {
            ss.push(i);
        }
    }
    let mut h = [(0.0, 0.0); 4];
    for (hh, slot) in h.iter_mut().enumerate().skip(1) {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (j, &v) in ss.iter().enumerate() {
            let w = 2.0 * std::f64::consts::PI * hh as f64 * F * j as f64 / FS;
            re += v * w.cos();
            im += v * w.sin();
        }
        *slot = (2.0 * re / ss.len() as f64, 2.0 * im / ss.len() as f64);
    }
    h
}

/// melange's complex harmonics 1..3 of i_L for each `a`, at one bias.
fn c3_melange(idc: f64, amps: &[f64], tag: &str) -> Vec<[(f64, f64); 4]> {
    let spice = c3_deck(idc);
    let code = nodal_code(&spice, "in");
    if idc != 0.0 {
        // The only DC quantity is the inductor current; it must still be baked,
        // or the core starts unbiased and settles over L/R.
        assert!(
            code.contains("pub const DC_OP: "),
            "operating point must be baked"
        );
    }
    let inp = node(&spice, "in");
    let amps: Vec<String> = amps.iter().map(|a| format!("{a:?}")).collect();
    let bad = bad_counters(&code);
    // The drive starts at its t = 0 value. The charge form carries the
    // start's C*dx/dt in `q_dot` (the reference recurrence starts from the
    // same KCL), so it is seeded from KCL at t = 0 with the source at `amp`.
    let rc = if code.contains("pub const RHS_CONST") {
        "RHS_CONST[r]"
    } else {
        "0.0"
    };
    let main = format!(
        "fn main() {{
    for amp in [{amps}] {{
        let mut s = CircuitState::default();
        s.set_sample_rate({FS:?});
        s.input_prev = amp;
        s.v_prev[{inp}] = amp;
        for r in 0..N {{
            if A_NEG_DEFAULT[r].iter().any(|&h| h != 0.0) {{
                let mut q = {rc} + if r == INPUT_NODE {{ amp / INPUT_RESISTANCE }} else {{ 0.0 }};
                for j in 0..N {{ q -= G[r][j] * s.v_prev[j]; }}
                s.q_dot[r] = q;
            }}
        }}
        let n = (2.0 * {FS:?}) as usize;
        let mut ss: Vec<f64> = Vec::with_capacity(n / 2);
        for k in 1..=n {{
            let x = amp * (2.0 * std::f64::consts::PI * {F:?} * k as f64 / {FS:?}).cos();
            let _ = process_sample(x, &mut s);
            if k > n / 2 {{ ss.push(s.v_prev[SAT_IND_0_AUG_ROW]); }}
        }}
        let mut out = String::new();
        for hh in 1..4 {{
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (j, &v) in ss.iter().enumerate() {{
                let w = 2.0 * std::f64::consts::PI * hh as f64 * {F:?} * j as f64 / {FS:?};
                re += v * w.cos();
                im += v * w.sin();
            }}
            out += &format!(\"{{}} {{}} \", 2.0 * re / ss.len() as f64, 2.0 * im / ss.len() as f64);
        }}
        println!(\"{{out}}{{}}\", {bad});
    }}
}}",
        amps = amps.join(", ")
    );
    support::compile_and_run(&code, &main, tag)
        .stdout
        .lines()
        .map(|l| {
            let v: Vec<f64> = l.split_whitespace().map(|t| t.parse().unwrap()).collect();
            assert_eq!(v[6], 0.0, "{tag}: unsolved samples");
            [(0.0, 0.0), (v[0], v[1]), (v[2], v[3]), (v[4], v[5])]
        })
        .collect()
}

fn mag(z: (f64, f64)) -> f64 {
    z.0.hypot(z.1)
}

fn db(x: f64) -> f64 {
    20.0 * x.log10()
}

/// Two gates per (a, bias) row. Implementation: melange against the same
/// circuit integrated independently, at 1× (same discretisation) and at 256×
/// (≈continuous), at C1's tolerances. Physics: the review's exact flux-drive
/// values for this law, to 0.5 dB.
///
/// This row set is what caught the NR stopping criterion accepting Newton's
/// first iterate. The quadratic remainder is one-signed under bias, so the
/// flux integrates it for L/R: up to 0.7 % on H1 at 2 s under the old
/// 1e-3-of-increment tolerance, about 1e-7 at 1e-5.
#[test]
fn c3_dc_bias_makes_h2_as_the_physics_predicts() {
    let mut biases: Vec<f64> = Vec::new();
    for r in C3_REF_256.iter() {
        if !biases.contains(&r.1) {
            biases.push(r.1);
        }
    }
    for &b in &biases {
        let rows: Vec<_> = C3_REF_256.iter().filter(|r| r.1 == b).collect();
        let amps: Vec<f64> = rows.iter().map(|r| c3_amp(r.0)).collect();
        let idc = b * C3_ISAT;
        let got = c3_melange(idc, &amps, &format!("c3_{}", (b * 1000.0).round() as i64));
        for ((&&(a, _, h1_256, h2_256, h3_256), h), &amp) in rows.iter().zip(got).zip(&amps) {
            let rf = c3_reference(amp, idc);
            let h1 = mag(h[1]);
            let (h2r, h3r) = (mag(h[2]) / h1, mag(h[3]) / h1);
            // Implementation, 1×: the same discretisation, so tight.
            assert!(
                ((h1 - mag(rf[1])) / mag(rf[1])).abs() <= 1e-5,
                "a={a} Idc/Isat={b}: H1 {h1:.6e} vs 1x recurrence {:.6e}",
                mag(rf[1])
            );
            for (k, m) in [(2, h2r), (3, h3r)] {
                let r = mag(rf[k]) / mag(rf[1]);
                assert!(
                    (m - r).abs() <= 1e-3 * r + 1e-9,
                    "a={a} Idc/Isat={b}: H{k}/H1 {m:.4e} vs 1x recurrence {r:.4e}"
                );
            }
            // Implementation, 256×: C1's gates.
            assert!(
                ((h1 - h1_256) / h1_256).abs() <= 1e-5,
                "a={a} Idc/Isat={b}: H1 {h1:.6e} vs 256x {h1_256:.6e}"
            );
            assert!(
                (h2r - h2_256).abs() <= 1e-4 && (h3r - h3_256).abs() <= 1e-4,
                "a={a} Idc/Isat={b}: H2/H1 {h2r:.4e} H3/H1 {h3r:.4e} vs 256x {h2_256:.4e} / {h3_256:.4e}"
            );
            // Physics: exact flux-drive values, to 0.5 dB.
            let table = C3_REF.iter().find(|r| r.0 == a).map(|r| r.1).unwrap();
            if let Some(&(_, h2_db, h3_db)) = table.iter().find(|r| r.0 == b) {
                let (h2, h3) = (db(h2r), db(h3r));
                assert!(
                    (h2 - h2_db).abs() <= 0.5 && (h3 - h3_db).abs() <= 0.5,
                    "a={a} Idc/Isat={b}: H2 {h2:.1} dB H3 {h3:.1} dB vs {h2_db} / {h3_db}"
                );
            }
        }
    }
}

#[test]
fn c3_h2_flips_with_bias_sign_and_crosses_h3_near_a_over_6() {
    let a = 0.3;
    let amp = c3_amp(a);
    // Relative H2 phase: arg(H2) − 2·arg(H1) is invariant to the time origin.
    let rel = |h: [(f64, f64); 4]| h[2].1.atan2(h[2].0) - 2.0 * h[1].1.atan2(h[1].0);
    let pos = c3_melange(0.25 * C3_ISAT, &[amp], "c3_pos")[0];
    let neg = c3_melange(-0.25 * C3_ISAT, &[amp], "c3_neg")[0];
    assert!(
        (rel(pos) - rel(neg)).cos() < -0.999,
        "H2 must flip sign with the bias: relative phases {:.3} / {:.3} rad",
        rel(pos),
        rel(neg)
    );
    // Magnitudes match to the start-up transient: the drive starts at the same
    // phase for either bias, so the R-driven transient is not an exact mirror
    // (measured 1.7e-4).
    let (mp, mn) = (mag(pos[2]), mag(neg[2]));
    assert!(
        (mp - mn).abs() <= 1e-3 * mp,
        "|H2| {mp:.4e} vs {mn:.4e} under ± bias"
    );
    // Unbiased: no H2, H3 dominates. At φ0 ≈ 0.1 (> a/6 = 0.05): H2 dominates.
    let zero = c3_melange(0.0, &[amp], "c3_zero")[0];
    assert!(
        mag(zero[2]) < mag(zero[3]),
        "unbiased core must be H3-dominated"
    );
    let biased = c3_melange(0.1 * C3_ISAT, &[amp], "c3_tenth")[0];
    assert!(
        mag(biased[2]) > mag(biased[3]),
        "φ0 ≈ 0.1 > a/6: H2 must dominate"
    );
}
