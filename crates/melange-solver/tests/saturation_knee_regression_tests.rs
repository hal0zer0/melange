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
//!   `Φ(i) = L0·Isat·tanh(i/Isat)`, solved by bracketed Newton to 1e-16, at the
//!   same 48 kHz (checks the implementation), and
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

/// Saturating RL: R = 99 + the 1 Ω input = 100 Ω, L0 = 1 H, Isat = 10 mA.
const C1: &str = "saturating RL driven into the knee at LF\nR1 in a 99\nL1 a 0 1 ISAT=10m\n";

/// (drive V, pk/Isat, H1 A [1x], H3/H1 [1x], H5/H1 [1x], H1 A [1024x], H3/H1 [1024x], H5/H1 [1024x])
const C1_REF: [(f64, f64, f64, f64, f64, f64, f64, f64); 3] = [
    (
        0.5,
        0.2380,
        2.368764e-3,
        0.004600,
        0.000039,
        2.368766e-3,
        0.004600,
        0.000039,
    ),
    (
        2.0,
        1.2888,
        1.146205e-2,
        0.103742,
        0.020109,
        1.146206e-2,
        0.103742,
        0.020109,
    ),
    (
        5.0,
        5.0000,
        4.079957e-2,
        0.272241,
        0.110613,
        4.079984e-2,
        0.272238,
        0.110613,
    ),
];

/// 1:1 shared-core transformer, 1 H windings, K = 0.99, ISAT on the primary.
fn c2(load: &str) -> String {
    format!(
        "shared-core saturating transformer\nR_p in p 99\nL_pri p 0 1 ISAT=10m\nL_sec s 0 1\n\
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

/// C1's gates against both references. `Err` names the first gate that fails.
fn c1_gates(code: &str, tag: &str) -> Result<(), String> {
    let drives: Vec<f64> = C1_REF.iter().map(|r| r.0).collect();
    let rows = render(code, &drives, "s.v_prev[SAT_IND_0_AUG_ROW]", "0.0", tag);
    let isat = 10e-3;
    for (row, r) in rows.iter().zip(C1_REF.iter()) {
        let (amp, pk, h1, h2, h3, h5, bad) =
            (row[0], row[1], row[2], row[3], row[4], row[6], row[8]);
        if bad != 0.0 {
            return Err(format!(
                "{amp} V: {bad} unsolved / sub-step / BE / reset samples"
            ));
        }
        let (h3r, h5r) = (h3 / h1, h5 / h1);
        for (name, h1_ref, h3_ref, h5_ref) in [("1x", r.2, r.3, r.4), ("1024x", r.5, r.6, r.7)] {
            if ((h1 - h1_ref) / h1_ref).abs() > 1e-5 {
                return Err(format!("{amp} V: H1 {h1:.6e} vs {name} {h1_ref:.6e}"));
            }
            if (h3r - h3_ref).abs() > 1e-4 || (h5r - h5_ref).abs() > 1e-4 {
                return Err(format!(
                    "{amp} V: H3/H1 {h3r:.6} H5/H1 {h5r:.6} vs {name} {h3_ref:.6} / {h5_ref:.6}"
                ));
            }
        }
        if (pk / isat - r.1).abs() > 1e-3 * r.1 {
            return Err(format!("{amp} V: peak {:.4}·Isat vs {:.4}", pk / isat, r.1));
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
    c1_gates(&code, "c1").unwrap();
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
        c1_gates(&no_history, "c1_nohist").is_err(),
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
        c1_gates(&no_jacobian, "c1_nojac").is_err(),
        "Jacobian mutant passed"
    );
    // Linear inductor: no knee at all. Same topology, so its branch current is
    // on the augmented row the saturating build names SAT_IND_0_AUG_ROW (2).
    let linear = nodal_code("linear RL\nR1 in a 99\nL1 a 0 1\n", "a")
        + "\npub const SAT_IND_0_AUG_ROW: usize = 2;\n";
    assert!(
        c1_gates(&linear, "c1_lin").is_err(),
        "linear inductor passed"
    );
}

/// Every Newton site that can commit a sample (main loop, adaptive sub-step,
/// backward-Euler fallback) checks the flux row the same way, so a wrong flux
/// Jacobian is caught wherever it is. Deleting the Jacobian correction while
/// keeping the companion moves the Newton fixed point off the solution; the
/// step check alone accepts it. Deleted at one site, the next site down the
/// ladder recovers the right answer; deleted everywhere, every sample is
/// counted as held. With a site's residual removed as well, that site accepts
/// the wrong point and nothing is counted, which is what shows the residual
/// is the detector.
#[test]
fn c1_jacobian_deletion_is_caught_at_every_newton_site() {
    let code = nodal_code(C1, "a");
    let jac = |site: &str| format!("{site}[SAT_IND_0_AUG_ROW][SAT_IND_0_AUG_ROW] +=");
    let (main, sub, be) = (jac("chord_lu"), jac("g_s"), jac("g_aug"));
    let flag = |f: &str| format!("{f} = true; }}");
    let mutate = |dels: &[&String], blind: Option<String>| -> String {
        let mut hits = vec![0usize; dels.len()];
        let mut blinded = 0usize;
        let out: Vec<&str> = code
            .lines()
            .filter(|l| {
                for (h, d) in hits.iter_mut().zip(dels) {
                    if l.contains(d.as_str()) {
                        *h += 1;
                        return false;
                    }
                }
                if let Some(f) = &blind {
                    if l.contains("acc.abs()") && l.contains(f.as_str()) {
                        blinded += 1;
                        return false;
                    }
                }
                true
            })
            .collect();
        assert!(
            hits.iter().all(|&h| h == 1),
            "test premise: one stamp per site, got {hits:?}"
        );
        assert!(
            blind.is_none() || blinded == 1,
            "test premise: the site's residual line"
        );
        out.join("\n")
    };
    // (peak i_L / Isat, sub-steps, BE fallbacks, holds) over 1-2 s at 5 V.
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

    let (pk, subs, _, holds) = run(&mutate(&[&main], None), "c1_jac_main");
    assert!(
        subs > 1000 && holds == 0 && right(pk),
        "main: pk {pk} sub {subs} hold {holds}"
    );

    let (pk, _, bes, holds) = run(&mutate(&[&main, &sub], None), "c1_jac_sub");
    assert!(
        bes > 1000 && holds == 0 && right(pk),
        "sub-step: pk {pk} be {bes} hold {holds}"
    );
    let (pk, _, _, holds) = run(
        &mutate(&[&main, &sub], Some(flag("sub_step_exceeded"))),
        "c1_jac_sub_blind",
    );
    assert!(
        holds == 0 && !right(pk),
        "sub-step without its residual must accept the wrong point: pk {pk}"
    );

    let (_, _, _, holds) = run(&mutate(&[&main, &sub, &be], None), "c1_jac_all");
    assert!(holds > 1000, "all sites: only {holds} held samples");
    let (pk, _, _, holds) = run(
        &mutate(&[&main, &sub, &be], Some(flag("be_step_exceeded"))),
        "c1_jac_be_blind",
    );
    assert!(
        holds == 0 && !right(pk),
        "BE without its residual must accept the wrong point: pk {pk}"
    );
}

/// Deep saturation rings under the trapezoidal rule: as L_diff → 0 the RL
/// step factor tends to −1, and the current overshoots the physical ceiling
/// V/R. Same scalar recurrence as C1's reference: at 1× the peak is 11.83 and
/// 23.94 Isat at 10 and 20 V against a ceiling of 10 and 20, and 4× does not
/// cure it (10.49, 24.69). The 1024× recurrence lands on the ceiling.
///
/// melange closes this with the runtime BE-latch, which detects the
/// sample-to-sample alternation and switches the instance to backward Euler.
/// The cost is first-order accuracy for the REST of the stream, because the
/// latch is sticky: here H1 moves 1.8e-4 (10 V) and 6.8e-5 (20 V) relative.
///
/// (drive V, 1024× peak/Isat, 1024× H1 A)
const C1_DEEP: [(f64, f64, f64); 2] = [(10.0, 10.0000, 9.162930e-2), (20.0, 20.0000, 1.930403e-1)];

#[test]
fn c1_deep_saturation_ring_is_caught_by_the_latch() {
    let code = nodal_code(C1, "a");
    let drives: Vec<String> = C1_DEEP.iter().map(|r| format!("{:?}", r.0)).collect();
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
    let out = support::compile_and_run(&code, &main, "c1_deep").stdout;
    for (line, &(amp, pk_ref, h1_ref)) in out.lines().zip(C1_DEEP.iter()) {
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
Lp   vcc    drain  5 ISAT=20m
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
/// (the flux law is nonlinear on its own), and once latched the trapezoidal
/// build runs every sample through its BE fallback. That fallback must be the
/// backward-Euler solution: compared with a `--backward-euler` build of the
/// same circuit it agrees to 1e-9 relative on the output (measured 2e-10 and
/// 3e-11).
///
/// Harness trap, worth keeping: `CircuitState::default()` runs a 50-sample
/// warmup, on the trapezoidal rule in the latched build (the latch is not set
/// yet) and on BE in the BE build. That alone put them 3e-7 apart. Either set
/// `be_latched` before the warmup or restart both from the baked operating
/// point, as here.
#[test]
fn forced_latch_matches_the_backward_euler_build() {
    for (spice, out_name, amp, tag) in [
        (c2("1Meg"), "out", 5.0, "c2_open"),
        (CHOKE_STAGE.to_string(), "out", 3.0, "choke"),
    ] {
        let out = node(&spice, out_name);
        let run = |backward_euler: bool, sub: &str| -> Vec<f64> {
            let mut config = support::config_for_spice(&spice, FS);
            config.backward_euler = backward_euler;
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
            let bad = bad_counters(&code)
                .replace("s.diag_be_fallback_count + ", "")
                .replace(" + s.diag_be_fallback_count", "");
            let main = format!(
                "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    s.v_prev = s.dc_operating_point;
    s.input_prev = 0.0;
    {restart_nl}
    {latch}
    let n = (2.0 * {FS:?}) as usize;
    for i in 0..n {{
        let _ = process_sample({amp:?} * (2.0 * std::f64::consts::PI * {F:?} * i as f64 / {FS:?}).sin(), &mut s);
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
        let peak = be.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        let diff = latched
            .iter()
            .zip(&be)
            .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));
        assert!(
            diff <= 1e-9 * peak,
            "{tag}: forced latch vs BE build differ by {diff:.3e} on a {peak:.3e} output ({:.2e} relative)",
            diff / peak
        );
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
    // H3/H1 = 0.50513, i_mag/Isat = 4.999.
    let spice = c2("1MEG");
    let code = nodal_code(&spice, "out");
    let row = &render(&code, &[5.0], "s.v_prev[OUTPUT_NODES[0]]", "0.0", "c2_open")[0];
    let (h1, h3, bad) = (row[2], row[4], row[8]);
    assert_eq!(bad, 0.0, "open: unsolved samples");
    assert!(
        (h3 / h1 - 0.50513).abs() <= 1e-3,
        "open secondary H3/H1 {:.5} vs reference 0.50513",
        h3 / h1
    );
}

// ─── C3: single-ended DC-biased core — where H2 comes from ─────────────────
//
// A symmetric tanh core under symmetric drive makes odd harmonics only; DC
// bias breaks the symmetry and H2 appears. Flux-drive analysis (analog-EE
// review), with φ0 = tanh(Idc/Isat) and a = AC flux / saturation flux:
//   H2/H1 ≈ φ0·a / (2(1−φ0²)),  H3/H1 ≈ (2 + 6φ0²)·a² / (24(1−φ0²)²),
// so H2 overtakes H3 once φ0 > ~a/6. The exact-FFT values below are the
// review's (reproduced to 0.05 dB by ideal flux drive, φ0 = tanh(Idc/Isat)).
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

/// (a, [(Idc/Isat, H2/H1 dB, H3/H1 dB)]) from the review's exact FFT.
/// a = 0.3 stops at Idc/Isat = 0.5: at 1.0 the flux would pass saturation
/// (φ0 + a > 1), which is not flux drive any more.
const C3_REF: [(f64, &[(f64, f64, f64)]); 2] = [
    (
        0.1,
        &[
            (0.05, -52.0, -61.4),
            (0.1, -45.9, -61.1),
            (0.25, -37.6, -59.0),
            (0.5, -30.5, -52.9),
            (1.0, -20.4, -36.9),
        ],
    ),
    (
        0.3,
        &[
            (0.05, -41.9, -42.0),
            (0.1, -35.8, -41.6),
            (0.25, -27.4, -39.3),
            (0.5, -20.0, -32.4),
        ],
    ),
];

/// The same recurrence as `c3_reference`, run at 256× (≈continuous) in a
/// standalone release build and fitted on every 256th sample:
/// (a, Idc/Isat, |H1| A, H2/H1, H3/H1). Every row is within 0.05 dB of
/// `C3_REF`; the 1× recurrence is within 1.5e-6 on H1 and 2e-7 on the ratios.
/// Computed 2026-09-28.
const C3_REF_256: [(f64, f64, f64, f64, f64); 11] = [
    (0.1, 0.0, 1.002512572e-3, 2.638759463e-7, 8.375280792e-4),
    (0.1, 0.05, 1.005052155e-3, 2.522763217e-3, 8.480528588e-4),
    (0.1, 0.1, 1.012698578e-3, 5.071707744e-3, 8.801774832e-4),
    (0.1, 0.25, 1.067378915e-3, 1.313975917e-2, 1.121583060e-3),
    (0.1, 0.5, 1.280023649e-3, 2.976242889e-2, 2.249874204e-3),
    (0.1, 1.0, 2.479242787e-3, 9.554806116e-2, 1.424389503e-2),
    (0.3, 0.0, 3.070719883e-3, 2.574341870e-6, 7.857769168e-3),
    (0.3, 0.05, 3.079356488e-3, 8.056066379e-3, 7.966648415e-3),
    (0.3, 0.1, 3.105418864e-3, 1.621706447e-2, 8.298774374e-3),
    (0.3, 0.25, 3.294311977e-3, 4.240093859e-2, 1.083999947e-2),
    (0.3, 0.5, 4.081094389e-3, 1.003148903e-1, 2.393806226e-2),
];

const C3_ISAT: f64 = 10e-3;
const C3_L0: f64 = 100.0;

fn c3_deck(idc: f64) -> String {
    format!("biased core\nL1 in 0 {C3_L0:?} ISAT=10m\nI_b 0 in DC {idc:e}\n")
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
    let (l0, r, t) = (C3_L0, 1.0f64, 1.0 / FS);
    let phi = |i: f64| l0 * C3_ISAT * (i / C3_ISAT).tanh();
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
            let mut yn = y - f / (l0 / (ch * ch) + t / 2.0 * r);
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
    let main = format!(
        "fn main() {{
    for amp in [{amps}] {{
        let mut s = CircuitState::default();
        s.set_sample_rate({FS:?});
        s.input_prev = amp;
        s.v_prev[{inp}] = amp;
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
/// values, to 0.5 dB.
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
            // Physics: the review's exact flux-drive values, to 0.5 dB.
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
