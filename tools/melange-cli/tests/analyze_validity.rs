//! `melange analyze` measures solutions, at steady state, over a stated band.
//!
//! - A point whose render was not a solution (here: a `.linearize`d triode
//!   driven into cutoff, every exit a `reduced_model_exit_count` sample) is
//!   refused, naming the point and the counter, the way `simulate` refuses
//!   the render. It used to print a THD for it with no warning.
//! - Each point is measured at steady state at its own drive. The sweep used
//!   to pre-roll one 10-cycle window per point, so on a circuit with a slow
//!   bias time constant the reading was the sweep's history.
//! - `thd_pct` sums harmonics below 20 kHz only.

use std::process::{Command, Output};

/// A cathode follower at ~25 uA idle, linearized: -10 V swings take the linear
/// plate current below zero (cutoff), outside the reduced model's region.
const FOLLOWER: &str = "\
linearized cathode follower
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

/// A diode clamp behind a coupling cap: the cap charges through 10k (10 ms)
/// and bleeds through 100k (110 ms), so the clamp's DC shift, and with it the
/// distortion, takes many 10-cycle windows at 1 kHz to settle.
const CLAMP: &str = "\
slow clamp
Rs in x 10k
C1 x a 1u
R1 a 0 100k
D1 a 0 DX
R2 a out 1k
C2 out 0 1n
.model DX D(IS=1e-14 N=1)
";

struct Scratch {
    dir: tempfile::TempDir,
}

impl Scratch {
    fn new() -> Self {
        let dir = tempfile::Builder::new()
            .prefix("melange_analyze_validity_")
            .tempdir()
            .unwrap();
        std::fs::write(dir.path().join("follower.cir"), FOLLOWER).unwrap();
        std::fs::write(dir.path().join("clamp.cir"), CLAMP).unwrap();
        Scratch { dir }
    }

    fn deck(&self, name: &str) -> String {
        self.dir.path().join(name).to_str().unwrap().to_string()
    }

    fn analyze(&self, deck: &str, extra: &[&str]) -> Output {
        let deck = self.deck(deck);
        let mut args = vec!["analyze", deck.as_str()];
        args.extend_from_slice(extra);
        Command::new(env!("CARGO_BIN_EXE_melange"))
            .args(&args)
            .output()
            .expect("run melange")
    }
}

fn stderr(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

/// CSV data rows as (frequency, named columns).
fn rows(out: &Output) -> Vec<Vec<(String, String)>> {
    let stdout = String::from_utf8_lossy(&out.stdout);
    let mut lines = stdout.lines();
    let Some(header) = lines.next() else {
        return Vec::new();
    };
    let names: Vec<&str> = header.split(',').collect();
    lines
        .map(|l| {
            names
                .iter()
                .zip(l.split(','))
                .map(|(n, v)| (n.to_string(), v.to_string()))
                .collect()
        })
        .collect()
}

fn col(row: &[(String, String)], name: &str) -> f64 {
    let v = &row
        .iter()
        .find(|(n, _)| n == name)
        .unwrap_or_else(|| panic!("no column {name}"))
        .1;
    if v == "nan" {
        f64::NAN
    } else {
        v.parse()
            .unwrap_or_else(|_| panic!("unparseable {name} {v:?}"))
    }
}

/// Two points at 1 kHz, five harmonics.
const AT_1K: [&str; 8] = [
    "--start-freq",
    "1000",
    "--end-freq",
    "1000.001",
    "--points-per-decade",
    "1",
    "--harmonics",
    "5",
];

#[test]
fn a_point_solved_on_a_reduced_model_outside_its_region_is_refused() {
    let s = Scratch::new();
    let mut args = vec!["--amplitude", "10"];
    args.extend_from_slice(&AT_1K);
    let out = s.analyze("follower.cir", &args);
    let err = stderr(&out);
    assert!(!out.status.success(), "must refuse:\n{err}");
    assert!(
        err.contains("1000.00 Hz")
            && err.contains("reduced_model_exit_count")
            && err.contains("not a solution")
            && err.contains("--allow-nr-hold"),
        "the refusal must name the point, the counter and the override:\n{err}"
    );
    assert!(
        rows(&out).is_empty(),
        "a refused sweep must not print a CSV to be consumed"
    );

    args.push("--allow-nr-hold");
    let out = s.analyze("follower.cir", &args);
    assert!(out.status.success(), "{}", stderr(&out));
    assert_eq!(rows(&out).len(), 2, "the override reports the points");
}

#[test]
fn a_healthy_drive_still_measures() {
    let s = Scratch::new();
    let mut args = vec!["--amplitude", "0.1"];
    args.extend_from_slice(&AT_1K);
    let out = s.analyze("follower.cir", &args);
    let err = stderr(&out);
    assert!(out.status.success(), "{err}");
    assert!(!err.contains("not a solution"), "{err}");
    let r = rows(&out);
    assert_eq!(r.len(), 2);
    // A cathode follower: just under unity gain.
    let g = col(&r[0], "gain_db");
    assert!((-2.0..0.0).contains(&g), "gain {g} dB");
}

#[test]
fn each_point_is_measured_at_steady_state() {
    let s = Scratch::new();
    let thd = |extra: &[&str]| -> (f64, f64) {
        let mut args = vec!["--amplitude", "2"];
        args.extend_from_slice(&AT_1K);
        args.extend_from_slice(extra);
        let out = s.analyze("clamp.cir", &args);
        assert!(out.status.success(), "{}", stderr(&out));
        let r = rows(&out);
        (col(&r[0], "thd_pct"), col(&r[1], "thd_pct"))
    };
    let rel = |a: f64, b: f64| (a - b).abs() / b.abs();

    // Default: the first point (right after the zero-drive settle) and the
    // second (after a full point of drive) read the same steady state...
    let (first, second) = thd(&[]);
    assert!(rel(first, second) < 2e-3, "{first} vs {second}");
    // ...which is the one a long fixed pre-roll reaches.
    let (long, _) = thd(&["--preroll-secs", "2", "--preroll-max-secs", "0"]);
    assert!(rel(first, long) < 2e-3, "{first} vs 2 s pre-roll {long}");

    // The witness bites: one window of pre-roll (the old behaviour) reads
    // the transient, and the two points disagree.
    let (old_first, old_second) = thd(&["--preroll-secs", "0.02", "--preroll-max-secs", "0"]);
    assert!(
        rel(old_first, long) > 0.5 && rel(old_first, old_second) > 0.1,
        "one-window pre-roll should be far from steady state on this deck: \
         {old_first} / {old_second} vs {long}"
    );
}

#[test]
fn a_point_that_never_settles_is_named() {
    let s = Scratch::new();
    let mut args = vec![
        "--amplitude",
        "2",
        "--preroll-secs",
        "0.001",
        "--preroll-max-secs",
        "0.002",
    ];
    args.extend_from_slice(&AT_1K);
    let out = s.analyze("clamp.cir", &args);
    let err = stderr(&out);
    assert!(
        out.status.success(),
        "an unsettled point warns, it does not fail:\n{err}"
    );
    assert!(
        err.contains("WARNING") && err.contains("did not reach steady state"),
        "{err}"
    );
    assert!(
        err.contains("1000.00 Hz"),
        "the warning names the point:\n{err}"
    );
}

#[test]
fn preroll_cap_below_the_preroll_is_refused() {
    let s = Scratch::new();
    let out = s.analyze(
        "clamp.cir",
        &["--preroll-secs", "0.5", "--preroll-max-secs", "0.1"],
    );
    assert!(!out.status.success());
    assert!(
        stderr(&out).contains("--preroll-max-secs"),
        "{}",
        stderr(&out)
    );
}

/// `thd_pct` sums harmonics below 20 kHz only: at 8 kHz that is H2 alone,
/// and at 12 kHz there is no harmonic in band, so no THD.
#[test]
fn thd_band_stops_at_20_khz() {
    let s = Scratch::new();
    let out = s.analyze(
        "clamp.cir",
        &[
            "--amplitude",
            "2",
            "--start-freq",
            "8000",
            "--end-freq",
            "12000",
            "--points-per-decade",
            "1",
            "--harmonics",
            "5",
        ],
    );
    let err = stderr(&out);
    assert!(out.status.success(), "{err}");
    assert!(err.contains("below 20 kHz"), "band stated: {err}");
    let r = rows(&out);
    assert_eq!(r.len(), 2);
    let (f8, f12) = (col(&r[0], "frequency_hz"), col(&r[1], "frequency_hz"));
    assert!((f8 - 8000.0).abs() < 1.0 && (f12 - 12000.0).abs() < 1.0);
    // H3 at 24 kHz is measured but outside the band.
    assert!(col(&r[0], "h3_dbc").is_finite());
    let h2_only = 100.0 * 10f64.powf(col(&r[0], "h2_dbc") / 20.0);
    let thd8 = col(&r[0], "thd_pct");
    assert!(
        (thd8 - h2_only).abs() < 1e-3 * h2_only + 1e-3,
        "THD at 8 kHz must be H2 alone: {thd8} vs {h2_only}"
    );
    assert!(
        col(&r[1], "thd_pct").is_nan(),
        "no in-band harmonic at 12 kHz"
    );
}
