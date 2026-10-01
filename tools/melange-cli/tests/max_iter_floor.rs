//! `--max-iter` against the nodal Newton budget floor.
//!
//! A nodal build never ships fewer than `NODAL_MAX_ITER_FLOOR` (100) Newton
//! iterations per sample. A pin below it used to be raised silently: the code
//! and the provenance `Build:` line said 100 while `-v` printed the pin. Now
//! every verb refuses such a pin on a nodal build, and the console prints the
//! budget the code carries.

use std::process::{Command, Output};

const FLOOR: usize = melange_solver::codegen::policy::NODAL_MAX_ITER_FLOOR;

const CLIPPER: &str = "\
Diode clipper
R1 in out 1k
D1 out 0 D1N4148
D2 0 out D1N4148
C1 out 0 10n
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

struct Scratch {
    dir: tempfile::TempDir,
}

impl Scratch {
    fn new() -> Self {
        let dir = tempfile::Builder::new()
            .prefix("melange_max_iter_floor_")
            .tempdir()
            .unwrap();
        std::fs::write(dir.path().join("clipper.cir"), CLIPPER).unwrap();
        Scratch { dir }
    }

    fn path(&self, name: &str) -> String {
        self.dir.path().join(name).to_str().unwrap().to_string()
    }

    fn melange(&self, args: &[&str]) -> Output {
        Command::new(env!("CARGO_BIN_EXE_melange"))
            .args(args)
            .output()
            .expect("run melange")
    }

    /// `compile -v`; returns (stdout, generated code).
    fn compile(&self, extra: &[&str]) -> (Output, String) {
        let out_rs = self.path("out.rs");
        let _ = std::fs::remove_file(&out_rs);
        let deck = self.path("clipper.cir");
        let mut args = vec!["compile", deck.as_str(), "-o", out_rs.as_str(), "-v"];
        args.extend_from_slice(extra);
        let out = self.melange(&args);
        let code = std::fs::read_to_string(&out_rs).unwrap_or_default();
        (out, code)
    }
}

fn text(out: &Output) -> String {
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

/// `MAX_ITER` as the generated code declares it.
fn emitted_max_iter(code: &str) -> usize {
    code.lines()
        .find_map(|l| {
            l.trim()
                .strip_prefix("pub const MAX_ITER: usize = ")
                .and_then(|r| r.strip_suffix(';'))
        })
        .unwrap_or_else(|| panic!("no `pub const MAX_ITER` in:\n{code}"))
        .parse()
        .unwrap()
}

fn assert_refused_naming_the_floor(out: &Output, verb: &str) {
    let all = text(out);
    assert!(
        !out.status.success(),
        "{verb}: a sub-floor pin must fail:\n{all}"
    );
    assert!(
        all.contains("--max-iter 50")
            && all.contains(&format!("floor of {FLOOR}"))
            && all.contains("Armijo line search"),
        "{verb}: the refusal must name the pin, the floor and why:\n{all}"
    );
}

#[test]
fn nodal_compile_refuses_a_pin_below_the_floor() {
    let s = Scratch::new();
    let (out, _) = s.compile(&["--solver", "nodal", "--max-iter", "50"]);
    assert_refused_naming_the_floor(&out, "compile");
}

#[test]
fn nodal_simulate_and_analyze_refuse_a_pin_below_the_floor() {
    let s = Scratch::new();
    let deck = s.path("clipper.cir");
    let wav = s.path("out.wav");
    let sim = s.melange(&[
        "simulate",
        &deck,
        "-o",
        &wav,
        "-d",
        "0.01",
        "--solver",
        "nodal",
        "--max-iter",
        "50",
    ]);
    assert_refused_naming_the_floor(&sim, "simulate");
    let ana = s.melange(&[
        "analyze",
        &deck,
        "--solver",
        "nodal",
        "--max-iter",
        "50",
        "--points-per-decade",
        "1",
    ]);
    assert_refused_naming_the_floor(&ana, "analyze");
}

#[test]
fn nodal_compile_ships_a_pin_above_the_floor_and_prints_it() {
    let s = Scratch::new();
    let (out, code) = s.compile(&["--solver", "nodal", "--max-iter", "150"]);
    let all = text(&out);
    assert!(out.status.success(), "{all}");
    assert!(
        all.contains("Max NR iterations: 150 (--max-iter)"),
        "the console must print the pinned budget:\n{all}"
    );
    assert_eq!(emitted_max_iter(&code), 150);
    assert!(code.contains("max_iter=150,"), "provenance Build: line");
}

#[test]
fn dk_compile_ships_a_pin_below_the_nodal_floor_and_prints_it() {
    let s = Scratch::new();
    let (out, code) = s.compile(&["--solver", "dk", "--max-iter", "50"]);
    let all = text(&out);
    assert!(out.status.success(), "DK has no floor:\n{all}");
    assert!(
        all.contains("Max NR iterations: 50 (--max-iter)"),
        "the console must print the budget that ships:\n{all}"
    );
    assert_eq!(emitted_max_iter(&code), 50);
    assert!(code.contains("max_iter=50,"), "provenance Build: line");
}

/// Unpinned, the console prints the budget the code carries (the auto-tuned
/// one, raised to the floor on nodal), never the pre-floor figure.
#[test]
fn auto_budget_console_matches_the_code() {
    let s = Scratch::new();
    for solver in ["dk", "nodal"] {
        let (out, code) = s.compile(&["--solver", solver]);
        let all = text(&out);
        assert!(out.status.success(), "{all}");
        let shipped = emitted_max_iter(&code);
        assert!(
            all.contains(&format!("Max NR iterations: {shipped} (auto-tuned")),
            "--solver {solver}: the console must print the shipped {shipped}:\n{all}"
        );
        assert!(code.contains(&format!("max_iter={shipped},")));
        if solver == "nodal" {
            assert!(shipped >= FLOOR);
        }
    }
}
