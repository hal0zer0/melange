//! `melange dc-op` reports the operating point the build ships.
//!
//! The verb assembled its own circuit (its own MNA, input stamp and junction
//! caps), so it left out whatever else the build does: `.inject` impedances,
//! reductions, the route's internal-node expansion. It now goes through the
//! one build (`melange_solver::build::assemble`), and every value it prints is
//! the one `compile` embeds as `DC_OP` for the same flags.

use std::collections::HashMap;
use std::process::Command;

fn write_deck(tag: &str, deck: &str) -> std::path::PathBuf {
    let path = std::env::temp_dir().join(format!(
        "melange_dc_op_build_{tag}_{}.cir",
        std::process::id()
    ));
    std::fs::write(&path, deck).unwrap();
    path
}

fn melange(args: &[&str], deck: &std::path::Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_melange"))
        .args(args)
        .arg(deck)
        .output()
        .expect("run melange")
}

/// `melange dc-op --format json`, parsed.
fn dc_op_json(deck: &std::path::Path, extra: &[&str]) -> serde_json::Value {
    let mut args = vec!["dc-op", "--format", "json"];
    args.extend_from_slice(extra);
    let out = melange(&args, deck);
    assert!(
        out.status.success(),
        "dc-op failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let json = stdout
        .lines()
        .find(|l| l.trim_start().starts_with('{'))
        .expect("a JSON line on stdout");
    serde_json::from_str(json).expect("dc-op JSON")
}

/// Node name → `DC_OP` value of `melange compile`'s generated code.
fn compiled_dc_op(deck: &std::path::Path, extra: &[&str]) -> HashMap<String, f64> {
    let rs = deck.with_extension("rs");
    let mut args = vec!["compile", "--format", "code", "-o", rs.to_str().unwrap()];
    args.extend_from_slice(extra);
    let out = melange(&args, deck);
    assert!(
        out.status.success(),
        "compile failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let code = std::fs::read_to_string(&rs).unwrap();
    let _ = std::fs::remove_file(&rs);
    let list = |name: &str| -> Vec<String> {
        let line = code
            .lines()
            .find(|l| l.starts_with(&format!("pub const {name}: [")))
            .unwrap_or_else(|| panic!("no {name}"));
        let body = &line[line.find("= [").unwrap() + 3..line.rfind(']').unwrap()];
        body.split(", ")
            .map(|v| v.trim_matches('"').to_string())
            .collect()
    };
    list("NODE_NAMES")
        .into_iter()
        .zip(list("DC_OP"))
        .filter(|(n, _)| !n.is_empty())
        .map(|(n, v)| (n, v.parse().unwrap()))
        .collect()
}

/// Every node dc-op prints equals the compiled `DC_OP` (dc-op prints six
/// significant digits).
fn assert_matches_compile(json: &serde_json::Value, compiled: &HashMap<String, f64>) {
    let nodes = json["nodes"].as_object().expect("nodes");
    assert!(!nodes.is_empty());
    for (name, v) in nodes {
        let got = v.as_f64().unwrap();
        let want = *compiled
            .get(name)
            .unwrap_or_else(|| panic!("compile has no node {name}"));
        assert!(
            (got - want).abs() <= 1e-5 * want.abs().max(1e-3),
            "v({name}): dc-op {got}, compile DC_OP {want}"
        );
    }
}

/// A common-emitter stage with a 10 kOhm `.inject` at its collector. The
/// injection's impedance pulls the collector to 3.476 V; without it the
/// collector sits at 6.953 V, which the verb used to print.
const INJECTED_CE: &str = "inject at a collector
V1 vcc 0 9
Rb1 vcc a 100k
Rb2 a 0 22k
C1 in a 1u
Q1 c a e QN
Re e 0 4.7k
Rc vcc c 10k
C2 c out 1u
Rl out 0 100k
.inject c fb R=10k
.model QN NPN(IS=1e-14 BF=100)
";

#[test]
fn dc_op_includes_the_inject_impedance() {
    let deck = write_deck("inject", INJECTED_CE);
    let json = dc_op_json(&deck, &[]);
    let compiled = compiled_dc_op(&deck, &[]);
    let _ = std::fs::remove_file(&deck);
    let vc = json["nodes"]["c"].as_f64().unwrap();
    assert!((vc - 3.476).abs() < 2e-3, "v(c) = {vc}");
    assert_matches_compile(&json, &compiled);
}

/// The master oscillator of a transistor organ's top-octave chain: its DC
/// operating point has a growing pole, so the DK build refuses it and it
/// builds on nodal. The verb reports the nodal route's operating point.
const SELF_STARTING_OSCILLATOR: &str =
    "Transistor-organ master oscillator (LC tank, regenerative feedback) + squarer
Vrail rail 0 DC 8
Vvib vterm 0 DC 8
.runtime Vvib as v_g10_vterm
C_kick in b1 1n
R_e18 rail node_a 1.8k
C_e25 rail node_a 25u
L_fb node_b node_a 8.35m
R_180 node_b e1 180
Q_TN1G c1 b1 e1 SFT307
L_tank c1 tanklo 1.36
K1 L_tank L_fb 0.3
C_tank c1 tanklo 13.5n
R_47k tanklo b1 47k
R_27k2 tanklo 0 2.7k
R_10k rail b1 10k
R_27k vterm b1 27k
C_sq c1 sq_n 10n
R_sqb sq_n b2 47k
R_470k b2 0 470k
Q_TN2G c2 b2 rail SFT307
R_c2 c2 0 10k
C_fout c2 term_f 1u
R_fbleed term_f 0 100k
.model SFT307 PNP(IS=2e-7 BF=110 VAF=60 RB=50 RC=5 RE=1 CJE=60p CJC=25p TF=1n)
.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

#[test]
fn dc_op_reports_the_route_a_self_starting_oscillator_ships_on() {
    let deck = write_deck("osc", SELF_STARTING_OSCILLATOR);
    let json = dc_op_json(&deck, &[]);
    let compiled = compiled_dc_op(&deck, &["-n", "term_f"]);
    let _ = std::fs::remove_file(&deck);
    assert_eq!(json["solver"], "nodal", "{json}");
    assert_eq!(json["converged"], true);
    assert_matches_compile(&json, &compiled);
}

#[test]
fn dc_op_needs_an_input_port_like_compile() {
    let deck = write_deck("noin", "no input\nV1 vcc 0 9\nR1 vcc out 1k\nR2 out 0 1k\n");
    let out = melange(&["dc-op"], &deck);
    let _ = std::fs::remove_file(&deck);
    assert!(!out.status.success(), "a deck without `in` must be refused");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("Input node 'in' not found"), "{stderr}");
}

/// A high-gain op-amp whose output sits beyond its rail at rest, driving a BJT
/// base, under `--opamp-rail-mode boyle-diodes` (the rail is a pair of catch
/// diodes): its DC operating point does not converge (open finding; the same
/// deck converges under every other rail mode).
const UNCONVERGED: &str = "Railed op-amp into a BJT base, catch-diode rail
Vref ref 0 DC 1
R1 in inv 10k
R2 inv oa 100k
U1 ref inv oa OA1
Rload oa 0 1k
Rb oa b 1meg
Q1 c b 0 QN
Rc vcc c 4.7k
Vcc vcc 0 DC 12
Co c out 1u
Rl out 0 100k
.model OA1 OA(AOL=200000 VCC=9 VEE=-9)
.model QN NPN(IS=1e-14 BF=100)
";

#[test]
fn an_unconverged_operating_point_is_refused_unless_allowed() {
    let deck = write_deck("unconverged", UNCONVERGED);
    let rs = deck.with_extension("rs");
    let compile = melange(
        &[
            "compile",
            "--format",
            "code",
            "-o",
            rs.to_str().unwrap(),
            "--opamp-rail-mode",
            "boyle-diodes",
        ],
        &deck,
    );
    let dc_op = melange(&["dc-op", "--opamp-rail-mode", "boyle-diodes"], &deck);
    let dc_op_allowed = melange(
        &[
            "dc-op",
            "--format",
            "json",
            "--allow-unconverged-dc-op",
            "--opamp-rail-mode",
            "boyle-diodes",
        ],
        &deck,
    );
    let _ = std::fs::remove_file(&deck);
    let _ = std::fs::remove_file(&rs);
    for (verb, out) in [("compile", &compile), ("dc-op", &dc_op)] {
        assert!(!out.status.success(), "{verb} must refuse");
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(
            stderr.contains("the DC operating point did not converge")
                && stderr.contains("--allow-unconverged-dc-op"),
            "{verb}: {stderr}"
        );
    }
    assert!(dc_op_allowed.status.success());
    let stdout = String::from_utf8_lossy(&dc_op_allowed.stdout);
    let json: serde_json::Value = serde_json::from_str(
        stdout
            .lines()
            .find(|l| l.trim_start().starts_with('{'))
            .unwrap(),
    )
    .unwrap();
    assert_eq!(json["converged"], false);
}

/// `compile`'s two DC flags reach their own options (they are adjacent bool
/// parameters; a swap once dropped `recompute_dc_op` from every build that
/// asked for it).
#[test]
fn compile_dc_flags_reach_their_own_options() {
    let deck = write_deck("flags", INJECTED_CE);
    let code_with = |flags: &[&str]| -> String {
        let rs = deck.with_extension(format!("{}.rs", flags.len()));
        let mut args = vec!["compile", "--format", "code", "-o", rs.to_str().unwrap()];
        args.extend_from_slice(flags);
        let out = melange(&args, &deck);
        assert!(
            out.status.success(),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
        let code = std::fs::read_to_string(&rs).unwrap();
        let _ = std::fs::remove_file(&rs);
        code
    };
    let recompute = code_with(&["--emit-dc-op-recompute"]);
    let allow = code_with(&["--allow-unconverged-dc-op", "--no-dc-block"]);
    let _ = std::fs::remove_file(&deck);
    assert!(recompute.contains("pub fn recompute_dc_op(&mut self)"));
    assert!(!allow.contains("pub fn recompute_dc_op(&mut self)"));
}
