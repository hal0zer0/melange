//! Every `.model` key melange accepts must change the circuit it compiles.
//!
//! `model_param_table_drift_tests` (melange-solver) proves each accepted key
//! has a *reader*; a reader is not an effect. A key read into a field nothing
//! uses, or honoured on one route or rail mode only, is accepted on a card
//! and silently ignored — the same failure as a silent default. This test
//! measures the effect: for each accepted key it compiles a witness card
//! twice through `melange compile` (the real pipeline, junction-cap stamping
//! and routing included) with only that key's value changed, and requires
//! the generated code to differ.
//!
//! Two directions:
//! * every key a swept class accepts appears in [`CASES`], so a newly
//!   accepted key has to show its effect here before it ships;
//! * every entry's witness changes the generated code.
//!
//! A key that is inert by design unless a companion key is set (IBV without
//! BV, VJE without CJE, KF without noise) is witnessed in the context where
//! it acts: the rich card sets every companion, and noise keys compile with
//! `--noise full`. Recognised-but-unimplemented keys are checked separately:
//! they must compile and print a notice naming the cost.
//!
//! Glow lamps are not swept.

use melange_solver::model_params::ModelClass;
use std::collections::HashSet;
use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Where a key's effect is witnessed.
#[derive(Clone, Copy)]
enum Witness {
    /// The rich card (every key of the class at its first value), this key
    /// changed to its second value.
    Rich,
    /// As `Rich`, compiled with `--noise full`.
    Noise,
    /// A card with only this key, against a card without it. For aliases and
    /// keys superseded by another key on the rich card.
    Alone,
}

use Witness::*;

struct Case {
    class: ModelClass,
    /// Netlist with `{CARD}` where the `.model` line goes.
    deck: &'static str,
    /// `NAME TYPE` of the card.
    card: &'static str,
    /// (key, rich value, changed value, witness).
    keys: &'static [(&'static str, &'static str, &'static str, Witness)],
    /// Keys left off the rich card: aliases, or keys that conflict with it.
    alone_only: &'static [&'static str],
}

const CASES: &[Case] = &[
    Case {
        class: ModelClass::Diode,
        deck: "diode\nR1 in out 1k\nD1 out 0 DX\nR2 out 0 10k\nC1 out 0 1n\n{CARD}\n",
        card: "DX D",
        keys: &[
            ("IS", "2.52e-9", "1e-12", Rich),
            ("N", "1.752", "1.3", Rich),
            ("CJO", "4e-12", "8e-12", Rich),
            ("RS", "0.5", "2", Rich),
            ("BV", "100", "30", Rich),
            ("IBV", "1e-10", "1e-6", Rich),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
            ("RTH", "100", "200", Rich),
            ("CTH", "1e-4", "1e-3", Rich),
            ("XTI", "3", "2", Rich),
            ("EG", "1.11", "0.69", Rich),
            // The device temperature: acts on its own, as SPICE's `.temp`.
            ("TAMB", "", "320", Alone),
        ],
        alone_only: &["TAMB"],
    },
    Case {
        class: ModelClass::Bjt,
        deck: "bjt\nCin in base 10u\nR1 vcc base 100k\nR2 base 0 22k\nQ1 coll base emit QX\n\
               Rc vcc coll 4.7k\nRe emit 0 1k\nCe emit 0 100u\nCout coll out 10u\n\
               Rload out 0 100k\nVcc vcc 0 DC 12\n{CARD}\n",
        card: "QX NPN",
        keys: &[
            ("IS", "1e-14", "2e-14", Rich),
            ("BF", "200", "100", Rich),
            ("BR", "3", "1", Rich),
            ("VAF", "100", "50", Rich),
            ("VAR", "50", "20", Rich),
            ("IKF", "0.1", "0.01", Rich),
            ("IKR", "0.01", "0.001", Rich),
            ("CJE", "10e-12", "20e-12", Rich),
            ("CJC", "5e-12", "8e-12", Rich),
            ("VJE", "0.75", "0.6", Rich),
            ("MJE", "0.33", "0.4", Rich),
            ("VJC", "0.75", "0.6", Rich),
            ("MJC", "0.33", "0.4", Rich),
            ("FC", "0.5", "0.4", Rich),
            ("TF", "1e-9", "2e-9", Rich),
            ("NF", "1", "1.1", Rich),
            ("NR", "1", "1.1", Rich),
            ("ISE", "1e-15", "1e-14", Rich),
            ("NE", "1.5", "2", Rich),
            ("ISC", "1e-15", "1e-14", Rich),
            ("NC", "2", "1.8", Rich),
            ("RB", "10", "20", Rich),
            ("RC", "1", "2", Rich),
            ("RE", "0.5", "1", Rich),
            ("RTH", "100", "200", Rich),
            ("CTH", "1e-3", "1e-4", Rich),
            ("XTI", "3", "2", Rich),
            ("XTB", "1.5", "1", Rich),
            ("EG", "1.11", "0.69", Rich),
            ("TAMB", "", "320", Alone),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
            ("VT", "", "0.03", Alone),
            ("VA", "", "50", Alone),
            ("VB", "", "20", Alone),
            ("JBF", "", "0.01", Alone),
            ("JBR", "", "0.001", Alone),
        ],
        alone_only: &["TAMB", "VT", "VA", "VB", "JBF", "JBR"],
    },
    Case {
        class: ModelClass::Jfet,
        deck: "jfet\nR1 in g 1k\nRg g 0 1Meg\nJ1 d g s JX\nRd vcc d 10k\nRs s 0 1k\n\
               Vcc vcc 0 DC 12\nC1 d out 1u\nR3 out 0 100k\n{CARD}\n",
        card: "JX NJF",
        keys: &[
            ("VTO", "-2", "-1.5", Rich),
            ("BETA", "1e-3", "2e-3", Rich),
            ("LAMBDA", "0.01", "0.02", Rich),
            ("CGS", "2e-12", "4e-12", Rich),
            ("CGD", "1e-12", "2e-12", Rich),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
            ("IDSS", "", "4e-3", Alone),
        ],
        alone_only: &["IDSS"],
    },
    Case {
        class: ModelClass::Mosfet,
        deck: "mosfet\nR1 in g 1k\nRg g 0 1Meg\nM1 d g s 0 MX\nRd vcc d 10k\nRs s 0 1k\n\
               Vcc vcc 0 DC 12\nC1 d out 1u\nR3 out 0 100k\n{CARD}\n",
        card: "MX NMOS",
        keys: &[
            ("KP", "1e-3", "2e-3", Rich),
            ("VTO", "1", "1.5", Rich),
            ("LAMBDA", "0.01", "0.02", Rich),
            ("CGS", "2e-12", "4e-12", Rich),
            ("CGD", "1e-12", "2e-12", Rich),
            ("GAMMA", "0.5", "0.3", Rich),
            ("PHI", "0.6", "0.7", Rich),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
            ("VT", "", "1.5", Alone),
        ],
        alone_only: &["VT"],
    },
    Case {
        class: ModelClass::Triode,
        deck: "triode\nCin in g 100n\nRg g 0 1Meg\nT1 g p k TX\nRa vcc p 100k\nRk k 0 1.5k\n\
               Ck k 0 25u\nCout p out 100n\nRl out 0 1Meg\nVCC vcc 0 250\n{CARD}\n",
        card: "TX TRIODE",
        keys: &[
            ("MU", "100", "90", Rich),
            ("EX", "1.4", "1.3", Rich),
            ("KG1", "1060", "1000", Rich),
            ("KP", "600", "500", Rich),
            ("KVB", "300", "250", Rich),
            ("GG", "6.177e-4", "5e-4", Rich),
            ("XI", "1.314", "1.4", Rich),
            ("CG", "9.901", "8", Rich),
            ("LAMBDA", "0.001", "0.002", Rich),
            ("CCG", "2e-12", "3e-12", Rich),
            ("CGP", "1.7e-12", "2e-12", Rich),
            ("CCP", "0.5e-12", "1e-12", Rich),
            ("RGI", "1000", "2000", Rich),
            ("MU_B", "60", "50", Rich),
            ("SVAR", "0.5", "0.3", Rich),
            ("EX_B", "1.4", "1.3", Rich),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
            ("RTH", "500", "600", Rich),
            ("CTH", "5e-3", "1e-3", Rich),
            ("VBIAS_ALPHA", "3e-4", "1e-4", Rich),
            ("TAMB", "300.15", "320", Rich),
            ("SHOT_GAMMA2", "0.5", "0.8", Noise),
        ],
        alone_only: &[],
    },
    Case {
        class: ModelClass::Pentode,
        deck: "pentode\nCin in g 100n\nRg g 0 1Meg\nP1 p g k scr PX\nRk k 0 150\nCk k 0 100u\n\
               Rp vcc p 5k\nRscr vcc scr 1k\nVCC vcc 0 300\nCout p out 100n\nRl out 0 1Meg\n\
               {CARD}\n",
        card: "PX PENTODE",
        keys: &[
            ("MU", "20", "18", Rich),
            ("EX", "1.4", "1.3", Rich),
            ("KG1", "1500", "1400", Rich),
            ("KG2", "4500", "4000", Rich),
            ("KP", "200", "180", Rich),
            ("KVB", "300", "250", Rich),
            ("ALPHA_S", "7", "6", Rich),
            ("A_FACTOR", "0.0000125", "0.00002", Rich),
            ("BETA_FACTOR", "0.05", "0.1", Rich),
            ("PARTITION_F", "0.5", "0.6", Noise),
            ("SCREEN_FORM", "0", "1", Rich),
            ("IG_MAX", "2e-3", "4e-3", Rich),
            ("VGK_ONSET", "0.5", "0.7", Rich),
            ("LAMBDA", "0.001", "0.002", Rich),
            ("CCG", "2e-12", "3e-12", Rich),
            ("CGP", "1e-12", "2e-12", Rich),
            ("CCP", "5e-12", "6e-12", Rich),
            ("RGI", "1000", "2000", Rich),
            ("MU_B", "10", "8", Rich),
            ("SVAR", "0.5", "0.3", Rich),
            ("EX_B", "1.4", "1.3", Rich),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
        ],
        alone_only: &[],
    },
    Case {
        class: ModelClass::Opamp,
        // Capacitor-coupled downstream, so the auto rail mode is active-set:
        // R_SAG acts through the active-set pin.
        deck: "opamp\nR1 in inv 10k\nR2 inv out 100k\nU1 0 inv out OX\nRl out 0 10k\n\
               Cc out o2 1u\nR3 o2 0 10k\n{CARD}\n",
        card: "OX OA",
        keys: &[
            ("AOL", "100000", "200000", Rich),
            ("ROUT", "100", "50", Rich),
            ("R_SAG", "200", "100", Rich),
            ("VCC", "15", "12", Rich),
            ("VEE", "-15", "-12", Rich),
            ("SR", "1", "0.5", Rich),
            ("VOH_DROP", "1", "1.5", Rich),
            ("VOL_DROP", "1", "1.5", Rich),
            ("AOL_TRANSIENT_CAP", "1000", "500", Rich),
            ("IB", "1e-9", "1e-8", Rich),
            ("RIN", "1e6", "2e6", Rich),
            ("EN", "10e-9", "20e-9", Noise),
            ("IN", "1e-12", "2e-12", Noise),
            // Superseded by VCC/VEE on the rich card; alone, VSAT sets the
            // rails, and GBW (not a bandwidth pole) sets the default ±13 V.
            ("VSAT", "", "13", Alone),
            ("GBW", "", "3e6", Alone),
        ],
        alone_only: &["VSAT", "GBW"],
    },
    Case {
        class: ModelClass::Vca,
        deck: "vca\nR1 in a 10k\nY1 a sum cv 0 VX\nRs sum 0 10k\nR3 sum out 1k\nRl out 0 100k\n\
               Vcv cv 0 DC 0.1\n{CARD}\n",
        card: "VX VCA",
        keys: &[
            ("VSCALE", "0.05", "0.1", Rich),
            ("G0", "1", "0.5", Rich),
            ("THD", "0.001", "0.01", Rich),
            ("MODE", "0", "1", Rich),
        ],
        alone_only: &[],
    },
    Case {
        class: ModelClass::Ldr,
        deck: "ldr\nRin in a 10k\nO1 a out lfo 0 LX\nRload out 0 100k\nCout out 0 10n\n\
               Vlfo lfo 0 DC 0.5\n{CARD}\n",
        card: "LX LDR",
        keys: &[
            ("RMIN", "100", "200", Rich),
            ("RMAX", "2e6", "1e6", Rich),
            ("GAMMA", "1.0", "0.8", Rich),
            ("TAU_A", "0.001", "0.002", Rich),
            ("TAU_R", "0.05", "0.1", Rich),
        ],
        alone_only: &[],
    },
];

static SEQ: AtomicUsize = AtomicUsize::new(0);

/// A scratch directory per test (tests in one binary run concurrently).
fn scratch(test: &str) -> PathBuf {
    let dir =
        std::env::temp_dir().join(format!("melange_key_effect_{}_{test}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn card(case: &Case, params: &[(&str, &str)]) -> String {
    let p: Vec<String> = params.iter().map(|(k, v)| format!("{k}={v}")).collect();
    case.deck
        .replace("{CARD}", &format!(".model {}({})", case.card, p.join(" ")))
}

/// Compile `deck` and return the generated code without comments and
/// without lines naming the scratch file; `Err` carries the CLI output.
fn compile(deck: &str, args: &[&str], test: &str) -> Result<Vec<String>, String> {
    let stem = format!("k{}", SEQ.fetch_add(1, Ordering::Relaxed));
    let dir = scratch(test);
    let cir = dir.join(format!("{stem}.cir"));
    let rs = dir.join(format!("{stem}.rs"));
    std::fs::write(&cir, deck).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_melange"))
        .arg("compile")
        .arg(&cir)
        .arg("-o")
        .arg(&rs)
        .args(args)
        .output()
        .expect("run melange");
    if !out.status.success() {
        return Err(format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    let code = std::fs::read_to_string(&rs).unwrap();
    let _ = std::fs::remove_file(&cir);
    let _ = std::fs::remove_file(&rs);
    Ok(code
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with("//") && !l.starts_with("#!"))
        .filter(|l| !l.contains(&stem) && !l.contains("fnv1a64") && !l.contains(".model"))
        .map(String::from)
        .collect())
}

fn lines_differing(a: &[String], b: &[String]) -> usize {
    let sa: HashSet<&String> = a.iter().collect();
    let sb: HashSet<&String> = b.iter().collect();
    sa.symmetric_difference(&sb).count()
}

/// The first and second compile for one key's witness.
fn witness_pair(
    case: &Case,
    key: &str,
    a: &str,
    b: &str,
    w: Witness,
) -> (String, String, Vec<&'static str>) {
    let rich: Vec<(&str, &str)> = case
        .keys
        .iter()
        .filter(|(k, ..)| !case.alone_only.contains(k))
        .map(|(k, v, ..)| (*k, *v))
        .collect();
    let changed: Vec<(&str, &str)> = rich
        .iter()
        .map(|&(k, v)| if k == key { (k, b) } else { (k, v) })
        .collect();
    match w {
        Rich => (card(case, &rich), card(case, &changed), vec![]),
        Noise => (
            card(case, &rich),
            card(case, &changed),
            vec!["--noise", "full", "--noise-seed", "1"],
        ),
        Alone => {
            let with: Vec<(&str, &str)> = vec![(key, b)];
            let without: Vec<(&str, &str)> = if a.is_empty() { vec![] } else { vec![(key, a)] };
            (card(case, &without), card(case, &with), vec![])
        }
    }
}

#[test]
fn every_swept_class_lists_every_accepted_key() {
    let mut missing = Vec::new();
    for case in CASES {
        for key in case.class.honored() {
            if !case.keys.iter().any(|(k, ..)| k.eq_ignore_ascii_case(key)) {
                missing.push(format!("{} / {}", case.class.label(), key));
            }
        }
        for (k, ..) in case.keys {
            assert!(
                case.class.is_honored(k),
                "{} / {k} is swept but not accepted — drop it from CASES",
                case.class.label()
            );
        }
    }
    assert!(
        missing.is_empty(),
        "accepted .model keys with no effect witness in CASES: {missing:?}. Add each \
         with a witness that shows it changes the generated code."
    );
}

#[test]
fn every_accepted_key_changes_the_generated_code() {
    let jobs: Vec<(&Case, &str, &str, &str, Witness)> = CASES
        .iter()
        .flat_map(|c| c.keys.iter().map(move |&(k, a, b, w)| (c, k, a, b, w)))
        .collect();
    let next = AtomicUsize::new(0);
    let failures = std::sync::Mutex::new(Vec::<String>::new());
    std::thread::scope(|s| {
        for _ in 0..8 {
            s.spawn(|| loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                let Some(&(case, key, a, b, w)) = jobs.get(i) else {
                    break;
                };
                let (d1, d2, args) = witness_pair(case, key, a, b, w);
                let verdict = match (compile(&d1, &args, "effect"), compile(&d2, &args, "effect")) {
                    (Ok(x), Ok(y)) if lines_differing(&x, &y) > 0 => None,
                    (Ok(_), Ok(_)) => Some("no effect on the generated code".to_string()),
                    (Err(e), _) | (_, Err(e)) => Some(format!(
                        "compile failed: {}",
                        e.lines().last().unwrap_or("")
                    )),
                };
                if let Some(v) = verdict {
                    failures
                        .lock()
                        .unwrap()
                        .push(format!("{} / {key}: {v}", case.class.label()));
                }
            });
        }
    });
    let _ = std::fs::remove_dir_all(scratch("effect"));
    let failures = failures.into_inner().unwrap();
    assert!(
        failures.is_empty(),
        "accepted .model keys that did not change the compiled circuit in their \
         witness context: {failures:?}"
    );
}

#[test]
fn unimplemented_keys_compile_with_a_costed_notice() {
    for case in CASES {
        for (key, _) in case.class.unimplemented() {
            let rich: Vec<(&str, &str)> = case
                .keys
                .iter()
                .filter(|(k, ..)| !case.alone_only.contains(k))
                .map(|(k, v, ..)| (*k, *v))
                .chain(std::iter::once((*key, "1")))
                .collect();
            let deck = card(case, &rich);
            let dir = scratch("notice");
            let cir = dir.join(format!("u{}.cir", SEQ.fetch_add(1, Ordering::Relaxed)));
            std::fs::write(&cir, &deck).unwrap();
            let out = Command::new(env!("CARGO_BIN_EXE_melange"))
                .arg("compile")
                .arg(&cir)
                .arg("-o")
                .arg(cir.with_extension("rs"))
                .output()
                .expect("run melange");
            let text = format!(
                "{}{}",
                String::from_utf8_lossy(&out.stdout),
                String::from_utf8_lossy(&out.stderr)
            );
            assert!(
                out.status.success(),
                "{} / {key}: a recognised-but-unimplemented key must compile:\n{text}",
                case.class.label()
            );
            assert!(
                text.contains(&format!(
                    "'{key}' is a recognized parameter that melange does not model yet"
                )),
                "{} / {key}: no notice naming the cost:\n{text}",
                case.class.label()
            );
        }
    }
    let _ = std::fs::remove_dir_all(scratch("notice"));
}

#[test]
fn an_unknown_key_is_refused() {
    for case in CASES {
        let rich: Vec<(&str, &str)> = case
            .keys
            .iter()
            .filter(|(k, ..)| !case.alone_only.contains(k))
            .map(|(k, v, ..)| (*k, *v))
            .chain(std::iter::once(("ZORP", "1")))
            .collect();
        let Err(err) = compile(&card(case, &rich), &[], "unknown") else {
            panic!(
                "{}: a card with an unknown key compiled",
                case.class.label()
            );
        };
        assert!(
            err.contains("unknown parameter 'ZORP'"),
            "{}: an unknown key must be refused by name:\n{err}",
            case.class.label()
        );
    }
    let _ = std::fs::remove_dir_all(scratch("unknown"));
}
