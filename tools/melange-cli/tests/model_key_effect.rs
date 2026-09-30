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
//! A key can change the generated code and still not change the answer: a
//! JFET's `RS=` once reached only the Newton Jacobian (its constants were in
//! the code), so the converged circuit was the device without it. So each
//! key is also required to move the DC operating point (`melange dc-op`)
//! on its witness card, unless it is listed in [`DC_INERT`] with the reason
//! it cannot (charge storage, time constants, noise, slew).
//!
//! And the converse: every per-device parameter the build emits must be read
//! (`every_emitted_device_parameter_is_read`). A `DEVICE_n_*` constant or
//! `device_n_*` state field that is declared and never read changes the
//! generated code, so it passes the effect check above, and changes nothing.
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
    /// The DC operating-point witness when `deck` has no DC excitation that
    /// lets the device's keys act (empty = `deck`).
    dc_deck: &'static str,
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
        dc_deck: "diode dc\nRin in 0 1k\nVs a 0 DC 1\nR1 a out 1k\nD1 out 0 DX\n{CARD}\n",
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
        dc_deck: "",
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
            ("IS", "1e-14", "1e-13", Rich),
            ("N", "1", "1.5", Rich),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
            ("IDSS", "", "4e-3", Alone),
        ],
        alone_only: &["IDSS"],
        dc_deck: "",
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
        dc_deck: "mosfet dc\nRin in 0 1k\nRg1 vcc g 1Meg\nRg2 g 0 470k\nM1 d g s 0 MX\n\
                  Rd vcc d 10k\nRs s 0 1k\nVcc vcc 0 DC 12\n{CARD}\n",
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
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
            ("RTH", "500", "600", Rich),
            ("CTH", "5e-3", "1e-3", Rich),
            ("VBIAS_ALPHA", "3e-4", "1e-4", Rich),
            ("TAMB", "300.15", "320", Rich),
            ("SHOT_GAMMA2", "0.5", "0.8", Noise),
        ],
        alone_only: &[],
        dc_deck: "",
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
            ("CCG", "2e-12", "3e-12", Rich),
            ("CGP", "1e-12", "2e-12", Rich),
            ("CCP", "5e-12", "6e-12", Rich),
            ("MU_B", "10", "8", Rich),
            ("SVAR", "0.5", "0.3", Rich),
            ("EX_B", "1.4", "1.3", Rich),
            ("KF", "1e-16", "1e-14", Noise),
            ("AF", "1", "1.3", Noise),
        ],
        alone_only: &[],
        dc_deck: "",
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
        // Two stages driven into opposite rails, so the rail keys act at DC;
        // capacitor-coupled so the auto rail mode is active-set (R_SAG acts).
        dc_deck: "opamp dc\nRin in 0 1k\nVp p 0 DC 1\nVn n 0 DC -1\nR1 p i1 10k\nR2 i1 o1 200k\n\
                  U1 0 i1 o1 OX\nRl1 o1 0 10k\nR3 n i2 10k\nR4 i2 o2 200k\nU2 0 i2 o2 OX\n\
                  Rl2 o2 0 10k\nCc1 o1 x1 1u\nRx1 x1 0 10k\nCc2 o2 x2 1u\nRx2 x2 0 10k\n{CARD}\n",
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
        dc_deck: "vca dc\nRin in 0 1k\nVs s0 0 DC 1\nR1 s0 a 10k\nY1 a sum cv 0 VX\nRs sum 0 10k\n\
                  Vcv cv 0 DC 0.1\n{CARD}\n",
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
        dc_deck: "ldr dc\nRin in 0 1k\nVs s0 0 DC 1\nR1 s0 a 10k\nO1 a out lfo 0 LX\n\
                  Rload out 0 100k\nVlfo lfo 0 DC 0.5\n{CARD}\n",
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
/// With `--format plugin` the code is the project's `src/circuit.rs` and
/// `src/lib.rs`.
fn compile(deck: &str, args: &[&str], test: &str) -> Result<Vec<String>, String> {
    let stem = format!("k{}", SEQ.fetch_add(1, Ordering::Relaxed));
    let dir = scratch(test);
    let cir = dir.join(format!("{stem}.cir"));
    let plugin = args.windows(2).any(|w| w == ["--format", "plugin"]);
    let rs = if plugin {
        dir.join(&stem)
    } else {
        dir.join(format!("{stem}.rs"))
    };
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
    let code = if plugin {
        let src = rs.join("src");
        let code = std::fs::read_to_string(src.join("circuit.rs")).unwrap()
            + "\n"
            + &std::fs::read_to_string(src.join("lib.rs")).unwrap();
        let _ = std::fs::remove_dir_all(&rs);
        code
    } else {
        let code = std::fs::read_to_string(&rs).unwrap();
        let _ = std::fs::remove_file(&rs);
        code
    };
    let _ = std::fs::remove_file(&cir);
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

/// Identifiers in `line` with the byte offset just past each.
fn identifiers(line: &str) -> Vec<(&str, usize)> {
    let b = line.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        if b[i].is_ascii_alphabetic() || b[i] == b'_' {
            let start = i;
            while i < b.len() && (b[i].is_ascii_alphanumeric() || b[i] == b'_') {
                i += 1;
            }
            // Not the tail of a number literal such as `1e3` or `0_f64`.
            if start == 0 || !b[start - 1].is_ascii_digit() {
                out.push((&line[start..i], i));
            }
        } else {
            i += 1;
        }
    }
    out
}

/// `DEVICE_<n>_<KEY>` / `device_<n>_<key>`.
fn is_device_param(id: &str, prefix: &str) -> bool {
    id.strip_prefix(prefix)
        .and_then(|rest| rest.split_once('_'))
        .is_some_and(|(n, key)| {
            !n.is_empty() && n.bytes().all(|c| c.is_ascii_digit()) && !key.is_empty()
        })
}

/// The per-device parameters `code` declares and never reads: a
/// `DEVICE_n_*` constant named nowhere but its declaration, and a
/// `device_n_*` state field that is only ever written (declared,
/// initialised, assigned).
fn unread_device_params(code: &[String]) -> Vec<String> {
    let lines: Vec<&str> = code
        .iter()
        .map(|l| l.split_once("//").map_or(l.as_str(), |(c, _)| c))
        .collect();
    let mut consts: Vec<&str> = Vec::new();
    let mut fields: Vec<&str> = Vec::new();
    let mut uses: std::collections::HashMap<&str, usize> = Default::default();
    let mut field_reads: HashSet<&str> = HashSet::new();
    for line in &lines {
        let ids = identifiers(line);
        for (k, &(id, end)) in ids.iter().enumerate() {
            if is_device_param(id, "DEVICE_") {
                *uses.entry(id).or_default() += 1;
                if k > 0 && ids[k - 1].0 == "const" {
                    consts.push(id);
                }
            }
            if is_device_param(id, "device_") {
                let rest = line[end..].trim_start();
                let dotted = line[..end - id.len()].ends_with('.');
                if !dotted && rest.starts_with(':') && !rest.starts_with("::") {
                    // `pub device_0_mu: f64` or `device_0_mu: DEVICE_0_MU,`
                    if line.trim_start().starts_with("pub ") {
                        fields.push(id);
                    }
                } else if dotted {
                    // `x.device_0_mu = ..` writes; anything else reads it
                    // (`+=` included).
                    let write = rest.starts_with('=') && !rest.starts_with("==");
                    if !write {
                        field_reads.insert(id);
                    }
                }
            }
        }
    }
    let mut unread: Vec<String> = consts
        .into_iter()
        .filter(|c| uses.get(c).copied().unwrap_or(0) < 2)
        .map(|c| format!("const {c}"))
        .collect();
    unread.extend(
        fields
            .into_iter()
            .filter(|f| !field_reads.contains(f))
            .map(|f| format!("state.{f}")),
    );
    unread.sort();
    unread.dedup();
    unread
}

#[test]
fn the_scan_finds_a_declared_and_unread_parameter() {
    let code: Vec<String> = [
        "const DEVICE_0_MU: f64 = 100.0;",
        "const DEVICE_0_MU_B: f64 = 60.0;",
        "const DEVICE_0_EX: f64 = 1.4;",
        "pub device_0_mu: f64,",
        "pub device_0_lambda: f64,",
        "device_0_mu: DEVICE_0_MU,",
        "device_0_lambda: 0.0, // state.device_0_lambda * 2.0",
        "self.device_0_lambda = DEVICE_0_EX;",
        "let ip = tube_ip(vgk, state.device_0_mu);",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    assert_eq!(
        unread_device_params(&code),
        vec!["const DEVICE_0_MU_B", "state.device_0_lambda"]
    );
}

/// A parameter the build emits and nothing reads is a key accepted and
/// silently ignored, the class behind a pentode's LAMBDA and RGI and a
/// triode's MU_B/SVAR/EX_B, each found one at a time. Every class's rich card
/// (every key at once, noise on), on the DK and nodal routes, with and
/// without the runtime DC-OP recompute, and as a plugin project, must read
/// every `DEVICE_n_*` constant and `device_n_*` state field it declares.
#[test]
fn every_emitted_device_parameter_is_read() {
    let mut failures = Vec::new();
    for case in CASES {
        let rich: Vec<(&str, &str)> = case
            .keys
            .iter()
            .filter(|(k, ..)| !case.alone_only.contains(k))
            .map(|(k, v, ..)| (*k, *v))
            .collect();
        let deck = card(case, &rich);
        let builds: [(&str, &[&str]); 5] = [
            ("dk", &["--solver", "dk"]),
            ("nodal", &["--solver", "nodal"]),
            (
                "dk recompute",
                &["--solver", "dk", "--emit-dc-op-recompute"],
            ),
            (
                "nodal recompute",
                &["--solver", "nodal", "--emit-dc-op-recompute"],
            ),
            ("plugin", &["--format", "plugin", "--emit-dc-op-recompute"]),
        ];
        for (build, extra) in builds {
            let mut args = vec!["--noise", "full", "--noise-seed", "1"];
            args.extend_from_slice(extra);
            match compile(&deck, &args, "unread") {
                Ok(code) => {
                    let recompute = code.iter().any(|l| l.contains("fn recompute_dc_op"));
                    if recompute != extra.contains(&"--emit-dc-op-recompute") {
                        failures.push(format!(
                            "{} ({build}): the build is not the one asked for \
                             (recompute_dc_op present: {recompute})",
                            case.class.label()
                        ));
                    }
                    for p in unread_device_params(&code) {
                        failures.push(format!("{} ({build}): {p}", case.class.label()));
                    }
                }
                // DK refuses some classes (active-set op-amps); nodal must build.
                Err(e) if build.starts_with("dk") && e.contains("--solver nodal") => {}
                Err(e) => failures.push(format!(
                    "{} ({build}): compile failed: {}",
                    case.class.label(),
                    e.lines().last().unwrap_or("")
                )),
            }
        }
    }
    let _ = std::fs::remove_dir_all(scratch("unread"));
    assert!(
        failures.is_empty(),
        "device parameters emitted and never read (an accepted key the circuit \
         ignores): {failures:#?}"
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

/// A known key that is not in the solution is refused with its reason when
/// nonzero, never accepted and ignored (FET RD/RS, pentode LAMBDA).
#[test]
fn a_refused_key_is_refused_with_its_reason() {
    let mut seen = 0;
    for case in CASES {
        for (key, note) in case.class.refused() {
            seen += 1;
            let rich: Vec<(&str, &str)> = case
                .keys
                .iter()
                .filter(|(k, ..)| !case.alone_only.contains(k))
                .map(|(k, v, ..)| (*k, *v))
                .chain(std::iter::once((*key, "10")))
                .collect();
            let Err(err) = compile(&card(case, &rich), &[], "refused") else {
                panic!(
                    "{} / {key}: a card carrying it compiled",
                    case.class.label()
                );
            };
            let head: String = note.chars().take(40).collect();
            assert!(
                err.contains(&format!("{key}=10 is refused")) && err.contains(&head),
                "{} / {key}: must be refused with its reason:\n{err}",
                case.class.label()
            );
            let zero: Vec<(&str, &str)> = rich
                .iter()
                .map(|&(k, v)| if k == *key { (k, "0") } else { (k, v) })
                .collect();
            if let Err(e) = compile(&card(case, &zero), &[], "refused") {
                panic!(
                    "{} / {key}=0 (the model without it) must build: {e}",
                    case.class.label()
                );
            }
        }
    }
    assert!(
        seen >= 9,
        "expected RD/RS on JFET and MOSFET, pentode LAMBDA/RGI and triode MU_B/SVAR/EX_B, \
         saw {seen}"
    );
    let _ = std::fs::remove_dir_all(scratch("refused"));
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

/// Keys that legitimately leave the DC operating point unchanged on their
/// witness card, each with the reason. Everything else must move it.
const DC_INERT: &[(ModelClass, &str, &str)] = &[
    (ModelClass::Diode, "CJO", CHARGE),
    (
        ModelClass::Diode,
        "CTH",
        "thermal capacitance: a time constant",
    ),
    (ModelClass::Diode, "RTH", COLD),
    (ModelClass::Diode, "XTI", COLD),
    (ModelClass::Diode, "EG", COLD),
    (
        ModelClass::Diode,
        "BV",
        "acts only in reverse breakdown, which neither witness reaches",
    ),
    (ModelClass::Bjt, "CJE", CHARGE),
    (ModelClass::Bjt, "CJC", CHARGE),
    (ModelClass::Bjt, "VJE", CHARGE),
    (ModelClass::Bjt, "MJE", CHARGE),
    (ModelClass::Bjt, "VJC", CHARGE),
    (ModelClass::Bjt, "MJC", CHARGE),
    (ModelClass::Bjt, "FC", CHARGE),
    (ModelClass::Bjt, "TF", CHARGE),
    (
        ModelClass::Bjt,
        "CTH",
        "thermal capacitance: a time constant",
    ),
    (ModelClass::Bjt, "RTH", COLD),
    (ModelClass::Bjt, "XTI", COLD),
    (ModelClass::Bjt, "XTB", COLD),
    (ModelClass::Bjt, "EG", COLD),
    (
        ModelClass::Bjt,
        "NR",
        "the witness's B-C junction is reverse biased; exp(Vbc/(NR*Vt)) is below f64 there",
    ),
    (
        ModelClass::Bjt,
        "NC",
        "the witness's B-C junction is reverse biased; exp(Vbc/(NC*Vt)) is below f64 there",
    ),
    (ModelClass::Jfet, "CGS", CHARGE),
    (ModelClass::Jfet, "CGD", CHARGE),
    (ModelClass::Mosfet, "CGS", CHARGE),
    (ModelClass::Mosfet, "CGD", CHARGE),
    (ModelClass::Triode, "CCG", CHARGE),
    (ModelClass::Triode, "CGP", CHARGE),
    (ModelClass::Triode, "CCP", CHARGE),
    (
        ModelClass::Triode,
        "CTH",
        "thermal capacitance: a time constant",
    ),
    (ModelClass::Triode, "RTH", COLD),
    (ModelClass::Triode, "VBIAS_ALPHA", COLD),
    (ModelClass::Triode, "TAMB", COLD),
    (ModelClass::Pentode, "CCG", CHARGE),
    (ModelClass::Pentode, "CGP", CHARGE),
    (ModelClass::Pentode, "CCP", CHARGE),
    (
        ModelClass::Pentode,
        "IG_MAX",
        "control-grid current is exactly 0 at the witness's negative grid bias",
    ),
    (
        ModelClass::Pentode,
        "VGK_ONSET",
        "control-grid current is exactly 0 at the witness's negative grid bias",
    ),
    (
        ModelClass::Opamp,
        "SR",
        "slew rate: acts only on the transient",
    ),
    (
        ModelClass::Opamp,
        "AOL_TRANSIENT_CAP",
        "the transient's AOL; the DC operating point keeps the full AOL by design",
    ),
    (
        ModelClass::Ldr,
        "RMIN",
        "the DC operating point is the dark seed (RMAX); the light state develops in the transient",
    ),
    (
        ModelClass::Ldr,
        "GAMMA",
        "the DC operating point is the dark seed (RMAX); the light state develops in the transient",
    ),
    (ModelClass::Ldr, "TAU_A", "a time constant"),
    (ModelClass::Ldr, "TAU_R", "a time constant"),
];

const CHARGE: &str = "charge storage: acts only on the transient";
const COLD: &str = "the DC operating point is the cold power-on state; self-heating develops in the transient over RTH*CTH (XTI/EG/XTB/VBIAS_ALPHA/TAMB act here only through it)";

/// DC keys the operating point does not see today: defects, each queued for
/// its fix. The test requires them to STILL not move it, so a fix removes
/// its entry here.
const KNOWN_DC_DEFECTS: &[(ModelClass, &str, &str)] = &[];

/// Node voltages and device voltages/currents of `melange dc-op --format
/// json` (full precision), in a stable order.
fn dc_op(deck: &str, test: &str) -> Result<Vec<(String, f64)>, String> {
    let stem = format!("d{}", SEQ.fetch_add(1, Ordering::Relaxed));
    let cir = scratch(test).join(format!("{stem}.cir"));
    std::fs::write(&cir, deck).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_melange"))
        .args(["dc-op", "--format", "json"])
        .arg(&cir)
        .output()
        .expect("run melange");
    let _ = std::fs::remove_file(&cir);
    let stdout = String::from_utf8_lossy(&out.stdout);
    let Some(line) = stdout.lines().find(|l| l.trim_start().starts_with('{')) else {
        return Err(format!("{stdout}{}", String::from_utf8_lossy(&out.stderr)));
    };
    let json: serde_json::Value = serde_json::from_str(line).map_err(|e| e.to_string())?;
    let mut v: Vec<(String, f64)> = Vec::new();
    for (k, x) in json["nodes"].as_object().into_iter().flatten() {
        v.push((format!("v({k})"), x.as_f64().unwrap_or(f64::NAN)));
    }
    for (k, d) in json["devices"].as_object().into_iter().flatten() {
        v.push((format!("{k}.v"), d["v_nl"].as_f64().unwrap_or(f64::NAN)));
        v.push((format!("{k}.i"), d["i_nl"].as_f64().unwrap_or(f64::NAN)));
    }
    v.sort_by(|a, b| a.0.cmp(&b.0));
    Ok(v)
}

#[test]
fn every_dc_key_changes_the_operating_point() {
    let jobs: Vec<(&Case, &str, &str, &str, Witness)> = CASES
        .iter()
        .flat_map(|c| c.keys.iter().map(move |&(k, a, b, w)| (c, k, a, b, w)))
        .filter(|(_, _, _, _, w)| !matches!(w, Noise))
        .collect();
    let next = AtomicUsize::new(0);
    let failures = std::sync::Mutex::new(Vec::<String>::new());
    let stale = std::sync::Mutex::new(Vec::<String>::new());
    std::thread::scope(|s| {
        for _ in 0..8 {
            s.spawn(|| loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                let Some(&(case, key, a, b, w)) = jobs.get(i) else {
                    break;
                };
                let listed = |list: &[(ModelClass, &str, &str)]| {
                    list.iter()
                        .any(|(c, k, _)| *c == case.class && k.eq_ignore_ascii_case(key))
                };
                let inert = listed(DC_INERT) || listed(KNOWN_DC_DEFECTS);
                // Moves on the code-effect card, or on the DC card when there is one.
                let mut moved = false;
                let mut error = None;
                for deck in [case.deck, case.dc_deck]
                    .into_iter()
                    .filter(|d| !d.is_empty())
                {
                    let dc_case = Case {
                        class: case.class,
                        deck,
                        card: case.card,
                        keys: case.keys,
                        alone_only: case.alone_only,
                        dc_deck: "",
                    };
                    let (d1, d2, _) = witness_pair(&dc_case, key, a, b, w);
                    match (dc_op(&d1, "dc"), dc_op(&d2, "dc")) {
                        (Ok(x), Ok(y)) => moved |= x != y,
                        (Err(e), _) | (_, Err(e)) => error = Some(e),
                    }
                }
                if let (false, Some(e)) = (moved, error) {
                    failures.lock().unwrap().push(format!(
                        "{} / {key}: dc-op failed: {}",
                        case.class.label(),
                        e.lines().last().unwrap_or("")
                    ));
                    continue;
                }
                let label = format!("{} / {key}", case.class.label());
                match (moved, inert) {
                    (false, false) => failures.lock().unwrap().push(label),
                    (true, true) => stale.lock().unwrap().push(label),
                    _ => {}
                }
            });
        }
    });
    let _ = std::fs::remove_dir_all(scratch("dc"));
    let failures = failures.into_inner().unwrap();
    let stale = stale.into_inner().unwrap();
    assert!(
        failures.is_empty(),
        "keys that change the generated code but not the DC operating point on their witness \
         card (a DC key the solution ignores, or one to list in DC_INERT with its reason): \
         {failures:?}"
    );
    assert!(
        stale.is_empty(),
        "DC_INERT / KNOWN_DC_DEFECTS keys that DO move the operating point (drop them from \
         the list; a fixed defect leaves KNOWN_DC_DEFECTS): {stale:?}"
    );
}
