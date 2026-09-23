//! Parse-time diagnostics: does an error say **where**, and does a rejection
//! say **why**?
//!
//! Three defects are pinned here, all from a cold first-user report:
//!
//! * every post-parse validation error reported `line 0` — the messages named
//!   the component and the accepted set, but claimed a location that does not
//!   exist. Fine on a 30-line pedal; on a 300-line amp deck it means counting
//!   lines by hand;
//! * the line counter counted *processed* lines, so continuation (`+`) joining
//!   silently shifted every subsequent error's line number;
//! * a value the parser did not recognize was rejected with no reason at all,
//!   in a parser whose other messages are its best feature.
//!
//! The no-ground warning is here too: a deck that never mentions node `0`
//! solved, converged, and reported a KCL residual of ~1e-19 with nothing said.

use std::sync::{Mutex, Once, OnceLock};

use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ── Warning capture ─────────────────────────────────────────────────────
//
// `log` allows one global logger per process; the buffer is global and tests
// serialize on `TEST_LOCK` while they read it.

static CAPTURED: OnceLock<Mutex<Vec<String>>> = OnceLock::new();
static TEST_LOCK: OnceLock<Mutex<()>> = OnceLock::new();
static INIT: Once = Once::new();
static LOGGER: CaptureLogger = CaptureLogger;

struct CaptureLogger;

impl log::Log for CaptureLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= log::Level::Warn
    }
    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            buffer().lock().unwrap().push(record.args().to_string());
        }
    }
    fn flush(&self) {}
}

fn buffer() -> &'static Mutex<Vec<String>> {
    CAPTURED.get_or_init(|| Mutex::new(Vec::new()))
}

fn start_capture() -> std::sync::MutexGuard<'static, ()> {
    INIT.call_once(|| {
        log::set_logger(&LOGGER).expect("no other logger in this binary");
        log::set_max_level(log::LevelFilter::Warn);
    });
    let guard = TEST_LOCK
        .get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    buffer().lock().unwrap().clear();
    guard
}

fn warnings() -> Vec<String> {
    buffer().lock().unwrap().clone()
}

// ── Line numbers ────────────────────────────────────────────────────────

/// Undefined `.model` reference: the reported line must be the element's.
#[test]
fn undefined_model_reference_reports_the_element_line() {
    // 1 title / 2 R1 / 3 D1 / 4 R2 / 5 C1
    let deck = "\
Undefined model probe
R1 in a 1k
D1 a out 1N4148
R2 out 0 10k
C1 out 0 10n
";
    let err = Netlist::parse(deck).expect_err("undefined model must be rejected");
    assert_eq!(err.line, 3, "{}", err);
    assert!(err.to_string().contains("line 3"), "{}", err);
}

/// Model/element type mismatch, and the line must survive continuation
/// joining: the `.model` card below occupies three raw lines but one
/// *processed* line, which is exactly where the old counter drifted.
#[test]
fn line_number_survives_continuation_joining() {
    // 1 title / 2 R1 / 3-5 .model (joined) / 6 D1 / 7 R2 / 8 C1
    let deck = "\
Continuation drift probe
R1 in a 1k
.model 1N4148 NPN(IS=2.52e-9
+ BF=200
+ VAF=100)
D1 a out 1N4148
R2 out 0 10k
C1 out 0 10n
";
    let err = Netlist::parse(deck).expect_err("diode bound to an NPN card must be rejected");
    // Counting processed lines gives 4 here. The raw source line is 6.
    assert_eq!(err.line, 6, "{}", err);
}

/// A syntactic error *after* a continuation block: same drift, different path
/// (`Parser::error`, which reads `line_num` directly).
#[test]
fn syntax_error_after_a_continuation_block_reports_the_raw_line() {
    // 1 title / 2-4 .model (joined) / 5 R1 / 6 D1 / 7 R2(bad) / 8 C1
    let deck = "\
Continuation drift probe 2
.model 1N4148 D(IS=2.52e-9
+ N=1.752
+ RS=0.568)
R1 in a 1k
D1 a out 1N4148
R2 out 0 banana
C1 out 0 10n
";
    let err = Netlist::parse(deck).expect_err("'banana' is not a resistance");
    assert_eq!(err.line, 7, "{}", err);
}

/// A directive error points at the directive, not at line 0 and not at the
/// component it names (which does not exist — that is the error).
#[test]
fn pot_target_typo_reports_the_directive_line() {
    // 1 title / 2 R1 / 3 R_tone / 4 C1 / 5 .pot
    let deck = "\
Pot target probe
R1 in out 1k
R_tone out 0 10k
C1 out 0 10n
.pot R_tonne 1k 100k \"Tone\"
";
    let err = Netlist::parse(deck).expect_err(".pot target must exist");
    assert_eq!(err.line, 5, "{}", err);
    assert!(err.message.contains("R_tonne"), "{}", err);
}

/// Duplicate names must point at the *second* declaration — the one to delete.
#[test]
fn duplicate_component_name_reports_the_second_declaration() {
    // 1 title / 2 R1 / 3 C1 / 4 R1 (dup)
    let deck = "\
Duplicate name probe
R1 in out 1k
C1 out 0 10n
R1 out 0 2k
";
    let err = Netlist::parse(deck).expect_err("duplicate name must be rejected");
    assert_eq!(err.line, 4, "{}", err);
}

/// `.model` range validation reports the card's own line.
#[test]
fn model_parameter_range_error_reports_the_card_line() {
    // 1 title / 2 R1 / 3 C1 / 4 .model
    let deck = "\
Model range probe
R1 in a 1k
C1 a 0 10n
.model DX D(IS=-1e-9)
D1 a 0 DX
";
    let err = Netlist::parse(deck).expect_err("negative IS must be rejected");
    assert_eq!(err.line, 4, "{}", err);
}

/// `line == 0` means "no single line is responsible" and must NOT print as a
/// location. A whole-file size cap is the honest example.
#[test]
fn display_omits_the_location_when_no_line_is_known() {
    let oversized = "x".repeat(melange_solver::parser::MAX_NETLIST_BYTES + 1);
    let err = match Netlist::parse(&oversized) {
        Err(e) => e,
        Ok(_) => panic!("oversized netlist must be rejected"),
    };
    assert_eq!(err.line, 0);
    let text = err.to_string();
    assert!(
        !text.contains("line 0"),
        "a line-less error still claims a location: {text}"
    );
    assert!(text.starts_with("Parse error: "), "{text}");
}

// ── Rejected values ─────────────────────────────────────────────────────

/// `1R5` / `10R` are BS-1852, like the `4k7` melange accepts — so the
/// rejection has to say why it is not symmetric, and what to write instead.
///
/// Reason (measured against system ngspice): ngspice reads `1R5` as 1 Ω and
/// `10R` as 10 Ω, silently, because `R` is not a scale letter and it discards
/// the rest of the token. `melange validate` hands the author's own deck to
/// ngspice, so accepting `1R5` as 1.5 Ω would have the two engines simulate
/// different circuits.
#[test]
fn bs1852_ohms_marker_rejection_explains_itself() {
    for (raw, intended) in [("1R5", "1.5"), ("10R", "10"), ("4R7", "4.7")] {
        let deck = format!("BS1852 probe\nR1 in out {raw}\nC1 out 0 10n\nR2 out 0 1k\n");
        let err = Netlist::parse(&deck).expect_err("BS-1852 ohms marker must be rejected");
        assert_eq!(err.line, 2, "{}", err);
        let m = &err.message;
        assert!(m.contains(raw), "message does not quote the token: {m}");
        assert!(
            m.contains(&format!("'{intended}'")),
            "message does not suggest the value to write: {m}"
        );
        assert!(m.contains("ngspice"), "message does not give the reason: {m}");
    }
}

/// The generic case still has to say something — a value error that reads
/// "Failed to parse" and nothing else is the one place this parser went quiet.
#[test]
fn unrecognized_value_names_the_accepted_forms() {
    let deck = "Bad value probe\nR1 in out banana\nC1 out 0 10n\nR2 out 0 1k\n";
    let err = Netlist::parse(deck).expect_err("'banana' is not a resistance");
    assert_eq!(err.line, 2, "{}", err);
    let m = &err.message;
    assert!(m.contains("'banana'"), "{m}");
    assert!(m.contains("4k7"), "accepted forms not listed: {m}");
}

/// The accepted-forms sentence must not contradict the parser it describes.
/// Two traps it used to walk straight into:
///
/// * a trailing `f` in an ELEMENT value is the Farad unit, not femto — the
///   parser already warns about exactly that, so listing `f` as a scale suffix
///   would have one melange message teach the mistake another one flags;
/// * `10kohm` is a hard parse error (the unit-letter strip set is F/H/V/A/S/Z,
///   so `10KOHM` reduces to `10KOH`), which is the single most plausible thing
///   an author types next after reading "optionally followed by a unit letter".
#[test]
fn accepted_forms_sentence_matches_the_parser() {
    use melange_solver::parser::parse_value;

    let deck = "Bad value probe\nR1 in out banana\nC1 out 0 10n\nR2 out 0 1k\n";
    let m = Netlist::parse(deck).expect_err("reject").message;

    // Ground truth, measured here rather than assumed.
    assert_eq!(parse_value("10f").expect("10f parses"), 10.0, "10f is Farad");
    assert!((parse_value("10fF").expect("10fF parses") - 10e-15).abs() < 1e-20);
    assert!(parse_value("10kohm").is_err(), "10kohm must still be an error");
    for good in ["10pF", "4.7uF", "100nH", "10kHz", "9V", "4k7", "6n8", "1meg"] {
        assert!(parse_value(good).is_ok(), "{good} must parse");
    }

    assert!(
        m.contains("trailing 'f' is the Farad unit"),
        "message does not warn about the femto trap: {m}"
    );
    assert!(
        m.contains("10kohm"),
        "message does not warn that 'ohm' is not a unit: {m}"
    );
    // The scale-suffix list must not offer `f` as a scale in element position.
    assert!(
        !m.contains("u/µ n p f"),
        "'f' is listed as an element-value scale suffix: {m}"
    );
}

/// The infix forms melange *does* accept must keep working — this is the
/// behaviour the same reporter called out as a delight.
#[test]
fn infix_scale_forms_still_parse() {
    use melange_solver::parser::Element;
    let deck = "Infix probe\nR1 in out 4k7\nC1 out 0 6n8\nR2 out 0 1k\n";
    let netlist = Netlist::parse(deck).expect("4k7 / 6n8 must still parse");
    let r1 = netlist
        .elements
        .iter()
        .find_map(|e| match e {
            Element::Resistor { name, value, .. } if name.eq_ignore_ascii_case("R1") => {
                Some(*value)
            }
            _ => None,
        })
        .expect("R1");
    // 4k7 = 4700 Ω — melange's reading. (ngspice reads the same token as 4000;
    // see `explain_rejected_value` for why that divergence is not this
    // parser's to resolve.)
    assert!((r1 - 4700.0).abs() < 1e-9, "4k7 parsed as {r1}");
}

// ── No ground ───────────────────────────────────────────────────────────

#[test]
fn deck_with_no_ground_reference_warns() {
    let _guard = start_capture();
    let deck = "\
No ground probe
R1 in a 1k
R2 a b 1k
C1 a out 10n
R3 out b 10k
";
    Netlist::parse(deck).expect("a groundless deck still parses");
    assert!(
        warnings().iter().any(|w| w.contains("ground node '0'")),
        "no warning for a deck with no ground: {:?}",
        warnings()
    );
}

#[test]
fn deck_with_ground_is_silent() {
    let _guard = start_capture();
    let deck = "\
Grounded probe
R1 in out 1k
C1 out 0 10n
R2 out 0 10k
";
    Netlist::parse(deck).expect("parse");
    assert!(
        !warnings().iter().any(|w| w.contains("ground node '0'")),
        "false no-ground warning: {:?}",
        warnings()
    );
}

#[test]
fn gnd_alias_counts_as_ground() {
    let _guard = start_capture();
    let deck = "\
Gnd alias probe
R1 in out 1k
C1 out GND 10n
R2 out gnd 10k
";
    Netlist::parse(deck).expect("parse");
    assert!(
        !warnings().iter().any(|w| w.contains("ground node '0'")),
        "'gnd' should alias to ground: {:?}",
        warnings()
    );
}

// ── Deterministic node listing ──────────────────────────────────────────

/// Node lists printed at users must not be ordered by hash iteration. Three
/// runs of one failing command used to give three different orders, which
/// makes error output undiffable and unassertable.
#[test]
fn node_listing_order_is_stable_and_index_ordered() {
    let deck = "\
Node order probe
R1 in a 1k
R2 a b 1k
R3 b out 1k
C1 out 0 10n
";
    let netlist = Netlist::parse(deck).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let first = mna.node_names_in_index_order();
    // Ground first (index 0), then netlist appearance order.
    assert_eq!(first, vec!["0", "in", "a", "b", "out"]);
    // Rebuild from scratch: a fresh HashMap with a fresh random seed.
    for _ in 0..8 {
        let n2 = Netlist::parse(deck).expect("parse");
        let m2 = MnaSystem::from_netlist(&n2).expect("mna");
        assert_eq!(
            m2.node_names_in_index_order(),
            first,
            "node order is not stable across builds"
        );
    }
}

// ── Pipeline narration ──────────────────────────────────────────────────

/// "Skipping BJT internal-node expansion (K ill-conditioned)" used to print on
/// any deck whose K diagonal tripped the −100 gate — including the shipped
/// four-tube `passive-eq1a` demo, which contains no BJT at all. Read cold,
/// "Skipping" + "ill-conditioned" on the flagship example says *you broke it*.
///
/// The deck below is BJT-free. Whatever its conditioning, the expansion gate
/// must not narrate a decision about a device class it does not contain.
#[test]
fn bjt_free_deck_never_narrates_bjt_expansion() {
    use melange_solver::dk::DkKernel;
    use melange_solver::pipeline::expand_internal_nodes_if_conditioned;

    let deck = "\
BJT-free clipper
R1 in a 4.7k
D1 a 0 DTEST
D2 0 a DTEST
C1 a out 100n
R2 out 0 10k
.model DTEST D(IS=2.52e-9 N=1.752)
";
    let netlist = Netlist::parse(deck).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let kernel = DkKernel::from_mna(&mna, 48_000.0).expect("kernel");

    let lines = std::sync::Mutex::new(Vec::<String>::new());
    let rep = |a: std::fmt::Arguments<'_>| lines.lock().unwrap().push(a.to_string());
    expand_internal_nodes_if_conditioned(&mut mna, &netlist, &kernel, &rep);

    let printed = lines.lock().unwrap().clone();
    assert!(
        !printed.iter().any(|l| l.contains("BJT")),
        "BJT expansion narrated on a BJT-free deck: {printed:?}"
    );
}

// ── `melange nodes`-grade .model key check ──────────────────────────────
//
// `nodes` never builds the codegen IR, so the resolvers' hard error on an
// unknown `.model` key never fires there: `.model 1N4148 D(RSS=100)` — `RS`
// mistyped on the most common nonlinear part in a pedal — was reported by
// nothing, while the op-amp card two lines down warned from the MNA build.
// `warn_unknown_keys_on_referenced_models` is the helper that closes it; the
// single call site belongs to `melange nodes` in the CLI crate.

#[test]
fn referenced_model_key_helper_reports_a_diode_typo() {
    let _guard = start_capture();
    let deck = "\
Referenced diode card
R1 in out 4.7k
D1 out 0 DTEST
D2 0 out DTEST
C1 out 0 1n
.model DTEST D(IS=2.52e-9 N=1.752 RSS=100)
";
    let netlist = Netlist::parse(deck).expect("parse");
    melange_solver::model_params::warn_unknown_keys_on_referenced_models(&netlist);
    let warns: Vec<String> = warnings()
        .into_iter()
        .filter(|w| w.contains("unrecognized parameter"))
        .collect();
    // TWO elements share the card; the key is reported once, not twice.
    assert_eq!(warns.len(), 1, "expected exactly one warning, got {warns:?}");
    assert!(
        warns[0].contains("DTEST") && warns[0].contains("RSS"),
        "{warns:?}"
    );
}

#[test]
fn referenced_model_key_helper_skips_opamps_and_vcas() {
    // `nodes` builds the MNA, whose op-amp / VCA resolution loops already warn
    // from the same table. Warning here too would print every such line twice.
    let _guard = start_capture();
    let deck = "\
Referenced op-amp card
.model OA1 OA(AOL=1e5 ROUT=75 BANANA=999)
Rin in inv 10k
Rfb inv out 100k
U1 0 inv out OA1
Rload out 0 47k
";
    let netlist = Netlist::parse(deck).expect("parse");
    melange_solver::model_params::warn_unknown_keys_on_referenced_models(&netlist);
    assert!(
        warnings()
            .iter()
            .all(|w| !w.contains("unrecognized parameter")),
        "op-amp card double-reported: {:?}",
        warnings()
    );
}

#[test]
fn referenced_model_key_helper_is_silent_on_honored_keys() {
    let _guard = start_capture();
    let deck = "\
Honored keys only
R1 in out 4.7k
D1 out 0 DTEST
C1 out 0 1n
.model DTEST D(IS=2.52e-9 N=1.752 RS=0.6 CJO=4p KF=1e-16 AF=1)
";
    let netlist = Netlist::parse(deck).expect("parse");
    melange_solver::model_params::warn_unknown_keys_on_referenced_models(&netlist);
    assert!(
        warnings()
            .iter()
            .all(|w| !w.contains("unrecognized parameter")),
        "honored keys warned: {:?}",
        warnings()
    );
}

/// The alias hint now lives with the tables, so the *warning* path carries the
/// same pointed explanation the hard-error path always had. Before the move a
/// `VP` on an unreferenced JFET card said only "unrecognized parameter", while
/// the identical key on a referenced card explained the VTO confusion and the
/// sign trap — the same mistake answered two different ways depending on
/// whether some other line of the deck happened to use the card.
#[test]
fn jfet_vp_warning_carries_the_vto_alias_hint() {
    use melange_solver::model_params::{alias_hint, ModelClass};
    assert!(alias_hint(ModelClass::Jfet, "VP").contains("VTO"));
    assert!(alias_hint(ModelClass::Jfet, "vp").contains("VTO"));
    assert!(alias_hint(ModelClass::Mosfet, "VP").contains("VTO"));
    // Not a JFET key elsewhere — no hint to give.
    assert_eq!(alias_hint(ModelClass::Diode, "VP"), "");
    assert_eq!(alias_hint(ModelClass::Jfet, "ZORP"), "");

    let _guard = start_capture();
    let deck = "\
Orphan JFET card
R1 in out 4.7k
C1 out 0 10n
.model J2N5457 NJF(VP=-2.5)
";
    Netlist::parse(deck).expect("parse");
    let warns: Vec<String> = warnings()
        .into_iter()
        .filter(|w| w.contains("unrecognized parameter"))
        .collect();
    assert_eq!(warns.len(), 1, "{warns:?}");
    assert!(
        warns[0].contains("VTO"),
        "warning lacks the alias hint the hard error gives: {warns:?}"
    );
}

// ── Suffix 'M' is milli, and says so ────────────────────────────────────
//
// SPICE reads a bare `M` suffix as MILLI. melange must keep doing so — ngspice
// reads it the same way, and `melange validate` hands the author's deck straight
// to ngspice, so reinterpreting `1M` as mega would make the two engines simulate
// different circuits. The defect was that melange warned on the *correct* form
// (`1M0`, infix, mega) and was silent on the catastrophic one, which is 10^9 out.

/// The value must NOT change: `1M` stays milli on every path.
#[test]
fn suffix_m_still_parses_as_milli_and_meg_is_unaffected() {
    use melange_solver::parser::parse_value;
    for (raw, expected) in [
        ("1M", 1e-3),
        ("1m", 1e-3),
        ("4M", 4e-3),
        ("1meg", 1e6),
        ("1MEG", 1e6),
        ("1M0", 1e6),
        ("2M2", 2.2e6),
    ] {
        let got = parse_value(raw).unwrap_or_else(|_| panic!("'{raw}' must parse"));
        assert!(
            (got - expected).abs() <= expected.abs() * 1e-12,
            "'{raw}' must parse as {expected:e}, got {got:e}"
        );
    }
}

/// An uppercase `M` suffix warns; lowercase `m` does not, because that is how
/// milli is actually authored. `meg` and the infix forms must stay quiet on this
/// particular warning.
#[test]
fn uppercase_m_suffix_warns_but_lowercase_m_does_not() {
    use melange_solver::parser::parse_value;
    // `start_capture` takes a non-reentrant global lock, so each phase gets its
    // own scope and releases the guard before the next one asks for it.
    {
        let _guard = start_capture();
        parse_value("1M").expect("'1M' parses");
        let warned: Vec<String> = warnings();
        assert!(
            warned.iter().any(|w| w.contains("is MILLI in SPICE")),
            "uppercase 'M' suffix must warn, got: {warned:?}"
        );
        assert!(
            warned.iter().any(|w| w.contains("1meg")),
            "the warning must name the unambiguous spelling, got: {warned:?}"
        );
    }

    for quiet in ["1m", "10m", "1meg", "1MEG"] {
        let _guard = start_capture();
        parse_value(quiet).unwrap_or_else(|_| panic!("'{quiet}' parses"));
        let warned: Vec<String> = warnings();
        assert!(
            !warned.iter().any(|w| w.contains("is MILLI in SPICE")),
            "'{quiet}' must not raise the suffix-M warning, got: {warned:?}"
        );
    }
}

/// Both readings of the ambiguous letter are now reported. Before this, melange
/// warned only on the reading that was correct.
#[test]
fn both_readings_of_m_are_reported() {
    use melange_solver::parser::parse_value;
    {
        let _guard = start_capture();
        parse_value("1M0").expect("'1M0' parses");
        let warned: Vec<String> = warnings();
        assert!(
            warned.iter().any(|w| w.contains("MEGA")),
            "infix M must still warn: {warned:?}"
        );
    }
    {
        let _guard = start_capture();
        parse_value("1M").expect("'1M' parses");
        let warned: Vec<String> = warnings();
        assert!(
            warned.iter().any(|w| w.contains("MILLI")),
            "suffix M must warn too: {warned:?}"
        );
    }
}
