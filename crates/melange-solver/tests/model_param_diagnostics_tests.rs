//! `.model` unrecognized-parameter diagnostics: who reports a typo'd key, and
//! who used to say nothing.
//!
//! Before the central table in `src/model_params.rs`, three separate checks
//! disagreed about the same question:
//!
//! * a **referenced** diode/BJT/JFET/MOSFET/tube/VCA/LDR/glow card → hard error
//!   from the codegen resolver, naming the accepted keys;
//! * a **referenced op-amp** card → `log::warn!` from `mna.rs`, against a key
//!   list maintained separately from every other one;
//! * an **unreferenced** card → *nothing at all*, on any path. `.model 2N3904
//!   NPN(ZORP=5)` with no `Q` using it compiled clean. That silence is the
//!   damaging case: a deck author who sees one card report a typo learns to
//!   read silence on the others as approval.
//!
//! These tests pin the outcome of each path. The key sets they are checked
//! against are guarded separately by `model_param_table_drift_tests.rs`.

mod support;

use std::sync::{Mutex, Once, OnceLock};

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ── Warning capture ─────────────────────────────────────────────────────
//
// `log` allows exactly one global logger per process, so the capture buffer is
// global and tests serialize on `capture_lock()` while they read it.

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

/// Install the capture logger, take the serialization lock, and clear the
/// buffer. Returns the guard; warnings emitted while it is held are captured.
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

fn unrecognized_warnings() -> Vec<String> {
    warnings()
        .into_iter()
        .filter(|w| w.contains("unrecognized parameter"))
        .collect()
}

// ── Decks ───────────────────────────────────────────────────────────────

/// An RC deck with no active device, so any `.model` card in it is an orphan.
fn orphan_deck(model_card: &str) -> String {
    format!("Orphan model card probe\nR1 in out 4.7k\nC1 out 0 10n\n{model_card}\n.END\n")
}

const OPAMP_DECK: &str = "\
Op-amp model card probe
.model OA1 OA(AOL=1e5 ROUT=75 BANANA=999 VSATT=4.5)
Rin in inv 10k
Rfb inv out 100k
U1 0 inv out OA1
Rload out 0 47k
.END
";

/// The VCA isolation deck's shape, with `THD` present on the card. `THD` is
/// read by the codegen VCA resolver, so it is honored — but the `mna.rs` match
/// arm did not know about it and reported it as unrecognized.
const VCA_DECK: &str = "\
VCA model card probe
.model VCA2180 VCA(VSCALE=0.05298 G0=1.0 THD=0.002 MODE=1)
.model OA1 OA(AOL=100000 ROUT=100 VSAT=13 GBW=10MEG)
Rdrive in vca_in 27K
Vctrl vca_ctrl 0 DC 0.1
Rpull vca_ctrl 0 1MEG
Y1 vca_in iv_inv vca_ctrl 0 VCA2180
U1 0 iv_inv iv_out OA1
Rfb iv_out iv_inv 15K
Rout iv_out out 100
Rload out 0 47K
.END
";

const DIODE_DECK: &str = "\
Diode clipper probe
R1 in out 4.7k
D1 out 0 DTEST
D2 0 out DTEST
C1 out 0 1n
.model DTEST D(IS=2.52e-9 N=1.752 RSS=100)
.END
";

// ── Orphan cards: the silent path ───────────────────────────────────────

#[test]
fn orphan_bjt_card_reports_an_unknown_key() {
    let _guard = start_capture();
    Netlist::parse(&orphan_deck(".model 2N3904 NPN(IS=1e-14 BF=200 ZORP=5)")).expect("parse");
    let warns = unrecognized_warnings();
    assert_eq!(
        warns.len(),
        1,
        "expected exactly one unrecognized-parameter warning, got {warns:?}"
    );
    assert!(
        warns[0].contains("2N3904") && warns[0].contains("ZORP"),
        "warning does not name the card and the key: {warns:?}"
    );
}

#[test]
fn orphan_diode_card_reports_an_rs_typo() {
    // `RSS` for `RS` on a diode — the most ordinary typo on the most common
    // nonlinear part there is.
    let _guard = start_capture();
    Netlist::parse(&orphan_deck(
        ".model 1N4148 D(IS=2.52e-9 N=1.752 RSS=100 FLORB=7)",
    ))
    .expect("parse");
    let warns = unrecognized_warnings();
    assert_eq!(warns.len(), 2, "expected two warnings, got {warns:?}");
    assert!(warns.iter().any(|w| w.contains("RSS")), "{warns:?}");
    assert!(warns.iter().any(|w| w.contains("FLORB")), "{warns:?}");
}

#[test]
fn orphan_card_with_only_honored_keys_is_silent() {
    let _guard = start_capture();
    Netlist::parse(&orphan_deck(
        ".model 1N4148 D(IS=2.52e-9 N=1.752 RS=0.6 CJO=4p KF=1e-16 AF=1)",
    ))
    .expect("parse");
    assert!(
        unrecognized_warnings().is_empty(),
        "honored keys warned: {:?}",
        unrecognized_warnings()
    );
}

#[test]
fn orphan_card_of_an_unknown_type_is_silent() {
    // melange has no table for this type, so it has no opinion about the keys.
    // Guessing here would mean warning about every key of a device class
    // melange does not model — noise, not a diagnostic.
    let _guard = start_capture();
    Netlist::parse(&orphan_deck(".model SW1 ZZTOP(RON=1 ROFF=1e9)")).expect("parse");
    assert!(
        unrecognized_warnings().is_empty(),
        "warned about an unknown model type: {:?}",
        unrecognized_warnings()
    );
}

#[test]
fn orphan_bjt_card_recognized_but_unimplemented_key_is_not_reported_as_unknown() {
    // `TR` is a real SPICE parameter melange does not model; the codegen
    // resolver reports it with the cost of the omission. The orphan pass must
    // not relabel it "unrecognized".
    let _guard = start_capture();
    Netlist::parse(&orphan_deck(".model Q1 NPN(IS=1e-14 BF=200 TR=1n)")).expect("parse");
    assert!(
        unrecognized_warnings().is_empty(),
        "unimplemented key reported as unrecognized: {:?}",
        unrecognized_warnings()
    );
}

// ── Referenced cards ────────────────────────────────────────────────────

#[test]
fn referenced_opamp_card_still_warns_exactly_as_before() {
    let _guard = start_capture();
    let netlist = Netlist::parse(OPAMP_DECK).expect("parse");
    MnaSystem::from_netlist(&netlist).expect("mna");
    let warns = unrecognized_warnings();
    assert_eq!(warns.len(), 2, "expected two warnings, got {warns:?}");
    assert!(warns.iter().any(|w| w.contains("BANANA")), "{warns:?}");
    assert!(warns.iter().any(|w| w.contains("VSATT")), "{warns:?}");
    // Same format string as before the table landed.
    assert!(
        warns
            .iter()
            .all(|w| w.starts_with(".model OA1: unrecognized parameter '")
                && w.ends_with("(ignored)")),
        "warning format changed: {warns:?}"
    );
    // And the card is still accepted — an op-amp typo warns, it does not refuse.
    assert!(
        warns
            .iter()
            .all(|w| !w.contains("AOL") && !w.contains("ROUT")),
        "honored op-amp keys warned: {warns:?}"
    );
}

#[test]
fn referenced_vca_card_does_not_warn_about_honored_thd() {
    // Regression: the `mna.rs` VCA match arm knew VSCALE/G0/MODE only, so it
    // reported `THD` — which the codegen VCA resolver reads — as unrecognized.
    let _guard = start_capture();
    let netlist = Netlist::parse(VCA_DECK).expect("parse");
    MnaSystem::from_netlist(&netlist).expect("mna");
    assert!(
        unrecognized_warnings().is_empty(),
        "honored VCA key warned: {:?}",
        unrecognized_warnings()
    );
}

#[test]
fn referenced_diode_card_typo_is_still_a_hard_error() {
    // Unchanged behaviour, pinned here because the orphan pass must not have
    // downgraded it to a warning: a card a device actually uses is refused.
    let netlist = Netlist::parse(DIODE_DECK).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let config = CodegenConfig {
        circuit_name: "diode_probe".to_string(),
        sample_rate: 48000.0,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let input_node = config.input_node;
    mna.g[input_node][input_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, config.sample_rate).expect("dk kernel");
    let err = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect_err("a typo'd key on a referenced diode card must be refused");
    let msg = err.to_string();
    assert!(
        msg.contains("unknown parameter 'RSS'"),
        "error does not name the key: {msg}"
    );
    assert!(
        msg.contains("Accepted for this device"),
        "error does not list the accepted keys: {msg}"
    );
}
