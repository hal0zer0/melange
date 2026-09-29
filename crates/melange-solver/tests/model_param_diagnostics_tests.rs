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

/// `GBW` is parsed, but no solver path models a bandwidth pole: the gain is
/// `AOL` at every frequency and `GBW` only defaults the rails. A card that
/// authors it gets a notice naming the op-amp; a card without it gets none.
#[test]
fn authored_gbw_says_it_is_not_a_pole() {
    let gbw_notices = || -> Vec<String> {
        warnings()
            .into_iter()
            .filter(|w| w.contains("GBW is not modelled"))
            .collect()
    };
    let _guard = start_capture();
    let with_gbw = OPAMP_DECK.replace("BANANA=999 VSATT=4.5", "GBW=1k");
    MnaSystem::from_netlist(&Netlist::parse(&with_gbw).expect("parse")).expect("mna");
    let notices = gbw_notices();
    assert_eq!(notices.len(), 1, "expected one GBW notice, got {notices:?}");
    assert!(
        notices[0].contains("U1"),
        "notice does not name the op-amp: {notices:?}"
    );

    buffer().lock().unwrap().clear();
    let without = OPAMP_DECK.replace("BANANA=999 VSATT=4.5", "VSAT=13");
    MnaSystem::from_netlist(&Netlist::parse(&without).expect("parse")).expect("mna");
    assert!(
        gbw_notices().is_empty(),
        "notice without GBW: {:?}",
        warnings()
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

// ── Retired triode grid-current keys ────────────────────────────────────
//
// `IG_MAX` / `VGK_ONSET` parameterised the Leach grid law that Dempwolf &
// Zölzer eq. (11) replaced. They are refused, not aliased and not repurposed:
// an alias preserves a name that was wrong about its own meaning, and the same
// key silently meaning something else is the class of bug melange exists not to
// have. The refusal carries the conversion so an author who wants the old curve
// can ask for it deliberately.

/// A 12AX7 common-cathode stage; `extra` goes on the `.model` card.
fn triode_deck(extra: &str) -> String {
    format!(
        "Triode grid-law probe\n\
         .model ECC83 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300{extra})\n\
         Rg in g 68k\n\
         T1 g p k ECC83\n\
         Rp vcc p 100k\n\
         Rk k 0 1.5k\n\
         Ck k 0 22u\n\
         Vcc vcc 0 250\n\
         Cout p out 22n\n\
         Rl out 0 1meg\n\
         .END\n"
    )
}

fn compile_triode(extra: &str) -> Result<String, String> {
    let src = triode_deck(extra);
    let netlist = Netlist::parse(&src).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let config = CodegenConfig {
        circuit_name: "triode_probe".to_string(),
        sample_rate: 48000.0,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let input_node = config.input_node;
    mna.g[input_node][input_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, config.sample_rate).expect("dk kernel");
    CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .map(|r| r.code)
        .map_err(|e| e.to_string())
}

#[test]
fn retired_ig_max_is_refused_with_its_conversion() {
    let msg = compile_triode(" IG_MAX=5e-3")
        .expect_err("a retired key on a referenced triode card must be refused");
    assert!(msg.contains("'IG_MAX' is RETIRED"), "{msg}");
    assert!(
        msg.contains("IG_MAX/VGK_ONSET^1.5"),
        "the refusal must print the conversion to the new law: {msg}"
    );
    assert!(
        !msg.contains("unknown parameter"),
        "a retired key is not a typo and must not be reported as one: {msg}"
    );
}

#[test]
fn retired_vgk_onset_is_refused_and_says_it_was_never_the_onset() {
    let msg = compile_triode(" VGK_ONSET=0.75").expect_err("VGK_ONSET must be refused");
    assert!(msg.contains("'VGK_ONSET' is RETIRED"), "{msg}");
    assert!(
        msg.contains("never the onset"),
        "the refusal must say what the key actually was: {msg}"
    );
}

#[test]
fn retired_keys_are_still_honored_on_a_pentode_card() {
    // Deliberate asymmetry: the pentode CONTROL grid still carries the Leach
    // law, because no published fit of the D&Z form exists for one and melange
    // does not invent device parameters. The rule follows the laws, not the
    // spelling of the key.
    let deck = "Pentode grid-law probe\n\
                .model EL84P VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 KP=152.4 KVB=4015.8 \
                ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148 IG_MAX=8m VGK_ONSET=0.7)\n\
                Rg in g 68k\n\
                P1 p g k s EL84P\n\
                Rp vcc p 5k\n\
                Rs vcc s 1k\n\
                Cs s 0 47u\n\
                Rk k 0 150\n\
                Ck k 0 100u\n\
                Vcc vcc 0 300\n\
                Cout p out 22n\n\
                Rl out 0 1meg\n\
                .END\n";
    let netlist = Netlist::parse(deck).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let config = CodegenConfig {
        circuit_name: "pentode_probe".to_string(),
        sample_rate: 48000.0,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let input_node = config.input_node;
    mna.g[input_node][input_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, config.sample_rate).expect("dk kernel");
    let code = CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("IG_MAX/VGK_ONSET must still compile on a pentode card")
        .code;
    assert!(
        code.contains("const DEVICE_0_IG_MAX"),
        "pentode lost IG_MAX"
    );
}

#[test]
fn new_grid_keys_are_accepted_and_reach_the_generated_constants() {
    // D&Z Table 1 row RSD-2, spelled out on the card.
    let code = compile_triode(" GG=5.911e-4 XI=1.358 CG=11.76").expect("GG/XI/CG must compile");
    // Emitted normalized to one leading digit at full precision, so CG=11.76
    // prints as `1.17599999999999998e1` — match the mantissa, not the decimal
    // spelling on the card.
    assert!(
        code.contains("const DEVICE_0_GG: f64 = 5.911"),
        "GG missing"
    );
    assert!(
        code.contains("const DEVICE_0_XI: f64 = 1.358"),
        "XI missing"
    );
    assert!(
        code.contains("const DEVICE_0_CG: f64 = 1.1759"),
        "CG missing; emitted: {:?}",
        code.lines()
            .filter(|l| l.starts_with("const DEVICE_0_"))
            .collect::<Vec<_>>()
    );
    assert!(
        !code.contains("DEVICE_0_IG_MAX"),
        "a triode must not emit a Leach constant"
    );
}

// ── Compile-time grid-current starting-point check ──────────────────────

#[test]
fn onset_outside_the_per_type_philips_bracket_warns() {
    let _guard = start_capture();
    // Slack turn-on (small CG) pushes the derived 0.3 uA starting point to
    // -1.568 V, past the ECC83's published max of -0.9 V.
    compile_triode(" CG=3").expect("compiles — the check warns, it does not refuse");
    let warns: Vec<String> = warnings()
        .into_iter()
        .filter(|w| w.contains("grid-current starting point"))
        .collect();
    assert_eq!(
        warns.len(),
        1,
        "expected exactly one distinct warning text: {warns:?}"
    );
    assert!(
        warns[0].contains("BELOW the manufacturer limit"),
        "{:?}",
        warns[0]
    );
    assert!(
        warns[0].contains("-0.9 V"),
        "must name the type's own limit: {:?}",
        warns[0]
    );
}

#[test]
fn onset_inside_the_bracket_does_not_warn() {
    let _guard = start_capture();
    compile_triode("").expect("the shipped default must compile clean");
    let warns: Vec<String> = warnings()
        .into_iter()
        .filter(|w| w.contains("grid-current starting point"))
        .collect();
    assert!(
        warns.is_empty(),
        "the shipped RSD-1 row sits at -0.353 V, well inside the ECC83 bracket: {warns:?}"
    );
}
