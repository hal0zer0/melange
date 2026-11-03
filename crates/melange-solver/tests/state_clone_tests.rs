//! The generated `CircuitState` clones field by field: `clone_from` copies
//! every field in place, the cold matrices into the box the target already
//! owns, so a plugin that shares one solve while its channels are identical can
//! split the state on the audio thread without allocating. Proven on both
//! routes with a counting global allocator, a full `Debug` comparison of the
//! two states after the copy, and identical outputs from then on.
mod support;

/// A nodal deck with a boxed cold block (full-LU: the coupled transformer
/// gives it a negative K diagonal), pots and a switch.
const TRANSFORMER_BJT: &str = "\
Transformer-loaded BJT stage
VCC vcc 0 DC 12
Rin in b 10k
Rb1 vcc b 47k
Rb2 b 0 10k
Q1 c b e QN
Re e 0 1k
Ce e 0 100u
L1 vcc c 1
L2 out 0 0.25
K1 L1 L2 0.99
Rl out 0 600
Rsw out 0 100k
.switch Rsw 100k 10k \"Load\"
.pot Re 500 2k 1k \"Bias\"
.model QN NPN(IS=1e-14 BF=200)
";

/// A DK deck with a diode pair and a pot.
const DIODE_CLIPPER: &str = "\
Diode clipper
R1 in mid 4.7k
D1 mid 0 DSW
D2 0 mid DSW
C1 mid 0 10n
R2 mid out 1k
.pot R1 1k 10k 4.7k \"Drive\"
.model DSW D(IS=2.52n N=1.752)
";

const MAIN: &str = r#"
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};
struct Counting;
static ALLOCS: AtomicUsize = AtomicUsize::new(0);
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) }
    }
}
#[global_allocator]
static GA: Counting = Counting;

fn main() {
    let mut a = CircuitState::default();
    let mut b = CircuitState::default();
    // Drive the two states apart, with a control move on one of them.
    a.set_pot_0(POT_0_MAX_R);
    for i in 0..2000 {
        let x = 0.5 * (i as f64 * 0.1).sin();
        let _ = process_sample(x, &mut a);
        let _ = process_sample(-x, &mut b);
    }
    let before = ALLOCS.load(Ordering::SeqCst);
    b.clone_from(&a);
    let allocs = ALLOCS.load(Ordering::SeqCst) - before;
    println!("allocs={allocs}");
    println!("same_debug={}", format!("{a:?}") == format!("{b:?}"));
    let mut same_out = true;
    for i in 0..500 {
        let x = 0.3 * (i as f64 * 0.2).sin();
        if process_sample(x, &mut a) != process_sample(x, &mut b) {
            same_out = false;
        }
    }
    println!("same_out={same_out}");
    let c = a.clone();
    println!("clone_same={}", format!("{a:?}") == format!("{c:?}"));
}
"#;

fn check(code: &str, tag: &str) {
    assert!(
        code.contains("impl Clone for CircuitState {")
            && !code.contains("#[derive(Clone, Debug)]\npub struct CircuitState {"),
        "{tag}: the state's Clone is written out, not derived"
    );
    // Every field of the struct is assigned by clone_from (the cold block's
    // fields through the box).
    let fields = |name: &str| -> usize {
        let open = format!("pub struct {name} {{\n");
        let Some(start) = code.find(&open) else {
            return 0;
        };
        code[start + open.len()..]
            .lines()
            .take_while(|l| *l != "}")
            .map(|l| l.trim())
            .filter(|t| {
                !t.is_empty()
                    && !t.starts_with("//")
                    && !t.starts_with('#')
                    && t.ends_with(',')
                    && t.contains(':')
            })
            .count()
    };
    let state_fields = fields("CircuitState");
    let cold_fields = fields("CircuitStateCold");
    let has_cold = code.contains("pub cold: Box<CircuitStateCold>,");
    let cf = code
        .find("fn clone_from(&mut self, source: &Self) {")
        .expect("clone_from");
    let body = &code[cf..];
    let body = &body[..body.find("\n    }\n").unwrap()];
    // Every top-level field appears on the left of an assignment (a struct
    // array's elements as `self.f[i].x`), so count distinct field names, and
    // every cold field through the box.
    let mut assigned: Vec<&str> = body
        .lines()
        .filter_map(|l| l.trim_start().strip_prefix("self."))
        .map(|r| r.split([' ', '[', '.']).next().unwrap_or(""))
        .filter(|f| *f != "cold")
        .collect();
    assigned.sort_unstable();
    assigned.dedup();
    let cold_assigned = body
        .lines()
        .filter(|l| l.trim_start().starts_with("self.cold."))
        .count();
    assert_eq!(
        assigned.len() + usize::from(has_cold),
        state_fields,
        "{tag}: clone_from assigns every field: {assigned:?}"
    );
    assert_eq!(cold_assigned, cold_fields, "{tag}: and every cold field");

    let out = support::compile_and_run(code, MAIN, tag);
    assert!(
        out.stdout.contains("allocs=0\n"),
        "{tag}: clone_from must not allocate:\n{}",
        out.stdout
    );
    assert!(
        out.stdout.contains("same_debug=true"),
        "{tag}: every field copied:\n{}",
        out.stdout
    );
    assert!(
        out.stdout.contains("same_out=true"),
        "{tag}: the copy continues identically:\n{}",
        out.stdout
    );
    assert!(
        out.stdout.contains("clone_same=true"),
        "{tag}: clone() still a full copy:\n{}",
        out.stdout
    );
}

#[test]
fn nodal_state_clone_from_copies_every_field_in_place_without_allocating() {
    let mut cfg = support::config_in_out_or_node1(TRANSFORMER_BJT, 48000.0);
    cfg.circuit_name = "clone_nodal".to_string();
    let (code, _, _) = support::build_as_shipped(TRANSFORMER_BJT, &cfg, "nodal");
    assert!(
        code.contains("pub cold: Box<CircuitStateCold>,"),
        "the witness has a boxed cold block"
    );
    check(&code, "clone_nodal");
}

#[test]
fn dk_state_clone_from_copies_every_field_in_place_without_allocating() {
    let mut cfg = support::config_in_out_or_node1(DIODE_CLIPPER, 48000.0);
    cfg.circuit_name = "clone_dk".to_string();
    let (code, _, _) = support::build_as_shipped(DIODE_CLIPPER, &cfg, "dk");
    check(&code, "clone_dk");
}

#[test]
fn provenance_records_a_baked_switch_override() {
    use melange_solver::build::BuildOptions;
    let mut cfg = support::config_in_out_or_node1(TRANSFORMER_BJT, 48000.0);
    cfg.circuit_name = "prov_switch".to_string();
    let (plain, _, _) = support::build_as_shipped(TRANSFORMER_BJT, &cfg, "nodal");
    assert!(!plain.contains("switch_overrides"), "no override, no key");
    let built =
        support::try_build_shipped_with(TRANSFORMER_BJT, &cfg, "nodal", |o: &mut BuildOptions| {
            o.switch_overrides = Some(vec!["Load=1".to_string()]);
        })
        .expect("build with an override");
    let prov = built
        .generated
        .code
        .lines()
        .find(|l| l.starts_with("// provenance:"))
        .expect("provenance line");
    assert!(prov.contains("\"switch_overrides\":{\"Load\":1}"), "{prov}");
}
