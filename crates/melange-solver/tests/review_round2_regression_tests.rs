//! Per-fix regression tests for the review-round-2 "batch B" latent
//! twin-path fixes (2026-07-21).
//!
//! Each fix below was applied to close a *latent* twin-path gap (the code path
//! was correct on one emitter/solver twin and wrong or incoherent on the
//! other). Because they are latent, the golden-audio gate stays 42/42
//! bit-identical — so nothing else in the suite fails if any of them silently
//! regresses. These tests are the tripwires: every one is written to FAIL on
//! the pre-fix code and PASS at HEAD-with-fixes.
//!
//! Covered fixes (numbering follows the batch-B list; #4 self-heating dt is
//! deliberately NOT duplicated here — it already has coverage in
//! `nodal_emitter_regression_tests.rs`):
//!
//!  1. K_eff parasitic-BJT absorption applied in the no-pot DK template's
//!     `set_sample_rate` rebuild for BOTH the trap and the BE matrix sets
//!     (`state.rs.tera` + `dk_emitter::k_eff_adjust_stmts`).
//!  2. NaN-honest convergence predicate on the DK NR path
//!     (`nr_helpers.rs`): `!(x <= thr)` so a NaN step reads as NOT converged.
//!  3. Nodal same-rate `set_sample_rate` PRESERVES transient state
//!     (DC-blocker history, OS history) instead of zeroing it.
//!  5. `dc_op.rs` internal-node stamps ACCUMULATE (not ASSIGN) and the
//!     grounded-terminal `Option`-row fix (ground no longer aliases node 0).
//!  6. sqrt/ln derivative guard coherence in `expr.rs`: the paired derivative
//!     is exactly 0 in the clamped-flat region (no ~1e300 cliff).

mod support;

use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::device_types::{BjtParams, DeviceParams, DeviceSlot, DeviceType};
use melange_solver::expr::{EvalCtx, Expr, Var};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::{Element, Netlist};

const SR: f64 = 48000.0;

// ───────────────────────────── shared helpers ─────────────────────────────

/// Slice a generated method body: from `sig` to the next method declaration at
/// method indentation (or end of file). Panics if `sig` is absent.
fn slice_method<'a>(code: &'a str, sig: &str) -> &'a str {
    let start = code
        .find(sig)
        .unwrap_or_else(|| panic!("signature `{sig}` not found in generated code"));
    let body_start = start + sig.len();
    let rest = &code[body_start..];
    let end_rel = rest
        .find("\n    pub fn ")
        .into_iter()
        .chain(rest.find("\n    fn "))
        .min()
        .unwrap_or(rest.len());
    &code[start..body_start + end_rel]
}

// ═══════════════════════════════════════════════════════════════════════════
// Fix #1 — K_eff parasitic-BJT absorption in the no-pot DK set_sample_rate body
// ═══════════════════════════════════════════════════════════════════════════

/// A pot-less DK circuit with a BJT carrying parasitic RB/RC/RE. The
/// parasitics keep the device 2-D (excluded from FA reduction) and route it
/// through the K_eff absorption path (`has_parasitics && !has_internal_mna_nodes
/// && dimension == 2`). The baked `K_DEFAULT`/`K_BE_DEFAULT` bake the absorption
/// in; the no-pot `set_sample_rate` rebuild body must apply the same absorption
/// or the parasitics are electrically deleted at any host rate != 48 kHz.
const DK_PARASITIC_BJT: &str = "Parasitic BJT DK amp
VCC vcc 0 DC 12
Rb vcc base 220k
Rc vcc coll 4.7k
Q1 coll base emit QNPN
Re emit 0 470
Cin in base 1u
Cout coll out 1u
Rl out 0 100k
.MODEL QNPN NPN(IS=1e-14 BF=200 BR=3 RB=10 RC=5 RE=2)
";

#[test]
fn fix1_no_pot_dk_set_sample_rate_applies_k_eff_absorption() {
    let config = support::config_for_spice(DK_PARASITIC_BJT, SR);
    let (code, _n, m) = support::generate_circuit_code(DK_PARASITIC_BJT, &config);

    // Sanity: this must be the DK no-pot template path (the fix target).
    // Nodal codegen emits `g_aug`; a pot circuit emits `fn rebuild_matrices`.
    assert!(
        !code.contains("g_aug"),
        "fix1: circuit must route to the DK path, not nodal"
    );
    assert!(
        !code.contains("fn rebuild_matrices"),
        "fix1: circuit must use the no-pot set_sample_rate template (no pots) \
         — that is the twin the fix targets"
    );
    assert!(m >= 2, "fix1: expected a 2-D BJT device slot, got M={m}");

    let ssr = slice_method(&code, "fn set_sample_rate");

    // Trap kernel K_eff absorption inside the rate-change rebuild.
    assert!(
        ssr.contains("absorb parasitic-BJT R drops"),
        "fix1: no-pot set_sample_rate must carry the K_eff absorption comment.\n\
         set_sample_rate body:\n{ssr}"
    );
    assert!(
        ssr.contains("k[0][0] -="),
        "fix1: no-pot set_sample_rate must apply the trap K_eff adjustment \
         (`k[s][s] -= RE` ...), not leave K raw.\nset_sample_rate body:\n{ssr}"
    );
    // BE kernel K_eff absorption — the fix covers BOTH matrix sets.
    assert!(
        ssr.contains("K_eff for the BE kernel"),
        "fix1: no-pot set_sample_rate must apply K_eff to the BE kernel too.\n\
         set_sample_rate body:\n{ssr}"
    );
    assert!(
        ssr.contains("k_be[0][0] -="),
        "fix1: no-pot set_sample_rate must apply the BE K_eff adjustment \
         (`k_be[s][s] -= RE` ...).\nset_sample_rate body:\n{ssr}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Fix #2 — NaN-honest DK NR convergence predicate
// ═══════════════════════════════════════════════════════════════════════════

/// A DK nonlinear circuit exercising `emit_nr_limit_and_converge`. The BJT amp
/// reuses the fix-#1 fixture (any DK device NR body emits the same predicate).
#[test]
fn fix2_dk_nr_convergence_predicate_is_nan_honest() {
    let config = support::config_for_spice(DK_PARASITIC_BJT, SR);
    let (code, _n, _m) = support::generate_circuit_code(DK_PARASITIC_BJT, &config);

    // The current-residual check must be NaN-honest: `!(f_i.abs() <= i_thr)`,
    // so a NaN residual reads as NOT converged (falls into BE fallback rather
    // than exiting NR "converged" and tripping the sample-level NaN reset).
    assert!(
        code.contains("if !(f0.abs() <= i_thr)"),
        "fix2: DK current-residual convergence check must use the NaN-honest \
         `!(f0.abs() <= i_thr)` form"
    );
    // The voltage-step check likewise.
    assert!(
        code.contains("if !(step.abs() <= v_thr)"),
        "fix2: DK voltage-step convergence check must use the NaN-honest \
         `!(step.abs() <= v_thr)` form"
    );

    // And the bare `>` form (pre-fix) must be gone from the NR predicates: a
    // NaN step there compares false and is silently treated as converged.
    assert!(
        !code.contains("if f0.abs() > i_thr"),
        "fix2: pre-fix bare `if f0.abs() > i_thr` predicate must not survive"
    );
    assert!(
        !code.contains("if step.abs() > v_thr"),
        "fix2: pre-fix bare `if step.abs() > v_thr` predicate must not survive"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Fix #3 — nodal same-rate set_sample_rate preserves transient state
// ═══════════════════════════════════════════════════════════════════════════

/// A diode clipper forced onto the nodal path, with a DC-blocker on the output
/// so the blocker history is load-bearing transient state.
const NODAL_DIODE_CLIPPER: &str = "Diode clipper nodal
Rin in n1 1k
D1 n1 out DMOD
D2 out n1 DMOD
Rout out 0 47k
Cout out 0 10n
.MODEL DMOD D(IS=2.52e-9 N=1.9)
";

#[test]
fn fix3_nodal_same_rate_preserves_transient_state_emission() {
    let config = support::config_for_spice(NODAL_DIODE_CLIPPER, SR);
    let (code, _n, _m) = support::generate_circuit_code_nodal(NODAL_DIODE_CLIPPER, &config);

    let ssr = slice_method(&code, "fn set_sample_rate");

    // Isolate the same-rate arm: from the same-rate guard to its `return;`.
    let guard = "if (sample_rate - SAMPLE_RATE).abs()";
    let g_start = ssr
        .find(guard)
        .expect("fix3: same-rate guard not found in nodal set_sample_rate");
    let arm_rest = &ssr[g_start..];
    let ret_rel = arm_rest
        .find("return;")
        .expect("fix3: same-rate arm has no return");
    let same_rate_arm = &arm_rest[..ret_rel];

    // POLICY marker present: the same-rate call is a no-op reconfiguration.
    assert!(
        same_rate_arm.contains("POLICY") && same_rate_arm.contains("must not click"),
        "fix3: nodal same-rate arm must carry the transient-preservation POLICY.\n\
         same-rate arm:\n{same_rate_arm}"
    );
    // The same-rate arm must NOT reseed/zero DC-blocker history (that is the
    // pre-fix behavior — `emit_dc_block_history_reseed` set x_prev<-DC_OP and
    // y_prev<-0, injecting an LF transient on every same-rate call).
    assert!(
        !same_rate_arm.contains("dc_block_y_prev = [0.0"),
        "fix3: nodal same-rate arm must NOT zero dc_block_y_prev.\n\
         same-rate arm:\n{same_rate_arm}"
    );
    assert!(
        !same_rate_arm.contains("dc_block_x_prev"),
        "fix3: nodal same-rate arm must NOT reseed dc_block_x_prev.\n\
         same-rate arm:\n{same_rate_arm}"
    );
    // The genuine rate-change path below still zeroes the blocker history —
    // confirm the whole method still has the reseed (the fix only moved it off
    // the same-rate arm, it did not delete it).
    assert!(
        ssr.contains("dc_block_y_prev = [0.0"),
        "fix3: the genuine rate-change arm must still reseed the blocker"
    );
}

#[test]
fn fix3_nodal_same_rate_call_is_click_free() {
    let config = support::config_for_spice(NODAL_DIODE_CLIPPER, SR);
    let (code, _n, _m) = support::generate_circuit_code_nodal(NODAL_DIODE_CLIPPER, &config);

    // Behavioral: a same-rate set_sample_rate mid-stream must be transparent.
    // `ctrl` never makes the call; `test` calls it at sample 2205 while a
    // healthy AC swing keeps the DC-blocker history nonzero. Pre-fix the call
    // zeroed y_prev / reseeded x_prev -> an LF transient -> nonzero max diff.
    let main_code = r#"
fn main() {
    let sr = 48000.0_f64;
    let mut test = CircuitState::default();
    let mut ctrl = CircuitState::default();
    let mut max_diff = 0.0_f64;
    for i in 0..4800 {
        let x = (2.0 * std::f64::consts::PI * 300.0 * i as f64 / sr).sin() * 0.6;
        if i == 2205 {
            test.set_sample_rate(sr); // same-rate no-op
        }
        let yt = process_sample(x, &mut test)[0];
        let yc = process_sample(x, &mut ctrl)[0];
        max_diff = max_diff.max((yt - yc).abs());
    }
    println!("max_diff={:.6e}", max_diff);
}
"#;
    let out = support::compile_and_run(&code, main_code, "fix3_nodal_same_rate");
    let max_diff = out.parse_kv("max_diff").expect("missing max_diff");
    assert!(
        max_diff < 1e-12,
        "fix3: a same-rate nodal set_sample_rate must not perturb transient \
         state (got max diff {max_diff:.3e})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Fix #5 — dc_op internal-node stamps: accumulate + grounded Option-row
// ═══════════════════════════════════════════════════════════════════════════

/// Build DC-OP device slots that carry the model's parasitic RB/RC/RE (the
/// stock `nonlinear_dc_op_tests::build_device_slots` hardcodes them to 0, which
/// would never enter the internal-node path). Slot order mirrors the MNA
/// nonlinear-device order (element order), as `solve_dc_operating_point`
/// requires.
fn bjt_slots_with_parasitics(netlist: &Netlist) -> Vec<DeviceSlot> {
    let vt = 0.02585;
    let mut slots = Vec::new();
    let mut dim_offset = 0;
    for elem in &netlist.elements {
        if let Element::Bjt { model, .. } = elem {
            let p = |name: &str, dflt: f64| model_param(netlist, model, name).unwrap_or(dflt);
            let is_pnp = netlist
                .models
                .iter()
                .find(|m| m.name.eq_ignore_ascii_case(model))
                .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
                .unwrap_or(false);
            slots.push(DeviceSlot {
                device_type: DeviceType::Bjt,
                start_idx: dim_offset,
                dimension: 2,
                params: DeviceParams::Bjt(BjtParams {
                    is: p("IS", 1e-14),
                    vt: p("VT", vt),
                    beta_f: p("BF", 200.0),
                    beta_r: p("BR", 3.0),
                    is_pnp,
                    vaf: f64::INFINITY,
                    var: f64::INFINITY,
                    ikf: f64::INFINITY,
                    ikr: f64::INFINITY,
                    cje: 0.0,
                    cjc: 0.0,
                    tf: 0.0,
                    vje: 0.75,
                    mje: 0.33,
                    vjc: 0.75,
                    mjc: 0.33,
                    fc: 0.5,
                    nf: 1.0,
                    nr: 1.0,
                    ise: p("ISE", 0.0),
                    ne: p("NE", 1.5),
                    isc: p("ISC", 0.0),
                    nc: p("NC", 2.0),
                    rb: p("RB", 0.0),
                    rc: p("RC", 0.0),
                    re: p("RE", 0.0),
                    rth: f64::INFINITY,
                    cth: 1e-3,
                    xti: 3.0,
                    eg: 1.11,
                    tamb: 300.15,
                }),
                has_internal_mna_nodes: false,
                vg2k_frozen: 0.0,
                stateful: None,
            });
            dim_offset += 2;
        }
    }
    slots
}

fn model_param(netlist: &Netlist, model_name: &str, param_name: &str) -> Option<f64> {
    netlist
        .models
        .iter()
        .find(|m| m.name.eq_ignore_ascii_case(model_name))
        .and_then(|m| {
            m.params
                .iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(param_name))
                .map(|(_, v)| *v)
        })
}

/// Diode-connected BJT (base tied to collector) with RE>0 to force internal
/// -node expansion but RB=RC=0 so `int_base == int_collector` (both alias the
/// external `bc` matrix row). The Vbc N_v extraction row then has two stamps at
/// the SAME index. ACCUMULATE cancels them to 0 (base==collector => Vbc=0);
/// the pre-fix ASSIGN overwrote to a single `-1`, sensing a phantom
/// `Vbc = -V(bc)`.
const DIODE_CONNECTED_BJT: &str = "Diode connected BJT DC OP
VCC vcc 0 DC 5
R1 vcc bc 2.2k
Q1 bc bc 0 QMOD
.MODEL QMOD NPN(IS=1e-14 BF=200 BR=3 RE=1)
";

#[test]
fn fix5_accumulate_stamps_diode_connected_bjt_has_zero_vbc() {
    let netlist = Netlist::parse(DIODE_CONNECTED_BJT).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA");
    let slots = bjt_slots_with_parasitics(&netlist);
    let cfg = DcOpConfig::default();

    let r = solve_dc_operating_point(&mna, &slots, &cfg);
    assert!(r.converged, "fix5(accumulate): DC OP must converge");

    // v_nl = N_v * v_node. For the single BJT, v_nl[1] is the Vbc controlling
    // voltage. Base and collector are the SAME node, so Vbc is 0 by topology.
    let vbc = r.v_nl[1];
    assert!(
        vbc.abs() < 1e-6,
        "fix5(accumulate): diode-connected BJT must have Vbc == 0 \
         (base tied to collector); phantom Vbc = {vbc:.4} V means the internal \
         -node N_v stamp ASSIGNED instead of ACCUMULATED"
    );
    // And the internal-junction voltage this circuit sits at is a real forward
    // drop (~0.7 V), proving the device is actually conducting (not a
    // degenerate all-zero "solution").
    let vbe = r.v_nl[0];
    assert!(
        (0.4..=0.9).contains(&vbe),
        "fix5(accumulate): expected a forward Vbe ~0.7 V, got {vbe:.4} V"
    );
}

/// Grounded-emitter parasitic BJT: RB>0 (internal base row), RC=0, RE=0 and the
/// emitter tied to ground. The emitter terminal has R=0 on a grounded node, so
/// the fixed code maps `int_emitter = None` (ground has no matrix row). The
/// pre-fix `else { 0 }` fallback aliased it onto circuit node index 0 —
/// injecting the device's emitter current into whatever node happened to be
/// first, and extracting Vbe against V(node0) instead of ground.
const GROUNDED_EMITTER_BJT: &str = "Grounded emitter parasitic BJT DC OP
VCC vcc 0 DC 12
R1 vcc base 2.2meg
Q1 coll base 0 QMOD
R2 vcc coll 5.6k
.MODEL QMOD NPN(IS=1e-14 BF=200 BR=3 RB=10)
";

#[test]
fn fix5_grounded_terminal_not_aliased_onto_node0() {
    let netlist = Netlist::parse(GROUNDED_EMITTER_BJT).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA");
    let slots = bjt_slots_with_parasitics(&netlist);
    let cfg = DcOpConfig::default();

    let r = solve_dc_operating_point(&mna, &slots, &cfg);
    assert!(r.converged, "fix5(grounded): DC OP must converge");

    // Emitter is ground (0 V), so Vbe = V(base') - 0 must be a sane forward
    // junction voltage. Pre-fix the emitter aliased onto node 0, so Vbe was
    // extracted as V(base') - V(node0): the BJT saw a wildly wrong (cut-off or
    // huge-reverse) junction and the "solution" was garbage.
    let vbe = r.v_nl[0];
    assert!(
        (0.4..=0.8).contains(&vbe),
        "fix5(grounded): grounded-emitter Vbe must be a forward drop ~0.6 V, \
         got {vbe:.4} V — a grounded terminal with R=0 was stamped into node 0"
    );
    // Collector must sit strictly between ground and the 12 V rail (device in
    // forward-active, dropping voltage across R2). Node-0 current injection
    // pushed this off in the pre-fix code.
    let coll_idx = mna
        .node_map
        .get("coll")
        .copied()
        .expect("coll node")
        .saturating_sub(1);
    let v_coll = r.v_node[coll_idx];
    assert!(
        v_coll > 0.2 && v_coll < 11.5,
        "fix5(grounded): collector must be in forward-active bias (0.2..11.5 V), \
         got {v_coll:.3} V"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Fix #6 — sqrt/ln derivative guard coherence (flat region is exactly 0)
// ═══════════════════════════════════════════════════════════════════════════

/// Minimal single-node evaluation context: `V(node)` resolves to `val`,
/// everything else to 0.
struct OneNode {
    node: String,
    val: f64,
}
impl EvalCtx for OneNode {
    fn node_v(&self, name: &str) -> f64 {
        if name == self.node {
            self.val
        } else {
            0.0
        }
    }
    fn branch_i(&self, _: &str) -> f64 {
        0.0
    }
    fn param(&self, _: &str) -> f64 {
        0.0
    }
    fn time(&self) -> f64 {
        0.0
    }
    fn inv_dt(&self) -> f64 {
        0.0
    }
    fn half_dt(&self) -> f64 {
        0.0
    }
    fn x_prev(&self, _: usize) -> f64 {
        0.0
    }
    fn integ_prev(&self, _: usize) -> f64 {
        0.0
    }
}

/// Value of d/dV(a) `src` evaluated at V(a) = x (mirrors the in-crate
/// `analytic_diff`: parse -> diff -> simplify -> eval).
fn deriv_at(src: &str, x: f64) -> f64 {
    let e = Expr::parse(src).expect("parse expr");
    e.diff(&Var::Node("a".to_string()))
        .simplify()
        .eval(&OneNode {
            node: "a".to_string(),
            val: x,
        })
}

#[test]
fn fix6_sqrt_derivative_is_zero_in_flat_region_and_finite_everywhere() {
    // Clamped-flat region x <= 0: value is max(x,0).sqrt() == 0, so the paired
    // derivative must be exactly 0 (pre-fix: 0.5/(sqrt(0)+1e-150) ~= 5e149).
    for x in [-1.0, -1e-6, 0.0] {
        let d = deriv_at("sqrt(V(a))", x);
        assert_eq!(
            d, 0.0,
            "fix6: sqrt' at x={x} must be exactly 0 in the clamped-flat region \
             (pre-fix emitted a ~5e149 cliff)"
        );
    }
    // Just inside the domain the true derivative 0.5/sqrt(x) is emitted and is
    // finite. At x=0.25 -> 0.5/0.5 = 1.0.
    let d = deriv_at("sqrt(V(a))", 0.25);
    assert!(
        (d - 1.0).abs() < 1e-12,
        "fix6: sqrt' at 0.25 must be 1.0, got {d}"
    );
    // No huge-cliff anywhere near / below the boundary.
    for x in [-10.0, -1.0, 0.0, 1e-12, 1e-6] {
        let d = deriv_at("sqrt(V(a))", x);
        assert!(
            d.is_finite() && d.abs() < 1e12,
            "fix6: sqrt' at x={x} must be finite and free of a ~1e300 cliff, got {d}"
        );
    }
}

#[test]
fn fix6_ln_derivative_is_zero_in_flat_region_and_finite_everywhere() {
    // Clamped-flat region x <= 1e-300: value is ln(max(x,1e-300)) (pinned), so
    // the paired derivative must be exactly 0 (pre-fix: 1/max(x,1e-300) = 1e300).
    for x in [-1.0, -1e-6, 0.0] {
        let d = deriv_at("ln(V(a))", x);
        assert_eq!(
            d, 0.0,
            "fix6: ln' at x={x} must be exactly 0 in the clamped-flat region \
             (pre-fix emitted a ~1e300 cliff)"
        );
    }
    // Inside the domain the true 1/x is emitted. At x=0.5 -> 2.0.
    let d = deriv_at("ln(V(a))", 0.5);
    assert!(
        (d - 2.0).abs() < 1e-12,
        "fix6: ln' at 0.5 must be 2.0, got {d}"
    );
    // No ~1e300 cliff anywhere in / below the flat region.
    for x in [-10.0, -1.0, 0.0] {
        let d = deriv_at("ln(V(a))", x);
        assert!(
            d.is_finite() && d.abs() < 1e12,
            "fix6: ln' at x={x} must be finite and free of the ~1e300 cliff, got {d}"
        );
    }
}
