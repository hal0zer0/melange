//! Regression test for the nodal full-LU BE-fallback alpha-floor bug
//! (2026-08-03).
//!
//! ## The bug
//!
//! The nodal emitter (`codegen/rust_emitter/nodal_emitter/full_lu_newton.rs`,
//! `emit_nodal_newton`) emits a
//! "global node voltage damping" layer (both in the primary trap/BE-primary
//! NR loop and in the Backward Euler fallback loop) that caps the worst-case
//! per-iteration node voltage step at a threshold (`damp_thresh`, or a fixed
//! 10.0 V in the BE fallback):
//!
//! ```text
//! if max_node_dv > damp_thresh {
//!     alpha *= (damp_thresh / max_node_dv).max(0.01);   // BUGGY
//! }
//! ```
//!
//! The `.max(0.01)` floor bounds how much the damping ratio can shrink BY,
//! not what the resulting damped step actually IS. When a single NR
//! iteration's companion-model LU solve produces a raw voltage delta many
//! orders of magnitude beyond `damp_thresh` (observed: 3.8e7 V on
//! wurli-power-amp at a class-AB crossover device-state transition), the 1%
//! floor still lets a multiple of `damp_thresh` through (0.01 * 3.8e7 =
//! 380,000 V — nowhere near the intended <=10 V cap). The resulting
//! ~3.8 kV single-iteration jump launches the trajectory into a deeply
//! nonphysical operating point that the remaining NR iterations, still
//! locally damped, cannot recover from within the iteration budget.
//!
//! The BE-fallback path's voltage-step convergence criterion
//! (`be_step_exceeded`) then declares false convergence: its relative
//! tolerance (`1e-3 * v[node].abs()`) scales with the ALREADY-DIVERGED node
//! voltage, so once a node sits at ~-16,000 V, an oscillating ~10-160 V
//! per-iteration step trivially satisfies the tolerance. The wildly
//! nonphysical state gets committed to `state.v_prev`/`state.i_nl_prev`,
//! corrupting every subsequent sample.
//!
//! ## Real-circuit confirmation (CLI, not this test)
//!
//! Reproduced and fixed on `melange-circuits/unstable/amp/wurli-power-amp.cir`
//! (N=20, M=14 after `.linearize Q9`, auto-routed to nodal full-LU NR with
//! Backward-Euler-primary integration) via `melange compile` /
//! `melange simulate`. A 1 kHz sine at 88.2 kHz drove an internal node
//! (`emit_pair`, the differential pair's shared emitter) as high as
//! -16,079 V to -27,977 V depending on drive amplitude — thousands of volts
//! outside any physically sane range for a +-22.5 V-rail amplifier. After
//! removing the `.max(0.01)` floor, the same sweep (amplitudes 0.05-2.00 V)
//! stayed within 20-32 V (matching the +-22.5 V rails) at every amplitude,
//! and `diag_nr_max_iter_count`/`diag_be_fallback_count` both dropped by
//! 10-70x (bad state no longer cascades into subsequent samples' NR).
//!
//! The blowup needs the `.linearize`d topology: the `.linearize` reduction is
//! the DC-OP preflight `pipeline::apply_linearize_reductions`
//! (`crates/melange-solver/src/pipeline.rs`, run by `build::build`), and the
//! un-linearized M=16 variant doesn't converge at all in melange (Q9's
//! full-nonlinear Vbe-multiplier topology is why it was linearized). Simpler
//! nodal circuits don't ill-condition the crossover the same way, so they stay
//! bounded with or without the floor (a false guard). The behavioral test below
//! builds the real circuit through `build::build`.
//!
//! ## The fix
//!
//! Remove the `.max(0.01)` floor — `alpha *= damp_thresh / max_node_dv`
//! (uncapped division). This keeps the worst-case per-iteration node step
//! at exactly `damp_thresh` regardless of how large the raw delta is,
//! matching the layer's documented intent ("Global node voltage damping").
//! Applied to both the primary-loop damping and the BE-fallback damping.
//!
//! ## Tests here
//!
//! 1. `..._no_ratio_floor` — a code-string pin on the full-LU nodal path
//!    (forced via the inert behavioral-B-source trick from
//!    `nodal_emitter_regression_tests.rs`): the emitted damping must divide
//!    uncapped and the `.max(0.01)` floor must be absent from both loops. Fast,
//!    runs everywhere.
//! 2. `..._internal_peak_stays_physical` — the behavioral guard. Builds the
//!    embedded wurli-power-amp snapshot as `melange simulate` does, drives it
//!    at 88.2 kHz across the once-divergent amplitudes and asserts the
//!    internal peak stays < 200 V (it peaks at ~32 V). It no longer fails with
//!    the floor restored (see its doc), so test 1 is what guards the fix.

mod support;

const SR: f64 = 48000.0;

/// Small BJT common-emitter stage. Only needs to exercise the nodal
/// full-LU path's device-evaluation + damping code — the specific circuit
/// doesn't matter for a code-string pin, unlike the real wurli-power-amp
/// blowup (which needs the exact M=14 linearized topology, see module docs).
const BJT_CE_SPICE: &str = "\
BJT Common Emitter
Cin in base 1u
R1 vcc base 47k
R2 base 0 10k
Q1 coll base emit Q2N3904
Rc vcc coll 2.2k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 1u
Rload out 0 100k
Vcc vcc 0 DC 12
.model Q2N3904 NPN(IS=6.734e-15 BF=416.4 VAF=74.03 NF=1)
";

/// Code-string pin: the `.max(0.01)` ratio floor must not reappear in
/// either the primary-loop or the BE-fallback global node damping.
#[test]
fn test_nodal_full_lu_node_damping_has_no_ratio_floor() {
    let config = melange_solver::codegen::CodegenConfig {
        circuit_name: "nodal_full_lu_damping_test".to_string(),
        sample_rate: SR,
        backward_euler: true,
        // Force full-LU explicitly (was a behavioral-dummy routing lever).
        nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride::FullLu,
        ..support::config_for_spice(BJT_CE_SPICE, SR)
    };
    let (code, n, m) = support::generate_circuit_code_nodal(BJT_CE_SPICE, &config);
    assert!(
        n > 0 && m > 0,
        "expected a nontrivial nodal circuit (n={n}, m={m})"
    );

    assert!(
        code.contains("alpha *= damp_thresh / max_node_dv;"),
        "primary-loop node damping should divide uncapped by damp_thresh (no ratio floor)"
    );
    // The separate BE-fallback ladder (fixed 10 V damping) is gone: the
    // backward-Euler solve is the primary routine above, so its damping is the
    // `damp_thresh` line just checked.
    assert!(
        !code.contains("(damp_thresh / max_node_dv).max(0.01)"),
        "primary-loop node damping must not reintroduce the 1% ratio floor \
         (lets a multiple of damp_thresh through when the raw NR step is huge)"
    );
    assert!(
        !code.contains("(10.0 / max_node_dv).max(0.01)"),
        "BE-fallback node damping must not reintroduce the 1% ratio floor \
         (lets a multiple of 10.0 V through when the raw NR step is huge)"
    );
}

/// Behavioral guard on the real circuit.
///
/// A library-level stand-in circuit does not work: a simple nodal circuit (e.g.
/// a 12 V BJT common-emitter, even hammered far past clipping) does not
/// ill-condition its Newton Jacobian the way the wurli-power-amp class-AB
/// crossover does, so it stays bounded with OR without the floor — a false
/// guard. The blowup needed the exact M=14 topology, which the `.linearize`
/// DC-OP preflight produces (`pipeline::apply_linearize_reductions`, run by
/// `build::build`). This builds the actual circuit as `melange simulate` does
/// (`build::build` with simulate's options), drives it with simulate's 1 kHz
/// test tone and asserts the internal peak stays physical: the largest
/// `|v_prev|` over every sample, the measurand behind simulate's
/// `max_abs_v_prev`. On the 2026-08-03 solver it failed with the floor
/// (internal peak ~16-28 kV) and passed without it (~32 V, at the ±22.5 V
/// rails).
///
/// It no longer discriminates the floor: on the current solver, patching
/// `.max(0.01)` back into this build's emitted damping leaves every peak
/// bit-identical (measured 2026-10-01, amplitudes 0.05-2.0). The code-string
/// pin above is what guards the fix; this test guards the physical bound.
///
/// Uses an embedded netlist snapshot (not the external melange-circuits copy)
/// so it is self-contained and immune to sync-drift.
#[test]
fn test_wurli_power_amp_internal_peak_stays_physical() {
    // Embedded snapshot of melange-circuits testing/amp/wurli-power-amp.cir as
    // of 2026-10-01. The 2026-08-03 snapshot (R-28 tied to the +22.5 V rail, no
    // C-13/C-14/Zobel) is refused by today's build: its DC operating point does
    // not converge. `.linearize Q9` is load-bearing: the un-linearized M=16
    // form does not converge at all.
    const WPA: &str = include_str!("data/wurli_power_amp_snapshot.cir");
    // 88.2 kHz native rate — the blowup is convergence-path-dependent and only
    // manifests at the amp's design rate (at 48 kHz none of these diverge).
    const WPA_SR: f64 = 88200.0;

    // `melange simulate`'s build: auto route, DC kept, ±10 V output clamp,
    // the auto-tuned Newton budget, no forward-active BJT reduction.
    let config = support::config_for_spice(WPA, WPA_SR);
    let built = support::try_build_shipped_with(WPA, &config, "auto", |o| {
        o.tolerance = 1e-9;
        o.output_scale = 1.0;
        o.output_clamp = 10.0;
        o.dc_block = false;
        o.max_iter = None;
        o.bjt_fa_mode = melange_solver::codegen::BjtFaMode::Off;
        o.pot_overrides = Some(Vec::new());
        o.resolve_taps = false;
    })
    .unwrap_or_else(|e| panic!("wurli-power-amp build failed: {e}"));
    assert!(
        built.linearize_outcome.bjts_linearized > 0,
        "`.linearize Q9` must apply: the blowup needs the linearized M=14 topology"
    );
    assert_eq!(built.solver_label, "nodal");

    // With the floor (2026-08-03), amplitudes 0.05 / 1.0 / 2.0 each diverged to
    // 16-28 kV internal while 0.10-0.50 stayed physical; sweeping that set
    // means the guard does not hinge on any single convergence path. 0.5 s
    // each, a fresh state per amplitude.
    let main = format!(
        "fn main() {{
    let sr: f64 = {WPA_SR:.1};
    let n = (sr * 0.5) as usize;
    for amp in [0.05f64, 1.0, 2.0] {{
        let mut state = CircuitState::default();
        state.set_sample_rate(sr);
        let mut peak = 0.0f64;
        for i in 0..n {{
            let x = amp * (2.0 * std::f64::consts::PI * 1000.0 * (i as f64) / sr).sin();
            let _ = process_sample(x, &mut state);
            for &v in &state.v_prev {{
                peak = peak.max(v.abs());
            }}
        }}
        println!(\"{{amp}} {{peak:e}}\");
    }}
}}"
    );
    let out = support::compile_and_run(&built.generated.code, &main, "wpa_peak").stdout;
    let peaks: Vec<(f64, f64)> = out
        .lines()
        .map(|l| {
            let v: Vec<f64> = l.split_whitespace().map(|t| t.parse().unwrap()).collect();
            (v[0], v[1])
        })
        .collect();
    assert_eq!(peaks.len(), 3, "one peak per amplitude, got:\n{out}");
    let worst = peaks.iter().map(|p| p.1).fold(0.0f64, f64::max);

    // ±22.5 V rails: every amplitude peaks at 22-32 V. A non-finite peak fails
    // too.
    assert!(
        worst.is_finite() && worst < 200.0 && peaks.iter().all(|p| p.1.is_finite()),
        "wurli-power-amp internal peak reached {worst:.3e} V ({peaks:?}) — node-step \
         damping regression (the floored ratio lets a huge crossover NR delta through)?"
    );
}
