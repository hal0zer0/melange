//! Regression test for the nodal full-LU BE-fallback alpha-floor bug
//! (2026-08-03, dr-debuggenshmirtz investigation).
//!
//! ## The bug
//!
//! `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs` emits a
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
//! A *library-level* repro is not possible: `.linearize` reduction requires
//! the CLI's `apply_linearize_reductions` DC-OP preflight
//! (`tools/melange-cli/src/main.rs`), not exposed to library tests, and the
//! un-linearized M=16 variant doesn't converge at all in melange (Q9's
//! full-nonlinear Vbe-multiplier topology is why it was linearized). Simpler
//! nodal circuits don't ill-condition the crossover the same way, so they stay
//! bounded with or without the floor (a false guard). The behavioral test below
//! therefore drives the real circuit through the built `melange` binary, where
//! the CLI does the linearize.
//!
//! ## The fix
//!
//! Remove the `.max(0.01)` floor — `alpha *= damp_thresh / max_node_dv`
//! (uncapped division). This keeps the worst-case per-iteration node step
//! at exactly `damp_thresh` regardless of how large the raw delta is,
//! matching the layer's documented intent ("Global node voltage damping").
//! Applied to both the primary-loop damping and the BE-fallback damping in
//! `emit_nodal` (`nodal_emitter.rs`).
//!
//! ## Tests here
//!
//! 1. `..._no_ratio_floor` — a code-string pin on the full-LU nodal path
//!    (forced via the inert behavioral-B-source trick from
//!    `nodal_emitter_regression_tests.rs`): the emitted damping must divide
//!    uncapped and the `.max(0.01)` floor must be absent from both loops. Fast,
//!    runs everywhere.
//! 2. `..._internal_peak_stays_physical` — the behavioral guard, `#[ignore]`d
//!    (needs the built `melange` binary; run with `-- --ignored`). Drives the
//!    embedded wurli-power-amp snapshot at 88.2 kHz across the divergent
//!    amplitudes and asserts the internal peak stays < 200 V. Genuinely fails
//!    pre-fix (~28 kV) and passes post-fix (~32 V).

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
    assert!(
        code.contains("alpha *= 10.0 / max_node_dv;"),
        "BE-fallback node damping should divide uncapped by 10.0 (no ratio floor)"
    );
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

/// Behavioral regression test — the REAL one, with teeth.
///
/// A library-level stand-in does not work: a simple nodal circuit (e.g. a 12 V
/// BJT common-emitter, even hammered far past clipping) does not ill-condition
/// its Newton Jacobian the way the wurli-power-amp class-AB crossover does, so
/// it stays bounded with OR without the floor — a false guard. The blowup needs
/// the exact M=14 topology, which requires the CLI-only `.linearize` DC-OP
/// preflight (`tools/melange-cli/src/main.rs`). So this drives the actual circuit
/// through the built `melange` binary and asserts the internal peak stays
/// physical (`max_abs_v_prev`). It genuinely fails pre-fix (internal peak
/// ~16-28 kV) and passes post-fix (~32 V, at the ±22.5 V rails).
///
/// `#[ignore]` because it needs `melange` on the target path (build it with
/// `cargo build -p melange-cli`); run with `-- --ignored`, same as the SPICE
/// validation suite. Uses an embedded netlist snapshot (not the external
/// melange-circuits copy) so it is self-contained and immune to sync-drift.
#[test]
#[ignore = "requires the melange binary (cargo build -p melange-cli)"]
fn test_wurli_power_amp_internal_peak_stays_physical() {
    // Embedded snapshot of wurli-power-amp.cir (functionally identical across the
    // melange-circuits / openwurli copies as of 2026-08-03). `.linearize Q9` is
    // load-bearing: the un-linearized M=16 form does not converge at all.
    const WPA: &str = include_str!("data/wurli_power_amp_snapshot.cir");

    // Locate the workspace `melange` binary (debug or release).
    let manifest = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let target = manifest.join("../../target");
    let bin = ["debug/melange", "release/melange"]
        .iter()
        .map(|p| target.join(p))
        .find(|p| p.exists())
        .unwrap_or_else(|| {
            panic!("melange binary not found under {target:?} — run `cargo build -p melange-cli`")
        });

    let dir = std::env::temp_dir();
    let cir = dir.join(format!("wpa_regress_{}.cir", std::process::id()));
    let wav = dir.join(format!("wpa_regress_{}.wav", std::process::id()));
    std::fs::write(&cir, WPA).expect("write netlist");

    // 88.2 kHz native rate — the blowup is convergence-path-dependent and only
    // manifests at the amp's design rate (at 48 kHz none of these diverge). Pre-
    // fix, amplitudes 0.05 / 1.0 / 2.0 each diverge to 16-28 kV internal while
    // 0.10-0.50 stay physical; sweeping the divergent set means the guard does
    // not hinge on any single convergence path.
    let mut worst = 0.0f64;
    for amp in ["0.05", "1.0", "2.0"] {
        let out = std::process::Command::new(&bin)
            .args([
                "simulate",
                cir.to_str().unwrap(),
                "-o",
                wav.to_str().unwrap(),
                "-s",
                "88200",
                "-d",
                "0.5",
                "--amplitude",
                amp,
            ])
            .output()
            .expect("run melange simulate");
        let text = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        let peak: f64 = text
            .lines()
            .find(|l| l.contains("max_abs_v_prev:"))
            .and_then(|l| l.split(':').nth(1))
            .and_then(|s| s.trim().parse().ok())
            .unwrap_or_else(|| panic!("no max_abs_v_prev at amp {amp} in output:\n{text}"));
        worst = worst.max(peak);
    }
    let _ = std::fs::remove_file(&cir);
    let _ = std::fs::remove_file(&wav);

    // ±22.5 V rails. Post-fix all amplitudes stay ≈ 22-32 V; pre-fix (floored
    // damping) diverges to 16-28 kV on the divergent amplitudes.
    assert!(
        worst < 200.0,
        "wurli-power-amp internal peak reached {worst:.3e} V — node-step damping \
         regression (the floored ratio lets a huge crossover NR delta through)?"
    );
}
