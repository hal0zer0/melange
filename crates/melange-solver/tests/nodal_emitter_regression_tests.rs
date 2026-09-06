//! Regression tests for nodal-emitter fixes (2026-07-18).
//!
//! Covers:
//! 1. FET Jacobian column swap in the full-LU device evaluation body
//!    (`emit_nodal_device_evaluation_body` mapped `jac[0..3]` in helper order
//!    dVgs-first, but NR dims are s=Vds, s+1=Vgs — transposed Jacobian).
//! 2. Single-sided op-amp rail clamps emitting the invalid Rust token `-inf`
//!    when only one of VCC/VEE is finite.
//! 3. PMOS body-effect sign: the GAMMA correction must carry the channel sign
//!    so reverse body bias always increases |VT| (VT is signed; PMOS VT < 0).
//! 4. Self-heating thermal step: exact exponential update (unconditionally
//!    stable), internal (oversampled) rate for dt, and the SPICE3f5 diode
//!    IS(T) 1/N exponent division.
//!
//! The extreme-IS no-RS diode fix (shared `diode_current` helpers replacing
//! the legacy inline 40·n_vt clamp) is covered in
//! `numerical_edge_case_tests.rs::test_wide_bandgap_clipper_no_rs_nodal_full_lu`.

mod support;

const SR: f64 = 48000.0;

fn jfet_cs_spice() -> String {
    "\
JFET Common Source
Cin in gate 10u
Rg gate 0 1Meg
J1 drain gate source J2N5457
Rd vdd drain 2.2k
Rs source 0 1k
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 12
.model J2N5457 NJ(VTO=-2.0 IDSS=5e-3 LAMBDA=0.001)
"
    .to_string()
}

fn mosfet_cs_spice() -> String {
    "\
MOSFET Common Source
Cin in gate 10u
R1 vdd gate 47k
R2 gate 0 100k
M1 drain gate source 0 NMOD
Rd vdd drain 1k
Rs source 0 100
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 5
.model NMOD NM(VTO=2.0 KP=0.1 LAMBDA=0.01)
"
    .to_string()
}

/// Compare a FET circuit on the DK path against the same circuit forced onto
/// the nodal full-LU path (plus the inert forcing branch). The converged NR
/// fixed point is identical on both paths — with the transposed Jacobian the
/// full-LU NR walked wrong step directions, diverging or exhausting MAX_ITER
/// under drive.
fn assert_full_lu_fet_matches_dk(spice: &str, tag: &str, amp: f64) {
    let mut cb = support::config_for_spice(spice, SR);
    let mut cf = support::config_for_spice(spice, SR);
    // Pin BOTH paths to backward Euler. The forced nodal build sits at
    // spectral radius ≈ 1.0 and auto-promotes to BE while the DK build stays
    // trapezoidal — a legitimate integration-scheme difference that would
    // mask the Jacobian comparison this test is about.
    cb.backward_euler = true;
    cf.backward_euler = true;
    // Force the nodal path to full-LU explicitly (was an inert behavioral-dummy
    // routing lever appended to the SPICE). Both paths now run the IDENTICAL
    // circuit — the only difference is DK-Schur vs nodal-full-LU.
    cf.nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::FullLu;
    let dk = support::build_circuit(spice, &cb, &format!("{tag}_dk"));
    let lu = support::build_circuit_nodal(spice, &cf, &format!("{tag}_lu"));
    assert!(
        lu.code.contains("g_aug"),
        "{tag}: forcing branch must route the nodal build to full-LU NR"
    );

    let n = 1920; // 40 ms
    let dk_out = support::run_sine(&dk, 500.0, amp, n, SR);
    let lu_full = support::run_sine_full(&lu, 500.0, amp, n, SR);
    let lu_out = lu_full.parse_samples();

    support::assert_finite(&dk_out);
    support::assert_finite(&lu_out);
    let dk_peak: f64 = dk_out.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    assert!(dk_peak > 1e-4, "{tag}: DK peak too low ({dk_peak:.3e})");

    support::assert_samples_match(&dk_out, &lu_out, 5e-3, tag);

    // With the correct Jacobian the full-LU NR converges every sample even
    // under drive; the transposed Jacobian exhausted MAX_ITER.
    let max_iter_hits = lu_full.diag("nr_max_iter_count").unwrap_or(f64::NAN);
    assert_eq!(
        max_iter_hits, 0.0,
        "{tag}: full-LU NR should never exhaust MAX_ITER (got {max_iter_hits} samples)"
    );
}

#[test]
fn test_full_lu_jfet_jacobian_matches_dk() {
    assert_full_lu_fet_matches_dk(&jfet_cs_spice(), "jfet_full_lu_jac", 0.5);
}

#[test]
fn test_full_lu_mosfet_jacobian_matches_dk() {
    assert_full_lu_fet_matches_dk(&mosfet_cs_spice(), "mosfet_full_lu_jac", 0.5);
}

// ============================================================================
// Single-sided op-amp rail clamp (VCC only, no VEE/VSAT)
// ============================================================================

/// Inverting amp (gain -4.7) with only the top rail specified. Before the
/// fix, every rail-clamp emission site formatted `f64::NEG_INFINITY` with
/// `{:.17e}` producing the invalid token `-inf` — the generated code did not
/// compile. After the fix only finite bounds are emitted (`.min(hi)` here),
/// so the top rail clamps and the bottom swing is unbounded.
/// The `Rdx`/`Ddx` branch is a barely-loading (1 MΩ) diode hung off the
/// output: the nodal Schur emitter only emits the rail-clamp block inside its
/// M > 0 branch, so a purely linear circuit would skip clamping entirely and
/// the test would not exercise the clamp formatting at all.
const SINGLE_RAIL_OPAMP_SPICE: &str = "\
Single-rail op-amp
Rterm in 0 1Meg
Rg in oa_neg 10k
Rfb oa_neg out 47k
U1 0 oa_neg out OA1
Rload out 0 100k
Rdx out dx 1Meg
Ddx dx 0 DDX
.model DDX D(IS=1e-15)
.model OA1 OA(AOL=200000 ROUT=100 VCC=9)
";

#[test]
fn test_single_rail_opamp_compiles_and_clamps_top_only() {
    let mut config = support::config_for_spice(SINGLE_RAIL_OPAMP_SPICE, SR);
    // Observe the op-amp output node directly (no DC-block recentering).
    config.dc_block = false;
    // build_circuit_nodal panics on rustc failure — compiling at all is the
    // primary regression assertion.
    let circuit = support::build_circuit_nodal(SINGLE_RAIL_OPAMP_SPICE, &config, "single_rail_oa");
    assert!(
        !circuit.code.contains("-inf"),
        "generated code must not contain the invalid token `-inf`"
    );

    // Ideal output is ±23.5 V (gain -4.7 × 5 V drive): the top rail must
    // clamp at VCC=9, the bottom must NOT clamp (no VEE).
    let out = support::run_sine(&circuit, 200.0, 5.0, 1920, SR);
    support::assert_finite(&out);
    let max = out.iter().copied().fold(f64::MIN, f64::max);
    let min = out.iter().copied().fold(f64::MAX, f64::min);
    assert!(
        max <= 9.0 + 1e-6,
        "top rail must clamp at VCC=9 V, got max {max:.3} V"
    );
    assert!(
        max > 8.5,
        "output should actually reach the top rail, got max {max:.3} V"
    );
    assert!(
        min < -15.0,
        "bottom swing must be unclamped (no VEE), got min {min:.3} V \
         (a symmetric clamp at -9 V means the infinite bound was materialized)"
    );
}

/// AC-coupled variant of the single-rail op-amp: the output coupling cap makes
/// the Auto rail-mode resolver pick ActiveSet/ActiveSetBe, exercising the
/// active-set pinned-branch and rail-violation-check emitters (which formatted
/// both bounds unconditionally before the fix).
const SINGLE_RAIL_OPAMP_AC_SPICE: &str = "\
Single-rail op-amp AC-coupled
Rterm in 0 1Meg
Rg in oa_neg 10k
Rfb oa_neg oa_out 47k
U1 0 oa_neg oa_out OA1
Cout oa_out out 10u
Rload out 0 100k
.model OA1 OA(AOL=200000 ROUT=100 VCC=9)
";

#[test]
fn test_single_rail_opamp_active_set_compiles() {
    let config = support::config_for_spice(SINGLE_RAIL_OPAMP_AC_SPICE, SR);
    // Compiling is the regression assertion: the active-set pinned/check
    // emitters previously rendered `-inf` for the missing VEE.
    let circuit =
        support::build_circuit_nodal(SINGLE_RAIL_OPAMP_AC_SPICE, &config, "single_rail_oa_ac");
    assert!(
        !circuit.code.contains("-inf"),
        "generated code must not contain the invalid token `-inf`"
    );
    let out = support::run_sine(&circuit, 200.0, 5.0, 1920, SR);
    support::assert_finite(&out);
}

// ============================================================================
// PMOS body-effect sign (magnitude-space GAMMA correction)
// ============================================================================

fn pmos_body_effect_spice() -> &'static str {
    "\
PMOS CS with body effect
Cin in gate 1u
Vdd vdd 0 DC 12
R1 vdd gate 47k
R2 gate 0 100k
M1 drain gate source vdd PMOD
Rs vdd source 470
Rd drain 0 2.2k
Cout drain out 1u
Rload out 0 100k
.model PMOD PM(VTO=-2.0 KP=0.1 LAMBDA=0.01 GAMMA=0.5 PHI=0.65)
"
}

fn nmos_body_effect_spice() -> &'static str {
    "\
NMOS CS with body effect
Cin in gate 1u
Vdd vdd 0 DC 12
R1 vdd gate 47k
R2 gate 0 100k
M1 drain gate source 0 NMOD
Rd vdd drain 2.2k
Rs source 0 470
Cout drain out 1u
Rload out 0 100k
.model NMOD NM(VTO=2.0 KP=0.1 LAMBDA=0.01 GAMMA=0.5 PHI=0.65)
"
}

/// The body-effect VT update must apply the GAMMA correction in magnitude
/// space: `vt_eff = VT + sign·GAMMA·(sqrt(PHI+vsb⁺) − sqrt(PHI))`. For PMOS
/// (sign = -1, VT < 0) the old unsigned form SHRANK |VT| under reverse body
/// bias instead of growing it.
#[test]
fn test_pmos_body_effect_gamma_carries_channel_sign() {
    let spice = pmos_body_effect_spice();
    let config = support::config_for_spice(spice, SR);
    let (code, _n, _m) = support::generate_circuit_code_nodal(spice, &config);
    assert!(
        code.contains("+ (-1.0) * DEVICE_0_GAMMA *"),
        "PMOS body-effect update must negate the GAMMA term (magnitude-space |VT| increase)"
    );

    let nspice = nmos_body_effect_spice();
    let nconfig = support::config_for_spice(nspice, SR);
    let (ncode, _n, _m) = support::generate_circuit_code_nodal(nspice, &nconfig);
    assert!(
        ncode.contains("+ (1.0) * DEVICE_0_GAMMA *"),
        "NMOS body-effect update must keep the positive GAMMA term"
    );
}

// ============================================================================
// Self-heating thermal step (exact exponential, internal rate, diode 1/N)
// ============================================================================

/// Germanium-style self-heating diode (sad-bastard card shape) on the nodal
/// path. N=1.8 so the SPICE3f5 1/N exponent division is observable in the
/// emitted IS(T) code.
const SELF_HEATING_DIODE_SPICE: &str = "\
Self-heating diode clipper
Rin in mid 1k
D1 mid 0 DGE
C1 mid 0 100n
Cout mid out 1u
Rload out 0 100k
.model DGE D(IS=2e-7 N=1.8 RTH=1200 CTH=1e-4 EG=0.67 XTI=3.0)
";

#[test]
fn test_self_heating_exact_exponential_thermal_step() {
    let config = support::config_for_spice(SELF_HEATING_DIODE_SPICE, SR);
    let (code, _n, _m) = support::generate_circuit_code_nodal(SELF_HEATING_DIODE_SPICE, &config);

    // Exact exponential update toward steady state, not forward Euler.
    assert!(
        code.contains("(1.0 - (-dt / tau).exp())"),
        "thermal update must use the exact exponential step"
    );
    assert!(
        !code.contains("let d_tj ="),
        "forward-Euler thermal step must be gone"
    );
    // dt must track the LIVE sample rate (set_sample_rate), not a baked
    // compile-rate const — a baked dt gives the wrong thermal tau after a
    // host rate change (DK parity; 2026-07 review-round-2 fix). With
    // oversampling off, OVERSAMPLING_FACTOR is 1 and this is the base rate.
    assert!(
        code.contains("let dt = 1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64);"),
        "thermal dt must derive from state.current_sample_rate"
    );
    assert!(
        !code.contains("let dt = 1.0 / SAMPLE_RATE;"),
        "baked compile-rate thermal dt must be gone"
    );

    // SPICE3f5 diode IS(T): both exponents divided by the ideality factor N
    // (N=1.8 formats as 1.7999… under {:.17e} — match the leading digits).
    assert!(
        code.contains("powf(DEVICE_0_XTI / 1.7999") || code.contains("powf(DEVICE_0_XTI / 1.8000"),
        "diode IS(T) temperature exponent must be XTI/N"
    );
    assert!(
        code.contains("DEVICE_0_EG / (1.7999") || code.contains("DEVICE_0_EG / (1.8000"),
        "diode IS(T) bandgap exponent must be EG/(N*vt_nom)"
    );

    // The circuit must still compile and run sanely.
    let circuit = support::build_circuit_nodal(SELF_HEATING_DIODE_SPICE, &config, "sh_diode_exp");
    let out = support::run_sine(&circuit, 500.0, 1.0, 960, SR);
    support::assert_finite(&out);
}

#[test]
fn test_self_heating_dt_uses_internal_rate_under_oversampling() {
    let mut config = support::config_for_spice(SELF_HEATING_DIODE_SPICE, SR);
    config.oversampling_factor = 4;
    let (code, _n, _m) = support::generate_circuit_code_nodal(SELF_HEATING_DIODE_SPICE, &config);
    // The update runs OVERSAMPLING_FACTOR times per base sample, so dt is the
    // internal period — derived from the LIVE rate, not the baked
    // INTERNAL_SAMPLE_RATE const (wrong thermal tau after set_sample_rate;
    // 2026-07 review-round-2 fix, DK parity).
    assert!(
        code.contains("let dt = 1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64);"),
        "thermal dt must be the internal-rate period derived from state.current_sample_rate"
    );
    assert!(
        !code.contains("let dt = 1.0 / INTERNAL_SAMPLE_RATE;"),
        "baked internal-rate thermal dt must be gone"
    );
    // Sanity: it still compiles and runs at 4x OS.
    let circuit = support::build_circuit_nodal(SELF_HEATING_DIODE_SPICE, &config, "sh_diode_os4");
    let out = support::run_sine(&circuit, 500.0, 1.0, 960, SR);
    support::assert_finite(&out);
}

/// CTH=0 declares an instant thermal pole: the emitter must produce the
/// quasi-static form (no division by CTH — the old forward-Euler step
/// divided by CTH and produced inf/NaN).
const SELF_HEATING_DIODE_CTH0_SPICE: &str = "\
Self-heating diode quasi-static
Rin in mid 1k
D1 mid 0 DGE
C1 mid 0 100n
Cout mid out 1u
Rload out 0 100k
.model DGE D(IS=2e-7 N=1.8 RTH=1200 CTH=0 EG=0.67 XTI=3.0)
";

#[test]
fn test_self_heating_cth_zero_quasi_static() {
    let config = support::config_for_spice(SELF_HEATING_DIODE_CTH0_SPICE, SR);
    let (code, _n, _m) =
        support::generate_circuit_code_nodal(SELF_HEATING_DIODE_CTH0_SPICE, &config);
    // The Tj advance now comes from the shared `helpers::emit_thermal_tj_advance`
    // (twin-identical to the DK path). Its CTH<=0 branch emits the quasi-static
    // `Tj = clamp(Tss)` form directly, with no dt/tau/exp step.
    assert!(
        code.contains("state.device_0_tj = tss.clamp(200.0, 500.0);"),
        "CTH=0 must emit the quasi-static Tj = clamp(Tss) form"
    );
    assert!(
        !code.contains("(-dt / tau).exp()") && !code.contains("let tau ="),
        "CTH=0 must not emit the exponential step (tau would be zero)"
    );
    let circuit =
        support::build_circuit_nodal(SELF_HEATING_DIODE_CTH0_SPICE, &config, "sh_diode_qs");
    let out = support::run_sine(&circuit, 500.0, 1.0, 960, SR);
    support::assert_finite(&out);
}
