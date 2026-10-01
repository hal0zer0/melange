//! Nodal solver code emission methods.
//!
//! Contains the nodal entry point (`emit_nodal`) and all nodal-specific
//! methods: constants, state, LU solve/factor/back-solve, sparse LU,
//! Schur and full-LU process_sample, active-set resolve, device evaluation,
//! and voltage limiting.

mod behavioral;
mod constants;
mod device_eval;
mod equil;
mod full_lu;
mod full_lu_newton;
mod lu;
mod rail;
mod reset;
mod residual;
mod sat_ind;
mod schur;
mod sites;
mod stamps;
mod state;
mod substep;

use super::RustEmitter;
use crate::codegen::ir::{CircuitIR, DeviceParams};
use crate::codegen::CodegenError;
use equil::{build_equil_pattern, emit_equil_pattern_table};
use residual::emit_kcl_residual_fns;
use sites::{emit_k_seed_helpers, schur_exact_seed};

// ============================================================================
// Nodal solver emission (full N×N NR per sample, LU solve per iteration)
// ============================================================================

impl RustEmitter {
    /// Emit complete generated code for the nodal solver path.
    ///
    /// The nodal path differs from DK in that it does full N-dimensional
    /// Newton-Raphson with LU factorization per iteration, instead of
    /// precomputing S=A^{-1} and doing M-dimensional NR.
    ///
    /// Shared with DK: header, device models, SPICE limiting, safe_exp.
    /// Nodal-specific: constants, state, process_sample, LU solve, set_sample_rate.
    /// Emit the nodal solver. Returns the code and the sub-path actually taken,
    /// so the choice is observable instead of being an unreported internal
    /// decision (see [`crate::codegen::NodalSubPath`]).
    pub(super) fn emit_nodal(
        &self,
        ir: &CircuitIR,
    ) -> Result<(String, crate::codegen::NodalSubPath, Option<&'static str>), CodegenError> {
        let mut code = String::new();

        // Compute use_full_nodal flag FIRST — needed by emit_nodal_state for hot/cold split.
        let m = ir.topology.m;
        // ⚠ `> 0.0` here is NOT a typo for routing.rs's `>= 0.0`. See the
        // matching note at `codegen/routing.rs` (k_diag_unsafe) and
        // `tests/f7_routing_predicate_reachability_tests.rs`. The two sites read
        // different matrices and feed different decisions, and the sign
        // difference is MEASURED reachable: 20 dimensions across 8 circuits have
        // `K[i][i] == 0.0` with a live `N_i` column, so harmonising the spellings
        // would move the Schur-vs-full-LU decision on real, shipped circuits.
        let has_positive_k_with_current = if m > 0 {
            (0..m).any(|i| {
                let k_ii = ir.matrices.k[i * m + i];
                if k_ii <= 0.0 {
                    return false;
                }
                // Only flag positive K for dimensions with actual current injection
                // (VCA control ports have zero N_i column — positive K there is harmless)
                ir.sparsity.n_i.nz_by_row.iter().any(|row| row.contains(&i))
            })
        } else {
            false
        };
        let k_diag_min = if m > 0 {
            (0..m)
                .map(|i| ir.matrices.k[i * m + i])
                .fold(0.0_f64, f64::min)
        } else {
            0.0
        };
        // Check for degenerate K (all near-zero). When K ≈ 0, the Schur NR
        // has J = I (identity) — no device feedback. The device Jacobian that
        // provides essential damping for high-gain active elements (Boyle op-amps)
        // is invisible to the Schur decomposition. The full N×N LU NR must be
        // used instead, as it builds G_aug = A - N_I*J_dev*N_V each iteration.
        let k_max_abs = if m > 0 {
            (0..m * m)
                .map(|i| ir.matrices.k[i].abs())
                .fold(0.0_f64, f64::max)
        } else {
            0.0
        };
        let k_degenerate = m > 0 && k_max_abs < 1e-6;
        // The spectral radius of S*A_neg measures LINEAR prediction stability.
        // When rho < 1.0, the Schur NR's nonlinear correction (S*N_i*i_nl)
        // damps the marginal mode — the DK path uses identical matrices and
        // works fine. When rho > 1.0, the linear prediction genuinely amplifies
        // errors. With well-conditioned K (negative diagonal, not degenerate),
        // the Schur NR still handles rho slightly above 1.0 (up to
        // TRAP_BE_PROMOTION_RHO = 1.002, the growth margin on a spectral
        // radius). Only route to full-LU when K
        // itself is pathological OR spectral radius indicates true instability
        // that even the Schur NR can't damp.
        //
        // K magnitude check: circuits where nonlinear devices are the sole
        // current path between nodes (no parallel resistors) produce K entries
        // spanning many orders of magnitude (e.g. 5×10^11 for a transistor
        // ladder filter). The Schur NR forms J = I - J_dev*K; when |K| is
        // extreme, J_dev*K >> I and the 16×16 Gauss elimination is hopelessly
        // ill-conditioned. The full LU NR avoids K entirely by stamping device
        // Jacobians into G_aug directly.
        let k_ill_conditioned = m > 0 && k_max_abs > crate::codegen::routing::K_ILL_COND_MAX;
        // S matrix check: nodes connected only through caps and device junctions
        // (no resistive path) produce extreme S = A^{-1} entries. This is
        // invariant to FA reduction — FA changes N_V/N_I/K but not A/S. When
        // S has entries > 1e6, the Schur prediction v = S*rhs amplifies roundoff
        // into the nonlinear solver, producing garbage regardless of K magnitude.
        let n = ir.topology.n;
        let s_max_abs = if n > 0 && !ir.matrices.s.is_empty() {
            ir.matrices
                .s
                .iter()
                .map(|v| v.abs())
                .fold(0.0_f64, f64::max)
        } else {
            0.0
        };
        let s_ill_conditioned = s_max_abs > crate::codegen::routing::S_ILL_COND_MAX;

        // Linearized device bypass: when triodes/BJTs are linearized at DC OP,
        // their small-signal conductances (gm, 1/rp) stamp into G, creating
        // high-gain coupling chains. These inflate S = A^{-1}, K = N_V*S*N_I,
        // and K diagonals (both negative and positive) — but the values
        // represent correct high-gain transfer functions, not numerical
        // instability. The Schur NR iterates in M-space using K directly and
        // converges in ~4 iterations regardless of chain length. The full-LU
        // NR redundantly re-solves the constant linear partition every
        // iteration, degrading to ~39 iterations for long chains.
        //
        // When linearized devices are present, suppress ALL magnitude-based
        // guards (positive K, extreme K diagonal, K ill-conditioned, S
        // ill-conditioned) and rely solely on:
        //   1. k_degenerate (K ≈ 0 makes Schur trivial: J = I, no damping)
        //   2. spectral radius (rho > 1.0 means linear prediction unstable)
        let has_linearized = ir.topology.num_linearized_devices > 0;
        let linearized_bypass = has_linearized && !k_degenerate;
        // `spectral_radius_s_aneg` is measured on the pair that ships: the
        // BE S·A_neg on a backward-Euler build, the trap pair otherwise.
        // Label it so a pinned-BE compile log isn't mistaken for the trap
        // discriminator's quiescent rho.
        let rho_pair = if ir.solver_config.backward_euler {
            "BE pair"
        } else {
            "trap pair"
        };
        if linearized_bypass {
            log::warn!(
                "Nodal: {} linearized devices — suppressing magnitude guards \
                 (max|K|={:.2e}, max|S|={:.2e}, K_diag_min={:.2e}, \
                 pos_K_with_I={}). Spectral radius = {:.4} ({}).",
                ir.topology.num_linearized_devices,
                k_max_abs,
                s_max_abs,
                k_diag_min,
                has_positive_k_with_current,
                ir.matrices.spectral_radius_s_aneg,
                rho_pair
            );
        }

        let k_well_conditioned = m > 0
            && !k_degenerate
            && !has_positive_k_with_current
            && !k_ill_conditioned
            && !s_ill_conditioned
            && k_diag_min > -1e12;
        // Spectral radius thresholds:
        // - k_well_conditioned: TRAP_BE_PROMOTION_RHO = 1.002 (the growth
        //   margin on a spectral radius, tight)
        // - linearized_bypass: 1.05 (relaxed — the Schur NR corrects the
        //   linear prediction every sample via M-dim NR, and the runtime
        //   BE fallback catches any sample where trapezoidal diverges.
        //   At rho=1.01, the prediction error is ~1%/sample — trivially
        //   corrected by the first NR iteration)
        // - pathological K (no bypass): 1.0 (strict — unknown K structure)
        let schur_unstable = if k_well_conditioned {
            ir.matrices.spectral_radius_s_aneg > crate::codegen::stability::TRAP_BE_PROMOTION_RHO
        } else if linearized_bypass {
            ir.matrices.spectral_radius_s_aneg > 1.05
        } else {
            ir.matrices.spectral_radius_s_aneg > 1.0
        };
        // High-magnitude K with linearized devices: even with the linearization
        // bypass otherwise keeping us on Schur, if |K| >> 1 the M-space NR
        // becomes hypersensitive — a mA change in `i_nl` swings `v_d` by tens
        // of volts, pushing junction exponentials into nonphysical regions and
        // producing the oscillate-between-wrong-branches pattern that
        // wurli-power-amp hits (M=14 Schur, max|K|=9.9e3, ~20% NR failure at
        // 10 mV input). Full-LU NR operates in v-space where the capacitor
        // matrix naturally bounds step size, and converges on the same
        // circuit. Threshold 1e3: K*i_nl with mA currents gives 1 V/step, the
        // edge of where device `safe_exp` clamping still gives physical
        // results.
        // Near-marginal trapezoidal stability with many coupled NR dims and
        // high-magnitude K pushes Schur NR into the oscillate-between-wrong-
        // branches pattern. Full-LU NR operates in v-space where the
        // capacitor matrix naturally bounds step size, and converges on
        // circuits that Schur can't.
        //
        // Motivating case: wurli-power-amp (M=14 after linearized Q9,
        // max|K|=9.9e3, spectral_radius(S*A_neg) under BE = 0.9999 — i.e.
        // right at the L-stability edge). On Schur: ~20% NR failure at 10 mV
        // input with divergent output. On full-LU: zero NaN resets, peak
        // bounded, gain matches ngspice within 1 dB. All three gate
        // conditions had to match to exclude M=14 circuits that are stable
        // on Schur (sad-bastard rho=0.979, tungsten-thunder-horse rho=0.993,
        // uniquorn rho=0.994):
        //   - max|K| > 1e3: strong nonlinear coupling (i_nl-step amplifies
        //     in v-space)
        //   - M ≥ 10: enough coupled dims that Schur step-direction error
        //     compounds (basic-bitch at M=8 has max|K|=6e4 and is fine)
        //   - rho > 0.995: marginal trap/BE stability means any NR
        //     over-correction persists through several samples
        let k_large_magnitude_with_linearization = linearized_bypass
            && k_max_abs > 1.0e3
            && m >= 10
            && ir.matrices.spectral_radius_s_aneg > 0.995;

        // Saturating inductors (single inductors, and the magnetizing branch of
        // a shared-core T-model) are genuine nonlinear devices on their
        // augmented branch row, stamped inside the full-LU NR loop (flux integral
        // Φ(i) residual, differential L_diff Jacobian — SATURATING_TRANSFORMERS.md
        // §3). They therefore force the full-LU path unconditionally, even at
        // M=0: a linear-plus-saturating-inductor circuit has no M-devices and
        // would otherwise route to Schur / the M==0 direct-LU fast-path, neither
        // of which iterates on the inductor nonlinearity.
        let force_full_lu_sat = !ir.saturating_inductors.is_empty();
        // Structural requirements for full-LU, as opposed to conditioning
        // heuristics. Saturating inductors are stamped as nonlinear
        // devices on their augmented branch row INSIDE the full-LU NR loop, and
        // behavioral B-sources are stamped in node space only on the full-LU
        // path. The Schur reduction cannot express either, so these are not
        // overridable — forcing Schur here would emit code that silently drops
        // the nonlinearity rather than code that is merely slower or less well
        // conditioned.
        let structurally_needs_full_lu = force_full_lu_sat || !ir.behavioral_sources.is_empty();
        let auto_use_full_nodal = structurally_needs_full_lu
            || if !ir.behavioral_sources.is_empty() {
                // Behavioral B-sources are stamped in node space only on the full-LU
                // path (the Schur reduction can't express their rectangular control).
                true
            } else if linearized_bypass {
                // Only k_degenerate, spectral radius, or large-|K| can block Schur
                k_degenerate || schur_unstable || k_large_magnitude_with_linearization
            } else {
                has_positive_k_with_current
                    || k_diag_min < -1e12
                    || k_degenerate
                    || k_ill_conditioned
                    || s_ill_conditioned
                    || schur_unstable
            };

        // Apply the `--nodal-subpath` override. `Auto` is the shipping path and
        // is byte-identical to the pre-flag emitter. The forcing modes are
        // diagnostic escape hatches in the spirit of `--force-trap`: they let
        // the sub-path be isolated as a variable, which is otherwise impossible
        // because the emitter chooses it for you.
        use crate::codegen::NodalSubPathOverride;
        let use_full_nodal = match ir.solver_config.nodal_sub_path_override {
            NodalSubPathOverride::Auto => auto_use_full_nodal,
            NodalSubPathOverride::FullLu => {
                if !auto_use_full_nodal {
                    log::warn!(
                        "Nodal: --nodal-subpath full-lu overrides the auto choice \
                         (auto selected Schur for this circuit). Full-LU is always \
                         structurally valid but slower; this is a diagnostic mode, \
                         not a production setting."
                    );
                }
                true
            }
            NodalSubPathOverride::Schur => {
                if structurally_needs_full_lu {
                    let reason = if force_full_lu_sat {
                        "saturating inductors are stamped as nonlinear devices \
                         inside the full-LU NR loop"
                    } else {
                        "behavioral B-sources are stamped in node space only on the \
                         full-LU path"
                    };
                    return Err(CodegenError::InvalidConfig(format!(
                        "--nodal-subpath schur refused: this circuit STRUCTURALLY requires \
                         full-LU ({reason}); the Schur reduction cannot express it, so \
                         forcing Schur would emit a solver that silently drops the \
                         nonlinearity. This is not a conditioning heuristic and is not \
                         overridable."
                    )));
                }
                if auto_use_full_nodal {
                    log::warn!(
                        "Nodal: --nodal-subpath schur overrides the auto choice (auto \
                         selected full-LU on conditioning grounds: k_degenerate={}, \
                         k_ill_conditioned={}, s_ill_conditioned={}, schur_unstable={}). \
                         The Schur reduction may be inaccurate or diverge on this \
                         circuit. Diagnostic mode — do not ship.",
                        k_degenerate,
                        k_ill_conditioned,
                        s_ill_conditioned,
                        schur_unstable
                    );
                }
                false
            }
        };
        // Which condition placed this circuit on the full-LU sub-path — recorded
        // in the glow provenance reason (`nodal-full-lu:<trigger>`) so a consumer
        // reads WHY sub-sample-fire is inactive, not just that it is. Best-effort,
        // following the sub-path decision's own precedence.
        let full_lu_trigger: &str = if !use_full_nodal {
            ""
        } else if matches!(
            ir.solver_config.nodal_sub_path_override,
            crate::codegen::NodalSubPathOverride::FullLu
        ) && !auto_use_full_nodal
        {
            "override"
        } else if force_full_lu_sat {
            "saturating-inductor"
        } else if !ir.behavioral_sources.is_empty() {
            "behavioral-source"
        } else if has_positive_k_with_current {
            "positive-k"
        } else if k_diag_min < -1e12 {
            "k-diag-negative"
        } else if k_degenerate {
            "k-degenerate"
        } else if k_ill_conditioned {
            "k-ill-conditioned"
        } else if s_ill_conditioned {
            "s-ill-conditioned"
        } else if schur_unstable {
            "schur-unstable"
        } else if k_large_magnitude_with_linearization {
            "k-large-linearized"
        } else {
            "unknown"
        };
        // Fail-loud refusal (design review): a relaxing-section / delayed-
        // overvoltage / subnormal (KSUB) glow on the full-LU sub-path runs a
        // MIXED, silently-wrong model — the full-LU device eval solves the static
        // maintaining line `i=(v−V0)/RS` for the lit branch while the strike seed
        // and extinction test read `glow_lit_eval`'s section current. This has
        // masqueraded as a circuit failure (a dead divider) for days, invisible
        // because the route line is only a stderr WARN. Refuse rather than emit
        // it; the author picks a visible exit. Overridable ONLY with the explicit
        // `allow_static_glow_on_full_lu` (which runs today's static line and marks
        // the sections inert in provenance).
        if use_full_nodal && !ir.solver_config.allow_static_glow_on_full_lu {
            if let Some((dev_num, _)) = ir.device_slots.iter().enumerate().find(|(_, slot)| {
                matches!(
                    &slot.params,
                    DeviceParams::Glow(gp)
                        if gp.has_sections() || gp.has_d() || gp.ksub > 0.0
                )
            }) {
                return Err(CodegenError::InvalidConfig(format!(
                    "glow device #{dev_num} uses a relaxing-section / delayed-overvoltage \
                     / subnormal (KSUB) lit branch, but this circuit routes the nodal \
                     FULL-LU sub-path (trigger: {full_lu_trigger}, max|S|={s_max_abs:.2e}). \
                     On full-LU the lit branch is evaluated as the STATIC maintaining line \
                     (v0+RS·i) while the strike seed and extinction test read the section \
                     model — a mixed model that silently produces wrong output (a dead \
                     divider). Refusing to emit it (design review). NB: the route is \
                     per (deck, SAMPLE RATE) — this same deck may route Schur (which honors \
                     the sections) at another rate, so read the sub-path from the actual \
                     run, not from a compile at a different rate. Choose one:\n\
                     \x20 (a) --nodal-subpath schur : force the Schur route, which honors \
                     the section model. Allowed here (this is a conditioning heuristic, not \
                     a structural requirement). BUT note the auto-router chose full-LU FOR \
                     this circuit (trigger: {full_lu_trigger}) — forcing Schur runs the very \
                     route it rejected as unreliable, so verify the printed spectral radius \
                     and cross-check --backward-euler; the accuracy of the forced route is \
                     then yours to own.\n\
                     \x20 (b) remove the section / D / KSUB keys from the .model to run the \
                     static card knowingly.\n\
                     \x20 (c) --allow-static-glow-on-full-lu : run today's static line on \
                     full-LU with the section keys INERT (stamped glow_sections: inert \
                     (full-lu) in the Build line and provenance). Diagnostic use survives; \
                     silence does not."
                )));
            }
        }

        // Sub-sample fire (variable-dt glow-strike re-solve) is implemented on
        // the Schur sub-path only (Stage A). A forced `on` is refused here
        // rather than silently ignored; `auto` falls back to the whole-sample
        // latch on full-LU, and the IR flag is cleared on a local clone so the
        // provenance header and every downstream emitter agree that the
        // feature is inactive.
        let ir_subsample_cleared: CircuitIR;
        let ir: &CircuitIR = if use_full_nodal && ir.solver_config.subsample_fire {
            if ir.solver_config.subsample_fire_mode == crate::codegen::SubsampleFireMode::On {
                return Err(CodegenError::InvalidConfig(
                    "--subsample-fire on: this circuit takes the nodal full-LU sub-path, \
                     where the variable-dt glow-strike re-solve is not implemented \
                     (Stage A covers nodal-Schur only). Use --nodal-subpath schur if the \
                     circuit permits it, or --subsample-fire auto/off."
                        .to_string(),
                ));
            }
            log::warn!(
                "Nodal: sub-sample fire is inactive on the full-LU sub-path (Stage A \
                 implements nodal-Schur only); glow strikes stay whole-sample latched."
            );
            let mut cleared = ir.clone();
            cleared.solver_config.subsample_fire = false;
            ir_subsample_cleared = cleared;
            &ir_subsample_cleared
        } else {
            ir
        };
        // The dense `lu_solve` helper is emitted whenever any generated code
        // path needs it. The full-LU nodal path always needs it. The Schur
        // path needs it for the sub-step ladder (M > 0), and when op-amp rail
        // handling is in `ActiveSet` mode, because the post-NR constrained
        // resolve does one dense LU solve per clamp-active sample. Without this
        // gate the Schur path would reference an undefined function and fail
        // to compile.
        use crate::codegen::OpampRailMode;
        let needs_lu_solve = use_full_nodal
            || ir.topology.m > 0
            || (matches!(
                ir.solver_config.opamp_rail_mode,
                OpampRailMode::ActiveSet | OpampRailMode::ActiveSetBe
            ) && !ir.opamps.is_empty());

        // Behavioral B-sources are stamped only in the primary trapezoidal NR
        // loop, so the sub-step and BE fallbacks are gated OFF for behavioral
        // circuits (they rebuild from base G/C and would drop the source). The
        // ActiveSetBe op-amp rail mode, however, RESOLVES rails via the BE
        // fallback (BE+pin does not ring where trap+pin does). With that fallback
        // gated off, an ActiveSetBe behavioral circuit would silently lose its
        // rail resolution. Emitting behavioral sources inside the fallback loops
        // (a tracked follow-up — see BEHAVIORAL_SOURCES.md) is the real fix; until
        // then this combination is unsupported. It does not occur in the shipped
        // corpus. Fail loud rather than mis-resolve the rails.
        if use_full_nodal
            && !ir.behavioral_sources.is_empty()
            && matches!(ir.solver_config.opamp_rail_mode, OpampRailMode::ActiveSetBe)
        {
            return Err(CodegenError::UnsupportedTopology(
                "behavioral B-source(s) with an ActiveSetBe op-amp rail mode on the nodal \
                 full-LU path is not yet supported: ActiveSetBe resolves op-amp rails via the \
                 backward-Euler fallback, which is gated off for behavioral circuits (it \
                 rebuilds from the base matrices and would drop the behavioral source). Stamp \
                 the behavioral source inside the fallback loops (tracked follow-up) to lift \
                 this, or select a non-ActiveSetBe rail mode."
                    .to_string(),
            ));
        }

        // Now emit header, constants, device models, state (needs use_full_nodal)
        let glow_prov =
            super::dk_emitter::GlowProvenance::for_nodal(ir, use_full_nodal, full_lu_trigger);
        // Record the resolved nodal sub-path in provenance so a deck can assert
        // it (a silent Schur↔full-LU flip is otherwise only an unread WARN).
        let resolved_sub_path = if use_full_nodal {
            crate::codegen::NodalSubPath::FullLu
        } else {
            crate::codegen::NodalSubPath::Schur
        };
        code.push_str(&self.emit_header(ir, &glow_prov, Some(resolved_sub_path))?);
        code.push_str(&self.emit_nodal_constants(ir, use_full_nodal));
        // Authentic circuit noise (Phase 1: thermal). Returns an empty
        // `NoiseEmission` when noise mode is Off — every fragment is "" and
        // emission is byte-identical to a noiseless build.
        let noise = self.build_noise_emission(ir);
        if noise.enabled {
            code.push_str(&noise.top_level);
        }
        code.push_str(&self.emit_device_models(ir)?);
        // `emit_nodal_state` emits `rebuild_matrices` and every dynamic-parameter
        // setter, recording each literal `(row, col)` those setters stamp. That
        // recorded set is the load-bearing input to the equilibration pattern
        // (see `EquilPattern`), so it must be collected here — before the LU
        // helpers below consume it — rather than re-derived from the IR.
        let mut setter_stamps: std::collections::BTreeSet<(usize, usize)> =
            std::collections::BTreeSet::new();
        code.push_str(&self.emit_nodal_state(ir, use_full_nodal, &noise, &mut setter_stamps));
        let equil_pat = build_equil_pattern(ir, &setter_stamps);
        if let Some(p) = &equil_pat {
            log::info!(
                "Equilibration: sparse pattern {} of {} entries ({:.1}% density)",
                p.len(),
                ir.topology.n * ir.topology.n,
                100.0 * p.density()
            );
            code.push_str(&emit_equil_pattern_table(p));
        }
        // `invert_n` exists solely to build the Schur matrices (S, S_be, S_sub)
        // in `rebuild_matrices`. The full-LU path never reads those matrices —
        // it solves against A / A_be / chord_lu directly — so on that path
        // `rebuild_matrices` skips the inversions and `invert_n` has no callers.
        if !use_full_nodal {
            code.push_str(&Self::emit_nodal_invert_n(ir));
            // Sub-sample fire scratch Schur triple + builder ("" when inactive).
            code.push_str(&super::subsample_fire::emit_subsample_schur_builder(ir));
        }

        if use_full_nodal {
            if has_positive_k_with_current {
                log::warn!(
                    "Nodal: using full N×N LU NR (positive K diagonal with current injection)"
                );
            } else if k_degenerate {
                log::warn!(
                    "Nodal: using full N×N LU NR (K degenerate, max|K|={:.2e} — device Jacobian provides essential damping)",
                    k_max_abs
                );
            } else if k_ill_conditioned {
                log::warn!(
                    "Nodal: using full N×N LU NR (max|K|={:.2e}, extreme K magnitude — device nodes lack resistive paths)",
                    k_max_abs
                );
            } else if s_ill_conditioned {
                log::warn!(
                    "Nodal: using full N×N LU NR (max|S|={:.2e}, cap-only nodes lack resistive paths — Schur prediction unreliable)",
                    s_max_abs
                );
            } else if schur_unstable {
                log::warn!(
                    "Nodal: using full N×N LU NR (spectral_radius(S*A_neg) = {:.4} ({}), Schur feedback unstable)",
                    ir.matrices.spectral_radius_s_aneg,
                    rho_pair
                );
            } else if k_large_magnitude_with_linearization {
                log::warn!(
                    "Nodal: using full N×N LU NR (max|K|={:.2e} with linearized devices — Schur NR in M-space hypersensitive to i_nl steps; v-space NR converges better)",
                    k_max_abs
                );
            } else {
                log::warn!(
                    "Nodal: using full N×N LU NR (K_diag_min={:.1}, ill-conditioned)",
                    k_diag_min
                );
            }
            code.push_str(&Self::emit_nodal_lu_solve(ir, equil_pat.as_ref()));
            code.push_str(&Self::emit_nodal_lu_factor(ir, equil_pat.as_ref()));
            code.push_str(&Self::emit_nodal_lu_back_solve(ir));
            // Sparse LU when available (dense kept as fallback)
            if ir.sparsity.lu.is_some() {
                code.push_str(&Self::emit_sparse_lu_factor(ir, equil_pat.as_ref()));
                code.push_str(&Self::emit_sparse_lu_back_solve(ir));
            }
            code.push_str(&Self::emit_nodal_process_sample(ir, &noise, &setter_stamps));
        } else {
            if linearized_bypass {
                log::warn!(
                    "Nodal: using Schur NR (M={}, {} linearized devices, rho={:.4} ({}))",
                    m,
                    ir.topology.num_linearized_devices,
                    ir.matrices.spectral_radius_s_aneg,
                    rho_pair
                );
            }
            // The Schur solve itself needs no N×N LU; the active-set resolve
            // and the sub-step ladder (M > 0) do, and the ladder also reads the
            // node-space KCL residual.
            if needs_lu_solve {
                code.push_str(&Self::emit_nodal_lu_solve(ir, equil_pat.as_ref()));
            }
            if ir.topology.m > 0 {
                code.push_str(&emit_kcl_residual_fns(ir, &setter_stamps));
            }
            if schur_exact_seed(ir, use_full_nodal) {
                code.push_str(&emit_k_seed_helpers());
            }
            code.push_str(&Self::emit_nodal_schur_process_sample(
                ir,
                &noise,
                &setter_stamps,
            )?);
        }

        if ir.solver_config.oversampling_factor > 1 {
            code.push_str(&Self::emit_oversampler(ir));
        } else if ir.solver_config.has_inject_or_tap() {
            // No oversampling, but `.inject`/`.tap` still emit a private
            // process_sample_inner; wrap it in the array-API public entry.
            code.push_str(&Self::emit_inject_wrapper_1x(ir));
        }

        let sub_path = if use_full_nodal {
            crate::codegen::NodalSubPath::FullLu
        } else {
            crate::codegen::NodalSubPath::Schur
        };
        // Carry the trigger out with the decision. Reporting the route without
        // the reason invites the reader to supply one, and the CLI did exactly
        // that: it printed the nodal spectral radius as the deciding value for
        // every full-LU route, including the ones a radius had no part in.
        let trigger = if full_lu_trigger.is_empty() {
            None
        } else {
            Some(full_lu_trigger)
        };
        Ok((code, sub_path, trigger))
    }
}
