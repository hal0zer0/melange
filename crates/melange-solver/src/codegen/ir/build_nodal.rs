//! The nodal-route `CircuitIR` builder.

use super::*;

impl CircuitIR {
    /// The nodal builder; `dc_result` and `promoted` as in [`Self::build_dk`].
    pub(super) fn build_nodal(
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        dc_result: &dc_op::DcOpResult,
        promoted: Option<&crate::codegen::ring::RingVerdict>,
    ) -> Result<Self, CodegenError> {
        let n_nodes = mna.n;
        let n_aug = mna.n_aug;
        let m = mna.m;

        // Resolve the effective integration scheme: CLI flags override the
        // `.integrator` netlist directive, which overrides auto-promotion.
        let (cfg_backward_euler, cfg_force_trap, mut integrator_selection) =
            resolve_integrator_pref(config, netlist.integrator);

        if m > dk::MAX_M {
            return Err(CodegenError::InvalidConfig(dk::max_m_refusal(m)));
        }

        // Build augmented G/C matrices (includes inductor branch variables)
        let mut aug = mna.build_augmented_matrices();
        let n = aug.n_nodal;

        // Gmin regularization: prevent singular Jacobians on floating nodes.
        // The same floor the DC operating-point solver stamps
        // (`dc_op::build_dc_system`).
        for i in 0..n_nodes {
            aug.g[i][i] += GMIN_REGULARISATION;
        }

        let sample_rate = config.sample_rate;
        let internal_rate = sample_rate * config.oversampling_factor as f64;
        // Provisional integrator. When the ring predicate (`codegen::ring`)
        // promotes a default-trapezoidal build, this builder runs again with
        // `promoted` set (see `CircuitIR::ring_promotion`), and the promotion
        // block below swaps in the BE matrices already built as the transient
        // fallback and flips `alpha`/`solver_config` in place.
        let mut alpha = if cfg_backward_euler {
            internal_rate
        } else {
            2.0 * internal_rate
        };
        let alpha_be = internal_rate;

        // Validate output_nodes against circuit node count
        for (i, &node) in config.output_nodes.iter().enumerate() {
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (circuit node count)",
                    i, node, n_nodes
                )));
            }
        }

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Build A = G + alpha*C, A_neg = alpha*C - G (trapezoidal) or alpha*C (BE).
        // A/A_neg/A_be/A_neg_be are built once below, after an author's
        // AOL_TRANSIENT_CAP has possibly modified aug.g.
        //
        // Behavioral B-sources are stamped current-only (no trapezoidal history
        // term yet), which is exact under backward Euler (steady state G·v = i)
        // but only half-right under trapezoidal. BE is also the stable choice for
        // strongly-nonlinear sources (atan2 discriminators). Force it on.
        // (Trapezoidal + a behavioral i_prev history term is a future refinement
        // — see docs/aidocs/BEHAVIORAL_SOURCES.md.)
        let be = cfg_backward_euler || !mna.behavioral_sources.is_empty();
        if be && !cfg_backward_euler {
            // Behavioral forcing outranks a `.integrator trap` pin (the
            // current-only stamp is simply wrong under trap), so it also
            // overwrites a provisional trap selection.
            integrator_selection = IntegratorSelection::BeBehavioral;
        }
        // A/A_neg/A_be/A_neg_be are built ONCE below, after the selective
        // Rule-D' Gm cap has (possibly) modified aug.g. A former pre-cap build
        // here was immediately overwritten by that rebuild and never read in
        // between, so only the declarations remain.
        let mut a_flat = vec![0.0f64; n * n];
        let mut a_neg_flat = vec![0.0f64; n * n];
        let mut a_be_flat = vec![0.0f64; n * n];
        let mut a_neg_be_flat = vec![0.0f64; n * n];

        // Expand N_v (m × n_aug → m × n_nodal) and N_i (n_aug × m → n_nodal × m)
        let mut n_v_flat = vec![0.0f64; m * n];
        for i in 0..m {
            for j in 0..n_aug {
                n_v_flat[i * n + j] = mna.n_v[i][j];
            }
        }
        let mut n_i_flat = vec![0.0f64; n * m];
        for i in 0..n_aug {
            for j in 0..m {
                n_i_flat[i * m + j] = mna.n_i[i][j];
            }
        }

        // DC sources at ×1 on every row (current sources on node rows, V_dc
        // on voltage-source rows). Both integrators enter each source once,
        // at n+1, so this one vector is the shipped `rhs_const` (trapezoidal
        // charge form, forced or promoted BE) and the BE fallback's
        // `rhs_const_be`.
        let rhs_const_be = rhs_const_1x(mna, n);

        // Op-amp transient AOL cap, from the card's `AOL_TRANSIENT_CAP` only:
        // the VCCS stamp's excess Gm is removed from G for the transient
        // matrices (the DC operating point keeps the full AOL). Routing sends
        // such a card nodal, so DK never needs it.
        for oa in &mna.opamps {
            if oa.n_out_idx == 0 {
                continue;
            }
            let aol_cap = effective_aol_cap(oa);
            if !aol_cap.is_finite() || oa.aol <= aol_cap {
                continue;
            }
            let gm_full = oa.aol / oa.r_out;
            let gm_capped = aol_cap / oa.r_out;
            let delta = gm_full - gm_capped;
            let o = oa.n_out_idx - 1;
            if o >= n {
                continue;
            }
            // Un-stamp direction must mirror the (corrected) VCCS stamp:
            // stamp is np -= gm / nm += gm, so the cap removes delta with
            // np += delta / nm -= delta.
            if oa.n_plus_idx > 0 && oa.n_plus_idx - 1 < n {
                aug.g[o][oa.n_plus_idx - 1] += delta;
            }
            if oa.n_minus_idx > 0 && oa.n_minus_idx - 1 < n {
                aug.g[o][oa.n_minus_idx - 1] -= delta;
            }
            log::info!(
                "Selective Gm cap on op-amp {}: AOL {:.0} → {:.0} (delta_Gm={:.1} S)",
                oa.name,
                oa.aol,
                aol_cap,
                delta,
            );
        }

        // Now rebuild A/A_neg from the (possibly modified) G matrix
        for i in 0..n {
            for j in 0..n {
                let g = aug.g[i][j];
                let c = aug.c[i][j];
                a_flat[i * n + j] = g + alpha * c;
                a_neg_flat[i * n + j] = if be { alpha * c } else { alpha * c - g };
                a_be_flat[i * n + j] = g + alpha_be * c;
                a_neg_be_flat[i * n + j] = alpha_be * c;
            }
        }
        // Re-zero augmented (VS/VCVS/xfmr/VCA/behavioral) history rows in A_neg and
        // A_neg_be — but EXCLUDE parasitic-BJT internal nodes, which are physical
        // G/C nodes that must keep their trapezoidal history (zeroing them makes the
        // DC OP not a trap fixed point → a z=-1 collector-row ring). The helper masks
        // them out; this inline loop previously did not (design review root fix).
        zero_augmented_history_rows(&mut a_neg_flat, n, n_nodes, n_aug, &mna.bjt_internal_nodes);
        zero_augmented_history_rows(
            &mut a_neg_be_flat,
            n,
            n_nodes,
            n_aug,
            &mna.bjt_internal_nodes,
        );

        // Flatten G and C (with the selective Rule-D' Gm cap applied) for codegen constants
        let g_matrix = dk::flatten_matrix(&aug.g, n, n);
        let c_matrix = dk::flatten_matrix(&aug.c, n, n);

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: mna.num_devices,
            n_aug,
            augmented_inductors: true,
            num_linearized_devices: mna.linearized_triodes.len() + mna.linearized_bjts.len(),
            linearized_checks: linearized_checks(mna),
            history_zero_rows: history_zero_rows(n, n_nodes, n_aug, &mna.bjt_internal_nodes),
        };

        let rail_mode = resolve_opamp_rail_mode(mna, config.opamp_rail_mode);
        log::info!(
            "Op-amp rail mode: {} ({})",
            rail_mode.mode,
            rail_mode.reason.as_str()
        );
        let rail_mode_reason =
            opamp_rail_reason_with_override(mna, config.opamp_rail_mode, &rail_mode);

        // Provisional solver_config. `alpha` and `backward_euler` may still be
        // updated by the auto-BE promotion block below.
        let mut solver_config = SolverConfig {
            sample_rate,
            alpha,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            extra_input_nodes: config.extra_input_nodes.clone(),
            extra_input_resistances: config.extra_input_resistances.clone(),
            oversampling_factor: config.oversampling_factor,
            runtime_oversampling: None,
            output_scales: config.output_scales.clone(),
            output_clamp_v: config.output_clamp_v,
            backward_euler: be,
            // Resolved after the auto-BE promotion block below (needs the
            // final `solver_config.backward_euler`).
            runtime_be_latch: false,
            breakpoint_be: false,
            opamp_rail_mode: rail_mode.mode,
            opamp_rail_mode_reason: rail_mode_reason.clone(),
            emit_dc_op_recompute: config.emit_dc_op_recompute,
            nodal_sub_path_override: config.nodal_sub_path_override,
            allow_static_glow_on_full_lu: config.allow_static_glow_on_full_lu,
            injections: config.injections.clone(),
            taps: config.taps.clone(),
            subsample_fire_mode: config.subsample_fire,
            subsample_lit_factor: config.subsample_lit_factor,
            // Resolved below next to `breakpoint_be` (needs `has_glow`).
            subsample_fire: false,
        };

        // Compute S = A^{-1} for Schur complement NR (O(M³) instead of O(N³) per iteration)
        let mut s_flat = invert_flat_matrix(&a_flat, n)?;
        let mut k_flat = if m > 0 {
            compute_k_from_s(&s_flat, &n_v_flat, &n_i_flat, n, m)
        } else {
            Vec::new()
        };

        // Also compute S_be = A_be^{-1} for backward Euler fallback
        let s_be_flat = invert_flat_matrix(&a_be_flat, n)?;
        let k_be_flat = if m > 0 {
            compute_k_from_s(&s_be_flat, &n_v_flat, &n_i_flat, n, m)
        } else {
            Vec::new()
        };

        // Compute spectral radius of S * A_neg to detect Schur instability.
        // When rho(S * A_neg) > 1, the linear prediction v_pred = S*(A_neg*v_prev + ...)
        // amplifies errors exponentially, and the nodal emitter's
        // `schur_unstable` gate routes to full LU NR instead.
        //
        // This measurement does not decide backward-Euler promotion: that is
        // the ring predicate (`codegen::ring`, see `CircuitIR::ring_promotion`),
        // on the charge-form propagator. The power-iteration analysis
        // (including the dominant-eigenvalue sign) is in
        // `crate::codegen::stability`.
        let trap_stability = crate::codegen::stability::analyze_trap_stability_deflated(
            &s_flat,
            &a_neg_flat,
            n,
            &config.input_node_indices(),
        );
        let mut spectral_radius_s_aneg = trap_stability.rho;
        // Diagnostic copy of the trap-side rho: `spectral_radius_s_aneg` is
        // overwritten with the post-promotion BE rho when auto-BE fires, but
        // CodegenMeta must report the value that TRIGGERED the promotion.
        // When the build is already BE (flag/directive/behavioral), the pair
        // analyzed above IS the BE pair — no trap discriminator ran, so the
        // field stays 0.0 (matching the DK path and the field contract).
        let trap_discriminator_rho = promoted.map_or(0.0, |v| v.rho);
        if spectral_radius_s_aneg > 0.99 {
            log::info!(
                "Nodal: spectral_radius(S*A_neg) = {:.4} ({} pair), dominant_sign = {:+.0}, \
                 max_abs_s = {:.4e} \
                 (marginally stable; Schur used when K well-conditioned)",
                spectral_radius_s_aneg,
                if be { "BE" } else { "trap" },
                trap_stability.dominant_sign,
                trap_stability.max_abs_s
            );
        }

        // Auto-BE promotion for the nodal path. The decision is the ring
        // predicate's (`codegen::ring`), taken on the finished trapezoidal IR
        // at its DC operating point; this build is repeated with `promoted`
        // set (see `CircuitIR::ring_promotion`). The BE matrices are already
        // built above as the transient fallback (`a_be_flat`,
        // `a_neg_be_flat`, `s_be_flat`, `k_be_flat`, `rhs_const_be`); a
        // promoted build clones them into the primary slot and flips
        // `alpha`/`solver_config.backward_euler` so every downstream emitter
        // picks BE formulas.
        if promoted.is_some() && !be {
            alpha = alpha_be;
            a_flat = a_be_flat.clone();
            a_neg_flat = a_neg_be_flat.clone();
            s_flat = s_be_flat.clone();
            k_flat = k_be_flat.clone();
            solver_config.backward_euler = true;
            solver_config.alpha = alpha;
            integrator_selection = IntegratorSelection::BeAuto;
            // Recompute rho on BE matrices for the emitter's Schur-vs-full-LU
            // gate (`spectral_radius_s_aneg`, consumed by
            // `emit_nodal`'s `schur_unstable`, `nodal_emitter/mod.rs`). This power iteration is
            // intentionally the coarse, un-deflected, fixed-100-iteration
            // form — it is the historically-calibrated value the emitter
            // gate thresholds (1.002/1.05/1.0) were tuned against (see
            // wurli-power-amp notes below); do not swap it for the shared
            // `analyze_trap_stability_deflated` without re-validating every
            // threshold against the golden-audio suite.
            let new_rho = if n > 0 && !s_flat.is_empty() {
                let mut x = vec![1.0 / (n as f64).sqrt(); n];
                let mut rho = 0.0f64;
                for _ in 0..100 {
                    let mut ax = vec![0.0; n];
                    for i in 0..n {
                        for j in 0..n {
                            ax[i] += a_neg_flat[i * n + j] * x[j];
                        }
                    }
                    let mut y = vec![0.0; n];
                    for i in 0..n {
                        for j in 0..n {
                            y[i] += s_flat[i * n + j] * ax[j];
                        }
                    }
                    let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
                    if norm < 1e-30 {
                        break;
                    }
                    rho = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
                    x.fill(0.0);
                    for (i, yi) in y.iter().enumerate() {
                        x[i] = yi / norm;
                    }
                }
                rho
            } else {
                0.0
            };
            // `new_rho` (above) is the coarse un-deflected metric that drives
            // the emitter's Schur-vs-full-LU gate. Separately, cross-check
            // with the accurate/deflated analyzer and log a diagnostic that
            // correctly distinguishes "genuinely unstable circuit" from "BE
            // matrix-builder defect" — see `log_be_post_promotion_check` doc
            // comment for the full rationale and verification.
            if new_rho > crate::codegen::stability::BE_POST_PROMOTION_LIMIT {
                crate::codegen::stability::log_be_post_promotion_check(
                    "Nodal",
                    &s_flat,
                    &a_neg_flat,
                    n,
                    &config.input_node_indices(),
                );
            }
            spectral_radius_s_aneg = new_rho;
        }

        // Emit the runtime BE-latch safety net for genuine trapezoidal builds
        // with a nonlinear system only: nothing to catch once we are already on
        // backward Euler (by flag, `.integrator be`, behavioral sources, or the
        // promotion above), and `--force-trap` / `.integrator trap` opt out
        // entirely. The `m > 0` gate matches the auto-BE promotion: a passive
        // linear circuit has nothing to seed a Nyquist cycle, so it stays
        // byte-identical (no detector emitted).
        //
        // Saturating inductors make a circuit nonlinear with M = 0 (the flux
        // law lives on an augmented row, not in N_i), so they qualify on their
        // own. They were once excluded because the latch forces the BE fallback
        // every sample and the old decimated saturation path never updated the
        // BE matrices; that path is gone, and the flux device is stamped at
        // every Newton site at the site's own alpha.
        let has_saturating = mna.has_saturating_inductor();
        solver_config.runtime_be_latch =
            !solver_config.backward_euler && !cfg_force_trap && (m > 0 || has_saturating);

        // Event-triggered breakpoint backward-Euler at a reactive `.switch`
        // swap or a glow-discharge strike. Independent of `cfg_force_trap` (a
        // targeted correctness fix at an explicit event, not the Nyquist-latch
        // heuristic that force-trap disables). Emitted only when the circuit
        // has such an event and runs on trap; the machinery is byte-inert
        // until a `set_switch_*` call on a C/L switch or a glow strike arms
        // it, so golden fixtures (which never toggle) are unaffected. Gated
        // off for BE builds (nothing to fix). Pot setters and the per-sample
        // `.runtime R` setter never arm it.
        // Only a switch that swaps a capacitor or an inductor needs it: under
        // the charge form the history carries q_dot = C·dx/dt and no G term,
        // so a conductance change (a pot, a resistor-only switch) leaves the
        // carried state consistent, and a backward-Euler sample there only
        // costs first-order accuracy. A reactance change leaves q_dot built on
        // the old value.
        let has_reactive_switch = mna.switches.iter().any(|sw| {
            sw.components
                .iter()
                .any(|c| matches!(c.component_type, 'C' | 'L'))
        });
        // A glow-discharge device is a runtime conductance swap of the same
        // kind (RS lit <-> ROFF dark, ~1e5 step) that fires on its own latch
        // instead of on a setter. On the nodal route the lit phase is held on
        // the BE matrices (trap is A- but not L-stable: its damping factor on
        // the stiff lit mode tends to -1 and rings into the cathode diode's
        // breakdown at 44.1-96 kHz), so the machinery must be emitted for
        // glow decks too. DK is not re-armed by the glow (unchanged behaviour).
        let has_glow = mna
            .nonlinear_devices
            .iter()
            .any(|d| d.device_type == crate::mna::NonlinearDeviceType::Glow);
        // An op-amp rail pin or release is not a source: under the charge form
        // the pinned solve and the release commit a consistent q_dot, and a
        // backward-Euler sample there only re-seeds q_dot with a backward
        // difference across the edge (measured: it no longer lowers the
        // residual and costs output accuracy).
        solver_config.breakpoint_be =
            !solver_config.backward_euler && (has_reactive_switch || has_glow);

        // Sub-sample fire: variable-dt breakpoint re-solve at a glow strike.
        // Nodal route only (this builder), latched device required; the
        // emitter clears it again on the full-LU sub-path (Stage A = Schur).
        // `off` → false; `auto`/`on` → gated on a glow device being present.
        // Without a glow device nothing can fire, so the machinery is not
        // emitted and the generated source is byte-identical to pre-feature.
        solver_config.subsample_fire =
            config.subsample_fire != crate::codegen::SubsampleFireMode::Off && has_glow;
        if config.subsample_fire == crate::codegen::SubsampleFireMode::On && !has_glow {
            crate::diag_warn!(
                "--subsample-fire on: circuit has no latched (glow) device; the flag is inert."
            );
        }

        // Charge (companion) form: a trapezoidal build ships the history
        // matrix `alpha·C` and the DC sources at ×1 (they enter at n+1 only).
        // The whole-system `alpha·C − G` built above is what the stability
        // discriminators were calibrated against, so it is replaced only here,
        // after they ran.
        let a_neg_flat = if solver_config.backward_euler {
            a_neg_flat
        } else {
            charge_form_history(&c_matrix, n, alpha, &topology.history_zero_rows)
        };
        let rhs_const = rhs_const_be.clone();
        let matrices = Matrices {
            s: s_flat,
            k: k_flat,
            a_neg: a_neg_flat,
            n_v: n_v_flat,
            n_i: n_i_flat,
            rhs_const,
            g_matrix,
            c_matrix,
            a_matrix: a_flat,
            a_matrix_be: a_be_flat,
            a_neg_be: a_neg_be_flat,
            rhs_const_be,
            s_be: s_be_flat,
            k_be: k_be_flat,
            spectral_radius_s_aneg,
        };

        // The DC OP was solved on this MNA (`mna.g`, which still has the full
        // op-amp Gm; only the `aug.g` copy was stripped).
        let dc_config = dc_op_config(mna, config);
        // Build device info with MNA so FA reductions are reflected in dimensions
        let device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;

        // Judge significance over exactly what the nodal path emits: all N rows,
        // inductor branch currents included (`dc_operating_point` is resized to
        // `n` below and baked whole). Judging only the first `n_aug` rows dropped
        // the operating point of a circuit whose only DC quantity is an inductor
        // current (a current-biased grounded inductor: every node at 0 V), which
        // then started from i_L = 0 and settled over L/R — seconds, for a
        // henry-class winding on the 1 Ω input.
        let has_dc_op = dc_result.v_node.iter().take(n).any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_op_method = format!("{:?}", dc_result.method);
        let dc_op_rail_pin = dc_result.rail_pin.label();
        let dc_op_iterations = dc_result.iterations;
        // Paired with `dc_operating_point` (plain, non-IC quiescent point).
        // Do NOT repoint this at the IC-seeded solve — see
        // `dc_nl_currents_ic_seed` below and its DK-path twin comment for
        // why: the plain (dc_operating_point, dc_nl_currents) pair is what
        // the per-sample magnitude/NaN-reset fallback resets state to, and
        // it must stay self-consistent independent of any IC= seed.
        let dc_nl_currents = dc_result.i_nl.clone();
        let has_dc_sources = !mna.voltage_sources.is_empty() || !mna.current_sources.is_empty();

        // IC=-bearing capacitors: solve a second, independent initial-state
        // operating point (each such cap temporarily replaced by an ideal
        // voltage source of its IC value) used to seed `v_prev`. `None` when
        // the netlist has no `IC=` caps — see `mna.capacitor_ics`.
        //
        // `dc_nl_currents_ic_seed` (i_nl at that same IC-consistent point) is
        // captured alongside it and used ONLY to seed `i_nl_prev` at
        // construction/reset() time, paired with `v_prev_ic_seed` — see the
        // identical reasoning in the DK-path twin of this block. It must
        // NEVER be paired with the plain `dc_operating_point`/`dc_nl_currents`
        // (that pairing belongs to the reset fallback, see the
        // "v_prev / i_nl_prev inconsistency" comment a few lines below).
        let mut dc_nl_currents_ic_seed: Option<Vec<f64>> = None;
        let v_prev_ic_seed = dc_op::solve_ic_seeded_operating_point(mna, &device_slots, &dc_config)
            .map(|ic_result| {
                if !ic_result.converged {
                    crate::diag_warn!(
                        "IC= initial-state solve did not converge (method: {:?}); v_prev seed uses best estimate",
                        ic_result.method
                    );
                }
                dc_nl_currents_ic_seed = Some(ic_result.i_nl.clone());
                let mut v = ic_result.v_node;
                v.resize(n, 0.0);
                v.truncate(n);
                v
            });
        let q_dot_ic_seed = match (&v_prev_ic_seed, &dc_nl_currents_ic_seed) {
            (Some(x), Some(i_nl)) if !solver_config.backward_euler => {
                Some(q_dot_at(&matrices, n, m, x, i_nl))
            }
            _ => None,
        };

        // Resize DC OP to n_nodal dimension. Do NOT clamp op-amp outputs
        // to supply rails here — the emitted per-sample active-set resolve
        // already handles rail violations at runtime, and pre-clamping the
        // stored DC_OP creates a v_prev / i_nl_prev inconsistency (v_out
        // clamped but `dc_nl_currents` comes from the unclamped solve, so
        // the downstream diode states encoded in `i_nl` don't match the
        // clamped nodes) that slowly drifts the state over thousands of
        // samples before NR blows up (observed 4kbuscomp failure: ~2300
        // stable samples then 1e27 V explosion).
        let mut dc_operating_point = dc_result.v_node.clone();
        dc_operating_point.resize(n, 0.0);

        // Sparsity analysis (K is now computed for Schur complement NR)
        //
        // Behavioral B-source Jacobian stamps (`emit_behavioral_jacobian`) hit
        // positions outside the device N_i·J_dev·N_v envelope — the aug/terminal
        // rows × every referenced-node column. They MUST be part of the symbolic
        // pattern: a position absent from the pattern is never eliminated by the
        // straight-line sparse schedule, silently dropping the stamp and
        // converging to a wrong fixed point.
        let behavioral_stamp_patterns: Vec<lu::BehavioralStamp> = mna
            .behavioral_sources
            .iter()
            .map(|b| lu::BehavioralStamp {
                is_voltage: b.v_ext_idx.is_some(),
                aug_row: b.aug_row,
                n_plus_idx: b.n_plus_idx,
                n_minus_idx: b.n_minus_idx,
                referenced_node_indices: b.referenced_node_indices.values().copied().collect(),
            })
            .collect();
        // Belt-and-braces: if any V={} source somehow lacks its aug_row, the
        // stamp geometry cannot be guaranteed complete — prefer correctness and
        // route to the dense LU (which pivots at runtime and needs no pattern).
        let behavioral_pattern_complete = behavioral_stamp_patterns
            .iter()
            .all(|b| !b.is_voltage || b.aug_row.is_some());
        let mut g_aug_density = 0.0f64;
        let lu_sparsity = if m > 0 && behavioral_pattern_complete {
            // Compute G_aug = A - N_i*J_dev*N_v sparsity pattern
            // (+ behavioral B-source stamp positions)
            let g_aug_pattern = lu::compute_g_aug_pattern(
                &matrices.a_matrix,
                &matrices.n_i,
                &matrices.n_v,
                n,
                m,
                &device_slots,
                &behavioral_stamp_patterns,
            );
            let g_aug_nnz: usize = g_aug_pattern.iter().map(|r| r.len()).sum();
            let density = g_aug_nnz as f64 / (n * n) as f64;
            g_aug_density = density;
            log::info!(
                "Sparse LU: G_aug pattern has {} nonzeros out of {} ({:.1}% density)",
                g_aug_nnz,
                n * n,
                density * 100.0
            );
            // Only use sparse LU if matrix is sufficiently sparse (< 40% density)
            // and large enough to benefit (N >= 8)
            if density < 0.4 && n >= 8 {
                let elim_order = lu::amd_ordering(&g_aug_pattern, n);
                let row_swaps = lu::find_row_swaps(&g_aug_pattern, &elim_order, n);
                let lu_plan = lu::symbolic_lu(&g_aug_pattern, &elim_order, &row_swaps, n);
                Some(lu_plan)
            } else {
                log::info!(
                    "Sparse LU: skipping (density {:.1}%, N={})",
                    density * 100.0,
                    n
                );
                None
            }
        } else {
            None
        };

        let sparsity = SparseInfo {
            a_neg: analyze_matrix_sparsity(&matrices.a_neg, n, n),
            n_v: analyze_matrix_sparsity(&matrices.n_v, m, n),
            n_i: analyze_matrix_sparsity(&matrices.n_i, n, m),
            a_neg_be: if matrices.a_neg_be.is_empty() {
                MatrixSparsity {
                    rows: n,
                    cols: n,
                    nnz: 0,
                    nz_by_row: vec![Vec::new(); n],
                }
            } else {
                analyze_matrix_sparsity(&matrices.a_neg_be, n, n)
            },
            k: analyze_matrix_sparsity(&matrices.k, m, m),
            k_be: if matrices.k_be.len() == m * m && m > 0 {
                analyze_matrix_sparsity(&matrices.k_be, m, m)
            } else {
                MatrixSparsity {
                    rows: m,
                    cols: m,
                    nnz: 0,
                    nz_by_row: vec![Vec::new(); m],
                }
            },
            lu: lu_sparsity,
            g_aug_density,
        };

        let named_constants = build_named_constants(mna, topology.n_nodes);
        let runtime_sources: Vec<RuntimeSourceIR> = mna
            .runtime_sources
            .iter()
            .map(|rt| RuntimeSourceIR {
                vs_name: rt.vs_name.clone(),
                field_name: rt.field_name.clone(),
                vs_row: rt.vs_row,
            })
            .collect();
        let behavioral_sources = build_behavioral_sources_ir(mna);
        let behavioral_param_consts: Vec<(String, f64)> = netlist
            .params
            .iter()
            .map(|p| (p.name.clone(), p.value))
            .collect();
        let behavioral_scalar_runtimes: Vec<ScalarRuntimeIR> = netlist
            .runtime_scalars
            .iter()
            .map(|r| ScalarRuntimeIR {
                name: r.name.clone(),
                field_name: r.field_name.clone(),
                min: r.min_value,
                max: r.max_value,
                default: r.min_value.clamp(r.min_value, r.max_value),
            })
            .collect();

        let ir = CircuitIR {
            metadata,
            topology,
            solver_mode: SolverMode::Nodal,
            solver_config,
            matrices,
            dc_operating_point,
            v_prev_ic_seed,
            device_node_indices: Self::device_node_indices_for(&device_slots, mna),
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_nl_currents_ic_seed,
            q_dot_ic_seed,
            dc_op_converged,
            linearize_bias_unconverged: linearize_bias_unconverged(mna),
            dc_op_method,
            dc_op_rail_pin,
            dc_op_iterations,
            dc_block: config.dc_block,
            pots: mna
                .pots
                .iter()
                .map(|p| PotentiometerIR {
                    g_nominal: p.g_nominal,
                    node_p: p.node_p,
                    node_q: p.node_q,
                    min_resistance: p.min_resistance,
                    max_resistance: p.max_resistance,
                    grounded: p.grounded,
                    runtime_field: p.runtime_field.clone(),
                })
                .collect(),
            wiper_groups: mna
                .wiper_groups
                .iter()
                .map(|wg| WiperGroupIR {
                    cw_pot_index: wg.cw_pot_index,
                    ccw_pot_index: wg.ccw_pot_index,
                    total_resistance: wg.total_resistance,
                    default_position: wg.default_position,
                    label: wg.label.clone(),
                })
                .collect(),
            gang_groups: mna
                .gang_groups
                .iter()
                .map(|gg| GangGroupIR {
                    label: gg.label.clone(),
                    pot_members: gg
                        .pot_members
                        .iter()
                        .map(|&(pot_idx, inverted)| GangPotMemberIR {
                            pot_index: pot_idx,
                            min_resistance: mna.pots[pot_idx].min_resistance,
                            max_resistance: mna.pots[pot_idx].max_resistance,
                            inverted,
                        })
                        .collect(),
                    wiper_members: gg
                        .wiper_members
                        .iter()
                        .map(|&(wg_idx, inverted)| GangWiperMemberIR {
                            wiper_group_index: wg_idx,
                            total_resistance: mna.wiper_groups[wg_idx].total_resistance,
                            inverted,
                        })
                        .collect(),
                    default_position: gg.default_position,
                })
                .collect(),
            switches: {
                // Build inductor name → augmented row mapping for switch L components.
                // In augmented MNA, each inductor's L value lives on the C matrix diagonal
                // at row n_aug + offset (not at circuit node rows).
                let mut inductor_aug_rows: std::collections::HashMap<String, usize> =
                    std::collections::HashMap::new();
                let mut var_idx = n_aug; // Original n_aug in the full system
                for ind in &mna.inductors {
                    inductor_aug_rows.insert(ind.name.to_ascii_uppercase(), var_idx);
                    var_idx += 1;
                }
                for ci in &mna.coupled_inductors {
                    inductor_aug_rows.insert(ci.l1_name.to_ascii_uppercase(), var_idx);
                    inductor_aug_rows.insert(ci.l2_name.to_ascii_uppercase(), var_idx + 1);
                    var_idx += 2;
                }
                for group in &mna.transformer_groups {
                    for (widx, name) in group.winding_names.iter().enumerate() {
                        inductor_aug_rows.insert(name.to_ascii_uppercase(), var_idx + widx);
                    }
                    var_idx += group.num_windings;
                }
                // Inductor augmented row indices are used directly at the N dimension.

                mna.switches
                    .iter()
                    .enumerate()
                    .map(|(idx, sw)| {
                        // Collect inductor names in this switch for mutual lookup
                        let switch_inductor_names: std::collections::HashSet<String> = sw
                            .components
                            .iter()
                            .filter(|c| c.component_type == 'L')
                            .map(|c| c.name.to_ascii_uppercase())
                            .collect();

                        // Build mutual entries for coupled pairs where at least one winding is in this switch
                        let mut mutual_entries = Vec::new();
                        for ci in &mna.coupled_inductors {
                            let l1 = ci.l1_name.to_ascii_uppercase();
                            let l2 = ci.l2_name.to_ascii_uppercase();
                            if switch_inductor_names.contains(&l1)
                                || switch_inductor_names.contains(&l2)
                            {
                                if let (Some(&ra), Some(&rb)) =
                                    (inductor_aug_rows.get(&l1), inductor_aug_rows.get(&l2))
                                {
                                    mutual_entries.push(SwitchMutualEntry {
                                        row_a: ra,
                                        row_b: rb,
                                        coupling: ci.coupling,
                                    });
                                }
                            }
                        }
                        for group in &mna.transformer_groups {
                            for i in 0..group.num_windings {
                                for j in (i + 1)..group.num_windings {
                                    let ni = group.winding_names[i].to_ascii_uppercase();
                                    let nj = group.winding_names[j].to_ascii_uppercase();
                                    if switch_inductor_names.contains(&ni)
                                        || switch_inductor_names.contains(&nj)
                                    {
                                        if let (Some(&ra), Some(&rb)) =
                                            (inductor_aug_rows.get(&ni), inductor_aug_rows.get(&nj))
                                        {
                                            mutual_entries.push(SwitchMutualEntry {
                                                row_a: ra,
                                                row_b: rb,
                                                coupling: group.coupling_matrix[i][j],
                                            });
                                        }
                                    }
                                }
                            }
                        }

                        SwitchIR {
                            index: idx,
                            label: sw.label.clone().unwrap_or_else(|| {
                                sw.components
                                    .iter()
                                    .map(|c| c.name.as_str())
                                    .collect::<Vec<_>>()
                                    .join("+")
                            }),
                            components: sw
                                .components
                                .iter()
                                .map(|comp| {
                                    let augmented_row = if comp.component_type == 'L' {
                                        inductor_aug_rows
                                            .get(&comp.name.to_ascii_uppercase())
                                            .copied()
                                    } else {
                                        None
                                    };
                                    SwitchComponentIR {
                                        name: comp.name.clone(),
                                        component_type: comp.component_type,
                                        node_p: comp.node_p,
                                        node_q: comp.node_q,
                                        nominal_value: comp.nominal_value,
                                        augmented_row,
                                    }
                                })
                                .collect(),
                            positions: sw.positions.clone(),
                            num_positions: sw.positions.len(),
                            mutual_entries,
                        }
                    })
                    .collect()
            },
            saturating_inductors: {
                // Build list of inductors with ISAT (iron-core saturation).
                // Reuse the same augmented row mapping as switches.
                let mut sat_inds = Vec::new();
                // aug_row for inductor i is n_aug + i (one augmented row each,
                // matching the switch row mapping); no separate counter needed.
                for (i, ind) in mna.inductors.iter().enumerate() {
                    if let Some(isat) = ind.isat {
                        let (lair, lair_source) = match &ind.shared_core {
                            // Single inductor: the floor is a fraction of its own L.
                            None => {
                                let (lair, src) = crate::parser::resolve_air_floor(ind.air_floor);
                                (lair, src.to_string())
                            }
                            // Shared core: this is the magnetizing branch. Its
                            // floor was resolved against the whole core when the
                            // group was built (mna.rs), as a fraction of this branch.
                            Some(core) => {
                                if let Some(k) = core.implicit_k.filter(|&k| k < 0.9995) {
                                    let floor = core.floor_frac * k;
                                    let k_air = floor / ((1.0 - k) + floor);
                                    crate::diag_warn!(
                                        "Saturating shared core ({}): coupling k = {k} is looser than real \
                                         audio iron (1 - k ~ 1e-5..1e-4); the implied coupling in \
                                         deep saturation is k_air = {k_air:.3}.",
                                        ind.name
                                    );
                                }
                                (core.floor_frac, core.floor_reading.clone())
                            }
                        };
                        // Never silent: a default is announced, and so is an
                        // explicit zero floor.
                        if ind.air_floor.is_none() {
                            let d = crate::parser::DEFAULT_AIR_FLOOR;
                            let whose = if ind.shared_core.is_some() {
                                format!(
                                    "the core's magnetizing inductance floors at {d:e} of the \
                                     reference winding's inductance"
                                )
                            } else {
                                format!(
                                    "its saturated inductance floors at {d:e} of its inductance"
                                )
                            };
                            let what = if ind.shared_core.is_some() {
                                format!("shared core ({})", ind.name)
                            } else {
                                format!("inductor {}", ind.name)
                            };
                            crate::diag_warn!(
                                "Saturating {what}: no LAIR= or CORE= given, so {whose} \
                                 (rule-of-thumb for ungapped steel). Set LAIR=<fraction> from a \
                                 measured or core-data value, or CORE=gapped|steel|nickel."
                            );
                        } else if lair == 0.0 {
                            crate::diag_warn!(
                                "Saturating inductor {}: LAIR=0 gives a zero final slope; driven far \
                                 past ISAT (beyond ~10x) its current is set by a numerical, not a \
                                 physical, floor.",
                                ind.name
                            );
                        }
                        sat_inds.push(SaturatingInductorIR {
                            name: ind.name.clone(),
                            l0: ind.value,
                            isat,
                            aug_row: n_aug + i,
                            inductor_index: i,
                            lair,
                            lair_source,
                        });
                    }
                }
                sat_inds
            },
            opamps: mna
                .opamps
                .iter()
                .filter(|oa| {
                    (oa.vcc.is_finite() || oa.vee.is_finite() || oa.sr.is_finite())
                        && oa.n_out_idx > 0
                })
                .map(opamp_ir_from_info)
                .collect(),
            sparsity,
            noise: build_noise_ir(config, netlist, mna),
            named_constants,
            runtime_sources,
            behavioral_sources,
            behavioral_param_consts,
            behavioral_scalar_runtimes,
            trap_discriminator_rho,
            integrator_selection,
            integration_reason: String::new(),
            be_latch_reference: None,
        };
        // Measured, not a gate (design review): a railing op-amp driving a
        // saturating inductor crosses the core's knee within one sample with
        // the full rail across it, and trapezoidal integration at 1x
        // overshoots the inductor's internal current. The output is not
        // affected, and 4x is accurate.
        let rails_into_core = !ir.saturating_inductors.is_empty()
            && matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::ActiveSet
                    | crate::codegen::OpampRailMode::ActiveSetBe
            )
            && ir
                .opamps
                .iter()
                .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite());
        if rails_into_core && config.oversampling_factor < 4 {
            crate::diag_warn!(
                "An op-amp that can rail drives a saturating inductor: at 1x, the \
                 inductor's internal current can overshoot by up to ~13 % where the \
                 op-amp rails into the core (output H1 is unaffected); 4x is accurate. \
                 Consider --oversampling 4 (or `.oversampling 4` in the deck)."
            );
        }
        // Measured, not a gate (design review): a railing op-amp switches rail
        // to rail within a sample, and at 1x the harmonics of those edges fold
        // back below Nyquist. `active-set-be` shows less of it only because
        // backward Euler dissipates the edges; oversampling is the remedy.
        let railing_at_1x = rail_mode.reason == OpampRailModeReason::AcCoupledDownstream
            && ir.solver_config.opamp_rail_mode == crate::codegen::OpampRailMode::ActiveSet
            && ir
                .opamps
                .iter()
                .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
            && config.oversampling_factor == 1;
        if railing_at_1x {
            crate::diag_warn!(
                "An op-amp here can rail (rail mode active-set, chosen automatically). \
                 Rail clipping makes harmonics above Nyquist, which alias at 1x: on a \
                 single-supply overdrive, a 16 kHz tone at 48 kHz put a 66 Hz alias on \
                 the output at 18x the level of its fundamental; at 4x the alias was \
                 0.5 mV. Consider --oversampling 4 (or `.oversampling 4` in the deck)."
            );
        }
        // In augmented MNA every inductor has its own branch row; an L switch
        // component without one would be stamped into the node block as a
        // capacitance, which is a different circuit. Never emit that.
        for sw in &ir.switches {
            for comp in &sw.components {
                if comp.component_type == 'L' && comp.augmented_row.is_none() {
                    return Err(CodegenError::InvalidConfig(format!(
                        ".switch '{}': inductor {} has no branch row in this build, so \
                         the switch cannot change it.",
                        sw.label, comp.name
                    )));
                }
            }
        }
        Ok(ir)
    }
}
