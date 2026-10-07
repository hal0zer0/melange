//! The DK-route `CircuitIR` builder.

use super::*;

impl CircuitIR {
    /// The DK builder, from the operating point `dc_result` the build ships.
    /// `promoted` is the ring predicate's verdict when this
    /// build is the backward-Euler rebuild of a trapezoidal IR it promoted
    /// (see [`Self::ring_promotion`]); `None` builds the scheme the flags and
    /// directive select.
    pub(super) fn build_dk(
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        dc_result: &dc_op::DcOpResult,
        promoted: Option<&crate::codegen::ring::RingVerdict>,
    ) -> Result<Self, CodegenError> {
        let n = kernel.n; // = n_aug (system dimension)
        let n_nodes = kernel.n_nodes; // original circuit node count
        let m = kernel.m;

        // Resolve the effective integration scheme: CLI flags override the
        // `.integrator` netlist directive, which overrides auto-promotion.
        let (cfg_backward_euler, _, mut integrator_selection) =
            resolve_integrator_pref(config, netlist.integrator);

        if m > crate::dk::MAX_M {
            return Err(CodegenError::UnsupportedTopology(crate::dk::max_m_refusal(
                m,
            )));
        }

        // `--subsample-fire on` is nodal-only: the DK kernel bakes S = A^-1 at
        // compile time, so a variable-dt sub-step (matrices at rate/alpha) has
        // no home here. Refuse loudly rather than emit a solver that silently
        // ignores the request. `auto` simply stays off on this route.
        if config.subsample_fire == crate::codegen::SubsampleFireMode::On {
            return Err(CodegenError::InvalidConfig(
                "--subsample-fire on requires the nodal route: the DK kernel bakes \
                 S = A^-1 at compile time and cannot carry the variable-dt glow-strike \
                 sub-step. Use --solver nodal (or --subsample-fire auto, which is \
                 inert on DK)."
                    .to_string(),
            ));
        }

        // Inductors are augmented-MNA branch rows (DkKernel::from_mna_augmented,
        // as the build makes every inductor kernel): L lives in C on rows
        // n_aug..n. A companion-model kernel (DkKernel::from_mna on an inductor
        // deck, the runtime LinearSolver's form) has no codegen path.
        // The companion fields are deprecated (0.1.14) along with `LinearSolver`.
        #[allow(deprecated)]
        let companion = (
            kernel.inductors.len(),
            kernel.coupled_inductors.len(),
            kernel.transformer_groups.len(),
        );
        if companion != (0, 0, 0) {
            return Err(CodegenError::UnsupportedTopology(format!(
                "a DK kernel with {} companion-modelled inductor(s), {} coupled pair(s) \
                 and {} transformer group(s) cannot be code-generated: inductors are \
                 generated as augmented-MNA branch rows. Build the kernel with \
                 DkKernel::from_mna_augmented (as melange_solver::build::build does).",
                companion.0, companion.1, companion.2,
            )));
        }
        let augmented_inductors = kernel.n > mna.n_aug;

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: kernel.num_devices,
            n_aug: mna.n_aug,
            augmented_inductors,
            num_linearized_devices: mna.linearized_triodes.len() + mna.linearized_bjts.len(),
            linearized_checks: linearized_checks(mna),
            history_zero_rows: history_zero_rows(n, n_nodes, mna.n_aug, &mna.bjt_internal_nodes),
        };

        let os_factor = config.oversampling_factor;
        let internal_rate = config.sample_rate * os_factor as f64;

        // Store the raw G and C matrices for runtime sample rate recomputation.
        // The MNA G matrix already includes input conductance (stamped before kernel build).
        // When augmented inductors are used, kernel.n > mna.n_aug, so we need the
        // augmented G/C (with inductor KCL/KVL/L stamps) at the full n_nodal dimension.
        let (g_matrix, c_matrix) = if augmented_inductors {
            let aug = mna.build_augmented_matrices();
            (
                dk::flatten_matrix(&aug.g, n, n),
                dk::flatten_matrix(&aug.c, n, n),
            )
        } else {
            (
                dk::flatten_matrix(&mna.g, n, n),
                dk::flatten_matrix(&mna.c, n, n),
            )
        };

        // When oversampling, the shipped trap matrices are rebuilt at the
        // internal (oversampled) rate. Skipped when BE ships (forced, or this
        // is the promoted rebuild) — no trap pair is shipped then.
        let os_trap_s = if os_factor > 1 && !cfg_backward_euler && promoted.is_none() {
            let a_flat = build_dk_trap_a_at_rate(&g_matrix, &c_matrix, n, internal_rate);
            let s = invert_flat_matrix(&a_flat, n)?;
            Some((a_flat, s))
        } else {
            None
        };

        // Auto-promotion to backward Euler is decided on the finished
        // trapezoidal IR by the ring predicate (`codegen::ring`), which needs
        // the DC operating point; this build is then repeated with
        // `promoted` set. See `CircuitIR::ring_promotion`.
        let auto_be = promoted.is_some();
        let trap_discriminator_rho = promoted.map_or(0.0, |v| v.rho);
        let be = cfg_backward_euler || auto_be;

        if auto_be {
            integrator_selection = IntegratorSelection::BeAuto;
        }
        let alpha = if be {
            internal_rate
        } else {
            2.0 * internal_rate
        };

        // Validate output_nodes against circuit node count
        for (i, &node) in config.output_nodes.iter().enumerate() {
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (circuit node count)",
                    i, node, n_nodes
                )));
            }
        }

        let rail_mode = resolve_opamp_rail_mode(mna, config.opamp_rail_mode);
        log::info!(
            "Op-amp rail mode: {} ({})",
            rail_mode.mode,
            rail_mode.reason.as_str()
        );
        let rail_mode_reason =
            opamp_rail_reason_with_override(mna, config.opamp_rail_mode, &rail_mode);

        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            extra_input_nodes: config.extra_input_nodes.clone(),
            extra_input_resistances: config.extra_input_resistances.clone(),
            oversampling_factor: os_factor,
            output_scales: config.output_scales.clone(),
            output_clamp_v: config.output_clamp_v,
            backward_euler: be,
            // DK codegen path does not emit the runtime BE-latch net yet.
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
            // DK bakes S = A^-1 at compile time and cannot carry a variable-dt
            // sub-step; `auto` is inert here, `on` is refused above.
            subsample_fire: false,
        };

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Each branch also yields the A its S was inverted from (and the BE
        // fallback's A_be) for the structural-sparsity rounding bound.
        let (mut matrices, a_of_s, a_of_s_be) = if os_factor > 1 && be {
            // BE + oversampling: build backward-Euler matrices at the
            // INTERNAL rate, mirroring the os=1 BE branch below. This branch
            // previously did not exist — the oversampled path unconditionally
            // baked trap matrices (alpha = 2·rate, A_neg = alpha·C − G,
            // rhs_const ×2) even when `be` was true, while
            // `solver_config.backward_euler` made the emitter use BE
            // semantics everywhere else. That mixed integrator shipped a
            // wrong DC fixed point (the ×2 trap rhs_const against BE update
            // equations halves the effective nonlinear bias current) and the
            // first runtime `rebuild_matrices()` silently swapped the
            // convention to consistent BE, stepping the operating point
            // mid-signal.
            let (s, a_neg_flat, rhs_const_be, a_of_s) = build_dk_be_matrices_at_rate(
                &g_matrix,
                &c_matrix,
                n,
                n_nodes,
                mna.n_aug,
                internal_rate,
                mna,
            )?;
            // Diagnostic parity with the nodal path's post-promotion check
            // (previously DK had none at all — a genuinely mis-stamped BE
            // build here would have shipped silently). See
            // `log_be_post_promotion_check` for the accurate/deflated
            // methodology and the "growing pole vs matrix defect" rationale.
            crate::codegen::stability::log_be_post_promotion_check(
                "DK",
                &s,
                &a_neg_flat,
                n,
                &config.input_node_indices(),
            );
            let k = compute_k_from_s(&s, &kernel.n_v, &kernel.n_i, n, m);
            let a_of_s_be: Option<Vec<f64>> = None;
            (
                Matrices {
                    s,
                    a_neg: a_neg_flat,
                    k,
                    n_v: kernel.n_v.clone(),
                    n_i: kernel.n_i.clone(),
                    rhs_const: rhs_const_be,
                    g_matrix,
                    c_matrix,
                    a_matrix: Vec::new(),
                    a_matrix_be: Vec::new(),
                    // Primary integrator is already BE — no BE fallback set
                    // (mirrors the os=1 BE branch).
                    a_neg_be: Vec::new(),
                    rhs_const_be: Vec::new(),
                    s_be: Vec::new(),
                    k_be: Vec::new(),
                    spectral_radius_s_aneg: 0.0,
                },
                a_of_s,
                a_of_s_be,
            )
        } else if os_factor > 1 {
            // Trapezoidal + oversampling: the internal-rate S was already
            // built above.
            let (a_of_s, s) =
                os_trap_s.expect("internal-rate trap S built when os>1 and trap ships");

            // Compute K = N_v * S * N_i
            let k = compute_k_from_s(&s, &kernel.n_v, &kernel.n_i, n, m);

            // Compute BE fallback matrices for adaptive per-sample fallback
            let want_be_fallback = !config.backward_euler && !config.disable_be_fallback && m > 0;
            let (s_be, k_be, a_neg_be, rhs_const_be, a_be) = if want_be_fallback {
                compute_dk_be_fallback(
                    &g_matrix,
                    &c_matrix,
                    n,
                    m,
                    n_nodes,
                    &kernel.n_v,
                    &kernel.n_i,
                    internal_rate,
                    mna,
                )?
            } else {
                (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new())
            };
            let a_of_s_be = want_be_fallback.then_some(a_be);

            // Charge form: history `alpha·C`, DC ×1.
            (
                Matrices {
                    s,
                    a_neg: charge_form_history(
                        &c_matrix,
                        n,
                        2.0 * internal_rate,
                        &topology.history_zero_rows,
                    ),
                    k,
                    n_v: kernel.n_v.clone(),
                    n_i: kernel.n_i.clone(),
                    rhs_const: rhs_const_1x(mna, n),
                    g_matrix,
                    c_matrix,
                    a_matrix: Vec::new(),
                    a_matrix_be: Vec::new(),
                    a_neg_be,
                    rhs_const_be,
                    s_be,
                    k_be,
                    spectral_radius_s_aneg: 0.0,
                },
                a_of_s,
                a_of_s_be,
            )
        } else if be {
            // Backward Euler: recompute S, A_neg, K from G/C with alpha = 1/T
            let mut a_flat = vec![0.0f64; n * n];
            let mut a_neg_flat = vec![0.0f64; n * n];
            for i in 0..n {
                for j in 0..n {
                    let g = g_matrix[i * n + j];
                    let c = c_matrix[i * n + j];
                    a_flat[i * n + j] = g + alpha * c;
                    a_neg_flat[i * n + j] = alpha * c; // BE: no -G term
                }
            }
            // #5: Blanket-zero ALL augmented algebraic rows (n_nodes..n_aug) —
            // VS/VCVS/ideal-transformer AND Boyle op-amp internal / current-mode
            // VCA / behavioral-V rows. The former per-type enumeration
            // (VS/VCVS/xfmr only) left stale trapezoidal history on the latter
            // three constraints. Routed through the shared helper, matching the
            // os>1 build_dk_be_matrices_at_rate path. Inductor branch rows
            // (n_aug..n) keep their history and are untouched.
            zero_augmented_history_rows(
                &mut a_neg_flat,
                n,
                mna.n,
                mna.n_aug,
                &mna.bjt_internal_nodes,
            );
            let s_flat = invert_flat_matrix(&a_flat, n)?;
            let a_of_s = a_flat;
            let a_of_s_be: Option<Vec<f64>> = None;
            let k_flat = if m > 0 {
                compute_k_from_s(&s_flat, &kernel.n_v, &kernel.n_i, n, m)
            } else {
                Vec::new()
            };
            // BE rhs_const: current sources x1 (not x2), VS x1
            let mut rhs_const_be = vec![0.0f64; n];
            for src in &mna.current_sources {
                crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
                crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
            }
            for vs in &mna.voltage_sources {
                let k_row = mna.n + vs.ext_idx;
                if k_row < n {
                    rhs_const_be[k_row] = vs.dc_value;
                }
            }
            // Diagnostic parity with the nodal path's post-promotion check —
            // see `log_be_post_promotion_check` doc comment.
            crate::codegen::stability::log_be_post_promotion_check(
                "DK",
                &s_flat,
                &a_neg_flat,
                n,
                &config.input_node_indices(),
            );
            (
                Matrices {
                    s: s_flat,
                    a_neg: a_neg_flat,
                    k: k_flat,
                    n_v: kernel.n_v.clone(),
                    n_i: kernel.n_i.clone(),
                    rhs_const: rhs_const_be,
                    g_matrix,
                    c_matrix,
                    a_matrix: Vec::new(),
                    a_matrix_be: Vec::new(),
                    a_neg_be: Vec::new(),
                    rhs_const_be: Vec::new(),
                    s_be: Vec::new(),
                    k_be: Vec::new(),
                    spectral_radius_s_aneg: 0.0,
                },
                a_of_s,
                a_of_s_be,
            )
        } else {
            // Standard trapezoidal: use kernel matrices directly.
            // Also compute BE fallback matrices for adaptive per-sample fallback.
            let want_be_fallback = !config.disable_be_fallback && m > 0;
            let (s_be, k_be, a_neg_be, rhs_const_be, a_be) = if want_be_fallback {
                compute_dk_be_fallback(
                    &g_matrix,
                    &c_matrix,
                    n,
                    m,
                    n_nodes,
                    &kernel.n_v,
                    &kernel.n_i,
                    internal_rate,
                    mna,
                )?
            } else {
                (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new())
            };
            let a_of_s_be = want_be_fallback.then_some(a_be);
            // The kernel inverted G + 2·fs·C of this same (patched) MNA at this
            // rate; the same expression on the stored G and C is that matrix.
            let a_of_s = build_dk_trap_a_at_rate(&g_matrix, &c_matrix, n, internal_rate);

            // Charge form: history `alpha·C`, DC ×1. The whole-system
            // `kernel.a_neg` was the discriminator's operator only.
            (
                Matrices {
                    s: kernel.s.clone(),
                    a_neg: charge_form_history(
                        &c_matrix,
                        n,
                        2.0 * internal_rate,
                        &topology.history_zero_rows,
                    ),
                    k: kernel.k.clone(),
                    n_v: kernel.n_v.clone(),
                    n_i: kernel.n_i.clone(),
                    rhs_const: rhs_const_1x(mna, n),
                    g_matrix,
                    c_matrix,
                    a_matrix: Vec::new(),
                    a_matrix_be: Vec::new(),
                    a_neg_be,
                    rhs_const_be,
                    s_be,
                    k_be,
                    spectral_radius_s_aneg: 0.0,
                },
                a_of_s,
                a_of_s_be,
            )
        };

        // BE fallback matrices are populated for nonlinear circuits (m>0) unless
        // config.disable_be_fallback is set. Linear circuits (m=0) skip BE
        // fallback since they don't have NR iteration that could diverge.

        let device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;
        let device_node_indices = Self::device_node_indices_for(&device_slots, mna);

        let pots = kernel
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
            .collect();

        let wiper_groups: Vec<WiperGroupIR> = kernel
            .wiper_groups
            .iter()
            .map(|wg| WiperGroupIR {
                cw_pot_index: wg.cw_pot_index,
                ccw_pot_index: wg.ccw_pot_index,
                total_resistance: wg.total_resistance,
                default_position: wg.default_position,
                label: wg.label.clone(),
            })
            .collect();

        let gang_groups: Vec<GangGroupIR> = kernel
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
            .collect();

        // Build switches from MNA resolved info.
        //
        // Inductor branch rows: `rebuild_matrices()` stamps a switched L
        // delta into `c_eff[aug_row][aug_row]`, mirroring the nodal path. See
        // `dk_emitter::emit_switch_methods`.
        let inductor_aug_rows: std::collections::HashMap<String, usize> = if augmented_inductors {
            let mut map = std::collections::HashMap::new();
            let mut var_idx = mna.n_aug;
            for ind in &mna.inductors {
                map.insert(ind.name.to_ascii_uppercase(), var_idx);
                var_idx += 1;
            }
            for ci in &mna.coupled_inductors {
                map.insert(ci.l1_name.to_ascii_uppercase(), var_idx);
                map.insert(ci.l2_name.to_ascii_uppercase(), var_idx + 1);
                var_idx += 2;
            }
            for group in &mna.transformer_groups {
                for (widx, name) in group.winding_names.iter().enumerate() {
                    map.insert(name.to_ascii_uppercase(), var_idx + widx);
                }
                var_idx += group.num_windings;
            }
            map
        } else {
            std::collections::HashMap::new()
        };

        let switches: Vec<SwitchIR> = mna
            .switches
            .iter()
            .enumerate()
            .map(|(idx, sw)| {
                let components = sw
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
                    .collect();
                SwitchIR {
                    index: idx,
                    label: sw.label.clone().unwrap_or_else(|| {
                        sw.components
                            .iter()
                            .map(|c| c.name.as_str())
                            .collect::<Vec<_>>()
                            .join("+")
                    }),
                    components,
                    positions: sw.positions.clone(),
                    num_positions: sw.positions.len(),
                    mutual_entries: Vec::new(),
                }
            })
            .collect();

        let has_dc_sources = kernel.rhs_const.iter().any(|&v| v != 0.0);

        let dc_config = dc_op_config(mna, config);
        // Check DC OP significance on the truncated vector (n_aug), not the full
        // n_dc vector which includes inductor branch currents.
        let dc_op_len = dc_result.v_node.len();
        let dc_op_truncated = &dc_result.v_node[..kernel.n.min(dc_op_len)];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_op_method = format!("{:?}", dc_result.method);
        let dc_op_rail_pin = dc_result.rail_pin.label();
        let dc_op_iterations = dc_result.iterations;
        // Paired with `dc_operating_point` (plain, non-IC quiescent point) —
        // see the pairing note on `dc_operating_point` below, and the
        // per-sample magnitude/NaN-reset fallback (`process_sample.rs.tera`)
        // which resets `v_prev`/`i_nl_prev` back to this exact pair. Do NOT
        // repoint this at the IC-seeded solve — that was tried and created a
        // *different* v_prev/i_nl_prev mismatch at the reset fallback (which
        // always uses the plain `dc_operating_point`), producing the same
        // failure class it was meant to fix. See `dc_nl_currents_ic_seed`
        // below for the seed actually paired with `v_prev_ic_seed`.
        let dc_nl_currents = dc_result.i_nl.clone();

        // IC=-bearing capacitors: solve a second, independent initial-state
        // operating point (each such cap temporarily replaced by an ideal
        // voltage source of its IC value) used to seed `v_prev`. `None`
        // when the netlist has no `IC=` caps — see `mna.capacitor_ics`.
        //
        // `dc_nl_currents_ic_seed` (i_nl at that same IC-consistent point)
        // is captured alongside it and used ONLY to seed `i_nl_prev` at
        // construction/reset() time (paired with `v_prev_ic_seed`, NEVER
        // with the plain `dc_operating_point`). The IC constraint can move
        // node voltages far from the quiescent bias point (that's the whole
        // purpose of IC=), so `dc_result.i_nl` — device currents evaluated
        // at the *unperturbed* operating point — would be inconsistent with
        // an IC-seeded `v_prev`. Feeding mismatched (v_prev, i_nl_prev) into
        // the first trapezoidal step is the same class of bug documented
        // below at the nodal DC_OP resize comment ("pre-clamping... creates
        // a v_prev / i_nl_prev inconsistency... that slowly drifts the
        // state... before NR blows up").
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
                v.resize(kernel.n, 0.0);
                v.truncate(kernel.n);
                v
            });
        let q_dot_ic_seed = match (&v_prev_ic_seed, &dc_nl_currents_ic_seed) {
            (Some(x), Some(i_nl)) if !solver_config.backward_euler => {
                Some(q_dot_at(&matrices, n, m, x, i_nl))
            }
            _ => None,
        };

        if !dc_result.converged && m > 0 {
            crate::diag_warn!(
                "nonlinear DC OP solver did not converge (method: {:?}), using best estimate",
                dc_result.method
            );
        }

        // Forward-active BJT detection happens BEFORE from_kernel is called.
        // The caller (detect_forward_active_bjts + CLI) handles MNA/kernel rebuild.
        // By the time we get here, kernel/mna already have the correct M dimension.

        // Which entries of S and K exist, from the circuit's structure rather
        // than the computed values (rounding noise in the inverse is set to 0).
        // The emitted K also carries the parasitic-BJT block (`state.k` holds
        // K − R_p), whose positions the NR loop must not skip.
        let structure = settle_structural_sparsity(
            &mut matrices,
            n,
            m,
            &parasitic_bjt_k_positions(&device_slots),
            &a_of_s,
            a_of_s_be.as_deref(),
        )?;
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
            k: sparsity_of_pattern(&structure.k),
            k_be: if matrices.k_be.len() == m * m && m > 0 {
                sparsity_of_pattern(&structure.k)
            } else {
                MatrixSparsity {
                    rows: m,
                    cols: m,
                    nnz: 0,
                    nz_by_row: vec![Vec::new(); m],
                }
            },
            lu: None, // DK path doesn't use full LU
            g_aug_density: 0.0,
            a: sparsity_of_pattern(&structure.a),
            noise_ratio: structure.noise_ratio,
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

        Ok(CircuitIR {
            metadata,
            topology,
            solver_mode: SolverMode::Dk,
            solver_config,
            matrices,
            // For augmented inductors, the DC OP solver returns n_aug-sized vectors
            // but the kernel dimension is n_nodal = n_aug + n_inductor_vars.
            // Pad with zeros for inductor branch currents (DC OP doesn't solve them).
            dc_operating_point: {
                // DC OP may return fewer nodes than kernel.n (e.g., when computed
                // on unexpanded MNA before internal node expansion). Pad with zeros.
                let mut dc = dc_result.v_node.clone();
                dc.resize(kernel.n, 0.0);
                dc.truncate(kernel.n);
                // Do NOT clamp op-amp output nodes to VCC/VEE supply rails
                // here — matching the nodal-path policy. Clamping v_prev
                // while `dc_nl_currents` (→ i_nl_prev) comes from the
                // UNCLAMPED solve leaves the state pair inconsistent: the
                // downstream device states encoded in i_nl no longer match
                // the clamped node voltages, and that inconsistency drifts
                // the state over thousands of samples before NR blows up
                // (observed on the nodal path as the 4kbuscomp failure:
                // ~2300 stable samples then 1e27 V explosion). A consistent
                // state pair beats prettier initial voltages; the emitted
                // per-sample rail handling plus the warmup samples cover
                // the transient from a beyond-rail DC seed.
                dc
            },
            v_prev_ic_seed,
            device_slots,
            device_node_indices,
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
            saturating_inductors: Vec::new(), // DK path: saturation routes to nodal
            pots,
            wiper_groups,
            gang_groups,
            switches,
            opamps: mna
                .opamps
                .iter()
                .filter(|oa| {
                    // Include op-amps that need any codegen-emitted post-NR
                    // processing: rail clamping (finite VCC/VEE) OR slew-rate
                    // limiting (finite SR). Pure ideal op-amps with all three
                    // infinite are skipped — the emitter produces no op-amp
                    // code for them, preserving byte-identical output for
                    // existing circuits.
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
        })
    }
}
