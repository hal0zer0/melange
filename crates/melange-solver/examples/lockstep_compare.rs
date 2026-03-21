/// Lockstep comparison: runtime NodalSolver vs manual codegen-style NR loop.
///
/// Runs BOTH paths through the same circuit (pultec-eq.cir) on silence,
/// comparing v_prev and i_nl_prev after EACH sample to find the first divergence.
///
/// The "codegen path" is replicated manually using the IR's matrices, matching
/// the exact same algorithm that `emit_nodal_process_sample` would generate.

fn main() {
    use melange_solver::codegen::ir::CircuitIR;
    use melange_solver::codegen::CodegenConfig;
    use melange_solver::dk::DkKernel;
    use melange_solver::mna::MnaSystem;
    use melange_solver::parser::Netlist;
    use melange_solver::solver::NodalSolver;

    let sample_rate = 48000.0;
    let num_samples = 500;

    // ── Step 1: Parse circuit ──────────────────────────────────────────
    let source = std::fs::read_to_string("circuits/pultec-eq.cir")
        .expect("failed to read circuits/pultec-eq.cir");
    let netlist = Netlist::parse(&source).expect("failed to parse netlist");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");

    let input_node = *mna.node_map.get("in").expect("no 'in' node");
    let output_node = *mna.node_map.get("out").expect("no 'out' node");
    let input_resistance = 600.0;
    let input_conductance = 1.0 / input_resistance;

    // Stamp input conductance BEFORE building kernel (Thevenin source model)
    if input_node > 0 {
        mna.g[input_node - 1][input_node - 1] += input_conductance;
    }

    // Build device slots and stamp junction caps
    let device_slots = CircuitIR::build_device_info(&netlist).expect("failed to build device info");
    mna.stamp_device_junction_caps(&device_slots);

    // ── Step 2: Build runtime NodalSolver ──────────────────────────────
    let kernel =
        DkKernel::from_mna_augmented(&mna, sample_rate).expect("failed to build DK kernel");
    let mut runtime = NodalSolver::new(
        kernel.clone(),
        &mna,
        &netlist,
        device_slots.clone(),
        input_node - 1,
        output_node - 1,
    )
    .expect("Failed to create nodal solver");
    runtime.set_input_conductance(input_conductance);
    runtime.initialize_dc_op(&mna, &device_slots);

    // ── Step 3: Build codegen IR ───────────────────────────────────────
    let config = CodegenConfig {
        circuit_name: "pultec".to_string(),
        sample_rate,
        max_iterations: 50,
        tolerance: 1e-6,
        input_resistance,
        input_node: input_node - 1,
        output_nodes: vec![output_node - 1],
        oversampling_factor: 1,
        output_scales: vec![1.0],
        include_dc_op: true,
        dc_op_max_iterations: 200,
        dc_op_tolerance: 1e-9,
        dc_block: true,
        pot_settle_samples: 64,
        backward_euler: false,
    };
    let ir = CircuitIR::from_mna(&mna, &netlist, &config).expect("failed to build CircuitIR");

    let n = ir.topology.n;
    let m = ir.topology.m;
    let n_nodes = ir.topology.n_nodes;
    let n_aug = ir.topology.n_aug;

    println!("Circuit: N={n}, M={m}, n_nodes={n_nodes}, n_aug={n_aug}");
    println!("Devices: {}", ir.device_slots.len());
    for (i, slot) in ir.device_slots.iter().enumerate() {
        println!(
            "  Device {i}: {:?} start={} dim={}",
            slot.device_type, slot.start_idx, slot.dimension
        );
    }

    // ── Step 4: Initialize codegen state from RUNTIME's post-warmup state ─
    // Force bit-identical initial conditions to isolate per-sample divergence.
    // Both paths will start from the exact same v_prev and i_nl_prev.
    let mut cg_v_prev = runtime.v_prev().to_vec();
    let mut cg_i_nl_prev = runtime.i_nl_prev().to_vec();
    let mut cg_input_prev = 0.0f64; // runtime's input_prev is also 0.0 after silence warm-up

    println!("\nInitial state: FORCED IDENTICAL (copied from runtime post-warmup)");

    // ── Step 4b: Verify matrices are identical ─────────────────────────
    // The runtime stores a_neg as flat Vec<f64> (n_nodal * n_nodal)
    // Compare A_neg between runtime (solver) and IR
    println!("\n=== Matrix comparison ===");
    {
        // We can't access runtime's private a_neg directly, but we can verify
        // the IR matrices by rebuilding from G/C the same way the runtime does.
        let aug = mna.build_augmented_matrices();
        let nn = aug.n_nodal;
        let t = 1.0 / sample_rate;
        let alpha_tr = 2.0 / t;

        // Rebuild A_neg the same way the runtime does
        let mut runtime_a_neg = vec![0.0f64; nn * nn];
        let mut runtime_a_mat = vec![0.0f64; nn * nn];
        let mut g_nod = aug.g.clone();
        // Add Gmin
        for i in 0..n_nodes {
            g_nod[i][i] += 1e-12;
        }
        for i in 0..nn {
            for j in 0..nn {
                runtime_a_neg[i * nn + j] = alpha_tr * aug.c[i][j] - g_nod[i][j];
                runtime_a_mat[i * nn + j] = g_nod[i][j] + alpha_tr * aug.c[i][j];
            }
        }
        // Zero VS/VCVS rows
        for i in n_nodes..n_aug {
            for j in 0..nn {
                runtime_a_neg[i * nn + j] = 0.0;
            }
        }

        let mut max_a_neg_diff = 0.0f64;
        let mut max_a_neg_idx = (0, 0);
        for i in 0..nn {
            for j in 0..nn {
                let diff = (runtime_a_neg[i * nn + j] - ir.matrices.a_neg[i * nn + j]).abs();
                if diff > max_a_neg_diff {
                    max_a_neg_diff = diff;
                    max_a_neg_idx = (i, j);
                }
            }
        }
        println!(
            "  max|A_neg diff| = {:.6e} at ({},{})",
            max_a_neg_diff, max_a_neg_idx.0, max_a_neg_idx.1
        );

        let mut max_a_diff = 0.0f64;
        for i in 0..nn {
            for j in 0..nn {
                let diff = (runtime_a_mat[i * nn + j] - ir.matrices.a_matrix[i * nn + j]).abs();
                if diff > max_a_diff {
                    max_a_diff = diff;
                }
            }
        }
        println!("  max|A diff|     = {:.6e}", max_a_diff);

        // Compare N_v, N_i
        let mut max_nv_diff = 0.0f64;
        let mut n_v_rebuilt = vec![0.0f64; m * nn];
        for i in 0..m {
            for j in 0..n_aug {
                n_v_rebuilt[i * nn + j] = mna.n_v[i][j];
            }
        }
        for i in 0..m * nn {
            let diff = (n_v_rebuilt[i] - ir.matrices.n_v[i]).abs();
            if diff > max_nv_diff {
                max_nv_diff = diff;
            }
        }
        println!("  max|N_v diff|   = {:.6e}", max_nv_diff);

        let mut max_ni_diff = 0.0f64;
        let mut n_i_rebuilt = vec![0.0f64; nn * m];
        for i in 0..n_aug {
            for j in 0..m {
                n_i_rebuilt[i * m + j] = mna.n_i[i][j];
            }
        }
        for i in 0..nn * m {
            let diff = (n_i_rebuilt[i] - ir.matrices.n_i[i]).abs();
            if diff > max_ni_diff {
                max_ni_diff = diff;
            }
        }
        println!("  max|N_i diff|   = {:.6e}", max_ni_diff);

        // rhs_const: can't rebuild (build_rhs_const is pub(crate)), but IR value is used directly
        println!(
            "  rhs_const: using IR value directly (can't access build_rhs_const from example)"
        );
    }

    // ── Step 6: Lockstep comparison ────────────────────────────────────
    println!(
        "\n=== Lockstep comparison: {} samples of silence ===",
        num_samples
    );

    let mut first_diverge_sample = None;
    let mut max_ever_v_diff = 0.0f64;
    let mut max_ever_i_diff = 0.0f64;
    let mut prev_max_v = 0.0f64;

    for sample in 0..num_samples {
        // Run one sample through runtime
        let _runtime_out = runtime.process_sample(0.0);

        // Run one sample through codegen-style path
        let _codegen_out = codegen_process_sample(
            0.0,
            &ir,
            &mut cg_v_prev,
            &mut cg_i_nl_prev,
            &mut cg_input_prev,
            n,
            m,
            n_nodes,
            n_aug,
            input_conductance,
        );

        // Compare v_prev
        let mut sample_max_v_diff = 0.0f64;
        let mut sample_max_v_idx = 0;
        for i in 0..n {
            let diff = (runtime.v_prev()[i] - cg_v_prev[i]).abs();
            if diff > sample_max_v_diff {
                sample_max_v_diff = diff;
                sample_max_v_idx = i;
            }
        }

        // Compare i_nl_prev
        let mut sample_max_i_diff = 0.0f64;
        let mut sample_max_i_idx = 0;
        for i in 0..m {
            let diff = (runtime.i_nl_prev()[i] - cg_i_nl_prev[i]).abs();
            if diff > sample_max_i_diff {
                sample_max_i_diff = diff;
                sample_max_i_idx = i;
            }
        }

        if sample_max_v_diff > max_ever_v_diff {
            max_ever_v_diff = sample_max_v_diff;
        }
        if sample_max_i_diff > max_ever_i_diff {
            max_ever_i_diff = sample_max_i_diff;
        }

        // Report first divergence above threshold
        if first_diverge_sample.is_none() && sample_max_v_diff > 1e-10 {
            first_diverge_sample = Some(sample);
            println!("\n*** FIRST DIVERGENCE at sample {} ***", sample);
            println!(
                "  max|v_diff| = {:.6e} at node {}",
                sample_max_v_diff, sample_max_v_idx
            );
            println!(
                "  max|i_diff| = {:.6e} at dim {}",
                sample_max_i_diff, sample_max_i_idx
            );

            // Detailed node-by-node comparison
            println!("\n  Top 10 divergent nodes:");
            let mut diffs: Vec<(usize, f64, f64, f64)> = (0..n)
                .map(|i| {
                    (
                        i,
                        (runtime.v_prev()[i] - cg_v_prev[i]).abs(),
                        runtime.v_prev()[i],
                        cg_v_prev[i],
                    )
                })
                .collect();
            diffs.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
            for &(idx, diff, rt, cg) in diffs.iter().take(10) {
                println!(
                    "    node {:3}: diff={:+.6e}  runtime={:+.10e}  codegen={:+.10e}",
                    idx, diff, rt, cg
                );
            }

            if m > 0 {
                println!("\n  Nonlinear currents:");
                for i in 0..m {
                    let diff = (runtime.i_nl_prev()[i] - cg_i_nl_prev[i]).abs();
                    if diff > 1e-15 {
                        println!(
                            "    i_nl[{:2}]: diff={:+.6e}  runtime={:+.10e}  codegen={:+.10e}",
                            i,
                            diff,
                            runtime.i_nl_prev()[i],
                            cg_i_nl_prev[i]
                        );
                    }
                }
            }
        }

        // Report growth rate every 50 samples
        let growth_rate = if prev_max_v > 1e-30 {
            sample_max_v_diff / prev_max_v
        } else {
            0.0
        };

        if sample % 50 == 0 || sample < 10 {
            println!(
                "  sample {:4}: max|v_diff|={:.6e}  max|i_diff|={:.6e}  growth={:.4}x",
                sample, sample_max_v_diff, sample_max_i_diff, growth_rate
            );
        }

        prev_max_v = sample_max_v_diff;
    }

    // ── Step 7: Summary ────────────────────────────────────────────────
    println!("\n=== Summary ===");
    println!("  Samples processed: {}", num_samples);
    println!("  Max ever |v_diff|: {:.6e}", max_ever_v_diff);
    println!("  Max ever |i_diff|: {:.6e}", max_ever_i_diff);
    match first_diverge_sample {
        Some(s) => println!("  First divergence > 1e-10: sample {}", s),
        None => println!("  No divergence > 1e-10 detected"),
    }
    println!(
        "  Runtime BE fallbacks: {}",
        runtime.diag_be_fallback_count()
    );
    println!(
        "  Runtime NR max-iter: {}",
        runtime.diag_nr_max_iter_count()
    );
}

/// Replicate the codegen's process_sample algorithm using IR matrices.
///
/// This matches the exact algorithm emitted by `emit_nodal_process_sample`:
/// 1. Build RHS from rhs_const + A_neg * v_prev + N_i * i_nl_prev + input
/// 2. NR loop: extract v_nl, evaluate devices, build Jacobian, LU solve, limit, converge
/// 3. BE fallback if trapezoidal NR fails
/// 4. Update state
///
/// Returns the raw output voltage (before DC blocking).
fn codegen_process_sample(
    input: f64,
    ir: &melange_solver::codegen::ir::CircuitIR,
    v_prev: &mut Vec<f64>,
    i_nl_prev: &mut Vec<f64>,
    input_prev: &mut f64,
    n: usize,
    m: usize,
    n_nodes: usize,
    _n_aug: usize,
    input_conductance: f64,
) -> f64 {
    let input = if input.is_finite() {
        input.clamp(-100.0, 100.0)
    } else {
        0.0
    };

    let a_flat = &ir.matrices.a_matrix;
    let a_neg_flat = &ir.matrices.a_neg;
    let rhs_const = &ir.matrices.rhs_const;
    let n_v_flat = &ir.matrices.n_v;
    let n_i_flat = &ir.matrices.n_i;
    let a_be_flat = &ir.matrices.a_matrix_be;
    let a_neg_be_flat = &ir.matrices.a_neg_be;
    let rhs_const_be = &ir.matrices.rhs_const_be;

    let max_iter = ir.solver_config.max_iterations;
    let tol = ir.solver_config.tolerance;
    let input_node = ir.solver_config.input_node;

    // Step 1: Build RHS = rhs_const + A_neg * v_prev + N_i * i_nl_prev
    let mut rhs = vec![0.0f64; n];
    for i in 0..n {
        let mut sum = rhs_const[i];
        for j in 0..n {
            sum += a_neg_flat[i * n + j] * v_prev[j];
        }
        for j in 0..m {
            sum += n_i_flat[i * m + j] * i_nl_prev[j];
        }
        rhs[i] = sum;
    }

    // Input source (Thevenin, trapezoidal)
    rhs[input_node] += (input + *input_prev) * input_conductance;
    *input_prev = input;

    // Step 2: Newton-Raphson
    let mut v = v_prev.clone();
    let mut converged = false;
    let mut i_nl = vec![0.0f64; m];
    let mut j_dev = vec![0.0f64; m * m];
    let mut _nr_iters_used = 0u32;

    for _iter in 0..max_iter {
        // 2a. Extract nonlinear voltages: v_nl = N_v * v
        let mut v_nl = vec![0.0f64; m];
        for i in 0..m {
            let mut sum = 0.0;
            for j in 0..n {
                sum += n_v_flat[i * n + j] * v[j];
            }
            v_nl[i] = sum;
        }

        // 2b. Evaluate device currents and Jacobian
        evaluate_devices_codegen(&v_nl, &ir.device_slots, &mut i_nl, &mut j_dev, m);

        // 2c. Build Jacobian: G_aug = A - N_i * J_dev * N_v
        let mut g_aug = vec![vec![0.0f64; n]; n];
        for i in 0..n {
            for j in 0..n {
                g_aug[i][j] = a_flat[i * n + j];
            }
        }
        // Sparse stamping (matching codegen's block-diagonal structure)
        for slot in &ir.device_slots {
            let s = slot.start_idx;
            let dim = slot.dimension;
            for di in 0..dim {
                let ii = s + di;
                for dj in 0..dim {
                    let jj = s + dj;
                    let jd = j_dev[ii * m + jj];
                    if jd.abs() < 1e-30 {
                        continue;
                    }
                    for a in 0..n {
                        let ni_ai = n_i_flat[a * m + ii];
                        if ni_ai.abs() < 1e-30 {
                            continue;
                        }
                        for b in 0..n {
                            let nv_jb = n_v_flat[jj * n + b];
                            if nv_jb.abs() < 1e-30 {
                                continue;
                            }
                            g_aug[a][b] -= ni_ai * jd * nv_jb;
                        }
                    }
                }
            }
        }

        // 2d. Build companion RHS: rhs + N_i * (i_nl - J_dev * v_nl)
        let mut rhs_work = rhs.clone();
        for slot in &ir.device_slots {
            let s = slot.start_idx;
            let dim = slot.dimension;
            for di in 0..dim {
                let ii = s + di;
                let mut jdv = 0.0;
                for dj in 0..dim {
                    let jj = s + dj;
                    jdv += j_dev[ii * m + jj] * v_nl[jj];
                }
                let i_comp = i_nl[ii] - jdv;
                for a in 0..n {
                    rhs_work[a] += n_i_flat[a * m + ii] * i_comp;
                }
            }
        }

        // 2e. LU solve (equilibrated, matching codegen's lu_solve)
        let v_new = match solve_equilibrated_codegen(&g_aug, &rhs_work) {
            Some(v) => v,
            None => break,
        };

        // 2f. SPICE voltage limiting + node damping
        let mut alpha = 1.0f64;

        // Layer 1: Device voltage limiting
        for slot in &ir.device_slots {
            let s = slot.start_idx;
            for d in 0..slot.dimension {
                let i = s + d;
                let mut v_nl_proposed = 0.0;
                for j in 0..n {
                    v_nl_proposed += n_v_flat[i * n + j] * v_new[j];
                }
                let v_nl_current = v_nl[i];
                let dv: f64 = v_nl_proposed - v_nl_current;
                if dv.abs() > 1e-15 {
                    let v_limited =
                        limit_slot_voltage_codegen(slot, d, v_nl_proposed, v_nl_current);
                    let dv_limited = v_limited - v_nl_current;
                    let ratio = (dv_limited / dv).clamp(0.01, 1.0);
                    if ratio < alpha {
                        alpha = ratio;
                    }
                }
            }
        }

        // Layer 2: Global node voltage damping (10V threshold)
        {
            let mut max_node_dv = 0.0f64;
            for i in 0..n_nodes {
                let dv = alpha * (v_new[i] - v[i]);
                max_node_dv = max_node_dv.max(dv.abs());
            }
            if max_node_dv > 10.0 {
                alpha *= (10.0 / max_node_dv).max(0.01);
            }
        }

        // Codegen convergence: compute step, check, then apply (all in one loop)
        // This matches the codegen's generated code exactly:
        //   let step = alpha * (v_new[i] - v[i]);
        //   let threshold = 1e-6 * v[i].abs().max((v[i] + step).abs()) + TOL;
        //   if step.abs() >= threshold { max_step_exceeded = true; }
        //   v[i] += step;
        let mut max_step_exceeded = false;
        for i in 0..n {
            let step = alpha * (v_new[i] - v[i]);
            let threshold = 1e-6 * v[i].abs().max((v[i] + step).abs()) + tol;
            if step.abs() >= threshold {
                max_step_exceeded = true;
            }
            v[i] += step;
        }
        let converged_check = !max_step_exceeded;

        if converged_check {
            converged = true;
            _nr_iters_used = _iter as u32;
            // Final device evaluation at converged point
            let mut v_nl_final = vec![0.0f64; m];
            for i in 0..m {
                let mut sum = 0.0;
                for j in 0..n {
                    sum += n_v_flat[i * n + j] * v[j];
                }
                v_nl_final[i] = sum;
            }
            evaluate_devices_codegen_final(&v_nl_final, &ir.device_slots, &mut i_nl, m);
            break;
        }
    }

    // Backward Euler fallback
    if !converged {
        v.copy_from_slice(v_prev);

        // Rebuild RHS with BE matrices
        let mut rhs_be = vec![0.0f64; n];
        for i in 0..n {
            let mut sum = if !rhs_const_be.is_empty() {
                rhs_const_be[i]
            } else {
                0.0
            };
            for j in 0..n {
                sum += a_neg_be_flat[i * n + j] * v_prev[j];
            }
            for j in 0..m {
                sum += n_i_flat[i * m + j] * i_nl_prev[j];
            }
            rhs_be[i] = sum;
        }
        rhs_be[input_node] += input * input_conductance;

        for _iter in 0..max_iter {
            let mut v_nl = vec![0.0f64; m];
            for i in 0..m {
                let mut sum = 0.0;
                for j in 0..n {
                    sum += n_v_flat[i * n + j] * v[j];
                }
                v_nl[i] = sum;
            }

            evaluate_devices_codegen(&v_nl, &ir.device_slots, &mut i_nl, &mut j_dev, m);

            let mut g_aug = vec![vec![0.0f64; n]; n];
            for i in 0..n {
                for j in 0..n {
                    g_aug[i][j] = a_be_flat[i * n + j];
                }
            }
            for slot in &ir.device_slots {
                let s = slot.start_idx;
                let dim = slot.dimension;
                for di in 0..dim {
                    let ii = s + di;
                    for dj in 0..dim {
                        let jj = s + dj;
                        let jd = j_dev[ii * m + jj];
                        if jd.abs() < 1e-30 {
                            continue;
                        }
                        for a in 0..n {
                            let ni_ai = n_i_flat[a * m + ii];
                            if ni_ai.abs() < 1e-30 {
                                continue;
                            }
                            for b in 0..n {
                                let nv_jb = n_v_flat[jj * n + b];
                                if nv_jb.abs() < 1e-30 {
                                    continue;
                                }
                                g_aug[a][b] -= ni_ai * jd * nv_jb;
                            }
                        }
                    }
                }
            }

            let mut rhs_work = rhs_be.clone();
            for slot in &ir.device_slots {
                let s = slot.start_idx;
                let dim = slot.dimension;
                for di in 0..dim {
                    let ii = s + di;
                    let mut jdv = 0.0;
                    for dj in 0..dim {
                        let jj = s + dj;
                        jdv += j_dev[ii * m + jj] * v_nl[jj];
                    }
                    let i_comp = i_nl[ii] - jdv;
                    for a in 0..n {
                        rhs_work[a] += n_i_flat[a * m + ii] * i_comp;
                    }
                }
            }

            let v_new = match solve_equilibrated_codegen(&g_aug, &rhs_work) {
                Some(v) => v,
                None => break,
            };

            let mut alpha = 1.0f64;
            for slot in &ir.device_slots {
                let s = slot.start_idx;
                for d in 0..slot.dimension {
                    let i = s + d;
                    let mut v_nl_proposed = 0.0;
                    for j in 0..n {
                        v_nl_proposed += n_v_flat[i * n + j] * v_new[j];
                    }
                    let v_nl_current = v_nl[i];
                    let dv: f64 = v_nl_proposed - v_nl_current;
                    if dv.abs() > 1e-15 {
                        let v_limited =
                            limit_slot_voltage_codegen(slot, d, v_nl_proposed, v_nl_current);
                        let dv_limited = v_limited - v_nl_current;
                        let ratio = (dv_limited / dv).clamp(0.01, 1.0);
                        if ratio < alpha {
                            alpha = ratio;
                        }
                    }
                }
            }
            {
                let mut max_node_dv = 0.0f64;
                for i in 0..n_nodes {
                    let dv = alpha * (v_new[i] - v[i]);
                    max_node_dv = max_node_dv.max(dv.abs());
                }
                if max_node_dv > 10.0 {
                    alpha *= (10.0 / max_node_dv).max(0.01);
                }
            }

            let mut be_step_exceeded = false;
            for i in 0..n {
                let step = alpha * (v_new[i] - v[i]);
                let threshold = 1e-6 * v[i].abs().max((v[i] + step).abs()) + tol;
                if step.abs() >= threshold {
                    be_step_exceeded = true;
                }
                v[i] += step;
            }
            let be_converged = !be_step_exceeded;

            if be_converged {
                converged = true;
                let mut v_nl_final = vec![0.0f64; m];
                for i in 0..m {
                    let mut sum = 0.0;
                    for j in 0..n {
                        sum += n_v_flat[i * n + j] * v[j];
                    }
                    v_nl_final[i] = sum;
                }
                evaluate_devices_codegen_final(&v_nl_final, &ir.device_slots, &mut i_nl, m);
                break;
            }
        }

        // If still not converged, ensure i_nl is consistent
        if !converged {
            let mut v_nl_final = vec![0.0f64; m];
            for i in 0..m {
                let mut sum = 0.0;
                for j in 0..n {
                    sum += n_v_flat[i * n + j] * v[j];
                }
                v_nl_final[i] = sum;
            }
            evaluate_devices_codegen_final(&v_nl_final, &ir.device_slots, &mut i_nl, m);
        }
    }

    // Step 3: Update state
    v_prev.copy_from_slice(&v);
    i_nl_prev.copy_from_slice(&i_nl);

    // NaN check
    if !v_prev.iter().all(|x| x.is_finite()) {
        v_prev.fill(0.0);
        i_nl_prev.fill(0.0);
        *input_prev = 0.0;
    }

    // Return raw output (node voltage, before DC blocking)
    let out_node = ir.solver_config.output_nodes[0];
    if out_node < n {
        v_prev[out_node]
    } else {
        0.0
    }
}

/// Device evaluation matching the codegen's inline code (current + Jacobian).
///
/// Uses the same melange-devices functions that the runtime dc_op::evaluate_devices uses.
fn evaluate_devices_codegen(
    v_nl: &[f64],
    device_slots: &[melange_solver::codegen::ir::DeviceSlot],
    i_nl: &mut [f64],
    j_dev: &mut [f64],
    m: usize,
) {
    for x in i_nl.iter_mut() {
        *x = 0.0;
    }
    for x in j_dev.iter_mut() {
        *x = 0.0;
    }

    // Call through to the same evaluate_devices used by the runtime
    melange_solver::dc_op::evaluate_devices(v_nl, device_slots, i_nl, j_dev, m);
}

/// Device evaluation at converged point (current only, no Jacobian needed).
///
/// In the codegen, the "final" evaluation only writes i_nl (Jacobian is unused).
/// We reuse evaluate_devices since it also computes i_nl correctly.
fn evaluate_devices_codegen_final(
    v_nl: &[f64],
    device_slots: &[melange_solver::codegen::ir::DeviceSlot],
    i_nl: &mut [f64],
    m: usize,
) {
    let mut j_dev_dummy = vec![0.0f64; m * m];
    melange_solver::dc_op::evaluate_devices(v_nl, device_slots, i_nl, &mut j_dev_dummy, m);
}

/// Equilibrated LU solve matching the codegen's lu_solve function exactly.
///
/// This is a direct Rust translation of `solve_equilibrated` from dk.rs,
/// which is the same algorithm emitted by `emit_nodal_lu_solve`.
fn solve_equilibrated_codegen(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    melange_solver::dk::solve_equilibrated(a, b)
}

/// SPICE voltage limiting matching the runtime's limit_slot_voltage.
fn limit_slot_voltage_codegen(
    slot: &melange_solver::codegen::ir::DeviceSlot,
    dim: usize,
    vnew: f64,
    vold: f64,
) -> f64 {
    use melange_solver::codegen::ir::DeviceParams;

    match &slot.params {
        DeviceParams::Diode(dp) => {
            let vcrit = pn_vcrit(dp.n_vt, dp.is);
            pnjlim(vnew, vold, dp.n_vt, vcrit)
        }
        DeviceParams::Bjt(bp) => {
            let vcrit = pn_vcrit(bp.vt, bp.is);
            pnjlim(vnew, vold, bp.vt, vcrit)
        }
        DeviceParams::Jfet(jp) => {
            if dim == 0 {
                fetlim(vnew, vold, 0.0)
            } else {
                fetlim(vnew, vold, jp.vp)
            }
        }
        DeviceParams::Mosfet(mp) => {
            if dim == 0 {
                fetlim(vnew, vold, 0.0)
            } else {
                fetlim(vnew, vold, mp.vt)
            }
        }
        DeviceParams::Tube(tp) => {
            if dim == 0 {
                let n_vt = tp.vgk_onset / 3.0;
                let vcrit = pn_vcrit(n_vt, 1e-10);
                pnjlim(vnew, vold, n_vt, vcrit)
            } else {
                fetlim(vnew, vold, 0.0)
            }
        }
    }
}

/// SPICE pnjlim (PN junction voltage limiting).
fn pnjlim(vnew: f64, vold: f64, n_vt: f64, vcrit: f64) -> f64 {
    if vnew > vcrit && (vnew - vold).abs() > 2.0 * n_vt {
        if vold > 0.0 {
            let arg = 1.0 + (vnew - vold) / n_vt;
            if arg > 0.0 {
                vold + n_vt * arg.ln()
            } else {
                vcrit
            }
        } else {
            vcrit
        }
    } else {
        vnew
    }
}

/// SPICE fetlim (FET voltage limiting).
fn fetlim(vnew: f64, vold: f64, vto: f64) -> f64 {
    let vtstep = (vold - vto).abs();
    let vtstep = vtstep.max(0.5); // minimum step size 0.5V
    if vnew > vold + vtstep {
        vold + vtstep
    } else if vnew < vold - vtstep {
        vold - vtstep
    } else {
        vnew
    }
}

/// SPICE pn_vcrit (critical voltage for PN junction limiting).
fn pn_vcrit(n_vt: f64, is: f64) -> f64 {
    n_vt * (n_vt / (std::f64::consts::SQRT_2 * is)).ln()
}
