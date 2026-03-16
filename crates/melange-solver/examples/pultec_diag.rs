use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use melange_solver::solver::NodalSolver;
use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::CodegenConfig;

fn main() {
    let src = std::fs::read_to_string("circuits/pultec-eq.cir").unwrap();
    let netlist = Netlist::parse(&src).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Stamp input conductance
    let g_in = 1.0 / 600.0;
    let in_node = *mna.node_map.get("in").unwrap();
    let out_node = *mna.node_map.get("out").unwrap();
    if in_node > 0 {
        mna.g[in_node - 1][in_node - 1] += g_in;
    }

    let sample_rate = 48000.0;
    let kernel = DkKernel::from_mna(&mna, sample_rate).unwrap();

    // Print companion model condition number (old approach)
    let n_aug = kernel.n;
    println!("N_aug={}, N_nodes={}, M={}", n_aug, kernel.n_nodes, kernel.m);
    println!("Inductors: {} uncoupled, {} coupled pairs, {} transformer groups",
        mna.inductors.len(), mna.coupled_inductors.len(), mna.transformer_groups.len());

    let n_inductor_vars: usize = mna.inductors.len()
        + mna.coupled_inductors.len() * 2
        + mna.transformer_groups.iter().map(|g| g.num_windings).sum::<usize>();
    println!("N_nodal={} (N_aug + {} inductor variables)", n_aug + n_inductor_vars, n_inductor_vars);

    println!("\ncond(A_companion) ~ {:.2e} (old companion model)", {
        let a = mna.get_a_matrix(sample_rate);
        let norm_a: f64 = a.iter().map(|row| row.iter().map(|x| x.abs()).sum::<f64>()).fold(0.0_f64, f64::max);
        let n = kernel.n;
        let mut norm_s = 0.0_f64;
        for i in 0..n {
            let row_sum: f64 = (0..n).map(|j| kernel.s[i * n + j].abs()).sum();
            norm_s = norm_s.max(row_sum);
        }
        norm_a * norm_s
    });

    // Build device slots for NodalSolver
    let config = CodegenConfig {
        circuit_name: "pultec_diag".to_string(),
        sample_rate,
        input_node: in_node - 1,
        output_nodes: vec![out_node - 1],
        input_resistance: 600.0,
        ..CodegenConfig::default()
    };
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();

    // Create NodalSolver (uses augmented MNA for inductors)
    let mut solver = NodalSolver::new(
        kernel, &mna, &netlist, ir.device_slots.clone(),
        in_node - 1, out_node - 1,
    );
    solver.input_conductance = g_in;

    // Initialize DC OP
    println!("\nInitializing DC operating point...");
    solver.initialize_dc_op(&mna, &ir.device_slots);

    // Print DC voltages at key nodes
    let node_map = &mna.node_map;
    for name in &["vcc", "v250", "plate1", "cathode", "bias_31v", "cath2", "out"] {
        if let Some(&idx) = node_map.get(*name) {
            if idx > 0 && idx <= solver.v_prev.len() {
                println!("  V({}) = {:.2}V", name, solver.v_prev[idx - 1]);
            }
        }
    }

    // Process samples (reduced from 4800 for faster diagnostic)
    println!("\nProcessing 480 samples (10ms at 48kHz)...");
    let num_samples = 480;
    let mut peak_out = 0.0_f64;
    for i in 0..num_samples {
        let t = i as f64 / sample_rate;
        let input = 0.05 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let out = solver.process_sample(input);
        peak_out = peak_out.max(out.abs());
    }

    println!("Peak output: {:.4}V", peak_out);
    println!("NR max iterations: {} / {} samples", solver.diag_nr_max_iter_count, num_samples);
    println!("NaN resets: {}", solver.diag_nan_reset_count);
    println!("Clamp count: {}", solver.diag_clamp_count);
}
