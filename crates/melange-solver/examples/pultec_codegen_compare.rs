/// Run runtime NodalSolver for 48000 samples, check for oscillation
fn main() {
    use melange_solver::codegen::ir::CircuitIR;
    use melange_solver::mna::MnaSystem;
    use melange_solver::parser::Netlist;

    let source = std::fs::read_to_string("circuits/pultec-eq.cir").unwrap();
    let netlist = Netlist::parse(&source).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = *mna.node_map.get("in").unwrap();
    let output_node = *mna.node_map.get("out").unwrap();
    let input_conductance = 1.0 / 600.0;
    if input_node > 0 {
        mna.g[input_node - 1][input_node - 1] += input_conductance;
    }

    let device_slots = CircuitIR::build_device_info(&netlist).unwrap();
    mna.stamp_device_junction_caps(&device_slots);

    let kernel = melange_solver::dk::DkKernel::from_mna_augmented(&mna, 48000.0).unwrap();
    let mut solver = melange_solver::solver::NodalSolver::new(
        kernel,
        &mna,
        &netlist,
        device_slots.clone(),
        input_node - 1,
        output_node - 1,
    );
    solver.input_conductance = input_conductance;
    solver.initialize_dc_op(&mna, &device_slots);

    let mut peak = 0.0f64;
    for batch in 0..10 {
        let mut batch_peak = 0.0f64;
        for _ in 0..4800 {
            let out = solver.process_sample(0.0);
            batch_peak = batch_peak.max(out.abs());
        }
        peak = peak.max(batch_peak);
        println!(
            "{:.1}s: peak={:.6e}  nr={}  be={}",
            (batch + 1) as f64 * 0.1,
            batch_peak,
            solver.diag_nr_max_iter_count,
            solver.diag_be_fallback_count
        );
    }
    println!("\nOverall peak: {:.6e}", peak);
}
