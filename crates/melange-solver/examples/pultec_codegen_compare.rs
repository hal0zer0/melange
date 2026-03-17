/// Dump v_prev and i_nl_prev at sample 1 from runtime
fn main() {
    use melange_solver::parser::Netlist;
    use melange_solver::mna::MnaSystem;
    use melange_solver::codegen::ir::CircuitIR;

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
        kernel, &mna, &netlist, device_slots.clone(), input_node - 1, output_node - 1,
    );
    solver.input_conductance = input_conductance;
    solver.initialize_dc_op(&mna, &device_slots);

    // Run 1 sample, dump state
    solver.process_sample(0.0);

    println!("RUNTIME v_prev[0..10]:");
    for i in 0..10 { println!("  v[{}] = {:+.15e}", i, solver.v_prev[i]); }
    println!("RUNTIME i_nl_prev:");
    for i in 0..solver.i_nl_prev.len() { println!("  i[{}] = {:+.15e}", i, solver.i_nl_prev[i]); }
}
