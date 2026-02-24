use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use melange_solver::solver::{CircuitSolver, DeviceEntry};
use melange_devices::diode::DiodeShockley;

fn main() {
    let spice = r#"* Antiparallel Diode Test
.model D1N4148 D(IS=2.52e-9)
R1 in n1 10k
D1 n1 0 D1N4148
D2 0 n1 D1N4148
Vin in 0 0
.END"#;

    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    
    // Add input conductance
    mna.g[0][0] += 1.0;
    
    let sample_rate = 48000.0;
    let kernel = DkKernel::from_mna(&mna, sample_rate).unwrap();
    
    println!("Antiparallel diodes: N={}, M={}", kernel.n, kernel.m);
    
    println!("K matrix:");
    for i in 0..kernel.m {
        print!("  ");
        for j in 0..kernel.m {
            print!("{:12.4} ", kernel.k[i * kernel.m + j]);
        }
        println!();
    }
    
    // Test with CircuitSolver
    let diode1 = DiodeShockley::new_room_temp(2.52e-9, 1.0);
    let diode2 = DiodeShockley::new_room_temp(2.52e-9, 1.0);
    let devices = vec![
        DeviceEntry::new_diode(diode1, 0),
        DeviceEntry::new_diode(diode2, 1),
    ];
    
    let mut solver = CircuitSolver::new(kernel, devices, 0, 0);
    
    println!("\nTesting with CircuitSolver:");
    for amp in [0.01_f64, 0.1, 1.0, 5.0].iter() {
        solver.reset();
        let mut max_out = 0.0_f64;
        for i in 0..100 {
            let output = solver.process_sample(*amp);
            max_out = max_out.max(output.abs());
            if i < 3 {
                println!("  amp={}V, sample {}: out={:.6e}", amp, i, output);
            }
        }
        println!("  amp={}V, max_out={:.3}V (finite: {})", amp, max_out, max_out.is_finite());
    }
}
