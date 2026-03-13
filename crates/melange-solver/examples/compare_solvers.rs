use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use melange_solver::solver::{CircuitSolver, DeviceEntry};
use melange_devices::diode::DiodeShockley;

fn main() {
    let spice = r#"* Mordor Screamer
.model D1N4148 D(IS=2.52e-9 RS=0.568 N=1.906)
C1 in n1 0.1u
R1 n1 0 1meg
R2 n1 n2 10k
C2 n2 n3 0.047u
D1 n3 n2 D1N4148
D2 n2 n3 D1N4148
R3 n3 n4 1k
C3 n4 0 0.022u
R4 n4 out 10k
C4 out 0 0.1u
Vin in 0 0
.END"#;

    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;
    
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    
    let d = DiodeShockley::new(2.52e-9, 1.906, 2.585e-2);
    let devices = vec![
        DeviceEntry::new_diode(d.clone(), 0),
        DeviceEntry::new_diode(d, 1),
    ];
    
    let mut solver = CircuitSolver::new(kernel, devices, 0, 5).unwrap();
    
    println!("CircuitSolver with 10mV DC:");
    for i in 0..20 {
        let out = solver.process_sample(0.01);
        if i % 5 == 0 {
            println!("sample {}: {:.4}V", i, out);
        }
    }
}
