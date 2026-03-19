use melange_devices::diode::DiodeShockley;
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::solver::{CircuitSolver, DeviceEntry};

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
    println!("N={}, M={}", kernel.n, kernel.m);

    let d1 = DiodeShockley::new(2.52e-9, 1.906, 2.585e-2);
    let d2 = DiodeShockley::new(2.52e-9, 1.906, 2.585e-2);

    let devices = vec![DeviceEntry::new_diode(d1, 0), DeviceEntry::new_diode(d2, 1)];

    let mut solver = CircuitSolver::new(kernel, devices, 0, 5).unwrap();

    println!("Testing with 50mV sine:");
    for i in 0..100 {
        let t = i as f64 / 48000.0;
        let input = (2.0 * 3.14159 * 440.0 * t).sin() * 0.05;
        let output = solver.process_sample(input);

        if i % 10 == 0 {
            println!("sample {}: in={:.3}V, out={:.3}V", i, input, output);
        }

        if !output.is_finite() || output.abs() > 100.0 {
            println!("EXPLODED at sample {}: {}", i, output);
            return;
        }
    }
    println!("SUCCESS - circuit stable!");
}
