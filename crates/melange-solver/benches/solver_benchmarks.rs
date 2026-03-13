//! Criterion benchmark suite for melange-solver hot paths.
//!
//! Measures performance of:
//! - process_sample throughput (samples/second)
//! - NR solver convergence (iterations, time per iteration)
//! - Matrix operations (S * rhs, K * i_nl)
//! - Device evaluation (diode/BJT current and jacobian)

use criterion::{
    black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput,
};
use melange_devices::{BjtEbersMoll, DiodeShockley};
use melange_solver::{
    parser::Netlist, mna::MnaSystem, dk::DkKernel, solver::{CircuitSolver, DeviceEntry, LinearSolver},
};
use melange_primitives::nr::{nr_solve_1d, nr_solve_2d, pn_vcrit, pnjlim};

// =============================================================================
// Circuit Netlists
// =============================================================================

const RC_LOWPASS_SPICE: &str = r#"RC Lowpass
R1 in out 1k
C1 out 0 1u
"#;

const DIODE_CLIPPER_SPICE: &str = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;

const BJT_AMP_SPICE: &str = r#"Common Emitter
Q1 coll base emit 2N2222
Rc coll vcc 10k
R1 base 0 100k
Re emit 0 1k
Rbias vcc 0 10k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;

// =============================================================================
// Benchmark 1: process_sample Throughput
// =============================================================================

fn benchmark_rc_lowpass(c: &mut Criterion) {
    let netlist = Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    let mut solver = LinearSolver::new(kernel, 0, 1);

    let mut group = c.benchmark_group("process_sample/rc_lowpass");
    group.throughput(Throughput::Elements(1));
    group.measurement_time(std::time::Duration::from_secs(5));

    group.bench_function("single_sample", |b| {
        b.iter(|| solver.process_sample(black_box(0.5)))
    });

    // Benchmark block processing
    let input_block: Vec<f64> = (0..64).map(|i| (i as f64 / 64.0).sin()).collect();
    group.bench_function("block_64", |b| {
        let mut output = vec![0.0; 64];
        b.iter(|| {
            for i in 0..64 {
                output[i] = solver.process_sample(black_box(input_block[i]));
            }
        })
    });

    group.finish();
}

fn benchmark_diode_clipper(c: &mut Criterion) {
    let netlist = Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    let diode = DiodeShockley::silicon();
    let devices = vec![DeviceEntry::new_diode(diode, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();

    let mut group = c.benchmark_group("process_sample/diode_clipper");
    group.throughput(Throughput::Elements(1));
    group.measurement_time(std::time::Duration::from_secs(5));

    // Test with different input levels
    for &input_level in &[0.01, 0.1, 0.5, 1.0] {
        group.bench_with_input(
            BenchmarkId::new("single_sample", format!("input_{:.2}", input_level)),
            &input_level,
            |b, &level| {
                solver.reset();
                b.iter(|| solver.process_sample(black_box(level)))
            },
        );
    }

    group.finish();
}

fn benchmark_bjt_amp(c: &mut Criterion) {
    let netlist = Netlist::parse(BJT_AMP_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    // Skip if kernel dimensions don't match BJT (2D)
    if kernel.m != 2 {
        println!("Skipping BJT benchmark: kernel.m = {}, expected 2", kernel.m);
        return;
    }

    let bjt = BjtEbersMoll::npn_2n2222a();
    let devices = vec![DeviceEntry::new_bjt(bjt, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, 0, 0).unwrap();

    let mut group = c.benchmark_group("process_sample/bjt_amp");
    group.throughput(Throughput::Elements(1));
    group.measurement_time(std::time::Duration::from_secs(5));

    // Test with DC bias + small AC signal
    for &input_level in &[0.0, 0.1, 0.5, 1.0] {
        group.bench_with_input(
            BenchmarkId::new("single_sample", format!("input_{:.2}", input_level)),
            &input_level,
            |b, &level| {
                solver.reset();
                b.iter(|| solver.process_sample(black_box(level)))
            },
        );
    }

    group.finish();
}

// =============================================================================
// Benchmark 2: NR Solver Convergence
// =============================================================================

fn benchmark_nr_1d_convergence(c: &mut Criterion) {
    let diode = DiodeShockley::silicon();
    let k = 1000.0; // Typical K value for diode clipper
    let p = 0.5;    // Typical prediction voltage

    let mut group = c.benchmark_group("nr_solver/1d");
    group.measurement_time(std::time::Duration::from_secs(5));

    // Benchmark with different initial guesses (warm start effect)
    for &warm_start in &[false, true] {
        let name = if warm_start { "with_warm_start" } else { "cold_start" };
        
        group.bench_function(name, |b| {
            let mut v_prev = if warm_start { 0.4 } else { 0.0 };
            b.iter(|| {
                let v0 = if warm_start { v_prev } else { 0.0 };
                let n_vt = diode.n * diode.vt;
                let vcrit = pn_vcrit(n_vt, diode.is);
                let (v, result) = nr_solve_1d(
                    |v| {
                        let i = diode.current_at(v);
                        v - p - k * i
                    },
                    |v| {
                        let g = diode.conductance_at(v);
                        1.0 - k * g
                    },
                    |vnew, vold| pnjlim(vnew, vold, n_vt, vcrit),
                    v0,
                    20,
                    1e-10,
                );
                if warm_start {
                    v_prev = v;
                }
                black_box(result);
            })
        });
    }

    group.finish();
}

fn benchmark_nr_2d_convergence(c: &mut Criterion) {
    // Two diode system (like a full-wave rectifier or back-to-back clipper)
    let diode1 = DiodeShockley::silicon();
    let diode2 = DiodeShockley::silicon();
    
    // K matrix for two coupled diodes
    let k = [[500.0, 200.0], [200.0, 500.0]];
    let p = [0.3, -0.3]; // Differential drive
    let n_vt = diode1.n * diode1.vt;
    let vcrit = pn_vcrit(n_vt, diode1.is);

    let mut group = c.benchmark_group("nr_solver/2d");
    group.measurement_time(std::time::Duration::from_secs(5));

    for &warm_start in &[false, true] {
        let name = if warm_start { "with_warm_start" } else { "cold_start" };
        
        group.bench_function(name, |b| {
            let mut v_prev = [0.0_f64; 2];
            if warm_start {
                v_prev = [0.2, -0.2];
            }
            
            b.iter(|| {
                let v0 = if warm_start { v_prev } else { [0.0, 0.0] };
                let (v1, v2, result) = nr_solve_2d(
                    |v1, v2| {
                        let i1 = diode1.current_at(v1);
                        let i2 = diode2.current_at(v2);
                        v1 - p[0] - (k[0][0] * i1 + k[0][1] * i2)
                    },
                    |v1, v2| {
                        let i1 = diode1.current_at(v1);
                        let i2 = diode2.current_at(v2);
                        v2 - p[1] - (k[1][0] * i1 + k[1][1] * i2)
                    },
                    |v1, _v2| {
                        let g = diode1.conductance_at(v1);
                        1.0 - k[0][0] * g
                    },
                    |_v1, v2| {
                        let g = diode2.conductance_at(v2);
                        -k[0][1] * g
                    },
                    |v1, _v2| {
                        let g = diode1.conductance_at(v1);
                        -k[1][0] * g
                    },
                    |_v1, v2| {
                        let g = diode2.conductance_at(v2);
                        1.0 - k[1][1] * g
                    },
                    |vnew, vold| pnjlim(vnew, vold, n_vt, vcrit),
                    |vnew, vold| pnjlim(vnew, vold, n_vt, vcrit),
                    v0[0], v0[1],
                    20,
                    1e-10,
                );
                if warm_start {
                    v_prev = [v1, v2];
                }
                black_box(result);
            })
        });
    }

    group.finish();
}

// =============================================================================
// Benchmark 3: Matrix Operations
// =============================================================================

fn benchmark_matrix_prediction(c: &mut Criterion) {
    let netlist = Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    let n = kernel.n;
    let rhs: Vec<f64> = (0..n).map(|i| i as f64 * 0.1).collect();
    let mut v_pred = vec![0.0; n];

    let mut group = c.benchmark_group("matrix/prediction");
    group.throughput(Throughput::Elements(n as u64));

    group.bench_function("s_times_rhs", |b| {
        b.iter(|| {
            kernel.predict_into(black_box(&rhs), black_box(&mut v_pred));
        })
    });

    group.finish();
}

fn benchmark_matrix_correction(c: &mut Criterion) {
    let netlist = Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    let n = kernel.n;
    let m = kernel.m;
    
    let v_pred: Vec<f64> = (0..n).map(|i| i as f64 * 0.01).collect();
    let i_nl: Vec<f64> = (0..m).map(|i| i as f64 * 1e-6).collect();
    let i_nl_prev: Vec<f64> = vec![0.0; m];
    let mut v = vec![0.0; n];
    let mut ni_inl = vec![0.0; n];

    let mut group = c.benchmark_group("matrix/correction");
    group.throughput(Throughput::Elements(n as u64));

    group.bench_function("apply_correction", |b| {
        b.iter(|| {
            kernel.apply_correction_into(
                black_box(&v_pred),
                black_box(&i_nl),
                black_box(&i_nl_prev),
                black_box(&mut v),
                black_box(&mut ni_inl),
            );
        })
    });

    group.finish();
}

fn benchmark_kernel_contribution(c: &mut Criterion) {
    let netlist = Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    let m = kernel.m;
    let i_nl: Vec<f64> = (0..m).map(|i| (i + 1) as f64 * 1e-6).collect();
    let mut k_contrib = vec![0.0; m];

    let mut group = c.benchmark_group("matrix/kernel_contribution");
    group.throughput(Throughput::Elements(m as u64));

    group.bench_function("k_times_i_nl", |b| {
        b.iter(|| {
            kernel.kernel_contribution_into(black_box(&i_nl), black_box(&mut k_contrib));
        })
    });

    group.finish();
}

fn benchmark_rhs_construction(c: &mut Criterion) {
    let netlist = Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    let diode = DiodeShockley::silicon();
    let devices = vec![DeviceEntry::new_diode(diode, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();

    // Prime the solver
    for _ in 0..10 {
        solver.process_sample(0.1);
    }

    let mut group = c.benchmark_group("matrix/rhs_construction");
    group.measurement_time(std::time::Duration::from_secs(5));

    group.bench_function("build_rhs", |b| {
        b.iter(|| {
            // Access internal method through a test interface or simulate
            // Since build_rhs is private, we test the full prediction step
            // which includes RHS construction
            black_box(solver.process_sample(0.1));
        })
    });

    group.finish();
}

// =============================================================================
// Benchmark 4: Device Evaluation
// =============================================================================

fn benchmark_diode_evaluation(c: &mut Criterion) {
    let diode = DiodeShockley::silicon();
    let voltages: Vec<f64> = (-100..=100).map(|v| v as f64 * 0.01).collect();

    let mut group = c.benchmark_group("device/diode");
    group.measurement_time(std::time::Duration::from_secs(5));

    group.bench_function("current_only", |b| {
        b.iter(|| {
            for &v in &voltages {
                black_box(diode.current_at(black_box(v)));
            }
        })
    });

    group.bench_function("conductance_only", |b| {
        b.iter(|| {
            for &v in &voltages {
                black_box(diode.conductance_at(black_box(v)));
            }
        })
    });

    group.bench_function("current_and_jacobian", |b| {
        b.iter(|| {
            for &v in &voltages {
                let i = diode.current_at(v);
                let g = diode.conductance_at(v);
                black_box((i, g));
            }
        })
    });

    group.finish();
}

fn benchmark_bjt_evaluation(c: &mut Criterion) {
    let bjt = BjtEbersMoll::npn_2n2222a();
    let vbe_values: Vec<f64> = (0..50).map(|v| v as f64 * 0.02).collect();
    let vbc_values: Vec<f64> = (-50..0).map(|v| v as f64 * 0.1).collect();

    let mut group = c.benchmark_group("device/bjt");
    group.measurement_time(std::time::Duration::from_secs(5));

    group.bench_function("collector_current", |b| {
        b.iter(|| {
            for &vbe in &vbe_values {
                for &vbc in &vbc_values {
                    black_box(bjt.collector_current(black_box(vbe), black_box(vbc)));
                }
            }
        })
    });

    group.bench_function("base_current", |b| {
        b.iter(|| {
            for &vbe in &vbe_values {
                for &vbc in &vbc_values {
                    black_box(bjt.base_current(black_box(vbe), black_box(vbc)));
                }
            }
        })
    });

    group.bench_function("jacobian", |b| {
        b.iter(|| {
            for &vbe in &vbe_values {
                for &vbc in &vbc_values {
                    black_box(bjt.collector_jacobian(black_box(vbe), black_box(vbc)));
                }
            }
        })
    });

    group.finish();
}

fn benchmark_device_array_vs_vec(c: &mut Criterion) {
    let diode = DiodeShockley::silicon();
    
    let mut group = c.benchmark_group("device/array_vs_vec");
    
    // Array-based (stack allocated)
    group.bench_function("array_stack", |b| {
        b.iter(|| {
            for i in 0..1000 {
                let v = [i as f64 * 0.001];
                black_box(diode.current_at(v[0]));
            }
        })
    });

    group.finish();
}

// =============================================================================
// Benchmark 5: Comparison Across Sample Rates
// =============================================================================

fn benchmark_sample_rate_comparison(c: &mut Criterion) {
    let netlist = Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let mut group = c.benchmark_group("comparison/sample_rate");
    group.measurement_time(std::time::Duration::from_secs(5));

    for &sample_rate in &[44100.0, 48000.0, 88200.0, 96000.0, 192000.0] {
        let kernel = DkKernel::from_mna(&mna, sample_rate).unwrap();
        let diode = DiodeShockley::silicon();
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, 0, 1).unwrap();

        group.bench_with_input(
            BenchmarkId::from_parameter(sample_rate as u64),
            &sample_rate,
            |b, _| {
                b.iter(|| solver.process_sample(black_box(0.5)))
            },
        );
    }

    group.finish();
}

// =============================================================================
// Benchmark Groups
// =============================================================================

criterion_group!(
    benches,
    // Process sample throughput
    benchmark_rc_lowpass,
    benchmark_diode_clipper,
    benchmark_bjt_amp,
    // NR solver convergence
    benchmark_nr_1d_convergence,
    benchmark_nr_2d_convergence,
    // Matrix operations
    benchmark_matrix_prediction,
    benchmark_matrix_correction,
    benchmark_kernel_contribution,
    benchmark_rhs_construction,
    // Device evaluation
    benchmark_diode_evaluation,
    benchmark_bjt_evaluation,
    benchmark_device_array_vs_vec,
    // Comparisons
    benchmark_sample_rate_comparison,
);

criterion_main!(benches);
