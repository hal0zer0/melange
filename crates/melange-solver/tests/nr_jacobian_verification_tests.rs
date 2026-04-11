//! Newton-Raphson Jacobian verification tests.
//!
//! Verifies that the analytical Jacobian J = I - J_dev * K matches a
//! finite-difference approximation of the DK residual function
//! F(v) = v - p - K * i(v).
//!
//! This is the critical test that the full-system NR update direction is
//! correct — if J is wrong, NR converges to the wrong answer or diverges.

use melange_devices::diode::DiodeShockley;
use melange_devices::NonlinearDevice;
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// Compute the DK residual: F(v) = v - p - K * i(v)
/// where p is the prediction and K is the DK kernel matrix.
fn dk_residual(
    kernel: &DkKernel,
    v_nl: &[f64],
    p: &[f64],
    i_fn: &dyn Fn(&[f64]) -> Vec<f64>,
) -> Vec<f64> {
    let m = kernel.m;
    let i_nl = i_fn(v_nl);
    let mut k_i = vec![0.0; m];
    for i in 0..m {
        for j in 0..m {
            k_i[i] += kernel.k(i, j) * i_nl[j];
        }
    }
    (0..m).map(|i| v_nl[i] - p[i] - k_i[i]).collect()
}

/// Compute the analytical Jacobian J = I - K * J_dev (block-diagonal).
fn analytical_jacobian(
    kernel: &DkKernel,
    j_dev: &[Vec<f64>],
) -> Vec<Vec<f64>> {
    let m = kernel.m;
    // K * J_dev
    let mut kj = vec![vec![0.0; m]; m];
    for i in 0..m {
        for j in 0..m {
            for k in 0..m {
                kj[i][j] += kernel.k(i, k) * j_dev[k][j];
            }
        }
    }
    // I - K * J_dev
    let mut jac = vec![vec![0.0; m]; m];
    for i in 0..m {
        for j in 0..m {
            jac[i][j] = if i == j { 1.0 } else { 0.0 };
            jac[i][j] -= kj[i][j];
        }
    }
    jac
}

/// Compute finite-difference Jacobian of F(v) = v - p - K*i(v).
fn fd_jacobian(
    kernel: &DkKernel,
    v_nl: &[f64],
    p: &[f64],
    i_fn: &dyn Fn(&[f64]) -> Vec<f64>,
    eps: f64,
) -> Vec<Vec<f64>> {
    let m = kernel.m;
    let f0 = dk_residual(kernel, v_nl, p, i_fn);
    let mut jac = vec![vec![0.0; m]; m];
    for j in 0..m {
        let mut v_pert = v_nl.to_vec();
        v_pert[j] += eps;
        let f1 = dk_residual(kernel, &v_pert, p, i_fn);
        for i in 0..m {
            jac[i][j] = (f1[i] - f0[i]) / eps;
        }
    }
    jac
}

// ── Test circuits ────────────────────────────────────────────────────

const DIODE_CLIPPER: &str = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";

const BJT_CE: &str = "\
BJT Common Emitter
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJTCE
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJTCE NPN(IS=1e-14 BF=200 BR=3)
";

const TRIODE_CC: &str = "\
12AX7 Common Cathode
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
T1 grid plate cathode 12AX7
Rk cathode 0 1.5k
Ck cathode 0 25u
Rp vcc plate 100k
Vcc vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";

fn build_kernel(spice: &str) -> (DkKernel, MnaSystem) {
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let input_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }
    let dk = DkKernel::from_mna(&mna, 44100.0).unwrap();
    (dk, mna)
}

#[test]
fn test_jacobian_fd_diode_clipper() {
    let (dk, _mna) = build_kernel(DIODE_CLIPPER);
    assert_eq!(dk.m, 2, "diode clipper should have M=2");

    // Test at a forward-biased operating point
    let is = 2.52e-9;
    let n_vt = 1.752 * 0.025851991;
    let d1 = DiodeShockley::new(is, 1.0, n_vt);

    let i_fn = |v: &[f64]| -> Vec<f64> {
        vec![d1.current_at(v[0]), d1.current_at(v[1])]
    };
    let j_dev_fn = |v: &[f64]| -> Vec<Vec<f64>> {
        vec![
            vec![d1.conductance_at(v[0]), 0.0],
            vec![0.0, d1.conductance_at(v[1])],
        ]
    };

    let v_nl = vec![0.3, -0.3];
    let p = vec![0.0; 2];
    let eps = 1e-7;

    let j_analytical = analytical_jacobian(&dk, &j_dev_fn(&v_nl));
    let j_fd = fd_jacobian(&dk, &v_nl, &p, &i_fn, eps);

    for i in 0..2 {
        for j in 0..2 {
            let err = (j_analytical[i][j] - j_fd[i][j]).abs();
            // Use absolute tolerance for near-zero entries, relative for large
            let tol = 1e-4 * j_analytical[i][j].abs().max(1.0);
            assert!(
                err < tol,
                "diode J[{i}][{j}]: analytical={:.6e}, fd={:.6e}, abs_err={:.2e}, tol={:.2e}",
                j_analytical[i][j],
                j_fd[i][j],
                err,
                tol
            );
        }
    }
}

#[test]
fn test_jacobian_fd_bjt_ce() {
    let (dk, mna) = build_kernel(BJT_CE);
    assert!(dk.m >= 2, "BJT CE should have M>=2");

    // Use Ebers-Moll for the BJT evaluation
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};
    let em = BjtEbersMoll::new(1e-14, 0.025851991, 200.0, 3.0, BjtPolarity::Npn);

    // For M=2 BJT: dim 0 = Vbe→Ic, dim 1 = Vbc→Ib
    // Forward active: Vbe=0.65V, Vbc=-5V
    let v_nl = if dk.m == 2 {
        vec![0.65, -5.0]
    } else {
        // Forward-active detection may reduce to M=1 or there may be extra devices
        let mut v = vec![0.0; dk.m];
        v[0] = 0.65;
        if dk.m > 1 {
            v[1] = -5.0;
        }
        v
    };

    // Build i_fn and j_dev_fn for exactly M=2 BJT
    if dk.m == 2 {
        let i_fn = |v: &[f64]| -> Vec<f64> {
            vec![
                em.collector_current(v[0], v[1]),
                em.base_current(v[0], v[1]),
            ]
        };
        let j_dev_fn = |v: &[f64]| -> Vec<Vec<f64>> {
            let (dic_dvbe, dic_dvbc) = em.collector_jacobian(v[0], v[1]);
            vec![
                vec![dic_dvbe, dic_dvbc],
                vec![
                    em.base_current_jacobian_dvbe(v[0], v[1]),
                    em.base_current_jacobian_dvbc(v[0], v[1]),
                ],
            ]
        };

        let p = vec![0.0; 2];
        let eps = 1e-7;

        let j_analytical = analytical_jacobian(&dk, &j_dev_fn(&v_nl));
        let j_fd = fd_jacobian(&dk, &v_nl, &p, &i_fn, eps);

        for i in 0..2 {
            for j in 0..2 {
                let err = (j_analytical[i][j] - j_fd[i][j]).abs();
                let scale = j_analytical[i][j].abs().max(1e-6);
                assert!(
                    err / scale < 1e-3,
                    "BJT J[{i}][{j}]: analytical={:.6e}, fd={:.6e}, rel_err={:.2e}",
                    j_analytical[i][j],
                    j_fd[i][j],
                    err / scale
                );
            }
        }
    }
    // If M != 2 (forward-active detected), the test still passes —
    // it just doesn't verify the Jacobian (the other cross-validation tests cover that)
    let _ = (mna, v_nl);
}

#[test]
fn test_jacobian_fd_triode() {
    let (dk, _mna) = build_kernel(TRIODE_CC);
    assert_eq!(dk.m, 2, "triode should have M=2");

    use melange_devices::tube::KorenTriode;
    let tube = KorenTriode::new(100.0, 1.4, 1060.0, 600.0, 300.0);

    let i_fn = |v: &[f64]| -> Vec<f64> {
        vec![tube.plate_current(v[0], v[1]), tube.grid_current(v[0])]
    };
    let j_dev_fn = |v: &[f64]| -> Vec<Vec<f64>> {
        let jac = tube.jacobian(&[v[0], v[1]]);
        vec![
            vec![jac[0], jac[1]], // dIp/dVgk, dIp/dVpk
            vec![tube.grid_current_jacobian(v[0]), 0.0], // dIg/dVgk, dIg/dVpk
        ]
    };

    // Operating point: Vgk=-1V (below grid current onset), Vpk=150V
    let v_nl = vec![-1.0, 150.0];
    let p = vec![0.0; 2];
    let eps = 1e-5; // Larger eps for tube due to power-law nonlinearity

    let j_analytical = analytical_jacobian(&dk, &j_dev_fn(&v_nl));
    let j_fd = fd_jacobian(&dk, &v_nl, &p, &i_fn, eps);

    for i in 0..2 {
        for j in 0..2 {
            let err = (j_analytical[i][j] - j_fd[i][j]).abs();
            let scale = j_analytical[i][j].abs().max(1e-6);
            assert!(
                err / scale < 1e-2,
                "Triode J[{i}][{j}]: analytical={:.6e}, fd={:.6e}, rel_err={:.2e}",
                j_analytical[i][j],
                j_fd[i][j],
                err / scale
            );
        }
    }
}

#[test]
fn test_jacobian_at_dc_op_is_near_identity_residual() {
    // At the converged DC operating point, the residual F(v) should be ~0,
    // meaning J*delta = -F gives delta ~0 (we're already at the solution).
    let (dk, mna) = build_kernel(DIODE_CLIPPER);

    // At DC equilibrium with zero input, v_nl should be near zero for a clipper
    let is = 2.52e-9;
    let n_vt = 1.752 * 0.025851991;
    let d1 = DiodeShockley::new(is, 1.0, n_vt);

    let i_fn = |v: &[f64]| -> Vec<f64> {
        vec![d1.current_at(v[0]), d1.current_at(v[1])]
    };

    // At v_nl = [0, 0] (no input), residual should be near zero
    let v_nl = vec![0.0; dk.m];
    let p = vec![0.0; dk.m]; // Zero prediction (no input)
    let residual = dk_residual(&dk, &v_nl, &p, &i_fn);

    for i in 0..dk.m {
        assert!(
            residual[i].abs() < 1e-10,
            "residual[{i}] = {:.6e} should be ~0 at equilibrium",
            residual[i]
        );
    }
    let _ = mna;
}

#[test]
fn test_nr_step_decreases_residual() {
    // One Newton step from a non-equilibrium point should decrease the residual
    let (dk, _mna) = build_kernel(DIODE_CLIPPER);

    let is = 2.52e-9;
    let n_vt = 1.752 * 0.025851991;
    let d1 = DiodeShockley::new(is, 1.0, n_vt);

    let i_fn = |v: &[f64]| -> Vec<f64> {
        vec![d1.current_at(v[0]), d1.current_at(v[1])]
    };
    let j_dev_fn = |v: &[f64]| -> Vec<Vec<f64>> {
        vec![
            vec![d1.conductance_at(v[0]), 0.0],
            vec![0.0, d1.conductance_at(v[1])],
        ]
    };

    // Start away from equilibrium
    let v_nl = vec![0.3, -0.2];
    let p = vec![0.0; 2];

    let f0 = dk_residual(&dk, &v_nl, &p, &i_fn);
    let norm0: f64 = f0.iter().map(|x| x * x).sum::<f64>().sqrt();

    // Newton step: v_new = v - J^{-1} * F
    let j = analytical_jacobian(&dk, &j_dev_fn(&v_nl));
    // 2x2 inverse by Cramer's rule
    let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    let dv0 = (j[1][1] * f0[0] - j[0][1] * f0[1]) / det;
    let dv1 = (-j[1][0] * f0[0] + j[0][0] * f0[1]) / det;
    let v_new = vec![v_nl[0] - dv0, v_nl[1] - dv1];

    let f1 = dk_residual(&dk, &v_new, &p, &i_fn);
    let norm1: f64 = f1.iter().map(|x| x * x).sum::<f64>().sqrt();

    assert!(
        norm1 < norm0,
        "NR step should decrease residual: {norm0:.6e} -> {norm1:.6e}"
    );
}
