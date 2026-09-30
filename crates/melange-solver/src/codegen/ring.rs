//! The trapezoidal ring predicate: does this circuit need backward Euler?
//!
//! The trapezoidal rule maps a stiff mode (a continuous pole far above the
//! audio band) to a discrete pole just inside `z = −1`, so it rings at the
//! Nyquist rate and decays slowly. Backward Euler maps the same mode near
//! `z = 0`. A ring is only a defect when it lasts, and when the program can
//! excite it and the output can see it. The predicate:
//!
//! ```text
//! promote to BE  iff  growth (rho > TRAP_BE_PROMOTION_RHO, and BE removes it)
//!                 or  some pole z with Re z < 0 and |z|^(0.01·fs) ≥ 1e-3
//!                     (still above −60 dB after 10 ms: the ring lasts)
//!                     has an input-to-output modal residue ≥ −60 dB
//!                     relative to the passband gain (it starts loud).
//! ```
//!
//! The passband gain is the pink-weighted RMS gain of the linearised
//! continuous network, `sqrt(mean |H|²)` over a log-spaced 20 Hz–20 kHz grid:
//! the output RMS for a unit pink-spectrum input, the level a program comes
//! out at. A narrow resonance counts by its width, not its height; a
//! band-limited deck is referred to its own band.
//!
//! Both constants are stated in advance, not fitted. The program level
//! cancels: the ring and the passband response scale with it alike.
//!
//! The system analysed is the SHIPPED trapezoidal integrator (the charge form,
//! `COMPANION_MODELS.md`) linearised at the DC operating point: `G` becomes
//! `G − N_i·J·N_v` with the device Jacobian `J` at the DC OP, and a saturating
//! inductor's row carries its differential inductance `L_diff(i_dc)`. Rest is
//! where a Nyquist ring is most exposed (quiet passages, after a stop).
//! Signal-dependent excitation (device current swings, edges) is not knowable
//! at compile time and is left to the runtime BE latch. Only the input port
//! is excited; a pole that no input reaches, or that no output sees, costs
//! nothing and does not promote.
//!
//! With `x` the MNA unknowns and `q` the carried charge derivative, the
//! charge form is
//!
//! ```text
//! x' = S·(H·x + q + b·u')          S = (G_l + alpha·C_l)^-1, H = alpha·C_l (zero rows)
//! q' = H·(x' − x) − q
//! ```
//!
//! `q` lives in `range(H)`. With `U` an orthonormal basis of `range(H)` and
//! `q = U·ξ`, the propagator on `(x, ξ)` is
//!
//! ```text
//! P = [ S·H              S·U           ]      B = [ S·b     ]
//!     [ Uᵀ·H·(S·H − I)   Uᵀ·H·S·U − I  ]          [ Uᵀ·H·S·b ]
//! ```
//!
//! Algebraic directions (rows with no charge) go to `z = 0` here; the
//! whole-system operator `S·(alpha·C − G)` put them at `z = −1`, which is why
//! it was the wrong operator to judge ringing on. An index-2 structure (an
//! inductor-only cutset) keeps an exact `−1`, and the predicate decides it
//! like any other pole: by its residue.

use crate::eigen::{self, Complex};

/// Growth threshold on the propagator's spectral radius.
pub use crate::codegen::stability::TRAP_BE_PROMOTION_RHO;

/// A ring that has not decayed below this factor after 10 ms lasts.
pub const RING_PERSISTENCE_FACTOR: f64 = 1e-3;
/// The window [`RING_PERSISTENCE_FACTOR`] applies over, in seconds.
pub const RING_PERSISTENCE_SECONDS: f64 = 0.01;
/// A ring whose modal residue, relative to the passband gain, is at or above
/// this starts loud (−60 dB).
pub const RING_RESIDUE_REL: f64 = 1e-3;
/// The cost comparison (ring against backward Euler's in-band change) holds
/// only where backward Euler's change is a genuine perturbation of the
/// small-signal response: under 10 % of the passband (−20 dB). Stated, not
/// derived. Above it the linearisation is near-marginal (typical of
/// regenerative circuits at their DC operating point: any `s`-mapping moves
/// the response by more than the passband itself), says nothing about what
/// happens on switching edges, and the ring threshold alone decides.
pub const BE_COMPARISON_VALID_REL: f64 = 0.1;
/// Lower edge of the band the passband gain is averaged over, Hz.
pub const PASSBAND_LO_HZ: f64 = 20.0;
/// Upper edge of the passband-gain band, Hz.
pub const PASSBAND_HI_HZ: f64 = 20000.0;
/// BE removes a growth when its own propagator's spectral radius is at most
/// this (the post-promotion limit).
pub use crate::codegen::stability::BE_POST_PROMOTION_LIMIT;

/// A saturating inductor's branch row, for linearisation at the DC OP.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SatRow {
    pub row: usize,
    pub l0: f64,
    pub isat: f64,
    /// Air-core fraction `L_air / l0`.
    pub lair: f64,
}

impl SatRow {
    /// `L_diff(i) = L_mag/cosh²(i/Isat) + L_air`, floored at `1e-6·L0` — the
    /// generated code's Jacobian entry (`SATURATING_TRANSFORMERS.md` §3.2).
    pub fn l_diff(&self, i: f64) -> f64 {
        let l_mag = self.l0 * (1.0 - self.lair);
        let l_air = self.l0 * self.lair;
        let u = (i / self.isat).clamp(-40.0, 40.0);
        let ch = u.cosh();
        (l_mag / (ch * ch) + l_air).max(1e-6 * self.l0)
    }
}

/// The linearised circuit the predicate analyses. Matrices are row-major.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RingSystem {
    pub n: usize,
    /// `G` as shipped (input conductances stamped).
    pub g: Vec<f64>,
    /// `C` as shipped (inductances on branch rows).
    pub c: Vec<f64>,
    pub m: usize,
    /// `N_v`, `m × n`.
    pub n_v: Vec<f64>,
    /// `N_i`, `n × m`.
    pub n_i: Vec<f64>,
    /// Device Jacobian `dI/dV` at the DC OP, `m × m`.
    pub j_dev: Vec<f64>,
    /// Rows whose history is zeroed (algebraic augmented rows).
    pub zero_rows: Vec<usize>,
    pub sat: Vec<SatRow>,
    /// The DC operating point (for the saturating rows' `i_dc`).
    pub dc_op: Vec<f64>,
    /// The internal (oversampled) sample rate.
    pub rate: f64,
    /// Input ports: (node, source conductance).
    pub inputs: Vec<(usize, f64)>,
    pub outputs: Vec<usize>,
}

/// A pole that lasts (`Re z < 0`, `|z|^(0.01·fs) ≥ 1e-3`), with its largest
/// input-to-output residue.
#[derive(Debug, Clone)]
pub struct RingMode {
    pub z: Complex,
    /// Max over input/output pairs of `|residue| / passband gain`.
    pub residue_rel: f64,
    /// Decay time constant of the ring envelope, seconds.
    pub tau_s: f64,
    /// The (input, output) index pair that gave `residue_rel`.
    pub port: (usize, usize),
}

impl RingMode {
    pub fn residue_db(&self) -> f64 {
        20.0 * self.residue_rel.max(1e-300).log10()
    }
}

/// The predicate's verdict with the numbers behind it.
#[derive(Debug, Clone)]
pub struct RingVerdict {
    /// Spectral radius of the trapezoidal charge propagator at the DC OP.
    pub rho: f64,
    /// `rho > TRAP_BE_PROMOTION_RHO`.
    pub growth: bool,
    /// Spectral radius of the backward-Euler propagator, evaluated only on
    /// growth.
    pub rho_be: Option<f64>,
    /// Every lasting pole on the negative side, loudest first.
    pub ring_modes: Vec<RingMode>,
    /// The verdict.
    pub promote: bool,
    /// Passband gain (pink-weighted RMS gain, 20 Hz–20 kHz) from the primary
    /// input to the primary output: the runtime BE-latch's program reference.
    pub passband_gain: f64,
    /// Continuous-time poles `λ` (rad/s, `z = (1 + λT/2)/(1 − λT/2)`) of the
    /// linearised circuit that can ring at the Nyquist rate at any host rate
    /// down to a quarter of the compiled one: stable, non-algebraic, and
    /// `|λ| > fs/2`. The runtime latch's program reference must remember the
    /// program at least as long as the slowest of them rings.
    pub ring_poles: Vec<Complex>,
    /// A pole at `z = −1` exactly (to 1e-12): an index-2 structure. It rings
    /// forever under trapezoidal integration at every rate.
    pub index2: bool,
    /// Backward Euler's worst in-band change of the response, the cost a
    /// promotion pays. Always evaluated: the decision needs it for a ring at
    /// or above the threshold or for growth, and the runtime latch, which
    /// applies the same comparison, needs it for every build it runs on.
    pub be_error: Option<InbandError>,
    /// The same for the trapezoidal rule, for the notice.
    pub trap_error: Option<InbandError>,
}

/// An integration rule's worst in-band change of the small-signal response:
/// `max |H_rule(e^{jωT}) − H(jω)|` over 20 Hz to min(20 kHz, 0.45·fs),
/// relative to the passband gain, the currency the ring level is in.
#[derive(Debug, Clone, Copy)]
pub struct InbandError {
    pub rel: f64,
    /// Where the worst change is, Hz.
    pub hz: f64,
}

impl InbandError {
    pub fn db(&self) -> f64 {
        20.0 * self.rel.max(1e-300).log10()
    }
}

impl RingVerdict {
    /// The loudest lasting ring, if any.
    pub fn loudest(&self) -> Option<&RingMode> {
        self.ring_modes.first()
    }

    /// One line for the log and the provenance: why this build integrates
    /// the way it does.
    pub fn reason(&self, rate: f64) -> String {
        if self.growth {
            return match self.rho_be {
                Some(rb) if rb <= BE_POST_PROMOTION_LIMIT => format!(
                    "trapezoidal grows at the DC operating point (rho {:.6}); backward Euler \
                     removes it (rho {:.6})",
                    self.rho, rb
                ),
                Some(rb) => format!(
                    "a real growing pole at the DC operating point (rho {:.6}) that backward \
                     Euler also keeps (rho {:.6}): an oscillator or latch; trapezoidal kept",
                    self.rho, rb
                ),
                None => format!("trapezoidal grows (rho {:.6})", self.rho),
            };
        }
        match self.loudest() {
            None => "no lasting Nyquist-side pole at the DC operating point".to_string(),
            Some(r) => {
                let mode = format!(
                    "stiff mode z = {:+.6}{} (tau {:.3} s), input residue {:.1} dB rel the \
                     passband",
                    r.z.re,
                    if r.z.im.abs() > 0.0 {
                        format!("{:+.3e}i", r.z.im)
                    } else {
                        String::new()
                    },
                    r.tau_s,
                    r.residue_db(),
                );
                let costs = match (&self.be_error, &self.trap_error) {
                    (Some(be), Some(tr)) => format!(
                        "; backward Euler would change the in-band response by up to {:.1} dB \
                         rel the passband ({:.0} Hz), trapezoidal by {:.1} dB ({:.0} Hz)",
                        be.db(),
                        be.hz,
                        tr.db(),
                        tr.hz
                    ),
                    _ => String::new(),
                };
                let comparison_valid = self
                    .be_error
                    .is_some_and(|be| be.rel <= BE_COMPARISON_VALID_REL);
                if self.promote && !comparison_valid && self.be_error.is_some() {
                    format!(
                        "{mode}; rings from the input at fs/2{costs}. The small-signal \
                         comparison is not valid here (backward Euler's change is not a \
                         perturbation: a near-marginal linearisation, typical of regenerative \
                         circuits), so the ring threshold decides: backward Euler"
                    )
                } else if self.promote {
                    format!(
                        "{mode}; rings from the input at fs/2, louder than backward Euler's own \
                         in-band change{costs}: backward Euler"
                    )
                } else if r.residue_rel >= RING_RESIDUE_REL {
                    format!(
                        "{mode}; at or above the -60 dB ring threshold but quieter than backward \
                         Euler's own in-band change{costs}: trapezoidal"
                    )
                } else {
                    format!(
                        "{mode}; below the -60 dB ring threshold: trapezoidal. A single event \
                         rings at fs/2 = {:.0} Hz; phase-coherent even-period clicks can \
                         accumulate it by up to 1/(1-|z|^P); oversampling moves it to the \
                         internal Nyquist",
                        0.5 * rate
                    )
                }
            }
        }
    }
}

/// The predicate failed to evaluate (a singular matrix or a QR iteration
/// that did not converge). The caller fails loud: a routing decision cannot
/// silently default.
#[derive(Debug, Clone)]
pub struct RingError(pub String);

impl std::fmt::Display for RingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ring predicate: {}", self.0)
    }
}

impl std::error::Error for RingError {}

/// Dense real LU inverse with partial pivoting.
fn invert(a: &[f64], n: usize) -> Result<Vec<f64>, RingError> {
    let mut m = a.to_vec();
    let mut inv = vec![0.0; n * n];
    for i in 0..n {
        inv[i * n + i] = 1.0;
    }
    let scale = a.iter().fold(0.0_f64, |s, v| s.max(v.abs()));
    for k in 0..n {
        let mut p = k;
        for i in (k + 1)..n {
            if m[i * n + k].abs() > m[p * n + k].abs() {
                p = i;
            }
        }
        if m[p * n + k].abs() <= 1e-300_f64.max(scale * 1e-300) {
            return Err(RingError(format!("singular matrix at column {k}")));
        }
        if p != k {
            for j in 0..n {
                m.swap(k * n + j, p * n + j);
                inv.swap(k * n + j, p * n + j);
            }
        }
        let d = m[k * n + k];
        for j in 0..n {
            m[k * n + j] /= d;
            inv[k * n + j] /= d;
        }
        for i in 0..n {
            if i != k {
                let f = m[i * n + k];
                if f != 0.0 {
                    for j in 0..n {
                        m[i * n + j] -= f * m[k * n + j];
                        inv[i * n + j] -= f * inv[k * n + j];
                    }
                }
            }
        }
    }
    Ok(inv)
}

fn matmul(a: &[f64], b: &[f64], r: usize, k: usize, c: usize) -> Vec<f64> {
    let mut out = vec![0.0; r * c];
    for i in 0..r {
        for l in 0..k {
            let v = a[i * k + l];
            if v != 0.0 {
                for j in 0..c {
                    out[i * c + j] += v * b[l * c + j];
                }
            }
        }
    }
    out
}

/// A row combination of the equilibrated charge rows below this (relative to
/// the largest singular value) is an exact dependency. Measured on the
/// 42-deck corpus: exact dependencies sit at <= 1.4e-16, real charge
/// directions at >= 3.5e-8.
pub const CHARGE_DEPENDENCY_TOL: f64 = 1e-12;

/// Left singular vectors and singular values of the `rows × cols` matrix
/// `m`, by one-sided Jacobi on `mᵀ` (accurate on graded matrices).
fn jacobi_left_svd(m: &[f64], rows: usize, cols: usize) -> (Vec<f64>, Vec<f64>) {
    // a = mᵀ, stored by column: a[j] = row j of m.
    let mut a: Vec<Vec<f64>> = (0..rows)
        .map(|j| m[j * cols..(j + 1) * cols].to_vec())
        .collect();
    let mut v = vec![0.0; rows * rows];
    for i in 0..rows {
        v[i * rows + i] = 1.0;
    }
    for _sweep in 0..80 {
        let mut rotated = false;
        for p in 0..rows {
            for q in (p + 1)..rows {
                let alpha: f64 = a[p].iter().map(|x| x * x).sum();
                let beta: f64 = a[q].iter().map(|x| x * x).sum();
                let gamma: f64 = a[p].iter().zip(&a[q]).map(|(x, y)| x * y).sum();
                if gamma == 0.0 || gamma.abs() <= f64::EPSILON * (alpha * beta).sqrt() {
                    continue;
                }
                rotated = true;
                let zeta = (beta - alpha) / (2.0 * gamma);
                let t = zeta.signum() / (zeta.abs() + (1.0 + zeta * zeta).sqrt());
                let t = if zeta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = c * t;
                for k in 0..cols {
                    let (x, y) = (a[p][k], a[q][k]);
                    a[p][k] = c * x - s * y;
                    a[q][k] = s * x + c * y;
                }
                for k in 0..rows {
                    let (x, y) = (v[k * rows + p], v[k * rows + q]);
                    v[k * rows + p] = c * x - s * y;
                    v[k * rows + q] = s * x + c * y;
                }
            }
        }
        if !rotated {
            break;
        }
    }
    let sigma = a
        .iter()
        .map(|col| col.iter().map(|x| x * x).sum::<f64>().sqrt())
        .collect();
    (v, sigma)
}

/// Orthonormal basis (columns, `n × r`) of `range(h)`.
///
/// `q = h·(…)` lives on the rows of `h` that carry charge, less any exact
/// dependency among those rows (a capacitor between two nodes that carry no
/// other charge makes their rows sum to zero). The dependencies are found on
/// the row- and column-equilibrated charge rows, where rank does not depend
/// on scale; the basis is then the exact unit vectors of the rows no
/// dependency touches, plus an orthonormal complement of the dependencies on
/// the rows they do. No direction is mixed with another of a different
/// scale, which a numerical basis of the graded `h` itself (charge
/// directions spanning 1e-13 of the largest on the corpus) cannot promise.
fn range_basis(h: &[f64], n: usize) -> (Vec<f64>, usize) {
    let rows: Vec<usize> = (0..n)
        .filter(|&i| h[i * n..(i + 1) * n].iter().any(|&x| x != 0.0))
        .collect();
    let nr = rows.len();
    // Equilibrate the charge rows (alternating row and column max scaling).
    let mut m: Vec<f64> = rows
        .iter()
        .flat_map(|&i| h[i * n..(i + 1) * n].to_vec())
        .collect();
    let mut row_scale = vec![1.0; nr];
    for _ in 0..3 {
        for (k, rs) in row_scale.iter_mut().enumerate() {
            let mx = m[k * n..(k + 1) * n]
                .iter()
                .fold(0.0_f64, |a, x| a.max(x.abs()));
            if mx > 0.0 {
                m[k * n..(k + 1) * n].iter_mut().for_each(|x| *x /= mx);
                *rs /= mx;
            }
        }
        for j in 0..n {
            let mx = (0..nr).fold(0.0_f64, |a, k| a.max(m[k * n + j].abs()));
            if mx > 0.0 {
                for k in 0..nr {
                    m[k * n + j] /= mx;
                }
            }
        }
    }
    let (v, sigma) = jacobi_left_svd(&m, nr, n);
    let smax = sigma.iter().fold(0.0_f64, |a, &x| a.max(x));
    // Dependencies y with yᵀ·h_rows = 0, in unscaled row coordinates.
    let deps: Vec<Vec<f64>> = (0..nr)
        .filter(|&j| sigma[j] <= CHARGE_DEPENDENCY_TOL * smax)
        .map(|j| {
            let mut y: Vec<f64> = (0..nr).map(|k| v[k * nr + j] * row_scale[k]).collect();
            let nrm = y.iter().map(|x| x * x).sum::<f64>().sqrt();
            y.iter_mut().for_each(|x| *x /= nrm);
            y
        })
        .collect();
    let touched: Vec<usize> = (0..nr)
        .filter(|&k| deps.iter().any(|y| y[k].abs() > 0.0))
        .collect();
    let mut basis: Vec<Vec<f64>> = Vec::new();
    for k in 0..nr {
        if !touched.contains(&k) {
            let mut e = vec![0.0; n];
            e[rows[k]] = 1.0;
            basis.push(e);
        }
    }
    if !deps.is_empty() {
        // Orthonormal complement of span(deps) within the touched rows:
        // Gram-Schmidt (twice) of the touched unit vectors against the
        // dependencies and the vectors already kept.
        let t = touched.len();
        let mut ortho: Vec<Vec<f64>> = Vec::new();
        for y in &deps {
            let mut u: Vec<f64> = touched.iter().map(|&k| y[k]).collect();
            for _ in 0..2 {
                for o in &ortho {
                    let d: f64 = o.iter().zip(&u).map(|(a, b)| a * b).sum();
                    u.iter_mut().zip(o).for_each(|(x, oo)| *x -= d * oo);
                }
            }
            let nrm = u.iter().map(|x| x * x).sum::<f64>().sqrt();
            if nrm > 1e-8 {
                u.iter_mut().for_each(|x| *x /= nrm);
                ortho.push(u);
            }
        }
        let n_deps = ortho.len();
        for kk in 0..t {
            let mut u = vec![0.0; t];
            u[kk] = 1.0;
            for _ in 0..2 {
                for o in &ortho {
                    let d: f64 = o.iter().zip(&u).map(|(a, b)| a * b).sum();
                    u.iter_mut().zip(o).for_each(|(x, oo)| *x -= d * oo);
                }
            }
            let nrm = u.iter().map(|x| x * x).sum::<f64>().sqrt();
            if nrm > 1e-8 && ortho.len() < t {
                u.iter_mut().for_each(|x| *x /= nrm);
                ortho.push(u);
            }
        }
        for u in ortho.into_iter().skip(n_deps) {
            let mut e = vec![0.0; n];
            for (kk, &k) in touched.iter().enumerate() {
                e[rows[k]] = u[kk];
            }
            basis.push(e);
        }
    }
    let r = basis.len();
    let mut q = vec![0.0; n * r];
    for (col, e) in basis.iter().enumerate() {
        for i in 0..n {
            q[i * r + col] = e[i];
        }
    }
    (q, r)
}

/// The linearised matrices: `(G_l, C_l)`.
fn linearised(sys: &RingSystem) -> (Vec<f64>, Vec<f64>) {
    let n = sys.n;
    let m = sys.m;
    let mut g = sys.g.clone();
    if m > 0 {
        // G_l = G − N_i·J·N_v
        let jn = matmul(&sys.j_dev, &sys.n_v, m, m, n);
        let nij = matmul(&sys.n_i, &jn, n, m, n);
        for (gi, d) in g.iter_mut().zip(&nij) {
            *gi -= d;
        }
    }
    let mut c = sys.c.clone();
    for s in &sys.sat {
        let i_dc = sys.dc_op.get(s.row).copied().unwrap_or(0.0);
        c[s.row * n + s.row] += s.l_diff(i_dc) - s.l0;
    }
    (g, c)
}

/// `|H(j·2π·f)|` from input port `inp` to output `out` of the linearised
/// continuous network.
fn gain_at(g: &[f64], c: &[f64], n: usize, inp: (usize, f64), out: usize, f: f64) -> f64 {
    let w = 2.0 * std::f64::consts::PI * f;
    let mut m: Vec<Complex> = g
        .iter()
        .zip(c)
        .map(|(&gv, &cv)| Complex::new(gv, w * cv))
        .collect();
    let mut b = vec![Complex::ZERO; n];
    b[inp.0] = Complex::real(inp.1);
    let piv = eigen::lu_factor(&mut m, n, 1e-300);
    let x = eigen::lu_solve(&m, &piv, n, &b);
    x[out].abs()
}

/// `H(s)` from input port `inp` to output `out` of the linearised network,
/// `(G + s·C)⁻¹` at the output row, driven by the port's source conductance.
fn response_at(
    g: &[f64],
    c: &[f64],
    n: usize,
    inp: (usize, f64),
    out: usize,
    s: Complex,
) -> Complex {
    let mut m: Vec<Complex> = g
        .iter()
        .zip(c)
        .map(|(&gv, &cv)| Complex::real(gv) + s * cv)
        .collect();
    let mut b = vec![Complex::ZERO; n];
    b[inp.0] = Complex::real(inp.1);
    let piv = eigen::lu_factor(&mut m, n, 1e-300);
    let x = eigen::lu_solve(&m, &piv, n, &b);
    x[out]
}

/// An integration rule as the `s` it substitutes at `z = e^{jωT}`.
#[derive(Clone, Copy)]
enum Rule {
    BackwardEuler,
    Trapezoidal,
}

impl Rule {
    fn s(self, w: f64, t: f64) -> Complex {
        // z^-1 = e^{-jωT}
        let zi = Complex::new((w * t).cos(), -(w * t).sin());
        let one = Complex::real(1.0);
        match self {
            Rule::BackwardEuler => (one - zi) * (1.0 / t),
            Rule::Trapezoidal => (one - zi) / (one + zi) * (2.0 / t),
        }
    }
}

/// The rule's worst in-band change of the response over every input/output
/// pair, relative to that pair's passband gain `pbs`.
fn inband_error(
    sys: &RingSystem,
    g: &[f64],
    c: &[f64],
    pbs: &[Vec<f64>],
    rule: Rule,
) -> InbandError {
    const POINTS: usize = 481;
    let t = 1.0 / sys.rate;
    let hi = PASSBAND_HI_HZ.min(0.45 * sys.rate);
    let (l0, l1) = (PASSBAND_LO_HZ.log10(), hi.log10());
    let mut worst = InbandError { rel: 0.0, hz: 0.0 };
    for k in 0..POINTS {
        let f = 10f64.powf(l0 + (l1 - l0) * k as f64 / (POINTS - 1) as f64);
        let w = 2.0 * std::f64::consts::PI * f;
        let s_rule = rule.s(w, t);
        for (ii, &inp) in sys.inputs.iter().enumerate() {
            for (oi, &out) in sys.outputs.iter().enumerate() {
                let pb = pbs[ii][oi];
                if !(pb.is_finite() && pb > 0.0) {
                    continue;
                }
                let exact = response_at(g, c, sys.n, inp, out, Complex::new(0.0, w));
                let discrete = response_at(g, c, sys.n, inp, out, s_rule);
                let rel = (discrete - exact).abs() / pb;
                if rel > worst.rel {
                    worst = InbandError { rel, hz: f };
                }
            }
        }
    }
    worst
}

/// The passband gain: the pink-weighted RMS gain,
/// `sqrt(mean |H|²)` over a 481-point log-spaced grid from
/// [`PASSBAND_LO_HZ`] to [`PASSBAND_HI_HZ`] — the output RMS for a unit
/// pink-spectrum input. It stands for the output level a program produces:
/// a narrow resonance counts by its width, not its height, and a band-limited
/// deck is referred to its own band.
pub fn passband_gain(g: &[f64], c: &[f64], n: usize, inp: (usize, f64), out: usize) -> f64 {
    const POINTS: usize = 481;
    let (l0, l1) = (PASSBAND_LO_HZ.log10(), PASSBAND_HI_HZ.log10());
    let sum: f64 = (0..POINTS)
        .map(|k| {
            let f = 10f64.powf(l0 + (l1 - l0) * k as f64 / (POINTS - 1) as f64);
            let h = gain_at(g, c, n, inp, out, f);
            h * h
        })
        .sum();
    (sum / POINTS as f64).sqrt()
}

/// The trapezoidal charge propagator `P` (`dim × dim`, `dim = n + rank(H)`),
/// the input columns `B` (one per input port), and `(S, H)` for reuse.
pub struct Propagator {
    pub dim: usize,
    pub p: Vec<f64>,
    pub b: Vec<Vec<f64>>,
    pub rank_h: usize,
}

/// Build the reduced charge propagator of `sys` at its internal rate.
pub fn propagator(sys: &RingSystem) -> Result<Propagator, RingError> {
    let n = sys.n;
    let (g, c) = linearised(sys);
    let alpha = 2.0 * sys.rate;
    let mut h: Vec<f64> = c.iter().map(|v| alpha * v).collect();
    for &r in &sys.zero_rows {
        if r < n {
            h[r * n..(r + 1) * n].fill(0.0);
        }
    }
    let a: Vec<f64> = g.iter().zip(&c).map(|(gv, cv)| gv + alpha * cv).collect();
    let s = invert(&a, n)?;
    let (u, r) = range_basis(&h, n);
    let dim = n + r;
    let sh = matmul(&s, &h, n, n, n);
    let su = matmul(&s, &u, n, n, r);
    // Uᵀ·H
    let mut uth = vec![0.0; r * n];
    for k in 0..r {
        for j in 0..n {
            uth[k * n + j] = (0..n).map(|i| u[i * r + k] * h[i * n + j]).sum();
        }
    }
    let mut sh_i = sh.clone();
    for i in 0..n {
        sh_i[i * n + i] -= 1.0;
    }
    let bl = matmul(&uth, &sh_i, r, n, n); // Uᵀ·H·(S·H − I)
    let br = matmul(&uth, &su, r, n, r); // Uᵀ·H·S·U
    let mut p = vec![0.0; dim * dim];
    for i in 0..n {
        for j in 0..n {
            p[i * dim + j] = sh[i * n + j];
        }
        for j in 0..r {
            p[i * dim + n + j] = su[i * r + j];
        }
    }
    for k in 0..r {
        for j in 0..n {
            p[(n + k) * dim + j] = bl[k * n + j];
        }
        for j in 0..r {
            p[(n + k) * dim + n + j] = br[k * r + j] - if j == k { 1.0 } else { 0.0 };
        }
    }
    let b = sys
        .inputs
        .iter()
        .map(|&(node, gin)| {
            let sb: Vec<f64> = (0..n).map(|i| s[i * n + node] * gin).collect();
            let mut col = sb.clone();
            for k in 0..r {
                col.push((0..n).map(|j| uth[k * n + j] * sb[j]).sum());
            }
            col
        })
        .collect();
    Ok(Propagator {
        dim,
        p,
        b,
        rank_h: r,
    })
}

/// Spectral radius of the backward-Euler propagator `S_be·H_be` at the same
/// linearisation (`H_be = C_l/T`, zero rows kept).
fn rho_be(sys: &RingSystem) -> Result<f64, RingError> {
    let n = sys.n;
    let (g, c) = linearised(sys);
    let alpha = sys.rate;
    let mut h: Vec<f64> = c.iter().map(|v| alpha * v).collect();
    for &r in &sys.zero_rows {
        if r < n {
            h[r * n..(r + 1) * n].fill(0.0);
        }
    }
    let a: Vec<f64> = g.iter().zip(&c).map(|(gv, cv)| gv + alpha * cv).collect();
    let s = invert(&a, n)?;
    let pbe = matmul(&s, &h, n, n, n);
    let e = eigen::eigenvalues(&pbe, n).map_err(|e| RingError(e.to_string()))?;
    Ok(e.iter().fold(0.0_f64, |m, z| m.max(z.abs())))
}

/// All eigenvalues of the charge propagator (for tests and diagnostics).
pub fn propagator_eigenvalues(sys: &RingSystem) -> Result<Vec<Complex>, RingError> {
    let prop = propagator(sys)?;
    eigen::eigenvalues(&prop.p, prop.dim).map_err(|e| RingError(e.to_string()))
}

/// Evaluate the predicate.
pub fn analyze(sys: &RingSystem) -> Result<RingVerdict, RingError> {
    let prop = propagator(sys)?;
    let dim = prop.dim;
    let eig = eigen::eigenvalues(&prop.p, dim).map_err(|e| RingError(e.to_string()))?;
    let rho = eig.iter().fold(0.0_f64, |m, z| m.max(z.abs()));
    let growth = rho > TRAP_BE_PROMOTION_RHO;

    let (g, c) = linearised(sys);
    let pbs: Vec<Vec<f64>> = sys
        .inputs
        .iter()
        .map(|&inp| {
            sys.outputs
                .iter()
                .map(|&out| passband_gain(&g, &c, sys.n, inp, out))
                .collect()
        })
        .collect();
    let lasts_n = RING_PERSISTENCE_SECONDS * sys.rate;
    let mut ring_modes = Vec::new();
    for &z in &eig {
        if z.re >= 0.0 || z.abs().powf(lasts_n) < RING_PERSISTENCE_FACTOR {
            continue;
        }
        // A conjugate pair shares its residue magnitude: evaluate one member.
        if z.im < 0.0 {
            continue;
        }
        let r = eigen::eigenvector(&prop.p, dim, z, false);
        let l = eigen::eigenvector(&prop.p, dim, z, true);
        let lr = eigen::dot(&l, &r);
        let mut best = (0.0_f64, (0usize, 0usize));
        for (ii, bcol) in prop.b.iter().enumerate() {
            let bc: Vec<Complex> = bcol.iter().map(|&v| Complex::real(v)).collect();
            let lb = eigen::dot(&l, &bc);
            for (oi, &out) in sys.outputs.iter().enumerate() {
                let pb = pbs[ii][oi];
                if pb.is_nan() || pb <= 0.0 || pb.is_infinite() {
                    continue;
                }
                let res = (r[out] * lb / lr).abs();
                let rel = res / pb;
                if rel > best.0 {
                    best = (rel, (ii, oi));
                }
            }
        }
        let decay = 1.0 - z.abs();
        ring_modes.push(RingMode {
            z,
            residue_rel: best.0,
            tau_s: if decay > 0.0 {
                1.0 / (decay * sys.rate)
            } else {
                f64::INFINITY
            },
            port: best.1,
        });
    }
    ring_modes.sort_by(|a, b| b.residue_rel.total_cmp(&a.residue_rel));

    let rho_be = if growth { Some(rho_be(sys)?) } else { None };
    // What each integrator would cost in band, on the same small-signal
    // system and in the same currency as the ring level. The decision needs
    // it for a loud ring or growth; the runtime latch, which applies the
    // same comparison at run time, needs it for every build.
    let (be_error, trap_error) = (
        Some(inband_error(sys, &g, &c, &pbs, Rule::BackwardEuler)),
        Some(inband_error(sys, &g, &c, &pbs, Rule::Trapezoidal)),
    );
    // A ring promotes only when trapezoidal's artefact is louder than the
    // damage backward Euler would do in band, where that comparison holds
    // (BE_COMPARISON_VALID_REL). Growth still promotes unconditionally when
    // backward Euler removes it.
    let promote = match rho_be {
        Some(rb) => rb <= BE_POST_PROMOTION_LIMIT,
        None => ring_modes.first().is_some_and(|r| {
            r.residue_rel >= RING_RESIDUE_REL
                && be_error.is_none_or(|be| {
                    // The comparison, where it holds; the threshold alone
                    // where it does not (announced in the reason).
                    be.rel > BE_COMPARISON_VALID_REL || r.residue_rel > be.rel
                })
        }),
    };
    // Poles that can ring at fs/2 at host rates down to fs/4: map each
    // non-algebraic eigenvalue back through the bilinear transform,
    // lambda = 2 fs (z - 1)/(z + 1), and keep the stable ones with
    // |lambda| > fs/2 (z(fs') is Nyquist-side iff |lambda| > 2 fs').
    let mut ring_poles = Vec::new();
    let mut index2 = false;
    for &z in &eig {
        if z.abs() < 1e-9 {
            continue;
        }
        let zp1 = z + Complex::real(1.0);
        if zp1.abs() < 1e-12 {
            index2 = true;
            continue;
        }
        let lambda = (z - Complex::real(1.0)) / zp1 * (2.0 * sys.rate);
        if lambda.re < 0.0 && lambda.abs() > 0.5 * sys.rate && lambda.im >= 0.0 {
            ring_poles.push(lambda);
        }
    }
    let passband_gain = pbs
        .first()
        .and_then(|row| row.first())
        .copied()
        .unwrap_or(0.0);
    Ok(RingVerdict {
        rho,
        growth,
        rho_be,
        ring_modes,
        promote,
        passband_gain,
        ring_poles,
        index2,
        be_error,
        trap_error,
    })
}

impl RingSystem {
    /// The linearised system of a built trapezoidal IR: its shipped `G` and
    /// `C`, the device Jacobian evaluated at its DC operating point, its
    /// saturating rows, internal rate, and input/output ports.
    pub fn from_ir(ir: &crate::codegen::ir::CircuitIR) -> Result<Self, RingError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let dc_op: Vec<f64> = (0..n)
            .map(|i| ir.dc_operating_point.get(i).copied().unwrap_or(0.0))
            .collect();
        let mut j_dev = vec![0.0; m * m];
        if m > 0 {
            let mut v_nl = vec![0.0; m];
            for (i, v) in v_nl.iter_mut().enumerate() {
                *v = (0..n).map(|j| ir.matrices.n_v[i * n + j] * dc_op[j]).sum();
            }
            let mut i_nl = vec![0.0; m];
            crate::dc_op::evaluate_devices_with_nodes(
                &v_nl,
                &ir.device_slots,
                &mut i_nl,
                &mut j_dev,
                m,
                &dc_op,
            );
        }
        let sc = &ir.solver_config;
        let mut inputs = vec![(sc.input_node, 1.0 / sc.input_resistance)];
        for (node, r) in sc.extra_input_nodes.iter().zip(&sc.extra_input_resistances) {
            inputs.push((*node, 1.0 / r));
        }
        Ok(RingSystem {
            n,
            g: ir.matrices.g_matrix.clone(),
            c: ir.matrices.c_matrix.clone(),
            m,
            n_v: ir.matrices.n_v.clone(),
            n_i: ir.matrices.n_i.clone(),
            j_dev,
            zero_rows: ir.topology.history_zero_rows.clone(),
            sat: ir
                .saturating_inductors
                .iter()
                .map(|s| SatRow {
                    row: s.aug_row,
                    l0: s.l0,
                    isat: s.isat,
                    lair: s.lair,
                })
                .collect(),
            dc_op,
            rate: sc.sample_rate * sc.oversampling_factor as f64,
            inputs,
            outputs: sc.output_nodes.clone(),
        })
    }
}
