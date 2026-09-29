//! Dense eigenvalues of a real nonsymmetric matrix, and modal residues.
//!
//! Compile-time only: the trapezoidal ring predicate
//! (`codegen::ring`) needs every eigenvalue of the reduced charge
//! propagator (for growth and the near-(−1) set) and, for the few flagged
//! poles, the right and left eigenvectors that give a pole's modal residue.
//! Matrices are small (a few hundred rows at most) and dense.
//!
//! Method (EISPACK `rg` without vectors, then inverse iteration):
//! 1. [`balance`]: diagonal similarity by powers of 2 (exact in floating
//!    point) so row and column norms are comparable. The propagator mixes
//!    dimensionless blocks with blocks in siemens and amperes, so this
//!    matters.
//! 2. Reduction to upper Hessenberg form by stabilized elementary
//!    similarity transformations (EISPACK `elmhes`).
//! 3. Francis double-shift QR on the Hessenberg matrix (EISPACK `hqr`),
//!    with the double-precision deflation and small-subdiagonal tests.
//! 4. [`eigenvector`]: inverse iteration in complex arithmetic on the
//!    ORIGINAL matrix at a computed eigenvalue, for the right vector
//!    (`A·r = λ·r`) or the left one (`lᵀ·A = λ·lᵀ`).
//!
//! Accuracy is gated against LAPACK (numpy/scipy `eig`) by
//! `tests/eigen_reference_tests.rs`.

use std::ops::{Add, Div, Mul, Neg, Sub};

/// A complex number, just enough for eigenvalues and inverse iteration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub const ZERO: Complex = Complex { re: 0.0, im: 0.0 };

    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn real(re: f64) -> Self {
        Self { re, im: 0.0 }
    }

    pub fn abs(self) -> f64 {
        self.re.hypot(self.im)
    }

    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }
}

impl Add for Complex {
    type Output = Complex;
    fn add(self, o: Complex) -> Complex {
        Complex::new(self.re + o.re, self.im + o.im)
    }
}

impl Sub for Complex {
    type Output = Complex;
    fn sub(self, o: Complex) -> Complex {
        Complex::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for Complex {
    type Output = Complex;
    fn mul(self, o: Complex) -> Complex {
        Complex::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}

impl Mul<f64> for Complex {
    type Output = Complex;
    fn mul(self, k: f64) -> Complex {
        Complex::new(self.re * k, self.im * k)
    }
}

impl Div for Complex {
    type Output = Complex;
    /// Smith's algorithm (no overflow for large components).
    fn div(self, o: Complex) -> Complex {
        if o.re.abs() >= o.im.abs() {
            let r = o.im / o.re;
            let d = o.re + o.im * r;
            Complex::new((self.re + self.im * r) / d, (self.im - self.re * r) / d)
        } else {
            let r = o.re / o.im;
            let d = o.re * r + o.im;
            Complex::new((self.re * r + self.im) / d, (self.im * r - self.re) / d)
        }
    }
}

impl Neg for Complex {
    type Output = Complex;
    fn neg(self) -> Complex {
        Complex::new(-self.re, -self.im)
    }
}

/// The QR iteration did not converge (EISPACK `hqr`'s `ierr`).
#[derive(Debug, Clone, PartialEq)]
pub struct EigenError {
    /// Eigenvalues still undetermined when the iteration budget ran out.
    pub unresolved: usize,
}

impl std::fmt::Display for EigenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "QR iteration did not converge: {} eigenvalue(s) unresolved",
            self.unresolved
        )
    }
}

impl std::error::Error for EigenError {}

/// QR sweeps allowed per eigenvalue before giving up. EISPACK allows 30;
/// the extra room only costs time on a matrix that is already pathological.
const MAX_SWEEPS_PER_EIGENVALUE: usize = 60;

/// 1-based square matrix storage, so the EISPACK loops read as published.
struct Mat1 {
    n: usize,
    a: Vec<f64>,
}

impl Mat1 {
    fn from_row_major(a: &[f64], n: usize) -> Self {
        let mut m = Mat1 {
            n,
            a: vec![0.0; (n + 1) * (n + 1)],
        };
        for i in 0..n {
            for j in 0..n {
                m.set(i + 1, j + 1, a[i * n + j]);
            }
        }
        m
    }

    #[inline]
    fn get(&self, i: usize, j: usize) -> f64 {
        self.a[i * (self.n + 1) + j]
    }

    #[inline]
    fn set(&mut self, i: usize, j: usize, v: f64) {
        self.a[i * (self.n + 1) + j] = v;
    }

    #[inline]
    fn sub_assign(&mut self, i: usize, j: usize, v: f64) {
        self.a[i * (self.n + 1) + j] -= v;
    }

    #[inline]
    fn add_assign(&mut self, i: usize, j: usize, v: f64) {
        self.a[i * (self.n + 1) + j] += v;
    }

    fn swap(&mut self, i1: usize, j1: usize, i2: usize, j2: usize) {
        let w = self.n + 1;
        self.a.swap(i1 * w + j1, i2 * w + j2);
    }
}

/// Balance by powers of 2 (EISPACK `balanc`, no permutations). A diagonal
/// similarity `D⁻¹·A·D`: eigenvalues are unchanged and, the factors being
/// powers of 2, so are the entries' mantissas. Returns `D` (1-based).
fn balance(a: &mut Mat1) -> Vec<f64> {
    const RADIX: f64 = 2.0;
    let sqrdx = RADIX * RADIX;
    let n = a.n;
    let mut d = vec![1.0; n + 1];
    let mut done = false;
    while !done {
        done = true;
        for i in 1..=n {
            let mut c = 0.0;
            let mut r = 0.0;
            for j in 1..=n {
                if j != i {
                    c += a.get(j, i).abs();
                    r += a.get(i, j).abs();
                }
            }
            if c != 0.0 && r != 0.0 {
                let mut g = r / RADIX;
                let mut f = 1.0;
                let s = c + r;
                while c < g {
                    f *= RADIX;
                    c *= sqrdx;
                }
                g = r * RADIX;
                while c > g {
                    f /= RADIX;
                    c /= sqrdx;
                }
                if (c + r) / f < 0.95 * s {
                    done = false;
                    let g = 1.0 / f;
                    for j in 1..=n {
                        a.set(i, j, a.get(i, j) * g);
                    }
                    for j in 1..=n {
                        a.set(j, i, a.get(j, i) * f);
                    }
                    d[i] *= f;
                }
            }
        }
    }
    d
}

/// Reduce to upper Hessenberg form by elimination with partial pivoting
/// (EISPACK `elmhes`), then clear the multipliers left below the
/// subdiagonal.
fn hessenberg(a: &mut Mat1) {
    let n = a.n;
    for m in 2..n {
        let mut x = 0.0_f64;
        let mut i = m;
        for j in m..=n {
            if a.get(j, m - 1).abs() > x.abs() {
                x = a.get(j, m - 1);
                i = j;
            }
        }
        if i != m {
            for j in (m - 1)..=n {
                a.swap(i, j, m, j);
            }
            for j in 1..=n {
                a.swap(j, i, j, m);
            }
        }
        if x != 0.0 {
            for i in (m + 1)..=n {
                let mut y = a.get(i, m - 1);
                if y != 0.0 {
                    y /= x;
                    a.set(i, m - 1, y);
                    for j in m..=n {
                        let v = y * a.get(m, j);
                        a.sub_assign(i, j, v);
                    }
                    for j in 1..=n {
                        let v = y * a.get(j, i);
                        a.add_assign(j, m, v);
                    }
                }
            }
        }
    }
    for i in 1..=n {
        for j in 1..=n {
            if i > j + 1 {
                a.set(i, j, 0.0);
            }
        }
    }
}

#[inline]
fn sign(a: f64, b: f64) -> f64 {
    if b >= 0.0 {
        a.abs()
    } else {
        -a.abs()
    }
}

/// Eigenvalues of an upper Hessenberg matrix by Francis double-shift QR
/// (EISPACK `hqr`). Destroys `a`.
fn hqr(a: &mut Mat1) -> Result<Vec<Complex>, EigenError> {
    let n = a.n;
    let mut wr = vec![0.0; n + 1];
    let mut wi = vec![0.0; n + 1];

    let mut anorm = 0.0;
    for i in 1..=n {
        for j in i.saturating_sub(1).max(1)..=n {
            anorm += a.get(i, j).abs();
        }
    }

    let mut nn = n;
    let mut t = 0.0;
    while nn >= 1 {
        let mut its = 0usize;
        loop {
            // Look for a single small subdiagonal element.
            let mut l = nn;
            while l >= 2 {
                let mut s = a.get(l - 1, l - 1).abs() + a.get(l, l).abs();
                if s == 0.0 {
                    s = anorm;
                }
                if a.get(l, l - 1).abs() + s == s {
                    a.set(l, l - 1, 0.0);
                    break;
                }
                l -= 1;
            }
            let mut x = a.get(nn, nn);
            if l == nn {
                // One root found.
                wr[nn] = x + t;
                wi[nn] = 0.0;
                nn -= 1;
                break;
            }
            let mut y = a.get(nn - 1, nn - 1);
            let mut w = a.get(nn, nn - 1) * a.get(nn - 1, nn);
            if l == nn - 1 {
                // Two roots found.
                let p = 0.5 * (y - x);
                let q = p * p + w;
                let mut z = q.abs().sqrt();
                x += t;
                if q >= 0.0 {
                    z = p + sign(z, p);
                    wr[nn - 1] = x + z;
                    wr[nn] = x + z;
                    if z != 0.0 {
                        wr[nn] = x - w / z;
                    }
                    wi[nn - 1] = 0.0;
                    wi[nn] = 0.0;
                } else {
                    wr[nn - 1] = x + p;
                    wr[nn] = x + p;
                    wi[nn - 1] = -z;
                    wi[nn] = z;
                }
                nn -= 2;
                break;
            }
            if its == MAX_SWEEPS_PER_EIGENVALUE {
                return Err(EigenError { unresolved: nn });
            }
            if its == 10 || its == 20 {
                // Exceptional shift.
                t += x;
                for i in 1..=nn {
                    a.sub_assign(i, i, x);
                }
                let s = a.get(nn, nn - 1).abs() + a.get(nn - 1, nn - 2).abs();
                x = 0.75 * s;
                y = x;
                w = -0.4375 * s * s;
            }
            its += 1;

            // Form the shift and look for two consecutive small subdiagonal
            // elements.
            let mut m = nn - 2;
            let (mut p, mut q, mut r);
            loop {
                let z = a.get(m, m);
                let r0 = x - z;
                let s0 = y - z;
                p = (r0 * s0 - w) / a.get(m + 1, m) + a.get(m, m + 1);
                q = a.get(m + 1, m + 1) - z - r0 - s0;
                r = a.get(m + 2, m + 1);
                let s = p.abs() + q.abs() + r.abs();
                p /= s;
                q /= s;
                r /= s;
                if m == l {
                    break;
                }
                let u = a.get(m, m - 1).abs() * (q.abs() + r.abs());
                let v = p.abs() * (a.get(m - 1, m - 1).abs() + z.abs() + a.get(m + 1, m + 1).abs());
                if u + v == v {
                    break;
                }
                m -= 1;
            }
            for i in (m + 2)..=nn {
                a.set(i, i - 2, 0.0);
                if i != m + 2 {
                    a.set(i, i - 3, 0.0);
                }
            }

            // Double QR step on rows l..nn and columns m..nn.
            let mut xk = 0.0;
            for k in m..nn {
                if k != m {
                    p = a.get(k, k - 1);
                    q = a.get(k + 1, k - 1);
                    r = 0.0;
                    if k != nn - 1 {
                        r = a.get(k + 2, k - 1);
                    }
                    xk = p.abs() + q.abs() + r.abs();
                    if xk != 0.0 {
                        p /= xk;
                        q /= xk;
                        r /= xk;
                    }
                }
                let s = sign((p * p + q * q + r * r).sqrt(), p);
                if s != 0.0 {
                    if k == m {
                        if l != m {
                            a.set(k, k - 1, -a.get(k, k - 1));
                        }
                    } else {
                        a.set(k, k - 1, -s * xk);
                    }
                    p += s;
                    let xx = p / s;
                    let yy = q / s;
                    let zz = r / s;
                    q /= p;
                    r /= p;
                    for j in k..=nn {
                        let mut pp = a.get(k, j) + q * a.get(k + 1, j);
                        if k != nn - 1 {
                            pp += r * a.get(k + 2, j);
                            a.sub_assign(k + 2, j, pp * zz);
                        }
                        a.sub_assign(k + 1, j, pp * yy);
                        a.sub_assign(k, j, pp * xx);
                    }
                    let mmin = if nn < k + 3 { nn } else { k + 3 };
                    for i in l..=mmin {
                        let mut pp = xx * a.get(i, k) + yy * a.get(i, k + 1);
                        if k != nn - 1 {
                            pp += zz * a.get(i, k + 2);
                            a.sub_assign(i, k + 2, pp * r);
                        }
                        a.sub_assign(i, k + 1, pp * q);
                        a.sub_assign(i, k, pp);
                    }
                }
            }
        }
    }
    Ok((1..=n).map(|i| Complex::new(wr[i], wi[i])).collect())
}

/// All eigenvalues of the real `n × n` row-major matrix `a`, in no
/// particular order. Complex eigenvalues come in conjugate pairs.
pub fn eigenvalues(a: &[f64], n: usize) -> Result<Vec<Complex>, EigenError> {
    assert_eq!(a.len(), n * n, "eigenvalues: matrix is not n x n");
    if n == 0 {
        return Ok(Vec::new());
    }
    let mut m = Mat1::from_row_major(a, n);
    let _ = balance(&mut m);
    hessenberg(&mut m);
    hqr(&mut m)
}

/// Solve `M·x = b` in place for the complex matrix `m` (row-major, `n × n`),
/// by LU with partial pivoting. A pivot below `tiny` is replaced by `tiny`:
/// inverse iteration deliberately factors a (numerically) singular matrix.
pub(crate) fn lu_factor(m: &mut [Complex], n: usize, tiny: f64) -> Vec<usize> {
    let mut piv: Vec<usize> = (0..n).collect();
    for k in 0..n {
        let mut p = k;
        let mut best = m[k * n + k].abs();
        for i in (k + 1)..n {
            let v = m[i * n + k].abs();
            if v > best {
                best = v;
                p = i;
            }
        }
        if p != k {
            for j in 0..n {
                m.swap(k * n + j, p * n + j);
            }
            piv.swap(k, p);
        }
        if m[k * n + k].abs() < tiny {
            m[k * n + k] = Complex::real(tiny);
        }
        let d = m[k * n + k];
        for i in (k + 1)..n {
            let f = m[i * n + k] / d;
            m[i * n + k] = f;
            if f != Complex::ZERO {
                for j in (k + 1)..n {
                    let v = f * m[k * n + j];
                    m[i * n + j] = m[i * n + j] - v;
                }
            }
        }
    }
    piv
}

pub(crate) fn lu_solve(lu: &[Complex], piv: &[usize], n: usize, b: &[Complex]) -> Vec<Complex> {
    let mut x: Vec<Complex> = piv.iter().map(|&p| b[p]).collect();
    for i in 0..n {
        let mut s = x[i];
        for j in 0..i {
            s = s - lu[i * n + j] * x[j];
        }
        x[i] = s;
    }
    for i in (0..n).rev() {
        let mut s = x[i];
        for j in (i + 1)..n {
            s = s - lu[i * n + j] * x[j];
        }
        x[i] = s / lu[i * n + i];
    }
    x
}

fn normalize(v: &mut [Complex]) {
    let norm = v
        .iter()
        .map(|z| z.re * z.re + z.im * z.im)
        .sum::<f64>()
        .sqrt();
    if norm > 0.0 {
        for z in v.iter_mut() {
            *z = *z * (1.0 / norm);
        }
    }
}

/// Unit eigenvector of the real matrix `a` (row-major, `n × n`) for the
/// computed eigenvalue `lambda`, by inverse iteration on `a − λ·I`.
///
/// `left = false` returns `r` with `A·r = λ·r`; `left = true` returns `l`
/// with `lᵀ·A = λ·lᵀ` (plain transpose, no conjugation), so that the modal
/// residue of pole `λ` from input `b` to output `c` is
/// `(c·r)·(l·b) / (l·r)` with unconjugated products.
pub fn eigenvector(a: &[f64], n: usize, lambda: Complex, left: bool) -> Vec<Complex> {
    assert_eq!(a.len(), n * n, "eigenvector: matrix is not n x n");
    // Iterate on the balanced matrix D⁻¹·A·D (a badly scaled matrix makes the
    // LU of A − λI lose the small components), then map back:
    // r = D·r_b, l = D⁻¹·l_b.
    let mut bal = Mat1::from_row_major(a, n);
    let d = balance(&mut bal);
    let ab: Vec<f64> = (0..n * n).map(|k| bal.get(k / n + 1, k % n + 1)).collect();
    let norm = ab
        .iter()
        .fold(0.0_f64, |m, v| m.max(v.abs()))
        .max(lambda.abs());
    let tiny = f64::EPSILON * norm.max(f64::MIN_POSITIVE);
    let mut m = vec![Complex::ZERO; n * n];
    for i in 0..n {
        for j in 0..n {
            let v = if left { ab[j * n + i] } else { ab[i * n + j] };
            m[i * n + j] = Complex::real(v);
        }
        m[i * n + i] = m[i * n + i] - lambda;
    }
    let piv = lu_factor(&mut m, n, tiny);
    // A start vector with no special structure, so it is not orthogonal to
    // the wanted vector by construction.
    let mut v: Vec<Complex> = (0..n)
        .map(|i| Complex::real(1.0 + ((i * 7919) % 101) as f64 / 101.0))
        .collect();
    normalize(&mut v);
    for _ in 0..4 {
        v = lu_solve(&m, &piv, n, &v);
        normalize(&mut v);
    }
    for (i, z) in v.iter_mut().enumerate() {
        *z = if left {
            *z * (1.0 / d[i + 1])
        } else {
            *z * d[i + 1]
        };
    }
    normalize(&mut v);
    v
}

/// Unconjugated dot product `Σ uᵢ·vᵢ`.
pub fn dot(u: &[Complex], v: &[Complex]) -> Complex {
    u.iter().zip(v).fold(Complex::ZERO, |s, (&a, &b)| s + a * b)
}

/// Modal residue of the pole `lambda` of `x_{k+1} = A·x_k + b·u_k`,
/// `y = c·x`: the coefficient of `λ^k` that pole contributes to the impulse
/// response, `(c·r)·(l·b)/(l·r)`.
pub fn modal_residue(a: &[f64], n: usize, lambda: Complex, b: &[f64], c: &[f64]) -> Complex {
    let r = eigenvector(a, n, lambda, false);
    let l = eigenvector(a, n, lambda, true);
    let bc: Vec<Complex> = b.iter().map(|&v| Complex::real(v)).collect();
    let cc: Vec<Complex> = c.iter().map(|&v| Complex::real(v)).collect();
    dot(&cc, &r) * dot(&l, &bc) / dot(&l, &r)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sorted(mut v: Vec<Complex>) -> Vec<Complex> {
        v.sort_by(|a, b| {
            a.re.partial_cmp(&b.re)
                .unwrap()
                .then(a.im.partial_cmp(&b.im).unwrap())
        });
        v
    }

    #[test]
    fn triangular_matrix_gives_its_diagonal() {
        let a = [2.0, 5.0, -1.0, 0.0, -0.5, 3.0, 0.0, 0.0, 7.0];
        let e = sorted(eigenvalues(&a, 3).unwrap());
        for (z, want) in e.iter().zip([-0.5, 2.0, 7.0]) {
            assert!(
                (z.re - want).abs() < 1e-14 && z.im == 0.0,
                "{z:?} vs {want}"
            );
        }
    }

    #[test]
    fn rotation_gives_a_conjugate_pair() {
        let (c, s) = (0.3_f64.cos(), 0.3_f64.sin());
        let a = [0.9 * c, -0.9 * s, 0.9 * s, 0.9 * c];
        let e = sorted(eigenvalues(&a, 2).unwrap());
        assert!((e[0].re - 0.9 * c).abs() < 1e-15 && (e[0].im + 0.9 * s).abs() < 1e-15);
        assert!((e[1].re - 0.9 * c).abs() < 1e-15 && (e[1].im - 0.9 * s).abs() < 1e-15);
    }

    #[test]
    fn residue_of_a_diagonal_system() {
        // x' = diag(-0.9, 0.5) x + b u, y = c x: residues are c_i b_i.
        let a = [-0.9, 0.0, 0.0, 0.5];
        let r = modal_residue(&a, 2, Complex::real(-0.9), &[2.0, 3.0], &[5.0, 7.0]);
        assert!((r.re - 10.0).abs() < 1e-12 && r.im.abs() < 1e-12, "{r:?}");
        let r = modal_residue(&a, 2, Complex::real(0.5), &[2.0, 3.0], &[5.0, 7.0]);
        assert!((r.re - 21.0).abs() < 1e-12 && r.im.abs() < 1e-12, "{r:?}");
    }

    #[test]
    fn residues_sum_to_the_impulse_response() {
        // For distinct poles, h[k] = sum_i res_i * lambda_i^k (k >= 0, with
        // h[0] = c.b). Check k = 0..6 on a non-normal matrix with a complex pair.
        let n = 4;
        let a = [
            0.2, 1.5, 0.0, -0.3, //
            -0.8, 0.1, 0.7, 0.0, //
            0.0, 0.4, -0.95, 2.0, //
            0.1, 0.0, 0.0, 0.6,
        ];
        let b = [1.0, -2.0, 0.5, 3.0];
        let c = [0.3, 0.0, -1.0, 2.0];
        let e = eigenvalues(&a, n).unwrap();
        let res: Vec<Complex> = e.iter().map(|&z| modal_residue(&a, n, z, &b, &c)).collect();
        let mut x = b.to_vec();
        for k in 0..7 {
            let h: f64 = c.iter().zip(&x).map(|(p, q)| p * q).sum();
            let mut sum = Complex::ZERO;
            for (z, r) in e.iter().zip(&res) {
                let mut zk = Complex::real(1.0);
                for _ in 0..k {
                    zk = zk * *z;
                }
                sum = sum + *r * zk;
            }
            assert!(
                (sum.re - h).abs() < 1e-12 * h.abs().max(1.0) && sum.im.abs() < 1e-12,
                "k={k}: {sum:?} vs {h}"
            );
            let mut y = vec![0.0; n];
            for i in 0..n {
                for j in 0..n {
                    y[i] += a[i * n + j] * x[j];
                }
            }
            x = y;
        }
    }
}
