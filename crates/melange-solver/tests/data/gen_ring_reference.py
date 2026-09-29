#!/usr/bin/env python3
"""Reference values for the eigen solver and the trapezoidal ring predicate.

Independent implementation: numpy/scipy (LAPACK dgeev with left and right
eigenvectors). The Rust side (`src/eigen.rs`, `src/codegen/ring.rs`) must
match these (`tests/eigen_reference_tests.rs`).

Usage:
  gen_ring_reference.py eigen OUT.json
      Synthetic fuzz matrices: dense random, clusters near -1, near-defective
      pairs, complex pairs near -1, badly scaled similarities.
  gen_ring_reference.py ring OUT.json SYSTEM.json...
      Predicate references for RingSystem JSON files (the serialized
      `ring::RingSystem` of a circuit).
"""
import json
import sys

import numpy as np
import scipy.linalg as sla


def residues(a, b, c):
    """Eigenvalues and the modal residue (c.r)(l.b)/(l.r) of every pole."""
    w, vl, vr = sla.eig(a, left=True, right=True)
    out = []
    for k in range(len(w)):
        r = vr[:, k]
        l = vl[:, k].conj()  # scipy: vl^H a = w vl^H; the plain left vector is conj(vl)
        out.append((c @ r) * (l @ b) / (l @ r))
    return w, np.array(out), condition(vl, vr)


def condition(vl, vr):
    """Eigenvalue condition numbers 1/|l^H r| (unit-norm vectors, LAPACK's 1/s)."""
    return [float(1.0 / abs(np.vdot(vl[:, k], vr[:, k]))) for k in range(vl.shape[1])]


def cplx(v):
    return [[float(z.real), float(z.imag)] for z in v]


def random_similarity(rng, n, cond_target):
    """A random matrix with singular values spread over [1, cond_target]."""
    u, _ = np.linalg.qr(rng.standard_normal((n, n)))
    v, _ = np.linalg.qr(rng.standard_normal((n, n)))
    s = np.logspace(0, np.log10(cond_target), n)
    return u @ np.diag(s) @ v.T


def with_spectrum(rng, blocks, cond_target):
    """A real matrix with the given real 1x1 / 2x2 blocks, by similarity."""
    n = sum(b.shape[0] for b in blocks)
    d = sla.block_diag(*blocks)
    t = random_similarity(rng, n, cond_target)
    return t @ d @ np.linalg.inv(t)


def rot(r, theta):
    return np.array([[r * np.cos(theta), -r * np.sin(theta)], [r * np.sin(theta), r * np.cos(theta)]])


def eigen_cases():
    rng = np.random.default_rng(20260929)
    cases = []
    for n in (3, 8, 20, 60, 120):
        cases.append((f"dense-{n}", rng.standard_normal((n, n))))
    for delta in (1e-3, 1e-5, 1e-7):
        lam = [-1 + delta, -1 + 2 * delta, -1 + 3.5 * delta, -0.5, 0.3, 0.9, 0.0, 0.0]
        blocks = [np.array([[x]]) for x in lam] + [rot(0.95, 2.9)]
        cases.append((f"cluster-near-minus-1-{delta:g}", with_spectrum(rng, blocks, 1e2)))
    for split in (1e-4, 1e-6):
        # A 2x2 block with eigenvalues -0.999 +/- split: near-defective.
        jb = np.array([[-0.999, 1.0], [split * split, -0.999]])
        blocks = [jb, np.array([[0.5]]), np.array([[-0.2]]), rot(0.9, 1.0)]
        cases.append((f"near-defective-{split:g}", with_spectrum(rng, blocks, 10.0)))
    for r in (0.99999, 0.9999):
        blocks = [rot(r, np.pi - 1e-3), rot(r, np.pi - 0.3), np.array([[-r]]), np.array([[0.2]])]
        cases.append((f"complex-near-minus-1-{r:g}", with_spectrum(rng, blocks, 30.0)))
    for n in (10, 40):
        a = rng.standard_normal((n, n))
        d = np.logspace(-6, 6, n)
        cases.append((f"badly-scaled-{n}", np.diag(d) @ a @ np.diag(1 / d)))
    # A charge-propagator-like case: many exact zeros and a stiff pair.
    blocks = [np.array([[0.0]])] * 6 + [np.array([[-0.99997]]), np.array([[-0.9999962]]), np.array([[0.999]]), rot(0.98, 0.1)]
    cases.append(("propagator-like", with_spectrum(rng, blocks, 1e3)))
    out = []
    for name, a in cases:
        n = a.shape[0]
        b = rng.standard_normal(n)
        c = rng.standard_normal(n)
        w, res, cond = residues(a, b, c)
        out.append({
            "norm": float(np.linalg.norm(a)),
            "cond": cond,
            "name": name,
            "n": n,
            "a": a.flatten().tolist(),
            "b": b.tolist(),
            "c": c.tolist(),
            "eigenvalues": cplx(w),
            "residues": cplx(res),
        })
    return out


# ---- ring predicate ----------------------------------------------------------

RING_PERSISTENCE_FACTOR = 1e-3
RING_PERSISTENCE_SECONDS = 0.01
PASSBAND_HZ = 1000.0


def l_diff(s, i):
    l_mag = s["l0"] * (1 - s["lair"])
    l_air = s["l0"] * s["lair"]
    u = np.clip(i / s["isat"], -40, 40)
    return max(l_mag / np.cosh(u) ** 2 + l_air, 1e-6 * s["l0"])


CHARGE_DEPENDENCY_TOL = 1e-12


def charge_basis(h):
    """Orthonormal basis of range(h): the charge rows less their exact
    dependencies, found by SVD of the row/column-equilibrated charge rows
    (rank is scale-invariant there; a global threshold on the graded h drops
    real small capacitors)."""
    n = h.shape[0]
    rows = [i for i in range(n) if np.abs(h[i]).max() > 0]
    m = h[rows].copy()
    rs = np.ones(len(rows))
    for _ in range(3):
        mx = np.abs(m).max(1)
        m /= mx[:, None]
        rs /= mx
        cm = np.abs(m).max(0)
        m /= np.where(cm > 0, cm, 1)[None, :]
    u, sv, _ = np.linalg.svd(m)
    full = np.zeros(len(rows))
    full[: len(sv)] = sv
    deps = u[:, full <= CHARGE_DEPENDENCY_TOL * sv.max()] * rs[:, None]
    touched = [k for k in range(len(rows)) if np.any(deps[k] != 0)]
    cols = []
    for k, i in enumerate(rows):
        if k not in touched:
            e = np.zeros(n)
            e[i] = 1.0
            cols.append(e)
    if touched:
        # complement of span(deps) within the touched rows only, so no
        # direction is mixed with a row of a different scale
        q, _ = np.linalg.qr(deps[touched], mode="complete")
        for comp in q[:, deps.shape[1]:].T:
            e = np.zeros(n)
            e[[rows[k] for k in touched]] = comp
            cols.append(e)
    return np.array(cols).T if cols else np.zeros((n, 0))


def ring_reference(sys_):
    n, m = sys_["n"], sys_["m"]
    g = np.array(sys_["g"]).reshape(n, n)
    c = np.array(sys_["c"]).reshape(n, n)
    if m:
        nv = np.array(sys_["n_v"]).reshape(m, n)
        ni = np.array(sys_["n_i"]).reshape(n, m)
        j = np.array(sys_["j_dev"]).reshape(m, m)
        g = g - ni @ j @ nv
    c = c.copy()
    for s in sys_["sat"]:
        r = s["row"]
        c[r, r] += l_diff(s, sys_["dc_op"][r]) - s["l0"]
    rate = sys_["rate"]
    a = 2 * rate
    h = a * c
    for r in sys_["zero_rows"]:
        h[r, :] = 0.0
    s_ = np.linalg.inv(g + a * c)
    ur = charge_basis(h)
    r = ur.shape[1]
    i_n = np.eye(n)
    p = np.vstack([
        np.hstack([s_ @ h, s_ @ ur]),
        np.hstack([ur.T @ h @ (s_ @ h - i_n), ur.T @ h @ s_ @ ur - np.eye(r)]),
    ])
    w, vl, vr = sla.eig(p, left=True, right=True)
    lasts = RING_PERSISTENCE_SECONDS * rate
    modes = []
    om = 2 * np.pi * PASSBAND_HZ
    for k in range(len(w)):
        z = w[k]
        if z.real >= 0 or abs(z) ** lasts < RING_PERSISTENCE_FACTOR or z.imag < 0:
            continue
        rv = vr[:, k]
        lv = vl[:, k].conj()
        best = 0.0
        for node, gin in sys_["inputs"]:
            b = np.zeros(n)
            b[node] = gin
            bb = np.concatenate([s_ @ b, ur.T @ h @ s_ @ b])
            for out in sys_["outputs"]:
                pb = abs(np.linalg.solve(g + 1j * om * c, b)[out])
                if not pb > 0:
                    continue
                res = abs(rv[out] * (lv @ bb) / (lv @ rv))
                best = max(best, res / pb)
        modes.append({"z": [float(z.real), float(z.imag)], "residue_rel": float(best)})
    modes.sort(key=lambda x: -x["residue_rel"])
    return {"rank_h": r, "eigenvalues": cplx(w), "cond": condition(vl, vr), "norm": float(np.linalg.norm(p)),
            "ring_modes": modes, "rho": float(max(abs(w)))}


def main():
    if sys.argv[1] == "eigen":
        json.dump({"generator": f"numpy {np.__version__}, scipy {sla.__name__} (LAPACK dgeev)", "cases": eigen_cases()}, open(sys.argv[2], "w"))
    elif sys.argv[1] == "ring":
        cases = []
        for f in sys.argv[3:]:
            sys_ = json.load(open(f))
            cases.append({"system": sys_, "reference": ring_reference(sys_)})
        json.dump({"generator": f"numpy {np.__version__} (LAPACK dgeev)", "cases": cases}, open(sys.argv[2], "w"))
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
