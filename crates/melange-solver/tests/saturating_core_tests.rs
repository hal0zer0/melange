//! A saturating shared core of any number of windings with a stated core:
//! `λ = Lm(φ)·n nᵀ·i + L_leak·i`, one saturable magnetizing term and a full
//! constant leakage matrix (SATURATING_TRANSFORMERS.md §2.2).
//!
//! The deck states the core with `TURNS=` on every winding and `LM=` on one;
//! `L_leak = L − LM·n nᵀ` must be positive-definite. Witnesses:
//! - (i) a two-winding core stated as `LM = k·L_ref`, `TURNS = √L` builds the
//!   same system as the implicit form;
//! - (ii) linear: the realized circuit's response equals ngspice's
//!   K-coupled `[L]` at 20 Hz, 1 kHz and 20 kHz, for 2, 3 and 4 windings,
//!   and melange's own exact `[L]` path;
//! - (iii) saturating, three windings: against an independent trapezoidal
//!   recurrence of the flux law at 1× (the implementation) and 1024× (the
//!   physics), C1-style; and a loaded core stays linear;
//! - (iv) the refusals;
//! - (v) zeroing the leakage's off-diagonal terms fails (ii);
//! - (vi) two windings with `LM ≠ k·L_ref` (a coupled leakage the implicit
//!   form cannot state): linear and saturating, as (ii) and (iii).
//!
//! The decks are built from a chosen split (LM, n, L_leak), so the ground
//! truth is known; their L and K values are that split's.

mod support;

use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const FS: f64 = 48000.0;
const F: f64 = 30.0;

/// Three windings: LM = 0.99 H on L1, n = (1, 0.5, 2), and
/// L_leak = [[0.01, 0.002, -0.003], [0.002, 0.004, 0.001], [-0.003, 0.001, 0.05]].
/// CORE=steel on L1 (1 H): L_air = 3e-4 H. The four-winding deck adds L4
/// (n = 0.25, leakage row [0.0005, 0.0002, -0.001, 0.002]).
fn w3(r2: &str, core: bool) -> String {
    let (k1, k2, k3) = if core {
        (
            " ISAT=10m CORE=steel TURNS=1 LM=0.99",
            " TURNS=0.5",
            " TURNS=2",
        )
    } else {
        ("", "", "")
    };
    format!(
        "three-winding core\nRp in p 99\nL1 p 0 1.0{k1}\nL2 s2 0 0.2515{k2}\nL3 s3 0 4.01{k3}\n\
         K12 L1 L2 0.991031352255357\nK13 L1 L3 0.9872666869807495\n\
         K23 L2 L3 0.9868075724724488\nR2 s2 0 {r2}\nR3 s3 0 2k\n"
    )
}

fn w4(core: bool) -> String {
    let k4 = if core { " TURNS=0.25" } else { "" };
    format!(
        "{}L4 s4 0 0.063875{k4}\nK14 L1 L4 0.981264809428674\nK24 L2 L4 0.9779397065905727\n\
         K34 L3 L4 0.9760887471710743\nR4 s4 0 10\n",
        w3("50", core).replacen("three-winding", "four-winding", 1)
    )
}

/// Two windings, LM = 0.97 H on L1 against k·L_ref = 0.995: the leakage
/// carries a coupling of its own, [[0.03, 0.0125], [0.0125, 0.0075]].
fn w2(core: bool) -> String {
    let (k1, k2) = if core {
        (" ISAT=10m CORE=steel TURNS=1 LM=0.97", " TURNS=0.5")
    } else {
        ("", "")
    };
    format!(
        "two-winding core, stated\nRp in p 99\nL1 p 0 1.0{k1}\nL2 s2 0 0.25{k2}\n\
         K12 L1 L2 0.995\nR2 s2 0 50\n"
    )
}

fn mna(spice: &str) -> MnaSystem {
    MnaSystem::from_netlist(&Netlist::parse(spice).expect("parse")).expect("must build")
}

fn mna_err(spice: &str) -> String {
    match MnaSystem::from_netlist(&Netlist::parse(spice).expect("parse")) {
        Ok(_) => panic!("expected a refusal for:\n{spice}"),
        Err(e) => e.to_string(),
    }
}

// ---------------------------------------------------------------- (i) ----

/// The implicit pair and the same core stated (`LM = k·L_ref` on the larger
/// winding, `TURNS = √L`) build the same system.
#[test]
fn a_stated_pair_equal_to_the_implicit_one_builds_the_same_system() {
    // n_sec = sqrt(0.25/1) = 0.5 against the primary; LM = k*L_ref = 0.99.
    let implicit_src = "pair\nRp in p 99\nL_pri p 0 1\nL_sec s 0 0.25 ISAT=5m CORE=steel\n\
                        K1 L_pri L_sec 0.99\nR_load s 0 1k\n";
    let stated_src = "pair\nRp in p 99\nL_pri p 0 1 TURNS=2 LM=0.99\n\
                      L_sec s 0 0.25 ISAT=5m CORE=steel TURNS=1\n\
                      K1 L_pri L_sec 0.99\nR_load s 0 1k\n";
    let (a, b) = (mna(implicit_src), mna(stated_src));

    assert_eq!(a.inductors.len(), b.inductors.len());
    for (x, y) in a.inductors.iter().zip(&b.inductors) {
        assert_eq!((&x.name, x.node_i, x.node_j), (&y.name, y.node_i, y.node_j));
        assert!(
            (x.value - y.value).abs() <= 1e-12 * x.value,
            "{}: {} vs {}",
            x.name,
            x.value,
            y.value
        );
        match (x.isat, y.isat) {
            (Some(p), Some(q)) => {
                assert!((p - q).abs() <= 1e-12 * p, "{}: ISAT {p} vs {q}", x.name)
            }
            (p, q) => assert_eq!(p, q),
        }
        let floor =
            |l: &melange_solver::mna::InductorElement| l.shared_core.as_ref().map(|c| c.floor_frac);
        match (floor(x), floor(y)) {
            (Some(p), Some(q)) => {
                assert!((p - q).abs() <= 1e-12 * p, "{}: floor {p} vs {q}", x.name)
            }
            (p, q) => assert_eq!(p, q),
        }
    }
    assert!(a.transformer_groups.is_empty() && b.transformer_groups.is_empty());
    assert_eq!(a.ideal_transformers.len(), b.ideal_transformers.len());
    for (x, y) in a.ideal_transformers.iter().zip(&b.ideal_transformers) {
        assert!((x.turns_ratio - y.turns_ratio).abs() <= 1e-15);
    }
    let (ga, gb) = (a.build_augmented_matrices(), b.build_augmented_matrices());
    assert_eq!(ga.n_nodal, gb.n_nodal);
    for i in 0..ga.n_nodal {
        for j in 0..ga.n_nodal {
            assert!((ga.g[i][j] - gb.g[i][j]).abs() <= 1e-12, "G[{i}][{j}]");
            assert!(
                (ga.c[i][j] - gb.c[i][j]).abs() <= 1e-12 * ga.c[i][i].abs().max(1.0),
                "C[{i}][{j}]"
            );
        }
    }
}

// ------------------------------------------------------- (ii), (v), (vi) --

/// Output voltage phasor for a 1 V phasor behind melange's 1 Ω input, from
/// the built system's own `G + jωC`.
fn ac(sys: &MnaSystem, out: &str, f: f64) -> (f64, f64) {
    let aug = sys.build_augmented_matrices();
    let n = aug.n_nodal;
    let w = 2.0 * std::f64::consts::PI * f;
    let (inp, o) = (sys.node_map["in"] - 1, sys.node_map[out] - 1);
    let mut a: Vec<Vec<(f64, f64)>> = (0..n)
        .map(|i| (0..n).map(|j| (aug.g[i][j], w * aug.c[i][j])).collect())
        .collect();
    let mut b = vec![(0.0, 0.0); n];
    a[inp][inp].0 += 1.0;
    b[inp] = (1.0, 0.0);
    let mul = |x: (f64, f64), y: (f64, f64)| (x.0 * y.0 - x.1 * y.1, x.0 * y.1 + x.1 * y.0);
    let div = |x: (f64, f64), y: (f64, f64)| {
        let d = y.0 * y.0 + y.1 * y.1;
        ((x.0 * y.0 + x.1 * y.1) / d, (x.1 * y.0 - x.0 * y.1) / d)
    };
    let abs = |x: (f64, f64)| x.0.hypot(x.1);
    for col in 0..n {
        let p = (col..n)
            .max_by(|&r, &s| abs(a[r][col]).total_cmp(&abs(a[s][col])))
            .unwrap();
        a.swap(col, p);
        b.swap(col, p);
        for r in col + 1..n {
            let m = div(a[r][col], a[col][col]);
            for c in col..n {
                let t = mul(m, a[col][c]);
                a[r][c] = (a[r][c].0 - t.0, a[r][c].1 - t.1);
            }
            let t = mul(m, b[col]);
            b[r] = (b[r].0 - t.0, b[r].1 - t.1);
        }
    }
    let mut x = vec![(0.0, 0.0); n];
    for r in (0..n).rev() {
        let mut s = b[r];
        for c in r + 1..n {
            let t = mul(a[r][c], x[c]);
            s = (s.0 - t.0, s.1 - t.1);
        }
        x[r] = div(s, a[r][r]);
    }
    x[o]
}

fn rel(z: (f64, f64), r: (f64, f64)) -> f64 {
    (z.0 - r.0).hypot(z.1 - r.1) / r.0.hypot(r.1)
}

/// ngspice `.ac` of the linear decks (K-coupled [L]), 1 V behind 1 Ω.
const NGSPICE: [(&str, f64, (f64, f64)); 9] = [
    ("w2", 20.0, (2.597168306787500e-01, 1.366988784542052e-01)),
    ("w2", 1000.0, (3.191173439830309e-01, -6.32827506006796e-02)),
    (
        "w2",
        20000.0,
        (1.797270408026228e-02, -7.50861422722616e-02),
    ),
    ("w3", 20.0, (9.590419005034183e-01, 4.431268929823795e-01)),
    ("w3", 1000.0, (1.057851892777955e+00, -3.24759515597993e-01)),
    (
        "w3",
        20000.0,
        (3.325870543838971e-02, -1.87783320217434e-01),
    ),
    ("w4", 20.0, (7.642092750180534e-01, 2.601672398008399e-01)),
    ("w4", 1000.0, (8.643876159184450e-01, -1.94961271111287e-01)),
    (
        "w4",
        20000.0,
        (2.915995611911293e-02, -1.59710451862213e-01),
    ),
];

fn deck(name: &str, core: bool) -> (String, &'static str) {
    match name {
        "w2" => (w2(core), "s2"),
        "w3" => (w3("50", core), "s3"),
        _ => (w4(core), "s3"),
    }
}

#[test]
fn a_stated_core_realizes_its_linear_inductance_matrix() {
    for &(name, f, reference) in &NGSPICE {
        let (stated, out) = deck(name, true);
        let (linear, _) = deck(name, false);
        let (sys, exact) = (mna(&stated), mna(&linear));
        assert!(
            sys.inductors.iter().any(|l| l.shared_core.is_some()),
            "{name}: not a core"
        );
        assert!(exact.inductors.iter().all(|l| l.shared_core.is_none()));
        let z = ac(&sys, out, f);
        assert!(
            rel(z, reference) < 1e-9,
            "{name} {f} Hz: {z:?} vs ngspice {reference:?}"
        );
        let e = ac(&exact, out, f);
        assert!(
            rel(z, e) < 1e-10,
            "{name} {f} Hz: {z:?} vs the exact [L] path {e:?}"
        );
    }
}

/// The off-diagonal leakage is load-bearing: without it the realized [L] is
/// wrong, and (ii) says so.
#[test]
fn dropping_the_leakage_coupling_breaks_the_realization() {
    for name in ["w2", "w3", "w4"] {
        let (stated, out) = deck(name, true);
        let mut sys = mna(&stated);
        assert_eq!(
            sys.transformer_groups.len(),
            1,
            "{name}: the leakage is a coupled group"
        );
        let g = &mut sys.transformer_groups[0];
        for i in 0..g.num_windings {
            for j in 0..g.num_windings {
                if i != j {
                    g.coupling_matrix[i][j] = 0.0;
                }
            }
        }
        let worst = NGSPICE
            .iter()
            .filter(|r| r.0 == name)
            .map(|&(_, f, reference)| rel(ac(&sys, out, f), reference))
            .fold(0.0_f64, f64::max);
        assert!(worst > 1e-3, "{name}: the mutant still matches ({worst:e})");
    }
}

// --------------------------------------------------------- (iii), (vi) ---

/// Independent reference: trapezoidal steps of `dλ/dt = u(i)` with
/// `λ = n·Φ(nᵀi) + L_leak·i`, `Φ(x) = L_mag·Isat·tanh(x/Isat) + L_air·x`,
/// `L_mag = LM − L_air`; `u_1 = V_s − R_1·i_1` (R_1 includes the 1 Ω input),
/// `u_k = −R_k·i_k`; Newton on the winding currents to 1e-15. Computed offline
/// (2026-09-29; the 1024× runs take about 15 CPU-minutes each, too long for a
/// test) at 1× (checks the implementation) and 1024× (≈continuous; checks
/// the physics). Per drive: H1 [V] and H3/H1 of `V(out) = −R_out·i_out` over
/// 1–2 s at 1× and at 1024×, then the peak magnetizing and primary winding
/// currents over Isat.
type Row = (f64, f64, f64, f64, f64, f64, f64);

/// Three windings, R = (100, 50, 2k), output V(s2).
const W3_REF: [Row; 3] = [
    (
        0.5,
        1.399338661e-1,
        5.864552047e-4,
        1.399338497e-1,
        5.864572774e-4,
        0.1505,
        0.2476,
    ),
    (
        2.0,
        5.542956206e-1,
        1.220991871e-2,
        5.542955270e-1,
        1.220996882e-2,
        0.6792,
        1.0084,
    ),
    (
        5.0,
        1.041616860e0,
        2.914757186e-1,
        1.041616167e0,
        2.914745637e-1,
        4.7547,
        4.7632,
    ),
];

/// Two windings with shared leakage, R = (100, 50), output V(s2).
const W2_REF: [Row; 3] = [
    (
        0.5,
        1.565259568e-1,
        8.127669301e-4,
        1.565259338e-1,
        8.127697247e-4,
        0.1684,
        0.2301,
    ),
    (
        2.0,
        6.164089416e-1,
        1.780892394e-2,
        6.164088002e-1,
        1.780899586e-2,
        0.7813,
        0.9631,
    ),
    (
        5.0,
        1.107666599e0,
        3.186510342e-1,
        1.107665724e0,
        3.186495427e-1,
        4.6948,
        4.7056,
    ),
];

/// The three-winding core with R2 = 1 Ω, at 5 V.
const W3_LOADED_REF: Row = (
    5.0,
    9.430557996e-2,
    2.733565724e-5,
    9.430558005e-2,
    2.733573784e-5,
    0.1182,
    4.8107,
);

/// melange at 48 kHz, trapezoidal (the shipped build may promote to backward
/// Euler; the flux law is checked on the trapezoidal one): H1 and H3/H1 of
/// V(`out`) over 1–2 s per drive, and the count of unclean samples.
fn melange(spice: &str, out: &str, drives: &[f64], tag: &str) -> Vec<(f64, f64, f64)> {
    let mut config = support::config_for_spice(spice, FS);
    config.output_nodes = vec![support::node_index(spice, out)];
    config.dc_block = false;
    config.force_trap = true;
    let code = support::generate_circuit_code_nodal(spice, &config).0;
    let bad = support::unsolved_expr(&code, "s");
    let drives: Vec<String> = drives.iter().map(|d| format!("{d:?}")).collect();
    let main = format!(
        "fn main() {{
    for amp in [{drives}] {{
        let mut s = CircuitState::default();
        s.set_sample_rate({FS:?});
        let n = (2.0 * {FS:?}) as usize;
        let mut ss: Vec<f64> = Vec::with_capacity(n / 2);
        for k in 1..=n {{
            let x = amp * (2.0 * std::f64::consts::PI * {F:?} * k as f64 / {FS:?}).sin();
            let _ = process_sample(x, &mut s);
            if k > n / 2 {{ ss.push(s.v_prev[OUTPUT_NODES[0]]); }}
        }}
        let mut h = [0.0f64; 4];
        for hh in [1usize, 3] {{
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (j, &v) in ss.iter().enumerate() {{
                let w = 2.0 * std::f64::consts::PI * hh as f64 * {F:?} * j as f64 / {FS:?};
                re += v * w.cos();
                im += v * w.sin();
            }}
            h[hh] = 2.0 * (re * re + im * im).sqrt() / ss.len() as f64;
        }}
        println!(\"{{}} {{}} {{}}\", h[1], h[3] / h[1], {bad});
    }}
}}",
        drives = drives.join(", ")
    );
    support::compile_and_run(&code, &main, tag)
        .stdout
        .lines()
        .map(|l| {
            let v: Vec<f64> = l.split_whitespace().map(|t| t.parse().unwrap()).collect();
            (v[0], v[1], v[2])
        })
        .collect()
}

/// melange against both references: H1 to 1e-5 relative and H3/H1 to 1e-4
/// absolute (C1's gates).
fn check(tag: &str, got: (f64, f64, f64), row: &Row) {
    let (amp, h1_1x, r3_1x, h1_c, r3_c, im, _) = *row;
    let (h1, r3, bad) = got;
    assert_eq!(bad, 0.0, "{tag} {amp} V: unclean samples");
    for (h1_ref, r3_ref, which) in [(h1_1x, r3_1x, "1x"), (h1_c, r3_c, "1024x")] {
        assert!(
            ((h1 - h1_ref) / h1_ref).abs() <= 1e-5 && (r3 - r3_ref).abs() <= 1e-4,
            "{tag} {amp} V vs {which}: melange H1 {h1:.9e} H3/H1 {r3:.9e}, reference \
             {h1_ref:.9e} {r3_ref:.9e} (i_mag/Isat {im})"
        );
    }
}

fn check_table(spice: &str, table: &[Row; 3], tag: &str) {
    // The top drive takes the core well past its knee.
    assert!(
        table[2].5 > 4.0,
        "{tag}: test premise, i_mag/Isat {}",
        table[2].5
    );
    let drives: Vec<f64> = table.iter().map(|r| r.0).collect();
    for (got, row) in melange(spice, "s2", &drives, tag).into_iter().zip(table) {
        check(tag, got, row);
    }
}

#[test]
fn three_winding_core_follows_the_flux_law() {
    check_table(&w3("50", true), &W3_REF, "core_w3");
}

#[test]
fn stated_pair_with_coupled_leakage_follows_the_flux_law() {
    check_table(&w2(true), &W2_REF, "core_w2");
}

/// Under a heavy load the winding MMFs cancel: the primary carries many
/// times Isat while the core stays linear.
#[test]
fn a_loaded_three_winding_core_stays_linear() {
    let row = W3_LOADED_REF;
    assert!(
        row.6 > 4.0 && row.5 < 0.5,
        "test premise: i1/Isat {}, i_mag/Isat {}",
        row.6,
        row.5
    );
    let got = melange(&w3("1", true), "s2", &[row.0], "core_w3_loaded")[0];
    assert!(
        got.1 <= 1e-3,
        "loaded H3/H1 {:.3e}: the core saturated on winding current",
        got.1
    );
    check("core_w3_loaded", got, &row);
}

// ---------------------------------------------------------------- (iv) ---

#[test]
fn three_windings_without_a_stated_core_are_refused_with_the_star_split() {
    let e = mna_err(&w3("50", false).replacen("L1 p 0 1.0", "L1 p 0 1.0 ISAT=10m", 1));
    assert!(
        e.contains("State the core") && e.contains("one core loop"),
        "{e}"
    );
    assert!(
        e.contains("star") && e.contains("TURNS=") && e.contains("LM="),
        "{e}"
    );
    let e = mna_err(&w4(false).replacen("L1 p 0 1.0", "L1 p 0 1.0 ISAT=10m", 1));
    assert!(e.contains("State the core") && !e.contains("star"), "{e}");
}

#[test]
fn an_incomplete_or_contradictory_core_is_refused() {
    let partial = w3("50", true).replace(" TURNS=2", "");
    assert!(
        mna_err(&partial).contains("not on L3"),
        "{}",
        mna_err(&partial)
    );
    let no_lm = w3("50", true).replace(" LM=0.99", "");
    assert!(mna_err(&no_lm).contains("no LM="));
    let two_lm = w3("50", true).replace(" TURNS=2", " TURNS=2 LM=3.96");
    assert!(mna_err(&two_lm).contains("one winding only"));
    let linear = w3("50", true).replace(" ISAT=10m CORE=steel", "");
    assert!(mna_err(&linear).contains("no winding carries ISAT"));
    let uncoupled = "lone\nR1 in a 100\nL1 a 0 1 ISAT=10m TURNS=1 LM=0.9\n";
    assert!(mna_err(uncoupled).contains("not K-coupled"));
}

#[test]
fn a_core_that_leaves_no_positive_leakage_is_refused() {
    // LM = 1 H on a 1 H winding coupled at 0.995: the common mode's leakage
    // is negative.
    let e = mna_err(&w2(true).replace("LM=0.97", "LM=1.0"));
    assert!(
        e.contains("not positive-definite") && e.contains("L1") && e.contains("L2"),
        "{e}"
    );
}

#[test]
fn air_floors_a_stated_core_cannot_hold_are_refused() {
    // LAIR is the winding's total air-core inductance; L1's leakage is 0.03.
    let e = mna_err(&w2(true).replace("CORE=steel", "LAIR=0.02"));
    assert!(e.contains("no magnetizing air floor"), "{e}");
    // A floor not below LM leaves nothing to saturate.
    let tiny = "tiny core\nRp in p 99\nL1 p 0 1.0 ISAT=10m CORE=gapped TURNS=1 LM=1e-4\n\
                L2 s2 0 0.25 TURNS=0.5\nK12 L1 L2 0.995\nR2 s2 0 50\n";
    assert!(
        mna_err(tiny).contains("nothing is left to saturate"),
        "{}",
        mna_err(tiny)
    );
}
