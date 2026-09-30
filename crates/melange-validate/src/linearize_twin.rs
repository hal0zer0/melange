//! `.linearize`d devices in the ngspice reference, as melange built them.
//!
//! melange replaces a linearized triode or BJT with its small-signal model at
//! the DC operating point: the terminal-current Jacobian, a constant that
//! puts the operating point back, and the capacitances it keeps. The
//! reference used to run the full nonlinear device instead, so on a deck
//! whose linearized stage leaves its small-signal region (a cathode follower
//! cut off on every negative swing) validate compared two different circuits
//! and charged the difference to the solver. The reference now gets the same
//! model, as ngspice elements, and the report says so. Whether the
//! linearization is valid at the drive is a separate question, which the
//! build answers: a linearized device outside its region makes the sample
//! unsolved and the run is refused.
//!
//! Current directions are SPICE's: `G n+ n- nc+ nc- g` and `I n+ n- DC i`
//! draw their current out of `n+` and into `n-`.

use melange_solver::mna::MnaSystem;

use crate::ValidationError;

/// One linearized device, by node name, with the model melange stamped.
#[derive(Debug, Clone)]
pub enum LinearizedTwin {
    /// Plate current `ip_dc + gm·(vgk − vgk0) + gp·(vpk − vpk0)`, grid
    /// current `ig_dc`, and the inter-electrode capacitances.
    Triode {
        name: String,
        g: String,
        p: String,
        k: String,
        gm: f64,
        gp: f64,
        ip_dc: f64,
        ig_dc: f64,
        vgk0: f64,
        vpk0: f64,
        ccg: f64,
        cgp: f64,
        ccp: f64,
    },
    /// Collector and base currents linear in (vbe, vbc) about (vbe0, vbc0),
    /// and the junction capacitances at the operating point.
    Bjt {
        name: String,
        c: String,
        b: String,
        e: String,
        dic_dvbe: f64,
        dic_dvbc: f64,
        dib_dvbe: f64,
        dib_dvbc: f64,
        ic_dc: f64,
        ib_dc: f64,
        vbe0: f64,
        vbc0: f64,
        cbe: f64,
        cbc: f64,
    },
}

impl LinearizedTwin {
    fn name(&self) -> &str {
        match self {
            LinearizedTwin::Triode { name, .. } | LinearizedTwin::Bjt { name, .. } => name,
        }
    }
}

/// The linearized devices of a finished build, by node name (empty for a
/// node the build has no netlist name for).
pub fn linearized_twins(mna: &MnaSystem) -> Vec<LinearizedTwin> {
    let name = |idx: usize| -> String {
        if idx == 0 {
            return "0".to_string();
        }
        mna.node_map
            .iter()
            .find(|(_, &i)| i == idx)
            .map(|(n, _)| n.clone())
            .unwrap_or_default()
    };
    let triodes = mna
        .linearized_triodes
        .iter()
        .map(|t| LinearizedTwin::Triode {
            name: t.name.clone(),
            g: name(t.ng),
            p: name(t.np),
            k: name(t.nk),
            gm: t.gm,
            gp: t.gp,
            ip_dc: t.ip_dc,
            ig_dc: t.ig_dc,
            vgk0: t.vgk0,
            vpk0: t.vpk0,
            ccg: t.ccg,
            cgp: t.cgp,
            ccp: t.ccp,
        });
    let bjts = mna.linearized_bjts.iter().map(|b| LinearizedTwin::Bjt {
        name: b.name.clone(),
        c: name(b.nc),
        b: name(b.nb),
        e: name(b.ne),
        dic_dvbe: b.dic_dvbe,
        dic_dvbc: b.dic_dvbc,
        dib_dvbe: b.dib_dvbe,
        dib_dvbc: b.dib_dvbc,
        ic_dc: b.ic_dc,
        ib_dc: b.ib_dc,
        vbe0: b.vbe0,
        vbc0: b.vbc0,
        cbe: b.cbe,
        cbc: b.cbc,
    });
    triodes.chain(bjts).collect()
}

/// A capacitor line, or nothing for a zero capacitance.
fn cap(device: &str, label: &str, a: &str, b: &str, c: f64) -> String {
    if c > 0.0 {
        format!("C_mlin_{device}_{label} {a} {b} {c:e}\n")
    } else {
        String::new()
    }
}

/// The ngspice lines for one linearized device.
fn twin_lines(d: &LinearizedTwin) -> String {
    let mut out = format!(
        "* melange .linearize {}: small-signal model at its DC operating point\n",
        d.name()
    );
    match d {
        LinearizedTwin::Triode {
            name,
            g,
            p,
            k,
            gm,
            gp,
            ip_dc,
            ig_dc,
            vgk0,
            vpk0,
            ccg,
            cgp,
            ccp,
        } => {
            out.push_str(&cap(name, "ccg", k, g, *ccg));
            out.push_str(&cap(name, "cgp", g, p, *cgp));
            out.push_str(&cap(name, "ccp", k, p, *ccp));
            out.push_str(&format!("G_mlin_{name}_gm {p} {k} {g} {k} {gm:e}\n"));
            out.push_str(&format!("G_mlin_{name}_gp {p} {k} {p} {k} {gp:e}\n"));
            out.push_str(&format!(
                "I_mlin_{name}_p {p} {k} DC {:e}\n",
                ip_dc - gm * vgk0 - gp * vpk0
            ));
            if *ig_dc != 0.0 {
                out.push_str(&format!("I_mlin_{name}_g {g} {k} DC {ig_dc:e}\n"));
            }
        }
        LinearizedTwin::Bjt {
            name,
            c,
            b,
            e,
            dic_dvbe,
            dic_dvbc,
            dib_dvbe,
            dib_dvbc,
            ic_dc,
            ib_dc,
            vbe0,
            vbc0,
            cbe,
            cbc,
        } => {
            out.push_str(&cap(name, "cbe", b, e, *cbe));
            out.push_str(&cap(name, "cbc", b, c, *cbc));
            for (t, x, dvbe, dvbc, i0) in [
                ("c", c, dic_dvbe, dic_dvbc, ic_dc),
                ("b", b, dib_dvbe, dib_dvbc, ib_dc),
            ] {
                out.push_str(&format!("G_mlin_{name}_{t}be {x} {e} {b} {e} {dvbe:e}\n"));
                out.push_str(&format!("G_mlin_{name}_{t}bc {x} {e} {b} {c} {dvbc:e}\n"));
                out.push_str(&format!(
                    "I_mlin_{name}_{t} {x} {e} DC {:e}\n",
                    i0 - dvbe * vbe0 - dvbc * vbc0
                ));
            }
        }
    }
    out
}

/// `netlist` with each linearized device's element line replaced by the
/// small-signal model melange built. Unchanged when `devs` is empty.
///
/// Refuses a device it cannot place: one inside a subcircuit (its element
/// line is not the deck's own) or at a node with no netlist name.
pub fn with_linearized_devices(
    netlist: &str,
    devs: &[LinearizedTwin],
) -> Result<String, ValidationError> {
    if devs.is_empty() {
        return Ok(netlist.to_string());
    }
    let mut remaining: Vec<&LinearizedTwin> = devs.iter().collect();
    let mut out = String::with_capacity(netlist.len() + 512 * devs.len());
    let mut skipping = false;
    for (i, line) in netlist.lines().enumerate() {
        let t = crate::deck_guard::strip_inline_comment(line).trim();
        if skipping {
            if t.starts_with('+') {
                continue;
            }
            skipping = false;
        }
        let first = t.split_whitespace().next().unwrap_or("");
        if i > 0 {
            if let Some(pos) = remaining
                .iter()
                .position(|d| d.name().eq_ignore_ascii_case(first))
            {
                let d = remaining.remove(pos);
                out.push_str(&twin_lines(d));
                skipping = true;
                continue;
            }
        }
        out.push_str(line);
        out.push('\n');
    }
    if let Some(d) = remaining.first() {
        return Err(ValidationError::InvalidInput(format!(
            "melange linearized {} but its element line is not in the deck (a device inside a \
             subcircuit); the reference cannot carry its small-signal model",
            d.name()
        )));
    }
    let nameless = devs.iter().find(|d| match d {
        LinearizedTwin::Triode { g, p, k, .. } => [g, p, k].iter().any(|n| n.is_empty()),
        LinearizedTwin::Bjt { c, b, e, .. } => [c, b, e].iter().any(|n| n.is_empty()),
    });
    if let Some(d) = nameless {
        return Err(ValidationError::InvalidInput(format!(
            "melange linearized {} at a node with no netlist name; the reference cannot carry it",
            d.name()
        )));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_linearized_triode_becomes_its_norton_model() {
        let deck = "t\nT1 g p k TX\nRa vcc p 100k\n.end\n";
        let d = LinearizedTwin::Triode {
            name: "T1".into(),
            g: "g".into(),
            p: "p".into(),
            k: "k".into(),
            gm: 1e-3,
            gp: 1e-5,
            ip_dc: 1e-3,
            ig_dc: 0.0,
            vgk0: -1.5,
            vpk0: 150.0,
            ccg: 0.0,
            cgp: 1e-12,
            ccp: 0.0,
        };
        let out = with_linearized_devices(deck, &[d]).unwrap();
        assert!(!out.contains("T1 g p k TX"), "{out}");
        assert!(out.contains("G_mlin_T1_gm p k g k 1e-3\n"), "{out}");
        assert!(out.contains("G_mlin_T1_gp p k p k 1e-5\n"), "{out}");
        // 1e-3 - 1e-3*(-1.5) - 1e-5*150 = 1e-3
        assert!(out.contains("I_mlin_T1_p p k DC 1e-3\n"), "{out}");
        assert!(out.contains("C_mlin_T1_cgp g p 1e-12\n"), "{out}");
        assert!(out.contains("Ra vcc p 100k"), "{out}");
    }

    #[test]
    fn a_device_not_in_the_deck_text_is_refused() {
        let d = LinearizedTwin::Triode {
            name: "X1.T1".into(),
            g: "x1.g".into(),
            p: "x1.p".into(),
            k: "x1.k".into(),
            gm: 1e-3,
            gp: 1e-5,
            ip_dc: 1e-3,
            ig_dc: 0.0,
            vgk0: -1.5,
            vpk0: 150.0,
            ccg: 0.0,
            cgp: 0.0,
            ccp: 0.0,
        };
        assert!(with_linearized_devices("t\nX1 a b c STAGE\n.end\n", &[d]).is_err());
    }
}
