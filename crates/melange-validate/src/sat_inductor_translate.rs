//! Melange-only element parameters in the ngspice reference.
//!
//! ngspice's inductor is linear and its resistor has no flicker-noise
//! parameters, so a deck carrying melange's `ISAT=` (iron-core saturation) or a
//! resistor's `KF=`/`AF=` fails to simulate ("unknown parameter"). Each is
//! translated where ngspice has an equivalent, and otherwise stripped with a
//! stated notice of what the reference then does not model. Never silently.
//!
//! - A single saturating inductor becomes its own flux law, melange's
//!   `Φ(i) = L_mag·ISAT·tanh(i/ISAT) + L_air·i`, with the current as a state:
//!   a node `x` on a unit capacitor carries it, `C·dx/dt = v / L_diff(x)` with
//!   `L_diff = L_mag/cosh²(x/ISAT) + L_air`, and the branch draws `I = x`. At
//!   DC the capacitor is open, so `v = 0` (a short, as melange's DC operating
//!   point treats an inductor) and the circuit sets `x`. `ISAT` and the floor
//!   are melange's own resolved values (datasheet forms converted,
//!   `LAIR`/`CORE` default). Two other forms fail in ngspice, measured on a
//!   10 mH / 2 mA / 100 Ω RL at 5x ISAT: `ddt(Φ(i))` in a B source (timestep
//!   too small, a 7 kA spike) and a flux integrator with `I = ISAT·atanh(Φ/…)`
//!   (timestep too small at the knee). This one runs clean and peaks within
//!   1.3 % of melange at 48 kHz.
//! - A `K`-coupled saturating group (melange's shared-core T-model) has no
//!   ngspice primitive here: its saturation parameters are stripped and the
//!   reference models that core as LINEAR, which is said.
//! - A resistor's `KF=`/`AF=` set its flicker noise. `validate` renders
//!   noise-free on both sides, so stripping them leaves the reference
//!   unchanged, which is said.

use std::collections::{HashMap, HashSet};

use melange_solver::mna::MnaSystem;
use melange_solver::parser::{resolve_air_floor, Element, Netlist};

use crate::spice_runner::SpiceError;

/// Inductor-line keys that exist only in melange.
const SAT_KEYS: [&str; 6] = [
    "ISAT=",
    "ISAT_DROP=",
    "ISAT_BASIS=",
    "L_AT_IDC=",
    "LAIR=",
    "CORE=",
];
/// Resistor-line keys ngspice's resistor rejects.
const NOISE_KEYS: [&str; 2] = ["KF=", "AF="];

fn has_key(tok: &str, keys: &[&str]) -> bool {
    let up = tok.to_ascii_uppercase();
    keys.iter().any(|k| up.starts_with(k))
}

fn num(v: f64) -> String {
    format!("{v:.17e}")
}

/// Rewrite `content` for ngspice. `source` is the pristine deck melange parses
/// (the resolved `ISAT` and floors come from melange's own MNA build).
pub(crate) fn translate_melange_params_for_ngspice(
    content: &str,
    source: &str,
) -> Result<String, SpiceError> {
    let body = |l: &str| {
        crate::deck_guard::strip_inline_comment(l)
            .trim()
            .to_string()
    };
    let any_sat = content.lines().skip(1).any(|l| {
        let t = body(l);
        t.to_ascii_uppercase().starts_with('L')
            && t.split_whitespace().any(|x| has_key(x, &SAT_KEYS))
    });
    let any_noise = content.lines().skip(1).any(|l| {
        let t = body(l);
        t.to_ascii_uppercase().starts_with('R')
            && t.split_whitespace().any(|x| has_key(x, &NOISE_KEYS))
    });
    if !any_sat && !any_noise {
        return Ok(content.to_string());
    }

    // Single saturating inductors, by upper-cased name: (L0, L_air, ISAT).
    let mut single: HashMap<String, (f64, f64, f64)> = HashMap::new();
    let mut coupled: HashSet<String> = HashSet::new();
    if any_sat {
        let netlist = Netlist::parse(source)
            .map_err(|e| SpiceError::ParseError(format!("saturating-inductor translation: {e}")))?;
        let in_coupling: HashSet<String> = netlist
            .couplings
            .iter()
            .flat_map(|c| {
                [
                    c.inductor1_name.to_ascii_uppercase(),
                    c.inductor2_name.to_ascii_uppercase(),
                ]
            })
            .collect();
        let mna = MnaSystem::from_netlist(&netlist)
            .map_err(|e| SpiceError::ParseError(format!("saturating-inductor translation: {e}")))?;
        for e in &netlist.elements {
            let Element::Inductor {
                name,
                isat: Some(_),
                air_floor,
                ..
            } = e
            else {
                continue;
            };
            let up = name.to_ascii_uppercase();
            if in_coupling.contains(&up) {
                coupled.insert(up);
                continue;
            }
            let Some(ind) = mna
                .inductors
                .iter()
                .find(|i| i.name.eq_ignore_ascii_case(name))
            else {
                return Err(SpiceError::ParseError(format!(
                    "saturating-inductor translation: {name} is not in melange's inductor set"
                )));
            };
            let Some(isat) = ind.isat else { continue };
            let frac = resolve_air_floor(*air_floor).0;
            single.insert(up, (ind.value, frac * ind.value, isat));
        }
        // Every inductor of a group that contains a saturating one shares its
        // core; strip the saturation keys from all of them.
        for c in &netlist.couplings {
            let (a, b) = (
                c.inductor1_name.to_ascii_uppercase(),
                c.inductor2_name.to_ascii_uppercase(),
            );
            if coupled.contains(&a) || coupled.contains(&b) {
                coupled.insert(a);
                coupled.insert(b);
            }
        }
    }

    let mut out = String::with_capacity(content.len() + 512);
    for (i, line) in content.lines().enumerate() {
        let t = body(line);
        let toks: Vec<&str> = t.split_whitespace().collect();
        let name_up = toks
            .first()
            .map(|n| n.to_ascii_uppercase())
            .unwrap_or_default();
        if i > 0 && toks.len() >= 4 {
            if let Some(&(l0, l_air, isat)) = single.get(&name_up) {
                let (name, np, nm) = (toks[0], toks[1], toks[2]);
                let l_mag = l0 - l_air;
                log::warn!(
                    "validate: {name} (ISAT={isat:e}) is a saturating inductor; the ngspice \
                     reference carries melange's flux law as a current-state integrator \
                     (dx/dt = v / (L_mag/cosh^2(x/ISAT) + L_air))."
                );
                let x = format!("xsat_{name}");
                out.push_str(&format!(
                    "* {name}: melange saturating inductor (validate twin)\n"
                ));
                out.push_str(&format!(
                    "Bx_{name} 0 {x} I=(v({np})-v({nm}))/({}/cosh(v({x})/{})^2+{})\n",
                    num(l_mag),
                    num(isat),
                    num(l_air)
                ));
                out.push_str(&format!("Cx_{name} {x} 0 1\n"));
                out.push_str(&format!("Rx_{name} {x} 0 1e12\n"));
                out.push_str(&format!("Bi_{name} {np} {nm} I=v({x})\n"));
                continue;
            }
            let strip = if coupled.contains(&name_up) {
                Some(&SAT_KEYS[..])
            } else if name_up.starts_with('R') && toks.iter().any(|x| has_key(x, &NOISE_KEYS)) {
                Some(&NOISE_KEYS[..])
            } else {
                None
            };
            if let Some(keys) = strip {
                let kept: Vec<&str> = toks.iter().copied().filter(|x| !has_key(x, keys)).collect();
                if keys == &SAT_KEYS[..] {
                    log::warn!(
                        "validate: {} is K-coupled to a saturating core; the ngspice reference has \
                         no shared-core saturation, so it models this core as LINEAR. Differences \
                         past the knee are not a melange error.",
                        toks[0]
                    );
                } else {
                    log::warn!(
                        "validate: {}'s KF=/AF= (flicker noise) are stripped from the ngspice \
                         reference; validate renders noise-free on both sides, so the reference is \
                         unchanged.",
                        toks[0]
                    );
                }
                out.push_str(&kept.join(" "));
                out.push('\n');
                continue;
            }
        }
        out.push_str(line);
        out.push('\n');
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHOKE: &str =
        "choke\nVcc vcc 0 DC 12\nLp vcc drain 5 ISAT=20m ; bias choke\nRd drain 0 1k\n.end\n";

    #[test]
    fn a_single_saturating_inductor_becomes_its_flux_law() {
        let out = translate_melange_params_for_ngspice(CHOKE, CHOKE).unwrap();
        assert!(!out.to_ascii_uppercase().contains("ISAT="), "{out}");
        // Default floor: ungapped steel, 3e-4 of L0.
        let lair = num(3e-4 * 5.0);
        let lmag = num(5.0 - 3e-4 * 5.0);
        assert!(
            out.contains(&format!(
                "Bx_Lp 0 xsat_Lp I=(v(vcc)-v(drain))/({lmag}/cosh(v(xsat_Lp)/{})^2+{lair})",
                num(0.02)
            )),
            "{out}"
        );
        assert!(out.contains("Cx_Lp xsat_Lp 0 1\n"), "{out}");
        assert!(out.contains("Bi_Lp vcc drain I=v(xsat_Lp)\n"), "{out}");
    }

    #[test]
    fn an_explicit_floor_is_honoured() {
        let deck = CHOKE.replace("ISAT=20m", "ISAT=20m LAIR=0.01");
        let out = translate_melange_params_for_ngspice(&deck, &deck).unwrap();
        assert!(out.contains(&format!("^2+{})", num(0.05))), "{out}");
    }

    #[test]
    fn a_coupled_saturating_core_is_stripped_to_linear() {
        let deck = "xfmr\nVcc vcc 0 DC 12\nL_pri vcc c 1.9 ISAT=520m CORE=gapped\nL_sec o 0 5.36\n\
                    K1 L_pri L_sec 0.9999\nRc c 0 100\nRo o 0 10k\n.end\n";
        let out = translate_melange_params_for_ngspice(deck, deck).unwrap();
        assert!(out.contains("L_pri vcc c 1.9\n"), "{out}");
        assert!(out.contains("L_sec o 0 5.36\n"), "{out}");
        assert!(out.contains("K1 L_pri L_sec 0.9999"), "{out}");
    }

    #[test]
    fn resistor_flicker_parameters_are_stripped() {
        let deck = "r\nVcc vcc 0 DC 9\nR_bias vcc mid 10k kf=1e-11 af=1\nR2 mid 0 10k\n.end\n";
        let out = translate_melange_params_for_ngspice(deck, deck).unwrap();
        assert!(out.contains("R_bias vcc mid 10k\n"), "{out}");
        assert!(out.contains("R2 mid 0 10k\n"), "{out}");
    }

    #[test]
    fn a_deck_without_melange_parameters_is_unchanged() {
        let deck = "rc\nR1 in out 1k\nC1 out 0 1u\nL1 out 0 1m\n.end\n";
        assert_eq!(
            translate_melange_params_for_ngspice(deck, deck).unwrap(),
            deck
        );
    }
}
