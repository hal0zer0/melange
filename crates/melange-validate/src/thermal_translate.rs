//! Device thermal keys in the ngspice reference.
//!
//! melange's diode and BJT self-heat through `RTH`/`CTH` (junction
//! temperature following dissipation), and `TAMB` sets the device's static
//! temperature (its `IS` and thermal voltage scaled from TNOM). ngspice's
//! diode and BJT have no `RTH`/`CTH`/`TAMB` ("unrecognized parameter -
//! ignored"). validate builds the melange side isothermal
//! (`ParseOptions::disable_self_heating`), so the reference drops the dynamic
//! keys and keeps the static temperature: each instance of a card whose `TAMB`
//! is not TNOM is placed at that temperature with ngspice's instance `temp=`.

use std::collections::HashMap;

use melange_solver::parser::{Element, Netlist, ParseOptions};

use crate::spice_runner::SpiceError;

/// Model-card keys ngspice's diode and BJT lack.
const THERMAL_KEYS: [&str; 3] = ["RTH", "CTH", "TAMB"];

fn is_diode_or_bjt(model_type: &str) -> bool {
    matches!(
        model_type.to_ascii_uppercase().as_str(),
        "D" | "NPN" | "PNP"
    )
}

/// `content` with diode and BJT cards stripped of `RTH`/`CTH`/`TAMB` and
/// their instances at `TAMB`. `source` is the pristine deck. Unchanged when
/// no `.model` line carries a thermal key.
pub(crate) fn translate_thermal_for_ngspice(
    content: &str,
    source: &str,
) -> Result<String, SpiceError> {
    let has_thermal = source.lines().any(|l| {
        let up = crate::deck_guard::strip_inline_comment(l).to_ascii_uppercase();
        THERMAL_KEYS.iter().any(|k| {
            up.split(|c: char| c.is_whitespace() || c == '(' || c == ',')
                .any(|t| t.starts_with(&format!("{k}=")))
        })
    });
    if !has_thermal {
        return Ok(content.to_string());
    }
    let err = |e: String| SpiceError::ParseError(format!("thermal translation: {e}"));
    let netlist = Netlist::parse_with_options(
        source,
        ParseOptions {
            disable_unit_variation: true,
            disable_self_heating: true,
        },
    )
    .map_err(|e| err(e.to_string()))?;

    // Card (upper-cased name) -> (regenerated card, TAMB if not TNOM).
    let mut cards: HashMap<String, (String, Option<f64>)> = HashMap::new();
    for m in netlist
        .models
        .iter()
        .filter(|m| is_diode_or_bjt(&m.model_type))
    {
        let thermal = |k: &str| THERMAL_KEYS.iter().any(|t| t.eq_ignore_ascii_case(k));
        if !m.params.iter().any(|(k, _)| thermal(k)) {
            continue;
        }
        let kept: Vec<String> = m
            .params
            .iter()
            .filter(|(k, _)| !thermal(k))
            .map(|(k, v)| format!("{k}={v:e}"))
            .collect();
        let tamb = m
            .params
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case("TAMB"))
            .map(|&(_, t)| t)
            .filter(|&t| t != melange_primitives::T_NOM);
        cards.insert(
            m.name.to_ascii_uppercase(),
            (
                format!(".model {} {}({})", m.name, m.model_type, kept.join(" ")),
                tamb,
            ),
        );
    }
    if cards.is_empty() {
        return Ok(content.to_string());
    }
    // Instance (upper-cased name) -> temperature in Celsius.
    let mut temps: HashMap<String, f64> = HashMap::new();
    for e in &netlist.elements {
        let (name, model) = match e {
            Element::Diode { name, model, .. } | Element::Bjt { name, model, .. } => (name, model),
            _ => continue,
        };
        if let Some((_, Some(tamb))) = cards.get(&model.to_ascii_uppercase()) {
            temps.insert(name.to_ascii_uppercase(), tamb - 273.15);
        }
    }

    let mut out = String::with_capacity(content.len() + 256);
    let mut skipping = false;
    for line in content.lines() {
        let t = crate::deck_guard::strip_inline_comment(line)
            .trim()
            .to_string();
        if skipping {
            if t.starts_with('+') {
                continue;
            }
            skipping = false;
        }
        let toks: Vec<&str> = t.split_whitespace().collect();
        if toks.len() >= 2 && toks[0].eq_ignore_ascii_case(".model") {
            let name = toks[1].split('(').next().unwrap_or(toks[1]);
            if let Some((card, _)) = cards.get(&name.to_ascii_uppercase()) {
                out.push_str(card);
                out.push('\n');
                skipping = true;
                continue;
            }
        }
        if let Some(c) = toks
            .first()
            .and_then(|n| temps.get(&n.to_ascii_uppercase()))
        {
            out.push_str(&format!("{t} temp={c}\n"));
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thermal_keys_leave_the_card_and_tamb_places_the_instance() {
        let deck = "t\nR1 in a 1k\nD1 a 0 DX\n.model DX D(IS=2.52e-9 N=1.752 RTH=50 CTH=2e-3\n\
                    + TAMB=300)\n.end\n";
        let out = translate_thermal_for_ngspice(deck, deck).unwrap();
        assert!(
            !out.contains("RTH") && !out.contains("CTH") && !out.contains("TAMB"),
            "{out}"
        );
        assert!(out.contains(".model DX D(IS=2.52e-9 N=1.752e0)"), "{out}");
        assert!(out.contains("D1 a 0 DX temp=26.850000000000023"), "{out}");
    }

    #[test]
    fn rth_alone_changes_only_the_card() {
        let deck = "t\nR1 in a 1k\nD1 a 0 DX\n.model DX D(IS=2.52e-9 RTH=500 CTH=2e-4)\n.end\n";
        let out = translate_thermal_for_ngspice(deck, deck).unwrap();
        assert!(out.contains(".model DX D(IS=2.52e-9)\n"), "{out}");
        assert!(out.contains("D1 a 0 DX\n"), "{out}");
    }

    #[test]
    fn a_deck_without_thermal_keys_is_unchanged() {
        let deck = "t\nD1 a 0 DX\n.model DX D(IS=1e-14)\n.end\n";
        assert_eq!(translate_thermal_for_ngspice(deck, deck).unwrap(), deck);
    }
}
