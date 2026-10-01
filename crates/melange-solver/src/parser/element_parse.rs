//! Element-line parsers.

use super::*;

impl Parser {
    /// Validate that two nodes are not the same (self-connected component).
    fn check_self_connection(&self, n1: &str, n2: &str, component: &str) -> Result<(), ParseError> {
        if n1.eq_ignore_ascii_case(n2) {
            return Err(self.error(format!(
                "Component '{}' has both terminals connected to the same node '{}'",
                component, n1
            )));
        }
        Ok(())
    }

    /// Validate minimum field count for a component line.
    pub(super) fn require_parts(
        &self,
        parts: &[&str],
        min: usize,
        description: &str,
    ) -> Result<(), ParseError> {
        if parts.len() < min {
            return Err(self.error(format!(
                "{} requires: {}",
                parts.first().unwrap_or(&"?"),
                description
            )));
        }
        Ok(())
    }

    /// Parse a component value and validate it is positive and finite.
    ///
    /// Used by R, C, L parsers which all require strictly positive values.
    pub(super) fn parse_positive_value(
        &self,
        raw: &str,
        component_type: &str,
    ) -> Result<f64, ParseError> {
        let value = parse_value(raw).map_err(|_| {
            self.error(format!(
                "Invalid {} value '{}'.{}",
                component_type,
                raw,
                explain_rejected_value(raw, component_type)
            ))
        })?;
        if value <= 0.0 || !value.is_finite() {
            return Err(self.error(format!(
                "{} value must be positive and finite, got {}",
                component_type, value
            )));
        }
        Ok(value)
    }

    pub(super) fn parse_element(&self, line: &str) -> Result<Element, ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return Err(self.error("Empty element line"));
        }

        let name = parts[0];
        let first_char = name.chars().next().unwrap_or(' ').to_ascii_uppercase();

        let mut elem = match first_char {
            'R' => self.parse_resistor(&parts),
            'C' => self.parse_capacitor(&parts),
            'L' => self.parse_inductor(&parts),
            'V' => self.parse_voltage_source(&parts),
            'I' => self.parse_current_source(&parts),
            'D' => self.parse_diode(&parts),
            'Q' => self.parse_bjt(&parts),
            'J' => self.parse_jfet(&parts),
            'M' => self.parse_mosfet(&parts),
            'T' => self.parse_triode(&parts),
            'P' => self.parse_pentode(&parts),
            'U' => self.parse_opamp(&parts),
            'Y' => self.parse_vca(&parts),
            'N' => self.parse_glow(&parts),
            'O' => self.parse_ldr(&parts),
            'E' => self.parse_vcvs(&parts),
            'G' => self.parse_vccs(&parts),
            'X' => self.parse_subckt_instance(&parts),
            // Behavioral source: parse from the RAW line — the `{expr}` body
            // contains spaces/commas/parens that `split_whitespace` shreds.
            'B' => self.parse_bsource(line),
            _ => Err(self.error(format!("Unknown element type: {}", first_char))),
        }?;

        // Normalize every node reference: fold to lowercase (SPICE/ngspice
        // node names are case-insensitive — `IN` and `in` are the same net)
        // and alias `gnd`/`ground` to "0". Done here, once, so mna.rs only
        // ever sees normalized names.
        normalize_element_nodes(&mut elem);

        // Re-check self-connection AFTER normalization: `R1 gnd 0 1k` passes
        // the pre-normalization check ("gnd" != "0") but is self-connected
        // once the alias fires.
        match &elem {
            Element::Resistor {
                name,
                n_plus,
                n_minus,
                ..
            }
            | Element::Capacitor {
                name,
                n_plus,
                n_minus,
                ..
            }
            | Element::Inductor {
                name,
                n_plus,
                n_minus,
                ..
            }
            | Element::VoltageSource {
                name,
                n_plus,
                n_minus,
                ..
            }
            | Element::CurrentSource {
                name,
                n_plus,
                n_minus,
                ..
            }
            | Element::Diode {
                name,
                n_plus,
                n_minus,
                ..
            }
            | Element::BSource {
                name,
                n_plus,
                n_minus,
                ..
            } => {
                self.check_self_connection(n_plus, n_minus, name)?;
            }
            Element::Vca {
                name,
                n_sig_p,
                n_sig_n,
                ..
            } => {
                self.check_self_connection(n_sig_p, n_sig_n, name)?;
            }
            _ => {}
        }

        Ok(elem)
    }

    /// Parse a behavioral source: `B<name> n+ n- V={expr}` / `I={expr}`.
    ///
    /// Operates on the raw line so the braced expression body survives. The
    /// `V`/`I` keyword may be written with or without spaces around `=`
    /// (`V={..}`, `V ={..}`, `V = {..}` all accepted).
    fn parse_bsource(&self, line: &str) -> Result<Element, ParseError> {
        let open = line.find('{').ok_or_else(|| {
            self.error("Behavioral source requires a braced expression: B<name> n+ n- V={expr}")
        })?;
        // Brace-match for the closing `}` (expressions don't nest braces, but
        // count defensively).
        let bytes = line.as_bytes();
        let mut depth = 0usize;
        let mut close = None;
        for (i, &b) in bytes.iter().enumerate().skip(open) {
            match b {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        close = Some(i);
                        break;
                    }
                }
                _ => {}
            }
        }
        let close = close.ok_or_else(|| self.error("Behavioral source: unterminated '{'"))?;
        let expr_str = &line[open + 1..close];

        // Head = "B<name> n+ n- V=" (or "... I ="). Strip the trailing '='.
        let head = line[..open].trim_end();
        let head = head.strip_suffix('=').ok_or_else(|| {
            self.error("Behavioral source: expected `V={expr}` or `I={expr}` (missing '=')")
        })?;
        let toks: Vec<&str> = head.split_whitespace().collect();
        if toks.len() != 4 {
            return Err(self.error(format!(
                "Behavioral source '{}': expected `B<name> n+ n- V={{expr}}` (got head `{}`)",
                toks.first().copied().unwrap_or("B?"),
                head.trim()
            )));
        }
        let name = toks[0];
        let n_plus = toks[1];
        let n_minus = toks[2];
        let kind = match toks[3].to_ascii_uppercase().as_str() {
            "V" => BSourceKind::Voltage,
            "I" => BSourceKind::Current,
            other => {
                return Err(self.error(format!(
                    "Behavioral source '{}': expected V or I before '=', got '{}'",
                    name, other
                )));
            }
        };
        self.check_self_connection(n_plus, n_minus, name)?;

        let expr = crate::expr::Expr::parse(expr_str)
            .map_err(|e| self.error(format!("Behavioral source '{}': {}", name, e)))?;

        Ok(Element::BSource {
            name: name.to_string(),
            n_plus: n_plus.to_string(),
            n_minus: n_minus.to_string(),
            kind,
            expr,
        })
    }

    fn parse_resistor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 4, "Rname n+ n- value")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        let value = self.parse_positive_value(parts[3], "Resistor")?;

        // Optional trailing KF=/AF= for Phase 3.5 resistor flicker (Hooge).
        // Both default to `None`; codegen emits no flicker source unless
        // `KF > 0`. Order-independent. AF defaults to 2.0 when KF is set
        // without an explicit AF.
        let mut kf: Option<f64> = None;
        let mut af: Option<f64> = None;
        for token in &parts[4..] {
            let t = token.trim();
            let Some(eq) = t.find('=') else {
                return Err(self.error(format!(
                    "Resistor '{}': unrecognized trailing token '{}' (expected KF=… or AF=…)",
                    parts[0], t
                )));
            };
            let key = t[..eq].to_ascii_uppercase();
            let val = parse_value(&t[eq + 1..]).map_err(|_| {
                self.error(format!("Resistor '{}': invalid '{}' value", parts[0], key))
            })?;
            match key.as_str() {
                "KF" => {
                    if !val.is_finite() || val < 0.0 {
                        return Err(self.error(format!(
                            "Resistor '{}': KF must be a finite non-negative number, got {}",
                            parts[0], val
                        )));
                    }
                    kf = Some(val);
                }
                "AF" => {
                    if !val.is_finite() || val <= 0.0 {
                        return Err(self.error(format!(
                            "Resistor '{}': AF must be a finite positive number, got {}",
                            parts[0], val
                        )));
                    }
                    af = Some(val);
                }
                _ => {
                    return Err(self.error(format!(
                        "Resistor '{}': unrecognized parameter '{}' (only KF and AF are supported)",
                        parts[0], key
                    )));
                }
            }
        }
        // KF=0 is equivalent to KF unset — a noise-source filter at codegen
        // time excludes both. Normalize here so downstream code only has to
        // check `kf.is_some()` to know the user opted in.
        if matches!(kf, Some(v) if v == 0.0) {
            kf = None;
            af = None;
        }
        // AF without KF is meaningless (the formula has no flicker term);
        // strip it to keep the IR signal clean and avoid surprising users
        // who set only AF and wonder why nothing happens.
        if kf.is_none() {
            af = None;
        }

        Ok(Element::Resistor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
            kf,
            af,
        })
    }

    fn parse_capacitor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 4, "Cname n+ n- value")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        let value = self.parse_positive_value(parts[3], "Capacitor")?;

        let mut ic = None;
        for p in &parts[4..] {
            if p.to_uppercase().starts_with("IC=") {
                ic = Some(parse_value(&p[3..]).map_err(|_| self.error("Invalid IC value"))?);
            } else {
                return Err(self.error(format!(
                    "Capacitor '{}': unrecognized trailing token '{}' (only IC=<value> is supported)",
                    parts[0], p
                )));
            }
        }

        Ok(Element::Capacitor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
            ic,
        })
    }

    fn parse_inductor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(
            parts,
            4,
            "Lname n+ n- value [ISAT=value [ISAT_DROP=d [ISAT_BASIS=incremental|apparent]] | \
             L_AT_IDC=L,I] [LAIR=fraction | CORE=gapped|steel|nickel] [TURNS=t] [LM=henries]",
        )?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        let value = self.parse_positive_value(parts[3], "Inductor")?;
        let mut isat = None;
        let mut lair = None;
        let mut core = None;
        let mut drop = None;
        let mut basis = None;
        let mut l_at_idc = None;
        let mut turns = None;
        let mut lm = None;
        // Keyword matched case-insensitively; the value keeps its case so SPICE
        // suffixes parse exactly as elsewhere.
        let key = |part: &str, k: &str| -> Option<String> {
            part.get(..k.len())
                .filter(|head| head.eq_ignore_ascii_case(k) && part.len() > k.len())
                .map(|_| part[k.len()..].to_string())
        };
        for &part in &parts[4..] {
            if let Some(stripped) = key(part, "ISAT=") {
                let v = parse_value(&stripped)
                    .map_err(|_| self.error(format!("Invalid ISAT value: {}", stripped)))?;
                if !(v > 0.0 && v.is_finite()) {
                    return Err(self.error("ISAT must be positive and finite"));
                }
                isat = Some(v);
            } else if let Some(stripped) = key(part, "LAIR=") {
                let v = parse_value(&stripped)
                    .map_err(|_| self.error(format!("Invalid LAIR value: {}", stripped)))?;
                if !(v.is_finite() && (0.0..1.0).contains(&v)) {
                    return Err(self.error(format!(
                        "Inductor '{}': LAIR is the air-core inductance as a fraction of the \
                         inductance (0 <= LAIR < 1), got {}",
                        parts[0], stripped
                    )));
                }
                lair = Some(v);
            } else if let Some(stripped) = key(part, "ISAT_DROP=") {
                let v = parse_value(&stripped)
                    .map_err(|_| self.error(format!("Invalid ISAT_DROP value: {}", stripped)))?;
                if !(v > 0.0 && v < 1.0) {
                    return Err(self.error(format!(
                        "Inductor '{}': ISAT_DROP is the fraction the inductance has fallen by \
                         at ISAT (0 < ISAT_DROP < 1), got {}",
                        parts[0], stripped
                    )));
                }
                drop = Some(v);
            } else if let Some(stripped) = key(part, "ISAT_BASIS=") {
                basis = Some(match stripped.to_ascii_uppercase().as_str() {
                    "INCREMENTAL" => IsatBasis::Incremental,
                    "APPARENT" => IsatBasis::Apparent,
                    _ => {
                        return Err(self.error(format!(
                            "Inductor '{}': ISAT_BASIS must be incremental or apparent, got '{}'",
                            parts[0], stripped
                        )))
                    }
                });
            } else if let Some(stripped) = key(part, "L_AT_IDC=") {
                let fields: Vec<&str> = stripped.split(',').collect();
                let parsed = match fields.as_slice() {
                    [l, i] => parse_value(l).ok().zip(parse_value(i).ok()),
                    _ => None,
                };
                let Some((l, i)) = parsed else {
                    return Err(self.error(format!(
                        "Inductor '{}': L_AT_IDC takes <inductance>,<DC current> with no \
                         spaces (e.g. L_AT_IDC=17,100m), got '{}'",
                        parts[0], stripped
                    )));
                };
                if !(l > 0.0 && l < value && i > 0.0 && i.is_finite()) {
                    return Err(self.error(format!(
                        "Inductor '{}': L_AT_IDC needs 0 < L < the inductance ({}) and a \
                         positive current, got '{}'",
                        parts[0], value, stripped
                    )));
                }
                l_at_idc = Some((l, i));
            } else if let Some(stripped) = key(part, "TURNS=") {
                let v = parse_value(&stripped)
                    .map_err(|_| self.error(format!("Invalid TURNS value: {}", stripped)))?;
                if !(v > 0.0 && v.is_finite()) {
                    return Err(self.error(format!(
                        "Inductor '{}': TURNS is the winding's relative turns count, positive \
                         and finite, got {}",
                        parts[0], stripped
                    )));
                }
                turns = Some(v);
            } else if let Some(stripped) = key(part, "LM=") {
                let v = parse_value(&stripped)
                    .map_err(|_| self.error(format!("Invalid LM value: {}", stripped)))?;
                if !(v > 0.0 && v.is_finite()) {
                    return Err(self.error(format!(
                        "Inductor '{}': LM is the core's magnetizing inductance seen from this \
                         winding, positive and finite, got {}",
                        parts[0], stripped
                    )));
                }
                lm = Some(v);
            } else if let Some(stripped) = key(part, "CORE=") {
                core = Some(match stripped.to_ascii_uppercase().as_str() {
                    "GAPPED" => CoreClass::Gapped,
                    "STEEL" => CoreClass::Steel,
                    "NICKEL" => CoreClass::Nickel,
                    _ => {
                        return Err(self.error(format!(
                            "Inductor '{}': CORE must be gapped, steel or nickel, got '{}'",
                            parts[0], stripped
                        )))
                    }
                });
            } else {
                return Err(self.error(format!(
                    "Inductor '{}': unrecognized trailing token '{}' (supported: ISAT=, \
                     ISAT_DROP=, ISAT_BASIS=, L_AT_IDC=, LAIR=, CORE=, TURNS=, LM=)",
                    parts[0], part
                )));
            }
        }
        let air_floor = match (lair, core) {
            (Some(_), Some(_)) => {
                return Err(self.error(format!(
                "Inductor '{}': give LAIR= or CORE=, not both (CORE= picks a rule-of-thumb LAIR)",
                parts[0]
            )))
            }
            (Some(v), None) => Some(SatFloor::Explicit(v)),
            (None, Some(c)) => Some(SatFloor::Class(c)),
            (None, None) => None,
        };
        let isat_spec = match (l_at_idc, drop) {
            (Some(_), _) if isat.is_some() || drop.is_some() => {
                return Err(self.error(format!(
                    "Inductor '{}': L_AT_IDC= fixes the saturation current from the \
                     inductance itself; do not also give ISAT= or ISAT_DROP=",
                    parts[0]
                )))
            }
            (Some((l, i)), _) => {
                isat = Some(i);
                if basis.is_some() {
                    return Err(self.error(format!(
                        "Inductor '{}': L_AT_IDC= is an incremental rating; ISAT_BASIS= \
                         applies to ISAT_DROP= only",
                        parts[0]
                    )));
                }
                Some(IsatSpec::LAtIdc { l })
            }
            (None, Some(d)) => {
                if isat.is_none() {
                    return Err(self.error(format!(
                        "Inductor '{}': ISAT_DROP= describes the current given by ISAT= \
                         and needs it on the same line",
                        parts[0]
                    )));
                }
                Some(IsatSpec::Drop {
                    drop: d,
                    basis: basis.unwrap_or(IsatBasis::Incremental),
                })
            }
            (None, None) => {
                if basis.is_some() {
                    return Err(self.error(format!(
                        "Inductor '{}': ISAT_BASIS= needs ISAT_DROP=",
                        parts[0]
                    )));
                }
                None
            }
        };
        if air_floor.is_some() && isat.is_none() {
            return Err(self.error(format!(
                "Inductor '{}': LAIR= and CORE= describe saturation and need ISAT= (or \
                 L_AT_IDC=) on the same line",
                parts[0]
            )));
        }
        Ok(Element::Inductor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
            isat,
            isat_spec,
            air_floor,
            turns,
            lm,
        })
    }

    fn parse_voltage_source(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 3, "Vname n+ n-")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;

        let mut dc = None;
        let mut ac = None;

        // Parse DC and AC values
        let mut i = 3;
        while i < parts.len() {
            let part_upper = parts[i].to_uppercase();
            if part_upper == "DC" && i + 1 < parts.len() {
                dc = Some(
                    parse_value(parts[i + 1])
                        .map_err(|_| self.error(format!("Invalid DC value: {}", parts[i + 1])))?,
                );
                i += 2;
            } else if part_upper == "AC" && i + 1 < parts.len() {
                let mag = parse_value(parts[i + 1])
                    .map_err(|_| self.error(format!("Invalid AC magnitude: {}", parts[i + 1])))?;
                let (phase, consumed) = if i + 2 < parts.len() {
                    match parse_value(parts[i + 2]) {
                        Ok(v) => (v, 3),
                        Err(_) => {
                            // Check if the token is a keyword for the next iteration
                            let next_upper = parts[i + 2].to_uppercase();
                            if next_upper == "DC" || next_upper == "AC" {
                                (0.0, 2)
                            } else {
                                return Err(
                                    self.error(format!("Invalid AC phase: {}", parts[i + 2]))
                                );
                            }
                        }
                    }
                } else {
                    (0.0, 2)
                };
                ac = Some((mag, phase));
                i += consumed;
            } else if Self::is_transient_spec_token(parts[i]) {
                return Err(self.error(Self::transient_spec_message("Voltage", parts[0], parts[i])));
            } else if dc.is_none() {
                // Bare value is DC
                dc = Some(
                    parse_value(parts[i])
                        .map_err(|_| self.error(format!("Invalid DC value: {}", parts[i])))?,
                );
                i += 1;
            } else {
                // Previously silently skipped — a wrong circuit that solves
                // perfectly. Every token on a source line must be consumed.
                return Err(self.error(format!(
                    "Voltage source '{}': unexpected token '{}' (expected DC/AC specifications only)",
                    parts[0], parts[i]
                )));
            }
        }

        Ok(Element::VoltageSource {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            dc,
            ac,
        })
    }

    /// The refusal for a `SIN(...)`/`PULSE(...)`/... on a V or I source.
    ///
    /// Names both remedies, because the right one depends on what the line
    /// is: a test signal on the input must go entirely (a source left on the
    /// input node shorts it), a supply keeps its DC value.
    fn transient_spec_message(kind: &str, name: &str, token: &str) -> String {
        format!(
            "{kind} source '{name}': transient specification '{token}' (SIN/PULSE/PWL/EXP/...) \
             is not supported — melange has no time-domain sources; audio enters through the \
             input node (`-i`, default `in`), which melange drives itself. If this line is your \
             test signal, delete the whole line: a source left on the input node shorts it. If \
             it is a DC supply, keep only the DC value (`Vcc vcc 0 DC 9`)."
        )
    }

    /// Does this token open a SPICE transient specification (`SIN(...)`,
    /// `PULSE(...)`, `PWL(...)`, ...)? Used to give a targeted error instead
    /// of a generic "invalid value" / silent skip.
    fn is_transient_spec_token(tok: &str) -> bool {
        let upper = tok.to_ascii_uppercase();
        ["SIN", "SINE", "PULSE", "PWL", "EXP", "SFFM", "AM"]
            .iter()
            .any(|kw| upper == *kw || upper.starts_with(&format!("{}(", kw)))
    }

    fn parse_current_source(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 3, "Iname n+ n-")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;

        let (dc, consumed) = match parts.get(3) {
            Some(p) if p.to_uppercase() == "DC" => {
                let val_str = parts
                    .get(4)
                    .ok_or_else(|| self.error("DC keyword requires a value"))?;
                (
                    Some(
                        parse_value(val_str)
                            .map_err(|_| self.error(format!("Invalid DC value: {}", val_str)))?,
                    ),
                    5,
                )
            }
            Some(p) if Self::is_transient_spec_token(p) => {
                return Err(self.error(Self::transient_spec_message("Current", parts[0], p)));
            }
            Some(p) => (
                Some(parse_value(p).map_err(|_| self.error(format!("Invalid DC value: {}", p)))?),
                4,
            ),
            None => (None, 3),
        };

        // Previously tokens past the DC value were silently ignored — hard
        // error so SIN/PULSE specs (or plain junk) fail loudly.
        if parts.len() > consumed {
            let extra = &parts[consumed..];
            if extra.iter().any(|t| Self::is_transient_spec_token(t)) {
                return Err(self.error(Self::transient_spec_message(
                    "Current",
                    parts[0],
                    extra
                        .iter()
                        .find(|t| Self::is_transient_spec_token(t))
                        .unwrap(),
                )));
            }
            return Err(self.error(format!(
                "Current source '{}': unexpected trailing token(s): {}",
                parts[0],
                extra.join(" ")
            )));
        }

        Ok(Element::CurrentSource {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            dc,
        })
    }

    fn parse_diode(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 4, "Dname n+ n- modelname")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        if parts.len() > 4 {
            return Err(self.error(format!(
                "Diode '{}': unexpected trailing token(s): '{}' — area factors and instance \
                 parameters are not supported (scale IS in the .model card instead)",
                parts[0],
                parts[4..].join(" ")
            )));
        }
        Ok(Element::Diode {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            model: parts[3].to_string(),
        })
    }

    fn parse_bjt(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Qname nc nb ne [ns] modelname
        self.require_parts(parts, 5, "Qname nc nb ne modelname")?;
        // Model is always the last part (substrate node, if present, is skipped)
        if parts.len() > 6 {
            return Err(self.error(format!(
                "BJT '{}': unexpected trailing token(s): '{}' — expected 'Qname nc nb ne [ns] \
                 modelname'; area factors and instance parameters are not supported",
                parts[0],
                parts[6..].join(" ")
            )));
        }
        Ok(Element::Bjt {
            name: parts[0].to_string(),
            nc: parts[1].to_string(),
            nb: parts[2].to_string(),
            ne: parts[3].to_string(),
            model: parts[parts.len() - 1].to_string(),
        })
    }

    fn parse_jfet(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 5, "Jname nd ng ns modelname")?;
        if parts.len() > 5 {
            return Err(self.error(format!(
                "JFET '{}': unexpected trailing token(s): '{}' — area factors and instance \
                 parameters are not supported",
                parts[0],
                parts[5..].join(" ")
            )));
        }
        Ok(Element::Jfet {
            name: parts[0].to_string(),
            nd: parts[1].to_string(),
            ng: parts[2].to_string(),
            ns: parts[3].to_string(),
            model: parts[4].to_string(),
        })
    }

    fn parse_mosfet(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 6, "Mname nd ng ns nb modelname")?;
        if parts.len() > 6 {
            let extra = &parts[6..];
            let has_geometry = extra.iter().any(|t| {
                let upper = t.to_ascii_uppercase();
                matches!(
                    upper.split('=').next().unwrap_or(""),
                    "L" | "W" | "M" | "AD" | "AS" | "PD" | "PS" | "NRD" | "NRS"
                ) && upper.contains('=')
            });
            if has_geometry {
                return Err(self.error(format!(
                    "MOSFET '{}': instance geometry parameters ('{}') are not yet supported; \
                     fold W/L into KP or use a .model card",
                    parts[0],
                    extra.join(" ")
                )));
            }
            return Err(self.error(format!(
                "MOSFET '{}': unexpected trailing token(s): '{}'",
                parts[0],
                extra.join(" ")
            )));
        }
        Ok(Element::Mosfet {
            name: parts[0].to_string(),
            nd: parts[1].to_string(),
            ng: parts[2].to_string(),
            ns: parts[3].to_string(),
            nb: parts[4].to_string(),
            model: parts[5].to_string(),
        })
    }

    fn parse_opamp(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Uname n_plus n_minus n_out modelname
        self.require_parts(parts, 5, "Uname n_plus n_minus n_out modelname")?;
        if parts.len() > 5 {
            return Err(self.error(format!(
                "Op-amp '{}': unexpected trailing token(s): '{}' — expected \
                 'Uname n_plus n_minus n_out modelname'. There are no supply pins: \
                 the rails go on the model card (`.model {} OA(VCC=9 VEE=0)`).",
                parts[0],
                parts[5..].join(" "),
                parts[parts.len() - 1]
            )));
        }
        Ok(Element::Opamp {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            n_out: parts[3].to_string(),
            model: parts[4].to_string(),
        })
    }

    fn parse_triode(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Tname n_grid n_plate n_cathode modelname
        self.require_parts(parts, 5, "Tname n_grid n_plate n_cathode modelname")?;
        if parts.len() > 5 {
            return Err(self.error(format!(
                "Triode '{}': unexpected trailing token(s): '{}' — expected \
                 'Tname n_grid n_plate n_cathode modelname'",
                parts[0],
                parts[5..].join(" ")
            )));
        }
        Ok(Element::Triode {
            name: parts[0].to_string(),
            n_grid: parts[1].to_string(),
            n_plate: parts[2].to_string(),
            n_cathode: parts[3].to_string(),
            model: parts[4].to_string(),
        })
    }

    /// Parse a pentode (or beam tetrode) element line:
    ///
    /// ```spice
    /// Pname n_plate n_grid n_cathode n_screen modelname                      ; 4-terminal (suppressor → cathode)
    /// Pname n_plate n_grid n_cathode n_screen n_suppressor modelname         ; 5-terminal (explicit suppressor)
    /// ```
    ///
    /// **Node ordering is plate-first** (`P plate grid cathode screen …`),
    /// matching LTspice/PSpice/Ayumi convention and differing from the existing
    /// triode `T grid plate cathode …` order. This is intentional — documented
    /// in `docs/spice-grammar.md`.
    fn parse_pentode(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Minimum: Pname + 4 nodes + model = 6 parts
        match parts.len() {
            6 => Ok(Element::Pentode {
                name: parts[0].to_string(),
                n_plate: parts[1].to_string(),
                n_grid: parts[2].to_string(),
                n_cathode: parts[3].to_string(),
                n_screen: parts[4].to_string(),
                n_suppressor: None,
                model: parts[5].to_string(),
            }),
            7 => Ok(Element::Pentode {
                name: parts[0].to_string(),
                n_plate: parts[1].to_string(),
                n_grid: parts[2].to_string(),
                n_cathode: parts[3].to_string(),
                n_screen: parts[4].to_string(),
                n_suppressor: Some(parts[5].to_string()),
                model: parts[6].to_string(),
            }),
            _ => Err(self.error(format!(
                "Pentode '{}' requires 4 or 5 nodes: \
                 Pname n_plate n_grid n_cathode n_screen [n_suppressor] modelname",
                parts.first().copied().unwrap_or("")
            ))),
        }
    }

    fn parse_vca(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Yname sig+ sig- ctrl+ ctrl- modelname
        self.require_parts(parts, 6, "Yname sig+ sig- ctrl+ ctrl- modelname")?;
        // Validate signal pair not self-connected
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        if parts.len() > 6 {
            return Err(self.error(format!(
                "VCA '{}': unexpected trailing token(s): '{}' — expected \
                 'Yname sig+ sig- ctrl+ ctrl- modelname'",
                parts[0],
                parts[6..].join(" ")
            )));
        }
        Ok(Element::Vca {
            name: parts[0].to_string(),
            n_sig_p: parts[1].to_string(),
            n_sig_n: parts[2].to_string(),
            n_ctrl_p: parts[3].to_string(),
            n_ctrl_n: parts[4].to_string(),
            model: parts[5].to_string(),
        })
    }

    fn parse_glow(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Nname a k modelname (EXPERIMENTAL glow-discharge / neon lamp)
        self.require_parts(parts, 4, "Nname a k modelname")?;
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        if parts.len() > 4 {
            return Err(self.error(format!(
                "NEON glow '{}': unexpected trailing token(s): '{}' — expected \
                 'Nname a k modelname'",
                parts[0],
                parts[4..].join(" ")
            )));
        }
        Ok(Element::Glow {
            name: parts[0].to_string(),
            n_anode: parts[1].to_string(),
            n_cathode: parts[2].to_string(),
            model: parts[3].to_string(),
        })
    }

    fn parse_ldr(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Oname r+ r- ctrl+ ctrl- modelname
        self.require_parts(parts, 6, "Oname r+ r- ctrl+ ctrl- modelname")?;
        // The resistance path must not be self-connected (both terminals same
        // net → zero-length short with no defined current).
        self.check_self_connection(parts[1], parts[2], parts[0])?;
        if parts.len() > 6 {
            return Err(self.error(format!(
                "LDR '{}': unexpected trailing token(s): '{}' — expected \
                 'Oname r+ r- ctrl+ ctrl- modelname'",
                parts[0],
                parts[6..].join(" ")
            )));
        }
        Ok(Element::Ldr {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            n_ctrl_p: parts[3].to_string(),
            n_ctrl_n: parts[4].to_string(),
            model: parts[5].to_string(),
        })
    }

    fn parse_vcvs(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Ename out+ out- ctrl+ ctrl- gain
        self.require_parts(parts, 6, "Ename out+ out- ctrl+ ctrl- gain")?;
        if parts.len() > 6 {
            return Err(self.error(format!(
                "VCVS '{}': unexpected trailing token(s): '{}'",
                parts[0],
                parts[6..].join(" ")
            )));
        }
        let gain = parse_value(parts[5])
            .map_err(|_| self.error(format!("Invalid VCVS gain: {}", parts[5])))?;
        if !gain.is_finite() || gain == 0.0 {
            return Err(self.error(format!(
                "VCVS gain must be finite and non-zero, got {}",
                gain
            )));
        }
        Ok(Element::Vcvs {
            name: parts[0].to_string(),
            out_p: parts[1].to_string(),
            out_n: parts[2].to_string(),
            ctrl_p: parts[3].to_string(),
            ctrl_n: parts[4].to_string(),
            gain,
        })
    }

    fn parse_vccs(&self, parts: &[&str]) -> Result<Element, ParseError> {
        // Gname out+ out- ctrl+ ctrl- gm
        self.require_parts(parts, 6, "Gname out+ out- ctrl+ ctrl- gm")?;
        if parts.len() > 6 {
            return Err(self.error(format!(
                "VCCS '{}': unexpected trailing token(s): '{}'",
                parts[0],
                parts[6..].join(" ")
            )));
        }
        let gm = parse_value(parts[5])
            .map_err(|_| self.error(format!("Invalid VCCS transconductance: {}", parts[5])))?;
        if !gm.is_finite() || gm == 0.0 {
            return Err(self.error(format!(
                "VCCS transconductance must be finite and non-zero, got {}",
                gm
            )));
        }
        Ok(Element::Vccs {
            name: parts[0].to_string(),
            out_p: parts[1].to_string(),
            out_n: parts[2].to_string(),
            ctrl_p: parts[3].to_string(),
            ctrl_n: parts[4].to_string(),
            gm,
        })
    }

    fn parse_subckt_instance(&self, parts: &[&str]) -> Result<Element, ParseError> {
        self.require_parts(parts, 3, "Xname nodes... subcktname")?;
        Ok(Element::SubcktInstance {
            name: parts[0].to_string(),
            nodes: parts[1..parts.len() - 1]
                .iter()
                .map(|s| s.to_string())
                .collect(),
            subckt: parts[parts.len() - 1].to_string(),
        })
    }
}
