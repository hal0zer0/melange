//! Dot-command directive parsers.

use super::*;

impl Parser {
    /// Dispatch one dot-command line.
    ///
    /// Adding a melange-only (non-standard-SPICE) arm here REQUIRES adding the
    /// same name to `MELANGE_ONLY_DIRECTIVES`; the drift-guard test fails
    /// otherwise.
    pub(super) fn parse_directive(
        &mut self,
        line: &str,
        netlist: &mut Netlist,
    ) -> Result<(), ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return Ok(());
        }

        match parts[0].to_lowercase().as_str() {
            ".model" => {
                self.require_parts(&parts, 3, "name and type")?;
                if netlist.models.len() >= MAX_MODELS {
                    return Err(self.error(format!(
                        "too many .model directives: {} exceeds MAX_MODELS ({})",
                        netlist.models.len() + 1,
                        MAX_MODELS
                    )));
                }
                let model = self.parse_model(&parts)?;
                netlist.models.push(model);
            }
            ".param" => {
                let param = self.parse_param(&parts)?;
                netlist.params.push(param);
            }
            ".subckt" => {
                self.require_parts(&parts, 2, "a name")?;
                let subckt_name = parts[1].to_string();
                let subckt_nodes: Vec<String> =
                    parts[2..].iter().map(|s| normalize_node_name(s)).collect();
                let mut subckt_elements = Vec::new();

                // Collect elements until .ends
                let mut found_ends = false;
                while let Some(sub_line) = self.next_line() {
                    let sub_line = sub_line.trim().to_string();
                    if sub_line.is_empty() || sub_line.starts_with('*') {
                        continue;
                    }
                    if sub_line.to_lowercase().starts_with(".ends") {
                        found_ends = true;
                        break;
                    }
                    if sub_line.starts_with('.') {
                        // Nested directives inside subckt (e.g. .model)
                        let sub_parts: Vec<&str> = sub_line.split_whitespace().collect();
                        if sub_parts[0].to_lowercase() == ".model" {
                            if netlist.models.len() >= MAX_MODELS {
                                return Err(self.error(format!(
                                    "too many .model directives: {} exceeds MAX_MODELS ({})",
                                    netlist.models.len() + 1,
                                    MAX_MODELS
                                )));
                            }
                            let model = self.parse_model(&sub_parts)?;
                            netlist.models.push(model);
                        } else {
                            log::warn!(
                                "directive '{}' inside .subckt '{}' is ignored — only .model is \
                                 honored inside subcircuit bodies; move it to the top level",
                                sub_parts[0],
                                subckt_name
                            );
                        }
                        continue;
                    }
                    let elem = self.parse_element(&sub_line)?;
                    validate_element_node_lengths(&elem).map_err(|e| self.error(e))?;
                    subckt_elements.push(elem);
                }

                if !found_ends {
                    return Err(self.error(format!(
                        "Subcircuit '{}' missing .ends directive",
                        subckt_name
                    )));
                }

                netlist.subcircuits.push(Subcircuit {
                    name: subckt_name,
                    nodes: subckt_nodes,
                    elements: subckt_elements,
                });
            }
            ".pot" => {
                let pot = self.parse_pot_directive(&parts, netlist)?;
                netlist.pots.push(pot);
            }
            ".wiper" => {
                let wiper = self.parse_wiper_directive(&parts, netlist)?;
                netlist.wipers.push(wiper);
            }
            ".gang" => {
                let gang = self.parse_gang_directive(&parts)?;
                netlist.gangs.push(gang);
            }
            ".switch" => {
                let sw = self.parse_switch_directive(&parts, netlist)?;
                netlist.switches.push(sw);
            }
            ".delay_feedback" => {
                // .delay_feedback node1 node2 ... — node names to freeze during NR
                if parts.len() < 2 {
                    return Err(self.error(".delay_feedback requires at least one node name"));
                }
                for name in &parts[1..] {
                    netlist.delay_feedback_nodes.push(normalize_node_name(name));
                }
            }
            ".linearize" => {
                // .linearize Q9 — linearize BJT at DC operating point
                if parts.len() < 2 {
                    return Err(self.error(".linearize requires at least one device name"));
                }
                for name in &parts[1..] {
                    netlist.linearize_devices.push(name.to_ascii_uppercase());
                }
            }
            ".runtime" => {
                // Dispatch on target prefix: V → voltage source, R → resistor.
                let target = parts.get(1).copied().unwrap_or("");
                let first = target.chars().next().unwrap_or(' ').to_ascii_uppercase();
                match first {
                    'V' => {
                        let rt = self.parse_runtime_directive(&parts)?;
                        netlist.runtime_sources.push(rt);
                    }
                    'R' => {
                        let rr = self.parse_runtime_resistor_directive(&parts)?;
                        netlist.runtime_resistors.push(rr);
                    }
                    _ => {
                        // Non-R/V target → bare plugin-driven scalar param.
                        let rs = self.parse_runtime_scalar_directive(&parts)?;
                        netlist.runtime_scalars.push(rs);
                    }
                }
            }
            ".inject" => {
                let inj = self.parse_inject_directive(&parts)?;
                netlist.injections.push(inj);
            }
            ".tap" => {
                let tap = self.parse_tap_directive(&parts)?;
                netlist.taps.push(tap);
            }
            ".port" => {
                self.parse_port_directive(&parts, netlist)?;
            }
            ".input_impedance" => {
                self.parse_input_impedance_directive(&parts, netlist)?;
            }
            ".mismatch" => {
                let spec = self.parse_mismatch_directive(&parts)?;
                netlist.mismatch_specs.push(spec);
            }
            ".seed" => {
                if parts.len() < 2 {
                    return Err(self.error(".seed requires a u64 value"));
                }
                let v: u64 = parts[1].parse().map_err(|_| {
                    self.error(format!(".seed value '{}' is not a valid u64", parts[1]))
                })?;
                netlist.seed = Some(v);
            }
            ".tolerance" => {
                self.parse_tolerance_directive(&parts, netlist)?;
            }
            ".integrator" => {
                // .integrator trap | .integrator be — pin the integration scheme.
                self.require_parts(&parts, 2, "trap or be")?;
                let pref = match parts[1].to_lowercase().as_str() {
                    "trap" | "trapezoidal" => IntegratorPref::Trap,
                    "be" | "backward-euler" | "backward_euler" | "euler" => IntegratorPref::Be,
                    other => {
                        return Err(self.error(format!(
                            ".integrator value '{other}' must be 'trap' or 'be'"
                        )));
                    }
                };
                if let Some(prev) = netlist.integrator {
                    if prev != pref {
                        return Err(self.error(
                            "conflicting .integrator directives (both trap and be specified)",
                        ));
                    }
                }
                netlist.integrator = Some(pref);
            }
            ".oversampling" => {
                // .oversampling N — declare a recommended (accuracy-minimum)
                // oversampling factor, N in {1,2,4}. NOT a mandate: an explicit
                // `--oversampling` CLI flag always wins (see CLI resolution),
                // and validate ignores this entirely. See
                // `Netlist::recommended_oversampling`.
                self.require_parts(&parts, 2, "an oversampling factor (1, 2, or 4)")?;
                let n: usize = parts[1].parse().map_err(|_| {
                    self.error(format!(
                        ".oversampling value '{}' is not a valid integer (must be 1, 2, or 4)",
                        parts[1]
                    ))
                })?;
                if !matches!(n, 1 | 2 | 4) {
                    return Err(self.error(format!(".oversampling must be 1, 2, or 4, got {n}")));
                }
                if let Some(prev) = netlist.recommended_oversampling {
                    if prev != n {
                        return Err(self.error(format!(
                            "conflicting .oversampling directives ({prev} and {n})"
                        )));
                    }
                }
                netlist.recommended_oversampling = Some(n);
                // `.oversampling N allow=a,b[,c]`: the factors the generated
                // code can switch between at runtime (`set_oversampling`),
                // N the default. Without `allow=` the factor is fixed.
                for tok in &parts[2..] {
                    let Some(list) = tok
                        .split_once('=')
                        .filter(|(k, _)| k.eq_ignore_ascii_case("allow"))
                        .map(|(_, v)| v)
                    else {
                        return Err(self.error(format!(
                            ".oversampling: unexpected '{tok}' (the only option is \
                             allow=a,b[,c], the factors selectable at runtime)"
                        )));
                    };
                    let mut set = Vec::new();
                    for f in list.split(',') {
                        let f: usize = f.trim().parse().map_err(|_| {
                            self.error(format!(".oversampling allow= value '{f}' is not 1, 2 or 4"))
                        })?;
                        if !matches!(f, 1 | 2 | 4) {
                            return Err(self.error(format!(
                                ".oversampling allow= factors must be 1, 2 or 4, got {f}"
                            )));
                        }
                        if set.contains(&f) {
                            return Err(self.error(format!(".oversampling allow= lists {f} twice")));
                        }
                        set.push(f);
                    }
                    if set.len() < 2 {
                        return Err(self.error(
                            ".oversampling allow= needs at least two factors; a single \
                             factor is `.oversampling N` without allow=",
                        ));
                    }
                    set.sort_unstable();
                    if let Some(prev) = &netlist.oversampling_set {
                        if *prev != set {
                            return Err(self.error(format!(
                                "conflicting .oversampling allow= sets ({prev:?} and {set:?})"
                            )));
                        }
                    }
                    netlist.oversampling_set = Some(set);
                }
            }
            ".end" | ".ends" => {
                // End of netlist or subcircuit
            }
            other => {
                log::warn!("Unknown directive: {}", other);
            }
        }

        Ok(())
    }

    fn parse_tolerance_directive(
        &self,
        parts: &[&str],
        netlist: &mut Netlist,
    ) -> Result<(), ParseError> {
        if parts.len() < 2 {
            return Err(self.error(".tolerance requires at least one CLASS=TOL pair (R, C, or L)"));
        }
        for tok in &parts[1..] {
            let (k, v) = tok.split_once('=').ok_or_else(|| {
                self.error(format!(
                    ".tolerance entry '{tok}' must be CLASS=TOL (R, C, or L)"
                ))
            })?;
            let tol: f64 = v
                .parse()
                .map_err(|_| self.error(format!(".tolerance value '{v}' is not a valid number")))?;
            if !tol.is_finite() || !(0.0..1.0).contains(&tol) {
                return Err(self.error(format!(".tolerance must be in [0.0, 1.0), got {tol}")));
            }
            match k.to_ascii_uppercase().as_str() {
                "R" => netlist.tolerance_r = tol,
                "C" => netlist.tolerance_c = tol,
                "L" => netlist.tolerance_l = tol,
                other => {
                    return Err(
                        self.error(format!(".tolerance class '{other}' must be R, C, or L"))
                    );
                }
            }
        }
        Ok(())
    }

    fn parse_mismatch_directive(&self, parts: &[&str]) -> Result<MismatchSpec, ParseError> {
        if parts.len() < 3 {
            return Err(self.error(
                ".mismatch requires a device class (D/Q/J/M/T) and at least one PARAM=TOL pair",
            ));
        }
        let class_str = parts[1];
        if class_str.len() != 1 {
            return Err(self.error(format!(
                ".mismatch device class '{class_str}' must be a single character (D, Q, J, M, or T)"
            )));
        }
        let device_class = class_str.chars().next().unwrap().to_ascii_uppercase();
        if !matches!(device_class, 'D' | 'Q' | 'J' | 'M' | 'T') {
            return Err(self.error(format!(
                ".mismatch device class '{device_class}' is not one of D, Q, J, M, T"
            )));
        }
        let mut params = Vec::new();
        for tok in &parts[2..] {
            let (k, v) = tok.split_once('=').ok_or_else(|| {
                self.error(format!(
                    ".mismatch entry '{tok}' must be PARAM=TOL (e.g. IS=0.02)"
                ))
            })?;
            let tol: f64 = v.parse().map_err(|_| {
                self.error(format!(".mismatch tolerance '{v}' is not a valid number"))
            })?;
            if !tol.is_finite() || !(0.0..1.0).contains(&tol) {
                return Err(self.error(format!(
                    ".mismatch tolerance must be in [0.0, 1.0), got {tol}"
                )));
            }
            let key = k.to_ascii_uppercase();
            let accepted = mismatch_keys(device_class);
            if !accepted.contains(&key.as_str()) {
                // A key nothing reads would leave the deck unjittered while
                // its author believes it is jittered.
                return Err(self.error(format!(
                    ".mismatch {device_class}: unknown parameter '{k}'. Accepted for \
                     {device_class}: {}",
                    accepted.join(", ")
                )));
            }
            params.push((key, tol));
        }
        Ok(MismatchSpec {
            device_class,
            params,
        })
    }

    fn parse_model(&self, parts: &[&str]) -> Result<Model, ParseError> {
        let name = parts[1].to_string();
        let mut params = Vec::new();

        // Reconstruct everything after ".model NAME" to handle TYPE(PARAMS...) correctly
        // e.g. ".model D1N4148 D(IS=1e-15)" or ".model 2N2222 NPN(IS=1e-15 BF=200)".
        // Rejoin `KEY = VAL` / `KEY =VAL` / `KEY= VAL` token triples up front so
        // whitespace around '=' does not silently drop parameters.
        let rest = collapse_ws_around_eq(&parts[2..].join(" "));

        // Locate the parameter region. Two accepted grammars:
        //   .model NAME TYPE(K=V K=V ...)   — parenthesized (classic)
        //   .model NAME TYPE K=V K=V ...    — paren-less (ngspice-compatible)
        // Unbalanced parens are a HARD error — previously they silently
        // produced a model with zero parameters.
        let open = rest.find('(');
        let close = rest.rfind(')');
        let (model_type, params_region) = match (open, close) {
            (Some(o), Some(c)) if o < c => {
                let after = rest[c + 1..].trim();
                if !after.is_empty() {
                    return Err(self.error(format!(
                        ".model '{}': unexpected text after closing ')': '{}'",
                        name, after
                    )));
                }
                (
                    rest[..o].trim().to_ascii_uppercase(),
                    rest[o + 1..c].to_string(),
                )
            }
            (None, None) => {
                // Paren-less: first token is the type, the remainder must be
                // KEY=VAL pairs.
                let mut it = rest.split_whitespace();
                let type_tok = it.next().unwrap_or("");
                if type_tok.contains('=') {
                    return Err(self.error(format!(
                        ".model '{}': missing model type before parameter '{}'",
                        name, type_tok
                    )));
                }
                let region_start = rest.find(type_tok).map(|p| p + type_tok.len()).unwrap_or(0);
                (
                    type_tok.trim().to_ascii_uppercase(),
                    rest[region_start..].to_string(),
                )
            }
            _ => {
                return Err(self.error(format!(
                    ".model '{}': unbalanced parentheses in '{}'",
                    name, rest
                )));
            }
        };

        // Parse params. SPICE allows both space-separated and comma-separated:
        //   NPN(IS=1e-14 BF=200 RE=0.001)    — spaces only
        //   NPN(IS=1e-14,BF=200,RE=0.001)    — commas only
        //   NPN(IS=1e-14, BF=200, RE=0.001)  — commas + spaces
        // Anything in the params region that is not KEY=VAL is a HARD error —
        // previously such tokens were silently dropped.
        for token in params_region.split(|c: char| c.is_ascii_whitespace() || c == ',') {
            let token = token.trim();
            if token.is_empty() {
                continue;
            }
            let Some(eq_pos) = token.find('=') else {
                return Err(self.error(format!(
                    ".model '{}': unexpected token '{}' in parameter list (expected KEY=VAL)",
                    name, token
                )));
            };
            if params.len() >= MAX_MODEL_PARAMS {
                return Err(self.error(format!(
                    "too many parameters on .model '{}': {} exceeds MAX_MODEL_PARAMS ({})",
                    name,
                    params.len() + 1,
                    MAX_MODEL_PARAMS
                )));
            }
            let key = token[..eq_pos].to_ascii_uppercase();
            let value_str = &token[eq_pos + 1..];
            if key.is_empty() || value_str.is_empty() {
                return Err(self.error(format!(
                    ".model '{}': malformed parameter '{}' (expected KEY=VAL)",
                    name, token
                )));
            }
            // Model parameters are dimensionless context: a single trailing
            // 'f'/'F' after a digit is femto (IS=6.734f → 6.734e-15), matching
            // ngspice. Element positions keep the Farad reading for bare 'F'.
            let value = parse_value_model_param(value_str)
                .map_err(|_| self.error(format!("Invalid model parameter value: {}", value_str)))?;
            params.push((key, value));
        }

        Ok(Model {
            name,
            model_type,
            params,
        })
    }

    fn parse_param(&self, parts: &[&str]) -> Result<Parameter, ParseError> {
        self.require_parts(parts, 2, "name=value")?;
        // Accept `name=value`, `name = value`, `name =value`, `name= value`.
        let joined: String = parts[1..].concat();
        let eq_pos = joined
            .find('=')
            .ok_or_else(|| self.error("Parameter must be name=value format"))?;
        let name = joined[..eq_pos].to_string();
        let value_str = &joined[eq_pos + 1..];
        let value = parse_value(value_str)
            .map_err(|_| self.error(format!("Invalid parameter value: {}", value_str)))?;
        Ok(Parameter { name, value })
    }

    /// Parse a K (coupling) element: `K1 L1 L2 0.95`
    pub(super) fn parse_coupling(&self, line: &str) -> Result<CouplingDirective, ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 4 {
            return Err(self.error("Coupling element requires: Kname L1 L2 coupling_coeff"));
        }

        let name = parts[0].to_string();
        let inductor1_name = parts[1].to_string();
        let inductor2_name = parts[2].to_string();

        // Validate inductor names start with L/l
        if !inductor1_name.starts_with('L') && !inductor1_name.starts_with('l') {
            return Err(self.error(format!(
                "Coupling '{}': first reference must be an inductor (L), got '{}'",
                name, inductor1_name
            )));
        }
        if !inductor2_name.starts_with('L') && !inductor2_name.starts_with('l') {
            return Err(self.error(format!(
                "Coupling '{}': second reference must be an inductor (L), got '{}'",
                name, inductor2_name
            )));
        }

        // Reject self-coupling
        if inductor1_name.eq_ignore_ascii_case(&inductor2_name) {
            return Err(self.error(format!(
                "Coupling '{}': cannot couple an inductor to itself ('{}')",
                name, inductor1_name
            )));
        }

        let coupling = parse_value(parts[3])
            .map_err(|_| self.error(format!("Invalid coupling coefficient: {}", parts[3])))?;

        // Written so a NaN fails it too.
        if !(coupling > 0.0 && coupling < 1.0) {
            return Err(self.error(format!(
                "Coupling coefficient must be in (0, 1) exclusive, got {}",
                coupling
            )));
        }

        Ok(CouplingDirective {
            name,
            inductor1_name,
            inductor2_name,
            coupling,
        })
    }

    fn parse_pot_directive(
        &self,
        parts: &[&str],
        netlist: &Netlist,
    ) -> Result<PotDirective, ParseError> {
        // .pot Rname min max
        self.require_parts(parts, 4, ".pot Rname min_value max_value")?;

        let resistor_name = parts[1].to_string();

        // Check component type prefix: either starts with R, or for expanded subcircuit
        // names like "X1.R1", the part after the last dot starts with R.
        let base_name = resistor_name.rsplit('.').next().unwrap_or(&resistor_name);
        if !base_name.starts_with('R') && !base_name.starts_with('r') {
            return Err(self.error(format!(
                ".pot target must be a resistor (name starting with R), got '{}'",
                resistor_name
            )));
        }

        let min_value = self.parse_positive_value(parts[2], ".pot min")?;
        let max_value = self.parse_positive_value(parts[3], ".pot max")?;

        if min_value >= max_value {
            return Err(self.error(format!(
                ".pot min ({}) must be less than max ({})",
                min_value, max_value
            )));
        }
        if netlist
            .pots
            .iter()
            .any(|p| p.resistor_name.eq_ignore_ascii_case(&resistor_name))
        {
            return Err(self.error(format!(
                "Duplicate .pot directive for resistor '{}'",
                resistor_name
            )));
        }
        if netlist.pots.len() >= 64 {
            return Err(self.error("Maximum of 64 .pot directives supported"));
        }

        // Optional default value and/or quoted label:
        //   .pot Rname min max "Label"           — default = netlist nominal
        //   .pot Rname min max default "Label"   — explicit default
        let mut default_value = None;
        let mut label_start = 4;

        // A 5th token that starts like a number is the default; anything else
        // (a quoted or a bare word) starts the label.
        let looks_numeric = |t: &str| {
            t.chars()
                .next()
                .is_some_and(|c| c.is_ascii_digit() || c == '.' || c == '+' || c == '-')
        };
        if parts.len() > 4 && looks_numeric(parts[4]) {
            // 5th token is a number — the default value
            default_value = Some(self.parse_positive_value(parts[4], ".pot default")?);
            let dv = default_value.unwrap();
            if dv < min_value || dv > max_value {
                return Err(self.error(format!(
                    ".pot default ({}) must be between min ({}) and max ({})",
                    dv, min_value, max_value
                )));
            }
            label_start = 5;
        }

        // Label: quoted ("Tone") or bare (Tone) — a bare trailing token used
        // to be silently dropped.
        let label = if parts.len() > label_start {
            let rest = parts[label_start..].join(" ");
            let quoted = rest.len() >= 2 && rest.starts_with('"') && rest.ends_with('"');
            let bare_word = parts.len() == label_start + 1 && !rest.contains('"');
            if !quoted && !bare_word {
                return Err(self.error(format!(
                    ".pot {resistor_name}: cannot read '{rest}' as a label. The form is: \
                     .pot Rname min_value max_value [default] [\"Label\"] — a label with \
                     spaces must be quoted."
                )));
            }
            let trimmed = rest.trim_matches('"');
            if trimmed.is_empty() {
                None
            } else {
                Some(trimmed.to_string())
            }
        } else {
            None
        };

        // Resistor existence is validated after full parse (order-independent)
        Ok(PotDirective {
            resistor_name,
            min_value,
            max_value,
            default_value,
            label,
        })
    }

    fn parse_wiper_directive(
        &self,
        parts: &[&str],
        netlist: &Netlist,
    ) -> Result<WiperDirective, ParseError> {
        // .wiper R_cw R_ccw total_R [default_pos] ["Label"]
        self.require_parts(parts, 4, ".wiper R_cw R_ccw total_resistance")?;

        let resistor_cw = parts[1].to_string();
        let resistor_ccw = parts[2].to_string();

        // Both must be resistors
        if !resistor_cw.to_ascii_uppercase().starts_with('R') {
            return Err(self.error(format!(
                ".wiper CW leg must be a resistor (name starting with R), got '{}'",
                resistor_cw
            )));
        }
        if !resistor_ccw.to_ascii_uppercase().starts_with('R') {
            return Err(self.error(format!(
                ".wiper CCW leg must be a resistor (name starting with R), got '{}'",
                resistor_ccw
            )));
        }
        if resistor_cw.eq_ignore_ascii_case(&resistor_ccw) {
            return Err(self.error(".wiper CW and CCW legs must be different resistors"));
        }

        let total_resistance = self.parse_positive_value(parts[3], ".wiper total_resistance")?;
        if total_resistance <= 20.0 {
            return Err(self.error(format!(
                ".wiper total_resistance ({}) must be > 20 ohms",
                total_resistance
            )));
        }

        // Check that neither resistor is already claimed by a .pot or another .wiper
        let all_claimed: Vec<&str> = netlist
            .pots
            .iter()
            .map(|p| p.resistor_name.as_str())
            .chain(
                netlist
                    .wipers
                    .iter()
                    .flat_map(|w| [w.resistor_cw.as_str(), w.resistor_ccw.as_str()]),
            )
            .collect();
        for name in [&resistor_cw, &resistor_ccw] {
            if all_claimed.iter().any(|c| c.eq_ignore_ascii_case(name)) {
                return Err(self.error(format!(
                    ".wiper resistor '{}' is already used by a .pot or .wiper directive",
                    name
                )));
            }
        }

        // Each wiper adds 2 internal pots
        if netlist.pots.len() + (netlist.wipers.len() + 1) * 2 > 64 {
            return Err(self.error("Maximum of 64 combined .pot + .wiper leg entries supported"));
        }

        // Optional default position and label
        let mut default_position = None;
        let mut label_start = 4;

        if parts.len() > 4 && !parts[4].starts_with('"') {
            let pos: f64 = parts[4].parse().map_err(|_| {
                self.error(format!(
                    ".wiper default_position '{}' is not a valid number",
                    parts[4]
                ))
            })?;
            if !(0.0..=1.0).contains(&pos) {
                return Err(self.error(format!(
                    ".wiper default_position ({}) must be between 0.0 and 1.0",
                    pos
                )));
            }
            default_position = Some(pos);
            label_start = 5;
        }

        // Label: quoted or bare, same grammar as .pot (bare trailing tokens
        // used to be silently dropped).
        let label = if parts.len() > label_start {
            let rest = parts[label_start..].join(" ");
            let trimmed = rest.trim_matches('"');
            if trimmed.is_empty() {
                None
            } else {
                Some(trimmed.to_string())
            }
        } else {
            None
        };

        Ok(WiperDirective {
            resistor_cw,
            resistor_ccw,
            total_resistance,
            default_position,
            label,
        })
    }

    /// Expand `.wiper` directives into two `PotDirective` entries each.
    ///
    /// Called after parsing but before validation. Each wiper creates two pots
    /// with complementary default values and a min of 1Ω (wiper contact resistance).
    pub(super) fn expand_wipers(netlist: &mut Netlist) -> Result<(), ParseError> {
        /// Minimum resistance per wiper leg (models wiper contact resistance).
        /// Must be ≥10Ω for Sherman-Morrison numerical stability at extreme positions.
        /// Real pots have 1–50Ω contact resistance; 10Ω is conservative.
        const MIN_LEG_R: f64 = 10.0;

        // With no explicit default position, the knob starts where the netlist
        // puts it: the two legs' values, as for a `.pot`. (It started at 0.5
        // whatever the legs said.) Legs that do not add up to the total have no
        // single position, so that is refused rather than guessed.
        let resistance = |netlist: &Netlist, name: &str| {
            netlist.elements.iter().find_map(|e| match e {
                Element::Resistor { name: n, value, .. } if n.eq_ignore_ascii_case(name) => {
                    Some(*value)
                }
                _ => None,
            })
        };
        for i in 0..netlist.wipers.len() {
            let wiper = &netlist.wipers[i];
            if wiper.default_position.is_some() {
                continue;
            }
            let (Some(r_cw), Some(r_ccw)) = (
                resistance(netlist, &wiper.resistor_cw),
                resistance(netlist, &wiper.resistor_ccw),
            ) else {
                continue; // a missing leg is reported by validation
            };
            let r_total = wiper.total_resistance;
            if ((r_cw + r_ccw) - r_total).abs() > 0.01 * r_total {
                return Err(ParseError {
                    line: 0,
                    message: format!(
                        ".wiper {} {}: the legs are {} + {} = {} ohm, but the wiper's total is {} \
                         ohm, so they give no single default position. Make the legs add up \
                         to the total, or give the position explicitly: \
                         `.wiper {} {} {} <0..1>`.",
                        wiper.resistor_cw,
                        wiper.resistor_ccw,
                        r_cw,
                        r_ccw,
                        r_cw + r_ccw,
                        r_total,
                        wiper.resistor_cw,
                        wiper.resistor_ccw,
                        r_total,
                    ),
                });
            }
            // Inverse of the leg mapping below.
            let pos = ((r_ccw - MIN_LEG_R) / (r_total - 2.0 * MIN_LEG_R)).clamp(0.0, 1.0);
            netlist.wipers[i].default_position = Some(pos);
        }

        for wiper in &netlist.wipers {
            let pos = wiper.default_position.unwrap_or(0.5);
            let r_total = wiper.total_resistance;
            // pos=1.0 → wiper at CW end → R_cw≈0, R_ccw≈total
            let r_cw = (1.0 - pos) * (r_total - 2.0 * MIN_LEG_R) + MIN_LEG_R;
            let r_ccw = pos * (r_total - 2.0 * MIN_LEG_R) + MIN_LEG_R;

            netlist.pots.push(PotDirective {
                resistor_name: wiper.resistor_cw.clone(),
                min_value: MIN_LEG_R,
                max_value: r_total - MIN_LEG_R,
                default_value: Some(r_cw),
                label: None, // label lives on the wiper group
            });
            netlist.pots.push(PotDirective {
                resistor_name: wiper.resistor_ccw.clone(),
                min_value: MIN_LEG_R,
                max_value: r_total - MIN_LEG_R,
                default_value: Some(r_ccw),
                label: None,
            });
        }
        Ok(())
    }

    /// Parse a `.gang` directive: `.gang "Label" member1 [!]member2 ... [default]`
    ///
    /// Groups existing `.pot` and `.wiper` entries under a single UI parameter.
    /// Members are referenced by resistor name. Prefix with `!` to invert.
    /// Optional trailing float is the default position (0.0–1.0).
    fn parse_gang_directive(&self, parts: &[&str]) -> Result<GangDirective, ParseError> {
        // .gang "Label" member1 member2 ... [default_pos]
        if parts.len() < 4 {
            return Err(ParseError {
                line: self.line_num,
                message: ".gang requires at least a label and two member names".to_string(),
            });
        }

        // Parse label (must be quoted; may span multiple whitespace-separated
        // tokens, e.g. `.gang "Stereo Volume" R1 R2` — the same join-then-
        // unquote logic as .pot/.wiper/.switch labels).
        if !parts[1].starts_with('"') {
            return Err(ParseError {
                line: self.line_num,
                message: ".gang label must be a quoted string".to_string(),
            });
        }
        let (label, members_start) = if parts[1].len() >= 2 && parts[1].ends_with('"') {
            (parts[1][1..parts[1].len() - 1].to_string(), 2)
        } else {
            // Multi-token label: join tokens until one ends with '"'.
            let close = parts[2..].iter().position(|p| p.ends_with('"'));
            let Some(close) = close else {
                return Err(ParseError {
                    line: self.line_num,
                    message: ".gang label has an unterminated quote".to_string(),
                });
            };
            let joined = parts[1..=2 + close].join(" ");
            (joined.trim_matches('"').to_string(), 3 + close)
        };

        // Parse members and optional trailing default position
        let mut members = Vec::new();
        let mut default_position = None;

        for &part in &parts[members_start..] {
            // Try to parse as a float (default position) — only valid as last arg
            if let Ok(pos) = part.parse::<f64>() {
                if (0.0..=1.0).contains(&pos) {
                    default_position = Some(pos);
                    continue;
                }
            }

            // Parse as a member reference (optionally prefixed with !)
            let (inverted, name) = if let Some(stripped) = part.strip_prefix('!') {
                (true, stripped)
            } else {
                (false, part)
            };

            if name.is_empty() {
                return Err(ParseError {
                    line: self.line_num,
                    message: ".gang member name cannot be empty".to_string(),
                });
            }

            members.push(GangMember {
                resistor_name: name.to_ascii_uppercase(),
                inverted,
            });
        }

        if members.len() < 2 {
            return Err(ParseError {
                line: self.line_num,
                message: ".gang requires at least two members".to_string(),
            });
        }

        Ok(GangDirective {
            label,
            members,
            default_position,
        })
    }

    fn parse_switch_directive(
        &self,
        parts: &[&str],
        netlist: &Netlist,
    ) -> Result<SwitchDirective, ParseError> {
        // .switch C1,L1 val0a/val0b val1a/val1b ...
        // Minimum: .switch <names> <pos0> <pos1>  (at least 2 positions)
        self.require_parts(parts, 4, ".switch names pos0 pos1 [pos2 ...]")?;

        // Parse component names (comma-separated)
        let component_names: Vec<String> = parts[1]
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        if component_names.is_empty() {
            return Err(self.error(".switch requires at least one component name"));
        }

        // Validate component name prefixes: for expanded subcircuit names like "X1.C1",
        // check the part after the last dot.
        for name in &component_names {
            let base = name.rsplit('.').next().unwrap_or(name);
            let first = base.chars().next().unwrap_or(' ').to_ascii_uppercase();
            if !matches!(first, 'R' | 'C' | 'L') {
                return Err(self.error(format!(
                    ".switch component '{}' must start with R, C, or L",
                    name
                )));
            }
        }

        let num_comps = component_names.len();

        // Separate position values from optional trailing quoted label
        let value_parts = &parts[2..];
        let label_start = value_parts.iter().position(|p| p.starts_with('"'));
        let (pos_parts, label) = if let Some(idx) = label_start {
            let label_text = value_parts[idx..].join(" ");
            let trimmed = label_text.trim_matches('"');
            let label = if trimmed.is_empty() {
                None
            } else {
                Some(trimmed.to_string())
            };
            (&value_parts[..idx], label)
        } else {
            (value_parts, None)
        };

        // Parse position values
        let mut positions = Vec::new();
        for &pos_str in pos_parts {
            let values: Vec<f64> = pos_str
                .split('/')
                .map(|v| {
                    let val = parse_value(v.trim())
                        .map_err(|_| self.error(format!("Invalid .switch value: '{}'", v)))?;
                    if val <= 0.0 || !val.is_finite() {
                        return Err(self.error(format!(
                            ".switch value must be positive and finite, got {}",
                            val
                        )));
                    }
                    Ok(val)
                })
                .collect::<Result<Vec<_>, _>>()?;

            if values.len() != num_comps {
                return Err(self.error(format!(
                    ".switch position '{}' has {} values but {} components were specified",
                    pos_str,
                    values.len(),
                    num_comps
                )));
            }
            positions.push(values);
        }

        if positions.len() < 2 {
            return Err(self.error(".switch requires at least 2 positions"));
        }
        if positions.len() > 32 {
            return Err(self.error("Maximum of 32 positions per switch"));
        }

        // Check for duplicate component names across all switches
        for name in &component_names {
            if netlist.switches.iter().any(|sw| {
                sw.component_names
                    .iter()
                    .any(|n| n.eq_ignore_ascii_case(name))
            }) {
                return Err(self.error(format!(
                    "Component '{}' is already used in another .switch directive",
                    name
                )));
            }
        }

        if netlist.switches.len() >= 16 {
            return Err(self.error("Maximum of 16 .switch directives supported"));
        }

        Ok(SwitchDirective {
            component_names,
            positions,
            label,
        })
    }

    /// Parse `.runtime Vname as field_name`.
    ///
    /// `Vname` must reference a voltage source declared elsewhere in the
    /// netlist (validated later in the netlist-wide validation pass, so this
    /// parser doesn't require source-order). `field_name` must be a valid
    /// Rust identifier — this is where codegen will emit `pub <field>: f64`
    /// on the generated CircuitState.
    fn parse_runtime_directive(&self, parts: &[&str]) -> Result<RuntimeDirective, ParseError> {
        // .runtime Vname as field_name
        self.require_parts(parts, 4, ".runtime Vname as field_name")?;
        if !parts[2].eq_ignore_ascii_case("as") {
            return Err(self.error(format!(
                ".runtime expects 'as' between source name and field name, got '{}'",
                parts[2]
            )));
        }
        let vs_name = parts[1].to_string();
        let first_char = vs_name.chars().next().unwrap_or(' ').to_ascii_uppercase();
        if first_char != 'V' {
            return Err(self.error(format!(
                ".runtime target '{}' must be a voltage source (name starts with V)",
                vs_name
            )));
        }
        let field_name = parts[3].to_string();
        if !is_valid_rust_ident(&field_name) {
            return Err(self.error(format!(
                ".runtime field name '{}' is not a valid Rust identifier \
                 (must start with letter or _, contain only ASCII letters/digits/_)",
                field_name
            )));
        }
        Ok(RuntimeDirective {
            vs_name,
            field_name,
        })
    }

    /// Parse `.runtime Rname min max as field_name`.
    ///
    /// The resistor must already exist in the netlist (checked in the
    /// netlist-wide validation pass). `min` and `max` bound the audio-rate
    /// clamp applied by the generated setter. `field_name` names the
    /// `set_runtime_R_<field>` setter and `<field>()` getter emitted on
    /// `CircuitState`.
    fn parse_runtime_resistor_directive(
        &self,
        parts: &[&str],
    ) -> Result<RuntimeResistorDirective, ParseError> {
        // .runtime Rname min max as field_name
        self.require_parts(parts, 6, ".runtime Rname min max as field_name")?;
        if !parts[4].eq_ignore_ascii_case("as") {
            return Err(self.error(format!(
                ".runtime expects 'as' before field name, got '{}'",
                parts[4]
            )));
        }
        let resistor_name = parts[1].to_string();
        let base_name = resistor_name.rsplit('.').next().unwrap_or(&resistor_name);
        if !base_name.starts_with('R') && !base_name.starts_with('r') {
            return Err(self.error(format!(
                ".runtime R target '{}' must be a resistor (name starting with R)",
                resistor_name
            )));
        }
        let min_value = self.parse_positive_value(parts[2], ".runtime R min")?;
        let max_value = self.parse_positive_value(parts[3], ".runtime R max")?;
        if min_value >= max_value {
            return Err(self.error(format!(
                ".runtime R min ({}) must be less than max ({})",
                min_value, max_value
            )));
        }
        let field_name = parts[5].to_string();
        if !is_valid_rust_ident(&field_name) {
            return Err(self.error(format!(
                ".runtime R field name '{}' is not a valid Rust identifier \
                 (must start with letter or _, contain only ASCII letters/digits/_)",
                field_name
            )));
        }
        Ok(RuntimeResistorDirective {
            resistor_name,
            min_value,
            max_value,
            field_name,
        })
    }

    /// Parse a bare plugin-driven scalar param:
    /// `.runtime <name> <min> <max> as <field>`.
    fn parse_runtime_scalar_directive(
        &self,
        parts: &[&str],
    ) -> Result<RuntimeScalarDirective, ParseError> {
        self.require_parts(parts, 6, ".runtime name min max as field_name")?;
        if !parts[4].eq_ignore_ascii_case("as") {
            return Err(self.error(format!(
                ".runtime scalar expects 'as' before field name, got '{}'",
                parts[4]
            )));
        }
        let name = parts[1].to_string();
        if !is_valid_rust_ident(&name) {
            return Err(self.error(format!(
                ".runtime scalar name '{}' is not a valid identifier",
                name
            )));
        }
        let min_value =
            parse_value(parts[2]).map_err(|_| self.error(".runtime scalar: invalid min"))?;
        let max_value =
            parse_value(parts[3]).map_err(|_| self.error(".runtime scalar: invalid max"))?;
        if min_value >= max_value {
            return Err(self.error(format!(
                ".runtime scalar min ({}) must be less than max ({})",
                min_value, max_value
            )));
        }
        let field_name = parts[5].to_string();
        if !is_valid_rust_ident(&field_name) {
            return Err(self.error(format!(
                ".runtime scalar field name '{}' is not a valid Rust identifier",
                field_name
            )));
        }
        Ok(RuntimeScalarDirective {
            name,
            min_value,
            max_value,
            field_name,
        })
    }

    /// Parse `.inject <node> <field> R=<ohms>` (Thevenin) or
    /// `.inject <node> <field> RSHUNT=<ohms>` (Norton), optionally followed by
    /// `rate=host|inner` (default `host`).
    ///
    /// Impedance is MANDATORY — a directive with neither `R=` nor `RSHUNT=`
    /// is rejected (an ideal source would clamp the injection node and destroy
    /// the dry path; this is the single most important guardrail). Node
    /// existence is validated later against `node_map` at MNA/CLI resolution.
    /// Keys and the rate value are case-insensitive, like every other SPICE
    /// keyword; any other trailing token is refused.
    fn parse_inject_directive(&self, parts: &[&str]) -> Result<InjectDirective, ParseError> {
        // .inject <node> <field> R=<ohms>|RSHUNT=<ohms> [rate=host|inner]
        self.require_parts(
            parts,
            4,
            "<node> <field> and a mandatory impedance R=<ohms>|RSHUNT=<ohms>",
        )?;
        let node = normalize_node_name(parts[1]);
        if node == "0" {
            return Err(self.error(
                ".inject node cannot be ground (0) — injection is single-ended (node-to-ground)",
            ));
        }
        let field_name = parts[2].to_string();
        if !is_valid_rust_ident(&field_name) {
            return Err(self.error(format!(
                ".inject field name '{}' is not a valid Rust identifier \
                 (must start with letter or _, contain only ASCII letters/digits/_)",
                field_name
            )));
        }
        // Impedance token: exactly one of R=<ohms> / RSHUNT=<ohms>. MANDATORY.
        let imp_tok = parts[3];
        let (key, val_str) = imp_tok.split_once('=').ok_or_else(|| {
            self.error(format!(
                ".inject requires a mandatory source impedance 'R=<ohms>' (Thevenin) or \
                 'RSHUNT=<ohms>' (Norton); got '{}'. An ideal source with no impedance \
                 would clamp the node and destroy the dry path.",
                imp_tok
            ))
        })?;
        let ohms = self.parse_positive_value(val_str, ".inject impedance")?;
        let impedance = match key.to_ascii_uppercase().as_str() {
            "R" => InjectImpedance::Thevenin(ohms),
            "RSHUNT" => InjectImpedance::Norton(ohms),
            other => {
                return Err(self.error(format!(
                    ".inject impedance key '{}' must be 'R' (Thevenin) or 'RSHUNT' (Norton)",
                    other
                )));
            }
        };
        // Optional trailing `rate=host|inner`. Anything else is refused: a
        // misspelt rate must not silently fall back to the default.
        let mut rate: Option<InjectRate> = None;
        for tok in &parts[4..] {
            let Some((k, v)) = tok.split_once('=') else {
                return Err(self.error(format!(
                    ".inject: unexpected token '{tok}' (the only option after the \
                     impedance is rate=host|inner)"
                )));
            };
            if !k.eq_ignore_ascii_case("rate") {
                return Err(self.error(format!(
                    ".inject: unknown option '{k}' (the only option after the \
                     impedance is rate=host|inner)"
                )));
            }
            if rate.is_some() {
                return Err(self.error(".inject: rate= given more than once"));
            }
            rate = Some(match v.to_ascii_lowercase().as_str() {
                "host" => InjectRate::Host,
                "inner" => InjectRate::Inner,
                _ => {
                    return Err(self.error(format!(
                        ".inject rate '{v}' must be 'host' (an audio-rate input, \
                         supplied per host sample and upsampled like the audio input) \
                         or 'inner' (supplied per inner oversampled sub-step)"
                    )));
                }
            });
        }
        Ok(InjectDirective {
            node,
            field_name,
            impedance,
            rate: rate.unwrap_or_default(),
        })
    }

    /// Parse `.tap <node> [name]` — declare a raw inner-rate probe node.
    fn parse_tap_directive(&self, parts: &[&str]) -> Result<TapDirective, ParseError> {
        // .tap <node> [name]
        self.require_parts(parts, 2, ".tap <node> [name]")?;
        let node = normalize_node_name(parts[1]);
        if node == "0" {
            return Err(self.error(".tap node cannot be ground (0)"));
        }
        let name = match parts.get(2) {
            Some(n) => {
                if !is_valid_rust_ident(n) {
                    return Err(
                        self.error(format!(".tap name '{}' is not a valid Rust identifier", n))
                    );
                }
                n.to_string()
            }
            None => node.clone(),
        };
        Ok(TapDirective { node, name })
    }

    /// `.port <node> [<node> ...]` — declare the board's pins.
    ///
    /// Direction-neutral (see [`PortDirective`]) and repeatable: several
    /// `.port` lines accumulate, so a board can group its pins by branch the
    /// way its schematic does. Ground is rejected — node `0` is connected by
    /// definition and declaring it as a pin can only be a mistake — and so is
    /// a pin declared twice, which is a copy-paste artifact with no meaning.
    ///
    /// Whether the named node EXISTS is deliberately not checked here: that
    /// diagnostic wants the same nearest-name suggestion the dangling check
    /// gives, so it lives with it in [`crate::topology`].
    fn parse_port_directive(
        &self,
        parts: &[&str],
        netlist: &mut Netlist,
    ) -> Result<(), ParseError> {
        self.require_parts(parts, 2, ".port <node> [<node> ...]")?;
        for raw in &parts[1..] {
            let node = normalize_node_name(raw);
            if node == "0" {
                return Err(self.error(
                    ".port cannot declare ground (0): every element already connects to it",
                ));
            }
            if netlist.ports.iter().any(|p| p.node == node) {
                return Err(self.error(format!(".port declares node '{node}' more than once")));
            }
            netlist.ports.push(PortDirective {
                node,
                line: self.line_num,
            });
        }
        Ok(())
    }

    fn parse_input_impedance_directive(
        &self,
        parts: &[&str],
        netlist: &mut Netlist,
    ) -> Result<(), ParseError> {
        // .input_impedance <value>
        self.require_parts(parts, 2, ".input_impedance <value>")?;

        if netlist.input_impedance.is_some() {
            return Err(self.error("Duplicate .input_impedance directive"));
        }

        let value = self.parse_positive_value(parts[1], ".input_impedance")?;
        netlist.input_impedance = Some(value);
        Ok(())
    }
}

/// The `.mismatch` keys each device class reads, as the codegen IR's
/// `apply_mismatch` calls read them (held equal by
/// `model_param_table_drift_tests`). `J` reads `IDSS` on a level-1 JFET and
/// `BETA` on a level-2 one.
pub fn mismatch_keys(device_class: char) -> &'static [&'static str] {
    match device_class {
        'D' => &["IS", "N", "RS"],
        'Q' => &["IS", "BF", "BR"],
        'J' => &["IDSS", "BETA", "VP", "LAMBDA"],
        'M' => &["KP", "VT", "LAMBDA"],
        'T' => &["MU", "EX", "KG1", "KP", "KVB", "KG2"],
        _ => &[],
    }
}
