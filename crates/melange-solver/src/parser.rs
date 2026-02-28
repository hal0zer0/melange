//! SPICE netlist parser.
//!
//! Parses a subset of SPICE sufficient for audio circuits:
//! - Components: R, C, L, V (DC/AC), I, D, Q, J, M, X
//! - Directives: .model, .subckt, .param, .end
//!
//! The parser builds an AST (Abstract Syntax Tree) representation
//! of the circuit that can be processed by the MNA assembler.

/// A parsed SPICE netlist.
#[derive(Debug, Clone, PartialEq)]
pub struct Netlist {
    /// Circuit title (first line)
    pub title: String,
    /// Circuit elements
    pub elements: Vec<Element>,
    /// Model definitions
    pub models: Vec<Model>,
    /// Subcircuit definitions
    pub subcircuits: Vec<Subcircuit>,
    /// Global parameters
    pub params: Vec<Parameter>,
}

impl Netlist {
    /// Create an empty netlist.
    pub fn new(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            elements: Vec::new(),
            models: Vec::new(),
            subcircuits: Vec::new(),
            params: Vec::new(),
        }
    }

    /// Parse a netlist from a string.
    pub fn parse(input: &str) -> Result<Self, ParseError> {
        Parser::new(input).parse()
    }
}

/// A circuit element (component).
#[derive(Debug, Clone, PartialEq)]
pub enum Element {
    /// Resistor: Rname n+ n- value
    Resistor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
    },
    /// Capacitor: Cname n+ n- value [IC=initial]
    Capacitor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
        ic: Option<f64>,
    },
    /// Inductor: Lname n+ n- value
    Inductor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
    },
    /// Voltage source: Vname n+ n- [DC value] [AC mag phase]
    VoltageSource {
        name: String,
        n_plus: String,
        n_minus: String,
        dc: Option<f64>,
        ac: Option<(f64, f64)>,
    },
    /// Current source: Iname n+ n- [DC value]
    CurrentSource {
        name: String,
        n_plus: String,
        n_minus: String,
        dc: Option<f64>,
    },
    /// Diode: Dname n+ n- modelname
    Diode {
        name: String,
        n_plus: String,
        n_minus: String,
        model: String,
    },
    /// BJT: Qname nc nb ne [ns] modelname
    Bjt {
        name: String,
        nc: String,
        nb: String,
        ne: String,
        model: String,
    },
    /// JFET: Jname nd ng ns modelname
    Jfet {
        name: String,
        nd: String,
        ng: String,
        ns: String,
        model: String,
    },
    /// MOSFET: Mname nd ng ns nb modelname
    Mosfet {
        name: String,
        nd: String,
        ng: String,
        ns: String,
        nb: String,
        model: String,
    },
    /// Subcircuit instance: Xname nodes... subcktname
    SubcktInstance {
        name: String,
        nodes: Vec<String>,
        subckt: String,
    },
}

impl Element {
    /// Get the element name.
    pub fn name(&self) -> &str {
        match self {
            Element::Resistor { name, .. } => name,
            Element::Capacitor { name, .. } => name,
            Element::Inductor { name, .. } => name,
            Element::VoltageSource { name, .. } => name,
            Element::CurrentSource { name, .. } => name,
            Element::Diode { name, .. } => name,
            Element::Bjt { name, .. } => name,
            Element::Jfet { name, .. } => name,
            Element::Mosfet { name, .. } => name,
            Element::SubcktInstance { name, .. } => name,
        }
    }
}

/// A model definition (.model).
#[derive(Debug, Clone, PartialEq)]
pub struct Model {
    pub name: String,
    pub model_type: String,
    pub params: Vec<(String, f64)>,
}

/// A subcircuit definition (.subckt).
#[derive(Debug, Clone, PartialEq)]
pub struct Subcircuit {
    pub name: String,
    pub nodes: Vec<String>,
    pub elements: Vec<Element>,
}

/// A parameter definition (.param).
#[derive(Debug, Clone, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub value: f64,
}

/// Parse error.
#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Parse error at line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for ParseError {}

/// SPICE netlist parser.
struct Parser {
    /// Pre-processed lines (after continuation joining and comment stripping)
    processed_lines: Vec<String>,
    /// Current position in processed_lines
    pos: usize,
    /// Line number tracking for error messages
    line_num: usize,
}

impl Parser {
    fn new(input: &str) -> Self {
        // Pre-process: join continuation lines and strip comments
        let raw_lines: Vec<&str> = input.lines().collect();
        let mut processed = Vec::new();
        let mut i = 0;

        while i < raw_lines.len() {
            // Strip inline comments (semicolon and $ delimiter)
            let line = raw_lines[i]
                .split(|c| c == ';' || c == '$')
                .next()
                .unwrap_or(raw_lines[i])
                .trim();

            // Accumulate continuation lines: next line starts with '+'
            let mut accumulated = line.to_string();
            while i + 1 < raw_lines.len() {
                let next = raw_lines[i + 1]
                    .split(|c| c == ';' || c == '$')
                    .next()
                    .unwrap_or(raw_lines[i + 1])
                    .trim();
                if next.starts_with('+') {
                    // Continuation line: strip '+' and append
                    accumulated.push(' ');
                    accumulated.push_str(next[1..].trim());
                    i += 1;
                } else {
                    break;
                }
            }

            processed.push(accumulated);
            i += 1;
        }

        Self {
            processed_lines: processed,
            pos: 0,
            line_num: 0,
        }
    }

    fn parse(mut self) -> Result<Netlist, ParseError> {
        // First line is title
        let title = self.next_line().unwrap_or_default();
        let mut netlist = Netlist::new(title);

        while let Some(line) = self.next_line() {
            let line = line.trim().to_string();

            // Skip empty lines and comments
            if line.is_empty() || line.starts_with('*') {
                continue;
            }

            // Parse directive or element
            if line.starts_with('.') {
                self.parse_directive(&line, &mut netlist)?;
            } else {
                let element = self.parse_element(&line)?;
                netlist.elements.push(element);
            }
        }

        Ok(netlist)
    }

    fn next_line(&mut self) -> Option<String> {
        if self.pos < self.processed_lines.len() {
            let line = self.processed_lines[self.pos].clone();
            self.pos += 1;
            self.line_num += 1;
            Some(line)
        } else {
            None
        }
    }

    fn error(&self, message: impl Into<String>) -> ParseError {
        ParseError {
            line: self.line_num,
            message: message.into(),
        }
    }

    fn parse_directive(&mut self, line: &str, netlist: &mut Netlist) -> Result<(), ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return Ok(());
        }

        match parts[0].to_lowercase().as_str() {
            ".model" => {
                if parts.len() < 3 {
                    return Err(self.error(".model requires name and type"));
                }
                let model = self.parse_model(&parts)?;
                netlist.models.push(model);
            }
            ".param" => {
                let param = self.parse_param(&parts)?;
                netlist.params.push(param);
            }
            ".subckt" => {
                if parts.len() < 2 {
                    return Err(self.error(".subckt requires a name"));
                }
                let subckt_name = parts[1].to_string();
                let subckt_nodes: Vec<String> = parts[2..].iter().map(|s| s.to_string()).collect();
                let mut subckt_elements = Vec::new();

                // Collect elements until .ends
                while let Some(sub_line) = self.next_line() {
                    let sub_line = sub_line.trim().to_string();
                    if sub_line.is_empty() || sub_line.starts_with('*') {
                        continue;
                    }
                    if sub_line.to_lowercase().starts_with(".ends") {
                        break;
                    }
                    if sub_line.starts_with('.') {
                        // Nested directives inside subckt (e.g. .model)
                        let sub_parts: Vec<&str> = sub_line.split_whitespace().collect();
                        if sub_parts[0].to_lowercase() == ".model" {
                            let model = self.parse_model(&sub_parts)?;
                            netlist.models.push(model);
                        }
                        continue;
                    }
                    let elem = self.parse_element(&sub_line)?;
                    subckt_elements.push(elem);
                }

                netlist.subcircuits.push(Subcircuit {
                    name: subckt_name,
                    nodes: subckt_nodes,
                    elements: subckt_elements,
                });
            }
            ".end" | ".ends" => {
                // End of netlist or subcircuit
            }
            _ => {
                // Unknown directive, ignore
            }
        }

        Ok(())
    }

    fn parse_model(&self, parts: &[&str]) -> Result<Model, ParseError> {
        let name = parts[1].to_string();
        let mut params = Vec::new();

        // Reconstruct everything after ".model NAME" to handle TYPE(PARAMS...) correctly
        // e.g. ".model D1N4148 D(IS=1e-15)" or ".model 2N2222 NPN(IS=1e-15 BF=200)"
        let rest: String = parts[2..].join(" ");

        // Extract model type: everything before the first '('
        let model_type = if let Some(paren_pos) = rest.find('(') {
            rest[..paren_pos].trim().to_ascii_uppercase()
        } else {
            rest.trim().to_ascii_uppercase()
        };

        // Extract params from between parentheses (if any)
        if let (Some(open), Some(close)) = (rest.find('('), rest.rfind(')')) {
            let params_str = &rest[open + 1..close];
            for token in params_str.split_whitespace() {
                if let Some(eq_pos) = token.find('=') {
                    let key = token[..eq_pos].to_ascii_uppercase();
                    let value_str = &token[eq_pos + 1..];
                    let value = parse_value(value_str)
                        .map_err(|_| self.error(format!("Invalid model parameter value: {}", value_str)))?;
                    params.push((key, value));
                }
            }
        }

        Ok(Model {
            name,
            model_type,
            params,
        })
    }

    fn parse_param(&self, parts: &[&str]) -> Result<Parameter, ParseError> {
        // .param name=value
        if parts.len() < 2 {
            return Err(self.error(".param requires name=value"));
        }

        let param_str = parts[1];
        if let Some(eq_pos) = param_str.find('=') {
            let name = param_str[..eq_pos].to_string();
            let value_str = &param_str[eq_pos + 1..];
            let value = parse_value(value_str)
                .map_err(|_| self.error(format!("Invalid parameter value: {}", value_str)))?;
            Ok(Parameter { name, value })
        } else {
            Err(self.error("Parameter must be name=value format"))
        }
    }

    fn parse_element(&self, line: &str) -> Result<Element, ParseError> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return Err(self.error("Empty element line"));
        }

        let name = parts[0];
        let first_char = name.chars().next().unwrap_or(' ').to_ascii_uppercase();

        match first_char {
            'R' => self.parse_resistor(&parts),
            'C' => self.parse_capacitor(&parts),
            'L' => self.parse_inductor(&parts),
            'V' => self.parse_voltage_source(&parts),
            'I' => self.parse_current_source(&parts),
            'D' => self.parse_diode(&parts),
            'Q' => self.parse_bjt(&parts),
            'J' => self.parse_jfet(&parts),
            'M' => self.parse_mosfet(&parts),
            'X' => self.parse_subckt_instance(&parts),
            _ => Err(self.error(format!("Unknown element type: {}", first_char))),
        }
    }

    fn parse_resistor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        if parts.len() < 4 {
            return Err(self.error("Resistor requires: Rname n+ n- value"));
        }
        let value = parse_value(parts[3])
            .map_err(|_| self.error(format!("Invalid resistor value: {}", parts[3])))?;
        if value <= 0.0 || !value.is_finite() {
            return Err(self.error(format!(
                "Resistor value must be positive and finite, got {}", value
            )));
        }
        Ok(Element::Resistor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
        })
    }

    fn parse_capacitor(&self, parts: &[&str]) -> Result<Element, ParseError> {
        if parts.len() < 4 {
            return Err(self.error("Capacitor requires: Cname n+ n- value"));
        }

        let value = parse_value(parts[3])
            .map_err(|_| self.error(format!("Invalid capacitor value: {}", parts[3])))?;
        if value <= 0.0 || !value.is_finite() {
            return Err(self.error(format!(
                "Capacitor value must be positive and finite, got {}", value
            )));
        }

        let mut ic = None;
        for part in &parts[4..] {
            if part.to_uppercase().starts_with("IC=") {
                let ic_val = &part[3..];
                ic = Some(parse_value(ic_val)
                    .map_err(|_| self.error(format!("Invalid IC value: {}", ic_val)))?);
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
        if parts.len() < 4 {
            return Err(self.error("Inductor requires: Lname n+ n- value"));
        }
        let value = parse_value(parts[3])
            .map_err(|_| self.error(format!("Invalid inductor value: {}", parts[3])))?;
        if value <= 0.0 || !value.is_finite() {
            return Err(self.error(format!(
                "Inductor value must be positive and finite, got {}", value
            )));
        }
        Ok(Element::Inductor {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            value,
        })
    }

    fn parse_voltage_source(&self, parts: &[&str]) -> Result<Element, ParseError> {
        if parts.len() < 3 {
            return Err(self.error("Voltage source requires: Vname n+ n-"));
        }

        let mut dc = None;
        let mut ac = None;

        // Parse DC and AC values
        let mut i = 3;
        while i < parts.len() {
            let part_upper = parts[i].to_uppercase();
            if part_upper == "DC" && i + 1 < parts.len() {
                dc = Some(parse_value(parts[i + 1])
                    .map_err(|_| self.error(format!("Invalid DC value: {}", parts[i + 1])))?);
                i += 2;
            } else if part_upper == "AC" && i + 1 < parts.len() {
                let mag = parse_value(parts[i + 1])
                    .map_err(|_| self.error(format!("Invalid AC magnitude: {}", parts[i + 1])))?;
                let phase = if i + 2 < parts.len() {
                    parse_value(parts[i + 2]).unwrap_or(0.0)
                } else {
                    0.0
                };
                ac = Some((mag, phase));
                i += 3;
            } else if dc.is_none() {
                // Bare value is DC
                dc = Some(parse_value(parts[i])
                    .map_err(|_| self.error(format!("Invalid DC value: {}", parts[i])))?);
                i += 1;
            } else {
                i += 1;
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

    fn parse_current_source(&self, parts: &[&str]) -> Result<Element, ParseError> {
        if parts.len() < 3 {
            return Err(self.error("Current source requires: Iname n+ n-"));
        }

        let mut dc = None;

        if parts.len() >= 4 {
            if parts[3].to_uppercase() == "DC" && parts.len() >= 5 {
                dc = Some(parse_value(parts[4])
                    .map_err(|_| self.error(format!("Invalid DC value: {}", parts[4])))?);
            } else {
                dc = Some(parse_value(parts[3])
                    .map_err(|_| self.error(format!("Invalid DC value: {}", parts[3])))?);
            }
        }

        Ok(Element::CurrentSource {
            name: parts[0].to_string(),
            n_plus: parts[1].to_string(),
            n_minus: parts[2].to_string(),
            dc,
        })
    }

    fn parse_diode(&self, parts: &[&str]) -> Result<Element, ParseError> {
        if parts.len() < 4 {
            return Err(self.error("Diode requires: Dname n+ n- modelname"));
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
        if parts.len() < 5 {
            return Err(self.error("BJT requires: Qname nc nb ne modelname"));
        }
        
        let nc = parts[1].to_string();
        let nb = parts[2].to_string();
        let ne = parts[3].to_string();
        
        // Model is always the last part
        // If we have 6+ parts, there's a substrate node before the model
        let model = parts[parts.len() - 1].to_string();

        Ok(Element::Bjt {
            name: parts[0].to_string(),
            nc,
            nb,
            ne,
            model,
        })
    }

    fn parse_jfet(&self, parts: &[&str]) -> Result<Element, ParseError> {
        if parts.len() < 5 {
            return Err(self.error("JFET requires: Jname nd ng ns modelname"));
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
        if parts.len() < 6 {
            return Err(self.error("MOSFET requires: Mname nd ng ns nb modelname"));
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

    fn parse_subckt_instance(&self, parts: &[&str]) -> Result<Element, ParseError> {
        if parts.len() < 3 {
            return Err(self.error("Subcircuit instance requires: Xname nodes... subcktname"));
        }
        
        let name = parts[0].to_string();
        let subckt = parts[parts.len() - 1].to_string();
        let nodes = parts[1..parts.len() - 1].iter().map(|s| s.to_string()).collect();

        Ok(Element::SubcktInstance {
            name,
            nodes,
            subckt,
        })
    }
}

/// Parse a SPICE value with optional scale suffix.
///
/// Examples:
/// - "1k" -> 1000.0
/// - "4.7u" -> 4.7e-6
/// - "10pF" -> 10e-12
/// - "1F" -> 1.0 (Farad, not femto)
fn parse_value(s: &str) -> Result<f64, ParseFloatError> {
    let s = s.trim();
    if s.is_empty() {
        return Err(ParseFloatError);
    }
    let s_upper = s.to_uppercase();

    // Check for MEG first (must be before stripping units)
    if s_upper.ends_with("MEG") {
        let num_part = &s[..s.len()-3];
        if num_part.is_empty() {
            return Err(ParseFloatError);
        }
        let value: f64 = num_part.parse().map_err(|_| ParseFloatError)?;
        return Ok(value * 1e6);
    }

    // Strip unit suffixes FIRST (F, H, V, A, OHM, HZ)
    // Note: We remove from s_upper to get the uppercase version, then apply to original s
    let stripped_upper = s_upper.trim_end_matches(|c: char| {
        matches!(c.to_ascii_uppercase(), 'F' | 'H' | 'V' | 'A' | 'S' | 'Z')
    });

    // Calculate how much we stripped
    let chars_stripped = s_upper.len() - stripped_upper.len();
    let mut num_part = &s[..s.len().saturating_sub(chars_stripped)];
    let mut scale = 1.0;

    // Now parse scale suffix from what's left
    if let Some(c) = stripped_upper.chars().last() {
        let (new_num, new_scale) = match c {
            'T' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e12),
            'G' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e9),
            'K' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e3),
            'M' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-3),  // M = milli
            'U' | 'µ' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-6),
            'N' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-9),
            'P' if num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-12),
            // Note: 'F' as femto is only recognized if no unit was stripped
            // This avoids ambiguity with Farad
            'F' if chars_stripped == 0 && num_part.len() > 1 => (&num_part[..num_part.len()-1], 1e-15),
            _ => (num_part, 1.0),
        };
        num_part = new_num;
        scale = new_scale;
    }

    if num_part.is_empty() {
        return Err(ParseFloatError);
    }

    let value: f64 = num_part.parse().map_err(|_| ParseFloatError)?;
    let result = value * scale;
    if !result.is_finite() {
        return Err(ParseFloatError);
    }
    Ok(result)
}

/// Error type for float parsing failures.
#[derive(Debug, Clone, Copy)]
struct ParseFloatError;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_resistor() {
        let netlist = Netlist::parse("Test Circuit\nR1 1 0 1k\n").unwrap();
        assert_eq!(netlist.elements.len(), 1);
        match &netlist.elements[0] {
            Element::Resistor { name, n_plus, n_minus, value } => {
                assert_eq!(name, "R1");
                assert_eq!(n_plus, "1");
                assert_eq!(n_minus, "0");
                assert_eq!(*value, 1000.0);
            }
            _ => panic!("Expected resistor"),
        }
    }

    #[test]
    fn test_parse_capacitor() {
        let netlist = Netlist::parse("Test\nC1 1 0 10u\n").unwrap();
        match &netlist.elements[0] {
            Element::Capacitor { name, value, .. } => {
                assert_eq!(name, "C1");
                assert!((value - 10e-6).abs() < 1e-15, "Expected ~10uF, got {}", value);
            }
            _ => panic!("Expected capacitor"),
        }
    }

    #[test]
    fn test_parse_bjt() {
        let netlist = Netlist::parse("Test\nQ1 3 2 1 2N2222\n").unwrap();
        match &netlist.elements[0] {
            Element::Bjt { name, nc, nb, ne, model } => {
                assert_eq!(name, "Q1");
                assert_eq!(nc, "3");
                assert_eq!(nb, "2");
                assert_eq!(ne, "1");
                assert_eq!(model, "2N2222");
            }
            _ => panic!("Expected BJT"),
        }
    }

    #[test]
    fn test_parse_model() {
        let netlist = Netlist::parse("Test\n.model 2N2222 NPN(IS=1e-15 BF=200)\n").unwrap();
        assert_eq!(netlist.models.len(), 1);
        let model = &netlist.models[0];
        assert_eq!(model.name, "2N2222");
        assert!(model.model_type.starts_with("NPN"), "Model type should be NPN, got {}", model.model_type);
    }

    #[test]
    fn test_parse_value() {
        assert_eq!(parse_value("1k").unwrap(), 1e3);
        assert_eq!(parse_value("4.7u").unwrap(), 4.7e-6);
        assert_eq!(parse_value("10pF").unwrap(), 10e-12);
        assert_eq!(parse_value("1MEG").unwrap(), 1e6);
    }

    #[test]
    fn test_parse_valid_scale_suffixes() {
        assert!((parse_value("1T").unwrap() - 1e12).abs() < 1e3);
        assert!((parse_value("1G").unwrap() - 1e9).abs() < 1.0);
        assert!((parse_value("1MEG").unwrap() - 1e6).abs() < 1.0);
        assert!((parse_value("1k").unwrap() - 1e3).abs() < 1e-6);
        assert!((parse_value("1m").unwrap() - 1e-3).abs() < 1e-12);
        assert!((parse_value("1u").unwrap() - 1e-6).abs() < 1e-15);
        assert!((parse_value("1n").unwrap() - 1e-9).abs() < 1e-18);
        assert!((parse_value("1p").unwrap() - 1e-12).abs() < 1e-21);
        // "1f" is ambiguous with Farad unit suffix; femto requires no unit suffix after it
        // "1F" = 1.0 Farad (F is unit, not scale), by design
        assert!((parse_value("1F").unwrap() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_parse_negative_resistance_rejected() {
        let result = Netlist::parse("Test\nR1 1 0 -1k\n");
        assert!(result.is_err(), "Negative resistance should be rejected");
    }

    #[test]
    fn test_parse_zero_resistance_rejected() {
        let result = Netlist::parse("Test\nR1 1 0 0\n");
        assert!(result.is_err(), "Zero resistance should be rejected");
    }

    #[test]
    fn test_parse_nan_value_rejected() {
        let result = parse_value("NaN");
        assert!(result.is_err(), "NaN value should be rejected");
    }

    #[test]
    fn test_parse_inf_value_rejected() {
        let result = parse_value("inf");
        assert!(result.is_err(), "Infinity value should be rejected");
    }

    #[test]
    fn test_parse_negative_capacitance_rejected() {
        let result = Netlist::parse("Test\nC1 1 0 -1u\n");
        assert!(result.is_err(), "Negative capacitance should be rejected");
    }

    #[test]
    fn test_parse_zero_inductance_rejected() {
        let result = Netlist::parse("Test\nL1 1 0 0\n");
        assert!(result.is_err(), "Zero inductance should be rejected");
    }

    #[test]
    fn test_parse_missing_value() {
        let result = Netlist::parse("Test\nR1 1 0\n");
        assert!(result.is_err(), "Missing value should be rejected");
    }

    #[test]
    fn test_common_emitter_amp() {
        let spice = r#"Common Emitter Amplifier
Vcc vcc 0 9V
Vin in 0 DC 0 AC 1V
R1 vcc base 100k
R2 base 0 22k
Rc vcc coll 4.7k
Re emit 0 1k
C1 in base 10u
C2 coll out 10u
Q1 coll base emit 2N2222
.model 2N2222 NPN(IS=1e-15 BF=200)
.end
"#;
        let netlist = Netlist::parse(spice).unwrap();
        assert_eq!(netlist.elements.len(), 9);
        assert_eq!(netlist.models.len(), 1);
    }
}
