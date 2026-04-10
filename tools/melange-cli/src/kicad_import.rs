//! KiCad netlist import — converts KiCad XML or SPICE netlists to Melange .cir format.

use anyhow::{Context, Result, bail};
use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use crate::ImportFormat;

/// Entry point for `melange import`.
pub fn import_kicad(input: &Path, output: &Path, format: &ImportFormat) -> Result<()> {
    let content = std::fs::read_to_string(input)
        .with_context(|| format!("Failed to read {}", input.display()))?;

    let is_xml = match format {
        ImportFormat::Auto => content.trim_start().starts_with("<?xml") || content.trim_start().starts_with("<export"),
        ImportFormat::Xml => true,
        ImportFormat::Spice => false,
    };

    let result = if is_xml {
        println!("melange import (KiCad XML → Melange .cir)");
        println!("  Source: {}", input.display());
        import_xml(&content)?
    } else {
        println!("melange import (KiCad SPICE → Melange .cir)");
        println!("  Source: {}", input.display());
        import_spice(&content)?
    };

    std::fs::write(output, &result)
        .with_context(|| format!("Failed to write {}", output.display()))?;

    println!("  Output: {}", output.display());
    Ok(())
}

// ─── XML import (full fidelity) ─────────────────────────────────────

struct XmlComponent {
    ref_des: String,
    value: String,
    lib_part: String,
    fields: HashMap<String, String>,
    pin_nets: HashMap<String, String>, // pin_number -> net_name
}

fn import_xml(content: &str) -> Result<String> {
    use quick_xml::events::Event;
    use quick_xml::Reader;

    let mut reader = Reader::from_str(content);
    let mut components: Vec<XmlComponent> = Vec::new();
    let mut nets: Vec<(String, Vec<(String, String)>)> = Vec::new(); // (net_name, [(ref, pin)])
    let mut title = String::new();

    // State machine for XML parsing
    let mut in_components = false;
    let mut in_comp = false;
    let mut in_fields = false;
    let mut in_nets = false;
    let mut in_net = false;
    let mut in_design = false;
    let mut current_comp: Option<XmlComponent> = None;
    let mut current_net_name = String::new();
    let mut current_net_nodes: Vec<(String, String)> = Vec::new();
    let mut current_field_name = String::new();
    let mut current_text = String::new();

    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                let local_name = e.local_name();
                let local = std::str::from_utf8(local_name.as_ref()).unwrap_or("");
                match local {
                    "design" => in_design = true,
                    "components" => in_components = true,
                    "comp" if in_components => {
                        in_comp = true;
                        let ref_des = attr_str(e, "ref");
                        current_comp = Some(XmlComponent {
                            ref_des,
                            value: String::new(),
                            lib_part: String::new(),
                            fields: HashMap::new(),
                            pin_nets: HashMap::new(),
                        });
                    }
                    "fields" if in_comp => in_fields = true,
                    "field" if in_fields => {
                        current_field_name = attr_str(e, "name");
                    }
                    "libsource" if in_comp => {
                        if let Some(ref mut comp) = current_comp {
                            comp.lib_part = attr_str(e, "part");
                        }
                    }
                    "nets" => in_nets = true,
                    "net" if in_nets => {
                        in_net = true;
                        current_net_name = attr_str(e, "name");
                        current_net_nodes.clear();
                    }
                    "node" if in_net => {
                        let ref_des = attr_str(e, "ref");
                        let pin = attr_str(e, "pin");
                        current_net_nodes.push((ref_des.clone(), pin.clone()));
                        // Also record on component
                        for comp in &mut components {
                            if comp.ref_des == ref_des {
                                comp.pin_nets.insert(pin.clone(), current_net_name.clone());
                            }
                        }
                        // Check current_comp too (for components not yet pushed)
                        if let Some(ref mut comp) = current_comp {
                            if comp.ref_des == ref_des {
                                comp.pin_nets.insert(pin, current_net_name.clone());
                            }
                        }
                    }
                    _ => {}
                }
                current_text.clear();
            }
            Ok(Event::Text(ref e)) => {
                current_text = e.unescape().unwrap_or_default().to_string();
            }
            Ok(Event::End(ref e)) => {
                let local_name = e.local_name();
                let local = std::str::from_utf8(local_name.as_ref()).unwrap_or("");
                match local {
                    "design" => in_design = false,
                    "source" if in_design && title.is_empty() => {
                        let base = Path::new(&current_text)
                            .file_stem()
                            .and_then(|s| s.to_str())
                            .unwrap_or("Imported Circuit");
                        title = base.replace(['_', '-'], " ");
                    }
                    "components" => in_components = false,
                    "comp" => {
                        in_comp = false;
                        if let Some(comp) = current_comp.take() {
                            components.push(comp);
                        }
                    }
                    "value" if in_comp => {
                        if let Some(ref mut comp) = current_comp {
                            comp.value = current_text.clone();
                        }
                    }
                    "fields" => in_fields = false,
                    "field" if in_fields => {
                        if let Some(ref mut comp) = current_comp {
                            if !current_field_name.is_empty() {
                                comp.fields.insert(current_field_name.clone(), current_text.clone());
                            }
                        }
                    }
                    "nets" => in_nets = false,
                    "net" => {
                        in_net = false;
                        nets.push((current_net_name.clone(), current_net_nodes.clone()));
                        // Backfill pin_nets for components parsed before nets
                        for (ref_des, pin) in &current_net_nodes {
                            for comp in &mut components {
                                if comp.ref_des == *ref_des {
                                    comp.pin_nets
                                        .entry(pin.clone())
                                        .or_insert_with(|| current_net_name.clone());
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => bail!("XML parse error at position {}: {}", reader.error_position(), e),
            _ => {}
        }
        buf.clear();
    }

    // Now generate the .cir file
    let mut lines = Vec::new();
    lines.push(title);

    // Collect models (deduplicated)
    let mut models: BTreeMap<String, String> = BTreeMap::new();
    let mut comp_lines = Vec::new();
    let mut pot_directives = Vec::new();
    let mut switch_directives = Vec::new();
    let mut gang_entries: Vec<(String, String, bool)> = Vec::new(); // (label, ref, inverted)
    let mut comp_count = 0u32;

    for comp in &components {
        // Skip I/O markers
        if comp.fields.get("Melange.Input").map(|v| v.to_lowercase()) == Some("true".into()) {
            continue;
        }
        if comp.fields.get("Melange.Output").map(|v| v.to_lowercase()) == Some("true".into()) {
            continue;
        }

        let prefix = ref_prefix(&comp.ref_des);
        let get_node = |pin: &str| -> String {
            comp.pin_nets
                .get(pin)
                .map(|n| sanitize_node(n))
                .unwrap_or_else(|| "0".into())
        };

        let line = match prefix.as_str() {
            "R" | "C" | "L" => {
                let n1 = get_node("1");
                let n2 = get_node("2");
                let val = format_value(&comp.value);
                // Check for pot
                if let Some(pot_field) = comp.fields.get("Melange.Pot") {
                    let label = comp.fields.get("Melange.Label").cloned().unwrap_or_default();
                    let parts: Vec<&str> = pot_field.split_whitespace().collect();
                    let mut pot_str = format!(".pot {} {} {}", comp.ref_des,
                        parts.first().unwrap_or(&"1k"), parts.get(1).unwrap_or(&"100k"));
                    if let Some(default) = parts.get(2) {
                        pot_str.push_str(&format!(" {default}"));
                    }
                    if !label.is_empty() {
                        pot_str.push_str(&format!(" \"{label}\""));
                    }
                    pot_directives.push(pot_str);
                }
                // Check for switch
                if let Some(sw_field) = comp.fields.get("Melange.Switch") {
                    let label = comp.fields.get("Melange.Label").cloned().unwrap_or_default();
                    let mut sw_str = format!(".switch {} {}", comp.ref_des, sw_field.trim());
                    if !label.is_empty() {
                        sw_str.push_str(&format!(" \"{label}\""));
                    }
                    switch_directives.push(sw_str);
                }
                // Check for gang
                if let Some(gang_label) = comp.fields.get("Melange.Gang") {
                    let inverted = comp.fields.get("Melange.GangInvert")
                        .map(|v| v.to_lowercase() == "true").unwrap_or(false);
                    gang_entries.push((gang_label.clone(), comp.ref_des.clone(), inverted));
                }
                format!("{} {} {} {}", comp.ref_des, n1, n2, val)
            }
            "D" => {
                let anode = get_node("2"); // pin A
                let cathode = get_node("1"); // pin K
                collect_model(comp, "D", &mut models);
                format!("{} {} {} {}", comp.ref_des, anode, cathode, comp.value)
            }
            "Q" => {
                let nc = get_node("1");
                let nb = get_node("2");
                let ne = get_node("3");
                collect_model(comp, "", &mut models);
                format!("{} {} {} {} {}", comp.ref_des, nc, nb, ne, comp.value)
            }
            "J" => {
                let nd = get_node("1");
                let ng = get_node("2");
                let ns = get_node("3");
                collect_model(comp, "", &mut models);
                format!("{} {} {} {} {}", comp.ref_des, nd, ng, ns, comp.value)
            }
            "M" => {
                let nd = get_node("1");
                let ng = get_node("2");
                let ns = get_node("3");
                let nb = if comp.pin_nets.contains_key("4") { get_node("4") } else { ns.clone() };
                collect_model(comp, "", &mut models);
                format!("{} {} {} {} {} {}", comp.ref_des, nd, ng, ns, nb, comp.value)
            }
            "T" => {
                let ng = get_node("1");
                let np = get_node("2");
                let nk = get_node("3");
                collect_model(comp, "TRIODE", &mut models);
                format!("{} {} {} {} {}", comp.ref_des, ng, np, nk, comp.value)
            }
            "U" => {
                let nplus = get_node("1");
                let nminus = get_node("2");
                let nout = get_node("3");
                collect_model(comp, "OA", &mut models);
                format!("{} {} {} {} {}", comp.ref_des, nplus, nminus, nout, comp.value)
            }
            "Y" => {
                let sp = get_node("1");
                let sn = get_node("2");
                let cp = get_node("3");
                let cn = get_node("4");
                collect_model(comp, "VCA", &mut models);
                format!("{} {} {} {} {} {}", comp.ref_des, sp, sn, cp, cn, comp.value)
            }
            "V" => {
                let n1 = get_node("1");
                let n2 = get_node("2");
                let val = format_value(&comp.value);
                format!("{} {} {} DC {}", comp.ref_des, n1, n2, val)
            }
            "I" => {
                let n1 = get_node("1");
                let n2 = get_node("2");
                let val = format_value(&comp.value);
                format!("{} {} {} DC {}", comp.ref_des, n1, n2, val)
            }
            "K" => {
                format!("{} {}", comp.ref_des, comp.value)
            }
            "E" | "G" => {
                let n1 = get_node("1");
                let n2 = get_node("2");
                let n3 = get_node("3");
                let n4 = get_node("4");
                let val = format_value(&comp.value);
                format!("{} {} {} {} {} {}", comp.ref_des, n1, n2, n3, n4, val)
            }
            "X" => {
                let mut nodes = Vec::new();
                let mut pin_num = 1;
                while comp.pin_nets.contains_key(&pin_num.to_string()) {
                    nodes.push(get_node(&pin_num.to_string()));
                    pin_num += 1;
                }
                format!("{} {} {}", comp.ref_des, nodes.join(" "), comp.value)
            }
            _ => continue,
        };

        comp_lines.push(line);
        comp_count += 1;
    }

    // Write models
    if !models.is_empty() {
        lines.push(String::new());
        lines.push("* --- Device Models ---".into());
        for model_line in models.values() {
            lines.push(model_line.clone());
        }
    }

    // Write components
    lines.push(String::new());
    lines.push("* --- Circuit ---".into());
    lines.extend(comp_lines);

    // Write directives
    let mut directives = Vec::new();
    directives.extend(pot_directives.clone());
    directives.extend(switch_directives.clone());

    // Gang groups
    let mut gang_groups: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (label, ref_des, inverted) in &gang_entries {
        let entry = if *inverted {
            format!("!{ref_des}")
        } else {
            ref_des.clone()
        };
        gang_groups.entry(label.clone()).or_default().push(entry);
    }
    for (label, members) in &gang_groups {
        if members.len() >= 2 {
            directives.push(format!(".gang \"{}\" {}", label, members.join(" ")));
        }
    }

    if !directives.is_empty() {
        lines.push(String::new());
        lines.push("* --- Controls ---".into());
        for d in &directives {
            lines.push(d.clone());
        }
    }

    lines.push(String::new());
    lines.push(".END".into());
    lines.push(String::new());

    println!("  Components: {comp_count}");
    println!("  Models: {}", models.len());
    println!("  Pots: {}", pot_directives.len());
    println!("  Switches: {}", switch_directives.len());
    println!("  Gangs: {}", gang_groups.len());

    Ok(lines.join("\n"))
}

fn collect_model(comp: &XmlComponent, default_type: &str, models: &mut BTreeMap<String, String>) {
    if models.contains_key(&comp.value) {
        return;
    }
    if let Some(model_field) = comp.fields.get("Melange.Model") {
        models.insert(comp.value.clone(), format!(".model {} {}", comp.value, model_field));
    } else if let Some(sim_params) = comp.fields.get("Sim.Params") {
        if !default_type.is_empty() {
            models.insert(
                comp.value.clone(),
                format!(".model {} {}({})", comp.value, default_type, sim_params),
            );
        }
    }
}

fn attr_str(e: &quick_xml::events::BytesStart, name: &str) -> String {
    e.attributes()
        .filter_map(|a| a.ok())
        .find(|a| a.key.as_ref() == name.as_bytes())
        .and_then(|a| String::from_utf8(a.value.to_vec()).ok())
        .unwrap_or_default()
}

// ─── SPICE import (best-effort) ─────────────────────────────────────

fn import_spice(content: &str) -> Result<String> {
    let mut lines = Vec::new();
    let mut comp_count = 0u32;
    let mut skipped = 0u32;

    for (i, line) in content.lines().enumerate() {
        let trimmed = line.trim();

        // First line is always the title
        if i == 0 {
            lines.push(trimmed.to_string());
            continue;
        }

        // Skip empty lines and comments (pass through)
        if trimmed.is_empty() || trimmed.starts_with('*') {
            lines.push(line.to_string());
            continue;
        }

        // Skip KiCad-specific SPICE directives that melange doesn't use
        let lower = trimmed.to_lowercase();
        if lower.starts_with(".tran")
            || lower.starts_with(".ac ")
            || lower.starts_with(".dc ")
            || lower.starts_with(".options")
            || lower.starts_with(".save")
            || lower.starts_with(".probe")
            || lower.starts_with(".control")
            || lower.starts_with(".endc")
            || lower.starts_with(".measure")
            || lower.starts_with(".meas")
            || lower.starts_with(".op")
            || lower.starts_with(".ic ")
            || lower.starts_with(".nodeset")
            || lower.starts_with(".global")
            || lower.starts_with(".lib ")
            || lower.starts_with(".include ")
            || lower.starts_with(".title")
        {
            lines.push(format!("* [skipped] {trimmed}"));
            skipped += 1;
            continue;
        }

        // Pass through .model, .subckt, .ends, .end, .param and component lines
        if lower.starts_with('.') || trimmed.chars().next().map(|c| c.is_ascii_alphabetic()).unwrap_or(false) {
            // Sanitize node name "GND" -> "0" in component lines
            if !lower.starts_with('.') {
                let sanitized = sanitize_component_nodes(trimmed);
                lines.push(sanitized);
            } else {
                lines.push(trimmed.to_string());
            }
            comp_count += 1;
            continue;
        }

        // Pass through anything else
        lines.push(line.to_string());
    }

    // Ensure .END exists
    if !lines.iter().any(|l| l.trim().to_lowercase() == ".end") {
        lines.push(String::new());
        lines.push(".END".into());
    }

    lines.push(String::new());

    println!("  Components: ~{comp_count}");
    println!("  Skipped directives: {skipped}");
    println!("  Note: SPICE import is best-effort. Add .pot/.switch/.gang directives manually.");

    Ok(lines.join("\n"))
}

fn sanitize_component_nodes(line: &str) -> String {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.is_empty() {
        return line.to_string();
    }
    parts
        .iter()
        .map(|p| {
            if p.eq_ignore_ascii_case("gnd") {
                "0"
            } else {
                p
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

// ─── Shared helpers ─────────────────────────────────────────────────

fn ref_prefix(ref_des: &str) -> String {
    ref_des.chars().take_while(|c| c.is_ascii_alphabetic()).collect()
}

fn sanitize_node(name: &str) -> String {
    let low = name.trim().to_lowercase();
    if low == "gnd" || low == "0" || low == "/gnd" {
        return "0".into();
    }
    let s = name.trim_start_matches('/');
    let s: String = s.chars().map(|c| {
        if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' }
    }).collect();
    let s = s.trim_matches('_').to_lowercase();
    if s.is_empty() {
        return "node".into();
    }
    if s.chars().next().map(|c| c.is_ascii_digit()).unwrap_or(false) {
        format!("n{s}")
    } else {
        s
    }
}

fn format_value(value: &str) -> String {
    if value.is_empty() {
        return "0".into();
    }
    let s = value.trim().trim_end_matches(|c: char| "ΩFHVAΩfhva".contains(c));
    if s.is_empty() { value.trim().to_string() } else { s.to_string() }
}
