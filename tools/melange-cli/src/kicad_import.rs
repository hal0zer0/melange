//! KiCad netlist import — converts KiCad XML or SPICE netlists to Melange .cir format.

use anyhow::{bail, Context, Result};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::Path;

use crate::ImportFormat;

/// Entry point for `melange import`.
pub fn import_kicad(
    input: &Path,
    output: &Path,
    format: &ImportFormat,
    from_schematic: bool,
) -> Result<()> {
    // Auto-detect .kicad_sch files
    let is_schematic =
        from_schematic || input.extension().map(|e| e == "kicad_sch").unwrap_or(false);

    if is_schematic {
        return import_from_schematic(input, output, format);
    }

    let content = std::fs::read_to_string(input)
        .with_context(|| format!("Failed to read {}", input.display()))?;

    let is_xml = match format {
        ImportFormat::Auto => {
            content.trim_start().starts_with("<?xml") || content.trim_start().starts_with("<export")
        }
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

/// Leading integer of a `kicad-cli --version` string, e.g. `"7.0.11"` -> 7,
/// `"8.0.1-unknown-abc"` -> 8. `None` when the output does not start with a
/// number, in which case the caller must NOT block: an unparseable banner from
/// some future or patched build is not evidence that the tool is too old.
fn kicad_cli_major_version(ver: &str) -> Option<u32> {
    let digits: String = ver
        .trim()
        .trim_start_matches(|c: char| !c.is_ascii_digit())
        .chars()
        .take_while(|c| c.is_ascii_digit())
        .collect();
    if digits.is_empty() {
        return None;
    }
    digits.parse().ok()
}

/// Import directly from a .kicad_sch schematic file via kicad-cli.
fn import_from_schematic(input: &Path, output: &Path, _format: &ImportFormat) -> Result<()> {
    // Check if kicad-cli is available
    let version_check = std::process::Command::new("kicad-cli")
        .arg("--version")
        .output();

    match version_check {
        Ok(out) if out.status.success() => {
            let ver = String::from_utf8_lossy(&out.stdout);
            let ver = ver.trim().to_string();
            println!("melange import (KiCad schematic → XML → Melange .cir)");
            println!("  kicad-cli: {}", ver);
            // Gate on the version we just printed. kicad-cli 7 cannot open a
            // KiCad 8 `.kicad_sch` (file format version 20231120) and reports
            // only "Failed to load schematic file" — which a user reasonably
            // reads as "my schematic is broken" and answers by redrawing it.
            // We already know whose fault it is; say so.
            if let Some(major) = kicad_cli_major_version(&ver) {
                if major < 8 {
                    bail!(
                        "kicad-cli {ver} is too old for melange's schematic import, which needs \
                         KiCad 8 or newer.\n\
                         \x20 Why: `.kicad_sch` files melange targets carry file format version \
                         20231120 (KiCad 8). kicad-cli 7 cannot open them, and reports only \
                         \"Failed to load schematic file\" whatever the cause — so that message \
                         on its own does not tell you whether your schematic is at fault.\n\
                         \x20 Note: \"Failed to load schematic file\" is also what a genuinely \
                         malformed file gets, so if the same message persists after upgrading, \
                         suspect the file. (melange's own bundled `melange.kicad_sym` and \
                         `kicad/examples/rc-lowpass` were in that state until they were \
                         repaired; both now load in KiCad 10.0.6.)\n\
                         \x20 Fix: install KiCad 8+, or, from a machine that has it, export the \
                         intermediate netlist and import that instead:\n    \
                         kicad-cli sch export python-bom -o circuit.xml circuit.kicad_sch\n    \
                         melange import circuit.xml -o circuit.cir"
                    );
                }
            }
        }
        _ => {
            bail!(
                "kicad-cli not found. Install KiCad 8+ to import .kicad_sch files directly.\n\
                 Alternatively, export XML manually:\n  \
                 kicad-cli sch export python-bom -o circuit.xml circuit.kicad_sch\n  \
                 melange import circuit.xml -o circuit.cir"
            );
        }
    }

    println!("  Source: {}", input.display());

    // Export XML to a temp file
    let tmp = tempfile::NamedTempFile::with_suffix(".xml")
        .context("Failed to create temp file for XML export")?;
    let tmp_path = tmp.path().to_path_buf();

    let export = std::process::Command::new("kicad-cli")
        .args(["sch", "export", "python-bom", "-o"])
        .arg(&tmp_path)
        .arg(input)
        .output()
        .context("Failed to run kicad-cli")?;

    if !export.status.success() {
        let stderr = String::from_utf8_lossy(&export.stderr);
        bail!("kicad-cli export failed:\n{stderr}");
    }

    let xml_content =
        std::fs::read_to_string(&tmp_path).context("Failed to read kicad-cli XML output")?;

    let result = import_xml(&xml_content)?;

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
                // quick-xml >= 0.41 no longer unescapes entities inside text and,
                // with its reference-splitting tokenizer, delivers a text run in
                // multiple pieces: plain spans arrive as `Text`, while each
                // `&ref;` / `&#NN;` arrives as a separate `GeneralRef` event
                // (handled below). Text spans carry no unresolved entities, so
                // decode() alone is the faithful replacement for the old
                // `BytesText::unescape()`. Accumulate; `current_text` is reset at
                // each element Start/Empty (see the `current_text.clear()` below).
                if let Ok(s) = e.decode() {
                    current_text.push_str(&s);
                }
            }
            Ok(Event::GeneralRef(ref e)) => {
                // A character/entity reference the tokenizer split out of the
                // surrounding text. Reconstruct `&name;` and run it through
                // escape::unescape so predefined entities (amp/lt/gt/quot/apos)
                // and numeric char refs (`&#107;` / `&#x6B;`) resolve uniformly —
                // together with the Text arm this reproduces the old
                // `BytesText::unescape()` result. An unresolvable custom entity is
                // preserved in raw `&name;` form rather than silently dropped.
                if let Ok(name) = e.decode() {
                    let raw = format!("&{name};");
                    match quick_xml::escape::unescape(&raw) {
                        Ok(resolved) => current_text.push_str(&resolved),
                        Err(_) => current_text.push_str(&raw),
                    }
                }
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
                                comp.fields
                                    .insert(current_field_name.clone(), current_text.clone());
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
            Err(e) => bail!(
                "XML parse error at position {}: {}",
                reader.error_position(),
                e
            ),
            _ => {}
        }
        buf.clear();
    }

    // Two net-name checks before anything is emitted. Both guard against the
    // generated deck being quietly wrong rather than obviously broken: a
    // collision solders two schematic nets together, and an auto-named
    // `Net-(#PWR..)` net means a power symbol never became a global net at all.
    check_node_name_collisions(&nets)?;
    warn_unmarked_power_symbol_nets(&nets);

    // Now generate the .cir file
    let mut lines = Vec::new();
    lines.push(title);

    // Collect models (deduplicated)
    let mut models: BTreeMap<String, String> = BTreeMap::new();
    let mut comp_lines = Vec::new();
    let mut pot_directives = Vec::new();
    let mut wiper_directives = Vec::new();
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
            match comp.pin_nets.get(pin) {
                Some(n) => sanitize_node(n),
                None => {
                    eprintln!(
                        "  Warning: {} pin {} is unconnected — tying to ground (0). \
                         Check the schematic if this pin should carry signal.",
                        comp.ref_des, pin
                    );
                    "0".into()
                }
            }
        };
        // format_value with an empty-value warning carrying the ref-des context.
        let fmt_value = |v: &str| -> String {
            if v.trim().is_empty() {
                eprintln!(
                    "  Warning: {} has an empty value — emitting 0. \
                     Edit the generated .cir before compiling.",
                    comp.ref_des
                );
            }
            format_value(v)
        };

        let line = match prefix.as_str() {
            "RW" => {
                // Wiper pot: 3 pins (CW, W, CCW) → two resistor legs + .wiper directive
                let cw_node = get_node("1");
                let w_node = get_node("2");
                let ccw_node = get_node("3");
                let wiper_field = comp
                    .fields
                    .get("Melange.Wiper")
                    .cloned()
                    .unwrap_or_default();
                let parts: Vec<&str> = wiper_field.split_whitespace().collect();
                let total_r = parts.first().copied().unwrap_or("100k");
                let default_pos = parts.get(1).copied().unwrap_or("0.5");
                let label = comp
                    .fields
                    .get("Melange.Label")
                    .cloned()
                    .unwrap_or_default();

                // Parse total resistance to compute half for nominal values
                let half_r = melange_solver::parser::parse_value(total_r)
                    .map(|v| format!("{}", v / 2.0))
                    .unwrap_or_else(|_| total_r.to_string());

                let cw_ref = format!("{}_cw", comp.ref_des);
                let ccw_ref = format!("{}_ccw", comp.ref_des);

                // Emit two resistor elements
                comp_lines.push(format!("{cw_ref} {cw_node} {w_node} {half_r}"));
                comp_lines.push(format!("{ccw_ref} {w_node} {ccw_node} {half_r}"));
                comp_count += 2;

                // Emit .wiper directive
                let mut w_str = format!(".wiper {cw_ref} {ccw_ref} {total_r} {default_pos}");
                if !label.is_empty() {
                    w_str.push_str(&format!(" \"{label}\""));
                }
                wiper_directives.push(w_str);

                // Check for gang
                if let Some(gang_label) = comp.fields.get("Melange.Gang") {
                    let inverted = comp
                        .fields
                        .get("Melange.GangInvert")
                        .map(|v| v.to_lowercase() == "true")
                        .unwrap_or(false);
                    gang_entries.push((gang_label.clone(), cw_ref.clone(), inverted));
                }

                continue; // already pushed to comp_lines
            }
            "R" | "C" | "L" => {
                let n1 = get_node("1");
                let n2 = get_node("2");
                let val = fmt_value(&comp.value);
                // Check for pot
                if let Some(pot_field) = comp.fields.get("Melange.Pot") {
                    let label = comp
                        .fields
                        .get("Melange.Label")
                        .cloned()
                        .unwrap_or_default();
                    let parts: Vec<&str> = pot_field.split_whitespace().collect();
                    let mut pot_str = format!(
                        ".pot {} {} {}",
                        comp.ref_des,
                        parts.first().unwrap_or(&"1k"),
                        parts.get(1).unwrap_or(&"100k")
                    );
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
                    let label = comp
                        .fields
                        .get("Melange.Label")
                        .cloned()
                        .unwrap_or_default();
                    let mut sw_str = format!(".switch {} {}", comp.ref_des, sw_field.trim());
                    if !label.is_empty() {
                        sw_str.push_str(&format!(" \"{label}\""));
                    }
                    switch_directives.push(sw_str);
                }
                // Check for gang
                if let Some(gang_label) = comp.fields.get("Melange.Gang") {
                    let inverted = comp
                        .fields
                        .get("Melange.GangInvert")
                        .map(|v| v.to_lowercase() == "true")
                        .unwrap_or(false);
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
                let nb = if comp.pin_nets.contains_key("4") {
                    get_node("4")
                } else {
                    ns.clone()
                };
                collect_model(comp, "", &mut models);
                format!(
                    "{} {} {} {} {} {}",
                    comp.ref_des, nd, ng, ns, nb, comp.value
                )
            }
            "T" => {
                let ng = get_node("1");
                let np = get_node("2");
                let nk = get_node("3");
                collect_model(comp, "TRIODE", &mut models);
                format!("{} {} {} {} {}", comp.ref_des, ng, np, nk, comp.value)
            }
            "P" => {
                // Pentode: reorder KiCad pins (G,P,K,G2,G3) to SPICE order (P,G,K,G2,[G3])
                let ng = get_node("1");
                let np = get_node("2");
                let nk = get_node("3");
                let nscr = get_node("4");
                collect_model(comp, "VP", &mut models);
                // Suppressor is optional — include if pin 5 is connected to a non-ground net
                if comp.pin_nets.contains_key("5") {
                    let nsup = get_node("5");
                    if nsup != "0" {
                        format!(
                            "{} {} {} {} {} {} {}",
                            comp.ref_des, np, ng, nk, nscr, nsup, comp.value
                        )
                    } else {
                        format!(
                            "{} {} {} {} {} {}",
                            comp.ref_des, np, ng, nk, nscr, comp.value
                        )
                    }
                } else {
                    format!(
                        "{} {} {} {} {} {}",
                        comp.ref_des, np, ng, nk, nscr, comp.value
                    )
                }
            }
            "U" => {
                let nplus = get_node("1");
                let nminus = get_node("2");
                let nout = get_node("3");
                collect_model(comp, "OA", &mut models);
                format!(
                    "{} {} {} {} {}",
                    comp.ref_des, nplus, nminus, nout, comp.value
                )
            }
            "Y" => {
                let sp = get_node("1");
                let sn = get_node("2");
                let cp = get_node("3");
                let cn = get_node("4");
                collect_model(comp, "VCA", &mut models);
                format!(
                    "{} {} {} {} {} {}",
                    comp.ref_des, sp, sn, cp, cn, comp.value
                )
            }
            "V" => {
                let n1 = get_node("1");
                let n2 = get_node("2");
                let val = fmt_value(&comp.value);
                format!("{} {} {} DC {}", comp.ref_des, n1, n2, val)
            }
            "I" => {
                let n1 = get_node("1");
                let n2 = get_node("2");
                let val = fmt_value(&comp.value);
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
                let val = fmt_value(&comp.value);
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
    directives.extend(wiper_directives.clone());
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
    println!("  Wipers: {}", wiper_directives.len());
    println!("  Switches: {}", switch_directives.len());
    println!("  Gangs: {}", gang_groups.len());

    let cir_content = lines.join("\n");

    // Validate the generated netlist by parsing it through melange's parser
    validate_generated_netlist(&cir_content);

    Ok(cir_content)
}

fn validate_generated_netlist(content: &str) {
    match melange_solver::parser::Netlist::parse(content) {
        Ok(_netlist) => {
            println!("  Validation: OK (netlist parses cleanly)");
        }
        Err(e) => {
            eprintln!("  Warning: Generated netlist has parse errors:");
            eprintln!("    {e}");
            eprintln!("    The output file was still written. Fix manually or re-import.");
        }
    }
}

fn collect_model(comp: &XmlComponent, default_type: &str, models: &mut BTreeMap<String, String>) {
    if models.contains_key(&comp.value) {
        return;
    }
    if let Some(model_field) = comp.fields.get("Melange.Model") {
        models.insert(
            comp.value.clone(),
            format!(".model {} {}", comp.value, model_field),
        );
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
        if lower.starts_with('.')
            || trimmed
                .chars()
                .next()
                .map(|c| c.is_ascii_alphabetic())
                .unwrap_or(false)
        {
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

    let cir_content = lines.join("\n");
    validate_generated_netlist(&cir_content);

    Ok(cir_content)
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
    ref_des
        .chars()
        .take_while(|c| c.is_ascii_alphabetic())
        .collect()
}

/// Net names KiCad's *ground* power symbols carry, after the leading sheet-path
/// separator is stripped and case is folded.
///
/// A KiCad power symbol drives a global net named after its Value field, so
/// `power:GND` produces a net literally called `GND`. `0` and `ground` are here
/// because melange's own parser aliases both to the reference node
/// (`parser::normalize_node_name`), so a schematic labelled either way must land
/// on `0` in the emitted deck rather than on a node named after the label.
///
/// Deliberately NOT included: `AGND`, `DGND`, `GNDA`, `GNDD`, `GNDPWR`,
/// `GNDREF`, `VSS`, `EARTH`. KiCad ships power symbols for all of them and they
/// are *separate nets* from `GND` in the schematic that uses them — a board with
/// both `GND` and `AGND` has drawn two nets on purpose. Folding them together
/// here would silently rewire the circuit, so they import as ordinary named
/// nodes and the author ties them to `0` explicitly if that is what they meant.
const GROUND_NET_NAMES: [&str; 3] = ["0", "gnd", "ground"];

/// Convert a KiCad net name into a melange node name.
///
/// KiCad's ground power symbols map onto melange's reference node `0`; every
/// other net — including non-ground power symbols such as `VCC`, `+15V` and
/// `-15V` — becomes an ordinary named node, because that is what they are. A
/// rail is a net the circuit still has to drive: importing `+15V` as a named
/// node leaves the author a node to hang a `V` source on, whereas dropping it
/// or folding it into `0` would short the rail to ground.
fn sanitize_node(name: &str) -> String {
    // A root-sheet local label exports as `/in`; a global net (which is what a
    // power symbol makes) exports bare. Strip one leading separator before the
    // ground test so `/GND` and `GND` agree.
    let trimmed = name.trim();
    let unrooted = trimmed.strip_prefix('/').unwrap_or(trimmed);
    if GROUND_NET_NAMES.contains(&unrooted.to_lowercase().as_str()) {
        return "0".into();
    }

    // A leading sign is part of a rail's identity, not punctuation: `+15V` and
    // `-15V` both collapse to `15v` under the generic map below, which would
    // short a dual supply into one node. Carry the sign across as the SPICE
    // spelling (`p`/`n`) instead. Interior signs stay generic — `Net-(C1-Pad1)`
    // is an auto-generated name, not a signed quantity — and anything that
    // still collides is caught by `check_node_name_collisions`.
    let (sign, rest) = match unrooted.strip_prefix('+') {
        Some(r) => ("p", r),
        None => match unrooted.strip_prefix('-') {
            Some(r) => ("n", r),
            None => ("", unrooted),
        },
    };

    let s: String = rest
        .trim_start_matches('/')
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    let s = s.trim_matches('_').to_lowercase();
    if s.is_empty() {
        return if sign.is_empty() {
            "node".into()
        } else {
            sign.into()
        };
    }
    if !sign.is_empty() {
        return format!("{sign}{s}");
    }
    if s.chars()
        .next()
        .map(|c| c.is_ascii_digit())
        .unwrap_or(false)
    {
        format!("n{s}")
    } else {
        s
    }
}

/// Refuse to emit a deck in which two distinct KiCad nets sanitize to the same
/// melange node name.
///
/// `sanitize_node` is lossy — it folds case and rewrites every character SPICE
/// cannot carry — so distinct schematic nets can land on one node. That failure
/// is invisible in the output: the deck parses, simulates, and quietly has two
/// nets soldered together. `+15V` and `-15V` used to do exactly this (both
/// became `n15v`, shorting a dual supply). Fold-to-ground is the one legitimate
/// merge and is exempt: `GND`, `gnd`, `ground` and `0` genuinely are one node.
fn check_node_name_collisions(nets: &[(String, Vec<(String, String)>)]) -> Result<()> {
    let mut by_node: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for (net_name, _) in nets {
        let node = sanitize_node(net_name);
        if node == "0" {
            continue; // ground aliases are meant to merge
        }
        by_node.entry(node).or_default().insert(net_name.clone());
    }
    for (node, sources) in &by_node {
        if sources.len() > 1 {
            let list: Vec<&str> = sources.iter().map(|s| s.as_str()).collect();
            bail!(
                "KiCad nets {} all import as melange node '{}', which would short them \
                 together in the generated deck.\n\
                 \x20 Node names are folded to lower case and stripped of characters SPICE \
                 cannot carry, so distinct schematic nets can collide.\n\
                 \x20 Fix: rename the nets in the schematic so they stay distinct after that \
                 folding, then re-export.",
                list.iter()
                    .map(|s| format!("'{s}'"))
                    .collect::<Vec<_>>()
                    .join(", "),
                node
            );
        }
    }
    Ok(())
}

/// Warn about nets KiCad auto-named after a `#`-referenced symbol.
///
/// KiCad gives a symbol a `#` reference prefix when it is not a real part —
/// power symbols (`#PWR`), ERC flags (`#FLG`), and melange's own I/O markers
/// (`#AIN`, `#AOUT`). Such a symbol only creates a *global net* if its library
/// definition is marked `(power)`. Without that token KiCad treats it as an
/// ordinary symbol, gives it no net name of its own, and the net falls back to
/// the auto-generated `Net-(#REF-PIN)` form.
///
/// For a ground symbol that is silent and fatal: the deck imports with no
/// reference node at all. melange's bundled `rc-lowpass` example shipped in
/// exactly that state — its embedded `power:GND` was missing `(power)`, so the
/// import produced `net___pwr01_gnd` instead of `0`. Name the cause rather than
/// leaving the author to work backwards from a mystery node.
fn warn_unmarked_power_symbol_nets(nets: &[(String, Vec<(String, String)>)]) {
    for (net_name, _) in nets {
        let Some(inner) = net_name
            .strip_prefix("Net-(#")
            .and_then(|s| s.strip_suffix(')'))
        else {
            continue;
        };
        let (sym, pin) = match inner.split_once('-') {
            Some((s, p)) => (s, p),
            None => (inner, ""),
        };
        eprintln!(
            "  Warning: net '{net_name}' is an auto-generated name, not a net the schematic \
             names. KiCad only turns symbol #{sym} into a named global net if its library \
             definition carries the `(power)` token; without it the symbol connects nothing \
             beyond the wire it sits on. If #{sym} (pin {pin}) is a ground or supply symbol, \
             replace it with one from KiCad's stock `power` library and re-export — otherwise \
             this net imports as the ordinary node '{node}'.",
            node = sanitize_node(net_name)
        );
    }
}

/// Strip trailing unit letters (Ω, F, H, V, A) from a KiCad value string.
///
/// `f`/`F` (farad) and `a`/`A` (ampere) double as SPICE engineering prefixes
/// (femto, atto). They are only stripped when preceded by another letter —
/// i.e. an engineering prefix, as in "4.7nF" — never when preceded by a digit:
/// "100f" means 100 femto-units and must survive intact ("100 F" would be a
/// 15-order-of-magnitude mangling). Ω and V are never prefixes and are always
/// stripped.
fn format_value(value: &str) -> String {
    if value.is_empty() {
        return "0".into();
    }
    let mut s = value.trim().to_string();
    while let Some(last) = s.chars().last() {
        if !"ΩFHVAfhva".contains(last) {
            break;
        }
        if matches!(last, 'f' | 'F' | 'a' | 'A') {
            // Ambiguous letter: unit only when it follows an engineering
            // prefix letter (e.g. "4.7nF" → strip); after a digit it IS the
            // engineering prefix (e.g. "100f" femtofarad → keep).
            let follows_letter = s
                .chars()
                .rev()
                .nth(1)
                .map(|p| p.is_alphabetic() && p != 'Ω')
                .unwrap_or(false);
            if !follows_letter {
                break;
            }
        }
        s.pop();
    }
    if s.is_empty() {
        value.trim().to_string()
    } else {
        s
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn kicad_cli_major_version_parses_real_banners() {
        use super::kicad_cli_major_version;
        // The version actually installed on the machine that hit this wall.
        assert_eq!(kicad_cli_major_version("7.0.11"), Some(7));
        assert_eq!(kicad_cli_major_version("8.0.1"), Some(8));
        assert_eq!(kicad_cli_major_version("  9.0.0-rc1  "), Some(9));
        assert_eq!(kicad_cli_major_version("KiCad 8.0.4"), Some(8));
        assert_eq!(kicad_cli_major_version("10.0.0"), Some(10));
        // Unparseable must NOT be treated as old — the caller only blocks on a
        // version it could actually read.
        assert_eq!(kicad_cli_major_version(""), None);
        assert_eq!(kicad_cli_major_version("unknown"), None);
    }

    use super::*;

    // Regression guard for the quick-xml 0.41 migration (0.1.1). Its
    // reference-splitting tokenizer no longer keeps `&#NN;` / `&amp;` inside
    // `Event::Text`; each reference is emitted as a separate `Event::GeneralRef`.
    // A naive port that only handles `Text` silently DROPS every entity — e.g.
    // a value written `10&#107;` collapsed to `10`. These decks exercise both a
    // numeric char reference and a named predefined entity end-to-end through
    // `import_xml`, so a future regression fails loudly instead of mangling a
    // netlist.
    #[test]
    fn import_xml_resolves_numeric_char_reference_in_value() {
        // `&#107;` is 'k'; `&#48;` is '0'. Correct resolution yields "10k".
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<export version="E">
  <design><source>/tmp/entity-test.kicad_sch</source></design>
  <components>
    <comp ref="R1"><value>1&#48;&#107;</value>
      <libsource lib="Device" part="R"/></comp>
  </components>
  <nets>
    <net code="1" name="in"><node ref="R1" pin="1"/></net>
    <net code="2" name="out"><node ref="R1" pin="2"/></net>
  </nets>
</export>"#;
        let cir = import_xml(xml).expect("import_xml should succeed");
        assert!(
            cir.contains("R1 in out 10k"),
            "numeric char reference not resolved; got:\n{cir}"
        );
        // The pre-fix bug produced "R1 in out 10" (references dropped).
        assert!(
            !cir.contains("R1 in out 10\n"),
            "reference silently dropped:\n{cir}"
        );
    }

    #[test]
    fn import_xml_resolves_named_entity_in_text() {
        // Named predefined entity in text content (the title, derived from the
        // <source> element's text). `&amp;` must resolve to '&': the file stem of
        // "hi&lo.kicad_sch" is "hi&lo", so the title line is "hi&lo". A dropped
        // reference would fuse the spans to "hilo"; a raw one would read
        // "hi&amp;lo" — both distinct from the correct "hi&lo".
        // (Note: entity resolution here applies to TEXT nodes only; attribute
        // values such as net `name="..."` are read raw in both 0.37 and 0.41.)
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<export version="E">
  <design><source>/tmp/hi&amp;lo.kicad_sch</source></design>
  <components>
    <comp ref="R1"><value>1k</value><libsource lib="Device" part="R"/></comp>
  </components>
  <nets>
    <net code="1" name="in"><node ref="R1" pin="1"/></net>
    <net code="2" name="out"><node ref="R1" pin="2"/></net>
  </nets>
</export>"#;
        let cir = import_xml(xml).expect("import_xml should succeed");
        let title = cir.lines().next().unwrap_or_default();
        assert_eq!(
            title, "hi&lo",
            "named entity in text not resolved to '&'; got title {title:?}\nfull:\n{cir}"
        );
    }

    #[test]
    fn format_value_keeps_digit_preceded_femto_and_atto() {
        // `f`/`a` after a digit are SPICE engineering prefixes, not unit
        // letters. Stripping them turned 100 fF into 100 F (15 orders off).
        assert_eq!(format_value("100f"), "100f");
        assert_eq!(format_value("2.2a"), "2.2a");
        assert_eq!(format_value("100F"), "100F");
    }

    #[test]
    fn format_value_strips_unit_after_prefix_letter() {
        assert_eq!(format_value("4.7nF"), "4.7n");
        assert_eq!(format_value("10pF"), "10p");
        assert_eq!(format_value("100uF"), "100u");
        assert_eq!(format_value("10uH"), "10u");
        assert_eq!(format_value("100mA"), "100m");
    }

    #[test]
    fn format_value_leaves_plain_engineering_values() {
        assert_eq!(format_value("2.2k"), "2.2k");
        assert_eq!(format_value("470"), "470");
        assert_eq!(format_value("1Meg"), "1Meg");
    }

    #[test]
    fn format_value_strips_unambiguous_units() {
        // Ω and V are never engineering prefixes — always stripped,
        // even directly after a digit.
        assert_eq!(format_value("470Ω"), "470");
        assert_eq!(format_value("2.2kΩ"), "2.2k");
        assert_eq!(format_value("9V"), "9");
        assert_eq!(format_value("100H"), "100");
    }

    #[test]
    fn format_value_empty_becomes_zero() {
        assert_eq!(format_value(""), "0");
    }

    // ─── Ground / power-symbol net mapping ──────────────────────────
    //
    // KiCad power symbols never appear in the exported `<components>` list —
    // their references start with `#` and KiCad excludes those from the BOM —
    // so the importer only ever sees them as net *names*. That makes
    // `sanitize_node` the whole of melange's power-symbol handling, and these
    // tests pin its contract.

    #[test]
    fn sanitize_node_maps_kicad_ground_symbols_to_reference_node() {
        // `power:GND` drives a global net literally named "GND".
        assert_eq!(sanitize_node("GND"), "0");
        assert_eq!(sanitize_node("gnd"), "0");
        // A root-sheet local label exports with the sheet-path separator.
        assert_eq!(sanitize_node("/GND"), "0");
        // melange's parser also aliases "ground"; the emitted deck should say 0.
        assert_eq!(sanitize_node("GROUND"), "0");
        assert_eq!(sanitize_node("0"), "0");
        assert_eq!(sanitize_node("/0"), "0");
    }

    #[test]
    fn sanitize_node_keeps_non_ground_power_symbols_as_named_nodes() {
        // Supply rails are nets, not ground: they import as ordinary nodes the
        // author can hang a source on.
        assert_eq!(sanitize_node("VCC"), "vcc");
        assert_eq!(sanitize_node("VDD"), "vdd");
        // Analog/digital grounds are SEPARATE nets in a schematic that draws
        // them, so they must NOT be folded into 0 behind the author's back.
        assert_eq!(sanitize_node("AGND"), "agnd");
        assert_eq!(sanitize_node("GNDREF"), "gndref");
    }

    #[test]
    fn sanitize_node_keeps_dual_supply_rails_distinct() {
        // Regression: the generic character map turned every non-alphanumeric
        // into '_', so "+15V" and "-15V" both trimmed to "15v" and then took
        // the digit-guard prefix to become "n15v" — one node. A dual supply
        // imported with its rails shorted together and nothing said so.
        assert_eq!(sanitize_node("+15V"), "p15v");
        assert_eq!(sanitize_node("-15V"), "n15v");
        assert_ne!(sanitize_node("+15V"), sanitize_node("-15V"));
        assert_eq!(sanitize_node("+5V"), "p5v");
        assert_eq!(sanitize_node("-12V"), "n12v");
        assert_eq!(sanitize_node("+3V3"), "p3v3");
    }

    #[test]
    fn sanitize_node_leaves_autogenerated_net_names_alone() {
        // KiCad's auto-name for an unnamed net carries interior hyphens that
        // are punctuation, not signs. Machine-generated but correct — leave it.
        assert_eq!(sanitize_node("Net-(C1-Pad1)"), "net__c1_pad1");
        assert_eq!(sanitize_node("Net-(#PWR01-GND)"), "net___pwr01_gnd");
        assert_eq!(sanitize_node("/in"), "in");
        assert_eq!(sanitize_node("/out"), "out");
    }

    #[test]
    fn collision_check_refuses_nets_that_would_be_shorted() {
        // Two distinct schematic nets landing on one node name is a silent
        // rewire of the circuit, so it is a refusal, not a warning.
        let nets = vec![("Vout+".to_string(), vec![]), ("Vout-".to_string(), vec![])];
        let err =
            check_node_name_collisions(&nets).expect_err("nets that fold together must be refused");
        let msg = err.to_string();
        assert!(msg.contains("Vout+") && msg.contains("Vout-"), "{msg}");
        assert!(msg.contains("short"), "{msg}");
    }

    #[test]
    fn collision_check_allows_ground_aliases_to_merge() {
        // GND / gnd / ground / 0 really are one node; merging them is correct.
        let nets = vec![
            ("GND".to_string(), vec![]),
            ("0".to_string(), vec![]),
            ("ground".to_string(), vec![]),
            ("in".to_string(), vec![]),
        ];
        check_node_name_collisions(&nets).expect("ground aliases must be allowed to merge");
    }

    #[test]
    fn import_xml_puts_kicad_ground_on_node_zero() {
        // End-to-end through the XML reader: a `power:GND` net becomes 0.
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<export version="E">
  <design><source>/tmp/gnd-test.kicad_sch</source></design>
  <components>
    <comp ref="C1"><value>0.1u</value><libsource lib="Device" part="C"/></comp>
  </components>
  <nets>
    <net code="1" name="/in"><node ref="C1" pin="1"/></net>
    <net code="2" name="GND"><node ref="C1" pin="2"/></net>
  </nets>
</export>"#;
        let cir = import_xml(xml).expect("import_xml should succeed");
        assert!(
            cir.contains("C1 in 0 0.1u"),
            "ground not mapped to 0:\n{cir}"
        );
    }

    #[test]
    fn import_xml_keeps_supply_rail_as_a_named_node() {
        // A `power:VCC` symbol is a net, not ground and not something to drop.
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<export version="E">
  <design><source>/tmp/vcc-test.kicad_sch</source></design>
  <components>
    <comp ref="R1"><value>10k</value><libsource lib="Device" part="R"/></comp>
  </components>
  <nets>
    <net code="1" name="VCC"><node ref="R1" pin="1"/></net>
    <net code="2" name="GND"><node ref="R1" pin="2"/></net>
  </nets>
</export>"#;
        let cir = import_xml(xml).expect("import_xml should succeed");
        assert!(
            cir.contains("R1 vcc 0 10k"),
            "rail not kept as a node:\n{cir}"
        );
    }

    #[test]
    fn import_xml_does_not_invent_a_ground_for_an_unmarked_power_symbol() {
        // This is the net name melange's own rc-lowpass example produced before
        // its embedded `power:GND` was marked `(power)`: KiCad never made it a
        // global net, so it is NOT a ground and must not be guessed into one.
        // The same `Net-(#REF-PIN)` shape also comes from `#FLG` ERC flags and
        // melange's own `#AIN`/`#AOUT` markers, which are not grounds either.
        // The correct outcome is an ordinary node plus the existing no-ground
        // diagnostic — see `warn_unmarked_power_symbol_nets`.
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<export version="E">
  <design><source>/tmp/unmarked.kicad_sch</source></design>
  <components>
    <comp ref="C1"><value>0.1u</value><libsource lib="Device" part="C"/></comp>
  </components>
  <nets>
    <net code="1" name="/in"><node ref="C1" pin="1"/></net>
    <net code="2" name="Net-(#PWR01-GND)"><node ref="C1" pin="2"/></net>
  </nets>
</export>"#;
        let cir = import_xml(xml).expect("import_xml should succeed");
        assert!(
            cir.contains("C1 in net___pwr01_gnd 0.1u"),
            "an unnamed net must import as itself, not as ground:\n{cir}"
        );
    }

    // ─── End-to-end: the shipped example imports to the shipped reference ───

    /// Repo-root path, derived from this crate's manifest dir (`tools/melange-cli`).
    fn repo_path(rel: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(rel)
    }

    /// A deck reduced to what is electrically true about it: for every element,
    /// its kind, its value, and the pair of nodes it bridges.
    type Edge = (char, f64, [String; 2]);

    fn edges(cir: &str, what: &str) -> Vec<Edge> {
        let netlist = melange_solver::parser::Netlist::parse(cir)
            .unwrap_or_else(|e| panic!("{what} must parse: {e}\n{cir}"));
        use melange_solver::parser::Element;
        netlist
            .elements
            .iter()
            .map(|el| match el {
                Element::Resistor {
                    n_plus,
                    n_minus,
                    value,
                    ..
                } => ('R', *value, [n_plus.clone(), n_minus.clone()]),
                Element::Capacitor {
                    n_plus,
                    n_minus,
                    value,
                    ..
                } => ('C', *value, [n_plus.clone(), n_minus.clone()]),
                Element::Inductor {
                    n_plus,
                    n_minus,
                    value,
                    ..
                } => ('L', *value, [n_plus.clone(), n_minus.clone()]),
                // Fail loud rather than quietly comparing a subset: if the
                // fixture grows a transistor, this comparator has to grow too.
                other => panic!(
                    "{what} contains an element this topology comparator does not \
                     model yet: {other:?}"
                ),
            })
            .collect()
    }

    /// Canonical form of one element: kind, value, and its two endpoints sorted,
    /// with free (schematic-internal) nodes replaced via `map`. R and C are
    /// symmetric two-terminal parts, so endpoint order carries no information.
    fn canon(e: &Edge, map: &BTreeMap<String, String>) -> (char, String, String, String) {
        let mut ends: Vec<String> =
            e.2.iter()
                .map(|n| map.get(n).cloned().unwrap_or_else(|| n.clone()))
                .collect();
        ends.sort();
        // Values come from identical source strings on both sides; format to a
        // fixed precision so the comparison is on the number, not the bits.
        (
            e.0,
            format!("{:.12e}", e.1),
            ends[0].clone(),
            ends[1].clone(),
        )
    }

    /// True when the two decks are the same circuit. Ground and the named I/O
    /// nodes are pinned by name; every other node is a schematic internal whose
    /// name is free to differ, so they are matched by structure — any bijection
    /// that makes the element multisets equal proves the topologies agree.
    fn electrically_equivalent(a: &[Edge], b: &[Edge]) -> bool {
        const PINNED: [&str; 3] = ["0", "in", "out"];
        let free = |es: &[Edge]| -> Vec<String> {
            let mut v: Vec<String> = es
                .iter()
                .flat_map(|e| e.2.iter().cloned())
                .filter(|n| !PINNED.contains(&n.as_str()))
                .collect();
            v.sort();
            v.dedup();
            v
        };
        let (fa, fb) = (free(a), free(b));
        if fa.len() != fb.len() || a.len() != b.len() {
            return false;
        }
        assert!(
            fa.len() <= 6,
            "brute-force node matching is only for small fixtures ({} free nodes)",
            fa.len()
        );

        let target: BTreeSet<_> = b.iter().map(|e| canon(e, &BTreeMap::new())).collect();
        let mut perm: Vec<usize> = (0..fb.len()).collect();
        loop {
            let map: BTreeMap<String, String> = fa
                .iter()
                .cloned()
                .zip(perm.iter().map(|&i| fb[i].clone()))
                .collect();
            let mapped: BTreeSet<_> = a.iter().map(|e| canon(e, &map)).collect();
            if mapped == target && mapped.len() == a.len() {
                return true;
            }
            if !next_permutation(&mut perm) {
                return false;
            }
        }
    }

    fn next_permutation(v: &mut [usize]) -> bool {
        if v.len() < 2 {
            return false;
        }
        let Some(i) = (0..v.len() - 1).rev().find(|&i| v[i] < v[i + 1]) else {
            return false;
        };
        let j = (i + 1..v.len()).rev().find(|&j| v[j] > v[i]).unwrap();
        v.swap(i, j);
        v[i + 1..].reverse();
        true
    }

    /// The whole KiCad path, end to end, with no KiCad installed: the committed
    /// `rc-lowpass.xml` (real Eeschema 10.0.6 output) must import to the same
    /// circuit as the hand-written `rc-lowpass-reference.cir` that ships beside
    /// it — same parts, same values, same topology, and ground on node 0.
    ///
    /// This is the regression guard for the ground defect. Before the example
    /// schematic's `power:GND` was marked `(power)`, KiCad emitted the net as
    /// `Net-(#PWR01-GND)`, the import produced `C1 net__c1_pad1 net___pwr01_gnd`,
    /// and the deck had no reference node at all — the MNA system was solving
    /// relative to a reference melange picked for itself and ngspice would have
    /// rejected the deck outright.
    #[test]
    fn shipped_kicad_example_imports_to_the_shipped_reference_circuit() {
        let xml_path = repo_path("kicad/examples/rc-lowpass/rc-lowpass.xml");
        let ref_path = repo_path("kicad/examples/rc-lowpass/rc-lowpass-reference.cir");
        let xml = std::fs::read_to_string(&xml_path)
            .unwrap_or_else(|e| panic!("{}: {e}", xml_path.display()));
        let reference = std::fs::read_to_string(&ref_path)
            .unwrap_or_else(|e| panic!("{}: {e}", ref_path.display()));

        let imported = import_xml(&xml).expect("the shipped example must import");

        let imp = edges(&imported, "imported deck");
        let rfr = edges(&reference, "reference deck");

        // Ground first: this is the defect the test exists for, so report it on
        // its own rather than as an opaque topology mismatch.
        assert!(
            imp.iter().any(|e| e.2.iter().any(|n| n == "0")),
            "imported deck has no node 0 — KiCad's ground symbol did not map to \
             melange's reference node. Nodes seen: {:?}\n{imported}",
            imp.iter()
                .flat_map(|e| e.2.clone())
                .collect::<BTreeSet<_>>()
        );

        assert!(
            electrically_equivalent(&imp, &rfr),
            "imported deck is not the reference circuit.\n\
             imported: {imp:?}\nreference: {rfr:?}\n\n{imported}"
        );
    }
}
