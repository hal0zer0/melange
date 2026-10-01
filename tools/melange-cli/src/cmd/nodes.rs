use crate::circuits;
use crate::common::load_circuit_text;
use anyhow::{Context, Result};

pub(crate) fn list_nodes_source(circuit_source: &circuits::CircuitSource) -> Result<()> {
    use melange_solver::{mna::MnaSystem, parser::Netlist};

    println!("melange nodes");
    println!("  Source: {}", circuit_source.name());
    println!();

    let netlist_str = load_circuit_text(circuit_source, &|l| println!("{l}"))?;

    let mut netlist =
        Netlist::parse(&netlist_str).with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        netlist
            .expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
    }

    // Topology gate: the wiring defects a solver cannot see. A typo'd node
    // name invents a node and floats whatever it was on, and every number
    // melange prints afterwards is correct for the circuit it was handed. One
    // implementation for every verb — `melange_solver::topology`.
    // `nodes` REPORTS and never refuses, and says so by calling
    // `topology_report` rather than the gate: this is the command a user
    // reaches for to FIND the typo, so no finding at any severity may stop it
    // (a `.port` naming a node the deck lacks refuses everywhere else).
    // `Ports::inferred` guesses the input port from a node named `in` — enough
    // to keep the report free of input-coupling-cap islands the other verbs do
    // not see — and a `.port` declaration supersedes the guess about where the
    // deck's edges are, which is what drops the "no port was declared for this
    // run" hedge from the messages.
    melange_solver::pipeline::topology_report(
        &netlist,
        &melange_solver::topology::Ports::inferred(&netlist).with_deck_pins(&netlist),
        &|m| println!("{m}"),
    );

    let mna = MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;

    // Unrecognized `.model` keys. `compile`/`simulate`/`analyze` hard-error on
    // these; `nodes` was the one inspection command that stayed silent, so a
    // deck could be read here, look clean, and then be refused downstream.
    // Warn (never error) — `nodes` exists to show what a deck contains, and
    // refusing to list a pot range because a diode card has a typo'd key would
    // be the worse trade. See `model_params::warn_unknown_keys_on_referenced_models`.
    melange_solver::model_params::warn_unknown_keys_on_referenced_models(&netlist);

    // Say the count out loud AND say what it counts: `nodes` lists ground,
    // `dc-op`'s N does not, and `analyze`/`compile`'s N adds the augmented
    // constraint rows on top. Three legitimate numbers for one circuit —
    // see `format_system_size`.
    println!(
        "Nodes in circuit: {} entries (ground + {} circuit nodes)",
        mna.n + 1,
        mna.n
    );
    println!("  (0) GND - Ground reference");

    let mut nodes: Vec<_> = mna.node_map.iter().collect();
    nodes.sort_by(|a, b| a.1.cmp(b.1));

    for (name, &idx) in nodes {
        if name != "0" {
            println!("  ({}) {}", idx, name);
        }
    }

    if !mna.nonlinear_devices.is_empty() {
        println!();
        println!("Nonlinear devices:");
        for dev in &mna.nonlinear_devices {
            println!(
                "  {}: {:?} (dimension: {})",
                dev.name, dev.device_type, dev.dimension
            );
        }
    }

    // Op-amps are stamped into the linear system (rail limits applied on top),
    // so they are not among the nonlinear devices above; list them on their
    // own so a deck's op-amp is visibly there.
    let opamps: Vec<String> = netlist
        .elements
        .iter()
        .filter_map(|e| match e {
            melange_solver::parser::Element::Opamp {
                name,
                n_plus,
                n_minus,
                n_out,
                model,
            } => Some(format!(
                "  {name}: model {model} (+in {n_plus}, -in {n_minus}, out {n_out})"
            )),
            _ => None,
        })
        .collect();
    if !opamps.is_empty() {
        println!();
        println!("Op-amps:");
        for line in &opamps {
            println!("{line}");
        }
    }

    // Controls: the names a user needs for --pot / --switch. Either the
    // human-readable label OR the component name is accepted, so print both.
    if !netlist.pots.is_empty()
        || !netlist.switches.is_empty()
        || !netlist.wipers.is_empty()
        || !netlist.gangs.is_empty()
    {
        println!();
        println!("Controls (name or label works with --pot / --switch):");
        // A `.wiper` emits two pots — the halves of its track. The plugin
        // makes the wiper ONE knob, so it is listed as one control, with its
        // halves under it: they are real setters in the generated API (and
        // `--pot` takes either half in ohms), so hiding them would mislead a
        // plugin author.
        let is_wiper_half = |r: &str| {
            netlist
                .wipers
                .iter()
                .any(|w| w.resistor_cw == r || w.resistor_ccw == r)
        };
        // "R_vol_a (10..99990 ohm)", or the bare name if it has no pot entry.
        let half = |r: &str| {
            netlist
                .pots
                .iter()
                .find(|p| p.resistor_name == r)
                .map(|p| format!("{r} ({:.0}..{:.0} ohm)", p.min_value, p.max_value))
                .unwrap_or_else(|| r.to_string())
        };
        for pot in netlist
            .pots
            .iter()
            .filter(|p| !is_wiper_half(&p.resistor_name))
        {
            let label = pot.label.as_deref().unwrap_or(&pot.resistor_name);
            // No explicit default means the resistor's own declared value —
            // `mna.rs` resolves it with `default_value.unwrap_or(*value)`. The
            // number is knowable, so print it rather than the word "nominal",
            // which reads like a missing value next to every other pot's figure.
            let default = pot
                .default_value
                .or_else(|| {
                    netlist.elements.iter().find_map(|e| match e {
                        melange_solver::parser::Element::Resistor { name, value, .. }
                            if name.eq_ignore_ascii_case(&pot.resistor_name) =>
                        {
                            Some(*value)
                        }
                        _ => None,
                    })
                })
                .map(|d| format!("{d:.0}"))
                .unwrap_or_else(|| "nominal".to_string());
            println!(
                "  pot     {:<26} [{}]  {:.0}..{:.0} ohm, default {}",
                format!("\"{label}\""),
                pot.resistor_name,
                pot.min_value,
                pot.max_value,
                default,
            );
        }
        for wiper in &netlist.wipers {
            let label = wiper.label.as_deref().unwrap_or(&wiper.resistor_cw);
            println!(
                "  wiper   {:<26} [{}/{}]  total {:.0} ohm, position 0..1 (one knob)",
                format!("\"{label}\""),
                wiper.resistor_cw,
                wiper.resistor_ccw,
                wiper.total_resistance
            );
            println!(
                "          its two halves: cw {}, ccw {}; each also takes --pot <half>=<ohms>",
                half(&wiper.resistor_cw),
                half(&wiper.resistor_ccw),
            );
        }
        for sw in &netlist.switches {
            let label = sw
                .label
                .clone()
                .unwrap_or_else(|| sw.component_names.join(","));
            println!(
                "  switch  {:<26} {} positions (controls {})",
                format!("\"{label}\""),
                sw.positions.len(),
                sw.component_names.join(",")
            );
        }
        for gang in &netlist.gangs {
            println!(
                "  gang    {:<26} {} members, position 0..1",
                format!("\"{}\"", gang.label),
                gang.members.len()
            );
        }
    }

    Ok(())
}
