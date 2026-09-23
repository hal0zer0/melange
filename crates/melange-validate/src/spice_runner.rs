//! Execute ngspice and capture transient analysis output
//!
//! This module provides a wrapper around ngspice for running transient simulations
//! and parsing the output into structured data for comparison with melange solver results.

use std::collections::HashMap;
use std::path::Path;
use std::process::Command;
use std::sync::atomic::{AtomicU64, Ordering};
use thiserror::Error;

/// Thread-safe counter for unique temp file names
static TEMP_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Errors that can occur when running SPICE simulations
#[derive(Debug, Error, Clone)]
#[non_exhaustive]
pub enum SpiceError {
    /// ngspice executable not found in PATH
    #[error("ngspice not found in PATH. Please install ngspice.")]
    NgspiceNotFound,

    /// Simulation failed with an error message
    #[error("ngspice simulation failed: {0}")]
    SimulationFailed(String),

    /// Failed to parse ngspice output
    #[error("Failed to parse SPICE output: {0}")]
    ParseError(String),

    /// IO error when reading/writing files
    #[error("IO error: {0}")]
    IoError(String),

    /// Specified node not found in output
    #[error("Node '{0}' not found in SPICE output")]
    NodeNotFound(String),

    /// The deck would not describe the same circuit to both engines, so the
    /// reference run was never started.
    ///
    /// See [`crate::deck_guard`]. This is deliberately raised *before* ngspice
    /// is invoked: once a correlation number exists, a caveat next to it reads
    /// as a footnote rather than a retraction.
    #[error("{0}")]
    DeckNotComparable(String),
}

impl From<std::io::Error> for SpiceError {
    fn from(e: std::io::Error) -> Self {
        SpiceError::IoError(e.to_string())
    }
}

/// Data structure for SPICE simulation results
#[derive(Debug, Clone, Default)]
pub struct SpiceData {
    /// Time points from the simulation
    pub time: Vec<f64>,
    /// Node voltages: node_name -> samples
    pub voltages: HashMap<String, Vec<f64>>,
    /// Currents through voltage sources: source_name -> samples
    pub currents: HashMap<String, Vec<f64>>,
    /// Sample rate inferred from tstep
    pub sample_rate: f64,
    /// Actual time step used (may differ from requested tstep)
    pub actual_tstep: f64,
}

impl SpiceData {
    /// Get voltage samples for a specific node
    pub fn get_node_voltage(&self, node: &str) -> Result<&[f64], SpiceError> {
        let normalized = normalize_node_name(node);
        self.voltages
            .get(&normalized)
            .or_else(|| self.voltages.get(node))
            .map(|v| v.as_slice())
            .ok_or_else(|| SpiceError::NodeNotFound(node.to_string()))
    }

    /// Get number of samples in the simulation
    pub fn len(&self) -> usize {
        self.time.len()
    }

    /// Check if simulation has no data
    pub fn is_empty(&self) -> bool {
        self.time.is_empty()
    }

    /// Get sample rate inferred from time data
    pub fn inferred_sample_rate(&self) -> f64 {
        if self.time.len() < 2 {
            return 0.0;
        }
        let dt = self.time[1] - self.time[0];
        if dt > 0.0 {
            1.0 / dt
        } else {
            self.sample_rate
        }
    }
}

/// Run ngspice on a netlist and extract transient analysis data
///
/// # Arguments
///
/// * `netlist_path` - Path to the SPICE netlist file (.cir)
/// * `tstep` - Time step for the transient analysis
/// * `tstop` - Stop time for the transient analysis
/// * `nodes_to_capture` - List of node names to capture voltages for (if empty, captures all)
///
/// # Returns
///
/// Returns `SpiceData` containing time points and node voltages, or a `SpiceError`.
///
/// # Example
///
/// ```rust,no_run
/// use std::path::Path;
/// use melange_validate::spice_runner::run_transient;
///
/// let data = run_transient(
///     Path::new("tests/data/rc_lowpass/circuit.cir"),
///     1e-6,  // 1us timestep
///     10e-3, // 10ms duration
///     &vec!["out".to_string(), "in".to_string()],
/// ).expect("Simulation failed");
///
/// println!("Captured {} samples at {} Hz", data.len(), data.sample_rate);
/// ```
/// Scan a deck for floating cap-only DC islands and emit a loud warning naming
/// each one's nodes.
///
/// A *floating cap-only DC island* is a set of nodes connected to the rest of
/// the circuit and to ground ONLY through elements that are open at DC.
/// ngspice's DC operating-point solve is singular over such an island; the
/// `rshunt=1e12` option injected into the reference deck regularizes it
/// (matching melange's own per-node gmin) so the deck now converges and
/// validates. That silently turns what used to be a hard DC-non-convergence
/// abort into a quiet pass — so a genuine wiring defect (a node that lost its
/// only real resistor and is now accidentally cap-only) would regularize and
/// pass unnoticed. Naming the islands lets the author confirm the intended
/// coupling-cap islands and catch the accidental ones.
///
/// **The analysis is not local any more.** This used to be a private union-find
/// here that unioned every terminal of every non-capacitor element and knew
/// nothing about the input port — which gave it a false positive on the input
/// node of every cap-coupled deck (the whole guitar-pedal category) and false
/// negatives wherever a terminal that does not conduct at DC held an island
/// together (an op-amp input, a FET gate, a tube grid). It is now a consumer of
/// `melange_solver::topology`, which models the DC graph melange actually
/// stamps, port conductance included. Only the framing below is validate's own,
/// because only validate knows about `rshunt`.
///
/// `input_node` is the node the harness attaches its Thevenin PWL drive to, and
/// therefore a real DC path to ground on both sides of the comparison.
///
/// Best-effort: if the deck cannot be parsed, the scan is skipped silently —
/// its job is to add a warning, never to fail a validation that would otherwise
/// run.
fn warn_floating_cap_only_islands(deck_content: &str, input_node: &str) {
    use melange_solver::parser::Netlist;
    use melange_solver::topology::{self, Finding, Ports};

    let netlist = match Netlist::parse(deck_content) {
        Ok(n) => n,
        Err(_) => return,
    };

    let ports = Ports::declared([input_node.to_string()], std::iter::empty());
    for finding in topology::check(&netlist, &ports) {
        let Finding::FloatingIsland { nodes } = finding else {
            // Dangling terminals are refused on the melange side of the
            // comparison (`run_melange_solver_from_str`), where the refusal
            // can stop the run instead of footnoting it.
            continue;
        };
        log::warn!(
            "validate: auto-regularized floating cap-only DC island {{{}}} — \
             ngspice would go singular here; rshunt covers it. Confirm this is an \
             intended coupling-cap island, not a wiring defect.",
            nodes.join(", ")
        );
    }
}

pub fn run_transient(
    netlist_path: &Path,
    tstep: f64,
    tstop: f64,
    nodes_to_capture: &[String],
) -> Result<SpiceData, SpiceError> {
    use std::io::Write;

    // Check if ngspice is available
    if Command::new("ngspice").arg("--version").output().is_err() {
        return Err(SpiceError::NgspiceNotFound);
    }

    // Read original netlist
    let original_content = std::fs::read_to_string(netlist_path)?;

    // Create modified netlist with updated .TRAN and .PRINT statements
    let mut modified_content = String::new();
    let mut first_line = true;
    let mut has_tran = false;
    let mut has_print = false;

    for line in original_content.lines() {
        let trimmed_upper = line.trim().to_uppercase();

        // After copying the title line (first line in SPICE is always the title),
        // inject .OPTIONS INTERP to force uniform output timestep.
        // Without this, ngspice prints at all internal adaptive timestep points,
        // which gives non-uniform time spacing and breaks index-based comparison.
        //
        // reltol=1e-4 (vs ngspice's 1e-3 default) tightens the reference's own
        // truncation error so the comparison measures melange against a better
        // reference. Measured 2026-07-18 across the full 20-test suite: every
        // correlation held or improved (no test moved by more than its noise
        // floor), so it is applied unconditionally rather than per config class.
        if first_line {
            modified_content.push_str(line);
            modified_content.push('\n');
            modified_content.push_str(".OPTIONS INTERP reltol=1e-4\n");
            // Mirror melange's global gmin regularization in the ngspice
            // reference: melange stamps GMIN_REGULARISATION = 1e-12 S on every
            // node diagonal (codegen/ir/mod.rs), and rshunt=1e12 adds the
            // identical 1e-12 S to ground on every analog node. This
            // de-singularizes floating cap-only DC islands (a coupling-cap pot
            // island / a two-cap node has no DC path) EXACTLY as melange does,
            // node-for-node, so the two engines regularize the same nodes the
            // same way. Switch-state-safe by construction (applies to whatever
            // nodes the elaborated deck has, no per-deck topology analysis).
            modified_content.push_str(".options rshunt=1e12\n");
            first_line = false;
            continue;
        }

        if let Some(action) = translate_melange_directive(line) {
            // melange-only directive: either strip it (ngspice can't parse it)
            // or substitute an ngspice-equivalent element line.
            if let Some(replacement) = action {
                modified_content.push_str(&replacement);
                modified_content.push('\n');
            }
            continue;
        } else if trimmed_upper.starts_with(".TRAN") {
            // Replace with uniform timestep for sample-accurate comparison
            // Use the specified tstep and tstop instead of netlist values
            has_tran = true;
            modified_content.push_str(&format!(
                ".TRAN {} {}\n",
                format_scientific(tstep),
                format_scientific(tstop)
            ));
        } else if trimmed_upper.starts_with(".OPTIONS") {
            // Keep deck-author .OPTIONS lines: ngspice merges multiple
            // .OPTIONS statements, so author options coexist with the
            // injected INTERP/reltol line instead of being discarded.
            // (Discarding them silently altered decks that set e.g. their
            // own reltol/abstol/gmin.)
            modified_content.push_str(line);
            modified_content.push('\n');
        } else if trimmed_upper.starts_with(".PRINT") {
            // Update .PRINT to capture at our timestep
            has_print = true;
            modified_content.push_str(&format!(
                ".PRINT TRAN {}\n",
                nodes_to_capture
                    .iter()
                    .map(|n| format!("V({})", n))
                    .collect::<Vec<_>>()
                    .join(" ")
            ));
        } else if trimmed_upper != ".END" {
            // Copy every line except the final `.END` (re-added at the end).
            // Exact match, NOT starts_with(".END"): a generated triode subckt's
            // `.ends` terminator also starts with ".END" and must survive.
            modified_content.push_str(line);
            modified_content.push('\n');
        }
    }
    // Inject analysis cards the source deck never declared. Most
    // melange-circuits netlists carry no .TRAN/.PRINT (they exist to be
    // *compiled*, not ngspice-run); without this the deck reaches ngspice with
    // no transient analysis and no print, and parse_printed_output fails with
    // "No simulation data found in ngspice output". validate owns
    // tstep/tstop/nodes_to_capture, so we can supply a complete card. Flags are
    // independent, so a deck declaring only one of the two still gets the other
    // supplied. The rewrite branches above take precedence when the deck *does*
    // declare its own.
    if !has_tran {
        modified_content.push_str(&format!(
            ".TRAN {} {}\n",
            format_scientific(tstep),
            format_scientific(tstop)
        ));
    }
    if !has_print {
        modified_content.push_str(&format!(
            ".PRINT TRAN {}\n",
            nodes_to_capture
                .iter()
                .map(|n| format!("V({})", n))
                .collect::<Vec<_>>()
                .join(" ")
        ));
    }

    // Ensure .END is present
    modified_content.push_str(".END\n");

    // Write to temp file
    let temp_dir = std::env::temp_dir();
    let temp_path = temp_dir.join(format!(
        "melange_tran_{}_{}.cir",
        std::process::id(),
        TEMP_COUNTER.fetch_add(1, Ordering::Relaxed)
    ));
    {
        let mut file = std::fs::File::create(&temp_path)?;
        file.write_all(modified_content.as_bytes())?;
    }

    // Run ngspice on the modified netlist
    let output = Command::new("ngspice")
        .arg("-b") // Batch mode
        .arg(&temp_path)
        .output()?;

    // Clean up temp file
    let _ = std::fs::remove_file(&temp_path);

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() && stderr.to_lowercase().contains("error") {
        return Err(SpiceError::SimulationFailed(stderr.to_string()));
    }

    // Fail early if ngspice's DC solve did not converge. These print as
    // warnings (not a non-zero exit, not "error"), so the transient runs on from
    // a garbage operating point and the comparison would silently report nonsense
    // (e.g. a floating cap-only DC island with no DC path to ground → singular
    // matrix → corr ~0.14 reported as if it were a real result). Name the root
    // cause instead of comparing garbage.
    {
        let combined_lc = format!("{stdout}\n{stderr}").to_lowercase();
        const DC_FAIL_SIGNATURES: &[&str] = &[
            "singular matrix",
            "gmin stepping failed",
            "source stepping failed",
            "iteration limit reached",
        ];
        if let Some(sig) = DC_FAIL_SIGNATURES
            .iter()
            .find(|s| combined_lc.contains(**s))
        {
            let combined = format!("{stdout}\n{stderr}");
            let detail: String = combined
                .lines()
                .filter(|l| {
                    let ll = l.to_lowercase();
                    ll.contains(sig) || ll.contains("check node")
                })
                .map(str::trim)
                .take(4)
                .collect::<Vec<_>>()
                .join("; ");
            return Err(SpiceError::SimulationFailed(format!(
                "ngspice DC did not converge ({sig}) — the transient would run from a garbage \
                 operating point (common cause: a floating cap-only DC island with no DC path to \
                 ground). ngspice said: {detail}"
            )));
        }
    }

    // Parse the printed output (not raw file - the .PRINT output goes to stdout)
    let spice_data = parse_printed_output(&stdout, nodes_to_capture)?;

    Ok(spice_data)
}

/// Run ngspice with a PWL (piecewise linear) input source
///
/// This is useful for validating against specific input signals.
pub fn run_transient_with_pwl(
    netlist_path: &Path,
    tstep: f64,
    tstop: f64,
    pwl_source_name: &str,
    pwl_data: &[(f64, f64)], // (time, voltage) pairs
    nodes_to_capture: &[String],
) -> Result<SpiceData, SpiceError> {
    // Create a temporary netlist with the PWL source
    let modified_netlist = inject_pwl_source(netlist_path, pwl_source_name, pwl_data)?;

    run_transient(
        modified_netlist.netlist_path.as_path(),
        tstep,
        tstop,
        nodes_to_capture,
    )
}

/// Inject a PWL voltage source into a netlist
///
/// This function replaces an existing voltage source with a PWL (piecewise linear)
/// source while preserving the original node connections. It correctly parses
/// the original source line to extract node names (not hardcoded to "in 0").
fn inject_pwl_source(
    original_netlist: &Path,
    source_name: &str,
    pwl_data: &[(f64, f64)],
) -> Result<ModifiedNetlist, SpiceError> {
    use std::io::Write;

    // Build PWL string
    let pwl_string: String = pwl_data
        .iter()
        .map(|(t, v)| format!("{} {}", format_scientific(*t), format_scientific(*v)))
        .collect::<Vec<_>>()
        .join(" ");

    // Read original netlist
    let original_content = std::fs::read_to_string(original_netlist)?;

    // Case-insensitive search for existing voltage source
    let lines: Vec<&str> = original_content.lines().collect();
    let mut modified_lines = Vec::new();
    let mut source_replaced = false;

    // Pattern to match: V<name> where name matches source_name case-insensitively
    let source_pattern = format!("V{}", source_name.to_uppercase());

    for line in lines {
        let trimmed = line.trim();
        let trimmed_upper = trimmed.to_uppercase();

        // Check if this line is the voltage source definition (case-insensitive)
        if trimmed_upper.starts_with(&source_pattern) {
            // Parse the original source line to extract node names
            // Format: Vname n+ n- [DC value] [AC ...] or Vname n+ n- PWL(...)
            let parts: Vec<&str> = trimmed.split_whitespace().collect();

            // We need at least: Vname n+ n-
            if parts.len() >= 3 {
                let node_plus = parts[1];
                let node_minus = parts[2];

                // Build PWL source line with correct node names
                let pwl_source_line = format!(
                    "V{} {} {} PWL({})",
                    source_name.to_uppercase(),
                    node_plus,
                    node_minus,
                    pwl_string
                );

                // Find comment position to preserve comments
                let comment_pos = line.find('*').unwrap_or(line.len());
                let comment = &line[comment_pos..];
                if comment.starts_with('*') {
                    modified_lines.push(format!("{} {}", pwl_source_line, comment));
                } else {
                    modified_lines.push(pwl_source_line);
                }
                source_replaced = true;
            } else {
                // Malformed source line, keep it as-is
                modified_lines.push(line.to_string());
            }
        } else if trimmed_upper.starts_with("*PWL_SOURCE_") {
            // Replace placeholder - use default nodes "in" and "0"
            let pwl_source_line =
                format!("V{} in 0 PWL({})", source_name.to_uppercase(), pwl_string);
            modified_lines.push(pwl_source_line);
            source_replaced = true;
        } else {
            modified_lines.push(line.to_string());
        }
    }

    // If source wasn't found and no placeholder, add it before .END
    let modified_content = if !source_replaced {
        // Look for .END and insert before it
        let mut content = modified_lines.join("\n");
        let end_pos = content.to_uppercase().find(".END");

        // Default nodes for auto-added source
        let default_pwl_line = format!("V{} in 0 PWL({})", source_name.to_uppercase(), pwl_string);

        if let Some(pos) = end_pos {
            content.insert_str(pos, &format!("{}\n", default_pwl_line));
            content
        } else {
            // No .END found, just append
            format!("{}\n{}\n", content, default_pwl_line)
        }
    } else {
        modified_lines.join("\n")
    };

    let temp_dir = std::env::temp_dir();
    let modified_path = temp_dir.join(format!(
        "melange_pwl_{}_{}.cir",
        std::process::id(),
        TEMP_COUNTER.fetch_add(1, Ordering::Relaxed)
    ));

    let mut file = std::fs::File::create(&modified_path)?;
    file.write_all(modified_content.as_bytes())?;

    Ok(ModifiedNetlist {
        netlist_path: modified_path,
    })
}

pub(crate) struct ModifiedNetlist {
    pub(crate) netlist_path: std::path::PathBuf,
}

impl Drop for ModifiedNetlist {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.netlist_path);
    }
}

/// Check if a line is a melange-specific directive that ngspice doesn't understand
///
/// A deck carrying one cannot be fed to ngspice unmodified (ngspice errors
/// `unimplemented dot command` and the whole run fails). The set is NOT
/// maintained here: it is `melange_solver::parser::MELANGE_ONLY_DIRECTIVES`,
/// which the parser's drift-guard test keeps in lockstep with
/// `parse_directive` — so a new melange directive is stripped here the moment
/// the parser learns it. (This list drifted twice when it was hand-kept:
/// `.integrator` and `.inject` were both missed.)
///
/// Matching is on the whole first token, case-insensitive, so `.POTX ...`
/// does not match `.pot`.
///
/// Notes on what stripping means per directive (none has an ngspice
/// equivalent element; the reference simply does not carry the hint):
/// - `.linearize <dev>` collapses a device to its small-signal stamp on the
///   melange side; the ngspice reference runs the full nonlinear device.
/// - `.tap <node> [name]` is a pure readout, no circuit element.
/// - `.integrator <trap|be>` is a melange integrator hint; ngspice always
///   integrates adaptive-trapezoidal. It is NOT an alignment lever.
/// - `.inject` IS listed, but `translate_melange_directive` handles it FIRST
///   (it substitutes a real resistor); this predicate only sees it as a
///   fallback.
fn is_melange_directive(line: &str) -> bool {
    let Some(first) = line.split_whitespace().next() else {
        return false;
    };
    let first = first.to_ascii_lowercase();
    melange_solver::parser::MELANGE_ONLY_DIRECTIVES
        .iter()
        .any(|d| *d == first)
}

/// Translate a melange-only directive line for the ngspice reference deck.
///
/// Returns:
/// - `None` — not a melange directive; the caller keeps the line unchanged.
/// - `Some(None)` — strip the line (ngspice can't parse it, no element behind it).
/// - `Some(Some(repl))` — substitute `repl` (an ngspice-equivalent element line).
///
/// Order matters: `.inject` is checked BEFORE the generic strip because it
/// carries a real element (see below); a bare strip would compare two
/// different circuits.
///
/// `.inject <node> <field> R=<ohms>|RSHUNT=<ohms>` declares an audio-rate
/// injection port. melange stamps a conductance G=1/R from `<node>` to ground as
/// the at-rest injection impedance — present in its DC-OP and every solve — with
/// the injection source held at 0 at rest. The faithful ngspice equivalent is
/// therefore a real resistor of value R from `<node>` to ground. Both `R=`
/// (Thevenin-series) and `RSHUNT=` (Norton-shunt) collapse to the identical
/// at-rest topology (source off), so both map to the same node-to-ground
/// resistor `R_minj_<field> <node> 0 <R-token>`. Bare-stripping would drop a
/// real element melange solves with, comparing two different circuits. Field
/// names are unique per deck, so `R_minj_<field>` is collision-safe. The raw
/// value token is preserved verbatim (ngspice understands SPICE suffixes).
fn translate_melange_directive(line: &str) -> Option<Option<String>> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts
        .first()
        .is_some_and(|t| t.eq_ignore_ascii_case(".inject"))
    {
        if parts.len() >= 4 {
            let node = parts[1];
            let field = parts[2];
            // Value token follows the '=' in the R=/RSHUNT= parameter.
            let r_token = parts[3].split_once('=').map_or(parts[3], |x| x.1);
            return Some(Some(format!("R_minj_{} {} 0 {}", field, node, r_token)));
        }
        // Malformed `.inject` — strip so ngspice still parses the deck.
        return Some(None);
    }
    if is_melange_directive(line) {
        return Some(None);
    }
    None
}

/// Substitute each dynamic element's melange-DEFAULT value into its element line
/// so the ngspice reference deck solves the SAME circuit melange simulates at its
/// default parameter state.
///
/// melange stamps a `.pot`/`.wiper` resistor at `default_value.unwrap_or(nominal)`
/// and a `.switch` component at its position-0 value (a switch's initial state is
/// always position 0). See `mna.rs` (`MnaSystem::from_netlist`). The reference-deck
/// builder strips those melange-only directive lines but historically left the
/// element at its NETLIST NOMINAL — so any element whose default differs from its
/// nominal made the two engines solve different circuits. Proven on passive-eq1a:
/// `.pot R_lfc 100 100k 100` (default 100 Ω) on the `R_lfc eq h 100k` element
/// (nominal 100 kΩ) → correlation 0.808 instead of ~1.0; matching the nominal to
/// the default restores corr 0.99999.
///
/// This rewrites each such element's value token to melange's default:
/// - `.pot Rname min max [default]`: if `default_value` is Some, the element gets
///   that value; if None, melange uses the nominal too, so the line is left as-is.
/// - `.wiper` legs are covered automatically — the parser's `expand_wipers` pushes
///   two `PotDirective`s into `netlist.pots` with concrete per-leg `default_value`s
///   (the leg resistances at the default position), so they flow through the pot
///   branch above with no special handling here.
/// - `.switch`: each `component_names[i]` element gets `positions[0][i]`.
///
/// `.gang` default positions are deliberately NOT applied. A `.gang` is a
/// plugin-UI construct (one nih-plug FloatParam, see parser.rs): its
/// `default_position` is forwarded to codegen as the UI parameter default but is
/// never baked into melange's default *stamped* state — the member pots keep their
/// own `default_value` / `g_nominal`, and the validate path runs `process_sample`
/// from `CircuitState::default()` with no setter calls. Substituting a gang
/// position would therefore make the reference diverge from what melange actually
/// simulates at default. (Grep confirms `gang` appears only in the codegen IR, in
/// no emitter.)
///
/// Best-effort: a deck that melange's parser cannot read (e.g. one already
/// tube-translated to ngspice subckts, or Thevenin-injected) is returned
/// unchanged. Callers MUST therefore run this on the pristine melange deck BEFORE
/// any ngspice-specific rewrite.
pub(crate) fn substitute_dynamic_element_defaults(deck: &str) -> String {
    use melange_solver::parser::Netlist;

    let netlist = match Netlist::parse(deck) {
        Ok(n) => n,
        Err(_) => return deck.to_string(),
    };

    // element name (uppercase) -> substituted value token (plain SPICE numeric).
    let mut overrides: HashMap<String, String> = HashMap::new();

    // `.pot` (and expanded `.wiper` legs): stamp at `default_value` when present.
    // `default_value = None` means melange uses the nominal, so we leave it alone.
    for pot in &netlist.pots {
        if let Some(dv) = pot.default_value {
            overrides.insert(
                pot.resistor_name.to_ascii_uppercase(),
                format_scientific(dv),
            );
        }
    }

    // `.switch`: melange's initial state is always position 0.
    for sw in &netlist.switches {
        if let Some(pos0) = sw.positions.first() {
            for (i, comp) in sw.component_names.iter().enumerate() {
                if let Some(&val) = pos0.get(i) {
                    overrides.insert(comp.to_ascii_uppercase(), format_scientific(val));
                }
            }
        }
    }

    if overrides.is_empty() {
        return deck.to_string();
    }

    let mut out = String::with_capacity(deck.len() + 32);
    for line in deck.lines() {
        let trimmed = line.trim_start();
        // Only rewrite two-terminal passive element lines: `<R|C|L>name n+ n- value`.
        // Comments, directives, sources and everything else are copied verbatim.
        let first = trimmed.chars().next().unwrap_or(' ').to_ascii_uppercase();
        if matches!(first, 'R' | 'C' | 'L') {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 4 {
                if let Some(newval) = overrides.get(&parts[0].to_ascii_uppercase()) {
                    // Rebuild `name n+ n- <default>`, preserving any trailing tokens.
                    let mut rebuilt = format!("{} {} {} {}", parts[0], parts[1], parts[2], newval);
                    for extra in &parts[4..] {
                        rebuilt.push(' ');
                        rebuilt.push_str(extra);
                    }
                    out.push_str(&rebuilt);
                    out.push('\n');
                    continue;
                }
            }
        }
        out.push_str(line);
        out.push('\n');
    }
    out
}

/// Inject a Thevenin-equivalent PWL source into a netlist string
///
/// Creates a modified netlist with a voltage source + series resistance matching
/// melange's 1-ohm Thevenin input model. If an existing voltage source is found
/// with n+ matching `input_node`, it is replaced; otherwise the pair is inserted.
///
/// The intermediate node `in_mlg_src` avoids collisions with existing node names.
pub(crate) fn inject_thevenin_pwl(
    original_content: &str,
    input_node: &str,
    pwl_data: &[(f64, f64)],
    series_resistance: f64,
) -> Result<ModifiedNetlist, SpiceError> {
    use std::io::Write;

    let pwl_string: String = pwl_data
        .iter()
        .map(|(t, v)| format!("{} {}", format_scientific(*t), format_scientific(*v)))
        .collect::<Vec<_>>()
        .join(" ");

    let input_upper = input_node.to_uppercase();
    let mut modified_lines = Vec::new();
    let mut source_replaced = false;

    for (i, line) in original_content.lines().enumerate() {
        let trimmed = line.trim();

        // Line 0 is ALWAYS the free-text SPICE title, never an element. Pass it
        // through verbatim so a title whose 2nd token equals the input node
        // (e.g. "Valve in preamp") is not mistaken for VIN and rewritten as a
        // Thevenin pair — which would drop the title and mangle the deck.
        if i == 0 {
            modified_lines.push(line.to_string());
            continue;
        }

        // Skip commented lines
        if trimmed.starts_with('*') {
            modified_lines.push(line.to_string());
            continue;
        }

        let trimmed_upper = trimmed.to_uppercase();

        // Check if this line is a voltage source with n+ matching input_node
        if !source_replaced && trimmed_upper.starts_with('V') {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 3 && parts[1].to_uppercase() == input_upper {
                // Found voltage source at input node — replace with Thevenin pair
                // Preserve original source name to avoid breaking I(Vname) references
                let original_name = parts[0];
                let n_minus = parts[2];
                modified_lines.push(format!(
                    "{} in_mlg_src {} PWL({})",
                    original_name, n_minus, pwl_string
                ));
                modified_lines.push(format!(
                    "R_mlg_src in_mlg_src {} {}",
                    input_node, series_resistance
                ));
                source_replaced = true;
                continue;
            }
        }

        modified_lines.push(line.to_string());
    }

    // If no source was found at the input node, insert Thevenin pair before .END
    // Use V_mlg_in (not VIN) to avoid name collisions with existing sources
    if !source_replaced {
        let thevenin_v = format!("V_mlg_in in_mlg_src 0 PWL({})", pwl_string);
        let thevenin_r = format!("R_mlg_src in_mlg_src {} {}", input_node, series_resistance);

        let mut insert_idx = modified_lines.len();
        for (i, line) in modified_lines.iter().enumerate() {
            if line.trim().to_uppercase().starts_with(".END") {
                insert_idx = i;
                break;
            }
        }
        modified_lines.insert(insert_idx, thevenin_v);
        modified_lines.insert(insert_idx + 1, thevenin_r);
    }

    let content = modified_lines.join("\n");

    let temp_dir = std::env::temp_dir();
    let modified_path = temp_dir.join(format!(
        "melange_thev_{}_{}.cir",
        std::process::id(),
        TEMP_COUNTER.fetch_add(1, Ordering::Relaxed)
    ));

    let mut file = std::fs::File::create(&modified_path)?;
    file.write_all(content.as_bytes())?;

    Ok(ModifiedNetlist {
        netlist_path: modified_path,
    })
}

/// Run ngspice with a Thevenin-equivalent PWL input source
///
/// This injects a voltage source + series resistor matching melange's 1-ohm
/// Thevenin input model. Melange-specific directives (.pot, .switch, .input_impedance)
/// are automatically stripped so ngspice can parse the netlist.
pub fn run_transient_with_thevenin_pwl(
    netlist_content: &str,
    tstep: f64,
    tstop: f64,
    input_node: &str,
    pwl_data: &[(f64, f64)],
    series_resistance: f64,
    nodes_to_capture: &[String],
) -> Result<SpiceData, SpiceError> {
    // Masking-risk guardrail: rshunt=1e12 (injected into the reference deck)
    // silently regularizes floating cap-only DC islands so ngspice's DC solve no
    // longer goes singular. That is the intended behavior for genuine
    // coupling-cap islands, but it also masks a wiring defect — a node that lost
    // its only real resistor and became accidentally cap-only would now
    // regularize and pass unnoticed. Scan the melange deck (as-authored, before
    // the Thevenin/PWL injection that melange's own parser rejects; this
    // reflects the current switch state that produced this reference) and name
    // any such island so intended ones are confirmed and accidental ones caught.
    warn_floating_cap_only_islands(netlist_content, input_node);

    // Substitute each dynamic (`.pot`/`.wiper`/`.switch`) element's melange-DEFAULT
    // value into its element line so ngspice solves the SAME circuit melange does at
    // its default parameter state. The directive lines themselves are still stripped
    // downstream in `run_transient`; this only rewrites the element VALUES they
    // govern. Must run on the pristine deck BEFORE the tube/Thevenin rewrites below,
    // which melange's parser cannot read back. See
    // `substitute_dynamic_element_defaults` for the exact melange mapping matched.
    let netlist_content = substitute_dynamic_element_defaults(netlist_content);
    let netlist_content = netlist_content.as_str();

    // Refuse decks the two engines would not read as the same circuit, BEFORE
    // ngspice is invoked. Two classes (see `deck_guard`):
    //
    //  * value tokens each engine reads as a different number — melange accepts
    //    the SI-prefix infix form (`4k7` = 4.7k), ngspice reads the mantissa,
    //    applies the scale letter and discards the rest of the token (`4k7` =
    //    4k). Neither side errors, so a run would report a correlation between
    //    a 4.7k circuit and a 4.0k one and attribute the gap to the solver;
    //  * devices ngspice has no element or model for (op-amp, LDR, VCA, glow).
    //    ngspice's own complaint names the author's `.model` card, which is
    //    correct, so relaying it sends them to fix the wrong line.
    //
    // Scanned AFTER `substitute_dynamic_element_defaults` (pot/switch values are
    // plain numerics by then, so an infix nominal on a pot resistor is correctly
    // not a hazard) and BEFORE the tube/pentode/Thevenin rewrites, whose emitted
    // text is generated, not author-written.
    let hazards = crate::deck_guard::scan_deck(netlist_content);
    if !hazards.is_empty() {
        return Err(SpiceError::DeckNotComparable(
            crate::deck_guard::format_refusal(&hazards),
        ));
    }

    // Translate any melange triode (`T`) elements into Koren B-source subckts
    // before the deck reaches ngspice (which would parse `T` as a transmission
    // line). No-op when the deck has no triode. See tube_translate.rs.
    let translated = crate::tube_translate::translate_tubes_for_ngspice(netlist_content)?;
    // Translate any melange pentode (`P`) elements into Reefman/Koren B-source
    // subckts too (ngspice has no native pentode). No-op when the deck has no
    // pentode. A deck may carry both T and P, so this runs after the triode
    // pass. See pentode_translate.rs.
    // Parse the PRISTINE original (netlist_content) for pentode model params —
    // the triode pass may have injected ngspice `.subckt`/`X` that melange's own
    // parser cannot re-parse — while rewriting the (triode-)translated string.
    let translated =
        crate::pentode_translate::translate_pentodes_for_ngspice(&translated, netlist_content)?;

    // An explicit `.model OA(AOL_TRANSIENT_CAP=…)` makes melange's TRANSIENT G
    // matrix carry a different Gm than its own DC stamp (`codegen::ir::
    // opamp_ir_from_info` subtracts `delta_Gm`). One ngspice `G` card has one
    // transconductance, so whichever we emit is wrong for one of the two
    // phases. Refuse before ngspice runs rather than pick.
    let capped = crate::opamp_translate::transient_aol_cap_opamps(netlist_content);
    if !capped.is_empty() {
        let list = capped
            .iter()
            .map(|(name, model, cap)| format!("{name} (.model {model}, AOL_TRANSIENT_CAP={cap})"))
            .collect::<Vec<_>>()
            .join(", ");
        return Err(SpiceError::DeckNotComparable(format!(
            "op-amp {list} caps the open-loop gain for the transient solve only, so melange's \
             transient G matrix uses a different Gm than its DC operating point. The ngspice \
             reference has one transconductance for both, so no single reference deck matches \
             melange in both phases. Remove `AOL_TRANSIENT_CAP=` for the validation run — it is a \
             conditioning aid for precision-rectifier topologies, not part of the circuit."
        )));
    }

    // Translate any melange op-amp (`U`) elements into the linear VCCS
    // macromodel melange itself stamps (`G` + output `R`, plus RIN/IB when the
    // `.model` sets them). No-op when the deck has no `U` card — including the
    // shipped decks that hand-expanded their op-amps. See opamp_translate.rs.
    let translated =
        crate::opamp_translate::translate_opamps_for_ngspice(&translated, netlist_content)?;

    // The VCCS twin is LINEAR: it has no VCC/VEE rail clamp and no SR slew
    // clamp. Watch each such op-amp's output node on the reference trace so a
    // correlation is never reported across a clamp event melange applied and
    // ngspice did not. Parsed from the PRISTINE deck, for the same reason the
    // pentode pass is.
    let rail_probes = crate::opamp_translate::rail_probes(netlist_content);
    let mut capture: Vec<String> = nodes_to_capture.to_vec();
    for p in &rail_probes {
        if !capture
            .iter()
            .any(|n| n.eq_ignore_ascii_case(p.node.as_str()))
        {
            capture.push(p.node.clone());
        }
    }

    let modified = inject_thevenin_pwl(&translated, input_node, pwl_data, series_resistance)?;
    let spice_data = run_transient(modified.netlist_path.as_path(), tstep, tstop, &capture)?;
    crate::opamp_translate::check_rail_probes(&rail_probes, &spice_data.voltages, tstep)?;
    Ok(spice_data)
}

/// Parse ngspice printed output format (from .PRINT TRAN statements)
///
/// This handles the ASCII table output format that ngspice produces when
/// running with .PRINT statements in the netlist.
///
/// ngspice prints **at most three data columns per table**. A `.PRINT` naming
/// more vectors than that is emitted as consecutive *column blocks*, each one
/// repeating the whole row range for its own subset of vectors (and each block
/// repeating its own header every ~50 rows for pagination):
///
/// ```text
/// Index   time   v(n1)  v(n2)  v(n3)      <- block 1, rows 0..N
/// Index   time   v(n1)  v(n2)  v(n3)      <- same block, page 2
/// Index   time   v(n4)  v(n5)  v(n6)      <- block 2, rows 0..N again
/// ```
///
/// So a repeated header means "keep going" only when the column set is
/// unchanged; a *different* column set restarts the row range, and its `time`
/// column is a duplicate of one already collected. Appending it (as this
/// function did before op-amp rail probes made >3-column captures reachable)
/// left `time` a multiple of the trace length while each voltage stayed at the
/// true length — a silent length mismatch downstream.
fn parse_printed_output(output: &str, expected_nodes: &[String]) -> Result<SpiceData, SpiceError> {
    let mut spice_data = SpiceData::default();
    let mut column_names: Vec<String> = Vec::new();
    let mut in_data_section = false;
    // Row index within the CURRENT column block. `time` is recorded only while
    // this outruns what has already been collected, i.e. only for the first
    // block (and for a later block that is somehow longer).
    let mut row_idx = 0usize;

    for line in output.lines() {
        let trimmed = line.trim();

        // Skip empty lines and comments
        if trimmed.is_empty() || trimmed.starts_with('*') {
            continue;
        }

        // Look for header line: "Index   time            v(out)          v(in)"
        if trimmed.starts_with("Index") && trimmed.contains("time") {
            // Parse header to get column names
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            let new_names: Vec<String> = parts.iter().skip(1).map(|p| p.to_string()).collect();
            // Same columns as the header before it: pagination inside one
            // block, so the row counter carries on. Different columns: a new
            // block, which restarts at row 0.
            if new_names != column_names {
                row_idx = 0;
                column_names = new_names;
            }
            in_data_section = true;
            continue;
        }

        // Skip separator lines (dashes)
        if trimmed.starts_with("---") || trimmed.starts_with("==") {
            continue;
        }

        // Parse data lines
        if in_data_section
            && trimmed
                .chars()
                .next()
                .map(|c| c.is_ascii_digit())
                .unwrap_or(false)
        {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();

            if parts.len() >= 2 {
                // First column is index, second is time
                if let Ok(time) = parts[1].parse::<f64>() {
                    if spice_data.time.len() <= row_idx {
                        spice_data.time.push(time);
                    }
                    row_idx += 1;

                    // Remaining columns are voltages
                    for (i, col_name) in column_names.iter().enumerate().skip(1) {
                        // col_names[0] is "time", so data starts at index 2
                        let data_idx = i + 1;
                        if data_idx < parts.len() {
                            if let Ok(voltage) = parts[data_idx].parse::<f64>() {
                                let normalized = normalize_node_name(col_name);
                                spice_data
                                    .voltages
                                    .entry(normalized)
                                    .or_default()
                                    .push(voltage);
                            }
                        }
                    }
                }
            }
        }
    }

    // Validate that we got data
    if spice_data.time.is_empty() {
        return Err(SpiceError::ParseError(
            "No simulation data found in ngspice output".to_string(),
        ));
    }

    // Verify expected nodes were found
    for node in expected_nodes {
        let normalized = normalize_node_name(node);
        if !spice_data.voltages.contains_key(&normalized) {
            return Err(SpiceError::NodeNotFound(node.clone()));
        }
    }

    // Calculate sample rate
    if spice_data.time.len() >= 2 {
        spice_data.actual_tstep = spice_data.time[1] - spice_data.time[0];
        spice_data.sample_rate = 1.0 / spice_data.actual_tstep;
    }

    Ok(spice_data)
}

/// Normalize node names for consistent lookup
fn normalize_node_name(name: &str) -> String {
    let normalized = name.trim().to_lowercase();
    // Remove common prefixes
    normalized
        .trim_start_matches("v(")
        .trim_end_matches(')')
        .to_string()
}

/// Format a number in scientific notation suitable for SPICE
fn format_scientific(value: f64) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    format!("{:e}", value)
}

/// Check if ngspice is installed and available
pub fn is_ngspice_available() -> bool {
    Command::new("ngspice")
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Get ngspice version string
pub fn get_ngspice_version() -> Option<String> {
    Command::new("ngspice")
        .arg("--version")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim ngspice-42 batch output for a `.PRINT TRAN` naming 8 vectors.
    /// ngspice prints at most three data columns per table, so this is three
    /// column BLOCKS, each repeating rows 0..4. Captured by running the deck,
    /// not hand-written — the parser has to survive what ngspice really emits.
    const MULTI_BLOCK: &str = "\
No. of Data Rows : 5
                               * many column test
                               Transient Analysis  Wed Sep 23 09:00:41  2026
--------------------------------------------------------------------------------
Index   time            v(n1)           v(n2)           v(n3)
--------------------------------------------------------------------------------
0\t0.000000e+00\t0.000000e+00\t0.000000e+00\t0.000000e+00
1\t1.000000e-05\t6.279038e-02\t5.494158e-02\t4.709279e-02
2\t2.000000e-05\t1.253329e-01\t1.096663e-01\t9.399969e-02
3\t3.000000e-05\t1.873809e-01\t1.639583e-01\t1.405357e-01
4\t4.000000e-05\t2.486899e-01\t2.176037e-01\t1.865174e-01

                               * many column test
                               Transient Analysis  Wed Sep 23 09:00:41  2026
--------------------------------------------------------------------------------
Index   time            v(n4)           v(n5)           v(n6)
--------------------------------------------------------------------------------
0\t0.000000e+00\t0.000000e+00\t0.000000e+00\t0.000000e+00
1\t1.000000e-05\t3.924399e-02\t3.139519e-02\t2.354639e-02
2\t2.000000e-05\t7.833307e-02\t6.266646e-02\t4.699984e-02
3\t3.000000e-05\t1.171131e-01\t9.369045e-02\t7.026784e-02
4\t4.000000e-05\t1.554312e-01\t1.243449e-01\t9.325871e-02

                               * many column test
                               Transient Analysis  Wed Sep 23 09:00:41  2026
--------------------------------------------------------------------------------
Index   time            v(n7)           v(n8)
--------------------------------------------------------------------------------
0\t0.000000e+00\t0.000000e+00\t0.000000e+00
1\t1.000000e-05\t1.569760e-02\t7.848798e-03
2\t2.000000e-05\t3.133323e-02\t1.566661e-02
3\t3.000000e-05\t4.684523e-02\t2.342261e-02
4\t4.000000e-05\t6.217247e-02\t3.108624e-02
";

    /// Before op-amp rail probes there was never more than one column block, so
    /// the extra blocks' duplicate `time` column was appended and `time` came
    /// out 3x the length of every voltage. Watching a clamped op-amp's output
    /// node makes >3 captured vectors ordinary, so the blocks have to line up.
    #[test]
    fn multi_column_block_output_is_one_aligned_table() {
        let nodes: Vec<String> = (1..=8).map(|i| format!("n{i}")).collect();
        let data = parse_printed_output(MULTI_BLOCK, &nodes).expect("parse");
        assert_eq!(data.time.len(), 5, "time must not repeat per column block");
        for n in &nodes {
            assert_eq!(data.voltages[n].len(), 5, "node {n}");
        }
        // Values landed under the right names, not shifted by a block.
        assert_eq!(data.voltages["n1"][1], 6.279038e-02);
        assert_eq!(data.voltages["n4"][1], 3.924399e-02);
        assert_eq!(data.voltages["n8"][4], 3.108624e-02);
        // Sample rate still derived from the first two time points.
        assert!((data.actual_tstep - 1.0e-5).abs() < 1e-18);
    }

    /// A header repeated with the SAME columns is ngspice paginating one block
    /// (every ~50 rows), not starting a new one — the row counter must carry on.
    #[test]
    fn repeated_identical_header_is_pagination_not_a_new_block() {
        let paginated = "\
Index   time            v(out)
--------------------------------------------------------------------------------
0\t0.000000e+00\t1.0
1\t1.000000e-05\t2.0
Index   time            v(out)
--------------------------------------------------------------------------------
2\t2.000000e-05\t3.0
3\t3.000000e-05\t4.0
";
        let data = parse_printed_output(paginated, &["out".to_string()]).expect("parse");
        assert_eq!(data.time.len(), 4);
        assert_eq!(data.voltages["out"], vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_normalize_node_name() {
        assert_eq!(normalize_node_name("V(out)"), "out");
        assert_eq!(normalize_node_name("v(in)"), "in");
        assert_eq!(normalize_node_name("  OUT  "), "out");
        assert_eq!(normalize_node_name("NODE1"), "node1");
    }

    #[test]
    fn test_format_scientific() {
        assert_eq!(format_scientific(1e-6), "1e-6");
        assert_eq!(format_scientific(0.0), "0");
        assert!(format_scientific(1.5e-3).contains('e'));
    }

    #[test]
    fn test_spice_data_getters() {
        let mut data = SpiceData::default();
        data.time = vec![0.0, 1e-6, 2e-6];
        data.voltages.insert("out".to_string(), vec![0.0, 0.5, 1.0]);

        assert_eq!(data.len(), 3);
        assert!(!data.is_empty());

        let volts = data.get_node_voltage("out").unwrap();
        assert_eq!(volts, &[0.0, 0.5, 1.0]);

        assert!(data.get_node_voltage("nonexistent").is_err());
    }

    #[test]
    fn test_translate_tap_and_inject_directives() {
        // `.tap` is a pure readout: stripped from the ngspice deck.
        assert_eq!(translate_melange_directive(".tap n1"), Some(None));
        assert_eq!(translate_melange_directive(".tap n1 my_out"), Some(None));

        // `.inject ... R=` converts to a node-to-ground resistor (Thevenin).
        assert_eq!(
            translate_melange_directive(".inject n2 f2 R=1k"),
            Some(Some("R_minj_f2 n2 0 1k".to_string()))
        );
        // `.inject ... RSHUNT=` maps to the SAME node-to-ground resistor (Norton).
        assert_eq!(
            translate_melange_directive(".inject tank_ret wet_return RSHUNT=470k"),
            Some(Some("R_minj_wet_return tank_ret 0 470k".to_string()))
        );

        // Simulate the deck-translation line filter on a small deck and confirm
        // `.tap`/`.inject` do not survive and the R line is present.
        let deck = "* title\nR1 a b 10k\n.tap n1\n.inject n2 f2 R=1k\n.END\n";
        let mut out = String::new();
        for line in deck.lines() {
            if let Some(action) = translate_melange_directive(line) {
                if let Some(replacement) = action {
                    out.push_str(&replacement);
                    out.push('\n');
                }
                continue;
            }
            out.push_str(line);
            out.push('\n');
        }
        assert!(!out.to_uppercase().contains(".TAP"), "deck: {out}");
        assert!(!out.to_uppercase().contains(".INJECT"), "deck: {out}");
        assert!(out.contains("R_minj_f2 n2 0 1k"), "deck: {out}");
    }

    #[test]
    fn test_strip_set_derived_from_parser_directive_list() {
        use melange_solver::parser::MELANGE_ONLY_DIRECTIVES;

        // Every parser-declared melange-only directive leaves the ngspice deck,
        // in any case and with tab or space separation.
        for d in MELANGE_ONLY_DIRECTIVES {
            let lower = format!("{d} x y");
            let upper = format!("{} X Y", d.to_uppercase());
            let tabbed = format!("  {d}\tx");
            for line in [&lower, &upper, &tabbed] {
                assert!(is_melange_directive(line), "{line:?} should be stripped");
                assert!(
                    translate_melange_directive(line).is_some(),
                    "{line:?} must not reach ngspice"
                );
            }
        }
        // The list must cover the two directives that were historically missed.
        assert!(MELANGE_ONLY_DIRECTIVES.contains(&".integrator"));
        assert!(MELANGE_ONLY_DIRECTIVES.contains(&".inject"));
        // `.oversampling` is honored on the shipping path but must be stripped
        // for the (base-rate) ngspice comparison — validate ignores it.
        assert!(MELANGE_ONLY_DIRECTIVES.contains(&".oversampling"));
        assert!(is_melange_directive(".oversampling 4"));

        // Whole-token match: a longer token sharing a prefix is not a hit.
        assert!(!is_melange_directive(".potx R1 1k 100k"));
        assert!(!is_melange_directive(".POTX R1 1k 100k"));
        // Standard SPICE directives and element lines pass through untouched.
        for line in [
            ".model D1N4148 D IS=2.5n",
            ".param k=1",
            ".subckt foo a b",
            ".ends",
            ".end",
            ".tran 1u 1m",
            "R1 a b 1k",
            "",
        ] {
            assert!(!is_melange_directive(line), "{line:?} must pass through");
            assert_eq!(translate_melange_directive(line), None, "{line:?}");
        }
    }

    /// Helper: find the value token (parts[3]) of the element line named `name`.
    fn element_value(deck: &str, name: &str) -> Option<f64> {
        deck.lines().find_map(|l| {
            let p: Vec<&str> = l.split_whitespace().collect();
            if p.first().map(|s| s.eq_ignore_ascii_case(name)) == Some(true) && p.len() >= 4 {
                p[3].parse::<f64>().ok()
            } else {
                None
            }
        })
    }

    #[test]
    fn test_pot_default_substituted_into_deck() {
        // `.pot` default (100) differs from the element nominal (100k): the ngspice
        // reference element line must be rewritten to melange's default value.
        let deck = "test deck\nR_lfc eq h 100k\nR_gnd h 0 1k\n.pot R_lfc 100 100k 100\n.end\n";
        let out = substitute_dynamic_element_defaults(deck);
        assert_eq!(
            element_value(&out, "R_lfc"),
            Some(100.0),
            "pot default not substituted; deck:\n{out}"
        );
        // The directive line itself is still present (stripped later, not here).
        assert!(
            out.to_uppercase().contains(".POT R_LFC"),
            "directive line should survive this pass; deck:\n{out}"
        );
    }

    #[test]
    fn test_pot_no_default_keeps_nominal() {
        // `.pot` with no explicit default: melange uses the nominal, so the element
        // line must be left untouched (keeps its original suffix token, `4700`).
        let deck = "test deck\nR1 a b 4700\nR2 b 0 1k\n.pot R1 100 10k\n.end\n";
        let out = substitute_dynamic_element_defaults(deck);
        assert!(
            out.lines()
                .any(|l| l.split_whitespace().collect::<Vec<_>>() == ["R1", "a", "b", "4700"]),
            "nominal element line should be unchanged; deck:\n{out}"
        );
    }

    #[test]
    fn test_switch_position0_substituted() {
        // A switch's initial state is always position 0. The pos-0 value (270n)
        // differs from the element nominal (999n) and must land in the deck.
        let deck = "test deck\nC_hfc a b 999n\nR_gnd a 0 1k\n.switch C_hfc 270n 135n 68n\n.end\n";
        let out = substitute_dynamic_element_defaults(deck);
        let v = element_value(&out, "C_hfc").expect("C_hfc line missing");
        assert!(
            (v - 2.7e-7).abs() < 1e-15,
            "switch pos-0 not substituted (got {v}); deck:\n{out}"
        );
    }

    #[test]
    fn test_wiper_legs_substituted_at_default_position() {
        // `.wiper` expands (in the parser) into two `.pot`s whose default leg
        // resistances at the default position are what melange stamps. With
        // total=100k, pos=0.85, MIN_LEG_R=10: r_cw = 0.15*(100000-20)+10 = 15007,
        // r_ccw = 0.85*(100000-20)+10 = 84993. Both element lines must be rewritten.
        let deck = "test deck\nR_cw n1 w 1k\nR_ccw w 0 1k\n.wiper R_cw R_ccw 100k 0.85\n.end\n";
        let out = substitute_dynamic_element_defaults(deck);
        let cw = element_value(&out, "R_cw").expect("R_cw line missing");
        let ccw = element_value(&out, "R_ccw").expect("R_ccw line missing");
        assert!((cw - 15007.0).abs() < 1e-6, "R_cw={cw}; deck:\n{out}");
        assert!((ccw - 84993.0).abs() < 1e-6, "R_ccw={ccw}; deck:\n{out}");
    }

    #[test]
    fn test_no_dynamic_directives_is_byte_identical_noop() {
        // A deck with no `.pot`/`.wiper`/`.switch` must be returned unchanged, so
        // every existing non-dynamic golden deck is unaffected.
        let deck = "RC lowpass\nVIN in 0 DC 0 AC 1\nR1 in out 10k\nC1 out 0 10n\n.end\n";
        assert_eq!(substitute_dynamic_element_defaults(deck), deck);
    }
}
