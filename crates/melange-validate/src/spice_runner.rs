//! Execute ngspice and capture transient analysis output
//!
//! This module provides a wrapper around ngspice for running transient simulations
//! and parsing the output into structured data for comparison with melange solver results.

use std::collections::HashMap;
use std::path::Path;
use std::process::Command;
use thiserror::Error;

/// Errors that can occur when running SPICE simulations
#[derive(Debug, Error, Clone)]
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
pub fn run_transient(
    netlist_path: &Path,
    _tstep: f64,
    _tstop: f64,
    nodes_to_capture: &[String],
) -> Result<SpiceData, SpiceError> {
    // Check if ngspice is available
    if Command::new("ngspice").arg("--version").output().is_err() {
        return Err(SpiceError::NgspiceNotFound);
    }

    // Run ngspice directly on the netlist (which already has .TRAN and .PRINT)
    let output = Command::new("ngspice")
        .arg("-b")  // Batch mode
        .arg(netlist_path)
        .output()?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    
    if !output.status.success() && stderr.contains("error") {
        return Err(SpiceError::SimulationFailed(stderr.to_string()));
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

    // Build PWL source line (SPICE format: Vname node+ node- PWL(t1 v1 t2 v2 ...))
    let pwl_source_line = format!("V{} in 0 PWL({})", source_name.to_uppercase(), pwl_string);

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
            // Replace the entire source line with PWL version
            // Find comment position to preserve comments
            let comment_pos = line.find('*').unwrap_or(line.len());
            let comment = &line[comment_pos..];
            if comment.starts_with('*') {
                modified_lines.push(format!("{} {}", pwl_source_line, comment));
            } else {
                modified_lines.push(pwl_source_line.clone());
            }
            source_replaced = true;
        } else if trimmed_upper.starts_with("*PWL_SOURCE_") {
            // Replace placeholder
            modified_lines.push(pwl_source_line.clone());
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
        if let Some(pos) = end_pos {
            content.insert_str(pos, &format!("{}\n", pwl_source_line));
            content
        } else {
            // No .END found, just append
            format!("{}\n{}\n", content, pwl_source_line)
        }
    } else {
        modified_lines.join("\n")
    };

    let temp_dir = std::env::temp_dir();
    let modified_path = temp_dir.join(format!("melange_pwl_{}.cir", std::process::id()));

    let mut file = std::fs::File::create(&modified_path)?;
    file.write_all(modified_content.as_bytes())?;

    Ok(ModifiedNetlist {
        netlist_path: modified_path,
    })
}

struct ModifiedNetlist {
    netlist_path: std::path::PathBuf,
}

impl Drop for ModifiedNetlist {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.netlist_path);
    }
}

/// Parse ngspice printed output format (from .PRINT TRAN statements)
///
/// This handles the ASCII table output format that ngspice produces when
/// running with .PRINT statements in the netlist.
fn parse_printed_output(
    output: &str,
    expected_nodes: &[String],
) -> Result<SpiceData, SpiceError> {
    let mut spice_data = SpiceData::default();
    let mut column_names: Vec<String> = Vec::new();
    let mut in_data_section = false;
    
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
            column_names.clear();
            for (i, part) in parts.iter().enumerate() {
                if i == 0 {
                    continue; // Skip "Index"
                }
                column_names.push(part.to_string());
            }
            in_data_section = true;
            continue;
        }
        
        // Skip separator lines (dashes)
        if trimmed.starts_with("---") || trimmed.starts_with("==") {
            continue;
        }
        
        // Parse data lines
        if in_data_section && trimmed.chars().next().map(|c| c.is_ascii_digit()).unwrap_or(false) {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            
            if parts.len() >= 2 {
                // First column is index, second is time
                if let Ok(time) = parts[1].parse::<f64>() {
                    spice_data.time.push(time);
                    
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
            "No simulation data found in ngspice output".to_string()
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

    let abs_val = value.abs();
    let exponent = abs_val.log10().floor() as i32;
    let mantissa = value / 10f64.powi(exponent);

    format!("{:e}", mantissa * 10f64.powi(exponent))
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
}
