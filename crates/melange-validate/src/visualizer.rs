//! Generate comparison plots and reports for failed validations
//!
//! This module provides functionality to generate HTML reports with embedded SVG plots
//! and CSV export for external analysis (e.g., in Python/MATLAB).

use crate::comparison::{ComparisonReport, Signal};
use std::fs::File;
use std::io::Write;
use std::path::Path;

/// Errors that can occur during report generation
#[derive(Debug, Clone)]
pub enum VisualizerError {
    IoError(String),
    InvalidData(String),
}

impl std::fmt::Display for VisualizerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VisualizerError::IoError(msg) => write!(f, "IO error: {}", msg),
            VisualizerError::InvalidData(msg) => write!(f, "Invalid data: {}", msg),
        }
    }
}

impl std::error::Error for VisualizerError {}

impl From<std::io::Error> for VisualizerError {
    fn from(e: std::io::Error) -> Self {
        VisualizerError::IoError(e.to_string())
    }
}

/// Color palette for SVG plots
struct SvgColors {
    bg: &'static str,
    grid: &'static str,
    text: &'static str,
    axis: &'static str,
    spice: &'static str,
    melange: &'static str,
    error: &'static str,
}

impl Default for SvgColors {
    fn default() -> Self {
        Self {
            bg: "rgb(22,33,62)",
            grid: "rgb(45,45,68)",
            text: "rgb(136,136,136)",
            axis: "rgb(102,102,102)",
            spice: "rgb(78,204,163)",
            melange: "rgb(233,69,96)",
            error: "rgb(233,69,96)",
        }
    }
}

/// Generate HTML report with plots and metrics
pub fn generate_html_report(
    report: &ComparisonReport,
    spice: &Signal,
    melange: &Signal,
    output_path: &Path,
) -> Result<(), VisualizerError> {
    let len = report.sample_count.min(spice.len()).min(melange.len());
    if len == 0 {
        return Err(VisualizerError::InvalidData(
            "No samples to visualize".to_string(),
        ));
    }

    // Generate SVG plots
    let time_data: Vec<f64> = (0..len).map(|i| i as f64 / report.sample_rate).collect();

    let signal_overlay_svg = generate_signal_overlay_svg(&time_data, spice, melange, len)?;
    let error_signal_svg = generate_error_signal_svg(&time_data, spice, melange, len)?;
    let histogram_svg = generate_histogram_svg(report, len)?;
    let scatter_svg = generate_scatter_svg(spice, melange, len)?;

    // Build HTML document
    let status_class = if report.passed { "passed" } else { "failed" };
    let status_text = if report.passed { "PASSED" } else { "FAILED" };

    let html = format_html_report(
        report,
        status_class,
        status_text,
        &signal_overlay_svg,
        &error_signal_svg,
        &histogram_svg,
        &scatter_svg,
    );

    let mut file = File::create(output_path)?;
    file.write_all(html.as_bytes())?;

    Ok(())
}

fn format_html_report(
    report: &ComparisonReport,
    status_class: &str,
    status_text: &str,
    signal_overlay_svg: &str,
    error_signal_svg: &str,
    histogram_svg: &str,
    scatter_svg: &str,
) -> String {
    let colors = SvgColors::default();

    format!(
        r#"<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Melange Validation Report - {circuit_name}</title>
    <style>
        :root {{
            --bg-color: {bg};
            --text-color: #eaeaea;
            --accent-color: #16213e;
            --border-color: #0f3460;
            --pass-color: {spice};
            --fail-color: {melange};
            --grid-color: {grid};
        }}
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            margin: 0;
            padding: 20px;
            line-height: 1.6;
        }}
        h1 {{ color: var(--pass-color); border-bottom: 2px solid var(--border-color); padding-bottom: 10px; }}
        h2 {{ color: var(--text-color); margin-top: 30px; }}
        .status {{
            font-size: 24px;
            font-weight: bold;
            padding: 10px 20px;
            border-radius: 5px;
            display: inline-block;
            margin: 10px 0;
        }}
        .status.passed {{ background-color: var(--pass-color); color: var(--bg-color); }}
        .status.failed {{ background-color: var(--fail-color); color: white; }}
        .metrics {{
            background-color: var(--accent-color);
            border-radius: 8px;
            padding: 20px;
            margin: 20px 0;
        }}
        table {{ width: 100%; border-collapse: collapse; margin: 15px 0; }}
        th, td {{ text-align: left; padding: 12px; border-bottom: 1px solid var(--border-color); }}
        th {{ background-color: var(--border-color); font-weight: 600; }}
        .metric-value {{ font-family: 'Consolas', 'Monaco', monospace; }}
        .failure {{
            color: var(--fail-color);
            background-color: rgba(233, 69, 96, 0.1);
            padding: 10px;
            border-radius: 4px;
            margin: 5px 0;
        }}
        .plot-container {{
            background-color: var(--accent-color);
            border-radius: 8px;
            padding: 20px;
            margin: 20px 0;
        }}
        .plot-container h3 {{ margin-top: 0; color: #a0a0a0; }}
        svg {{ width: 100%; height: auto; max-width: 900px; }}
        .info {{ color: #888; font-size: 14px; }}
    </style>
</head>
<body>
    <h1>Melange Validation Report</h1>
    
    <div class="status {status_class}">{status_text}</div>
    
    <div class="info">
        <strong>Circuit:</strong> {circuit_name}<br>
        <strong>Node:</strong> {node_name}<br>
        <strong>Samples:</strong> {sample_count} at {sample_rate:.2} kHz<br>
        <strong>Duration:</strong> {duration:.4} s
    </div>

    <h2>Metrics</h2>
    <div class="metrics">
        <table>
            <tr><th>Metric</th><th>Value</th><th>Status</th></tr>
            <tr>
                <td>RMS Error</td>
                <td class="metric-value">{rms_error:.6e}</td>
                <td>{rms_status}</td>
            </tr>
            <tr>
                <td>Normalized RMS Error</td>
                <td class="metric-value">{norm_rms:.6} ({norm_rms_pct:.4}%)</td>
                <td>{rms_status}</td>
            </tr>
            <tr>
                <td>Peak Error</td>
                <td class="metric-value">{peak_error:.6e}</td>
                <td>{peak_status}</td>
            </tr>
            <tr>
                <td>Mean Absolute Error</td>
                <td class="metric-value">{mae:.6e}</td>
                <td>-</td>
            </tr>
            <tr>
                <td>Max Relative Error</td>
                <td class="metric-value">{max_rel:.6} ({max_rel_pct:.4}%)</td>
                <td>{rel_status}</td>
            </tr>
            <tr>
                <td>Correlation Coefficient</td>
                <td class="metric-value">{corr:.8}</td>
                <td>{corr_status}</td>
            </tr>
            <tr>
                <td>SNR</td>
                <td class="metric-value">{snr:.2} dB</td>
                <td>-</td>
            </tr>
            <tr>
                <td>THD (SPICE)</td>
                <td class="metric-value">{thd_spice:.2} dB</td>
                <td>-</td>
            </tr>
            <tr>
                <td>THD (Melange)</td>
                <td class="metric-value">{thd_melange:.2} dB</td>
                <td>-</td>
            </tr>
            <tr>
                <td>THD Error</td>
                <td class="metric-value">{thd_err:.2} dB</td>
                <td>{thd_status}</td>
            </tr>
        </table>
    </div>

    {failures_section}

    <h2>Plots</h2>
    
    <div class="plot-container">
        <h3>Signal Overlay</h3>
        {signal_overlay_svg}
    </div>

    <div class="plot-container">
        <h3>Error Signal (Melange - SPICE)</h3>
        {error_signal_svg}
    </div>

    <div class="plot-container">
        <h3>Error Distribution</h3>
        {histogram_svg}
    </div>

    <div class="plot-container">
        <h3>Scatter Plot (SPICE vs Melange)</h3>
        {scatter_svg}
    </div>

    <div class="info">
        <em>Generated by melange-validate</em>
    </div>
</body>
</html>"#,
        bg = colors.bg,
        spice = colors.spice,
        melange = colors.melange,
        grid = colors.grid,
        circuit_name = report.circuit_name,
        node_name = report.node_name,
        sample_count = report.sample_count,
        sample_rate = report.sample_rate / 1000.0,
        duration = report.sample_count as f64 / report.sample_rate,
        rms_error = report.rms_error,
        norm_rms = report.normalized_rms_error,
        norm_rms_pct = report.normalized_rms_error * 100.0,
        rms_status = format_pass(report.normalized_rms_error < 0.001),
        peak_error = report.peak_error,
        peak_status = format_pass(report.peak_error < 0.01),
        mae = report.mean_absolute_error,
        max_rel = report.max_relative_error,
        max_rel_pct = report.max_relative_error * 100.0,
        rel_status = format_pass(report.max_relative_error < 0.01),
        corr = report.correlation_coefficient,
        corr_status = format_pass(report.correlation_coefficient > 0.9999),
        snr = report.snr_db,
        thd_spice = report.thd_spice,
        thd_melange = report.thd_melange,
        thd_err = report.thd_error_db,
        thd_status = format_pass(report.thd_error_db.abs() < 1.0),
        failures_section = generate_failures_html(report),
        signal_overlay_svg = signal_overlay_svg,
        error_signal_svg = error_signal_svg,
        histogram_svg = histogram_svg,
        scatter_svg = scatter_svg,
        status_class = status_class,
        status_text = status_text,
    )
}

/// Generate CSV for external analysis
pub fn generate_csv(
    spice: &Signal,
    melange: &Signal,
    output_path: &Path,
) -> Result<(), VisualizerError> {
    let len = spice.len().min(melange.len());
    if len == 0 {
        return Err(VisualizerError::InvalidData(
            "No samples to export".to_string(),
        ));
    }

    let sample_rate = spice.sample_rate;
    let mut file = File::create(output_path)?;

    // Write header
    writeln!(
        file,
        "time,spice_voltage,melange_voltage,absolute_error,relative_error"
    )?;

    // Write data rows
    for i in 0..len {
        let time = i as f64 / sample_rate;
        let spice_val = spice.samples.get(i).copied().unwrap_or(0.0);
        let melange_val = melange.samples.get(i).copied().unwrap_or(0.0);
        let abs_error = (melange_val - spice_val).abs();
        let rel_error = if spice_val.abs() > 1e-12 {
            abs_error / spice_val.abs()
        } else {
            0.0
        };

        writeln!(
            file,
            "{:.10e},{:.10e},{:.10e},{:.10e},{:.10e}",
            time, spice_val, melange_val, abs_error, rel_error
        )?;
    }

    Ok(())
}

/// Generate a JSON report for programmatic analysis
pub fn generate_json_report(
    report: &ComparisonReport,
    output_path: &Path,
) -> Result<(), VisualizerError> {
    let json = serde_json::to_string_pretty(&SerializableReport::from(report))
        .map_err(|e| VisualizerError::IoError(e.to_string()))?;

    let mut file = File::create(output_path)?;
    file.write_all(json.as_bytes())?;

    Ok(())
}

/// Helper struct for JSON serialization
#[derive(serde::Serialize)]
struct SerializableReport {
    circuit_name: String,
    node_name: String,
    sample_count: usize,
    sample_rate: f64,
    passed: bool,
    metrics: Metrics,
    failures: Vec<String>,
}

#[derive(serde::Serialize)]
struct Metrics {
    rms_error: f64,
    normalized_rms_error: f64,
    peak_error: f64,
    mean_absolute_error: f64,
    max_relative_error: f64,
    correlation_coefficient: f64,
    snr_db: f64,
    thd_spice: f64,
    thd_melange: f64,
    thd_error_db: f64,
}

impl From<&ComparisonReport> for SerializableReport {
    fn from(r: &ComparisonReport) -> Self {
        Self {
            circuit_name: r.circuit_name.clone(),
            node_name: r.node_name.clone(),
            sample_count: r.sample_count,
            sample_rate: r.sample_rate,
            passed: r.passed,
            metrics: Metrics {
                rms_error: r.rms_error,
                normalized_rms_error: r.normalized_rms_error,
                peak_error: r.peak_error,
                mean_absolute_error: r.mean_absolute_error,
                max_relative_error: r.max_relative_error,
                correlation_coefficient: r.correlation_coefficient,
                snr_db: r.snr_db,
                thd_spice: r.thd_spice,
                thd_melange: r.thd_melange,
                thd_error_db: r.thd_error_db,
            },
            failures: r.failures.clone(),
        }
    }
}

// === SVG Plot Generation ===

fn generate_signal_overlay_svg(
    time: &[f64],
    spice: &Signal,
    melange: &Signal,
    len: usize,
) -> Result<String, VisualizerError> {
    let width = 800;
    let height = 300;
    let padding = 50;

    let plot_width = width - 2 * padding;
    let plot_height = height - 2 * padding;

    // Find data ranges
    let time_min = time[0];
    let time_max = time[len - 1];
    let time_range = time_max - time_min;

    let all_values: Vec<f64> = spice.samples[..len]
        .iter()
        .chain(melange.samples[..len].iter())
        .copied()
        .collect();
    let val_min = all_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    let val_max = all_values.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
    let val_range = val_max - val_min;
    let val_margin = val_range * 0.1;
    let val_min = val_min - val_margin;
    let val_max = val_max + val_margin;

    // Scale functions
    let scale_x =
        |t: f64| -> i32 { padding + ((t - time_min) / time_range * plot_width as f64) as i32 };
    let scale_y = |v: f64| -> i32 {
        height - padding - ((v - val_min) / (val_max - val_min) * plot_height as f64) as i32
    };

    // Downsample for SVG (max 1000 points)
    let step = (len / 1000).max(1);

    // Build SPICE path
    let mut spice_path = String::new();
    for (i, idx) in (0..len).step_by(step).enumerate() {
        let x = scale_x(time[idx]);
        let y = scale_y(spice.samples[idx]);
        if i == 0 {
            spice_path.push_str(&format!("M {},{} ", x, y));
        } else {
            spice_path.push_str(&format!("L {},{} ", x, y));
        }
    }

    // Build melange path
    let mut melange_path = String::new();
    for (i, idx) in (0..len).step_by(step).enumerate() {
        let x = scale_x(time[idx]);
        let y = scale_y(melange.samples[idx]);
        if i == 0 {
            melange_path.push_str(&format!("M {},{} ", x, y));
        } else {
            melange_path.push_str(&format!("L {},{} ", x, y));
        }
    }

    let c = SvgColors::default();

    // Build SVG using string concatenation
    let mut svg = String::new();
    svg.push_str(&format!(
        r#"<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">"#
    ));
    svg.push_str(&format!(
        r#"<rect width="{width}" height="{height}" fill="{bg}"/>"#,
        bg = c.bg
    ));

    // Grid lines
    svg.push_str(&format!(
        r#"<g stroke="{grid}" stroke-width="1">"#,
        grid = c.grid
    ));
    for i in 1..=3 {
        let y = scale_y(val_min + i as f64 * 0.25 * (val_max - val_min));
        svg.push_str(&format!(
            r#"<line x1="{padding}" y1="{y}" x2="{x2}" y2="{y}"/>"#,
            x2 = width - padding
        ));
    }
    for i in 1..=3 {
        let x = scale_x(time_min + i as f64 * 0.25 * time_range);
        svg.push_str(&format!(
            r#"<line x1="{x}" y1="{padding}" x2="{x}" y2="{y2}"/>"#,
            y2 = height - padding
        ));
    }
    svg.push_str("</g>");

    // Axes
    let axis_y = height - padding;
    svg.push_str(&format!(
        r#"<line x1="{padding}" y1="{axis_y}" x2="{x2}" y2="{axis_y}" stroke="{axis}" stroke-width="2"/>"#,
        x2 = width - padding,
        axis = c.axis
    ));
    svg.push_str(&format!(
        r#"<line x1="{padding}" y1="{padding}" x2="{padding}" y2="{y2}" stroke="{axis}" stroke-width="2"/>"#,
        y2 = height - padding,
        axis = c.axis
    ));

    // SPICE signal
    svg.push_str(&format!(
        r#"<path d="{spice_path}" fill="none" stroke="{spice}" stroke-width="2"/>"#,
        spice = c.spice
    ));

    // Melange signal
    svg.push_str(&format!(
        r#"<path d="{melange_path}" fill="none" stroke="{melange}" stroke-width="2" stroke-dasharray="5,3"/>"#,
        melange = c.melange
    ));

    // Legend
    let legend_x = width - 200;
    svg.push_str(&format!(r#"<g transform="translate({legend_x}, 20)">"#));
    svg.push_str(&format!(
        r#"<line x1="0" y1="0" x2="20" y2="0" stroke="{spice}" stroke-width="2"/>"#,
        spice = c.spice
    ));
    svg.push_str(&format!(
        r#"<text x="25" y="4" fill="{text}" font-size="12">SPICE</text>"#,
        text = c.text
    ));
    svg.push_str(&format!(
        r#"<line x1="80" y1="0" x2="100" y2="0" stroke="{melange}" stroke-width="2" stroke-dasharray="5,3"/>"#,
        melange = c.melange
    ));
    svg.push_str(&format!(
        r#"<text x="105" y="4" fill="{text}" font-size="12">Melange</text>"#,
        text = c.text
    ));
    svg.push_str("</g>");

    svg.push_str("</svg>");

    Ok(svg)
}

fn generate_error_signal_svg(
    time: &[f64],
    spice: &Signal,
    melange: &Signal,
    len: usize,
) -> Result<String, VisualizerError> {
    let width = 800;
    let height = 200;
    let padding = 50;

    let plot_width = width - 2 * padding;
    let plot_height = height - 2 * padding;

    // Compute error
    let errors: Vec<f64> = (0..len)
        .map(|i| melange.samples[i] - spice.samples[i])
        .collect();

    let time_min = time[0];
    let time_max = time[len - 1];
    let time_range = time_max - time_min;

    let err_max = errors.iter().map(|&e| e.abs()).fold(0.0, f64::max) * 1.1;
    let err_min = -err_max;

    // Scale functions
    let scale_x =
        |t: f64| -> i32 { padding + ((t - time_min) / time_range * plot_width as f64) as i32 };
    let scale_y = |v: f64| -> i32 {
        height - padding - ((v - err_min) / (err_max - err_min) * plot_height as f64) as i32
    };

    // Downsample
    let step = (len / 1000).max(1);

    // Build error path
    let mut error_path = String::new();
    for (i, idx) in (0..len).step_by(step).enumerate() {
        let x = scale_x(time[idx]);
        let y = scale_y(errors[idx]);
        if i == 0 {
            error_path.push_str(&format!("M {},{} ", x, y));
        } else {
            error_path.push_str(&format!("L {},{} ", x, y));
        }
    }

    // Zero line
    let zero_y = scale_y(0.0);

    let c = SvgColors::default();
    let mut svg = String::new();
    svg.push_str(&format!(
        r#"<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">"#
    ));
    svg.push_str(&format!(
        r#"<rect width="{width}" height="{height}" fill="{bg}"/>"#,
        bg = c.bg
    ));

    // Zero line
    svg.push_str(&format!(
        r#"<line x1="{padding}" y1="{zero_y}" x2="{x2}" y2="{zero_y}" stroke="{axis}" stroke-width="1" stroke-dasharray="3,3"/>"#,
        x2 = width - padding,
        axis = c.axis
    ));

    // Error signal
    svg.push_str(&format!(
        r#"<path d="{error_path}" fill="none" stroke="{error}" stroke-width="1.5"/>"#,
        error = c.error
    ));

    svg.push_str("</svg>");

    Ok(svg)
}

fn generate_histogram_svg(
    report: &ComparisonReport,
    _len: usize,
) -> Result<String, VisualizerError> {
    let width = 400;
    let height = 200;
    let padding = 40;

    let abs_errors = report
        .absolute_errors
        .as_ref()
        .ok_or_else(|| VisualizerError::InvalidData("No absolute errors in report".to_string()))?;

    if abs_errors.is_empty() {
        return Ok("<p>No error data available</p>".to_string());
    }

    // Create histogram bins
    let num_bins = 30;
    let max_error = abs_errors.iter().copied().fold(0.0, f64::max);
    let bin_width = max_error / num_bins as f64;

    let mut bins = vec![0usize; num_bins];
    for &err in abs_errors {
        let bin = ((err / bin_width) as usize).min(num_bins - 1);
        bins[bin] += 1;
    }

    let max_count = *bins.iter().max().unwrap_or(&1);
    let bin_pixel_width = (width - 2 * padding) / num_bins;
    let plot_height = height - 2 * padding;

    // Build histogram bars
    let c = SvgColors::default();
    let mut svg = String::new();
    svg.push_str(&format!(
        r#"<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">"#
    ));
    svg.push_str(&format!(
        r#"<rect width="{width}" height="{height}" fill="{bg}"/>"#,
        bg = c.bg
    ));

    for (i, &count) in bins.iter().enumerate() {
        let bar_height = (count as f64 / max_count as f64 * plot_height as f64) as i32;
        let x = padding + i * bin_pixel_width;
        let y = height - padding - bar_height as usize;

        svg.push_str(&format!(
            r#"<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{spice}" opacity="0.7"/>"#,
            w = bin_pixel_width - 1,
            h = bar_height,
            spice = c.spice
        ));
    }

    svg.push_str("</svg>");

    Ok(svg)
}

fn generate_scatter_svg(
    spice: &Signal,
    melange: &Signal,
    len: usize,
) -> Result<String, VisualizerError> {
    let width = 300;
    let height = 300;
    let padding = 50;

    let plot_size = width - 2 * padding;

    // Find range
    let all_values: Vec<f64> = spice.samples[..len]
        .iter()
        .chain(melange.samples[..len].iter())
        .copied()
        .collect();
    let val_min = all_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    let val_max = all_values.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
    let val_margin = (val_max - val_min) * 0.05;
    let val_min = val_min - val_margin;
    let val_max = val_max + val_margin;

    // Scale function
    let scale = |v: f64| -> i32 {
        padding + ((v - val_min) / (val_max - val_min) * plot_size as f64) as i32
    };

    // Downsample points for SVG performance
    let step = (len / 500).max(1);

    // Build points
    let c = SvgColors::default();
    let mut svg = String::new();
    svg.push_str(&format!(
        r#"<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">"#
    ));
    svg.push_str(&format!(
        r#"<rect width="{width}" height="{height}" fill="{bg}"/>"#,
        bg = c.bg
    ));

    // Ideal line (y=x)
    let line_start = scale(val_min);
    let line_end = scale(val_max);
    let line_y_start = height - line_start;
    let line_y_end = height - line_end;
    svg.push_str(&format!(
        r#"<line x1="{line_start}" y1="{line_y_start}" x2="{line_end}" y2="{line_y_end}" stroke="{axis}" stroke-width="1" stroke-dasharray="3,3"/>"#,
        axis = c.axis
    ));

    // Data points
    for i in (0..len).step_by(step) {
        let x = scale(spice.samples[i]);
        let y = height - scale(melange.samples[i]);
        svg.push_str(&format!(
            r#"<circle cx="{x}" cy="{y}" r="1.5" fill="{spice}" opacity="0.5"/>"#,
            spice = c.spice
        ));
    }

    svg.push_str("</svg>");

    Ok(svg)
}

// Helper functions

fn format_pass(passed: bool) -> String {
    if passed {
        r#"<span style="color: rgb(78,204,163);">✓</span>"#.to_string()
    } else {
        r#"<span style="color: rgb(233,69,96);">✗</span>"#.to_string()
    }
}

fn generate_failures_html(report: &ComparisonReport) -> String {
    if report.failures.is_empty() {
        String::new()
    } else {
        let mut html = String::from("<h2>Failures</h2><div>");
        for failure in &report.failures {
            html.push_str(&format!(r#"<div class="failure">{}</div>"#, failure));
        }
        html.push_str("</div>");
        html
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comparison::{compare_signals, ComparisonConfig};

    #[test]
    fn test_generate_csv() {
        let spice = Signal::new(vec![0.0, 0.5, 1.0, 0.5, 0.0], 1000.0, "spice");
        let melange = Signal::new(vec![0.001, 0.501, 1.002, 0.499, 0.001], 1000.0, "melange");

        let temp_file = tempfile::NamedTempFile::new().unwrap();
        generate_csv(&spice, &melange, temp_file.path()).unwrap();

        let contents = std::fs::read_to_string(temp_file.path()).unwrap();
        assert!(contents.contains("time,spice_voltage,melange_voltage"));
        assert!(contents.lines().count() == 6); // header + 5 data rows
    }

    #[test]
    fn test_generate_html_report() {
        let ref_samples: Vec<f64> = (0..100)
            .map(|i| (2.0 * std::f64::consts::PI * i as f64 / 100.0).sin())
            .collect();
        let act_samples: Vec<f64> = ref_samples.iter().map(|&v| v + 0.001).collect();

        let spice = Signal::new(ref_samples, 1000.0, "spice");
        let melange = Signal::new(act_samples, 1000.0, "melange");

        let config = ComparisonConfig::default();
        let mut report = compare_signals(&spice, &melange, &config);
        report.circuit_name = "test_circuit".to_string();

        let temp_file = tempfile::NamedTempFile::new().unwrap();
        generate_html_report(&report, &spice, &melange, temp_file.path()).unwrap();

        let contents = std::fs::read_to_string(temp_file.path()).unwrap();
        assert!(contents.contains("Melange Validation Report"));
        assert!(contents.contains("test_circuit"));
    }

    #[test]
    fn test_generate_json_report() {
        let spice = Signal::new(vec![0.0, 0.5, 1.0], 1000.0, "spice");
        let melange = Signal::new(vec![0.001, 0.501, 1.002], 1000.0, "melange");

        let config = ComparisonConfig::default();
        let mut report = compare_signals(&spice, &melange, &config);
        report.circuit_name = "test_circuit".to_string();

        let temp_file = tempfile::NamedTempFile::new().unwrap();
        generate_json_report(&report, temp_file.path()).unwrap();

        let contents = std::fs::read_to_string(temp_file.path()).unwrap();
        assert!(contents.contains("test_circuit"));
        assert!(contents.contains("metrics"));
    }
}
