# Melange Rich CLI Implementation Plan

**Status:** Draft  
**Last Updated:** 2026-02-23

This document outlines the plan for enhancing melange-cli with rich terminal UI elements (colors, progress bars, tables) while maintaining composability and CI/CD compatibility.

---

## Goals

1. **Modern appearance** — Polished output that matches tools like `cargo`, `docker`, `gh`
2. **TTY-aware** — Rich features in interactive use, plain text when piped
3. **CI/CD compatible** — Respects `NO_COLOR`, provides `--color` and `--output` flags
4. **Non-breaking** — Existing scripts continue to work

---

## Architecture

```
tools/melange-cli/
├── Cargo.toml
├── src/
│   ├── main.rs              # Entry point, CLI args
│   ├── ui/                  # NEW: UI abstraction layer
│   │   ├── mod.rs           # Public API: progress(), table(), style()
│   │   ├── color.rs         # TTY detection, color choice logic
│   │   ├── progress.rs      # Progress bar wrapper
│   │   ├── style.rs         # Styled text helpers
│   │   └── table.rs         # Table formatting wrapper
│   ├── cmd/                 # NEW: Command modules (refactored from main.rs)
│   │   ├── mod.rs
│   │   ├── compile.rs
│   │   ├── validate.rs
│   │   ├── simulate.rs
│   │   ├── nodes.rs
│   │   ├── sources.rs
│   │   ├── builtins.rs
│   │   └── cache.rs
│   ├── output.rs            # NEW: Structured output types for JSON mode
│   ├── cache.rs
│   ├── circuits.rs
│   ├── sources.rs
│   └── plugin_template.rs
```

---

## Phase 0: Foundation

### 0.1 Dependencies

Add to `tools/melange-cli/Cargo.toml`:

```toml
[dependencies]
# ... existing deps ...
anstyle = "1.0"
anstream = "0.6"
indicatif = "0.17"
tabled = "0.16"
console = "0.15"  # For terminal width detection
serde_json = "1.0"  # For --output=json
```

### 0.2 Global CLI Flags

Add to the main `Cli` struct in `main.rs`:

```rust
#[derive(Parser)]
#[command(name = "melange")]
#[command(about = "Circuit modeling toolkit - from SPICE to real-time DSP")]
#[command(version)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
    
    /// Control colored output
    #[arg(long, global = true, value_enum, default_value = "auto")]
    color: ColorPreference,
    
    /// Suppress non-essential output
    #[arg(short, long, global = true)]
    quiet: bool,
    
    /// Output format
    #[arg(long, global = true, value_enum, default_value = "human")]
    output: OutputMode,
}

#[derive(Clone, Copy, Debug, Default, ValueEnum)]
enum ColorPreference {
    #[default]
    Auto,
    Always,
    Never,
}

#[derive(Clone, Copy, Debug, Default, ValueEnum)]
enum OutputMode {
    /// Human-readable with colors and progress
    #[default]
    Human,
    /// Plain text without colors
    Plain,
    /// Machine-readable JSON
    Json,
}
```

### 0.3 Color Module

**`src/ui/color.rs`**:

```rust
//! Terminal color and styling utilities
use anstream::ColorChoice;
use std::io::{self, IsTerminal};

/// Color output preference
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum ColorPreference {
    #[default]
    Auto,
    Always,
    Never,
}

impl ColorPreference {
    /// Determine actual color choice based on preference and environment
    pub fn to_color_choice(self) -> ColorChoice {
        match self {
            ColorPreference::Auto => {
                // Check NO_COLOR env var
                if std::env::var_os("NO_COLOR").is_some() {
                    return ColorChoice::Never;
                }
                // Check if stdout is a TTY
                if io::stdout().is_terminal() {
                    ColorChoice::Auto
                } else {
                    ColorChoice::Never
                }
            }
            ColorPreference::Always => ColorChoice::Always,
            ColorPreference::Never => ColorChoice::Never,
        }
    }
}

impl std::str::FromStr for ColorPreference {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "auto" => Ok(ColorPreference::Auto),
            "always" | "yes" => Ok(ColorPreference::Always),
            "never" | "no" => Ok(ColorPreference::Never),
            _ => Err(format!("Invalid color preference: {}", s)),
        }
    }
}

/// Check if we should show interactive elements (progress bars, etc.)
pub fn is_interactive() -> bool {
    io::stderr().is_terminal() && !is_ci()
}

/// Detect if running in CI environment
fn is_ci() -> bool {
    std::env::var_os("CI").is_some()
        || std::env::var_os("CONTINUOUS_INTEGRATION").is_some()
        || std::env::var_os("GITHUB_ACTIONS").is_some()
}
```

---

## Phase 1: Progress Bars

### 1.1 Progress Module

**`src/ui/progress.rs`**:

```rust
//! Progress bar handling with TTY-aware fallback

use indicatif::{ProgressBar, ProgressStyle};
use std::time::Duration;

/// A progress tracker that works in both interactive and non-interactive modes
pub struct ProgressTracker {
    pb: Option<ProgressBar>,
    step: usize,
    total: usize,
    prefix: String,
}

impl ProgressTracker {
    /// Create a new progress tracker
    pub fn new(total: usize, prefix: &str) -> Self {
        let pb = if super::is_interactive() {
            let pb = ProgressBar::new(total as u64);
            pb.set_style(
                ProgressStyle::default_bar()
                    .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len}: {msg}")
                    .unwrap()
                    .progress_chars("#>-"),
            );
            pb.enable_steady_tick(Duration::from_millis(100));
            Some(pb)
        } else {
            None
        };

        Self {
            pb,
            step: 0,
            total,
            prefix: prefix.to_string(),
        }
    }

    /// Advance to next step with message
    pub fn next_step(&mut self, msg: &str) {
        self.step += 1;
        if let Some(ref pb) = self.pb {
            pb.set_position(self.step as u64);
            pb.set_message(msg.to_string());
        } else {
            // Non-interactive: print to stderr
            eprintln!("{} [{}/{}] {}", self.prefix, self.step, self.total, msg);
        }
    }

    /// Finish with success message
    pub fn finish(self, msg: &str) {
        if let Some(pb) = self.pb {
            pb.finish_with_message(msg.to_string());
        } else {
            eprintln!("{} ✓ {}", self.prefix, msg);
        }
    }

    /// Finish with error
    pub fn finish_with_error(self, msg: &str) {
        if let Some(pb) = self.pb {
            pb.abandon_with_message(format!("✗ {}", msg));
        } else {
            eprintln!("{} ✗ {}", self.prefix, msg);
        }
    }
}

impl Drop for ProgressTracker {
    fn drop(&mut self) {
        if let Some(ref pb) = self.pb {
            if !pb.is_finished() {
                pb.abandon();
            }
        }
    }
}
```

### 1.2 Compile Command Integration

**`src/cmd/compile.rs`**:

```rust
use crate::ui::{ColorPreference, OutputMode, ProgressTracker};
use anyhow::{Context, Result};
use std::path::PathBuf;

pub fn compile(
    circuit_source: &CircuitSource,
    output: &PathBuf,
    sample_rate: f64,
    input_node: &str,
    output_node: &str,
    max_iter: usize,
    tolerance: f64,
    format: OutputFormat,
    color: ColorPreference,
    quiet: bool,
    output_mode: OutputMode,
) -> Result<()> {
    // Setup color output
    let _color_guard = anstream::adapter::set_color_choice(color.to_color_choice());

    match output_mode {
        OutputMode::Json => compile_json(circuit_source, output, sample_rate, /* ... */),
        OutputMode::Plain => compile_plain(circuit_source, output, sample_rate, /* ... */, quiet),
        OutputMode::Human => compile_human(circuit_source, output, sample_rate, /* ... */, quiet),
    }
}

fn compile_human(
    circuit_source: &CircuitSource,
    output: &PathBuf,
    sample_rate: f64,
    input_node: &str,
    output_node: &str,
    max_iter: usize,
    tolerance: f64,
    format: OutputFormat,
    quiet: bool,
) -> Result<()> {
    if !quiet {
        eprintln!("🔧 melange compile  {} → {}", 
            circuit_source.name(),
            output.display()
        );
    }

    // Create progress tracker for the 5 steps
    let mut progress = if quiet {
        None
    } else {
        Some(ProgressTracker::new(5, "Compiling"))
    };

    // Step 1: Resolve and parse
    if let Some(ref mut p) = progress {
        p.next_step("Resolving circuit source...");
    }
    let netlist_str = circuit_source.content()?;

    if let Some(ref mut p) = progress {
        p.next_step("Parsing SPICE netlist...");
    }
    let netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;

    // Step 2: Build MNA
    if let Some(ref mut p) = progress {
        p.next_step(&format!("Building MNA system ({} elements)...", netlist.elements.len()));
    }
    let mna = MnaSystem::from_netlist(&netlist)
        .with_context(|| "Failed to build MNA system")?;

    // Step 3: DK Kernel
    if let Some(ref mut p) = progress {
        p.next_step(&format!("Creating DK kernel (N={}, M={})...", mna.n, mna.nonlinear_devices.len()));
    }
    let kernel = DkKernel::from_mna(&mna, sample_rate)
        .with_context(|| "Failed to create DK kernel")?;

    // Step 4: Generate code
    if let Some(ref mut p) = progress {
        p.next_step("Generating Rust code...");
    }
    let config = CodegenConfig {
        circuit_name: output.file_stem().and_then(|s| s.to_str()).unwrap_or("circuit").to_string(),
        input_node: mna.node_map.get(input_node).copied().unwrap_or(1).saturating_sub(1),
        output_node: mna.node_map.get(output_node).copied().unwrap_or(2).saturating_sub(1),
        sample_rate,
        max_iter,
        tol: tolerance,
        include_dc_op: true,
        input_conductance: 1.0,
    };
    let generator = CodeGenerator::new(config);
    let generated = generator.generate(&kernel, &mna, &netlist)
        .with_context(|| "Code generation failed")?;

    // Step 5: Write output
    if let Some(ref mut p) = progress {
        p.next_step("Writing output...");
    }
    write_output(&generated, output, format)?;

    let line_count = generated.code.lines().count();

    if let Some(p) = progress {
        p.finish(&format!("Generated {} lines", line_count));
    }

    if !quiet {
        eprintln!();
        eprintln!("✓ Success! Output written to: {}", output.display());
    }

    Ok(())
}
```

---

## Phase 2: Rich Tables

### 2.1 Table Module

**`src/ui/table.rs`**:

```rust
//! Table formatting with TTY-aware width handling

use tabled::{Table, Tabled, Style, Modify, Alignment, object::Rows};

pub struct TableFormatter;

impl TableFormatter {
    /// Format a list of items as a table
    pub fn format<T: Tabled>(items: &[T]) -> String {
        if items.is_empty() {
            return "(no items)".to_string();
        }

        let mut table = Table::new(items);
        table
            .with(Style::rounded())
            .with(Modify::new(Rows::first()).with(Alignment::center()));

        table.to_string()
    }

    /// Format with compact style (for narrow terminals)
    pub fn format_compact<T: Tabled>(items: &[T]) -> String {
        let mut table = Table::new(items);
        table.with(Style::blank());
        table.to_string()
    }
}

/// Re-export Tabled derive for convenience
pub use tabled::Tabled;
```

### 2.2 Sources List Integration

**`src/cmd/sources.rs`**:

```rust
use crate::ui::{TableFormatter, table::Tabled};
use crate::sources::SourcesConfig;
use anyhow::Result;

#[derive(Tabled)]
struct SourceRow {
    #[tabled(rename = "Name")]
    name: String,
    #[tabled(rename = "URL")]
    url: String,
    #[tabled(rename = "License")]
    license: String,
}

pub fn list_sources() -> Result<()> {
    let config = SourcesConfig::load()?;
    
    let rows: Vec<SourceRow> = config.sources.iter()
        .map(|(name, source)| SourceRow {
            name: name.clone(),
            url: source.url.clone(),
            license: source.license.clone().unwrap_or_else(|| "-".to_string()),
        })
        .collect();

    println!("{}", TableFormatter::format(&rows));
    
    Ok(())
}
```

### 2.3 Builtins List Integration

```rust
use crate::ui::{TableFormatter, table::Tabled};

#[derive(Tabled)]
struct BuiltinRow {
    #[tabled(rename = "Name")]
    name: &'static str,
    #[tabled(rename = "Description")]
    description: &'static str,
    #[tabled(rename = "Devices")]
    devices: &'static str,
}

pub fn list_builtins() -> Result<()> {
    let builtins = vec![
        BuiltinRow { name: "rc-lowpass", description: "Simple RC filter", devices: "0" },
        BuiltinRow { name: "tube-screamer", description: "Op-amp clipper with diodes", devices: "2" },
        BuiltinRow { name: "fuzz-face", description: "2-transistor NPN fuzz", devices: "2" },
        BuiltinRow { name: "big-muff", description: "4-transistor fuzz", devices: "6" },
    ];

    println!("{}", TableFormatter::format(&builtins));
    
    Ok(())
}
```

---

## Phase 3: Styled Output

### 3.1 Style Module

**`src/ui/style.rs`**:

```rust
//! Styled text helpers using anstyle

use anstyle::{Style, AnsiColor};

pub fn success(msg: &str) -> String {
    let style = Style::new()
        .fg_color(Some(anstyle::Color::Ansi(AnsiColor::Green)))
        .bold();
    format!("{}{}{}", style.render(), msg, style.render_reset())
}

pub fn error(msg: &str) -> String {
    let style = Style::new()
        .fg_color(Some(anstyle::Color::Ansi(AnsiColor::Red)))
        .bold();
    format!("{}{}{}", style.render(), msg, style.render_reset())
}

pub fn warning(msg: &str) -> String {
    let style = Style::new()
        .fg_color(Some(anstyle::Color::Ansi(AnsiColor::Yellow)));
    format!("{}{}{}", style.render(), msg, style.render_reset())
}

pub fn info(msg: &str) -> String {
    let style = Style::new()
        .fg_color(Some(anstyle::Color::Ansi(AnsiColor::Cyan)));
    format!("{}{}{}", style.render(), msg, style.render_reset())
}

pub fn dim(msg: &str) -> String {
    let style = Style::new()
        .fg_color(Some(anstyle::Color::Ansi(AnsiColor::White)))
        .dimmed();
    format!("{}{}{}", style.render(), msg, style.render_reset())
}
```

---

## Phase 4: JSON Output Mode

### 4.1 Output Types

**`src/output.rs`**:

```rust
//! Structured output types for --output=json

use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize)]
pub struct CompileResult {
    pub success: bool,
    pub circuit_name: String,
    pub nodes: usize,
    pub nonlinear_devices: usize,
    pub generated_lines: usize,
    pub output_path: String,
    pub errors: Vec<String>,
}

#[derive(Serialize, Deserialize)]
pub struct ValidationResult {
    pub success: bool,
    pub circuit_name: String,
    pub thd_db: Option<f64>,
    pub max_error: Option<f64>,
    pub details: Vec<ValidationDetail>,
}

#[derive(Serialize, Deserialize)]
pub struct ValidationDetail {
    pub test: String,
    pub passed: bool,
    pub message: String,
}

#[derive(Serialize, Deserialize)]
pub struct SourcesList {
    pub sources: Vec<SourceInfo>,
}

#[derive(Serialize, Deserialize)]
pub struct SourceInfo {
    pub name: String,
    pub url: String,
    pub license: Option<String>,
}

#[derive(Serialize, Deserialize)]
pub struct CacheStats {
    pub location: String,
    pub file_count: usize,
    pub size_bytes: u64,
}
```

### 4.2 JSON Mode Implementation Pattern

```rust
use crate::output::CompileResult;

fn compile_json(
    circuit_source: &CircuitSource,
    output: &PathBuf,
    sample_rate: f64,
    // ... other args
) -> Result<()> {
    let result = match try_compile(circuit_source, output, sample_rate, /* ... */) {
        Ok(stats) => CompileResult {
            success: true,
            circuit_name: circuit_source.name().to_string(),
            nodes: stats.nodes,
            nonlinear_devices: stats.nonlinear_devices,
            generated_lines: stats.lines,
            output_path: output.display().to_string(),
            errors: vec![],
        },
        Err(e) => CompileResult {
            success: false,
            circuit_name: circuit_source.name().to_string(),
            nodes: 0,
            nonlinear_devices: 0,
            generated_lines: 0,
            output_path: output.display().to_string(),
            errors: vec![e.to_string()],
        },
    };
    
    println!("{}", serde_json::to_string_pretty(&result)?);
    
    if !result.success {
        std::process::exit(1);
    }
    
    Ok(())
}
```

---

## UI Module Public API

**`src/ui/mod.rs`**:

```rust
//! UI abstraction for melange-cli
//! 
//! Provides styled output, progress bars, and tables that gracefully 
//! degrade when not in a TTY.
//!
//! # Usage
//!
//! ```rust
//! use melange_cli::ui::{ProgressTracker, TableFormatter, style};
//!
//! // Progress bars (auto-disabled when piped)
//! let mut progress = ProgressTracker::new(5, "Compiling");
//! progress.next_step("Parsing...");
//! progress.finish("Done!");
//!
//! // Styled text
//! println!("{}", style::success("Build successful!"));
//!
//! // Tables
//! let rows = vec![MyRow { ... }, ...];
//! println!("{}", TableFormatter::format(&rows));
//! ```

pub mod color;
pub mod progress;
pub mod style;
pub mod table;

pub use color::{ColorPreference, is_interactive};
pub use progress::ProgressTracker;
pub use style::{success, error, warning, info, dim};
pub use table::{TableFormatter, Tabled};
```

---

## Implementation Checklist

### Phase 0: Foundation
- [ ] Add dependencies to `Cargo.toml`
- [ ] Create `src/ui/` directory structure
- [ ] Implement `ui/color.rs` with TTY detection
- [ ] Add global `--color` and `--quiet` flags
- [ ] Update `main.rs` to pass UI config to commands

### Phase 1: Progress Bars
- [ ] Implement `ui/progress.rs`
- [ ] Refactor `compile` command to use progress tracker
- [ ] Test TTY detection with `| cat`
- [ ] Test `NO_COLOR=1` behavior
- [ ] Test `CI=1` behavior

### Phase 2: Tables
- [ ] Implement `ui/table.rs`
- [ ] Update `sources list` command
- [ ] Update `builtins` command
- [ ] Update `cache list` command
- [ ] Update `nodes` command

### Phase 3: Styled Output
- [ ] Implement `ui/style.rs`
- [ ] Apply styles to error messages
- [ ] Apply styles to success messages
- [ ] Add emoji indicators (🔧 ✓ ✗ ⚠)

### Phase 4: JSON Output
- [ ] Create `src/output.rs` with result types
- [ ] Implement `--output=json` for `compile`
- [ ] Implement `--output=json` for `validate`
- [ ] Implement `--output=json` for `sources list`
- [ ] Implement `--output=json` for `builtins`
- [ ] Add integration tests for JSON output

---

## Testing Strategy

### Manual Testing

```bash
# Test TTY detection (rich output)
melange compile tube-screamer --output ts.rs

# Test piping (plain output)
melange compile tube-screamer --output ts.rs 2>&1 | cat

# Test NO_COLOR
NO_COLOR=1 melange compile tube-screamer --output ts.rs

# Test CI mode
CI=1 melange compile tube-screamer --output ts.rs

# Test JSON output
melange compile tube-screamer --output ts.rs --output=json | jq '.success'

# Test quiet mode
melange compile tube-screamer --output ts.rs --quiet

# Test explicit color flags
melange compile tube-screamer --output ts.rs --color=always
melange compile tube-screamer --output ts.rs --color=never
```

### Automated Testing

Add tests to verify:
- TTY detection works correctly
- `NO_COLOR` disables colors
- `CI` disables progress bars
- JSON output is valid and parseable
- Quiet mode suppresses non-essential output

---

## Design Principles

1. **Progress to stderr** — Keep stdout clean for piping
2. **Respect conventions** — `NO_COLOR`, `CI`, `TERM=dumb`
3. **Graceful degradation** — Rich features only when appropriate
4. **Always provide escape hatches** — `--color`, `--output`, `--quiet`
5. **Don't break scripts** — Plain text when piped, JSON for machines

---

## References

- [anstyle documentation](https://docs.rs/anstyle/)
- [indicatif examples](https://github.com/console-rs/indicatif/tree/main/examples)
- [tabled book](https://tabled.rustcli.com/)
- [clig.dev - CLI guidelines](https://clig.dev/)
- [no-color.org](https://no-color.org/)
