//! Plugin project template generation

use std::hash::{Hash, Hasher};
use std::path::Path;

use anyhow::Result;

/// FTZ (Flush-to-Zero) + DAZ (Denormals-are-Zero) initialization block for generated plugins.
///
/// Concatenated via `+` into the generated `lib.rs`, so `{` appears as-is in the output
/// (no format!() interpolation). Keeps the use-tree braces normal and legible.
///
/// _mm_setcsr/_mm_getcsr were soft-deprecated in Rust in favor of inline asm, but remain
/// correct and portable across all x86/x86_64 targets. We wrap in #[allow(deprecated)]
/// so generated plugins compile cleanly under `-D warnings`. TODO: migrate to inline asm
/// (stmxcsr/ldmxcsr) or add AArch64 FPCR equivalent when Rust stabilizes a non-deprecated
/// cross-platform path.
const FTZ_DAZ_BLOCK: &str = "\
\x20       // Enable Flush-to-Zero and Denormals-are-Zero for audio performance\n\
\x20       #[cfg(target_arch = \"x86_64\")]\n\
\x20       #[allow(deprecated)]\n\
\x20       unsafe {\n\
\x20           use std::arch::x86_64::{_mm_setcsr, _mm_getcsr, _MM_FLUSH_ZERO_ON};\n\
\x20           _mm_setcsr(_mm_getcsr() | _MM_FLUSH_ZERO_ON | (1 << 6)); // FTZ + DAZ\n\
\x20       }\n\
\x20       #[cfg(target_arch = \"x86\")]\n\
\x20       #[allow(deprecated)]\n\
\x20       unsafe {\n\
\x20           use std::arch::x86::{_mm_setcsr, _mm_getcsr, _MM_FLUSH_ZERO_ON};\n\
\x20           _mm_setcsr(_mm_getcsr() | _MM_FLUSH_ZERO_ON | (1 << 6)); // FTZ + DAZ\n\
\x20       }\n";

/// Pot info for plugin parameter generation.
pub struct PotParamInfo {
    /// Index (0-based)
    pub index: usize,
    /// Human-readable name (e.g., "R1 (Tone)")
    pub name: String,
    /// Minimum resistance (ohms)
    pub min_resistance: f64,
    /// Maximum resistance (ohms)
    pub max_resistance: f64,
    /// Default/nominal resistance (ohms)
    pub default_resistance: f64,
}

/// Wiper pot info for plugin parameter generation.
///
/// A single position parameter (0.0–1.0) controls two linked pot resistances.
pub struct WiperParamInfo {
    /// Wiper group index (0-based, for `wiper_N` param naming)
    pub wiper_index: usize,
    /// Index into the pots array for the CW (top→wiper) leg
    pub cw_pot_index: usize,
    /// Index into the pots array for the CCW (wiper→bottom) leg
    pub ccw_pot_index: usize,
    /// Total resistance (R_cw + R_ccw = total)
    pub total_resistance: f64,
    /// Default wiper position (0.0–1.0)
    pub default_position: f64,
    /// Human-readable name (e.g., "Tone")
    pub name: String,
}

/// Gang info for plugin parameter generation.
///
/// A single position parameter (0.0–1.0) controls multiple linked pots/wipers.
pub struct GangParamInfo {
    /// Gang index (0-based, for `gang_N` param naming)
    pub index: usize,
    /// Human-readable label (e.g., "Gain")
    pub label: String,
    /// Default position (0.0–1.0)
    pub default_position: f64,
    /// Pot members: (pot_index, min_r, max_r, inverted)
    pub pot_members: Vec<(usize, f64, f64, bool)>,
    /// Wiper members: (cw_pot_index, ccw_pot_index, total_r, inverted)
    pub wiper_members: Vec<(usize, usize, f64, bool)>,
}

/// Switch info for plugin parameter generation.
pub struct SwitchParamInfo {
    /// Index (0-based)
    pub index: usize,
    /// Human-readable name (e.g., "SW0 (HF Boost Freq)")
    pub name: String,
    /// Number of positions
    pub num_positions: usize,
}

/// Options for plugin project generation that go beyond the core circuit parameters.
#[derive(Default)]
pub struct PluginOptions<'a> {
    /// Custom display name for the plugin (overrides auto-generated name from circuit filename).
    pub plugin_name: Option<&'a str>,
    /// Generate mono (1-channel) plugin instead of stereo.
    pub mono: bool,
    /// Add a wet/dry mix parameter to the generated plugin.
    pub wet_dry_mix: bool,
    /// Add ear-protection soft limiter on final output (default: true).
    pub ear_protection: bool,
    /// Plugin vendor name. If `None`, defaults to "Melange".
    pub vendor: Option<&'a str>,
    /// Plugin URL. If `None`, defaults to "https://github.com/melange".
    pub url: Option<&'a str>,
    /// Plugin contact email. If `None`, defaults to "dev@melange.audio".
    pub email: Option<&'a str>,
}

/// Generate a complete plugin project
pub fn generate_plugin_project(
    output_dir: &Path,
    circuit_code: &str,
    circuit_name: &str,
    with_level_params: bool,
    pots: &[PotParamInfo],
    wipers: &[WiperParamInfo],
    gangs: &[GangParamInfo],
    switches: &[SwitchParamInfo],
    num_outputs: usize,
) -> Result<()> {
    generate_plugin_project_with_oversampling(
        output_dir,
        circuit_code,
        circuit_name,
        with_level_params,
        pots,
        wipers,
        gangs,
        switches,
        num_outputs,
        1,
        &PluginOptions::default(),
    )
}

pub fn generate_plugin_project_with_oversampling(
    output_dir: &Path,
    circuit_code: &str,
    circuit_name: &str,
    with_level_params: bool,
    pots: &[PotParamInfo],
    wipers: &[WiperParamInfo],
    gangs: &[GangParamInfo],
    switches: &[SwitchParamInfo],
    num_outputs: usize,
    oversampling_factor: usize,
    options: &PluginOptions<'_>,
) -> Result<()> {
    std::fs::create_dir_all(output_dir.join("src"))?;
    std::fs::write(
        output_dir.join("Cargo.toml"),
        generate_cargo_toml(circuit_name),
    )?;
    std::fs::write(output_dir.join("src/circuit.rs"), circuit_code)?;
    std::fs::write(
        output_dir.join("src/lib.rs"),
        generate_lib_rs(
            circuit_name,
            with_level_params,
            pots,
            wipers,
            gangs,
            switches,
            num_outputs,
            oversampling_factor,
            options,
        ),
    )?;
    // .gitignore for generated plugin project
    std::fs::write(output_dir.join(".gitignore"), "/target\n")?;
    // README.md for generated plugin project
    std::fs::write(
        output_dir.join("README.md"),
        generate_readme(circuit_name, options.plugin_name),
    )?;
    // build.sh wrapper around nih_plug_xtask for bundling
    let build_sh_path = output_dir.join("build.sh");
    std::fs::write(&build_sh_path, generate_build_sh(circuit_name))?;
    // Make build.sh executable on Unix
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if let Ok(metadata) = std::fs::metadata(&build_sh_path) {
            let mut perms = metadata.permissions();
            perms.set_mode(0o755);
            let _ = std::fs::set_permissions(&build_sh_path, perms);
        }
    }
    Ok(())
}

fn generate_cargo_toml(circuit_name: &str) -> String {
    format!(
        r#"[package]
name = "{circuit_name}"
version = "0.1.0"
edition = "2021"

[dependencies]
# Pin nih-plug to a specific rev for reproducible builds
nih_plug = {{ git = "https://github.com/robbert-vdh/nih-plug.git", rev = "28b149ec" }}

[lib]
crate-type = ["cdylib"]

# Release profile tuned for solver-heavy DSP plugins.
# Fat LTO + single codegen unit squeezes the NR inner loop; strip reduces
# bundle size; panic=abort avoids unwinding in the audio thread.
[profile.release]
lto = "fat"
codegen-units = 1
panic = "abort"
strip = "symbols"
opt-level = 3
"#
    )
}

fn generate_readme(circuit_name: &str, plugin_name: Option<&str>) -> String {
    let display_name = if let Some(name) = plugin_name {
        name.to_string()
    } else {
        circuit_name
            .split(['-', '_'])
            .map(capitalize_word)
            .collect::<Vec<_>>()
            .join(" ")
    };
    format!(
        r#"# {display_name}

Audio plugin generated by [Melange](https://github.com/melange) from a SPICE circuit netlist.

## Build

### Quick: unbundled `.so`/`.dylib`/`.dll` (for development)

```bash
cargo build --release
```

The raw library ends up in `target/release/`. Most DAWs won't load it in this form
— you need the CLAP/VST3 bundle layout, which the `nih_plug_xtask` helper produces.

### Bundled CLAP + VST3 (for use in a DAW)

`nih-plug` ships its bundling logic as a separate `xtask` crate. The cleanest
workflow is to clone `nih-plug` once and run its bundler against this project:

```bash
# One-time setup (pick a directory outside this project):
git clone https://github.com/robbert-vdh/nih-plug.git ~/src/nih-plug

# From this project's directory:
cargo run --release --manifest-path ~/src/nih-plug/nih_plug_xtask/Cargo.toml -- \
    bundle {circuit_name} --release
```

The compiled plugin (CLAP + VST3) will be in `target/bundled/`.

Or run the convenience script that wraps the above (edit `NIH_PLUG_PATH` first):

```bash
bash build.sh
```

## Files

- `src/circuit.rs` — Generated circuit DSP code (do not edit by hand)
- `src/lib.rs` — Plugin wrapper (customize parameters, GUI, presets here)
- `build.sh` — Convenience script wrapping `nih_plug_xtask bundle`

To regenerate `circuit.rs` after changing the circuit netlist:

```bash
melange compile your-circuit.cir --format code -o src/circuit.rs
```
"#
    )
}

fn generate_build_sh(circuit_name: &str) -> String {
    format!(
        r#"#!/usr/bin/env bash
# Convenience wrapper around nih_plug_xtask for bundling {circuit_name}.
# Edit NIH_PLUG_PATH below to point at your local nih-plug checkout.
set -euo pipefail

NIH_PLUG_PATH="${{NIH_PLUG_PATH:-$HOME/src/nih-plug}}"

if [[ ! -d "$NIH_PLUG_PATH" ]]; then
    echo "error: nih-plug checkout not found at $NIH_PLUG_PATH" >&2
    echo "set NIH_PLUG_PATH, or clone with:" >&2
    echo "    git clone https://github.com/robbert-vdh/nih-plug.git \"$NIH_PLUG_PATH\"" >&2
    exit 1
fi

cargo run --release \
    --manifest-path "$NIH_PLUG_PATH/nih_plug_xtask/Cargo.toml" -- \
    bundle {circuit_name} --release

echo
echo "Bundled plugins are in: target/bundled/"
"#
    )
}

#[cfg(test)]
pub(crate) fn test_generate_cargo_toml(circuit_name: &str) -> String {
    generate_cargo_toml(circuit_name)
}

#[cfg(test)]
pub(crate) fn test_generate_lib_rs(
    circuit_name: &str,
    with_level_params: bool,
    pots: &[PotParamInfo],
) -> String {
    generate_lib_rs(
        circuit_name,
        with_level_params,
        pots,
        &[],
        &[],
        &[],
        1,
        1,
        &PluginOptions::default(),
    )
}

/// Capitalize first character, lowercase the rest (e.g., "hello" -> "Hello", "LOUD" -> "Loud").
fn capitalize_word(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        None => String::new(),
        Some(first) => first.to_uppercase().collect::<String>() + &chars.as_str().to_lowercase(),
    }
}

/// Compute a stable 16-byte VST3 ID string from a circuit name.
///
/// Uses `DefaultHasher` (SipHash) for better distribution, then derives 16
/// bytes via two hashes (name and name + salt) to fill the full ID space.
/// Each byte is mapped to an uppercase ASCII letter for a valid `b"..."` literal.
///
/// Note: renaming the circuit file will change the ID and break DAW sessions.
/// A future `--vst3-id` CLI override could allow pinning the ID explicitly.
fn compute_vst3_id(circuit_name: &str) -> String {
    // First 8 bytes from hashing the name directly
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    circuit_name.hash(&mut hasher);
    let h1 = hasher.finish().to_le_bytes();

    // Second 8 bytes from hashing the name with a salt
    let mut hasher2 = std::collections::hash_map::DefaultHasher::new();
    "melange-vst3-salt".hash(&mut hasher2);
    circuit_name.hash(&mut hasher2);
    let h2 = hasher2.finish().to_le_bytes();

    let mut id = [0u8; 16];
    for i in 0..8 {
        id[i] = b'A' + (h1[i] % 26);
        id[i + 8] = b'A' + (h2[i] % 26);
    }
    String::from_utf8(id.to_vec()).unwrap()
}

fn generate_pot_field(pot: &PotParamInfo) -> String {
    format!(
        "    #[id = \"pot_{idx}\"]\n    pub pot_{idx}: FloatParam,\n",
        idx = pot.index,
    )
}

fn generate_pot_default(pot: &PotParamInfo) -> String {
    let name = pot.name.replace('\\', "\\\\").replace('"', "\\\"");
    // Normalize default resistance to 0.0-1.0 position
    let range = pot.max_resistance - pot.min_resistance;
    let default_pos = if range > 0.0 {
        (pot.default_resistance - pot.min_resistance) / range
    } else {
        0.5
    };
    format!(
        r#"            pot_{idx}: FloatParam::new(
                "{name}",
                {default:.4},
                FloatRange::Linear {{
                    min: 0.0,
                    max: 1.0,
                }},
            )
            .with_smoother(SmoothingStyle::Linear(10.0))
            .with_unit("%")
            .with_value_to_string(Arc::new(|v| format!("{{:.0}}", v * 100.0)))
            .with_string_to_value(Arc::new(|s| s.trim_end_matches('%').trim().parse::<f32>().ok().map(|v| v / 100.0))),
"#,
        idx = pot.index,
        name = name,
        default = default_pos,
    )
}

fn generate_switch_field(sw: &SwitchParamInfo) -> String {
    format!(
        "    #[id = \"switch_{idx}\"]\n    pub switch_{idx}: IntParam,\n",
        idx = sw.index,
    )
}

fn generate_switch_default(sw: &SwitchParamInfo) -> String {
    let name = sw.name.replace('\\', "\\\\").replace('"', "\\\"");
    format!(
        r#"            switch_{idx}: IntParam::new(
                "{name}",
                0,
                IntRange::Linear {{
                    min: 0,
                    max: {max},
                }},
            ),
"#,
        idx = sw.index,
        name = name,
        max = sw.num_positions as i32 - 1,
    )
}

fn generate_wiper_field(wiper: &WiperParamInfo) -> String {
    format!(
        "    #[id = \"wiper_{idx}\"]\n    pub wiper_{idx}: FloatParam,\n",
        idx = wiper.wiper_index,
    )
}

fn generate_wiper_default(wiper: &WiperParamInfo) -> String {
    let name = wiper.name.replace('\\', "\\\\").replace('"', "\\\"");
    format!(
        r#"            wiper_{idx}: FloatParam::new(
                "{name}",
                {default},
                FloatRange::Linear {{
                    min: 0.0,
                    max: 1.0,
                }},
            )
            .with_smoother(SmoothingStyle::Linear(10.0)),
"#,
        idx = wiper.wiper_index,
        name = name,
        default = wiper.default_position,
    )
}

fn generate_params_struct(
    with_level_params: bool,
    pots: &[PotParamInfo],
    wipers: &[WiperParamInfo],
    gangs: &[GangParamInfo],
    switches: &[SwitchParamInfo],
    wet_dry_mix: bool,
    ear_protection: bool,
) -> String {
    let has_any_params = with_level_params
        || !pots.is_empty()
        || !wipers.is_empty()
        || !gangs.is_empty()
        || !switches.is_empty()
        || wet_dry_mix
        || ear_protection;
    if !has_any_params {
        return "#[derive(Params, Default)]\npub struct CircuitParams {}".to_string();
    }

    let mut fields = String::new();
    let mut defaults = String::new();

    if with_level_params {
        fields.push_str(LEVEL_PARAM_FIELDS);
        defaults.push_str(LEVEL_PARAM_DEFAULTS);
    }

    if wet_dry_mix {
        fields.push_str(MIX_PARAM_FIELD);
        defaults.push_str(MIX_PARAM_DEFAULT);
    }

    if ear_protection {
        fields.push_str(EAR_PROTECTION_PARAM_FIELD);
        defaults.push_str(EAR_PROTECTION_PARAM_DEFAULT);
    }

    for pot in pots {
        fields.push_str(&generate_pot_field(pot));
        defaults.push_str(&generate_pot_default(pot));
    }

    for wiper in wipers {
        fields.push_str(&generate_wiper_field(wiper));
        defaults.push_str(&generate_wiper_default(wiper));
    }

    for gang in gangs {
        fields.push_str(&format!(
            "    #[id = \"gang_{idx}\"]\n    pub gang_{idx}: FloatParam,\n",
            idx = gang.index,
        ));
        let name = gang.label.replace('\\', "\\\\").replace('"', "\\\"");
        defaults.push_str(&format!(
            r#"            gang_{idx}: FloatParam::new(
                "{name}",
                {default},
                FloatRange::Linear {{
                    min: 0.0,
                    max: 1.0,
                }},
            )
            .with_smoother(SmoothingStyle::Linear(10.0)),
"#,
            idx = gang.index,
            default = gang.default_position,
        ));
    }

    for sw in switches {
        fields.push_str(&generate_switch_field(sw));
        defaults.push_str(&generate_switch_default(sw));
    }

    format!(
        r#"#[derive(Params)]
pub struct CircuitParams {{
{fields}}}

impl Default for CircuitParams {{
    fn default() -> Self {{
        Self {{
{defaults}        }}
    }}
}}"#
    )
}

fn generate_process_loop(
    with_level_params: bool,
    pots: &[PotParamInfo],
    wipers: &[WiperParamInfo],
    gangs: &[GangParamInfo],
    switches: &[SwitchParamInfo],
    num_outputs: usize,
    mono: bool,
    wet_dry_mix: bool,
    ear_protection: bool,
) -> String {
    // Final output expression: ear protection applies soft limiter when enabled at runtime
    let output_write = if ear_protection {
        "*sample = if out.is_finite() { if ep { ear_protection_limit(out) } else { out } } else { 0.0 };"
    } else {
        "*sample = if out.is_finite() { out.clamp(-1.0, 1.0) } else { 0.0 };"
    };

    // Read ear protection toggle per buffer (BoolParam doesn't smooth)
    let ep_read = if ear_protection {
        "            let ep = self.params.ear_protection.value();\n"
    } else {
        ""
    };

    let has_any_params = with_level_params
        || !pots.is_empty()
        || !wipers.is_empty()
        || !switches.is_empty()
        || wet_dry_mix
        || ear_protection;
    if !has_any_params && num_outputs <= 1 {
        if mono {
            return format!(
                "        for channel_samples in buffer.iter_samples() {{\n\
                 \x20           let sample = channel_samples.into_iter().next().unwrap();\n\
                 \x20           let state = &mut self.circuit_states[0];\n\
                 \x20           let out = process_sample(*sample as f64, state)[0] as f32;\n\
                 \x20           {output_write}\n\
                 \x20       }}"
            );
        }
        return format!(
            "        for channel_samples in buffer.iter_samples() {{\n\
             \x20           for (ch, sample) in channel_samples.into_iter().enumerate() {{\n\
             \x20               let state = &mut self.circuit_states[ch];\n\
             \x20               let out = process_sample(*sample as f64, state)[0] as f32;\n\
             \x20               {output_write}\n\
             \x20           }}\n\
             \x20       }}"
        );
    }
    if !has_any_params && num_outputs > 1 {
        return format!(
            "        for mut channel_samples in buffer.iter_samples() {{\n\
             \x20           let input = *channel_samples.get_mut(0).unwrap() as f64;\n\
             \x20           let outs = process_sample(input, &mut self.circuit_state);\n\
             \x20           for (ch, sample) in channel_samples.into_iter().enumerate() {{\n\
             \x20               let out = outs[ch.min(NUM_OUTPUTS - 1)] as f32;\n\
             \x20               {output_write}\n\
             \x20           }}\n\
             \x20       }}"
        );
    }

    let gain_reads = if with_level_params {
        "            let input_gain = util::db_to_gain_fast(self.params.input_level.smoothed.next());\n\
         \x20           let output_gain = util::db_to_gain_fast(self.params.output_level.smoothed.next());\n"
    } else {
        ""
    };

    let mix_read = if wet_dry_mix {
        "            let mix = self.params.mix.smoothed.next();\n"
    } else {
        ""
    };

    // Pot and wiper reads: always per-block (set_pot triggers O(N³) rebuild).
    // Per-sample SM was removed — per-block rebuild is exact and fast enough.
    let pot_reads = String::new();
    let wiper_reads = String::new();

    // Switch reads + assignments: done once per buffer before the sample loop
    // (no smoother, and set_switch triggers O(N³) matrix rebuild)
    let switch_pre_loop: String = switches
        .iter()
        .map(|s| {
            if num_outputs > 1 {
                format!(
                    "        {{\n\
                     \x20           let sw_{i}_pos = self.params.switch_{i}.value() as usize;\n\
                     \x20           if sw_{i}_pos < {n} {{\n\
                     \x20               let state = &mut self.circuit_state;\n\
                     \x20               if state.switch_{i}_position != sw_{i}_pos {{\n\
                     \x20                   state.set_switch_{i}(sw_{i}_pos);\n\
                     \x20               }}\n\
                     \x20           }}\n\
                     \x20       }}\n",
                    i = s.index,
                    n = s.num_positions,
                )
            } else {
                format!(
                    "        {{\n\
                     \x20           let sw_{i}_pos = self.params.switch_{i}.value() as usize;\n\
                     \x20           if sw_{i}_pos < {n} {{\n\
                     \x20               for state in self.circuit_states.iter_mut() {{\n\
                     \x20                   if state.switch_{i}_position != sw_{i}_pos {{\n\
                     \x20                       state.set_switch_{i}(sw_{i}_pos);\n\
                     \x20                   }}\n\
                     \x20               }}\n\
                     \x20           }}\n\
                     \x20       }}\n",
                    i = s.index,
                    n = s.num_positions,
                )
            }
        })
        .collect();

    // Pot and wiper assignments: always per-block (moved to pre-loop).
    let pot_assignments = String::new();
    let wiper_assignments = String::new();

    // Gang reads: per-sample for DK, per-block for nodal
    // Gang reads/assignments: always per-block (moved to pre-loop).
    let gang_reads = String::new();
    let gang_assignments = String::new();

    // Per-block pot reads: done once per buffer before the sample loop.
    // set_pot_N() triggers O(N³) rebuild; skips if value unchanged.
    // Param is 0.0-1.0 position; convert to resistance: min + position * (max - min).
    let pot_pre_loop: String = if !pots.is_empty() {
        let reads: String = pots
            .iter()
            .map(|p| {
                // Use .value() not .smoothed.next() — smoother can't advance fast enough
                // at per-block rate. set_pot_N() skips rebuild if value unchanged.
                // Convert 0-1 position to resistance in ohms.
                format!(
                    "            let pot_{i}_val = {min:.17e}_f64 + self.params.pot_{i}.value() as f64 * {range:.17e}_f64;\n",
                    i = p.index,
                    min = p.min_resistance,
                    range = p.max_resistance - p.min_resistance,
                )
            })
            .collect();
        let assigns: String = if num_outputs > 1 {
            // Single state for multi-output
            pots.iter()
                .map(|p| {
                    format!(
                        "            self.circuit_state.set_pot_{i}(pot_{i}_val);\n",
                        i = p.index
                    )
                })
                .collect()
        } else {
            // Per-channel states
            pots.iter().map(|p| {
                format!(
                    "            for state in self.circuit_states.iter_mut() {{ state.set_pot_{i}(pot_{i}_val); }}\n",
                    i = p.index
                )
            }).collect()
        };
        format!("        {{ // Per-block pot updates (O(N³) rebuild on change)\n{reads}{assigns}        }}\n")
    } else {
        String::new()
    };

    // Per-block wiper reads: done once per buffer before the sample loop.
    let wiper_pre_loop: String = if !wipers.is_empty() {
        let reads: String = wipers
            .iter()
            .map(|w| {
                format!(
                    "            let wiper_{i}_pos = self.params.wiper_{i}.value() as f64;\n",
                    i = w.wiper_index
                )
            })
            .collect();
        let assigns: String = if num_outputs > 1 {
            wipers
                .iter()
                .map(|w| {
                    format!(
                        "            self.circuit_state.set_pot_{cw}((1.0 - wiper_{i}_pos) * {range:.17e} + 10.0);\n\
                         \x20           self.circuit_state.set_pot_{ccw}(wiper_{i}_pos * {range:.17e} + 10.0);\n",
                        i = w.wiper_index,
                        cw = w.cw_pot_index,
                        ccw = w.ccw_pot_index,
                        range = w.total_resistance - 20.0,
                    )
                })
                .collect()
        } else {
            wipers.iter().map(|w| {
                format!(
                    "            for state in self.circuit_states.iter_mut() {{\n\
                     \x20               state.set_pot_{cw}((1.0 - wiper_{i}_pos) * {range:.17e} + 10.0);\n\
                     \x20               state.set_pot_{ccw}(wiper_{i}_pos * {range:.17e} + 10.0);\n\
                     \x20           }}\n",
                    i = w.wiper_index,
                    cw = w.cw_pot_index,
                    ccw = w.ccw_pot_index,
                    range = w.total_resistance - 20.0,
                )
            }).collect()
        };
        format!("        {{ // Per-block wiper updates (O(N³) rebuild on change)\n{reads}{assigns}        }}\n")
    } else {
        String::new()
    };

    // Per-block gang reads: done once per buffer before the sample loop.
    let gang_pre_loop: String = if !gangs.is_empty() {
        let reads: String = gangs
            .iter()
            .map(|g| {
                format!(
                    "            let gang_{i}_pos = self.params.gang_{i}.smoothed.next() as f64;\n",
                    i = g.index,
                )
            })
            .collect();
        let assigns: String = gangs
            .iter()
            .flat_map(|g| {
                let mut lines = Vec::new();
                for &(pot_idx, min_r, max_r, inverted) in &g.pot_members {
                    let range = max_r - min_r;
                    let (r_expr, comment) = if inverted {
                        (format!("{min_r:.17e} + gang_{gi}_pos * {range:.17e}", gi = g.index), "inverted")
                    } else {
                        (format!("{max_r:.17e} - gang_{gi}_pos * {range:.17e}", gi = g.index), "")
                    };
                    let _ = comment; // suppress unused warning
                    if num_outputs > 1 {
                        lines.push(format!(
                            "            self.circuit_state.set_pot_{pot_idx}({r_expr});\n",
                        ));
                    } else {
                        lines.push(format!(
                            "            for state in self.circuit_states.iter_mut() {{\n\
                             \x20               state.set_pot_{pot_idx}({r_expr});\n\
                             \x20           }}\n",
                        ));
                    }
                }
                for &(cw_idx, ccw_idx, total_r, inverted) in &g.wiper_members {
                    let wrange = total_r - 20.0;
                    let pos_expr = if inverted {
                        format!("(1.0 - gang_{}_pos)", g.index)
                    } else {
                        format!("gang_{}_pos", g.index)
                    };
                    if num_outputs > 1 {
                        lines.push(format!(
                            "            self.circuit_state.set_pot_{cw_idx}((1.0 - {pos_expr}) * {wrange:.17e} + 10.0);\n\
                             \x20           self.circuit_state.set_pot_{ccw_idx}({pos_expr} * {wrange:.17e} + 10.0);\n",
                        ));
                    } else {
                        lines.push(format!(
                            "            for state in self.circuit_states.iter_mut() {{\n\
                             \x20               state.set_pot_{cw_idx}((1.0 - {pos_expr}) * {wrange:.17e} + 10.0);\n\
                             \x20               state.set_pot_{ccw_idx}({pos_expr} * {wrange:.17e} + 10.0);\n\
                             \x20           }}\n",
                        ));
                    }
                }
                lines
            })
            .collect();
        format!("        {{ // Per-block gang updates (nodal: O(N³) rebuild on change)\n{reads}{assigns}        }}\n")
    } else {
        String::new()
    };

    // Switch assignments removed from inner loop — handled in switch_pre_loop above

    // Build sample processing snippet
    let dry_capture = if wet_dry_mix {
        "                let dry = *sample;\n"
    } else {
        ""
    };

    let sample_processing = if num_outputs > 1 {
        if with_level_params {
            "                let input = input_sample as f64 * input_gain as f64;\n\
             \x20               let outs = process_sample(input, state);\n"
        } else {
            "                let outs = process_sample(input_sample as f64, state);\n"
        }
    } else if with_level_params {
        "                let input = *sample as f64 * input_gain as f64;\n\
         \x20               let out = process_sample(input, state)[0] as f32;\n\
         \x20               let out = out * output_gain;\n"
    } else {
        "                let out = process_sample(*sample as f64, state)[0] as f32;\n"
    };

    let mix_blend = if wet_dry_mix {
        "                let out = mix * out + (1.0 - mix) * dry;\n"
    } else {
        ""
    };

    // Build the inner loop: multi-output, mono, or stereo
    let inner_loop = if num_outputs > 1 {
        // Multi-output: ONE circuit instance, input from channel 0, multiple output channels
        let multi_dry_capture = if wet_dry_mix {
            "                let dry = input_sample;\n"
        } else {
            ""
        };
        let output_gain_line = if with_level_params {
            "                    let out = out * output_gain;\n"
        } else {
            ""
        };
        format!(
            "\x20           let mut channel_samples = channel_samples;\n\
             \x20               let input_sample = *channel_samples.get_mut(0).unwrap();\n\
             \x20               let state = &mut self.circuit_state;\n\
             {pot_assignments}\
             {wiper_assignments}\
             {gang_assignments}\
             {multi_dry_capture}\
             {sample_processing}\
             \x20               for (ch, sample) in channel_samples.into_iter().enumerate() {{\n\
             \x20                   let out = outs[ch.min(NUM_OUTPUTS - 1)] as f32;\n\
             {output_gain_line}\
             {mix_blend}\
             \x20                   {output_write}\n\
             \x20               }}\n",
        )
    } else if mono {
        format!(
            "\x20           let sample = channel_samples.into_iter().next().unwrap();\n\
             \x20               let state = &mut self.circuit_states[0];\n\
             {pot_assignments}\
             {wiper_assignments}\
             {gang_assignments}\
             {dry_capture}\
             {sample_processing}\
             {mix_blend}\
             \x20               {output_write}\n"
        )
    } else {
        format!(
            "\x20           for (ch, sample) in channel_samples.into_iter().enumerate() {{\n\
             \x20               let state = &mut self.circuit_states[ch];\n\
             {pot_assignments}\
             {wiper_assignments}\
             {gang_assignments}\
             {dry_capture}\
             {sample_processing}\
             {mix_blend}\
             \x20               {output_write}\n\
             \x20           }}\n"
        )
    };

    format!(
        "{switch_pre_loop}\
         {pot_pre_loop}\
         {wiper_pre_loop}\
         {gang_pre_loop}\
         {ep_read}\
         \x20       for channel_samples in buffer.iter_samples() {{\n\
         {gain_reads}\
         {mix_read}\
         {pot_reads}\
         {wiper_reads}\
         {gang_reads}\
         {inner_loop}\
         \x20       }}"
    )
}

const LEVEL_PARAM_FIELDS: &str = r#"    #[id = "input_level"]
    pub input_level: FloatParam,
    #[id = "output_level"]
    pub output_level: FloatParam,
"#;

const LEVEL_PARAM_DEFAULTS: &str = r#"            input_level: FloatParam::new(
                "Input Level",
                0.0,
                FloatRange::Linear {
                    min: -24.0,
                    max: 24.0,
                },
            )
            .with_smoother(SmoothingStyle::Linear(50.0))
            .with_unit(" dB"),

            output_level: FloatParam::new(
                "Output Level",
                0.0,
                FloatRange::Linear {
                    min: -24.0,
                    max: 24.0,
                },
            )
            .with_smoother(SmoothingStyle::Linear(50.0))
            .with_unit(" dB"),
"#;

const MIX_PARAM_FIELD: &str = r#"    #[id = "mix"]
    pub mix: FloatParam,
"#;

const MIX_PARAM_DEFAULT: &str = r#"            mix: FloatParam::new(
                "Mix",
                1.0,
                FloatRange::Linear {
                    min: 0.0,
                    max: 1.0,
                },
            )
            .with_smoother(SmoothingStyle::Linear(10.0))
            .with_unit("%")
            .with_value_to_string(Arc::new(|v| format!("{:.0}", v * 100.0)))
            .with_string_to_value(Arc::new(|s| s.trim_end_matches('%').trim().parse::<f32>().ok().map(|v| v / 100.0))),
"#;

const EAR_PROTECTION_PARAM_FIELD: &str = r#"    #[id = "ear_protection"]
    pub ear_protection: BoolParam,
"#;

const EAR_PROTECTION_PARAM_DEFAULT: &str = r#"            ear_protection: BoolParam::new(
                "Ear Protection",
                true,
            ),
"#;

fn generate_lib_rs(
    circuit_name: &str,
    with_level_params: bool,
    pots: &[PotParamInfo],
    wipers: &[WiperParamInfo],
    gangs: &[GangParamInfo],
    switches: &[SwitchParamInfo],
    num_outputs: usize,
    oversampling_factor: usize,
    options: &PluginOptions<'_>,
) -> String {
    let display_name: String = if let Some(name) = options.plugin_name {
        name.to_string()
    } else {
        circuit_name
            .split(['-', '_'])
            .map(capitalize_word)
            .collect::<Vec<_>>()
            .join(" ")
    };
    let clap_id = format!("com.melange.{circuit_name}");
    let vst3_id_str = compute_vst3_id(circuit_name);
    let vendor = options.vendor.unwrap_or("Melange");
    let url = options.url.unwrap_or("https://github.com/melange");
    let email = options.email.unwrap_or("dev@melange.audio");
    let params_struct = generate_params_struct(
        with_level_params,
        pots,
        wipers,
        gangs,
        switches,
        options.wet_dry_mix,
        options.ear_protection,
    );
    let process_loop = generate_process_loop(
        with_level_params,
        pots,
        wipers,
        gangs,
        switches,
        num_outputs,
        options.mono,
        options.wet_dry_mix,
        options.ear_protection,
    );

    // Conditional sections based on num_outputs
    let (circuit_import, plugin_struct, plugin_default, init_method, reset_method, deactivate_method) = if num_outputs
        > 1
    {
        // Multi-output: single circuit state, mono input → multi-output
        (
                "use circuit::{process_sample, CircuitState, NUM_OUTPUTS};".to_string(),
                "pub struct CircuitPlugin {\n\
                 \x20   params: Arc<CircuitParams>,\n\
                 \x20   circuit_state: CircuitState,\n\
                 \x20   current_sample_rate: f64,\n\
                 }"
                .to_string(),
                "impl Default for CircuitPlugin {\n\
                 \x20   fn default() -> Self {\n\
                 \x20       Self {\n\
                 \x20           params: Arc::new(CircuitParams::default()),\n\
                 \x20           circuit_state: CircuitState::default(),\n\
                 \x20           current_sample_rate: 0.0,\n\
                 \x20       }\n\
                 \x20   }\n\
                 }"
                .to_string(),
                "    fn initialize(\n\
                 \x20       &mut self,\n\
                 \x20       _audio_io_layout: &AudioIOLayout,\n\
                 \x20       buffer_config: &BufferConfig,\n\
                 \x20       _context: &mut impl InitContext<Self>,\n\
                 \x20   ) -> bool {\n"
                    .to_string()
                    + FTZ_DAZ_BLOCK
                    + "\x20       self.current_sample_rate = buffer_config.sample_rate as f64;\n\
                 \x20       self.circuit_state = CircuitState::default();\n\
                 \x20       self.circuit_state.set_sample_rate(buffer_config.sample_rate as f64);\n\
                 \x20       true\n\
                 \x20   }",
                "    fn reset(&mut self) {\n\
                 \x20       self.circuit_state.reset();\n\
                 \x20   }"
                    .to_string(),
                "    fn deactivate(&mut self) {\n\
                 \x20       self.circuit_state.reset();\n\
                 \x20   }"
                    .to_string(),
            )
    } else if options.mono {
        // Mono: single-channel state
        (
                "use circuit::{process_sample, CircuitState};".to_string(),
                "pub struct CircuitPlugin {\n\
                 \x20   params: Arc<CircuitParams>,\n\
                 \x20   circuit_states: Vec<CircuitState>,\n\
                 \x20   current_sample_rate: f64,\n\
                 }"
                .to_string(),
                "impl Default for CircuitPlugin {\n\
                 \x20   fn default() -> Self {\n\
                 \x20       Self {\n\
                 \x20           params: Arc::new(CircuitParams::default()),\n\
                 \x20           circuit_states: vec![CircuitState::default(); 1],\n\
                 \x20           current_sample_rate: 0.0,\n\
                 \x20       }\n\
                 \x20   }\n\
                 }"
                .to_string(),
                "    fn initialize(\n\
                 \x20       &mut self,\n\
                 \x20       _audio_io_layout: &AudioIOLayout,\n\
                 \x20       buffer_config: &BufferConfig,\n\
                 \x20       _context: &mut impl InitContext<Self>,\n\
                 \x20   ) -> bool {\n"
                    .to_string()
                    + FTZ_DAZ_BLOCK
                    + "\x20       self.current_sample_rate = buffer_config.sample_rate as f64;\n\
                 \x20       self.circuit_states = vec![{\n\
                 \x20           let mut s = CircuitState::default();\n\
                 \x20           s.set_sample_rate(buffer_config.sample_rate as f64);\n\
                 \x20           s\n\
                 \x20       }];\n\
                 \x20       true\n\
                 \x20   }",
                "    fn reset(&mut self) {\n\
                 \x20       for state in &mut self.circuit_states {\n\
                 \x20           state.reset();\n\
                 \x20       }\n\
                 \x20   }"
                    .to_string(),
                "    fn deactivate(&mut self) {\n\
                 \x20       for state in &mut self.circuit_states {\n\
                 \x20           state.reset();\n\
                 \x20       }\n\
                 \x20   }"
                    .to_string(),
            )
    } else {
        // Single output: per-channel state duplication (stereo from mono)
        (
                "use circuit::{process_sample, CircuitState};".to_string(),
                "pub struct CircuitPlugin {\n\
                 \x20   params: Arc<CircuitParams>,\n\
                 \x20   circuit_states: Vec<CircuitState>,\n\
                 \x20   current_sample_rate: f64,\n\
                 }"
                .to_string(),
                "impl Default for CircuitPlugin {\n\
                 \x20   fn default() -> Self {\n\
                 \x20       Self {\n\
                 \x20           params: Arc::new(CircuitParams::default()),\n\
                 \x20           circuit_states: vec![CircuitState::default(); 2],\n\
                 \x20           current_sample_rate: 0.0,\n\
                 \x20       }\n\
                 \x20   }\n\
                 }"
                .to_string(),
                "    fn initialize(\n\
                 \x20       &mut self,\n\
                 \x20       audio_io_layout: &AudioIOLayout,\n\
                 \x20       buffer_config: &BufferConfig,\n\
                 \x20       _context: &mut impl InitContext<Self>,\n\
                 \x20   ) -> bool {\n"
                    .to_string()
                    + FTZ_DAZ_BLOCK
                    + "\x20       let num_channels = audio_io_layout.main_input_channels\n\
                 \x20           .map(|c| c.get() as usize).unwrap_or(2);\n\
                 \x20       self.current_sample_rate = buffer_config.sample_rate as f64;\n\
                 \x20       self.circuit_states = (0..num_channels).map(|_| {\n\
                 \x20           let mut s = CircuitState::default();\n\
                 \x20           s.set_sample_rate(buffer_config.sample_rate as f64);\n\
                 \x20           s\n\
                 \x20       }).collect();\n\
                 \x20       true\n\
                 \x20   }",
                "    fn reset(&mut self) {\n\
                 \x20       for state in &mut self.circuit_states {\n\
                 \x20           state.reset();\n\
                 \x20       }\n\
                 \x20   }"
                    .to_string(),
                "    fn deactivate(&mut self) {\n\
                 \x20       for state in &mut self.circuit_states {\n\
                 \x20           state.reset();\n\
                 \x20       }\n\
                 \x20   }"
                    .to_string(),
            )
    };

    // Audio channel count for IO layout
    let num_channels = if options.mono { 1 } else { 2 };
    let clap_channel_feature = if options.mono {
        "ClapFeature::Mono"
    } else {
        "ClapFeature::Stereo"
    };

    // Oversampling latency: half-band IIR group delay in output samples
    let latency_method = if oversampling_factor > 1 {
        let latency_samples = match oversampling_factor {
            2 => 2,
            4 => 4,
            _ => oversampling_factor as u32,
        };
        format!(
            "\n    fn latency(&self) -> u32 {{\n\
             \x20       // Oversampling {}x half-band IIR decimation filter group delay\n\
             \x20       {}\n\
             \x20   }}\n",
            oversampling_factor, latency_samples,
        )
    } else {
        String::new()
    };

    let ear_protection_fn = if options.ear_protection {
        r#"
/// Ear-protection soft limiter: transparent below 0.9, smoothly limits to ±1.0.
/// Cubic soft-knee — zero latency, no state, no artifacts.
/// Disable with `--no-ear-protection` if you need unprocessed output for measurement.
#[inline(always)]
fn ear_protection_limit(x: f32) -> f32 {
    const KNEE: f32 = 0.9;
    if x.abs() <= KNEE {
        x
    } else {
        // Cubic soft-clip: maps [KNEE, inf) -> [KNEE, 1.0) smoothly
        let sign = x.signum();
        let abs_x = x.abs();
        let over = abs_x - KNEE;
        let range = 1.0 - KNEE; // 0.1
        let t = (over / range).min(1.0); // 0..1 in the knee region
        // Cubic ease-out: fast rise, smooth approach to 1.0
        sign * (KNEE + range * (1.0 - (1.0 - t) * (1.0 - t) * (1.0 - t)))
    }
}
"#
        .to_string()
    } else {
        String::new()
    };

    format!(
        r#"// =============================================================================
// {display_name} — generated by Melange
//
// This code is derived from GPL-licensed templates and device models in the
// Melange project and is therefore subject to the GNU General Public License
// v3.0 or later. See https://www.gnu.org/licenses/gpl-3.0.html
// =============================================================================
//
// This file (lib.rs) is your plugin wrapper — customize freely:
//   - Parameter names, ranges, and defaults
//   - GUI (nih-plug supports VIZIA and iced)
//   - Presets and metadata
//   - Processing pre/post (e.g. wet/dry mix)
//
// circuit.rs is generated — don't edit it by hand. To update it after
// changing component values in your .cir file:
//
//   melange compile your-circuit.cir --format code -o src/circuit.rs
//
// This regenerates circuit.rs without touching lib.rs, so your
// customizations are preserved.
// =============================================================================

use nih_plug::prelude::*;
use std::sync::Arc;

mod circuit;
{circuit_import}
{ear_protection_fn}
{plugin_struct}

{plugin_default}

{params_struct}

impl Plugin for CircuitPlugin {{
    const NAME: &'static str = "{display_name}";
    const VENDOR: &'static str = "{vendor}";
    const URL: &'static str = "{url}";
    const EMAIL: &'static str = "{email}";
    const VERSION: &'static str = env!("CARGO_PKG_VERSION");

    const AUDIO_IO_LAYOUTS: &'static [AudioIOLayout] = &[
        AudioIOLayout {{
            main_input_channels: NonZeroU32::new({num_channels}),
            main_output_channels: NonZeroU32::new({num_channels}),
            aux_input_ports: &[],
            aux_output_ports: &[],
            names: PortNames::const_default(),
        }},
    ];

    const SAMPLE_ACCURATE_AUTOMATION: bool = false;

    type SysExMessage = ();
    type BackgroundTask = ();

    fn params(&self) -> Arc<dyn Params> {{
        self.params.clone()
    }}

{init_method}

{reset_method}

{deactivate_method}
{latency_method}
    fn process(
        &mut self,
        buffer: &mut Buffer,
        _aux: &mut AuxiliaryBuffers,
        _context: &mut impl ProcessContext<Self>,
    ) -> ProcessStatus {{
{process_loop}
        ProcessStatus::Normal
    }}
}}

impl ClapPlugin for CircuitPlugin {{
    const CLAP_ID: &'static str = "{clap_id}";
    const CLAP_DESCRIPTION: Option<&'static str> = Some("Circuit modeled with Melange");
    const CLAP_MANUAL_URL: Option<&'static str> = Some(Self::URL);
    const CLAP_SUPPORT_URL: Option<&'static str> = None;

    const CLAP_FEATURES: &'static [ClapFeature] = &[
        ClapFeature::AudioEffect,
        {clap_channel_feature},
    ];
}}

impl Vst3Plugin for CircuitPlugin {{
    const VST3_CLASS_ID: [u8; 16] = *b"{vst3_id_str}";

    const VST3_SUBCATEGORIES: &'static [Vst3SubCategory] = &[
        Vst3SubCategory::Fx,
    ];
}}

nih_export_clap!(CircuitPlugin);
nih_export_vst3!(CircuitPlugin);
"#
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    // === Cargo.toml generation tests ===

    #[test]
    fn cargo_toml_contains_package_name() {
        let toml = test_generate_cargo_toml("my-circuit");
        assert!(toml.contains("name = \"my-circuit\""));
    }

    #[test]
    fn cargo_toml_contains_nih_plug_dependency() {
        let toml = test_generate_cargo_toml("test");
        assert!(toml.contains("nih_plug"));
        assert!(toml.contains("git = \"https://github.com/robbert-vdh/nih-plug.git\""));
    }

    #[test]
    fn cargo_toml_cdylib_crate_type() {
        let toml = test_generate_cargo_toml("test");
        assert!(toml.contains("crate-type = [\"cdylib\"]"));
    }

    #[test]
    fn cargo_toml_edition_2021() {
        let toml = test_generate_cargo_toml("test");
        assert!(toml.contains("edition = \"2021\""));
    }

    // === Display name formatting tests ===

    #[test]
    fn display_name_simple() {
        let lib = test_generate_lib_rs("screamer", false, &[]);
        assert!(lib.contains("const NAME: &'static str = \"Screamer\""));
    }

    #[test]
    fn display_name_hyphenated() {
        let lib = test_generate_lib_rs("tube-screamer", false, &[]);
        assert!(lib.contains("const NAME: &'static str = \"Tube Screamer\""));
    }

    #[test]
    fn display_name_multi_hyphen() {
        let lib = test_generate_lib_rs("big-muff-pi", false, &[]);
        assert!(lib.contains("const NAME: &'static str = \"Big Muff Pi\""));
    }

    #[test]
    fn display_name_already_capitalized() {
        // Should lowercase the rest of the word after capitalizing first char
        let lib = test_generate_lib_rs("LOUD", false, &[]);
        assert!(lib.contains("const NAME: &'static str = \"Loud\""));
    }

    // === CLAP ID tests ===

    #[test]
    fn clap_id_format() {
        let lib = test_generate_lib_rs("my-plugin", false, &[]);
        assert!(lib.contains("const CLAP_ID: &'static str = \"com.melange.my-plugin\""));
    }

    // === VST3 ID tests ===

    #[test]
    fn vst3_id_is_16_bytes() {
        let lib = test_generate_lib_rs("test-circuit", false, &[]);
        // The VST3 ID is embedded as b"..." which should be 16 chars
        // Find the VST3_CLASS_ID line
        let vst3_line = lib
            .lines()
            .find(|l| l.contains("VST3_CLASS_ID"))
            .expect("Should have VST3_CLASS_ID line");
        // Extract the b"..." content
        let start = vst3_line.find("*b\"").expect("Should have b\"") + 3;
        let end = vst3_line[start..]
            .find('"')
            .expect("Should have closing quote")
            + start;
        let id_str = &vst3_line[start..end];
        assert_eq!(
            id_str.len(),
            16,
            "VST3 ID should be exactly 16 bytes, got {}",
            id_str.len()
        );
    }

    #[test]
    fn vst3_id_printable_ascii() {
        let lib = test_generate_lib_rs("test-circuit", false, &[]);
        let vst3_line = lib
            .lines()
            .find(|l| l.contains("VST3_CLASS_ID"))
            .expect("Should have VST3_CLASS_ID line");
        let start = vst3_line.find("*b\"").unwrap() + 3;
        let end = vst3_line[start..].find('"').unwrap() + start;
        let id_str = &vst3_line[start..end];
        for ch in id_str.chars() {
            assert!(
                ch.is_ascii_uppercase(),
                "VST3 ID char '{}' should be uppercase ASCII letter",
                ch
            );
        }
    }

    #[test]
    fn vst3_id_deterministic() {
        let lib1 = test_generate_lib_rs("my-circuit", false, &[]);
        let lib2 = test_generate_lib_rs("my-circuit", false, &[]);
        // Same input should produce same VST3 ID
        let extract_id = |lib: &str| -> String {
            let line = lib.lines().find(|l| l.contains("VST3_CLASS_ID")).unwrap();
            let start = line.find("*b\"").unwrap() + 3;
            let end = line[start..].find('"').unwrap() + start;
            line[start..end].to_string()
        };
        assert_eq!(extract_id(&lib1), extract_id(&lib2));
    }

    #[test]
    fn vst3_id_differs_for_different_names() {
        let lib1 = test_generate_lib_rs("circuit-a", false, &[]);
        let lib2 = test_generate_lib_rs("circuit-b", false, &[]);
        let extract_id = |lib: &str| -> String {
            let line = lib.lines().find(|l| l.contains("VST3_CLASS_ID")).unwrap();
            let start = line.find("*b\"").unwrap() + 3;
            let end = line[start..].find('"').unwrap() + start;
            line[start..end].to_string()
        };
        assert_ne!(extract_id(&lib1), extract_id(&lib2));
    }

    // === Plugin structure tests (no params) ===

    #[test]
    fn lib_no_params_has_empty_params_struct() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("#[derive(Params, Default)]"));
        assert!(lib.contains("pub struct CircuitParams {}"));
    }

    #[test]
    fn lib_contains_plugin_struct() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("pub struct CircuitPlugin"));
        assert!(lib.contains("params: Arc<CircuitParams>"));
        assert!(lib.contains("circuit_states: Vec<CircuitState>"));
    }

    #[test]
    fn lib_contains_default_impl_with_stereo() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("circuit_states: vec![CircuitState::default(); 2]"));
    }

    #[test]
    fn lib_contains_plugin_trait_impl() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("impl Plugin for CircuitPlugin"));
        assert!(lib.contains("const VENDOR: &'static str = \"Melange\""));
        assert!(lib.contains("const SAMPLE_ACCURATE_AUTOMATION: bool = false"));
    }

    #[test]
    fn lib_contains_stereo_io_layout() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("main_input_channels: NonZeroU32::new(2)"));
        assert!(lib.contains("main_output_channels: NonZeroU32::new(2)"));
    }

    #[test]
    fn lib_contains_initialize_method() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("fn initialize("));
        // Should dynamically determine channel count
        assert!(lib.contains("main_input_channels"));
    }

    #[test]
    fn lib_contains_reset_method() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("fn reset(&mut self)"));
        assert!(lib.contains("state.reset()"));
    }

    #[test]
    fn lib_contains_process_method() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("fn process("));
        assert!(lib.contains("ProcessStatus::Normal"));
    }

    #[test]
    fn lib_process_has_finite_check() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("out.is_finite()"));
        assert!(lib.contains("clamp(-1.0, 1.0)"));
    }

    #[test]
    fn lib_contains_clap_plugin_impl() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("impl ClapPlugin for CircuitPlugin"));
        assert!(lib.contains("ClapFeature::AudioEffect"));
        assert!(lib.contains("ClapFeature::Stereo"));
    }

    #[test]
    fn lib_contains_vst3_plugin_impl() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("impl Vst3Plugin for CircuitPlugin"));
        assert!(lib.contains("Vst3SubCategory::Fx"));
    }

    #[test]
    fn lib_contains_export_macros() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("nih_export_clap!(CircuitPlugin)"));
        assert!(lib.contains("nih_export_vst3!(CircuitPlugin)"));
    }

    #[test]
    fn lib_imports_circuit_module() {
        let lib = test_generate_lib_rs("test", false, &[]);
        assert!(lib.contains("mod circuit;"));
        assert!(lib.contains("use circuit::{process_sample, CircuitState}"));
    }

    // === Level parameters tests ===

    #[test]
    fn lib_with_level_params_has_input_level() {
        let lib = test_generate_lib_rs("test", true, &[]);
        assert!(lib.contains("pub input_level: FloatParam"));
        assert!(lib.contains("#[id = \"input_level\"]"));
    }

    #[test]
    fn lib_with_level_params_has_output_level() {
        let lib = test_generate_lib_rs("test", true, &[]);
        assert!(lib.contains("pub output_level: FloatParam"));
        assert!(lib.contains("#[id = \"output_level\"]"));
    }

    #[test]
    fn lib_with_level_params_has_gain_smoothing() {
        let lib = test_generate_lib_rs("test", true, &[]);
        assert!(lib.contains("input_gain"));
        assert!(lib.contains("output_gain"));
        assert!(lib.contains("db_to_gain_fast"));
    }

    #[test]
    fn lib_with_level_params_has_db_range() {
        let lib = test_generate_lib_rs("test", true, &[]);
        assert!(lib.contains("Input Level"));
        assert!(lib.contains("Output Level"));
        assert!(lib.contains("\" dB\""));
    }

    #[test]
    fn lib_with_level_params_has_derive_params() {
        let lib = test_generate_lib_rs("test", true, &[]);
        assert!(lib.contains("#[derive(Params)]"));
        // Should NOT have Default derive since it has custom Default impl
        assert!(lib.contains("impl Default for CircuitParams"));
    }

    #[test]
    fn lib_with_level_params_applies_gain_in_process() {
        let lib = test_generate_lib_rs("test", true, &[]);
        // Input gain applied to sample
        assert!(lib.contains("*sample as f64 * input_gain as f64"));
        // Output gain applied to result
        assert!(lib.contains("out * output_gain"));
    }

    // === Pot parameter tests ===

    #[test]
    fn lib_with_single_pot() {
        let pots = vec![PotParamInfo {
            index: 0,
            name: "R1 (Tone)".to_string(),
            min_resistance: 100.0,
            max_resistance: 50000.0,
            default_resistance: 25000.0,
        }];
        let lib = test_generate_lib_rs("test", false, &pots);
        assert!(lib.contains("#[id = \"pot_0\"]"));
        assert!(lib.contains("pub pot_0: FloatParam"));
        assert!(lib.contains("\"R1 (Tone)\""));
        // Pot params use normalized 0.0-1.0 range displayed as percentage
        assert!(lib.contains("min: 0.0"));
        assert!(lib.contains("max: 1.0"));
    }

    #[test]
    fn lib_with_pot_has_percent_unit() {
        let pots = vec![PotParamInfo {
            index: 0,
            name: "R1".to_string(),
            min_resistance: 100.0,
            max_resistance: 10000.0,
            default_resistance: 5000.0,
        }];
        let lib = test_generate_lib_rs("test", false, &pots);
        // Pot parameters display as percentage (0-100%)
        assert!(
            lib.contains(r#".with_unit("%")"#),
            "Should contain percent unit in generated code"
        );
    }

    #[test]
    fn lib_with_pot_converts_position_to_resistance() {
        let pots = vec![PotParamInfo {
            index: 0,
            name: "R1".to_string(),
            min_resistance: 100.0,
            max_resistance: 10000.0,
            default_resistance: 5000.0,
        }];
        let lib = test_generate_lib_rs("test", false, &pots);
        // Position-to-resistance conversion: min + position * (max - min)
        assert!(lib.contains("self.params.pot_0.value()"));
        assert!(lib.contains("set_pot_0(pot_0_val)"));
    }

    #[test]
    fn lib_with_two_pots() {
        let pots = vec![
            PotParamInfo {
                index: 0,
                name: "R1 (Tone)".to_string(),
                min_resistance: 100.0,
                max_resistance: 50000.0,
                default_resistance: 25000.0,
            },
            PotParamInfo {
                index: 1,
                name: "R5 (Volume)".to_string(),
                min_resistance: 0.0,
                max_resistance: 100000.0,
                default_resistance: 50000.0,
            },
        ];
        let lib = test_generate_lib_rs("test", false, &pots);
        assert!(lib.contains("#[id = \"pot_0\"]"));
        assert!(lib.contains("#[id = \"pot_1\"]"));
        assert!(lib.contains("pub pot_0: FloatParam"));
        assert!(lib.contains("pub pot_1: FloatParam"));
        assert!(lib.contains("set_pot_0(pot_0_val)"));
        assert!(lib.contains("set_pot_1(pot_1_val)"));
    }

    #[test]
    fn lib_with_level_params_and_pots() {
        let pots = vec![PotParamInfo {
            index: 0,
            name: "R1".to_string(),
            min_resistance: 100.0,
            max_resistance: 10000.0,
            default_resistance: 5000.0,
        }];
        let lib = test_generate_lib_rs("test", true, &pots);
        // Should have both level params and pot params
        assert!(lib.contains("pub input_level: FloatParam"));
        assert!(lib.contains("pub output_level: FloatParam"));
        assert!(lib.contains("pub pot_0: FloatParam"));
        // Process should have both gains and pot values
        assert!(lib.contains("input_gain"));
        assert!(lib.contains("output_gain"));
        assert!(lib.contains("pot_0_val"));
    }

    // === Project generation (filesystem) tests ===

    #[test]
    fn generate_plugin_project_creates_directory_structure() {
        let dir = std::env::temp_dir().join("melange_test_plugin_project");
        let _ = std::fs::remove_dir_all(&dir); // clean up from previous run
        let result = generate_plugin_project(
            &dir,
            "// circuit code stub",
            "test-circuit",
            false,
            &[],
            &[],
            &[],
            &[],
            1,
        );
        assert!(
            result.is_ok(),
            "generate_plugin_project should succeed: {:?}",
            result.err()
        );
        assert!(dir.join("Cargo.toml").exists(), "Should create Cargo.toml");
        assert!(dir.join("src").is_dir(), "Should create src/");
        assert!(
            dir.join("src/circuit.rs").exists(),
            "Should create src/circuit.rs"
        );
        assert!(dir.join("src/lib.rs").exists(), "Should create src/lib.rs");
        // Clean up
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn generate_plugin_project_writes_circuit_code() {
        let dir = std::env::temp_dir().join("melange_test_plugin_circuit");
        let _ = std::fs::remove_dir_all(&dir);
        let circuit_code = "// This is the generated circuit code\npub fn process_sample() {}";
        let result = generate_plugin_project(&dir, circuit_code, "test", false, &[], &[], &[], &[], 1);
        assert!(result.is_ok());
        let written = std::fs::read_to_string(dir.join("src/circuit.rs")).unwrap();
        assert_eq!(written, circuit_code);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn generate_plugin_project_cargo_toml_has_correct_name() {
        let dir = std::env::temp_dir().join("melange_test_plugin_name");
        let _ = std::fs::remove_dir_all(&dir);
        let result =
            generate_plugin_project(&dir, "// code", "my-cool-plugin", false, &[], &[], &[], &[], 1);
        assert!(result.is_ok());
        let toml = std::fs::read_to_string(dir.join("Cargo.toml")).unwrap();
        assert!(toml.contains("name = \"my-cool-plugin\""));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn generate_plugin_project_lib_rs_has_plugin_code() {
        let dir = std::env::temp_dir().join("melange_test_plugin_lib");
        let _ = std::fs::remove_dir_all(&dir);
        let result =
            generate_plugin_project(&dir, "// code", "my-plugin", false, &[], &[], &[], &[], 1);
        assert!(result.is_ok());
        let lib_rs = std::fs::read_to_string(dir.join("src/lib.rs")).unwrap();
        assert!(lib_rs.contains("CircuitPlugin"));
        assert!(lib_rs.contains("My Plugin")); // display name
        assert!(lib_rs.contains("com.melange.my-plugin")); // CLAP ID
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn generate_plugin_project_idempotent() {
        let dir = std::env::temp_dir().join("melange_test_plugin_idempotent");
        let _ = std::fs::remove_dir_all(&dir);
        // Generate twice, should not fail
        let _ = generate_plugin_project(&dir, "// v1", "test", false, &[], &[], &[], &[], 1);
        let result = generate_plugin_project(&dir, "// v2", "test", false, &[], &[], &[], &[], 1);
        assert!(result.is_ok());
        // Second write should overwrite
        let circuit = std::fs::read_to_string(dir.join("src/circuit.rs")).unwrap();
        assert_eq!(circuit, "// v2");
        let _ = std::fs::remove_dir_all(&dir);
    }

    // === Edge cases ===

    #[test]
    fn empty_circuit_name() {
        // Should not panic, just produce empty/minimal output
        let toml = test_generate_cargo_toml("");
        assert!(toml.contains("name = \"\""));
    }

    #[test]
    fn single_char_circuit_name() {
        let lib = test_generate_lib_rs("x", false, &[]);
        assert!(lib.contains("const NAME: &'static str = \"X\""));
        assert!(lib.contains("com.melange.x"));
    }

    #[test]
    fn long_circuit_name_vst3_id_still_16_bytes() {
        // A name longer than 16 bytes should still produce a 16-byte VST3 ID
        let lib = test_generate_lib_rs(
            "this-is-a-very-long-circuit-name-that-exceeds-sixteen-bytes",
            false,
            &[],
        );
        let vst3_line = lib.lines().find(|l| l.contains("VST3_CLASS_ID")).unwrap();
        let start = vst3_line.find("*b\"").unwrap() + 3;
        let end = vst3_line[start..].find('"').unwrap() + start;
        assert_eq!(end - start, 16);
    }

    #[test]
    fn pot_with_linear_smoother() {
        let pots = vec![PotParamInfo {
            index: 0,
            name: "R1".to_string(),
            min_resistance: 100.0,
            max_resistance: 10000.0,
            default_resistance: 5000.0,
        }];
        let lib = test_generate_lib_rs("test", false, &pots);
        assert!(lib.contains("SmoothingStyle::Linear(10.0)"));
    }

    // === --name flag tests ===

    #[test]
    fn lib_with_custom_name() {
        let opts = PluginOptions {
            plugin_name: Some("My Custom Plugin"),
            ..Default::default()
        };
        let lib = generate_lib_rs("test-circuit", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("const NAME: &'static str = \"My Custom Plugin\""));
    }

    #[test]
    fn lib_without_custom_name_uses_auto() {
        let opts = PluginOptions::default();
        let lib = generate_lib_rs("tube-screamer", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("const NAME: &'static str = \"Tube Screamer\""));
    }

    // === --mono flag tests ===

    #[test]
    fn lib_mono_has_single_channel_io() {
        let opts = PluginOptions {
            mono: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("main_input_channels: NonZeroU32::new(1)"));
        assert!(lib.contains("main_output_channels: NonZeroU32::new(1)"));
    }

    #[test]
    fn lib_mono_has_mono_clap_feature() {
        let opts = PluginOptions {
            mono: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("ClapFeature::Mono"));
        assert!(!lib.contains("ClapFeature::Stereo"));
    }

    #[test]
    fn lib_stereo_default_has_stereo_clap_feature() {
        let opts = PluginOptions::default();
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("ClapFeature::Stereo"));
        assert!(!lib.contains("ClapFeature::Mono"));
    }

    #[test]
    fn lib_mono_has_single_state() {
        let opts = PluginOptions {
            mono: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("circuit_states: vec![CircuitState::default(); 1]"));
    }

    // === --wet-dry-mix flag tests ===

    #[test]
    fn lib_with_wet_dry_mix_has_mix_param() {
        let opts = PluginOptions {
            wet_dry_mix: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("#[id = \"mix\"]"));
        assert!(lib.contains("pub mix: FloatParam"));
    }

    #[test]
    fn lib_with_wet_dry_mix_has_blend_in_process() {
        let opts = PluginOptions {
            wet_dry_mix: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("let dry = *sample;"));
        assert!(lib.contains("mix * out + (1.0 - mix) * dry"));
    }

    #[test]
    fn lib_without_wet_dry_mix_has_no_mix_param() {
        let opts = PluginOptions::default();
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        assert!(!lib.contains("pub mix: FloatParam"));
        assert!(!lib.contains("let dry = *sample;"));
    }

    #[test]
    fn lib_with_wet_dry_mix_default_is_fully_wet() {
        let opts = PluginOptions {
            wet_dry_mix: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", false, &[], &[], &[], &[], 1, 1, &opts);
        // Mix default should be 1.0 (fully wet)
        assert!(lib.contains("\"Mix\""));
        assert!(lib.contains("1.0,"));
    }

    // === README generation tests ===

    #[test]
    fn generate_plugin_project_creates_readme() {
        let dir = std::env::temp_dir().join("melange_test_plugin_readme");
        let _ = std::fs::remove_dir_all(&dir);
        let result = generate_plugin_project(
            &dir,
            "// circuit code stub",
            "test-circuit",
            false,
            &[],
            &[],
            &[],
            &[],
            1,
        );
        assert!(result.is_ok());
        assert!(dir.join("README.md").exists(), "Should create README.md");
        let readme = std::fs::read_to_string(dir.join("README.md")).unwrap();
        assert!(readme.contains("Test Circuit"));
        // README should document the working build path via nih_plug_xtask and the
        // bundled build.sh convenience wrapper.
        assert!(readme.contains("nih_plug_xtask"));
        assert!(readme.contains("bundle test-circuit --release"));
        assert!(readme.contains("build.sh"));
        assert!(readme.contains("Melange"));
        // build.sh should be emitted alongside README.
        assert!(
            dir.join("build.sh").exists(),
            "Should create build.sh wrapper"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn readme_uses_custom_plugin_name() {
        let readme = generate_readme("my-circuit", Some("My Custom Name"));
        assert!(readme.contains("# My Custom Name"));
    }

    #[test]
    fn readme_uses_auto_name_when_no_custom() {
        let readme = generate_readme("tube-screamer", None);
        assert!(readme.contains("# Tube Screamer"));
    }

    // === Combined flag tests ===

    #[test]
    fn lib_mono_with_level_params() {
        let opts = PluginOptions {
            mono: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", true, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("main_input_channels: NonZeroU32::new(1)"));
        assert!(lib.contains("pub input_level: FloatParam"));
        assert!(lib.contains("input_gain"));
    }

    #[test]
    fn lib_wet_dry_with_level_params() {
        let opts = PluginOptions {
            wet_dry_mix: true,
            ..Default::default()
        };
        let lib = generate_lib_rs("test", true, &[], &[], &[], &[], 1, 1, &opts);
        assert!(lib.contains("pub input_level: FloatParam"));
        assert!(lib.contains("pub mix: FloatParam"));
        assert!(lib.contains("let dry = *sample;"));
        assert!(lib.contains("mix * out + (1.0 - mix) * dry"));
    }
}
