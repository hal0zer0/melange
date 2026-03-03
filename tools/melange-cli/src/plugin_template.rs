//! Plugin project template generation

use std::path::Path;

use anyhow::Result;

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

/// Switch info for plugin parameter generation.
pub struct SwitchParamInfo {
    /// Index (0-based)
    pub index: usize,
    /// Human-readable name (e.g., "SW0 (HF Boost Freq)")
    pub name: String,
    /// Number of positions
    pub num_positions: usize,
}

/// Generate a complete plugin project
pub fn generate_plugin_project(
    output_dir: &Path,
    circuit_code: &str,
    circuit_name: &str,
    with_level_params: bool,
    pots: &[PotParamInfo],
    switches: &[SwitchParamInfo],
    num_outputs: usize,
) -> Result<()> {
    std::fs::create_dir_all(output_dir.join("src"))?;
    std::fs::write(output_dir.join("Cargo.toml"), generate_cargo_toml(circuit_name))?;
    std::fs::write(output_dir.join("src/circuit.rs"), circuit_code)?;
    std::fs::write(output_dir.join("src/lib.rs"), generate_lib_rs(circuit_name, with_level_params, pots, switches, num_outputs))?;
    Ok(())
}

fn generate_cargo_toml(circuit_name: &str) -> String {
    format!(r#"[package]
name = "{circuit_name}"
version = "0.1.0"
edition = "2021"

[dependencies]
nih_plug = {{ git = "https://github.com/robbert-vdh/nih-plug.git", rev = "28b149e" }}

[lib]
crate-type = ["cdylib"]
"#)
}

#[cfg(test)]
pub(crate) fn test_generate_cargo_toml(circuit_name: &str) -> String {
    generate_cargo_toml(circuit_name)
}

#[cfg(test)]
pub(crate) fn test_generate_lib_rs(circuit_name: &str, with_level_params: bool, pots: &[PotParamInfo]) -> String {
    generate_lib_rs(circuit_name, with_level_params, pots, &[], 1)
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
/// Uses XOR folding to produce 16 bytes, then maps each to uppercase ASCII
/// so the result is valid for a `b"..."` literal.
fn compute_vst3_id(circuit_name: &str) -> String {
    let mut hash = [0u8; 16];
    for (i, &b) in circuit_name.as_bytes().iter().enumerate() {
        hash[i % 16] ^= b;
    }
    for h in hash.iter_mut() {
        *h = b'A' + (*h % 26);
    }
    String::from_utf8(hash.to_vec()).unwrap()
}

fn generate_pot_field(pot: &PotParamInfo) -> String {
    format!(
        "    #[id = \"pot_{idx}\"]\n    pub pot_{idx}: FloatParam,\n",
        idx = pot.index,
    )
}

fn generate_pot_default(pot: &PotParamInfo) -> String {
    format!(
        r#"            pot_{idx}: FloatParam::new(
                "{name}",
                {default:.1},
                FloatRange::Linear {{
                    min: {min:.1},
                    max: {max:.1},
                }},
            )
            .with_smoother(SmoothingStyle::Linear(10.0))
            .with_unit(" \u{{2126}}"),
"#,
        idx = pot.index,
        name = pot.name,
        default = pot.default_resistance,
        min = pot.min_resistance,
        max = pot.max_resistance,
    )
}

fn generate_switch_field(sw: &SwitchParamInfo) -> String {
    format!(
        "    #[id = \"switch_{idx}\"]\n    pub switch_{idx}: IntParam,\n",
        idx = sw.index,
    )
}

fn generate_switch_default(sw: &SwitchParamInfo) -> String {
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
        name = sw.name,
        max = sw.num_positions as i32 - 1,
    )
}

fn generate_params_struct(with_level_params: bool, pots: &[PotParamInfo], switches: &[SwitchParamInfo]) -> String {
    let has_any_params = with_level_params || !pots.is_empty() || !switches.is_empty();
    if !has_any_params {
        return "#[derive(Params, Default)]\npub struct CircuitParams {}".to_string();
    }

    let mut fields = String::new();
    let mut defaults = String::new();

    if with_level_params {
        fields.push_str(LEVEL_PARAM_FIELDS);
        defaults.push_str(LEVEL_PARAM_DEFAULTS);
    }

    for pot in pots {
        fields.push_str(&generate_pot_field(pot));
        defaults.push_str(&generate_pot_default(pot));
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

fn generate_process_loop(with_level_params: bool, pots: &[PotParamInfo], switches: &[SwitchParamInfo], num_outputs: usize) -> String {
    let has_any_params = with_level_params || !pots.is_empty() || !switches.is_empty();
    if !has_any_params && num_outputs <= 1 {
        return r#"        for channel_samples in buffer.iter_samples() {
            for (ch, sample) in channel_samples.into_iter().enumerate() {
                let state = &mut self.circuit_states[ch];
                let out = process_sample(*sample as f64, state)[0] as f32;
                *sample = if out.is_finite() { out.clamp(-1.0, 1.0) } else { 0.0 };
            }
        }"#
            .to_string();
    }
    if !has_any_params && num_outputs > 1 {
        return r#"        for channel_samples in buffer.iter_samples() {
            let input = *channel_samples.into_iter().next().unwrap() as f64;
            let outs = process_sample(input, &mut self.circuit_state);
            for (ch, sample) in channel_samples.into_iter().enumerate() {
                let out = outs[ch.min(NUM_OUTPUTS - 1)] as f32;
                *sample = if out.is_finite() { out.clamp(-1.0, 1.0) } else { 0.0 };
            }
        }"#
            .to_string();
    }

    let gain_reads = if with_level_params {
        "            let input_gain = self.params.input_level.smoothed.next();\n\
         \x20           let output_gain = self.params.output_level.smoothed.next();\n"
    } else {
        ""
    };

    let pot_reads: String = pots
        .iter()
        .map(|p| {
            format!(
                "            let pot_{i}_val = self.params.pot_{i}.smoothed.next() as f64;\n",
                i = p.index,
            )
        })
        .collect();

    // Switch reads: read once per buffer (not per sample — no smoother)
    let switch_reads: String = switches
        .iter()
        .map(|s| {
            format!(
                "            let sw_{i}_pos = self.params.switch_{i}.value() as usize;\n",
                i = s.index,
            )
        })
        .collect();

    let pot_assignments: String = pots
        .iter()
        .map(|p| {
            format!(
                "                state.pot_{i}_resistance = pot_{i}_val;\n",
                i = p.index,
            )
        })
        .collect();

    // Switch assignments: only call set_switch_N when position changed (avoids O(N^3) rebuild)
    let switch_assignments: String = switches
        .iter()
        .map(|s| {
            format!(
                "                if state.switch_{i}_position != sw_{i}_pos {{\n\
                 \x20                   state.set_switch_{i}(sw_{i}_pos);\n\
                 \x20               }}\n",
                i = s.index,
            )
        })
        .collect();

    let sample_processing = if num_outputs > 1 {
        // Multi-output: handled differently (mono input → stereo output)
        // This path shouldn't be reached because multi-output with params is handled above
        if with_level_params {
            "                let input = *sample as f64 * input_gain as f64;\n\
             \x20               let out = process_sample(input, state)[0] as f32;\n\
             \x20               let out = out * output_gain;\n"
        } else {
            "                let out = process_sample(*sample as f64, state)[0] as f32;\n"
        }
    } else if with_level_params {
        "                let input = *sample as f64 * input_gain as f64;\n\
         \x20               let out = process_sample(input, state)[0] as f32;\n\
         \x20               let out = out * output_gain;\n"
    } else {
        "                let out = process_sample(*sample as f64, state)[0] as f32;\n"
    };

    format!(
        "\x20       for channel_samples in buffer.iter_samples() {{\n\
         {gain_reads}\
         {pot_reads}\
         {switch_reads}\
         \x20           for (ch, sample) in channel_samples.into_iter().enumerate() {{\n\
         \x20               let state = &mut self.circuit_states[ch];\n\
         {pot_assignments}\
         {switch_assignments}\
         {sample_processing}\
         \x20               *sample = if out.is_finite() {{ out.clamp(-1.0, 1.0) }} else {{ 0.0 }};\n\
         \x20           }}\n\
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
                util::db_to_gain(-12.0),
                FloatRange::Skewed {
                    min: util::db_to_gain(-36.0),
                    max: util::db_to_gain(12.0),
                    factor: FloatRange::gain_skew_factor(-36.0, 12.0),
                },
            )
            .with_smoother(SmoothingStyle::Logarithmic(50.0))
            .with_unit(" dB")
            .with_value_to_string(formatters::v2s_f32_gain_to_db(1))
            .with_string_to_value(formatters::s2v_f32_gain_to_db()),

            output_level: FloatParam::new(
                "Output Level",
                util::db_to_gain(0.0),
                FloatRange::Skewed {
                    min: util::db_to_gain(-60.0),
                    max: util::db_to_gain(12.0),
                    factor: FloatRange::gain_skew_factor(-60.0, 12.0),
                },
            )
            .with_smoother(SmoothingStyle::Logarithmic(50.0))
            .with_unit(" dB")
            .with_value_to_string(formatters::v2s_f32_gain_to_db(1))
            .with_string_to_value(formatters::s2v_f32_gain_to_db()),
"#;

fn generate_lib_rs(circuit_name: &str, with_level_params: bool, pots: &[PotParamInfo], switches: &[SwitchParamInfo], num_outputs: usize) -> String {
    let display_name: String = circuit_name.split('-').map(capitalize_word).collect::<Vec<_>>().join(" ");
    let clap_id = format!("com.melange.{circuit_name}");
    let vst3_id_str = compute_vst3_id(circuit_name);
    let params_struct = generate_params_struct(with_level_params, pots, switches);
    let process_loop = generate_process_loop(with_level_params, pots, switches, num_outputs);

    // Conditional sections based on num_outputs
    let (circuit_import, plugin_struct, plugin_default, init_method, reset_method) = if num_outputs > 1 {
        // Multi-output: single circuit state, mono input → multi-output
        (
            format!("use circuit::{{process_sample, CircuitState, NUM_OUTPUTS}};"),
            format!(
                "pub struct CircuitPlugin {{\n\
                 \x20   params: Arc<CircuitParams>,\n\
                 \x20   circuit_state: CircuitState,\n\
                 \x20   current_sample_rate: f64,\n\
                 }}"
            ),
            format!(
                "impl Default for CircuitPlugin {{\n\
                 \x20   fn default() -> Self {{\n\
                 \x20       Self {{\n\
                 \x20           params: Arc::new(CircuitParams::default()),\n\
                 \x20           circuit_state: CircuitState::default(),\n\
                 \x20           current_sample_rate: 0.0,\n\
                 \x20       }}\n\
                 \x20   }}\n\
                 }}"
            ),
            format!(
                "    fn initialize(\n\
                 \x20       &mut self,\n\
                 \x20       _audio_io_layout: &AudioIOLayout,\n\
                 \x20       buffer_config: &BufferConfig,\n\
                 \x20       _context: &mut impl InitContext<Self>,\n\
                 \x20   ) -> bool {{\n\
                 \x20       self.current_sample_rate = buffer_config.sample_rate as f64;\n\
                 \x20       self.circuit_state = CircuitState::default();\n\
                 \x20       self.circuit_state.set_sample_rate(buffer_config.sample_rate as f64);\n\
                 \x20       true\n\
                 \x20   }}"
            ),
            format!(
                "    fn reset(&mut self) {{\n\
                 \x20       let sr = self.current_sample_rate;\n\
                 \x20       self.circuit_state = CircuitState::default();\n\
                 \x20       if sr > 0.0 {{\n\
                 \x20           self.circuit_state.set_sample_rate(sr);\n\
                 \x20       }}\n\
                 \x20   }}"
            ),
        )
    } else {
        // Single output: per-channel state duplication (stereo from mono)
        (
            format!("use circuit::{{process_sample, CircuitState}};"),
            format!(
                "pub struct CircuitPlugin {{\n\
                 \x20   params: Arc<CircuitParams>,\n\
                 \x20   circuit_states: Vec<CircuitState>,\n\
                 \x20   current_sample_rate: f64,\n\
                 }}"
            ),
            format!(
                "impl Default for CircuitPlugin {{\n\
                 \x20   fn default() -> Self {{\n\
                 \x20       Self {{\n\
                 \x20           params: Arc::new(CircuitParams::default()),\n\
                 \x20           circuit_states: vec![CircuitState::default(); 2],\n\
                 \x20           current_sample_rate: 0.0,\n\
                 \x20       }}\n\
                 \x20   }}\n\
                 }}"
            ),
            format!(
                "    fn initialize(\n\
                 \x20       &mut self,\n\
                 \x20       audio_io_layout: &AudioIOLayout,\n\
                 \x20       buffer_config: &BufferConfig,\n\
                 \x20       _context: &mut impl InitContext<Self>,\n\
                 \x20   ) -> bool {{\n\
                 \x20       let num_channels = audio_io_layout.main_input_channels\n\
                 \x20           .map(|c| c.get() as usize).unwrap_or(2);\n\
                 \x20       self.current_sample_rate = buffer_config.sample_rate as f64;\n\
                 \x20       self.circuit_states = (0..num_channels).map(|_| {{\n\
                 \x20           let mut s = CircuitState::default();\n\
                 \x20           s.set_sample_rate(buffer_config.sample_rate as f64);\n\
                 \x20           s\n\
                 \x20       }}).collect();\n\
                 \x20       true\n\
                 \x20   }}"
            ),
            format!(
                "    fn reset(&mut self) {{\n\
                 \x20       let sr = self.current_sample_rate;\n\
                 \x20       for state in &mut self.circuit_states {{\n\
                 \x20           *state = CircuitState::default();\n\
                 \x20           if sr > 0.0 {{\n\
                 \x20               state.set_sample_rate(sr);\n\
                 \x20           }}\n\
                 \x20       }}\n\
                 \x20   }}"
            ),
        )
    };

    format!(
        r#"// =============================================================================
// {display_name} — generated by Melange
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

{plugin_struct}

{plugin_default}

{params_struct}

impl Plugin for CircuitPlugin {{
    const NAME: &'static str = "{display_name}";
    const VENDOR: &'static str = "Melange";
    const URL: &'static str = "https://github.com/melange";
    const EMAIL: &'static str = "dev@melange.audio";
    const VERSION: &'static str = env!("CARGO_PKG_VERSION");

    const AUDIO_IO_LAYOUTS: &'static [AudioIOLayout] = &[
        AudioIOLayout {{
            main_input_channels: NonZeroU32::new(2),
            main_output_channels: NonZeroU32::new(2),
            aux_input_ports: &[],
            aux_output_ports: &[],
            names: PortNames::const_default(),
        }},
    ];

    const SAMPLE_ACCURATE_AUTOMATION: bool = true;

    type SysExMessage = ();
    type BackgroundTask = ();

    fn params(&self) -> Arc<dyn Params> {{
        self.params.clone()
    }}

{init_method}

{reset_method}

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
        ClapFeature::Stereo,
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
        let vst3_line = lib.lines()
            .find(|l| l.contains("VST3_CLASS_ID"))
            .expect("Should have VST3_CLASS_ID line");
        // Extract the b"..." content
        let start = vst3_line.find("*b\"").expect("Should have b\"") + 3;
        let end = vst3_line[start..].find('"').expect("Should have closing quote") + start;
        let id_str = &vst3_line[start..end];
        assert_eq!(id_str.len(), 16, "VST3 ID should be exactly 16 bytes, got {}", id_str.len());
    }

    #[test]
    fn vst3_id_printable_ascii() {
        let lib = test_generate_lib_rs("test-circuit", false, &[]);
        let vst3_line = lib.lines()
            .find(|l| l.contains("VST3_CLASS_ID"))
            .expect("Should have VST3_CLASS_ID line");
        let start = vst3_line.find("*b\"").unwrap() + 3;
        let end = vst3_line[start..].find('"').unwrap() + start;
        let id_str = &vst3_line[start..end];
        for ch in id_str.chars() {
            assert!(ch.is_ascii_uppercase(), "VST3 ID char '{}' should be uppercase ASCII letter", ch);
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
        assert!(lib.contains("const SAMPLE_ACCURATE_AUTOMATION: bool = true"));
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
        assert!(lib.contains("CircuitState::default()"));
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
        assert!(lib.contains("SmoothingStyle::Logarithmic"));
    }

    #[test]
    fn lib_with_level_params_has_db_formatters() {
        let lib = test_generate_lib_rs("test", true, &[]);
        assert!(lib.contains("v2s_f32_gain_to_db"));
        assert!(lib.contains("s2v_f32_gain_to_db"));
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
        assert!(lib.contains("min: 100.0"));
        assert!(lib.contains("max: 50000.0"));
    }

    #[test]
    fn lib_with_pot_has_ohm_unit_string() {
        let pots = vec![PotParamInfo {
            index: 0,
            name: "R1".to_string(),
            min_resistance: 100.0,
            max_resistance: 10000.0,
            default_resistance: 5000.0,
        }];
        let lib = test_generate_lib_rs("test", false, &pots);
        // The template embeds the ohm symbol escape as a literal string in the generated code
        // (it will be interpreted by the Rust compiler when the generated code is compiled)
        assert!(lib.contains(r"\u{2126}"), "Should contain ohm symbol escape in generated code");
    }

    #[test]
    fn lib_with_pot_sets_resistance_in_process() {
        let pots = vec![PotParamInfo {
            index: 0,
            name: "R1".to_string(),
            min_resistance: 100.0,
            max_resistance: 10000.0,
            default_resistance: 5000.0,
        }];
        let lib = test_generate_lib_rs("test", false, &pots);
        assert!(lib.contains("pot_0_val = self.params.pot_0.smoothed.next()"));
        assert!(lib.contains("state.pot_0_resistance = pot_0_val"));
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
        assert!(lib.contains("state.pot_0_resistance = pot_0_val"));
        assert!(lib.contains("state.pot_1_resistance = pot_1_val"));
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
            1,
        );
        assert!(result.is_ok(), "generate_plugin_project should succeed: {:?}", result.err());
        assert!(dir.join("Cargo.toml").exists(), "Should create Cargo.toml");
        assert!(dir.join("src").is_dir(), "Should create src/");
        assert!(dir.join("src/circuit.rs").exists(), "Should create src/circuit.rs");
        assert!(dir.join("src/lib.rs").exists(), "Should create src/lib.rs");
        // Clean up
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn generate_plugin_project_writes_circuit_code() {
        let dir = std::env::temp_dir().join("melange_test_plugin_circuit");
        let _ = std::fs::remove_dir_all(&dir);
        let circuit_code = "// This is the generated circuit code\npub fn process_sample() {}";
        let result = generate_plugin_project(&dir, circuit_code, "test", false, &[], &[], 1);
        assert!(result.is_ok());
        let written = std::fs::read_to_string(dir.join("src/circuit.rs")).unwrap();
        assert_eq!(written, circuit_code);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn generate_plugin_project_cargo_toml_has_correct_name() {
        let dir = std::env::temp_dir().join("melange_test_plugin_name");
        let _ = std::fs::remove_dir_all(&dir);
        let result = generate_plugin_project(&dir, "// code", "my-cool-plugin", false, &[], &[], 1);
        assert!(result.is_ok());
        let toml = std::fs::read_to_string(dir.join("Cargo.toml")).unwrap();
        assert!(toml.contains("name = \"my-cool-plugin\""));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn generate_plugin_project_lib_rs_has_plugin_code() {
        let dir = std::env::temp_dir().join("melange_test_plugin_lib");
        let _ = std::fs::remove_dir_all(&dir);
        let result = generate_plugin_project(&dir, "// code", "my-plugin", false, &[], &[], 1);
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
        let _ = generate_plugin_project(&dir, "// v1", "test", false, &[], &[], 1);
        let result = generate_plugin_project(&dir, "// v2", "test", false, &[], &[], 1);
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
        let lib = test_generate_lib_rs("this-is-a-very-long-circuit-name-that-exceeds-sixteen-bytes", false, &[]);
        let vst3_line = lib.lines()
            .find(|l| l.contains("VST3_CLASS_ID"))
            .unwrap();
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
}
