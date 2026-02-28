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

/// Generate a complete plugin project
pub fn generate_plugin_project(
    output_dir: &Path,
    circuit_code: &str,
    circuit_name: &str,
    with_level_params: bool,
    pots: &[PotParamInfo],
) -> Result<()> {
    // Create directory structure
    std::fs::create_dir_all(output_dir.join("src"))?;

    // Write Cargo.toml
    let cargo_toml = generate_cargo_toml(circuit_name);
    std::fs::write(output_dir.join("Cargo.toml"), cargo_toml)?;

    // Write circuit.rs (the generated code)
    std::fs::write(output_dir.join("src/circuit.rs"), circuit_code)?;

    // Write lib.rs (plugin wrapper)
    let lib_rs = generate_lib_rs(circuit_name, with_level_params, pots);
    std::fs::write(output_dir.join("src/lib.rs"), lib_rs)?;

    Ok(())
}

fn generate_cargo_toml(circuit_name: &str) -> String {
    format!(r#"[package]
name = "{}"
version = "0.1.0"
edition = "2021"

[dependencies]
nih_plug = {{ git = "https://github.com/robbert-vdh/nih-plug.git", rev = "28b149e" }}

[lib]
crate-type = ["cdylib"]
"#, circuit_name)
}

fn generate_lib_rs(circuit_name: &str, with_level_params: bool, pots: &[PotParamInfo]) -> String {
    // Convert to a display-friendly name (e.g., "mordor-screamer" -> "Mordor Screamer")
    let display_name = circuit_name
        .split('-')
        .map(|s| {
            let mut chars = s.chars();
            match chars.next() {
                None => String::new(),
                Some(first) => first.to_uppercase().collect::<String>() + &chars.as_str().to_lowercase(),
            }
        })
        .collect::<Vec<_>>()
        .join(" ");

    // Create unique CLAP ID based on circuit name
    let clap_id = format!("com.melange.{}", circuit_name);

    // Compute a stable 16-byte VST3 ID from circuit name using XOR hash
    let mut hash = [0u8; 16];
    let name_bytes = circuit_name.as_bytes();
    for (i, &b) in name_bytes.iter().enumerate() {
        hash[i % 16] ^= b;
    }
    // Ensure all bytes are printable ASCII for b"..." literal
    for h in hash.iter_mut() {
        *h = b'A' + (*h % 26);
    }
    let vst3_id_str = String::from_utf8(hash.to_vec()).unwrap();

    // Build pot parameter fields and process_sample lines
    let mut pot_param_fields = String::new();
    let mut pot_param_defaults = String::new();
    let mut pot_process_lines = String::new();
    for pot in pots {
        // Parameter field
        pot_param_fields.push_str(&format!(
            "    #[id = \"pot_{}\"]\n    pub pot_{}: FloatParam,\n",
            pot.index, pot.index
        ));
        // Default initialization
        pot_param_defaults.push_str(&format!(
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
        ));
        // Process loop: read param smoothed value (per sample, before inner channel loop)
        pot_process_lines.push_str(&format!(
            "            let pot_{}_val = self.params.pot_{}.smoothed.next() as f64;\n",
            pot.index, pot.index
        ));
    }

    let has_any_params = with_level_params || !pots.is_empty();
    let (params_struct, process_loop) = if has_any_params {
        // Build params struct fields
        let mut fields = String::new();
        let mut defaults = String::new();
        if with_level_params {
            fields.push_str(r#"    #[id = "input_level"]
    pub input_level: FloatParam,
    #[id = "output_level"]
    pub output_level: FloatParam,
"#);
            defaults.push_str(r#"            input_level: FloatParam::new(
                "Input Level",
                util::db_to_gain(0.0),
                FloatRange::Skewed {
                    min: util::db_to_gain(-12.0),
                    max: util::db_to_gain(12.0),
                    factor: FloatRange::gain_skew_factor(-12.0, 12.0),
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
                    min: util::db_to_gain(-30.0),
                    max: util::db_to_gain(12.0),
                    factor: FloatRange::gain_skew_factor(-30.0, 12.0),
                },
            )
            .with_smoother(SmoothingStyle::Logarithmic(50.0))
            .with_unit(" dB")
            .with_value_to_string(formatters::v2s_f32_gain_to_db(1))
            .with_string_to_value(formatters::s2v_f32_gain_to_db()),
"#);
        }
        fields.push_str(&pot_param_fields);
        defaults.push_str(&pot_param_defaults);

        let params = format!(r#"#[derive(Params)]
pub struct CircuitParams {{
{fields}}}

impl Default for CircuitParams {{
    fn default() -> Self {{
        Self {{
{defaults}        }}
    }}
}}"#);

        // Build process loop
        let mut loop_code = String::from("        for channel_samples in buffer.iter_samples() {\n");
        if with_level_params {
            loop_code.push_str("            let input_gain = self.params.input_level.smoothed.next();\n");
            loop_code.push_str("            let output_gain = self.params.output_level.smoothed.next();\n");
        }
        loop_code.push_str(&pot_process_lines);
        loop_code.push_str("            for (ch, sample) in channel_samples.into_iter().enumerate() {\n");
        loop_code.push_str("                let state = &mut self.circuit_states[ch];\n");
        // Set pot resistances on state
        for pot in pots {
            loop_code.push_str(&format!(
                "                state.pot_{}_resistance = pot_{}_val;\n",
                pot.index, pot.index
            ));
        }
        if with_level_params {
            loop_code.push_str("                let input = *sample as f64 * input_gain as f64;\n");
            loop_code.push_str("                let out = process_sample(input, state) as f32;\n");
            loop_code.push_str("                let out = out * output_gain;\n");
        } else {
            loop_code.push_str("                let out = process_sample(*sample as f64, state) as f32;\n");
        }
        loop_code.push_str("                *sample = if out.is_finite() { out.clamp(-1.0, 1.0) } else { 0.0 };\n");
        loop_code.push_str("            }\n");
        loop_code.push_str("        }");

        (params, loop_code)
    } else {
        (
            "#[derive(Params, Default)]\npub struct CircuitParams {}".to_string(),
            r#"        for channel_samples in buffer.iter_samples() {
            for (ch, sample) in channel_samples.into_iter().enumerate() {
                let state = &mut self.circuit_states[ch];
                let out = process_sample(*sample as f64, state) as f32;
                *sample = if out.is_finite() { out.clamp(-1.0, 1.0) } else { 0.0 };
            }
        }"#.to_string(),
        )
    };

    format!(
        r#"use nih_plug::prelude::*;
use std::sync::Arc;

mod circuit;
use circuit::{{process_sample, CircuitState}};

pub struct CircuitPlugin {{
    params: Arc<CircuitParams>,
    circuit_states: Vec<CircuitState>,
}}

impl Default for CircuitPlugin {{
    fn default() -> Self {{
        Self {{
            params: Arc::new(CircuitParams::default()),
            circuit_states: vec![CircuitState::default(); 2],
        }}
    }}
}}

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

    fn initialize(
        &mut self,
        audio_io_layout: &AudioIOLayout,
        _buffer_config: &BufferConfig,
        _context: &mut impl InitContext<Self>,
    ) -> bool {{
        let num_channels = audio_io_layout.main_input_channels
            .map(|c| c.get() as usize).unwrap_or(2);
        self.circuit_states = (0..num_channels).map(|_| CircuitState::default()).collect();
        true
    }}

    fn reset(&mut self) {{
        for state in &mut self.circuit_states {{
            *state = CircuitState::default();
        }}
    }}

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
