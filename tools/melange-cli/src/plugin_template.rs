//! Plugin project template generation

use std::path::Path;
use anyhow::Result;

/// Generate a complete plugin project
pub fn generate_plugin_project(
    output_dir: &Path,
    circuit_code: &str,
    circuit_name: &str,
) -> Result<()> {
    // Create directory structure
    std::fs::create_dir_all(output_dir.join("src"))?;
    
    // Write Cargo.toml
    let cargo_toml = generate_cargo_toml(circuit_name);
    std::fs::write(output_dir.join("Cargo.toml"), cargo_toml)?;
    
    // Write circuit.rs (the generated code)
    std::fs::write(output_dir.join("src/circuit.rs"), circuit_code)?;
    
    // Write lib.rs (plugin wrapper)
    let lib_rs = generate_lib_rs(circuit_name);
    std::fs::write(output_dir.join("src/lib.rs"), lib_rs)?;
    
    Ok(())
}

fn generate_cargo_toml(circuit_name: &str) -> String {
    format!(r#"[package]
name = "{}"
version = "0.1.0"
edition = "2021"

[dependencies]
nih_plug = {{ git = "https://github.com/robbert-vdh/nih-plug.git" }}

[lib]
crate-type = ["cdylib"]
"#, circuit_name)
}

fn generate_lib_rs(circuit_name: &str) -> String {
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
    
    // Create unique CLAP ID and VST3 ID based on circuit name
    let clap_id = format!("com.melange.{}", circuit_name);
    // VST3 ID must be exactly 16 bytes
    let vst3_base = circuit_name.replace("-", "").to_uppercase();
    let vst3_id = format!("{: <16}", &vst3_base[..vst3_base.len().min(16)]);
    let vst3_id_str = vst3_id;
    
    format!(r#"use nih_plug::prelude::*;
use std::sync::Arc;

mod circuit;
use circuit::{{process_sample, CircuitState}};

#[derive(Default)]
pub struct CircuitPlugin {{
    params: Arc<CircuitParams>,
    circuit_state: CircuitState,
}}

#[derive(Params)]
pub struct CircuitParams {{
    #[id = "gain"]
    pub gain: FloatParam,
}}

impl Default for CircuitParams {{
    fn default() -> Self {{
        Self {{
            gain: FloatParam::new(
                "Gain",
                0.5,
                FloatRange::Linear {{ min: 0.0, max: 1.0 }},
            ),
        }}
    }}
}}

impl Plugin for CircuitPlugin {{
    const NAME: &'static str = "{}";
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
        _audio_io_layout: &AudioIOLayout,
        _buffer_config: &BufferConfig,
        _context: &mut impl InitContext<Self>,
    ) -> bool {{
        self.circuit_state = CircuitState::default();
        true
    }}
    
    fn reset(&mut self) {{
        self.circuit_state = CircuitState::default();
    }}
    
    fn process(
        &mut self,
        buffer: &mut Buffer,
        _aux: &mut AuxiliaryBuffers,
        _context: &mut impl ProcessContext<Self>,
    ) -> ProcessStatus {{
        for channel_samples in buffer.iter_samples() {{
            for sample in channel_samples {{
                let out = process_sample(*sample as f64, &mut self.circuit_state) as f32;
                // Safety: clamp output and replace NaN/inf with silence
                *sample = if out.is_finite() {{ out.clamp(-10.0, 10.0) }} else {{ 0.0 }};
            }}
        }}
        ProcessStatus::Normal
    }}
}}

impl ClapPlugin for CircuitPlugin {{
    const CLAP_ID: &'static str = "{}";
    const CLAP_DESCRIPTION: Option<&'static str> = Some("Circuit modeled with Melange");
    const CLAP_MANUAL_URL: Option<&'static str> = Some(Self::URL);
    const CLAP_SUPPORT_URL: Option<&'static str> = None;
    
    const CLAP_FEATURES: &'static [ClapFeature] = &[
        ClapFeature::AudioEffect,
        ClapFeature::Stereo,
    ];
}}

impl Vst3Plugin for CircuitPlugin {{
    const VST3_CLASS_ID: [u8; 16] = *b"{}";
    
    const VST3_SUBCATEGORIES: &'static [Vst3SubCategory] = &[
        Vst3SubCategory::Fx,
    ];
}}

nih_export_clap!(CircuitPlugin);
nih_export_vst3!(CircuitPlugin);
"#, display_name, clap_id, vst3_id_str)
}
