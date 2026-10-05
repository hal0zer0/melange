//! The shipped build for melange-validate's tests.

#![allow(dead_code)]

use melange_solver::build::{BuildOptions, Built};

/// `BuildOptions` at the production defaults for a deck driven at `input` and
/// read at `outputs`: the auto-tuned Newton budget, the deck's
/// `.input_impedance` / `.oversampling`, auto routing.
pub fn options(sample_rate: f64, input: &str, outputs: &[&str]) -> BuildOptions {
    let d = melange_solver::codegen::CodegenConfig::default();
    BuildOptions {
        sample_rate,
        circuit_name: "validate_test".to_string(),
        input_nodes: vec![input.to_string()],
        output_nodes: outputs.iter().map(|s| s.to_string()).collect(),
        max_iter: None,
        tolerance: d.tolerance,
        output_scale: 1.0,
        output_clamp: d.output_clamp_v,
        input_resistance: None,
        oversampling: None,
        oversampling_set: melange_solver::build::OversamplingSet::Off,
        dc_block: d.dc_block,
        solver: "auto".to_string(),
        backward_euler: false,
        force_trap: false,
        tube_grid_fa: "auto".to_string(),
        subsample_fire: d.subsample_fire,
        subsample_lit_factor: d.subsample_lit_factor,
        bjt_fa_mode: d.bjt_fa_mode,
        opamp_rail_mode: d.opamp_rail_mode,
        nodal_sub_path_override: d.nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode: d.noise_mode,
        noise_seed: d.noise_master_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        pot_overrides: None,
        resolve_taps: true,
        inject_runtime: true,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op: false,
        dc_op_max_iterations: None,
        output_clamp_auto: false,
    }
}

/// The shipped build of `spice` for `opts`; panics on a refusal.
pub fn build(spice: &str, opts: &BuildOptions) -> Built {
    let silent = &melange_solver::pipeline::silent;
    melange_solver::build::build(spice, opts, silent, silent)
        .unwrap_or_else(|e| panic!("build failed: {e:#}"))
}
