use anyhow::Result;
use std::path::Path;

/// Parse the `--bjt-fa` string into a [`melange_solver::codegen::BjtFaMode`].
/// Assumes the value was already validated (`auto` | `off` | `force`); an
/// unrecognized value falls back to `Off`, the default.
pub(crate) fn parse_bjt_fa_mode(s: &str) -> melange_solver::codegen::BjtFaMode {
    match s {
        "off" => melange_solver::codegen::BjtFaMode::Off,
        "force" => melange_solver::codegen::BjtFaMode::Force,
        "auto" => melange_solver::codegen::BjtFaMode::Auto,
        _ => melange_solver::codegen::BjtFaMode::Off,
    }
}

/// Parse `--subsample-fire {auto|on|off}`. Unknown values are user errors.
pub(crate) fn parse_subsample_fire_mode(
    s: &str,
) -> Result<melange_solver::codegen::SubsampleFireMode> {
    melange_solver::codegen::SubsampleFireMode::parse(s).ok_or_else(|| {
        anyhow::anyhow!(
            "Unknown --subsample-fire '{}'. Valid values: auto, on, off",
            s
        )
    })
}

/// The rate `simulate` builds the circuit at, and where it came from.
///
/// The build (solver route, integrator verdict, oversampling) is taken at one
/// rate and the rendering binary runs at the input WAV's own rate, which
/// `set_sample_rate` does not re-route. So with an input WAV the two must be
/// the same rate: absent `--sample-rate` the WAV's rate is used, and an
/// explicit `--sample-rate` that differs is refused rather than rendered at a
/// rate the build was not made for. Without a WAV: the flag, else 48 kHz.
pub(crate) fn resolve_simulate_sample_rate(
    flag: Option<f64>,
    wav_rate: Option<u32>,
) -> Result<(f64, &'static str)> {
    let (rate, source) = match (flag, wav_rate) {
        (Some(sr), Some(wav)) if sr != wav as f64 => anyhow::bail!(
            "--sample-rate {sr} Hz does not match the --input-audio WAV's {wav} Hz. The \
             circuit is built for one rate (solver route, integrator, oversampling) and the \
             WAV would be rendered at the other. Omit --sample-rate to build at the WAV's \
             {wav} Hz, or resample the WAV to {sr} Hz."
        ),
        (Some(sr), _) => (sr, "--sample-rate"),
        (None, Some(wav)) => (wav as f64, "from the --input-audio WAV"),
        (None, None) => (48_000.0, "default"),
    };
    if rate <= 0.0 || !rate.is_finite() {
        anyhow::bail!("sample rate must be positive and finite, got {rate}");
    }
    Ok((rate, source))
}

/// A path as the UTF-8 argument the generated simulate binary reads
/// (`std::env::args`). A path that is not UTF-8 is refused, naming the flag:
/// substituting a default name (`output.wav`) wrote somewhere the user never
/// asked for.
pub(crate) fn utf8_path_arg<'p>(path: &'p Path, flag: &str) -> Result<&'p str> {
    path.to_str().ok_or_else(|| {
        anyhow::anyhow!(
            "{flag} path is not valid UTF-8: {}. simulate hands its paths to the rendering \
             binary as UTF-8 arguments; rename the file or pick a UTF-8 path.",
            path.display()
        )
    })
}
