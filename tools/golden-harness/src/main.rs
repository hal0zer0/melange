//! golden-harness — golden-audio regression gate for melange codegen.
//!
//! Captures deterministic rendered audio of shipped circuits (via the
//! installed `melange` CLI, the same compilation path oomox uses) and diffs
//! captures against each other so any compiler change that alters
//! plugin-observable output is caught, classified, and loud.
//!
//! Subcommands:
//!   capture --manifest <json> --out <dir> [--fs 48000] [--timeout 600] [--keep-work]
//!   compare <dirA> <dirB> [--json <path>]
//!
//! Execution mechanism is ported from melange-validate's
//! `run_melange_codegen_with_main` (crates/melange-validate/tests/
//! spice_validation.rs): generated circuit module + appended `fn main()`
//! driver, compiled with `rustc --edition=2024 -O`, samples piped through
//! stdin/stdout with a writer thread to avoid pipe deadlock.

mod capture;
mod compare;
mod manifest;
mod programs;
mod runner;
mod stats;

use std::path::PathBuf;
use std::process::ExitCode;

const USAGE: &str = "golden-harness — golden-audio regression harness for melange

USAGE:
  golden-harness capture --manifest <manifest.json> --out <baseline-dir>
                         [--fs <hz>] [--timeout <secs>] [--keep-work] [--dry-run]
  golden-harness compare <baseline-A> <baseline-B> [--json <report.json>] [--strict]

COMPARE MODES:
  default   release gate — passes IDENTICAL and NEGLIGIBLE. Asks: did anything
            audibly change?
  --strict  refactor gate — passes ONLY IDENTICAL, and additionally requires
            every generated circuit.rs to match. Asks: did anything change at
            all? Use this for any change that is supposed to be behaviour-
            preserving; NEGLIGIBLE is precisely the band a refactor bug hides in.

EXIT CODES:
  capture: 0 = all circuits captured, 3 = some circuit failed (run completed)
  compare: 0 = gate passed, 1 = gate failed, 2 = usage error
";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(|s| s.as_str()) {
        Some("capture") => {
            let mut manifest: Option<PathBuf> = None;
            let mut out: Option<PathBuf> = None;
            let mut fs = 48000.0_f64;
            let mut timeout = 600u64;
            let mut keep_work = false;
            let mut dry_run = false;
            let mut i = 1;
            while i < args.len() {
                match args[i].as_str() {
                    "--manifest" => {
                        i += 1;
                        manifest = args.get(i).map(PathBuf::from);
                    }
                    "--out" => {
                        i += 1;
                        out = args.get(i).map(PathBuf::from);
                    }
                    "--fs" => {
                        i += 1;
                        fs = match args.get(i).and_then(|s| s.parse().ok()) {
                            Some(v) => v,
                            None => return usage_err("--fs needs a numeric value"),
                        };
                    }
                    "--timeout" => {
                        i += 1;
                        timeout = match args.get(i).and_then(|s| s.parse().ok()) {
                            Some(v) => v,
                            None => return usage_err("--timeout needs a numeric value"),
                        };
                    }
                    "--keep-work" => keep_work = true,
                    "--dry-run" => dry_run = true,
                    other => return usage_err(&format!("unknown capture flag: {other}")),
                }
                i += 1;
            }
            if dry_run {
                let Some(manifest) = manifest else {
                    return usage_err("capture --dry-run requires --manifest");
                };
                return match capture::dry_run(&manifest) {
                    Ok(bad) => {
                        if bad == 0 {
                            ExitCode::SUCCESS
                        } else {
                            ExitCode::from(3)
                        }
                    }
                    Err(e) => {
                        eprintln!("dry-run error: {e}");
                        ExitCode::from(2)
                    }
                };
            }
            let (Some(manifest), Some(out)) = (manifest, out) else {
                return usage_err("capture requires --manifest and --out");
            };
            match capture::run(&manifest, &out, fs, timeout, keep_work) {
                Ok(failed) => {
                    if failed == 0 {
                        ExitCode::SUCCESS
                    } else {
                        ExitCode::from(3)
                    }
                }
                Err(e) => {
                    eprintln!("capture error: {e}");
                    ExitCode::from(2)
                }
            }
        }
        Some("compare") => {
            let mut positional: Vec<PathBuf> = Vec::new();
            let mut json_out: Option<PathBuf> = None;
            let mut strict = false;
            let mut i = 1;
            while i < args.len() {
                match args[i].as_str() {
                    "--json" => {
                        i += 1;
                        json_out = args.get(i).map(PathBuf::from);
                    }
                    "--strict" => strict = true,
                    other => positional.push(PathBuf::from(other)),
                }
                i += 1;
            }
            if positional.len() != 2 {
                return usage_err("compare requires exactly two baseline directories");
            }
            let json_out = json_out.unwrap_or_else(|| PathBuf::from("golden-compare-report.json"));
            match compare::run(&positional[0], &positional[1], &json_out, strict) {
                Ok(changed) => {
                    if changed == 0 {
                        ExitCode::SUCCESS
                    } else {
                        ExitCode::from(1)
                    }
                }
                Err(e) => {
                    eprintln!("compare error: {e}");
                    ExitCode::from(2)
                }
            }
        }
        _ => {
            eprintln!("{USAGE}");
            ExitCode::from(2)
        }
    }
}

fn usage_err(msg: &str) -> ExitCode {
    eprintln!("error: {msg}\n\n{USAGE}");
    ExitCode::from(2)
}
