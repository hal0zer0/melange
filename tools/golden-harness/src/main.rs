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
//!   convergence <baseline-dir>...
//!
//! Execution mechanism is ported from melange-validate's
//! `run_melange_codegen_with_main` (crates/melange-validate/tests/
//! spice_validation.rs): generated circuit module + appended `fn main()`
//! driver, compiled with `rustc --edition=2024 -O`, samples piped through
//! stdin/stdout with a writer thread to avoid pipe deadlock.

mod capture;
mod compare;
mod convergence;
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
  golden-harness convergence <baseline-dir> [<baseline-dir> ...]

COMPARE MODES:
  default   release gate — passes IDENTICAL and NEGLIGIBLE. Asks: did anything
            audibly change?
  --strict  refactor gate — passes ONLY IDENTICAL, and additionally requires
            every generated circuit.rs to match. Asks: did anything change at
            all? Use this for any change that is supposed to be behaviour-
            preserving; NEGLIGIBLE is precisely the band a refactor bug hides in.

CONVERGENCE:
  Every capture and compare prints a CONVERGENCE HEALTH section: the fraction
  of internal samples on which Newton-Raphson hit its iteration ceiling and
  emitted a capped iterate instead of a solution. A capped iterate is bounded
  and smooth, so the audio metrics cannot see it. The section is REPORT-ONLY
  and never changes an exit code; `convergence <dir>...` prints it on its own.

NEWTON HOLD (all subcommands):
  A render is HELD when every Newton path failed on a sample and the previous
  state was committed as that sample's output. It is not a solution, and it is
  bounded and smooth, so no level measurand can see it. Under constant input the
  hold is a fixed point, so one hard sample can freeze a render to its end.
  HELD fails every gate. Capped-but-recovered samples do NOT: those are a
  converged solution by another consistent scheme, and stay report-only.
  --allow-nr-hold  proceed anyway. Per-invocation only, never a deck property:
                   a netlist cannot know whether it will trip (one corpus deck
                   is clean at 0.035 V and frozen at 0.04). A capture made under
                   it writes an ALLOW_NR_HOLD marker into the baseline.

EXIT CODES:
  capture: 0 = all circuits captured, 3 = some circuit failed (run completed)
  compare: 0 = gate passed, 1 = gate failed, 2 = usage error
  convergence: 0 = no render held, 2 = usage error
  any:     4 = a render shipped samples that are not solutions (see above)
";

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // Per-invocation ONLY, never a deck declaration: a netlist cannot know
    // whether it will trip the hold — steve-1073-preamp is clean at 0.035 V and
    // frozen at 0.04 — so a deck-level opt-out would be a claim about inputs its
    // author never ran (arbiter t536). Stripped here so each subcommand's own
    // parser does not have to know about it.
    let allow_nr_hold = args.iter().any(|a| a == "--allow-nr-hold");
    args.retain(|a| a != "--allow-nr-hold");
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
            match capture::run(&manifest, &out, fs, timeout, keep_work, allow_nr_hold) {
                Ok(capture::Outcome { failed, held }) => {
                    if failed != 0 {
                        ExitCode::from(3)
                    } else if held != 0 && !allow_nr_hold {
                        eprintln!(
                            "FAIL: {held} captured render(s) contain samples that are not \
                             solutions (death-spiral hold). The baseline was still written \
                             so the evidence exists; re-run with --allow-nr-hold to accept it."
                        );
                        ExitCode::from(4)
                    } else {
                        if held != 0 {
                            eprintln!(
                                "warning: {held} captured render(s) shipped non-solutions; \
                                 --allow-nr-hold suppressed the failure and the baseline \
                                 records it"
                            );
                        }
                        ExitCode::SUCCESS
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
                Ok(compare::Outcome { failures, held }) => {
                    if held != 0 && !allow_nr_hold {
                        eprintln!(
                            "FAIL: {held} render(s) across these baselines contain samples \
                             that are not solutions (death-spiral hold). Comparing against a \
                             frozen-circuit golden does not measure the change. Re-run with \
                             --allow-nr-hold to proceed anyway."
                        );
                        ExitCode::from(4)
                    } else if failures == 0 {
                        if held != 0 {
                            eprintln!(
                                "warning: {held} render(s) shipped non-solutions; \
                                 --allow-nr-hold suppressed the failure"
                            );
                        }
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
        Some("convergence") => {
            let dirs: Vec<PathBuf> = args[1..].iter().map(PathBuf::from).collect();
            if dirs.is_empty() {
                return usage_err("convergence requires at least one baseline directory");
            }
            match convergence::run(&dirs) {
                Ok(0) => ExitCode::SUCCESS,
                Ok(held) => {
                    if allow_nr_hold {
                        eprintln!(
                            "warning: {held} render(s) shipped non-solutions; \
                             --allow-nr-hold suppressed the failure"
                        );
                        ExitCode::SUCCESS
                    } else {
                        eprintln!(
                            "FAIL: {held} render(s) contain samples that are not solutions \
                             (death-spiral hold). Re-run with --allow-nr-hold to proceed \
                             anyway; the bypass is recorded."
                        );
                        ExitCode::from(4)
                    }
                }
                Err(e) => {
                    eprintln!("convergence error: {e}");
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
