# melange — prebuilt CLI, Linux x86_64

This archive is the `melange` command-line tool: the compiler that turns a
SPICE circuit netlist into real-time audio DSP code, and with `--format plugin`
into a plugin project you build yourself. It contains the binary and the
license, nothing else. The source, the documentation and the examples for this
exact version are in the `Source code` asset on the same release page, and the
current ones are at <https://github.com/hal0zer0/melange>.

```bash
./melange --version            # prints the version and the release commit
./melange builtins             # circuits that ship inside the binary
./melange analyze passive-eq1a # frequency response of the built-in demo circuit
./melange compile passive-eq1a --format plugin -o ~/my-eq   # a plugin project
cd ~/my-eq && bash build.sh    # CLAP + VST3 land in target/bundled/
```

`passive-eq1a` is the demo circuit built into the binary: a passive program
equaliser in the classic 1950s tube topology. Point `melange compile` at any
`.cir` file of your own the same way.

## What it needs on your machine

- `melange compile` is self-contained: it writes Rust source.
- `melange simulate` and `melange analyze` need a Rust toolchain (`rustc` and
  `cargo`, Rust 1.85 or newer, <https://rustup.rs>) on your PATH: they compile
  the circuit they run.
- `melange validate` needs `ngspice` on your PATH.
- Building a generated `--format plugin` project needs the Rust toolchain and,
  the first time, network access (it fetches the nih-plug framework).

## Where to read next

The documentation ships with the source, not in this archive: open the
`Source code` asset from this release (or the repository) and read
`docs/GETTING_STARTED.md`, then `docs/NETLIST_GUIDE.md` for the netlist
dialect. The README there says what melange is and is not; the `CHANGELOG.md`
section for this version says what changed for you.

Melange is 0.x software: the CLI flags, the netlist rules and the generated
code may change between releases. A file melange has already generated does
not depend on melange and does not change with it.

## License

GPL-3.0; see `LICENSE`. Generated code is derived from GPL-licensed templates;
the repository's README covers what that means for your plugin.
