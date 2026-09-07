//! Runtime build identity: an EXACT hash of the running executable.
//!
//! A crate version and a git commit are a *source*-side stamp — a prediction of
//! what went into the binary, made at build time. They cannot observe a dirty
//! working tree (the build script does not re-run on every source edit) or a
//! different feature/profile build of the same source. Two binaries built from
//! HEAD `7b39da7` — one clean, one with three uncommitted solver edits — both
//! printed `melange 0.1.6 (7b39da7)`, and a peer published tables from each with
//! nothing in the output distinguishing them (robogogo thread 288, arbiter
//! ruling). The FNV-1a-64 exe hashes told them apart: `8b62d950c3c6f88b` (dirty)
//! vs `55c248b088a69bb6` (clean).
//!
//! The fix per that ruling: hash the ARTIFACT at runtime. It is exact by
//! construction (computed from the binary, not predicted from the source) and it
//! also separates one source built with different features or profiles, which a
//! commit never can. The version+commit(+dirty) remain, but as a labelled
//! best-effort POINTER to where to look, not the identity.
//!
//! Algorithm is FNV-1a 64 over the whole executable file, printed as 16 lowercase
//! hex — matching the digest oomox's rulers emit (`instrument_exe`) and the one
//! openphilicorda computes over a binary on disk, so a binary's self-reported
//! hash equals the hash a peer computes over its file.

use std::sync::OnceLock;

const FNV_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// FNV-1a 64 over `bytes`.
fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash = FNV_OFFSET_BASIS;
    for &byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

/// 16-hex FNV-1a-64 digest of the running executable's bytes, or `None` when the
/// executable path or its bytes are unavailable (e.g. the binary was removed or
/// is unreadable). Computed once per process and cached — subsequent calls (the
/// CLI `--version` path and every codegen provenance stamp share it) are free.
pub fn current_exe_hash() -> Option<&'static str> {
    static HASH: OnceLock<Option<String>> = OnceLock::new();
    HASH.get_or_init(|| {
        let path = std::env::current_exe().ok()?;
        let bytes = std::fs::read(path).ok()?;
        Some(format!("{:016x}", fnv1a64(&bytes)))
    })
    .as_deref()
}

/// The exe hash, or the literal `"unknown"` when it cannot be computed — for
/// contexts (provenance JSON, `--version`) that always emit a value.
pub fn current_exe_hash_or_unknown() -> &'static str {
    current_exe_hash().unwrap_or("unknown")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a64_matches_known_vectors() {
        // Canonical FNV-1a 64 test vectors.
        assert_eq!(fnv1a64(b""), FNV_OFFSET_BASIS);
        assert_eq!(fnv1a64(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(fnv1a64(b"foobar"), 0x8594_4171_f739_67e8);
    }

    #[test]
    fn exe_hash_is_16_hex_and_stable() {
        // The test binary is a real executable, so this resolves; and it is
        // cached, so two calls agree.
        let a = current_exe_hash().expect("test binary is readable");
        let b = current_exe_hash().expect("cached");
        assert_eq!(a, b);
        assert_eq!(a.len(), 16);
        assert!(a.chars().all(|c| c.is_ascii_hexdigit()));
    }
}
