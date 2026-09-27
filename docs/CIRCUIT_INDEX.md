# `circuits-index.json` — the circuit index format

A circuit repository publishes one file at its root so that tools can resolve a
**short name** to a **path**, without knowing how the repository is organised.

```
melange compile myrepo:big-muff      # not myrepo:fuzz/germanium/big-muff.cir
```

This is melange's format: melange is the consumer, so the definition lives here.
melange-circuits' `tools/circuits_index.py` is a reference implementation, and
`melange index` (below) is another. Neither is the definition.

## Why not just use paths

Because a path contains the thing most likely to change. Circuit repositories
sort decks into directories that encode *status* — `unstable/`, `testing/`,
`stable/` — and a deck's status is supposed to change as it gets tested. A tool
that hard-codes `unstable/preamp/foo.cir` breaks the day `foo` is promoted, and
it breaks with a message about a missing file rather than about a promotion.

That is not hypothetical. On 2026-09-27 a single afternoon of tier promotions in
melange-circuits broke melange's golden-baseline manifest twice and openwurli's
netlist-sync test once. An index makes the same promotion invisible to every
consumer.

## The file

Location: `circuits-index.json` at the repository root, so it is served at
`<base-url>/circuits-index.json` alongside the circuits.

```json
{
  "schema": 1,
  "circuits": {
    "big-muff": { "path": "fuzz/big-muff.cir" },
    "rc":       { "path": "rc.cir" }
  }
}
```

**Required**

| Field | Type | Meaning |
|---|---|---|
| `schema` | integer | Format version. Currently `1`. |
| `circuits` | object | Map of circuit **name** to entry. |
| `circuits[name].path` | string | Path to the `.cir`, **relative to the index file**. |

**Names** are the `.cir` basename without the extension, and **must be unique**
across the repository. A generator that finds a collision must fail rather than
pick a winner.

**Optional extras** are allowed and readers **must ignore keys they do not
recognise** — that is what lets the format grow without breaking older clients.
melange-circuits publishes `tier` and `category`; melange reads neither.

```json
"passive-eq1a": {
  "path": "testing/filters/passive-eq1a.cir",
  "tier": "testing",
  "category": "filters"
}
```

## What the file deliberately does NOT contain

**No commit hash and no timestamp.** They look like provenance and are a trap: a
self-describing hash is wrong the moment the file is written, so a `--check` job
would fail on every rebuild, and the only way to keep it green is to regenerate
on every commit whether or not any circuit moved.

The file describes *the tree it sits in*. Its version is the ref you fetched it
from. (Credit where due: melange-circuits caught this in review of an earlier
draft that had both fields.)

**No unpublished circuits.** Generate the index from the tree you actually
publish. Generated from a private tree and copied, it leaks the names of
unpublished work into a public file.

## How a consumer must resolve

1. `GET <base>/circuits-index.json`.
2. **Present** → look up the name; the entry's `path` is joined to `<base>`.
3. **Absent (404)** → fall back to a flat `<base>/<name>.cir`. Repositories
   without an index keep working.
4. **Present, name missing** → **error**, and do not try step 3.

Step 4 matters. An indexed repository has *declared* its contents; guessing past
that declaration costs a request and then reports the wrong problem — "not found
at `.../passiveq1a.cir`" when the truth is "that name is not in this source, did
you mean `passive-eq1a`". melange lists near-matches instead.

Consumers that cache the index should re-fetch it once when a circuit path
404s, then retry: that is a deck that moved after the index was cached, and it
self-heals in one extra request instead of requiring a manual cache clear.

## Publishing one

```bash
melange index .                  # write ./circuits-index.json
melange index . --check          # exit non-zero if missing, stale or ambiguous
```

Put `--check` in CI. It is the part that matters: an index that silently stops
matching the tree is worse than no index, because consumers trust it.

**Generators** should emit a canonical byte format so two of them agree on the
same tree: circuit names sorted, two-space indent, trailing newline, `schema`
before `circuits`, `path` first within an entry.

**`--check` compares meaning, not bytes** — the schema version and the
name→path mapping — and ignores keys it does not recognise. It has to: the rule
above says readers ignore unknown keys, so a repository that enriches its
entries is conforming, and a byte comparison would hold that against it and go
permanently red. `melange index --check` accepts melange-circuits' published
index unchanged, `tier` and `category` included, while melange itself emits
neither.

(An earlier draft of this document said `--check` byte-compares. That was
wrong, and it contradicted the paragraph above it; found by running melange's
generator against the real published file.)

## Serving it

Any static host works — the consumer only does `GET`. For a GitLab or GitHub
repository the raw base URL is enough:

```bash
melange sources add myrepo https://gitlab.com/you/yourrepo/-/raw/main
melange compile myrepo:big-muff --format plugin -o my-plugin
```
