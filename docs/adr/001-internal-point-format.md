# ADR 001: Internal Point Format (v0.3.0)

**Status:** Proposed  
**Date:** 2026-06-18  
**Deciders:** Mark Bouck, AI DevOps review  

## Context

rubipont currently uses a flat 26-byte internal format: 3×f64 (XYZ, 24 bytes) + u16 (intensity, 2 bytes). This format is:
- Defined in 4 separate modules as `INTERNAL_POINT_SIZE` with slightly different names and doc comments
- Hardcoded as a literal in 2 more modules
- **Incapable of carrying LAS extended fields** (RGB, GPS time, NIR, classification, return number) — these are silently discarded on every read
- **Incapable of carrying E57 colour** or other format-specific metadata
- **Incapable of representing points without intensity** — the E57 reader fabricates `0u16` when intensity is absent, producing spurious zero-intensity output that downstream formats interpret as real data (silent corruption)
- Misleadingly reflected in `layout.point_size`, where some readers reported the source format's on-disk record size while chunks were always 26-byte (fixed in 0.1.2 → 0.1.3)

The fundamental tension: rubipont markets itself as "lossless format translation" but the internal format cannot represent what LAS format 2 or E57 actually contain. Every conversion is potentially lossy, silently.

## Decision

**Replace the 26-byte flat format with a core-fields + named-extras schema.**

```rust
pub struct PointBatch {
    /// Number of points in this batch.
    len: usize,

    // --- Core fields (always present, always direct-access) ---

    /// X coordinates (f64).
    x: Vec<f64>,
    /// Y coordinates (f64).
    y: Vec<f64>,
    /// Z coordinates (f64).
    z: Vec<f64>,

    // --- Named optional fields ---

    /// Fields beyond XYZ.  See `PointField` for the type registry.
    /// The key carries the semantic name (e.g., "intensity", "gps_time",
    /// "rgb", "classification").  Readers populate what they have; writers
    /// consume what they understand.
    extras: BTreeMap<String, PointField>,
}

/// Type-level description of a point field carried in `PointBatch.extras`.
/// Variants are purely type-based — the `BTreeMap` key supplies the semantic
/// name.  No LAS-specific names appear here.
pub enum PointField {
    /// 64-bit floating-point array (XYZ coords are never here; core handles those).
    F64(Vec<f64>),
    /// 16-bit unsigned integer array (e.g., intensity, scan angle rank).
    U16(Vec<u16>),
    /// 8-bit unsigned integer array (e.g., classification, return number).
    U8(Vec<u8>),
    /// RGB colour triplets, one per point.
    Rgb(Vec<[u8; 3]>),
    /// Raw byte blob for unknown/future formats.  Callers interpret as needed.
    Raw(Vec<u8>),
}
```

### Why This Shape

1. **Core fast path is untouched.** XYZ is `Vec<f64>` — zero allocations, zero HashMap lookups, zero dynamic dispatch. 99% of conversions only touch XYZ + optionally intensity. Performance parity with the 26-byte flat format.

2. **No silent data loss.** A LAS format 2 file produces `extras["rgb"] = Rgb(...)`. The PCD writer doesn't understand RGB, so it sidecars it. The LAS writer *does* understand RGB and embeds it. The user is told what didn't make it (`--warn-on-loss`). Nothing is discarded without notification.

3. **Formats are additive, not disruptive.** Adding LAS format 6 support means teaching the LAS reader to emit `extras["nir"] = U16(...)`. No existing writer needs to change. No existing test breaks.

4. **No LAS bias in the type system.** `GpsTime`, `Classification`, `ReturnNumber` are not in the enum. Those are LAS semantic names; they map to plain `F64`, `U8`, `U8` keys in the map. An E57 reader can emit `extras["colour_r"] = U16(...)` with E57-native naming without waiting for the type system to cover it.

### What Does NOT Go In Core

Only **XYZ** is core. Intensity, classification, RGB, GPS time, and everything else lives in `extras` as `Option<Vec<T>>` semantics — present when the source has it, absent otherwise.

**Rationale for excluding intensity from core:**

- E57 files commonly have no intensity — the current reader fabricates `0u16`, which is indistinguishable from real zero-intensity output. Downstream formats treat those zeros as real data. Silence should mean "not measured," not "value zero."
- PCD ASCII files may or may not have an intensity column — the current parser silently reads `0.0` from thin air via `vals.get(3).copied().unwrap_or(0.0) as u16`.
- The core+extras schema makes the distinction explicit: `extras.contains_key("intensity")` is the truth signal. Absence means "no data." Zero means "measured zero."

## Sidecar Discovery Rules

When a writer cannot embed a field, it writes a `.meta.json` sidecar alongside the output file listing the unwritten fields. The reader side of the pipeline must answer: *does it look for the sidecar?*

### Rule: Automatic same-directory lookup with explicit override

1. **Opening a file for reading (`rp info`, `rp convert`):** The reader looks for `<filename>.meta.json` in the same directory. If found, the sidecar's unwritten-fields manifest is attached to the reader's metadata. No additional user flags needed.

2. **Sidecar was moved or renamed:** The sidecar is keyed by filename (e.g., `output.pcd.meta.json` pairs with `output.pcd`). If the user renames the file but not the sidecar, the link breaks silently. This is documented behaviour — the sidecar is a convenience aid, not a cryptographic signature. Users are advised to rename both together.

3. **Explicit sidecar path:** `rp convert input.pcd output.las --sidecar path/to/manual.json` overrides automatic lookup and uses the specified sidecar instead. This covers moved files and programmatic workflows.

4. **No sidecar, no error:** Missing sidecar is a warning, not an error. The conversion proceeds with only the embedded fields.

### What `rp info` Shows

When a file has an associated sidecar:

```
$ rp info scan.pcd
File: scan.pcd
Points: 1,847,234
Point stride: varies (core+extras format)
Fields (embedded):
  x (f64), y (f64), z (f64), intensity (u16)
Fields (sidecar: scan.pcd.meta.json):
  rgb (u8×3), gps_time (f64), classification (u8)
Source: LAS 1.4 format 2
CRS: WGS 84 (EPSG:4326)
```

Without a sidecar:

```
$ rp info scan.pcd
File: scan.pcd
Points: 1,847,234
Fields (embedded):
  x (f64), y (f64), z (f64), intensity (u16)
CRS: WGS 84 (EPSG:4326)
```

`rp formats` gains a column showing which extras each format can embed:

```
$ rp formats
Format      Read  Write  Embeddable extras
.las 1.2    +     +      xyz, intensity
.las 1.4    +     +      xyz, intensity, rgb, gps_time, classification, nir, user_data
.laz        +     +      (same as LAS 1.4)
.pcd        +     +      xyz, intensity
.e57        +     +      xyz, intensity, rgb, gps_time, row_index, column_index
.mcap PC2   +     +      (all fields via PointField descriptor — lossless by design)
.bag PC2    +     -      (all fields via PointField descriptor)
```

## Consequences

### Positive

- **Honest about capability.** Users know before converting what will survive.
- **Extensible.** Adding a field to a format is a local change to one reader and optionally one writer.
- **Backward-compatible with awareness.** Old flat-format chunks can be read into the new `PointBatch` by splitting the 26-byte blob into core x/y/z and `extras["intensity"]`. Migration is mechanical.

### Negative

- **Breaking change for Rust API consumers.** `PointChunk` is currently public. The new `PointBatch` replaces it. Downstream code that reads `chunk.data` directly breaks.
- **Performance regression for the intensity fast path.** Currently, intensity is always at offset 24 in the flat buffer — one pointer add, zero branches. With extras, it's a HashMap lookup. Mitigation: the core fast path (XYZ-only reads) incurs no lookup. Intensity access is a single `BTreeMap::get` — negligible for multi-million-point batches.
- **Memory overhead.** `BTreeMap` + `Vec` headers add a few hundred bytes per batch. Measurable but trivial (batches are 4096 points × N bytes).

### Non-Decision

This ADR does not specify the exact migration path from 26-byte `PointChunk` to `PointBatch`. That is deferred to an implementation ADR. The principle is: v0.3.0 introduces `PointBatch` alongside `PointChunk` (deprecated), v0.4.0 removes `PointChunk`.

### References

- `rubipont-core/src/format/e57.rs:68` — E57ReaderImpl already sets `point_size: 26` with a comment (the reference implementation for the flat format)
- `rubipont-core/src/pipeline.rs:79,103-116` — the reprojection loop that strides by `layout.point_size`; the stride bug (fixed in parent commit) was caused by mismatch between this loop and the readers' reported point_size
- Review A §2.3, §2.8 — silent LAS extended field loss
- Review B §F6 — no LAS formats 1–10 write support
