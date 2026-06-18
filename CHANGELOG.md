# Changelog

All notable changes to rubipont are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.3] — 2026-06-18

### Fixed
- **pipeline**: `layout.point_size` is now consistently set to the internal 26‑byte stride in all readers.  LasReader, LazReader, and PcdReader were reporting the on‑disk record size (e.g. 20 B for LAS format 0) while producing chunks in the 26‑byte internal format, causing the CRS reprojection loop to stride by the wrong value and silently mangle all coordinates after the first point (E57ReaderImpl was already correct).
- **e57 reader**: O(n²) performance bug — every `read_chunk()` call re‑created the point cloud iterator and re‑skipped all previously‑read points.  A 10 M‑point file needed ~125 M read operations.  Now buffers eagerly during construction (one upfront read).
- **mcap reader**: `unsafe { memmap2::Mmap::map(&file) }` risked a SIGBUS crash if the file was truncated or modified while being read.  Replaced with `std::fs::read`.  Removed the now‑unused `memmap2` dependency.
- **mcap/bag reader**: Five PointCloud2 correctness fixes in the shared parser:
  - `point_step=0` now returns a `ParseError` instead of panicking on division‑by‑zero.
  - Missing `x`, `y`, or `z` field in the field descriptor array now returns a `ParseError` instead of silently producing `(0.0, 0.0, 0.0)` points.
  - When `is_dense == 0`, points with NaN coordinates are skipped per the PointCloud2 spec (previously passed through as corrupt output).
  - Big‑endian coordinate bytes (`is_bigendian == 1`) are now swapped to little‑endian on read; previously ignored, producing silent garbage for big‑endian ROS files.
  - Intensity `f32→u16` conversion now clamps to `[0.0, 1.0]` before scaling to prevent overflow wraparound for values > 1.0 (common in some sensors).
- **laz reader**: Extracts WKT CRS from the LAS 1.4 header (via `header.get_wkt_crs_bytes()`) instead of building `PipelineContext` with `..Default::default()`.  LAZ→LAS round‑trips now preserve the coordinate reference system.
- **laz writer**: Embeds `metadata.crs_wkt` into the output header via `header.set_wkt_crs()` before writing, matching the LAS writer.  Conversions to LAZ (e.g. E57→LAZ, LAS→LAZ) no longer silently drop CRS.
- **pcd writer**: `WIDTH` and `POINTS` header fields are now written during `finalize()` using the actual point count instead of the source file's declared count at construction time.  This prevents a corrupted header if the pipeline stops early.
- **pcd ASCII reader**: Non‑numeric tokens (`nan`, `inf`, corrupted text) now return a `ParseError` instead of being silently skipped by `filter_map`, which could cause column shifts and wrong values for all subsequent fields on the same line.  Also enforces a minimum of 3 fields per line.

### Added
- **shared PointCloud2 parser**: `format/pointcloud2.rs` consolidates the CDR‑encoded `sensor_msgs/PointCloud2` parser that was previously duplicated in `mcap.rs` and `bag.rs`.  Single code path for both MCAP (with 4‑byte CDR header) and ROS 1 bag (without).  Bug fixes now apply to both formats automatically.
- **ADR 001**: Architecture Decision Record for the v0.3.0 core‑extras internal point format (`docs/adr/001-internal-point-format.md`).

### Changed
- **constants**: `INTERNAL_POINT_SIZE` (26 B) is now defined once in `layout.rs` and imported by all format modules.  Removed six duplicate definitions.  No functional change — same value, one source of truth.
- **dependency**: Removed `memmap2 = "0.9"` direct dependency (mcap reader no longer uses memory‑mapped I/O).

[0.1.3]: https://github.com/mabo-du/rubipont/compare/v0.1.2...v0.1.3

## [0.1.2] — 2026-06-12
- **pipeline**: CRS reprojection loop now uses `layout.point_size` instead of hardcoded `26`, ensuring correctness if the internal point format changes. Added bounds check that rejects truncated chunks with a clear error instead of silently reading past the buffer.
- **laz writer**: Truncated chunk data now returns a `ParseError` instead of silently dropping points via `break`.
- **laz writer**: Rejects zero-valued coordinate scale components with a clear error, preventing silent garbage output from division-by-zero when converting f64 coordinates back to scaled integers.
- **mcap/bag reader**: `is_dense` byte is now actually read from the PointCloud2 data blob instead of dead code that always returned `1`.
- **pcd reader**: `read_field_as_f64` and `read_field_as_u16` now return `ParseError` for unrecognized field type/size combinations instead of silently returning `0.0` / `0`.
- **array utilities**: Added `debug_assert!` to `read_u32_unchecked` for defense-in-depth against misuse in debug builds.
- **transform**: Added `// SAFETY:` comment documenting the invariant for `get_unchecked` in EPSG code extraction.
- **cli**: `mcap-io` Cargo feature was never declared in `rubipont-cli/Cargo.toml`, making all `#[cfg(feature = "mcap-io")]` code in the CLI dead. Fixed by adding feature forwarding.

### Added
- **cli**: `rp info` now supports E57, MCAP, and BAG files (previously only LAS/LAZ/PCD).
- **cli**: `rp formats` now lists all five supported formats (previously omitted E57 and BAG).

## [0.1.1] — 2026-06-12

### Changed
- Multi-platform binary release builds (Linux, macOS, Windows).

## [0.1.0] — 2026-06-11

### Added
- **LAS 1.2–1.4** read/write with WKT CRS preservation via EVLRs.
- **LAZ** read/write via `laszip` compression.
- **PCD** read/write (binary, ASCII, binary_compressed).
- **E57** read/write with CRS metadata preservation.
- **ROS 2 MCAP** read/write with PointCloud2 CDR message parsing.
- **ROS 1 bag** read (PointCloud2 message extraction).
- **CRS transformation** via `proj` crate (`--target-crs` flag, `proj` Cargo feature).
- **Python bindings** via PyO3 + maturin (`pip install rubipont`).
- **WASM** build support (`wasm32-unknown-unknown`).
- **CLI** binary (`rp`) with `convert`, `info`, and `formats` commands.
- Criterion benchmarks for format conversion performance.
- Round-trip integration tests (LAS ↔ PCD, LAS ↔ LAZ, LAS ↔ E57, LAS ↔ MCAP).
- GitHub Actions CI (build, test, clippy).
- Dual MIT / Apache 2.0 licence.

[0.1.2]: https://github.com/mabo-du/rubipont/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/mabo-du/rubipont/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/mabo-du/rubipont/releases/tag/v0.1.0
