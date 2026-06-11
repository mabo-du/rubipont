# Changelog

All notable changes to rubipont are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.2] — 2026-06-12

### Fixed
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
