# pointbridge — LiDAR Format Translation Library Design Spec

## Overview

An open-source Rust library (with CLI) for zero-copy, lossless conversion between
the major LiDAR point cloud data formats: **LAS/LAZ**, **PCD**, **E57**, and
**ROS bag/MCAP**. Working name: **pointbridge**.

## Problem

Researchers and engineers who work across geospatial, robotics, and AEC domains
routinely convert point cloud data between incompatible formats. Existing tools
(PDAL, LAStools, CloudCompare, Open3D, ROS utilities) each have critical gaps:

- **PDAL** suffers OOM crashes on files >10GB and silently strips CRS metadata
- **LAStools** is partially proprietary and LAS-only
- **CloudCompare** destroys E57 metadata (timestamps, pose, images)
- **Open3D** discards geodetic metadata entirely
- **ROS tooling** bloats data by ~46% due to Eigen alignment padding

No single tool handles all formats losslessly and efficiently at scale.

## Target Users

- Academic researchers working across geospatial and robotics domains
- Robotics and AV teams ingesting data from multiple sensor vendors
- GIS engineers processing terrestrial laser scan data
- ML engineers preparing training datasets from mixed-source point clouds

## Licence

MIT / Apache 2.0 dual-licence.

## Workspace Architecture

Four-crate Cargo workspace:

```
pointbridge/
├── Cargo.toml                  # [workspace] root
├── pointbridge-core/           # library — format-agnostic translation engine
│   ├── Cargo.toml              # deps: las-rs, laz-rs, e57, pasture, bytemuck
│   └── src/
│       ├── lib.rs
│       ├── format/             # one module per format
│       │   ├── las.rs          # LAS 1.2/1.3/1.4 via las-rs
│       │   ├── laz.rs          # LAZ via laz-rs (parallel decompression)
│       │   ├── pcd.rs          # PCD binary/ASCII/binary_compressed
│       │   └── e57.rs          # E57 read via e57 crate
│       ├── pipeline.rs         # Reader → translation → Writer orchestration
│       ├── layout.rs           # Unified point layout (pasture-based)
│       └── error.rs            # PointbridgeError enum
├── pointbridge-cli/            # CLI binary
│   ├── Cargo.toml              # deps: pointbridge-core, clap
│   └── src/main.rs
└── pointbridge-py/             # PyO3 bindings (Phase 3)
    ├── Cargo.toml              # deps: pointbridge-core, pyo3, numpy
    ├── src/lib.rs
    └── pointbridge/
        └── __init__.py
```

## Core Pipeline

```
[Source file] → Reader trait → PointLayout + PointChunks → Writer trait → [Target file]
                                              ↕
                                     PipelineContext
                                   (CRS, metadata, sidecars)
```

### Reader Trait

```rust
#[async_trait]
trait PointCloudReader: Send {
    fn can_read(path: &Path) -> bool;
    async fn open(path: &Path) -> Result<Box<Self>>;
    async fn read_chunk(&mut self) -> Result<Option<PointChunk>>;
    fn metadata(&self) -> &PipelineContext;
}
```

### Writer Trait

```rust
#[async_trait]
trait PointCloudWriter: Send {
    fn supported_extensions() -> &'static [&'static str];
    async fn create(path: &Path, layout: &PointLayout, ctx: &PipelineContext) -> Result<Box<Self>>;
    async fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()>;
    async fn finalize(&mut self) -> Result<()>;
}
```

### Reader/Writer Registry

Extension-keyed `HashMap` for auto-detection. Adding a new format means
registering a constructor pair, not editing a match statement.

## Memory Model

Two regimes selected dynamically based on format:

| Format type | Strategy | Details |
|---|---|---|
| Uncompressed (LAS, binary PCD, uncompressed MCAP) | **mmap** + `bytemuck` cast | Kernel-managed page cache; zero overhead regardless of file size |
| Compressed (LAZ, binary_compressed PCD) | **Fixed ring buffer** (50K points) | Matches LAZ chunk size; parallel decompression fills buffer; writer drains it |

## Precision Strategy

LAS coordinates are stored as `i32` with header scale/offset. The pipeline
preserves raw integers internally. Conversion to `f32`/`f64` happens only at
the final serialisation step for float-native targets (PCD, ROS). LAS→LAS
writes integers directly — zero precision loss.

## Metadata & Pipeline Context

```rust
struct PipelineContext {
    crs_wkt: Option<String>,
    coordinate_scale: Option<LasScale>,
    coordinate_offset: Option<LasOffset>,
    viewpoint: Option<PcdViewpoint>,
    e57_images: Vec<E57Image>,
    extra_fields: HashMap<String, MetadataValue>,
}
```

- Formats that can embed metadata (LAS EVLRs) do so natively
- Formats that cannot (PCD) get a sidecar `.meta.json` file
- Irreconcilable metadata is logged as a warning on stderr

## CLI Design

```bash
pointbridge convert input.las output.pcd              # basic conversion
pointbridge convert input.las output.laz -c 6          # with compression level
pointbridge convert input.e57 output.pcd --extract-images ./images/
pointbridge info input.las                              # file header/metadata
pointbridge formats                                     # list supported formats
```

## Error Handling

```rust
#[derive(Debug, thiserror::Error)]
pub enum PointbridgeError {
    UnsupportedFormat(String),
    ParseError { format: String, offset: u64, detail: String },
    CorruptChunk { format: String, chunk: u64, detail: String },
    Io(#[from] std::io::Error),
    PrecisionLoss(String),
}
```

- Zero panics in production code. No `.unwrap()`, no `.expect()`, no unsafe
  dereferences outside `bytemuck`/mmap casts.
- Corrupt chunks are logged and skipped; batch processing continues.
- Round-trip tests verify lossless conversion for every format pair.

## Implementation Phases

### Phase 1 — MVP (4–6 weeks)

1. Scaffold workspace + pointbridge-core skeleton
2. LAS 1.2 read/write (via las-rs)
3. PCD binary read/write (custom on pasture)
4. LAZ read/write (via laz-rs parallel decompression)
5. CLI with auto-detection
6. Round-trip test suite
7. CI pipeline

### Phase 2 — Extended Formats (2–4 weeks)

8. E57 read (via e57 crate)
9. LAS 1.3/1.4 read/write
10. PCD binary_compressed + ASCII
11. Sidecar metadata preservation

### Phase 3 — Stretch Goals

12. ROS bag/MCAP read
13. Python bindings (PyO3)
14. WASM build
15. CRS transformation

## Key Dependencies

| Crate | Purpose | Status |
|---|---|---|
| `las-rs` | LAS 1.2–1.4 read/write | Stable, maintained |
| `laz-rs` | LAZ compression/decompression | Stable, parallel feature flag |
| `e57` | E57 read | Pure Rust, maintained |
| `pasture` | Point cloud framework (PointLayout, VectorBuffer) | Stable |
| `bytemuck` | Zero-copy byte slice casting | Stable |
| `clap` | CLI argument parsing | Stable |
| `thiserror` | Error type derivation | Stable |

## Risks

| Risk | Mitigation |
|---|---|
| LAZ spec edge cases (1.4 compatibility mode) | Test against real-world multi-return files; document known issues |
| `pasture` API instability | Pin minor version; contribute upstream fixes if needed |
| E57 write complexity | Defer to Phase 3 unless user demand justifies earlier |
| Performance gap vs LAStools | Benchmark early; profile hot paths; use laz-rs parallel feature |
