# rubipont — Zero-Copy LiDAR Format Translation

**rubipont** (Latin: *rubigo* "rust" + *pons/pontis* "bridge" = "rust bridge") is a high-performance Rust library and CLI tool for lossless conversion between major LiDAR point cloud formats.

```
$ rp convert survey.las output.pcd
$ rp convert rosbag.mcap archive.laz --target-crs 3857
$ rp info scan.las
```

**Current version:** [v0.1.3](CHANGELOG.md) — see [CHANGELOG](CHANGELOG.md) for full history.

## Features

### Supported Formats

| Format | Read | Write | Description |
|---|---|---|---|
| **LAS 1.2–1.4** | ✅ | ✅ | Geospatial surveying standard with WKT CRS, EVLRs |
| **LAZ** | ✅ | ✅ | Lossless LAS compression (laszip) |
| **PCD** | ✅ | ✅ | Point Cloud Library (binary, ASCII, binary_compressed) |
| **E57** | ✅ | ✅ | ASTM terrestrial laser scanning standard |
| **ROS 2 MCAP** | ✅ | ✅ | Robotic middleware container format |
| **ROS 1 bag** | ✅ | — | Legacy robotics logging format (read only) |

### Key Capabilities

- **Multi-format pipeline** — single command converts between any supported format pair
- **WKT CRS preservation** — coordinate reference system metadata survives E57 ↔ LAS 1.4 and LAZ round-trips with correct EPSG extraction
- **CRS transformation** — optional `--target-crs <epsg>` for coordinate reprojection (requires `proj` feature)
- **Sidecar metadata** — `.meta.json` files preserve CRS / VIEWPOINT when target format cannot embed it
- **Eigen padding stripping** — automatically removes C++ struct alignment padding (up to 65% bloat reduction)
- **Shared PointCloud2 parser** — single CDR-encoded `sensor_msgs/PointCloud2` parser for both MCAP and ROS 1 bag (handles big-endian, NaN filtering, missing fields, zero point_step)
- **Python bindings** — `pip install rubipont` via PyO3 + maturin
- **WASM build** — browser-compatible build for web-based tooling

## Quick Start

### Prerequisites
- Rust 1.75+
- For CRS transformation: PROJ C library (`libproj-dev` on Debian/Ubuntu)

### Build & Run

```bash
git clone https://github.com/mabo-du/rubipont
cd rubipont

# Build all crates
cargo build --workspace

# Run the CLI
cargo run -p rubipont-cli -- convert input.las output.pcd
cargo run -p rubipont-cli -- info input.las
cargo run -p rubipont-cli -- formats
```

### Install CLI

```bash
cargo install --path rubipont-cli
rp convert scan.laz points.pcd
```

### Python Bindings

```bash
cd rubipont-py
pip install maturin
maturin build --release
pip install target/wheels/rubipont-*.whl
python -c "import rubipont; print(rubipont.formats())"
```

### Run Tests

```bash
cargo test --workspace
```

### Run Benchmarks

```bash
cargo bench -p rubipont-core
```

## Usage Guide

### Basic Conversion

```bash
# Convert LAS to PCD
rp convert input.las output.pcd

# Convert LAZ to LAS
rp convert compressed.laz output.las

# Convert E57 to LAS (with CRS preserved)
rp convert scan.e57 output.las

# Convert ROS 2 MCAP to LAZ
rp convert vehicle_run.mcap output.laz
```

### Inspecting Files

```bash
# Show file metadata (points, point size, CRS, scale/offset)
rp info survey.las

# Show supported formats
rp formats
```

### Advanced Options

```bash
# CRS transformation (requires proj feature)
rp convert input.las output.pcd --target-crs 3857

# CRS transformation from MCAP data
rp convert lidar.mcap output.laz --target-crs 4326
```

### From Python

```python
import rubipont

# Convert file
rubipont.convert("scan.las", "cloud.pcd")

# Inspect file (includes CRS when available)
info = rubipont.info("scan.las")
print(info)

# List formats
for fmt in rubipont.formats():
    print(fmt)
```

## Project Structure

```
rubipont/
├── Cargo.toml                    # Workspace root
├── rubipont-core/                # Core translation engine (library)
│   ├── src/format/               # Format-specific modules
│   │   ├── las.rs                # LAS 1.2–1.4 reader/writer
│   │   ├── laz.rs                # LAZ reader/writer
│   │   ├── pcd.rs                # PCD reader/writer
│   │   ├── e57.rs                # E57 reader/writer
│   │   ├── mcap.rs               # ROS 2 MCAP reader/writer
│   │   ├── bag.rs                # ROS 1 bag reader
│   │   ├── pointcloud2.rs        # Shared CDR-encoded PointCloud2 parser
│   │   └── mod.rs                # Module registry
│   ├── src/pipeline.rs           # Convert orchestrator + dispatch + format_info
│   ├── src/layout.rs             # PointLayout, PointChunk, PipelineContext
│   ├── src/transform.rs          # CRS transformation (EPSG extraction + proj)
│   ├── src/error.rs              # Error types
│   ├── src/array.rs              # Array read helpers
│   └── benches/conversion.rs     # Criterion benchmarks
├── rubipont-cli/                 # CLI binary (rp)
├── rubipont-py/                  # Python bindings (PyO3)
├── wasm-demo/                    # WASM demo page
├── docs/                         # Design specs, research, ADRs
│   └── adr/                      # Architecture Decision Records
│       └── 001-internal-point-format.md
└── CHANGELOG.md                  # Version history
```

## Memory & Performance Notes

| Format | Memory per read | Memory per write |
|---|---|---|
| LAS | ~0 bytes (memory-mapped via `las` crate) | Streamed (per-point write via `las` crate) |
| LAZ | ~50k × 26B ring buffer (chunked streaming) | Per-point compression (chunked) |
| PCD | ~0 bytes (memory-mapped via `BufReader`) | All points buffered, then written in `finalize()` |
| E57 | All points buffered in internal format on construction | All points buffered, then written in `finalize()` |
| MCAP | File read into `Vec<u8>` + point data in internal format | All points buffered, then written in `finalize()` |
| Bag | All points buffered on construction (doubly iterates chunks) | N/A (read only) |

**Known limitations:**
- MCAP, bag, E57, and PCD writers currently buffer all points in RAM before writing. Large files (>50M points) may require significant memory. The v0.3.0 PointBatch migration plans to address this with streaming support.
- Bag reader iterates chunk records twice (connections + messages). Single-pass refactor planned.

## Licence

MIT / Apache 2.0 dual-licence.
