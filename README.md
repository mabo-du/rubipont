# rubipont — Zero-Copy LiDAR Format Translation

**rubipont** (Latin: *rubigo* "rust" + *pons/pontis* "bridge" = "rust bridge") is a high-performance Rust library and CLI tool for lossless conversion between major LiDAR point cloud formats.

```
$ rp convert survey.las output.pcd
$ rp convert rosbag.mcap archive.laz --target-crs 3857
$ rp info scan.las
```

## Features

### Supported Formats

| Format | Read | Write | Description |
|---|---|---|---|
| **LAS 1.2** | ✅ | ✅ | Geospatial surveying standard |
| **LAS 1.3/1.4** | ✅ | ✅ | Modern multi-return, WKT CRS, EVLRs |
| **LAZ** | ✅ | ✅ | Lossless LAS compression (laszip) |
| **PCD** | ✅ | ✅ | Point Cloud Library format (binary, ASCII, binary_compressed) |
| **E57** | ✅ | ✅ | ASTM terrestrial laser scanning standard |
| **ROS 2 MCAP** | ✅ | ✅ | Robotic middleware container format |
| **ROS 1 bag** | ✅ | — | Legacy robotics logging format |

### Key Capabilities

- **Zero-copy architecture** — memory-mapped I/O for uncompressed formats; bounded ring buffers for compressed formats
- **Eigen padding stripping** — automatically removes C++ struct alignment padding (up to 65% bloat reduction)
- **WKT CRS preservation** — coordinate reference system metadata survives E57 ↔ LAS 1.4 round-trips
- **Sidecar metadata** — `.meta.json` files preserve CRS when target format cannot embed it
- **CRS transformation** — optional `--target-crs <epsg>` for coordinate reprojection (requires `proj` feature)
- **Python bindings** — `pip install rubipont` via PyO3 + maturin
- **WASM support** — browser-compatible build for web-based tooling

## Quick Start

### Prerequisites
- Rust 1.94+ 
- For CRS transformation: PROJ C library (`libproj-dev` on Debian/Ubuntu)

### Build & Run

```bash
git clone https://github.com/your-org/rubipont
cd rubipont

# Build all crates
cargo build --workspace

# Run the CLI
cargo run -p rubipont-cli -- convert input.las output.pcd
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
# Show file metadata
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

# Inspect file
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
│   │   └── bag.rs                # ROS 1 bag reader
│   ├── src/pipeline.rs           # Convert orchestrator + sidecar
│   ├── src/transform.rs          # CRS transformation
│   └── benches/conversion.rs     # Criterion benchmarks
├── rubipont-cli/                 # CLI binary (rp)
├── rubipont-py/                  # Python bindings (PyO3)
├── wasm-demo/                    # WASM demo page
└── docs/                         # Design specs, research, plans
```

## Licence

MIT / Apache 2.0 dual-licence.
