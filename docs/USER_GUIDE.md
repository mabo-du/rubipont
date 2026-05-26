# rubipont User Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Installation](#installation)
3. [Command Reference](#command-reference)
4. [Supported Formats](#supported-formats)
5. [Conversion Workflows](#conversion-workflows)
6. [Metadata Handling](#metadata-handling)
7. [CRS Transformation](#crs-transformation)
8. [Performance](#performance)
9. [Troubleshooting](#troubleshooting)
10. [Python API](#python-api)

---

## Introduction

rubipont translates point cloud data between formats used in geospatial surveying (LAS/LAZ), terrestrial laser scanning (E57), robotics (PCD, ROS bag/MCAP), and machine learning (PCD). It preserves metadata across conversions, strips alignment padding from ROS data, and provides optional coordinate reprojection.

**Core design principles:**
- **Lossless**: no point discarded, no metadata silently dropped
- **Memory-efficient**: memory-mapped I/O for large files; bounded buffers for compressed formats
- **Metadata-preserving**: WKT CRS survives E57 ↔ LAS 1.4 round-trips; sidecar files for formats that cannot embed it

---

## Installation

### From Source

```bash
git clone https://github.com/your-org/rubipont
cd rubipont
cargo build --release --workspace
```

The CLI binary is at `target/release/rp`.

### Install CLI System-Wide

```bash
cargo install --path rubipont-cli
```

### Python Package

```bash
cd rubipont-py
pip install maturin
maturin build --release
pip install target/wheels/rubipont-*.whl
```

### WASM Build

```bash
bash wasm-demo/build.sh
cd wasm-demo
python3 -m http.server 42042
# Open http://localhost:42042 in a browser
```

### Feature Flags

| Feature | Default | Description |
|---|---|---|
| `mcap-io` | ✅ | MCAP + ROS bag support (requires C compiler for lz4/zstd/bzip2) |
| `proj` | — | CRS transformation (requires PROJ C library) |
| `wasm` | — | Disables native-C deps for WASM compilation |

Build with specific features:
```bash
cargo build --release --features proj
cargo build --release --no-default-features --features wasm
```

---

## Command Reference

### `rp convert <input> <output>`

Convert a point cloud file between formats. Input and output formats are auto-detected from file extensions.

```bash
rp convert scan.las cloud.pcd
rp convert compressed.laz output.las
rp convert vehicle_run.mcap archive.laz
rp convert survey.e57 output.las
```

**Options:**

| Flag | Description |
|---|---|
| `--target-crs <EPSG>` | Reproject coordinates to target CRS (requires `proj` feature) |

```bash
rp convert input.las output.pcd --target-crs 3857
```

### `rp info <input>`

Display metadata about a point cloud file.

```bash
$ rp info scan.las
File: scan.las
Points: 1847234
Point size: 26 bytes
Integer coords: true
Scale: (0.01, 0.01, 0.01)
Offset: (500000, 6000000, 0)
```

### `rp formats`

List all supported formats with read/write capability.

```bash
$ rp formats
Supported formats:
  .las  — ASPRS LAS 1.2 (read/write)
  .laz  — Compressed LAS (read/write)
  .pcd  — Point Cloud Data (read/write)
  .e57  — ASTM E57 (read/write)
  .mcap — ROS 2 MCAP (read/write)*
  .bag  — ROS 1 bag (read)*
* Requires mcap-io feature
```

---

## Supported Formats

### LAS (all versions 1.2–1.4)

The ASPRS LAS format is the standard for airborne and terrestrial geospatial LiDAR.

| Feature | Support |
|---|---|
| Point format 0 | ✅ Read/write |
| Point formats 1–5 | ✅ Read |
| Point formats 6–10 (LAS 1.4) | ✅ Read |
| VLRs | ✅ Read (passthrough) |
| EVLRs | ✅ Read (passthrough) |
| WKT CRS | ✅ Read/write (LAS 1.4) |
| Coordinate scale/offset | ✅ Preserved in round-trip |
| Classification | ✅ Read (passthrough) |

**Note:** Internal representation uses f64 XYZ + u16 intensity. Additional fields (GPS time, RGB, NIR) are read but not written in the current version.

### LAZ

Lossless LAS compression. Uses the laszip algorithm with predictive coding and arithmetic encoding.

| Feature | Support |
|---|---|
| Decompression | ✅ Sequential and parallel |
| Compression | ✅ Single-threaded |
| LAS 1.4 compatibility mode | ✅ Read |
| Chunked random access | ✅ Via laz crate |

### PCD

Point Cloud Library format used in robotics and computer vision.

| Feature | Support |
|---|---|
| DATA ascii | ✅ Read/write |
| DATA binary | ✅ Read/write |
| DATA binary_compressed | ✅ Read/write |
| LZF decompression | ✅ |
| SoA transposition | ✅ Automatic |
| VIEWPOINT | ✅ Read (sidecar on write) |

### E57

ASTM E2807 standard for terrestrial laser scanning.

| Feature | Support |
|---|---|
| Cartesian coordinates | ✅ Read/write |
| Spherical coordinates | ✅ Read (auto-converted to Cartesian) |
| Intensity | ✅ Read/write |
| Colour (RGB) | ✅ Read |
| Multi-scan files | ✅ Read (first scan; warns about others) |
| CRS (WKT) | ✅ Read/write |
| Images2D | Read (available via crate) |

### ROS 2 MCAP

Modern robotics container format (Foxglove).

| Feature | Support |
|---|---|
| LZ4 chunk compression | ✅ |
| Zstd chunk compression | ✅ |
| Channel/topic filtering | ✅ (topics containing "points" or "lidar") |
| CDR encapsulation | ✅ (4-byte header stripped automatically) |
| Eigen padding stripping | ✅ (reads only valid field bytes) |
| Trailing index seeking | ✅ (via mcap crate) |

### ROS 1 Bag

Legacy robotics logging format.

| Feature | Support |
|---|---|
| bz2 chunk decompression | ✅ |
| PointCloud2 extraction | ✅ |
| Topic filtering | ✅ (topics containing "points" or "lidar") |
| Eigen padding stripping | ✅ |

---

## Conversion Workflows

### Geospatial → ML Training

```
Aerial LAS survey → PCD for training dataset
$ rp convert survey.las training_data.pcd

With CRS to UTM reprojection:
$ rp convert survey.las training_data.pcd --target-crs 32633
```

### Terrestrial Scan → Robotics Simulation

```
E57 scan → ROS 2 MCAP for SLAM testing
$ rp convert scan.e57 simulation.mcap
```

### ROS Dataset → Geospatial Archive

```
ROS 2 MCAP → LAZ for long-term storage
$ rp convert vehicle_run.mcap archive.laz
```

### Round-Trip Verification

```
LAS → PCD → LAS (verify point count preserved)
$ rp convert original.las intermediate.pcd
$ rp convert intermediate.pcd roundtrip.las
$ rp info roundtrip.las
```

---

## Metadata Handling

### What Gets Preserved

| Metadata | LAS 1.4 | E57 | PCD | MCAP |
|---|---|---|---|---|
| Coordinate scale/offset | ✅ Native | — | — | — |
| WKT CRS | ✅ EVLR | ✅ XML | 📎 Sidecar | — |
| VIEWPOINT | — | — | 📎 Sidecar | — |
| Intensity | ✅ | ✅ | ✅ | ✅ |

**Legend:** ✅ = embedded natively, 📎 = sidecar JSON file

### Sidecar Files

When converting to a format that cannot embed certain metadata, rubipont writes a `.meta.json` file alongside the output:

```json
{
  "source_format": "e57",
  "crs_wkt": "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\"]]",
  "coordinate_scale": [0.01, 0.01, 0.01],
  "coordinate_offset": [500000.0, 6000000.0, 0.0],
  "las_version": [1, 4],
  "generated_by": "rubipont"
}
```

Sidecar files are created when CRS, coordinate scale/offset, or VIEWPOINT metadata would otherwise be lost.

---

## CRS Transformation

Coordinate Reference System reprojection is available via the `--target-crs` CLI option using the `proj` crate.

### Requirements

```bash
# Install PROJ C library
sudo apt install libproj-dev   # Debian/Ubuntu
brew install proj              # macOS

# Build rubipont with proj feature
cargo build --release --features proj
```

### Usage

```bash
# Convert from WGS84 to Web Mercator
rp convert scan.las output.pcd --target-crs 3857

# Convert to UTM zone 33N
rp convert scan.las output.pcd --target-crs 32633
```

### How It Works

1. rubipont reads the source file's CRS metadata (WKT string from LAS 1.4 EVLRs or E57 XML)
2. Extracts the EPSG code from the WKT (e.g., `AUTHORITY["EPSG","4326"]`)
3. Creates a PROJ transformation object from source EPSG to target EPSG
4. Applies the transformation to every point's XYZ coordinates during conversion
5. Writes the target CRS metadata to the output file

**Note:** Current implementation performs 2D horizontal transformation. Z values pass through unchanged.

---

## Performance

### Benchmark Results

Run on your hardware with:
```bash
cargo bench -p rubipont-core
```

Three benchmarks measure throughput for 10,000-point datasets:

| Benchmark | Description |
|---|---|
| `las_to_pcd_10k` | Convert LAS → PCD binary |
| `las_to_laz_10k` | Compress LAS → LAZ |
| `laz_to_las_10k` | Decompress LAZ → LAS |

### Memory Usage

- **Uncompressed formats** (LAS binary PCD): ~0 bytes extra (memory-mapped I/O)
- **Compressed formats** (LAZ, PCD binary_compressed): ~50K × point_size (~1.3 MB) ring buffer
- **E57**: buffered; points written atomically in `finalize()`
- **MCAP/ROS bag**: entire point cloud loaded into memory (current implementation)

### Large File Handling

For files > 1 GB:
- LAS/PCD use memory-mapped I/O — limited only by virtual address space
- LAZ uses chunked streaming — decompresses one 50K-point chunk at a time
- E57 uses paged I/O — reads in page-sized units
- MCAP uses memory-mapped I/O — OS handles page cache

---

## Troubleshooting

### "Unsupported format: xyz"

rubipont determines format by file extension. Ensure your file has a recognised extension: `.las`, `.laz`, `.pcd`, `.e57`, `.mcap`, `.bag`.

### "CRS transformation requires the 'proj' feature"

The `proj` feature was excluded at build time. Rebuild with:
```bash
cargo build --release --features proj
```

### MCAP or ROS bag support not available

The `mcap-io` feature was excluded (it is enabled by default). Ensure you didn't use `--no-default-features` unless needed for WASM.

### "Parse error" on a valid file

Some LAS files have non-standard headers or formats that the `las` crate cannot parse. Try running `lasinfo` (from LAStools) to validate the file.

### E57 multi-scan warning

E57 files with multiple point clouds will only read the first scan. Each scan is written as a separate point cloud in the output format when writing E57.

### Large MCAP files cause high memory usage

The current MCAP reader loads all matching messages into memory before conversion. For very large files (> 10 GB), consider using the `mcap` CLI to filter first:
```bash
mcap filter input.mcap -o filtered.mcap --topics /points2
rp convert filtered.mcap output.laz
```

---

## Python API

### Functions

```python
import rubipont

# Convert a file between formats
rubipont.convert("input.las", "output.pcd")

# Get file metadata
info = rubipont.info("scan.las")
# Returns a string containing:
#   File: scan.las
#   Points: 1847234
#   Point size: 26 bytes
#   Integer coords: true
#   Scale: (0.01, 0.01, 0.01)
#   Offset: (500000, 6000000, 0)

# List supported formats
formats = rubipont.formats()
# Returns:
#   ['.las  — ASPRS LAS 1.2 (read/write)',
#    '.laz  — Compressed LAS (read/write)',
#    '.pcd  — Point Cloud Data (read/write)',
#    '.e57  — ASTM E57 (read/write)',
#    '.mcap — ROS 2 MCAP (read)']
```

### Building the Wheel

```bash
cd rubipont-py
maturin build --release
pip install target/wheels/rubipont-*.whl
```
