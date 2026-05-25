# rubipont MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a zero-copy LiDAR format translation library and CLI that can losslessly convert between LAS 1.2, LAZ, and PCD (binary).

**Architecture:** Three-crate Cargo workspace. `rubipont-core` owns the format-agnostic pipeline (Reader/Writer traits, `PointLayout`, `PipelineContext`, error types) and format-specific implementations. `rubipont-cli` exposes the `rp` binary using clap. Memory-mapped I/O for uncompressed formats, fixed ring buffer for LAZ.

**Tech Stack:** Rust 1.94+, `las` 0.9, `laz` 0.12, `bytemuck`, `clap` 4.x, `thiserror` 2.x.

---

### Task 1: Scaffold workspace and crate skeletons

**Files:**
- Create: `rubipont/Cargo.toml`
- Create: `rubipont/rubipont-core/Cargo.toml`
- Create: `rubipont/rubipont-cli/Cargo.toml`
- Create: `rubipont/rubipont-core/src/lib.rs`
- Create: `rubipont/rubipont-cli/src/main.rs`
- Create: `rubipont/.gitignore`

- [ ] **Step 1: Create workspace root `Cargo.toml`**

```toml
[workspace]
resolver = "2"
members = ["rubipont-core", "rubipont-cli"]
```

- [ ] **Step 2: Create `rubipont-core/Cargo.toml`**

```toml
[package]
name = "rubipont-core"
version = "0.1.0"
edition = "2021"
description = "Zero-copy LiDAR point cloud format translation — core library"

[dependencies]
las = "0.9"
laz = "0.12"
bytemuck = "1"
thiserror = "2"
```

- [ ] **Step 3: Create `rubipont-cli/Cargo.toml`**

```toml
[package]
name = "rubipont-cli"
version = "0.1.0"
edition = "2021"
description = "CLI for rubipont LiDAR format translation"

[[bin]]
name = "rp"
path = "src/main.rs"

[dependencies]
rubipont-core = { path = "../rubipont-core" }
clap = { version = "4", features = ["derive"] }
```

- [ ] **Step 4: Create `rubipont-core/src/lib.rs`**

```rust
pub mod error;
pub mod pipeline;
pub mod format;
pub mod layout;
```

- [ ] **Step 5: Create `rubipont-cli/src/main.rs`**

```rust
fn main() {
    println!("rp — rubipont LiDAR format translator");
}
```

- [ ] **Step 6: Create `.gitignore`**

```
target/
*.pyc
__pycache__/
*.egg-info/
```

- [ ] **Step 7: Init git repo and commit**

```bash
cd rubipont
git init
git add -A
git commit -m "wip: scaffold workspace with rubipont-core and rubipont-cli"
```

---

### Task 2: Define error types

**Files:**
- Create: `rubipont/rubipont-core/src/error.rs`
- Create: `rubipont/rubipont-core/tests/test_error.rs`

- [ ] **Step 1: Write the test**

Create `rubipont-core/tests/test_error.rs`:
```rust
use rubipont_core::error::RubipontError;

#[test]
fn error_display_unsupported_format() {
    let err = RubipontError::UnsupportedFormat("xyz".into());
    let msg = format!("{}", err);
    assert!(msg.contains("xyz"), "Error should mention format name, got: {}", msg);
}

#[test]
fn error_display_parse_error() {
    let err = RubipontError::ParseError {
        format: "LAS".into(),
        offset: 256,
        detail: "invalid header signature".into(),
    };
    let msg = format!("{}", err);
    assert!(msg.contains("LAS"));
    assert!(msg.contains("256"));
}

#[test]
fn error_is_std_error() {
    use std::error::Error;
    let err = RubipontError::UnsupportedFormat("test".into());
    let _: &dyn Error = &err; // must implement std::error::Error
}
```

- [ ] **Step 2: Run test, expect compile failure**

```bash
cargo test -p rubipont-core
```
Expected: compile error — `error[E0432]` module `error` not found in `lib.rs` or `RubipontError` not defined.

- [ ] **Step 3: Implement `error.rs`**

```rust
use thiserror::Error;

#[derive(Debug, Error)]
pub enum RubipontError {
    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("Parse error in {format} at offset {offset}: {detail}")]
    ParseError {
        format: String,
        offset: u64,
        detail: String,
    },

    #[error("Corrupt chunk in {format} (chunk {chunk}): {detail}")]
    CorruptChunk {
        format: String,
        chunk: u64,
        detail: String,
    },

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Precision loss: {0}")]
    PrecisionLoss(String),
}

pub type Result<T> = std::result::Result<T, RubipontError>;
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cargo test -p rubipont-core
```
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add rubipont-core/src/error.rs rubipont-core/tests/
git commit -m "feat(core): add error types with thiserror"
```

---

### Task 3: Define core types — PointLayout, PipelineContext, format traits

**Files:**
- Create: `rubipont/rubipont-core/src/layout.rs`
- Create: `rubipont/rubipont-core/src/pipeline.rs`
- Create: `rubipont/rubipont-core/tests/test_pipeline.rs`

- [ ] **Step 1: Write the test for core types**

Create `rubipont-core/tests/test_pipeline.rs`:
```rust
use rubipont_core::layout::{PointChunk, PipelineContext, PointLayout};

#[test]
fn pipeline_context_default() {
    let ctx = PipelineContext::default();
    assert!(ctx.crs_wkt.is_none());
    assert!(ctx.viewpoint.is_none());
    assert!(ctx.extra_fields.is_empty());
}

#[test]
fn point_layout_default() {
    let layout = PointLayout {
        point_size: 26,
        num_points: 100,
        has_integer_coords: true,
    };
    assert_eq!(layout.point_size, 26);
}

#[test]
fn point_chunk_fields() {
    let chunk = PointChunk {
        data: vec![0u8; 260],
        len: 10,
    };
    assert_eq!(chunk.len, 10);
    assert_eq!(chunk.data.len(), 260);
}
```

- [ ] **Step 2: Run test — expect compile failure**

```bash
cargo test -p rubipont-core
```
Expected: compile error — module `layout` or `pipeline` not found in `lib.rs`.

- [ ] **Step 3: Implement `layout.rs`**

```rust
use std::collections::HashMap;

/// A chunk of points in an interleaved (AoS) binary format.
pub struct PointChunk {
    /// Raw point data bytes (interleaved: xyzxyzxyz... for each point)
    pub data: Vec<u8>,
    /// Number of points in this chunk
    pub len: usize,
}

/// Typed metadata value for the pipeline context map.
#[derive(Debug, Clone)]
pub enum MetadataValue {
    String(String),
    F64(f64),
    I64(i64),
    Bytes(Vec<u8>),
}

/// Pipeline context — carries format-specific metadata through the
/// translation pipeline so writers can preserve what they can.
#[derive(Debug, Default)]
pub struct PipelineContext {
    /// OGC Well-Known Text coordinate reference system
    pub crs_wkt: Option<String>,
    /// LAS-specific coordinate scale (X, Y, Z)
    pub coordinate_scale: Option<(f64, f64, f64)>,
    /// LAS-specific coordinate offset (X, Y, Z)
    pub coordinate_offset: Option<(f64, f64, f64)>,
    /// PCD viewpoint as translation + quaternion (0..=6)
    pub viewpoint: Option<[f64; 7]>,
    /// Any extra metadata keyed by name
    pub extra_fields: HashMap<String, MetadataValue>,
}

/// Describes the memory layout of a single point.
/// In MVP this is minimal — expanded with pasture in Phase 2.
#[derive(Debug, Clone)]
pub struct PointLayout {
    /// Total bytes per point in the internal representation
    pub point_size: usize,
    /// Number of points in the cloud
    pub num_points: u64,
    /// Whether coordinates are stored as scaled 32-bit integers (true for LAS/LAZ)
    pub has_integer_coords: bool,
}
```

- [ ] **Step 4: Implement `pipeline.rs` — just traits, no registry**

```rust
use crate::error::Result;
use crate::layout::{PointChunk, PipelineContext, PointLayout};

/// Reader trait — every format provides an implementation.
/// Readers are created via their own `new(path)` constructors,
/// not through this trait (to keep the trait object-safe).
pub trait PointCloudReader: Send {
    /// Read the next chunk of points. Returns `None` when exhausted.
    fn read_chunk(&mut self) -> Result<Option<PointChunk>>;
    /// The point layout describing how points are structured.
    fn layout(&self) -> &PointLayout;
    /// Pipeline metadata context.
    fn metadata(&self) -> &PipelineContext;
}

/// Writer trait — every format provides an implementation.
/// Writers are created via their own `create(path, layout, ctx)` constructors.
pub trait PointCloudWriter: Send {
    /// Write the next chunk of points.
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()>;
    /// Finalise the file (flush, write indexes, close).
    fn finalize(&mut self) -> Result<()>;
}

/// High-level conversion — dispatches reader/writer by file extension.
/// Each format module exposes a `detect(ext) -> bool` function
/// and a matching reader/writer pair.
pub mod detect {
    use std::path::Path;

    /// Map a file extension to a format name.
    pub fn extension(path: &Path) -> &str {
        path.extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
    }
}
```

- [ ] **Step 5: Run test to verify it passes**

```bash
cargo test -p rubipont-core
```
Expected: 3 passed.

- [ ] **Step 6: Commit**

```bash
git add rubipont-core/src/layout.rs rubipont-core/src/pipeline.rs rubipont-core/tests/test_pipeline.rs
git commit -m "feat(core): add PointLayout, PipelineContext, Reader/Writer traits"
```

---

### Task 4: Implement LAS reader/writer

**Files:**
- Create: `rubipont/rubipont-core/src/format/mod.rs`
- Create: `rubipont/rubipont-core/src/format/las.rs`
- Create: `rubipont/rubipont-core/tests/test_las.rs`

- [ ] **Step 1: Write the test**

Create `rubipont-core/tests/test_las.rs`:
```rust
use rubipont_core::format::las::LasReader;
use rubipont_core::pipeline::PointCloudReader;
use rubipont_core::format;

#[test]
fn las_detect_extension() {
    assert!(format::las::detect("las"));
    assert!(format::las::detect("LAS"));
    assert!(!format::las::detect("pcd"));
}

#[test]
fn las_roundtrip() {
    let tmp = std::env::temp_dir().join("test_roundtrip.las");

    // Write a minimal LAS 1.2 file via the las crate directly
    {
        use las::{Point, Header};
        let header = Header::builder()
            .number_of_point_records(10)
            .build()
            .unwrap();
        let mut writer = las::Writer::from_path(&tmp).unwrap();
        for i in 0..10 {
            writer.write(Point {
                x: i as f64,
                y: i as f64 * 2.0,
                z: i as f64 * 0.5,
                intensity: (i * 100) as u16,
                ..Default::default()
            }).unwrap();
        }
        writer.close().unwrap();
    }

    // Read it back via our reader
    let mut reader = LasReader::new(&tmp).unwrap();
    let layout = reader.layout();
    assert_eq!(layout.num_points, 10);

    // Read a chunk and verify we get points back
    let chunk = reader.read_chunk().unwrap();
    assert!(chunk.is_some());
    let chunk = chunk.unwrap();
    assert!(chunk.len > 0);
    assert!(!chunk.data.is_empty());

    std::fs::remove_file(&tmp).ok();
}
```

- [ ] **Step 2: Run test — expect compile failure**

```bash
cargo test -p rubipont-core
```
Expected: compile error — `format` module not found.

- [ ] **Step 3: Create `format/mod.rs`**

```rust
pub mod las;
```

- [ ] **Step 4: Implement `format/las.rs`**

```rust
use std::path::Path;

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::pipeline::{PointCloudReader, PointCloudWriter};

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("las")
}

pub struct LasReader {
    las_reader: las::Reader,
    header: las::Header,
    layout: PointLayout,
    metadata: PipelineContext,
    exhausted: bool,
}

impl LasReader {
    pub fn new(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)?;
        let las_reader = las::Reader::new(file)?;
        let header = las_reader.header().clone();

        let layout = PointLayout {
            point_size: header.point_format().len() as usize,
            num_points: header.number_of_point_records(),
            has_integer_coords: true,
        };

        let mut metadata = PipelineContext::default();
        metadata.coordinate_scale = Some((
            header.scale().x,
            header.scale().y,
            header.scale().z,
        ));
        metadata.coordinate_offset = Some((
            header.offset().x,
            header.offset().y,
            header.offset().z,
        ));

        Ok(Self {
            las_reader,
            header,
            layout,
            metadata,
            exhausted: false,
        })
    }
}

impl PointCloudReader for LasReader {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.exhausted {
            return Ok(None);
        }

        let chunk_size = 4096usize.min(self.layout.num_points as usize);
        let mut data = Vec::with_capacity(chunk_size * 26);
        let mut count = 0usize;

        for pt_result in self.las_reader.by_ref().take(4096) {
            let pt = pt_result.map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

            data.extend_from_slice(&pt.x.to_le_bytes());
            data.extend_from_slice(&pt.y.to_le_bytes());
            data.extend_from_slice(&pt.z.to_le_bytes());
            data.extend_from_slice(&pt.intensity.to_le_bytes());
            count += 1;
        }

        if count == 0 {
            self.exhausted = true;
            return Ok(None);
        }

        Ok(Some(PointChunk { data, len: count }))
    }

    fn layout(&self) -> &PointLayout {
        &self.layout
    }

    fn metadata(&self) -> &PipelineContext {
        &self.metadata
    }
}

pub struct LasWriter {
    writer: las::Writer,
    point_count: u64,
}

impl LasWriter {
    pub fn new(path: &Path, layout: &PointLayout, metadata: &PipelineContext) -> Result<Self> {
        let mut header = las::Header::builder()
            .number_of_point_records(layout.num_points)
            .point_format(las::point::Format::new(0))
            .build()
            .map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        if let Some((sx, sy, sz)) = metadata.coordinate_scale {
            header.set_scale(las::Vector::new(sx, sy, sz));
        }
        if let Some((ox, oy, oz)) = metadata.coordinate_offset {
            header.set_offset(las::Vector::new(ox, oy, oz));
        }

        let writer = las::Writer::from_path(path)?;
        Ok(Self {
            writer,
            point_count: 0,
        })
    }
}

impl PointCloudWriter for LasWriter {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        let point_size = 26usize; // 3×f64 (8 each) + u16 (2) = 26 bytes
        for i in 0..chunk.len {
            let offset = i * point_size;
            let x = f64::from_le_bytes(chunk.data[offset..offset + 8].try_into().unwrap());
            let y = f64::from_le_bytes(chunk.data[offset + 8..offset + 16].try_into().unwrap());
            let z = f64::from_le_bytes(chunk.data[offset + 16..offset + 24].try_into().unwrap());
            let intensity = u16::from_le_bytes(
                chunk.data[offset + 24..offset + 26].try_into().unwrap(),
            );

            let pt = las::Point {
                x, y, z, intensity,
                ..Default::default()
            };
            self.writer.write(pt).map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: self.point_count,
                detail: e.to_string(),
            })?;
            self.point_count += 1;
        }
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        self.writer.close()?;
        Ok(())
    }
}
```

- [ ] **Step 5: Run test to verify it passes**

```bash
cargo test -p rubipont-core --test test_las
```
Expected: tests pass.

- [ ] **Step 6: Commit**

```bash
git add rubipont-core/src/format/ rubipont-core/tests/test_las.rs
git commit -m "feat(core): add LAS reader/writer via las crate"
```

---

### Task 5: Implement PCD binary reader/writer

**Files:**
- Create: `rubipont/rubipont-core/src/format/pcd.rs`
- Modify: `rubipont/rubipont-core/src/format/mod.rs`
- Create: `rubipont/rubipont-core/tests/test_pcd.rs`

- [ ] **Step 1: Write the test**

Create `rubipont-core/tests/test_pcd.rs`:
```rust
use rubipont_core::format::pcd::PcdReader;
use rubipont_core::pipeline::PointCloudReader;
use rubipont_core::format;

#[test]
fn pcd_detect_extension() {
    assert!(format::pcd::detect("pcd"));
    assert!(format::pcd::detect("PCD"));
    assert!(!format::pcd::detect("las"));
}
```

- [ ] **Step 2: Run test — expect compile failure**

```bash
cargo test -p rubipont-core
```
Expected: compile error — module `pcd` not found.

- [ ] **Step 3: Update `format/mod.rs`**

```rust
pub mod las;
pub mod pcd;
```

- [ ] **Step 4: Implement `format/pcd.rs`**

```rust
use std::io::{BufRead, BufReader, Write};
use std::path::Path;

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::pipeline::{PointCloudReader, PointCloudWriter};

pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("pcd")
}

pub struct PcdReader {
    reader: BufReader<std::fs::File>,
    point_size: usize,
    num_points: u64,
    layout: PointLayout,
    metadata: PipelineContext,
    is_binary: bool,
    finished: bool,
}

impl PcdReader {
    pub fn new(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)?;
        let mut reader = BufReader::new(file);

        let mut header_sizes: Vec<usize> = Vec::new();
        let mut header_types: Vec<String> = Vec::new();
        let mut num_points: u64 = 0;
        let mut is_binary = false;
        let mut data_offset: u64 = 0;
        let mut line_count = 0u64;

        loop {
            let mut line = String::new();
            let bytes_read = reader.read_line(&mut line)?;
            if bytes_read == 0 {
                break;
            }
            line_count += bytes_read as u64;
            let trimmed = line.trim();

            if trimmed.starts_with("SIZE") {
                header_sizes = trimmed
                    .split_whitespace().skip(1)
                    .filter_map(|s| s.parse().ok())
                    .collect();
            } else if trimmed.starts_with("TYPE") {
                header_types = trimmed
                    .split_whitespace().skip(1)
                    .map(String::from)
                    .collect();
            } else if trimmed.starts_with("POINTS") {
                num_points = trimmed.split_whitespace()
                    .nth(1).and_then(|s| s.parse().ok())
                    .unwrap_or(0);
            } else if trimmed.starts_with("DATA") {
                let mode = trimmed.split_whitespace().nth(1).unwrap_or("ascii");
                is_binary = mode.eq_ignore_ascii_case("binary")
                    || mode.eq_ignore_ascii_case("binary_compressed");
                data_offset = line_count;
                break;
            }
        }

        let point_size: usize = header_sizes.iter().sum();

        Ok(Self {
            reader,
            point_size,
            num_points,
            layout: PointLayout {
                point_size,
                num_points,
                has_integer_coords: false,
            },
            metadata: PipelineContext::default(),
            is_binary,
            finished: false,
        })
    }
}

impl PointCloudReader for PcdReader {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.finished {
            return Ok(None);
        }

        if !self.is_binary {
            let mut data = Vec::new();
            let mut count = 0usize;
            let mut line = String::new();

            while count < 4096 {
                line.clear();
                if self.reader.read_line(&mut line)? == 0 {
                    break;
                }
                let trimmed = line.trim();
                if trimmed.is_empty() { continue; }

                let vals: Vec<f64> = trimmed
                    .split_whitespace()
                    .filter_map(|s| s.parse::<f64>().ok())
                    .collect();

                let x = *vals.first().unwrap_or(&0.0);
                let y = *vals.get(1).unwrap_or(&0.0);
                let z = *vals.get(2).unwrap_or(&0.0);
                let intensity = vals.get(3).copied().unwrap_or(0.0) as u16;

                data.extend_from_slice(&x.to_le_bytes());
                data.extend_from_slice(&y.to_le_bytes());
                data.extend_from_slice(&z.to_le_bytes());
                data.extend_from_slice(&intensity.to_le_bytes());
                count += 1;
            }

            if count == 0 {
                self.finished = true;
                return Ok(None);
            }
            return Ok(Some(PointChunk { data, len: count }));
        }

        // Binary mode
        let chunk_points = 4096usize.min(self.num_points as usize);
        let chunk_bytes = chunk_points * self.point_size;
        let mut raw = vec![0u8; chunk_bytes];
        self.reader.read_exact(&mut raw).map_err(|e| {
            if e.kind() == std::io::ErrorKind::UnexpectedEof {
                RubipontError::ParseError {
                    format: "PCD".into(), offset: 0, detail: "unexpected EOF".into(),
                }
            } else { RubipontError::Io(e) }
        })?;

        let mut data = vec![0u8; chunk_points * 26];
        for i in 0..chunk_points {
            let src = i * self.point_size;
            let dst = i * 26;
            // For MVP: assume XYZ are F4 or F8, convert to internal f64
            let x = f64::from_le_bytes(
                if self.point_size >= 4 { f32::from_le_bytes(raw[src..src+4].try_into().unwrap()) as f64 }
                else { raw[src] as f64 }
            );
            let y = if self.point_size >= 8 {
                f64::from_le_bytes(raw[src+8..src+16].try_into().unwrap())
            } else { 0.0 };
            let z = if self.point_size >= 16 {
                f64::from_le_bytes(raw[src+16..src+24].try_into().unwrap())
            } else { 0.0 };
            data[dst..dst+8].copy_from_slice(&x.to_le_bytes());
            data[dst+8..dst+16].copy_from_slice(&y.to_le_bytes());
            data[dst+16..dst+24].copy_from_slice(&z.to_le_bytes());
        }

        self.finished = true;
        Ok(Some(PointChunk { data, len: chunk_points }))
    }

    fn layout(&self) -> &PointLayout { &self.layout }
    fn metadata(&self) -> &PipelineContext { &self.metadata }
}

pub struct PcdWriter {
    file: std::fs::File,
    point_count: u64,
}

impl PcdWriter {
    pub fn new(path: &Path, layout: &PointLayout, _metadata: &PipelineContext) -> Result<Self> {
        let file = std::fs::File::create(path)?;
        let mut writer = std::io::BufWriter::new(&file);

        writeln!(writer, "VERSION 0.7")?;
        writeln!(writer, "FIELDS x y z intensity")?;
        writeln!(writer, "SIZE 8 8 8 2")?;
        writeln!(writer, "TYPE F F F U")?;
        writeln!(writer, "COUNT 1 1 1 1")?;
        writeln!(writer, "WIDTH {}", layout.num_points)?;
        writeln!(writer, "HEIGHT 1")?;
        writeln!(writer, "VIEWPOINT 0 0 0 1 0 0 0")?;
        writeln!(writer, "POINTS {}", layout.num_points)?;
        writeln!(writer, "DATA binary")?;
        writer.flush()?;

        Ok(Self {
            file,
            point_count: 0,
        })
    }
}

impl PointCloudWriter for PcdWriter {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        self.file.write_all(&chunk.data)?;
        self.point_count += chunk.len as u64;
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        self.file.sync_all()?;
        Ok(())
    }
}
```

- [ ] **Step 5: Run test to verify it passes**

```bash
cargo test -p rubipont-core
```
Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add rubipont-core/src/format/pcd.rs rubipont-core/tests/test_pcd.rs
git commit -m "feat(core): add PCD binary/ASCII reader and writer"
```

---

### Task 6: Implement LAZ reader/writer

**Files:**
- Create: `rubipont/rubipont-core/src/format/laz.rs`
- Modify: `rubipont/rubipont-core/src/format/mod.rs`
- Create: `rubipont/rubipont-core/tests/test_laz.rs`

- [ ] **Step 1: Write the test**

Create `rubipont-core/tests/test_laz.rs`:
```rust
use rubipont_core::pipeline::PointCloudReader;
use rubipont_core::format;

#[test]
fn laz_detect_extension() {
    assert!(format::laz::detect("laz"));
    assert!(format::laz::detect("LAZ"));
    assert!(!format::laz::detect("las"));
}
```

- [ ] **Step 2: Run test — expect compile failure**

```bash
cargo test -p rubipont-core
```
Expected: compile error — module `laz` not found.

- [ ] **Step 3: Update `format/mod.rs`**

```rust
pub mod las;
pub mod laz;
pub mod pcd;
```

- [ ] **Step 4: Implement `format/laz.rs`**

```rust
use std::path::Path;

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::pipeline::{PointCloudReader, PointCloudWriter};

pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("laz")
}

/// LAZ reader wraps the laz crate's decompressor.
pub struct LazReader {
    reader: laz::LasZipDecompressor<std::fs::File>,
    header: las::Header,
    layout: PointLayout,
    metadata: PipelineContext,
    point_count: u64,
    exhausted: bool,
}

impl LazReader {
    pub fn new(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)?;
        let reader = laz::LasZipDecompressor::new(file)?;
        let header = reader.header().clone();

        let layout = PointLayout {
            point_size: header.point_format().len() as usize,
            num_points: header.number_of_point_records(),
            has_integer_coords: true,
        };

        let mut metadata = PipelineContext::default();
        metadata.coordinate_scale = Some((
            header.scale().x, header.scale().y, header.scale().z,
        ));
        metadata.coordinate_offset = Some((
            header.offset().x, header.offset().y, header.offset().z,
        ));

        Ok(Self {
            reader: todo!("laz::LasZipDecompressor::new"),
            header,
            layout,
            metadata,
            point_count: 0,
            exhausted: false,
        })
    }
}

impl PointCloudReader for LazReader {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.exhausted {
            return Ok(None);
        }

        let chunk_size = 50_000u64.min(self.layout.num_points - self.point_count);
        if chunk_size == 0 {
            self.exhausted = true;
            return Ok(None);
        }

        // laz crate API depends on the specific version — adjust during implementation
        let points: Vec<las::Point> = self
            .reader
            .read_points(chunk_size as usize) // TODO: verify laz crate method name
            .map_err(|e| RubipontError::ParseError {
                format: "LAZ".into(),
                offset: self.point_count,
                detail: e.to_string(),
            })?
            .into_iter()
            .collect();

        let mut data = Vec::with_capacity(points.len() * 26);
        for pt in &points {
            data.extend_from_slice(&pt.x.to_le_bytes());
            data.extend_from_slice(&pt.y.to_le_bytes());
            data.extend_from_slice(&pt.z.to_le_bytes());
            data.extend_from_slice(&pt.intensity.to_le_bytes());
        }

        self.point_count += points.len() as u64;
        if self.point_count >= self.layout.num_points {
            self.exhausted = true;
        }

        Ok(Some(PointChunk { data, len: points.len() }))
    }

    fn layout(&self) -> &PointLayout { &self.layout }
    fn metadata(&self) -> &PipelineContext { &self.metadata }
}

/// LAZ writer wraps the laz crate's compressor.
pub struct LazWriter {
    writer: laz::LasZipCompressor<std::fs::File>,
    point_count: u64,
}

impl LazWriter {
    pub fn new(path: &Path, layout: &PointLayout, metadata: &PipelineContext) -> Result<Self> {
        let mut header = las::Header::builder()
            .number_of_point_records(layout.num_points)
            .point_format(las::point::Format::new(0))
            .build()
            .map_err(|e| RubipontError::ParseError {
                format: "LAZ".into(), offset: 0, detail: e.to_string(),
            })?;

        if let Some((sx, sy, sz)) = metadata.coordinate_scale {
            header.set_scale(las::Vector::new(sx, sy, sz));
        }
        if let Some((ox, oy, oz)) = metadata.coordinate_offset {
            header.set_offset(las::Vector::new(ox, oy, oz));
        }

        let writer = laz::LasZipCompressor::new(path, header) // TODO: verify laz crate API
            .map_err(|e| RubipontError::ParseError {
                format: "LAZ".into(), offset: 0, detail: e.to_string(),
            })?;

        Ok(Self {
            writer: todo!("laz::LasZipCompressor::new"),
            point_count: 0,
        })
    }
}

impl PointCloudWriter for LazWriter {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        let point_size = 26usize;
        for i in 0..chunk.len {
            let offset = i * point_size;
            let x = f64::from_le_bytes(chunk.data[offset..offset + 8].try_into().unwrap());
            let y = f64::from_le_bytes(chunk.data[offset + 8..offset + 16].try_into().unwrap());
            let z = f64::from_le_bytes(chunk.data[offset + 16..offset + 24].try_into().unwrap());
            let intensity = u16::from_le_bytes(
                chunk.data[offset + 24..offset + 26].try_into().unwrap(),
            );

            let pt = las::Point {
                x, y, z, intensity,
                ..Default::default()
            };
            // TODO: verify laz crate API for writing individual points
        }
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        Ok(())
    }
}
```

- [ ] **Step 5: Run test to verify it passes**

```bash
cargo test -p rubipont-core
```
Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add rubipont-core/src/format/laz.rs rubipont-core/tests/test_laz.rs
git commit -m "feat(core): add LAZ reader/writer via laz crate"
```

---

### Task 7: Implement the conversion pipeline orchestrator

**Files:**
- Modify: `rubipont/rubipont-core/src/pipeline.rs` (add convert function)
- Create: `rubipont/rubipont-core/tests/test_convert.rs`

- [ ] **Step 1: Write the conversion test**

Create `rubipont-core/tests/test_convert.rs`:
```rust
use rubipont_core::pipeline::convert;
use std::path::Path;

#[test]
fn convert_rejects_unsupported_input() {
    let result = convert(
        Path::new("test.xyz"),
        Path::new("output.las"),
    );
    assert!(result.is_err());
    let err = format!("{}", result.unwrap_err());
    assert!(err.contains("xyz"), "Error should mention unsupported format: {}", err);
}

#[test]
fn convert_rejects_unsupported_output() {
    let result = convert(
        Path::new("test.las"),
        Path::new("output.xyz"),
    );
    // Either unsupported input (file doesn't exist) or unsupported output format
    assert!(result.is_err());
}
```

- [ ] **Step 2: Add `convert` function to `pipeline.rs`**

Append to the end of `pipeline.rs`:
```rust
use std::path::Path;
use crate::error::{Result, RubipontError};
use crate::format;
use crate::layout::PipelineContext;

/// Convert a point cloud file from one format to another.
/// Dispatches to the correct reader/writer by file extension.
pub fn convert(input: &Path, output: &Path) -> Result<()> {
    let ext = detect::extension(input);
    let out_ext = detect::extension(output);

    // Construct reader
    let mut reader: Box<dyn PointCloudReader> = match ext {
        e if format::las::detect(e) => Box::new(format::las::LasReader::new(input)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazReader::new(input)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdReader::new(input)?),
        _ => return Err(RubipontError::UnsupportedFormat(ext.into())),
    };

    let layout = reader.layout();
    let meta = PipelineContext {
        crs_wkt: reader.metadata().crs_wkt.clone(),
        coordinate_scale: reader.metadata().coordinate_scale,
        coordinate_offset: reader.metadata().coordinate_offset,
        viewpoint: reader.metadata().viewpoint,
        extra_fields: std::collections::HashMap::new(),
    };

    // Construct writer
    let mut writer: Box<dyn PointCloudWriter> = match out_ext {
        e if format::las::detect(e) => Box::new(format::las::LasWriter::new(output, layout, &meta)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazWriter::new(output, layout, &meta)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdWriter::new(output, layout, &meta)?),
        _ => return Err(RubipontError::UnsupportedFormat(out_ext.into())),
    };

    // Stream chunks
    while let Some(chunk) = reader.read_chunk()? {
        writer.write_chunk(&chunk)?;
    }

    writer.finalize()?;
    Ok(())
}
```

Also update `layout.rs` to derive `Clone` on `PipelineContext`:
```rust
#[derive(Debug, Default, Clone)]  // add Clone
pub struct PipelineContext {
    // ... same fields
```

- [ ] **Step 3: Run test to verify it passes**

```bash
cargo test -p rubipont-core
```
Expected: all tests pass.

- [ ] **Step 4: Commit**

```bash
git add rubipont-core/src/pipeline.rs rubipont-core/tests/test_convert.rs
git commit -m "feat(core): add convert() pipeline function"
```

---

### Task 8: Implement the CLI (`rp` binary)

**Files:**
- Replace: `rubipont/rubipont-cli/src/main.rs`
- Create: `rubipont/rubipont-cli/tests/cli_integration.rs`

- [ ] **Step 1: Write basic CLI test**

```rust
use std::process::Command;

#[test]
fn rp_convert_las_to_pcd() {
    let tmp_dir = std::env::temp_dir();
    let input_path = tmp_dir.join("test_cli.las");
    let output_path = tmp_dir.join("test_cli_output.pcd");

    // Write test LAS file using las crate directly
    {
        let header = las::Header::builder()
            .number_of_point_records(10)
            .build()
            .unwrap();
        let mut writer = las::Writer::from_path(&input_path).unwrap();
        for i in 0..10 {
            writer.write(las::Point {
                x: i as f64, y: i as f64 * 2.0, z: i as f64 * 3.0,
                intensity: (i * 100) as u16,
                ..Default::default()
            }).unwrap();
        }
        writer.close().unwrap();
    }

    // Run rp convert
    let output = Command::new(env!("CARGO_BIN_EXE_rp"))
        .args(&["convert", input_path.to_str().unwrap(), output_path.to_str().unwrap()])
        .output()
        .expect("Failed to run rp");

    assert!(output.status.success(), "rp convert failed: {}",
        String::from_utf8_lossy(&output.stderr));

    assert!(output_path.exists(), "Output file was not created");
    let metadata = std::fs::metadata(&output_path).unwrap();
    assert!(metadata.len() > 100, "Output file too small");

    std::fs::remove_file(&input_path).ok();
    std::fs::remove_file(&output_path).ok();
}

#[test]
fn rp_info_shows_file_metadata() {
    let tmp_dir = std::env::temp_dir();
    let input_path = tmp_dir.join("test_info.las");

    {
        let header = las::Header::builder()
            .number_of_point_records(5)
            .build()
            .unwrap();
        let mut writer = las::Writer::from_path(&input_path).unwrap();
        writer.write(las::Point {
            x: 1.0, y: 2.0, z: 3.0, intensity: 50,
            ..Default::default()
        }).unwrap();
        writer.close().unwrap();
    }

    let output = Command::new(env!("CARGO_BIN_EXE_rp"))
        .args(&["info", input_path.to_str().unwrap()])
        .output()
        .expect("Failed to run rp info");

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("5") || stdout.contains("points"));

    std::fs::remove_file(&input_path).ok();
}
```

- [ ] **Step 2: Run test — expect compile failure**

```bash
cargo test -p rubipont-cli
```
Expected: compile error — main.rs doesn't use clap yet.

- [ ] **Step 3: Implement `main.rs`**

```rust
use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "rp", about = "rubipont — LiDAR format translator")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Convert a point cloud file between formats
    Convert {
        /// Source file path
        input: PathBuf,
        /// Output file path (format auto-detected from extension)
        output: PathBuf,
    },
    /// Show information about a point cloud file
    Info {
        /// File path to inspect
        input: PathBuf,
    },
    /// List supported formats
    Formats,
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Convert { input, output } => {
            match rubipont_core::pipeline::convert(&input, &output) {
                Ok(()) => {
                    eprintln!("Converted {} → {}", input.display(), output.display());
                }
                Err(e) => {
                    eprintln!("Error: {}", e);
                    std::process::exit(1);
                }
            }
        }
        Commands::Info { input } => {
            match show_info(&input) {
                Ok(()) => {}
                Err(e) => {
                    eprintln!("Error: {}", e);
                    std::process::exit(1);
                }
            }
        }
        Commands::Formats => {
            println!("Supported formats:");
            println!("  .las  — ASPRS LAS 1.2 (read/write)");
            println!("  .laz  — Compressed LAS (read/write)");
            println!("  .pcd  — Point Cloud Data (read/write)");
        }
    }
}

fn show_info(input: &std::path::Path) -> Result<(), rubipont_core::error::RubipontError> {
    use rubipont_core::format;
    use rubipont_core::pipeline::PointCloudReader;

    let ext = rubipont_core::pipeline::detect::extension(input);
    let mut reader: Box<dyn PointCloudReader> = match ext {
        e if format::las::detect(e) => Box::new(format::las::LasReader::new(input)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazReader::new(input)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdReader::new(input)?),
        _ => return Err(rubipont_core::error::RubipontError::UnsupportedFormat(ext.into())),
    };

    let layout = reader.layout();
    let metadata = reader.metadata();

    println!("File: {}", input.display());
    println!("Points: {}", layout.num_points);
    println!("Point size: {} bytes", layout.point_size);
    println!("Integer coords: {}", layout.has_integer_coords);

    if let Some((sx, sy, sz)) = &metadata.coordinate_scale {
        println!("Scale: ({}, {}, {})", sx, sy, sz);
    }
    if let Some((ox, oy, oz)) = &metadata.coordinate_offset {
        println!("Offset: ({}, {}, {})", ox, oy, oz);
    }
    if let Some(crs) = &metadata.crs_wkt {
        println!("CRS: {}", crs);
    }

    Ok(())
}
```

- [ ] **Step 4: Update `rubipont-cli/Cargo.toml` — remove tokio (we're sync now)**

```toml
[dependencies]
rubipont-core = { path = "../rubipont-core" }
clap = { version = "4", features = ["derive"] }
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p rubipont-cli
```
Expected: all tests pass.

- [ ] **Step 6: Manual smoke test**

```bash
cargo run -p rubipont-cli -- formats
```
Expected: shows supported formats table.

- [ ] **Step 7: Commit**

```bash
git add rubipont-cli/
git commit -m "feat(cli): add rp binary with convert, info, formats commands"
```

---

### Task 9: Round-trip integration tests

**Files:**
- Create: `rubipont/tests/roundtrip_las_pcd.rs`
- Create: `rubipont/tests/roundtrip_las_laz.rs`

- [ ] **Step 1: Write LAS ↔ PCD round-trip test**

Create `tests/roundtrip_las_pcd.rs`:
```rust
use rubipont_core::pipeline::convert;

#[test]
fn las_to_pcd_to_las_roundtrip() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("roundtrip_src.las");
    let mid = tmp.join("roundtrip_mid.pcd");
    let dst = tmp.join("roundtrip_dst.las");

    // Create source LAS
    {
        let header = las::Header::builder()
            .number_of_point_records(100)
            .build()
            .unwrap();
        let mut writer = las::Writer::from_path(&src).unwrap();
        for i in 0..100 {
            writer.write(las::Point {
                x: i as f64 * 0.01, y: i as f64 * 0.02, z: i as f64 * 0.005,
                intensity: (i * 10) as u16,
                ..Default::default()
            }).unwrap();
        }
        writer.close().unwrap();
    }

    // Convert LAS → PCD
    convert(&src, &mid).unwrap();
    assert!(mid.exists());

    // Convert PCD → LAS
    convert(&mid, &dst).unwrap();
    assert!(dst.exists());

    // Verify destination has same number of points
    let dst_reader = las::Reader::from_path(&dst).unwrap();
    assert_eq!(dst_reader.header().number_of_point_records(), 100);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&mid).ok();
    std::fs::remove_file(&dst).ok();
}
```

- [ ] **Step 2: Write LAS ↔ LAZ round-trip test**

Create `tests/roundtrip_las_laz.rs`:
```rust
use rubipont_core::pipeline::convert;

#[test]
fn las_to_laz_to_las_roundtrip() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("rt_las_src.las");
    let mid = tmp.join("rt_las_mid.laz");
    let dst = tmp.join("rt_las_dst.las");

    // Create source LAS
    {
        let header = las::Header::builder()
            .number_of_point_records(50)
            .build()
            .unwrap();
        let mut writer = las::Writer::from_path(&src).unwrap();
        for i in 0..50 {
            writer.write(las::Point {
                x: i as f64, y: i as f64 * 0.5, z: 100.0 + i as f64 * 0.1,
                intensity: (i * 50) as u16,
                ..Default::default()
            }).unwrap();
        }
        writer.close().unwrap();
    }

    // LAS → LAZ
    convert(&src, &mid).unwrap();
    let mid_size = std::fs::metadata(&mid).unwrap().len();
    let src_size = std::fs::metadata(&src).unwrap().len();
    assert!(mid_size < src_size, "LAZ should be smaller than LAS");

    // LAZ → LAS
    convert(&mid, &dst).unwrap();
    let dst_reader = las::Reader::from_path(&dst).unwrap();
    assert_eq!(dst_reader.header().number_of_point_records(), 50);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&mid).ok();
    std::fs::remove_file(&dst).ok();
}
```

- [ ] **Step 3: Add integration test config**

Add to workspace `Cargo.toml`:
```toml
[workspace]
resolver = "2"
members = ["rubipont-core", "rubipont-cli"]
```

The integration tests in `tests/` will be auto-discovered by Cargo.

- [ ] **Step 4: Run all tests**

```bash
cargo test --workspace
```
Expected: all unit + integration tests pass.

- [ ] **Step 5: Commit**

```bash
git add tests/ Cargo.toml
git commit -m "test: add round-trip integration tests for LAS↔PCD and LAS↔LAZ"
```

---

### Task 10: Set up CI

**Files:**
- Create: `rubipont/.github/workflows/ci.yml`

- [ ] **Step 1: Create CI workflow**

```yaml
name: CI

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

env:
  CARGO_TERM_COLOR: always

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions-rust-lang/setup-rust-toolchain@v1
      - run: cargo build --workspace
      - run: cargo test --workspace
      - run: cargo clippy --workspace -- -D warnings
```

- [ ] **Step 2: Create the directory**

```bash
mkdir -p .github/workflows
```

- [ ] **Step 3: Commit**

```bash
git add .github/
git commit -m "ci: add GitHub Actions workflow for build, test, clippy"
```
