// rubipont-core format translation pipeline

use crate::error::Result;
use crate::layout::{PointChunk, PipelineContext, PointLayout};

/// Trait for reading point clouds in chunks
pub trait PointCloudReader {
    /// Read the next chunk of points
    fn read_chunk(&mut self) -> Result<Option<PointChunk>>;
    /// Get the point layout
    fn layout(&self) -> &PointLayout;
    /// Get pipeline metadata
    fn metadata(&self) -> &PipelineContext;
}

/// Trait for writing point clouds in chunks
pub trait PointCloudWriter {
    /// Write a chunk of points
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()>;
    /// Finalize the write (close files, flush, etc.)
    fn finalize(&mut self) -> Result<()>;
}

use std::path::Path;
use crate::error::RubipontError;
use crate::format;
use crate::array::read_array;
use crate::transform;

/// Extract the file extension from a path.
pub fn extension(path: &Path) -> &str {
    path.extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
}

/// Create a reader for the file at `path` based on its extension.
///
/// Used by `convert()`, `format_info()`, and the Python `info()` entry point.
/// Consolidating dispatch here eliminates the 3‑way match‑on‑extension
/// duplication between the CLI, Python bindings, and pipeline.
fn create_reader(path: &Path) -> std::result::Result<Box<dyn PointCloudReader>, RubipontError> {
    let ext = extension(path);
    let r: Box<dyn PointCloudReader> = match ext {
        e if format::las::detect(e) => Box::new(format::las::LasReader::new(path)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazReader::new(path)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdReader::new(path)?),
        e if format::e57::detect(e) => Box::new(format::e57::E57ReaderImpl::new(path)?),
        #[cfg(feature = "mcap-io")]
        e if format::mcap::detect(e) => Box::new(format::mcap::McapReader::new(path)?),
        #[cfg(feature = "mcap-io")]
        e if format::bag::detect(e) => Box::new(format::bag::BagReader::new(path)?),
        _ => return Err(RubipontError::UnsupportedFormat(ext.into())),
    };
    Ok(r)
}

/// Create a writer for the output path based on its extension.
///
/// Mirror of `create_reader` for the write side.  Keeps format‑extension
/// dispatch in one place so adding a new format doesn't require hunting
/// down multiple match arms.
fn create_writer(
    path: &Path,
    layout: &PointLayout,
    metadata: &PipelineContext,
) -> std::result::Result<Box<dyn PointCloudWriter>, RubipontError> {
    let ext = extension(path);
    let w: Box<dyn PointCloudWriter> = match ext {
        e if format::las::detect(e) => Box::new(format::las::LasWriter::new(path, layout, metadata)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazWriter::new(path, layout, metadata)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdWriter::new(path, layout, metadata)?),
        e if format::e57::detect(e) => Box::new(format::e57::E57WriterImpl::new(path, layout, metadata)?),
        #[cfg(feature = "mcap-io")]
        e if format::mcap::detect(e) => Box::new(format::mcap::McapWriterImpl::new(path, layout, metadata)?),
        _ => return Err(RubipontError::UnsupportedFormat(ext.into())),
    };
    Ok(w)
}

/// Read point cloud metadata from `path` and return a human-readable string.
///
/// Replaces `show_info` / `info` that were duplicated between the CLI and
/// Python bindings.  Callers (CLI prints the string; Python returns it) no
/// longer need to dispatch format readers or format display output themselves.
pub fn format_info(path: &Path) -> std::result::Result<String, RubipontError> {
    let reader = create_reader(path)?;
    let layout = reader.layout();
    let meta = reader.metadata();
    let mut info = String::new();
    info.push_str(&format!("File: {}\n", path.display()));
    info.push_str(&format!("Points: {}\n", layout.num_points));
    info.push_str(&format!("Point size: {} bytes\n", layout.point_size));
    info.push_str(&format!("Integer coords: {}\n", layout.has_integer_coords));
    if let Some((sx, sy, sz)) = &meta.coordinate_scale {
        info.push_str(&format!("Scale: ({}, {}, {})\n", sx, sy, sz));
    }
    if let Some((ox, oy, oz)) = &meta.coordinate_offset {
        info.push_str(&format!("Offset: ({}, {}, {})\n", ox, oy, oz));
    }
    if let Some(crs) = &meta.crs_wkt {
        info.push_str(&format!("CRS: {}\n", crs));
    }
    Ok(info)
}

/// List supported formats with descriptions and read/write capability.
///
/// Each entry has the form `<extension> — <description> (<capability>)`.
/// This is the single source of truth for the `rp formats` CLI command
/// and the Python `rubipont.formats()` function.
pub fn formats_list() -> Vec<&'static str> {
    let mut list = vec![
        ".las  — ASPRS LAS 1.2/1.4 (read/write)",
        ".laz  — Compressed LAS       (read/write)",
        ".pcd  — Point Cloud Data     (read/write)",
        ".e57  — ASTM E57             (read/write)",
    ];
    #[cfg(feature = "mcap-io")]
    {
        list.push(".mcap — ROS 2 MCAP           (read/write)");
        list.push(".bag  — ROS 1 bag            (read)");
    }
    list
}

/// Convert a point cloud file between formats.
///
/// Dispatches reader/writer by file extension.  When `target_epsg` is
/// `Some(…)`, coordinates are reprojected from the source CRS (derived
/// from metadata) to the target EPSG code during the read phase.
pub fn convert(
    input: &Path,
    output: &Path,
    target_epsg: Option<u32>,
) -> std::result::Result<(), RubipontError> {
    let mut reader = create_reader(input)?;

    let layout = reader.layout().clone();
    let meta = reader.metadata().clone();

    let mut writer = create_writer(output, &layout, &meta)?;

    while let Some(chunk) = reader.read_chunk()? {
        if let Some(tgt_epsg) = target_epsg {
            // Reproject points in this chunk
            let src_epsg = transform::source_epsg_from_crs_wkt(meta.crs_wkt.as_deref());
            let ps = layout.point_size;

            // Guard against malformed chunks: data must be large enough for all points
            let expected_len = chunk.len.checked_mul(ps)
                .ok_or_else(|| RubipontError::ParseError {
                    format: "pipeline".into(),
                    offset: 0,
                    detail: "chunk point count overflow".into(),
                })?;
            if chunk.data.len() < expected_len {
                return Err(RubipontError::ParseError {
                    format: "pipeline".into(),
                    offset: 0,
                    detail: format!(
                        "truncated chunk: expected {} bytes for {} points, got {}",
                        expected_len,
                        chunk.len,
                        chunk.data.len()
                    ),
                });
            }

            let mut data = chunk.data;

            for i in 0..chunk.len {
                let offset = i * ps;
                let x = f64::from_le_bytes(read_array(&data, offset)?);
                let y = f64::from_le_bytes(read_array(&data, offset + 8)?);
                let z = f64::from_le_bytes(read_array(&data, offset + 16)?);

                if let Ok((tx, ty, tz)) =
                    transform::transform_coords(x, y, z, src_epsg, Some(tgt_epsg))
                {
                    data[offset..offset + 8].copy_from_slice(&tx.to_le_bytes());
                    data[offset + 8..offset + 16].copy_from_slice(&ty.to_le_bytes());
                    data[offset + 16..offset + 24].copy_from_slice(&tz.to_le_bytes());
                }
            }

            writer.write_chunk(&PointChunk {
                data,
                len: chunk.len,
            })?;
        } else {
            writer.write_chunk(&chunk)?;
        }
    }

    writer.finalize()?;
    Ok(())
}
