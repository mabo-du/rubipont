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
use crate::transform;

/// Extract the file extension from a path.
pub fn extension(path: &Path) -> &str {
    path.extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
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
    let ext = extension(input);
    let out_ext = extension(output);

    let mut reader: Box<dyn PointCloudReader> = match ext {
        e if format::las::detect(e) => Box::new(format::las::LasReader::new(input)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazReader::new(input)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdReader::new(input)?),
        e if format::e57::detect(e) => Box::new(format::e57::E57ReaderImpl::new(input)?),
        #[cfg(feature = "mcap-io")]
        e if format::mcap::detect(e) => Box::new(format::mcap::McapReader::new(input)?),
        #[cfg(feature = "mcap-io")]
        e if format::bag::detect(e) => Box::new(format::bag::BagReader::new(input)?),
        _ => return Err(RubipontError::UnsupportedFormat(ext.into())),
    };

    let layout = reader.layout().clone();
    let meta = reader.metadata().clone();

    let mut writer: Box<dyn PointCloudWriter> = match out_ext {
        e if format::las::detect(e) => Box::new(format::las::LasWriter::new(output, &layout, &meta)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazWriter::new(output, &layout, &meta)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdWriter::new(output, &layout, &meta)?),
        e if format::e57::detect(e) => Box::new(format::e57::E57WriterImpl::new(output, &layout, &meta)?),
        _ => return Err(RubipontError::UnsupportedFormat(out_ext.into())),
    };

    while let Some(chunk) = reader.read_chunk()? {
        if let Some(tgt) = target_epsg {
            // Reproject points in this chunk
            let src_epsg = transform::source_epsg_from_crs_wkt(meta.crs_wkt.as_deref());
            let ps = layout.point_size;
            let mut data = chunk.data;

            for i in 0..chunk.len {
                let off = i * ps;
                // x, y, z occupy the first 24 bytes of each point
                if off + 24 <= data.len() {
                    let x = f64::from_le_bytes(data[off..off + 8].try_into().unwrap());
                    let y = f64::from_le_bytes(data[off + 8..off + 16].try_into().unwrap());
                    let z = f64::from_le_bytes(data[off + 16..off + 24].try_into().unwrap());

                    if let Ok((tx, ty, tz)) =
                        transform::transform_coords(x, y, z, src_epsg, Some(tgt))
                    {
                        data[off..off + 8].copy_from_slice(&tx.to_le_bytes());
                        data[off + 8..off + 16].copy_from_slice(&ty.to_le_bytes());
                        data[off + 16..off + 24].copy_from_slice(&tz.to_le_bytes());
                    }
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
