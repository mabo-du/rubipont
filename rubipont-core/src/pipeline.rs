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

/// Extract the file extension from a path.
pub fn extension(path: &Path) -> &str {
    path.extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
}

/// Convert a point cloud file between formats.
/// Dispatches reader/writer by file extension.
pub fn convert(input: &Path, output: &Path) -> std::result::Result<(), RubipontError> {
    let ext = extension(input);
    let out_ext = extension(output);

    let mut reader: Box<dyn PointCloudReader> = match ext {
        e if format::las::detect(e) => Box::new(format::las::LasReader::new(input)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazReader::new(input)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdReader::new(input)?),
        e if format::e57::detect(e) => Box::new(format::e57::E57ReaderImpl::new(input)?),
        e if format::mcap::detect(e) => Box::new(format::mcap::McapReader::new(input)?),
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
        writer.write_chunk(&chunk)?;
    }

    writer.finalize()?;
    Ok(())
}
