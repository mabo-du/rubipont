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
