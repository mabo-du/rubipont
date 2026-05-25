// rubipont-core memory layout helpers

/// Layout description for a point cloud in memory
pub struct PointLayout {
    /// Size of a single point in bytes
    pub point_size: usize,
    /// Total number of points
    pub num_points: u64,
    /// Whether coordinates are stored as integers (vs floats)
    pub has_integer_coords: bool,
}

/// A chunk of points read from a reader
pub struct PointChunk {
    /// Raw point data (packed binary)
    pub data: Vec<u8>,
    /// Number of points in this chunk
    pub len: usize,
}

/// Pipeline-level context/metadata carried between stages
#[derive(Default)]
pub struct PipelineContext {
    /// Scale factors for coordinate conversion
    pub coordinate_scale: Option<(f64, f64, f64)>,
    /// Offset for coordinate conversion
    pub coordinate_offset: Option<(f64, f64, f64)>,
}
