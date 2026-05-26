// rubipont-core memory layout helpers

/// Layout description for a point cloud in memory
#[derive(Clone)]
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
#[derive(Default, Clone)]
pub struct PipelineContext {
    /// Scale factors for coordinate conversion
    pub coordinate_scale: Option<(f64, f64, f64)>,
    /// Offset for coordinate conversion
    pub coordinate_offset: Option<(f64, f64, f64)>,
    /// CRS (Coordinate Reference System) WKT string (from E57 or LAS 1.4 files)
    pub crs_wkt: Option<String>,
    /// LAS format version (major, minor) — set when source is LAS
    pub las_version: Option<(u8, u8)>,
}
