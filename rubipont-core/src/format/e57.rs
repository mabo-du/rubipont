use std::io::{BufReader, Read, Seek};
use std::path::Path;

use e57::{CartesianCoordinate, E57Reader, PointCloud};

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::pipeline::PointCloudReader;

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("e57")
}

pub struct E57ReaderImpl<T: Read + Seek> {
    reader: E57Reader<T>,
    layout: PointLayout,
    metadata: PipelineContext,
    pc_index: usize,
    points_read: u64,
    exhausted: bool,
}

impl<T: Read + Seek> E57ReaderImpl<T> {
    /// Create a new E57 reader from a generic reader.
    pub fn from_reader(reader: T) -> Result<Self> {
        let e57_reader = E57Reader::new(reader).map_err(|e| RubipontError::ParseError {
            format: "E57".into(),
            offset: 0,
            detail: e.to_string(),
        })?;

        Self::from_e57_reader(e57_reader)
    }

    fn from_e57_reader(reader: E57Reader<T>) -> Result<Self> {
        let pointclouds = reader.pointclouds();

        if pointclouds.is_empty() {
            return Err(RubipontError::ParseError {
                format: "E57".into(),
                offset: 0,
                detail: "No point clouds found in E57 file".into(),
            });
        }

        // Warn about additional point clouds beyond the first
        if pointclouds.len() > 1 {
            eprintln!(
                "Warning: E57 file contains {} point clouds; reading only the first one",
                pointclouds.len()
            );
        }

        let pc = &pointclouds[0];
        let num_points = pc.records;

        // Extract CRS metadata if available
        let crs_wkt = reader.coordinate_metadata().map(|s| s.to_string());

        let metadata = PipelineContext {
            coordinate_scale: None,
            coordinate_offset: None,
            crs_wkt,
        };

        let layout = PointLayout {
            point_size: 26, // 3×f64 (24 bytes) + u16 (2 bytes) = 26 bytes internal format
            num_points,
            has_integer_coords: false,
        };

        Ok(Self {
            reader,
            layout,
            metadata,
            pc_index: 0,
            points_read: 0,
            exhausted: false,
        })
    }
}

impl E57ReaderImpl<BufReader<std::fs::File>> {
    /// Open an E57 file from a filesystem path.
    pub fn new(path: &Path) -> Result<Self> {
        let reader = E57Reader::from_file(path).map_err(|e| RubipontError::ParseError {
            format: "E57".into(),
            offset: 0,
            detail: e.to_string(),
        })?;

        Self::from_e57_reader(reader)
    }
}

impl<T: Read + Seek> PointCloudReader for E57ReaderImpl<T> {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.exhausted {
            return Ok(None);
        }

        // Get the first point cloud descriptor
        let pcs = self.reader.pointclouds();
        let pc: &PointCloud = match pcs.get(self.pc_index) {
            Some(pc) => pc,
            None => {
                self.exhausted = true;
                return Ok(None);
            }
        };

        // Create a fresh iterator each time (the iterator borrows self.reader mutably,
        // so it must be created and consumed within this method)
        let mut iter = self
            .reader
            .pointcloud_simple(pc)
            .map_err(|e| RubipontError::ParseError {
                format: "E57".into(),
                offset: self.points_read,
                detail: e.to_string(),
            })?;

        // Skip already-read points
        for _ in 0..self.points_read {
            match iter.next() {
                Some(Ok(_)) => {}
                Some(Err(e)) => {
                    return Err(RubipontError::ParseError {
                        format: "E57".into(),
                        offset: self.points_read,
                        detail: e.to_string(),
                    });
                }
                None => {
                    self.exhausted = true;
                    return Ok(None);
                }
            }
        }

        // Read up to CHUNK_SIZE points
        const CHUNK_SIZE: usize = 4096;
        let mut data = Vec::with_capacity(CHUNK_SIZE * 26);
        let mut count = 0usize;

        for pt_result in iter {
            if count >= CHUNK_SIZE {
                break;
            }

            let pt = pt_result.map_err(|e| RubipontError::ParseError {
                format: "E57".into(),
                offset: self.points_read + count as u64,
                detail: e.to_string(),
            })?;

            // Extract coordinates: pass through NaN for invalid coordinates
            let (x, y, z) = match pt.cartesian {
                CartesianCoordinate::Valid { x, y, z } => (x, y, z),
                CartesianCoordinate::Direction { x, y, z } => (x, y, z),
                CartesianCoordinate::Invalid => (f64::NAN, f64::NAN, f64::NAN),
            };

            // Intensity: normalized 0..1 from e57 crate, scale to u16
            let intensity = match pt.intensity {
                Some(v) => (v.clamp(0.0, 1.0) * 65535.0) as u16,
                None => 0,
            };

            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());
            data.extend_from_slice(&intensity.to_le_bytes());

            count += 1;
        }

        self.points_read += count as u64;

        if count == 0 {
            self.exhausted = true;
            return Ok(None);
        }

        if self.points_read >= pc.records {
            self.exhausted = true;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_extension() {
        assert!(detect("e57"));
        assert!(detect("E57"));
        assert!(detect("E57"));
        assert!(!detect("las"));
        assert!(!detect("pcd"));
    }

    #[test]
    fn detect_no_false_positive() {
        assert!(!detect(""));
        assert!(!detect("e5"));
        assert!(!detect("57"));
    }
}
