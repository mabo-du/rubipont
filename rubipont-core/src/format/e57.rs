use std::io::{Read, Seek};
use std::path::Path;

use e57::{CartesianCoordinate, E57Reader};

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::layout::INTERNAL_POINT_SIZE;
use crate::pipeline::{PointCloudReader, PointCloudWriter};
use crate::array::read_array;

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("e57")
}

pub struct E57ReaderImpl {
    /// All points buffered in the internal 26-byte format
    data: Vec<u8>,
    /// Bytes consumed so far (for chunked reading)
    consumed: usize,
    layout: PointLayout,
    metadata: PipelineContext,
    exhausted: bool,
}

impl E57ReaderImpl {
    /// Open an E57 file from a filesystem path.
    pub fn new(path: &Path) -> Result<Self> {
        let reader = E57Reader::from_file(path).map_err(|e| RubipontError::ParseError {
            format: "E57".into(),
            offset: 0,
            detail: e.to_string(),
        })?;

        Self::from_e57_reader(reader)
    }

    /// Create a new E57 reader from a generic reader.
    pub fn from_reader(reader: impl Read + Seek) -> Result<Self> {
        let e57_reader = E57Reader::new(reader).map_err(|e| RubipontError::ParseError {
            format: "E57".into(),
            offset: 0,
            detail: e.to_string(),
        })?;

        Self::from_e57_reader(e57_reader)
    }

    fn from_e57_reader(mut reader: E57Reader<impl Read + Seek>) -> Result<Self> {
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

        // Buffer all points eagerly.  The e57 crate's iterator borrows the
        // reader mutably, making self-referential iterator storage impossible
        // in safe Rust.  Rather than re-creating the iterator and re-skipping
        // every point on each read_chunk() call (O(n²) for the full file),
        // we read every point once and store the internal-format bytes.
        let mut data = Vec::with_capacity(num_points as usize * INTERNAL_POINT_SIZE);
        let mut iter = reader
            .pointcloud_simple(pc)
            .map_err(|e| RubipontError::ParseError {
                format: "E57".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        for pt_result in &mut iter {
            let pt = pt_result.map_err(|e| RubipontError::ParseError {
                format: "E57".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

            // Extract coordinates: pass through NaN for invalid coordinates
            let (x, y, z) = match pt.cartesian {
                CartesianCoordinate::Valid { x, y, z } => (x, y, z),
                CartesianCoordinate::Direction { x, y, z } => (x, y, z),
                CartesianCoordinate::Invalid => (f64::NAN, f64::NAN, f64::NAN),
            };

            // Intensity: normalized 0..1 from e57 crate, scale to u16.
            // TODO(v0.3.0): fabricated intensity — when the E57 point has
            // no intensity value, this produces 0u16 which is
            // indistinguishable from a measured zero.  PointBatch
            // migration replaces this with an explicit optional field
            // so absence means "not measured" (ADR 001).
            let intensity = match pt.intensity {
                Some(v) => (v.clamp(0.0, 1.0) * 65535.0) as u16,
                None => 0,
            };

            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());
            data.extend_from_slice(&intensity.to_le_bytes());
        }

        let total_points = data.len() / INTERNAL_POINT_SIZE;

        let layout = PointLayout {
            point_size: INTERNAL_POINT_SIZE,
            num_points: total_points as u64,
            has_integer_coords: false,
        };

        Ok(Self {
            data,
            consumed: 0,
            layout,
            metadata: PipelineContext {
                crs_wkt,
                ..Default::default()
            },
            exhausted: false,
        })
    }
}

impl PointCloudReader for E57ReaderImpl {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.exhausted {
            return Ok(None);
        }

        let remaining_bytes = self.data.len().saturating_sub(self.consumed);
        if remaining_bytes == 0 {
            self.exhausted = true;
            return Ok(None);
        }

        const CHUNK_SIZE: usize = 4096;
        let chunk_points = (remaining_bytes / INTERNAL_POINT_SIZE).min(CHUNK_SIZE);
        let chunk_bytes = chunk_points * INTERNAL_POINT_SIZE;
        let start = self.consumed;
        let end = start + chunk_bytes;

        let chunk_data = self.data[start..end].to_vec();

        self.consumed += chunk_bytes;
        if self.consumed >= self.data.len() {
            self.exhausted = true;
        }

        Ok(Some(PointChunk {
            data: chunk_data,
            len: chunk_points,
        }))
    }

    fn layout(&self) -> &PointLayout {
        &self.layout
    }

    fn metadata(&self) -> &PipelineContext {
        &self.metadata
    }
}

/// Writes point clouds to E57 format by buffering points and writing them
/// on finalize via the e57 crate's E57Writer and PointCloudWriter types.
pub struct E57WriterImpl {
    path: std::path::PathBuf,
    #[allow(dead_code)]
    layout: PointLayout,
    metadata: PipelineContext,
    point_count: u64,
    /// Buffered points: (x, y, z, intensity_u16)
    points: Vec<(f64, f64, f64, u16)>,
}

impl E57WriterImpl {
    /// Create a new E57 writer that buffers points until finalize.
    pub fn new(path: &Path, layout: &PointLayout, metadata: &PipelineContext) -> Result<Self> {
        Ok(Self {
            path: path.to_path_buf(),
            layout: layout.clone(),
            metadata: metadata.clone(),
            point_count: 0,
            points: Vec::with_capacity(layout.num_points as usize),
        })
    }
}

impl PointCloudWriter for E57WriterImpl {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        for i in 0..chunk.len {
            let offset = i * INTERNAL_POINT_SIZE;
            let x = f64::from_le_bytes(read_array(&chunk.data, offset)?);
            let y = f64::from_le_bytes(read_array(&chunk.data, offset + 8)?);
            let z = f64::from_le_bytes(read_array(&chunk.data, offset + 16)?);
            let intensity = u16::from_le_bytes(read_array(&chunk.data, offset + 24)?);
            self.points.push((x, y, z, intensity));
        }
        self.point_count += chunk.len as u64;
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        use e57::{E57Writer, Record, RecordDataType, RecordName, RecordValue};

        if self.points.is_empty() {
            return Ok(());
        }

        // Create E57 writer — from_file handles file creation with read+write+seek
        let mut writer = E57Writer::from_file(&self.path, "rubipont")
            .map_err(|e| RubipontError::ParseError {
                format: "E57".into(),
                offset: 0,
                detail: format!("Cannot create E57 writer: {}", e),
            })?;

        // Set CRS if available
        if let Some(crs) = &self.metadata.crs_wkt {
            writer.set_coordinate_metadata(Some(crs.clone()));
        }

        // Define prototype: Cartesian X/Y/Z as f64, Intensity as unit f32
        let prototype = vec![
            Record { name: RecordName::CartesianX, data_type: RecordDataType::F64 },
            Record { name: RecordName::CartesianY, data_type: RecordDataType::F64 },
            Record { name: RecordName::CartesianZ, data_type: RecordDataType::F64 },
            Record { name: RecordName::Intensity, data_type: RecordDataType::UNIT_F32 },
        ];

        let mut pc_writer = writer.add_pointcloud("pc0", prototype)
            .map_err(|e| RubipontError::ParseError {
                format: "E57".into(),
                offset: 0,
                detail: format!("Cannot create point cloud writer: {}", e),
            })?;

        // Write all buffered points — convert u16 intensity to normalized f32 (0..1)
        for (x, y, z, intensity) in &self.points {
            let values = vec![
                RecordValue::Double(*x),
                RecordValue::Double(*y),
                RecordValue::Double(*z),
                RecordValue::Single(*intensity as f32 / 65535.0),
            ];
            pc_writer.add_point(values).map_err(|e| RubipontError::ParseError {
                format: "E57".into(),
                offset: self.point_count,
                detail: format!("Cannot write point: {}", e),
            })?;
        }

        pc_writer.finalize().map_err(|e| RubipontError::ParseError {
            format: "E57".into(),
            offset: self.point_count,
            detail: format!("Cannot finalize point cloud: {}", e),
        })?;

        writer.finalize().map_err(|e| RubipontError::ParseError {
            format: "E57".into(),
            offset: self.point_count,
            detail: format!("Cannot finalize E57 file: {}", e),
        })?;

        Ok(())
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
