use std::collections::HashMap;
use std::path::Path;

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::layout::INTERNAL_POINT_SIZE;
use crate::pipeline::PointCloudReader;

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("bag")
}

pub struct BagReader {
    /// Raw point data as (x, y, z, intensity) stored in a flat Vec.
    /// Each point is 26 bytes: 3 × f64 (24 bytes) + u16 (2 bytes).
    data: Vec<u8>,
    /// Number of points consumed so far (for chunked reading).
    consumed: usize,
    /// Total number of points.
    num_points: usize,
    layout: PointLayout,
    metadata: PipelineContext,
    exhausted: bool,
}

impl BagReader {
    pub fn new(path: &Path) -> Result<Self> {
        use rosbag::{ChunkRecord, MessageRecord, RosBag};

        let bag = RosBag::new(path).map_err(|e| RubipontError::ParseError {
            format: "ROS bag".into(),
            offset: 0,
            detail: format!("Cannot open ROS bag: {}", e),
        })?;

        // Phase 1: collect all connection records → (conn_id → topic)
        let mut conns: HashMap<u32, String> = HashMap::new();
        for rec in bag.chunk_records() {
            let rec = rec.map_err(|e| RubipontError::ParseError {
                format: "ROS bag".into(),
                offset: 0,
                detail: format!("ROS bag chunk record error: {}", e),
            })?;
            if let ChunkRecord::Chunk(chunk) = rec {
                for msg in chunk.messages() {
                    let msg = msg.map_err(|e| RubipontError::ParseError {
                        format: "ROS bag".into(),
                        offset: 0,
                        detail: format!("ROS bag chunk message error: {}", e),
                    })?;
                    if let MessageRecord::Connection(conn) = msg {
                        if conn.topic.contains("points") || conn.topic.contains("lidar") {
                            conns.insert(conn.id, conn.topic.to_string());
                        }
                    }
                }
            }
        }

        // If we found no point cloud connections, try widening the filter
        // on a second pass to catch any topic that might be PointCloud2
        if conns.is_empty() {
            for rec in bag.chunk_records() {
                let rec = rec.map_err(|e| RubipontError::ParseError {
                    format: "ROS bag".into(),
                    offset: 0,
                    detail: format!("ROS bag chunk record error: {}", e),
                })?;
                if let ChunkRecord::Chunk(chunk) = rec {
                    for msg in chunk.messages() {
                        let msg = msg.map_err(|e| RubipontError::ParseError {
                            format: "ROS bag".into(),
                            offset: 0,
                            detail: format!("ROS bag chunk message error: {}", e),
                        })?;
                        if let MessageRecord::Connection(conn) = msg {
                            if conn.tp == "sensor_msgs/PointCloud2" {
                                conns.insert(conn.id, conn.topic.to_string());
                            }
                        }
                    }
                }
            }
        }

        // Phase 2: read messages matching point cloud connections
        let mut data: Vec<u8> = Vec::new();
        let mut total_points: usize = 0;

        for rec in bag.chunk_records() {
            let rec = rec.map_err(|e| RubipontError::ParseError {
                format: "ROS bag".into(),
                offset: 0,
                detail: format!("ROS bag chunk record error: {}", e),
            })?;
            if let ChunkRecord::Chunk(chunk) = rec {
                for msg in chunk.messages() {
                    let msg = msg.map_err(|e| RubipontError::ParseError {
                        format: "ROS bag".into(),
                        offset: 0,
                        detail: format!("ROS bag chunk message error: {}", e),
                    })?;
                    if let MessageRecord::MessageData(msg_data) = msg {
                        // Check if this message's connection is in our interest set
                        if !conns.contains_key(&msg_data.conn_id) {
                            continue;
                        }

                        // ROS 1 bag PointCloud2 has NO 4-byte CDR encapsulation header.
                        // Start parsing at offset 0.
                        let extracted = extract_points_from_pointcloud2(msg_data.data)?;
                        total_points += extracted.len();

                        for (x, y, z, intensity) in extracted {
                            data.extend_from_slice(&x.to_le_bytes());
                            data.extend_from_slice(&y.to_le_bytes());
                            data.extend_from_slice(&z.to_le_bytes());
                            data.extend_from_slice(&intensity.to_le_bytes());
                        }
                    }
                }
            }
        }

        let layout = PointLayout {
            point_size: INTERNAL_POINT_SIZE,
            num_points: total_points as u64,
            has_integer_coords: false,
        };

        Ok(Self {
            data,
            consumed: 0,
            num_points: total_points,
            layout,
            metadata: PipelineContext::default(),
            exhausted: false,
        })
    }
}

impl PointCloudReader for BagReader {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.exhausted {
            return Ok(None);
        }

        let remaining = self.num_points.saturating_sub(self.consumed);
        if remaining == 0 {
            self.exhausted = true;
            return Ok(None);
        }

        let chunk_points = remaining.min(4096);
        let chunk_bytes = chunk_points * INTERNAL_POINT_SIZE;
        let start = self.consumed * INTERNAL_POINT_SIZE;
        let end = start + chunk_bytes;

        let chunk_data = self.data[start..end].to_vec();

        self.consumed += chunk_points;
        if self.consumed >= self.num_points {
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

/// Extract (x, y, z, intensity) tuples from a ROS 1 bag PointCloud2 message.
///
/// ROS 1 bag PointCloud2 has NO 4-byte CDR encapsulation header — parsing
/// starts directly at offset 0.  The field layout is otherwise identical to
/// the ROS 2 version found in MCAP files.
fn extract_points_from_pointcloud2(data: &[u8]) -> Result<Vec<(f64, f64, f64, u16)>> {
    crate::format::pointcloud2::extract_points_from_pointcloud2(data, false, "ROS bag")
}
