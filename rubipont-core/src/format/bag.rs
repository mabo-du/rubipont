use std::collections::HashMap;
use std::path::Path;

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::pipeline::PointCloudReader;

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("bag")
}

/// Internal point size used by rubipont-core: 3×f64 (24 bytes) + u16 (2 bytes)
const INTERNAL_POINT_SIZE: usize = 26;

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
/// starts directly at offset 0. The field layout is otherwise identical to
/// the ROS 2 version found in MCAP files.
fn extract_points_from_pointcloud2(data: &[u8]) -> Result<Vec<(f64, f64, f64, u16)>> {
    let mut offset = 0usize; // No CDR header — start at byte 0

    // Parse std_msgs/Header
    // seq: u32
    let _seq = read_u32_le(data, &mut offset)?;
    // stamp: sec + nsec (2 × u32)
    let _stamp_sec = read_u32_le(data, &mut offset)?;
    let _stamp_nsec = read_u32_le(data, &mut offset)?;
    // frame_id: string (u32 length + UTF-8)
    let _frame_id = read_string(data, &mut offset)?;

    // height: u32
    let _height = read_u32_le(data, &mut offset)?;
    // width: u32
    let width = read_u32_le(data, &mut offset)?;

    // fields array: u32 count + PointField entries
    let field_count = read_u32_le(data, &mut offset)?;
    let mut fields: Vec<(String, u32, u8, u32)> = Vec::new();
    for _ in 0..field_count {
        let name = read_string(data, &mut offset)?;
        let field_offset = read_u32_le(data, &mut offset)?;
        let datatype = read_u8(data, &mut offset)?;
        let count = read_u32_le(data, &mut offset)?;
        fields.push((name, field_offset, datatype, count));
    }

    // is_bigendian: u8
    let _is_bigendian = read_u8(data, &mut offset)?;
    // point_step: u32
    let point_step = read_u32_le(data, &mut offset)?;
    // row_step: u32
    let _row_step = read_u32_le(data, &mut offset)?;

    // data: u32 length + raw bytes
    let data_len = read_u32_le(data, &mut offset)? as usize;
    let data_start = offset;

    // is_dense: u8 (may be missing in some files)
    let _is_dense = if offset + data_len < data.len() {
        1u8
    } else {
        1u8
    };

    // Extract field offsets for x, y, z, intensity
    let x_off = fields
        .iter()
        .find(|(n, _, _, _)| n == "x")
        .map(|(_, o, _, _)| *o as usize);
    let y_off = fields
        .iter()
        .find(|(n, _, _, _)| n == "y")
        .map(|(_, o, _, _)| *o as usize);
    let z_off = fields
        .iter()
        .find(|(n, _, _, _)| n == "z")
        .map(|(_, o, _, _)| *o as usize);
    let intensity_field = fields.iter().find(|(n, _, _, _)| n == "intensity");
    let intensity_off = intensity_field.map(|(_, o, _, _)| *o as usize);
    let intensity_type = intensity_field.map(|(_, _, t, _)| *t);

    let num_points = (data_len / point_step as usize).min(width as usize);
    let mut result = Vec::with_capacity(num_points);

    let blob = &data[data_start..data_start + data_len];
    for i in 0..num_points {
        let pt_start = i * point_step as usize;
        if pt_start + point_step as usize > blob.len() {
            break;
        }
        let pt = &blob[pt_start..pt_start + point_step as usize];

        // Read XYZ as FLOAT32 (type 7)
        let x = x_off
            .and_then(|o| read_f32_at(pt, o))
            .unwrap_or(0.0) as f64;
        let y = y_off
            .and_then(|o| read_f32_at(pt, o))
            .unwrap_or(0.0) as f64;
        let z = z_off
            .and_then(|o| read_f32_at(pt, o))
            .unwrap_or(0.0) as f64;

        // Read intensity
        let intensity: u16 = match (intensity_off, intensity_type) {
            (Some(off), Some(7)) => {
                // FLOAT32 — scale to u16
                (read_f32_at(pt, off).unwrap_or(0.0) * 65535.0) as u16
            }
            (Some(off), Some(4)) => {
                // UINT16
                read_u16_at(pt, off).unwrap_or(0)
            }
            (Some(off), Some(2)) => {
                // UINT8 — widen to u16
                pt.get(off).copied().unwrap_or(0) as u16
            }
            _ => 0,
        };

        result.push((x, y, z, intensity));
    }

    Ok(result)
}

// ---------------------------------------------------------------------------
// Binary read helpers
// ---------------------------------------------------------------------------

fn read_u32_le(data: &[u8], offset: &mut usize) -> Result<u32> {
    if *offset + 4 > data.len() {
        return Err(RubipontError::ParseError {
            format: "ROS bag".into(), offset: *offset as u64,
            detail: "Unexpected end of data".into(),
        });
    }
    let val = crate::array::read_u32_unchecked(data, *offset);
    *offset += 4;
    Ok(val)
}

fn read_u8(data: &[u8], offset: &mut usize) -> Result<u8> {
    if *offset >= data.len() {
        return Err(RubipontError::ParseError {
            format: "ROS bag".into(),
            offset: *offset as u64,
            detail: "Unexpected end of data while reading u8".into(),
        });
    }
    let val = data[*offset];
    *offset += 1;
    Ok(val)
}

fn read_string(data: &[u8], offset: &mut usize) -> Result<String> {
    let len = read_u32_le(data, offset)? as usize;
    if *offset + len > data.len() {
        return Err(RubipontError::ParseError {
            format: "ROS bag".into(),
            offset: *offset as u64,
            detail: "String exceeds data bounds".into(),
        });
    }
    let s = String::from_utf8_lossy(&data[*offset..*offset + len]).to_string();
    *offset += len;
    Ok(s)
}

fn read_f32_at(data: &[u8], offset: usize) -> Option<f32> {
    if offset + 4 > data.len() {
        None
    } else {
        Some(f32::from_le_bytes(data[offset..offset + 4].try_into().ok()?))
    }
}

fn read_u16_at(data: &[u8], offset: usize) -> Option<u16> {
    if offset + 2 > data.len() {
        None
    } else {
        Some(u16::from_le_bytes(data[offset..offset + 2].try_into().ok()?))
    }
}
