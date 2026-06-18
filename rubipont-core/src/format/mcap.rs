use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::layout::INTERNAL_POINT_SIZE;
use crate::pipeline::{PointCloudReader, PointCloudWriter};

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("mcap")
}

pub struct McapReader {
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

impl McapReader {
    pub fn new(path: &Path) -> Result<Self> {
        // Read the full file into memory.  This avoids the SIGBUS risk of
        // memory-mapped I/O (the file could be truncated or modified while
        // being read).  Since all point data is already collected into a
        // Vec<u8> below, the extra allocation for the file bytes is freed
        // after the message stream is consumed.
        let raw_bytes = std::fs::read(path)?;

        // Read the summary section to discover channels
        let summary = mcap::read::Summary::read(&raw_bytes[..])
            .map_err(|e| RubipontError::ParseError {
                format: "MCAP".into(),
                offset: 0,
                detail: format!("Cannot read MCAP summary: {}", e),
            })?
            .unwrap_or_default();

        // Build set of point-cloud channel IDs from the summary
        let point_channel_ids: std::collections::HashSet<u16> = summary
            .channels
            .iter()
            .filter(|(_, ch)| {
                ch.topic.contains("points") || ch.topic.contains("lidar")
            })
            .map(|(id, _)| *id)
            .collect();

        // Read all point data into a flat buffer
        let mut data: Vec<u8> = Vec::new();
        let mut total_points: usize = 0;

        let stream = mcap::read::MessageStream::new(&raw_bytes[..])
            .map_err(|e| RubipontError::ParseError {
                format: "MCAP".into(),
                offset: 0,
                detail: format!("Cannot create MCAP stream: {}", e),
            })?;

        for msg_result in stream {
            let msg = msg_result.map_err(|e| RubipontError::ParseError {
                format: "MCAP".into(),
                offset: 0,
                detail: format!("MCAP read error: {}", e),
            })?;

            // Only process point cloud topics
            if !point_channel_ids.contains(&msg.channel.id) {
                continue;
            }

            let raw_data = msg.data.as_ref();
            let extracted = extract_points_from_pointcloud2(raw_data)?;
            total_points += extracted.len();

            for (x, y, z, intensity) in extracted {
                data.extend_from_slice(&x.to_le_bytes());
                data.extend_from_slice(&y.to_le_bytes());
                data.extend_from_slice(&z.to_le_bytes());
                data.extend_from_slice(&intensity.to_le_bytes());
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

impl PointCloudReader for McapReader {
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

/// Extract (x, y, z, intensity) tuples from a CDR-encoded PointCloud2 message.
fn extract_points_from_pointcloud2(data: &[u8]) -> Result<Vec<(f64, f64, f64, u16)>> {
    crate::format::pointcloud2::extract_points_from_pointcloud2(data, true, "MCAP")
}

// ---------------------------------------------------------------------------
// CDR encoding for PointCloud2 write
// ---------------------------------------------------------------------------

/// Build a CDR-encoded PointCloud2 binary message from internal 26-byte points.
///
/// Fields (internal format → PointCloud2 PointField):
/// - x: f64 at offset 0 → FLOAT64 (type 8)
/// - y: f64 at offset 8 → FLOAT64 (type 8)
/// - z: f64 at offset 16 → FLOAT64 (type 8)
/// - intensity: u16 at offset 24 → UINT16 (type 4)
fn build_pointcloud2_cdr(points: &[u8], num_points: usize) -> Vec<u8> {
    let mut buf = Vec::with_capacity(4 + 1024 + points.len());

    // CDR header (Little Endian encapsulation)
    buf.extend_from_slice(&[0x00, 0x01, 0x00, 0x00]);

    // Header.seq = 0
    buf.extend_from_slice(&0u32.to_le_bytes());
    // Header.stamp.sec = 0
    buf.extend_from_slice(&0u32.to_le_bytes());
    // Header.stamp.nanosec = 0
    buf.extend_from_slice(&0u32.to_le_bytes());
    // Header.frame_id = "map"
    let frame_id = b"map";
    buf.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
    buf.extend_from_slice(frame_id);

    // height = 1 (unordered point cloud)
    buf.extend_from_slice(&1u32.to_le_bytes());
    // width = num_points
    buf.extend_from_slice(&(num_points as u32).to_le_bytes());

    // PointField[] — 4 entries: x, y, z, intensity
    buf.extend_from_slice(&4u32.to_le_bytes());

    // "x" — FLOAT64 @ offset 0
    write_field(&mut buf, b"x", 0, 8, 1);
    // "y" — FLOAT64 @ offset 8
    write_field(&mut buf, b"y", 8, 8, 1);
    // "z" — FLOAT64 @ offset 16
    write_field(&mut buf, b"z", 16, 8, 1);
    // "intensity" — UINT16 @ offset 24
    write_field(&mut buf, b"intensity", 24, 4, 1);

    // is_bigendian = false
    buf.push(0u8);
    // point_step = 26
    buf.extend_from_slice(&26u32.to_le_bytes());
    // row_step = num_points * 26
    buf.extend_from_slice(&((num_points as u32) * 26).to_le_bytes());
    // data: u32 length prefix + raw bytes
    buf.extend_from_slice(&(points.len() as u32).to_le_bytes());
    buf.extend_from_slice(points);
    // is_dense = true
    buf.push(1u8);

    buf
}

/// Write a single PointField into the CDR buffer.
fn write_field(buf: &mut Vec<u8>, name: &[u8], offset: u32, datatype: u8, count: u32) {
    buf.extend_from_slice(&(name.len() as u32).to_le_bytes());
    buf.extend_from_slice(name);
    buf.extend_from_slice(&offset.to_le_bytes());
    buf.push(datatype);
    buf.extend_from_slice(&count.to_le_bytes());
}

// ---------------------------------------------------------------------------
// MCAP writer
// ---------------------------------------------------------------------------

pub struct McapWriterImpl {
    path: PathBuf,
    points: Vec<u8>,
    #[allow(dead_code)]
    layout: PointLayout,
    #[allow(dead_code)]
    metadata: PipelineContext,
    point_count: u64,
}

impl McapWriterImpl {
    pub fn new(path: &Path, layout: &PointLayout, metadata: &PipelineContext) -> Result<Self> {
        Ok(Self {
            path: path.to_path_buf(),
            points: Vec::with_capacity(layout.num_points as usize * 26),
            layout: layout.clone(),
            metadata: metadata.clone(),
            point_count: 0,
        })
    }
}

impl PointCloudWriter for McapWriterImpl {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        self.points.extend_from_slice(&chunk.data);
        self.point_count += chunk.len as u64;
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        let file = std::fs::File::create(&self.path)?;

        // Use NoCompression + no chunks for simplicity.
        let mut writer = mcap::write::WriteOptions::new()
            .compression(None)
            .use_chunks(false)
            .create(file)
            .map_err(|e| RubipontError::ParseError {
                format: "MCAP write".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        // Schema: ROS 2 PointCloud2 message definition
        let schema_def = "# PointCloud2 message definition
std_msgs/Header header
uint32 height
uint32 width
PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
";
        let schema_id = writer
            .add_schema(
                "sensor_msgs/msg/PointCloud2",
                "ros2msg",
                schema_def.as_bytes(),
            )
            .map_err(|e| RubipontError::ParseError {
                format: "MCAP write".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        // Channel: point cloud topic with CDR encoding
        let channel_id = writer
            .add_channel(
                schema_id,
                "/points2",
                "cdr",
                &BTreeMap::<String, String>::new(),
            )
            .map_err(|e| RubipontError::ParseError {
                format: "MCAP write".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        // Build the CDR-encoded PointCloud2 binary blob
        let cdr_data =
            build_pointcloud2_cdr(&self.points, self.point_count as usize);

        let header = mcap::records::MessageHeader {
            channel_id,
            sequence: 0,
            log_time: 0,
            publish_time: 0,
        };

        writer
            .write_to_known_channel(&header, &cdr_data)
            .map_err(|e| RubipontError::ParseError {
                format: "MCAP write".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        writer.finish().map_err(|e| RubipontError::ParseError {
            format: "MCAP write".into(),
            offset: 0,
            detail: e.to_string(),
        })?;

        Ok(())
    }
}
