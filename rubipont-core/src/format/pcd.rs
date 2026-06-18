use std::io::{BufRead, BufReader, Read, Write};
use std::path::Path;

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::layout::INTERNAL_POINT_SIZE;
use crate::pipeline::{PointCloudReader, PointCloudWriter};

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("pcd")
}

/// Metadata for each field in a PCD file.
#[derive(Debug, Clone)]
struct FieldDef {
    name: String,
    size: usize,
    typ: FieldType,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum FieldType {
    I, // signed integer
    U, // unsigned integer
    F, // float
}

/// Parse a PCD header field value (e.g., "FIELDS x y z" => ["x","y","z"]).
fn parse_header_values(line: &str) -> Vec<String> {
    line.split_whitespace().skip(1).map(|s| s.to_string()).collect()
}

/// Helper: read N bytes from buf and return them as a fixed-size array.
fn read_n<const N: usize>(buf: &[u8]) -> Result<[u8; N]> {
    if buf.len() < N {
        return Err(RubipontError::ParseError {
            format: "PCD".into(), offset: 0,
            detail: format!("Expected {} bytes, got {}", N, buf.len()),
        });
    }
    let mut arr = [0u8; N];
    arr.copy_from_slice(&buf[..N]);
    Ok(arr)
}

/// Parse one f64 from a binary field given its size and type.
fn read_field_as_f64(buf: &[u8], size: usize, typ: FieldType) -> Result<f64> {
    match (typ, size) {
        (FieldType::F, 4) => Ok(f32::from_le_bytes(read_n(buf)?) as f64),
        (FieldType::F, 8) => Ok(f64::from_le_bytes(read_n(buf)?)),
        (FieldType::U, 1) => Ok(buf[0] as f64),
        (FieldType::U, 2) => Ok(u16::from_le_bytes(read_n(buf)?) as f64),
        (FieldType::U, 4) => Ok(u32::from_le_bytes(read_n(buf)?) as f64),
        (FieldType::U, 8) => Ok(u64::from_le_bytes(read_n(buf)?) as f64),
        (FieldType::I, 1) => Ok(buf[0] as i8 as f64),
        (FieldType::I, 2) => Ok(i16::from_le_bytes(read_n(buf)?) as f64),
        (FieldType::I, 4) => Ok(i32::from_le_bytes(read_n(buf)?) as f64),
        (FieldType::I, 8) => Ok(i64::from_le_bytes(read_n(buf)?) as f64),
        _ => Err(RubipontError::ParseError {
            format: "PCD".into(),
            offset: 0,
            detail: format!(
                "unsupported field type for f64 conversion: {:?} size={}",
                typ, size
            ),
        }),
    }
}

/// Read one u16 from a binary field (for intensity).
fn read_field_as_u16(buf: &[u8], size: usize, typ: FieldType) -> Result<u16> {
    match (typ, size) {
        (FieldType::U, 1) => Ok(buf[0] as u16),
        (FieldType::U, 2) => Ok(u16::from_le_bytes(read_n(buf)?)),
        (FieldType::U, 4) => Ok((u32::from_le_bytes(read_n(buf)?) & 0xFFFF) as u16),
        (FieldType::F, 4) => Ok(f32::from_le_bytes(read_n(buf)?) as u16),
        (FieldType::F, 8) => Ok(f64::from_le_bytes(read_n(buf)?) as u16),
        _ => Err(RubipontError::ParseError {
            format: "PCD".into(),
            offset: 0,
            detail: format!(
                "unsupported field type for u16 conversion: {:?} size={}",
                typ, size
            ),
        }),
    }
}

pub struct PcdReader {
    reader: BufReader<std::fs::File>,
    fields: Vec<FieldDef>,
    total_point_size: usize,
    num_points: u64,
    points_read: u64,
    layout: PointLayout,
    metadata: PipelineContext,
    is_binary: bool,
    finished: bool,
}

impl PcdReader {
    pub fn new(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)?;
        let mut reader = BufReader::new(file);

        let mut field_names: Vec<String> = Vec::new();
        let mut field_sizes: Vec<usize> = Vec::new();
        let mut field_types: Vec<FieldType> = Vec::new();
        let mut field_counts: Vec<usize> = Vec::new();
        let mut num_points: u64 = 0;
        let mut is_binary = false;

        loop {
            let mut line = String::new();
            if reader.read_line(&mut line)? == 0 {
                break;
            }
            let trimmed = line.trim();

            if trimmed.starts_with("FIELDS") {
                field_names = parse_header_values(trimmed);
            } else if trimmed.starts_with("SIZE") {
                field_sizes = parse_header_values(trimmed)
                    .iter()
                    .filter_map(|s| s.parse().ok())
                    .collect();
            } else if trimmed.starts_with("TYPE") {
                field_types = parse_header_values(trimmed)
                    .iter()
                    .filter_map(|s| match s.as_str() {
                        "I" => Some(FieldType::I),
                        "U" => Some(FieldType::U),
                        "F" => Some(FieldType::F),
                        _ => None,
                    })
                    .collect();
            } else if trimmed.starts_with("COUNT") {
                field_counts = parse_header_values(trimmed)
                    .iter()
                    .filter_map(|s| s.parse().ok())
                    .collect();
            } else if trimmed.starts_with("POINTS") {
                num_points = trimmed
                    .split_whitespace()
                    .nth(1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0);
            } else if trimmed.starts_with("DATA") {
                let mode = trimmed.split_whitespace().nth(1).unwrap_or("ascii");
                is_binary = mode.eq_ignore_ascii_case("binary")
                    || mode.eq_ignore_ascii_case("binary_compressed");
                break;
            }
        }

        // Build field definitions
        let field_count = field_names.len();
        let fields: Vec<FieldDef> = (0..field_count)
            .map(|i| FieldDef {
                name: field_names.get(i).cloned().unwrap_or_default(),
                size: field_sizes.get(i).copied().unwrap_or(1) * field_counts.get(i).copied().unwrap_or(1),
                typ: field_types.get(i).copied().unwrap_or(FieldType::F),
            })
            .collect();

        let total_point_size: usize = fields.iter().map(|f| f.size).sum();

        Ok(Self {
            reader,
            fields,
            total_point_size,
            num_points,
            points_read: 0,
            layout: PointLayout {
                // Reports the internal chunk stride (26 bytes), not the
                // source PCD record size.  The reader packs points into the
                // internal format regardless of the file's FIELD layout.
                point_size: INTERNAL_POINT_SIZE,
                num_points,
                has_integer_coords: false,
            },
            metadata: PipelineContext::default(),
            is_binary,
            finished: false,
        })
    }

    /// Find the index and definition of a field by name.
    fn find_field(&self, name: &str) -> Option<(usize, &FieldDef)> {
        self.fields.iter().enumerate().find(|(_, f)| f.name == name)
    }
}

impl PointCloudReader for PcdReader {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.finished {
            return Ok(None);
        }

        if !self.is_binary {
            // ASCII mode — parse line by line
            let mut data = Vec::new();
            let mut count = 0usize;
            let mut line = String::new();

            while count < 4096 {
                line.clear();
                if self.reader.read_line(&mut line)? == 0 {
                    break;
                }
                let trimmed = line.trim();
                if trimmed.is_empty() {
                    continue;
                }

                let vals: Vec<f64> = trimmed
                    .split_whitespace()
                    .map(|s| s.parse::<f64>().map_err(|_| RubipontError::ParseError {
                        format: "PCD".into(),
                        offset: self.points_read + count as u64,
                        detail: format!("non-numeric value '{}' in ASCII data", s),
                    }))
                    .collect::<std::result::Result<Vec<_>, _>>()?;

                if vals.len() < 3 {
                    return Err(RubipontError::ParseError {
                        format: "PCD".into(),
                        offset: self.points_read + count as u64,
                        detail: format!(
                            "expected at least 3 fields (x y z), got {}",
                            vals.len()
                        ),
                    });
                }

                let x = vals[0];
                let y = vals[1];
                let z = vals[2];
                // TODO(v0.3.0): fabricated intensity — when no 4th column
                // exists, this synthesises 0u16 which downstream formats
                // interpret as real data.  PointBatch migration replaces
                // this with an explicit optional field (ADR 001).
                let intensity = vals.get(3).copied().unwrap_or(0.0) as u16;

                data.extend_from_slice(&x.to_le_bytes());
                data.extend_from_slice(&y.to_le_bytes());
                data.extend_from_slice(&z.to_le_bytes());
                data.extend_from_slice(&intensity.to_le_bytes());
                count += 1;
            }

            if count == 0 {
                self.finished = true;
                return Ok(None);
            }
            return Ok(Some(PointChunk { data, len: count }));
        }

        // Binary mode
        let remaining = self.num_points.saturating_sub(self.points_read);
        let chunk_points = 4096usize.min(remaining as usize);
        if chunk_points == 0 {
            self.finished = true;
            return Ok(None);
        }

        let chunk_bytes = chunk_points * self.total_point_size;
        let mut raw = vec![0u8; chunk_bytes];
        self.reader.read_exact(&mut raw).map_err(|e| {
            if e.kind() == std::io::ErrorKind::UnexpectedEof {
                RubipontError::ParseError {
                    format: "PCD".into(),
                    offset: self.points_read,
                    detail: "unexpected EOF".into(),
                }
            } else {
                RubipontError::Io(e)
            }
        })?;

        // Locate x, y, z, intensity field offsets within the point record
        let x_info = self.find_field("x");
        let y_info = self.find_field("y");
        let z_info = self.find_field("z");
        let intensity_info = self.find_field("intensity");

        let mut data = vec![0u8; chunk_points * INTERNAL_POINT_SIZE];
        for i in 0..chunk_points {
            let pt_offset = i * self.total_point_size;

            // Parse x
            if let Some((idx, def)) = &x_info {
                let field_offset: usize = self.fields[..*idx].iter().map(|f| f.size).sum();
                let val = read_field_as_f64(&raw[pt_offset + field_offset..], def.size, def.typ)?;
                let start = i * INTERNAL_POINT_SIZE;
                data[start..start + 8].copy_from_slice(&val.to_le_bytes());
            }

            // Parse y
            if let Some((idx, def)) = &y_info {
                let field_offset: usize = self.fields[..*idx].iter().map(|f| f.size).sum();
                let val = read_field_as_f64(&raw[pt_offset + field_offset..], def.size, def.typ)?;
                let start = i * INTERNAL_POINT_SIZE + 8;
                data[start..start + 8].copy_from_slice(&val.to_le_bytes());
            }

            // Parse z
            if let Some((idx, def)) = &z_info {
                let field_offset: usize = self.fields[..*idx].iter().map(|f| f.size).sum();
                let val = read_field_as_f64(&raw[pt_offset + field_offset..], def.size, def.typ)?;
                let start = i * INTERNAL_POINT_SIZE + 16;
                data[start..start + 8].copy_from_slice(&val.to_le_bytes());
            }

            // Parse intensity
            if let Some((idx, def)) = &intensity_info {
                let field_offset: usize = self.fields[..*idx].iter().map(|f| f.size).sum();
                let val = read_field_as_u16(&raw[pt_offset + field_offset..], def.size, def.typ)?;
                let start = i * INTERNAL_POINT_SIZE + 24;
                data[start..start + 2].copy_from_slice(&val.to_le_bytes());
            }
        }

        self.points_read += chunk_points as u64;
        if self.points_read >= self.num_points {
            self.finished = true;
        }

        Ok(Some(PointChunk {
            data,
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

pub struct PcdWriter {
    path: std::path::PathBuf,
    data: Vec<u8>,
    point_count: u64,
}

impl PcdWriter {
    pub fn new(path: &Path, layout: &PointLayout, _metadata: &PipelineContext) -> Result<Self> {
        // TODO(v0.3.0): streaming write — buffering all points in memory
        // before writing the header means a 100M-point file consumes ~2.6GB
        // of RAM before any data lands on disk.  This matches the E57Writer
        // and McapWriter patterns but should be replaced with a seek-back-
        // and-patch approach or a double-pass during the PointBatch migration.
        Ok(Self {
            path: path.to_path_buf(),
            data: Vec::with_capacity(layout.num_points as usize * INTERNAL_POINT_SIZE),
            point_count: 0,
        })
    }
}

impl PointCloudWriter for PcdWriter {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        // TODO(v0.3.0): streaming write — currently buffers all point data
        // in RAM before the header is written.  For 100M-point files this is
        // ~2.6 GB held in memory.  Replace with seek-back-and-patch or a
        // double-pass approach during the PointBatch migration.
        self.data.extend_from_slice(&chunk.data);
        self.point_count += chunk.len as u64;
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        let mut file = std::fs::File::create(&self.path)?;
        writeln!(file, "VERSION 0.7")?;
        writeln!(file, "FIELDS x y z intensity")?;
        writeln!(file, "SIZE 8 8 8 2")?;
        writeln!(file, "TYPE F F F U")?;
        writeln!(file, "COUNT 1 1 1 1")?;
        writeln!(file, "WIDTH {}", self.point_count)?;
        writeln!(file, "HEIGHT 1")?;
        writeln!(file, "VIEWPOINT 0 0 0 1 0 0 0")?;
        writeln!(file, "POINTS {}", self.point_count)?;
        writeln!(file, "DATA binary")?;
        file.write_all(&self.data)?;
        file.sync_all()?;
        Ok(())
    }
}
