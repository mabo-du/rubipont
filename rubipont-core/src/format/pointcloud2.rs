// Shared PointCloud2 message parser used by both MCAP and ROS bag readers.
//
// The ROS 2 `sensor_msgs/PointCloud2` message format is used by both MCAP
// (ROS 2) and ROS 1 bag files.  The only difference between the two is that
// MCAP wraps the message in a 4-byte CDR (Common Data Representation)
// encapsulation header (0x00 0x01 0x00 0x00), while ROS 1 bag messages
// have no wrapper and start directly at the ROS message layer.
//
// This module provides a single `extract_points_from_pointcloud2()` function
// parameterised by `skip_cdr_header` — true for MCAP, false for ROS bag.
// The binary read helpers are private to this module; callers only interact
// through the top-level extraction function.

use crate::error::{Result, RubipontError};

/// Extract (x, y, z, intensity) tuples from a PointCloud2 message.
///
/// * `data` — the raw message bytes (possibly CDR-wrapped).
/// * `skip_cdr_header` — if true, skip a 4‑byte CDR header at the start (MCAP).
/// * `format_name` — used in error messages (e.g., `"MCAP"`, `"ROS bag"`).
pub fn extract_points_from_pointcloud2(
    data: &[u8],
    skip_cdr_header: bool,
    format_name: &str,
) -> Result<Vec<(f64, f64, f64, u16)>> {
    let mut offset = 0usize;

    if skip_cdr_header {
        // CDR (Common Data Representation) LE encapsulation header
        if data.len() < 4 || data[0] != 0x00 || data[1] != 0x01 {
            return Err(RubipontError::ParseError {
                format: format_name.into(),
                offset: 0,
                detail: "Invalid CDR header — expected 0x00 0x01 LE marker".into(),
            });
        }
        offset = 4;
    }

    // --- std_msgs/Header ---
    let _seq = read_u32_le(data, &mut offset, format_name)?;
    let _stamp_sec = read_u32_le(data, &mut offset, format_name)?;
    let _stamp_nsec = read_u32_le(data, &mut offset, format_name)?;
    let _frame_id = read_string(data, &mut offset, format_name)?;

    // --- PointCloud2 metadata ---
    let _height = read_u32_le(data, &mut offset, format_name)?;
    let width = read_u32_le(data, &mut offset, format_name)?;

    // fields array: u32 count + PointField entries
    let field_count = read_u32_le(data, &mut offset, format_name)?;
    let mut fields: Vec<(String, u32, u8, u32)> = Vec::new();
    for _ in 0..field_count {
        let name = read_string(data, &mut offset, format_name)?;
        let field_offset = read_u32_le(data, &mut offset, format_name)?;
        let datatype = read_u8(data, &mut offset, format_name)?;
        let count = read_u32_le(data, &mut offset, format_name)?;
        fields.push((name, field_offset, datatype, count));
    }

    let is_bigendian = read_u8(data, &mut offset, format_name)?;
    let point_step = read_u32_le(data, &mut offset, format_name)?;
    let _row_step = read_u32_le(data, &mut offset, format_name)?;

    // data: u32 length + raw bytes
    let data_len = read_u32_le(data, &mut offset, format_name)? as usize;
    let data_start = offset;

    // is_dense: u8 (may be missing — default to true)
    let is_dense = if offset + data_len < data.len() {
        *data.get(offset + data_len).unwrap_or(&1u8)
    } else {
        1u8
    };

    // Guard: zero point_step would panic in data_len / point_step below
    if point_step == 0 {
        return Err(RubipontError::ParseError {
            format: format_name.into(),
            offset: offset as u64,
            detail: "point_step is zero — corrupt PointCloud2 message".into(),
        });
    }

    // Locate x, y, z, intensity field offsets in the field descriptor array.
    // If x, y, or z is absent, the message is malformed — return a ParseError
    // rather than silently producing (0.0, 0.0, 0.0) points.
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
    let (Some(x_off), Some(y_off), Some(z_off)) = (x_off, y_off, z_off) else {
        return Err(RubipontError::ParseError {
            format: format_name.into(),
            offset: offset as u64,
            detail: "PointCloud2 message missing required x, y, or z field".into(),
        });
    };
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

        // Read XYZ as FLOAT32 (type 7).  When is_bigendian is set, swap
        // the bytes after reading so coordinates are correct regardless
        // of the source platform's endianness.
        let mut x = read_f32_at(pt, x_off).unwrap_or(0.0);
        let mut y = read_f32_at(pt, y_off).unwrap_or(0.0);
        let mut z = read_f32_at(pt, z_off).unwrap_or(0.0);
        if is_bigendian != 0 {
            x = f32::from_bits(x.to_bits().swap_bytes());
            y = f32::from_bits(y.to_bits().swap_bytes());
            z = f32::from_bits(z.to_bits().swap_bytes());
        }

        // When is_dense == 0, skip points whose x, y, or z is NaN
        // (invalid/unobserved measurement per PointCloud2 spec).
        if is_dense == 0 && (x.is_nan() || y.is_nan() || z.is_nan()) {
            continue;
        }

        // Read intensity
        let intensity: u16 = match (intensity_off, intensity_type) {
            (Some(off), Some(7)) => {
                // FLOAT32 — clamp to [0,1] then scale to u16.
                // Without the clamp, intensity values >1.0 (common in
                // some sensors) would overflow the u16 multiplication.
                let raw = read_f32_at(pt, off).unwrap_or(0.0);
                let raw = if is_bigendian != 0 {
                    f32::from_bits(raw.to_bits().swap_bytes())
                } else {
                    raw
                };
                (raw.clamp(0.0, 1.0) * 65535.0) as u16
            }
            (Some(off), Some(4)) => {
                // UINT16 — conditionally swap for big-endian
                let raw = read_u16_at(pt, off).unwrap_or(0);
                if is_bigendian != 0 {
                    raw.swap_bytes()
                } else {
                    raw
                }
            }
            (Some(off), Some(2)) => {
                // UINT8 — single byte, no endian swap needed
                pt.get(off).copied().unwrap_or(0) as u16
            }
            _ => 0,
        };

        result.push((x as f64, y as f64, z as f64, intensity));
    }

    Ok(result)
}

// ---------------------------------------------------------------------------
// Binary read helpers (private — callers use extract_points_from_pointcloud2)
// ---------------------------------------------------------------------------

fn read_u32_le(data: &[u8], offset: &mut usize, format_name: &str) -> Result<u32> {
    if *offset + 4 > data.len() {
        return Err(RubipontError::ParseError {
            format: format_name.into(),
            offset: *offset as u64,
            detail: "Unexpected end of data".into(),
        });
    }
    let val = crate::array::read_u32_unchecked(data, *offset);
    *offset += 4;
    Ok(val)
}

fn read_u8(data: &[u8], offset: &mut usize, format_name: &str) -> Result<u8> {
    if *offset >= data.len() {
        return Err(RubipontError::ParseError {
            format: format_name.into(),
            offset: *offset as u64,
            detail: "Unexpected end of data while reading u8".into(),
        });
    }
    let val = data[*offset];
    *offset += 1;
    Ok(val)
}

fn read_string(data: &[u8], offset: &mut usize, format_name: &str) -> Result<String> {
    let len = read_u32_le(data, offset, format_name)? as usize;
    if *offset + len > data.len() {
        return Err(RubipontError::ParseError {
            format: format_name.into(),
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
