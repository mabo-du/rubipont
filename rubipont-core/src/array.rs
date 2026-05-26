/// Utility functions for safe byte array extraction.
/// Replaces `data[a..b].try_into().unwrap()` with checked alternatives.

use crate::error::{Result, RubipontError};

/// Read a fixed-size array from a byte slice at the given offset.
/// Returns a ParseError if the slice is too short.
#[inline]
pub fn read_array<const N: usize>(data: &[u8], offset: usize) -> Result<[u8; N]> {
    if offset + N > data.len() {
        return Err(RubipontError::ParseError {
            format: "internal".into(),
            offset: offset as u64,
            detail: format!("expected {} bytes at offset {}, only {} available", N, offset, data.len()),
        });
    }
    let mut arr = [0u8; N];
    arr.copy_from_slice(&data[offset..offset + N]);
    Ok(arr)
}

/// Read a u32 from a byte slice at the given offset (little-endian).
/// Already bound-checked by the caller; panics only if slice is too short
/// (should be guarded by prior bounds check).
#[inline]
pub fn read_u32_unchecked(data: &[u8], offset: usize) -> u32 {
    let mut buf = [0u8; 4];
    buf.copy_from_slice(&data[offset..offset + 4]);
    u32::from_le_bytes(buf)
}
