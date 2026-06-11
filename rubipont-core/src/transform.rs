// rubipont-core coordinate reference system transformation

use crate::error::{Result, RubipontError};

#[cfg(feature = "proj")]
use proj::Coord;

/// Extract an EPSG authority code from WKT CRS metadata.
///
/// Parses `AUTHORITY["EPSG","XXXX"]` patterns and falls back to
/// known datum names (e.g. "WGS 84" → 4326).
pub fn source_epsg_from_crs_wkt(crs_wkt: Option<&str>) -> Option<u32> {
    let crs = crs_wkt?;

    // Look for AUTHORITY["EPSG","XXXX"] in the WKT string
    if let Some(pos) = crs.find("AUTHORITY") {
        if let Some(rest) = crs.get(pos..) {
            // Match the pattern: AUTHORITY["EPSG","<code>"]
            if let Some(start) = rest.find("\"EPSG\",\"") {
                let num_start = start + 9; // skip past "\"EPSG\",\""
                // SAFETY: find("\"EPSG\",\"") above confirmed the substring
                // exists at `start`, so num_start is within bounds of `rest`
                #[allow(unused_unsafe)]
                let remainder = unsafe { rest.get_unchecked(num_start..) };
                if let Some(end) = remainder.find('\"') {
                    if let Ok(code) = remainder[..end].parse::<u32>() {
                        return Some(code);
                    }
                }
            }
        }
    }

    // Fallback: common well-known datums
    if crs.contains("WGS 84") || crs.contains("WGS_1984") {
        return Some(4326);
    }

    None
}

/// Transform a single (x, y) coordinate from source CRS to target CRS.
///
/// The z (height) component is passed through unchanged because the
/// underlying `proj` crate only exposes 2-D coordinate transformation
/// via the `Coord` trait.
///
/// When source and target are the same (or either is `None`) the input
/// coordinates are returned unchanged.
///
/// This function requires the `proj` Cargo feature.  When the feature is
/// disabled and a real transformation would be needed, a
/// `RubipontError::PrecisionLoss` error is returned.
#[cfg(feature = "proj")]
pub fn transform_coords(
    x: f64,
    y: f64,
    z: f64,
    source_epsg: Option<u32>,
    target_epsg: Option<u32>,
) -> Result<(f64, f64, f64)> {
    match (source_epsg, target_epsg) {
        (Some(src), Some(tgt)) if src != tgt => {
            let from = proj::Proj::new_known_crs(
                &format!("EPSG:{src}"),
                &format!("EPSG:{tgt}"),
                None,
            )
            .map_err(|e| {
                RubipontError::PrecisionLoss(format!(
                    "Cannot create CRS transform from {src} to {tgt}: {e}"
                ))
            })?;

            // The proj crate Coord trait only covers 2D tuples.
            // Transform (x, y) and keep z (vertical) unchanged — most
            // common CRS reprojections are horizontal-only.
            let transformed = from.convert((x, y)).map_err(|e| {
                RubipontError::PrecisionLoss(format!("CRS transform failed: {e}"))
            })?;

            Ok((transformed.x(), transformed.y(), z))
        }
        // Same CRS or missing — no transform
        _ => Ok((x, y, z)),
    }
}

/// Stub: returns an error if a real transform is needed and the `proj`
/// feature is not enabled.
#[cfg(not(feature = "proj"))]
pub fn transform_coords(
    x: f64,
    y: f64,
    z: f64,
    source_epsg: Option<u32>,
    target_epsg: Option<u32>,
) -> Result<(f64, f64, f64)> {
    match (source_epsg, target_epsg) {
        (Some(src), Some(tgt)) if src != tgt => Err(RubipontError::PrecisionLoss(
            "CRS transformation requires the 'proj' feature (enable with: --features proj)".into(),
        )),
        _ => Ok((x, y, z)),
    }
}
