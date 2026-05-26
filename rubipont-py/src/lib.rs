use pyo3::prelude::*;
use std::path::Path;

/// Convert a point cloud file between formats.
#[pyfunction]
fn convert(input: String, output: String) -> PyResult<()> {
    rubipont_core::pipeline::convert(Path::new(&input), Path::new(&output))
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
}

/// Show information about a point cloud file.
#[pyfunction]
fn info(path: String) -> PyResult<String> {
    use rubipont_core::pipeline::PointCloudReader;
    use rubipont_core::format;
    use rubipont_core::pipeline::extension;

    let input = Path::new(&path);
    let ext = extension(input);

    let reader: Box<dyn PointCloudReader> = match ext {
        e if format::las::detect(e) => Box::new(
            format::las::LasReader::new(input)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?,
        ),
        e if format::laz::detect(e) => Box::new(
            format::laz::LazReader::new(input)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?,
        ),
        e if format::pcd::detect(e) => Box::new(
            format::pcd::PcdReader::new(input)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?,
        ),
        e if format::e57::detect(e) => Box::new(
            format::e57::E57ReaderImpl::new(input)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?,
        ),
        #[cfg(feature = "mcap-io")]
        e if format::mcap::detect(e) => Box::new(
            format::mcap::McapReader::new(input)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?,
        ),
        _ => {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Unsupported format: {}",
                ext
            )))
        }
    };

    let layout = reader.layout();
    let meta = reader.metadata();

    let mut info = String::new();
    info.push_str(&format!("File: {}\n", path));
    info.push_str(&format!("Points: {}\n", layout.num_points));
    info.push_str(&format!("Point size: {} bytes\n", layout.point_size));
    info.push_str(&format!("Integer coords: {}\n", layout.has_integer_coords));
    if let Some((sx, sy, sz)) = &meta.coordinate_scale {
        info.push_str(&format!("Scale: ({}, {}, {})\n", sx, sy, sz));
    }
    if let Some((ox, oy, oz)) = &meta.coordinate_offset {
        info.push_str(&format!("Offset: ({}, {}, {})\n", ox, oy, oz));
    }
    if let Some(crs) = &meta.crs_wkt {
        info.push_str(&format!("CRS: {}\n", crs));
    }

    Ok(info)
}

/// List supported formats.
#[pyfunction]
fn formats() -> Vec<String> {
    vec![
        ".las  — ASPRS LAS 1.2 (read/write)".into(),
        ".laz  — Compressed LAS (read/write)".into(),
        ".pcd  — Point Cloud Data (read/write)".into(),
        ".e57  — ASTM E57 (read/write)".into(),
        #[cfg(feature = "mcap-io")]
        ".mcap — ROS 2 MCAP (read)".into(),
    ]
}

/// Python module definition.
#[pymodule]
fn rubipont(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(convert, m)?)?;
    m.add_function(wrap_pyfunction!(info, m)?)?;
    m.add_function(wrap_pyfunction!(formats, m)?)?;
    Ok(())
}
