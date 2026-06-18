use pyo3::prelude::*;
use std::path::Path;

/// Convert a point cloud file between formats.
#[pyfunction]
fn convert(input: String, output: String) -> PyResult<()> {
    rubipont_core::pipeline::convert(Path::new(&input), Path::new(&output), None)
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
}

/// Show information about a point cloud file.
#[pyfunction]
fn info(path: String) -> PyResult<String> {
    rubipont_core::pipeline::format_info(Path::new(&path))
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
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
