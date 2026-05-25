use rubipont_core::pipeline::convert;
use std::path::Path;

#[test]
fn convert_rejects_unsupported_input() {
    let result = convert(Path::new("test.xyz"), Path::new("output.las"));
    assert!(result.is_err());
    let err = format!("{}", result.unwrap_err());
    assert!(err.contains("xyz"), "Error should mention format: {}", err);
}

#[test]
fn convert_rejects_unsupported_output() {
    let result = convert(Path::new("test.las"), Path::new("output.xyz"));
    assert!(result.is_err());
}

#[test]
fn convert_las_to_pcd() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("convert_test_src.las");
    let dst = tmp.join("convert_test_dst.pcd");

    // Write a test LAS file using las crate API
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&src, header).unwrap();
        for i in 0u16..10 {
            writer
                .write_point(las::Point {
                    x: i as f64,
                    y: (i as f64) * 2.0,
                    z: (i as f64) * 0.5,
                    intensity: i * 100,
                    ..Default::default()
                })
                .unwrap();
        }
        writer.close().unwrap();
    }

    // Convert LAS -> PCD via pipeline
    convert(&src, &dst).unwrap();
    assert!(dst.exists(), "PCD output should exist");

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&dst).ok();
}
