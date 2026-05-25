use std::process::Command;

#[test]
fn rp_convert_las_to_pcd() {
    let tmp = std::env::temp_dir();
    let input_path = tmp.join("test_cli.las");
    let output_path = tmp.join("test_cli_output.pcd");

    // Write test LAS file
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&input_path, header).unwrap();
        for i in 0u16..10 {
            writer
                .write_point(las::Point {
                    x: i as f64,
                    y: (i as f64) * 2.0,
                    z: (i as f64) * 3.0,
                    intensity: i * 100,
                    ..Default::default()
                })
                .unwrap();
        }
        writer.close().unwrap();
    }

    // Run rp convert
    let output = Command::new(env!("CARGO_BIN_EXE_rp"))
        .args(&[
            "convert",
            input_path.to_str().unwrap(),
            output_path.to_str().unwrap(),
        ])
        .output()
        .expect("Failed to run rp");

    assert!(
        output.status.success(),
        "rp convert failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    assert!(output_path.exists(), "Output file was not created");
    let metadata = std::fs::metadata(&output_path).unwrap();
    assert!(metadata.len() > 100, "Output file too small");

    std::fs::remove_file(&input_path).ok();
    std::fs::remove_file(&output_path).ok();
}

#[test]
fn rp_info_shows_file_metadata() {
    let tmp = std::env::temp_dir();
    let input_path = tmp.join("test_info.las");

    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&input_path, header).unwrap();
        writer
            .write_point(las::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
                intensity: 50,
                ..Default::default()
            })
            .unwrap();
        writer.close().unwrap();
    }

    let output = Command::new(env!("CARGO_BIN_EXE_rp"))
        .args(&["info", input_path.to_str().unwrap()])
        .output()
        .expect("Failed to run rp info");

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("1") || stdout.contains("point"));

    std::fs::remove_file(&input_path).ok();
}

#[test]
fn rp_shows_formats() {
    let output = Command::new(env!("CARGO_BIN_EXE_rp"))
        .args(&["formats"])
        .output()
        .expect("Failed to run rp formats");

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains(".las"));
    assert!(stdout.contains(".laz"));
    assert!(stdout.contains(".pcd"));
}
