use rubipont_core::format::las::LasReader;
use rubipont_core::pipeline::PointCloudReader;
use rubipont_core::format;

#[test]
fn las_detect_extension() {
    assert!(format::las::detect("las"));
    assert!(format::las::detect("LAS"));
    assert!(!format::las::detect("pcd"));
}

#[test]
fn las_reads_points() {
    let tmp = std::env::temp_dir().join("test_roundtrip.las");

    // Write a minimal LAS 1.2 file via the las crate directly
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&tmp, header).unwrap();
        for i in 0..10u16 {
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

    // Read it back via our reader
    let mut reader = LasReader::new(&tmp).unwrap();
    let layout = reader.layout();
    assert_eq!(layout.num_points, 10);

    // Read a chunk and verify we get points back
    let chunk = reader.read_chunk().unwrap();
    assert!(chunk.is_some());
    let chunk = chunk.unwrap();
    assert_eq!(chunk.len, 10);
    assert_eq!(chunk.data.len(), 10 * 26);

    std::fs::remove_file(&tmp).ok();
}
