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

#[test]
fn las14_version_preserved() {
    let tmp = std::env::temp_dir().join("test_las14.las");

    // Write a LAS 1.4 file via the las crate directly
    {
        let mut builder = las::Builder::from((1, 4));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&tmp, header).unwrap();
        writer
            .write_point(las::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
                intensity: 100,
                ..Default::default()
            })
            .unwrap();
        writer.close().unwrap();
    }

    // Read back via our reader to verify version preservation
    let reader = LasReader::new(&tmp).unwrap();
    let meta = reader.metadata();
    assert_eq!(meta.las_version, Some((1, 4)));

    std::fs::remove_file(&tmp).ok();
}

#[test]
fn las14_wkt_crs_roundtrip() {
    let tmp = std::env::temp_dir().join("test_crs.las");
    let wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\"]]";

    // Write a LAS 1.4 with WKT CRS via the las crate directly
    {
        let mut builder = las::Builder::from((1, 4));
        builder.point_format = las::point::Format::new(0).unwrap();
        let mut header = builder.into_header().unwrap();
        header.set_wkt_crs(wkt.as_bytes().to_vec()).unwrap();

        let mut writer = las::Writer::from_path(&tmp, header).unwrap();
        writer
            .write_point(las::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
                intensity: 100,
                ..Default::default()
            })
            .unwrap();
        writer.close().unwrap();
    }

    // Read via our reader, verify CRS and version are extracted
    let our_reader = LasReader::new(&tmp).unwrap();
    let meta = our_reader.metadata();

    assert!(
        meta.crs_wkt.is_some(),
        "CRS should be extracted from LAS 1.4"
    );
    if let Some(crs) = &meta.crs_wkt {
        assert!(
            crs.contains("WGS 84"),
            "CRS should contain 'WGS 84': {}",
            crs
        );
    }

    assert_eq!(meta.las_version, Some((1, 4)));

    std::fs::remove_file(&tmp).ok();
}
