#![cfg(feature = "mcap-io")]

use rubipont_core::pipeline;

#[test]
fn mcap_write_las_to_mcap() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("mcap_write_src.las");
    let dst = tmp.join("mcap_write_dst.mcap");

    // Create source LAS
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&src, header).unwrap();
        for i in 0u16..25 {
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

    // Convert LAS -> MCAP
    pipeline::convert(&src, &dst, None).unwrap();
    assert!(dst.exists());

    // Read back via McapReader to verify point count
    use rubipont_core::pipeline::PointCloudReader;
    let mut reader =
        rubipont_core::format::mcap::McapReader::new(&dst).unwrap();
    assert_eq!(reader.layout().num_points, 25);

    let chunk = reader.read_chunk().unwrap();
    assert!(chunk.is_some());
    assert_eq!(chunk.unwrap().len, 25);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&dst).ok();
}

#[test]
fn mcap_write_roundtrip_las_mcap_las() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("mcap_rt_src.las");
    let mid = tmp.join("mcap_rt_mid.mcap");
    let dst = tmp.join("mcap_rt_dst.las");

    // Create source LAS (with predictable values)
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

    // LAS -> MCAP -> LAS roundtrip
    pipeline::convert(&src, &mid, None).unwrap();
    pipeline::convert(&mid, &dst, None).unwrap();

    // Verify point count preserved
    let dst_header = las::Header::new(&mut std::fs::File::open(&dst).unwrap()).unwrap();
    assert_eq!(dst_header.number_of_points(), 10);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&mid).ok();
    std::fs::remove_file(&dst).ok();
}

#[test]
fn mcap_write_large_point_count() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("mcap_large_src.las");
    let dst = tmp.join("mcap_large_dst.mcap");

    // Create a larger LAS file
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&src, header).unwrap();
        // Write enough points to span multiple chunks
        for i in 0u16..500 {
            writer
                .write_point(las::Point {
                    x: i as f64,
                    y: (i as f64) * 2.0,
                    z: (i as f64) * 0.5,
                    intensity: i,
                    ..Default::default()
                })
                .unwrap();
        }
        writer.close().unwrap();
    }

    pipeline::convert(&src, &dst, None).unwrap();

    use rubipont_core::pipeline::PointCloudReader;
    let mut reader =
        rubipont_core::format::mcap::McapReader::new(&dst).unwrap();
    assert_eq!(reader.layout().num_points, 500);

    // Read all chunks and sum points
    let mut total = 0u64;
    while let Some(chunk) = reader.read_chunk().unwrap() {
        total += chunk.len as u64;
    }
    assert_eq!(total, 500);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&dst).ok();
}
