use rubipont_core::pipeline;

#[test]
fn e57_write_las_to_e57() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("e57_write_src.las");
    let dst = tmp.join("e57_write_dst.e57");

    // Create source LAS
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

    // Convert LAS -> E57
    pipeline::convert(&src, &dst).unwrap();
    assert!(dst.exists());

    // Read back via e57 reader to verify
    use e57::E57Reader;
    let reader = E57Reader::from_file(&dst).unwrap();
    let pcs = reader.pointclouds();
    assert_eq!(pcs.len(), 1);
    assert_eq!(pcs[0].records, 10);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&dst).ok();
}

#[test]
fn e57_roundtrip_las_e57_las() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("e57_rt_src.las");
    let mid = tmp.join("e57_rt_mid.e57");
    let dst = tmp.join("e57_rt_dst.las");

    // Create source LAS
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

    // LAS -> E57 -> LAS roundtrip
    pipeline::convert(&src, &mid).unwrap();
    pipeline::convert(&mid, &dst).unwrap();

    let dst_header = las::Header::new(&mut std::fs::File::open(&dst).unwrap()).unwrap();
    assert_eq!(dst_header.number_of_points(), 10);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&mid).ok();
    std::fs::remove_file(&dst).ok();
}
