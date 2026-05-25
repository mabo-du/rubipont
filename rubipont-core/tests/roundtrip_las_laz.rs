use rubipont_core::pipeline::convert;

#[test]
fn las_to_laz_to_las_roundtrip() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("rt_las_src.las");
    let mid = tmp.join("rt_las_mid.laz");
    let dst = tmp.join("rt_las_dst.las");

    // Create source LAS
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&src, header).unwrap();
        for i in 0u16..50 {
            writer
                .write_point(las::Point {
                    x: i as f64,
                    y: i as f64 * 0.5,
                    z: 100.0 + i as f64 * 0.1,
                    intensity: i * 50,
                    ..Default::default()
                })
                .unwrap();
        }
        writer.close().unwrap();
    }

    // LAS -> LAZ
    convert(&src, &mid).unwrap();
    let mid_size = std::fs::metadata(&mid).unwrap().len();
    // LAZ writes compressed bytes but no LAS header — it will be smaller
    assert!(mid_size > 0, "LAZ output should exist");

    // LAZ -> LAS
    convert(&mid, &dst).unwrap();
    assert!(dst.exists());
    let dst_header =
        las::Header::new(&mut std::fs::File::open(&dst).unwrap()).unwrap();
    assert_eq!(dst_header.number_of_points(), 50);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&mid).ok();
    std::fs::remove_file(&dst).ok();
}
