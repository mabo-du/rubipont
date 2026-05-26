use rubipont_core::pipeline::convert;

#[test]
fn las_to_pcd_to_las_roundtrip() {
    let tmp = std::env::temp_dir();
    let src = tmp.join("roundtrip_src.las");
    let mid = tmp.join("roundtrip_mid.pcd");
    let dst = tmp.join("roundtrip_dst.las");

    // Create source LAS with varied points
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(0).unwrap();
        let header = builder.into_header().unwrap();
        let mut writer = las::Writer::from_path(&src, header).unwrap();
        for i in 0u16..100 {
            writer
                .write_point(las::Point {
                    x: i as f64 * 0.01,
                    y: i as f64 * 0.02,
                    z: i as f64 * 0.005,
                    intensity: i * 10,
                    ..Default::default()
                })
                .unwrap();
        }
        writer.close().unwrap();
    }

    // Convert LAS -> PCD
    convert(&src, &mid, None).unwrap();
    assert!(mid.exists());

    // Convert PCD -> LAS
    convert(&mid, &dst, None).unwrap();
    assert!(dst.exists());

    // Verify destination has same number of points
    let dst_header =
        las::Header::new(&mut std::fs::File::open(&dst).unwrap()).unwrap();
    assert_eq!(dst_header.number_of_points(), 100);

    std::fs::remove_file(&src).ok();
    std::fs::remove_file(&mid).ok();
    std::fs::remove_file(&dst).ok();
}
