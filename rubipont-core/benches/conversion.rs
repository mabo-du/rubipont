use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::path::Path;

/// Generate a test LAS file with N points.
fn generate_las(path: &Path, num_points: u64) {
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(0).unwrap();
    let header = builder.into_header().unwrap();
    let mut writer = las::Writer::from_path(path, header).unwrap();
    for i in 0u16..num_points as u16 {
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

fn bench_las_to_pcd(c: &mut Criterion) {
    let tmp = std::env::temp_dir();
    let src = tmp.join("bench_las_to_pcd.las");
    let dst = tmp.join("bench_las_to_pcd.pcd");

    generate_las(&src, 10_000);

    c.bench_function("las_to_pcd_10k", |b| {
        b.iter(|| {
            rubipont_core::pipeline::convert(black_box(&src), black_box(&dst), None).unwrap();
            let _ = std::fs::remove_file(&dst);
        });
    });

    std::fs::remove_file(&src).ok();
}

fn bench_las_to_laz(c: &mut Criterion) {
    let tmp = std::env::temp_dir();
    let src = tmp.join("bench_las_to_laz.las");
    let dst = tmp.join("bench_las_to_laz.laz");

    generate_las(&src, 10_000);

    c.bench_function("las_to_laz_10k", |b| {
        b.iter(|| {
            rubipont_core::pipeline::convert(black_box(&src), black_box(&dst), None).unwrap();
            let _ = std::fs::remove_file(&dst);
        });
    });

    std::fs::remove_file(&src).ok();
}

fn bench_laz_to_las(c: &mut Criterion) {
    let tmp = std::env::temp_dir();
    let src_las = tmp.join("bench_laz_src.las");
    let src = tmp.join("bench_laz_src.laz");
    let dst = tmp.join("bench_laz_to_las.las");

    generate_las(&src_las, 10_000);
    rubipont_core::pipeline::convert(&src_las, &src, None).unwrap();
    std::fs::remove_file(&src_las).ok();

    c.bench_function("laz_to_las_10k", |b| {
        b.iter(|| {
            rubipont_core::pipeline::convert(black_box(&src), black_box(&dst), None).unwrap();
            let _ = std::fs::remove_file(&dst);
        });
    });

    std::fs::remove_file(&src).ok();
}

criterion_group!(benches, bench_las_to_pcd, bench_las_to_laz, bench_laz_to_las);
criterion_main!(benches);
