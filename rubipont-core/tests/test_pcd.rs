use rubipont_core::layout::{PointChunk, PointLayout, PipelineContext};
use rubipont_core::pipeline::{PointCloudReader, PointCloudWriter};
use rubipont_core::format;

#[test]
fn pcd_detect_extension() {
    assert!(format::pcd::detect("pcd"));
    assert!(format::pcd::detect("PCD"));
    assert!(format::pcd::detect("Pcd"));
    assert!(!format::pcd::detect("las"));
}

#[test]
fn pcd_reads_ascii_points() {
    let tmp = std::env::temp_dir().join("test_pcd_ascii.pcd");

    // Write a PCD ASCII file
    std::fs::write(
        &tmp,
        "VERSION 0.7\n\
         FIELDS x y z intensity\n\
         SIZE 4 4 4 2\n\
         TYPE F F F U\n\
         COUNT 1 1 1 1\n\
         WIDTH 5\n\
         HEIGHT 1\n\
         VIEWPOINT 0 0 0 1 0 0 0\n\
         POINTS 5\n\
         DATA ascii\n\
         1.0 2.0 3.0 100\n\
         4.0 5.0 6.0 200\n\
         7.0 8.0 9.0 300\n\
         -1.0 -2.0 -3.0 400\n\
         0.0 0.0 0.0 0\n",
    )
    .unwrap();

    let mut reader = format::pcd::PcdReader::new(&tmp).unwrap();
    let layout = reader.layout();
    assert_eq!(layout.num_points, 5);

    let chunk = reader.read_chunk().unwrap();
    assert!(chunk.is_some());
    let chunk = chunk.unwrap();
    assert_eq!(chunk.len, 5);
    assert_eq!(chunk.data.len(), 5 * 26);

    // No more chunks
    assert!(reader.read_chunk().unwrap().is_none());

    std::fs::remove_file(&tmp).ok();
}

#[test]
fn pcd_roundtrip_binary() {
    let tmp = std::env::temp_dir().join("test_pcd_binary.pcd");

    let layout = PointLayout {
        point_size: 26,
        num_points: 10,
        has_integer_coords: false,
    };
    let metadata = PipelineContext::default();

    // Build 10 points (26 bytes each = 260 bytes total)
    let mut data = Vec::with_capacity(10 * 26);
    for i in 0u16..10 {
        let x = i as f64;
        let y = (i as f64) * 2.0;
        let z = (i as f64) * 0.5;
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&(i * 100).to_le_bytes());
    }
    let chunk = PointChunk {
        data,
        len: 10,
    };

    // Write via PcdWriter
    {
        let mut writer = format::pcd::PcdWriter::new(&tmp, &layout, &metadata).unwrap();
        writer.write_chunk(&chunk).unwrap();
        writer.finalize().unwrap();
    }

    // Read back via PcdReader
    let mut reader = format::pcd::PcdReader::new(&tmp).unwrap();
    assert_eq!(reader.layout().num_points, 10);

    let read_chunk = reader.read_chunk().unwrap();
    assert!(read_chunk.is_some());
    let read_chunk = read_chunk.unwrap();
    assert_eq!(read_chunk.len, 10);
    assert_eq!(read_chunk.data.len(), 10 * 26);

    // Verify first point
    let x0 = f64::from_le_bytes(read_chunk.data[0..8].try_into().unwrap());
    let y0 = f64::from_le_bytes(read_chunk.data[8..16].try_into().unwrap());
    let z0 = f64::from_le_bytes(read_chunk.data[16..24].try_into().unwrap());
    let i0 = u16::from_le_bytes(read_chunk.data[24..26].try_into().unwrap());
    assert_eq!(x0, 0.0);
    assert_eq!(y0, 0.0);
    assert_eq!(z0, 0.0);
    assert_eq!(i0, 0);

    // Verify last point
    let x9 = f64::from_le_bytes(read_chunk.data[9 * 26..9 * 26 + 8].try_into().unwrap());
    let y9 = f64::from_le_bytes(read_chunk.data[9 * 26 + 8..9 * 26 + 16].try_into().unwrap());
    let z9 = f64::from_le_bytes(read_chunk.data[9 * 26 + 16..9 * 26 + 24].try_into().unwrap());
    let i9 = u16::from_le_bytes(read_chunk.data[9 * 26 + 24..9 * 26 + 26].try_into().unwrap());
    assert_eq!(x9, 9.0);
    assert_eq!(y9, 18.0);
    assert_eq!(z9, 4.5);
    assert_eq!(i9, 900);

    // No more chunks
    assert!(reader.read_chunk().unwrap().is_none());

    std::fs::remove_file(&tmp).ok();
}
