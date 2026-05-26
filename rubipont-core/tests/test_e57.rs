use rubipont_core::pipeline::PointCloudReader;
use rubipont_core::format;

#[test]
fn e57_detect_extension() {
    assert!(format::e57::detect("e57"));
    assert!(format::e57::detect("E57"));
    assert!(!format::e57::detect("las"));
    assert!(!format::e57::detect("pcd"));
}

#[test]
fn e57_reads_points() {
    use e57::{
        E57Writer, Record, RecordDataType, RecordName, RecordValue,
    };

    let tmp = std::env::temp_dir().join("test_e57_reader.e57");

    // Write a small E57 file with known points using the e57 writer API
    let num_points = 10u64;
    {
        let mut writer = E57Writer::from_file(&tmp, "test-guid-0000-0000-0000-000000000001")
            .expect("Failed to create E57 writer");

        // Prototype: Cartesian X, Y, Z as f64, intensity as unit f32
        let prototype = vec![
            Record {
                name: RecordName::CartesianX,
                data_type: RecordDataType::F64,
            },
            Record {
                name: RecordName::CartesianY,
                data_type: RecordDataType::F64,
            },
            Record {
                name: RecordName::CartesianZ,
                data_type: RecordDataType::F64,
            },
            Record {
                name: RecordName::Intensity,
                data_type: RecordDataType::UNIT_F32,
            },
        ];

        let mut pc_writer = writer
            .add_pointcloud("pc-guid-0000-0000-0000-000000000001", prototype)
            .expect("Failed to create point cloud writer");

        for i in 0u16..num_points as u16 {
            let x = i as f64;
            let y = (i as f64) * 2.0;
            let z = (i as f64) * 0.5;
            let intensity = if i % 2 == 0 { 0.5 } else { 1.0 };

            let values = vec![
                RecordValue::Double(x),
                RecordValue::Double(y),
                RecordValue::Double(z),
                RecordValue::Single(intensity),
            ];
            pc_writer
                .add_point(values)
                .expect("Failed to add point");
        }

        pc_writer
            .finalize()
            .expect("Failed to finalize point cloud");
        writer.finalize().expect("Failed to finalize E57 writer");
    }

    // Read it back via our reader
    let mut reader =
        format::e57::E57ReaderImpl::new(&tmp).expect("Failed to open E57 file");
    let layout = reader.layout();
    assert_eq!(layout.num_points, num_points);
    assert_eq!(layout.point_size, 26);

    // Read a chunk
    let chunk = reader.read_chunk().expect("Failed to read chunk");
    assert!(
        chunk.is_some(),
        "Expected at least one chunk, got None"
    );
    let chunk = chunk.unwrap();
    assert_eq!(chunk.len, num_points as usize);
    assert_eq!(chunk.data.len(), num_points as usize * 26);

    // Verify first point
    let x0 = f64::from_le_bytes(chunk.data[0..8].try_into().unwrap());
    let y0 = f64::from_le_bytes(chunk.data[8..16].try_into().unwrap());
    let z0 = f64::from_le_bytes(chunk.data[16..24].try_into().unwrap());
    let i0 = u16::from_le_bytes(chunk.data[24..26].try_into().unwrap());
    assert_eq!(x0, 0.0, "x0 mismatch");
    assert_eq!(y0, 0.0, "y0 mismatch");
    assert_eq!(z0, 0.0, "z0 mismatch");
    assert_eq!(i0, 32767, "i0 = (0.5 * 65535) as u16 = 32767 (truncation)");

    // Verify last point
    let last = (num_points - 1) as usize;
    let offset = last * 26;
    let x9 = f64::from_le_bytes(chunk.data[offset..offset + 8].try_into().unwrap());
    let y9 = f64::from_le_bytes(
        chunk.data[offset + 8..offset + 16]
            .try_into()
            .unwrap(),
    );
    let z9 = f64::from_le_bytes(
        chunk.data[offset + 16..offset + 24]
            .try_into()
            .unwrap(),
    );
    let i9 = u16::from_le_bytes(
        chunk.data[offset + 24..offset + 26]
            .try_into()
            .unwrap(),
    );
    assert_eq!(x9, 9.0, "x9 mismatch");
    assert_eq!(y9, 18.0);
    assert_eq!(z9, 4.5);
    assert_eq!(i9, 65535, "i9 = 1.0 * 65535");

    // No more chunks
    assert!(
        reader.read_chunk().unwrap().is_none(),
        "Expected no more chunks"
    );

    std::fs::remove_file(&tmp).ok();
}

#[test]
fn e57_crs_metadata() {
    use e57::{
        E57Writer, Record, RecordDataType, RecordName, RecordValue,
    };

    let tmp = std::env::temp_dir().join("test_e57_crs.e57");

    {
        let mut writer = E57Writer::from_file(&tmp, "test-guid-crs-0000-0000-0001")
            .expect("Failed to create E57 writer");

        writer.set_coordinate_metadata(Some(
            r#"GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563]]]"#.to_string(),
        ));

        let prototype = vec![
            Record {
                name: RecordName::CartesianX,
                data_type: RecordDataType::F64,
            },
            Record {
                name: RecordName::CartesianY,
                data_type: RecordDataType::F64,
            },
            Record {
                name: RecordName::CartesianZ,
                data_type: RecordDataType::F64,
            },
        ];

        let mut pc_writer = writer
            .add_pointcloud("pc-guid-crs-0000-0000-0002", prototype)
            .expect("Failed to create point cloud writer");

        pc_writer
            .add_point(vec![
                RecordValue::Double(1.0),
                RecordValue::Double(2.0),
                RecordValue::Double(3.0),
            ])
            .expect("Failed to add point");

        pc_writer
            .finalize()
            .expect("Failed to finalize point cloud");
        writer.finalize().expect("Failed to finalize E57 writer");
    }

    let reader =
        format::e57::E57ReaderImpl::new(&tmp).expect("Failed to open E57 file");
    let meta = reader.metadata();

    assert!(
        meta.crs_wkt.is_some(),
        "Expected CRS metadata to be present"
    );
    let crs = meta.crs_wkt.as_ref().unwrap();
    assert!(crs.contains("WGS 84"), "CRS should contain WGS 84");
    assert!(crs.contains("GEOGCS"), "CRS should be a GEOGCS");

    std::fs::remove_file(&tmp).ok();
}

#[test]
fn e57_crs_metadata_absent_when_not_set() {
    // No coordinate_metadata set — should produce None
    use e57::{
        E57Writer, Record, RecordDataType, RecordName, RecordValue,
    };

    let tmp = std::env::temp_dir().join("test_e57_no_crs.e57");

    {
        let mut writer = E57Writer::from_file(&tmp, "test-guid-nocrs-0000-0000-0003")
            .expect("Failed to create E57 writer");

        let prototype = vec![
            Record {
                name: RecordName::CartesianX,
                data_type: RecordDataType::F64,
            },
            Record {
                name: RecordName::CartesianY,
                data_type: RecordDataType::F64,
            },
            Record {
                name: RecordName::CartesianZ,
                data_type: RecordDataType::F64,
            },
        ];

        let mut pc_writer = writer
            .add_pointcloud("pc-guid-nocrs-0000-0000-0004", prototype)
            .expect("Failed to create point cloud writer");

        pc_writer
            .add_point(vec![
                RecordValue::Double(1.0),
                RecordValue::Double(2.0),
                RecordValue::Double(3.0),
            ])
            .expect("Failed to add point");

        pc_writer
            .finalize()
            .expect("Failed to finalize point cloud");
        writer.finalize().expect("Failed to finalize E57 writer");
    }

    let reader =
        format::e57::E57ReaderImpl::new(&tmp).expect("Failed to open E57 file");
    let meta = reader.metadata();
    assert!(meta.crs_wkt.is_none());

    std::fs::remove_file(&tmp).ok();
}
