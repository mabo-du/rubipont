use std::path::Path;
use std::io::{Seek, SeekFrom, Write};
use byteorder::{LittleEndian, WriteBytesExt};

use crate::array::read_array;
use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::layout::INTERNAL_POINT_SIZE;
use crate::pipeline::{PointCloudReader, PointCloudWriter};

pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("laz")
}

pub struct LazReader {
    reader: laz::las::file::SimpleReader<'static>,
    layout: PointLayout,
    metadata: PipelineContext,
    exhausted: bool,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
}

impl LazReader {
    pub fn new(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)?;

        // Read the full LAS header to get scale/offset transforms.
        // We open a separate handle since SimpleReader takes ownership of the file.
        let mut header_file = std::fs::File::open(path)?;
        let las_header = las::Header::new(&mut header_file).map_err(|e| {
            RubipontError::ParseError {
                format: "LAZ".into(),
                offset: 0,
                detail: format!("Could not read LAZ header via las crate: {}", e),
            }
        })?;

        let transforms = las_header.transforms();
        let scale = (transforms.x.scale, transforms.y.scale, transforms.z.scale);
        let offset = (transforms.x.offset, transforms.y.offset, transforms.z.offset);
        let num_points = las_header.number_of_points();

        // Extract WKT CRS (LAS 1.4 EVLRs) before building metadata
        let crs_wkt = las_header
            .get_wkt_crs_bytes()
            .and_then(|wkt_bytes| String::from_utf8(wkt_bytes.to_vec()).ok());

        let reader = laz::las::file::SimpleReader::new(file).map_err(|e| {
            RubipontError::ParseError {
                format: "LAZ".into(),
                offset: 0,
                detail: e.to_string(),
            }
        })?;

        let layout = PointLayout {
            // point_size describes the internal chunk format, not the on-disk
            // LAS format.  The reader always produces 26-byte records regardless
            // of the source format's point record length.  The pipeline
            // (pipeline.rs) strides point data using this value, so reporting
            // the on-disk size would cause misaligned reads when reprojecting.
            point_size: INTERNAL_POINT_SIZE,
            num_points,
            has_integer_coords: true,
        };

        let metadata = PipelineContext {
            coordinate_scale: Some(scale),
            coordinate_offset: Some(offset),
            las_version: Some((las_header.version().major, las_header.version().minor)),
            crs_wkt,
        };

        Ok(Self {
            reader,
            layout,
            metadata,
            exhausted: false,
            scale,
            offset,
        })
    }
}

impl PointCloudReader for LazReader {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.exhausted {
            return Ok(None);
        }

        let mut data = Vec::with_capacity(4096 * INTERNAL_POINT_SIZE);
        let mut count = 0usize;

        for _ in 0..4096 {
            match self.reader.read_next() {
                Some(Ok(raw)) => {
                    // Raw LAS Point Format 0 (and compat formats): 20+ bytes
                    // [0..4]: X as i32
                    // [4..8]: Y as i32
                    // [8..12]: Z as i32
                    // [12..14]: intensity as u16
                    let x_i32 = i32::from_le_bytes(read_array(raw, 0)?);
                    let y_i32 = i32::from_le_bytes(read_array(raw, 4)?);
                    let z_i32 = i32::from_le_bytes(read_array(raw, 8)?);
                    let intensity = u16::from_le_bytes(read_array(raw, 12)?);

                    // Convert scaled integers to f64
                    let x = x_i32 as f64 * self.scale.0 + self.offset.0;
                    let y = y_i32 as f64 * self.scale.1 + self.offset.1;
                    let z = z_i32 as f64 * self.scale.2 + self.offset.2;

                    data.extend_from_slice(&x.to_le_bytes());
                    data.extend_from_slice(&y.to_le_bytes());
                    data.extend_from_slice(&z.to_le_bytes());
                    data.extend_from_slice(&intensity.to_le_bytes());
                    count += 1;
                }
                Some(Err(e)) => {
                    return Err(RubipontError::ParseError {
                        format: "LAZ".into(),
                        offset: count as u64,
                        detail: e.to_string(),
                    });
                }
                None => break,
            }
        }

        if count == 0 {
            self.exhausted = true;
            return Ok(None);
        }

        Ok(Some(PointChunk { data, len: count }))
    }

    fn layout(&self) -> &PointLayout {
        &self.layout
    }

    fn metadata(&self) -> &PipelineContext {
        &self.metadata
    }
}

pub struct LazWriter {
    compressor: laz::LasZipCompressor<'static, std::fs::File>,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    point_count: u64,
}

impl LazWriter {
    pub fn new(
        path: &Path,
        _layout: &PointLayout,
        metadata: &PipelineContext,
    ) -> Result<Self> {
        let mut file = std::fs::File::create(path)?;

        let scale = metadata.coordinate_scale.unwrap_or((0.01, 0.01, 0.01));
        let offset = metadata.coordinate_offset.unwrap_or((0.0, 0.0, 0.0));

        // Reject zero scale components — would cause division by zero
        // when converting f64 coordinates back to scaled integers.
        if scale.0 == 0.0 || scale.1 == 0.0 || scale.2 == 0.0 {
            return Err(RubipontError::ParseError {
                format: "LAZ".into(),
                offset: 0,
                detail: "coordinate scale components must be non-zero".into(),
            });
        }

        // Build LAZ items for Point Format 0
        let laz_vlr = {
            let items = laz::LazItemRecordBuilder::new()
                .add_item(laz::LazItemType::Point10)
                .build();
            laz::LazVlr::from_laz_items(items)
        };

        // Build a LAS header with the LAZ VLR embedded in the VLR list.
        // We write the point format as uncompressed (0), then patch the
        // compressed bit (bit 7) in the raw header bytes afterwards.
        // This avoids the las crate rejecting compressed formats when built
        // without the "laz" feature.
        let mut builder = las::Builder::from((1, 4)); // LAS 1.4
        builder.point_format = las::point::Format::new(0)
            .map_err(|e| RubipontError::ParseError {
                format: "LAZ".into(), offset: 0, detail: e.to_string(),
            })?;
        builder.transforms = las::Vector {
            x: las::Transform { scale: scale.0, offset: offset.0 },
            y: las::Transform { scale: scale.1, offset: offset.1 },
            z: las::Transform { scale: scale.2, offset: offset.2 },
        };

        // Serialize the LAZ VLR and add it to the header's VLR list
        let mut vlr_data = Vec::new();
        laz_vlr.write_to(&mut vlr_data).map_err(|e| {
            RubipontError::Io(e)
        })?;
        builder.vlrs.push(las::Vlr {
            user_id: "laszip encoded".to_string(),
            record_id: 22204,
            description: String::new(),
            data: vlr_data,
        });

        let mut header = builder.into_header().map_err(|e| {
            RubipontError::Io(std::io::Error::other(e.to_string()))
        })?;

        // Write WKT CRS into EVLRs for LAS 1.4 (same pattern as LasWriter)
        if let Some(crs_wkt) = &metadata.crs_wkt {
            header.set_wkt_crs(crs_wkt.as_bytes().to_vec()).ok();
        }

        // Write the LAS header (including VLRs) to the file
        header.write_to(&mut file).map_err(|e| {
            RubipontError::Io(std::io::Error::other(e.to_string()))
        })?;

        // Patch the point data record format byte to set the
        // compressed flag (bit 7). This byte is at offset 104
        // in a LAS 1.4 header.
        // Save the current position (= offset_to_point_data) first.
        let data_start = file.stream_position().map_err(|e| {
            RubipontError::Io(e)
        })?;
        file.seek(SeekFrom::Start(104)).map_err(|e| {
            RubipontError::Io(e)
        })?;
        file.write_all(&[0x80]).map_err(|e| {
            RubipontError::Io(e)
        })?;
        // Seek back to the point data start position
        file.seek(SeekFrom::Start(data_start)).map_err(|e| {
            RubipontError::Io(e)
        })?;

        // Create the LAZ compressor at the current position (right after VLRs)
        let compressor = laz::LasZipCompressor::new(file, laz_vlr).map_err(|e| {
            RubipontError::ParseError {
                format: "LAZ".into(),
                offset: 0,
                detail: e.to_string(),
            }
        })?;

        Ok(Self {
            compressor,
            scale,
            offset,
            point_count: 0,
        })
    }
}

impl PointCloudWriter for LazWriter {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        for i in 0..chunk.len {
            let offset = i * INTERNAL_POINT_SIZE;
            if offset + INTERNAL_POINT_SIZE > chunk.data.len() {
                return Err(RubipontError::ParseError {
                    format: "LAZ".into(),
                    offset: self.point_count,
                    detail: format!(
                        "truncated chunk: expected {} bytes, got {} (point stride {INTERNAL_POINT_SIZE})",
                        offset + INTERNAL_POINT_SIZE,
                        chunk.data.len()
                    ),
                });
            }

            // Read our internal f64 format
            let x = f64::from_le_bytes(read_array(&chunk.data, offset)?);
            let y = f64::from_le_bytes(read_array(&chunk.data, offset + 8)?);
            let z = f64::from_le_bytes(read_array(&chunk.data, offset + 16)?);
            let intensity = u16::from_le_bytes(read_array(&chunk.data, offset + 24)?);

            // Convert to LAS Point Format 0 raw bytes (20 bytes)
            let x_i32 = ((x - self.offset.0) / self.scale.0) as i32;
            let y_i32 = ((y - self.offset.1) / self.scale.1) as i32;
            let z_i32 = ((z - self.offset.2) / self.scale.2) as i32;

            let mut raw = vec![0u8; 20];
            raw[0..4].copy_from_slice(&x_i32.to_le_bytes());
            raw[4..8].copy_from_slice(&y_i32.to_le_bytes());
            raw[8..12].copy_from_slice(&z_i32.to_le_bytes());
            raw[12..14].copy_from_slice(&intensity.to_le_bytes());
            // Byte 14: bit 0-2 = return number (1), bit 3-5 = number of returns (1)
            raw[14] = 0b_001_001; // return 1 of 1
            // Remaining bytes stay 0

            self.compressor.compress_one(&raw).map_err(|e| {
                RubipontError::ParseError {
                    format: "LAZ".into(),
                    offset: self.point_count,
                    detail: e.to_string(),
                }
            })?;
            self.point_count += 1;
        }
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        self.compressor.done().map_err(|e| {
            RubipontError::ParseError {
                format: "LAZ".into(),
                offset: self.point_count,
                detail: e.to_string(),
            }
        })?;

        // Update the LAS header with the actual point count
        // LAS 1.4 stores u64 count at offset 247, legacy u32 at offset 107
        {
            let file = self.compressor.get_mut();
            // Legacy u32 number_of_point_records at offset 107
            file.seek(SeekFrom::Start(107)).map_err(|e| {
                RubipontError::Io(e)
            })?;
            file.write_u32::<LittleEndian>(self.point_count as u32).map_err(|e| {
                RubipontError::Io(e)
            })?;
            // LAS 1.4 u64 number_of_point_records at offset 247
            file.seek(SeekFrom::Start(247)).map_err(|e| {
                RubipontError::Io(e)
            })?;
            file.write_u64::<LittleEndian>(self.point_count).map_err(|e| {
                RubipontError::Io(e)
            })?;
        }

        Ok(())
    }
}
