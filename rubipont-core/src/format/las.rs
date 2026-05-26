use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use crate::error::{Result, RubipontError};
use crate::layout::{PointChunk, PipelineContext, PointLayout};
use crate::pipeline::{PointCloudReader, PointCloudWriter};

/// Extension detection — called by the conversion pipeline dispatcher.
pub fn detect(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("las")
}

pub struct LasReader {
    las_reader: las::Reader,
    layout: PointLayout,
    metadata: PipelineContext,
    exhausted: bool,
}

impl LasReader {
    pub fn new(path: &Path) -> Result<Self> {
        let las_reader =
            las::Reader::from_path(path).map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: 0,
                detail: e.to_string(),
            })?;
        let header = las_reader.header().clone();

        let layout = PointLayout {
            point_size: header.point_format().len() as usize,
            num_points: header.number_of_points(),
            has_integer_coords: true,
        };

        let transforms = header.transforms();
        let mut metadata = PipelineContext::default();
        metadata.coordinate_scale = Some((
            transforms.x.scale,
            transforms.y.scale,
            transforms.z.scale,
        ));
        metadata.coordinate_offset = Some((
            transforms.x.offset,
            transforms.y.offset,
            transforms.z.offset,
        ));

        // Store LAS version
        metadata.las_version = Some((
            header.version().major,
            header.version().minor,
        ));

        // Extract WKT CRS (LAS 1.4 EVLRs)
        if let Some(wkt_bytes) = header.get_wkt_crs_bytes() {
            if let Ok(crs_str) = String::from_utf8(wkt_bytes.to_vec()) {
                metadata.crs_wkt = Some(crs_str);
            }
        }

        Ok(Self {
            las_reader,
            layout,
            metadata,
            exhausted: false,
        })
    }
}

impl PointCloudReader for LasReader {
    fn read_chunk(&mut self) -> Result<Option<PointChunk>> {
        if self.exhausted {
            return Ok(None);
        }

        let mut data = Vec::with_capacity(4096 * 26);
        let mut count = 0usize;

        for pt_result in self.las_reader.points().take(4096) {
            let pt = pt_result.map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: count as u64,
                detail: e.to_string(),
            })?;

            data.extend_from_slice(&pt.x.to_le_bytes());
            data.extend_from_slice(&pt.y.to_le_bytes());
            data.extend_from_slice(&pt.z.to_le_bytes());
            data.extend_from_slice(&pt.intensity.to_le_bytes());
            count += 1;
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

pub struct LasWriter {
    writer: las::Writer<BufWriter<File>>,
    point_count: u64,
}

impl LasWriter {
    pub fn new(path: &Path, _layout: &PointLayout, metadata: &PipelineContext) -> Result<Self> {
        let version = metadata.las_version.unwrap_or((1, 2));
        let mut builder = las::Builder::from(version);
        builder.point_format =
            las::point::Format::new(0).map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        if let Some((sx, sy, sz)) = metadata.coordinate_scale {
            builder.transforms.x.scale = sx;
            builder.transforms.y.scale = sy;
            builder.transforms.z.scale = sz;
        }
        if let Some((ox, oy, oz)) = metadata.coordinate_offset {
            builder.transforms.x.offset = ox;
            builder.transforms.y.offset = oy;
            builder.transforms.z.offset = oz;
        }

        let mut header = builder
            .into_header()
            .map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        // Write WKT CRS into EVLRs for LAS 1.4+
        if version >= (1, 4) {
            if let Some(crs_wkt) = &metadata.crs_wkt {
                header.set_wkt_crs(crs_wkt.as_bytes().to_vec()).ok();
            }
        }

        let writer =
            las::Writer::from_path(path, header).map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: 0,
                detail: e.to_string(),
            })?;

        Ok(Self {
            writer,
            point_count: 0,
        })
    }
}

impl PointCloudWriter for LasWriter {
    fn write_chunk(&mut self, chunk: &PointChunk) -> Result<()> {
        let point_size = 26usize; // 3×f64 (8 each) + u16 (2) = 26 bytes
        for i in 0..chunk.len {
            let offset = i * point_size;
            let x = f64::from_le_bytes(chunk.data[offset..offset + 8].try_into().unwrap());
            let y =
                f64::from_le_bytes(chunk.data[offset + 8..offset + 16].try_into().unwrap());
            let z =
                f64::from_le_bytes(chunk.data[offset + 16..offset + 24].try_into().unwrap());
            let intensity = u16::from_le_bytes(
                chunk.data[offset + 24..offset + 26].try_into().unwrap(),
            );

            let pt = las::Point {
                x,
                y,
                z,
                intensity,
                ..Default::default()
            };
            self.writer
                .write_point(pt)
                .map_err(|e| RubipontError::ParseError {
                    format: "LAS".into(),
                    offset: self.point_count,
                    detail: e.to_string(),
                })?;
            self.point_count += 1;
        }
        Ok(())
    }

    fn finalize(&mut self) -> Result<()> {
        self.writer
            .close()
            .map_err(|e| RubipontError::ParseError {
                format: "LAS".into(),
                offset: self.point_count,
                detail: e.to_string(),
            })?;
        Ok(())
    }
}
