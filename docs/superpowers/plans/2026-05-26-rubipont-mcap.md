# rubipont Phase 3 — ROS 2 MCAP Read Support

**Goal:** Add ROS 2 MCAP file read support to rubipont, enabling conversion from MCAP containing PointCloud2 messages to LAS, LAZ, PCD, and E57.

**Architecture:** Use the `mcap` crate for container reading (chunk decompression, trailing index seeking). Custom CDR header stripping. PointField-offset-based extraction with Eigen alignment padding stripping.

**Tech Stack:** `mcap` (Foxglove, maintained), `ros_pointcloud2`, `memmap2` (via mcap).

---

### Key Implementation Details

**MCAP reading flow:**
1. Open MCAP via `mcap::FileStream` or `mcap::MmapStream` (memmap2)
2. Read Summary Section to discover Channel records (topic → channel_id mapping)
3. Filter for topics containing `/points` or `/lidar` (user-configurable)
4. Iterate Message records for matching channel_ids
5. For each message: strip 4-byte CDR header, parse PointCloud2, extract points

**CDR header:** First 4 bytes: 0x00 0x01 (LE marker) + 0x00 0x00 (options). Strip these.

**PointCloud2 parsing (manual):**
```
[Header: seq+stamp+frame_id] [height:u32] [width:u32] [fields array] [is_bigendian:u8]
[point_step:u32] [row_step:u32] [data length:u32] [data bytes] [is_dense:u8]
```

**Field extraction:**
- Parse PointField entries (name, offset, datatype, count)
- For each point at `point_step` stride:
  - Read x,y,z at their offsets (FLOAT32 = 4 bytes each)
  - Read intensity at its offset if present
  - **Discard padding bytes** — only read fields at their specified offsets
- Convert to internal 26-byte format (3×f64 + u16)

**Parallel processing:** Use `rayon` or chunked iteration for the data blob.
