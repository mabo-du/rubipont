# rubipont — ROS Bag/MCAP Deep Research Prompt

I am building a Rust library for lossless LiDAR point cloud format translation (rubipont) and need to add read support for ROS bag files (both ROS 1 .bag and ROS 2 .mcap). Please conduct a detailed investigation covering:

---

## 1. ROS 1 Bag Format Specification

### File structure
- What is the exact binary layout of a ROS 1 .bag file?
- How are messages indexed (connection records, chunk records, index data)?
- How is timestamp-based seeking implemented?
- What is the structure of message record headers and how are they serialized?

### PointCloud2 in ROS 1
- How is sensor_msgs/PointCloud2 serialized within a bag file?
- What is the PointField structure — data types, byte offsets, count?
- How does the 16-byte Eigen alignment padding affect the binary layout of PointCloud2 messages?
- Are there known tools or crates that decompress ROS 1 bag files (e.g., bz2, lz4 compression)?

### Existing Rust tooling
- Are there any Rust crates for reading ROS 1 bag files? (e.g., rosbag-rs, rosrust)
- What is their maintenance status and completeness?
- Can they be used as a dependency, or would we need to write a reader from scratch?

---

## 2. ROS 2 / MCAP Format Specification

### MCAP file structure
- What is the MCAP binary layout summary (magic bytes, records, chunks, indexes)?
- How are messages stored (chunked compression, record types)?
- What is the attachment handling and how are schema/message data stored?
- How does the trailing index work for seeking and summarization?

### MCAP + CDR serialization
- How are ROS 2 messages serialized via CDR (Common Data Representation) in MCAP?
- How to extract PointCloud2 raw byte payloads from MCAP records?
- How does the DDS/CDR serialization handle the Eigen alignment padding issue (where C++ structs enforce 16-byte alignment, bloating point data by ~46%)?
- What is the byte-masking strategy to strip padding and extract only the valid point data fields?

### MCAP compression
- What compression schemes does MCAP support for chunk compression (LZ4, Zstd, none)?
- How is the compression metadata stored in the MCAP records?
- Are there Rust crates for reading MCAP files? (mcap crate?)

---

## 3. Key Conversion Workflows

### MCAP → LAZ / MCAP → PCD
- What is the standard workflow for extracting LiDAR data from ROS 2 bags for offline processing?
- How do current tools (rosbag2_py, Foxglove CLI, mcap CLI) perform?
- What are the known performance bottlenecks (CPU, disk I/O, memory)?
- What PointCloud2 fields are commonly populated (x, y, z, intensity, ring, range, timestamp) vs always zero?

### E57 → ROS / LAS → ROS
- Are there established patterns for converting static point cloud files into ROS bag streams for SLAM simulation?
- What timestamp and frame_id conventions are expected?

---

## 4. Rust Ecosystem Status

### MCAP Rust support
- Is there a maintained `mcap` Rust crate? What does its API look like?
- Does it support reading MCAP files, iterating messages, decompression?
- Can it handle ROS 2 PointCloud2 message schemas?

### ROS 2 message definitions in Rust
- Are there Rust crates for deserializing ROS 2 message types (sensor_msgs/PointCloud2)?
- How are ROS 2 message definitions (.msg files) converted to Rust types?
- What is the state of `ros2-client`, `rclrust`, or similar Rust ROS 2 libraries?

### CDR deserialization in Rust
- Are there Rust crates for CDR deserialization (e.g., `cdr-rs`, `dds-rs`)?
- How to manually parse a PointCloud2 CDR stream into raw bytes?

---

## 5. ROS Bag Utilities and Special Cases

- Does rosbag2 have a command-line tool that can write individual bag messages as JSON/YAML for debugging?
- Are there sample MCAP files with PointCloud2 data available for testing?
- What are the minimum and maximum MCAP file sizes commonly encountered in automotive datasets?
- Are there edge cases with multi-topic bags (LIDAR + camera + IMU) where we need to filter for /points2 topics only?

---

Please provide specific GitHub repos, crate names with versions, documentation links, and relevant specification URLs where available. Flag any areas where the ecosystem is rapidly changing or uncertain.
