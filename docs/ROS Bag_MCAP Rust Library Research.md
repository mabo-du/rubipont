# **Comprehensive Architecture and Ecosystem Analysis for Lossless LiDAR Point Cloud Translation: ROS 1 Bag and ROS 2 MCAP Paradigms**

## **1\. Introduction to Robotic Data Serialization and the rubipont Architecture**

The translation of high-dimensional LiDAR data from robotic middleware logging formats into standardized spatial formats (such as LAZ, PCD, or E57) represents a critical bottleneck in autonomous systems engineering. As autonomous platforms integrate solid-state, high-resolution, and multi-return LiDAR sensors, the volume of data generated per second has scaled exponentially. Logging mechanisms designed for real-time robotic inter-process communication (IPC) are inherently optimized for network transport, not for offline spatial analytics or long-term archival storage.  
Developing a high-performance, Rust-based library for lossless LiDAR format translation—such as the proposed rubipont architecture—demands an exhaustive understanding of two primary serialization ecosystems: the legacy ROS 1 rosbag format and the modern ROS 2 MCAP (Multimodal Container for Pub/Sub) format. A robust translation engine must navigate complex, deeply nested binary layouts, handle multiple layers of chunked compression, and mitigate pervasive data bloat introduced by C++ memory alignment paradigms. Furthermore, the library must leverage the modern Rust ecosystem to achieve zero-copy deserialization and parallelized byte-masking, overcoming the fundamental performance limitations observed in existing Python-based utilities.  
This report provides a rigorous architectural analysis of ROS 1 and ROS 2 serialization standards, the specific structural mechanics of the sensor\_msgs/PointCloud2 message, the mathematical and systemic implications of memory alignment padding, and the state of the Rust ecosystem for parsing and translating these immense automotive datasets.

## **2\. ROS 1 Bag Format Specification Architecture**

The ROS 1 bag format (version 2.0) was developed as an efficient, sequential logging mechanism meant to mirror the ROS network transport layer.1 By directly writing serialized network messages to disk, the format minimizes overhead during data collection. However, this monolithic, append-only structure introduces significant complexities for downstream processors attempting to extract specific spatial data streams without linear scanning.

### **2.1 Binary Layout and File Structure**

A standard ROS 1 bag file initiates with a human-readable, newline-terminated magic string, which serves as the format version identifier: \#ROSBAG V2.0\\n.2 Following this initial declaration, the file is entirely composed of a continuous sequence of records.  
To facilitate stream parsing, every record in a ROS 1 bag adheres to a strict bipartite structure consisting of a header and a data payload. The binary layout of a generic record is structured as follows:

1. **Header Length**: A 4-byte little-endian integer specifying the total byte length of the upcoming header.  
2. **Header Payload**: A sequence of field-value pairs. Each field consists of a 4-byte little-endian integer specifying the field length, followed by an ASCII string formatted precisely as name=value.2  
3. **Data Length**: A 4-byte little-endian integer specifying the total byte length of the upcoming data payload.  
4. **Data Payload**: The raw binary data, the structure of which is dictated by the record's opcode.

The header payload must contain an op (opcode) field, encoded as a single byte, which dictates the semantic meaning and expected data payload of the record.4

| Opcode | Record Type | Description and Structural Role |
| :---- | :---- | :---- |
| 0x01 | Message Definition | Contains the raw .msg text definition. In a well-formed bag, this appears exactly once prior to any message data for a given topic.4 |
| 0x02 | Message Data | The serialized ROS message payload. The header must contain conn (connection ID) and time (timestamp) fields.2 |
| 0x03 | Bag Header | The primary index pointer. Contains index\_pos (byte offset to the first record after the chunk section), conn\_count, and chunk\_count. It is padded with ASCII space characters (0x20) to exactly 4096 bytes to allow in-place modification.2 |
| 0x04 | Index Data | Maps message timestamps to their specific offsets within an uncompressed chunk. The header includes the ver (version), conn, and count.2 |
| 0x05 | Chunk | A block containing a sequence of Connection and Message Data records. The header includes a compression field (e.g., "none", "bz2", "lz4") and the size of the uncompressed data.2 |
| 0x06 | Chunk Info | Maps the temporal boundaries of a chunk. Contains start\_time, end\_time, and an array of connection IDs with their respective message counts.2 |
| 0x07 | Connection | Maps a numerical conn ID to a specific topic string. The payload contains the connection header, including type, md5sum, and the full message\_definition.2 |

### **2.2 Indexing Mechanisms and Timestamp-Based Seeking**

In autonomous vehicle datasets, bag files frequently exceed hundreds of gigabytes. Consequently, linear scanning to find a specific LiDAR frame is computationally unviable. ROS 1 implements timestamp-based seeking via a multi-tiered lookup utilizing the Bag Header, Chunk Info, and Index Data records.  
The seeking algorithm operates sequentially:

1. The parser reads the Bag Header (0x03) at the start of the file to extract the index\_pos integer.2  
2. The parser performs a seek operation directly to index\_pos, which points to the trailing metadata section.  
3. The parser iterates over the Chunk Info (0x06) records, reading the bounding start\_time and end\_time timestamps to identify which chunk encapsulates the target timestamp.2  
4. Once the target chunk is identified, the parser locates the corresponding Index Data (0x04) record for the desired connection ID. This record yields a precise array of message timestamps and their corresponding byte offsets relative to the start of the uncompressed chunk.2  
5. The parser jumps to the Chunk (0x05), decompresses the payload into memory (if bz2 or lz4 compression is indicated), and utilizes the offset to extract the exact 0x02 Message Data record without scanning surrounding messages.2

### **2.3 sensor\_msgs/PointCloud2 Serialization in ROS 1**

The payload of a 0x02 Message Data record mapped to a LiDAR connection is the direct byte-for-byte serialization of the sensor\_msgs/PointCloud2 message. In the ROS 1 ecosystem, message serialization lacks an encapsulating middleware layer; it relies entirely on the direct, sequential binary representation of the fields as defined in the .msg file.  
The PointCloud2 structure consists of the following serialized components, written continuously to the byte stream 6:

1. **header (std\_msgs/Header)**: Contains a seq (uint32), stamp (composed of a 4-byte seconds integer and a 4-byte nanoseconds integer), and frame\_id (a 4-byte string length prefix followed by the UTF-8 string bytes).6  
2. **height (uint32)**: Represents the 2D structure of the cloud. For unordered LiDAR clouds (e.g., spinning Velodyne or Ouster sensors), this is hardcoded to 1\.6  
3. **width (uint32)**: The total length of the point cloud (number of points).6  
4. **fields (PointField array)**: Describes the structural layout of a single point within the binary blob. This begins with a 4-byte integer indicating the number of fields, followed by a sequence of PointField structures.6  
5. **is\_bigendian (bool)**: A single byte indicating the endianness of the data payload (almost exclusively false/0 in modern x86 or ARM architectures).6  
6. **point\_step (uint32)**: The exact length of a single point in bytes.6  
7. **row\_step (uint32)**: The exact length of a row in bytes (calculated as point\_step \* width for unordered clouds).6  
8. **data (uint8 array)**: The raw binary blob containing the interleaved point data. This begins with a 4-byte length prefix (equal to row\_step \* height), followed by the raw bytes.6  
9. **is\_dense (bool)**: A single byte indicating whether the cloud contains invalid (NaN/Inf) points. True indicates all points are valid.6

#### **2.3.1 The PointField Structure**

The dynamic interpretation of the data blob is entirely dependent on the fields array. Each PointField is serialized as:

* **name (string)**: A 4-byte length prefix followed by the ASCII name of the channel (e.g., "x", "y", "z", "intensity", "ring").8  
* **offset (uint32)**: The byte offset from the beginning of a point's memory boundary to this specific data element.8  
* **datatype (uint8)**: An enumeration defining the primitive type (1=INT8, 2=UINT8, 3=INT16, 4=UINT16, 5=INT32, 6=UINT32, 7=FLOAT32, 8=FLOAT64).8  
* **count (uint32)**: The number of elements in the field (usually 1, unless the field is an array).8

### **2.4 The Eigen Alignment Padding Penalty**

A critical, systemic inefficiency in translating ROS point clouds stems from hardware-accelerated memory alignment enforced by lower-level C++ libraries, such as the Point Cloud Library (PCL) and the Eigen linear algebra framework.9 To optimize CPU vectorized operations (like SSE/AVX instructions), C++ structures frequently enforce strict 16-byte memory alignments via macros such as EIGEN\_ALIGN16 or EIGEN\_MAKE\_ALIGNED\_OPERATOR\_NEW.9  
When these C++ memory structures are indiscriminately serialized by sensor drivers into the ROS network transport, the empty alignment padding bytes are serialized alongside the valid coordinate data. This creates a severe data bloat issue that permanently impacts network bandwidth, disk storage, and the I/O throughput of downstream translation tools.9  
The mathematical efficiency of the point data serialization can be modeled as:  
![][image1]  
This phenomenon is highly visible in Ouster LiDAR drivers. The driver instantiates a point struct containing the following fields: x (float32, 4B), y (float32, 4B), z (float32, 4B), intensity (float32, 4B), t (uint32, 4B), reflectivity (uint16, 2B), ring (uint8, 1B), ambient (uint16, 2B), and range (uint32, 4B).9  
The sum of the valid payload is exactly 29 bytes. However, due to the EIGEN\_ALIGN16 macro, the struct is padded out to the nearest multiple of 16, artificially pushing the point\_step to 48 bytes.9  
The binary layout of a single point in this configuration contains vast dead zones:

* 4 bytes of padding between z and intensity.12  
* 2 bytes of padding between ambient and range.12  
* 12 bytes of padding at the tail of the struct.12

This results in an efficiency ratio of ![][image2]. The C++ alignment padding inflates the raw point data array by approximately 65%, effectively wasting nearly 40% of the byte stream and the resulting ROS bag storage space.9 When architecting the rubipont translator, it is imperative to implement an aggressive byte-masking strategy. The translator must parse the fields array, utilize the specified offset values, and apply a bitmask to read only the valid bytes, actively stripping the padding before encoding the data into dense formats like LAZ or PCD.

### **2.5 Existing Rust Tooling for ROS 1 Bags**

The Rust ecosystem for pure ROS 1 bag ingestion contains several historical solutions, though maintenance patterns indicate a broader community shift toward ROS 2 tooling.

* **rosbag crate**: Developed by SkoltechRobotics, this crate offers utilities for reading chunk records, message records, and index records.13 It handles bzip2 and lz4 decompression natively and utilizes the memmap2 crate to map the file into virtual memory.13 However, it is largely unmaintained, with the last significant update occurring over four years ago (version 0.6.0).13  
* **rosbags-rs crate**: Authored by amin-abouee, this library claims high-performance, byte-for-byte compatibility with the Python rosbags library.16 While it heavily targets ROS 2 formats (SQLite3 and MCAP), its underlying architecture provides comprehensive ROS 1 bridging and supports advanced filtering by topic and time range.16  
* **rustbag crate**: A relatively newer CLI tool and player that solves starvation issues present in the original C++ rosbag2 player by utilizing Rust's asynchronous message passing channels.19

Given the finalized, static nature of the ROS 1 specification, utilizing an existing crate like rosbag (if updated for modern Rust editions) or the more modern rosbags-rs as a foundational dependency for rubipont is a highly viable approach. Writing a ROS 1 parser from scratch is generally unnecessary unless specific zero-copy optimizations for the 0x02 opcodes cannot be satisfied by the memmap2 implementations within these existing libraries.

## **3\. ROS 2 and MCAP Format Specification Architecture**

The transition to ROS 2 introduced a fundamental paradigm shift away from the monolithic ROS 1 .bag (and the intermediate ROS 2 SQLite3 .db3) to the MCAP (Multimodal Container for Pub/Sub) format. Developed by Foxglove and adopted as the default storage container for ROS 2 from the Iron release onward, MCAP is serialization-agnostic, self-describing, and optimized for high-throughput append-only writes in rigorous robotic environments.20

### **3.1 MCAP Binary Layout and Record Semantics**

MCAP operates on a strictly defined, sequential binary layout characterized by magic bytes, interleaved records, and complex trailing summarization indexing.  
The file must begin and end with an identical magic byte sequence: 0x89, M, C, A, P, 0x30, \\r, \\n. The byte following "MCAP" (0x30, representing ASCII 0\) signifies the major version of the format.23  
The primary data layer is composed of records. Every record, regardless of its function, adheres to a universal binary envelope:

1. **Opcode**: A 1-byte identifier (0x01 to 0x7F reserved for standard usage).23  
2. **Record Length**: An 8-byte (uint64) little-endian integer indicating the exact byte length of the payload.23  
3. **Payload**: The serialized record data.23

Unlike ROS 1, MCAP enforces the embedding of message schemas directly within the file, guaranteeing that the container remains mathematically decodable in perpetuity without requiring external dependencies, workspaces, or compiled C++ headers.21

| Opcode | Record Type | Functionality and Architectural Role |
| :---- | :---- | :---- |
| 0x03 | Schema | Defines the structural layout of messages. Contains a name, encoding (e.g., ros2msg, ros2idl, protobuf), and the concatenated .msg or .idl definition data.23 |
| 0x04 | Channel | Links a specific robotic topic (e.g., /lidar/points) to a Schema ID and defines the network message encoding (e.g., cdr, json).23 |
| 0x05 | Message | Encodes a single timestamped event. Contains the Channel ID, a sequence counter, log\_time, publish\_time, and the raw encoded byte payload.23 |
| 0x06 | Chunk | A compressed container encapsulating multiple Schema, Channel, and Message records to optimize disk I/O and reduce file bloat.23 |
| 0x09 | Attachment | Stores heterogeneous metadata, such as camera calibration matrices, LiDAR extrinsics, or core dumps. Contains log\_time, create\_time, name, media\_type, and a uint64-prefixed raw data byte array.21 |

### **3.2 Chunk Compression and Trailing Index Mechanics**

To mitigate the massive storage requirements of modern autonomous vehicle datasets—where continuous multi-sensor recording can yield 10 GB to 25 GB per file—MCAP bundles messages into Chunk records (0x06).27  
The Chunk payload includes the message\_start\_time, message\_end\_time, uncompressed\_size, a crc32 checksum, and a compression string identifier.23 MCAP natively supports well-known compression formats:

* **LZ4**: Prioritizes rapid decompression, ensuring that offline translation tools and data pipelines are not excessively bottlenecked by CPU overhead.23  
* **Zstd**: Provides a highly tunable compression ratio. In empirical benchmark tests against traditional SQLite3 bags, recording high-throughput RGB-D and LiDAR data using MCAP with Zstd (slow) compression reduced file sizes from 1.35 GB to 393 MB (a \~70% reduction) while preserving lossless data integrity.28

To enable ![][image3] or ![][image4] seeking across these compressed chunks, MCAP utilizes a complex trailing index. When an MCAP reader initiates, it reads the trailing magic bytes and parses the preceding Footer record (0x02).23 The Footer contains the summary\_start and summary\_offset\_start pointers (uint64).23  
Jumping to the Summary Offset section, the reader locates the Chunk Index (0x08) records.23 Each Chunk Index contains a message\_index\_offsets map, which correlates a specific Channel ID to a Message Index record (0x07).23 The Message Index provides the exact byte offset of a timestamped message within an uncompressed chunk. This architecture allows a translation tool to leap over gigabytes of irrelevant camera or IMU data, extracting only the LiDAR point clouds directly from disk.21 Similarly, Attachment Index (0x0A) records allow instant access to static calibration data embedded at the time of recording.

### **3.3 Middleware Integration: MCAP, DDS, and CDR Serialization**

While MCAP acts as the agnostic container format, the actual PointCloud2 payload housed within a Message record (0x05) is dictated by the underlying ROS 2 middleware: DDS (Data Distribution Service). DDS serializes network messages utilizing the OMG Common Data Representation (CDR) specification.24  
When rubipont accesses an MCAP file, the Message record's raw data payload contains the CDR-encoded bytes. This protocol introduces structural complexities absent in ROS 1:  
**1\. The Encapsulation Header:** Every CDR payload transmitted by DDS must begin with a 4-byte encapsulation header.29

* Bytes 0-1 indicate the representation identifier. 0x00 0x00 denotes Classic CDR Big Endian (CDR\_BE), while 0x00 0x01 denotes Classic CDR Little Endian (CDR\_LE).30 Other variants, such as 0x00 0x02 (PL\_CDR\_BE), dictate ParameterList representations for mutable types.31  
* Bytes 2-3 are reserved for serialization options and padding indicators (usually 0x00 0x00).33

A point cloud translator must intercept, validate, and strip these four bytes before attempting to map the payload to a native Rust struct.36  
**2\. CDR Alignment Padding:** CDR enforces rigorous memory alignment rules relative to the start of the CDR stream (immediately post-header). For instance, a float32 must align to a 4-byte boundary, and a float64 or uint64 must align to an 8-byte boundary.34  
For sensor\_msgs/PointCloud2, this requires a dynamic parsing strategy:

* The std\_msgs/Header is serialized first.  
* The height and width (uint32) are serialized, maintaining 4-byte alignment.  
* The fields sequence requires a 4-byte sequence length prefix, followed by the array of PointField elements.  
* The data sequence (the actual point cloud blob) requires a 4-byte sequence length prefix.23 Because it is a sequence of uint8, the individual data bytes do not require padding, but the starting address of the sequence length integer must align perfectly to a 4-byte boundary.

**3\. Point Cloud Eigen Alignment in ROS 2:** Despite the transition to ROS 2 and the encapsulation within CDR, the underlying Eigen alignment issue originating from C++ PCL libraries persists. The data blob contained within the CDR sequence still houses the 48-byte point\_step with 19 bytes of dead padding per point if generated by unoptimized sensor drivers.9  
To extract the raw LiDAR coordinates seamlessly, the translation algorithm within rubipont must:

1. Strip the 4-byte CDR encapsulation header.  
2. Traverse the CDR stream, strictly respecting 4-byte and 8-byte boundary alignments, to read the dynamic sequence of PointField definitions.  
3. Locate the 4-byte length prefix of the data blob.  
4. Iterate over the blob using the point\_step stride.  
5. Apply precise byte-masking to extract only the bytes indicated by the offset and datatype of the parsed PointField structures, forcefully discarding the C++ struct padding before writing to the target format.8

## **4\. Rust Ecosystem Status for MCAP and CDR**

Implementing a lossless translation tool in Rust requires navigating a highly active, yet somewhat fragmented, ecosystem of parsers, middleware connectors, and message generators. The synthesis of the container (MCAP), the protocol (CDR), and the domain-specific logic (PointCloud2) demands careful architectural crate selection.

### **4.1 Official MCAP Rust Support**

The primary interface for reading MCAP files is the mcap crate.38 Originally developed as mcap-rs by Anduril, it was upstreamed and is now officially maintained by the Foxglove core team.40  
The mcap crate provides robust, production-ready APIs capable of streaming records via memory-mapped files. The architecture relies heavily on memmap2::Mmap combined with mcap::MessageStream.38

* **Decompression and Streaming**: The ChunkReader and ChunkFlattener submodules seamlessly handle the decompression of lz4 and zstd chunks under the hood, yielding continuous streams of Record enumerations.43  
* **Indexing and Seeking**: The crate natively parses the trailing indices, allowing developers to query for specific channel\_ids or time ranges. This prevents allocating the entire file into heap memory, which is critical when handling 25 GB log files.43

By mapping the .mcap file directly to virtual memory (Mmap), the operating system handles paging, and the mcap crate yields borrowed byte slices (&\[u8\]) of the underlying messages.38 This enables a true zero-copy architecture up until the point of payload deserialization.

### **4.2 ROS 2 Message Definitions and Rust Integration**

To translate the raw byte slices into strongly-typed PointCloud2 objects, the Rust ecosystem offers several paradigms.  
Converting native ROS 2 .msg files to Rust types is generally handled by middleware crates:

* **ros2-client**: A pure Rust implementation of the ROS 2 client library, built on top of RustDDS.44 It provides native serialization and asynchronous API endpoints.  
* **r2r**: Provides ergonomic Rust bindings to the standard ROS 2 C API (rclc / rcl), allowing for minimal friction when integrating with standard message types.  
* **rclrust**: An alternative Rust client library aiming for high performance, though ecosystem fragmentation remains a challenge.  
* **oxidros\_core**: Provides foundational abstractions for ROS 2 functionality, allowing for multiple DDS backend implementations.46

For point cloud specific translation, the **ros\_pointcloud2** crate provides specialized, ergonomic abstractions.47 To maintain framework agnosticism, it relies on its own PointCloud2Msg struct, offering integrations with rosrust, r2r, and ros2-client via feature flags (e.g., ros2-interfaces-jazzy-serde).49  
A core architectural advantage of ros\_pointcloud2 is its integration with rayon for parallel iteration.47 Because un-padding and converting millions of LiDAR points (e.g., from a 128-channel LiDAR spinning at 10Hz) is overwhelmingly CPU-bound, the try\_into\_par\_iter implementation allows the translation pipeline to distribute the byte-masking load across all available CPU cores via work-stealing schedulers, dramatically outperforming standard sequential C++ PCL conversions.47

### **4.3 Native Rust CDR Deserialization**

Before the ros\_pointcloud2 crate can interpret the payload, the CDR byte stream must be successfully parsed.

* **cdr-encoding crate**: Maintained by Atostek (used heavily by RustDDS), this crate provides a robust implementation of the OMG CDR serialization protocol leveraging the serde framework.51 It supports both Big and Little Endian formats and automatically handles the alignment padding required by the CDR specification.51  
* **cdr / cdr-rs crate**: Generalized Serde implementations for Common Data Representation in Rust.53

While these general-purpose serde crates exist, applying a full Serde deserialization to a gigabyte-scale point cloud introduces severe memory allocation overhead. Serde typically allocates a new Vec\<u8\> to hold dynamically sized sequences. For optimal performance in rubipont, it is highly recommended to implement a custom, lightweight byte-parser that merely validates the 0x00 0x01 CDR header, extracts the dynamic metadata (height, width, point\_step), and maps the data array as a raw borrowed slice (&\[u8\]). This slice can then be passed directly to the parallel iterators, bypassing serde allocations entirely.

## **5\. Key Conversion Workflows (MCAP to LAZ/PCD)**

Extracting LiDAR data from sequential networking logs and converting it into spatial formats intended for offline modeling requires navigating specific systemic bottlenecks and establishing rigorous data mapping conventions.

### **5.1 Standard Workflows and Performance Bottlenecks**

The standard workflow for extracting point clouds involves filtering the bag container for relevant topics, decompressing the data, interpreting the point schema, and encoding the spatial coordinates via specialized point cloud codecs.  
Current tools in the ecosystem demonstrate varying levels of performance:

* **Foxglove CLI and MCAP CLI**: Tools like mcap filter allow for the rapid extraction of specific topics and time ranges. This is highly efficient due to the MCAP Chunk Index, preventing the unnecessary parsing of high-bandwidth camera streams multiplexed in the same file.24  
* **rosbag2\_py**: Python bindings for the core ROS 2 bag reading libraries. While convenient for scripting, Python's overhead when manipulating raw byte arrays makes it exceptionally slow for high-density point cloud extraction.  
* **pointcloudset**: A Python-based utility utilizing pandas and pyarrow to read ROS 1 bags and ROS 2 MCAP files, converting them directly to LAS/PCD formats.58 While effective for small datasets, Python's Global Interpreter Lock (GIL) and memory overhead create severe CPU and memory bottlenecks when handling massive point clouds.58

The primary bottleneck in format translation is uncompressing Zstd/LZ4 chunks and iterating over the unaligned binary blobs.59 I/O throughput is the secondary bottleneck. A Rust-based implementation (rubipont) bypasses Python's limitations, enabling memory-mapped reads and concurrent thread-pool point encoding.

### **5.2 PointCloud2 Field Mapping**

In practice, a PointCloud2 message contains numerous fields defined by the sensor driver, but only a subset are commonly populated and useful for spatial formats.

* **Commonly Populated Fields**: x, y, z (float32 coordinates), intensity (float32 or uint8), ring (uint16 laser channel ID), time or t (uint32 or float64 relative timestamps per point), and range.9  
* **Always Zero / Padding**: Due to the Eigen alignment issues, up to 19 bytes per point may contain zeroes or undefined memory garbage.9

Downstream formats like LAZ support standard ASPRS (American Society for Photogrammetry and Remote Sensing) point schemas which natively map x, y, z, and intensity. To losslessly retain proprietary fields like ring or ambient reflectivity, custom Extra Byte records must be defined in the LAZ Variable Length Records (VLR) header.

### **5.3 Synthesizing ROS Streams from Static Clouds (E57/LAS to ROS)**

Conversely, testing SLAM algorithms requires converting static point cloud files (E57, LAS) back into timestamped ROS streams. This presents a unique challenge: static files generally lack the temporal cadence required by robotic middleware.  
When streaming an E57 or LAS file into an MCAP container, established patterns necessitate the synthesis of a std\_msgs/Header for each point cloud frame.

* **Timestamp Injection**: If the static point cloud lacks per-point or per-frame GPS timestamps, the translation tool must simulate a constant ![][image5] (e.g., 100ms for a simulated 10Hz LiDAR) to generate sequential stamp fields.61 Failure to inject monotonically increasing timestamps will cause ROS tf (transform) trees to collapse during SLAM playback, rendering the data useless.  
* **Frame ID Binding**: The frame\_id (e.g., os\_sensor or velodyne) must be rigidly specified and consistent with the robotic URDF (Unified Robot Description Format) expected by the simulation engine.8

## **6\. Advanced Utilities, Testing, and Edge Cases**

Developing a robust parsing engine requires leveraging existing debugging tools to validate the binary outputs against known ground truths.

### **6.1 Debugging Workflows and JSON/YAML Extraction**

To inspect the precise contents of a ROS 2 point cloud without writing custom scripts, the ROS 2 command-line interface provides the ros2 topic echo utility. Executing ros2 topic echo \--once /points\_topic \> cloud.yaml extracts a single message and serializes it to a human-readable YAML file.62 For JSON output, tools like rospy\_message\_converter or Foxglove Studio's "Raw Messages Panel" can intercept the CDR-encoded streams and render them as hierarchical JSON trees.63 These tools are invaluable for reverse-engineering proprietary sensor driver padding logic and verifying the exact structure of the fields array.

### **6.2 Datasets and Minimum/Maximum MCAP Bounds**

For validation testing, the ecosystem provides several high-fidelity datasets:

* **Ouster Sample Data**: Contains uncompressed PCAP and MCAP files demonstrating the 48-byte point\_step alignment bloat.65  
* **Sydney Urban Objects**: Available in CSV and often converted to MCAP via JSON schema for testing simple x, y, z, intensity mappings.60  
* **Hilti Handheld SLAM**: Features high-frequency Hesai LiDAR data mixed with camera imagery, serving as an excellent stress test for multi-channel chunk filtering.68

In automotive contexts, the scale of MCAP files varies wildly. A short testing sequence may be as small as 10-50 MB. However, a continuous 10-minute multi-sensor recording (e.g., 4x cameras, 2x 128-channel LiDARs, IMU, and CAN bus) compressed with Zstd typically spans between 10 GB to 25 GB per file.27 Uncompressed SQLite3 bags from similar configurations often exceed 100 GB.28 The translator must therefore be capable of processing datasets substantially larger than available system RAM, strictly mandating the use of iterative reading and chunk-by-chunk processing.

### **6.3 Edge Cases: Multi-Topic Bags and Schema Evolution**

A primary edge case involves handling multi-topic bags. A naive iteration over all messages will severely bottleneck the pipeline by pointlessly decompressing and deserializing high-bandwidth H.264 camera streams multiplexed alongside the LiDAR data.69  
The translator must first read the MCAP Summary Section to index all Channel records (0x04). It must search the topic strings for targets (e.g., /points2) and verify the schema\_id points to a sensor\_msgs/PointCloud2 schema definition.23 Once the specific Channel IDs are isolated, the reader must utilize the Message Index records to leap directly to the required payloads, discarding any message whose channel\_id does not match the target.23  
Furthermore, MCAP supports schema evolution.24 A single file might contain multiple point cloud streams with varying point\_step sizes (e.g., merging a Velodyne VLP-16 with an Ouster OS1). The translation architecture must dynamically compile a byte-masking profile for *each* unique Channel ID, relying entirely on the runtime values of the PointField array rather than hardcoding static offsets.6

## **7\. Conclusions and Strategic Implementation**

The construction of rubipont as a Rust-based, lossless format translator requires precise orchestration of memory mapping, protocol decoding, and parallel processing. The transition from ROS 1's sequential serialization to ROS 2's MCAP and DDS-CDR paradigms has significantly improved data retrieval efficiency via trailing indexes, but has preserved historical inefficiencies regarding C++ memory alignment.  
**Key strategic insights for the architecture of rubipont include:**

1. **Leverage Official Container Crates**: Utilize the Foxglove mcap crate combined with memmap2. This ensures standard-compliant traversal of chunked compression, trailing indexes, and rapid Channel ID filtering, deferring memory allocation to the OS page cache.  
2. **Bypass Heavy Serialization Frameworks**: For the sensor\_msgs/PointCloud2 inner payload, avoid passing gigabytes of data through generalized Serde deserializers (e.g., cdr-encoding). Instead, implement a zero-copy pointer mapping that strips the 4-byte CDR encapsulation header and natively interprets the width, row\_step, and data boundary.  
3. **Implement Aggressive Padding Stripping**: The \~65% data bloat caused by C++ EIGEN\_ALIGN16 memory alignment mandates that rubipont dynamically calculates the valid payload bytes per point. Applying a byte-mask to drop padding bytes before feeding the data to LAZ/PCD encoders will vastly improve output file sizes and disk write speeds.  
4. **Embrace Parallel Iterators**: Utilize the ros\_pointcloud2 crate's Rayon integrations (try\_into\_par\_iter). Distributing the byte-masking and coordinate transformations across multicore CPUs will resolve the primary computational bottleneck inherent to massive point cloud translation.

#### **Works cited**

1. Bags \- ROS Wiki, accessed May 26, 2026, [https://wiki.ros.org/Bags](https://wiki.ros.org/Bags)  
2. Bags/Format/2.0 \- ROS Wiki, accessed May 26, 2026, [https://wiki.ros.org/Bags/Format/2.0](https://wiki.ros.org/Bags/Format/2.0)  
3. Bags/Format \- ROS Wiki, accessed May 26, 2026, [https://wiki.ros.org/Bags/Format](https://wiki.ros.org/Bags/Format)  
4. Bags/Format/1.2 \- ROS Wiki, accessed May 26, 2026, [https://wiki.ros.org/Bags/Format/1.2](https://wiki.ros.org/Bags/Format/1.2)  
5. rosbag package \- github.com/rovechkin1/go-rosbag \- Go Packages, accessed May 26, 2026, [https://pkg.go.dev/github.com/rovechkin1/go-rosbag](https://pkg.go.dev/github.com/rovechkin1/go-rosbag)  
6. sensor\_msgs/PointCloud2 Message \- ROS documentation, accessed May 26, 2026, [http://docs.ros.org/en/noetic/api/sensor\_msgs/html/msg/PointCloud2.html](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)  
7. sensor\_msgs/Reviews/2010-03-01 PointCloud2\_API\_Review \- ROS Wiki, accessed May 26, 2026, [https://wiki.ros.org/sensor\_msgs/Reviews/2010-03-01%20PointCloud2\_API\_Review](https://wiki.ros.org/sensor_msgs/Reviews/2010-03-01%20PointCloud2_API_Review)  
8. PointCloud2 message explained \- Medium, accessed May 26, 2026, [https://medium.com/@tonyjacob\_/pointcloud2-message-explained-853bd9907743](https://medium.com/@tonyjacob_/pointcloud2-message-explained-853bd9907743)  
9. \[conversions\] toPCLPointCloud2 or toROSMsg  
10. pcl/CHANGES.md at master · PointCloudLibrary/pcl \- GitHub, accessed May 26, 2026, [https://github.com/PointCloudLibrary/pcl/blob/master/CHANGES.md](https://github.com/PointCloudLibrary/pcl/blob/master/CHANGES.md)  
11. MVision/PCL\_APP/0\_pcl点云库基本介绍.md at master \- GitHub, accessed May 26, 2026, [https://github.com/Ewenwan/MVision/blob/master/PCL\_APP/0\_pcl%E7%82%B9%E4%BA%91%E5%BA%93%E5%9F%BA%E6%9C%AC%E4%BB%8B%E7%BB%8D.md?plain=1](https://github.com/Ewenwan/MVision/blob/master/PCL_APP/0_pcl%E7%82%B9%E4%BA%91%E5%BA%93%E5%9F%BA%E6%9C%AC%E4%BB%8B%E7%BB%8D.md?plain=1)  
12. \[ROS2\] configurable pointcloud type · Issue \#97 · ouster-lidar/ouster-ros \- GitHub, accessed May 26, 2026, [https://github.com/ouster-lidar/ouster-ros/issues/97](https://github.com/ouster-lidar/ouster-ros/issues/97)  
13. Cargo.toml \- SkoltechRobotics/rosbag-rs \- GitHub, accessed May 26, 2026, [https://github.com/SkoltechRobotics/rosbag-rs/blob/master/Cargo.toml](https://github.com/SkoltechRobotics/rosbag-rs/blob/master/Cargo.toml)  
14. rosbag \- Rust \- Docs.rs, accessed May 26, 2026, [https://docs.rs/rosbag](https://docs.rs/rosbag)  
15. SkoltechRobotics/rosbag-rs: Reading rosbag files in pure Rust \- GitHub, accessed May 26, 2026, [https://github.com/SkoltechRobotics/rosbag-rs](https://github.com/SkoltechRobotics/rosbag-rs)  
16. rosbags-rs \- crates.io: Rust Package Registry, accessed May 26, 2026, [https://crates.io/crates/rosbags-rs](https://crates.io/crates/rosbags-rs)  
17. amin-abouee/rosbags-rs: A Rust library for reading ROS2 bag files, inspired by the Python \[rosbags\](https://gitlab.com/ternaris/rosbags) library. · GitHub \- GitHub, accessed May 26, 2026, [https://github.com/amin-abouee/rosbags-rs](https://github.com/amin-abouee/rosbags-rs)  
18. rosbags\_rs \- Rust \- Docs.rs, accessed May 26, 2026, [https://docs.rs/rosbags-rs](https://docs.rs/rosbags-rs)  
19. GitHub \- iv461/rustbag: A high performace rosbag player for ROS 2, playing high-bandwidth data from multiple files, accessed May 26, 2026, [https://github.com/iv461/rustbag](https://github.com/iv461/rustbag)  
20. MCAP | Foxglove, accessed May 26, 2026, [https://foxglove.dev/product/mcap](https://foxglove.dev/product/mcap)  
21. Introducing the MCAP File Format \- Foxglove, accessed May 26, 2026, [https://foxglove.dev/blog/introducing-the-mcap-file-format](https://foxglove.dev/blog/introducing-the-mcap-file-format)  
22. Import and Export Robotics Data Using the Foxglove CLI, accessed May 26, 2026, [https://foxglove.dev/blog/import-and-export-robotics-data-using-the-foxglove-cli](https://foxglove.dev/blog/import-and-export-robotics-data-using-the-foxglove-cli)  
23. MCAP Format Specification, accessed May 26, 2026, [https://mcap.dev/spec](https://mcap.dev/spec)  
24. MCAP, accessed May 26, 2026, [https://mcap.dev/](https://mcap.dev/)  
25. MCAP Format Registry, accessed May 26, 2026, [https://mcap.dev/spec/registry](https://mcap.dev/spec/registry)  
26. Converting the EuRoC MAV Dataset to MCAP and visualizing SLAM using Foxglove., accessed May 26, 2026, [https://foxglove.dev/blog/converting-euroc-mav-dataset-to-mcap](https://foxglove.dev/blog/converting-euroc-mav-dataset-to-mcap)  
27. Convert ROS2 SQLite bag to MCAP format \- Robotics Stack Exchange, accessed May 26, 2026, [https://robotics.stackexchange.com/questions/104976/convert-ros2-sqlite-bag-to-mcap-format](https://robotics.stackexchange.com/questions/104976/convert-ros2-sqlite-bag-to-mcap-format)  
28. Using rosbag2\_storage\_mcap: Integrating MCAP for Efficient ROS 2 Data Storage \- Blog, accessed May 26, 2026, [https://blog.us.fixstars.com/using-rosbag2\_storage\_mcap/](https://blog.us.fixstars.com/using-rosbag2_storage_mcap/)  
29. The Real-time Publish-Subscribe Protocol (RTPS) DDS Interoperability Wire Protocol Specification, accessed May 26, 2026, [https://www.omg.org/spec/DDSI-RTPS/2.2/PDF/changebar](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF/changebar)  
30. Safe DDS: eprosima::safedds::serialization Namespace Reference, accessed May 26, 2026, [https://safe-dds.docs.eprosima.com/main/doxygen/namespaceeprosima\_1\_1safedds\_1\_1serialization.html](https://safe-dds.docs.eprosima.com/main/doxygen/namespaceeprosima_1_1safedds_1_1serialization.html)  
31. Issue \#42 \- Update Section 10.docx, accessed May 26, 2026, [https://issues.omg.org/secure/attachment/16333/Issue%20%2342%20-%20Update%20Section%2010.docx](https://issues.omg.org/secure/attachment/16333/Issue%20%2342%20-%20Update%20Section%2010.docx)  
32. Extensible and Dynamic Topic Types for DDS \- Object Management Group, accessed May 26, 2026, [https://www.omg.org/spec/DDS-XTypes/1.1/PDF](https://www.omg.org/spec/DDS-XTypes/1.1/PDF)  
33. Chapter 4 Data Representation \- RTI Community, accessed May 26, 2026, [https://community.rti.com/static/documentation/connext-dds/current/doc/manuals/connext\_dds\_professional/extensible\_types\_guide/extensible\_types/Data\_Representation.htm](https://community.rti.com/static/documentation/connext-dds/current/doc/manuals/connext_dds_professional/extensible_types_guide/extensible_types/Data_Representation.htm)  
34. Inter-operability issues btw RTI and OpenDDS due to "align to 8 bytes" issue, accessed May 26, 2026, [https://community.rti.com/forum-topic/inter-operability-issues-btw-rti-and-opendds-due-align-8-bytes-issue](https://community.rti.com/forum-topic/inter-operability-issues-btw-rti-and-opendds-due-align-8-bytes-issue)  
35. United Modeling Language 2.0 Proposal \- RTI Community, accessed May 26, 2026, [https://community.rti.com/static/documentation/connext-dds/4.5f/RTI\_Wireshark\_4.5f/doc/RTPS\_Protocol\_v2.1.pdf](https://community.rti.com/static/documentation/connext-dds/4.5f/RTI_Wireshark_4.5f/doc/RTPS_Protocol_v2.1.pdf)  
36. foxglove/omgidl-serialization \- NPM, accessed May 26, 2026, [https://www.npmjs.com/package/@foxglove/omgidl-serialization](https://www.npmjs.com/package/@foxglove/omgidl-serialization)  
37. Function Cloudini::SerializeCompressedPointCloud2 — cloudini\_ros \- ROS Docs, accessed May 26, 2026, [https://docs.ros.org/en/humble/p/cloudini\_ros/generated/function\_conversion\_\_utils\_8hpp\_1aca584d83247ea7c622357bc6a435c796.html](https://docs.ros.org/en/humble/p/cloudini_ros/generated/function_conversion__utils_8hpp_1aca584d83247ea7c622357bc6a435c796.html)  
38. mcap \- Rust \- Docs.rs, accessed May 26, 2026, [https://docs.rs/mcap](https://docs.rs/mcap)  
39. mcap \- crates.io: Rust Package Registry, accessed May 26, 2026, [https://crates.io/crates/mcap](https://crates.io/crates/mcap)  
40. GitHub \- anduril/mcap-rs: A Rust library for reading and writing Foxglove MCAP files, accessed May 26, 2026, [https://github.com/anduril/mcap-rs](https://github.com/anduril/mcap-rs)  
41. mcap-rs \- crates.io: Rust Package Registry, accessed May 26, 2026, [https://crates.io/crates/mcap-rs](https://crates.io/crates/mcap-rs)  
42. GitHub \- foxglove/mcap: MCAP is a modular, performant, and serialization-agnostic container file format, useful for pub/sub and robotics applications., accessed May 26, 2026, [https://github.com/foxglove/mcap](https://github.com/foxglove/mcap)  
43. mcap::read \- Rust \- Docs.rs, accessed May 26, 2026, [https://docs.rs/mcap/latest/mcap/read/index.html](https://docs.rs/mcap/latest/mcap/read/index.html)  
44. ROS2 client library based on RustDDS \- GitHub, accessed May 26, 2026, [https://github.com/Atostek/ros2-client/](https://github.com/Atostek/ros2-client/)  
45. rustdds \- crates.io: Rust Package Registry, accessed May 26, 2026, [https://crates.io/crates/rustdds](https://crates.io/crates/rustdds)  
46. oxidros\_core \- Rust \- Docs.rs, accessed May 26, 2026, [https://docs.rs/oxidros-core/latest/oxidros\_core/](https://docs.rs/oxidros-core/latest/oxidros_core/)  
47. ros\_pointcloud2 \- Rust \- Docs.rs, accessed May 26, 2026, [https://docs.rs/ros\_pointcloud2](https://docs.rs/ros_pointcloud2)  
48. uos/ros\_pointcloud2: A PointCloud2 message conversion library for ROS1 and ROS2., accessed May 26, 2026, [https://github.com/uos/ros\_pointcloud2](https://github.com/uos/ros_pointcloud2)  
49. ros\_pointcloud2 \- crates.io: Rust Package Registry, accessed May 26, 2026, [https://crates.io/crates/ros\_pointcloud2](https://crates.io/crates/ros_pointcloud2)  
50. ros\_pointcloud2 \- crates.io: Rust Package Registry, accessed May 26, 2026, [https://crates.io/crates/ros\_pointcloud2/0.5.0-rc.1](https://crates.io/crates/ros_pointcloud2/0.5.0-rc.1)  
51. cdr-encoding \- Lib.rs, accessed May 26, 2026, [https://lib.rs/crates/cdr-encoding](https://lib.rs/crates/cdr-encoding)  
52. Atostek/cdr-encoding: OMG Common Data Representation in Rust and Serde \- GitHub, accessed May 26, 2026, [https://github.com/Atostek/cdr-encoding](https://github.com/Atostek/cdr-encoding)  
53. cdr \- Rust \- Docs.rs, accessed May 26, 2026, [https://docs.rs/cdr](https://docs.rs/cdr)  
54. cdr \- Rust, accessed May 26, 2026, [https://hrektts.github.io/cdr-rs/](https://hrektts.github.io/cdr-rs/)  
55. cdr \- crates.io: Rust Package Registry, accessed May 26, 2026, [https://crates.io/crates/cdr](https://crates.io/crates/cdr)  
56. CLI \- MCAP, accessed May 26, 2026, [https://mcap.dev/guides/cli](https://mcap.dev/guides/cli)  
57. 'No parser available for encoding \[cdr\] nor \[ros2msg\]' after mcap conversion from ros2 db3 bag · Issue \#878 \- GitHub, accessed May 26, 2026, [https://github.com/PlotJuggler/PlotJuggler/issues/878](https://github.com/PlotJuggler/PlotJuggler/issues/878)  
58. Welcome to pointcloudset's documentation\! \- GitHub Pages, accessed May 26, 2026, [https://virtual-vehicle.github.io/pointcloudset/](https://virtual-vehicle.github.io/pointcloudset/)  
59. Processing .bag files to convert to .las \- Robotics Stack Exchange, accessed May 26, 2026, [https://robotics.stackexchange.com/questions/114312/processing-bag-files-to-convert-to-las](https://robotics.stackexchange.com/questions/114312/processing-bag-files-to-convert-to-las)  
60. Writing JSON \- MCAP, accessed May 26, 2026, [https://mcap.dev/guides/python/json](https://mcap.dev/guides/python/json)  
61. How to convert data to rosbags : r/ROS \- Reddit, accessed May 26, 2026, [https://www.reddit.com/r/ROS/comments/nrefpt/how\_to\_convert\_data\_to\_rosbags/](https://www.reddit.com/r/ROS/comments/nrefpt/how_to_convert_data_to_rosbags/)  
62. Publishing messages using YAML files — ROS 2 Documentation, accessed May 26, 2026, [https://docs.ros.org/en/rolling/Tutorials/Intermediate/Publishing-Messages-Using-YAML-Files.html](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Publishing-Messages-Using-YAML-Files.html)  
63. Raw Messages Panel \- Avala Documentation \- Avala AI, accessed May 26, 2026, [https://avala.ai/docs/visualization/panels/raw-messages-panel](https://avala.ai/docs/visualization/panels/raw-messages-panel)  
64. Parse rostopics to JSON \- ROS Answers archive, accessed May 26, 2026, [https://answers.ros.org/question/358240/](https://answers.ros.org/question/358240/)  
65. Sample Lidar Data \- Ouster, accessed May 26, 2026, [https://ouster.com/downloads/sample-lidar-data](https://ouster.com/downloads/sample-lidar-data)  
66. ouster-sdk/python/src/ouster/sdk/examples/pcap.py at master · ouster-lidar/ouster-sdk \- GitHub, accessed May 26, 2026, [https://github.com/ouster-lidar/ouster\_example/blob/master/python/src/ouster/sdk/examples/pcap.py](https://github.com/ouster-lidar/ouster_example/blob/master/python/src/ouster/sdk/examples/pcap.py)  
67. mcap/python/examples/jsonschema/pointcloud\_csv\_to\_mcap.py at main \- GitHub, accessed May 26, 2026, [https://github.com/foxglove/mcap/blob/main/python/examples/jsonschema/pointcloud\_csv\_to\_mcap.py](https://github.com/foxglove/mcap/blob/main/python/examples/jsonschema/pointcloud_csv_to_mcap.py)  
68. Example multimodal datasets \- Foxglove, accessed May 26, 2026, [https://foxglove.dev/examples](https://foxglove.dev/examples)  
69. isaac\_ros\_data\_recorder — isaac\_ros\_docs documentation \- NVIDIA Isaac ROS, accessed May 26, 2026, [https://nvidia-isaac-ros.github.io/v/release-3.1/repositories\_and\_packages/isaac\_ros\_nova/isaac\_ros\_data\_recorder/index.html](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nova/isaac_ros_data_recorder/index.html)

[image1]: <data:image/png;base64,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>

[image2]: <data:image/png;base64,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>

[image3]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACsAAAAaCAYAAAAue6XIAAACIUlEQVR4Xu2WP0iVURjG3yIq+4uTS5a1VJDQktQk5GAQFBY5h4NIRDTYIFYogjUozYESREsRRTU0OBVBqUSQhkM1JaIkDTlJQz6P59zu8eF89zuB3yDcH/y493vec895L993z7lmVTYGt+EDDXPYDj/D81oomhm4U8OAkxp4GuFPeEwLRbEfXtLQsxVeNddQFsPwg4apdMHH5hb4C+fhmTUj1jIKN2to5c9+9++zOAT/wHNaqMQWeA3eh6fNNbAJtppb7GZ56D/43C1p6Gnwr4NWuVnyBH7SMMYRuAh/wX1SK9FpbsGPkvMOzEmm3LH8Zg9b/phV+I048LLkIU3mxtDdQf4NvgiuY6Q0S35rEIMTvddQaLZys/wFk73++l5pUAapzU5ooDSYmyjv4S49BrTGZyf8dZ+/ziK12TENlLfePJbNLXgjyM76rCPIYqQ2OwL3aBgyCZ9pGIGL8fncFmTcMZhfCbIYqc0+hDs0DOGAVxoKPF242AXJj/u8R3IltdmXGijdcNbimzrhVjYFr2sBHDDXxF0tCKnNvtEgxri5E4TPI5s+BQfgAnxqbh/O4jX8qmEADxXOwWYrzVNvaV9odcJ2+Mjcwtw3eWs5QR63LHsR5jF5SircjbLmWTeO2vos0g9/aFgEeSdYHryz07BXC0XQAg9q+B9cNPebqdNCUXyBbRomMASfa1g0/Nf0DtZqoQLcdfifZJcWqlTZ6KwAF6V2eBvbTBgAAAAASUVORK5CYII=>

[image4]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEsAAAAaCAYAAAD/nKG4AAADuElEQVR4Xu2YaahNURTHl3lKMk/hmRUKJR9IZiIpMoRESpIPyFQKn5SIZIoMH0QRoRQR75WZDJHhg1BmyhiKxPpbe9+z7nrn3Hvfvdd16P7q3337v/cZ9j57r732IypS5F9gMeuwNWNKO9Yj9/tXwMPrq/Ip1lnWK1Yj5ceF6axb1iwELVgzjHeQ9dMpjoMFLlsjG2ax9rJeknT2CGtCUotktrKqW5O5S/EerMmsVtbMlCqs2axdrGGuDM6QdHojq5LzPDVZ743nuUnxHqyqrG3WTEdHktjygdXW1HnGkHT8Gauy8jED36my5grFe7AA3q+vNVPhO4VZFUUbCmJQJ+XfZ51QZU3UYA1ifWI9YW1n9Uqu/k0z1lXWG9Y+1mhWGauUkjeSXMH7LbFmKnDBdSq/xDSdKRis4c6r58pbfCND2GANZX1llZB0eibro6oH2NJfkMx4PGMlyX36u99RiZYBzVkPWGtI+tGD5PkIBXtIBj8M3G+nNaNoTXLBRFth8MsQau+8nq68yjcy2MFqwnrLOpRoISBHG6/KG0iu8yB2YibiF7Mw7KMeYy2ioPPY6dBusPM2B02T+E6ygWVEKeuSNQ0NSDqJh65XPmYYPOyeYdjBwpdHGYOsqeP8pq68zpU9iJEYrGrK00wiWdroi/6YYITzdihPg1zrtjWjwBc4ak3DapIHIpA3VL5fFvOUp9GDVdv9DdkYhV0J/gBXxvLERlLDlRe4+nR8Y90z3lqSa8cZ34OYe8OaUeym6ADt+UzywDnG7+b8Fcb36MHC7ECsQnmgbsTUdT7iDMDMuEjy1RHPEHfmurpU4B46fmLZPiX5yEhxwnhNsnFkxHySYBo1xRuTvAS+kKUlSR2WTRjXKBgs4PO1aYkWQheSd/B53VSSoFxRcG+dPPswscmVh6g6zw9Kv7KSOE9y02UkL9yHZAfCUjgeNAsFD0IKYOnOek5yXx28lzqvtytjdj5O1Ar4CFhSOF96nWTV0o0MiKvohwbvjmfhurGmzoP6KdZMB9Y0vibWMHaW5SS5VTp85y3wtDQjSYI9EuH9JKcFDXboO1T+Hhj8qONJV9ZC4/UjObIhjumNSYOZhbSjICAnwgPzBeLXQ9ZpkhDgQbBH/MpmeaYCB/6CcsAaOYBZhlmEVMCCc1w+PwxiqU+wCwamO4J0PihhfSFJVC1lrHPWzAGkRGEJ7h8H2zt2sXyBxBXJMg732P4R4O0ZM1uQxlygCp4J80kHktwo6gwWJ7AR4N9QRYoUKfJf8gvP+uMJ/6KLVgAAAABJRU5ErkJggg==>

[image5]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABkAAAAZCAYAAADE6YVjAAABCElEQVR4Xu2Suw4BQRSGDxFKiY5KofIeIhG3QkMhIaFVKD2BWi/xDBKPQCEhQoFqE41Sq+I/2dkYx6yMZAvFfskXs//ZcXYuRCEB0oU5GQZJHT7gUhaCZA9P5DZaiZof/O5Yht9owQ65E1kb+L2mDP2owSiMwTP91iQjQz922pgPnycXtcxEGq5laCICN7Ahcq9RWeQ6M1iQoQn+8wO5W6XD28ZN+AP8uMKEDCW8ii1sy4LCuwAVWQB5sjw3XgUfMn+1Ce8CmK5znyybODAuQw2uOWRudIE3NeYdmarfDwYyMDCk17aVVJZVzwv1PIITNX6jSq/JtnqrScE7nMMePMKkqoWEhPwTT5ksP+lTUd6ZAAAAAElFTkSuQmCC>