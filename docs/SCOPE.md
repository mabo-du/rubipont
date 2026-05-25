# Project Scope: LiDAR Format Translation Library

## Summary
An open-source Rust library (with CLI and optional Python bindings) for zero-copy, lossless conversion between the major LiDAR point cloud data formats. The research explicitly identifies this as a missing tool that causes daily friction in R&D labs and industry teams.

## The Problem
The LiDAR ecosystem is fragmented across incompatible data formats, each tied to a different domain:
- **LAS / LAZ** — geospatial surveying standard (compressed LAZ variant)
- **E57** — terrestrial laser scanning (dense indoor/outdoor scans)
- **PCD** — robotics and autonomous driving (used by PCL/ROS)
- **ROS bag** — robotics middleware; streams sensor data over time

Converting between these formats currently strips metadata, alters point ordering, corrupts coordinate systems, or introduces severe latency. No single tool handles all of them reliably.

## Target Users
- Academic researchers working across geospatial and robotics domains
- Robotics and AV teams ingesting data from multiple sensor vendors
- GIS engineers processing terrestrial laser scan data
- ML engineers preparing training datasets from mixed-source point clouds

## Proposed Solution
A Rust library called **pointbridge** (working name) that:
1. Parses each format natively with zero unnecessary copies in memory
2. Preserves all metadata (intensity, colour, return number, timestamps, coordinate systems)
3. Provides a unified internal representation that maps cleanly to/from all supported formats
4. Exposes a simple CLI: `pointbridge convert input.las output.pcd`
5. Exposes a Rust API for use as a dependency in other projects
6. Optionally exposes Python bindings via PyO3 for researcher adoption

## MVP Scope
- LAS 1.2, 1.3, 1.4 read/write
- LAZ read/write (via integration with laszip or native implementation)
- PCD (binary and ASCII) read/write
- E57 read (write stretch goal)
- CLI with format auto-detection from file extension
- Lossless round-trip test suite

## Stretch Goals
- ROS bag read support (streaming)
- Streaming/chunked API for very large files (>10GB scans)
- Python bindings via PyO3
- WASM build for browser-based tooling
- Coordinate reference system (CRS) transformation support

## Why Rust?
- Zero-copy parsing is idiomatic in Rust (avoid cloning large buffers)
- Safe memory handling for large files without GC pauses
- Compiles to native speed; competitive with C++ implementations
- Fits existing skills (Kontor, CunAIform background)

## Competitive Landscape
- **PDAL** — C++ pipeline tool, powerful but complex to use as a library, lossy in some conversions
- **libLAS** — deprecated
- **Open3D** — Python-first, not designed for lossless translation
- **ROS tools** — ROS-specific, not portable

## Key Risk
The LAZ compression spec is complex. Integration with an existing laszip binding may be the pragmatic path for MVP rather than reimplementing compression from scratch.

## Effort Estimate
- MVP: 4–8 weeks solo, moderate complexity
- Core challenge is format spec reading (well-documented) rather than novel algorithms
