use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "rp", about = "rubipont — LiDAR format translator")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Convert a point cloud file between formats
    Convert {
        /// Source file path
        input: PathBuf,
        /// Output file path (format auto-detected from extension)
        output: PathBuf,
    },
    /// Show information about a point cloud file
    Info {
        /// File path to inspect
        input: PathBuf,
    },
    /// List supported formats
    Formats,
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Convert { input, output } => {
            match rubipont_core::pipeline::convert(&input, &output) {
                Ok(()) => {
                    eprintln!("Converted {} → {}", input.display(), output.display());
                }
                Err(e) => {
                    eprintln!("Error: {}", e);
                    std::process::exit(1);
                }
            }
        }
        Commands::Info { input } => {
            match show_info(&input) {
                Ok(()) => {}
                Err(e) => {
                    eprintln!("Error: {}", e);
                    std::process::exit(1);
                }
            }
        }
        Commands::Formats => {
            println!("Supported formats:");
            println!("  .las  — ASPRS LAS 1.2 (read/write)");
            println!("  .laz  — Compressed LAS (read/write)");
            println!("  .pcd  — Point Cloud Data (read/write)");
        }
    }
}

fn show_info(input: &std::path::Path) -> Result<(), rubipont_core::error::RubipontError> {
    use rubipont_core::format;
    use rubipont_core::pipeline::{PointCloudReader, extension};

    let ext = extension(input);
    let reader: Box<dyn PointCloudReader> = match ext {
        e if format::las::detect(e) => Box::new(format::las::LasReader::new(input)?),
        e if format::laz::detect(e) => Box::new(format::laz::LazReader::new(input)?),
        e if format::pcd::detect(e) => Box::new(format::pcd::PcdReader::new(input)?),
        _ => return Err(rubipont_core::error::RubipontError::UnsupportedFormat(ext.into())),
    };

    let layout = reader.layout();
    let metadata = reader.metadata();

    println!("File: {}", input.display());
    println!("Points: {}", layout.num_points);
    println!("Point size: {} bytes", layout.point_size);
    println!("Integer coords: {}", layout.has_integer_coords);

    if let Some((sx, sy, sz)) = &metadata.coordinate_scale {
        println!("Scale: ({}, {}, {})", sx, sy, sz);
    }
    if let Some((ox, oy, oz)) = &metadata.coordinate_offset {
        println!("Offset: ({}, {}, {})", ox, oy, oz);
    }

    Ok(())
}
