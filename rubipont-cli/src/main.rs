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
        /// Target CRS EPSG code (e.g., 3857 for Web Mercator)
        #[arg(long)]
        target_crs: Option<u32>,
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
        Commands::Convert { input, output, target_crs } => {
            match rubipont_core::pipeline::convert(&input, &output, target_crs) {
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
            println!("  .las  — ASPRS LAS 1.2/1.4 (read/write)");
            println!("  .laz  — Compressed LAS       (read/write)");
            println!("  .pcd  — Point Cloud Data     (read/write)");
            println!("  .e57  — ASTM E57             (read/write)");
            #[cfg(feature = "mcap-io")]
            println!("  .mcap — ROS 2 MCAP           (read/write)");
            #[cfg(feature = "mcap-io")]
            println!("  .bag  — ROS 1 bag            (read)");
        }
    }
}

fn show_info(input: &std::path::Path) -> Result<(), rubipont_core::error::RubipontError> {
    println!("{}", rubipont_core::pipeline::format_info(input)?);
    Ok(())
}
