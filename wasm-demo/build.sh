#!/usr/bin/env bash
# Build script for rubipont WASM demo
# Requires: wasm-pack, npm (for local server)

set -euo pipefail
cd "$(dirname "$0")"

echo "Building rubipont-core for WASM..."
cd ../rubipont-core

# Build with wasm-pack (excludes mcap-io which needs native C deps)
wasm-pack build --target web --out-dir ../wasm-demo/pkg -- --no-default-features --features wasm

echo ""
echo "WASM module built at wasm-demo/pkg/"
echo ""
echo "To serve the demo:"
echo "  cd wasm-demo && python3 -m http.server 8080"
echo "Then open http://localhost:8080 in a browser"
echo ""
echo "Note: WASM build excludes MCAP and ROS bag formats (require native C deps)."
echo "Supported: LAS, LAZ, PCD, E57"
