#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Change to Isaac Sim directory
cd ~/isaacsim

# Add infinigen_sdg_utils to Python path
export PYTHONPATH="$HOME/isaacsim/standalone_examples/replicator/infinigen:$PYTHONPATH"

# Check if arguments are provided
if [ $# -eq 0 ]; then
    echo "Error: No USD path provided."
    echo "Usage: $0 <usd_path> [options]"
    echo "Example: $0 omniverse/0730/APT400_topview/export_scene.blend/export_scene.usdc"
    exit 1
else
    # Pass all arguments to the Python script
    ./python.sh "$SCRIPT_DIR/infinigen_launchoutput.py" "$@"
fi