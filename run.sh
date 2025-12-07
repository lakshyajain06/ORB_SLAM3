#!/bin/bash

# 1. Get the directory where THIS script lives
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# CHeck for input
if [ -z "$1" ]; then
    echo "Error: No run name provided."
    echo "Usage: $0 <run_name>"
    echo "Example: $0 run9"
    exit 1
fi

# 2. Assign the first argument ($1) to RUN_NAME
RUN_NAME="$1"

2. Define Paths RELATIVE to the script location
# Assuming your folder structure is:
# ORB_SLAM3/
# ├── Vocabulary/
# ├── Examples/
# ├── data/
# └── run_slam.sh  <-- The script is here
VOCAB="$SCRIPT_DIR/Vocabulary/ORBvoc.txt"
CAMERA_CALIB="$SCRIPT_DIR/data/custom/camera.yaml"
DATA="$SCRIPT_DIR/data/custom"

echo "Launching ORB-SLAM3 for: $RUN_NAME"

# 4. Run ORB-SLAM3 monocular example
./Examples/Monocular/mono_euroc \
    "$VOCAB" \
    "$CAMERA_CALIB" \
    "$DATA" \
    "$RUN_NAME"