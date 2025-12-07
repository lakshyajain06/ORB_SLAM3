#!/bin/bash

# Paths
VOCAB="$HOME/Documents/Code/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIB="$HOME/Documents/Code/ORB_SLAM3/data/custom/camera.yaml"
RUN_NAME="run9"
DATA="$HOME/Documents/Code/ORB_SLAM3/data/custom/$RUN_NAME"

# Run ORB-SLAM3 monocular example
./Examples/Monocular/mono_euroc \
    "$VOCAB" \
    "$CAMERA_CALIB" \
    "$DATA" \
    "$DATA/cam0/times.txt"
