#!/usr/bin/env bash
# Record the topics that make up one LeRobot frame into a rosbag2 (mcap).
#   scripts/record_demos.sh bags/my_demos
# Stop with Ctrl+C when you have enough episodes.
set -euo pipefail
OUT=${1:-bags/demos_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$(dirname "$OUT")"
exec ros2 bag record -s mcap -o "$OUT" \
  /joint_states /omy/action /omy/episode /front_cam/image_raw /wrist_cam/image_raw
