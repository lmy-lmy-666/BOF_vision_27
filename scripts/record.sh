#!/usr/bin/env bash
# Record a rosbag for replay / calibration. Usage: bash scripts/record.sh PRESET [NAME]
#   aim      poses, attitude, commands, solutions (match/training replay, default)
#   images   aim + camera images (large: ~100 MB/s at 1440x1080 bgr8)
#   handeye  camera images + attitude (bof_handeye)
#   latency  poses + attitude (bof_aim_replay --mode latency)
set -euo pipefail
cd -P "$(dirname "${BASH_SOURCE[0]}")/.."
preset="${1:-aim}"; name="${2:-$preset}"
aim=(/perception/armor_poses /gimbal/imu /gimbal/state /gimbal/command /aim/solution /tracking/spatial_target
     /sentry/aim_directive /camera/generation /diagnostics)
case "$preset" in
  aim) topics=("${aim[@]}") ;;
  images) topics=("${aim[@]}" /camera/image_raw) ;;
  handeye) topics=(/camera/image_raw /gimbal/imu) ;;
  latency) topics=(/perception/armor_poses /gimbal/imu) ;;
  *) echo "preset must be aim, images, handeye or latency" >&2; exit 2 ;;
esac
out="$PWD/artifacts/bags/$(date +%Y%m%d_%H%M%S)_$name"
mkdir -p "$(dirname "$out")"
source scripts/env.sh
echo "recording ${topics[*]} -> $out (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$out" "${topics[@]}"
