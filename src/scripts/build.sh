#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
# Keep recordings and archived source/build trees out of colcon discovery.
mkdir -p build install log
touch log/COLCON_IGNORE
# Standard workspace: build/, install/, log/. No alias or external cache.
colcon build --base-paths src --symlink-install --packages-up-to bof_bringup \
  --cmake-args --no-warn-unused-cli -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF \
  -DBOF_BUILD_LIVE=ON -DBOF_WITH_MVS=OFF -DBOF_WITH_OPENVINO=OFF "$@"
echo "Build complete. Load this workspace: source install/setup.bash (or source src/scripts/env.sh)"
