#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
source src/scripts/env.sh
export ROS_DOMAIN_ID=188
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 src/bof_vision/test/integration/config_ros.py "$(ros2 pkg prefix bof_vision)/lib/bof_vision/bof_replay"
