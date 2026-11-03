#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
source src/scripts/env.sh
# Keep integration test topics separate from any real robot session.
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 src/bof_vision/test/integration/smoke.py "$@"
