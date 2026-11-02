#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source scripts/env.sh
colcon test --packages-select bof_vision --event-handlers console_direct+
colcon test-result --test-result-base build --verbose
