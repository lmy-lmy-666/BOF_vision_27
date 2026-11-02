#!/usr/bin/env bash
set -euo pipefail
cd -P "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
# One build and one configuration path. CLI can select another configuration.
defaults=(--config "$PWD/src/bof_bringup/config/sentry.yaml")
for argument in "$@"; do
  if [[ "$argument" == --config ]]; then defaults=(); break; fi
done
exec ros2 run bof_vision bof_live "${defaults[@]}" "$@"
