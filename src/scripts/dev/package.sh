#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
mkdir -p build/dist
# Package the checked working tree, including models and GitHub configuration.
# Validate before opening the archive, so a failed check preserves the last package.
bof_file_list="$(mktemp)"
trap 'rm -f "$bof_file_list"' EXIT
python3 src/scripts/dev/check_repo.py --list-files > "$bof_file_list"
tar --null -czf build/dist/BOF_vision_27-source.tar.gz -T "$bof_file_list"
(
  cd build/dist
  sha256sum BOF_vision_27-source.tar.gz > BOF_vision_27-source.tar.gz.sha256
)
echo "Created: $(pwd)/build/dist/BOF_vision_27-source.tar.gz"
