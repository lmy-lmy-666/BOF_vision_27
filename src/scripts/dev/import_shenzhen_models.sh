#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
if [ "$#" -ne 1 ]; then
  echo 'Usage: bash src/scripts/dev/import_shenzhen_models.sh /path/to/rmcs_auto_aim_v2' >&2
  exit 2
fi
source_repo="$1"
expected_0526=b53470b42a5778aca8f3382ebe3c623482599cbf8ebad2e9a0dcc0c7f42c9255
expected_0708=0a535eb10d0cc7c9fe3151f4ddba81418bbec188c6389cadc3850e8771c7ab78
test "$(sha256sum "$source_repo/models/shenzhen-0526.onnx" | cut -d' ' -f1)" = "$expected_0526" || { echo 'Unexpected 0526 hash' >&2; exit 1; }
test "$(sha256sum "$source_repo/models/shenzhen-0708.onnx" | cut -d' ' -f1)" = "$expected_0708" || { echo 'Unexpected 0708 hash' >&2; exit 1; }
mkdir -p src/models/shenzhen
cp -- "$source_repo/models/shenzhen-0526.onnx" src/models/shenzhen/
cp -- "$source_repo/models/shenzhen-0708.onnx" src/models/shenzhen/
echo 'Imported pinned evaluation models; see src/models/README.md.'
