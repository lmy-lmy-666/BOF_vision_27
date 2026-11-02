#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
if [ "$#" -ne 1 ]; then
  echo "Usage: bash scripts/dev/import_model.sh /path/to/sp_vision_25" >&2
  exit 2
fi
source_repo="$1"
xml_expected=1cf9f2eb20e0312e8277ec457cfc67fb6867c4ff634d337c9c441a6aea052f8a
bin_expected=aeae1db78494b631ddc1420a04714608b5dcf8bdf11f360ce503309757b1a7de
test "$(sha256sum "$source_repo/assets/yolov5.xml" | cut -d' ' -f1)" = "$xml_expected" || { echo 'Unexpected model XML hash' >&2; exit 1; }
test "$(sha256sum "$source_repo/assets/yolov5.bin" | cut -d' ' -f1)" = "$bin_expected" || { echo 'Unexpected model BIN hash' >&2; exit 1; }
mkdir -p models/tongji_yolov5
cp -- "$source_repo/assets/yolov5.xml" models/tongji_yolov5/yolov5.xml
cp -- "$source_repo/assets/yolov5.bin" models/tongji_yolov5/yolov5.bin
# YOLOv8 (416 input, colour head + tiny_resnet number classifier) and YOLO11 (640 input, 38 classes).
check() { test "$(sha256sum "$source_repo/assets/$1" | cut -d' ' -f1)" = "$2" || { echo "Unexpected hash: $1" >&2; exit 1; }; }
check yolov8.xml 4eecaa89fd3ad4e80b3090c59e9261edddc2b9f4df86df2c5d1778451943fe1e
check yolov8.bin 91299dac7405d01e9a1a35c3ee8850f0f3a6775e4c2c9beac1ef2069f222434f
check yolo11.xml c513dea404aa704d7e4923441b4b51a0f1c3ee54186f5215a04048c297a239c0
check yolo11.bin ace99cf0ce36a7ac5deb0d779ef0a15e952a67da9f57f6e16119e093d713963c
check tiny_resnet.onnx 9f45670da9e61f31b60c12efb08c8a7522ff56cc220c7fa9907283ce7e730179
mkdir -p models/tongji_yolov8 models/tongji_yolo11
cp -- "$source_repo/assets/yolov8.xml" "$source_repo/assets/yolov8.bin" "$source_repo/assets/tiny_resnet.onnx" models/tongji_yolov8/
cp -- "$source_repo/assets/yolo11.xml" "$source_repo/assets/yolo11.bin" models/tongji_yolo11/
echo 'Imported pinned Tongji YOLOv5/YOLOv8/YOLO11 models; see models/README.md for provenance and limits.'
