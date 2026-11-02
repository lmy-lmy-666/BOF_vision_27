#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
mkdir -p artifacts/dist
# Explicit source allowlist keeps recordings, weights and build caches out.
tar --exclude='__pycache__' --exclude='models/tongji_yolov5' --exclude='models/tongji_yolov8' --exclude='models/tongji_yolo11' \
  --exclude='models/shenzhen' \
  -czf artifacts/dist/BOF_vision_27-source.tar.gz \
  构建方式说明.txt README.md LICENSE .gitignore src scripts docs third_party models
sha256sum artifacts/dist/BOF_vision_27-source.tar.gz > artifacts/dist/BOF_vision_27-source.tar.gz.sha256
echo "Created: $(pwd)/artifacts/dist/BOF_vision_27-source.tar.gz"
