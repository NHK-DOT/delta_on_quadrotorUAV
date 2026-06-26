#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/../build/orbbec_depth_fast_preview"
mkdir -p "${BUILD_DIR}"

OPENCV_FLAGS="$(pkg-config --cflags --libs opencv 2>/dev/null || pkg-config --cflags --libs opencv4)"

g++ -std=c++11 \
  "${SCRIPT_DIR}/orbbec_depth_fast_preview.cpp" \
  -I/usr/local/include \
  -L/usr/local/lib \
  -lOrbbecSDK \
  ${OPENCV_FLAGS} \
  -Wl,-rpath,/usr/local/lib \
  -o "${BUILD_DIR}/orbbec_depth_fast_preview"

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
exec "${BUILD_DIR}/orbbec_depth_fast_preview"
