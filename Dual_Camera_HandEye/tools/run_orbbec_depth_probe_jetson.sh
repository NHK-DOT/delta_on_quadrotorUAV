#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/../build/orbbec_depth_probe"
mkdir -p "${BUILD_DIR}"

g++ -std=c++11 \
  "${SCRIPT_DIR}/orbbec_depth_probe.cpp" \
  -I/usr/local/include \
  -L/usr/local/lib \
  -lOrbbecSDK \
  -Wl,-rpath,/usr/local/lib \
  -o "${BUILD_DIR}/orbbec_depth_probe"

LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}" \
  "${BUILD_DIR}/orbbec_depth_probe"

