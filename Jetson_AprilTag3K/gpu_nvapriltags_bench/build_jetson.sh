#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

CXX=${CXX:-g++}
OPENCV_FLAGS="$(pkg-config --cflags --libs opencv4 2>/dev/null || pkg-config --cflags --libs opencv)"
CUDA_INC=/usr/local/cuda/include
CUDA_LIB=/usr/local/cuda/lib64
if [ ! -d "$CUDA_LIB" ]; then
  CUDA_LIB=/usr/local/cuda-10.2/targets/aarch64-linux/lib
fi

"$CXX" -O3 -std=c++14 \
  -I. -I"$CUDA_INC" \
  nv_gpu_apriltag_bench.cpp \
  libapril_tagging.a \
  $OPENCV_FLAGS \
  -L"$CUDA_LIB" -lcudart -lpthread -ldl \
  -Wl,-rpath,"$CUDA_LIB" \
  -o nv_gpu_apriltag_bench

echo "built $(pwd)/nv_gpu_apriltag_bench"
