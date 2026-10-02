#!/usr/bin/env bash
# 编译在独立 build/ 下，不复制或覆盖系统 RKNN runtime。
set -eu
root="$(cd "$(dirname "$0")" && pwd)"
include="${RKNN_INCLUDE:-/home/orangepi/Downloads/rknn_model_zoo-main/3rdparty/rknpu2/include}"
test -f "$include/rknn_api.h"
mkdir -p "$root/build"
g++ -O2 -std=c++17 -fPIC -shared "$root/backend.cpp" -I"$include" -L/usr/lib -lrknnrt -o "$root/build/libroad_rknn.so"
