#!/usr/bin/env bash
# 编译在独立 build/ 下，不复制或覆盖系统 RKNN runtime。
set -eu
root="$(cd "$(dirname "$0")" && pwd)"
include="${RKNN_INCLUDE:-/home/orangepi/Downloads/rknn_model_zoo-main/3rdparty/rknpu2/include}"
test -f "$include/rknn_api.h"
mkdir -p "$root/build"
g++ -O2 -std=c++17 -fPIC -shared "$root/backend.cpp" -I"$include" -L/usr/lib -lrknnrt -o "$root/build/libroad_rknn.so"

# RGA 只供离线硬件实验使用；默认 RKNN 构建不应额外要求 librga 开发头文件。
# 需要复现该候选时显式 BUILD_RGA=1 bash build.sh，产物仍放在私有 build/ 下。
if [ "${BUILD_RGA:-0}" = "1" ]; then
  g++ -O2 -std=c++17 -fPIC -shared "$root/rga_letterbox.cpp" \
    -I/usr/include/rga -L/usr/lib -lrga -o "$root/build/librga_letterbox.so"
fi
