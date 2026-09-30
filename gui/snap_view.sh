#!/usr/bin/env bash
# 全屏查看一路摄像头并拍照。先关掉上位机，避免两路程序抢同一个摄像头。
# HDMI 模式与 run.sh 相同：1024x600 在这块板上会黑屏。

set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/orangepi/.Xauthority}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# 与主界面共用 Conda 环境，避免两套解释器加载不同版本的 Qt/OpenCV。
PYTHON="${PYTHON:-/home/orangepi/miniconda3/envs/pi_upper/bin/python}"
ROOT="$(cd "$(dirname "$0")" && pwd)"

xrandr --output HDMI-1 --primary --mode 1280x720 --rate 60
xrandr --output HDMI-1 --set color_format rgb || true
xrandr --output HDMI-1 --set output_hdmi_dvi force_hdmi || true

cd "$ROOT"
exec "$PYTHON" snap_view.py "$@"
