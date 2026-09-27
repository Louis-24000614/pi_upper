#!/usr/bin/env bash
# 在 XFCE / X11 桌面的 HDMI 屏上启动上位机。
# 本机 Rockchip HDMI PHY 在 1024x600 下会 phy poweron failed，导致实体屏无信号；
# 请使用 1280x720（或 1920x1080），并强制 RGB。

set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/orangepi/.Xauthority}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

PYTHON="${PYTHON:-/home/orangepi/pyside6-venv/bin/python}"
ROOT="$(cd "$(dirname "$0")" && pwd)"

xrandr --output HDMI-1 --primary --mode 1280x720 --rate 60
xrandr --output HDMI-1 --set color_format rgb || true
xrandr --output HDMI-1 --set output_hdmi_dvi force_hdmi || true

cd "$ROOT"
exec "$PYTHON" main.py
