#!/usr/bin/env bash
# 通过 HTTP/MJPEG 把香橙派摄像头画面提供给远程主机浏览器。

set -euo pipefail

DEVICE="${1:-/dev/video0}"
PORT="${PORT:-8080}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"

if [[ ! -e "$DEVICE" ]]; then
  echo "找不到摄像头设备: $DEVICE" >&2
  exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "找不到 ffmpeg" >&2
  exit 1
fi

echo "仅监听香橙派本机，避免把摄像头画面暴露到局域网。"
echo "请先在 VS Code 转发端口 ${PORT}，再在主机浏览器打开:"
echo "http://127.0.0.1:${PORT}/camera.mjpg"
echo "按 Ctrl+C 停止；运行期间不要让其他程序占用 ${DEVICE}。"

exec ffmpeg \
  -hide_banner \
  -loglevel warning \
  -f v4l2 \
  -input_format mjpeg \
  -video_size "${WIDTH}x${HEIGHT}" \
  -framerate "$FPS" \
  -i "$DEVICE" \
  -an \
  -vsync 0 \
  -c:v mjpeg \
  -q:v 5 \
  -f mpjpeg \
  -listen 1 \
  "http://127.0.0.1:${PORT}/camera.mjpg"
