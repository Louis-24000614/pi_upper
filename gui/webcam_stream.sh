#!/usr/bin/env bash
# 在局域网浏览器里看一路摄像头，并在页面上拍照。
# 保存的是原始 BGR 帧（默认 1280×720），写到仓库 data/road/。
# 运行期间不要再开上位机或 snap_view.sh，它们会抢同一个摄像头。

set -euo pipefail

DEVICE="${1:-/dev/video0}"
PORT="${PORT:-8080}"
BIND="${BIND:-0.0.0.0}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
ROLE="${ROLE:-recognition_camera}"
PYTHON="${PYTHON:-/home/orangepi/pyside6-venv/bin/python}"
ROOT="$(cd "$(dirname "$0")" && pwd)"

if [[ ! -e "$DEVICE" ]]; then
  echo "找不到摄像头设备: $DEVICE" >&2
  exit 1
fi

if [[ ! -x "$PYTHON" ]]; then
  echo "找不到 Python: $PYTHON" >&2
  exit 1
fi

HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ "$BIND" == "0.0.0.0" && -n "$HOST_IP" ]]; then
  echo "在浏览器打开 http://${HOST_IP}:${PORT}/"
else
  echo "在浏览器打开 http://${BIND}:${PORT}/"
fi
echo "点「拍照」或按空格存 PNG；点「录制」或按 R 存视频。都写到 data/road/。"
echo "看的时候不要再跑 snap_view.sh 或上位机，它们会占用 ${DEVICE}。"

cd "$ROOT"
exec "$PYTHON" webcam_view.py \
  --device "$DEVICE" \
  --bind "$BIND" \
  --port "$PORT" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --fps "$FPS" \
  --role "$ROLE"
