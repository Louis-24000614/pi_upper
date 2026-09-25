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

# EDID data on small HDMI panels is not always reliable.  Do not abort the
# camera viewer merely because a particular mode (or connector name) is absent.
if XRANDR_INFO="$(xrandr --query 2>&1)"; then
  OUTPUT="${XRANDR_OUTPUT:-$(awk '/ connected/{print $1; exit}' <<<"$XRANDR_INFO")}"

  if [[ -n "$OUTPUT" ]]; then
    modes_for_output() {
      awk -v output="$OUTPUT" '
        $1 == output && $2 == "connected" { in_output = 1; next }
        in_output && /^[^[:space:]]/ { exit }
        in_output && /^[[:space:]]/ { print $1 }
      ' <<<"$XRANDR_INFO"
    }

    AVAILABLE_MODES="$(modes_for_output)"
    MODE=""
    for candidate in 1280x720 1920x1080; do
      if grep -Fxq "$candidate" <<<"$AVAILABLE_MODES"; then
        MODE="$candidate"
        break
      fi
    done

    if [[ -n "$MODE" ]]; then
      xrandr --output "$OUTPUT" --primary --mode "$MODE" --rate 60 2>/dev/null \
        || xrandr --output "$OUTPUT" --primary --mode "$MODE" 2>/dev/null \
        || printf 'warning: could not switch %s to %s; keeping the current mode\n' "$OUTPUT" "$MODE" >&2
    else
      printf 'warning: %s offers neither 1280x720 nor 1920x1080; keeping the current mode\n' "$OUTPUT" >&2
    fi

    xrandr --output "$OUTPUT" --set color_format rgb 2>/dev/null || true
    xrandr --output "$OUTPUT" --set output_hdmi_dvi force_hdmi 2>/dev/null || true
  else
    printf 'warning: xrandr found no connected display; continuing without changing the mode\n' >&2
  fi
else
  printf 'warning: xrandr is unavailable (%s); continuing without changing the mode\n' "$XRANDR_INFO" >&2
fi

cd "$ROOT"
exec "$PYTHON" snap_view.py "$@"
