#!/bin/bash
# Orange Pi 5 Plus + 外接 HDMI LCD：强制 1280x720。
# 开机 EDID 首选 1024x600 时，Rockchip HDMI PHY 会
# “hdptx phy lane can't ready”，实体屏无信号。

export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ]; then
  if [ -r /home/orangepi/.Xauthority ]; then
    export XAUTHORITY=/home/orangepi/.Xauthority
  elif [ -r /var/run/lightdm/root/:0 ]; then
    export XAUTHORITY=/var/run/lightdm/root/:0
  fi
fi

for _ in 1 2 3 4 5 6 7 8 9 10; do
  if xrandr >/dev/null 2>&1; then
    break
  fi
  sleep 0.3
done

OUT="$(xrandr 2>/dev/null | awk '/ connected/{print $1; exit}')"
[ -n "$OUT" ] || OUT=HDMI-1

xrandr --output "$OUT" --primary --mode 1280x720 --rate 60 2>/dev/null \
  || xrandr --output "$OUT" --primary --mode 1280x720 2>/dev/null \
  || true

xrandr --output "$OUT" --set color_format rgb 2>/dev/null || true
xrandr --output "$OUT" --set output_hdmi_dvi force_hdmi 2>/dev/null || true
exit 0
