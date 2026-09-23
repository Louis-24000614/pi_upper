#!/bin/bash
# 安装开机默认 HDMI 1280x720（需 sudo）。
# 用法：sudo bash /home/orangepi/Desktop/pi_upper/gui/scripts/install-hdmi-lcd.sh

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用: sudo bash $0" >&2
  exit 1
fi

SCRIPT_SRC="$(cd "$(dirname "$0")" && pwd)/opi-hdmi-lcd.sh"
install -d /usr/local/sbin
install -m 755 "$SCRIPT_SRC" /usr/local/sbin/opi-hdmi-lcd.sh

# 内核启动参数：尽早请求 1280x720，避免卡在 1024x600 无信号
if [ -f /boot/orangepiEnv.txt ]; then
  cp -a /boot/orangepiEnv.txt "/boot/orangepiEnv.txt.bak.hdmi720.$(date +%Y%m%d%H%M%S)"
  if grep -q '^extraargs=' /boot/orangepiEnv.txt; then
    if grep -q 'video=HDMI-A-1:1280x720' /boot/orangepiEnv.txt; then
      :
    else
      sed -i 's/^extraargs=\(.*\)$/extraargs=\1 video=HDMI-A-1:1280x720@60/' /boot/orangepiEnv.txt
    fi
  else
    printf '\nextraargs=video=HDMI-A-1:1280x720@60\n' >> /boot/orangepiEnv.txt
  fi
fi

# LightDM：登录界面出现前切模式
cat > /etc/lightdm/lightdm.conf.d/30-hdmi-lcd.conf <<'EOF'
[Seat:*]
display-setup-script=/usr/local/sbin/opi-hdmi-lcd.sh
EOF

# Xorg：首选模式
cat > /etc/X11/xorg.conf.d/30-hdmi-lcd.conf <<'EOF'
Section "Monitor"
    Identifier "HDMI-1"
    Option "PreferredMode" "1280x720"
    Option "Primary" "true"
EndSection
EOF

# XFCE 会话再执行一次（防止桌面改回）
install -d /etc/xdg/autostart
cat > /etc/xdg/autostart/opi-hdmi-lcd.desktop <<'EOF'
[Desktop Entry]
Type=Application
Name=HDMI LCD 1280x720
Comment=Force HDMI-1 to 1280x720 for external LCD signal
Exec=/usr/local/sbin/opi-hdmi-lcd.sh
OnlyShowIn=XFCE;
X-GNOME-Autostart-enabled=true
EOF

# 用户级配置（XFCE displays）
USER_HOME=/home/orangepi
install -d "$USER_HOME/.config/autostart"
install -d "$USER_HOME/.config/xfce4/xfconf/xfce-perchannel-xml"
cat > "$USER_HOME/.config/autostart/opi-hdmi-lcd.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=HDMI LCD 1280x720
Comment=Force HDMI-1 to 1280x720 for external LCD signal
Exec=/usr/local/sbin/opi-hdmi-lcd.sh
OnlyShowIn=XFCE;
X-GNOME-Autostart-enabled=true
EOF

cat > "$USER_HOME/.config/xfce4/xfconf/xfce-perchannel-xml/displays.xml" <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>

<channel name="displays" version="1.0">
  <property name="ActiveProfile" type="string" value="Default"/>
  <property name="Default" type="empty">
    <property name="HDMI-1" type="string" value="HDMI-1">
      <property name="Active" type="bool" value="true"/>
      <property name="EDID" type="string" value=""/>
      <property name="Resolution" type="string" value="1280x720"/>
      <property name="RefreshRate" type="double" value="60.000000"/>
      <property name="Rotation" type="int" value="0"/>
      <property name="Reflection" type="string" value="0"/>
      <property name="Primary" type="bool" value="true"/>
      <property name="Scale" type="empty">
        <property name="X" type="double" value="1.000000"/>
        <property name="Y" type="double" value="1.000000"/>
      </property>
      <property name="Position" type="empty">
        <property name="X" type="int" value="0"/>
        <property name="Y" type="int" value="0"/>
      </property>
    </property>
  </property>
  <property name="Fallback" type="empty">
    <property name="HDMI-1" type="string" value="HDMI-1">
      <property name="Active" type="bool" value="true"/>
      <property name="EDID" type="string" value=""/>
      <property name="Resolution" type="string" value="1280x720"/>
      <property name="RefreshRate" type="double" value="60.000000"/>
      <property name="Rotation" type="int" value="0"/>
      <property name="Reflection" type="string" value="0"/>
      <property name="Primary" type="bool" value="true"/>
      <property name="Scale" type="empty">
        <property name="X" type="double" value="1.000000"/>
        <property name="Y" type="double" value="1.000000"/>
      </property>
      <property name="Position" type="empty">
        <property name="X" type="int" value="0"/>
        <property name="Y" type="int" value="0"/>
      </property>
    </property>
  </property>
</channel>
EOF

chown -R orangepi:orangepi \
  "$USER_HOME/.config/autostart/opi-hdmi-lcd.desktop" \
  "$USER_HOME/.config/xfce4/xfconf/xfce-perchannel-xml/displays.xml"

# 立刻生效（若当前有图形会话）
if [ -n "${DISPLAY:-}" ] || [ -S /tmp/.X11-unix/X0 ]; then
  DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-/home/orangepi/.Xauthority}" \
    /usr/local/sbin/opi-hdmi-lcd.sh || true
fi

echo "已安装开机默认 HDMI 1280x720。"
echo "请执行: sudo reboot"
echo "重启后检查: xrandr | head  以及  dmesg | grep 'phy lane'"
