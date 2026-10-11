#!/bin/sh
# 安装与启用识别服务、PWM 权限准备；只在 --start-models 时启动模型。
set -eu
project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "$(id -u)" != 0 ]; then
    echo "请使用 sudo 运行安装脚本。" >&2
    exit 1
fi
if [ "$project_root" != /home/orangepi/pi_upper ]; then
    echo "这些服务用于 /home/orangepi/pi_upper；其他安装路径需先调整 unit。" >&2
    exit 1
fi
case "${1-}" in
    "") start_models=no ;;
    --start-models) start_models=yes ;;
    *) echo "用法：sudo sh tools/install_inspection_services.sh [--start-models]" >&2; exit 2 ;;
esac
test -x "$project_root/vision/arcface-lite/.venv/bin/python"
test -x /home/orangepi/.venvs/pi-upper-culvert-services/bin/python
test -f "$project_root/vision/arcface-lite/face_db.npz"
install -d -m 755 /usr/local/lib/pi-upper
install -m 644 "$project_root/tools/prepare_pwm14m0.py" /usr/local/lib/pi-upper/prepare_pwm14m0.py
for unit in pi-upper-pwm-prepare pi-upper-face pi-upper-knife; do
    install -m 644 "$project_root/deploy/systemd/$unit.service" "/etc/systemd/system/$unit.service"
done
systemd-analyze verify /etc/systemd/system/pi-upper-pwm-prepare.service /etc/systemd/system/pi-upper-face.service /etc/systemd/system/pi-upper-knife.service
systemctl daemon-reload
systemctl enable pi-upper-pwm-prepare.service pi-upper-face.service pi-upper-knife.service
# 仅导出/核对/恢复权限，脚本不写 enable、duty_cycle、period、polarity。
systemctl restart pi-upper-pwm-prepare.service
if [ "$start_models" = yes ]; then
    systemctl start pi-upper-face.service pi-upper-knife.service
fi
systemctl --no-pager --full status pi-upper-pwm-prepare.service
