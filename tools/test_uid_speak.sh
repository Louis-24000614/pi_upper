#!/bin/sh
# 独立 UID 读卡播报测试，复用现有 uart_rfid_speak，不改变导航或配置。
# 该程序只发送 HELLO 和 SPEAK_AUDIO，不使能电机、不发送运动命令。
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
uart_bin="$project_root/build-turn/uart/uart_rfid_speak"

case "${1-}" in
    -h|--help)
        cat <<'EOF'
用法：sh tools/test_uid_speak.sh [--device 串口] [--baud 波特率]
默认串口 /dev/ttyS6，波特率 921600（与当前导航配置一致）。

贴上 1～12 号卡后播报对应编号，同一世代的卡持续在场不重复播报。
Ctrl+C 结束。请先退出导航或其他占用同一串口的程序。
“播报 N 号已收下”仅表示收到 ACK，不代表实际发声或播放完成。
本脚本不修改配置，不启动车辆运动。
EOF
        exit 0
        ;;
esac

if [ ! -x "$uart_bin" ]; then
    echo "缺少可执行程序：$uart_bin" >&2
    echo "请在仓库根目录构建已有目标：cmake --build build-turn --target uart_rfid_speak" >&2
    exit 1
fi

exec "$uart_bin" "$@"
