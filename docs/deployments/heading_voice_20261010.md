# 航向参考与语音接口部署记录（2026-10-10）

已将本轮源码部署到 `/home/orangepi/pi_upper`，从在用仓库重新完成 Linux CMake
构建，并更新 `build/uart` 与 `build-turn/uart` 中的 uart_vel、uart_turn、uart_speak、
uart_rfid_speak。两个目录中的对应程序来自同一构建，SHA256 已逐项校验。

部署前提交：`be62e9e394a0c3fe09197b16101cfca0db0b70a4`。此前巡检配置的 recognition.roi[3] 从 0.75 调整到 0.8
单独保存在 c0b85ed；相机高度 13 cm 和 a4dd513 的手动标定网页提交均保留。
本轮没有改动该巡检配置、视觉服务配置或模型。

## 验证

- 在用源码 Linux 构建通过，开启 -Werror；6 组 UART CTest 通过。
- 在用源码的 142 项相关 Python 测试通过。
- 实际安装的 `build/uart/uart_vel` 对 PTY 假 MCU 的 7 场景通过。
- 两端 49 个变更源文件与离线验收清单的 SHA256（统一 LF）一致。
- 没有启动实车导航、访问真实相机/串口或发送真实运动命令。

原源码及可执行文件备份位于 `/home/orangepi/deployment-backups/heading-voice-20261010-c0b85ed`。
本次日志保存在 `/tmp/codex-heading-voice-at8u9mk2/deployment_*.log` 与 JSON 文件中。

## 当前运行条件

航向参考校正默认启用；STM32 还没有构建/烧录本轮固件，旧固件若没有 HELLO
capability bit4，上位机会在 ARM 前拒绝拓扑行驶。必须先完成下位机配套固件部署。
本轮只校正动作航向参考，不重估 gyro bias，不清原始 yaw 或 ODOM。

`config/speech.json` 的 32 条音轨时长仍未配置。UID 原有播报路径保留，
嫌疑人/刀具连续播报待补齐真实音轨时长后启用；没有进行实物出声或漂移验收。

接口详情见 [航向参考](../reference/comm/heading_reference.md) 和
[语音](../reference/comm/speech.md)。
