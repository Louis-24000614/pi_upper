# 恢复旧固件路口流程（2026-10-10）

按用户要求，当前 `config/nav_camera.yaml` 的 `heading_anchor.enabled` 设为 false，
省略该配置时也默认关闭。保留校正接口，暂不依赖未烧录的下位机新功能。

视觉摆正后沿用原交接流程：锁存当前 yaw，用相对 yaw 变化维持短距离直行，
完成原有 17/20 cm 距离后按原流程交接、停车和转弯。没有新增校准停稳 500 ms、
HEADING_REFERENCE 请求或校准 ACK 等待，不要求旧固件具备 HELLO capability bit4。
原有转弯前停车和视觉/里程计异常保护保留，速度与距离参数保持原值。

仅在上位机调整 yaw 偏移无法校正旧 MCU 内部的离散动作方向档位，不能当成
下位机 IMU 零偏已经重估。配套固件烧录后，可显式启用保留的校正接口。

部署前提交为 df54d130b26efb4c38598b62b7de9a15c4fb2e5a，已推送。
本次基于板端最新提交，保留 b2aa082 及之前的手动地面标定改动。

Linux 144 项相关 Python 测试通过，Windows 可移植测试 143 项通过。
四种普通/巡检、格网/短边路径验证不发校准请求、不增加校准停车，且仍按原
短距离锁存当前航向；显式启用接口的停稳/ACK 验证仍通过。
现有已安装 uart_vel 对 capability=0 的 PTY 假 MCU 测试通过。

只更新 Python、配置、测试与说明；C++ UART 程序没有变更，无需重新构建。
没有启动实车导航、打开真实相机/串口或发送真实运动指令。
下位机仓库和固件未改动，语音配置与音轨待办保持原状态。

修改前的 7 个文件备份在
`/home/orangepi/deployment-backups/heading-legacy-20261010-df54d13`。
验证日志在 `/tmp/codex-heading-voice-at8u9mk2/legacy_*.log` 与 JSON 文件中。
