# UID 与识别结果播报

语音复用导航已有的 `uart_vel` 会话，不另开串口。巡逻点编号仍为 1～12；
`suspect_01`～`suspect_10` 对应音频 13～22；`knife_01`～`knife_10` 对应音频 23～32。
涵洞识别只提交已经连续确认且身份合法的结果，未确认不播报。同一次任务按
`session_id + edge_id + A/B 侧` 去重；另一涵洞或另一侧看见相同身份仍可播报。

## 默认播报与配置兼容

UID、嫌疑人和刀具播报都不要求填写音轨时长。确认合法身份后直接提交音频编号，
UID 与识别结果进入同一有界 FIFO。桥收到上一条请求的 ACK 后即可发送下一条，
不等待录音播放结束，也不处理连续命令可能造成的播放打断。

保留 `config/speech.json` 的原有格式和 `--speech-config` 参数用于兼容，
`durations_ms` 全部为 `null` 或部分缺项都不会阻止识别播报。即使填写完整时长表，
Python 也不传入播放等待参数；桥兼容接受原 `--speech-hold-ms 编号:毫秒` 参数，
但不按这些时长等待。容量内已接受项不覆盖；队满明确拒绝并记录。

## 桥接口与结果含义

识别层向现有桥 stdin 写入 `speech <event_id> <audio_id>`。桥 stdout 返回：

- `SPEECH_QUEUED <event_id> <audio_id>`：队列接受。
- `SPEECH_REJECT <event_id> <audio_id> <reason>`：没有接受该事件。
- `SPEECH_SENT <event_id> <audio_id>`：已经提交 UART 帧。
- `SPEECH_RESULT <event_id> <audio_id> <result>`：本次请求的 ACK、超时或断线结果。

`ACK_OK` 只表示下位机接受并完成本次 UART4 发送，**不表示声音已经播完或实际发声**。
模块没有播放 BUSY 或播放完成反馈，桥不保留音轨占用窗口。
ACK 超时或链路丢失的已发送请求记录为不确定，不自动重播；语音通道暂停至重新建链。
等待中的队列项明确返回拒绝原因；超时后新请求返回 `CHANNEL_BLOCKED`，
不在后台积累并于重连后突然播放。
语音请求使用独立等待状态，不能占用运动管理请求；STOP 与运动继续优先处理。

## 构建与验收边界

部署后必须明确指定本次新构建的 `uart_vel`，路径以实际部署目录为准，避免默认路径
加载旧二进制。当前验证程序位于独立的 `/tmp/codex-heading-voice-*/upper/build-linux`，
没有覆盖在用程序。已通过 Linux 伪终端内的实际桥进程联调；此类离线测试不能证明
已烧录对应 STM32 固件、模块音轨正确或实际播放无截断。
