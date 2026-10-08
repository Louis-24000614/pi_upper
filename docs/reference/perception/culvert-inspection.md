# 涵洞两侧识别与局域网调试

## 当前实现与启用边界

显式 `--culvert-inspect` 替换现有 5 秒 PauseTask；未传该选项时仍使用原涵洞停车流程。
模型复用测试1：ArcFace 本机 20004、刀具本机 20005；测试2仍是 AprilTag 标定。
不修改模型、导航速度、停车保护、静态地图、UART 协议或系统 overlay。

进入涵洞后，沿用 STOP 完成确认、连续 2 秒停稳及中央位置检查。第一侧识别后转到
另一绝对端点，等待 3 秒、清空旧帧，再识别第二侧；每个涵洞翻转一次，完成不归位。
下一涵洞从当前端点开始，反向翻转。启动时先设置 JSON 中的初始端点并等待稳定，
此初始化不属于某个涵洞内的翻转。

相同类型和身份连续 3 张不同采集帧达到相似度 0.5 即确认；两种模型均采用 0.5，
刀具分差只显示。每侧姿态稳定后最多识别 45 秒，未确认后继续另一侧或恢复导航。
相似度不是成功概率。刀具模型没有经过验证的无刀/未知类拒识能力，不能把高分候选
或超时结果解释为“安全”“无刀”。0.5 为用户选择的调试值，需正负样本验收。

同一外层裁剪帧先请求人脸；明确无脸才请求刀具。未知/低分/多人脸以及人脸请求失败
不当作无脸，也不采纳该帧刀具候选。刀具还须有有效内部自动取景结果。HTTP 与 JPEG
编码在后台执行，每约 1.2 秒取最新帧，不积压任务、不阻塞导航主循环。

## 独立配置与硬件检查

未接舵机时可先运行独立单侧调试，使用明确指定的 USB 相机，不连接 PWM/UART，
不启动导航，也不把调试结果标记为真实两侧任务或写入任务地图：

```sh
PYTHONPATH=navigation:vision:. /home/orangepi/miniconda3/envs/pi_upper/bin/python -u -B \
  -m road_follow.inspection_debug \
  --camera /dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0
```

网页为 `http://192.168.92.163:8081/`。每轮按原阈值、连续新帧和 45 秒期限检查当前
相机方向，结果展示 5 秒后重新检测同一方向；页面明确显示 PWM 禁用。ROI 和阈值
仍保存到独立 JSON，原导航入口的硬件检查保持有效。`--camera` 仅覆盖本次调试来源，
不写入正式导航相机配置。Ctrl+C 或 SIGTERM 停止网页和相机。

`config/culvert_inspection.json` 保存 camera、recognition、services、servo、web。
默认 camera.device 为 null，必须填写实际侧视设备，不能与导航相机相同。
servo.hardware_verified 默认 false；确认 PWM14_M0 接线、live DT 芯片映射、权限及
0°/180° 安全端点后才能设置 true。真实路由检查参见 [舵机硬件](../hw/servo.md)。
不猜测 pwmchip 编号，不自动安装依赖、修改系统或重启。

构建常驻 PWM CLI 与离线 C++ 测试会写入指定 build 目录；部署及构建权限确认后执行：

```sh
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build --target servo_cli servo_app_test -j
ctest --test-dir build -R servo_app --output-on-failure
```

启动前要获得相机、串口、舵机和实车动作授权。以下命令会运动、初始化 PWM 并打开
侧视摄像头，不是离线验证命令；保留当前现场原有导航参数和 UART 二进制路径：

```sh
PYTHONPATH=navigation:vision:. python3 -u -B -m road_follow \
  --drive --uart-bin build/uart/uart_vel --turn-at-junction left \
  --culvert-stop --culvert-estimated-camera --culvert-inspect --inspection-web \
  --inspection-config config/culvert_inspection.json --record-video
```

估算相机模式不表示中央停车精度已验收；已有标定模式沿用原用法。
侧视启动失败、PWM 回执失败或网页端口不可用时不开始导航。运行期间识别超时可继续，
但 PWM 失败、里程失效、车辆移动和导航画面不安全仍故障停车。
退出关闭 PWM、不归位，停止网页与侧视采集线程。

## 网页与外层 ROI

启用 --inspection-web 后，整次导航持续提供侧视画面，默认
`http://192.168.92.163:8081/`；模型服务仍只在本机使用。
网页显示原图、当前侧、阶段、候选、相似度、刀具分差、连续计数、剩余时间及任务地图结果。

在原图拖框排除底部板卡；归一化坐标 [x1,y1,x2,y2] 同时裁剪两种模型输入。
已保存并提交的 ROI 为 [0,0,1,0.8]，保留原图上方 80%，排除底部 20%；
其他设备可按实际遮挡再次调整。此外层 ROI 不代替刀具服务内部的
自动单刀取景或前景提取，刀具请求仍传 roi_selected=false；显示框会加回原图偏移。

保存位置为 config/culvert_inspection.json 的 recognition.roi。另一台机器拉取最新
main 后，启动或重启识别程序即可从同一路径加载；归一化 ROI 按本机实际帧尺寸裁剪。
已运行的进程不会因外部 Git 更新而自动重读 JSON。局域网浏览器访问本板网页时共用
本板正在运行的配置；网页后续保存仍需另行提交和推送，才能通过 Git 分享到其他机器。

点击“应用并保存 JSON”原子保存识别参数；下次启动保留。网页仅可改两种分数、连续
帧数、每侧超时和 ROI，不开放相机、PWM、服务地址或运动控制。人脸阈值最低 0.45。
修改后清空当前侧累计并丢弃旧配置请求，当前侧 deadline 不延长，新超时从下一侧生效。
转向、请求失败、低分或身份变化也重计数。多个页面保存冲突会返回 409，需刷新。
浏览器断开或慢客户端不影响任务；不接管导航相机。

## 地图与语音

原任务 .culverts.json/.culverts.svg 新增两侧参数、身份、分数、确认帧数、方向端点和
未确认原因。done 为两侧均确认（绿色），partial 为检查结束但部分未确认（紫色）。
两种状态本次任务均去重，partial 不等于成功；静态拓扑与道路覆盖状态不改变。
安全故障仍是 failed（红色），不标记检查完成。

本轮只发软件识别结果事件，speech_enabled=false，不发送语音命令。现有
Session::RequestSpeech / SPEAK_AUDIO (0x14) 仅支持真实音频 1～12，尚无身份编号表；
后续取得映射后复用导航串口会话，不能同时启动独占串口的 uart_speak。
ACK_OK 仅接受请求，不表示播放完成。

## 离线验证与待验收

```sh
PYTHONPATH=navigation:vision:. python3 -B -m unittest discover \
  -s navigation/road_follow/tests -p 'test_inspection*.py'
PYTHONPATH=navigation:vision:. python3 -B -m unittest discover \
  -s navigation/road_follow/tests -p 'test_culvert*.py'
```

新测试只使用合成相机、假 HTTP、假 PWM 与系统临时目录；网页测试监听本机随机端口。
识别测试读取 tests/fixtures/culvert_inspection.json 的固定默认样本，不读取或覆盖现场
已经保存的运行配置；在网页修改 ROI 或阈值后仍可执行离线测试。提交的正式配置保留
已保存的 ROI [0,0,1,0.8]、未指定相机及 hardware_verified=false。
既有涵洞配置测试读取本地模型指纹，不推理。实际实体识别准确率、负样本误判率、
PWM 机械到位和混合 NPU 负载尚须授权联调；刀具服务用核0/1/2可能与导航争用资源，
不得通过降低既有时效保护解决。离线通过不代表实车验收。
