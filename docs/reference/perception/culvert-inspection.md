# 涵洞两侧识别与局域网调试

## 当前实现与启用边界

显式 `--culvert-inspect` 替换现有 5 秒 PauseTask；未传该选项时仍使用原涵洞停车流程。
模型复用测试1：ArcFace 本机 20004、刀具本机 20005；测试2仍是 AprilTag 标定。
不修改模型、导航速度、静态地图、UART 协议或系统 overlay。按现场调试要求，全局取消
导航 200 ms 时效上限；道路丢失、里程失效、车辆移动及有限动作失败仍停车。

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

网页为 `http://<板端IP>:8081/`。每轮按原阈值、连续新帧和 45 秒期限检查当前
相机方向，结果展示 5 秒后重新检测同一方向；页面明确显示 PWM 禁用。ROI 和阈值
仍保存到独立 JSON，原导航入口的硬件检查保持有效。`--camera` 仅覆盖本次调试来源，
不写入正式导航相机配置。Ctrl+C 或 SIGTERM 停止网页和相机。

`config/culvert_inspection.json` 保存 camera、recognition、services、servo、web、obstacle。
当前板端侧视设备为 `/dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0`
（现场确认的 60 fps 摄像头），导航使用另一台 180 fps 摄像头，不能占用同一设备。
现场已确认 PWM14_M0 的两端正常且安全，当前 hardware_verified=true；其他机器
应重新核实相机、live DT 芯片映射和权限。0°=500000 ns，180°=2500000 ns。真实路由检查参见 [舵机硬件](../hw/servo.md)。
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
  --drive --uart-bin build-turn/uart/uart_vel --turn-at-junction right \
  --culvert-stop --culvert-estimated-camera --culvert-inspect --inspection-web \
  --inspection-config config/culvert_inspection.json --record-video
```

估算相机模式不表示中央停车精度已验收；已有标定模式沿用原用法。
侧视启动失败、PWM 回执失败或网页端口不可用时不开始导航。运行期间识别超时可继续，
但 PWM 失败、里程失效、车辆移动和导航画面不安全仍故障停车。
退出关闭 PWM、不归位，停止网页与侧视采集线程。

### 导航时效设置

导航的分割耗时、分割帧龄、YOLO 后及发速度前均不再使用 0.2 秒上限。
这对整次导航默认生效，无需任何额外开关。旧配置 max_frame_age_s 仅为读取兼容
保留，不重新启用控制上限。采集时间必须有效，重复帧及未来时间戳仍拒绝；
里程时间匹配、道路内容、停车回执和车辆移动检查继续有效。

识别结束后清空积压分割结果和中心线平滑状态，完成识别的这一帧保持停车，
随后用不同的新导航帧重新确认道路，再恢复行驶。慢推理可能使运动使用较旧画面，
取消时效上限并不说明导航与识别混合负载已通过实车验收。
当前板端命令：

```sh
cd /home/orangepi/pi_upper
PYTHONPATH=navigation:vision:. python3 -u -B -m road_follow \
  --drive --uart-bin build-turn/uart/uart_vel --turn-at-junction right \
  --culvert-stop --culvert-estimated-camera --culvert-inspect --inspection-web \
  --inspection-config config/culvert_inspection.json --record-video
```

## 当前板端识别服务准备

2026-10-09 的 10.211.30.33 使用两种已有本机服务，均只监听 127.0.0.1。
ArcFace 继续使用原有 `.venv`、buffalo_sc 模型和 `face_db.npz`；不重新注册或改写人脸库。
该库身份名为 "1"～"10"，涵洞执行器将这些名字转换为任务编号 suspect_01～suspect_10，
同时兼容已有 suspect_01～suspect_10。未知身份、低分和多人脸仍不接受，也不回退刀具。

刀具服务使用独立环境 `/home/orangepi/.venvs/pi-upper-culvert-services`，
继承板端已安装的 RKNN Lite2，并按 `vision/knife/requirements.txt` 补齐服务依赖。
不升级系统 Python 或 RKNN。缺少 ensurepip 时可用已有 pip 的 --python 选项：

```sh
python3 -m venv --without-pip --system-site-packages /home/orangepi/.venvs/pi-upper-culvert-services
env -u http_proxy -u https_proxy -u all_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
  PIP_CONFIG_FILE=/dev/null python3 -m pip \
  --python /home/orangepi/.venvs/pi-upper-culvert-services/bin/python \
  install -r vision/knife/requirements.txt
```

从项目根目录分别在两个终端启动；已运行时不要重复启动：

```sh
vision/arcface-lite/.venv/bin/python -u -B vision/arcface-lite/server.py --host 127.0.0.1 --port 20004
/home/orangepi/.venvs/pi-upper-culvert-services/bin/python -u -B -m vision.knife.service --config config/knife.json --host 127.0.0.1 --port 20005
```

检查两个 `/health` 后，再运行已授权的正式导航命令。服务准备本身不连接串口或驱动车辆。
当前网页由导航的 --inspection-web 启动，地址为 http://10.211.30.33:8081/；模型接口仅供本机访问。
本次后台启动不安装开机服务，重启后须重新启动模型服务；PWM 临时权限恢复见舵机文档。
参考图自匹配及离线测试通过不代表实车识别准确率或导航/识别混合负载已验收。

## 网页与外层 ROI

启用 --inspection-web 后，整次导航持续提供侧视画面，默认
`http://<板端IP>:8081/`（当前为 `http://10.211.30.33:8081/`）；模型服务仍只在本机使用。
网页用中文状态卡片、两侧结果卡片及参数表格显示原图、当前侧、阶段、候选、相似度、
刀具分差、连续计数、剩余时间和失败原因。使用参数保留名称、单位及 ROI 百分比，
页面不展示原始 JSON；编辑输入框与当前已应用参数分别展示。

在原图拖框排除底部板卡；归一化坐标 [x1,y1,x2,y2] 同时裁剪两种模型输入。
当前板端已保存的 ROI 为 [0,0,1,0.75]，保留原图上方 75%，排除底部 25%；
其他设备可按实际遮挡再次调整。此外层 ROI 不代替刀具服务内部的
自动单刀取景或前景提取，刀具请求仍传 roi_selected=false；显示框会加回原图偏移。

保存位置为 config/culvert_inspection.json 的 recognition.roi。另一台机器拉取最新
main 后，启动或重启识别程序即可从同一路径加载；归一化 ROI 按本机实际帧尺寸裁剪。
已运行的进程不会因外部 Git 更新而自动重读 JSON。局域网浏览器访问本板网页时共用
本板正在运行的配置；网页后续保存仍需另行提交和推送，才能通过 Git 分享到其他机器。

点击“应用并保存识别参数”原子保存识别参数；下次启动保留。另设障碍参数保存按钮。
不开放相机、PWM、服务地址或运动控制。人脸阈值最低 0.45。
修改后清空当前侧累计并丢弃旧配置请求，当前侧 deadline 不延长，新超时从下一侧生效。
转向、请求失败、低分或身份变化也重计数。多个页面保存冲突会返回 409，需刷新。
浏览器断开或慢客户端不影响任务；不接管导航相机。

## 障碍归属与倒车调试

障碍框底部中心投影到地面，并使用采集时刻的 ODOM 沿边进度计算剩余距离。
仅目标位于当前边且距离不超过“剩余距离减终点保留余量”时，才参与当前边的
连续确认。框底接地点截断、投影无效、横向超界或没有对应里程时不参与封边。
例如 5_4→4_4 接近终点时，3_4—4_4 的障碍可以标为前方看见，不封 5_4—4_4。
跨边定位只沿唯一的直行延续边；分支或转弯不猜测。普通拓扑模式未启用涵洞
ODOM 历史时使用当前进度估计；现场应使用上面的涵洞命令获得采集时刻匹配。

网页“障碍判断调参”保存到 config/culvert_inspection.json 的 obstacle 区：

| 参数 | 初始值 | 作用 |
| --- | --- | --- |
| 路段终点保留余量 | 0.05 米 | 终点附近暂停当前边累计 |
| 投影距离修正 | 0 米 | 现场修正接地点估算偏差，可为负 |
| 最大判定距离 | 0.80 米 | 远处目标暂不参与归属判断 |
| 相对道路中心的横向容差 | 0.18 米 | 排除其他道路上的目标 |
| 障碍检测置信度下限 | 0.45 | 确认候选最低分 |
| 框底部最低位置 | 原图高度的 35% | 保留原近距离过滤 |
| 连续确认帧数 | 3 帧 | 通过条件后连续确认 |

障碍配置独立计数版本；保存后从后续帧生效，清空未确认障碍累计，不影响
当前侧的人脸/刀具累计。切换有向边或重新计里程也清空障碍累计。
已封边仍保留本次任务记录，需重启任务才清空。距离依赖地面标定和接地点，
上述调试值需现场逐步调整，不能视为已验收的障碍定位精度。

原倒车已经使用导航视觉 near_x_m 计算纠偏角速度，并通过负向 CMD_VEL 倒车；
没有修改其 0.08 m/s、0.28 米预瞄和 0.4 rad/s 上限。新任务日志增加
obstacle_observation 与 obstacle_backup_control，可核对框归属、剩余距离、
中心偏差和实际给定 v/w；旧日志未逐帧记录倒车输出，不能还原当时的纠偏效果。

## 地图与语音

网页拓扑图显示完整节点/道路、车辆位置、障碍、涵洞和标签。
每次启动任务创建空动态覆盖层，不读取上轮动态标记；原任务历史文件仍保留，
不修改静态拓扑 YAML。看见障碍为橙色，确认障碍为红色；只有原重规划流程封边。
地图位置来自框地面接点、拓扑边长和里程，属于估计位置。

任务 .culverts.json/.culverts.svg 保存两侧参数、身份、分数、确认帧数、方向端点、
未确认原因、障碍和标签记录。done 为两侧均确认（绿色），partial 为检查结束但
部分未确认（紫色）。两种状态本次任务均去重；安全故障为 failed（红色）。
高频障碍位置更新最多每秒保存一次，新发现/确认立即保存，退出补存最终快照。

当前 RFID_EVENT 协议只提供 1～12 的标签号和 generation，没有原始 UID 字节。
地图标注真实收到的标签号，接收时的拓扑里程位置为估计；不伪造 UID，也不让
这些地图事件接管视觉路口转向。原始 UID 显示需后续明确修改下位机协议。

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
现场 ROI [0,0,1,0.75]、已确认相机及 hardware_verified=true；测试样本独立。
既有涵洞配置测试读取本地模型指纹，不推理。实际实体识别准确率、负样本误判率、
PWM 机械到位和混合 NPU 负载尚须授权联调；刀具服务用核0/1/2可能与导航争用资源。
本轮没有实施 NPU 独占交接或修改模型核配置，后续仍需混合负载实测。
离线通过不代表实车验收。

## 2026-10-09 最近涵洞故障核查

最后一次任务的第二涵洞位于 2_2→2_1。B 侧连续三帧确认嫌疑人6号，
相似度约 0.7702；翻转后 A 侧识别中出现 vision_unsafe，沿边里程没有变化，
日志没有 PWM 写入失败或车辆移动故障。该任务已经启用旧临时时效豁免，故障
不是 200 ms 限制。保存导航视频约 144.6 秒时手指遮挡镜头；离线道路模型回放
在 144.6/144.8/145.0 秒连续输出零道路像素和 stop_road，与故障时刻吻合。
保留导航道路内容检查，移开遮挡后再运行；离线回放不代表新版本实车验收。
