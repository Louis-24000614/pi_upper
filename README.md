# 侦察机器人上位机（pi_upper）

运行在 Orange Pi 5 Plus（RK3588）上的机器人上位机。当前主链路为：前视相机道路分割、BEV 中心线提取、纯视觉拓扑巡航，以及通过 UART 驱动 STM32 下位机完成速度控制和有限运动动作。RFID 代码仅保留为独立硬件测试，不参与当前拓扑巡航到点。

本仓库不使用 ROS。导航主体目前是 Python，UART、舵机等硬件接口是 C++，调试界面使用 PySide6。

## 当前能力

- RKNN 道路分割与 BEV 鸟瞰转换
- Pure Pursuit 视觉循迹，输出线速度和角速度
- 白底白色矮沿场地的道路宽度先验与中心线时间平滑
- 出发区定距前进到 `0_J`、停车、按拓扑路线 IMU 原地转向，再恢复视觉循迹
- 路口侧向开口/道路端头识别，定距前进后原地转弯
- 视觉路口到点、拓扑节点推进和地图决定转向
- 道路丢失、路口视觉未确认和有限动作失败时停车
- 原始相机视频录制和逐帧控制日志
- 障碍物检测与倒车重规划原型
- Qt/PySide6 调试界面、舵机及刀具/人脸识别模块

障碍重规划、完整任务编排和 GUI 集成仍在继续联调；不要把原型功能视为已经完成实车验收。

## 硬件与运行环境

- 主控：Orange Pi 5 Plus（RK3588）
- 系统：Ubuntu 22.04
- 前视相机：UVC，当前配置为 `/dev/video0`、1280×720、约 85° 水平视场
- 下位机：STM32H743VIT6，ICM42688 IMU、编码器和 RFID 由下位机管理
- UART：当前板端设备 `/dev/ttyS6`，921600 8N1，3.3 V TTL
- 推理：RKNN Runtime；导航默认模型 `models/road_yolo11n_seg.rknn`
- 主要软件：Python 3、OpenCV、NumPy、PyYAML、RKNNLite；C++17、CMake

摄像头、串口设备和导航参数以 [`config/nav_camera.yaml`](config/nav_camera.yaml) 为准。STM32 引脚和线协议以下位机工程及其 `UART_PROTOCOL.md` 为准，避免根据本文硬编码接线。

## 目录结构

| 路径 | 用途 |
| --- | --- |
| `navigation/road_follow/` | 实车道路分割、视觉循迹和纯视觉拓扑路口状态机 |
| `navigation/topo_proto/` | 拓扑加载、寻路和封边原型 |
| `vision/ipm_proto/` | IPM、中心线、路口几何和时间平滑 |
| `vision/obstacle/` | 障碍物检测与硬堵塞判定 |
| `uart/` | UART 协议、会话以及 `uart_vel`/`uart_turn` 工具 |
| `agent/` | 路线 Agent 和运行状态 |
| `mission/` | 任务协调模块 |
| `servo/` | 舵机控制与命令行工具 |
| `gui/` | PySide6 调试界面 |
| `config/` | 相机、导航拓扑、障碍物和舵机配置 |
| `docs/` | 架构、协议和专题设计文档 |
| `data/road/` | 实车录像和日志（运行时生成） |

## 构建

在仓库根目录执行：

```bash
cd ~/pi_upper

cmake -S . -B build-turn -DBUILD_TESTS=ON
cmake --build build-turn -j
ctest --test-dir build-turn --output-on-failure
```

导航命令使用的 UART 桥为：

```text
build-turn/uart/uart_vel
```

## 运行前检查

首次联调或修改运动参数后，先架空驱动轮并确认周围可以立即断电。

检查相机：

```bash
v4l2-ctl --device /dev/video0 --all
```

只检查 UART 接收，不 ARM、不发送速度：

```bash
python3 uart/tools/uart_monitor.py --device /dev/ttyS6 --baud 921600
```

日志中应看到 `HELLO_RX`、`state=READY`，并持续收到 `ODOM`、`IMU`、`STATUS` 等消息。不要同时运行多个占用 `/dev/ttyS6` 的进程。

只运行相机和视觉推理、不驱动车辆：

```bash
mkdir -p data/road

PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --frames 100 \
  --preview data/road/preview.jpg
```

只有显式添加 `--drive` 才会打开 UART 并驱动车辆。

## 视觉直线循迹

只使用道路分割和视觉中心线循迹，不启用路口或 RFID 转弯：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive \
  --uart-bin build-turn/uart/uart_vel
```

同时录制原始相机画面和终端日志：

```bash
mkdir -p data/road

PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive \
  --uart-bin build-turn/uart/uart_vel \
  --record-video \
  2>&1 | tee data/road/drive_test.log
```

不指定录像文件时，视频自动保存为 `data/road/drive_时间.avi`。也可以显式指定：

```bash
--record-video data/road/my_test.avi
```

按 `Ctrl+C` 会请求停车并退出。实车测试不要依赖终端快捷键作为唯一急停手段。

## 出发区与路口导航

当前实车联调命令：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive \
  --turn-at-junction right \
  --uart-bin build-turn/uart/uart_vel \
  --record-video \
  2>&1 | tee data/road/drive_junction.log
```

启用 `--turn-at-junction` 后，不再等价于单独视觉直走。程序会加载拓扑路线，并首先执行出发区动作：

```text
从 0_0 前进 15 cm 到 0_J
→ 停车并等待 2 s
→ Agent 推进 0_0→0_J，并读取下一条边
→ 默认按 0_J→1_2 执行 IMU 原地右转
→ 清空转弯期间的中心线历史
→ 原地连续 3 帧确认新道路
→ 以不高于 0.06 m/s 视觉循迹 10 帧
→ 恢复正常视觉循迹
```

后续道路仍使用与直线模式相同的分割、BEV 和视觉控制器。`patrol_slot` 保留原来的巡检点视觉接近状态机，普通 `junction` 保留原来的路口状态机；唯一取消的是拓扑任务中的 UID 输入。只有有限动作状态机会在需要时接管速度输出。

路口交接原则：左右任一侧的矮沿端头先锁存「这里有路口」；之后继续视觉循迹。丁字口或道路端头可由 BEV 正前方 34–48 cm 检测带连续消失触发交接；`2_2`、`3_2` 这类正前方仍有道路的十字口，则在支路已锁存后由当前边 ODOM 到达“边长减 20 cm”处触发。两种方式最终都只让下位机用 IMU 锁航向固定前进一次 20 cm。动作完成后停车，Agent 根据地图中的入边和下一条边决定直行、左转、右转或倒车；转弯后重新进入视觉寻路。固定动作期间不会同时发送 `CMD_VEL`，交接未成功而 ODOM 已到边末端时会强制安全停车。

命令里的 `right` 继续表示普通路口观察右侧开口；它不强制后续节点全部右转，最终动作始终来自拓扑。巡检点仍按原逻辑保存实际看到的左侧、右侧或两侧。

RFID 只保留为独立硬件转向测试，不属于上面的拓扑任务：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive \
  --turn-at-rfid right \
  --uart-bin build-turn/uart/uart_vel
```

`--turn-at-junction` 是当前纯视觉拓扑任务；`--turn-at-rfid` 和 `--backup-on-obstacle` 是独立测试模式，部分组合会被命令行拒绝；以 `python3 -m road_follow --help` 为准。

## 关键导航配置

主要参数位于 [`config/nav_camera.yaml`](config/nav_camera.yaml)：

| 配置段 | 作用 |
| --- | --- |
| `camera` / `bev` | 相机外参、内参与鸟瞰范围 |
| `capture` | 摄像头设备和分辨率 |
| `uart` | 串口设备与波特率 |
| `follow` | 速度、预瞄距离、转向增益和角速度限幅 |
| `entrance` | 出发区前进到 `0_J`、停车、按拓扑转向和视觉重新捕获 |
| `road_prior` | 道路宽度范围和中心线提取先验 |
| `junction_turn` | 普通路口原有的锁存、20 cm 交接和原地转向参数 |
| `rfid_turn` | 巡检点原有的侧边/检测带/20 cm 参数；也供独立 RFID 硬件测试复用 |
| `temporal` | 中心线 EMA 平滑参数 |

当前循迹坐标约定：地面 `X` 向右、`Y` 向前；正角速度表示左转。`x_bias_m` 是横向标定量，`steering_gain` 是视觉角速度增益，最终仍受 `max_abs_omega` 限制。每次修改参数后应保留录像与日志，不能只根据肉眼印象连续加大增益。

拓扑节点和边长位于 [`config/nav_topology.yaml`](config/nav_topology.yaml)，相关约定见[拓扑定位、纯视觉到点与可选 RFID 测试](docs/reference/navigation/topology-rfid-navigation.md)。

## 日志排查

运行日志采用中文关键事件和限频状态摘要：状态发生变化时立即输出；状态不变时每 2 秒输出一次，避免停车、定距动作或 RFID 搜索期间逐帧刷屏。常用内容：

| 中文内容 | 含义 |
| --- | --- |
| `[状态] 视觉循迹` / `近距离低速循迹` | 当前控制方式 |
| `速度` / `角速度` | 当前准备发送的线速度和角速度 |
| `道路` / `中心线` / `可见距离` | BEV 道路像素、中心线点数和距离范围 |
| `路口` / `开口` / `检测带` | 稳定路口类型、可通方向和正前方 mask 占比 |
| `支路=已锁存` | 已确认左/右支路，正等待道路端头交接 |
| `[动作]` | 定距前进、停车或原地转弯的完成/失败结果 |
| `[RFID]` | 只会在显式运行 `--turn-at-rfid` 独立测试时出现 |
| `停车：……` | 直接给出中文停车原因 |

分析偏航时，应同时比较录像、状态摘要中的角速度和下位机实际 ODOM/IMU。需要逐帧几何量时应结合录像离线回放；默认实车日志只保留运行决策所需的关键量。

## 测试

道路循迹状态机测试：

```bash
PYTHONPATH=.:navigation:vision python3 -m unittest discover \
  -s navigation/road_follow/tests -p 'test_*.py'
```

IPM、路口和拓扑模块可分别执行各目录下的 `tests/`。C++ 测试使用：

```bash
ctest --test-dir build-turn --output-on-failure
```

## UART 通信

线协议为二进制帧 `55 AA | TYPE | LENGTH | PAYLOAD | CRC8`，协议标识为 1。上位机完成 `HELLO` 后请求 ARM：

- 连续视觉循迹使用 `CMD_VEL(v, omega)`。
- 定距前进、原地 90° 转弯、停车使用 `MOTION_ACTION`，并等待 `MOTION_RESULT`。
- `CMD_VEL` 与有限动作互斥。
- 退出时上位机主动 STOP/DISARM；通信故障和动作失败必须停车。

上位机实现约定见 [`docs/api/uart.md`](docs/api/uart.md)，线协议以下位机仓库的 `UART_PROTOCOL.md` 为唯一权威。

## GUI 与其他模块

在 Orange Pi 的 X11 桌面启动调试界面：

```bash
bash gui/run.sh
```

刀具识别参见 [`vision/knife/README.md`](vision/knife/README.md) 和 [`docs/reference/perception/knife.md`](docs/reference/perception/knife.md)。舵机参见 [`servo/README.md`](servo/README.md) 与 [`docs/reference/hw/servo.md`](docs/reference/hw/servo.md)。

## 文档入口

- [`docs/doc_layout.md`](docs/doc_layout.md)：文档地图
- [`docs/nav.md`](docs/nav.md)：导航与障碍重规划设计
- [`docs/architecture/overview.md`](docs/architecture/overview.md)：总体架构
- [`docs/api/uart.md`](docs/api/uart.md)：UART 上位机接口约定
- [`docs/reference/navigation/upper-planner.md`](docs/reference/navigation/upper-planner.md)：上层路线规划
- [`docs/reference/navigation/topology-rfid-navigation.md`](docs/reference/navigation/topology-rfid-navigation.md)：纯视觉拓扑到点、地图转向与独立 RFID 测试
