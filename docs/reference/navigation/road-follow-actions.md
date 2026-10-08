# 道路跟随有限动作说明

本文说明 `feat/road-follow-uart-actions` 引入的两层能力：

- `uart_vel` 把上层文本命令转换为下位机 `MOTION_ACTION`，并把动作结果、RFID 和里程计事件输出给上层。
- `navigation/road_follow` 在视觉循线之上增加路口转向、RFID 到点转向和遇障倒车三个局部状态机。

这些功能用于单项联调，不是全局任务规划器。三个局部模式需要分别启用，不能同时运行。

## 代码位置

| 文件 | 职责 |
| --- | --- |
| `uart/app/vel_main.cpp` | 速度桥、有限动作命令、动作结果、RFID 与里程计输出 |
| `uart/app/turn_main.cpp` | 独立的 90° 左右转命令行测试工具 |
| `uart/tools/uart_monitor.py` | 串口帧监控和诊断 |
| `vision/ipm_proto/junction.py` | 从 BEV 道路掩码识别直道、T 字、十字、拐角和道路末端 |
| `navigation/road_follow/junction_turn.py` | 路口定距前进、转弯和视觉重捕获状态机 |
| `navigation/road_follow/rfid_turn.py` | RFID 停车、转弯和视觉重捕获状态机 |
| `navigation/road_follow/backup.py` | 遇障摆正并按沿边里程倒车 |
| `navigation/road_follow/__main__.py` | 摄像头、分割、UART 和三个状态机的集成入口 |
| `config/nav_camera.yaml` | 相机、BEV、循线、路口和 RFID 参数 |

## 构建

在仓库根目录执行：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target uart_vel uart_turn -j
```

默认使用 `/dev/ttyS6`、921600 baud；可在 `config/nav_camera.yaml` 的 `uart` 段修改。

## 运行模式

只观察视觉诊断，不驱动车辆：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow
```

普通视觉循线：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow --drive
```

发现左侧路口后定距前进并左转：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive --turn-at-junction left
```

RFID 到点后右转：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive --turn-at-rfid right
```

连续检测到障碍后倒回上一个路口：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive --backup-on-obstacle
```

若构建目录不是默认的 `build/`，用 `--uart-bin` 指定 `uart_vel`：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive --uart-bin /path/to/uart_vel --turn-at-junction left
```

`--turn-at-junction`、`--turn-at-rfid` 和 `--backup-on-obstacle` 互斥。遇障倒车必须配合 `--drive`，因为它需要下发有限动作并读取里程计。

## 上下层交互

Python 进程持续向 `uart_vel` 的标准输入写入命令。普通循线使用：

```text
0.100 -0.120
```

两个数字分别是线速度 `m/s` 和角速度 `rad/s`。有限动作使用：

```text
forward 320 50
backward 420 100
turn left
turn right
stop
```

`forward`、`backward` 的两个参数分别是距离 `mm` 和速度 `mm/s`。有限动作执行期间，UART 会话拒绝新的速度指令，避免速度环覆盖动作环。

`uart_vel` 通过标准输出通知 Python：

```text
FORWARD_DONE
FORWARD_FAIL
BACKWARD_DONE
BACKWARD_FAIL
TURN_DONE
TURN_FAIL
STOP_DONE
STOP_FAIL
RFID_EVENT <card_number> <generation>
RFID_REMOVED <generation>
RFID_INVALID <generation>
ODOM <x_m> <y_m> <yaw_rad> 1
```

任一有限动作返回 `*_FAIL` 或超过本地超时后，上层进入故障停车状态，不自动继续行驶。

## 路口转向

状态流为：

```text
follow -> approach -> heading_hold -> arrived -> turn -> reacquire -> follow
                    \------------- failure ---------------> fault
```

1. BEV 分类器判断左右支路和前方道路是否存在。
2. 远处看见目标支路时先锁存方向，仍由视觉循线靠近。
3. 路口距离进入交接窗口并稳定若干帧后，先按近处中心线摆正。
4. 摆正后进入 `heading_hold`：锁住当时的车体直行，下发 `CMD_VEL(v, 0)`，不再发 `forward`，也不再更新视觉角度。
5. 沿边里程再走完配置距离（默认 0.20 m）后停车，再按规划下发 `turn left/right`。转弯仍交给 IMU。
6. 收到 `TURN_DONE` 后等待新方向中心线连续稳定，再恢复视觉速度环。

侧向支路被墙遮挡时，可以用“道路末端进入指定距离窗口”作为备用交接信号。计算结果必须落在 `min_forward_mm` 到 `max_forward_mm` 之间，否则停车，不下发盲行动作。

关键配置位于 `junction_turn`：

- `handoff_min_distance_m` / `handoff_max_distance_m`：允许交接的路口距离窗口。
- `camera_ahead_of_turn_center_m`：相机光心相对两轮旋转中心的纵向偏移。
- `stop_before_center_m`：希望停在旋转中心前方的余量。
- `forward_speed_mmps`：定距前进速度。
- `reacquire_frames`：转弯后恢复循线所需的连续有效帧数。
- `road_end_*`：道路末端备用交接条件。
- `branch_observe_*`：远处支路锁存范围和最长保持帧数。

## RFID 到点转向

状态流为：

```text
follow -> stopping -> turning -> reacquire -> complete
   \----> searching ----/
                  \---- failure ----> fault
```

- 只接受卡号 `1..12`，一次运行只处理第一张有效卡。
- 正常读卡后先输出零速，等待 `stop_settle_ms`，再下发 90° 转向。
- 如果道路还存在但预瞄点已经丢失，会先执行有限距离前进搜索。
- 搜索距离耗尽仍未读卡、STOP 失败或转向失败都会锁定停车。
- 转向完成后，中心线连续有效 `reacquire_frames` 帧才恢复循线。

搜索参数位于 `rfid_turn.search_distance_mm` 和 `rfid_turn.search_speed_mmps`。

## 遇障倒车

状态流为：

```text
idle -> align -> backing -> done
          \--------- failure -> fault
```

1. 障碍检测连续命中 3 帧后触发。
2. 记录触发时沿当前边累计的里程。
3. 使用近处中心线原地摆正；中心线不可用或摆正超时则直接进入倒车。
4. 下发 `backward <distance_mm> <speed_mmps>`，距离不超过 700 mm。
5. 收到 `BACKWARD_DONE` 后保持停车，等待更高层决定下一步。

当前 `BackupConfig` 使用代码内默认值，尚未接入 YAML。RFID 事件会把沿边进度清零，用来近似表示车辆刚经过一个已知节点。

## 日志怎么看

主循环每帧会输出速度、道路点数量、预瞄覆盖、路口类型、可通方向、车道中心、车道宽度和道路末端距离。状态切换写到标准错误，例如：

```text
EVENT phase=follow->forward side=left forward_mm=320
EVENT uart=FORWARD_DONE
EVENT rfid=detected card=3 generation=8
EVENT backup=align->backing s=0.42 distance_mm=420
```

排查时优先确认：

1. `command.reason` 是否仍为 `follow`。
2. `jraw` 与稳定后的 `junc` 是否一致。
3. `lane_x`、`lane_w` 和路口距离是否落在配置窗口。
4. UART 是否返回对应的 `*_DONE`，而不是超时或 `*_FAIL`。

## 测试

Python 状态机和视觉几何：

```bash
PYTHONPATH=.:navigation:vision python3 -m unittest discover \
  -s navigation/road_follow/tests -p 'test_*.py'
PYTHONPATH=vision python3 -m unittest vision.ipm_proto.tests.test_junction
```

UART 和其他 C++ 测试：

```bash
ctest --test-dir build --output-on-failure
```

测试覆盖正常完成、动作失败、距离越界、远处支路锁存、道路末端备用判定、RFID 搜索上限、视觉重捕获和速度/动作互斥。

## 使用前检查与限制

- 先架空车轮验证转向方向、STOP 和动作完成回报，再落地低速测试。
- `camera_ahead_of_turn_center_m` 必须实测；错误会直接转化为路口中心停车误差。
- 道路宽度默认按 200 mm 设置，换场地后必须重新标定 BEV 与宽度范围。
- 路口、RFID 和倒车目前是三个独立局部测试模式，尚未由全局路线自动选择。
- 路口和巡检点最后一段锁存摆正后的视觉航向，用 `CMD_VEL` 直行，并用沿边里程结束；90° 转弯仍交给 IMU。不要再用前进动作锁 IMU 航向。
- 遇障倒车完成后不会自动选择另一条边，需要全局规划器接管。
