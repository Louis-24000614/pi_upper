# 拓扑定位、纯视觉到点与可选 RFID 测试

本文说明当前 `--turn-at-junction` 任务怎样确认拓扑节点、怎样决定转向，以及 RFID 代码现在处于什么位置。

## 当前运行结论

当前主任务不依赖 UID，但没有合并或重写原来的到点逻辑：`patrol_slot` 继续使用原巡检点视觉接近状态机，`junction` 继续使用原普通路口状态机。只删除“读到标签才算到达”这一项；巡检点原有的侧边锁存、检测带、视觉安全阈值和固定 200 mm 全部保留。

```text
视觉中心线循迹
→ 连续看到左侧或右侧支路，锁存“正在接近路口”
→ 继续视觉循迹
→ 正前方检测带中的道路 mask 连续消失
→ 下位机用 IMU 锁定当前方向档位，定距前进 200 mm
→ FORWARD_DONE，确认到达拓扑预期节点
→ 停车 2 秒
→ Agent 根据地图的入边和出边选择直行、左转、右转或倒回
→ 转弯后稳定识别新道路 3 帧，再恢复视觉循迹
```

这条链路不等待 UID，不使用卡号推进 `current_node`，也不会因为读到 RFID 改变路线。下位机即使仍上报 `RFID_EVENT`，`--turn-at-junction` 模式也会丢弃这些事件。

RFID 只保留给显式的 `--turn-at-rfid` 单次硬件测试；它与拓扑巡航互斥。

## 小车怎样知道所在位置

本方案采用先验拓扑定位，不做依赖厘米级全局坐标的 SLAM：

1. 比赛开始时，起点 `0_0` 和车头朝向已知。
2. Agent 在 `config/nav_topology.yaml` 上生成待执行的边序列。
3. 执行器保存 `current_node`、`current_edge` 和 `next_node`。
4. 视觉负责沿路居中和发现路口；编码器只累计当前边进度 `s`。
5. 视觉交接和 200 mm 有限动作完成后，只能上报当前规划中的 `next_node`，不能根据对称路口画面猜位置。
6. Agent 推进节点状态、覆盖当前边并给出下一条边。

出发区顶端有一个真实丁字口 `0_J`：

```text
         0_0
          │  0.15 m
1_2 ──── 0_J ──── 1_3
 │                  │
2_2                2_3
```

起步先以有限动作完成 `0_0 → 0_J` 的 150 mm。收到 `FORWARD_DONE` 后向 Agent 上报到达 `0_J`，再由地图中的下一条边决定原地左转或右转。方向不是由启动参数写死。

## 保留的两类视觉到点条件

YAML 继续用 `patrol_slot` 表示固定巡逻位置，用 `junction` 表示中间路口。两类节点都不读取 UID，但仍运行各自原有的状态机。

| 目标角色 | 最终到达证据 | RFID 是否参与 |
| --- | --- | --- |
| `patrol_slot` | 原逻辑：任一侧端头锁存、检测带连续 3 帧不高于 0.10、原视觉安全门限通过、200 mm 动作完成 | 否 |
| `junction` | 指定侧支路锁存后：道路端头使用检测带交接，十字口使用边末端 ODOM 交接；随后完成 200 mm 动作 | 否 |

没有修改的关键条件如下：

- `patrol_slot` 仍保存实际看到的左侧、右侧或两侧端头；连续满足原来的 `edge_visible_frames` 后锁存。
- `junction` 仍只观察 `--turn-at-junction left/right` 指定侧，并使用原来的投票和几何门限。
- 锁存后即使支路被墙遮挡或退出画面，也继续使用道路分割和中心线纠偏。
- 普通路口支路锁存后，检测带消失或 ODOM 到达 `edge.length_m - 0.20 m` 时发送 `forward 200 50`；后者用于前方道路不会消失的十字口。
- 最后 200 mm 不再依赖视觉转向；MCU 用编码器控制距离，并用 IMU 锁住由上电基准和已完成转弯维护出的当前方向档位。
- 唯一行为变化是：`patrol_slot` 收到 `FORWARD_DONE` 后直接确认当前预期节点，不再等 UID；普通 `junction` 完全不变。

ODOM 末端保护只作用于普通 `junction`，没有扩展到 `patrol_slot`。即使支路已经锁存，若最后 200 mm 交接到边末端仍未启动，也必须停车，不能越过节点继续行驶。

## 拓扑怎样决定转弯

到点后，转向只由 Agent 的“入边 → 出边”计算：

| 几何关系 | 动作 |
| --- | --- |
| 同方向 | `straight`，停稳后恢复视觉循迹 |
| 左侧出边 | 原地 `turn left` |
| 右侧出边 | 原地 `turn right` |
| 下一条就是来路 | `backup`，不原地转 180° |

因此，视觉看到的是左侧支路并不代表一定左转；它只证明车辆接近一个路口。比如 Agent 的下一条边在右侧，即使本次首先锁存的是左侧证据，停车后仍按拓扑右转。

命令中的 `--turn-at-junction right` 继续表示普通路口观察右侧开口；`left` 同理。它不覆盖 Agent 根据入边和出边计算出的最终动作。巡检点的原状态机仍独立记录实际看见的左侧、右侧或两侧。

## 路径状态推进顺序

一次节点交接必须按顺序完成：

```text
视觉确认路口并完成最后 200 mm
→ 到达 next_node
→ current_node = next_node
→ 当前边标记为已覆盖
→ s = 0
→ Agent 取下一条边
→ 计算 straight / left / right / backup
→ 停车 2 秒并执行动作
→ 视觉稳定识别新道路后进入下一路段
```

转弯失败时进入故障停车，不能假装已经进入下一条边。

## 运行命令

当前纯视觉拓扑巡航命令：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive \
  --turn-at-junction right \
  --uart-bin build-turn/uart/uart_vel \
  --record-video 2>&1 | tee data/road/drive_test_2.log
```

整个任务中不会读取 UID 作为到点条件。普通路口仍取 `junction_turn.turn_forward_m=0.20`；巡检点仍取原来的 `rfid_turn.search_step_distance_mm=200`；速度均保持原值 50 mm/s。这次修改没有改变任何参数值。

## RFID 独立测试

如果只想验证读卡器、停车和单次 90° 转弯，可以显式运行：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive \
  --turn-at-rfid right \
  --uart-bin build-turn/uart/uart_vel
```

该模式只响应本进程启动后的第一张有效卡，完成一次转弯后恢复视觉循迹；它不推进完整拓扑路线。不要同时使用 `--turn-at-junction` 和 `--turn-at-rfid`。

当前下位机把 MIFARE 扇区 0、Block 1 的第 0 字节作为 `card_number`，这只属于 RFID 硬件测试约定。若以后重新启用赛场 UID 计分，需要单独确认标签格式，并把“计分/播报”作为旁路功能接回；不能再次让随机卡号决定物理拓扑位置或转向。

## 异常处理

- 没有先稳定看到任何侧向支路：禁止启动最后 200 mm。
- 普通 `junction` 进入地图边长末端仍无视觉路口证据：按原逻辑停车，不伪造到达；该 ODOM 条件不扩展到 `patrol_slot`。
- 200 mm、停车或转弯动作失败：进入故障停车，不自动重发。
- 转弯后道路未连续稳定 3 帧：保持停车，不提前恢复速度。
- `--turn-at-junction` 收到 RFID 帧：直接忽略，不记录到点、不改变 Agent 状态。
