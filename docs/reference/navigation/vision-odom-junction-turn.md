# 视觉路口与里程计补盲转弯

本文说明侦查机器人为什么不能只靠道路中心线自然拐弯，以及如何在导航相机视野受限时，用道路分割发现路口、用下位机编码器和 IMU 穿过最后一段盲区，再完成 90° 转弯。

## 问题

赛题 PDF 图 3 给出的道路净宽约为 **200 mm**，相邻格网尺度约为 800 mm。此前 `road_prior.expected_width_m=0.8` 把格网尺度误当成车道宽度，会把正常 200 mm 道路当成过窄区域，也会让路口的大块 mask 更像“正常宽路”。运行配置现以 0.20 m 为期望宽度，上下限只用于容纳施工、分割毛边和 BEV 标定误差。

现有道路分割在直道上表现稳定，但仅按每行 road 像素中位数提取中心线。十字或对称丁字路口的中位数仍在画面中央，Pure Pursuit 因而会继续给出接近零的角速度。与此同时，相机安装较低、俯角较大，50 mm 侧墙和涵洞立柱会遮住横向道路；车辆接近路口后，先前可见的支路可能离开有效视野。

工作区中的 `wall_end.py` 是原图边缘/墙缝实验，现有实拍回放同时出现漏检和直道误检，因此本链路不再把它接入自动控制，只保留用于对照实验。

因此，单一机制无法同时完成三个问题：

1. 道路分割回答“路在哪里、路口还有多远”；
2. 拓扑/任务策略回答“该向左、向右还是直行”；
3. 编码器和 IMU回答“进入视觉盲区后还要走多远、怎样保持直行”。

本文机制用于**没有 RFID 标签的普通路口**。有标签的巡逻位置采用“新读卡事件到点 → 停车 → 按拓扑方向原地转 90°”，两类触发的统一状态约定见 [拓扑定位、RFID 到点与路口转向](topology-rfid-navigation.md)。卡号只负责播报和去重，不决定左右转方向。

## 控制边界

正常循迹继续使用 `CMD_VEL`：道路 mask 经 BEV、中心线和 Pure Pursuit 产生 `(v, omega)`。路口最后一段改用有限 `MOTION_ACTION`，两种运动通道保持互斥。

```text
FOLLOW（分割循迹）
    │ 在 0.55..0.95 m 看到所需方向支路，只锁存方向
    ▼
APPROACH（继续分割循迹，支路被遮挡也不遗忘）
    │ 中央走廊末端进入交接窗
    ▼
FORWARD（下位机定距直行）
    │ 编码器达到 distance_mm，IMU保持动作起始航向
    ▼
TURNING（下位机 N×90°）
    │ 收到 MOTION_RESULT=DONE
    ▼
REACQUIRE（原地重新观察）
    │ 新方向中心线连续稳定
    └──────────────────────────────► FOLLOW
```

里程计只承担几十厘米的局部补盲，不从上一个巡逻点开始开环跑完整个格网。这样可限制轮胎打滑、轮径误差和累计航向漂移的影响。

## 视觉路口距离

`vision/ipm_proto/junction.py` 在 BEV 中先估计近处本车道，再检查车道左右外侧是否存在连续 road 区域。若发现支路，`junction_y_m` 取左右支路有效纵向区间的中心，表示相机地面原点到路口中心的前向距离。`corridor_end_y_m` 则直接从原始 BEV road mask 读取当前车道中央走廊的最远位置，不经过 0.20 m 直道路宽先验，因此路口变宽时不会被先验中心线提前截断。

只有同时满足以下条件才允许交接：

- 稳定类型是 `t_junction`、`cross` 或 `corner`；
- 规划要求的方向确实存在支路；
- 当前视觉速度指令仍为 `follow`，不是 mask 丢失或推理超时；
- 距离连续若干帧位于 `handoff_min_distance_m..handoff_max_distance_m`；
- 计算出的有限前进距离位于安全上下限。

转向方向目前由命令行显式指定，用于单方向实车验证；正式任务应由拓扑路径生成，不能让对称路口的 mask 自行猜测任务方向。

### 侧路不可见时的道路末端备用信号

实车日志显示，支路常在 `0.76..0.90 m` 时可见，进入原交接范围后反而被墙遮挡。现在先把与规划方向一致的远处支路锁存为 `approach`，继续视觉循迹；后续即使 `dirs=-` 也保留方向，直到中央走廊末端进入交接窗。锁存默认最多保留 200 帧，超时而未接近路口就自动清除。

道路末端交接默认要求：

- 单帧类型为 `blocked`；未提前锁存支路时，稳定类型也必须为 `blocked`；
- 当前指令仍是 `follow`，0.45 m 预瞄点尚未丢失；
- `corridor_end_y_m` 位于 `0.48..0.56 m`；
- 近处车道宽度位于 `0.14..0.32 m`，中心偏差不超过 `0.08 m`。

提前锁存支路后，一帧可靠道路末端即可交接；没有锁存时仍要求连续两帧，避免把普通分割抖动当作路口。这些条件使交接发生在 `stop_lookahead` 之前。没有 `--turn-at-junction left/right` 时，此备用信号不会控制车辆。指定方向来自任务策略，不是由不可见的侧路猜测。

## 距离换算

交给下位机的距离为：

```text
forward_distance
  = turn_center_distance_m
  + camera_ahead_of_turn_center_m
  - stop_before_center_m
```

看见侧路时，`turn_center_distance_m` 就是 `junction_y_m`。使用道路末端备用信号时：

```text
turn_center_distance_m
  = corridor_end_y_m
  - road_end_beyond_turn_center_m
```

默认 `road_end_beyond_turn_center_m=0.10 m`，即 200 mm 道宽的一半。这只是符合赛场几何的初值，仍需根据车辆实际停止位置标定。

`camera_ahead_of_turn_center_m` 是相机光心比两轮旋转中心靠前的实测距离。相机在旋转中心前方时取正值。`stop_before_center_m` 用来让车辆提前少量停车，给惯性和机械间隙留余量。

这两个值不能从照片猜测，必须在整车上测量和低速标定。默认均为 0，仅用于软件链路验证，不代表可直接参加实车高速测试。

## 下位机有限动作

现有协议无需扩展：

```text
MOTION_ACTION
  action       = FORWARD (1)
  quarter_turns= 0
  speed_mmps   = 20..400
  distance_mm  = 1..1000
```

动作开始时 MCU 锁存当前 `relative_yaw`，角度 PID 在前进期间修正左右轮目标；编码器融合里程计的 `path_length_m` 达到目标后停车并发 `MOTION_RESULT`。随后上位机再发送 `TURN_LEFT/RIGHT, quarter_turns=1`。

`uart_vel` 的进程内文本接口为：

```text
forward <distance_mm> <speed_mmps>
turn left
turn right
```

完成通知写到标准输出：`FORWARD_DONE/FAIL`、`TURN_DONE/FAIL`。有限动作有上位机超时；超时发送 STOP，绝不能把没有结果当成成功。
任一有限动作失败都会进入锁定停车状态，本次进程不会自行恢复循迹。

## 运行与观测

自动转弯默认关闭。固定左转兼容命令：

```bash
PYTHONPATH=navigation:vision python3 -m road_follow \
  --drive \
  --left-at-junction \
  --uart-bin build-turn/uart/uart_vel
```

显式选择方向：

```bash
PYTHONPATH=navigation:vision python3 -m road_follow \
  --drive \
  --turn-at-junction left \
  --uart-bin build-turn/uart/uart_vel

PYTHONPATH=navigation:vision python3 -m road_follow \
  --drive \
  --turn-at-junction right \
  --uart-bin build-turn/uart/uart_vel
```

每帧日志包含：

```text
mask_px=<原图road像素> bev_px=<BEV road像素>
pts=<先验点数>/<原始点数>/<最终点数> fallback=<是否回退原始中心线>
y=<最终中心线最近..最远米数> look=<是否覆盖预瞄距离>
jraw=<单帧类型> junc=<稳定类型> dirs=<前/左/右开口>
lane_x=<车道中心> lane_w=<车道宽> jdist=<可见侧路中心米数或-->
cend=<中央走廊末端米数或-->
cue=<是否满足交接> cue_src=<side_branch/road_end/none>
cdist=<换算后的转弯中心距离> latched=<是否锁存支路方向>
arm=<连续交接计数> phase=<状态>
```

状态切换和下位机结果另打印 `EVENT phase=...`、`EVENT uart=...`。若车辆停止，先看控制原因：`stop_slow` 是推理超过 200 ms，`stop_road` 是 **BEV** road 像素不足，`stop_centerline` 是最终中心线点数不足，`stop_lookahead` 是中心线没有覆盖 0.45 m 预瞄点。

没有显式转弯参数时只打印观察结果，不会发送有限前进或转弯动作。

## 标定顺序

1. 用地面四点或实测内外参校正 IPM，禁止长期依赖默认 63.3° 视场估计。
2. 测量相机光心到两轮中心的纵向距离，填写 `camera_ahead_of_turn_center_m`。
3. 架空或低风险区域单独测试 `forward 100 50`、`forward 150 50`、`forward 200 50`。
4. 验证直行期间航向保持、完成后自动停车及 `MOTION_RESULT`。
5. 只开日志回放路口，确认直道不进入交接窗。
6. 实车先以 50 mm/s 固定左转，逐次调整交接窗和停车提前量。
7. 左右转稳定后再让拓扑规划提供方向，最后接障碍封边重规划。

## 安全与已知限制

- 当前路口几何只在合成测试上验证过；实拍 ONNX 回放仍有较多 `blocked/unknown`，不能视作完成实车验收。
- 下位机有限距离使用累计路径长度，不是世界坐标投影；适用于短距离补盲，不适用于全场定位。
- 编码器不增长时，有限动作可能无法自行达到距离终点；上位机超时和现场断电手段必须保留。
- IMU 未完成静止校准时下位机会拒绝有限动作。
- `MOTION_RESULT` 超时后只允许 STOP 和人工检查，禁止自动重发 90°，否则可能再转一次。
- `camera_ahead_of_turn_center_m=0` 是待标定值。完成测量前应架空车轮或使用可随时断电的低速测试环境。

## 测试

- `vision/ipm_proto/tests/test_junction.py`：合成直道、丁字、十字、拐角、截断及路口距离。
- `navigation/road_follow/tests/test_junction_turn.py`：视觉连续确认、有限前进、转弯结果和重新捕获状态。
- `uart` C++ 测试：8 字节 `MOTION_ACTION` 编解码、会话互斥和终态处理。

这些测试是无硬件单元/模拟测试；最终验收必须包含实拍回放和低速整车测试。
