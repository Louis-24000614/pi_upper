# Mission 任务编排层

## 作用

`mission/` 是上位机最终任务的编排层。它不负责直接读取摄像头、加载模型或操作串口，而是把几个相互独立的大任务连接起来：

```text
导航 / 路口 / 回场
        |
RFID 到点确认 ---- Mission Coordinator ---- 语音播报
        |                    |
隧道通过逻辑          障碍物重规划
```

当前固定赛场地图中的巡逻点位置不变，但比赛卡片的 UID 和卡号可能变化。因此任务层不能通过卡号判断物理点位，也不能把卡号写成 `card_number -> point_id` 的固定映射。

正确分工是：

```text
固定拓扑和导航进度 -> 判断当前到了哪个物理巡逻点
RFID_CARD           -> 判断是否出现了一张新卡
Mission             -> 将“新卡 + 当前物理点”确认成一次到达事件
SpeechTask          -> 播报该物理点对应的音频
```

核心实现位于 `mission_coordinator.h/.cpp`，构建目标为 `mission_core`。

## 调用方式

上层运行线程需要创建一次 `mission::Coordinator`，然后周期性调用 `Tick()`：

```cpp
mission::Services services{
    &navigation_task,
    &speech_task,
    &tunnel_task,
    &obstacle_task,
};

mission::Coordinator mission(services);
mission.Start();

for (;;) {
  mission::TaskObservation observation = CollectObservation();
  mission.Tick(observation);
  const mission::Output& output = mission.output();
  PublishMissionState(output);
}
```

`Coordinator` 不创建线程，也不拥有传入的专项任务对象。调用方必须保证这些对象在协调器使用期间一直有效。`Tick()` 应在固定周期的任务线程中调用，建议周期为 20～50 ms；专项模型可以低频运行，但应把最近一次有效结果填入观察量。

## 输入接口

### `TaskObservation`

`TaskObservation` 是专项模块传给 Mission 的单帧摘要。

| 字段 | 来源 | 含义 |
|---|---|---|
| `link_ready` | `uart::Session` | 上下位机已建链且允许运动 |
| `road_visible` | 道路分割 / BEV | 当前是否能继续进行道路跟踪 |
| `search_timed_out` | 导航层 | 转弯后找线超时，进入故障停车 |
| `turn_finished` | `uart::Session::telemetry()` | 下位机有限转弯是否已经结束 |
| `turn_succeeded` | `MotionResult` | 转弯是否正常完成 |
| `obstacle_hard_blocked` | 障碍物层 | 当前道路是否已经无法安全通过 |
| `obstacle_clear` | 障碍物层 | 当前前方是否恢复可通行 |
| `tunnel_active` | 拓扑 / 隧道层 | 当前是否处于隧道处理区间 |
| `patrol_point_detected` | 导航层 | 是否确认到达固定物理巡逻点 |
| `patrol_point_id` | 导航层 | 已确认的物理点编号，范围为 1～12 |
| `rfid` | `RFID_CARD` 遥测 | 当前卡是否在场、卡号、卡片世代号 |

其中最重要的是 `patrol_point_detected` 和 `patrol_point_id`。它们必须由导航层依据固定地图、当前拓扑边和里程进度产生，不能由 RFID 卡号直接产生。

### `RfidObservation`

```cpp
struct RfidObservation {
  bool present;
  uint8_t card_number;
  uint8_t generation;
};
```

下位机的 `RFID_CARD` 帧已经包含这三个字段。上位机 UART 层只需把最近一帧复制到 `TaskObservation::rfid`：

- `present == 1`：当前读卡器认为有卡；
- `generation` 变化：出现一次新的卡到达事件；
- `card_number`：保留用于日志和调试，不能用于推断物理巡逻点。

同一张卡持续贴在读卡器附近时，`generation` 不变，Mission 不会重复播报。卡片离开后再出现，或者出现另一张卡时，读卡器应产生新的世代号。

## 输出接口

### `Output`

`Coordinator::output()` 返回最近一次任务快照：

| 字段 | 用途 |
|---|---|
| `state` | GUI、日志和上层调度显示当前任务状态 |
| `motion` | 当前期望的运动意图 |
| `target_point` | 当前固定地图巡逻目标，1～12；回场时为 0 |
| `visited` | 已完成巡逻点集合 |
| `visited_count` | 已完成巡逻点数量 |
| `last_card_number` | 最近一次读到的卡号，仅用于日志 |
| `speak_requested` | 本次 `Tick()` 是否产生新的播报事件 |
| `speech_id` | 播报编号，当前默认等于物理巡逻点编号 |
| `changed` | 状态或目标是否发生变化 |

`Output` 是状态结果，不是底层串口命令。真正发送 `CMD_VEL`、`MOTION_ACTION` 或语音帧的工作由对应专项实现完成。

## 专项接口实现要求

### 1. `NavigationTask`

导航层负责“固定地图怎么走”，实现以下行为：

```cpp
class NavigationTaskImpl final : public mission::NavigationTask {
 public:
  void SetTargetPoint(uint8_t point_id) override;
  void RequestTurn(mission::TurnDirection direction) override;
  void FollowRoad() override;
  void SearchRoad() override;
  void ReturnHome() override;
  void Stop() override;
};
```

实现应持有固定拓扑图，并维护至少这些运行时变量：

```text
current_node       当前拓扑节点
current_edge       当前道路边
edge_progress_m    当前边上的里程
target_point       当前巡逻目标
blocked_edges      已被障碍物封闭的道路
navigation_state   FOLLOW / APPROACH_JUNCTION / TURNING / SEARCH_LINE / RETURN_HOME
```

职责如下：

1. 根据 `target_point` 和当前拓扑位置规划下一条边；
2. 普通道路使用道路分割、BEV、中心线和 `Session::SetVelocity()`；
3. 提前识别路口，在相机丢线前决定直行、左转或右转；
4. 左右转调用 `Session::RequestMotionAction()`；
5. 转弯期间不因道路暂时不可见而立即停车；
6. 收到 `MOTION_RESULT` 后进入 `SearchRoad()`；
7. 重新看到道路后恢复 `FollowRoad()`；
8. 根据固定物理位置和 `edge_progress_m` 设置 `patrol_point_detected` 与 `patrol_point_id`；
9. 全部巡逻点完成后规划并执行回出发区。

运动通道必须保持互斥：

```text
普通寻线、微调、倒车 -> CMD_VEL
路口 90 度转弯、急停 -> MOTION_ACTION
```

不要用下位机的“锁航向前进”替代长距离视觉寻线。IMU 只适合辅助短时间转角，当前赛道的路线判断由固定拓扑和视觉完成。

### 2. `SpeechTask`

语音采用下位机 UART4 模块，完整链路应为：

```text
Mission::SpeechTask
    -> 上位机 USART2 语音播放消息
    -> 下位机 UART2 协议解析
    -> Speaker_Speak(audio_id)
    -> UART4 / PC10
    -> 语音模块
```

下位机已经有 `Speaker_Speak(Audioname_t)`，上下位机 USART2 协议现在使用 `SPEAK_AUDIO (0x14)` 传递音频编号。`SpeechTask::Speak()` 的具体实现应调用 `uart::Session::RequestSpeech(speech_id)`，并根据返回值记录“已提交”或“未提交”。

建议让语音层负责编号映射，不要让 Mission 直接依赖语音模块的文件名：

```text
巡逻点 1 -> ARRIVE_1
巡逻点 2 -> ARRIVE_2
...
涵洞嫌疑人 -> INSPECT_SUSPECT
涵洞物体   -> INSPECT_OBJECT
隧道通过   -> TUNNEL_DONE
```

如果语音模块最终只支持 1～12 号文件，应在 `SpeechTask` 内部校验编号，发送失败时记录错误，但不要重复推进巡逻点状态。

### 3. 涵洞侦查任务

当前接口中的 `TunnelTask` 只对应“隧道通过”占位，不等同于规则中的“涵洞目标识别”。后续应增加独立的涵洞接口，例如：

```cpp
class CulvertInspectionTask {
 public:
  virtual ~CulvertInspectionTask() = default;
  virtual bool Update(const Frame& frame) = 0;
  virtual bool HasResult() const = 0;
  virtual InspectionTarget result() const = 0;
};
```

涵洞层的职责：

- 根据固定地图或导航触发条件判断进入哪个涵洞；
- 使用侧向识别相机采图；
- 识别“嫌疑人”或“物体”；
- 对同一涵洞只产生一次有效结果；
- 输出识别结果给 Mission；
- 由 `SpeechTask` 播报对应内容。

不要把涵洞目标识别直接写入 `NavigationTask`，否则道路跟踪、障碍重规划和目标识别会相互耦合。

### 4. `TunnelTask`

隧道是地图中的固定路段，当前由 `nav_topology.yaml` 的 `tunnel: true` 标记。实现应维护：

```text
tunnel_id
entered
passed
exit_detected
```

进入隧道后：

- 暂停依赖正常光照的道路判断；
- 以低速 `CMD_VEL`、编码器和短时 IMU 保持前进；
- 设置超时和最大距离保护；
- 重新看到出口道路后恢复视觉跟踪；
- 每段隧道只上报一次通过事件。

### 5. `ObstacleTask`

障碍物层不能只返回“检测到框”，还需要完成从检测到重规划的闭环：

```text
检测框
 -> 投影到 BEV
 -> 判断可通行宽度
 -> 软避障或硬堵塞
 -> 安全停车
 -> 必要时退回上一拓扑节点
 -> 标记 blocked edge
 -> Dijkstra 重新规划
 -> 新边正向寻线
```

`obstacle_hard_blocked` 用于触发 `kAvoidObstacle`。`ReplanFinished()` 只有在已经完成必要的封边、回退和新路径选择后才应返回 true，不能在刚检测到障碍物时立即返回 true。

## RFID 到点确认流程

固定物理点和随机卡号的完整流程如下：

```text
导航根据固定地图判断接近 P3
    -> 导航输出 patrol_point_detected=true, patrol_point_id=3
    -> 下位机上报 RFID_CARD present=1, generation 变化
    -> Mission 确认“P3 到达”
    -> visited[3]=true
    -> speech_id=3
    -> 选择下一个未完成巡逻点
```

如果读到了新卡但导航没有确认具体物理点，Mission 不应猜测点号，也不应播报。这样可以避免卡号变化导致错误播报和扣分。

## 当前状态机

当前协调器已经定义这些状态：

```text
IDLE
WAITING_FOR_LINK
FOLLOW_ROAD
APPROACH_JUNCTION
TURNING
SEARCH_ROAD
AT_PATROL_POINT
AVOID_OBSTACLE
TUNNEL
RETURN_HOME
COMPLETED
FAULT
```

当前文件是任务层骨架，不代表所有下层能力已经完成。尤其以下部分仍需专项实现：

- 固定拓扑的实时定位和路口选边；
- 转弯前的路口检测和转弯后的重新找线；
- 将 `SpeechTask` 绑定到真实的 `uart::Session`，并处理 `RequestSpeech()` 的 ACK/超时结果；
- 4 段隧道的进入、通过和出口判断；
- 8 个涵洞的嫌疑人/物体识别；
- 障碍物 BEV 投影、倒车和重新规划；
- GUI 对 `Output` 的展示和正式任务启动入口。

## 推荐接入顺序

1. 先实现 `NavigationTask` 的固定地图、路口和回场逻辑；
2. 接入 `uart::Session`，打通 `CMD_VEL`、`MOTION_ACTION` 和 `MOTION_RESULT`；
3. 用下位机 `RFID_CARD` 遥测验证“新卡 + 物理点”确认流程；
4. 将 `SpeechTask` 接入 `uart::Session::RequestSpeech()`，完成任务层的语音播报；
5. 接入隧道通过状态机；
6. 接入涵洞识别和一次性播报；
7. 接入障碍物重规划；
8. 最后把 `Output` 接入 GUI、日志和比赛计分。

每个专项模块都应先提供 fake/mock 实现，使 Mission 可以脱离摄像头、NPU 和真实小车进行状态机测试。
