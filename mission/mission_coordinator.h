/// @file
/// 固定赛场地图上的最终任务编排层。
///
/// 本模块只负责把导航、RFID 到点、语音、障碍物和涵洞等大任务编排起来，
/// 不直接依赖摄像头、模型、Qt 或真实串口。各专项模块通过 Task 接口接入，
/// 尚未实现的专项任务可以先使用空实现，不能阻塞主任务状态机的开发。

#ifndef MISSION_COORDINATOR_H_
#define MISSION_COORDINATOR_H_

#include <array>
#include <cstdint>

namespace mission {

constexpr uint8_t kPatrolPointCount = 12;

/// 任务总状态。状态机按固定地图推进，不按 RFID 数值推断路线。
enum class State : uint8_t {
  kIdle,
  kWaitingForLink,
  kFollowRoad,
  kApproachJunction,
  kTurning,
  kSearchRoad,
  kAtPatrolPoint,
  kAvoidObstacle,
  kTunnel,
  kReturnHome,
  kCompleted,
  kFault,
};

/// 固定地图上下一段路口动作的方向。
enum class TurnDirection : uint8_t {
  kUnknown,
  kStraight,
  kLeft,
  kRight,
};

/// 由导航层提供的本帧路口信息。
struct JunctionObservation {
  bool detected = false;
  bool direction_valid = false;
  TurnDirection direction = TurnDirection::kUnknown;
};

/// 由下位机 RFID_CARD 遥测转换得到的卡信息。
///
/// `card_number` 是卡内数据解码出的编号。比赛中卡号可能变化，不能用它
/// 推断物理巡逻点；物理点由固定地图和导航层的位置判断提供。
struct RfidObservation {
  bool present = false;
  uint8_t card_number = 0;
  uint8_t generation = 0;
};

/// 外部专项任务对本帧环境的摘要。
struct TaskObservation {
  bool link_ready = false;
  bool road_visible = false;
  bool search_timed_out = false;
  bool turn_finished = false;
  bool turn_succeeded = false;
  bool obstacle_hard_blocked = false;
  bool obstacle_clear = true;
  bool tunnel_active = false;
  /// 导航根据固定地图和里程/路口状态确认的物理巡逻点。
  bool patrol_point_detected = false;
  uint8_t patrol_point_id = 0;
  JunctionObservation junction;
  RfidObservation rfid;
};

/// 任务层向底层专项模块发出的运动意图。
enum class MotionIntent : uint8_t {
  kStop,
  kFollowRoad,
  kTurnLeft,
  kTurnRight,
  kSearchRoad,
  kReturnHome,
};

/// 任务状态快照和本帧动作输出。
struct Output {
  State state = State::kIdle;
  MotionIntent motion = MotionIntent::kStop;
  uint8_t target_point = 0;
  uint8_t visited_count = 0;
  std::array<bool, kPatrolPointCount + 1U> visited{};

  /// 最近一次读到的卡号，仅用于日志和调试，不参与路线决策。
  uint8_t last_card_number = 0;

  /// true 表示本帧产生一次新的播报事件；实际播放由 SpeechTask 完成。
  bool speak_requested = false;
  uint8_t speech_id = 0;

  /// 状态或目标发生变化时置 true，便于 GUI/日志按事件刷新。
  bool changed = false;
};

/// 导航专项任务接口。
///
/// 该接口的具体实现负责道路分割、BEV、路口识别、固定地图选岔、
/// CMD_VEL 和 MOTION_ACTION 的实际调用。任务编排层只消费结果，不依赖实现语言。
class NavigationTask {
 public:
  virtual ~NavigationTask() = default;

  /// 通知导航层当前固定地图上的目标巡逻点。
  virtual void SetTargetPoint(uint8_t point_id) = 0;

  /// 请求执行已经确定的左/右转动作；完成结果通过 TaskObservation 返回。
  virtual void RequestTurn(TurnDirection direction) = 0;

  /// 请求普通道路跟踪。
  virtual void FollowRoad() = 0;

  /// 请求转弯后的重新找线。
  virtual void SearchRoad() = 0;

  /// 请求回到固定地图的出发区。
  virtual void ReturnHome() = 0;

  /// 请求立即停车。
  virtual void Stop() = 0;
};

/// 语音专项任务接口。
///
/// 具体实现应把 `speech_id` 转成 USART2 语音命令，交给下位机 UART4
/// 和语音模块播放；本模块不直接操作 UART4。
class SpeechTask {
 public:
  virtual ~SpeechTask() = default;

  /// 请求播放一个预录音频编号，成功提交返回 true。
  virtual bool Speak(uint8_t speech_id) = 0;
};

/// 涵洞检测专项任务接口。当前没有实现时可传 nullptr。
class TunnelTask {
 public:
  virtual ~TunnelTask() = default;

  /// 更新一帧涵洞检测，返回是否处于涵洞相关处理阶段。
  virtual bool Update() = 0;
};

/// 障碍物专项任务接口。当前没有实现时可传 nullptr。
class ObstacleTask {
 public:
  virtual ~ObstacleTask() = default;

  /// 更新一帧障碍处理，返回是否已经完成重新规划并可继续行驶。
  virtual bool ReplanFinished() = 0;
};

/// 专项任务依赖集合；未实现的任务允许为空。
struct Services {
  NavigationTask* navigation = nullptr;
  SpeechTask* speech = nullptr;
  TunnelTask* tunnel = nullptr;
  ObstacleTask* obstacle = nullptr;
};

/// 固定地图任务协调器。
///
/// 默认巡逻顺序是 P1 到 P12，再回到出发区。RFID 只确认导航已经定位的
/// 当前物理点，不决定地图路线，也不需要建立卡号到点位的映射。
class Coordinator {
 public:
  /// 创建任务协调器，并绑定可选的导航、语音、涵洞和障碍物专项模块。
  explicit Coordinator(Services services = {});

  /// 重置任务，回到等待上位机通信链路的状态。
  void Reset();

  /// 开始固定地图巡逻；链路尚未就绪时会停留在等待状态。
  void Start();

  /// 请求停止任务并清除当前运动意图。
  void Stop();

  /// 消费一帧传感器/专项任务结果并推进状态机。
  void Tick(const TaskObservation& observation);

  /// 获取最近一次任务输出快照。
  const Output& output() const { return output_; }

 private:
  static bool ValidPoint(uint8_t point_id);
  static uint8_t CountVisited(const std::array<bool, kPatrolPointCount + 1U>& visited);
  void SetState(State state);
  void SetMotion(MotionIntent motion);
  void SetTargetPoint(uint8_t point_id);
  void HandleRfid(const TaskObservation& observation);
  void HandleJunction(const JunctionObservation& junction);
  void HandleTurnResult(const TaskObservation& observation);
  void HandleObstacle(const TaskObservation& observation);
  void HandleTunnel(const TaskObservation& observation);
  void EnterReturnHome();
  void CompleteMission();

  Services services_;
  State state_ = State::kIdle;
  bool started_ = false;
  bool return_home_started_ = false;
  std::array<uint8_t, kPatrolPointCount> patrol_order_{};
  std::array<bool, kPatrolPointCount + 1U> visited_{};
  uint8_t route_index_ = 0;
  uint8_t last_rfid_generation_ = 0;
  bool have_last_rfid_generation_ = false;
  bool pending_card_ = false;
  uint8_t pending_card_number_ = 0;
  Output output_{};
};

}  // namespace mission

#endif  // MISSION_COORDINATOR_H_
