/// @file
/// 会话层：建链、ARM 流程、token 生命周期、50 Hz 速度下发、超时与重连。
///
/// 这是本模块安全逻辑最集中的一层。设计前提是**下位机才是安全的主体**：token 由
/// 它生成、ARM 要现场按键确认、250 ms 收不到有效命令它自己刹车。上位机这边做的
/// 事情是不制造坏命令、不把过期命令当有效、不越过 config_valid 去请求使能。
///
/// 不拥有线程、不做阻塞等待：@ref Session::Poll 由上层的通信线程反复调用，
/// 时间全部来自注入的 @ref Clock，因此整层可以用假传输加假时钟完整测试。

#ifndef UART_LINK_SESS_H_
#define UART_LINK_SESS_H_

#include <cstdint>

#include "link/clock.h"
#include "proto/codec.h"
#include "proto/frame.h"
#include "proto/msg.h"
#include "link/port.h"

namespace uart {

/// 上位机这一侧看到的链路状态。与下位机上报的 @ref RemoteState 是两件事：
/// 前者描述"我能不能跟它说话"，后者描述"它允不允许动"。
enum class LinkState {
  /// 传输未打开，或已因错误关闭。
  kClosed,
  /// 已打开，正在重复发 HELLO_REQ 等 HELLO_INFO。
  kConnecting,
  /// 已拿到 boot_id，可以正常收发。
  kConnected,
};

/// 最近一次收到的各类遥测。带接收时刻，便于上层判断新鲜度。
struct Telemetry {
  OdomState odom;
  uint64_t odom_us = 0;
  bool has_odom = false;

  ImuState imu;
  uint64_t imu_us = 0;
  bool has_imu = false;

  /// 仅供漂移观察，禁止进入导航或控制链路。
  ImuDebug imu_debug;
  uint64_t imu_debug_us = 0;
  bool has_imu_debug = false;

  SystemStatus status;
  uint64_t status_us = 0;
  bool has_status = false;

  FaultEvent fault;
  uint64_t fault_us = 0;
  bool has_fault = false;

  HelloInfo hello;
  bool has_hello = false;

  Ack last_ack;
  bool has_ack = false;
};

/// 时间同步的估计结果，用 NTP 风格的四时间戳法。
struct TimeSync {
  bool valid = false;
  /// MCU 微秒时钟加上该偏移即为本地微秒时钟。可能为负。
  int64_t offset_us = 0;
  /// 采纳该样本时的往返时延。
  uint64_t rtt_us = 0;
};

/// 链路诊断计数，供上报 /diagnostics 与界面显示。
struct Diagnostics {
  /// 收帧统计（帧数、CRC 错误、格式错误、溢出）。
  Reassembler::Stats rx;
  uint32_t tx_frames = 0;
  /// 发送失败次数（传输报错）。
  uint32_t tx_errors = 0;
  uint32_t cmd_frames = 0;
  /// 因本地指令过期而改发零速的次数。
  uint32_t zero_substitutions = 0;
  uint32_t hello_sent = 0;
  uint32_t arm_requests = 0;
  /// 对端协议版本与本端不一致的次数。
  uint32_t version_mismatches = 0;
  /// 因已有管理请求在等 ACK 而被拒绝发出的请求数。
  uint32_t requests_refused_busy = 0;
  /// 管理请求等 ACK 超时的次数。
  uint32_t ack_timeouts = 0;
  /// 因超时或错误导致链路掉线的次数。
  uint32_t link_drops = 0;
  /// 因 boot_id 变化而丢弃会话的次数。
  uint32_t boot_id_changes = 0;
};

/// 会话时序参数。默认值按协议要求选定，改动前先读注释。
struct SessionConfig {
  /// HELLO_REQ 重发间隔。
  uint32_t hello_retry_ms = 500;
  /// 速度下发周期。协议要求 50 Hz，即 20 ms；下位机 250 ms 无有效命令就刹车。
  uint32_t cmd_period_ms = 20;
  /// 上层指令的本地有效期。超期后改发零速而不是保持旧值——
  /// 协议明确禁止把低频"最后速度保持"当作控制方式。
  uint32_t cmd_validity_ms = 200;
  /// 多久没有任何有效帧就认为链路断开。协议里固件侧用的是约 1 s。
  uint32_t link_timeout_ms = 1000;
  /// 时间同步请求间隔，0 表示不做时间同步。
  uint32_t time_sync_period_ms = 1000;
  /// 管理请求等待 ACK 的超时。协议 v2 没有序号，ACK 只能按消息类型配对，
  /// 因此同一时刻只允许一个在途的管理请求，超时后才能发下一个。
  uint32_t ack_timeout_ms = 500;
  /// 停车时连发多少帧零速再发 DISARM。
  uint32_t stop_zero_frames = 3;
};

/// 上位机侧的协议会话。
///
/// 生命周期：构造后调用 @ref Start 进入建链，之后由通信线程持续调用 @ref Poll。
/// 退出前调用 @ref Shutdown 以尽力零速加 DISARM 收尾。
///
/// 非线程安全。上层跨线程写入速度指令应在外面加锁或走消息队列。
class Session {
 public:
  /// @p port 与 @p clock 的生命周期必须长于本对象。
  Session(Transport& port, const Clock& clock, const SessionConfig& config = {});

  /// 进入建链流程，重置本地会话状态（不清诊断计数）。
  void Start();

  /// 推进一次会话：读取并处理收到的字节，按需发送 HELLO、时间同步和速度命令。
  ///
  /// 非阻塞，可以按任意频率调用；实际发送节奏由内部时间戳控制。
  /// 通信线程的典型用法是 `port.WaitReadable(几毫秒)` 之后调用一次。
  void Poll();

  /// 写入目标速度（覆盖式最新值，不排队）。
  ///
  /// 非有限值会被拒绝并计入诊断，不会污染下发路径。每次写入都会刷新本地有效期。
  /// @return 是否被接受。
  bool SetVelocity(float linear_x_mps, float angular_z_radps);

  /// 请求使能。仅在已建链、`config_valid=1` 且下位机处于 DISARMED 时才真正发出。
  ///
  /// 发出后需要操作者在 10 s 内现场短按 K2，上位机无法单方面完成使能。
  /// @return 是否已发出请求。
  bool RequestArm();

  /// 请求解除使能，并立即把本地目标速度归零。
  bool RequestDisarm();

  /// 请求清零里程计。下位机仅在非 ARMED 状态接受。
  bool RequestResetOdom();

  /// 表达清故障意图。不会绕过现场长按 K2 的确认。
  bool RequestClearFault();

  /// 退出前的收尾：连发若干帧零速，再发 DISARM。传输已断时静默返回。
  void Shutdown();

  LinkState link_state() const { return link_state_; }

  /// 下位机上报的远程状态。没有有效状态包时为 kDisconnected。
  RemoteState remote_state() const { return remote_state_; }

  /// 当前 ARM token，0 表示未持有。
  uint32_t arm_token() const { return arm_token_; }

  const SessionConfig& config() const { return config_; }

  /// 本次会话的 boot_id，未建链时为 0。
  uint32_t boot_id() const { return boot_id_; }

  /// 对端上报的协议版本，未收到 HELLO_INFO 时为 0。
  /// 与 @ref kProtocolVersion 不一致时不会进入已连接状态。
  uint8_t peer_protocol_version() const { return peer_protocol_version_; }

  /// 是否有管理请求正在等待 ACK。为真时新的管理请求会被拒绝。
  bool request_pending() const { return pending_request_type_ != 0; }

  /// 下位机是否报告配置有效。为假时不允许请求 ARM。
  bool config_valid() const { return config_valid_; }

  const Telemetry& telemetry() const { return telemetry_; }
  const TimeSync& time_sync() const { return time_sync_; }
  const Diagnostics& diagnostics() const { return diagnostics_; }

 private:
  /// 发送一帧。payload 可为空。
  bool Send(MsgType type, const uint8_t* payload, size_t payload_len);
  /// 发送零 payload 的管理命令。
  bool SendEmpty(MsgType type);
  /// 处理一条解出来的完整帧。
  void OnFrame(uint8_t msg_type, const uint8_t* payload, size_t len);
  void OnHelloInfo(const uint8_t* payload, size_t len);
  void OnSystemStatus(const uint8_t* payload, size_t len);
  void OnTimeSyncResp(const uint8_t* payload, size_t len, uint64_t rx_us);
  void OnAck(const uint8_t* payload, size_t len);
  /// 发出一个需要 ACK 的管理请求。已有在途请求时拒绝，避免无法配对响应。
  bool SendRequest(MsgType type, const uint8_t* payload, size_t payload_len);
  /// 丢掉当前会话状态（token、boot_id、遥测新鲜度），回到建链。
  void DropSession();
  /// 按周期发送速度命令。
  void PumpCommand(uint64_t now_ms);
  void SendCmdVel(float linear, float angular, uint64_t now_ms);
  bool armed() const;

  Transport& port_;
  const Clock& clock_;
  SessionConfig config_;

  Reassembler rx_;
  LinkState link_state_ = LinkState::kClosed;
  RemoteState remote_state_ = RemoteState::kDisconnected;

  uint32_t boot_id_ = 0;
  bool has_boot_id_ = false;
  uint32_t arm_token_ = 0;
  bool config_valid_ = false;
  uint8_t peer_protocol_version_ = 0;

  /// 在途管理请求的消息类型，0 表示没有。协议 v2 无序号，只能按类型配对 ACK。
  uint8_t pending_request_type_ = 0;
  uint64_t pending_request_ms_ = 0;

  float target_linear_ = 0.0f;
  float target_angular_ = 0.0f;
  uint64_t target_set_ms_ = 0;
  bool has_target_ = false;

  uint64_t last_hello_ms_ = 0;
  uint64_t last_cmd_ms_ = 0;
  uint64_t last_sync_ms_ = 0;
  uint64_t last_rx_ms_ = 0;

  Telemetry telemetry_;
  TimeSync time_sync_;
  Diagnostics diagnostics_;
};

}  // namespace uart

#endif  // UART_LINK_SESS_H_
