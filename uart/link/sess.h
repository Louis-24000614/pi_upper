/// @file
/// 会话层：建链、ARM（以 ACK 判定配置）、速度环与动作环互斥、超时与重连。
///
/// 下位机固件 v1.2.22：无 ARM token、无通信超时看门狗、上电可直接 ARMED。
/// 上位机仍必须先 HELLO 再 ARM_REQUEST，并以 ACK 判断是否允许动。
/// 贴线走 CMD_VEL；路口 90° / 急停走 MOTION_ACTION；同一时刻只走一条。
///
/// 不拥有线程：@ref Session::Poll 由通信线程反复调用。

#ifndef UART_LINK_SESS_H_
#define UART_LINK_SESS_H_

#include <cstdint>

#include "link/clock.h"
#include "link/port.h"
#include "proto/codec.h"
#include "proto/frame.h"
#include "proto/msg.h"

namespace uart {

enum class LinkState {
  kClosed,
  kConnecting,
  kConnected,
};

/// 运动通道。速度环与动作环互斥。
enum class MotionMode {
  kIdle,
  kVelocity,
  kAction,
};

struct Telemetry {
  OdomState odom;
  uint64_t odom_us = 0;
  bool has_odom = false;

  ImuState imu;
  uint64_t imu_us = 0;
  bool has_imu = false;

  ImuDebug imu_debug;
  uint64_t imu_debug_us = 0;
  bool has_imu_debug = false;

  SystemStatus status;
  uint64_t status_us = 0;
  bool has_status = false;

  MotionResult motion;
  uint64_t motion_us = 0;
  bool has_motion = false;

  HelloInfo hello;
  bool has_hello = false;

  Ack last_ack;
  bool has_ack = false;

  RfidCard rfid;
  uint64_t rfid_us = 0;
  bool has_rfid = false;
};

struct Diagnostics {
  Reassembler::Stats rx;
  uint32_t tx_frames = 0;
  uint32_t tx_errors = 0;
  uint32_t cmd_frames = 0;
  uint32_t action_frames = 0;
  uint32_t zero_substitutions = 0;
  uint32_t hello_sent = 0;
  uint32_t arm_requests = 0;
  uint32_t version_mismatches = 0;
  uint32_t requests_refused_busy = 0;
  uint32_t ack_timeouts = 0;
  uint32_t link_drops = 0;
  uint32_t boot_id_changes = 0;
};

struct SessionConfig {
  uint32_t hello_retry_ms = 500;
  /// 速度环刷新周期。寻线建议 20 Hz～50 Hz；MCU 不再靠断流看门狗停车。
  uint32_t cmd_period_ms = 20;
  /// 上层速度指令本地有效期。超期改发零速，避免香橙派卡死时继续用旧速度。
  uint32_t cmd_validity_ms = 200;
  uint32_t link_timeout_ms = 1000;
  uint32_t ack_timeout_ms = 500;
  uint32_t stop_zero_frames = 3;
};

class Session {
 public:
  Session(Transport& port, const Clock& clock, const SessionConfig& config = {});

  void Start();
  void Poll();

  /// 写入目标速度并切到速度环。有限动作未结束时拒绝，避免和 90° 对拧。
  bool SetVelocity(float linear_x_mps, float angular_z_radps);

  /// 请求使能。已建链且非 FAULT 时发出 ARM_REQUEST；是否允许动看 ACK。
  bool RequestArm();
  bool RequestDisarm();

  /// 发送 MOTION_ACTION。STOP 始终可发；有限动作在等待 0x94 时拒绝新的有限动作。
  bool RequestMotionAction(uint8_t action, uint8_t quarter_turns = 0, uint16_t speed_mmps = 0,
                           uint32_t distance_mm = 0);

  void Shutdown();

  LinkState link_state() const { return link_state_; }
  RemoteState remote_state() const { return remote_state_; }
  MotionMode motion_mode() const { return motion_mode_; }
  bool awaiting_motion_result() const { return awaiting_motion_result_; }

  const SessionConfig& config() const { return config_; }
  uint32_t boot_id() const { return boot_id_; }
  uint8_t peer_protocol_version() const { return peer_protocol_version_; }
  bool request_pending() const { return pending_request_type_ != 0; }

  /// 最近一次 ARM_REQUEST 的 ACK 为 OK。
  bool config_valid() const { return config_valid_; }

  /// 已建链、ARM 成功且下位机处于 ARMED。
  bool command_enabled() const { return armed(); }

  const Telemetry& telemetry() const { return telemetry_; }
  const Diagnostics& diagnostics() const { return diagnostics_; }

 private:
  bool Send(MsgType type, const uint8_t* payload, size_t payload_len);
  bool SendEmpty(MsgType type);
  void OnFrame(uint8_t msg_type, const uint8_t* payload, size_t len);
  void OnHelloInfo(const uint8_t* payload, size_t len);
  void OnSystemStatus(const uint8_t* payload, size_t len);
  void OnAck(const uint8_t* payload, size_t len);
  void OnMotionResult(const uint8_t* payload, size_t len);
  bool SendRequest(MsgType type, const uint8_t* payload, size_t payload_len);
  void DropSession();
  void PumpCommand(uint64_t now_ms);
  void SendCmdVel(float linear, float angular, uint64_t now_ms);
  void EnterIdleMotion();
  bool armed() const;
  static bool IsFiniteMotion(uint8_t action, uint32_t distance_mm);
  static bool IsStop(uint8_t action);

  Transport& port_;
  const Clock& clock_;
  SessionConfig config_;

  Reassembler rx_;
  LinkState link_state_ = LinkState::kClosed;
  RemoteState remote_state_ = RemoteState::kDisabled;
  MotionMode motion_mode_ = MotionMode::kIdle;

  uint32_t boot_id_ = 0;
  bool has_boot_id_ = false;
  bool arm_ok_ = false;
  bool config_valid_ = false;
  uint8_t peer_protocol_version_ = 0;
  bool awaiting_motion_result_ = false;

  uint8_t pending_request_type_ = 0;
  uint64_t pending_request_ms_ = 0;

  float target_linear_ = 0.0f;
  float target_angular_ = 0.0f;
  uint64_t target_set_ms_ = 0;
  bool has_target_ = false;

  uint64_t last_hello_ms_ = 0;
  uint64_t last_cmd_ms_ = 0;
  uint64_t last_rx_ms_ = 0;

  Telemetry telemetry_;
  Diagnostics diagnostics_;
};

}  // namespace uart

#endif  // UART_LINK_SESS_H_
