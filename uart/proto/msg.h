/// @file
/// 消息 ID、枚举常量与各消息的进程内表示。
///
/// 字段定义对应下位机仓库 UART_PROTOCOL.md / UART_MESSAGES.md（固件 v1.2.22，
/// 协议标识 1）。这里的结构体只在进程内使用，**不允许**直接 memcpy 上线，
/// 序列化由 codec 层逐字节完成。
///
/// 本文件不含任何逻辑，只有数据定义；会话约束（何时允许 ARM、速度环与动作环互斥）
/// 属于 sess 层。

#ifndef UART_PROTO_MSG_H_
#define UART_PROTO_MSG_H_

#include <cstdint>

#include "proto/frame.h"

namespace uart {

/// 消息 ID。0x00–0x7F 为主机到 MCU，0x80 以上为 MCU 到主机。
enum class MsgType : uint8_t {
  kHelloReq = 0x01,
  kArmRequest = 0x10,
  kDisarm = 0x11,
  kCmdVel = 0x12,
  kMotionAction = 0x13,
  kAck = 0x80,
  kHelloInfo = 0x81,
  kOdomState = 0x90,
  kImuState = 0x91,
  kImuDebug = 0x92,
  kSystemStatus = 0x93,
  kMotionResult = 0x94,
};

/// ACK 的结果码。取值与固件 `UartAckResult` 一致。
enum class AckResult : uint8_t {
  kOk = 0,
  kDeniedState = 1,
  kDeniedConfig = 2,
  kBadPayload = 3,
  kUnsupported = 4,
  kBusy = 5,
  kVersionMismatch = 6,
};

/// 下位机的远程控制状态。取值与固件 `REMOTE_*` 一致。
enum class RemoteState : uint8_t {
  kDisabled = 0,
  kReady = 1,
  kArmed = 2,
  kFault = 3,
};

/// HELLO_INFO 的 capabilities 位（8 位位图）。
enum CapabilityBit : uint8_t {
  kCapMotor = 1u << 0,
  kCapEncoder = 1u << 1,
  kCapImu = 1u << 2,
  kCapOled = 1u << 3,
};

/// ODOM_STATE 的 status_flags 位（8 位位图）。
enum OdomFlag : uint8_t {
  kOdomValid = 1u << 0,
  kOdomImuFused = 1u << 1,
  kOdomDegradedImu = 1u << 2,
};

/// IMU_STATE 与 IMU_DEBUG 共用的 status_flags 位（8 位位图）。
enum ImuFlag : uint8_t {
  kImuValid = 1u << 0,
  kImuCalibrating = 1u << 1,
  kImuCalibrated = 1u << 2,
  /// 仅供调试观察，禁止进入导航或控制链路。
  kImuNotForNavigation = 1u << 3,
};

/// MOTION_ACTION 的 action 字段。
enum class MotionActionId : uint8_t {
  kStop = 0,
  kForward = 1,
  kBackward = 2,
  kTurnLeft = 3,
  kTurnRight = 4,
  kRotateLeft = 5,
  kRotateRight = 6,
};

/// MOTION_RESULT 的 result 字段。
enum class MotionResultCode : uint8_t {
  kCompleted = 0,
  kInterrupted = 1,
  kImuUnavailable = 2,
};

/// HELLO_REQ (0x01)
struct HelloReq {
  uint8_t protocol_version = kProtocolVersion;
};

/// ARM_REQUEST (0x10)
struct ArmRequest {
  /// 必须等于本次 HELLO_INFO 返回的 boot_id。
  uint32_t boot_id = 0;
};

/// CMD_VEL (0x12)。贴线/微调主接口，无 token。
struct CmdVel {
  /// 车体前向线速度，m/s。
  float linear_x_mps = 0.0f;
  /// 绕 +Z 的角速度，rad/s，逆时针（左转）为正。
  float angular_z_radps = 0.0f;
};

/// MOTION_ACTION (0x13)。路口 90°、急停与持续转向。
struct MotionAction {
  uint8_t action = 0;
  uint8_t quarter_turns = 0;
  uint16_t speed_mmps = 0;
  uint32_t distance_mm = 0;
};

/// ACK (0x80)。无序号，按 `request_type` 配对。
struct Ack {
  uint8_t request_type = 0;
  AckResult result = AckResult::kOk;
};

/// HELLO_INFO (0x81)
struct HelloInfo {
  uint8_t protocol_version = 0;
  /// @ref CapabilityBit 的位组合。
  uint8_t capabilities = 0;
  uint32_t boot_id = 0;
  RemoteState remote_state = RemoteState::kDisabled;
};

/// ODOM_STATE (0x90)。权威平面里程计。
struct OdomState {
  float x_m = 0.0f;
  float y_m = 0.0f;
  float yaw_rad = 0.0f;
  float linear_mps = 0.0f;
  float angular_radps = 0.0f;
  /// @ref OdomFlag 的位组合。未置 kOdomValid 时不得使用。
  uint8_t status_flags = 0;
};

/// IMU_STATE (0x91)
struct ImuState {
  float accel_x = 0.0f;  ///< m/s²
  float accel_y = 0.0f;
  float accel_z = 0.0f;
  float gyro_x = 0.0f;  ///< rad/s
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
  /// 上电后的相对航向，无磁力计时会漂移。
  float yaw = 0.0f;
  /// @ref ImuFlag 的位组合。kImuCalibrated 置起前姿态视为未就绪。
  uint8_t status_flags = 0;
};

/// IMU_DEBUG (0x92)。纯加速度积分，禁止用于导航与控制。
struct ImuDebug {
  float velocity_x_mps = 0.0f;
  float velocity_y_mps = 0.0f;
  float position_x_m = 0.0f;
  float position_y_m = 0.0f;
  float yaw_rad = 0.0f;
  uint8_t status_flags = 0;
};

/// SYSTEM_STATUS (0x93)
struct SystemStatus {
  RemoteState remote_state = RemoteState::kDisabled;
  uint8_t command_saturated = 0;
  uint8_t fault_code = 0;
  uint32_t communication_errors = 0;
};

/// MOTION_RESULT (0x94)。有限动作终态；持续动作不会自动上报完成。
struct MotionResult {
  uint8_t action = 0;
  uint8_t quarter_turns = 0;
  MotionResultCode result = MotionResultCode::kCompleted;
  float final_yaw_rad = 0.0f;
  float target_yaw_rad = 0.0f;
};

}  // namespace uart

#endif  // UART_PROTO_MSG_H_
