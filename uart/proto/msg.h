/// @file
/// 消息 ID、枚举常量与各消息的进程内表示。
///
/// 字段定义对应下位机仓库 UART_PROTOCOL.md 第 4 至第 6 节。这里的结构体只在
/// 进程内使用，**不允许**直接 memcpy 上线，序列化由 codec 层逐字节完成。
///
/// 本文件不含任何逻辑，只有数据定义；语义约束（哪些状态允许 ARM、token 何时失效）
/// 属于 sess 层。

#ifndef UART_PROTO_MSG_H_
#define UART_PROTO_MSG_H_

#include <cstdint>

namespace uart {

/// 消息 ID。0x00–0x7F 为主机到 MCU，0x80 以上为 MCU 到主机。
enum class MsgType : uint8_t {
  kHelloReq = 0x01,
  kTimeSyncReq = 0x02,
  kArmRequest = 0x10,
  kDisarm = 0x11,
  kCmdVel = 0x12,
  kResetOdom = 0x13,
  kClearFaultRequest = 0x14,
  kAck = 0x80,
  kHelloInfo = 0x81,
  kTimeSyncResp = 0x82,
  kOdomState = 0x90,
  kImuState = 0x91,
  kImuDebug = 0x92,
  kSystemStatus = 0x93,
  kFaultEvent = 0x94,
};

/// ACK 的结果码。
enum class AckResult : uint8_t {
  kOk = 0,
  /// 已接受，等待 K2 确认或后续条件。
  kPending = 1,
  kDeniedState = 2,
  /// 硬件绑定或底盘参数无效。当前固件参数未补齐时 ARM 会返回这个，属于预期的安全行为。
  kDeniedConfig = 3,
  kBadPayload = 4,
  kBadToken = 5,
  kUnsupported = 6,
  kBusy = 7,
  kVersionMismatch = 8,
};

/// 下位机的远程控制状态。
enum class RemoteState : uint8_t {
  kDisconnected = 0,
  kDisarmed = 1,
  /// 已受理 ARM 请求，等待操作者在 10 s 窗口内短按 K2。
  kArmPending = 2,
  kArmed = 3,
  kFault = 4,
};

/// HELLO_INFO 的 capabilities 位。表示固件构建与板级绑定情况，**不等于**传感器当前健康状态。
enum CapabilityBit : uint32_t {
  kCapMotor = 1u << 0,
  kCapEncoder = 1u << 1,
  kCapImu = 1u << 2,
  kCapOled = 1u << 3,
};

/// ODOM_STATE 的 status_flags 位。
enum OdomFlag : uint32_t {
  kOdomValid = 1u << 0,
  kOdomImuFused = 1u << 1,
  kOdomDegradedImu = 1u << 2,
  kOdomCommandSaturated = 1u << 3,
};

/// IMU_STATE 与 IMU_DEBUG 共用的 status_flags 位。
enum ImuFlag : uint32_t {
  kImuValid = 1u << 0,
  kImuCalibrating = 1u << 1,
  kImuCalibrated = 1u << 2,
  /// 仅供调试观察，禁止进入导航或控制链路。
  kImuNotForNavigation = 1u << 3,
};

/// SYSTEM_STATUS 中 command_age_ms 的"无有效命令"取值。
constexpr uint32_t kCommandAgeNone = 0xFFFFFFFFu;

/// TIME_SYNC_REQ (0x02)
struct TimeSyncReq {
  /// 主机单调时间，纳秒。注意与 MCU 侧的微秒单位不同。
  uint64_t t1_host_ns = 0;
};

/// ARM_REQUEST (0x10)
struct ArmRequest {
  /// 必须等于本次 HELLO_INFO 返回的 boot_id，否则会被拒绝。
  uint32_t boot_id = 0;
};

/// CMD_VEL (0x12)
struct CmdVel {
  uint32_t arm_token = 0;
  /// 车体前向线速度，m/s。
  float linear_x_mps = 0.0f;
  /// 绕 +Z 的角速度，rad/s，逆时针（左转）为正。
  float angular_z_radps = 0.0f;
};

/// ACK (0x80)
struct Ack {
  uint16_t request_sequence = 0;
  uint8_t request_type = 0;
  AckResult result = AckResult::kOk;
  /// 无有效 token 时为 0。
  uint32_t arm_token = 0;
};

/// HELLO_INFO (0x81)
struct HelloInfo {
  uint8_t protocol_version = 0;
  uint8_t fw_major = 0;
  uint8_t fw_minor = 0;
  uint8_t fw_patch = 0;
  /// @ref CapabilityBit 的位组合。
  uint32_t capabilities = 0;
  /// 本次上电的唯一标识。变化意味着下位机复位，必须重新建链。
  uint32_t boot_id = 0;
  /// 为 0 时禁止发起 ARM。
  uint8_t config_valid = 0;
  RemoteState remote_state = RemoteState::kDisconnected;
  uint8_t peer_version_echo = 0;
};

/// TIME_SYNC_RESP (0x82)
struct TimeSyncResp {
  /// 原样回显的主机时间，ns。
  uint64_t t1_host_ns = 0;
  /// MCU 收到请求的时刻，µs。
  uint64_t t2_mcu_rx_us = 0;
  /// MCU 发出响应的时刻，µs。
  uint64_t t3_mcu_tx_us = 0;
};

/// ODOM_STATE (0x90)。权威平面里程计，来自编码器平移量与陀螺仪 Z 轴转角融合。
struct OdomState {
  int64_t left_ticks = 0;
  int64_t right_ticks = 0;
  float left_speed_mps = 0.0f;
  float right_speed_mps = 0.0f;
  float x_m = 0.0f;
  float y_m = 0.0f;
  float yaw_rad = 0.0f;
  float linear_mps = 0.0f;
  float angular_radps = 0.0f;
  float path_length_m = 0.0f;
  /// @ref OdomFlag 的位组合。未置 kOdomValid 时不得使用。
  uint32_t status_flags = 0;
};

/// IMU_STATE (0x91)
struct ImuState {
  float accel_x = 0.0f;  ///< m/s²
  float accel_y = 0.0f;
  float accel_z = 0.0f;
  float gyro_x = 0.0f;  ///< rad/s
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
  float temperature = 0.0f;  ///< °C
  float roll = 0.0f;         ///< rad
  float pitch = 0.0f;
  float yaw = 0.0f;
  /// @ref ImuFlag 的位组合。kImuCalibrated 置起前姿态视为未就绪。
  uint32_t status_flags = 0;
};

/// IMU_DEBUG (0x92)。纯加速度积分的漂移观察接口，禁止用于导航与控制。
struct ImuDebug {
  float velocity_x_mps = 0.0f;
  float velocity_y_mps = 0.0f;
  float position_x_m = 0.0f;
  float position_y_m = 0.0f;
  float yaw_rad = 0.0f;
  uint32_t status_flags = 0;
};

/// SYSTEM_STATUS (0x93)。ARM token 只在这里上报，是上位机取 token 的唯一来源。
struct SystemStatus {
  RemoteState remote_state = RemoteState::kDisconnected;
  uint8_t config_valid = 0;
  uint8_t command_saturated = 0;
  uint32_t fault_code = 0;
  uint32_t uptime_ms = 0;
  /// 距上一条有效命令的毫秒数；无有效命令时为 @ref kCommandAgeNone。
  uint32_t command_age_ms = 0;
  uint32_t crc_errors = 0;
  uint32_t format_errors = 0;
  /// 协议解码与 UART 溢出的累计值。
  uint32_t rx_overflows = 0;
  uint32_t tx_drops = 0;
  /// 非 ARMED 时为 0。
  uint32_t arm_token = 0;
};

/// FAULT_EVENT (0x94)
struct FaultEvent {
  uint32_t fault_code = 0;
  uint32_t timestamp_ms = 0;
  /// 故障相关的补充值，含义随 fault_code 而定。
  uint32_t detail = 0;
};

}  // namespace uart

#endif  // UART_PROTO_MSG_H_
