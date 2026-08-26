#include "proto/codec.h"

#include <cmath>

#include "proto/detail/bytes.h"

namespace uart {
namespace {

/// 读取一个 float32 字段，非有限值直接判为非法 payload。
bool ReadFinite(const uint8_t* src, float* out) {
  const float value = GetF32(src);
  if (!std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

/// 把 uint8 安全地映射成 RemoteState。未知取值按断连处理，宁可保守也不要把
/// 未知状态当成 ARMED。
RemoteState ToRemoteState(uint8_t value) {
  return value <= static_cast<uint8_t>(RemoteState::kFault) ? static_cast<RemoteState>(value)
                                                            : RemoteState::kDisconnected;
}

/// 未知的 ACK 结果码映射为 kUnsupported，避免被误判成 kOk。
AckResult ToAckResult(uint8_t value) {
  return value <= static_cast<uint8_t>(AckResult::kVersionMismatch)
             ? static_cast<AckResult>(value)
             : AckResult::kUnsupported;
}

}  // namespace

size_t EncodeHelloReq(const HelloReq& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeHelloReq) {
    return 0;
  }
  dst[0] = msg.protocol_version;
  return kSizeHelloReq;
}

bool DecodeHelloReq(const uint8_t* payload, size_t len, HelloReq* out) {
  if (len != kSizeHelloReq) {
    return false;
  }
  out->protocol_version = payload[0];
  return true;
}

size_t EncodeTimeSyncReq(const TimeSyncReq& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeTimeSyncReq) {
    return 0;
  }
  PutU64(dst, msg.t1_host_ns);
  return kSizeTimeSyncReq;
}

bool DecodeTimeSyncReq(const uint8_t* payload, size_t len, TimeSyncReq* out) {
  if (len != kSizeTimeSyncReq) {
    return false;
  }
  out->t1_host_ns = GetU64(payload);
  return true;
}

size_t EncodeArmRequest(const ArmRequest& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeArmRequest) {
    return 0;
  }
  PutU32(dst, msg.boot_id);
  return kSizeArmRequest;
}

bool DecodeArmRequest(const uint8_t* payload, size_t len, ArmRequest* out) {
  if (len != kSizeArmRequest) {
    return false;
  }
  out->boot_id = GetU32(payload);
  return true;
}

size_t EncodeCmdVel(const CmdVel& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeCmdVel) {
    return 0;
  }
  // 非有限值绝不下发：即使下位机会拒收，也不该把坏命令放到线上。
  if (!std::isfinite(msg.linear_x_mps) || !std::isfinite(msg.angular_z_radps)) {
    return 0;
  }
  PutU32(dst + 0, msg.arm_token);
  PutF32(dst + 4, msg.linear_x_mps);
  PutF32(dst + 8, msg.angular_z_radps);
  return kSizeCmdVel;
}

bool DecodeCmdVel(const uint8_t* payload, size_t len, CmdVel* out) {
  if (len != kSizeCmdVel) {
    return false;
  }
  if (!ReadFinite(payload + 4, &out->linear_x_mps) ||
      !ReadFinite(payload + 8, &out->angular_z_radps)) {
    return false;
  }
  out->arm_token = GetU32(payload + 0);
  return true;
}

size_t EncodeAck(const Ack& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeAck) {
    return 0;
  }
  dst[0] = msg.request_type;
  dst[1] = static_cast<uint8_t>(msg.result);
  PutU32(dst + 2, msg.arm_token);
  return kSizeAck;
}

bool DecodeAck(const uint8_t* payload, size_t len, Ack* out) {
  if (len != kSizeAck) {
    return false;
  }
  out->request_type = payload[0];
  out->result = ToAckResult(payload[1]);
  out->arm_token = GetU32(payload + 2);
  return true;
}

size_t EncodeHelloInfo(const HelloInfo& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeHelloInfo) {
    return 0;
  }
  dst[0] = msg.protocol_version;
  dst[1] = msg.fw_major;
  dst[2] = msg.fw_minor;
  dst[3] = msg.fw_patch;
  PutU32(dst + 4, msg.capabilities);
  PutU32(dst + 8, msg.boot_id);
  dst[12] = msg.config_valid;
  dst[13] = static_cast<uint8_t>(msg.remote_state);
  dst[14] = msg.peer_version_echo;
  dst[15] = 0;
  return kSizeHelloInfo;
}

bool DecodeHelloInfo(const uint8_t* payload, size_t len, HelloInfo* out) {
  if (len != kSizeHelloInfo) {
    return false;
  }
  out->protocol_version = payload[0];
  out->fw_major = payload[1];
  out->fw_minor = payload[2];
  out->fw_patch = payload[3];
  out->capabilities = GetU32(payload + 4);
  out->boot_id = GetU32(payload + 8);
  out->config_valid = payload[12];
  out->remote_state = ToRemoteState(payload[13]);
  out->peer_version_echo = payload[14];
  return true;
}

size_t EncodeTimeSyncResp(const TimeSyncResp& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeTimeSyncResp) {
    return 0;
  }
  PutU64(dst + 0, msg.t1_host_ns);
  PutU64(dst + 8, msg.t2_mcu_rx_us);
  PutU64(dst + 16, msg.t3_mcu_tx_us);
  return kSizeTimeSyncResp;
}

bool DecodeTimeSyncResp(const uint8_t* payload, size_t len, TimeSyncResp* out) {
  if (len != kSizeTimeSyncResp) {
    return false;
  }
  out->t1_host_ns = GetU64(payload + 0);
  out->t2_mcu_rx_us = GetU64(payload + 8);
  out->t3_mcu_tx_us = GetU64(payload + 16);
  return true;
}

size_t EncodeOdomState(const OdomState& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeOdomState) {
    return 0;
  }
  PutI64(dst + 0, msg.left_ticks);
  PutI64(dst + 8, msg.right_ticks);
  PutF32(dst + 16, msg.left_speed_mps);
  PutF32(dst + 20, msg.right_speed_mps);
  PutF32(dst + 24, msg.x_m);
  PutF32(dst + 28, msg.y_m);
  PutF32(dst + 32, msg.yaw_rad);
  PutF32(dst + 36, msg.linear_mps);
  PutF32(dst + 40, msg.angular_radps);
  PutF32(dst + 44, msg.path_length_m);
  PutU32(dst + 48, msg.status_flags);
  return kSizeOdomState;
}

bool DecodeOdomState(const uint8_t* payload, size_t len, OdomState* out) {
  if (len != kSizeOdomState) {
    return false;
  }
  if (!ReadFinite(payload + 16, &out->left_speed_mps) ||
      !ReadFinite(payload + 20, &out->right_speed_mps) || !ReadFinite(payload + 24, &out->x_m) ||
      !ReadFinite(payload + 28, &out->y_m) || !ReadFinite(payload + 32, &out->yaw_rad) ||
      !ReadFinite(payload + 36, &out->linear_mps) ||
      !ReadFinite(payload + 40, &out->angular_radps) ||
      !ReadFinite(payload + 44, &out->path_length_m)) {
    return false;
  }
  out->left_ticks = GetI64(payload + 0);
  out->right_ticks = GetI64(payload + 8);
  out->status_flags = GetU32(payload + 48);
  return true;
}

size_t EncodeImuState(const ImuState& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeImuState) {
    return 0;
  }
  PutF32(dst + 0, msg.accel_x);
  PutF32(dst + 4, msg.accel_y);
  PutF32(dst + 8, msg.accel_z);
  PutF32(dst + 12, msg.gyro_x);
  PutF32(dst + 16, msg.gyro_y);
  PutF32(dst + 20, msg.gyro_z);
  PutF32(dst + 24, msg.temperature);
  PutF32(dst + 28, msg.roll);
  PutF32(dst + 32, msg.pitch);
  PutF32(dst + 36, msg.yaw);
  PutU32(dst + 40, msg.status_flags);
  return kSizeImuState;
}

bool DecodeImuState(const uint8_t* payload, size_t len, ImuState* out) {
  if (len != kSizeImuState) {
    return false;
  }
  if (!ReadFinite(payload + 0, &out->accel_x) || !ReadFinite(payload + 4, &out->accel_y) ||
      !ReadFinite(payload + 8, &out->accel_z) || !ReadFinite(payload + 12, &out->gyro_x) ||
      !ReadFinite(payload + 16, &out->gyro_y) || !ReadFinite(payload + 20, &out->gyro_z) ||
      !ReadFinite(payload + 24, &out->temperature) || !ReadFinite(payload + 28, &out->roll) ||
      !ReadFinite(payload + 32, &out->pitch) || !ReadFinite(payload + 36, &out->yaw)) {
    return false;
  }
  out->status_flags = GetU32(payload + 40);
  return true;
}

size_t EncodeImuDebug(const ImuDebug& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeImuDebug) {
    return 0;
  }
  PutF32(dst + 0, msg.velocity_x_mps);
  PutF32(dst + 4, msg.velocity_y_mps);
  PutF32(dst + 8, msg.position_x_m);
  PutF32(dst + 12, msg.position_y_m);
  PutF32(dst + 16, msg.yaw_rad);
  PutU32(dst + 20, msg.status_flags);
  return kSizeImuDebug;
}

bool DecodeImuDebug(const uint8_t* payload, size_t len, ImuDebug* out) {
  if (len != kSizeImuDebug) {
    return false;
  }
  if (!ReadFinite(payload + 0, &out->velocity_x_mps) ||
      !ReadFinite(payload + 4, &out->velocity_y_mps) ||
      !ReadFinite(payload + 8, &out->position_x_m) ||
      !ReadFinite(payload + 12, &out->position_y_m) || !ReadFinite(payload + 16, &out->yaw_rad)) {
    return false;
  }
  out->status_flags = GetU32(payload + 20);
  return true;
}

size_t EncodeSystemStatus(const SystemStatus& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeSystemStatus) {
    return 0;
  }
  dst[0] = static_cast<uint8_t>(msg.remote_state);
  dst[1] = msg.config_valid;
  dst[2] = msg.command_saturated;
  dst[3] = 0;
  PutU32(dst + 4, msg.fault_code);
  PutU32(dst + 8, msg.uptime_ms);
  PutU32(dst + 12, msg.command_age_ms);
  PutU32(dst + 16, msg.crc_errors);
  PutU32(dst + 20, msg.format_errors);
  PutU32(dst + 24, msg.rx_overflows);
  PutU32(dst + 28, msg.tx_drops);
  PutU32(dst + 32, msg.arm_token);
  return kSizeSystemStatus;
}

bool DecodeSystemStatus(const uint8_t* payload, size_t len, SystemStatus* out) {
  if (len != kSizeSystemStatus) {
    return false;
  }
  out->remote_state = ToRemoteState(payload[0]);
  out->config_valid = payload[1];
  out->command_saturated = payload[2];
  out->fault_code = GetU32(payload + 4);
  out->uptime_ms = GetU32(payload + 8);
  out->command_age_ms = GetU32(payload + 12);
  out->crc_errors = GetU32(payload + 16);
  out->format_errors = GetU32(payload + 20);
  out->rx_overflows = GetU32(payload + 24);
  out->tx_drops = GetU32(payload + 28);
  out->arm_token = GetU32(payload + 32);
  return true;
}

size_t EncodeFaultEvent(const FaultEvent& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeFaultEvent) {
    return 0;
  }
  PutU32(dst + 0, msg.fault_code);
  PutU32(dst + 4, msg.timestamp_ms);
  PutU32(dst + 8, msg.detail);
  return kSizeFaultEvent;
}

bool DecodeFaultEvent(const uint8_t* payload, size_t len, FaultEvent* out) {
  if (len != kSizeFaultEvent) {
    return false;
  }
  out->fault_code = GetU32(payload + 0);
  out->timestamp_ms = GetU32(payload + 4);
  out->detail = GetU32(payload + 8);
  return true;
}

}  // namespace uart
