#include "proto/codec.h"

#include <cmath>

#include "proto/detail/bytes.h"

namespace uart {
namespace {

bool ReadFinite(const uint8_t* src, float* out) {
  const float value = GetF32(src);
  if (!std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

RemoteState ToRemoteState(uint8_t value) {
  return value <= static_cast<uint8_t>(RemoteState::kFault) ? static_cast<RemoteState>(value)
                                                            : RemoteState::kDisabled;
}

AckResult ToAckResult(uint8_t value) {
  return value <= static_cast<uint8_t>(AckResult::kVersionMismatch)
             ? static_cast<AckResult>(value)
             : AckResult::kUnsupported;
}

MotionResultCode ToMotionResultCode(uint8_t value) {
  return value <= static_cast<uint8_t>(MotionResultCode::kImuUnavailable)
             ? static_cast<MotionResultCode>(value)
             : MotionResultCode::kInterrupted;
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
  if (!std::isfinite(msg.linear_x_mps) || !std::isfinite(msg.angular_z_radps)) {
    return 0;
  }
  PutF32(dst + 0, msg.linear_x_mps);
  PutF32(dst + 4, msg.angular_z_radps);
  return kSizeCmdVel;
}

bool DecodeCmdVel(const uint8_t* payload, size_t len, CmdVel* out) {
  if (len != kSizeCmdVel) {
    return false;
  }
  if (!ReadFinite(payload + 0, &out->linear_x_mps) ||
      !ReadFinite(payload + 4, &out->angular_z_radps)) {
    return false;
  }
  return true;
}

size_t EncodeMotionAction(const MotionAction& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeMotionAction) {
    return 0;
  }
  dst[0] = msg.action;
  dst[1] = msg.quarter_turns;
  PutU16(dst + 2, msg.speed_mmps);
  PutU32(dst + 4, msg.distance_mm);
  return kSizeMotionAction;
}

bool DecodeMotionAction(const uint8_t* payload, size_t len, MotionAction* out) {
  if (len != kSizeMotionAction) {
    return false;
  }
  out->action = payload[0];
  out->quarter_turns = payload[1];
  out->speed_mmps = GetU16(payload + 2);
  out->distance_mm = GetU32(payload + 4);
  return true;
}

size_t EncodeSpeakAudio(const SpeakAudio& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeSpeakAudio || msg.audio_id < 1U || msg.audio_id > 12U) {
    return 0;
  }
  dst[0] = msg.audio_id;
  return kSizeSpeakAudio;
}

bool DecodeSpeakAudio(const uint8_t* payload, size_t len, SpeakAudio* out) {
  if (len != kSizeSpeakAudio || payload[0] < 1U || payload[0] > 12U) {
    return false;
  }
  out->audio_id = payload[0];
  return true;
}

size_t EncodeAck(const Ack& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeAck) {
    return 0;
  }
  dst[0] = msg.request_type;
  dst[1] = static_cast<uint8_t>(msg.result);
  return kSizeAck;
}

bool DecodeAck(const uint8_t* payload, size_t len, Ack* out) {
  if (len != kSizeAck) {
    return false;
  }
  out->request_type = payload[0];
  out->result = ToAckResult(payload[1]);
  return true;
}

size_t EncodeHelloInfo(const HelloInfo& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeHelloInfo) {
    return 0;
  }
  dst[0] = msg.protocol_version;
  dst[1] = msg.capabilities;
  PutU32(dst + 2, msg.boot_id);
  dst[6] = static_cast<uint8_t>(msg.remote_state);
  return kSizeHelloInfo;
}

bool DecodeHelloInfo(const uint8_t* payload, size_t len, HelloInfo* out) {
  if (len != kSizeHelloInfo) {
    return false;
  }
  out->protocol_version = payload[0];
  out->capabilities = payload[1];
  out->boot_id = GetU32(payload + 2);
  out->remote_state = ToRemoteState(payload[6]);
  return true;
}

size_t EncodeOdomState(const OdomState& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeOdomState) {
    return 0;
  }
  PutF32(dst + 0, msg.x_m);
  PutF32(dst + 4, msg.y_m);
  PutF32(dst + 8, msg.yaw_rad);
  PutF32(dst + 12, msg.linear_mps);
  PutF32(dst + 16, msg.angular_radps);
  dst[20] = msg.status_flags;
  return kSizeOdomState;
}

bool DecodeOdomState(const uint8_t* payload, size_t len, OdomState* out) {
  if (len != kSizeOdomState) {
    return false;
  }
  if (!ReadFinite(payload + 0, &out->x_m) || !ReadFinite(payload + 4, &out->y_m) ||
      !ReadFinite(payload + 8, &out->yaw_rad) || !ReadFinite(payload + 12, &out->linear_mps) ||
      !ReadFinite(payload + 16, &out->angular_radps)) {
    return false;
  }
  out->status_flags = payload[20];
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
  PutF32(dst + 24, msg.yaw);
  dst[28] = msg.status_flags;
  return kSizeImuState;
}

bool DecodeImuState(const uint8_t* payload, size_t len, ImuState* out) {
  if (len != kSizeImuState) {
    return false;
  }
  if (!ReadFinite(payload + 0, &out->accel_x) || !ReadFinite(payload + 4, &out->accel_y) ||
      !ReadFinite(payload + 8, &out->accel_z) || !ReadFinite(payload + 12, &out->gyro_x) ||
      !ReadFinite(payload + 16, &out->gyro_y) || !ReadFinite(payload + 20, &out->gyro_z) ||
      !ReadFinite(payload + 24, &out->yaw)) {
    return false;
  }
  out->status_flags = payload[28];
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
  dst[20] = msg.status_flags;
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
  out->status_flags = payload[20];
  return true;
}

size_t EncodeSystemStatus(const SystemStatus& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeSystemStatus) {
    return 0;
  }
  dst[0] = static_cast<uint8_t>(msg.remote_state);
  dst[1] = msg.command_saturated;
  dst[2] = msg.fault_code;
  PutU32(dst + 3, msg.communication_errors);
  return kSizeSystemStatus;
}

bool DecodeSystemStatus(const uint8_t* payload, size_t len, SystemStatus* out) {
  if (len != kSizeSystemStatus) {
    return false;
  }
  out->remote_state = ToRemoteState(payload[0]);
  out->command_saturated = payload[1];
  out->fault_code = payload[2];
  out->communication_errors = GetU32(payload + 3);
  return true;
}

size_t EncodeMotionResult(const MotionResult& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeMotionResult) {
    return 0;
  }
  if (!std::isfinite(msg.final_yaw_rad) || !std::isfinite(msg.target_yaw_rad)) {
    return 0;
  }
  dst[0] = msg.action;
  dst[1] = msg.quarter_turns;
  dst[2] = static_cast<uint8_t>(msg.result);
  PutF32(dst + 3, msg.final_yaw_rad);
  PutF32(dst + 7, msg.target_yaw_rad);
  return kSizeMotionResult;
}

bool DecodeMotionResult(const uint8_t* payload, size_t len, MotionResult* out) {
  if (len != kSizeMotionResult) {
    return false;
  }
  if (!ReadFinite(payload + 3, &out->final_yaw_rad) ||
      !ReadFinite(payload + 7, &out->target_yaw_rad)) {
    return false;
  }
  out->action = payload[0];
  out->quarter_turns = payload[1];
  out->result = ToMotionResultCode(payload[2]);
  return true;
}

size_t EncodeRfidCard(const RfidCard& msg, uint8_t* dst, size_t cap) {
  if (cap < kSizeRfidCard) {
    return 0;
  }
  dst[0] = msg.present;
  dst[1] = msg.card_number;
  dst[2] = msg.generation;
  return kSizeRfidCard;
}

bool DecodeRfidCard(const uint8_t* payload, size_t len, RfidCard* out) {
  if (len != kSizeRfidCard) {
    return false;
  }
  out->present = payload[0];
  out->card_number = payload[1];
  out->generation = payload[2];
  return true;
}

}  // namespace uart
