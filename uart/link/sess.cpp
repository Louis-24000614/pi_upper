#include "link/sess.h"

#include <cmath>

namespace uart {
namespace {

constexpr size_t kReadChunk = 512;
constexpr size_t kMaxBytesPerPoll = 8 * kReadChunk;

}  // namespace

Session::Session(Transport& port, const Clock& clock, const SessionConfig& config)
    : port_(port), clock_(clock), config_(config) {}

bool Session::IsStop(uint8_t action) {
  return action == static_cast<uint8_t>(MotionActionId::kStop);
}

bool Session::IsFiniteMotion(uint8_t action, uint32_t distance_mm) {
  if (action == static_cast<uint8_t>(MotionActionId::kTurnLeft) ||
      action == static_cast<uint8_t>(MotionActionId::kTurnRight)) {
    return true;
  }
  if ((action == static_cast<uint8_t>(MotionActionId::kForward) ||
       action == static_cast<uint8_t>(MotionActionId::kBackward)) &&
      distance_mm != 0) {
    return true;
  }
  return false;
}

void Session::EnterIdleMotion() {
  motion_mode_ = MotionMode::kIdle;
  awaiting_motion_result_ = false;
  has_target_ = false;
  target_linear_ = 0.0f;
  target_angular_ = 0.0f;
}

bool Session::armed() const {
  return link_state_ == LinkState::kConnected && arm_ok_ && remote_state_ == RemoteState::kArmed;
}

void Session::Start() {
  rx_.Reset();
  link_state_ = port_.IsOpen() ? LinkState::kConnecting : LinkState::kClosed;
  remote_state_ = RemoteState::kDisabled;
  EnterIdleMotion();
  boot_id_ = 0;
  has_boot_id_ = false;
  arm_ok_ = false;
  config_valid_ = false;
  peer_protocol_version_ = 0;
  pending_request_type_ = 0;
  speech_pending_ = false;
  speech_blocked_ = false;
  heading_blocked_ = false;
  completions_.clear();
  telemetry_ = Telemetry{};

  const uint64_t now_ms = clock_.NowMs();
  last_rx_ms_ = now_ms;
  last_hello_ms_ = now_ms - config_.hello_retry_ms;
  last_cmd_ms_ = now_ms;
}

bool Session::Send(MsgType type, const uint8_t* payload, size_t payload_len) {
  if (!port_.IsOpen()) {
    link_state_ = LinkState::kClosed;
    DropSession();
    return false;
  }
  uint8_t frame[kMaxFrameSize] = {};
  const size_t len =
      EncodeFrame(static_cast<uint8_t>(type), payload, payload_len, frame, sizeof(frame));
  if (len == 0) {
    ++diagnostics_.tx_errors;
    return false;
  }
  if (port_.Write(frame, len) < 0) {
    ++diagnostics_.tx_errors;
    ++diagnostics_.link_drops;
    port_.Close();
    link_state_ = LinkState::kClosed;
    DropSession();
    return false;
  }
  ++diagnostics_.tx_frames;
  return true;
}

bool Session::SendEmpty(MsgType type) { return Send(type, nullptr, 0); }

bool Session::SendRequest(MsgType type, const uint8_t* payload, size_t payload_len) {
  if (pending_request_type_ != 0) {
    ++diagnostics_.requests_refused_busy;
    return false;
  }
  if (!Send(type, payload, payload_len)) {
    return false;
  }
  pending_request_type_ = static_cast<uint8_t>(type);
  pending_request_ms_ = clock_.NowMs();
  pending_request_serial_ = ++request_serial_;
  return true;
}

void Session::CompleteManagement(RequestCompletionCode code, AckResult result) {
  if (pending_request_type_ == 0) return;
  completions_.push_back({pending_request_serial_, boot_id_,
                         static_cast<MsgType>(pending_request_type_), code, result});
  if (pending_request_type_ == static_cast<uint8_t>(MsgType::kHeadingReference) &&
      code != RequestCompletionCode::kAck) heading_blocked_ = true;
  pending_request_type_ = 0;
}

void Session::CompleteSpeech(RequestCompletionCode code, AckResult result) {
  if (!speech_pending_) return;
  completions_.push_back({speech_request_serial_, boot_id_, MsgType::kSpeakAudio, code, result});
  speech_pending_ = false;
  if (code != RequestCompletionCode::kAck) speech_blocked_ = true;
}

bool Session::PopRequestCompletion(RequestCompletion* out) {
  if (out == nullptr || completions_.empty()) return false;
  *out = completions_.front();
  completions_.pop_front();
  return true;
}

void Session::OnAck(const uint8_t* payload, size_t len) {
  Ack ack;
  if (!DecodeAck(payload, len, &ack)) return;
  telemetry_.last_ack = ack;
  telemetry_.has_ack = true;
  if (ack.request_type == static_cast<uint8_t>(MsgType::kSpeakAudio)) {
    CompleteSpeech(RequestCompletionCode::kAck, ack.result);
    return;
  }
  if (ack.request_type != pending_request_type_) return;
  CompleteManagement(RequestCompletionCode::kAck, ack.result);
  if (ack.request_type == static_cast<uint8_t>(MsgType::kArmRequest)) {
    arm_ok_ = ack.result == AckResult::kOk;
    config_valid_ = ack.result == AckResult::kOk;
  }
}

void Session::OnMotionResult(const uint8_t* payload, size_t len) {
  MotionResult result;
  if (!DecodeMotionResult(payload, len, &result)) {
    return;
  }
  telemetry_.motion = result;
  telemetry_.motion_us = clock_.NowUs();
  telemetry_.has_motion = true;
  if (awaiting_motion_result_) {
    awaiting_motion_result_ = false;
    motion_mode_ = MotionMode::kIdle;
  }
}

void Session::DropSession() {
  CompleteManagement(RequestCompletionCode::kLinkLost);
  CompleteSpeech(RequestCompletionCode::kLinkLost);
  arm_ok_ = false;
  config_valid_ = false;
  remote_state_ = RemoteState::kDisabled;
  has_boot_id_ = false;
  boot_id_ = 0;
  peer_protocol_version_ = 0;
  pending_request_type_ = 0;
  EnterIdleMotion();
  telemetry_.has_status = false;
  telemetry_.has_hello = false;
  rx_.Reset();
  if (port_.IsOpen()) {
    link_state_ = LinkState::kConnecting;
  } else {
    link_state_ = LinkState::kClosed;
  }
}

void Session::OnHelloInfo(const uint8_t* payload, size_t len) {
  HelloInfo info;
  if (!DecodeHelloInfo(payload, len, &info)) {
    return;
  }

  const bool new_session = link_state_ != LinkState::kConnected || !has_boot_id_ ||
                           info.boot_id != boot_id_;
  if (has_boot_id_ && info.boot_id != boot_id_) {
    ++diagnostics_.boot_id_changes;
    CompleteManagement(RequestCompletionCode::kLinkLost);
    CompleteSpeech(RequestCompletionCode::kLinkLost);
    arm_ok_ = false;
    config_valid_ = false;
    EnterIdleMotion();
  }

  peer_protocol_version_ = info.protocol_version;
  if (info.protocol_version != kProtocolVersion) {
    ++diagnostics_.version_mismatches;
    telemetry_.hello = info;
    telemetry_.has_hello = true;
    return;
  }

  boot_id_ = info.boot_id;
  has_boot_id_ = true;
  remote_state_ = info.remote_state;
  telemetry_.hello = info;
  telemetry_.has_hello = true;
  if (new_session) {
    speech_blocked_ = false;
    heading_blocked_ = false;
  }
  link_state_ = LinkState::kConnected;
}

void Session::OnSystemStatus(const uint8_t* payload, size_t len) {
  SystemStatus status;
  if (!DecodeSystemStatus(payload, len, &status)) {
    return;
  }
  telemetry_.status = status;
  telemetry_.status_us = clock_.NowUs();
  telemetry_.has_status = true;

  remote_state_ = status.remote_state;
  if (remote_state_ != RemoteState::kArmed) {
    arm_ok_ = false;
    if (motion_mode_ == MotionMode::kVelocity) {
      EnterIdleMotion();
    }
  }
}

void Session::OnFrame(uint8_t msg_type, const uint8_t* payload, size_t len) {
  const uint64_t now_us = clock_.NowUs();
  last_rx_ms_ = now_us / 1000;

  switch (static_cast<MsgType>(msg_type)) {
    case MsgType::kHelloInfo:
      OnHelloInfo(payload, len);
      break;
    case MsgType::kSystemStatus:
      OnSystemStatus(payload, len);
      break;
    case MsgType::kAck:
      OnAck(payload, len);
      break;
    case MsgType::kMotionResult:
      OnMotionResult(payload, len);
      break;
    case MsgType::kOdomState:
      if (DecodeOdomState(payload, len, &telemetry_.odom)) {
        telemetry_.odom_us = now_us;
        telemetry_.has_odom = true;
      }
      break;
    case MsgType::kImuState:
      if (DecodeImuState(payload, len, &telemetry_.imu)) {
        telemetry_.imu_us = now_us;
        telemetry_.has_imu = true;
      }
      break;
    case MsgType::kImuDebug:
      if (DecodeImuDebug(payload, len, &telemetry_.imu_debug)) {
        telemetry_.imu_debug_us = now_us;
        telemetry_.has_imu_debug = true;
      }
      break;
    case MsgType::kRfidCard:
      if (DecodeRfidCard(payload, len, &telemetry_.rfid)) {
        telemetry_.rfid_us = now_us;
        telemetry_.has_rfid = true;
      }
      break;
    default:
      break;
  }
}

void Session::SendCmdVel(float linear, float angular, uint64_t now_ms) {
  CmdVel cmd;
  cmd.linear_x_mps = linear;
  cmd.angular_z_radps = angular;

  uint8_t payload[kSizeCmdVel] = {};
  if (EncodeCmdVel(cmd, payload, sizeof(payload)) != kSizeCmdVel) {
    ++diagnostics_.tx_errors;
    return;
  }
  if (Send(MsgType::kCmdVel, payload, sizeof(payload))) {
    ++diagnostics_.cmd_frames;
    last_cmd_ms_ = now_ms;
  }
}

void Session::PumpCommand(uint64_t now_ms) {
  if (!armed() || motion_mode_ != MotionMode::kVelocity) {
    return;
  }
  if (now_ms - last_cmd_ms_ < config_.cmd_period_ms) {
    return;
  }

  const bool fresh = has_target_ && (now_ms - target_set_ms_) <= config_.cmd_validity_ms;
  if (fresh) {
    SendCmdVel(target_linear_, target_angular_, now_ms);
  } else {
    if (has_target_) {
      ++diagnostics_.zero_substitutions;
      has_target_ = false;
      target_linear_ = 0.0f;
      target_angular_ = 0.0f;
    }
    SendCmdVel(0.0f, 0.0f, now_ms);
  }
}

void Session::Poll() {
  if (!port_.IsOpen()) {
    if (link_state_ != LinkState::kClosed) {
      ++diagnostics_.link_drops;
      link_state_ = LinkState::kClosed;
    }
    DropSession();
    return;
  }

  size_t consumed = 0;
  while (consumed < kMaxBytesPerPoll) {
    uint8_t buf[kReadChunk];
    const int n = port_.Read(buf, sizeof(buf));
    if (n < 0) {
      ++diagnostics_.link_drops;
      port_.Close();
      link_state_ = LinkState::kClosed;
      DropSession();
      return;
    }
    if (n == 0) {
      break;
    }
    consumed += static_cast<size_t>(n);
    rx_.Feed(buf, static_cast<size_t>(n), clock_.NowUs(),
             [this](uint8_t msg_type, const uint8_t* payload, size_t len) {
               OnFrame(msg_type, payload, len);
             });
  }
  diagnostics_.rx = rx_.stats();

  const uint64_t now_ms = clock_.NowMs();

  if (pending_request_type_ != 0 && now_ms - pending_request_ms_ > config_.ack_timeout_ms) {
    ++diagnostics_.ack_timeouts;
    CompleteManagement(RequestCompletionCode::kTimeout);
  }
  if (speech_pending_ && now_ms - speech_request_ms_ > config_.ack_timeout_ms) {
    ++diagnostics_.ack_timeouts;
    CompleteSpeech(RequestCompletionCode::kTimeout);
  }

  if (link_state_ == LinkState::kConnected && now_ms - last_rx_ms_ > config_.link_timeout_ms) {
    ++diagnostics_.link_drops;
    DropSession();
    last_rx_ms_ = now_ms;
    last_hello_ms_ = now_ms - config_.hello_retry_ms;
  }

  if (link_state_ == LinkState::kConnecting) {
    if (now_ms - last_hello_ms_ >= config_.hello_retry_ms) {
      last_hello_ms_ = now_ms;
      ++diagnostics_.hello_sent;
      uint8_t payload[kSizeHelloReq] = {};
      if (EncodeHelloReq(HelloReq{}, payload, sizeof(payload)) == kSizeHelloReq) {
        Send(MsgType::kHelloReq, payload, sizeof(payload));
      }
    }
    return;
  }

  PumpCommand(now_ms);
}

bool Session::SetVelocity(float linear_x_mps, float angular_z_radps) {
  if (!std::isfinite(linear_x_mps) || !std::isfinite(angular_z_radps)) {
    ++diagnostics_.tx_errors;
    return false;
  }
  if ((pending_request_type_ == static_cast<uint8_t>(MsgType::kHeadingReference) &&
       (linear_x_mps != 0.0f || angular_z_radps != 0.0f)) ||
      awaiting_motion_result_ || motion_mode_ == MotionMode::kAction) {
    return false;
  }
  target_linear_ = linear_x_mps;
  target_angular_ = angular_z_radps;
  target_set_ms_ = clock_.NowMs();
  has_target_ = true;
  motion_mode_ = MotionMode::kVelocity;
  return true;
}

bool Session::RequestArm() {
  if (link_state_ != LinkState::kConnected || !has_boot_id_) {
    return false;
  }
  if (remote_state_ == RemoteState::kFault || remote_state_ == RemoteState::kDisabled) {
    return false;
  }

  ArmRequest req;
  req.boot_id = boot_id_;
  uint8_t payload[kSizeArmRequest] = {};
  if (EncodeArmRequest(req, payload, sizeof(payload)) != kSizeArmRequest) {
    return false;
  }
  if (!SendRequest(MsgType::kArmRequest, payload, sizeof(payload))) {
    return false;
  }
  ++diagnostics_.arm_requests;
  return true;
}

bool Session::RequestDisarm() {
  if (pending_request_type_ == static_cast<uint8_t>(MsgType::kHeadingReference))
    CompleteManagement(RequestCompletionCode::kCancelled);
  EnterIdleMotion();
  arm_ok_ = false;
  return SendEmpty(MsgType::kDisarm);
}

bool Session::RequestMotionAction(uint8_t action, uint8_t quarter_turns, uint16_t speed_mmps,
                                  uint32_t distance_mm) {
  if (!armed()) {
    return false;
  }

  const bool stop = IsStop(action);
  const bool finite = IsFiniteMotion(action, distance_mm);
  if (!stop && awaiting_motion_result_) {
    return false;
  }

  MotionAction msg;
  msg.action = action;
  msg.quarter_turns = quarter_turns;
  msg.speed_mmps = speed_mmps;
  msg.distance_mm = distance_mm;
  uint8_t payload[kSizeMotionAction] = {};
  if (EncodeMotionAction(msg, payload, sizeof(payload)) != kSizeMotionAction) {
    return false;
  }

  if (stop) {
    if (pending_request_type_ == static_cast<uint8_t>(MsgType::kHeadingReference))
      CompleteManagement(RequestCompletionCode::kCancelled);
    has_target_ = false;
    target_linear_ = 0.0f;
    target_angular_ = 0.0f;
    motion_mode_ = MotionMode::kIdle;
    awaiting_motion_result_ = false;
    if (!Send(MsgType::kMotionAction, payload, sizeof(payload))) {
      return false;
    }
    ++diagnostics_.action_frames;
    return true;
  }

  if (!SendRequest(MsgType::kMotionAction, payload, sizeof(payload))) {
    return false;
  }
  ++diagnostics_.action_frames;
  has_target_ = false;
  target_linear_ = 0.0f;
  target_angular_ = 0.0f;
  motion_mode_ = MotionMode::kAction;
  awaiting_motion_result_ = finite;
  return true;
}

bool Session::RequestSpeech(uint8_t speech_id) {
  if (link_state_ != LinkState::kConnected || speech_id < 1U || speech_id > 32U ||
      speech_pending_ || speech_blocked_) return false;
  SpeakAudio msg;
  msg.audio_id = speech_id;
  uint8_t payload[kSizeSpeakAudio] = {};
  if (EncodeSpeakAudio(msg, payload, sizeof(payload)) != kSizeSpeakAudio ||
      !Send(MsgType::kSpeakAudio, payload, sizeof(payload))) return false;
  speech_pending_ = true;
  speech_request_ms_ = clock_.NowMs();
  speech_request_serial_ = ++request_serial_;
  return true;
}

bool Session::supports_heading_reference() const {
  return link_state_ == LinkState::kConnected && telemetry_.has_hello &&
         (telemetry_.hello.capabilities & kCapHeadingReference) != 0;
}

bool Session::RequestHeadingReference() {
  if (!supports_heading_reference() || heading_blocked_ || awaiting_motion_result_ ||
      motion_mode_ == MotionMode::kAction ||
      (has_target_ && (target_linear_ != 0.0f || target_angular_ != 0.0f))) return false;
  return SendRequest(MsgType::kHeadingReference, nullptr, 0);
}

void Session::Shutdown() {
  if (!port_.IsOpen()) {
    return;
  }
  if (armed() && motion_mode_ == MotionMode::kVelocity) {
    const uint64_t now_ms = clock_.NowMs();
    for (uint32_t i = 0; i < config_.stop_zero_frames; ++i) {
      SendCmdVel(0.0f, 0.0f, now_ms);
    }
  } else if (armed()) {
    MotionAction stop;
    uint8_t payload[kSizeMotionAction] = {};
    if (EncodeMotionAction(stop, payload, sizeof(payload)) == kSizeMotionAction) {
      Send(MsgType::kMotionAction, payload, sizeof(payload));
    }
  }
  RequestDisarm();
}

}  // namespace uart
