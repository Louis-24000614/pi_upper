#include "link/sess.h"

#include <cmath>

namespace uart {
namespace {

/// 单次 Poll 从传输里读取的上限。921600 波特率下每毫秒约 92 字节，
/// 这个大小足够吸收调度抖动带来的积压，又不会让一次 Poll 占用太久。
constexpr size_t kReadChunk = 512;

/// 单次 Poll 处理的字节上限。读到返回 0 才算读空，但对端异常刷数据时不能一直读下去，
/// 否则 50 Hz 的发送节拍会被饿死。超出上限的字节留到下一次 Poll。
constexpr size_t kMaxBytesPerPoll = 8 * kReadChunk;

/// 时间同步样本的往返时延上限。超过这个值的样本通常是被调度延迟污染的，
/// 采纳它只会把偏移估计带偏。
constexpr uint64_t kMaxAcceptableRttUs = 20000;

}  // namespace

Session::Session(Transport& port, const Clock& clock, const SessionConfig& config)
    : port_(port), clock_(clock), config_(config) {}

void Session::Start() {
  rx_.Reset();
  link_state_ = port_.IsOpen() ? LinkState::kConnecting : LinkState::kClosed;
  remote_state_ = RemoteState::kDisconnected;
  boot_id_ = 0;
  has_boot_id_ = false;
  arm_token_ = 0;
  config_valid_ = false;
  target_linear_ = 0.0f;
  target_angular_ = 0.0f;
  has_target_ = false;
  telemetry_ = Telemetry{};
  time_sync_ = TimeSync{};

  const uint64_t now_ms = clock_.NowMs();
  last_rx_ms_ = now_ms;
  // 让首次 HELLO 立刻发出，而不是等一个重试周期。
  last_hello_ms_ = now_ms - config_.hello_retry_ms;
  last_cmd_ms_ = now_ms;
  last_sync_ms_ = now_ms;
}

bool Session::armed() const {
  return remote_state_ == RemoteState::kArmed && arm_token_ != 0;
}

bool Session::Send(MsgType type, const uint8_t* payload, size_t payload_len) {
  if (!port_.IsOpen()) {
    link_state_ = LinkState::kClosed;
    return false;
  }
  Header header;
  header.msg_type = static_cast<uint8_t>(type);
  header.sequence = next_sequence_++;
  header.timestamp_us = clock_.NowUs();

  uint8_t frame[kMaxEncodedSize] = {};
  const size_t len = EncodeFrame(header, payload, payload_len, frame, sizeof(frame));
  if (len == 0) {
    ++diagnostics_.tx_errors;
    return false;
  }
  if (port_.Write(frame, len) < 0) {
    ++diagnostics_.tx_errors;
    ++diagnostics_.link_drops;
    port_.Close();
    link_state_ = LinkState::kClosed;
    return false;
  }
  ++diagnostics_.tx_frames;
  return true;
}

bool Session::SendEmpty(MsgType type) { return Send(type, nullptr, 0); }

void Session::DropSession() {
  // token 绑定单次会话，链路一断就必须作废：协议规定重连不恢复旧 token。
  arm_token_ = 0;
  config_valid_ = false;
  remote_state_ = RemoteState::kDisconnected;
  has_boot_id_ = false;
  boot_id_ = 0;
  has_target_ = false;
  target_linear_ = 0.0f;
  target_angular_ = 0.0f;
  telemetry_.has_status = false;
  telemetry_.has_hello = false;
  rx_.Reset();
  time_sync_ = TimeSync{};
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

  // boot_id 变化意味着下位机复位或重新上电：旧 token 已失效，
  // 继续沿用会让上位机以为自己还在控制一台刚刚重启的车。
  if (has_boot_id_ && info.boot_id != boot_id_) {
    ++diagnostics_.boot_id_changes;
    arm_token_ = 0;
    has_target_ = false;
    target_linear_ = 0.0f;
    target_angular_ = 0.0f;
  }

  boot_id_ = info.boot_id;
  has_boot_id_ = true;
  config_valid_ = info.config_valid != 0;
  remote_state_ = info.remote_state;
  telemetry_.hello = info;
  telemetry_.has_hello = true;
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
  config_valid_ = status.config_valid != 0;
  // SYSTEM_STATUS 是 token 的唯一来源。非 ARMED 时它上报 0，正好用来清除本地 token。
  arm_token_ = status.remote_state == RemoteState::kArmed ? status.arm_token : 0;
}

void Session::OnTimeSyncResp(const uint8_t* payload, size_t len, uint64_t rx_us) {
  TimeSyncResp resp;
  if (!DecodeTimeSyncResp(payload, len, &resp)) {
    return;
  }
  // 四时间戳法：t1/t4 是本地时刻，t2/t3 是 MCU 时刻。t1 按纳秒发出，这里换回微秒。
  const uint64_t t1_us = resp.t1_host_ns / 1000;
  if (rx_us < t1_us) {
    return;  // 时钟异常，丢弃该样本。
  }
  const uint64_t rtt_us = (rx_us - t1_us) - (resp.t3_mcu_tx_us - resp.t2_mcu_rx_us);
  if (rtt_us > kMaxAcceptableRttUs) {
    return;  // 往返偏大的样本不采纳。
  }
  const int64_t offset_us =
      (static_cast<int64_t>(t1_us) - static_cast<int64_t>(resp.t2_mcu_rx_us) +
       static_cast<int64_t>(rx_us) - static_cast<int64_t>(resp.t3_mcu_tx_us)) /
      2;
  time_sync_.valid = true;
  time_sync_.offset_us = offset_us;
  time_sync_.rtt_us = rtt_us;
}

void Session::OnFrame(const Header& header, const uint8_t* payload, size_t len) {
  const uint64_t now_us = clock_.NowUs();
  last_rx_ms_ = now_us / 1000;

  // 主版本不兼容时只做诊断，不进入控制流程。帧层刻意不拦版本号就是为了留出这一步。
  if (header.version != kProtocolVersion) {
    return;
  }

  switch (static_cast<MsgType>(header.msg_type)) {
    case MsgType::kHelloInfo:
      OnHelloInfo(payload, len);
      break;
    case MsgType::kSystemStatus:
      OnSystemStatus(payload, len);
      break;
    case MsgType::kAck:
      if (DecodeAck(payload, len, &telemetry_.last_ack)) {
        telemetry_.has_ack = true;
      }
      break;
    case MsgType::kTimeSyncResp:
      OnTimeSyncResp(payload, len, now_us);
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
    case MsgType::kFaultEvent:
      if (DecodeFaultEvent(payload, len, &telemetry_.fault)) {
        telemetry_.fault_us = now_us;
        telemetry_.has_fault = true;
      }
      break;
    default:
      // 主机方向的消息或未知类型：忽略，不改变控制状态。
      break;
  }
}

void Session::SendCmdVel(float linear, float angular, uint64_t now_ms) {
  CmdVel cmd;
  cmd.arm_token = arm_token_;
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
  if (!armed()) {
    return;
  }
  if (now_ms - last_cmd_ms_ < config_.cmd_period_ms) {
    return;
  }

  // 上层指令过期就主动发零速。协议要求 50 Hz 持续发送，停发会触发下位机
  // 250 ms 看门狗刹车并锁存故障，所以这里发零速而不是干脆不发。
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
    return;
  }

  // 读到 Read 返回 0 才算读空。短读只说明"此刻可用这么多"，不代表后面没有了——
  // 拿短读当结束条件会在分片较小时丢掉同一帧的后续字节。
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
    rx_.Feed(buf, static_cast<size_t>(n),
             [this](const Header& header, const uint8_t* payload, size_t len) {
               OnFrame(header, payload, len);
             });
  }
  diagnostics_.rx = rx_.stats();

  const uint64_t now_ms = clock_.NowMs();

  // 长时间收不到任何有效帧：链路已不可信，丢掉会话重新建链。
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
      SendEmpty(MsgType::kHelloReq);
    }
    return;  // 建链完成前不发速度命令和时间同步。
  }

  if (config_.time_sync_period_ms != 0 && now_ms - last_sync_ms_ >= config_.time_sync_period_ms) {
    last_sync_ms_ = now_ms;
    TimeSyncReq req;
    // 协议规定该字段为纳秒，与 MCU 侧的微秒不同量纲，这里显式换算。
    req.t1_host_ns = clock_.NowUs() * 1000;
    uint8_t payload[kSizeTimeSyncReq] = {};
    if (EncodeTimeSyncReq(req, payload, sizeof(payload)) == kSizeTimeSyncReq) {
      Send(MsgType::kTimeSyncReq, payload, sizeof(payload));
    }
  }

  PumpCommand(now_ms);
}

bool Session::SetVelocity(float linear_x_mps, float angular_z_radps) {
  if (!std::isfinite(linear_x_mps) || !std::isfinite(angular_z_radps)) {
    ++diagnostics_.tx_errors;
    return false;
  }
  target_linear_ = linear_x_mps;
  target_angular_ = angular_z_radps;
  target_set_ms_ = clock_.NowMs();
  has_target_ = true;
  return true;
}

bool Session::RequestArm() {
  if (link_state_ != LinkState::kConnected || !has_boot_id_) {
    return false;
  }
  // config_valid=0 说明下位机的电机、编码器或底盘参数没绑定好。协议禁止绕过这个检查，
  // 重发 ARM 或伪造 token 都不行，所以这里直接不发。
  if (!config_valid_) {
    return false;
  }
  if (remote_state_ != RemoteState::kDisarmed) {
    return false;
  }

  ArmRequest req;
  req.boot_id = boot_id_;
  uint8_t payload[kSizeArmRequest] = {};
  if (EncodeArmRequest(req, payload, sizeof(payload)) != kSizeArmRequest) {
    return false;
  }
  if (!Send(MsgType::kArmRequest, payload, sizeof(payload))) {
    return false;
  }
  ++diagnostics_.arm_requests;
  return true;
}

bool Session::RequestDisarm() {
  // 先归零本地目标，避免 DISARM 之后还有一帧非零速度排在后面。
  target_linear_ = 0.0f;
  target_angular_ = 0.0f;
  has_target_ = false;
  return SendEmpty(MsgType::kDisarm);
}

bool Session::RequestResetOdom() {
  if (armed()) {
    return false;  // 下位机仅在非 ARMED 状态接受，这里提前拦掉省一次 ACK。
  }
  return SendEmpty(MsgType::kResetOdom);
}

bool Session::RequestClearFault() { return SendEmpty(MsgType::kClearFaultRequest); }

void Session::Shutdown() {
  if (!port_.IsOpen()) {
    return;
  }
  // 正常停车序列：先连发若干帧零速，再 DISARM。只在确实持有 token 时才需要零速，
  // 否则速度帧会被下位机以 BAD_TOKEN 拒掉，白占带宽。
  if (armed()) {
    const uint64_t now_ms = clock_.NowMs();
    for (uint32_t i = 0; i < config_.stop_zero_frames; ++i) {
      SendCmdVel(0.0f, 0.0f, now_ms);
    }
  }
  RequestDisarm();
}

}  // namespace uart
