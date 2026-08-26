/// @file
/// 会话层测试。全部用假传输加假时钟驱动，不碰硬件、不 sleep。
///
/// 这一层的 bug 都是安全性质的：绕过 config_valid 去 ARM、下位机复位后继续用旧
/// token、指令过期还在保持旧速度。每条测试对应协议里一条明确要求。

#include "link/sess.h"

#include <cmath>
#include <limits>

#include "check.h"
#include "mock/fake.h"
#include "mock/mcu.h"

namespace {

using namespace uart;        // NOLINT(build/namespaces)
using namespace uart::test;  // NOLINT(build/namespaces)

constexpr uint32_t kBootId = 0xAABBCCDD;
constexpr uint32_t kToken = 0x11223344;

/// 一套完整的测试环境：假串口、假时钟、假下位机和被测会话。
struct Fixture {
  FakePort port;
  FakeClock clock;
  FakeMcu mcu;
  Session session{port, clock};

  /// 推进时间并轮询若干次，模拟通信线程的循环。
  void Tick(uint64_t ms) {
    clock.AdvanceMs(ms);
    session.Poll();
    mcu.Drain(port);
  }

  /// 走完建链：上位机发 HELLO_REQ，下位机回 HELLO_INFO。
  void Connect(uint8_t config_valid = 1, uint32_t boot_id = kBootId,
               RemoteState state = RemoteState::kDisarmed) {
    session.Start();
    Tick(1);
    HelloInfo info;
    info.protocol_version = kProtocolVersion;
    info.boot_id = boot_id;
    info.config_valid = config_valid;
    info.remote_state = state;
    info.capabilities = kCapMotor | kCapEncoder | kCapImu;
    mcu.SendHelloInfo(port, info);
    Tick(1);
  }

  /// 让下位机报告已使能并带上 token。
  void Arm(uint32_t token = kToken) {
    SystemStatus status;
    status.remote_state = RemoteState::kArmed;
    status.config_valid = 1;
    status.arm_token = token;
    mcu.SendSystemStatus(port, status);
    Tick(1);
  }
};

/// 建链前应重复发 HELLO_REQ，收到 HELLO_INFO 后停止并进入已连接。
void TestHelloRetryUntilAnswered() {
  Fixture f;
  f.session.Start();
  CHECK(f.session.link_state() == LinkState::kConnecting);

  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kHelloReq) == 1);

  // 未收到回复时按重试周期继续发。
  f.Tick(500);
  f.Tick(500);
  CHECK(f.mcu.CountOf(MsgType::kHelloReq) == 3);
  CHECK(f.session.link_state() == LinkState::kConnecting);

  HelloInfo info;
  info.protocol_version = kProtocolVersion;
  info.boot_id = kBootId;
  info.config_valid = 1;
  info.remote_state = RemoteState::kDisarmed;
  f.mcu.SendHelloInfo(f.port, info);
  f.Tick(1);

  CHECK(f.session.link_state() == LinkState::kConnected);
  CHECK(f.session.boot_id() == kBootId);
  CHECK(f.session.config_valid());
  CHECK(f.session.remote_state() == RemoteState::kDisarmed);

  // 上位机发出的帧本身必须合法。
  CHECK(f.mcu.stats().crc_errors == 0);
  CHECK(f.mcu.stats().overflows == 0);
}

/// config_valid=0 时不得发起 ARM。协议规定这是安全检查，不允许绕过。
void TestArmRefusedWhenConfigInvalid() {
  Fixture f;
  f.Connect(/*config_valid=*/0);
  CHECK(!f.session.config_valid());

  CHECK(!f.session.RequestArm());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kArmRequest) == 0);
  CHECK(f.session.diagnostics().arm_requests == 0);
}

/// 未建链时也不得发 ARM。
void TestArmRefusedBeforeConnect() {
  Fixture f;
  f.session.Start();
  CHECK(!f.session.RequestArm());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kArmRequest) == 0);
}

/// ARM_REQUEST 必须带本次会话的 boot_id，否则下位机会以 BAD_TOKEN 拒绝。
void TestArmRequestCarriesBootId() {
  Fixture f;
  f.Connect();
  CHECK(f.session.RequestArm());
  f.Tick(1);

  const FakeMcu::Received* frame = f.mcu.Last(MsgType::kArmRequest);
  CHECK(frame != nullptr);
  if (frame != nullptr) {
    ArmRequest req;
    CHECK(DecodeArmRequest(frame->payload.data(), frame->payload.size(), &req));
    CHECK(req.boot_id == kBootId);
  }
}

/// 未持有 token 时绝不能发速度命令。
void TestNoCommandsBeforeArmed() {
  Fixture f;
  f.Connect();
  f.session.SetVelocity(0.5f, 0.1f);
  f.Tick(200);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);
  CHECK(f.session.arm_token() == 0);
}

/// 拿到 token 后应以 50 Hz 稳定下发，且 token 与速度都正确。
void TestCommandRateAfterArm() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.arm_token() == kToken);
  CHECK(f.session.remote_state() == RemoteState::kArmed);

  f.mcu.ClearReceived();
  f.session.SetVelocity(0.5f, -0.25f);

  // 100 ms 内按 20 ms 周期应发出 5 帧。
  for (int i = 0; i < 100; ++i) {
    f.Tick(1);
  }
  const size_t count = f.mcu.CountOf(MsgType::kCmdVel);
  CHECK(count == 5);

  const FakeMcu::Received* frame = f.mcu.Last(MsgType::kCmdVel);
  CHECK(frame != nullptr);
  if (frame != nullptr) {
    CmdVel cmd;
    CHECK(DecodeCmdVel(frame->payload.data(), frame->payload.size(), &cmd));
    CHECK(cmd.arm_token == kToken);
    CHECK(cmd.linear_x_mps == 0.5f);
    CHECK(cmd.angular_z_radps == -0.25f);
  }
}

/// 上层指令过期后必须改发零速，而不是继续保持旧速度，也不是停发。
/// 停发会触发下位机 250 ms 看门狗刹车并锁存故障。
void TestStaleCommandBecomesZero() {
  Fixture f;
  f.Connect();
  f.Arm();
  f.session.SetVelocity(1.0f, 0.5f);
  f.mcu.ClearReceived();

  // 有效期内仍是原速度。
  for (int i = 0; i < 100; ++i) {
    f.Tick(1);
  }
  const FakeMcu::Received* fresh = f.mcu.Last(MsgType::kCmdVel);
  CHECK(fresh != nullptr);
  if (fresh != nullptr) {
    CmdVel cmd;
    CHECK(DecodeCmdVel(fresh->payload.data(), fresh->payload.size(), &cmd));
    CHECK(cmd.linear_x_mps == 1.0f);
  }

  // 超过 200 ms 有效期后改发零速，但帧照发。
  f.mcu.ClearReceived();
  for (int i = 0; i < 200; ++i) {
    f.Tick(1);
  }
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 10);
  const FakeMcu::Received* stale = f.mcu.Last(MsgType::kCmdVel);
  CHECK(stale != nullptr);
  if (stale != nullptr) {
    CmdVel cmd;
    CHECK(DecodeCmdVel(stale->payload.data(), stale->payload.size(), &cmd));
    CHECK(cmd.linear_x_mps == 0.0f);
    CHECK(cmd.angular_z_radps == 0.0f);
  }
  CHECK(f.session.diagnostics().zero_substitutions == 1);
}

/// 非有限速度必须在写入时就被拒绝，不能进入下发路径。
void TestNonFiniteVelocityRejected() {
  Fixture f;
  f.Connect();
  f.Arm();

  CHECK(!f.session.SetVelocity(std::numeric_limits<float>::quiet_NaN(), 0.0f));
  CHECK(!f.session.SetVelocity(0.0f, std::numeric_limits<float>::infinity()));

  f.mcu.ClearReceived();
  f.Tick(25);
  // 没有有效目标，发出的应是零速。
  const FakeMcu::Received* frame = f.mcu.Last(MsgType::kCmdVel);
  CHECK(frame != nullptr);
  if (frame != nullptr) {
    CmdVel cmd;
    CHECK(DecodeCmdVel(frame->payload.data(), frame->payload.size(), &cmd));
    CHECK(cmd.linear_x_mps == 0.0f);
  }
}

/// 下位机复位（boot_id 变化）后必须丢弃旧 token 并停止下发。
void TestBootIdChangeDropsToken() {
  Fixture f;
  f.Connect();
  f.Arm();
  f.session.SetVelocity(0.5f, 0.0f);
  f.Tick(25);
  CHECK(f.session.arm_token() == kToken);

  // 复位后下位机以新 boot_id 回 HELLO_INFO。
  HelloInfo info;
  info.protocol_version = kProtocolVersion;
  info.boot_id = 0x99999999;
  info.config_valid = 1;
  info.remote_state = RemoteState::kDisarmed;
  f.mcu.SendHelloInfo(f.port, info);
  f.Tick(1);

  CHECK(f.session.boot_id() == 0x99999999);
  CHECK(f.session.arm_token() == 0);
  CHECK(f.session.diagnostics().boot_id_changes == 1);

  f.mcu.ClearReceived();
  f.Tick(100);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);
}

/// 下位机报告非 ARMED 时 token 上报 0，本地必须同步清除。
void TestTokenClearedWhenNotArmed() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.arm_token() == kToken);

  SystemStatus status;
  status.remote_state = RemoteState::kFault;
  status.config_valid = 1;
  status.fault_code = 0x04;
  status.arm_token = 0;
  f.mcu.SendSystemStatus(f.port, status);
  f.Tick(1);

  CHECK(f.session.arm_token() == 0);
  CHECK(f.session.remote_state() == RemoteState::kFault);

  f.mcu.ClearReceived();
  f.Tick(100);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);
}

/// 长时间收不到任何有效帧应掉线、清 token 并重新开始建链。
void TestLinkTimeoutDropsSession() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.link_state() == LinkState::kConnected);

  f.mcu.ClearReceived();
  // 静默超过 1 s。
  for (int i = 0; i < 1200; ++i) {
    f.Tick(1);
  }
  CHECK(f.session.link_state() == LinkState::kConnecting);
  CHECK(f.session.arm_token() == 0);
  CHECK(f.session.diagnostics().link_drops >= 1);
  CHECK(f.mcu.CountOf(MsgType::kHelloReq) >= 1);
}

/// 传输报错应关闭链路并清掉会话状态，而不是继续假装在控制。
void TestTransportErrorDropsSession() {
  Fixture f;
  f.Connect();
  f.Arm();

  f.port.set_fail_read(true);
  f.Tick(1);
  CHECK(f.session.link_state() == LinkState::kClosed);
  CHECK(f.session.arm_token() == 0);
  CHECK(!f.port.IsOpen());
}

/// 退出序列：先零速若干帧，最后一帧是 DISARM。
void TestShutdownSendsZeroThenDisarm() {
  Fixture f;
  f.Connect();
  f.Arm();
  f.session.SetVelocity(1.0f, 1.0f);
  f.Tick(25);

  f.mcu.ClearReceived();
  f.session.Shutdown();
  f.mcu.Drain(f.port);

  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 3);
  CHECK(f.mcu.CountOf(MsgType::kDisarm) == 1);
  // 顺序要求：DISARM 必须在所有零速之后。
  CHECK(!f.mcu.received().empty());
  if (!f.mcu.received().empty()) {
    CHECK(f.mcu.received().back().type == static_cast<uint8_t>(MsgType::kDisarm));
  }
  for (const FakeMcu::Received& frame : f.mcu.received()) {
    if (frame.type == static_cast<uint8_t>(MsgType::kCmdVel)) {
      CmdVel cmd;
      CHECK(DecodeCmdVel(frame.payload.data(), frame.payload.size(), &cmd));
      CHECK(cmd.linear_x_mps == 0.0f);
      CHECK(cmd.angular_z_radps == 0.0f);
    }
  }
}

/// ARMED 状态下不接受 RESET_ODOM，提前拦掉省一次必然失败的往返。
void TestResetOdomRefusedWhileArmed() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(!f.session.RequestResetOdom());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kResetOdom) == 0);

  SystemStatus status;
  status.remote_state = RemoteState::kDisarmed;
  status.config_valid = 1;
  f.mcu.SendSystemStatus(f.port, status);
  f.Tick(1);
  CHECK(f.session.RequestResetOdom());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kResetOdom) == 1);
}

/// 遥测应被正确解析并带上接收时刻。
void TestTelemetryCaptured() {
  Fixture f;
  f.Connect();

  OdomState odom;
  odom.x_m = 1.5f;
  odom.yaw_rad = 0.25f;
  odom.left_ticks = 1000;
  odom.status_flags = kOdomValid | kOdomImuFused;
  f.mcu.SendOdom(f.port, odom);

  FaultEvent fault;
  fault.fault_code = 0x08;
  fault.detail = 42;
  f.mcu.SendFault(f.port, fault);
  f.Tick(1);

  CHECK(f.session.telemetry().has_odom);
  CHECK(f.session.telemetry().odom.x_m == 1.5f);
  CHECK(f.session.telemetry().odom.left_ticks == 1000);
  CHECK((f.session.telemetry().odom.status_flags & kOdomValid) != 0);
  CHECK(f.session.telemetry().odom_us > 0);

  CHECK(f.session.telemetry().has_fault);
  CHECK(f.session.telemetry().fault.fault_code == 0x08);
  CHECK(f.session.telemetry().fault.detail == 42);
}

/// 时间同步：偏移与往返时延应被正确估出。
void TestTimeSyncEstimatesOffset() {
  Fixture f;
  f.Connect();
  f.mcu.ClearReceived();

  // 触发一次 TIME_SYNC_REQ。
  f.Tick(1000);
  const FakeMcu::Received* req = f.mcu.Last(MsgType::kTimeSyncReq);
  CHECK(req != nullptr);
  if (req == nullptr) {
    return;
  }
  TimeSyncReq decoded;
  CHECK(DecodeTimeSyncReq(req->payload.data(), req->payload.size(), &decoded));

  // 构造一个 MCU 时钟比本地慢 500000 µs 的回复，处理耗时 200 µs。
  const uint64_t t1_us = decoded.t1_host_ns / 1000;
  TimeSyncResp resp;
  resp.t1_host_ns = decoded.t1_host_ns;
  resp.t2_mcu_rx_us = t1_us - 500000;
  resp.t3_mcu_tx_us = resp.t2_mcu_rx_us + 200;
  f.mcu.SendTimeSyncResp(f.port, resp);

  // 回复在 1 ms 后到达。
  f.Tick(1);

  CHECK(f.session.time_sync().valid);
  // 偏移应接近 +500000 µs（MCU 时钟加上偏移得到本地时钟）。
  const int64_t offset = f.session.time_sync().offset_us;
  CHECK(offset > 499000 && offset < 501000);
  CHECK(f.session.time_sync().rtt_us < 2000);
}

/// 往返时延过大的样本必须被丢弃，否则偏移估计会被调度延迟带偏。
void TestTimeSyncRejectsHighRtt() {
  Fixture f;
  f.Connect();
  f.mcu.ClearReceived();
  f.Tick(1000);
  const FakeMcu::Received* req = f.mcu.Last(MsgType::kTimeSyncReq);
  CHECK(req != nullptr);
  if (req == nullptr) {
    return;
  }
  TimeSyncReq decoded;
  CHECK(DecodeTimeSyncReq(req->payload.data(), req->payload.size(), &decoded));

  TimeSyncResp resp;
  resp.t1_host_ns = decoded.t1_host_ns;
  resp.t2_mcu_rx_us = decoded.t1_host_ns / 1000;
  resp.t3_mcu_tx_us = resp.t2_mcu_rx_us + 100;
  f.mcu.SendTimeSyncResp(f.port, resp);

  // 回复晚了 100 ms 才到，远超可接受的往返时延。
  f.Tick(100);
  CHECK(!f.session.time_sync().valid);
}

/// 协议主版本不兼容时不得进入控制流程：帧能收到，但不认为建链成功。
void TestVersionMismatchDoesNotConnect() {
  Fixture f;
  f.session.Start();
  f.Tick(1);

  HelloInfo info;
  info.boot_id = kBootId;
  info.config_valid = 1;
  info.remote_state = RemoteState::kDisarmed;
  // 下位机报了一个和本端不一致的版本。v2 把版本号放在 HELLO_INFO 的 payload 里，
  // 所以这一帧本身是合法的，能被解出来，只是不该让上位机认为建链成功。
  f.mcu.SendHelloInfoWithVersion(f.port, info, kProtocolVersion + 1);
  f.Tick(1);

  CHECK(f.session.link_state() == LinkState::kConnecting);
  CHECK(f.session.boot_id() == 0);
  CHECK(!f.session.RequestArm());
  CHECK(f.session.peer_protocol_version() == kProtocolVersion + 1);
  CHECK(f.session.diagnostics().version_mismatches == 1);
}

/// 坏帧不能影响会话状态，且必须计入诊断。
void TestBadFrameDoesNotDisturbSession() {
  Fixture f;
  f.Connect();
  f.Arm();
  const uint32_t token_before = f.session.arm_token();

  // 文档第 9 节的坏 CRC 黄金帧：token 首字节改成 0x79 但保留原 CRC 0xCD。
  const std::vector<uint8_t> bad = {0x55, 0xAA, 0x12, 0x0C, 0x79, 0x56, 0x34, 0x12, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD};
  f.mcu.SendRaw(f.port, bad);
  f.Tick(1);

  CHECK(f.session.arm_token() == token_before);
  CHECK(f.session.link_state() == LinkState::kConnected);
  CHECK(f.session.diagnostics().rx.crc_errors == 1);
}

/// 串口把帧切成任意分片时会话行为不变。
void TestWorksWithByteAtATimeReads() {
  Fixture f;
  f.port.set_read_chunk(1);
  f.Connect();
  CHECK(f.session.link_state() == LinkState::kConnected);
  f.Arm();
  CHECK(f.session.arm_token() == kToken);
}

/// v2 没有序号，ACK 只能按类型配对，所以同一时刻只允许一个在途管理请求。
/// 第二个请求必须被拒，否则收到 ACK 时无法判断它回应的是哪一次。
void TestOneOutstandingRequestAtATime() {
  Fixture f;
  f.Connect();

  CHECK(f.session.RequestClearFault());
  CHECK(f.session.request_pending());
  // 在途期间的第二个请求被拒，且没有真的发上线。
  f.mcu.ClearReceived();
  CHECK(!f.session.RequestResetOdom());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kResetOdom) == 0);
  CHECK(f.session.diagnostics().requests_refused_busy == 1);

  // 收到对应类型的 ACK 后名额释放。
  Ack ack;
  ack.request_type = static_cast<uint8_t>(MsgType::kClearFaultRequest);
  ack.result = AckResult::kPending;
  f.mcu.SendAck(f.port, ack);
  f.Tick(1);
  CHECK(!f.session.request_pending());
  CHECK(f.session.RequestResetOdom());
}

/// 类型对不上的 ACK 不能把在途请求误清掉。CMD_VEL 出错时也会回 ACK，
/// 它不占名额，更不该顶掉正在等的管理请求。
void TestMismatchedAckDoesNotClearPending() {
  Fixture f;
  f.Connect();

  CHECK(f.session.RequestClearFault());
  Ack ack;
  ack.request_type = static_cast<uint8_t>(MsgType::kCmdVel);
  ack.result = AckResult::kBadToken;
  f.mcu.SendAck(f.port, ack);
  f.Tick(1);

  CHECK(f.session.request_pending());
  CHECK(f.session.telemetry().has_ack);
}

/// ACK 迟迟不来时要超时放开名额，让上层能重试；但协议要求上位机自己不重发，
/// 所以超时只清标志，不产生新的请求帧。
void TestPendingRequestTimesOut() {
  Fixture f;
  f.Connect();

  CHECK(f.session.RequestClearFault());
  // 先把请求帧收掉再清计数，这样后面统计到的就只有超时之后新发的帧。
  f.Tick(1);
  f.mcu.ClearReceived();

  f.Tick(f.session.config().ack_timeout_ms + 1);
  CHECK(!f.session.request_pending());
  CHECK(f.session.diagnostics().ack_timeouts == 1);
  CHECK(f.mcu.CountOf(MsgType::kClearFaultRequest) == 0);
}

/// DISARM 是停车动作，安全优先：即使有在途管理请求也必须能发出去。
void TestDisarmIsNotBlockedByPendingRequest() {
  Fixture f;
  f.Connect();

  CHECK(f.session.RequestClearFault());
  CHECK(f.session.request_pending());
  f.mcu.ClearReceived();

  CHECK(f.session.RequestDisarm());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kDisarm) == 1);
}

/// HELLO_REQ 在 v2 里带一个字节的协议版本，且因为幂等、由 HELLO_INFO 回应，
/// 不占在途请求名额，可以按重试周期一直发。
void TestHelloReqCarriesVersion() {
  Fixture f;
  f.session.Start();
  f.Tick(1);

  const FakeMcu::Received* hello = f.mcu.Last(MsgType::kHelloReq);
  CHECK(hello != nullptr);
  if (hello != nullptr) {
    CHECK(hello->payload.size() == kSizeHelloReq);
    if (hello->payload.size() == kSizeHelloReq) {
      CHECK(hello->payload[0] == kProtocolVersion);
    }
  }
  CHECK(!f.session.request_pending());
}

}  // namespace

int main() {
  TestHelloRetryUntilAnswered();
  TestArmRefusedWhenConfigInvalid();
  TestArmRefusedBeforeConnect();
  TestArmRequestCarriesBootId();
  TestNoCommandsBeforeArmed();
  TestCommandRateAfterArm();
  TestStaleCommandBecomesZero();
  TestNonFiniteVelocityRejected();
  TestBootIdChangeDropsToken();
  TestTokenClearedWhenNotArmed();
  TestLinkTimeoutDropsSession();
  TestTransportErrorDropsSession();
  TestShutdownSendsZeroThenDisarm();
  TestResetOdomRefusedWhileArmed();
  TestTelemetryCaptured();
  TestTimeSyncEstimatesOffset();
  TestTimeSyncRejectsHighRtt();
  TestVersionMismatchDoesNotConnect();
  TestBadFrameDoesNotDisturbSession();
  TestWorksWithByteAtATimeReads();
  TestOneOutstandingRequestAtATime();
  TestMismatchedAckDoesNotClearPending();
  TestPendingRequestTimesOut();
  TestDisarmIsNotBlockedByPendingRequest();
  TestHelloReqCarriesVersion();
  return uart::test::Finish("sess");
}
