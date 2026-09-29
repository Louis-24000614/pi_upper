/// @file
/// 会话层测试。假传输加假时钟，不碰硬件。

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

struct Fixture {
  FakePort port;
  FakeClock clock;
  FakeMcu mcu;
  Session session{port, clock};

  void Tick(uint64_t ms) {
    clock.AdvanceMs(ms);
    session.Poll();
    mcu.Drain(port);
  }

  void Connect(uint32_t boot_id = kBootId, RemoteState state = RemoteState::kReady) {
    session.Start();
    Tick(1);
    HelloInfo info;
    info.protocol_version = kProtocolVersion;
    info.boot_id = boot_id;
    info.remote_state = state;
    info.capabilities = kCapMotor | kCapEncoder | kCapImu;
    mcu.SendHelloInfo(port, info);
    Tick(1);
  }

  void Arm() {
    CHECK(session.RequestArm());
    Tick(1);
    Ack ack;
    ack.request_type = static_cast<uint8_t>(MsgType::kArmRequest);
    ack.result = AckResult::kOk;
    mcu.SendAck(port, ack);
    SystemStatus status;
    status.remote_state = RemoteState::kArmed;
    mcu.SendSystemStatus(port, status);
    Tick(1);
  }
};

void TestHelloRetryUntilAnswered() {
  Fixture f;
  f.session.Start();
  CHECK(f.session.link_state() == LinkState::kConnecting);

  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kHelloReq) == 1);

  f.Tick(500);
  f.Tick(500);
  CHECK(f.mcu.CountOf(MsgType::kHelloReq) == 3);
  CHECK(f.session.link_state() == LinkState::kConnecting);

  HelloInfo info;
  info.protocol_version = kProtocolVersion;
  info.boot_id = kBootId;
  info.remote_state = RemoteState::kReady;
  f.mcu.SendHelloInfo(f.port, info);
  f.Tick(1);

  CHECK(f.session.link_state() == LinkState::kConnected);
  CHECK(f.session.boot_id() == kBootId);
  CHECK(!f.session.config_valid());
  CHECK(f.session.remote_state() == RemoteState::kReady);
  CHECK(f.mcu.stats().crc_errors == 0);
}

void TestArmDeniedConfigDoesNotEnableCommands() {
  Fixture f;
  f.Connect();
  CHECK(f.session.RequestArm());
  f.Tick(1);

  Ack ack;
  ack.request_type = static_cast<uint8_t>(MsgType::kArmRequest);
  ack.result = AckResult::kDeniedConfig;
  f.mcu.SendAck(f.port, ack);
  f.Tick(1);

  CHECK(!f.session.config_valid());
  CHECK(!f.session.command_enabled());
  f.session.SetVelocity(0.5f, 0.0f);
  f.mcu.ClearReceived();
  f.Tick(50);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);
}

void TestArmRefusedBeforeConnect() {
  Fixture f;
  f.session.Start();
  CHECK(!f.session.RequestArm());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kArmRequest) == 0);
}

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

void TestNoCommandsBeforeArmed() {
  Fixture f;
  f.Connect();
  CHECK(f.session.SetVelocity(0.5f, 0.1f));
  f.Tick(200);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);
  CHECK(!f.session.command_enabled());
}

void TestCommandRateAfterArm() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.command_enabled());
  CHECK(f.session.remote_state() == RemoteState::kArmed);

  f.mcu.ClearReceived();
  CHECK(f.session.SetVelocity(0.5f, -0.25f));

  for (int i = 0; i < 100; ++i) {
    f.Tick(1);
  }
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 5);

  const FakeMcu::Received* frame = f.mcu.Last(MsgType::kCmdVel);
  CHECK(frame != nullptr);
  if (frame != nullptr) {
    CmdVel cmd;
    CHECK(DecodeCmdVel(frame->payload.data(), frame->payload.size(), &cmd));
    CHECK(cmd.linear_x_mps == 0.5f);
    CHECK(cmd.angular_z_radps == -0.25f);
  }
}

void TestStaleCommandBecomesZero() {
  Fixture f;
  f.Connect();
  f.Arm();
  f.session.SetVelocity(1.0f, 0.5f);
  f.mcu.ClearReceived();

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

void TestNonFiniteVelocityRejected() {
  Fixture f;
  f.Connect();
  f.Arm();

  CHECK(!f.session.SetVelocity(std::numeric_limits<float>::quiet_NaN(), 0.0f));
  CHECK(!f.session.SetVelocity(0.0f, std::numeric_limits<float>::infinity()));

  f.mcu.ClearReceived();
  CHECK(f.session.SetVelocity(0.0f, 0.0f));
  f.Tick(25);
  const FakeMcu::Received* frame = f.mcu.Last(MsgType::kCmdVel);
  CHECK(frame != nullptr);
}

void TestBootIdChangeDropsArm() {
  Fixture f;
  f.Connect();
  f.Arm();
  f.session.SetVelocity(0.5f, 0.0f);
  f.Tick(25);
  CHECK(f.session.command_enabled());

  HelloInfo info;
  info.protocol_version = kProtocolVersion;
  info.boot_id = 0x99999999;
  info.remote_state = RemoteState::kReady;
  f.mcu.SendHelloInfo(f.port, info);
  f.Tick(1);

  CHECK(f.session.boot_id() == 0x99999999);
  CHECK(!f.session.command_enabled());
  CHECK(f.session.diagnostics().boot_id_changes == 1);

  f.mcu.ClearReceived();
  f.Tick(100);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);
}

void TestFaultStopsCommands() {
  Fixture f;
  f.Connect();
  f.Arm();

  SystemStatus status;
  status.remote_state = RemoteState::kFault;
  status.fault_code = 13;
  f.mcu.SendSystemStatus(f.port, status);
  f.Tick(1);

  CHECK(!f.session.command_enabled());
  CHECK(f.session.remote_state() == RemoteState::kFault);

  f.mcu.ClearReceived();
  f.Tick(100);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);
}

void TestLinkTimeoutDropsSession() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.link_state() == LinkState::kConnected);

  f.mcu.ClearReceived();
  for (int i = 0; i < 1200; ++i) {
    f.Tick(1);
  }
  CHECK(f.session.link_state() == LinkState::kConnecting);
  CHECK(!f.session.command_enabled());
  CHECK(f.session.diagnostics().link_drops >= 1);
  CHECK(f.mcu.CountOf(MsgType::kHelloReq) >= 1);
}

void TestTransportErrorDropsSession() {
  Fixture f;
  f.Connect();
  f.Arm();

  f.port.set_fail_read(true);
  f.Tick(1);
  CHECK(f.session.link_state() == LinkState::kClosed);
  CHECK(!f.session.command_enabled());
  CHECK(!f.port.IsOpen());
}

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
  CHECK(!f.mcu.received().empty());
  if (!f.mcu.received().empty()) {
    CHECK(f.mcu.received().back().type == static_cast<uint8_t>(MsgType::kDisarm));
  }
}

void TestTelemetryCaptured() {
  Fixture f;
  f.Connect();

  OdomState odom;
  odom.x_m = 1.5f;
  odom.yaw_rad = 0.25f;
  odom.status_flags = kOdomValid | kOdomImuFused;
  f.mcu.SendOdom(f.port, odom);
  f.Tick(1);

  CHECK(f.session.telemetry().has_odom);
  CHECK(f.session.telemetry().odom.x_m == 1.5f);
  CHECK((f.session.telemetry().odom.status_flags & kOdomValid) != 0);
  CHECK(f.session.telemetry().odom_us > 0);
}

void TestRfidCardCaptured() {
  Fixture f;
  f.Connect();

  RfidCard card;
  card.present = 1;
  card.card_number = 3;
  card.generation = 1;
  f.mcu.SendRfidCard(f.port, card);
  f.Tick(1);

  CHECK(f.session.telemetry().has_rfid);
  CHECK(f.session.telemetry().rfid.present == 1);
  CHECK(f.session.telemetry().rfid.card_number == 3);
  CHECK(f.session.telemetry().rfid.generation == 1);
  CHECK(f.session.telemetry().rfid_us > 0);

  uint8_t bad_payload[2] = {1, 3};
  uint8_t frame[16] = {};
  const size_t frame_len = EncodeFrame(static_cast<uint8_t>(MsgType::kRfidCard), bad_payload,
                                       sizeof(bad_payload), frame, sizeof(frame));
  f.mcu.SendRaw(f.port, std::vector<uint8_t>(frame, frame + frame_len));
  f.Tick(1);

  CHECK(f.session.telemetry().rfid.present == 1);
  CHECK(f.session.telemetry().rfid.card_number == 3);
  CHECK(f.session.telemetry().rfid.generation == 1);
}

void TestVersionMismatchDoesNotConnect() {
  Fixture f;
  f.session.Start();
  f.Tick(1);

  HelloInfo info;
  info.boot_id = kBootId;
  info.remote_state = RemoteState::kReady;
  f.mcu.SendHelloInfoWithVersion(f.port, info, kProtocolVersion + 1);
  f.Tick(1);

  CHECK(f.session.link_state() == LinkState::kConnecting);
  CHECK(f.session.boot_id() == 0);
  CHECK(!f.session.RequestArm());
  CHECK(f.session.peer_protocol_version() == kProtocolVersion + 1);
  CHECK(f.session.diagnostics().version_mismatches == 1);
}

void TestBadFrameDoesNotDisturbSession() {
  Fixture f;
  f.Connect();
  f.Arm();

  const std::vector<uint8_t> bad = {0x55, 0xAA, 0x12, 0x08, 0x01, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x83};
  f.mcu.SendRaw(f.port, bad);
  f.Tick(1);

  CHECK(f.session.command_enabled());
  CHECK(f.session.link_state() == LinkState::kConnected);
  CHECK(f.session.diagnostics().rx.crc_errors == 1);
}

void TestWorksWithByteAtATimeReads() {
  Fixture f;
  f.port.set_read_chunk(1);
  f.Connect();
  CHECK(f.session.link_state() == LinkState::kConnected);
  f.Arm();
  CHECK(f.session.command_enabled());
}

void TestOneOutstandingRequestAtATime() {
  Fixture f;
  f.Connect();
  CHECK(f.session.RequestArm());
  CHECK(f.session.request_pending());
  f.Tick(1);
  f.mcu.ClearReceived();
  CHECK(!f.session.RequestArm());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kArmRequest) == 0);
  CHECK(f.session.diagnostics().requests_refused_busy == 1);

  Ack ack;
  ack.request_type = static_cast<uint8_t>(MsgType::kArmRequest);
  ack.result = AckResult::kOk;
  f.mcu.SendAck(f.port, ack);
  SystemStatus status;
  status.remote_state = RemoteState::kArmed;
  f.mcu.SendSystemStatus(f.port, status);
  f.Tick(1);
  CHECK(!f.session.request_pending());
}

void TestMismatchedAckDoesNotClearPending() {
  Fixture f;
  f.Connect();
  CHECK(f.session.RequestArm());
  Ack ack;
  ack.request_type = static_cast<uint8_t>(MsgType::kCmdVel);
  ack.result = AckResult::kBadPayload;
  f.mcu.SendAck(f.port, ack);
  f.Tick(1);

  CHECK(f.session.request_pending());
  CHECK(f.session.telemetry().has_ack);
}

void TestPendingRequestTimesOut() {
  Fixture f;
  f.Connect();
  CHECK(f.session.RequestArm());
  f.Tick(1);
  f.mcu.ClearReceived();

  f.Tick(f.session.config().ack_timeout_ms + 1);
  CHECK(!f.session.request_pending());
  CHECK(f.session.diagnostics().ack_timeouts == 1);
  CHECK(f.mcu.CountOf(MsgType::kArmRequest) == 0);
}

void TestDisarmIsNotBlockedByPendingRequest() {
  Fixture f;
  f.Connect();
  CHECK(f.session.RequestArm());
  CHECK(f.session.request_pending());
  f.mcu.ClearReceived();

  CHECK(f.session.RequestDisarm());
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kDisarm) == 1);
}

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

void TestFiniteActionBlocksVelocityUntilResult() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.RequestMotionAction(static_cast<uint8_t>(MotionActionId::kTurnLeft), 1));
  CHECK(f.session.motion_mode() == MotionMode::kAction);
  CHECK(f.session.awaiting_motion_result());
  CHECK(!f.session.SetVelocity(0.3f, 0.0f));

  f.mcu.ClearReceived();
  f.Tick(50);
  CHECK(f.mcu.CountOf(MsgType::kCmdVel) == 0);

  MotionResult result;
  result.action = 3;
  result.quarter_turns = 1;
  result.result = MotionResultCode::kCompleted;
  f.mcu.SendMotionResult(f.port, result);
  f.Tick(1);
  CHECK(!f.session.awaiting_motion_result());
  CHECK(f.session.motion_mode() == MotionMode::kIdle);
  CHECK(f.session.SetVelocity(0.3f, 0.0f));
}

void TestStopClearsActionAndAllowsVelocity() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.RequestMotionAction(static_cast<uint8_t>(MotionActionId::kTurnRight), 1));
  CHECK(f.session.RequestMotionAction(static_cast<uint8_t>(MotionActionId::kStop)));
  CHECK(!f.session.awaiting_motion_result());
  CHECK(f.session.motion_mode() == MotionMode::kIdle);
  CHECK(f.session.SetVelocity(0.2f, 0.0f));
}

void TestSecondFiniteActionRefusedWhileWaiting() {
  Fixture f;
  f.Connect();
  f.Arm();
  CHECK(f.session.RequestMotionAction(static_cast<uint8_t>(MotionActionId::kTurnLeft), 1));
  f.Tick(1);
  f.mcu.ClearReceived();
  CHECK(!f.session.RequestMotionAction(static_cast<uint8_t>(MotionActionId::kTurnRight), 1));
  f.Tick(1);
  CHECK(f.mcu.CountOf(MsgType::kMotionAction) == 0);
}

void TestRequestSpeechSendsAudioIdAndWaitsAck() {
  Fixture f;
  f.Connect();
  CHECK(f.session.RequestSpeech(8));
  f.Tick(1);

  const FakeMcu::Received* frame = f.mcu.Last(MsgType::kSpeakAudio);
  CHECK(frame != nullptr);
  if (frame != nullptr) {
    SpeakAudio audio;
    CHECK(DecodeSpeakAudio(frame->payload.data(), frame->payload.size(), &audio));
    CHECK(audio.audio_id == 8);
  }
  CHECK(f.session.request_pending());

  Ack ack;
  ack.request_type = static_cast<uint8_t>(MsgType::kSpeakAudio);
  ack.result = AckResult::kOk;
  f.mcu.SendAck(f.port, ack);
  f.Tick(1);
  CHECK(!f.session.request_pending());
}

void TestRequestSpeechRejectsInvalidIdAndDisconnectedLink() {
  Fixture f;
  f.session.Start();
  CHECK(!f.session.RequestSpeech(1));
  f.Connect();
  CHECK(!f.session.RequestSpeech(0));
  CHECK(!f.session.RequestSpeech(13));
}

}  // namespace

int main() {
  TestHelloRetryUntilAnswered();
  TestArmDeniedConfigDoesNotEnableCommands();
  TestArmRefusedBeforeConnect();
  TestArmRequestCarriesBootId();
  TestNoCommandsBeforeArmed();
  TestCommandRateAfterArm();
  TestStaleCommandBecomesZero();
  TestNonFiniteVelocityRejected();
  TestBootIdChangeDropsArm();
  TestFaultStopsCommands();
  TestLinkTimeoutDropsSession();
  TestTransportErrorDropsSession();
  TestShutdownSendsZeroThenDisarm();
  TestTelemetryCaptured();
  TestRfidCardCaptured();
  TestVersionMismatchDoesNotConnect();
  TestBadFrameDoesNotDisturbSession();
  TestWorksWithByteAtATimeReads();
  TestOneOutstandingRequestAtATime();
  TestMismatchedAckDoesNotClearPending();
  TestPendingRequestTimesOut();
  TestDisarmIsNotBlockedByPendingRequest();
  TestHelloReqCarriesVersion();
  TestFiniteActionBlocksVelocityUntilResult();
  TestStopClearsActionAndAllowsVelocity();
  TestSecondFiniteActionRefusedWhileWaiting();
  TestRequestSpeechSendsAudioIdAndWaitsAck();
  TestRequestSpeechRejectsInvalidIdAndDisconnectedLink();
  return uart::test::Finish("sess");
}
