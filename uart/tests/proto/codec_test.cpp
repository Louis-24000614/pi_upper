/// @file
/// 消息层单元测试。对照固件 UART_MESSAGES.md 硬编码 payload 长度和关键字节位置。

#include "proto/codec.h"

#include <cmath>
#include <cstring>
#include <limits>

#include "check.h"

namespace {

using namespace uart;  // NOLINT(build/namespaces)

static_assert(kSizeArmRequest == 4, "ARM_REQUEST payload 应为 4 字节");
static_assert(kSizeCmdVel == 8, "CMD_VEL payload 应为 8 字节");
static_assert(kSizeMotionAction == 8, "MOTION_ACTION payload 应为 8 字节");
static_assert(kSizeAck == 2, "ACK payload 应为 2 字节");
static_assert(kSizeHelloReq == 1, "HELLO_REQ payload 应为 1 字节");
static_assert(kSizeHelloInfo == 7, "HELLO_INFO payload 应为 7 字节");
static_assert(kSizeOdomState == 21, "ODOM_STATE payload 应为 21 字节");
static_assert(kSizeImuState == 29, "IMU_STATE payload 应为 29 字节");
static_assert(kSizeImuDebug == 21, "IMU_DEBUG payload 应为 21 字节");
static_assert(kSizeSystemStatus == 7, "SYSTEM_STATUS payload 应为 7 字节");
static_assert(kSizeMotionResult == 11, "MOTION_RESULT payload 应为 11 字节");

void TestMsgTypeValues() {
  CHECK(static_cast<uint8_t>(MsgType::kHelloReq) == 0x01);
  CHECK(static_cast<uint8_t>(MsgType::kArmRequest) == 0x10);
  CHECK(static_cast<uint8_t>(MsgType::kDisarm) == 0x11);
  CHECK(static_cast<uint8_t>(MsgType::kCmdVel) == 0x12);
  CHECK(static_cast<uint8_t>(MsgType::kMotionAction) == 0x13);
  CHECK(static_cast<uint8_t>(MsgType::kAck) == 0x80);
  CHECK(static_cast<uint8_t>(MsgType::kHelloInfo) == 0x81);
  CHECK(static_cast<uint8_t>(MsgType::kOdomState) == 0x90);
  CHECK(static_cast<uint8_t>(MsgType::kImuState) == 0x91);
  CHECK(static_cast<uint8_t>(MsgType::kImuDebug) == 0x92);
  CHECK(static_cast<uint8_t>(MsgType::kSystemStatus) == 0x93);
  CHECK(static_cast<uint8_t>(MsgType::kMotionResult) == 0x94);
}

void TestCmdVelLayout() {
  CmdVel msg;
  msg.linear_x_mps = 1.0f;
  msg.angular_z_radps = -2.0f;

  uint8_t buf[kSizeCmdVel] = {};
  CHECK(EncodeCmdVel(msg, buf, sizeof(buf)) == kSizeCmdVel);
  CHECK(buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x80 && buf[3] == 0x3F);
  CHECK(buf[4] == 0x00 && buf[5] == 0x00 && buf[6] == 0x00 && buf[7] == 0xC0);

  CmdVel back;
  CHECK(DecodeCmdVel(buf, sizeof(buf), &back));
  CHECK(back.linear_x_mps == msg.linear_x_mps);
  CHECK(back.angular_z_radps == msg.angular_z_radps);
}

void TestZeroCmdVelPayload() {
  CmdVel msg;
  uint8_t buf[kSizeCmdVel] = {};
  CHECK(EncodeCmdVel(msg, buf, sizeof(buf)) == kSizeCmdVel);
  const uint8_t want[kSizeCmdVel] = {0, 0, 0, 0, 0, 0, 0, 0};
  CHECK(std::memcmp(buf, want, sizeof(want)) == 0);
}

void TestMotionActionLayout() {
  MotionAction msg;
  msg.action = static_cast<uint8_t>(MotionActionId::kTurnLeft);
  msg.quarter_turns = 1;
  msg.speed_mmps = 75;
  msg.distance_mm = 0;

  uint8_t buf[kSizeMotionAction] = {};
  CHECK(EncodeMotionAction(msg, buf, sizeof(buf)) == kSizeMotionAction);
  CHECK(buf[0] == 3 && buf[1] == 1);
  CHECK(buf[2] == 75 && buf[3] == 0);
  CHECK(buf[4] == 0 && buf[7] == 0);

  MotionAction back;
  CHECK(DecodeMotionAction(buf, sizeof(buf), &back));
  CHECK(back.action == 3);
  CHECK(back.quarter_turns == 1);
  CHECK(back.speed_mmps == 75);
  CHECK(back.distance_mm == 0);
}

void TestNonFiniteRejected() {
  uint8_t buf[kSizeCmdVel] = {};

  CmdVel nan_linear;
  nan_linear.linear_x_mps = std::numeric_limits<float>::quiet_NaN();
  CHECK(EncodeCmdVel(nan_linear, buf, sizeof(buf)) == 0);

  CmdVel inf_angular;
  inf_angular.angular_z_radps = std::numeric_limits<float>::infinity();
  CHECK(EncodeCmdVel(inf_angular, buf, sizeof(buf)) == 0);

  const uint8_t nan_payload[kSizeCmdVel] = {0x00, 0x00, 0xC0, 0x7F, 0, 0, 0, 0};
  CmdVel out;
  CHECK(!DecodeCmdVel(nan_payload, sizeof(nan_payload), &out));

  uint8_t odom[kSizeOdomState] = {};
  OdomState valid;
  valid.status_flags = kOdomValid;
  CHECK(EncodeOdomState(valid, odom, sizeof(odom)) == kSizeOdomState);
  OdomState decoded;
  CHECK(DecodeOdomState(odom, sizeof(odom), &decoded));
  odom[0] = 0x00;
  odom[1] = 0x00;
  odom[2] = 0x80;
  odom[3] = 0x7F;  // x_m = +Inf
  CHECK(!DecodeOdomState(odom, sizeof(odom), &decoded));
}

void TestStrictLength() {
  uint8_t buf[kSizeSystemStatus + 1] = {};
  SystemStatus status;
  status.remote_state = RemoteState::kArmed;
  CHECK(EncodeSystemStatus(status, buf, sizeof(buf)) == kSizeSystemStatus);

  SystemStatus out;
  CHECK(DecodeSystemStatus(buf, kSizeSystemStatus, &out));
  CHECK(!DecodeSystemStatus(buf, kSizeSystemStatus - 1, &out));
  CHECK(!DecodeSystemStatus(buf, kSizeSystemStatus + 1, &out));
  CHECK(EncodeSystemStatus(status, buf, kSizeSystemStatus - 1) == 0);
}

void TestHelloInfoRoundTrip() {
  HelloInfo msg;
  msg.protocol_version = 1;
  msg.capabilities = kCapMotor | kCapImu;
  msg.boot_id = 0xDEADBEEF;
  msg.remote_state = RemoteState::kReady;

  uint8_t buf[kSizeHelloInfo] = {};
  CHECK(EncodeHelloInfo(msg, buf, sizeof(buf)) == kSizeHelloInfo);
  CHECK(buf[0] == 1);
  CHECK(buf[1] == 0x05);
  CHECK(buf[2] == 0xEF && buf[5] == 0xDE);
  CHECK(buf[6] == 1);

  HelloInfo back;
  CHECK(DecodeHelloInfo(buf, sizeof(buf), &back));
  CHECK(back.boot_id == msg.boot_id);
  CHECK(back.capabilities == msg.capabilities);
  CHECK(back.remote_state == RemoteState::kReady);
  CHECK(back.protocol_version == 1);
}

void TestAckRoundTrip() {
  Ack msg;
  msg.request_type = static_cast<uint8_t>(MsgType::kArmRequest);
  msg.result = AckResult::kDeniedConfig;

  uint8_t buf[kSizeAck] = {};
  CHECK(EncodeAck(msg, buf, sizeof(buf)) == kSizeAck);
  CHECK(buf[0] == 0x10);
  CHECK(buf[1] == 2);

  Ack back;
  CHECK(DecodeAck(buf, sizeof(buf), &back));
  CHECK(back.request_type == msg.request_type);
  CHECK(back.result == AckResult::kDeniedConfig);
}

void TestOdomRoundTrip() {
  OdomState msg;
  msg.x_m = 1.25f;
  msg.y_m = -2.5f;
  msg.yaw_rad = 3.125f;
  msg.linear_mps = 0.25f;
  msg.angular_radps = -0.125f;
  msg.status_flags = kOdomValid | kOdomImuFused;

  uint8_t buf[kSizeOdomState] = {};
  CHECK(EncodeOdomState(msg, buf, sizeof(buf)) == kSizeOdomState);
  CHECK(buf[20] == 0x03);

  OdomState back;
  CHECK(DecodeOdomState(buf, sizeof(buf), &back));
  CHECK(back.x_m == msg.x_m);
  CHECK(back.y_m == msg.y_m);
  CHECK(back.yaw_rad == msg.yaw_rad);
  CHECK(back.linear_mps == msg.linear_mps);
  CHECK(back.angular_radps == msg.angular_radps);
  CHECK(back.status_flags == msg.status_flags);
}

void TestImuRoundTrip() {
  ImuState msg;
  msg.accel_x = 0.5f;
  msg.accel_y = -0.25f;
  msg.accel_z = 9.75f;
  msg.gyro_x = 0.125f;
  msg.gyro_y = -0.0625f;
  msg.gyro_z = 1.5f;
  msg.yaw = 2.0f;
  msg.status_flags = kImuValid | kImuCalibrated;

  uint8_t buf[kSizeImuState] = {};
  CHECK(EncodeImuState(msg, buf, sizeof(buf)) == kSizeImuState);
  CHECK(buf[28] == 0x05);

  ImuState back;
  CHECK(DecodeImuState(buf, sizeof(buf), &back));
  CHECK(back.accel_x == msg.accel_x && back.accel_z == msg.accel_z);
  CHECK(back.gyro_z == msg.gyro_z);
  CHECK(back.yaw == msg.yaw);
  CHECK(back.status_flags == msg.status_flags);

  ImuDebug dbg;
  dbg.velocity_x_mps = 0.5f;
  dbg.velocity_y_mps = -0.5f;
  dbg.position_x_m = 2.0f;
  dbg.position_y_m = -2.0f;
  dbg.yaw_rad = 1.0f;
  dbg.status_flags = kImuValid | kImuNotForNavigation;
  uint8_t dbg_buf[kSizeImuDebug] = {};
  CHECK(EncodeImuDebug(dbg, dbg_buf, sizeof(dbg_buf)) == kSizeImuDebug);
  CHECK(dbg_buf[20] == 0x09);
  ImuDebug dbg_back;
  CHECK(DecodeImuDebug(dbg_buf, sizeof(dbg_buf), &dbg_back));
  CHECK(dbg_back.position_x_m == dbg.position_x_m);
  CHECK((dbg_back.status_flags & kImuNotForNavigation) != 0);
}

void TestSystemStatusRoundTrip() {
  SystemStatus msg;
  msg.remote_state = RemoteState::kArmed;
  msg.command_saturated = 1;
  msg.fault_code = 13;
  msg.communication_errors = 0x01020304;

  uint8_t buf[kSizeSystemStatus] = {};
  CHECK(EncodeSystemStatus(msg, buf, sizeof(buf)) == kSizeSystemStatus);
  CHECK(buf[0] == 2);
  CHECK(buf[1] == 1);
  CHECK(buf[2] == 13);
  CHECK(buf[3] == 0x04 && buf[6] == 0x01);

  SystemStatus back;
  CHECK(DecodeSystemStatus(buf, sizeof(buf), &back));
  CHECK(back.remote_state == RemoteState::kArmed);
  CHECK(back.fault_code == 13);
  CHECK(back.communication_errors == 0x01020304);
}

void TestMotionResultRoundTrip() {
  MotionResult msg;
  msg.action = 3;
  msg.quarter_turns = 1;
  msg.result = MotionResultCode::kCompleted;
  msg.final_yaw_rad = 1.5703125f;
  msg.target_yaw_rad = 1.5703125f;

  uint8_t buf[kSizeMotionResult] = {};
  CHECK(EncodeMotionResult(msg, buf, sizeof(buf)) == kSizeMotionResult);
  CHECK(buf[0] == 3 && buf[1] == 1 && buf[2] == 0);

  MotionResult back;
  CHECK(DecodeMotionResult(buf, sizeof(buf), &back));
  CHECK(back.action == 3);
  CHECK(back.quarter_turns == 1);
  CHECK(back.result == MotionResultCode::kCompleted);
  CHECK(back.final_yaw_rad == msg.final_yaw_rad);
  CHECK(back.target_yaw_rad == msg.target_yaw_rad);
}

void TestArmRequestRoundTrip() {
  ArmRequest msg;
  msg.boot_id = 0x01020304;
  uint8_t buf[kSizeArmRequest] = {};
  CHECK(EncodeArmRequest(msg, buf, sizeof(buf)) == kSizeArmRequest);
  CHECK(buf[0] == 0x04 && buf[3] == 0x01);
  ArmRequest back;
  CHECK(DecodeArmRequest(buf, sizeof(buf), &back));
  CHECK(back.boot_id == msg.boot_id);
}

void TestUnknownEnumsAreConservative() {
  uint8_t status[kSizeSystemStatus] = {};
  status[0] = 99;
  SystemStatus out;
  CHECK(DecodeSystemStatus(status, sizeof(status), &out));
  CHECK(out.remote_state == RemoteState::kDisabled);

  uint8_t ack[kSizeAck] = {};
  ack[1] = 200;
  Ack ack_out;
  CHECK(DecodeAck(ack, sizeof(ack), &ack_out));
  CHECK(ack_out.result == AckResult::kUnsupported);
}

}  // namespace

int main() {
  TestMsgTypeValues();
  TestCmdVelLayout();
  TestZeroCmdVelPayload();
  TestMotionActionLayout();
  TestNonFiniteRejected();
  TestStrictLength();
  TestHelloInfoRoundTrip();
  TestAckRoundTrip();
  TestOdomRoundTrip();
  TestImuRoundTrip();
  TestSystemStatusRoundTrip();
  TestMotionResultRoundTrip();
  TestArmRequestRoundTrip();
  TestUnknownEnumsAreConservative();
  return uart::test::Finish("codec");
}
